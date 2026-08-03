// Copyright (c) LMCAD. Licensed under the MIT License.

//! **Coplanar face re-coalescing** — FRICTION #20's remedy, opt-in.
//!
//! Booleans and re-tessellation can leave one geometric plane represented as
//! several adjacent `Face`s (a flush-stack union carries each side wall as
//! two coplanar rectangles sharing the seam edge). Downstream that costs
//! real capability: witness-addressed fillets hit `EdgeAmbiguous` on the
//! fragment seams, STEP exports carry redundant faces, and face counts blow
//! up. [`coalesce_coplanar`] merges groups of adjacent faces that lie on the
//! SAME plane (normal-parallel within 1e-9, offset within 1e-7) into single
//! multi-loop faces and rebuilds the solid.
//!
//! Scope, stated honestly: PLANE faces only (the fragmentation complaint is
//! about planes; curved bands carry their own seam semantics) and groups merge
//! only across SHARED EDGES (two coplanar islands separated by a slot stay
//! two faces — correctly).
//!
//! **Provenance survives the rebuild** (the FRICTION #20 residual, LIFTED
//! 2026-07-30): an unmerged face keeps its [`crate::topo::FaceName`] exactly, a
//! merged face inherits the lexicographically-least constituent name (policy
//! documented on [`crate::topo::FaceName`]), and analytic edge curves whose
//! endpoints both survive are re-attached — so the pass may run MID-CHAIN and
//! witness-addressed features ([`crate::fillet_edge_near`] and friends)
//! re-resolve afterwards. Remaining caveat, stated: the names of fragments
//! fully consumed by a merge — and the non-least names when fragments of
//! several source faces merge into one (a flush-coplanar union seam) — no
//! longer resolve; they name faces that no longer exist, which is correct.
//! The result must validate and conserve volume exactly; callers should gate
//! both (the unit test does).

use std::collections::HashMap;

use kernel_core::math::DVec3;

use crate::geom::Surface;
use crate::topo::{FaceId, FaceLoops, FaceName, Solid};

/// Merge adjacent coplanar planar faces into single multi-loop faces.
/// Returns the rebuilt solid, or the input clone unchanged when nothing
/// merges. See the module doc for scope and caveats.
pub fn coalesce_coplanar(s: &Solid) -> Solid {
	let nf = s.face_count();
	// ---- group faces: same plane key, connected through shared edges --------
	let plane_key = |f: FaceId| -> Option<(i64, i64, i64, i64)> {
		match s.face(f).surface {
			Surface::Plane { origin, normal } => {
				let n = normal.normalize_or_zero();
				let flip = if n.z < 0.0 || (n.z == 0.0 && n.y < 0.0) || (n.z == 0.0 && n.y == 0.0 && n.x < 0.0) {
					-1.0
				} else {
					1.0
				};
				let n = n * flip;
				let q = |x: f64| (x / crate::tol::SURF_KEY_QUANTUM).round() as i64;
				Some((q(n.x), q(n.y), q(n.z), q(n.dot(origin) * flip)))
			}
			_ => None,
		}
	};
	// union-find over faces
	let mut parent: Vec<u32> = (0..nf as u32).collect();
	fn find(p: &mut [u32], mut i: u32) -> u32 {
		while p[i as usize] != i {
			p[i as usize] = p[p[i as usize] as usize];
			i = p[i as usize];
		}
		i
	}
	// walk edges: join the two incident faces when both are planes on one key
	for e in s.edges() {
		let he = s.half_edge(s.edge(e).half_edge);
		let f1 = he.face;
		let Some(twin) = he.twin else { continue };
		let f2 = s.half_edge(twin).face;
		if f1 == f2 {
			continue;
		}
		if let (Some(k1), Some(k2)) = (plane_key(f1), plane_key(f2)) {
			if k1 == k2 {
				let (r1, r2) = (find(&mut parent, f1.0), find(&mut parent, f2.0));
				parent[r1 as usize] = r2;
			}
		}
	}
	let mut groups: HashMap<u32, Vec<FaceId>> = HashMap::new();
	for f in s.faces() {
		groups.entry(find(&mut parent, f.0)).or_default().push(f);
	}
	if groups.values().all(|g| g.len() == 1) {
		return s.clone();
	}

	// ---- emit every face; merged groups get re-chained boundary loops -------
	// Provenance rides along: untouched faces keep their FaceName exactly, a
	// merged face inherits the lexicographically-least constituent name (the
	// policy documented on `FaceName`).
	let positions: Vec<DVec3> = (0..s.vertex_count() as u32).map(|i| s.position(crate::topo::VertexId(i))).collect();
	let mut faces_out: Vec<FaceLoops> = Vec::new();
	let mut names_out: Vec<Option<FaceName>> = Vec::new();
	// deterministic order: by smallest face id in the group
	let mut group_list: Vec<(u32, Vec<FaceId>)> = groups.into_iter().collect();
	for (_, g) in group_list.iter_mut() {
		g.sort_by_key(|f| f.0);
	}
	group_list.sort_by_key(|(_, g)| g[0].0);

	for (_, group) in &group_list {
		if group.len() == 1 {
			// untouched face: re-emit its existing loops verbatim
			let f = group[0];
			let face = s.face(f);
			let mut loops: Vec<Vec<u32>> = Vec::new();
			for lp in std::iter::once(face.outer).chain(face.inner.iter().copied()) {
				loops.push(s.loop_half_edges(lp).iter().map(|&he| s.half_edge(he).origin.0).collect());
			}
			faces_out.push(FaceLoops { loops, surface: face.surface });
			names_out.push(s.face_name(f));
			continue;
		}
		// merged group: boundary half-edges are those whose twin's face is
		// OUTSIDE the group. Chain them BY HALF-EDGE (a vertex can carry two
		// boundary chains at fragment T-corners — vertex-keyed chaining
		// overwrote one and emitted broken loops): follow `next`; when `next`
		// is interior to the group, hop twin.next until re-emerging on the
		// boundary — the standard region-boundary walk.
		let in_group = |f: FaceId| group.binary_search_by_key(&f.0, |x| x.0).is_ok();
		let is_boundary = |he_id: crate::topo::HalfEdgeId| -> bool {
			match s.half_edge(he_id).twin {
				Some(t) => !in_group(s.half_edge(t).face),
				None => true,
			}
		};
		let mut boundary_hes: std::collections::BTreeSet<u32> = std::collections::BTreeSet::new();
		for &f in group {
			let face = s.face(f);
			for lp in std::iter::once(face.outer).chain(face.inner.iter().copied()) {
				for &he_id in &s.loop_half_edges(lp) {
					if is_boundary(he_id) {
						boundary_hes.insert(he_id.0);
					}
				}
			}
		}
		let next_boundary = |he_id: crate::topo::HalfEdgeId| -> Option<crate::topo::HalfEdgeId> {
			let mut n = s.half_edge(he_id).next;
			for _ in 0..s.half_edge_count() {
				if is_boundary(n) {
					return Some(n);
				}
				// interior: hop across to the neighbouring group face
				n = s.half_edge(s.half_edge(n).twin?).next;
			}
			None // defensive: walk did not close
		};
		let mut loops: Vec<Vec<u32>> = Vec::new();
		while let Some(&start_raw) = boundary_hes.iter().next() {
			let start = crate::topo::HalfEdgeId(start_raw);
			let mut lp: Vec<u32> = Vec::new();
			let mut cur = start;
			let mut ok_loop = true;
			loop {
				boundary_hes.remove(&cur.0);
				lp.push(s.half_edge(cur).origin.0);
				match next_boundary(cur) {
					Some(nx) if nx == start => break,
					Some(nx) => cur = nx,
					None => {
						ok_loop = false;
						break;
					}
				}
			}
			if ok_loop && lp.len() >= 3 {
				loops.push(lp);
			}
		}
		if loops.is_empty() {
			// defensive fallback: re-emit the group unmerged
			for &f in group {
				let face = s.face(f);
				let mut ls: Vec<Vec<u32>> = Vec::new();
				for lp in std::iter::once(face.outer).chain(face.inner.iter().copied()) {
					ls.push(s.loop_half_edges(lp).iter().map(|&he| s.half_edge(he).origin.0).collect());
				}
				faces_out.push(FaceLoops { loops: ls, surface: face.surface });
				names_out.push(s.face_name(f));
			}
			continue;
		}
		// outer loop = largest projected area; the rest are holes
		let surface = s.face(group[0]).surface;
		let n = match surface {
			Surface::Plane { normal, .. } => normal,
			_ => unreachable!("groups only form over planes"),
		};
		let area = |lp: &Vec<u32>| -> f64 {
			let mut a = DVec3::ZERO;
			for i in 0..lp.len() {
				let p = positions[lp[i] as usize];
				let q = positions[lp[(i + 1) % lp.len()] as usize];
				a += p.cross(q);
			}
			(a.dot(n) * 0.5).abs()
		};
		let outer_ix = (0..loops.len())
			.max_by(|&i, &j| area(&loops[i]).total_cmp(&area(&loops[j])))
			.unwrap();
		loops.swap(0, outer_ix);
		faces_out.push(FaceLoops { loops, surface });
		names_out.push(group.iter().map(|&f| s.face_name(f)).collect::<Option<Vec<_>>>().and_then(|ns| ns.into_iter().min()));
	}

	// Compact to REFERENCED vertices only: merging orphans the interior
	// fragment-junction vertices, and phantom array entries inflate V — the
	// rebuilt solid read χ = 9 / genus = −3 (closed and manifold!) until the
	// unused positions were dropped.
	let mut remap: Vec<u32> = vec![u32::MAX; positions.len()];
	let mut compact: Vec<DVec3> = Vec::new();
	for fl in &mut faces_out {
		for lp in &mut fl.loops {
			for ix in lp.iter_mut() {
				if remap[*ix as usize] == u32::MAX {
					remap[*ix as usize] = compact.len() as u32;
					compact.push(positions[*ix as usize]);
				}
				*ix = remap[*ix as usize];
			}
		}
	}
	let mut out = Solid::from_faces_multiloop(compact, faces_out);
	// Provenance carry (all-or-nothing, heal's rule): set only when every
	// emitted face resolved a name — an unnamed input stays unnamed.
	if let Some(names) = names_out.into_iter().collect::<Option<Vec<FaceName>>>() {
		if names.len() == out.face_count() {
			out.set_provenance(names);
		}
	}
	// Analytic edge curves survive when both endpoints survive (heal's rule).
	for e in s.edges() {
		if let Some(c) = s.edge_curve(e) {
			let he = s.half_edge(s.edge(e).half_edge);
			let a = remap[he.origin.0 as usize];
			let b = remap[s.half_edge(he.next).origin.0 as usize];
			if a != u32::MAX && b != u32::MAX && a != b {
				out.set_edge_curve(crate::topo::VertexId(a), crate::topo::VertexId(b), c);
			}
		}
	}
	out
}
