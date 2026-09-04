// Copyright (c) LMCAD. Licensed under the MIT License.

//! Step 5 of the boolean pipeline: weld the kept fragments, cancel coincident
//! facets, recover maximal planar faces, heal T-junctions, snap the cut seams
//! onto the true surface-surface intersection, and build the closed [`Solid`].

use std::collections::{HashMap, HashSet};

use kernel_core::math::DVec3;

use crate::geom::Surface;
use crate::topo::{FaceInput, Solid};

use crate::tol::{TJUNCTION_EPS, WELD_EPS};

use super::faces::{boundary_covers_region, cancel_coincident, recover_faces, resolve_t_junctions, RawTri};
use super::snap::snap_seam_vertices;
use super::triangulate::chain_redundant_in_rings;
use super::Tri;

/// Merge coplanar adjacent kept triangles back into maximal planar faces and
/// build a closed [`Solid`] via [`Solid::from_faces`]. Vertices are welded to
/// [`WELD_EPS`] so twin half-edges pair up.
pub(super) fn stitch(kept: &[Tri]) -> Solid {
	if kept.is_empty() {
		return Solid::default();
	}

	// Weld vertices to a shared index space.
	let mut verts: Vec<DVec3> = Vec::new();
	let mut grid: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::new();
	let inv = 1.0 / WELD_EPS;
	let key = |p: DVec3| ((p.x * inv).round() as i64, (p.y * inv).round() as i64, (p.z * inv).round() as i64);
	let weld = |p: DVec3, verts: &mut Vec<DVec3>, grid: &mut HashMap<(i64, i64, i64), Vec<u32>>| -> u32 {
		let k = key(p);
		for dz in -1..=1 {
			for dy in -1..=1 {
				for dx in -1..=1 {
					if let Some(ids) = grid.get(&(k.0 + dx, k.1 + dy, k.2 + dz)) {
						for &id in ids {
							if (verts[id as usize] - p).length() <= WELD_EPS {
								return id;
							}
						}
					}
				}
			}
		}
		let id = verts.len() as u32;
		verts.push(p);
		grid.entry(k).or_default().push(id);
		id
	};

	// Weld every triangle's vertices first (ids only; filters run after the
	// duplicate merge below so they see final positions).
	let widx: Vec<[u32; 3]> = kept
		.iter()
		.map(|t| [weld(t.v[0], &mut verts, &mut grid), weld(t.v[1], &mut verts, &mut grid), weld(t.v[2], &mut verts, &mut grid)])
		.collect();

	// Merge sub-heal-scale vertex DUPLICATES the greedy weld left distinct. The
	// weld's first-fit ball is [`WELD_EPS`]; the stitch's own resolution is
	// [`TJUNCTION_EPS`] (the sliver filter drops thinner triangles, the healer
	// moves geometry by up to it) — so two distinct vertices closer than
	// TJUNCTION_EPS are the SAME point at stitch resolution, yet they are
	// unstitchable: `resolve_t_junctions` cannot insert either into an edge
	// ending at the other (the projection parameter lands within EPS of the
	// endpoint) and the welder already ruled. Fuzz seed 83894724550888 measured
	// the class: two fragments of one operand face disagreed about a seam corner
	// by 1.051e-7 — 5% OVER the weld ball — leaving an unpairable zero-area slit
	// (`8→74, 74→73, 73→8`). Clusters are found on a TJUNCTION_EPS-sized grid
	// and united by min id (union-find, ids ascending — deterministic); the
	// representative keeps its own coordinates, so every survivor is bit-stable
	// and a merged PAIR moves one vertex ≤ TJUNCTION_EPS. Transitive chains can
	// in principle span further (the weld only guarantees pairwise > WELD_EPS,
	// so a string of ~2e-7 steps would unify) — but such a string is already
	// sub-resolution geometry to the sliver filter and healer, and the honest
	// arbiter is the corpus: N=200/2000/10 000 all at 100 % with this merge
	// (ROBUSTNESS.md 2026-07-30), exact-volume and 1e-9 seam gates unchanged.
	let remap: Vec<u32> = {
		let cell = 1.0 / TJUNCTION_EPS;
		let ckey = |p: DVec3| ((p.x * cell).round() as i64, (p.y * cell).round() as i64, (p.z * cell).round() as i64);
		let mut cgrid: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::new();
		for (i, p) in verts.iter().enumerate() {
			cgrid.entry(ckey(*p)).or_default().push(i as u32);
		}
		let mut parent: Vec<u32> = (0..verts.len() as u32).collect();
		fn find(p: &mut [u32], mut i: u32) -> u32 {
			while p[i as usize] != i {
				p[i as usize] = p[p[i as usize] as usize];
				i = p[i as usize];
			}
			i
		}
		let mut candidates: Vec<u32> = Vec::new();
		for i in 0..verts.len() as u32 {
			let p = verts[i as usize];
			let k = ckey(p);
			candidates.clear();
			for dz in -1..=1_i64 {
				for dy in -1..=1_i64 {
					for dx in -1..=1_i64 {
						if let Some(ids) = cgrid.get(&(k.0 + dx, k.1 + dy, k.2 + dz)) {
							candidates.extend(ids.iter().copied().filter(|&j| j > i));
						}
					}
				}
			}
			candidates.sort_unstable();
			for &j in &candidates {
				if (verts[j as usize] - p).length() <= TJUNCTION_EPS {
					let (ri, rj) = (find(&mut parent, i), find(&mut parent, j));
					if ri != rj {
						// Union by MIN root so the cluster representative is the
						// smallest id — first-insertion order, like the weld itself.
						let (lo, hi) = if ri < rj { (ri, rj) } else { (rj, ri) };
						parent[hi as usize] = lo;
					}
				}
			}
		}
		(0..verts.len() as u32).map(|i| find(&mut parent, i)).collect()
	};

	// Indexed triangles, dropping any that collapse on welding (or on the
	// duplicate merge). Each carries the persistent name and analytic surface of
	// the operand face it came from.
	let mut raw: Vec<RawTri> = Vec::with_capacity(kept.len());
	for (t, ids) in kept.iter().zip(&widx) {
		let a = remap[ids[0] as usize];
		let b = remap[ids[1] as usize];
		let c = remap[ids[2] as usize];
		if a == b || b == c || a == c {
			continue;
		}
		// Drop slivers thinner than the T-junction healing tolerance (min altitude =
		// 2·area / longest side, on the WELDED positions). Such a triangle is wedged
		// between two differently-subdivided runs of the same line — re-triangulating
		// a previous boolean's face emits them along healed near-collinear chains —
		// and it cannot be healed by vertex insertion: `resolve_t_junctions` would
		// fold its own apex into its base edge, tripling that directed edge and
		// breaking twin pairing. Deleting it instead leaves a sub-tolerance gap whose
		// two sides T-junction-heal directly against each other (the apex IS within
		// `TJUNCTION_EPS` of the base, so it gets inserted into the neighbour's edge).
		//
		// CAVEAT (the former disjoint-difference fuzz failure): that heal argument is
		// LOCAL — it fails for a RUN of consecutive needles fanned from a far apex
		// along a near-collinear boundary chain ("deck of cards"), where dropping the
		// run leaves flanks a multiple of `TJUNCTION_EPS` apart. Those chains are now
		// removed at the source: `chain_redundant_vertices` strips redundant boundary
		// micro-subdivisions before triangulation, so such fans no longer arise.
		let (pa, pb, pc) = (verts[a as usize], verts[b as usize], verts[c as usize]);
		let area2 = (pb - pa).cross(pc - pa).length();
		let longest = (pb - pa).length().max((pc - pb).length()).max((pa - pc).length());
		if area2 < longest * TJUNCTION_EPS {
			continue;
		}
		raw.push(([a, b, c], t.normal, t.source, t.surface));
	}

	// Coincident-facet cancellation: two fragments occupying the same triangle in
	// opposite orientations form an interior membrane (delete both); two with the
	// same orientation are an exact duplicate (keep one). Without this, coincident
	// coplanar overlaps (e.g. two prisms sharing a cap) would emit a duplicate
	// directed edge and break the half-edge twin matcher.
	let (itris, tri_normal, tri_source, tri_surface) = cancel_coincident(&raw);

	// Group coplanar triangles that are connected, and recover their boundary
	// polygon. This turns the triangle soup back into B-rep faces. As a robust
	// fallback (e.g. a face that re-merges into a non-simple polygon) the group
	// is emitted as its individual triangles, which is still a valid closed solid.
	// `provenance` runs parallel to `faces`, carrying each face's source operand;
	// `face_members` (also parallel) carries each merged region's triangle indices.
	let (mut faces, mut provenance, face_members) = recover_faces(&itris, &tri_normal, &tri_source, &tri_surface, &verts);

	// Resolve T-junctions: a vertex lying in the interior of another face's edge
	// must be inserted into that edge, otherwise the half-edge twin matcher cannot
	// pair the long edge with the two shorter edges across the junction. This is
	// what makes the stitched solid closed + manifold for general overlaps. It edits
	// boundaries in place (a slice) so face count/order — and the parallel
	// `provenance` — stay aligned.
	resolve_t_junctions(&mut faces, &verts);

	// FINAL geometric verification, after healing: every merged region face must
	// (a) have an id-sane boundary — no repeated vertex, hence no (x,y)+(y,x)
	// self-fold that would put 3+ half-edges on one undirected edge — and (b)
	// ear-clip (in both the boolean's and the tessellator's clipper) to exactly its
	// member triangles' area. A region that wrapped around an unhealed over-split
	// seam walks an id-simple boundary with zero-width SPURS; healing can also jam a
	// previously clean polygon. Such a face tessellates to garbage area — and which
	// rotation of the identical boundary jammed depended on HashMap order, making
	// result volumes flake run to run. Re-expand any failing face into its member
	// triangles (always clippable), then re-heal those against the vertex pool
	// (`resolve_t_junctions` is idempotent on already-healed edges).
	let boundary_id_sane = |b: &[u32]| {
		let mut ids = b.to_vec();
		ids.sort_unstable();
		ids.windows(2).all(|w| w[0] != w[1])
	};
	let mut replaced = false;
	for fi in 0..faces.len() {
		let members = &face_members[fi];
		if members.len() < 2
			|| (boundary_id_sane(&faces[fi].boundary)
				&& boundary_covers_region(&faces[fi].boundary, members, &itris, &verts, tri_normal[members[0]]))
		{
			continue;
		}
		replaced = true;
		let face_surface = |ti: usize| -> Surface {
			let t = itris[ti];
			match tri_surface[ti] {
				s if !matches!(s, Surface::Plane { .. }) => s,
				_ => Surface::Plane { origin: verts[t[0] as usize], normal: tri_normal[ti] },
			}
		};
		let tri_input = |ti: usize| FaceInput { boundary: itris[ti].to_vec(), surface: face_surface(ti) };
		faces[fi] = tri_input(members[0]);
		provenance[fi] = tri_source[members[0]];
		for &ti in &members[1..] {
			faces.push(tri_input(ti));
			provenance.push(tri_source[ti]);
		}
	}
	if replaced {
		resolve_t_junctions(&mut faces, &verts);
	}

	// Strip redundant collinear chain micro-subdivisions from the RESULT boundaries —
	// the same per-vertex predicate `triangulate_solid` applies to operands, applied
	// at build time so the output is born clean. Welding and T-junction healing leave
	// mid-edge subdivision vertices on cut seams; on a CURVED operand's chords they
	// sit OFF the true intersection curve by the chord sagitta, and they are exactly
	// the vertices the seam snapper below must NOT pull onto the surface: bending an
	// exact chord facet there would unbalance the analytic bulge corrections that
	// make `exact_volume` machine-exact for ⟂ cuts. A removed vertex is used by
	// exactly two rings and is removed from both, so twin pairing is preserved.
	let rings: Vec<Vec<u32>> = faces.iter().map(|f| f.boundary.clone()).collect();
	let drop_v = chain_redundant_in_rings(&rings, &verts);
	if drop_v.iter().any(|&d| d) {
		for f in faces.iter_mut() {
			f.boundary.retain(|&v| !drop_v[v as usize]);
			// The per-ring removal budget keeps every boundary ≥ 3 vertices, so the
			// face list (and the parallel `provenance`) never shrinks here.
			debug_assert!(f.boundary.len() >= 3, "chain strip must leave a valid boundary");
		}
	}

	// Snap the remaining cut-seam vertices — now all genuine seam corners — onto the
	// exact surface–surface intersection of the operands' analytic surfaces.
	snap_seam_vertices(&mut verts, &faces);

	// Drop vertices no longer referenced by any face boundary. `recover_faces` merges a
	// coplanar triangle region into one maximal face, orphaning the vertices that were
	// interior to the merge; left in the array they inflate V (and the Euler characteristic
	// — a box crossing many curved facets otherwise reports a spurious genus). Compact the
	// vertex list and remap the boundaries so `from_faces` builds the true topology.
	let used: HashSet<u32> = faces.iter().flat_map(|f| f.boundary.iter().copied()).collect();
	if used.len() < verts.len() {
		let mut remap = vec![u32::MAX; verts.len()];
		let mut compact: Vec<DVec3> = Vec::with_capacity(used.len());
		for (i, p) in verts.iter().enumerate() {
			if used.contains(&(i as u32)) {
				remap[i] = compact.len() as u32;
				compact.push(*p);
			}
		}
		for f in faces.iter_mut() {
			for v in f.boundary.iter_mut() {
				*v = remap[*v as usize];
			}
		}
		verts = compact;
	}

	let mut solid = Solid::from_faces(verts, faces);
	solid.provenance = provenance;
	solid
}
