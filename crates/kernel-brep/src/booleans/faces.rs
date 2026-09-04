// Copyright (c) LMCAD. Licensed under the MIT License.

//! Face recovery for the stitched soup: cancel coincident facets, merge
//! coplanar adjacent triangles back into maximal planar faces (area-verified),
//! orient the recovered boundaries, and heal the T-junctions welding leaves
//! behind.

use std::collections::HashMap;

use kernel_core::math::DVec3;

use crate::geom::{perp_basis, Surface};
use crate::topo::{FaceInput, FaceName, FaceSource};

use crate::tol::{EPS, TJUNCTION_EPS, WELD_EPS};

use super::triangulate::{ear_clip, newell_normal};
use super::Tri;

/// A welded soup triangle: vertex ids, face normal, provenance name, and the
/// operand face's analytic surface.
pub(super) type RawTri = ([u32; 3], DVec3, FaceName, Surface);

/// Cancel coincident facets occupying the same welded triangle. A triangle that
/// appears with both orientations forms an interior membrane and is removed
/// entirely; identical-orientation duplicates collapse to a single copy. The
/// surviving triangle keeps its **original winding** (never reconstructed), so the
/// global orientation of the soup is preserved.
pub(super) fn cancel_coincident(raw: &[RawTri]) -> (Vec<[u32; 3]>, Vec<DVec3>, Vec<FaceName>, Vec<Surface>) {
	// Net signed multiplicity per unordered vertex triple, plus a representative
	// triangle for each winding seen.
	let key_of = |t: [u32; 3]| -> [u32; 3] {
		let mut s = t;
		s.sort_unstable();
		s
	};
	// canonical-positive winding = the even rotations of the sorted triple.
	let is_positive = |t: [u32; 3]| -> bool {
		let mut s = t;
		s.sort_unstable();
		[[s[0], s[1], s[2]], [s[1], s[2], s[0]], [s[2], s[0], s[1]]].contains(&t)
	};
	// (net count, +winding rep, −winding rep); each rep carries its normal,
	// provenance and surface.
	type Slot = (i32, Option<RawTri>, Option<RawTri>);
	let mut acc: HashMap<[u32; 3], Slot> = HashMap::new();
	// Emit in FIRST-INSERTION key order, not HashMap iteration order: the order of
	// this soup decides every downstream face-recovery choice (region representative
	// → normal/surface/provenance tags, face order → the next chained boolean's
	// entire arrangement), so a per-instance-random drain made results flake run to
	// run (R5). The map stays for O(1) accumulation; `order` drives the output.
	let mut order: Vec<[u32; 3]> = Vec::with_capacity(raw.len());
	for &(t, n, src, surf) in raw {
		let k = key_of(t);
		let e = acc.entry(k).or_insert_with(|| {
			order.push(k);
			(0, None, None)
		});
		if is_positive(t) {
			e.0 += 1;
			e.1 = Some((t, n, src, surf));
		} else {
			e.0 -= 1;
			e.2 = Some((t, n, src, surf));
		}
	}
	let mut tris = Vec::new();
	let mut normals = Vec::new();
	let mut sources = Vec::new();
	let mut surfaces = Vec::new();
	for k in order {
		let (count, pos, neg) = acc.remove(&k).expect("every recorded key was inserted exactly once");
		// Net zero ⇒ membrane (equal opposite copies) ⇒ drop. Otherwise keep one
		// copy in the winning winding, with its original (un-reconstructed) order.
		let pick = if count > 0 { pos } else if count < 0 { neg } else { None };
		if let Some((t, n, src, surf)) = pick {
			tris.push(t);
			normals.push(n);
			sources.push(src);
			surfaces.push(surf);
		}
	}
	(tris, normals, sources, surfaces)
}

/// Recover maximal planar faces from indexed triangles. Triangles are grouped by
/// connected coplanar regions sharing an edge; each region's outer boundary is
/// extracted by walking unshared (boundary) edges. Non-simple regions fall back
/// to per-triangle faces.
///
/// Returns, parallel to the faces: each face's provenance, and the soup-triangle
/// indices a merged region face covers (a single index for a fallback triangle) —
/// so [`stitch`]'s post-healing verification can re-expand a face that fails to
/// ear-clip back into its triangles.
pub(super) fn recover_faces(
	itris: &[[u32; 3]],
	normals: &[DVec3],
	sources: &[FaceName],
	surfaces: &[Surface],
	verts: &[DVec3],
) -> (Vec<FaceInput>, Vec<FaceName>, Vec<Vec<usize>>) {
	// The analytic surface a result region inherits from its source face — but only a
	// CURVED one (a plane is recovered exactly from the geometry). `None` ⇒ tag a plane.
	let curved_surface = |ti: usize| -> Option<Surface> {
		match surfaces[ti] {
			s if !matches!(s, Surface::Plane { .. }) => Some(s),
			_ => None,
		}
	};
	let n = itris.len();
	// Union-find over triangles that are coplanar and edge-adjacent.
	let mut parent: Vec<usize> = (0..n).collect();
	fn find(p: &mut [usize], mut x: usize) -> usize {
		while p[x] != x {
			p[x] = p[p[x]];
			x = p[x];
		}
		x
	}
	// Map undirected edge → triangles touching it.
	let mut edge_map: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
	for (ti, t) in itris.iter().enumerate() {
		for &(a, b) in &[(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
			let k = if a < b { (a, b) } else { (b, a) };
			edge_map.entry(k).or_default().push(ti);
		}
	}
	for tlist in edge_map.values() {
		// Merge ONLY across 2-manifold edges (exactly two incident triangles). An edge
		// used by ≥3 triangles is a non-manifold junction (e.g. a leftover fold, or a
		// wall meeting a cap along a shared line): merging across it would (a) depend on
		// which triangle happens to be `tlist[0]` — the soup order is HashMap-random, so
		// the recovered faces (and even the result's volume) varied run to run — and
		// (b) chain triangles from opposite sides of the junction into one region whose
		// boundary walk is id-simple yet geometrically self-overlapping.
		if tlist.len() == 2 && coplanar(normals[tlist[0]], normals[tlist[1]]) {
			let (ri, rj) = (find(&mut parent, tlist[0]), find(&mut parent, tlist[1]));
			if ri != rj {
				parent[ri] = rj;
			}
		}
	}
	// Bucket triangles by region.
	let mut regions: HashMap<usize, Vec<usize>> = HashMap::new();
	for ti in 0..n {
		let r = find(&mut parent, ti);
		regions.entry(r).or_default().push(ti);
	}

	// Deterministic region order. The union-find PARTITION is merge-order-independent,
	// but the surviving ROOT id per region is not: merges run in `edge_map.values()`
	// order, which is HashMap-random per run, so sorting the root keys (the previous
	// fix) still shuffled the relative region — hence result-face — order between runs
	// (R5). Each member list is filled in ascending `ti`, so its first element is the
	// region's smallest triangle index: a stable key that depends only on the
	// (deterministic) soup order. Sort by that instead of by root.
	let mut region_list: Vec<Vec<usize>> = regions.into_values().collect();
	region_list.sort_unstable_by_key(|m| m[0]);

	let mut faces = Vec::new();
	let mut provenance = Vec::new();
	let mut face_members: Vec<Vec<usize>> = Vec::new();
	for members in &region_list {
		let normal = normals[members[0]];
		// A coplanar-connected region comes from one operand's surface; take the
		// representative member's provenance for the merged face.
		let region_src = sources[members[0]];
		// A recovered boundary is only accepted if it is a single simple loop whose
		// winding can be made to agree with the region normal; otherwise we fall
		// back to per-triangle faces, which always stitch into a valid closed solid.
		// (Whether the merged polygon also EAR-CLIPS to the region's area is verified
		// after T-junction healing, in `stitch` — failures re-expand to triangles.)
		let recovered = region_boundary(members, itris)
			.filter(|b| b.len() >= 3)
			.and_then(|b| orient_boundary(b, normal, verts));
		match recovered {
			Some(boundary) => {
				let origin = verts[boundary[0] as usize];
				// A coplanar-connected region of a curved primitive is a SINGLE chord facet
				// of that surface — adjacent facets differ in normal (e.g. a 48-gon cylinder's
				// bands are 7.5° apart, far above the coplanar tolerance) so they never merge
				// here. Keep the analytic tag regardless of vertex count: a clipped bore band
				// recovers as a >4-gon (T-junctions add collinear verts) but is still one flat
				// facet, so it tessellates flat at subdivision 1 (watertight) yet adaptive
				// tessellation and exact_volume recover the true curved surface from the tag.
				let surface = curved_surface(members[0]).unwrap_or(Surface::Plane { origin, normal });
				faces.push(FaceInput { boundary, surface });
				provenance.push(region_src);
				face_members.push(members.clone());
			}
			None => {
				for &ti in members {
					let t = itris[ti];
					let origin = verts[t[0] as usize];
					// A per-triangle fallback facet is always ≤3 vertices, so a curved
					// source surface is safe to keep (it tessellates flat).
					let surface = curved_surface(ti).unwrap_or(Surface::Plane { origin, normal });
					faces.push(FaceInput { boundary: vec![t[0], t[1], t[2]], surface });
					provenance.push(sources[ti]);
					face_members.push(vec![ti]);
				}
			}
		}
	}
	(faces, provenance, face_members)
}

/// Whether ear-clipping `boundary` reproduces the region's exact triangle area —
/// the guarantee that the recovered polygon is geometrically equivalent to the
/// triangles it replaces. An id-simple walk can still DOUBLE BACK along an
/// unhealed split-line seam interior to the region (the two sides of a full-line
/// over-split carry different subdivision vertices, so their directed edges do not
/// cancel and both chains land in the walk as zero-width spurs). Such a spur
/// polygon has the right Newell area but cannot be ear-clipped — the fan fallback
/// covers the wrong geometry, which (pre-fix) made result volumes wrong AND
/// HashMap-order flaky. The area check converts that into a per-triangle fallback.
pub(super) fn boundary_covers_region(boundary: &[u32], members: &[usize], itris: &[[u32; 3]], verts: &[DVec3], normal: DVec3) -> bool {
	let tri_area = |t: &[u32; 3]| {
		let (a, b, c) = (verts[t[0] as usize], verts[t[1] as usize], verts[t[2] as usize]);
		(b - a).cross(c - a).length() * 0.5
	};
	let member_area: f64 = members.iter().map(|&ti| tri_area(&itris[ti])).sum();
	let poly: Vec<DVec3> = boundary.iter().map(|&i| verts[i as usize]).collect();

	// BOTH downstream triangulators must reproduce the area: the boolean's own
	// `ear_clip` (this polygon is re-triangulated when the result enters another
	// boolean) and the tessellator's `ear_clip_ring` (volume / rendering / export
	// meshes). Their blocking predicates differ slightly, so a polygon can clip
	// cleanly in one and jam (fan-fallback garbage) in the other.
	let mut clipped: Vec<Tri> = Vec::new();
	let scratch = FaceName { operand: FaceSource::Primitive, source_face: 0 };
	ear_clip(&poly, normal, scratch, Surface::Plane { origin: poly[0], normal }, &mut clipped);
	let boolean_area: f64 = clipped.iter().map(|t| t.area()).sum();
	if (boolean_area - member_area).abs() > member_area.max(1.0) * 1e-9 {
		return false;
	}

	let mut mesh = kernel_core::mesh::Mesh::new();
	let (u, v) = perp_basis(normal);
	let p2: Vec<glam::DVec2> = poly.iter().map(|p| glam::DVec2::new(p.dot(u), p.dot(v))).collect();
	crate::tessellate::ear_clip_ring(&mut mesh, &poly, &p2, (0..poly.len()).collect(), normal);
	let mut mesh_area = 0.0;
	for t in mesh.indices.chunks_exact(3) {
		let (a, b, c) = (mesh.positions[t[0] as usize], mesh.positions[t[1] as usize], mesh.positions[t[2] as usize]);
		mesh_area += ((b - a).cross(c - a)).length() as f64 * 0.5;
	}
	// The mesh stores f32 positions, so this comparison carries ~1e-6 relative noise;
	// a jammed clip diverges by whole percents, far above the 1e-4 line.
	(mesh_area - member_area).abs() <= member_area.max(1.0) * 1e-4
}

/// Force a recovered boundary loop's winding to agree with `normal`, or return
/// `None` if the loop is geometrically degenerate (near-zero area). This makes the
/// orientation independent of which directed edges the walk happened to pick.
fn orient_boundary(boundary: Vec<u32>, normal: DVec3, verts: &[DVec3]) -> Option<Vec<u32>> {
	let poly: Vec<DVec3> = boundary.iter().map(|&i| verts[i as usize]).collect();
	let nrm = newell_normal(&poly);
	if nrm.length_squared() < 0.5 {
		return None; // degenerate / self-overlapping loop
	}
	if nrm.dot(normal) < 0.0 {
		let mut b = boundary;
		b.reverse();
		Some(b)
	} else {
		Some(boundary)
	}
}

/// Insert collinear-interior vertices into face boundary edges so that every
/// shared boundary is split identically on both incident faces (eliminating
/// T-junctions). General over any face configuration.
pub(super) fn resolve_t_junctions(faces: &mut [FaceInput], verts: &[DVec3]) {
	// Candidate vertices: every vertex that appears on some face boundary.
	let mut used: Vec<u32> = faces.iter().flat_map(|f| f.boundary.iter().copied()).collect();
	used.sort_unstable();
	used.dedup();

	// Spatial hash over the candidates. The previous form scanned EVERY used
	// vertex for EVERY boundary edge — O(E·V), and at campaign scale (28k
	// faces) that is billions of point/segment tests: observed as a silent
	// 45-minute 100%-CPU quasi-hang (DRYBOX 2026-07-28, `sample` pinned 100%
	// of time here). Per edge we now visit only candidates whose grid cells
	// the edge's inflated AABB touches. Determinism is preserved EXACTLY:
	// gathered candidates are re-sorted into ascending-index order — the same
	// order the linear scan produced — before the identical filtering and the
	// identical stable sort by parameter, so the output boundaries are
	// bit-for-bit what the O(E·V) loop built (the map is only ever queried by
	// key, never iterated — the R5 lesson).
	let cell = {
		let mut lo = DVec3::splat(f64::INFINITY);
		let mut hi = DVec3::splat(f64::NEG_INFINITY);
		for &c in &used {
			let p = verts[c as usize];
			lo = lo.min(p);
			hi = hi.max(p);
		}
		let diag = (hi - lo).length().max(1e-6);
		(diag / (used.len().max(1) as f64).cbrt()).max(TJUNCTION_EPS * 8.0)
	};
	let key = |p: DVec3| ((p.x / cell).floor() as i64, (p.y / cell).floor() as i64, (p.z / cell).floor() as i64);
	let mut grid: std::collections::HashMap<(i64, i64, i64), Vec<u32>> = std::collections::HashMap::new();
	for &c in &used {
		grid.entry(key(verts[c as usize])).or_default().push(c);
	}
	let mut candidates: Vec<u32> = Vec::new();

	for f in faces.iter_mut() {
		let mut out: Vec<u32> = Vec::with_capacity(f.boundary.len());
		let n = f.boundary.len();
		for i in 0..n {
			let a = f.boundary[i];
			let b = f.boundary[(i + 1) % n];
			out.push(a);
			let pa = verts[a as usize];
			let pb = verts[b as usize];
			let ab = pb - pa;
			let len2 = ab.length_squared();
			// Degenerate-edge guard: reject only sub-weld (physically impossible
			// post-weld, so in practice repeated-id / NaN) edges. This compares
			// SQUARED length, so the old `len2 < EPS` form silently exempted every
			// edge shorter than √EPS ≈ 3.2e-5 mm from healing — 80× the healing
			// tolerance. A micro cut stub near a seam corner (~1e-5, born where a
			// seam polyline crosses both operands' facet edges close together)
			// could then never receive the OTHER operand's interior T-vertex, and
			// the sliver filter's safety argument ("a dropped sliver's apex gets
			// inserted into the neighbour's edge") broke: fuzz seed 83894724552572
			// (sphere∩sphere) left an unhealable 2.4e-5 micro-triangle hole.
			if len2 < WELD_EPS * WELD_EPS {
				continue;
			}
			// Gather candidates from the cells the edge's inflated AABB covers,
			// then restore the ascending-index order of the original scan.
			candidates.clear();
			let (elo, ehi) = (pa.min(pb) - DVec3::splat(TJUNCTION_EPS), pa.max(pb) + DVec3::splat(TJUNCTION_EPS));
			let (k0, k1) = (key(elo), key(ehi));
			for kx in k0.0..=k1.0 {
				for ky in k0.1..=k1.1 {
					for kz in k0.2..=k1.2 {
						if let Some(cs) = grid.get(&(kx, ky, kz)) {
							candidates.extend_from_slice(cs);
						}
					}
				}
			}
			candidates.sort_unstable();
			// Collect vertices strictly interior to edge a→b, ordered by parameter.
			let mut on_edge: Vec<(f64, u32)> = Vec::new();
			for &c in &candidates {
				if c == a || c == b {
					continue;
				}
				let pc = verts[c as usize];
				let t = (pc - pa).dot(ab) / len2;
				// `!(in range)` rather than `t <= EPS || t >= ...` so a non-finite `t`
				// (NaN from a degenerate zero-length edge, `len2 ≈ 0`) is rejected too.
				if !(t > EPS && t < 1.0 - EPS) {
					continue;
				}
				// Perpendicular distance to the line a→b.
				let proj = pa + ab * t;
				if (pc - proj).length() <= TJUNCTION_EPS {
					on_edge.push((t, c));
				}
			}
			on_edge.sort_by(|x, y| x.0.total_cmp(&y.0));
			for (_, c) in on_edge {
				out.push(c);
			}
		}
		// Drop any consecutive duplicate that may arise from the insertion.
		out.dedup();
		if out.len() >= 2 && out[0] == *out.last().unwrap() {
			out.pop();
		}
		// Collapse zero-width back-track spurs `… v, w, v …` (cyclically). Insertion
		// creates them when a face's boundary doubles back along a sub-tolerance-wide
		// arm — e.g. an over-split seam the region wrapped around — and a vertex of
		// one arm lands on the other arm's edge. Left in place, the boundary carries
		// both (v,w) and (w,v): a third+fourth half-edge on one undirected edge, which
		// the twin matcher cannot pair (the un-closable seam R2 exposed). The spur is
		// geometrically a zero-area sliver, so removing it is exact to `TJUNCTION_EPS`.
		collapse_backtracks(&mut out);
		if out.len() >= 3 {
			f.boundary = out;
		}
	}
}

/// Iteratively remove cyclic back-track patterns `v, w, v` from a boundary ring
/// (deleting `w` and one `v`), then consecutive duplicates, until stable.
fn collapse_backtracks(ring: &mut Vec<u32>) {
	loop {
		let n = ring.len();
		if n < 3 {
			return;
		}
		let mut collapsed = false;
		for i in 0..n {
			let prev = ring[(i + n - 1) % n];
			let next = ring[(i + 1) % n];
			if prev == next {
				// Remove the spur tip `ring[i]` and one copy of the repeated vertex.
				let (hi, lo) = if i > (i + 1) % n { (i, (i + 1) % n) } else { ((i + 1) % n, i) };
				ring.remove(hi);
				ring.remove(lo);
				collapsed = true;
				break;
			}
		}
		if !collapsed {
			return;
		}
		ring.dedup();
		if ring.len() >= 2 && ring[0] == *ring.last().unwrap() {
			ring.pop();
		}
	}
}

pub(super) fn coplanar(a: DVec3, b: DVec3) -> bool {
	a.dot(b) > 0.0 && a.cross(b).length() < 1e-7
}

/// Extract the single outer boundary loop of a connected coplanar triangle region
/// by walking its boundary (singly-used directed) edges. Returns `None` if the
/// region's boundary is not a single simple loop (holes, pinches, T-junctions).
fn region_boundary(members: &[usize], itris: &[[u32; 3]]) -> Option<Vec<u32>> {
	// Count directed edges; a boundary edge appears once, an interior edge cancels
	// with its reverse.
	let mut dir_count: HashMap<(u32, u32), i32> = HashMap::new();
	for &ti in members {
		let t = itris[ti];
		for &(a, b) in &[(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
			*dir_count.entry((a, b)).or_insert(0) += 1;
		}
	}
	// Boundary directed edges: those whose reverse is absent.
	let mut next: HashMap<u32, u32> = HashMap::new();
	for (&(a, b), &c) in &dir_count {
		let rev = dir_count.get(&(b, a)).copied().unwrap_or(0);
		let net = c - rev;
		if net > 0 {
			if next.contains_key(&a) {
				return None; // branching boundary (not a simple loop)
			}
			next.insert(a, b);
		}
	}
	if next.is_empty() {
		return None;
	}
	// Walk the loop from a deterministic start (smallest vertex id) so the result
	// does not depend on HashMap iteration order.
	let start = *next.keys().min().unwrap();
	let mut loop_v = vec![start];
	let mut cur = start;
	let mut guard = 0;
	while let Some(&nx) = next.get(&cur) {
		guard += 1;
		if guard > next.len() + 1 {
			return None; // failed to close cleanly
		}
		if nx == start {
			break;
		}
		loop_v.push(nx);
		cur = nx;
	}
	if loop_v.len() != next.len() {
		return None; // disjoint loops ⇒ region has a hole; fall back
	}
	Some(loop_v)
}
