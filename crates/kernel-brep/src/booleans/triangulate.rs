// Copyright (c) LMCAD. Licensed under the MIT License.

//! Step 1 of the boolean pipeline: turn each operand [`Solid`] into a flat list
//! of plane-tagged [`Tri`]angles (ear clipping, hole bridging, and the
//! redundant-collinear-vertex strip that keeps chained booleans from
//! accumulating needle fans).

use kernel_core::math::DVec3;
use kernel_core::orient2d;

use crate::geom::{perp_basis, Surface, SurfaceChart};
use crate::topo::{FaceName, FaceSource, Solid};

use crate::tol::{EPS, TJUNCTION_EPS};

use super::Tri;

// --- Step 1: triangulation ---------------------------------------------------

/// Distance from `p` to the segment `a..b` (clamped to the endpoints).
pub(super) fn dist_point_segment(p: DVec3, a: DVec3, b: DVec3) -> f64 {
	let ab = b - a;
	let len2 = ab.length_squared();
	if len2 <= EPS * EPS {
		return (p - a).length();
	}
	let t = ((p - a).dot(ab) / len2).clamp(0.0, 1.0);
	(p - a - ab * t).length()
}

/// Vertices of `s` that are redundant micro-subdivisions of a face-boundary chain:
/// used by exactly TWO loops (a plain interior subdivision of one shared edge, with
/// matching reversed neighbours on both sides) and within [`TJUNCTION_EPS`] of the
/// segment joining those neighbours. Booleans bequeath such chains to their results
/// (every T-junction heal and weld inserts collinear boundary vertices), and a later
/// triangulation of those faces fans NEEDLE triangles along the chain whose
/// altitudes straddle the stitch sliver filter — dropping a run of them rips an
/// unhealable slit (the disjoint-difference fuzz failure), while near-coincident
/// chain steps make the two operands disagree about seam corners at the ~1e-6 scale
/// (the nano-hole fuzz failures). Removing the chain vertex from BOTH loops is
/// exact to the same tolerance the healer already moves geometry by, and keeps the
/// two sides' triangulations consistent because the decision is per-vertex, not
/// per-face. Vertices on 3+ loops (real topological junctions) are never touched.
pub(super) fn chain_redundant_vertices(s: &Solid) -> Vec<bool> {
	// Every boundary ring (outer + inner) of every face, as vertex-id cycles.
	let mut rings: Vec<Vec<u32>> = Vec::new();
	for f in s.faces() {
		let face = s.face(f);
		for &lid in std::iter::once(&face.outer).chain(face.inner.iter()) {
			rings.push(s.loop_half_edges(lid).into_iter().map(|he| s.half_edge(he).origin.0).collect());
		}
	}
	let pos: Vec<DVec3> = (0..s.vertex_count() as u32).map(|v| s.position(crate::topo::VertexId(v))).collect();
	chain_redundant_in_rings(&rings, &pos)
}

/// Core of [`chain_redundant_vertices`], over explicit boundary `rings` (vertex-id
/// cycles into `pos`). Shared between operand triangulation (rings from a [`Solid`]'s
/// loops) and [`stitch`]'s output cleanup (rings from the result's [`FaceInput`]
/// boundaries), so a boolean result is born without the chain micro-subdivisions the
/// next boolean would strip anyway.
pub(super) fn chain_redundant_in_rings(rings: &[Vec<u32>], pos: &[DVec3]) -> Vec<bool> {
	let nv = pos.len();
	let mut removed = vec![false; nv];
	loop {
		// Per-vertex usages over the live (not-yet-removed) rings: (ring, prev, next).
		// Indexed by vertex id (not hashed) so the scan order — and hence which vertex
		// of a budget-limited ring wins — is deterministic.
		let mut usage: Vec<Vec<(usize, u32, u32)>> = vec![Vec::new(); nv];
		let mut live_len = vec![0usize; rings.len()];
		for (ri, ring) in rings.iter().enumerate() {
			let live: Vec<u32> = ring.iter().copied().filter(|&v| !removed[v as usize]).collect();
			let n = live.len();
			live_len[ri] = n;
			if n < 3 {
				continue;
			}
			for (i, &v) in live.iter().enumerate() {
				usage[v as usize].push((ri, live[(i + n - 1) % n], live[(i + 1) % n]));
			}
		}
		// A ring must keep ≥ 3 vertices; spend at most (len − 3) removals per round.
		let mut budget: Vec<usize> = live_len.iter().map(|&l| l.saturating_sub(3)).collect();
		let mut changed = false;
		for v in 0..nv as u32 {
			if removed[v as usize] || usage[v as usize].len() != 2 {
				continue;
			}
			let ((r1, p1, n1), (r2, p2, n2)) = (usage[v as usize][0], usage[v as usize][1]);
			// Exactly two loops, traversing the same neighbour pair in opposite
			// directions — the signature of a mid-edge subdivision vertex. Defer to the
			// next round if a neighbour was itself removed this round, so the
			// collinearity test below never reads a stale segment endpoint.
			if r1 == r2 || p1 != n2 || n1 != p2 || p1 == n1 || p1 == v || n1 == v {
				continue;
			}
			if budget[r1] == 0 || budget[r2] == 0 || removed[p1 as usize] || removed[n1 as usize] {
				continue;
			}
			let (pv, pa, pb) = (pos[v as usize], pos[p1 as usize], pos[n1 as usize]);
			if dist_point_segment(pv, pa, pb) > TJUNCTION_EPS {
				continue;
			}
			removed[v as usize] = true;
			budget[r1] -= 1;
			budget[r2] -= 1;
			changed = true;
		}
		if !changed {
			return removed;
		}
	}
}

/// Ear-clip every face of `s` into triangles in its supporting plane. Planar
/// faces triangulate exactly; the outward normal is taken from the topological
/// winding so the orientation is reliable regardless of the surface tag sign.
///
/// A face carrying inner (hole) loops is triangulated **loop-aware**: each hole is
/// bridged into the outer ring (the same algorithm as the multi-loop tessellator)
/// and the merged ring is ear-clipped, so the hole is a true opening. Without this,
/// a holed operand face (an `extrude_with_holes` cap, a mirrored pocket) was
/// triangulated by its outer loop alone — the hole was silently FILLED, the soup
/// double-covered the hole's rim against the hole's wall facets, and any boolean
/// touching such a face exploded (R2).
///
/// Boundaries are first stripped of redundant collinear chain vertices (see
/// [`chain_redundant_vertices`]) so a chained boolean does not inherit the previous
/// ops' T-junction micro-subdivisions — the residual Level-6 stitch-explosion source.
/// Sequential by MEASUREMENT, not necessity: this is a pure per-face map (each
/// face reads only the immutable solid and `drop_v`), but it is 1–3% of boolean
/// time and allocation-bound — threading it measurably lost (see the module's
/// parallelism section), so it keeps the plain loop.
pub(super) fn triangulate_solid(s: &Solid, operand: FaceSource) -> Vec<Tri> {
	let drop_v = chain_redundant_vertices(s);
	let live_positions =
		|ids: &[crate::topo::VertexId]| -> Vec<DVec3> { ids.iter().filter(|v| !drop_v[v.0 as usize]).map(|&v| s.position(v)).collect() };
	let mut out = Vec::new();
	for f in s.faces() {
		let poly = live_positions(&s.face_vertices(f));
		if poly.len() < 3 {
			continue;
		}
		let normal = newell_normal(&poly);
		if normal.length_squared() < EPS {
			continue;
		}
		// Name every triangle by the operand face it tessellates. If this operand is
		// itself a boolean result, CARRY its existing face name (so identity survives
		// nested booleans — a face from A stays an A-face through `(A∪B)−C`). A
		// primitive's own face names are re-tagged by THIS operand instead, so a
		// direct primitive name never leaks into boolean-result provenance.
		let source = match s.face_name(f) {
			Some(name) if name.operand != FaceSource::Primitive => name,
			_ => FaceName { operand, source_face: f.0 },
		};
		let surface = s.face(f).surface;
		let inner = &s.face(f).inner;
		if inner.is_empty() {
			ear_clip(&poly, normal, source, surface, &mut out);
		} else {
			let holes: Vec<Vec<DVec3>> = inner
				.iter()
				.map(|&lid| {
					let ids: Vec<crate::topo::VertexId> = s.loop_half_edges(lid).into_iter().map(|he| s.half_edge(he).origin).collect();
					live_positions(&ids)
				})
				.collect();
			ear_clip_with_holes(&poly, &holes, normal, source, surface, &mut out);
		}
	}
	out
}

/// Ear-clip a planar face with inner hole loops into [`Tri`]s: project to the face
/// plane, orient the outer ring CCW and each hole CW, bridge every hole into the
/// outer ring ([`crate::tessellate::bridge_hole_into`] — the proven multi-loop washer
/// path), then ear-clip the merged simple ring. Emitted triangles keep the face's
/// outward `normal` winding, like [`ear_clip`].
fn ear_clip_with_holes(outer3d: &[DVec3], holes3d: &[Vec<DVec3>], normal: DVec3, source: FaceName, surface: Surface, out: &mut Vec<Tri>) {
	let mut all: Vec<DVec3> = outer3d.to_vec();
	for h in holes3d {
		all.extend_from_slice(h);
	}
	let (p2, chart) = face_clip_p2(&all, outer3d, normal, &surface);

	let mut outer: Vec<usize> = (0..outer3d.len()).collect();
	if signed_area_2d(&p2, &outer) < 0.0 {
		outer.reverse(); // outer CCW in the (u, v) projection = wound about `normal`
	}
	let mut holes: Vec<Vec<usize>> = Vec::new();
	let mut start = outer3d.len();
	for h in holes3d {
		let mut ring: Vec<usize> = (start..start + h.len()).collect();
		if signed_area_2d(&p2, &ring) > 0.0 {
			ring.reverse(); // holes CW (opposite the outer)
		}
		holes.push(ring);
		start += h.len();
	}
	// Bridge the right-most holes first so their bridges don't cross later ones.
	holes.sort_by(|a, b| {
		crate::tessellate::ring_max_x(&p2, b).partial_cmp(&crate::tessellate::ring_max_x(&p2, a)).unwrap_or(std::cmp::Ordering::Equal)
	});
	let all_rings = holes.clone();
	for hole in &holes {
		crate::tessellate::bridge_hole_into(&p2, &mut outer, hole, &all_rings);
	}
	// Ear-clip the merged INDEX ring (bridge vertices repeat as the same index, so
	// they never block each other's ears — see `ear_clip_ring_tris`).
	ear_clip_ring_tris(&all, &p2, outer, normal, source, surface, chart, out);
}

/// Area-weighted polygon normal following the winding (Newell's method).
pub(super) fn newell_normal(poly: &[DVec3]) -> DVec3 {
	let mut n = DVec3::ZERO;
	let len = poly.len();
	for i in 0..len {
		let c = poly[i];
		let d = poly[(i + 1) % len];
		n.x += (c.y - d.y) * (c.z + d.z);
		n.y += (c.z - d.z) * (c.x + d.x);
		n.z += (c.x - d.x) * (c.y + d.y);
	}
	n.normalize_or_zero()
}

/// Ear-clip a face polygon into triangles, each carrying `normal` and `source`.
/// A planar polygon clips in its projection plane (exact); a WARPED curved-tagged
/// polygon clips in its surface's parameter space (see [`face_clip_p2`]).
pub(super) fn ear_clip(poly: &[DVec3], normal: DVec3, source: FaceName, surface: Surface, out: &mut Vec<Tri>) {
	let (p2, chart) = face_clip_p2(poly, poly, normal, &surface);
	ear_clip_ring_tris(poly, &p2, (0..poly.len()).collect(), normal, source, surface, chart, out);
}

/// 2-D clip coordinates for the `pts` of one face (outer boundary first, then any
/// hole vertices), plus whether they are CHART coordinates: the PARAMETER-SPACE
/// chart of the face's analytic surface when the boundary is measurably warped
/// off its plane (a seam-snapped curved face — projection-plane ear-clipping can
/// fold there, see [`crate::geom::CURVED_WARP_EPS`]), else the plain projection
/// onto the face plane. `ring` (the outer boundary) anchors the chart's angular
/// unwrap. Falls back to the projection plane whenever the chart refuses (planar
/// ring, degenerate ring, a vertex outside the injective domain) — the
/// pre-parameter-space behaviour, never a guess.
pub(super) fn face_clip_p2(pts: &[DVec3], ring: &[DVec3], normal: DVec3, surface: &Surface) -> (Vec<glam::DVec2>, bool) {
	if let Some(p2) = SurfaceChart::for_warped_ring(surface, ring, normal).and_then(|c| c.uv_ring(pts)) {
		return (p2, true);
	}
	let (u, v) = perp_basis(normal);
	(pts.iter().map(|p| glam::DVec2::new(p.dot(u), p.dot(v))).collect(), false)
}

/// Ear-clip an index ring over `poly`/`p2` into [`Tri`]s. The ring may repeat an
/// index (a bridged hole's doubled corridor vertices): the ear containment test
/// skips occurrences of the ear's own indices by INDEX equality, so a repeated
/// bridge vertex never blocks its other occurrence's ears — the property that makes
/// bridged multi-loop rings clip cleanly (mirrors `tessellate::ear_clip_ring`).
///
/// `chart` marks `p2` as PARAMETER-SPACE coordinates (a warped curved face). The
/// clip then refuses to take a 3-D-degenerate corner as an ear: a chart bends a
/// 3-D-collinear boundary run (T-junction-heal samples shared with 3+ loops) into
/// a genuinely convex 2-D corner, and clipping it would emit a zero-area triangle
/// the degenerate filter silently DROPS — deleting a boundary step the neighbour
/// face still carries, an unpairable directed edge that explodes the stitch. In a
/// projection plane this guard is unreachable (the projection is affine, so a
/// 2-D-convex corner is never 3-D-collinear), so the planar path is untouched.
#[allow(clippy::too_many_arguments)] // one ring + its clip space + face tags
fn ear_clip_ring_tris(
	poly: &[DVec3],
	p2: &[glam::DVec2],
	mut idx: Vec<usize>,
	normal: DVec3,
	source: FaceName,
	surface: Surface,
	chart: bool,
	out: &mut Vec<Tri>,
) {
	if idx.len() < 3 {
		return;
	}
	// Restore the original winding if the index list was reversed for the ear
	// test, so emitted triangles keep the face's outward orientation.
	let reversed = signed_area_2d(p2, &idx) < 0.0;
	if reversed {
		idx.reverse();
	}
	let emit = |out: &mut Vec<Tri>, a: usize, b: usize, c: usize| {
		let mut t = if reversed {
			Tri { v: [poly[c], poly[b], poly[a]], normal, source, surface }
		} else {
			Tri { v: [poly[a], poly[b], poly[c]], normal, source, surface }
		};
		if !t.is_degenerate() {
			// Carry the triangle's OWN unit plane normal (sign-matched to the face's
			// outward `normal`), not the face's averaged Newell normal. For an exactly
			// planar face the two agree to f64 round-off, but a stitched result face
			// is only planar to the weld/heal tolerance (~1e-7): re-triangulating it
			// in a LATER boolean must reconstruct each fragment in the plane of the
			// triangle it came from (`split_triangle_by_segments` works ⊥ `t.normal`
			// through `t.v[0]`), or fragments flatten onto the borrowed average plane
			// and the two operands disagree about shared seam corners at the heal
			// scale — unhealable micro-holes in chained booleans.
			let gn = t.area_vec().normalize_or_zero();
			if gn.length_squared() > 0.5 {
				t.normal = if gn.dot(normal) < 0.0 { -gn } else { gn };
			}
			out.push(t);
		}
	};

	let mut guard = 0;
	while idx.len() > 3 && guard < 100_000 {
		guard += 1;
		let n = idx.len();
		let mut clipped = false;
		for i in 0..n {
			let ip = idx[(i + n - 1) % n];
			let ic = idx[i];
			let inx = idx[(i + 1) % n];
			let (a, b, c) = (p2[ip], p2[ic], p2[inx]);
			// Exact orientation: a near-collinear corner classifies as reflex/flat
			// consistently rather than by a rounded f64 cross product.
			if orient2d([a.x, a.y], [b.x, b.y], [c.x, c.y]) <= 0.0 {
				continue; // reflex corner
			}
			// In a chart, a 2-D-convex corner can still be a 3-D-collinear run whose
			// ear would be dropped as degenerate, deleting a shared boundary step
			// (see the doc above) — leave the vertex for a neighbouring ear instead.
			if chart {
				let area2 = (poly[ic] - poly[ip]).cross(poly[inx] - poly[ic]).length();
				if area2 * 0.5 <= EPS * EPS {
					continue;
				}
			}
			let mut ok = true;
			for &j in &idx {
				if j == ip || j == ic || j == inx {
					continue;
				}
				if point_in_tri_2d(p2[j], a, b, c) {
					ok = false;
					break;
				}
			}
			if ok {
				emit(out, ip, ic, inx);
				idx.remove(i);
				clipped = true;
				break;
			}
		}
		if !clipped {
			break;
		}
	}
	for w in 1..idx.len().saturating_sub(1) {
		emit(out, idx[0], idx[w], idx[w + 1]);
	}
}

fn signed_area_2d(p2: &[glam::DVec2], idx: &[usize]) -> f64 {
	let mut a = 0.0;
	let n = idx.len();
	for i in 0..n {
		let c = p2[idx[i]];
		let d = p2[idx[(i + 1) % n]];
		a += c.x * d.y - d.x * c.y;
	}
	a * 0.5
}

fn point_in_tri_2d(p: glam::DVec2, a: glam::DVec2, b: glam::DVec2, c: glam::DVec2) -> bool {
	// Exact orientation sign (`sign(p1,p2,p3) = orient2d(p3,p1,p2)`): a point on a
	// candidate ear's edge classifies consistently instead of by a rounded f64.
	let sign = |p1: glam::DVec2, p2: glam::DVec2, p3: glam::DVec2| orient2d([p3.x, p3.y], [p1.x, p1.y], [p2.x, p2.y]);
	let d1 = sign(p, a, b);
	let d2 = sign(p, b, c);
	let d3 = sign(p, c, a);
	let has_neg = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
	let has_pos = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
	!(has_neg && has_pos)
}
