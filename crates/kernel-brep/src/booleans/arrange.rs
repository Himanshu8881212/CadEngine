// Copyright (c) LMCAD. Licensed under the MIT License.

//! Step 2 of the boolean pipeline: co-refinement. Every triangle of the subject
//! is split along its intersection with every triangle of the cutter, so that
//! afterwards no fragment straddles the other solid's surface.

use std::collections::HashMap;

use kernel_core::math::DVec3;
use kernel_core::orient3d;

use crate::geom::perp_basis;

use crate::tol::EPS;

use super::par::{stage_flat_map, CLASSIFY_CHUNK_WORK, CLASSIFY_WORK_CUTOFF, PAR_CHUNK, PAR_CUTOFF};
use super::Tri;

// --- Step 2: co-refinement ---------------------------------------------------

/// Split every triangle of `subject` along its intersection with every triangle
/// of `cutter`, returning the fragments. After this the surface of `cutter` never
/// crosses the interior of any returned fragment, so each fragment is uniformly
/// inside or outside the `cutter` solid.
pub(super) fn co_refine(subject: &[Tri], cutter: &[Tri], workers: usize) -> Vec<Tri> {
	// Broadphase: two triangles can only cut each other if their AABBs overlap. A
	// uniform spatial grid over the cutter boxes yields the nearby candidates in
	// ~O(1), turning the naïve O(n·m) scan into O(n+m) typical. It is purely a cull —
	// the f64 AABB re-test below keeps the output byte-for-byte identical.
	let cutter_boxes: Vec<(DVec3, DVec3)> = cutter.iter().map(tri_aabb).collect();
	let grid = CutterGrid::build(&cutter_boxes);
	// Pure per-SUBJECT-TRIANGLE map (parallelism-safe, see the module's
	// parallelism section): the grid and cutter list are read-only shared input
	// (`candidates` sorts + dedups, so candidate ORDER never depends on HashMap
	// iteration), the scratch buffers are chunk-local and cleared per triangle
	// exactly as the sequential loop cleared them, and fragments concatenate in
	// subject order.
	//
	// Scheduling (output-invariant): a FEW large subject triangles against MANY
	// small cutters is the expensive shape — each large AABB overflows the grid's
	// span cap into the check-all path, costing O(|cutter|) per item — so the
	// engage test and chunk length are work-based like classification's, with
	// the pessimistic |subject|×|cutter| bound as the work proxy (over-engaging
	// a well-pruned stage wastes only the ~0.1 ms spawn, never correctness).
	let work = subject.len().saturating_mul(cutter.len().max(1));
	let chunk_len = (CLASSIFY_CHUNK_WORK / cutter.len().max(1)).clamp(1, PAR_CHUNK);
	let engage = subject.len() >= PAR_CUTOFF || work >= CLASSIFY_WORK_CUTOFF;
	stage_flat_map(workers, subject, chunk_len, engage, |chunk| {
		let mut out = Vec::with_capacity(chunk.len());
		let mut segments: Vec<(DVec3, DVec3)> = Vec::new();
		let mut cand: Vec<u32> = Vec::new();
		for &t in chunk {
			segments.clear();
			let tb = tri_aabb(&t);
			grid.candidates(tb, cutter.len(), &mut cand);
			for &ci in &cand {
				if aabb_overlap(tb, cutter_boxes[ci as usize]) {
					tri_tri_cuts(&t, &cutter[ci as usize], &mut segments);
				}
			}
			if segments.is_empty() {
				out.push(t);
			} else {
				split_triangle_by_segments(&t, &segments, &mut out);
			}
		}
		out
	})
}

/// A uniform spatial hash over cutter triangle AABBs for broadphase candidate
/// lookup. Cutters spanning too many cells go in a `large` list checked against
/// every subject. It returns a *superset* of the overlapping cutters (overlapping
/// AABBs always share a cell), so the f64 re-test in [`co_refine`] still culls
/// exactly and the boolean result is unchanged.
struct CutterGrid {
	inv_cell: f64,
	cells: HashMap<[i64; 3], Vec<u32>>,
	large: Vec<u32>,
}

impl CutterGrid {
	fn key(inv: f64, p: DVec3) -> [i64; 3] {
		[(p.x * inv).floor() as i64, (p.y * inv).floor() as i64, (p.z * inv).floor() as i64]
	}

	fn build(boxes: &[(DVec3, DVec3)]) -> Self {
		const CAP: i64 = 64; // cell budget before a cutter is treated as "large"
		if boxes.is_empty() {
			return CutterGrid { inv_cell: 1.0, cells: HashMap::new(), large: Vec::new() };
		}
		// Cell ≈ mean triangle extent, so a typical triangle spans ~one cell.
		let mean = boxes.iter().map(|(lo, hi)| (*hi - *lo).max_element()).sum::<f64>() / boxes.len() as f64;
		let inv_cell = 1.0 / mean.max(1e-12);
		let mut cells: HashMap<[i64; 3], Vec<u32>> = HashMap::new();
		let mut large = Vec::new();
		for (i, &(lo, hi)) in boxes.iter().enumerate() {
			let (klo, khi) = (Self::key(inv_cell, lo), Self::key(inv_cell, hi));
			let span = (khi[0] - klo[0] + 1) * (khi[1] - klo[1] + 1) * (khi[2] - klo[2] + 1);
			if span > CAP {
				large.push(i as u32);
				continue;
			}
			for cx in klo[0]..=khi[0] {
				for cy in klo[1]..=khi[1] {
					for cz in klo[2]..=khi[2] {
						cells.entry([cx, cy, cz]).or_default().push(i as u32);
					}
				}
			}
		}
		CutterGrid { inv_cell, cells, large }
	}

	fn candidates(&self, (lo, hi): (DVec3, DVec3), n: usize, out: &mut Vec<u32>) {
		out.clear();
		let (klo, khi) = (Self::key(self.inv_cell, lo), Self::key(self.inv_cell, hi));
		let span = (khi[0] - klo[0] + 1) * (khi[1] - klo[1] + 1) * (khi[2] - klo[2] + 1);
		if span > 4096 {
			out.extend(0..n as u32); // pathologically large subject: just check all
			return;
		}
		out.extend_from_slice(&self.large);
		for cx in klo[0]..=khi[0] {
			for cy in klo[1]..=khi[1] {
				for cz in klo[2]..=khi[2] {
					if let Some(v) = self.cells.get(&[cx, cy, cz]) {
						out.extend_from_slice(v);
					}
				}
			}
		}
		out.sort_unstable();
		out.dedup();
	}
}

/// Axis-aligned bounding box of a triangle.
fn tri_aabb(t: &Tri) -> (DVec3, DVec3) {
	(t.v[0].min(t.v[1]).min(t.v[2]), t.v[0].max(t.v[1]).max(t.v[2]))
}

/// Do two AABBs overlap? Uses *exact* comparisons (no `EPS` slack) so this stays
/// consistent with [`CutterGrid`]'s exact `floor` cell keys: overlapping boxes then
/// provably share a grid cell, preserving the broadphase superset invariant. A
/// genuine geometric tolerance is applied downstream by `tri_tri_cuts`, not here.
fn aabb_overlap(a: (DVec3, DVec3), b: (DVec3, DVec3)) -> bool {
	a.0.x <= b.1.x && a.1.x >= b.0.x && a.0.y <= b.1.y && a.1.y >= b.0.y && a.0.z <= b.1.z && a.1.z >= b.0.z
}

/// The intersection segment of two triangles, in `subject`'s plane, or `None` if
/// they do not cross in a segment (disjoint, touching at a point, or coplanar —
/// coplanar overlap is handled by classification, not by splitting).
fn tri_tri_cuts(subject: &Tri, cutter: &Tri, out: &mut Vec<(DVec3, DVec3)>) {
	let n_s = subject.area_vec().normalize_or_zero();
	let n_c = cutter.area_vec().normalize_or_zero();
	if n_s.length_squared() < 0.5 || n_c.length_squared() < 0.5 {
		return;
	}
	// `|n_s × n_c|` is the sine of the angle between the two face normals. Use the
	// same ~1e-6 tolerance as the in-plane / on-plane coincidence tests below and in
	// `point_in_tri_3d`, so a pair of near-coplanar shared faces is imprinted (not
	// co-refined as transversal, which would leave the stitched solid non-manifold).
	if n_s.cross(n_c).length() < 1e-6 {
		// Coplanar: imprint the cutter's three edge *lines* onto the subject so the
		// coplanar overlap region is carved out along the cutter's boundary. The
		// raw (unclipped) edges are emitted as cut segments; `split_convex_by_line`
		// only acts where a line actually crosses the subject, so a cutter edge that
		// runs outside the subject still contributes its bounding line — which is
		// essential, since the overlap polygon is bounded by every cutter edge, not
		// only the ones passing through the subject triangle.
		if (n_s.dot(cutter.v[0] - subject.v[0])).abs() > 1e-6 {
			return; // parallel but not in the same plane
		}
		for i in 0..3 {
			out.push((cutter.v[i], cutter.v[(i + 1) % 3]));
		}
		return;
	}
	// Transversal case: plane–plane SSI line clipped to both triangle polygons.
	let d_s = n_s.dot(subject.v[0]);
	let Some(chord) = triangle_plane_chord(&cutter.v, &subject.v, n_s, d_s) else {
		return;
	};
	let Some(seg) = clip_segment_to_triangle(chord, subject) else {
		return;
	};
	let Some(seg) = clip_segment_to_triangle(seg, cutter) else {
		return;
	};
	if (seg.1 - seg.0).length() >= EPS {
		out.push(seg);
	}
}

/// Exact side of `p` relative to the oriented plane through `plane`'s three
/// points: `+1` / `-1` / `0` (on the plane), via the exact [`orient3d`] predicate.
/// The absolute sign convention is irrelevant to the callers here — they only use
/// equality-to-zero and opposite-signs, both invariant under a global flip.
fn plane_side(plane: &[DVec3; 3], p: DVec3) -> i32 {
	let arr = |v: DVec3| [v.x, v.y, v.z];
	let o = orient3d(arr(plane[0]), arr(plane[1]), arr(plane[2]), arr(p));
	if o > 0.0 {
		1
	} else if o < 0.0 {
		-1
	} else {
		0
	}
}

/// Chord where triangle `tri` crosses the supporting plane of `plane` (the
/// subject's three vertices): the segment of `tri` lying on the plane, or `None`
/// if `tri` does not straddle it.
///
/// Which side each vertex lies on — and therefore which edges cross — is decided
/// by the **exact** [`orient3d`] predicate, so a near-coplanar vertex is classified
/// by its true side rather than swallowed by an absolute epsilon. The crossing
/// *position* is still interpolated in `f64` from the plane distance (`n`, `d`),
/// which is all the downstream geometry needs.
fn triangle_plane_chord(tri: &[DVec3; 3], plane: &[DVec3; 3], n: DVec3, d: f64) -> Option<(DVec3, DVec3)> {
	let side = [plane_side(plane, tri[0]), plane_side(plane, tri[1]), plane_side(plane, tri[2])];
	let dist = [n.dot(tri[0]) - d, n.dot(tri[1]) - d, n.dot(tri[2]) - d];
	let mut hits: Vec<DVec3> = Vec::new();
	for i in 0..3 {
		let j = (i + 1) % 3;
		if side[i] == 0 {
			hits.push(tri[i]);
		}
		// Exactly-opposite sides ⇒ the edge crosses the plane between the vertices.
		if side[i] * side[j] < 0 {
			let (di, dj) = (dist[i], dist[j]);
			let t = di / (di - dj);
			hits.push(tri[i] + (tri[j] - tri[i]) * t);
		}
	}
	dedup_close(&mut hits);
	if hits.len() >= 2 {
		Some((hits[0], hits[hits.len() - 1]))
	} else {
		None
	}
}

fn dedup_close(pts: &mut Vec<DVec3>) {
	let mut i = 0;
	while i < pts.len() {
		let mut j = i + 1;
		while j < pts.len() {
			if (pts[i] - pts[j]).length() <= EPS {
				pts.remove(j);
			} else {
				j += 1;
			}
		}
		i += 1;
	}
}

/// Clip a 3D segment to a triangle, treating the segment and triangle as
/// coplanar (the segment already lies in the triangle's plane). Returns the
/// portion of the segment inside the triangle, or `None` if it misses.
fn clip_segment_to_triangle(seg: (DVec3, DVec3), tri: &Tri) -> Option<(DVec3, DVec3)> {
	let n = tri.area_vec().normalize_or_zero();
	if n.length_squared() < 0.5 {
		return None;
	}
	let (mut t0, mut t1) = (0.0f64, 1.0f64);
	let dir = seg.1 - seg.0;
	// Half-plane clip against each triangle edge (inward normal = n × edge).
	for i in 0..3 {
		let a = tri.v[i];
		let b = tri.v[(i + 1) % 3];
		let edge = b - a;
		let inward = n.cross(edge); // points into the triangle for CCW (n,winding)
							  // Constraint: inward · (P − a) >= 0.
		let denom = inward.dot(dir);
		let num = inward.dot(seg.0 - a);
		if denom.abs() < EPS {
			if num < -EPS {
				return None; // segment entirely outside this edge
			}
		} else {
			let t = -num / denom;
			if denom > 0.0 {
				if t > t0 {
					t0 = t;
				}
			} else if t < t1 {
				t1 = t;
			}
		}
		if t0 > t1 + EPS {
			return None;
		}
	}
	let p0 = seg.0 + dir * t0;
	let p1 = seg.0 + dir * t1;
	if (p1 - p0).length() < EPS {
		None
	} else {
		Some((p0, p1))
	}
}

/// Split triangle `t` so that none of the `segments` (each lying in `t`'s plane)
/// crosses a fragment's interior. Each cut segment defines a supporting line in
/// the face plane; the triangle (as a convex polygon) is split by every such line
/// into convex sub-polygons, which are then fanned into triangles.
///
/// Splitting by the *whole* supporting line can over-split (a fragment may be cut
/// where the segment does not actually reach), but it never under-splits, so no
/// fragment ever straddles a cutter face — exactly the invariant classification
/// needs. Over-splitting is harmless: coplanar adjacent fragments are merged back
/// into one face during stitching.
pub(super) fn split_triangle_by_segments(t: &Tri, segments: &[(DVec3, DVec3)], out: &mut Vec<Tri>) {
	let n = t.normal;
	let (u, v) = perp_basis(n);
	let to2 = |p: DVec3| glam::DVec2::new(p.dot(u), p.dot(v));
	let to3 = |q: glam::DVec2| {
		// Reconstruct the 3D point: u,v span the plane through t.v[0].
		let o = t.v[0];
		o + u * (q.x - o.dot(u)) + v * (q.y - o.dot(v))
	};

	// Work in 2D. Start with the triangle as one convex polygon.
	let mut polys: Vec<Vec<glam::DVec2>> = vec![t.v.iter().map(|&p| to2(p)).collect()];

	for &(p, q) in segments {
		let a = to2(p);
		let b = to2(q);
		let dir = b - a;
		if dir.length_squared() < EPS * EPS {
			continue;
		}
		// UNIT line normal (in-plane); signed distance side = line_n · (x − a).
		// Normalizing is load-bearing: with the raw perp (length = |segment|) the
		// EPS on-line band in `split_convex_by_line` was EPS/|segment| in real
		// distance — a SHORT cut segment (a chord clipped to a sliver of the other
		// operand near a seam corner, often ~1e-5 mm) ballooned the band to ~1e-4,
		// swallowing genuine crossings; the two operands then disagreed about the
		// seam polyline by far more than WELD/TJUNCTION_EPS and the stitch left a
		// micro-triangle hole (the dominant residual fuzz-failure class).
		let line_n = glam::DVec2::new(-dir.y, dir.x).normalize_or_zero();
		if line_n.length_squared() < 0.5 {
			continue;
		}
		let mut next: Vec<Vec<glam::DVec2>> = Vec::with_capacity(polys.len());
		for poly in &polys {
			split_convex_by_line(poly, a, line_n, &mut next);
		}
		polys = next;
	}

	// Fan each convex sub-polygon into triangles.
	for poly in &polys {
		if poly.len() < 3 {
			continue;
		}
		for w in 1..poly.len() - 1 {
			let frag = Tri { v: [to3(poly[0]), to3(poly[w]), to3(poly[w + 1])], normal: n, source: t.source, surface: t.surface };
			if !frag.is_degenerate() {
				out.push(orient_tri(frag, n));
			}
		}
	}
}

/// Split a convex polygon by the line `{x : line_n·(x − a) = 0}` (`line_n` is a
/// UNIT normal) into the pieces on each side, appending the pieces to `out`. A
/// polygon entirely on one side passes through unchanged.
fn split_convex_by_line(poly: &[glam::DVec2], a: glam::DVec2, line_n: glam::DVec2, out: &mut Vec<Vec<glam::DVec2>>) {
	// NB: an absolute EPS (not an exact orient2d sign) is load-bearing here. This
	// polygon lives in f64-*projected* 2-D coordinates, so a vertex that is
	// geometrically on the split line is only approximately on it numerically; the
	// EPS keeps it shared between both sub-polygons (crack-free). Exact orientation
	// on these inexact coordinates would split such a vertex to one side and open a
	// crack on off-axis geometry — the win requires an exact *projection*, not just
	// an exact predicate.
	let dist: Vec<f64> = poly.iter().map(|&p| line_n.dot(p - a)).collect();
	let has_pos = dist.iter().any(|&d| d > EPS);
	let has_neg = dist.iter().any(|&d| d < -EPS);
	if !(has_pos && has_neg) {
		out.push(poly.to_vec()); // does not straddle the line
		return;
	}
	let mut pos: Vec<glam::DVec2> = Vec::new();
	let mut neg: Vec<glam::DVec2> = Vec::new();
	let n = poly.len();
	for i in 0..n {
		let j = (i + 1) % n;
		let (di, dj) = (dist[i], dist[j]);
		let pi = poly[i];
		// Classify current vertex.
		if di >= -EPS {
			pos.push(pi);
		}
		if di <= EPS {
			neg.push(pi);
		}
		// If the edge crosses the line, add the split point to both sides.
		if (di > EPS && dj < -EPS) || (di < -EPS && dj > EPS) {
			let tparam = di / (di - dj);
			let x = pi + (poly[j] - pi) * tparam;
			pos.push(x);
			neg.push(x);
		}
	}
	// Keep BOTH pieces regardless of area. A piece whose area is tiny (a seam-corner
	// micro-triangle where two cut lines cross near a mesh edge) can still have edges
	// ~1e-4 long; discarding it (the old `area > EPS` gate) ripped a hole in the
	// operand's coverage that welding (1e-7) and T-junction healing (4e-7) cannot
	// close — the second residual fuzz-failure mechanism. True zero-extent debris is
	// still pruned downstream: the triangle fan skips `len < 3` polygons and drops
	// `Tri::is_degenerate()` fragments, and welding collapses sub-WELD_EPS pieces.
	if pos.len() >= 3 {
		out.push(pos);
	}
	if neg.len() >= 3 {
		out.push(neg);
	}
}

/// True if `p` lies inside or on triangle `t` (coplanar test).
pub(super) fn point_in_tri_3d(p: DVec3, t: &Tri) -> bool {
	// Scale-invariant degeneracy guard: `normalize_or_zero` yields a unit normal for
	// any non-collapsed triangle (however small its area) and the zero vector only
	// for a genuinely degenerate one — matching the convention used elsewhere here.
	let nn = t.area_vec().normalize_or_zero();
	if nn.length_squared() < 0.5 {
		return false;
	}
	// Must be (near) coplanar.
	if (p - t.v[0]).dot(nn).abs() > 1e-6 {
		return false;
	}
	// EPS slack (not an exact predicate) is load-bearing: `p` is typically an
	// interpolated point, so its incidence with an edge is only approximate. See the
	// note in `split_convex_by_line`.
	for i in 0..3 {
		let a = t.v[i];
		let b = t.v[(i + 1) % 3];
		let c = (b - a).cross(p - a);
		if c.dot(nn) < -EPS {
			return false;
		}
	}
	true
}

/// Re-orient a triangle's winding so its geometric normal agrees with `n`.
fn orient_tri(mut t: Tri, n: DVec3) -> Tri {
	if t.area_vec().dot(n) < 0.0 {
		t.v.swap(1, 2);
	}
	t.normal = n;
	t
}
