// Copyright (c) LMCAD. Licensed under the MIT License.

//! Mesh↔mesh measurement: exact minimum distance, exact proper-crossing,
//! and banded radial extents. Split out of mesh.rs (2026-07-28); the
//! triangle feature-distance helpers live here with their only callers.

use super::*;

/// Shortest distance between two segments `p1q1` and `p2q2` (Ericson, clamped
/// closest points on segments).
pub(crate) fn segment_segment_distance(p1: Vec3, q1: Vec3, p2: Vec3, q2: Vec3) -> f32 {
	let (d1, d2, r) = (q1 - p1, q2 - p2, p1 - p2);
	let (a, e, f) = (d1.dot(d1), d2.dot(d2), d2.dot(r));
	let eps = 1e-12f32;
	let (s, t);
	if a <= eps && e <= eps {
		return r.length();
	}
	if a <= eps {
		s = 0.0;
		t = (f / e).clamp(0.0, 1.0);
	} else {
		let c = d1.dot(r);
		if e <= eps {
			t = 0.0;
			s = (-c / a).clamp(0.0, 1.0);
		} else {
			let b = d1.dot(d2);
			let denom = a * e - b * b;
			let s0 = if denom != 0.0 { ((b * f - c * e) / denom).clamp(0.0, 1.0) } else { 0.0 };
			let t0 = (b * s0 + f) / e;
			if t0 < 0.0 {
				t = 0.0;
				s = (-c / a).clamp(0.0, 1.0);
			} else if t0 > 1.0 {
				t = 1.0;
				s = ((b - c) / a).clamp(0.0, 1.0);
			} else {
				t = t0;
				s = s0;
			}
		}
	}
	((p1 + d1 * s) - (p2 + d2 * t)).length()
}

/// Is this triangle degenerate — zero area to within its own scale? Compared
/// RELATIVE to the longest edge squared so the test is scale-invariant: a
/// 0.001 mm triangle is not degenerate just because it is small, and a 100 mm
/// sliver is degenerate when its width is below float resolution.
pub(crate) fn triangle_is_degenerate(t: [Vec3; 3]) -> bool {
	let (e0, e1, e2) = (t[1] - t[0], t[2] - t[1], t[0] - t[2]);
	let longest = e0.length_squared().max(e1.length_squared()).max(e2.length_squared());
	if longest <= 0.0 {
		return true; // all three corners coincide
	}
	// |cross| = 2·area; compare against the longest edge so the ratio is a
	// WIDTH (area / base) relative to the triangle's own size.
	let twice_area = e0.cross(-e2).length();
	twice_area <= 1e-7 * longest
}

/// Exact minimum distance between two triangles: `0` if they intersect, else the
/// least of the six vertex–face and nine edge–edge feature distances.
///
/// **Degenerate operands never claim an intersection.** A zero-area triangle has
/// no interior to be pierced, and the Möller triangle–triangle predicate is
/// undefined on it: its plane normal is the zero vector, so the plane-side
/// rejection cannot fire and the "planes are parallel" branch is entered with a
/// null normal, which reports an intersection against essentially anything that
/// straddles the sliver. Tessellations legitimately contain such slivers (a
/// subdivided seam whose interior points are collinear), and a single one used
/// to collapse the WHOLE clearance query to `0.0` — a real 0.30 mm gap read as
/// contact (campaign/friction/ENGINE.md, campaign theme T5). Degenerate triangles are
/// therefore routed straight to the feature distances, where the edge–edge terms
/// measure the segment they actually are. Nothing is lost for interference
/// detection: a sliver's edges are shared with non-degenerate neighbours in any
/// surface mesh, so a real penetration through the sliver is still reported as
/// `0` by those neighbouring pairs.
pub(crate) fn triangle_triangle_distance(a: [Vec3; 3], b: [Vec3; 3]) -> f32 {
	if !triangle_is_degenerate(a) && !triangle_is_degenerate(b) && crate::meshcheck::tri_tri_intersect(a, b) {
		return 0.0;
	}
	let mut m = f32::INFINITY;
	for &v in &a {
		m = m.min((closest_point_on_triangle(v, b[0], b[1], b[2]) - v).length());
	}
	for &v in &b {
		m = m.min((closest_point_on_triangle(v, a[0], a[1], a[2]) - v).length());
	}
	for i in 0..3 {
		for j in 0..3 {
			m = m.min(segment_segment_distance(a[i], a[(i + 1) % 3], b[j], b[(j + 1) % 3]));
		}
	}
	m
}

impl Mesh {
	/// Minimum separation between this surface and `other` — the clearance between
	/// two parts. `0.0` when they touch or interfere (a true triangle–triangle
	/// intersection test catches penetration); positive otherwise. Returns
	/// [`f64::INFINITY`] if either mesh is empty.
	///
	/// Computed exactly from triangle–triangle feature distances (vertex–face and
	/// edge–edge), seeded by vertex sampling and pruned by per-triangle bounding
	/// boxes against the running best. The pruning makes typical part pairs fast,
	/// but the worst case is quadratic in the triangle counts — for very large
	/// meshes, decimate first or section to the region of interest.
	pub fn min_distance(&self, other: &Mesh) -> f64 {
		if self.indices.is_empty() || other.indices.is_empty() {
			return f64::INFINITY;
		}
		let tris = |m: &Mesh| -> Vec<[Vec3; 3]> {
			m.indices
				.chunks_exact(3)
				.map(|t| [m.positions[t[0] as usize], m.positions[t[1] as usize], m.positions[t[2] as usize]])
				.collect()
		};
		let (ta, tb) = (tris(self), tris(other));
		let boxa: Vec<Aabb> = ta.iter().map(|t| Aabb::from_points(t)).collect();
		let boxb: Vec<Aabb> = tb.iter().map(|t| Aabb::from_points(t)).collect();

		// Seed an upper bound by sampling each mesh's vertices against the other.
		let mut best = f64::INFINITY;
		for (src, dst) in [(self, other), (other, self)] {
			let stride = (src.positions.len() / 64).max(1);
			let mut i = 0;
			while i < src.positions.len() {
				if let Some(cp) = dst.closest_point(src.positions[i]) {
					best = best.min(cp.distance as f64);
				}
				i += stride;
			}
		}

		// Exact refinement with bounding-box pruning against the running best.
		for (i, a) in ta.iter().enumerate() {
			for (j, b) in tb.iter().enumerate() {
				if (boxa[i].distance_squared_box(boxb[j]) as f64) >= best * best {
					continue;
				}
				let d = triangle_triangle_distance(*a, *b) as f64;
				if d < best {
					best = d;
					if best <= 0.0 {
						return 0.0;
					}
				}
			}
		}
		best
	}
}

/// WHERE a mesh crosses itself — the actionable half of the self-intersection
/// flag. `triangles` are the two triangle indices, `point` is the pierce point
/// (where an edge of one triangle passes through the other's interior), and
/// `pairs` is how many crossing pairs exist in total.
#[derive(Clone, Copy, Debug)]
pub struct SelfIntersection {
	pub triangles: [usize; 2],
	pub point: Vec3,
	pub pairs: usize,
}

/// Does segment `o`→`e` pierce the INTERIOR of triangle `abc`, and where?
///
/// Möller–Trumbore with epsilon margins that exclude edge and vertex grazes, so
/// only a proper crossing counts. This is the predicate behind
/// [`Mesh::has_self_intersection`]; it is spelled out here because that one
/// answers only yes/no and a bare yes on a validity flag is not actionable.
/// `tests/self_intersection_witness.rs` in `kernel-api` pins the two to the same
/// answer so they cannot drift apart.
fn segment_pierces_triangle(o: Vec3, e: Vec3, a: Vec3, b: Vec3, c: Vec3) -> Option<Vec3> {
	// Evaluated in f64 (the house f32-store / f64-compute doctrine): at f32 a
	// rotated mesh's last-ulp coordinate noise manufactured crossings that do
	// not exist in the stored coordinates — a posed solid's export was demoted
	// to the voxel heal by a witness that vanished at double precision
	// (friction folding_book_stand F4, 2026-08-27).
	let (o, e, a, b, c) = (o.as_dvec3(), e.as_dvec3(), a.as_dvec3(), b.as_dvec3(), c.as_dvec3());
	let eps = 1e-9_f64;
	let dir = e - o;
	let ab = b - a;
	let ac = c - a;
	let pv = dir.cross(ac);
	let det = ab.dot(pv);
	// Parallel guard, scale-aware: det = ±(n·dir) with n = ab×ac, so compare
	// against the magnitudes rather than an absolute volume epsilon.
	if det.abs() < 1e-12 * ab.cross(ac).length() * dir.length() {
		return None; // segment parallel to the triangle plane
	}
	let inv = 1.0 / det;
	let tv = o - a;
	let u = tv.dot(pv) * inv;
	if u <= eps || u >= 1.0 - eps {
		return None;
	}
	let qv = tv.cross(ab);
	let v = dir.dot(qv) * inv;
	if v <= eps || u + v >= 1.0 - eps {
		return None;
	}
	let t = ac.dot(qv) * inv;
	if t > eps && t < 1.0 - eps {
		Some((o + dir * t).as_vec3())
	} else {
		None
	}
}

/// The first proper crossing between two triangles, as a point, or `None`.
fn triangles_cross_at(t1: [Vec3; 3], t2: [Vec3; 3]) -> Option<Vec3> {
	for i in 0..3 {
		if let Some(p) = segment_pierces_triangle(t1[i], t1[(i + 1) % 3], t2[0], t2[1], t2[2]) {
			return Some(p);
		}
	}
	for i in 0..3 {
		if let Some(p) = segment_pierces_triangle(t2[i], t2[(i + 1) % 3], t1[0], t1[1], t1[2]) {
			return Some(p);
		}
	}
	None
}

impl Mesh {
	/// Locate a proper self-intersection: the same predicate as
	/// [`Mesh::has_self_intersection`], but reporting WHERE.
	///
	/// A validity flag that only says "something is wrong somewhere" trains its
	/// readers to ignore it — which is exactly what happened to
	/// `validate.geometric_ok` across three campaigns (theme T15). The witness
	/// turns it back into a defect report: two triangle indices and a point to
	/// look at.
	///
	/// Deterministic and independent of traversal order: the reported witness is
	/// always the lexicographically lowest crossing pair `(a, b)`, and `pairs`
	/// counts them all. Candidate pairs come from a sweep-and-prune along the
	/// mesh's longest axis, so a clean mesh costs ~O(T log T), not O(T²).
	pub fn self_intersection_witness(&self) -> Option<SelfIntersection> {
		let n = self.triangle_count();
		if n < 2 {
			return None;
		}
		let tris: Vec<[u32; 3]> = self.triangles().collect();
		let verts: Vec<[Vec3; 3]> =
			tris.iter().map(|t| [self.positions[t[0] as usize], self.positions[t[1] as usize], self.positions[t[2] as usize]]).collect();
		let bounds: Vec<Aabb> = verts.iter().map(|t| Aabb::from_points(t)).collect();

		// Sweep along the widest axis so the active interval stays short.
		let extent = self.aabb().size();
		let axis = if extent.x >= extent.y && extent.x >= extent.z {
			0
		} else if extent.y >= extent.z {
			1
		} else {
			2
		};
		let lo = |i: usize| -> f32 { [bounds[i].min.x, bounds[i].min.y, bounds[i].min.z][axis] };
		let hi = |i: usize| -> f32 { [bounds[i].max.x, bounds[i].max.y, bounds[i].max.z][axis] };
		let mut order: Vec<usize> = (0..n).collect();
		// Ties broken by index so the sweep is byte-reproducible.
		order.sort_by(|&i, &j| lo(i).total_cmp(&lo(j)).then(i.cmp(&j)));

		let mut first: Option<SelfIntersection> = None;
		let mut pairs = 0usize;
		let mut active: Vec<usize> = Vec::new();
		for &t in &order {
			active.retain(|&a| hi(a) >= lo(t));
			for &a in &active {
				let (x, y) = if a < t { (a, t) } else { (t, a) };
				// Adjacent triangles are EXPECTED to touch along shared edges.
				if tris[x].iter().any(|v| tris[y].contains(v)) {
					continue;
				}
				if bounds[x].distance_squared_box(bounds[y]) > 0.0 {
					continue; // boxes disjoint ⇒ triangles cannot cross
				}
				if let Some(p) = triangles_cross_at(verts[x], verts[y]) {
					pairs += 1;
					if first.is_none_or(|w| [x, y] < w.triangles) {
						first = Some(SelfIntersection { triangles: [x, y], point: p, pairs: 0 });
					}
				}
			}
			active.push(t);
		}
		first.map(|w| SelfIntersection { pairs, ..w })
	}

	/// Exact **radial extent** `(min, max)` of the surface about an axis,
	/// optionally restricted to a slab `band = (t0, t1)` of axial coordinate
	/// (`t = (p − axis_point)·axis_dir`). Returns `None` when no surface lies
	/// in the band.
	///
	/// Correct where vertex scanning is not: triangles are CLIPPED to the band
	/// first (a cuboid's silhouette has no vertices mid-height — the RESPOOL
	/// campaign hit exactly that measuring rib envelopes), the maximum is
	/// taken over clipped-polygon vertices (exact: radius is convex over a
	/// planar region, so its max sits at an extreme point), and the minimum
	/// additionally checks axis-through-polygon piercing (→ 0), the
	/// parallel-plane interior foot, and line↔edge distances — a box face's
	/// closest point to a centered axis is mid-face, on no vertex or edge.
	pub fn radial_extent(&self, axis_point: Vec3, axis_dir: Vec3, band: Option<(f32, f32)>) -> Option<(f64, f64)> {
		let o = axis_point.as_dvec3();
		let d = axis_dir.as_dvec3().normalize_or_zero();
		if d.length_squared() < 0.5 {
			return None;
		}
		let radial = |p: crate::math::DVec3| {
			let v = p - o;
			(v - d * v.dot(d)).length()
		};
		let (t0, t1) = match band {
			Some((a, b)) => (a.min(b) as f64, a.max(b) as f64),
			None => (f64::NEG_INFINITY, f64::INFINITY),
		};
		let mut rmin = f64::INFINITY;
		let mut rmax = f64::NEG_INFINITY;
		for t in self.indices.chunks_exact(3) {
			let tri = [
				self.positions[t[0] as usize].as_dvec3(),
				self.positions[t[1] as usize].as_dvec3(),
				self.positions[t[2] as usize].as_dvec3(),
			];
			// clip the triangle to the axial slab (Sutherland–Hodgman, 2 planes)
			let mut poly: Vec<crate::math::DVec3> = tri.to_vec();
			for (sign, lim) in [(1.0f64, t0), (-1.0, t1)] {
				// keep points with sign*(t(p) - lim) >= 0
				let side = |p: crate::math::DVec3| sign * ((p - o).dot(d) - lim);
				let mut next: Vec<crate::math::DVec3> = Vec::with_capacity(poly.len() + 2);
				for i in 0..poly.len() {
					let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
					let (sa, sb) = (side(a), side(b));
					if sa >= 0.0 {
						next.push(a);
					}
					if (sa > 0.0 && sb < 0.0) || (sa < 0.0 && sb > 0.0) {
						next.push(a + (b - a) * (sa / (sa - sb)));
					}
				}
				poly = next;
				if poly.is_empty() {
					break;
				}
			}
			if poly.len() < 3 {
				if let Some(&p) = poly.first() {
					// degenerate sliver: still count its points
					rmax = rmax.max(radial(p));
					rmin = rmin.min(poly.iter().map(|&q| radial(q)).fold(f64::INFINITY, f64::min));
				}
				continue;
			}
			// max: exact at clipped-polygon vertices
			for &p in &poly {
				rmax = rmax.max(radial(p));
				rmin = rmin.min(radial(p));
			}
			// min: piercing / parallel-interior / edge cases
			let n = (poly[1] - poly[0]).cross(poly[2] - poly[0]);
			let nn = n.normalize_or_zero();
			if nn.length_squared() > 0.5 {
				let nd = nn.dot(d);
				// 2D containment test in the polygon's plane basis
				let u = (poly[1] - poly[0]).normalize_or_zero();
				let v = nn.cross(u);
				let to2 = |p: crate::math::DVec3| ((p - poly[0]).dot(u), (p - poly[0]).dot(v));
				let inside = |q: (f64, f64)| {
					let mut inside = false;
					for i in 0..poly.len() {
						let (x0, y0) = to2(poly[i]);
						let (x1, y1) = to2(poly[(i + 1) % poly.len()]);
						if (y0 > q.1) != (y1 > q.1) {
							let xi = x0 + (q.1 - y0) / (y1 - y0) * (x1 - x0);
							if xi > q.0 {
								inside = !inside;
							}
						}
					}
					inside
				};
				if nd.abs() > 1e-9 {
					// axis pierces the plane: inside ⇒ the surface touches the axis
					let s = (poly[0] - o).dot(nn) / nd;
					let q = o + d * s;
					if inside(to2(q)) {
						rmin = 0.0;
					}
				} else {
					// axis parallel to the plane: interior foot at plane distance
					let h = ((o - poly[0]).dot(nn)).abs();
					let q = o - nn * (o - poly[0]).dot(nn);
					if inside(to2(q)) {
						rmin = rmin.min(h);
					}
				}
			}
			// line ↔ edge-segment distances
			for i in 0..poly.len() {
				let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
				let e = b - a;
				let w = a - o;
				let (ee, ed, wd, we) = (e.dot(e), e.dot(d), w.dot(d), w.dot(e));
				let denom = ee - ed * ed;
				let s = if denom.abs() > 1e-15 { ((ed * wd - we) / denom).clamp(0.0, 1.0) } else { 0.0 };
				let p = a + e * s;
				rmin = rmin.min(radial(p));
			}
		}
		if rmax.is_finite() {
			Some((rmin, rmax))
		} else {
			None
		}
	}
}

impl Mesh {
	/// Exact **proper-crossing** test between two meshes: `true` iff any
	/// triangle of `self` properly intersects a triangle of `other`
	/// (touching/kissing and coplanar overlap do NOT count — same predicate as
	/// [`has_self_intersection`](Self::has_self_intersection)). This is the
	/// oracle vertex-sampled penetration cannot fake: a thin wall crossing a
	/// plate with no vertices inside each other reads penetration 0.0 but
	/// crosses — the miss that hid a real slider-through-parapet collision
	/// (DRYBOX 2026-07-28). AABB-pruned per pair; worst case quadratic.
	pub fn crosses_mesh(&self, other: &Mesh) -> bool {
		if self.indices.is_empty() || other.indices.is_empty() {
			return false;
		}
		let tris = |m: &Mesh| -> Vec<[Vec3; 3]> {
			m.indices
				.chunks_exact(3)
				.map(|t| [m.positions[t[0] as usize], m.positions[t[1] as usize], m.positions[t[2] as usize]])
				.collect()
		};
		let (ta, tb) = (tris(self), tris(other));
		let boxa: Vec<Aabb> = ta.iter().map(|t| Aabb::from_points(t)).collect();
		let boxb: Vec<Aabb> = tb.iter().map(|t| Aabb::from_points(t)).collect();
		for (i, a) in ta.iter().enumerate() {
			for (j, b) in tb.iter().enumerate() {
				if boxa[i].distance_squared_box(boxb[j]) > 0.0 {
					continue;
				}
				if crate::meshcheck::tri_tri_intersect(*a, *b) {
					return true;
				}
			}
		}
		false
	}
}

impl Mesh {
	/// This mesh with every vertex mapped through `t` (f64 affine; positions
	/// stay f32). The posed-mesh idiom every campaign example re-implemented.
	pub fn transformed_by(&self, t: crate::math::DAffine3) -> Mesh {
		let mut out = self.clone();
		for p in &mut out.positions {
			let q = t.transform_point3(crate::math::DVec3::new(p.x as f64, p.y as f64, p.z as f64));
			*p = Vec3::new(q.x as f32, q.y as f32, q.z as f32);
		}
		out
	}

	/// Append `other`'s triangles (indices re-based). Scene assembly for
	/// merged ASSEMBLY.stl exports — the other half of the campaign idiom.
	pub fn append(&mut self, other: &Mesh) {
		let base = self.positions.len() as u32;
		self.positions.extend_from_slice(&other.positions);
		self.indices.extend(other.indices.iter().map(|i| i + base));
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	fn tri(a: [f32; 3], b: [f32; 3], c: [f32; 3]) -> [Vec3; 3] {
		[Vec3::from_array(a), Vec3::from_array(b), Vec3::from_array(c)]
	}

	/// The T5 root cause, in isolation: a ZERO-AREA triangle (three collinear
	/// points, which subdivided seams legitimately produce) used to make the
	/// Möller triangle–triangle predicate claim an intersection against anything
	/// straddling it, collapsing the measured distance to 0. A real 0.30 mm gap
	/// read as contact because of one sliver.
	#[test]
	fn a_degenerate_triangle_never_fakes_contact() {
		// A vertical sliver at x = 0, spanning z ∈ [0, 10].
		let sliver = tri([0.0, 0.0, 0.0], [0.0, 0.0, 5.0], [0.0, 0.0, 10.0]);
		assert!(triangle_is_degenerate(sliver));
		// A real triangle 2 mm away in x, straddling the sliver in z.
		let real = tri([2.0, -3.0, 5.0], [2.0, 3.0, 5.0], [2.0, 0.0, 8.0]);
		assert!(!triangle_is_degenerate(real));
		let d = triangle_triangle_distance(sliver, real);
		assert!((d - 2.0).abs() < 1e-4, "the sliver's true distance is 2 mm, got {d}");
	}

	/// The guard must not blind the predicate: two genuinely crossing triangles
	/// still measure 0.
	#[test]
	fn genuinely_crossing_triangles_still_measure_zero() {
		let a = tri([-5.0, 0.0, 0.0], [5.0, 0.0, 0.0], [0.0, 0.0, 5.0]);
		let b = tri([0.0, -5.0, 1.0], [0.0, 5.0, 1.0], [0.0, 0.0, 4.0]);
		assert_eq!(triangle_triangle_distance(a, b), 0.0);
	}

	/// Degeneracy is judged RELATIVE to the triangle's own size, so a legitimately
	/// tiny facet is not mistaken for a sliver.
	#[test]
	fn degeneracy_is_scale_invariant() {
		let tiny = tri([0.0, 0.0, 0.0], [1e-3, 0.0, 0.0], [0.0, 1e-3, 0.0]);
		assert!(!triangle_is_degenerate(tiny), "a small but well-formed facet is not degenerate");
		let huge_sliver = tri([0.0, 0.0, 0.0], [100.0, 0.0, 0.0], [50.0, 1e-9, 0.0]);
		assert!(triangle_is_degenerate(huge_sliver), "a 100 mm edge with 1e-9 mm width is degenerate");
	}
}
