// Copyright (c) LMCAD. Licensed under the MIT License.

//! Signed clearance between two solids, and the mesh **warp bridge** for
//! interference checks on elastically deformed working states.
//!
//! [`Mesh::min_distance`] answers "how far apart are these surfaces?" but
//! returns `0.0` for *both* a coplanar datum contact and a real interference —
//! assembly gates were forced to run boolean-volume checks just to learn the
//! sign. [`Mesh::signed_clearance`] answers the full question in one call:
//! positive separation, ~0 at touch, negative penetration.
//!
//! [`Mesh::warped`] + [`radial_wave_field`] exist because a B-rep cannot
//! represent an elastically deformed working state (the canonical case: a
//! harmonic-drive flexspline pressed into a two-lobe ellipse by its wave
//! generator). Warping the *tessellation* lets the same 3D interference gates
//! run against the deformed geometry.

use crate::bvh::MeshBvh;
use crate::math::{Aabb, Vec3};
use crate::mesh::Mesh;

/// Cap on penetration-depth sample points taken per mesh (vertices and
/// triangle centroids are strided down to this budget each).
const SAMPLE_CAP: usize = 512;

impl Mesh {
	/// Signed clearance between two **closed** meshes, treated as solids:
	///
	/// - **positive** — the solids are disjoint; the value is the minimum
	///   separation, identical to [`min_distance`](Self::min_distance) (found via
	///   the same BVH simultaneous-descent machinery).
	/// - **~0** (within a touch band of `1e-4 ×` the combined bbox diagonal,
	///   floored at `1e-4`) — exact surface contact, e.g. a coplanar datum face.
	/// - **negative** — the solids interpenetrate; the magnitude is an
	///   approximate penetration depth (see below). This includes full
	///   containment of one solid inside the other, which `min_distance` cannot
	///   even detect (nested disjoint surfaces report a positive gap).
	///
	/// Why it exists: boolean-volume gates were previously needed just to
	/// distinguish "coplanar datum contact" from "real interference", because
	/// `min_distance` returns `0.000` for both. This query replaces that
	/// pattern with one call whose *sign* is the answer.
	///
	/// **Penetration convention.** The magnitude is the *deepest-sample escape
	/// distance*: the maximum, over sampled surface points of either mesh that
	/// lie strictly inside the other solid, of that point's distance to the
	/// other solid's surface. Any separating translation must move the deepest
	/// trapped surface point at least that far, so the value is a lower bound
	/// on the true (minimum-translation) penetration depth; with the
	/// vertex + triangle-centroid + intersecting-pair-midpoint sampling used
	/// here it lands within ~20% of truth for ordinarily tessellated parts
	/// (exact for axis-aligned box-box overlap). For full containment this
	/// convention yields the *deepest* contained point's outward escape
	/// distance — for a centred part that equals the uniform surface-to-surface
	/// gap; for an off-centre part it exceeds the minimal gap (it reports how
	/// trapped the part is, not how close it is to a wall).
	///
	/// Sampled candidates: strided vertices and triangle centroids of each mesh
	/// that lie inside the other (ray-parity containment, majority of three
	/// non-axis-aligned rays via [`MeshBvh::contains_point`]), plus centroid
	/// midpoints of intersecting triangle pairs found by
	/// [`MeshBvh::intersecting_triangle_pairs`].
	///
	/// Preconditions and honesty: the inside/outside classification assumes
	/// both meshes are closed 2-manifolds ([`is_watertight`](Self::is_watertight));
	/// for open shells only the positive (separation) branch is meaningful.
	/// Penetrations smaller than the touch band are reported as `0.0` (they are
	/// indistinguishable from contact at tessellation precision). Returns
	/// [`f64::INFINITY`] if either mesh is empty, mirroring `min_distance`.
	pub fn signed_clearance(&self, other: &Mesh) -> f64 {
		if self.indices.is_empty() || other.indices.is_empty() {
			return f64::INFINITY;
		}
		let bvh_a = self.build_bvh();
		let bvh_b = other.build_bvh();
		let sep = bvh_a.min_distance(&bvh_b);

		let (box_a, box_b) = (self.aabb(), other.aabb());
		let touch = 1e-4 * (box_a.union(box_b).diagonal() as f64).max(1.0);

		// Deepest sampled surface point of one mesh trapped inside the other.
		let mut depth = max_depth_inside(&surface_samples(self), box_b, &bvh_b)
			.max(max_depth_inside(&surface_samples(other), box_a, &bvh_a));

		if sep > touch {
			// Surfaces are separated: either truly disjoint (positive) or one
			// solid is nested inside the other (negative, depth ≥ sep).
			return if depth > 0.0 { -depth } else { sep };
		}

		// Surfaces touch or cross: add centroid midpoints of intersecting
		// triangle pairs — they sit on/near the intersection zone and catch
		// crossings whose vertices all lie outside (e.g. two thin rods in an X).
		// A midpoint is a penetration witness ONLY if it lies inside BOTH
		// solids (Möller counts mere touching contact as intersecting, and a
		// midpoint of two touching triangles can land deep inside one solid
		// while being outside the other — that is contact, not penetration).
		// For a point in the intersection volume, its distance to EITHER
		// surface is a valid lower bound on the separating translation, so
		// take the larger.
		let pairs = bvh_a.intersecting_triangle_pairs(&bvh_b);
		let stride = (pairs.len() / SAMPLE_CAP).max(1);
		for &(i, j) in pairs.iter().step_by(stride) {
			let m = (triangle_centroid(self, i) + triangle_centroid(other, j)) * 0.5;
			if box_a.contains(m) && box_b.contains(m) && bvh_a.contains_point(m) && bvh_b.contains_point(m) {
				if let (Some(ca), Some(cb)) = (bvh_a.closest_point(m), bvh_b.closest_point(m)) {
					depth = depth.max((ca.distance as f64).max(cb.distance as f64));
				}
			}
		}

		if depth > touch {
			-depth
		} else {
			0.0
		}
	}

	/// Apply an arbitrary displacement `field` to every vertex and return the
	/// warped mesh: `p ↦ p + field(p)`. Indices (topology) are unchanged and
	/// nothing else is recomputed — in particular the stored `normals` are
	/// copied verbatim and are stale for a non-rigid field; call
	/// [`compute_normals`](Self::compute_normals) on the result if you need
	/// them. Because connectivity is untouched, a watertight input stays
	/// watertight (the field may of course create self-intersections if it
	/// folds the surface through itself — warp fields are expected to be small
	/// elastic displacements).
	///
	/// Why it exists: a B-rep cannot represent an elastically deformed working
	/// state (the harmonic-drive flexspline flexed into its two-lobe ellipse is
	/// the canonical case). This bridge deforms the *tessellation*, so gates
	/// like [`signed_clearance`](Self::signed_clearance) can run true 3D
	/// interference checks on the deformed geometry. See [`radial_wave_field`]
	/// for the standard strain-wave displacement.
	pub fn warped(&self, field: impl Fn(Vec3) -> Vec3) -> Mesh {
		let mut out = self.clone();
		for p in out.positions.iter_mut() {
			*p += field(*p);
		}
		out
	}
}

/// The standard inextensible-ring strain-wave displacement field about the Z
/// axis, for [`Mesh::warped`]: at azimuth `φ = atan2(y, x)`,
///
/// - radial: `w(φ) = w0 · cos(lobes · (φ − theta))`
/// - tangential: `v(φ) = −(w0 / lobes) · sin(lobes · (φ − theta))`
///
/// applied in the XY plane (`z` unchanged), where `theta` is the wave-generator
/// rotation angle. The tangential term is the classic inextensibility
/// correction `dv/dφ = −w`, which keeps the deformed ring's circumference
/// first-order constant — with `lobes = 2` this is the two-lobe elliptical
/// deformation a wave generator imposes on a harmonic-drive flexspline
/// (`v = −(w0/2)·sin 2(φ − theta)`). Points on the Z axis (and a `lobes` of 0,
/// which is meaningless) get zero displacement so cap-fan apex vertices are
/// left alone.
pub fn radial_wave_field(w0: f32, lobes: u32, theta: f32) -> impl Fn(Vec3) -> Vec3 {
	move |p: Vec3| {
		if lobes == 0 || (p.x * p.x + p.y * p.y) <= f32::EPSILON {
			return Vec3::ZERO;
		}
		let phi = p.y.atan2(p.x);
		let arg = lobes as f32 * (phi - theta);
		let w = w0 * arg.cos();
		let v = -(w0 / lobes as f32) * arg.sin();
		let (s, c) = phi.sin_cos();
		// w·ê_r + v·ê_t with ê_r = (c, s, 0), ê_t = (−s, c, 0).
		Vec3::new(w * c - v * s, w * s + v * c, 0.0)
	}
}

/// Strided surface sample points of `m`: up to [`SAMPLE_CAP`] vertices plus up
/// to [`SAMPLE_CAP`] triangle centroids (centroids catch face-interior depth
/// that corner vertices miss, e.g. the face centre of a box overlap).
fn surface_samples(m: &Mesh) -> Vec<Vec3> {
	let mut pts = Vec::new();
	let vstride = (m.positions.len() / SAMPLE_CAP).max(1);
	pts.extend(m.positions.iter().step_by(vstride).copied());
	let tri_count = m.indices.len() / 3;
	let tstride = (tri_count / SAMPLE_CAP).max(1);
	for ti in (0..tri_count).step_by(tstride) {
		pts.push(triangle_centroid(m, ti));
	}
	pts
}

/// Centroid of mesh triangle `ti`.
fn triangle_centroid(m: &Mesh, ti: usize) -> Vec3 {
	let t = &m.indices[3 * ti..3 * ti + 3];
	(m.positions[t[0] as usize] + m.positions[t[1] as usize] + m.positions[t[2] as usize]) / 3.0
}

/// Maximum, over `samples` that lie inside the solid bounded by `other`, of the
/// sample's distance to `other`'s surface (`0.0` if none are inside).
/// `other_box` is `other`'s bounding box, used as a cheap pre-filter so
/// clearly-outside samples skip the parity test.
fn max_depth_inside(samples: &[Vec3], other_box: Aabb, other: &MeshBvh) -> f64 {
	let mut depth = 0.0f64;
	for &p in samples {
		if !other_box.contains(p) || !other.contains_point(p) {
			continue;
		}
		if let Some(cp) = other.closest_point(p) {
			depth = depth.max(cp.distance as f64);
		}
	}
	depth
}
