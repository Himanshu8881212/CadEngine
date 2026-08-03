// Copyright (c) LMCAD. Licensed under the MIT License.

//! True constant-radius fillet and chamfer feature operators.
//!
//! The CSG [`Node`](crate::ops::Node) tree only offers a polynomial smooth-min
//! blend (`smooth_union`), which is a *blob*: its blend size depends on the
//! local field magnitude rather than a real geometric radius. This module adds
//! the standard exact-ish edge-treatment operators (after Inigo Quilez):
//!
//! - **Round / fillet union** — `opUnionRound`: rounds the concave seam between
//!   two solids with a quarter-round of radius `r`.
//! - **Chamfer union** — `opUnionChamfer`: bevels the seam with a 45° flat of
//!   width controlled by `r`.
//! - The matching **difference** (subtract) variants, which treat the edge
//!   formed where the cutter `b` carves into `a`.
//!
//! Each operator is a tiny [`Sdf`] wrapper struct holding two boxed children and
//! the radius, plus a precomputed combined bound. Because [`Node`] is itself an
//! [`Sdf`], a whole sub-tree can be boxed as a child here, and the result is
//! re-wrapped as a leaf via [`Node::primitive`]. The whole thing therefore meshes
//! with `kernel_core::surface_nets` exactly like any other node.

use kernel_core::math::{Aabb, DVec3, Vec2, Vec3};
use kernel_core::sdf::Sdf;

use crate::ops::Node;
use crate::primitives::Sphere;

/// Rounded (filleted) union of two distance fields with blend radius `r`
/// (iquilezles `opUnionRound`): `max(r, min(a, b)) - length(max(vec2(r-a, r-b), 0))`.
///
/// This is algebraically the spec's `min(min(a,b), …)` rounding seam written in
/// the numerically robust canonical form: inside the corner region (both fields
/// within `r` of the surface) it carves a quarter-round of radius `r`; outside
/// it collapses exactly to the hard `min(a, b)`. Falls back to a hard `min` when
/// `r <= 0`.
#[inline]
fn fillet_union_dist(da: f32, db: f32, r: f32) -> f32 {
	if r <= 0.0 {
		return da.min(db);
	}
	let u = Vec2::new((r - da).max(0.0), (r - db).max(0.0));
	r.max(da.min(db)) - u.length()
}

/// Chamfered union of two distance fields with bevel parameter `r`.
///
/// `opUnionChamfer`: `min(min(a, b), (a + b - r) * sqrt(0.5))`.
/// Falls back to a hard `min` when `r <= 0`.
#[inline]
fn chamfer_union_dist(da: f32, db: f32, r: f32) -> f32 {
	let hard = da.min(db);
	if r <= 0.0 {
		return hard;
	}
	hard.min((da + db - r) * std::f32::consts::FRAC_1_SQRT_2)
}

/// `f64` mirror of [`fillet_union_dist`].
fn fillet_union_dist64(da: f64, db: f64, r: f64) -> f64 {
	if r <= 0.0 {
		return da.min(db);
	}
	let (ux, uy) = ((r - da).max(0.0), (r - db).max(0.0));
	r.max(da.min(db)) - (ux * ux + uy * uy).sqrt()
}

/// `f64` mirror of [`chamfer_union_dist`].
fn chamfer_union_dist64(da: f64, db: f64, r: f64) -> f64 {
	let hard = da.min(db);
	if r <= 0.0 {
		return hard;
	}
	hard.min((da + db - r) * std::f64::consts::FRAC_1_SQRT_2)
}

/// A fillet/chamfer wrapper that owns its two children and the combined bound.
///
/// `combine` dispatches on `subtract` (union vs. difference) and `chamfer`
/// (round vs. bevel). The difference variants reuse the union combinators via
/// the De Morgan identity `a - b = -((-a) ∪ b)`, so a single pair of seam
/// formulas serves all four operators.
struct Feature {
	a: Box<dyn Sdf>,
	b: Box<dyn Sdf>,
	r: f32,
	bounds: Aabb,
	/// `false` for union variants, `true` for difference.
	subtract: bool,
	/// `false` for round/fillet, `true` for chamfer.
	chamfer: bool,
}

impl Feature {
	#[inline]
	fn seam(&self, da: f32, db: f32) -> f32 {
		if self.chamfer {
			chamfer_union_dist(da, db, self.r)
		} else {
			fillet_union_dist(da, db, self.r)
		}
	}

	#[inline]
	fn combine(&self, da: f32, db: f32) -> f32 {
		if self.subtract {
			// Difference `a - b` is the rounded/chamfered intersection of `a` with
			// the complement of `b`. Via De Morgan that is the negation of the
			// rounded/chamfered union of `-a` and `b`, which keeps the seam on the
			// concave cut edge (matching iquilezles `opDifferenceRound`).
			-self.seam(-da, db)
		} else {
			self.seam(da, db)
		}
	}

	#[inline]
	fn seam64(&self, da: f64, db: f64) -> f64 {
		let r = self.r as f64;
		if self.chamfer {
			chamfer_union_dist64(da, db, r)
		} else {
			fillet_union_dist64(da, db, r)
		}
	}

	#[inline]
	fn combine64(&self, da: f64, db: f64) -> f64 {
		if self.subtract {
			-self.seam64(-da, db)
		} else {
			self.seam64(da, db)
		}
	}
}

impl Sdf for Feature {
	fn distance(&self, p: Vec3) -> f32 {
		self.combine(self.a.distance(p), self.b.distance(p))
	}

	fn distance64(&self, p: DVec3) -> f64 {
		// Thread f64 through the seam so the feature does not silently drop to f32
		// precision inside a larger f64 CSG evaluation.
		self.combine64(self.a.distance64(p), self.b.distance64(p))
	}

	fn bounds(&self) -> Aabb {
		self.bounds
	}
}

/// Build a `Feature` leaf node from two child nodes.
///
/// Bounds: a union variant covers both child bounds padded by `r` (the fillet /
/// chamfer adds material outside the hard union seam). A difference variant is
/// bounded by the first child's bounds padded by `r` (subtracting can only round
/// the edge slightly outward of `a`'s original surface).
fn make_feature(a: Node, b: Node, r: f32, subtract: bool, chamfer: bool) -> Node {
	let r = r.max(0.0);
	let ab = a.bounds();
	let bb = b.bounds();
	let bounds = if subtract { ab.pad(r) } else { ab.union(bb).pad(r) };
	// A fillet/chamfer seam field is a distance BOUND, not an exact SDF (the
	// round/bevel blend understates the distance inside the seam region), so the
	// leaf is tagged `DistanceBound` — a downstream `offset`/`shell` on it is
	// honestly flagged approximate. See the field-quality contract in `ops`.
	Node::primitive_bound(Feature {
		a: Box::new(a),
		b: Box::new(b),
		r,
		bounds,
		subtract,
		chamfer,
	})
}

/// Constant-radius **fillet union** of `a` and `b` (`opUnionRound`).
///
/// Rounds the concave seam between the two solids with a quarter-round of radius
/// `r`, adding material. With `r == 0` this degenerates to a hard boolean union.
pub fn fillet_union(a: Node, b: Node, r: f32) -> Node {
	make_feature(a, b, r, false, false)
}

/// **Chamfer union** of `a` and `b` (`opUnionChamfer`).
///
/// Bevels the seam between the two solids with a 45° flat whose size is set by
/// `r`. With `r == 0` this degenerates to a hard boolean union.
pub fn chamfer_union(a: Node, b: Node, r: f32) -> Node {
	make_feature(a, b, r, false, true)
}

/// Constant-radius **fillet difference**, `a - b`, rounding the cut edge.
///
/// The seam where the cutter `b` meets the surface of `a` is rounded with radius
/// `r`. With `r == 0` this degenerates to a hard boolean difference.
pub fn fillet_difference(a: Node, b: Node, r: f32) -> Node {
	make_feature(a, b, r, true, false)
}

/// **Chamfer difference**, `a - b`, beveling the cut edge.
///
/// The seam where the cutter `b` meets the surface of `a` is chamfered with a
/// 45° flat sized by `r`. With `r == 0` this degenerates to a hard difference.
pub fn chamfer_difference(a: Node, b: Node, r: f32) -> Node {
	make_feature(a, b, r, true, true)
}

/// A **metaball** model: the smooth union of a set of spheres given as `(center, radius)`,
/// blended with radius `k`. The canonical organic-modelling primitive — overlapping balls
/// merge into a single smooth blob that is a watertight *solid* (not a thin shell). `k <= 0`
/// degenerates to a hard union; returns `None` for an empty list.
pub fn metaballs(balls: &[(Vec3, f32)], k: f32) -> Option<Node> {
	let mut iter = balls.iter();
	let &(c0, r0) = iter.next()?;
	let mut node = Node::primitive(Sphere::new(c0, r0));
	for &(c, r) in iter {
		node = node.smooth_union(Node::primitive(Sphere::new(c, r)), k);
	}
	Some(node)
}

#[cfg(test)]
mod tests {
	use super::*;
	use crate::primitives::Cuboid;
	use kernel_core::mesher::{surface_nets, Resolution};
	use kernel_core::sdf::Sdf;

	/// Two unit-ish boxes whose corners touch, used across the seam tests.
	fn two_boxes() -> (Node, Node) {
		let a = Node::primitive(Cuboid::new(Vec3::new(-5.0, 0.0, 0.0), Vec3::splat(6.0)));
		let b = Node::primitive(Cuboid::new(Vec3::new(5.0, 0.0, 0.0), Vec3::splat(6.0)));
		(a, b)
	}

	fn vol(node: &Node, vs: f32) -> f64 {
		surface_nets(node, node.bounds(), Resolution::VoxelSize(vs)).signed_volume()
	}

	#[test]
	fn metaballs_blend_into_a_watertight_organic_solid() {
		// Two overlapping spheres smooth-unioned into one blobby organic solid. The voxel half
		// (Manifold Dual Contouring) must mesh it WATERTIGHT — a true solid blob, unlike a thin
		// TPMS shell — with a volume between one sphere and the two summed (the smooth blend
		// adds a neck, the overlap removes a lens). Proves the kernel does organic SOLIDS.
		let r = 5.0_f32;
		let node = metaballs(&[(Vec3::new(-3.0, 0.0, 0.0), r), (Vec3::new(3.0, 0.0, 0.0), r)], 2.0).expect("non-empty");
		let mesh = crate::manifold_dual_contour(&node, node.bounds(), Resolution::VoxelSize(0.4));
		let vol = mesh.signed_volume().abs();
		let one = 4.0 / 3.0 * std::f64::consts::PI * (r as f64).powi(3);
		assert!(
			mesh.is_watertight() && vol > one && vol < 2.0 * one,
			"metaball blob: watertight={} vol={vol:.0} (want {one:.0} < v < {:.0})",
			mesh.is_watertight(),
			2.0 * one
		);
	}

	#[test]
	fn fillet_union_adds_material_over_hard_union() {
		// A fillet rounds the concave seam outward, so it must have at least as
		// much volume as the hard union (within meshing tolerance).
		let hard = {
			let (a, b) = two_boxes();
			vol(&a.union(b), 0.3)
		};
		let filleted = {
			let (a, b) = two_boxes();
			vol(&fillet_union(a, b, 4.0), 0.3)
		};
		assert!(
			filleted >= hard - 1.0,
			"fillet union {filleted} should be >= hard union {hard}"
		);
		// And the rounding must actually add a meaningful amount of material.
		assert!(
			filleted > hard + 1.0,
			"fillet should add material: {filleted} vs {hard}"
		);
	}

	#[test]
	fn chamfer_differs_from_fillet() {
		// A 45° chamfer and a quarter-round fillet are geometrically distinct, so
		// their volumes must not coincide for a non-trivial radius.
		let filleted = {
			let (a, b) = two_boxes();
			vol(&fillet_union(a, b, 4.0), 0.3)
		};
		let chamfered = {
			let (a, b) = two_boxes();
			vol(&chamfer_union(a, b, 4.0), 0.3)
		};
		assert!(
			(filleted - chamfered).abs() > 1.0,
			"chamfer {chamfered} should differ from fillet {filleted}"
		);
	}

	#[test]
	fn filleted_box_box_is_watertight() {
		// The whole point of an exact-ish field is that it still meshes to a closed
		// manifold surface — and at every resolution, not one lucky voxel size
		// (concave-crease manifoldness is resolution-dependent).
		for vs in [0.25f32, 0.3, 0.4, 0.5] {
			let (a, b) = two_boxes();
			let part = fillet_union(a, b, 4.0);
			let mesh = surface_nets(&part, part.bounds(), Resolution::VoxelSize(vs));
			assert!(!mesh.is_empty(), "filleted union should produce geometry at voxel {vs}");
			assert!(mesh.is_watertight(), "filleted box-box mesh must be watertight at voxel {vs}");
		}
	}

	#[test]
	fn fillet_difference_rounds_cut_edge_and_stays_watertight() {
		// A corner-notch cut: the cutter carves a chunk out of the base. The
		// filleted cut rounds the concave interior edges of the notch, so it must
		// (a) remove material vs. the solid, (b) differ from the hard cut by only a
		// small amount (the rounded edges), and (c) still mesh watertight.
		let base = || Node::primitive(Cuboid::new(Vec3::ZERO, Vec3::splat(10.0)));
		let cutter = || Node::primitive(Cuboid::new(Vec3::new(10.0, 10.0, 0.0), Vec3::splat(6.0)));

		let solid = vol(&base(), 0.4);
		let hard_cut = vol(&base().difference(cutter()), 0.4);
		let part = fillet_difference(base(), cutter(), 3.0);
		let fillet_cut = vol(&part, 0.4);

		assert!(hard_cut < solid, "the cut must remove material: {hard_cut} vs {solid}");
		// The fillet only reworks the cut edge, so the volumes are close but distinct.
		let delta = (fillet_cut - hard_cut).abs();
		assert!(delta > 1.0, "rounding the cut edge should change volume: {fillet_cut} vs {hard_cut}");
		assert!(
			delta < 0.05 * solid,
			"fillet should only rework the edge, not bulk material: {fillet_cut} vs {hard_cut}"
		);

		// Watertight across resolutions, not just one (a single voxel size can pass
		// by luck — concave-crease manifoldness is resolution-dependent).
		for vs in [0.3f32, 0.4, 0.5] {
			let mesh = surface_nets(&part, part.bounds(), Resolution::VoxelSize(vs));
			assert!(mesh.is_watertight(), "filleted difference mesh must be watertight at voxel {vs}");
		}
	}

	#[test]
	fn zero_radius_falls_back_to_hard_boolean() {
		// r == 0 must reproduce the plain min/max booleans pointwise.
		let (a, b) = two_boxes();
		let feat = fillet_union(a, b, 0.0);
		let hard_a = Cuboid::new(Vec3::new(-5.0, 0.0, 0.0), Vec3::splat(6.0));
		let hard_b = Cuboid::new(Vec3::new(5.0, 0.0, 0.0), Vec3::splat(6.0));
		for p in [
			Vec3::new(0.0, 0.0, 0.0),
			Vec3::new(-5.0, 3.0, 1.0),
			Vec3::new(11.0, 0.0, 0.0),
		] {
			let expected = hard_a.distance(p).min(hard_b.distance(p));
			let got = feat.distance(p);
			assert!(
				(got - expected).abs() < 1e-4,
				"r=0 fillet at {p:?}: got {got}, want hard min {expected}"
			);
		}
	}
}
