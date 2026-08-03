// Copyright (c) LMCAD. Licensed under the MIT License.

//! Property-based (fuzz) tests for the boolean engine — the spec's §7 robustness
//! oracle. Random CSG trees of random primitives must mesh without NaNs and
//! stay watertight; the boolean algebra identities (`A−A=∅`, `A∪A=A`) and
//! rigid-motion invariance must hold under random inputs.

use kernel_core::check_mesh;
use kernel_core::math::Aabb;
use kernel_implicit::{manifold_dual_contour, redistanced, surface_nets, Cuboid, Cylinder, Node, Resolution, Sdf, Sphere, Vec3};
use proptest::prelude::*;

/// A sphere SDF scaled by `k` — a deliberately non-metric field (|∇| = k ≠ 1),
/// used to check that redistancing recovers a true unit-gradient distance field.
struct ScaledSphere {
	center: Vec3,
	radius: f32,
	k: f32,
}

impl Sdf for ScaledSphere {
	fn distance(&self, p: Vec3) -> f32 {
		((p - self.center).length() - self.radius) * self.k
	}
	fn bounds(&self) -> Aabb {
		Aabb::from_center_half_extent(self.center, Vec3::splat(self.radius + 2.0))
	}
}

/// A bounded random point.
fn triple() -> impl Strategy<Value = (f32, f32, f32)> {
	(-8.0f32..8.0, -8.0f32..8.0, -8.0f32..8.0)
}

fn vec3(t: (f32, f32, f32)) -> Vec3 {
	Vec3::new(t.0, t.1, t.2)
}

/// Build a bounded random primitive from a kind selector and two parameter
/// triples (kept well-sized so meshing is cheap and the bound is well-defined).
fn build(kind: u8, p: (f32, f32, f32), q: (f32, f32, f32)) -> Node {
	let c = vec3(p);
	match kind % 3 {
		0 => Node::primitive(Sphere::new(c, 2.0 + q.0.abs() % 5.0)),
		1 => Node::primitive(Cuboid::new(
			c,
			Vec3::new(2.0 + q.0.abs() % 4.0, 2.0 + q.1.abs() % 4.0, 2.0 + q.2.abs() % 4.0),
		)),
		_ => {
			let axis = Vec3::Y;
			let h = 3.0 + q.1.abs() % 6.0;
			Node::primitive(Cylinder::new(c - axis * h, c + axis * h, 2.0 + q.0.abs() % 4.0))
		}
	}
}

fn combine(a: Node, b: Node, op: u8) -> Node {
	match op % 3 {
		0 => a.union(b),
		1 => a.difference(b),
		_ => a.intersection(b),
	}
}

proptest! {
	#![proptest_config(ProptestConfig::with_cases(48))]

	/// Any random depth-2 CSG tree meshes to a finite, **closed** mesh (no
	/// boundary holes) — the invariant naive Surface Nets guarantees at any
	/// resolution. (Full 2-manifoldness also holds once features are adequately
	/// resolved; a handful of non-manifold *edges* can remain when a feature is
	/// sub-cell — see the mesher docs — so we assert closedness here, not the
	/// resolution-dependent manifold-edge property.)
	#[test]
	fn random_csg_meshes_are_closed_and_finite(
		k1 in 0u8..3, p1 in triple(), q1 in triple(),
		k2 in 0u8..3, p2 in triple(), q2 in triple(),
		k3 in 0u8..3, p3 in triple(), q3 in triple(),
		op1 in 0u8..3, op2 in 0u8..3,
	) {
		let tree = combine(combine(build(k1, p1, q1), build(k2, p2, q2), op1), build(k3, p3, q3), op2);
		let b = tree.bounds();
		prop_assume!(b.is_valid() && b.size().min_element() > 0.1 && b.size().max_element() < 80.0);
		let mesh = surface_nets(&tree, b, Resolution::VoxelSize(0.8));
		prop_assert!(mesh.positions.iter().all(|p| p.is_finite()), "mesh has non-finite vertices");
		let report = check_mesh(&mesh);
		prop_assert!(mesh.is_empty() || report.boundary_edges == 0, "CSG mesh has {} open boundary edges (a hole)", report.boundary_edges);
	}

	/// `make_manifold` is a safe topological repair: for ANY random CSG it must
	/// (a) keep the mesh closed (no boundary opened), (b) preserve the volume
	/// exactly (only connectivity changes), and (c) never increase the
	/// non-manifold count — fully resolving the separable cases while leaving
	/// connected pinches untouched. (Full 2-manifoldness for connected pinches
	/// needs source-level Manifold Dual Contouring, tracked separately.)
	#[test]
	fn make_manifold_is_safe_and_monotone(
		k1 in 0u8..3, p1 in triple(), q1 in triple(),
		k2 in 0u8..3, p2 in triple(), q2 in triple(),
		op in 0u8..3,
	) {
		let tree = combine(build(k1, p1, q1), build(k2, p2, q2), op);
		let b = tree.bounds();
		prop_assume!(b.is_valid() && b.size().min_element() > 0.3 && b.size().max_element() < 60.0);
		let mesh = surface_nets(&tree, b, Resolution::VoxelSize(0.4));
		prop_assume!(!mesh.is_empty());
		let v0 = mesh.signed_volume();
		let before = check_mesh(&mesh);

		let repaired = kernel_core::make_manifold(&mesh);
		let after = check_mesh(&repaired);

		// Closedness preserved (the dual-mesher output has no boundary).
		prop_assert_eq!(after.boundary_edges, 0, "repair opened a boundary (hole)");
		// Volume preserved (geometry unchanged).
		prop_assert!((repaired.signed_volume() - v0).abs() / v0.abs() < 1e-4, "repair changed the volume");
		// Monotone: never worse on either non-manifold count.
		prop_assert!(after.non_manifold_edges <= before.non_manifold_edges, "repair increased non-manifold edges");
		prop_assert!(after.non_manifold_vertices <= before.non_manifold_vertices, "repair increased non-manifold vertices");
		// And it must never introduce non-orientable (flipped) adjacency — the
		// corruption class the previous radial-pairing version could produce.
		prop_assert!(after.non_orientable_edges <= before.non_orientable_edges, "repair introduced non-orientable adjacency");
	}

	/// Manifold Dual Contouring is closed, finite, correctly oriented, and **no
	/// worse than naive Surface Nets** on non-manifold edges for ANY random CSG —
	/// and empirically fully 2-manifold across the vast majority of cases at
	/// typical resolution (it resolves the connected-pinch body-saddle the naive
	/// meshers cannot). A complete guarantee at every resolution has a rare
	/// residual; for guaranteed-clean output compose with `make_manifold`.
	#[test]
	fn manifold_dc_is_closed_and_no_worse_than_naive(
		k1 in 0u8..3, p1 in triple(), q1 in triple(),
		k2 in 0u8..3, p2 in triple(), q2 in triple(),
		op in 0u8..3,
	) {
		let tree = combine(build(k1, p1, q1), build(k2, p2, q2), op);
		let b = tree.bounds();
		prop_assume!(b.is_valid() && b.size().min_element() > 0.3 && b.size().max_element() < 60.0);
		let mdc = manifold_dual_contour(&tree, b, Resolution::VoxelSize(0.5));
		prop_assume!(!mdc.is_empty());
		let naive = surface_nets(&tree, b, Resolution::VoxelSize(0.5));
		let (rm, rn) = (check_mesh(&mdc), check_mesh(&naive));
		prop_assert!(mdc.positions.iter().all(|p| p.is_finite()), "non-finite vertex");
		prop_assert_eq!(rm.boundary_edges, 0, "MDC must stay closed");
		prop_assert!(rm.non_manifold_edges <= rn.non_manifold_edges, "MDC worse than naive ({} > {})", rm.non_manifold_edges, rn.non_manifold_edges);
		prop_assert!(mdc.signed_volume() > 0.0, "outward orientation");
	}

	/// `A − A = ∅` for any random primitive.
	#[test]
	fn self_difference_is_empty(k in 0u8..3, p in triple(), q in triple()) {
		let b = build(k, p, q).bounds();
		prop_assume!(b.size().min_element() > 0.1);
		let diff = build(k, p, q).difference(build(k, p, q));
		let mesh = surface_nets(&diff, b, Resolution::VoxelSize(0.6));
		prop_assert!(mesh.is_empty() || mesh.signed_volume().abs() < 1.0, "A−A not empty: vol {}", mesh.signed_volume());
	}

	/// `A ∪ A = A` (volume preserved) for any random primitive.
	#[test]
	fn union_with_self_preserves_volume(k in 0u8..3, p in triple(), q in triple()) {
		let b = build(k, p, q).bounds();
		prop_assume!(b.size().min_element() > 0.5 && b.size().max_element() < 60.0);
		let va = surface_nets(&build(k, p, q), b, Resolution::VoxelSize(0.5)).signed_volume();
		prop_assume!(va > 1.0);
		let vu = surface_nets(&build(k, p, q).union(build(k, p, q)), b, Resolution::VoxelSize(0.5)).signed_volume();
		prop_assert!((vu - va).abs() / va < 0.05, "A∪A vol {vu} != A vol {va}");
	}

	/// Redistancing a non-metric (scaled) sphere field recovers the true Euclidean
	/// distance (≈ the offset) and preserves the inside/outside sign, regardless of
	/// the scale `k`. This is what makes offset/shell correct after a boolean.
	#[test]
	fn redistance_recovers_true_distance(
		cx in -4.0f32..4.0, cy in -4.0f32..4.0, cz in -4.0f32..4.0,
		radius in 4.0f32..8.0, k in 1.5f32..4.0,
	) {
		let center = Vec3::new(cx, cy, cz);
		let field = ScaledSphere { center, radius, k };
		let grid = redistanced(&field, field.bounds(), 0.4);
		for &d in &[Vec3::X, Vec3::Y, Vec3::Z, Vec3::new(1.0, 1.0, 1.0).normalize()] {
			let outside = center + d * (radius + 1.0);
			prop_assert!((grid.distance(outside) - 1.0).abs() < 0.7, "redistanced outside dist {} (expect ~1)", grid.distance(outside));
			let inside = center + d * (radius - 1.0);
			prop_assert!(grid.distance(inside) < 0.0, "inside sign must stay negative, got {}", grid.distance(inside));
		}
	}

	/// A rigid translation preserves volume.
	#[test]
	fn translation_preserves_volume(k in 0u8..3, p in triple(), q in triple(), d in triple()) {
		let b = build(k, p, q).bounds();
		prop_assume!(b.size().min_element() > 0.5 && b.size().max_element() < 60.0);
		let v0 = surface_nets(&build(k, p, q), b, Resolution::VoxelSize(0.5)).signed_volume();
		prop_assume!(v0 > 1.0);
		let moved = build(k, p, q).translate(vec3(d));
		let v1 = surface_nets(&moved, moved.bounds(), Resolution::VoxelSize(0.5)).signed_volume();
		prop_assert!((v0 - v1).abs() / v0 < 0.05, "translation changed volume {v0} -> {v1}");
	}
}
