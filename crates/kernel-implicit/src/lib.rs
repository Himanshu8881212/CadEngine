// Copyright (c) LMCAD. Licensed under the MIT License.

//! `kernel-implicit` — the implicit / voxel half of the hybrid kernel.
//!
//! Analytic [`primitives`] (each an [`Sdf`](kernel_core::Sdf)) plus a CSG
//! [`Node`] tree whose booleans are `min`/`max` on signed distances. Hand the
//! whole tree to a mesher (`kernel_core::surface_nets`) to get a watertight mesh.

pub mod dual_contour;
pub mod expr_sdf;
pub mod fast_winding;
pub mod features;
pub mod grid;
pub mod grid_field;
pub mod lattice;
pub mod manifold_dc;
pub mod mesh_boolean;
pub mod meshsdf;
pub mod narrow_band;
pub mod ops;
pub mod primitives;
pub mod redistance;
pub mod sparse;
pub mod strut;
pub mod text;
pub mod texture;
pub mod voronoi;

pub use dual_contour::dual_contour;
pub use expr_sdf::{scalar_field, Expr, ExprSdf};
pub use fast_winding::FastWindingSdf;
pub use features::{chamfer_difference, chamfer_union, fillet_difference, fillet_union, metaballs};
pub use grid::{SparseGrid, VoxelGrid};
pub use lattice::{BeamLattice, LatticeCell, Pipe, VoronoiLattice};
pub use manifold_dc::manifold_dual_contour;
pub use mesh_boolean::{mesh_boolean_implicit, BoolOp};
pub use meshsdf::MeshSdf;
pub use narrow_band::{dual_contour_narrowband, surface_nets_narrowband};
pub use ops::{FieldQuality, Node, OffsetOutcome, ScalarField, Xform};
pub use primitives::{Capsule, Cone, Cuboid, Cylinder, Gyroid, Plane, Sphere, Torus, Tpms, TpmsKind};
pub use redistance::{redistance, redistanced};

// Re-export the core meshing entry points for convenience.
pub use kernel_core::{check_mesh, make_manifold, surface_nets, Aabb, Mesh, MeshReport, Resolution, Sdf, Vec3};

#[cfg(test)]
mod tests {
	use super::*;
	use kernel_core::math::Affine3A;

	fn vol(node: &Node, vs: f32) -> f64 {
		surface_nets(node, node.bounds(), Resolution::VoxelSize(vs)).signed_volume()
	}

	#[test]
	fn f64_distance_is_correct_and_precise_at_large_scale() {
		use kernel_core::math::DVec3;
		// 1) At human scale the f64 evaluation must AGREE with the f32 one — proving
		// the f64 formulas are correct, not merely higher-precision. Exercises the
		// Cuboid, Sphere and the difference combinator in f64.
		let part = Node::primitive(Cuboid::new(Vec3::ZERO, Vec3::splat(10.0)))
			.difference(Node::primitive(Sphere::new(Vec3::new(0.0, 0.0, 12.0), 5.0)));
		for p in [Vec3::new(3.0, 4.0, 0.0), Vec3::new(0.0, 0.0, 9.0), Vec3::new(20.0, 0.0, 0.0)] {
			let (a, b) = (part.distance(p) as f64, part.distance64(p.as_dvec3()));
			assert!((a - b).abs() < 1e-4, "f64 {b} disagrees with f32 {a} at {p:?}");
		}
		// 2) On a metre-scale part the f32 path suffers catastrophic cancellation and
		// rounds a 0.03 mm-outside point ONTO the surface; the f64 path recovers it.
		let s = Node::primitive(Sphere::new(Vec3::ZERO, 1.0e6));
		let d64 = s.distance64(DVec3::new(1.0e6 + 0.03, 0.0, 0.0));
		assert!((d64 - 0.03).abs() < 1e-4, "f64 should recover 0.03, got {d64}");
		let d32 = s.distance(Vec3::new(1.0e6 + 0.03, 0.0, 0.0));
		assert!(d32.abs() < 1e-4, "f32 rounds the point onto the surface (~0), got {d32}");
	}

	#[test]
	fn linear_pattern_replicates_volume() {
		// Four Ø6 spheres spaced 10 mm apart (disjoint) → 4× the single volume.
		let v1 = vol(&Node::primitive(Sphere::new(Vec3::ZERO, 3.0)), 0.2);
		let row = Node::primitive(Sphere::new(Vec3::ZERO, 3.0)).linear_pattern(Vec3::new(10.0, 0.0, 0.0), 4);
		let v4 = vol(&row, 0.2);
		assert!((v4 - 4.0 * v1).abs() / (4.0 * v1) < 0.03, "4-pattern vol {v4} vs {}", 4.0 * v1);
		assert!(row.bounds().max.x > 28.0, "pattern bounds must span all copies, got {:?}", row.bounds());
	}

	#[test]
	fn circular_pattern_makes_a_ring() {
		// Six Ø4 spheres on a ring of radius 12 (a full 2π/6 step) — disjoint → 6×.
		let v1 = vol(&Node::primitive(Sphere::new(Vec3::new(12.0, 0.0, 0.0), 2.0)), 0.15);
		let ring = Node::primitive(Sphere::new(Vec3::new(12.0, 0.0, 0.0), 2.0)).circular_pattern(
			Vec3::ZERO,
			Vec3::Z,
			std::f32::consts::TAU / 6.0,
			6,
		);
		let v6 = vol(&ring, 0.15);
		assert!((v6 - 6.0 * v1).abs() / (6.0 * v1) < 0.05, "6-ring vol {v6} vs {}", 6.0 * v1);
		// The ring wraps to the −x side too.
		assert!(ring.bounds().min.x < -10.0, "ring should wrap to −x, bounds {:?}", ring.bounds());
	}

	#[test]
	fn mirror_doubles_an_offset_shape() {
		// A sphere at +x mirrored across the yz-plane → two disjoint spheres.
		let v1 = vol(&Node::primitive(Sphere::new(Vec3::new(8.0, 0.0, 0.0), 3.0)), 0.2);
		let sym = Node::primitive(Sphere::new(Vec3::new(8.0, 0.0, 0.0), 3.0)).mirror(Vec3::ZERO, Vec3::X);
		let v2 = vol(&sym, 0.2);
		assert!((v2 - 2.0 * v1).abs() / (2.0 * v1) < 0.03, "mirrored vol {v2} vs {}", 2.0 * v1);
		assert!(sym.bounds().min.x < -4.0, "mirror must extend to −x, bounds {:?}", sym.bounds());
	}

	#[test]
	fn audit_regressions_primitive_and_pattern_robustness() {
		use kernel_core::math::DVec3;
		// (#2/#4/#11) A near-degenerate axis must put distance and distance64 on the
		// SAME branch — they previously disagreed (1e-12 vs 1e-18) and flipped sign.
		let p = Vec3::new(0.5, 0.0, 0.0);
		let cyl = Cylinder::new(Vec3::ZERO, Vec3::new(1e-7, 0.0, 0.0), 1.0);
		assert_eq!(
			cyl.distance(p).signum() as f64,
			cyl.distance64(p.as_dvec3()).signum(),
			"cylinder branch disagrees near a degenerate axis"
		);
		let cone = Cone::new(Vec3::ZERO, Vec3::new(1e-7, 0.0, 0.0), 1.0, 1.0);
		assert_eq!(
			cone.distance(p).signum() as f64,
			cone.distance64(p.as_dvec3()).signum(),
			"cone branch disagrees near a degenerate axis"
		);
		// (#12/#14) Gyroid with zero frequency must be finite, not NaN poisoning the grid.
		let g = Gyroid::new(Aabb::from_center_half_extent(Vec3::ZERO, Vec3::splat(5.0)), 0.0, 0.1);
		assert!(g.distance(Vec3::new(1.0, 2.0, 3.0)).is_finite(), "gyroid scale=0 produced non-finite");
		assert!(g.distance64(DVec3::new(1.0, 2.0, 3.0)).is_finite(), "gyroid64 scale=0 produced non-finite");
		// (#5) A degenerate polar-pattern axis must still bound the replicated geometry.
		let ring = Node::primitive(Sphere::new(Vec3::new(5.0, 0.0, 0.0), 1.0)).circular_pattern(Vec3::ZERO, Vec3::ZERO, 1.0, 4);
		let b = ring.bounds();
		assert!(b.is_valid() && b.max.x >= 4.0, "degenerate-axis polar bounds do not contain the geometry: {b:?}");
	}

	#[test]
	fn f64_distance_covers_every_primitive() {
		// Every analytic primitive's f64 distance must AGREE with its f32 distance at
		// human scale — the f64 formula is correct, not merely more precise. Covers
		// the primitives beyond the core four (Cone, Torus, Capsule, Gyroid).
		let prims: [Node; 4] = [
			Node::primitive(Cone::new(Vec3::new(0.0, 0.0, -5.0), Vec3::new(0.0, 0.0, 5.0), 6.0, 2.0)),
			Node::primitive(Torus::new(Vec3::ZERO, Vec3::Z, 8.0, 3.0)),
			Node::primitive(Capsule::new(Vec3::new(-5.0, 0.0, 0.0), Vec3::new(5.0, 0.0, 0.0), 2.0)),
			Node::primitive(Gyroid::new(Aabb::from_center_half_extent(Vec3::ZERO, Vec3::splat(10.0)), 0.3, 0.1)),
		];
		for node in &prims {
			for p in [Vec3::new(3.0, 4.0, 1.0), Vec3::new(-7.0, 2.0, 5.0), Vec3::new(0.5, 0.5, 0.5)] {
				let (a, b) = (node.distance(p) as f64, node.distance64(p.as_dvec3()));
				assert!((a - b).abs() < 1e-4, "f64 {b} disagrees with f32 {a} at {p:?}");
			}
		}
	}

	#[test]
	fn cube_boolean_identities() {
		// A ∪ A = A  and  A − A = ∅
		let a = || Node::primitive(Cuboid::new(Vec3::ZERO, Vec3::splat(10.0)));
		let exact = 20.0f64.powi(3);

		let self_vol = vol(&a(), 0.5);
		assert!((self_vol - exact).abs() / exact < 0.02, "cube vol {self_vol}");

		let union = a().union(a());
		assert!((vol(&union, 0.5) - exact).abs() / exact < 0.02, "A∪A should equal A");

		let diff = a().difference(a());
		let mesh = surface_nets(&diff, a().bounds(), Resolution::VoxelSize(0.5));
		assert!(mesh.is_empty(), "A−A should be empty, got {} tris", mesh.triangle_count());
	}

	#[test]
	fn gyroid_lattice_meshes_a_real_bounded_block() {
		// The signature voxel workflow: a gyroid TPMS shell ∩ a cube → a bounded
		// lattice block, meshed via Manifold Dual Contouring. The mesh is a real,
		// rich, plausibly-sized solid contained in the cube. (HONEST: a TPMS *shell*
		// has thousands of saddle pinches, so it is NOT fully watertight even with
		// MDC — watertightness is proven separately for voxel CSG *solids*, e.g.
		// kernel-model's voxel_path_unions_a_tilted_box_watertight.)
		let half = 20.0;
		let region = Aabb::from_center_half_extent(Vec3::ZERO, Vec3::splat(half));
		let lattice =
			Node::primitive(Gyroid::new(region, 0.35, 0.30)).intersection(Node::primitive(Cuboid::new(Vec3::ZERO, Vec3::splat(half))));
		let mesh = manifold_dual_contour(&lattice, region, Resolution::VoxelSize(0.8));
		let vol = mesh.signed_volume();
		let bb = mesh.aabb();
		let cube_vol = 8.0 * (half as f64).powi(3);
		assert!(
			mesh.triangle_count() > 5000
				&& vol > 0.01 * cube_vol
				&& vol < 0.6 * cube_vol
				&& bb.min.x >= -half - 0.5
				&& bb.max.x <= half + 0.5
				&& bb.min.z >= -half - 0.5
				&& bb.max.z <= half + 0.5,
			"gyroid block must be a rich bounded lattice: tris={} vol={vol} (cube {cube_vol}) bb=({:?}..{:?})",
			mesh.triangle_count(),
			bb.min,
			bb.max
		);
	}

	#[test]
	fn gyroid_lattice_meshes_watertight_at_adequate_resolution() {
		// The thin-shell gyroid in `gyroid_lattice_meshes_a_real_bounded_block` is NOT watertight
		// ONLY because its 0.30 shell is thinner than the 0.8 voxel — under-resolved, the two shell
		// surfaces pinch into non-manifold edges (a genuine geometry collapse that make_manifold
		// cannot repair). When the shell is resolved (voxel ≲ thickness) the SAME MDC pipeline
		// yields a FULLY watertight 2-manifold lattice: a 0.6-thick gyroid block at a 0.8 voxel has
		// zero non-manifold edges and zero boundary edges — a printable TPMS lattice straight from
		// the implicit half, the signature organic workflow done correctly.
		let half = 20.0;
		let region = Aabb::from_center_half_extent(Vec3::ZERO, Vec3::splat(half));
		let lattice =
			Node::primitive(Gyroid::new(region, 0.35, 0.6)).intersection(Node::primitive(Cuboid::new(Vec3::ZERO, Vec3::splat(half))));
		let mesh = manifold_dual_contour(&lattice, region, Resolution::VoxelSize(0.8));
		let r = check_mesh(&mesh);
		let vol = mesh.signed_volume().abs();
		let cube_vol = 8.0 * (half as f64).powi(3);
		assert!(
			mesh.is_watertight()
				&& r.non_manifold_edges == 0
				&& r.boundary_edges == 0
				&& mesh.triangle_count() > 5000
				&& vol > 0.01 * cube_vol
				&& vol < 0.6 * cube_vol,
			"watertight gyroid lattice: wt={} nme={} bnd={} tris={} vol={vol} (cube {cube_vol})",
			mesh.is_watertight(),
			r.non_manifold_edges,
			r.boundary_edges,
			mesh.triangle_count()
		);
	}

	#[test]
	fn difference_removes_volume() {
		// 20mm cube minus a Ø8 cylindrical hole straight through it.
		let cube = Node::primitive(Cuboid::new(Vec3::ZERO, Vec3::splat(10.0)));
		let hole = Node::primitive(Cylinder::new(Vec3::new(0.0, -11.0, 0.0), Vec3::new(0.0, 11.0, 0.0), 4.0));
		let part = cube.difference(hole);
		let v = vol(&part, 0.4);
		let expected = 20.0f64.powi(3) - std::f64::consts::PI * 4.0f64.powi(2) * 20.0;
		assert!((v - expected).abs() / expected < 0.03, "drilled cube vol {v} vs {expected}");
	}

	#[test]
	fn translate_preserves_volume() {
		let s = Node::primitive(Sphere::new(Vec3::ZERO, 8.0));
		let moved = Node::primitive(Sphere::new(Vec3::ZERO, 8.0)).transform(Affine3A::from_translation(Vec3::new(5.0, -3.0, 2.0)));
		let a = vol(&s, 0.3);
		let b = vol(&moved, 0.3);
		assert!((a - b).abs() / a < 0.01, "translation changed volume: {a} vs {b}");
	}

	#[test]
	fn plane_clips_sphere_to_hemisphere() {
		// Sphere ∩ half-space (z <= 0) → hemisphere with exactly half the volume.
		let sphere = Node::primitive(Sphere::new(Vec3::ZERO, 10.0));
		let below = Node::primitive(Plane::new(Vec3::ZERO, Vec3::Z)); // inside where z < 0
		let hemi = sphere.intersection(below);

		// Intersection bounds must be finite (plane reports an infinite box, so the
		// sphere supplies the bound). Meshing must succeed.
		assert!(hemi.bounds().is_valid() && hemi.bounds().min.is_finite());
		let mesh = surface_nets(&hemi, hemi.bounds(), Resolution::VoxelSize(0.3));
		let expected = 0.5 * 4.0 / 3.0 * std::f64::consts::PI * 10.0f64.powi(3);
		let v = mesh.signed_volume();
		assert!((v - expected).abs() / expected < 0.02, "hemisphere vol {v} vs {expected}");
	}

	#[test]
	fn bare_plane_is_not_meshable() {
		// A standalone half-space is unbounded; the mesher must reject it cleanly.
		let plane = Node::primitive(Plane::new(Vec3::ZERO, Vec3::Y));
		let mesh = surface_nets(&plane, plane.bounds(), Resolution::VoxelSize(1.0));
		assert!(mesh.is_empty());
	}

	#[test]
	fn offset_by_grades_a_cylinder_wall() {
		use std::sync::Arc;
		// Field modulation: a solid cylinder minus an inner void shrunk by a
		// z-graded offset → a shell whose wall ramps 2 mm → 4 mm bottom → top.
		// The field gradient is 0.05, so the result is 1.05-Lipschitz; this
		// test meshes DENSE (surface_nets samples every cell — only continuity
		// needed), per the documented offset_by contract.
		let t = |z: f32| 2.0 + 0.05 * z;
		let cyl = || Node::primitive(Cylinder::new(Vec3::ZERO, Vec3::new(0.0, 0.0, 40.0), 10.0));
		let inner = cyl().offset_by(Arc::new(|p: Vec3| -(2.0 + 0.05 * p.z)), 6.0);
		let part = cyl().difference(inner);

		// Pointwise the graded surface is exact: the inner wall sits at radius
		// 10 − t(z) (the cylinder side-wall distance is radial there).
		for z in [5.0f32, 20.0, 35.0] {
			let on = Vec3::new(10.0 - t(z), 0.0, z);
			assert!(part.distance(on).abs() < 1e-5, "inner wall at z={z} should sit at r={}, d={}", 10.0 - t(z), part.distance(on));
		}

		// Meshed: measure the wall with the DFM thickness probe in a band near
		// each end (median over side-wall triangles; |n_z| < 0.5 excludes the
		// caps, voxel 0.3 bounds the tolerance).
		let mesh = surface_nets(&part, part.bounds().pad(0.5), Resolution::VoxelSize(0.3));
		assert!(mesh.is_watertight(), "graded shell must be watertight");
		let th = mesh.wall_thickness(0.5);
		let band_median = |z0: f32, z1: f32| -> f64 {
			let mut vals: Vec<f64> = mesh
				.triangles()
				.enumerate()
				.filter_map(|(i, tri)| {
					let [a, b, c] = [mesh.positions[tri[0] as usize], mesh.positions[tri[1] as usize], mesh.positions[tri[2] as usize]];
					let zc = (a.z + b.z + c.z) / 3.0;
					let n = (b - a).cross(c - a).normalize_or_zero();
					(zc > z0 && zc < z1 && n.z.abs() < 0.5 && th.thickness[i].is_finite()).then_some(th.thickness[i])
				})
				.collect();
			vals.sort_by(|x, y| x.partial_cmp(y).unwrap());
			vals[vals.len() / 2]
		};
		let (lo, hi) = (band_median(4.0, 8.0), band_median(32.0, 36.0));
		assert!(
			(lo - t(6.0) as f64).abs() < 0.25 && (hi - t(34.0) as f64).abs() < 0.25,
			"graded wall: measured {lo:.2} / {hi:.2} mm, field says {:.2} / {:.2} mm",
			t(6.0),
			t(34.0)
		);
	}

	#[test]
	fn lerp_blend_is_exact_at_fixed_weight_and_grades_along_z() {
		use std::sync::Arc;
		// Concentric spheres r6 / r12: at constant weight w the lerp of their
		// exact SDFs is EXACTLY the sphere of radius lerp(6, 12, w) — assert
		// pointwise. A z-graded weight then sweeps the radius 6 → 12: the
		// meshed solid must be watertight, span z −6 … +12 (the two extreme
		// poles, where the clamped weight pins w = 0 / w = 1) and hold a
		// volume strictly between the two spheres'. (Dense meshing — see the
		// lerp Lipschitz contract; here |a−b|·|∇w| = 6/12 = 0.5.)
		let s6 = || Node::primitive(Sphere::new(Vec3::ZERO, 6.0));
		let s12 = || Node::primitive(Sphere::new(Vec3::ZERO, 12.0));
		let half = s6().lerp(s12(), Arc::new(|_| 0.5));
		for p in [Vec3::new(9.0, 0.0, 0.0), Vec3::new(0.0, -2.0, 1.0), Vec3::new(5.0, 8.0, -3.0)] {
			let want = p.length() - 9.0;
			assert!((half.distance(p) - want).abs() < 1e-6, "w=0.5 lerp must be the r=9 sphere: {} vs {want}", half.distance(p));
		}

		let graded = s6().lerp(s12(), Arc::new(|p: Vec3| (p.z + 6.0) / 12.0));
		let mesh = surface_nets(&graded, graded.bounds(), Resolution::VoxelSize(0.25));
		let v = mesh.signed_volume();
		let (v6, v12) = (4.0 / 3.0 * std::f64::consts::PI * 216.0, 4.0 / 3.0 * std::f64::consts::PI * 1728.0);
		let bb = mesh.aabb();
		assert!(
			mesh.is_watertight() && (bb.min.z + 6.0).abs() < 0.3 && (bb.max.z - 12.0).abs() < 0.3 && v > v6 && v < v12,
			"graded blend: watertight={} z-span {:.2}..{:.2} (want -6..12) vol={v:.0} (want between {v6:.0} and {v12:.0})",
			mesh.is_watertight(),
			bb.min.z,
			bb.max.z
		);
	}

	#[test]
	fn smooth_union_volume_at_least_hard_union() {
		// A fillet adds material, so smooth union volume ≥ hard union volume.
		let mk = |smooth: bool| {
			let x = Node::primitive(Cuboid::new(Vec3::new(-6.0, 0.0, 0.0), Vec3::splat(8.0)));
			let y = Node::primitive(Cuboid::new(Vec3::new(6.0, 0.0, 0.0), Vec3::splat(8.0)));
			if smooth {
				x.smooth_union(y, 5.0)
			} else {
				x.union(y)
			}
		};
		let hard = vol(&mk(false), 0.4);
		let soft = vol(&mk(true), 0.4);
		assert!(soft >= hard - 1.0, "smooth union {soft} should be >= hard union {hard}");
	}
}
