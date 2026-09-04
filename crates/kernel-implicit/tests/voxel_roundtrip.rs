// Copyright (c) LMCAD. Licensed under the MIT License.

//! Phase 2 acceptance: round-trip primitive → grid → mesh with bounded error,
//! and confirm the sparse grid agrees with the dense one near the surface while
//! storing far fewer samples (memory scales with surface area).

use kernel_core::math::{Aabb, Vec3};
use kernel_core::sdf::Sdf;
use kernel_implicit::{surface_nets, Resolution, SparseGrid, Sphere, VoxelGrid};

fn sphere() -> Sphere {
	Sphere::new(Vec3::new(1.0, 0.0, -2.0), 10.0)
}

fn domain() -> Aabb {
	Aabb::from_center_half_extent(sphere().center, Vec3::splat(11.0))
}

#[test]
fn dense_grid_interpolates_within_voxel_error() {
	let s = sphere();
	let vs = 0.5;
	let grid = VoxelGrid::from_sdf(&s, domain(), vs);

	// Trilinear interpolation error is O(vs^2) for this smooth field.
	let mut max_err = 0.0f32;
	for i in 0..20 {
		for j in 0..20 {
			let theta = std::f32::consts::TAU * i as f32 / 20.0;
			let phi = std::f32::consts::PI * j as f32 / 20.0;
			let dir = Vec3::new(phi.sin() * theta.cos(), phi.sin() * theta.sin(), phi.cos());
			// Sample just outside the surface, well inside the grid.
			let p = s.center + dir * (s.radius + 1.5);
			max_err = max_err.max((grid.distance(p) - s.distance(p)).abs());
		}
	}
	assert!(max_err < 0.5 * vs, "dense grid interp error {max_err} exceeded half a voxel");
}

#[test]
fn dense_grid_roundtrip_volume() {
	let s = sphere();
	let grid = VoxelGrid::from_sdf(&s, domain(), 0.4);
	let mesh = surface_nets(&grid, grid.bounds(), Resolution::VoxelSize(0.4));
	let exact = 4.0 / 3.0 * std::f64::consts::PI * 10.0f64.powi(3);
	let v = mesh.signed_volume();
	assert!(mesh.is_watertight(), "grid mesh must be watertight");
	assert!((v - exact).abs() / exact < 0.02, "grid-meshed sphere vol {v} vs {exact}");
}

#[test]
fn sparse_grid_matches_dense_and_saves_memory() {
	let s = sphere();
	let vs = 0.4;
	let dense = VoxelGrid::from_sdf(&s, domain(), vs);
	let sparse = SparseGrid::from_sdf(&s, domain(), vs, 3.0);

	// Near-surface agreement between the two representations.
	let mut max_diff = 0.0f32;
	for i in 0..16 {
		for j in 0..16 {
			let theta = std::f32::consts::TAU * i as f32 / 16.0;
			let phi = std::f32::consts::PI * j as f32 / 16.0;
			let dir = Vec3::new(phi.sin() * theta.cos(), phi.sin() * theta.sin(), phi.cos());
			let p = s.center + dir * (s.radius + 0.7);
			max_diff = max_diff.max((sparse.distance(p) - dense.distance(p)).abs());
		}
	}
	assert!(max_diff < 1e-3, "sparse vs dense near surface differ by {max_diff}");

	// Memory: the sparse grid stores only near-surface blocks, far fewer samples.
	assert!(
		sparse.sample_count() < sparse.dense_sample_count() / 2,
		"sparse stored {} of {} dense samples — expected < half",
		sparse.sample_count(),
		sparse.dense_sample_count()
	);

	// And it still meshes to the correct solid.
	let mesh = surface_nets(&sparse, sparse.bounds(), Resolution::VoxelSize(vs));
	let exact = 4.0 / 3.0 * std::f64::consts::PI * 10.0f64.powi(3);
	assert!(mesh.is_watertight());
	assert!((mesh.signed_volume() - exact).abs() / exact < 0.02);
}
