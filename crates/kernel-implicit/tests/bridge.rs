// Copyright (c) LMCAD. Licensed under the MIT License.

//! Phase 4 acceptance: the bridge.
//! - mesh → SDF (BVH + generalized winding number) round-trips a B-rep solid.
//! - Dual Contouring reproduces a cube's 8 sharp corners that Surface Nets rounds.

use kernel_brep::math::DVec3 as BDVec3;
use kernel_brep::{cuboid, tessellate_default};
use kernel_implicit::{dual_contour, surface_nets, Cuboid, MeshSdf, Resolution, Sdf, VoxelGrid};
use kernel_core::math::Vec3;

#[test]
fn meshsdf_signs_and_roundtrips_volume() {
	// A B-rep box brought into the implicit world via the mesh→SDF bridge.
	let solid = cuboid(BDVec3::new(-6.0, -4.0, -3.0), BDVec3::new(6.0, 4.0, 3.0));
	let mesh = tessellate_default(&solid);
	let msdf = MeshSdf::new(&mesh);

	// Winding-number sign: deep inside is negative, far outside positive.
	assert!(msdf.distance(Vec3::ZERO) < 0.0, "center should be inside");
	assert!(msdf.distance(Vec3::new(100.0, 0.0, 0.0)) > 0.0, "far point should be outside");
	// Distance magnitude near a face matches the offset (here 1mm outside +X face).
	let d = msdf.distance(Vec3::new(7.0, 0.0, 0.0));
	assert!((d - 1.0).abs() < 0.05, "distance off +X face = {d} (expected ~1)");

	// Voxelize the mesh SDF, then re-mesh: volume must match the box.
	let grid = VoxelGrid::from_sdf(&msdf, msdf.bounds(), 0.3);
	let out = surface_nets(&grid, grid.bounds(), Resolution::VoxelSize(0.3));
	let exact = 12.0 * 8.0 * 6.0;
	assert!(out.is_watertight(), "round-tripped mesh must be watertight");
	assert!((out.signed_volume() - exact).abs() / exact < 0.03, "bridge volume {} vs {exact}", out.signed_volume());
}

#[test]
fn dual_contour_preserves_sharp_corners() {
	let cube = Cuboid::new(Vec3::ZERO, Vec3::splat(10.0));
	let domain = cube.bounds().pad(2.0);
	let vs = 1.0;

	let dc = dual_contour(&cube, domain, Resolution::VoxelSize(vs));
	let sn = surface_nets(&cube, domain, Resolution::VoxelSize(vs));

	assert!(dc.is_watertight(), "DC output must be watertight");

	// Distance from each true corner to the nearest mesh vertex.
	let nearest = |m: &kernel_core::Mesh, c: Vec3| {
		m.positions.iter().map(|&p| (p - c).length()).fold(f32::INFINITY, f32::min)
	};
	let corners = [
		Vec3::new(10.0, 10.0, 10.0),
		Vec3::new(-10.0, 10.0, 10.0),
		Vec3::new(10.0, -10.0, 10.0),
		Vec3::new(10.0, 10.0, -10.0),
		Vec3::new(-10.0, -10.0, -10.0),
	];
	let dc_worst = corners.iter().map(|&c| nearest(&dc, c)).fold(0.0, f32::max);
	let sn_worst = corners.iter().map(|&c| nearest(&sn, c)).fold(0.0, f32::max);

	// Dual Contouring sits a vertex right at each sharp corner; Surface Nets rounds.
	assert!(dc_worst < 0.4 * vs, "DC should reproduce sharp corners (worst {dc_worst})");
	assert!(sn_worst > dc_worst, "Surface Nets should round corners more than DC (sn {sn_worst} vs dc {dc_worst})");

	// And DC volume is accurate.
	let exact = 20.0f64.powi(3);
	assert!((dc.signed_volume() - exact).abs() / exact < 0.02, "DC cube volume {} vs {exact}", dc.signed_volume());
}
