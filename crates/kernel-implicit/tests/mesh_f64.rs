// Copyright (c) LMCAD. Licensed under the MIT License.

//! The f64 meshers reach the high-level API: any implicit primitive or CSG `Node`
//! can be meshed in full f64 through `surface_nets_sdf_f64` / `dual_contour_sdf_f64`.

use std::f64::consts::PI;

use kernel_core::{dual_contour_sdf_f64, surface_nets_sdf_f64};
use kernel_implicit::{Cuboid, Cylinder, Node, Sdf, Sphere, Vec3};

#[test]
fn f64_entry_point_meshes_a_sphere_primitive() {
	let s = Node::primitive(Sphere::new(Vec3::ZERO, 8.0));
	let mesh = surface_nets_sdf_f64(&s, s.bounds(), 0.5);
	assert!(mesh.triangle_count() > 500, "should finely mesh the sphere");
	let exact = 4.0 / 3.0 * PI * 8.0f64.powi(3);
	let vol = mesh.signed_volume();
	assert!((vol - exact).abs() / exact < 0.03, "f64 sphere volume {vol} vs {exact}");
}

#[test]
fn f64_entry_point_meshes_a_csg_difference() {
	// A 20³ cube with a radius-4 cylindrical bore drilled through it.
	let cube = Node::primitive(Cuboid::new(Vec3::ZERO, Vec3::splat(10.0)));
	let hole = Node::primitive(Cylinder::new(Vec3::new(0.0, 0.0, -11.0), Vec3::new(0.0, 0.0, 11.0), 4.0));
	let part = cube.difference(hole);

	let sn = surface_nets_sdf_f64(&part, part.bounds(), 0.4);
	let expected = 20.0f64.powi(3) - PI * 4.0f64.powi(2) * 20.0;
	let vol = sn.signed_volume();
	assert!((vol - expected).abs() / expected < 0.05, "f64 CSG volume {vol} vs {expected}");

	// The sharp-feature entry point also meshes the same CSG.
	let dc = dual_contour_sdf_f64(&part, part.bounds(), 0.5);
	assert!(dc.triangle_count() > 200, "dual contour should mesh the part");
}
