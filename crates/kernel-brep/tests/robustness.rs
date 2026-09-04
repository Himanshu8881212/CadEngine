// Copyright (c) LMCAD. Licensed under the MIT License.

//! Robustness tests for the B-rep constructors: degenerate (but ≥3-point)
//! profiles must not panic the builders / tessellator and must not produce
//! NaN/inf vertices. (Correctness is not asserted for degenerate input — only
//! that the kernel stays well-defined.)

use kernel_brep::math::DVec2;
use kernel_brep::{extrude, revolve, tessellate_default};

fn assert_finite_tessellation(solid: &kernel_brep::Solid, what: &str) {
	let mesh = tessellate_default(solid);
	assert!(mesh.positions.iter().all(|p| p.is_finite()), "{what}: non-finite mesh vertex");
	assert!(mesh.normals.iter().all(|n| n.is_finite()), "{what}: non-finite mesh normal");
}

#[test]
fn degenerate_extrude_profiles_do_not_panic_or_nan() {
	let cases: Vec<(&str, Vec<DVec2>)> = vec![
		("collinear (zero area)", vec![DVec2::new(0.0, 0.0), DVec2::new(1.0, 0.0), DVec2::new(2.0, 0.0)]),
		("coincident points", vec![DVec2::new(0.0, 0.0), DVec2::new(0.0, 0.0), DVec2::new(1.0, 1.0)]),
		("near-zero scale", vec![DVec2::new(0.0, 0.0), DVec2::new(1e-9, 0.0), DVec2::new(0.0, 1e-9)]),
		(
			"non-convex with a near-degenerate spike",
			vec![DVec2::new(0.0, 0.0), DVec2::new(4.0, 0.0), DVec2::new(4.0, 4.0), DVec2::new(2.0, 0.001), DVec2::new(0.0, 4.0)],
		),
	];
	for (what, profile) in cases {
		// Must not panic during construction or tessellation.
		let solid = extrude(&profile, 2.0);
		assert_finite_tessellation(&solid, what);
	}
}

#[test]
fn degenerate_revolve_profiles_do_not_panic_or_nan() {
	let cases: Vec<(&str, Vec<DVec2>)> = vec![
		("zero-thickness ring (collinear)", vec![DVec2::new(5.0, 0.0), DVec2::new(5.0, 1.0), DVec2::new(5.0, 2.0)]),
		("profile through the axis", vec![DVec2::new(0.0, 0.0), DVec2::new(3.0, 0.0), DVec2::new(0.0, 0.0)]),
		("tiny ring", vec![DVec2::new(1.0, 0.0), DVec2::new(1.0 + 1e-9, 0.0), DVec2::new(1.0, 1e-9)]),
	];
	for (what, profile) in cases {
		let solid = revolve(&profile, 12);
		assert_finite_tessellation(&solid, what);
	}
}
