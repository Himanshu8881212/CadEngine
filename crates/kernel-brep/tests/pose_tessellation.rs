//! Regression: rigid transforms must not degrade the exact tessellation.
//!
//! Friction folding_book_stand F4 (2026-08-27): exporting a POSED solid
//! (rotation −118° about x, and +30°) demoted to the voxel heal — and for a
//! large body that heal ground for many minutes — while the identical solid
//! unposed (or posed by −118.5°, −90°, −45°, −10°) exported on the exact
//! route. A rigid isometry cannot change whether a tessellation is
//! manufacturing-ready, so whatever term of that predicate flips under
//! specific rotations is measuring an artifact of the check, not the
//! geometry. This test pins the invariant: for every probed angle, the exact
//! adaptive tessellation of the rotated solid must agree with the unrotated
//! one on watertightness, degeneracy, and the self-intersection witness.

use kernel_brep::{difference, extrude, tessellate_adaptive_tol};
use kernel_core::check_mesh;
use kernel_core::math::{DAffine3, DVec2, DVec3};

/// The H1 bore-knuckle profile from the folding_book_stand campaign — the
/// smallest real geometry that reproduced the demotion.
fn knuckle() -> kernel_brep::Solid {
	// verbatim from school_system/folding_book_stand/programs/stand.json
	// (ops p_k1_p / p_d1_p) — the exact geometry that reproduced the demotion
	let profile: Vec<DVec2> = [
		(-6.7, 0.0),
		(-4.3, 0.0),
		(0.35, 5.5335),
		(0.35, 15.8394),
		(-1.6964, 17.1568),
		(-3.9372, 17.8633),
		(-6.2844, 17.9658),
		(-8.5782, 17.4572),
		(-12.5, 14.6569),
		(-12.5, 10.0),
		(-13.5, 10.0),
		(-13.5, 5.12),
		(-11.0, 5.12),
	]
	.iter()
	.map(|&(x, y)| DVec2::new(x, y))
	.collect();
	let bore: Vec<DVec2> = [
		(-9.1373, 11.1),
		(-9.6362, 9.7293),
		(-9.6362, 8.2707),
		(-9.1373, 6.9),
		(-8.1997, 5.7826),
		(-6.9365, 5.0533),
		(-5.5, 4.8),
		(-4.0635, 5.0533),
		(-2.8003, 5.7826),
		(-1.8627, 6.9),
		(-1.3638, 8.2707),
		(-1.3638, 9.7293),
		(-1.8627, 11.1),
		(-2.8003, 12.2174),
		(-5.5, 15.534),
	]
	.iter()
	.map(|&(x, y)| DVec2::new(x, y))
	.collect();
	let body = extrude(&profile, 13.0);
	let mut cutter = extrude(&bore, 15.0);
	cutter = cutter.transformed(DAffine3::from_translation(DVec3::new(0.0, 0.0, -1.0)));
	difference(&body, &cutter)
}

#[derive(Debug, PartialEq)]
struct Readiness {
	watertight: bool,
	degenerate: usize,
	self_intersecting: bool,
}

fn readiness(solid: &kernel_brep::Solid, tol: f64) -> Readiness {
	let mesh = tessellate_adaptive_tol(solid, tol);
	let report = check_mesh(&mesh);
	Readiness {
		watertight: report.watertight,
		degenerate: report.degenerate_triangles,
		self_intersecting: mesh.self_intersection_witness().is_some(),
	}
}

#[test]
fn rigid_rotation_preserves_manufacturing_readiness() {
	let solid = knuckle();
	let base = readiness(&solid, 0.01);
	assert_eq!(
		base,
		Readiness { watertight: true, degenerate: 0, self_intersecting: false },
		"unposed baseline must be manufacturing-ready"
	);
	// The two angles measured to fail (−118, +30) plus a sweep of neighbours.
	for angle in [-118.0f64, 30.0, -118.5, -90.0, -45.0, -10.0, 63.0, 141.0, -77.0] {
		let m = DAffine3::from_translation(DVec3::new(0.0, -5.5, 9.0))
			* DAffine3::from_rotation_x(angle.to_radians())
			* DAffine3::from_translation(DVec3::new(0.0, 5.5, -9.0));
		let posed = solid.transformed(m);
		let got = readiness(&posed, 0.01);
		assert_eq!(
			got, base,
			"rotation by {angle}° about x changed manufacturing readiness — a rigid isometry \
			 must not (friction folding_book_stand F4)"
		);
	}
}

#[test]
fn no_manufactured_witness_at_minus_118() {
	let solid = knuckle();
	let m = DAffine3::from_translation(DVec3::new(0.0, -5.5, 9.0))
		* DAffine3::from_rotation_x((-118.0f64).to_radians())
		* DAffine3::from_translation(DVec3::new(0.0, 5.5, -9.0));
	let posed = solid.transformed(m);
	let mesh = tessellate_adaptive_tol(&posed, 0.01);
	// The f32 predicate manufactured a crossing (pair [139, 143]) that does
	// not exist in the stored coordinates at double precision; the f64
	// predicate must report none.
	assert!(mesh.self_intersection_witness().is_none(), "the -118 deg pose must not produce a self-intersection witness");
}
