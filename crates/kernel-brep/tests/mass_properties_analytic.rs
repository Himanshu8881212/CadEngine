//! Which primitives have ANALYTIC inertia (independent of tessellation) vs
//! tessellation-level. mass_properties adds closed-form "lens" second-moment
//! corrections for cylinder, cone and sphere faces (validate.rs), so their
//! inertia is exact at ANY facet count for plane/cylinder/cone/sphere AND torus, so
//! their inertia is tessellation-level. The discriminator is coarse-vs-fine
//! agreement of the inertia trace.

use kernel_brep::math::DVec3;
use kernel_brep::{cone, cuboid, cylinder, mass_properties, sphere, torus, Solid};

fn inertia_trace(s: &Solid) -> f64 {
	let m = mass_properties(s);
	m.inertia.x_axis.x + m.inertia.y_axis.y + m.inertia.z_axis.z
}

#[test]
fn inertia_is_analytic_for_quadrics_and_tessellation_level_for_torus() {
	// Analytic: a coarse facet count must give the SAME inertia trace as a fine one.
	let cyl = |seg| cylinder(DVec3::new(0.0, 0.0, -15.0), DVec3::Z, 8.0, 30.0, seg);
	let sph = |lon, lat| sphere(DVec3::ZERO, 10.0, lon, lat);
	let con = |seg| cone(DVec3::new(0.0, 0.0, -10.0), DVec3::Z, 8.0, 20.0, seg);
	let analytic = [
		("cylinder", inertia_trace(&cyl(12)), inertia_trace(&cyl(200))),
		("sphere", inertia_trace(&sph(16, 8)), inertia_trace(&sph(200, 100))),
		("cone", inertia_trace(&con(12)), inertia_trace(&con(200))),
	];
	for (name, coarse, fine) in analytic {
		let rel = (coarse - fine).abs() / fine.abs();
		assert!(
			rel < 1e-5,
			"{name} inertia must be analytic (coarse == fine via lens correction): coarse={coarse} fine={fine} reldiff={rel:.2e}"
		);
	}

	// Cuboid: exact planar integration.
	let c = cuboid(DVec3::new(-5.0, -10.0, -15.0), DVec3::new(5.0, 10.0, 15.0));
	let m = 6000.0;
	let trace = m * ((400.0 + 900.0) + (100.0 + 900.0) + (100.0 + 400.0)) / 12.0;
	assert!((inertia_trace(&c) - trace).abs() / trace < 1e-9, "cuboid inertia must be exact: {} vs {trace}", inertia_trace(&c));

	// Torus: analytically corrected as of the torus_lens_moments landing —
	// coarse now equals fine (the deep verification lives in
	// tests/mass_properties_torus.rs; this keeps the quadric-family assertion
	// that inertia is facet-count-independent for ALL tagged surfaces).
	let tor = |r, t| torus(DVec3::ZERO, DVec3::Z, 12.0, 4.0, r, t);
	let (tc, tf) = (inertia_trace(&tor(12, 8)), inertia_trace(&tor(200, 100)));
	let trel = (tc - tf).abs() / tf.abs();
	assert!(trel < 1e-5, "torus inertia must now be analytic (coarse==fine): coarse={tc} fine={tf} reldiff={trel:.2e}");
}
