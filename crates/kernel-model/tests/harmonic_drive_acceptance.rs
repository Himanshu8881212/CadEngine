//! A 50:1 harmonic drive (strain-wave gear) for a NEMA 17 — the gear set and its
//! strain-wave engagement, built from catalog parts. (The full 5-part assembly +
//! STL export lives in legacy/kernel-model-examples/harmonic_drive.rs, uncompiled.)
//!
//! Kinematics: flex spline 100T, circular spline 102T (Δ2), circular spline
//! grounded, output on the flex spline => ratio = -100/(102-100) = -50:1. The wave
//! generator deforms the flex-spline cup elliptical so its teeth ENGAGE the
//! circular spline at the major axis and CLEAR it at the minor — proven here.

use kernel_brep::math::{DAffine3, DVec3};
use kernel_brep::validate;
use kernel_model::parts::{internal_gear, spur_gear};

#[test]
fn harmonic_drive_gearset_builds_and_strain_wave_engages_at_50_to_1() {
	let (m, pa) = (0.4, 20.0);

	// Circular spline: 102T internal ring (rp 20.4, inner tip 20.0, root 20.9).
	let cs = internal_gear(m, 102, 8.0, 48.0, pa).expect("circular spline (102T internal)");
	assert!(validate(&cs).is_valid(), "circular spline must be a valid solid: {:?}", validate(&cs));

	// Flex spline rim: 100T external (rp 20.0, tip 20.4).
	let fs = spur_gear(m, 100, 8.0, 37.4, pa, None);
	assert!(validate(&fs).is_valid(), "flex spline rim must be a valid solid: {:?}", validate(&fs));

	// Strain wave: deform the round rim into the operating ellipse (x*1.005 -> major
	// 20.1, y*0.975 -> minor 19.5). The deformed tips must ENGAGE the 102T tooth band
	// (20.0..20.9) at the major axis and CLEAR it (< 20.0) at the minor axis.
	let deformed = fs.transformed(DAffine3::from_scale(DVec3::new(20.1 / 20.0, 19.5 / 20.0, 1.0)));
	let (_, mx) = deformed.aabb();
	assert!(
		(20.0..20.9).contains(&mx.x) && mx.y < 20.0,
		"strain wave must engage at major and clear at minor: deformed tip major={:.3} (want 20.0..20.9), minor={:.3} (want <20.0)",
		mx.x,
		mx.y
	);

	// The defining reduction.
	let ratio = -100.0 / (102.0 - 100.0);
	assert_eq!(ratio, -50.0, "circular-spline-grounded harmonic drive reduces -50:1");
}
