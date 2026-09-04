//! A V-belt pulley with a keyed bore + lightening holes (acceptance), and an
//! honest note on one DEGENERATE input that is gracefully refused.
//!
//! Correction of an earlier mischaracterisation: a realistic keyway is cut INTO
//! the bore — its slot OVERLAPS the bore wall, extending from inside the bore void
//! into the hub. Built that way, the whole pulley (concave V-groove revolve +
//! keyway + a ring of lightening holes) builds cleanly and validly on both a
//! revolve-built and a boolean-built tube. The engine handles real keyed pulleys.
//!
//! The failure first seen was a DEGENERATE input: a keyway slot whose inner face
//! sat EXACTLY tangent to the bore cylinder (slot starting at y = bore radius), so
//! a planar box face merely GRAZES the curved wall — a coincident/tangent-face
//! degeneracy, the same class as a sub-tolerance near-coplanar cut. On a
//! boolean-built tube it fails outright; on a revolve-built disc it survives alone
//! but leaves the topology fragile so a later hole then fails. Either way the
//! checked try_difference REFUSES the invalid result. Any real overlap or
//! clearance avoids it. Not a practical pulley bug.

use kernel_brep::checked::try_difference_diagnosed;
use kernel_brep::math::{DVec2, DVec3};
use kernel_brep::{boolean_hazards, cuboid, cylinder, revolve, try_difference, validate, HazardKind, Solid};

fn pulley_disc() -> Solid {
	let profile = [
		DVec2::new(10.0, 0.0),
		DVec2::new(40.0, 0.0),
		DVec2::new(40.0, 4.0),
		DVec2::new(32.0, 9.0),
		DVec2::new(40.0, 14.0),
		DVec2::new(40.0, 18.0),
		DVec2::new(10.0, 18.0),
	];
	revolve(&profile, 96)
}

#[test]
fn realistic_keyed_pulley_with_lightening_holes_builds_valid() {
	// Keyway overlapping the bore (y from 8, inside the Ø20 bore, out to 13).
	let mut m = try_difference(&pulley_disc(), &cuboid(DVec3::new(-3.0, 8.0, -1.0), DVec3::new(3.0, 13.0, 19.0)))
		.expect("a realistic keyway overlapping the bore must cut cleanly");
	for i in 0..5 {
		let a = std::f64::consts::TAU * i as f64 / 5.0;
		let hole = cylinder(DVec3::new(25.0 * a.cos(), 25.0 * a.sin(), -1.0), DVec3::Z, 4.0, 20.0, 32);
		m = try_difference(&m, &hole).unwrap_or_else(|e| panic!("lightening hole {i} must cut on a realistically-keyed pulley: {e:?}"));
	}
	let v = validate(&m);
	// genus 6 = bored hub (1) + 5 through lightening holes; the keyway notch merges
	// into the bore wall without adding a handle.
	assert!(
		v.closed && v.manifold && v.is_valid() && v.genus == 6,
		"a realistic keyed V-pulley with 5 lightening holes must be a valid genus-6 solid: {v:?}"
	);
}

#[test]
fn keyway_face_exactly_tangent_to_a_cylindrical_wall_is_a_degenerate_coincident_face() {
	// A boolean-built tube (outer cylinder minus inner bore), then a keyway whose
	// inner face is EXACTLY at the bore radius (y = 10) — a planar face tangent to
	// the curved wall. Coincident-face degeneracy: the checked op must refuse,
	// never ship an invalid solid.
	let tube =
		try_difference(&cylinder(DVec3::ZERO, DVec3::Z, 40.0, 18.0, 96), &cylinder(DVec3::new(0.0, 0.0, -1.0), DVec3::Z, 10.0, 20.0, 96))
			.expect("hollow tube");
	let tangent = cuboid(DVec3::new(-3.0, 10.0, -1.0), DVec3::new(3.0, 13.0, 19.0));
	assert!(
		try_difference(&tube, &tangent).is_err(),
		"a keyway face exactly tangent to the bore wall is a coincident-face degeneracy and must be refused (graceful, not an invalid solid)"
	);
	// ...and a real keyway that OVERLAPS the same wall cuts cleanly.
	assert!(
		try_difference(&tube, &cuboid(DVec3::new(-3.0, 8.0, -1.0), DVec3::new(3.0, 13.0, 19.0))).is_ok(),
		"a keyway overlapping the wall (the realistic case) must cut cleanly"
	);
}

/// The tube of the degeneracy repro above, rebuilt for the QoL tests.
fn bored_tube() -> Solid {
	try_difference(&cylinder(DVec3::ZERO, DVec3::Z, 40.0, 18.0, 96), &cylinder(DVec3::new(0.0, 0.0, -1.0), DVec3::Z, 10.0, 20.0, 96))
		.expect("hollow tube")
}

#[test]
fn the_tangent_degeneracy_is_named_by_the_pre_flight_linter_before_the_op_runs() {
	// The refusal above is CORRECT but used to be undiagnosable: you learned
	// only that the difference failed. `boolean_hazards` now classifies this
	// input pattern PRE-FLIGHT — a planar face kissing a cylindrical wall —
	// with the measured tangency gap and the §7.7 remedy, so an authoring loop
	// (or an AI driving the kernel) fixes the placement instead of bisecting.
	let tube = bored_tube();
	let tangent = cuboid(DVec3::new(-3.0, 10.0, -1.0), DVec3::new(3.0, 13.0, 19.0));
	let report = boolean_hazards(&tube, &tangent, 0.05);
	let hit = report.iter().find(|h| h.kind == HazardKind::TangentPlaneOnCylinder);

	// The remedy string is part of the contract: it must name the fix, not just
	// the symptom, and it must ride along on Display.
	let remedy = HazardKind::TangentPlaneOnCylinder.remedy().unwrap_or("");
	let displayed = hit.map(|h| h.to_string()).unwrap_or_default();
	assert!(
		hit.is_some_and(|h| h.separation < 1e-9)
			&& remedy.contains("embed")
			&& remedy.contains("never kiss")
			&& displayed.contains("TangentPlaneOnCylinder")
			&& displayed.contains("remedy"),
		"the exactly-tangent keyway must pre-flight as TangentPlaneOnCylinder with a ~0 gap and an actionable remedy; \
		 got {hit:?} (gap {:?}), remedy {remedy:?}, display {displayed:?}. Full report: {report:?}",
		hit.map(|h| h.separation)
	);

	// It must also fire across the whole KISS BAND, not just at exact tangency
	// (a slot inner face 0.02 outside the bore radius is the same degeneracy
	// with a sliver attached), and stay QUIET for a properly embedded keyway —
	// otherwise the linter would cry wolf on every real design.
	let near = cuboid(DVec3::new(-3.0, 10.02, -1.0), DVec3::new(3.0, 13.0, 19.0));
	let near_hit = boolean_hazards(&tube, &near, 0.05).into_iter().find(|h| h.kind == HazardKind::TangentPlaneOnCylinder);
	let embedded = boolean_hazards(&tube, &cuboid(DVec3::new(-3.0, 8.0, -1.0), DVec3::new(3.0, 13.0, 19.0)), 0.05);
	let embedded_kiss = embedded.iter().filter(|h| h.kind == HazardKind::TangentPlaneOnCylinder).count();
	assert!(
		near_hit.is_some_and(|h| (h.separation - 0.02).abs() < 1e-6) && embedded_kiss == 0,
		"the kiss band must cover near-tangency (0.02 slot → gap {:?}) and stay silent on a properly embedded keyway \
		 (2.0 mm inside the wall → {embedded_kiss} tangency hazards, want 0)",
		near_hit.map(|h| h.separation)
	);
}

#[test]
fn the_refusal_itself_now_carries_the_remedy_hint() {
	// The failure path, made actionable: `try_difference_diagnosed` runs the
	// IDENTICAL boolean, and on refusal enriches the error with the pre-flight
	// hazard most likely implicated. The machine-readable half is untouched
	// (op + Validity), so existing callers keep matching on it; the hint is a
	// documented Display suffix.
	let tube = bored_tube();
	let tangent = cuboid(DVec3::new(-3.0, 10.0, -1.0), DVec3::new(3.0, 13.0, 19.0));
	let refusal = try_difference_diagnosed(&tube, &tangent).expect_err("the tangent keyway must still be refused");
	let line = refusal.to_string();
	assert!(
		refusal.error.op == "difference"
			&& !refusal.error.validity.is_valid()
			&& refusal.hazard.is_some_and(|h| h.kind == HazardKind::TangentPlaneOnCylinder)
			&& line.contains("pre-flight linter implicates")
			&& line.contains("never kiss")
			&& !line.contains('\n'),
		"the enriched refusal must keep the machine-readable error AND name the remedy in one line: op={:?} valid={} \
		 hazard={:?} line={line:?}",
		refusal.error.op,
		refusal.error.validity.is_valid(),
		refusal.hazard.map(|h| h.kind)
	);

	// The success path is untouched: a realistic embedded keyway returns the
	// same solid the strict API returns, with no diagnosis run.
	let real = cuboid(DVec3::new(-3.0, 8.0, -1.0), DVec3::new(3.0, 13.0, 19.0));
	let diagnosed = try_difference_diagnosed(&tube, &real).expect("the realistic keyway must still cut");
	let strict = try_difference(&tube, &real).expect("the realistic keyway must still cut");
	assert!(
		kernel_brep::volume(&diagnosed).to_bits() == kernel_brep::volume(&strict).to_bits(),
		"the diagnosed API must return the strict result bit for bit on success: {} vs {}",
		kernel_brep::volume(&diagnosed),
		kernel_brep::volume(&strict)
	);
}
