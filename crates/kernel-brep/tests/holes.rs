// Copyright (c) LMCAD. Licensed under the MIT License.

//! Hole-wizard acceptance: every hole kind lands on a plate/block as a closed,
//! manifold solid of the expected genus, removing exactly the closed-form volume
//! of the faceted cutter. The cutters are 32-gon prisms/pyramids, so the oracle
//! is the inscribed n-gon closed form (exact to boolean tolerance), NOT the πr²
//! limit — comparing against π would only document the faceting gap, not verify
//! the boolean. `volume()` (tessellation) is used rather than `exact_volume()`
//! on purpose: the bore walls carry exact `Surface::Cylinder` tags, so the
//! analytic integral would measure the true cylinder, not the faceted cut.

use std::f64::consts::TAU;

use kernel_brep::math::DVec3;
use kernel_brep::{
	bearing_seat, bearing_spec, bearing_specs, bolt_circle, clearance_hole, counterbore_hole, countersink_hole, cuboid, drill,
	metric_hole_spec, metric_hole_specs, tap_drill_hole, tessellate_default, validate, volume, Fit, HoleDepth, HoleError, Solid,
};

/// Default tool faceting (mirrors `DEFAULT_HOLE_SEGMENTS`, recomputed here as an
/// independent oracle).
const SEG: usize = 32;
/// Booleans are exact for planar facets up to stitching: welding perturbs
/// vertices by ≤1e-7 mm, so each cut can shift the volume by ~(cut surface area
/// ≈ 10³ mm²) × 1e-7 ≈ 1e-4 mm³ (measured: 8e-6 single cut, 8.4e-5 across a
/// six-hole chain). 1e-3 bounds a ≤6-boolean chain with headroom while still
/// pinning the table diameters to ~1e-5 mm through the volume delta.
const VOL_EPS: f64 = 1e-3;

/// Area of the regular `SEG`-gon inscribed in the circle of **diameter** `d` —
/// the exact cross-section of a faceted cutter.
fn ngon_area(d: f64) -> f64 {
	let r = d * 0.5;
	0.5 * SEG as f64 * r * r * (TAU / SEG as f64).sin()
}

/// Height of the 118° drill point below the full-diameter depth (half-angle 59°).
fn tip_h(d: f64) -> f64 {
	d * 0.5 / 59.0_f64.to_radians().tan()
}

/// Extra volume a 90° countersink of surface-Ø `dk` removes beyond an existing
/// Ø `dc` bore: the faceted cone (circumradius `R − z` at depth `z`) minus the
/// bore cylinder, integrated over the frustum depth `(dk − dc)/2`.
fn csk_extra(dk: f64, dc: f64) -> f64 {
	let (big_r, r) = (dk * 0.5, dc * 0.5);
	let tf = big_r - r;
	let c = 0.5 * SEG as f64 * (TAU / SEG as f64).sin();
	c * ((big_r.powi(3) - r.powi(3)) / 3.0 - r * r * tf)
}

/// The standard test plate: 40 × 30 × 12 mm, entry on the top face (z = 12),
/// cutting along −Z.
fn plate() -> Solid {
	cuboid(DVec3::ZERO, DVec3::new(40.0, 30.0, 12.0))
}

const PLATE_T: f64 = 12.0;

fn top(x: f64, y: f64) -> DVec3 {
	DVec3::new(x, y, PLATE_T)
}

#[test]
fn drill_through_is_a_genus1_bore_of_exact_volume() {
	let plate = plate();
	let drilled = drill(&plate, top(20.0, 15.0), -DVec3::Z, 6.0, HoleDepth::Through(PLATE_T), None).unwrap();
	let v = validate(&drilled);
	let removed = volume(&plate) - volume(&drilled);
	let expected = ngon_area(6.0) * PLATE_T;
	assert!(
		v.closed && v.manifold && v.genus == 1 && (removed - expected).abs() < VOL_EPS,
		"Ø6 through-drill must pierce the plate as a valid genus-1 bore removing {expected:.6} mm³: {v:?} removed={removed:.6}"
	);
}

#[test]
fn drill_blind_does_not_pierce_and_matches_cylinder_plus_tip_cone() {
	let plate = plate();
	let drilled = drill(&plate, top(20.0, 15.0), -DVec3::Z, 6.0, HoleDepth::Blind(5.0), None).unwrap();
	let v = validate(&drilled);
	let removed = volume(&plate) - volume(&drilled);
	// Full-diameter cylinder to depth 5 plus the 118° tip pyramid (n-gon cone).
	let expected = ngon_area(6.0) * 5.0 + ngon_area(6.0) * tip_h(6.0) / 3.0;
	assert!(
		v.closed && v.manifold && v.genus == 0 && (removed - expected).abs() < VOL_EPS,
		"Ø6 blind drill (5 deep) must NOT pierce (genus 0) and must remove the cylinder+tip-cone closed form {expected:.6} mm³: {v:?} removed={removed:.6}"
	);
}

#[test]
fn clearance_hole_cuts_iso273_bores_through_all() {
	// M5 across all three fits: ISO 273 gives 5.3 / 5.5 / 5.8.
	for (fit, d) in [(Fit::Close, 5.3), (Fit::Medium, 5.5), (Fit::Coarse, 5.8)] {
		let plate = plate();
		let cut = clearance_hole(&plate, top(20.0, 15.0), -DVec3::Z, 5.0, fit, None).unwrap();
		let v = validate(&cut);
		let removed = volume(&plate) - volume(&cut);
		let expected = ngon_area(d) * PLATE_T;
		assert!(
			v.closed && v.manifold && v.genus == 1 && (removed - expected).abs() < VOL_EPS,
			"M5 {fit:?} clearance hole must be a genus-1 Ø{d} through-bore removing {expected:.6} mm³: {v:?} removed={removed:.6}"
		);
	}
	// Every table size at medium fit cuts a valid through-bore of the table diameter.
	for spec in metric_hole_specs() {
		let plate = plate();
		let cut = clearance_hole(&plate, top(20.0, 15.0), -DVec3::Z, spec.m, Fit::Medium, None).unwrap();
		let v = validate(&cut);
		let removed = volume(&plate) - volume(&cut);
		let expected = ngon_area(spec.clearance[1]) * PLATE_T;
		assert!(
			v.closed && v.manifold && v.genus == 1 && (removed - expected).abs() < VOL_EPS,
			"M{} medium clearance hole must remove {expected:.6} mm³: {v:?} removed={removed:.6}",
			spec.m
		);
	}
}

#[test]
fn counterbore_dims_measured_back_from_the_geometry() {
	// M5 close per DIN 974-1: Ø10 counterbore, 5.8 deep (≥ the 5 mm DIN 912 head),
	// over a Ø5.3 clearance bore — the volume delta pins BOTH diameters and the depth.
	let plate = plate();
	let cut = counterbore_hole(&plate, top(20.0, 15.0), -DVec3::Z, 5.0, Fit::Close, None).unwrap();
	let v = validate(&cut);
	let removed = volume(&plate) - volume(&cut);
	let expected = ngon_area(10.0) * 5.8 + ngon_area(5.3) * (PLATE_T - 5.8);
	assert!(
		v.closed && v.manifold && v.genus == 1 && (removed - expected).abs() < VOL_EPS,
		"M5 counterbore must cut Ø10×5.8 over a Ø5.3 through-bore (DIN 974-1), removing {expected:.6} mm³: {v:?} removed={removed:.6}"
	);
}

#[test]
fn countersink_dims_measured_back_from_the_geometry() {
	// M5 medium per DIN 74-1 form F: 90° sink to Ø12.5 at the surface over a Ø5.5
	// bore — the volume delta pins the sink diameter, angle and bore together.
	let plate = plate();
	let cut = countersink_hole(&plate, top(20.0, 15.0), -DVec3::Z, 5.0, Fit::Medium, None).unwrap();
	let v = validate(&cut);
	let removed = volume(&plate) - volume(&cut);
	let expected = ngon_area(5.5) * PLATE_T + csk_extra(12.5, 5.5);
	assert!(
		v.closed && v.manifold && v.genus == 1 && (removed - expected).abs() < VOL_EPS,
		"M5 countersink must cut a 90° Ø12.5 sink over a Ø5.5 through-bore (DIN 74 F), removing {expected:.6} mm³: {v:?} removed={removed:.6}"
	);
}

#[test]
fn tap_drill_pilot_is_m_minus_coarse_pitch() {
	// M6×1 blind → Ø5 pilot with a 118° tip; M8×1.25 through → the exact Ø6.75
	// (the chart's 6.8 is a rounding of d − pitch).
	let plate = plate();
	let blind = tap_drill_hole(&plate, top(12.0, 15.0), -DVec3::Z, 6.0, HoleDepth::Blind(6.0), None).unwrap();
	let v_blind = validate(&blind);
	let removed_blind = volume(&plate) - volume(&blind);
	let expected_blind = ngon_area(5.0) * 6.0 + ngon_area(5.0) * tip_h(5.0) / 3.0;

	let through = tap_drill_hole(&plate, top(28.0, 15.0), -DVec3::Z, 8.0, HoleDepth::Through(PLATE_T), None).unwrap();
	let v_through = validate(&through);
	let removed_through = volume(&plate) - volume(&through);
	let expected_through = ngon_area(6.75) * PLATE_T;

	assert!(
		v_blind.closed && v_blind.manifold && v_blind.genus == 0 && (removed_blind - expected_blind).abs() < VOL_EPS
			&& v_through.closed && v_through.manifold && v_through.genus == 1
			&& (removed_through - expected_through).abs() < VOL_EPS,
		"tap pilots must cut m − pitch: M6 blind Ø5 ({expected_blind:.6} mm³, got {removed_blind:.6}, {v_blind:?}); M8 through Ø6.75 ({expected_through:.6} mm³, got {removed_through:.6}, {v_through:?})"
	);
}

#[test]
fn bolt_circle_of_six_clearance_holes_is_genus_6() {
	let plate = cuboid(DVec3::ZERO, DVec3::new(80.0, 80.0, 6.0));
	let center = DVec3::new(40.0, 40.0, 6.0);
	let cut =
		bolt_circle(&plate, center, -DVec3::Z, 50.0, 6, 0.0, |s, p| clearance_hole(&s, p, -DVec3::Z, 5.0, Fit::Medium, None)).unwrap();
	let v = validate(&cut);
	let removed = volume(&plate) - volume(&cut);
	let expected = 6.0 * ngon_area(5.5) * 6.0;
	assert!(
		v.closed && v.manifold && v.genus == 6 && (removed - expected).abs() < VOL_EPS,
		"six M5 clearance holes on a Ø50 BCD must yield a valid genus-6 plate removing {expected:.6} mm³: {v:?} removed={removed:.6}"
	);
}

#[test]
fn chained_mixed_holes_on_one_face_stay_valid() {
	// Four different hole kinds chained into the SAME top face (the R2-fixed
	// boolean case): counterbore + countersink + blind tap pilot + plain drill.
	let plate = cuboid(DVec3::ZERO, DVec3::new(60.0, 40.0, 12.0));
	let axis = -DVec3::Z;
	let s = counterbore_hole(&plate, DVec3::new(12.0, 10.0, 12.0), axis, 5.0, Fit::Close, None).unwrap();
	let s = countersink_hole(&s, DVec3::new(30.0, 10.0, 12.0), axis, 4.0, Fit::Medium, None).unwrap();
	let s = tap_drill_hole(&s, DVec3::new(48.0, 10.0, 12.0), axis, 6.0, HoleDepth::Blind(6.0), None).unwrap();
	let s = drill(&s, DVec3::new(30.0, 30.0, 12.0), axis, 3.0, HoleDepth::Through(12.0), None).unwrap();

	let v = validate(&s);
	let removed = volume(&plate) - volume(&s);
	let expected = (ngon_area(10.0) * 5.8 + ngon_area(5.3) * (12.0 - 5.8)) // M5 counterbore
		+ (ngon_area(4.5) * 12.0 + csk_extra(10.0, 4.5)) // M4 countersink
		+ (ngon_area(5.0) * 6.0 + ngon_area(5.0) * tip_h(5.0) / 3.0) // M6 tap pilot, blind
		+ ngon_area(3.0) * 12.0; // Ø3 drill
	let watertight = tessellate_default(&s).is_watertight();
	assert!(
		v.closed && v.manifold && v.genus == 3 && watertight && (removed - expected).abs() < VOL_EPS,
		"cbore+csk+tap+drill chained on one face must stay a valid genus-3 watertight solid removing {expected:.6} mm³: {v:?} watertight={watertight} removed={removed:.6}"
	);
}

#[test]
fn bearing_seat_608_cuts_pocket_plus_shoulder_bore() {
	// 608: 8×22×7 → Ø22×7 pocket and a Ø(8+22)/2 = Ø15 shoulder bore through.
	let block = cuboid(DVec3::ZERO, DVec3::new(40.0, 40.0, 14.0));
	let seat = bearing_seat(&block, DVec3::new(20.0, 20.0, 14.0), -DVec3::Z, "608", None).unwrap();
	let v = validate(&seat);
	let removed = volume(&block) - volume(&seat);
	let expected = ngon_area(22.0) * 7.0 + ngon_area(15.0) * (14.0 - 7.0);
	assert!(
		v.closed && v.manifold && v.genus == 1 && (removed - expected).abs() < VOL_EPS,
		"a 608 bearing seat must cut a Ø22×7 pocket plus Ø15 shoulder bore, removing {expected:.6} mm³: {v:?} removed={removed:.6}"
	);
}

#[test]
fn out_of_range_inputs_return_documented_errors() {
	let plate = plate();
	let at = top(20.0, 15.0);
	let axis = -DVec3::Z;
	let observed = [
		clearance_hole(&plate, at, axis, 7.0, Fit::Medium, None).unwrap_err(), // M7 not in ISO 273 table
		countersink_hole(&plate, at, axis, 2.5, Fit::Medium, None).unwrap_err(), // DIN 74 form F starts at M3
		tap_drill_hole(&plate, at, axis, 14.0, HoleDepth::Through(12.0), None).unwrap_err(), // table ends at M12
		drill(&plate, at, axis, -1.0, HoleDepth::Through(12.0), None).unwrap_err(),
		drill(&plate, at, axis, 6.0, HoleDepth::Blind(0.0), None).unwrap_err(),
		drill(&plate, at, DVec3::ZERO, 6.0, HoleDepth::Through(12.0), None).unwrap_err(),
		bolt_circle(&plate, at, axis, 50.0, 0, 0.0, |s, _| Ok(s)).unwrap_err(),
		bearing_seat(&plate, at, axis, "9999", None).unwrap_err(),
	];
	assert_eq!(
		observed,
		[
			HoleError::UnsupportedSize { m: 7.0 },
			HoleError::UnsupportedSize { m: 2.5 },
			HoleError::UnsupportedSize { m: 14.0 },
			HoleError::BadDiameter,
			HoleError::BadDepth,
			HoleError::BadAxis,
			HoleError::BadCount,
			HoleError::UnknownBearing { designation: "9999".to_string() },
		]
	);
}

#[test]
fn dimension_tables_satisfy_standard_invariants() {
	let mut violations: Vec<String> = Vec::new();
	for s in metric_hole_specs() {
		let [close, medium, coarse] = s.clearance;
		if !(s.m < close && close < medium && medium < coarse) {
			violations.push(format!("M{}: clearance fits not ascending above m: {:?}", s.m, s.clearance));
		}
		if s.counterbore_d <= coarse {
			violations.push(format!("M{}: counterbore Ø{} not larger than the coarse clearance", s.m, s.counterbore_d));
		}
		// DIN 912 head height k = m: DIN 974-1 depth t1 must recess the head flush.
		if s.counterbore_depth < s.m {
			violations.push(format!("M{}: counterbore depth {} shallower than the head", s.m, s.counterbore_depth));
		}
		if !(s.pitch > 0.0 && s.pitch < s.m) {
			violations.push(format!("M{}: implausible coarse pitch {}", s.m, s.pitch));
		}
		// DIN 74 form F exists exactly from M3 up and must clear the screw head past the bore.
		match s.countersink_d {
			Some(dk) if s.m >= 3.0 && dk > coarse => {}
			None if s.m < 3.0 => {}
			other => violations.push(format!("M{}: countersink entry {other:?} out of DIN 74 F range", s.m)),
		}
		if metric_hole_spec(s.m) != Some(s) {
			violations.push(format!("M{}: spec lookup does not round-trip", s.m));
		}
	}
	for b in bearing_specs() {
		if !(b.bore > 0.0 && b.bore < b.outer && b.width > 0.0) {
			violations.push(format!("bearing {}: implausible envelope {b:?}", b.designation));
		}
		if bearing_spec(b.designation) != Some(b) {
			violations.push(format!("bearing {}: lookup does not round-trip", b.designation));
		}
	}
	assert_eq!(violations, Vec::<String>::new(), "dimension tables must satisfy the cited standards' invariants");
}

#[test]
fn min_ligament_echo_reads_the_thin_edge_web_and_clears_a_centred_hole() {
	use kernel_brep::holes::min_ligament;
	let plate = plate();
	// Ø6 planned hole centred 5 mm from the x = 0 edge: web = 5 − 3 = 2 mm.
	let near = min_ligament(&plate, top(5.0, 15.0), -DVec3::Z, 6.0);
	// Same hole at the plate centre: the nearest lateral wall is 15 − 3 = 12 mm
	// away, so the echo is the documented mid-span clamp to the pierced
	// top/bottom faces — 6 mm for the 12 mm plate — comfortably above the bar.
	// Measured (deterministic): near = 2.000, far = 6.000.
	let far = min_ligament(&plate, top(20.0, 15.0), -DVec3::Z, 6.0);
	assert!(
		(near - 2.0).abs() < 0.2 && far > 5.0,
		"min-ligament echo must read the 2 mm edge web (measured {near:.4} mm) and stay above 5 for a centred hole \
		 (measured {far:.4} mm — the mid-span entry/exit clamp of a 12 mm plate, per the documented caveat)"
	);
}
