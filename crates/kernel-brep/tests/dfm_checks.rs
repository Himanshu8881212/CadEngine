//! DFM (design-for-manufacturing) checks must FLAG real defects with the right
//! magnitudes AND not false-flag a good part — the manufacturability gate users
//! rely on. Exact planar areas make the magnitudes assertable.

use kernel_brep::math::{DVec2, DVec3};
use kernel_brep::{cuboid, draft_analysis, extrude_tapered, overhang_analysis, sphere, wall_thickness};

#[test]
fn dfm_checks_flag_defects_and_pass_good_geometry() {
	// Thin wall: a 20x20x0.5 plate. wall_thickness(flag_below=1.0) must measure the
	// 0.5 mm thickness exactly and flag the two 20x20 = 800 mm^2 flat faces.
	let plate = cuboid(DVec3::new(-10.0, -10.0, 0.0), DVec3::new(10.0, 10.0, 0.5));
	let t = wall_thickness(&plate, 1.0);
	assert!(
		(t.min_thickness - 0.5).abs() < 1e-6 && (t.thin_area - 800.0).abs() < 1.0,
		"thin plate: min_thickness={} (want 0.5), thin_area={} (want 800)",
		t.min_thickness,
		t.thin_area
	);

	// Zero draft: a plain block pulled along +Z. Its four 20x20 vertical walls are
	// parallel to the pull => min_draft 0 and low_draft_area = 1600 mm^2.
	let block = cuboid(DVec3::new(-10.0, -10.0, 0.0), DVec3::new(10.0, 10.0, 20.0));
	let d = draft_analysis(&block, DVec3::Z, 2.0);
	assert!(
		d.min_draft_deg < 1e-6 && (d.low_draft_area - 1600.0).abs() < 1.0,
		"zero-draft block: min_draft={}° (want 0), low_draft_area={} (want 1600)",
		d.min_draft_deg,
		d.low_draft_area
	);

	// Overhang: a sphere printed +Z overhangs its lower portion past the 45°
	// self-support threshold => a meaningful flagged fraction.
	let ball = sphere(DVec3::ZERO, 10.0, 48, 24);
	let o = overhang_analysis(&ball, DVec3::Z, 45.0);
	let flagged = o.needs_support.iter().filter(|&&b| b).count();
	assert!(
		o.overhang_fraction > 0.1 && flagged > 0,
		"overhang sphere: fraction={} flagged={}/{}",
		o.overhang_fraction,
		flagged,
		o.needs_support.len()
	);

	// Control: a 5°-drafted block must NOT be flagged at a 2° minimum (no false
	// positive — the property that makes the check trustworthy).
	let tapered = extrude_tapered(
		&[DVec2::new(-10.0, -10.0), DVec2::new(10.0, -10.0), DVec2::new(10.0, 10.0), DVec2::new(-10.0, 10.0)],
		20.0,
		5.0_f64.to_radians(),
	);
	let dt = draft_analysis(&tapered, DVec3::Z, 2.0);
	assert!(
		dt.low_draft_area < 1.0 && dt.min_draft_deg > 1.9,
		"5°-draft block must pass a 2° check (no false flag): min_draft={}° low_draft_area={}",
		dt.min_draft_deg,
		dt.low_draft_area
	);
}
