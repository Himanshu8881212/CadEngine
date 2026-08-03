//! `holes::teardrop_hole` — the FDM support-free idiom for horizontal-axis
//! holes. A plain round hole bored horizontally has a sagging ceiling arc
//! (facets steeper than 45° from vertical near the top); the teardrop replaces
//! that arc with a two-flank ≥46° roof. The support-necessity gate itself
//! (`Mesh::support_free_report`) is the arbiter, with the plain hole as the
//! negative control proving the gate sees the defect the teardrop removes.

use kernel_brep::math::DVec3;
use kernel_brep::{cuboid, drill, teardrop_hole, tessellate_default, validate, HoleDepth};
use kernel_core::math::Vec3;

#[test]
fn teardrop_hole_prints_support_free_where_a_plain_hole_does_not() {
	// A 30×20×12 wall printed as modelled (+Z up); bore along +Y (horizontal).
	let wall = cuboid(DVec3::new(0.0, 0.0, 0.0), DVec3::new(30.0, 20.0, 12.0));
	let at = DVec3::new(15.0, 0.0, 6.0);

	let plain = drill(&wall, at, DVec3::Y, 6.2, HoleDepth::Through(20.0), None).expect("plain drill");
	let tear = teardrop_hole(&wall, at, DVec3::Y, DVec3::Z, 6.2, 20.0, 46.0, None).expect("teardrop");

	let (vp, vt) = (validate(&plain), validate(&tear));
	let (mp, mt) = (tessellate_default(&plain), tessellate_default(&tear));
	let rp = mp.support_free_report(Vec3::Z, 45.0, 0.3);
	let rt = mt.support_free_report(Vec3::Z, 45.0, 0.3);

	// The plain bore's ceiling arc needs support (steep and/or a curved ceiling
	// that bridges only at the very crown); the teardrop needs none at all and
	// stays a valid watertight genus-1 solid.
	assert!(
		vp.is_valid()
			&& vt.is_valid()
			&& mp.is_watertight()
			&& mt.is_watertight()
			&& vt.genus == 1
			&& rp.steep_area > 1.0
			&& rt.steep_area == 0.0,
		"teardrop vs plain horizontal bore: plain(valid={} wt={} steep={:.3}) teardrop(valid={} wt={} genus={} steep={:.3}) \
		 — want plain steep>1, teardrop steep==0",
		vp.is_valid(),
		mp.is_watertight(),
		rp.steep_area,
		vt.is_valid(),
		mt.is_watertight(),
		vt.genus,
		rt.steep_area,
	);
}
