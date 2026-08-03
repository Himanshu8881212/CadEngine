//! A real common part: an involute SPUR GEAR with a keyed bore (built by
//! extrude_bored, the keyway as an inner loop of the extrusion) plus a ring of
//! web lightening holes (subsequent booleans). The keyed bore sidesteps the
//! tangent-face coincidence class (the keyway is part of the profile, not a
//! tangent box subtraction), and the web holes add clean handles: each through
//! hole increments the genus, ending a valid, watertight, self-intersection-free
//! genus-6 solid.

use kernel_brep::math::DVec3;
use kernel_brep::{cylinder, tessellate_default, try_difference, validate};
use kernel_model::parts::{din6885_key_size, spur_gear};

#[test]
fn keyed_involute_gear_with_web_lightening_holes_is_valid_genus_6() {
	let gear = spur_gear(2.0, 24, 10.0, 12.0, 20.0, din6885_key_size(12.0));
	let g0 = validate(&gear);
	assert!(g0.closed && g0.manifold && g0.genus == 1, "keyed gear blank must be a valid genus-1 solid (the bore): {g0:?}");

	// Five Ø5 lightening holes on a Ø28 bolt circle — clear of the bore (r=6) and
	// the tooth root (r≈21.5), so each is an enclosed through-hole (+1 genus).
	let mut m = gear;
	for i in 0..5 {
		let a = std::f64::consts::TAU * i as f64 / 5.0;
		let hole = cylinder(DVec3::new(14.0 * a.cos(), 14.0 * a.sin(), -1.0), DVec3::Z, 2.5, 12.0, 32);
		m = try_difference(&m, &hole).unwrap_or_else(|e| panic!("web lightening hole {i} must cut cleanly: {e:?}"));
		assert_eq!(validate(&m).genus, (2 + i) as i64, "each web through-hole must add exactly one handle (after hole {i})");
	}
	let v = validate(&m);
	let mesh = tessellate_default(&m);
	assert!(
		v.closed && v.manifold && v.genus == 6 && mesh.is_watertight() && !mesh.has_self_intersection(),
		"keyed gear + 5 web lightening holes must be a valid watertight genus-6 part: {v:?} self_int={}",
		mesh.has_self_intersection()
	);
}
