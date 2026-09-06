// Copyright (c) LMCAD. Licensed under the MIT License.

//! ENGINE #29: a hexagonal nut pocket whose FLOOR is coplanar with the end cap
//! of a blind hole below it (the rotor carriage's M5 pocket). The profile's
//! six-decimal coordinates put one of the cap's imprint lines 7e-7 mm past the
//! hexagon's corner; the split minted a near-duplicate vertex there, the
//! duplicate merge moved the corner onto it, and the wall's edge lost the chain
//! of floor vertices along it — open edges, `difference failed validate()`.
//! `split_convex_by_line` now snaps a split point within 4·TJUNCTION_EPS of a
//! polygon corner onto the corner, so the corner stays exact.

use kernel_brep::build::{cuboid, cylinder, extrude};
use kernel_brep::math::{DAffine3, DVec2, DVec3};
use kernel_brep::{difference, exact_volume, validate};

fn hex_profile() -> Vec<DVec2> {
	[(13.5, 4.792007), (9.35, 2.396004), (9.35, -2.396004), (13.5, -4.792007), (17.65, -2.396004), (17.65, 2.396004)]
		.iter()
		.map(|&(x, y)| DVec2::new(x, y))
		.collect()
}

fn shoelace(p: &[DVec2]) -> f64 {
	let n = p.len();
	(0..n).map(|i| p[i].x * p[(i + 1) % n].y - p[(i + 1) % n].x * p[i].y).sum::<f64>().abs() * 0.5
}

#[test]
fn hex_pocket_floor_coplanar_with_a_hole_cap_binds_valid() {
	let block = cuboid(DVec3::new(7.0, -7.0, 18.0), DVec3::new(20.0, 7.0, 26.0));
	// Blind hole from below, ending at z = 21 (48 facets, as the campaign's M5 hole).
	let hole = cylinder(DVec3::new(13.5, 0.0, 10.7), DVec3::Z, 2.75, 10.3, 48);
	let bored = difference(&block, &hole);
	assert!(validate(&bored).is_valid());
	// Pocket 21..27: its floor is the hole's cap plane, its top overshoots the block.
	let pocket = extrude(&hex_profile(), 6.0).transformed(DAffine3::from_translation(DVec3::new(0.0, 0.0, 21.0)));
	let result = difference(&bored, &pocket);
	let v = validate(&result);
	assert!(v.is_valid() && v.shells == 1 && v.genus == 1, "pocket on cap must bind a valid through-passage: {v:?}");
	// Volume: block − hole slice (z 18..21, π-exact through the cylinder tag) −
	// pocket slice (z 21..26, the polygon's own area).
	let hole_area = std::f64::consts::PI * 2.75 * 2.75;
	let expected = 13.0 * 14.0 * 8.0 - hole_area * 3.0 - shoelace(&hex_profile()) * 5.0;
	let vol = exact_volume(&result);
	assert!((vol - expected).abs() < 1e-6, "volume {vol} vs closed form {expected}");
}
