//! A realistic difficult part end-to-end: a hydraulic MANIFOLD block built from
//! nine chained boolean/feature operations — a main through-bore, two Z
//! cross-bores intersecting it, a Y port bore through the centre, and four
//! counterbored mounting holes. It must come out a valid, watertight,
//! self-intersection-free genus-11 solid and rebuild bit-identically
//! (determinism). Heavier than any gallery part; guards the chained-boolean
//! pipeline + hole wizard + boolean determinism together.

use kernel_brep::holes::{counterbore_hole, Fit};
use kernel_brep::math::DVec3;
use kernel_brep::{cuboid, cylinder, tessellate_default, try_difference, validate, volume, Solid};

fn build_manifold() -> Solid {
	let mut m = cuboid(DVec3::new(-30.0, -20.0, -15.0), DVec3::new(30.0, 20.0, 15.0));
	for bore in [
		cylinder(DVec3::new(-35.0, 0.0, 0.0), DVec3::X, 6.0, 70.0, 48), // main X
		cylinder(DVec3::new(-15.0, 0.0, -20.0), DVec3::Z, 4.0, 40.0, 48), // cross Z1
		cylinder(DVec3::new(15.0, 0.0, -20.0), DVec3::Z, 4.0, 40.0, 48),  // cross Z2
		cylinder(DVec3::new(0.0, -25.0, 0.0), DVec3::Y, 4.0, 50.0, 48),   // port Y
	] {
		m = try_difference(&m, &bore).expect("manifold bore boolean succeeds");
	}
	for (x, y) in [(-24.0, -14.0), (24.0, -14.0), (24.0, 14.0), (-24.0, 14.0)] {
		m = counterbore_hole(&m, DVec3::new(x, y, -15.0), DVec3::Z, 5.0, Fit::Medium, None).expect("mounting hole cuts");
	}
	m
}

#[test]
fn hydraulic_manifold_builds_valid_genus_11_and_is_deterministic() {
	let m = build_manifold();
	let v = validate(&m);
	let mesh = tessellate_default(&m);
	let vol = volume(&m).abs();

	// Topology: main(+1) + two crosses each intersecting main (+2 each) + port
	// intersecting main (+2) + four through mount holes (+1 each) = genus 11.
	assert!(
		v.closed && v.manifold && v.genus == 11 && mesh.is_watertight() && !mesh.has_self_intersection() && (55_000.0..62_000.0).contains(&vol),
		"manifold must be a valid watertight genus-11 solid (no self-intersection) of plausible volume: {v:?} watertight={} self_int={} vol={vol:.0}",
		mesh.is_watertight(),
		mesh.has_self_intersection()
	);

	// Determinism: an independent rebuild must match bit-for-bit.
	assert_eq!(
		volume(&m).to_bits(),
		volume(&build_manifold()).to_bits(),
		"the manifold must rebuild bit-identically (boolean determinism over a 9-op chain)"
	);
}
