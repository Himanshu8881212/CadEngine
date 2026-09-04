//! Loft-through-boolean: a square->round lofted BOSS sitting on a plate (a
//! coplanar union of the loft's bottom cap with the plate's top face), then a
//! bore through both. Stresses faceted loft faces meeting planar faces in the
//! arrangement. The coplanar union must be exact-ADDITIVE in volume (the two
//! bodies meet only on the z=0 plane), and the bored result a valid genus-1 part.

use kernel_brep::math::DVec3;
use kernel_brep::{cuboid, cylinder, loft_solid, tessellate_default, try_difference, try_union, validate, volume};
use std::f64::consts::TAU;

fn square_pt(theta: f64, half: f64) -> (f64, f64) {
	let (c, s) = (theta.cos(), theta.sin());
	let t = half / c.abs().max(s.abs());
	(t * c, t * s)
}

#[test]
fn lofted_boss_on_plate_unions_coplanar_and_bores_through() {
	let m = 48usize;
	let bottom: Vec<DVec3> = (0..m)
		.map(|i| {
			let (x, y) = square_pt(TAU * i as f64 / m as f64, 20.0);
			DVec3::new(x, y, 0.0)
		})
		.collect();
	let top: Vec<DVec3> = (0..m)
		.map(|i| {
			let th = TAU * i as f64 / m as f64;
			DVec3::new(15.0 * th.cos(), 15.0 * th.sin(), 40.0)
		})
		.collect();
	let boss = loft_solid(&[bottom, top]).expect("boss lofts");
	let plate = cuboid(DVec3::new(-30.0, -30.0, -5.0), DVec3::new(30.0, 30.0, 0.0));
	let (boss_v, plate_v) = (volume(&boss).abs(), volume(&plate).abs());

	// Coplanar union: bodies meet only on z=0, so volume is exactly additive.
	let part = try_union(&plate, &boss).expect("coplanar loft-boss union must succeed");
	let v = validate(&part);
	let mesh = tessellate_default(&part);
	assert!(
		v.closed
			&& v.manifold
			&& v.genus == 0
			&& mesh.is_watertight()
			&& !mesh.has_self_intersection()
			&& (volume(&part).abs() - (plate_v + boss_v)).abs() / (plate_v + boss_v) < 1e-4,
		"plate ∪ loft-boss must be watertight genus-0 with additive volume {}≈{}+{}: {v:?} self_int={}",
		volume(&part).abs(),
		plate_v,
		boss_v,
		mesh.has_self_intersection()
	);

	// Bore through the round top down through the plate -> genus 1.
	let bored = try_difference(&part, &cylinder(DVec3::new(0.0, 0.0, -10.0), DVec3::Z, 8.0, 60.0, 48))
		.expect("bore through loft+plate must succeed");
	let bv = validate(&bored);
	let bm = tessellate_default(&bored);
	assert!(
		bv.closed && bv.manifold && bv.genus == 1 && bm.is_watertight() && !bm.has_self_intersection(),
		"bored loft-boss+plate must be a watertight genus-1 part: {bv:?} self_int={}",
		bm.has_self_intersection()
	);
}
