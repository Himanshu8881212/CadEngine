//! Acceptance for two bread-and-butter parts that exercise the curved-face
//! frontier (loft + curved/curved booleans), neither previously covered:
//!
//! 1. a square-to-round transition reducer via `loft_solid` — the faceted
//!    vertex-morph must close exactly to the prismatoid (Simpson) volume, since
//!    the cross-section area is quadratic in the loft parameter;
//! 2. a cross-bore manifold — two intersecting cylindrical through-bores in a
//!    block (a Steinmetz seam). Topology must be genus 3 (k intersecting
//!    perpendicular holes give genus 2k-1) and the volume must match the
//!    inclusion-exclusion analytic to facet tolerance. A second pair crossing at
//!    60 deg must stay genus 3 — the angle changes geometry, never topology.

use kernel_brep::math::DVec3;
use kernel_brep::{cuboid, cylinder, loft_solid, sweep_solid, tessellate_default, try_difference, validate, volume};
use std::f64::consts::{PI, TAU};

/// Boundary point of the square `max(|x|,|y|)=half` along the ray at `theta`.
fn square_pt(theta: f64, half: f64) -> (f64, f64) {
	let (c, s) = (theta.cos(), theta.sin());
	let t = half / c.abs().max(s.abs());
	(t * c, t * s)
}

fn poly_area(p: &[(f64, f64)]) -> f64 {
	let n = p.len();
	let mut a = 0.0;
	for i in 0..n {
		let j = (i + 1) % n;
		a += p[i].0 * p[j].1 - p[j].0 * p[i].1;
	}
	a.abs() * 0.5
}

#[test]
fn square_to_round_reducer_lofts_to_the_exact_prismatoid_volume() {
	let m = 64usize;
	let (half, r, h) = (20.0, 15.0, 50.0);
	let bottom: Vec<DVec3> = (0..m)
		.map(|i| {
			let (x, y) = square_pt(TAU * i as f64 / m as f64, half);
			DVec3::new(x, y, 0.0)
		})
		.collect();
	let top: Vec<DVec3> = (0..m)
		.map(|i| {
			let th = TAU * i as f64 / m as f64;
			DVec3::new(r * th.cos(), r * th.sin(), h)
		})
		.collect();

	let solid = loft_solid(&[bottom.clone(), top.clone()]).expect("reducer lofts");
	let v = validate(&solid);
	let vol = volume(&solid).abs();

	// Exact prismatoid volume of the linear vertex-morph (area is quadratic in t,
	// so Simpson's rule is exact): V = h/6 * (A0 + 4*A_mid + A1).
	let b2: Vec<(f64, f64)> = bottom.iter().map(|p| (p.x, p.y)).collect();
	let t2: Vec<(f64, f64)> = top.iter().map(|p| (p.x, p.y)).collect();
	let mid: Vec<(f64, f64)> = b2.iter().zip(&t2).map(|(a, b)| ((a.0 + b.0) * 0.5, (a.1 + b.1) * 0.5)).collect();
	let expected = h / 6.0 * (poly_area(&b2) + 4.0 * poly_area(&mid) + poly_area(&t2));

	assert!(
		v.closed && v.manifold && v.genus == 0 && tessellate_default(&solid).is_watertight() && (vol - expected).abs() / expected < 1e-6,
		"square->round reducer must be a watertight genus-0 solid whose faceted volume equals the prismatoid integral exactly: \
		 got {v:?} vol={vol:.4} expected={expected:.4} relerr={:.2e}",
		(vol - expected).abs() / expected
	);
}

#[test]
fn cross_bore_manifold_is_genus_3_with_correct_volume() {
	let (seg, r, l, half) = (64usize, 10.0, 60.0, 25.0);
	let block = cuboid(DVec3::new(-half, -half, -half), DVec3::new(half, half, half));
	let bore_x = cylinder(DVec3::new(-l / 2.0, 0.0, 0.0), DVec3::X, r, l, seg);
	let bore_y = cylinder(DVec3::new(0.0, -l / 2.0, 0.0), DVec3::Y, r, l, seg);

	let manifold = try_difference(&block, &bore_x)
		.and_then(|s| try_difference(&s, &bore_y))
		.expect("perpendicular cross-bore booleans succeed");
	let v = validate(&manifold);
	let vol = volume(&manifold).abs();

	// Inclusion-exclusion: removed = 2*(pi r^2 L_in) - Steinmetz(16/3 r^3), with the
	// bicylinder (extent r < half) fully inside the cube. Facet error < ~0.5%.
	let span = 2.0 * half;
	let expected = span.powi(3) - (2.0 * PI * r * r * span - 16.0 / 3.0 * r.powi(3));

	assert!(
		v.closed && v.manifold && v.genus == 3 && tessellate_default(&manifold).is_watertight() && (vol - expected).abs() / expected < 5e-3,
		"perpendicular cross-bore manifold must be a watertight genus-3 solid matching the inclusion-exclusion volume: \
		 got {v:?} vol={vol:.2} expected={expected:.2} relerr={:.2e}",
		(vol - expected).abs() / expected
	);
}

#[test]
fn angled_cross_bore_stays_genus_3() {
	// Two cylinders crossing at 60 deg, each exiting an opposite face pair cleanly
	// (no edge clipping). Geometry differs from the perpendicular case but the
	// topology must not: still two intersecting cylinders -> genus 3.
	let (seg, r, l, half) = (64usize, 10.0, 60.0, 25.0);
	let block = cuboid(DVec3::new(-half, -half, -half), DVec3::new(half, half, half));
	let bore_x = cylinder(DVec3::new(-l / 2.0, 0.0, 0.0), DVec3::X, r, l, seg);
	let theta = 60.0_f64.to_radians();
	let dir = DVec3::new(theta.cos(), theta.sin(), 0.0);
	let bore_b = cylinder(dir * (-l / 2.0), dir, r, l, seg);

	let manifold = try_difference(&block, &bore_x)
		.and_then(|s| try_difference(&s, &bore_b))
		.expect("angled cross-bore booleans succeed");
	let v = validate(&manifold);

	assert!(
		v.closed && v.manifold && v.genus == 3 && tessellate_default(&manifold).is_watertight() && volume(&manifold).abs() > 0.0,
		"60-degree cross-bore manifold must stay a watertight genus-3 solid (angle changes geometry, not topology): got {v:?}"
	);
}

/// A hollow bent pipe: an octagonal outer profile swept along an L-path, minus an
/// inner octagon on the same path — the sweep + boolean combination (a boolean on
/// triangulated swept faces). The result must be a watertight genus-1 through-bore
/// tube (coincident end caps subtract to annular rings) with volume equal to
/// outer − inner (the inner solid lies wholly inside the outer) to boolean
/// re-tessellation precision (~1e-8 relative; not bit-exact, the cut faces are
/// re-triangulated).
#[test]
fn hollow_bent_pipe_via_sweep_and_boolean_is_watertight_genus_1() {
	let octagon = |r: f64| -> Vec<DVec3> {
		(0..8).map(|i| {
			let a = TAU * i as f64 / 8.0;
			DVec3::new(r * a.cos(), r * a.sin(), 0.0)
		}).collect()
	};
	let path = [DVec3::new(0.0, 0.0, 0.0), DVec3::new(0.0, 0.0, 25.0), DVec3::new(20.0, 0.0, 25.0)];
	let outer = sweep_solid(&octagon(8.0), &path).expect("outer sweeps");
	let inner = sweep_solid(&octagon(5.0), &path).expect("inner sweeps");
	let pipe = try_difference(&outer, &inner).expect("hollow-pipe boolean succeeds");

	let v = validate(&pipe);
	let mesh = tessellate_default(&pipe);
	let vol = volume(&pipe).abs();
	let expected = volume(&outer).abs() - volume(&inner).abs();
	assert!(
		v.closed
			&& v.manifold
			&& v.genus == 1
			&& v.shells == 1
			&& mesh.is_watertight()
			&& !mesh.has_self_intersection()
			&& (vol - expected).abs() / expected < 1e-4,
		"hollow bent pipe must be a watertight genus-1 through-bore tube with volume = outer - inner: \
		 got {v:?} watertight={} self_int={} vol={vol:.3} expected={expected:.3}",
		mesh.is_watertight(),
		mesh.has_self_intersection()
	);
}
