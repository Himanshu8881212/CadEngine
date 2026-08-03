// Copyright (c) LMCAD. Licensed under the MIT License.

//! `Mesh::signed_clearance` (sign-correct proximity: separation / touch /
//! penetration) and `Mesh::warped` + `radial_wave_field` (the deformation
//! bridge for interference gates on elastically deformed working states).

use kernel_core::math::Vec3;
use kernel_core::mesh::Mesh;
use kernel_core::radial_wave_field;

/// A closed axis-aligned cube: corner `min`, edge length `size`, 8 shared
/// vertices, 12 outward-wound triangles.
fn cube(min: Vec3, size: f32) -> Mesh {
	let (x0, y0, z0) = (min.x, min.y, min.z);
	let (x1, y1, z1) = (x0 + size, y0 + size, z0 + size);
	let mut m = Mesh::new();
	m.positions = vec![
		Vec3::new(x0, y0, z0),
		Vec3::new(x1, y0, z0),
		Vec3::new(x1, y1, z0),
		Vec3::new(x0, y1, z0),
		Vec3::new(x0, y0, z1),
		Vec3::new(x1, y0, z1),
		Vec3::new(x1, y1, z1),
		Vec3::new(x0, y1, z1),
	];
	m.indices = vec![
		0, 2, 1, 0, 3, 2, // bottom  (-z)
		4, 5, 6, 4, 6, 7, // top     (+z)
		0, 1, 5, 0, 5, 4, // front   (-y)
		2, 3, 7, 2, 7, 6, // back    (+y)
		0, 4, 7, 0, 7, 3, // left    (-x)
		1, 2, 6, 1, 6, 5, // right   (+x)
	];
	m.ensure_outward();
	m
}

/// A closed annular tube about +Z: outer radius `router`, inner radius
/// `rinner`, height `h`, `n` segments; outer/inner walls + flat annular caps,
/// consistently outward-wound. No vertex sits on the axis, so every vertex
/// carries a well-defined azimuth for the wave field.
fn tube(router: f32, rinner: f32, h: f32, n: u32) -> Mesh {
	let mut m = Mesh::new();
	let ring = |radius: f32, z: f32, m: &mut Mesh| -> u32 {
		let first = m.positions.len() as u32;
		for k in 0..n {
			let phi = 2.0 * std::f32::consts::PI * k as f32 / n as f32;
			m.positions.push(Vec3::new(radius * phi.cos(), radius * phi.sin(), z));
		}
		first
	};
	let (ob, ot) = (ring(router, 0.0, &mut m), ring(router, h, &mut m));
	let (ib, it) = (ring(rinner, 0.0, &mut m), ring(rinner, h, &mut m));
	let mut quad = |a: u32, b: u32, c: u32, d: u32| {
		m.indices.extend_from_slice(&[a, b, c, a, c, d]);
	};
	for k in 0..n {
		let k1 = (k + 1) % n;
		quad(ob + k, ob + k1, ot + k1, ot + k); // outer wall  (+r)
		quad(ib + k, it + k, it + k1, ib + k1); // inner wall  (-r, toward axis)
		quad(ob + k, ib + k, ib + k1, ob + k1); // bottom cap  (-z)
		quad(ot + k, ot + k1, it + k1, it + k); // top cap     (+z)
	}
	m
}

#[test]
fn disjoint_cubes_report_their_separation() {
	// Unit cubes 0.5 apart along X: the positive branch must match min_distance.
	let a = cube(Vec3::ZERO, 1.0);
	let b = cube(Vec3::new(1.5, 0.0, 0.0), 1.0);
	let v = a.signed_clearance(&b);
	assert!(
		(v - 0.5).abs() < 1e-3 && (v - a.min_distance(&b)).abs() < 1e-6,
		"signed_clearance of cubes 0.5 apart must be ≈ +0.5 and equal min_distance ({}), got {v}",
		a.min_distance(&b)
	);
}

#[test]
fn overlapping_cubes_report_negative_penetration() {
	// Unit cubes overlapping by 0.3 along X. min_distance says 0.000 — useless.
	// signed_clearance must be negative with magnitude ≈ the 0.3 overlap
	// (contract: a lower bound within 20%, so |v| ∈ [0.24, 0.36]).
	let a = cube(Vec3::ZERO, 1.0);
	let b = cube(Vec3::new(0.7, 0.0, 0.0), 1.0);
	let v = a.signed_clearance(&b);
	assert!(
		v < 0.0 && (0.24..=0.36).contains(&v.abs()),
		"cubes overlapping by 0.3 must report negative clearance with |v| in [0.24, 0.36] (min_distance sees only {}), got {v}",
		a.min_distance(&b)
	);
}

#[test]
fn face_touching_cubes_report_zero() {
	// Cubes sharing the exact face x = 1: coplanar datum contact, not
	// interference — the whole point of the signed query.
	let a = cube(Vec3::ZERO, 1.0);
	let b = cube(Vec3::new(1.0, 0.0, 0.0), 1.0);
	let v = a.signed_clearance(&b);
	assert!(v.abs() <= 1e-3, "exact face contact must report ~0 (|v| ≤ 1e-3), got {v}");
}

#[test]
fn contained_cube_reports_negative_escape_depth() {
	// A 0.5 cube centred inside a 2.0 cube: surfaces are disjoint (min_distance
	// = +0.75!), but the solids interpenetrate totally. Documented convention:
	// magnitude = deepest sampled surface point's outward escape distance,
	// which for this centred pair is uniformly 0.75 over the whole small-cube
	// surface. Assert ≥ 0.9 × true and never above true (it is a lower bound).
	let big = cube(Vec3::ZERO, 2.0);
	let small = cube(Vec3::splat(0.75), 0.5);
	let (v, w) = (big.signed_clearance(&small), small.signed_clearance(&big));
	assert!(
		v < 0.0 && (0.9 * 0.75..=0.75 + 1e-3).contains(&v.abs()) && (v - w).abs() < 1e-6,
		"full containment must be negative with |v| in [0.675, 0.75] and symmetric (min_distance sees +{}), got {v} / swapped {w}",
		big.min_distance(&small)
	);
}

#[test]
fn warped_tube_follows_the_strain_wave_and_stays_watertight() {
	// A Ø20/Ø16 tube warped by the two-lobe strain-wave field with w0 = 0.6:
	// at φ = 0 (cos 2φ = 1, sin 2φ = 0) the displacement is purely radial +0.6,
	// at φ = π/2 purely radial −0.6 — so the max vertex radius must grow by
	// ≈ 0.6 and the min shrink by ≈ 0.6 (within 5% of w0), with topology (and
	// hence watertightness) untouched.
	let t = tube(10.0, 8.0, 4.0, 64);
	assert!(t.is_watertight() && t.signed_volume() > 0.0, "test tube must start closed and outward");
	let w = t.warped(radial_wave_field(0.6, 2, 0.0));
	let radius = |m: &Mesh| {
		m.positions.iter().fold((f32::NEG_INFINITY, f32::INFINITY), |(hi, lo), p| {
			let r = (p.x * p.x + p.y * p.y).sqrt();
			(hi.max(r), lo.min(r))
		})
	};
	let (max_r, min_r) = radius(&w);
	let (grow, shrink) = (max_r - 10.0, 8.0 - min_r);
	assert!(
		(grow - 0.6).abs() <= 0.03 && (shrink - 0.6).abs() <= 0.03 && w.is_watertight() && w.vertex_count() == t.vertex_count() && w.indices == t.indices,
		"strain-wave warp must grow max radius by ≈0.6 (got +{grow:.4}), shrink min by ≈0.6 (got -{shrink:.4}), preserve topology and watertightness (watertight: {})",
		w.is_watertight()
	);
}
