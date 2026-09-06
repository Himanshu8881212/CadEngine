// Copyright (c) LMCAD. Licensed under the MIT License.

//! `Mesh::min_distance` (BVH pair descent) must give the SAME answer as the
//! O(n·m) reference `Mesh::min_distance_brute` on the pairs that once broke a
//! BVH clearance route: curved surfaces, arbitrary rotations, a part engulfed
//! inside another, touching and interpenetrating pairs. A flat-grid equivalence
//! is not sufficient (see the note on the reverted "BVH-accelerate assembly
//! clearance" commit), so every case here is curved or rotated.

use kernel_core::math::{Mat3, Vec3};
use kernel_core::Mesh;

fn uv_sphere(center: Vec3, r: f32, n: usize) -> Mesh {
	let mut m = Mesh::new();
	let mut push = |a: Vec3, b: Vec3, c: Vec3| {
		let base = m.positions.len() as u32;
		for p in [a, b, c] {
			m.positions.push(p);
			m.normals.push((p - center).normalize());
		}
		m.indices.extend_from_slice(&[base, base + 1, base + 2]);
	};
	let pt = |i: usize, j: usize| {
		let th = std::f32::consts::PI * i as f32 / n as f32;
		let ph = 2.0 * std::f32::consts::PI * j as f32 / (2 * n) as f32;
		center + Vec3::new(th.sin() * ph.cos(), th.sin() * ph.sin(), th.cos()) * r
	};
	for i in 0..n {
		for j in 0..2 * n {
			let (a, b, c, d) = (pt(i, j), pt(i + 1, j), pt(i + 1, j + 1), pt(i, j + 1));
			push(a, b, c);
			push(a, c, d);
		}
	}
	m
}

fn tube(r: f32, h: f32, n: usize, rot: Mat3, shift: Vec3) -> Mesh {
	let mut m = Mesh::new();
	let mut push = |a: Vec3, b: Vec3, c: Vec3| {
		let base = m.positions.len() as u32;
		for p in [a, b, c] {
			m.positions.push(rot * p + shift);
			m.normals.push(Vec3::Z);
		}
		m.indices.extend_from_slice(&[base, base + 1, base + 2]);
	};
	for k in 0..n {
		let (t0, t1) = (2.0 * std::f32::consts::PI * k as f32 / n as f32, 2.0 * std::f32::consts::PI * (k + 1) as f32 / n as f32);
		let (a, b) = (Vec3::new(r * t0.cos(), r * t0.sin(), 0.0), Vec3::new(r * t1.cos(), r * t1.sin(), 0.0));
		let (c, d) = (a + Vec3::Z * h, b + Vec3::Z * h);
		push(a, b, d);
		push(a, d, c);
		// caps
		push(Vec3::ZERO, b, a);
		push(Vec3::Z * h, c, d);
	}
	m
}

fn lcg(seed: &mut u64) -> f32 {
	*seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
	((*seed >> 33) as f32) / (u32::MAX >> 1) as f32
}

fn rot(seed: &mut u64) -> Mat3 {
	let (a, b, c) = (lcg(seed) * 6.28, lcg(seed) * 6.28, lcg(seed) * 6.28);
	Mat3::from_rotation_z(a) * Mat3::from_rotation_y(b) * Mat3::from_rotation_x(c)
}

fn check(name: &str, a: &Mesh, b: &Mesh) {
	let (fast, brute) = (a.min_distance(b), a.min_distance_brute(b));
	let (fast_r, brute_r) = (b.min_distance(a), b.min_distance_brute(a));
	assert!((fast - brute).abs() <= 1e-5 * (1.0 + brute.abs()), "{name}: bvh {fast} vs brute {brute}");
	assert!((fast_r - brute_r).abs() <= 1e-5 * (1.0 + brute_r.abs()), "{name} (swapped): bvh {fast_r} vs brute {brute_r}");
	assert!((fast - fast_r).abs() <= 1e-5 * (1.0 + fast.abs()), "{name}: asymmetric {fast} / {fast_r}");
}

#[test]
fn bvh_matches_brute_on_curved_rotated_pairs() {
	let mut seed = 0x5eed_u64;
	for case in 0..40 {
		let ra = rot(&mut seed);
		let rb = rot(&mut seed);
		let shift = Vec3::new(lcg(&mut seed) * 30.0 - 15.0, lcg(&mut seed) * 30.0 - 15.0, lcg(&mut seed) * 30.0 - 15.0);
		let a = tube(4.0 + lcg(&mut seed) * 4.0, 10.0 + lcg(&mut seed) * 10.0, 24, ra, Vec3::ZERO);
		let b = tube(2.0 + lcg(&mut seed) * 6.0, 6.0 + lcg(&mut seed) * 12.0, 17, rb, shift);
		check(&format!("tube pair {case}"), &a, &b);
	}
}

#[test]
fn bvh_matches_brute_on_engulfed_and_touching() {
	let big = uv_sphere(Vec3::ZERO, 10.0, 24);
	let small = uv_sphere(Vec3::new(1.0, -2.0, 0.5), 2.5, 12);
	check("engulfed sphere", &big, &small);
	assert!(big.min_distance(&small) > 4.0, "engulfed part reports the shell gap, not 0");
	// Two tubes sharing an end face: touching → 0.
	let a = tube(5.0, 8.0, 32, Mat3::IDENTITY, Vec3::ZERO);
	let b = tube(3.0, 8.0, 20, Mat3::IDENTITY, Vec3::Z * 8.0);
	check("touching tubes", &a, &b);
	assert_eq!(a.min_distance(&b), 0.0);
	// Interpenetrating spheres (the small one straddles the big shell) → 0.
	let c = uv_sphere(Vec3::new(9.5, 0.0, 0.0), 2.0, 16);
	check("interpenetrating spheres", &big, &c);
	assert_eq!(big.min_distance(&c), 0.0);
	// A known gap: sphere r=2 centred 15 from the big sphere's centre → 3.0.
	let d = uv_sphere(Vec3::new(15.0, 0.0, 0.0), 2.0, 16);
	check("gapped spheres", &big, &d);
	let g = big.min_distance(&d);
	assert!((g - 3.0).abs() < 0.05, "faceted spheres 3.0 apart, got {g}");
}
