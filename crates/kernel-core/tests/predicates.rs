// Copyright (c) LMCAD. Licensed under the MIT License.

//! Correctness + robustness of the exact orientation predicate.

use kernel_core::{incircle, incircle_exact, orient2d, orient2d_exact, orient3d, orient3d_exact};

/// Tiny deterministic LCG so the tests need no `rand` dependency.
struct Lcg(u64);
impl Lcg {
	fn next(&mut self) -> u64 {
		self.0 = self.0.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
		self.0
	}
	/// Integer in `[-range, range]`.
	fn int(&mut self, range: i64) -> i64 {
		(self.next() % (2 * range as u64 + 1)) as i64 - range
	}
}

fn sgn(x: f64) -> i32 {
	if x > 0.0 {
		1
	} else if x < 0.0 {
		-1
	} else {
		0
	}
}

/// Independent exact reference for integer coordinates, computed in `i128`.
fn orient2d_i128(a: [i64; 2], b: [i64; 2], c: [i64; 2]) -> i32 {
	let det = (a[0] as i128 - c[0] as i128) * (b[1] as i128 - c[1] as i128) - (a[1] as i128 - c[1] as i128) * (b[0] as i128 - c[0] as i128);
	det.signum() as i32
}

#[test]
fn exact_path_matches_an_independent_i128_determinant() {
	// On integer coordinates the i128 determinant is exact ground truth. This
	// exercises the full expansion arithmetic of `orient2d_exact` directly (the
	// fast filter is bypassed by calling the exact entry point), so a bug in
	// two_product / grow-expansion / sign extraction would surface here.
	let mut rng = Lcg(0x1234_5678_9abc_def0);
	for _ in 0..50_000 {
		let a = [rng.int(100_000), rng.int(100_000)];
		let b = [rng.int(100_000), rng.int(100_000)];
		let c = [rng.int(100_000), rng.int(100_000)];
		let got = sgn(orient2d_exact([a[0] as f64, a[1] as f64], [b[0] as f64, b[1] as f64], [c[0] as f64, c[1] as f64]));
		let want = orient2d_i128(a, b, c);
		assert_eq!(got, want, "exact orient2d disagrees with i128 on a={a:?} b={b:?} c={c:?}");
	}
}

#[test]
fn filtered_orient2d_always_agrees_with_the_exact_sign() {
	// The fast filter must never return a sign that differs from the exact one.
	let mut rng = Lcg(0xfeed_face_dead_beef);
	for _ in 0..50_000 {
		// Mix of scales, including values whose products nearly cancel.
		let s = [1.0, 1e-6, 1e6][(rng.next() % 3) as usize];
		let g = |r: &mut Lcg| (r.int(1_000) as f64) * s + (r.int(1_000) as f64) * s * 1e-12;
		let a = [g(&mut rng), g(&mut rng)];
		let b = [g(&mut rng), g(&mut rng)];
		let c = [g(&mut rng), g(&mut rng)];
		assert_eq!(sgn(orient2d(a, b, c)), sgn(orient2d_exact(a, b, c)), "filter/exact mismatch a={a:?} b={b:?} c={c:?}");
	}
}

#[test]
fn exact_predicate_is_self_consistent_where_naive_f64_is_not() {
	// Naive f64 orientation: the textbook determinant with rounded intermediates.
	fn naive(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> f64 {
		(a[0] - c[0]) * (b[1] - c[1]) - (a[1] - c[1]) * (b[0] - c[0])
	}
	// A determinant is invariant under cyclic permutation, so orient(a,b,c),
	// orient(b,c,a), orient(c,a,b) must share one sign — a hard invariant the
	// arrangement code relies on. Generate near-collinear triples (c placed on the
	// a→b line, then nudged by 1 ULP) and count how often each predicate breaks it.
	let mut rng = Lcg(0x0bad_c0de_cafe_f00d);
	let mut naive_bad = 0usize;
	let mut exact_bad = 0usize;
	let nudge = |x: f64, up: bool| if up { f64::from_bits(x.to_bits() + 1) } else { f64::from_bits(x.to_bits().wrapping_sub(1)) };
	for _ in 0..60_000 {
		let a = [rng.int(64) as f64 + 0.5, rng.int(64) as f64 + 0.5];
		let b = [rng.int(64) as f64 + 0.5, rng.int(64) as f64 + 0.5];
		// Point on the segment, then perturbed by one unit in the last place.
		let t = 0.5;
		let mut c = [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])];
		c[1] = nudge(c[1], rng.next() & 1 == 0);

		let cyc = |f: &dyn Fn([f64; 2], [f64; 2], [f64; 2]) -> f64| {
			let s1 = sgn(f(a, b, c));
			let s2 = sgn(f(b, c, a));
			let s3 = sgn(f(c, a, b));
			s1 == s2 && s2 == s3
		};
		if !cyc(&naive) {
			naive_bad += 1;
		}
		if !cyc(&|x, y, z| orient2d(x, y, z)) {
			exact_bad += 1;
		}
	}
	// The exact predicate is *never* inconsistent; the naive one demonstrably is —
	// that gap is the whole reason this module exists.
	assert_eq!(exact_bad, 0, "exact predicate violated cyclic-sign invariance {exact_bad} times");
	assert!(naive_bad > 0, "test did not reach the degenerate regime where naive f64 fails");
}

/// Independent exact `orient3d` reference (sign of `det[a−d; b−d; c−d]`) in i128.
fn orient3d_i128(a: [i64; 3], b: [i64; 3], c: [i64; 3], d: [i64; 3]) -> i32 {
	let u = [a[0] as i128 - d[0] as i128, a[1] as i128 - d[1] as i128, a[2] as i128 - d[2] as i128];
	let v = [b[0] as i128 - d[0] as i128, b[1] as i128 - d[1] as i128, b[2] as i128 - d[2] as i128];
	let w = [c[0] as i128 - d[0] as i128, c[1] as i128 - d[1] as i128, c[2] as i128 - d[2] as i128];
	let det = u[0] * (v[1] * w[2] - v[2] * w[1]) - u[1] * (v[0] * w[2] - v[2] * w[0]) + u[2] * (v[0] * w[1] - v[1] * w[0]);
	det.signum() as i32
}

#[test]
fn orient3d_exact_matches_an_independent_i128_determinant() {
	// Exercises the full 4×4 triple-product expansion against i128 ground truth.
	let mut rng = Lcg(0xa5a5_5a5a_1234_9999);
	let f = |v: [i64; 3]| [v[0] as f64, v[1] as f64, v[2] as f64];
	for _ in 0..50_000 {
		let a = [rng.int(50_000), rng.int(50_000), rng.int(50_000)];
		let b = [rng.int(50_000), rng.int(50_000), rng.int(50_000)];
		let c = [rng.int(50_000), rng.int(50_000), rng.int(50_000)];
		let d = [rng.int(50_000), rng.int(50_000), rng.int(50_000)];
		assert_eq!(
			sgn(orient3d_exact(f(a), f(b), f(c), f(d))),
			orient3d_i128(a, b, c, d),
			"exact orient3d disagrees with i128 on a={a:?} b={b:?} c={c:?} d={d:?}"
		);
	}
}

#[test]
fn filtered_orient3d_always_agrees_with_the_exact_sign() {
	let mut rng = Lcg(0xc0ff_ee00_1357_2468);
	for _ in 0..50_000 {
		let s = [1.0, 1e-5, 1e5][(rng.next() % 3) as usize];
		let g = |r: &mut Lcg| (r.int(500) as f64) * s + (r.int(500) as f64) * s * 1e-12;
		let p = |r: &mut Lcg| [g(r), g(r), g(r)];
		let (a, b, c, d) = (p(&mut rng), p(&mut rng), p(&mut rng), p(&mut rng));
		assert_eq!(sgn(orient3d(a, b, c, d)), sgn(orient3d_exact(a, b, c, d)), "orient3d filter/exact mismatch");
	}
}

#[test]
fn orient3d_is_antisymmetric_where_naive_f64_is_not() {
	fn naive(a: [f64; 3], b: [f64; 3], c: [f64; 3], d: [f64; 3]) -> f64 {
		let u = [a[0] - d[0], a[1] - d[1], a[2] - d[2]];
		let v = [b[0] - d[0], b[1] - d[1], b[2] - d[2]];
		let w = [c[0] - d[0], c[1] - d[1], c[2] - d[2]];
		u[0] * (v[1] * w[2] - v[2] * w[1]) - u[1] * (v[0] * w[2] - v[2] * w[0]) + u[2] * (v[0] * w[1] - v[1] * w[0])
	}
	// Swapping the first two points negates the determinant, so the signs must be
	// opposite. Build near-coplanar quadruples (d on the a,b,c plane, nudged 1 ULP)
	// and count how often each predicate breaks the invariant.
	let mut rng = Lcg(0x1111_2222_3333_4444);
	let nudge = |x: f64| f64::from_bits(x.to_bits() + 1);
	let (mut naive_bad, mut exact_bad) = (0usize, 0usize);
	for _ in 0..60_000 {
		let a = [rng.int(32) as f64 + 0.5, rng.int(32) as f64 + 0.5, rng.int(32) as f64 + 0.5];
		let b = [rng.int(32) as f64 + 0.5, rng.int(32) as f64 + 0.5, rng.int(32) as f64 + 0.5];
		let c = [rng.int(32) as f64 + 0.5, rng.int(32) as f64 + 0.5, rng.int(32) as f64 + 0.5];
		// d on the plane of a,b,c (barycentric), then nudged just off it.
		let (s, t) = (0.3, 0.3);
		let mut d = [
			a[0] + s * (b[0] - a[0]) + t * (c[0] - a[0]),
			a[1] + s * (b[1] - a[1]) + t * (c[1] - a[1]),
			a[2] + s * (b[2] - a[2]) + t * (c[2] - a[2]),
		];
		d[2] = nudge(d[2]);
		if sgn(naive(a, b, c, d)) != -sgn(naive(b, a, c, d)) {
			naive_bad += 1;
		}
		if sgn(orient3d(a, b, c, d)) != -sgn(orient3d(b, a, c, d)) {
			exact_bad += 1;
		}
	}
	assert_eq!(exact_bad, 0, "exact orient3d violated antisymmetry {exact_bad} times");
	assert!(naive_bad > 0, "test did not reach the near-coplanar regime where naive f64 fails");
}

/// Independent exact in-circle reference (diff-form determinant) in i128;
/// positive ⇒ `d` inside the circle through CCW `a, b, c`.
fn incircle_i128(a: [i64; 2], b: [i64; 2], c: [i64; 2], d: [i64; 2]) -> i32 {
	let lift = |p: [i64; 2]| -> (i128, i128, i128) {
		let (x, y) = (p[0] as i128 - d[0] as i128, p[1] as i128 - d[1] as i128);
		(x, y, x * x + y * y)
	};
	let (ax, ay, al) = lift(a);
	let (bx, by, bl) = lift(b);
	let (cx, cy, cl) = lift(c);
	let det = ax * (by * cl - bl * cy) - ay * (bx * cl - bl * cx) + al * (bx * cy - by * cx);
	det.signum() as i32
}

#[test]
fn incircle_exact_matches_an_independent_i128_determinant() {
	// Exercises the full degree-4 lifted expansion against i128 ground truth.
	let mut rng = Lcg(0xdead_beef_1234_5678);
	let f = |v: [i64; 2]| [v[0] as f64, v[1] as f64];
	// 5k cases: each call runs the full ~384-component exact expansion (O(n²)).
	for _ in 0..5_000 {
		let a = [rng.int(5_000), rng.int(5_000)];
		let b = [rng.int(5_000), rng.int(5_000)];
		let c = [rng.int(5_000), rng.int(5_000)];
		let d = [rng.int(5_000), rng.int(5_000)];
		assert_eq!(
			sgn(incircle_exact(f(a), f(b), f(c), f(d))),
			incircle_i128(a, b, c, d),
			"exact incircle disagrees with i128 on a={a:?} b={b:?} c={c:?} d={d:?}"
		);
	}
}

#[test]
fn filtered_incircle_agrees_with_exact_and_uses_the_inside_positive_convention() {
	let mut rng = Lcg(0x5151_2727_9393_0606);
	for _ in 0..5_000 {
		let s = [1.0, 1e-4, 1e4][(rng.next() % 3) as usize];
		let g = |r: &mut Lcg| (r.int(800) as f64) * s + (r.int(800) as f64) * s * 1e-11;
		let p = |r: &mut Lcg| [g(r), g(r)];
		let (a, b, c, d) = (p(&mut rng), p(&mut rng), p(&mut rng), p(&mut rng));
		assert_eq!(sgn(incircle(a, b, c, d)), sgn(incircle_exact(a, b, c, d)), "incircle filter/exact mismatch");
	}
	// Convention: for a CCW triangle, a point inside the circumcircle is positive.
	let inside = incircle([0.0, 0.0], [1.0, 0.0], [0.0, 1.0], [0.25, 0.25]);
	let outside = incircle([0.0, 0.0], [1.0, 0.0], [0.0, 1.0], [2.0, 2.0]);
	let on = incircle_exact([0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]); // unit square corners → cocircular
	assert_eq!((sgn(inside), sgn(outside), sgn(on)), (1, -1, 0), "inside/outside/cocircular convention");
}

#[test]
fn sign_algebra_holds_exactly() {
	// Collinear → 0; swapping two vertices flips the sign; a clear CCW triangle is positive.
	let collinear = orient2d_exact([0.0, 0.0], [2.0, 2.0], [5.0, 5.0]);
	let ccw = orient2d([0.0, 0.0], [1.0, 0.0], [0.0, 1.0]);
	let swapped = orient2d([0.0, 0.0], [0.0, 1.0], [1.0, 0.0]);
	assert_eq!((sgn(collinear), sgn(ccw), sgn(swapped)), (0, 1, -1));
}
