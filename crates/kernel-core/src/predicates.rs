// Copyright (c) LMCAD. Licensed under the MIT License.

//! Exact geometric predicates (adaptive-precision, after Shewchuk).
//!
//! The sign of an orientation determinant decides which side of a line a point
//! lies on — and the whole correctness of arrangements, ear-clipping, convex
//! hulls and boolean classification rests on every such sign being *consistent*.
//! A naive `f64` determinant rounds its intermediate products, so on nearly
//! degenerate input it can return the wrong sign (or zero for a non-zero
//! determinant), breaking the geometric invariants the algorithms above assume
//! (e.g. `orient(a,b,c)` and the cyclic `orient(b,c,a)` disagreeing). That class
//! of failure is the single biggest gap between an epsilon-based prototype and a
//! production kernel.
//!
//! [`orient2d`] returns a value whose **sign is exact**. It first tries a fast
//! `f64` evaluation guarded by a conservative error bound; only when the result
//! is too close to zero to trust does it fall back to [`orient2d_exact`], which
//! evaluates the determinant in exact floating-point *expansions* (each product
//! is split into a non-overlapping (high, low) pair with no rounding, via fused
//! multiply-add, and the terms are summed without error). The expansion toolkit
//! ([`two_sum`], [`two_product`], grow-expansion summation) is shared so that
//! `orient3d` / `incircle` can be added on the same exact foundation.

/// Unit roundoff for `f64` (`2⁻⁵³`).
const EPSILON: f64 = 1.110_223_024_625_156_5e-16;
/// Conservative static error bound for the 2-D orientation filter
/// (`(3 + 16·ε)·ε`, after Shewchuk). If `|det|` clears this multiple of the
/// magnitude sum, the fast `f64` sign is certain.
const CCWERRBOUND_A: f64 = (3.0 + 16.0 * EPSILON) * EPSILON;
/// Conservative static error bound for the 3-D orientation filter
/// (`(7 + 56·ε)·ε`, after Shewchuk).
const O3DERRBOUND_A: f64 = (7.0 + 56.0 * EPSILON) * EPSILON;
/// Conservative static error bound for the 2-D in-circle filter
/// (`(10 + 96·ε)·ε`, after Shewchuk).
const ICCERRBOUND_A: f64 = (10.0 + 96.0 * EPSILON) * EPSILON;

/// Exact sum `a + b = x + e`: `x` is the rounded sum, `e` the exact roundoff.
/// Knuth's TwoSum — valid for all finite `a`, `b` with no assumption on order.
#[inline]
pub fn two_sum(a: f64, b: f64) -> (f64, f64) {
	let x = a + b;
	let bv = x - a;
	let av = x - bv;
	let br = b - bv;
	let ar = a - av;
	(x, ar + br)
}

/// Exact product `a · b = x + e`: `x` is the rounded product, `e` the exact
/// roundoff, recovered with a fused multiply-add (`e = fma(a, b, -x)`).
#[inline]
pub fn two_product(a: f64, b: f64) -> (f64, f64) {
	let x = a * b;
	let e = a.mul_add(b, -x);
	(x, e)
}

/// Exact sign of the value represented by the sum of `components` (each an
/// arbitrary `f64`, typically the high/low halves of [`two_product`] results).
///
/// The components are merged one at a time into a single non-overlapping
/// floating-point *expansion* (Shewchuk's grow-expansion: a chain of [`two_sum`]s
/// that pushes the exact roundoff down and carries the running sum up). In a
/// non-overlapping expansion ordered by increasing magnitude, the most
/// significant non-zero term carries the sign of the whole — so scanning from the
/// top for the first non-zero component yields the exact sign (`-1`, `0`, `1`).
fn expansion_sign(components: &[f64]) -> i32 {
	let mut e: Vec<f64> = Vec::with_capacity(components.len() + 1);
	for (i, &c) in components.iter().enumerate() {
		if i == 0 {
			e.push(c);
			continue;
		}
		// grow-expansion: add the single scalar `c` to the expansion `e`.
		let mut q = c;
		let mut h = Vec::with_capacity(e.len() + 1);
		for &ei in &e {
			let (qn, hi) = two_sum(q, ei);
			h.push(hi);
			q = qn;
		}
		h.push(q);
		e = h;
	}
	for &v in e.iter().rev() {
		if v > 0.0 {
			return 1;
		}
		if v < 0.0 {
			return -1;
		}
	}
	0
}

/// Orientation of the triangle `(pa, pb, pc)`, evaluated **exactly**.
///
/// Returns `+1` if the points are counter-clockwise, `-1` if clockwise, `0` if
/// exactly collinear. The determinant is expanded into products of the original
/// coordinates (no inexact coordinate subtraction), each made exact by
/// [`two_product`], then summed exactly by [`expansion_sign`].
pub fn orient2d_exact(pa: [f64; 2], pb: [f64; 2], pc: [f64; 2]) -> f64 {
	let [ax, ay] = pa;
	let [bx, by] = pb;
	let [cx, cy] = pc;
	// det = ax·by − ax·cy − ay·bx + ay·cx + bx·cy − by·cx
	let terms: [((f64, f64), bool); 6] = [
		(two_product(ax, by), false),
		(two_product(ax, cy), true),
		(two_product(ay, bx), true),
		(two_product(ay, cx), false),
		(two_product(bx, cy), false),
		(two_product(by, cx), true),
	];
	let mut comps = Vec::with_capacity(12);
	for ((hi, lo), neg) in terms {
		if neg {
			comps.push(-hi);
			comps.push(-lo);
		} else {
			comps.push(hi);
			comps.push(lo);
		}
	}
	expansion_sign(&comps) as f64
}

/// Orientation of the triangle `(pa, pb, pc)` with an **exact sign**.
///
/// Positive when the points are counter-clockwise, negative when clockwise, zero
/// when exactly collinear. A fast filtered `f64` determinant is returned when its
/// magnitude provably exceeds the rounding error; otherwise the result falls back
/// to [`orient2d_exact`]. Only the sign is guaranteed — the magnitude is the
/// `f64` estimate on the fast path and `±1` on the exact path.
pub fn orient2d(pa: [f64; 2], pb: [f64; 2], pc: [f64; 2]) -> f64 {
	let detleft = (pa[0] - pc[0]) * (pb[1] - pc[1]);
	let detright = (pa[1] - pc[1]) * (pb[0] - pc[0]);
	let det = detleft - detright;
	let detsum = detleft.abs() + detright.abs();
	// |det| ≤ detsum always; when detsum is 0 the filter falls through to exact.
	let errbound = CCWERRBOUND_A * detsum;
	if det.abs() > errbound {
		return det;
	}
	orient2d_exact(pa, pb, pc)
}

/// Push the exact components of `s · (x · y · z)` (`s = ±1`) onto `comps`. The
/// triple product is split into four non-overlapping `f64`s with no rounding:
/// `x·y = p + q` (exact via [`two_product`]), then `(p + q)·z` is two more exact
/// products. Multiplying by `s = ±1` is exact (a sign flip).
#[inline]
fn push_triple(comps: &mut Vec<f64>, x: f64, y: f64, z: f64, s: f64) {
	let (p, q) = two_product(x, y);
	let (pz, pze) = two_product(p, z);
	let (qz, qze) = two_product(q, z);
	comps.push(s * pz);
	comps.push(s * pze);
	comps.push(s * qz);
	comps.push(s * qze);
}

/// Push the six signed triple-products of `s · det[p; q; r]` onto `comps`, where
/// `det[p; q; r] = px(qy·rz − qz·ry) − py(qx·rz − qz·rx) + pz(qx·ry − qy·rx)`.
fn add_det3(comps: &mut Vec<f64>, p: [f64; 3], q: [f64; 3], r: [f64; 3], s: f64) {
	push_triple(comps, p[0], q[1], r[2], s);
	push_triple(comps, p[0], q[2], r[1], -s);
	push_triple(comps, p[1], q[0], r[2], -s);
	push_triple(comps, p[1], q[2], r[0], s);
	push_triple(comps, p[2], q[0], r[1], s);
	push_triple(comps, p[2], q[1], r[0], -s);
}

/// Orientation of `pd` relative to the plane through `(pa, pb, pc)`, evaluated
/// **exactly**.
///
/// Returns a positive value when `pd` lies below the plane (i.e. `pa, pb, pc`
/// appear counter-clockwise seen from `pd`'s far side), negative when above, and
/// `0` when the four points are exactly coplanar. The determinant equals
/// `det[pa−pd; pb−pd; pc−pd]`; it is evaluated as the 4×4 homogeneous determinant
/// `−det[b;c;d] + det[a;c;d] − det[a;b;d] + det[a;b;c]` expanded into exact
/// triple-products of the original coordinates, then summed by [`expansion_sign`]
/// — so no inexact coordinate subtraction enters the exact path.
pub fn orient3d_exact(pa: [f64; 3], pb: [f64; 3], pc: [f64; 3], pd: [f64; 3]) -> f64 {
	let mut comps = Vec::with_capacity(96);
	add_det3(&mut comps, pb, pc, pd, -1.0);
	add_det3(&mut comps, pa, pc, pd, 1.0);
	add_det3(&mut comps, pa, pb, pd, -1.0);
	add_det3(&mut comps, pa, pb, pc, 1.0);
	expansion_sign(&comps) as f64
}

/// Push the exact components of `s · (a · b · c · d)` (`s = ±1`) onto `comps`: a
/// degree-4 monomial split into eight non-overlapping `f64`s with no rounding.
#[inline]
fn push_quad(comps: &mut Vec<f64>, a: f64, b: f64, c: f64, d: f64, s: f64) {
	let (p, q) = two_product(a, b); // a·b = p + q
	let (r, t) = two_product(c, d); // c·d = r + t
	for (x, y) in [(p, r), (p, t), (q, r), (q, t)] {
		let (hi, lo) = two_product(x, y);
		comps.push(s * hi);
		comps.push(s * lo);
	}
}

/// Push the exact components of `s · m · n · (ux² + uy²)` onto `comps` — the
/// "lifted" term that appears in the in-circle determinant (the paraboloid lift of
/// the point `(ux, uy)`).
#[inline]
fn push_lifted(comps: &mut Vec<f64>, m: f64, n: f64, ux: f64, uy: f64, s: f64) {
	push_quad(comps, m, n, ux, ux, s);
	push_quad(comps, m, n, uy, uy, s);
}

/// Push `s · det[ (px,py,pL); (qx,qy,qL); (rx,ry,rL) ]` where the third column is
/// the paraboloid lift `·L = ·x² + ·y²` — the 3×3 minor of the lifted in-circle
/// determinant.
fn add_det3_lifted(comps: &mut Vec<f64>, p: [f64; 2], q: [f64; 2], r: [f64; 2], s: f64) {
	// px·qy·rL − px·ry·qL − py·qx·rL + py·rx·qL + qx·ry·pL − qy·rx·pL
	push_lifted(comps, p[0], q[1], r[0], r[1], s);
	push_lifted(comps, p[0], r[1], q[0], q[1], -s);
	push_lifted(comps, p[1], q[0], r[0], r[1], -s);
	push_lifted(comps, p[1], r[0], q[0], q[1], s);
	push_lifted(comps, q[0], r[1], p[0], p[1], s);
	push_lifted(comps, q[1], r[0], p[0], p[1], -s);
}

/// In-circle test of `pd` against the circle through `(pa, pb, pc)`, evaluated
/// **exactly**.
///
/// Returns a positive value when `pd` lies inside the circumcircle (for a
/// counter-clockwise `pa, pb, pc`), negative when outside, `0` when the four points
/// are exactly cocircular. Evaluated as the 4×4 paraboloid-lift determinant
/// `−det[b,c,d] + det[a,c,d] − det[a,b,d] + det[a,b,c]` expanded into exact
/// degree-4 monomials of the original coordinates, then summed by [`expansion_sign`].
pub fn incircle_exact(pa: [f64; 2], pb: [f64; 2], pc: [f64; 2], pd: [f64; 2]) -> f64 {
	let mut comps = Vec::with_capacity(384);
	add_det3_lifted(&mut comps, pb, pc, pd, -1.0);
	add_det3_lifted(&mut comps, pa, pc, pd, 1.0);
	add_det3_lifted(&mut comps, pa, pb, pd, -1.0);
	add_det3_lifted(&mut comps, pa, pb, pc, 1.0);
	expansion_sign(&comps) as f64
}

/// In-circle test of `pd` against the circle through `(pa, pb, pc)` with an
/// **exact sign** (positive inside, negative outside, zero cocircular — for a
/// counter-clockwise `pa, pb, pc`).
///
/// A fast filtered `f64` determinant is returned when its magnitude provably
/// exceeds the rounding error; otherwise the result falls back to
/// [`incircle_exact`]. Only the sign is guaranteed.
pub fn incircle(pa: [f64; 2], pb: [f64; 2], pc: [f64; 2], pd: [f64; 2]) -> f64 {
	let adx = pa[0] - pd[0];
	let ady = pa[1] - pd[1];
	let bdx = pb[0] - pd[0];
	let bdy = pb[1] - pd[1];
	let cdx = pc[0] - pd[0];
	let cdy = pc[1] - pd[1];

	let bdxcdy = bdx * cdy;
	let cdxbdy = cdx * bdy;
	let alift = adx * adx + ady * ady;
	let cdxady = cdx * ady;
	let adxcdy = adx * cdy;
	let blift = bdx * bdx + bdy * bdy;
	let adxbdy = adx * bdy;
	let bdxady = bdx * ady;
	let clift = cdx * cdx + cdy * cdy;

	let det = alift * (bdxcdy - cdxbdy) + blift * (cdxady - adxcdy) + clift * (adxbdy - bdxady);
	let permanent = (bdxcdy.abs() + cdxbdy.abs()) * alift
		+ (cdxady.abs() + adxcdy.abs()) * blift
		+ (adxbdy.abs() + bdxady.abs()) * clift;
	let errbound = ICCERRBOUND_A * permanent;
	if det.abs() > errbound {
		return det;
	}
	incircle_exact(pa, pb, pc, pd)
}

/// Orientation of `pd` relative to the plane through `(pa, pb, pc)` with an
/// **exact sign** (positive below, negative above, zero coplanar).
///
/// A fast filtered `f64` determinant is returned when its magnitude provably
/// exceeds the rounding error; otherwise the result falls back to
/// [`orient3d_exact`]. Only the sign is guaranteed.
pub fn orient3d(pa: [f64; 3], pb: [f64; 3], pc: [f64; 3], pd: [f64; 3]) -> f64 {
	let adx = pa[0] - pd[0];
	let ady = pa[1] - pd[1];
	let adz = pa[2] - pd[2];
	let bdx = pb[0] - pd[0];
	let bdy = pb[1] - pd[1];
	let bdz = pb[2] - pd[2];
	let cdx = pc[0] - pd[0];
	let cdy = pc[1] - pd[1];
	let cdz = pc[2] - pd[2];

	let bdxcdy = bdx * cdy;
	let cdxbdy = cdx * bdy;
	let cdxady = cdx * ady;
	let adxcdy = adx * cdy;
	let adxbdy = adx * bdy;
	let bdxady = bdx * ady;

	let det = adz * (bdxcdy - cdxbdy) + bdz * (cdxady - adxcdy) + cdz * (adxbdy - bdxady);
	let permanent = (bdxcdy.abs() + cdxbdy.abs()) * adz.abs()
		+ (cdxady.abs() + adxcdy.abs()) * bdz.abs()
		+ (adxbdy.abs() + bdxady.abs()) * cdz.abs();
	let errbound = O3DERRBOUND_A * permanent;
	if det.abs() > errbound {
		return det;
	}
	orient3d_exact(pa, pb, pc, pd)
}
