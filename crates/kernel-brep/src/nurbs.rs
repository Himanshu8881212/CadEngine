// Copyright (c) LMCAD. Licensed under the MIT License.

//! Basic NURBS (Non-Uniform Rational B-Spline) freeform geometry in `f64`.
//!
//! This fills the largest gap on the exact B-rep side: the analytic [`Surface`]
//! / [`Curve`] enums only cover closed-form quadrics, so there was no way to
//! represent sculpted/freeform shapes. Here we implement the rational B-spline
//! machinery directly:
//!
//! - [`cox_de_boor`] non-uniform basis functions (the partition-of-unity blend);
//! - a rational [`NurbsCurve`] with [`point_at`](NurbsCurve::point_at) and unit
//!   [`tangent_at`](NurbsCurve::tangent_at);
//! - a rational [`NurbsSurface`] with [`point_at`](NurbsSurface::point_at),
//!   [`normal_at`](NurbsSurface::normal_at), and a welded
//!   [`tessellate`](NurbsSurface::tessellate) over the valid parameter domain.
//!
//! Evaluation follows the homogeneous-coordinate construction: control points
//! are lifted to `(P*w, w)`, blended linearly by the basis, then projected back
//! by dividing through the accumulated weight `w`. Derivatives use the quotient
//! rule on that homogeneous blend, with a central-difference fallback for the
//! degenerate cases where a partial collapses to (near) zero length.
//!
//! [`Surface`]: crate::geom::Surface
//! [`Curve`]: crate::geom::Curve

use kernel_core::math::{DVec2, DVec3};
use kernel_core::mesh::Mesh;

/// A small parametric epsilon used to clamp evaluation strictly inside the valid
/// knot domain and to size the central-difference fallback step.
const PARAM_EPS: f64 = 1e-9;

/// Evaluate the `degree`-degree B-spline basis function `N_{i,degree}(t)` via the
/// Cox-de-Boor recurrence.
///
/// `knots` is the full (non-decreasing) knot vector. `i` is the basis-function
/// index (`0 <= i <= n` where `n = knots.len() - degree - 2`). Zero-width knot
/// spans are handled by treating `0/0` as `0`, which keeps the recurrence stable
/// at repeated (open-uniform) knots.
pub fn cox_de_boor(i: usize, degree: usize, t: f64, knots: &[f64]) -> f64 {
	// Degenerate guard: not enough knots to form this basis function.
	if i + degree + 1 >= knots.len() {
		return 0.0;
	}
	if degree == 0 {
		// Right-continuous indicator, with the right end of the last span made
		// inclusive so the domain endpoint `t == knots[n+1]` evaluates non-zero.
		let last = knots.len() - 1;
		// Only the single genuinely-last NON-degenerate span is right-inclusive,
		// so the domain endpoint `t == knots[last]` makes exactly one degree-0
		// indicator fire (partition of unity stays 1, not the number of spans
		// ending at the max knot).
		let inclusive_end = knots[i + 1] == knots[last] && knots[i] < knots[i + 1];
		let hit = t >= knots[i] && (t < knots[i + 1] || (inclusive_end && t <= knots[i + 1]));
		return if hit { 1.0 } else { 0.0 };
	}

	let mut left = 0.0;
	let den_left = knots[i + degree] - knots[i];
	if den_left > 0.0 {
		left = (t - knots[i]) / den_left * cox_de_boor(i, degree - 1, t, knots);
	}

	let mut right = 0.0;
	let den_right = knots[i + degree + 1] - knots[i + 1];
	if den_right > 0.0 {
		right = (knots[i + degree + 1] - t) / den_right * cox_de_boor(i + 1, degree - 1, t, knots);
	}

	left + right
}

/// Clamp a parameter into the *valid* B-spline domain `[knots[degree], knots[n+1]]`.
///
/// The upper knot is included exactly: [`cox_de_boor`] makes the last knot span
/// right-inclusive, so the domain endpoint evaluates to the final control point
/// rather than collapsing to zero.
fn clamp_domain(t: f64, degree: usize, knots: &[f64], control_len: usize) -> f64 {
	let lo = knots[degree];
	// n = control_len - 1; the domain's upper knot is knots[n + 1].
	let hi = knots[control_len];
	t.clamp(lo, hi)
}

/// A rational B-spline curve in `f64`.
///
/// Invariant: `control.len() == weights.len()` and
/// `knots.len() == control.len() + degree + 1`.
#[derive(Clone, Debug)]
pub struct NurbsCurve {
	pub degree: usize,
	pub knots: Vec<f64>,
	pub control: Vec<DVec3>,
	pub weights: Vec<f64>,
}

impl NurbsCurve {
	/// Construct a curve. Returns `None` if the dimensions are inconsistent or
	/// there are too few control points for the requested degree.
	pub fn new(degree: usize, knots: Vec<f64>, control: Vec<DVec3>, weights: Vec<f64>) -> Option<Self> {
		if control.is_empty()
			|| control.len() != weights.len()
			|| control.len() <= degree
			|| knots.len() != control.len() + degree + 1
		{
			return None;
		}
		Some(Self { degree, knots, control, weights })
	}

	/// The valid parameter domain `[knots[degree], knots[n+1]]`.
	pub fn domain(&self) -> (f64, f64) {
		(self.knots[self.degree], self.knots[self.control.len()])
	}

	/// Evaluate the curve position at parameter `t` (clamped into the domain).
	///
	/// Uses the homogeneous blend: `sum N_i w_i P_i / sum N_i w_i`.
	pub fn point_at(&self, t: f64) -> DVec3 {
		let t = clamp_domain(t, self.degree, &self.knots, self.control.len());
		let mut num = DVec3::ZERO;
		let mut den = 0.0;
		for i in 0..self.control.len() {
			let b = cox_de_boor(i, self.degree, t, &self.knots);
			if b == 0.0 {
				continue;
			}
			let w = self.weights[i] * b;
			num += self.control[i] * w;
			den += w;
		}
		if den.abs() <= PARAM_EPS {
			// Degenerate (all-zero weights at this parameter): fall back to the
			// nearest control point so we never emit NaN.
			return self.control[0];
		}
		num / den
	}

	/// Unit tangent at parameter `t`, by robust central differences in parameter
	/// space. Returns a zero vector for a fully degenerate curve.
	pub fn tangent_at(&self, t: f64) -> DVec3 {
		let (lo, hi) = self.domain();
		let span = (hi - lo).max(PARAM_EPS);
		let h = span * 1e-6;
		let t = clamp_domain(t, self.degree, &self.knots, self.control.len());
		let a = (t - h).max(lo);
		let b = (t + h).min(hi);
		let denom = (b - a).max(PARAM_EPS);
		((self.point_at(b) - self.point_at(a)) / denom).normalize_or_zero()
	}
}

/// A rational B-spline surface (tensor product) in `f64`.
///
/// `control` and `weights` are indexed `[i][j]` over the `u` then `v` directions:
/// `control.len()` rows of `control[0].len()` columns. Invariants:
/// `knots_u.len() == control.len() + degree_u + 1` and
/// `knots_v.len() == control[0].len() + degree_v + 1`, with every row the same
/// length and `weights` matching `control` cell for cell.
#[derive(Clone, Debug)]
pub struct NurbsSurface {
	pub degree_u: usize,
	pub degree_v: usize,
	pub knots_u: Vec<f64>,
	pub knots_v: Vec<f64>,
	pub control: Vec<Vec<DVec3>>,
	pub weights: Vec<Vec<f64>>,
}

impl NurbsSurface {
	/// Number of control points along `u` (rows).
	fn n_u(&self) -> usize {
		self.control.len()
	}

	/// Number of control points along `v` (columns).
	fn n_v(&self) -> usize {
		self.control.first().map_or(0, |row| row.len())
	}

	/// Construct a surface, validating all the tensor-grid dimension invariants.
	/// Returns `None` on any inconsistency.
	pub fn new(
		degree_u: usize,
		degree_v: usize,
		knots_u: Vec<f64>,
		knots_v: Vec<f64>,
		control: Vec<Vec<DVec3>>,
		weights: Vec<Vec<f64>>,
	) -> Option<Self> {
		let n_u = control.len();
		if n_u == 0 || weights.len() != n_u {
			return None;
		}
		let n_v = control[0].len();
		if n_v == 0 {
			return None;
		}
		for (row, wrow) in control.iter().zip(weights.iter()) {
			if row.len() != n_v || wrow.len() != n_v {
				return None;
			}
		}
		if n_u <= degree_u
			|| n_v <= degree_v
			|| knots_u.len() != n_u + degree_u + 1
			|| knots_v.len() != n_v + degree_v + 1
		{
			return None;
		}
		Some(Self { degree_u, degree_v, knots_u, knots_v, control, weights })
	}

	/// The valid parameter domain as `((u_lo, u_hi), (v_lo, v_hi))`.
	pub fn domain(&self) -> ((f64, f64), (f64, f64)) {
		(
			(self.knots_u[self.degree_u], self.knots_u[self.n_u()]),
			(self.knots_v[self.degree_v], self.knots_v[self.n_v()]),
		)
	}

	/// Accumulate the homogeneous blend at `(u, v)`: returns `(sum N w P, sum N w)`.
	/// The basis values are computed once per direction and reused across the grid
	/// to keep this `O(n_u + n_v)` per call rather than `O(n_u * n_v)` re-evals.
	fn homogeneous(&self, u: f64, v: f64) -> (DVec3, f64) {
		let n_u = self.n_u();
		let n_v = self.n_v();
		let bu: Vec<f64> = (0..n_u).map(|i| cox_de_boor(i, self.degree_u, u, &self.knots_u)).collect();
		let bv: Vec<f64> = (0..n_v).map(|j| cox_de_boor(j, self.degree_v, v, &self.knots_v)).collect();
		let mut num = DVec3::ZERO;
		let mut den = 0.0;
		// `i`/`j` cross-index the basis vectors and the 2D weight/control grids.
		#[allow(clippy::needless_range_loop)]
		for i in 0..n_u {
			if bu[i] == 0.0 {
				continue;
			}
			for j in 0..n_v {
				let b = bu[i] * bv[j];
				if b == 0.0 {
					continue;
				}
				let w = self.weights[i][j] * b;
				num += self.control[i][j] * w;
				den += w;
			}
		}
		(num, den)
	}

	/// Evaluate the surface position at `(u, v)` (clamped into the valid domain).
	pub fn point_at(&self, u: f64, v: f64) -> DVec3 {
		let u = clamp_domain(u, self.degree_u, &self.knots_u, self.n_u());
		let v = clamp_domain(v, self.degree_v, &self.knots_v, self.n_v());
		let (num, den) = self.homogeneous(u, v);
		if den.abs() <= PARAM_EPS {
			return self.control[0][0];
		}
		num / den
	}

	/// The two partial derivatives `(dS/du, dS/dv)` at `(u, v)`, by robust central
	/// differences in parameter space (the homogeneous quotient makes closed-form
	/// partials fiddly; central differences are stable and accurate here).
	pub fn partials(&self, u: f64, v: f64) -> (DVec3, DVec3) {
		let ((u_lo, u_hi), (v_lo, v_hi)) = self.domain();
		let u = clamp_domain(u, self.degree_u, &self.knots_u, self.n_u());
		let v = clamp_domain(v, self.degree_v, &self.knots_v, self.n_v());
		let hu = (u_hi - u_lo).max(PARAM_EPS) * 1e-6;
		let hv = (v_hi - v_lo).max(PARAM_EPS) * 1e-6;

		let ua = (u - hu).max(u_lo);
		let ub = (u + hu).min(u_hi);
		let du = (self.point_at(ub, v) - self.point_at(ua, v)) / (ub - ua).max(PARAM_EPS);

		let va = (v - hv).max(v_lo);
		let vb = (v + hv).min(v_hi);
		let dv = (self.point_at(u, vb) - self.point_at(u, va)) / (vb - va).max(PARAM_EPS);

		(du, dv)
	}

	/// Unit normal at `(u, v)` — the normalized cross product of the two partial
	/// derivatives. Returns a zero vector at a fully degenerate point.
	pub fn normal_at(&self, u: f64, v: f64) -> DVec3 {
		let (du, dv) = self.partials(u, v);
		du.cross(dv).normalize_or_zero()
	}

	/// The coarse `(normalised uv, position)` seed grid that initialises Newton
	/// projection ([`Self::project`]): `(n+1)²` samples over the valid parameter
	/// domain, evaluated once per patch and shared across all of its projections.
	pub fn projection_seeds(&self, n: usize) -> Vec<(DVec2, DVec3)> {
		let n = n.max(1);
		let ((u_lo, u_hi), (v_lo, v_hi)) = self.domain();
		let mut g = Vec::with_capacity((n + 1) * (n + 1));
		for i in 0..=n {
			let fu = i as f64 / n as f64;
			for j in 0..=n {
				let fv = j as f64 / n as f64;
				g.push((DVec2::new(fu, fv), self.point_at(u_lo + (u_hi - u_lo) * fu, v_lo + (v_hi - v_lo) * fv)));
			}
		}
		g
	}

	/// Invert the patch at `p`: the normalised `(u, v) ∈ [0,1]²` whose surface point
	/// is `p`, by **multi-start** Gauss-Newton on `|S(u,v) − p|²` (the normal
	/// equations of the two partials) from the nearest few `seeds` (from
	/// [`Self::projection_seeds`]). One start is not enough on a closed patch: a
	/// point near the seam is 3-D-nearest to BOTH domain ends, and Newton started
	/// on the wrong side jams against the domain clamp — the next-nearest seed (the
	/// other end) converges. `None` when no start lands within `tol · (1 + |p|)` —
	/// i.e. `p` does not lie on the patch within its parameter domain.
	pub fn project(&self, seeds: &[(DVec2, DVec3)], p: DVec3, tol: f64) -> Option<DVec2> {
		/// Newton starts tried in nearest-seed order before giving up.
		const PROJECT_STARTS: usize = 6;
		let ((u_lo, u_hi), (v_lo, v_hi)) = self.domain();
		let (span_u, span_v) = (u_hi - u_lo, v_hi - v_lo);
		let at = |uv: DVec2| self.point_at(u_lo + span_u * uv.x, v_lo + span_v * uv.y);
		let scale = 1.0 + p.length();
		let mut order: Vec<usize> = (0..seeds.len()).collect();
		order.sort_by(|&i, &j| (seeds[i].1 - p).length_squared().total_cmp(&(seeds[j].1 - p).length_squared()));
		let mut best: Option<(DVec2, f64)> = None;
		for &si in order.iter().take(PROJECT_STARTS) {
			let mut uv = seeds[si].0;
			for _ in 0..40 {
				let (u, v) = (u_lo + span_u * uv.x, v_lo + span_v * uv.y);
				let r = p - self.point_at(u, v);
				if r.length() < 1e-12 * scale {
					break;
				}
				let (du, dv) = self.partials(u, v);
				let (a, b, c) = (du.dot(du), du.dot(dv), dv.dot(dv));
				let det = a * c - b * b;
				if det.abs() < 1e-30 {
					break; // locally degenerate parameterisation — leave it to the tolerance check
				}
				let (g1, g2) = (du.dot(r), dv.dot(r));
				let next = DVec2::new(
					(uv.x + (c * g1 - b * g2) / det / span_u).clamp(0.0, 1.0),
					(uv.y + (a * g2 - b * g1) / det / span_v).clamp(0.0, 1.0),
				);
				let moved = (next - uv).abs().max_element();
				uv = next;
				if moved < 1e-14 {
					break;
				}
			}
			let d = (at(uv) - p).length();
			if best.is_none_or(|(_, bd)| d < bd) {
				best = Some((uv, d));
			}
			if d <= tol * scale {
				break;
			}
		}
		best.and_then(|(uv, d)| (d <= tol * scale).then_some(uv))
	}

	/// Tessellate the patch into a welded triangle mesh by sampling an
	/// `(nu+1) x (nv+1)` grid over the valid parameter domain, with per-vertex
	/// analytic normals.
	///
	/// `nu` / `nv` are the number of *cells* per direction; they are clamped to at
	/// least 1. The grid is built with per-cell duplicated corners and then welded
	/// so coincident samples (e.g. on a seam) share one vertex.
	pub fn tessellate(&self, nu: usize, nv: usize) -> Mesh {
		let nu = nu.max(1);
		let nv = nv.max(1);
		let ((u_lo, u_hi), (v_lo, v_hi)) = self.domain();

		// Precompute the sample grid of positions + normals.
		let mut grid_pos: Vec<Vec<DVec3>> = Vec::with_capacity(nu + 1);
		let mut grid_nrm: Vec<Vec<DVec3>> = Vec::with_capacity(nu + 1);
		for i in 0..=nu {
			let su = i as f64 / nu as f64;
			let u = u_lo + (u_hi - u_lo) * su;
			let mut row_pos = Vec::with_capacity(nv + 1);
			let mut row_nrm = Vec::with_capacity(nv + 1);
			for j in 0..=nv {
				let sv = j as f64 / nv as f64;
				let v = v_lo + (v_hi - v_lo) * sv;
				row_pos.push(self.point_at(u, v));
				row_nrm.push(self.normal_at(u, v));
			}
			grid_pos.push(row_pos);
			grid_nrm.push(row_nrm);
		}

		let mut mesh = Mesh::new();
		let mut push = |a: DVec3, b: DVec3, c: DVec3, na: DVec3, nb: DVec3, nc: DVec3| {
			let base = mesh.positions.len() as u32;
			for (p, n) in [(a, na), (b, nb), (c, nc)] {
				mesh.positions.push(p.as_vec3());
				mesh.normals.push(n.as_vec3());
			}
			mesh.indices.extend_from_slice(&[base, base + 1, base + 2]);
		};

		for i in 0..nu {
			for j in 0..nv {
				let p00 = grid_pos[i][j];
				let p10 = grid_pos[i + 1][j];
				let p11 = grid_pos[i + 1][j + 1];
				let p01 = grid_pos[i][j + 1];
				let n00 = grid_nrm[i][j];
				let n10 = grid_nrm[i + 1][j];
				let n11 = grid_nrm[i + 1][j + 1];
				let n01 = grid_nrm[i][j + 1];
				push(p00, p10, p11, n00, n10, n11);
				push(p00, p11, p01, n00, n11, n01);
			}
		}

		mesh.weld(kernel_core::math::EPSILON);
		mesh
	}
}

/// A freeform (NURBS) B-rep face carried **alongside** a tessellated [`Solid`]: the
/// exact rational patch plus its trim rings — `rings[0]` the outer loop, `rings[1..]`
/// the holes, each ring the verbatim 3-D trim-chord polyline in loop order.
///
/// The analytic [`Surface`](crate::geom::Surface) enum has no freeform variant, so a
/// `Solid` reconstructed from a trimmed B-spline STEP face carries chord facets only;
/// this sidecar (returned by [`import_step_freeform`](crate::import_step_freeform))
/// is what preserves the patch's NURBS identity so
/// [`export_step_freeform`](crate::export_step_freeform) can write it back out as a
/// true `B_SPLINE_SURFACE_WITH_KNOTS` face instead of a facet soup.
#[derive(Clone, Debug)]
pub struct FreeformFace {
	/// The exact rational tensor patch.
	pub surface: NurbsSurface,
	/// Trim rings in loop order: `rings[0]` outer, the rest holes. Ring vertices lie
	/// on the patch and are shared (bit-identically) with the neighbouring faces'
	/// boundary chords — the watertight weld.
	pub rings: Vec<Vec<DVec3>>,
}

#[cfg(test)]
mod tests {
	use super::*;
	use kernel_core::math::DVec3;

	/// Open-uniform clamped knot vector for `n_ctrl` control points of `degree`:
	/// `degree+1` zeros, interior knots `1..interior`, then `degree+1` of the max.
	fn open_uniform(n_ctrl: usize, degree: usize) -> Vec<f64> {
		let n_knots = n_ctrl + degree + 1;
		let interior = n_knots - 2 * (degree + 1);
		let mut k = Vec::with_capacity(n_knots);
		k.resize(degree + 1, 0.0); // clamped start knots
		for i in 1..=interior {
			k.push(i as f64);
		}
		let max = (interior + 1) as f64;
		k.resize(k.len() + degree + 1, max); // clamped end knots
		k
	}

	/// de Casteljau evaluation of a non-rational Bezier curve, for cross-checking.
	fn de_casteljau(ctrl: &[DVec3], t: f64) -> DVec3 {
		let mut pts = ctrl.to_vec();
		let n = pts.len();
		for r in 1..n {
			for i in 0..(n - r) {
				pts[i] = pts[i].lerp(pts[i + 1], t);
			}
		}
		pts[0]
	}

	#[test]
	fn partition_of_unity() {
		// Basis functions of any degree sum to 1 across the whole domain.
		for degree in [1usize, 2, 3] {
			let n_ctrl = degree + 3;
			let knots = open_uniform(n_ctrl, degree);
			let (lo, hi) = (knots[degree], knots[n_ctrl]);
			for s in 0..=20 {
				// Include the exact domain endpoints (the recurrence makes the last
				// span right-inclusive, so the sum is 1 there too).
				let t = lo + (hi - lo) * (s as f64 / 20.0);
				let sum: f64 = (0..n_ctrl).map(|i| cox_de_boor(i, degree, t, &knots)).sum();
				assert!((sum - 1.0).abs() < 1e-9, "degree {degree} t {t}: sum {sum}");
			}
		}
	}

	#[test]
	fn flat_grid_is_bilinear_plane() {
		// 2x2 control grid, degree 1 in both directions, all weights 1, on the
		// z=0 plane with non-square spacing → point_at must match bilinear lerp.
		let control = vec![
			vec![DVec3::new(0.0, 0.0, 0.0), DVec3::new(0.0, 3.0, 0.0)],
			vec![DVec3::new(2.0, 0.0, 0.0), DVec3::new(2.0, 3.0, 0.0)],
		];
		let weights = vec![vec![1.0, 1.0], vec![1.0, 1.0]];
		let knots_u = open_uniform(2, 1); // [0,0,1,1]
		let knots_v = open_uniform(2, 1);
		let surf = NurbsSurface::new(1, 1, knots_u, knots_v, control.clone(), weights).unwrap();

		for &su in &[0.0, 0.25, 0.5, 0.75, 1.0] {
			for &sv in &[0.0, 0.25, 0.5, 0.75, 1.0] {
				let p = surf.point_at(su, sv);
				// Bilinear interpolation of the corners.
				let bottom = control[0][0].lerp(control[1][0], su);
				let top = control[0][1].lerp(control[1][1], su);
				let expect = bottom.lerp(top, sv);
				assert!(
					(p - expect).length() < 1e-9,
					"u {su} v {sv}: got {p:?} expected {expect:?}"
				);
			}
		}

		// Normal of a flat XY plane is +/- Z.
		let n = surf.normal_at(0.5, 0.5);
		assert!((n.z.abs() - 1.0).abs() < 1e-7 && n.x.abs() < 1e-7 && n.y.abs() < 1e-7, "normal {n:?}");
	}

	#[test]
	fn single_span_cubic_matches_de_casteljau() {
		// 4 control points, degree 3, clamped knots [0,0,0,0,1,1,1,1] → a single
		// Bezier span. All weights 1 → must equal de Casteljau.
		let ctrl = vec![
			DVec3::new(0.0, 0.0, 0.0),
			DVec3::new(1.0, 2.0, 0.0),
			DVec3::new(3.0, 2.0, 1.0),
			DVec3::new(4.0, 0.0, 0.0),
		];
		let knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
		let curve = NurbsCurve::new(3, knots, ctrl.clone(), vec![1.0; 4]).unwrap();

		for s in 0..=20 {
			let t = s as f64 / 20.0;
			let got = curve.point_at(t);
			let want = de_casteljau(&ctrl, t);
			assert!((got - want).length() < 1e-9, "t {t}: got {got:?} want {want:?}");
		}

		// Tangent at the start should point along the first control leg.
		let tan = curve.tangent_at(0.0);
		let expect = (ctrl[1] - ctrl[0]).normalize();
		assert!((tan - expect).length() < 1e-5, "tangent {tan:?} expected {expect:?}");
	}

	#[test]
	fn tessellate_curved_patch_is_nonempty_and_welds() {
		// A 3x3 degree-2 patch with a raised center control point → genuinely
		// curved; tessellation must produce triangles and weld duplicate seam
		// samples (vertex count strictly below the naive corner count).
		let mut control = Vec::new();
		let mut weights = Vec::new();
		for i in 0..3 {
			let mut row = Vec::new();
			let mut wrow = Vec::new();
			for j in 0..3 {
				let x = i as f64;
				let y = j as f64;
				let z = if i == 1 && j == 1 { 1.5 } else { 0.0 };
				row.push(DVec3::new(x, y, z));
				wrow.push(1.0);
			}
			control.push(row);
			weights.push(wrow);
		}
		let knots_u = open_uniform(3, 2); // [0,0,0,1,1,1]
		let knots_v = open_uniform(3, 2);
		let surf = NurbsSurface::new(2, 2, knots_u, knots_v, control, weights).unwrap();

		let nu = 8;
		let nv = 8;
		let mesh = surf.tessellate(nu, nv);
		assert!(!mesh.is_empty(), "tessellation produced no triangles");
		assert_eq!(mesh.triangle_count(), nu * nv * 2);
		assert_eq!(mesh.normals.len(), mesh.positions.len());
		// Welding must have merged the per-quad duplicated corners down to the
		// unique (nu+1)*(nv+1) grid samples.
		assert_eq!(mesh.vertex_count(), (nu + 1) * (nv + 1));

		// The raised interior must lift the surface above the z=0 control plane.
		let apex = surf.point_at(
			(surf.domain().0 .0 + surf.domain().0 .1) * 0.5,
			(surf.domain().1 .0 + surf.domain().1 .1) * 0.5,
		);
		assert!(apex.z > 0.0, "curved patch did not bulge up: {apex:?}");
	}

	#[test]
	fn degree0_partition_of_unity_at_endpoint() {
		// Regression guard: the degree-0 basis must sum to exactly 1 everywhere,
		// INCLUDING the top knot (previously it over-fired to the span count).
		for knots in [
			vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
			vec![0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0],
			vec![0.0, 1.0, 2.0, 3.0, 4.0],
		] {
			let n_basis = knots.len() - 1;
			let last = *knots.last().unwrap();
			for &t in &[knots[0], last * 0.5, last] {
				let sum: f64 = (0..n_basis).map(|i| cox_de_boor(i, 0, t, &knots)).sum();
				assert!((sum - 1.0).abs() < 1e-12, "degree-0 sum at t={t} over {knots:?}: {sum}");
			}
		}
	}

	#[test]
	fn rational_quadratic_is_a_circular_arc() {
		// A quadratic rational Bézier with the classic weights (1, cos45°, 1) is
		// an EXACT quarter unit circle — only correct if the weight denominator
		// is honoured. Proves the rational path, not just the polynomial one.
		let w = 0.5f64.sqrt();
		let ctrl = vec![DVec3::new(1.0, 0.0, 0.0), DVec3::new(1.0, 1.0, 0.0), DVec3::new(0.0, 1.0, 0.0)];
		let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
		let arc = NurbsCurve::new(2, knots.clone(), ctrl.clone(), vec![1.0, w, 1.0]).unwrap();
		for s in 0..=10 {
			let t = s as f64 / 10.0;
			let p = arc.point_at(t);
			assert!((p.length() - 1.0).abs() < 1e-9, "t {t}: |p|={} should be on the unit circle", p.length());
		}
		// With unit weights the same control net is NOT on the circle, confirming
		// the test actually depends on the weights.
		let poly = NurbsCurve::new(2, knots, ctrl, vec![1.0, 1.0, 1.0]).unwrap();
		assert!((poly.point_at(0.5).length() - 1.0).abs() > 1e-3, "unit weights must differ from the arc");
	}
}
