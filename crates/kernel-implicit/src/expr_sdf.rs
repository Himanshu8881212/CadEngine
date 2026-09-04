// Copyright (c) LMCAD. Licensed under the MIT License.

//! User-authored scalar fields as DATA — the runtime-composability leaf (BAR.md I6).
//!
//! A deployed AI cannot add a Rust `Sdf` impl, so this module carries custom
//! fields as a tiny math [`Expr`] tree over `(x, y, z)` that the kernel
//! evaluates itself (a plain enum tree walk in `f64` — no codegen, no `unsafe`,
//! no I/O: the expression language is total except for the IEEE poles of
//! `div` / `sqrt` / `mod`, which produce NaN/∞ values the caller must probe
//! for — see `kernel-api`'s structured probe errors).
//!
//! Two consumers:
//! - [`ExprSdf`] wraps an expression as an SDF **leaf** for the CSG
//!   [`Node`](crate::ops::Node) tree, normalizing by a REQUIRED user-declared
//!   Lipschitz bound (see the struct docs for the honest contract).
//! - [`scalar_field`] adapts an expression to the [`ScalarField`] consumed by
//!   the field-modulated operators ([`Node::offset_by`](crate::ops::Node::offset_by),
//!   [`Node::lerp`](crate::ops::Node::lerp)) — graded walls and graded lattices
//!   from pure data.

use std::sync::Arc;

use kernel_core::math::{Aabb, DVec3, Vec3};
use kernel_core::sdf::Sdf;

use crate::ops::ScalarField;

/// A scalar math expression over a world-space point `(x, y, z)`.
///
/// Evaluation is a recursive tree walk in `f64` ([`Expr::eval`]); every
/// operator is the plain IEEE function, so the only non-finite sources are
/// `Div` by 0, `Sqrt` of a negative, and `Mod` with modulus 0. `Clamp` is the
/// non-panicking `min(max(v, lo), hi)` (returns `hi` when `lo > hi`), `Mod` is
/// `rem_euclid` (result in `[0, |m|)` for finite inputs — the helical-unwrap
/// idiom needs the non-negative branch), and `Atan2` follows the standard
/// `atan2(y, x)` argument order.
#[derive(Debug)]
pub enum Expr {
	/// The query point's x coordinate (mm).
	X,
	/// The query point's y coordinate (mm).
	Y,
	/// The query point's z coordinate (mm).
	Z,
	/// A constant.
	Const(f64),
	Add(Box<Expr>, Box<Expr>),
	Sub(Box<Expr>, Box<Expr>),
	Mul(Box<Expr>, Box<Expr>),
	Div(Box<Expr>, Box<Expr>),
	Min(Box<Expr>, Box<Expr>),
	Max(Box<Expr>, Box<Expr>),
	Neg(Box<Expr>),
	Abs(Box<Expr>),
	Sqrt(Box<Expr>),
	Sin(Box<Expr>),
	Cos(Box<Expr>),
	/// `atan2(y, x)` — the angle of the point `(x, y)`, in radians.
	Atan2 {
		y: Box<Expr>,
		x: Box<Expr>,
	},
	/// Euclidean remainder `a.rem_euclid(m)` — non-negative for finite inputs.
	Mod(Box<Expr>, Box<Expr>),
	/// `min(max(value, lo), hi)` — never panics, even for `lo > hi`.
	Clamp {
		value: Box<Expr>,
		lo: Box<Expr>,
		hi: Box<Expr>,
	},
	/// `sqrt(a² + b²)`.
	Length2(Box<Expr>, Box<Expr>),
	/// `sqrt(a² + b² + c²)`.
	Length3(Box<Expr>, Box<Expr>, Box<Expr>),
}

impl Expr {
	/// Evaluate at the point `p` (a plain `f64` tree walk).
	pub fn eval(&self, p: DVec3) -> f64 {
		match self {
			Expr::X => p.x,
			Expr::Y => p.y,
			Expr::Z => p.z,
			Expr::Const(c) => *c,
			Expr::Add(a, b) => a.eval(p) + b.eval(p),
			Expr::Sub(a, b) => a.eval(p) - b.eval(p),
			Expr::Mul(a, b) => a.eval(p) * b.eval(p),
			Expr::Div(a, b) => a.eval(p) / b.eval(p),
			Expr::Min(a, b) => a.eval(p).min(b.eval(p)),
			Expr::Max(a, b) => a.eval(p).max(b.eval(p)),
			Expr::Neg(a) => -a.eval(p),
			Expr::Abs(a) => a.eval(p).abs(),
			Expr::Sqrt(a) => a.eval(p).sqrt(),
			Expr::Sin(a) => a.eval(p).sin(),
			Expr::Cos(a) => a.eval(p).cos(),
			Expr::Atan2 { y, x } => y.eval(p).atan2(x.eval(p)),
			Expr::Mod(a, m) => a.eval(p).rem_euclid(m.eval(p)),
			Expr::Clamp { value, lo, hi } => value.eval(p).max(lo.eval(p)).min(hi.eval(p)),
			Expr::Length2(a, b) => {
				let (a, b) = (a.eval(p), b.eval(p));
				(a * a + b * b).sqrt()
			}
			Expr::Length3(a, b, c) => {
				let (a, b, c) = (a.eval(p), b.eval(p), c.eval(p));
				(a * a + b * b + c * c).sqrt()
			}
		}
	}
}

/// Adapt an expression to the [`ScalarField`] driving
/// [`Node::offset_by`](crate::ops::Node::offset_by) /
/// [`Node::lerp`](crate::ops::Node::lerp). The closure evaluates in `f64` and
/// narrows to `f32` (the documented field precision of those operators); the
/// `Arc` keeps the same tree shareable with a probing caller.
pub fn scalar_field(expr: Arc<Expr>) -> ScalarField {
	Arc::new(move |p: Vec3| expr.eval(p.as_dvec3()) as f32)
}

/// A user [`Expr`] as an SDF leaf: `distance(p) = expr(p) / lipschitz_bound`.
///
/// # The Lipschitz contract (honest, load-bearing)
///
/// The narrow-band meshers' block pruning is only correct for fields that
/// never OVERSTATE the distance to their zero set — guaranteed by
/// `|∇d| ≤ 1`. An arbitrary expression cannot be auto-normalized, so the
/// caller MUST declare a bound `L ≥ sup|∇expr|`, and the kernel divides the
/// raw value by it:
///
/// - the **zero set — and therefore the meshed geometry — is unchanged**
///   (division by a positive constant moves no zero, and the meshers place
///   vertices by interpolating sign changes, where a constant factor cancels);
/// - the **slope is normalized**: if the declaration is truthful the field is
///   `≤ 1`-Lipschitz and narrow-band pruning is safe; over-declaring is safe
///   too (the field only flattens further);
/// - an **under-declared bound is dangerous**: sampled values overstate
///   distances and the narrow-band meshers may prune blocks that contain
///   surface (holes). If a bound is hard to derive, mesh dense
///   (`manifold_dual_contour` / `surface_nets` sample every cell and only
///   need continuity) or redistance first via [`crate::redistance`].
///
/// `new` asserts `lipschitz_bound` finite and `> 0` (the JSON binding rejects
/// bad bounds with a structured error before construction).
///
/// `bounds` is the user's declaration of where the surface can live: `None`
/// means UNBOUNDED (an infinite box, like [`crate::primitives::Plane`]) —
/// meshable only after intersecting with a bounded node or under an explicit
/// meshing domain.
pub struct ExprSdf {
	expr: Arc<Expr>,
	inv_lipschitz: f64,
	bounds: Aabb,
}

impl ExprSdf {
	/// Wrap `expr` with its declared Lipschitz bound and (optional) surface bounds.
	pub fn new(expr: Arc<Expr>, lipschitz_bound: f64, bounds: Option<Aabb>) -> Self {
		assert!(
			lipschitz_bound.is_finite() && lipschitz_bound > 0.0,
			"ExprSdf: lipschitz_bound must be finite and > 0, got {lipschitz_bound}"
		);
		let bounds = bounds.unwrap_or(Aabb::new(Vec3::splat(f32::NEG_INFINITY), Vec3::splat(f32::INFINITY)));
		Self { expr, inv_lipschitz: 1.0 / lipschitz_bound, bounds }
	}
}

impl Sdf for ExprSdf {
	fn distance(&self, p: Vec3) -> f32 {
		(self.expr.eval(p.as_dvec3()) * self.inv_lipschitz) as f32
	}

	fn distance64(&self, p: DVec3) -> f64 {
		self.expr.eval(p) * self.inv_lipschitz
	}

	fn bounds(&self) -> Aabb {
		self.bounds
	}
}

#[cfg(test)]
mod tests {
	use std::sync::Arc;

	use kernel_core::mesher::Resolution;
	use kernel_core::sdf::Sdf;

	use super::*;
	use crate::narrow_band::dual_contour_narrowband;
	use crate::ops::Node;
	use crate::primitives::{Cylinder, Sphere};

	fn b(e: Expr) -> Box<Expr> {
		Box::new(e)
	}

	#[test]
	fn eval_covers_every_operator() {
		// One snapshot per operator at a fixed probe point — covers the whole
		// grammar, the rem_euclid branch (negative numerator → non-negative
		// result), atan2 argument order, and the non-panicking clamp with an
		// inverted range.
		let p = DVec3::new(3.0, -4.0, 0.5);
		let cases: Vec<(Expr, f64)> = vec![
			(Expr::X, 3.0),
			(Expr::Y, -4.0),
			(Expr::Z, 0.5),
			(Expr::Const(2.5), 2.5),
			(Expr::Add(b(Expr::X), b(Expr::Y)), -1.0),
			(Expr::Sub(b(Expr::X), b(Expr::Y)), 7.0),
			(Expr::Mul(b(Expr::X), b(Expr::Z)), 1.5),
			(Expr::Div(b(Expr::X), b(Expr::Const(2.0))), 1.5),
			(Expr::Min(b(Expr::X), b(Expr::Y)), -4.0),
			(Expr::Max(b(Expr::X), b(Expr::Y)), 3.0),
			(Expr::Neg(b(Expr::X)), -3.0),
			(Expr::Abs(b(Expr::Y)), 4.0),
			(Expr::Sqrt(b(Expr::Const(9.0))), 3.0),
			(Expr::Sin(b(Expr::Const(std::f64::consts::FRAC_PI_2))), 1.0),
			(Expr::Cos(b(Expr::Const(0.0))), 1.0),
			(Expr::Atan2 { y: b(Expr::Const(1.0)), x: b(Expr::Const(0.0)) }, std::f64::consts::FRAC_PI_2),
			(Expr::Mod(b(Expr::Const(-1.0)), b(Expr::Const(3.0))), 2.0),
			(Expr::Clamp { value: b(Expr::X), lo: b(Expr::Const(0.0)), hi: b(Expr::Const(1.0)) }, 1.0),
			(Expr::Clamp { value: b(Expr::X), lo: b(Expr::Const(5.0)), hi: b(Expr::Const(1.0)) }, 1.0), // inverted range must not panic
			(Expr::Length2(b(Expr::X), b(Expr::Y)), 5.0),
			(Expr::Length3(b(Expr::Const(2.0)), b(Expr::Const(3.0)), b(Expr::Const(6.0))), 7.0),
		];
		let got: Vec<f64> = cases.iter().map(|(e, _)| e.eval(p)).collect();
		let want: Vec<f64> = cases.iter().map(|(_, w)| *w).collect();
		assert_eq!(got, want, "expression operators (in declaration order) disagree with their closed forms");
	}

	#[test]
	fn expr_sphere_matches_analytic_primitive_and_meshes() {
		// length3(x, y, z) − 8 IS the exact unit-slope sphere SDF: with L = 1 the
		// ExprSdf must agree pointwise with the analytic primitive, and a narrow-
		// band extraction must be watertight at the analytic volume.
		let expr = Arc::new(Expr::Sub(b(Expr::Length3(b(Expr::X), b(Expr::Y), b(Expr::Z))), b(Expr::Const(8.0))));
		let leaf = ExprSdf::new(expr, 1.0, Some(Aabb::from_center_half_extent(Vec3::ZERO, Vec3::splat(8.0))));
		let exact = Sphere::new(Vec3::ZERO, 8.0);
		for p in [Vec3::ZERO, Vec3::new(3.0, -2.0, 5.0), Vec3::new(10.0, 0.0, 0.0), Vec3::new(-7.0, 7.0, 7.0)] {
			let (got, want) = (leaf.distance(p), exact.distance(p));
			assert!((got - want).abs() < 1e-5, "expr sphere vs analytic at {p:?}: {got} vs {want}");
		}
		let node = Node::primitive(ExprSdf::new(
			Arc::new(Expr::Sub(b(Expr::Length3(b(Expr::X), b(Expr::Y), b(Expr::Z))), b(Expr::Const(8.0)))),
			1.0,
			Some(Aabb::from_center_half_extent(Vec3::ZERO, Vec3::splat(8.0))),
		));
		let mesh = dual_contour_narrowband(&node, node.bounds().pad(1.0), Resolution::VoxelSize(0.25));
		let vol = mesh.signed_volume();
		let want = 4.0 / 3.0 * std::f64::consts::PI * 512.0;
		assert!(
			mesh.is_watertight() && (vol - want).abs() / want < 0.01,
			"expr sphere mesh: watertight={} vol={vol:.1} want {want:.1}",
			mesh.is_watertight()
		);
	}

	#[test]
	fn lipschitz_division_preserves_zero_set_and_normalizes_slope() {
		// The same sphere written with a steep field (3× the metric slope) and
		// the matching declared bound: values must be exactly the raw value / 3
		// (slope normalized), so the zero set — probed ON the surface — is
		// unchanged while off-surface values shrink threefold.
		let raw = |p: Vec3| 3.0 * (p.as_dvec3().length() - 8.0);
		let steep = Arc::new(Expr::Mul(
			b(Expr::Const(3.0)),
			b(Expr::Sub(b(Expr::Length3(b(Expr::X), b(Expr::Y), b(Expr::Z))), b(Expr::Const(8.0)))),
		));
		let leaf = ExprSdf::new(steep, 3.0, None);
		for p in [Vec3::new(8.0, 0.0, 0.0), Vec3::new(0.0, -8.0, 0.0), Vec3::new(12.0, 0.0, 0.0), Vec3::new(2.0, 2.0, 2.0)] {
			let (got, want) = (leaf.distance64(p.as_dvec3()), raw(p) / 3.0);
			assert!((got - want).abs() < 1e-9, "normalized field at {p:?}: {got} vs {want}");
		}
		// Unbounded leaf (bounds: None) reports the infinite box — composing it
		// under an intersection must yield the finite operand's bound, exactly
		// like the Plane primitive.
		let leaf = ExprSdf::new(Arc::new(Expr::Z), 1.0, None);
		assert!(!leaf.bounds().min.is_finite(), "bounds-less ExprSdf must report an unbounded box");
		let clipped =
			Node::primitive(Sphere::new(Vec3::ZERO, 5.0)).intersection(Node::primitive(ExprSdf::new(Arc::new(Expr::Z), 1.0, None)));
		let bb = clipped.bounds();
		assert!(bb.is_valid() && bb.min.is_finite() && bb.max.is_finite(), "intersection with a finite operand must bound: {bb:?}");
	}

	#[test]
	fn expr_field_drives_offset_by_like_a_closure() {
		// The graded-wall workflow from data: an expression field handed to
		// offset_by must reproduce the hand-written closure field pointwise
		// (same z-ramp, same clamp), per the offset_by contract.
		let cyl = || Node::primitive(Cylinder::new(Vec3::ZERO, Vec3::new(0.0, 0.0, 40.0), 10.0));
		let by_closure = cyl().offset_by(Arc::new(|p: Vec3| -(2.0 + 0.05 * p.z)), 6.0);
		let ramp = Arc::new(Expr::Neg(b(Expr::Add(b(Expr::Const(2.0)), b(Expr::Mul(b(Expr::Const(0.05)), b(Expr::Z)))))));
		let by_expr = cyl().offset_by(scalar_field(ramp), 6.0);
		for p in [Vec3::new(7.0, 0.0, 5.0), Vec3::new(0.0, 6.5, 35.0), Vec3::new(9.0, 1.0, 20.0), Vec3::new(11.0, 0.0, -2.0)] {
			let (got, want) = (by_expr.distance(p), by_closure.distance(p));
			assert!((got - want).abs() < 1e-6, "expr field vs closure field at {p:?}: {got} vs {want}");
		}
	}

	#[test]
	#[should_panic(expected = "lipschitz_bound must be finite and > 0")]
	fn expr_sdf_rejects_nonpositive_lipschitz_bound() {
		let _ = ExprSdf::new(Arc::new(Expr::X), 0.0, None);
	}
}
