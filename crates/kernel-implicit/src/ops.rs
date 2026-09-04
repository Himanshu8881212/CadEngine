// Copyright (c) LMCAD. Licensed under the MIT License.

//! The CSG tree.
//!
//! A [`Node`] composes any [`Sdf`] leaves into a solid model. Booleans are just
//! `min` / `max` on signed distances — trivially robust, never failing, with no
//! surface–surface intersection engine. This is the whole boolean engine of the
//! kernel (per the spec's core decision).
//!
//! `Node` itself implements [`Sdf`], so a whole CSG tree can be handed to any
//! mesher (e.g. `kernel_core::surface_nets`). Composite gradients use the
//! default central difference, which behaves well on `min`/`max` fields.
//!
//! # Field-quality contract (which nodes are an EXACT distance vs. a BOUND)
//!
//! Not every node evaluates to the *true* signed Euclidean distance. Some
//! combinators only produce a **1-Lipschitz bound** — a field whose magnitude
//! never overstates the distance to the zero set, and which agrees with the
//! exact distance away from seams/blends but understates it near them. Meshers
//! only need a continuous field (or a ≤ 1-Lipschitz bound for narrow-band
//! pruning), so a bound meshes fine. But the **distance-assuming ops**
//! ([`Node::offset`], [`Node::shell`], [`Node::offset_by`]) compute `d ∓ t`
//! and are only correct when `d` is the exact distance — offsetting a bound
//! gives subtly wrong walls near the seams. [`Node::field_quality`] makes this
//! checkable and it propagates through the tree:
//!
//! | node | quality | why |
//! |---|---|---|
//! | [`Node::primitive`] leaf | **ExactSdf** | the [`Sdf`] contract is an exact signed distance (analytic sphere/box/cylinder/cone/plane/torus/capsule, `ExprSdf` with a truthful bound) |
//! | [`Node::primitive_bound`] leaf | **DistanceBound** | leaves that are only fields/bounds: [`crate::Gyroid`], [`crate::Tpms`], [`crate::MeshSdf`], a `min`-union lattice inside overlaps, the fillet/chamfer [`crate::features`] wrappers |
//! | `Offset`, `Shell`, `Transform` | **propagates the child** | subtracting a constant / `abs` / a similarity map preserves an exact distance (away from the universal offset medial-axis caveat, which afflicts exact fields too) |
//! | `Union`, `Intersection`, `Difference` | **DistanceBound** | `min`/`max` of exact SDFs is exact in the far field but only a bound near a seam — the nearest surface point of one operand can be occluded by the other (rigorous: NOT an exact SDF even for exact children) |
//! | `SmoothUnion/Intersection/Difference` | **DistanceBound** | polynomial smooth-min is a bound throughout the blend band by construction |
//! | `LinearArray`, `PolarArray`, `Mirror` | **DistanceBound** | a `min`-union of transformed copies — a bound near overlaps (and disjointness cannot be proven at classify time) |
//! | `OffsetBy` | **DistanceBound** | a position-varying offset is only `(1+g)`-Lipschitz — a true SDF only for a constant field |
//! | `LerpBlend` | **DistanceBound** | a convex blend of two distance fields is not itself a distance field |
//!
//! The classifier is deliberately **conservative — when in doubt, `DistanceBound`**
//! (it never over-claims exactness). Because the [`Sdf`] trait (in `kernel-core`)
//! carries no quality method, a `Node::primitive` leaf defaults to `ExactSdf`
//! (the trait's documented exact-distance contract); the known field/bound leaves
//! must be wrapped with [`Node::primitive_bound`] so the propagation stays honest.

use std::sync::Arc;

use kernel_core::math::{Aabb, Affine3A, DQuat, DVec3, Quat, Vec3};
use kernel_core::sdf::Sdf;

/// A user scalar field over world space, driving the field-modulated operators
/// ([`Node::offset_by`], [`Node::lerp`]). Stored in an [`Arc`] so a node tree
/// holding one stays `Send + Sync` and cheap to share across meshing threads.
/// Fields are evaluated in `f32` even on the `f64` distance path (documented on
/// the operators).
pub type ScalarField = Arc<dyn Fn(Vec3) -> f32 + Send + Sync>;

/// Whether a [`Node`]'s field is the true signed Euclidean distance or only a
/// 1-Lipschitz **bound** on it. See the module docs for the per-node table and
/// the honest reasoning; the short version:
///
/// - `ExactSdf` — the value equals the signed distance to the zero set
///   everywhere. Safe to feed to the distance-assuming ops ([`Node::offset`],
///   [`Node::shell`], [`Node::offset_by`]).
/// - `DistanceBound` — the value is `≤` the true distance in magnitude and only
///   agrees with it away from seams/blends. Still meshable, but offsetting it is
///   **approximate** near those regions.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FieldQuality {
	/// A true signed Euclidean distance field.
	ExactSdf,
	/// A 1-Lipschitz bound — not the exact distance near seams/blends.
	DistanceBound,
}

impl FieldQuality {
	/// `true` for [`FieldQuality::ExactSdf`].
	#[inline]
	pub fn is_exact(self) -> bool {
		matches!(self, FieldQuality::ExactSdf)
	}
}

/// The result of a **checked** distance-assuming op ([`Node::offset_checked`],
/// [`Node::shell_checked`], [`Node::offset_by_checked`]): the built node plus an
/// honest verdict on whether the operation was exact.
///
/// `offset`/`shell`/`offset_by` compute `d ∓ t`, which is only the correct
/// offset when the input `d` is an exact SDF. The checked variants query the
/// input's [`FieldQuality`] up front and surface it here so a caller offsetting
/// a smooth-blended / fillet / TPMS field is TOLD the offset is approximate —
/// the numeric result is never silently changed.
pub struct OffsetOutcome {
	/// The constructed node (identical to what the unchecked op would return).
	pub node: Node,
	/// Quality of the INPUT field the op assumed to be an exact distance.
	pub input_quality: FieldQuality,
	/// Which op produced this: `"offset"`, `"shell"`, or `"offset_by"`.
	pub op: &'static str,
}

impl OffsetOutcome {
	/// `true` when the input was only a [`FieldQuality::DistanceBound`], so the
	/// offset/shell is **approximate** (walls may be subtly wrong near seams,
	/// smooth blends, or fillet/chamfer edges).
	#[inline]
	pub fn is_approximate(&self) -> bool {
		!self.input_quality.is_exact()
	}

	/// A loud, human-readable warning when [`Self::is_approximate`], else `None`.
	pub fn warning(&self) -> Option<String> {
		self.is_approximate().then(|| {
			format!(
				"{}: the input field is a DistanceBound, not an exact SDF — the result is APPROXIMATE. \
				 Walls/offsets can be subtly wrong near seams, smooth blends, or fillet/chamfer edges. \
				 Redistance the input (crate::redistance) or offset an exact primitive instead.",
				self.op
			)
		})
	}
}

/// Smooth minimum (polynomial). Falls back to a hard `min` when `k <= 0`.
#[inline]
fn smin(a: f32, b: f32, k: f32) -> f32 {
	if k <= 0.0 {
		return a.min(b);
	}
	let h = (0.5 + 0.5 * (b - a) / k).clamp(0.0, 1.0);
	(b + h * (a - b)) - k * h * (1.0 - h)
}

/// Smooth maximum, the De Morgan dual of [`smin`].
#[inline]
fn smax(a: f32, b: f32, k: f32) -> f32 {
	-smin(-a, -b, k)
}

/// Double-precision polynomial smooth-min (mirrors [`smin`]).
fn smin64(a: f64, b: f64, k: f64) -> f64 {
	if k <= 0.0 {
		return a.min(b);
	}
	let h = (0.5 + 0.5 * (b - a) / k).clamp(0.0, 1.0);
	(b + h * (a - b)) - k * h * (1.0 - h)
}

fn smax64(a: f64, b: f64, k: f64) -> f64 {
	-smin64(-a, -b, k)
}

/// A rigid + uniform-scale transform, precomputed for cheap sampling.
///
/// `inv` maps world → local (used when sampling distance); `fwd` maps local →
/// world (used to transform bounds); `scale` is the uniform scale factor so
/// that local distances become correct world distances.
#[derive(Clone, Copy, Debug)]
pub struct Xform {
	inv: Affine3A,
	fwd: Affine3A,
	scale: f32,
}

impl Xform {
	pub fn new(fwd: Affine3A) -> Self {
		// Assumes uniform scale; take it from a basis column length.
		let scale = fwd.matrix3.x_axis.length();
		Self { inv: fwd.inverse(), fwd, scale }
	}
}

/// A node in the CSG tree.
pub enum Node {
	/// A leaf primitive (any boxed [`Sdf`]) with its declared [`FieldQuality`].
	/// Built via [`Node::primitive`] (exact) or [`Node::primitive_bound`] (bound).
	Prim(Box<dyn Sdf>, FieldQuality),
	Union(Box<Node>, Box<Node>),
	Intersection(Box<Node>, Box<Node>),
	Difference(Box<Node>, Box<Node>),
	SmoothUnion(Box<Node>, Box<Node>, f32),
	SmoothIntersection(Box<Node>, Box<Node>, f32),
	SmoothDifference(Box<Node>, Box<Node>, f32),
	Offset(Box<Node>, f32),
	Shell(Box<Node>, f32),
	Transform(Box<Node>, Box<Xform>),
	/// `count` copies of the child, each offset by an added `step` (linear pattern).
	LinearArray(Box<Node>, Vec3, u32),
	/// `count` copies of the child, each rotated an extra `step_angle` (rad) about
	/// `axis` through `center` (polar pattern). The `axis` is stored unit-length.
	PolarArray(Box<Node>, Vec3, Vec3, u32, f32),
	/// The child unioned with its reflection across the plane (`point`, unit `normal`).
	Mirror(Box<Node>, Vec3, Vec3),
	/// The child offset by a position-varying amount, clamped to ± the stored
	/// bound: `d(p) − clamp(field(p), ±max_abs)` (see [`Node::offset_by`]).
	OffsetBy(Box<Node>, ScalarField, f32),
	/// Pointwise blend of the two children driven by the field, clamped to
	/// 0..1: `(1−w)·a + w·b` (see [`Node::lerp`]).
	LerpBlend(Box<Node>, Box<Node>, ScalarField),
}

#[inline]
fn boxed(n: Node) -> Box<Node> {
	Box::new(n)
}

impl Node {
	/// Wrap any [`Sdf`] as a leaf node, tagged [`FieldQuality::ExactSdf`].
	///
	/// This is the default because the [`Sdf`] trait's documented contract is
	/// an exact signed Euclidean distance (the analytic primitives, and an
	/// [`crate::ExprSdf`] with a truthful Lipschitz bound, honour it). Leaves
	/// that are only a field/bound — [`crate::Gyroid`], [`crate::Tpms`],
	/// [`crate::MeshSdf`], a strut lattice (a `min`-union that understates depth
	/// inside overlaps) — must instead be wrapped with [`Node::primitive_bound`]
	/// so a downstream [`Node::offset`]/[`Node::shell`] is honestly flagged as
	/// approximate. (The trait carries no quality method — it lives in
	/// `kernel-core` — so the leaf's quality cannot be auto-detected here.)
	pub fn primitive(sdf: impl Sdf + 'static) -> Node {
		Node::Prim(Box::new(sdf), FieldQuality::ExactSdf)
	}

	/// Wrap an [`Sdf`] leaf that is only a distance **bound**, not an exact SDF
	/// — tagged [`FieldQuality::DistanceBound`]. Use for [`crate::Gyroid`] /
	/// [`crate::Tpms`] fields, the winding-number [`crate::MeshSdf`] bridge, and
	/// any leaf whose `distance` can understate the true distance to its zero
	/// set. The geometry (zero set) is unaffected — only the honesty of the
	/// field-quality propagation, so [`Node::offset_checked`] and friends surface
	/// the approximation.
	pub fn primitive_bound(sdf: impl Sdf + 'static) -> Node {
		Node::Prim(Box::new(sdf), FieldQuality::DistanceBound)
	}

	/// Boolean union (`min`).
	pub fn union(self, other: Node) -> Node {
		Node::Union(boxed(self), boxed(other))
	}

	/// Boolean intersection (`max`).
	pub fn intersection(self, other: Node) -> Node {
		Node::Intersection(boxed(self), boxed(other))
	}

	/// Boolean difference, `self - other` (`max(a, -b)`).
	pub fn difference(self, other: Node) -> Node {
		Node::Difference(boxed(self), boxed(other))
	}

	/// Smooth (filleted) union with blend radius `k`.
	pub fn smooth_union(self, other: Node, k: f32) -> Node {
		Node::SmoothUnion(boxed(self), boxed(other), k)
	}

	/// Smooth intersection with blend radius `k`.
	pub fn smooth_intersection(self, other: Node, k: f32) -> Node {
		Node::SmoothIntersection(boxed(self), boxed(other), k)
	}

	/// Smooth difference with blend radius `k`.
	pub fn smooth_difference(self, other: Node, k: f32) -> Node {
		Node::SmoothDifference(boxed(self), boxed(other), k)
	}

	/// Offset the surface outward (`t > 0` inflates, `t < 0` deflates).
	///
	/// **Distance-assuming:** computes `d - t`, correct only when `d` is an
	/// exact SDF. Offsetting a [`FieldQuality::DistanceBound`] input (a smooth
	/// blend, a boolean seam, a TPMS field) is APPROXIMATE near those regions —
	/// use [`Node::offset_checked`] to be TOLD, or check [`Node::field_quality`]
	/// first. (Even an exact input is only exact away from the offset's medial
	/// axis, a universal geometric caveat of offsetting.)
	pub fn offset(self, t: f32) -> Node {
		Node::Offset(boxed(self), t)
	}

	/// [`Node::offset`] that surfaces the input's [`FieldQuality`] — the checked
	/// path. Returns an [`OffsetOutcome`] whose [`OffsetOutcome::is_approximate`]
	/// /[`OffsetOutcome::warning`] tell the caller whether the offset is exact.
	pub fn offset_checked(self, t: f32) -> OffsetOutcome {
		let input_quality = self.field_quality();
		OffsetOutcome { node: self.offset(t), input_quality, op: "offset" }
	}

	/// Hollow shell of total wall thickness `2 * t`.
	///
	/// **Distance-assuming:** computes `|d| - t`, correct only when `d` is an
	/// exact SDF (see [`Node::offset`]). Shelling a [`FieldQuality::DistanceBound`]
	/// input is APPROXIMATE — use [`Node::shell_checked`] to be told.
	pub fn shell(self, t: f32) -> Node {
		Node::Shell(boxed(self), t)
	}

	/// [`Node::shell`] that surfaces the input's [`FieldQuality`] — see
	/// [`Node::offset_checked`].
	pub fn shell_checked(self, t: f32) -> OffsetOutcome {
		let input_quality = self.field_quality();
		OffsetOutcome { node: self.shell(t), input_quality, op: "shell" }
	}

	/// Apply an arbitrary rigid + uniform-scale transform.
	pub fn transform(self, fwd: Affine3A) -> Node {
		Node::Transform(boxed(self), Box::new(Xform::new(fwd)))
	}

	/// Translate by `v`.
	pub fn translate(self, v: Vec3) -> Node {
		self.transform(Affine3A::from_translation(v))
	}

	/// A linear pattern: `count` copies of `self`, each offset by an added `step`
	/// (so copy *i* sits at `i·step`). Evaluated by SDF domain repetition, so the
	/// child is stored once regardless of `count`.
	pub fn linear_pattern(self, step: Vec3, count: usize) -> Node {
		Node::LinearArray(boxed(self), step, count.max(1) as u32)
	}

	/// A polar pattern: `count` copies of `self`, each rotated an additional
	/// `step_angle` radians about `axis` through `center`. For a full ring of `n`,
	/// pass `step_angle = 2π / n` (e.g. a bolt circle).
	pub fn circular_pattern(self, center: Vec3, axis: Vec3, step_angle: f32, count: usize) -> Node {
		// Keep a genuine unit axis so distance, distance64 and bounds all rotate
		// consistently; a degenerate axis falls back to Z rather than collapsing the
		// rotation (which would scale the field but not the bounds).
		let axis = axis.try_normalize().unwrap_or(Vec3::Z);
		Node::PolarArray(boxed(self), center, axis, count.max(1) as u32, step_angle)
	}

	/// Union `self` with its mirror image across the plane through `point` with
	/// `normal` — build a symmetric part from one half.
	pub fn mirror(self, point: Vec3, normal: Vec3) -> Node {
		Node::Mirror(boxed(self), point, normal.normalize_or_zero())
	}

	/// Offset the surface by a **position-varying** amount: at `p` the surface
	/// moves outward by `field(p)` world units (negative values carve inward) —
	/// the graded-wall-thickness / graded-lattice-inflation operator. The field
	/// value is clamped to `±max_abs` (asserted finite and ≥ 0), which also pads
	/// the reported bounds, so the bound stays correct whatever the closure
	/// returns.
	///
	/// **Lipschitz contract (honest):** the result is `d(p) − f(p)`. With a
	/// 1-Lipschitz child and `|∇f| ≤ g` the result is only `(1 + g)`-Lipschitz —
	/// a true signed distance ONLY for constant `f`. The narrow-band meshers'
	/// block pruning assumes ≤ 1-Lipschitz fields, so keep the field gradient
	/// small (`g ≪ 1`, e.g. a thickness ramp of a few % per mm), mesh densely
	/// (`surface_nets` / `manifold_dual_contour` sample every cell and only need
	/// continuity), or redistance first via [`crate::redistance`]. An arbitrary
	/// closure cannot be auto-normalized the way [`crate::primitives::Gyroid`]
	/// normalizes its fixed sine field — the contract is on the caller.
	pub fn offset_by(self, field: ScalarField, max_abs: f32) -> Node {
		assert!(max_abs.is_finite() && max_abs >= 0.0, "offset_by: max_abs must be finite and >= 0, got {max_abs}");
		Node::OffsetBy(boxed(self), field, max_abs)
	}

	/// [`Node::offset_by`] that surfaces the input's [`FieldQuality`] — see
	/// [`Node::offset_checked`]. Note the RESULT of `offset_by` is always a
	/// [`FieldQuality::DistanceBound`] (a position-varying offset is not a true
	/// SDF); this checked variant reports on the **input** the op assumes to be
	/// exact, so an already-bound input is flagged as doubly approximate.
	pub fn offset_by_checked(self, field: ScalarField, max_abs: f32) -> OffsetOutcome {
		let input_quality = self.field_quality();
		OffsetOutcome { node: self.offset_by(field, max_abs), input_quality, op: "offset_by" }
	}

	/// Blend pointwise between `self` (weight 0) and `other` (weight 1) with a
	/// position-varying weight `w = clamp(field(p), 0, 1)` — the nTop/PicoGK
	/// style implicit lerp for graded transitions (e.g. a solid wall morphing
	/// into a lattice along a ramp). The blend solid is always contained in the
	/// union of the operands (a convex combination of two positive distances is
	/// positive), so the reported union bound is exact.
	///
	/// **Lipschitz contract (honest):** the gradient is bounded by
	/// `max(L_a, L_b) + |a − b|·|∇w|`, so the result is near-1-Lipschitz only
	/// where the operand surfaces are close or the field varies slowly. Same
	/// guidance as [`Node::offset_by`]: dense meshing is always safe; for
	/// narrow-band extraction keep `|a − b|·|∇w|` small or redistance.
	pub fn lerp(self, other: Node, field: ScalarField) -> Node {
		Node::LerpBlend(boxed(self), boxed(other), field)
	}

	/// Rotate by quaternion `q` about the origin.
	pub fn rotate(self, q: Quat) -> Node {
		self.transform(Affine3A::from_quat(q))
	}

	/// Uniformly scale by `s` about the origin.
	pub fn scale(self, s: f32) -> Node {
		self.transform(Affine3A::from_scale(Vec3::splat(s)))
	}

	/// Classify this node's field as an exact SDF or only a distance **bound**
	/// (see the module docs for the per-node table and the honest reasoning).
	///
	/// The rule: a combinator is [`FieldQuality::ExactSdf`] only if its rule
	/// preserves the exact-distance property AND every child it draws on is
	/// exact — otherwise [`FieldQuality::DistanceBound`]. Only `Offset`, `Shell`
	/// and `Transform` preserve distance (a constant shift, an `abs`, a
	/// similarity map); every boolean/blend/pattern is at best a 1-Lipschitz
	/// bound, so it forces `DistanceBound` regardless of its children. Leaves
	/// carry the tag set at construction ([`Node::primitive`] /
	/// [`Node::primitive_bound`]). Conservative by design: never over-claims.
	pub fn field_quality(&self) -> FieldQuality {
		use FieldQuality::DistanceBound as Bound;
		match self {
			Node::Prim(_, q) => *q,
			// Distance-preserving: propagate the child (a constant shift / abs /
			// similarity of an exact distance is still an exact distance).
			Node::Offset(a, _) | Node::Shell(a, _) | Node::Transform(a, _) => a.field_quality(),
			// min/max booleans: exact in the far field, but only a bound near a
			// seam (an operand's nearest surface point can be occluded by the
			// other) — NOT an exact SDF even for exact children.
			Node::Union(..) | Node::Intersection(..) | Node::Difference(..) => Bound,
			// Polynomial smooth-min is a bound throughout the blend band.
			Node::SmoothUnion(..) | Node::SmoothIntersection(..) | Node::SmoothDifference(..) => Bound,
			// A min-union of transformed copies — a bound near overlaps.
			Node::LinearArray(..) | Node::PolarArray(..) | Node::Mirror(..) => Bound,
			// Position-varying offset is (1+g)-Lipschitz; a convex blend of two
			// distance fields is not a distance field.
			Node::OffsetBy(..) | Node::LerpBlend(..) => Bound,
		}
	}

	/// True if the tree contains a distance-ASSUMING op — `offset`, `shell`, or
	/// `offset_by` — applied to a child whose field is only a
	/// [`FieldQuality::DistanceBound`]: an UNSOUND offset whose wall/offset is
	/// silently approximate. This is stricter and more useful than
	/// [`Node::field_quality`] (which is `Bound` whenever ANY bound op appears —
	/// e.g. a perfectly legitimate `smooth_union`): it flags specifically the
	/// case the checked constructors warn about, and lets the meshing boundary
	/// surface it so an approximate offset can never pass unnoticed.
	pub fn has_approximate_offset(&self) -> bool {
		match self {
			Node::Prim(..) => false,
			// A distance-assuming op on a non-exact child is unsound here.
			Node::Offset(a, _) | Node::Shell(a, _) | Node::OffsetBy(a, _, _) => !a.field_quality().is_exact() || a.has_approximate_offset(),
			Node::Transform(a, _) | Node::LinearArray(a, _, _) | Node::PolarArray(a, _, _, _, _) | Node::Mirror(a, _, _) => {
				a.has_approximate_offset()
			}
			Node::Union(a, b)
			| Node::Intersection(a, b)
			| Node::Difference(a, b)
			| Node::SmoothUnion(a, b, _)
			| Node::SmoothIntersection(a, b, _)
			| Node::SmoothDifference(a, b, _)
			| Node::LerpBlend(a, b, _) => a.has_approximate_offset() || b.has_approximate_offset(),
		}
	}
}

/// AABB of `b` after transforming its 8 corners by `m`.
fn transform_aabb(b: Aabb, m: Affine3A) -> Aabb {
	let mut out = Aabb::empty();
	for c in b.corners() {
		out = out.expand_point(m.transform_point3(c));
	}
	out
}

impl Sdf for Node {
	fn distance(&self, p: Vec3) -> f32 {
		match self {
			Node::Prim(s, _) => s.distance(p),
			Node::Union(a, b) => a.distance(p).min(b.distance(p)),
			Node::Intersection(a, b) => a.distance(p).max(b.distance(p)),
			Node::Difference(a, b) => a.distance(p).max(-b.distance(p)),
			Node::SmoothUnion(a, b, k) => smin(a.distance(p), b.distance(p), *k),
			Node::SmoothIntersection(a, b, k) => smax(a.distance(p), b.distance(p), *k),
			Node::SmoothDifference(a, b, k) => smax(a.distance(p), -b.distance(p), *k),
			Node::Offset(a, t) => a.distance(p) - t,
			Node::Shell(a, t) => a.distance(p).abs() - t,
			Node::Transform(a, x) => x.scale * a.distance(x.inv.transform_point3(p)),
			Node::LinearArray(a, step, count) => {
				let mut d = f32::INFINITY;
				for i in 0..*count {
					d = d.min(a.distance(p - *step * i as f32));
				}
				d
			}
			Node::PolarArray(a, center, axis, count, ang) => {
				let mut d = f32::INFINITY;
				for i in 0..*count {
					let q = Quat::from_axis_angle(*axis, -*ang * i as f32);
					d = d.min(a.distance(*center + q * (p - *center)));
				}
				d
			}
			Node::Mirror(a, point, n) => {
				let refl = p - *n * (2.0 * (p - *point).dot(*n));
				a.distance(p).min(a.distance(refl))
			}
			Node::OffsetBy(a, f, m) => a.distance(p) - f(p).clamp(-*m, *m),
			Node::LerpBlend(a, b, f) => {
				let w = f(p).clamp(0.0, 1.0);
				a.distance(p) * (1.0 - w) + b.distance(p) * w
			}
		}
	}

	fn distance64(&self, p: DVec3) -> f64 {
		match self {
			Node::Prim(s, _) => s.distance64(p),
			Node::Union(a, b) => a.distance64(p).min(b.distance64(p)),
			Node::Intersection(a, b) => a.distance64(p).max(b.distance64(p)),
			Node::Difference(a, b) => a.distance64(p).max(-b.distance64(p)),
			Node::SmoothUnion(a, b, k) => smin64(a.distance64(p), b.distance64(p), *k as f64),
			Node::SmoothIntersection(a, b, k) => smax64(a.distance64(p), b.distance64(p), *k as f64),
			Node::SmoothDifference(a, b, k) => smax64(a.distance64(p), -b.distance64(p), *k as f64),
			Node::Offset(a, t) => a.distance64(p) - *t as f64,
			Node::Shell(a, t) => a.distance64(p).abs() - *t as f64,
			// The transform itself is applied in f32 (the stored affine), but the
			// child primitive evaluates the transformed point in f64.
			Node::Transform(a, x) => x.scale as f64 * a.distance64(x.inv.transform_point3(p.as_vec3()).as_dvec3()),
			Node::LinearArray(a, step, count) => {
				let (mut d, s) = (f64::INFINITY, step.as_dvec3());
				for i in 0..*count {
					d = d.min(a.distance64(p - s * i as f64));
				}
				d
			}
			Node::PolarArray(a, center, axis, count, ang) => {
				let (mut d, c, ax) = (f64::INFINITY, center.as_dvec3(), axis.as_dvec3());
				for i in 0..*count {
					let q = DQuat::from_axis_angle(ax, -*ang as f64 * i as f64);
					d = d.min(a.distance64(c + q * (p - c)));
				}
				d
			}
			Node::Mirror(a, point, n) => {
				let (pt, nn) = (point.as_dvec3(), n.as_dvec3());
				let refl = p - nn * (2.0 * (p - pt).dot(nn));
				a.distance64(p).min(a.distance64(refl))
			}
			// User scalar fields are f32-valued; the children still evaluate in f64.
			Node::OffsetBy(a, f, m) => a.distance64(p) - f(p.as_vec3()).clamp(-*m, *m) as f64,
			Node::LerpBlend(a, b, f) => {
				let w = f(p.as_vec3()).clamp(0.0, 1.0) as f64;
				a.distance64(p) * (1.0 - w) + b.distance64(p) * w
			}
		}
	}

	fn bounds(&self) -> Aabb {
		match self {
			Node::Prim(s, _) => s.bounds(),
			Node::Union(a, b) => a.bounds().union(b.bounds()),
			Node::Intersection(a, b) => a.bounds().intersection(b.bounds()),
			Node::Difference(a, _) => a.bounds(),
			Node::SmoothUnion(a, b, k) => a.bounds().union(b.bounds()).pad(*k),
			Node::SmoothIntersection(a, b, k) => a.bounds().intersection(b.bounds()).pad(*k),
			Node::SmoothDifference(a, _, k) => a.bounds().pad(*k),
			Node::Offset(a, t) => a.bounds().pad(t.max(0.0)),
			Node::Shell(a, t) => a.bounds().pad(*t),
			Node::Transform(a, x) => transform_aabb(a.bounds(), x.fwd),
			Node::LinearArray(a, step, count) => {
				let bb = a.bounds();
				let mut out = Aabb::empty();
				for i in 0..*count {
					let t = *step * i as f32;
					out = out.union(Aabb::new(bb.min + t, bb.max + t));
				}
				out
			}
			Node::PolarArray(a, center, axis, count, ang) => {
				let bb = a.bounds();
				let mut out = Aabb::empty();
				for i in 0..*count {
					let rot = Affine3A::from_axis_angle(*axis, *ang * i as f32);
					let m = Affine3A::from_translation(*center) * rot * Affine3A::from_translation(-*center);
					out = out.union(transform_aabb(bb, m));
				}
				out
			}
			Node::Mirror(a, point, n) => {
				let bb = a.bounds();
				let mut out = bb;
				for c in bb.corners() {
					out = out.expand_point(c - *n * (2.0 * (c - *point).dot(*n)));
				}
				out
			}
			// The clamp guarantees the surface moves at most max_abs outward.
			Node::OffsetBy(a, _, m) => a.bounds().pad(*m),
			// The blend solid is contained in A ∪ B (see `Node::lerp`).
			Node::LerpBlend(a, b, _) => a.bounds().union(b.bounds()),
		}
	}
}
