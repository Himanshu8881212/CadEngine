// Copyright (c) LMCAD. Licensed under the MIT License.

//! The GPU-lowerable field tree — a typed mirror of the CSG [`Node`] tree.
//!
//! # Why a mirror tree instead of lowering `Node` directly (honest design note)
//!
//! `Node::Prim` boxes a type-erased `dyn Sdf` and the `Sdf` trait deliberately
//! has no `Any` supertrait, so a built `Node` cannot be introspected back into
//! primitive parameters — and this crate must not modify `kernel-core` /
//! `kernel-implicit` to add one. [`GpuNode`] therefore carries the primitive
//! parameters as data and is the **single source of truth** for both halves:
//!
//! - [`GpuNode::to_node`] builds the ordinary CPU [`Node`] through the public
//!   constructors — the **bit-authoritative** evaluation (this is what the CPU
//!   meshers consume, and what the parity suite uses as the oracle);
//! - [`crate::codegen`] lowers the same description to WGSL for the GPU
//!   evaluator, which is only ever **tolerance-equivalent** (see `NUMERICS.md`).
//!
//! Field-modulated operators ([`GpuNode::offset_by`], [`GpuNode::lerp`]) take
//! the field as a data [`Expr`], not an opaque Rust closure — an `Arc<dyn Fn>`
//! cannot be lowered to WGSL. CPU-only closure fields stay on the CPU path by
//! construction.

use std::sync::Arc;

use kernel_core::math::{Aabb, Affine3A, Quat, Vec3};
use kernel_core::sdf::Sdf;
use kernel_implicit::expr_sdf::{scalar_field, Expr, ExprSdf};
use kernel_implicit::features::{chamfer_difference, chamfer_union, fillet_difference, fillet_union};
use kernel_implicit::grid::VoxelGrid;
use kernel_implicit::lattice::{BeamLattice, Pipe};
use kernel_implicit::ops::Node;
use kernel_implicit::primitives::{Capsule, Cone, Cuboid, Cylinder, Gyroid, Plane, Sphere, Torus};

/// `Arc`-shared dense grid leaf so [`GpuNode::to_node`] does not clone the
/// sample data (a grid can be hundreds of MB).
struct SharedGrid(Arc<VoxelGrid>);

impl Sdf for SharedGrid {
	fn distance(&self, p: Vec3) -> f32 {
		self.0.distance(p)
	}
	fn bounds(&self) -> Aabb {
		self.0.bounds()
	}
}

/// A GPU-lowerable field tree mirroring [`Node`]: 12 primitive leaves and 18
/// combinators (every `Node` variant, plus the four fillet/chamfer feature
/// operators from `kernel_implicit::features`). Construct leaves with the
/// associated functions and combine with the builder methods — the API mirrors
/// `Node`'s so a tree reads the same on both sides.
pub enum GpuNode {
	// ---- the 12 primitive leaves -------------------------------------------
	/// A solid sphere (mirrors [`Sphere`]).
	Sphere { center: Vec3, radius: f32 },
	/// An axis-aligned box (mirrors [`Cuboid`]).
	Cuboid { center: Vec3, half: Vec3 },
	/// A capped cylinder (mirrors [`Cylinder`]).
	Cylinder { a: Vec3, b: Vec3, radius: f32 },
	/// A capped cone / frustum (mirrors [`Cone`]).
	Cone { a: Vec3, b: Vec3, ra: f32, rb: f32 },
	/// A half-space through `point` with outward `normal` (mirrors [`Plane`]).
	Plane { point: Vec3, normal: Vec3 },
	/// A torus (mirrors [`Torus`]).
	Torus { center: Vec3, axis: Vec3, major: f32, minor: f32 },
	/// A capsule (mirrors [`Capsule`]).
	Capsule { a: Vec3, b: Vec3, radius: f32 },
	/// A gyroid TPMS shell (mirrors [`Gyroid`]).
	Gyroid { region: Aabb, scale: f32, thickness: f32 },
	/// A beam lattice given as the same node/strut graph [`BeamLattice::new`]
	/// takes. Lowered to a strut storage buffer; the GPU evaluates the exact
	/// brute-force `min` over the struts (the CPU's spatial-grid acceleration
	/// "never changes the field", so the two agree to rounding).
	Lattice { nodes: Vec<Vec3>, struts: Vec<(u32, u32, f32, f32)> },
	/// A polyline-swept tube, the same data [`Pipe::new`] takes. Lowered to the
	/// same strut storage buffer as [`GpuNode::Lattice`].
	Pipe { path: Vec<Vec3>, radii: Vec<f32> },
	/// A dense sampled grid (mirrors [`VoxelGrid`]), lowered to a storage
	/// buffer with the same trilinear interpolation.
	Grid(Arc<VoxelGrid>),
	/// A user math expression as an SDF leaf (mirrors [`ExprSdf`], including
	/// the mandatory declared Lipschitz bound). NOTE: the CPU evaluates `Expr`
	/// in f64; the GPU evaluates it in f32 (WGSL has no f64) — covered by the
	/// declared GPU tolerance at part scale, see `NUMERICS.md`.
	Expr { expr: Arc<Expr>, lipschitz: f64, bounds: Option<Aabb> },

	// ---- the 18 combinators ------------------------------------------------
	Union(Box<GpuNode>, Box<GpuNode>),
	Intersection(Box<GpuNode>, Box<GpuNode>),
	Difference(Box<GpuNode>, Box<GpuNode>),
	SmoothUnion(Box<GpuNode>, Box<GpuNode>, f32),
	SmoothIntersection(Box<GpuNode>, Box<GpuNode>, f32),
	SmoothDifference(Box<GpuNode>, Box<GpuNode>, f32),
	Offset(Box<GpuNode>, f32),
	Shell(Box<GpuNode>, f32),
	/// Rigid + uniform-scale transform; stores the forward affine (the inverse
	/// and scale are derived at lowering exactly as `ops::Xform::new` does).
	Transform(Box<GpuNode>, Affine3A),
	LinearArray(Box<GpuNode>, Vec3, u32),
	/// `(child, center, unit axis, count, step_angle)` — axis stored unit
	/// length, mirroring `Node::circular_pattern`.
	PolarArray(Box<GpuNode>, Vec3, Vec3, u32, f32),
	/// `(child, point, unit normal)` — mirroring `Node::mirror`.
	Mirror(Box<GpuNode>, Vec3, Vec3),
	/// Position-varying offset driven by a data field: `d(p) − clamp(expr(p),
	/// ±max_abs)`. Same Lipschitz caveats as `Node::offset_by`.
	OffsetBy(Box<GpuNode>, Arc<Expr>, f32),
	/// Pointwise blend `(1−w)·a + w·b`, `w = clamp(expr(p), 0, 1)`.
	LerpBlend(Box<GpuNode>, Box<GpuNode>, Arc<Expr>),
	FilletUnion(Box<GpuNode>, Box<GpuNode>, f32),
	ChamferUnion(Box<GpuNode>, Box<GpuNode>, f32),
	FilletDifference(Box<GpuNode>, Box<GpuNode>, f32),
	ChamferDifference(Box<GpuNode>, Box<GpuNode>, f32),
}

#[inline]
fn boxed(n: GpuNode) -> Box<GpuNode> {
	Box::new(n)
}

impl GpuNode {
	// ---- leaf constructors (mirror the primitive `new`s) -------------------

	pub fn sphere(center: Vec3, radius: f32) -> GpuNode {
		GpuNode::Sphere { center, radius }
	}

	pub fn cuboid(center: Vec3, half: Vec3) -> GpuNode {
		GpuNode::Cuboid { center, half }
	}

	pub fn cylinder(a: Vec3, b: Vec3, radius: f32) -> GpuNode {
		GpuNode::Cylinder { a, b, radius }
	}

	pub fn cone(a: Vec3, b: Vec3, ra: f32, rb: f32) -> GpuNode {
		GpuNode::Cone { a, b, ra, rb }
	}

	pub fn plane(point: Vec3, normal: Vec3) -> GpuNode {
		GpuNode::Plane { point, normal }
	}

	pub fn torus(center: Vec3, axis: Vec3, major: f32, minor: f32) -> GpuNode {
		GpuNode::Torus { center, axis, major, minor }
	}

	pub fn capsule(a: Vec3, b: Vec3, radius: f32) -> GpuNode {
		GpuNode::Capsule { a, b, radius }
	}

	pub fn gyroid(region: Aabb, scale: f32, thickness: f32) -> GpuNode {
		GpuNode::Gyroid { region, scale, thickness }
	}

	/// A beam lattice from the same explicit graph [`BeamLattice::new`] takes
	/// (indices in range, radii positive — validated by the CPU constructor in
	/// [`GpuNode::to_node`] and by lowering).
	pub fn lattice(nodes: Vec<Vec3>, struts: Vec<(u32, u32, f32, f32)>) -> GpuNode {
		GpuNode::Lattice { nodes, struts }
	}

	/// A tube swept along `path` with per-vertex `radii` ([`Pipe::new`] data).
	pub fn pipe(path: Vec<Vec3>, radii: Vec<f32>) -> GpuNode {
		GpuNode::Pipe { path, radii }
	}

	/// A dense sampled signed-distance grid leaf.
	pub fn grid(grid: Arc<VoxelGrid>) -> GpuNode {
		GpuNode::Grid(grid)
	}

	/// A user expression leaf with its REQUIRED declared Lipschitz bound (see
	/// [`ExprSdf`] for the honest contract; `to_node` asserts the bound).
	pub fn expr(expr: Arc<Expr>, lipschitz: f64, bounds: Option<Aabb>) -> GpuNode {
		GpuNode::Expr { expr, lipschitz, bounds }
	}

	// ---- combinator builders (mirror `Node`'s API) --------------------------

	pub fn union(self, other: GpuNode) -> GpuNode {
		GpuNode::Union(boxed(self), boxed(other))
	}

	pub fn intersection(self, other: GpuNode) -> GpuNode {
		GpuNode::Intersection(boxed(self), boxed(other))
	}

	pub fn difference(self, other: GpuNode) -> GpuNode {
		GpuNode::Difference(boxed(self), boxed(other))
	}

	pub fn smooth_union(self, other: GpuNode, k: f32) -> GpuNode {
		GpuNode::SmoothUnion(boxed(self), boxed(other), k)
	}

	pub fn smooth_intersection(self, other: GpuNode, k: f32) -> GpuNode {
		GpuNode::SmoothIntersection(boxed(self), boxed(other), k)
	}

	pub fn smooth_difference(self, other: GpuNode, k: f32) -> GpuNode {
		GpuNode::SmoothDifference(boxed(self), boxed(other), k)
	}

	pub fn offset(self, t: f32) -> GpuNode {
		GpuNode::Offset(boxed(self), t)
	}

	pub fn shell(self, t: f32) -> GpuNode {
		GpuNode::Shell(boxed(self), t)
	}

	pub fn transform(self, fwd: Affine3A) -> GpuNode {
		GpuNode::Transform(boxed(self), fwd)
	}

	pub fn translate(self, v: Vec3) -> GpuNode {
		self.transform(Affine3A::from_translation(v))
	}

	pub fn rotate(self, q: Quat) -> GpuNode {
		self.transform(Affine3A::from_quat(q))
	}

	pub fn scale(self, s: f32) -> GpuNode {
		self.transform(Affine3A::from_scale(Vec3::splat(s)))
	}

	pub fn linear_pattern(self, step: Vec3, count: usize) -> GpuNode {
		GpuNode::LinearArray(boxed(self), step, count.max(1) as u32)
	}

	pub fn circular_pattern(self, center: Vec3, axis: Vec3, step_angle: f32, count: usize) -> GpuNode {
		// Same axis sanitation as Node::circular_pattern (degenerate axis → Z).
		let axis = axis.try_normalize().unwrap_or(Vec3::Z);
		GpuNode::PolarArray(boxed(self), center, axis, count.max(1) as u32, step_angle)
	}

	pub fn mirror(self, point: Vec3, normal: Vec3) -> GpuNode {
		GpuNode::Mirror(boxed(self), point, normal.normalize_or_zero())
	}

	/// Position-varying surface offset by a DATA field (an [`Expr`], so it can
	/// be lowered). Mirrors `Node::offset_by` (same `max_abs` clamp/assert).
	pub fn offset_by(self, field: Arc<Expr>, max_abs: f32) -> GpuNode {
		assert!(max_abs.is_finite() && max_abs >= 0.0, "offset_by: max_abs must be finite and >= 0, got {max_abs}");
		GpuNode::OffsetBy(boxed(self), field, max_abs)
	}

	/// Pointwise lerp toward `other` weighted by a DATA field. Mirrors
	/// `Node::lerp`.
	pub fn lerp(self, other: GpuNode, field: Arc<Expr>) -> GpuNode {
		GpuNode::LerpBlend(boxed(self), boxed(other), field)
	}

	pub fn fillet_union(self, other: GpuNode, r: f32) -> GpuNode {
		GpuNode::FilletUnion(boxed(self), boxed(other), r.max(0.0))
	}

	pub fn chamfer_union(self, other: GpuNode, r: f32) -> GpuNode {
		GpuNode::ChamferUnion(boxed(self), boxed(other), r.max(0.0))
	}

	pub fn fillet_difference(self, other: GpuNode, r: f32) -> GpuNode {
		GpuNode::FilletDifference(boxed(self), boxed(other), r.max(0.0))
	}

	pub fn chamfer_difference(self, other: GpuNode, r: f32) -> GpuNode {
		GpuNode::ChamferDifference(boxed(self), boxed(other), r.max(0.0))
	}

	// ---- the CPU authority ---------------------------------------------------

	/// Build the ordinary CPU [`Node`] for this tree through the public
	/// kernel-implicit constructors. This is the **bit-authoritative**
	/// evaluation — the same code path every CPU mesher consumes — and the
	/// oracle the GPU parity suite compares against.
	pub fn to_node(&self) -> Node {
		match self {
			GpuNode::Sphere { center, radius } => Node::primitive(Sphere::new(*center, *radius)),
			GpuNode::Cuboid { center, half } => Node::primitive(Cuboid::new(*center, *half)),
			GpuNode::Cylinder { a, b, radius } => Node::primitive(Cylinder::new(*a, *b, *radius)),
			GpuNode::Cone { a, b, ra, rb } => Node::primitive(Cone::new(*a, *b, *ra, *rb)),
			GpuNode::Plane { point, normal } => Node::primitive(Plane::new(*point, *normal)),
			GpuNode::Torus { center, axis, major, minor } => Node::primitive(Torus::new(*center, *axis, *major, *minor)),
			GpuNode::Capsule { a, b, radius } => Node::primitive(Capsule::new(*a, *b, *radius)),
			GpuNode::Gyroid { region, scale, thickness } => Node::primitive(Gyroid::new(*region, *scale, *thickness)),
			GpuNode::Lattice { nodes, struts } => Node::primitive(BeamLattice::new(nodes.clone(), struts.clone())),
			GpuNode::Pipe { path, radii } => Node::primitive(Pipe::new(path.clone(), radii.clone())),
			GpuNode::Grid(g) => Node::primitive(SharedGrid(g.clone())),
			GpuNode::Expr { expr, lipschitz, bounds } => Node::primitive(ExprSdf::new(expr.clone(), *lipschitz, *bounds)),
			GpuNode::Union(a, b) => a.to_node().union(b.to_node()),
			GpuNode::Intersection(a, b) => a.to_node().intersection(b.to_node()),
			GpuNode::Difference(a, b) => a.to_node().difference(b.to_node()),
			GpuNode::SmoothUnion(a, b, k) => a.to_node().smooth_union(b.to_node(), *k),
			GpuNode::SmoothIntersection(a, b, k) => a.to_node().smooth_intersection(b.to_node(), *k),
			GpuNode::SmoothDifference(a, b, k) => a.to_node().smooth_difference(b.to_node(), *k),
			GpuNode::Offset(a, t) => a.to_node().offset(*t),
			GpuNode::Shell(a, t) => a.to_node().shell(*t),
			GpuNode::Transform(a, fwd) => a.to_node().transform(*fwd),
			GpuNode::LinearArray(a, step, count) => a.to_node().linear_pattern(*step, *count as usize),
			GpuNode::PolarArray(a, center, axis, count, ang) => a.to_node().circular_pattern(*center, *axis, *ang, *count as usize),
			GpuNode::Mirror(a, point, normal) => a.to_node().mirror(*point, *normal),
			GpuNode::OffsetBy(a, field, max_abs) => a.to_node().offset_by(scalar_field(field.clone()), *max_abs),
			GpuNode::LerpBlend(a, b, field) => a.to_node().lerp(b.to_node(), scalar_field(field.clone())),
			GpuNode::FilletUnion(a, b, r) => fillet_union(a.to_node(), b.to_node(), *r),
			GpuNode::ChamferUnion(a, b, r) => chamfer_union(a.to_node(), b.to_node(), *r),
			GpuNode::FilletDifference(a, b, r) => fillet_difference(a.to_node(), b.to_node(), *r),
			GpuNode::ChamferDifference(a, b, r) => chamfer_difference(a.to_node(), b.to_node(), *r),
		}
	}

	/// Bound containing the surface, as reported by the CPU tree.
	pub fn bounds(&self) -> Aabb {
		self.to_node().bounds()
	}
}
