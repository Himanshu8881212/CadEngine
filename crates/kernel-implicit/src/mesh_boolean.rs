// Copyright (c) LMCAD. Licensed under the MIT License.

//! Robust **watertight** booleans between two closed triangle meshes, via the
//! implicit domain.
//!
//! The exact mesh-arrangement boolean in `kernel_brep::mesh_boolean` co-refines
//! triangle against triangle and classifies the pieces with floating-point
//! predicates. That is exact for planar / coarsely faceted inputs, but finely
//! tessellated *curved* meshes (high-resolution spheres, near-coplanar facets)
//! stress the predicates and can yield an orientation-inconsistent,
//! **non-watertight** result whose volume is close but whose surface is not a
//! clean manifold.
//!
//! This module takes the other road. Each input mesh is exposed as a signed
//! distance field ([`MeshSdf`], via a generalized winding number for the sign),
//! the two fields are combined with the standard CSG min/max operators, and the
//! combined field is re-meshed with [`manifold_dual_contour`] — a dual mesher
//! that is **2-manifold by construction even at the sharp concave crease** a
//! difference carves (where naive Surface Nets would leave a few non-manifold
//! edges). The result is a **guaranteed closed, 2-manifold, sharp-seamed** mesh
//! regardless of how pathological the inputs are — at the cost of a
//! *voxel-approximate* seam (the surface is resampled at `voxel` resolution
//! rather than following the exact input facets). Pick this path when a
//! guaranteed-closed result matters more than an exact seam; pick the arrangement
//! path when the inputs are planar enough that an exact seam is achievable.

use kernel_core::{Aabb, Mesh, Sdf, Vec3};

use crate::manifold_dc::manifold_dual_contour;
use crate::meshsdf::MeshSdf;

/// Which CSG boolean [`mesh_boolean_implicit`] evaluates.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BoolOp {
	/// `A ∪ B` — the union of the two solids.
	Union,
	/// `A − B` — `A` with `B` carved away.
	Difference,
	/// `A ∩ B` — the region inside both solids.
	Intersection,
}

/// Combine two signed distances under `op`. The convention matches [`MeshSdf`]:
/// negative inside, positive outside. These min/max operators have the correct
/// zero level set (the boolean's boundary), which is all the mesher samples.
fn combine(op: BoolOp, da: f32, db: f32) -> f32 {
	match op {
		BoolOp::Union => da.min(db),
		BoolOp::Intersection => da.max(db),
		BoolOp::Difference => da.max(-db),
	}
}

/// The CSG combination of two [`MeshSdf`] fields, presented as a single [`Sdf`]
/// so it can be fed to the [`manifold_dual_contour`] mesher.
struct CsgMeshField<'a> {
	a: &'a MeshSdf,
	b: &'a MeshSdf,
	op: BoolOp,
	domain: Aabb,
}

impl Sdf for CsgMeshField<'_> {
	fn distance(&self, p: Vec3) -> f32 {
		combine(self.op, self.a.distance(p), self.b.distance(p))
	}

	fn bounds(&self) -> Aabb {
		self.domain
	}
}

/// A copy of `m` re-oriented outward (positive signed volume), matching the
/// outward winding the mesher produces — so a verbatim-returned operand is wound
/// consistently with a meshed result.
fn oriented(m: &Mesh) -> Mesh {
	let mut c = m.clone();
	c.ensure_outward();
	c
}

/// Concatenate two solids into one mesh (their disjoint union), each oriented
/// outward and normals recomputed over the merged vertex set.
fn concat_outward(a: &Mesh, b: &Mesh) -> Mesh {
	let mut out = oriented(a);
	let bo = oriented(b);
	let base = out.positions.len() as u32;
	out.positions.extend_from_slice(&bo.positions);
	out.indices.extend(bo.indices.iter().map(|&i| i + base));
	out.compute_normals();
	out
}

/// Watertight boolean of two closed triangle meshes via the implicit path.
///
/// Each input is turned into a signed distance field, the fields are combined
/// with the CSG `op`, and the result is re-meshed by [`manifold_dual_contour`]
/// — so the output is always a closed **2-manifold** with a sharp seam, even on
/// inputs where the exact mesh-arrangement boolean breaks down (see the
/// [module docs](self)). The seam is voxel-approximate. Operand winding need not
/// be outward — each is normalized first, so an inside-out (reversed) mesh
/// booleans identically to its outward twin.
///
/// `voxel` is the sampling resolution (smaller = finer & slower). Pass a
/// non-positive value to auto-pick ≈ 1/96 of the combined bounding-box diagonal
/// — a bounded lattice that resolves comparably-sized inputs well, but can lose a
/// feature much smaller than the overall model (the inherent limit of a voxel
/// method; pass an explicit finer `voxel` if you need it).
///
/// Trivial cases are handled exactly without meshing: an empty operand follows
/// the algebra (`A ∪ ∅ = A`, `A − ∅ = A`, `A ∩ ∅ = ∅`), and operands with
/// **disjoint bounding boxes** cannot interact, so `A ∪ B` concatenates them,
/// `A − B = A`, and `A ∩ B = ∅` — returning the operand(s) verbatim (re-oriented
/// outward) rather than resampling a domain dominated by the empty gap between
/// them.
///
/// ```
/// use kernel_implicit::{mesh_boolean_implicit, BoolOp, Sphere};
/// use kernel_core::{surface_nets_sdf_f64, Sdf};
/// use kernel_core::math::Vec3;
/// // Two overlapping spheres, meshed from analytic fields, then intersected.
/// let sa = Sphere::new(Vec3::ZERO, 8.0);
/// let sb = Sphere::new(Vec3::X * 8.0, 8.0);
/// let a = surface_nets_sdf_f64(&sa, sa.bounds(), 0.6).to_mesh();
/// let b = surface_nets_sdf_f64(&sb, sb.bounds(), 0.6).to_mesh();
/// let lens = mesh_boolean_implicit(&a, &b, BoolOp::Intersection, 0.0);
/// assert!(lens.is_watertight(), "implicit-path result is always watertight");
/// assert!(lens.triangle_count() > 50);
/// ```
pub fn mesh_boolean_implicit(a: &Mesh, b: &Mesh, op: BoolOp, voxel: f64) -> Mesh {
	// Degenerate operands: follow the boolean algebra exactly, returning the
	// surviving operand (oriented) rather than resampling it.
	let a_solid = a.indices.len() >= 3;
	let b_solid = b.indices.len() >= 3;
	if !a_solid || !b_solid {
		return match op {
			BoolOp::Union if a_solid => oriented(a),
			BoolOp::Union if b_solid => oriented(b),
			BoolOp::Difference if a_solid => oriented(a),
			_ => Mesh::new(),
		};
	}

	// Provably-disjoint bounding boxes ⇒ the solids cannot interact, so the result
	// is exact and trivial. Short-circuit instead of meshing a domain dominated by
	// the gap between them — which would under-resolve the inputs (or, for a very
	// large gap, overflow the lattice and silently return nothing).
	if !a.aabb().intersection(b.aabb()).is_valid() {
		return match op {
			BoolOp::Union => concat_outward(a, b),
			BoolOp::Difference => oriented(a),
			BoolOp::Intersection => Mesh::new(),
		};
	}

	// Orient outward before building the fields: [`MeshSdf`] takes its inside/outside
	// sign from the winding *number*, which inverts for a globally inward-wound input
	// — so without this an inside-out operand would read as empty space and silently
	// drop out of the boolean.
	let (ao, bo) = (oriented(a), oriented(b));
	let sa = MeshSdf::new(&ao);
	let sb = MeshSdf::new(&bo);
	let ba = sa.bounds();
	let bb = sb.bounds();

	// The boolean's surface is confined to a domain we can pick per op: a union
	// spans both boxes, a difference stays within `A`, an intersection within the
	// overlap (always valid here — disjoint bounds were handled above).
	let domain = match op {
		BoolOp::Union => ba.union(bb),
		BoolOp::Difference => ba,
		BoolOp::Intersection => ba.intersection(bb),
	};

	// Choose the resolution from the combined extent so it is scale-correct, then
	// pad the domain a couple of voxels so the mesher brackets zero crossings that
	// graze the domain wall.
	let diag = ba.union(bb).diagonal();
	let voxel = if voxel.is_finite() && voxel > 0.0 { voxel as f32 } else { (diag / 96.0).max(1e-6) };
	let domain = domain.pad(2.0 * voxel);

	let field = CsgMeshField { a: &sa, b: &sb, op, domain };
	manifold_dual_contour(&field, domain, voxel)
}
