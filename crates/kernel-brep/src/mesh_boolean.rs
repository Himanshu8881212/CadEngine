// Copyright (c) LMCAD. Licensed under the MIT License.

//! Booleans between two arbitrary triangle meshes (two tessellated solids).
//!
//! The [`booleans`](crate::booleans) mesh-arrangement algorithm co-refines
//! triangle against triangle, which is already general — it does not require the
//! input faces to be the planar faces of a B-rep. So a boolean between any two
//! closed triangle meshes is just: wrap each triangle as a planar [`FaceInput`],
//! run the arrangement, and tessellate the result. The seam follows the input
//! tessellation (it is *not* re-fitted to an underlying analytic surface), so the
//! accuracy is that of the input meshes — refine them for a finer result.
//!
//! **Precondition:** each input must be a *closed* triangle mesh (a watertight
//! 2-manifold). Winding need not be outward — it is normalised here — but an open
//! or non-manifold input has no well-defined inside, so the result is undefined.
//!
//! **Robustness.** The arrangement is exact and watertight for planar / coarsely
//! faceted inputs (boxes, [`crate::cuboid`]/[`crate::extrude`]d B-rep solids). Finely
//! tessellated *curved* inputs (e.g. high-resolution spheres) stress the
//! floating-point co-refinement and classification and can yield an
//! **orientation-inconsistent, non-watertight** result whose *volume* is still close
//! but whose surface is not a clean manifold — a known limitation of inexact
//! mesh-arrangement booleans. For a guaranteed-watertight curved cut, prefer the
//! mesh-vs-analytic-surface path in [`crate::curved_boolean`]
//! (`subtract_sphere`/`drill_cylinder`/…), which trims and caps exactly.

use kernel_core::math::DVec3;
use kernel_core::mesh::Mesh;

use crate::booleans::{difference, intersection, union};
use crate::geom::Surface;
use crate::ssi::{snap_seam_to_intersection, ImplicitSurface};
use crate::tessellate::tessellate_default;
use crate::topo::{FaceInput, Solid};

/// Wrap a triangle mesh as a [`Solid`], each triangle a planar face. The mesh is
/// first re-oriented outward (`ensure_outward`), so a closed input with the
/// opposite (clockwise-from-outside) winding still builds a correctly-oriented
/// solid rather than an inside-out one. Public so mixed-representation callers
/// (e.g. `kernel-model`'s `hybrid_boolean`) can put a scanned/meshed body on one
/// side of the exact arrangement; the topological quality of the result is that
/// of the input mesh (a closed 2-manifold mesh wraps to a closed solid, an open
/// soup wraps to an open shell — `validate` tells the truth either way).
pub fn solid_from_mesh(mesh: &Mesh) -> Solid {
	let mut m = mesh.clone();
	m.ensure_outward();
	let positions: Vec<DVec3> = m.positions.iter().map(|p| p.as_dvec3()).collect();
	let mut faces = Vec::with_capacity(m.indices.len() / 3);
	for t in m.indices.chunks_exact(3) {
		let (a, b, c) = (positions[t[0] as usize], positions[t[1] as usize], positions[t[2] as usize]);
		let normal = (b - a).cross(c - a).normalize_or_zero();
		faces.push(FaceInput { boundary: vec![t[0], t[1], t[2]], surface: Surface::Plane { origin: a, normal } });
	}
	Solid::from_faces(positions, faces)
}

/// Union of two closed triangle meshes (`A ∪ B`).
pub fn mesh_union(a: &Mesh, b: &Mesh) -> Mesh {
	tessellate_default(&union(&solid_from_mesh(a), &solid_from_mesh(b)))
}

/// Difference of two closed triangle meshes (`A − B`).
pub fn mesh_difference(a: &Mesh, b: &Mesh) -> Mesh {
	tessellate_default(&difference(&solid_from_mesh(a), &solid_from_mesh(b)))
}

/// Intersection of two closed triangle meshes (`A ∩ B`).
pub fn mesh_intersection(a: &Mesh, b: &Mesh) -> Mesh {
	tessellate_default(&intersection(&solid_from_mesh(a), &solid_from_mesh(b)))
}

/// Which boolean to evaluate — shared by [`exact_boolean`] /
/// [`exact_boolean_auto`] and the tolerant solid path
/// ([`crate::heal::boolean_tolerant`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MeshBoolOp {
	Union,
	Difference,
	Intersection,
}

/// Exact-curved boolean of two solids bounded by the analytic surfaces `fa`, `gb`:
/// run the tessellation-level mesh boolean, then snap the seam onto the exact
/// `fa ∩ gb` intersection so the join is analytically exact (to `f32` resolution)
/// rather than following the facets. `band` is the snap radius — set it near the
/// inputs' edge length so only the seam, not a wide neighbourhood, is pulled in.
pub fn exact_boolean<F, G>(a: &Mesh, b: &Mesh, fa: &F, gb: &G, op: MeshBoolOp, band: f64) -> Mesh
where
	F: ImplicitSurface + ?Sized,
	G: ImplicitSurface + ?Sized,
{
	let mut out = match op {
		MeshBoolOp::Union => mesh_union(a, b),
		MeshBoolOp::Difference => mesh_difference(a, b),
		MeshBoolOp::Intersection => mesh_intersection(a, b),
	};
	snap_seam_to_intersection(&mut out, fa, gb, band);
	out
}

/// A snap-`band` for [`exact_boolean`] inferred from a mesh's tessellation: roughly
/// the facet chord error, `mean_edge² / bbox_diagonal` (≈ `e²/8R`). Scale-correct —
/// it grows with the model and shrinks as the mesh is refined.
pub fn auto_seam_band(mesh: &Mesh) -> f64 {
	if mesh.indices.len() < 3 {
		return 0.0;
	}
	let mut sum = 0.0;
	let mut n = 0u64;
	for t in mesh.indices.chunks_exact(3) {
		let p = [mesh.positions[t[0] as usize], mesh.positions[t[1] as usize], mesh.positions[t[2] as usize]];
		for i in 0..3 {
			sum += (p[(i + 1) % 3] - p[i]).length() as f64;
			n += 1;
		}
	}
	let (mut lo, mut hi) = (mesh.positions[0], mesh.positions[0]);
	for &v in &mesh.positions {
		lo = lo.min(v);
		hi = hi.max(v);
	}
	let mean_edge = sum / n.max(1) as f64;
	let diag = (hi - lo).length() as f64;
	2.0 * mean_edge * mean_edge / diag.max(1e-9)
}

/// [`exact_boolean`] with the snap band chosen automatically from the result's
/// tessellation (see [`auto_seam_band`]) — a true one-call exact two-solid boolean.
///
/// ```
/// use kernel_brep::{exact_boolean_auto, sphere, tessellate_default, MeshBoolOp, Surface};
/// use kernel_brep::math::DVec3;
/// // Intersect two overlapping spheres; the seam is snapped onto their exact circle.
/// let sa = Surface::Sphere { center: DVec3::ZERO, radius: 8.0 };
/// let sb = Surface::Sphere { center: DVec3::X * 8.0, radius: 8.0 };
/// let ma = tessellate_default(&sphere(DVec3::ZERO, 8.0, 20, 14));
/// let mb = tessellate_default(&sphere(DVec3::X * 8.0, 8.0, 20, 14));
/// let lens = exact_boolean_auto(&ma, &mb, &sa, &sb, MeshBoolOp::Intersection);
/// assert!(lens.triangle_count() > 50);
/// ```
pub fn exact_boolean_auto<F, G>(a: &Mesh, b: &Mesh, fa: &F, gb: &G, op: MeshBoolOp) -> Mesh
where
	F: ImplicitSurface + ?Sized,
	G: ImplicitSurface + ?Sized,
{
	let mut out = match op {
		MeshBoolOp::Union => mesh_union(a, b),
		MeshBoolOp::Difference => mesh_difference(a, b),
		MeshBoolOp::Intersection => mesh_intersection(a, b),
	};
	let band = auto_seam_band(&out);
	snap_seam_to_intersection(&mut out, fa, gb, band);
	out
}
