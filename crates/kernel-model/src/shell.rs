// Copyright (c) LMCAD. Licensed under the MIT License.

//! Shell/hollow and signed surface offset for B-rep solids — **via the voxel
//! route, honestly labeled** (the PROGRESS.md Tier-1 "shell/hollow (voxel
//! route, honestly labeled)" capability).
//!
//! The exact B-rep half of the kernel has no general face offset (offsetting
//! every analytic face and re-intersecting all the neighbouring trims is
//! classic hard B-rep work, out of scope today), so these helpers route
//! through the hybrid's SDF half instead — the same move as
//! [`crate::watertight_mesh`]: tessellate → winding-number [`MeshSdf`] →
//! shifted/banded field → Manifold Dual Contouring.
//!
//! **What you get — and what you do not.** The results are voxel-accurate,
//! watertight *meshes*: surfaces land within about half a `voxel` of the true
//! offset surface, plus the chord error of the input tessellation. They are
//! NOT exact B-rep offsets — analytic faces come back as triangles, and sharp
//! corners of the input reproduce (and, for the shell's inner cavity, round)
//! at ~`voxel` scale. The [`offset_to_solid`] / [`shell_to_solid`]
//! conveniences wrap the mesh back into a **faceted** B-rep (one planar face
//! per triangle, no analytic surfaces) so the result can re-enter exact
//! booleans and measures; the faceting is stated here, never hidden.

use kernel_core::math::{Aabb, Vec3};
use kernel_core::mesh::Mesh;
use kernel_core::mesher::Resolution;
use kernel_core::sdf::Sdf;
use kernel_implicit::{manifold_dual_contour, MeshSdf};

/// The signed-offset field `d(p) − delta` over a winding-number [`MeshSdf`]
/// base. Because `d` is a true Euclidean SDF (nearest-triangle distance,
/// winding-number sign), `d − delta` is exactly the SDF of the offset solid —
/// the Minkowski sum with a ball for `delta > 0`, the erosion for `delta < 0`.
struct OffsetField {
	base: MeshSdf,
	delta: f32,
}

impl Sdf for OffsetField {
	fn distance(&self, p: Vec3) -> f32 {
		self.base.distance(p) - self.delta
	}

	fn bounds(&self) -> Aabb {
		// A positive offset moves the surface outward by `delta`; a negative one
		// stays strictly inside the base bounds.
		self.base.bounds().pad(self.delta.max(0.0))
	}
}

/// The shell field `max(d, −(d + t))`: material exactly where `−t ≤ d ≤ 0`,
/// i.e. the original solid minus its erosion by `t` — a closed wall of
/// thickness `t` whose outer surface is the original surface. Both branches
/// are 1-Lipschitz true distances near their own surface, so the extracted
/// outer (`d = 0`) and inner (`d = −t`) level sets are voxel-accurate; the
/// `max` crease sits mid-wall (`d = −t/2`), away from either surface.
struct ShellField {
	base: MeshSdf,
	thickness: f32,
}

impl Sdf for ShellField {
	fn distance(&self, p: Vec3) -> f32 {
		let d = self.base.distance(p);
		d.max(-(d + self.thickness))
	}

	fn bounds(&self) -> Aabb {
		// The wall keeps the outer surface, so the base bound already contains it.
		self.base.bounds()
	}
}

/// Signed surface offset of a B-rep solid **via the voxel route** — honest
/// routing per the kernel rules: the result is a voxel-accurate watertight
/// **mesh**, NOT an exact B-rep offset, and this doc says so instead of
/// pretending otherwise.
///
/// The solid is tessellated ([`kernel_brep::tessellate_default`]), lifted into
/// a winding-number signed-distance field ([`MeshSdf`]) `d`, and the level set
/// of `d(p) − delta` is re-extracted with Manifold Dual Contouring at grid
/// pitch `voxel` (mm):
///
/// - `delta > 0` **grows** the solid: mathematically the Minkowski sum with a
///   ball of radius `delta`, so convex edges and corners gain a genuine
///   `delta`-radius round (that round is exact offset geometry, not a voxel
///   artifact);
/// - `delta < 0` **shrinks** it (erosion): any region thinner than
///   `2·|delta|` disappears, and a solid smaller than `|delta|` everywhere
///   vanishes to an **empty mesh** (honest empty, never an error);
/// - `delta = 0` degenerates to the plain voxel re-heal of
///   [`crate::watertight_mesh`].
///
/// Accuracy: input-tessellation chord error + roughly `voxel/2` surface
/// placement; sharp features of the *original* surface reproduce at ~`voxel`
/// fidelity. The extraction grid is inflated by `|delta|` plus a 3-voxel
/// apron so a grown surface is never clipped. Degenerate input (empty solid,
/// non-finite `delta`, `voxel ≤ 0`) yields an empty mesh.
pub fn offset_mesh(solid: &kernel_brep::Solid, delta: f64, voxel: f32) -> Mesh {
	let tess = kernel_brep::tessellate_default(solid);
	if tess.triangle_count() == 0 || !delta.is_finite() || !voxel.is_finite() || voxel <= 0.0 {
		return Mesh::new();
	}
	let field = OffsetField { base: MeshSdf::new(&tess), delta: delta as f32 };
	// Inflate by |delta| + a few voxels: a positive offset needs the headroom, and
	// the apron keeps boundary cells fully sampled on either sign.
	let domain = field.base.bounds().pad(delta.abs() as f32 + voxel * 3.0);
	manifold_dual_contour(&field, domain, Resolution::VoxelSize(voxel))
}

/// Hollow a B-rep solid into a closed shell of wall `thickness` (mm), keeping
/// the OUTER surface — **via the voxel route**, honestly labeled: the result
/// is a voxel-accurate watertight **mesh** (two nested closed surfaces), NOT
/// an exact B-rep shell, and there is no face opening — the cavity is sealed
/// inside (drain holes are a boolean away).
///
/// Field: with `d` the winding-number SDF of the tessellated solid, the wall
/// is `max(d, −(d + thickness))` — material exactly where
/// `−thickness ≤ d ≤ 0`, i.e. the original solid minus its erosion by
/// `thickness`, so outer dimensions are preserved (to ~`voxel/2` + input
/// chord error). The **inner cavity is the erosion surface**: its concave
/// corners round at ~`voxel` scale — the honest voxel-route caveat; an exact
/// B-rep shell would keep them sharp. A `thickness ≤ 0` yields an empty mesh
/// (no wall to keep); a `thickness` at or beyond the part's inradius leaves
/// no cavity at all (the "shell" is just the re-healed solid).
pub fn shell_mesh(solid: &kernel_brep::Solid, thickness: f64, voxel: f32) -> Mesh {
	let tess = kernel_brep::tessellate_default(solid);
	if tess.triangle_count() == 0 || !thickness.is_finite() || thickness <= 0.0 || !voxel.is_finite() || voxel <= 0.0 {
		return Mesh::new();
	}
	let field = ShellField { base: MeshSdf::new(&tess), thickness: thickness as f32 };
	// The outer surface IS the original surface, so only the sampling apron pads.
	let domain = field.base.bounds().pad(voxel * 3.0);
	manifold_dual_contour(&field, domain, Resolution::VoxelSize(voxel))
}

/// [`offset_mesh`] wrapped back into a B-rep [`kernel_brep::Solid`] via
/// [`kernel_brep::solid_from_mesh`] — a **faceted** B-rep (one planar face per
/// voxel-extracted triangle; no analytic surfaces), stated honestly so nobody
/// mistakes it for an exact offset body. Useful to re-enter exact booleans,
/// `validate`, `volume` and STEP export with an offset part. An empty offset
/// result (see [`offset_mesh`]) wraps to the default empty solid.
pub fn offset_to_solid(solid: &kernel_brep::Solid, delta: f64, voxel: f32) -> kernel_brep::Solid {
	let mesh = offset_mesh(solid, delta, voxel);
	if mesh.triangle_count() == 0 {
		return kernel_brep::Solid::default();
	}
	kernel_brep::solid_from_mesh(&mesh)
}

/// [`shell_mesh`] wrapped back into a B-rep [`kernel_brep::Solid`] via
/// [`kernel_brep::solid_from_mesh`] — a **faceted** B-rep (one planar face per
/// voxel-extracted triangle; no analytic surfaces), stated honestly. The
/// hollow topology survives the wrap: the cavity arrives as a second, nested
/// shell of the solid (`validate(..).shells == 2`), not as filled material.
/// An empty shell result (see [`shell_mesh`]) wraps to the default empty solid.
pub fn shell_to_solid(solid: &kernel_brep::Solid, thickness: f64, voxel: f32) -> kernel_brep::Solid {
	let mesh = shell_mesh(solid, thickness, voxel);
	if mesh.triangle_count() == 0 {
		return kernel_brep::Solid::default();
	}
	kernel_brep::solid_from_mesh(&mesh)
}
