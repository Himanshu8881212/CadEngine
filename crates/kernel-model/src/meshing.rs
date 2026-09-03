// Copyright (c) LMCAD. Licensed under the MIT License.

//! Meshing routes for an exact B-rep solid: the exact tessellation, the voxel
//! heal through the implicit half, and the [`routed_mesh`] policy that picks
//! between them and reports which one it took.

use kernel_core::mesh::Mesh;
use kernel_core::mesher::Resolution;
use kernel_core::sdf::Sdf;
use kernel_implicit::manifold_dual_contour;

/// Heal a B-rep [`kernel_brep::Solid`] into a watertight mesh **via the voxel half** —
/// the hybrid's core move. The solid is tessellated, lifted into a winding-number
/// signed-distance field ([`kernel_implicit::MeshSdf`]), and re-meshed with Manifold
/// Dual Contouring. This recovers a watertight, 2-manifold mesh for a solid whose
/// *exact* tessellation has T-junctions / cracks — e.g. the faceted curved↔planar
/// joints of a drilled or filleted curved part — so the engine can hand a printable
/// mesh back for geometry the exact path can't tessellate watertight. `voxel_size`
/// trades fidelity for speed; a closed surface is recovered regardless of the input's
/// per-face cracks because the SDF sign is global (winding number), not edge-based.
pub fn watertight_mesh(solid: &kernel_brep::Solid, voxel_size: f32) -> Mesh {
	watertight_mesh_of(&kernel_brep::tessellate_default(solid), voxel_size)
}

/// Mesh a B-rep [`kernel_brep::Solid`] to a watertight surface at chord tolerance `tol`
/// (mm), **preferring the exact analytic path**: [`kernel_brep::tessellate_adaptive_tol`]
/// follows the true surfaces (a micron-smooth cylinder/cone/sphere, not a voxel grid) and
/// is watertight by its shared-edge invariant. Only when the exact tessellation is *not*
/// watertight (a self-intersecting input, or a topology the adaptive stitcher can't seal)
/// does it fall back to the voxel heal ([`watertight_mesh`]). This is the precision
/// counterpart of [`Document::watertight_brep_mesh`] — an AI gets crisp, resin-quality
/// geometry for the common analytic parts and a guaranteed-watertight mesh for the rest.
pub fn precise_mesh(solid: &kernel_brep::Solid, tol: f64) -> Mesh {
	let exact = kernel_brep::tessellate_adaptive_tol(solid, tol);
	if exact.is_watertight() {
		exact
	} else {
		// Voxel fallback: clamp to a sane voxel size (a micron chord tol is far finer than
		// any practical voxel grid).
		watertight_mesh(solid, heal_voxel_size(tol))
	}
}

/// The voxel size the heal fallback runs at for a chord tolerance `tol` (clamped to a
/// sane grid; a micron chord tol is far finer than any practical voxel grid).
pub(crate) fn heal_voxel_size(tol: f64) -> f32 {
	(tol * 20.0).clamp(0.1, 0.5) as f32
}

/// Which path a routed export took (see [`RouteReport`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MeshRoute {
	/// The exact analytic tessellation: triangles lie on the true surfaces to the
	/// chord tolerance, watertight by the shared-edge invariant.
	Exact,
	/// The voxel heal: tessellation → winding-number SDF → Manifold Dual
	/// Contouring. Watertight by construction, accurate to the heal voxel size.
	Healed,
}

/// The routing verdict of [`routed_mesh`] / [`Document::export_mesh`] — *which*
/// mesh an AI was handed and *why*, so the exact-else-heal decision is auditable
/// instead of silent.
#[derive(Clone, Debug)]
pub struct RouteReport {
	/// The path taken.
	pub route: MeshRoute,
	/// Human/AI-readable reason for the routing decision.
	pub why: String,
	/// Triangle count of the returned mesh.
	pub tris: usize,
	/// Whether the returned mesh is watertight (`false` only for an empty result —
	/// both routes otherwise return closed meshes).
	pub watertight: bool,
}

impl RouteReport {
	/// Assemble the report for a finished `mesh`.
	pub(crate) fn for_mesh(mesh: &Mesh, route: MeshRoute, why: impl Into<String>) -> RouteReport {
		RouteReport { route, why: why.into(), tris: mesh.triangle_count(), watertight: mesh.is_watertight() }
	}
}

/// Mesh a B-rep solid at chord tolerance `tol` (mm) through the kernel's **routing
/// policy**, returning the mesh together with the [`RouteReport`] saying which path
/// produced it — the auditable core of [`Document::export_mesh`]:
///
/// 1. a solid whose B-rep **self-intersects** (e.g. an overlapping helical sweep)
///    is healed immediately — its exact tessellation can be edge-watertight yet
///    geometrically corrupt, so it must never ship as "exact";
/// 2. otherwise the **exact** adaptive tessellation is used when it is watertight;
/// 3. otherwise (curved-face cracks the stitcher cannot seal) the **voxel heal**
///    recovers a guaranteed-watertight mesh.
pub fn routed_mesh(solid: &kernel_brep::Solid, tol: f64) -> (Mesh, RouteReport) {
	if kernel_brep::self_intersects(solid) {
		let mesh = watertight_mesh(solid, heal_voxel_size(tol));
		let report = RouteReport::for_mesh(
			&mesh,
			MeshRoute::Healed,
			"exact B-rep self-intersects (its tessellation would be geometrically corrupt); healed via the winding-number voxel field",
		);
		return (mesh, report);
	}
	let exact = kernel_brep::tessellate_adaptive_tol(solid, tol);
	if exact.is_watertight() {
		let report = RouteReport::for_mesh(&exact, MeshRoute::Exact, "exact adaptive tessellation is watertight (analytic surfaces, no voxel grid)");
		(exact, report)
	} else {
		let mesh = watertight_mesh(solid, heal_voxel_size(tol));
		let report = RouteReport::for_mesh(
			&mesh,
			MeshRoute::Healed,
			"exact tessellation is not watertight (curved-face cracks); healed via the winding-number voxel field",
		);
		(mesh, report)
	}
}

/// Heal an arbitrary triangle-soup [`Mesh`] into a watertight, 2-manifold mesh via the
/// voxel half: the soup is lifted into a winding-number signed-distance field
/// ([`kernel_implicit::MeshSdf`]) and re-meshed with Manifold Dual Contouring. Unlike
/// [`watertight_mesh`] (which starts from a single B-rep solid), this **fuses pre-meshed
/// parts** — e.g. a shank and a self-intersecting helical thread whose *exact* B-rep
/// union is not a valid manifold — into one clean closed surface, because the
/// winding-number sign is global (inside/outside of the whole soup), not per-edge.
/// `voxel_size` sets the surface precision (small enough resolves sub-millimetre
/// features like thread crests). Returns an empty mesh for empty input.
pub fn watertight_mesh_of(mesh: &Mesh, voxel_size: f32) -> Mesh {
	if mesh.triangle_count() == 0 {
		return Mesh::new();
	}
	let msdf = kernel_implicit::MeshSdf::new(mesh);
	let bounds = msdf.bounds().pad(voxel_size * 2.0);
	manifold_dual_contour(&msdf, bounds, Resolution::VoxelSize(voxel_size))
}
