// Copyright (c) LMCAD. Licensed under the MIT License.

//! **Reverse bridge** (implicit → B-rep → STEP) and field interrogation.
//!
//! Implicit results are meshes: [`kernel_implicit::manifold_dual_contour`] and
//! the narrow-band extractors hand back watertight triangle meshes, and until
//! now nothing could carry such a result back into a [`kernel_brep::Solid`] for
//! STEP export or the exact boolean pipeline. This module reopens that one-way
//! street — plus [`thin_wall_report`], a sampled manufacturability interrogation
//! of the field *before* committing it to a B-rep.
//!
//! # V1 CONTRACT — stated bluntly
//!
//! The output of [`mesh_to_solid`] / [`implicit_to_solid`] is a **FACETED
//! B-rep**: one planar face per mesh triangle, with groups of adjacent,
//! exactly-coplanar facets merged into multi-loop planar faces by
//! [`kernel_brep::coalesce_coplanar`]. There is **no analytic curved-surface
//! recovery on this path** — a voxelized cylinder comes back as N
//! `Surface::Plane` facets, never a `Surface::Cylinder`. Geometric accuracy is
//! therefore exactly the mesher's chord accuracy at the chosen voxel size; the
//! bridge itself adds nothing beyond the weld epsilon. What v1 buys, honestly:
//! an implicit result (TPMS block, smooth blend, lattice) becomes a real
//! validated `Solid`, so it can be **exported to STEP** and used as an operand
//! in the **exact planar booleans** — the two things a bare mesh cannot do.
//! Failures are loud (`Err` with the concrete watertightness / validity
//! counts), never a silently degraded solid.
//!
//! # V2 CONTRACT — analytic quadric recovery
//!
//! [`mesh_to_solid_recovered`] / [`implicit_to_solid_recovered`] run the SAME
//! v1 pipeline, then [`kernel_brep::recover::recover_quadrics`] as a finishing
//! pass: connected facet regions that fit one cylinder / sphere / cone / torus
//! (or one tolerant plane — the mesher-cap case) within the caller's `tol` get
//! that analytic [`kernel_brep::Surface`] carrier. No vertex is moved —
//! recovery changes the surface carrier, not the point set — and the fit
//! residual is REPORTED in the returned [`RecoveryReport`], never hidden.
//! Single-curved regions (cylinder, cone) merge into span-budgeted sector
//! faces, so the face count collapses and STEP exports carry
//! `CYLINDRICAL_SURFACE`/`CONICAL_SURFACE` geometry instead of thousands of
//! facet planes; doubly-curved regions (sphere, torus) are retagged
//! facet-by-facet (carrier recovery WITHOUT face-count collapse — their merge
//! is fidelity-limited by the boundary-only tessellators; see the `recover`
//! module doc). Refusals stay loud: an invalid rebuild or a
//! default-tessellation volume drift over 0.5% is an `Err` carrying the
//! measured numbers, and a solid with nothing recoverable passes through
//! structurally unchanged with a zeroed report. The v1 entry points are
//! untouched in behavior.

use kernel_brep::recover::recover_quadrics;
use kernel_brep::{coalesce_coplanar, solid_from_mesh, validate, volume, Solid};
use kernel_core::math::{Aabb, Vec3};
use kernel_core::mesh::Mesh;
use kernel_core::mesher::Resolution;
use kernel_core::sdf::Sdf;
use kernel_implicit::manifold_dual_contour;

pub use kernel_brep::recover::RecoveryReport;

/// Weld tolerance (mm) applied to a non-watertight input before wrapping — the
/// crate's scan-import tolerance (see `hybrid.rs`). Mesher output is already
/// indexed and welded, so this path only runs for foreign/soup inputs.
const BRIDGE_WELD: f32 = 1e-5;

/// Wrap a closed triangle mesh as a validated **faceted** B-rep [`Solid`]
/// (see the module doc for the v1 contract: planar facets only, no analytic
/// surface recovery).
///
/// Pipeline: weld at [`BRIDGE_WELD`] if the mesh is not already watertight →
/// [`kernel_brep::solid_from_mesh`] (each triangle a planar face, winding
/// normalised outward) → [`kernel_brep::coalesce_coplanar`] (adjacent coplanar
/// facets merge into multi-loop planar faces — a tessellated box becomes 6
/// faces again) → [`kernel_brep::validate`]. Volume is gated against the input
/// mesh (relative 1e-6), so a coalesce or stitch that changed geometry is a
/// refusal, not a quiet corruption.
///
/// Errors carry the concrete counts: non-manifold/boundary edge counts before
/// and after the weld for a leaky input, the full [`kernel_brep::Validity`]
/// breakdown for a wrap that fails validation, and the measured volumes for a
/// conservation failure.
///
/// Example: `mesh_to_solid(&kernel_brep::tessellate_default(&cuboid))` → a
/// 6-face box solid.
pub fn mesh_to_solid(mesh: &Mesh) -> Result<Solid, String> {
	if mesh.triangle_count() == 0 {
		return Err("mesh_to_solid: input mesh is empty (0 triangles)".to_string());
	}
	// Weld only when needed: mesher output is already indexed/watertight and a
	// redundant weld would only risk collapsing legitimate short edges.
	let defects_before = mesh.non_manifold_edge_count();
	let welded;
	let closed: &Mesh = if defects_before == 0 {
		mesh
	} else {
		let mut m = mesh.clone();
		m.weld(BRIDGE_WELD);
		let defects_after = m.non_manifold_edge_count();
		if defects_after != 0 {
			return Err(format!(
				"mesh_to_solid: mesh is not watertight even after weld({BRIDGE_WELD}): {defects_after} non-manifold/boundary edges remain (was {defects_before} before weld, {} triangles) — repair the mesh (weld/fill_holes/make_manifold) or re-mesh before bridging",
				mesh.triangle_count()
			));
		}
		welded = m;
		&welded
	};
	let wrapped = solid_from_mesh(closed);
	let solid = coalesce_coplanar(&wrapped);
	let v = validate(&solid);
	if !v.is_valid() {
		return Err(format!(
			"mesh_to_solid: wrapped solid failed validation: closed={} manifold={} genus={} shells={} χ={} ({} faces from {} triangles) — the input mesh is edge-closed but not a clean 2-manifold (pinched vertex / non-orientable patch)",
			v.closed,
			v.manifold,
			v.genus,
			v.shells,
			v.euler_characteristic,
			solid.face_count(),
			closed.triangle_count()
		));
	}
	// Conservation gate (coalesce.rs contract: "callers should gate both").
	let v_mesh = closed.signed_volume().abs();
	let v_solid = volume(&solid);
	if (v_solid - v_mesh).abs() > 1e-6 * v_mesh.max(1.0) {
		return Err(format!(
			"mesh_to_solid: volume not conserved through wrap+coalesce: mesh {v_mesh:.9} mm³ vs solid {v_solid:.9} mm³ — refusing to hand back silently altered geometry"
		));
	}
	Ok(solid)
}

/// Extract an implicit field to a validated **faceted** B-rep [`Solid`] — the
/// reverse bridge's front door (see the module doc for the v1 contract).
///
/// Meshing uses the SAME plumbing as the hybrid heal (`watertight_mesh` /
/// `watertight_mesh_of` in this crate): [`kernel_implicit::manifold_dual_contour`]
/// at `Resolution::VoxelSize(voxel)` over `bounds` padded by two voxels, so a
/// surface touching the given box still closes cleanly. The mesh then goes
/// through [`mesh_to_solid`]. The field must be closed inside `bounds` (clip an
/// open TPMS/labyrinth with a solid first — same doctrine as `hybrid_boolean`);
/// a field with no surface in `bounds` is an `Err`, not an empty solid.
///
/// `voxel` is the fidelity/size trade: every voxel of surface becomes ~2 planar
/// faces of the result, so a fine voxel on a large part produces a very large
/// faceted solid (and STEP file). That is inherent to the v1 faceted contract.
///
/// Example: `implicit_to_solid(&node, node.bounds(), 0.8)?` → STEP-exportable solid.
pub fn implicit_to_solid<S: Sdf + ?Sized>(sdf: &S, bounds: Aabb, voxel: f32) -> Result<Solid, String> {
	if !(voxel.is_finite() && voxel > 0.0) {
		return Err(format!("implicit_to_solid: voxel size must be positive and finite, got {voxel}"));
	}
	if !(bounds.min.is_finite() && bounds.max.is_finite()) || bounds.size().min_element() <= 0.0 {
		return Err(format!("implicit_to_solid: bounds must be finite and non-degenerate, got {bounds:?}"));
	}
	let domain = bounds.pad(voxel * 2.0);
	let mesh = manifold_dual_contour(sdf, domain, Resolution::VoxelSize(voxel));
	if mesh.triangle_count() == 0 {
		return Err(format!(
			"implicit_to_solid: the field has no surface inside {bounds:?} at voxel {voxel} (or the lattice exceeded the mesher's cell cap) — nothing to bridge"
		));
	}
	mesh_to_solid(&mesh)
}

/// Reverse bridge **v2**: [`mesh_to_solid`] followed by analytic quadric
/// recovery ([`kernel_brep::recover::recover_quadrics`]) — see the module
/// doc's V2 CONTRACT. `tol` (mm) is the fit acceptance band: every recovered
/// region's vertices, facet edge midpoints and centroids lie within `tol` of
/// the fitted surface, and the achieved residual comes back in the
/// [`RecoveryReport`]. Vertices are never moved; a mesh with nothing
/// recoverable returns the v1 solid unchanged with a zeroed report.
///
/// Example: `mesh_to_solid_recovered(&scan, 0.05)?` → `(solid, report)` with
/// `report.cylinders` counting the recovered bores/bosses.
pub fn mesh_to_solid_recovered(mesh: &Mesh, tol: f64) -> Result<(Solid, RecoveryReport), String> {
	let faceted = mesh_to_solid(mesh)?;
	recover_quadrics(&faceted, tol)
}

/// Reverse bridge **v2**: [`implicit_to_solid`] followed by analytic quadric
/// recovery — see the module doc's V2 CONTRACT. The v1 extraction (same
/// mesher, same weld, same conservation gates) runs first; then
/// [`kernel_brep::recover::recover_quadrics`] fits cylinder / sphere / cone /
/// torus carriers onto facet regions within `tol` (mm), collapsing
/// single-curved regions into sector faces and retagging doubly-curved ones.
/// Refusals are loud at both stages (`Err` with measured counts); the
/// residual and face counts are reported, not hidden.
///
/// Example: `implicit_to_solid_recovered(&node, node.bounds(), 0.4, 0.05)?` →
/// a STEP-exportable solid whose voxelized cylinder walls carry exact
/// `Surface::Cylinder` tags again.
pub fn implicit_to_solid_recovered<S: Sdf + ?Sized>(
	sdf: &S,
	bounds: Aabb,
	voxel: f32,
	tol: f64,
) -> Result<(Solid, RecoveryReport), String> {
	let faceted = implicit_to_solid(sdf, bounds, voxel)?;
	recover_quadrics(&faceted, tol)
}

/// The result of [`thin_wall_report`] — a SAMPLED thin-wall census of an
/// implicit part. All figures are estimates at the sampling resolution; see
/// the function doc for exactly what is (and is not) measured.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ThinWallReport {
	/// Smallest local wall-thickness estimate found (mm), `+∞` when no interior
	/// medial sample was found (e.g. an empty field or too-coarse sampling).
	pub thinnest: f32,
	/// Number of accepted (medial) samples whose thickness estimate is below
	/// `t_min` — a **sample census**, not a defect count (one thin wall yields
	/// many samples).
	pub below_count: usize,
	/// Position of the thinnest sample (`Vec3::ZERO` when none was found).
	pub at: Vec3,
}

/// Lattice-sampled **thin-wall estimate** of an implicit part — field
/// interrogation before committing to a mesh or the reverse bridge.
///
/// What is measured, exactly: `samples_per_axis`³ lattice points over `bounds`;
/// at each interior point (`d < 0`) whose `|d|` is a **local maximum along the
/// ±gradient direction** (probe step = one lattice cell, slack 5% of a cell) —
/// i.e. a medial-surface-ish point, where the nearest-surface distance stops
/// growing — the local wall thickness is estimated as `2·|d|`, exact for a
/// parallel-sided wall of a true distance field. `thinnest` is the minimum
/// estimate, `below_count` counts medial samples under `t_min`, `at` locates
/// the thinnest one.
///
/// **A SAMPLED ESTIMATE, not an oracle** (the same honesty contract as
/// `penetration_estimate`'s vertex sampling): it can **under-report** by up to
/// ~one lattice cell (an accepted sample sits up to half a cell off the true
/// mid-surface — conservative for a minimum-wall warning), it can **miss** a
/// wall thinner than the lattice spacing entirely (no sample lands inside it),
/// and a CSG bound that is not a true distance field (smooth blends, offsets of
/// booleans) makes `2·|d|` a lower bound rather than the exact thickness. Cost
/// is O(samples_per_axis³) field evaluations plus ~8 more per interior point.
/// Use it to warn; gate a final claim on finer sampling.
///
/// Example: `thin_wall_report(&hollow, hollow.bounds(), 96, 1.2)` →
/// `thinnest ≈` the thinnest wall, `below_count > 0` iff anything is under 1.2 mm.
pub fn thin_wall_report<S: Sdf + ?Sized>(sdf: &S, bounds: Aabb, samples_per_axis: usize, t_min: f32) -> ThinWallReport {
	let n = samples_per_axis.max(2);
	let size = bounds.size();
	let step = size / (n as f32 - 1.0);
	// Probe step: the coarsest axis spacing, so the medial acceptance band is at
	// least one lattice cell wide along every axis (no wall column is skipped).
	let h = step.max_element();
	let slack = 0.05 * h;
	let mut report = ThinWallReport { thinnest: f32::INFINITY, below_count: 0, at: Vec3::ZERO };
	for i in 0..n {
		for j in 0..n {
			for k in 0..n {
				let p = bounds.min + Vec3::new(step.x * i as f32, step.y * j as f32, step.z * k as f32);
				let d0 = sdf.distance(p);
				if d0 >= 0.0 {
					continue; // not interior material
				}
				// Medial test: |d| must not grow when stepping along ±gradient.
				// (Toward the surface it always shrinks; deeper into a wall it grows
				// until the mid-surface — so acceptance ⇔ within ~half a cell of it.
				// Probes that exit the material, d > 0, pass trivially: a wall
				// thinner than two cells is still caught, never medial-rejected.)
				let g = sdf.gradient(p);
				let d_out = sdf.distance(p + g * h);
				let d_in = sdf.distance(p - g * h);
				if d0 <= d_out + slack && d0 <= d_in + slack {
					let thickness = -2.0 * d0;
					if thickness < t_min {
						report.below_count += 1;
					}
					if thickness < report.thinnest {
						report.thinnest = thickness;
						report.at = p;
					}
				}
			}
		}
	}
	report
}
