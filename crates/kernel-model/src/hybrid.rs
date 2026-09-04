// Copyright (c) LMCAD. Licensed under the MIT License.

//! Mixed-operand booleans (convergent-style): **one public operation** where one
//! operand is an exact B-rep [`Solid`] and the other is a triangle mesh or an
//! implicit field — the BAR.md Level-9 "true convergence" criterion, without the
//! caller hand-building representation twins.
//!
//! # The contract, honestly
//!
//! [`hybrid_boolean`] keeps the **exact side exact wherever the other operand
//! does not touch it**, and states precisely what each part of the result is:
//!
//! - The non-B-rep operand is *realized as a closed triangle mesh* first: a
//!   [`HybridOperand::Mesh`] is used verbatim (its facets ARE its geometry — a
//!   scan's accuracy is the scan's); a [`HybridOperand::Node`] field is meshed
//!   by Manifold Dual Contouring at `voxel` resolution, so the field side
//!   carries **voxel-level accuracy** by construction.
//! - **Exact-stitch route** (`route == `[`HybridRoute::ExactStitch`]): the
//!   operand mesh is wrapped as a polyhedral solid and combined with the B-rep
//!   through the exact mesh-arrangement boolean ([`kernel_brep::booleans`]).
//!   Co-refinement splits only the triangles the two surfaces actually cross,
//!   so every B-rep face the operand does not touch passes through **verbatim**
//!   — bit-identical vertices, analytic [`Surface`](kernel_brep::Surface) tags
//!   (a kept bore wall is still `Surface::Cylinder`) — while straddling faces
//!   are trimmed along an exact, conforming seam at the operand's facet
//!   accuracy, and fully-swallowed faces are dropped. The result is **(a)** a
//!   watertight tessellation and **(b)** the full half-edge [`Solid`] — the
//!   "partial-credit B-rep": untouched exact faces plus seam-band facets, every
//!   face provenance-tagged (`OperandA`/`source_face = input face index` for
//!   the B-rep side — the operand's provenance is re-keyed via
//!   [`Solid::with_primitive_names`] so the tags are collision-free even when
//!   the input brep is itself a boolean result).
//! - **Healed route** (`route == `[`HybridRoute::Healed`]): when the exact
//!   arrangement is *not safe* — the operand mesh is not a closed 2-manifold,
//!   or it is denser than the [`HYBRID_EXACT_MAX_OPERAND_TRIS`] complexity rail
//!   (a time bound, measured: a ~25k-triangle TPMS operand stitches in-suite,
//!   a ~121k-triangle one grinds the arrangement for tens of minutes), or the
//!   stitched result fails validation (the documented failure class of
//!   mesh-arrangement booleans on finely tessellated curved input), or its
//!   tessellation is not watertight — the result is re-meshed through the
//!   implicit twin path ([`kernel_implicit::mesh_boolean_implicit`]) instead:
//!   **always watertight, but voxel-approximate everywhere**, and `solid` is
//!   `None`. The `reason` string says exactly why the exact route was refused.
//!   Partial-but-verified beats a silently wrong Solid.
//! - The per-face accounting in [`HybridReport`] is **measured on the result,
//!   never assumed**: a face counts as `kept_exact` only if a result face is
//!   geometrically *identical* to it (same loops, same vertices, cyclically —
//!   verified to 1e-9 mm); `retiled` vs `trimmed` is decided by comparing the
//!   surviving provenance-tagged area against the input face's area in f64.
//!   On the healed route everything is resampled, so the accounting honestly
//!   reports zero kept faces.
//!
//! The watertight-mesh guarantee is **checked, not promised**: whichever route
//! produced the mesh, `hybrid_boolean` verifies closedness (zero boundary or
//! non-manifold edges) and returns [`HybridError::NotWatertight`] rather than
//! hand back a leaky mesh (Manifold DC's honest guarantee is "closed, never
//! worse than Surface Nets" — the rare pinch case is caught here, remedied with
//! [`kernel_core::make_manifold`] when possible, and refused otherwise).
//!
//! ## Known exact-route ceiling (measured, pinned by tests)
//!
//! A mesh operand's coordinates are f32 by representation. Where its seam
//! chords cross a B-rep face, the arrangement splits that face by each chord's
//! full supporting line (the deliberate over-split design of
//! `kernel_brep::booleans`); a dense ring of near-collinear f32-quantized
//! chords can then produce splinter fragments **thinner than `WELD_EPS`**
//! (measured ≈ 4.8e-7 mm on a 48×24 torus scan crossing a plane), which neither
//! welding (1e-7) nor T-junction healing (4e-7) can absorb — the stitched solid
//! fails validation and the call **routes itself to `Healed`** with that reason.
//! Empirically: scanned boxes and cylinders cross-face exact-stitch fine; a
//! scanned torus exact-stitches when it does not cross a B-rep face (enclosed
//! cavity) and heals when it does — both behaviors are asserted in this
//! module's tests rather than papered over. Widening the weld ladder would
//! shift, not remove, the class (f32 ulp at part scale is ~2e-6 mm) and is a
//! global-tolerance decision owned by `kernel_brep::booleans`, not made
//! silently here.

use std::error::Error;
use std::fmt;

use kernel_core::math::DVec3;
use kernel_core::mesh::Mesh;
use kernel_core::mesher::Resolution;
use kernel_core::sdf::Sdf;
use kernel_implicit::mesh_boolean::BoolOp;
use kernel_implicit::ops::Node;
use kernel_implicit::{manifold_dual_contour, mesh_boolean_implicit};

use kernel_brep::topo::{FaceId, FaceName, FaceSource, Solid};
use kernel_brep::{solid_from_mesh, tessellate, tessellate_default, validate, Surface, TessOptions};

use crate::BooleanOp;

/// Complexity rail of the exact-stitch attempt: an operand mesh with more
/// triangles than this is routed **straight to the heal** with a stated
/// density reason, never into the arrangement. This is a *time* bound, not a
/// correctness bound — the mesh-arrangement boolean's cost grows superlinearly
/// in operand triangles (measured 2026-06-10: a 6.4k-triangle lattice operand
/// stitches in ≈1.4 s, the flagship ~25k-triangle TPMS in-suite, while a
/// ~121k-triangle TPMS ground `classify_select` for 15+ minutes without
/// finishing). Re-mesh a denser field at a coarser `voxel`, or accept the
/// healed route, to stay under it.
pub const HYBRID_EXACT_MAX_OPERAND_TRIS: usize = 50_000;

/// The non-B-rep operand of a [`hybrid_boolean`].
pub enum HybridOperand<'a> {
	/// A triangle mesh body (a scan, an import, a healed print part). Used
	/// verbatim on the exact route — the result's seam accuracy is the mesh's
	/// own facet accuracy. Must be a closed 2-manifold for the exact route;
	/// an open/leaky mesh is still accepted but routes through the voxel heal
	/// (its winding-number inside/outside remains well defined for small gaps).
	Mesh(&'a Mesh),
	/// An implicit CSG field (lattice, TPMS, blend tree). Meshed at `voxel`
	/// resolution before the boolean, so this side carries voxel-level accuracy.
	/// If Manifold DC pinches (its documented junction/saddle case) the snip
	/// remedy (`make_manifold`) is applied at intake and the result re-verified —
	/// a still-unfixed pinch refuses the exact route with its measured edge
	/// count. The field must have **finite bounds** (clamp an unbounded TPMS
	/// with an intersection node first) and respect the 1-Lipschitz contract of
	/// `NUMERICS.md` for its mesh to be sound.
	Node(&'a Node),
}

/// Which representation produced the result mesh — the honest provenance label.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum HybridRoute {
	/// Exact mesh-arrangement: untouched B-rep faces are verbatim (analytic tags
	/// intact), the seam is exact against the operand's facets, and
	/// [`HybridResult::solid`] carries the full stitched B-rep.
	ExactStitch,
	/// The exact route was refused for the stated reason; the mesh was re-meshed
	/// through the implicit field twin — watertight, voxel-approximate
	/// everywhere, no `solid`.
	Healed {
		/// Why the exact route could not be trusted.
		reason: String,
	},
}

/// Per-face accounting of what happened to the exact operand, **measured on the
/// result** (see the [module docs](self)): every input face lands in exactly one
/// of `kept_exact` / `retiled` / `trimmed` / `consumed`. Counts are over the
/// input B-rep's faces; on the healed route the first three are 0 because
/// nothing survives resampling.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct HybridReport {
	/// Faces of the input B-rep.
	pub brep_faces: usize,
	/// Input faces present in the result **verbatim** (geometrically verified:
	/// identical loops and vertices to 1e-9 mm).
	pub kept_exact: usize,
	/// Of `kept_exact`, those carrying a non-planar analytic surface tag
	/// (cylinder/sphere/cone/torus) — proof the exact curved data survived.
	pub kept_exact_curved: usize,
	/// Input faces whose **full area survives** (within 1e-9 relative) but not as
	/// one verbatim face: the operand never touched them, yet face recovery
	/// re-tiled them (e.g. coplanar-adjacent cap faces of an annulus merge into a
	/// two-loop region, which `recover_faces` honestly re-emits as triangles), or
	/// a coplanar union extended them. The *geometry* is still the exact input
	/// surface — only the face subdivision changed.
	pub retiled: usize,
	/// Input faces that survive with **reduced area** — genuinely cut back at the
	/// seam by the other operand.
	pub trimmed: usize,
	/// Input faces entirely absent from the result (swallowed by the operand).
	pub consumed: usize,
	/// Triangle count of the realized operand mesh (the other side's fidelity).
	pub operand_triangles: usize,
}

/// The result of a [`hybrid_boolean`].
#[derive(Debug)]
pub struct HybridResult {
	/// The boolean surface — **verified watertight and 2-manifold** on both routes.
	pub mesh: Mesh,
	/// The stitched half-edge solid (exact route only): untouched exact faces +
	/// provenance-tagged seam-band facets. `None` on the healed route.
	pub solid: Option<Solid>,
	/// Which route produced `mesh`, and why.
	pub route: HybridRoute,
	/// Measured per-face accounting.
	pub report: HybridReport,
}

/// A [`hybrid_boolean`] input/result that cannot be made sound — returned loudly
/// instead of a degraded body.
#[derive(Clone, Debug)]
pub enum HybridError {
	/// A [`HybridOperand::Node`] field has non-finite bounds; clamp it (e.g.
	/// intersect with a box node) before meshing.
	UnboundedField,
	/// Neither route produced a closed 2-manifold mesh; the detail names the
	/// route and the edge counts. Nothing was returned.
	NotWatertight {
		/// What was attempted and how it failed.
		detail: String,
	},
}

impl fmt::Display for HybridError {
	fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
		match self {
			HybridError::UnboundedField => {
				write!(f, "hybrid boolean: the field operand has unbounded extent — intersect it with a finite region node first")
			}
			HybridError::NotWatertight { detail } => {
				write!(f, "hybrid boolean: no route produced a watertight result — {detail}; result withheld")
			}
		}
	}
}

impl Error for HybridError {}

/// Whether two vertex loops are the same closed polygon: equal length and
/// equal vertices under some rotation (orientation preserved), to 1e-9 mm.
fn loops_equal_cyclic(a: &[DVec3], b: &[DVec3]) -> bool {
	if a.len() != b.len() || a.is_empty() {
		return a.len() == b.len();
	}
	let n = a.len();
	'shift: for shift in 0..n {
		for i in 0..n {
			if (a[i] - b[(i + shift) % n]).length() > 1e-9 {
				continue 'shift;
			}
		}
		return true;
	}
	false
}

/// All boundary loops of a face (outer first, then inner), as vertex positions.
fn face_loops(s: &Solid, f: FaceId) -> Vec<Vec<DVec3>> {
	let face = s.face(f);
	std::iter::once(face.outer).chain(face.inner.iter().copied()).map(|lid| s.loop_polygon(lid)).collect()
}

/// Whether result face `rf` is geometrically identical to input face `af`:
/// same loop count, outer loops cyclically equal, and each inner loop matched
/// one-to-one (cyclically) — the verification behind `kept_exact`.
fn faces_identical(input: &Solid, af: FaceId, result: &Solid, rf: FaceId) -> bool {
	let la = face_loops(input, af);
	let lb = face_loops(result, rf);
	if la.len() != lb.len() || !loops_equal_cyclic(&la[0], &lb[0]) {
		return false;
	}
	let mut used = vec![false; lb.len()];
	for inner_a in la.iter().skip(1) {
		let mut found = false;
		for (j, inner_b) in lb.iter().enumerate().skip(1) {
			if !used[j] && loops_equal_cyclic(inner_a, inner_b) {
				used[j] = true;
				found = true;
				break;
			}
		}
		if !found {
			return false;
		}
	}
	true
}

/// Newell area vector of a polygon (length = 2 · area, direction follows winding).
fn newell_area_vec(poly: &[DVec3]) -> DVec3 {
	let mut n = DVec3::ZERO;
	let len = poly.len();
	for i in 0..len {
		let a = poly[i];
		let b = poly[(i + 1) % len];
		n.x += (a.y - b.y) * (a.z + b.z);
		n.y += (a.z - b.z) * (a.x + b.x);
		n.z += (a.x - b.x) * (a.y + b.y);
	}
	n
}

/// Area of a face including hole loops: the outer loop's area minus the inner
/// loops' (inner loops are wound opposite, so the *signed* sum against the
/// outer normal does it).
fn face_area(s: &Solid, f: FaceId) -> f64 {
	let loops = face_loops(s, f);
	let outer = newell_area_vec(&loops[0]);
	let n = outer.normalize_or_zero();
	loops.iter().map(|lp| newell_area_vec(lp).dot(n) * 0.5).sum()
}

/// Measure the per-face accounting of an exact-route result against the input
/// B-rep: result faces are grouped by their (re-keyed, collision-free)
/// `OperandA` provenance, then each input face is classified as kept-exact (a
/// provenance match verified geometrically identical), retiled (full area
/// survives, subdivision changed), trimmed (area reduced at the seam), or
/// consumed (no result face left). Everything is measured — areas in f64 from
/// the actual result polygons, never inferred from the classification.
fn measure_report(input: &Solid, result: &Solid, operand_triangles: usize) -> HybridReport {
	let mut report = HybridReport { brep_faces: input.face_count(), operand_triangles, ..HybridReport::default() };
	for k in 0..input.face_count() as u32 {
		let af = FaceId(k);
		let name = FaceName { operand: FaceSource::OperandA, source_face: k };
		let survivors = result.faces_named(name);
		if survivors.is_empty() {
			report.consumed += 1;
		} else if survivors.iter().any(|&rf| faces_identical(input, af, result, rf)) {
			report.kept_exact += 1;
			if !matches!(input.face(af).surface, Surface::Plane { .. }) {
				report.kept_exact_curved += 1;
			}
		} else {
			let input_area = face_area(input, af);
			let surviving: f64 = survivors.iter().map(|&rf| face_area(result, rf)).sum();
			if surviving >= input_area * (1.0 - 1e-9) - 1e-12 {
				report.retiled += 1;
			} else {
				report.trimmed += 1;
			}
		}
	}
	report
}

/// Closed-2-manifold check with a diagnosable detail string.
fn watertight_or(mesh: &Mesh, what: &str) -> Result<(), String> {
	let nme = mesh.non_manifold_edge_count();
	if mesh.is_empty() || nme == 0 {
		Ok(())
	} else {
		Err(format!("{what}: {nme} boundary/non-manifold edges over {} triangles", mesh.triangle_count()))
	}
}

/// One boolean between an exact B-rep [`Solid`] and a mesh or implicit-field
/// body — see the [module docs](self) for the full honest contract. `op` keeps
/// the B-rep on the left (`brep ∪ other`, `brep − other`, `brep ∩ other`).
///
/// `voxel` (mm) controls the fidelity of everything *resampled*: the meshing of
/// a [`HybridOperand::Node`] field and the healed fallback route. Pass a
/// non-positive value to auto-pick ≈ 1/96 of the relevant bounding diagonal.
/// On the exact route with a [`HybridOperand::Mesh`] operand, `voxel` is unused
/// — accuracy is the operand's own facets and the B-rep's exact faces.
///
/// ```
/// use kernel_brep::cuboid;
/// use kernel_brep::math::DVec3;
/// use kernel_implicit::{Cuboid as VoxCuboid, Node};
/// use kernel_core::math::Vec3;
/// use kernel_model::{hybrid_boolean, BooleanOp, HybridOperand};
/// // Exact 20 mm block ∪ an implicit boss field poking out of its top.
/// let block = cuboid(DVec3::ZERO, DVec3::splat(20.0));
/// let boss = Node::primitive(VoxCuboid::new(Vec3::new(10.0, 10.0, 20.0), Vec3::new(4.0, 4.0, 5.0)));
/// let out = hybrid_boolean(&block, HybridOperand::Node(&boss), BooleanOp::Union, 0.5).unwrap();
/// assert!(out.mesh.is_watertight(), "the hybrid result mesh is always watertight");
/// assert!(out.solid.is_some(), "the exact route also yields the stitched B-rep");
/// assert!(out.report.kept_exact == 5, "the five untouched block faces stay exact (the top is trimmed)");
/// ```
pub fn hybrid_boolean(brep: &Solid, other: HybridOperand<'_>, op: BooleanOp, voxel: f32) -> Result<HybridResult, HybridError> {
	// --- Realize the non-B-rep operand as a triangle mesh. -----------------------
	let op_mesh: Mesh = match other {
		HybridOperand::Mesh(m) => {
			let mut c = m.clone();
			if !c.is_empty() {
				c.ensure_outward();
			}
			c
		}
		HybridOperand::Node(node) => {
			let bounds = node.bounds();
			if !bounds.is_valid() || !bounds.min.is_finite() || !bounds.max.is_finite() {
				return Err(HybridError::UnboundedField);
			}
			let v = if voxel.is_finite() && voxel > 0.0 { voxel } else { (bounds.diagonal() / 96.0).max(1e-6) };
			let mut m = manifold_dual_contour(node, bounds, Resolution::VoxelSize(v));
			if !m.is_empty() && (m.non_manifold_edge_count() != 0 || !m.is_watertight()) {
				// Manifold DC's documented pinch case (junction-rich lattice nodes,
				// TPMS saddles): apply the same snip remedy the healed route already
				// uses. The result is RE-VERIFIED by the manifoldness gate below,
				// never assumed — an unfixed pinch still refuses the exact route
				// with its measured edge count.
				m = kernel_implicit::make_manifold(&m);
			}
			m
		}
	};
	let operand_triangles = op_mesh.triangle_count();

	// --- Trivial algebra (an absent operand never reaches the arrangement). ------
	if brep.face_count() == 0 {
		// ∅ ∪ B = B; ∅ − B = ∅; ∅ ∩ B = ∅.
		let report = HybridReport { operand_triangles, ..HybridReport::default() };
		if !matches!(op, BooleanOp::Union) {
			return Ok(HybridResult { mesh: Mesh::new(), solid: Some(Solid::default()), route: HybridRoute::ExactStitch, report });
		}
		if op_mesh.is_watertight() && op_mesh.non_manifold_edge_count() == 0 {
			let solid = Some(solid_from_mesh(&op_mesh));
			return Ok(HybridResult { mesh: op_mesh, solid, route: HybridRoute::ExactStitch, report });
		}
		// ∅ ∪ open-mesh: nothing exact to keep — heal the operand into a closed body.
		let v = if voxel.is_finite() && voxel > 0.0 { voxel } else { (op_mesh.aabb().diagonal() / 96.0).max(1e-6) };
		let healed = crate::watertight_mesh_of(&op_mesh, v);
		watertight_or(&healed, "empty-brep union: healed operand").map_err(|detail| HybridError::NotWatertight { detail })?;
		let reason = "operand mesh is not a closed 2-manifold (and the B-rep operand is empty)".to_string();
		return Ok(HybridResult { mesh: healed, solid: None, route: HybridRoute::Healed { reason }, report });
	}
	if op_mesh.triangle_count() == 0 {
		// A ∪ ∅ = A − ∅ = A; A ∩ ∅ = ∅.
		let keep_a = !matches!(op, BooleanOp::Intersection);
		let (solid, mesh) = if keep_a { (brep.clone(), tessellate_default(brep)) } else { (Solid::default(), Mesh::new()) };
		watertight_or(&mesh, "operand-less result tessellation").map_err(|detail| HybridError::NotWatertight { detail })?;
		let report = if keep_a {
			// Every face is untouched by a void operand — and verbatim by identity.
			let curved = brep.faces().filter(|&f| !matches!(brep.face(f).surface, Surface::Plane { .. })).count();
			HybridReport {
				brep_faces: brep.face_count(),
				kept_exact: brep.face_count(),
				kept_exact_curved: curved,
				..HybridReport::default()
			}
		} else {
			HybridReport { brep_faces: brep.face_count(), ..HybridReport::default() }
		};
		return Ok(HybridResult { mesh, solid: Some(solid), route: HybridRoute::ExactStitch, report });
	}

	// --- Exact-stitch attempt. ----------------------------------------------------
	// Re-key the brep's provenance to {OperandA, input-face-index}: collision-free
	// even when the brep is itself a boolean result (whose faces would otherwise
	// carry their OWN OperandA/B names into the result and alias the mesh side's).
	let mut refused: Option<String> = None;
	let nme = op_mesh.non_manifold_edge_count();
	if nme != 0 {
		refused = Some(format!("operand mesh is not a closed 2-manifold ({nme} boundary/non-manifold edges)"));
	}
	if refused.is_none() && operand_triangles > HYBRID_EXACT_MAX_OPERAND_TRIS {
		// The complexity rail (see HYBRID_EXACT_MAX_OPERAND_TRIS): beyond it the
		// arrangement's superlinear cost is an effective hang, so the call routes
		// itself to the heal and says so instead of grinding open-endedly.
		refused = Some(format!(
			"operand mesh too dense for the exact arrangement ({operand_triangles} triangles > the {HYBRID_EXACT_MAX_OPERAND_TRIS}-triangle rail); re-mesh coarser or accept the heal"
		));
	}
	if refused.is_none() {
		let a = brep.clone().with_primitive_names();
		let b = solid_from_mesh(&op_mesh);
		let stitched = match op {
			BooleanOp::Union => kernel_brep::union(&a, &b),
			BooleanOp::Difference => kernel_brep::difference(&a, &b),
			BooleanOp::Intersection => kernel_brep::intersection(&a, &b),
		};
		let validity = validate(&stitched);
		if validity.is_valid() {
			// Tessellate with a TIGHT weld: every shared edge of the stitched solid is
			// an exact f64 coincidence (one vertex id), which rounds to identical f32
			// bits — so no geometric welding is needed at all. The default 1e-5 weld
			// (sized for primitive curved control corners) OVER-merges the sub-1e-5
			// seam fragments a mesh-operand arrangement produces, and a single
			// over-merge makes the tessellation non-manifold even though the f64 solid
			// is valid (measured: 2 over-used edges in 24k triangles on the flange ∪
			// gyroid flagship; 0 at 1e-7).
			let mesh = tessellate(&stitched, &TessOptions { curved_subdivisions: 1, weld_tolerance: 1e-7 });
			match watertight_or(&mesh, "exact-stitch tessellation") {
				Ok(()) => {
					let report = measure_report(brep, &stitched, operand_triangles);
					return Ok(HybridResult { mesh, solid: Some(stitched), route: HybridRoute::ExactStitch, report });
				}
				Err(detail) => refused = Some(detail),
			}
		} else {
			refused = Some(format!(
				"exact arrangement failed validation (closed={} manifold={} genus={} shells={})",
				validity.closed, validity.manifold, validity.genus, validity.shells
			));
		}
	}

	// --- Healed fallback: the implicit twin route, watertight by re-meshing. ------
	let reason = refused.unwrap_or_else(|| "unreachable: exact route neither succeeded nor refused".into());
	let bool_op = match op {
		BooleanOp::Union => BoolOp::Union,
		BooleanOp::Difference => BoolOp::Difference,
		BooleanOp::Intersection => BoolOp::Intersection,
	};
	let a_mesh = tessellate_default(brep);
	let mut healed = mesh_boolean_implicit(&a_mesh, &op_mesh, bool_op, voxel as f64);
	if !healed.is_empty() && healed.non_manifold_edge_count() != 0 {
		// Manifold DC's rare pinch case (strut joints / TPMS tangencies grazing the
		// grid): the FULL documented remedy chain, inside the engine so no caller
		// ever needs to know it — (1) snip the pinches apart; (2) if any survive,
		// the pinch sits exactly on this lattice, so re-extract on a slightly
		// shifted grid; (3) snip once more. `watertight_or` below still verdicts
		// honestly if even that fails.
		healed = kernel_implicit::make_manifold(&healed);
		if healed.non_manifold_edge_count() != 0 {
			healed = mesh_boolean_implicit(&a_mesh, &op_mesh, bool_op, voxel as f64 * 1.07);
			if healed.non_manifold_edge_count() != 0 {
				healed = kernel_implicit::make_manifold(&healed);
			}
		}
	}
	watertight_or(&healed, &format!("healed route (after exact refusal: {reason})"))
		.map_err(|detail| HybridError::NotWatertight { detail })?;
	Ok(HybridResult {
		mesh: healed,
		solid: None,
		route: HybridRoute::Healed { reason },
		report: HybridReport { brep_faces: brep.face_count(), operand_triangles, ..HybridReport::default() },
	})
}

#[cfg(test)]
mod tests {
	use super::*;
	use kernel_brep::math::DVec3 as BDVec3;
	use kernel_brep::{cuboid, cylinder, exact_volume, torus};
	use kernel_core::math::Vec3;
	use kernel_implicit::{Cuboid as VoxCuboid, Gyroid};

	/// The "scan" stand-in: a tessellated + welded torus mesh (closed 2-manifold).
	fn scanned_torus(center: BDVec3, major: f64, minor: f64) -> Mesh {
		let mut m = tessellate_default(&torus(center, BDVec3::Z, major, minor, 48, 24));
		m.weld(1e-5);
		assert!(m.is_watertight(), "the scan fixture must be a closed mesh");
		m
	}

	/// A flange that is ITSELF a boolean result (provenance-carrying operand):
	/// Ø36 disc, Ø10 through-bore — 36-segment exact cylinder walls.
	fn flange() -> Solid {
		let disc = cylinder(BDVec3::new(0.0, 0.0, 0.0), BDVec3::Z, 18.0, 6.0, 36);
		let bore = cylinder(BDVec3::new(0.0, 0.0, -1.0), BDVec3::Z, 5.0, 8.0, 36);
		kernel_brep::difference(&disc, &bore)
	}

	#[test]
	fn block_minus_scanned_cylinder_keeps_untouched_faces_exact() {
		// A 40×40×20 block minus a scanned cylinder poking through its top: the
		// five faces the scan never touches must appear in the result VERBATIM,
		// the top face is genuinely trimmed (a bore rim), and the mesh is
		// watertight with the seam exactly on the scan's facets.
		let block = cuboid(BDVec3::ZERO, BDVec3::new(40.0, 40.0, 20.0));
		let mut scan = tessellate_default(&cylinder(BDVec3::new(20.0, 20.0, 15.0), BDVec3::Z, 5.0, 10.0, 24));
		scan.weld(1e-5);
		assert!(scan.is_watertight(), "the scan fixture must be a closed mesh");
		let out = hybrid_boolean(&block, HybridOperand::Mesh(&scan), BooleanOp::Difference, 0.0)
			.expect("hybrid difference must produce a result");

		// Twin-route reference: both sides meshed, boolean in the implicit domain.
		let twin = mesh_boolean_implicit(&tessellate_default(&block), &scan, BoolOp::Difference, 0.5);
		let (v, vt) = (out.mesh.signed_volume(), twin.signed_volume());

		let solid = out.solid.as_ref().expect("exact route must yield the partial-credit solid");
		let validity = validate(solid);
		assert!(
			out.route == HybridRoute::ExactStitch
				&& out.report.brep_faces == 6
				&& out.report.kept_exact == 5 // 4 sides + bottom, verbatim
				&& out.report.retiled == 0
				&& out.report.trimmed == 1 // the top face, now carrying the bore rim
				&& out.report.consumed == 0
				&& out.report.operand_triangles == scan.triangle_count()
				&& validity.is_valid()
				&& out.mesh.is_watertight()
				&& out.mesh.non_manifold_edge_count() == 0
				&& ((v - vt) / vt).abs() <= 0.01,
			"block − scan must keep 5 faces exact and match the twin route ≤1%: route={:?} report={:?} validity={validity:?} v={v} twin={vt} (rel {:+.4}%)",
			out.route,
			out.report,
			100.0 * (v - vt) / vt
		);
	}

	#[test]
	fn poking_torus_scan_routes_healed_and_stays_watertight() {
		// The module-doc "known exact-route ceiling", pinned: a scanned TORUS
		// crossing a B-rep face produces sub-WELD_EPS splinter fragments in the
		// over-split arrangement (measured ≈ 4.8e-7 mm), the stitched solid fails
		// validation, and the hybrid must (a) refuse the exact route with that
		// reason, (b) still deliver a watertight mesh via the implicit twin, and
		// (c) stay within 1% of the twin-route volume (it IS the twin route here).
		let block = cuboid(BDVec3::ZERO, BDVec3::new(40.0, 40.0, 20.0));
		let scan = scanned_torus(BDVec3::new(20.0, 20.0, 19.5), 8.0, 3.0); // tube top 22.5 > 20
		let out = hybrid_boolean(&block, HybridOperand::Mesh(&scan), BooleanOp::Difference, 0.4).expect("hybrid must heal, not fail");
		let twin = mesh_boolean_implicit(&tessellate_default(&block), &scan, BoolOp::Difference, 0.4);
		let (v, vt) = (out.mesh.signed_volume(), twin.signed_volume());
		assert!(
			matches!(&out.route, HybridRoute::Healed { reason } if reason.contains("failed validation"))
				&& out.solid.is_none()
				&& out.report.kept_exact == 0
				&& out.mesh.is_watertight()
				&& out.mesh.non_manifold_edge_count() == 0
				&& ((v - vt) / vt).abs() <= 0.01,
			"a poking torus scan must route Healed with the validation reason and stay watertight: route={:?} report={:?} v={v} twin={vt}",
			out.route,
			out.report
		);
	}

	#[test]
	fn fully_enclosed_scan_cavity_is_inclusion_exclusion_exact() {
		// The scan fully inside the block ⇒ a closed cavity: ALL six block faces
		// stay verbatim, the result solid grows a second shell, and — because every
		// facet of both operands passes through the arrangement unchanged — the
		// result volume equals block − scan to f64 round-off (no voxel error).
		let block = cuboid(BDVec3::ZERO, BDVec3::splat(40.0));
		let scan = scanned_torus(BDVec3::new(20.0, 20.0, 20.0), 8.0, 3.0);
		let out = hybrid_boolean(&block, HybridOperand::Mesh(&scan), BooleanOp::Difference, 0.0)
			.expect("hybrid cavity difference must produce a result");
		let solid = out.solid.as_ref().expect("exact route must yield the solid");
		let validity = validate(solid);
		let v = exact_volume(solid);
		let expect = 40.0f64.powi(3) - scan.signed_volume();
		assert!(
			out.route == HybridRoute::ExactStitch
				&& out.report.kept_exact == 6
				&& out.report.retiled == 0
				&& out.report.trimmed == 0
				&& out.report.consumed == 0
				&& validity.is_valid()
				&& validity.shells == 2
				&& out.mesh.is_watertight()
				&& out.mesh.non_manifold_edge_count() == 0
				&& ((v - expect) / expect).abs() < 1e-9,
			"an enclosed scan is a cavity with ALL block faces verbatim and an exact volume: route={:?} report={:?} validity={validity:?} v={v} expect={expect} (rel {:+e})",
			out.route,
			out.report,
			(v - expect) / expect
		);
	}

	#[test]
	fn flange_union_gyroid_node_keeps_uncut_curved_walls_exact() {
		// The convergent flagship: an exact flange (ITSELF a boolean result — the
		// provenance re-key must hold) ∪ a gyroid TPMS field clamped to a block
		// that straddles the flange's top. The flange's outer wall and everything
		// below the block must stay verbatim — including curved facets with their
		// exact Surface::Cylinder tags — while the field side enters at voxel
		// accuracy.
		let f = flange();
		// Gyroid sheet network clamped to a 12 mm block overlapping the flange top
		// (z ∈ [3, 15] vs flange z ∈ [0, 6]); the block (|x|,|y| ≤ 6) crosses the
		// Ø10 bore wall transversally and stays clear of the Ø36 outer wall.
		let region = kernel_core::math::Aabb { min: Vec3::new(-8.0, -8.0, 1.0), max: Vec3::new(8.0, 8.0, 17.0) };
		let node = Node::primitive(Gyroid::new(region, 0.8, 0.45))
			.intersection(Node::primitive(VoxCuboid::new(Vec3::new(0.0, 0.0, 9.0), Vec3::splat(6.0))));
		let out = hybrid_boolean(&f, HybridOperand::Node(&node), BooleanOp::Union, 0.4).expect("hybrid union must produce a result");

		// Twin-route reference at the same voxel.
		let gyroid_mesh = manifold_dual_contour(&node, node.bounds(), Resolution::VoxelSize(0.4));
		let twin = mesh_boolean_implicit(&tessellate_default(&f), &gyroid_mesh, BoolOp::Union, 0.4);
		let (v, vt) = (out.mesh.signed_volume(), twin.signed_volume());

		let r = out.report;
		let solid = out.solid.as_ref().expect("the flagship union must exact-stitch (deterministic fixture)");
		let validity = validate(solid);
		// Geometry-derived floors (robust to a libm-induced gyroid-mesh wiggle, cf.
		// NUMERICS.md cross-platform note): the Ø36 outer wall — 36 curved facets at
		// radius 18, untouched by the block (corner radius √72 ≈ 8.49) — must survive
		// VERBATIM with its analytic Surface::Cylinder tag; only curved faces can be
		// verbatim here (the coplanar cap annuli re-tile, see HybridReport::retiled);
		// the caps overlapping the block footprint are genuinely trimmed; nothing is
		// consumed (the gyroid adds material in a union); and the four buckets must
		// partition the flange's faces. Measured on this platform: kept_exact = 41
		// (36 outer wall + 5 uncrossed bore facets), retiled = 28, trimmed = 31.
		assert!(
			out.route == HybridRoute::ExactStitch
				&& out.mesh.is_watertight()
				&& out.mesh.non_manifold_edge_count() == 0
				&& r.operand_triangles == gyroid_mesh.triangle_count()
				&& ((v - vt) / vt).abs() <= 0.01
				&& validity.is_valid()
				&& validity.genus > 0 // the gyroid's handles fused onto the flange
				&& r.kept_exact >= 36
				&& r.kept_exact_curved == r.kept_exact
				&& r.retiled >= 8
				&& r.trimmed >= 1
				&& r.consumed == 0
				&& r.kept_exact + r.retiled + r.trimmed + r.consumed == r.brep_faces,
			"flange ∪ gyroid must exact-stitch, keep the outer wall verbatim and match the twin ≤1%: route={:?} report={r:?} validity={validity:?} v={v} twin={vt} (rel {:+.4}%)",
			out.route,
			100.0 * (v - vt) / vt
		);
	}

	#[test]
	fn open_scan_routes_through_the_heal_and_stays_watertight() {
		// An OPEN scan (a real-world partial torus scan: 40 triangles deleted) has
		// no well-defined exact arrangement — the hybrid must refuse the exact
		// route, say why, and still deliver a watertight mesh via the winding-number
		// heal, with the volume close to the closed-scan exact result.
		let block = cuboid(BDVec3::ZERO, BDVec3::splat(40.0));
		let closed = scanned_torus(BDVec3::new(20.0, 20.0, 20.0), 8.0, 3.0);
		let mut open = closed.clone();
		open.indices.truncate(open.indices.len() - 40 * 3);
		assert!(!open.is_watertight(), "fixture must be an open scan");

		let out = hybrid_boolean(&block, HybridOperand::Mesh(&open), BooleanOp::Difference, 0.4)
			.expect("hybrid must heal an open scan, not fail");
		let exact = hybrid_boolean(&block, HybridOperand::Mesh(&closed), BooleanOp::Difference, 0.4).expect("closed-scan reference");
		let (v, ve) = (out.mesh.signed_volume(), exact.mesh.signed_volume());
		assert!(
			matches!(&out.route, HybridRoute::Healed { reason } if reason.contains("not a closed 2-manifold"))
				&& out.solid.is_none()
				&& out.report.kept_exact == 0
				&& out.mesh.is_watertight()
				&& out.mesh.non_manifold_edge_count() == 0
				&& ((v - ve) / ve).abs() < 0.02, // winding sign bridges the 40-tri hole
			"open scan must route Healed with a reason and stay watertight: route={:?} v={v} exact={ve} (rel {:+.4}%)",
			out.route,
			100.0 * (v - ve) / ve
		);
	}

	#[test]
	fn empty_operands_follow_the_boolean_algebra() {
		// A ∪ ∅ = A − ∅ = A (every face kept verbatim, by identity); A ∩ ∅ = ∅;
		// ∅ ∪ B = B (operand verbatim, wrapped as a solid); ∅ − B = ∅ ∩ B = ∅.
		let plate = cuboid(BDVec3::ZERO, BDVec3::splat(10.0));
		let empty_mesh = Mesh::new();
		let keep = hybrid_boolean(&plate, HybridOperand::Mesh(&empty_mesh), BooleanOp::Difference, 0.0).unwrap();
		let gone = hybrid_boolean(&plate, HybridOperand::Mesh(&empty_mesh), BooleanOp::Intersection, 0.0).unwrap();
		let mut scan = tessellate_default(&cylinder(BDVec3::new(0.0, 0.0, 0.0), BDVec3::Z, 4.0, 8.0, 16));
		scan.weld(1e-5);
		let adopt = hybrid_boolean(&Solid::default(), HybridOperand::Mesh(&scan), BooleanOp::Union, 0.0).unwrap();
		assert!(
			keep.report.kept_exact == 6
				&& keep.report.kept_exact + keep.report.retiled + keep.report.trimmed + keep.report.consumed == 6
				&& keep.solid.is_some()
				&& keep.mesh.is_watertight()
				&& gone.mesh.is_empty()
				&& gone.solid.is_some()
				&& adopt.route == HybridRoute::ExactStitch
				&& adopt.mesh.triangle_count() == scan.triangle_count()
				&& adopt.solid.is_some(),
			"empty-operand algebra: keep={:?} gone={:?} adopt={:?}",
			keep.report,
			gone.report,
			adopt.report
		);
	}

	#[test]
	fn hybrid_boolean_is_deterministic() {
		// Same inputs twice ⇒ identical route, report, and result volume BITS
		// (the arrangement, MDC and the measured report must all be deterministic).
		let block = cuboid(BDVec3::ZERO, BDVec3::new(40.0, 40.0, 20.0));
		let mut scan = tessellate_default(&cylinder(BDVec3::new(20.0, 20.0, 15.0), BDVec3::Z, 5.0, 10.0, 24));
		scan.weld(1e-5);
		let a = hybrid_boolean(&block, HybridOperand::Mesh(&scan), BooleanOp::Difference, 0.0).unwrap();
		let b = hybrid_boolean(&block, HybridOperand::Mesh(&scan), BooleanOp::Difference, 0.0).unwrap();
		assert!(
			a.route == b.route
				&& a.report == b.report
				&& a.mesh.signed_volume().to_bits() == b.mesh.signed_volume().to_bits()
				&& a.mesh.triangle_count() == b.mesh.triangle_count(),
			"hybrid_boolean must be deterministic: {:?}/{:?} vs {:?}/{:?}",
			a.route,
			a.report,
			b.route,
			b.report
		);
	}
}
