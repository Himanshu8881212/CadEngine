// Copyright (c) LMCAD. Licensed under the MIT License.

//! Checked, `Result`-typed booleans — the **AI-facing guardrail API** (BAR.md, I2).
//!
//! The raw [`union`] / [`difference`] / [`intersection`] always hand back a
//! [`Solid`], even when the arrangement degrades (an open input, a pathological
//! chain): the caller is expected to run [`validate`] before trusting the result.
//! A human in a REPL does that; an autonomous caller — an AI driving the kernel,
//! a script replaying a feature program — reliably does not. These wrappers make
//! the contract structural instead of cultural: each runs the *identical* boolean
//! (no algorithmic difference, byte-for-byte the same result on success), then
//! validates, and **refuses to return** a solid that is not closed, not manifold,
//! or of negative genus. Through this API an invalid B-rep can NEVER propagate
//! silently — failure arrives as a machine-readable [`BooleanError`] naming the
//! op and the exact broken invariants, so the caller can re-route (e.g. through
//! the voxel heal) rather than chain features onto corrupt topology.

use std::error::Error;
use std::fmt;

use crate::booleans::{difference, intersection, union};
use crate::curved_boolean::Keep;
use crate::freeform::{freeform_plane_cut, FreeformBoolError, FreeformCut, FreeformCutOptions, FreeformSolid};
use crate::hazards::{boolean_hazards, Hazard, HazardKind};
use crate::mesh_boolean::MeshBoolOp;
use crate::topo::Solid;
use crate::validate::{validate, Validity};

/// A checked boolean produced a topologically invalid solid, which was withheld.
///
/// Both fields are machine-readable: `op` identifies the operation and
/// [`Validity`] carries the full report ([`closed`](Validity::closed) /
/// [`manifold`](Validity::manifold) / genus / shells), so a caller can branch on
/// the exact failure instead of parsing a message. [`fmt::Display`] renders the
/// same information as one informative line.
#[derive(Clone, Copy, Debug)]
pub struct BooleanError {
	/// Which boolean failed: `"union"`, `"difference"` or `"intersection"`.
	pub op: &'static str,
	/// The validity report of the rejected result.
	pub validity: Validity,
}

impl fmt::Display for BooleanError {
	fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
		let v = self.validity;
		let mut reasons: Vec<&str> = Vec::new();
		if !v.closed {
			reasons.push("not closed (unpaired boundary half-edges)");
		}
		if !v.manifold {
			reasons.push("non-manifold (edge/loop/vertex invariants broken)");
		}
		if v.genus < 0 {
			reasons.push("negative genus (pinched or self-touching arrangement)");
		}
		write!(
			f,
			"boolean {} produced an invalid solid — {} [closed={} manifold={} genus={} shells={}]; result withheld",
			self.op,
			reasons.join(", "),
			v.closed,
			v.manifold,
			v.genus,
			v.shells
		)
	}
}

impl Error for BooleanError {}

/// Validate a boolean result, withholding it unless every invariant holds.
fn checked(result: Solid, op: &'static str) -> Result<Solid, BooleanError> {
	let validity = validate(&result);
	if validity.is_valid() {
		Ok(result)
	} else {
		Err(BooleanError { op, validity })
	}
}

/// Hazard band (model units) for the failure-path pre-flight scan of
/// [`BooleanError::with_preflight`] / the `try_*_diagnosed` family — the §7.7
/// authoring default: real design clearances live above 0.05, slivers below.
const REFUSAL_HAZARD_TOL: f64 = 0.05;

/// A refused boolean **with its pre-flight diagnosis attached**: the same
/// machine-readable [`BooleanError`] the strict `try_*` family returns, plus
/// the [`Hazard`] (if any) that [`boolean_hazards`] finds between the operands
/// — run *on refusal only*, so the success path stays byte-identical and free.
///
/// `hazard` is a *hypothesis*, not a proof: the linter's best explanation of
/// the refusal — the worst (smallest-gap) [`HazardKind::TangentPlaneOnCylinder`]
/// if one fired (the keyed-pulley kiss class, always implicated when present),
/// else the linter's worst hazard of any kind, else `None` (the refusal is not
/// one of the linted input patterns). Match on `error.op` / `error.validity`
/// exactly as with [`BooleanError`]; the hint is the documented `Display`
/// suffix `" — pre-flight linter implicates: …"` carrying the hazard and its
/// [`HazardKind::remedy`].
#[derive(Clone, Copy, Debug)]
pub struct BooleanRefusal {
	/// The strict refusal, unchanged (op + full validity report).
	pub error: BooleanError,
	/// The pre-flight hazard most likely implicated — see the type docs.
	pub hazard: Option<Hazard>,
}

impl fmt::Display for BooleanRefusal {
	fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
		self.error.fmt(f)?;
		if let Some(h) = &self.hazard {
			write!(f, " — pre-flight linter implicates: {h}")?;
		}
		Ok(())
	}
}

impl Error for BooleanRefusal {}

impl BooleanError {
	/// Enrich this refusal with a pre-flight diagnosis of its operands: run
	/// [`boolean_hazards`] (band 0.05, the §7.7 authoring default) and attach
	/// the hazard most likely implicated (see [`BooleanRefusal::hazard`] for
	/// the selection rule). Works on any [`BooleanError`] — from `try_*`,
	/// [`crate::boolean_with_policy`] or [`crate::boolean_tolerant`] — as long
	/// as you still hold the operands.
	pub fn with_preflight(self, a: &Solid, b: &Solid) -> BooleanRefusal {
		let report = boolean_hazards(a, b, REFUSAL_HAZARD_TOL);
		let hazard = report.iter().find(|h| h.kind == HazardKind::TangentPlaneOnCylinder).or_else(|| report.first()).copied();
		BooleanRefusal { error: self, hazard }
	}
}

/// [`try_union`] with the failure-path diagnosis: identical boolean and
/// identical result on success; on refusal the error arrives as a
/// [`BooleanRefusal`] carrying the implicated pre-flight [`Hazard`] and its
/// §7.7 remedy in the `Display` suffix.
pub fn try_union_diagnosed(a: &Solid, b: &Solid) -> Result<Solid, BooleanRefusal> {
	try_union(a, b).map_err(|e| e.with_preflight(a, b))
}

/// [`try_difference`] with the failure-path diagnosis — see
/// [`try_union_diagnosed`] for the contract.
pub fn try_difference_diagnosed(a: &Solid, b: &Solid) -> Result<Solid, BooleanRefusal> {
	try_difference(a, b).map_err(|e| e.with_preflight(a, b))
}

/// [`try_intersection`] with the failure-path diagnosis — see
/// [`try_union_diagnosed`] for the contract.
pub fn try_intersection_diagnosed(a: &Solid, b: &Solid) -> Result<Solid, BooleanRefusal> {
	try_intersection(a, b).map_err(|e| e.with_preflight(a, b))
}

/// The exact union `A ∪ B`, returned **only if it validates** (closed, manifold,
/// genus ≥ 0). Identical geometry to [`union`]; on failure the invalid solid is
/// withheld and the [`BooleanError`] carries the full [`Validity`] report.
pub fn try_union(a: &Solid, b: &Solid) -> Result<Solid, BooleanError> {
	checked(union(a, b), "union")
}

/// The exact difference `A − B`, returned **only if it validates** (closed,
/// manifold, genus ≥ 0). Identical geometry to [`difference`]; on failure the
/// invalid solid is withheld and the [`BooleanError`] carries the full
/// [`Validity`] report.
pub fn try_difference(a: &Solid, b: &Solid) -> Result<Solid, BooleanError> {
	checked(difference(a, b), "difference")
}

/// The exact intersection `A ∩ B`, returned **only if it validates** (closed,
/// manifold, genus ≥ 0). Identical geometry to [`intersection`]; on failure the
/// invalid solid is withheld and the [`BooleanError`] carries the full
/// [`Validity`] report.
pub fn try_intersection(a: &Solid, b: &Solid) -> Result<Solid, BooleanError> {
	checked(intersection(a, b), "intersection")
}

/// A sealed boolean failed: either the topology check (as [`BooleanError`]) or
/// the tessellation seal — a VALID B-rep whose default tessellation is not
/// watertight (the "valid but leaky" class of DESIGN_GUIDE §7.6; observed when
/// boolean features fight the facet grid, see [`crate::boolean_hazards`]).
#[derive(Debug)]
pub enum SealedError {
	/// The result failed topology validation (see [`BooleanError`]).
	Invalid(BooleanError),
	/// The result validated but `tessellate_default` produced a leaky mesh.
	/// The counts locate the damage class: `boundary_edges` = open-fan cracks,
	/// `non_manifold_edges` = over-shared edges.
	Leaky { op: &'static str, boundary_edges: usize, non_manifold_edges: usize },
}

impl fmt::Display for SealedError {
	fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
		match self {
			SealedError::Invalid(e) => e.fmt(f),
			SealedError::Leaky { op, boundary_edges, non_manifold_edges } => write!(
				f,
				"boolean {op} validated but its default tessellation is NOT watertight ({boundary_edges} boundary edges, {non_manifold_edges} non-manifold edges); route the mesh via precise_mesh/watertight_mesh, or fix the inputs (boolean_hazards names the usual suspects)"
			),
		}
	}
}

impl Error for SealedError {}

/// Run a checked boolean AND seal it: the result must validate *and* its
/// default tessellation must be watertight. Returns the solid together with
/// that already-computed mesh (so a caller gating on the seal never pays for
/// tessellation twice). This closes the gap `try_*` leaves open: a valid
/// B-rep with a cracked default tessellation used to surface only at a
/// downstream watertight gate — now it fails HERE, named, with edge counts.
fn sealed(result: Result<Solid, BooleanError>, op: &'static str) -> Result<(Solid, kernel_core::mesh::Mesh), SealedError> {
	let solid = result.map_err(SealedError::Invalid)?;
	let mesh = crate::tessellate::tessellate_default(&solid);
	if mesh.is_watertight() {
		Ok((solid, mesh))
	} else {
		Err(SealedError::Leaky { op, boundary_edges: mesh.boundary_edge_count(), non_manifold_edges: mesh.non_manifold_edge_count() })
	}
}

/// [`try_union`] plus the tessellation seal — see [`sealed`] for the contract.
pub fn try_union_sealed(a: &Solid, b: &Solid) -> Result<(Solid, kernel_core::mesh::Mesh), SealedError> {
	sealed(try_union(a, b), "union")
}

/// [`try_difference`] plus the tessellation seal — see [`sealed`] for the contract.
pub fn try_difference_sealed(a: &Solid, b: &Solid) -> Result<(Solid, kernel_core::mesh::Mesh), SealedError> {
	sealed(try_difference(a, b), "difference")
}

/// [`try_intersection`] plus the tessellation seal — see [`sealed`] for the contract.
pub fn try_intersection_sealed(a: &Solid, b: &Solid) -> Result<(Solid, kernel_core::mesh::Mesh), SealedError> {
	sealed(try_intersection(a, b), "intersection")
}

/// The tool side of a freeform boolean — what is cut against a
/// [`FreeformSolid`]. Only [`HalfSpace`](FreeformTool::HalfSpace) is inside
/// the shipped slice; every other variant exists so an out-of-scope request
/// refuses LOUDLY through [`try_freeform_boolean`] with a message naming the
/// slice, instead of not compiling or silently degrading.
#[derive(Clone, Copy, Debug)]
pub enum FreeformTool<'a> {
	/// The half-space `(p − origin)·normal ≤ 0` — the IN-SLICE tool.
	/// Difference removes it; intersection keeps it.
	HalfSpace {
		/// A point of the cut plane.
		origin: kernel_core::math::DVec3,
		/// The plane normal; the tool material lies on its NEGATIVE side.
		normal: kernel_core::math::DVec3,
	},
	/// An analytic quadric tool (sphere/cylinder/cone/torus…) — out of scope.
	Quadric(&'a crate::geom::Surface),
	/// A general B-rep solid tool — out of scope.
	Solid(&'a Solid),
	/// Another freeform body — out of scope.
	Freeform(&'a FreeformSolid),
}

/// The checked entry point of the **freeform boolean slice** (DESIGN_GUIDE §24
/// item 1): dispatch `op` between a single-patch [`FreeformSolid`] and a
/// [`FreeformTool`].
///
/// Supported today — honestly and exactly one thing: **difference or
/// intersection with a half-space** (a planar cut), routed through
/// [`freeform_plane_cut`] with its exact-surface / tolerance-curve contract
/// and validity gate. EVERYTHING else — union with a half-space (unbounded),
/// quadric tools, general solids, freeform∩freeform, multi-patch operands —
/// refuses with [`FreeformBoolError::OutOfScope`] whose message names the
/// supported slice and the resolved chord tolerance, so an autonomous caller
/// learns the boundary instead of chasing a panic or a silent facet-level cut.
pub fn try_freeform_boolean(
	a: &FreeformSolid,
	tool: &FreeformTool<'_>,
	op: MeshBoolOp,
	opts: &FreeformCutOptions,
) -> Result<FreeformCut, FreeformBoolError> {
	// Resolve the slice's stated chord tolerance for the refusal message even
	// when we refuse before cutting.
	let chord_tol = if opts.chord_tol > 0.0 {
		opts.chord_tol
	} else if a.mesh.is_empty() {
		0.0
	} else {
		let bb = a.mesh.aabb();
		(bb.max - bb.min).as_dvec3().length().max(1e-9) * 1e-4
	};
	let out_of_scope = |detail: String| Err(FreeformBoolError::OutOfScope { detail, chord_tol });
	match (tool, op) {
		(FreeformTool::HalfSpace { origin, normal }, MeshBoolOp::Difference) => {
			freeform_plane_cut(a, *origin, *normal, Keep::Outside, opts)
		}
		(FreeformTool::HalfSpace { origin, normal }, MeshBoolOp::Intersection) => {
			freeform_plane_cut(a, *origin, *normal, Keep::Inside, opts)
		}
		(FreeformTool::HalfSpace { .. }, MeshBoolOp::Union) => out_of_scope("union with a half-space (an unbounded result)".into()),
		(FreeformTool::Quadric(s), _) => {
			let kind = match s {
				crate::geom::Surface::Plane { .. } => "plane",
				crate::geom::Surface::Cylinder { .. } => "cylinder",
				crate::geom::Surface::Sphere { .. } => "sphere",
				crate::geom::Surface::Cone { .. } => "cone",
				crate::geom::Surface::Torus { .. } => "torus",
			};
			out_of_scope(format!("{op:?} with a {kind} quadric tool"))
		}
		(FreeformTool::Solid(_), _) => out_of_scope(format!("{op:?} with a general B-rep solid tool")),
		(FreeformTool::Freeform(_), _) => out_of_scope(format!("freeform ∩ freeform ({op:?} between two freeform bodies)")),
	}
}

#[cfg(test)]
mod tests {
	use super::*;
	use crate::build::{cuboid, cylinder};
	use crate::geom::Surface;
	use crate::topo::FaceInput;
	use crate::validate::volume;
	use kernel_core::math::DVec3;

	#[test]
	fn checked_difference_returns_the_exact_unchecked_result_for_a_drilled_plate() {
		// Ok path: a through-bore in a plate is a valid genus-1 boolean. The checked
		// API must hand back exactly what the raw op builds — volume BITS identical
		// (the boolean pipeline is deterministic, R5) — plus the validity guarantee.
		let plate = cuboid(DVec3::new(-10.0, -10.0, -3.0), DVec3::new(10.0, 10.0, 3.0));
		let bore = cylinder(DVec3::new(0.0, 0.0, -4.0), DVec3::Z, 2.5, 8.0, 48);
		let drilled = try_difference(&plate, &bore).expect("a drilled plate is a valid difference");
		let v = validate(&drilled);
		let unchecked = difference(&plate, &bore);
		assert!(
			v.is_valid() && v.genus == 1 && volume(&drilled).to_bits() == volume(&unchecked).to_bits(),
			"checked must equal unchecked on the Ok path: {v:?}, vol {} vs {}",
			volume(&drilled),
			volume(&unchecked)
		);
	}

	#[test]
	fn checked_boolean_on_an_open_shell_errors_instead_of_returning_invalid() {
		// Err path: an OPEN shell (a 10mm box missing its top face) is not a solid, so
		// a boolean over it cannot produce a closed result. The unchecked op returns
		// that broken solid; the checked op must withhold it and report WHY.
		let p = |x: f64, y: f64, z: f64| DVec3::new(x, y, z);
		let positions = vec![
			p(0.0, 0.0, 0.0),
			p(10.0, 0.0, 0.0),
			p(10.0, 10.0, 0.0),
			p(0.0, 10.0, 0.0),
			p(0.0, 0.0, 10.0),
			p(10.0, 0.0, 10.0),
			p(10.0, 10.0, 10.0),
			p(0.0, 10.0, 10.0),
		];
		let plane = |origin: DVec3, normal: DVec3| Surface::Plane { origin, normal };
		// All five faces CCW from outside; the +Z cap [4,5,6,7] is deliberately MISSING.
		let faces = vec![
			FaceInput { boundary: vec![0, 3, 2, 1], surface: plane(p(0.0, 0.0, 0.0), -DVec3::Z) },
			FaceInput { boundary: vec![0, 1, 5, 4], surface: plane(p(0.0, 0.0, 0.0), -DVec3::Y) },
			FaceInput { boundary: vec![1, 2, 6, 5], surface: plane(p(10.0, 0.0, 0.0), DVec3::X) },
			FaceInput { boundary: vec![2, 3, 7, 6], surface: plane(p(0.0, 10.0, 0.0), DVec3::Y) },
			FaceInput { boundary: vec![3, 0, 4, 7], surface: plane(p(0.0, 0.0, 0.0), -DVec3::X) },
		];
		let open_box = Solid::from_faces(positions, faces);
		assert!(!validate(&open_box).closed, "the fixture must be an open shell");

		let tool = cuboid(DVec3::new(5.0, 5.0, 5.0), DVec3::new(15.0, 15.0, 15.0));
		// The unchecked union of an open shell is invalid (this is what would have
		// propagated silently); the checked union must catch exactly that.
		let unchecked = validate(&union(&open_box, &tool));
		assert!(!unchecked.is_valid(), "unchecked union of an open shell stays invalid: {unchecked:?}");
		let err = try_union(&open_box, &tool).expect_err("checked union must withhold the invalid result");
		assert!(
			err.op == "union"
				&& !err.validity.is_valid()
				&& err.validity.closed == unchecked.closed
				&& err.validity.manifold == unchecked.manifold,
			"the error must carry the op and the real validity report: {err:?} vs {unchecked:?}"
		);
		// The error formats as ONE informative line naming the op and the failure.
		let line = err.to_string();
		assert!(
			!line.contains('\n')
				&& line.contains("union")
				&& line.contains("withheld")
				&& (line.contains("not closed") || line.contains("non-manifold")),
			"Display must be one informative line: {line:?}"
		);
	}

	#[test]
	fn boolean_error_display_names_every_broken_invariant() {
		// Display coverage for all three reject reasons, from hand-built Validity
		// reports (each is a state `validate` genuinely produces: open shells from
		// degenerate inputs, non-manifold stitches, negative genus from pinches).
		let cases = [
			(Validity { closed: false, manifold: true, euler_characteristic: 1, genus: 0, shells: 1 }, "not closed"),
			(Validity { closed: true, manifold: false, euler_characteristic: 2, genus: 0, shells: 1 }, "non-manifold"),
			(Validity { closed: true, manifold: true, euler_characteristic: 4, genus: -1, shells: 1 }, "negative genus"),
		];
		for (validity, expect) in cases {
			let line = BooleanError { op: "intersection", validity }.to_string();
			assert!(
				!line.contains('\n') && line.contains("intersection") && line.contains(expect),
				"Display must name the broken invariant {expect:?} in one line: {line:?}"
			);
		}
	}
}
