// Copyright (c) LMCAD. Licensed under the MIT License.

//! The program interpreter: parses `{"ops": [...]}`, executes each op against a
//! named-value environment, and produces a structured [`Report`].
//!
//! Failure policy (loud, no silent invalidity):
//! - Execution STOPS at the first failing op; the report names that op and its
//!   structured [`ErrorKind`].
//! - Every solid-producing op is gated through `validate()`: a result that is not
//!   a closed manifold (or is unexpectedly empty) FAILS the op with the full
//!   `Validity` details instead of binding a broken solid.
//! - Kernel panics are caught and surfaced as `internal` errors so the driving
//!   process always receives a parseable report.
//!
//! What lives here and what does not: this module owns the program loop, the
//! named-value environment, the pre-dispatch allocation caps and the dispatch
//! table in [`exec_op`]. What each op *does* lives in [`crate::ops`], one module
//! per family. `exec_op` routes whole families in one arm each, so the compiler
//! still proves every [`OpKind`] variant is handled exactly once.

use std::collections::{BTreeMap, BTreeSet};
use std::panic::AssertUnwindSafe;
use std::path::Path;

use kernel_brep::Solid;
use kernel_core::Mesh;
use kernel_model::Sketch;
use serde_json::Value;

use crate::ops;
use crate::program::OpKind;
use crate::report::{ErrorKind, OpError, OpReport, Report};

/// A named value in the program environment.
///
/// # Why meshes are values, and why that does not break the one-directional bridge
///
/// The voxel/implicit route produces MESHES, not B-reps: `implicit`, `tpms`,
/// `gyroid_block`, `hybrid_boolean`, `mesh_carve`, `shell`, `import_mesh` and
/// the exports all end in a triangle mesh. Those meshes are the files that get
/// PRINTED, and until they were values there was no way to gate them in-program
/// at all — two campaigns shipped print files with no gate on them (theme T10).
///
/// A mesh value is a mesh forever. Nothing here promotes one to a [`Solid`]:
/// the only field→exact route stays the explicit, honestly-labelled
/// `solid_from_implicit` reverse bridge, which re-meshes and wraps a FACETED
/// B-rep under its own `route: "voxel"` receipt. Binding the mesh adds an
/// oracle; it does not add a conversion.
pub(crate) enum EnvValue {
	/// An exact B-rep solid (most ops).
	Solid(Solid),
	/// A solved 2D sketch (consumed by `sketch_extrude` / `sketch_revolve`).
	Sketch(Sketch),
	/// A triangle mesh — the voxel/implicit route's result, and what an export
	/// actually wrote. Accepted by the mesh-capable measures (`validate`,
	/// `volume`, `bounding_box`, `mesh_components`, `support_report`, `assert`).
	Mesh(Mesh),
}

impl EnvValue {
	/// Human-readable kind name for `wrong_type` messages.
	fn kind_name(&self) -> &'static str {
		match self {
			EnvValue::Solid(_) => "solid",
			EnvValue::Sketch(_) => "sketch",
			EnvValue::Mesh(_) => "mesh",
		}
	}
}

/// What a successful op hands back to the runner.
pub(crate) struct Outcome {
	/// The value to bind under the op's id (geometry-producing ops only).
	pub(crate) value: Option<EnvValue>,
	/// Op-specific measurements for the report entry.
	pub(crate) measures: Option<Value>,
	/// The file written (export ops only).
	pub(crate) file: Option<String>,
}

impl Outcome {
	/// An outcome that binds nothing and only reports measures.
	pub(crate) fn measures(measures: Value) -> Outcome {
		Outcome { value: None, measures: Some(measures), file: None }
	}
}

/// Shorthand [`OpError`] constructor.
pub(crate) fn err(kind: ErrorKind, message: impl Into<String>) -> OpError {
	OpError { kind, message: message.into() }
}

/// Parse and execute a JSON program. Export files are written relative to
/// `out_dir` (created on demand); relative INPUT paths (`load_part`) also
/// resolve against `out_dir` (library compatibility — the CLI passes the
/// program file's own directory instead, see
/// [`run_program_with_input_base`]). Always returns a report — parse failures
/// become a single `$program` entry with kind `parse`.
pub fn run_program(json_text: &str, out_dir: &Path) -> Report {
	run_program_with_input_base(json_text, out_dir, out_dir)
}

/// [`run_program`] with the directory relative **input** paths (`load_part
/// file`) resolve against stated explicitly. The CLI passes the program
/// file's parent directory, so a program references its parts relative to
/// ITSELF and stays relocatable (FRICTION #13); `.lmcasm` `path` sources
/// resolve the same way. Output paths still resolve against `out_dir`.
pub fn run_program_with_input_base(json_text: &str, out_dir: &Path, input_base: &Path) -> Report {
	let parsed: Value = match serde_json::from_str(json_text) {
		Ok(v) => v,
		Err(e) => return Report::program_failure(ErrorKind::Parse, format!("program is not valid JSON: {e}")),
	};
	let Some(ops) = parsed.get("ops").and_then(Value::as_array) else {
		return Report::program_failure(ErrorKind::Parse, "program must be a JSON object with an 'ops' array: {\"ops\": [...]}");
	};

	let mut env: BTreeMap<String, EnvValue> = BTreeMap::new();
	let mut all_ids: BTreeSet<String> = BTreeSet::new();
	let mut asm = crate::asmops::AsmProgramState::default();
	let mut reports: Vec<OpReport> = Vec::new();

	for (index, raw) in ops.iter().enumerate() {
		let (id, warnings, result) = run_one(index, raw, &mut env, &mut all_ids, &mut asm, out_dir, input_base);
		match result {
			Ok(outcome) => {
				if let Some(value) = outcome.value {
					env.insert(id.clone(), value);
				}
				reports.push(OpReport { id, ok: true, measures: outcome.measures, warnings, file: outcome.file, error: None });
			}
			Err(error) => {
				reports.push(OpReport { id, ok: false, measures: None, warnings, file: None, error: Some(error) });
				return Report { ok: false, ops: reports };
			}
		}
	}
	Report { ok: true, ops: reports }
}

/// Identify, parse, and execute one raw op value. Returns the op id (or a
/// synthetic `#<index>` when none could be read), any non-fatal warnings
/// (unknown-param hazards), plus the outcome.
fn run_one(
	index: usize,
	raw: &Value,
	env: &mut BTreeMap<String, EnvValue>,
	all_ids: &mut BTreeSet<String>,
	asm: &mut crate::asmops::AsmProgramState,
	out_dir: &Path,
	input_base: &Path,
) -> (String, Vec<String>, Result<Outcome, OpError>) {
	let fallback_id = format!("#{index}");
	let Some(obj) = raw.as_object() else {
		return (
			fallback_id.clone(),
			Vec::new(),
			Err(err(ErrorKind::InvalidParam, format!("op {fallback_id}: each entry of 'ops' must be a JSON object"))),
		);
	};
	let Some(id) = obj.get("id").and_then(Value::as_str) else {
		return (
			fallback_id.clone(),
			Vec::new(),
			Err(err(ErrorKind::InvalidParam, format!("op {fallback_id}: missing required string field 'id'"))),
		);
	};
	let id = id.to_string();
	if !all_ids.insert(id.clone()) {
		return (
			id.clone(),
			Vec::new(),
			Err(err(ErrorKind::DuplicateId, format!("op '{id}': this id was already used by an earlier op — ids must be unique"))),
		);
	}
	let Some(op_name) = obj.get("op").and_then(Value::as_str) else {
		return (id.clone(), Vec::new(), Err(err(ErrorKind::InvalidParam, format!("op '{id}': missing required string field 'op'"))));
	};
	// Direction-like vectors have no meaningful zero value. Reject them before
	// constructors can normalize NaN/zero and produce a misleading success.
	for name in ["axis", "direction", "normal", "up", "build_direction", "x_dir", "y_dir"] {
		if let Some(a) = obj.get(name).and_then(Value::as_array) {
			if a.len() == 3 {
				let xyz = [a[0].as_f64(), a[1].as_f64(), a[2].as_f64()];
				if xyz.iter().all(Option::is_some) {
					let v = [xyz[0].unwrap(), xyz[1].unwrap(), xyz[2].unwrap()];
					let norm2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
					if !v.iter().all(|n| n.is_finite()) || !norm2.is_finite() || norm2 <= 1e-24 {
						return (
							id.clone(), Vec::new(),
							Err(err(ErrorKind::InvalidParam,
								format!("op '{id}': {name} must be a non-zero finite 3-vector"))),
						);
					}
				}
			}
		}
	}

	// Unknown parameters fail closed. A misspelled manufacturing dimension must
	// never silently select a default and still return an apparently valid part.
	// `_`-prefixed keys remain the explicit in-op comment convention.
	let mut warnings: Vec<String> = Vec::new();
	if let Some(params) = crate::discover::op_params(op_name) {
		for key in obj.keys() {
			// `require` is UNIVERSAL (crate::require) — accepted on every op and
			// applied to that op's own measures, so it is never an unknown param.
			if key == "id" || key == "op" || key == crate::require::REQUIRE_KEY || key.starts_with('_') {
				continue;
			}
			if !params.iter().any(|p| p.name == key || p.aliases.contains(&key.as_str())) {
				warnings.push(format!(
					"unknown param '{key}' — '{op_name}' does not accept it; call describe {{\"name\":\"{op_name}\"}} for the accepted params"
				));
			}
		}
	}
	if !warnings.is_empty() {
		let message = format!("op '{id}': {}", warnings.join("; "));
		return (id.clone(), warnings, Err(err(ErrorKind::InvalidParam, message)));
	}

	let kind: OpKind = match serde_json::from_value(raw.clone()) {
		Ok(k) => k,
		// serde reports an unrecognized tag as "unknown variant `...`"; the
		// error-paths test pins this mapping so a serde message change is caught.
		// The SAME serde message names an unrecognised enum VALUE of a known op's param
		// (`"mode": "lenient"`), which is a bad param, not an unknown op — so only treat it
		// as an unknown op when the name is not a known op at all.
		Err(e) if e.to_string().starts_with("unknown variant") && crate::discover::op_params(op_name).is_none() => {
			// A tag that exists only behind the `catalog` cargo feature is refused by name, so a
			// `--no-default-features` build says "compiled out", not "typo".
			let message = if crate::discover::CATALOG_OP_NAMES.contains(&op_name) {
				format!("op '{id}': op '{op_name}' is behind the `catalog` cargo feature, which this build of kernel-api was compiled without — rebuild with default features (or `--features catalog`) to use it")
			} else {
				format!("op '{id}': unknown op '{op_name}' — not one of the {} supported ops; call the `describe` op to enumerate them", crate::discover::OP_COUNT)
			};
			return (id.clone(), warnings, Err(err(ErrorKind::UnknownOp, message)));
		}
		Err(e) => {
			return (id.clone(), warnings, Err(err(ErrorKind::InvalidParam, format!("op '{id}' ('{op_name}'): bad params: {e}"))));
		}
	};

	// V3: reject allocation-hostile params (huge segment/pattern/voxel counts) BEFORE any op
	// runs, so a mistaken or malicious count can't OOM the shared process past the panic net.
	if let Err(e) = check_limits(&id, obj) {
		return (id, warnings, Err(e));
	}

	// A kernel panic must not kill the driving process without a report.
	let result = std::panic::catch_unwind(AssertUnwindSafe(|| exec_op(&id, kind, env, all_ids, asm, out_dir, input_base)));
	let mut result = match result {
		Ok(r) => r,
		Err(payload) => {
			let detail = payload
				.downcast_ref::<&str>()
				.map(|s| (*s).to_string())
				.or_else(|| payload.downcast_ref::<String>().cloned())
				.unwrap_or_else(|| "<non-string panic payload>".to_string());
			Err(err(ErrorKind::Internal, format!("op '{id}' ('{op_name}'): kernel panic: {detail}")))
		}
	};

	// The universal gate, applied to whatever the op measured. A `require` that
	// is unmet turns a successful op into an `assert_failed` FAILURE, so the
	// program stops here and the report names the gate — the whole point of an
	// in-program gate over an external grep.
	if let Ok(outcome) = &mut result {
		match crate::require::apply(&id, obj, outcome.measures.as_ref()) {
			Ok(Some(gated)) => outcome.measures = Some(gated),
			Ok(None) => {}
			Err(e) => result = Err(e),
		}
	}
	(id, warnings, result)
}

// --- Environment access -------------------------------------------------------

/// Look up `name`, distinguishing "never defined" from "defined but bound no value".
fn fetch<'e>(
	env: &'e BTreeMap<String, EnvValue>,
	all_ids: &BTreeSet<String>,
	op_id: &str,
	param: &str,
	name: &str,
) -> Result<&'e EnvValue, OpError> {
	env.get(name).ok_or_else(|| {
		if all_ids.contains(name) {
			err(
				ErrorKind::MissingRef,
				format!("op '{op_id}' param '{param}': '{name}' is a measure/export op and binds no geometry"),
			)
		} else {
			err(
				ErrorKind::MissingRef,
				format!("op '{op_id}' param '{param}': no result named '{name}' — it must be the id of an earlier geometry-producing op"),
			)
		}
	})
}

/// Fetch a [`Solid`] from the environment, or a `wrong_type` error.
pub(crate) fn fetch_solid<'e>(
	env: &'e BTreeMap<String, EnvValue>,
	all_ids: &BTreeSet<String>,
	op_id: &str,
	param: &str,
	name: &str,
) -> Result<&'e Solid, OpError> {
	match fetch(env, all_ids, op_id, param, name)? {
		EnvValue::Solid(s) => Ok(s),
		other => Err(err(
			ErrorKind::WrongType,
			format!("op '{op_id}' param '{param}': '{name}' is a {}, expected a solid", other.kind_name()),
		)),
	}
}

/// What a mesh-capable measure was handed: an exact solid (measured on its
/// tessellation) or a mesh value (measured directly).
///
/// The two carry DIFFERENT provenance and the measures say which: a solid can be
/// re-tessellated at any tolerance, a mesh is the fixed set of triangles that
/// was written to (or read from) a file. Gating the mesh is the only way to gate
/// what actually prints.
pub(crate) enum Measurable<'e> {
	Solid(&'e Solid),
	Mesh(&'e Mesh),
}

impl Measurable<'_> {
	/// The triangles to measure. A solid is tessellated crack-free at `tol`; a
	/// mesh IS its triangles and `tol` does not apply to it (saying otherwise
	/// would be a claim the geometry cannot support).
	pub(crate) fn mesh(&self, tol: f64) -> std::borrow::Cow<'_, Mesh> {
		match self {
			Measurable::Solid(s) => std::borrow::Cow::Owned(kernel_brep::tessellate_adaptive_tol(s, tol)),
			Measurable::Mesh(m) => std::borrow::Cow::Borrowed(m),
		}
	}
	/// `"solid"` for an exact B-rep measured through its tessellation, `"mesh"`
	/// for a bound mesh — the provenance every measure taken this way carries.
	pub(crate) fn source(&self) -> &'static str {
		match self {
			Measurable::Solid(_) => "solid",
			Measurable::Mesh(_) => "mesh",
		}
	}
}

/// Fetch a solid OR a mesh — the input rule of every mesh-capable measure.
pub(crate) fn fetch_measurable<'e>(
	env: &'e BTreeMap<String, EnvValue>,
	all_ids: &BTreeSet<String>,
	op_id: &str,
	param: &str,
	name: &str,
) -> Result<Measurable<'e>, OpError> {
	match fetch(env, all_ids, op_id, param, name)? {
		EnvValue::Solid(s) => Ok(Measurable::Solid(s)),
		EnvValue::Mesh(m) => Ok(Measurable::Mesh(m)),
		other => Err(err(
			ErrorKind::WrongType,
			format!("op '{op_id}' param '{param}': '{name}' is a {}, expected a solid or a mesh", other.kind_name()),
		)),
	}
}

/// Fetch a [`Sketch`] from the environment, or a `wrong_type` error.
pub(crate) fn fetch_sketch<'e>(
	env: &'e BTreeMap<String, EnvValue>,
	all_ids: &BTreeSet<String>,
	op_id: &str,
	param: &str,
	name: &str,
) -> Result<&'e Sketch, OpError> {
	match fetch(env, all_ids, op_id, param, name)? {
		EnvValue::Sketch(s) => Ok(s),
		other => Err(err(
			ErrorKind::WrongType,
			format!("op '{op_id}' param '{param}': '{name}' is a {}, expected a sketch", other.kind_name()),
		)),
	}
}

// --- Allocation caps (audit V3) -----------------------------------------------------------------
// An agent request must not OOM or hang the shared process. These ceilings sit FAR above any real
// design (a smooth primitive is ~256 facets; a large voxel grid is ~256³ = 16.7M) and FAR below the
// point where a `usize` count would exhaust memory. The check is a cheap structural pass on the raw
// op JSON, so it covers every op carrying one of these fields uniformly and rejects BEFORE any
// allocation with a matchable `InvalidParam`.
const MAX_SEGMENTS: u64 = 8_192; // facet count on any primitive / revolve / arc
const MAX_PATTERN: u64 = 100_000; // repeated-feature instance count
pub(crate) const MAX_GRID_CELLS: u64 = 50_000_000; // voxel-grid cell product (~200 MB as f32)
pub(crate) const MAX_PATTERN_COUNT: u64 = 500; // solid-clone count of linear_pattern / polar_pattern
pub(crate) const MAX_PATTERN_FACES: usize = 100_000; // count × per-clone faces budget of the pattern union

/// Reject allocation-hostile op parameters before dispatch. Structural (field-name based) so any op
/// with a `segments`/`shape`/`n`/gyroid field is covered without a per-variant match.
fn check_limits(op_id: &str, obj: &serde_json::Map<String, Value>) -> Result<(), OpError> {
	let over = |field: &str, cap: u64| -> Result<(), OpError> {
		match obj.get(field).and_then(Value::as_u64) {
			Some(v) if v > cap => Err(err(
				ErrorKind::InvalidParam,
				format!("op '{op_id}': '{field}' {v} exceeds the safety cap {cap} — rejected before allocation"),
			)),
			_ => Ok(()),
		}
	};
	for field in ["segments", "ring_segments", "tube_segments", "arc_segments"] {
		over(field, MAX_SEGMENTS)?;
	}
	over("n", MAX_PATTERN)?;
	// solid-clone patterns (`linear_pattern` / `polar_pattern` `count`): each clone is a full
	// B-rep union fold, far heavier than a repeated hole, so the ceiling is much lower.
	over("count", MAX_PATTERN_COUNT)?;
	// voxel-grid cell product (SampleDensityGrid/MeshDensityGrid `shape`)
	if let Some(shape) = obj.get("shape").and_then(Value::as_array) {
		let cells: u128 = shape.iter().filter_map(Value::as_u64).map(u128::from).product();
		if cells > u128::from(MAX_GRID_CELLS) {
			return Err(err(
				ErrorKind::InvalidParam,
				format!("op '{op_id}': grid shape product {cells} exceeds the cap {MAX_GRID_CELLS} cells — rejected before allocation"),
			));
		}
	}
	// gyroid dual-contour grid: (2·half / voxel)³ cells
	if let (Some(half), Some(voxel)) = (obj.get("half").and_then(Value::as_f64), obj.get("voxel").and_then(Value::as_f64)) {
		if half > 0.0 && voxel > 0.0 {
			let per_axis = (2.0 * half / voxel).ceil();
			let cells = per_axis * per_axis * per_axis;
			if cells > MAX_GRID_CELLS as f64 {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': gyroid grid ≈{cells:.0} cells (2·half/voxel)³ exceeds the cap {MAX_GRID_CELLS} — rejected before allocation"),
				));
			}
		}
	}
	Ok(())
}

// --- The op dispatcher ------------------------------------------------------------

/// Execute one parsed op against the environment.
///
/// One arm per op FAMILY: the arm names every variant that family implements and
/// hands the whole [`OpKind`] to its `exec` in [`crate::ops`]. The match is still
/// exhaustive over `OpKind`, so adding a variant without routing it is a compile
/// error — the same guarantee the one-arm-per-op form gave.
fn exec_op(
	op_id: &str,
	kind: OpKind,
	env: &mut BTreeMap<String, EnvValue>,
	all_ids: &BTreeSet<String>,
	asm: &mut crate::asmops::AsmProgramState,
	out_dir: &Path,
	input_base: &Path,
) -> Result<Outcome, OpError> {
	match kind {
		// --- Assemblies (in-program) — see asmops.rs -------------------------------
		kind @ (OpKind::AsmInstance { .. }
			| OpKind::AsmInstanceMesh { .. }
			| OpKind::AsmMate { .. }
			| OpKind::AsmMateAxis { .. }
			| OpKind::AsmMateFace { .. }
			| OpKind::AsmSolve { .. }
			| OpKind::AsmContacts { .. }
			| OpKind::AsmInterferenceVolume { .. }
			| OpKind::AsmMassProperties { .. }
			| OpKind::AsmExport { .. }
			| OpKind::AsmExportStep { .. }
			| OpKind::AsmSave { .. }
			| OpKind::GearTrainPoses { .. }) => ops::assemblies::exec(op_id, env, all_ids, asm, out_dir, input_base, kind),
		// --- Solid primitives & sweeps ----------------------------------------
		kind @ (OpKind::Box { .. } | OpKind::Cylinder { .. } | OpKind::Sphere { .. }
			| OpKind::Cone { .. } | OpKind::Torus { .. } | OpKind::Extrude { .. }
			| OpKind::ExtrudeWithHoles { .. } | OpKind::ExtrudeTapered { .. }
			| OpKind::Revolve { .. } | OpKind::Loft { .. } | OpKind::Sweep { .. }) => ops::primitives::exec(op_id, kind),
		// --- Sketch --------------------------------------------------------------
		kind @ (OpKind::Sketch { .. } | OpKind::SketchRevolve { .. }) => ops::sketch::exec(op_id, env, all_ids, kind),
		#[cfg(feature = "catalog")]
		kind @ OpKind::SketchExtrude { .. } => ops::sketch::exec(op_id, env, all_ids, kind),
		// --- Booleans ----------------------------------------------------------------
		kind @ (OpKind::Union { .. } | OpKind::Difference { .. }
			| OpKind::Intersection { .. } | OpKind::UnionAll { .. }) => ops::booleans::exec(op_id, env, all_ids, kind),
		// --- Features & transforms ------------------------------------------------------
		kind @ (OpKind::FilletEdgeNear { .. } | OpKind::ChamferEdgeNear { .. }
			| OpKind::FilletCircularRim { .. } | OpKind::Translate { .. }
			| OpKind::RotateZ { .. } | OpKind::Pose { .. } | OpKind::RotateX { .. }
			| OpKind::RotateY { .. } | OpKind::Mirror { .. }
			| OpKind::LinearPattern { .. } | OpKind::PolarPattern { .. }) => ops::features::exec(op_id, env, all_ids, kind),
		// --- Measures ----------------------------------------------------------------------
		kind @ (OpKind::Validate { .. } | OpKind::Volume { .. }
			| OpKind::ExactVolume { .. } | OpKind::MassProperties { .. }
			| OpKind::BoundingBox { .. } | OpKind::WallThickness { .. }
			| OpKind::DraftAnalysis { .. } | OpKind::MeshComponents { .. }) => ops::measure::exec(op_id, env, all_ids, kind),
		// --- Assertions ----------------------------------------------------------------------
		kind @ (OpKind::Assert { .. } | OpKind::AssertDisjoint { .. }
			| OpKind::CoincidentFit { .. } | OpKind::SupportReport { .. }
			| OpKind::Clearance { .. } | OpKind::Describe { .. }
			| OpKind::ListFaces { .. } | OpKind::ListEdges { .. }) => ops::measure::exec(op_id, env, all_ids, kind),
		// --- Exports -------------------------------------------------------------------------
		kind @ (OpKind::ExportStl { .. } | OpKind::Export3mf { .. }
			| OpKind::ExportStep { .. }) => ops::io::exec(op_id, env, all_ids, asm, out_dir, input_base, kind),
		// --- Native formats ----------------------------------------------------------------------
		kind @ OpKind::LoadPart { .. } => ops::io::exec(op_id, env, all_ids, asm, out_dir, input_base, kind),
		// --- Imports -----------------------------------------------------------------------------
		kind @ (OpKind::ImportStep { .. }
			| OpKind::ImportMesh { .. } | OpKind::MeshCarve { .. }
			| OpKind::MeasureDimension { .. }
			| OpKind::HybridBoolean { .. }) => ops::io::exec(op_id, env, all_ids, asm, out_dir, input_base, kind),
		#[cfg(feature = "catalog")]
		kind @ OpKind::Tpms { .. } => ops::io::exec(op_id, env, all_ids, asm, out_dir, input_base, kind),
		// --- Implicit / hybrid ------------------------------------------------------------------
		kind @ (OpKind::SampleDensityGrid { .. }
			| OpKind::MeshDensityGrid { .. }
			| OpKind::Implicit { .. } | OpKind::Shell { .. }) => ops::hybrid::exec(op_id, env, all_ids, out_dir, input_base, kind),
		#[cfg(feature = "catalog")]
		kind @ OpKind::GyroidBlock { .. } => ops::hybrid::exec(op_id, env, all_ids, out_dir, input_base, kind),
		// --- Voxel-route solid ops & interrogation probes (2026-07-29 implicit wave) -------------
		kind @ (OpKind::OffsetSolid { .. }
			| OpKind::ShellSolid { .. }
			| OpKind::SolidFromImplicit { .. }
			| OpKind::ThinWall { .. } | OpKind::MinLigament { .. }) => ops::hybrid::exec(op_id, env, all_ids, out_dir, input_base, kind),
		// --- Parts library (curated, admission-gated; BAR.md I7) -------------------------------
		#[cfg(feature = "catalog")]
		kind @ (OpKind::LibraryAdd { .. } | OpKind::LibrarySearch { .. }
			| OpKind::LibraryInstantiate { .. } | OpKind::LibraryDeprecate { .. }
			| OpKind::LibraryRemove { .. }) => ops::library::exec(op_id, out_dir, kind),
		// --- Standard parts catalog ---------------------------------------------------------------
		kind @ (OpKind::SpurGear { .. } | OpKind::HexNut { .. } | OpKind::Washer { .. }
			| OpKind::SocketHeadCapScrew { .. } | OpKind::DowelPin { .. }
			| OpKind::CirclipExternal { .. } | OpKind::FlatHeadScrew { .. }
			| OpKind::ButtonHeadScrew { .. } | OpKind::SetScrew { .. } | OpKind::LockNut { .. }
			| OpKind::CompressionSpring { .. } | OpKind::ORing { .. } | OpKind::ORingCord { .. }
			| OpKind::DeepGrooveBearing { .. } | OpKind::FlangedBearing { .. }) => ops::catalog::exec(op_id, kind),
		#[cfg(feature = "catalog")]
		kind @ (OpKind::HexBolt { .. } | OpKind::Gt2Pulley { .. } | OpKind::ChainSprocket { .. }
			| OpKind::Shaft { .. } | OpKind::ParallelKey { .. } | OpKind::CirclipInternal { .. }
			| OpKind::ThreadedRod { .. } | OpKind::Standoff { .. } | OpKind::Extrusion2020 { .. }
			| OpKind::Extrusion3030 { .. } | OpKind::Tnut2020 { .. } | OpKind::JawCouplingHub { .. }
			| OpKind::JawCouplingSpider { .. } | OpKind::SetScrewCoupling { .. }
			| OpKind::ClampCoupling { .. } | OpKind::LinearBearingLmuu { .. }
			| OpKind::Sc8uuBlock { .. } | OpKind::ShaftSupportSk8 { .. }
			| OpKind::ShaftSupportShf8 { .. } | OpKind::Mgn12Rail { .. } | OpKind::Mgn12Carriage { .. }
			| OpKind::ThrustBearing { .. } | OpKind::Kp08PillowBlock { .. } | OpKind::PipeBossG { .. }
			| OpKind::HoseBarb { .. } | OpKind::ShoulderBolt { .. } | OpKind::SpringWasher { .. }
			| OpKind::LeadScrewTr8 { .. } | OpKind::LeadScrewNutTr8 { .. } | OpKind::NemaMotor { .. }
			| OpKind::NemaMountPlate { .. } | OpKind::GearRack { .. } | OpKind::InternalGear { .. }) => ops::catalog::exec(op_id, kind),
		// --- Standard feature cuts ------------------------------------------------------------------
		kind @ (OpKind::HeatsetInsertBoss { .. } | OpKind::ORingFaceGland { .. }
			| OpKind::TeardropHole { .. } | OpKind::BridgedCounterbore { .. }
			| OpKind::BoardMount { .. }) => ops::cuts::exec(op_id, env, all_ids, kind),
		#[cfg(feature = "catalog")]
		kind @ (OpKind::CirclipGrooveExternal { .. } | OpKind::CirclipGrooveInternal { .. }
			| OpKind::ORingGroove { .. } | OpKind::ORingFaceGlandRacetrack { .. }
			| OpKind::Pc4Port { .. } | OpKind::Tr8NutTrap { .. }
			| OpKind::NemaMountCut { .. } | OpKind::ServoPocket { .. }) => ops::cuts::exec(op_id, env, all_ids, kind),
		// --- Design-math lookups ----------------------------------------------------------------------
		kind @ (OpKind::Iso286Fit { .. } | OpKind::HeatsetSpec { .. }
			| OpKind::MetricCordGland { .. } | OpKind::RacetrackCordLength { .. }) => ops::designmath::exec(op_id, kind),
		#[cfg(feature = "catalog")]
		kind @ (OpKind::Gt2Belt { .. } | OpKind::Gt2CenterDistance { .. }
			| OpKind::PipeThreadG { .. }) => ops::designmath::exec(op_id, kind),
		// --- Hole wizard ----------------------------------------------------------------------------
		kind @ (OpKind::Drill { .. } | OpKind::ClearanceHole { .. }
			| OpKind::CounterboreHole { .. } | OpKind::CountersinkHole { .. }
			| OpKind::TapDrillHole { .. } | OpKind::BoltCircle { .. }
			| OpKind::BearingSeat { .. }) => ops::holes::exec(op_id, env, all_ids, kind),
		// --- Modelled ISO threads -------------------------------------------------------------------
		kind @ (OpKind::ThreadSpec { .. } | OpKind::ThreadRidge { .. }
			| OpKind::ExportThreaded { .. }) => ops::threads::exec(op_id, env, all_ids, out_dir, kind),
	}
}
