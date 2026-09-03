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

use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::panic::AssertUnwindSafe;
use std::path::{Component, Path, PathBuf};

use kernel_brep::holes::{self, HoleDepth, HoleError};
use kernel_brep::math::{DAffine3, DVec2, DVec3};
use kernel_brep::{FilletError, Solid, StepError};
use kernel_core::math::Vec3;
use kernel_core::{check_mesh, make_manifold, Aabb, Mesh, MeshReport, Resolution, Sdf};
use kernel_implicit::{
	dual_contour_narrowband, manifold_dual_contour, mesh_boolean_implicit, BoolOp, Cuboid as ImplicitCuboid, Gyroid, MeshSdf, Node,
};
use kernel_model::library::{AddOptions, AdmissionError, EntryMeta, Library, LibraryError, ParamSpec, Provenance};
use kernel_model::{
	format, hybrid_boolean, parts, watertight_mesh, watertight_mesh_of, BooleanOp, ConstraintState, HybridError, HybridOperand,
	HybridRoute, Sketch, SketchConstraint, SketchError,
};
use serde_json::{json, Value};

use crate::implicit;
use crate::program::{BoltHoleSpec, BoolOpSpec, ConstraintSpec, FitSpec, LibraryMetaSpec, MesherSpec, OpKind};
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

/// A rotation taking +Z onto the unit vector `dir` (any rotation about `dir`
/// will do — the shapes placed with it are surfaces of revolution). Uses the
/// shortest-arc axis; the antipodal case gets an explicit 180° flip because the
/// cross product vanishes there.
fn align_z_to(dir: DVec3) -> kernel_brep::math::DMat3 {
	use kernel_brep::math::DMat3;
	let z = DVec3::Z;
	let c = z.dot(dir);
	if c > 1.0 - 1e-12 {
		return DMat3::IDENTITY;
	}
	if c < -1.0 + 1e-12 {
		return DMat3::from_rotation_x(std::f64::consts::PI);
	}
	let axis = z.cross(dir).normalize();
	DMat3::from_axis_angle(axis, c.clamp(-1.0, 1.0).acos())
}

/// The witness block `validate` reports when `geometric_ok` is false: the two
/// crossing triangles, a point on the crossing, and the total pair count.
fn self_intersection_json(w: &kernel_core::mesh::SelfIntersection) -> Value {
	json!({
		"triangles": w.triangles,
		"point": [w.point.x, w.point.y, w.point.z],
		"pairs": w.pairs,
	})
}

/// Validate the two knobs of the connectivity oracle. Shared by `mesh_components`
/// and `assert`, so the gate and its diagnostic can never be tuned differently.
fn connectivity_tolerances(op_id: &str, tol: f64, weld_tol: f64) -> Result<(), OpError> {
	if !(tol.is_finite() && tol > 0.0) {
		return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': tol must be a positive chord tolerance in mm")));
	}
	if !(weld_tol.is_finite() && weld_tol > 0.0) {
		return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': weld_tol must be a positive weld scale in mm")));
	}
	Ok(())
}

/// The connected-body count plus the receipt that says whether it can be
/// believed.
///
/// # The trust rule
///
/// Union-find over welded triangles answers "how many connected pieces is this
/// SURFACE in". That is the part's body count only when the surface is the whole
/// boundary of the part. A bound solid is closed and manifold by construction
/// (every solid-producing op is gated through `validate`), so if its measurement
/// tessellation has boundary edges the faceter has dropped geometry, and the
/// count is then counting faceter cracks. Reporting that number as a body count
/// is precisely the confident-wrong-answer this engine refuses to give, so the
/// op FAILS instead, and says what to gate meanwhile.
///
/// A bound MESH is different: openness is a property of the data, not a defect
/// of the measurement, so an open mesh is measured and reported honestly with
/// `watertight: false` for `require` to gate.
fn connectivity_measures(
	op_id: &str,
	mesh: &Mesh,
	tol: f64,
	weld_tol: f64,
	source: &str,
) -> Result<serde_json::Map<String, Value>, OpError> {
	// Topology only — `check_mesh` would also run the self-intersection sweep,
	// which is orders of magnitude more expensive and answers a question this
	// gate does not ask. (`validate` is where self-intersection is paid for, on
	// demand.) These two are edge-hash passes, linear in triangle count.
	let boundary_edges = mesh.boundary_edge_count();
	let watertight = mesh.is_two_manifold();
	let components = mesh.component_count(weld_tol as f32);
	// Openings are the only defect that can break the count. A winding
	// inconsistency (`non_orientable`) leaves every triangle in place and every
	// vertex shared, so connectivity is untouched — it is reported, never
	// refused. (It USED to refuse: `boundary_edge_count` counted non-orientable
	// edges as boundary edges until 2026-08-08, which turned this guard on 11
	// shipped part programs whose tessellations are closed.)
	if source == "solid" && boundary_edges > 0 {
		return Err(err(
			ErrorKind::InvalidGeometry,
			format!(
				"op '{op_id}': the connectivity oracle cannot be trusted on this solid — tessellating it at tol {tol} mm left {} boundary edges ({} triangles), so the measurement surface is NOT closed and its component count ({components}) counts faceter cracks, not severed bodies. A bound solid is closed by construction, so this is a tessellation defect, not a geometry defect (a planar face carrying inner/hole loops is the known case: `extrude_with_holes` and `import_step` pockets). Gate this part with `validate` (closed / manifold / shells) meanwhile, and/or `export_stl` it and run this measure on the export's bound mesh — the exported mesh IS what prints",
				boundary_edges,
				mesh.triangle_count()
			),
		));
	}
	let mut m = serde_json::Map::new();
	m.insert("components".into(), json!(components));
	m.insert("is_one_body".into(), json!(components == 1));
	m.insert("triangles".into(), json!(mesh.triangle_count()));
	m.insert("tol".into(), json!(tol));
	m.insert("weld_tol".into(), json!(weld_tol));
	m.insert("watertight".into(), json!(watertight));
	m.insert("boundary_edges".into(), json!(boundary_edges));
	// Reported, not gated: this is what `watertight: false` means whenever
	// `boundary_edges` is 0, and without it that pair is unexplained. Another
	// edge-hash pass, not `check_mesh` (which would also pay for the
	// self-intersection sweep this gate does not ask about).
	let non_orientable = mesh.non_orientable_edge_count();
	m.insert("non_orientable_edges".into(), json!(non_orientable));
	if non_orientable > 0 {
		// A nonzero count must be locatable, not just countable: midpoints of
		// the first few offending edges, for aiming a fix or a disclosure.
		m.insert("non_orientable_witness".into(), json!(mesh.non_orientable_edge_witnesses(8)));
	}
	m.insert("source".into(), json!(source));
	Ok(m)
}

/// The `describe` entry for the universal `require` gate — identical for every
/// op, because `require` IS identical for every op.
fn universal_require_param() -> Value {
	json!({ "name": crate::require::REQUIRE_KEY, "type": "object", "required": false, "doc": crate::require::REQUIRE_DOC })
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
		// The SAME message names an unrecognised enum VALUE of a known op's param
		// (`"mode": "lenient"`), which is a bad param, not an unknown op.
		Err(e) if e.to_string().starts_with("unknown variant") && crate::discover::op_params(op_name).is_none() => {
			return (
				id.clone(),
				warnings,
				Err(err(ErrorKind::UnknownOp, format!("op '{id}': unknown op '{op_name}' — not one of the {} supported ops; call the `describe` op to enumerate them", crate::discover::OP_COUNT))),
			);
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
	fn mesh(&self, tol: f64) -> std::borrow::Cow<'_, Mesh> {
		match self {
			Measurable::Solid(s) => std::borrow::Cow::Owned(kernel_brep::tessellate_adaptive_tol(s, tol)),
			Measurable::Mesh(m) => std::borrow::Cow::Borrowed(m),
		}
	}
	/// `"solid"` for an exact B-rep measured through its tessellation, `"mesh"`
	/// for a bound mesh — the provenance every measure taken this way carries.
	fn source(&self) -> &'static str {
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
fn fetch_sketch<'e>(
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

// --- Conversions & gates --------------------------------------------------------

/// DVec3 → JSON array, for entity descriptors.
pub(crate) fn v3a(v: DVec3) -> [f64; 3] {
	[v.x, v.y, v.z]
}
/// Centroid of a boundary polygon (a witness point on/near the face).
pub(crate) fn polygon_centroid(pts: &[DVec3]) -> DVec3 {
	if pts.is_empty() {
		return DVec3::ZERO;
	}
	pts.iter().fold(DVec3::ZERO, |a, &p| a + p) / pts.len() as f64
}
/// Newell area of a planar polygon (exact for planar faces; boundary-only for curved).
fn polygon_area(pts: &[DVec3]) -> f64 {
	if pts.len() < 3 {
		return 0.0;
	}
	let mut n = DVec3::ZERO;
	for i in 0..pts.len() {
		n += pts[i].cross(pts[(i + 1) % pts.len()]);
	}
	n.length() * 0.5
}
fn dv3(a: [f64; 3]) -> DVec3 {
	DVec3::new(a[0], a[1], a[2])
}

fn profile2d(points: &[[f64; 2]]) -> Vec<DVec2> {
	points.iter().map(|p| DVec2::new(p[0], p[1])).collect()
}

/// Gate every solid-producing op: an empty result is an `invalid_param` failure
/// (the kernel rejected degenerate input) and a non-valid result is an
/// `invalid_geometry` failure carrying the `Validity` details. A solid is bound
/// to the environment ONLY through this gate.
fn bind_solid(op_id: &str, what: &str, solid: Solid) -> Result<Outcome, OpError> {
	if solid.face_count() == 0 {
		return Err(err(
			ErrorKind::InvalidParam,
			format!("op '{op_id}': {what} produced an empty solid — degenerate input, parameters outside the op's documented domain, or an empty boolean result (e.g. a disjoint intersection); see API.md"),
		));
	}
	let v = kernel_brep::validate(&solid);
	if !v.is_valid() {
		return Err(err(
			ErrorKind::InvalidGeometry,
			format!(
				"op '{op_id}': {what} failed validate(): closed={} manifold={} genus={} euler_characteristic={} shells={} — refusing to bind an invalid solid",
				v.closed, v.manifold, v.genus, v.euler_characteristic, v.shells
			),
		));
	}
	Ok(Outcome { value: Some(EnvValue::Solid(solid)), measures: None, file: None })
}

/// `import_step` in `tolerant` mode: the kernel's tolerant importer, whose
/// receipt — every solid of the file with its product name, status and placed
/// envelope; every skip and repair with the entity id and verbatim reason —
/// becomes the measures, and the compound of the imported solids binds. Nothing
/// imported is a loud `invalid_geometry` whose message carries the counts and
/// the first reasons (a bound solid is what the op promises; the envelope census
/// is a measure of a *successful* import, never a substitute for one).
fn import_step_tolerant_op(op_id: &str, path: &Path, text: &str) -> Result<Outcome, OpError> {
	use kernel_brep::{ImportEvent, SolidStatus};
	let imp = kernel_brep::import_step_tolerant(text).map_err(|e| {
		let kind = match &e {
			StepError::Topology(_) => ErrorKind::InvalidGeometry,
			_ => ErrorKind::InvalidParam,
		};
		err(kind, format!("op '{op_id}': import_step '{}' (tolerant): {e}", path.display()))
	})?;
	let v3 = |v: kernel_core::math::DVec3| json!([v.x, v.y, v.z]);
	let event = |e: &ImportEvent| json!({ "entity": e.entity, "kind": e.kind, "solid": e.solid, "reason": e.reason });
	let solids: Vec<Value> = imp
		.solids
		.iter()
		.map(|s| {
			let mut o = json!({
				"name": s.name,
				"path": s.path,
				"entity": s.entity,
				"status": s.status.as_str(),
				"bbox_min": v3(s.bbox_min),
				"bbox_max": v3(s.bbox_max),
				"bbox_source": s.bbox_source,
				"faces": s.faces,
				"faces_repaired": s.faces_repaired,
				"faces_skipped": s.faces_skipped,
			});
			if let Some(reason) = &s.reason {
				o["reason"] = json!(reason);
			}
			o
		})
		.collect();
	let total = imp.solids.len();
	let imported = imp.solids.iter().filter(|s| s.status == SolidStatus::Imported).count();
	let faces_skipped = imp.skipped.iter().filter(|e| e.kind == "ADVANCED_FACE").count();
	let faces_repaired = imp.repaired.iter().filter(|e| e.kind == "ADVANCED_FACE").count();
	let Some(solid) = imp.solid else {
		let first: Vec<String> =
			imp.skipped.iter().take(5).map(|e| format!("#{} {} ({}): {}", e.entity, e.kind, e.solid, e.reason)).collect();
		return Err(err(
			ErrorKind::InvalidGeometry,
			format!(
				"op '{op_id}': import_step '{}' (tolerant): none of the {total} solid(s) could be imported — {} skip(s), {} repair(s); first skips: {}",
				path.display(),
				imp.skipped.len(),
				imp.repaired.len(),
				first.join(" | ")
			),
		));
	};
	let v = kernel_brep::validate(&solid);
	let measures = json!({
		"source": "step",
		"mode": "tolerant",
		"shells": v.shells,
		"genus": v.genus,
		"faces": solid.face_count(),
		"volume": kernel_brep::volume(&solid),
		"freeform_faces": imp.freeform.len(),
		"uncertainty_mm": imp.uncertainty,
		"solids_total": total,
		"solids_imported": imported,
		"solids_skipped": total - imported,
		"faces_skipped": faces_skipped,
		"faces_repaired": faces_repaired,
		"solids": solids,
		"skipped": imp.skipped.iter().map(event).collect::<Vec<Value>>(),
		"repaired": imp.repaired.iter().map(event).collect::<Vec<Value>>(),
	});
	Ok(Outcome { measures: Some(measures), ..bind_solid(op_id, "import_step", solid)? })
}

/// Gate a pattern op's instance count: 2..=[`MAX_PATTERN_COUNT`] (the structural
/// `check_limits` pass already rejected larger counts before dispatch — this arm
/// repeats the ceiling so the invariant does not depend on field-name matching)
/// AND `count × per-clone face count` within the union fold's face budget.
fn pattern_guard(op_id: &str, what: &str, count: usize, clone_faces: usize) -> Result<(), OpError> {
	if count < 2 {
		return Err(err(
			ErrorKind::InvalidParam,
			format!("op '{op_id}': {what}: count must be at least 2 (a 1-pattern is a no-op — use the input id directly)"),
		));
	}
	if count as u64 > MAX_PATTERN_COUNT {
		return Err(err(
			ErrorKind::InvalidParam,
			format!("op '{op_id}': {what}: count {count} exceeds the safety cap {MAX_PATTERN_COUNT} — rejected before allocation"),
		));
	}
	let total = count.saturating_mul(clone_faces);
	if total > MAX_PATTERN_FACES {
		return Err(err(
			ErrorKind::InvalidParam,
			format!("op '{op_id}': {what}: count {count} × {clone_faces} faces per clone = {total} faces exceeds the pattern budget {MAX_PATTERN_FACES} — pattern a simpler solid or reduce the count"),
		));
	}
	Ok(())
}

/// Distance from `p` to the segment `a → b`.
fn point_segment_distance(p: DVec3, a: DVec3, b: DVec3) -> f64 {
	let ab = b - a;
	let len2 = ab.length_squared();
	if len2 <= f64::EPSILON {
		return (p - a).length();
	}
	let t = ((p - a).dot(ab) / len2).clamp(0.0, 1.0);
	(a + ab * t - p).length()
}

/// The named edge of `solid` nearest to `witness` plus its distance, or `None`
/// when the solid carries no edge names at all.
fn nearest_named_edge(solid: &Solid, witness: DVec3) -> Option<(kernel_brep::EdgeName, f64)> {
	let mut best: Option<(kernel_brep::EdgeName, f64)> = None;
	for e in solid.edges() {
		let Some(name) = solid.edge_name(e) else { continue };
		let he = *solid.half_edge(solid.edge(e).half_edge);
		let a = solid.position(he.origin);
		let b = solid.position(solid.half_edge(he.next).origin);
		let d = point_segment_distance(witness, a, b);
		if best.is_none_or(|(_, bd)| d < bd) {
			best = Some((name, d));
		}
	}
	best
}

/// Diagonal length of the solid's vertex bounding box (0 for an empty solid).
fn bbox_diagonal(solid: &Solid) -> f64 {
	let mut min = DVec3::splat(f64::INFINITY);
	let mut max = DVec3::splat(f64::NEG_INFINITY);
	for i in 0..solid.vertex_count() as u32 {
		let p = solid.position(kernel_brep::VertexId(i));
		min = min.min(p);
		max = max.max(p);
	}
	if min.x > max.x {
		return 0.0;
	}
	(max - min).length()
}

/// Resolve a fillet/chamfer witness to the nearest named edge, enforcing the
/// max-distance guard (default: 10% of the bounding-box diagonal) so a witness
/// that matches nothing is a structured failure, not a far-away surprise edge.
///
/// Returns the chosen [`kernel_brep::EdgeName`] together with the witness→edge
/// distance and the limit that was in effect — the raw material for the
/// `resolved_edge` receipt (which edge a spatial witness actually latched, and
/// how close the match was). Selection and the guard are unchanged; the extra
/// return values are only *recorded*, never acted on here.
fn witness_edge(
	op_id: &str,
	solid: &Solid,
	witness: DVec3,
	max_distance: Option<f64>,
) -> Result<(kernel_brep::EdgeName, f64, f64), OpError> {
	let Some((name, distance)) = nearest_named_edge(solid, witness) else {
		return Err(err(
			ErrorKind::FeatureFailed,
			format!("op '{op_id}': the solid carries no named edges — fillet/chamfer needs primitive or boolean provenance"),
		));
	};
	let limit = max_distance.unwrap_or(0.1 * bbox_diagonal(solid));
	if distance > limit {
		return Err(err(
			ErrorKind::FeatureFailed,
			format!(
				"op '{op_id}': witness [{}, {}, {}] matched no edge — nearest edge is {distance:.3} mm away (limit {limit:.3}; pass max_distance to widen)",
				witness.x, witness.y, witness.z
			),
		));
	}
	Ok((name, distance, limit))
}

/// Serialize a [`kernel_brep::FaceName`] exactly as kernel-model's
/// `edge_name_serde` does (operand variant name + `source_face`), so the
/// stateless op layer and the durable feature layer describe the same face the
/// same way and their names are directly comparable.
fn face_name_json(f: kernel_brep::FaceName) -> Value {
	let operand = match f.operand {
		kernel_brep::FaceSource::Primitive => "Primitive",
		kernel_brep::FaceSource::OperandA => "OperandA",
		kernel_brep::FaceSource::OperandB => "OperandB",
	};
	json!({ "operand": operand, "source_face": f.source_face })
}

/// The op-result measures carrying the `resolved_edge` receipt a
/// witness-selecting op leaves behind: the canonical face-pair
/// [`kernel_brep::EdgeName`] the spatial witness actually latched, plus how
/// close the match was and the limit in effect. Its purpose is detection — a
/// parameter sweep can compare this identity across candidates and catch a
/// witness silently jumping to a different edge. Selection is unchanged; this
/// only records what was chosen.
fn resolved_edge_measures(name: kernel_brep::EdgeName, witness_distance: f64, max_distance: f64) -> Value {
	json!({
		"resolved_edge": {
			"faces": [face_name_json(name.faces[0]), face_name_json(name.faces[1])],
			"witness_distance": witness_distance,
			"max_distance": max_distance,
		}
	})
}

/// Map a kernel [`FilletError`] to a structured op error.
fn map_fillet_error(op_id: &str, what: &str, e: FilletError) -> OpError {
	match e {
		FilletError::BadRadius => err(ErrorKind::InvalidParam, format!("op '{op_id}': {what}: radius must be positive and finite")),
		FilletError::RadiusTooLarge => err(
			ErrorKind::FeatureFailed,
			format!("op '{op_id}': {what}: the radius does not fit within the adjacent faces"),
		),
		FilletError::EdgeNotFound => err(ErrorKind::FeatureFailed, format!("op '{op_id}': {what}: the selected edge no longer resolves")),
		FilletError::EdgeAmbiguous => err(
			ErrorKind::FeatureFailed,
			format!("op '{op_id}': {what}: the edge name resolves to several fragments — move the witness closer to one"),
		),
		// One kernel variant covers every scope refusal (concave junction, curved
		// wall, non-trivalent corner, …), so the message states the WHOLE
		// supported scope and calls out the most common trap: a blind-agent
		// live-fire hit a concave junction and was told the edge "is not
		// straight/perpendicular" when it was both — the real reason was
		// convexity. Verified: chamfer_edge_near shares the convexity check, so
		// it is NOT offered as the concave alternative.
		FilletError::Unsupported => err(
			ErrorKind::FeatureFailed,
			format!(
				"op '{op_id}': {what}: the edge near the witness is outside the supported scope — supported: CONVEX straight edges between two planar faces (any convex dihedral angle, simple 3-face corners at both ends) via fillet_edge_near/chamfer_edge_near, and convex circular rims via fillet_circular_rim. Concave junctions (inside corners, where the round would ADD material) are out of scope for BOTH fillet_edge_near and chamfer_edge_near — model the cove explicitly instead: difference a cylinder from a corner bar to leave a quarter-round strip, then union it into the junction"
			),
		),
	}
}

/// Map a kernel [`SketchError`] to a structured op error.
fn map_sketch_error(op_id: &str, what: &str, e: SketchError) -> OpError {
	let reason = match e {
		SketchError::Degenerate => "the profile is degenerate (fewer than 3 points, or it encloses no area)",
		SketchError::NotClosed => "the segments/arcs do not form a single closed loop",
		SketchError::EmptySolid => "the sweep produced no solid (zero height, or radii outside the revolve domain)",
	};
	err(ErrorKind::SketchFailed, format!("op '{op_id}': {what}: {reason}"))
}

/// Map a kernel [`HoleError`] to a structured op error (every variant is a bad
/// or out-of-table parameter, so the kind is `invalid_param`; the message
/// carries the kernel's precise reason).
fn map_hole_error(op_id: &str, what: &str, e: HoleError) -> OpError {
	err(ErrorKind::InvalidParam, format!("op '{op_id}': {what}: {e} — see API.md for the supported table sizes"))
}

/// Map a kernel [`AdmissionError`] to a structured op error: gate failures
/// (build / validity / determinism at a sample) are `admission_rejected`, file
/// writes are `io`, and meta/format problems are `invalid_param` — each with
/// the kernel's precise message (gate messages name the failing sample).
fn map_admission_error(op_id: &str, e: AdmissionError) -> OpError {
	let kind = match &e {
		AdmissionError::GateBuildFailed { .. } | AdmissionError::GateInvalid { .. } | AdmissionError::GateNondeterministic { .. } => {
			ErrorKind::AdmissionRejected
		}
		AdmissionError::Io { .. } => ErrorKind::Io,
		_ => ErrorKind::InvalidParam,
	};
	err(kind, format!("op '{op_id}': library_add: {e}"))
}

/// Map a kernel [`LibraryError`] to a structured op error: the dependents
/// refusal keeps its own machine-matchable kind, I/O stays `io`, and the rest
/// (unknown name/version/parameter, out-of-range value, corrupt index/part)
/// is `invalid_param` with the kernel's precise message.
fn map_library_error(op_id: &str, what: &str, e: LibraryError) -> OpError {
	let kind = match &e {
		LibraryError::DependentsExist { .. } => ErrorKind::DependentsExist,
		LibraryError::Io { .. } | LibraryError::AsmUnreadable { .. } => ErrorKind::Io,
		_ => ErrorKind::InvalidParam,
	};
	err(kind, format!("op '{op_id}': {what}: {e}"))
}

/// Open (creating on demand) the library at `dir`, resolved like input paths
/// (confined under `--out-dir` — absolute paths and `..` are refused).
fn open_library(op_id: &str, out_dir: &Path, dir: &str) -> Result<Library, OpError> {
	Library::open(resolve_input_path(op_id, out_dir, dir)?).map_err(|e| map_library_error(op_id, "library", e))
}

/// Translate the JSON `meta` of `library_add` into the kernel's [`EntryMeta`].
fn to_kernel_meta(meta: LibraryMetaSpec) -> EntryMeta {
	EntryMeta {
		name: meta.name,
		version: meta.version,
		category: meta.category,
		tags: meta.tags,
		description: meta.description,
		provenance: Provenance {
			author: meta.provenance.author,
			created_with: meta.provenance.created_with,
			date: meta.provenance.date,
		},
		param_interface: meta
			.params
			.into_iter()
			.map(|p| ParamSpec { name: p.name, units: p.units, default: p.default, min: p.min, max: p.max, description: p.description })
			.collect(),
	}
}

/// Resolve the mutually exclusive `depth` (blind) / `through` JSON params of the
/// drilling ops into a [`HoleDepth`].
fn hole_depth(op_id: &str, depth: Option<f64>, through: Option<f64>) -> Result<HoleDepth, OpError> {
	match (depth, through) {
		(Some(d), None) => Ok(HoleDepth::Blind(d)),
		(None, Some(t)) => Ok(HoleDepth::Through(t)),
		_ => Err(err(
			ErrorKind::InvalidParam,
			format!("op '{op_id}': exactly one of 'depth' (blind hole) or 'through' (through-hole material span) is required"),
		)),
	}
}

/// Translate the JSON fit series into the kernel's [`holes::Fit`].
fn to_kernel_fit(fit: FitSpec) -> holes::Fit {
	match fit {
		FitSpec::Close => holes::Fit::Close,
		FitSpec::Medium => holes::Fit::Medium,
		FitSpec::Coarse => holes::Fit::Coarse,
	}
}

/// The JSON name of a fit series, for echoing in measures.
fn fit_name(fit: FitSpec) -> &'static str {
	match fit {
		FitSpec::Close => "close",
		FitSpec::Medium => "medium",
		FitSpec::Coarse => "coarse",
	}
}

/// The ISO/DIN table row a hole-wizard cut used, echoed as measures so a caller
/// can pose mating hardware without reading the kernel source (FRICTION #9).
/// Call only after the cut succeeded (which proves `m` is in the table).
fn metric_spec_row(m: f64) -> &'static holes::MetricHoleSpec {
	holes::metric_hole_spec(m).expect("the cut succeeded, so the size is in the table")
}

/// The blind/through depth facts of a drill-style cut, for measures.
fn depth_measures(measures: &mut serde_json::Map<String, Value>, d: f64, dep: HoleDepth) {
	match dep {
		HoleDepth::Blind(depth) => {
			measures.insert("kind".into(), json!("blind"));
			measures.insert("depth".into(), json!(depth));
			// the 118° point extends past the full-diameter depth
			measures.insert("point_depth".into(), json!(depth + holes::drill_tip_height(d)));
		}
		HoleDepth::Through(span) => {
			measures.insert("kind".into(), json!("through"));
			measures.insert("through".into(), json!(span));
		}
	}
}

/// Structured error for a catalog part size outside its standard's table.
fn size_err(op_id: &str, what: &str, standard: &str, m: f64, supported: &str) -> OpError {
	err(ErrorKind::InvalidParam, format!("op '{op_id}': {what}: M{m} is not in the {standard} table (supported: {supported})"))
}

/// The fastener tables (ISO 4017 / ISO 4032 / ISO 7089 / DIN 912 / ISO 10642 /
/// DIN 985 / ISO 261 coarse) share rows.
const FASTENER_SIZES: &str = "M3, M4, M5, M6, M8, M10, M12, M16";

/// The M3–M12 screw tables (ISO 7380 / DIN 916).
const SCREW_SIZES_M3_M12: &str = "M3, M4, M5, M6, M8, M10, M12";

/// The small-thread tables (heat-set inserts, hex standoffs).
const SMALL_SIZES_M2_M6: &str = "M2, M2.5, M3, M4, M5, M6";

/// DIN 471 external circlip shaft diameters.
const DIN471_SIZES: &str = "Ø8, 10, 12, 15, 20, 25, 30";

/// DIN 472 internal circlip bore diameters.
const DIN472_SIZES: &str = "Ø16, 20, 22, 26, 32, 35, 42, 47";

/// The supported AS568 dash numbers (see `kernel_model::parts::as568_spec`).
const AS568_DASHES: &str = "10, 12, 14, 16, 18, 20, 110, 112, 115, 120, 210, 214, 218, 222, 325";

/// Stocked metric O-ring cord cross-sections (see `kernel_model::parts::metric_cord_gland`).
const METRIC_CORD_SIZES: &str = "Ø1, 1.5, 1.78, 2, 2.5, 2.62, 3, 3.53, 4, 5, 5.33, 6, 7";

/// Jaw-coupling body sizes (see `kernel_model::parts::jaw_coupling_spec`).
const JAW_COUPLING_SIZES: &str = "20 (L25), 25 (L30), 30 (L35), 40 (L50)";

/// Stocked set-screw rigid-coupling bores.
const SET_SCREW_COUPLING_BORES: &str = "Ø4, 5, 6, 6.35, 8, 10, 12";

/// Stocked clamp-coupling bores.
const CLAMP_COUPLING_BORES: &str = "Ø4, 5, 6, 8, 10, 12";

/// NEMA stepper frames in the table (see `kernel_model::parts::nema_spec`).
const NEMA_FRAMES: &str = "17, 23";

/// Hobby-servo models in the table (see `kernel_model::parts::servo_spec`).
const SERVO_MODELS: &str = "sg90, mg996r";


/// Snap rotation-matrix entries that are pure float dirt to exact 0 / ±1.
///
/// An axis-permutation rotation (90/180/270° about a coordinate axis, 120°
/// about [1,1,1], …) SHOULD be an exact signed permutation matrix, but the
/// axis-angle construction leaves ~1e-16 residue in the "zero" entries. That
/// residue is what turned exactly-coplanar face pairs into near-coplanar
/// limbo inside the boolean arrangement: a prism posed by the [1,1,1]/120°
/// permutation and abutting a hole wall failed `union` with
/// `invalid_geometry` while the identical axis-aligned box unioned fine
/// (friction folding_book_stand F1/F3, 2026-08-27). Entries within 1e-12 of
/// {0, ±1} are snapped; a genuinely oblique rotation has no such entries and
/// passes through unchanged.
fn snap_rotation(m: DAffine3) -> DAffine3 {
	let snap = |v: f64| {
		if v.abs() < 1e-12 {
			0.0
		} else if (v - 1.0).abs() < 1e-12 {
			1.0
		} else if (v + 1.0).abs() < 1e-12 {
			-1.0
		} else {
			v
		}
	};
	let mut out = m;
	let m3 = &mut out.matrix3;
	for col in [&mut m3.x_axis, &mut m3.y_axis, &mut m3.z_axis] {
		col.x = snap(col.x);
		col.y = snap(col.y);
		col.z = snap(col.z);
	}
	out
}

// --- Tessellation & file helpers ---------------------------------------------------

/// A manufacturing export must bound one unambiguous solid volume: closed,
/// consistently oriented, free of bow-tie vertices, collapsed triangles, and
/// non-adjacent triangle contacts/overlaps.
fn manufacturing_ready(mesh: &Mesh, report: &MeshReport) -> bool {
	report.watertight && report.degenerate_triangles == 0 && mesh.self_intersection_witness().is_none()
}

/// Mesh a solid on the exact adaptive path only when the resulting triangles are
/// manufacturing-ready; otherwise use the voxel heal. Returns the mesh, the
/// route taken (`"exact"` / `"voxel_healed"`), and the heal voxel actually used
/// (= the requested voxel unless the heal budget coarsened it; meaningful only
/// on the healed route).
pub(crate) fn solid_mesh(solid: &Solid, tol: f64, voxel: f64) -> (Mesh, &'static str, f64) {
	let exact = kernel_brep::tessellate_adaptive_tol(solid, tol);
	if manufacturing_ready(&exact, &check_mesh(&exact)) {
		(exact, "exact", voxel)
	} else {
		let heal_voxel = heal_voxel_for_budget(solid, voxel);
		(watertight_mesh(solid, heal_voxel as f32), "voxel_healed", heal_voxel)
	}
}

/// The heal voxel that keeps the winding-number lattice inside the heal's
/// TIME budget. The mesher's own [`kernel_core::mesher::MAX_LATTICE_CELLS`]
/// (2²⁸) is a MEMORY bound: a heal well under it — a 160×140×20 mm body at
/// voxel 0.3 is ~19M cells — still costs one winding-number SDF traversal per
/// cell and ground for many minutes with no feedback, indistinguishable from a
/// hang (friction folding_book_stand F4, 2026-08-27). 2²² cells (~4M) keeps
/// the worst heal around a minute; the receipt reports the voxel used, so the
/// coarsening is on the record, never silent.
fn heal_voxel_for_budget(solid: &Solid, voxel: f64) -> f64 {
	const HEAL_CELL_BUDGET: f64 = (1u64 << 22) as f64;
	let Some(b) = kernel_brep::measure::bounding_box(solid) else {
		return voxel;
	};
	let s = b.max - b.min;
	// pad + margins mirrored from the mesher's lattice sizing
	let cells = |vs: f64| {
		let g = |d: f64| (d + 4.0 * vs) / vs + 3.0;
		g(s.x) * g(s.y) * g(s.z)
	};
	if cells(voxel) <= HEAL_CELL_BUDGET {
		return voxel;
	}
	let mut vs = voxel;
	while cells(vs) > HEAL_CELL_BUDGET {
		vs *= 1.05;
	}
	(vs * 100.0).ceil() / 100.0
}

/// Join `file` onto the input base directory for READING — the input twin of
/// [`resolve_path`], without creating directories. Confined exactly like output
/// paths: absolute paths and any `..` component are refused (audit V1), so a
/// program can only read files UNDER its input base.
pub(crate) fn resolve_input_path(op_id: &str, input_base: &Path, file: &str) -> Result<PathBuf, OpError> {
	confined_join(op_id, input_base, file)
}

/// [`resolve_input_path`] with the write-side fallback that heals the T4
/// path-root asymmetry (campaign friction, 7/10 campaigns): a program that
/// `export_step`s a file (which lands under `--out-dir`) and then imports it
/// back used to fail with `io` unless `--out-dir` happened to BE the program's
/// directory — writes resolved against one root, reads against another.
/// Resolution order, both roots confined: the program's own directory first
/// (relocatable program-relative inputs keep priority), then `--out-dir` iff
/// the file exists there and not beside the program. The error on a total miss
/// names BOTH tried roots so the operator sees where the engine looked.
pub(crate) fn resolve_input_or_out(
	op_id: &str,
	input_base: &Path,
	out_dir: &Path,
	file: &str,
) -> Result<PathBuf, OpError> {
	let primary = confined_join(op_id, input_base, file)?;
	if primary.exists() || input_base == out_dir {
		return Ok(primary);
	}
	let fallback = confined_join(op_id, out_dir, file)?;
	if fallback.exists() {
		return Ok(fallback);
	}
	Err(err(
		ErrorKind::Io,
		format!(
			"op '{op_id}': cannot read '{file}': not found beside the program ('{}') nor under --out-dir ('{}')",
			primary.display(),
			fallback.display()
		),
	))
}

// --- Allocation caps (audit V3) -----------------------------------------------------------------
// An agent request must not OOM or hang the shared process. These ceilings sit FAR above any real
// design (a smooth primitive is ~256 facets; a large voxel grid is ~256³ = 16.7M) and FAR below the
// point where a `usize` count would exhaust memory. The check is a cheap structural pass on the raw
// op JSON, so it covers every op carrying one of these fields uniformly and rejects BEFORE any
// allocation with a matchable `InvalidParam`.
const MAX_SEGMENTS: u64 = 8_192; // facet count on any primitive / revolve / arc
const MAX_PATTERN: u64 = 100_000; // repeated-feature instance count
const MAX_GRID_CELLS: u64 = 50_000_000; // voxel-grid cell product (~200 MB as f32)
const MAX_PATTERN_COUNT: u64 = 500; // solid-clone count of linear_pattern / polar_pattern
const MAX_PATTERN_FACES: usize = 100_000; // count × per-clone faces budget of the pattern union

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

/// Confine an agent-supplied path to the sandbox `base`: reject absolute paths and any
/// `..` / root / drive-prefix component so a work-order can only reach files UNDER the
/// output (or input) directory. Existing symlink components are also refused: a
/// lexical `base/link/file` check is not confinement when `link -> /outside`.
fn confined_join(op_id: &str, base: &Path, file: &str) -> Result<PathBuf, OpError> {
	// An EMPTY base is the current directory: `Path::parent()` of a bare
	// program filename ("part.json") yields "", and canonicalizing "" fails —
	// which broke every campaign whose Reproducing invokes `run <prog>.json`
	// from inside programs/ (measured on the cleat's own README commands).
	let base = if base.as_os_str().is_empty() { Path::new(".") } else { base };
	let rel = Path::new(file);
	if rel.is_absolute() {
		return Err(err(
			ErrorKind::InvalidParam,
			format!("op '{op_id}': path '{file}' must be relative to the sandbox (absolute paths are not allowed)"),
		));
	}
	for comp in rel.components() {
		match comp {
			Component::Normal(_) | Component::CurDir => {}
			Component::ParentDir => {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': path '{file}' must not contain '..' (it would escape the sandbox)"),
				));
			}
			Component::RootDir | Component::Prefix(_) => {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': path '{file}' must be a plain relative path (no root or drive prefix)"),
				));
			}
		}
	}
	let canonical_base = fs::canonicalize(base).map_err(|e| {
		err(ErrorKind::Io, format!("op '{op_id}': cannot canonicalize sandbox '{}': {e}", base.display()))
	})?;
	let mut current = canonical_base.clone();
	for comp in rel.components() {
		if let Component::Normal(name) = comp {
			current.push(name);
			match fs::symlink_metadata(&current) {
				Ok(meta) if meta.file_type().is_symlink() => {
					return Err(err(
						ErrorKind::InvalidParam,
						format!("op '{op_id}': path '{file}' crosses a symbolic link at '{}' — sandbox symlinks are refused", current.display()),
					));
				}
				Ok(_) => {}
				Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
				Err(e) => {
					return Err(err(ErrorKind::Io, format!("op '{op_id}': cannot inspect '{}': {e}", current.display())));
				}
			}
		}
	}
	Ok(canonical_base.join(rel))
}

pub(crate) fn resolve_path(op_id: &str, out_dir: &Path, file: &str) -> Result<PathBuf, OpError> {
	fs::create_dir_all(out_dir)
		.map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot create sandbox '{}': {e}", out_dir.display())))?;
	let path = confined_join(op_id, out_dir, file)?;
	if let Some(parent) = path.parent() {
		if !parent.as_os_str().is_empty() {
			std::fs::create_dir_all(parent)
				.map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot create directory '{}': {e}", parent.display())))?;
		}
	}
	Ok(path)
}

/// Mesh + watertightness gate + write for the STL/3MF export ops.
fn export_mesh(
	op_id: &str,
	solid: &Solid,
	tol: f64,
	voxel: f64,
	out_dir: &Path,
	file: &str,
	format: &'static str,
) -> Result<Outcome, OpError> {
	if !(tol.is_finite() && tol > 0.0) {
		return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': tol must be a positive chord tolerance in mm")));
	}
	if !(voxel.is_finite() && voxel > 0.0) {
		return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': voxel must be a positive voxel size in mm")));
	}
	let (mut mesh, route, heal_voxel) = solid_mesh(solid, tol, voxel);
	// An EMPTY healed mesh is the dual-contour mesher's only refusal channel —
	// it means the heal never ran (its lattice would blow the cell budget), not
	// that the geometry healed to nothing. Letting it fall through to the
	// counter-based refusal below produces the worst message in the engine:
	// "not manufacturing-ready: boundary_edges=0, …, self_intersections=0" with
	// every counter zero. Name the real cause and the fix instead.
	if route == "voxel_healed" && mesh.triangle_count() == 0 {
		let budget = kernel_core::mesher::MAX_LATTICE_CELLS;
		let (extent, vmin) = match kernel_brep::measure::bounding_box(solid) {
			Some(b) => {
				let s = b.max - b.min;
				// Smallest voxel whose heal lattice fits the budget for this
				// part's extent (pad + 3-point margins mirrored from the
				// mesher), with 5% headroom, coarsened to 2 decimals.
				let fits = |vs: f64| {
					let g = |d: f64| (d + 4.0 * vs) / vs + 3.0;
					g(s.x) * g(s.y) * g(s.z) <= budget
				};
				let mut vs = (s.x * s.y * s.z / budget).cbrt() * 1.05;
				while !fits(vs) {
					vs *= 1.05;
				}
				(format!("{:.0}×{:.0}×{:.0} mm", s.x, s.y, s.z), (vs * 100.0).ceil() / 100.0)
			}
			None => ("unbounded".into(), voxel),
		};
		return Err(err(
			ErrorKind::InvalidGeometry,
			format!(
				"op '{op_id}': the exact tessellation at tol {tol} mm is not manufacturing-ready, and the voxel heal cannot run at voxel {voxel} mm — this part's {extent} bounds need a lattice over the mesher's {budget:.0}-cell budget, so the heal returns nothing. Re-export with voxel ≥ {vmin} mm, or at a tol where the exact route is manufacturing-ready",
			),
		));
	}
	// The implicit mesher can emit near-coincident vertices on neighbouring cells.
	// Normalize the healed mesh with the same weld used by STL round-trip import so
	// the in-memory gate checks the topology that downstream readers reconstruct.
	if route == "voxel_healed" {
		mesh.weld(1e-4);
		mesh.compute_normals();
	}
	let mesh_report = check_mesh(&mesh);
	let proper_self_intersections = mesh.self_intersection_witness().map_or(0, |witness| witness.pairs);
	// Route-aware refusal. The EXACT route promises an arrangement-exact solid,
	// so any self-intersection there is a lie worth refusing. The VOXEL-HEALED
	// route promises voxel-accurate closure only — dual-contoured TPMS/lattice
	// output legitimately carries crossing slivers that slicers resolve by
	// covered volume, so crossings REPORT (see `self_intersections` +
	// `manufacturing_ready` in the receipt, both `require`-gateable) while true
	// breakage (open edges, non-manifold, degenerate triangles) still refuses.
	let route_ready = if route == "voxel_healed" {
		mesh_report.watertight && mesh_report.degenerate_triangles == 0
	} else {
		manufacturing_ready(&mesh, &mesh_report)
	};
	if !route_ready {
		return Err(err(
			ErrorKind::InvalidGeometry,
			format!(
				"op '{op_id}': mesh is not manufacturing-ready even after the voxel heal (voxel {voxel} mm): boundary_edges={}, non_manifold_edges={}, non_orientable_edges={}, non_manifold_vertices={}, degenerate_triangles={}, self_intersections={} — refusing export",
				mesh_report.boundary_edges,
				mesh_report.non_manifold_edges,
				mesh_report.non_orientable_edges,
				mesh_report.non_manifold_vertices,
				mesh_report.degenerate_triangles,
				proper_self_intersections,
			),
		));
	}
	let path = resolve_path(op_id, out_dir, file)?;
	let write_result = match format {
		"stl" => mesh.write_stl_binary(&path),
		_ => mesh.write_3mf(&path),
	};
	write_result.map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot write '{}': {e}", path.display())))?;
	// Gate the serialized artifact, not merely the in-memory source mesh. STL is
	// a triangle soup, so reconstruct shared topology with the kernel's standard
	// import weld before applying the same strict manufacturing predicate.
	let mut round_trip = match format {
		"stl" => Mesh::read_stl(&path),
		_ => Mesh::read_3mf(&path),
	}
	.map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot read back '{}': {e}", path.display())))?;
	if format == "stl" {
		round_trip.weld(1e-4);
		round_trip.compute_normals();
	}
	let round_trip_report = check_mesh(&round_trip);
	let round_trip_crossings = round_trip.self_intersection_witness().map_or(0, |witness| witness.pairs);
	let round_trip_ready = if route == "voxel_healed" {
		round_trip_report.watertight && round_trip_report.degenerate_triangles == 0
	} else {
		manufacturing_ready(&round_trip, &round_trip_report)
	};
	if !round_trip_ready {
		let _ = std::fs::remove_file(&path);
		return Err(err(
			ErrorKind::InvalidGeometry,
			format!(
				"op '{op_id}': serialized {format} failed strict round-trip validation: boundary_edges={}, non_manifold_edges={}, non_orientable_edges={}, non_manifold_vertices={}, degenerate_triangles={}, self_intersections={} — artifact removed",
				round_trip_report.boundary_edges,
				round_trip_report.non_manifold_edges,
				round_trip_report.non_orientable_edges,
				round_trip_report.non_manifold_vertices,
				round_trip_report.degenerate_triangles,
				round_trip_crossings,
			),
		));
	}
	// Bind and report the exact mesh that was written. `watertight` uses the
	// strict closed-orientable-2-manifold definition. `manufacturing_ready` is
	// the FULL predicate (incl. zero self-intersections) — on the healed route
	// it can honestly read false while the export still ships, and a campaign
	// that needs the strict bar gates it with `require {manufacturing_ready: true}`.
	Ok(Outcome {
		value: Some(EnvValue::Mesh(round_trip.clone())),
		measures: Some(json!({
			"route": route,
			"heal_voxel_mm": if route == "voxel_healed" { json!(heal_voxel) } else { json!(null) },
			"triangles": round_trip.triangle_count(),
			"manufacturing_ready": manufacturing_ready(&round_trip, &round_trip_report),
			"round_trip_validated": true,
			"watertight": round_trip_report.watertight,
			"watertight_means": "closed, consistently oriented 2-manifold: no boundary, non-manifold, or non-orientable edges and no non-manifold vertices",
			"boundary_edges": round_trip_report.boundary_edges,
			"non_manifold_edges": round_trip_report.non_manifold_edges,
			"non_orientable_edges": round_trip_report.non_orientable_edges,
			"non_manifold_vertices": round_trip_report.non_manifold_vertices,
			"degenerate_triangles": round_trip_report.degenerate_triangles,
			"self_intersections": round_trip_crossings,
			"contacts_or_coplanar_overlaps": round_trip_report.self_intersections,
			"two_manifold": round_trip_report.watertight,
		})),
		file: Some(path.display().to_string()),
	})
}

/// Resolve `file` under `out_dir`, enforce the manufacturing mesh contract,
/// write `.stl` / `.3mf`, then re-read and validate the bytes actually written.
/// Invalid files are removed rather than left behind as plausible artifacts.
pub(crate) fn write_mesh_auto(op_id: &str, out_dir: &Path, file: &str, mesh: &Mesh) -> Result<String, OpError> {
	write_mesh_policy(op_id, out_dir, file, mesh, MeshWritePolicy::Strict)
}

/// [`write_mesh_auto`] for a VOXEL-HEALED result: closure and non-degeneracy
/// still refuse, but proper self-intersections REPORT instead of refusing —
/// dual-contoured TPMS/lattice output legitimately carries crossing slivers
/// that slicers resolve by covered volume, and the receipt carries the count
/// for `require` gating. The exact route keeps the full strict predicate.
pub(crate) fn write_mesh_healed(op_id: &str, out_dir: &Path, file: &str, mesh: &Mesh) -> Result<String, OpError> {
	write_mesh_policy(op_id, out_dir, file, mesh, MeshWritePolicy::Healed)
}

/// Refusal policy for [`write_mesh_policy`], per the writing op's route contract.
#[derive(Clone, Copy, PartialEq)]
pub(crate) enum MeshWritePolicy {
	/// Arrangement-exact print file: the full manufacturing predicate refuses.
	Strict,
	/// Voxel-accurate print file: breakage refuses, crossings report.
	Healed,
	/// Diagnostic scene: IO validated only; quality counters are the caller's receipt.
	Scene,
}

/// [`write_mesh_auto`] for a DIAGNOSTIC SCENE: a merged multi-instance pose
/// snapshot, not a print file. A negative-control scene is DESIGNED to
/// interpenetrate (`overlap_volume > 0` is the whole claim), so refusing it on
/// `proper_self_intersections` would make every failure-attitude export fail
/// the run (campaign friction: SLAS F9). Scene writes skip the
/// manufacturing-readiness refusals; IO and read-back are still validated, and
/// the caller must report the quality counters so the exemption is on the
/// record. Per-instance part files stay on the strict path — only the merged
/// soup is a scene.
pub(crate) fn write_mesh_scene(op_id: &str, out_dir: &Path, file: &str, mesh: &Mesh) -> Result<String, OpError> {
	write_mesh_policy(op_id, out_dir, file, mesh, MeshWritePolicy::Scene)
}

fn write_mesh_policy(op_id: &str, out_dir: &Path, file: &str, mesh: &Mesh, policy: MeshWritePolicy) -> Result<String, OpError> {
	let ready = |m: &Mesh, r: &MeshReport| -> bool {
		match policy {
			MeshWritePolicy::Strict => manufacturing_ready(m, r),
			// Healed = voxel-accurate closure: every EDGE closed, consistently
			// oriented, no degenerate triangles. Non-manifold VERTICES (pinch
			// points at TPMS saddle tangencies) and crossing slivers are
			// characteristic dual-contoured output that slicers resolve by
			// covered volume — they REPORT in the receipt instead of refusing.
			MeshWritePolicy::Healed => {
				r.boundary_edges == 0
					&& r.non_manifold_edges == 0
					&& r.non_orientable_edges == 0
					&& r.degenerate_triangles == 0
			}
			MeshWritePolicy::Scene => true,
		}
	};
	let path = resolve_path(op_id, out_dir, file)?;
	let format = match path.extension().and_then(|e| e.to_str()) {
		Some("stl") => "stl",
		Some("3mf") => "3mf",
		other => {
			return Err(err(
				ErrorKind::InvalidParam,
				format!("op '{op_id}': the output file must end in .stl or .3mf, got extension {other:?}"),
			));
		}
	};
	let mut output_mesh = mesh.clone();
	let mut report = check_mesh(&output_mesh);
	if !ready(&output_mesh, &report) {
		// Imported STL soups and grid meshing can carry near-coincident, unshared
		// vertices. Normalize once before refusing; geometry is not otherwise healed.
		output_mesh.weld(1e-4);
		output_mesh.compute_normals();
		report = check_mesh(&output_mesh);
	}
	if !ready(&output_mesh, &report) {
		return Err(err(
			ErrorKind::InvalidGeometry,
			format!(
				"op '{op_id}': refusing manufacturing output: boundary_edges={}, non_manifold_edges={}, non_orientable_edges={}, non_manifold_vertices={}, degenerate_triangles={}, proper_self_intersections={}",
				report.boundary_edges, report.non_manifold_edges,
				report.non_orientable_edges, report.non_manifold_vertices,
				report.degenerate_triangles,
				output_mesh.self_intersection_witness().as_ref().map_or(0, |w| w.pairs)
			),
		));
	}
	let write_result = if format == "stl" {
		output_mesh.write_stl_binary(&path)
	} else {
		output_mesh.write_3mf(&path)
	};
	write_result.map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot write '{}': {e}", path.display())))?;
	let read_result = if format == "stl" { Mesh::read_stl(&path) } else { Mesh::read_3mf(&path) };
	let mut round_trip = match read_result {
		Ok(mesh) => mesh,
		Err(e) => {
			let _ = fs::remove_file(&path);
			return Err(err(ErrorKind::Io, format!("op '{op_id}': cannot read back '{}': {e}", path.display())));
		}
	};
	round_trip.weld(1e-4);
	round_trip.compute_normals();
	let round_trip_report = check_mesh(&round_trip);
	if !ready(&round_trip, &round_trip_report) {
		let _ = fs::remove_file(&path);
		return Err(err(
			ErrorKind::InvalidGeometry,
			format!(
				"op '{op_id}': serialized manufacturing mesh failed round-trip validation (policy {}): boundary_edges={}, non_manifold_edges={}, non_orientable_edges={}, non_manifold_vertices={}, degenerate_triangles={} — partial artifact removed",
				match policy { MeshWritePolicy::Strict => "strict", MeshWritePolicy::Healed => "healed", MeshWritePolicy::Scene => "scene" },
				round_trip_report.boundary_edges,
				round_trip_report.non_manifold_edges,
				round_trip_report.non_orientable_edges,
				round_trip_report.non_manifold_vertices,
				round_trip_report.degenerate_triangles,
			),
		));
	}
	Ok(path.display().to_string())
}

/// Read a mesh interchange file — `.stl` / `.obj` / `.3mf` / `.ply`, sniffed by
/// extension (the kernel has NO glTF reader) — and ALWAYS weld it (STL and many
/// exporters store an unshared triangle soup; welding recovers shared topology
/// so the `check_mesh` receipt is meaningful). Returns the welded mesh plus the
/// sniffed format name. An unreadable file is `io`; an empty one `invalid_param`.
pub(crate) fn read_mesh_file(op_id: &str, input_base: &Path, out_dir: &Path, file: &str) -> Result<(Mesh, &'static str), OpError> {
	// T4: program-relative first, then --out-dir (a mesh written by an earlier op lands there).
	let path = resolve_input_or_out(op_id, input_base, out_dir, file)?;
	let format = match path.extension().and_then(|e| e.to_str()).map(str::to_ascii_lowercase).as_deref() {
		Some("stl") => "stl",
		Some("obj") => "obj",
		Some("3mf") => "3mf",
		Some("ply") => "ply",
		other => {
			return Err(err(
				ErrorKind::InvalidParam,
				format!("op '{op_id}': mesh file '{file}' has unsupported extension {other:?} — supported: .stl, .obj, .3mf, .ply (the kernel has no glTF reader)"),
			));
		}
	};
	let mut mesh = match format {
		"stl" => Mesh::read_stl(&path),
		"obj" => Mesh::read_obj(&path),
		"3mf" => Mesh::read_3mf(&path),
		_ => Mesh::read_ply(&path),
	}
	.map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot read '{}': {e}", path.display())))?;
	if mesh.triangle_count() == 0 {
		return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': '{file}' contains no triangles")));
	}
	mesh.weld(1e-4); // the kernel's STL-soup weld tolerance (kernel-core convention)
	Ok((mesh, format))
}

/// Append `src`'s triangles onto `dst` as a plain soup extension (no weld — the
/// winding-number heal consumes a soup directly).
fn merge_soup(dst: &mut Mesh, src: &Mesh) {
	let base = dst.positions.len() as u32;
	dst.positions.extend_from_slice(&src.positions);
	for t in src.triangles() {
		dst.push_triangle(base + t[0], base + t[1], base + t[2]);
	}
}

/// The full [`check_mesh`] receipt as report measures — every count, never a
/// summary, so a caller sees exactly what is (and is not) wrong with a mesh.
fn mesh_receipt(m: &mut serde_json::Map<String, Value>, report: &MeshReport) {
	m.insert("watertight".into(), json!(report.watertight));
	m.insert("boundary_edges".into(), json!(report.boundary_edges));
	m.insert("non_manifold_edges".into(), json!(report.non_manifold_edges));
	m.insert("non_orientable_edges".into(), json!(report.non_orientable_edges));
	m.insert("non_manifold_vertices".into(), json!(report.non_manifold_vertices));
	m.insert("degenerate_triangles".into(), json!(report.degenerate_triangles));
	m.insert("self_intersections".into(), json!(report.self_intersections));
}

/// Validate an op's optional explicit `domain` box (`{min, max}`), or `None`
/// when the caller should fall back to the geometry's own bounds.
fn explicit_domain(op_id: &str, domain: &Option<crate::program::DomainSpec>) -> Result<Option<Aabb>, OpError> {
	match domain {
		Some(d) => {
			let lo = Vec3::new(d.min[0] as f32, d.min[1] as f32, d.min[2] as f32);
			let hi = Vec3::new(d.max[0] as f32, d.max[1] as f32, d.max[2] as f32);
			if !(lo.is_finite() && hi.is_finite() && lo.x < hi.x && lo.y < hi.y && lo.z < hi.z) {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': 'domain.min' must be finite and strictly below 'domain.max' on every axis"),
				));
			}
			Ok(Some(Aabb::new(lo, hi)))
		}
		None => Ok(None),
	}
}

/// The finite bounds of a parsed implicit tree, with the same refusal guidance
/// as the `implicit` op: an empty tree (disjoint intersection) and an unbounded
/// one (bare plane, periodic lattice without a shroud, bounds-less `expr_sdf`)
/// are loud `invalid_param`s, never a silent empty/endless lattice.
fn tree_bounds(op_id: &str, node: &Node) -> Result<Aabb, OpError> {
	let b = node.bounds();
	if !b.is_valid() {
		return Err(err(
			ErrorKind::InvalidParam,
			format!("op '{op_id}': the expression tree has empty bounds (e.g. an intersection of disjoint shapes) — nothing to mesh/measure"),
		));
	}
	if !(b.min.is_finite() && b.max.is_finite()) {
		return Err(err(
			ErrorKind::InvalidParam,
			format!("op '{op_id}': the expression tree is unbounded (a bare 'plane', a periodic 'strut_lattice'/'tpms' without a shroud, or a bounds-less 'expr_sdf' leaf) — intersect it with a bounded shape or pass an explicit 'domain'"),
		));
	}
	Ok(b)
}

/// The bind-side receipt of a **voxel-route solid** op (`offset_solid` /
/// `shell_solid` / `solid_from_implicit`): honest route `"voxel"` (the body
/// re-entered the solid environment through a voxel lattice — a FACETED B-rep,
/// accurate to ~`voxel`, never exact), the achieved volume, face count, and the
/// full validity verdict. Compute BEFORE `bind_solid` consumes the solid.
fn voxel_solid_measures(solid: &Solid, voxel: f64) -> Value {
	let v = kernel_brep::validate(solid);
	json!({
		"route": "voxel",
		"faceted": true,
		"voxel": voxel,
		"faces": solid.face_count(),
		"volume": kernel_brep::volume(solid),
		"closed": v.closed,
		"manifold": v.manifold,
		"shells": v.shells,
		"genus": v.genus,
	})
}

/// Reject a voxel lattice over `domain` that would exceed [`MAX_GRID_CELLS`]
/// BEFORE allocating it (the same discipline as `shell` / the density grids).
fn grid_guard(op_id: &str, what: &str, domain: Aabb, voxel: f64) -> Result<(), OpError> {
	let size = domain.size();
	let cells = (f64::from(size.x) / voxel).ceil() * (f64::from(size.y) / voxel).ceil() * (f64::from(size.z) / voxel).ceil();
	if !(cells.is_finite() && cells <= MAX_GRID_CELLS as f64) {
		return Err(err(
			ErrorKind::InvalidParam,
			format!("op '{op_id}': {what}: the voxel lattice would be ≈{cells:.0} cells (bbox/voxel per axis), over the cap {MAX_GRID_CELLS} — use a coarser voxel"),
		));
	}
	Ok(())
}

// --- Modelled ISO threads ---------------------------------------------------------

/// Cap on the helical turns a thread op will loft (the ridge is stitched at
/// 96 stations per turn, so unbounded turns would be an allocation hazard).
const MAX_THREAD_TURNS: f64 = 200.0;

/// Radial crest clearance (mm) of an `export_threaded` INTERNAL cut: the
/// male-profile ridge is enlarged to crest Ø `m + 2 × this` before being
/// subtracted from the bore wall — the documented print-practical female
/// approximation (NOT the ISO D1/D4 basic female form).
const INTERNAL_CREST_CLEARANCE: f64 = 0.2;

/// The ISO 261 coarse pitch for nominal Ø `m`, or a structured error naming the
/// supported table sizes.
fn iso_pitch(op_id: &str, what: &str, m: f64) -> Result<f64, OpError> {
	parts::iso_coarse_pitch(m).ok_or_else(|| size_err(op_id, what, "ISO 261 coarse-pitch", m, FASTENER_SIZES))
}

/// Reject a degenerate or allocation-hostile threaded span before any loft.
fn thread_turns_guard(op_id: &str, what: &str, length: f64, pitch: f64) -> Result<(), OpError> {
	if !(length.is_finite() && length > 0.0 && pitch.is_finite() && pitch > 0.0) {
		return Err(err(
			ErrorKind::InvalidParam,
			format!("op '{op_id}': {what}: length ({length}) and pitch ({pitch}) must be positive and finite"),
		));
	}
	let turns = length / pitch;
	if turns > MAX_THREAD_TURNS {
		return Err(err(
			ErrorKind::InvalidParam,
			format!(
				"op '{op_id}': {what}: {turns:.0} turns (length/pitch) exceeds the cap {MAX_THREAD_TURNS:.0} — the 96-station-per-turn loft would be enormous; thread a shorter span"
			),
		));
	}
	Ok(())
}

// --- Sketch construction --------------------------------------------------------

/// Build and solve the kernel sketch for an `op: "sketch"`, with full index
/// bounds-checking (the kernel solver would panic on an out-of-range PointId).
#[allow(clippy::too_many_arguments)]
fn build_sketch(
	op_id: &str,
	points: &[[f64; 2]],
	segments: &[[usize; 2]],
	arcs: &[crate::program::ArcSpec],
	circles: &[crate::program::CircleSpec],
	constraints: &[ConstraintSpec],
) -> Result<Outcome, OpError> {
	let n = points.len();
	let check = |what: &str, k: usize, indices: &[usize]| -> Result<(), OpError> {
		for &i in indices {
			if i >= n {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': {what} #{k} references point {i}, but the sketch has only {n} points"),
				));
			}
		}
		Ok(())
	};

	let mut sketch = Sketch::new();
	for p in points {
		sketch.add_point(DVec2::new(p[0], p[1]));
	}
	for (k, s) in segments.iter().enumerate() {
		check("segment", k, &[s[0], s[1]])?;
		sketch.add_segment(s[0], s[1]);
	}
	for (k, a) in arcs.iter().enumerate() {
		check("arc", k, &[a.a, a.b, a.center])?;
		sketch.add_arc(a.a, a.b, a.center, a.ccw);
	}
	for (k, c) in circles.iter().enumerate() {
		check("circle", k, &[c.center, c.radius_point])?;
		sketch.add_circle(c.center, c.radius_point);
	}
	for (k, c) in constraints.iter().enumerate() {
		check("constraint", k, &c.point_indices())?;
		sketch.add_constraint(to_kernel_constraint(c));
	}

	let solve = sketch.solve();
	let analysis = sketch.analyze();
	let state = match analysis.state {
		ConstraintState::UnderConstrained => "under_constrained",
		ConstraintState::WellConstrained => "well_constrained",
		ConstraintState::OverConstrained => "over_constrained",
	};
	if !solve.converged {
		return Err(err(
			ErrorKind::SketchFailed,
			format!(
				"op '{op_id}': constraints did not converge (residual {:.3e} after {} iterations, state {state}) — they are conflicting or inconsistent",
				solve.residual, solve.iterations
			),
		));
	}
	Ok(Outcome {
		value: Some(EnvValue::Sketch(sketch)),
		measures: Some(json!({
			"residual": solve.residual,
			"iterations": solve.iterations,
			"converged": solve.converged,
			"dof": analysis.dof,
			"rank": analysis.rank,
			"free_dof": analysis.free_dof,
			"redundant": analysis.redundant,
			"state": state,
		})),
		file: None,
	})
}

/// Translate a JSON constraint into the kernel's [`SketchConstraint`] (degrees →
/// radians at this boundary).
fn to_kernel_constraint(c: &ConstraintSpec) -> SketchConstraint {
	match *c {
		ConstraintSpec::Fixed { point, at } => SketchConstraint::Fixed { point, at: DVec2::new(at[0], at[1]) },
		ConstraintSpec::Coincident { a, b } => SketchConstraint::Coincident { a, b },
		ConstraintSpec::Horizontal { a, b } => SketchConstraint::Horizontal { a, b },
		ConstraintSpec::Vertical { a, b } => SketchConstraint::Vertical { a, b },
		ConstraintSpec::Distance { a, b, distance } => SketchConstraint::Distance { a, b, distance },
		ConstraintSpec::Parallel { a, b, c, d } => SketchConstraint::Parallel { a, b, c, d },
		ConstraintSpec::Perpendicular { a, b, c, d } => SketchConstraint::Perpendicular { a, b, c, d },
		ConstraintSpec::EqualLength { a, b, c, d } => SketchConstraint::EqualLength { a, b, c, d },
		ConstraintSpec::Tangent { line_a, line_b, center, radius_point } => {
			SketchConstraint::Tangent { line_a, line_b, center, radius_point }
		}
		ConstraintSpec::Angle { a, b, c, d, degrees } => SketchConstraint::Angle { a, b, c, d, radians: degrees.to_radians() },
		ConstraintSpec::Symmetric { a, b, line_a, line_b } => SketchConstraint::Symmetric { a, b, line_a, line_b },
	}
}

// --- The op dispatcher ------------------------------------------------------------

/// Execute one parsed op against the environment.
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
		OpKind::AsmInstance { solid, name, translate, rotate, material } => {
			crate::asmops::instance(asm, env, all_ids, op_id, &solid, &name, &translate, &rotate, &material)
		}
		OpKind::AsmInstanceMesh { file, name, translate, rotate, material } => {
			crate::asmops::instance_mesh(asm, input_base, op_id, &file, &name, &translate, &rotate, &material)
		}
		OpKind::AsmMate {
			kind,
			a,
			b,
			a_point,
			b_point,
			a_dir,
			b_dir,
			a_axis_point,
			a_axis_dir,
			b_axis_point,
			b_axis_dir,
			distance,
			degrees,
		} => crate::asmops::mate(
			asm, op_id, &kind, &a, &b, &a_point, &b_point, &a_dir, &b_dir, &a_axis_point, &a_axis_dir, &b_axis_point,
			&b_axis_dir, &distance, &degrees,
		),
		OpKind::AsmMateAxis { a, a_witness, b, b_witness, distance } => {
			crate::asmops::mate_axis(asm, env, all_ids, op_id, &a, a_witness, &b, b_witness, &distance)
		}
		OpKind::AsmMateFace { a, a_witness, b, b_witness, offset } => {
			crate::asmops::mate_face(asm, env, all_ids, op_id, &a, a_witness, &b, b_witness, &offset)
		}
		OpKind::AsmSolve { iterations, max_residual, allow_unconverged } => {
			crate::asmops::solve(asm, op_id, &iterations, &max_residual, allow_unconverged)
		}
		OpKind::AsmContacts { window, tol } => crate::asmops::contacts(asm, env, all_ids, op_id, &window, &tol),
		OpKind::AsmInterferenceVolume { a, b, voxel } => {
			crate::asmops::interference_volume(asm, env, all_ids, op_id, &a, &b, &voxel)
		}
		OpKind::AsmMassProperties {} => crate::asmops::mass_properties(asm, env, all_ids, op_id),
		OpKind::AsmExport { file, parts_dir, tol, voxel } => {
			crate::asmops::export(asm, env, all_ids, op_id, out_dir, &file, &parts_dir, &tol, &voxel)
		}
		OpKind::AsmExportStep { file } => crate::asmops::export_step(asm, env, all_ids, op_id, out_dir, &file),
		OpKind::AsmSave { file, name, parts_dir } => {
			crate::asmops::save(asm, env, all_ids, op_id, out_dir, &file, &name, &parts_dir)
		}
		OpKind::GearTrainPoses { sun_teeth, ring1_teeth, planet_a_teeth, planet_b_teeth, ring2_teeth, n_planets, module, theta_deg } => {
			crate::asmops::gear_train_poses(
				op_id, sun_teeth, ring1_teeth, planet_a_teeth, planet_b_teeth, ring2_teeth, n_planets, module, theta_deg,
			)
		}

		// --- Solid primitives & sweeps ----------------------------------------
		OpKind::Box { min, max } => {
			// Reject a degenerate box (non-positive extent on any axis) up front — an inverted or
			// zero-thickness box is a user error, not something to silently normalize/build.
			if (0..3).any(|i| max[i] <= min[i]) {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': box has non-positive extent — max must exceed min on every axis (min={min:?}, max={max:?})"),
				));
			}
			bind_solid(op_id, "box", kernel_brep::cuboid(dv3(min), dv3(max)))
		}
		OpKind::Cylinder { base, axis, radius, height, segments } => {
			bind_solid(op_id, "cylinder", kernel_brep::cylinder(dv3(base), dv3(axis), radius, height, segments))
		}
		OpKind::Sphere { center, radius, u, v } => bind_solid(op_id, "sphere", kernel_brep::sphere(dv3(center), radius, u, v)),
		OpKind::Cone { base, axis, radius, height, segments, top_radius } => {
			let top = top_radius.unwrap_or(0.0);
			if !(top.is_finite() && top >= 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': top_radius must be a finite non-negative radius in mm")));
			}
			if top == 0.0 {
				return bind_solid(op_id, "cone", kernel_brep::cone(dv3(base), dv3(axis), radius, height, segments));
			}
			if !(radius.is_finite() && radius > 0.0 && height.is_finite() && height != 0.0) {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': a frustum needs a positive finite 'radius' and a non-zero finite 'height'"),
				));
			}
			if (top - radius).abs() <= 1e-12 * radius.abs().max(1.0) {
				return Err(err(
					ErrorKind::InvalidParam,
					format!(
						"op '{op_id}': top_radius {top} equals radius {radius} — that solid is a CYLINDER, not a frustum; use the 'cylinder' op (a cone surface with no apex is not representable)"
					),
				));
			}
			// A frustum is the revolution of the trapezoid (0,0)→(r,0)→(rt,h)→(0,h).
			// Reusing `revolve` is not a shortcut: it is what gives the lateral band
			// its exact `Surface::Cone` tag (and the caps their planes), so
			// `exact_volume` / `mass_properties` / STEP export stay analytic —
			// exactly as they are for the un-truncated `cone`.
			let profile = [DVec2::new(0.0, 0.0), DVec2::new(radius, 0.0), DVec2::new(top, height.abs()), DVec2::new(0.0, height.abs())];
			let solid = kernel_brep::revolve(&profile, segments.max(3));
			if solid.face_count() == 0 {
				return Err(err(
					ErrorKind::InvalidGeometry,
					format!("op '{op_id}': the frustum profile (radius {radius}, top_radius {top}, height {height}) does not revolve to a valid solid"),
				));
			}
			// `revolve` builds about +Z through the origin; place it on the requested
			// base/axis with the same conventions the `cone` op already uses (a
			// negative height puts the small end below the base plane).
			let ax = dv3(axis);
			let Some(dir) = (ax * height.signum()).try_normalize() else {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': axis must be a non-zero finite vector")));
			};
			let b = dv3(base);
			if !b.is_finite() {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': base must be finite")));
			}
			let m = DAffine3::from_translation(b) * DAffine3::from_mat3(align_z_to(dir));
			bind_solid(op_id, "cone", solid.transformed(m))
		}
		OpKind::Torus { center, axis, major, minor, ring_segments, tube_segments } => bind_solid(
			op_id,
			"torus",
			kernel_brep::torus(dv3(center), dv3(axis), major, minor, ring_segments, tube_segments),
		),
		OpKind::Extrude { profile, height } => bind_solid(op_id, "extrude", kernel_brep::extrude(&profile2d(&profile), height)),
		OpKind::ExtrudeWithHoles { outer, holes, height } => {
			let holes: Vec<Vec<DVec2>> = holes.iter().map(|h| profile2d(h)).collect();
			bind_solid(op_id, "extrude_with_holes", kernel_brep::extrude_with_holes(&profile2d(&outer), &holes, height))
		}
		OpKind::ExtrudeTapered { profile, height, draft_deg } => bind_solid(
			op_id,
			"extrude_tapered",
			kernel_brep::extrude_tapered(&profile2d(&profile), height, draft_deg.to_radians()),
		),
		OpKind::Revolve { profile, segments } => {
			bind_solid(op_id, "revolve", kernel_brep::revolve(&profile2d(&profile), segments))
		}
		OpKind::Loft { sections } => {
			let secs: Vec<Vec<DVec3>> = sections.iter().map(|s| s.iter().map(|&p| dv3(p)).collect()).collect();
			match kernel_brep::loft_solid(&secs) {
				Some(s) => bind_solid(op_id, "loft", s),
				None => Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': 'loft' needs ≥2 sections of ≥3 points each, all the same length, with finite coordinates; see API.md"),
				)),
			}
		}
		OpKind::Sweep { profile, path } => {
			let prof: Vec<DVec3> = profile.iter().map(|&p| dv3(p)).collect();
			let pth: Vec<DVec3> = path.iter().map(|&p| dv3(p)).collect();
			match kernel_brep::sweep_solid(&prof, &pth) {
				Some(s) => bind_solid(op_id, "sweep", s),
				None => Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': 'sweep' needs a profile of ≥3 points and a path of ≥2 points, all finite; see API.md"),
				)),
			}
		}

		// --- Sketch --------------------------------------------------------------
		OpKind::Sketch { points, segments, arcs, circles, constraints } => {
			build_sketch(op_id, &points, &segments, &arcs, &circles, &constraints)
		}
		OpKind::SketchExtrude { sketch, height } => {
			let sk = fetch_sketch(env, all_ids, op_id, "sketch", &sketch)?;
			let solid = sk.extrude(height).map_err(|e| map_sketch_error(op_id, "sketch_extrude", e))?;
			bind_solid(op_id, "sketch_extrude", solid)
		}
		OpKind::SketchRevolve { sketch, segments } => {
			let sk = fetch_sketch(env, all_ids, op_id, "sketch", &sketch)?;
			let solid = sk.revolve(segments).map_err(|e| map_sketch_error(op_id, "sketch_revolve", e))?;
			bind_solid(op_id, "sketch_revolve", solid)
		}

		// --- Booleans ----------------------------------------------------------------
		OpKind::Union { a, b } => {
			let sa = fetch_solid(env, all_ids, op_id, "a", &a)?;
			let sb = fetch_solid(env, all_ids, op_id, "b", &b)?;
			bind_solid(op_id, "union", kernel_brep::union(sa, sb))
		}
		OpKind::Difference { a, b } => {
			let sa = fetch_solid(env, all_ids, op_id, "a", &a)?;
			let sb = fetch_solid(env, all_ids, op_id, "b", &b)?;
			bind_solid(op_id, "difference", kernel_brep::difference(sa, sb))
		}
		OpKind::Intersection { a, b } => {
			let sa = fetch_solid(env, all_ids, op_id, "a", &a)?;
			let sb = fetch_solid(env, all_ids, op_id, "b", &b)?;
			bind_solid(op_id, "intersection", kernel_brep::intersection(sa, sb))
		}
		OpKind::UnionAll { input } => {
			if input.len() < 2 {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': union_all needs at least two ids in 'in' (got {})", input.len()),
				));
			}
			// Robustness-aware fold order. A left fold in argument order used to
			// re-arrange the SAME rebuilt face once per contacting operand: four
			// prisms abutting one plate's hole wall died at the third union with
			// `invalid_geometry`, while grouping the mutually-disjoint prisms
			// first and touching the plate once succeeded (friction
			// folding_book_stand F1/F3, 2026-08-27). So operands are folded in
			// ascending order of how many other operands' AABBs they touch —
			// mutually-disjoint operands merge first (a cheap multi-shell
			// union), the touch-everything operand arranges last and once. The
			// result is the same solid (union is associative); ties keep the
			// argument order, so the fold stays deterministic.
			let solids: Vec<&Solid> = input
				.iter()
				.map(|name| fetch_solid(env, all_ids, op_id, "in", name))
				.collect::<Result<_, _>>()?;
			let boxes: Vec<Option<kernel_brep::BoundingBox>> =
				solids.iter().map(|s| kernel_brep::measure::bounding_box(s)).collect();
			let touches = |i: usize, j: usize| -> bool {
				match (&boxes[i], &boxes[j]) {
					(Some(a), Some(b)) => {
						a.min.x <= b.max.x
							&& b.min.x <= a.max.x && a.min.y <= b.max.y
							&& b.min.y <= a.max.y && a.min.z <= b.max.z
							&& b.min.z <= a.max.z
					}
					_ => true, // unknown bounds: treat as touching (conservative)
				}
			};
			let mut order: Vec<usize> = (0..solids.len()).collect();
			let degree: Vec<usize> =
				(0..solids.len()).map(|i| (0..solids.len()).filter(|&j| j != i && touches(i, j)).count()).collect();
			order.sort_by_key(|&i| (degree[i], i));
			let mut acc = kernel_brep::union(solids[order[0]], solids[order[1]]);
			for &i in &order[2..] {
				acc = kernel_brep::union(&acc, solids[i]);
			}
			bind_solid(op_id, "union_all", acc)
		}

		// --- Features & transforms ------------------------------------------------------
		OpKind::FilletEdgeNear { input, witness, radius, max_distance } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let w = dv3(witness);
			let (name, distance, limit) = witness_edge(op_id, s, w, max_distance)?;
			let solid = kernel_brep::fillet_edge_near(s, name, radius, w).map_err(|e| map_fillet_error(op_id, "fillet_edge_near", e))?;
			let mut outcome = bind_solid(op_id, "fillet_edge_near", solid)?;
			outcome.measures = Some(resolved_edge_measures(name, distance, limit));
			Ok(outcome)
		}
		OpKind::ChamferEdgeNear { input, witness, radius, max_distance } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let w = dv3(witness);
			let (name, distance, limit) = witness_edge(op_id, s, w, max_distance)?;
			let solid =
				kernel_brep::chamfer_edge_near(s, name, radius, w).map_err(|e| map_fillet_error(op_id, "chamfer_edge_near", e))?;
			let mut outcome = bind_solid(op_id, "chamfer_edge_near", solid)?;
			outcome.measures = Some(resolved_edge_measures(name, distance, limit));
			Ok(outcome)
		}
		OpKind::FilletCircularRim { input, witness, radius, arc_segments } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid = kernel_brep::fillet_circular_rim(s, dv3(witness), radius, arc_segments).ok_or_else(|| {
				err(
					ErrorKind::FeatureFailed,
					format!(
						"op '{op_id}': no fillable circular rim near witness [{}, {}, {}] — the rim must be a convex cylinder-wall/planar-cap ring and the radius must fit (see API.md)",
						witness[0], witness[1], witness[2]
					),
				)
			})?;
			bind_solid(op_id, "fillet_circular_rim", solid)
		}
		OpKind::Translate { input, offset } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			bind_solid(op_id, "translate", s.transformed(DAffine3::from_translation(dv3(offset))))
		}
		OpKind::RotateZ { input, degrees } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			bind_solid(op_id, "rotate_z", s.transformed(snap_rotation(DAffine3::from_rotation_z(degrees.to_radians()))))
		}
		OpKind::Pose { input, translate, rotate } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			if translate.is_none() && rotate.is_none() {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': pose needs 'translate' and/or 'rotate' — an empty pose would be a no-op"),
				));
			}
			let mut m = DAffine3::IDENTITY;
			if let Some(r) = rotate {
				let Some(axis) = dv3(r.axis).try_normalize() else {
					return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': rotate.axis must be a non-zero finite vector")));
				};
				if !r.degrees.is_finite() {
					return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': rotate.degrees must be finite")));
				}
				let center = dv3(r.center);
				m = DAffine3::from_translation(center)
					* snap_rotation(DAffine3::from_axis_angle(axis, r.degrees.to_radians()))
					* DAffine3::from_translation(-center);
			}
			if let Some(t) = translate {
				m = DAffine3::from_translation(dv3(t)) * m;
			}
			bind_solid(op_id, "pose", s.transformed(m))
		}
		OpKind::RotateX { input, degrees } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			if !degrees.is_finite() {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': degrees must be finite")));
			}
			bind_solid(op_id, "rotate_x", s.transformed(snap_rotation(DAffine3::from_rotation_x(degrees.to_radians()))))
		}
		OpKind::RotateY { input, degrees } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			if !degrees.is_finite() {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': degrees must be finite")));
			}
			bind_solid(op_id, "rotate_y", s.transformed(snap_rotation(DAffine3::from_rotation_y(degrees.to_radians()))))
		}
		OpKind::Mirror { input, plane } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let p = dv3(plane.point);
			let n = dv3(plane.normal);
			if !p.is_finite() {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': plane.point must be finite")));
			}
			// The kernel's `mirrored` silently returns an unchanged clone for a
			// degenerate normal — reject it loudly here instead.
			if !(n.is_finite() && n.length_squared() > f64::EPSILON) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': plane.normal must be a non-zero finite vector")));
			}
			// Orientation-safe by construction: `Solid::mirrored` rebuilds every
			// face loop reversed, so the reflection is a valid outward solid (a raw
			// negative-determinant `transformed` would leave it inside-out).
			bind_solid(op_id, "mirror", s.mirrored(p, n))
		}
		OpKind::LinearPattern { input, count, step } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			pattern_guard(op_id, "linear_pattern", count, s.face_count())?;
			let st = dv3(step);
			if !(st.is_finite() && st.length_squared() > 0.0) {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': linear_pattern: step must be a non-zero finite vector — a zero step stacks every clone onto the original (a coincident-face degeneracy)"),
				));
			}
			let mut acc = s.clone();
			for i in 1..count {
				acc = kernel_brep::union(&acc, &s.transformed(DAffine3::from_translation(st * i as f64)));
			}
			bind_solid(op_id, "linear_pattern", acc)
		}
		OpKind::PolarPattern { input, count, center, axis, step_deg } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			pattern_guard(op_id, "polar_pattern", count, s.face_count())?;
			let Some(ax) = dv3(axis).try_normalize() else {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': polar_pattern: axis must be a non-zero finite vector")));
			};
			let c = dv3(center);
			if !c.is_finite() {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': polar_pattern: center must be finite")));
			}
			let step = step_deg.unwrap_or(360.0 / count as f64);
			if !step.is_finite() || step % 360.0 == 0.0 {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': polar_pattern: step_deg ({step}) must be finite and not a multiple of 360° — coincident clones are degenerate"),
				));
			}
			let mut acc = s.clone();
			for k in 1..count {
				let m = DAffine3::from_translation(c)
					* snap_rotation(DAffine3::from_axis_angle(ax, (step * k as f64).to_radians()))
					* DAffine3::from_translation(-c);
				acc = kernel_brep::union(&acc, &s.transformed(m));
			}
			bind_solid(op_id, "polar_pattern", acc)
		}

		// --- Measures ----------------------------------------------------------------------
		OpKind::Validate { input } => {
			let target = fetch_measurable(env, all_ids, op_id, "in", &input)?;
			// A bound mesh has no B-rep record to validate; report the triangle
			// topology under the same key names, plus the mesh-only counts, and say
			// `source: "mesh"` so no reader mistakes one for the other.
			let s = match target {
				Measurable::Solid(s) => s,
				Measurable::Mesh(m) => {
					// `closed` is CLOSURE — no openings — and nothing else, so it can
					// never contradict the `boundary_edges` printed beside it. It used
					// to be `check_mesh().watertight`, which folds orientability and
					// vertex-manifoldness in, and a mesh with a flipped triangle then
					// reported `closed: false` next to `boundary_edges: 0` in the same
					// receipt. Everything the old `closed` covered still gates through
					// `manifold`, so `valid` (closed AND manifold) is unchanged.
					let r = check_mesh(m);
					let closed = r.boundary_edges == 0 && m.triangle_count() > 0;
					let manifold =
						r.non_manifold_edges == 0 && r.non_orientable_edges == 0 && r.non_manifold_vertices == 0;
					let witness = m.self_intersection_witness();
					let mut out = json!({
						"closed": closed,
						"manifold": manifold,
						"valid": closed && manifold,
						"triangles": m.triangle_count(),
						"boundary_edges": r.boundary_edges,
						"non_manifold_edges": r.non_manifold_edges,
						"non_orientable_edges": r.non_orientable_edges,
						"non_manifold_vertices": r.non_manifold_vertices,
						"geometric_ok": witness.is_none(),
						"source": "mesh",
					});
					if let Some(w) = &witness {
						out["self_intersection"] = self_intersection_json(w);
					}
					return Ok(Outcome::measures(out));
				}
			};
			let v = kernel_brep::validate(s);
			// M2 trust: `geometric_ok` is the geometric-validity flag (no self-intersection),
			// distinct from the topological validity above — a solid can be closed+manifold yet
			// self-overlapping with a silently-wrong volume. self_intersects() tessellates and is
			// O(tri²)-ish, so it is computed here on the EXPLICIT validate op (on demand), not on
			// every bind. false ⇒ measure the fit / re-route; do not trust the volume as exact.
			//
			// A bare `false` is not actionable, and a validity flag nobody can act on
			// is a validity flag everybody learns to ignore (theme T15 — three
			// campaigns shipped `geometric_ok:false` disclosed as "unexplained").
			// When the flag trips, the report now carries the WITNESS: which two
			// triangles cross, where in space, and how many pairs do it.
			let witness = kernel_brep::tessellate_default(s).self_intersection_witness();
			let mut out = json!({
				"closed": v.closed,
				"manifold": v.manifold,
				"euler_characteristic": v.euler_characteristic,
				"genus": v.genus,
				"shells": v.shells,
				"valid": v.is_valid(),
				"geometric_ok": witness.is_none(),
				"source": "solid",
			});
			if let Some(w) = &witness {
				out["self_intersection"] = self_intersection_json(w);
			}
			Ok(Outcome::measures(out))
		}
		OpKind::Volume { input } => {
			let target = fetch_measurable(env, all_ids, op_id, "in", &input)?;
			// Provenance (M2): `volume` is the tessellated (faceted) volume — use `exact_volume`
			// or `mass_properties` for the analytic value where the faces carry analytic surfaces.
			// A bound mesh's enclosed volume is only defined when it is watertight;
			// a leaky mesh gets a refusal, never a plausible number.
			match target {
				Measurable::Solid(s) => {
					Ok(Outcome::measures(json!({ "volume": kernel_brep::volume(s), "provenance": "faceted", "source": "solid" })))
				}
				Measurable::Mesh(m) => {
					// Edge topology only: whether the surface closes is an O(T) question,
					// and `check_mesh` would additionally run the self-intersection sweep
					// that `validate` exists to pay for.
					if !m.is_two_manifold() {
						return Err(err(
							ErrorKind::InvalidGeometry,
							format!(
								"op '{op_id}': '{input}' is a mesh with {} boundary edges and {} edges not shared by exactly two triangles — an open or non-manifold surface encloses no volume, so there is no number to report. Heal it (`import_mesh` with heal, or re-mesh through the voxel route) or measure its `bounding_box` instead",
								m.boundary_edge_count(),
								m.non_manifold_edge_count()
							),
						));
					}
					Ok(Outcome::measures(json!({ "volume": m.signed_volume(), "provenance": "faceted", "source": "mesh" })))
				}
			}
		}
		OpKind::ExactVolume { input } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			// Analytic where faces carry a quadric/torus surface; degrades to faceted per untagged face.
			Ok(Outcome::measures(json!({ "exact_volume": kernel_brep::exact_volume(s), "provenance": "analytic" })))
		}
		OpKind::MassProperties { input } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let m = kernel_brep::mass_properties(s);
			// `inertia_tensor` is the FULL 3×3 inertia tensor rows [[Ixx,Ixy,Ixz],…] about the
			// center of mass at unit density (mm⁵) — balance/imbalance analysis needs the
			// products of inertia, which the diagonal alone cannot carry. Convention: standard
			// dynamics tensor, off-diagonals are −∫xy dV etc. (glam stores columns; the tensor
			// is symmetric, rows are emitted explicitly). `inertia_diag` stays for compatibility.
			let i = &m.inertia;
			Ok(Outcome::measures(json!({
				"volume": m.volume,
				"center_of_mass": [m.center_of_mass.x, m.center_of_mass.y, m.center_of_mass.z],
				"inertia_diag": [m.inertia.x_axis.x, m.inertia.y_axis.y, m.inertia.z_axis.z],
				"inertia_tensor": [
					[i.x_axis.x, i.y_axis.x, i.z_axis.x],
					[i.x_axis.y, i.y_axis.y, i.z_axis.y],
					[i.x_axis.z, i.y_axis.z, i.z_axis.z],
				],
				"provenance": "analytic",
			})))
		}
		OpKind::BoundingBox { input, envelope } => {
			let target = fetch_measurable(env, all_ids, op_id, "in", &input)?;
			let b = match target {
				Measurable::Solid(s) => kernel_brep::measure::bounding_box(s),
				// The mesh's own extent — for an exported print file this is the
				// envelope check that matters, not the solid's.
				Measurable::Mesh(m) => {
					let a = m.aabb();
					(!m.is_empty() && a.is_valid()).then(|| kernel_brep::measure::BoundingBox {
						min: DVec3::new(a.min.x as f64, a.min.y as f64, a.min.z as f64),
						max: DVec3::new(a.max.x as f64, a.max.y as f64, a.max.z as f64),
					})
				}
			};
			let b = b.ok_or_else(|| {
				err(ErrorKind::InvalidGeometry, format!("op '{op_id}': 'bounding_box' has no finite geometry to measure"))
			})?;
			let (mn, mx, sz, c) = (b.min, b.max, b.size(), b.center());
			let mut m = serde_json::Map::new();
			m.insert("source".into(), json!(target.source()));
			m.insert("min".into(), json!([mn.x, mn.y, mn.z]));
			m.insert("max".into(), json!([mx.x, mx.y, mx.z]));
			m.insert("size".into(), json!([sz.x, sz.y, sz.z]));
			m.insert("center".into(), json!([c.x, c.y, c.z]));
			m.insert("diagonal".into(), json!(b.diagonal()));
			if let Some(e) = envelope {
				let ev = dv3(e);
				m.insert("fits_within".into(), json!(b.fits_within(ev)));
				m.insert("fits_within_rotated".into(), json!(b.fits_within_rotated(ev)));
			}
			Ok(Outcome::measures(Value::Object(m)))
		}
		OpKind::WallThickness { input, flag_below } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let t = kernel_brep::wall_thickness(s, flag_below);
			// `min_thickness` is corner noise in practice (oblique rays at sharp
			// corners, FRICTION #17); report percentiles of the finite samples as
			// the robust signal alongside it.
			let mut finite: Vec<f64> = t.thickness.iter().copied().filter(|d| d.is_finite()).collect();
			finite.sort_unstable_by(f64::total_cmp);
			let pct = |p: f64| -> Value {
				if finite.is_empty() {
					Value::Null
				} else {
					json!(finite[((finite.len() - 1) as f64 * p).round() as usize])
				}
			};
			Ok(Outcome::measures(json!({
				"min_thickness": t.min_thickness,
				"p05_thickness": pct(0.05),
				"median_thickness": pct(0.5),
				"thin_area": t.thin_area,
				"flag_below": flag_below,
				"sampled_triangles": t.thickness.len(),
			})))
		}
		OpKind::DraftAnalysis { input, pull, min_deg } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let pull = dv3(pull);
			if pull.length_squared() <= f64::EPSILON {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': pull direction must be a non-zero vector")));
			}
			let d = kernel_brep::draft_analysis(s, pull, min_deg);
			Ok(Outcome::measures(json!({
				"min_draft_deg": d.min_draft_deg,
				"low_draft_area": d.low_draft_area,
				"undercut_area": d.undercut_area,
			})))
		}
		OpKind::MeshComponents { input, tol, weld_tol } => {
			let target = fetch_measurable(env, all_ids, op_id, "in", &input)?;
			connectivity_tolerances(op_id, tol, weld_tol)?;
			// Raw exact tessellation (never the voxel heal): connectivity of the
			// modelled surfaces is the question, and welding is what makes
			// coincident-but-unshared boolean vertices count as one point.
			let mesh = target.mesh(tol);
			let mut m = connectivity_measures(op_id, &mesh, tol, weld_tol, target.source())?;
			m.insert("provenance".into(), json!("faceted"));
			Ok(Outcome::measures(Value::Object(m)))
		}

		// --- Assertions ----------------------------------------------------------------------
		OpKind::Assert { input, volume_within, exact_volume_within, genus, shells, components, closed, manifold, valid, tol, weld_tol } => {
			let target = fetch_measurable(env, all_ids, op_id, "in", &input)?;
			let any_check = volume_within.is_some()
				|| exact_volume_within.is_some()
				|| genus.is_some() || shells.is_some() || components.is_some()
				|| closed.is_some() || manifold.is_some() || valid.is_some();
			if !any_check {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': assert has no checks — give at least one of volume_within / exact_volume_within / genus / shells / components / closed / manifold / valid"),
				));
			}
			connectivity_tolerances(op_id, tol, weld_tol)?;
			// A bound MESH has no B-rep topology: genus / shells / exact_volume are
			// records of the solid model, and inventing them from triangles would be
			// exactly the plausible-looking number this surface refuses to produce.
			// The mesh-meaningful checks (components / closed / manifold / valid /
			// volume_within) are answered from the mesh itself.
			let s = match target {
				Measurable::Solid(s) => Some(s),
				Measurable::Mesh(_) => {
					for (name, present) in
						[("genus", genus.is_some()), ("shells", shells.is_some()), ("exact_volume_within", exact_volume_within.is_some())]
					{
						if present {
							return Err(err(
								ErrorKind::WrongType,
								format!(
									"op '{op_id}': assert '{name}' needs a bound SOLID — '{input}' is a mesh, which carries no B-rep topology or analytic surfaces. On a mesh assert components / closed / manifold / valid / volume_within instead"
								),
							));
						}
					}
					None
				}
			};
			let v = s.map(kernel_brep::validate);
			// Closed / manifold for a mesh come from edge topology alone; running the
			// full `check_mesh` here would drag in the self-intersection sweep, which
			// `assert` never reports and which is the expensive part of that call.
			let mesh_report = match target {
				Measurable::Mesh(m) => Some((m.boundary_edge_count() == 0 && !m.is_empty(), m.is_two_manifold())),
				Measurable::Solid(_) => None,
			};
			let mut measures = serde_json::Map::new();
			let mut failures: Vec<String> = Vec::new();
			let mut within = |what: &str, measured: f64, spec: &crate::program::WithinSpec| -> Result<(), OpError> {
				let half_width = match (spec.abs, spec.percent) {
					(Some(abs), None) if abs.is_finite() && abs >= 0.0 => abs,
					(None, Some(pct)) if pct.is_finite() && pct >= 0.0 => spec.target.abs() * pct / 100.0,
					_ => {
						return Err(err(
							ErrorKind::InvalidParam,
							format!("op '{op_id}': {what}: exactly one of 'abs' / 'percent' is required (a finite non-negative tolerance)"),
						));
					}
				};
				if (measured - spec.target).abs() > half_width {
					failures.push(format!("{what}: measured {measured} is outside {} ± {half_width}", spec.target));
				}
				Ok(())
			};
			if let Some(spec) = &volume_within {
				let measured = match (s, &target) {
					(Some(s), _) => kernel_brep::volume(s),
					(None, Measurable::Mesh(m)) => m.signed_volume(),
					(None, _) => unreachable!("a non-solid target is a mesh"),
				};
				within("volume_within", measured, spec)?;
				measures.insert("volume".to_string(), json!(measured));
			}
			if let (Some(spec), Some(s)) = (&exact_volume_within, s) {
				let measured = kernel_brep::exact_volume(s);
				within("exact_volume_within", measured, spec)?;
				measures.insert("exact_volume".to_string(), json!(measured));
			}
			let mut equals = |what: &str, measured: Value, expected: Value| {
				if measured != expected {
					failures.push(format!("{what}: measured {measured}, expected {expected}"));
				}
				measures.insert(what.to_string(), measured);
			};
			if let (Some(g), Some(v)) = (genus, &v) {
				equals("genus", json!(v.genus), json!(g));
			}
			if let (Some(n), Some(v)) = (shells, &v) {
				equals("shells", json!(v.shells), json!(n));
			}
			if let Some(n) = components {
				// The single-body oracle (FRICTION #24): union-find over welded
				// triangle connectivity — `shells` counts B-rep records and cannot
				// catch a severed part, while this cannot see a severance narrower
				// than `weld_tol`. They are COMPLEMENTARY; neither dominates.
				let mesh = target.mesh(tol);
				let m = connectivity_measures(op_id, &mesh, tol, weld_tol, target.source())?;
				equals("components", m["components"].clone(), json!(n));
			}
			// closed / manifold / valid come from the B-rep record for a solid and
			// from the triangle topology for a mesh — same question, same answer
			// shape, measured where the geometry actually lives.
			if let Some(c) = closed {
				let measured = match (&v, &mesh_report) {
					(Some(v), _) => v.closed,
					(None, Some((closed, _))) => *closed,
					(None, None) => unreachable!("a target is a solid or a mesh"),
				};
				equals("closed", json!(measured), json!(c));
			}
			if let Some(m) = manifold {
				let measured = match (&v, &mesh_report) {
					(Some(v), _) => v.manifold,
					(None, Some((_, manifold))) => *manifold,
					(None, None) => unreachable!("a target is a solid or a mesh"),
				};
				equals("manifold", json!(measured), json!(m));
			}
			if let Some(ok) = valid {
				let measured = match (&v, &mesh_report) {
					(Some(v), _) => v.is_valid(),
					(None, Some((closed, manifold))) => *closed && *manifold,
					(None, None) => unreachable!("a target is a solid or a mesh"),
				};
				equals("valid", json!(measured), json!(ok));
			}
			if failures.is_empty() {
				Ok(Outcome::measures(Value::Object(measures)))
			} else {
				Err(err(ErrorKind::AssertFailed, format!("op '{op_id}': assert failed: {}", failures.join("; "))))
			}
		}
		OpKind::AssertDisjoint { a, b, min_clearance, tol } => {
			let ta = fetch_measurable(env, all_ids, op_id, "a", &a)?;
			let tb = fetch_measurable(env, all_ids, op_id, "b", &b)?;
			if !(tol.is_finite() && tol > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': tol must be a positive chord tolerance in mm")));
			}
			if !(min_clearance.is_finite() && min_clearance >= 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': min_clearance must be a finite non-negative gap in mm")));
			}
			// Raw exact tessellations: vertices lie on the true surfaces, and a
			// distance query needs no watertightness — never the voxel heal.
			let ma = ta.mesh(tol);
			let mb = tb.mesh(tol);
			let distance = ma.min_distance(&mb);
			if distance > min_clearance {
				Ok(Outcome::measures(
					json!({ "distance": distance, "min_clearance": min_clearance, "tol": tol, "source": [ta.source(), tb.source()] }),
				))
			} else {
				Err(err(
					ErrorKind::AssertFailed,
					format!(
						"op '{op_id}': assert_disjoint failed: surface distance {distance} mm ≤ required clearance {min_clearance} mm — '{a}' and '{b}' touch or interfere"
					),
				))
			}
		}

		OpKind::CoincidentFit { a, b } => {
			// Advisory pre-check the agent runs BEFORE a boolean to avoid the coincident-fit
			// hazard (audit V4). Wires the existing kernel scan; refuses nothing — the hard
			// hang backstop is the server request timeout (V6).
			let sa = fetch_solid(env, all_ids, op_id, "a", &a)?;
			let sb = fetch_solid(env, all_ids, op_id, "b", &b)?;
			Ok(Outcome::measures(json!({ "coincident_fit": kernel_brep::detect_coincident_fit(sa, sb) })))
		}
		OpKind::SupportReport { input, build_dir, overhang_deg } => {
			// M5: FDM support-necessity audit — wires the existing Mesh::support_free_report.
			// Accepts a bound mesh so the audit can be run on the file that actually
			// prints (an export's healed mesh is not the solid's tessellation).
			let target = fetch_measurable(env, all_ids, op_id, "in", &input)?;
			let mesh = target.mesh(0.05);
			let up = Vec3::new(build_dir[0] as f32, build_dir[1] as f32, build_dir[2] as f32);
			let r = mesh.support_free_report(up, overhang_deg as f32, 0.2);
			Ok(Outcome::measures(json!({
				"support_free": r.steep_area < 1e-6,
				"bed_area": r.bed_area,
				"bridge_area": r.bridge_area,
				"steep_area": r.steep_area,
				"total_area": r.total_area,
				"max_bridge_span": r.max_bridge_span,
				"provenance": "faceted",
				"source": target.source(),
			})))
		}
		OpKind::Clearance { a, b, tol } => {
			// M5: non-asserting clearance/interference — the measuring twin of assert_disjoint.
			let ta = fetch_measurable(env, all_ids, op_id, "a", &a)?;
			let tb = fetch_measurable(env, all_ids, op_id, "b", &b)?;
			if !(tol.is_finite() && tol > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': tol must be a positive chord tolerance in mm")));
			}
			let ma = ta.mesh(tol);
			let mb = tb.mesh(tol);
			let distance = ma.min_distance(&mb);
			// overlap_volume runs an EXACT boolean intersection, so it needs two exact
			// solids, and it is skipped on the coincident-fit hazard (a press-fit) so a
			// clearance query can't trigger the coincident-fit boolean hang (V4).
			// `overlap_volume: null` used to arrive with no explanation attached —
			// a null that does not say why is indistinguishable from a bug, so the
			// reason is now a first-class field and is never absent when the value is.
			let (overlap, hazard, reason) = match (&ta, &tb) {
				(Measurable::Solid(sa), Measurable::Solid(sb)) => {
					let hazard = kernel_brep::detect_coincident_fit(sa, sb);
					if hazard {
						(None, true, Some("coincident_fit_hazard: the operands share a flush/press-fit face pair, and the exact intersection across it is the known boolean-hang case (V4) — measure the fit analytically (measure_dimension diameter) instead"))
					} else {
						match kernel_brep::overlap_volume(sa, sb) {
							Some(v) => (Some(v), false, None),
							// The exact arrangement can fail on posed/near-degenerate
							// pairs while the meshes overlap plainly (friction
							// folding_book_stand F5: `overlap_volume: null` on an
							// interfering posed pair). Fall back to the mesh-level
							// boolean of the already-tessellated operands — a faceted
							// estimate, labelled as such, instead of a null.
							None => {
								let common = kernel_brep::mesh_intersection(&ma, &mb);
								let v = common.signed_volume().abs();
								(
									Some(v),
									false,
									Some("the exact boolean intersection did not produce a measurable body for this operand pair — `overlap_volume` is the FACETED mesh-boolean volume of the tessellated operands at `tol` (an estimate, not the analytic overlap); gate `exact_volume` on an explicit `intersection` body when the exact number matters"),
								)
							}
						}
					}
				}
				_ => (
					None,
					false,
					Some("overlap_volume needs two exact solids; at least one operand is a bound MESH, which carries no exact boolean — `distance` is measured on the meshes and is the honest answer here"),
				),
			};
			// With no overlap volume the only evidence is the surface gap. A gap of
			// exactly 0 on faceted operands is CONTACT within the faceting, not proof
			// of interpenetration, so it is reported as such rather than as a boolean.
			let interfering = overlap.map(|v| v > 1e-9).unwrap_or(distance < 1e-6);
			let mut m = json!({
				"distance": distance,
				"interfering": interfering,
				"overlap_volume": overlap,
				"coincident_fit_hazard": hazard,
				"tol": tol,
				"provenance": "faceted",
				"source": [ta.source(), tb.source()],
			});
			if let Some(r) = reason {
				m["overlap_volume_reason"] = json!(r);
			}
			Ok(Outcome::measures(m))
		}
		OpKind::Describe { name } => {
			// M3: self-describe the op surface from the single authoritative catalogue (discover.rs),
			// which is compile-forced complete via op_tag — the list cannot drift from what runs.
			// With `name`, a real op also gets its parameter specs (generated OP_PARAMS table);
			// no-arg describe stays names+count (the full 139-op param dump would be huge) and
			// advertises `params_available` so callers know to ask per-op.
			match name {
				Some(n) => {
					let params = crate::discover::op_params(&n);
					let mut m = json!({ "name": n, "exists": params.is_some() });
					if let Some(specs) = params {
						// The generated per-op table PLUS the universal params every op
						// accepts, so `describe` is the complete answer to "what may I
						// pass here" — a param advertised nowhere is a param nobody uses.
						let mut list: Vec<Value> = specs
							.iter()
							.map(|p| {
								let mut spec = json!({ "name": p.name, "type": p.ty, "required": p.required, "doc": p.doc });
								if !p.aliases.is_empty() {
									spec["aliases"] = json!(p.aliases);
								}
								spec
							})
							.collect();
						list.push(universal_require_param());
						m["params"] = Value::Array(list);
					}
					Ok(Outcome::measures(m))
				}
				None => Ok(Outcome::measures(json!({
					"count": crate::discover::OP_COUNT,
					"ops": crate::discover::OP_NAMES,
					"params_available": true,
					"universal_params": [universal_require_param()],
				}))),
			}
		}
		OpKind::ListFaces { input } => {
			// M4 loop: enumerate faces as references (analytic descriptor + a witness point), read
			// from the existing kernel topology — no build, no geometry change.
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let faces: Vec<Value> = s
				.faces()
				.enumerate()
				.map(|(i, fid)| {
					let (kind, descriptor) = match s.face(fid).surface {
						kernel_brep::Surface::Plane { origin, normal } => ("plane", json!({ "normal": v3a(normal), "point": v3a(origin) })),
						kernel_brep::Surface::Cylinder { origin, axis, radius } => ("cylinder", json!({ "axis": v3a(axis), "point": v3a(origin), "radius": radius })),
						kernel_brep::Surface::Sphere { center, radius } => ("sphere", json!({ "center": v3a(center), "radius": radius })),
						kernel_brep::Surface::Cone { apex, axis, half_angle } => ("cone", json!({ "apex": v3a(apex), "axis": v3a(axis), "half_angle": half_angle })),
						kernel_brep::Surface::Torus { center, axis, major, minor } => ("torus", json!({ "center": v3a(center), "axis": v3a(axis), "major": major, "minor": minor })),
					};
					let poly = s.face_polygon(fid);
					let area = if kind == "plane" { Some(polygon_area(&poly)) } else { None };
					json!({ "index": i, "type": kind, "descriptor": descriptor, "witness": v3a(polygon_centroid(&poly)), "area": area })
				})
				.collect();
			Ok(Outcome::measures(json!({ "count": faces.len(), "faces": faces })))
		}
		OpKind::ListEdges { input } => {
			// M4 loop: enumerate edges as references (midpoint witness + chord length).
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let edges: Vec<Value> = s
				.edges()
				.enumerate()
				.map(|(i, eid)| {
					let he = s.half_edge(s.edge(eid).half_edge);
					let a = s.position(he.origin);
					let b = s.position(s.half_edge(he.next).origin);
					json!({ "index": i, "midpoint": v3a((a + b) * 0.5), "length": (a - b).length(), "curved": s.edge_curve(eid).is_some() })
				})
				.collect();
			Ok(Outcome::measures(json!({ "count": edges.len(), "edges": edges })))
		}

		// --- Exports -------------------------------------------------------------------------
		OpKind::ExportStl { input, file, tol, voxel } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			export_mesh(op_id, s, tol, voxel, out_dir, &file, "stl")
		}
		OpKind::Export3mf { input, file, tol, voxel } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			export_mesh(op_id, s, tol, voxel, out_dir, &file, "3mf")
		}
		OpKind::ExportStep { input, file } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let path = resolve_path(op_id, out_dir, &file)?;
			let name = path.file_stem().and_then(|s| s.to_str()).unwrap_or("part").to_string();
			let step = kernel_brep::export_step(s, &name);
			std::fs::write(&path, step).map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot write '{}': {e}", path.display())))?;
			Ok(Outcome { value: None, measures: None, file: Some(path.display().to_string()) })
		}

		// --- Implicit / hybrid ------------------------------------------------------------------
		OpKind::GyroidBlock { center, half, scale, thickness, voxel, file } => {
			for (name, value) in [("half", half), ("scale", scale), ("thickness", thickness), ("voxel", voxel)] {
				if !(value.is_finite() && value > 0.0) {
					return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': {name} must be a positive number")));
				}
			}
			let c = Vec3::new(center[0] as f32, center[1] as f32, center[2] as f32);
			let region = Aabb::from_center_half_extent(c, Vec3::splat(half as f32));
			let lattice = Node::primitive(Gyroid::new(region, scale as f32, thickness as f32))
				.intersection(Node::primitive(ImplicitCuboid::new(c, Vec3::splat(half as f32))));
			let domain = region.pad(3.0 * voxel as f32);
			let mut mesh = manifold_dual_contour(&lattice, domain, Resolution::VoxelSize(voxel as f32));
			let mut healed = false;
			if !mesh.is_watertight() || check_mesh(&mesh).non_manifold_edges > 0 {
				mesh = make_manifold(&mesh);
				healed = true;
			}
			let report = check_mesh(&mesh);
			if mesh.triangle_count() == 0 || !mesh.is_watertight() || report.non_manifold_edges > 0 {
				return Err(err(
					ErrorKind::InvalidGeometry,
					format!(
						"op '{op_id}': gyroid lattice did not mesh watertight at voxel {voxel} (triangles={}, watertight={}, non_manifold_edges={}) — try a smaller voxel or a thicker wall",
						mesh.triangle_count(),
						mesh.is_watertight(),
						report.non_manifold_edges
					),
				));
			}
			let path = resolve_path(op_id, out_dir, &file)?;
			mesh.write_stl_binary(&path)
				.map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot write '{}': {e}", path.display())))?;
			let measures = json!({
				"triangles": mesh.triangle_count(),
				"watertight": true,
				"healed": healed,
			});
			Ok(Outcome { value: Some(EnvValue::Mesh(mesh.clone())), measures: Some(measures), file: Some(path.display().to_string()) })
		}

		OpKind::SampleDensityGrid { input, expr, origin, voxel, shape, supersample, file } => {
			if !(voxel.is_finite() && voxel > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': voxel must be a positive voxel size in mm")));
			}
			if shape.iter().any(|&n| n == 0 || n > 2048) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': shape axes must be 1..=2048, got {shape:?}")));
			}
			if supersample == 0 || supersample > 4 {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': supersample must be 1..=4")));
			}
			let o = Vec3::new(origin[0] as f32, origin[1] as f32, origin[2] as f32);
			let h = voxel as f32;
			let rho = match (&input, &expr) {
				(Some(id), None) => {
					let solid = fetch_solid(env, all_ids, op_id, "in", id)?;
					let mesh = kernel_brep::tessellate_default(solid);
					let sdf = crate::bridge::mesh_sdf(&mesh);
					crate::bridge::sample_density(&sdf, o, h, shape, supersample)
				}
				(None, Some(tree)) => {
					let parsed = implicit::parse_tree(op_id, tree, input_base)?;
					crate::bridge::sample_density(&parsed.node, o, h, shape, supersample)
				}
				_ => {
					return Err(err(
						ErrorKind::InvalidParam,
						format!("op '{op_id}': give exactly one of 'in' (a solid id) or 'expr' (an implicit tree)"),
					));
				}
			};
			let mean: f32 = rho.iter().sum::<f32>() / rho.len() as f32;
			let bytes = crate::bridge::write_npy_f32(&shape, &rho);
			let path = resolve_path(op_id, out_dir, &file)?;
			std::fs::write(&path, &bytes).map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot write '{}': {e}", path.display())))?;
			Ok(Outcome {
				value: None,
				measures: Some(json!({
					"voxels": shape[0] * shape[1] * shape[2],
					"shape": shape,
					"solid_fraction_mean": mean,
					"bytes": bytes.len(),
				})),
				file: Some(path.display().to_string()),
			})
		}
		OpKind::MeshDensityGrid { npy, origin, voxel, iso, file } => {
			if !(voxel.is_finite() && voxel > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': voxel must be a positive voxel size in mm")));
			}
			let path_in = resolve_input_or_out(op_id, input_base, out_dir, &npy)?;
			let bytes = std::fs::read(&path_in).map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot read '{}': {e}", path_in.display())))?;
			let (nshape, rho) = crate::bridge::read_npy_f32(&bytes)
				.map_err(|e| err(ErrorKind::InvalidParam, format!("op '{op_id}': '{npy}': {e}")))?;
			if nshape.len() != 3 {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': '{npy}' must be a 3-D array, got shape {nshape:?}")));
			}
			let dims = [nshape[0], nshape[1], nshape[2]];
			let o = Vec3::new(origin[0] as f32, origin[1] as f32, origin[2] as f32);
			let grid = crate::bridge::density_to_grid(dims, &rho, o, voxel as f32, iso as f32);
			let domain = grid.lattice_bounds();
			let mut mesh = dual_contour_narrowband(&grid, domain, Resolution::VoxelSize(voxel as f32));
			let mut healed = false;
			if !mesh.is_watertight() || check_mesh(&mesh).non_manifold_edges > 0 {
				mesh = make_manifold(&mesh);
				healed = true;
			}
			if mesh.triangle_count() == 0 || !mesh.is_watertight() || check_mesh(&mesh).non_manifold_edges > 0 {
				return Err(err(
					ErrorKind::InvalidGeometry,
					format!(
						"op '{op_id}': the density level-set did not mesh watertight at voxel {voxel} (triangles={}, watertight={}) — refine the grid, check iso, or inspect disconnected debris in the density field",
						mesh.triangle_count(),
						mesh.is_watertight()
					),
				));
			}
			let volume = mesh.signed_volume();
			let path = resolve_path(op_id, out_dir, &file)?;
			let write_result = match path.extension().and_then(|e| e.to_str()) {
				Some("stl") => mesh.write_stl_binary(&path),
				Some("3mf") => mesh.write_3mf(&path),
				other => {
					return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': 'file' must end in .stl or .3mf, got extension {other:?}")));
				}
			};
			write_result.map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot write '{}': {e}", path.display())))?;
			Ok(Outcome {
				value: Some(EnvValue::Mesh(mesh.clone())),
				measures: Some(json!({
					"ok": true,
					"volume_mm3": volume,
					"num_triangles": mesh.triangle_count(),
					"watertight": true,
					"healed": healed,
				})),
				file: Some(path.display().to_string()),
			})
		}
		OpKind::Implicit { expr, voxel, mesher, domain, file } => {
			if !(voxel.is_finite() && voxel > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': voxel must be a positive voxel size in mm")));
			}
			let parsed = implicit::parse_tree(op_id, &expr, input_base)?;
			// Field-quality honesty (Unit 4a): offset/shell/offset_by on a DistanceBound
			// field is only APPROXIMATE. Surface it in the op measures so it can never
			// pass unnoticed — meshing is where the approximation becomes a real solid.
			let approximate_offset = parsed.node.has_approximate_offset();
			let domain_box = match domain {
				Some(d) => {
					let lo = Vec3::new(d.min[0] as f32, d.min[1] as f32, d.min[2] as f32);
					let hi = Vec3::new(d.max[0] as f32, d.max[1] as f32, d.max[2] as f32);
					if !(lo.is_finite() && hi.is_finite() && lo.x < hi.x && lo.y < hi.y && lo.z < hi.z) {
						return Err(err(
							ErrorKind::InvalidParam,
							format!("op '{op_id}': 'domain.min' must be finite and strictly below 'domain.max' on every axis"),
						));
					}
					Aabb::new(lo, hi)
				}
				None => {
					let b = parsed.node.bounds();
					if !b.is_valid() {
						return Err(err(
							ErrorKind::InvalidParam,
							format!("op '{op_id}': the expression tree has empty bounds (e.g. an intersection of disjoint shapes) — nothing to mesh"),
						));
					}
					if !(b.min.is_finite() && b.max.is_finite()) {
						return Err(err(
							ErrorKind::InvalidParam,
							format!("op '{op_id}': the expression tree is unbounded (a bare 'plane' or a bounds-less 'expr_sdf' leaf) — intersect it with a bounded shape, give the expr_sdf leaf min/max bounds, or pass an explicit 'domain'"),
						));
					}
					b.pad(3.0 * voxel as f32)
				}
			};
			implicit::probe_fields(op_id, &parsed.fields, domain_box)?;
			let mut mesh = match mesher {
				MesherSpec::Narrowband => {
					// The narrow-band extractor prunes by the Lipschitz contract, so an
					// under-declared expr_sdf bound would silently tear holes — verify the
					// declarations against the sampled field first (dense meshers skip this).
					implicit::probe_lipschitz(op_id, &parsed.fields, domain_box)?;
					dual_contour_narrowband(&parsed.node, domain_box, Resolution::VoxelSize(voxel as f32))
				}
				MesherSpec::Manifold => manifold_dual_contour(&parsed.node, domain_box, Resolution::VoxelSize(voxel as f32)),
			};
			let mut healed = false;
			if !mesh.is_watertight() || check_mesh(&mesh).non_manifold_edges > 0 {
				mesh = make_manifold(&mesh);
				healed = true;
			}
			let report = check_mesh(&mesh);
			if mesh.triangle_count() == 0 || !mesh.is_watertight() || report.non_manifold_edges > 0 {
				return Err(err(
					ErrorKind::InvalidGeometry,
					format!(
						"op '{op_id}': the implicit tree did not mesh watertight at voxel {voxel} (triangles={}, watertight={}, non_manifold_edges={}) — refine the voxel (thin walls need ≥ ~3 voxels), check that the tree is non-empty inside the domain, verify expr_sdf lipschitz_bound declarations, or switch to \"mesher\": \"manifold\" for junction-rich lattices",
						mesh.triangle_count(),
						mesh.is_watertight(),
						report.non_manifold_edges
					),
				));
			}
			let volume = mesh.signed_volume();
			let written = match file {
				Some(file) => {
					let path = resolve_path(op_id, out_dir, &file)?;
					let write_result = match path.extension().and_then(|e| e.to_str()) {
						Some("stl") => mesh.write_stl_binary(&path),
						Some("3mf") => mesh.write_3mf(&path),
						other => {
							return Err(err(
								ErrorKind::InvalidParam,
								format!("op '{op_id}': 'file' must end in .stl or .3mf, got extension {other:?}"),
							));
						}
					};
					write_result.map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot write '{}': {e}", path.display())))?;
					Some(path.display().to_string())
				}
				None => None,
			};
			Ok(Outcome {
				value: Some(EnvValue::Mesh(mesh.clone())),
				measures: Some(json!({
					"triangles": mesh.triangle_count(),
					"watertight": true,
					"healed": healed,
					"volume": volume,
					// Unit 4a: true when offset/shell/offset_by acted on a distance-BOUND field,
					// so the offset distance is only approximate — surfaced in measures, not silent.
					"approximate_offset": approximate_offset,
				})),
				file: written,
			})
		}
		OpKind::Shell { input, wall, voxel, file } => {
			// Voxel-route hollow, reusing the kernel's EXISTING machinery end to end:
			// the same winding-number `MeshSdf` lift as `kernel_model::watertight_mesh`,
			// the same inward-offset difference as `kernel_model::Feature::Shell`
			// (`outer − offset(inner, −wall)`, outer surface preserved), and the same
			// Manifold-Dual-Contour + heal + watertight gate as `gyroid_block`. The
			// result is `voxel_healed` BY CONSTRUCTION — accurate to `voxel`, not exact.
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			if !(wall.is_finite() && wall > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': wall must be a positive thickness in mm")));
			}
			if !(voxel.is_finite() && voxel > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': voxel must be a positive voxel size in mm")));
			}
			// A wall the grid cannot resolve fails DETERMINISTICALLY here, instead of
			// as a mysterious non-watertight mesh three seconds later.
			if wall < 2.0 * voxel {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': wall {wall} mm is under 2 × voxel ({voxel} mm) — the grid cannot resolve it; shrink 'voxel' or thicken the wall"),
				));
			}
			let base = kernel_brep::tessellate_default(s);
			let outer = MeshSdf::new(&base);
			let domain = outer.bounds().pad(2.0 * voxel as f32);
			// Same allocation discipline as the gyroid/density grids: reject a grid
			// beyond the cell cap before allocating it.
			let size = domain.size();
			let cells = (f64::from(size.x) / voxel).ceil() * (f64::from(size.y) / voxel).ceil() * (f64::from(size.z) / voxel).ceil();
			if !(cells.is_finite() && cells <= MAX_GRID_CELLS as f64) {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': shell grid ≈{cells:.0} cells (bbox/voxel)³ exceeds the cap {MAX_GRID_CELLS} — use a coarser voxel"),
				));
			}
			// Built twice (like `Feature::Shell`): the SDF tree owns its leaves.
			let inner = MeshSdf::new(&base);
			let node = Node::primitive(outer).difference(Node::primitive(inner).offset(-wall as f32));
			let mut mesh = manifold_dual_contour(&node, domain, Resolution::VoxelSize(voxel as f32));
			let mut healed = false;
			if !mesh.is_watertight() || check_mesh(&mesh).non_manifold_edges > 0 {
				mesh = make_manifold(&mesh);
				healed = true;
			}
			let report = check_mesh(&mesh);
			if mesh.triangle_count() == 0 || !mesh.is_watertight() || report.non_manifold_edges > 0 {
				return Err(err(
					ErrorKind::InvalidGeometry,
					format!(
						"op '{op_id}': the shelled solid did not mesh watertight at voxel {voxel} (triangles={}, watertight={}, non_manifold_edges={}) — walls need ≥ ~3 voxels; shrink 'voxel' or thicken the wall",
						mesh.triangle_count(),
						mesh.is_watertight(),
						report.non_manifold_edges
					),
				));
			}
			let volume = mesh.signed_volume();
			let written = match file {
				Some(file) => {
					let path = resolve_path(op_id, out_dir, &file)?;
					let write_result = match path.extension().and_then(|e| e.to_str()) {
						Some("stl") => mesh.write_stl_binary(&path),
						Some("3mf") => mesh.write_3mf(&path),
						other => {
							return Err(err(
								ErrorKind::InvalidParam,
								format!("op '{op_id}': 'file' must end in .stl or .3mf, got extension {other:?}"),
							));
						}
					};
					write_result.map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot write '{}': {e}", path.display())))?;
					Some(path.display().to_string())
				}
				None => None,
			};
			Ok(Outcome {
				value: Some(EnvValue::Mesh(mesh.clone())),
				measures: Some(json!({
					"route": "voxel_healed",
					"triangles": mesh.triangle_count(),
					"watertight": true,
					"healed": healed,
					"volume": volume,
					"wall": wall,
					"voxel": voxel,
				})),
				file: written,
			})
		}

		// --- Voxel-route solid ops & interrogation probes (2026-07-29 implicit wave) -------------
		OpKind::OffsetSolid { input, delta, voxel } => {
			// Signed surface offset via `kernel_model::shell::offset_to_solid`:
			// grow (delta > 0, Minkowski sum with a ball — convex edges gain a true
			// delta-radius round) or shrink (delta < 0, erosion — anything thinner
			// than 2·|delta| vanishes). The result re-enters the SOLID environment
			// as a FACETED B-rep; the receipts say route "voxel", never exact.
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			if !delta.is_finite() {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': delta must be a finite signed offset in mm (positive grows, negative shrinks)"),
				));
			}
			if !(voxel.is_finite() && voxel > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': voxel must be a positive voxel size in mm")));
			}
			let (smin, smax) = s.aabb();
			let domain = Aabb::new(
				Vec3::new(smin.x as f32, smin.y as f32, smin.z as f32),
				Vec3::new(smax.x as f32, smax.y as f32, smax.z as f32),
			)
			.pad(delta.abs() as f32 + 3.0 * voxel as f32);
			grid_guard(op_id, "offset_solid", domain, voxel)?;
			let out = kernel_model::shell::offset_to_solid(s, delta, voxel as f32);
			if out.face_count() == 0 {
				return Err(err(
					ErrorKind::InvalidParam,
					format!(
						"op '{op_id}': offset_solid produced an empty result — a negative delta ({delta} mm) at or beyond the part's inradius erodes it away entirely (regions thinner than 2·|delta| vanish); shrink |delta|"
					),
				));
			}
			let mut measures = voxel_solid_measures(&out, voxel);
			measures["delta"] = json!(delta);
			let outcome = bind_solid(op_id, "offset_solid", out)?;
			Ok(Outcome { measures: Some(measures), ..outcome })
		}
		OpKind::ShellSolid { input, thickness, voxel } => {
			// Hollow into the SOLID environment via `kernel_model::shell::
			// shell_to_solid` (outer surface preserved, cavity sealed): the
			// solid-binding sibling of the file-writing `shell` op. Faceted B-rep,
			// route "voxel"; the cavity shows up as a second nested shell.
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			if !(thickness.is_finite() && thickness > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': thickness must be a positive wall thickness in mm")));
			}
			if !(voxel.is_finite() && voxel > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': voxel must be a positive voxel size in mm")));
			}
			// A wall the grid cannot resolve fails DETERMINISTICALLY here (the same
			// guard as the `shell` op), not as a leaky mesh later.
			if thickness < 2.0 * voxel {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': thickness {thickness} mm is under 2 × voxel ({voxel} mm) — the grid cannot resolve the wall; shrink 'voxel' or thicken it"),
				));
			}
			let (smin, smax) = s.aabb();
			let domain = Aabb::new(
				Vec3::new(smin.x as f32, smin.y as f32, smin.z as f32),
				Vec3::new(smax.x as f32, smax.y as f32, smax.z as f32),
			)
			.pad(3.0 * voxel as f32);
			grid_guard(op_id, "shell_solid", domain, voxel)?;
			let out = kernel_model::shell::shell_to_solid(s, thickness, voxel as f32);
			if out.face_count() == 0 {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': shell_solid produced an empty result at voxel {voxel} — the wall did not survive re-extraction; shrink 'voxel'"),
				));
			}
			let mut measures = voxel_solid_measures(&out, voxel);
			measures["thickness"] = json!(thickness);
			// shells == 2 proves the sealed cavity survived the bridge back to
			// B-rep; shells == 1 means the wall met itself (thickness ≥ inradius —
			// the "shell" is just the re-healed solid). Stated, not hidden.
			let cavity = measures["shells"].as_u64().is_some_and(|n| n >= 2);
			measures["cavity"] = json!(cavity);
			let outcome = bind_solid(op_id, "shell_solid", out)?;
			Ok(Outcome { measures: Some(measures), ..outcome })
		}
		OpKind::SolidFromImplicit { expr, voxel, domain } => {
			// Reverse bridge v1 (`kernel_model::reverse::implicit_to_solid`): the
			// implicit tree is meshed dense (Manifold DC — no Lipschitz assumption)
			// and wrapped into a validated FACETED B-rep, gated on volume
			// conservation. This is the one honest crossing from the field world
			// back into the solid environment — at chord fidelity `voxel`, with no
			// analytic curved-surface recovery (that is the ledgered v2).
			if !(voxel.is_finite() && voxel > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': voxel must be a positive voxel size in mm")));
			}
			let parsed = implicit::parse_tree(op_id, &expr, input_base)?;
			let approximate_offset = parsed.node.has_approximate_offset();
			let bounds = match explicit_domain(op_id, &domain)? {
				Some(b) => b,
				None => tree_bounds(op_id, &parsed.node)?,
			};
			implicit::probe_fields(op_id, &parsed.fields, bounds)?;
			// implicit_to_solid meshes over bounds padded by 2 voxels — cap that grid.
			grid_guard(op_id, "solid_from_implicit", bounds.pad(2.0 * voxel as f32), voxel)?;
			let solid = kernel_model::reverse::implicit_to_solid(&parsed.node, bounds, voxel as f32).map_err(|e| {
				// "nothing to bridge" = no surface inside the bounds (a degenerate
				// question → invalid_param); every other bridge refusal (weld,
				// validation, volume-conservation) is a geometry-integrity failure.
				let kind = if e.contains("nothing to bridge") { ErrorKind::InvalidParam } else { ErrorKind::InvalidGeometry };
				err(kind, format!("op '{op_id}': {e}"))
			})?;
			let mut measures = voxel_solid_measures(&solid, voxel);
			// The bridge's conservation gate (|solid − mesh| ≤ 1e-6 relative)
			// REFUSED any drift, so success here is the proof it passed.
			measures["volume_conserved"] = json!(true);
			measures["approximate_offset"] = json!(approximate_offset);
			let outcome = bind_solid(op_id, "solid_from_implicit", solid)?;
			Ok(Outcome { measures: Some(measures), ..outcome })
		}
		OpKind::ThinWall { input, expr, t_min, samples, domain } => {
			// Field interrogation BEFORE committing to a mesh or the bridge: the
			// SAMPLED medial thin-wall census (`kernel_model::reverse::
			// thin_wall_report`). An estimate at lattice resolution — it can
			// under-report by up to ~one cell and can MISS walls thinner than the
			// cell entirely; use it to warn, gate final claims on finer sampling.
			if !(t_min.is_finite() && t_min > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': t_min must be a positive thickness in mm")));
			}
			if !(8..=256).contains(&samples) {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': samples must be in 8..=256 (the census costs samples³ field evaluations), got {samples}"),
				));
			}
			let dbox = explicit_domain(op_id, &domain)?;
			let rep = match (&input, &expr) {
				(Some(id), None) => {
					// A bound solid, lifted through the winding-number MeshSdf — the
					// same honest bridge as `sample_density_grid`.
					let solid = fetch_solid(env, all_ids, op_id, "in", id)?;
					let mesh = kernel_brep::tessellate_default(solid);
					let sdf = crate::bridge::mesh_sdf(&mesh);
					// Default census box: the solid's aabb padded by half a lattice
					// step. The pad DE-PHASES the lattice from the solid's own
					// axis-aligned faces: a sample landing exactly ON a face reads
					// |d| ≈ 0 with an ambiguous winding sign, and a −ε accepted as
					// "interior medial" would report a phantom ~0 mm wall (measured
					// during bring-up on a plain box). An explicit 'domain' is the
					// caller's contract and is used verbatim.
					let bounds = match dbox {
						Some(b) => b,
						None => {
							let raw = mesh.aabb();
							let step = (raw.size() / (samples as f32 - 1.0)).max_element();
							raw.pad(0.5 * step)
						}
					};
					kernel_model::reverse::thin_wall_report(&sdf, bounds, samples, t_min as f32)
				}
				(None, Some(tree)) => {
					let parsed = implicit::parse_tree(op_id, tree, input_base)?;
					let bounds = match dbox {
						Some(b) => b,
						None => tree_bounds(op_id, &parsed.node)?,
					};
					implicit::probe_fields(op_id, &parsed.fields, bounds)?;
					kernel_model::reverse::thin_wall_report(&parsed.node, bounds, samples, t_min as f32)
				}
				_ => {
					return Err(err(
						ErrorKind::InvalidParam,
						format!("op '{op_id}': give exactly one of 'in' (a solid id) or 'expr' (an implicit tree)"),
					));
				}
			};
			// thinnest = +∞ means no interior medial sample was found (empty field
			// or too-coarse lattice): an explicit status, never a raw non-finite
			// float smuggled into JSON.
			let m = if rep.thinnest.is_finite() {
				json!({
					"status": "measured",
					"basis": "sampled_medial_estimate",
					"thinnest": rep.thinnest,
					"at": [rep.at.x, rep.at.y, rep.at.z],
					"below_count": rep.below_count,
					"t_min": t_min,
					"samples": samples,
				})
			} else {
				json!({
					"status": "no_interior_samples",
					"basis": "sampled_medial_estimate",
					"thinnest": null,
					"below_count": 0,
					"t_min": t_min,
					"samples": samples,
				})
			};
			Ok(Outcome::measures(m))
		}
		OpKind::MinLigament { input, at, axis, d } => {
			// Advisory pre-cut interrogation (`kernel_brep::holes::min_ligament`):
			// the thinnest wall a PLANNED Ø d bore would leave, 64 stations on one
			// mid-span ring against the exact closest point of the default
			// tessellation. Nothing is cut; the echo is clamped above by ~half the
			// material span (pierce faces are part of the boundary).
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let a = dv3(at);
			let ax = dv3(axis);
			if !a.is_finite() {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': 'at' must be a finite point, got {at:?}")));
			}
			if !(ax.is_finite() && ax.length_squared() > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': 'axis' must be a non-zero finite direction, got {axis:?}")));
			}
			if !(d.is_finite() && d > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': d must be a positive bore diameter in mm, got {d}")));
			}
			let lig = holes::min_ligament(s, a, ax, d);
			// Parameters were validated above, so the kernel's NaN sentinel now
			// means exactly one thing: no material along +axis from `at`. The ∞
			// sentinel (no boundary at all) is unreachable for a bound solid but
			// mapped anyway. Both become explicit statuses — never raw NaN/∞.
			let m = if lig.is_nan() {
				json!({ "status": "no_material", "ligament": null, "d": d, "at": at, "axis": axis })
			} else if lig.is_infinite() {
				json!({ "status": "no_boundary", "ligament": null, "d": d, "at": at, "axis": axis })
			} else {
				json!({
					"status": "measured",
					"basis": "mid_span_ring_64_stations",
					"ligament": lig,
					"d": d,
					"at": at,
					"axis": axis,
				})
			};
			Ok(Outcome::measures(m))
		}

		// --- Native formats ----------------------------------------------------------------------
		OpKind::LoadPart { file } => {
			// T4: program-relative first, then --out-dir (a generated .lmcpart lands there).
			let path = resolve_input_or_out(op_id, input_base, out_dir, &file)?;
			let text = std::fs::read_to_string(&path)
				.map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot read '{}': {e}", path.display())))?;
			let (doc, meta) = format::load_part(&text)
				.map_err(|e| err(ErrorKind::InvalidParam, format!("op '{op_id}': '{}' is not a loadable .lmcpart: {e}", path.display())))?;
			let solid = doc.evaluate_brep().ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': the part's feature tree produced no exact B-rep (voxel-half-only features — shell, gyroid, smooth booleans — cannot enter the solid environment)"),
				)
			})?;
			let outcome = bind_solid(op_id, "load_part", solid)?;
			// Provenance for asm_save: an instance built from this solid can
			// reference the ORIGINAL .lmcpart instead of exporting a mesh.
			asm.solid_sources.insert(op_id.to_string(), file.clone());
			Ok(Outcome {
				measures: Some(json!({ "name": meta.name, "units": meta.units, "created_with": meta.created_with })),
				..outcome
			})
		}

		// --- Imports -----------------------------------------------------------------------------
		OpKind::ImportStep { file, mode } => {
			// STEP → exact B-rep, through the kernel's analytic importer. A multi-solid
			// file merges into ONE multi-shell solid (each MANIFOLD_SOLID_BREP keeps its
			// own shell — `shells` in the measures is the honest count). Trimmed-NURBS
			// faces enter as their chord facets; `freeform_faces` counts them.
			// T4: fall back to --out-dir so an exported STEP re-imports under any out dir.
			let path = resolve_input_or_out(op_id, input_base, out_dir, &file)?;
			let text = std::fs::read_to_string(&path)
				.map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot read '{}': {e}", path.display())))?;
			if mode == crate::program::StepImportMode::Tolerant {
				return import_step_tolerant_op(op_id, &path, &text);
			}
			let (solid, freeform) = kernel_brep::import_step_freeform(&text).map_err(|e| {
				// Parse/Reference/Unsupported are input problems (the message carries the
				// kernel's verbatim reason); Topology means the faces don't form a solid.
				let kind = match &e {
					StepError::Topology(_) => ErrorKind::InvalidGeometry,
					_ => ErrorKind::InvalidParam,
				};
				err(kind, format!("op '{op_id}': import_step '{}': {e}", path.display()))
			})?;
			let v = kernel_brep::validate(&solid);
			let measures = json!({
				"source": "step",
				"shells": v.shells,
				"genus": v.genus,
				"faces": solid.face_count(),
				"volume": kernel_brep::volume(&solid),
				"freeform_faces": freeform.len(),
			});
			Ok(Outcome { measures: Some(measures), ..bind_solid(op_id, "import_step", solid)? })
		}
		OpKind::ImportMesh { file, heal, out } => {
			// Mesh file → welded mesh → full check_mesh receipt. Binds NOTHING (the
			// environment stays Solid|Sketch); `volume` is reported ONLY when the mesh
			// is watertight — a leaky mesh has no defined enclosed volume.
			let (mut mesh, mesh_format) = read_mesh_file(op_id, input_base, out_dir, &file)?;
			if heal {
				// The kernel's deterministic import repair: cap boundary loops, then
				// split non-manifold junctions (never worse than the input).
				mesh.fill_holes();
				mesh = make_manifold(&mesh);
			}
			let report = check_mesh(&mesh);
			if heal && !report.watertight {
				return Err(err(
					ErrorKind::InvalidGeometry,
					format!(
						"op '{op_id}': '{file}' is still not watertight after healing (boundary_edges={}, non_manifold_edges={}, non_orientable_edges={}) — route it through the voxel half instead (e.g. `mesh_carve` re-meshes watertight) or repair it upstream",
						report.boundary_edges, report.non_manifold_edges, report.non_orientable_edges
					),
				));
			}
			let bb = mesh.aabb();
			let mut m = serde_json::Map::new();
			m.insert("format".into(), json!(mesh_format));
			m.insert("triangles".into(), json!(mesh.triangle_count()));
			m.insert("healed".into(), json!(heal));
			mesh_receipt(&mut m, &report);
			m.insert("bbox_min".into(), json!([bb.min.x, bb.min.y, bb.min.z]));
			m.insert("bbox_max".into(), json!([bb.max.x, bb.max.y, bb.max.z]));
			if report.watertight {
				m.insert("volume".into(), json!(mesh.signed_volume()));
			}
			let written = match out {
				Some(f) => Some(write_mesh_healed(op_id, out_dir, &f, &mesh)?),
				None => None,
			};
			// `import_mesh` BINDS the mesh it read: a print file that came from
			// anywhere — this engine's voxel route, another tool, a repaired STL —
			// becomes gateable with the ordinary measures.
			Ok(Outcome { value: Some(EnvValue::Mesh(mesh.clone())), measures: Some(Value::Object(m)), file: written })
		}
		OpKind::MeshCarve { input, file, bool_op, voxel, out } => {
			// The hybrid solid∘mesh boolean: the bound solid is meshed on the honest
			// exact-else-heal route, the mesh file is welded in, and both are lifted
			// into winding-number SDFs and re-meshed by the voxel boolean. The result
			// is GUARANTEED a closed 2-manifold, but the seam is VOXEL-RESAMPLED —
			// accurate to `voxel`, never exact — hence route "voxel_implicit".
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			if !(voxel.is_finite() && voxel > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': voxel must be a positive voxel size in mm")));
			}
			let (a, _, _) = solid_mesh(s, 0.01, voxel);
			let (b, _) = read_mesh_file(op_id, input_base, out_dir, &file)?;
			// The boolean's lattice spans BOTH operands — same allocation cap as `shell`.
			grid_guard(op_id, "mesh_carve", a.aabb().union(b.aabb()).pad(2.0 * voxel as f32), voxel)?;
			let op = match bool_op {
				BoolOpSpec::Union => BoolOp::Union,
				BoolOpSpec::Difference => BoolOp::Difference,
				BoolOpSpec::Intersection => BoolOp::Intersection,
			};
			let mesh = mesh_boolean_implicit(&a, &b, op, voxel);
			if mesh.triangle_count() == 0 || !mesh.is_watertight() {
				return Err(err(
					ErrorKind::InvalidGeometry,
					format!(
						"op '{op_id}': mesh_carve produced no watertight result (triangles={}) — an empty boolean (e.g. an intersection of disjoint parts) or a voxel ({voxel} mm) too coarse to resolve the operands",
						mesh.triangle_count()
					),
				));
			}
			let path = write_mesh_healed(op_id, out_dir, &out, &mesh)?;
			Ok(Outcome {
				value: Some(EnvValue::Mesh(mesh.clone())),
				measures: Some(json!({
					"route": "voxel_implicit",
					"triangles": mesh.triangle_count(),
					"watertight": true,
					"volume": mesh.signed_volume(),
					"voxel": voxel,
				})),
				file: Some(path),
			})
		}

		OpKind::MeasureDimension { input, kind, a, b, near } => {
			// FRICTION #21: one dimension, exact where the analytic tags allow,
			// with the receipts a drawing callout needs. Face selection is by
			// nearest face-polygon centroid to the witness — deterministic and
			// the same anchor `list_faces` reports.
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let pick_face = |witness: [f64; 3]| -> (usize, kernel_brep::topo::FaceId, DVec3) {
				let w = DVec3::new(witness[0], witness[1], witness[2]);
				let mut best = None;
				for (i, fid) in s.faces().enumerate() {
					let c = polygon_centroid(&s.face_polygon(fid));
					let d = (c - w).length();
					if best.as_ref().map(|&(_, _, _, bd)| d < bd).unwrap_or(true) {
						best = Some((i, fid, c, d));
					}
				}
				let (i, fid, c, _) = best.expect("a bound solid has faces");
				(i, fid, c)
			};
			match kind.as_str() {
				"point_point" => {
					let (pa, pb) = match (a, b) {
						(Some(pa), Some(pb)) => (pa, pb),
						_ => {
							return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': kind 'point_point' needs both 'a' and 'b' points")));
						}
					};
					let va = DVec3::new(pa[0], pa[1], pa[2]);
					let vb = DVec3::new(pb[0], pb[1], pb[2]);
					Ok(Outcome::measures(json!({
						"kind": "point_point",
						"value": (va - vb).length(),
						"provenance": "coordinates",
						"a": pa, "b": pb,
						"delta": v3a(vb - va),
					})))
				}
				"face_face" => {
					let (wa, wb) = match (a, b) {
						(Some(wa), Some(wb)) => (wa, wb),
						_ => {
							return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': kind 'face_face' needs witness points 'a' and 'b'")));
						}
					};
					let (ia, fa, ca) = pick_face(wa);
					let (ib, fb, cb) = pick_face(wb);
					let plane = |fid| match s.face(fid).surface {
						kernel_brep::Surface::Plane { origin, normal } => Some((origin, normal.normalize())),
						_ => None,
					};
					let (Some((oa, na)), Some((ob, nb))) = (plane(fa), plane(fb)) else {
						let ty = |fid| match s.face(fid).surface {
							kernel_brep::Surface::Plane { .. } => "plane",
							kernel_brep::Surface::Cylinder { .. } => "cylinder",
							kernel_brep::Surface::Sphere { .. } => "sphere",
							kernel_brep::Surface::Cone { .. } => "cone",
							kernel_brep::Surface::Torus { .. } => "torus",
						};
						return Err(err(
							ErrorKind::InvalidParam,
							format!(
								"op '{op_id}': face_face needs two PLANAR faces; the witnesses selected {} (face {ia}) and {} (face {ib}) — move the witnesses or use 'diameter' for curved faces",
								ty(fa),
								ty(fb)
							),
						));
					};
					let align = na.dot(nb).abs();
					if align < 1.0 - 1e-9 {
						let angle_deg = align.clamp(-1.0, 1.0).acos().to_degrees();
						return Err(err(
							ErrorKind::InvalidParam,
							format!(
								"op '{op_id}': face_face needs PARALLEL planes; faces {ia} and {ib} meet at {angle_deg:.4}° — a between-planes distance is not defined"
							),
						));
					}
					Ok(Outcome::measures(json!({
						"kind": "face_face",
						"value": (ob - oa).dot(na).abs(),
						"provenance": "analytic",
						"face_a": {"index": ia, "point": v3a(oa), "normal": v3a(na), "witness": v3a(ca)},
						"face_b": {"index": ib, "point": v3a(ob), "normal": v3a(nb), "witness": v3a(cb)},
					})))
				}
				"diameter" => {
					let Some(w) = near else {
						return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': kind 'diameter' needs a 'near' witness point")));
					};
					let (i, fid, c) = pick_face(w);
					match s.face(fid).surface {
						kernel_brep::Surface::Cylinder { origin, axis, radius } => Ok(Outcome::measures(json!({
							"kind": "diameter",
							"value": 2.0 * radius,
							"provenance": "analytic",
							"face": {"index": i, "type": "cylinder", "point": v3a(origin), "axis": v3a(axis.normalize()), "radius": radius, "witness": v3a(c)},
						}))),
						kernel_brep::Surface::Sphere { center, radius } => Ok(Outcome::measures(json!({
							"kind": "diameter",
							"value": 2.0 * radius,
							"provenance": "analytic",
							"face": {"index": i, "type": "sphere", "center": v3a(center), "radius": radius, "witness": v3a(c)},
						}))),
						kernel_brep::Surface::Cone { half_angle, .. } => Err(err(
							ErrorKind::InvalidParam,
							format!(
								"op '{op_id}': the face nearest 'near' (face {i}) is a CONE (half-angle {half_angle:.4} rad) — its Ø varies along the axis; measure a point_point at a chosen station instead"
							),
						)),
						kernel_brep::Surface::Torus { major, minor, .. } => Err(err(
							ErrorKind::InvalidParam,
							format!(
								"op '{op_id}': the face nearest 'near' (face {i}) is a TORUS (major {major}, minor {minor}) — name the circle you mean via point_point instead"
							),
						)),
						kernel_brep::Surface::Plane { .. } => Err(err(
							ErrorKind::InvalidParam,
							format!("op '{op_id}': the face nearest 'near' (face {i}) is a PLANE — 'diameter' needs a cylindrical or spherical face; move the witness onto the bore/boss wall"),
						)),
					}
				}
				other => Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': kind must be 'point_point' / 'face_face' / 'diameter', got {other:?}"),
				)),
			}
		}

		OpKind::Tpms { kind, min, max, cell, mode, level, voxel, file } => {
			// One vocabulary: build the `implicit` tree's `tpms` leaf verbatim and
			// run it through the SAME parser — kind strings, mode/level semantics,
			// bounds checks and the `primitive_bound` field-quality wrapping all
			// come from one place (kernel-api/implicit.rs), not a twin re-encoding.
			if !(voxel.is_finite() && voxel > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': voxel must be a positive voxel size in mm")));
			}
			let mut leaf = json!({
				"shape": "tpms",
				"kind": kind,
				"min": min,
				"max": max,
				"cell": cell,
			});
			if let Some(mode) = &mode {
				leaf["mode"] = json!(mode);
			}
			if let Some(level) = level {
				leaf["level"] = json!(level);
			}
			// A raw TPMS is an OPEN labyrinth (the region box cuts its tubes) —
			// clamp with the same box so the block is a closed solid, exactly like
			// `Feature::Gyroid` / the damper acceptance idiom.
			let tree = json!({
				"op": "intersection",
				"a": leaf,
				"b": {"shape": "box", "min": min, "max": max},
			});
			let parsed = implicit::parse_tree(op_id, &tree, input_base)?;
			let b = parsed.node.bounds();
			if !b.is_valid() {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': the lattice block is empty — 'min' must be strictly below 'max' on every axis"),
				));
			}
			let domain = b.pad(3.0 * voxel as f32);
			grid_guard(op_id, "tpms", domain, voxel)?;
			let mut mesh = manifold_dual_contour(&parsed.node, domain, Resolution::VoxelSize(voxel as f32));
			let mut healed = false;
			if !mesh.is_watertight() || check_mesh(&mesh).non_manifold_edges > 0 {
				mesh = make_manifold(&mesh);
				healed = true;
			}
			let report = check_mesh(&mesh);
			if mesh.triangle_count() == 0 || !mesh.is_watertight() || report.non_manifold_edges > 0 {
				return Err(err(
					ErrorKind::InvalidGeometry,
					format!(
						"op '{op_id}': the {kind} lattice did not mesh watertight at voxel {voxel} (triangles={}, watertight={}, non_manifold_edges={}) — refine the voxel (walls need ≥ ~3 voxels) or thicken the sheet/level",
						mesh.triangle_count(),
						mesh.is_watertight(),
						report.non_manifold_edges
					),
				));
			}
			let path = write_mesh_healed(op_id, out_dir, &file, &mesh)?;
			Ok(Outcome {
				value: Some(EnvValue::Mesh(mesh.clone())),
				measures: Some(json!({
					"route": "voxel_implicit",
					"kind": kind,
					"mode": mode.as_deref().unwrap_or("network"),
					"triangles": mesh.triangle_count(),
					"watertight": true,
					"healed": healed,
					"volume": mesh.signed_volume(),
					"voxel": voxel,
				})),
				file: Some(path),
			})
		}

		OpKind::HybridBoolean { input, field, file, bool_op, voxel, out } => {
			// The flagship convergence op: exact B-rep × (implicit field | mesh),
			// exact wherever untouched, honest voxel fallback otherwise — a thin
			// wire over `kernel_model::hybrid_boolean`, which measures the per-face
			// accounting ON THE RESULT (nothing here asserts what the kernel did
			// not verify).
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			if !(voxel.is_finite() && voxel > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': voxel must be a positive voxel size in mm")));
			}
			// Realize the non-B-rep operand. Exactly one of `field` / `file`.
			let (field_node, operand_mesh, operand_label) = match (field, file) {
				(Some(expr), None) => {
					let parsed = implicit::parse_tree(op_id, &expr, input_base)?;
					let b = parsed.node.bounds();
					if !b.is_valid() || !b.min.is_finite() || !b.max.is_finite() {
						return Err(err(
							ErrorKind::InvalidParam,
							format!(
								"op '{op_id}': the 'field' operand is unbounded or empty — clamp it (intersect with a box / give expr_sdf bounds) so it can be meshed"
							),
						));
					}
					implicit::probe_fields(op_id, &parsed.fields, b)?;
					(Some(parsed.node), None, "implicit_field")
				}
				(None, Some(f)) => {
					let (m, fmt) = read_mesh_file(op_id, input_base, out_dir, &f)?;
					if m.triangle_count() == 0 {
						return Err(err(ErrorKind::InvalidGeometry, format!("op '{op_id}': the mesh operand '{f}' ({fmt}) has no triangles")));
					}
					(None, Some(m), "mesh_file")
				}
				_ => {
					return Err(err(
						ErrorKind::InvalidParam,
						format!("op '{op_id}': exactly one of 'field' (implicit CSG tree) or 'file' (mesh path) is required"),
					));
				}
			};
			// The healed fallback lifts BOTH operands onto one voxel lattice — cap
			// the allocation up front like `mesh_carve` / `shell`.
			let (smin, smax) = s.aabb();
			let mut domain = Aabb::new(
				Vec3::new(smin.x as f32, smin.y as f32, smin.z as f32),
				Vec3::new(smax.x as f32, smax.y as f32, smax.z as f32),
			);
			if let Some(node) = &field_node {
				domain = domain.union(node.bounds());
			}
			if let Some(m) = &operand_mesh {
				domain = domain.union(m.aabb());
			}
			grid_guard(op_id, "hybrid_boolean", domain.pad(2.0 * voxel as f32), voxel)?;
			let op = match bool_op {
				BoolOpSpec::Union => BooleanOp::Union,
				BoolOpSpec::Difference => BooleanOp::Difference,
				BoolOpSpec::Intersection => BooleanOp::Intersection,
			};
			let operand = match (&field_node, &operand_mesh) {
				(Some(node), None) => HybridOperand::Node(node),
				(None, Some(m)) => HybridOperand::Mesh(m),
				_ => unreachable!("exactly one operand was selected above"),
			};
			let result = hybrid_boolean(s, operand, op, voxel as f32).map_err(|e| match e {
				HybridError::UnboundedField => err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': the field operand has unbounded extent — intersect it with a finite region node first"),
				),
				HybridError::NotWatertight { detail } => err(
					ErrorKind::InvalidGeometry,
					format!("op '{op_id}': hybrid_boolean produced no watertight result on either route — {detail}; result withheld"),
				),
			})?;
			let (route, healed_reason) = match &result.route {
				HybridRoute::ExactStitch => ("exact_stitch", None),
				HybridRoute::Healed { reason } => ("voxel_healed", Some(reason.clone())),
			};
			// Route-aware write: the healed route reports crossings instead of
			// refusing on them; the exact stitch keeps the strict predicate.
			let path = match &result.route {
				HybridRoute::ExactStitch => write_mesh_auto(op_id, out_dir, &out, &result.mesh)?,
				HybridRoute::Healed { .. } => write_mesh_healed(op_id, out_dir, &out, &result.mesh)?,
			};
			let r = &result.report;
			let mut measures = json!({
				"route": route,
				"operand": operand_label,
				"triangles": result.mesh.triangle_count(),
				"watertight": true,
				"volume": result.mesh.signed_volume(),
				"voxel": voxel,
				// Per-face convergence receipts, measured on the result: every input
				// B-rep face lands in exactly one bucket.
				"brep_faces": r.brep_faces,
				"kept_exact": r.kept_exact,
				"kept_exact_curved": r.kept_exact_curved,
				"retiled": r.retiled,
				"trimmed": r.trimmed,
				"consumed": r.consumed,
				"operand_triangles": r.operand_triangles,
			});
			if let Some(reason) = healed_reason {
				measures["healed_reason"] = json!(reason);
			}
			Ok(Outcome { value: Some(EnvValue::Mesh(result.mesh)), measures: Some(measures), file: Some(path) })
		}

		// --- Parts library (curated, admission-gated; BAR.md I7) -------------------------------
		OpKind::LibraryAdd { dir, part, part_file, meta } => {
			let part_json = match (part, part_file) {
				// An inline envelope object; a JSON string is accepted too and
				// treated as the raw envelope text (e.g. pasted file contents).
				(Some(Value::String(text)), None) => text,
				(Some(value), None) => value.to_string(),
				(None, Some(file)) => {
					let path = resolve_input_path(op_id, out_dir, &file)?;
					std::fs::read_to_string(&path)
						.map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot read '{}': {e}", path.display())))?
				}
				_ => {
					return Err(err(
						ErrorKind::InvalidParam,
						format!("op '{op_id}': exactly one of 'part' (inline .lmcpart envelope) or 'part_file' (path to one) is required"),
					));
				}
			};
			let mut library = open_library(op_id, out_dir, &dir)?;
			let entry = library
				.add(&part_json, to_kernel_meta(meta), &AddOptions::default())
				.map_err(|e| map_admission_error(op_id, e))?;
			Ok(Outcome::measures(json!({
				"name": entry.name,
				"version": entry.version,
				"file": entry.file,
				"gate_samples": entry.admitted.samples.len(),
				"gate_rebuilds": entry.admitted.rebuilds,
				// The gate's first sample is always the interface defaults.
				"volume_at_defaults": entry.admitted.samples.first().map(|s| s.volume),
			})))
		}
		OpKind::LibrarySearch { dir, text, tags } => {
			let library = open_library(op_id, out_dir, &dir)?;
			let matches: Vec<Value> = library
				.search(&text, &tags)
				.into_iter()
				.map(|e| {
					json!({
						"name": e.name,
						"version": e.version,
						"category": e.category,
						"tags": e.tags,
						"description": e.description,
						"params": e
							.param_interface
							.iter()
							.map(|p| json!({ "name": p.name, "units": p.units, "default": p.default, "min": p.min, "max": p.max }))
							.collect::<Vec<_>>(),
					})
				})
				.collect();
			Ok(Outcome::measures(json!({ "matches": matches })))
		}
		OpKind::LibraryInstantiate { dir, name, version, params } => {
			let library = open_library(op_id, out_dir, &dir)?;
			let built = library
				.instantiate(&name, version, &params)
				.map_err(|e| map_library_error(op_id, "library_instantiate", e))?;
			let solid = built.document.evaluate_brep().ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': the entry's feature tree produced no exact B-rep (voxel-half-only features — shell, gyroid, smooth booleans — cannot enter the solid environment)"),
				)
			})?;
			let outcome = bind_solid(op_id, "library_instantiate", solid)?;
			let mut measures = json!({
				"name": built.name,
				"version": built.version,
				"deprecated": built.deprecated,
				"params": params,
			});
			if built.deprecated {
				// BAR.md I7: a deprecated entry still builds, but instantiate WARNS.
				measures["warning"] = json!(format!(
					"'{}' v{} is deprecated — it still builds, but curation retired it; search the library for a successor",
					built.name, built.version
				));
			}
			Ok(Outcome { measures: Some(measures), ..outcome })
		}
		OpKind::LibraryDeprecate { dir, name } => {
			let mut library = open_library(op_id, out_dir, &dir)?;
			let count = library.deprecate(&name).map_err(|e| map_library_error(op_id, "library_deprecate", e))?;
			Ok(Outcome::measures(json!({ "name": name, "deprecated_versions": count })))
		}
		OpKind::LibraryRemove { dir, name, force } => {
			let mut library = open_library(op_id, out_dir, &dir)?;
			let removed = library.remove(&name, force).map_err(|e| map_library_error(op_id, "library_remove", e))?;
			Ok(Outcome::measures(json!({ "name": name, "removed_files": removed, "forced": force })))
		}

		// --- Standard parts catalog ---------------------------------------------------------------
		OpKind::SpurGear { module, teeth, face_width, bore, pressure_angle_deg, keyway } => {
			let key = if keyway {
				Some(parts::din6885_key_size(bore).ok_or_else(|| {
					err(
						ErrorKind::InvalidParam,
						format!("op '{op_id}': spur_gear: no DIN 6885-1 key size for a {bore} mm bore (table covers over 6 up to 75 mm)"),
					)
				})?)
			} else {
				None
			};
			bind_solid(op_id, "spur_gear", parts::spur_gear(module, teeth, face_width, bore, pressure_angle_deg, key))
		}
		OpKind::HexBolt { m, length } => {
			let solid = parts::hex_bolt_iso4017(m, length).ok_or_else(|| size_err(op_id, "hex_bolt", "ISO 4017", m, FASTENER_SIZES))?;
			bind_solid(op_id, "hex_bolt", solid)
		}
		OpKind::HexNut { m } => {
			let solid = parts::hex_nut_iso4032(m).ok_or_else(|| size_err(op_id, "hex_nut", "ISO 4032", m, FASTENER_SIZES))?;
			bind_solid(op_id, "hex_nut", solid)
		}
		OpKind::Washer { m } => {
			let solid = parts::washer_iso7089(m).ok_or_else(|| size_err(op_id, "washer", "ISO 7089", m, FASTENER_SIZES))?;
			bind_solid(op_id, "washer", solid)
		}
		OpKind::SocketHeadCapScrew { m, length } => {
			let solid = parts::socket_head_cap_screw(m, length)
				.ok_or_else(|| size_err(op_id, "socket_head_cap_screw", "DIN 912", m, FASTENER_SIZES))?;
			bind_solid(op_id, "socket_head_cap_screw", solid)
		}
		OpKind::Gt2Pulley { teeth, belt_width, bore, flanged } => {
			bind_solid(op_id, "gt2_pulley", parts::gt2_pulley(teeth, belt_width, bore, flanged))
		}
		OpKind::ChainSprocket { pitch, roller_d, teeth, bore } => {
			bind_solid(op_id, "chain_sprocket", parts::chain_sprocket(pitch, roller_d, teeth, bore))
		}
		OpKind::Shaft { d, length, keyway } => {
			let keyway = match keyway {
				None => None,
				Some(spec) => {
					let size = parts::din6885_key_size(d).ok_or_else(|| {
						err(
							ErrorKind::InvalidParam,
							format!("op '{op_id}': shaft: no DIN 6885-1 key size for a {d} mm shaft (table covers over 6 up to 75 mm)"),
						)
					})?;
					Some(parts::ShaftKeyway { size, length: spec.length, offset: spec.offset })
				}
			};
			bind_solid(op_id, "shaft", parts::shaft(d, length, keyway))
		}

		OpKind::ParallelKey { b, h, l } => bind_solid(op_id, "parallel_key", parts::parallel_key(b, h, l)),
		OpKind::DowelPin { d, length } => {
			let solid = parts::dowel_pin(d, length).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': dowel_pin: Ø{d} ×{length} — Ø must be an ISO 2338 size (1, 1.5, 2, 2.5, 3, 4, 5, 6, 8, 10, 12) and the length must exceed the two 0.2·d chamfers"),
				)
			})?;
			bind_solid(op_id, "dowel_pin", solid)
		}
		OpKind::CirclipExternal { shaft_d } => {
			let solid = parts::circlip_external(shaft_d).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': circlip_external: Ø{shaft_d} is not in the DIN 471 table (supported: {DIN471_SIZES})"),
				)
			})?;
			bind_solid(op_id, "circlip_external", solid)
		}
		OpKind::CirclipInternal { bore_d } => {
			let solid = parts::circlip_internal(bore_d).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': circlip_internal: Ø{bore_d} is not in the DIN 472 table (supported: {DIN472_SIZES})"),
				)
			})?;
			bind_solid(op_id, "circlip_internal", solid)
		}
		OpKind::FlatHeadScrew { m, length } => {
			let solid = parts::flat_head_screw(m, length).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': flat_head_screw: M{m}×{length} — M{m} must be an ISO 10642 size ({FASTENER_SIZES}) and the overall length must contain the head cone and socket"),
				)
			})?;
			bind_solid(op_id, "flat_head_screw", solid)
		}
		OpKind::ButtonHeadScrew { m, length } => {
			let solid = parts::button_head_screw(m, length).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': button_head_screw: M{m}×{length} — M{m} must be an ISO 7380 size ({SCREW_SIZES_M3_M12}) and the length positive"),
				)
			})?;
			bind_solid(op_id, "button_head_screw", solid)
		}
		OpKind::SetScrew { m, length } => {
			let solid = parts::set_screw(m, length).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': set_screw: M{m}×{length} — M{m} must be a DIN 916 size ({SCREW_SIZES_M3_M12}) and the length must hold the cup, socket and a 0.5 mm web"),
				)
			})?;
			bind_solid(op_id, "set_screw", solid)
		}
		OpKind::LockNut { m } => {
			let solid = parts::lock_nut(m).ok_or_else(|| size_err(op_id, "lock_nut", "DIN 985", m, FASTENER_SIZES))?;
			bind_solid(op_id, "lock_nut", solid)
		}
		OpKind::ThreadedRod { m, length } => {
			let solid = parts::threaded_rod(m, length).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': threaded_rod: M{m}×{length} — M{m} must be an ISO 261 coarse size ({FASTENER_SIZES}) and the length must exceed the two half-pitch chamfers"),
				)
			})?;
			bind_solid(op_id, "threaded_rod", solid)
		}
		OpKind::Standoff { m, length } => {
			let solid = parts::standoff(m, length).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': standoff: M{m}×{length} — M{m} must be a standoff size ({SMALL_SIZES_M2_M6}) and the length positive"),
				)
			})?;
			bind_solid(op_id, "standoff", solid)
		}
		OpKind::CompressionSpring { wire_d, outer_d, pitch, turns } => {
			let solid = parts::compression_spring(wire_d, outer_d, pitch, turns).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': compression_spring: needs wire_d > 0, outer_d > 2·wire_d, turns > 0 and pitch > wire_d (touching coils would self-intersect)"),
				)
			})?;
			bind_solid(op_id, "compression_spring", solid)
		}
		OpKind::Extrusion2020 { length } => bind_solid(op_id, "extrusion_2020", parts::extrusion_2020(length)),
		OpKind::Extrusion3030 { length } => bind_solid(op_id, "extrusion_3030", parts::extrusion_3030(length)),
		OpKind::Tnut2020 {} => bind_solid(op_id, "tnut_2020", parts::tnut_2020()),
		OpKind::ORing { dash } => {
			let solid = parts::o_ring(dash).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': o_ring: dash -{dash} is not an AS568 table size (supported: {AS568_DASHES})"),
				)
			})?;
			bind_solid(op_id, "o_ring", solid)
		}
		OpKind::ORingCord { ring_id, cord_d } => {
			let solid = parts::o_ring_cord(ring_id, cord_d).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': o_ring_cord: needs a positive finite ring_id and a stocked metric cord Ø ({METRIC_CORD_SIZES}); got Ø{ring_id} × {cord_d}"),
				)
			})?;
			bind_solid(op_id, "o_ring_cord", solid)
		}
		OpKind::JawCouplingHub { od, bore } => {
			let solid = parts::jaw_coupling_hub(od, bore).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': jaw_coupling_hub: OD {od} must be a coupling size ({JAW_COUPLING_SIZES}) and bore Ø{bore} within that row's range"),
				)
			})?;
			bind_solid(op_id, "jaw_coupling_hub", solid)
		}
		OpKind::JawCouplingSpider { od } => {
			let solid = parts::jaw_coupling_spider(od).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': jaw_coupling_spider: OD {od} is not a coupling size ({JAW_COUPLING_SIZES})"),
				)
			})?;
			bind_solid(op_id, "jaw_coupling_spider", solid)
		}
		OpKind::SetScrewCoupling { bore1, bore2 } => {
			let solid = parts::set_screw_coupling(bore1, bore2).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': set_screw_coupling: both bores (Ø{bore1} × Ø{bore2}) must be stocked sizes ({SET_SCREW_COUPLING_BORES})"),
				)
			})?;
			bind_solid(op_id, "set_screw_coupling", solid)
		}
		OpKind::ClampCoupling { bore1, bore2 } => {
			let solid = parts::clamp_coupling(bore1, bore2).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': clamp_coupling: both bores (Ø{bore1} × Ø{bore2}) must be stocked sizes ({CLAMP_COUPLING_BORES})"),
				)
			})?;
			bind_solid(op_id, "clamp_coupling", solid)
		}
		OpKind::LinearBearingLmuu { bore } => {
			let solid = parts::linear_bearing_lmuu(bore).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': linear_bearing_lmuu: bore Ø{bore} must be 8 (LM8UU) or 12 (LM12UU)"),
				)
			})?;
			bind_solid(op_id, "linear_bearing_lmuu", solid)
		}
		OpKind::Sc8uuBlock {} => bind_solid(op_id, "sc8uu_block", parts::sc8uu_block()),
		OpKind::ShaftSupportSk8 {} => bind_solid(op_id, "shaft_support_sk8", parts::shaft_support_sk8()),
		OpKind::ShaftSupportShf8 {} => bind_solid(op_id, "shaft_support_shf8", parts::shaft_support_shf8()),
		OpKind::Mgn12Rail { length } => {
			let solid = parts::mgn12_rail(length).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': mgn12_rail: length ({length}) must be finite and at least one 25 mm hole pitch"),
				)
			})?;
			bind_solid(op_id, "mgn12_rail", solid)
		}
		OpKind::Mgn12Carriage {} => bind_solid(op_id, "mgn12_carriage", parts::mgn12_carriage()),
		OpKind::DeepGrooveBearing { designation } => {
			let solid = parts::deep_groove_bearing(&designation).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': deep_groove_bearing: '{designation}' is not in the seat table (603, 608, 625, 688, 6000, 6001, 6804)"),
				)
			})?;
			bind_solid(op_id, "deep_groove_bearing", solid)
		}
		OpKind::FlangedBearing { designation } => {
			let solid = parts::flanged_bearing(&designation).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': flanged_bearing: '{designation}' must be F608 or F623"),
				)
			})?;
			bind_solid(op_id, "flanged_bearing", solid)
		}
		OpKind::ThrustBearing { designation } => {
			let solid = parts::thrust_bearing(&designation).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': thrust_bearing: '{designation}' must be 51100 or 51101"),
				)
			})?;
			bind_solid(op_id, "thrust_bearing", solid)
		}
		OpKind::Kp08PillowBlock {} => bind_solid(op_id, "kp08_pillow_block", parts::kp08_pillow_block()),
		OpKind::PipeBossG { designation, wall, length } => {
			let solid = parts::pipe_boss_g(&designation, wall, length).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': pipe_boss_g: '{designation}' must be G1/8…G1/2, wall ({wall}) ≥ 1, length ({length}) past chamfer + pitch"),
				)
			})?;
			bind_solid(op_id, "pipe_boss_g", solid)
		}
		OpKind::HoseBarb { hose_id, barbs } => {
			let solid = parts::hose_barb(hose_id, barbs).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': hose_barb: hose_id (Ø{hose_id}) must be positive finite and barbs ({barbs}) ≥ 1"),
				)
			})?;
			bind_solid(op_id, "hose_barb", solid)
		}
		OpKind::ShoulderBolt { shoulder_d, shoulder_len } => {
			let solid = parts::shoulder_bolt(shoulder_d, shoulder_len).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': shoulder_bolt: shoulder Ø{shoulder_d} must be an ISO 7379 size (6.5, 8, 10, 13, 16) and shoulder_len ({shoulder_len}) positive finite"),
				)
			})?;
			bind_solid(op_id, "shoulder_bolt", solid)
		}
		OpKind::SpringWasher { m } => {
			let solid = parts::spring_washer(m).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': spring_washer: M{m} is outside the DIN 127 B table (M3–M12)"),
				)
			})?;
			bind_solid(op_id, "spring_washer", solid)
		}
		OpKind::LeadScrewTr8 { length, lead } => {
			let solid = parts::lead_screw_tr8(length, lead).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': lead_screw_tr8: lead {lead} must be a Tr8 variant (2, 4, 8 — all pitch 2) and length ({length}) > one pitch"),
				)
			})?;
			bind_solid(op_id, "lead_screw_tr8", solid)
		}
		OpKind::LeadScrewNutTr8 {} => bind_solid(op_id, "lead_screw_nut_tr8", parts::lead_screw_nut_tr8()),
		OpKind::NemaMotor { frame, body_len } => {
			let solid = parts::nema_motor(frame, body_len).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': nema_motor: frame {frame} must be a NEMA table size ({NEMA_FRAMES}) and body_len ({body_len}) positive"),
				)
			})?;
			bind_solid(op_id, "nema_motor", solid)
		}
		OpKind::NemaMountPlate { frame, thickness, margin } => {
			let solid = parts::nema_mount_plate(frame, thickness, margin).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': nema_mount_plate: frame {frame} must be a NEMA table size ({NEMA_FRAMES}), thickness ({thickness}) positive and margin ({margin}) ≥ 0"),
				)
			})?;
			bind_solid(op_id, "nema_mount_plate", solid)
		}
		OpKind::GearRack { module, length, width, pressure_angle_deg } => {
			let solid = parts::gear_rack(module, length, width, pressure_angle_deg).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': gear_rack: needs positive dimensions, a pressure angle in (0°, 32°), and a bar long enough for one whole tooth"),
				)
			})?;
			bind_solid(op_id, "gear_rack", solid)
		}
		OpKind::InternalGear { module, teeth, face_width, rim_od, pressure_angle_deg } => {
			let solid = parts::internal_gear(module, teeth, face_width, rim_od, pressure_angle_deg).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': internal_gear: needs teeth ≥ 8, rim_od > m·(teeth + 2.5) (the root circle), positive dimensions, and a pressure angle low enough to keep the root land open"),
				)
			})?;
			bind_solid(op_id, "internal_gear", solid)
		}

		// --- Standard feature cuts ------------------------------------------------------------------
		OpKind::HeatsetInsertBoss { input, at, axis, m } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid = parts::heatset_insert_boss(s, dv3(at), dv3(axis), m).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': heatset_insert_boss: M{m} must be a heat-set insert size ({SMALL_SIZES_M2_M6}) and the axis non-zero"),
				)
			})?;
			bind_solid(op_id, "heatset_insert_boss", solid)
		}
		OpKind::CirclipGrooveExternal { input, at, axis, shaft_d } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid = parts::circlip_groove_external(s, dv3(at), dv3(axis), shaft_d).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': circlip_groove_external: Ø{shaft_d} must be a DIN 471 size ({DIN471_SIZES}) and the axis non-zero"),
				)
			})?;
			bind_solid(op_id, "circlip_groove_external", solid)
		}
		OpKind::CirclipGrooveInternal { input, at, axis, bore_d } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid = parts::circlip_groove_internal(s, dv3(at), dv3(axis), bore_d).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': circlip_groove_internal: Ø{bore_d} must be a DIN 472 size ({DIN472_SIZES}) and the axis non-zero"),
				)
			})?;
			bind_solid(op_id, "circlip_groove_internal", solid)
		}
		OpKind::ORingGroove { input, at, axis, dash } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid = parts::o_ring_groove(s, dv3(at), dv3(axis), dash).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': o_ring_groove: dash -{dash} must be an AS568 table size ({AS568_DASHES}) and the axis non-zero"),
				)
			})?;
			bind_solid(op_id, "o_ring_groove", solid)
		}
		OpKind::ORingFaceGland { input, at, axis, gland_center_d, cord_d } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid = parts::o_ring_face_gland(s, dv3(at), dv3(axis), gland_center_d, cord_d).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': o_ring_face_gland: cord Ø{cord_d} must be a stocked metric size ({METRIC_CORD_SIZES}), the axis non-zero, and gland_center_d (Ø{gland_center_d}) wider than the groove"),
				)
			})?;
			let outcome = bind_solid(op_id, "o_ring_face_gland", solid)?;
			// Echo the gland dimensions the table chose (the FRICTION #9 lesson:
			// report what the cut used, so the seal stack can be posed without
			// re-reading kernel tables).
			let g = parts::metric_cord_gland(cord_d).expect("cord validated above");
			Ok(Outcome {
				measures: Some(json!({
					"gland_depth": g.gland_depth,
					"groove_width": g.groove_width,
					"squeeze": (g.cord_d - g.gland_depth) / g.cord_d,
					"fill": std::f64::consts::PI * (g.cord_d * 0.5) * (g.cord_d * 0.5) / (g.gland_depth * g.groove_width),
					"cord_length": std::f64::consts::PI * gland_center_d,
				})),
				..outcome
			})
		}
		OpKind::ORingFaceGlandRacetrack { input, at, axis, x_len, y_len, corner_r, cord_d } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid = parts::o_ring_face_gland_racetrack(s, dv3(at), dv3(axis), x_len, y_len, corner_r, cord_d).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': o_ring_face_gland_racetrack: cord Ø{cord_d} must be a stocked metric size ({METRIC_CORD_SIZES}), the axis non-zero, corner_r at least half the groove width, and 2·corner_r within both {x_len}×{y_len} sides"),
				)
			})?;
			let outcome = bind_solid(op_id, "o_ring_face_gland_racetrack", solid)?;
			let g = parts::metric_cord_gland(cord_d).expect("cord validated above");
			let cord_length = parts::racetrack_cord_length(x_len, y_len, corner_r).expect("path validated above");
			Ok(Outcome {
				measures: Some(json!({
					"gland_depth": g.gland_depth,
					"groove_width": g.groove_width,
					"squeeze": (g.cord_d - g.gland_depth) / g.cord_d,
					"fill": std::f64::consts::PI * (g.cord_d * 0.5) * (g.cord_d * 0.5) / (g.gland_depth * g.groove_width),
					"cord_length": cord_length,
				})),
				..outcome
			})
		}

		OpKind::Pc4Port { input, at, axis, m, through } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid = parts::pc4_port_cut(s, dv3(at), dv3(axis), m, through).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': pc4_port: m ({m}) must be 6 or 10, the axis non-zero, and 'through' ({through}) past the pocket depth"),
				)
			})?;
			bind_solid(op_id, "pc4_port", solid)
		}
		OpKind::TeardropHole { input, at, axis, up, d, through } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid = parts::teardrop_hole(s, dv3(at), dv3(axis), dv3(up), d, through).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': teardrop_hole: needs a non-zero axis, 'up' not parallel to it, and positive finite d ({d}) / through ({through})"),
				)
			})?;
			bind_solid(op_id, "teardrop_hole", solid)
		}
		OpKind::BridgedCounterbore { input, at, axis, m, through, bridge } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid = parts::bridged_counterbore(s, dv3(at), dv3(axis), m, through, bridge).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': bridged_counterbore: M{m} must be M2–M12 with a positive bridge ({bridge}) and through ({through}) > pocket + bridge"),
				)
			})?;
			bind_solid(op_id, "bridged_counterbore", solid)
		}
		OpKind::BoardMount { input, at, axis, board } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid = parts::board_mount_cut(s, dv3(at), dv3(axis), &board).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': board_mount: '{board}' must be rpi, arduino_uno, vesa75 or vesa100 (with a non-zero axis)"),
				)
			})?;
			bind_solid(op_id, "board_mount", solid)
		}
		OpKind::Tr8NutTrap { input, at, axis, through } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid = parts::tr8_nut_trap(s, dv3(at), dv3(axis), through).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': tr8_nut_trap: the axis must be non-zero and 'through' ({through}) must exceed the 3.7 mm flange recess"),
				)
			})?;
			bind_solid(op_id, "tr8_nut_trap", solid)
		}
		OpKind::NemaMountCut { input, at, axis, frame, through } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid = parts::nema_mount_cut(s, dv3(at), dv3(axis), frame, through).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': nema_mount_cut: frame {frame} must be a NEMA table size ({NEMA_FRAMES}), the axis non-zero and 'through' ({through}) positive"),
				)
			})?;
			bind_solid(op_id, "nema_mount_cut", solid)
		}
		OpKind::ServoPocket { input, at, axis, model, through } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid = parts::servo_pocket(s, dv3(at), dv3(axis), &model, through).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': servo_pocket: model '{model}' must be a servo table size ({SERVO_MODELS}), the axis non-zero and 'through' ({through}) positive"),
				)
			})?;
			bind_solid(op_id, "servo_pocket", solid)
		}

		// --- Design-math lookups ----------------------------------------------------------------------
		OpKind::Gt2Belt { center_distance, t1, t2 } => {
			let (pitch_length, belt_teeth) = parts::gt2_belt(center_distance, t1, t2).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': gt2_belt: needs t1, t2 ≥ 2 and center_distance beyond the pitch-radius sum (pitch Ø = teeth·2/π)"),
				)
			})?;
			Ok(Outcome::measures(json!({ "pitch_length": pitch_length, "belt_teeth": belt_teeth })))
		}
		OpKind::Gt2CenterDistance { belt_teeth, t1, t2 } => {
			let center_distance = parts::gt2_center_distance(belt_teeth, t1, t2).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': gt2_center_distance: needs t1, t2 ≥ 2 and a belt long enough to wrap both pulleys"),
				)
			})?;
			Ok(Outcome::measures(json!({ "center_distance": center_distance })))
		}
		OpKind::Iso286Fit { d, fit } => {
			let f = parts::iso286_fit(d, &fit).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': iso286_fit: '{fit}' at Ø{d} — supported fits are H7/g6, H7/h6, H7/k6, H7/n6, H7/p6, H7/s6, H8/f7 for 0 < d ≤ 120 mm"),
				)
			})?;
			Ok(Outcome::measures(json!({
				"hole": [f.hole.0, f.hole.1],
				"shaft": [f.shaft.0, f.shaft.1],
				"clearance": [f.clearance.0, f.clearance.1],
			})))
		}
		OpKind::HeatsetSpec { m } => {
			let spec = parts::heatset_spec(m).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': heatset_spec: M{m} is not a heat-set insert size ({SMALL_SIZES_M2_M6})"),
				)
			})?;
			// pocket/boss sizing rules per `heatset_insert_boss` (documented there):
			// pocket depth = insert length + 1 mm melt room; boss Ø = 2 × pilot.
			Ok(Outcome::measures(json!({
				"m": spec.m,
				"pilot_d": spec.pilot_d,
				"insert_length": spec.length,
				"pocket_depth": spec.length + 1.0,
				"boss_d": 2.0 * spec.pilot_d,
			})))
		}
		OpKind::MetricCordGland { cord_d } => {
			let g = parts::metric_cord_gland(cord_d).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': metric_cord_gland: Ø{cord_d} is not a stocked metric cord size (supported: {METRIC_CORD_SIZES})"),
				)
			})?;
			Ok(Outcome::measures(json!({
				"gland_depth": g.gland_depth,
				"groove_width": g.groove_width,
				"squeeze": (g.cord_d - g.gland_depth) / g.cord_d,
				"fill": std::f64::consts::PI * (g.cord_d * 0.5) * (g.cord_d * 0.5) / (g.gland_depth * g.groove_width),
			})))
		}
		OpKind::RacetrackCordLength { x_len, y_len, corner_r } => {
			let cord_length = parts::racetrack_cord_length(x_len, y_len, corner_r).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': racetrack_cord_length: needs positive finite sides with 2·corner_r ({}) within both ({x_len} × {y_len})", 2.0 * corner_r),
				)
			})?;
			Ok(Outcome::measures(json!({ "cord_length": cord_length })))
		}
		OpKind::PipeThreadG { designation } => {
			let g = parts::g_thread_spec(&designation).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': pipe_thread_g: '{designation}' is not stocked (G1/8, G1/4, G3/8, G1/2)"),
				)
			})?;
			Ok(Outcome::measures(json!({
				"major_d": g.major_d,
				"tpi": g.tpi,
				"pitch": g.pitch,
				"tap_drill_d": g.tap_drill_d,
			})))
		}

		// --- Hole wizard ----------------------------------------------------------------------------
		OpKind::Drill { input, at, axis, d, depth, through, segments } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let dep = hole_depth(op_id, depth, through)?;
			let solid = holes::drill(s, dv3(at), dv3(axis), d, dep, segments).map_err(|e| map_hole_error(op_id, "drill", e))?;
			let mut measures = serde_json::Map::new();
			measures.insert("d".into(), json!(d));
			depth_measures(&mut measures, d, dep);
			Ok(Outcome { measures: Some(Value::Object(measures)), ..bind_solid(op_id, "drill", solid)? })
		}
		OpKind::ClearanceHole { input, at, axis, m, fit, segments } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid = holes::clearance_hole(s, dv3(at), dv3(axis), m, to_kernel_fit(fit), segments)
				.map_err(|e| map_hole_error(op_id, "clearance_hole", e))?;
			let spec = metric_spec_row(m);
			let measures = json!({
				"m": m,
				"fit": fit_name(fit),
				"clearance_d": spec.clearance[to_kernel_fit(fit) as usize],
			});
			Ok(Outcome { measures: Some(measures), ..bind_solid(op_id, "clearance_hole", solid)? })
		}
		OpKind::CounterboreHole { input, at, axis, m, fit, segments } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid = holes::counterbore_hole(s, dv3(at), dv3(axis), m, to_kernel_fit(fit), segments)
				.map_err(|e| map_hole_error(op_id, "counterbore_hole", e))?;
			let spec = metric_spec_row(m);
			let measures = json!({
				"m": m,
				"fit": fit_name(fit),
				"clearance_d": spec.clearance[to_kernel_fit(fit) as usize],
				"counterbore_d": spec.counterbore_d,
				"counterbore_depth": spec.counterbore_depth,
			});
			Ok(Outcome { measures: Some(measures), ..bind_solid(op_id, "counterbore_hole", solid)? })
		}
		OpKind::CountersinkHole { input, at, axis, m, fit, segments } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid = holes::countersink_hole(s, dv3(at), dv3(axis), m, to_kernel_fit(fit), segments)
				.map_err(|e| map_hole_error(op_id, "countersink_hole", e))?;
			let spec = metric_spec_row(m);
			let measures = json!({
				"m": m,
				"fit": fit_name(fit),
				"clearance_d": spec.clearance[to_kernel_fit(fit) as usize],
				// the cut succeeded, so the form-F row exists (M3+)
				"countersink_d": spec.countersink_d.expect("countersink cut succeeded, so the form-F row exists"),
			});
			Ok(Outcome { measures: Some(measures), ..bind_solid(op_id, "countersink_hole", solid)? })
		}
		OpKind::TapDrillHole { input, at, axis, m, depth, through, segments } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let dep = hole_depth(op_id, depth, through)?;
			let solid =
				holes::tap_drill_hole(s, dv3(at), dv3(axis), m, dep, segments).map_err(|e| map_hole_error(op_id, "tap_drill_hole", e))?;
			let spec = metric_spec_row(m);
			let pilot_d = spec.m - spec.pitch;
			let mut measures = serde_json::Map::new();
			measures.insert("m".into(), json!(m));
			measures.insert("pitch".into(), json!(spec.pitch));
			measures.insert("pilot_d".into(), json!(pilot_d));
			depth_measures(&mut measures, pilot_d, dep);
			Ok(Outcome { measures: Some(Value::Object(measures)), ..bind_solid(op_id, "tap_drill_hole", solid)? })
		}
		OpKind::BoltCircle { input, center, axis, circle_d, n, start_deg, hole, segments } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			if !start_deg.is_finite() {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': start_deg must be finite")));
			}
			// Validate exclusive depth params up front (bolt_circle would surface
			// them per hole otherwise) and pre-build the per-hole measure echo.
			let axis_v = dv3(axis);
			let mut hole_measures = serde_json::Map::new();
			let solid = match hole {
				BoltHoleSpec::Drill { d, depth, through } => {
					let dep = hole_depth(op_id, depth, through)?;
					hole_measures.insert("hole".into(), json!("drill"));
					hole_measures.insert("d".into(), json!(d));
					depth_measures(&mut hole_measures, d, dep);
					holes::bolt_circle(s, dv3(center), axis_v, circle_d, n, start_deg.to_radians(), |acc, p| {
						holes::drill(&acc, p, axis_v, d, dep, segments)
					})
				}
				BoltHoleSpec::Clearance { m, fit } => {
					hole_measures.insert("hole".into(), json!("clearance"));
					hole_measures.insert("m".into(), json!(m));
					hole_measures.insert("fit".into(), json!(fit_name(fit)));
					holes::bolt_circle(s, dv3(center), axis_v, circle_d, n, start_deg.to_radians(), |acc, p| {
						holes::clearance_hole(&acc, p, axis_v, m, to_kernel_fit(fit), segments)
					})
				}
				BoltHoleSpec::Counterbore { m, fit } => {
					hole_measures.insert("hole".into(), json!("counterbore"));
					hole_measures.insert("m".into(), json!(m));
					hole_measures.insert("fit".into(), json!(fit_name(fit)));
					holes::bolt_circle(s, dv3(center), axis_v, circle_d, n, start_deg.to_radians(), |acc, p| {
						holes::counterbore_hole(&acc, p, axis_v, m, to_kernel_fit(fit), segments)
					})
				}
				BoltHoleSpec::Countersink { m, fit } => {
					hole_measures.insert("hole".into(), json!("countersink"));
					hole_measures.insert("m".into(), json!(m));
					hole_measures.insert("fit".into(), json!(fit_name(fit)));
					holes::bolt_circle(s, dv3(center), axis_v, circle_d, n, start_deg.to_radians(), |acc, p| {
						holes::countersink_hole(&acc, p, axis_v, m, to_kernel_fit(fit), segments)
					})
				}
				BoltHoleSpec::TapDrill { m, depth, through } => {
					let dep = hole_depth(op_id, depth, through)?;
					hole_measures.insert("hole".into(), json!("tap_drill"));
					hole_measures.insert("m".into(), json!(m));
					holes::bolt_circle(s, dv3(center), axis_v, circle_d, n, start_deg.to_radians(), |acc, p| {
						holes::tap_drill_hole(&acc, p, axis_v, m, dep, segments)
					})
				}
			}
			.map_err(|e| map_hole_error(op_id, "bolt_circle", e))?;
			// Echo the table row for metric cuts now that the cut proved m valid.
			match hole {
				BoltHoleSpec::Clearance { m, fit } | BoltHoleSpec::Counterbore { m, fit } | BoltHoleSpec::Countersink { m, fit } => {
					let spec = metric_spec_row(m);
					hole_measures.insert("clearance_d".into(), json!(spec.clearance[to_kernel_fit(fit) as usize]));
					if matches!(hole, BoltHoleSpec::Counterbore { .. }) {
						hole_measures.insert("counterbore_d".into(), json!(spec.counterbore_d));
						hole_measures.insert("counterbore_depth".into(), json!(spec.counterbore_depth));
					}
					if matches!(hole, BoltHoleSpec::Countersink { .. }) {
						hole_measures
							.insert("countersink_d".into(), json!(spec.countersink_d.expect("countersink cut succeeded, so the form-F row exists")));
					}
				}
				BoltHoleSpec::TapDrill { m, depth, through } => {
					let spec = metric_spec_row(m);
					let pilot_d = spec.m - spec.pitch;
					hole_measures.insert("pitch".into(), json!(spec.pitch));
					hole_measures.insert("pilot_d".into(), json!(pilot_d));
					// re-derive the (already validated) depth for the echo
					depth_measures(&mut hole_measures, pilot_d, hole_depth(op_id, depth, through)?);
				}
				BoltHoleSpec::Drill { .. } => {}
			}
			let measures = json!({
				"n": n,
				"circle_d": circle_d,
				"start_deg": start_deg,
				"hole": Value::Object(hole_measures),
			});
			Ok(Outcome { measures: Some(measures), ..bind_solid(op_id, "bolt_circle", solid)? })
		}
		OpKind::BearingSeat { input, at, axis, bearing, segments } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid =
				holes::bearing_seat(s, dv3(at), dv3(axis), &bearing, segments).map_err(|e| map_hole_error(op_id, "bearing_seat", e))?;
			let spec = holes::bearing_spec(&bearing).expect("the seat cut succeeded, so the designation is in the table");
			let measures = json!({
				"bearing": spec.designation,
				"bore_d": spec.bore,
				"outer_d": spec.outer,
				"width": spec.width,
				"pocket_d": spec.outer,
				"pocket_depth": spec.width,
				"shoulder_d": (spec.bore + spec.outer) * 0.5,
			});
			Ok(Outcome { measures: Some(measures), ..bind_solid(op_id, "bearing_seat", solid)? })
		}

		// --- Modelled ISO threads -------------------------------------------------------------------
		OpKind::ThreadSpec { m } => {
			// Measures-only table lookup (the `pipe_thread_g` pattern): the ISO 261
			// coarse pitch plus the ISO 68-1 derived dimensions a designer needs.
			let pitch = iso_pitch(op_id, "thread_spec", m)?;
			let h = 3.0_f64.sqrt() * 0.5 * pitch; // ISO 68-1 fundamental triangle height
			Ok(Outcome::measures(json!({
				"m": m,
				"pitch": pitch,
				"h": h,
				// basic minor Ø: crests − 2 × (5/8)H, the kernel ridge's root-flat Ø
				"minor_d": m - 1.25 * h,
				// the standard tap-drill rule Ø = m − pitch
				"tap_drill_d": m - pitch,
			})))
		}
		OpKind::ThreadRidge { m, major_d, pitch, z0, length } => {
			// The exact ISO 68-1 ridge solid, bound to the environment (it validates —
			// closed, manifold, genus 0). Its exact union with a shank SELF-INTERSECTS
			// by design (the root is buried P/4 into the shank): fuse via
			// `export_threaded`, never the exact `union` op.
			let (d, p) = match (m, major_d, pitch) {
				(Some(m), None, None) => (m, iso_pitch(op_id, "thread_ridge", m)?),
				(None, Some(d), Some(p)) => (d, p),
				_ => {
					return Err(err(
						ErrorKind::InvalidParam,
						format!("op '{op_id}': thread_ridge: give either 'm' (ISO coarse, {FASTENER_SIZES}) or BOTH 'major_d' and 'pitch' — not a mixture"),
					));
				}
			};
			thread_turns_guard(op_id, "thread_ridge", length, p)?;
			if !z0.is_finite() {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': thread_ridge: z0 must be finite")));
			}
			let solid = parts::iso_thread_solid(d, p, z0, length).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!(
						"op '{op_id}': thread_ridge: degenerate thread — major_d ({d}), pitch ({p}) and length ({length}) must be positive finite with the buried root radius still positive (the pitch is too large for the diameter)"
					),
				)
			})?;
			let h = 3.0_f64.sqrt() * 0.5 * p;
			let measures = json!({
				"major_d": d,
				"pitch": p,
				"minor_d": d - 1.25 * h,
				"z0": z0,
				"length": length,
				"turns": length / p,
			});
			Ok(Outcome { measures: Some(measures), ..bind_solid(op_id, "thread_ridge", solid)? })
		}
		OpKind::ExportThreaded { input, m, z0, length, internal, voxel, file } => {
			// Thread a bound body through the VOXEL half — the proven hybrid route,
			// because the exact union(body, ridge) self-intersects and no planar
			// arrangement can stitch it. External: merge the tessellation soups and
			// heal via the winding-number SDF (route "voxel_healed"). Internal: voxel-
			// subtract an oversized male ridge from the bore wall (route
			// "voxel_implicit") — a print-practical approximation of a female thread,
			// NOT the ISO D1/D4 form (documented in API.md). The thread axis is world
			// +Z through the origin: place the body's shank/bore there first.
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let pitch = iso_pitch(op_id, "export_threaded", m)?;
			thread_turns_guard(op_id, "export_threaded", length, pitch)?;
			if !z0.is_finite() {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': z0 must be finite")));
			}
			let voxel = voxel.unwrap_or(pitch / 8.0);
			if !(voxel.is_finite() && voxel > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': voxel must be a positive voxel size in mm")));
			}
			// VOXEL GUARD (deterministic): a lattice coarser than pitch/6 cannot
			// resolve the ISO profile — it smears the crests into a smooth band and
			// the "thread" silently becomes decoration. Refused, never degraded.
			if voxel > pitch / 6.0 {
				return Err(err(
					ErrorKind::InvalidParam,
					format!(
						"op '{op_id}': voxel {voxel} mm is coarser than pitch/6 ({:.4} mm for the M{m} pitch {pitch}) — the grid would smear the thread crests; use voxel ≤ pitch/6 (the default is pitch/8)",
						pitch / 6.0
					),
				));
			}
			let ridge_d = if internal { m + 2.0 * INTERNAL_CREST_CLEARANCE } else { m };
			let ridge = parts::iso_thread_solid(ridge_d, pitch, z0, length).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': export_threaded: the M{m} ridge is degenerate over z0 {z0}, length {length} — see thread_ridge"),
				)
			})?;
			// Raw exact tessellations: the winding-number SDF consumes soups directly.
			let body_mesh = kernel_brep::tessellate_adaptive_tol(s, 0.01);
			let ridge_mesh = kernel_brep::tessellate_adaptive_tol(&ridge, 0.01);
			// Deterministic misplacement refusal: a thread that does not even overlap
			// the body's bounding box cannot fuse with (or cut) it — a floating ridge
			// would still pass a naive volume check as its own shell, so this is
			// caught HERE, not left to the delta guard.
			if !body_mesh.aabb().intersection(ridge_mesh.aabb()).is_valid() {
				return Err(err(
					ErrorKind::InvalidParam,
					format!(
						"op '{op_id}': the M{m} thread span (z {z0}..{}, crest Ø{ridge_d}) does not overlap the body's bounding box — the thread axis is world +Z through the origin; pose/translate the body onto it first",
						z0 + length
					),
				));
			}
			let domain = body_mesh.aabb().union(ridge_mesh.aabb()).pad(2.0 * voxel as f32);
			grid_guard(op_id, "export_threaded", domain, voxel)?;
			// The body alone, healed at the SAME voxel, is the volume baseline — the
			// in-tree regression's guard (voxel noise cancels between the two heals).
			let baseline = watertight_mesh_of(&body_mesh, voxel as f32).signed_volume();
			let (mesh, route) = if internal {
				(mesh_boolean_implicit(&body_mesh, &ridge_mesh, BoolOp::Difference, voxel), "voxel_implicit")
			} else {
				let mut soup = body_mesh.clone();
				merge_soup(&mut soup, &ridge_mesh);
				(watertight_mesh_of(&soup, voxel as f32), "voxel_healed")
			};
			let report = check_mesh(&mesh);
			if mesh.triangle_count() == 0 || !mesh.is_watertight() || report.non_manifold_edges > 0 {
				return Err(err(
					ErrorKind::InvalidGeometry,
					format!(
						"op '{op_id}': the threaded result did not mesh watertight at voxel {voxel} (triangles={}, watertight={}, non_manifold_edges={}) — refine the voxel or check the body placement on the +Z axis",
						mesh.triangle_count(),
						mesh.is_watertight(),
						report.non_manifold_edges
					),
				));
			}
			let volume = mesh.signed_volume();
			let delta = volume - baseline;
			// The regression guard, asserted per direction: an external thread MUST add
			// material, an internal one MUST remove it — a zero delta means the ridge
			// missed the body (wrong axis placement, bore too wide, shank too thin).
			if !internal && delta <= 0.0 {
				return Err(err(
					ErrorKind::InvalidGeometry,
					format!(
						"op '{op_id}': the external thread added no material (volume delta {delta:.3} mm³) — the ridge does not overlap the body; the shank must sit on the +Z axis through the origin and reach the ridge's buried base Ø{:.3}",
						ridge_d - 1.25 * (3.0_f64.sqrt() * 0.5 * pitch) - 0.5 * pitch
					),
				));
			}
			if internal && delta >= 0.0 {
				return Err(err(
					ErrorKind::InvalidGeometry,
					format!(
						"op '{op_id}': the internal thread removed no material (volume delta {delta:.3} mm³) — the ridge (crests Ø{ridge_d}) does not reach the bore wall; the bore must sit on the +Z axis through the origin with Ø below {ridge_d}"
					),
				));
			}
			let path = write_mesh_healed(op_id, out_dir, &file, &mesh)?;
			Ok(Outcome {
				value: Some(EnvValue::Mesh(mesh.clone())),
				measures: Some(json!({
					"route": route,
					"m": m,
					"pitch": pitch,
					"internal": internal,
					"voxel": voxel,
					"triangles": mesh.triangle_count(),
					"watertight": true,
					"volume": volume,
					"volume_delta_vs_body": delta,
				})),
				file: Some(path),
			})
		}
	}
}
