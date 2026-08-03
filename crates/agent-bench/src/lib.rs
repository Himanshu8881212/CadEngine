//! agent-bench — the RULER for CAD Code's agent surface (plan milestone M0).
//!
//! Red-first eval harness. Every criterion drives ONLY the kernel-api JSON surface
//! (`kernel_api::run_program`) — never a direct kernel geometry call — so it scores
//! what an AI agent can actually discover, trust, inspect, and do through the API,
//! not what the Rust library can do internally. Criteria flip green as milestones
//! M0..M6 land; under CI the score can only ratchet up. The kernel dimension is
//! pinned at 9.0 (frozen: bit-deterministic, analytic, exact).
//!
//! This is the SEED set (a partial slice of the ~40-criterion target). Its value is
//! the framework + the visible red criteria; the absolute number firms up as coverage
//! grows. It is honest, not calibrated to flatter — it reports what the surface does.

use kernel_api::{run_program, ErrorKind, OpReport, Report};
use serde_json::{json, Value};
use std::path::{Path, PathBuf};
use std::time::Instant;

/// The seven axes the agent surface is scored on (kernel excluded — it is frozen at 9).
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Dim {
	Safety,
	Trust,
	Discovery,
	Loop,
	Measure,
	Performance,
	Contract,
}
pub const DIMS: [Dim; 7] =
	[Dim::Safety, Dim::Trust, Dim::Discovery, Dim::Loop, Dim::Measure, Dim::Performance, Dim::Contract];
impl Dim {
	pub fn name(self) -> &'static str {
		match self {
			Dim::Safety => "safety",
			Dim::Trust => "trust",
			Dim::Discovery => "discovery",
			Dim::Loop => "loop",
			Dim::Measure => "measure",
			Dim::Performance => "performance",
			Dim::Contract => "contract",
		}
	}
}

/// One machine-checkable criterion evaluated against the JSON surface.
pub struct Criterion {
	pub dim: Dim,
	pub id: &'static str,
	pub desc: &'static str,
	pub passed: bool,
}
fn crit(dim: Dim, id: &'static str, desc: &'static str, passed: bool) -> Criterion {
	Criterion { dim, id, desc, passed }
}

fn bench_dir() -> PathBuf {
	let d = std::env::temp_dir().join(format!("agentbench_{}", std::process::id()));
	std::fs::create_dir_all(&d).ok();
	d
}
fn run(dir: &Path, ops: Value) -> Report {
	run_program(&serde_json::to_string(&json!({ "ops": ops })).unwrap(), dir)
}
fn get<'r>(r: &'r Report, id: &str) -> Option<&'r OpReport> {
	r.ops.iter().find(|o| o.id == id)
}
fn refused_invalid(r: &Report, id: &str) -> bool {
	get(r, id)
		.map(|o| !o.ok && o.error.as_ref().map(|e| e.kind) == Some(ErrorKind::InvalidParam))
		.unwrap_or(false)
}
fn op_ok(r: &Report, id: &str) -> bool {
	get(r, id).map(|o| o.ok).unwrap_or(false)
}
fn has_measure(r: &Report, id: &str, key: &str) -> bool {
	get(r, id).and_then(|o| o.measures.as_ref()).and_then(|m| m.get(key)).is_some()
}
fn num(r: &Report, id: &str, key: &str) -> Option<f64> {
	get(r, id).and_then(|o| o.measures.as_ref()).and_then(|m| m.get(key)).and_then(|v| v.as_f64())
}
fn flag(r: &Report, id: &str, key: &str) -> Option<bool> {
	get(r, id).and_then(|o| o.measures.as_ref()).and_then(|m| m.get(key)).and_then(|v| v.as_bool())
}
fn text_eq(r: &Report, id: &str, key: &str, val: &str) -> bool {
	get(r, id).and_then(|o| o.measures.as_ref()).and_then(|m| m.get(key)).and_then(|v| v.as_str()) == Some(val)
}
fn err_is(r: &Report, id: &str, kind: ErrorKind) -> bool {
	get(r, id).map(|o| !o.ok && o.error.as_ref().map(|e| e.kind) == Some(kind)).unwrap_or(false)
}
/// Wall-clock a program (release build); returns (report, elapsed_ms).
fn timed(dir: &Path, ops: Value) -> (Report, u128) {
	let t = Instant::now();
	let r = run(dir, ops);
	(r, t.elapsed().as_millis())
}

/// Evaluate every seed criterion. Pure of side effects the caller cares about
/// (uses a per-process temp sandbox, cleaned up on exit).
pub fn run_all() -> Vec<Criterion> {
	let dir = bench_dir();
	let cube = || json!({"id":"b","op":"box","min":[0,0,0],"max":[10,10,10]});
	let mut c: Vec<Criterion> = Vec::new();

	// ---- SAFETY (M1) ----
	let r = run(&dir, json!([cube(), {"id":"e","op":"export_stl","in":"b","file":"/tmp/agentbench_ESCAPE.stl"}]));
	c.push(crit(Dim::Safety, "path_confine_absolute", "absolute export path refused (InvalidParam)", refused_invalid(&r, "e")));
	let r = run(&dir, json!([cube(), {"id":"e","op":"export_stl","in":"b","file":"../agentbench_ESCAPE.stl"}]));
	c.push(crit(Dim::Safety, "path_confine_dotdot", "'..' traversal export path refused", refused_invalid(&r, "e")));
	// A clearly-excessive-but-safe segment count: must be rejected BEFORE allocating (V3, not yet).
	let r = run(&dir, json!([{"id":"c","op":"cylinder","base":[0,0,0],"axis":[0,0,1],"radius":5,"height":10,"segments":20000}]));
	c.push(crit(Dim::Safety, "alloc_cap_segments", "excessive cylinder segments refused before alloc", refused_invalid(&r, "c")));
	// Coincident-fit hazard pre-check tool (V4): a Ø2 pin vs a Ø1.95 bore must flag true — FAST.
	let r = run(&dir, json!([
		{"id":"pin","op":"cylinder","base":[0,0,-1],"axis":[0,0,1],"radius":1.0,"height":12},
		{"id":"block","op":"box","min":[-5,-5,0],"max":[5,5,10]},
		{"id":"bore","op":"cylinder","base":[0,0,-1],"axis":[0,0,1],"radius":0.975,"height":12},
		{"id":"housing","op":"difference","a":"block","b":"bore"},
		{"id":"fit","op":"coincident_fit","a":"pin","b":"housing"}
	]));
	let flagged = get(&r, "fit").and_then(|o| o.measures.as_ref()).and_then(|m| m.get("coincident_fit")).and_then(Value::as_bool).unwrap_or(false);
	c.push(crit(Dim::Safety, "coincident_fit_precheck", "coincident_fit tool flags the boolean-hang hazard class", flagged));

	// ---- TRUST (M2) ----
	let r = run(&dir, json!([cube(), {"id":"v","op":"validate","in":"b"}]));
	c.push(crit(Dim::Trust, "geometric_ok_exposed", "validate exposes a geometric_ok flag", has_measure(&r, "v", "geometric_ok")));
	let r = run(&dir, json!([cube(), {"id":"vol","op":"volume","in":"b"}]));
	c.push(crit(Dim::Trust, "provenance_exposed", "a measurement carries exact|analytic|faceted provenance", has_measure(&r, "vol", "provenance")));

	// ---- DISCOVERY (M3) ----
	let r = run(&dir, json!([{"id":"d","op":"describe"}]));
	c.push(crit(Dim::Discovery, "describe_op", "a describe op enumerates the API from one source", op_ok(&r, "d")));

	// ---- AGENTIC LOOP (M4) ----
	let r = run(&dir, json!([cube(), {"id":"lf","op":"list_faces","in":"b"}]));
	c.push(crit(Dim::Loop, "list_faces", "list_faces returns selectable entity references", op_ok(&r, "lf")));

	// ---- MEASURE / DFM (M5) ----
	let r = run(&dir, json!([cube(), {"id":"sr","op":"support_report","in":"b"}]));
	c.push(crit(Dim::Measure, "support_report", "support/overhang report is callable", op_ok(&r, "sr")));
	let r = run(&dir, json!([cube(), {"id":"cl","op":"clearance","a":"b","b":"b"}]));
	c.push(crit(Dim::Measure, "clearance_op", "non-asserting clearance/distance is callable", op_ok(&r, "cl")));

	// ---- CONTRACT (M0) ----
	let r1 = run(&dir, json!([cube(), {"id":"vol","op":"volume","in":"b"}]));
	let r2 = run(&dir, json!([cube(), {"id":"vol","op":"volume","in":"b"}]));
	let deterministic = serde_json::to_string(&r1).ok() == serde_json::to_string(&r2).ok();
	c.push(crit(Dim::Contract, "determinism_same_bytes", "same program twice → identical report bytes", deterministic));
	let versioned = serde_json::to_string(&r1).map(|s| s.contains("api_version")).unwrap_or(false);
	c.push(crit(Dim::Contract, "api_version", "the response envelope carries an api_version", versioned));

	// ============ GROWN COVERAGE (cycle 10) — per-dimension depth, red-first ============
	// -- safety: input-path confine, a second cap vector, degenerate refusal --
	let r = run(&dir, json!([{"id":"lp","op":"load_part","file":"../escape.lmcpart"}]));
	c.push(crit(Dim::Safety, "path_confine_input", "'..' in an input-file path refused", refused_invalid(&r, "lp")));
	let r = run(&dir, json!([{"id":"rv","op":"revolve","profile":[[1,0],[2,0],[2,1],[1,1]],"angle":360,"segments":40000}]));
	c.push(crit(Dim::Safety, "alloc_cap_revolve", "excessive revolve segments refused before alloc", refused_invalid(&r, "rv")));
	let r = run(&dir, json!([{"id":"db","op":"box","min":[10,10,10],"max":[0,0,0]}]));
	c.push(crit(Dim::Safety, "degenerate_solid_refused", "an inverted/empty box is refused, not silently bound", !op_ok(&r, "db")));

	// -- trust: no false-flag, specific provenance values, mass_props provenance (RED gap) --
	let r = run(&dir, json!([cube(), {"id":"v","op":"validate","in":"b"}]));
	c.push(crit(Dim::Trust, "geometric_ok_true_clean", "a clean solid reads geometric_ok:true (no false-flag)", flag(&r, "v", "geometric_ok") == Some(true)));
	let r = run(&dir, json!([cube(), {"id":"vol","op":"volume","in":"b"}, {"id":"xv","op":"exact_volume","in":"b"}]));
	c.push(crit(Dim::Trust, "provenance_faceted", "volume is stamped provenance:faceted", text_eq(&r, "vol", "provenance", "faceted")));
	c.push(crit(Dim::Trust, "provenance_analytic", "exact_volume is stamped provenance:analytic", text_eq(&r, "xv", "provenance", "analytic")));
	let r = run(&dir, json!([cube(), {"id":"mp","op":"mass_properties","in":"b"}]));
	c.push(crit(Dim::Trust, "provenance_on_mass_props", "mass_properties carries provenance too (RED gap)", has_measure(&r, "mp", "provenance")));

	// -- discovery: full enumeration, exists-filter, per-op params (RED gap) --
	let r = run(&dir, json!([{"id":"d","op":"describe"}]));
	let enumerated = num(&r, "d", "count").map(|n| n > 100.0).unwrap_or(false)
		&& get(&r, "d").and_then(|o| o.measures.as_ref()).and_then(|m| m.get("ops")).and_then(|v| v.as_array()).map(|a| !a.is_empty()).unwrap_or(false);
	c.push(crit(Dim::Discovery, "describe_enumerates", "describe returns the full op list + count", enumerated));
	// describe_params tests the REAL per-op design (criterion updated 2026-07-09 when the feature
	// landed — the placeholder written before the design existed probed the NO-ARG describe for a
	// params/schema key; the shipped design keeps the no-arg reply small and serves specs per op):
	// no-arg describe advertises params_available, and `describe {name:"box"}` must return a
	// params array whose required `min`/`max` entries carry a type.
	let advertised = flag(&r, "d", "params_available") == Some(true);
	let rb = run(&dir, json!([{"id":"p","op":"describe","name":"box"}]));
	let specs = get(&rb, "p").and_then(|o| o.measures.as_ref()).and_then(|m| m.get("params")).and_then(|v| v.as_array()).cloned();
	let has_req = |n: &str| {
		specs
			.as_ref()
			.map(|a| {
				a.iter().any(|p| {
					p.get("name").and_then(Value::as_str) == Some(n)
						&& p.get("required").and_then(Value::as_bool) == Some(true)
						&& p.get("type").and_then(Value::as_str).is_some()
				})
			})
			.unwrap_or(false)
	};
	c.push(crit(
		Dim::Discovery,
		"describe_params",
		"describe {name} returns the op's param specs (box: required min/max) + no-arg advertises params_available",
		advertised && has_req("min") && has_req("max"),
	));
	let real = run(&dir, json!([{"id":"y","op":"describe","name":"fillet_edge_near"}]));
	let bogus = run(&dir, json!([{"id":"z","op":"describe","name":"filet_edge"}]));
	c.push(crit(Dim::Discovery, "describe_exists_filter", "describe reports exists for real vs bogus op (did-you-mean)",
		flag(&real, "y", "exists") == Some(true) && flag(&bogus, "z", "exists") == Some(false)));

	// -- loop (M4, largely unbuilt): stateful session + edge refs read RED honestly --
	let r = run(&dir, json!([{"id":"se","op":"open_session","doc":"x"}]));
	c.push(crit(Dim::Loop, "session_stateful", "stateful session (server-layer; kernel-api stays pure — coverage gap, not a run_program op)", op_ok(&r, "se")));
	let r = run(&dir, json!([cube(), {"id":"le","op":"list_edges","in":"b"}]));
	c.push(crit(Dim::Loop, "entity_refs", "edge/entity refs op exists (RED: M4)", op_ok(&r, "le")));

	// -- measure: overhang case, interference case, full inertia, bbox dims --
	let r = run(&dir, json!([{"id":"s","op":"sphere","center":[0,0,10],"radius":10}, {"id":"sr","op":"support_report","in":"s"}]));
	c.push(crit(Dim::Measure, "support_flags_overhang", "a sphere underside reads steep_area>0", num(&r, "sr", "steep_area").map(|a| a > 0.0).unwrap_or(false)));
	let r = run(&dir, json!([
		{"id":"a","op":"box","min":[0,0,0],"max":[10,10,10]}, {"id":"bb","op":"box","min":[5,5,5],"max":[15,15,15]},
		{"id":"cl","op":"clearance","a":"a","b":"bb"}
	]));
	c.push(crit(Dim::Measure, "clearance_detects_overlap", "overlap -> interfering + overlap_volume>0",
		flag(&r, "cl", "interfering") == Some(true) && num(&r, "cl", "overlap_volume").map(|v| v > 0.0).unwrap_or(false)));
	let r = run(&dir, json!([cube(), {"id":"mp","op":"mass_properties","in":"b"}]));
	c.push(crit(Dim::Measure, "mass_properties_full", "mass_properties returns com + inertia",
		has_measure(&r, "mp", "center_of_mass") && has_measure(&r, "mp", "inertia_diag")));
	let r = run(&dir, json!([cube(), {"id":"bx","op":"bounding_box","in":"b"}]));
	c.push(crit(Dim::Measure, "bounding_box_dims", "bounding_box returns size/min/max", has_measure(&r, "bx", "size")));

	// -- performance (was 0): throughput, fast-fail, large legal build, stable re-run --
	let mut many: Vec<Value> = Vec::with_capacity(1000);
	for i in 0..1000 {
		many.push(json!({"id": format!("b{i}"), "op":"box", "min":[0,0,0], "max":[1,1,1]}));
	}
	let (rp, ms) = timed(&dir, json!(many));
	c.push(crit(Dim::Performance, "perf_1000_ops", "a 1000-op program completes in a few seconds", rp.ok && ms < 5000));
	let (rcb, mscb) = timed(&dir, json!([{"id":"c","op":"cylinder","base":[0,0,0],"axis":[0,0,1],"radius":5,"height":10,"segments":20000}]));
	c.push(crit(Dim::Performance, "perf_countbomb_fast_fail", "an over-cap count-bomb refused in <100ms (pre-alloc)", refused_invalid(&rcb, "c") && mscb < 100));
	let (rlb, mslb) = timed(&dir, json!([{"id":"c","op":"cylinder","base":[0,0,0],"axis":[0,0,1],"radius":20,"height":40,"segments":2000}, {"id":"vol","op":"volume","in":"c"}]));
	c.push(crit(Dim::Performance, "perf_large_legal_build", "a 2000-seg cylinder build completes <5s", op_ok(&rlb, "vol") && mslb < 5000));
	let (rs, mss) = timed(&dir, json!([cube(), {"id":"vol","op":"volume","in":"b"}]));
	let stable = serde_json::to_string(&r1).ok() == serde_json::to_string(&rs).ok();
	c.push(crit(Dim::Performance, "perf_repeat_stable", "a re-run is deterministic and fast (<500ms)", stable && mss < 500));

	// -- contract: bare-ops parses, matchable unknown-op, stop-on-first-failure, id echo --
	c.push(crit(Dim::Contract, "bare_ops_parses", "a bare ops program parses and runs", op_ok(&r1, "vol")));
	let r = run(&dir, json!([{"id":"u","op":"no_such_op_xyz"}]));
	c.push(crit(Dim::Contract, "unknown_op_matchable", "an unknown op yields ErrorKind::UnknownOp", err_is(&r, "u", ErrorKind::UnknownOp)));
	let r = run(&dir, json!([{"id":"bad","op":"volume","in":"nope"}, {"id":"after","op":"box","min":[0,0,0],"max":[1,1,1]}]));
	c.push(crit(Dim::Contract, "stops_on_first_failure", "a program halts at the first failing op", !op_ok(&r, "bad") && get(&r, "after").is_none()));
	let r = run(&dir, json!([cube(), {"id":"vol","op":"volume","in":"b"}]));
	c.push(crit(Dim::Contract, "stable_ids_echoed", "each op's id is echoed in its report", get(&r, "b").is_some() && get(&r, "vol").is_some()));

	let _ = std::fs::remove_dir_all(&dir);
	c
}

/// The rolled-up score. `agent_surface = 10·passed/total`; kernel is frozen at 9.0;
/// `composite = 0.35·kernel + 0.65·agent_surface` (reproduces the audit's 5.5 at 9/4,
/// reaches 9.0 at agent 9).
pub struct Scorecard {
	pub kernel: f64,
	pub agent_surface: f64,
	pub composite: f64,
	pub passed: usize,
	pub total: usize,
	pub per_dim: Vec<(&'static str, usize, usize)>,
}
pub fn score(criteria: &[Criterion]) -> Scorecard {
	let total = criteria.len();
	let passed = criteria.iter().filter(|c| c.passed).count();
	let agent_surface = if total > 0 { 10.0 * passed as f64 / total as f64 } else { 0.0 };
	let kernel = 9.0;
	let composite = 0.35 * kernel + 0.65 * agent_surface;
	let per_dim = DIMS
		.iter()
		.map(|&d| {
			let n = criteria.iter().filter(|c| c.dim == d).count();
			let p = criteria.iter().filter(|c| c.dim == d && c.passed).count();
			(d.name(), p, n)
		})
		.collect();
	Scorecard { kernel, agent_surface, composite, passed, total, per_dim }
}

/// The committed, versioned scorecard artifact (diffed like a BAR.md re-grade row).
pub fn scorecard_json(criteria: &[Criterion], s: &Scorecard) -> Value {
	json!({
		"schema": "cadcode.agent-bench.v0",
		"note": "GROWN ruler (36 criteria, cycle 10 — toward the ~40 target). Every dimension has depth. Red-first. Honest, not calibrated to flatter — the number DROPPED (8.57->~8.4) when coverage grew, because a thin ruler over-reads. describe_params flipped GREEN 2026-07-09 when per-op ParamSpec discovery landed (criterion updated from the pre-design no-arg placeholder to test the real per-op form). Remaining red: session_stateful only — a server-layer concern by design (kernel-api stays a pure run_program), kept as an honest coverage marker, not a bug.",
		"kernel": s.kernel,
		"agent_surface": s.agent_surface,
		"composite": s.composite,
		"passed": s.passed,
		"total": s.total,
		"per_dim": s.per_dim.iter().map(|(n, p, t)| json!({"dim": n, "passed": p, "total": t})).collect::<Vec<_>>(),
		"criteria": criteria.iter().map(|c| json!({"dim": c.dim.name(), "id": c.id, "desc": c.desc, "passed": c.passed})).collect::<Vec<_>>(),
	})
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn ruler_runs_scores_and_pins_cycle1() {
		let criteria = run_all();
		assert!(!criteria.is_empty(), "the ruler must produce criteria");
		let s = score(&criteria);
		assert!(
			(0.0..=10.0).contains(&s.agent_surface) && (0.0..=10.0).contains(&s.composite),
			"scores must be in range: agent={} composite={}",
			s.agent_surface,
			s.composite
		);
		// Regression guard tying the ruler to cycle 1: the path-confinement fix must stay green.
		let confine: Vec<_> = criteria.iter().filter(|c| c.id.starts_with("path_confine")).collect();
		assert!(
			confine.len() >= 2 && confine.iter().all(|c| c.passed),
			"path confinement criteria (export absolute/dotdot + input) must all be green (cycle 1 / M1-V1): {:?}",
			confine.iter().map(|c| (c.id, c.passed)).collect::<Vec<_>>()
		);
	}
}
