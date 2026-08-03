// Copyright (c) LMCAD. Licensed under the MIT License.

//! BAR.md **I7 acceptance**: the curated, admission-gated parts library driven
//! ENTIRELY through JSON programs — no Rust geometry calls anywhere.
//!
//! - Program 1 **authors** a parametric part (an inline `.lmcpart` envelope —
//!   the feature tree is data) with a declared parameter interface and admits
//!   it through the gate; a **separate** program 2 instantiates it with
//!   different parameters, booleans it onto another body, validates and
//!   exports — the falsifiable I7 sentence, end to end.
//! - A breaking range corner is **rejected loudly** (`admission_rejected`,
//!   naming the corner and values) and pollutes nothing.
//! - The curation lifecycle: deprecate hides from search but instantiate still
//!   builds WITH a warning; remove refuses with the dependent `.lmcasm` list
//!   (`dependents_exist`) unless forced.
//!
//! All dates in these programs are caller-supplied literals — the library
//! never reads a clock.

use std::path::{Path, PathBuf};

use kernel_api::{run_program, ErrorKind, OpReport, Report};
use serde_json::{json, Value};

/// A unique per-test output directory under the system temp dir.
fn out_dir(name: &str) -> PathBuf {
	let dir = std::env::temp_dir().join(format!("kernel_api_lib_{name}_{}", std::process::id()));
	let _ = std::fs::remove_dir_all(&dir);
	std::fs::create_dir_all(&dir).expect("create test out dir");
	dir
}

/// The report entry for op `id` (panics with the report when absent).
fn entry<'r>(report: &'r Report, id: &str) -> &'r OpReport {
	report
		.ops
		.iter()
		.find(|o| o.id == id)
		.unwrap_or_else(|| panic!("no report entry for op '{id}' in {report:#?}"))
}

/// True when `path` exists and is non-empty.
fn file_ok(path: &Path) -> bool {
	std::fs::metadata(path).map(|m| m.len() > 0).unwrap_or(false)
}

/// The AI-authored candidate: a parametric bushing as an INLINE `.lmcpart`
/// envelope (pure data — its features reference the parameters `outer_r`,
/// `bore_r`, `h` that the declared interface ranges drive).
fn bushing_envelope() -> Value {
	json!({
		"format": "lmc-part",
		"version": 1,
		"units": "mm",
		"name": "bushing",
		"created_with": "i7 acceptance test (authored as JSON)",
		"document": {
			"params": {"outer_r": 12.0, "bore_r": 4.0, "h": 10.0},
			"features": [
				{"Cylinder": {"center": [{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": 0.0}],
				              "radius": {"Param": "outer_r"}, "height": {"Param": "h"}},
				 "label": "body"},
				{"Cylinder": {"center": [{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": 0.0}],
				              "radius": {"Param": "bore_r"}, "height": {"Literal": 200.0}},
				 "label": "bore tool (always through)"},
				{"Boolean": {"op": "Difference", "a": 0, "b": 1}}
			],
			"root": 2,
			"suppressed": []
		}
	})
}

/// The `meta` block declaring the bushing's public interface (3 parameters →
/// the gate runs defaults + all 2³ corners + midpoint = 10 samples).
fn bushing_meta() -> Value {
	json!({
		"name": "bushing",
		"version": 1,
		"category": "spacers",
		"tags": ["bearing", "sleeve"],
		"description": "parametric plain bushing with a through bore",
		"provenance": {"author": "acceptance-ai", "date": "2026-06-10"},
		"params": [
			{"name": "outer_r", "units": "mm", "default": 12.0, "min": 8.0, "max": 16.0, "description": "outer radius"},
			{"name": "bore_r",  "units": "mm", "default": 4.0,  "min": 2.0, "max": 5.0,  "description": "bore radius"},
			{"name": "h",       "units": "mm", "default": 10.0, "min": 4.0, "max": 40.0, "description": "height"}
		]
	})
}

/// The I7 falsifiable sentence: an AI-authored part admitted via pure JSON
/// (program 1) is instantiated with NEW parameters by a SEPARATE program 2,
/// booleaned onto another body, validated and exported — only the library
/// directory persists between the two programs.
#[test]
fn i7_acceptance_author_admit_then_separate_program_builds_product() {
	let dir = out_dir("acceptance");

	// --- Program 1: author + admit + prove it is findable. -----------------------
	let program1 = json!({"ops": [
		{"id": "admit", "op": "library_add", "dir": "lib", "part": bushing_envelope(), "meta": bushing_meta()},
		{"id": "find", "op": "library_search", "dir": "lib", "text": "bushing", "tags": ["bearing"]}
	]});
	let report1 = run_program(&serde_json::to_string(&program1).expect("serialize"), &dir);
	let admit = entry(&report1, "admit").measures.clone().unwrap_or_default();
	let matches = entry(&report1, "find").measures.as_ref().and_then(|m| m["matches"].as_array().cloned()).unwrap_or_default();
	// 32-gon annulus closed form (16 sin(π/16) (R² − r²) h) at the defaults.
	let k32 = 16.0 * (std::f64::consts::PI / 16.0).sin();
	let defaults_volume = k32 * (144.0 - 16.0) * 10.0;
	assert!(
		report1.ok
			&& admit["gate_samples"] == json!(10)
			&& admit["gate_rebuilds"] == json!(2)
			&& admit["file"] == json!("bushing-v1.lmcpart")
			&& (admit["volume_at_defaults"].as_f64().unwrap_or(f64::NAN) - defaults_volume).abs() < 1e-6 * defaults_volume
			&& matches.len() == 1
			&& matches[0]["params"].as_array().map(Vec::len) == Some(3)
			&& file_ok(&dir.join("lib/bushing-v1.lmcpart"))
			&& file_ok(&dir.join("lib/index.json")),
		"program 1 (author + admit, pure JSON): admit={admit} matches={matches:?} report={report1:#?}"
	);

	// --- Program 2 (SEPARATE run; only the library dir persists): instantiate
	// with DIFFERENT parameters, sink it into a plate, validate, export. ----------
	let program2 = json!({"ops": [
		{"id": "bush", "op": "library_instantiate", "dir": "lib", "name": "bushing",
		 "params": {"outer_r": 14.0, "bore_r": 3.0, "h": 20.0}},
		{"id": "plate", "op": "box", "min": [0, 0, 0], "max": [40, 30, 8]},
		{"id": "placed", "op": "translate", "in": "bush", "offset": [20, 15, 12]},
		{"id": "product", "op": "union", "a": "plate", "b": "placed"},
		{"id": "check", "op": "validate", "in": "product"},
		{"id": "vol", "op": "volume", "in": "product"},
		{"id": "stl", "op": "export_stl", "in": "product", "file": "product.stl"}
	]});
	let report2 = run_program(&serde_json::to_string(&program2).expect("serialize"), &dir);
	let bush = entry(&report2, "bush").measures.clone().unwrap_or_default();
	let check = entry(&report2, "check").measures.clone().unwrap_or_default();
	let volume = entry(&report2, "vol").measures.as_ref().and_then(|m| m["volume"].as_f64()).unwrap_or(f64::NAN);
	// Plate 9600 + annulus k32·(14² − 3²)·20, minus the overlap slice z ∈ [2, 8]
	// (k32·187·6); the bore over the plate becomes a blind pocket → genus 0.
	let expected = 9600.0 + k32 * 187.0 * (20.0 - 6.0);
	assert!(
		report2.ok
			&& bush["name"] == json!("bushing")
			&& bush["version"] == json!(1)
			&& bush["deprecated"] == json!(false)
			&& bush.get("warning").is_none()
			&& bush["params"]["h"] == json!(20.0)
			&& check["valid"] == json!(true)
			&& check["genus"] == json!(0)
			&& check["shells"] == json!(1)
			&& (volume - expected).abs() < 1e-6 * expected
			&& file_ok(&dir.join("product.stl")),
		"program 2 (separate instantiate + boolean + export): bush={bush} check={check} volume={volume} (want {expected}) report={report2:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// Gate rejection: the declared range reaches a corner where the bore swallows
/// the whole block (empty difference). Admission must fail with the
/// machine-matchable kind `admission_rejected`, NAME the corner and its
/// parameter values, and admit nothing — a later search finds an empty library.
#[test]
fn i7_gate_rejects_breaking_corner_and_pollutes_nothing() {
	let dir = out_dir("reject");
	let candidate = json!({
		"format": "lmc-part", "version": 1, "units": "mm", "name": "overdrilled",
		"created_with": "i7 acceptance test",
		"document": {
			"params": {"r": 5.0},
			"features": [
				{"Box": {"center": [{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": 0.0}],
				         "size": [{"Literal": 20.0}, {"Literal": 20.0}, {"Literal": 10.0}]}},
				{"Cylinder": {"center": [{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": 0.0}],
				              "radius": {"Param": "r"}, "height": {"Literal": 50.0}}},
				{"Boolean": {"op": "Difference", "a": 0, "b": 1}}
			],
			"root": 2,
			"suppressed": []
		}
	});
	let program = json!({"ops": [
		{"id": "admit", "op": "library_add", "dir": "lib", "part": candidate,
		 "meta": {
			"name": "overdrilled", "version": 1,
			"provenance": {"author": "acceptance-ai", "date": "2026-06-10"},
			// r = 15 > the block's half-diagonal 14.14: the high corner empties the part.
			"params": [{"name": "r", "units": "mm", "default": 5.0, "min": 2.0, "max": 15.0}]
		 }}
	]});
	let report = run_program(&serde_json::to_string(&program).expect("serialize"), &dir);
	let error = report.ops[0].error.clone().expect("the gate must reject");

	let search = run_program(
		r#"{"ops": [{"id": "find", "op": "library_search", "dir": "lib"}]}"#,
		&dir,
	);
	let matches = entry(&search, "find").measures.as_ref().and_then(|m| m["matches"].as_array().cloned()).unwrap_or_default();
	assert!(
		!report.ok
			&& error.kind == ErrorKind::AdmissionRejected
			&& error.message.contains("corner_h")
			&& error.message.contains("r=15")
			&& search.ok
			&& matches.is_empty()
			&& !dir.join("lib/overdrilled-v1.lmcpart").exists(),
		"gate rejection must be loud (naming the corner) and admit NOTHING: error={error:?} matches={matches:?} report={report:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// The curation lifecycle through JSON: deprecate hides from search while
/// instantiate still builds WITH a warning; an out-of-range value is loud;
/// remove refuses with the dependent `.lmcasm` named (`dependents_exist`);
/// force removes, after which the library is empty and the file is gone.
#[test]
fn i7_curation_lifecycle_deprecate_warn_dependents_force_remove() {
	let dir = out_dir("lifecycle");

	// Admit, deprecate, prove hidden-but-warning-instantiable in ONE program.
	let program = json!({"ops": [
		{"id": "admit", "op": "library_add", "dir": "lib", "part": bushing_envelope(), "meta": bushing_meta()},
		{"id": "retire", "op": "library_deprecate", "dir": "lib", "name": "bushing"},
		{"id": "find", "op": "library_search", "dir": "lib", "text": "bushing"},
		{"id": "legacy", "op": "library_instantiate", "dir": "lib", "name": "bushing", "params": {"h": 12.0}},
		{"id": "check", "op": "validate", "in": "legacy"}
	]});
	let report = run_program(&serde_json::to_string(&program).expect("serialize"), &dir);
	let retire = entry(&report, "retire").measures.clone().unwrap_or_default();
	let matches = entry(&report, "find").measures.as_ref().and_then(|m| m["matches"].as_array().cloned()).unwrap_or_default();
	let legacy = entry(&report, "legacy").measures.clone().unwrap_or_default();
	let valid = entry(&report, "check").measures.as_ref().map(|m| m["valid"] == json!(true));

	// An out-of-range instantiate is a loud structured error naming the range.
	let bad = run_program(
		r#"{"ops": [{"id": "too_tall", "op": "library_instantiate", "dir": "lib", "name": "bushing", "params": {"h": 41.0}}]}"#,
		&dir,
	);
	let bad_error = bad.ops[0].error.clone().expect("41 > max 40 must fail");

	// A hand-written assembly in the library dir references the part by path…
	std::fs::write(
		dir.join("lib/stack.lmcasm"),
		r#"{"format": "lmc-asm", "version": 1, "units": "mm", "name": "stack",
		"instances": [{"source": {"path": "bushing-v1.lmcpart"}, "pose": {"translation": [0.0, 0.0, 0.0]}}],
		"mates": []}"#,
	)
	.expect("write dependent assembly");

	// …so removal refuses, names it, and force overrides.
	let refused = run_program(r#"{"ops": [{"id": "rm", "op": "library_remove", "dir": "lib", "name": "bushing"}]}"#, &dir);
	let refused_error = refused.ops[0].error.clone().expect("dependent must block removal");
	let forced = run_program(
		r#"{"ops": [
			{"id": "rm", "op": "library_remove", "dir": "lib", "name": "bushing", "force": true},
			{"id": "find", "op": "library_search", "dir": "lib"}
		]}"#,
		&dir,
	);
	let removed = entry(&forced, "rm").measures.clone().unwrap_or_default();
	let after: Vec<Value> = entry(&forced, "find").measures.as_ref().and_then(|m| m["matches"].as_array().cloned()).unwrap_or_default();

	assert!(
		report.ok
			&& retire["deprecated_versions"] == json!(1)
			&& matches.is_empty()
			&& legacy["deprecated"] == json!(true)
			&& legacy["warning"].as_str().is_some_and(|w| w.contains("deprecated"))
			&& valid == Some(true)
			&& !bad.ok
			&& bad_error.kind == ErrorKind::InvalidParam
			&& bad_error.message.contains("[4, 40]")
			&& !refused.ok
			&& refused_error.kind == ErrorKind::DependentsExist
			&& refused_error.message.contains("stack.lmcasm")
			&& forced.ok
			&& removed["removed_files"] == json!(["bushing-v1.lmcpart"])
			&& after.is_empty()
			&& !dir.join("lib/bushing-v1.lmcpart").exists(),
		"curation lifecycle: retire={retire} hidden={} legacy={legacy} valid={valid:?} bad={bad_error:?} refused={refused_error:?} removed={removed} after={} report={report:#?}",
		matches.len(),
		after.len()
	);
	let _ = std::fs::remove_dir_all(&dir);
}
