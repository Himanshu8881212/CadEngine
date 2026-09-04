//! The universal `require` gate (campaign theme T10) — the assertive vocabulary
//! for every measure, and the refusals that keep it from ever passing silently.
//!
//! Each test names the defect it pins. Without the `require` engine every one of
//! these programs runs to `ok: true` with the gate simply absent, which is the
//! outcome the whole file exists to prevent.

use kernel_api::{run_program, ErrorKind, OpReport, Report};
use serde_json::{json, Value};
use std::path::{Path, PathBuf};

fn dir(tag: &str) -> PathBuf {
	let d = std::env::temp_dir().join(format!("cadcode_require_{tag}_{}", std::process::id()));
	std::fs::create_dir_all(&d).unwrap();
	d
}
fn run(d: &Path, ops: Value) -> Report {
	run_program(&serde_json::to_string(&json!({ "ops": ops })).unwrap(), d)
}
fn op<'r>(r: &'r Report, id: &str) -> &'r OpReport {
	r.ops.iter().find(|o| o.id == id).unwrap_or_else(|| panic!("no op '{id}' in {r:#?}"))
}
fn measure(r: &Report, id: &str, key: &str) -> Value {
	op(r, id).measures.as_ref().and_then(|m| m.get(key)).cloned().unwrap_or(Value::Null)
}
/// The failing op's error, asserted to be of `kind`.
fn failure(r: &Report, id: &str, kind: ErrorKind) -> String {
	let e = op(r, id).error.as_ref().unwrap_or_else(|| panic!("op '{id}' did not fail — {r:#?}"));
	assert_eq!(e.kind, kind, "op '{id}' failed with the wrong kind — {r:#?}");
	e.message.clone()
}

/// T10: the four `DELIVERABLE_SPEC` §2 gates that had NO in-program expression.
/// Before `require`, none of `route`/`watertight`, `steep_area`, `thin_area`/
/// `p05_thickness` or `fits_within` could fail a program.
#[test]
fn the_four_unexpressible_spec_gates_now_gate() {
	let d = dir("spec4");
	let r = run(
		&d,
		json!([
			{"id":"p","op":"box","min":[0,0,0],"max":[60,40,8]},
			{"id":"stl","op":"export_stl","in":"p","file":"p.stl",
			 "require":{"route":"exact","watertight":true}},
			{"id":"sup","op":"support_report","in":"p","build_dir":[0,0,1],
			 "require":{"steep_area":{"max":0.0},"support_free":true}},
			{"id":"wall","op":"wall_thickness","in":"p","flag_below":1.2,
			 "require":{"thin_area":{"max":0.0},"p05_thickness":{"min":1.2}}},
			{"id":"bed","op":"bounding_box","in":"p","envelope":[256,256,256],
			 "require":{"fits_within":true}},
			{"id":"mass","op":"mass_properties","in":"p",
			 "require":{"volume":{"within":{"target":19200.0,"percent":0.1}}}}
		]),
	);
	assert!(r.ok, "every gate must pass on a clean plate — {r:#?}");
	// The receipt records the gate that was applied, not merely that it passed.
	assert_eq!(measure(&r, "stl", "required"), json!({"route":"exact","watertight":true}));

	// …and each one FAILS when the part violates it.
	let r = run(
		&d,
		json!([
			{"id":"p","op":"box","min":[0,0,0],"max":[600,40,8]},
			{"id":"bed","op":"bounding_box","in":"p","envelope":[256,256,256],
			 "require":{"fits_within":true}}
		]),
	);
	assert!(!r.ok);
	let msg = failure(&r, "bed", ErrorKind::AssertFailed);
	assert!(msg.contains("fits_within"), "the failure must name the gate — {msg}");

	let r = run(
		&d,
		json!([
			{"id":"s","op":"sphere","center":[0,0,10],"radius":10},
			{"id":"sup","op":"support_report","in":"s","require":{"steep_area":{"max":0.0}}}
		]),
	);
	assert!(!r.ok);
	failure(&r, "sup", ErrorKind::AssertFailed);
	let _ = std::fs::remove_dir_all(&d);
}

/// A per-axis gate on an ARRAY measure, without inventing an op-specific
/// vocabulary: `null` is the documented don't-care slot.
#[test]
fn array_measures_gate_element_wise() {
	let d = dir("array");
	let r = run(
		&d,
		json!([
			{"id":"p","op":"box","min":[0,0,0],"max":[60,40,8]},
			{"id":"bb","op":"bounding_box","in":"p","require":{"size":[{"max":100.0},null,{"min":5.0}]}}
		]),
	);
	assert!(r.ok, "{r:#?}");

	let r = run(
		&d,
		json!([
			{"id":"p","op":"box","min":[0,0,0],"max":[60,40,8]},
			{"id":"bb","op":"bounding_box","in":"p","require":{"size":[{"max":10.0},null,null]}}
		]),
	);
	let msg = failure(&r, "bb", ErrorKind::AssertFailed);
	assert!(msg.contains("size.0"), "the failure must name the element — {msg}");
	let _ = std::fs::remove_dir_all(&d);
}

/// Silence is the forbidden outcome. A `require` that cannot possibly do its job
/// must REFUSE, never quietly succeed: an empty gate, a typo'd key, a numeric
/// bound on a string, a bad clause name, or a gate on an op with no measures.
#[test]
fn a_gate_that_cannot_check_anything_refuses() {
	let d = dir("refuse");
	let cases: &[(Value, &str)] = &[
		(json!({}), "is empty"),
		(json!({"watertigt": true}), "names no measure"),
		(json!({"route": {"min": 3}}), "need a numeric measure"),
		(json!({"triangles": {"nax": 3}}), "unknown clause"),
		(json!({"triangles": {}}), "expectation object is empty"),
		(json!({"triangles": {"within": {"target": 1.0}}}), "EXACTLY one"),
		(json!({"triangles": {"within": {"target": 1.0, "abs": 1.0, "percent": 1.0}}}), "EXACTLY one"),
	];
	for (spec, needle) in cases {
		let r = run(
			&d,
			json!([
				{"id":"p","op":"box","min":[0,0,0],"max":[10,10,10]},
				{"id":"x","op":"export_stl","in":"p","file":"r.stl","require":spec}
			]),
		);
		let msg = failure(&r, "x", ErrorKind::InvalidParam);
		assert!(msg.contains(needle), "expected '{needle}' in the refusal for {spec}, got: {msg}");
	}
	// `require` is not a valid gate on an op that measures nothing.
	let r = run(
		&d,
		json!([
			{"id":"p","op":"box","min":[0,0,0],"max":[10,10,10],"require":{"volume":1}}
		]),
	);
	let msg = failure(&r, "p", ErrorKind::InvalidParam);
	assert!(msg.contains("no measures"), "{msg}");
	let _ = std::fs::remove_dir_all(&d);
}

/// `require` is universal, so it must NOT be reported as an unknown param by the
/// typo tripwire — otherwise every gated op would ship a spurious warning.
#[test]
fn require_never_warns_as_an_unknown_param() {
	let d = dir("warn");
	let r = run(
		&d,
		json!([
			{"id":"p","op":"box","min":[0,0,0],"max":[10,10,10]},
			{"id":"v","op":"volume","in":"p","require":{"volume":{"min":1.0}}}
		]),
	);
	assert!(r.ok, "{r:#?}");
	assert!(op(&r, "v").warnings.is_empty(), "require must not warn — {r:#?}");
	// A genuine typo still warns.
	let r = run(
		&d,
		json!([
			{"id":"p","op":"box","min":[0,0,0],"max":[10,10,10]},
			{"id":"v","op":"volume","in":"p","reqiure":{"volume":{"min":1.0}}}
		]),
	);
	assert!(!op(&r, "v").warnings.is_empty(), "a typo'd 'reqiure' must warn — {r:#?}");
	let _ = std::fs::remove_dir_all(&d);
}

/// A program that declares no `require` must produce BYTE-IDENTICAL measures to
/// before the gate existed — the whole surface is additive.
#[test]
fn ungated_programs_are_unchanged() {
	let d = dir("compat");
	let ops = json!([
		{"id":"p","op":"box","min":[0,0,0],"max":[10,10,10]},
		{"id":"v","op":"volume","in":"p"}
	]);
	let r = run(&d, ops);
	assert_eq!(measure(&r, "v", "volume"), json!(1000.0));
	assert!(op(&r, "v").measures.as_ref().unwrap().get("required").is_none(), "no gate ⇒ no echo — {r:#?}");
	let _ = std::fs::remove_dir_all(&d);
}

/// `describe` must ADVERTISE the universal parameter — a gate documented only in
/// prose is a gate nobody discovers from the binary.
#[test]
fn describe_advertises_the_universal_gate() {
	let d = dir("describe");
	let r = run(
		&d,
		json!([
			{"id":"all","op":"describe"},
			{"id":"one","op":"describe","name":"support_report"}
		]),
	);
	assert!(r.ok, "{r:#?}");
	let universal = measure(&r, "all", "universal_params");
	assert_eq!(universal[0]["name"], json!("require"), "{r:#?}");
	let params = measure(&r, "one", "params");
	assert!(params.as_array().unwrap().iter().any(|p| p["name"] == json!("require")), "per-op describe must list require — {params}");
	let _ = std::fs::remove_dir_all(&d);
}
