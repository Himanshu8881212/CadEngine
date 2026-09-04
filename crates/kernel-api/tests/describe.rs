//! M3 discovery (CAD Code): the `describe` op enumerates the API from ONE authoritative source
//! (the `OpKind` enum via the compile-forced `op_tag` match), so an agent that cannot read the
//! source can discover every op — and the catalogue can never drift from what actually runs.

use kernel_api::{run_program, ErrorKind, Report, OP_COUNT, OP_NAMES, OP_PARAMS};
use serde_json::json;
use std::path::Path;

fn run(dir: &Path, ops: serde_json::Value) -> Report {
	run_program(&serde_json::to_string(&json!({ "ops": ops })).unwrap(), dir)
}
fn measures<'a>(r: &'a Report, id: &str) -> Option<&'a serde_json::Value> {
	r.ops.iter().find(|o| o.id == id).and_then(|o| o.measures.as_ref())
}

#[test]
fn describe_enumerates_the_whole_api_and_every_advertised_op_is_real() {
	let dir = std::env::temp_dir().join(format!("cadcode_describe_{}", std::process::id()));
	std::fs::create_dir_all(&dir).unwrap();

	// No filter → the full catalogue + count, exactly matching the exported source of truth.
	let r = run(&dir, json!([{"id":"d","op":"describe"}]));
	let m = measures(&r, "d").unwrap_or_else(|| panic!("describe must return measures — {r:#?}"));
	let count = m.get("count").and_then(|v| v.as_u64()).expect("count") as usize;
	let ops = m.get("ops").and_then(|v| v.as_array()).expect("ops array");
	assert_eq!(count, OP_COUNT, "describe count must equal OP_COUNT");
	assert_eq!(ops.len(), OP_NAMES.len(), "describe list must be the whole catalogue");
	assert_eq!(count, ops.len(), "count must equal the enumerated list length");

	// Anti-drift, the load-bearing pin: EVERY advertised op must actually parse — never a phantom.
	// (Most fail on missing params, which is fine; what must never happen is `unknown_op`.)
	for name in OP_NAMES {
		let r = run(&dir, json!([{"id":"x","op": name}]));
		if let Some(e) = r.ops.iter().find(|o| o.id == "x").and_then(|o| o.error.as_ref()) {
			assert_ne!(e.kind, ErrorKind::UnknownOp, "describe advertises '{name}' but it is not a real op — {e:?}");
		}
	}

	// Filtered form → the exists flag (basis of did-you-mean): a real op true, a bogus one false.
	let r = run(&dir, json!([{"id":"y","op":"describe","name":"fillet_edge_near"}]));
	assert_eq!(
		measures(&r, "y").and_then(|m| m.get("exists")).and_then(|v| v.as_bool()),
		Some(true),
		"a real op must report exists:true — {r:#?}"
	);
	let r = run(&dir, json!([{"id":"z","op":"describe","name":"filet_edge"}]));
	assert_eq!(
		measures(&r, "z").and_then(|m| m.get("exists")).and_then(|v| v.as_bool()),
		Some(false),
		"a bogus op must report exists:false — {r:#?}"
	);

	let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn op_params_covers_every_op_and_describe_serves_the_specs() {
	// Drift pin for the generated per-op parameter table: OP_PARAMS must stay PARALLEL to
	// OP_NAMES — one entry per op, same tag, same declaration order. Both tables come out of
	// `tools/gen_discover.py`; this pin catches a hand edit or a partial regeneration.
	let mismatches: Vec<String> = OP_NAMES
		.iter()
		.enumerate()
		.filter_map(|(i, name)| match OP_PARAMS.get(i) {
			Some((tag, _)) if tag == name => None,
			Some((tag, _)) => Some(format!("index {i}: OP_NAMES '{name}' vs OP_PARAMS '{tag}'")),
			None => Some(format!("index {i}: OP_NAMES '{name}' has no OP_PARAMS entry")),
		})
		.collect();
	assert!(
		OP_PARAMS.len() == OP_NAMES.len() && mismatches.is_empty(),
		"OP_PARAMS must have an entry for EVERY OP_NAMES entry in the same order (regenerate with \
		 tools/gen_discover.py): len {} vs {}; mismatches: {mismatches:?}",
		OP_PARAMS.len(),
		OP_NAMES.len()
	);

	let dir = std::env::temp_dir().join(format!("cadcode_describe_params_{}", std::process::id()));
	std::fs::create_dir_all(&dir).unwrap();

	// Per-op form serves the specs over the wire: box must list required min/max as [x,y,z].
	let r = run(&dir, json!([{"id":"p","op":"describe","name":"box"}]));
	let params = measures(&r, "p").and_then(|m| m.get("params")).and_then(|v| v.as_array()).cloned().unwrap_or_default();
	let spec =
		|n: &str| params.iter().find(|p| p.get("name").and_then(|v| v.as_str()) == Some(n)).cloned().unwrap_or(serde_json::Value::Null);
	let ok = ["min", "max"].iter().all(|n| {
		let s = spec(n);
		s.get("type").and_then(|v| v.as_str()) == Some("[x,y,z]")
			&& s.get("required").and_then(|v| v.as_bool()) == Some(true)
			&& s.get("doc").is_some()
	});
	assert!(ok, "describe {{name:\"box\"}} must return param specs with required [x,y,z] min/max — got {params:?}");

	// No-arg describe stays names+count (no giant inline dump) but advertises the capability.
	let r = run(&dir, json!([{"id":"d","op":"describe"}]));
	let m = measures(&r, "d").expect("no-arg describe measures");
	assert!(
		m.get("params_available").and_then(|v| v.as_bool()) == Some(true) && m.get("params").is_none(),
		"no-arg describe must advertise params_available:true and NOT inline the per-op dump — got {m:?}"
	);

	let _ = std::fs::remove_dir_all(&dir);
}
