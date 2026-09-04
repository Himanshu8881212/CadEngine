//! M2 trust (CAD Code): the `validate` op exposes `geometric_ok` (no self-intersection),
//! and measurements carry provenance. The honesty-critical direction is that a LEGITIMATE
//! solid is never false-flagged — a clean box and a normally-drilled plate must both report
//! `geometric_ok: true`. (Detection of a real self-intersection is covered by kernel-brep's
//! own `has_self_intersection` tests; here we guard against the false-positive that would make
//! the flag untrustworthy.)

use kernel_api::{run_program, Report};
use serde_json::json;
use std::path::Path;

fn bool_measure(r: &Report, id: &str, key: &str) -> Option<bool> {
	r.ops.iter().find(|o| o.id == id).and_then(|o| o.measures.as_ref()).and_then(|m| m.get(key)).and_then(|v| v.as_bool())
}
fn str_measure(r: &Report, id: &str, key: &str) -> Option<String> {
	r.ops
		.iter()
		.find(|o| o.id == id)
		.and_then(|o| o.measures.as_ref())
		.and_then(|m| m.get(key))
		.and_then(|v| v.as_str())
		.map(str::to_string)
}
fn run(dir: &Path, ops: serde_json::Value) -> Report {
	run_program(&serde_json::to_string(&json!({ "ops": ops })).unwrap(), dir)
}

#[test]
fn validate_exposes_geometric_ok_and_measures_carry_provenance() {
	let dir = std::env::temp_dir().join(format!("cadcode_trust_{}", std::process::id()));
	std::fs::create_dir_all(&dir).unwrap();

	// A clean primitive: geometric_ok:true, and the tessellated volume is stamped faceted.
	let r = run(
		&dir,
		json!([
			{"id":"b","op":"box","min":[0,0,0],"max":[10,10,10]},
			{"id":"v","op":"validate","in":"b"},
			{"id":"vol","op":"volume","in":"b"},
			{"id":"xvol","op":"exact_volume","in":"b"}
		]),
	);
	assert_eq!(bool_measure(&r, "v", "geometric_ok"), Some(true), "a clean box must report geometric_ok:true — {r:#?}");
	assert_eq!(str_measure(&r, "vol", "provenance").as_deref(), Some("faceted"), "volume must carry provenance:faceted — {r:#?}");
	assert_eq!(str_measure(&r, "xvol", "provenance").as_deref(), Some("analytic"), "exact_volume must carry provenance:analytic — {r:#?}");

	// A legitimate boolean (a drilled plate) heals cleanly and must NOT be false-flagged.
	let r = run(
		&dir,
		json!([
			{"id":"block","op":"box","min":[-5,-5,0],"max":[5,5,10]},
			{"id":"bore","op":"cylinder","base":[0,0,-1],"axis":[0,0,1],"radius":2,"height":12},
			{"id":"plate","op":"difference","a":"block","b":"bore"},
			{"id":"v","op":"validate","in":"plate"}
		]),
	);
	assert_eq!(
		bool_measure(&r, "v", "geometric_ok"),
		Some(true),
		"a normally-drilled plate is a legit boolean and must report geometric_ok:true (no false-flag) — {r:#?}"
	);

	let _ = std::fs::remove_dir_all(&dir);
}
