//! The ≥5-part END-TO-END benchmark — CAD Code's real STOP gate (SC-J acceptance).
//!
//! Drives ONLY the JSON surface (`kernel_api::run_program`), never a direct kernel call, so it
//! proves what an AGENT can build through the wire: five distinct real parts, each cleared through
//! the full gate stack — binds without crashing, `geometric_ok:true`, provenance-stamped
//! measurements, a watertight export with an honest mesh route, and a callable DFM report — plus a
//! mating clearance check. Any gate miss PANICS the test. No silent-wrong, full provenance.

use kernel_api::{run_program, Report};
use serde_json::{json, Value};
use std::path::Path;

fn run(dir: &Path, ops: Value) -> Report {
	run_program(&serde_json::to_string(&json!({ "ops": ops })).unwrap(), dir)
}
fn flag(r: &Report, id: &str, key: &str) -> Option<bool> {
	r.ops.iter().find(|o| o.id == id).and_then(|o| o.measures.as_ref()).and_then(|m| m.get(key)).and_then(|v| v.as_bool())
}
fn has(r: &Report, id: &str, key: &str) -> bool {
	r.ops.iter().find(|o| o.id == id).and_then(|o| o.measures.as_ref()).and_then(|m| m.get(key)).is_some()
}
fn op_ok(r: &Report, id: &str) -> bool {
	r.ops.iter().find(|o| o.id == id).map(|o| o.ok).unwrap_or(false)
}

/// Run a part-building program (which must bind the finished part to id `p`), append the full gate
/// stack, and assert every gate. Panics with the whole report on any miss.
fn gate_part(dir: &Path, name: &str, mut build: Vec<Value>) {
	build.push(json!({ "id":"v",   "op":"validate",       "in":"p" }));
	build.push(json!({ "id":"vol", "op":"volume",         "in":"p" }));
	build.push(json!({ "id":"sr",  "op":"support_report", "in":"p" }));
	build.push(json!({ "id":"ex",  "op":"export_stl",     "in":"p", "file": format!("{name}.stl") }));
	let r = run(dir, json!(build));

	assert!(r.ok, "[{name}] the build+gate program must fully succeed (no crash, no failed op) — {r:#?}");
	assert_eq!(flag(&r, "v", "geometric_ok"), Some(true), "[{name}] must be geometrically sound (geometric_ok:true) — {r:#?}");
	assert!(has(&r, "vol", "provenance"), "[{name}] its volume must carry a provenance tag — {r:#?}");
	assert!(op_ok(&r, "sr"), "[{name}] a DFM support report must be callable — {r:#?}");
	assert_eq!(flag(&r, "ex", "watertight"), Some(true), "[{name}] the export must be watertight — {r:#?}");
	assert!(has(&r, "ex", "route"), "[{name}] the export must declare its mesh route (exact vs voxel_healed) — {r:#?}");
}

#[test]
fn five_part_assembly_passes_every_gate_through_the_api() {
	let dir = std::env::temp_dir().join(format!("cadcode_e2e_{}", std::process::id()));
	std::fs::create_dir_all(&dir).unwrap();

	// 1 — a drilled bracket (box − through-bore).
	gate_part(
		&dir,
		"bracket",
		vec![
			json!({"id":"blk","op":"box","min":[0,0,0],"max":[40,20,8]}),
			json!({"id":"hole","op":"cylinder","base":[20,10,-1],"axis":[0,0,1],"radius":3,"height":10}),
			json!({"id":"p","op":"difference","a":"blk","b":"hole"}),
		],
	);
	// 2 — an involute spur gear (a standard part).
	gate_part(&dir, "spur_gear", vec![json!({"id":"p","op":"spur_gear","module":2,"teeth":18,"face_width":8,"bore":6})]);
	// 3 — a shaft (primitive cylinder).
	gate_part(&dir, "shaft", vec![json!({"id":"p","op":"cylinder","base":[0,0,0],"axis":[0,0,1],"radius":4,"height":30})]);
	// 4 — a housing shell (box − pocket open at the top).
	gate_part(
		&dir,
		"housing",
		vec![
			json!({"id":"o","op":"box","min":[0,0,0],"max":[30,30,20]}),
			json!({"id":"i","op":"box","min":[2,2,2],"max":[28,28,21]}),
			json!({"id":"p","op":"difference","a":"o","b":"i"}),
		],
	);
	// 5 — a two-hole flanged plate (box − two bores).
	gate_part(
		&dir,
		"flange_plate",
		vec![
			json!({"id":"pl","op":"box","min":[0,0,0],"max":[50,20,6]}),
			json!({"id":"h1","op":"cylinder","base":[10,10,-1],"axis":[0,0,1],"radius":2.5,"height":8}),
			json!({"id":"h2","op":"cylinder","base":[40,10,-1],"axis":[0,0,1],"radius":2.5,"height":8}),
			json!({"id":"d1","op":"difference","a":"pl","b":"h1"}),
			json!({"id":"p","op":"difference","a":"d1","b":"h2"}),
		],
	);

	// Mating check — the assembly-relevant signal is INTERFERENCE, asserted both ways.
	// (Honest limitation surfaced building this: clearance's `distance` uses Mesh::min_distance,
	// which returns 0 for a solid NESTED in another's cavity even when it clears — so a nested
	// running-fit gap is not readable from `distance`; `interfering`/`overlap_volume` ARE correct.
	// Analytic nested clearance is a documented follow-up. The interference gate below is robust.)
	let bore = |shaft_r: f64| {
		json!([
			{"id":"blk","op":"box","min":[0,0,0],"max":[40,20,8]},
			{"id":"hole","op":"cylinder","base":[20,10,-1],"axis":[0,0,1],"radius":3,"height":10},
			{"id":"brk","op":"difference","a":"blk","b":"hole"},
			{"id":"shaft","op":"cylinder","base":[20,10,1],"axis":[0,0,1],"radius":shaft_r,"height":6},
			{"id":"cl","op":"clearance","a":"shaft","b":"brk"}
		])
	};
	// A Ø4 shaft FITS the Ø6 bore with clearance — no material interference.
	let fit = run(&dir, bore(2.0));
	assert!(fit.ok, "[mating] the fitting shaft program must succeed — {fit:#?}");
	assert_eq!(flag(&fit, "cl", "interfering"), Some(false), "[mating] a Ø4 shaft clears a Ø6 bore (no interference) — {fit:#?}");
	// A Ø10 shaft is OVERSIZED for the Ø6 bore — it bites into the bracket → interference.
	let press = run(&dir, bore(5.0));
	assert!(press.ok, "[mating] the oversized shaft program must succeed — {press:#?}");
	assert_eq!(flag(&press, "cl", "interfering"), Some(true), "[mating] a Ø10 shaft interferes with a Ø6 bore — {press:#?}");

	let _ = std::fs::remove_dir_all(&dir);
}
