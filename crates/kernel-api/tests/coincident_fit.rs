//! V4 pre-check tool (CAD Code): `coincident_fit` flags the near-coincident-face
//! hazard class FAST (a cheap parameter scan, never a boolean across the coincident
//! pair), so an agent can measure the fit numerically instead of grinding a union for
//! minutes. Refuses nothing — the hard hang backstop is the server request timeout (V6).

use kernel_api::{run_program, Report};
use serde_json::json;
use std::path::Path;
use std::time::Instant;

fn fit_flag(r: &Report, id: &str) -> Option<bool> {
	r.ops.iter().find(|o| o.id == id).and_then(|o| o.measures.as_ref()).and_then(|m| m.get("coincident_fit")).and_then(|v| v.as_bool())
}
fn run(dir: &Path, ops: serde_json::Value) -> Report {
	run_program(&serde_json::to_string(&json!({ "ops": ops })).unwrap(), dir)
}

#[test]
fn coincident_fit_flags_press_fit_fast_and_clears_separated() {
	let dir = std::env::temp_dir().join(format!("cadcode_coinfit_{}", std::process::id()));
	std::fs::create_dir_all(&dir).unwrap();

	// A Ø2 pin vs a housing with a Ø1.95 bore (coaxial, radius diff 0.025 mm) — the
	// exact press-fit class the audit flagged as a 53-minute boolean hang.
	let t0 = Instant::now();
	let r = run(
		&dir,
		json!([
			{"id":"pin","op":"cylinder","base":[0,0,-1],"axis":[0,0,1],"radius":1.0,"height":12},
			{"id":"block","op":"box","min":[-5,-5,0],"max":[5,5,10]},
			{"id":"bore","op":"cylinder","base":[0,0,-1],"axis":[0,0,1],"radius":0.975,"height":12},
			{"id":"housing","op":"difference","a":"block","b":"bore"},
			{"id":"fit","op":"coincident_fit","a":"pin","b":"housing"}
		]),
	);
	let elapsed = t0.elapsed();
	assert_eq!(fit_flag(&r, "fit"), Some(true), "a Ø2 pin vs a Ø1.95 bore must flag coincident_fit:true — {r:#?}");
	// The whole point of V4: the pre-check is a scan, NOT a boolean across the coincident
	// pair, so it returns in milliseconds instead of the ~53 minutes the union would take.
	assert!(elapsed.as_secs() < 10, "the coincident_fit pre-check must be fast — took {elapsed:?}");

	// Two well-separated boxes share no faces → not the hazard class.
	let r = run(
		&dir,
		json!([
			{"id":"a","op":"box","min":[0,0,0],"max":[10,10,10]},
			{"id":"b","op":"box","min":[100,100,100],"max":[110,110,110]},
			{"id":"fit","op":"coincident_fit","a":"a","b":"b"}
		]),
	);
	assert_eq!(fit_flag(&r, "fit"), Some(false), "separated solids must flag coincident_fit:false — {r:#?}");

	let _ = std::fs::remove_dir_all(&dir);
}
