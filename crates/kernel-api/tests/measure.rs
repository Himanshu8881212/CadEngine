//! M5 measure/DFM ops (CAD Code): `support_report` (FDM support-necessity) and `clearance`
//! (non-asserting distance/interference). Both wire existing kernel functions and refuse
//! nothing — they MEASURE, so an agent can verify manufacturability and fit through the API.

use kernel_api::{run_program, Report};
use serde_json::json;
use std::path::Path;

fn num(r: &Report, id: &str, key: &str) -> Option<f64> {
	r.ops.iter().find(|o| o.id == id).and_then(|o| o.measures.as_ref()).and_then(|m| m.get(key)).and_then(|v| v.as_f64())
}
fn flag(r: &Report, id: &str, key: &str) -> Option<bool> {
	r.ops.iter().find(|o| o.id == id).and_then(|o| o.measures.as_ref()).and_then(|m| m.get(key)).and_then(|v| v.as_bool())
}
fn run(dir: &Path, ops: serde_json::Value) -> Report {
	run_program(&serde_json::to_string(&json!({ "ops": ops })).unwrap(), dir)
}

#[test]
fn support_report_clears_a_flat_box_and_flags_an_overhang() {
	let dir = std::env::temp_dir().join(format!("cadcode_support_{}", std::process::id()));
	std::fs::create_dir_all(&dir).unwrap();

	// A flat box: bottom on the bed, top flat, sides vertical → prints support-free.
	let r = run(&dir, json!([
		{"id":"box","op":"box","min":[0,0,0],"max":[10,10,10]},
		{"id":"sr","op":"support_report","in":"box"}
	]));
	assert_eq!(flag(&r, "sr", "support_free"), Some(true), "a flat box must be support_free:true — {r:#?}");

	// A sphere: its lower hemisphere faces down past the overhang threshold → needs support.
	let r = run(&dir, json!([
		{"id":"s","op":"sphere","center":[0,0,10],"radius":10},
		{"id":"sr","op":"support_report","in":"s"}
	]));
	assert!(num(&r, "sr", "steep_area").unwrap_or(0.0) > 0.0, "a sphere's underside must give steep_area>0 — {r:#?}");
	assert_eq!(flag(&r, "sr", "support_free"), Some(false), "a sphere must be support_free:false — {r:#?}");

	let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn clearance_measures_gap_and_overlap_without_asserting() {
	let dir = std::env::temp_dir().join(format!("cadcode_clearance_{}", std::process::id()));
	std::fs::create_dir_all(&dir).unwrap();

	// Two boxes 5 mm apart → distance ≈ 5, not interfering (and it does NOT error on the query).
	let r = run(&dir, json!([
		{"id":"a","op":"box","min":[0,0,0],"max":[10,10,10]},
		{"id":"b","op":"box","min":[15,0,0],"max":[25,10,10]},
		{"id":"cl","op":"clearance","a":"a","b":"b"}
	]));
	assert!((num(&r, "cl", "distance").unwrap_or(-1.0) - 5.0).abs() < 0.5, "5 mm gap → distance≈5 — {r:#?}");
	assert_eq!(flag(&r, "cl", "interfering"), Some(false), "a gap is not interfering — {r:#?}");

	// Two overlapping boxes → interfering, with a positive overlap volume.
	let r = run(&dir, json!([
		{"id":"a","op":"box","min":[0,0,0],"max":[10,10,10]},
		{"id":"b","op":"box","min":[5,5,5],"max":[15,15,15]},
		{"id":"cl","op":"clearance","a":"a","b":"b"}
	]));
	assert_eq!(flag(&r, "cl", "interfering"), Some(true), "overlapping solids interfere — {r:#?}");
	assert!(num(&r, "cl", "overlap_volume").unwrap_or(0.0) > 0.0, "overlap_volume must be >0 for interference — {r:#?}");

	let _ = std::fs::remove_dir_all(&dir);
}

/// `measure_dimension` (FRICTION #21): exact analytic callouts — a drilled
/// plate's thickness via parallel plane faces (exact 8), the bore Ø via the
/// cylinder tag (exact 6), a point_point diagonal, and the three loud
/// refusals (non-parallel planes with the measured angle, diameter on a
/// plane, unknown kind). One assert, full report.
#[test]
fn measure_dimension_exact_callouts_and_loud_refusals() {
	let dir = std::env::temp_dir().join(format!("kernel_api_measure_dim_{}", std::process::id()));
	std::fs::create_dir_all(&dir).expect("create out dir");
	let run = |program: serde_json::Value| run_program(&serde_json::to_string(&program).expect("serialize"), &dir);
	let good = run(serde_json::json!({"ops": [
		{"id": "plate", "op": "box", "min": [0,0,0], "max": [60,40,8]},
		{"id": "bore", "op": "drill", "in": "plate", "at": [30,20,8], "axis": [0,0,-1], "d": 6, "through": 8},
		{"id": "thick", "op": "measure_dimension", "in": "bore", "kind": "face_face", "a": [30,20,0], "b": [30,20,8]},
		{"id": "dia", "op": "measure_dimension", "in": "bore", "kind": "diameter", "near": [33,20,4]},
		{"id": "diag", "op": "measure_dimension", "in": "bore", "kind": "point_point", "a": [0,0,0], "b": [60,40,0]}
	]}));
	let m = |id: &str, key: &str| -> serde_json::Value {
		good.ops.iter().find(|o| o.id == id).and_then(|o| o.measures.as_ref()).map(|m| m[key].clone()).unwrap_or(serde_json::Value::Null)
	};
	let thick = m("thick", "value").as_f64().unwrap_or(f64::NAN);
	let dia = m("dia", "value").as_f64().unwrap_or(f64::NAN);
	let diag = m("diag", "value").as_f64().unwrap_or(f64::NAN);
	let exact = (thick - 8.0).abs() < 1e-12 && (dia - 6.0).abs() < 1e-12 && (diag - (60.0f64 * 60.0 + 40.0 * 40.0).sqrt()).abs() < 1e-12;
	let analytic = m("thick", "provenance") == serde_json::json!("analytic") && m("dia", "provenance") == serde_json::json!("analytic");

	let refusals = [
		("angle", serde_json::json!({"id": "bad", "op": "measure_dimension", "in": "plate", "kind": "face_face", "a": [30,20,0], "b": [60,20,4]}), "°"),
		("plane_dia", serde_json::json!({"id": "bad", "op": "measure_dimension", "in": "plate", "kind": "diameter", "near": [30,20,8]}), "PLANE"),
		("kind", serde_json::json!({"id": "bad", "op": "measure_dimension", "in": "plate", "kind": "girth"}), "point_point"),
	];
	let mut refusal_report = String::new();
	let mut refusals_ok = true;
	for (name, op, needle) in refusals {
		let r = run(serde_json::json!({"ops": [{"id": "plate", "op": "box", "min": [0,0,0], "max": [60,40,8]}, op]}));
		let e = r.ops.iter().find(|o| o.id == "bad").and_then(|o| o.error.as_ref());
		let ok = !r.ok
			&& e.map(|e| e.kind == kernel_api::ErrorKind::InvalidParam && e.message.contains(needle)).unwrap_or(false);
		refusals_ok &= ok;
		refusal_report += &format!("\n  {name}: loud={ok} msg={:?}", e.map(|e| &e.message));
	}
	assert!(
		good.ok && exact && analytic && refusals_ok,
		"measure_dimension must be machine-exact from analytic tags and refuse ambiguous asks loudly:\n  ok={} thick={thick} (want 8) dia={dia} (want 6) diag={diag} analytic={analytic}{refusal_report}\n  report: {good:#?}",
		good.ok
	);
	let _ = std::fs::remove_dir_all(&dir);
}
