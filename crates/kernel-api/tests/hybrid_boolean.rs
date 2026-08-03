// Copyright (c) LMCAD. Licensed under the MIT License.

//! The `hybrid_boolean` op — BAR.md Level-9 "true convergence" **on the op
//! surface**: an exact B-rep operand booleaned against an implicit tree or a
//! mesh file through pure JSON, no Rust. Pins the three contracts an AI caller
//! leans on: (1) the exact route keeps untouched faces verbatim and the
//! per-face receipts partition the input faces exactly; (2) a dense organic
//! operand (TPMS) routes honestly through the voxel heal and SAYS so; (3) the
//! operand-shape errors are loud and structured (`invalid_param`), never a
//! degraded body.

use std::path::{Path, PathBuf};

use kernel_api::{run_program, ErrorKind, OpReport, Report};
use serde_json::{json, Value};

/// A unique per-test output directory under the system temp dir.
fn out_dir(name: &str) -> PathBuf {
	let dir = std::env::temp_dir().join(format!("kernel_api_hybrid_{name}_{}", std::process::id()));
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

fn run(program: &Value, dir: &Path) -> Report {
	run_program(&serde_json::to_string(program).expect("serialize"), dir)
}

fn m_u64(r: &OpReport, key: &str) -> u64 {
	r.measures.as_ref().and_then(|m| m[key].as_u64()).unwrap_or(u64::MAX)
}

/// (1) EXACT ROUTE: a planar box field cuts a square pocket into a cylinder's
/// top cap. The cylindrical wall is never touched, so it must survive VERBATIM
/// with its curved analytic tag (`kept_exact_curved ≥ 1`), the receipts must
/// partition the input faces exactly, and the result volume must sit on the
/// analytic value (cylinder − pocket) to tessellation accuracy.
#[test]
fn exact_stitch_keeps_untouched_curved_faces_verbatim_with_partition_receipts() {
	let dir = out_dir("exact");
	let program = json!({"ops": [
		{"id": "puck", "op": "cylinder", "base": [0,0,0], "axis": [0,0,1], "radius": 12, "height": 10},
		{"id": "pocketed", "op": "hybrid_boolean", "in": "puck", "bool": "difference",
		 "field": {"shape": "box", "min": [-5,-5,6], "max": [5,5,14]},
		 "voxel": 0.3, "out": "pocketed.stl"}
	]});
	let report = run(&program, &dir);
	let e = entry(&report, "pocketed");
	let measures = e.measures.as_ref().cloned().unwrap_or(json!({}));
	let route = measures["route"].as_str().unwrap_or("<missing>").to_string();
	let (faces, kept, curved, retiled, trimmed, consumed) = (
		m_u64(e, "brep_faces"),
		m_u64(e, "kept_exact"),
		m_u64(e, "kept_exact_curved"),
		m_u64(e, "retiled"),
		m_u64(e, "trimmed"),
		m_u64(e, "consumed"),
	);
	let volume = measures["volume"].as_f64().unwrap_or(f64::NAN);
	// Analytic: π·12²·10 − 10·10·(10−6). The tessellated cylinder is chord-
	// inscribed, so allow 2% (the pocket walls are exact planes).
	let analytic = std::f64::consts::PI * 144.0 * 10.0 - 400.0;
	let vol_ok = ((volume - analytic) / analytic).abs() < 0.02;
	let partition_ok = faces == kept + retiled + trimmed + consumed;
	assert!(
		report.ok
			&& route == "exact_stitch"
			&& curved >= 1
			&& trimmed >= 1
			&& partition_ok
			&& measures["watertight"] == json!(true)
			&& vol_ok
			&& dir.join("pocketed.stl").exists(),
		"hybrid_boolean exact route must keep the untouched cylinder wall verbatim and account for every input face:\n  ok={} route={route} brep_faces={faces} kept_exact={kept} kept_exact_curved={curved} retiled={retiled} trimmed={trimmed} consumed={consumed} (partition_ok={partition_ok})\n  volume={volume:.3} vs analytic {analytic:.3} (ok={vol_ok})\n  report: {report:#?}",
		report.ok
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (2) HONEST HEAL: a box-clipped gyroid sheet is a dense organic operand the
/// exact planar arrangement won't stitch — the op must fall back to the voxel
/// twin and SAY so: route `voxel_healed`, a non-empty `healed_reason`, and
/// `kept_exact == 0` (nothing survives resampling — claiming otherwise would
/// be the exact lie this receipt exists to prevent). The result is still
/// verified watertight.
#[test]
fn tpms_operand_routes_through_the_heal_and_says_so() {
	let dir = out_dir("healed");
	let program = json!({"ops": [
		{"id": "plate", "op": "box", "min": [-15,-15,0], "max": [15,15,8]},
		{"id": "fused", "op": "hybrid_boolean", "in": "plate", "bool": "union",
		 "field": {"op": "intersection",
			"a": {"shape": "tpms", "kind": "gyroid", "mode": "sheet", "level": 0.8, "cell": 6,
			      "min": [-10,-10,8], "max": [10,10,24]},
			"b": {"shape": "box", "min": [-10,-10,8], "max": [10,10,24]}},
		 "voxel": 0.5, "out": "fused.stl"}
	]});
	let report = run(&program, &dir);
	let e = entry(&report, "fused");
	let measures = e.measures.as_ref().cloned().unwrap_or(json!({}));
	let route = measures["route"].as_str().unwrap_or("<missing>").to_string();
	let reason = measures["healed_reason"].as_str().unwrap_or("").to_string();
	assert!(
		report.ok
			&& route == "voxel_healed"
			&& !reason.is_empty()
			&& m_u64(e, "kept_exact") == 0
			&& measures["watertight"] == json!(true)
			&& m_u64(e, "triangles") > 0,
		"a TPMS operand must route through the voxel heal LOUDLY (route=voxel_healed, reason stated, kept_exact=0):\n  ok={} route={route} reason={reason:?} kept_exact={} watertight={}\n  report: {report:#?}",
		report.ok,
		m_u64(e, "kept_exact"),
		measures["watertight"]
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (3) MESH-FILE OPERAND + STRUCTURED ERRORS: a mesh file exported in the same
/// program is a valid operand (planar-vs-planar stitches exact); an unbounded
/// field, a missing operand, and a double operand each fail `invalid_param`
/// loudly — never a written file.
#[test]
fn mesh_file_operand_works_and_operand_shape_errors_are_loud() {
	let dir = out_dir("operands");
	let good = json!({"ops": [
		{"id": "tool", "op": "box", "min": [5,-20,-2], "max": [30,20,12]},
		{"id": "toolstl", "op": "export_stl", "in": "tool", "file": "tool.stl"},
		{"id": "stock", "op": "box", "min": [-20,-15,0], "max": [20,15,10]},
		{"id": "cut", "op": "hybrid_boolean", "in": "stock", "bool": "difference",
		 "file": "tool.stl", "voxel": 0.4, "out": "cut.stl"}
	]});
	let report = run(&good, &dir);
	let e = entry(&report, "cut");
	let measures = e.measures.as_ref().cloned().unwrap_or(json!({}));
	let good_ok = report.ok
		&& measures["route"] == json!("exact_stitch")
		&& measures["operand"] == json!("mesh_file")
		&& measures["watertight"] == json!(true);

	let cases: [(&str, Value); 3] = [
		("unbounded", json!({"field": {"shape": "plane", "point": [0,0,0], "normal": [0,0,1]}})),
		("missing", json!({})),
		("double", json!({"field": {"shape": "box", "min": [0,0,0], "max": [1,1,1]}, "file": "tool.stl"})),
	];
	let mut errs = String::new();
	let mut errs_ok = true;
	for (name, extra) in cases {
		let mut op = json!({"id": "bad", "op": "hybrid_boolean", "in": "stock", "bool": "union", "out": "bad.stl"});
		for (k, v) in extra.as_object().unwrap() {
			op[k] = v.clone();
		}
		let prog = json!({"ops": [
			{"id": "stock", "op": "box", "min": [-20,-15,0], "max": [20,15,10]},
			op
		]});
		let r = run(&prog, &dir);
		let bad = entry(&r, "bad");
		let kind_ok = bad.error.as_ref().map(|e| e.kind == ErrorKind::InvalidParam).unwrap_or(false);
		errs_ok &= !r.ok && kind_ok;
		errs += &format!("\n  {name}: ok={} error_kind={:?}", r.ok, bad.error.as_ref().map(|e| e.kind));
	}
	assert!(
		good_ok && errs_ok,
		"mesh-file operand must stitch exact, and operand-shape misuse must fail invalid_param:\n  good: ok={} route={} operand={} watertight={}{errs}\n  good report: {report:#?}",
		report.ok,
		measures["route"],
		measures["operand"],
		measures["watertight"]
	);
	let _ = std::fs::remove_dir_all(&dir);
}
