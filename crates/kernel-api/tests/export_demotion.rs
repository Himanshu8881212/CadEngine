// Copyright (c) LMCAD. Licensed under the MIT License.

//! `export_stl` / `export_3mf` demotion receipts (friction l12_mini_case F3,
//! uphill_roller F2, 2026-09): when the exact route is abandoned for the voxel
//! heal, the receipt must say WHICH defect abandoned it and WHERE — a bare
//! `route: "voxel_healed"` cost campaigns a day of bisection each. An exact
//! export carries no `demotion` field at all.

use std::path::PathBuf;

use kernel_api::{run_program, Report};
use serde_json::{json, Value};

fn out_dir(name: &str) -> PathBuf {
	let dir = std::env::temp_dir().join(format!("kernel_api_{name}_{}", std::process::id()));
	std::fs::create_dir_all(&dir).expect("create test out dir");
	dir
}

fn measures(report: &Report, id: &str) -> Value {
	report
		.ops
		.iter()
		.find(|o| o.id == id)
		.unwrap_or_else(|| panic!("no report entry for op '{id}' in {report:#?}"))
		.measures
		.clone()
		.unwrap_or_else(|| panic!("op '{id}' has no measures in {report:#?}"))
}

/// A body whose exact tessellation is NOT manufacturing-ready although
/// `mesh_components` calls it clean (watertight, 0 non-orientable edges) —
/// the F3 shape of the problem: a Ø8.5 boss (48 segments) with a concentric
/// blind Ø2.4 pin pocket (24 segments). Deterministic (the same construction
/// demoted in the l12_mini_case campaign).
fn demoting_body_ops() -> Vec<Value> {
	vec![
		json!({"id": "boss", "op": "cylinder", "base": [0,0,0], "axis": [0,0,1], "radius": 4.25, "height": 6, "segments": 48}),
		json!({"id": "pin", "op": "cylinder", "base": [0,0,2], "axis": [0,0,1], "radius": 1.2, "height": 5, "segments": 24}),
		json!({"id": "cut", "op": "difference", "a": "boss", "b": "pin"}),
	]
}

const REASONS: [&str; 7] = [
	"boundary_edges",
	"non_manifold_edges",
	"non_orientable_edges",
	"non_manifold_vertices",
	"degenerate_triangles",
	"self_intersection",
	"tessellation_failed",
];

/// The demotion object names a real defect of the abandoned exact
/// tessellation, its counts agree with the reason, and its witnesses sit
/// inside the body's bounds.
fn check_demotion(m: &Value, bounds: (f64, f64, f64)) -> String {
	assert_eq!(m["route"], json!("voxel_healed"), "the body must demote: {m}");
	let d = &m["demotion"];
	assert!(d.is_object(), "a healed export must carry `demotion`: {m}");
	let reason = d["reason"].as_str().unwrap_or_else(|| panic!("demotion.reason must be a string: {d}"));
	assert!(REASONS.contains(&reason), "unknown demotion reason {reason:?}: {d}");
	let count = |k: &str| d[k].as_u64().unwrap_or_else(|| panic!("demotion.{k} must be a count: {d}"));
	// The named reason is backed by its own counter (or the crossing sweep).
	let backed = match reason {
		"self_intersection" => d["self_intersections"].as_u64().unwrap_or(0) >= 1,
		"tessellation_failed" => count("exact_triangles") == 0,
		k => count(k) >= 1,
	};
	assert!(backed, "demotion.reason {reason:?} must be backed by its counter: {d}");
	// Every defect kind is reported, so the reader sees the whole picture.
	for k in
		["boundary_edges", "non_manifold_edges", "non_orientable_edges", "non_manifold_vertices", "degenerate_triangles", "exact_triangles"]
	{
		count(k);
	}
	assert!(d["self_intersections"].is_u64() || d["self_intersections"].is_null(), "{d}");
	let witness = d["witness"].as_array().unwrap_or_else(|| panic!("demotion.witness must be an array: {d}"));
	assert!(!witness.is_empty() && witness.len() <= 8, "witness must locate the defect (1..=8 points): {d}");
	for w in witness {
		let p: Vec<f64> = w.as_array().expect("witness point").iter().map(|v| v.as_f64().expect("coordinate")).collect();
		assert_eq!(p.len(), 3, "{w}");
		assert!(
			p[0].abs() <= bounds.0 + 1e-3 && p[1].abs() <= bounds.1 + 1e-3 && p[2] >= -1e-3 && p[2] <= bounds.2 + 1e-3,
			"witness {p:?} must lie in the body's frame (|x| ≤ {}, |y| ≤ {}, 0 ≤ z ≤ {})",
			bounds.0,
			bounds.1,
			bounds.2
		);
	}
	reason.to_string()
}

/// STL and 3MF exports of the demoting body both carry the same demotion
/// receipt (same exact tessellation, same verdict), and it is locatable.
#[test]
fn a_demoted_export_says_why_and_where() {
	let dir = out_dir("demotion");
	let mut ops = demoting_body_ops();
	ops.push(json!({"id": "mc", "op": "mesh_components", "in": "cut", "tol": 0.01}));
	// A coarse heal voxel keeps the (debug-build) heal short; the demotion
	// verdict is taken on the exact tessellation and does not depend on it.
	ops.push(json!({"id": "stl", "op": "export_stl", "in": "cut", "file": "cut.stl", "voxel": 0.6}));
	ops.push(json!({"id": "mf", "op": "export_3mf", "in": "cut", "file": "cut.3mf", "voxel": 0.6}));
	let report = run_program(&serde_json::to_string(&json!({ "ops": ops })).expect("serialize"), &dir);
	assert!(report.ok, "program must run: {report:#?}");
	let bounds = (4.25, 4.25, 6.0);
	let stl = measures(&report, "stl");
	let mf = measures(&report, "mf");
	let reason = check_demotion(&stl, bounds);
	let reason_3mf = check_demotion(&mf, bounds);
	assert_eq!(reason, reason_3mf, "STL and 3MF judge the same exact tessellation");
	assert_eq!(stl["demotion"], mf["demotion"], "same exact tessellation, same receipt");
	// The F3 trap, on the record: the topology-only oracle calls it clean,
	// and the actual reason is one that oracle never checks — a collapsed
	// sliver on the pocket wall (measured: 1 degenerate triangle of 896, at
	// r = 1.2 mid-wall). Pinned so a tessellation change that alters the
	// verdict is noticed, not absorbed.
	let mc = measures(&report, "mc");
	assert_eq!(mc["watertight"], json!(true), "{mc}");
	assert_eq!(mc["non_orientable_edges"], json!(0), "{mc}");
	assert_eq!(reason, "degenerate_triangles", "{}", stl["demotion"]);
	let d = &stl["demotion"];
	assert_eq!(d["self_intersections"], Value::Null, "the crossing sweep never ran (topology demoted first): {d}");
	let w = &d["witness"][0];
	let r = (w[0].as_f64().unwrap().powi(2) + w[1].as_f64().unwrap().powi(2)).sqrt();
	assert!((r - 1.2).abs() < 0.05, "the witness sits on the Ø2.4 pocket wall: {w}");
	let _ = std::fs::remove_dir_all(&dir);
}

/// An exact export carries no `demotion` field — the receipt of a clean body
/// is unchanged.
#[test]
fn an_exact_export_has_no_demotion_field() {
	let dir = out_dir("no_demotion");
	let report = run_program(
		r#"{"ops": [
			{"id": "b", "op": "box", "min": [0,0,0], "max": [20,10,5]},
			{"id": "stl", "op": "export_stl", "in": "b", "file": "b.stl"},
			{"id": "mf", "op": "export_3mf", "in": "b", "file": "b.3mf"}
		]}"#,
		&dir,
	);
	assert!(report.ok, "{report:#?}");
	for id in ["stl", "mf"] {
		let m = measures(&report, id);
		assert_eq!(m["route"], json!("exact"), "{m}");
		assert!(m.get("demotion").is_none(), "an exact export must not carry `demotion`: {m}");
	}
	let _ = std::fs::remove_dir_all(&dir);
}
