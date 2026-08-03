//! Boundary-fuzz tests for the agent surface (CAD Code M0 / audit V1).
//!
//! An agent-supplied path must NOT escape the sandbox: absolute paths and any
//! `..` traversal are refused with `InvalidParam`, and nothing is written outside
//! the out-dir. Plain relative paths still work. This is the security boundary a
//! hosted, agent-driven backend lives or dies by.

use kernel_api::{run_program, ErrorKind, OpReport, Report};
use serde_json::json;
use std::path::Path;

fn op<'r>(r: &'r Report, id: &str) -> &'r OpReport {
	r.ops.iter().find(|o| o.id == id).unwrap_or_else(|| panic!("no op '{id}' in {r:#?}"))
}

fn run_export(dir: &Path, file: serde_json::Value) -> Report {
	let program = json!({"ops": [
		{"id": "b", "op": "box", "min": [0, 0, 0], "max": [10, 10, 10]},
		{"id": "stl", "op": "export_stl", "in": "b", "file": file}
	]});
	run_program(&serde_json::to_string(&program).expect("serialize"), dir)
}

#[test]
fn sandbox_rejects_escaping_export_paths() {
	let pid = std::process::id();
	let dir = std::env::temp_dir().join(format!("cadcode_boundary_{pid}"));
	std::fs::create_dir_all(&dir).unwrap();
	let outside = dir.parent().unwrap(); // the sandbox is `dir`; its parent is off-limits

	// 1. Absolute path -> refused with InvalidParam, and the file is NOT created.
	let escape_abs = outside.join(format!("cadcode_ESCAPED_{pid}.stl"));
	let _ = std::fs::remove_file(&escape_abs);
	let r = run_export(&dir, json!(escape_abs.to_str().unwrap()));
	let stl = op(&r, "stl");
	assert!(
		!r.ok
			&& !stl.ok
			&& stl.error.as_ref().map(|e| e.kind) == Some(ErrorKind::InvalidParam)
			&& !escape_abs.exists(),
		"absolute export path must be refused (InvalidParam) and NOT written — escaped_exists={} report={r:#?}",
		escape_abs.exists()
	);

	// 2. `..` traversal -> refused, and the file is NOT created.
	let escape_rel = outside.join(format!("cadcode_DOTDOT_{pid}.stl"));
	let _ = std::fs::remove_file(&escape_rel);
	let r = run_export(&dir, json!(format!("../cadcode_DOTDOT_{pid}.stl")));
	let stl = op(&r, "stl");
	assert!(
		!r.ok
			&& stl.error.as_ref().map(|e| e.kind) == Some(ErrorKind::InvalidParam)
			&& !escape_rel.exists(),
		"'..' export path must be refused and NOT written — report={r:#?}"
	);

	// 3. A plain relative path still works (regression — confinement must not block legit use).
	let r = run_export(&dir, json!("out/part.stl"));
	assert!(
		r.ok && op(&r, "stl").ok && dir.join("out/part.stl").exists(),
		"a plain relative path must still export under the sandbox — report={r:#?}"
	);

	let _ = std::fs::remove_dir_all(&dir);
	let _ = std::fs::remove_file(&escape_abs);
	let _ = std::fs::remove_file(&escape_rel);
}

#[test]
fn caps_reject_allocation_bombs_before_alloc() {
	let dir = std::env::temp_dir().join(format!("cadcode_caps_{}", std::process::id()));
	std::fs::create_dir_all(&dir).unwrap();
	let run = |ops: serde_json::Value| run_program(&serde_json::to_string(&json!({ "ops": ops })).unwrap(), &dir);
	let refused = |r: &Report, id: &str| op(r, id).error.as_ref().map(|e| e.kind) == Some(ErrorKind::InvalidParam);

	// Over-cap cylinder facet count → refused (before it can allocate ~gigabytes).
	let r = run(json!([{"id": "c", "op": "cylinder", "base": [0, 0, 0], "axis": [0, 0, 1], "radius": 5, "height": 10, "segments": 20000}]));
	assert!(!op(&r, "c").ok && refused(&r, "c"), "over-cap segments must be refused with InvalidParam — {r:#?}");

	// A normal facet count still builds (the cap must not block legitimate work).
	let r = run(json!([{"id": "c", "op": "cylinder", "base": [0, 0, 0], "axis": [0, 0, 1], "radius": 5, "height": 10, "segments": 64}]));
	assert!(op(&r, "c").ok, "a normal cylinder (64 segments) must still build — {r:#?}");

	// Over-cap voxel grid (10000³ = 1e12 cells) → refused before allocation.
	let r = run(json!([
		{"id": "b", "op": "box", "min": [0, 0, 0], "max": [10, 10, 10]},
		{"id": "g", "op": "sample_density_grid", "in": "b", "origin": [0, 0, 0], "voxel": 1.0, "shape": [10000, 10000, 10000], "file": "g.npy"}
	]));
	assert!(!op(&r, "g").ok && refused(&r, "g"), "over-cap grid shape must be refused with InvalidParam — {r:#?}");

	let _ = std::fs::remove_dir_all(&dir);
}
