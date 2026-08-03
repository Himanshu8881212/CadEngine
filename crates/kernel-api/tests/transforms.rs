// Copyright (c) LMCAD. Licensed under the MIT License.

//! Tier-1 op wave 1: the axis rotations (`rotate_x`/`rotate_y`), the
//! orientation-safe `mirror`, the clone-union `linear_pattern`/`polar_pattern`,
//! and the voxel-route `shell` hollow — all through `run_program`, no direct
//! Rust geometry calls.
//!
//! Load-bearing background finding (verified live, and pinned in
//! `kernel-brep/src/booleans.rs::union_of_disjoint_boxes_keeps_both_volumes`):
//! the exact boolean union of DISJOINT operands is a valid multi-shell solid
//! (`validate().shells == n`, closed, manifold, volume = sum), so the pattern
//! ops fold clones with plain `union` and report shell counts honestly.

use std::path::{Path, PathBuf};

use kernel_api::{run_program, ErrorKind, OpReport, Report};
use serde_json::json;

/// A unique per-test output directory under the system temp dir.
fn out_dir(name: &str) -> PathBuf {
	let dir = std::env::temp_dir().join(format!("kernel_api_{name}_{}", std::process::id()));
	std::fs::create_dir_all(&dir).expect("create test out dir");
	dir
}

fn run(dir: &Path, ops: serde_json::Value) -> Report {
	run_program(&serde_json::to_string(&json!({ "ops": ops })).unwrap(), dir)
}

/// The report entry for op `id` (panics with the report when absent).
fn entry<'r>(report: &'r Report, id: &str) -> &'r OpReport {
	report
		.ops
		.iter()
		.find(|o| o.id == id)
		.unwrap_or_else(|| panic!("no report entry for op '{id}' in {report:#?}"))
}

/// A named f64 measure of op `id` (NaN when absent, so the assert message shows it).
fn measure(report: &Report, id: &str, key: &str) -> f64 {
	entry(report, id).measures.as_ref().and_then(|m| m[key].as_f64()).unwrap_or(f64::NAN)
}

/// `rotate_x` / `rotate_y` are true siblings of `rotate_z`: each must land the
/// solid EXACTLY where the general `pose` about the same world axis lands it.
/// Witness: full mass properties (volume + center of mass) of an asymmetric box.
#[test]
fn rotate_x_and_rotate_y_match_the_equivalent_pose() {
	let dir = out_dir("rotxy");
	let r = run(
		&dir,
		json!([
			{"id": "b", "op": "box", "min": [0.0, 0.0, 0.0], "max": [10.0, 4.0, 3.0]},
			{"id": "rx",  "op": "rotate_x", "in": "b", "degrees": 30.0},
			{"id": "px",  "op": "pose",     "in": "b", "rotate": {"axis": [1, 0, 0], "degrees": 30.0}},
			{"id": "ry",  "op": "rotate_y", "in": "b", "degrees": -47.5},
			{"id": "py",  "op": "pose",     "in": "b", "rotate": {"axis": [0, 1, 0], "degrees": -47.5}},
			{"id": "mrx", "op": "mass_properties", "in": "rx"},
			{"id": "mpx", "op": "mass_properties", "in": "px"},
			{"id": "mry", "op": "mass_properties", "in": "ry"},
			{"id": "mpy", "op": "mass_properties", "in": "py"},
		]),
	);
	let com = |id: &str| -> Vec<f64> {
		entry(&r, id)
			.measures
			.as_ref()
			.and_then(|m| m["center_of_mass"].as_array().map(|a| a.iter().filter_map(|v| v.as_f64()).collect()))
			.unwrap_or_default()
	};
	let close = |a: &[f64], b: &[f64]| a.len() == 3 && b.len() == 3 && a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-9);
	assert!(
		r.ok
			&& close(&com("mrx"), &com("mpx"))
			&& close(&com("mry"), &com("mpy"))
			&& (measure(&r, "mrx", "volume") - 120.0).abs() < 1e-9
			&& (measure(&r, "mry", "volume") - 120.0).abs() < 1e-9,
		"rotate_x/rotate_y must compose to the same pose as `pose` about the same axis: ok={} com(rx)={:?} com(px)={:?} com(ry)={:?} com(py)={:?} report={r:#?}",
		r.ok,
		com("mrx"),
		com("mpx"),
		com("mry"),
		com("mpy")
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// `mirror` reflects a MARKED asymmetric solid (an L: 10×4×3 bar + a 2×4×3 nub
/// on top of its x∈[0,2] end) across the x = 0 plane. The reflection must be a
/// valid, orientation-correct solid (closed, manifold, geometric_ok — never
/// inside-out), keep the exact volume, and flip the witness geometry: the whole
/// part moves to x ∈ [−10, 0] with the nub's height mark preserved at z = 6.
#[test]
fn mirror_reflects_a_marked_asymmetric_solid_orientation_correct() {
	let dir = out_dir("mirror");
	let r = run(
		&dir,
		json!([
			{"id": "bar", "op": "box", "min": [0.0, 0.0, 0.0], "max": [10.0, 4.0, 3.0]},
			{"id": "nub", "op": "box", "min": [0.0, 0.0, 3.0], "max": [2.0, 4.0, 6.0]},
			{"id": "l",   "op": "union", "a": "bar", "b": "nub"},
			{"id": "m",   "op": "mirror", "in": "l", "plane": {"point": [0, 0, 0], "normal": [1, 0, 0]}},
			{"id": "v",   "op": "validate", "in": "m"},
			{"id": "vol", "op": "volume", "in": "m"},
			{"id": "bb",  "op": "bounding_box", "in": "m"},
		]),
	);
	let v = entry(&r, "v").measures.as_ref().cloned().unwrap_or_default();
	let bb = entry(&r, "bb").measures.as_ref().cloned().unwrap_or_default();
	let volume = measure(&r, "vol", "volume");
	assert!(
		r.ok
			&& v["valid"] == json!(true)
			&& v["geometric_ok"] == json!(true)
			&& (volume - 144.0).abs() < 1e-9
			&& bb["min"] == json!([-10.0, 0.0, 0.0])
			&& bb["max"] == json!([0.0, 4.0, 6.0]),
		"mirror must be a valid orientation-correct reflection (volume 144 kept, bbox flipped to [-10,0]×[0,4]×[0,6]): ok={} validate={v} volume={volume} bbox={bb} report={r:#?}",
		r.ok
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// `linear_pattern` of a 10³ box, count 4, step [20,0,0]: four DISJOINT clones
/// folded by the exact union into one valid 4-shell solid of exactly 4× the
/// volume — asserted by the binding's own `assert` op (shells + volume window),
/// so the intent lives in the program.
#[test]
fn linear_pattern_of_a_box_is_four_disjoint_shells_at_4x_volume() {
	let dir = out_dir("linpat");
	let r = run(
		&dir,
		json!([
			{"id": "b",  "op": "box", "min": [0.0, 0.0, 0.0], "max": [10.0, 10.0, 10.0]},
			{"id": "lp", "op": "linear_pattern", "in": "b", "count": 4, "step": [20.0, 0.0, 0.0]},
			{"id": "ok", "op": "assert", "in": "lp",
				"valid": true, "shells": 4, "genus": 0,
				"volume_within": {"target": 4000.0, "abs": 1e-6}},
		]),
	);
	assert!(r.ok, "linear_pattern count=4 must union to a valid 4-shell solid of volume 4×1000: report={r:#?}");
	let _ = std::fs::remove_dir_all(&dir);
}

/// `polar_pattern` of an off-axis arm about Z, count 6 (default step 360/6):
/// six disjoint clones → one valid 6-shell solid at 6× volume. The volume
/// window is ±0.01 mm³ absolute: rotated clones carry only f64 rotation noise
/// (measured ~6e-5 mm³ on 720), never a re-tessellation error.
#[test]
fn polar_pattern_of_six_about_z_is_six_shells_at_6x_volume() {
	let dir = out_dir("polpat");
	let r = run(
		&dir,
		json!([
			{"id": "arm", "op": "box", "min": [10.0, -2.0, 0.0], "max": [20.0, 2.0, 3.0]},
			{"id": "pp",  "op": "polar_pattern", "in": "arm", "count": 6, "center": [0, 0, 0], "axis": [0, 0, 1]},
			{"id": "ok",  "op": "assert", "in": "pp",
				"valid": true, "shells": 6, "genus": 0,
				"volume_within": {"target": 720.0, "abs": 0.01}},
		]),
	);
	assert!(r.ok, "polar_pattern count=6 must union to a valid 6-shell solid of volume 6×120: report={r:#?}");
	let _ = std::fs::remove_dir_all(&dir);
}

/// `shell` hollows a 30 mm cube to a 2 mm wall: watertight mesh, route honestly
/// reported as `voxel_healed` (it is one by construction), volume ≈ outer −
/// inner = 30³ − 26³ = 9424 mm³. Tolerance: ±1% — the voxel route at the 0.3 mm
/// default measured 9424.35 (0.004% off, Manifold DC preserves the sharp cube
/// features), so 1% is a comfortably honest ceiling, not a tuned one.
#[test]
fn shell_hollows_a_cube_to_the_outer_minus_inner_wall_volume() {
	let dir = out_dir("shellcube");
	let r = run(
		&dir,
		json!([
			{"id": "cube",   "op": "box", "min": [0.0, 0.0, 0.0], "max": [30.0, 30.0, 30.0]},
			{"id": "hollow", "op": "shell", "in": "cube", "wall": 2.0, "file": "hollow.stl"},
		]),
	);
	let m = entry(&r, "hollow").measures.as_ref().cloned().unwrap_or_default();
	let volume = measure(&r, "hollow", "volume");
	let expected = 30.0_f64.powi(3) - 26.0_f64.powi(3); // 9424: outer minus inner cavity
	let stl = dir.join("hollow.stl");
	let file_ok = std::fs::metadata(&stl).map(|md| md.len() > 0).unwrap_or(false);
	assert!(
		r.ok
			&& m["watertight"] == json!(true)
			&& m["route"] == json!("voxel_healed")
			&& (volume - expected).abs() <= 0.01 * expected
			&& file_ok,
		"shell of a 30 cube at wall 2 must be a watertight voxel_healed wall of {expected}±1% mm³: ok={} measures={m} volume={volume} file_ok={file_ok} report={r:#?}",
		r.ok
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// The wave-1 guards, each a structured `invalid_param` (never a hang, an OOM,
/// or a mystery meshing failure): pattern count over the 500 cap (rejected
/// structurally BEFORE dispatch), a no-op 1-pattern, a zero pattern step
/// (coincident clones), a polar step that is a multiple of 360°, a zero mirror
/// normal, and a shell wall the voxel grid cannot resolve.
#[test]
fn wave1_guards_reject_hostile_or_degenerate_params() {
	let dir = out_dir("wave1guards");
	let cases: Vec<(&str, serde_json::Value)> = vec![
		("count over cap", json!({"id": "x", "op": "linear_pattern", "in": "b", "count": 501, "step": [20.0, 0.0, 0.0]})),
		("1-pattern no-op", json!({"id": "x", "op": "linear_pattern", "in": "b", "count": 1, "step": [20.0, 0.0, 0.0]})),
		("zero step", json!({"id": "x", "op": "linear_pattern", "in": "b", "count": 3, "step": [0.0, 0.0, 0.0]})),
		("360° polar step", json!({"id": "x", "op": "polar_pattern", "in": "b", "count": 3, "center": [0, 0, 0], "axis": [0, 0, 1], "step_deg": 360.0})),
		("zero mirror normal", json!({"id": "x", "op": "mirror", "in": "b", "plane": {"point": [0, 0, 0], "normal": [0, 0, 0]}})),
		("unresolvable shell wall", json!({"id": "x", "op": "shell", "in": "b", "wall": 0.5, "voxel": 0.3})),
	];
	let mut failures: Vec<String> = Vec::new();
	for (what, op) in cases {
		let r = run(
			&dir,
			json!([
				{"id": "b", "op": "box", "min": [0.0, 0.0, 0.0], "max": [10.0, 10.0, 10.0]},
				op,
			]),
		);
		match &entry(&r, "x").error {
			Some(e) if e.kind == ErrorKind::InvalidParam => {}
			other => failures.push(format!("{what}: expected invalid_param, got {other:?}")),
		}
	}
	assert!(failures.is_empty(), "every wave-1 guard must fail structured: {failures:#?}");
	let _ = std::fs::remove_dir_all(&dir);
}
