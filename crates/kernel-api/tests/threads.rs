// Copyright (c) LMCAD. Licensed under the MIT License.

//! Tier-1 wave 2 THREADS: the ISO 261/68-1 lookup (`thread_spec`), the exact
//! ridge solid (`thread_ridge`), and the voxel-half thread fuse/cut
//! (`export_threaded`) — table fidelity, param exclusivity, the turns cap, the
//! crest-smear voxel guard, and the volume-delta regression guards in both
//! directions (external ADDS material, internal REMOVES it).

use std::path::{Path, PathBuf};

use kernel_api::{run_program, ErrorKind, OpReport, Report};
use serde_json::json;

/// A unique per-test output directory under the system temp dir.
fn out_dir(name: &str) -> PathBuf {
	let dir = std::env::temp_dir().join(format!("kernel_api_{name}_{}", std::process::id()));
	std::fs::create_dir_all(&dir).expect("create test out dir");
	dir
}

/// Run `ops` as a program with `dir` as both out-dir and input base.
fn run(dir: &Path, ops: serde_json::Value) -> Report {
	run_program(&serde_json::to_string(&json!({ "ops": ops })).expect("serialize"), dir)
}

/// The report entry for op `id` (panics with the report when absent).
fn entry<'r>(report: &'r Report, id: &str) -> &'r OpReport {
	report
		.ops
		.iter()
		.find(|o| o.id == id)
		.unwrap_or_else(|| panic!("no report entry for op '{id}' in {report:#?}"))
}

/// A measure of op `id` as f64 (NaN when absent — assertions then fail loudly).
fn num(report: &Report, id: &str, key: &str) -> f64 {
	entry(report, id).measures.as_ref().and_then(|m| m[key].as_f64()).unwrap_or(f64::NAN)
}

/// (1) thread_spec is the ISO table verbatim: M6 coarse is P=1.0 with
/// H=(√3/2)·P, minor Ø = 6 − 1.25·H, tap drill Ø = 6 − P; a size outside the
/// table is a structured invalid_param naming the supported sizes.
#[test]
fn thread_spec_matches_the_iso_tables() {
	let dir = out_dir("thread_spec");
	let r = run(&dir, json!([{"id": "s", "op": "thread_spec", "m": 6.0}]));
	let h = 3.0_f64.sqrt() * 0.5;
	let (pitch, hh, minor, tap) = (num(&r, "s", "pitch"), num(&r, "s", "h"), num(&r, "s", "minor_d"), num(&r, "s", "tap_drill_d"));
	assert!(
		r.ok && (pitch - 1.0).abs() < 1e-12
			&& (hh - h).abs() < 1e-12
			&& (minor - (6.0 - 1.25 * h)).abs() < 1e-12
			&& (tap - 5.0).abs() < 1e-12,
		"M6 spec: pitch={pitch} h={hh} minor_d={minor} tap_drill_d={tap} report={r:#?}"
	);

	let r = run(&dir, json!([{"id": "bad", "op": "thread_spec", "m": 7.0}]));
	let e = entry(&r, "bad").error.as_ref().expect("must fail");
	assert!(
		!r.ok && e.kind == ErrorKind::InvalidParam && e.message.contains("M3"),
		"M7 must be refused naming the supported table sizes: {r:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (2) thread_ridge binds a VALID exact solid: closed, manifold, genus 0, with
/// the crest envelope at the major Ø (bbox x/y span ≈ 6 mm — the 96-station
/// polygonization keeps it within half a station's chord) and the turns echoed.
#[test]
fn thread_ridge_binds_a_watertight_genus0_ridge() {
	let dir = out_dir("thread_ridge");
	let r = run(
		&dir,
		json!([
			{"id": "ridge", "op": "thread_ridge", "m": 6.0, "length": 6.0},
			{"id": "check", "op": "assert", "in": "ridge", "valid": true, "genus": 0},
			{"id": "bb", "op": "bounding_box", "in": "ridge"},
			{"id": "vol", "op": "volume", "in": "ridge"},
		]),
	);
	let turns = num(&r, "ridge", "turns");
	let size_x = entry(&r, "bb").measures.as_ref().and_then(|m| m["size"][0].as_f64()).unwrap_or(f64::NAN);
	let vol = num(&r, "vol", "volume");
	assert!(
		r.ok && turns == 6.0 && (size_x - 6.0).abs() < 0.02 && vol > 0.0,
		"M6×6 ridge: turns={turns} bbox_x={size_x} (want ≈6, crest Ø) volume={vol} report={r:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (3) Param honesty: `m` mixed with `major_d`/`pitch` is refused, `major_d`
/// without `pitch` is refused, and a span over the 200-turn cap is refused
/// BEFORE any loft (allocation guard).
#[test]
fn thread_ridge_param_exclusivity_and_turns_cap() {
	let dir = out_dir("thread_ridge_params");
	for (id, op, needle) in [
		("mix", json!({"id": "mix", "op": "thread_ridge", "m": 6.0, "major_d": 6.0, "pitch": 1.0, "length": 5.0}), "either"),
		("half", json!({"id": "half", "op": "thread_ridge", "major_d": 6.0, "length": 5.0}), "either"),
		("cap", json!({"id": "cap", "op": "thread_ridge", "m": 3.0, "length": 150.0}), "cap"),
	] {
		let r = run(&dir, json!([op]));
		let e = entry(&r, id).error.as_ref().expect("must fail");
		assert!(
			e.kind == ErrorKind::InvalidParam && e.message.contains(needle),
			"'{id}' must fail invalid_param mentioning '{needle}': {r:#?}"
		);
	}
	let _ = std::fs::remove_dir_all(&dir);
}

/// (4) export_threaded EXTERNAL: an M6 ridge fused onto its root-Ø shank
/// through the voxel half is watertight and ADDS material — the in-tree
/// regression's guard, here re-asserted end-to-end through the JSON surface
/// (an M6×11 thread adds ≈45 mm³; asserted > 10 mm³, far above voxel noise).
#[test]
fn export_threaded_external_adds_material_and_is_watertight() {
	let dir = out_dir("export_threaded_ext");
	// Shank at the ISO root radius for M6 (3 − 0.625·H = 2.4588) on the +Z axis.
	let r = run(
		&dir,
		json!([
			{"id": "shank", "op": "cylinder", "base": [0, 0, 0], "axis": [0, 0, 1], "radius": 2.4588, "height": 12.0, "segments": 48},
			{"id": "stud", "op": "export_threaded", "in": "shank", "m": 6.0, "z0": 0.5, "length": 11.0, "file": "m6_stud.stl"},
		]),
	);
	let m = entry(&r, "stud").measures.as_ref().cloned().unwrap_or_default();
	let delta = m["volume_delta_vs_body"].as_f64().unwrap_or(f64::NAN);
	assert!(
		r.ok && m["route"] == json!("voxel_healed")
			&& m["watertight"] == json!(true)
			&& m["internal"] == json!(false)
			&& m["voxel"] == json!(0.125)
			&& delta > 10.0
			&& std::fs::metadata(dir.join("m6_stud.stl")).map(|f| f.len() > 0).unwrap_or(false),
		"M6 external stud: delta={delta} mm³ (want > 10, thread must ADD material) measures={m} report={r:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (5) export_threaded INTERNAL: the print-practical female cut (male-profile
/// ridge at Ø m+0.4 voxel-subtracted from the bore wall) is watertight and
/// REMOVES material (delta < 0) — and the receipt says internal + the route.
#[test]
fn export_threaded_internal_removes_material() {
	let dir = out_dir("export_threaded_int");
	let r = run(
		&dir,
		json!([
			{"id": "blank", "op": "cylinder", "base": [0, 0, 0], "axis": [0, 0, 1], "radius": 5.0, "height": 8.0, "segments": 48},
			{"id": "bored", "op": "drill", "in": "blank", "at": [0, 0, 8], "axis": [0, 0, -1], "d": 5.0, "through": 8.0},
			{"id": "nut", "op": "export_threaded", "in": "bored", "m": 6.0, "length": 8.0, "internal": true, "file": "m6_nut.stl"},
		]),
	);
	let m = entry(&r, "nut").measures.as_ref().cloned().unwrap_or_default();
	let delta = m["volume_delta_vs_body"].as_f64().unwrap_or(f64::NAN);
	assert!(
		r.ok && m["route"] == json!("voxel_implicit")
			&& m["watertight"] == json!(true)
			&& m["internal"] == json!(true)
			&& delta < -10.0
			&& std::fs::metadata(dir.join("m6_nut.stl")).map(|f| f.len() > 0).unwrap_or(false),
		"M6 internal cut: delta={delta} mm³ (want < -10, thread must REMOVE material) measures={m} report={r:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (6) The crest-smear voxel guard is deterministic: an M3 thread (pitch 0.5,
/// cap pitch/6 ≈ 0.083) at voxel 0.3 is REFUSED up front with the reason —
/// never silently degraded into a smooth band.
#[test]
fn export_threaded_voxel_guard_refuses_crest_smear() {
	let dir = out_dir("export_threaded_guard");
	let r = run(
		&dir,
		json!([
			{"id": "shank", "op": "cylinder", "base": [0, 0, 0], "axis": [0, 0, 1], "radius": 1.2, "height": 6.0},
			{"id": "stud", "op": "export_threaded", "in": "shank", "m": 3.0, "length": 5.0, "voxel": 0.3, "file": "m3.stl"},
		]),
	);
	let e = entry(&r, "stud").error.as_ref().expect("must fail");
	assert!(
		!r.ok && e.kind == ErrorKind::InvalidParam && e.message.contains("pitch/6") && e.message.contains("smear"),
		"voxel 0.3 on an M3 pitch 0.5 must be refused naming the pitch/6 cap: {r:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (7) A body that never meets the thread is refused DETERMINISTICALLY: a
/// shank 30 mm off the +Z thread axis does not overlap the thread span's
/// bounding box, so the op fails invalid_param naming the placement rule —
/// BEFORE any voxel work (a floating ridge would otherwise still "add volume"
/// as its own shell, which is why this is a pre-check, not the delta guard).
#[test]
fn export_threaded_misplaced_body_is_refused_up_front() {
	let dir = out_dir("export_threaded_miss");
	let r = run(
		&dir,
		json!([
			{"id": "shank", "op": "cylinder", "base": [30, 0, 0], "axis": [0, 0, 1], "radius": 2.4588, "height": 12.0},
			{"id": "stud", "op": "export_threaded", "in": "shank", "m": 6.0, "z0": 0.5, "length": 11.0, "file": "miss.stl"},
		]),
	);
	let e = entry(&r, "stud").error.as_ref().expect("must fail");
	assert!(
		!r.ok && e.kind == ErrorKind::InvalidParam && e.message.contains("+Z") && !dir.join("miss.stl").exists(),
		"a shank 30 mm off the thread axis must be refused up front: {r:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}
