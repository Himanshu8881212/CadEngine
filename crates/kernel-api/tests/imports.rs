// Copyright (c) LMCAD. Licensed under the MIT License.

//! Tier-1 wave 2 IMPORTS: STEP files back in as exact B-reps (`import_step`),
//! mesh files in with the honest `check_mesh` receipt (`import_mesh`), and the
//! solid∘mesh voxel boolean (`mesh_carve`) — round-trips, multi-shell honesty,
//! leaky-mesh honesty, sandbox refusals, and the empty-boolean refusal.

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

/// (1) The STEP round trip: a box ∪ cylinder-boss part exported to STEP and
/// re-imported IN THE SAME PROGRAM must conserve the faceted volume. Measured
/// round-trip error on this corpus is < 1e-9 relative (the exporter prints full
/// f64 precision); asserted at 1e-6 relative for slack, stated here honestly.
#[test]
fn step_round_trip_conserves_volume() {
	let dir = out_dir("step_roundtrip");
	let r = run(
		&dir,
		json!([
			{"id": "plate", "op": "box", "min": [0, 0, 0], "max": [20, 10, 5]},
			{"id": "boss", "op": "cylinder", "base": [10, 5, 5], "axis": [0, 0, 1], "radius": 3, "height": 6},
			{"id": "part", "op": "union", "a": "plate", "b": "boss"},
			{"id": "vol", "op": "volume", "in": "part"},
			{"id": "out", "op": "export_step", "in": "part", "file": "part.step"},
			{"id": "back", "op": "import_step", "file": "part.step"},
			{"id": "check", "op": "assert", "in": "back", "valid": true, "shells": 1},
		]),
	);
	let v_src = num(&r, "vol", "volume");
	let v_back = num(&r, "back", "volume");
	let rel = (v_back - v_src).abs() / v_src;
	let faces = num(&r, "back", "faces");
	let source = entry(&r, "back").measures.as_ref().and_then(|m| m["source"].as_str().map(String::from));
	assert!(
		r.ok && rel < 1e-6 && v_src > 1000.0 && faces > 6.0 && source.as_deref() == Some("step"),
		"STEP round trip must conserve volume: src={v_src} back={v_back} rel_err={rel:.3e} (want < 1e-6) faces={faces} source={source:?} report={r:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (2) Multi-solid honesty: two DISJOINT boxes union to a 2-shell solid; the
/// STEP round trip must come back as ONE solid with TWO shells (the documented
/// multi-solid merge), volume-conserving.
#[test]
fn step_multi_shell_import_binds_one_two_shell_solid() {
	let dir = out_dir("step_multishell");
	let r = run(
		&dir,
		json!([
			{"id": "a", "op": "box", "min": [0, 0, 0], "max": [10, 10, 10]},
			{"id": "b", "op": "box", "min": [30, 0, 0], "max": [40, 10, 10]},
			{"id": "pair", "op": "union", "a": "a", "b": "b"},
			{"id": "out", "op": "export_step", "in": "pair", "file": "pair.step"},
			{"id": "back", "op": "import_step", "file": "pair.step"},
			{"id": "check", "op": "assert", "in": "back", "valid": true, "shells": 2},
		]),
	);
	let shells = num(&r, "back", "shells");
	let v_back = num(&r, "back", "volume");
	assert!(
		r.ok && shells == 2.0 && (v_back - 2000.0).abs() < 1e-6,
		"2-shell STEP import: shells={shells} (want 2) volume={v_back} (want 2000) report={r:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (3) Structured failure for garbage: a file that is not STEP fails loudly
/// with the kernel's verbatim reason — never a panic. The kernel's tokenizer is
/// lenient (it skips unrecognized text), so plain garbage surfaces as
/// `StepError::Topology` ("no ADVANCED_FACE entities found") → invalid_geometry,
/// not as a parse error; asserted as measured.
#[test]
fn step_import_of_garbage_is_a_structured_failure() {
	let dir = out_dir("step_garbage");
	std::fs::write(dir.join("junk.step"), "this is not a STEP physical file").expect("write junk");
	let r = run(&dir, json!([{"id": "back", "op": "import_step", "file": "junk.step"}]));
	let e = entry(&r, "back").error.as_ref().expect("must fail");
	assert!(
		!r.ok && e.kind == ErrorKind::InvalidGeometry && e.message.contains("STEP topology error"),
		"garbage STEP must fail with the kernel's verbatim topology reason: {r:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (4) The V1 sandbox holds for the new input ops: `..` traversal and absolute
/// paths are refused with `invalid_param` for import_step, import_mesh AND the
/// mesh_carve operand file.
#[test]
fn import_paths_are_confined_to_the_sandbox() {
	let dir = out_dir("import_confined");
	for (id, op) in [
		("s1", json!({"id": "s1", "op": "import_step", "file": "../escape.step"})),
		("s2", json!({"id": "s2", "op": "import_step", "file": "/etc/passwd"})),
		("m1", json!({"id": "m1", "op": "import_mesh", "file": "../escape.stl"})),
		("c1", json!({"id": "c0", "op": "box", "min": [0,0,0], "max": [5,5,5]})),
	] {
		if id == "c1" {
			// mesh_carve needs a bound solid first; run its refusal as a 2-op program.
			let r = run(
				&dir,
				json!([op, {"id": "c1", "op": "mesh_carve", "in": "c0", "file": "../tool.stl", "bool": "difference", "out": "x.stl"}]),
			);
			let e = entry(&r, "c1").error.as_ref().expect("must fail");
			assert!(
				e.kind == ErrorKind::InvalidParam && e.message.contains(".."),
				"mesh_carve '..' must be refused invalid_param: {r:#?}"
			);
		} else {
			let r = run(&dir, json!([op]));
			let e = entry(&r, id).error.as_ref().expect("must fail");
			assert_eq!(e.kind, ErrorKind::InvalidParam, "path escape on '{id}' must be invalid_param: {r:#?}");
		}
	}
	let _ = std::fs::remove_dir_all(&dir);
}

/// (5) Mesh import round trip: an exported box STL re-imports watertight with
/// the exact enclosed volume, the full receipt, and an optional re-write; a
/// hand-written OBJ tetrahedron proves the second reader path.
#[test]
fn mesh_import_receipt_round_trip_and_obj() {
	let dir = out_dir("mesh_import");
	let r = run(
		&dir,
		json!([
			{"id": "b", "op": "box", "min": [0, 0, 0], "max": [20, 20, 20]},
			{"id": "out", "op": "export_stl", "in": "b", "file": "box.stl"},
			{"id": "back", "op": "import_mesh", "file": "box.stl", "out": "again.3mf"},
		]),
	);
	let vol = num(&r, "back", "volume");
	let watertight = entry(&r, "back").measures.as_ref().and_then(|m| m["watertight"].as_bool());
	let fmt = entry(&r, "back").measures.as_ref().and_then(|m| m["format"].as_str().map(String::from));
	assert!(
		r.ok && watertight == Some(true)
			&& (vol - 8000.0).abs() < 1e-3
			&& fmt.as_deref() == Some("stl")
			&& std::fs::metadata(dir.join("again.3mf")).map(|m| m.len() > 0).unwrap_or(false),
		"box STL round trip: watertight={watertight:?} volume={vol} (want 8000) format={fmt:?} report={r:#?}"
	);

	// OBJ path: a hand-written outward tetrahedron, volume 10³/6 = 166.667 mm³.
	std::fs::write(
		dir.join("tet.obj"),
		"v 0 0 0\nv 10 0 0\nv 0 10 0\nv 0 0 10\nf 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n",
	)
	.expect("write obj");
	let r = run(&dir, json!([{"id": "tet", "op": "import_mesh", "file": "tet.obj"}]));
	let vol = num(&r, "tet", "volume");
	assert!(
		r.ok && (vol - 1000.0 / 6.0).abs() < 1e-3,
		"OBJ tetrahedron: volume={vol} (want {}) report={r:#?}",
		1000.0 / 6.0
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (6) Leaky-mesh honesty: a box STL with one facet surgically removed reports
/// watertight:false with boundary_edges=3 and NO volume key (a leaky mesh has
/// no defined enclosed volume); with heal:true the hole is capped and the exact
/// volume returns.
#[test]
fn leaky_mesh_reports_honestly_and_heals() {
	let dir = out_dir("mesh_leaky");
	let r = run(
		&dir,
		json!([
			{"id": "b", "op": "box", "min": [0, 0, 0], "max": [20, 20, 20]},
			{"id": "out", "op": "export_stl", "in": "b", "file": "box.stl"},
		]),
	);
	assert!(r.ok, "box export must succeed: {r:#?}");
	// Binary STL surgery: 80-byte header + u32 count + 50 bytes/triangle — drop the last one.
	let bytes = std::fs::read(dir.join("box.stl")).expect("read stl");
	let count = u32::from_le_bytes(bytes[80..84].try_into().unwrap());
	let mut cut = bytes[..bytes.len() - 50].to_vec();
	cut[80..84].copy_from_slice(&(count - 1).to_le_bytes());
	std::fs::write(dir.join("leaky.stl"), &cut).expect("write leaky stl");

	let r = run(&dir, json!([{"id": "raw", "op": "import_mesh", "file": "leaky.stl"}]));
	let m = entry(&r, "raw").measures.as_ref().expect("measures");
	assert!(
		r.ok && m["watertight"] == json!(false) && m["boundary_edges"] == json!(3) && m.get("volume").is_none(),
		"leaky receipt must be honest (watertight:false, 3 boundary edges, volume OMITTED): {r:#?}"
	);

	let r = run(&dir, json!([{"id": "fix", "op": "import_mesh", "file": "leaky.stl", "heal": true}]));
	let m = entry(&r, "fix").measures.as_ref().expect("measures");
	let vol = m["volume"].as_f64().unwrap_or(f64::NAN);
	assert!(
		r.ok && m["watertight"] == json!(true) && m["healed"] == json!(true) && (vol - 8000.0).abs() < 1e-3,
		"healed leaky box must be watertight at the exact volume (planar fan cap): volume={vol} report={r:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (7) mesh_carve: a sphere STL carved out of a bound box through the voxel
/// boolean — the result is guaranteed watertight, route "voxel_implicit", and
/// its volume matches (box − sphere) computed from the SAME program's own
/// measures within 2% (the seam is voxel-resampled at 0.3 mm — never exact).
#[test]
fn mesh_carve_difference_is_watertight_and_voxel_accurate() {
	let dir = out_dir("mesh_carve");
	let r = run(
		&dir,
		json!([
			{"id": "tool", "op": "sphere", "center": [10, 10, 10], "radius": 6},
			{"id": "tool_vol", "op": "volume", "in": "tool"},
			{"id": "tool_stl", "op": "export_stl", "in": "tool", "file": "tool.stl"},
			{"id": "stock", "op": "box", "min": [0, 0, 0], "max": [20, 20, 20]},
			{"id": "carved", "op": "mesh_carve", "in": "stock", "file": "tool.stl", "bool": "difference", "voxel": 0.3, "out": "carved.stl"},
		]),
	);
	let sphere_vol = num(&r, "tool_vol", "volume");
	let carved_vol = num(&r, "carved", "volume");
	let expected = 8000.0 - sphere_vol;
	let route = entry(&r, "carved").measures.as_ref().and_then(|m| m["route"].as_str().map(String::from));
	let watertight = entry(&r, "carved").measures.as_ref().and_then(|m| m["watertight"].as_bool());
	assert!(
		r.ok && route.as_deref() == Some("voxel_implicit")
			&& watertight == Some(true)
			&& (carved_vol - expected).abs() < 0.02 * expected
			&& std::fs::metadata(dir.join("carved.stl")).map(|m| m.len() > 0).unwrap_or(false),
		"carved box: volume={carved_vol} (want {expected}±2%) route={route:?} watertight={watertight:?} report={r:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (8) An EMPTY boolean result refuses loudly: intersecting a box with a
/// disjoint tool mesh yields nothing, and mesh_carve fails `invalid_geometry`
/// instead of writing an empty file.
#[test]
fn mesh_carve_empty_intersection_refuses() {
	let dir = out_dir("mesh_carve_empty");
	let r = run(
		&dir,
		json!([
			{"id": "tool", "op": "sphere", "center": [100, 100, 100], "radius": 5},
			{"id": "tool_stl", "op": "export_stl", "in": "tool", "file": "far.stl"},
			{"id": "stock", "op": "box", "min": [0, 0, 0], "max": [20, 20, 20]},
			{"id": "cut", "op": "mesh_carve", "in": "stock", "file": "far.stl", "bool": "intersection", "voxel": 0.3, "out": "empty.stl"},
		]),
	);
	let e = entry(&r, "cut").error.as_ref().expect("must fail");
	assert!(
		!r.ok && e.kind == ErrorKind::InvalidGeometry && !dir.join("empty.stl").exists(),
		"empty intersection must refuse invalid_geometry and write nothing: {r:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// T4 (path-root asymmetry, 7/10 campaigns): `export_step` writes under
/// `--out-dir`, but `import_step` used to resolve ONLY against the program's
/// directory — so a write-then-read-back program failed with `io` unless the
/// two roots happened to coincide. The read now falls back to `--out-dir`
/// when the file is not beside the program; a total miss names both roots.
#[test]
fn step_round_trip_resolves_across_the_out_dir_root() {
	let dir = std::env::temp_dir().join(format!("cadcode_t4_out_{}", std::process::id()));
	let out = dir.join("out"); // deliberately NOT the program's directory
	std::fs::create_dir_all(&out).unwrap();

	let program = serde_json::json!({"ops": [
		{"id": "p", "op": "box", "min": [0, 0, 0], "max": [12, 8, 4]},
		{"id": "w", "op": "export_step", "in": "p", "file": "rt/probe.step"},
		{"id": "r", "op": "import_step", "file": "rt/probe.step"},
		{"id": "g", "op": "assert", "in": "r", "exact_volume_within": {"target": 384.0, "percent": 0.5}}
	]});
	// input base = `dir` (where the "program" lives), out dir = `dir/out`.
	let report = kernel_api::run_program_with_input_base(&program.to_string(), &out, &dir);
	assert!(
		report.ok,
		"write-then-import must resolve across roots (write lands under --out-dir): {report:#?}"
	);

	// A total miss still refuses loudly, naming BOTH roots it tried.
	let missing = serde_json::json!({"ops": [
		{"id": "r", "op": "import_step", "file": "rt/nope.step"}
	]});
	let report = kernel_api::run_program_with_input_base(&missing.to_string(), &out, &dir);
	let e = report.ops[0].error.as_ref().expect("io error");
	assert!(
		e.message.contains("beside the program") && e.message.contains("--out-dir"),
		"the miss must name both tried roots: {}",
		e.message
	);
	let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn bare_program_filename_empty_input_base_still_resolves() {
	// `Path::parent()` of a bare program filename ("part.json") is the EMPTY
	// path. The sandbox join used to canonicalize "" and fail with "cannot
	// canonicalize sandbox ''" BEFORE the out-dir fallback could run — which
	// broke every campaign whose Reproducing invokes `run <prog>.json` from
	// inside programs/ (the cleat's README does exactly that). Empty base
	// means the current directory; this pins the fix.
	let dir = std::env::temp_dir().join(format!("lmcad_bare_base_{}", std::process::id()));
	let out = dir.join("out");
	std::fs::create_dir_all(&out).unwrap();
	let program = serde_json::json!({"ops": [
		{"id": "p", "op": "box", "min": [0, 0, 0], "max": [10, 6, 3]},
		{"id": "w", "op": "export_step", "in": "p", "file": "rt/bare.step"},
		{"id": "r", "op": "import_step", "file": "rt/bare.step"},
		{"id": "g", "op": "assert", "in": "r", "exact_volume_within": {"target": 180.0, "percent": 0.5}}
	]});
	let report = kernel_api::run_program_with_input_base(&program.to_string(), &out, std::path::Path::new(""));
	assert!(
		report.ok,
		"an empty input base (bare program filename) must behave as the current directory, not refuse on canonicalize: {report:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}
