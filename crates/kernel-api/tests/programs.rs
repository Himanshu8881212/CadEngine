// Copyright (c) LMCAD. Licensed under the MIT License.

//! End-to-end tests of the JSON binding (AI-Interface I1 + I4): real parts built
//! through `run_program` with NO direct Rust geometry calls, structured error
//! paths, report round-tripping, and the CLI binary contract.

// The wave-3 catalog program is one big `json!` literal; its nesting exceeds the
// default macro recursion limit (128).
#![recursion_limit = "256"]

use std::path::{Path, PathBuf};

use kernel_api::{run_program, ErrorKind, OpReport, Report};
use serde_json::json;

/// A unique per-test output directory under the system temp dir.
fn out_dir(name: &str) -> PathBuf {
	let dir = std::env::temp_dir().join(format!("kernel_api_{name}_{}", std::process::id()));
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

/// True when `path` exists and is non-empty.
fn file_ok(path: &Path) -> bool {
	std::fs::metadata(path).map(|m| m.len() > 0).unwrap_or(false)
}

/// (1) The I1 end-to-end proof: the FLANGE built ONLY through the JSON binding —
/// one revolved L-cross-section (64 sectors) minus a 6 × Ø7 bolt circle at R30 —
/// then validated (genus 7), measured, and exported as STL + STEP.
#[test]
fn flange_program_end_to_end() {
	let dir = out_dir("flange");
	let mut ops = vec![json!({
		"id": "body", "op": "revolve",
		"profile": [[10.0, 0.0], [40.0, 0.0], [40.0, 7.0], [39.0, 8.0], [10.0, 8.0]],
		"segments": 64,
	})];
	let mut prev = "body".to_string();
	for i in 0..6 {
		let a = i as f64 * std::f64::consts::PI / 3.0;
		ops.push(json!({
			"id": format!("hole{i}"), "op": "cylinder",
			"base": [30.0 * a.cos(), 30.0 * a.sin(), -1.0], "axis": [0.0, 0.0, 1.0],
			"radius": 3.5, "height": 10.0, "segments": 24,
		}));
		ops.push(json!({"id": format!("cut{i}"), "op": "difference", "a": prev, "b": format!("hole{i}")}));
		prev = format!("cut{i}");
	}
	ops.push(json!({"id": "check", "op": "validate", "in": prev}));
	ops.push(json!({"id": "vol", "op": "volume", "in": prev}));
	ops.push(json!({"id": "stl", "op": "export_stl", "in": prev, "file": "flange.stl", "tol": 0.01}));
	ops.push(json!({"id": "step", "op": "export_step", "in": prev, "file": "flange.step"}));
	let program = serde_json::to_string(&json!({ "ops": ops })).expect("serialize program");

	let report = run_program(&program, &dir);
	let genus = entry(&report, "check").measures.as_ref().and_then(|m| m["genus"].as_i64());
	let volume = entry(&report, "vol").measures.as_ref().and_then(|m| m["volume"].as_f64()).unwrap_or(f64::NAN);
	// Faceted closed form: Pappus 2π·M/6 scaled by the 64-gon ratio, minus six
	// 24-gon hole prisms ≈ 35688 mm³; the spec target 35686 is well inside 1%.
	let expected = 35686.0;
	assert!(
		report.ok
			&& genus == Some(7)
			&& (volume - expected).abs() <= 0.01 * expected
			&& file_ok(&dir.join("flange.stl"))
			&& file_ok(&dir.join("flange.step")),
		"flange through the JSON binding: ok={} genus={genus:?} volume={volume} (want {expected}±1%) report={report:#?}",
		report.ok
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (2) Sketch path: a deliberately skewed quad pinned to an exact 60 × 40
/// rectangle by constraints (well-constrained, converged), extruded to height 10
/// — the volume must be 24000 mm³ to solver precision (planar faces are exact).
#[test]
fn sketch_program_constrained_rectangle() {
	let dir = out_dir("sketch");
	let program = json!({"ops": [
		{"id": "sk", "op": "sketch",
			"points": [[1.0, 2.0], [55.0, 3.0], [58.0, 44.0], [-2.0, 38.0]],
			"segments": [[0, 1], [1, 2], [2, 3], [3, 0]],
			"constraints": [
				{"kind": "fixed", "point": 0, "at": [0.0, 0.0]},
				{"kind": "horizontal", "a": 0, "b": 1},
				{"kind": "distance", "a": 0, "b": 1, "distance": 60.0},
				{"kind": "vertical", "a": 0, "b": 3},
				{"kind": "distance", "a": 0, "b": 3, "distance": 40.0},
				{"kind": "horizontal", "a": 3, "b": 2},
				{"kind": "vertical", "a": 1, "b": 2}
			]},
		{"id": "solid", "op": "sketch_extrude", "sketch": "sk", "height": 10.0},
		{"id": "vol", "op": "volume", "in": "solid"}
	]});
	let report = run_program(&serde_json::to_string(&program).expect("serialize"), &dir);
	let sk = entry(&report, "sk").measures.as_ref().cloned().unwrap_or_default();
	let volume = entry(&report, "vol").measures.as_ref().and_then(|m| m["volume"].as_f64()).unwrap_or(f64::NAN);
	assert!(
		report.ok
			&& sk["state"] == json!("well_constrained")
			&& sk["converged"] == json!(true)
			&& sk["free_dof"] == json!(0)
			&& (volume - 24000.0).abs() < 1e-3,
		"constrained rectangle: volume={volume} (want 24000 exactly) sketch measures={sk} report={report:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (3) Error paths are STRUCTURED: a machine-matchable kind + the failing op id,
/// overall `ok == false` (the CLI exit contract), and no execution past the
/// first failure.
#[test]
fn error_paths_are_structured() {
	let dir = out_dir("errors");

	// (a) Unknown op — and the following op must NOT run (stop at first failure).
	let r = run_program(
		r#"{"ops": [
			{"id": "weird", "op": "frobnicate"},
			{"id": "never", "op": "box", "min": [0,0,0], "max": [1,1,1]}
		]}"#,
		&dir,
	);
	let e = r.ops[0].error.as_ref().expect("error entry");
	assert!(
		!r.ok && r.ops.len() == 1 && e.kind == ErrorKind::UnknownOp && e.message.contains("'weird'") && e.message.contains("frobnicate"),
		"unknown-op path: {r:#?}"
	);

	// (b) Dangling reference.
	let r = run_program(r#"{"ops": [{"id": "u", "op": "union", "a": "nope", "b": "alsonope"}]}"#, &dir);
	let e = r.ops[0].error.as_ref().expect("error entry");
	assert!(
		!r.ok && e.kind == ErrorKind::MissingRef && e.message.contains("'u'") && e.message.contains("'nope'"),
		"dangling-ref path: {r:#?}"
	);

	// (c) A fillet witness matching nothing (848 mm from the nearest edge of a
	// 10 mm box — far beyond the 10%-of-diagonal guard).
	let r = run_program(
		r#"{"ops": [
			{"id": "b", "op": "box", "min": [0,0,0], "max": [10,10,10]},
			{"id": "f", "op": "fillet_edge_near", "in": "b", "witness": [500,500,500], "radius": 1}
		]}"#,
		&dir,
	);
	let e = r.ops[1].error.as_ref().expect("error entry");
	assert!(
		!r.ok && r.ops[0].ok && e.kind == ErrorKind::FeatureFailed && e.message.contains("'f'") && e.message.contains("witness"),
		"far-witness path: {r:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (3b) The concave-junction refusal names the REAL reason. The inside edge of
/// an L (two unioned boxes) IS a straight edge between two perpendicular planar
/// faces, so the old message ("not a straight edge between two perpendicular
/// planar faces") misled a blind agent in live fire — the actual refusal is
/// convexity: a fillet/chamfer removes material, a concave junction needs
/// material added. Pins the enriched message: it must say "concave", state the
/// supported scope (convex straight edges + convex rims via
/// fillet_circular_rim), and suggest the honest cove workaround — for BOTH
/// fillet_edge_near and chamfer_edge_near (they share the convexity check).
#[test]
fn concave_junction_refusal_names_convexity_scope() {
	let dir = out_dir("concave_fillet");
	let l_solid = |feature: &str| {
		format!(
			r#"{{"ops": [
				{{"id": "base", "op": "box", "min": [0,0,0], "max": [40,20,10]}},
				{{"id": "wall", "op": "box", "min": [0,0,10], "max": [10,20,30]}},
				{{"id": "L", "op": "union", "a": "base", "b": "wall"}},
				{{"id": "f", "op": "{feature}", "in": "L", "witness": [10,10,10], "radius": 3}}
			]}}"#
		)
	};
	let fillet = run_program(&l_solid("fillet_edge_near"), &dir);
	let chamfer = run_program(&l_solid("chamfer_edge_near"), &dir);
	let fe = fillet.ops[3].error.as_ref().expect("fillet error entry");
	let ce = chamfer.ops[3].error.as_ref().expect("chamfer error entry");
	let enriched = |m: &str| {
		m.to_lowercase().contains("concave")
			&& m.contains("CONVEX straight edges between two planar faces")
			&& m.contains("fillet_circular_rim")
			&& m.contains("quarter-round")
	};
	assert!(
		!fillet.ok
			&& !chamfer.ok
			&& fillet.ops[2].ok // the L itself builds — only the feature refuses
			&& fe.kind == ErrorKind::FeatureFailed
			&& ce.kind == ErrorKind::FeatureFailed
			&& enriched(&fe.message)
			&& enriched(&ce.message),
		"concave-junction refusal must name convexity + the supported scope + the cove workaround for both ops;\nfillet: {fe:#?}\nchamfer: {ce:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (4) The report round-trips through serde unchanged — for a success report
/// (measures + file present) and a failure report (structured error present).
#[test]
fn report_roundtrips_serde() {
	let dir = out_dir("roundtrip");
	let success = run_program(
		r#"{"ops": [
			{"id": "b", "op": "box", "min": [0,0,0], "max": [20,10,5]},
			{"id": "v", "op": "validate", "in": "b"},
			{"id": "s", "op": "export_stl", "in": "b", "file": "rt.stl"}
		]}"#,
		&dir,
	);
	let failure = run_program(r#"{"ops": [{"id": "x", "op": "frobnicate"}]}"#, &dir);
	let success_back: Report =
		serde_json::from_str(&serde_json::to_string(&success).expect("serialize")).expect("deserialize success report");
	let failure_back: Report =
		serde_json::from_str(&serde_json::to_string(&failure).expect("serialize")).expect("deserialize failure report");
	assert_eq!((&success, &failure), (&success_back, &failure_back), "reports must round-trip bit-equal through JSON");
	assert!(success.ok && !failure.ok, "fixture sanity: success={success:#?} failure={failure:#?}");
	let _ = std::fs::remove_dir_all(&dir);
}

/// (5) Breadth: every op of the vocabulary not already exercised above runs
/// green in ONE program — features by witness, transforms, all measures, all
/// exports, the gyroid lattice — proving each API.md example family executes.
#[test]
fn breadth_program_covers_remaining_ops() {
	let dir = out_dir("breadth");
	let program = json!({"ops": [
		{"id": "base", "op": "box", "min": [0,0,0], "max": [30,20,10]},
		{"id": "fe", "op": "fillet_edge_near", "in": "base", "witness": [15,0,10], "radius": 2},
		{"id": "ce", "op": "chamfer_edge_near", "in": "fe", "witness": [15,20,10], "radius": 2},
		{"id": "sp", "op": "sphere", "center": [15,10,10], "radius": 6, "u": 24, "v": 12},
		{"id": "uni", "op": "union", "a": "ce", "b": "sp"},
		{"id": "bx2", "op": "box", "min": [-5,-5,-5], "max": [8,8,8]},
		{"id": "inter", "op": "intersection", "a": "base", "b": "bx2"},
		{"id": "tr", "op": "translate", "in": "uni", "offset": [100,0,0]},
		{"id": "rz", "op": "rotate_z", "in": "tr", "degrees": 30},
		{"id": "xv", "op": "exact_volume", "in": "rz"},
		{"id": "mp", "op": "mass_properties", "in": "rz"},
		{"id": "wt", "op": "wall_thickness", "in": "rz", "flag_below": 1},
		{"id": "da", "op": "draft_analysis", "in": "rz", "pull": [0,0,1], "min_deg": 1},
		{"id": "cyl", "op": "cylinder", "base": [50,10,0], "axis": [0,0,1], "radius": 6, "height": 12, "segments": 48},
		{"id": "rim", "op": "fillet_circular_rim", "in": "cyl", "witness": [56,10,12], "radius": 1.5, "arc_segments": 6},
		{"id": "rimcheck", "op": "validate", "in": "rim"},
		{"id": "cone1", "op": "cone", "base": [80,0,0], "axis": [0,0,1], "radius": 5, "height": 12, "segments": 32},
		{"id": "tor", "op": "torus", "center": [0,0,50], "axis": [0,0,1], "major": 20, "minor": 5},
		{"id": "ex", "op": "extrude", "profile": [[0,0],[20,0],[20,10],[0,10]], "height": 5},
		{"id": "exh", "op": "extrude_with_holes", "outer": [[0,0],[40,0],[40,30],[0,30]],
			"holes": [[[10,10],[20,10],[20,20],[10,20]]], "height": 6},
		{"id": "et", "op": "extrude_tapered", "profile": [[0,0],[30,0],[30,20],[0,20]], "height": 10, "draft_deg": 2},
		{"id": "sk2", "op": "sketch", "points": [[10,0],[16,0],[16,6],[10,6]], "segments": [[0,1],[1,2],[2,3],[3,0]]},
		{"id": "skr", "op": "sketch_revolve", "sketch": "sk2", "segments": 32},
		{"id": "stl", "op": "export_stl", "in": "rz", "file": "breadth.stl"},
		{"id": "stp", "op": "export_step", "in": "rim", "file": "breadth_rim.step"},
		{"id": "mf3", "op": "export_3mf", "in": "inter", "file": "breadth_inter.3mf"},
		{"id": "gy", "op": "gyroid_block", "center": [0,0,0], "half": 5, "scale": 0.9,
			"thickness": 0.5, "voxel": 0.25, "file": "breadth_gy.stl"}
	]});
	let report = run_program(&serde_json::to_string(&program).expect("serialize"), &dir);
	let rim_genus = entry(&report, "rimcheck").measures.as_ref().and_then(|m| m["genus"].as_i64());
	let gy_watertight = entry(&report, "gy").measures.as_ref().map(|m| m["watertight"] == json!(true));
	let files = ["breadth.stl", "breadth_rim.step", "breadth_inter.3mf", "breadth_gy.stl"];
	assert!(
		report.ok
			&& report.ops.len() == 27
			&& rim_genus == Some(0)
			&& gy_watertight == Some(true)
			&& files.iter().all(|f| file_ok(&dir.join(f))),
		"breadth program (every remaining op): rim_genus={rim_genus:?} gyroid_watertight={gy_watertight:?} report={report:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (7) The native-format consume path (I3b): a `.lmcpart` saved by the kernel
/// is LOADED through the JSON binding, machined with every hole-wizard op
/// (counterbore / blind drill / tap-drill through / countersink / clearance),
/// validated (genus 4 = the four through-holes), measured against the 32-gon
/// closed form, and exported as STL.
#[test]
fn load_part_and_hole_wizard_program() {
	let dir = out_dir("load_part_holes");
	// A 40 × 30 × 10 plate, saved as a native part file next to the program's outputs.
	let mut doc = kernel_model::Document::new();
	let plate = doc.add(kernel_model::Feature::Box {
		center: [kernel_model::Dim::Literal(20.0), kernel_model::Dim::Literal(15.0), kernel_model::Dim::Literal(5.0)],
		size: [kernel_model::Dim::Literal(40.0), kernel_model::Dim::Literal(30.0), kernel_model::Dim::Literal(10.0)],
	});
	doc.set_root(plate);
	std::fs::write(dir.join("plate.lmcpart"), kernel_model::format::save_part(&doc, "hole wizard plate")).expect("write part file");

	let program = json!({"ops": [
		{"id": "plate", "op": "load_part", "file": "plate.lmcpart"},
		{"id": "cb", "op": "counterbore_hole", "in": "plate", "at": [20, 15, 10], "axis": [0, 0, -1], "m": 5},
		{"id": "vol_cb", "op": "volume", "in": "cb"},
		{"id": "pin", "op": "drill", "in": "cb", "at": [8, 8, 10], "axis": [0, 0, -1], "d": 3, "depth": 4},
		{"id": "tap", "op": "tap_drill_hole", "in": "pin", "at": [32, 8, 10], "axis": [0, 0, -1], "m": 6, "through": 10},
		{"id": "csk", "op": "countersink_hole", "in": "tap", "at": [8, 22, 10], "axis": [0, 0, -1], "m": 5},
		{"id": "clr", "op": "clearance_hole", "in": "csk", "at": [32, 22, 10], "axis": [0, 0, -1], "m": 3},
		{"id": "check", "op": "validate", "in": "clr"},
		{"id": "stl", "op": "export_stl", "in": "clr", "file": "machined_plate.stl"}
	]});
	let report = run_program(&serde_json::to_string(&program).expect("serialize"), &dir);
	let loaded_name = entry(&report, "plate").measures.as_ref().and_then(|m| m["name"].as_str().map(str::to_string));
	let vol_cb = entry(&report, "vol_cb").measures.as_ref().and_then(|m| m["volume"].as_f64()).unwrap_or(f64::NAN);
	let genus = entry(&report, "check").measures.as_ref().and_then(|m| m["genus"].as_i64());
	// Counterbored plate, 32-gon closed form: 12000 − Ø5.5 bore prism (236.06)
	// − Ø10 × 5.8 counterbore annulus (315.69) = 11448.25.
	assert!(
		report.ok
			&& loaded_name.as_deref() == Some("hole wizard plate")
			&& (vol_cb - 11448.25).abs() < 0.1
			&& genus == Some(4)
			&& file_ok(&dir.join("machined_plate.stl")),
		"load_part + hole wizard: name={loaded_name:?} vol_cb={vol_cb} (want 11448.25) genus={genus:?} report={report:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (8) The standard-parts catalog through the JSON binding: all eight part ops
/// build, validate (expected genus where it is structural), measure and export
/// in ONE program — an AI can request standard components by dimension.
#[test]
fn catalog_parts_program() {
	let dir = out_dir("catalog");
	let program = json!({"ops": [
		{"id": "gear", "op": "spur_gear", "module": 2, "teeth": 20, "face_width": 10, "bore": 8, "keyway": true},
		{"id": "gear_check", "op": "validate", "in": "gear"},
		{"id": "bolt", "op": "hex_bolt", "m": 10, "length": 30},
		{"id": "nut", "op": "hex_nut", "m": 5},
		{"id": "washer", "op": "washer", "m": 5},
		{"id": "washer_vol", "op": "volume", "in": "washer"},
		{"id": "shcs", "op": "socket_head_cap_screw", "m": 5, "length": 16},
		{"id": "pulley", "op": "gt2_pulley", "teeth": 20, "belt_width": 6, "bore": 5, "flanged": true},
		{"id": "pulley_check", "op": "validate", "in": "pulley"},
		{"id": "sprocket", "op": "chain_sprocket", "pitch": 6.35, "roller_d": 3.302, "teeth": 12, "bore": 5},
		{"id": "shaft", "op": "shaft", "d": 8, "length": 40, "keyway": {"length": 20, "offset": 5}},
		{"id": "shaft_check", "op": "validate", "in": "shaft"},
		{"id": "stl", "op": "export_stl", "in": "gear", "file": "gear.stl"}
	]});
	let report = run_program(&serde_json::to_string(&program).expect("serialize"), &dir);
	let genus = |id: &str| entry(&report, id).measures.as_ref().and_then(|m| m["genus"].as_i64());
	let washer_vol = entry(&report, "washer_vol").measures.as_ref().and_then(|m| m["volume"].as_f64()).unwrap_or(f64::NAN);
	// ISO 7089 M5 washer: π(5² − 2.65²) × 1 ≈ 56.48 mm³ (64-gon ≈ 0.08% under).
	assert!(
		report.ok
			&& genus("gear_check") == Some(1)
			&& genus("pulley_check") == Some(1)
			&& genus("shaft_check") == Some(0)
			&& (washer_vol - 56.48).abs() < 0.6
			&& file_ok(&dir.join("gear.stl")),
		"catalog program: gear_genus={:?} pulley_genus={:?} shaft_genus={:?} washer_vol={washer_vol} (want ~56.48) report={report:#?}",
		genus("gear_check"),
		genus("pulley_check"),
		genus("shaft_check")
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (9) Catalog/hole failure paths are STRUCTURED: an out-of-table size (the
/// DIN 74 countersink table starts at M3, so M2 must fail), a drill with no
/// depth mode, an out-of-table keyway, and an unreadable part file each carry a
/// machine-matchable kind plus a message naming the offender.
#[test]
fn catalog_and_hole_error_paths_are_structured() {
	let dir = out_dir("hole_errors");

	// (a) M2 countersink: in the metric table, but below the DIN 74 form F range.
	let r = run_program(
		r#"{"ops": [
			{"id": "b", "op": "box", "min": [0,0,0], "max": [30,20,10]},
			{"id": "csk", "op": "countersink_hole", "in": "b", "at": [15,10,10], "axis": [0,0,-1], "m": 2}
		]}"#,
		&dir,
	);
	let e = r.ops[1].error.as_ref().expect("error entry");
	assert!(
		!r.ok && e.kind == ErrorKind::InvalidParam && e.message.contains("'csk'") && e.message.contains("M2"),
		"M2 countersink must fail structurally: {r:#?}"
	);

	// (b) A drill must say whether it is blind ('depth') or through ('through').
	let r = run_program(
		r#"{"ops": [
			{"id": "b", "op": "box", "min": [0,0,0], "max": [30,20,10]},
			{"id": "d", "op": "drill", "in": "b", "at": [15,10,10], "axis": [0,0,-1], "d": 5}
		]}"#,
		&dir,
	);
	let e = r.ops[1].error.as_ref().expect("error entry");
	assert!(
		!r.ok && e.kind == ErrorKind::InvalidParam && e.message.contains("'depth'") && e.message.contains("'through'"),
		"depth-mode-less drill must fail structurally: {r:#?}"
	);

	// (c) A shaft keyway outside the DIN 6885-1 diameter range.
	let r = run_program(
		r#"{"ops": [{"id": "s", "op": "shaft", "d": 200, "length": 50, "keyway": {"length": 20, "offset": 5}}]}"#,
		&dir,
	);
	let e = r.ops[0].error.as_ref().expect("error entry");
	assert!(
		!r.ok && e.kind == ErrorKind::InvalidParam && e.message.contains("DIN 6885-1"),
		"out-of-table keyway must fail structurally: {r:#?}"
	);

	// (d) load_part on a missing file (io) and on a non-part file (invalid_param).
	let r = run_program(r#"{"ops": [{"id": "p", "op": "load_part", "file": "nowhere.lmcpart"}]}"#, &dir);
	let missing_kind = r.ops[0].error.as_ref().map(|e| e.kind);
	std::fs::write(dir.join("not_a_part.lmcpart"), "{\"format\": \"something-else\"}").expect("write bogus part");
	let r2 = run_program(r#"{"ops": [{"id": "p", "op": "load_part", "file": "not_a_part.lmcpart"}]}"#, &dir);
	let bogus = r2.ops[0].error.as_ref().expect("error entry");
	assert!(
		missing_kind == Some(ErrorKind::Io) && bogus.kind == ErrorKind::InvalidParam && bogus.message.contains("lmc-part"),
		"load_part failures must be structured: missing={r:#?} bogus={r2:#?}"
	);

	// (e) Wave-2 catalog tables: an out-of-table dowel Ø, an unknown AS568 dash,
	// an unsupported fit string, and a belt drive with overlapping pitch circles —
	// each a loud invalid_param naming its table/domain.
	let cases = [
		(r#"{"ops": [{"id": "x", "op": "dowel_pin", "d": 7, "length": 20}]}"#, "ISO 2338"),
		(r#"{"ops": [{"id": "x", "op": "o_ring", "dash": 999}]}"#, "AS568"),
		(r#"{"ops": [{"id": "x", "op": "iso286_fit", "d": 25, "fit": "H7/u6"}]}"#, "H8/f7"),
		(r#"{"ops": [{"id": "x", "op": "gt2_belt", "center_distance": 5, "t1": 20, "t2": 20}]}"#, "pitch"),
		// Wave-3: an unstocked metric cord cross-section names the stocked list.
		(r#"{"ops": [{"id": "x", "op": "o_ring_cord", "ring_id": 150, "cord_d": 2.3}]}"#, "stocked metric cord"),
		(r#"{"ops": [{"id": "x", "op": "metric_cord_gland", "cord_d": 2.3}]}"#, "stocked metric cord"),
	];
	let wave2: Vec<(ErrorKind, bool)> = cases
		.iter()
		.map(|(program, needle)| {
			let r = run_program(program, &dir);
			let e = r.ops[0].error.as_ref().unwrap_or_else(|| panic!("error entry for {program}: {r:#?}"));
			(e.kind, e.message.contains(needle))
		})
		.collect();
	assert!(
		wave2.iter().all(|(kind, named)| *kind == ErrorKind::InvalidParam && *named),
		"wave-2 table failures must be invalid_param and name their table: {wave2:?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (10) Wave-2 catalog through the JSON binding: every new part, feature-cut and
/// design-math op in ONE program — sealing (AS568 gland + O-ring), retaining
/// (circlips + both grooves), the screw/pin/spacer breadth, 3D-printing stock
/// (extrusions, tee nut, heat-set boss), rack-and-ring gearing, and the
/// belt/fit lookups, with spot-checked measures.
#[test]
fn wave2_catalog_program_end_to_end() {
	let dir = out_dir("wave2");
	let program = json!({"ops": [
		// Sealing: a piston at the AS568-214 design Ø with its Parker gland.
		{"id": "piston", "op": "cylinder", "base": [0,0,0], "axis": [0,0,1], "radius": 15.315, "height": 30, "segments": 48},
		{"id": "gland", "op": "o_ring_groove", "in": "piston", "at": [0,0,12], "axis": [0,0,1], "dash": 214},
		{"id": "seal", "op": "o_ring", "dash": 214},
		{"id": "v_seal", "op": "validate", "in": "seal"},
		// Retaining: DIN 471 groove + clip on a Ø20 shaft; DIN 472 channel + clip in a bored block.
		{"id": "axle", "op": "shaft", "d": 20, "length": 40},
		{"id": "grooved", "op": "circlip_groove_external", "in": "axle", "at": [0,0,32], "axis": [0,0,1], "shaft_d": 20},
		{"id": "clip", "op": "circlip_external", "shaft_d": 20},
		{"id": "v_clip", "op": "validate", "in": "clip"},
		{"id": "block", "op": "box", "min": [-20,-20,0], "max": [20,20,20]},
		{"id": "bored", "op": "drill", "in": "block", "at": [0,0,20], "axis": [0,0,-1], "d": 16, "through": 20},
		{"id": "channel", "op": "circlip_groove_internal", "in": "bored", "at": [0,0,6], "axis": [0,0,1], "bore_d": 16},
		{"id": "v_channel", "op": "validate", "in": "channel"},
		{"id": "iclip", "op": "circlip_internal", "bore_d": 32},
		// Screws, pins, spacers, springs, keys.
		{"id": "fhs", "op": "flat_head_screw", "m": 5, "length": 16},
		{"id": "bhs", "op": "button_head_screw", "m": 5, "length": 16},
		{"id": "grub", "op": "set_screw", "m": 6, "length": 10},
		{"id": "nyloc", "op": "lock_nut", "m": 10},
		{"id": "stud", "op": "threaded_rod", "m": 8, "length": 60},
		{"id": "spacer", "op": "standoff", "m": 3, "length": 12},
		{"id": "pin", "op": "dowel_pin", "d": 6, "length": 24},
		{"id": "key", "op": "parallel_key", "b": 6, "h": 6, "l": 25},
		{"id": "spring", "op": "compression_spring", "wire_d": 2, "outer_d": 16, "pitch": 6, "turns": 5},
		// 3D-printing era: extrusion stock, tee nut, heat-set boss.
		{"id": "rail", "op": "extrusion_2020", "length": 100},
		{"id": "v_rail", "op": "volume", "in": "rail"},
		{"id": "rail30", "op": "extrusion_3030", "length": 50},
		{"id": "tnut", "op": "tnut_2020"},
		{"id": "lid", "op": "box", "min": [0,0,0], "max": [30,30,6]},
		{"id": "boss", "op": "heatset_insert_boss", "in": "lid", "at": [15,15,6], "axis": [0,0,1], "m": 3},
		// Gearing: basic rack + conjugate ring gear.
		{"id": "rack", "op": "gear_rack", "module": 2, "length": 100, "width": 10},
		{"id": "v_rack", "op": "volume", "in": "rack"},
		{"id": "ring", "op": "internal_gear", "module": 2, "teeth": 36, "face_width": 8, "rim_od": 84},
		{"id": "v_ring", "op": "validate", "in": "ring"},
		// Design math.
		{"id": "belt", "op": "gt2_belt", "center_distance": 100, "t1": 20, "t2": 20},
		{"id": "c", "op": "gt2_center_distance", "belt_teeth": 120, "t1": 20, "t2": 20},
		{"id": "fit", "op": "iso286_fit", "d": 8, "fit": "H7/g6"},
		{"id": "stl", "op": "export_stl", "in": "gland", "file": "piston.stl"}
	]});
	let report = run_program(&serde_json::to_string(&program).expect("serialize"), &dir);
	let genus = |id: &str| entry(&report, id).measures.as_ref().and_then(|m| m["genus"].as_i64());
	let num = |id: &str, key: &str| entry(&report, id).measures.as_ref().and_then(|m| m[key].as_f64()).unwrap_or(f64::NAN);
	let belt_teeth = entry(&report, "belt").measures.as_ref().and_then(|m| m["belt_teeth"].as_u64());
	let (rail_vol, rack_vol) = (num("v_rail", "volume"), num("v_rack", "volume"));
	let fit_clearance = entry(&report, "fit")
		.measures
		.as_ref()
		.and_then(|m| Some((m["clearance"][0].as_f64()?, m["clearance"][1].as_f64()?)))
		.unwrap_or((f64::NAN, f64::NAN));
	// Spot anchors: torus seal genus 1; external circlip genus 2 (two pliers
	// holes); bored + channelled block genus 1; ring gear genus 1; 2020 stock in
	// its published 160–195 mm²/mm metal-area band; the m2 ×100 rack closed form
	// 5892.98 mm³; the classic 240 mm / 120T GT2 loop with C = 100 back from the
	// inverse; Ø8 H7/g6 = +0.005…+0.029 mm clearance.
	assert!(
		report.ok
			&& genus("v_seal") == Some(1)
			&& genus("v_clip") == Some(2)
			&& genus("v_channel") == Some(1)
			&& genus("v_ring") == Some(1)
			&& rail_vol > 16000.0 && rail_vol < 19500.0
			&& (rack_vol - 5892.976).abs() < 0.01
			&& (num("belt", "pitch_length") - 240.0).abs() < 1e-9
			&& belt_teeth == Some(120)
			&& (num("c", "center_distance") - 100.0).abs() < 1e-9
			&& (fit_clearance.0 - 0.005).abs() < 1e-12
			&& (fit_clearance.1 - 0.029).abs() < 1e-12
			&& file_ok(&dir.join("piston.stl")),
		"wave-2 catalog program: seal_genus={:?} clip_genus={:?} channel_genus={:?} ring_genus={:?} rail_vol={rail_vol} rack_vol={rack_vol} belt=({}, {belt_teeth:?}) c={} fit={fit_clearance:?} report={report:#?}",
		genus("v_seal"),
		genus("v_clip"),
		genus("v_channel"),
		genus("v_ring"),
		num("belt", "pitch_length"),
		num("c", "center_distance")
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (11) Wave-3 catalog, sealing + the gear export route: the FRICTION #10 surface
/// (metric-cord face-seal glands, circular and racetrack, the free-ID cord ring,
/// and both design-math lookups) in one program, plus the FRICTION #6 regression —
/// the z=60 gearbox wheel that used to fall back to `voxel_healed` must now export
/// STL via the `exact` route with an analytic (π-exact) bore.
#[test]
fn wave3_face_seals_and_exact_gear_route() {
	let dir = out_dir("wave3_seals");
	let program = json!({"ops": [
		// FRICTION #6 regression: the gearbox wheel exports exact, bore analytic.
		{"id": "wheel", "op": "spur_gear", "module": 2, "teeth": 60, "face_width": 10, "bore": 12},
		{"id": "wheel_stl", "op": "export_stl", "in": "wheel", "file": "wheel.stl"},
		{"id": "wheel_vol", "op": "volume", "in": "wheel"},
		{"id": "wheel_xvol", "op": "exact_volume", "in": "wheel"},
		// FRICTION #10: racetrack lid gland, circular boss gland, free-ID cord ring.
		{"id": "lid", "op": "box", "min": [-60,-40,0], "max": [60,40,6]},
		{"id": "lid_gland", "op": "o_ring_face_gland_racetrack", "in": "lid", "at": [0,0,6], "axis": [0,0,1], "x_len": 100, "y_len": 60, "corner_r": 8, "cord_d": 2},
		{"id": "v_lid", "op": "validate", "in": "lid_gland"},
		{"id": "boss", "op": "cylinder", "base": [0,0,0], "axis": [0,0,1], "radius": 25, "height": 10, "segments": 48},
		{"id": "boss_gland", "op": "o_ring_face_gland", "in": "boss", "at": [0,0,10], "axis": [0,0,1], "gland_center_d": 36, "cord_d": 2},
		{"id": "v_boss", "op": "validate", "in": "boss_gland"},
		{"id": "ring", "op": "o_ring_cord", "ring_id": 150, "cord_d": 3},
		{"id": "v_ring", "op": "validate", "in": "ring"},
		{"id": "g", "op": "metric_cord_gland", "cord_d": 2},
		{"id": "cord", "op": "racetrack_cord_length", "x_len": 100, "y_len": 60, "corner_r": 8},
		// Shaft couplings: jaw hub + spider, stepped set-screw, slit clamp.
		{"id": "hub", "op": "jaw_coupling_hub", "od": 25, "bore": 8},
		{"id": "v_hub", "op": "validate", "in": "hub"},
		{"id": "spider", "op": "jaw_coupling_spider", "od": 25},
		{"id": "rigid", "op": "set_screw_coupling", "bore1": 5, "bore2": 8},
		{"id": "v_rigid", "op": "validate", "in": "rigid"},
		{"id": "clampc", "op": "clamp_coupling", "bore1": 8, "bore2": 10},
		{"id": "v_clamp", "op": "validate", "in": "clampc"},
		// Motor interfaces: NEMA body + plate, a mount cut, an SG90 pocket.
		{"id": "motor", "op": "nema_motor", "frame": 17, "body_len": 40},
		{"id": "plate", "op": "nema_mount_plate", "frame": 23, "thickness": 6, "margin": 0},
		{"id": "v_plate", "op": "validate", "in": "plate"},
		{"id": "bracket", "op": "box", "min": [-30,-30,0], "max": [30,30,6]},
		{"id": "mount", "op": "nema_mount_cut", "in": "bracket", "at": [0,0,6], "axis": [0,0,1], "frame": 17, "through": 6},
		{"id": "v_mount", "op": "validate", "in": "mount"},
		{"id": "panel", "op": "box", "min": [-40,-20,0], "max": [40,20,4]},
		{"id": "servo", "op": "servo_pocket", "in": "panel", "at": [0,0,4], "axis": [0,0,1], "model": "sg90", "through": 4},
		{"id": "v_servo", "op": "validate", "in": "servo"},
		// Tr8 lead-screw family: screw, flanged nut, the carriage nut trap.
		{"id": "screw", "op": "lead_screw_tr8", "length": 300, "lead": 8},
		{"id": "nut", "op": "lead_screw_nut_tr8"},
		{"id": "v_nut", "op": "validate", "in": "nut"},
		{"id": "carriage", "op": "box", "min": [-25,-25,0], "max": [25,25,10]},
		{"id": "trap", "op": "tr8_nut_trap", "in": "carriage", "at": [0,0,10], "axis": [0,0,1], "through": 10},
		{"id": "v_trap", "op": "validate", "in": "trap"},
		// Linear motion: LM bearing, SC8UU block, rod supports, MGN12 pair.
		{"id": "lm8uu", "op": "linear_bearing_lmuu", "bore": 8},
		{"id": "block", "op": "sc8uu_block"},
		{"id": "sk8", "op": "shaft_support_sk8"},
		{"id": "v_sk8", "op": "validate", "in": "sk8"},
		{"id": "shf8", "op": "shaft_support_shf8"},
		{"id": "rail", "op": "mgn12_rail", "length": 100},
		{"id": "v_rail", "op": "validate", "in": "rail"},
		{"id": "mgncar", "op": "mgn12_carriage"},
		{"id": "v_mgncar", "op": "validate", "in": "mgncar"},
		// Bearing bodies + the KP08 pillow block.
		{"id": "b608", "op": "deep_groove_bearing", "designation": "608"},
		{"id": "v_b608", "op": "validate", "in": "b608"},
		{"id": "f623", "op": "flanged_bearing", "designation": "F623"},
		{"id": "t51100", "op": "thrust_bearing", "designation": "51100"},
		{"id": "kp08", "op": "kp08_pillow_block"},
		{"id": "v_kp08", "op": "validate", "in": "kp08"},
		// Fluid: G-port boss, the thread lookup, a hose barb, a PC4-M6 port.
		{"id": "gboss", "op": "pipe_boss_g", "designation": "G1/4", "wall": 2.5, "length": 12},
		{"id": "v_gboss", "op": "validate", "in": "gboss"},
		{"id": "g14", "op": "pipe_thread_g", "designation": "G1/4"},
		{"id": "barb", "op": "hose_barb", "hose_id": 6, "barbs": 3},
		{"id": "v_barb", "op": "validate", "in": "barb"},
		{"id": "manifold", "op": "box", "min": [-15,-15,0], "max": [15,15,10]},
		{"id": "pc4", "op": "pc4_port", "in": "manifold", "at": [0,0,10], "axis": [0,0,1], "m": 6, "through": 10},
		{"id": "v_pc4", "op": "validate", "in": "pc4"},
		// Fastening: an ISO 7379 shoulder bolt and a DIN 127 B spring washer.
		{"id": "pivot", "op": "shoulder_bolt", "shoulder_d": 8, "shoulder_len": 20},
		{"id": "v_pivot", "op": "validate", "in": "pivot"},
		{"id": "lockw", "op": "spring_washer", "m": 5},
		{"id": "v_lockw", "op": "validate", "in": "lockw"},
		// Printing-native holes: teardrop (one tunnel), bridged cbore (sealed: genus 0).
		{"id": "wall", "op": "box", "min": [-10,-5,0], "max": [10,5,30]},
		{"id": "axle", "op": "teardrop_hole", "in": "wall", "at": [0,5,15], "axis": [0,-1,0], "up": [0,0,1], "d": 8, "through": 10},
		{"id": "v_axle", "op": "validate", "in": "axle"},
		{"id": "cbplate", "op": "box", "min": [-15,-15,0], "max": [15,15,10]},
		{"id": "bridged", "op": "bridged_counterbore", "in": "cbplate", "at": [0,0,10], "axis": [0,0,-1], "m": 5, "through": 10, "bridge": 0.3},
		{"id": "v_bridged", "op": "validate", "in": "bridged"},
		// A base panel carrying the Raspberry Pi mounting pattern (4 × M2.5).
		// (105 × 80, not 76: the y-mirror-symmetric panel is the pinned stitcher
		// degeneracy — see kernel-model parts::boards tests.)
		{"id": "base", "op": "box", "min": [0,0,0], "max": [105,80,4]},
		{"id": "pi", "op": "board_mount", "in": "base", "at": [10,10,0], "axis": [0,0,1], "board": "rpi"},
		{"id": "v_pi", "op": "validate", "in": "pi"}
	]});
	let report = run_program(&serde_json::to_string(&program).expect("serialize"), &dir);
	let genus = |id: &str| entry(&report, id).measures.as_ref().and_then(|m| m["genus"].as_i64());
	let num = |id: &str, key: &str| entry(&report, id).measures.as_ref().and_then(|m| m[key].as_f64()).unwrap_or(f64::NAN);
	let route = entry(&report, "wheel_stl").measures.as_ref().and_then(|m| m["route"].as_str().map(String::from));
	let (vol, xvol) = (num("wheel_vol", "volume"), num("wheel_xvol", "exact_volume"));
	// Anchors: exact STL route (no voxel heal) with xvol strictly below the faceted
	// volume by ~the 48-gon bore deficit (analytic cylinder bore, FRICTION #15);
	// genus 0 glanded lid/boss (a sunk channel adds no handle); genus 1 cord torus;
	// the Ø2 gland at the design point 1.5 deep × 2.7925 wide (25% squeeze, 75%
	// fill); racetrack cord length 2(100+60) − 64 + 16π = 306.2655, equal in the
	// gland echo and the standalone lookup.
	let expected_cord = 2.0 * (100.0_f64 + 60.0) - 64.0 + 16.0 * std::f64::consts::PI;
	assert!(
		report.ok
			&& route.as_deref() == Some("exact")
			&& xvol < vol
			&& vol - xvol > 2.0 && vol - xvol < 4.0
			&& genus("v_lid") == Some(0)
			&& genus("v_boss") == Some(0)
			&& genus("v_ring") == Some(1)
			&& genus("v_hub") == Some(1)
			&& genus("v_rigid") == Some(5)
			&& genus("v_clamp") == Some(4)
			&& genus("v_plate") == Some(5)
			&& genus("v_mount") == Some(5)
			&& genus("v_servo") == Some(3)
			&& genus("v_nut") == Some(5)
			&& genus("v_trap") == Some(5)
			&& genus("v_sk8") == Some(4)
			&& genus("v_rail") == Some(4)
			&& genus("v_mgncar") == Some(0)
			&& genus("v_b608") == Some(1)
			&& genus("v_kp08") == Some(3)
			&& genus("v_gboss") == Some(1)
			&& genus("v_barb") == Some(1)
			&& genus("v_pc4") == Some(1)
			&& genus("v_pivot") == Some(0)
			&& genus("v_lockw") == Some(0)
			&& genus("v_axle") == Some(1)
			&& genus("v_bridged") == Some(0)
			&& genus("v_pi") == Some(4)
			&& (num("g14", "tap_drill_d") - 11.8).abs() < 1e-12
			&& (num("g14", "pitch") - 25.4 / 19.0).abs() < 1e-12
			&& (num("g", "gland_depth") - 1.5).abs() < 1e-12
			&& (num("g", "groove_width") - 2.792526803190927).abs() < 1e-12
			&& (num("g", "squeeze") - 0.25).abs() < 1e-12
			&& (num("g", "fill") - 0.75).abs() < 1e-12
			&& (num("lid_gland", "cord_length") - expected_cord).abs() < 1e-9
			&& (num("cord", "cord_length") - expected_cord).abs() < 1e-9
			&& (num("boss_gland", "cord_length") - 36.0 * std::f64::consts::PI).abs() < 1e-9
			&& file_ok(&dir.join("wheel.stl")),
		"wave-3 sealing program: route={route:?} vol={vol} xvol={xvol} lid_genus={:?} boss_genus={:?} ring_genus={:?} gland=({}, {}, {}, {}) cord=({}, {}) report={report:#?}",
		genus("v_lid"),
		genus("v_boss"),
		genus("v_ring"),
		num("g", "gland_depth"),
		num("g", "groove_width"),
		num("g", "squeeze"),
		num("g", "fill"),
		num("lid_gland", "cord_length"),
		num("cord", "cord_length")
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (6) The CLI binary contract: report JSON on stdout, exit 0 iff all ops ok.
#[test]
fn cli_prints_report_and_exit_codes() {
	let dir = out_dir("cli");
	let ok_path = dir.join("ok.json");
	let bad_path = dir.join("bad.json");
	std::fs::write(&ok_path, r#"{"ops": [{"id": "b", "op": "box", "min": [0,0,0], "max": [5,5,5]}, {"id": "v", "op": "volume", "in": "b"}]}"#)
		.expect("write ok program");
	std::fs::write(&bad_path, r#"{"ops": [{"id": "u", "op": "union", "a": "nope", "b": "nope2"}]}"#).expect("write bad program");

	let run = |path: &Path| {
		let output = std::process::Command::new(env!("CARGO_BIN_EXE_kernel-api"))
			.args(["run", &path.display().to_string(), "--out-dir", &dir.display().to_string()])
			.output()
			.expect("spawn kernel-api");
		let report: Report = serde_json::from_slice(&output.stdout).expect("stdout must be a JSON report");
		(output.status.code(), report)
	};
	let (ok_code, ok_report) = run(&ok_path);
	let (bad_code, bad_report) = run(&bad_path);
	let volume = entry(&ok_report, "v").measures.as_ref().and_then(|m| m["volume"].as_f64()).unwrap_or(f64::NAN);
	assert!(
		ok_code == Some(0)
			&& ok_report.ok
			&& (volume - 125.0).abs() < 1e-9
			&& bad_code == Some(1)
			&& !bad_report.ok
			&& bad_report.ops[0].error.as_ref().map(|e| e.kind) == Some(ErrorKind::MissingRef),
		"CLI contract: ok=({ok_code:?}, {ok_report:#?}) bad=({bad_code:?}, {bad_report:#?})"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (7) The general rigid `pose` op (FRICTION #3): Rx(−90°) — the "gear lies
/// along Y" orientation that `rotate_z`+`translate` can never produce — plus a
/// translation, and a rotation about an off-origin center, both verified
/// through the posed center of mass.
#[test]
fn pose_op_applies_general_rigid_pose() {
	let dir = out_dir("pose");
	let program = json!({"ops": [
		{"id": "b", "op": "box", "min": [0.0, 0.0, 0.0], "max": [10.0, 20.0, 2.0]},
		{"id": "posed", "op": "pose", "in": "b",
			"rotate": {"axis": [1.0, 0.0, 0.0], "degrees": -90.0},
			"translate": [0.0, 0.0, 30.0]},
		{"id": "m1", "op": "mass_properties", "in": "posed"},
		{"id": "spun", "op": "pose", "in": "b",
			"rotate": {"axis": [0.0, 0.0, 1.0], "degrees": 180.0, "center": [5.0, 10.0, 0.0]}},
		{"id": "m2", "op": "mass_properties", "in": "spun"},
		{"id": "noop", "op": "pose", "in": "b"}
	]});
	let report = run_program(&serde_json::to_string(&program).expect("serialize"), &dir);
	let com = |id: &str| -> Vec<f64> {
		entry(&report, id).measures.as_ref().map(|m| m["center_of_mass"].as_array().expect("com array").iter().map(|x| x.as_f64().expect("number")).collect()).unwrap_or_default()
	};
	let (c1, c2) = (com("m1"), com("m2"));
	// Rx(−90°): (x, y, z) → (x, z, −y), so com (5, 10, 1) → (5, 1, −10); +[0,0,30] → (5, 1, 20).
	// 180° about Z through the box's own (x, y) centroid keeps the com in place.
	let near = |v: &[f64], want: [f64; 3]| v.len() == 3 && v.iter().zip(want).all(|(a, b)| (a - b).abs() < 1e-9);
	let noop = report.ops.iter().find(|o| o.id == "noop").expect("noop entry");
	assert!(
		!report.ok
			&& near(&c1, [5.0, 1.0, 20.0])
			&& near(&c2, [5.0, 10.0, 1.0])
			&& !noop.ok
			&& noop.error.as_ref().map(|e| e.kind) == Some(ErrorKind::InvalidParam),
		"pose: com1={c1:?} (want [5,1,20]) com2={c2:?} (want [5,10,1]); an empty pose must fail loudly: {report:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (8) The assertion family (FRICTION #4/#5): programs FAIL on unmet intent.
/// A passing run pins volume/genus/shells/validity and proves non-contact via
/// `assert_disjoint` (the exit-0 proof an empty `intersection` cannot give);
/// then a wrong `genus` expectation and a touching `assert_disjoint` each fail
/// with the structured `assert_failed` kind, stopping execution.
#[test]
fn assert_ops_enforce_intent() {
	let dir = out_dir("asserts");
	let pass = json!({"ops": [
		{"id": "a", "op": "box", "min": [0.0, 0.0, 0.0], "max": [10.0, 20.0, 2.0]},
		{"id": "b", "op": "box", "min": [13.0, 0.0, 0.0], "max": [20.0, 20.0, 2.0]},
		{"id": "ok_solid", "op": "assert", "in": "a",
			"volume_within": {"target": 400.0, "percent": 0.1},
			"genus": 0, "shells": 1, "closed": true, "manifold": true, "valid": true},
		{"id": "ok_gap", "op": "assert_disjoint", "a": "a", "b": "b", "min_clearance": 2.5},
		{"id": "u", "op": "union", "a": "a", "b": "b"},
		{"id": "ok_two_shells", "op": "assert", "in": "u", "shells": 2,
			"volume_within": {"target": 680.0, "abs": 0.5}}
	]});
	let pass_report = run_program(&serde_json::to_string(&pass).expect("serialize"), &dir);
	let gap = entry(&pass_report, "ok_gap").measures.as_ref().and_then(|m| m["distance"].as_f64()).unwrap_or(f64::NAN);

	let fail_genus = run_program(
		r#"{"ops": [
			{"id": "a", "op": "box", "min": [0,0,0], "max": [10,20,2]},
			{"id": "bad", "op": "assert", "in": "a", "genus": 1}
		]}"#,
		&dir,
	);
	let fail_touch = run_program(
		r#"{"ops": [
			{"id": "a", "op": "box", "min": [0,0,0], "max": [10,20,2]},
			{"id": "b", "op": "box", "min": [5,5,0], "max": [20,20,2]},
			{"id": "bad", "op": "assert_disjoint", "a": "a", "b": "b"}
		]}"#,
		&dir,
	);
	let kind_of = |r: &Report| r.ops.last().and_then(|o| o.error.as_ref()).map(|e| e.kind);
	let msg_of = |r: &Report| r.ops.last().and_then(|o| o.error.as_ref()).map(|e| e.message.clone()).unwrap_or_default();
	assert!(
		pass_report.ok
			&& (gap - 3.0).abs() < 1e-9
			&& !fail_genus.ok
			&& kind_of(&fail_genus) == Some(ErrorKind::AssertFailed)
			&& msg_of(&fail_genus).contains("genus: measured 0, expected 1")
			&& !fail_touch.ok
			&& kind_of(&fail_touch) == Some(ErrorKind::AssertFailed)
			&& msg_of(&fail_touch).contains("touch or interfere"),
		"asserts: pass={pass_report:#?} gap={gap} fail_genus={fail_genus:#?} fail_touch={fail_touch:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (9) FRICTION #7: `bore_d` — the `.lmcpart` Document field name — is a serde
/// alias for `bore` on the catalog gear ops, so a part translated between the
/// two grammars keeps its meaning (same gear, bit-identical volume).
#[test]
fn catalog_gear_bore_d_alias() {
	let dir = out_dir("bore_alias");
	let report = run_program(
		r#"{"ops": [
			{"id": "canonical", "op": "spur_gear", "module": 1.0, "teeth": 16, "face_width": 5.0, "bore": 6.0},
			{"id": "aliased", "op": "spur_gear", "module": 1.0, "teeth": 16, "face_width": 5.0, "bore_d": 6.0},
			{"id": "v1", "op": "volume", "in": "canonical"},
			{"id": "v2", "op": "volume", "in": "aliased"}
		]}"#,
		&dir,
	);
	let vol = |id: &str| entry(&report, id).measures.as_ref().and_then(|m| m["volume"].as_f64()).unwrap_or(f64::NAN);
	assert!(
		report.ok && vol("v1").is_finite() && vol("v1") == vol("v2"),
		"bore_d must alias bore on spur_gear: v1={} v2={} report={report:#?}",
		vol("v1"),
		vol("v2")
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (10) FRICTION #8: `bolt_circle` and `bearing_seat` as first-class data ops —
/// a 4×M4 counterbore ring (genus 4, the repeated cut's DIN 974-1 row echoed in
/// the measures) and a 608 bearing seat (genus 1, Ø22 × 7 pocket + Ø15
/// shoulder echoed).
#[test]
fn bolt_circle_and_bearing_seat_ops() {
	let dir = out_dir("boltcircle");
	let program = json!({"ops": [
		{"id": "plate", "op": "box", "min": [0.0, 0.0, 0.0], "max": [60.0, 60.0, 8.0]},
		{"id": "ring", "op": "bolt_circle", "in": "plate", "center": [30.0, 30.0, 8.0],
			"axis": [0.0, 0.0, -1.0], "circle_d": 40.0, "n": 4, "start_deg": 45.0,
			"hole": {"kind": "counterbore", "m": 4.0}},
		{"id": "ring_genus", "op": "assert", "in": "ring", "genus": 4},
		{"id": "block", "op": "box", "min": [0.0, 0.0, 0.0], "max": [40.0, 40.0, 20.0]},
		{"id": "seat", "op": "bearing_seat", "in": "block", "at": [20.0, 20.0, 20.0],
			"axis": [0.0, 0.0, -1.0], "bearing": "608"},
		{"id": "seat_genus", "op": "assert", "in": "seat", "genus": 1}
	]});
	let report = run_program(&serde_json::to_string(&program).expect("serialize"), &dir);
	let ring = entry(&report, "ring").measures.clone().unwrap_or_default();
	let seat = entry(&report, "seat").measures.clone().unwrap_or_default();
	assert!(
		report.ok
			&& ring["n"] == 4
			&& ring["hole"]["counterbore_d"] == 8.0
			&& ring["hole"]["counterbore_depth"] == 4.8
			&& ring["hole"]["clearance_d"] == 4.5
			&& seat["pocket_d"] == 22.0
			&& seat["pocket_depth"] == 7.0
			&& seat["shoulder_d"] == 15.0,
		"bolt_circle/bearing_seat: ring={ring} seat={seat} report={report:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (11) FRICTION #9: every standalone hole-wizard cut echoes the ISO/DIN table
/// row it actually used as measures (the M4 counterbore Ø8.0 × 4.8 that had to
/// be read out of kernel source during the dogfood), including the 118°
/// drill-point reach of blind holes.
#[test]
fn hole_wizard_ops_echo_table_rows() {
	let dir = out_dir("holerows");
	let program = json!({"ops": [
		{"id": "block", "op": "box", "min": [0.0, 0.0, 0.0], "max": [80.0, 40.0, 12.0]},
		{"id": "d1", "op": "drill", "in": "block", "at": [10.0, 20.0, 12.0], "axis": [0.0, 0.0, -1.0], "d": 6.0, "depth": 8.0},
		{"id": "h1", "op": "clearance_hole", "in": "d1", "at": [25.0, 20.0, 12.0], "axis": [0.0, 0.0, -1.0], "m": 5.0, "fit": "coarse"},
		{"id": "h2", "op": "counterbore_hole", "in": "h1", "at": [40.0, 20.0, 12.0], "axis": [0.0, 0.0, -1.0], "m": 4.0},
		{"id": "h3", "op": "countersink_hole", "in": "h2", "at": [55.0, 20.0, 12.0], "axis": [0.0, 0.0, -1.0], "m": 4.0},
		{"id": "h4", "op": "tap_drill_hole", "in": "h3", "at": [70.0, 20.0, 12.0], "axis": [0.0, 0.0, -1.0], "m": 5.0, "depth": 9.0}
	]});
	let report = run_program(&serde_json::to_string(&program).expect("serialize"), &dir);
	let m = |id: &str| entry(&report, id).measures.clone().unwrap_or_default();
	let (d1, h1, h2, h3, h4) = (m("d1"), m("h1"), m("h2"), m("h3"), m("h4"));
	// 118° drill point: tip height = (d/2)/tan 59° → Ø6 ≈ 1.8026, Ø4.2 ≈ 1.2618.
	let point = |meas: &serde_json::Value| meas["point_depth"].as_f64().unwrap_or(f64::NAN);
	assert!(
		report.ok
			&& d1["kind"] == "blind"
			&& d1["depth"] == 8.0
			&& (point(&d1) - (8.0 + 1.8026)).abs() < 1e-3
			&& h1["clearance_d"] == 5.8
			&& h1["fit"] == "coarse"
			&& h2["clearance_d"] == 4.5
			&& h2["counterbore_d"] == 8.0
			&& h2["counterbore_depth"] == 4.8
			&& h3["countersink_d"] == 10.0
			&& h4["pitch"] == 0.8
			&& h4["pilot_d"] == 4.2
			&& (point(&h4) - (9.0 + 1.2618)).abs() < 1e-3,
		"hole-wizard table echoes: d1={d1} h1={h1} h2={h2} h3={h3} h4={h4} report={report:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (12) FRICTION #16/#11/#17 minors: `union_all` folds n solids in one op (and
/// rejects a single-element list loudly), `heatset_spec` exposes the Ruthex
/// pilot/pocket table without growing a boss, and `wall_thickness` reports
/// percentile signals alongside the corner-noisy minimum.
#[test]
fn union_all_heatset_spec_and_wall_percentiles() {
	let dir = out_dir("minors");
	let program = json!({"ops": [
		{"id": "plate", "op": "box", "min": [0.0, 0.0, 0.0], "max": [60.0, 60.0, 8.0]},
		{"id": "wt", "op": "wall_thickness", "in": "plate", "flag_below": 1.0},
		{"id": "hs", "op": "heatset_spec", "m": 4.0},
		{"id": "a", "op": "box", "min": [0.0, 0.0, 0.0], "max": [5.0, 5.0, 5.0]},
		{"id": "b", "op": "box", "min": [10.0, 0.0, 0.0], "max": [15.0, 5.0, 5.0]},
		{"id": "c", "op": "box", "min": [20.0, 0.0, 0.0], "max": [25.0, 5.0, 5.0]},
		{"id": "all", "op": "union_all", "in": ["a", "b", "c"]},
		{"id": "three_bodies", "op": "assert", "in": "all", "shells": 3,
			"volume_within": {"target": 375.0, "abs": 1e-9}}
	]});
	let report = run_program(&serde_json::to_string(&program).expect("serialize"), &dir);
	let wt = entry(&report, "wt").measures.clone().unwrap_or_default();
	let hs = entry(&report, "hs").measures.clone().unwrap_or_default();
	let singleton = run_program(
		r#"{"ops": [
			{"id": "a", "op": "box", "min": [0,0,0], "max": [5,5,5]},
			{"id": "one", "op": "union_all", "in": ["a"]}
		]}"#,
		&dir,
	);
	assert!(
		report.ok
			&& (wt["p05_thickness"].as_f64().unwrap_or(f64::NAN) - 8.0).abs() < 1e-6
			&& wt["median_thickness"].as_f64().unwrap_or(f64::NAN) >= 8.0
			&& hs["pilot_d"] == 5.6
			&& hs["pocket_depth"] == 9.1
			&& !singleton.ok
			&& singleton.ops.last().and_then(|o| o.error.as_ref()).map(|e| e.kind) == Some(ErrorKind::InvalidParam),
		"union_all / heatset_spec / wall percentiles: wt={wt} hs={hs} singleton={singleton:#?} report={report:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (13) FRICTION #13: through the CLI, a program's relative `load_part` paths
/// resolve against the PROGRAM FILE's directory (like `.lmcasm` `path`
/// sources), so programs are relocatable; outputs still land under --out-dir.
#[test]
fn cli_load_part_resolves_against_program_dir() {
	let dir = out_dir("progdir");
	let program_dir = dir.join("programs");
	let out = dir.join("exports");
	std::fs::create_dir_all(&program_dir).expect("mkdir programs");
	let mut doc = kernel_model::Document::new();
	let b = doc.add(kernel_model::Feature::Box {
		center: [kernel_model::Dim::Literal(0.0), kernel_model::Dim::Literal(0.0), kernel_model::Dim::Literal(0.0)],
		size: [kernel_model::Dim::Literal(10.0), kernel_model::Dim::Literal(10.0), kernel_model::Dim::Literal(10.0)],
	});
	doc.set_root(b);
	// The part lives NEXT TO the program, not in the out dir.
	std::fs::write(program_dir.join("cube.lmcpart"), kernel_model::format::save_part(&doc, "cube")).expect("write part");
	std::fs::write(
		program_dir.join("p.json"),
		r#"{"ops": [
			{"id": "cube", "op": "load_part", "file": "cube.lmcpart"},
			{"id": "stl", "op": "export_stl", "in": "cube", "file": "cube.stl"}
		]}"#,
	)
	.expect("write program");
	let output = std::process::Command::new(env!("CARGO_BIN_EXE_kernel-api"))
		.args(["run", &program_dir.join("p.json").display().to_string(), "--out-dir", &out.display().to_string()])
		.output()
		.expect("spawn kernel-api run");
	let report: Report = serde_json::from_slice(&output.stdout).expect("stdout must be a JSON report");
	assert!(
		output.status.code() == Some(0) && report.ok && file_ok(&out.join("cube.stl")),
		"load_part must resolve against the program's directory and exports against --out-dir: exit={:?} report={report:#?}",
		output.status.code()
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// Loft op through the JSON binding: a square frustum (40×40 → 20×20 over h=30).
/// The faceted vertex-morph cross-section is quadratic in the loft parameter, so
/// the prismatoid (Simpson) volume h/6·(A0+4·A_mid+A1) = 30/6·(1600+4·900+400) =
/// 28000 mm³ is EXACT, and the solid must be a watertight genus-0 body.
#[test]
fn loft_op_builds_square_frustum_with_exact_volume() {
	let dir = out_dir("loft");
	let program = json!({"ops": [
		{"id": "f", "op": "loft", "sections": [
			[[-20.0, -20.0, 0.0], [20.0, -20.0, 0.0], [20.0, 20.0, 0.0], [-20.0, 20.0, 0.0]],
			[[-10.0, -10.0, 30.0], [10.0, -10.0, 30.0], [10.0, 10.0, 30.0], [-10.0, 10.0, 30.0]]
		]},
		{"id": "check", "op": "validate", "in": "f"},
		{"id": "vol", "op": "volume", "in": "f"},
		{"id": "stl", "op": "export_stl", "in": "f", "file": "frustum.stl", "tol": 0.01},
	]});
	let report = run_program(&serde_json::to_string(&program).expect("serialize"), &dir);
	let genus = entry(&report, "check").measures.as_ref().and_then(|m| m["genus"].as_i64());
	let volume = entry(&report, "vol").measures.as_ref().and_then(|m| m["volume"].as_f64()).unwrap_or(f64::NAN);
	assert!(
		report.ok && genus == Some(0) && (volume - 28000.0).abs() < 1e-6 && file_ok(&dir.join("frustum.stl")),
		"loft op: ok={} genus={genus:?} volume={volume} (want 28000 exactly) report={report:#?}",
		report.ok
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// Sweep op through the JSON binding: an 8×8 square section swept along an L-path
/// (up +Z then over +X) — a bent square tube no extrude/revolve can make. Must be
/// a watertight genus-0 solid with positive volume on the order of section×length.
#[test]
fn sweep_op_builds_a_bent_square_tube() {
	let dir = out_dir("sweep");
	let program = json!({"ops": [
		{"id": "p", "op": "sweep",
			"profile": [[-4.0, -4.0, 0.0], [4.0, -4.0, 0.0], [4.0, 4.0, 0.0], [-4.0, 4.0, 0.0]],
			"path": [[0.0, 0.0, 0.0], [0.0, 0.0, 25.0], [20.0, 0.0, 25.0]]},
		{"id": "check", "op": "validate", "in": "p"},
		{"id": "vol", "op": "volume", "in": "p"},
	]});
	let report = run_program(&serde_json::to_string(&program).expect("serialize"), &dir);
	let genus = entry(&report, "check").measures.as_ref().and_then(|m| m["genus"].as_i64());
	let volume = entry(&report, "vol").measures.as_ref().and_then(|m| m["volume"].as_f64()).unwrap_or(f64::NAN);
	// Section area 64, path length ~45 -> ~2880 mm^3, less the mitre overlap; bound loosely.
	assert!(
		report.ok && genus == Some(0) && volume > 1500.0 && volume < 3200.0,
		"sweep op: ok={} genus={genus:?} volume={volume} (want a valid genus-0 bent tube ~2000-3000) report={report:#?}",
		report.ok
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// bounding_box measure op through the JSON binding: a 40×20×10 extruded block
/// must report exact size / diagonal, and the envelope-fit flags must match the
/// as-is-fails / rotated-succeeds logic (25×45×15 only fits after a 90° turn).
#[test]
fn bounding_box_op_reports_dimensions_and_envelope_fit() {
	let dir = out_dir("bbox");
	let program = json!({"ops": [
		{"id": "b", "op": "extrude", "profile": [[0.0,0.0],[40.0,0.0],[40.0,20.0],[0.0,20.0]], "height": 10.0},
		{"id": "bb", "op": "bounding_box", "in": "b", "envelope": [25.0, 45.0, 15.0]},
	]});
	let report = run_program(&serde_json::to_string(&program).expect("serialize"), &dir);
	let m = entry(&report, "bb").measures.as_ref().cloned().unwrap_or_default();
	let size: Vec<f64> = m["size"].as_array().map(|a| a.iter().map(|v| v.as_f64().unwrap_or(f64::NAN)).collect()).unwrap_or_default();
	let diag = m["diagonal"].as_f64().unwrap_or(f64::NAN);
	let want_diag = (40.0f64.powi(2) + 20.0f64.powi(2) + 10.0f64.powi(2)).sqrt();
	assert!(
		report.ok
			&& size == vec![40.0, 20.0, 10.0]
			&& (diag - want_diag).abs() < 1e-9
			&& m["fits_within"].as_bool() == Some(false)
			&& m["fits_within_rotated"].as_bool() == Some(true),
		"bounding_box op: size={size:?} diag={diag} (want {want_diag}) fits={:?} fits_rot={:?} report={report:#?}",
		m["fits_within"].as_bool(),
		m["fits_within_rotated"].as_bool()
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// A witness-selecting op (`fillet_edge_near`) now leaves a `resolved_edge`
/// receipt: WHICH edge the spatial witness actually latched (its canonical
/// face-pair EdgeName, serialized like kernel-model's `edge_name_serde`), plus
/// the witness→edge distance and the max-distance limit in effect. This is the
/// evidence a parameter sweep needs to catch a witness silently jumping to a
/// different edge — so the identity must be present, well-formed, and STABLE
/// across identical reruns (i.e. comparable across candidates). Selection and
/// rounding are unchanged; this only records the choice.
#[test]
fn fillet_edge_near_records_which_edge_the_witness_chose() {
	let dir = out_dir("resolved_edge");
	let program = json!({"ops": [
		{"id": "b", "op": "box", "min": [0.0, 0.0, 0.0], "max": [40.0, 20.0, 20.0]},
		{"id": "f", "op": "fillet_edge_near", "in": "b", "witness": [38.0, 0.0, 9.0], "radius": 1.5},
	]});
	let src = serde_json::to_string(&program).expect("serialize");
	let report = run_program(&src, &dir);
	let rerun = run_program(&src, &dir); // determinism → comparable across candidates
	let re = |r: &Report| entry(r, "f").measures.as_ref().and_then(|m| m.get("resolved_edge").cloned());
	let m = re(&report).unwrap_or_default();
	let faces = m["faces"].as_array().cloned().unwrap_or_default();
	let well_formed = faces.len() == 2
		&& faces.iter().all(|f| f["operand"].is_string() && f["source_face"].is_u64());
	let dist = m["witness_distance"].as_f64().unwrap_or(f64::NAN);
	let limit = m["max_distance"].as_f64().unwrap_or(f64::NAN);
	assert!(
		report.ok
			&& well_formed
			&& dist.is_finite()
			&& limit.is_finite()
			&& re(&report) == re(&rerun), // same face-pair identity on rerun → comparable
		"fillet_edge_near resolved_edge: faces={faces:?} witness_distance={dist} max_distance={limit} \
		 well_formed={well_formed} stable_on_rerun={} report={report:#?}",
		re(&report) == re(&rerun)
	);
	let _ = std::fs::remove_dir_all(&dir);
}
