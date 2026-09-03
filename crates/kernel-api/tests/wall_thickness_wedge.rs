// Copyright (c) LMCAD. Licensed under the MIT License.

//! `wall_thickness` on acute wedges (friction l12_mini_case F4, uphill_roller
//! F3, 2026-09): the knife-edge lip of a female dovetail groove is genuinely
//! thin next to its edge, but it is edge geometry, not a wall — with
//! `exclude_wedge_deg` those readings land in `thin_area_wedge`, the receipt
//! locates every flagged patch (`thin_witness`), and the area-uniform sampler
//! reads mirror-image bodies alike instead of 5× apart.

use std::path::{Path, PathBuf};

use kernel_api::{run_program, ErrorKind, Report};
use serde_json::{json, Value};

fn out_dir(name: &str) -> PathBuf {
	let dir = std::env::temp_dir().join(format!("kernel_api_{name}_{}", std::process::id()));
	std::fs::create_dir_all(&dir).expect("create test out dir");
	dir
}

fn run(dir: &Path, ops: Value) -> Report {
	run_program(&serde_json::to_string(&json!({ "ops": ops })).expect("serialize"), dir)
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

fn f(v: &Value, key: &str) -> f64 {
	v[key].as_f64().unwrap_or_else(|| panic!("measure '{key}' missing or not a number in {v}"))
}

/// The F4 body: a 4.5 mm floor with a female dovetail groove cut through it
/// (neck 4, tip 6, depth 2.5; the groove runs along Z), built with the exact
/// boolean so its top face carries a boolean triangulation. The lip between
/// the top face and each undercut wall is a 68° material wedge.
fn dovetail_block_ops() -> Vec<Value> {
	vec![
		json!({"id": "blk", "op": "extrude", "profile": [[0,0],[40,0],[40,4.5],[0,4.5]], "height": 30}),
		json!({"id": "cutter0", "op": "extrude", "profile": [[17,2.0],[23,2.0],[21.8,5.0],[18.2,5.0]], "height": 32}),
		json!({"id": "cutter", "op": "translate", "in": "cutter0", "offset": [0,0,-1]}),
		json!({"id": "groove", "op": "difference", "a": "blk", "b": "cutter"}),
	]
}

/// With `exclude_wedge_deg: 75` the dovetail block reports `thin_area: 0` and
/// a positive `thin_area_wedge`; without it the same area is plain `thin_area`
/// and the wedge fields are absent. The witnesses point at the lips.
#[test]
fn dovetail_lip_reads_as_wedge_area_only_when_excluded() {
	let dir = out_dir("wedge_dovetail");
	let mut ops = dovetail_block_ops();
	ops.push(json!({"id": "plain", "op": "wall_thickness", "in": "groove", "flag_below": 1.6}));
	ops.push(json!({"id": "wedge", "op": "wall_thickness", "in": "groove", "flag_below": 1.6, "exclude_wedge_deg": 75}));
	let report = run(&dir, Value::Array(ops));
	assert!(report.ok, "program must run: {report:#?}");
	let (plain, wedge) = (measures(&report, "plain"), measures(&report, "wedge"));

	// The exclusion moves the lip bands (analytically 2 lips × (0.64 mm of top
	// face + 0.64 mm of wall) × 30 mm ≈ 77 mm²) out of thin_area, untouched.
	assert_eq!(f(&wedge, "thin_area"), 0.0, "lip readings must not count as wall area: {wedge}");
	let wedge_area = f(&wedge, "thin_area_wedge");
	assert!((60.0..95.0).contains(&wedge_area), "thin_area_wedge {wedge_area} must be the two lip bands (~77 mm²): {wedge}");
	assert!((f(&wedge, "thin_area_total") - wedge_area).abs() < 1e-9, "thin_area_total = thin_area + thin_area_wedge: {wedge}");
	assert!((f(&plain, "thin_area") - wedge_area).abs() < 1e-9, "without the exclusion the same area is thin_area: {plain}");
	assert!(
		plain.get("thin_area_wedge").is_none() && plain.get("thin_area_total").is_none() && plain.get("thin_wedge_witness").is_none(),
		"no wedge fields without exclude_wedge_deg: {plain}"
	);
	assert_eq!(wedge["exclude_wedge_deg"], json!(75.0));

	// The floor under the groove is 2.0 mm: with the lips set aside, that is
	// the thinnest counted wall. Without the exclusion the minimum is the lip.
	assert!((f(&wedge, "min_thickness") - 2.0).abs() < 1e-3, "min_thickness over the counted samples: {wedge}");
	assert!(f(&plain, "min_thickness") < 0.5, "the plain census still sees the knife edge: {plain}");
	assert!((f(&wedge, "p05_thickness") - 2.0).abs() < 1e-3 && (f(&wedge, "median_thickness") - 4.5).abs() < 1e-3, "{wedge}");

	// Witnesses: nothing counted → empty; the wedge witnesses sit on the lips
	// (x ≈ 18 / 22, high on the floor) and read below the flag.
	assert_eq!(wedge["thin_witness"], json!([]), "{wedge}");
	let lips = wedge["thin_wedge_witness"].as_array().cloned().unwrap_or_default();
	assert_eq!(lips.len(), 8, "up to 8 wedge witnesses: {wedge}");
	for w in &lips {
		let at = w["at"].as_array().expect("witness at");
		let (x, y) = (at[0].as_f64().unwrap(), at[1].as_f64().unwrap());
		let t = f(w, "thickness");
		assert!(t < 1.6 && ((x - 18.0).abs() < 1.2 || (x - 22.0).abs() < 1.2) && y > 3.5, "wedge witness must sit on a lip: {w}");
	}
	let plain_witness = plain["thin_witness"].as_array().cloned().unwrap_or_default();
	assert_eq!(plain_witness.len(), 8, "the plain census locates its thin patch: {plain}");
	assert!(plain_witness.windows(2).all(|p| f(&p[0], "thickness") <= f(&p[1], "thickness")), "thinnest first: {plain}");
	assert!(f(&plain, "samples") > f(&plain, "sampled_triangles") * 100.0, "area-uniform sampling, not one ray per triangle: {plain}");
	let _ = std::fs::remove_dir_all(&dir);
}

/// The same block mirrored in X reads the same numbers to within 5 % — both
/// the plain census and the wedge split. (The centroid-per-triangle sampler
/// read mirror-image grooves 19.6 vs 101 mm².)
#[test]
fn mirror_image_block_reads_the_same_thin_area() {
	let dir = out_dir("wedge_mirror");
	let mut ops = dovetail_block_ops();
	ops.push(json!({"id": "mir", "op": "mirror", "in": "groove", "plane": {"point": [20,0,0], "normal": [1,0,0]}}));
	for (id, body) in [("a_plain", "groove"), ("m_plain", "mir")] {
		ops.push(json!({"id": id, "op": "wall_thickness", "in": body, "flag_below": 1.6}));
	}
	for (id, body) in [("a_wedge", "groove"), ("m_wedge", "mir")] {
		ops.push(json!({"id": id, "op": "wall_thickness", "in": body, "flag_below": 1.6, "exclude_wedge_deg": 75}));
	}
	let report = run(&dir, Value::Array(ops));
	assert!(report.ok, "program must run: {report:#?}");
	let within_5pct = |a: f64, b: f64| (a - b).abs() <= 0.05 * a.max(b);
	let (ap, mp) = (measures(&report, "a_plain"), measures(&report, "m_plain"));
	assert!(
		within_5pct(f(&ap, "thin_area"), f(&mp, "thin_area")),
		"mirror images must agree on thin_area within 5 %: {} vs {}",
		f(&ap, "thin_area"),
		f(&mp, "thin_area")
	);
	let (aw, mw) = (measures(&report, "a_wedge"), measures(&report, "m_wedge"));
	assert_eq!(f(&aw, "thin_area"), 0.0, "{aw}");
	assert_eq!(f(&mw, "thin_area"), 0.0, "{mw}");
	assert!(
		f(&aw, "thin_area_wedge") > 0.0 && within_5pct(f(&aw, "thin_area_wedge"), f(&mw, "thin_area_wedge")),
		"mirror images must agree on thin_area_wedge within 5 %: {} vs {}",
		f(&aw, "thin_area_wedge"),
		f(&mw, "thin_area_wedge")
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// Parallel faces never share an edge, so a real thin wall is never a wedge:
/// a 0.5 mm plate keeps its full 800 mm² under `thin_area` with the exclusion
/// on. And the exclusion's angle is validated.
#[test]
fn a_thin_plate_is_never_a_wedge_and_the_angle_is_validated() {
	let dir = out_dir("wedge_plate");
	let report = run(
		&dir,
		json!([
			{"id": "plate", "op": "box", "min": [-10,-10,0], "max": [10,10,0.5]},
			{"id": "wt", "op": "wall_thickness", "in": "plate", "flag_below": 1.0, "exclude_wedge_deg": 75},
		]),
	);
	assert!(report.ok, "{report:#?}");
	let wt = measures(&report, "wt");
	assert!((f(&wt, "thin_area") - 800.0).abs() < 1e-3 && f(&wt, "thin_area_wedge") == 0.0, "plate: {wt}");
	assert!((f(&wt, "min_thickness") - 0.5).abs() < 1e-4, "{wt}");

	for bad in [0.0, -10.0, 181.0] {
		let r = run(
			&dir,
			json!([
				{"id": "plate", "op": "box", "min": [-10,-10,0], "max": [10,10,0.5]},
				{"id": "wt", "op": "wall_thickness", "in": "plate", "flag_below": 1.0, "exclude_wedge_deg": bad},
			]),
		);
		let kind = r.ops.iter().find(|o| o.id == "wt").and_then(|o| o.error.as_ref()).map(|e| e.kind);
		assert!(!r.ok && kind == Some(ErrorKind::InvalidParam), "exclude_wedge_deg {bad} must be invalid_param: {r:#?}");
	}
	let _ = std::fs::remove_dir_all(&dir);
}
