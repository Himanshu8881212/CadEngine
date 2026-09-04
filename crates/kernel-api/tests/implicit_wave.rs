// Copyright (c) LMCAD. Licensed under the MIT License.

//! The 2026-07-29 implicit wave on the JSON op surface (Mission D): strut
//! lattices, `pipe_path`, Hershey text, `displace` textures, `{"grid": …}`
//! NPY grade sources, the voxel-route solid ops (`offset_solid` /
//! `shell_solid` / `solid_from_implicit`) and the interrogation probes
//! (`thin_wall` / `min_ligament`) — every op exercised end-to-end through
//! `run_program` with a measured pin, and every refusal provoked for real
//! (machine-matchable `ErrorKind` + message needles). No direct Rust geometry
//! calls: the kernel math is proven in kernel-implicit / kernel-model /
//! kernel-brep's own suites; THIS file pins that pure JSON reaches it.

use std::path::PathBuf;

use kernel_api::{run_program, ErrorKind, OpReport, Report};
use serde_json::{json, Value};

/// A unique per-test output directory under the system temp dir.
fn out_dir(name: &str) -> PathBuf {
	let dir = std::env::temp_dir().join(format!("kernel_api_wave_{name}_{}", std::process::id()));
	std::fs::create_dir_all(&dir).expect("create test out dir");
	dir
}

/// Run a `{"ops": [...]}` program against `dir`.
fn run(dir: &std::path::Path, ops: Value) -> Report {
	run_program(&serde_json::to_string(&json!({ "ops": ops })).unwrap(), dir)
}

/// The report entry for op `id` (panics with the report when absent).
fn entry<'r>(report: &'r Report, id: &str) -> &'r OpReport {
	report.ops.iter().find(|o| o.id == id).unwrap_or_else(|| panic!("no report entry for op '{id}' in {report:#?}"))
}

/// A named measure of op `id` as f64 (NaN when absent).
fn num(report: &Report, id: &str, key: &str) -> f64 {
	entry(report, id).measures.as_ref().and_then(|m| m[key].as_f64()).unwrap_or(f64::NAN)
}

/// A named measure of op `id`.
fn measure<'r>(report: &'r Report, id: &str, key: &str) -> &'r Value {
	entry(report, id).measures.as_ref().map(|m| &m[key]).unwrap_or_else(|| panic!("op '{id}' has no measures in {report:#?}"))
}

/// Assert the FIRST failing op is `id` with `kind`, and its message carries
/// every needle (the machine-matchable refusal contract).
fn assert_refusal(report: &Report, id: &str, kind: ErrorKind, needles: &[&str]) {
	assert!(!report.ok, "program must fail on op '{id}', got ok report: {report:#?}");
	let e = entry(report, id).error.as_ref().unwrap_or_else(|| panic!("op '{id}' has no error in {report:#?}"));
	assert_eq!(e.kind, kind, "op '{id}' must fail with {kind:?}, got {:?}: {}", e.kind, e.message);
	for n in needles {
		assert!(e.message.contains(n), "op '{id}' error message must contain {n:?}, got: {}", e.message);
	}
}

// --- A) grammar additions ---------------------------------------------------------

/// `strut_lattice` leaf (green): the octet truss clipped by its own box meshes
/// watertight at 379.2 mm³ = 37.9% solid (executed pin for cell 10, r 1,
/// voxel 0.25) — the clipped-and-meshed sibling of the kernel's 39.3%
/// field-fraction pin (DESIGN_GUIDE §25.1 / kernel-implicit/tests/strut.rs;
/// the box clip shaves the boundary strut bulges).
#[test]
fn strut_lattice_leaf_meshes_watertight_at_the_pinned_solid_fraction() {
	let dir = out_dir("strut_leaf");
	let r = run(
		&dir,
		json!([{"id": "lat", "op": "implicit", "voxel": 0.25, "mesher": "manifold",
			"expr": {"op": "intersection",
				"a": {"shape": "strut_lattice", "kind": "octet", "cell": 10.0, "radius": 1.0,
				      "min": [0, 0, 0], "max": [10, 10, 10]},
				"b": {"shape": "box", "min": [0, 0, 0], "max": [10, 10, 10]}}}]),
	);
	assert!(r.ok, "octet strut_lattice ∩ box must mesh green: {r:#?}");
	let vol = num(&r, "lat", "volume");
	let fraction = vol / 1000.0;
	assert!(
		entry(&r, "lat").measures.as_ref().unwrap()["watertight"] == json!(true) && (fraction - 0.379).abs() < 0.01,
		"one clipped octet cell (cell 10, r 1) must be watertight at ≈37.9% solid (executed pin 379.2 mm³; kernel field-fraction pin 39.3% before the box clip); measured volume {vol:.1} mm³ = {:.1}% — {r:#?}",
		fraction * 100.0
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// `strut_lattice` refusals: an unknown family is a structured `invalid_param`
/// naming the supported kinds, and a bare (unshrouded) lattice is refused as
/// unbounded — never an endless meshing domain. Also pins that the op-count
/// arithmetic announced by `unknown_op` moved to 160.
#[test]
fn strut_lattice_bad_kind_and_unshrouded_lattice_refuse_loudly() {
	let dir = out_dir("strut_refusals");
	let bad_kind = run(
		&dir,
		json!([{"id": "lat", "op": "implicit", "voxel": 0.5,
			"expr": {"shape": "strut_lattice", "kind": "hexcomb", "cell": 10.0, "radius": 1.0,
			         "min": [0, 0, 0], "max": [10, 10, 10]}}]),
	);
	assert_refusal(&bad_kind, "lat", ErrorKind::InvalidParam, &["bcc|fcc|octet", "hexcomb"]);

	// The leaf alone is periodic over all of space; solid_from_implicit's
	// bounds come from the tree, and a graph_lattice-style unbounded tree via a
	// bare plane exercises the same guard — here the strut leaf HAS a bounds
	// hint, so the unbounded refusal is proven on `plane` (the shared path).
	let unbounded = run(
		&dir,
		json!([{"id": "s", "op": "solid_from_implicit", "voxel": 0.5,
			"expr": {"shape": "plane", "point": [0, 0, 0], "normal": [0, 0, 1]}}]),
	);
	assert_refusal(&unbounded, "s", ErrorKind::InvalidParam, &["unbounded", "domain"]);

	let unknown = run(&dir, json!([{"id": "x", "op": "frobnicate_solid"}]));
	// Derive the count from OP_COUNT so adding an op cannot silently stale this pin.
	let count_needle = format!("{} supported ops", kernel_api::OP_COUNT);
	assert_refusal(&unknown, "x", ErrorKind::UnknownOp, &[&count_needle, "describe"]);
	let _ = std::fs::remove_dir_all(&dir);
}

/// `pipe_path` leaf (green): the uniform-radius capsule chain is the SAME
/// field as the general `pipe` leaf given identical points — the two meshes'
/// volumes must agree exactly (one field, one mesher, one lattice).
#[test]
fn pipe_path_leaf_matches_the_general_pipe_leaf_exactly() {
	let dir = out_dir("pipe_path");
	let pts = json!([[0, 0, 0], [12, 0, 0], [12, 10, 0]]);
	let r = run(
		&dir,
		json!([
			{"id": "chain", "op": "implicit", "voxel": 0.3,
			 "expr": {"shape": "pipe_path", "points": pts, "radius": 2.0}},
			{"id": "general", "op": "implicit", "voxel": 0.3,
			 "expr": {"shape": "pipe", "path": pts, "radius": 2.0}}
		]),
	);
	assert!(r.ok, "pipe_path and pipe must both mesh green: {r:#?}");
	let (a, b) = (num(&r, "chain", "volume"), num(&r, "general", "volume"));
	assert!(
		a.is_finite() && (a - b).abs() < 1e-6 && a > 300.0,
		"pipe_path must be the exact same field as pipe (uniform radii): volumes {a:.6} vs {b:.6} mm³ (chain of two Ø4 capsules ≈ 300+ mm³) — {r:#?}"
	);

	let short = run(
		&dir,
		json!([{"id": "p", "op": "implicit", "voxel": 0.5,
		"expr": {"shape": "pipe_path", "points": [[0, 0, 0]], "radius": 2.0}}]),
	);
	assert_refusal(&short, "p", ErrorKind::InvalidParam, &["at least 2 points"]);
	let _ = std::fs::remove_dir_all(&dir);
}

/// `text` leaf (green): "LM-10" meshes watertight with ink in the expected
/// footprint; the volume pin is the executed receipt from this exact program.
#[test]
fn text_leaf_meshes_a_watertight_label() {
	let dir = out_dir("text_leaf");
	let r = run(
		&dir,
		json!([{"id": "label", "op": "implicit", "voxel": 0.15, "mesher": "manifold",
			"expr": {"shape": "text", "text": "LM-10", "height": 8.0, "stroke_radius": 0.6}}]),
	);
	assert!(r.ok, "text leaf must mesh green: {r:#?}");
	let vol = num(&r, "label", "volume");
	assert!(
		entry(&r, "label").measures.as_ref().unwrap()["watertight"] == json!(true) && (vol - 96.9).abs() < 5.0,
		"'LM-10' at height 8, stroke Ø1.2 must be watertight at ≈96.9 mm³ (executed pin, voxel 0.15 manifold DC); measured {vol:.2} mm³ — {r:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// `text` refusals: an unsupported character is a structured `invalid_param`
/// NAMING the character and the supported set — the kernel's panic is
/// unreachable from JSON (pre-validated); space-only text is refused too.
#[test]
fn text_unsupported_character_is_a_structured_refusal_not_a_panic() {
	let dir = out_dir("text_refusals");
	let bad = run(
		&dir,
		json!([{"id": "label", "op": "implicit", "voxel": 0.3,
			"expr": {"shape": "text", "text": "Ø8 BORE", "height": 8.0, "stroke_radius": 0.6}}]),
	);
	assert_refusal(&bad, "label", ErrorKind::InvalidParam, &["unsupported character 'Ø'", "Hershey Simplex", "0-9"]);

	let blank = run(
		&dir,
		json!([{"id": "label", "op": "implicit", "voxel": 0.3,
			"expr": {"shape": "text", "text": "   ", "height": 8.0, "stroke_radius": 0.6}}]),
	);
	assert_refusal(&blank, "label", ErrorKind::InvalidParam, &["non-space glyph"]);
	let _ = std::fs::remove_dir_all(&dir);
}

/// `displace` combinator (green): a knurled box grows (t ∈ [¼, ¾] outward for
/// positive amplitude), the zero set is preserved and the emitted field stays
/// ≤ 1-Lipschitz — proven operationally: narrow-band and dense extractions of
/// the SAME displaced tree agree (narrow-band pruning tore nothing).
#[test]
fn displace_knurl_grows_the_box_and_stays_narrowband_safe() {
	let dir = out_dir("displace");
	let tree = json!({"op": "displace", "amplitude": 0.4,
		"texture": {"kind": "knurl", "pitch": 2.0, "depth_frac": 1.0},
		"in": {"shape": "box", "min": [0, 0, 0], "max": [16, 16, 16]}});
	let r = run(
		&dir,
		json!([
			{"id": "nb",    "op": "implicit", "voxel": 0.2, "expr": tree,
			 "domain": {"min": [-1, -1, -1], "max": [17, 17, 17]}},
			{"id": "dense", "op": "implicit", "voxel": 0.2, "mesher": "manifold", "expr": tree,
			 "domain": {"min": [-1, -1, -1], "max": [17, 17, 17]}}
		]),
	);
	assert!(r.ok, "displaced box must mesh green on both meshers: {r:#?}");
	let (nb, dense) = (num(&r, "nb", "volume"), num(&r, "dense", "volume"));
	let base = 16.0f64.powi(3);
	assert!(
		nb > base && (nb - dense).abs() / dense < 0.01,
		"knurl amplitude +0.4 must GROW the 16³ box (base {base:.0} mm³) and narrow-band must agree with dense (≤ 1-Lipschitz contract): nb {nb:.1} vs dense {dense:.1} mm³ — {r:#?}"
	);

	let bad_kind = run(
		&dir,
		json!([{"id": "d", "op": "implicit", "voxel": 0.4,
		"expr": {"op": "displace", "amplitude": 0.4, "texture": {"kind": "wavy", "pitch": 2.0},
		         "in": {"shape": "box", "min": [0, 0, 0], "max": [8, 8, 8]}}}]),
	);
	assert_refusal(&bad_kind, "d", ErrorKind::InvalidParam, &["knurl|stipple|noise", "wavy"]);

	let bad_frac = run(
		&dir,
		json!([{"id": "d", "op": "implicit", "voxel": 0.4,
		"expr": {"op": "displace", "amplitude": 0.4, "texture": {"kind": "knurl", "pitch": 2.0, "depth_frac": 1.5},
		         "in": {"shape": "box", "min": [0, 0, 0], "max": [8, 8, 8]}}}]),
	);
	assert_refusal(&bad_frac, "d", ErrorKind::InvalidParam, &["depth_frac", "[0, 1]"]);
	let _ = std::fs::remove_dir_all(&dir);
}

/// `{"grid": …}` scalar source (green): a 2-sample NPY density ramp drives
/// `offset_by` as a grade law (0 → −1 mm, 1 → +1 mm along x) — the graded box
/// measurably loses material on the low-density side and gains on the high
/// side, versus the ungraded box.
#[test]
fn grid_field_npy_grade_law_drives_offset_by() {
	let dir = out_dir("grid_field");
	// rho = 0 at x = 0, 1 at x = 20 (dims (2,1,1), cell 20, origin at the box
	// low corner): constant in y/z by border clamping.
	let npy = kernel_api::bridge::write_npy_f32(&[2, 1, 1], &[0.0, 1.0]);
	std::fs::write(dir.join("ramp.npy"), &npy).expect("write ramp.npy");
	// A second grid stores the same ramp scaled ×50 (as a raw stress field
	// would be); `normalize: [0, 50]` must remap it onto the SAME grade law.
	let scaled = kernel_api::bridge::write_npy_f32(&[2, 1, 1], &[0.0, 50.0]);
	std::fs::write(dir.join("stress.npy"), &scaled).expect("write stress.npy");
	let grade = |path: &str, normalize: Value| {
		let mut grid = json!({"path": path, "origin": [0, 0, 0], "cell": 20.0, "law": [-1.0, 1.0]});
		if !normalize.is_null() {
			grid["normalize"] = normalize;
		}
		json!({"id": path.trim_end_matches(".npy"), "op": "implicit", "voxel": 0.4, "mesher": "manifold",
			"domain": {"min": [-2, -2, -2], "max": [22, 22, 12]},
			"expr": {"op": "offset_by", "max_abs": 1.0,
				"in": {"shape": "box", "min": [0, 0, 0], "max": [20, 20, 10]},
				"field": {"grid": grid}}})
	};
	let r = run(&dir, json!([grade("ramp.npy", Value::Null), grade("stress.npy", json!([0.0, 50.0]))]));
	assert!(r.ok, "NPY-graded offset_by must mesh green (raw density and normalized stress): {r:#?}");
	let vol = num(&r, "ramp", "volume");
	let vol_norm = num(&r, "stress", "volume");
	assert!(
		(vol - 4000.0).abs() < 250.0 && vol.is_finite() && (vol - vol_norm).abs() < 1e-6,
		"the ±1 mm x-ramp grade must reshape the 20×20×10 box (4000 mm³) roughly volume-neutrally, and normalize:[0,50] over the ×50 field must reproduce it EXACTLY (same law): raw {vol:.1} vs normalized {vol_norm:.1} mm³ — {r:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// `{"grid": …}` refusals: a missing NPY is `io` with the resolved path; a
/// malformed one is `invalid_param` carrying the kernel's precise reason; an
/// escape attempt (`..`) is confined like every other input path.
#[test]
fn grid_field_missing_and_malformed_npy_refuse_with_the_op_error_vocabulary() {
	let dir = out_dir("grid_refusals");
	let tree = |path: &str| {
		json!([{"id": "g", "op": "implicit", "voxel": 0.5,
			"expr": {"op": "offset_by", "max_abs": 1.0,
				"in": {"shape": "box", "min": [0, 0, 0], "max": [10, 10, 10]},
				"field": {"grid": {"path": path, "origin": [0, 0, 0], "cell": 10.0, "law": [-1.0, 1.0]}}}}])
	};
	let missing = run(&dir, tree("nope.npy"));
	assert_refusal(&missing, "g", ErrorKind::Io, &["cannot read", "nope.npy"]);

	std::fs::write(dir.join("garbage.npy"), b"this is not numpy").expect("write garbage");
	let malformed = run(&dir, tree("garbage.npy"));
	assert_refusal(&malformed, "g", ErrorKind::InvalidParam, &["garbage.npy", "NUMPY magic"]);

	let escape = run(&dir, tree("../outside.npy"));
	assert_refusal(&escape, "g", ErrorKind::InvalidParam, &[".."]);
	let _ = std::fs::remove_dir_all(&dir);
}

// --- B) voxel-route solid ops & probes ---------------------------------------------

/// `offset_solid` (green): growing a 20³ box by +2 mm lands on the exact
/// Steiner volume (a³ + 6a²r + 3πr²a + 4πr³/3 = 13587.5 mm³) within the voxel
/// class, shrinking by −2 mm lands on the sharp 16³ erosion; both re-enter the
/// solid environment (validate/volume run on the BOUND results).
#[test]
fn offset_solid_grow_and_shrink_land_on_the_analytic_volumes() {
	let dir = out_dir("offset_solid");
	let r = run(
		&dir,
		json!([
			{"id": "cube",   "op": "box", "min": [0, 0, 0], "max": [20, 20, 20]},
			{"id": "grown",  "op": "offset_solid", "in": "cube", "delta": 2.0, "voxel": 0.4},
			{"id": "shrunk", "op": "offset_solid", "in": "cube", "delta": -2.0, "voxel": 0.4},
			{"id": "vg", "op": "volume", "in": "grown"},
			{"id": "vs", "op": "volume", "in": "shrunk"}
		]),
	);
	assert!(r.ok, "offset_solid grow/shrink must bind green: {r:#?}");
	let steiner = 8000.0 + 6.0 * 400.0 * 2.0 + 3.0 * std::f64::consts::PI * 4.0 * 20.0 + 4.0 * std::f64::consts::PI * 8.0 / 3.0;
	let (vg, vs) = (num(&r, "vg", "volume"), num(&r, "vs", "volume"));
	let route_ok = measure(&r, "grown", "route") == &json!("voxel") && measure(&r, "grown", "faceted") == &json!(true);
	assert!(
		route_ok && (vg - steiner).abs() / steiner < 0.01 && (vs - 4096.0).abs() / 4096.0 < 0.01,
		"grown 20³+2 must be ≈ Steiner {steiner:.1} mm³ (measured {vg:.1}), shrunk −2 ≈ 16³ = 4096 (measured {vs:.1}), route 'voxel'/faceted — {r:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// `offset_solid` refusal: an erosion at/beyond the inradius leaves nothing —
/// a loud `invalid_param` naming the cause, never an empty bound solid.
#[test]
fn offset_solid_total_erosion_refuses() {
	let dir = out_dir("offset_erode");
	let r = run(
		&dir,
		json!([
			{"id": "cube", "op": "box", "min": [0, 0, 0], "max": [10, 10, 10]},
			{"id": "gone", "op": "offset_solid", "in": "cube", "delta": -6.0, "voxel": 0.4}
		]),
	);
	assert_refusal(&r, "gone", ErrorKind::InvalidParam, &["erodes it away", "inradius"]);
	let _ = std::fs::remove_dir_all(&dir);
}

/// `shell_solid` (green): a hollowed 30×30×20 box re-enters the solid
/// environment with the cavity intact — `shells: 2`, volume ≈ outer − sharp
/// erosion cavity (18000 − 26·26·16 = 7184 mm³) in the voxel class.
#[test]
fn shell_solid_binds_a_two_shell_hollow_at_the_analytic_wall_volume() {
	let dir = out_dir("shell_solid");
	let r = run(
		&dir,
		json!([
			{"id": "case",   "op": "box", "min": [0, 0, 0], "max": [30, 30, 20]},
			{"id": "hollow", "op": "shell_solid", "in": "case", "thickness": 2.0, "voxel": 0.4},
			{"id": "v", "op": "volume", "in": "hollow"},
			{"id": "check", "op": "assert", "in": "hollow", "shells": 2, "valid": true}
		]),
	);
	assert!(r.ok, "shell_solid must bind a valid 2-shell hollow: {r:#?}");
	let vol = num(&r, "v", "volume");
	let wall = 30.0 * 30.0 * 20.0 - 26.0 * 26.0 * 16.0;
	assert!(
		measure(&r, "hollow", "shells") == &json!(2)
			&& measure(&r, "hollow", "cavity") == &json!(true)
			&& measure(&r, "hollow", "route") == &json!("voxel")
			&& (vol - wall).abs() / wall < 0.02,
		"2 mm shell of 30×30×20 must carry its cavity (shells 2) at ≈{wall:.0} mm³ wall; measured {vol:.1} — {r:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// `shell_solid` refusals: a non-positive wall and a wall under 2×voxel are
/// both deterministic up-front `invalid_param`s (provoked, not theorized).
#[test]
fn shell_solid_refuses_nonpositive_and_unresolvable_walls() {
	let dir = out_dir("shell_refusals");
	let nonpositive = run(
		&dir,
		json!([
			{"id": "case", "op": "box", "min": [0, 0, 0], "max": [20, 20, 10]},
			{"id": "bad", "op": "shell_solid", "in": "case", "thickness": -1.0}
		]),
	);
	assert_refusal(&nonpositive, "bad", ErrorKind::InvalidParam, &["positive wall thickness"]);

	let unresolvable = run(
		&dir,
		json!([
			{"id": "case", "op": "box", "min": [0, 0, 0], "max": [20, 20, 10]},
			{"id": "bad", "op": "shell_solid", "in": "case", "thickness": 0.5, "voxel": 0.4}
		]),
	);
	assert_refusal(&unresolvable, "bad", ErrorKind::InvalidParam, &["under 2 × voxel", "cannot resolve"]);
	let _ = std::fs::remove_dir_all(&dir);
}

/// `solid_from_implicit` (green): a Ø16 sphere crosses the bridge into a
/// validated faceted solid — volume on the analytic 4πr³/3 within the voxel
/// class, conservation gate passed, genus 0.
#[test]
fn solid_from_implicit_bridges_a_sphere_at_the_analytic_volume() {
	let dir = out_dir("reverse_sphere");
	let r = run(
		&dir,
		json!([
			{"id": "ball", "op": "solid_from_implicit", "voxel": 0.4,
			 "expr": {"shape": "sphere", "center": [0, 0, 0], "radius": 8.0}},
			{"id": "val", "op": "validate", "in": "ball"}
		]),
	);
	assert!(r.ok, "sphere must bridge to a valid solid: {r:#?}");
	let analytic = 4.0 * std::f64::consts::PI * 512.0 / 3.0;
	let vol = num(&r, "ball", "volume");
	assert!(
		measure(&r, "ball", "volume_conserved") == &json!(true)
			&& measure(&r, "ball", "route") == &json!("voxel")
			&& measure(&r, "val", "genus") == &json!(0)
			&& (vol - analytic).abs() / analytic < 0.02,
		"bridged Ø16 sphere: volume {vol:.1} vs analytic {analytic:.1} mm³ (voxel 0.4 class), conserved, genus 0 — {r:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// `solid_from_implicit` refusal: a domain the field never crosses has nothing
/// to bridge — `invalid_param` carrying the kernel's own reason.
#[test]
fn solid_from_implicit_empty_domain_refuses_with_nothing_to_bridge() {
	let dir = out_dir("reverse_empty");
	let r = run(
		&dir,
		json!([{"id": "ghost", "op": "solid_from_implicit", "voxel": 0.5,
			"domain": {"min": [50, 50, 50], "max": [60, 60, 60]},
			"expr": {"shape": "sphere", "center": [0, 0, 0], "radius": 8.0}}]),
	);
	assert_refusal(&r, "ghost", ErrorKind::InvalidParam, &["nothing to bridge"]);
	let _ = std::fs::remove_dir_all(&dir);
}

/// THE ROUND-TRIP GATE: strut lattice ∩ box → `solid_from_implicit` →
/// `export_step`, pure JSON end to end — the deployed-operator path from a
/// periodic lattice to a STEP file, with volume and validity pinned from the
/// receipts of this exact program.
#[test]
fn roundtrip_strut_lattice_to_solid_to_step() {
	let dir = out_dir("roundtrip");
	let r = run(
		&dir,
		json!([
			{"id": "bridged", "op": "solid_from_implicit", "voxel": 0.5,
			 "expr": {"op": "intersection",
				"a": {"shape": "strut_lattice", "kind": "bcc", "cell": 10.0, "radius": 1.6,
					  "min": [0, 0, 0], "max": [20, 20, 20]},
				"b": {"shape": "box", "min": [0, 0, 0], "max": [20, 20, 20]}}},
			{"id": "val",  "op": "validate", "in": "bridged"},
			{"id": "vol",  "op": "volume", "in": "bridged"},
			{"id": "step", "op": "export_step", "in": "bridged", "file": "bcc_lattice.step"}
		]),
	);
	assert!(r.ok, "lattice → solid → STEP must run green end to end: {r:#?}");
	let vol = num(&r, "vol", "volume");
	let bridged_vol = num(&r, "bridged", "volume");
	let step_written = entry(&r, "step").file.as_deref().map(|f| std::fs::metadata(f).map(|m| m.len()).unwrap_or(0)).unwrap_or(0);
	assert!(
		measure(&r, "val", "closed") == &json!(true)
			&& measure(&r, "val", "manifold") == &json!(true)
			&& measure(&r, "bridged", "volume_conserved") == &json!(true)
			&& (vol - bridged_vol).abs() < 1e-6
			&& (vol / 8000.0 - 0.398).abs() < 0.02
			&& step_written > 0,
		"BCC 2×2×2 block (cell 10, r 1.6): bound volume {vol:.1} mm³ ({:.1}% solid, executed pin ≈39.8% / 3184.2 mm³ at voxel 0.5), closed+manifold, conserved, STEP {step_written} bytes — {r:#?}",
		vol / 80.0
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// `thin_wall` (green, both input forms) on a 3 mm plate — three executed
/// truths, pinned separately:
/// 1. tree census over the whole part reads 2.43 mm (a corner-bisector medial
///    sample — near a sharp 90° edge the material wedge genuinely thins, so a
///    whole-part census on a sharp-edged body reads the edge region, slightly
///    under the 3 mm wall);
/// 2. the solid path with an INTERIOR `domain` (the wall-interrogation idiom,
///    documented in API.md) reads 2.57 ≈ 3 mm − the documented ≤ one-cell
///    under-report;
/// 3. the solid path over the whole part reads the 0.60 mm edge-wedge sliver —
///    pinned so the documented caveat can never silently change.
#[test]
fn thin_wall_census_reads_a_3mm_plate_from_tree_and_solid() {
	let dir = out_dir("thin_wall");
	let plate_tree = json!({"shape": "box", "min": [0, 0, 0], "max": [30, 30, 3]});
	let r = run(
		&dir,
		json!([
			{"id": "plate", "op": "box", "min": [0, 0, 0], "max": [30, 30, 3]},
			{"id": "warn",  "op": "thin_wall", "expr": plate_tree, "t_min": 4.0, "samples": 48},
			{"id": "clear", "op": "thin_wall", "expr": plate_tree, "t_min": 1.5, "samples": 48},
			{"id": "walls", "op": "thin_wall", "in": "plate", "t_min": 4.0, "samples": 48,
			 "domain": {"min": [5, 5, -0.5], "max": [25, 25, 3.5]}},
			{"id": "edges", "op": "thin_wall", "in": "plate", "t_min": 4.0, "samples": 48}
		]),
	);
	assert!(r.ok, "thin_wall censuses must run green: {r:#?}");
	let (t_tree, t_walls, t_edges) = (num(&r, "warn", "thinnest"), num(&r, "walls", "thinnest"), num(&r, "edges", "thinnest"));
	let below_warn = num(&r, "warn", "below_count");
	let below_clear = num(&r, "clear", "below_count");
	assert!(
		measure(&r, "warn", "status") == &json!("measured")
			&& (2.2..=3.1).contains(&t_tree)
			&& (2.55..=3.05).contains(&t_walls)
			&& (0.55..=0.65).contains(&t_edges)
			&& below_warn > 0.0
			&& below_clear == 0.0,
		"3 mm plate (executed pins): tree census {t_tree:.2} mm (≈2.43, corner-bisector), interior-domain solid census {t_walls:.2} mm (≈2.57 = 3 − one-cell under-report), whole-part solid census {t_edges:.2} mm (≈0.60 edge-wedge sliver — the documented sharp-edge caveat), below t_min=4 fires ({below_warn} samples), below t_min=1.5 clear — {r:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// `thin_wall` explicit statuses & refusals: a census box with no interior is
/// `status: "no_interior_samples"` (never a raw ∞ in JSON); both-inputs and
/// out-of-range samples are structured refusals.
#[test]
fn thin_wall_empty_census_and_bad_params_are_explicit() {
	let dir = out_dir("thin_wall_refusals");
	let empty = run(
		&dir,
		json!([{"id": "w", "op": "thin_wall", "t_min": 1.0, "samples": 16,
			"domain": {"min": [50, 50, 50], "max": [60, 60, 60]},
			"expr": {"shape": "box", "min": [0, 0, 0], "max": [10, 10, 10]}}]),
	);
	assert!(empty.ok, "an empty census is a STATUS, not a failure: {empty:#?}");
	assert!(
		measure(&empty, "w", "status") == &json!("no_interior_samples") && measure(&empty, "w", "thinnest") == &json!(null),
		"no interior ⇒ status no_interior_samples with thinnest null — {empty:#?}"
	);

	let both = run(
		&dir,
		json!([
			{"id": "b", "op": "box", "min": [0, 0, 0], "max": [5, 5, 5]},
			{"id": "w", "op": "thin_wall", "in": "b", "t_min": 1.0,
			 "expr": {"shape": "box", "min": [0, 0, 0], "max": [5, 5, 5]}}
		]),
	);
	assert_refusal(&both, "w", ErrorKind::InvalidParam, &["exactly one of 'in'", "'expr'"]);

	let coarse = run(
		&dir,
		json!([{"id": "w", "op": "thin_wall", "t_min": 1.0, "samples": 4,
		"expr": {"shape": "box", "min": [0, 0, 0], "max": [5, 5, 5]}}]),
	);
	assert_refusal(&coarse, "w", ErrorKind::InvalidParam, &["8..=256"]);
	let _ = std::fs::remove_dir_all(&dir);
}

/// `min_ligament` (green): a planned Ø6 bore 5 mm from a plate edge echoes the
/// 2.0 mm ligament (the kernel-brep pin), and a bore aimed OUT of the material
/// is the explicit `no_material` status — never NaN in the JSON.
#[test]
fn min_ligament_echoes_the_edge_web_and_maps_sentinels_to_statuses() {
	let dir = out_dir("min_ligament");
	let r = run(
		&dir,
		json!([
			{"id": "plate", "op": "box", "min": [0, 0, 0], "max": [30, 30, 12]},
			{"id": "edge",  "op": "min_ligament", "in": "plate", "at": [5, 15, 12], "axis": [0, 0, -1], "d": 6.0},
			{"id": "sky",   "op": "min_ligament", "in": "plate", "at": [5, 15, 12], "axis": [0, 0, 1], "d": 6.0}
		]),
	);
	assert!(r.ok, "min_ligament is advisory — both questions must answer green: {r:#?}");
	let lig = num(&r, "edge", "ligament");
	assert!(
		measure(&r, "edge", "status") == &json!("measured")
			&& (lig - 2.0).abs() < 0.02
			&& measure(&r, "sky", "status") == &json!("no_material")
			&& measure(&r, "sky", "ligament") == &json!(null),
		"Ø6 at 5 mm from the edge must echo ≈2.000 mm (measured {lig:.4}); the up-facing bore is status no_material — {r:#?}"
	);

	let bad_d = run(
		&dir,
		json!([
			{"id": "plate", "op": "box", "min": [0, 0, 0], "max": [30, 30, 12]},
			{"id": "m", "op": "min_ligament", "in": "plate", "at": [5, 15, 12], "axis": [0, 0, -1], "d": -3.0}
		]),
	);
	assert_refusal(&bad_d, "m", ErrorKind::InvalidParam, &["positive bore diameter"]);

	let bad_axis = run(
		&dir,
		json!([
			{"id": "plate", "op": "box", "min": [0, 0, 0], "max": [30, 30, 12]},
			{"id": "m", "op": "min_ligament", "in": "plate", "at": [5, 15, 12], "axis": [0, 0, 0], "d": 6.0}
		]),
	);
	assert_refusal(&bad_axis, "m", ErrorKind::InvalidParam, &["non-zero finite"]);
	let _ = std::fs::remove_dir_all(&dir);
}
