//! The gate oracles the campaigns could not trust (themes T5, T6, T15) and the
//! frustum constructor (T8).
//!
//! Every test here is a REPRODUCTION first: the comment names the exact wrong
//! number the engine used to report, so the test fails loudly if the defect
//! comes back.

use kernel_api::{run_program, ErrorKind, OpReport, Report};
use serde_json::{json, Value};
use std::path::{Path, PathBuf};

fn dir(tag: &str) -> PathBuf {
	let d = std::env::temp_dir().join(format!("cadcode_oracle_{tag}_{}", std::process::id()));
	std::fs::create_dir_all(&d).unwrap();
	d
}
fn run(d: &Path, ops: Value) -> Report {
	run_program(&serde_json::to_string(&json!({ "ops": ops })).unwrap(), d)
}
fn op<'r>(r: &'r Report, id: &str) -> &'r OpReport {
	r.ops.iter().find(|o| o.id == id).unwrap_or_else(|| panic!("no op '{id}' in {r:#?}"))
}
fn measure(r: &Report, id: &str, key: &str) -> Value {
	op(r, id).measures.as_ref().and_then(|m| m.get(key)).cloned().unwrap_or(Value::Null)
}
fn num(r: &Report, id: &str, key: &str) -> f64 {
	measure(r, id, key).as_f64().unwrap_or_else(|| panic!("op '{id}' has no numeric '{key}' — {r:#?}"))
}
fn failure(r: &Report, id: &str, kind: ErrorKind) -> String {
	let e = op(r, id).error.as_ref().unwrap_or_else(|| panic!("op '{id}' did not fail — {r:#?}"));
	assert_eq!(e.kind, kind, "op '{id}' failed with the wrong kind — {r:#?}");
	e.message.clone()
}

// --- T5 -----------------------------------------------------------------------

/// T5 (horn F7): a coaxial spigot nested in a counterbore with a real 0.30 mm
/// radial gap and a 1.0 mm axial gap reported `distance: 0.0` alongside
/// `interfering: false, overlap_volume: 0.0` — two fields flatly contradicting
/// each other, and `assert_disjoint` trusting the wrong one.
///
/// Root cause: the bore wall's subdivided seam contains ZERO-AREA triangles, and
/// the Möller triangle–triangle predicate is undefined on those (null plane
/// normal) — it claimed an intersection against anything straddling the sliver,
/// collapsing the whole query to 0.
#[test]
fn nested_coaxial_pair_with_a_real_gap_measures_it() {
	let d = dir("t5");
	let body = json!([
		{"id":"blk","op":"cylinder","base":[0,0,0],"axis":[0,0,1],"radius":25,"height":20},
		{"id":"cb","op":"cylinder","base":[0,0,10],"axis":[0,0,1],"radius":17.1,"height":11},
		{"id":"body","op":"difference","a":"blk","b":"cb"},
		{"id":"spig","op":"cylinder","base":[0,0,11],"axis":[0,0,1],"radius":16.8,"height":8},
		{"id":"cl","op":"clearance","a":"body","b":"spig"}
	]);
	let r = run(&d, body);
	assert!(r.ok, "{r:#?}");
	let distance = num(&r, "cl", "distance");
	// Nominal radial gap 0.30 mm; both operands are INSCRIBED polygons, so the
	// measured surface gap is smaller but must be a real, positive number.
	assert!(distance > 0.15 && distance < 0.30, "nested pair must measure its gap, got {distance} — {r:#?}");
	assert_eq!(measure(&r, "cl", "interfering"), json!(false), "{r:#?}");
	// The two fields must agree: a positive distance is never "interfering".
	assert_eq!(measure(&r, "cl", "overlap_volume"), json!(0.0), "{r:#?}");

	// …and `assert_disjoint` can now prove the register clears.
	let r = run(
		&d,
		json!([
			{"id":"blk","op":"cylinder","base":[0,0,0],"axis":[0,0,1],"radius":25,"height":20},
			{"id":"cb","op":"cylinder","base":[0,0,10],"axis":[0,0,1],"radius":17.1,"height":11},
			{"id":"body","op":"difference","a":"blk","b":"cb"},
			{"id":"spig","op":"cylinder","base":[0,0,11],"axis":[0,0,1],"radius":16.8,"height":8},
			{"id":"gap","op":"assert_disjoint","a":"body","b":"spig","min_clearance":0.05}
		]),
	);
	assert!(r.ok, "assert_disjoint must pass on a genuinely clearing register — {r:#?}");
	let _ = std::fs::remove_dir_all(&d);
}

/// The guard must not blind the query: two solids that really do interpenetrate
/// still measure 0 and report the overlap volume.
#[test]
fn interpenetrating_solids_still_read_zero() {
	let d = dir("t5b");
	let r = run(
		&d,
		json!([
			{"id":"a","op":"box","min":[0,0,0],"max":[10,10,10]},
			{"id":"b","op":"box","min":[5,5,5],"max":[15,15,15]},
			{"id":"cl","op":"clearance","a":"a","b":"b"}
		]),
	);
	assert_eq!(num(d_ok(&r), "cl", "distance"), 0.0, "{r:#?}");
	assert_eq!(measure(&r, "cl", "interfering"), json!(true), "{r:#?}");
	assert!((num(&r, "cl", "overlap_volume") - 125.0).abs() < 1e-9, "{r:#?}");
	let _ = std::fs::remove_dir_all(&d);
}
/// Identity helper so the assertion above reads as one expression.
fn d_ok(r: &Report) -> &Report {
	assert!(r.ok, "{r:#?}");
	r
}

/// A `null` that does not say WHY is indistinguishable from a bug (T5).
#[test]
fn a_null_overlap_volume_always_carries_its_reason() {
	let d = dir("t5c");
	let r = run(
		&d,
		json!([
			{"id":"a","op":"box","min":[0,0,0],"max":[10,10,10]},
			{"id":"stl","op":"export_stl","in":"a","file":"a.stl"},
			{"id":"b","op":"box","min":[20,0,0],"max":[30,10,10]},
			{"id":"cl","op":"clearance","a":"stl","b":"b"}
		]),
	);
	assert!(r.ok, "clearance must accept a bound mesh — {r:#?}");
	assert_eq!(measure(&r, "cl", "overlap_volume"), Value::Null, "{r:#?}");
	let reason = measure(&r, "cl", "overlap_volume_reason");
	assert!(reason.as_str().unwrap_or("").contains("exact solids"), "{r:#?}");
	assert_eq!(num(&r, "cl", "distance"), 10.0, "{r:#?}");
	let _ = std::fs::remove_dir_all(&d);
}

// --- T6 -----------------------------------------------------------------------

/// T6(a), verified: two boxes with a 0.0005 mm gap read `shells: 2` (severance
/// seen) but `components: 1` at the default 1e-3 weld. The gate was untunable
/// because `weld_tol` existed only on the `mesh_components` MEASURE, never on
/// the `assert` path.
#[test]
fn the_connectivity_gate_exposes_its_weld_tolerance() {
	let d = dir("t6a");
	let severed = |extra: Value| {
		json!([
			{"id":"a","op":"box","min":[0,0,0],"max":[10,10,10]},
			{"id":"b","op":"box","min":[10.0005,0,0],"max":[20,10,10]},
			{"id":"u","op":"union_all","in":["a","b"]},
			{"id":"v","op":"validate","in":"u"},
			extra
		])
	};
	// Default weld: the 0.0005 mm severance is welded shut — one body.
	let r = run(&d, severed(json!({"id":"g","op":"assert","in":"u","components":1})));
	assert!(r.ok, "{r:#?}");
	assert_eq!(measure(&r, "v", "shells"), json!(2), "shells must see the severance — {r:#?}");

	// A weld BELOW the gap makes the same gate catch it. Without `weld_tol` on
	// `assert` this program cannot be written at all.
	let r = run(&d, severed(json!({"id":"g","op":"assert","in":"u","components":1,"weld_tol":1e-6})));
	let msg = failure(&r, "g", ErrorKind::AssertFailed);
	assert!(msg.contains("measured 2"), "{msg}");
	let _ = std::fs::remove_dir_all(&d);
}

/// T6(a), the deeper defect: `weld_tol` was a GRID PITCH, so whether a gap of a
/// given size welded depended on the part's absolute position. The same 0.0004 mm
/// gap read `components: 1` at one origin and `2` at another.
#[test]
fn the_weld_tolerance_is_position_independent() {
	let d = dir("t6a2");
	let pair = |a_max: f64, b_min: f64| {
		json!([
			{"id":"a","op":"box","min":[0,0,0],"max":[a_max,10,10]},
			{"id":"b","op":"box","min":[b_min,0,0],"max":[30,10,10]},
			{"id":"u","op":"union_all","in":["a","b"]},
			{"id":"mc","op":"mesh_components","in":"u"}
		])
	};
	// Both pairs have the SAME 0.0004 mm gap; only the absolute position differs.
	let on_grid = run(&d, pair(10.0, 10.0004));
	let off_grid = run(&d, pair(10.0003, 10.0007));
	assert_eq!(
		measure(&on_grid, "mc", "components"),
		measure(&off_grid, "mc", "components"),
		"the same gap must give the same answer wherever the part sits — {on_grid:#?} {off_grid:#?}"
	);
	assert_eq!(measure(&on_grid, "mc", "components"), json!(1), "0.0004 mm < weld_tol 1e-3 ⇒ welded");
	let _ = std::fs::remove_dir_all(&d);
}

/// T6(b)/(c): a topologically perfect `extrude_with_holes` plate reported
/// `components: 3` (one extra per hole loop) and a STEP-round-tripped pocket
/// reported 2, at EVERY weld tolerance. The count was measuring the faceter, not
/// the part. A count taken on a measurement surface that is not closed is not a
/// body count, so the oracle now refuses instead of reporting it.
#[test]
fn a_connectivity_count_on_a_cracked_measurement_surface_is_refused() {
	// HISTORY: this test originally pinned the mitigation for the sealed-hole
	// tessellation defect — `mesh_components` on a holed plate REFUSED because
	// the measurement surface had boundary cracks (the adaptive tessellator
	// dropped `Face::inner`). The ROOT was fixed on 2026-08-14 (see
	// kernel-brep/tests/adaptive_holes.rs), so the fixture that used to crack
	// now measures cleanly — and THAT is the new pinned truth. The solid-path
	// refusal branch stays in the interpreter as a tripwire for any future
	// tessellation defect; no known solid can trigger it today, which is
	// exactly what this test now proves for the two shapes that used to.
	let d = dir("t6b");
	let plate = json!([
		{"id":"p","op":"extrude_with_holes","outer":[[0,0],[40,0],[40,20],[0,20]],
		 "holes":[[[5,5],[10,5],[10,10],[5,10]],[[25,5],[30,5],[30,10],[25,10]]],"height":5},
		{"id":"v","op":"validate","in":"p"},
		{"id":"mc","op":"mesh_components","in":"p","require":{"components":1,"watertight":true}}
	]);
	let r = run(&d, plate);
	// The SOLID is provably fine: closed, manifold, one shell, genus 2 — and its
	// measurement surface now closes, so the oracle MEASURES it: one body.
	assert_eq!(measure(&r, "v", "closed"), json!(true), "{r:#?}");
	assert_eq!(measure(&r, "v", "shells"), json!(1), "{r:#?}");
	assert_eq!(measure(&r, "v", "genus"), json!(2), "{r:#?}");
	assert!(r.ok, "the two-hole plate must MEASURE (components 1) now that holes tessellate — {r:#?}");
	assert_eq!(measure(&r, "mc", "boundary_edges"), json!(0), "no cracks left — {r:#?}");

	// A mesh VALUE is measured as-is, never refused: an OPEN imported mesh is
	// not a lie about a solid, it IS the object — so the counters go on the
	// record (`watertight: false`, boundary_edges named) and gating is the
	// caller's `require` to write.
	std::fs::write(d.join("open.stl"), {
		let mut b: Vec<u8> = vec![0; 80];
		b.extend_from_slice(&1u32.to_le_bytes());
		for v in [[0f32, 0., 1.], [0., 0., 0.], [10., 0., 0.], [0., 10., 0.]] {
			for c in v {
				b.extend_from_slice(&c.to_le_bytes());
			}
		}
		b.extend_from_slice(&0u16.to_le_bytes());
		b
	})
	.unwrap();
	let r = kernel_api::run_program_with_input_base(
		&serde_json::to_string(&json!({ "ops": [
			{"id":"m","op":"import_mesh","file":"open.stl"},
			{"id":"mc","op":"mesh_components","in":"m"}
		]}))
		.unwrap(),
		&d,
		&d,
	);
	assert!(r.ok, "an open MESH measures honestly instead of refusing — {r:#?}");
	assert_eq!(measure(&r, "mc", "boundary_edges"), json!(3), "{r:#?}");
	assert_eq!(measure(&r, "mc", "watertight"), json!(false), "{r:#?}");

	// A solid whose tessellation IS closed is measured, not refused.
	let r = run(
		&d,
		json!([
			{"id":"a","op":"box","min":[0,0,0],"max":[10,10,10]},
			{"id":"b","op":"box","min":[3,3,5],"max":[7,7,11]},
			{"id":"t","op":"difference","a":"a","b":"b"},
			{"id":"mc","op":"mesh_components","in":"t","require":{"components":1,"watertight":true}}
		]),
	);
	assert!(r.ok, "a boolean pocket must still measure — {r:#?}");
	let _ = std::fs::remove_dir_all(&d);
}

// --- T10 second half ------------------------------------------------------------

/// T10: `implicit` / `hybrid_boolean` / `import_mesh` / the exports bound no
/// value, so two campaigns' ACTUAL PRINT FILES could not be gated at all. They
/// bind a MESH now — never a B-rep, so the one-directional implicit→exact
/// contract is untouched.
#[test]
fn a_print_file_always_has_a_gate() {
	let d = dir("t10");
	// Since the sealed-hole tessellation fix (2026-08-14) the holed plate takes
	// the EXACT export route — an upgrade this test must witness, not resist:
	// the mesh-value gating chain below works identically on either route, and
	// `route: "exact"` is now the pinned truth for a trivially exact part.
	let r = run(
		&d,
		json!([
			{"id":"p","op":"extrude_with_holes","outer":[[0,0],[40,0],[40,20],[0,20]],
			 "holes":[[[5,5],[10,5],[10,10],[5,10]]],"height":5},
			{"id":"stl","op":"export_stl","in":"p","file":"p.stl","require":{"watertight":true}},
			{"id":"one","op":"mesh_components","in":"stl","require":{"components":1,"watertight":true}},
			{"id":"bed","op":"bounding_box","in":"stl","envelope":[256,256,256],"require":{"fits_within":true}},
			{"id":"sup","op":"support_report","in":"stl","require":{"steep_area":{"max":0.0}}},
			{"id":"val","op":"validate","in":"stl","require":{"closed":true,"manifold":true}},
			{"id":"gate","op":"assert","in":"stl","components":1,"closed":true}
		]),
	);
	assert!(r.ok, "the print file must be fully gateable — {r:#?}");
	assert_eq!(measure(&r, "stl", "route"), json!("exact"), "a holed plate is trivially exact now — {r:#?}");
	assert_eq!(measure(&r, "one", "source"), json!("mesh"), "{r:#?}");
	assert_eq!(measure(&r, "val", "source"), json!("mesh"), "{r:#?}");
	let _ = std::fs::remove_dir_all(&d);
}

/// A mesh value must never masquerade as a B-rep: the topology-only assertions
/// refuse rather than invent a genus or a shell count from triangles.
#[test]
fn a_mesh_is_never_promoted_to_a_solid() {
	let d = dir("t10b");
	let r = run(
		&d,
		json!([
			{"id":"p","op":"box","min":[0,0,0],"max":[10,10,10]},
			{"id":"stl","op":"export_stl","in":"p","file":"p.stl"},
			{"id":"bad","op":"assert","in":"stl","genus":0}
		]),
	);
	let msg = failure(&r, "bad", ErrorKind::WrongType);
	assert!(msg.contains("needs a bound SOLID"), "{msg}");

	// An op that needs exact geometry refuses a mesh with a `wrong_type`.
	let r = run(
		&d,
		json!([
			{"id":"p","op":"box","min":[0,0,0],"max":[10,10,10]},
			{"id":"stl","op":"export_stl","in":"p","file":"p.stl"},
			{"id":"cut","op":"difference","a":"stl","b":"p"}
		]),
	);
	let msg = failure(&r, "cut", ErrorKind::WrongType);
	assert!(msg.contains("is a mesh"), "{msg}");

	// A leaky mesh has no enclosed volume — a refusal, not a plausible number.
	let r = run(
		&d,
		json!([
			{"id":"p","op":"box","min":[0,0,0],"max":[10,10,10]},
			{"id":"stl","op":"export_stl","in":"p","file":"p.stl"},
			{"id":"v","op":"volume","in":"stl"}
		]),
	);
	assert!(r.ok, "a watertight export DOES have a volume — {r:#?}");
	assert_eq!(num(&r, "v", "volume"), 1000.0, "{r:#?}");
	assert_eq!(measure(&r, "v", "source"), json!("mesh"), "{r:#?}");
	let _ = std::fs::remove_dir_all(&d);
}

// --- T15 -----------------------------------------------------------------------

/// T15 (turgo F1, ball F1, rotor F4): `validate.geometric_ok` flips false on
/// solids every other gate calls clean, with NOTHING to act on — so three
/// campaigns learned to ignore it, the worst outcome for a validity signal.
///
/// It is a TRUE positive here: the exported STL of this polar pattern really does
/// contain crossing triangles. The fix is therefore to say WHERE.
#[test]
fn a_failing_validity_flag_carries_a_witness() {
	let d = dir("t15");
	let r = run(
		&d,
		json!([
			{"id":"o","op":"cylinder","base":[24,0,7],"axis":[1,0,0],"radius":14.2,"height":24,"segments":64},
			{"id":"i","op":"cylinder","base":[23,0,7],"axis":[1,0,0],"radius":12.0,"height":26,"segments":64},
			{"id":"d","op":"difference","a":"o","b":"i"},
			{"id":"vd","op":"validate","in":"d"},
			{"id":"p3","op":"polar_pattern","in":"d","count":3,"center":[0,0,0],"axis":[0,0,1]},
			{"id":"v3","op":"validate","in":"p3"}
		]),
	);
	assert!(r.ok, "{r:#?}");
	// One tube is clean and carries no witness.
	assert_eq!(measure(&r, "vd", "geometric_ok"), json!(true), "{r:#?}");
	assert_eq!(measure(&r, "vd", "self_intersection"), Value::Null, "a clean solid must carry no witness — {r:#?}");
	// The pattern is not, and now says where.
	assert_eq!(measure(&r, "v3", "geometric_ok"), json!(false), "{r:#?}");
	let w = measure(&r, "v3", "self_intersection");
	assert!(w["triangles"].as_array().map(|a| a.len()) == Some(2), "witness must name two triangles — {w}");
	assert!(w["point"].as_array().map(|a| a.len()) == Some(3), "witness must give a point — {w}");
	assert!(w["pairs"].as_u64().unwrap_or(0) > 0, "witness must count the pairs — {w}");

	// Determinism: the witness is the lexicographically lowest pair, so a rebuild
	// reproduces it byte for byte.
	let again = run(
		&d,
		json!([
			{"id":"o","op":"cylinder","base":[24,0,7],"axis":[1,0,0],"radius":14.2,"height":24,"segments":64},
			{"id":"i","op":"cylinder","base":[23,0,7],"axis":[1,0,0],"radius":12.0,"height":26,"segments":64},
			{"id":"d","op":"difference","a":"o","b":"i"},
			{"id":"p3","op":"polar_pattern","in":"d","count":3,"center":[0,0,0],"axis":[0,0,1]},
			{"id":"v3","op":"validate","in":"p3"}
		]),
	);
	assert_eq!(w, measure(&again, "v3", "self_intersection"), "the witness must be deterministic");
	let _ = std::fs::remove_dir_all(&d);
}

// --- T8 ------------------------------------------------------------------------

/// T8: `cone` was a TRUE cone with no frustum constructor, so every draughted
/// boss had to be built as a cone with its tip differenced off.
#[test]
fn cone_builds_a_frustum_and_keeps_it_analytic() {
	let d = dir("t8");
	let r = run(
		&d,
		json!([
			{"id":"f","op":"cone","base":[0,0,0],"axis":[0,0,1],"radius":10,"height":20,"top_radius":4,"segments":128},
			{"id":"v","op":"validate","in":"f"},
			{"id":"ev","op":"exact_volume","in":"f"},
			{"id":"bb","op":"bounding_box","in":"f"},
			{"id":"off","op":"cone","base":[5,5,0],"axis":[1,1,1],"radius":6,"height":12,"top_radius":2,"segments":64},
			{"id":"evoff","op":"exact_volume","in":"off"},
			{"id":"step","op":"export_step","in":"f","file":"frustum.step"}
		]),
	);
	assert!(r.ok, "{r:#?}");
	assert_eq!(measure(&r, "v", "valid"), json!(true), "{r:#?}");
	assert_eq!(measure(&r, "v", "genus"), json!(0), "{r:#?}");
	// π-exact: V = πh(R² + Rr + r²)/3. Analytic, not faceted — the lateral band
	// keeps the same exact cone tag the un-truncated cone has.
	let exact = std::f64::consts::PI * 20.0 / 3.0 * (100.0 + 40.0 + 16.0);
	assert!((num(&r, "ev", "exact_volume") - exact).abs() < 1e-9 * exact, "{r:#?}");
	let off_exact = std::f64::consts::PI * 12.0 / 3.0 * (36.0 + 12.0 + 4.0);
	assert!((num(&r, "evoff", "exact_volume") - off_exact).abs() < 1e-9 * off_exact, "off-axis frustum — {r:#?}");
	assert_eq!(measure(&r, "bb", "size"), json!([20.0, 20.0, 20.0]), "{r:#?}");

	// `top_radius: 0` (and omitting it) is the historic true cone, unchanged.
	let r = run(
		&d,
		json!([
			{"id":"c","op":"cone","base":[0,0,0],"axis":[0,0,1],"radius":10,"height":20,"segments":128},
			{"id":"ev","op":"exact_volume","in":"c"},
			{"id":"c0","op":"cone","base":[0,0,0],"axis":[0,0,1],"radius":10,"height":20,"segments":128,"top_radius":0},
			{"id":"ev0","op":"exact_volume","in":"c0"}
		]),
	);
	assert_eq!(measure(&r, "ev", "exact_volume"), measure(&r, "ev0", "exact_volume"), "{r:#?}");

	// Equal radii is a cylinder — refused loudly, never silently degraded.
	let r = run(
		&d,
		json!([
			{"id":"f","op":"cone","base":[0,0,0],"axis":[0,0,1],"radius":10,"height":20,"top_radius":10}
		]),
	);
	let msg = failure(&r, "f", ErrorKind::InvalidParam);
	assert!(msg.contains("CYLINDER"), "{msg}");
	let _ = std::fs::remove_dir_all(&d);
}
