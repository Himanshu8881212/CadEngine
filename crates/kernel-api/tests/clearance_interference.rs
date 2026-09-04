// Copyright (c) LMCAD. Licensed under the MIT License.

//! `clearance` — the two defects the friction corpus names more often than any
//! other op (`clearance` 28 times, `overlap_volume` 20 across 22 campaigns).
//!
//! 1. **False-positive interference on disjoint curved-face pairs** — a pin
//!    coaxial inside a bore read `distance: 0.0` / `interfering: true`, so
//!    `assert_disjoint` false-failed provably disjoint geometry
//!    (`campaign/friction/iso9409_wedge_flexure_gripper.md` F2,
//!    `prosthetic_wrist_quick_disconnect.md`, digest §11b). Fixed 2026-08-08;
//!    pinned here so it cannot regress.
//! 2. **`overlap_volume: null`** — the number the doctrine hangs every
//!    must-not-fit claim on was simply absent whenever the operands shared a
//!    flush face pair, which is the *ordinary* case: two bodies overlapping
//!    while both sit on z=0 share coplanar faces
//!    (`jar_top_seed_singulator.md` F6, `folding_book_stand.md` F5). Six live
//!    campaign receipts carry that null today.
//!
//! Every case below is a pair whose true overlap is known in closed form, so
//! the assertions are on numbers, not on "did not crash".

use kernel_api::{run_program, Report};
use serde_json::json;
use std::path::Path;

fn measures<'a>(r: &'a Report, id: &str) -> &'a serde_json::Value {
	r.ops
		.iter()
		.find(|o| o.id == id)
		.unwrap_or_else(|| panic!("op '{id}' missing from the report — {r:#?}"))
		.measures
		.as_ref()
		.unwrap_or_else(|| panic!("op '{id}' produced no measures — {r:#?}"))
}
fn num(r: &Report, id: &str, key: &str) -> Option<f64> {
	measures(r, id).get(key).and_then(|v| v.as_f64())
}
fn flag(r: &Report, id: &str, key: &str) -> Option<bool> {
	measures(r, id).get(key).and_then(|v| v.as_bool())
}
fn text<'a>(r: &'a Report, id: &str, key: &str) -> Option<&'a str> {
	measures(r, id).get(key).and_then(|v| v.as_str())
}
fn run(dir: &Path, ops: serde_json::Value) -> Report {
	run_program(&serde_json::to_string(&json!({ "ops": ops })).unwrap(), dir)
}
fn tmp(tag: &str) -> std::path::PathBuf {
	let d = std::env::temp_dir().join(format!("cadcode_clr_{tag}_{}", std::process::id()));
	std::fs::create_dir_all(&d).unwrap();
	d
}

/// DEFECT 1 — disjoint curved-face pairs must read a positive gap and
/// `interfering: false`. A Ø11.4 pin coaxial inside a Ø12 bore is a 0.300 mm
/// radial gap; a boss beside a bore is a 3.000 mm gap. Both were reported as
/// `distance: 0.0, interfering: true` before 2026-08-08.
#[test]
fn disjoint_curved_pairs_are_not_interfering() {
	let dir = tmp("disjoint");
	let r = run(
		&dir,
		json!([
			// Nested: a tube bored Ø12, a Ø11.4 pin down its axis → 0.300 mm radial gap.
			{"id":"tube_o","op":"cylinder","base":[0,0,0],"axis":[0,0,1],"radius":10,"height":20,"segments":64},
			{"id":"tube_b","op":"cylinder","base":[0,0,-1],"axis":[0,0,1],"radius":6.0,"height":22,"segments":64},
			{"id":"tube","op":"difference","a":"tube_o","b":"tube_b"},
			{"id":"pin","op":"cylinder","base":[0,0,2],"axis":[0,0,1],"radius":5.7,"height":16,"segments":64},
			{"id":"nested","op":"clearance","a":"tube","b":"pin","tol":0.01},
			// The same pair posed obliquely — the arrangement's thinnest corner.
			{"id":"tube_p","op":"pose","in":"tube","rotate":{"axis":[1,0.3,0.2],"degrees":37.0,"center":[0,0,10]}},
			{"id":"pin_p","op":"pose","in":"pin","rotate":{"axis":[1,0.3,0.2],"degrees":37.0,"center":[0,0,10]}},
			{"id":"posed","op":"clearance","a":"tube_p","b":"pin_p","tol":0.01},
			// Side by side: two Ø10 bosses on 13 mm centres → a 3.000 mm gap.
			{"id":"boss_a","op":"cylinder","base":[0,0,0],"axis":[0,0,1],"radius":5,"height":10,"segments":64},
			{"id":"boss_b","op":"cylinder","base":[13,0,0],"axis":[0,0,1],"radius":5,"height":10,"segments":64},
			{"id":"side","op":"clearance","a":"boss_a","b":"boss_b","tol":0.01},
			// The proof the campaigns were forced to hand-roll instead.
			{"id":"nested_disjoint","op":"assert_disjoint","a":"tube","b":"pin","min_clearance":0.05}
		]),
	);
	assert!(r.ok, "the whole disjoint program must run clean — {r:#?}");

	for (id, truth) in [("nested", 0.300_f64), ("posed", 0.300), ("side", 3.000)] {
		let d = num(&r, id, "distance").unwrap_or(-1.0);
		assert_eq!(flag(&r, id, "interfering"), Some(false), "'{id}' is provably disjoint — {r:#?}");
		assert!(d > 0.0, "'{id}' must report a POSITIVE gap, got {d} — {r:#?}");
		// Faceted, so an under-read is expected and is the conservative
		// direction; it must not collapse the gap by more than the facet sagitta.
		assert!(d <= truth + 1e-6 && d >= truth * 0.9, "'{id}' distance {d} must bracket the true {truth} mm gap — {r:#?}");
		assert_eq!(num(&r, id, "overlap_volume"), Some(0.0), "'{id}' does not overlap — {r:#?}");
	}
	let _ = std::fs::remove_dir_all(&dir);
}

/// DEFECT 2 — `overlap_volume` is never a bare `null`. Each pair below shares a
/// flush face pair (both bodies stand on z=0), which is what tripped the
/// `coincident_fit_hazard` early-out into returning nothing at all.
#[test]
fn overlap_volume_is_a_number_on_flush_faced_interference() {
	let dir = tmp("flush");
	let r = run(
		&dir,
		json!([
			// Two 10 mm cubes overlapping by 1 mm in x, coplanar on all four other faces.
			// True overlap = 1 × 10 × 10 = 100 mm³.
			{"id":"ba","op":"box","min":[0,0,0],"max":[10,10,10]},
			{"id":"bb","op":"box","min":[9,0,0],"max":[19,10,10]},
			{"id":"boxes","op":"clearance","a":"ba","b":"bb","tol":0.01},
			// Two Ø10 × 10 cylinders on 8 mm centres, coplanar ends. True lens
			// area = 2r²·acos(d/2r) − (d/2)·√(4r²−d²) = 2·25·acos(0.8) − 4·6 =
			// 32.1750 − 24 = 8.1750 mm² → 81.750 mm³ over the 10 mm height.
			{"id":"ca","op":"cylinder","base":[0,0,0],"axis":[0,0,1],"radius":5,"height":10,"segments":128},
			{"id":"cb","op":"cylinder","base":[8,0,0],"axis":[0,0,1],"radius":5,"height":10,"segments":128},
			{"id":"cyls","op":"clearance","a":"ca","b":"cb","tol":0.01}
		]),
	);
	assert!(r.ok, "clearance never fails on overlap — {r:#?}");

	for (id, truth, tolerance) in [("boxes", 100.0_f64, 1e-6), ("cyls", 81.7503_f64, 0.5)] {
		let m = measures(&r, id);
		assert!(!m["overlap_volume"].is_null(), "'{id}': overlap_volume must never be a bare null — {r:#?}");
		let v = num(&r, id, "overlap_volume").unwrap();
		assert!((v - truth).abs() <= tolerance, "'{id}': overlap_volume {v} must match the closed-form {truth} mm³ — {r:#?}");
		assert_eq!(flag(&r, id, "interfering"), Some(true), "'{id}' really does interfere — {r:#?}");
		// The receipt says WHERE the number came from, always.
		assert!(
			matches!(text(&r, id, "overlap_volume_provenance"), Some("analytic") | Some("faceted")),
			"'{id}' must declare its overlap provenance — {r:#?}"
		);
	}
	let _ = std::fs::remove_dir_all(&dir);
}

/// Contact is not interference. Two cubes sharing a face have zero shared
/// material; the old receipt called that `interfering: true` with a `null`
/// volume, which is a verdict the op could not actually compute.
#[test]
fn face_to_face_contact_is_reported_as_contact_not_interference() {
	let dir = tmp("contact");
	let r = run(
		&dir,
		json!([
			{"id":"a","op":"box","min":[0,0,0],"max":[10,10,10]},
			{"id":"b","op":"box","min":[10,0,0],"max":[20,10,10]},
			{"id":"cl","op":"clearance","a":"a","b":"b","tol":0.01}
		]),
	);
	assert!(r.ok, "clearance measures, it does not refuse — {r:#?}");
	assert_eq!(num(&r, "cl", "distance"), Some(0.0), "abutting faces touch — {r:#?}");
	assert_eq!(num(&r, "cl", "overlap_volume"), Some(0.0), "touching bodies share no material — {r:#?}");
	assert_eq!(flag(&r, "cl", "interfering"), Some(false), "contact is not interference — {r:#?}");
	assert_eq!(flag(&r, "cl", "contact"), Some(true), "…but the touch must still be on the receipt — {r:#?}");
	let _ = std::fs::remove_dir_all(&dir);
}

/// A bound MESH operand carries no exact boolean, but it still has an inside
/// when it closes — so it gets a FACETED number, not a null. `import_mesh` of
/// an exported cube overlapping a solid cube by 1 mm → 100 mm³.
#[test]
fn a_bound_mesh_operand_still_yields_an_overlap_volume() {
	let dir = tmp("meshop");
	let r = run(
		&dir,
		json!([
			{"id":"src","op":"box","min":[9,0,0],"max":[19,10,10]},
			{"id":"stl","op":"export_stl","in":"src","out":"cube.stl"},
			{"id":"m","op":"import_mesh","file":"cube.stl"},
			{"id":"a","op":"box","min":[0,0,0],"max":[10,10,10]},
			{"id":"cl","op":"clearance","a":"a","b":"m","tol":0.01}
		]),
	);
	assert!(r.ok, "a mesh operand is measurable — {r:#?}");
	let m = measures(&r, "cl");
	assert!(!m["overlap_volume"].is_null(), "a CLOSED mesh operand has an inside — {r:#?}");
	let v = num(&r, "cl", "overlap_volume").unwrap();
	assert!((v - 100.0).abs() < 1e-6, "overlap_volume {v} must be the 100 mm³ shared block — {r:#?}");
	assert_eq!(text(&r, "cl", "overlap_volume_provenance"), Some("faceted"), "a mesh operand is faceted — {r:#?}");
	assert!(text(&r, "cl", "overlap_volume_reason").is_some(), "…and says why it is faceted — {r:#?}");
	let _ = std::fs::remove_dir_all(&dir);
}

/// A `null` overlap_volume, where one is genuinely unavoidable, arrives with a
/// NAMED reason and an explicit `unavailable` provenance — never silence.
/// An OPEN mesh has no inside at all, so no volume is definable.
#[test]
fn an_unavailable_overlap_volume_names_its_reason() {
	let dir = tmp("reason");
	// A single triangle: three boundary edges, so "inside" is undefined.
	std::fs::write(
		dir.join("open.stl"),
		"solid open\nfacet normal 0 0 1\nouter loop\nvertex 0 0 5\nvertex 10 0 5\nvertex 10 10 5\nendloop\nendfacet\nendsolid open\n",
	)
	.unwrap();
	let r = run(
		&dir,
		json!([
			{"id":"a","op":"box","min":[0,0,0],"max":[10,10,10]},
			{"id":"open","op":"import_mesh","file":"open.stl"},
			{"id":"cl","op":"clearance","a":"a","b":"open","tol":0.01}
		]),
	);
	assert!(r.ok, "an open operand is still measurable for DISTANCE — {r:#?}");
	let m = measures(&r, "cl");
	assert!(m["overlap_volume"].is_null(), "an open mesh encloses no volume — {r:#?}");
	let reason = text(&r, "cl", "overlap_volume_reason").unwrap_or("");
	assert!(reason.contains("boundary edge"), "a null MUST name its reason, got {reason:?} — {r:#?}");
	assert_eq!(
		text(&r, "cl", "overlap_volume_provenance"),
		Some("unavailable"),
		"an absent measure declares itself absent — {r:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}
