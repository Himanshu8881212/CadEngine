// Copyright (c) LMCAD. Licensed under the MIT License.

//! The in-program assembly surface end to end (assembly audit 2026-07-17,
//! gap 2): ONE JSON program builds parts, instances them, mates them (derived
//! from real B-rep faces AND raw), solves with DOF honesty, measures contacts /
//! interference / mass, exports STL + STEP, and saves a `.lmcasm` that the
//! file pipeline then re-executes — the full loop an agent runs, with no Rust
//! and no hand-authored assembly file.

use std::path::PathBuf;

use kernel_api::{run_program, run_assembly, AsmOptions, OpReport, Report};

fn test_dir(name: &str) -> PathBuf {
	let dir = std::env::temp_dir().join(format!("kernel_api_asmops_{name}_{}", std::process::id()));
	let _ = std::fs::remove_dir_all(&dir);
	std::fs::create_dir_all(&dir).expect("create test dir");
	dir
}

fn entry<'r>(report: &'r Report, id: &str) -> &'r OpReport {
	report.ops.iter().find(|o| o.id == id).unwrap_or_else(|| panic!("no report entry '{id}' in {report:#?}"))
}

/// The flagship: plate + shaft + block. The shaft is seeded 9 mm off and
/// tilted, mated into the plate's bore by a DERIVED axis mate (witness on the
/// real bore wall) + a raw axial distance; the block face-mates onto the plate
/// top — deliberately OVER the bore, so it interferes with the standing shaft
/// and the interference measure must catch it. Receipts must carry the DOF
/// truth (2 free: shaft spin + block spin — asm_mate_face CENTERS the faces
/// centroid-on-centroid, so plane slide is constrained, and the DOF report is
/// what says so).
#[test]
fn one_program_builds_mates_solves_measures_exports_and_saves() {
	let dir = test_dir("full_loop");
	let program = r#"{"ops": [
		{"id": "blank",  "op": "box", "min": [-20, -20, 0], "max": [20, 20, 8]},
		{"id": "bore",   "op": "cylinder", "base": [0, 0, 0], "axis": [0, 0, 1], "radius": 4.0, "height": 8.0},
		{"id": "plate",  "op": "difference", "a": "blank", "b": "bore"},
		{"id": "shaft",  "op": "cylinder", "base": [0, 0, 0], "axis": [0, 0, 1], "radius": 3.8, "height": 20.0},
		{"id": "block",  "op": "box", "min": [0, 0, 0], "max": [10, 10, 10]},

		{"id": "i_plate", "op": "asm_instance", "solid": "plate",
		 "material": {"name": "PLA", "density_g_cm3": 1.24}},
		{"id": "i_shaft", "op": "asm_instance", "solid": "shaft",
		 "translate": [9, -6, 3], "rotate": {"axis": [0, 1, 0], "degrees": 30},
		 "material": {"name": "PETG", "density_g_cm3": 1.27}},
		{"id": "i_block", "op": "asm_instance", "solid": "block", "translate": [25, 25, 30]},

		{"id": "m_axis", "op": "asm_mate_axis",
		 "a": "i_plate", "a_witness": [4, 0, 4], "b": "i_shaft", "b_witness": [3.8, 0, 10]},
		{"id": "m_seat", "op": "asm_mate", "kind": "distance",
		 "a": "i_plate", "a_point": [0, 0, 0], "b": "i_shaft", "b_point": [0, 0, 0], "distance": 4.0},
		{"id": "m_down", "op": "asm_mate", "kind": "coincident",
		 "a": "i_plate", "a_point": [0, 0, 8], "b": "i_block", "b_point": [5, 5, 0]},
		{"id": "m_flat", "op": "asm_mate", "kind": "parallel",
		 "a": "i_plate", "a_dir": [0, 0, 1], "b": "i_block", "b_dir": [0, 0, 1]},

		{"id": "solve", "op": "asm_solve"},
		{"id": "contacts", "op": "asm_contacts", "window": 1.0},
		{"id": "clash", "op": "asm_interference_volume", "a": "i_shaft", "b": "i_block", "voxel": 0.25},
		{"id": "mass", "op": "asm_mass_properties"},
		{"id": "merged", "op": "asm_export", "file": "asm/merged.stl", "parts_dir": "asm/parts"},
		{"id": "step", "op": "asm_export_step", "file": "asm/assembly.step"},
		{"id": "save", "op": "asm_save", "file": "asm/full_loop.lmcasm"}
	]}"#;
	let report = run_program(program, &dir);
	assert!(report.ok, "program must succeed end to end: {report:#?}");

	// -- solve: converged, per-mate ~0, DOF says exactly what remains free.
	let solve = entry(&report, "solve").measures.as_ref().expect("solve measures");
	let shaft_pose = &solve["poses"][1];
	let shaft_xy = (shaft_pose["translation"][0].as_f64().unwrap().powi(2)
		+ shaft_pose["translation"][1].as_f64().unwrap().powi(2))
	.sqrt();
	// -- derived axis mate echoed the REAL bore axis.
	let m_axis = entry(&report, "m_axis").measures.as_ref().expect("m_axis measures");
	// -- contacts: block seats flush on the plate (touching); shaft floats in
	//    the bore at the 0.2 mm radial ring gap.
	let contacts = entry(&report, "contacts").measures.as_ref().expect("contacts measures");
	let pair = |a: &str, b: &str| {
		contacts["pairs"]
			.as_array()
			.unwrap()
			.iter()
			.find(|p| (p["a"] == a && p["b"] == b) || (p["a"] == b && p["b"] == a))
			.cloned()
	};
	let plate_block = pair("i_plate", "i_block").expect("plate/block pair listed");
	let plate_shaft = pair("i_plate", "i_shaft").expect("plate/shaft pair listed");
	// -- interference: the block over the bore REALLY hits the standing shaft.
	let clash = entry(&report, "clash").measures.as_ref().expect("clash measures")["overlap_volume"].as_f64().unwrap();
	// Shaft spans z 4..24; block seats at z 8..18 fully covering the Ø7.6 circle.
	let clash_expect = std::f64::consts::PI * 3.8 * 3.8 * 10.0;
	// -- mass: exact volumes × densities for the two materialed parts, honest
	//    omission for the block.
	let mass = entry(&report, "mass").measures.as_ref().expect("mass measures");
	let plate_mass = mass["instances"][0]["mass_g"].as_f64().unwrap();
	let plate_expect = (40.0 * 40.0 * 8.0 - std::f64::consts::PI * 16.0 * 8.0) * 1.24 / 1000.0;
	// -- exports.
	let step = entry(&report, "step").measures.as_ref().expect("step measures");
	let save = entry(&report, "save").measures.as_ref().expect("save measures");

	assert!(
		solve["converged"] == true
			&& solve["residual"].as_f64().unwrap() < 1e-6
			&& solve["dof"]["verdict"] == "under_constrained (2 free DOF)"
			&& shaft_xy < 1e-3
			&& (shaft_pose["translation"][2].as_f64().unwrap() - 4.0).abs() < 1e-3
			&& m_axis["kind"] == "concentric"
			&& m_axis["derived"]["a"]["axis_dir"][2].as_f64().unwrap().abs() > 0.999
			&& plate_block["touching"] == true
			&& (plate_shaft["distance"].as_f64().unwrap() - 0.2).abs() < 0.06
			&& (clash - clash_expect).abs() / clash_expect < 0.1
			&& (plate_mass - plate_expect).abs() / plate_expect < 1e-6
			&& mass["mass_complete"] == false
			&& mass["instances"][0]["volume_source"] == "exact"
			&& step["parts"] == 3
			&& save["mates"] == 4
			&& save["sources"].as_array().unwrap().iter().all(|s| s["source"] == "mesh"),
		"full-loop receipts:\nsolve {solve:#?}\nm_axis {m_axis:#?}\ncontacts {contacts:#?}\n\
		 clash {clash:.1} (expect ~{clash_expect:.1})\nmass {mass:#?}\nstep {step:#?}\nsave {save:#?}"
	);

	// -- the saved .lmcasm is RE-EXECUTABLE by the file pipeline: mates re-solve
	//    under the gate, contacts scan runs, three mesh-source instances load.
	let asm_report = run_assembly(&dir.join("asm/full_loop.lmcasm"), &dir.join("asm_out"), &AsmOptions::default());
	let mates = entry(&asm_report, "mates");
	assert!(
		asm_report.ok && mates.ok && mates.measures.as_ref().unwrap()["residual"].as_f64().unwrap() < 1e-6,
		"the saved assembly must re-execute through kernel-api asm: {asm_report:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// The kinematics bridge op: PLAN-26 poses (ratio exactly 26, orbit radius
/// module·(S+Pa)/2 = 12) and the loud refusal for a non-assemblable train.
#[test]
fn gear_train_poses_bridge_and_refusal() {
	let dir = test_dir("gear_poses");
	let ok_prog = r#"{"ops": [
		{"id": "poses", "op": "gear_train_poses", "sun_teeth": 12, "ring1_teeth": 36,
		 "planet_a_teeth": 12, "planet_b_teeth": 11, "ring2_teeth": 39, "n_planets": 3,
		 "module": 1.0, "theta_deg": 45.0}
	]}"#;
	let r = run_program(ok_prog, &dir);
	let m = entry(&r, "poses").measures.as_ref().expect("poses measures");
	let planet0 = &m["planets"][0];
	let orbit = (planet0["translation"][0].as_f64().unwrap().powi(2) + planet0["translation"][1].as_f64().unwrap().powi(2)).sqrt();

	let bad_prog = r#"{"ops": [
		{"id": "bad", "op": "gear_train_poses", "sun_teeth": 13, "ring1_teeth": 36,
		 "planet_a_teeth": 12, "planet_b_teeth": 11, "ring2_teeth": 39, "n_planets": 3,
		 "module": 1.0, "theta_deg": 0.0}
	]}"#;
	let rb = run_program(bad_prog, &dir);
	let bad = entry(&rb, "bad");
	let msg = bad.error.as_ref().map(|e| e.message.clone()).unwrap_or_default();

	assert!(
		r.ok
			&& (m["ratio"].as_f64().unwrap() - 26.0).abs() < 1e-9
			&& (m["orbit_radius_mm"].as_f64().unwrap() - 12.0).abs() < 1e-12
			&& (orbit - 12.0).abs() < 1e-9
			&& m["planets"].as_array().unwrap().len() == 3
			&& !rb.ok
			&& msg.contains("odd"),
		"gear bridge: ratio {:?}, orbit {orbit:.6}, refusal '{msg}'",
		m["ratio"]
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// The refusal paths stay loud and actionable: unknown mate kind, an axis
/// witness aimed at a planar face, and an unsatisfiable mate set (which must
/// name its culprits and the DOF verdict).
#[test]
fn asm_ops_refuse_loudly_with_named_causes() {
	let dir = test_dir("refusals");
	let base = r#"
		{"id": "a", "op": "box", "min": [0, 0, 0], "max": [10, 10, 10]},
		{"id": "b", "op": "box", "min": [0, 0, 0], "max": [5, 5, 5]},
		{"id": "ia", "op": "asm_instance", "solid": "a"},
		{"id": "ib", "op": "asm_instance", "solid": "b", "translate": [30, 0, 0]}"#;

	let unknown = run_program(
		&format!(r#"{{"ops": [{base}, {{"id": "m", "op": "asm_mate", "kind": "magnetic", "a": "ia", "b": "ib"}}]}}"#),
		&dir,
	);
	let planar = run_program(
		&format!(
			r#"{{"ops": [{base}, {{"id": "m", "op": "asm_mate_axis", "a": "ia", "a_witness": [5, 5, 10], "b": "ib", "b_witness": [2.5, 2.5, 5]}}]}}"#
		),
		&dir,
	);
	let conflict = run_program(
		&format!(
			r#"{{"ops": [{base},
			 {{"id": "m1", "op": "asm_mate", "kind": "coincident", "a": "ia", "a_point": [0, 0, 0], "b": "ib", "b_point": [0, 0, 0]}},
			 {{"id": "m2", "op": "asm_mate", "kind": "distance", "a": "ia", "a_point": [0, 0, 0], "b": "ib", "b_point": [0, 0, 0], "distance": 6.0}},
			 {{"id": "solve", "op": "asm_solve"}}]}}"#
		),
		&dir,
	);
	let msg = |r: &Report, id: &str| entry(r, id).error.as_ref().map(|e| e.message.clone()).unwrap_or_default();
	let (mu, mp, mc) = (msg(&unknown, "m"), msg(&planar, "m"), msg(&conflict, "solve"));
	assert!(
		!unknown.ok
			&& mu.contains("unknown mate kind 'magnetic'")
			&& !planar.ok
			&& mp.contains("no axis")
			&& mp.contains("list_faces")
			&& !conflict.ok
			&& mc.contains("worst offenders")
			&& mc.contains("did not converge"),
		"refusals must be actionable:\nunknown → {mu}\nplanar → {mp}\nconflict → {mc}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// `asm_mate_face` on clean (unfragmented) faces: two plain boxes seat
/// flush-with-offset, the receipts echo the derived planes, and the gap is the
/// declared offset — measured, not assumed.
#[test]
fn mate_face_seats_clean_faces_at_the_declared_offset() {
	let dir = test_dir("mate_face");
	let program = r#"{"ops": [
		{"id": "base", "op": "box", "min": [-15, -15, 0], "max": [15, 15, 10]},
		{"id": "lid",  "op": "box", "min": [0, 0, 0], "max": [12, 12, 4]},
		{"id": "i_base", "op": "asm_instance", "solid": "base"},
		{"id": "i_lid",  "op": "asm_instance", "solid": "lid", "translate": [40, 9, 22], "rotate": {"axis": [1, 0, 0], "degrees": 25}},
		{"id": "m", "op": "asm_mate_face", "a": "i_base", "a_witness": [0, 0, 10], "b": "i_lid", "b_witness": [6, 6, 0], "offset": 0.5},
		{"id": "solve", "op": "asm_solve"},
		{"id": "contacts", "op": "asm_contacts", "window": 1.0}
	]}"#;
	let report = run_program(program, &dir);
	assert!(report.ok, "mate_face program must succeed: {report:#?}");
	let m = entry(&report, "m").measures.as_ref().expect("m measures");
	let solve = entry(&report, "solve").measures.as_ref().expect("solve measures");
	let lid_z = solve["poses"][1]["translation"][2].as_f64().unwrap();
	let contacts = entry(&report, "contacts").measures.as_ref().expect("contacts");
	let gap = contacts["pairs"][0]["distance"].as_f64().unwrap();
	assert!(
		m["derived"]["a"]["normal"][2].as_f64().unwrap() > 0.999
			&& solve["converged"] == true
			&& (lid_z - 10.5).abs() < 1e-3
			&& (gap - 0.5).abs() < 0.02,
		"clean-face seat: derived {m:#?}, lid z {lid_z:.4} (want 10.5), measured gap {gap:.4} (want 0.5)"
	);
	let _ = std::fs::remove_dir_all(&dir);
}
