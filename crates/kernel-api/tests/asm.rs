// Copyright (c) LMCAD. Licensed under the MIT License.

//! End-to-end tests of the `.lmcasm` executable surface (`kernel-api asm`,
//! FRICTION.md #1): a fixture assembly written through the public format API is
//! loaded, mate-checked, BOM'd, exported (merged / per-instance / per-state)
//! and contact-scanned, all through [`run_assembly`] and the CLI binary — and
//! the contact scan must SEE B-rep-only parts (FRICTION.md #2).

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use kernel_api::{run_assembly, AsmOptions, ErrorKind, OpReport, Report};
use kernel_core::math::{Affine3A, Vec3};
use kernel_model::format::{save_assembly_with_states, save_part, AsmInstance, AsmSource};
use kernel_model::{AsmState, CatalogPart, Dim, Document, Feature};

/// A unique per-test directory under the system temp dir.
fn test_dir(name: &str) -> PathBuf {
	let dir = std::env::temp_dir().join(format!("kernel_api_asm_{name}_{}", std::process::id()));
	let _ = std::fs::remove_dir_all(&dir);
	std::fs::create_dir_all(&dir).expect("create test dir");
	dir
}

/// The report entry whose id is `id` (panics with the report when absent).
fn entry<'r>(report: &'r Report, id: &str) -> &'r OpReport {
	report.ops.iter().find(|o| o.id == id).unwrap_or_else(|| panic!("no report entry '{id}' in {report:#?}"))
}

/// True when `path` exists and is non-empty.
fn file_ok(path: &Path) -> bool {
	std::fs::metadata(path).map(|m| m.len() > 0).unwrap_or(false)
}

/// Write the fixture: a 60×60×10 plate (top face at z = 0) and a Ø8×20 catalog
/// shaft standing on it (touching at z = 0), plus a suppressed spare shaft and
/// an "exploded" state lifting the shaft by 40. Returns the `.lmcasm` path.
fn write_fixture(dir: &Path) -> PathBuf {
	let mut plate = Document::new();
	let b = plate.add(Feature::Box {
		center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(-5.0)],
		size: [Dim::Literal(60.0), Dim::Literal(60.0), Dim::Literal(10.0)],
	});
	plate.set_root(b);
	let mut shaft = Document::new();
	let s = shaft.add(Feature::CatalogPart {
		part: CatalogPart::Shaft { d: Dim::Literal(8.0), length: Dim::Literal(20.0) },
	});
	shaft.set_root(s);
	std::fs::write(dir.join("plate.lmcpart"), save_part(&plate, "plate")).expect("write plate");
	std::fs::write(dir.join("shaft.lmcpart"), save_part(&shaft, "shaft")).expect("write shaft");

	let instances = vec![
		AsmInstance {
			name: Some("plate".to_string()),
			source: AsmSource::Path("plate.lmcpart".to_string()),
			pose: Affine3A::IDENTITY,
			suppressed: false,
		},
		AsmInstance {
			name: Some("pin".to_string()),
			source: AsmSource::Path("shaft.lmcpart".to_string()),
			pose: Affine3A::IDENTITY,
			suppressed: false,
		},
		AsmInstance {
			name: Some("spare".to_string()),
			source: AsmSource::Path("shaft.lmcpart".to_string()),
			pose: Affine3A::from_translation(Vec3::new(100.0, 0.0, 0.0)),
			suppressed: true,
		},
	];
	let mut states = BTreeMap::new();
	states.insert(
		"exploded".to_string(),
		AsmState {
			poses: vec![
				Affine3A::IDENTITY,
				Affine3A::from_translation(Vec3::new(0.0, 0.0, 40.0)),
				Affine3A::from_translation(Vec3::new(100.0, 0.0, 0.0)),
			],
			suppressed: vec![2],
		},
	);
	let text = save_assembly_with_states("pin_on_plate", &instances, &[], &states).expect("serialize fixture assembly");
	let path = dir.join("pin_on_plate.lmcasm");
	std::fs::write(&path, text).expect("write .lmcasm");
	path
}

/// (1) The full pipeline on the fixture: load + mates + BOM + per-instance /
/// merged / state exports + the contact scan that SEES the B-rep-only catalog
/// shaft touching the plate — the exact class of contact the assembly APIs used
/// to miss silently (FRICTION #2), now reachable end-to-end from a `.lmcasm`
/// file through the official surface (FRICTION #1).
#[test]
fn asm_pipeline_end_to_end() {
	let dir = test_dir("pipeline");
	let asm_path = write_fixture(&dir);
	let out = dir.join("out");
	let report = run_assembly(&asm_path, &out, &AsmOptions::default());

	let load = entry(&report, "load").measures.clone().unwrap_or_default();
	let mates = entry(&report, "mates").measures.clone().unwrap_or_default();
	let bom = entry(&report, "bom").measures.clone().unwrap_or_default();
	let plate_export = entry(&report, "export:00:plate").measures.clone().unwrap_or_default();
	let spare_export = entry(&report, "export:02:spare").measures.clone().unwrap_or_default();
	let merged = entry(&report, "export:assembly").measures.clone().unwrap_or_default();
	let contacts = entry(&report, "contacts").measures.clone().unwrap_or_default();
	let state = entry(&report, "state:exploded").measures.clone().unwrap_or_default();

	let bom_counts: Vec<(String, u64)> = bom["flat"]
		.as_array()
		.map(|lines| {
			lines
				.iter()
				.map(|l| (l["name"].as_str().unwrap_or("?").to_string(), l["count"].as_u64().unwrap_or(0)))
				.collect()
		})
		.unwrap_or_default();
	let touching_pair = contacts["pairs"].as_array().and_then(|p| p.first()).cloned().unwrap_or_default();

	assert!(
		report.ok
			&& load["instances"] == 3
			&& load["top_level"] == 3
			&& load["states"] == serde_json::json!(["exploded"])
			&& mates["residual"] == 0.0
			&& bom["schema"] == "bom/2"
			&& bom_counts == vec![("plate".to_string(), 1), ("shaft".to_string(), 1)]
			&& plate_export["route"] == "exact"
			&& spare_export["suppressed"] == true
			&& merged["triangles"].as_u64().unwrap_or(0) > 0
			&& merged["watertight"] == true
			&& contacts["touching"] == 1
			&& touching_pair["a"] == "plate"
			&& touching_pair["b"] == "pin"
			&& touching_pair["distance"].as_f64().unwrap_or(f64::NAN) <= 1e-6
			&& state["triangles"].as_u64().unwrap_or(0) > 0
			&& file_ok(&out.join("bom.json"))
			&& file_ok(&out.join("bom.csv"))
			&& file_ok(&out.join("parts/00_plate.stl"))
			&& file_ok(&out.join("parts/01_pin.stl"))
			&& !out.join("parts/02_spare.stl").exists()
			&& file_ok(&out.join("pin_on_plate_assembly.stl"))
			&& file_ok(&out.join("pin_on_plate_state_exploded.stl")),
		"asm pipeline: bom={bom_counts:?} contacts={contacts} report={report:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// A box document: center `(cx, cy, cz)`, size `(sx, sy, sz)`.
fn box_doc(center: [f64; 3], size: [f64; 3]) -> Document {
	let mut doc = Document::new();
	let b = doc.add(Feature::Box {
		center: [Dim::Literal(center[0]), Dim::Literal(center[1]), Dim::Literal(center[2])],
		size: [Dim::Literal(size[0]), Dim::Literal(size[1]), Dim::Literal(size[2])],
	});
	doc.set_root(b);
	doc
}

/// (1b) The nested pipeline (assembly nesting + BOM v2): a sub-assembly
/// (`asm_path`) of a foot+head stack placed on a plate. The runner must flatten
/// it to leaf exports with HIERARCHICAL names, find the designed contacts both
/// ACROSS levels (plate↔stack/foot) and INSIDE the sub-assembly
/// (stack/foot↔stack/head), report the BOM v2 payload (schema, tree rollup,
/// mass enrichment from part `meta`), write bom.csv alongside bom.json — and
/// two runs must produce byte-identical bom.json (determinism).
#[test]
fn asm_nested_pipeline_hierarchical_contacts_bom_v2_and_determinism() {
	let dir = test_dir("nested");
	let steel = kernel_model::format::PartBomMeta {
		part_number: Some("PLT-9".to_string()),
		material: Some(kernel_model::format::Material { name: "steel".to_string(), density_g_cm3: 7.85 }),
		make_or_buy: Some(kernel_model::format::MakeOrBuy::Make),
	};
	let plate = box_doc([0.0, 0.0, -5.0], [60.0, 60.0, 10.0]); // top face at z = 0
	std::fs::write(dir.join("plate.lmcpart"), kernel_model::format::save_part_with_meta(&plate, "plate", Some(&steel)))
		.expect("write plate");
	std::fs::write(dir.join("foot.lmcpart"), save_part(&box_doc([0.0, 0.0, 5.0], [10.0, 10.0, 10.0]), "foot")).expect("write foot");
	std::fs::write(dir.join("head.lmcpart"), save_part(&box_doc([0.0, 0.0, 13.0], [6.0, 6.0, 6.0]), "head")).expect("write head");
	let stack = kernel_model::format::save_assembly(
		"stack",
		&[
			AsmInstance { name: Some("foot".to_string()), source: AsmSource::Path("foot.lmcpart".to_string()), pose: Affine3A::IDENTITY, suppressed: false },
			AsmInstance { name: Some("head".to_string()), source: AsmSource::Path("head.lmcpart".to_string()), pose: Affine3A::IDENTITY, suppressed: false },
		],
		&[],
	)
	.expect("stack saves");
	std::fs::write(dir.join("stack.lmcasm"), stack).expect("write stack.lmcasm");
	let top = kernel_model::format::save_assembly(
		"tower",
		&[
			AsmInstance { name: Some("plate".to_string()), source: AsmSource::Path("plate.lmcpart".to_string()), pose: Affine3A::IDENTITY, suppressed: false },
			AsmInstance {
				name: Some("stack".to_string()),
				source: AsmSource::Assembly("stack.lmcasm".to_string()),
				pose: Affine3A::from_translation(Vec3::new(10.0, 5.0, 0.0)),
				suppressed: false,
			},
		],
		&[],
	)
	.expect("tower saves");
	let asm_path = dir.join("tower.lmcasm");
	std::fs::write(&asm_path, top).expect("write tower.lmcasm");

	let out = dir.join("out");
	let report = run_assembly(&asm_path, &out, &AsmOptions::default());
	let report2 = run_assembly(&asm_path, &dir.join("out2"), &AsmOptions::default());

	let load = entry(&report, "load").measures.clone().unwrap_or_default();
	let bom = entry(&report, "bom").measures.clone().unwrap_or_default();
	let foot_export = entry(&report, "export:01:stack/foot").measures.clone().unwrap_or_default();
	let contacts = entry(&report, "contacts").measures.clone().unwrap_or_default();
	let touching: Vec<(String, String)> = contacts["pairs"]
		.as_array()
		.map(|pairs| {
			pairs
				.iter()
				.filter(|p| p["touching"] == true)
				.map(|p| (p["a"].as_str().unwrap_or("?").to_string(), p["b"].as_str().unwrap_or("?").to_string()))
				.collect()
		})
		.unwrap_or_default();
	let tree = bom["tree"].as_array().cloned().unwrap_or_default();
	let plate_line = bom["flat"]
		.as_array()
		.and_then(|lines| lines.iter().find(|l| l["name"] == "plate").cloned())
		.unwrap_or_default();
	let bom_bytes = std::fs::read(out.join("bom.json")).expect("bom.json written");
	let bom_bytes2 = std::fs::read(dir.join("out2").join("bom.json")).expect("second bom.json written");
	let csv = std::fs::read_to_string(out.join("bom.csv")).expect("bom.csv written");

	assert!(
		report.ok
			&& report2.ok
			&& load["instances"] == 3
			&& load["top_level"] == 2
			&& foot_export["route"] == "exact"
			&& foot_export["part"] == "foot"
			&& touching == vec![("plate".to_string(), "stack/foot".to_string()), ("stack/foot".to_string(), "stack/head".to_string())]
			&& contacts["touching"] == 2
			&& bom["schema"] == "bom/2"
			&& tree.len() == 2
			&& tree[0]["name"] == "plate"
			&& tree[0]["count"] == 1
			&& tree[1]["instance"] == "stack"
			&& tree[1]["count"] == 2
			&& tree[1]["children"].as_array().map(Vec::len) == Some(2)
			// plate meta flows through: steel density × exact 36 cm³ = 282.6 g
			&& plate_line["part_number"] == "PLT-9"
			&& plate_line["volume_source"] == "exact"
			&& (plate_line["unit_mass_g"].as_f64().unwrap_or(f64::NAN) - 282.6).abs() < 1e-6
			&& csv.starts_with("name,count,params,part_number,material,density_g_cm3,volume_source,unit_mass_g,line_mass_g,make_or_buy\n")
			&& bom_bytes == bom_bytes2
			&& file_ok(&out.join("parts/01_stack_foot.stl"))
			&& file_ok(&out.join("parts/02_stack_head.stl")),
		"nested asm pipeline: touching={touching:?} bom={bom:#?} report={report:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (2) Loud failure paths: a missing part source fails the `load` step with a
/// structured error (no half-loaded assembly), and a too-loose mate residual is
/// an `assert_failed` — both under the exit-`ok` contract.
#[test]
fn asm_failure_paths_are_structured() {
	let dir = test_dir("failures");
	let asm_text = r#"{
		"format": "lmc-asm", "version": 1, "units": "mm", "name": "broken",
		"instances": [{"name": "ghost", "source": {"path": "missing.lmcpart"}, "pose": {"translation": [0, 0, 0]}}],
		"mates": []
	}"#;
	let path = dir.join("broken.lmcasm");
	std::fs::write(&path, asm_text).expect("write broken .lmcasm");
	let report = run_assembly(&path, &dir.join("out"), &AsmOptions::default());
	let load = &report.ops[0];
	assert!(
		!report.ok
			&& load.id == "load"
			&& load.error.as_ref().map(|e| e.kind) == Some(ErrorKind::InvalidParam)
			&& load.error.as_ref().is_some_and(|e| e.message.contains("missing.lmcpart")),
		"missing part source must fail the load step loudly: {report:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (3) The CLI contract for the `asm` subcommand: report JSON on stdout, exit 0
/// on success, exit 2 with usage on a bad invocation.
#[test]
fn asm_cli_contract() {
	let dir = test_dir("cli");
	let asm_path = write_fixture(&dir);
	let out = dir.join("cli_out");

	let output = std::process::Command::new(env!("CARGO_BIN_EXE_kernel-api"))
		.args(["asm", &asm_path.display().to_string(), "--out-dir", &out.display().to_string(), "--window", "2.0"])
		.output()
		.expect("spawn kernel-api asm");
	let report: Report = serde_json::from_slice(&output.stdout).expect("stdout must be a JSON report");
	let usage = std::process::Command::new(env!("CARGO_BIN_EXE_kernel-api"))
		.args(["frobnicate"])
		.output()
		.expect("spawn kernel-api with bad subcommand");
	assert!(
		output.status.code() == Some(0)
			&& report.ok
			&& entry(&report, "contacts").measures.as_ref().is_some_and(|m| m["window"] == 2.0)
			&& file_ok(&out.join("pin_on_plate_assembly.stl"))
			&& usage.status.code() == Some(2),
		"asm CLI contract: exit={:?} usage_exit={:?} report={report:#?}",
		output.status.code(),
		usage.status.code()
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (4) REAL mates through the pipeline (assembly audit 2026-07-17, gap 9: the
/// old fixtures carried empty mate lists, so `residual == 0` was trivial and
/// the 1e-6 gate was never exercised). A shaft seeded 9 mm off-axis and tilted
/// 45° must be pulled concentric into the plate's frame by the on-load solve;
/// the receipts must carry per-mate residuals and a DOF report that HONESTLY
/// says the assembly is under-constrained (spin + slide remain).
#[test]
fn asm_pipeline_solves_real_mates_with_dof_and_per_mate_receipts() {
	use kernel_core::math::DVec3;
	use kernel_model::Constraint;

	let dir = test_dir("real_mates");
	let mut plate = Document::new();
	let b = plate.add(Feature::Box {
		center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(-5.0)],
		size: [Dim::Literal(60.0), Dim::Literal(60.0), Dim::Literal(10.0)],
	});
	plate.set_root(b);
	let mut shaft = Document::new();
	let s = shaft.add(Feature::CatalogPart {
		part: CatalogPart::Shaft { d: Dim::Literal(8.0), length: Dim::Literal(20.0) },
	});
	shaft.set_root(s);
	std::fs::write(dir.join("plate.lmcpart"), save_part(&plate, "plate")).expect("write plate");
	std::fs::write(dir.join("shaft.lmcpart"), save_part(&shaft, "shaft")).expect("write shaft");
	let instances = vec![
		AsmInstance {
			name: Some("plate".to_string()),
			source: AsmSource::Path("plate.lmcpart".to_string()),
			pose: Affine3A::IDENTITY,
			suppressed: false,
		},
		AsmInstance {
			name: Some("pin".to_string()),
			source: AsmSource::Path("shaft.lmcpart".to_string()),
			pose: Affine3A::from_rotation_translation(
				kernel_core::math::Quat::from_axis_angle(Vec3::new(0.0, 1.0, 0.0), 0.785),
				Vec3::new(9.0, -6.0, 2.0),
			),
			suppressed: false,
		},
	];
	let mates = vec![Constraint::Concentric {
		a: 0,
		a_axis_point: DVec3::ZERO,
		a_axis_dir: DVec3::Z,
		b: 1,
		b_axis_point: DVec3::ZERO,
		b_axis_dir: DVec3::Z,
	}];
	let text = save_assembly_with_states("mated", &instances, &mates, &BTreeMap::new()).expect("serialize");
	let asm_path = dir.join("mated.lmcasm");
	std::fs::write(&asm_path, text).expect("write .lmcasm");

	let out = dir.join("out");
	let report = run_assembly(&asm_path, &out, &AsmOptions::default());
	let mates_op = entry(&report, "mates");
	let m = mates_op.measures.as_ref().expect("mates measures");
	let per_mate_ok = m["per_mate"][0]["kind"] == "concentric" && m["per_mate"][0]["residual"].as_f64().unwrap() < 1e-6;
	let dof_verdict = m["dof"]["verdict"].as_str().unwrap_or("");
	let failed: Vec<&OpReport> = report.ops.iter().filter(|o| !o.ok).collect();
	assert!(
		report.ok
			&& mates_op.ok
			&& m["residual"].as_f64().unwrap() < 1e-6
			&& per_mate_ok
			&& dof_verdict == "under_constrained (2 free DOF)"
			&& m["dof"]["rank"] == 4,
		"real-mate pipeline must solve the bad seed AND say what remains free:\n\
		 residual {:?}, per_mate {:?}, dof {:?}\n(want residual <1e-6, concentric per-mate ~0, \
		 under_constrained (2 free DOF) — spin + axial slide are honestly unmated)\nfailed ops: {failed:#?}",
		m["residual"],
		m["per_mate"],
		m["dof"]
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (5) The 1e-6 mate gate FAILS LOUDLY on an unsatisfiable mate set, naming the
/// worst offender — and a statically broken mate (index out of range) refuses
/// as `invalid_param` instead of being silently skipped (the old behavior).
#[test]
fn asm_pipeline_refuses_conflicting_and_statically_broken_mates() {
	let dir = test_dir("bad_mates");
	let asm_path = write_fixture(&dir); // plate + pin + spare, no mates
	let base: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(&asm_path).unwrap()).unwrap();

	// (a) Conflicting: Coincident at 0 vs Distance 4 between the same points.
	let mut conflict = base.clone();
	conflict["mates"] = serde_json::json!([
		{"Coincident": {"a": 0, "a_point": [0.0, 0.0, 0.0], "b": 1, "b_point": [0.0, 0.0, 0.0]}},
		{"Distance": {"a": 0, "a_point": [0.0, 0.0, 0.0], "b": 1, "b_point": [0.0, 0.0, 0.0], "distance": 4.0}}
	]);
	let conflict_path = dir.join("conflict.lmcasm");
	std::fs::write(&conflict_path, serde_json::to_string(&conflict).unwrap()).unwrap();
	let r1 = run_assembly(&conflict_path, &dir.join("out1"), &AsmOptions::default());
	let mates1 = entry(&r1, "mates");
	let msg1 = mates1.error.as_ref().map(|e| e.message.clone()).unwrap_or_default();

	// (b) Statically broken: an out-of-range instance index.
	let mut broken = base;
	broken["mates"] = serde_json::json!([
		{"Coincident": {"a": 0, "a_point": [0.0, 0.0, 0.0], "b": 99, "b_point": [0.0, 0.0, 0.0]}}
	]);
	let broken_path = dir.join("broken.lmcasm");
	std::fs::write(&broken_path, serde_json::to_string(&broken).unwrap()).unwrap();
	let r2 = run_assembly(&broken_path, &dir.join("out2"), &AsmOptions::default());
	let mates2 = entry(&r2, "mates");
	let msg2 = mates2.error.as_ref().map(|e| e.message.clone()).unwrap_or_default();

	assert!(
		!r1.ok
			&& !mates1.ok
			&& mates1.error.as_ref().map(|e| e.kind) == Some(ErrorKind::AssertFailed)
			&& msg1.contains("worst offenders")
			&& (msg1.contains("(coincident)") || msg1.contains("(distance)"))
			&& !r2.ok
			&& !mates2.ok
			&& mates2.error.as_ref().map(|e| e.kind) == Some(ErrorKind::InvalidParam)
			&& msg2.contains("out of range"),
		"bad mates must fail LOUDLY with named culprits:\nconflict → {msg1}\nbroken → {msg2}\n{r1:#?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// (6) `export:assembly_step` writes the AP214 assembly for B-rep instances at
/// their solved poses and round-trips volume through the kernel's own importer.
#[test]
fn asm_pipeline_exports_step_assembly_for_brep_instances() {
	let dir = test_dir("step_export");
	let asm_path = write_fixture(&dir);
	let out = dir.join("out");
	let report = run_assembly(&asm_path, &out, &AsmOptions::default());
	let step_op = entry(&report, "export:assembly_step");
	let step_file = out.join("pin_on_plate_assembly.step");
	let text = std::fs::read_to_string(&step_file).unwrap_or_default();
	let round = kernel_brep::import_step_assembly(&text).expect("kernel's own STEP assembly importer reads it back");
	let vol: f64 = round.iter().map(|(_, solid, pose)| kernel_brep::volume(&solid.transformed(*pose))).sum();
	let expect = 60.0 * 60.0 * 10.0 + std::f64::consts::PI * 16.0 * 20.0; // plate + Ø8×20 pin (spare suppressed)
	assert!(
		report.ok
			&& step_op.ok
			&& step_op.measures.as_ref().is_some_and(|m| m["parts"] == 2)
			&& (vol - expect).abs() / expect < 0.01,
		"STEP assembly must round-trip: op {step_op:#?}, round-trip volume {vol:.1} vs expected {expect:.1}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}
