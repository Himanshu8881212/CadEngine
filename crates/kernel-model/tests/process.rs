// Copyright (c) LMCAD. Licensed under the MIT License.

//! Pinning tests for `kernel_model::process` — the FDM profile layer and its
//! calibration pipeline: conservative defaults pinned WITH their provenance,
//! byte-stable JSON, fit helpers reproducing the frozen campaign consts,
//! sibling-process refusals provoked, and `tools/ingest_calibration.py`
//! exercised end-to-end (self-test + a synthetic ingest loaded back through
//! the Rust schema) whenever `python3` is available — skipped with a LOUD
//! banner, never silently, when it is not.

use kernel_brep::math::DVec3;
use kernel_brep::cuboid;
use kernel_model::process::{coupons, DfmFinding, FdmProfile, Process, ProcessError, HOLE_BORE_CROSSOVER_D};
use std::process::Command;

// ---- profile values + schema -----------------------------------------------------

#[test]
fn conservative_default_values_pinned_with_provenance() {
	let p = FdmProfile::conservative_default();
	let pinned: [(&str, f64, f64, &str); 13] = [
		("xy_clearance_tight", p.xy_clearance_tight, 0.05, "DRYBOX press stub: Ø7.9 seat (STUB_R 3.95) in the 608's Ø8.0 bore"),
		("xy_clearance_free", p.xy_clearance_free, 0.25, "RESPOOL C_R (tongue↔wall twist fit), DESIGN_GUIDE §22.6 band 0.2–0.3"),
		("z_clearance", p.z_clearance, 0.3, "RESPOOL CEIL_CLR (lug face ↔ pocket ceiling, axial)"),
		("hole_diameter_comp", p.hole_diameter_comp, 0.0, "frozen campaigns cut holes at nominal; shrink absorbed by clearance"),
		("bore_comp", p.bore_comp, 0.0, "DRYBOX seats the 608 without scaling"),
		("first_layer_comp", p.first_layer_comp, 0.0, "no frozen campaign compensates elephant foot explicitly"),
		("seam_allowance", p.seam_allowance, 0.0, "RESPOOL C_R absorbs the seam inside its 0.25"),
		("max_bridge", p.max_bridge, 6.0, "RESPOOL emit gate max_bridge_span <= 6.0 (DRYBOX's 10.5 is the looser precedent)"),
		("max_unsupported_angle", p.max_unsupported_angle, 45.0, "support_free_report(Z, 45.0, 0.3) in every campaign"),
		("min_wall", p.min_wall, 1.2, "DRYBOX RIB_T — thinnest wall a frozen campaign ships"),
		("bed_x", p.bed_x, 250.0, "RESPOOL/DRYBOX emit bed-fit gate 250×250×220"),
		("bed_y", p.bed_y, 250.0, "RESPOOL/DRYBOX emit bed-fit gate 250×250×220"),
		("bed_z", p.bed_z, 220.0, "RESPOOL/DRYBOX emit bed-fit gate 250×250×220"),
	];
	for (field, got, want, why) in pinned {
		assert!(
			(got - want).abs() < 1e-12,
			"conservative_default.{field} = {got}, pinned at {want} — this value is FROZEN provenance ({why}); changing it means the fallback no longer matches what was proven in print"
		);
	}
	assert_eq!(
		p.name, "conservative_default",
		"fallback profile must be named for what it is, got '{}'",
		p.name
	);
	p.validate().expect("the conservative default must pass its own range validation");
}

#[test]
fn profile_json_snapshot_byte_stable() {
	let p = FdmProfile::conservative_default();
	let snapshot = r#"{
  "name": "conservative_default",
  "xy_clearance_tight": 0.05,
  "xy_clearance_free": 0.25,
  "z_clearance": 0.3,
  "hole_diameter_comp": 0.0,
  "bore_comp": 0.0,
  "first_layer_comp": 0.0,
  "seam_allowance": 0.0,
  "max_bridge": 6.0,
  "max_unsupported_angle": 45.0,
  "min_wall": 1.2,
  "bed_x": 250.0,
  "bed_y": 250.0,
  "bed_z": 220.0
}
"#;
	assert_eq!(
		p.to_json(),
		snapshot,
		"FdmProfile::to_json byte-format drifted — data/profiles/*.json are diffable deliverables and tools/ingest_calibration.py writes the same shape; a format change must be deliberate and synchronized"
	);
	let back = FdmProfile::from_json(&p.to_json()).expect("canonical JSON must reload");
	assert_eq!(back, p, "save → load must round-trip every field bit-exactly (float_roundtrip discipline)");
	assert_eq!(back.to_json(), p.to_json(), "load → save must reproduce identical bytes");
}

#[test]
fn profile_schema_refuses_unknown_missing_and_insane() {
	let p = FdmProfile::conservative_default();
	// Unknown field (a typo) must refuse — silent default fallback is the footgun.
	let typo = p.to_json().replace("xy_clearance_free", "xy_clearence_free");
	let e = FdmProfile::from_json(&typo).expect_err("typo'd field name must refuse");
	assert!(
		matches!(e, ProcessError::Schema { .. }),
		"typo refusal should be a Schema error, got: {e}"
	);
	// Missing field must refuse (no serde defaults on the schema).
	let missing = p.to_json().replace("  \"min_wall\": 1.2,\n", "");
	assert!(
		FdmProfile::from_json(&missing).is_err(),
		"a profile missing min_wall must refuse to load"
	);
	// Range violation must refuse with the offending field named.
	let mut bad = p.clone();
	bad.min_wall = -0.4;
	let e = bad.validate().expect_err("negative min_wall must refuse");
	assert!(
		e.to_string().contains("min_wall") && e.to_string().contains("-0.4"),
		"range refusal must name field and value, got: {e}"
	);
	// tight > free is inconsistent.
	let mut swapped = p.clone();
	swapped.xy_clearance_tight = 0.5;
	assert!(
		swapped.validate().is_err(),
		"tight clearance 0.5 > free 0.25 must refuse"
	);
	// Placeholder names must never become profile files.
	let mut anon = p.clone();
	anon.name = "PLACEHOLDER_RENAME_ME".into();
	assert!(
		matches!(anon.validate(), Err(ProcessError::BadName(_))),
		"placeholder profile names must refuse"
	);
}

// ---- fit helpers -----------------------------------------------------------------

#[test]
fn fit_helpers_pinned_on_known_cases() {
	let p = FdmProfile::conservative_default();
	// The two frozen campaign consts the helpers must reproduce exactly:
	let r_to = p.fit_free_shaft_r(37.3);
	assert!(
		(r_to - 37.05).abs() < 1e-12,
		"fit_free_shaft_r(37.3) = {r_to}, want 37.05 — RESPOOL's R_TO = RI − C_R with RI 37.3, C_R 0.25; the helper IS that formula as data"
	);
	let stub = p.fit_tight_shaft_r(4.0);
	assert!(
		(stub - 3.95).abs() < 1e-12,
		"fit_tight_shaft_r(4.0) = {stub}, want 3.95 — DRYBOX's STUB_R for the 608's 4.0 mm bore radius"
	);
	// Bore-side recommendations for the coupon pin:
	assert!(
		(p.fit_tight_bore_d(6.0) - 6.1).abs() < 1e-12 && (p.fit_free_bore_d(6.0) - 6.5).abs() < 1e-12,
		"conservative bores for a Ø6 shaft: tight {} (want 6.1), free {} (want 6.5) — both must sit on the printed fit ladder",
		p.fit_tight_bore_d(6.0),
		p.fit_free_bore_d(6.0)
	);
	// A measured-style profile exercises compensation + seam arithmetic:
	let mut m = p.clone();
	m.name = "measured_example".into();
	m.hole_diameter_comp = 0.2;
	m.bore_comp = 0.1;
	m.seam_allowance = 0.08;
	assert!(
		(m.hole_d(3.0) - 3.2).abs() < 1e-12,
		"hole_d(3.0) with comp 0.2 = {}, want 3.2 (small-hole class below crossover {HOLE_BORE_CROSSOVER_D})",
		m.hole_d(3.0)
	);
	assert!(
		(m.hole_d(22.0) - 22.1).abs() < 1e-12,
		"hole_d(22.0) with bore comp 0.1 = {}, want 22.1 (large-bore class at/above crossover {HOLE_BORE_CROSSOVER_D})",
		m.hole_d(22.0)
	);
	assert!(
		(m.fit_free_bore_d(6.0) - 6.86).abs() < 1e-12,
		"fit_free_bore_d(6.0) = {} — want 6.0 + 2·(0.25 free + 0.08 seam) + 0.2 comp = 6.86",
		m.fit_free_bore_d(6.0)
	);
	assert!(
		(m.fit_free_shaft_r(10.0) - 9.67).abs() < 1e-12,
		"fit_free_shaft_r(10.0) = {} — want 10 − 0.25 − 0.08 = 9.67 (shafts carry no diametral comp; seam budgeted once)",
		m.fit_free_shaft_r(10.0)
	);
	// Envelope helpers:
	assert!(
		p.bridge_ok(6.0) && !p.bridge_ok(6.01) && p.wall_ok(1.2) && !p.wall_ok(1.19),
		"bridge_ok/wall_ok must be inclusive at the profile limits (bridge 6.0 ok, 6.01 not; wall 1.2 ok, 1.19 not)"
	);
	assert!(
		p.bed_fits([250.0, 250.0, 220.0]) && !p.bed_fits([250.1, 10.0, 10.0]),
		"bed_fits must be inclusive at 250×250×220 and refuse 250.1"
	);
}

// ---- sibling refusals ------------------------------------------------------------

#[test]
fn sibling_processes_refuse_loudly() {
	let plate = cuboid(DVec3::new(0.0, 0.0, 0.0), DVec3::new(10.0, 10.0, 5.0));
	for sib in [Process::SheetMetal, Process::Casting, Process::Cnc] {
		let name = sib.name();
		let e = sib.fdm_profile().expect_err("sibling must refuse fdm_profile()");
		let msg = e.to_string();
		assert!(
			msg.contains(name) && msg.contains("not implemented") && msg.contains("declared sibling"),
			"{name} refusal must name itself, say 'not implemented' and 'declared sibling', got: {msg}"
		);
		let e2 = sib.dfm_checks(&plate).expect_err("sibling must refuse dfm_checks()");
		assert!(
			e2.to_string().contains("not implemented"),
			"{name} dfm_checks refusal must be loud, got: {e2}"
		);
	}
	// Casting's refusal must point at the piece that DOES exist today.
	let msg = Process::Casting.fdm_profile().expect_err("casting refuses").to_string();
	assert!(
		msg.contains("draft_analysis"),
		"casting refusal must direct callers to kernel_brep::draft_analysis (the existing castability half), got: {msg}"
	);
	assert_eq!(
		Process::Fdm(FdmProfile::conservative_default()).name(),
		"fdm",
		"process names are stable identifiers"
	);
}

// ---- DFM checks ------------------------------------------------------------------

#[test]
fn dfm_checks_flag_defects_and_pass_good_parts() {
	let p = FdmProfile::conservative_default();
	// A 20×20×0.8 plate sits under min_wall 1.2: the two 400 mm² faces flag.
	let thin = cuboid(DVec3::new(0.0, 0.0, 0.0), DVec3::new(20.0, 20.0, 0.8));
	let findings = p.dfm_checks(&thin);
	let f: Vec<&DfmFinding> = findings.iter().filter(|f| f.check == "thin_wall").collect();
	assert!(
		f.len() == 1 && (f[0].measured - 800.0).abs() < 1.0 && (f[0].limit - 1.2).abs() < 1e-12,
		"0.8 mm plate must flag thin_wall with ~800 mm² at limit 1.2, got {findings:?}"
	);
	// A fat cube passes every implemented check.
	let cube = cuboid(DVec3::new(0.0, 0.0, 0.0), DVec3::new(20.0, 20.0, 20.0));
	let clean = p.dfm_checks(&cube);
	assert!(
		clean.is_empty(),
		"a 20 mm cube must audit clean under the conservative profile, got {clean:?}"
	);
	// An over-bed part must flag bed_fit with the extents in the detail.
	let huge = cuboid(DVec3::new(0.0, 0.0, 0.0), DVec3::new(300.0, 20.0, 20.0));
	let findings = p.dfm_checks(&huge);
	assert!(
		findings.iter().any(|f| f.check == "bed_fit" && f.detail.contains("300")),
		"a 300 mm part must flag bed_fit naming its extents, got {findings:?}"
	);
}

// ---- coupon nominals -------------------------------------------------------------

#[test]
fn coupon_nominals_pinned() {
	assert_eq!(coupons::VERSION, 1, "coupon set version — bump ONLY together with tools/ingest_calibration.py");
	assert_eq!(coupons::HOLE_LADDER_D.len(), 11, "hole ladder is Ø3–Ø8 in 0.5 steps");
	assert_eq!(coupons::FIT_BORE_D.len(), 7, "fit ladder is Ø6.0–Ø6.6 in 0.1 steps");
	assert!(
		coupons::HOLE_LADDER_D.windows(2).all(|w| w[1] > w[0])
			&& coupons::FIT_BORE_D.windows(2).all(|w| w[1] > w[0])
			&& coupons::BRIDGE_SPANS.windows(2).all(|w| w[1] > w[0])
			&& coupons::WALL_LADDER_T.windows(2).all(|w| w[1] > w[0])
			&& coupons::OVERHANG_DEG.windows(2).all(|w| w[1] > w[0]),
		"every coupon ladder must ascend strictly (the fiducial-corner convention depends on it)"
	);
	assert!(
		(coupons::HOLE_LADDER_D[0] - 3.0).abs() < 1e-12
			&& (coupons::HOLE_LADDER_D[10] - 8.0).abs() < 1e-12
			&& (coupons::FIT_PIN_D - 6.0).abs() < 1e-12
			&& (coupons::BORE_LARGE_D - 22.0).abs() < 1e-12
			&& (coupons::BRIDGE_SPANS[4] - 25.0).abs() < 1e-12
			&& (coupons::WALL_LADDER_T[0] - 0.8).abs() < 1e-12
			&& (coupons::OVERHANG_DEG[4] - 60.0).abs() < 1e-12,
		"coupon nominal endpoints are FROZEN (v1): holes 3–8, pin 6, bore 22, bridge ≤25, wall ≥0.8, fan ≤60 — printed coupons in the field must keep matching the ingest tool"
	);
	assert!(
		!coupons::OVERHANG_DEG.contains(&45.0),
		"45° must stay OFF the fan: it sits exactly on the default threshold and makes the audit a coin flip"
	);
	let p = FdmProfile::conservative_default();
	let (lo, hi) = (coupons::FIT_BORE_D[0], coupons::FIT_BORE_D[6]);
	let (t, fr) = (p.fit_tight_bore_d(coupons::FIT_PIN_D), p.fit_free_bore_d(coupons::FIT_PIN_D));
	assert!(
		(lo..=hi).contains(&t) && (fr < hi) && (fr > lo),
		"the fit ladder [{lo}, {hi}] must straddle the conservative tight ({t}) and free ({fr}) recommendations from both sides — otherwise a measured printer can fall off the ladder"
	);
}

// ---- ingest tool (python3) -------------------------------------------------------

fn python3() -> Option<()> {
	match Command::new("python3").arg("--version").output() {
		Ok(o) if o.status.success() => Some(()),
		_ => {
			println!("==============================================================");
			println!("  SKIPPED LOUDLY: python3 not found on PATH — the ingest");
			println!("  self-test / nominals-drift / round-trip pins did NOT run.");
			println!("  Install python3 and re-run to exercise tools/ingest_calibration.py");
			println!("==============================================================");
			None
		}
	}
}

fn ingest_tool() -> String {
	format!("{}/../../tools/ingest_calibration.py", env!("CARGO_MANIFEST_DIR"))
}

#[test]
fn ingest_self_test_round_trips_a_perfect_printer() {
	if python3().is_none() {
		return;
	}
	let out = Command::new("python3")
		.arg(ingest_tool())
		.arg("--self-test")
		.output()
		.expect("spawn python3");
	let stdout = String::from_utf8_lossy(&out.stdout);
	assert!(
		out.status.success() && stdout.contains("\"self_test\": \"PASS\""),
		"ingest --self-test must exit 0 with PASS (a synthetic perfect printer ⇒ compensations exactly 0).\nexit: {:?}\nstdout:\n{stdout}\nstderr:\n{}",
		out.status.code(),
		String::from_utf8_lossy(&out.stderr)
	);
}

#[test]
fn ingest_embedded_nominals_match_rust_consts() {
	if python3().is_none() {
		return;
	}
	let out = Command::new("python3")
		.arg(ingest_tool())
		.arg("--print-nominals")
		.output()
		.expect("spawn python3");
	assert!(out.status.success(), "--print-nominals must exit 0");
	let v: serde_json::Value = serde_json::from_slice(&out.stdout).expect("nominals JSON parses");
	let arr = |key: &str| -> Vec<f64> {
		v[key].as_array().unwrap_or(&Vec::new()).iter().map(|x| x.as_f64().unwrap()).collect()
	};
	assert!(
		v["coupons_version"].as_u64() == Some(coupons::VERSION as u64)
			&& arr("holes_d") == coupons::HOLE_LADDER_D
			&& arr("fit_bores_d") == coupons::FIT_BORE_D
			&& v["fit_pin_d"].as_f64() == Some(coupons::FIT_PIN_D)
			&& v["bore_large_d"].as_f64() == Some(coupons::BORE_LARGE_D)
			&& v["disc_d"].as_f64() == Some(coupons::DISC_D)
			&& arr("bridge_spans") == coupons::BRIDGE_SPANS
			&& arr("walls_t") == coupons::WALL_LADDER_T
			&& arr("overhang_deg") == coupons::OVERHANG_DEG,
		"tools/ingest_calibration.py's embedded nominals DRIFTED from kernel_model::process::coupons — the printed coupons and the ingest math must describe the same objects.\npython: {v}"
	);
}

#[test]
fn ingest_synthetic_measurements_load_back_through_rust_schema() {
	if python3().is_none() {
		return;
	}
	// A realistic imperfect printer: holes 0.12 undersized, bore 0.07 under,
	// seam 0.06, elephant 0.16 radial, bridges die past 15, 0.8 wall fails,
	// fan clean through 50°.
	let holes: serde_json::Map<String, serde_json::Value> = coupons::HOLE_LADDER_D
		.iter()
		.map(|d| (fmt_g(*d), serde_json::json!(d - 0.12)))
		.collect();
	let fit: serde_json::Map<String, serde_json::Value> = coupons::FIT_BORE_D
		.iter()
		.map(|d| {
			let class = if *d < 6.25 { "no_go" } else if *d < 6.35 { "press" } else { "free" };
			(fmt_g(*d), serde_json::json!(class))
		})
		.collect();
	let m = serde_json::json!({
		"printer_name": "synthetic_a1_pla",
		"material": "PLA",
		"nozzle_mm": 0.4,
		"layer_mm": 0.2,
		"bed_mm": [256.0, 256.0, 256.0],
		"coupons_version": coupons::VERSION,
		"holes": holes,
		"fit": fit,
		"bore_22": 21.93,
		"pin": {"d_min": 5.98, "d_max": 6.04},
		"disc": {"d_mid": 19.96, "d_first_layer": 20.28},
		"bridge_sag": {"5": 0.0, "10": 0.1, "15": 0.4, "20": 0.9, "25": 2.0},
		"walls": {"0.8": "gaps", "1.2": "solid", "1.6": "solid", "2": "solid", "2.4": "solid"},
		"overhang": {"35": "clean", "40": "clean", "50": "clean", "55": "rough", "60": "fail"},
	});
	let dir = std::env::temp_dir().join(format!("lmcad_process_test_{}", std::process::id()));
	let _ = std::fs::create_dir_all(&dir);
	let meas = dir.join("measurements.json");
	std::fs::write(&meas, serde_json::to_string_pretty(&m).unwrap()).expect("write measurements");
	let out = Command::new("python3")
		.arg(ingest_tool())
		.arg(&meas)
		.arg("--out")
		.arg(&dir)
		.output()
		.expect("spawn python3");
	let stdout = String::from_utf8_lossy(&out.stdout);
	assert!(
		out.status.success(),
		"ingest must accept the synthetic measurements.\nstdout:\n{stdout}\nstderr:\n{}",
		String::from_utf8_lossy(&out.stderr)
	);
	let prof_path = dir.join("synthetic_a1_pla.json");
	let p = FdmProfile::load(prof_path.to_str().unwrap())
		.expect("python-written profile must load through the Rust FdmProfile schema");
	let pinned: [(&str, f64, f64); 9] = [
		// (nominal − measured) = 0.12 everywhere ⇒ mean 0.12
		("hole_diameter_comp", p.hole_diameter_comp, 0.12),
		// 22.0 − 21.93
		("bore_comp", p.bore_comp, 0.07),
		// first press Ø6.3: measured bore 6.3 − 0.12 = 6.18; (6.18 − 6.04)/2
		("xy_clearance_tight", p.xy_clearance_tight, 0.07),
		// first free Ø6.4: (6.28 − 6.04)/2
		("xy_clearance_free", p.xy_clearance_free, 0.12),
		// 6.04 − 5.98
		("seam_allowance", p.seam_allowance, 0.06),
		// (20.28 − 19.96)/2
		("first_layer_comp", p.first_layer_comp, 0.16),
		// sag ≤ 0.5 holds through the 15 span
		("max_bridge", p.max_bridge, 15.0),
		// steepest clean fin
		("max_unsupported_angle", p.max_unsupported_angle, 50.0),
		// thinnest solid fin
		("min_wall", p.min_wall, 1.2),
	];
	for (field, got, want) in pinned {
		assert!(
			(got - want).abs() < 1e-9,
			"ingested {field} = {got}, want {want} (sign conventions: comp = nominal − measured, clearances = (measured bore − pin d_max)/2).\ningest stdout:\n{stdout}"
		);
	}
	// A placeholder-riddled template must REFUSE (the example file cannot be
	// mistaken for data).
	let bad = dir.join("placeholder.json");
	std::fs::write(&bad, serde_json::to_string_pretty(&serde_json::json!({"printer_name": "PLACEHOLDER_RENAME_ME"})).unwrap())
		.expect("write placeholder");
	let out = Command::new("python3").arg(ingest_tool()).arg(&bad).output().expect("spawn python3");
	assert!(
		!out.status.success() && String::from_utf8_lossy(&out.stdout).contains("\"ok\": false"),
		"ingest must refuse a placeholder measurement set with exit 1 + ok:false"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

/// Same trailing-zero-free key convention as the example and the ingest tool.
fn fmt_g(x: f64) -> String {
	if (x - x.round()).abs() < 1e-9 {
		format!("{}", x.round() as i64)
	} else {
		format!("{x}")
	}
}
