// Copyright (c) LMCAD. Licensed under the MIT License.

//! Pinning tests for `kernel_model::cost` — the per-process time/money model.
//!
//! What these gates prove:
//!
//! - **A solid part's mass IS `exact_volume × density`**, to the bit — no
//!   fudge factor hides in the material model.
//! - **An infilled part's mass is BRACKETED** by the shell-only and solid
//!   bounds. That is the honest way to gate an approximate model: assert the
//!   inequality the model must satisfy, not a number the model invented.
//! - **The time model's shape is pinned**, term by term: doubling the deposited
//!   volume doubles the extrusion term exactly, doubling the flow rate halves
//!   it, and the layer count is exact against hand arithmetic — including the
//!   IEEE-754 trap where `1.0 / 0.1` is `10.000000000000002`.
//! - **Support material appears only where the support report says it does**,
//!   and a flat-bottomed part gets none.
//! - **Sibling processes refuse**; absurd parameters refuse; a BOM cannot be
//!   half-costed.
//! - **Every breakdown carries its accuracy class** as a required field.

use kernel_brep::math::DVec3;
use kernel_brep::{area, cuboid, exact_volume, sphere, Solid};
use kernel_model::cost::{costed_bom, support_envelope_mm3, CostBreakdown, CostError, CostItem, CostProcess, FdmCostModel, FDM_ACCURACY_CLASS};
use kernel_model::materials::PLA_G_PER_MM3;

/// A 40 × 40 × 20 block: V = 32 000 mm³, A = 6 400 mm², 20 mm tall.
fn block() -> Solid {
	cuboid(DVec3::ZERO, DVec3::new(40.0, 40.0, 20.0))
}

/// A Ø40 ball resting on the plate — the support fixture (its lower cap
/// overhangs past 45°).
fn ball() -> Solid {
	sphere(DVec3::new(0.0, 0.0, 20.0), 20.0, 48, 24)
}

/// The conservative model with infill forced to 100% — the "solid part" case.
fn solid_model() -> FdmCostModel {
	FdmCostModel { infill_fraction: 1.0, ..FdmCostModel::conservative_default() }
}

// --- material mass ----------------------------------------------------------------

#[test]
fn a_solid_parts_mass_is_exact_volume_times_density() {
	let b = block();
	let m = solid_model();
	let c = m.estimate(&b).expect("FDM is the implemented process");
	let v = exact_volume(&b).abs();
	assert!(
		(v - 32000.0).abs() < 1e-9,
		"fixture drifted: kernel_brep::exact_volume reads {v} for a 40×40×20 block, expected 32000"
	);
	assert!(
		(c.material_g - v * m.density_g_mm3).abs() < 1e-12,
		"a 100%-infill part must weigh exactly exact_volume × density: got {} g, exact_volume × density = {} g (delta {:e})",
		c.material_g,
		v * m.density_g_mm3,
		(c.material_g - v * m.density_g_mm3).abs()
	);
	assert!(
		(c.material_g - 39.68).abs() < 1e-9,
		"32 000 mm³ of PLA at {PLA_G_PER_MM3} g/mm³ is 39.68 g; the model says {} g",
		c.material_g
	);
	assert_eq!(c.volume_source, "exact (kernel_brep::exact_volume)", "the receipt must say WHICH volume fed the mass");
	assert_eq!(c.deposited_volume_mm3, v, "at 100% infill the deposited volume IS the part volume, got {}", c.deposited_volume_mm3);
	assert_eq!(c.support_volume_mm3, 0.0, "a flat-bottomed block needs no support, got {} mm³", c.support_volume_mm3);
}

#[test]
fn an_infilled_parts_mass_sits_strictly_between_the_shell_only_and_solid_bounds() {
	let b = block();
	let m = FdmCostModel::conservative_default();
	let c = m.estimate(&b).expect("FDM");
	let v = exact_volume(&b).abs();
	let a = area(&b);
	let shell_only_g = (a * m.shell_thickness_mm).min(v) * m.density_g_mm3;
	let solid_g = v * m.density_g_mm3;
	assert!(
		c.material_g > shell_only_g && c.material_g < solid_g,
		"a {:.0}%-infill part must weigh MORE than its bare {:.3} mm shell ({shell_only_g:.5} g) and LESS than solid ({solid_g:.5} g); the model says {:.5} g — the bracket is the honest gate for an approximate model",
		m.infill_fraction * 100.0,
		m.shell_thickness_mm,
		c.material_g
	);
	// Hand arithmetic: shell 6400 × 1.2 = 7680 mm³, core 24 320 mm³ at 20 % →
	// 12 544 mm³ deposited → 15.554 56 g.
	assert!(
		(c.deposited_volume_mm3 - 12544.0).abs() < 1e-9,
		"deposited volume = min(V, A·t) + infill·(V − shell) = 7680 + 0.2·24320 = 12544 mm³, got {}",
		c.deposited_volume_mm3
	);
	assert!((c.material_g - 15.55456).abs() < 1e-9, "12 544 mm³ of PLA is 15.554 56 g, got {}", c.material_g);
}

#[test]
fn a_part_thinner_than_its_shell_costs_as_solid() {
	// A 0.5 mm-thick sheet cannot hold a 1.2 mm shell: the shell term is capped
	// at the part volume, so the mass must equal the solid mass exactly.
	let sheet = cuboid(DVec3::ZERO, DVec3::new(40.0, 40.0, 0.5));
	let m = FdmCostModel::conservative_default();
	let c = m.estimate(&sheet).expect("FDM");
	let v = exact_volume(&sheet).abs();
	assert!(
		(c.material_g - v * m.density_g_mm3).abs() < 1e-12,
		"a part thinner than twice the shell must cost as SOLID (that is what a slicer does): got {} g vs solid {} g",
		c.material_g,
		v * m.density_g_mm3
	);
}

// --- time -------------------------------------------------------------------------

#[test]
fn print_time_is_monotonic_in_volume_and_in_flow_with_pinned_ratios() {
	let m = FdmCostModel::conservative_default();
	let t1 = m.print_time_minutes(12000.0, 20.0).expect("valid model");
	let t2 = m.print_time_minutes(24000.0, 20.0).expect("valid model");
	let fast = FdmCostModel { volumetric_flow_mm3_s: 24.0, ..m.clone() };
	let t3 = fast.print_time_minutes(12000.0, 20.0).expect("valid model");

	assert!(t2 > t1, "doubling the deposited volume must take LONGER: {t2} vs {t1} min");
	assert!(t3 < t1, "doubling the flow rate must take LESS time: {t3} vs {t1} min");

	// (V/flow · 1.12 + 100 layers · 1.5 s) / 60 + 5 min setup.
	assert!((t1 - 26.166666666666668).abs() < 1e-9, "12 000 mm³ at 12 mm³/s over 100 layers is 26.1667 min, got {t1}");
	assert!((t2 - 44.833333333333336).abs() < 1e-9, "24 000 mm³ is 44.8333 min, got {t2}");
	assert!((t3 - 16.833333333333336).abs() < 1e-9, "12 000 mm³ at 24 mm³/s is 16.8333 min, got {t3}");

	// The extrusion component alone — the term the two knobs act on — scales
	// exactly 2× and exactly ½×.
	let extrusion = |t: f64| (t - m.setup_minutes) * 60.0 - 100.0 * m.per_layer_overhead_s;
	assert!(
		(extrusion(t2) / extrusion(t1) - 2.0).abs() < 1e-12,
		"doubling the volume must double the extrusion term exactly, ratio was {}",
		extrusion(t2) / extrusion(t1)
	);
	assert!(
		(extrusion(t3) / extrusion(t1) - 0.5).abs() < 1e-12,
		"doubling the flow must halve the extrusion term exactly, ratio was {}",
		extrusion(t3) / extrusion(t1)
	);
}

#[test]
fn layer_count_is_exact_against_hand_arithmetic_including_the_ieee_trap() {
	let m = FdmCostModel::conservative_default();
	assert_eq!(m.layer_count(20.0).expect("valid"), 100, "20.0 mm at 0.2 mm layers is 100 layers");
	assert_eq!(m.layer_count(20.05).expect("valid"), 101, "20.05 mm at 0.2 mm needs a 101st partial layer");
	assert_eq!(m.layer_count(0.05).expect("valid"), 1, "anything thinner than one layer still needs one");
	assert_eq!(m.layer_count(0.0).expect("valid"), 0, "a zero-height part has no layers");

	let fine = FdmCostModel { layer_height_mm: 0.1, ..m.clone() };
	assert_eq!(
		fine.layer_count(1.0).expect("valid"),
		10,
		"1.0 / 0.1 is 10.000000000000002 in IEEE-754; a raw ceil() would report 11 layers for a 1 mm part"
	);
	assert_eq!(fine.layer_count(1.05).expect("valid"), 11, "1.05 mm at 0.1 mm genuinely needs 11 layers");

	// And the count that reaches the breakdown is the same one.
	let c = m.estimate(&block()).expect("FDM");
	assert_eq!(c.layers, 100, "a 20 mm block at 0.2 mm layers reports {} layers", c.layers);
	assert!((c.print_height_mm - 20.0).abs() < 1e-12, "build height reads {}", c.print_height_mm);
}

// --- support ----------------------------------------------------------------------

#[test]
fn support_appears_only_where_the_support_report_says_it_does() {
	let flat = support_envelope_mm3(&block(), 45.0, 0.3);
	assert_eq!(
		flat, 0.0,
		"a flat-bottomed block prints support-free at 45°; the envelope must be exactly 0, got {flat} mm³"
	);

	let b = ball();
	let env = support_envelope_mm3(&b, 45.0, 0.3);
	assert!(
		(env - 1748.600088070012).abs() < 1e-9,
		"the Ø40 ball's support envelope reads {env} mm³ — pinned at 1748.600088070012 (48×24 tessellation, prism-to-bed under every steep facet)"
	);
	// The envelope is an UPPER bound by construction: it can never exceed the
	// prism under the part's own footprint.
	let footprint_prism = std::f64::consts::PI * 20.0 * 20.0 * 40.0;
	assert!(
		env < footprint_prism,
		"the support envelope ({env}) cannot exceed the whole prism under the part ({footprint_prism})"
	);

	let m = FdmCostModel::conservative_default();
	let c = m.estimate(&b).expect("FDM");
	assert!(
		(c.support_volume_mm3 - m.support_density * env).abs() < 1e-9,
		"support material must be support_density × envelope = {} mm³, got {}",
		m.support_density * env,
		c.support_volume_mm3
	);
	let none = FdmCostModel { support_density: 0.0, ..m.clone() };
	let c0 = none.estimate(&b).expect("FDM");
	assert_eq!(c0.support_volume_mm3, 0.0, "at zero support density no support material is deposited");
	assert!(
		c.material_g > c0.material_g,
		"a supported ball must weigh more than the same ball with support suppressed: {} vs {} g",
		c.material_g,
		c0.material_g
	);
	assert!(
		c.model_accuracy_note.contains("support envelope 1748.600 mm3 at 15% density"),
		"the accuracy note must state the support the estimate assumed, got:\n{}",
		c.model_accuracy_note
	);
}

// --- refusals ---------------------------------------------------------------------

#[test]
fn sibling_processes_refuse_instead_of_inventing_a_cost_model() {
	let b = block();
	for (p, name) in [
		(CostProcess::SheetMetal, "sheet_metal"),
		(CostProcess::Casting, "casting"),
		(CostProcess::Cnc, "cnc"),
	] {
		let err = p.estimate(&b).expect_err("only FDM has a cost model today");
		match &err {
			CostError::NotImplemented { process, note } => {
				assert_eq!(*process, name, "the refusal must name the process");
				assert!(!note.is_empty(), "the refusal for {name} must say what does NOT exist for it");
			}
			other => panic!("expected NotImplemented for {name}, got {other:?}"),
		}
		let msg = err.to_string();
		assert!(msg.contains("cost model not implemented"), "the {name} refusal must be legible, got '{msg}'");
	}
	// Casting's refusal points at the castability half that DOES exist.
	let casting = CostProcess::Casting.estimate(&b).expect_err("casting refuses");
	assert!(
		casting.to_string().contains("draft_analysis"),
		"the casting refusal must name what already exists (kernel_brep::draft_analysis), got '{casting}'"
	);
}

#[test]
fn absurd_parameters_refuse_loudly_and_never_produce_a_number() {
	let b = block();
	let base = FdmCostModel::conservative_default();
	let cases: [(&str, FdmCostModel); 6] = [
		("volumetric_flow_mm3_s", FdmCostModel { volumetric_flow_mm3_s: 0.0, ..base.clone() }),
		("density_g_mm3", FdmCostModel { density_g_mm3: -0.00124, ..base.clone() }),
		("infill_fraction", FdmCostModel { infill_fraction: 1.5, ..base.clone() }),
		("layer_height_mm", FdmCostModel { layer_height_mm: -0.2, ..base.clone() }),
		("machine_cost_per_hour", FdmCostModel { machine_cost_per_hour: f64::NAN, ..base.clone() }),
		("support_overhang_deg", FdmCostModel { support_overhang_deg: 0.0, ..base.clone() }),
	];
	for (field, m) in cases {
		let err = m.validate().expect_err("an impossible model must not validate");
		assert!(
			matches!(&err, CostError::BadParameter { field: f, .. } if *f == field),
			"expected BadParameter on '{field}', got {err:?}"
		);
		assert!(
			m.estimate(&b).is_err(),
			"'{field}' is out of range, so estimate() must refuse rather than return a number"
		);
		assert!(err.to_string().contains("refusing to produce a number"), "the '{field}' refusal must say why, got '{err}'");
	}
	// A zero flow rate would be an INFINITE time, not a free print.
	let zero_flow = FdmCostModel { volumetric_flow_mm3_s: 0.0, ..base.clone() };
	assert!(zero_flow.print_time_minutes(1000.0, 10.0).is_err(), "zero flow must refuse, not divide by zero");
	// A profile with no name is not a profile.
	let unnamed = FdmCostModel { name: "  ".to_string(), ..base };
	assert!(matches!(unnamed.validate(), Err(CostError::BadParameter { field: "name", .. })), "an unnamed profile must refuse");
}

#[test]
fn a_part_with_no_geometry_refuses() {
	let empty = Solid::from_faces(Vec::new(), Vec::new());
	let err = FdmCostModel::conservative_default().estimate(&empty).expect_err("nothing to cost");
	assert!(matches!(err, CostError::NoGeometry { .. }), "expected NoGeometry, got {err:?}");
	assert!(err.to_string().contains("nothing to cost"), "the refusal must be legible, got '{err}'");
}

// --- the required accuracy note ---------------------------------------------------

#[test]
fn every_breakdown_carries_its_accuracy_class_as_a_required_field() {
	let c: CostBreakdown = FdmCostModel::conservative_default().estimate(&block()).expect("FDM");
	assert!(
		c.model_accuracy_note.starts_with(FDM_ACCURACY_CLASS),
		"every breakdown must lead with the declared accuracy class, got:\n{}",
		c.model_accuracy_note
	);
	assert!(c.model_accuracy_note.contains("+/-30% CLASS ESTIMATE"), "the error bar must be stated in words");
	for excluded in ["acceleration", "cooling-limited minimum layer time", "first-layer slowdown", "retraction"] {
		assert!(
			c.model_accuracy_note.contains(excluded),
			"the note must name '{excluded}' among what the model does NOT capture, got:\n{}",
			c.model_accuracy_note
		);
	}
	assert!(
		c.model_accuracy_note.contains("Money rates are declared inputs, not measurements"),
		"the note must state that the money numbers are inputs, not measurements"
	);
	assert!(c.summary().contains("+/-30% class"), "even the one-line summary carries the error bar, got '{}'", c.summary());
	// The arithmetic of the breakdown is internally consistent.
	assert!(
		(c.total - (c.material_cost + c.machine_cost)).abs() < 1e-12,
		"total {} must be material {} + machine {}",
		c.total,
		c.material_cost,
		c.machine_cost
	);
	assert!(
		(c.material_cost - c.material_g / 1000.0 * 25.0).abs() < 1e-12,
		"material cost must be mass × price per kg, got {}",
		c.material_cost
	);
	assert!(
		(c.machine_cost - c.time_minutes / 60.0 * 1.0).abs() < 1e-12,
		"machine cost must be time × hourly rate, got {}",
		c.machine_cost
	);
}

#[test]
fn the_conservative_defaults_are_pinned_with_their_provenance() {
	let m = FdmCostModel::conservative_default();
	m.validate().expect("the declared default must pass its own range check");
	let pinned: [(&str, f64, f64, &str); 9] = [
		("layer_height_mm", m.layer_height_mm, 0.2, "the 0.4 mm-nozzle default every shipped campaign was sliced at"),
		("volumetric_flow_mm3_s", m.volumetric_flow_mm3_s, 12.0, "deliberately below advertised peak: a sliced average over short perimeters"),
		("per_layer_overhead_s", m.per_layer_overhead_s, 1.5, "layer change + Z move + seam + prime"),
		("travel_fraction", m.travel_fraction, 0.12, "travel as a fraction of extrusion time"),
		("shell_thickness_mm", m.shell_thickness_mm, 1.2, "drybox_roller RIB_T — the thinnest wall a shipped campaign prints"),
		("infill_fraction", m.infill_fraction, 0.20, "middle of the drybox_roller BOM's declared 15-25%"),
		("density_g_mm3", m.density_g_mm3, PLA_G_PER_MM3, "kernel_model::materials::PLA_G_PER_MM3"),
		("support_overhang_deg", m.support_overhang_deg, 45.0, "the support_free_report(Z, 45.0, 0.3) threshold every campaign gates on"),
		("support_density", m.support_density, 0.15, "typical sparse support density"),
	];
	for (field, got, want, why) in pinned {
		assert!(
			(got - want).abs() < 1e-12,
			"conservative_default.{field} = {got}, declared as {want} ({why}) — changing it changes every uncalibrated estimate"
		);
	}
	assert_eq!(m.name, "conservative_default", "the fallback profile must be named for what it is");
	// The money rates are placeholders, and the doc says so; pin them anyway so a
	// silent change is visible.
	assert!((m.material_cost_per_kg - 25.0).abs() < 1e-12, "PLACEHOLDER material price drifted to {}", m.material_cost_per_kg);
	assert!((m.machine_cost_per_hour - 1.0).abs() < 1e-12, "PLACEHOLDER machine rate drifted to {}", m.machine_cost_per_hour);
}

// --- costed BOM -------------------------------------------------------------------

#[test]
fn costed_bom_groups_like_section_18_4_and_is_byte_stable() {
	let small = block();
	let big = cuboid(DVec3::ZERO, DVec3::new(60.0, 40.0, 20.0));
	let build = || {
		let items = [
			CostItem { name: "spacer", params: "h=8", count: 2, solid: &small },
			CostItem { name: "spacer", params: "h=10", count: 1, solid: &big },
			CostItem { name: "cap", params: "", count: 3, solid: &small },
			CostItem { name: "spacer", params: "h=8", count: 1, solid: &small },
		];
		costed_bom(&items, &CostProcess::Fdm(FdmCostModel::conservative_default()), "USD").expect("FDM costs every line")
	};
	let bom = build();
	assert_eq!(
		bom.lines.len(),
		3,
		"§18.4 groups by (name, params): spacer/h=8, spacer/h=10 and cap are 3 lines, got {:?}",
		bom.lines.iter().map(|l| format!("{}/{}", l.name, l.params)).collect::<Vec<_>>()
	);
	assert_eq!(bom.lines[0].name, "cap", "lines sort by name then params; 'cap' comes first, got '{}'", bom.lines[0].name);
	assert_eq!(bom.lines[0].count, 3, "the cap line carries 3");
	// Ordering is the LEXICOGRAPHIC (name, params) sort of a BTreeMap — the same
	// key order `format::BomV2::flat` uses — so "h=10" precedes "h=8".
	assert_eq!(bom.lines[1].params, "h=10", "params sort lexicographically like BomV2's flat view, got '{}'", bom.lines[1].params);
	assert_eq!(bom.lines[2].params, "h=8", "the h=8 spacer line is last, got '{}'", bom.lines[2].params);
	assert_eq!(bom.lines[2].count, 3, "two spacer entries at h=8 (2 + 1) must merge into one line of 3, got {}", bom.lines[2].count);

	// Line arithmetic.
	for l in &bom.lines {
		assert!(
			(l.line_total - l.unit.total * l.count as f64).abs() < 1e-12,
			"line '{}' total {} must be unit {} × {}",
			l.name,
			l.line_total,
			l.unit.total,
			l.count
		);
		assert!((l.line_material_g - l.unit.material_g * l.count as f64).abs() < 1e-9, "line '{}' mass roll-up", l.name);
	}
	let sum: f64 = bom.lines.iter().map(|l| l.line_total).sum();
	assert!((bom.total - sum).abs() < 1e-12, "the BOM total {} must be the sum of its lines {sum}", bom.total);

	// Byte stability, both renderings, across two independent builds.
	let again = build();
	assert_eq!(bom.to_csv(), again.to_csv(), "the costed BOM CSV must be byte-identical across runs");
	assert_eq!(bom.to_markdown(), again.to_markdown(), "the costed BOM Markdown must be byte-identical across runs");
	assert!(
		bom.to_csv().starts_with("name,params,count,unit_material_g,unit_time_minutes,unit_total,line_total\n"),
		"the CSV header is a fixed contract, got:\n{}",
		bom.to_csv()
	);
	assert!(
		bom.to_markdown().contains("+/-30% CLASS ESTIMATE"),
		"a costed table must carry the error bar where the reader cannot miss it, got:\n{}",
		bom.to_markdown()
	);
	assert!(bom.to_markdown().contains("| **TOTAL** |"), "the table must total itself");
	assert!(bom.to_markdown().contains("unit cost (USD)"), "the currency label must reach the header");
}

#[test]
fn a_bom_cannot_be_half_costed_by_a_process_with_no_model() {
	let b = block();
	let items = [CostItem { name: "widget", params: "", count: 1, solid: &b }];
	let err = costed_bom(&items, &CostProcess::Cnc, "USD").expect_err("a BOM must refuse wholesale, not silently skip lines");
	assert!(matches!(err, CostError::NotImplemented { process: "cnc", .. }), "expected the CNC refusal, got {err:?}");
}

#[test]
fn an_empty_bom_is_empty_but_still_declares_its_accuracy_class() {
	let bom = costed_bom(&[], &CostProcess::Fdm(FdmCostModel::conservative_default()), "EUR").expect("an empty set costs nothing");
	assert!(bom.lines.is_empty(), "no items, no lines");
	assert_eq!(bom.total, 0.0, "an empty BOM totals zero");
	assert!(
		bom.model_accuracy_note.contains("+/-30%"),
		"even an empty table declares which model produced its (absent) numbers"
	);
}
