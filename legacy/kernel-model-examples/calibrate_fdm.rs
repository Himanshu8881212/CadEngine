// Copyright (c) LMCAD. Licensed under the MIT License.

//! CALIBRATE-FDM — the measured-reality coupon campaign behind
//! `kernel_model::process::FdmProfile`.
//!
//! Seven small, fast coupons measure everything the profile parameterizes:
//!
//! | coupon | measures | profile fields fed |
//! |---|---|---|
//! | `coupon_holes` | Ø3–Ø8 × 0.5 through-hole ladder | `hole_diameter_comp` |
//! | `coupon_fit` | Ø6.0–Ø6.6 × 0.1 bore ladder for the pin | `xy_clearance_tight/free` |
//! | `coupon_pin` | Ø6 post on a Ø20 disc | `seam_allowance`, `first_layer_comp` (+ the fit ladder's counterpart) |
//! | `coupon_bore` | Ø22 gauge (a 608 bearing's OD drops in) | `bore_comp` |
//! | `coupon_bridge` | 5/10/15/20/25 mm clear spans | `max_bridge` |
//! | `coupon_walls` | 0.8–2.4 × 0.4 fin ladder | `min_wall` |
//! | `coupon_overhang` | 35/40/50/55/60° fan | `max_unsupported_angle` |
//!
//! Workflow: print `parts/`, measure with calipers per `README.md`, copy
//! `measurements.example.json` → `measurements.json`, fill it in, then
//! `python3 tools/ingest_calibration.py calibration_system/fdm_coupons/measurements.json`
//! writes `profiles/<printer>.json` — the FdmProfile campaigns consume through
//! its fit helpers instead of frozen clearance consts.
//!
//! METROLOGY HONESTY: every measured feature is exact B-rep, tessellated with
//! enough segments that the chord sagitta is ≤ 0.005 mm (gated per feature via
//! `Mesh::radial_extent` below) — an order of magnitude under caliper
//! resolution. Labels are geometric (a 45° plan-view fiducial chamfer marks
//! each ladder's ascending end); the implicit text capability was NOT used
//! because routing these parts through a voxel remesh would smear the very
//! surfaces being calibrated (route honesty: exact stays exact).
//!
//! Run: cargo run --release -p kernel-model --example calibrate_fdm
//!   -> calibration_system/fdm_coupons/ (exit 1 on any FAIL)

use kernel_brep::math::{DAffine3, DVec2, DVec3};
use kernel_brep::{
	boolean_hazards, cuboid, cylinder, difference, export_step, export_step_assembly, extrude, force_ccw,
	overlap_volume, tessellate_default, union, validate, volume, HazardKind, Mesh, Solid,
};
use kernel_core::math::Vec3;
use kernel_model::campaign::gate;
use kernel_model::process::{coupons, FdmProfile, Process};
use kernel_model::{materials, sweep_check};
use std::f64::consts::PI;

// ---- coupon layout (mm) — nominals come from kernel_model::process::coupons -----

/// Chord sagitta budget for every metrological cylinder: 0.005 mm keeps facet
/// error an order of magnitude under caliper resolution (±0.02 typical).
const SAG_MAX: f64 = 0.005;

const HOLES_L: f64 = 126.0; // 11 holes at pitch 11 + 8 mm end margins
const HOLES_W: f64 = 16.0; // Ø8 max hole leaves a 4 mm rim each side
const HOLES_T: f64 = 4.0; // caliper jaws reach through; fast to print
const HOLES_X0: f64 = 8.0; // first (Ø3) hole centre, from the fiducial corner
const HOLES_PITCH: f64 = 11.0; // web ≥ 11 − 7.75 = 3.25 between the two biggest

const FIT_L: f64 = 92.0; // 7 bores at pitch 12 + 10 mm end margins
const FIT_W: f64 = 16.0;
const FIT_T: f64 = 5.0; // deep enough to feel a press vs a slide
const FIT_X0: f64 = 10.0; // first (Ø6.0) bore centre, from the fiducial corner
const FIT_PITCH: f64 = 12.0; // web 12 − 6.3 = 5.7

const BORE_SQ: f64 = 34.0; // Ø22 gauge plate: (34 − 22)/2 = 6 mm walls
const BORE_T: f64 = 6.0; // just under a 608's 7 mm width — the bearing stands proud to grab

const PIN_DISC_T: f64 = 4.0; // base disc: elephant-foot + XY scale reference
const PIN_LEN: f64 = 14.0; // exposed post length above the disc
const EMBED: f64 = 0.3; // standard union embed (drybox convention) — no kissing faces

const BR_PILLAR: f64 = 4.0; // bridge-ladder pillar wall thickness
const BR_DEPTH: f64 = 28.0; // > longest span, so the span metric reads the GAP
const BR_H: f64 = 8.0; // pillar height under the deck
const BR_DECK: f64 = 1.2; // deck plate thickness (the bridging membrane)
const BR_INSET: f64 = 0.5; // deck ends stop short of the end pillars' outer walls

const WALL_L: f64 = 64.0; // 5 fins at pitch 12 + 8 mm end margins
const WALL_W: f64 = 12.0;
const WALL_BASE_T: f64 = 3.0;
const WALL_FIN_H: f64 = 10.0; // exposed fin height above the base
const WALL_FIN_W: f64 = 10.0; // fin width, inset 1 mm from each base edge
const WALL_X0: f64 = 8.0;
const WALL_PITCH: f64 = 12.0;

const OV_BASE_L: f64 = 70.0;
const OV_BASE_W: f64 = 10.0;
const OV_BASE_T: f64 = 3.0;
const OV_FIN_T: f64 = 3.0; // fin thickness (X at the root)
const OV_FIN_W: f64 = 8.0; // fin width (Y), inset 1 mm from each base edge
const OV_FIN_H: f64 = 12.0; // exposed leaning height above the base
const OV_X0: [f64; 5] = [2.0, 12.0, 22.0, 34.0, 46.0]; // fin roots — gaps ≥ 7 grow with z

const PLA: f64 = materials::PLA_G_PER_MM3;

// ---- small helpers ---------------------------------------------------------------

fn v(x: f64, y: f64, z: f64) -> DVec3 {
	DVec3::new(x, y, z)
}

fn tr(x: f64, y: f64, z: f64) -> DAffine3 {
	DAffine3::from_translation(v(x, y, z))
}

/// Segment count whose chord sagitta `r·(1 − cos(π/n))` is ≤ [`SAG_MAX`],
/// rounded up to a multiple of 8 (min 32).
fn seg_for(r: f64) -> usize {
	let n = (PI / (1.0 - SAG_MAX / r).acos()).ceil() as usize;
	n.max(32).div_ceil(8) * 8
}

/// Trailing-zero-free decimal for JSON keys ("3", "3.5", "6.1") — the same
/// convention `tools/ingest_calibration.py` uses (`%g`).
fn fmt_g(x: f64) -> String {
	if (x - x.round()).abs() < 1e-9 {
		format!("{}", x.round() as i64)
	} else {
		format!("{x}")
	}
}

fn mesh_posed(m: &Mesh, t: DAffine3) -> Mesh {
	let mut out = m.clone();
	for p in &mut out.positions {
		let q = t.transform_point3(DVec3::new(p.x as f64, p.y as f64, p.z as f64));
		*p = Vec3::new(q.x as f32, q.y as f32, q.z as f32);
	}
	out
}

fn merge_into(dst: &mut Mesh, src: &Mesh) {
	let base = dst.positions.len() as u32;
	// Keep the "normals, when present, are one per vertex" invariant: merge
	// them only when both sides carry a full set, else drop them (STL/3MF
	// writers recompute from winding).
	let keep_normals = dst.normals.len() == dst.positions.len() && src.normals.len() == src.positions.len();
	dst.positions.extend_from_slice(&src.positions);
	dst.indices.extend(src.indices.iter().map(|i| i + base));
	if keep_normals {
		dst.normals.extend_from_slice(&src.normals);
	} else {
		dst.normals.clear();
	}
}

/// Union a list of mutually disjoint solids into one (pre-union pattern: the
/// later difference/union runs ONE arrangement instead of N).
fn union_all(parts: Vec<Solid>) -> Solid {
	let mut acc: Option<Solid> = None;
	for p in parts {
		acc = Some(match acc {
			Some(a) => union(&a, &p),
			None => p,
		});
	}
	acc.expect("union_all called with at least one solid")
}

/// 45° plan-view fiducial chamfer cutter for the corner at the ladder's
/// ascending origin: a triangular prism whose legs sit OUTSIDE the plate
/// (only the hypotenuse plane cuts — no coplanar cutter faces, §7.7).
fn fiducial_cutter(t: f64) -> Solid {
	let tri = force_ccw(vec![DVec2::new(-1.0, -1.0), DVec2::new(6.0, -1.0), DVec2::new(-1.0, 6.0)]);
	extrude(&tri, t + 1.0).transformed(tr(0.0, 0.0, -0.5))
}

// ---- coupons ---------------------------------------------------------------------

/// Hole ladder: Ø3–Ø8 through-holes ascending from the chamfered corner.
fn build_coupon_holes() -> Solid {
	let plate = difference(&cuboid(v(0.0, 0.0, 0.0), v(HOLES_L, HOLES_W, HOLES_T)), &fiducial_cutter(HOLES_T));
	let cutters = union_all(
		coupons::HOLE_LADDER_D
			.iter()
			.enumerate()
			.map(|(i, d)| {
				let x = HOLES_X0 + i as f64 * HOLES_PITCH;
				let r = d / 2.0;
				cylinder(v(x, HOLES_W / 2.0, -0.5), DVec3::Z, r, HOLES_T + 1.0, seg_for(r))
			})
			.collect(),
	);
	// §7.7 pre-flight: cutters are interior verticals — the linter must agree.
	let hz = boolean_hazards(&plate, &cutters, 0.05);
	let risky: Vec<_> = hz
		.iter()
		.filter(|h| {
			matches!(h.kind, HazardKind::NearCoincidentPlanes | HazardKind::NearCoincidentCylinders | HazardKind::EdgeInFace)
		})
		.collect();
	assert!(risky.is_empty(), "hole-ladder cut fails the §7.7 pre-flight: {risky:?}");
	difference(&plate, &cutters)
}

/// Fit ladder: Ø6.0–Ø6.6 bores for the printed Ø6 pin.
fn build_coupon_fit() -> Solid {
	let plate = difference(&cuboid(v(0.0, 0.0, 0.0), v(FIT_L, FIT_W, FIT_T)), &fiducial_cutter(FIT_T));
	let cutters = union_all(
		coupons::FIT_BORE_D
			.iter()
			.enumerate()
			.map(|(i, d)| {
				let x = FIT_X0 + i as f64 * FIT_PITCH;
				let r = d / 2.0;
				cylinder(v(x, FIT_W / 2.0, -0.5), DVec3::Z, r, FIT_T + 1.0, seg_for(r))
			})
			.collect(),
	);
	difference(&plate, &cutters)
}

/// Large-bore gauge: Ø22 through a square plate — a 608 bearing drops in.
fn build_coupon_bore() -> Solid {
	let r = coupons::BORE_LARGE_D / 2.0;
	difference(
		&cuboid(v(0.0, 0.0, 0.0), v(BORE_SQ, BORE_SQ, BORE_T)),
		&cylinder(v(BORE_SQ / 2.0, BORE_SQ / 2.0, -0.5), DVec3::Z, r, BORE_T + 1.0, seg_for(r)),
	)
}

/// Reference pin: Ø20 disc (first-layer + XY scale) carrying the Ø6 post
/// (seam + the fit ladder's counterpart). Post embedded 0.3 into the disc.
fn build_coupon_pin() -> Solid {
	let rd = coupons::DISC_D / 2.0;
	let rp = coupons::FIT_PIN_D / 2.0;
	union(
		&cylinder(v(0.0, 0.0, 0.0), DVec3::Z, rd, PIN_DISC_T, seg_for(rd)),
		&cylinder(v(0.0, 0.0, PIN_DISC_T - EMBED), DVec3::Z, rp, PIN_LEN + EMBED, seg_for(rp)),
	)
}

/// Bridge ladder: six pillar walls with 5/10/15/20/25 mm clear gaps under a
/// 1.2 mm deck. Deck ends stop 0.5 short of the outer pillar walls and the
/// pillars embed 0.3 into the deck — no coincident or coplanar face pairs.
fn build_coupon_bridge() -> (Solid, f64) {
	let mut walls = Vec::new();
	let mut x = 0.0;
	let mut length = 0.0;
	for (i, span) in coupons::BRIDGE_SPANS.iter().enumerate() {
		walls.push(cuboid(v(x, 0.0, 0.0), v(x + BR_PILLAR, BR_DEPTH, BR_H + EMBED)));
		x += BR_PILLAR + span;
		if i == coupons::BRIDGE_SPANS.len() - 1 {
			walls.push(cuboid(v(x, 0.0, 0.0), v(x + BR_PILLAR, BR_DEPTH, BR_H + EMBED)));
			length = x + BR_PILLAR;
		}
	}
	let deck = cuboid(v(BR_INSET, 0.0, BR_H), v(length - BR_INSET, BR_DEPTH, BR_H + BR_DECK));
	(union(&union_all(walls), &deck), length)
}

/// Wall ladder: 0.8–2.4 mm fins on a base bar, embedded 0.3, inset 1 mm from
/// the base edges (no coplanar side faces).
fn build_coupon_walls() -> Solid {
	let base = cuboid(v(0.0, 0.0, 0.0), v(WALL_L, WALL_W, WALL_BASE_T));
	let fins = union_all(
		coupons::WALL_LADDER_T
			.iter()
			.enumerate()
			.map(|(i, t)| {
				let x = WALL_X0 + i as f64 * WALL_PITCH;
				cuboid(
					v(x - t / 2.0, 1.0, WALL_BASE_T - EMBED),
					v(x + t / 2.0, 1.0 + WALL_FIN_W, WALL_BASE_T + WALL_FIN_H),
				)
			})
			.collect(),
	);
	union(&base, &fins)
}

/// Overhang fan: fins leaning 35/40/50/55/60° from vertical. Fin undersides
/// beyond the profile threshold are STEEP BY DESIGN — this coupon is both the
/// `max_unsupported_angle` measurement and a live positive control that the
/// steep-face detector reads the right area.
fn build_coupon_overhang() -> Solid {
	let base = cuboid(v(0.0, 0.0, 0.0), v(OV_BASE_L, OV_BASE_W, OV_BASE_T));
	let fins = union_all(
		coupons::OVERHANG_DEG
			.iter()
			.zip(OV_X0.iter())
			.map(|(deg, x0)| {
				let z0 = OV_BASE_T - EMBED;
				let z1 = OV_BASE_T + OV_FIN_H;
				let shear = (z1 - z0) * deg.to_radians().tan();
				let prof = force_ccw(vec![
					DVec2::new(*x0, z0),
					DVec2::new(x0 + OV_FIN_T, z0),
					DVec2::new(x0 + OV_FIN_T + shear, z1),
					DVec2::new(x0 + shear, z1),
				]);
				// (x, z) profile extruded along −Y by rot_x(+90°), then shifted
				// so the fin spans y ∈ [1, 1 + OV_FIN_W] inside the base.
				extrude(&prof, OV_FIN_W)
					.transformed(DAffine3::from_rotation_x(PI / 2.0))
					.transformed(tr(0.0, 1.0 + OV_FIN_W, 0.0))
			})
			.collect(),
	);
	union(&base, &fins)
}

/// Downward slant area the fan puts beyond `threshold_deg` — the analytic
/// expectation the steep-detector gate must reproduce.
fn expected_steep_area(threshold_deg: f64) -> f64 {
	coupons::OVERHANG_DEG
		.iter()
		.filter(|d| **d > threshold_deg)
		.map(|d| OV_FIN_W * OV_FIN_H / d.to_radians().cos())
		.sum()
}

// ---- per-part emit gate ----------------------------------------------------------

/// validate → drop to bed → support audit (window per part: the bridge and
/// overhang coupons CLAIM their flagged geometry, everything else claims
/// none) → watertight → profile bed fit → write STL/3MF. Returns the pass
/// verdict and the bed-posed mesh.
fn emit(dir: &str, name: &str, s: &Solid, steep_win: (f64, f64), bridge_hi: f64, p: &FdmProfile) -> (bool, Mesh) {
	let val = validate(s);
	let m0 = tessellate_default(s);
	let zmin = m0.positions.iter().map(|q| q.z as f64).fold(f64::INFINITY, f64::min);
	let mesh = mesh_posed(&m0, tr(0.0, 0.0, -zmin));
	let rep = mesh.support_free_report(Vec3::Z, p.max_unsupported_angle as f32, 0.3);
	let bb = mesh.aabb();
	let ext = [(bb.max.x - bb.min.x) as f64, (bb.max.y - bb.min.y) as f64, (bb.max.z - bb.min.z) as f64];
	let wt = mesh.is_watertight();
	let vol = volume(s).abs();
	let ok = val.is_valid()
		&& wt
		&& rep.steep_area >= steep_win.0
		&& rep.steep_area <= steep_win.1
		&& rep.max_bridge_span <= bridge_hi + 1e-6
		&& p.bed_fits(ext);
	let _ = std::fs::write(format!("calibration_system/fdm_coupons/{dir}/{name}.stl"), mesh.to_stl_binary());
	let _ = mesh.write_3mf(format!("calibration_system/fdm_coupons/{dir}/{name}.3mf"));
	println!(
		"  {name:16} valid={:5} wt={wt:5} steep={:7.1} mm² bridge≤{:5.2} {:5.1} g  {:7.0} mm³  {}",
		val.is_valid(),
		rep.steep_area,
		rep.max_bridge_span,
		vol * PLA,
		vol,
		if ok { "OK" } else { "<<< FAIL" }
	);
	(ok, mesh)
}

/// Worst chord error `nominal_r − min tessellated radius` over a set of
/// cylindrical features, measured with `radial_extent` (triangle-clipping
/// exact). Returns `(worst_err, label_of_worst)`.
fn worst_bore_sag(mesh: &Mesh, features: &[(f64, f64, f64)], band: (f32, f32)) -> (f64, String) {
	let mut worst = (f64::NEG_INFINITY, String::new());
	for (x, y, r) in features {
		let (rmin, _) = mesh
			.radial_extent(Vec3::new(*x as f32, *y as f32, 0.0), Vec3::Z, Some(band))
			.expect("bore band contains surface");
		let err = r - rmin;
		if err > worst.0 {
			worst = (err, format!("Ø{} at x={x}", fmt_g(2.0 * r)));
		}
	}
	worst
}

// ---- main ------------------------------------------------------------------------

fn main() {
	// Campaign runs always contribute to the Level-1 flywheel.
	kernel_core::telemetry::enable();
	for d in ["parts", "optional", "cad", "analysis", "assembly/scene", "renders", "publish"] {
		let _ = std::fs::create_dir_all(format!("calibration_system/fdm_coupons/{d}"));
	}
	let _ = std::fs::create_dir_all("profiles");

	let p = FdmProfile::conservative_default();
	println!("CALIBRATE-FDM coupon set — parts (STL+3MF, print-posed at z=0):\n");

	let c_holes = build_coupon_holes();
	let c_fit = build_coupon_fit();
	let c_bore = build_coupon_bore();
	let c_pin = build_coupon_pin();
	let (c_bridge, bridge_len) = build_coupon_bridge();
	let c_walls = build_coupon_walls();
	let c_over = build_coupon_overhang();

	let steep_expect = expected_steep_area(p.max_unsupported_angle);
	let max_span = coupons::BRIDGE_SPANS[coupons::BRIDGE_SPANS.len() - 1];

	let mut ok = true;
	let (o1, m_holes) = emit("parts", "coupon_holes", &c_holes, (0.0, 1e-6), 1e-6, &p);
	let (o2, m_fit) = emit("parts", "coupon_fit", &c_fit, (0.0, 1e-6), 1e-6, &p);
	let (o3, m_bore) = emit("parts", "coupon_bore", &c_bore, (0.0, 1e-6), 1e-6, &p);
	let (o4, m_pin) = emit("parts", "coupon_pin", &c_pin, (0.0, 1e-6), 1e-6, &p);
	// The bridge ladder's 25 mm span and the overhang fan's steep area are the
	// PRODUCT, not defects — their windows assert the audited value matches the
	// designed one (a detector-calibration gate in itself).
	let (o5, m_bridge) = emit("parts", "coupon_bridge", &c_bridge, (0.0, 1e-6), max_span + 0.01, &p);
	let (o6, m_walls) = emit("parts", "coupon_walls", &c_walls, (0.0, 1e-6), 1e-6, &p);
	let (o7, m_over) = emit("parts", "coupon_overhang", &c_over, (steep_expect * 0.98, steep_expect * 1.02), 1e-6, &p);
	ok &= o1 && o2 && o3 && o4 && o5 && o6 && o7;
	println!();

	// ---- STL nominal fidelity: the metrological core ------------------------------
	let hole_feats: Vec<(f64, f64, f64)> = coupons::HOLE_LADDER_D
		.iter()
		.enumerate()
		.map(|(i, d)| (HOLES_X0 + i as f64 * HOLES_PITCH, HOLES_W / 2.0, d / 2.0))
		.collect();
	let (sag_h, lab_h) = worst_bore_sag(&m_holes, &hole_feats, (0.5, HOLES_T as f32 - 0.5));
	gate(
		"STL fidelity: hole ladder chord error ≤ 0.005 mm",
		(0.0..=SAG_MAX + 1e-4).contains(&sag_h),
		format!("worst {:.4} mm ({lab_h})", sag_h),
		&mut ok,
	);
	let fit_feats: Vec<(f64, f64, f64)> = coupons::FIT_BORE_D
		.iter()
		.enumerate()
		.map(|(i, d)| (FIT_X0 + i as f64 * FIT_PITCH, FIT_W / 2.0, d / 2.0))
		.collect();
	let (sag_f, lab_f) = worst_bore_sag(&m_fit, &fit_feats, (0.5, FIT_T as f32 - 0.5));
	gate(
		"STL fidelity: fit ladder chord error ≤ 0.005 mm",
		(0.0..=SAG_MAX + 1e-4).contains(&sag_f),
		format!("worst {:.4} mm ({lab_f})", sag_f),
		&mut ok,
	);
	let (sag_b, _) = worst_bore_sag(
		&m_bore,
		&[(BORE_SQ / 2.0, BORE_SQ / 2.0, coupons::BORE_LARGE_D / 2.0)],
		(0.5, BORE_T as f32 - 0.5),
	);
	gate(
		"STL fidelity: Ø22 gauge chord error ≤ 0.005 mm",
		(0.0..=SAG_MAX + 1e-4).contains(&sag_b),
		format!("{:.4} mm", sag_b),
		&mut ok,
	);
	let (pin_min, pin_max) = m_pin
		.radial_extent(Vec3::ZERO, Vec3::Z, Some((PIN_DISC_T as f32 + 0.5, (PIN_DISC_T + PIN_LEN) as f32 - 0.5)))
		.expect("pin post band");
	let rp = coupons::FIT_PIN_D / 2.0;
	gate(
		"STL fidelity: pin post r max = 3.000, chord error ≤ 0.005",
		(pin_max - rp).abs() < 1e-4 && rp - pin_min <= SAG_MAX + 1e-4,
		format!("max {:.4}  min {:.4}", pin_max, pin_min),
		&mut ok,
	);

	// ---- bridge detector reads the designed ladder --------------------------------
	let rep_bridge = m_bridge.support_free_report(Vec3::Z, p.max_unsupported_angle as f32, 0.3);
	let spans: Vec<f64> = rep_bridge.bridge_patches.iter().map(|(s, _)| *s).take(5).collect();
	let ladder_seen = spans.len() == 5
		&& coupons::BRIDGE_SPANS
			.iter()
			.rev()
			.zip(&spans)
			.all(|(want, got)| (want - got).abs() < 0.01);
	gate(
		"bridge audit measures the designed 5–25 ladder",
		ladder_seen,
		format!("widest-first {:?}", spans.iter().map(|s| (s * 100.0).round() / 100.0).collect::<Vec<_>>()),
		&mut ok,
	);

	// ---- profile: save, reload, and the gate-consumption helpers ------------------
	let prof_path = FdmProfile::profiles_path("conservative_default");
	let saved = p.save(&prof_path).is_ok();
	let reloaded = FdmProfile::load(&prof_path);
	gate(
		"profile: conservative_default.json saved + byte-stable reload",
		saved && reloaded.as_ref().map(|q| q == &p && q.to_json() == p.to_json()).unwrap_or(false),
		prof_path.clone(),
		&mut ok,
	);
	let (tight_d, free_d) = (p.fit_tight_bore_d(coupons::FIT_PIN_D), p.fit_free_bore_d(coupons::FIT_PIN_D));
	let ladder_lo = coupons::FIT_BORE_D[0];
	let ladder_hi = coupons::FIT_BORE_D[coupons::FIT_BORE_D.len() - 1];
	gate(
		"fit ladder straddles the profile's tight/free bores",
		(ladder_lo..=ladder_hi).contains(&tight_d) && (ladder_lo..=ladder_hi).contains(&free_d),
		format!("tight {tight_d:.2}  free {free_d:.2} in [{ladder_lo}, {ladder_hi}]"),
		&mut ok,
	);
	gate(
		"fit helpers reproduce the frozen campaign consts",
		(p.fit_free_shaft_r(37.3) - 37.05).abs() < 1e-12 && (p.fit_tight_shaft_r(4.0) - 3.95).abs() < 1e-12,
		format!("R_TO {:.2} (respool 37.05)  STUB_R {:.2} (drybox 3.95)", p.fit_free_shaft_r(37.3), p.fit_tight_shaft_r(4.0)),
		&mut ok,
	);

	// ---- sibling processes refuse honestly ----------------------------------------
	let refusals = [Process::SheetMetal, Process::Casting, Process::Cnc];
	let all_refuse = refusals.iter().all(|pr| {
		matches!(pr.fdm_profile(), Err(e) if e.to_string().contains("not implemented"))
			&& pr.dfm_checks_mesh(&m_holes).is_err()
	});
	let casting_note = Process::Casting
		.fdm_profile()
		.err()
		.map(|e| e.to_string().contains("draft_analysis"))
		.unwrap_or(false);
	gate(
		"sibling processes refuse loudly (casting names draft_analysis)",
		all_refuse && casting_note,
		format!("{} refusals", refusals.len()),
		&mut ok,
	);

	// ---- DFM checks through the Process surface -----------------------------------
	let proc = Process::Fdm(p.clone());
	let f_holes = proc.dfm_checks_mesh(&m_holes).expect("fdm implements dfm");
	gate(
		"DFM: hole-ladder coupon audits clean",
		f_holes.is_empty(),
		format!("{} findings", f_holes.len()),
		&mut ok,
	);
	let f_walls = proc.dfm_checks_mesh(&m_walls).expect("fdm implements dfm");
	let thin = f_walls.iter().find(|f| f.check == "thin_wall");
	// Only the 0.8 fin sits under min_wall 1.2: two 10×10 faces = 200 mm².
	gate(
		"DFM NC: wall oracle flags exactly the 0.8 fin (~200 mm²)",
		f_walls.len() == 1 && thin.map(|f| (180.0..=230.0).contains(&f.measured)).unwrap_or(false),
		format!("{:?}", thin.map(|f| (f.check, (f.measured * 10.0).round() / 10.0))),
		&mut ok,
	);
	let f_over = proc.dfm_checks_mesh(&m_over).expect("fdm implements dfm");
	let steep = f_over.iter().find(|f| f.check == "support_steep");
	let steep_ok = steep.map(|f| (f.measured - steep_expect).abs() < 0.02 * steep_expect).unwrap_or(false);
	// The ray-based thickness check ALSO flags the leaning fins' horizontal
	// top caps: a vertical ray from a cap-triangle centroid exits through the
	// slant underside after Δx/tan θ, which undercuts min_wall 1.2 for the
	// 40/50/55/60° fins' near-edge centroids — analytically 12+12+12+24 =
	// 60 mm² of 3×8 caps. A real, stated property of direction-dependent ray
	// thickness (Mesh::wall_thickness docs), not a defect of the 3 mm fins —
	// assert the artifact at its analytic size instead of pretending absence.
	let extras: Vec<&kernel_model::process::DfmFinding> = f_over.iter().filter(|f| f.check != "support_steep").collect();
	let extras_ok = extras.iter().all(|f| f.check == "thin_wall")
		&& (40.0..=80.0).contains(&extras.iter().map(|f| f.measured).sum::<f64>());
	gate(
		"DFM NC: steep oracle reads the fan's designed area",
		steep_ok && extras_ok,
		format!(
			"measured {:.1} vs analytic {:.1} mm²; cap artifact {:?}",
			steep.map(|f| f.measured).unwrap_or(f64::NAN),
			steep_expect,
			extras.iter().map(|f| (f.check, f.measured.round())).collect::<Vec<_>>()
		),
		&mut ok,
	);

	// ---- kinematic fit proof on the posed pair ------------------------------------
	// Virtual pin gauge (bare Ø6 post) descending into the profile-recommended
	// free bore (Ø6.5, ladder index 5): free-run must be clean the whole way.
	let free_idx = coupons::FIT_BORE_D
		.iter()
		.position(|d| (*d - free_d).abs() < 1e-9)
		.expect("free bore is on the ladder");
	let bore_x = FIT_X0 + free_idx as f64 * FIT_PITCH;
	let gauge = cylinder(v(0.0, 0.0, 0.0), DVec3::Z, rp, FIT_T + 4.0, seg_for(rp));
	let m_gauge = tessellate_default(&gauge);
	let poses: Vec<DAffine3> = (0..30)
		.map(|i| tr(bore_x, FIT_W / 2.0, 8.0 - 10.0 * (i as f64 / 29.0)))
		.collect();
	let sw = sweep_check(&m_fit, &m_gauge, &poses);
	gate(
		"pin gauge free-runs the Ø6.5 bore (30-pose insertion)",
		sw.contacts == 0 && sw.crossings == 0 && sw.min_clearance > 0.2,
		format!("min_cl {:.3}  contacts {}  crossings {}", sw.min_clearance, sw.contacts, sw.crossings),
		&mut ok,
	);
	// NC: the same gauge shoved 0.4 mm off-centre in the Ø6.0 bore MUST
	// interfere — the exact overlap oracle has to fire (analytic ≈ 12 mm³).
	let bad = gauge.transformed(tr(FIT_X0 + 0.4, FIT_W / 2.0, -2.0));
	let bite = overlap_volume(&c_fit, &bad).unwrap_or(f64::NAN);
	gate(
		"NC: off-centre pin in the Ø6.0 bore interferes",
		bite > 5.0,
		format!("overlap {bite:.1} mm³ (analytic ≈ 12.0)"),
		&mut ok,
	);

	// ---- NC: the support oracle must bite on a wrong orientation ------------------
	let wrong = tessellate_default(&c_holes.transformed(DAffine3::from_rotation_y(30f64.to_radians())))
		.support_free_report(Vec3::Z, p.max_unsupported_angle as f32, 0.3);
	gate(
		"NC: hole plate tilted 30° — steep area must jump",
		wrong.steep_area > 1500.0,
		format!("steep {:8.0} mm²", wrong.steep_area),
		&mut ok,
	);

	// ---- NC: a typo'd profile field must refuse -----------------------------------
	let typo = FdmProfile::from_json(&p.to_json().replace("xy_clearance_free", "xy_clearence_free"));
	gate(
		"NC: profile JSON with a typo'd field refused",
		typo.is_err(),
		typo.err().map(|e| e.to_string().chars().take(24).collect::<String>()).unwrap_or_default(),
		&mut ok,
	);

	// ---- machine-readable nominals + the measurement template ---------------------
	let nominals = serde_json::json!({
		"coupons_version": coupons::VERSION,
		"holes_d": coupons::HOLE_LADDER_D,
		"fit_pin_d": coupons::FIT_PIN_D,
		"fit_bores_d": coupons::FIT_BORE_D,
		"bore_large_d": coupons::BORE_LARGE_D,
		"disc_d": coupons::DISC_D,
		"bridge_spans": coupons::BRIDGE_SPANS,
		"walls_t": coupons::WALL_LADDER_T,
		"overhang_deg": coupons::OVERHANG_DEG,
	});
	let _ = std::fs::write(
		"calibration_system/fdm_coupons/coupon_nominals.json",
		serde_json::to_string_pretty(&nominals).unwrap() + "\n",
	);
	let key = |xs: &[f64], val: serde_json::Value| -> serde_json::Value {
		serde_json::Value::Object(xs.iter().map(|x| (fmt_g(*x), val.clone())).collect())
	};
	// PLACEHOLDER values everywhere: ingest REFUSES this file untouched (name
	// placeholder, negative diameters, non-class strings) — it cannot be
	// mistaken for a real measurement set.
	let example = serde_json::json!({
		"printer_name": "PLACEHOLDER_RENAME_ME",
		"material": "PLACEHOLDER e.g. PLA",
		"nozzle_mm": -1.0,
		"layer_mm": -1.0,
		"bed_mm": [-1.0, -1.0, -1.0],
		"coupons_version": coupons::VERSION,
		"holes": key(&coupons::HOLE_LADDER_D, serde_json::json!(-1.0)),
		"fit": key(&coupons::FIT_BORE_D, serde_json::json!("PLACEHOLDER no_go|press|free")),
		"bore_22": -1.0,
		"pin": {"d_min": -1.0, "d_max": -1.0},
		"disc": {"d_mid": -1.0, "d_first_layer": -1.0},
		"bridge_sag": key(&coupons::BRIDGE_SPANS, serde_json::json!(-1.0)),
		"walls": key(&coupons::WALL_LADDER_T, serde_json::json!("PLACEHOLDER solid|gaps")),
		"overhang": key(&coupons::OVERHANG_DEG, serde_json::json!("PLACEHOLDER clean|rough|fail")),
	});
	let _ = std::fs::write(
		"calibration_system/fdm_coupons/measurements.example.json",
		serde_json::to_string_pretty(&example).unwrap() + "\n",
	);
	gate(
		"nominals + measurement template written",
		std::path::Path::new("calibration_system/fdm_coupons/coupon_nominals.json").exists()
			&& std::path::Path::new("calibration_system/fdm_coupons/measurements.example.json").exists(),
		"coupon_nominals.json, measurements.example.json".to_string(),
		&mut ok,
	);

	// ---- assembly scene (the print-plate layout), CAD, docs -----------------------
	let layout: [(&str, &Solid, f64, f64); 7] = [
		("coupon_holes", &c_holes, 0.0, 0.0),
		("coupon_fit", &c_fit, 0.0, 26.0),
		("coupon_bridge", &c_bridge, 0.0, 52.0),
		("coupon_overhang", &c_over, 0.0, 92.0),
		("coupon_walls", &c_walls, 90.0, 118.0),
		("coupon_bore", &c_bore, 106.0, 52.0),
		("coupon_pin", &c_pin, 40.0, 122.0),
	];
	let mut scene: Option<Mesh> = None;
	let mut step_parts = Vec::new();
	for (name, s, x, y) in &layout {
		let m = mesh_posed(&tessellate_default(s), tr(*x, *y, 0.0));
		let _ = std::fs::write(format!("calibration_system/fdm_coupons/assembly/scene/{name}.stl"), m.to_stl_binary());
		match &mut scene {
			Some(acc) => merge_into(acc, &m),
			None => scene = Some(m),
		}
		step_parts.push((name.to_string(), Solid::clone(s), tr(*x, *y, 0.0)));
		let _ = std::fs::write(format!("calibration_system/fdm_coupons/cad/{name}.step"), export_step(s, name));
	}
	let scene = scene.expect("layout has parts");
	let _ = std::fs::write("calibration_system/fdm_coupons/assembly/assembly.stl", scene.to_stl_binary());
	let bb = scene.aabb();
	let plate_ext = [(bb.max.x - bb.min.x) as f64, (bb.max.y - bb.min.y) as f64, (bb.max.z - bb.min.z) as f64];
	gate(
		"one-plate layout: disjoint scene, fits the profile bed",
		scene.is_watertight() && p.bed_fits(plate_ext),
		format!("{:.0} × {:.0} × {:.0} mm", plate_ext[0], plate_ext[1], plate_ext[2]),
		&mut ok,
	);
	match export_step_assembly(&step_parts, "fdm_coupon_set") {
		Ok(txt) => {
			let _ = std::fs::write("calibration_system/fdm_coupons/cad/coupon_set.step", txt);
		}
		Err(e) => {
			gate("cad: assembly STEP export", false, format!("{e:?}"), &mut ok);
		}
	}
	let sheet_job = serde_json::json!({
		"parts": layout.iter().map(|(n, _, _, _)| serde_json::json!({
			"name": n, "stl": format!("calibration_system/fdm_coupons/assembly/scene/{n}.stl")
		})).collect::<Vec<_>>(),
		"explode": {"axis": [0.0, 0.0, 1.0], "auto": true, "gap_mm": 6},
		"steps": [
			{"order": 1, "text": "Print all seven coupons flat as posed (same settings you will print real parts with)."},
			{"order": 2, "text": "Measure each ladder with calipers per README.md; try the pin in every fit bore; drop a 608 bearing in the Ø22 gauge."},
			{"order": 3, "text": "Copy measurements.example.json to measurements.json and fill every PLACEHOLDER with a measured value."},
			{"order": 4, "text": "Run: python3 tools/ingest_calibration.py calibration_system/fdm_coupons/measurements.json — it writes profiles/<printer>.json."}
		],
		"out_prefix": "calibration_system/fdm_coupons/assembly/coupons",
		"project": "LMCAD",
		"doc_title": "FDM calibration coupon set",
		"date": "2026-07-30"
	});
	let _ = std::fs::write(
		"calibration_system/fdm_coupons/assembly/scene/sheet_job.json",
		serde_json::to_string_pretty(&sheet_job).unwrap() + "\n",
	);

	// ---- generated docs (numbers from THIS run — nothing quotable can stale) ------
	let grams: f64 = [&c_holes, &c_fit, &c_bore, &c_pin, &c_bridge, &c_walls, &c_over]
		.iter()
		.map(|s| volume(s).abs() * PLA)
		.sum();
	write_analysis(sag_h, sag_f, sag_b, pin_max, pin_min, steep_expect, &sw, bite, tight_d, free_d, grams);
	write_design_doc();
	write_readme(tight_d, free_d, bridge_len, grams);
	write_listing(grams);
	write_instructions(&layout, grams);

	println!("\nprinted set: {grams:.0} g PLA solid-equivalent across 7 coupons");
	println!("\nCALIBRATE-FDM: {}", if ok { "ALL GATES PASS" } else { "<<< FAIL" });
	std::process::exit(if ok { 0 } else { 1 });
}

// ---- generated documents ---------------------------------------------------------

#[allow(clippy::too_many_arguments)]
fn write_analysis(
	sag_h: f64,
	sag_f: f64,
	sag_b: f64,
	pin_max: f64,
	pin_min: f64,
	steep_expect: f64,
	sw: &kernel_model::SweepReport,
	bite: f64,
	tight_d: f64,
	free_d: f64,
	grams: f64,
) {
	let analysis = format!(
		r#"# FDM calibration coupons — analysis (GENERATED by calibrate_fdm.rs; every number is from the emitting run)

## Analysis plan (research pass, frozen)

This artifact class is a **dimensional metrology gauge set**: zero external
load, room temperature, no fluid/thermal/modal physics. Required analyses:

| required analysis | answered |
|---|---|
| dimensional fidelity of the digital gauges (STL vs nominal) | **performed** — chord-sagitta gates below |
| printability of every coupon as posed | **performed** — per-part support/watertight/bed gates |
| detector calibration (the audits must read designed defects correctly) | **performed** — bridge-ladder, overhang-fan, thin-wall and interference gates |
| structural / thermal / flow / modal | **not required** — no load case exists; stating this per DESIGN_GUIDE §25.7 |

## STL nominal fidelity (chord error = nominal r − tessellated min r)

Budget: ≤ {SAG_MAX} mm, an order of magnitude under caliper resolution
(±0.02 mm class). Measured this run via `Mesh::radial_extent`:

| feature set | worst chord error (mm) |
|---|---|
| hole ladder Ø3–Ø8 | {sag_h:.4} |
| fit ladder Ø6.0–Ø6.6 | {sag_f:.4} |
| Ø22 gauge | {sag_b:.4} |
| pin post (r max / r min) | {pin_max:.4} / {pin_min:.4} vs 3.0000 |

Facet counts per feature come from `seg_for(r)` (sagitta bound inverted);
polygon vertices lie ON the nominal circle, so bores read ≤ nominal by at
most the budget and the pin reads nominal at vertices.

## Detector calibration

- Overhang fan: analytic steep area beyond 45° = Σ 8·12/cos θ for θ ∈
  {{50, 55, 60}} = {steep_expect:.1} mm²; the audit gate requires the measured
  value within ±2%.
- Bridge ladder: the audit must report the designed spans 25/20/15/10/5
  widest-first (span metric = 2 × deepest interior distance; deck depth
  {BR_DEPTH} mm > 25 keeps the metric reading the GAP, not the depth).
- Thin-wall oracle: only the 0.8 fin sits under `min_wall` 1.2 → expected
  flagged area 2 × 10 × 10 = 200 mm².

## Fit proof (virtual, this run)

- Pin gauge Ø6 free-ran the profile-recommended Ø{free_d:.1} bore over a
  30-pose insertion: min clearance {min_cl:.3} mm, contacts {contacts},
  crossings {crossings}.
- Negative control: the same gauge 0.4 mm off-centre in the Ø6.0 bore
  interfered by {bite:.1} mm³ (analytic lens-sliver estimate ≈ 12 mm³).
- Profile recommendations under conservative_default: tight bore
  Ø{tight_d:.1}, free bore Ø{free_d:.1} — both inside the printed ladder, so a
  measured printer can land anywhere around them.

## Print

{grams:.0} g PLA solid-equivalent for the full set. All seven coupons print
support-free as posed; the bridge ladder's spans and the overhang fan's
steep faces are the PRODUCT (they exercise the printer's limits), and are
claimed as such by their gates rather than waved through.

## Honesty ledger

- These are DIGITAL fidelity and detector-calibration receipts. The physical
  compensations come from the user's calipers via
  `tools/ingest_calibration.py`; nothing here predicts them.
- `z_clearance` is NOT measured by coupon set v1 (needs a mating vertical
  pair); ingest carries the conservative 0.30 forward and prints a note.
- The overhang fan tops out at 60°: a printer cleaner than that reports
  "≥ 60", never an extrapolated number.
"#,
		min_cl = sw.min_clearance,
		contacts = sw.contacts,
		crossings = sw.crossings,
	);
	let _ = std::fs::write("calibration_system/fdm_coupons/analysis/ANALYSIS.md", analysis);
}

fn write_design_doc() {
	let design = format!(
		r#"# FDM calibration coupons — design contract (authored)

## Why this campaign exists

Every LMCAD campaign so far froze printer behaviour as researched consts
(RESPOOL `C_R = 0.25`, DRYBOX `STUB_R = 3.95`, the 6.0 mm bridge gate...).
Those numbers were *researched*, then proven by print — but nothing fed a
user's own printer back into the engine. This coupon set + ingest tool turn
`kernel_model::process::FdmProfile` from a fallback into a measurement.

## Provenance of conservative_default (value | source)

| field | value | frozen source |
|---|---|---|
| xy_clearance_tight | 0.05 | DRYBOX press stub: seat Ø7.9 (STUB_R 3.95) in the 608's Ø8.0 bore — community-proven click fit |
| xy_clearance_free | 0.25 | RESPOOL C_R (tongue ↔ mate wall, twist fit); DESIGN_GUIDE §22.6 proven band 0.2–0.3 |
| z_clearance | 0.30 | RESPOOL CEIL_CLR (lug face ↔ pocket ceiling, axial) |
| hole_diameter_comp | 0.0 | frozen campaigns cut holes at nominal; shrink absorbed by designed clearance |
| bore_comp | 0.0 | DRYBOX seats the 608 without scaling; press ring does the work |
| first_layer_comp | 0.0 | no frozen campaign compensates elephant foot explicitly |
| seam_allowance | 0.0 | RESPOOL's C_R absorbs the seam inside 0.25 |
| max_bridge | 6.0 | RESPOOL per-part emit gate (DRYBOX ships 10.5 — default keeps the tighter bound) |
| max_unsupported_angle | 45 | every campaign's support_free_report threshold |
| min_wall | 1.2 | DRYBOX RIB_T — thinnest wall a frozen campaign ships |
| bed | 250 × 250 × 220 | RESPOOL/DRYBOX emit bed-fit gate |

## Measurement design

- Caliper class ±0.02 mm ⇒ digital gauges must be an order tighter:
  chord-sagitta budget {SAG_MAX} mm/feature, gated every run.
- Fit ladder = designed diametral clearances 0.0–0.6 over the Ø6 pin in 0.1
  steps: brackets the conservative tight (Ø6.1) and free (Ø6.5)
  recommendations from BOTH sides, so a measured printer can land anywhere
  realistic without falling off the ladder.
- The Ø22 gauge is deliberately a 608 bearing's OD: a real bearing is a
  free second gauge everyone in this repo's ecosystem already owns
  (DRYBOX uses four).
- Bridge sag pass threshold (ingest): 0.5 mm — a bridge that droops more
  than 2½ layers at 0.2 mm is not a usable ceiling.
- 45° is absent from the overhang fan on purpose: it sits exactly on the
  default threshold and would make both the machine audit and the user's
  call a coin flip. The fan brackets it (35/40 vs 50/55/60).

## Community failure modes designed against

- "Calibrated" profiles measured with one hole size: the ladder spans
  Ø3–Ø8 because shrink is diameter-dependent; ingest records the mean and
  refuses wild inconsistency (>1 mm deviation = typo class).
- Seam bump ignored in fit math: measured explicitly on the pin
  (Ø max − Ø min) and budgeted once per interface by the fit helpers.
- Elephant foot corrupting the first mm of every fit: measured on the disc
  (Ø first layer − Ø mid), budgeted radially, clamped at 0.
"#
	);
	let _ = std::fs::write("calibration_system/fdm_coupons/analysis/DESIGN.md", design);
}

fn write_readme(tight_d: f64, free_d: f64, bridge_len: f64, grams: f64) {
	let hole_rows: String = coupons::HOLE_LADDER_D
		.iter()
		.enumerate()
		.map(|(i, d)| format!("| {} | Ø{} | {} mm from the chamfered corner |\n", i + 1, fmt_g(*d), HOLES_X0 + i as f64 * HOLES_PITCH))
		.collect();
	let fit_rows: String = coupons::FIT_BORE_D
		.iter()
		.enumerate()
		.map(|(i, d)| format!("| {} | Ø{} | {} mm from the chamfered corner |\n", i + 1, fmt_g(*d), FIT_X0 + i as f64 * FIT_PITCH))
		.collect();
	let readme = format!(
		r#"# FDM CALIBRATION COUPONS — measure your printer into the engine

Print these seven small coupons ({grams:.0} g total), measure them with
calipers, and `tools/ingest_calibration.py` turns your numbers into
`profiles/<your printer>.json` — a measured `FdmProfile` that campaigns use
for fits, bridges, walls and overhangs instead of conservative defaults.

## Folder map

| you're asking... | open |
|---|---|
| what do I print? | `parts/` (all seven coupons — one plate) |
| how do I measure? | this file, below · `assembly/instructions.md` |
| can I modify the design? | `cad/` — STEP for every coupon |
| what does it look like? | `renders/` |
| is it verified? | `analysis/` — generated ANALYSIS.md + authored DESIGN.md |
| how do I share it? | `publish/` |

## Print

All coupons together fit one plate (see `assembly/assembly.stl`). Print them
with the SAME settings you will print real parts with — profile calibration
measures your process, not an idealized one. Suggested: 0.2 mm layers, 2+
walls, any infill (coupons are thin), no supports (none needed — gated).

## Measure (calipers, note every value in mm)

1. **coupon_holes** — hole ladder, ascending from the 45°-chamfered corner:

| # | nominal | centre position |
|---|---|---|
{hole_rows}
   Record each hole's measured diameter (average two perpendicular readings).

2. **coupon_fit + coupon_pin** — try the printed pin in each bore, smallest
   first. Classify each bore: `no_go` (will not enter), `press` (enters with
   firm push, holds), `free` (slides/spins freely):

| # | nominal | centre position |
|---|---|---|
{fit_rows}
   Also measure the pin post: rotate it in the caliper jaws and record the
   smallest (`d_min`) and largest (`d_max`) diameter — the difference is the
   seam bump.

3. **coupon_bore** — measure the Ø22 bore twice, perpendicular; record the
   mean. A 608 bearing should drop in or nearly so.

4. **coupon_pin disc** — measure the disc diameter at mid-height (`d_mid`)
   and right at the first layer (`d_first_layer`) — the difference is your
   elephant foot.

5. **coupon_bridge** — {bridge_len:.0} mm ladder, spans 5/10/15/20/25 mm.
   For each span, measure (or judge) the worst sag of the bridge underside
   in mm. Sag ≤ 0.5 mm counts as clean.

6. **coupon_walls** — fins 0.8/1.2/1.6/2.0/2.4 mm ascending from the thin
   end. Classify each: `solid` (continuous, no gaps between perimeters) or
   `gaps`.

7. **coupon_overhang** — fins leaning 35/40/50/55/60° from vertical,
   ascending lean. Classify each underside: `clean`, `rough` (usable but
   degraded), or `fail` (drooped/curled).

## Record + ingest

```sh
cp calibration_system/fdm_coupons/measurements.example.json \
   calibration_system/fdm_coupons/measurements.json
# edit measurements.json: replace EVERY placeholder with a measured value
python3 tools/ingest_calibration.py calibration_system/fdm_coupons/measurements.json
```

The tool refuses loudly (exit 1, `"ok": false`) on any missing, placeholder,
or inconsistent value — e.g. a `free` bore smaller than a `press` bore, or a
hole deviating more than 1 mm from nominal (typo class). On success it
writes `profiles/<printer_name>.json` and prints the derived profile with
each compensation's sign convention.

Under the conservative default profile, the recommended fit bores for the
Ø6 pin are Ø{tight_d:.1} (tight) and Ø{free_d:.1} (free) — your ladder
results will tell you what YOUR printer wants instead.

## Honesty notes

- `z_clearance` (vertical mating gap) is NOT measured by this coupon set;
  the ingest tool carries the conservative 0.30 mm forward and says so.
- If every wall fin shows gaps, or no overhang fin is clean, the tool
  refuses rather than writing a profile it cannot defend.
- The fan tops out at 60°: cleaner printers get `max_unsupported_angle =
  60` and a note, never an extrapolation.
"#
	);
	let _ = std::fs::write("calibration_system/fdm_coupons/README.md", readme);
}

/// `assembly/instructions.md` — there is nothing to assemble here (the
/// "assembly" is a print plate), so this is the measure-and-ingest workflow
/// with the plate layout generated from the live positions.
fn write_instructions(layout: &[(&str, &Solid, f64, f64); 7], grams: f64) {
	let rows: String = layout
		.iter()
		.map(|(n, _, x, y)| format!("| `{n}` | ({x:.0}, {y:.0}) |\n"))
		.collect();
	let instructions = format!(
		r#"# FDM calibration coupons — workflow (GENERATED)

There is nothing to assemble: the "assembly" is the print plate. The
workflow that turns these coupons into a profile:

## 1. Print ({grams:.0} g PLA solid-equivalent)

Print all seven from `parts/` **with the settings you use for real parts** —
this measures your process, not an ideal one. No supports are needed (gated
every build). `assembly/assembly.stl` is the plate layout used for the
bed-fit gate; its per-coupon origins:

| coupon | plate origin (x, y) mm |
|---|---|
{rows}
## 2. Measure

Follow `../README.md` step by step — hole ladder, fit ladder + pin, Ø22
gauge, disc, bridge sags, wall fins, overhang fan. Record in mm.

## 3. Record

```sh
cp calibration_system/fdm_coupons/measurements.example.json \
   calibration_system/fdm_coupons/measurements.json
```

Replace **every** PLACEHOLDER / −1.0 with a measured value or class.

## 4. Ingest

```sh
python3 tools/ingest_calibration.py calibration_system/fdm_coupons/measurements.json
```

Writes `profiles/<printer_name>.json`. On any missing, placeholder or
self-inconsistent value it prints `{{"ok": false, "errors": [...]}}` and exits
1 — it never guesses.

## 5. Use

Campaigns load the profile and call its fit helpers instead of hard-coded
clearances — see `profiles/README.md`.
"#
	);
	let _ = std::fs::write("calibration_system/fdm_coupons/assembly/instructions.md", instructions);
}

fn write_listing(grams: f64) {
	let listing = format!(
		r#"# Printables listing — FDM calibration coupon set (LMCAD)

**Measure your printer, not someone else's.** Seven tiny coupons
({grams:.0} g total) that feed a machine-readable printer profile: hole
ladder Ø3–Ø8, pin/bore fit ladder (clearances 0.0–0.6 mm), Ø22 bearing
gauge, bridge ladder 5–25 mm, wall ladder 0.8–2.4 mm, overhang fan 35–60°,
and a seam/elephant-foot reference pin.

- Print everything on one plate with your everyday settings.
- Measure with calipers per the README (10 minutes).
- Run the bundled ingest script → `profiles/<your printer>.json`.
- Every LMCAD campaign can then consume YOUR measured clearances,
  compensations, bridge and overhang limits through
  `kernel_model::process::FdmProfile` instead of conservative defaults.

Every coupon is machine-verified on every build: exact-B-rep gauges with
chord error ≤ 0.005 mm, support-free as posed, watertight, one-plate fit —
and the audits themselves are calibrated against designed defects (the
bridge ladder, overhang fan and 0.8 mm wall are supposed to be flagged, and
the build fails if they are not).
"#
	);
	let _ = std::fs::write("calibration_system/fdm_coupons/publish/PRINTABLES_LISTING.md", listing);
}
