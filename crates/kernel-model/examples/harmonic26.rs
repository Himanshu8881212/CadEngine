//! HARM-26 — a 26:1 STRAIN-WAVE (harmonic) drive in the SAME Cricket-class
//! envelope as cyclo26: the whole drive inside the NEMA-17 square (42.3²,
//! chamfered corners, flush with the motor), same interfaces everywhere —
//! register, M3×30 through-bolt sandwich into the motor's own face taps,
//! Ø20 spigot on ONE 6804 (shelf + top retainer), hex-register hub with the
//! 6×M3 Ø20 arm circle. LID, RETAINER and HUB are byte-identical parts with
//! the cyclo drive: the two actuators interchange on the same robot.
//!
//! Strain-wave specifics:
//! - Circular spline: 54 internal trapezoid teeth printed INTO the housing
//!   wall (no separate ring part, no dowels).
//! - Flexspline: thin-wall PETG cup (52 external teeth) whose diaphragm,
//!   Ø24 shoulder, Ø20 spigot and hex register are ONE printed part — the
//!   cup IS the output plate. ratio = −F/2 = −26 exactly.
//! - Wave generator: two 693ZZ bearings (3×8×4) as ROLLERS at the major
//!   axis on a printed carrier; their axles are plain M3×8 screws threaded
//!   into the carrier. No sliding cam, no loose balls.
//! - The as-printed flexspline is ROUND; the assembled state is elastically
//!   deformed (w0 = module at the major axis). 3D booleans cannot model that
//!   deformation, so the tooth mesh and roller preload are verified by the
//!   EXACT deformed-tooth 2D simulator (harmonic26_sim, same constants) and
//!   those two pairs are whitelisted in the 3D interference matrix with
//!   their expected preload volumes. Everything else is gated in 3D as usual.
//!
//! Run: cargo run --example harmonic26 -p kernel-model --release
//! (writes harmonic26/parts/*.stl + ASSEMBLY + STEP; exit 1 on any FAIL)

use kernel_brep::math::{DAffine3, DVec2, DVec3};
use kernel_brep::{
	cone, cylinder, difference, export_step_assembly, extrude, import_step_assembly, intersection, revolve,
	teardrop_hole, tessellate_default, try_difference, union, validate, volume, Solid,
};
use kernel_core::math::Vec3;
use kernel_core::Mesh;
use kernel_model::parts::{button_head_screw, deep_groove_bearing, flat_head_screw, nema_motor, trapezoid_tooth_offsets};
use std::f64::consts::{FRAC_PI_2, PI, TAU};

// ---- parameters (Excel/CSV) ---------------------------------------------------------
#[derive(Clone, Copy)]
struct P {
	teeth_flex: usize, // F; circular spline = F+2; ratio = F/2
	module: f64,
	wall: f64,        // flexspline wall under the tooth roots
	flank_deg: f64,   // trapezoid flank half-angle from radial
	slack: f64,       // circumferential tooth thinning per flank (backlash tunable)
	root_fillet: f64, // flexspline tooth-root relief fillet radius (fatigue hotspot)
	cone_boost: f64,  // raises the diaphragm inner cone + floor by this much, thickening
	// the cone->shoulder junction from the INSIDE (the torsion funnel hotspot found by
	// the v2 full-geometry FEA); 0 = original profile, byte-identical. Outer face and
	// lid clearance untouched; inner-cone support-free slope preserved (parallel shift).
	motor_len: f64,
}

fn load() -> P {
	let mut p = P { teeth_flex: 52, module: 0.6, wall: 1.2, flank_deg: 25.0, slack: 0.05, root_fillet: 0.15, cone_boost: 0.0, motor_len: 58.0 };
	if let Ok(txt) = std::fs::read_to_string("harmonic26/params.csv") {
		for line in txt.lines() {
			let l = line.trim();
			if l.is_empty() || l.starts_with('#') {
				continue;
			}
			let mut it = l.split(',');
			let (k, val) = (it.next().unwrap_or(""), it.next().unwrap_or(""));
			let Ok(x) = val.trim().parse::<f64>() else { continue };
			match k.trim() {
				"teeth_flex" => p.teeth_flex = x as usize,
				"module" => p.module = x,
				"wall" => p.wall = x,
				"flank_deg" => p.flank_deg = x,
				"slack" => p.slack = x,
				"root_fillet" => p.root_fillet = x,
				"cone_boost" => p.cone_boost = x,
				"motor_total_len" => p.motor_len = x,
				_ => {}
			}
		}
	}
	assert!(p.teeth_flex.is_multiple_of(2), "flex tooth count must be even (two-lobe wave)");
	p
}

// ---- the assembled stack (mm, z up, motor face at z = 0) ----
// IDENTICAL numbers to cyclo26 wherever the parts are shared.
const NEMA_W: f64 = 42.3;
const BACK_T: f64 = 5.5;
const REG_D: f64 = 22.3;
const REG_T: f64 = 2.2;
const SHAFT_BORE_D: f64 = 10.0;
const TOOTH_Z0: f64 = 6.0; // tooth band 6..14 (face width 8)
const TOOTH_Z1: f64 = 14.0;
const RING_TOP: f64 = 19.4;
const TOWER_BOT: f64 = 21.9;
const B6804_OD: f64 = 32.0;
const B6804_W: f64 = 7.0;
const B1_Z: f64 = TOWER_BOT + 1.2; // 23.1..30.1
const LIP_Z: f64 = B1_Z + B6804_W; // 30.1
const LID_TOP: f64 = 30.3;
const RET_TOP: f64 = 32.3;
const FACE_Z: f64 = 34.5;
const HEX_AF: f64 = 12.0;
const SPIG_R: f64 = 10.0;
const PLATE_R: f64 = 13.6; // lid cavity datum (shared lid geometry)
const BOLT_SQ: f64 = 15.5;
// Sandwich-bolt stack: M3×SANDWICH_L button heads seat in Ø6.5×SANDWICH_CB lid
// counterbores; tap engagement = SANDWICH_CB + SANDWICH_L − LID_TOP = 4.0 mm,
// inside the ~4.5 mm blind-tap depth of the NEMA-17 face (ICS 16). Gated in
// A-ASM (tap-engagement) — the 2026-07-19 audit found the previous M3×40/cb 3.0
// combination demanded 12.7 mm and would bottom out before clamping (same
// defect fixed in cyclo26 the same day).
const SANDWICH_L: f64 = 30.0; // M3×30 sandwich through-bolt length
const SANDWICH_CB: f64 = 4.3; // lid head-counterbore depth
const IF_PILOT_H: f64 = 2.5;
const ARM_CIRCLE_R: f64 = 10.0;
const ROLLER_OD: f64 = 8.0; // 693ZZ
const ROLLER_Z: f64 = 8.0; // rollers ride 8..12, centred in the tooth band
// four-roller wave generator: a SYMMETRIC roller pair straddles each major-axis
// lobe at ±ROLLER_PHI2 (so 4 rollers total, both lobes doubly supported). φ2 is
// derived so each roller sits tangent to the deformed bore at its own angle; 30°
// halves the worst unsupported rim arc from 180° (two apex rollers) to 120° while
// keeping roller/roller spacing ≥ 8.5 and clearing the +X set-screw boss ≥ 0.5.
const ROLLER_PHI2_DEG: f64 = 30.0;
const SEG: usize = 64;
const SEG_S: usize = 32;
const PLA: f64 = 0.00124;

const _: () = assert!(SHAFT_BORE_D < REG_D - 2.0, "shaft bore must preserve the register shoulder");
const _: () = assert!(LIP_Z == B1_Z + B6804_W, "the retainer lip must land exactly on the race top");

// derived tooth geometry (all from module + counts; asserted in A-GEOM)
fn geo(p: &P) -> Geo {
	let f = p.teeth_flex as f64;
	let c = f + 2.0;
	let rf = p.module * f / 2.0; // flex pitch radius (round state)
	let rc = p.module * c / 2.0; // circular-spline pitch radius
	let w0 = p.module; // radial deflection at the major axis
	let ha = 0.7 * p.module; // addendum (stub teeth print better)
	let hd = 0.9 * p.module; // dedendum
	let bore_r = rf - hd - p.wall;
	Geo {
		rf,
		rc,
		w0,
		flex_tip: rf + ha,
		flex_root: rf - hd,
		circ_tip: rc - ha,
		circ_root: rc + hd,
		bore_r,
		roller_c: bore_r + w0 - ROLLER_OD * 0.5, // apex roller centre (max deflection, reference)
		// offset roller centre at ±φ2: the deformed bore there is BORE_R + w0·cos2φ2,
		// so the tangent centre is that minus the roller radius — all four rollers sit here
		roller_c2: bore_r + w0 * (2.0 * ROLLER_PHI2_DEG.to_radians()).cos() - ROLLER_OD * 0.5,
	}
}
struct Geo {
	rf: f64,
	rc: f64,
	w0: f64,
	flex_tip: f64,
	flex_root: f64,
	circ_tip: f64,
	circ_root: f64,
	bore_r: f64,
	roller_c: f64,
	roller_c2: f64,
}

/// The four wave-roller centres (major axis along ±Y): a symmetric pair
/// straddling each lobe at ±ROLLER_PHI2, all at `roller_c2` so each is tangent
/// to the deformed bore at its own angle (verified in the simulator S4).
fn roller_centers(g: &Geo) -> [(f64, f64); 4] {
	let phi = ROLLER_PHI2_DEG.to_radians();
	let (sx, sy) = (g.roller_c2 * phi.sin(), g.roller_c2 * phi.cos());
	[(sx, sy), (-sx, sy), (sx, -sy), (-sx, -sy)]
}

fn v(x: f64, y: f64, z: f64) -> DVec3 {
	DVec3::new(x, y, z)
}
fn tr(x: f64, y: f64, z: f64) -> DAffine3 {
	DAffine3::from_translation(v(x, y, z))
}
fn ccw(mut p: Vec<DVec2>) -> Vec<DVec2> {
	let a2: f64 = p.windows(2).map(|w| w[0].x * w[1].y - w[1].x * w[0].y).sum::<f64>()
		+ (p[p.len() - 1].x * p[0].y - p[0].x * p[p.len() - 1].y);
	if a2 < 0.0 {
		p.reverse();
	}
	p
}

/// M3 thread-forming pilot (Ø2.5×8), drilled along `into` from `at`.
fn pilot(s: &Solid, at: DVec3, into: DVec3) -> Solid {
	difference(s, &cylinder(at - into * 0.5, into, 1.25, 8.5, 16))
}
fn bore(s: &Solid, face: DVec3, axis: DVec3, d: f64, len: f64, seg: usize) -> Solid {
	difference(s, &cylinder(face - axis * 0.5, axis, d * 0.5, len + 0.5, seg))
}

/// Round the convex corner at `v` (between edges `prev`→`v` and `v`→`next`) with
/// a tangent circular arc of radius `rf`, returning `nseg+1` points that REPLACE
/// `v`. The arc is inscribed on the interior (material) side, so it only REMOVES
/// material — used for the flexspline root-relief fillet, which must never
/// protrude outward into the mesh. Setback is clamped to 70 % of the shorter
/// edge so a short root land can never be over-run; degenerate corners fall
/// back to the sharp vertex.
fn fillet_corner(prev: DVec2, v: DVec2, next: DVec2, rf: f64, nseg: usize) -> Vec<DVec2> {
	let (e1x, e1y) = (prev.x - v.x, prev.y - v.y);
	let (e2x, e2y) = (next.x - v.x, next.y - v.y);
	let (l1, l2) = ((e1x * e1x + e1y * e1y).sqrt(), (e2x * e2x + e2y * e2y).sqrt());
	if l1 < 1e-9 || l2 < 1e-9 {
		return vec![v];
	}
	let (t1x, t1y) = (e1x / l1, e1y / l1);
	let (t2x, t2y) = (e2x / l2, e2y / l2);
	let half = (t1x * t2x + t1y * t2y).clamp(-1.0, 1.0).acos() * 0.5;
	if !(1e-4..=FRAC_PI_2 - 1e-4).contains(&half) {
		return vec![v];
	}
	let setback = (rf / half.tan()).min(0.7 * l1.min(l2));
	let rfe = setback * half.tan();
	let (bx, by) = (t1x + t2x, t1y + t2y);
	let bl = (bx * bx + by * by).sqrt();
	if bl < 1e-9 {
		return vec![DVec2::new(v.x + t1x * setback, v.y + t1y * setback), DVec2::new(v.x + t2x * setback, v.y + t2y * setback)];
	}
	let (cx, cy) = (v.x + (bx / bl) * (rfe / half.sin()), v.y + (by / bl) * (rfe / half.sin()));
	let (p1x, p1y) = (v.x + t1x * setback, v.y + t1y * setback);
	let (p2x, p2y) = (v.x + t2x * setback, v.y + t2y * setback);
	let start = (p1y - cy).atan2(p1x - cx);
	let mut d = (p2y - cy).atan2(p2x - cx) - start;
	while d > PI {
		d -= TAU;
	}
	while d < -PI {
		d += TAU;
	}
	(0..=nseg)
		.map(|i| {
			let a = start + d * (i as f64 / nseg as f64);
			DVec2::new(cx + a.cos() * rfe, cy + a.sin() * rfe)
		})
		.collect()
}

/// Trapezoid tooth ring outline. `external`: teeth point outward (flexspline);
/// otherwise the returned polygon is the CAVITY outline of an internal gear
/// (teeth pointing inward). `thin` shaves each flank circumferentially (the
/// backlash tunable, applied to the FLEX teeth only). `root_fillet` (external
/// only) rounds the sharp leading root↔flank corner — the flexspline fatigue
/// hotspot — cutting into the root without protruding outward.
/// Shared math with harmonic26_sim — keep in sync (asserted there).
#[allow(clippy::too_many_arguments)] // a parametric gear-profile builder: every arg is an independent tooth dimension
fn tooth_ring(n: usize, pitch_r: f64, tip_r: f64, root_r: f64, flank_deg: f64, external: bool, thin: f64, root_fillet: f64) -> Vec<DVec2> {
	let pitch = TAU / n as f64;
	// Tooth-corner geometry from the SHARED library generator — the SAME
	// function the kinematic simulator builds its circular spline and deformed
	// flexspline from, so the printed parts and the verified model cannot
	// desync (a root/tip half-width swap in a private copy of this math once
	// made the printed casing a sawtooth while the sim stayed correct). The
	// generator's taper is unit-tested (trapezoid_tooth_tapers_wide_root_narrow_tip).
	let offs = trapezoid_tooth_offsets(n, pitch_r, tip_r, root_r, flank_deg, external, thin);
	let mut pts: Vec<DVec2> = Vec::with_capacity(n * (offs.len() + 2));
	for k in 0..n {
		// internal (circular-spline) teeth carry the half-pitch phase so a
		// SPACE sits at angle 0 — the assembly datum the simulator verifies
		let c = if external { pitch * k as f64 } else { pitch * (k as f64 + 0.5) };
		let place = |da: f64, r: f64| DVec2::new(r * (c + da).cos(), r * (c + da).sin());
		if external {
			// offsets = [valley, root-lead, tip-lead, tip-trail, root-trail]. A
			// root-relief fillet rounds the sharp leading root↔flank corner (the
			// flexspline fatigue hotspot) — it cuts INTO the root only, never
			// protruding outward, so it cannot add mesh interference.
			let a = place(offs[0].0, offs[0].1);
			let b = place(offs[1].0, offs[1].1);
			let tip_lead = place(offs[2].0, offs[2].1);
			pts.push(a);
			if root_fillet > 0.0 {
				pts.extend(fillet_corner(a, b, tip_lead, root_fillet, 2));
			} else {
				pts.push(b);
			}
			pts.push(tip_lead);
			pts.push(place(offs[3].0, offs[3].1));
			pts.push(place(offs[4].0, offs[4].1));
		} else {
			for (da, r) in &offs {
				pts.push(place(*da, *r));
			}
		}
	}
	ccw(pts)
}

// ---- printed parts -------------------------------------------------------------------

/// Housing = circular spline: the NEMA square with 54 internal trapezoid
/// teeth printed straight into the wall (full cavity height — extra tooth
/// length above the flex band only adds stiffness), TRUE register underneath,
/// through-bolt passages, gabled wire exit. Prints as used; teeth are
/// vertical walls.
fn housing(p: &P) -> Solid {
	let g = geo(p);
	let n_c = p.teeth_flex + 2;
	let h2 = NEMA_W * 0.5;
	let c = 2.5;
	let outline = ccw(vec![
		DVec2::new(h2, -(h2 - c)),
		DVec2::new(h2, h2 - c),
		DVec2::new(h2 - c, h2),
		DVec2::new(-(h2 - c), h2),
		DVec2::new(-h2, h2 - c),
		DVec2::new(-h2, -(h2 - c)),
		DVec2::new(-(h2 - c), -h2),
		DVec2::new(h2 - c, -h2),
	]);
	let mut h = extrude(&outline, RING_TOP);
	// the toothed cavity: one polygon, 54 teeth pointing inward
	let cavity = tooth_ring(n_c, g.rc, g.circ_tip, g.circ_root, p.flank_deg, false, 0.0, 0.0);
	h = difference(&h, &extrude(&cavity, RING_TOP).transformed(tr(0.0, 0.0, BACK_T)));
	// NEMA register + shaft bore + 46° funnel
	h = difference(&h, &cylinder(v(0.0, 0.0, -0.5), DVec3::Z, REG_D * 0.5, REG_T + 0.5, SEG));
	h = bore(&h, v(0.0, 0.0, BACK_T), -DVec3::Z, SHAFT_BORE_D, BACK_T + 2.0, SEG);
	h = difference(&h, &cone(v(0.0, 0.0, REG_T - 0.2), DVec3::Z, REG_D * 0.5 + 0.3, 12.3, SEG));
	for (dx, dy) in [(BOLT_SQ, BOLT_SQ), (-BOLT_SQ, BOLT_SQ), (BOLT_SQ, -BOLT_SQ), (-BOLT_SQ, -BOLT_SQ)] {
		h = bore(&h, v(dx, dy, RING_TOP), -DVec3::Z, 3.4, RING_TOP + 2.0, 16);
	}
	// wire exit low (z≈8.5) — it doubles as the set-screw access port
	h = teardrop_hole(&h, v(0.0, -(h2 + 0.5), 8.5), DVec3::Y, DVec3::Z, 7.0, 6.0, 46.0, None).expect("wire exit");
	h
}

/// Flexspline cup + OUTPUT, one part: 52-tooth thin ring (band 6..14),
/// smooth 1.2 wall, 48° diaphragm cone (support-free both faces), Ø24
/// shoulder (lower-inner-race clamp), Ø20 spigot, hex torque register, hub
/// screw pilots and Ø7 shaft clearance bore. Prints teeth-down on the ring.
fn flex_cup(p: &P) -> Solid {
	let g = geo(p);
	let ring = tooth_ring(p.teeth_flex, g.rf, g.flex_tip, g.flex_root, p.flank_deg, true, p.slack, p.root_fillet);
	let mut f = extrude(&ring, TOOTH_Z1 - TOOTH_Z0).transformed(tr(0.0, 0.0, TOOTH_Z0));
	f = bore(&f, v(0.0, 0.0, TOOTH_Z1 + 1.0), -DVec3::Z, g.bore_r * 2.0, TOOTH_Z1 - TOOTH_Z0 + 3.0, SEG);
	// wall + 48° diaphragm cone + shoulder + spigot (one revolve). cone_boost
	// shifts the whole INNER face (floor + inner cone + bore top) up by b: a
	// parallel shift, so the inner-cone support-free slope is exactly preserved
	// and the cone->shoulder junction gains ~0.7*b of normal thickness (the
	// torsion-funnel hotspot); outer face and every interface are untouched.
	let root = g.flex_root;
	let b = p.cone_boost;
	// inner-cone lower endpoint: keep the baseline 45.8° support-free slope for
	// ANY wall (bore_r moves with wall; a fixed z would flatten a thinner wall's
	// inner cone below 45° — caught by the print audit on the wall-0.8 candidate).
	// At wall 1.2, b 0 this is exactly the original 15.6.
	let z_cone_lo = 19.0 + b - (g.bore_r - 10.55) * (3.4 / 3.31);
	let body = revolve(
		&[
			DVec2::new(g.bore_r, TOOTH_Z1),
			DVec2::new(root, TOOTH_Z1),
			DVec2::new(root, 17.0),
			DVec2::new(12.0, 20.4), // 48° cone outer face
			DVec2::new(12.0, B1_Z),
			DVec2::new(SPIG_R, B1_Z),
			DVec2::new(SPIG_R, LIP_Z),
			DVec2::new(0.05, LIP_Z),
			DVec2::new(0.05, 19.0 + b), // diaphragm centre floor
			DVec2::new(10.55, 19.0 + b),
			DVec2::new(g.bore_r, z_cone_lo), // 45.8° cone inner face (wall ≈ 1.4 + 0.7b)
			DVec2::new(g.bore_r, TOOTH_Z1),
		],
		SEG,
	);
	f = union(&f, &body);
	// hex torque register
	let hexp: Vec<DVec2> = (0..6)
		.map(|k| {
			let a = TAU * k as f64 / 6.0 + PI / 6.0;
			let r = HEX_AF * 0.5 / (PI / 6.0).cos();
			DVec2::new(r * a.cos(), r * a.sin())
		})
		.collect();
	f = union(&f, &extrude(&ccw(hexp), 4.0).transformed(tr(0.0, 0.0, LIP_Z)));
	// motor shaft (tip z=24) spins inside: Ø7 clearance up the spigot centre
	f = bore(&f, v(0.0, 0.0, 18.0), DVec3::Z, 7.0, 8.0, SEG_S);
	// hub bolts: two Ø2.5 thread-forming pilots down the hex boss, 11.0 deep
	// (the stock 8.5 `pilot` bottomed at z 26.1 — the old M3×12 hub screw's tip
	// at 22.5 crashed 3.6 mm of solid before its head could seat, the same
	// class as planetary26's hub screws, 2026-07-19 audit). 11.0 deep clears
	// the M3×10 tip (24.5) by 0.9; an M3×12 could NEVER go full depth — below
	// z 24 its flank lies line-to-line with the Ø5 motor shaft. Gated in A-ASM
	// (hub-tip core-clearance probe).
	for dx in [4.0f64, -4.0] {
		f = difference(&f, &cylinder(v(dx, 0.0, LIP_Z + 4.5), -DVec3::Z, 1.25, 11.0, 16));
	}
	f
}

/// Wave-generator carrier: D-bored hub with set-screw boss (same pattern as
/// the cyclo cam) and a top disc holding two M3×8 screw-axles at ±the major
/// axis — the 693ZZ rollers hang under it, pressing the flexspline bore.
/// Prints hub-down.
fn carrier(p: &P) -> Solid {
	let g = geo(p);
	// centre column 6..15 with the D-bore. r5.8: the four offset rollers sit at
	// c2=10.16, inner edge 6.16, so 5.8 keeps the same 0.36 clearance the old
	// two-roller design held at r6.1 (inner edge 6.46).
	let mut c = cylinder(v(0.0, 0.0, TOOTH_Z0), DVec3::Z, 5.8, 9.0, SEG_S);
	// top disc over the rollers
	c = union(&c, &cylinder(v(0.0, 0.0, 12.2), DVec3::Z, g.roller_c + 2.4, 2.8, SEG));
	let mut dbore = cylinder(v(0.0, 0.0, TOOTH_Z0 - 1.0), DVec3::Z, 2.55, 11.0, SEG_S);
	dbore = difference(&dbore, &cuboid_local(2.0, -3.0, TOOTH_Z0 - 2.0, 4.0, 3.0, 16.0));
	c = difference(&c, &dbore);
	// set-screw boss (flat entry face), pocket aligned with the wire port
	c = union(&c, &cuboid_local(4.6, -3.5, TOOTH_Z0, 8.2, 3.5, 9.5));
	// depth 6.5: must break into the D-flat (x=2.0) — the BOM audit found the
	// original 5.0 stopped 1.2 short of the shaft
	c = teardrop_hole(&c, v(8.2, 0.0, 8.5), -DVec3::X, DVec3::Z, 2.5, 6.5, 46.0, None).expect("carrier set screw");
	// four roller-axle pilots straight down through the disc — a symmetric pair
	// straddling each ±Y lobe at ±φ2 (the set-screw boss sits on +X, cleared by
	// ≥0.5 since the nearest roller centre is at y≈8.8, boss edge at y=3.5)
	for (rx, ry) in roller_centers(&g) {
		c = pilot(&c, v(rx, ry, 15.0), -DVec3::Z);
	}
	c
}
fn cuboid_local(x0: f64, y0: f64, z0: f64, x1: f64, y1: f64, z1: f64) -> Solid {
	kernel_brep::cuboid(v(x0, y0, z0), v(x1, y1, z1))
}

/// Lid — BYTE-IDENTICAL geometry to cyclo26's lid minus the dowel sockets
/// (no dowels in a strain-wave drive). Same envelope, shelf, retainer
/// pilots, counterbores.
fn lid() -> Solid {
	let h2 = NEMA_W * 0.5;
	let c = 2.5;
	let outline = ccw(vec![
		DVec2::new(h2, -(h2 - c)),
		DVec2::new(h2, h2 - c),
		DVec2::new(h2 - c, h2),
		DVec2::new(-(h2 - c), h2),
		DVec2::new(-h2, h2 - c),
		DVec2::new(-h2, -(h2 - c)),
		DVec2::new(-(h2 - c), -h2),
		DVec2::new(h2 - c, -h2),
	]);
	let mut l = extrude(&outline, LID_TOP - RING_TOP).transformed(tr(0.0, 0.0, RING_TOP));
	let cavity = revolve(
		&[
			DVec2::new(0.05, RING_TOP - 1.0),
			DVec2::new(PLATE_R + 0.6, RING_TOP - 1.0),
			DVec2::new(PLATE_R + 0.6, TOWER_BOT),
			DVec2::new(14.0, TOWER_BOT),
			DVec2::new(14.0, TOWER_BOT + 1.2),
			DVec2::new(B6804_OD * 0.5 + 0.05, TOWER_BOT + 1.2),
			DVec2::new(B6804_OD * 0.5 + 0.05, LID_TOP + 1.0),
			DVec2::new(0.05, LID_TOP + 1.0),
		],
		SEG,
	);
	l = difference(&l, &cavity);
	for k in 0..2 {
		let a = PI * k as f64 + FRAC_PI_2;
		l = pilot(&l, v(18.2 * a.cos(), 18.2 * a.sin(), LID_TOP), -DVec3::Z);
	}
	// through-bolt passages + head counterbores. Counterbore depth 4.3 (was 3.0)
	// so a standard M3×30 lands EXACTLY 4.0 mm in the motor's blind face taps:
	// engagement = cb + L − LID_TOP = 4.3 + 30 − 30.3 = 4.0 ≤ the ~4.5 mm NEMA-17
	// (ICS 16) tap depth. The original M3×40 at cb 3.0 demanded 12.7 mm — it
	// BOTTOMS OUT ~8 mm early and the sandwich never clamps (2026-07-19 audit;
	// the A-ASM tap-engagement gate now measures this interface).
	for (dx, dy) in [(BOLT_SQ, BOLT_SQ), (-BOLT_SQ, BOLT_SQ), (BOLT_SQ, -BOLT_SQ), (-BOLT_SQ, -BOLT_SQ)] {
		l = bore(&l, v(dx, dy, LID_TOP), -DVec3::Z, 3.4, LID_TOP - RING_TOP + 2.0, 16);
		l = difference(&l, &cylinder(v(dx, dy, LID_TOP - SANDWICH_CB), DVec3::Z, 3.25, SANDWICH_CB + 1.0, 16));
	}
	l
}

/// Top retainer — IDENTICAL part to cyclo26's.
fn retainer_ring() -> Solid {
	let mut r = cylinder(v(0.0, 0.0, LID_TOP), DVec3::Z, 19.8, RET_TOP - LID_TOP, SEG);
	r = union(
		&r,
		&revolve(
			&[
				DVec2::new(13.6, LIP_Z),
				DVec2::new(15.9, LIP_Z),
				DVec2::new(15.9, LID_TOP + 0.2),
				DVec2::new(13.6, LID_TOP + 0.2),
			],
			SEG,
		),
	);
	r = bore(&r, v(0.0, 0.0, RET_TOP), -DVec3::Z, 27.2, RET_TOP - LIP_Z + 2.0, SEG);
	for k in 0..2 {
		let a = PI * k as f64 + FRAC_PI_2;
		r = bore(&r, v(18.2 * a.cos(), 18.2 * a.sin(), RET_TOP), -DVec3::Z, 3.4, RET_TOP - LID_TOP + 2.0, 16);
	}
	r
}

/// Output hub — IDENTICAL part to cyclo26's.
fn output_hub() -> Solid {
	let mut h = cylinder(v(0.0, 0.0, RET_TOP + 0.2), DVec3::Z, 15.0, FACE_Z - RET_TOP - 0.2, SEG);
	h = union(&h, &cylinder(v(0.0, 0.0, LIP_Z), DVec3::Z, 11.9, RET_TOP + 0.4 - LIP_Z, SEG_S));
	let hexs: Vec<DVec2> = (0..6)
		.map(|k| {
			let a = TAU * k as f64 / 6.0 + PI / 6.0;
			let r = (HEX_AF + 0.3) * 0.5 / (PI / 6.0).cos();
			DVec2::new(r * a.cos(), r * a.sin())
		})
		.collect();
	h = difference(&h, &extrude(&ccw(hexs), 4.4).transformed(tr(0.0, 0.0, LIP_Z - 0.2)));
	for dx in [4.0f64, -4.0] {
		h = bore(&h, v(dx, 0.0, FACE_Z), -DVec3::Z, 3.4, FACE_Z - LIP_Z + 2.0, SEG_S);
		h = difference(&h, &cone(v(dx, 0.0, FACE_Z + 1.0), -DVec3::Z, 3.5, 3.6, SEG_S));
	}
	for k in 0..6 {
		let a = TAU * k as f64 / 6.0 + PI / 6.0;
		h = pilot(&h, v(ARM_CIRCLE_R * a.cos(), ARM_CIRCLE_R * a.sin(), FACE_Z), -DVec3::Z);
	}
	h = union(&h, &cylinder(v(0.0, 0.0, FACE_Z - 0.2), DVec3::Z, 4.0, IF_PILOT_H + 0.2, SEG_S));
	h = difference(&h, &cylinder(v(0.0, 0.0, FACE_Z + IF_PILOT_H - 2.0), DVec3::Z, 3.05, 2.5, SEG_S));
	h
}

// ---- emit / audit ---------------------------------------------------------------------

fn emit(name: &str, s: &Solid) -> bool {
	let val = validate(s);
	let mut printed = s.clone();
	let zmin = tessellate_default(&printed).positions.iter().map(|q| q.z as f64).fold(f64::INFINITY, f64::min);
	printed = printed.transformed(tr(0.0, 0.0, -zmin));
	let mesh = tessellate_default(&printed);
	let rep = mesh.support_free_report(Vec3::Z, 45.0, 0.3);
	let wt = mesh.is_watertight();
	let grams = volume(s).abs() * PLA;
	let ok = val.is_valid() && wt && rep.steep_area < 1e-6 && rep.max_bridge_span <= 12.0;
	let _ = std::fs::write(format!("harmonic26/parts/{name}.stl"), mesh.to_stl_binary());
	println!(
		"  {name:20} valid={:5} wt={wt:5} {}  {grams:4.0}g  {}",
		val.is_valid(),
		if rep.steep_area < 1e-6 {
			format!("sf br≤{:4.1}", rep.max_bridge_span)
		} else {
			format!("steep {:.0}mm²", rep.steep_area)
		},
		if ok { "OK" } else { "<<< FAIL" }
	);
	ok
}

fn merge_into(dst: &mut Mesh, src: &Mesh) {
	let base = dst.positions.len() as u32;
	dst.positions.extend_from_slice(&src.positions);
	dst.indices.extend(src.indices.iter().map(|i| i + base));
}

fn overlap_mm3(a: &Solid, b: &Solid) -> f64 {
	match try_difference(a, b) {
		Ok(r) => (volume(a).abs() - volume(&r).abs()).max(0.0),
		Err(_) => f64::NAN,
	}
}

fn main() {
	let p = load();
	let g = geo(&p);
	let mut ok = true;
	let _ = std::fs::create_dir_all("harmonic26/parts");
	let _ = std::fs::create_dir_all("harmonic26/sim");
	if let Ok(dir) = std::fs::read_dir("harmonic26/parts") {
		for e in dir.flatten() {
			let _ = std::fs::remove_file(e.path());
		}
	}
	println!(
		"HARM-26 CRICKET-CLASS — strain-wave {}:1 for NEMA-17: {}T flex / {}T circ, m={}, body {}×{} sq × {:.0} + motor\n",
		p.teeth_flex / 2,
		p.teeth_flex,
		p.teeth_flex + 2,
		p.module,
		NEMA_W,
		NEMA_W,
		LID_TOP
	);

	// ---- A-GEOM: printable teeth, honest flex strain, insertion squeeze ----
	let pitch_w = PI * p.module - 2.0 * p.slack; // tooth width at the pitch line
	let tip_w = pitch_w - 2.0 * (g.flex_tip - g.rf) * p.flank_deg.to_radians().tan();
	// classic thin-ring bending strain for a two-lobe wave: ε = (t/2)·Δκ,
	// Δκ ≈ 3·w0/rn² (neutral radius rn at mid-wall)
	let rn = g.bore_r + p.wall * 0.5;
	let strain = (p.wall * 0.5) * 3.0 * g.w0 / (rn * rn);
	let squeeze = g.flex_tip - g.circ_tip; // round-state insertion interference
	// leading root↔flank corner (tooth 0) → root-relief fillet: measure the flat
	// root land the fillet leaves behind (must stay positive; gate below).
	let pitch = TAU / p.teeth_flex as f64;
	let hp = pitch / 4.0;
	let thin_ang = p.slack / g.rf;
	let slope = p.flank_deg.to_radians().tan() / g.rf;
	let hr = hp - thin_ang + slope * (g.rf - g.flex_root);
	let ht = (hp - thin_ang - slope * (g.flex_tip - g.rf)).max(0.06 / g.flex_tip);
	let a_pt = DVec2::new(g.flex_root * (hr - 2.0 * hp).cos(), g.flex_root * (hr - 2.0 * hp).sin());
	let b_pt = DVec2::new(g.flex_root * (-hr).cos(), g.flex_root * (-hr).sin());
	let c_pt = DVec2::new(g.flex_tip * (-ht).cos(), g.flex_tip * (-ht).sin());
	let land_before = ((b_pt.x - a_pt.x).powi(2) + (b_pt.y - a_pt.y).powi(2)).sqrt();
	let fil = fillet_corner(a_pt, b_pt, c_pt, p.root_fillet, 2);
	let land_after = ((fil[0].x - a_pt.x).powi(2) + (fil[0].y - a_pt.y).powi(2)).sqrt();
	// roller-pair centre spacing (Ø8 bodies must not clash) and boss clearance
	let roll_gap = 2.0 * g.roller_c2 * ROLLER_PHI2_DEG.to_radians().sin() - ROLLER_OD;
	let boss_clear = (g.roller_c2 * ROLLER_PHI2_DEG.to_radians().cos()) - 3.5 - ROLLER_OD * 0.5;
	let geom_ok = tip_w >= 0.55
		&& pitch_w >= 0.8
		&& strain <= 0.02
		&& squeeze < 0.5
		&& g.roller_c > 6.0
		&& g.roller_c2 > 6.0
		&& land_after > 0.05
		&& roll_gap >= 0.5
		&& boss_clear >= 0.5;
	ok &= geom_ok;
	println!(
		"A-GEOM: tooth width pitch {pitch_w:.2}/tip {tip_w:.2} ≥ 0.8/0.55 · PETG wall strain {:.2}% ≤ 2% · insertion squeeze {squeeze:.2} (flexes in)  {}",
		strain * 100.0,
		if geom_ok { "OK" } else { "<<< FAIL" }
	);
	println!(
		"        4-roller wave gen: φ2={ROLLER_PHI2_DEG:.0}° c2={:.2}/apex c={:.2} · roller pair gap {roll_gap:.2}≥0.5 · boss clear {boss_clear:.2}≥0.5 · root-relief fillet r={:.2} leaves land {land_before:.3}→{land_after:.3}",
		g.roller_c2, g.roller_c, p.root_fillet
	);

	// ---- build + print-audit the 6 printed parts ----
	let house = housing(&p);
	let flex = flex_cup(&p);
	let carr = carrier(&p);
	let lid_p = lid();
	let retainer = retainer_ring();
	let hub = output_hub();
	for (name, s) in [
		("housing_circspline", &house),
		("flexspline_output", &flex),
		("wave_carrier", &carr),
		("lid_ring", &lid_p),
		("retainer_ring", &retainer),
		("output_hub", &hub),
	] {
		ok &= emit(name, s);
	}

	// ---- assembly ----
	let motor = nema_motor(17, 48.0).expect("nema17");
	let b6804 = deep_groove_bearing("6804").expect("6804");
	let b693 = deep_groove_bearing("693").expect("693zz");
	let m3x30 = button_head_screw(3.0, SANDWICH_L).expect("m3x30");
	let m3x8 = button_head_screw(3.0, 8.0).expect("m3x8");
	let m3x10f = flat_head_screw(3.0, 10.0).expect("m3x10 csk");
	let m3set = kernel_model::parts::set_screw(3.0, 5.0).expect("m3x5 din916");

	let mut instances: Vec<(String, Solid, DAffine3)> = Vec::new();
	let place = |list: &mut Vec<(String, Solid, DAffine3)>, n: &str, s: &Solid, x: DAffine3| {
		list.push((n.to_string(), s.clone(), x));
	};
	place(&mut instances, "hw_nema17", &motor, tr(0.0, 0.0, 0.0));
	place(&mut instances, "housing_circspline", &house, tr(0.0, 0.0, 0.0));
	place(&mut instances, "wave_carrier", &carr, tr(0.0, 0.0, 0.0));
	place(&mut instances, "hw_m3x5_set", &m3set, tr(2.1, 0.0, 8.5) * DAffine3::from_rotation_y(FRAC_PI_2));
	// four rollers + their screw axles: a symmetric pair straddling each ±Y lobe
	// at ±φ2, every one tangent to the deformed bore (S4 gates all four)
	for (rx, ry) in roller_centers(&g) {
		place(&mut instances, "hw_bearing_693zz", &b693, tr(rx, ry, ROLLER_Z));
		// shank base 7.0 = disc top 15.0 − 8: the button head seats ON the disc
		// top. (The old ROLLER_Z − 1.6 = 6.4 buried the head base 0.6 mm INSIDE
		// the disc — it could never seat; 2026-07-19 audit. Tip at 7.0 hangs in
		// free air below the disc — the roller bore 8..12 stays fully on the shank.)
		place(&mut instances, "hw_m3x8_axle", &m3x8, tr(rx, ry, 7.0));
	}
	place(&mut instances, "flexspline_output", &flex, tr(0.0, 0.0, 0.0));
	place(&mut instances, "lid_ring", &lid_p, tr(0.0, 0.0, 0.0));
	for (dx, dy) in [(BOLT_SQ, BOLT_SQ), (-BOLT_SQ, BOLT_SQ), (BOLT_SQ, -BOLT_SQ), (-BOLT_SQ, -BOLT_SQ)] {
		place(&mut instances, "hw_m3x30_sandwich", &m3x30, tr(dx, dy, LID_TOP - SANDWICH_CB - SANDWICH_L));
	}
	place(&mut instances, "hw_bearing_6804", &b6804, tr(0.0, 0.0, B1_Z));
	place(&mut instances, "retainer_ring", &retainer, tr(0.0, 0.0, 0.0));
	for k in 0..2 {
		let a = PI * k as f64 + FRAC_PI_2;
		place(&mut instances, "hw_m3x8_retainer", &m3x8, tr(18.2 * a.cos(), 18.2 * a.sin(), RET_TOP - 8.0));
	}
	place(&mut instances, "output_hub", &hub, tr(0.0, 0.0, 0.0));
	for dx in [4.0f64, -4.0] {
		place(&mut instances, "hw_m3x10_hub", &m3x10f, tr(dx, 0.0, FACE_Z - 10.0));
	}

	let mut asm = Mesh::default();
	for (_, s, x) in &instances {
		merge_into(&mut asm, &tessellate_default(&s.transformed(*x)));
	}
	let _ = asm.write_stl_binary("harmonic26/ASSEMBLY.stl");
	// exploded view
	let mut expl = Mesh::default();
	for (n, s, x) in &instances {
		let lift = match n.as_str() {
			"hw_nema17" => 0.0,
			"housing_circspline" => 26.0,
			"wave_carrier" => 60.0,
			"hw_bearing_693zz" => 80.0,
			"hw_m3x8_axle" => 100.0,
			"flexspline_output" => 130.0,
			"lid_ring" => 170.0,
			"hw_bearing_6804" => 200.0,
			"retainer_ring" => 225.0,
			"hw_m3x8_retainer" => 248.0,
			"output_hub" => 270.0,
			"hw_m3x10_hub" => 295.0,
			"hw_m3x30_sandwich" => 320.0,
			_ => 0.0,
		};
		merge_into(&mut expl, &tessellate_default(&s.transformed(tr(0.0, 0.0, lift) * *x)));
	}
	let _ = expl.write_stl_binary("harmonic26/ASSEMBLY_EXPLODED.stl");

	// ---- A-ASM: posed contacts and fits (3D, undeformed state) ----
	let mesh_of = |name: &str, nth: usize| -> Mesh {
		let mut c = 0;
		for (n, s, x) in &instances {
			if n == name {
				if c == nth {
					return tessellate_default(&s.transformed(*x));
				}
				c += 1;
			}
		}
		unreachable!("{name}[{nth}]")
	};
	let rel = |label: &str, a: &Mesh, b: &Mesh, expect_contact: bool, ok: &mut bool| {
		let d = a.min_distance(b);
		let good = if expect_contact { d < 0.02 } else { d >= 0.10 };
		*ok &= good;
		println!("  {label:48} min_dist={d:7.3}  {}", if good { "OK" } else { "<<< FAIL" });
	};
	println!();
	let house_m = mesh_of("housing_circspline", 0);
	let flex_m = mesh_of("flexspline_output", 0);
	let lid_m = mesh_of("lid_ring", 0);
	let hub_m = mesh_of("output_hub", 0);
	let ret_m = mesh_of("retainer_ring", 0);
	let ob1 = mesh_of("hw_bearing_6804", 0);
	let motor_m = mesh_of("hw_nema17", 0);
	let carr_m = mesh_of("wave_carrier", 0);
	rel("motor pilot seats in the register", &motor_m, &house_m, true, &mut ok);
	rel("lid seats on the ring wall", &lid_m, &house_m, true, &mut ok);
	rel("6804 inner seats on the flexspline shoulder", &ob1, &flex_m, true, &mut ok);
	rel("shelf carries the outer race", &ob1, &lid_m, true, &mut ok);
	rel("retainer lip lands on the outer race", &ob1, &ret_m, true, &mut ok);
	rel("retainer seats on the lid top", &ret_m, &lid_m, true, &mut ok);
	rel("hub clamps the inner race", &hub_m, &ob1, true, &mut ok);
	rel("carrier clears the housing everywhere", &carr_m, &house_m, false, &mut ok);
	// the rollers PRELOAD the flex bore by w0 — that pair is the deformation
	// (2D-sim territory); here gate the ROUND-state penetration ≈ the design w0
	let roll_m = mesh_of("hw_bearing_693zz", 0);
	let roll_pen = {
		let (idx, mut c) = (0usize, 0usize);
		let mut s0 = None;
		for (n, s, x) in &instances {
			if n == "hw_bearing_693zz" {
				if c == idx {
					s0 = Some(s.transformed(*x));
				}
				c += 1;
			}
		}
		let ix = intersection(&s0.unwrap(), &flex);
		if ix.face_count() == 0 {
			0.0
		} else {
			volume(&ix).abs()
		}
	};
	let pen_ok = (1.0..=40.0).contains(&roll_pen);
	ok &= pen_ok;
	let w_local = g.w0 * (2.0 * ROLLER_PHI2_DEG.to_radians()).cos();
	println!(
		"  roller[0] preloads the flex bore (round-state lens {roll_pen:.1} mm³, local defl {w_local:.2} at φ2)   {}",
		if pen_ok { "OK" } else { "<<< FAIL" }
	);
	let _ = roll_m;
	// screws: bites + free passes
	let engage = |label: &str, screw: &Solid, x: DAffine3, part: &Solid, ok: &mut bool| {
		let bite = overlap_mm3(&screw.transformed(x), part);
		let okb = (3.0..=45.0).contains(&bite);
		*ok &= okb;
		println!("  {label:48} bite={bite:6.1} mm³  {}", if okb { "OK" } else { "<<< FAIL" });
	};
	let axle_x = instances.iter().find(|(n, _, _)| n == "hw_m3x8_axle").map(|(_, _, x)| *x).unwrap();
	engage("roller axle threads the carrier pilot", &m3x8, axle_x, &carr, &mut ok);
	let hub_x = instances.iter().find(|(n, _, _)| n == "hw_m3x10_hub").map(|(_, _, x)| *x).unwrap();
	engage("hub screw threads the flexspline pilot", &m3x10f, hub_x, &flex, &mut ok);
	// hub-screw TIP core clearance: the csk screw must never bottom in its blind
	// pilot. A Ø1.9 probe (slides inside the Ø2.5 pilot) spanning hex-boss top →
	// 0.8 mm below the tip must meet ONLY pilot void. Falsifiable: the
	// pre-2026-07-19 M3×12 + 8.5-deep-pilot build reads ~7 mm³ of solid here —
	// its tip bottomed 3.6 mm before the head seated (planetary26's hub-screw
	// class). M3×12 is unfixable by deeper drilling alone: below z 24 its flank
	// lies line-to-line with the Ø5 motor shaft — hence M3×10 + the 11.0 pilot.
	let hub_tip = FACE_Z - 10.0; // M3×10 csk measures tip-to-head-top
	let probe = cylinder(v(4.0, 0.0, hub_tip - 0.8), DVec3::Z, 0.95, LIP_Z + 4.0 - (hub_tip - 0.8), SEG_S);
	let tip_clash = overlap_mm3(&probe, &flex);
	let tip_ok = tip_clash < 0.05;
	ok &= tip_ok;
	println!("  hub screw tip core clearance (Ø1.9 probe)        clash={tip_clash:.3} mm³  {}", if tip_ok { "OK" } else { "<<< FAIL" });
	let ret_x = instances.iter().find(|(n, _, _)| n == "hw_m3x8_retainer").map(|(_, _, x)| *x).unwrap();
	engage("retainer screw threads the lid pilot", &m3x8, ret_x, &lid_p, &mut ok);
	let bolt_x = instances.iter().find(|(n, _, _)| n == "hw_m3x30_sandwich").map(|(_, _, x)| *x).unwrap();
	let (fh, fl) = (overlap_mm3(&m3x30.transformed(bolt_x), &house), overlap_mm3(&m3x30.transformed(bolt_x), &lid_p));
	let bolt_ok = fh < 0.05 && fl < 0.05;
	ok &= bolt_ok;
	println!("  sandwich bolt passes lid + housing freely ({fh:.2}/{fl:.2} mm³)  {}", if bolt_ok { "OK" } else { "<<< FAIL" });
	// tap ENGAGEMENT: the sandwich bolt threads the motor's BLIND face taps —
	// NEMA-17 (ICS 16) taps are only ~4.5 mm deep, so the shank must land in the
	// usable 3.0–4.5 mm window: shallower strips the steel threads' bite, deeper
	// BOTTOMS OUT before the head clamps (the pre-2026-07-19 M3×40 build demanded
	// 12.7 mm — the sandwich could never clamp). The motor envelope is solid, so
	// depth = overlap / (π·r²) is exact up to shank faceting.
	let tap_bite = overlap_mm3(&m3x30.transformed(bolt_x), &motor);
	let tap_depth = tap_bite / (PI * 1.5 * 1.5); // Ø3 shank
	let tap_ok = (3.0..=4.5).contains(&tap_depth);
	ok &= tap_ok;
	println!("  sandwich bolt engages the motor taps             depth={tap_depth:5.2} mm (blind tap ~4.5)  {}", if tap_ok { "OK" } else { "<<< FAIL" });
	// interference-volume spin gates (shared datum planes)
	let fl_ov = overlap_mm3(&flex, &lid_p);
	let hb_ov = overlap_mm3(&hub, &lid_p);
	let spin_ok = fl_ov < 0.05 && hb_ov < 0.05;
	ok &= spin_ok;
	println!("  flexspline + hub spin free of the lid ({fl_ov:.3}/{hb_ov:.3} mm³)   {}", if spin_ok { "OK" } else { "<<< FAIL" });

	// ---- A-CAPTURE: negative controls ----
	let up_ov = overlap_mm3(&b6804.transformed(tr(0.0, 0.0, B1_Z + 0.5)), &retainer);
	let dn_ov = overlap_mm3(&b6804.transformed(tr(0.0, 0.0, B1_Z - 0.5)), &lid_p);
	let cap_ok = up_ov > 5.0 && dn_ov > 5.0;
	ok &= cap_ok;
	println!(
		"A-CAPTURE: bearing +0.5 hits the retainer ({up_ov:.2} mm³); −0.5 hits the shelf ({dn_ov:.2} mm³) — held BOTH ways  {}",
		if cap_ok { "OK" } else { "<<< FAIL" }
	);

	// ---- A-PROD: pairwise interference matrix ----
	println!("\nA-PROD (pairwise interference matrix, representative bodies):");
	let rep: Vec<(&str, usize)> = vec![
		("hw_nema17", 0),
		("housing_circspline", 0),
		("wave_carrier", 0),
		("hw_bearing_693zz", 0),
		("hw_bearing_693zz", 1),
		("hw_bearing_693zz", 2),
		("hw_bearing_693zz", 3),
		("hw_m3x8_axle", 0),
		("hw_m3x8_axle", 1),
		("hw_m3x8_axle", 2),
		("hw_m3x8_axle", 3),
		("flexspline_output", 0),
		("lid_ring", 0),
		("hw_bearing_6804", 0),
		("retainer_ring", 0),
		("output_hub", 0),
		("hw_m3x30_sandwich", 0),
		("hw_m3x8_retainer", 0),
		("hw_m3x10_hub", 0),
		("hw_m3x5_set", 0),
	];
	// designed-contact pairs (verified by their dedicated gates / the 2D sim)
	const CONTACT_PAIRS: &[(&str, &str, f64)] = &[
		("hw_nema17", "housing_circspline", 1.0),
		("wave_carrier", "hw_nema17", 30.0),          // D-bore on the shaft
		("hw_m3x8_axle", "wave_carrier", 25.0),       // axle thread bite
		("hw_m3x8_axle", "hw_bearing_693zz", 20.0),   // shank inside the roller bore
		("hw_bearing_693zz", "flexspline_output", 40.0), // the w0 preload lens (2D-sim verified)
		("flexspline_output", "housing_circspline", 40.0), // round-state tooth squeeze (2D-sim verified)
		("hw_bearing_6804", "flexspline_output", 1.0),
		("hw_bearing_6804", "output_hub", 1.5),
		("flexspline_output", "output_hub", 1.0),     // hex register mate
		("hw_bearing_6804", "retainer_ring", 1.5),
		("hw_bearing_6804", "lid_ring", 1.0),
		("hw_m3x8_retainer", "lid_ring", 25.0),
		("hw_m3x8_retainer", "retainer_ring", 5.0),
		("hw_m3x30_sandwich", "hw_nema17", 30.0),
		("hw_m3x5_set", "wave_carrier", 20.0),   // threads the printed pocket
		("hw_m3x5_set", "hw_nema17", 2.0),       // tip on the shaft flat
		("hw_m3x10_hub", "flexspline_output", 25.0),
		("hw_m3x10_hub", "output_hub", 8.0),
	];
	let allowed = |a: &str, b: &str| -> f64 {
		CONTACT_PAIRS
			.iter()
			.find(|(x, y, _)| (a == *x && b == *y) || (a == *y && b == *x))
			.map(|(_, _, lim)| *lim)
			.unwrap_or(0.05)
	};
	let solid_of = |name: &str, nth: usize| -> Option<Solid> {
		let mut c = 0;
		for (n, s, x) in &instances {
			if n == name {
				if c == nth {
					return Some(s.transformed(*x));
				}
				c += 1;
			}
		}
		None
	};
	let mut prod_bad = 0;
	for i in 0..rep.len() {
		for j in (i + 1)..rep.len() {
			let (an, ai) = rep[i];
			let (bn, bi) = rep[j];
			let lim = allowed(an, bn);
			if lim > 0.05 {
				continue;
			}
			let (Some(sa), Some(sb)) = (solid_of(an, ai), solid_of(bn, bi)) else { continue };
			let (ma, mb) = (tessellate_default(&sa), tessellate_default(&sb));
			let fold = |m: &Mesh| {
				m.positions
					.iter()
					.fold((Vec3::splat(f32::INFINITY), Vec3::splat(f32::NEG_INFINITY)), |(l, h), q| (l.min(*q), h.max(*q)))
			};
			let ((la, ha), (lb, hb)) = (fold(&ma), fold(&mb));
			if la.x > hb.x + 0.01 || lb.x > ha.x + 0.01 || la.y > hb.y + 0.01 || lb.y > ha.y + 0.01 || la.z > hb.z + 0.01 || lb.z > ha.z + 0.01 {
				continue;
			}
			let ix = intersection(&sa, &sb);
			let ov = if ix.face_count() == 0 { 0.0 } else { volume(&ix).abs() };
			if !(ov.is_finite() && ov <= lim.max(0.2)) {
				prod_bad += 1;
				println!("  <<< {an}[{ai}] ∩ {bn}[{bi}] = {ov:.2} mm³ (limit {lim})");
			}
		}
	}
	ok &= prod_bad == 0;
	println!(
		"  {} representative pairs checked, {prod_bad} violations  {}",
		rep.len() * (rep.len() - 1) / 2,
		if prod_bad == 0 { "OK" } else { "<<< FAIL" }
	);

	// ---- A-STEP round trip ----
	match export_step_assembly(&instances, "harm26") {
		Ok(step) => {
			let _ = std::fs::write("harmonic26/ASSEMBLY.step", &step);
			match import_step_assembly(&step) {
				Ok(back) => {
					let v0: f64 = instances.iter().map(|(_, s, _)| volume(s).abs()).sum();
					let v1: f64 = back.iter().map(|(_, s, _)| volume(s).abs()).sum();
					let dv = (v0 - v1).abs() / v0;
					let sok = back.len() == instances.len() && dv < 0.025;
					ok &= sok;
					println!(
						"\nA-STEP: {} instances, {} KB, round-trip Δ {:.2}%  {}",
						instances.len(),
						step.len() / 1024,
						dv * 100.0,
						if sok { "OK" } else { "<<< FAIL" }
					);
				}
				Err(e) => {
					ok = false;
					println!("\nA-STEP: re-import FAILED: {e:?}  <<< FAIL");
				}
			}
		}
		Err(e) => {
			ok = false;
			println!("\nA-STEP: export FAILED: {e:?}  <<< FAIL");
		}
	}

	// ---- BOM ----
	println!("\nBOM (hardware in the ASSEMBLY only — parts/ is the print queue):");
	println!("  1× NEMA-17 + driver · 1× 6804 · 4× 693ZZ (four-roller wave generator)");
	println!("  M3 ONLY (13 total): 4×30 sandwich · 4×8 roller axles · 2×8 retainer · 2×10 csk hub (hex carries torque) · 1×5 DIN916 set");
	println!("  Deformed-tooth kinematics: cargo run --example harmonic26_sim (exit 1 on FAIL)");

	println!("\nRESULT: {}", if ok { "PASS — every gate green" } else { "FAIL — see <<< lines" });
	std::process::exit(if ok { 0 } else { 1 });
}
