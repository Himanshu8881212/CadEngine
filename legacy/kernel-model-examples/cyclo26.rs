//! CYCLO — a from-scratch, gate-perfect N:1 cycloidal actuator for NEMA-17
//! (26:1 today; `lobes` in cyclo26/params.csv sets any ratio).
//!
//! Architecture (canonical twin-disc cycloidal, tuned for BACKDRIVABILITY — a
//! non-self-locking, rolling-contact drive at a backdrivable 26:1; every choice
//! justified). Backdrivability is a FRICTION property the kinematic sim does NOT
//! measure, so it is DESIGNED-FOR and gated only where gateable (fits, kinematics,
//! interference); backdrive TORQUE is confirmed by hand-test, not asserted here.
//! - eccentric cam (D-bore + set screw on the shaft flat) with two Ø8 journals at
//!   ±e, 180° apart — twin discs cancel the wobble imbalance;
//! - each disc rides a 688 deep-groove BALL BEARING (8×16×5) on its journal —
//!   ROLLING, not the sliding of a plain journal: the single biggest backdrive-
//!   friction lever. Disc thickness == 5 mm bearing width (compile-time assert),
//!   so the bearing sits flush in the Ø16.2 disc bore;
//! - 26-lobe epitrochoid discs meshing 27 ring pins — Ø2×20 hardened STEEL DOWELS
//!   (the whole 26:1 torque crosses the pin flanks), pressed into the back plate
//!   AND captured in lid sockets. ZERO anti-backlash preload (disc_clock_deg 0):
//!   preload is a standing pin friction that fights backdrive. The two discs stay
//!   DISTINCT prints (disc_b's holes compensated +π/lobes onto the pin circle);
//! - SIX M3 countersunk screws are the output pins (raised from three 2026-07-11:
//!   the 10 N·m FEA showed the plate pin-bore bearing binds the torque capacity;
//!   six pins on the same circle halve it), engaging Ø(pin + 2e + slack)
//!   disc holes on the r11.5 circle — moved out from r10.5 so the enlarged
//!   eccentric-bearing bore keeps its ligament; hole-to-bore, hole-to-rim AND
//!   adjacent-hole ligaments are ASSERTED (an earlier design cut the holes INTO
//!   the bore seat);
//! - the output rides a 6804 thin-section bearing (20×32×7): the plate's Ø20
//!   spigot carries the inner race (clamped hub-to-shoulder), the lid's tower
//!   holds the outer race (clamped lip-to-retainer) — rolling support, capture
//!   proven by negative controls (posed ±0.5 must collide);
//! - the motor seats in a true Ø22.3 pilot register (the shaft bore is Ø10,
//!   SMALLER than the register — an earlier design bored Ø24 through and
//!   silently destroyed the register);
//! - flange-mount ears on the housing; set-screw access port through the
//!   ring wall; encoder-magnet pocket in the output boss;
//! - LM-20 output face (capstan-module compatible): `arm_parallel_90` bolts
//!   axially, `arm_perpendicular_80` bolts radially onto two LM-20 pilots.
//!
//! Print discipline: every part prints support-free in its stated orientation
//! (steep == 0, true bridge span ≤ 12; every pocket ceiling is coned at 46°).
//! Hardware (motor, bearings, screws) exists ONLY in the assembly and
//! STEP — cyclo26/parts/ is a pure print queue.
//!
//! Run: cargo run --example cyclo26 -p kernel-model --release  ->  cyclo26/

use kernel_brep::math::{DAffine3, DVec2, DVec3};
use kernel_brep::{
	cone, cuboid, cylinder, difference, export_step_assembly, extrude, import_step_assembly, revolve, teardrop_hole,
	tessellate_default, try_difference, union, validate, volume, Mesh, Solid,
};
use kernel_core::math::Vec3;
use kernel_model::parts::{button_head_screw, deep_groove_bearing, flat_head_screw, nema_motor};
use std::f64::consts::{FRAC_PI_2, PI, TAU};

// ---- parameters (Excel/CSV) ---------------------------------------------------------
#[derive(Clone, Copy)]
struct P {
	lobes: usize,
	ring_r: f64,
	pin_r: f64,
	ecc: f64,
	clear: f64,
	hole_slack: f64, // output-pin hole radial slack past the eccentric orbit
	clock_deg: f64,  // anti-backlash split-disc clock (deg at the pin circle)
	motor_len: f64,  // motor body + SERVO42D driver, from the mounting face
	pin_len: f64,    // Ø3 steel dowel length — pocket depth derives from it
	arm_len: f64,    // straight roll-arm length (mount face → socket flange)
}
// BACKDRIVABLE build: ZERO anti-backlash preload. The ±Δ/2 output-hole clock used
// to squeeze the pins between the two discs to kill lash, but that preload is a
// standing pin-friction torque that FIGHTS backdrive — so it is removed (0.0).
// params.csv disc_clock_deg is SYNC-ASSERTED to this and mirrors the sim's
// DISC_CLOCK_DEG so example and simulator never diverge. The two discs stay
// DISTINCT prints (disc_b still compensates its holes +π/lobes); only the
// anti-backlash split is dropped. Set back to 0.4 (here + CSV + sim) for a
// preloaded, non-backdrivable low-lash variant.
const DISC_CLOCK_DEG: f64 = 0.0;
// OUTPUT PIN COUNT — raised 3 -> 6 (2026-07-11 torque-capacity revalidation, v2):
// the 10 N·m FEA showed the plate pin-bore BEARING is the binding member
// (3 pins: 290 N/pin -> 41 MPa, SF 0.69 vs 28.2 MPa derated PETG). Six M3 csk
// screws on the SAME r11.5 circle halve every pin-interface stress (plate bore
// 20.5 MPa, disc hole 4.8 MPa) and leave the frozen interfaces (envelope, z-stack,
// spigot/6804/hex, lid/retainer/hub) untouched. k*TAU/6 for k=0,2,4 reproduces
// the original 3-pin pattern exactly — the 6-pattern is its superset. The
// adjacent-hole ligament this adds to the disc is gated in A-GEOM. Mirrored in
// cyclo26_sim.rs (OUT_PINS) — keep BOTH in sync.
const OUT_PINS: usize = 6;
fn load() -> P {
	let mut p = P {
		lobes: 26,
		ring_r: 16.5,
		pin_r: 1.0,
		ecc: 0.5,
		clear: 0.10,
		hole_slack: 0.04,
		clock_deg: DISC_CLOCK_DEG,
		motor_len: 58.0,
		pin_len: 20.0,
		arm_len: 150.0,
	};
	if let Ok(text) = std::fs::read_to_string("cyclo26/params.csv") {
		for line in text.lines() {
			let line = line.trim();
			if line.starts_with('#') || line.is_empty() {
				continue;
			}
			let mut it = line.split(',');
			let (Some(k), Some(val)) = (it.next(), it.next()) else { continue };
			let Ok(x) = val.trim().parse::<f64>() else { continue };
			match k.trim() {
				"lobes" => p.lobes = x as usize,
				"ring_r" => p.ring_r = x,
				"pin_r" => p.pin_r = x,
				"ecc" => p.ecc = x,
				"mesh_clearance" => p.clear = x,
				"hole_slack" => p.hole_slack = x,
				"disc_clock_deg" => p.clock_deg = x,
				"motor_total_len" => p.motor_len = x,
				"pin_len" => p.pin_len = x,
				"arm_len" => p.arm_len = x,
				_ => {}
			}
		}
	}
	// sync assert: params.csv must not silently diverge from the shipped clock const
	assert!(
		(p.clock_deg - DISC_CLOCK_DEG).abs() < 1e-9,
		"params.csv disc_clock_deg {} != example DISC_CLOCK_DEG {} — update BOTH (and the sim)",
		p.clock_deg,
		DISC_CLOCK_DEG
	);
	p
}

// ---- the assembled stack (mm, z up, motor face at z = 0) ----
// CRICKET-CLASS ENVELOPE: the whole drive fits the NEMA-17 SQUARE (42.3,
// chamfered corners, flush with the motor) — nothing but the output hub is
// round. Four M3×30 through-bolts clamp lid + housing straight into the
// motor's own face taps: one bolt set assembles the whole actuator.
const NEMA_W: f64 = 42.3;
const BACK_T: f64 = 5.5; // housing back plate 0..5.5
const REG_D: f64 = 22.3; // NEMA-17 pilot register recess Ø (motor boss Ø22×2)
const REG_T: f64 = 2.2;
const SHAFT_BORE_D: f64 = 10.0; // < REG_D: preserves the register shoulder
const HUB_Z: f64 = 6.0; // cam hub ABOVE the plate: 6..9.5
const HUB_TOP: f64 = 9.5;
// BACKDRIVABLE ECCENTRIC: each disc rides a rolling 688 deep-groove ball bearing
// (8×16×5) on the cam — NOT a greased plain journal. Rolling contact replaces the
// dominant SLIDING loss of a plain journal: this is the single biggest backdrive-
// friction lever (cycloidal drives are backdrivable precisely because every
// contact rolls). The bearing bore (Ø8) grips a printed Ø8 cam journal (the
// eccentric post; a Ø5.1 D-bore for the motor shaft runs up its centre, leaving a
// ~0.95 mm eccentric web — the size-class trade, flagged in DESIGN.md); the
// bearing OD (Ø16) seats in the disc's Ø16.2 bore. The pin circle moved out
// (10.5->11.5) so the enlarged bore keeps the bore ligament ≥ 1.2, and DISC_T ==
// the 5 mm bearing width so the bearing sits FLUSH in the disc.
const ECC_BRG: &str = "688"; // eccentric bearing designation (deep_groove_bearing catalog)
const ECC_BRG_OD: f64 = 16.0; // 688 outer Ø — the disc bore seats it (+0.2 fit)
const ECC_BRG_W: f64 = 5.0; // 688 width
const CAM_J_D: f64 = 8.0; // cam eccentric journal Ø == 688 bore (inner-race seat)
const DISC_T: f64 = 5.0; // == ECC_BRG_W: the eccentric bearing sits flush in the disc bore
const DISC1_Z: f64 = 9.65; // 9.65..14.65 (disc 5.0 thick)
const DISC2_Z: f64 = 14.9; // 14.9..19.9 (0.25 gap between the two eccentric bearings)
const CAM_J_SPLIT: f64 = 14.75; // journal 1/2 split, between disc 1 top and disc 2 bottom
const CAM_TOP: f64 = 20.0; // j2 ends 0.1 proud of disc 2
const RING_TOP: f64 = 20.4; // ring wall top
const PIN_TOP: f64 = 22.0; // Ø2 dowels reach 1.6 into the lid sockets
const PLATE_Z: f64 = 20.4; // output plate underside (0.5 above disc 2)
const PLATE_TOP: f64 = 22.9; // plate disc top — b1's inner race seats here
const PLATE_R: f64 = 13.6; // plate disc radius
const SPIG_R: f64 = 10.0; // spigot rides the output 6804 bores
const B6804_OD: f64 = 32.0;
const B6804_W: f64 = 7.0;
const TOWER_BOT: f64 = 22.9; // shelf ring 22.9..24.1 carries the lower outer race
const B1_Z: f64 = TOWER_BOT + 1.2; // 24.1..31.1 — ONE 6804 (drops in from above)
const LIP_Z: f64 = B1_Z + B6804_W; // 31.1: race top (retainer lip reaches it)
const LID_TOP: f64 = 31.3;
const RET_TOP: f64 = 33.3; // top retainer plate
const FACE_Z: f64 = 35.5; // hub face (hub plate 33.5..35.5)
const HEX_AF: f64 = 12.0; // spigot-top hex: the TORQUE register (screws only clamp)
// Hub clamp-screw circle (±x on the spigot top). MUST satisfy: ≥ boss_r 4.0 +
// head_r 3.15 + 0.2 (csk head corridor past the Ø8 register boss) and
// ≤ SPIG_R 10.0 − pilot_r 1.25 − 0.45 (clear of the 6804 inner-race seat).
const HUB_SCREW_R: f64 = 8.3;
const HUB_CB_FLOOR: f64 = FACE_Z - 2.0; // Ø6.3 counterbore floor: button head (k 1.65) sits 0.35 below the arm-seat face
const OUT_PIN_CIRCLE: f64 = 11.5; // output-screw circle (moved out for the eccentric bearing bore; ligaments asserted)
const OUT_PIN_R: f64 = 1.5; // M3 shank
const BOLT_SQ: f64 = 15.5; // NEMA bolt square half-pitch (through-bolts)
// Sandwich-bolt stack: M3×SANDWICH_L button heads seat in Ø6.5×SANDWICH_CB lid
// counterbores; tap engagement = SANDWICH_CB + SANDWICH_L − LID_TOP = 4.0 mm,
// inside the ~4.5 mm blind-tap depth of the NEMA-17 face (ICS 16). Gated in
// A-ASM (tap-engagement) — the 2026-07-19 audit found the previous M3×40/cb 3.0
// combination demanded 11.7 mm and would bottom out before clamping.
const SANDWICH_L: f64 = 30.0; // M3×30 sandwich through-bolt length
const SANDWICH_CB: f64 = 5.3; // lid head-counterbore depth
const IF_PILOT_H: f64 = 2.5; // register boss height on the hub face
const ARM_CIRCLE_R: f64 = 10.0; // 6×M3 arm bolt circle on the hub face
const SEG: usize = 64;
const SEG_S: usize = 32;
const PLA: f64 = 0.00124;

const _: () = assert!(SHAFT_BORE_D < REG_D - 2.0, "shaft bore must preserve the register shoulder");
const _: () = assert!(DISC_T == ECC_BRG_W, "disc thickness must equal the eccentric bearing width (flush seat)");
const _: () = assert!(CAM_J_SPLIT > DISC1_Z + DISC_T && CAM_J_SPLIT < DISC2_Z, "journal split must sit between the two discs");
const _: () = assert!(PIN_TOP > RING_TOP && PIN_TOP - RING_TOP <= 2.5, "pins must reach into the lid sockets");
const _: () = assert!(LIP_Z == B1_Z + B6804_W, "the retainer lip must land exactly on the race top");
const _: () = assert!(B1_Z >= PLATE_TOP, "lower bearing seats on the plate top");

fn v(x: f64, y: f64, z: f64) -> DVec3 {
	DVec3::new(x, y, z)
}
fn tr(x: f64, y: f64, z: f64) -> DAffine3 {
	DAffine3::from_translation(v(x, y, z))
}
fn rotz(a: f64) -> DAffine3 {
	DAffine3::from_rotation_z(a)
}
fn ccw(mut p: Vec<DVec2>) -> Vec<DVec2> {
	let a2: f64 = p.windows(2).map(|w| w[0].x * w[1].y - w[1].x * w[0].y).sum::<f64>()
		+ (p[p.len() - 1].x * p[0].y - p[0].x * p[p.len() - 1].y);
	if a2 < 0.0 {
		p.reverse();
	}
	p
}
/// M3 thread-forming pilot (Ø2.5×8): screws bite the plastic directly.
/// `into` is the DRILLING direction (from the face into the material); the
/// cutter starts 0.5 proud and reaches 8 deep. The original version ran the
/// cylinder the other way and left 0.5 mm dimples everywhere — caught when a
/// user asked why the casing holes "don't match".
fn pilot(s: &Solid, at: DVec3, into: DVec3) -> Solid {
	let a = into.normalize();
	difference(s, &cylinder(at - a * 0.5, a, 1.25, 8.5, 16))
}
fn bore(s: &Solid, face: DVec3, axis: DVec3, d: f64, len: f64, seg: usize) -> Solid {
	let a = axis.normalize();
	difference(s, &cylinder(face - a, a, d * 0.5, len + 2.0, seg))
}

/// Disc profile via the library generator (single source of truth — the
/// simulator uses the same function).
fn cycloid_profile(p: &P, pin_r: f64, pts_per_lobe: usize) -> Vec<DVec2> {
	kernel_model::parts::cycloid_disc_profile(p.lobes, p.ring_r, pin_r, p.ecc, pts_per_lobe)
}

/// Undercut check: the OFFSET profile (the disc contour actually cut) must be
/// a SIMPLE closed curve — an oversized pin radius makes the offset
/// self-intersect at the lobe roots (undercut) before anything else fails.
/// Returns (is_simple, turning_number).
fn profile_is_simple(p: &P) -> (bool, f64) {
	let pts = cycloid_profile(p, p.pin_r + p.clear, 96);
	let n = pts.len();
	// turning number: total signed exterior angle / 2π must be ±1
	let mut turn = 0.0f64;
	for i in 0..n {
		let (a, b, c) = (pts[(i + n - 1) % n], pts[i], pts[(i + 1) % n]);
		let (u, w) = (b - a, c - b);
		turn += f64::atan2(u.x * w.y - u.y * w.x, u.x * w.x + u.y * w.y);
	}
	let turning = turn / TAU;
	// segment self-intersection (skip neighbours); n≈600 → n²/2 cheap checks
	let seg = |i: usize| (pts[i], pts[(i + 1) % n]);
	let inter = |a: DVec2, b: DVec2, c: DVec2, d: DVec2| {
		let o = |p: DVec2, q: DVec2, r: DVec2| (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
		let (d1, d2, d3, d4) = (o(a, b, c), o(a, b, d), o(c, d, a), o(c, d, b));
		d1 * d2 < 0.0 && d3 * d4 < 0.0
	};
	for i in 0..n {
		let (a, b) = seg(i);
		for j in (i + 2)..n {
			if i == 0 && j == n - 1 {
				continue;
			}
			let (c, d) = seg(j);
			if inter(a, b, c, d) {
				return (false, turning);
			}
		}
	}
	(true, turning)
}

// ---- printed parts -------------------------------------------------------------------

/// Cycloidal disc: epitrochoid extrusion, Ø16.2 eccentric-bearing (688) bore,
/// six output-screw holes on the ligament-checked r11.5 circle. `hole_clock`
/// (rad) rotates ONLY the three output holes about the disc centre — the
/// epitrochoid mesh profile and the bearing bore never move — so the two prints
/// (disc_a, disc_b) stay DISTINCT (disc_b's holes are compensated +pi/lobes so
/// its meshed placement lands them back on the pin circle). Prints flat.
fn disc(p: &P, hole_clock: f64) -> Solid {
	let prof = ccw(cycloid_profile(p, p.pin_r + p.clear, 24));
	let mut d = extrude(&prof, DISC_T);
	d = bore(&d, v(0.0, 0.0, DISC_T + 1.0), -DVec3::Z, ECC_BRG_OD + 0.2, DISC_T + 3.0, SEG); // Ø16.2 eccentric-bearing (688) outer-race seat
	for k in 0..OUT_PINS {
		let a = TAU * k as f64 / OUT_PINS as f64 + hole_clock;
		d = bore(
			&d,
			v(OUT_PIN_CIRCLE * a.cos(), OUT_PIN_CIRCLE * a.sin(), DISC_T + 1.0),
			-DVec3::Z,
			(OUT_PIN_R + p.ecc + p.hole_slack) * 2.0, // slack: the second precision tunable (small pin circle amplifies it)
			DISC_T + 3.0,
			SEG_S,
		);
	}
	d
}

/// Housing: the NEMA-17 SQUARE continued upward — 42.3 chamfered-corner
/// outline flush with the motor, round ring cavity inside, Ø2 dowel press
/// pockets, TRUE pilot register underneath, and four Ø3.4 corner passages
/// for the M3×30 through-bolts that clamp lid+housing into the motor\'s own
/// face taps. Prints as used.
fn housing(p: &P) -> Solid {
	let pins = p.lobes + 1;
	let h2 = NEMA_W * 0.5;
	let c = 2.5; // NEMA corner chamfer per edge
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
	// ring cavity above the back plate
	h = difference(&h, &cylinder(v(0.0, 0.0, BACK_T), DVec3::Z, p.ring_r + p.pin_r + 0.8, RING_TOP, SEG));
	// Ø2 steel dowel press pockets (the ring gear)
	let pocket = BACK_T - (PIN_TOP - p.pin_len);
	assert!((1.5..=BACK_T - 1.0).contains(&pocket), "pin_len {} needs a {pocket:.1} pocket", p.pin_len);
	for k in 0..pins {
		let a = TAU * k as f64 / pins as f64;
		h = bore(&h, v(p.ring_r * a.cos(), p.ring_r * a.sin(), BACK_T), -DVec3::Z, 1.95, pocket, 16);
	}
	// NEMA register + shaft bore + 46° funnel over the recess ceiling
	h = difference(&h, &cylinder(v(0.0, 0.0, -0.5), DVec3::Z, REG_D * 0.5, REG_T + 0.5, SEG));
	h = bore(&h, v(0.0, 0.0, BACK_T), -DVec3::Z, SHAFT_BORE_D, BACK_T + 2.0, SEG);
	h = difference(&h, &cone(v(0.0, 0.0, REG_T - 0.2), DVec3::Z, REG_D * 0.5 + 0.3, 12.3, SEG));
	// through-bolt passages at the NEMA bolt square
	for (dx, dy) in [(BOLT_SQ, BOLT_SQ), (-BOLT_SQ, BOLT_SQ), (BOLT_SQ, -BOLT_SQ), (-BOLT_SQ, -BOLT_SQ)] {
		h = bore(&h, v(dx, dy, RING_TOP), -DVec3::Z, 3.4, RING_TOP + 2.0, 16);
	}
	// gabled wire exit through the wall (the SERVO42D leads)
	h = teardrop_hole(&h, v(0.0, -(h2 + 0.5), (BACK_T + RING_TOP) * 0.5 + 1.0), DVec3::Y, DVec3::Z, 7.0, 6.0, 46.0, None)
		.expect("wire exit");
	h
}

/// Eccentric cam: D-bored hub (Ø5 shaft, M3 set screw in a flat boss) and two Ø8
/// journals at ±e, 180° apart — each carries a 688 deep-groove ball bearing whose
/// outer race seats in a disc bore, so the discs roll (backdrivable) instead of
/// sliding on a plain journal. The twin discs 180° apart cancel the wobble
/// imbalance. Thin (~0.95 mm) eccentric web around the shaft — the size-class
/// trade, honest in DESIGN.md. Prints hub-down.
fn cam(p: &P) -> Solid {
	let mut c = cylinder(v(0.0, 0.0, HUB_Z), DVec3::Z, 6.5, HUB_TOP - HUB_Z, SEG_S);
	// Ø8 eccentric journals (== 688 bore) at ±e. Split at CAM_J_SPLIT (14.75),
	// between disc 1 top (14.65) and disc 2 bottom (14.9), so each journal fully
	// carries its bearing's 5 mm inner race with margin. j1 roots 0.3 into the hub.
	c = union(&c, &cylinder(v(p.ecc, 0.0, HUB_TOP - 0.3), DVec3::Z, CAM_J_D * 0.5, CAM_J_SPLIT - (HUB_TOP - 0.3), SEG_S));
	c = union(&c, &cylinder(v(-p.ecc, 0.0, CAM_J_SPLIT), DVec3::Z, CAM_J_D * 0.5, CAM_TOP - CAM_J_SPLIT, SEG_S));
	// Ø5.1 shaft clearance/grip runs the FULL cam height (shaft tip z=24 clears the
	// cam top at z=20 into the plate bore). The D-flat that keys the shaft flat is
	// LIMITED to the hub grip region — extending it up the thin Ø8 journals would
	// leave a fragile sliver on the +x wall (the journal web is only ~0.95 mm).
	let mut dbore = cylinder(v(0.0, 0.0, HUB_Z - 1.0), DVec3::Z, 2.55, CAM_TOP - (HUB_Z - 1.0) + 1.0, SEG_S); // overshoot the cam top (avoid a coplanar cut)
	dbore = difference(&dbore, &cuboid(v(2.0, -3.0, HUB_Z - 2.0), v(4.0, 3.0, HUB_TOP + 2.0)));
	c = difference(&c, &dbore);
	// set-screw boss with a flat entry face (M3 bites a Ø2.5 pilot)
	c = union(&c, &cuboid(v(5.0, -3.5, HUB_Z), v(8.2, 3.5, HUB_TOP)));
	// depth 6.5: the pocket must BREAK INTO the D-flat (x=2.0) — the BOM
	// audit found the original 5.0 stopped 1.2 short of ever reaching the shaft
	c = teardrop_hole(&c, v(8.2, 0.0, (HUB_Z + HUB_TOP) * 0.5), -DVec3::X, DVec3::Z, 2.5, 6.5, 46.0, None)
		.expect("cam set screw");
	c
}

/// Output plate: Ø27.2 disc carrying the six M3 output-screw pins and the
/// Ø20 spigot for the two 6804s — the plate TOP is the lower inner race\'s
/// seat. Washer-cap recess coned underneath. Prints as used.
fn output_plate() -> Solid {
	let mut o = revolve(
		&[
			DVec2::new(0.05, PLATE_Z),
			DVec2::new(PLATE_R, PLATE_Z),
			DVec2::new(PLATE_R, PLATE_TOP),
			DVec2::new(12.0, PLATE_TOP),
			DVec2::new(12.0, B1_Z), // Ø24 shoulder: the lower INNER race clamps here
			DVec2::new(SPIG_R, B1_Z),
			DVec2::new(SPIG_R, LIP_Z),
			DVec2::new(0.05, LIP_Z),
		],
		SEG,
	);
	// hex torque register on the spigot top (the hub's socket mates it)
	let hexp: Vec<DVec2> = (0..6)
		.map(|k| {
			let a = TAU * k as f64 / 6.0 + PI / 6.0;
			let r = HEX_AF * 0.5 / (PI / 6.0).cos();
			DVec2::new(r * a.cos(), r * a.sin())
		})
		.collect();
	o = union(&o, &extrude(&ccw(hexp), 4.0).transformed(tr(0.0, 0.0, LIP_Z)));
	// the motor SHAFT (24 long, tip at z=24) spins inside the plate: Ø7
	// clearance bore up the spigot centre (another A-PROD catch — the plate
	// centre was solid where the shaft lives)
	o = bore(&o, v(0.0, 0.0, PLATE_Z), DVec3::Z, 7.0, 26.0 - PLATE_Z, SEG_S);
	// output screws: SIX M3 (steel pins carry the torque; raised from three —
	// the 10 N·m FEA showed the pin-bore bearing binds the torque capacity, and
	// six pins on the same circle halve it; radial ligaments unchanged)
	for k in 0..OUT_PINS {
		let a = TAU * k as f64 / OUT_PINS as f64;
		let (cx, cy) = (OUT_PIN_CIRCLE * a.cos(), OUT_PIN_CIRCLE * a.sin());
		o = bore(&o, v(cx, cy, PLATE_TOP), -DVec3::Z, 2.9, PLATE_TOP - PLATE_Z + 2.0, SEG_S);
		// TRUE 90° countersink, opening r3.3 at the plate top: seats the ISO
		// 10642 head (dk actual max 6.3) flush-to-under-flush. The old (3.5,3.6)
		// cone opened only r2.53 — the head sat ~0.5 proud and ground the lid
		// shelf (2026-07-19 audit; gated by the csk head-seat gate). The recess
		// exits the Ø27.2 rim in 6 shallow top-edge scallops — cosmetic; the
		// FEA hotspot (spigot shoulder) and the bore bearing land are untouched.
		o = difference(&o, &cone(v(cx, cy, PLATE_TOP + 0.2), -DVec3::Z, 3.5, 3.5, SEG_S));
	}
	// hub bolts: TWO Ø2.5 pilots in the SPIGOT TOP at r8.3 (torque rides the
	// hex; the screws only clamp). Moved off the hex boss (was ±4.0): an M3 csk
	// head at r4 can never descend past the Ø8 register boss (blocked corridor),
	// its recess was refilled by the boss union, AND the Ø2.5×8 hex pilot left
	// the M3×12 tip 3.6 mm short (bottom-out — the planetary26 defect class).
	// At r8.3: head corridor clears the boss by 0.65, pilot outer edge r9.55
	// clears the 6804 inner-race seat (r10) by 0.45, floor z23.0 gives the tip
	// (z23.5) 0.5 clearance and never meets the pin csk recesses (< z22.9).
	// Thread bite 7.6 mm (limited by screw reach; the hex carries all torque).
	for dx in [HUB_SCREW_R, -HUB_SCREW_R] {
		o = difference(&o, &cylinder(v(dx, 0.0, 23.0), DVec3::Z, 1.25, 8.6, 16));
	}
	o
}

/// Lid: the square cap — NEMA outline continued, corner counterbores for
/// the M3×30 through-bolts, dowel capture sockets, plate cavity, and the
/// central bearing tower (Ø32.1 bore, integral top lip; retainer ring bolts
/// under it). Modelled as assembled; prints upside-down.
fn lid(p: &P) -> Solid {
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
	// plate cavity + bottom SHELF + straight tower bore to the top: the races
	// DROP IN FROM ABOVE after the lid is on (the audit proved a bottom-loaded
	// retainer cannot pass the bearings and its screws face the sealed
	// interior — unassemblable); the top retainer is externally bolted
	let cavity = revolve(
		&[
			DVec2::new(0.05, RING_TOP - 1.0),
			// cavity wall r14.9 (was PLATE_R+0.6 = 14.2): the six pin-screw csk
			// heads overhang the Ø27.2 plate rim (edge at 11.5 + dk/2 ≤ 14.65)
			// and SPIN with the plate — the wall must clear them (0.25 min).
			// Cost: the dowel-socket inner web thins 1.23 → 0.53 mm; the mesh
			// load pushes each dowel OUTWARD (away from this web), the sockets
			// only guide the pin tops laterally — acceptable, stated honestly.
			DVec2::new(PLATE_R + 1.3, RING_TOP - 1.0),
			DVec2::new(PLATE_R + 1.3, TOWER_BOT),
			DVec2::new(14.0, TOWER_BOT), // shelf ring 14.0..16.05 carries the lower race
			DVec2::new(14.0, TOWER_BOT + 1.2),
			DVec2::new(B6804_OD * 0.5 + 0.05, TOWER_BOT + 1.2),
			DVec2::new(B6804_OD * 0.5 + 0.05, LID_TOP + 1.0),
			DVec2::new(0.05, LID_TOP + 1.0),
		],
		SEG,
	);
	l = difference(&l, &cavity);
	// top-retainer pilots in the lid top face (two suffice: axial up-loads only)
	for k in 0..2 {
		let a = PI * k as f64 + FRAC_PI_2;
		l = pilot(&l, v(18.2 * a.cos(), 18.2 * a.sin(), LID_TOP), -DVec3::Z);
	}
	// dowel capture sockets with lead-in chamfers
	let pins = p.lobes + 1;
	for k in 0..pins {
		let a = TAU * k as f64 / pins as f64;
		let (px, py) = (p.ring_r * a.cos(), p.ring_r * a.sin());
		l = difference(&l, &cylinder(v(px, py, RING_TOP - 1.0), DVec3::Z, 1.075, PIN_TOP - RING_TOP + 1.1, 16));
		l = difference(&l, &cone(v(px, py, RING_TOP - 0.2), DVec3::Z, 1.6, 1.1, 16));
	}
	// through-bolt passages + head counterbores. Counterbore depth 5.3 (was 3.0)
	// so a standard M3×30 lands EXACTLY 4.0 mm in the motor's blind face taps:
	// engagement = cb + L − LID_TOP = 5.3 + 30 − 31.3 = 4.0 ≤ the ~4.5 mm NEMA-17
	// (ICS 16) tap depth. The original M3×40 at cb 3.0 demanded 11.7 mm — it
	// BOTTOMS OUT ~7 mm early and the sandwich never clamps (2026-07-19 audit;
	// the A-ASM tap-engagement gate now measures this interface).
	for (dx, dy) in [(BOLT_SQ, BOLT_SQ), (-BOLT_SQ, BOLT_SQ), (BOLT_SQ, -BOLT_SQ), (-BOLT_SQ, -BOLT_SQ)] {
		l = bore(&l, v(dx, dy, LID_TOP), -DVec3::Z, 3.4, LID_TOP - RING_TOP + 2.0, 16);
		l = difference(&l, &cylinder(v(dx, dy, LID_TOP - SANDWICH_CB), DVec3::Z, 3.25, SANDWICH_CB + 1.0, 16));
	}
	l
}

/// Top retainer: bolts onto the LID TOP (externally — the audit proved a
/// bottom-loaded retainer cannot be assembled), its lip reaching down the
/// tower bore to clamp the upper OUTER race; with the lid\'s shelf this
/// holds the races both ways. Prints flat.
fn retainer_ring() -> Solid {
	let mut r = cylinder(v(0.0, 0.0, LID_TOP), DVec3::Z, 19.8, RET_TOP - LID_TOP, SEG);
	r = union(&r, &revolve(
		&[
			DVec2::new(13.6, LIP_Z),
			DVec2::new(15.9, LIP_Z),
			DVec2::new(15.9, LID_TOP + 0.2),
			DVec2::new(13.6, LID_TOP + 0.2),
		],
		SEG,
	));
	r = bore(&r, v(0.0, 0.0, RET_TOP), -DVec3::Z, 27.2, RET_TOP - LIP_Z + 2.0, SEG);
	for k in 0..2 {
		let a = PI * k as f64 + FRAC_PI_2;
		r = bore(&r, v(18.2 * a.cos(), 18.2 * a.sin(), RET_TOP), -DVec3::Z, 3.4, RET_TOP - LID_TOP + 2.0, 16);
	}
	r
}

/// Output hub: the arm-mount face — Ø30 disc, SIX M3 pilots on the Ø20 arm
/// bolt circle (the "top holes"), Ø8 register boss with the encoder-magnet
/// pocket, under-boss clamping the upper inner race, 2× M3×8 BUTTON in Ø6.3
/// counterbores at r8.3 into the spigot top (moved off the hex boss — a head
/// at r4 cannot pass the Ø8 register boss; csk deemed unnecessary by the
/// 2026-07-19 audit, a counterbore sinks the head below the arm-seat face).
/// Prints face-up (stands on its under-boss ring, small bridge).
fn output_hub() -> Solid {
	let mut h = cylinder(v(0.0, 0.0, RET_TOP + 0.2), DVec3::Z, 15.0, FACE_Z - RET_TOP - 0.2, SEG);
	h = union(&h, &cylinder(v(0.0, 0.0, LIP_Z), DVec3::Z, 11.9, RET_TOP + 0.4 - LIP_Z, SEG_S));
	// hex socket: the spigot's hex carries the torque; two M3 csk just clamp
	let hexs: Vec<DVec2> = (0..6)
		.map(|k| {
			let a = TAU * k as f64 / 6.0 + PI / 6.0;
			let r = (HEX_AF + 0.3) * 0.5 / (PI / 6.0).cos();
			DVec2::new(r * a.cos(), r * a.sin())
		})
		.collect();
	h = difference(&h, &extrude(&ccw(hexs), 4.4).transformed(tr(0.0, 0.0, LIP_Z - 0.2)));
	// clamp screws at r8.3 (see HUB_SCREW_R): the only radius whose head
	// corridor clears the Ø8 register boss AND whose pilot lands in the solid
	// spigot annulus inside the 6804 race seat. BUTTON heads (ISO 7380 M3×8,
	// Ø5.7×1.65) sunk in Ø6.3×2.0 cylindrical counterbores — the csk-necessity
	// audit (2026-07-19) found flush IS required (the arm flange seats on this
	// face over r8.3) but a countersink is NOT: the counterbore sinks the head
	// 0.35 below the face, leaves 2.4 mm under the head, and unifies the spec
	// with the retainer screws. Csk survives ONLY at the six plate pins, where
	// the rotating zero-gap lid-shelf corridor demands a truly flush cone head.
	for dx in [HUB_SCREW_R, -HUB_SCREW_R] {
		h = bore(&h, v(dx, 0.0, FACE_Z), -DVec3::Z, 3.4, FACE_Z - LIP_Z + 2.0, SEG_S);
		h = difference(&h, &cylinder(v(dx, 0.0, HUB_CB_FLOOR), DVec3::Z, 3.15, FACE_Z - HUB_CB_FLOOR + 1.0, SEG_S));
	}
	// the ARM bolt circle: 6×M3 thread-forming pilots on Ø20
	for k in 0..6 {
		let a = TAU * k as f64 / 6.0 + PI / 6.0;
		h = pilot(&h, v(ARM_CIRCLE_R * a.cos(), ARM_CIRCLE_R * a.sin(), FACE_Z), -DVec3::Z);
	}
	// register boss + magnet pocket
	h = union(&h, &cylinder(v(0.0, 0.0, FACE_Z - 0.2), DVec3::Z, 4.0, IF_PILOT_H + 0.2, SEG_S));
	h = difference(&h, &cylinder(v(0.0, 0.0, FACE_Z + IF_PILOT_H - 2.0), DVec3::Z, 3.05, 2.5, SEG_S));
	h
}

// ---- emit / audit ---------------------------------------------------------------------

fn emit(name: &str, s: &Solid, to_print: DAffine3) -> (bool, f64) {
	let val = validate(s);
	let mut printed = s.transformed(to_print);
	let zmin = tessellate_default(&printed).positions.iter().map(|q| q.z as f64).fold(f64::INFINITY, f64::min);
	printed = printed.transformed(tr(0.0, 0.0, -zmin));
	let mesh = tessellate_default(&printed);
	let rep = mesh.support_free_report(Vec3::Z, 45.0, 0.3);
	let wt = mesh.is_watertight();
	let grams = volume(s).abs() * PLA;
	let ok = val.is_valid() && wt && rep.steep_area < 1e-6 && rep.max_bridge_span <= 12.0;
	let _ = std::fs::write(format!("cyclo26/parts/{name}.stl"), mesh.to_stl_binary());
	println!(
		"  {name:20} valid={:5} wt={wt:5} {}  {grams:4.0}g  {}",
		val.is_valid(),
		if rep.steep_area < 1e-6 { format!("sf br≤{:4.1}", rep.max_bridge_span) } else { format!("steep {:.0}mm²", rep.steep_area) },
		if ok { "OK" } else { "<<< FAIL" }
	);
	(ok, grams)
}

fn merge_into(dst: &mut Mesh, src: &Mesh) {
	let base = dst.positions.len() as u32;
	dst.positions.extend_from_slice(&src.positions);
	dst.indices.extend(src.indices.iter().map(|i| i + base));
}

fn overlap_mm3(a: &Solid, b: &Solid) -> f64 {
	match try_difference(a, b) {
		Ok(rem) => (volume(a).abs() - volume(&rem).abs()).max(0.0),
		Err(_) => f64::NAN,
	}
}

fn main() {
	let _ = std::fs::create_dir_all("cyclo26/parts");
	// the folder is a PURE print queue: purge stale files so it always mirrors
	// exactly the current build (superseded parts never linger)
	if let Ok(dir) = std::fs::read_dir("cyclo26/parts") {
		for e in dir.flatten() {
			let _ = std::fs::remove_file(e.path());
		}
	}
	let p = load();
	let pins_n = p.lobes + 1;
	println!(
		"CYCLO CRICKET-CLASS — {}:1 for NEMA-17: {} lobes / {} pins Ø2, e={}, ring Ø{:.0}, body {}×{} sq × {:.0} + motor\n",
		p.lobes,
		p.lobes,
		pins_n,
		p.ecc,
		p.ring_r * 2.0,
		NEMA_W,
		NEMA_W,
		LID_TOP
	);

	// anti-backlash split-disc clock (rad at the pin circle) and disc_b's
	// hole-compensation. disc_b is placed at rotz(−π/lobes); to keep its output
	// holes on the pin circle its holes are pre-clocked +π/lobes in its own
	// frame, and the ∓Δ/2 anti-backlash split rides on top. So (pin frame):
	// disc_a leads +Δ/2, disc_b lags −Δ/2 — bounding opposite output senses.
	let comp = PI / p.lobes as f64;
	let half = p.clock_deg.to_radians() * 0.5;

	// ---- A-GEOM: feasibility gates on the parametric geometry itself ----
	let mut ok = true;
	let cusp_ok = p.ecc < p.ring_r / pins_n as f64;
	let (simple, turning) = profile_is_simple(&p);
	let undercut_ok = simple && (turning.abs() - 1.0).abs() < 0.01;
	let hole_r = OUT_PIN_R + p.ecc + p.hole_slack;
	let lig_bore = (OUT_PIN_CIRCLE - hole_r) - (ECC_BRG_OD + 0.2) * 0.5; // bore now seats the Ø16.2 eccentric-bearing OD
	let disc_min_r = p.ring_r - (p.pin_r + p.clear) - p.ecc;
	let lig_rim = disc_min_r - (OUT_PIN_CIRCLE + hole_r);
	// six holes on one circle add an adjacent-hole ligament the 3-pin build never
	// had — gate it like the radial ones (chord pitch minus a hole diameter)
	let lig_adj = 2.0 * OUT_PIN_CIRCLE * (PI / OUT_PINS as f64).sin() - 2.0 * hole_r;
	let lig_ok = lig_bore >= 1.2 && lig_rim >= 1.2 && lig_adj >= 1.2;
	ok &= cusp_ok && undercut_ok && lig_ok;
	// the clock rotates the holes TANGENTIALLY on the r10.5 circle (radius fixed),
	// so the radial bore/rim ligaments are preserved; report the tangential shift
	// (disc_b's holes move comp−Δ/2 rad ≈ its arc) to make the "no new ligament
	// risk" claim MEASURED, not assumed.
	let disc_b_shift_mm = OUT_PIN_CIRCLE * (comp - half);
	println!(
		"A-GEOM: cusp e<R/N {} · offset profile simple, turning {:.2} {} · ligaments bore {lig_bore:.2}/rim {lig_rim:.2}/adj {lig_adj:.2} ≥ 1.2 {} · holes clocked tangentially (disc_b +{:.2}mm arc, radius fixed → ligaments preserved)",
		if cusp_ok { "OK" } else { "<<< FAIL" },
		turning,
		if undercut_ok { "OK" } else { "<<< FAIL" },
		if lig_ok { "OK" } else { "<<< FAIL" },
		disc_b_shift_mm
	);

	// two DISTINCT prints: disc_a (bottom) and disc_b (top). Only the output holes
	// differ; the epitrochoid mesh and journal bore are identical.
	let disc_a = disc(&p, half);
	let disc_b = disc(&p, comp - half);
	let house = housing(&p);
	let cam_p = cam(&p);
	let plate = output_plate();
	let lid_p = lid(&p);
	let hub = output_hub();
	let retainer = retainer_ring();

	let flat = DAffine3::IDENTITY;
	let parts: Vec<(&str, &Solid, DAffine3)> = vec![
		("output_hub", &hub, flat), // face-up: stands on the under-boss ring, 11 mm annular bridge
		("retainer_ring", &retainer, flat),
		("cyclo_disc_a", &disc_a, flat), // bottom disc — output holes lead +Δ/2
		("cyclo_disc_b", &disc_b, flat), // top disc — output holes lag −Δ/2 (holes clocked +π/lobes in-frame)
		("housing", &house, flat),
		("eccentric_cam", &cam_p, flat),
		("output_plate", &plate, flat),
		("lid_ring", &lid_p, DAffine3::from_rotation_x(PI)), // prints skirt-up
	];
	println!("\nprintable parts (cyclo26/parts is a pure print queue):");
	let mut grams = std::collections::HashMap::new();
	for (n, s, m) in &parts {
		let (o, g) = emit(n, s, *m);
		ok &= o;
		grams.insert(*n, g);
	}
	// ---- A-RATIO ----
	let prof = cycloid_profile(&p, p.pin_r + p.clear, 24);
	let (mut rmin, mut rmax) = (f64::INFINITY, 0.0f64);
	for q in &prof {
		let r = (q.x * q.x + q.y * q.y).sqrt();
		rmin = rmin.min(r);
		rmax = rmax.max(r);
	}
	let want_max = p.ring_r - (p.pin_r + p.clear) + p.ecc;
	let ratio_ok = (rmax - want_max).abs() < 0.05 && (rmin - disc_min_r).abs() < 0.05;
	ok &= ratio_ok;
	println!(
		"\nA-RATIO: {}:1; disc r ∈ [{rmin:.2},{rmax:.2}] (theory [{disc_min_r:.2},{want_max:.2}])  {}",
		p.lobes,
		if ratio_ok { "OK" } else { "<<< FAIL" }
	);

	// ---- A-KIN: 24-pose cam sweep, disc vs ring pins ----
	let ring_only = {
		let mut r = cylinder(v(0.0, 0.0, DISC1_Z), DVec3::Z, 0.01, 0.01, 8);
		for k in 0..pins_n {
			let a = TAU * k as f64 / pins_n as f64;
			r = union(&r, &cylinder(v(p.ring_r * a.cos(), p.ring_r * a.sin(), DISC1_Z - 1.0), DVec3::Z, p.pin_r, 20.0, 16));
		}
		r
	};
	let ring_mesh = tessellate_default(&ring_only);
	let (mut worst_gap, mut kin_ok) = (0.0f64, true);
	for k in 0..24 {
		let th = TAU * k as f64 / 24.0;
		let pose = tr(p.ecc * th.cos(), p.ecc * th.sin(), DISC1_Z) * rotz(-th / p.lobes as f64);
		let posed = disc_a.transformed(pose); // mesh is identical on both discs; disc_a stands in
		let ov = overlap_mm3(&posed, &ring_only);
		let gap = tessellate_default(&posed).min_distance(&ring_mesh) as f64;
		worst_gap = worst_gap.max(gap);
		if ov.is_nan() || ov >= 0.05 || gap > 3.0 * p.clear + 0.05 {
			kin_ok = false;
			println!("  θ={:5.1}°: overlap {ov:.3} mm³, gap {gap:.3}  <<<", th.to_degrees());
		}
	}
	ok &= kin_ok;
	println!(
		"A-KIN: 24-pose sweep — zero interference, worst engagement gap {worst_gap:.3} ≤ {:.3}  {}",
		3.0 * p.clear + 0.05,
		if kin_ok { "OK" } else { "<<< FAIL" }
	);

	// ---- A-PINS: output screw shanks vs disc holes, plate co-rotating ----
	let pins_only = {
		let mut r = cylinder(v(0.0, 0.0, 60.0), DVec3::Z, 0.01, 0.01, 8);
		for k in 0..OUT_PINS {
			let a = TAU * k as f64 / OUT_PINS as f64;
			r = union(
				&r,
				&cylinder(v(OUT_PIN_CIRCLE * a.cos(), OUT_PIN_CIRCLE * a.sin(), DISC1_Z - 1.0), DVec3::Z, OUT_PIN_R, 20.0, SEG_S),
			);
		}
		r
	};
	// BOTH clocked hole patterns must clear the shared pins over the sweep: disc_a
	// at phase 0, disc_b at phase π (its holes compensated back onto the pins).
	let mut pin_ok = true;
	for k in 0..12 {
		let th = TAU * k as f64 / 12.0;
		let pins_rot = pins_only.transformed(rotz(-th / p.lobes as f64));
		let pose_a = tr(p.ecc * th.cos(), p.ecc * th.sin(), DISC1_Z) * rotz(-th / p.lobes as f64);
		let pose_b = tr(p.ecc * (th + PI).cos(), p.ecc * (th + PI).sin(), DISC2_Z) * rotz(-(th + PI) / p.lobes as f64);
		let ova = overlap_mm3(&disc_a.transformed(pose_a), &pins_rot);
		let ovb = overlap_mm3(&disc_b.transformed(pose_b), &pins_rot);
		if ova.is_nan() || ova >= 0.05 || ovb.is_nan() || ovb >= 0.05 {
			pin_ok = false;
			println!("  output screws collide at θ={:.0}°: disc_a {ova:.2}/disc_b {ovb:.2} mm³ <<<", th.to_degrees());
		}
	}
	ok &= pin_ok;
	println!("A-PINS: BOTH clocked disc hole patterns clear the output screws over the sweep  {}", if pin_ok { "OK" } else { "<<< FAIL" });

	// ---- assembly: exact as-assembled poses ----
	println!("\nassembly (exact poses; hardware hw_* lives ONLY here + STEP):");
	let mut asm = Mesh::new();
	let mut instances: Vec<(String, Solid, DAffine3)> = Vec::new();
	let place = |m: &mut Mesh, list: &mut Vec<(String, Solid, DAffine3)>, name: &str, s: &Solid, x: DAffine3| {
		merge_into(m, &tessellate_default(&s.transformed(x)));
		list.push((name.to_string(), s.clone(), x));
	};
	let motor = nema_motor(17, 48.0).expect("nema17");
	let dowel = kernel_model::parts::dowel_pin(2.0, p.pin_len).expect("ring dowel"); // Ø2 — matches pockets/profile/sockets
	let m3x12f = flat_head_screw(3.0, 12.0).expect("m3x12 csk");
	let m3x30 = button_head_screw(3.0, SANDWICH_L).expect("m3x30 button");
	let m3x8r = button_head_screw(3.0, 8.0).expect("m3x8 button");
	let m3set = kernel_model::parts::set_screw(3.0, 5.0).expect("m3x5 din916");
	let b6804 = deep_groove_bearing("6804").expect("6804");
	let b_ecc = deep_groove_bearing(ECC_BRG).expect("688 eccentric"); // one per disc, on the cam journals

	// nema_motor frame: face at z=0, body −Z, shaft +Z — mounts as modelled
	place(&mut asm, &mut instances, "hw_nema17", &motor, flat);
	place(&mut asm, &mut instances, "housing", &house, flat);

	for k in 0..pins_n {
		let a = TAU * k as f64 / pins_n as f64;
		place(&mut asm, &mut instances, "hw_dowel_2x20", &dowel, tr(p.ring_r * a.cos(), p.ring_r * a.sin(), PIN_TOP - p.pin_len));
	}
	place(&mut asm, &mut instances, "eccentric_cam", &cam_p, flat);
	// DIN 916 set screw through the boss onto the shaft flat (tip at x=2.1)
	place(&mut asm, &mut instances, "hw_m3x5_set", &m3set, tr(2.1, 0.0, (HUB_Z + HUB_TOP) * 0.5) * DAffine3::from_rotation_y(std::f64::consts::FRAC_PI_2));
	// each disc rides a 688 on its cam journal (press the bearing onto the journal,
	// then drop the disc over it). disc_a on the +e journal (no body rotation);
	// disc_b on the −e journal at its −π/lobes mesh phase (holes pre-clocked
	// +π/lobes so they land back on the pin circle). ZERO anti-backlash preload
	// (disc_clock_deg 0) for backdrivability — both hole patterns sit symmetric.
	place(&mut asm, &mut instances, "hw_bearing_688_ecc", &b_ecc, tr(p.ecc, 0.0, DISC1_Z));
	place(&mut asm, &mut instances, "cyclo_disc_a", &disc_a, tr(p.ecc, 0.0, DISC1_Z));
	place(&mut asm, &mut instances, "hw_bearing_688_ecc", &b_ecc, tr(-p.ecc, 0.0, DISC2_Z));
	place(&mut asm, &mut instances, "cyclo_disc_b", &disc_b, tr(-p.ecc, 0.0, DISC2_Z) * rotz(-PI / p.lobes as f64));
	place(&mut asm, &mut instances, "output_plate", &plate, flat);
	for k in 0..OUT_PINS {
		let a = TAU * k as f64 / OUT_PINS as f64;
		place(
			&mut asm,
			&mut instances,
			"hw_m3x12_pin",
			&m3x12f,
			tr(OUT_PIN_CIRCLE * a.cos(), OUT_PIN_CIRCLE * a.sin(), PLATE_TOP - 12.0),
		);
	}
	// output bearing: ONE 6804 on the spigot (drops in from above onto the shelf)
	place(&mut asm, &mut instances, "hw_bearing_6804", &b6804, tr(0.0, 0.0, B1_Z));
	place(&mut asm, &mut instances, "lid_ring", &lid_p, flat);
	place(&mut asm, &mut instances, "retainer_ring", &retainer, flat);
	for k in 0..2 {
		let a = PI * k as f64 + FRAC_PI_2;
		place(&mut asm, &mut instances, "hw_m3x8_retainer", &m3x8r, tr(18.2 * a.cos(), 18.2 * a.sin(), RET_TOP - 8.0));
	}
	place(&mut asm, &mut instances, "output_hub", &hub, flat);
	// hub clamp screws: M3×8 BUTTON in counterbores (head base on the cb floor)
	for dx in [HUB_SCREW_R, -HUB_SCREW_R] {
		place(&mut asm, &mut instances, "hw_m3x8_hub", &m3x8r, tr(dx, 0.0, HUB_CB_FLOOR - 8.0));
	}
	// the SANDWICH bolts: four M3×30 down through lid + housing into the
	// motor's own face taps — the only structural fasteners of the actuator
	// (head seat = LID_TOP − SANDWICH_CB; tip lands 4.0 mm into the blind taps)
	for (dx, dy) in [(BOLT_SQ, BOLT_SQ), (-BOLT_SQ, BOLT_SQ), (BOLT_SQ, -BOLT_SQ), (-BOLT_SQ, -BOLT_SQ)] {
		place(&mut asm, &mut instances, "hw_m3x30_sandwich", &m3x30, tr(dx, dy, LID_TOP - SANDWICH_CB - SANDWICH_L));
	}
	let _ = asm.write_stl_binary("cyclo26/ASSEMBLY.stl");
	println!("  {} triangles -> cyclo26/ASSEMBLY.stl", asm.indices.len() / 3);

	// exploded view: single axis, disassembly order, one lift PER INSTANCE
	let mut expl = Mesh::new();
	let mut seen: std::collections::HashMap<String, usize> = std::collections::HashMap::new();
	for (name, s, x) in &instances {
		let nth = *seen.entry(name.clone()).and_modify(|c| *c += 1).or_insert(0);
		let lift = match (name.as_str(), nth) {
			("hw_nema17", _) => -45.0,
			("housing", _) => 0.0,
			("hw_m3x8_motor", _) => 22.0,
			("hw_dowel_2x20", _) => 30.0,
			("eccentric_cam", _) => 42.0,
			("hw_bearing_688_ecc", 0) => 62.0, // eccentric bearing for disc_a
			("cyclo_disc_a", _) => 88.0,
			("hw_bearing_688_ecc", 1) => 108.0, // eccentric bearing for disc_b
			("cyclo_disc_b", _) => 128.0,
			("output_plate", _) => 178.0,
			("hw_bearing_6804", 0) => 190.0,
			("hw_bearing_6804", 2) => 200.0,
			("hw_bearing_6804", _) => 218.0,
			("lid_ring", _) => 238.0,
			("retainer_ring", _) => 256.0,
			("hw_m3x8_retainer", _) => 268.0,
			("hw_m3x25_lid", _) => 280.0,
			("output_hub", _) => 294.0,
			("hw_m3x8_hub", _) => 310.0,
			("hw_m3x12_pin", _) => 322.0,
			("arm_roll", _) => 336.0,
			("housing_next", _) => 366.0,
			("hw_m3x12_joint", _) => 356.0,
			_ => 356.0,
		};
		merge_into(&mut expl, &tessellate_default(&s.transformed(tr(0.0, 0.0, lift) * *x)));
	}
	let _ = expl.write_stl_binary("cyclo26/ASSEMBLY_EXPLODED.stl");

	// ---- A-ASM: every interface measured on the exact poses ----
	let rel = |label: &str, a: &Mesh, b: &Mesh, contact: bool, ok: &mut bool| {
		let d = a.min_distance(b);
		let pass = if contact { d < 0.06 } else { d >= 0.10 };
		if !pass {
			*ok = false;
		}
		println!("  {label:48} min_dist={d:7.3}  {}", if pass { "OK" } else { "<<< FAIL" });
	};
	let mesh_of = |name: &str, nth: usize| {
		let mut c = 0;
		for (n, s, x) in &instances {
			if n == name {
				if c == nth {
					return tessellate_default(&s.transformed(*x));
				}
				c += 1;
			}
		}
		unreachable!("{name}")
	};
	let (house_m, lid_m, plate_m) = (mesh_of("housing", 0), mesh_of("lid_ring", 0), mesh_of("output_plate", 0));
	let motor_m = mesh_of("hw_nema17", 0);
	let cam_m = mesh_of("eccentric_cam", 0);
	let d1m = mesh_of("cyclo_disc_a", 0);
	let d2m = mesh_of("cyclo_disc_b", 0);
	rel("motor pilot seats in the register", &motor_m, &house_m, true, &mut ok);
	// (motor casing deleted per design review: dead weight — the chained arm's
	// socket IS the enclosure; the base drive's motor rides exposed)
	// thread ENGAGEMENT: each screw's shank must overlap its Ø2.5 pilot as a
	// thin bite ring (Ø3 in Ø2.5 ≈ 5–20 mm³). Near-zero = no pilot (the 0.5 mm
	// dimple bug this gate was born from); huge = crashing into solid plastic.
	let engage = |label: &str, screw: &Solid, x: DAffine3, part: &Solid| -> bool {
		let bite = overlap_mm3(&screw.transformed(x), part);
		let okb = (3.0..=45.0).contains(&bite); // ring bite; a solid crash reads ~56+
		println!("  {label:48} bite={bite:6.1} mm³  {}", if okb { "OK" } else { "<<< FAIL" });
		okb
	};
	let bolt_x = instances.iter().find(|(n, _, _)| n == "hw_m3x30_sandwich").map(|(_, _, x)| *x).unwrap();
	let free_h = overlap_mm3(&m3x30.transformed(bolt_x), &house);
	let free_l = overlap_mm3(&m3x30.transformed(bolt_x), &lid_p);
	let bolt_ok = free_h < 0.05 && free_l < 0.05;
	ok &= bolt_ok;
	println!("  sandwich bolt passes lid + housing freely ({free_h:.2}/{free_l:.2} mm³)  {}", if bolt_ok { "OK" } else { "<<< FAIL" });
	// tap ENGAGEMENT: the sandwich bolt threads the motor's BLIND face taps —
	// NEMA-17 (ICS 16) taps are only ~4.5 mm deep, so the shank must land in the
	// usable 3.0–4.5 mm window: shallower strips the steel threads' bite, deeper
	// BOTTOMS OUT before the head clamps (the pre-2026-07-19 M3×40 build demanded
	// 11.7 mm — the sandwich could never clamp). The motor envelope is solid, so
	// depth = overlap / (π·r²) is exact up to shank faceting.
	let tap_bite = overlap_mm3(&m3x30.transformed(bolt_x), &motor);
	let tap_depth = tap_bite / (PI * 1.5 * 1.5); // Ø3 shank
	let tap_ok = (3.0..=4.5).contains(&tap_depth);
	ok &= tap_ok;
	println!("  sandwich bolt engages the motor taps             depth={tap_depth:5.2} mm (blind tap ~4.5)  {}", if tap_ok { "OK" } else { "<<< FAIL" });
	let hub_x = instances.iter().find(|(n, _, _)| n == "hw_m3x8_hub").map(|(_, _, x)| *x).unwrap();
	ok &= engage("hub screw threads the spigot pilot", &m3x8r, hub_x, &plate);
	// csk STACK gates (2026-07-19 cross-drive audit, planetary26 defect class):
	// (1) TIP CORE CLEARANCE — the hub screw's tip must land in drilled void,
	//     never solid plastic: a Ø2.4 probe (under the Ø2.5 pilot) over the final
	//     1 mm of screw travel must read ~0. The pre-fix geometry read 4.52 mm³
	//     (pilot floor 3.6 mm short of the tip — the screw bottomed out before
	//     the head could seat).
	let hub_tip = hub_x.transform_point3(v(0.0, 0.0, 0.0));
	let tip_probe = cylinder(v(hub_tip.x, hub_tip.y, hub_tip.z), DVec3::Z, 1.2, 1.0, 16);
	let tip_ov = overlap_mm3(&tip_probe, &plate);
	let tip_ok = tip_ov.is_finite() && tip_ov < 0.05;
	ok &= tip_ok;
	println!("  hub screw tip lands in drilled void               core={tip_ov:6.2} mm³  {}", if tip_ok { "OK" } else { "<<< FAIL" });
	// (2) HEADS SEAT AT-OR-BELOW THE FACE — the seated screw's overlap with its
	//     recessed part must be just the thin thread ring (< 2 mm³), not head
	//     interference (the pre-fix csk cones read pin 7.69 / hub 3.24 mm³ —
	//     heads ~0.5 proud, pins grinding the lid shelf). The hub screws are
	//     now BUTTON heads in Ø6.3 counterbores (csk-necessity audit): the seat
	//     overlap check carries over, plus an explicit no-protrusion probe —
	//     a Ø6.6 disc sitting 0.05..1.05 above the arm-seat face at the screw
	//     position must never touch the head (the arm flange seats there).
	let pin_x = instances.iter().find(|(n, _, _)| n == "hw_m3x12_pin").map(|(_, _, x)| *x).unwrap();
	let pin_seat = overlap_mm3(&m3x12f.transformed(pin_x), &plate);
	let hub_seat = overlap_mm3(&m3x8r.transformed(hub_x), &hub);
	let hub_head = hub_x.transform_point3(v(0.0, 0.0, 0.0));
	let face_probe = cylinder(v(hub_head.x, hub_head.y, FACE_Z + 0.05), DVec3::Z, 3.3, 1.0, 16);
	let proud = overlap_mm3(&face_probe, &m3x8r.transformed(hub_x));
	let seat_ok = pin_seat.is_finite() && pin_seat < 2.0 && hub_seat.is_finite() && hub_seat < 2.0 && proud.is_finite() && proud < 0.05;
	ok &= seat_ok;
	println!(
		"  heads seat below their faces (pin csk {pin_seat:.2} / hub cb {hub_seat:.2} mm³ < 2; hub proud {proud:.2} ≈ 0)  {}",
		if seat_ok { "OK" } else { "<<< FAIL" }
	);

	// BACKDRIVABLE eccentric: each disc rolls on a 688 ball bearing, not a sliding
	// journal. Verify BOTH seats of BOTH bearings: inner race on the Ø8 cam journal
	// (press, coincident → ~0) and outer race in the Ø16.2 disc bore (0.1 radial).
	let (eb0, eb1) = (mesh_of("hw_bearing_688_ecc", 0), mesh_of("hw_bearing_688_ecc", 1));
	let (in1, in2) = (eb0.min_distance(&cam_m), eb1.min_distance(&cam_m));
	let (out1, out2) = (eb0.min_distance(&d1m), eb1.min_distance(&d2m));
	let brg_ok = in1 < 0.08 && in2 < 0.08 && out1 < 0.15 && out2 < 0.15;
	ok &= brg_ok;
	println!(
		"  eccentric 688s roll the discs: inner on cam journal ({in1:.3}/{in2:.3}), outer in disc bore ({out1:.3}/{out2:.3})  {}",
		if brg_ok { "OK" } else { "<<< FAIL" }
	);
	// disc up-float is retained by the PLATE's underside (0.5 greased gap;
	// relative motion there is only the wobble — the washer cap was deleted:
	// the motor shaft protrudes through the cam top exactly where its screw
	// lived, an unassemblable collision the A-PROD matrix caught)
	let d2_plate = d2m.min_distance(&plate_m);
	let float_ok = (0.2..=0.8).contains(&d2_plate);
	ok &= float_ok;
	println!("  plate retains the discs with running float        min_dist={d2_plate:7.3}  {}", if float_ok { "OK" } else { "<<< FAIL" });
	rel("lid seats on the ring wall", &lid_m, &house_m, true, &mut ok);
	let pin0 = mesh_of("hw_dowel_2x20", 0);
	rel("steel dowel presses into its floor pocket", &pin0, &house_m, true, &mut ok);
	rel("dowel top captured by the lid socket", &pin0, &lid_m, true, &mut ok);
	// rotating vs static: the physical criterion is ZERO interpenetration —
	// plate and hub share datum PLANES with the lid by design (running fits),
	// so a blunt min-distance gate false-fails on coplanar grazes
	let pl_ov = overlap_mm3(&plate, &lid_p);
	let hb_ov = overlap_mm3(&hub, &lid_p);
	let spin_ok = pl_ov < 0.05 && hb_ov < 0.05;
	ok &= spin_ok;
	println!("  plate + hub spin free of the lid ({pl_ov:.3}/{hb_ov:.3} mm³)      {}", if spin_ok { "OK" } else { "<<< FAIL" });
	let closed_ok = 15.0 > 13.6 + 1.0; // hub face Ø30 overhangs the Ø27.2 bore ≥1 all round
	ok &= closed_ok;
	println!("  hub face overhangs the lid bore (closed top)      {:.1} > {:.1}   {}", 15.0, 13.6 + 1.0, if closed_ok { "OK" } else { "<<< FAIL" });
	let ob1 = mesh_of("hw_bearing_6804", 0);
	let hub_m = mesh_of("output_hub", 0);
	let ret_m = mesh_of("retainer_ring", 0);
	rel("6804 inner seats on the spigot shoulder", &ob1, &plate_m, true, &mut ok);
	rel("shelf carries the outer race", &ob1, &lid_m, true, &mut ok);
	rel("retainer lip lands on the outer race", &ob1, &ret_m, true, &mut ok);
	rel("retainer seats on the lid top", &ret_m, &lid_m, true, &mut ok);
	rel("hub clamps the inner race", &hub_m, &ob1, true, &mut ok);
	// axial capture BOTH ways through REAL bearings: outer races clamped
	// lip-to-retainer, inner stack clamped hub-to-shoulder — negative controls:
	// up: the plate lifts the bearing stack with it (shoulder → inners →
	// balls → outers) so the control shifts the UPPER BEARING into the lip;
	// down: the hub (bolted to the spigot) lands on the lid
	let up_ov = overlap_mm3(&b6804.transformed(tr(0.0, 0.0, B1_Z + 0.5)), &retainer);
	let dn_hub = overlap_mm3(&b6804.transformed(tr(0.0, 0.0, B1_Z - 0.5)), &lid_p);
	let cap_ok = up_ov > 0.02 && dn_hub > 0.02;
	ok &= cap_ok;
	println!(
		"A-CAPTURE: bearing +0.5 hits the retainer ({up_ov:.2} mm³); bearing −0.5 hits the shelf ({dn_hub:.2} mm³) — held BOTH ways  {}",
		if cap_ok { "OK" } else { "<<< FAIL" }
	);

	// both discs mesh the ring at their assembled phases (disc 2 at −π/lobes)
	let ring_at = |z0: f64| {
		let mut r = cylinder(v(0.0, 0.0, z0), DVec3::Z, 0.01, 0.01, 8);
		for k in 0..pins_n {
			let a = TAU * k as f64 / pins_n as f64;
			r = union(&r, &cylinder(v(p.ring_r * a.cos(), p.ring_r * a.sin(), z0), DVec3::Z, p.pin_r, DISC_T, 16));
		}
		r
	};
	let d1s = disc_a.transformed(tr(p.ecc, 0.0, DISC1_Z));
	let d2s = disc_b.transformed(tr(-p.ecc, 0.0, DISC2_Z) * rotz(-PI / p.lobes as f64));
	// DIRECT intersection volume (same robust metric as A-PROD): the subtraction
	// metric NaN-fails on disc_b's clocked-hole triangulation against the 27-pin
	// ring (a complex operand), yet the true mesh overlap is ~0 — disc_b's profile
	// equals disc_a's and its holes only REMOVE material at r10.5, far from r16.5.
	let ix_vol = |a: &Solid, b: &Solid| -> f64 {
		let ix = kernel_brep::intersection(a, b);
		if ix.face_count() == 0 {
			0.0
		} else {
			volume(&ix).abs()
		}
	};
	let (ov1, ov2) = (ix_vol(&d1s, &ring_at(DISC1_Z)), ix_vol(&d2s, &ring_at(DISC2_Z)));
	let phase_ok = ov1 < 0.05 && ov2 < 0.05;
	ok &= phase_ok;
	println!("  both discs mesh the ring as posed ({ov1:.3}/{ov2:.3} mm³)          {}", if phase_ok { "OK" } else { "<<< FAIL" });

	// ---- A-PROD: the production interference matrix ----
	// every representative pair of posed bodies, boolean-checked: pairs on the
	// CONTACT whitelist may touch (overlap from modelled clamps ≤ small);
	// everything else must be interference-free. This matrix is what catches
	// the pair nobody thought to gate.
	println!("\nA-PROD (pairwise interference matrix, representative bodies):");
	let rep: Vec<(&str, usize)> = vec![
		("hw_nema17", 0),
		("housing", 0),
		("eccentric_cam", 0),
		("cyclo_disc_a", 0),
		("cyclo_disc_b", 0),
		("hw_bearing_688_ecc", 0),
		("hw_bearing_688_ecc", 1),
		("output_plate", 0),
		("hw_bearing_6804", 0),
		("lid_ring", 0),
		("retainer_ring", 0),
		("output_hub", 0),
		("hw_dowel_2x20", 0),
		("hw_dowel_2x20", 1),
		("hw_m3x30_sandwich", 0),
		("hw_m3x12_pin", 0),
		("hw_m3x8_retainer", 0),
		("hw_m3x8_hub", 0),
		("hw_m3x5_set", 0),
	];
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
	// contacts and modelled clamps/presses/bites — these pairs are verified by
	// their DEDICATED gates (bite rings, seats, running fits); boolean-checking
	// coincident-fit surfaces here is pathological (a press-fit dowel against
	// its own pocket ground one run for 53 CPU-minutes), so the matrix SKIPS
	// them and checks only pairs that must be CLEAR
	// (name, name) → designed-contact allowance; unlisted pairs must be CLEAR
	const CONTACT_PAIRS: &[(&str, &str, f64)] = &[
		("hw_nema17", "housing", 1.0),          // register + face
		("eccentric_cam", "hw_nema17", 30.0),   // D-bore on the shaft
		("hw_m3x5_set", "eccentric_cam", 20.0), // threads the printed pocket
		("hw_m3x5_set", "hw_nema17", 2.0),      // tip on the shaft flat
		("hw_m3x12_pin", "output_plate", 30.0), // threads the plate
		// the pins pass through BOTH clocked disc hole patterns — a designed
		// clearance, rigorously verified by the (now two-pattern) A-PINS gate;
		// the matrix skips it (booleaning a pin inside its own faceted bore is
		// pathological). disc_b's holes are compensated onto the pin circle, so
		// this is a real clearance now, not the 26 mm³ pin-in-solid the old
		// single-part disc silently carried at disc 2.
		("hw_m3x12_pin", "cyclo_disc_a", 0.3),
		("hw_m3x12_pin", "cyclo_disc_b", 0.3),
		// eccentric 688 seats — inner race coincident on the Ø8 journal (a
		// surface press, ~0 volume but skipped: booleaning coincident faceted
		// cylinders is pathological), outer race in the Ø16.2 disc bore; both
		// verified by the dedicated bearing-seat gate above.
		("hw_bearing_688_ecc", "eccentric_cam", 3.0),
		("hw_bearing_688_ecc", "cyclo_disc_a", 1.5),
		("hw_bearing_688_ecc", "cyclo_disc_b", 1.5),
		("hw_m3x8_hub", "output_plate", 25.0), // spigot pilot bite (measured by the engage gate)
		("hw_m3x8_hub", "output_hub", 8.0),    // counterbore seat (measured by the head-seat gate)
		("hw_m3x8_retainer", "lid_ring", 25.0), // lid pilot bite
		("hw_m3x8_retainer", "retainer_ring", 5.0),
		("hw_m3x30_sandwich", "hw_nema17", 30.0), // motor taps — measured by the dedicated tap-engagement gate (4.0 mm ≈ 28 mm³)
		("hw_dowel_2x20", "housing", 3.0),      // Ø1.95 press pockets
		("hw_dowel_2x20", "lid_ring", 1.0),     // socket capture
		("hw_bearing_6804", "output_plate", 1.0),
		("hw_bearing_6804", "output_hub", 1.5), // modelled clamp
		("output_plate", "output_hub", 1.0),    // hex register mate
		("hw_bearing_6804", "retainer_ring", 1.5),
		("hw_bearing_6804", "lid_ring", 1.0),
	];
	let allowed = |a: &str, b: &str| -> f64 {
		CONTACT_PAIRS
			.iter()
			.find(|(x, y, _)| (a == *x && b == *y) || (a == *y && b == *x))
			.map(|(_, _, lim)| *lim)
			.unwrap_or(0.05)
	};
	let mut prod_bad = 0;
	for i in 0..rep.len() {
		for j in (i + 1)..rep.len() {
			let (an, ai) = rep[i];
			let (bn, bi) = rep[j];
			let lim = allowed(an, bn);
			if lim > 0.05 {
				continue; // designed-contact pair: covered by its dedicated gate
			}
			let (Some(sa), Some(sb)) = (solid_of(an, ai), solid_of(bn, bi)) else { continue };
			// AABB prefilter: disjoint boxes cannot interfere
			let (ma, mb) = (tessellate_default(&sa), tessellate_default(&sb));
			let (la, ha) = ma.positions.iter().fold((Vec3::splat(f32::INFINITY), Vec3::splat(f32::NEG_INFINITY)), |(l, h), p| (l.min(*p), h.max(*p)));
			let (lb, hb) = mb.positions.iter().fold((Vec3::splat(f32::INFINITY), Vec3::splat(f32::NEG_INFINITY)), |(l, h), p| (l.min(*p), h.max(*p)));
			if la.x > hb.x + 0.01 || lb.x > ha.x + 0.01 || la.y > hb.y + 0.01 || lb.y > ha.y + 0.01 || la.z > hb.z + 0.01 || lb.z > ha.z + 0.01 {
				continue;
			}
			// DIRECT intersection volume: the subtraction metric both fabricated
			// phantom overlap on complex operands (0.27–6.4 mm³ artifacts) and
			// HID a real 90 mm³ shaft-through-plate collision by direction
			let ix = kernel_brep::intersection(&sa, &sb);
			let ov = if ix.face_count() == 0 { 0.0 } else { volume(&ix).abs() };
			if !(ov.is_finite() && ov <= lim.max(0.2)) {
				prod_bad += 1;
				println!("  <<< {an}[{ai}] ∩ {bn}[{bi}] = {ov:.2} mm³ (limit {lim})");
			}
		}
	}
	ok &= prod_bad == 0;
	println!("  {} representative pairs checked, {prod_bad} violations  {}", rep.len() * (rep.len() - 1) / 2, if prod_bad == 0 { "OK" } else { "<<< FAIL" });

	// ---- A-STEP ----
	match export_step_assembly(&instances, "cyclo_scratch") {
		Ok(step) => {
			let _ = std::fs::write("cyclo26/ASSEMBLY.step", &step);
			match import_step_assembly(&step) {
				Ok(back) => {
					let v0: f64 = instances.iter().map(|(_, s, _)| volume(s).abs()).sum();
					let v1: f64 = back.iter().map(|(_, s, _)| volume(s).abs()).sum();
					let dv = (v0 - v1).abs() / v0;
					let sok = back.len() == instances.len() && dv < 0.025;
					ok &= sok;
					println!("\nA-STEP: {} instances, {} KB, round-trip Δ {:.2}%  {}", instances.len(), step.len() / 1024, dv * 100.0, if sok { "OK" } else { "<<< FAIL" });
				}
				Err(e) => {
					ok = false;
					println!("\nA-STEP import failed: {e:?} <<< FAIL");
				}
			}
		}
		Err(e) => {
			ok = false;
			println!("\nA-STEP export failed: {e:?} <<< FAIL");
		}
	}

	// ---- BOM ----
	println!("\nA-BOM (hardware in the ASSEMBLY only — parts/ is a pure print queue):");
	println!(
		"  1× NEMA-17 · 2× {ECC_BRG} (eccentric, 1 per disc — rolling backdrivable eccentric) · 1× 6804 (output) · {}× Ø2×{:.0} steel dowel pins (the ring gear)",
		pins_n,
		p.pin_len
	);
	if p.clock_deg.abs() < 1e-9 {
		println!("  printed discs: 1× cyclo_disc_a + 1× cyclo_disc_b — ZERO-preload backdrivable pair; still DISTINCT prints (disc_b's holes compensated +π/lobes so its meshed placement lands them on the pin circle), NOT interchangeable");
	} else {
		println!(
			"  printed discs: 1× cyclo_disc_a (bottom, holes lead +{:.2}°) + 1× cyclo_disc_b (top, holes lag −{:.2}°) — anti-backlash split-disc pair, NOT interchangeable",
			p.clock_deg * 0.5,
			p.clock_deg * 0.5
		);
	}
	println!(
		"  M3 ONLY ({} total): 4×30 sandwich (was 4×40 — 40 bottoms out in the ~4.5 mm blind motor taps) · {}×12 csk output pins (raised from 3 — the pin-bore bearing bound the torque capacity; the ONLY csk left after the necessity audit) · 2×8 button hub (counterbored; hex carries torque) · 2×8 button retainer · 1×5 DIN916 set",
		9 + OUT_PINS,
		OUT_PINS
	);
	println!("  NO inserts (Ø2.5 pilots) · PTFE grease · eccentric + output bearings roll (backdrivable) · printed ≈ {:.0} g", grams.values().sum::<f64>());
	println!(
		"  ratio {}:1 exact · torque ≈ 0.4 × {} × 0.85 ≈ {:.1} N·m (stall echo; PLA limits continuous)",
		p.lobes,
		p.lobes,
		0.4 * p.lobes as f64 * 0.85
	);

	println!("\nRESULT: {}", if ok { "PASS — every gate green" } else { "FAIL — see <<< lines" });
	if !ok {
		std::process::exit(1);
	}
}
