//! PLAN-26 — a 26:1 BACKDRIVABLE two-stage involute PLANETARY in the SAME
//! Cricket-class envelope as cyclo26 and harmonic26: NEMA-17 square (42.3²,
//! flush), same register, same 4× M3×30 through-bolt sandwich, same
//! Ø20-spigot-on-6804 output stack (shelf + top retainer, hex-register hub,
//! 6×M3 Ø20 arm circle, FACE_Z). LID, RETAINER and HUB are byte-identical
//! parts across all three drives.
//!
//! Why this replaced the Wolfrom sibling: a Wolfrom's high single-bay ratio
//! comes from a near-cancelling second ring, so it circulates latent power and
//! runs at low efficiency — which fights backdrivability (it holds pose partly
//! by internal friction). A SIMPLE involute planetary is backdrivable BY
//! ARCHITECTURE: involute spur meshes are non-self-locking (≈97 %/mesh), the
//! per-stage ratio is low (5.2 and 5.0), the output rides a real ball bearing,
//! and there is no flexure/worm. We trade the Wolfrom's torque density for
//! easy backdriving — accepted.
//!
//! Architecture (input on the motor shaft, carrier out, both rings fixed):
//! - STAGE 1 (input) m0.6: sun 15T on the Ø5 D-bore shaft, 3× planet 24T,
//!   ring 63T printed INTO the housing. Ratio 1 + 63/15 = 5.2. Carrier out.
//! - The stage-1 CARRIER is one printed part with the stage-2 SUN (12T)
//!   integral on top — it takes the ω/5.2 orbit straight into stage 2.
//! - STAGE 2 (output) m0.79: sun 12T (on the carrier-1 hub), 3× planet 18T,
//!   ring 48T printed INTO the housing. Ratio 1 + 48/12 = 5.0. Carrier out.
//! - The stage-2 carrier IS the output: it carries the Ø20 spigot up onto the
//!   6804 and the hex the hub registers on.
//! - EXACT ratio (rational, proven by integers): (S1+R1)(S2+R2) = 78·60 =
//!   4680 = 26·15·12 = 26·S1·S2 → 5.2 × 5.0 = 26.000 EXACTLY. Cross-checked at
//!   runtime through `kernel_model::kinematics::EpicyclicTrain::simple_ratio`.
//!
//! Two independent modules (m0.6 input, m0.79 output) are forced by the
//! envelope AND happen to be the right call: the 63T input ring needs m0.6 to
//! clear the 42.3 square, while the 12T output sun needs the bigger m0.79 to
//! keep a printable rim over the Ø5 shaft — and the larger output-stage teeth
//! are exactly where the full output torque lands. Both stages orbit at ~11.7
//! /11.85 mm centre distance.
//!
//! Backdrivability is a FRICTION/efficiency property; the kinematic gates here
//! and in the sim measure GEOMETRY (exact 26:1, non-interference, fits) only.
//! We DESIGN for backdrivability (non-self-locking involute spurs, low ratio
//! per stage, output on a 6804, M3 STEEL planet journals, no preload) and say
//! so — but we do NOT and CANNOT certify a backdrive torque here; confirm it by
//! hand on the printed drive.
//!
//! HARDWARE UNIFICATION (2026-07): the six planets rode greased PRINTED PETG
//! posts (PETG-on-PETG sliding journals — high, gritty friction). They now ride
//! the kit's shared M3 hardware, for swappability across the three drives AND to
//! cut journal friction (steel-on-PETG beats PETG-on-PETG). The PREFERRED
//! unification — a 693ZZ (Ø8×4) pressed into each planet on an M3 axle, the SAME
//! roller the harmonic drive uses — was fit-checked and REJECTED here: the 693
//! RIM passes (2.45 / 2.12 mm ≥ 1.5) but its 4.0 mm WIDTH did NOT fit the then
//! 3.6 / 3.4 mm planet faces in a ~3.8 mm inter-carrier pocket (the two stages
//! are near-coaxial, CD1 11.70 ≈ CD2 11.85). The v2 band raise (fw2 4.0, pocket
//! 4.4) reopens the STAGE-2 693 option — recorded in the ledger as an open
//! friction upgrade; stage 1 (fw 3.6) still refuses it, and the drive keeps ONE
//! journal design across both stages. The M3×8 / M3×12 screw shanks
//! are likewise too LONG (8 / 12 mm) for the ~5.4–5.9 mm planet-axle pockets:
//! they would gore the fixed housing floor (stage 1) or the differently-rotating
//! neighbour carrier (stage 2). The landing: 6× M3×5 DIN916 — the SAME set screw
//! the harmonic wave generator uses — as short steel journal axles, the only kit
//! M3 that fits. A-GEOM prints the full 693-vs-M3 fit ledger.
//!
//! V2 STRENGTH/STIFFNESS PASS (2026-07-11, planetary26/v2 receipts): tooth
//! counts, envelope, interfaces and the shared lid/retainer/hub are UNCHANGED.
//! Three levers, all receipt-driven: (1) stage-2 mesh face width 3.4 → 4.0
//! (S2_Z1/R2_TEETH1/C2_Z0 raised 0.6 — output-tooth bending ∝ 1/fw); (2) ISO 53
//! profile shift ±X2 = 0.14 on the output stage (sun +, planet/ring −), the
//! measured Lewis balance point of the 12T sun vs 18T planet, both meshes still
//! at standard centre distance; (3) output carrier plate 1.6 → 3.4 thick with
//! BLIND self-locating axle seats (torsional windup −20%, seat-ligament stress
//! halved). Honest cost: the stage-2 axle journal now covers 3.2 of the 4.0
//! planet face (80%, was 94%) — the M3×5 axle length is fixed kit hardware.
//! NOTE: DESIGN.md/params.csv still describe the pre-v2 fw/pocket numbers
//! (3.4/3.6 faces, 3.8 pocket); the printed A-GEOM ledger is the live truth.
//!
//! Run: cargo run --example planetary26 -p kernel-model --release
//! (writes planetary26/parts/*.stl + ASSEMBLY + STEP; exit 1 on any FAIL)

use kernel_brep::math::{DAffine3, DVec2, DVec3};
use kernel_brep::{
	cone, cuboid, cylinder, difference, export_step_assembly, extrude, import_step_assembly, intersection, revolve,
	teardrop_hole, tessellate_default, try_difference, union, validate, volume, Solid,
};
use kernel_core::math::Vec3;
use kernel_core::Mesh;
use kernel_model::kinematics::EpicyclicTrain;
use kernel_model::parts::{button_head_screw, deep_groove_bearing, flat_head_screw, involute_ring_outline_shifted, nema_motor, set_screw};
use std::f64::consts::{FRAC_PI_2, PI, TAU};

// ---- gearset — EXACTLY 26:1 across two simple planetary stages (see header) ----
const S1_T: usize = 15; // stage-1 sun (on the motor shaft)
const P1_T: usize = 24; // stage-1 planet ×3
const R1_T: usize = 63; // stage-1 ring (fixed, in the housing)
const S2_T: usize = 12; // stage-2 sun (integral on the stage-1 carrier)
const P2_T: usize = 18; // stage-2 planet ×3
const R2_T: usize = 48; // stage-2 ring (fixed, in the housing)
const N_PL: usize = 3;
const M1: f64 = 0.6; // stage-1 module — packs the 63T ring in the 42.3 square
const M2: f64 = 0.79; // stage-2 module — 12T sun clears the Ø5 shaft, teeth carry output torque
const PA: f64 = 25.0; // pressure angle — keeps the 12T sun clear of undercut (≈11.2T floor at 25°)
const LASH: f64 = 0.05; // backlash thinning per flank (printable meshes)
// v2 strength pass: ISO 53 profile shift on the OUTPUT stage, x = +X2 on the 12T
// sun / −X2 on the planet AND ring. Equal-and-opposite shifts keep BOTH meshes at
// the standard centre distance and working pressure angle (sun+planet: x₁+x₂ = 0;
// planet+ring: x_ring = x_planet — see involute_ring_outline_shifted docs), and
// every tip/root clearance is x-invariant (both members move by x·m together).
// X2 = 0.14 is the measured Lewis balance point where the 12T sun root and the
// 18T planet root carry EQUAL bending stress (sun alone was 22% weaker); the
// sim's S1 interference sweep re-proves the shifted meshes run clean.
const X2: f64 = 0.14;
const CD1: f64 = M1 * ((S1_T + P1_T) as f64) / 2.0; // 11.70
const CD2: f64 = M2 * ((S2_T + P2_T) as f64) / 2.0; // 11.85

// EXACT-26 proof as a rational identity (S1+R1)(S2+R2) == 26·S1·S2 — no float:
#[allow(clippy::manual_is_multiple_of)]
const _: () = assert!((S1_T + R1_T) * (S2_T + R2_T) == 26 * S1_T * S2_T, "product ratio must be EXACTLY 26:1");
const _: () = assert!(R1_T == S1_T + 2 * P1_T && R2_T == S2_T + 2 * P2_T, "planet-fit: ring = sun + 2·planet");
#[allow(clippy::manual_is_multiple_of)]
const _: () = assert!((S1_T + R1_T) % N_PL == 0 && (S2_T + R2_T) % N_PL == 0, "equal-spacing: (S+R) divisible by n");

// ---- the assembled stack (identical to cyclo26/harmonic26 where shared) ----
const NEMA_W: f64 = 42.3;
const BACK_T: f64 = 5.5;
const REG_D: f64 = 22.3;
const REG_T: f64 = 2.2;
const SHAFT_BORE_D: f64 = 10.0;
const RING_TOP: f64 = 19.4;
const TOWER_BOT: f64 = 21.9;
const B6804_OD: f64 = 32.0;
const B6804_W: f64 = 7.0;
const B1_Z: f64 = TOWER_BOT + 1.2; // 23.1
const LIP_Z: f64 = B1_Z + B6804_W; // 30.1
const LID_TOP: f64 = 30.3;
const RET_TOP: f64 = 32.3;
const FACE_Z: f64 = 34.5;
const HEX_AF: f64 = 12.0;
const SPIG_R: f64 = 10.0;
const PLATE_R: f64 = 13.6;
const BOLT_SQ: f64 = 15.5;
// Sandwich-bolt stack: M3×SANDWICH_L button heads seat in Ø6.5×SANDWICH_CB lid
// counterbores; tap engagement = SANDWICH_CB + SANDWICH_L − LID_TOP = 4.0 mm,
// inside the ~4.5 mm blind-tap depth of the NEMA-17 face (ICS 16). Gated in
// A-ASM (tap-engagement) — the 2026-07-19 audit found the previous M3×40/cb 3.0
// combination demanded 12.7 mm and would bottom out before clamping (same
// defect fixed in cyclo26 and harmonic26 the same day).
const SANDWICH_L: f64 = 30.0; // M3×30 sandwich through-bolt length
const SANDWICH_CB: f64 = 4.3; // lid head-counterbore depth
const IF_PILOT_H: f64 = 2.5;
const ARM_CIRCLE_R: f64 = 10.0;

// ---- gear-bay stack (motor face z = 0; two planetary stages stacked axially) ----
const S1_Z0: f64 = 6.0; // stage-1 band 6.0..9.6 (sun1 + planet1, fw 3.6)
const S1_Z1: f64 = 9.6;
const C1_Z0: f64 = 9.8; // carrier-1 plate 9.8..11.4 (posts hang DOWN to 5.9)
const C1_Z1: f64 = 11.4;
const S2_Z0: f64 = 11.4; // sun2 rises from the carrier-1 plate 11.4..15.6
const S2P_Z0: f64 = 11.6; // planet2 band 11.6..15.6 (0.2 above the carrier-1 plate)
const S2_Z1: f64 = 15.6; // raised 15.0→15.6 (v2 strength pass): stage-2 mesh face width
                         // 3.4→4.0 — the OUTPUT teeth are the drive's weakest members and
                         // bending stress scales 1/fw; the room comes out of the (shortened)
                         // output boss, every clearance in the chain kept at its old value
const C2_Z0: f64 = 15.8; // carrier-2 (OUTPUT) plate bottom (axles hang DOWN into the planets),
                         // 0.2 above the raised planet2 band top S2_Z1
const C2_Z1: f64 = 19.2; // plate top — thickened 16.8→19.2 (v2 stiffness pass): the output
                         // plate is the torsion member between the axle seats and the boss;
                         // 19.2 keeps 0.2 to the lid body (lid material starts at RING_TOP
                         // 19.4) and the r14.5 rim stays inside the r15.2 housing top bore
const R1_TEETH1: f64 = 9.7; // ring1 teeth BACK_T..9.7
const R2_TEETH0: f64 = 11.5; // ring2 teeth 11.5..15.7
const R2_TEETH1: f64 = 15.7; // raised with the stage-2 band (0.1 above the planet2 top)
const GAP_BORE_R: f64 = 15.0; // smooth housing bore between the two ring bands (< ring tips → no tooth nick)
const TOP_BORE_R: f64 = 15.2; // smooth housing bore above ring2 for the output boss
const SHAFT_D: f64 = 5.0; // Ø5 NEMA-17 motor shaft (the carriers journal it; sun1 D-bores it)
const J_CLR: f64 = 0.1; // shaft-journal radial running clearance
// --- planet axles: UNIFIED to the kit's shared M3 hardware. The planets ride
// the steel Ø3 body of an M3×5 DIN916 set screw (the SAME part the harmonic
// wave generator uses) as a metal journal, replacing the old greased PETG post.
// The A-GEOM 693-vs-M3 fit ledger below records WHY the 693ZZ roller (harmonic's
// shared bearing) and the M3×8/M3×12 shanks do not fit, and this M3×5 does.
const AXLE_D: f64 = 3.0; // M3 nominal — the journal Ø the planet rides on
const AXLE_BORE_D: f64 = 3.3; // planet running bore = M3 + 0.15/side running clearance
const AXLE_SEAT_D: f64 = 2.9; // carrier press seat = M3 − 0.05/side (light PETG press)
const AXLE_LEN: f64 = 5.0; // M3×5 DIN916 — the only kit M3 short enough for the pocket
const AXLE_SEAT_H: f64 = 1.6; // press depth into the carrier plate
const AXLE_JOURNAL: f64 = AXLE_LEN - AXLE_SEAT_H; // 3.4 mm of journal hangs below the carrier
const B693_OD: f64 = 8.0; // 693ZZ roller OD (harmonic's shared bearing) — fit-checked, rejected on width
const B693_W: f64 = 4.0; // 693ZZ width — the axial dealbreaker in this pocket
const C1_R: f64 = 14.3; // carrier-1 plate radius (backs the r11.7 axle seats)
const C2_R: f64 = 14.5; // carrier-2 plate radius (backs the r11.85 axle seats)
// Hub-screw pilot floor (2026-07-19 validation fix): the 2× M3×12 csk hub screws
// seat head-flush at FACE_Z, so their tips reach FACE_Z − 12 = 22.5. The shared
// 8.5 mm `pilot` helper, drilled from the hex top (LIP_Z + 4 = 34.1), floors at
// z 26.1 — the screw would BOTTOM 3.6 mm early in solid PETG and the hub could
// never clamp (the bite gate measures overlap VOLUME and cannot tell annular
// thread bite from full-diameter bottoming; a core-clearance gate now pins it).
// The pilots are drilled 0.3 past the tip; 6.4 mm of ligament remains above the
// plate bottom C2_Z0 = 15.8.
const HUB_PILOT_FLOOR: f64 = 22.2;
const SEG: usize = 64;
const SEG_S: usize = 32;
const PLA: f64 = 0.00124;

const _: () = assert!(LIP_Z == B1_Z + B6804_W, "retainer lip lands on the race top");
const _: () = assert!(SHAFT_BORE_D < REG_D - 2.0, "shaft bore preserves the register shoulder");
const _: () = assert!(HUB_PILOT_FLOOR <= FACE_Z - 12.0 - 0.2, "hub pilot must clear the M3x12 tip by >=0.2");
const _: () = assert!(HUB_PILOT_FLOOR >= C2_Z0 + 5.0, "hub pilot keeps >=5 mm ligament over the carrier plate bottom");

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
fn pilot(s: &Solid, at: DVec3, into: DVec3) -> Solid {
	difference(s, &cylinder(at - into * 0.5, into, 1.25, 8.5, 16))
}
fn bore(s: &Solid, face: DVec3, axis: DVec3, d: f64, len: f64, seg: usize) -> Solid {
	difference(s, &cylinder(face - axis * 0.5, axis, d * 0.5, len + 0.5, seg))
}
/// External spur outline (sun/planet), backlash-thinned + profile-shifted,
/// single source of truth (x = 0 reproduces the unshifted outline byte-for-byte).
fn spur(m: f64, z: usize, x: f64) -> Vec<DVec2> {
	ccw(involute_ring_outline_shifted(m, z, PA, true, false, LASH, x).expect("spur outline"))
}
/// Internal ring cavity outline (the "air tooth"), backlash-widened + shifted.
fn ring_cavity(m: f64, z: usize, x: f64) -> Vec<DVec2> {
	ccw(involute_ring_outline_shifted(m, z, PA, false, false, LASH, x).expect("ring outline"))
}
/// Lewis form factor Y measured from the generator's OWN tooth outline (σ =
/// W_t/(fw·m·Y)): tip-loaded cantilever, critical section = the max of 6·h/t²
/// scanned over the densified tooth boundary (h = radial arm from the tip,
/// t = chordal tooth thickness). Honest for THIS geometry — sharp un-filleted
/// root land and radial flank feet below the base circle — which is WEAKER than
/// the hobbed-tooth table values (~0.36 for 18T) that rate a trochoid-rooted
/// tooth. Internal (`external = false`) measures the ring tooth: the material
/// between two adjacent cavity "air teeth", tip-loaded at the ring tip circle.
fn lewis_y(m: f64, z: usize, external: bool, x: f64) -> f64 {
	let pts = involute_ring_outline_shifted(m, z, PA, external, false, LASH, x).expect("lewis outline");
	let pitch = TAU / z as f64;
	let ra = pts.iter().map(|p| p.length()).fold(0.0f64, f64::max); // external tip / ring root
	let r_tip = pts.iter().map(|p| p.length()).fold(f64::INFINITY, f64::min); // ring tooth tip circle
	let mut worst = 0.0f64; // max σ per unit (W_t / fw)
	for w in 0..pts.len() {
		let (a, b) = (pts[w], pts[(w + 1) % pts.len()]);
		for j in 0..8 {
			let p = a + (b - a) * (j as f64 / 8.0);
			let r = p.length();
			let th = p.y.atan2(p.x).abs();
			if th > pitch * 0.5 {
				// outside tooth 0's half-pitch window: neighbour teeth (whose small
				// sin θ would fake a razor-thin section) — not part of this tooth
				continue;
			}
			let (t, h) = if external {
				// tooth 0 is centred on +X: chordal thickness + radial arm from the tip
				(2.0 * r * th.sin(), ra - r * th.cos())
			} else {
				// ring tooth thickness = the angular gap between adjacent air teeth
				(2.0 * r * (pitch * 0.5 - th).max(0.0).sin(), r - r_tip)
			};
			if t > 1e-9 && h > 1e-9 {
				worst = worst.max(6.0 * h / (t * t));
			}
		}
	}
	1.0 / (m * worst)
}

// ---- printed parts -------------------------------------------------------------------

/// Housing = the two FIXED rings + NEMA square: ring1 (63T m0.6) teeth
/// BACK_T..9.7, a smooth mid bore, ring2 (48T m0.79) teeth 11.5..15.1, a
/// smooth top bore for the output boss, register, 4× M3×30 through-bolt
/// passages, gabled wire exit. Both ring gears print straight into the wall.
/// Prints as used (motor face down, open top up).
fn housing() -> Solid {
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
	// ring1: internal teeth cut from BACK_T up (opens the stage-1 cavity too)
	h = difference(&h, &extrude(&ring_cavity(M1, R1_T, 0.0), R1_TEETH1 - BACK_T).transformed(tr(0.0, 0.0, BACK_T)));
	// ring2: internal teeth (opens the stage-2 cavity)
	h = difference(&h, &extrude(&ring_cavity(M2, R2_T, -X2), R2_TEETH1 - R2_TEETH0).transformed(tr(0.0, 0.0, R2_TEETH0)));
	// smooth bore between the ring bands (r < both ring tips → cannot nick teeth)
	h = difference(&h, &cylinder(v(0.0, 0.0, R1_TEETH1 - 0.05), DVec3::Z, GAP_BORE_R, R2_TEETH0 - R1_TEETH1 + 0.1, SEG));
	// smooth bore above ring2 for the output boss (0.2 into the ring2 zone → no coincident plane)
	h = difference(&h, &cylinder(v(0.0, 0.0, R2_TEETH1 - 0.2), DVec3::Z, TOP_BORE_R, RING_TOP - R2_TEETH1 + 1.2, SEG));
	// NEMA register + shaft clearance + 46° funnel over the recess ceiling
	h = difference(&h, &cylinder(v(0.0, 0.0, -0.5), DVec3::Z, REG_D * 0.5, REG_T + 0.5, SEG));
	h = bore(&h, v(0.0, 0.0, BACK_T), -DVec3::Z, SHAFT_BORE_D, BACK_T + 2.0, SEG);
	h = difference(&h, &cone(v(0.0, 0.0, REG_T - 0.2), DVec3::Z, REG_D * 0.5 + 0.3, 12.3, SEG));
	for (dx, dy) in [(BOLT_SQ, BOLT_SQ), (-BOLT_SQ, BOLT_SQ), (BOLT_SQ, -BOLT_SQ), (-BOLT_SQ, -BOLT_SQ)] {
		h = bore(&h, v(dx, dy, RING_TOP), -DVec3::Z, 3.4, RING_TOP + 2.0, 16);
	}
	h = teardrop_hole(&h, v(0.0, -(h2 + 0.5), 9.0), DVec3::Y, DVec3::Z, 7.0, 6.0, 46.0, None).expect("wire exit");
	h
}

/// Stage-1 sun: 15T (m0.6) with the Ø5 D-bore and a plain Ø8 base collar that
/// rests on the housing floor — axially captive, zero fasteners, D-bore
/// driven. Prints base-down.
fn sun1() -> Solid {
	let mut s = extrude(&spur(M1, S1_T, 0.0), S1_Z1 - S1_Z0).transformed(tr(0.0, 0.0, S1_Z0));
	s = union(&s, &cylinder(v(0.0, 0.0, BACK_T + 0.2), DVec3::Z, 4.0, S1_Z0 - BACK_T - 0.2 + 0.1, SEG_S));
	let mut dbore = cylinder(v(0.0, 0.0, BACK_T - 0.5), DVec3::Z, 2.55, S1_Z1 - BACK_T + 1.0, SEG_S);
	dbore = difference(&dbore, &cuboid(v(2.0, -3.0, BACK_T - 1.0), v(4.0, 3.0, S1_Z1 + 1.0)));
	s = difference(&s, &dbore);
	s
}

/// Stage-1 planet: 24T (m0.6) with a Ø3.3 running bore that rides the M3 steel
/// journal axle (shared kit hardware). Print THREE, same orientation. Prints flat.
fn planet1() -> Solid {
	let mut p = extrude(&spur(M1, P1_T, 0.0), S1_Z1 - S1_Z0).transformed(tr(0.0, 0.0, S1_Z0));
	p = bore(&p, v(0.0, 0.0, S1_Z1 + 1.0), -DVec3::Z, AXLE_BORE_D, S1_Z1 - S1_Z0 + 2.0, SEG_S);
	p
}

/// Stage-1 carrier + stage-2 sun, ONE part (the compound coupling): a plate
/// with three Ø2.9 M3-axle SEATS (the steel journals press in and hang DOWN
/// into the stage-1 planets — were integral Ø5 posts), the stage-2 sun (12T
/// m0.79) rising from its centre, and a Ø5.2 journal bore that rides the motor
/// shaft (which passes on up to the output boss). The sun2 is clocked its
/// install half-pitch π/S2. Rotates at ω/5.2. Prints FLIPPED (sun2 down on the
/// bed, plate as a short bridge; the axle seats are plain vertical bores).
fn carrier1_sun2() -> Solid {
	let mut c = cylinder(v(0.0, 0.0, C1_Z0), DVec3::Z, C1_R, C1_Z1 - C1_Z0, SEG);
	// the sun2 gear rises straight off the plate top (started 0.1 INTO the plate so
	// the union overlaps in volume — no coincident base plane, which validate rejects)
	let sun2 = extrude(&spur(M2, S2_T, X2), S2_Z1 - S2_Z0 + 0.1).transformed(rotz(PI / S2_T as f64) * tr(0.0, 0.0, S2_Z0 - 0.1));
	c = union(&c, &sun2);
	// three M3×5 axle SEATS (light press bores through the plate) — the steel
	// journal presses UP into each and hangs DOWN into the planet1 running bore
	for k in 0..N_PL {
		let a = TAU * k as f64 / N_PL as f64;
		c = difference(&c, &cylinder(v(CD1 * a.cos(), CD1 * a.sin(), C1_Z0 - 0.5), DVec3::Z, AXLE_SEAT_D * 0.5, C1_Z1 - C1_Z0 + 0.6, 24));
	}
	// journal/clearance bore for the motor shaft, plate top through the sun2 top
	c = difference(&c, &cylinder(v(0.0, 0.0, C1_Z0 - 0.5), DVec3::Z, SHAFT_D * 0.5 + J_CLR, S2_Z1 - C1_Z0 + 1.5, SEG_S));
	c
}

/// Stage-2 planet: 18T (m0.79) with a Ø3.3 running bore that rides the M3 steel
/// journal axle (shared kit hardware). Print THREE, same orientation. Prints flat.
fn planet2() -> Solid {
	let mut p = extrude(&spur(M2, P2_T, -X2), S2_Z1 - S2P_Z0).transformed(tr(0.0, 0.0, S2P_Z0));
	p = bore(&p, v(0.0, 0.0, S2_Z1 + 1.0), -DVec3::Z, AXLE_BORE_D, S2_Z1 - S2P_Z0 + 2.0, SEG_S);
	p
}

/// Stage-2 carrier = the OUTPUT: a plate with three Ø2.9 M3-axle seats (the
/// steel journals hang DOWN into the stage-2 planets — were integral Ø5 posts),
/// and the shared output boss on top — Ø24 inner-race shoulder, Ø20 spigot on
/// the 6804, hex torque register, hub pilots, Ø7 shaft clearance. Prints FLIPPED
/// (spigot down as a pedestal; the axle seats are plain vertical bores).
fn carrier2_output() -> Solid {
	// the output boss (spigot / shoulder / hex seat), grafted onto the carrier plate
	let mut r = revolve(
		&[
			DVec2::new(0.05, C2_Z0),
			DVec2::new(C2_R, C2_Z0),
			DVec2::new(C2_R, C2_Z1),
			DVec2::new(12.0, C2_Z1), // horizontal step (not a shallow cone — a cone prints steep when flipped)
			DVec2::new(12.0, B1_Z),  // Ø24 shoulder: lower inner race clamps here
			DVec2::new(SPIG_R, B1_Z),
			DVec2::new(SPIG_R, LIP_Z), // Ø20 spigot rides the 6804 bore
			DVec2::new(0.05, LIP_Z),
		],
		SEG,
	);
	// three M3×5 axle SEATS — BLIND light-press bores, exactly AXLE_SEAT_H deep:
	// the steel journal presses in until it bottoms on the hole floor, which
	// self-locates the 3.4 mm journal below the plate (with the thickened plate a
	// through-bore would let the axle over-seat and starve the planet journal);
	// the Ø24 inner-race shoulder above stays untouched
	for k in 0..N_PL {
		let a = TAU * k as f64 / N_PL as f64;
		r = difference(&r, &cylinder(v(CD2 * a.cos(), CD2 * a.sin(), C2_Z0 - 0.5), DVec3::Z, AXLE_SEAT_D * 0.5, AXLE_SEAT_H + 0.5, 24));
	}
	// hex torque register on the spigot top
	let hexp: Vec<DVec2> = (0..6)
		.map(|k| {
			let a = TAU * k as f64 / 6.0 + PI / 6.0;
			let rr = HEX_AF * 0.5 / (PI / 6.0).cos();
			DVec2::new(rr * a.cos(), rr * a.sin())
		})
		.collect();
	r = union(&r, &extrude(&ccw(hexp), 4.0).transformed(tr(0.0, 0.0, LIP_Z)));
	// Ø7 clearance up the centre for the motor shaft (tip at z≈24 lives inside)
	r = bore(&r, v(0.0, 0.0, LIP_Z + 4.0), -DVec3::Z, 7.0, LIP_Z + 4.0 - C2_Z0 + 1.0, SEG_S);
	// hub-screw pilots, drilled DEEPER than the shared 8.5 mm `pilot` helper so
	// the full M3×12 seats: floor at HUB_PILOT_FLOOR (0.3 past the screw tip),
	// mouth 0.5 above the hex top — see the HUB_PILOT_FLOOR comment for the
	// bottoming defect this fixes (2026-07-19).
	for dx in [4.0f64, -4.0] {
		r = difference(&r, &cylinder(v(dx, 0.0, HUB_PILOT_FLOOR), DVec3::Z, 1.25, LIP_Z + 4.0 + 0.5 - HUB_PILOT_FLOOR, 16));
	}
	r
}

/// Lid — byte-identical to cyclo26/harmonic26 (no dowel sockets needed).
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
	for (dx, dy) in [(BOLT_SQ, BOLT_SQ), (-BOLT_SQ, BOLT_SQ), (BOLT_SQ, -BOLT_SQ), (-BOLT_SQ, -BOLT_SQ)] {
		l = bore(&l, v(dx, dy, LID_TOP), -DVec3::Z, 3.4, LID_TOP - RING_TOP + 2.0, 16);
		l = difference(&l, &cylinder(v(dx, dy, LID_TOP - SANDWICH_CB), DVec3::Z, 3.25, SANDWICH_CB + 1.0, 16));
	}
	l
}

/// Top retainer — byte-identical part to cyclo26/harmonic26.
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

/// Output hub — byte-identical part to cyclo26/harmonic26.
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

fn emit(name: &str, s: &Solid, to_print: DAffine3) -> bool {
	let val = validate(s);
	let mut printed = s.transformed(to_print);
	let zmin = tessellate_default(&printed).positions.iter().map(|q| q.z as f64).fold(f64::INFINITY, f64::min);
	printed = printed.transformed(tr(0.0, 0.0, -zmin));
	let mesh = tessellate_default(&printed);
	let rep = mesh.support_free_report(Vec3::Z, 45.0, 0.3);
	let wt = mesh.is_watertight();
	let grams = volume(s).abs() * PLA;
	let ok = val.is_valid() && wt && rep.steep_area < 1e-6 && rep.max_bridge_span <= 12.0;
	let _ = std::fs::write(format!("planetary26/parts/{name}.stl"), mesh.to_stl_binary());
	println!(
		"  {name:20} valid={:5} wt={wt:5} {}  {grams:4.0}g  {}",
		val.is_valid(),
		if rep.steep_area < 1e-6 {
			format!("sf br≤{:4.1}", rep.max_bridge_span)
		} else {
			format!("steep {:.1}mm²", rep.steep_area)
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
	let mut ok = true;
	let _ = std::fs::create_dir_all("planetary26/parts");
	if let Ok(dir) = std::fs::read_dir("planetary26/parts") {
		for e in dir.flatten() {
			let _ = std::fs::remove_file(e.path());
		}
	}

	// ---- exact ratio, cross-checked through the kinematics module ----
	let e1 = EpicyclicTrain { sun_teeth: S1_T, ring1_teeth: R1_T, planet_a_teeth: P1_T, planet_b_teeth: P1_T, ring2_teeth: R1_T, n_planets: N_PL };
	let e2 = EpicyclicTrain { sun_teeth: S2_T, ring1_teeth: R2_T, planet_a_teeth: P2_T, planet_b_teeth: P2_T, ring2_teeth: R2_T, n_planets: N_PL };
	// each stage is a SIMPLE planetary (sun in, carrier out, ring fixed); its ratio
	// is EpicyclicTrain::simple_ratio (1 + R/S). The Wolfrom `poses` of these trains
	// give the carrier/planet/sun poses of a simple stage exactly (a Wolfrom's first
	// stage IS a simple planetary — same carrier/planet/sun formulas); the ring2/
	// planet_b fields are unused because the output is the CARRIER, not a ring.
	let r1 = EpicyclicTrain::simple_ratio(S1_T, R1_T); // 5.2
	let r2 = EpicyclicTrain::simple_ratio(S2_T, R2_T); // 5.0
	let ratio = r1 * r2;
	let ratio_ok = (ratio - 26.0).abs() < 1e-9 && (S1_T + R1_T) * (S2_T + R2_T) == 26 * S1_T * S2_T;
	let asm_ok = e1.validate_assembly().is_ok() && e2.validate_assembly().is_ok();
	ok &= ratio_ok && asm_ok;
	println!(
		"PLAN-26 CRICKET-CLASS — BACKDRIVABLE 2-stage involute planetary {ratio:.4}:1 for NEMA-17:\n  \
		 stage-1 (in)  sun {S1_T}T / planet {P1_T}T ×{N_PL} / ring {R1_T}T fixed  m{M1}  = {r1:.3}\n  \
		 stage-2 (out) sun {S2_T}T / planet {P2_T}T ×{N_PL} / ring {R2_T}T fixed  m{M2}  = {r2:.3}\n  \
		 EXACT: (S1+R1)(S2+R2) = {}·{} = {} = 26·S1·S2  (rational, no float)  body {NEMA_W}×{NEMA_W} sq × {LID_TOP:.0} + motor\n",
		S1_T + R1_T,
		S2_T + R2_T,
		(S1_T + R1_T) * (S2_T + R2_T)
	);

	// ---- A-GEOM: rims, ring walls, undercut, M3 axle, honest tooth ratings ----
	// rims/walls (all machine-derived from the module + tooth count + profile shift:
	// +X2 raises the sun2 root radius, −X2 lowers the planet2 root / pulls the ring2
	// root inward — the shifted terms are ±X2·M2, zero for the unshifted stage 1)
	let sun1_rim = M1 * S1_T as f64 / 2.0 - 1.25 * M1 - 2.55; // over the Ø5.1 D-bore round part
	let sun2_rim = M2 * S2_T as f64 / 2.0 - (1.25 - X2) * M2 - (SHAFT_D * 0.5 + J_CLR); // over the Ø5.2 shaft bore
	let pl1_root_r = M1 * P1_T as f64 / 2.0 - 1.25 * M1; // 6.45 planet-1 tooth-root radius
	let pl2_root_r = M2 * P2_T as f64 / 2.0 - (1.25 + X2) * M2; // 6.01 planet-2 tooth-root radius (−X2 shift)
	let pl1_rim = pl1_root_r - AXLE_BORE_D * 0.5; // rim over the Ø3.3 M3-journal bore
	let pl2_rim = pl2_root_r - AXLE_BORE_D * 0.5;
	let ring1_wall = NEMA_W * 0.5 - (M1 * R1_T as f64 / 2.0 + 1.25 * M1);
	let ring2_wall = NEMA_W * 0.5 - (M2 * R2_T as f64 / 2.0 + (1.25 - X2) * M2); // −X2 ring shift GROWS the wall
	let fw1 = S1_Z1 - S1_Z0; // 3.6 stage-1 planet face width
	let fw2 = S2_Z1 - S2P_Z0; // 4.0 stage-2 planet face width (v2: was 3.4)
	// --- shared-hardware FIT LEDGER (measured, not assumed): the unification
	// decision. PREFERRED was a 693ZZ (Ø8×4, harmonic's shared roller) pressed
	// into each planet. Its RIM over the tooth root passes the 1.5 mm floor, but
	// its 4.0 mm WIDTH exceeds the planet faces AND the inter-carrier pocket, so
	// it cannot seat axially. M3×8/M3×12 shanks (8/12 mm) overrun the axle pocket.
	// Only the M3×5 (5 mm) fits — the planet rides its bare steel Ø3 body. ---
	let b693_rim1 = pl1_root_r - B693_OD * 0.5; // 2.45  (≥1.5 → 693 rim IS fine)
	let b693_rim2 = pl2_root_r - B693_OD * 0.5; // ~2.0 with the −x2 planet shift
	let pocket2 = C2_Z0 - C1_Z1; // 4.4 stage-2 inter-carrier axial pocket (band raised in v2)
	let b693_rim_ok = b693_rim1 >= 1.5 && b693_rim2 >= 1.5; // true — rim is not the problem
	// per-stage width fit, re-measured after the v2 face-width change: stage-1 (fw 3.6)
	// still CANNOT seat the 4.0-wide 693ZZ; stage-2 (fw 4.0, pocket 4.4) now CAN. The
	// drive KEEPS the one-journal-design M3×5 landing (no new hardware, both stages
	// identical); the stage-2 693 option is recorded as an OPEN friction upgrade.
	let b693_w1_fits = B693_W <= fw1 + 1e-9;
	let b693_w2_fits = B693_W <= fw2 + 1e-9 && B693_W <= pocket2 + 1e-9;
	let axle_pocket1 = C1_Z0 - BACK_T; // 4.3 stage-1 axle pocket (carrier bottom → housing floor)
	let axle_pocket2 = C2_Z0 - C1_Z1; // 4.4 stage-2 axle pocket (carrier2 → carrier1)
	// the axle length must fit in its pocket plus the seat depth it buries into
	let m3x5_fits = AXLE_LEN <= axle_pocket1 + AXLE_SEAT_H + 1e-9 && AXLE_LEN <= axle_pocket2 + AXLE_SEAT_H + 1e-9;
	// undercut floor at PA, SHIFT-AWARE: z_min = 2·(1 − x)/sin²α — a positive shift
	// lowers the floor for the 12T sun, the −X2 planet2 must clear the RAISED floor
	let undercut_floor = |x: f64| 2.0 * (1.0 - x) / (PA.to_radians().sin()).powi(2);
	let undercut_ok = (S1_T as f64) >= undercut_floor(0.0)
		&& (P1_T as f64) >= undercut_floor(0.0)
		&& (S2_T as f64) >= undercut_floor(X2)
		&& (P2_T as f64) >= undercut_floor(-X2);
	// mesh tip/root clearance (planet tip must not bottom the ring root) — the
	// stage-2 terms carry the shifts explicitly; equal shifts cancel, so clr2 is
	// x-invariant (ring root −X2·M2, planet tip −X2·M2)
	let clr1 = (M1 * R1_T as f64 / 2.0 + 1.25 * M1) - (CD1 + M1 * P1_T as f64 / 2.0 + M1);
	let clr2 = (M2 * R2_T as f64 / 2.0 + (1.25 - X2) * M2) - (CD2 + M2 * P2_T as f64 / 2.0 + (1.0 - X2) * M2);
	// honest tooth rating at the OUTPUT (loaded) stage: Lewis bending with Y
	// MEASURED from the generator's own outlines (lewis_y — sharp un-filleted
	// root, radial flank feet; genuinely lower than hobbed-tooth table values).
	// Weakest of sun2(+X2) / planet2(−X2) / ring2(−X2) governs; X2 was chosen to
	// balance sun and planet. n = 3 planets, balanced share (K-factors live in
	// the v2 receipts, not here).
	let y2s = lewis_y(M2, S2_T, true, X2);
	let y2p = lewis_y(M2, P2_T, true, -X2);
	let y2r = lewis_y(M2, R2_T, false, -X2);
	let y2 = y2s.min(y2p).min(y2r);
	let f_cont = 20.0 * fw2 * M2 * y2; // 20 MPa fatigue, per tooth
	let f_peak = 50.0 * fw2 * M2 * y2; // 50 MPa yield, per tooth
	let t_cont = 6.0 * f_cont * CD2 / 1000.0; // T_carrier = 6·F_t·CD (n=3 planets, balanced)
	let t_peak = 6.0 * f_peak * CD2 / 1000.0;
	// M3 STEEL journal axle: cantilevered, load 2·F_peak over the 3.4 mm journal.
	// Steel (≥200 MPa allowable) — a huge margin the old Ø5 PETG post never had.
	let axle_sigma = 2.0 * f_peak * (AXLE_JOURNAL * 0.5) * 32.0 / (PI * AXLE_D.powi(3));
	let geom_ok = sun1_rim >= 1.1
		&& sun2_rim >= 1.1
		&& pl1_rim >= 1.5
		&& pl2_rim >= 1.5
		&& ring1_wall >= 1.15
		&& ring2_wall >= 1.15
		&& undercut_ok
		&& clr1 >= 0.05
		&& clr2 >= 0.05
		&& t_peak >= 1.6
		&& axle_sigma <= 200.0
		&& m3x5_fits; // the chosen M3×5 journal axle fits the pocket
	ok &= geom_ok;
	println!(
		"A-GEOM: sun rim {sun1_rim:.2}/{sun2_rim:.2} ≥1.1 · planet rim {pl1_rim:.2}/{pl2_rim:.2} ≥1.5 (over the Ø{AXLE_BORE_D} M3 bore) · fixed-ring wall {ring1_wall:.2}/{ring2_wall:.2} ≥1.15 · shifted undercut floors {:.1}/{:.1}T {} · mesh clr {clr1:.2}/{clr2:.2} · stage-2 x ±{X2} measured Lewis Y {y2s:.3}/{y2p:.3}/{y2r:.3} (sun/planet/ring, sharp-root outline — NOT table 0.36) → PETG output-tooth rating {t_cont:.1} N·m cont / {t_peak:.1} peak (balanced share) · M3 steel axle {axle_sigma:.0} ≤200 MPa  {}",
		undercut_floor(X2),
		undercut_floor(-X2),
		if undercut_ok { "OK" } else { "<<< FAIL" },
		if geom_ok { "OK" } else { "<<< FAIL" }
	);
	println!(
		"  693-vs-M3 fit ledger: 693ZZ rim {b693_rim1:.2}/{b693_rim2:.2} ≥1.5 {} · 693 WIDTH {B693_W:.1} vs stage-1 face {fw1:.1} → {} / stage-2 face {fw2:.1} pocket {pocket2:.1} → {} · M3×5 ({AXLE_LEN:.0} mm) fits the {axle_pocket1:.1}/{axle_pocket2:.1} axle pocket {} → M3 steel journal KEPT on both stages (one journal design, no new hardware; stage-2 693 recorded as an open friction upgrade)",
		if b693_rim_ok { "PASS" } else { "FAIL" },
		if b693_w1_fits { "fits" } else { "STILL FAILS" },
		if b693_w2_fits { "now fits (v2 band)" } else { "FAILS" },
		if m3x5_fits { "OK" } else { "<<< FAIL" }
	);

	// ---- printed parts ----
	let house = housing();
	let sun1_p = sun1();
	let planet1_p = planet1();
	let carrier1 = carrier1_sun2();
	let planet2_p = planet2();
	let carrier2 = carrier2_output();
	let lid_p = lid();
	let retainer = retainer_ring();
	let hub = output_hub();
	let flip = DAffine3::from_rotation_x(PI);
	println!("\nprintable parts (planetary26/parts is a pure print queue):");
	for (name, s, m) in [
		("housing_rings", &house, DAffine3::IDENTITY),
		("sun1_15t", &sun1_p, DAffine3::IDENTITY),
		("planet1_24t_x3", &planet1_p, DAffine3::IDENTITY),
		("carrier1_sun2", &carrier1, flip),
		("planet2_18t_x3", &planet2_p, DAffine3::IDENTITY),
		("carrier2_output", &carrier2, flip),
		("lid_ring", &lid_p, flip),
		("retainer_ring", &retainer, DAffine3::IDENTITY),
		("output_hub", &hub, DAffine3::IDENTITY),
	] {
		ok &= emit(name, s, m);
	}

	// ---- assembly (θ = 0 install pose from the kinematics module) ----
	let motor = nema_motor(17, 48.0).expect("nema17");
	let b6804 = deep_groove_bearing("6804").expect("6804");
	let m3x30 = button_head_screw(3.0, SANDWICH_L).expect("m3x30");
	let m3x8 = button_head_screw(3.0, 8.0).expect("m3x8");
	let m3x12f = flat_head_screw(3.0, 12.0).expect("m3x12 csk");
	let m3x5 = set_screw(3.0, 5.0).expect("m3x5 din916"); // planet journal axle — shared with the harmonic wave-gen screw
	let p1 = e1.poses(0.0);
	let p2 = e2.poses(0.0);

	let mut instances: Vec<(String, Solid, DAffine3)> = Vec::new();
	let place = |list: &mut Vec<(String, Solid, DAffine3)>, n: &str, s: &Solid, x: DAffine3| {
		list.push((n.to_string(), s.clone(), x));
	};
	place(&mut instances, "hw_nema17", &motor, tr(0.0, 0.0, 0.0));
	place(&mut instances, "housing_rings", &house, tr(0.0, 0.0, 0.0));
	place(&mut instances, "sun1_15t", &sun1_p, rotz(p1.sun_install_phase)); // the input carries the half-pitch phase
	place(&mut instances, "carrier1_sun2", &carrier1, rotz(p1.carrier)); // ω/5.2 — at θ=0, phase 0 (sun2 is clocked inside the part)
	for j in 0..N_PL {
		let pp = p1.planets[j];
		place(&mut instances, "planet1_24t", &planet1_p, tr(CD1 * pp.azimuth.cos(), CD1 * pp.azimuth.sin(), 0.0) * rotz(pp.spin));
	}
	// stage-1 M3×5 journal axles: pressed into carrier1, body spans z 6.4..11.4
	// (3.4 mm journals planet1, 1.6 mm seated in the carrier plate)
	for j in 0..N_PL {
		let pp = p1.planets[j];
		place(&mut instances, "hw_m3x5_axle", &m3x5, tr(CD1 * pp.azimuth.cos(), CD1 * pp.azimuth.sin(), C1_Z0 - AXLE_JOURNAL));
	}
	place(&mut instances, "carrier2_output", &carrier2, rotz(p2.carrier)); // output ω/26 — phase 0 at θ=0
	for j in 0..N_PL {
		let pp = p2.planets[j];
		place(&mut instances, "planet2_18t", &planet2_p, tr(CD2 * pp.azimuth.cos(), CD2 * pp.azimuth.sin(), 0.0) * rotz(pp.spin));
	}
	// stage-2 M3×5 journal axles: pressed into carrier2, body spans z 12.4..17.4
	for j in 0..N_PL {
		let pp = p2.planets[j];
		place(&mut instances, "hw_m3x5_axle", &m3x5, tr(CD2 * pp.azimuth.cos(), CD2 * pp.azimuth.sin(), C2_Z0 - AXLE_JOURNAL));
	}
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
		place(&mut instances, "hw_m3x12_hub", &m3x12f, tr(dx, 0.0, FACE_Z - 12.0));
	}

	let mut asm = Mesh::default();
	for (_, s, x) in &instances {
		merge_into(&mut asm, &tessellate_default(&s.transformed(*x)));
	}
	let _ = asm.write_stl_binary("planetary26/ASSEMBLY.stl");
	let mut expl = Mesh::default();
	for (n, s, x) in &instances {
		let lift = match n.as_str() {
			"hw_nema17" => 0.0,
			"housing_rings" => 26.0,
			"sun1_15t" => 55.0,
			"planet1_24t" => 80.0,
			"hw_m3x5_axle" => 95.0,
			"carrier1_sun2" => 110.0,
			"planet2_18t" => 140.0,
			"carrier2_output" => 175.0,
			"lid_ring" => 215.0,
			"hw_bearing_6804" => 245.0,
			"retainer_ring" => 268.0,
			"hw_m3x8_retainer" => 290.0,
			"output_hub" => 312.0,
			"hw_m3x12_hub" => 335.0,
			"hw_m3x30_sandwich" => 358.0,
			_ => 0.0,
		};
		merge_into(&mut expl, &tessellate_default(&s.transformed(tr(0.0, 0.0, lift) * *x)));
	}
	let _ = expl.write_stl_binary("planetary26/ASSEMBLY_EXPLODED.stl");

	// ---- A-ASM: every interface measured on the exact poses ----
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
		let good = if expect_contact { d < 0.06 } else { d >= 0.10 };
		*ok &= good;
		println!("  {label:48} min_dist={d:7.3}  {}", if good { "OK" } else { "<<< FAIL" });
	};
	println!();
	let house_m = mesh_of("housing_rings", 0);
	let lid_m = mesh_of("lid_ring", 0);
	let hub_m = mesh_of("output_hub", 0);
	let ret_m = mesh_of("retainer_ring", 0);
	let ob1 = mesh_of("hw_bearing_6804", 0);
	let motor_m = mesh_of("hw_nema17", 0);
	let c2_m = mesh_of("carrier2_output", 0);
	rel("motor pilot seats in the register", &motor_m, &house_m, true, &mut ok);
	rel("lid seats on the ring wall", &lid_m, &house_m, true, &mut ok);
	rel("6804 inner seats on the output shoulder", &ob1, &c2_m, true, &mut ok);
	rel("shelf carries the outer race", &ob1, &lid_m, true, &mut ok);
	rel("retainer lip lands on the outer race", &ob1, &ret_m, true, &mut ok);
	rel("retainer seats on the lid top", &ret_m, &lid_m, true, &mut ok);
	rel("hub clamps the inner race", &hub_m, &ob1, true, &mut ok);
	// gear meshes: near-contact by design (engaged, never jammed — precise
	// non-interference is gated in A-PROD and the sim)
	let sun1_m = mesh_of("sun1_15t", 0);
	let c1_m = mesh_of("carrier1_sun2", 0);
	let pl1_0 = mesh_of("planet1_24t", 0);
	let pl2_0 = mesh_of("planet2_18t", 0);
	for (lbl, a, b) in [
		("sun1 meshes planet1 (running contact)", &sun1_m, &pl1_0),
		("planet1 meshes the fixed ring 63", &pl1_0, &house_m),
		("sun2 meshes planet2 (running contact)", &c1_m, &pl2_0),
		("planet2 meshes the fixed ring 48", &pl2_0, &house_m),
	] {
		let d = a.min_distance(b);
		let good = d < 0.25;
		ok &= good;
		println!("  {lbl:48} min_dist={d:7.3}  {}", if good { "OK" } else { "<<< FAIL" });
	}
	// planets ride their M3×5 STEEL journal axles (metal-on-PETG, low friction) —
	// each planet bore is coaxial with its axle at a Ø0.15 running clearance
	let axle1_m = mesh_of("hw_m3x5_axle", 0); // stage-1 axle[0], coaxial with planet1[0]
	let axle2_m = mesh_of("hw_m3x5_axle", N_PL); // stage-2 axle[0], coaxial with planet2[0]
	let j1 = pl1_0.min_distance(&axle1_m);
	let j2 = pl2_0.min_distance(&axle2_m);
	let jfit_ok = j1 < 0.20 && j2 < 0.20;
	ok &= jfit_ok;
	println!("  planets ride their M3 steel journal axles ({j1:.3}/{j2:.3} running clearance)  {}", if jfit_ok { "OK" } else { "<<< FAIL" });
	let engage = |label: &str, screw: &Solid, x: DAffine3, part: &Solid, ok: &mut bool| {
		let bite = overlap_mm3(&screw.transformed(x), part);
		let okb = (3.0..=45.0).contains(&bite);
		*ok &= okb;
		println!("  {label:48} bite={bite:6.1} mm³  {}", if okb { "OK" } else { "<<< FAIL" });
	};
	let hub_x = instances.iter().find(|(n, _, _)| n == "hw_m3x12_hub").map(|(_, _, x)| *x).unwrap();
	engage("hub screw threads the output pilot", &m3x12f, hub_x, &carrier2, &mut ok);
	// the hub screw's CORE must never meet solid material — real thread
	// engagement is only the Ø2.5→Ø3 annulus, and the bite VOLUME above cannot
	// distinguish that from a full-diameter tip bottoming in an under-drilled
	// pilot (which leaves the head proud and the hub unclamped — the exact
	// defect found and fixed 2026-07-19). Probe: a Ø2.3 core rod over the full
	// M3×12 shank span (tip FACE_Z−12 up the 10.5 mm shank), both stations.
	let core_ov: f64 = [4.0f64, -4.0]
		.iter()
		.map(|dx| overlap_mm3(&cylinder(v(*dx, 0.0, FACE_Z - 12.0), DVec3::Z, 1.15, 10.5, 16), &carrier2))
		.sum();
	let core_ok = core_ov < 0.05;
	ok &= core_ok;
	println!("  hub screw core seats without bottoming ({core_ov:.3} mm³ core overlap)  {}", if core_ok { "OK" } else { "<<< FAIL" });
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
	println!("  sandwich bolt tap engagement {tap_depth:.2} mm (3.0–4.5 usable NEMA-17 tap)  {}", if tap_ok { "OK" } else { "<<< FAIL" });
	let out_ov = overlap_mm3(&carrier2, &lid_p);
	let hb_ov = overlap_mm3(&hub, &lid_p);
	let spin_ok = out_ov < 0.05 && hb_ov < 0.05;
	ok &= spin_ok;
	println!("  output + hub spin free of the lid ({out_ov:.3}/{hb_ov:.3} mm³)      {}", if spin_ok { "OK" } else { "<<< FAIL" });

	// ---- A-CAPTURE ----
	let up_ov = overlap_mm3(&b6804.transformed(tr(0.0, 0.0, B1_Z + 0.5)), &retainer);
	let dn_ov = overlap_mm3(&b6804.transformed(tr(0.0, 0.0, B1_Z - 0.5)), &lid_p);
	let cap_ok = up_ov > 5.0 && dn_ov > 5.0;
	ok &= cap_ok;
	println!(
		"A-CAPTURE: bearing +0.5 hits the retainer ({up_ov:.2} mm³); −0.5 hits the shelf ({dn_ov:.2} mm³) — held BOTH ways  {}",
		if cap_ok { "OK" } else { "<<< FAIL" }
	);

	// ---- A-PROD matrix ----
	println!("\nA-PROD (pairwise interference matrix, representative bodies):");
	let rep: Vec<(&str, usize)> = vec![
		("hw_nema17", 0),
		("housing_rings", 0),
		("sun1_15t", 0),
		("carrier1_sun2", 0),
		("planet1_24t", 0),
		("planet1_24t", 1),
		("hw_m3x5_axle", 0), // stage-1 planet journal axle
		("planet2_18t", 0),
		("planet2_18t", 1),
		("hw_m3x5_axle", 3), // stage-2 planet journal axle (nth 3 = first stage-2)
		("carrier2_output", 0),
		("lid_ring", 0),
		("hw_bearing_6804", 0),
		("retainer_ring", 0),
		("output_hub", 0),
		("hw_m3x30_sandwich", 0),
		("hw_m3x8_retainer", 0),
		("hw_m3x12_hub", 0),
	];
	const CONTACT_PAIRS: &[(&str, &str, f64)] = &[
		("hw_nema17", "housing_rings", 1.0),
		("sun1_15t", "hw_nema17", 30.0),                 // D-bore on the shaft
		("sun1_15t", "planet1_24t", 2.0),                // running mesh (sim-verified)
		("planet1_24t", "housing_rings", 2.0),           // running mesh (sim-verified)
		("planet1_24t", "carrier1_sun2", 0.5),           // planet clears the carrier (rides the M3 axle)
		("carrier1_sun2", "planet2_18t", 2.0),           // sun2 running mesh (sim-verified)
		("planet2_18t", "housing_rings", 2.0),           // running mesh (sim-verified)
		("planet2_18t", "carrier2_output", 0.5),         // planet clears the carrier (rides the M3 axle)
		("hw_m3x5_axle", "carrier1_sun2", 3.0),          // M3×5 pressed into the carrier-1 seat
		("hw_m3x5_axle", "carrier2_output", 3.0),        // M3×5 pressed into the carrier-2 seat
		("carrier1_sun2", "hw_nema17", 6.0),             // shaft journal in the carrier hub
		("carrier2_output", "hw_nema17", 6.0),           // shaft passes up the Ø7 spigot bore
		("hw_bearing_6804", "carrier2_output", 1.0),
		("hw_bearing_6804", "output_hub", 1.5),
		("carrier2_output", "output_hub", 1.0),          // hex register mate
		("hw_bearing_6804", "retainer_ring", 1.5),
		("hw_bearing_6804", "lid_ring", 1.0),
		("hw_m3x8_retainer", "lid_ring", 25.0),
		("hw_m3x8_retainer", "retainer_ring", 5.0),
		("hw_m3x30_sandwich", "hw_nema17", 30.0),
		("hw_m3x12_hub", "carrier2_output", 25.0),
		("hw_m3x12_hub", "output_hub", 8.0),
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
	match export_step_assembly(&instances, "plan26") {
		Ok(step) => {
			let _ = std::fs::write("planetary26/ASSEMBLY.step", &step);
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

	println!("\nBOM (hardware in the ASSEMBLY only — parts/ is the print queue):");
	println!("  1× NEMA-17 + driver · 1× 6804 · UNIFIED planet hardware: 6× M3×5 DIN916 journal axles (the SAME set screw");
	println!("               the harmonic wave generator uses — swappable across the kit; 693ZZ was too wide, see A-GEOM ledger)");
	println!("  M3 ONLY (14 total): 4×30 sandwich · 6×5 planet axles · 2×8 retainer · 2×12 csk hub");
	println!("  BACKDRIVABLE: designed for it (non-self-locking involute spurs · 5.2 & 5.0 per stage · output on a 6804 · M3 STEEL");
	println!("               planet journals, metal-on-PETG · no preload) — backdrive TORQUE is a friction property, NOT gated; confirm by hand.");
	println!("  Exact tooth-level kinematics: cargo run --example planetary26_sim (exit 1 on FAIL)");

	println!("\nRESULT: {}", if ok { "PASS — every gate green" } else { "FAIL — see <<< lines" });
	std::process::exit(if ok { 0 } else { 1 });
}
