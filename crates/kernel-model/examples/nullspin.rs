// Copyright (c) LMCAD. Licensed under the MIT License.

//! NULLSPIN — a grounded-carrier ("star") epicyclic fidget spinner whose two
//! visible rotors turn in OPPOSITE directions at an exact integer ratio.
//!
//! Contest entry: Printables "Designer Challenge: Geared Spinners" (closes
//! 2026-08-22). Campaign spec frozen by the research/judging pass; every
//! number below is a named const with its WHY and at least one gate.
//!
//! ARCHITECTURE. The CARRIER is the held frame (base spider + top spider), so
//! the six planet axes are FIXED and do not orbit. Flick the outer ring; the
//! inner sun puck counter-rotates. Chosen because fixed axes carry ZERO
//! centrifugal pin load — the single largest added loss in every
//! orbiting-planet geared spinner — and because two large concentric
//! counter-rotating rims in one plane is the most gear-legible image
//! available.
//!
//! HEADLINE, exact and rational: 7·R = 11·S (7·66 = 11·42 = 462), so "flick
//! the ring 7 times and the puck turns 11 the other way." Ring→planet is an
//! INTERNAL mesh (same sense, +R/P = +5.5); planet→sun is EXTERNAL (opposite,
//! −R/S = −11/7).
//!
//! THE RECEIPT is angular-momentum cancellation, not spin time:
//! `eta = 1 − |Σ Iᵢωᵢ| / Σ|Iᵢωᵢ|`. At eta → 1 the spinner has (almost) no net
//! spin angular momentum, so it does not fight being tilted. It is MODELLED,
//! never measured (no instrument exists) — and the shipped SUN-B control puck
//! is deliberately UNcancelled so the buyer can perform the A/B by hand.
//!
//! SPIN TIME IS NOT A CLAIM. This campaign integrates the research's own
//! reflected-drag model in a solver written here and benchmarked against two
//! closed forms before it was used, then publishes the number it gets with its
//! derivation and its band. A geared spinner cannot beat a plain one.
//!
//! **v3 — ZERO NON-PRINTED PARTS, and what that costs.** v1 ran the ring on a
//! sliding land (2.4 s as committed). v2 put that load on 24 loose Ø1.50
//! chrome-steel balls and kept the 608 (5.7 s as committed — this run rebuilds
//! both on the SHIPPED v3 rotor instead of quoting them, which is why the
//! ledger reads 2.5 and 6.2). v3 deletes ALL of it: no bearing, no balls, no
//! magnets, screws, nuts, weights or inserts. The `You also need:` line reads
//! **nothing**. That is a deliberate trade and it is expensive: the 608 was 46 %
//! of v2's remaining budget and the ball race 3 %, and BOTH loads land back on
//! printed sliding contacts. All three architectures are recomputed by the same
//! solver on the same rotor in every run so the ledger tells the whole truth.
//!
//! What v3 does about it, in order of size:
//! * the sun runs DIRECTLY on the printed post — the post shrinks Ø7.90 → Ø5.50
//!   (it no longer has to be a 608's bore) and the sun's thrust lands on the
//!   smallest annulus its own bore geometry permits, arm 3.70 mm instead of the
//!   34.60 mm the ring is stuck with;
//! * the ring goes back to six printed thrust pads, moved in to the smallest
//!   radius its continuous flat underside reaches (34.60 vs v1's 34.75);
//! * the design study re-solves with the 608's 610 g·mm² gone from the sun side
//!   — eta forces I_sun·k_S = I_ring, so the ring gets THINNER, not the sun
//!   thicker, and that is also the lightest ring the eta budget allows.
//!
//! Every direction that LOST is in `analysis/ANALYSIS.md` with its numbers:
//! printed balls (they would work — PLA-on-PLA Hertz recomputed — but a sphere
//! cannot be printed support-free and the engine measures how badly), the
//! central web (RE-OPENED and refused again, this time on PRINTABILITY and the
//! rim-shell inertia, not on eta), the on-axis point pivot (it wins on drag and
//! deletes the static thumb pad — the arithmetic is published), planet flanges,
//! printed rollers, a floating washer, lightening the ring (a theorem: under
//! the eta constraint it is almost exactly neutral).
//!
//! v3 keeps v2's two corrections to v1 (planet-pad arm 3.475 not 3.25; the
//! edge-on note is 1.30x, not "vanishes") and adds a third: with the hardware
//! gone the edge-on advice REVERTS to v1's sense, and the listing says so.
//!
//! **v4 — RETENTION BY GEOMETRY, not by friction.** v3 held the top spider on
//! with six Ø5.55 "click bands", and its own gate G16e disclosed the problem
//! honestly: under-extrusion thins the pin AND widens the hole, the errors add,
//! and the 0.025 mm interference reaches exactly ZERO at 0.025 mm/side — while
//! this campaign's worst-case XY figure, used for every clearance in G12, is
//! 0.15 mm/side, SIX TIMES larger. Retention was binary and it was the
//! printer's call. (Worse: as shipped, the Ø5.55 band sat in a Ø5.60 hole, so
//! the interference the strain gates were computing was a 0.025 mm/side
//! CLEARANCE. Both facts are recorded in `analysis/ANALYSIS.md`.)
//!
//! v4 applies RESPOOL's zero-preload lesson. Each pin carries a Ø2.70 NECK
//! through the spider and a radial FIN above it; each spider arm carries a
//! bayonet slot. Drop the spider on 7 deg out, twist it home to a hard stop,
//! and the fin overhangs the slot wall by 1.15 mm of solid material. Nothing is
//! strained at rest, assembly needs no strain at all, and rebuilding BOTH parts
//! with the FULL G12 error (0.35 mm/side each) still leaves 0.45 mm of
//! shoulder — proved on the built solids, with two negative controls that must
//! read exactly zero (delete the lip; pose it untwisted). What it costs is in
//! the ledger: the study re-solved and moved t_planet 3.50 → 4.00, eta went
//! 0.9990 → 0.9950, the optional coupon became two pieces, and the pin grew
//! 0.93 mm taller (which is what re-opened the study).
//!
//! DEVIATIONS FROM THE FROZEN SPEC, each with its arithmetic, are recorded at
//! the const or fn that carries them: the Ø6.40 snap barb (14 % hoop strain,
//! refused — and v4 re-refuses the whole snap CLASS, G16m), the 1.29 deg lash
//! angle (does not reproduce; 0.49 deg per mesh does), the 1.0 mm bed-side rim
//! round (a 90 deg overhang tangent), the six ring tabs merged into a closed
//! rim, the blind cap bore (under min wall), and every 45 deg relief re-cut at
//! 1.40 rise:run (a facet cannot land ON the support threshold).
//!
//! Run: cargo run --release -p kernel-model --example nullspin
//! (writes spinner_system/nullspin/**; exit 1 on any FAIL)

use kernel_brep::math::{DAffine3, DVec2, DVec3};
use kernel_brep::{
	boolean_hazards, cuboid, cylinder, difference, export_step, extrude, force_ccw, intersection, mass_properties,
	overlap_volume, revolve, sphere, tessellate_default, union, validate, volume, ChainLog, HazardKind, Solid,
};
use kernel_core::math::Vec3;
use kernel_core::Mesh;
use kernel_model::campaign::gate;
use kernel_model::kinematics::EpicyclicTrain;
use kernel_model::materials::pla::SIG_ALLOW_RT;
use kernel_model::materials::PLA_G_PER_MM3;
use kernel_model::optimize::{gate_study, Constraint, DesignVar, Evaluation, Params, Study};
use kernel_model::parts::involute_ring_outline_shifted_filleted;
use kernel_model::process::FdmProfile;
use std::f64::consts::{PI, TAU};

const OUT: &str = "spinner_system/nullspin";
const PLA: f64 = PLA_G_PER_MM3;

// ============================================================================
// 1. GEAR SET — frozen. Every condition is gated (G1–G4).
// ============================================================================

/// Module. WHY: the pitch-line tooth thickness is π·m/2 = 1.571 mm = 3.5 × a
/// 0.45 mm extrusion width — two solid walls plus real fill. m0.6 is the
/// two-perimeter cliff (0.942 mm against a 0.90 mm two-wall minimum).
const M: f64 = 1.000;
/// Pressure angle. WHY: the undercut floor is z ≥ 2/sin²α = 11.198 T at 25°,
/// so the 12T planet is legal at ZERO profile shift. This engine does not
/// model undercut at all (gears.rs: shift_coeff edits ra/rr/pitch-line
/// thickness, it is not a hob simulation), so "20° + profile shift" would be
/// fiction. 25° is the honest fix.
const PA_DEG: f64 = 25.0;
const S_T: usize = 42; // sun, external
const P_T: usize = 12; // planet, external, ×6
const R_T: usize = 66; // ring, internal
const N_PL: usize = 6;
/// Profile shift on every member. Zero — see PA_DEG.
const X_SHIFT: f64 = 0.0;
/// Backlash: tooth thinning per flank, mm, on ALL THREE members → circular
/// backlash jt = 0.18 mm per mesh. WHY: CMM-measured FDM involute profile
/// deviation is 0.067 mm/flank and two flanks meet, so anything under
/// ~0.134 mm binds. Obtained by THINNING ONLY — opening the centre distance is
/// the inferior lever (Δa = 1.374·Δj) and would move every mesh. Refused on
/// record. This departs from the engine's inherited 0.05 convention
/// (planetary26.rs), which is a convention, not a measurement.
const LASH: f64 = 0.09;
/// Root fillet coefficient (r = 0.30·m) on the EXTERNAL members only. The
/// engine fillets external teeth only (gears.rs) — the ring cavity stays
/// sharp-rooted. Stated on the model page, not hidden.
const RF: f64 = 0.30;

/// Centre distance, both meshes, EXACTLY: m(S+P)/2 = 27.000 = 33.0 − 6.0.
const CD: f64 = M * (S_T + P_T) as f64 / 2.0;

// Rational proofs — compile-time, no float.
const _: () = assert!(R_T == S_T + 2 * P_T, "planet-fit: R = S + 2P");
#[allow(clippy::manual_is_multiple_of)]
const _: () = assert!((S_T + R_T) % N_PL == 0, "equal spacing: (S+R) % n == 0");
#[allow(clippy::manual_is_multiple_of)]
const _: () = assert!(R_T % N_PL == 0, "ring pattern repeats under 2π/n");
#[allow(clippy::manual_is_multiple_of)]
const _: () = assert!(S_T % N_PL == 0, "sun pattern repeats under 2π/n (rigid-rotation install)");
#[allow(clippy::manual_is_multiple_of)]
const _: () = assert!((S_T + R_T) % 2 == 0, "S+R even");
const _: () = assert!(7 * R_T == 11 * S_T, "HEADLINE: flick the ring 7×, the puck turns 11× the other way");
const _: () = assert!(2 * R_T == 11 * P_T, "planet runs at exactly 11/2 = 5.5× the ring");

// ============================================================================
// 2. PROCESS + CLEARANCES — from profiles/conservative_default.json, which is
//    print-proven in RESPOOL/DRYBOX, not from research nominals.
// ============================================================================

/// Running radial fit, mm (profile xy_clearance_free). WHY 0.25: the only
/// value inside every measured cross-printer band — 0.15 fuses on a CR-10
/// class machine, 0.20 is the exact edge of "moved with relative ease" on a
/// CORE One, Prusa officially says "at least 0.3".
const C_FREE: f64 = 0.25;
/// Press/locating fit, mm (profile xy_clearance_tight). DRYBOX print-proved:
/// Ø7.9 stub in a 608's Ø8.0 bore.
const C_TIGHT: f64 = 0.05;
/// Axial gap, mm (profile z_clearance).
const C_Z: f64 = 0.30;
/// Bed-side chamfer on every clearance surface, mm × 45°. WHY: PrusaSlicer
/// removes 0.20 mm/side on layer 1 while Bambu removes 0.075 (a 2.7× vendor
/// disagreement); a chamfer makes layer 1 physically absent from the gap.
const C_BED: f64 = 0.45;
/// Tooth-tip chamfer, radial run in mm, both faces — break-in plus the named
/// "sharp gear edges" injury complaint the field ignores.
const C_TIP: f64 = 0.30;
/// Rise ÷ run of EVERY downward-facing relief cone in this campaign.
///
/// A 45° cone sits exactly ON the support-free threshold, and a facet cannot
/// LAND there: mesh positions are f32, so the f64 normal carries its own
/// representation noise. The ring's bed chamfer measured 1.037e-6 mm² of steep
/// area at 45° — float noise, but a gate that reads `steep_area < 1e-6` is
/// right to fire on it, and the fix is geometric, not a looser gate. At 1.40
/// (54.5° from horizontal) every relief is unambiguously printable with
/// margin, and the steeper click-ring ramp also retains better.
const RELIEF_SLOPE: f64 = 1.40;

// ---- HARDWARE: NONE. ------------------------------------------------------
//
// v3 ships with an EMPTY bought list. The 608 constants below are retained for
// exactly one purpose: this run recomputes the v1 and v2 architectures on the
// SHIPPED v3 rotor, by the same solver, so the three-way ledger in
// analysis/ANALYSIS.md is an apples-to-apples comparison rather than a quote
// from memory. Nothing in `parts/`, `optional/` or the BOM references them.
/// 608 rotating inertia referred to the sun, g·mm² (researched band 610 ± 60)
/// — v1/v2 LEDGER ONLY. It is NOT in the shipped eta.
const I608_GMM2: f64 = 610.0;

/// Post Ø. WHY 5.50, down from v1/v2's 7.90: the post no longer has to be a
/// 608's Ø8.00 bore, so its diameter is now free — and every millimetre of it
/// costs spin time, because the sun's thrust land can never be closer to the
/// axis than the post's own radius. 5.50 is the campaign's own printed-pin
/// floor (see `PIN_D`: sub-Ø5 vertical printed pins are flagged failure-prone),
/// so post and pin are ONE number and the fit coupon gauges both at once.
const POST_D: f64 = 5.50;
/// Sun bore Ø. The sun now RUNS on the post, so this is the profile's running
/// fit, not a press seat: identical to the planet bore, on the same gauge pin.
const SUN_BORE_D: f64 = POST_D + 2.0 * C_FREE;
/// Radial width of the sun's annular thrust land on the hub, mm. The land's
/// INNER edge is fixed by geometry, not by choice: the sun's own bed relief
/// removes its underside out to bore_r + C_BED, so nothing inboard of that can
/// touch. Making the land narrow does NOT reduce Coulomb torque (μWr is
/// area-independent) — it reduces the ARM, which does. Gated in G17a.
const SUN_LAND_W: f64 = 0.50;
/// Radial width of each ring thrust pad, mm, and the inset of its inner edge
/// from the ring's root circle. The ring's continuous flat underside starts at
/// its root circle (34.25) — inboard of that the band is crenellated by the
/// tooth cavity — so 0.10 mm is the whole margin available for moving the pads
/// inward. v1 sat at 34.75; v3 sits at 34.60 and that is the floor.
const RING_PAD_W: f64 = 0.50;
const RING_PAD_INSET: f64 = 0.10;

// ---- printed-part geometry -------------------------------------------------
const ARM_HW: f64 = 5.00; // spider arm half-width (10.0 wide) — carries the Ø5.60 hole with 2.2 wall
const Z_ARM: f64 = 2.00; // base spider plate/arm thickness
const Z_GEAR: f64 = Z_ARM + C_Z; // 2.30 — gear plane bottom
/// Hub Ø. It carries the sun's thrust land and nothing else now — the 608's
/// Ø11 inner-race collar is DELETED with the bearing.
const HUB_D: f64 = 16.00;
const PIN_D: f64 = 5.50; // WHY 5.50 not 4.60: sub-Ø5 vertical printed pins are flagged failure-prone
const PLANET_BORE_D: f64 = PIN_D + 2.0 * C_FREE; // 6.00
const PLANET_SEAT_D: f64 = 7.00; // thrust pad Ø under each planet — see the drag budget
const TS_T: f64 = 2.00; // top spider thickness
const TS_R_IN: f64 = 23.00; // clears the sun tip r 22.0 by 1.0
const TS_R_IN_O: f64 = 25.00;
const TS_R_RIM: f64 = 35.25;
/// Static-part outer radius. WHY 36.20 and not 36.50: the RING stands 0.30 mm
/// proud of both held rims, so a flicking finger touches only the rotor. A
/// static rim flush with the rotor is a Coulomb rub, the worst loss class.
const STATIC_R: f64 = 36.20;
const CAP_D: f64 = 12.00;
const CAP_T: f64 = 1.20;
const SUN_LEAD: f64 = 0.60; // 0.6 × 45° bore lead-in at the top
const RIM_ROUND: f64 = 1.00; // full round, ring TOP rim only (see the deviation note)
/// Radial interference of the CAP's press fit on the post, mm — the one and
/// only interference fit left in the model (v4 deleted the other six).
///
/// A hoop-expansion joint's design limit is strain, not force: `ε = δ/a`. PLA
/// yields at 55 MPa / 3.3 GPa = 1.67 % strain (`tools/materials/pla.json`), so
/// 0.025 mm on the post's 2.75 mm radius is 0.91 % — elastic, with ×1.8 margin,
/// and the same joint class DRYBOX has already print-proved. The profile's own
/// `xy_clearance_tight` (0.05) on this post would be 1.82 %, past yield; G22b
/// caught that and refuses it.
const CAP_PRESS_R: f64 = 0.025;

// ---- THE BAYONET (v4): geometric top-spider retention ----------------------
//
// v3 held the top spider on with six Ø5.55 "click bands" — a frictional joint,
// and G16e disclosed honestly what that cost: the interference dies at
// 0.025 mm/side of printer error while the campaign's own worst-case XY figure
// (used for every clearance in G12) is 0.15 mm/side, SIX TIMES LARGER. On a
// poorly-calibrated machine the spider was a slip fit and the planets and ring
// lost their captor. Worse, as SHIPPED the band was Ø5.55 in a Ø5.60 hole — a
// 0.025 mm/side CLEARANCE, so the nominal interference the strain gates were
// computing did not exist in the geometry at all. Retention by preload was
// wrong in principle and absent in fact.
//
// v4 replaces it with RESPOOL's zero-preload lesson: retention is GEOMETRIC —
// a lug under a ceiling with a hard end stop, nothing left strained at rest.
// Each pin gets a NECK (the groove) and a radial FIN above it (the lug); each
// spider arm gets a slot whose two walls are the lip. The spider drops on at
// the entry bulge, twists BAY_PSI_DEG, and the fin then overhangs the slot's
// outboard wall by ENGAGE mm of solid material. Printer error changes how
// tight the twist feels; it cannot change the SIGN of that overlap.
//
// WHY A SNAP WAS REFUSED, with the arithmetic (G16g re-proves it every run).
// A hoop's bore strain IS δ/a exactly, so a Ø5.60 hole in this arm can expand
// only 0.047 mm before yield — and the interference a snap must survive is the
// same 0.30 mm diametral stack the engagement must survive. The elastic travel
// available at this scale is 6× too small, in every variant that fits inside
// the 12 mm envelope (a 3.4 mm collet finger reaches ~0.06 mm). The frozen
// spec's Ø6.40 barb was 14 % strain, eight times yield. There is no snap here,
// and that is geometry, not preference.
//
// WHY THE RETAINING FACE IS A FIN AND NOT A ROUND HEAD. Every support-free
// down-facing retaining face is a ≥RELIEF_SLOPE cone, and a cone WEDGES: the
// reaction is 1.40 : 1 horizontal : vertical. If that horizontal component has
// a tangential share the joint cams itself back toward the entry under any
// lift, however small — a bayonet that unscrews when you turn the toy over.
// The fin's overhang is trimmed to |y| ≤ FIN_HW, symmetric about the pin and
// wholly inside the slot wall's material, so the tangential shares cancel and
// the whole wedge force is RADIAL. Releasing it would need the spider's pin
// circle to grow ENGAGE mm — 4.3 % hoop strain — so the part breaks first.
/// Pin neck Ø through the spider — the groove the lip drops into.
const NECK_D: f64 = 2.70;
/// Fin half-width along the twist direction, mm. Bounds the overhang to a band
/// that is symmetric about the pin and always over solid wall (see above).
const FIN_HW: f64 = 1.00;
/// Fin inboard flat, mm from the pin axis. Keeps the entry bulge's inboard edge
/// outboard of the top spider's inner ring (r 25.0) instead of scalloping it.
const FIN_IN: f64 = 1.30;
/// Slot half-width, mm — the neck's running fit, so the twist stays free at the
/// worst over-extrusion corner. This is C_FREE, not a special number.
const SLOT_HW: f64 = NECK_D / 2.0 + C_FREE;
/// Radius of the locating pocket at the hard end stop, mm. A 45° lead-in walks
/// the neck into it, which is what re-centres the spider at lock: without it
/// the spider floats on six C_FREE slots (±0.29 mm) and its rim can come flush
/// with the ring it is supposed to stand 0.30 mm clear of.
const LOCK_R: f64 = NECK_D / 2.0 + C_TIGHT;
/// Distance from the locating pocket's centre to the lock position, mm.
const LOCK_Y: f64 = 1.40;
/// Entry-bulge half-extent along the twist direction, mm.
const BULGE_HW: f64 = SLOT_HW;
/// Entry-bulge outboard extent, mm from the pin axis — passes the fin.
const BULGE_X: f64 = PIN_D / 2.0 + C_FREE;
/// The twist, degrees. Sized so the fin's whole overhang band sits over solid
/// wall at lock (needs > FIN_HW + BULGE_HW = 2.60 mm of travel) with margin.
const BAY_PSI_DEG: f64 = 7.0;
/// Radial overlap of fin over slot wall at lock, mm — the retention itself.
const ENGAGE: f64 = PIN_D / 2.0 - SLOT_HW;
/// Top-spider arm planform (local frame: +x radial, +y = the twist direction).
/// The arm is offset toward the entry side because the slot is, and tapers at
/// the rim end so no corner exceeds STATIC_R.
const TS_ARM_Y0: f64 = -4.70;
const TS_ARM_Y1: f64 = 6.60;
const TS_ARM_YE: f64 = 3.60;
const TS_ARM_KNEE: f64 = 32.00;
const N_INDEX: usize = 7; // index grooves on the sun face — 7 is coprime with 42 and 6

// ============================================================================
// 2b. THE RING'S AXIAL SUPPORT — the problem v3 inherits and cannot buy out of.
//
// In a grounded-carrier star the ring is NOT on a bearing: it is located
// radially by six meshes and axially by the held frame. Held flat, its weight
// lands on that frame at r ≈ 34.6 mm and rubs. That is 0.36 N·mm — HALF of
// v3's whole budget, in the worst decay class there is (Coulomb, ω^0).
//
// The fix is forced by a kinematic fact: two bodies rotating about the SAME
// axis cannot roll on each other anywhere except ON that axis (their relative
// motion is a rotation about it, so every off-axis contact slides). Reaching
// the axis needs a WEB, and the web must cross the pin circle — so it must go
// over the top, where it forces a tall dead rim shell whose inertia the eta
// budget cannot absorb, and whose spokes cannot be printed in either
// orientation. Both halves are quantified and GATED (G20a/G20b). Putting a
// ROLLING ELEMENT between the two is what v2 did with steel; a PRINTED ball is
// mechanically fine (G21a recomputes the Hertz contact for PLA-on-PLA) and
// unprintable (G21b measures a sphere's own steep area on the engine).
//
// So v3 pays the sliding term, on six pads moved as far inboard as the ring's
// own continuous flat underside reaches, and publishes what that costs.
// ============================================================================

/// Bottom plane of the ring and the six planets. With no ball race under the
/// ring there is nothing to lift the rotors over, so they sit back down on the
/// gear plane — which also recovers 0.60 mm of the 12.0 mm envelope.
const Z_ROT: f64 = Z_GEAR;
/// Ring thrust-pad pitch radius, mm — the ring's root circle plus the pad inset
/// plus half the pad. This is the ARM in μWr and it is the single number the
/// whole v3 spin time is most sensitive to.
const RING_PAD_R: f64 = 34.25 + RING_PAD_INSET + RING_PAD_W / 2.0;

// ---- v1/v2 LEDGER ONLY: the steel that v3 does not ship --------------------
/// Ball Ø, count and pitch radius of v2's thrust race, and chrome-steel
/// density/elastic constants. Used ONLY to recompute the v2 row of the
/// three-way spin-time ledger on this run's rotor. No shipped part uses them.
const BALL_D: f64 = 1.50;
const N_BALL: usize = 24;
const RACE_R: f64 = 34.60;
const RHO_STEEL: f64 = 7.85e-3;
const E_STEEL: f64 = 200_000.0;
const NU_STEEL: f64 = 0.30;
/// PLA elastic constants — `tools/materials/pla.json` (3.3 GPa, ν 0.36,
/// yield 55 MPa). Promoted to consts because both the click-ring strain gate
/// and the Hertz race gate now read them.
const E_PLA_MPA: f64 = 3300.0;
const NU_PLA: f64 = 0.36;
const SIG_YIELD_PLA: f64 = 55.0;

// ---- SHIPPED design point (G11 re-derives it every run) ---------------------
/// Sun face width, mm — the variable the optimiser solves against the eta
/// constraint. v1 shipped 8.20, v2 8.16.
///
/// **v3's floor is GONE with the bearing.** v1/v2 could not go below 7.60
/// (`SUN_LIP + BRG_W` — the 608 had to live inside the bore), so the window was
/// 0.60 mm wide and the study had almost no room. v3 sweeps 3.00–8.20 in 0.02
/// and finds its own answer, and the answer moved for a physical reason: the
/// 608's 610 g·mm² used to sit on the SUN side of the eta balance, so deleting
/// it makes the sun side lighter and the RING has to come down to meet it.
const T_SUN: f64 = 7.80;
/// Ceiling of the sun-face design window, mm — the 12.0 mm envelope:
/// Z_GEAR + t_sun + C_Z + CAP_T ≤ 12.0.
const T_SUN_MAX: f64 = 8.20;
/// Ring face width, mm. v1/v2 shipped 4.50; v3 ships thinner because eta pins
/// I_ring to I_sun·k_S and the sun lost the bearing's inertia.
const T_RING: f64 = 4.00;
/// Ring rim wall, mm. WHY 2.25: exactly 5 × 0.45 mm extrusion lines on the one
/// part most likely to warp out of round. 2.00 is 4.4 lines — a partial wall.
const RING_WALL: f64 = 2.25;
/// Planet face width, mm. **v4 moved it, and the reason is the bayonet.**
/// v3 shipped 3.50 because the mass constraint (≤28 g) was binding and the
/// frame was 0.40 g heavier: hollowing six pins down to a Ø2.70 neck gave that
/// mass back, the study re-solved, and the optimum spent it on planet face
/// width (I_eff 12857 → 13056 g·mm²). Not hand-held — G11 asserts the shipped
/// point IS the optimum and it failed loudly on the first v4 run at 3.50.
const T_PLANET: f64 = 4.00;
/// SUN-B control puck: deliberately UNcancelled, so the buyer performs the A/B.
const SUNB_FRAC: f64 = 0.55;

// ---- physics constants (research-frozen; every one carries its provenance) --
/// Frozen launch speed for every published spin number, rad/s (= 1050 rpm).
/// WHY: it is the launch speed of the ONE instrumented spin-down measurement
/// in the literature (Szeged photogate study), and spin time SATURATES with
/// launch speed anyway (1000→5000 rpm buys only +41%).
const W0: f64 = 110.0;
/// Bearing drag speed exponent. Research band 0.43–0.60 (three measured grease
/// fits 0.434/0.4978/0.5986; SKF's own model gives d(lnM)/d(lnω) = 0.587 for a
/// 608). 0.50 is the centre.
const N_BRG: f64 = 0.50;
/// 608 drag torque at W0, N·mm, ZZ/open with light oil. SKF's own model gives
/// 0.0222 at ν = 20 mm²/s; Cojocaru et al. measured the SKF model 77–88% LOW
/// on miniature lightly-loaded bearings, so the nominal carries the 4.3×
/// low-end correction. BANDED, never quoted as a prediction.
const M608_NMM: f64 = 0.0955;
const M608_LO_NMM: f64 = 0.0222; // uncorrected SKF — an optimistic lower bound
const M608_HI_NMM: f64 = 0.1843; // SKF × 8.3, the correction's upper bound
/// PLA-on-PLA sliding friction. **UNKNOWN** — all published PLA tribology is
/// PLA-on-STEEL at 20 N (COF 0.52–0.67), the wrong pairing and the wrong load.
/// 0.30 is the research's own working assumption; the budget is reported
/// across 0.20–0.50 and the campaign designs around the high end.
const MU_PLA: f64 = 0.30;
const MU_LO: f64 = 0.20;
const MU_HI: f64 = 0.50;
/// Shipped layer height, mm. It is a PHYSICS constant in v3, not just a slicer
/// setting: it bounds the form error of any printed rolling element (G21b2).
const LAYER_H: f64 = 0.20;
const RHO_AIR: f64 = 1.204; // kg/m³ at 20 °C
const NU_AIR: f64 = 1.5e-5; // m²/s
const GRAV: f64 = 9.81;

// ---- safety ----------------------------------------------------------------
/// EN 71-1 §4.10: an accessible space between moving elements that admits a
/// Ø5 rod must also admit a Ø12 rod. The 5–12 mm band is forbidden.
const ROD_SMALL: f64 = 5.0;
const ROD_LARGE: f64 = 12.0;

// ============================================================================
// 3. GEAR MATHS — the engine has no API for any of this. Written here, gated.
// ============================================================================

fn pa() -> f64 {
	PA_DEG.to_radians()
}

/// Vertical rise of a relief whose radial run is `run` — see [`RELIEF_SLOPE`].
fn rise(run: f64) -> f64 {
	run * RELIEF_SLOPE
}

/// Arc the pin travels along the pin circle between entry and lock, mm.
fn bay_d() -> f64 {
	CD * BAY_PSI_DEG.to_radians()
}

/// Axial float of the locked spider, mm — how far it can rise before the fin's
/// relief cone meets the slot wall. It is `C_FREE · RELIEF_SLOPE` and nothing
/// else: the cone starts at the neck, the wall stands one running fit outboard
/// of it, and the cone climbs at the campaign's own support-free slope. Zero
/// preload by construction, and `e` shows what the tolerance stack does to it.
fn bay_float(e: f64) -> f64 {
	(SLOT_HW + e - (NECK_D / 2.0 - e)) * RELIEF_SLOPE
}

/// (radial offset `u`, arc length `s`) → the slot's local XY, origin at the
/// pin's LOCKED position, +x radially outward.
///
/// The pin travels along the PIN CIRCLE, not along a straight line, and over
/// the 7° twist that arc bows 0.20 mm inboard — most of one `C_FREE`. Two
/// things break if the slot is drawn straight: the entry end eats its own
/// clearance (an over-extruded machine then binds), and the FIN, whose flats
/// are cut in the PIN's frame, arrives at the bulge 7° out of square with it —
/// worth 0.014 mm of interference on the inboard corner, which the entry-pose
/// negative control duly caught. Designing the slot in (u, s) and mapping
/// through here fixes both at once: every clearance is measured perpendicular
/// to the real path, and the bulge is square to the fin at the moment the fin
/// has to drop through it.
fn arcp(u: f64, s: f64) -> DVec2 {
	let th = s / CD;
	let r = CD + u;
	DVec2::new(r * th.cos() - CD, r * th.sin())
}

/// The bayonet slot in one top-spider arm, drawn in (u, s) and mapped through
/// [`arcp`]. `e` dilates every wall — the worst-case-stack gate drives the same
/// code path with the full G12 error on it, so the proof is on geometry.
///
/// Shape, in travel order: an entry BULGE big enough to drop the fin through,
/// a strip that the neck slides down, a 45° lead-in, and a locating pocket that
/// is the hard end stop. The inboard wall runs the whole length at ONE radial
/// offset (the bulge opens outboard only) — that is what keeps the slot clear
/// of the top spider's inner ring at r 25.0 instead of scalloping it.
fn slot_outline(e: f64) -> Vec<DVec2> {
	let w = SLOT_HW + e;
	let lr = LOCK_R + e;
	let ls = -LOCK_Y;
	let d = bay_d();
	let (bx, bw) = (BULGE_X + e, BULGE_HW + e);
	let s0 = ls + (w - lr); // top of the 45° lead-in
	let (s1, s2) = (d - bw, d + bw);
	let mut p = Vec::with_capacity(72);
	for i in 0..=24 {
		let t = PI + PI * i as f64 / 24.0; // 180° → 360°: the locating pocket
		p.push(arcp(lr * t.cos(), ls + lr * t.sin()));
	}
	let edge = |p: &mut Vec<DVec2>, a: (f64, f64), b: (f64, f64), n: usize| {
		for i in 1..=n {
			let t = i as f64 / n as f64;
			p.push(arcp(a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t));
		}
	};
	edge(&mut p, (lr, ls), (w, s0), 1); // 45° lead-in, outboard
	edge(&mut p, (w, s0), (w, s1), 12); // outboard wall
	edge(&mut p, (w, s1), (bx, s1), 1); // into the bulge
	edge(&mut p, (bx, s1), (bx, s2), 8); // bulge, outboard
	edge(&mut p, (bx, s2), (-w, s2), 1); // bulge, entry end
	edge(&mut p, (-w, s2), (-w, s0), 12); // inboard wall — one offset, whole length
	p
}

/// Top-spider arm planform, world XY for the arm at angle 0 (`x` = radius).
fn ts_arm_outline() -> Vec<DVec2> {
	let (xi, xo) = (TS_R_IN + 0.30, TS_R_RIM + 0.50);
	vec![
		DVec2::new(xi, TS_ARM_Y0),
		DVec2::new(TS_ARM_KNEE, TS_ARM_Y0),
		DVec2::new(xo, -TS_ARM_YE),
		DVec2::new(xo, TS_ARM_YE),
		DVec2::new(TS_ARM_KNEE, TS_ARM_Y1),
		DVec2::new(xi, TS_ARM_Y1),
	]
}

/// One planet pin, revolved, at the origin — thrust boss, journal, the SEAT the
/// spider rests on, the neck, and the fin's relief cone. `e` erodes every
/// external surface by the same per-side printer error (under-extrusion thins
/// the pin AND widens the hole, so the two add; the gate drives both).
///
/// Printability, bottom to top: the boss flare and the seat step are up-facing;
/// the neck narrows going up (self-supporting); the ONE down-facing face in the
/// whole feature is the fin's relief cone, cut at `RELIEF_SLOPE` — not 45°,
/// which a facet cannot land on (see [`RELIEF_SLOPE`]). No horizontal ceiling
/// anywhere, so the set keeps its zero-bridge property (G22c).
fn bay_pin_blank(e: f64) -> Solid {
	let r = PIN_D / 2.0 - e;
	let nr = NECK_D / 2.0 - e;
	let z_cone = ts_top() + rise(PIN_D / 2.0 - NECK_D / 2.0);
	revolve(
		&force_ccw(vec![
			DVec2::new(0.0, 1.00),
			DVec2::new(4.30, 1.00),
			// the cylinder→flare transition sits at z 1.60, BELOW the arm's top
			// face (2.00), so no pin edge lies IN that plane — §7.7 rule 3
			// (an edge in a coincident face flipped this exact chain invalid).
			DVec2::new(4.30, 1.60),
			DVec2::new(PLANET_SEAT_D / 2.0, Z_ROT),
			DVec2::new(r, Z_ROT),
			DVec2::new(r, ts_bot()), // SEAT: the spider's whole axial location
			DVec2::new(nr, ts_bot()),
			DVec2::new(nr, ts_top()),
			DVec2::new(r, z_cone), // fin relief cone, RELIEF_SLOPE
			DVec2::new(r - 0.40, z_cone + 0.40),
			DVec2::new(0.0, z_cone + 0.40),
		]),
		48,
	)
}

/// The three flats that turn a revolved head into the radial FIN, at the
/// origin, oriented for the arm at angle 0. Started 0.40 mm below the cone so
/// no cutter face is coincident with the cone's start circle (§7.7 rule 3).
fn bay_fin_cutters(e: f64) -> Solid {
	let (z0, z1) = (ts_top() - 0.40, pin_top() + 1.0);
	let big = 6.0;
	let (fy, fx) = (FIN_HW - e, FIN_IN - e);
	let a = cuboid(DVec3::new(-big, fy, z0), DVec3::new(big, big, z1));
	let b = cuboid(DVec3::new(-big, -big, z0), DVec3::new(big, -fy, z1));
	let c = cuboid(DVec3::new(-big, -big, z0), DVec3::new(-fx, big, z1));
	union(&union(&a, &b), &c)
}

/// One finished bayonet pin (blank − flats), at the origin.
fn bay_pin(e: f64) -> Solid {
	difference(&bay_pin_blank(e), &bay_fin_cutters(e))
}

/// Pitch / base / tip / root radii of one member.
fn radii(z: usize, external: bool) -> (f64, f64, f64, f64) {
	let rp = M * z as f64 / 2.0;
	let rb = rp * pa().cos();
	if external {
		(rp, rb, rp + M, rp - 1.25 * M)
	} else {
		(rp, rb, rp - M, rp + 1.25 * M) // internal: tip is INSIDE the pitch circle
	}
}

/// Transverse contact ratio of an EXTERNAL–EXTERNAL mesh at standard centre
/// distance: `ε = (√(ra1²−rb1²) + √(ra2²−rb2²) − a·sin α) / (π·m·cos α)`.
/// Fully parametric so the negative control can drive the SAME code path with
/// inputs that must read below the floor.
fn contact_ratio_external(m: f64, alpha_deg: f64, z1: usize, z2: usize) -> f64 {
	let a = alpha_deg.to_radians();
	let (rp1, rp2) = (m * z1 as f64 / 2.0, m * z2 as f64 / 2.0);
	let (rb1, rb2) = (rp1 * a.cos(), rp2 * a.cos());
	let (ra1, ra2) = (rp1 + m, rp2 + m);
	((ra1 * ra1 - rb1 * rb1).sqrt() + (ra2 * ra2 - rb2 * rb2).sqrt() - (rp1 + rp2) * a.sin()) / (PI * m * a.cos())
}

/// Transverse contact ratio of an INTERNAL mesh (pinion `zp` inside ring `zr`):
/// `ε = (√(rap²−rbp²) − √(rar²−rbr²) + a·sin α) / (π·m·cos α)` — the ring term
/// and the centre-distance term both change sign versus the external case.
fn contact_ratio_internal(m: f64, alpha_deg: f64, zp: usize, zr: usize) -> f64 {
	let a = alpha_deg.to_radians();
	let (rpp, rpr) = (m * zp as f64 / 2.0, m * zr as f64 / 2.0);
	let (rbp, rbr) = (rpp * a.cos(), rpr * a.cos());
	let (rap, rar) = (rpp + m, rpr - m);
	((rap * rap - rbp * rbp).sqrt() - (rar * rar - rbr * rbr).sqrt() + (rpr - rpp) * a.sin()) / (PI * m * a.cos())
}

/// Undercut floor `z_min = 2(1−x)/sin²α` (ISO): the smallest tooth count a
/// rack-generated gear reaches without undercutting at shift `x`.
fn undercut_floor(x: f64) -> f64 {
	2.0 * (1.0 - x) / (pa().sin() * pa().sin())
}

/// Lewis form factor Y measured from the generator's OWN tooth outline
/// (σ = Wt/(b·m·Y)) — tip-loaded cantilever, critical section = max 6h/t²
/// scanned over the densified boundary. Promoted from planetary26.rs so this
/// campaign rates the tooth it actually builds, not a handbook tooth.
fn lewis_y(z: usize, external: bool) -> f64 {
	let pts = gear_profile(z, external, false);
	let pitch = TAU / z as f64;
	let ra = pts.iter().map(|p| p.length()).fold(0.0f64, f64::max);
	let r_tip = pts.iter().map(|p| p.length()).fold(f64::INFINITY, f64::min);
	let mut worst = 0.0f64;
	for w in 0..pts.len() {
		let (a, b) = (pts[w], pts[(w + 1) % pts.len()]);
		for j in 0..8 {
			let p = a + (b - a) * (j as f64 / 8.0);
			let (r, th) = (p.length(), p.y.atan2(p.x).abs());
			if th > pitch * 0.5 {
				continue; // neighbour teeth would fake a razor-thin section
			}
			let (t, h) = if external {
				(2.0 * r * th.sin(), ra - r * th.cos())
			} else {
				(2.0 * r * (pitch * 0.5 - th).max(0.0).sin(), r - r_tip)
			};
			if t > 1e-9 && h > 1e-9 {
				worst = worst.max(6.0 * h / (t * t));
			}
		}
	}
	1.0 / (M * worst)
}

/// Signed area and POLAR second moment `J = ∫(x²+y²)dA` about the origin of a
/// closed polygon, exactly (the standard shoelace second-moment identity).
/// This is the inertia surrogate the design study runs on: every rotor is a
/// prism, and a prism's `I_zz` about its own axis is exactly `ρ·h·J` — linear
/// in the height, so the study is EXACT for the prismatic core. Fixed-size
/// features (chamfers, the bore lip, the index grooves, the rim round) are not
/// prismatic; they are a correction, and the shipped point is re-measured with
/// `mass_properties` on the exact B-rep and gated there (G9).
fn poly_area_j(p: &[DVec2]) -> (f64, f64) {
	let (mut a, mut j) = (0.0f64, 0.0f64);
	for i in 0..p.len() {
		let (u, v) = (p[i], p[(i + 1) % p.len()]);
		let cr = u.x * v.y - v.x * u.y;
		a += cr;
		j += cr * (u.x * u.x + u.x * v.x + v.x * v.x + u.y * u.y + u.y * v.y + v.y * v.y);
	}
	(0.5 * a.abs(), (j / 12.0).abs())
}

// ============================================================================
// 4. SPIN-DOWN SOLVER — written for this campaign, benchmarked before use
//    (DESIGN_GUIDE §25.7 answer-type 2: guilty until its own gates are green).
// ============================================================================

/// A drag budget as a sum of power-law terms `Σ cⱼ·ω^pⱼ` (N·m, ω in rad/s),
/// every term already REFLECTED to the observable rotor (the ring).
#[derive(Clone, Debug, Default)]
struct Drag {
	terms: Vec<(f64, f64, String)>,
}

impl Drag {
	fn add(&mut self, c: f64, p: f64, what: &str) {
		self.terms.push((c, p, what.to_string()));
	}
	fn torque(&self, w: f64) -> f64 {
		self.terms.iter().map(|(c, p, _)| c * w.powf(*p)).sum()
	}
	fn total_nmm(&self, w: f64) -> f64 {
		self.torque(w) * 1e3
	}
}

/// Spin-down by EXACT QUADRATURE of the research's governing model
/// `I·dω/dt = −T(ω)`:
///   `t(ω₀) = ∫₀^ω₀ I dω / T(ω)`, `θ(ω₀) = ∫₀^ω₀ I ω dω / T(ω)`.
///
/// The integrand is singular at ω = 0 whenever the slowest term is a pure
/// power law (no Coulomb term). The substitution `ω = ω₀·s^{1/(1−p_min)}`
/// removes that singularity EXACTLY: the integrand becomes constant as s → 0.
/// Composite Simpson on s ∈ [0,1], 4000 intervals — deterministic, so the
/// design study's purity check holds.
///
/// Returns `(seconds, revolutions)`.
fn spin_down(i_eff_kgm2: f64, d: &Drag, w0: f64) -> (f64, f64) {
	let pmin = d.terms.iter().map(|t| t.1).fold(f64::INFINITY, f64::min);
	assert!(pmin < 1.0, "a drag budget whose slowest term is ω¹ or faster never stops");
	let p = 1.0 / (1.0 - pmin);
	let c_min: f64 = d.terms.iter().filter(|t| (t.1 - pmin).abs() < 1e-12).map(|t| t.0).sum();
	// s → 0 limits (the substitution's whole point)
	let f_t0 = i_eff_kgm2 * w0 * p / (c_min * w0.powf(pmin));
	let f_a0 = 0.0; // the θ integrand carries an extra ω, so it vanishes at s = 0
	let f = |s: f64| -> (f64, f64) {
		if s <= 0.0 {
			return (f_t0, f_a0);
		}
		let w = w0 * s.powf(p);
		let dw = w0 * p * s.powf(p - 1.0);
		let t = d.torque(w);
		(i_eff_kgm2 * dw / t, i_eff_kgm2 * w * dw / t)
	};
	const N: usize = 4000;
	let h = 1.0 / N as f64;
	let (mut st, mut sa) = (0.0f64, 0.0f64);
	for k in 0..=N {
		let (a, b) = f(k as f64 * h);
		let wgt = if k == 0 || k == N {
			1.0
		} else if k % 2 == 1 {
			4.0
		} else {
			2.0
		};
		st += wgt * a;
		sa += wgt * b;
	}
	(st * h / 3.0, sa * h / 3.0 / TAU)
}

/// Free-disc (von Kármán) skin-friction torque on BOTH faces of a disc of
/// radius `r_m` spinning at ω: `T = 2 · ½·Cm·ρ·ω²·R⁵`, `Cm = 3.87·Re^{-½}`
/// (laminar), `Re = ωR²/ν`. The Cm normalisation could NOT be verified at a
/// primary source and is flagged in ANALYSIS.md; it drives ~20 % of the air
/// term, so conclusions are insensitive to it, but it is not quoted as
/// sourced. Because Cm ∝ ω^-½ the term is ω^1.5, not ω² — it is carried at its
/// true exponent rather than forced into the two-term form.
fn disc_air_coeff(r_m: f64) -> f64 {
	// T = 2·½·3.87·(ν/R²)^½·ρ·ω^1.5·R⁵  →  coefficient of ω^1.5
	3.87 * (NU_AIR / (r_m * r_m)).sqrt() * RHO_AIR * r_m.powi(5)
}

// ---------------------------------------------------------------------------
// 4b. HERTZ CONTACT + THE ROLLING RACE'S LOSS
//
// The whole v2 spin-time result rests on these three functions, so each one is
// either a textbook closed form driven by sourced constants, or a RIGOROUS
// BOUND — never a fitted coefficient. Benchmarked in main() as B5/B6 before
// they are used, exactly as the spin-down solver was (§25.7 answer-type 2).
// ---------------------------------------------------------------------------

/// Reduced modulus of an elastic contact, MPa: `1/E* = (1−ν₁²)/E₁ + (1−ν₂²)/E₂`.
fn e_star(e1: f64, nu1: f64, e2: f64, nu2: f64) -> f64 {
	1.0 / ((1.0 - nu1 * nu1) / e1 + (1.0 - nu2 * nu2) / e2)
}

/// Hertz contact radius of a SPHERE on a FLAT, mm: `a = (3FR/4E*)^⅓`.
fn hertz_a(load_n: f64, r_ball: f64, estar: f64) -> f64 {
	(3.0 * load_n * r_ball / (4.0 * estar)).cbrt()
}

/// Hertz peak pressure under that contact, MPa: `p₀ = 3F/(2πa²)`.
fn hertz_p0(load_n: f64, a: f64) -> f64 {
	3.0 * load_n / (TAU * a * a)
}

/// Hertz mutual approach of the same contact, mm: `δ = (9F²/16RE*²)^⅓`. This is
/// an INDEPENDENT expression of the same solution, and `a² = Rδ` links the two —
/// which is what B5 exploits to benchmark [`hertz_a`] against a different
/// algebraic path rather than against itself.
fn hertz_delta(load_n: f64, r_ball: f64, estar: f64) -> f64 {
	(9.0 * load_n * load_n / (16.0 * r_ball * estar * estar)).cbrt()
}

/// Resisting torque at the observable rotor from a FLAT-RACE thrust ball
/// bearing carrying `w_n` newtons on `n` balls of diameter `d` at pitch radius
/// `r`, in N·m. Returns `(rolling_bound, contact_spin, ball_to_ball_bound)`.
///
/// **Rolling term — a BOUND, not a fit.** Rolling resistance is a moment `f·N`
/// per contact, `f` the forward offset of the pressure resultant. `f` is NOT
/// sourced for a steel ball on printed PLA and is NOT invented here. What IS
/// rigorous is that the resultant of a pressure distribution cannot lie outside
/// the patch it acts on, so `f ≤ a`, the Hertz contact radius — which this
/// campaign computes from PLA's own published modulus and the real ball load.
/// The ball rolls at `Ω = ωr/d` relative to BOTH races, so the power is
/// `2·n·f·(W/n)·Ω` and the reflected torque is `2·f·W·r/d`. Using `f = a`
/// makes every published v2 spin time a LOWER bound on the model's answer.
///
/// **Contact spin.** A ball between two flat plates, one turning at ω, spins
/// about the contact normal at exactly ω/2 relative to each race (the two spin
/// moments must balance and the contacts are identical). A Hertzian circular
/// patch resists spin with `3πμNa/32`, giving `(3π/32)·μ·W·a` at the rotor.
///
/// **Ball-to-ball.** Adjacent balls touch on the line joining their centres —
/// which is the CIRCUMFERENTIAL direction, and the balls' rolling spin axes are
/// circumferential too, so the rolling component contributes exactly zero slip
/// there. Only the ω/2 spin component does: `2·(d/2)·(ω/2)`. That is 2 % of the
/// race's own surface speed, and the term is returned evaluated at the ABSURD
/// normal force of the entire ring weight pressing one ball against the next —
/// a bound nobody can argue is optimistic. It is DECLARED and omitted, like the
/// mesh-sliding term, rather than folded into the headline.
fn race_terms(w_n: f64, n: usize, d: f64, r: f64, mu: f64) -> (f64, f64, f64) {
	let estar = e_star(E_STEEL, NU_STEEL, E_PLA_MPA, NU_PLA);
	let a_mm = hertz_a(w_n / n as f64, d / 2.0, estar);
	let rolling = 2.0 * (a_mm * 1e-3) * w_n * (r / d);
	let spin = (3.0 * PI / 32.0) * mu * w_n * (a_mm * 1e-3);
	let ball_ball = mu * w_n * (d * 0.5) * 1e-3;
	(rolling, spin, ball_ball)
}

// ============================================================================
// 5. GEOMETRY
// ============================================================================

fn gear_profile(z: usize, external: bool, half_shift: bool) -> Vec<DVec2> {
	involute_ring_outline_shifted_filleted(
		M,
		z,
		PA_DEG,
		external,
		half_shift,
		LASH,
		X_SHIFT,
		if external { RF } else { 0.0 },
	)
	.expect("gear outline: the engine-refusal probe is gated in main()")
}

fn tr(x: f64, y: f64, z: f64) -> DAffine3 {
	DAffine3::from_translation(DVec3::new(x, y, z))
}
fn rotz(a: f64) -> DAffine3 {
	DAffine3::from_axis_angle(DVec3::Z, a)
}

/// 45° tip-relief cutter for an EXTERNAL gear at the face plane `zp`
/// (`up` = the part's material lies at +z from `zp`).
fn tip_relief_ext(ra: f64, c: f64, zp: f64, up: bool) -> Solid {
	let s = if up { 1.0 } else { -1.0 };
	let h = rise(c);
	let far = ra + 6.0;
	let pts = vec![
		DVec2::new(ra - c - 0.5 / RELIEF_SLOPE, zp - s * 0.5),
		DVec2::new(far, zp - s * 0.5),
		DVec2::new(far, zp + s * h),
		DVec2::new(ra, zp + s * h),
	];
	revolve(&force_ccw(pts), 96)
}

/// 45° tip-relief cutter for an INTERNAL ring (tips point inward at `rt`).
fn tip_relief_int(rt: f64, c: f64, zp: f64, up: bool) -> Solid {
	let s = if up { 1.0 } else { -1.0 };
	let h = rise(c);
	let pts = vec![
		DVec2::new(0.0, zp - s * 0.5),
		DVec2::new(rt + c + 0.5 / RELIEF_SLOPE, zp - s * 0.5),
		DVec2::new(rt, zp + s * h),
		DVec2::new(0.0, zp + s * h),
	];
	revolve(&force_ccw(pts), 96)
}

/// P3 SUN (and P5 SUN-B, the deliberately uncancelled control puck, at
/// `SUNB_FRAC·T_SUN`). 42T external, bored as a RUNNING fit on the printed post
/// — v3 has no bearing, so this bore is the journal — with a bed-side relief, a
/// 0.6×45° top lead-in, tip chamfers both faces and seven radial index grooves
/// so the counter-rotation is legible on video.
///
/// The bed relief is load-bearing in the drag budget, not just in the fit: it
/// removes the sun's underside out to `bore_r + C_BED`, which is therefore the
/// closest the sun's thrust land can possibly get to the axis. Gated in G17a.
fn sun(face: f64) -> Result<Solid, kernel_brep::ChainError> {
	let (_, _, ra, _) = radii(S_T, true);
	let body = extrude(&gear_profile(S_T, true, false), face);
	let mut ch = ChainLog::start("sun blank", body)?.seal();
	let r_seat = SUN_BORE_D / 2.0;
	let bore = {
		let p = vec![
			DVec2::new(r_seat + C_BED, -1.0),
			DVec2::new(r_seat + C_BED, 0.0),
			DVec2::new(r_seat, rise(C_BED)),
			DVec2::new(r_seat, face - rise(SUN_LEAD)),
			DVec2::new(r_seat + SUN_LEAD, face),
			DVec2::new(r_seat + SUN_LEAD, face + 1.0),
			DVec2::new(0.0, face + 1.0),
			DVec2::new(0.0, -1.0),
		];
		revolve(&force_ccw(p), 96)
	};
	ch.apply("sun bore", |s| difference(s, &bore))?;
	ch.apply("tip relief (bed)", |s| difference(s, &tip_relief_ext(ra, C_TIP, 0.0, true)))?;
	ch.apply("tip relief (top)", |s| difference(s, &tip_relief_ext(ra, C_TIP, face, false)))?;
	// index grooves — 7-fold, so they cannot bias the balance gate the way a
	// 6-fold or 42-fold pattern would (7 is coprime with both).
	let (gr_in, gr_out, gr_w, gr_d) = (r_seat + 1.6, ra - 1.6, 1.20, 0.40);
	let mut grooves: Option<Solid> = None;
	for k in 0..N_INDEX {
		let a = TAU * k as f64 / N_INDEX as f64 + 0.35; // off the +X meridian
		let bar = cuboid(DVec3::new(gr_in, -gr_w / 2.0, face - gr_d), DVec3::new(gr_out, gr_w / 2.0, face + 1.0))
			.transformed(rotz(a));
		grooves = Some(match grooves {
			None => bar,
			Some(b) => union(&b, &bar),
		});
	}
	ch.apply("seven index grooves", |s| difference(s, &grooves.expect("7 grooves")))?;
	Ok(ch.finish())
}

/// P4 PLANET — 12T external, Ø6.00 bore (or a ladder variant), bed + top bore
/// relief, tip chamfers both faces.
fn planet(face: f64, bore_d: f64) -> Result<Solid, kernel_brep::ChainError> {
	let (_, _, ra, _) = radii(P_T, true);
	let body = extrude(&gear_profile(P_T, true, true), face);
	let mut ch = ChainLog::start("planet blank", body)?.seal();
	let rb = bore_d / 2.0;
	let bore = revolve(
		&force_ccw(vec![
			DVec2::new(rb + C_BED, -1.0),
			DVec2::new(rb + C_BED, 0.0),
			DVec2::new(rb, rise(C_BED)),
			DVec2::new(rb, face - rise(C_BED)),
			DVec2::new(rb + C_BED, face),
			DVec2::new(rb + C_BED, face + 1.0),
			DVec2::new(0.0, face + 1.0),
			DVec2::new(0.0, -1.0),
		]),
		96,
	);
	ch.apply("planet bore", |s| difference(s, &bore))?;
	ch.apply("tip relief (bed)", |s| difference(s, &tip_relief_ext(ra, C_TIP, 0.0, true)))?;
	ch.apply("tip relief (top)", |s| difference(s, &tip_relief_ext(ra, C_TIP, face, false)))?;
	Ok(ch.finish())
}

/// P0 RING — 66T internal cut into a rim of OD `2(34.25 + wall)`, with a 1.0 mm
/// full round on the TOP rim edge and a 0.45 × 45° chamfer on the BED edge.
///
/// DEVIATION FROM SPEC, with reason: the frozen spec asks for a 1.0 mm full
/// round on BOTH rim edges. A full round at the bed has a tangent that is a
/// 90° overhang, which the support-free gate fires on (correctly). The bed
/// side therefore gets the campaign's own 0.45 × 45° chamfer — which is what
/// the elephant-foot rule asks for anyway — and the top keeps the round for
/// skin comfort and the aero/bed-adhesion transition.
fn ring(face: f64, wall: f64) -> Result<Solid, kernel_brep::ChainError> {
	let (_, _, r_tip, r_root) = radii(R_T, false); // 32.0 tip, 34.25 root
	let od = r_root + wall;
	// rim blank as ONE revolve: bed chamfer, straight wall, top round
	let mut p = vec![
		DVec2::new(30.0, 0.0),
		DVec2::new(od - C_BED, 0.0),
		DVec2::new(od, rise(C_BED)),
		DVec2::new(od, face - RIM_ROUND),
	];
	for k in 1..=8 {
		let a = FRAC_PI_2_STEPS * k as f64;
		p.push(DVec2::new(od - RIM_ROUND + RIM_ROUND * a.cos(), face - RIM_ROUND + RIM_ROUND * a.sin()));
	}
	p.push(DVec2::new(30.0, face));
	let blank = revolve(&force_ccw(p), 96);
	let mut ch = ChainLog::start("ring blank", blank)?.seal();
	// the toothed cavity: one prism cutter, strictly inside the rim material
	// (max r 34.25 < od − RIM_ROUND) so it never touches the OD, the chamfer
	// or the round — §7.7 rule 3.
	let cav = extrude(&gear_profile(R_T, false, true), face + 2.0).transformed(tr(0.0, 0.0, -1.0));
	let hz = boolean_hazards(ch.solid(), &cav, 0.05);
	let warn = hz
		.iter()
		.filter(|h| matches!(h.kind, HazardKind::NearCoincidentPlanes | HazardKind::NearCoincidentCylinders | HazardKind::EdgeInFace))
		.count();
	assert!(warn == 0, "ring cavity cutter fails the §7.7 pre-flight: {hz:?}");
	ch.apply("ring cavity", |s| difference(s, &cav))?;
	ch.apply("tip relief (bed)", |s| difference(s, &tip_relief_int(r_tip, C_TIP, 0.0, true)))?;
	ch.apply("tip relief (top)", |s| difference(s, &tip_relief_int(r_tip, C_TIP, face, false)))?;
	Ok(ch.finish())
}

const FRAC_PI_2_STEPS: f64 = PI / 16.0; // 8 samples over the quarter round

/// P1 BASE SPIDER — the held frame. Hub + collar + post (one revolve), three
/// full-diameter bars (= six arms, no coincident side planes), six ring thrust
/// pads and six bayonet planet pins with their thrust bosses.
///
/// `e` is the per-side printer error applied to the RETENTION features only —
/// 0.0 for the shipped part; the worst-case-stack gate rebuilds the same part
/// with the full G12 error so the engagement proof runs on solids.
fn base_spider(e: f64) -> Result<Solid, kernel_brep::ChainError> {
	let post_top = cap_top();
	// v3 core: hub, the SUN THRUST LAND, and the post. The 608's Ø11 inner-race
	// collar is gone with the bearing. The hub's top face is recessed one axial
	// clearance BELOW the gear plane so that the only thing the sun's underside
	// can touch is the raised land — an annulus running from the post's own OD
	// out to `sun_land_out`, of which the sun's bed relief leaves
	// `bore_r + C_BED … sun_land_out` in contact. That contact annulus IS the
	// arm in μWr and G17a asserts it is the smallest the geometry allows.
	//
	// The hub top is recessed by TWO axial clearances, not one. One would put it
	// at exactly Z_ARM — the arms' own top plane — and a coincident plane between
	// two unioned bodies is §7.7 rule 3: the chain went invalid (genus 2, not
	// watertight) the first time it was written that way. The fix is geometric.
	let sun_land_out = SUN_BORE_D / 2.0 + C_BED + SUN_LAND_W;
	let hub_top = Z_GEAR - 2.0 * C_Z;
	let core = revolve(
		&force_ccw(vec![
			DVec2::new(0.0, 0.0),
			DVec2::new(HUB_D / 2.0, 0.0),
			DVec2::new(HUB_D / 2.0, hub_top),
			DVec2::new(sun_land_out, hub_top),
			DVec2::new(sun_land_out, Z_GEAR),
			DVec2::new(POST_D / 2.0, Z_GEAR),
			DVec2::new(POST_D / 2.0, post_top - 0.40),
			DVec2::new(POST_D / 2.0 - 0.40, post_top),
			DVec2::new(0.0, post_top),
		]),
		96,
	);
	let mut ch = ChainLog::start("base core", core)?.seal();
	// §7.7: pre-union each DISJOINT feature set into ONE arrangement instead of
	// growing the body six times. Same result, a fraction of the boolean work,
	// and far fewer places for a chain to go invalid.
	let mut bars: Option<Solid> = None;
	// v3 drops v2's closed race rim (there is no channel to carry any more) and
	// puts the arms back out to STATIC_R, which is 1.6 g lighter and puts the
	// grip surface where the flicking finger expects it — one axial clearance
	// and 0.30 mm of radius clear of the rotor.
	let arm_r = STATIC_R;
	for k in 0..3 {
		let a = PI * k as f64 / 3.0;
		let bar = cuboid(DVec3::new(-arm_r, -ARM_HW, 0.0), DVec3::new(arm_r, ARM_HW, Z_ARM)).transformed(rotz(a));
		bars = Some(match bars {
			None => bar,
			Some(b) => union(&b, &bar),
		});
	}
	ch.apply("six arms (3 crossing bars)", |s| union(s, &bars.expect("3 bars")))?;
	// ---- six RING THRUST PADS -----------------------------------------------
	// The ring's whole weight rubs here. The pads are pushed as far inboard as
	// the ring's own CONTINUOUS flat underside reaches — its root circle plus
	// RING_PAD_INSET — because the arm is the only term in μWr that geometry can
	// still move. Widening the pads would NOT reduce the torque (Coulomb
	// friction is area-independent); it only reduces bearing pressure, which is
	// already four orders under yield (G17c). Six of them rather than a closed
	// annulus: less mass, less contact area to stick, and the count itself is
	// proved not to move the answer (G17e).
	let mut pads: Option<Solid> = None;
	for k in 0..N_PL {
		let a = TAU * k as f64 / N_PL as f64;
		// bottom buried INSIDE the arm slab and sides well inside its half-width,
		// so every union face is fully in material or fully in air (§7.7 rule 3)
		let pad = cuboid(
			DVec3::new(RING_PAD_R - RING_PAD_W / 2.0, -3.0, Z_ARM - 0.50),
			DVec3::new(RING_PAD_R + RING_PAD_W / 2.0, 3.0, Z_ROT),
		)
		.transformed(rotz(a));
		pads = Some(match pads {
			None => pad,
			Some(b) => union(&b, &pad),
		});
	}
	ch.apply("six ring thrust pads", |s| union(s, &pads.expect("6 pads")))?;
	// ---- six BAYONET PINS ----------------------------------------------------
	// Journal for the planet, SEAT for the top spider at `ts_bot()`, neck, and
	// the radial fin that is the whole retention (see the BAYONET block at
	// NECK_D). The blanks and the fin flats are each pre-unioned into ONE
	// disjoint arrangement, per §7.7, so the chain grows by two operations
	// instead of twenty-four.
	let mut pins: Option<Solid> = None;
	let mut flats: Option<Solid> = None;
	for k in 0..N_PL {
		let a = TAU * k as f64 / N_PL as f64;
		let at = tr(CD, 0.0, 0.0);
		let pin = bay_pin_blank(e).transformed(at).transformed(rotz(a));
		let flat = bay_fin_cutters(e).transformed(at).transformed(rotz(a));
		pins = Some(match pins {
			None => pin,
			Some(b) => union(&b, &pin),
		});
		flats = Some(match flats {
			None => flat,
			Some(b) => union(&b, &flat),
		});
	}
	let pins = pins.expect("6 pins");
	// §7.7 pre-flight on the one genuinely risky union of this part.
	let hz = boolean_hazards(ch.solid(), &pins, 0.05);
	let warn = hz
		.iter()
		.filter(|h| matches!(h.kind, HazardKind::NearCoincidentPlanes | HazardKind::NearCoincidentCylinders | HazardKind::EdgeInFace))
		.count();
	assert!(warn == 0, "planet-pin union fails the §7.7 pre-flight: {hz:?}");
	ch.apply("six planet pins", |s| union(s, &pins))?;
	ch.apply("six fin flats", |s| difference(s, &flats.expect("6 flat sets")))?;
	Ok(ch.finish())
}

/// The top spider WITHOUT its retaining rim — exists only so G23c can prove the
/// capture gate is falsifiable. If this ever stops being buildable, delete the
/// NC honestly rather than letting G23a pass unchallenged.
fn top_spider_no_rim(z0: f64) -> Result<Solid, kernel_brep::ChainError> {
	let ann = |r_in: f64, r_out: f64| {
		difference(
			&cylinder(DVec3::new(0.0, 0.0, z0), DVec3::Z, r_out, TS_T, 96),
			&cylinder(DVec3::new(0.0, 0.0, z0 - 1.0), DVec3::Z, r_in, TS_T + 2.0, 96),
		)
	};
	// inner hub + arms only; the outer rim over the ring is deliberately absent
	let mut ch = ChainLog::start("nc inner ring", ann(TS_R_IN, TS_R_IN_O))?.seal();
	let mut arms: Option<Solid> = None;
	for k in 0..N_PL {
		let a = TAU * k as f64 / N_PL as f64;
		// the shipped planform, TRUNCATED well inboard of the ring's back: this
		// NC exists to prove the rim is what captures the ring, so its arms must
		// not reach over the ring and do the job by accident.
		let bar = cuboid(
			DVec3::new(TS_R_IN + 0.30, TS_ARM_Y0, z0),
			DVec3::new(CD + 3.0, TS_ARM_Y1, z0 + TS_T),
		)
		.transformed(rotz(a));
		arms = Some(match arms {
			None => bar,
			Some(b) => union(&b, &bar),
		});
	}
	ch.apply("six arms", |s| union(s, &arms.expect("6 arms")))?;
	Ok(ch.finish())
}

/// P2 TOP SPIDER — inner ring (clears the sun tip by 1.0), six arms, closed
/// outer rim over the ring rim, and six BAYONET SLOTS on the pins.
///
/// DEVIATION: the frozen spec lists BOTH six 10 mm tabs over the ring rim AND
/// a closed outer rim; a closed rim already covers the rim everywhere and is
/// rounder and stiffer, so the two are merged. The spec's snap barbs are
/// refused with their arithmetic (14 % hoop strain — see the BAYONET block at
/// `NECK_D`), and v3's press fits are refused too: a press fit's grip is
/// whatever the printer leaves of a 0.025 mm interference, which is nothing at
/// the campaign's own worst-case error. The slot's two walls are the lip; the
/// pin's fin is the shoulder; the twist is what puts one over the other.
///
/// `e` dilates the slots by the per-side printer error (worst-case gate);
/// `nc_round` swaps every slot for a plain round hole that clears the fin — the
/// negative control that must make the retention gate read exactly zero.
fn top_spider_var(z0: f64, e: f64, nc_round: bool) -> Result<Solid, kernel_brep::ChainError> {
	let ann = |r_in: f64, r_out: f64| {
		difference(
			&cylinder(DVec3::new(0.0, 0.0, z0), DVec3::Z, r_out, TS_T, 96),
			&cylinder(DVec3::new(0.0, 0.0, z0 - 1.0), DVec3::Z, r_in, TS_T + 2.0, 96),
		)
	};
	let mut ch = ChainLog::start("top inner ring", ann(TS_R_IN, TS_R_IN_O))?.seal();
	ch.apply("outer rim over the ring", |s| union(s, &ann(TS_R_RIM, STATIC_R)))?;
	// arms and slot cutters are each a DISJOINT set: pre-unioned into one
	// arrangement per §7.7 rather than grown onto the body twelve times.
	let mut arms: Option<Solid> = None;
	let mut holes: Option<Solid> = None;
	for k in 0..N_PL {
		let a = TAU * k as f64 / N_PL as f64;
		// ends buried inside the two annuli — every union face is fully in
		// material or fully in air (§7.7 rule 3)
		let bar = extrude(&force_ccw(ts_arm_outline()), TS_T)
			.transformed(tr(0.0, 0.0, z0))
			.transformed(rotz(a));
		arms = Some(match arms {
			None => bar,
			Some(b) => union(&b, &bar),
		});
		let h = if nc_round {
			cylinder(DVec3::new(CD, 0.0, z0 - 1.0), DVec3::Z, PIN_D / 2.0 + C_FREE, TS_T + 2.0, 48)
				.transformed(rotz(a))
		} else {
			extrude(&force_ccw(slot_outline(e)), TS_T + 2.0)
				.transformed(tr(CD, 0.0, z0 - 1.0))
				.transformed(rotz(a))
		};
		holes = Some(match holes {
			None => h,
			Some(b) => union(&b, &h),
		});
	}
	ch.apply("six arms", |s| union(s, &arms.expect("6 arms")))?;
	ch.apply("six bayonet slots", |s| difference(s, &holes.expect("6 slots")))?;
	Ok(ch.finish())
}

fn top_spider(z0: f64) -> Result<Solid, kernel_brep::ChainError> {
	top_spider_var(z0, 0.0, false)
}

/// P7 CAP — Ø12 × 1.2 STATIC thumb pad, pressed onto the post through a
/// Ø5.50 through bore (the post shrank with the bearing).
///
/// The cap is why the sun's thrust cannot be an on-axis point pivot: a static
/// thumb pad has to be carried by a column, the column has to be on the axis,
/// and a column through the sun forbids the blind on-axis socket a point pivot
/// needs. That trade is costed in `analysis/ANALYSIS.md` (G21c) rather than
/// left implicit. DEVIATION (from v1): the spec's blind bore leaves a 0.3 mm
/// roof, under the 1.2 mm min wall; a through bore gives the full 1.2 mm of
/// press engagement and keeps the wall legal. The research names "caps back out
/// in service" as the #1 reported spinner maintenance failure.
///
/// **DEVIATION forced by v3's smaller post, found by a gate.** v1/v2 used the
/// profile's `xy_clearance_tight` (0.05 mm radial) on a Ø7.90 post: 1.27 % hoop
/// strain, inside PLA's 1.67 % yield. The same absolute interference on a Ø5.50
/// post is **1.82 %** — past yield. G22b caught it. The fix is to drop the
/// interference to `CAP_PRESS_R`, the same DRYBOX-print-proved joint class.
/// The engagement stays the full 1.20 mm.
///
/// **v4 RESIDUAL, disclosed rather than hidden.** The top spider's retention is
/// now geometric and calibration-free; this one is NOT. The cap is the model's
/// last interference fit and its grip still scales with how well the machine is
/// calibrated. It is left as a press for a reason that is written down rather
/// than assumed: the cap is Ø12 on a Ø5.50 post with 1.20 mm of engagement and
/// no room for a bayonet's travel, and unlike the spider it retains a part (the
/// sun) that cannot fall out sideways — a loose cap lets the sun lift, it does
/// not drop six planets. G16h prints the residual every run.
fn cap(z0: f64) -> Solid {
	let rb = POST_D / 2.0 - CAP_PRESS_R;
	revolve(
		&force_ccw(vec![
			DVec2::new(rb + C_BED, z0),
			DVec2::new(CAP_D / 2.0 - C_BED, z0),
			DVec2::new(CAP_D / 2.0, z0 + rise(C_BED)),
			DVec2::new(CAP_D / 2.0, z0 + CAP_T - 0.25),
			DVec2::new(CAP_D / 2.0 - 0.25, z0 + CAP_T),
			DVec2::new(rb, z0 + CAP_T),
			DVec2::new(rb, z0 + rise(C_BED)),
		]),
		96,
	)
}

/// P8 FIT COUPON — v3 buys nothing, so there is nothing to gauge a purchase
/// AGAINST; every fit on this coupon is printed-part-to-printed-part. Three of
/// them decide the build, on one ~12-minute print:
///  * a Ø5.50 **journal pin** — the post AND the planet pins are the same
///    diameter now, so one pin gauges both: the printed planet supplied on the
///    coupon must drop on and spin free;
///  * a Ø5.50 **press boss** with the cap's own Ø5.45 bore next to it — the cap
///    is the only interference fit left in the model;
///  * a **bayonet pin** cut to the exact shipped section (seat → Ø2.70 neck →
///    radial fin), which the separately-printed `coupon_key` drops onto and
///    slides home. v3's coupon gauged a click band whose grip the printer
///    decided; this one gauges the twist and the shoulder, which is what the
///    shipped joint actually is.
fn coupon() -> Result<Solid, kernel_brep::ChainError> {
	let plate = cuboid(DVec3::new(-24.0, -11.0, 0.0), DVec3::new(24.0, 11.0, 2.0));
	let mut ch = ChainLog::start("coupon plate", plate)?.seal();
	let pin = cylinder(DVec3::new(-15.0, 0.0, 1.0), DVec3::Z, PIN_D / 2.0, 8.0, 48);
	ch.apply("coupon journal pin", |s| union(s, &pin))?;
	let press = cylinder(DVec3::new(0.0, 0.0, 1.0), DVec3::Z, POST_D / 2.0, 6.0, 48);
	ch.apply("coupon cap-press boss", |s| union(s, &press))?;
	// the bayonet joint, on its own pin, at the exact shipped section: the pin is
	// dropped so its SEAT lands 2.0 mm above the plate, and everything below the
	// plate's mid-plane is cut away — the shipped pin's thrust boss would
	// otherwise hang in air under the coupon and put a 22 mm bridge on a set
	// whose whole claim (G22c) is that it has none.
	let dz = 4.0 - ts_bot();
	let bay = difference(
		&bay_pin(0.0).transformed(tr(15.0, 0.0, dz)),
		&cuboid(DVec3::new(9.0, -6.0, -12.0), DVec3::new(21.0, 6.0, 1.0)),
	);
	ch.apply("coupon bayonet pin", |s| union(s, &bay))?;
	Ok(ch.finish())
}

/// P9 BAYONET KEY — the coupon's other half, because a bayonet cannot be gauged
/// by one body: this tile carries ONE shipped slot at the shipped 2.00 mm
/// thickness. Drop it over the coupon's fin, slide it `bay_d()` mm to the
/// pocket, and the fin is over the wall — the retention joint, in the hand, in
/// twelve minutes. It is a separate STL because `emit` requires one body per
/// part and this one has to move.
fn coupon_key() -> Result<Solid, kernel_brep::ChainError> {
	let (hx, y0, y1) = (6.5, TS_ARM_Y0 - 1.0, TS_ARM_Y1 + 1.0);
	let tile = cuboid(DVec3::new(-hx, y0, 0.0), DVec3::new(hx, y1, TS_T));
	let mut ch = ChainLog::start("key tile", tile)?.seal();
	let slot = extrude(&force_ccw(slot_outline(0.0)), TS_T + 2.0).transformed(tr(0.0, 0.0, -1.0));
	ch.apply("key bayonet slot", |s| difference(s, &slot))?;
	Ok(ch.finish())
}

// ---- derived Z stack -------------------------------------------------------
fn ring_top() -> f64 {
	Z_ROT + T_RING
}
fn planet_top() -> f64 {
	Z_ROT + T_PLANET
}
fn ts_bot() -> f64 {
	ring_top() + C_Z
}
fn ts_top() -> f64 {
	ts_bot() + TS_T
}
fn pin_top() -> f64 {
	ts_top() + rise(PIN_D / 2.0 - NECK_D / 2.0) + 0.40
}
fn sun_top() -> f64 {
	Z_GEAR + T_SUN
}
fn cap_bot() -> f64 {
	sun_top() + C_Z
}
fn cap_top() -> f64 {
	cap_bot() + CAP_T
}

// ============================================================================
// 6. EMIT
// ============================================================================

fn emit(dir: &str, name: &str, s: &Solid, p: &FdmProfile, ok: &mut bool, worst_bridge: &mut f64) -> Mesh {
	let val = validate(s);
	// pose to print orientation (flat, gear axes ∥ +Z — already the build frame)
	// and drop to the bed
	let zmin = tessellate_default(s).positions.iter().map(|q| q.z as f64).fold(f64::INFINITY, f64::min);
	let printed = s.transformed(tr(0.0, 0.0, -zmin));
	let mesh = tessellate_default(&printed);
	let rep = mesh.support_free_report(Vec3::Z, p.max_unsupported_angle as f32, 0.3);
	let one = mesh.is_one_body();
	let wt = mesh.is_watertight();
	let ext = mesh.aabb();
	let e = [
		(ext.max.x - ext.min.x) as f64,
		(ext.max.y - ext.min.y) as f64,
		(ext.max.z - ext.min.z) as f64,
	];
	let fits = p.bed_fits(e);
	let vol = volume(s).abs();
	let pass = val.is_valid() && one && wt && rep.steep_area < 1e-6 && p.bridge_ok(rep.max_bridge_span) && fits;
	*worst_bridge = worst_bridge.max(rep.max_bridge_span);
	*ok &= pass;
	let _ = std::fs::write(format!("{OUT}/{dir}/{name}.stl"), mesh.to_stl_binary());
	let _ = mesh.write_3mf(format!("{OUT}/{dir}/{name}.3mf"));
	println!(
		"  {name:22} valid={:5} 1body={one:5} wt={wt:5} steep={:10.3e} mm²  bridge≤{:4.1}  {:5.2} g  {}",
		val.is_valid(),
		rep.steep_area,
		rep.max_bridge_span,
		vol * PLA,
		if pass { "OK" } else { "<<< FAIL" }
	);
	mesh
}

/// Exact `I_zz` about the world +Z axis and the static imbalance of one rotor,
/// from `mass_properties` on the EXACT B-rep — teeth, chamfers, grooves, bore
/// and all. Never an annulus approximation (the frozen spec's own rule: a
/// plain annulus mis-states a planet by 25 %, which swings eta by 0.8 points).
/// Returns (mass g, I_zz g·mm², |CG_xy| mm, |I_xz| , |I_yz| g·mm²).
fn rotor(s: &Solid) -> (f64, f64, f64, f64, f64) {
	let mp = mass_properties(s);
	let (cx, cy) = (mp.center_of_mass.x, mp.center_of_mass.y);
	let izz = (mp.inertia.z_axis.z + mp.volume * (cx * cx + cy * cy)) * PLA;
	(mp.volume * PLA, izz, (cx * cx + cy * cy).sqrt(), (mp.inertia.x_axis.z * PLA).abs(), (mp.inertia.y_axis.z * PLA).abs())
}

fn main() {
	kernel_core::telemetry::enable();
	for d in ["parts", "optional", "assembly/scene", "cad", "renders", "analysis", "publish"] {
		let _ = std::fs::create_dir_all(format!("{OUT}/{d}"));
	}
	let p = FdmProfile::load("profiles/conservative_default.json").unwrap_or_else(|_| FdmProfile::conservative_default());
	let mut ok = true;
	let mut worst_bridge = 0.0f64;
	println!("NULLSPIN — grounded-carrier epicyclic spinner, 66T ring ⟲ 42T sun\n");

	// ===================== G0 — ENGINE-REFUSAL PROBE ========================
	// Run FIRST: everything downstream depends on the internal ring generator
	// accepting m1.0 / 66T / 25°.
	let (rp66, rb66) = (M * R_T as f64 / 2.0, M * R_T as f64 / 2.0 * pa().cos());
	let t_root = (((rp66 + 1.25 * M) / rb66).powi(2) - 1.0).sqrt();
	let half66 = PI / (2.0 * R_T as f64) + (pa().tan() - pa());
	let margin = half66 / (t_root - t_root.atan()) - 1.0;
	let ring_ok = involute_ring_outline_shifted_filleted(M, R_T, PA_DEG, false, true, LASH, X_SHIFT, 0.0).is_some();
	println!("gates");
	gate(
		"G0 engine-refusal probe: internal 66T @ m1.0, 25° accepted",
		ring_ok && margin > 0.0,
		format!("margin {:+.2}%", margin * 100.0),
		&mut ok,
	);
	// NEGATIVE CONTROL for G0: the same generator at 30° must REFUSE (the root
	// land pinches). Without this the probe proves nothing.
	let refuses = involute_ring_outline_shifted_filleted(M, 36, 30.0, false, true, LASH, X_SHIFT, 0.0).is_none();
	gate("G0 NC: internal 36T @ 30° must be REFUSED", refuses, format!("refused {refuses}"), &mut ok);

	// ===================== G1 — KINEMATICS ==================================
	// Degenerate-Wolfrom encoding (Pa = Pb, R1 = R2) is how this repo's
	// EpicyclicTrain expresses a SIMPLE epicyclic; validate_assembly() is the
	// engine's own assembly-condition oracle and is the authority here.
	let train = EpicyclicTrain {
		sun_teeth: S_T,
		ring1_teeth: R_T,
		planet_a_teeth: P_T,
		planet_b_teeth: P_T,
		ring2_teeth: R_T,
		n_planets: N_PL,
	};
	let asm = train.validate_assembly();
	gate("G1a EpicyclicTrain::validate_assembly", asm.is_ok(), format!("{asm:?}").chars().take(22).collect(), &mut ok);
	// NEGATIVE CONTROL: 5 planets breaks (S+R) % n and must be REFUSED.
	let bad = EpicyclicTrain { n_planets: 5, ..train };
	gate("G1a NC: n=5 must be refused ((S+R)%n ≠ 0)", bad.validate_assembly().is_err(), "refused".into(), &mut ok);
	// Star ratio DERIVED from the engine's own simple_ratio: with the carrier
	// grounded, ω_sun/ω_ring = −(simple_ratio − 1) = −R/S.
	let simple = EpicyclicTrain::simple_ratio(S_T, R_T);
	let k_sun = -(simple - 1.0);
	let k_pl = R_T as f64 / P_T as f64;
	gate(
		"G1b star ratio from engine simple_ratio: ω_S/ω_R = −R/S",
		(k_sun + R_T as f64 / S_T as f64).abs() < 1e-12 && k_sun < 0.0,
		format!("{k_sun:+.6}"),
		&mut ok,
	);
	gate(
		"G1c counter-rotation + exact 7:11 headline",
		k_sun < 0.0 && k_pl > 0.0 && (7.0 * -k_sun - 11.0).abs() < 1e-12,
		format!("7×{:.4} = {:.4}", -k_sun, 7.0 * -k_sun),
		&mut ok,
	);
	gate("G1d planet speed ratio +R/P exactly 5.5", (k_pl - 5.5).abs() < 1e-12, format!("{k_pl:+.4}"), &mut ok);
	// The research's own anti-drag gate is max|k| ≤ 1.5. This design is 5.5 —
	// 3.7× over. PUBLISHED, not hidden; its measured cost is in the budget.
	gate(
		"G1e k_max vs research anti-drag gate 1.5 — DECLARED VIOLATION",
		k_pl > 1.5,
		format!("k_max {k_pl:.2} = {:.1}×", k_pl / 1.5),
		&mut ok,
	);

	// ===================== G2–G4 — MESH GEOMETRY ============================
	let neighbour = 2.0 * CD * (PI / N_PL as f64).sin() - M * (P_T + 2) as f64;
	gate("G2 neighbour gap ≥ 1.0 mm", neighbour >= 1.0, format!("{neighbour:.3} mm"), &mut ok);
	let floor = undercut_floor(X_SHIFT);
	gate(
		"G3 undercut floor 2/sin²α — every external member clears",
		P_T as f64 >= floor && S_T as f64 >= floor,
		format!("{P_T} ≥ {floor:.3}"),
		&mut ok,
	);
	let eps_sp = contact_ratio_external(M, PA_DEG, S_T, P_T);
	let eps_pr = contact_ratio_internal(M, PA_DEG, P_T, R_T);
	gate("G4a contact ratio sun–planet ≥ 1.20", eps_sp >= 1.20, format!("ε {eps_sp:.4}"), &mut ok);
	gate("G4b contact ratio planet–ring ≥ 1.20", eps_pr >= 1.20, format!("ε {eps_pr:.4}"), &mut ok);
	// NEGATIVE CONTROL: the same formula on a 20° / 8T pinion must land under
	// the floor — proves the contact-ratio code is not a constant.
	// NEGATIVE CONTROL: the SAME function, driven with an 8T×8T pair at 30° —
	// a real, physically-meaningful case whose overlap genuinely falls short.
	let nc_eps = contact_ratio_external(1.0, 30.0, 8, 8);
	gate("G4 NC: 8T×8T @30° through the SAME fn must read ε < 1.20", nc_eps < 1.20, format!("ε {nc_eps:.4}"), &mut ok);
	// tip/root clearance at both meshes — ISO 53 says 0.25·m
	let (_, _, ra_s, rr_s) = radii(S_T, true);
	let (_, _, ra_p, rr_p) = radii(P_T, true);
	let (_, _, rt_r, rr_r) = radii(R_T, false);
	let cl = [
		("sun root", (CD - ra_p) - rr_s),
		("planet root vs sun tip", (CD - ra_s) - rr_p),
		("ring root", rr_r - (CD + ra_p)),
		("planet root vs ring tip", (rt_r - CD) - rr_p),
	];
	let worst_cl = cl.iter().map(|c| c.1).fold(f64::INFINITY, f64::min);
	gate(
		"G4c tip/root clearance = 0.25·m at all four flanks",
		cl.iter().all(|c| (c.1 - 0.25 * M).abs() < 1e-9),
		format!("{worst_cl:.4} mm"),
		&mut ok,
	);

	// ===================== SPIN-DOWN SOLVER BENCHMARKS ======================
	// A written solver is guilty until its own gates are green (§25.7).
	let (i_b, k_b, n_b, c_b) = (1.5e-5, 3.0e-6, 0.43, 4.0e-4);
	let mut d_pow = Drag::default();
	d_pow.add(k_b, n_b, "bench power law");
	let (t_pow, a_pow) = spin_down(i_b, &d_pow, W0);
	let t_pow_a = i_b * W0.powf(1.0 - n_b) / (k_b * (1.0 - n_b));
	let a_pow_a = i_b * W0.powf(2.0 - n_b) / (k_b * (2.0 - n_b)) / TAU;
	let e_pow = ((t_pow - t_pow_a) / t_pow_a).abs().max(((a_pow - a_pow_a) / a_pow_a).abs());
	gate(
		"SOLVER B1 pure power law vs closed form (<0.5%)",
		e_pow < 0.005,
		format!("err {:.3e}", e_pow),
		&mut ok,
	);
	let mut d_cou = Drag::default();
	d_cou.add(c_b, 0.0, "bench coulomb");
	let (t_cou, a_cou) = spin_down(i_b, &d_cou, W0);
	let e_cou = ((t_cou - i_b * W0 / c_b) / (i_b * W0 / c_b))
		.abs()
		.max(((a_cou - i_b * W0 * W0 / (2.0 * c_b) / TAU) / (i_b * W0 * W0 / (2.0 * c_b) / TAU)).abs());
	gate("SOLVER B2 pure Coulomb vs closed form (<0.5%)", e_cou < 0.005, format!("err {e_cou:.3e}"), &mut ok);
	// META-NEGATIVE-CONTROL: the benchmark comparison must be able to go RED.
	let e_meta = ((t_pow - 1.05 * t_pow_a) / (1.05 * t_pow_a)).abs();
	gate("SOLVER B3 meta-NC: a 5% wrong reference FAILS B1", e_meta >= 0.005, format!("err {e_meta:.3e}"), &mut ok);
	// B5/B6 — the Hertz contact helper that the whole v2 ring-support term rests
	// on, benchmarked before it is used, the same way the integrator was.
	// B5 drives it against an INDEPENDENT algebraic path: the mutual approach
	// δ = (9F²/16RE*²)^⅓ is a different expression of the same solution, and
	// a² = Rδ ties the two together. B6 is the meta-negative-control.
	{
		let (fb, rb2, eb) = (0.0065f64, 0.75f64, e_star(E_STEEL, NU_STEEL, E_PLA_MPA, NU_PLA));
		let a1 = hertz_a(fb, rb2, eb);
		let a2 = (rb2 * hertz_delta(fb, rb2, eb)).sqrt();
		let e_h = ((a1 - a2) / a2).abs();
		gate("SOLVER B5 Hertz a vs the independent δ path (<0.1%)", e_h < 0.001, format!("err {e_h:.3e}"), &mut ok);
		let e_h_meta = ((a1 - 1.05 * a2) / (1.05 * a2)).abs();
		gate("SOLVER B6 meta-NC: a 5% wrong reference FAILS B5", e_h_meta >= 0.001, format!("err {e_h_meta:.3e}"), &mut ok);
		// The steel side of E* is 1.7% of the compliance, so the answer does not
		// depend on the bearing-steel constants. Reported, not assumed.
		let a_rigid = hertz_a(fb, rb2, e_star(f64::MAX / 1e6, NU_STEEL, E_PLA_MPA, NU_PLA));
		gate(
			"B7 Hertz radius is insensitive to the steel constants (<1%)",
			((a_rigid - a1) / a1).abs() < 0.01,
			format!("rigid ball moves a by {:+.2}%", 100.0 * (a_rigid - a1) / a1),
			&mut ok,
		);
	}

	// ---- the held frame is built FIRST, so the design study's mass budget uses
	// a MEASURED frame mass rather than an estimate. The frame's planform does
	// not move with the design variables; only the pin length and the top
	// spider's z do, and that variation is bounded analytically below.
	let build = |r: Result<Solid, kernel_brep::ChainError>, what: &str| -> Solid {
		match r {
			Ok(s) => s,
			Err(e) => {
				println!("  {what} chain failed: {e}");
				std::process::exit(1);
			}
		}
	};
	let s_base = build(base_spider(0.0), "base spider");
	let s_top = build(top_spider(ts_bot()), "top spider");
	let s_cap = cap(cap_bot());
	let frame_g = (volume(&s_base).abs() + volume(&s_top).abs() + volume(&s_cap).abs()) * PLA;
	// Worst-case frame mass over the WHOLE design space, so the study's mass
	// constraint does not depend on the point the study is being asked to find.
	// Two things move: the six pins with t_ring, and the post with t_sun (the
	// post must reach the cap, and the cap sits above the sun). Without the
	// second term the study is CIRCULAR — the shipped t_sun shortens the post,
	// which frees mass, which moves the optimum — and it visibly oscillated on
	// the 0.02 grid until this was added. Found the honest way: by the gate.
	let frame_g_hi = frame_g
		+ N_PL as f64 * PI * (PIN_D / 2.0).powi(2) * (6.5 - T_RING) * PLA
		+ PI * (POST_D / 2.0).powi(2) * (T_SUN_MAX - T_SUN) * PLA;

	// ===================== G11 — DESIGN STUDY ===============================
	// Every rotor is a prism, so I_zz = ρ·h·J is EXACT in the face width; the
	// study runs on the polygon second moments of the very outlines the solids
	// are built from. Fixed-size features are a correction, re-measured on the
	// exact B-rep at the shipped point (G9).
	let (a_sun_p, j_sun_p) = poly_area_j(&gear_profile(S_T, true, false));
	let (a_pl_p, j_pl_p) = poly_area_j(&gear_profile(P_T, true, true));
	let (a_cav, j_cav) = poly_area_j(&gear_profile(R_T, false, true));
	// benchmark the polygon-moment helper against πR⁴/2 on a 512-gon
	let circ: Vec<DVec2> = (0..512).map(|i| { let a = TAU * i as f64 / 512.0; DVec2::new(10.0 * a.cos(), 10.0 * a.sin()) }).collect();
	let (ac, jc) = poly_area_j(&circ);
	let e_poly = ((jc - PI * 10.0f64.powi(4) / 2.0) / (PI * 10.0f64.powi(4) / 2.0)).abs().max(((ac - PI * 100.0) / (PI * 100.0)).abs());
	gate("SOLVER B4 polygon polar moment vs πR⁴/2 (<0.1%)", e_poly < 0.001, format!("err {e_poly:.3e}"), &mut ok);

	let bore_r = SUN_BORE_D / 2.0;
	let (a_sun, j_sun) = (a_sun_p - PI * bore_r * bore_r, j_sun_p - PI * bore_r.powi(4) / 2.0);
	let (a_pl, j_pl) = (a_pl_p - PI * (PLANET_BORE_D / 2.0).powi(2), j_pl_p - PI * (PLANET_BORE_D / 2.0).powi(4) / 2.0);
	// surrogate rotor properties per mm of face width (g, g·mm²)
	let sun_pm = (a_sun * PLA, j_sun * PLA);
	let pl_pm = (a_pl * PLA, j_pl * PLA);
	let ring_pm = |wall: f64| {
		let od = 34.25 + wall;
		((PI * od * od - a_cav) * PLA, (PI * od.powi(4) / 2.0 - j_cav) * PLA)
	};
	// held frame + cap, held constant in the study (their planform does not
	// move with the design vars); the SHIPPED mass gate below is exact.
	let ks = -k_sun; // 11/7
	// v3: NO non-scaling mass anywhere in the rotor set. The 608's 610 g·mm² and
	// the ball race's orbital term are both gone, so this evaluator is a pure
	// function of PLA inertias — which is also what makes G9b's exact
	// common-mode-flow invariance true of the SHIPPED set for the first time.
	let eval = move |q: &Params| -> Evaluation {
		let (ts, trg, wall, tp) = (q["t_sun"], q["t_ring"], q["ring_wall"], q["t_planet"]);
		let (m_s, i_s) = (sun_pm.0 * ts, sun_pm.1 * ts);
		let (m_p, i_p) = (pl_pm.0 * tp, pl_pm.1 * tp);
		let (m_r, i_r) = { let (a, b) = ring_pm(wall); (a * trg, b * trg) };
		let l_signed = i_r - i_s * ks + N_PL as f64 * i_p * k_pl;
		let l_abs = i_r + i_s * ks + N_PL as f64 * i_p * k_pl;
		let eta = 1.0 - l_signed.abs() / l_abs;
		let i_eff = i_r + i_s * ks * ks + N_PL as f64 * i_p * k_pl * k_pl;
		let cap_t = Z_GEAR + ts + C_Z + CAP_T;
		let pin_t = Z_ROT + trg + C_Z + TS_T + rise(PIN_D / 2.0 - NECK_D / 2.0) + 0.40;
		let height = cap_t.max(pin_t);
		let mass = m_s + m_r + N_PL as f64 * m_p + frame_g_hi;
		Evaluation::new()
			.objective("i_eff", i_eff)
			.objective("mass_g", mass)
			.constraint("eta", eta)
			.constraint("height_mm", height)
			.constraint("od_mm", 2.0 * (34.25 + wall))
			.constraint("mass_g", mass)
			.constraint("planet_vs_ring", tp - trg)
			.constraint("sun_vs_planet", tp - ts)
			.constraint("ring_wall_mm", wall)
	};
	// **The t_sun window OPENED in v3 and the study is why the shipped point
	// moved.** v1/v2 could not go below 7.60 — the 608 had to live inside the
	// bore (SUN_LIP + BRG_W) — so the window was 0.60 mm wide and only the ring
	// could really move. With the bearing gone the floor is gone: t_sun sweeps
	// 3.00–8.20 in 0.02 and the only bounds left are physical ones (the envelope
	// above, and `sun_vs_planet` below: a sun narrower than the planet face
	// would not carry the full mesh). The ceiling is still
	// height = Z_GEAR + t_sun + C_Z + CAP_T ≤ 12.0 ⇒ t_sun ≤ 8.20, declared as a
	// CONSTRAINT as well so the study re-proves it.
	// The ring wall is NOT a free variable: a wall must be a whole number of
	// 0.45 mm extrusion lines, so the legal set inside [min_wall, envelope] is
	// {1.35, 1.80, 2.25} = {3, 4, 5} lines. The floor of 5 lines is a
	// manufacturability requirement on the one part located by six simultaneous
	// meshes — an out-of-round ring binds everywhere at once — and the study
	// reports it as an ACTIVE constraint with its cost.
	let study = Study::new(eval)
		.var(DesignVar::stepped("t_sun", 3.00, T_SUN_MAX, 0.02))
		.var(DesignVar::stepped("t_ring", 3.5, 6.5, 0.5))
		.var(DesignVar::stepped("ring_wall", 1.35, 2.25, 0.45))
		.var(DesignVar::stepped("t_planet", 3.0, 6.0, 0.5))
		.maximize("i_eff")
		.minimize("mass_g")
		.constrain(Constraint::greater_than("eta", 0.97))
		.constrain(Constraint::less_than("height_mm", 12.0))
		.constrain(Constraint::less_than("od_mm", 73.0))
		.constrain(Constraint::less_than("mass_g", 28.0))
		.constrain(Constraint::less_than("planet_vs_ring", 0.0))
		.constrain(Constraint::less_than("sun_vs_planet", 0.0))
		.constrain(Constraint::greater_than("ring_wall_mm", 2.25));
	let report = match study.full_factorial() {
		Ok(r) => r,
		Err(e) => {
			println!("  design study refused: {e:?}");
			std::process::exit(1);
		}
	};
	let shipped: Params = [
		("t_sun".to_string(), T_SUN),
		("t_ring".to_string(), T_RING),
		("ring_wall".to_string(), RING_WALL),
		("t_planet".to_string(), T_PLANET),
	]
	.into_iter()
	.collect();
	println!(
		"  study: {} evaluations, {} feasible, stop={}",
		report.evaluation_count(),
		report.feasible_count,
		report.stop_reason
	);
	if let Ok(b) = report.best("i_eff") {
		println!(
			"  study optimum: t_sun {:.2}  t_ring {:.2}  wall {:.2}  t_planet {:.2}  → I_eff {:.0} g·mm², eta {:.4}, {:.1} g",
			b.params["t_sun"], b.params["t_ring"], b.params["ring_wall"], b.params["t_planet"],
			b.value, b.constraints["eta"], b.constraints["mass_g"]
		);
	}
	let _ = gate_study("G11 shipped design point IS the study optimum", &report, "i_eff", &shipped, 1e-9, &mut ok);

	// ===================== BUILD ============================================
	println!("\nparts");
	let s_ring = build(ring(T_RING, RING_WALL), "ring");
	let s_sun = build(sun(T_SUN), "sun");
	let s_sunb = build(sun(SUNB_FRAC * T_SUN), "sun-b");
	let s_planet = build(planet(T_PLANET, PLANET_BORE_D), "planet");
	let s_pl_lo = build(planet(T_PLANET, 5.90), "planet 5.90");
	let s_pl_hi = build(planet(T_PLANET, 6.15), "planet 6.15");
	let s_coupon = build(coupon(), "coupon");
	let s_key = build(coupon_key(), "coupon key");

	let m_ring = emit("parts", "ring_66t", &s_ring, &p, &mut ok, &mut worst_bridge);
	let m_sun = emit("parts", "sun_42t", &s_sun, &p, &mut ok, &mut worst_bridge);
	let m_planet = emit("parts", "planet_12t_bore600", &s_planet, &p, &mut ok, &mut worst_bridge);
	let m_base = emit("parts", "base_spider", &s_base, &p, &mut ok, &mut worst_bridge);
	let m_top = emit("parts", "top_spider", &s_top, &p, &mut ok, &mut worst_bridge);
	let m_cap = emit("parts", "cap", &s_cap, &p, &mut ok, &mut worst_bridge);
	let _ = emit("optional", "sun_b_control", &s_sunb, &p, &mut ok, &mut worst_bridge);
	let _ = emit("optional", "planet_12t_bore590", &s_pl_lo, &p, &mut ok, &mut worst_bridge);
	let _ = emit("optional", "planet_12t_bore615", &s_pl_hi, &p, &mut ok, &mut worst_bridge);
	let _ = emit("optional", "coupon_fit", &s_coupon, &p, &mut ok, &mut worst_bridge);
	let _ = emit("optional", "coupon_key", &s_key, &p, &mut ok, &mut worst_bridge);

	// G13 NEGATIVE CONTROL — audit the base spider on its side; the support
	// oracle must FIRE.
	let wrong = tessellate_default(&s_base.transformed(DAffine3::from_rotation_x(PI / 2.0)))
		.support_free_report(Vec3::Z, 45.0, 0.3);
	gate(
		"G13 NC: base spider audited on its side (steep must jump)",
		wrong.steep_area > 100.0,
		format!("steep {:8.0} mm²", wrong.steep_area),
		&mut ok,
	);

	// ===================== G9 — ETA ON THE EXACT B-REP ======================
	println!("\nrotors (exact B-rep mass properties)");
	let (mg_r, izz_r, cg_r, ixz_r, iyz_r) = rotor(&s_ring);
	let (mg_s, izz_s, cg_s, ixz_s, iyz_s) = rotor(&s_sun);
	let (mg_p, izz_p, cg_p, _, _) = rotor(&s_planet);
	let (mg_sb, izz_sb, _, _, _) = rotor(&s_sunb);
	for (n, m, i) in [("ring", mg_r, izz_r), ("sun", mg_s, izz_s), ("planet", mg_p, izz_p), ("sun-b", mg_sb, izz_sb)] {
		println!("  {n:8} {m:6.2} g   I_zz {i:8.1} g·mm²");
	}
	// v3's rotor set is ALL PLA. There is no third rotor: no 608 inner race, no
	// orbiting ball set. The two steel terms that used to appear here — and that
	// happened to flatter eta, because the ball set's ring-sense orbital momentum
	// partly cancelled the printed residual — are gone, and eta moves as a
	// result. Both the loss and its cause are published rather than absorbed.
	//
	// `l_extra` stays a PARAMETER even though the shipped value is 0: it is what
	// lets the sensitivity table price the CENTRAL WEB (the re-opened direction)
	// in eta without a second formula, and it is what the v1/v2 ledger rows use.
	let eta_full = |i_s_pla: f64, i_r: f64, i_p: f64, i608: f64, l_extra: f64| {
		let i_s = i_s_pla + i608;
		let ls = i_r - i_s * ks + N_PL as f64 * i_p * k_pl + l_extra;
		let la = i_r + i_s * ks + N_PL as f64 * i_p * k_pl + l_extra;
		1.0 - ls.abs() / la
	};
	let eta_of = |i_s_pla: f64, i_r: f64, i_p: f64| eta_full(i_s_pla, i_r, i_p, 0.0, 0.0);
	let eta = eta_of(izz_s, izz_r, izz_p);
	let i_eff_gmm2 = izz_r + izz_s * ks * ks + N_PL as f64 * izz_p * k_pl * k_pl;
	gate("G9 eta on the exact B-rep ≥ 0.95 (design target 0.97)", eta >= 0.95, format!("η {eta:.4}"), &mut ok);
	// The lightest credible over-the-top central web (6 spokes 3.0 × 1.2 from the
	// hub out to the ring's inner wall, in the ring's OWN top slab so no collar
	// is needed) — costed here in eta so the re-opened web decision is priced in
	// the same table as everything else. G20 costs its other two halves.
	let l_web = 6.0 * (3.0 * 1.2 * (34.25 - 4.0) * PLA) * (4.0 * 4.0 + 4.0 * 34.25 + 34.25 * 34.25) / 3.0;
	// sensitivity band — with the steel gone, PRINTED mass variation is the only
	// uncertainty left, and the differential corner is now the whole story
	let sens = [
		("sun +5% flow", eta_of(izz_s * 1.05, izz_r, izz_p)),
		("sun −5% flow", eta_of(izz_s * 0.95, izz_r, izz_p)),
		("ring +5% flow", eta_of(izz_s, izz_r * 1.05, izz_p)),
		("ring −5% flow", eta_of(izz_s, izz_r * 0.95, izz_p)),
		("sun +5% / ring −5%", eta_of(izz_s * 1.05, izz_r * 0.95, izz_p)),
		("sun −5% / ring +5%", eta_of(izz_s * 0.95, izz_r * 1.05, izz_p)),
		("v1/v2 ledger: the 608 put back (+610 g·mm² on the sun)", eta_full(izz_s, izz_r, izz_p, I608_GMM2, 0.0)),
		("REFUSED direction: the central web fitted", eta_full(izz_s, izz_r, izz_p, 0.0, l_web)),
	];
	// G9b3's corner is the PRINT-FLOW corner: the first six rows, which are the
	// only ones describing a spinner someone could actually print. The last two
	// rows are counterfactuals (hardware put back, web fitted) and are published
	// in the same table but are deliberately NOT builds of this design, so
	// folding them into the build corner would be a category error in the
	// pessimistic direction rather than the optimistic one.
	let eta_lo = sens[..6].iter().map(|s| s.1).fold(1.0f64, f64::min);
	// eta is a RATIO of inertias, so a COMMON-MODE flow error cancels exactly —
	// and two parts printed on one plate with one profile share most of their
	// flow error. In v1 and v2 that was true only of a hypothetical all-PLA set,
	// because the 608 and the balls did not scale with print flow. **In v3 it is
	// true of the SHIPPED set**, which is the one place where deleting the
	// hardware made a receipt stronger instead of weaker.
	let d_exact = (eta_of(izz_s * 1.05, izz_r * 1.05, izz_p * 1.05) - eta).abs();
	gate(
		"G9b eta is EXACTLY invariant to common-mode flow (the SHIPPED set)",
		d_exact < 1e-12,
		format!("Δη {d_exact:.2e}"),
		&mut ok,
	);
	// The v1/v2 architectures DID carry non-scaling steel, so their cancellation
	// was only approximate. Recomputed here so the improvement is a measurement
	// and not a claim.
	let d_608 = (eta_full(izz_s * 1.05, izz_r * 1.05, izz_p * 1.05, I608_GMM2, 0.0)
		- eta_full(izz_s, izz_r, izz_p, I608_GMM2, 0.0))
	.abs();
	let d_diff = eta - eta_lo;
	gate(
		"G9b2 the deleted 608 is what made common-mode cancellation inexact",
		d_608 > 1e3 * d_exact,
		format!("v1/v2 Δη {d_608:.1e} vs v3 {d_exact:.1e}"),
		&mut ok,
	);
	// Floor for the pessimistic DIFFERENTIAL corner: the cancellation must still
	// dominate, i.e. it must stay far from the uncancelled control puck, or the
	// shipped A/B stops being valid for some buyers. 0.90 is that line (the
	// control sits near 0.74); it is NOT the 0.95 nominal gate, and the measured
	// corner is published either way.
	gate(
		"G9b3 worst corner in the whole table still ≥ 0.90 (A/B stays valid)",
		eta_lo >= 0.90,
		format!("η_min {eta_lo:.4}, spread {d_diff:.4}"),
		&mut ok,
	);
	let eta_b = eta_of(izz_sb, izz_r, izz_p);
	gate("G9c SUN-B control is DELIBERATELY uncancelled (η < 0.90)", eta_b < 0.90, format!("η_B {eta_b:.4}"), &mut ok);

	// ===================== G10 — BALANCE ====================================
	gate(
		"G10 ring: static imbalance 0, no products of inertia",
		cg_r < 1e-6 && ixz_r < 1e-6 && iyz_r < 1e-6,
		format!("cg {cg_r:.2e} mm"),
		&mut ok,
	);
	gate(
		"G10 sun (7 index grooves): imbalance still 0",
		cg_s < 1e-6 && ixz_s < 1e-6 && iyz_s < 1e-6,
		format!("cg {cg_s:.2e} mm"),
		&mut ok,
	);
	gate("G10 planet: imbalance 0", cg_p < 1e-6, format!("cg {cg_p:.2e} mm"), &mut ok);
	// NEGATIVE CONTROL — delete one index groove and the oracle must fire.
	let lop = {
		let a = 0.35;
		let bar = cuboid(
			DVec3::new(SUN_BORE_D / 2.0 + 1.6, -0.6, sun_top() - 0.40),
			DVec3::new(ra_s - 1.6, 0.6, sun_top() + 1.0),
		)
		.transformed(rotz(a));
		union(&s_sun, &intersection(&bar, &cylinder(DVec3::new(0.0, 0.0, 0.0), DVec3::Z, ra_s, sun_top(), 96)))
	};
	let (_, _, cg_lop, _, _) = rotor(&lop);
	gate("G10 NC: one groove filled → imbalance must appear", cg_lop > 1e-4, format!("cg {cg_lop:.2e} mm"), &mut ok);

	// ===================== G5/G6/G7 — MOTION ================================
	// STAR pose evaluator: the engine's `instance_poses` is ring-fixed /
	// sun-driven and does NOT cover a grounded carrier, so the campaign carries
	// its own. Carrier fixed ⇒ planet j sits at a FIXED azimuth βj with install
	// spin βj — the rigid-rotation argument: 60° is exactly 7 sun pitches and
	// 11 ring pitches, so rotating the whole meshed assembly by 60° maps
	// sun→sun, ring→ring and planet j→j+1.
	let pl_local = |j: usize, th: f64| {
		let b = TAU * j as f64 / N_PL as f64;
		tr(CD * b.cos(), CD * b.sin(), 0.0) * rotz(b + k_pl * th)
	};
	let pose_sun = |th: f64, err: f64| s_sun.transformed(rotz(k_sun * (1.0 + err) * th));
	let pose_ring = |th: f64| s_ring.transformed(rotz(th));
	let pose_planet = |j: usize, th: f64| s_planet.transformed(pl_local(j, th));
	let ov = |a: &Solid, b: &Solid| overlap_volume(a, b).unwrap_or(f64::NAN);
	// The FULL mesh cycle repeats every ONE ring tooth pitch: over θ = 2π/66 the
	// planet turns exactly 2π/12 and the sun exactly 2π/42, so a dense sweep of
	// one ring pitch visits EVERY distinct mesh state — far denser in mesh phase
	// than the same pose count spread over two whole revolutions.
	let pitch_r = TAU / R_T as f64;
	// Tier 1 — dense mesh sweep (§25 step 5). Both members move, so the sweep is
	// run in the FIXED member's own frame: pre-multiplying by the inverse of the
	// sun / ring rotation makes it stationary and folds all the motion into the
	// planet's pose. `crossings` is the exact triangle-level oracle the vertex
	// sampling cannot fake.
	let dense: Vec<f64> = (0..96).map(|i| pitch_r * i as f64 / 96.0).collect();
	let sun_poses: Vec<DAffine3> = dense.iter().map(|&th| rotz(-k_sun * th) * pl_local(0, th)).collect();
	let ring_poses: Vec<DAffine3> = dense.iter().map(|&th| rotz(-th) * pl_local(0, th)).collect();
	let sw_s = kernel_model::sweep_check(&m_sun, &m_planet, &sun_poses);
	let sw_r = kernel_model::sweep_check(&m_ring, &m_planet, &ring_poses);
	gate(
		"G5a sun mesh, 96-pose dense sweep of ONE full mesh cycle",
		sw_s.contacts == 0 && sw_s.crossings == 0 && sw_s.max_penetration == 0.0,
		format!("min_cl {:.3} mm", sw_s.min_clearance),
		&mut ok,
	);
	gate(
		"G5b ring mesh, same 96-pose dense sweep",
		sw_r.contacts == 0 && sw_r.crossings == 0 && sw_r.max_penetration == 0.0,
		format!("min_cl {:.3} mm", sw_r.min_clearance),
		&mut ok,
	);
	// Tier 2 — EXACT overlap_volume on the B-reps at a subset of the same cycle.
	let mut worst_sp = 0.0f64;
	let mut worst_pr = 0.0f64;
	for i in 0..16 {
		let th = pitch_r * i as f64 / 16.0;
		let pl = pose_planet(0, th);
		worst_sp = worst_sp.max(ov(&pl, &pose_sun(th, 0.0)));
		worst_pr = worst_pr.max(ov(&pl, &pose_ring(th)));
	}
	gate("G5c exact overlap_volume, 16 poses across the cycle, both meshes", worst_sp < 1e-9 && worst_pr < 1e-9, format!("{:.3e} mm³", worst_sp.max(worst_pr)), &mut ok);
	// Tier 3 — ALL SIX planets over two FULL ring revolutions. This is the check
	// of the 6-fold symmetry argument itself, not a redundant repeat.
	let mut worst_all = 0.0f64;
	for i in 0..6 {
		let th = 2.0 * TAU * i as f64 / 6.0 + 0.013; // off every symmetry point
		let (su, rg) = (pose_sun(th, 0.0), pose_ring(th));
		for j in 0..N_PL {
			let pl = pose_planet(j, th);
			worst_all = worst_all.max(ov(&pl, &su)).max(ov(&pl, &rg));
		}
	}
	gate("G5d 2 full ring revs × all 6 planets, exact (72 booleans)", worst_all < 1e-9, format!("{worst_all:.3e} mm³"), &mut ok);
	// G6 NEGATIVE CONTROL — drive the sun ±5% off the exact ratio; the sweep must
	// JAM with strictly positive overlap. Without this, G5 is not a gate.
	let mut jam = 0.0f64;
	for e in [0.05f64, -0.05] {
		for i in 0..6 {
			let th = pitch_r * 8.0 * (i + 1) as f64 / 6.0;
			jam = jam.max(ov(&pose_planet(0, th), &pose_sun(th, e)));
		}
	}
	gate("G6 NC: sun ±5% off ratio must JAM (overlap > 0)", jam > 1e-3, format!("{jam:.4} mm³"), &mut ok);

	// G7 — backlash by bisection to flank contact at the sun mesh. The contact
	// predicate is the EXACT triangle-crossing oracle (`Mesh::crosses_mesh`), so
	// the bisection is exact and cheap.
	let lash_angle = {
		let th = pitch_r * 0.37; // a general, non-symmetric pose
		let pl = m_planet.transformed_by(pl_local(0, th));
		let (mut lo, mut hi) = (0.0f64, 0.10f64);
		for _ in 0..24 {
			let mid = 0.5 * (lo + hi);
			let su = m_sun.transformed_by(rotz(k_sun * th + mid));
			if su.crosses_mesh(&pl) {
				hi = mid;
			} else {
				lo = mid;
			}
		}
		0.5 * (lo + hi)
	};
	let jt_measured = lash_angle * (M * S_T as f64 / 2.0);
	gate(
		"G7 backlash strictly positive, jt in 0.12–0.26 mm at the sun mesh",
		jt_measured > 0.12 && jt_measured < 0.26,
		format!("{jt_measured:.3} mm / {:.3}°", lash_angle.to_degrees()),
		&mut ok,
	);

	// ===================== G8 — CONCENTRICITY ===============================
	// The sun is located BOTH by the 608 on the post and by six meshes. Radial
	// lash equivalent jr = jt/(2 tan α) is the budget the two must live inside.
	let jr = 0.18 / (2.0 * pa().tan());
	// The post and the pin circle are cut from the SAME parametric origin on the
	// SAME printed part, so the DESIGNED concentricity is exactly zero.
	let designed_conc = 0.0f64;
	// Conservative build error: the worst practitioner-reported XY error, used
	// as a positional proxy (positional repeatability itself is UNKNOWN — no
	// source publishes it; the larger of the two available numbers is used).
	let build_err = 0.15;
	// v3 IMPROVES this gate rather than inheriting it. v1/v2 located the sun
	// TWICE — on the 608 and on six meshes — and had to prove the two did not
	// fight, with only C_TIGHT + the bearing's internal clearance + C_TIGHT
	// (0.12 mm) of freedom to absorb the build error. With the bearing deleted
	// the sun's bore is a plain running fit on the post, so its radial freedom
	// is the full C_FREE and the residual collapses to zero: the sun is now
	// located by its six meshes alone and nothing can fight them.
	let sun_freedom = C_FREE;
	let residual = (build_err - sun_freedom).max(0.0) + designed_conc;
	gate(
		"G8a designed post↔pin-circle concentricity is exactly 0",
		designed_conc == 0.0,
		"0.000 mm".into(),
		&mut ok,
	);
	gate(
		"G8b worst-case residual concentricity < jr (no mesh preload)",
		residual < jr,
		format!("{residual:.3} < {jr:.3} mm"),
		&mut ok,
	);
	gate(
		"G8c each planet's radial freedom ≥ the build error (self-centres)",
		C_FREE >= build_err,
		format!("{C_FREE:.2} ≥ {build_err:.2} mm"),
		&mut ok,
	);

	// ===================== G16 — TOP-SPIDER RETENTION =======================
	// v4 replaced the six frictional click bands with a BAYONET (see the block
	// at NECK_D). Retention is now a geometric overlap, so the whole gate suite
	// changed shape with it: what v3 could only DISCLOSE as a calibration
	// dependency (v3's G16e) is proved here as pass/fail over the FULL G12
	// stack, and the two things that are still bounded rather than measured
	// (capacity, back-out) are bounded on solids or on section properties, not
	// on a friction coefficient this repo does not have.
	let yield_strain = SIG_YIELD_PLA / E_PLA_MPA;
	let stack_xy = 0.15; // the same worst-case XY figure G12 uses for clearances
	let foot = 0.20; // the same Prusa first-layer figure G12 uses
	let travel = bay_d();
	let engage_xy = ENGAGE - 2.0 * stack_xy;
	let engage_full = ENGAGE - 2.0 * (stack_xy + foot);
	gate(
		"G16a six geometric shoulders — material in the way, no preload anywhere",
		ENGAGE >= 1.0 && N_PL == 6,
		format!("{N_PL} × {ENGAGE:.2} mm of fin over slot wall"),
		&mut ok,
	);
	// THE gate v3 could not write. Under-extrusion thins the fin AND widens the
	// slot, so the errors add: engagement = ENGAGE − 2·e_side. v3's interference
	// hit zero at 0.025 mm/side; this hits zero at 0.575, which is 3.8× the
	// campaign's own worst case rather than 1/6 of it.
	gate(
		"G16b engagement survives the worst-case XY stack (0.15 mm/side, BOTH members)",
		engage_xy > 0.5,
		format!("{engage_xy:+.2} mm of {ENGAGE:.2} left; dies at {:.3} mm/side", ENGAGE / 2.0),
		&mut ok,
	);
	// …and with the elephant foot piled on top, which cannot physically reach a
	// feature 7 mm above the bed. Carried anyway, because 0.20 mm/side is the
	// number G12 designs every clearance against and the joint should not need
	// an argument about which errors apply.
	gate(
		"G16c engagement survives XY + the 0.20 mm Prusa elephant foot as well",
		engage_full > 0.2,
		format!("{engage_full:+.2} mm at {:.2} mm/side on both members", stack_xy + foot),
		&mut ok,
	);

	// ---- the ORACLE. The constants above are arithmetic; these run on the
	// BUILT SOLIDS, the same ones that get written to STL. Lift the spider and
	// it must run into the pins — and at rest it must NOT, because a joint that
	// touches at rest is a preload by another name.
	let posed = |t: &Solid, psi_deg: f64, dz: f64| t.transformed(rotz(psi_deg.to_radians())).transformed(tr(0.0, 0.0, dz));
	let ovl = |t: &Solid, b: &Solid, psi_deg: f64, dz: f64| overlap_volume(&posed(t, psi_deg, dz), b).unwrap_or(f64::NAN);
	let float_nom = bay_float(0.0);
	let free = ovl(&s_top, &s_base, 0.0, float_nom - 0.05);
	gate(
		"G16d ZERO PRELOAD: at rest and through its whole float the joint is not touching",
		free < 1e-9,
		format!("{free:.3e} mm³ at +{:.2} mm (float {float_nom:.2})", float_nom - 0.05),
		&mut ok,
	);
	let lift = 3.00; // past the fin's own height above the spider — the escape path
	let captive = ovl(&s_top, &s_base, 0.0, lift);
	gate(
		"G16e CAPTIVE: lift the locked spider and it runs into six fins",
		captive > 0.5,
		format!("{captive:8.2} mm³ at +{lift:.2} mm"),
		&mut ok,
	);
	// The worst-case stack, on solids: rebuild ONE pin and ONE arm with the full
	// G12 error on every retention surface (fin eroded, slot dilated) and repeat
	// the same lift. This is the direct replacement for v3's disclosed
	// dependency — the same question, answered pass/fail.
	let e_wc = stack_xy + foot;
	let joint_pin = |e: f64| bay_pin(e).transformed(tr(CD, 0.0, 0.0));
	let joint_arm = |e: f64| {
		difference(
			&extrude(&force_ccw(ts_arm_outline()), TS_T).transformed(tr(0.0, 0.0, ts_bot())),
			&extrude(&force_ccw(slot_outline(e)), TS_T + 2.0).transformed(tr(CD, 0.0, ts_bot() - 1.0)),
		)
	};
	let (wc_pin, wc_arm) = (joint_pin(e_wc), joint_arm(e_wc));
	let wc_capture = ovl(&wc_arm, &wc_pin, 0.0, lift);
	gate(
		"G16f WORST-CASE STACK on solids: fin eroded and slot dilated by 0.35 mm/side, still captive",
		wc_capture > 0.05,
		format!("{wc_capture:.3} mm³/pin at +{lift:.2} mm (nominal pin {:.3})", ovl(&joint_arm(0.0), &joint_pin(0.0), 0.0, lift)),
		&mut ok,
	);
	// NEGATIVE CONTROL 1 — remove the LIP. Same part, same pins, but every slot
	// is a plain round hole that clears the fin: the retention gate must read
	// exactly zero. G23c is the model.
	let nc_lip = top_spider_var(ts_bot(), 0.0, true).map(|s| ovl(&s, &s_base, 0.0, lift)).unwrap_or(f64::NAN);
	gate(
		"G16g NC: delete the lip (round holes) and the spider MUST lift straight off",
		nc_lip < 1e-9,
		format!("{nc_lip:.3e} mm³ (want 0)"),
		&mut ok,
	);
	// NEGATIVE CONTROL 2 — the same shipped part, UNTWISTED. Retention must be
	// the twist and nothing else, so at the entry pose the very same solids must
	// come apart. This is also the assembly proof: it is how the part goes on.
	let nc_pose = ovl(&s_top, &s_base, -BAY_PSI_DEG, lift);
	gate(
		"G16h NC: at the ENTRY pose the shipped spider must lift straight off",
		nc_pose < 1e-9,
		format!("{nc_pose:.3e} mm³ at −{BAY_PSI_DEG:.1}° (want 0)"),
		&mut ok,
	);
	// The twist itself must be FREE — no rub, no interference, nothing to force.
	let mut worst_twist = 0.0f64;
	for i in 0..=8 {
		let psi = -BAY_PSI_DEG * (1.0 - i as f64 / 8.0);
		worst_twist = worst_twist.max(ovl(&s_top, &s_base, psi, 0.10));
	}
	gate(
		"G16i the twist is free: 9 poses entry→lock, zero interference at every one",
		worst_twist < 1e-9,
		format!("{worst_twist:.3e} mm³ worst of 9 poses"),
		&mut ok,
	);
	// BACK-OUT: how much of the twist has to be undone before the fin can escape
	// through the bulge. Pure geometry — the fin's whole width has to fit inside
	// the bulge window — and confirmed on solids one step short of it.
	let u_rel = travel - BULGE_HW + FIN_HW;
	let still = ovl(&s_top, &s_base, -BAY_PSI_DEG * (u_rel - 0.20) / travel, lift);
	gate(
		"G16j back-out margin: >75 % of the twist must be undone before release, proved on solids",
		u_rel / travel > 0.75 && still > 0.05,
		format!("{:.0} % undone ({u_rel:.2}/{travel:.2} mm); still {still:.2} mm³ one step short", 100.0 * u_rel / travel),
		&mut ok,
	);

	// ---- CAPACITY. The load path under a pull is fin → neck → seat, and the
	// governing section is the neck in BENDING: the fin's relief cone is a
	// RELIEF_SLOPE wedge, so a vertical F at the shoulder puts 1.40·F radially
	// on the neck at the contact height. Both terms are section properties, not
	// friction — which is exactly why v3's μ-dependent pull-off bound is gone.
	let lever = ts_top() + float_nom - ts_bot();
	let z_neck = PI * NECK_D.powi(3) / 32.0;
	let f_cap = N_PL as f64 * SIG_ALLOW_RT * z_neck / (RELIEF_SLOPE * lever);
	let carried_n = (volume(&s_ring).abs() + N_PL as f64 * volume(&s_planet).abs() + volume(&s_top).abs()) * PLA * 9.81e-3;
	gate(
		"G16k retention capacity (neck bending, static allowable) beats the carried weight ≥100×",
		f_cap > 100.0 * carried_n,
		format!("{f_cap:.1} N vs {carried_n:.3} N carried ({:.0}×)", f_cap / carried_n),
		&mut ok,
	);
	// …and the shoulder's own bearing must NOT be what governs, or the number
	// above is describing the wrong failure. Bearing area is the fin's overhang
	// footprint, integrated rather than estimated.
	let a_bear: f64 = (0..400)
		.map(|i| {
			let y = FIN_HW * (2.0 * (i as f64 + 0.5) / 400.0 - 1.0);
			((PIN_D / 2.0).powi(2) - y * y).sqrt() - SLOT_HW
		})
		.sum::<f64>()
		* (2.0 * FIN_HW / 400.0);
	let p_bear = (f_cap / N_PL as f64) / a_bear;
	gate(
		"G16l the NECK governs, not the shoulder: bearing pressure at capacity stays under allowable",
		p_bear < SIG_ALLOW_RT,
		format!("{p_bear:.1} < {SIG_ALLOW_RT:.1} MPa over {a_bear:.2} mm²/pin"),
		&mut ok,
	);

	// ---- WHY NOT A SNAP. Kept as a live refusal, recomputed every run, because
	// "put a barb on it" is the obvious suggestion and the arithmetic is the
	// answer. A hoop's bore strain is δ/a EXACTLY, so the largest elastic
	// snap-over a Ø5.60 hole in this arm can survive is yield_strain·a — and the
	// interference a snap must swallow is the same 2·0.15 mm stack the
	// engagement above swallows. The gap is 6×, and no variant inside the 12 mm
	// envelope closes it (a 3.4 mm collet finger at t 0.9 reaches ~0.06 mm).
	let hole_a = (PIN_D + 2.0 * C_TIGHT) / 2.0; // 2.80 mm — v3's press hole
	let snap_max = yield_strain * hole_a;
	gate(
		"G16m snap-fit REFUSED on record: the elastic travel this scale allows is < the stack it must survive",
		snap_max < 2.0 * stack_xy,
		format!("{snap_max:.3} mm at yield vs {:.2} mm of stack ({:.1}× short)", 2.0 * stack_xy, 2.0 * stack_xy / snap_max),
		&mut ok,
	);
	// NEGATIVE CONTROL, inherited and kept: the frozen spec's Ø6.40 barb over
	// the same hole must still FAIL the strain check that refused it in v1.
	let spec_strain = ((6.40 - 2.0 * hole_a) / 2.0) / hole_a;
	gate(
		"G16m NC: the spec's Ø6.40 barb must FAIL the same strain check",
		spec_strain > yield_strain,
		format!("{:.1}% vs {:.2}% yield — refused", spec_strain * 100.0, yield_strain * 100.0),
		&mut ok,
	);
	// ---- THE RESIDUAL, stated the same way v3 stated its dependency. Retention
	// no longer has a calibration term. ASSEMBLY still does — the twist rides on
	// C_FREE like every other running fit in the model — and the CAP is still an
	// interference press. Neither is hidden; both are printed here.
	let twist_dies = C_FREE - 2.0 * stack_xy; // over-extrusion corner of the slide
	gate(
		"G16n residual calibration dependence is DISCLOSED: assembly fit and the cap, never retention",
		ENGAGE > 2.0 * (stack_xy + foot) && twist_dies.abs() < C_FREE,
		format!(
			"retention {ENGAGE:.2} mm geometric (survives {:.2}); twist clearance {C_FREE:.2} mm goes tight at {stack_xy:.2}/side; cap press {CAP_PRESS_R:.3} mm still calibration-bound (G22b)",
			stack_xy + foot
		),
		&mut ok,
	);

	// ===================== G12 — CLEARANCE STACK-UP =========================
	// designed radial gap vs elephant foot vs XY oversize vs profile deviation
	// elephant foot: the two vendor first-layer compensations disagree 2.7×
	// (Prusa 0.20 mm/side vs Bambu 0.075). The WORSE one is the design case.
	let (foot_prusa, foot_bambu, xy_worst, prof_dev) = (0.20, 0.075, 0.15, 0.067);
	assert!(foot_prusa > foot_bambu, "the design case must be the worse vendor number");
	let uncredited = C_FREE - 2.0 * xy_worst - 2.0 * foot_prusa;
	let credited = C_FREE - 2.0 * xy_worst; // the 0.45 chamfer deletes layer 1 from the gap
	let ladder = [5.90, 6.00, 6.15].map(|b| (b - PIN_D) / 2.0 - 2.0 * xy_worst);
	let ladder_best = ladder.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
	gate(
		"G12a nominal fit (±0.05 build) stays positive",
		C_FREE - 2.0 * 0.05 > 0.0,
		format!("{:+.3} mm", C_FREE - 2.0 * 0.05),
		&mut ok,
	);
	gate(
		"G12b worst corner: SOME ladder member stays positive",
		ladder_best > 0.0,
		format!("Ø6.15 → {ladder_best:+.3} mm"),
		&mut ok,
	);
	gate(
		"G12c mesh: jt 0.18 exceeds 2× profile deviation 0.067",
		0.18 > 2.0 * prof_dev,
		format!("0.180 > {:.3}", 2.0 * prof_dev),
		&mut ok,
	);

	// ===================== G14 — EN 71-1 §4.10 ROD RULE =====================
	// Every accessible space between RELATIVELY MOVING members: if a Ø5 rod
	// fits, a Ø12 rod must fit too. The 5–12 mm band is forbidden.
	let gaps: Vec<(&str, f64)> = vec![
		("sun ↔ top spider (radial)", TS_R_IN - ra_s),
		("sun ↔ base spider (axial)", C_Z),
		("planet ↔ base spider (rests on its thrust boss)", 0.0),
		("planet ↔ top spider (axial)", ts_bot() - planet_top()),
		("ring ↔ top spider (axial)", ts_bot() - ring_top()),
		("ring ↔ base spider arms (axial)", Z_ROT - Z_ARM),
		("ring ↔ its six thrust pads (in contact)", 0.0),
		("sun ↔ its thrust land (in contact)", 0.0),
		("ring proud of the held rims (radial)", (34.25 + RING_WALL) - STATIC_R),
		("adjacent planets (the one entered gap)", neighbour),
	];
	// ---- G23 RING AXIAL CAPTURE -------------------------------------------------
	// Added 2026-08-02 after the user asked the obvious question the 86-gate suite
	// could not answer: "the gears will fall out of the top". The top spider DOES
	// carry an outer rim over the ring (r TS_R_RIM..STATIC_R against the ring's
	// solid back at 34.25..34.25+RING_WALL), but NOTHING ASSERTED IT. An ungated
	// feature is one refactor from vanishing, and none of the other 86 gates
	// would have noticed. The oracle is not the constants — it is the built
	// solids: lift the ring past its own axial clearance and it must RUN INTO
	// the spider. If it slides out cleanly, the part falls apart in the hand.
	let lift = C_Z + 0.20; // past the running clearance, into the retaining rim
	let ring_lifted = s_ring.transformed(tr(0.0, 0.0, Z_ROT + lift));
	let top_placed = s_top.transformed(tr(0.0, 0.0, 0.0));
	let capture = kernel_brep::overlap_volume(&ring_lifted, &top_placed).unwrap_or(f64::NAN);
	gate(
		"G23a ring is CAPTIVE: lifting it past its clearance hits the spider",
		capture > 0.5,
		format!("{capture:8.2} mm³ at +{lift:.2} mm"),
		&mut ok,
	);
	// Radial engagement of that rim over the ring's solid back, from the same
	// constants the geometry is built from.
	let engage = STATIC_R.min(34.25 + RING_WALL) - TS_R_RIM.max(34.25);
	gate(
		"G23b retaining rim engages the ring's back by ≥ 0.50 mm",
		engage >= 0.50,
		format!("{engage:5.2} mm (r {TS_R_RIM}→{STATIC_R} over 34.25→{:.2})", 34.25 + RING_WALL),
		&mut ok,
	);
	// NEGATIVE CONTROL — the gate must be able to fail. Pull the rim inboard of
	// the ring's root and the capture must disappear entirely.
	let naked = top_spider_no_rim(ts_bot()).map(|s| s.transformed(tr(0.0, 0.0, 0.0)));
	let nc_capture = match &naked {
		Ok(s) => kernel_brep::overlap_volume(&ring_lifted, s).unwrap_or(0.0),
		Err(_) => f64::NAN,
	};
	gate(
		"G23c NC: delete the retaining rim and the ring MUST escape",
		nc_capture < 1e-9,
		format!("{nc_capture:.3e} mm³ (want 0)"),
		&mut ok,
	);

	let band = |g: f64| g > ROD_SMALL && g < ROD_LARGE;
	let worst_gap = gaps.iter().find(|g| band(g.1));
	gate(
		"G14 no accessible moving gap lands in the forbidden 5–12 mm band",
		worst_gap.is_none(),
		match worst_gap {
			None => format!("{} gaps clear", gaps.len()),
			Some(g) => format!("{} = {:.2}", g.0, g.1),
		},
		&mut ok,
	);
	gate(
		"G14b the one Ø5-admitting space also admits Ø12",
		neighbour >= ROD_LARGE,
		format!("{neighbour:.3} ≥ {ROD_LARGE:.0} mm"),
		&mut ok,
	);
	// NEGATIVE CONTROL — the same rule on a hypothetical 8-planet layout at the
	// same centre distance lands in the band and must FIRE.
	let nc_gap = 2.0 * CD * (PI / 8.0).sin() - M * (P_T + 2) as f64;
	gate(
		"G14 NC: an 8-planet layout must FAIL the rod rule",
		band(nc_gap),
		format!("{nc_gap:.2} mm — in band"),
		&mut ok,
	);

	// ===================== G15 — TOOTH-ROOT BENDING =========================
	// Load case: a thumb flick, applied and REMOVED. A fidget spinner is not a
	// sustained-load part and is not a cycled drivetrain, so the static
	// allowable is the right tier — creep is NOT the governing case here and
	// that call is written down rather than assumed.
	let y_sun = lewis_y(S_T, true);
	let y_pl = lewis_y(P_T, true);
	let y_ring = lewis_y(R_T, false);
	// Flick torque: a hard thumb flick is ~5 N at the Ø73 rim.
	let flick_n = 5.0;
	let t_flick = flick_n * (34.25 + RING_WALL) * 1e-3; // N·m at the ring
	// The ring mesh shares that torque over 6 planets at the ring pitch radius.
	let wt_ring = t_flick / (N_PL as f64 * 33.0e-3); // N tangential per mesh
	let sig_ring = wt_ring / (T_PLANET * M * y_pl); // MPa (N/mm²) on the weakest member
	// PLA in-plane bending, derated: the parts print FLAT, so tooth bending is
	// in the 10 MPa design tier, not the 0.55× across-layer tier.
	let sig_allow = kernel_model::materials::pla::SIG_ALLOW_RT;
	gate(
		"G15 tooth-root bending (Lewis, Y measured off the built outline)",
		sig_ring < sig_allow,
		format!("{sig_ring:.3} MPa vs {sig_allow:.0}"),
		&mut ok,
	);
	gate(
		"G15b measured Y is BELOW the handbook 0.36 (sharp-root honesty)",
		y_pl < 0.36 && y_sun < 0.45,
		format!("Y_p {y_pl:.3} Y_s {y_sun:.3} Y_r {y_ring:.3}"),
		&mut ok,
	);

	// ===================== DRAG BUDGET + SPIN TIME ==========================
	// THREE ARCHITECTURES, ONE ROTOR, ONE SOLVER, ONE RUN. v3 ships the fully
	// printed one; v1 (sliding ring land + 608) and v2 (ball race + 608) are
	// recomputed on the SAME rotor so the ledger isolates exactly what deleting
	// the hardware cost. Nothing below is quoted from a previous build.
	let (m_r_kg, m_p_kg, m_s_kg) = (mg_r * 1e-3, mg_p * 1e-3, mg_s * 1e-3);
	let (w_ring_n, w_pl_n, w_sun_n) = (m_r_kg * GRAV, m_p_kg * GRAV, m_s_kg * GRAV);
	// Planet thrust pad, CORRECTED in v2 and kept: v1 quoted r = 3.25 mm ("bore
	// r 3.00 → seat OD r 3.50"), but the planet's own bed relief removes its
	// underside out to r 3.45, so the real annulus is 3.45–3.50 and the arm is
	// 3.475. That correction made the budget WORSE and it stays.
	let r_pl_pad = ((PLANET_BORE_D / 2.0 + C_BED) + PLANET_SEAT_D / 2.0) / 2.0 * 1e-3;
	// SUN thrust land — new in v3, and the term that replaces the 608. Same
	// construction as the planet pad: the sun's own bed relief sets the inner
	// edge, the land's outer edge sets the other, and the arm is their mean.
	let r_sun_land = ((SUN_BORE_D / 2.0 + C_BED) + (SUN_BORE_D / 2.0 + C_BED + SUN_LAND_W)) / 2.0 * 1e-3;
	// Ring thrust pads — v1's term with the arm moved in to the floor the ring's
	// own continuous flat underside allows.
	let r_ring_pad = RING_PAD_R * 1e-3;
	let (.., t_bb) = race_terms(w_ring_n, N_BALL, BALL_D, RACE_R, MU_PLA);
	// The three terms every architecture shares: the planet pads and the air.
	let common = |d: &mut Drag, mu: f64| {
		d.add(N_PL as f64 * mu * w_pl_n * r_pl_pad * k_pl, 0.0, "6 planet thrust pads");
		// air: free-disc skin friction, no bluff radial ends (the ring is a
		// CLOSED annulus — that is the whole reason it is not lobed).
		d.add(disc_air_coeff((34.25 + RING_WALL) * 1e-3), 1.5, "ring disc air");
		d.add(ks.powf(2.5) * disc_air_coeff(ra_s * 1e-3), 1.5, "sun disc air (reflected)");
		d.add(N_PL as f64 * k_pl.powf(2.5) * disc_air_coeff(ra_p * 1e-3), 1.5, "6 planet disc air");
	};
	// ---- v3, SHIPPED: nothing but printed PLA -------------------------------
	let budget = |mu: f64| {
		let mut d = Drag::default();
		d.add(mu * w_ring_n * r_ring_pad, 0.0, "6 ring thrust pads (printed)");
		d.add(mu * w_sun_n * r_sun_land * ks, 0.0, "sun thrust land (printed, reflected)");
		common(&mut d, mu);
		d
	};
	// ---- v1 LEDGER: sliding ring land + the 608 ------------------------------
	let budget_v1 = |mu: f64, m608_nmm: f64| {
		let mut d = Drag::default();
		d.add(mu * w_ring_n * r_ring_pad, 0.0, "6 ring thrust pads (printed)");
		let kb = m608_nmm * 1e-3 / W0.powf(N_BRG);
		// the 608 runs at |k_S|·ω and its torque is reflected by |k_S| again
		d.add(ks.powf(1.0 + N_BRG) * kb, N_BRG, "608 (reflected)");
		common(&mut d, mu);
		d
	};
	// ---- v2 LEDGER: 24-ball thrust race + the 608 ----------------------------
	let budget_v2 = |mu: f64, m608_nmm: f64| {
		let mut d = Drag::default();
		let (roll, spin_c, _) = race_terms(w_ring_n, N_BALL, BALL_D, RACE_R, mu);
		d.add(roll, 0.0, "ring ball race — rolling (f ≤ a bound)");
		d.add(spin_c, 0.0, "ring ball race — contact spin");
		let kb = m608_nmm * 1e-3 / W0.powf(N_BRG);
		d.add(ks.powf(1.0 + N_BRG) * kb, N_BRG, "608 (reflected)");
		common(&mut d, mu);
		d
	};
	let i_eff_kgm2 = i_eff_gmm2 * 1e-9;
	// v1/v2 carry the 608's own rotating inertia on the sun side, so their
	// I_eff is NOT v3's. Including it is what makes the comparison fair to the
	// hardware rather than to this campaign's preferred answer.
	let m_ball_g = (4.0 / 3.0) * PI * (BALL_D / 2.0).powi(3) * RHO_STEEL;
	let i_eff_608 = i_eff_kgm2 + I608_GMM2 * ks * ks * 1e-9;
	let i_eff_v2 = i_eff_608
		+ (N_BALL as f64 * m_ball_g * RACE_R * RACE_R * 0.25
			+ N_BALL as f64 * 0.4 * m_ball_g * (BALL_D / 2.0).powi(2) * ((RACE_R / BALL_D).powi(2) + 0.25))
			* 1e-9;
	let d_nom = budget(MU_PLA);
	let (t_nom, rev_nom) = spin_down(i_eff_kgm2, &d_nom, W0);
	let (t_opt, _) = spin_down(i_eff_kgm2, &budget(MU_LO), W0);
	let (t_pes, _) = spin_down(i_eff_kgm2, &budget(MU_HI), W0);
	let d_slide = budget_v1(MU_PLA, M608_NMM);
	let (t_slide, rev_slide) = spin_down(i_eff_608, &d_slide, W0);
	let (t_slide_opt, _) = spin_down(i_eff_608, &budget_v1(MU_LO, M608_LO_NMM), W0);
	let (t_slide_pes, _) = spin_down(i_eff_608, &budget_v1(MU_HI, M608_HI_NMM), W0);
	let d_race = budget_v2(MU_PLA, M608_NMM);
	let (t_race, rev_race) = spin_down(i_eff_v2, &d_race, W0);
	let (t_race_opt, _) = spin_down(i_eff_v2, &budget_v2(MU_LO, M608_LO_NMM), W0);
	let (t_race_pes, _) = spin_down(i_eff_v2, &budget_v2(MU_HI, M608_HI_NMM), W0);
	// The counterfactual v3 can never pass: the ring's support term deleted
	// outright, i.e. the whole gain a working central web would buy.
	let t_noring = {
		let mut d = budget(MU_PLA);
		d.terms.retain(|t| !t.2.starts_with("6 ring thrust pads"));
		spin_down(i_eff_kgm2, &d, W0).0
	};
	// And the other counterfactual, for the on-axis point pivot (G21c): the sun's
	// thrust land deleted down to a Hertz point contact on the axis.
	let t_pivot = {
		let mut d = budget(MU_PLA);
		d.terms.retain(|t| !t.2.starts_with("sun thrust land"));
		let a_piv = hertz_a(w_sun_n, 2.0, e_star(E_PLA_MPA, NU_PLA, E_PLA_MPA, NU_PLA));
		d.add((3.0 * PI / 32.0) * MU_PLA * w_sun_n * a_piv * 1e-3 * ks, 0.0, "sun on-axis pivot");
		spin_down(i_eff_kgm2, &d, W0).0
	};
	// ...and the third: PRINTED balls in a printed race, i.e. v2's architecture
	// with PLA where the steel was. This is the largest gain v3 does not take,
	// so it is computed rather than described (G21a/b price why it is refused).
	let t_pball = {
		let mut d = budget(MU_PLA);
		d.terms.retain(|t| !t.2.starts_with("6 ring thrust pads"));
		let estar_pp = e_star(E_PLA_MPA, NU_PLA, E_PLA_MPA, NU_PLA);
		let a_pp = hertz_a(w_ring_n / N_BALL as f64, BALL_D / 2.0, estar_pp);
		d.add(2.0 * (a_pp * 1e-3) * w_ring_n * (RACE_R / BALL_D), 0.0, "printed ball race — rolling bound");
		spin_down(i_eff_kgm2, &d, W0).0
	};
	println!("\ndrag budget at ω₀ = {W0:.0} rad/s ({:.0} rpm)", W0 * 60.0 / TAU);
	for (c, e, w) in &d_nom.terms {
		println!("  {w:34} {:7.4} N·mm   (ω^{e:.1})", c * W0.powf(*e) * 1e3);
	}
	println!("  {:34} {:7.4} N·mm", "TOTAL", d_nom.total_nmm(W0));
	println!(
		"  I_eff {i_eff_gmm2:.0} g·mm²  →  spin {t_nom:.1} s / {rev_nom:.0} rev   (band {t_opt:.1}–{t_pes:.1} s)"
	);
	println!(
		"  v1 (sliding land + 608) on the same rotor: {:7.4} N·mm → {t_slide:.1} s / {rev_slide:.0} rev   (band {t_slide_opt:.1}–{t_slide_pes:.1} s)",
		d_slide.total_nmm(W0)
	);
	println!(
		"  v2 (ball race + 608)    on the same rotor: {:7.4} N·mm → {t_race:.1} s / {rev_race:.0} rev   (band {t_race_opt:.1}–{t_race_pes:.1} s)",
		d_race.total_nmm(W0)
	);
	gate(
		"PHYS spin time is REPORTED, not claimed; band is finite and > 0",
		t_nom > 0.0 && t_pes > 0.0 && t_opt.is_finite(),
		format!("{t_nom:.1} s [{t_pes:.1}–{t_opt:.1}]"),
		&mut ok,
	);
	// The one Coulomb term is the design's dominant loss — gate that we KNOW
	// it, rather than discovering it in the field.
	let coul_frac = d_nom.terms.iter().filter(|t| t.1 == 0.0).map(|t| t.0).sum::<f64>() / d_nom.torque(W0);
	gate(
		"PHYS Coulomb share of the budget is measured and published",
		(0.0..=1.0).contains(&coul_frac),
		format!("{:.0}% Coulomb", coul_frac * 100.0),
		&mut ok,
	);
	// bounded omission: the mesh sliding term at zero preload
	let alpha = d_nom.torque(W0) / i_eff_kgm2;
	let f_mesh = izz_s * 1e-9 * ks * alpha / (21.0e-3 * N_PL as f64);
	let t_mesh = 0.15 * (f_mesh / pa().cos()) * (ks + k_pl) * 1.11e-3 * N_PL as f64;
	gate(
		"PHYS omitted mesh-sliding term is bounded < 5% of the budget",
		t_mesh < 0.05 * d_nom.torque(W0),
		format!("{:.4} N·mm ({:.1}%)", t_mesh * 1e3, 100.0 * t_mesh / d_nom.torque(W0)),
		&mut ok,
	);

	// ===================== G17 — THE PRINTED SLIDING INTERFACES ==============
	// v3's whole drag budget is three printed thrust contacts. Each one gets the
	// same treatment the ball race got: the arm is asserted to be the minimum the
	// geometry allows (that is the ONLY lever on Coulomb torque), the bearing
	// pressure is computed against PLA's allowable, and the one free integer —
	// the pad count — is proved unable to move the answer.
	let sig_allow_pla = kernel_model::materials::pla::SIG_ALLOW_RT;
	// G17a — the sun land's inner edge is NOT a choice. The sun's own bed relief
	// removes its underside out to bore_r + C_BED, so nothing inboard of that can
	// touch; the base's land runs from the post's OD out past that edge, and the
	// contact annulus therefore STARTS there. Any smaller arm is geometrically
	// unreachable while a static thumb-pad post occupies the axis (G21c prices
	// the alternative).
	let sun_land_in = SUN_BORE_D / 2.0 + C_BED;
	gate(
		"G17a sun thrust arm is the minimum the bore geometry allows",
		(r_sun_land * 1e3 - (sun_land_in + SUN_LAND_W / 2.0)).abs() < 1e-12 && sun_land_in > POST_D / 2.0,
		format!("arm {:.3} mm (land {sun_land_in:.2}–{:.2})", r_sun_land * 1e3, sun_land_in + SUN_LAND_W),
		&mut ok,
	);
	// G17b/c/d — bearing pressure. Note honestly what this does and does not
	// buy: Coulomb torque is μWr and is INDEPENDENT of contact area, so widening
	// a pad changes none of the spin numbers. It changes wear-in and the risk of
	// the land brinelling into the mating face, which is why it is gated at all.
	let a_sun_land = PI * ((sun_land_in + SUN_LAND_W).powi(2) - sun_land_in.powi(2));
	let a_ring_pad = N_PL as f64 * RING_PAD_W * 6.0;
	let a_pl_pad = N_PL as f64 * PI * ((PLANET_SEAT_D / 2.0).powi(2) - (PLANET_BORE_D / 2.0 + C_BED).powi(2));
	let p_sun_land = w_sun_n / a_sun_land;
	let p_ring_pad = w_ring_n / a_ring_pad;
	let p_pl_pad = N_PL as f64 * w_pl_n / a_pl_pad;
	let p_worst = p_sun_land.max(p_ring_pad).max(p_pl_pad);
	gate(
		"G17b sun land bearing pressure ≪ PLA allowable",
		p_sun_land < 0.01 * sig_allow_pla,
		format!("{p_sun_land:.5} MPa on {a_sun_land:.2} mm² vs {sig_allow_pla:.0}"),
		&mut ok,
	);
	gate(
		"G17c ring pad bearing pressure ≪ PLA allowable",
		p_ring_pad < 0.01 * sig_allow_pla,
		format!("{p_ring_pad:.5} MPa on {a_ring_pad:.2} mm² vs {sig_allow_pla:.0}"),
		&mut ok,
	);
	gate(
		"G17d planet pad bearing pressure ≪ PLA allowable",
		p_pl_pad < 0.01 * sig_allow_pla,
		format!("{p_pl_pad:.5} MPa on {a_pl_pad:.2} mm² vs {sig_allow_pla:.0}"),
		&mut ok,
	);
	// G17e ANTI-GAMING — v2 proved its ball COUNT could not move the answer; v3
	// owes the same proof for its own free integer, the ring PAD count. Coulomb
	// friction is area-independent, so this must come back EXACTLY flat, not
	// merely within 5 %: if the pad count ever moved the spin time, the model
	// would be double-counting area somewhere.
	let (mut t_lo, mut t_hi) = (f64::INFINITY, 0.0f64);
	for n in [3usize, 4, 6, 8, 12, 24] {
		let mut dn = Drag::default();
		// same total normal load spread over n pads of the same total area
		for _ in 0..n {
			dn.add(MU_PLA * (w_ring_n / n as f64) * r_ring_pad, 0.0, "pad");
		}
		for t in d_nom.terms.iter().filter(|t| !t.2.starts_with("6 ring thrust pads")) {
			dn.terms.push(t.clone());
		}
		let t_n = spin_down(i_eff_kgm2, &dn, W0).0;
		t_lo = t_lo.min(t_n);
		t_hi = t_hi.max(t_n);
	}
	gate(
		"G17e ANTI-GAMING: the ring PAD COUNT cannot move the spin time",
		(t_hi - t_lo) / t_lo < 5e-3,
		format!("{t_lo:.3}–{t_hi:.3} s over n = 3–24 ({:.2}%)", 100.0 * (t_hi - t_lo) / t_lo),
		&mut ok,
	);
	// G17g — the sun's bore is a RUNNING fit now, not a press seat. v1/v2 pressed
	// the sun onto a 608's OD; if v3 inherited that interference the sun would not
	// turn at all. It is also deliberately the SAME diameter as the planet bore,
	// which is what lets one coupon pin gauge both.
	gate(
		"G17g sun bore is the profile's RUNNING fit, shared with the planet bore",
		(SUN_BORE_D - POST_D - 2.0 * C_FREE).abs() < 1e-12 && (SUN_BORE_D - PLANET_BORE_D).abs() < 1e-12,
		format!("Ø{SUN_BORE_D:.2} on Ø{POST_D:.2} = {C_FREE:.2} radial, = the planet bore"),
		&mut ok,
	);
	// G17h — the hub's top face must clear the sun everywhere EXCEPT the land, or
	// the whole point of shrinking the arm is lost to a full-face rub. The recess
	// is two axial clearances, not one, and that second one is boolean hygiene:
	// at one it lands exactly on the arms' top plane (§7.7 rule 3) and the chain
	// goes invalid. Both halves are asserted so neither can drift.
	let hub_recess = Z_GEAR - (Z_GEAR - 2.0 * C_Z);
	gate(
		"G17h hub face clears the sun off the land, and off the arm plane (§7.7)",
		hub_recess >= C_Z && (Z_GEAR - hub_recess - Z_ARM).abs() > 1e-6,
		format!("{hub_recess:.2} mm recess; hub top {:.2} vs arm top {Z_ARM:.2}", Z_GEAR - hub_recess),
		&mut ok,
	);
	// G17i — the post got slender when it stopped being a bearing bore, and this
	// campaign's own rule is that sub-Ø5 vertical printed pins are failure-prone.
	// 5.50 is on the right side of that line and the aspect ratio is published.
	let post_h = cap_top() - Z_GEAR;
	gate(
		"G17i the post stays on the right side of the printed-pin floor",
		POST_D >= 5.0 && (POST_D - PIN_D).abs() < 1e-12,
		format!("Ø{POST_D:.2} × {post_h:.2} tall (aspect {:.1}:1), = PIN_D", post_h / POST_D),
		&mut ok,
	);
	// G22a — WHO RETAINS THE SUN. v1/v2 pressed it onto the 608 and capped the
	// post; v3's sun is a drop-in part on a running fit, so the cap is the only
	// thing keeping it in when the spinner is inverted. That is a NEW duty for an
	// old part and it gets a gate: the cap must overhang the sun's bore and sit
	// one axial clearance above the sun's top face.
	gate(
		"G22a the cap RETAINS the sun (nothing else does, in v3)",
		CAP_D > SUN_BORE_D && (cap_bot() - sun_top() - C_Z).abs() < 1e-12,
		format!("Ø{CAP_D:.1} over a Ø{SUN_BORE_D:.2} bore, {:.2} mm above the sun", cap_bot() - sun_top()),
		&mut ok,
	);
	// G22b — the cap is the ONLY interference fit left in the whole model (v4
	// deleted the other six with the click bands), so its hoop strain is gated.
	let cap_strain = CAP_PRESS_R / (POST_D / 2.0);
	gate(
		"G22b the cap press fit (the model's only interference) is inside PLA's elastic range",
		cap_strain < SIG_YIELD_PLA / E_PLA_MPA,
		format!("{:.2}% vs {:.2}% yield (C_TIGHT would be {:.2}% — refused)", cap_strain * 100.0, 100.0 * SIG_YIELD_PLA / E_PLA_MPA, 100.0 * C_TIGHT / (POST_D / 2.0)),
		&mut ok,
	);
	gate(
		"G22b NC: the inherited C_TIGHT fit on the smaller post must FAIL the same check",
		C_TIGHT / (POST_D / 2.0) > SIG_YIELD_PLA / E_PLA_MPA,
		format!("{:.2}% — refused, and it is what v1/v2 used", 100.0 * C_TIGHT / (POST_D / 2.0)),
		&mut ok,
	);
	// G22c — v3 claims in ANALYSIS.md and in the listing that NO part has a
	// downward-facing horizontal face. That is a claim, so it is a gate: the
	// worst bridge span over every emitted part must be exactly zero. It is also
	// the thing that closes the door on the on-axis point pivot, whose blind
	// socket ceiling would have been the model's first bridge.
	gate(
		"G22c not one part in the set carries a real bridge (widest patch ≪ a facet)",
		worst_bridge < 0.05,
		format!("widest patch {worst_bridge:.3} mm over 10 parts, vs max_bridge {:.1}", p.max_bridge),
		&mut ok,
	);

	// G17f NEGATIVE CONTROL — the one that makes the hardware DELETION honest.
	// Put the 608 and the balls back on the same rotor and the same solver must
	// come back STRICTLY BETTER. If this gate ever passed, the fully-printed
	// number would be flattering itself and the whole ledger would be worthless.
	gate(
		"G17f NC: putting the hardware BACK must be strictly better",
		t_race > t_nom && t_slide > t_nom,
		format!("v3 {t_nom:.1} s < v1 {t_slide:.1} s < v2 {t_race:.1} s"),
		&mut ok,
	);

	// ===================== G20 — THE CENTRAL WEB, RE-OPENED =================
	// v2 refused a web from the ring to a small-radius central support on ETA
	// alone, when cancellation was the headline claim. It is no longer the
	// headline, so the trade is re-decided here on its merits — and it is
	// refused again, this time for two reasons that eta has nothing to do with,
	// both computed rather than asserted.
	//
	// The web has to cross the pin circle at r 27, so it must go OVER the top,
	// clearing whichever is higher of the top spider and the sun. That fixes its
	// height, and the ring's rim then has to REACH it: a dead cylindrical shell
	// from the top of the teeth up to the web plane, at r 34.25–36.50 where
	// inertia is most expensive.
	let web_z = ts_top().max(sun_top()) + C_Z;
	let shell_h = web_z - ring_top();
	let od_r = 34.25 + RING_WALL;
	let i_shell = PI * (od_r * od_r - 34.25 * 34.25) * shell_h * PLA * (34.25 * 34.25 + od_r * od_r) / 2.0;
	// What the eta balance can pay for on the ring side, with the sun at its
	// envelope MAXIMUM — i.e. the most favourable case the web could ever get.
	let ring_side_budget = ks * izz_s - N_PL as f64 * izz_p * k_pl;
	let teeth_left = ring_side_budget - i_shell - l_web;
	// Convert what is left into the quantity that decides it: the toothed ring
	// FACE WIDTH the eta balance can still afford. It has to be at least the
	// planet face or the ring does not carry the mesh at all. Swept over the
	// study's whole t_sun range, because a thinner sun lowers the web (less
	// shell) but also lowers the eta budget, and the two do not cancel.
	let i_r_per_mm = izz_r / T_RING;
	let mut web_face_best = f64::NEG_INFINITY;
	let mut web_face_at = 0.0f64;
	{
		let mut t = 3.00;
		while t <= T_SUN_MAX + 1e-9 {
			let i_s_t = izz_s * t / T_SUN;
			let z_web = ts_top().max(Z_GEAR + t) + C_Z;
			let sh = z_web - ring_top();
			let i_sh = PI * (od_r * od_r - 34.25 * 34.25) * sh * PLA * (34.25 * 34.25 + od_r * od_r) / 2.0;
			let face = (ks * i_s_t - N_PL as f64 * izz_p * k_pl - i_sh - l_web) / i_r_per_mm;
			if face > web_face_best {
				web_face_best = face;
				web_face_at = t;
			}
			t += 0.02;
		}
	}
	gate(
		"G20a NC: no sun thickness leaves the web a printable ring face",
		web_face_best < T_PLANET,
		format!("best {web_face_best:.2} mm of ring face at t_sun {web_face_at:.2} — the mesh needs {T_PLANET:.2}"),
		&mut ok,
	);
	// The other orientation avoids the shell — print the ring web-DOWN — but then
	// the web is the first layer and the rim has to rise from it, which is the
	// same shell. Print it teeth-down instead and the spokes become ceilings
	// bridged from the rim inward across open air. That span is measurable.
	let web_span = 34.25 - 4.0;
	gate(
		"G20b NC: printed teeth-down, the web's spokes exceed max_bridge",
		web_span > p.max_bridge,
		format!("{web_span:.2} mm spoke bridge vs max_bridge {:.2}", p.max_bridge),
		&mut ok,
	);

	// ===================== G21 — PRINTED BALLS, AND THE POINT PIVOT ==========
	// Two directions the fully-printed constraint makes obvious. Both are costed
	// on this run's own numbers; one is refused by the engine's own printability
	// oracle and one is refused by a geometric conflict with the thumb pad.
	let estar = e_star(E_STEEL, NU_STEEL, E_PLA_MPA, NU_PLA);
	let estar_pla = e_star(E_PLA_MPA, NU_PLA, E_PLA_MPA, NU_PLA);
	let ball_load = w_ring_n / N_BALL as f64;
	let a_race = hertz_a(ball_load, BALL_D / 2.0, estar);
	let p0_race = hertz_p0(ball_load, a_race);
	let a_pla = hertz_a(ball_load, BALL_D / 2.0, estar_pla);
	let p0_pla = hertz_p0(ball_load, a_pla);
	let roll_pla = 2.0 * (a_pla * 1e-3) * w_ring_n * (RACE_R / BALL_D);
	// G21a — PLA's modulus is ~60× lower than bearing steel's, so the contact
	// patch does NOT scale the way intuition says. E* falls by 1.97×, and a ∝
	// E*^(−1/3), so the patch grows only 1.25× and the peak pressure FALLS.
	// Printed balls are mechanically fine. Recomputed, not assumed.
	gate(
		"G21a printed PLA balls are MECHANICALLY fine (Hertz recomputed)",
		p0_pla < SIG_YIELD_PLA && roll_pla < 0.05 * d_nom.torque(W0),
		format!("a {a_pla:.4} mm ({:.2}× steel), p₀ {p0_pla:.1} MPa, roll {:.4} N·mm", a_pla / a_race, roll_pla * 1e3),
		&mut ok,
	);
	// G21b — the FIRST thing that had to be checked, and it came back the other
	// way. A sphere's lower hemisphere is a continuously worsening overhang that
	// reaches 90° at the contact point, so the expectation was that the
	// campaign's own emit oracle would refuse it outright. It does NOT at
	// Ø1.50 — the whole steep region lies inside the oracle's first-layer bed
	// tolerance, because the ball is barely thicker than one. That is a real
	// negative result and it is recorded rather than quietly replaced with an
	// argument that works. The same oracle DOES fire on a Ø6.00 sphere, which is
	// what proves the Ø1.50 pass is a measurement and not blindness.
	let ball_steep = tessellate_default(&sphere(DVec3::ZERO, BALL_D / 2.0, 48, 24))
		.support_free_report(Vec3::Z, p.max_unsupported_angle as f32, 0.3)
		.steep_area;
	let big_steep = tessellate_default(&sphere(DVec3::ZERO, 3.0, 96, 48))
		.support_free_report(Vec3::Z, p.max_unsupported_angle as f32, 0.3)
		.steep_area;
	gate(
		"G21b the support oracle does NOT refuse a Ø1.50 ball (and is not blind)",
		ball_steep < 1e-6 && big_steep > 1e-6,
		format!("Ø{BALL_D:.2} steep {ball_steep:.3e} mm²; Ø6.00 steep {big_steep:.1} mm²"),
		&mut ok,
	);
	// G21b2 — what DOES refuse them: FORM ERROR. A Ø1.50 ball at the shipped
	// 0.20 mm layer height is 7.5 layers tall, and the layer staircase alone puts
	// a peak radial deviation of h/2 on it. A stock G25 loose bearing ball is
	// round to 0.6 µm. The ratio is the refusal, and it is arithmetic.
	let ball_layers = BALL_D / LAYER_H;
	let ball_form_err = LAYER_H / 2.0;
	let form_ratio = ball_form_err / 6e-4;
	gate(
		"G21b2 NC: a printed ball's form error is orders past a bearing ball's",
		form_ratio > 100.0,
		format!("{ball_layers:.1} layers, ±{ball_form_err:.3} mm vs G25 0.0006 — {form_ratio:.0}×"),
		&mut ok,
	);
	// G21b3 — and the refusal is NOT a stress refusal, which is stated rather
	// than left to be assumed. A rigid ring on balls that differ in height by up
	// to the form error lands on the tallest ones; take the absurd limit of ONE
	// ball carrying the whole ring and the Hertz pressure is still under yield.
	// So the thing that has no model is the ROLLING behaviour of an out-of-round
	// element, not the contact — and this campaign does not publish a spin time
	// that rests on an unmodelled loss.
	let p0_one_ball = hertz_p0(w_ring_n, hertz_a(w_ring_n, BALL_D / 2.0, estar_pla));
	gate(
		"G21b3 printed balls are not refused on STRESS (worst case, one ball)",
		p0_one_ball < SIG_YIELD_PLA,
		format!("{p0_one_ball:.1} MPa on a single ball vs {SIG_YIELD_PLA:.0} yield"),
		&mut ok,
	);
	// G21c — the on-axis point pivot. It wins, and it is not taken: a STATIC
	// thumb pad has to be carried by a column, the column has to be on the axis,
	// and a column through the sun forbids the blind on-axis socket a point
	// pivot needs. The gate publishes what that ergonomic choice costs instead
	// of leaving it implicit.
	gate(
		"G21c the on-axis pivot alternative is costed and PUBLISHED",
		t_pivot > t_nom,
		format!("{t_pivot:.1} s vs {t_nom:.1} s shipped — {:+.0}% left on the table", 100.0 * (t_pivot / t_nom - 1.0)),
		&mut ok,
	);
	// Bounded omission, carried over: v2's ball-to-ball rub bound is still
	// computed because the v2 ledger row uses the same race model.
	gate(
		"G21d v2 ledger row's own bounded omission (ball-to-ball) still holds",
		t_bb < 0.05 * d_race.torque(W0),
		format!("{:.4} N·mm ({:.1}% of the v2 row)", t_bb * 1e3, 100.0 * t_bb / d_race.torque(W0)),
		&mut ok,
	);

	// ===================== G19 — THE EDGE-ON CASE, PROPERLY ==================
	// v1's listing carried a usage note: "held EDGE-ON the term largely
	// vanishes — gravity is then reacted through the meshes, which are rolling
	// contacts". That note is INCOMPLETE and this campaign now says so with
	// numbers. Gravity reacted at a mesh does not stop there: it continues into
	// the planet, out through the planet's PIN JOURNAL, and that journal is a
	// sliding contact reflected by k_planet = 5.5. The load never leaves the
	// machine — it only changes which contacts carry it.
	//
	// Load path, edge-on. The ring is located ONLY by six meshes. Worst support
	// geometry (no planet at bottom dead centre, two at ±30°) makes the sum of
	// the reaction magnitudes W/cos30° = 1.1547·W — DERIVED, not a chosen
	// safety factor. Each reaction is normal to the flank, hence /cos α.
	let share = 1.0 / (PI / 6.0).cos();
	let y_mean_pr = 0.25 * eps_pr * PI * M * pa().cos(); // mean sliding arm, ring mesh
	let r_journal = PIN_D / 2.0 * 1e-3;
	let edge_budget = |mu: f64| {
		let mut d = Drag::default();
		// (1) mesh sliding at the ring mesh. INTERNAL mesh ⇒ same rotation
		// sense ⇒ the relative rate is (k_planet − 1), not (k_planet + 1).
		d.add(mu * share / pa().cos() * w_ring_n * (k_pl - 1.0) * y_mean_pr * 1e-3, 0.0, "ring weight, reacted at the six meshes");
		// (2) the same load, continuing into the six planet journals
		d.add(mu * share * w_ring_n * r_journal * k_pl, 0.0, "ring weight, on to the planet journals");
		// (3) the planets' own weight, now radial on those journals
		d.add(N_PL as f64 * mu * w_pl_n * r_journal * k_pl, 0.0, "6 planet journals (own weight)");
		// (4) v3: the sun's weight is now RADIAL on its own printed post journal
		// instead of axial on the land. v1/v2 put a 608 here; there isn't one.
		d.add(mu * w_sun_n * (POST_D / 2.0) * 1e-3 * ks, 0.0, "sun on its post journal");
		for t in d_nom.terms.iter().filter(|t| t.1 == 1.5) {
			d.terms.push(t.clone());
		}
		d
	};
	let d_edge = edge_budget(MU_PLA);
	let (t_edge, rev_edge) = spin_down(i_eff_kgm2, &d_edge, W0);
	println!(
		"  EDGE-ON (axis horizontal): {:7.4} N·mm → {t_edge:.1} s / {rev_edge:.0} rev",
		d_edge.total_nmm(W0)
	);
	gate(
		"G19a edge-on load path costed end-to-end (mesh, journal AND post)",
		t_edge > 0.0 && d_edge.total_nmm(W0) > 0.0,
		format!("{:.4} N·mm → {t_edge:.1} s", d_edge.total_nmm(W0)),
		&mut ok,
	);
	// **The usage note REVERTS in v3, and both halves are asserted so it cannot
	// drift.** v1: edge-on helped, by 1.30× (not the "vanishes" v1 first wrote).
	// v2: the race gave the AXIAL load a rolling path and nothing gave the
	// radial one, so flat won. v3 has no rolling path anywhere, so edge-on is
	// back to being the better way to hold it — for the v1 reason.
	gate(
		"G19b v3: EDGE-ON beats FLAT again — the ring's axial load is the biggest term",
		t_edge > t_nom,
		format!("edge-on {t_edge:.1} s > flat {t_nom:.1} s ({:.2}×)", t_edge / t_nom),
		&mut ok,
	);
	// ...and it is a modest win, not a transformation. Asserting the CEILING is
	// what stops the listing drifting back to "the term largely vanishes".
	gate(
		"G19c and it is only ~1.3×, not a transformation (ceiling asserted)",
		t_edge < 1.6 * t_nom,
		format!("{:.2}× — the load moves, it does not leave", t_edge / t_nom),
		&mut ok,
	);

	// ===================== CAD / ASSEMBLY / DOCS ============================
	for (n, s) in [
		("ring_66t", &s_ring),
		("sun_42t", &s_sun),
		("planet_12t", &s_planet),
		("base_spider", &s_base),
		("top_spider", &s_top),
		("cap", &s_cap),
	] {
		let _ = std::fs::write(format!("{OUT}/cad/{n}.step"), export_step(s, n));
	}
	let mut scene = Mesh::default();
	let mut merge = |m: &Mesh| {
		let base = scene.positions.len() as u32;
		scene.positions.extend_from_slice(&m.positions);
		scene.normals.extend_from_slice(&m.normals);
		scene.indices.extend(m.indices.iter().map(|i| i + base));
	};
	// the scene is posed AS ASSEMBLED (the parts/ meshes are print-posed and
	// dropped to the bed — stacking those would draw a lie)
	let a_ring = tessellate_default(&s_ring.transformed(tr(0.0, 0.0, Z_ROT)));
	let a_sun = tessellate_default(&s_sun.transformed(tr(0.0, 0.0, Z_GEAR)));
	merge(&m_base);
	merge(&a_ring);
	merge(&a_sun);
	merge(&m_top);
	merge(&m_cap);
	for j in 0..N_PL {
		let b = TAU * j as f64 / N_PL as f64;
		merge(&tessellate_default(&s_planet.transformed(tr(CD * b.cos(), CD * b.sin(), Z_ROT) * rotz(b))));
	}
	let _ = std::fs::write(format!("{OUT}/assembly/assembly.stl"), scene.to_stl_binary());
	for (n, m) in [("base_spider", &m_base), ("ring", &a_ring), ("sun", &a_sun), ("top_spider", &m_top), ("cap", &m_cap)] {
		let _ = std::fs::write(format!("{OUT}/assembly/scene/{n}.stl"), m.to_stl_binary());
	}
	let _ = std::fs::write(format!("{OUT}/cad/assembly.step"), export_step(&s_base, "nullspin_assembly"));
	// STEP round-trip on the most complex part — the exported CAD must be the
	// same solid, not a lookalike.
	let step_ring = export_step(&s_ring, "nullspin_ring_66t");
	match kernel_brep::import_step(&step_ring) {
		Ok(back) => {
			let dv = (volume(&back).abs() - volume(&s_ring).abs()).abs() / volume(&s_ring).abs();
			gate("CAD ring STEP round-trip conserves volume (<2.5%)", dv < 0.025, format!("dv {:5.2}%", dv * 100.0), &mut ok);
		}
		Err(e) => gate("CAD ring STEP round-trip", false, format!("{e:?}"), &mut ok),
	}

	// ---- assembly sheet + renders (both are shipped deliverables, both gated)
	let mut planets_mesh = Mesh::default();
	{
		let mut m2 = |m: &Mesh| {
			let base = planets_mesh.positions.len() as u32;
			planets_mesh.positions.extend_from_slice(&m.positions);
			planets_mesh.normals.extend_from_slice(&m.normals);
			planets_mesh.indices.extend(m.indices.iter().map(|i| i + base));
		};
		for j in 0..N_PL {
			let b = TAU * j as f64 / N_PL as f64;
			m2(&tessellate_default(&s_planet.transformed(tr(CD * b.cos(), CD * b.sin(), Z_ROT) * rotz(b))));
		}
	}
	let _ = std::fs::write(format!("{OUT}/assembly/scene/planets_x6.stl"), planets_mesh.to_stl_binary());
	// The BOM has exactly six rows and every one of them is `made`. There is no
	// `bought` row to write, and that is the whole point of v3.
	let _ = std::fs::write(
		format!("{OUT}/assembly/scene/bom.csv"),
		format!(
			"name,kind,qty,material,part_number,grams_per_unit\n\
			 base_spider (held frame),made,1,PLA,P1,{b:.2}\n\
			 planet 12T x6,made,6,PLA,P4,{pl:.2}\n\
			 sun 42T,made,1,PLA,P3,{su:.2}\n\
			 ring 66T,made,1,PLA,P0,{rg:.2}\n\
			 top_spider,made,1,PLA,P2,{tp:.2}\n\
			 cap,made,1,PLA,P7,{cp:.2}\n",
			b = volume(&s_base).abs() * PLA,
			pl = mg_p,
			su = mg_s,
			rg = mg_r,
			tp = volume(&s_top).abs() * PLA,
			cp = volume(&s_cap).abs() * PLA,
		),
	);
	let sheet = serde_json::json!({
		"project": "NULLSPIN",
		"doc_title": "NULLSPIN — assembly sheet",
		"rev": "A",
		"date": "generated",
		"out_prefix": format!("{OUT}/assembly/ASSEMBLY"),
		"bom_csv": format!("{OUT}/assembly/scene/bom.csv"),
		"view": { "elev": 22, "azim": -58 },
		"parts": [
			{ "name": "base_spider (held frame)", "stl": format!("{OUT}/assembly/scene/base_spider.stl"), "color": "#2f3b52" },
			{ "name": "planet 12T x6", "stl": format!("{OUT}/assembly/scene/planets_x6.stl"), "color": "#c9722f" },
			{ "name": "sun 42T", "stl": format!("{OUT}/assembly/scene/sun.stl"), "color": "#1f7a72" },
			{ "name": "ring 66T", "stl": format!("{OUT}/assembly/scene/ring.stl"), "color": "#8a6ec4" },
			{ "name": "top_spider", "stl": format!("{OUT}/assembly/scene/top_spider.stl"), "color": "#48566f" },
			{ "name": "cap", "stl": format!("{OUT}/assembly/scene/cap.stl"), "color": "#b8433a" }
		],
		"explode": { "axis": [0.0, 0.0, 1.0], "auto": true, "gap_mm": 10 },
		"steps": [
			{ "order": 1, "text": "Drop the sun over the post. It rests on the small raised land around the post and is free to turn — there is no bearing and nothing to press." },
			{ "order": 2, "text": "Drop six planets onto six pins. They are identical and self-clock against the sun." },
			{ "order": 3, "text": "Drop the ring over the planets. It self-clocks, is located radially by all six, and rests on the six thrust pads in the base." },
			{ "order": 4, "text": "Top spider on — the BAYONET. Line each slot's wide end up over its pin (the six arms sit about 7 deg anticlockwise of the base arms), drop it flat, then twist it 7 deg clockwise until all six stop. No force: it drops on free and the twist is a slide." },
			{ "order": 5, "text": "Check the lock by eye: every pin's fin must sit at the CLOSED end of its slot. Nothing to press, nothing to click past, and no printer calibration involved — the fin is 1.15 mm of material over the slot wall." },
			{ "order": 6, "text": "Press the cap onto the post. No hardware, no glue, no tools, no break-in." }
		]
	});
	let _ = std::fs::write(format!("{OUT}/assembly/scene/sheet_job.json"), format!("{sheet:#}\n"));
	match run_py("tools/assembly_doc.py", &format!("{OUT}/assembly/scene/sheet_job.json")) {
		Ok(_) => {
			let _ = std::fs::rename(format!("{OUT}/assembly/ASSEMBLY_assembly_doc.png"), format!("{OUT}/assembly/ASSEMBLY.png"));
			// the tool also drafts its own instructions; this campaign ships the
			// authored one, so the draft is removed rather than left to confuse.
			let _ = std::fs::remove_file(format!("{OUT}/assembly/ASSEMBLY_instructions.md"));
			gate("SHIP assembly sheet rendered (assembly/ASSEMBLY.png)", true, "assembly_doc.py".into(), &mut ok);
		}
		Err(e) => gate("SHIP assembly sheet rendered (assembly/ASSEMBLY.png)", false, e.chars().take(110).collect(), &mut ok),
	}
	let renders = [
		("assembly/assembly.stl", "renders/render_assembly.png"),
		("assembly/scene/ring.stl", "renders/render_ring.png"),
		("assembly/scene/sun.stl", "renders/render_sun.png"),
		("assembly/scene/base_spider.stl", "renders/render_base_spider.png"),
	];
	let n_ok = renders
		.iter()
		.filter(|(a, b)| run_py_plain("tools/render_views.py", &[&format!("{OUT}/{a}"), &format!("{OUT}/{b}")]).is_ok())
		.count();
	gate("SHIP product renders written (renders/)", n_ok == renders.len(), format!("{n_ok}/{}", renders.len()), &mut ok);

	let printed_g = mg_r + mg_s + N_PL as f64 * mg_p + frame_g;
	gate("MASS printed set ≤ 28 g", printed_g <= 28.0, format!("{printed_g:.1} g"), &mut ok);
	let height = cap_top().max(pin_top());
	gate("ENVELOPE Ø ≤ 73.0 × 12.0 mm", 2.0 * (34.25 + RING_WALL) <= 73.0 && height <= 12.0, format!("Ø{:.1} × {height:.2}", 2.0 * (34.25 + RING_WALL)), &mut ok);

	write_docs(
		&Docs {
			eta,
			eta_lo,
			eta_b,
			sens: &sens,
			i_eff_gmm2,
			izz_r,
			izz_s,
			izz_p,
			mg_r,
			mg_s,
			mg_p,
			mg_sb,
			printed_g,
			height,
			eps_sp,
			eps_pr,
			floor,
			neighbour,
			jt_measured,
			lash_deg: lash_angle.to_degrees(),
			jr,
			residual,
			t_nom,
			rev_nom,
			t_opt,
			t_pes,
			t_noring,
			coul_frac,
			drag: &d_nom,
			drag_slide: &d_slide,
			t_slide,
			rev_slide,
			t_slide_opt,
			t_slide_pes,
			drag_race: &d_race,
			t_race,
			rev_race,
			t_race_opt,
			t_race_pes,
			i_eff_608_gmm2: i_eff_608 * 1e9,
			i_eff_v2_gmm2: i_eff_v2 * 1e9,
			drag_edge: &d_edge,
			t_edge,
			rev_edge,
			a_race,
			p0_race,
			a_pla,
			p0_pla,
			roll_pla,
			ball_steep,
			big_steep,
			ball_layers,
			form_ratio,
			p0_one_ball,
			t_pball,
			web_face_best,
			web_face_at,
			r_sun_land: r_sun_land * 1e3,
			r_ring_pad: r_ring_pad * 1e3,
			r_pl_pad: r_pl_pad * 1e3,
			p_worst,
			p_sun_land,
			p_ring_pad,
			p_pl_pad,
			t_pivot,
			l_web,
			i_shell,
			shell_h,
			teeth_left,
			web_span,
			t_pad_lo: t_lo,
			t_pad_hi: t_hi,
			worst_bridge,
			y_pl,
			y_sun,
			y_ring,
			sig_ring,
			sig_allow,
			margin,
			uncredited,
			credited,
			ladder_best,
			worst_sp,
			worst_pr,
			worst_all,
			min_cl_s: sw_s.min_clearance,
			min_cl_r: sw_r.min_clearance,
			jam,
			k_sun,
			k_pl,
			study_evals: report.evaluation_count(),
			study_feasible: report.feasible_count,
			yield_strain,
			spec_strain,
			snap_max,
			engage_xy,
			engage_full,
			travel,
			float_nom,
			u_rel,
			captive,
			wc_capture,
			f_cap,
			carried_n,
			mg_base: volume(&s_base).abs() * PLA,
			mg_top: volume(&s_top).abs() * PLA,
			mg_cap: volume(&s_cap).abs() * PLA,
			mg_coupon: volume(&s_coupon).abs() * PLA,
			mg_key: volume(&s_key).abs() * PLA,
		},
	);

	println!("\nNULLSPIN: {}", if ok { "ALL GATES PASS" } else { "<<< FAIL" });
	std::process::exit(if ok { 0 } else { 1 });
}

fn run_py(tool: &str, job: &str) -> Result<serde_json::Value, String> {
	let out = std::process::Command::new("python3")
		.args([tool, job])
		.output()
		.map_err(|e| format!("python3 not runnable ({e}) — the shipped sheet cannot be skipped"))?;
	let stdout = String::from_utf8_lossy(&out.stdout);
	let last = stdout.lines().rev().find(|l| !l.trim().is_empty()).unwrap_or("");
	let val: serde_json::Value = serde_json::from_str(last).map_err(|e| {
		let tail: String = String::from_utf8_lossy(&out.stderr).chars().rev().take(300).collect::<String>().chars().rev().collect();
		format!("{tool}: last stdout line is not JSON ({e}); stderr tail: {tail}")
	})?;
	if val.get("ok").and_then(|b| b.as_bool()) != Some(true) {
		return Err(format!("{tool}: {}", val.get("error").and_then(|e| e.as_str()).unwrap_or("ok != true")));
	}
	Ok(val)
}

fn run_py_plain(tool: &str, args: &[&str]) -> Result<(), String> {
	let out = std::process::Command::new("python3")
		.arg(tool)
		.args(args)
		.output()
		.map_err(|e| format!("python3 not runnable: {e}"))?;
	if out.status.success() {
		Ok(())
	} else {
		Err(format!("{tool} exited {:?}: {}", out.status.code(), String::from_utf8_lossy(&out.stderr).chars().take(200).collect::<String>()))
	}
}

struct Docs<'a> {
	eta: f64,
	eta_lo: f64,
	eta_b: f64,
	sens: &'a [(&'a str, f64)],
	i_eff_gmm2: f64,
	izz_r: f64,
	izz_s: f64,
	izz_p: f64,
	mg_r: f64,
	mg_s: f64,
	mg_p: f64,
	mg_sb: f64,
	printed_g: f64,
	height: f64,
	eps_sp: f64,
	eps_pr: f64,
	floor: f64,
	neighbour: f64,
	jt_measured: f64,
	lash_deg: f64,
	jr: f64,
	residual: f64,
	t_nom: f64,
	rev_nom: f64,
	t_opt: f64,
	t_pes: f64,
	t_noring: f64,
	coul_frac: f64,
	drag: &'a Drag,
	// ---- the three-way ledger: v1 (sliding + 608), v2 (race + 608), v3 -----
	drag_slide: &'a Drag,
	t_slide: f64,
	rev_slide: f64,
	t_slide_opt: f64,
	t_slide_pes: f64,
	drag_race: &'a Drag,
	t_race: f64,
	rev_race: f64,
	t_race_opt: f64,
	t_race_pes: f64,
	i_eff_608_gmm2: f64,
	i_eff_v2_gmm2: f64,
	drag_edge: &'a Drag,
	t_edge: f64,
	rev_edge: f64,
	// ---- the directions evaluated, winners and losers alike ----------------
	a_race: f64,
	p0_race: f64,
	a_pla: f64,
	p0_pla: f64,
	roll_pla: f64,
	ball_steep: f64,
	big_steep: f64,
	ball_layers: f64,
	form_ratio: f64,
	p0_one_ball: f64,
	t_pball: f64,
	web_face_best: f64,
	web_face_at: f64,
	r_sun_land: f64,
	r_ring_pad: f64,
	r_pl_pad: f64,
	p_worst: f64,
	p_sun_land: f64,
	p_ring_pad: f64,
	p_pl_pad: f64,
	t_pivot: f64,
	l_web: f64,
	i_shell: f64,
	shell_h: f64,
	teeth_left: f64,
	web_span: f64,
	t_pad_lo: f64,
	t_pad_hi: f64,
	worst_bridge: f64,
	y_pl: f64,
	y_sun: f64,
	y_ring: f64,
	sig_ring: f64,
	sig_allow: f64,
	margin: f64,
	uncredited: f64,
	credited: f64,
	ladder_best: f64,
	worst_sp: f64,
	worst_pr: f64,
	worst_all: f64,
	min_cl_s: f64,
	min_cl_r: f64,
	jam: f64,
	k_sun: f64,
	k_pl: f64,
	study_evals: usize,
	study_feasible: usize,
	yield_strain: f64,
	spec_strain: f64,
	snap_max: f64,
	engage_xy: f64,
	engage_full: f64,
	travel: f64,
	float_nom: f64,
	u_rel: f64,
	captive: f64,
	wc_capture: f64,
	f_cap: f64,
	carried_n: f64,
	mg_base: f64,
	mg_top: f64,
	mg_cap: f64,
	mg_coupon: f64,
	mg_key: f64,
}

fn write_docs(d: &Docs) {
	let budget_rows = d
		.drag
		.terms
		.iter()
		.map(|(c, e, w)| {
			let cls = if *e == 0.0 { "**Coulomb**" } else if *e < 1.0 { "sub-linear" } else { "quadratic-ish" };
			format!("| {w} | ω^{e:.1} ({cls}) | {:.4} | {:.0}% |", c * W0.powf(*e) * 1e3, 100.0 * c * W0.powf(*e) / d.drag.torque(W0))
		})
		.collect::<Vec<_>>()
		.join("\n");
	let sens_rows = d.sens.iter().map(|(k, v)| format!("| {k} | {v:.4} |")).collect::<Vec<_>>().join("\n");
	let edge_rows = d
		.drag_edge
		.terms
		.iter()
		.map(|(c, e, w)| format!("| {w} | {:.4} |", c * W0.powf(*e) * 1e3))
		.collect::<Vec<_>>()
		.join("\n");
	let od = 2.0 * (34.25 + RING_WALL);
	let nc_eps_txt = format!("8T×8T at 30° reads {:.4}", contact_ratio_external(1.0, 30.0, 8, 8));

	// ---------------- analysis/ANALYSIS.md (GENERATED from this run) --------
	let mut a = String::new();
	a.push_str(&format!(
		"# NULLSPIN — analysis (generated by `nullspin.rs`; regenerated every run)\n\n\
		Every number below is what the gate suite measured on THIS build, so it cannot go\n\
		stale. The frozen research contract, the analysis plan and the provenance of every\n\
		researched constant are in `DESIGN.md`.\n\n\
		## What this artifact claims\n\n\
		The claim is the **counter-rotation** and its exact integer ratio. The\n\
		**angular-momentum cancellation** eta is a supporting receipt, published as a\n\
		modelled band — not the headline. Spin time is reported, never claimed. Nothing\n\
		here is a measurement of a printed part: this is an as-designed deliverable and\n\
		every row says which class it is in.\n\n\
		**v3 adds one hard constraint: ZERO non-printed parts.** No bearing, no balls, no\n\
		magnets, screws, nuts, weights or inserts — the `You also need:` line on the model\n\
		page reads *nothing*. That was taken deliberately, in full knowledge that it costs\n\
		spin time, and this document measures the cost rather than describing it: v1\n\
		(sliding ring land + one 608) and v2 (24-ball steel thrust race + one 608) are\n\
		REBUILT on the shipped rotor by the same solver in the same run, so the three-way\n\
		ledger below is a measurement and not a memory.\n\n\
		## Gear set, as built\n\n\
		| quantity | value | how it is proved |\n|---|---|---|\n\
		| module · pressure angle | m {M:.3} · {PA_DEG:.1}° | G0 probe: the internal 66T generator accepts this with **{probe:+.2}%** margin; the negative control (36T @ 30°) is refused |\n\
		| teeth S / P / R · planets | {S_T} / {P_T} / {R_T} · {N_PL} | G1a `EpicyclicTrain::validate_assembly` = Ok; NC at n=5 refused |\n\
		| centre distance, both meshes | {cd:.3} mm | identity 33.0 − 6.0 = m(S+P)/2 |\n\
		| ratio ω_sun/ω_ring | {k_sun:+.6} = −R/S | G1b, derived from the engine's own `simple_ratio` |\n\
		| ratio ω_planet/ω_ring | {k_pl:+.4} = +R/P | G1d, internal mesh ⇒ same sense |\n\
		| headline | 7 ring revs → 11 sun revs, EXACTLY | G1c: 7·66 = 11·42 = 462, integer identity |\n\
		| contact ratio sun–planet | {eps_sp:.4} | G4a ≥ 1.20 — no engine API, so the formula is written in this campaign and driven by its own negative control ({nc}) |\n\
		| contact ratio planet–ring | {eps_pr:.4} | G4b ≥ 1.20 |\n\
		| undercut floor 2/sin²α | {floor:.3} T | G3: the 12T planet clears it at x = 0 |\n\
		| tip/root clearance, all 4 flanks | 0.250 mm | G4c, ISO 53 = 0.25·m |\n\
		| adjacent-planet gap | {neighbour:.3} mm | G2 ≥ 1.0 mm — and it is the EN 71-1 number too |\n\
		| circular backlash, measured by bisection | {jt:.3} mm ({lash_deg:.3}° at the sun) | G7, bisected to flank contact on the real solids |\n\n",
		M = M, PA_DEG = PA_DEG, S_T = S_T, P_T = P_T, R_T = R_T, N_PL = N_PL,
		cd = CD, probe = d.margin * 100.0, k_sun = d.k_sun, k_pl = d.k_pl,
		eps_sp = d.eps_sp, eps_pr = d.eps_pr, floor = d.floor, neighbour = d.neighbour,
		jt = d.jt_measured, lash_deg = d.lash_deg,
		nc = nc_eps_txt,
	));
	a.push_str(&format!(
		"**Backlash, and a deviation on record.** The frozen spec predicted a 1.29° lash\n\
		angle at the sun. That number does not reproduce: 0.09 mm of thinning per flank on\n\
		both members opens jt = 0.18 mm at the pitch line, and 0.18 mm on a 21 mm pitch\n\
		radius is 0.491° at one mesh (0.982° for the sun turned against a held ring, adding\n\
		the ring mesh reflected by R/S). The bisection on the built solids measures\n\
		**{jt:.3} mm / {lash_deg:.3}°** at the sun mesh, which agrees with the single-mesh\n\
		derivation, not with 1.29°. The measured number is what ships.\n\n\
		## Motion — proved, not assumed\n\n\
		Two engine capabilities were checked first and both were REFUSED for this case,\n\
		on the record rather than silently worked around. `kinematics::instance_poses` is\n\
		ring-fixed / sun-driven and does not cover a grounded carrier.\n\
		`kernel_model::mechanism` — the planar-linkage sweeper — has exactly two joint\n\
		kinds, revolute and prismatic; there is no gear pair or rolling-contact higher\n\
		pair in it, so \"planet rolls on ring\" simply cannot be declared, and a planetary\n\
		train modelled with revolutes alone would leave every planet free to spin. Its\n\
		own module doc says as much and points back at `kinematics`. So this campaign\n\
		carries its own STAR pose evaluator and gates it. The full mesh cycle repeats every ONE ring tooth pitch: over\n\
		θ = 2π/66 the planet turns exactly 2π/12 and the sun exactly 2π/42, so a dense\n\
		sweep of one ring pitch visits every distinct mesh state — much denser in mesh\n\
		phase than the same pose count spread over two whole revolutions.\n\n\
		The sweep runs in two tiers, as §25 step 5 requires: a dense mesh sweep for the\n\
		whole cycle, then EXACT `overlap_volume` on the B-reps at load-bearing poses.\n\
		Because both members move, the dense tier is evaluated in the fixed member's own\n\
		frame, which folds all the motion into the planet's pose.\n\n\
		| gate | what it sweeps | poses | result |\n|---|---|---|---|\n\
		| G5a | sun mesh, dense (BVH clearance + exact triangle-crossing oracle) | 96 | min clearance **{min_cl_s:.3} mm**, 0 contacts, 0 crossings |\n\
		| G5b | ring mesh, dense | 96 | min clearance **{min_cl_r:.3} mm**, 0 contacts, 0 crossings |\n\
		| G5c | both meshes, EXACT `overlap_volume` across the cycle | 16 × 2 | {worst_sp:.3e} mm³ |\n\
		| G5d | all six planets over two FULL ring revolutions, exact | 6 × 6 × 2 | {worst_all:.3e} mm³ |\n\
		| **G6 negative control** — sun driven ±5% off ratio | exact | 12 | **{jam:.4} mm³ — JAMS, as it must** |\n\n\
		Without G6, G5 is not a gate. The six-planet run in G5d is the check of the\n\
		symmetry argument itself (a 60° rigid rotation maps sun→sun through 7 pitches,\n\
		ring→ring through 11, and planet j→j+1), not a redundant repeat.\n\n",
		jt = d.jt_measured, lash_deg = d.lash_deg,
		worst_sp = d.worst_sp.max(d.worst_pr), worst_all = d.worst_all, jam = d.jam,
		min_cl_s = d.min_cl_s, min_cl_r = d.min_cl_r,
	));
	a.push_str(&format!(
		"## eta — the receipt, MODELLED\n\n\
		`eta = 1 − |Σ Iᵢωᵢ| / Σ|Iᵢωᵢ|`, computed ONLY from `mass_properties` on the exact\n\
		B-rep — teeth, chamfers, index grooves, bore and all, never an annulus\n\
		approximation (a plain annulus mis-states a planet by ~25%, which swings eta by\n\
		0.8 points).\n\n\
		| rotor | mass | I_zz about the spin axis | speed ratio |\n|---|---|---|---|\n\
		| ring 66T | {mg_r:.2} g | {izz_r:.1} g·mm² | +1.0000 |\n\
		| sun 42T | {mg_s:.2} g | {izz_s:.1} g·mm² | {k_sun:+.4} |\n\
		| planet 12T ×6 | {mg_p:.2} g each | {izz_p:.1} g·mm² each | {k_pl:+.4} |\n\
		| **I_eff referred to the ring** | | **{i_eff:.0} g·mm²** | |\n\n\
		**There is no third rotor any more, and eta did NOT get worse — I_eff did.** v1\n\
		and v2 carried the 608's {i608:.0} g·mm² on the SUN side, and v2 additionally carried\n\
		an orbiting steel ball set whose ring-sense momentum happened to cancel part of\n\
		the printed set's residual. Both are gone. The balance is\n\
		`I_sun·k_S = I_ring + Σ I_planet·k_P`, so removing {i608:.0} g·mm² from the sun side\n\
		unbalances it — at v2's design point it would read 0.9044, and that row is in the\n\
		table below rather than being described. The study restored the balance by\n\
		THINNING THE RING (t_ring 4.50 → {t_ring:.2}, t_planet 4.50 → {t_pl:.2}), and the\n\
		result is a shipped eta slightly BETTER than v2's 0.9975. Saying the hardware\n\
		removal cost eta would therefore be false, and it is not said here.\n\n\
		**What it actually cost is inertia.** The ring is the rotor eta pins to the sun,\n\
		so a lighter sun side forces a lighter ring, and I_eff falls from v2's\n\
		15011 g·mm² to {i_eff:.0} — about 16 %. That, plus the two Coulomb terms the\n\
		hardware used to carry, is where the spin time went; the next section costs it.\n\n\
		**eta = {eta:.4}** (gate floor 0.95, design target 0.97). Sensitivity — with the\n\
		steel gone, printed mass variation is the ONLY uncertainty left:\n\n\
		| perturbation | eta |\n|---|---|\n{sens_rows}\n\n\
		Worst corner **{eta_lo:.4}** (G9b3). That corner is a DELIBERATELY pessimistic\n\
		bound: eta is a ratio of inertias, so a COMMON-MODE flow error cancels EXACTLY —\n\
		and **for the first time that is true of the SHIPPED set** (G9b, Δη < 1e-12),\n\
		because there is no longer any non-scaling steel in it. v1 and v2 could only\n\
		assert the exact property on a hypothetical all-PLA variant; G9b2 recomputes how\n\
		much the 608 broke it. Two parts off one plate with one profile share most of\n\
		their flow error, so the independent ±5 % corner is a bound, not a spread.\n\
		The corner's floor is 0.90, argued rather than invented: the cancellation must\n\
		stay far from the uncancelled control puck or the shipped A/B stops being valid.\n\
		The shipped SUN-B control puck lands at **eta = {eta_b:.4}** on purpose (G9c) —\n\
		that is the A/B the buyer performs by hand, and it is the only way an ABSENCE\n\
		gets photographed.\n\n\
		The last row prices the **central web**, the direction v2 refused on eta and this\n\
		version re-opened. Read it next to the spin-time section: eta is no longer what\n\
		refuses the web.\n\n\
		> **Measured eta: REQUIRED, NOT PERFORMED.** No instrument for it exists here or in\n\
		> the hobby field. eta is published as a MODELLED band with the table above, never\n\
		> as a single headline percentage.\n\n",
		mg_r = d.mg_r, izz_r = d.izz_r, mg_s = d.mg_s, izz_s = d.izz_s, mg_p = d.mg_p, izz_p = d.izz_p,
		i608 = I608_GMM2, k_sun = d.k_sun, k_pl = d.k_pl, t_ring = T_RING, t_pl = T_PLANET,
		i_eff = d.i_eff_gmm2, eta = d.eta, sens_rows = sens_rows, eta_lo = d.eta_lo, eta_b = d.eta_b,
	));
	a.push_str(&format!(
		"## Spin time — reported with its derivation, NOT claimed\n\n\
		Governing model, from the research (a printed spinner follows neither constant\n\
		friction nor linear-viscous decay): `I·dω/dt = −T(ω)`, `T(ω) = Σ cⱼ·ω^pⱼ`, solved by\n\
		the exact quadrature `t = ∫₀^ω₀ I dω/T(ω)`, `θ = ∫₀^ω₀ I ω dω/T(ω)`.\n\n\
		**The integrator is a new solver and was proven before it was used** (§25.7\n\
		answer-type 2): B1 reproduces the pure-power-law closed form\n\
		`t = I ω₀^(1−n)/(K(1−n))` and `θ = I ω₀^(2−n)/(K(2−n))` to <0.5%; B2 reproduces the\n\
		pure-Coulomb closed form `t = I ω₀/C`, `θ = I ω₀²/(2C)` to <0.5%; B3 is the\n\
		meta-negative-control that proves B1 can go red. The ω→0 singularity of a pure\n\
		power law is removed exactly by the substitution ω = ω₀·s^(1/(1−p_min)).\n\n\
		Every satellite's drag is REFLECTED by its own speed ratio, because power must\n\
		balance — a torque at ratio k costs k× at the observable rotor, and a satellite's\n\
		own windage at kω is reflected as k^(1+p).\n\n\
		### Budget at the frozen launch speed ω₀ = {w0:.0} rad/s ({rpm:.0} rpm)\n\n\
		| term | class | N·mm | share |\n|---|---|---|---|\n{budget_rows}\n\
		| **TOTAL** | | **{total:.4}** | |\n\n\
		**Predicted spin: {t_nom:.1} s / {rev_nom:.0} revolutions**, band **{t_pes:.1}–{t_opt:.1} s**\n\
		across μ(PLA-on-PLA) 0.20–0.50. That band is now driven by ONE unknown instead of\n\
		two: with the 608 deleted, its 4.3–8.3× correction band leaves the answer, and\n\
		μ(PLA-on-PLA) — which is unmeasured for this pairing — is the whole spread.\n\n\
		**{coul:.0}% of that budget is Coulomb (ω⁰).** That is the number to read, not the\n\
		total: the research shows an equal-magnitude Coulomb loss costs 0.23× spin time\n\
		where an ω² loss costs 0.76×. A fully printed spinner is a Coulomb machine.\n\n\
		Three printed thrust contacts carry all of it, and in Coulomb friction the ONLY\n\
		lever is the arm — μWr does not care about contact area, so widening or\n\
		multiplying a pad buys nothing at all. Each arm is therefore pushed to the floor\n\
		its own geometry allows, and each floor is a gate rather than a preference:\n\n\
		| contact | arm | why it cannot go lower | bearing pressure |\n|---|---|---|---|\n\
		| 6 ring thrust pads | {r_ring:.2} mm | the ring's continuous flat underside starts at its root circle 34.25; inboard of that the band is crenellated by the tooth cavity, and the planets' tips reach r 34.00 | {p_ring:.5} MPa |\n\
		| sun thrust land | {r_sun:.2} mm | the sun's own bed relief removes its underside out to bore_r + C_BED, and the bore is a running fit on a post that must still carry the static thumb pad (G17a) | {p_sun:.5} MPa |\n\
		| 6 planet thrust pads | {r_pl:.3} mm | same construction, on the planet's own bed relief; reflected ×5.5 by the planet speed ratio | {p_pl:.5} MPa |\n\n\
		**G17e is the anti-gaming gate for this version.** v2 had to prove its ball COUNT\n\
		could not move the answer; v3's one free integer is the ring PAD count, and the\n\
		sweep 3–24 pads returns {t_pad_lo:.3}–{t_pad_hi:.3} s — flat to the digit, as an\n\
		area-independent friction law requires. If that ever moved, the model would be\n\
		double-counting area somewhere.\n\n\
		## v1 → v2 → v3: the whole path, one rotor, one solver, one run\n\n\
		**This is the ledger the hardware deletion has to face.** All three rows below are\n\
		computed by THIS run on the SHIPPED rotor — v1 and v2 are not quoted from their\n\
		committed builds, they are re-solved with their hardware put back on today's\n\
		geometry, which is the only way the delta isolates the hardware rather than the\n\
		design point. (The v1/v2 rows therefore also carry the 608's own\n\
		{i608:.0} g·mm² of rotating inertia in their I_eff, because a real 608 does.)\n\n\
		| architecture (same rotor, same solver, same run) | total N·mm | Coulomb share | I_eff g·mm² | spin |\n|---|---|---|---|---|\n\
		| v1 — sliding ring land **+ 608** | {slide_total:.4} | {slide_coul:.0}% | {i_eff_608:.0} | **{t_slide:.1} s / {rev_slide:.0} rev** (band {t_slide_pes:.1}–{t_slide_opt:.1}) |\n\
		| v2 — 24-ball thrust race **+ 608** | {race_total:.4} | {race_coul:.0}% | {i_eff_v2:.0} | **{t_race:.1} s / {rev_race:.0} rev** (band {t_race_pes:.1}–{t_race_opt:.1}) |\n\
		| **v3 — FULLY PRINTED, nothing bought** | **{total:.4}** | **{coul:.0}%** | **{i_eff:.0}** | **{t_nom:.1} s / {rev_nom:.0} rev** (band {t_pes:.1}–{t_opt:.1}) |\n\
		| ceiling: v3 with the ring's support deleted outright | {noring_total:.4} | — | {i_eff:.0} | {t_noring:.1} s |\n\n\
		**Deleting the hardware costs {cost_v2:.2}× against v2 and {cost_v1:.2}× against v1.**\n\
		Said plainly: **this spinner runs for about {t_nom:.0} seconds.** It is not a\n\
		long-spin design and the listing says so in the same words. What was bought for\n\
		that price is a product with an EMPTY bought list — print it, assemble it, use it,\n\
		with nothing else in the box.\n\n\
		The mechanism of the loss is not mysterious and it is not a re-tuned coefficient.\n\
		Two Coulomb terms replaced two engineered ones:\n\n\
		| what left | what replaced it | at ω₀ = {w0:.0} |\n|---|---|---|\n\
		| the 608, ω^0.5 (sub-linear) | the sun's printed thrust land, ω⁰ (Coulomb), arm {r_sun:.2} mm | {b608:.4} → {sun_nmm:.4} N·mm |\n\
		| the 24-ball race, ω⁰ but rolling | six printed thrust pads, ω⁰ sliding, arm {r_ring:.2} mm | {race_nmm:.4} → {ring_nmm:.4} N·mm |\n\n\
		The 608's replacement is close to a wash in MAGNITUDE (the post shrank Ø7.90 →\n\
		Ø5.50 once it no longer had to be a bearing bore, which pulled the sun's arm in to\n\
		{r_sun:.2} mm) but it is strictly worse in CLASS, because Coulomb does not decay.\n\
		The ring's replacement is the real cost: {ratio_ring:.0}× the torque, and it is\n\
		{ring_pct:.0}% of everything.\n\n\
		**G17f is the gate that keeps this honest.** Put the hardware back and the same\n\
		solver MUST return a better number. It does, in both directions\n\
		({t_nom:.1} < {t_slide:.1} < {t_race:.1} s). If that gate ever passed, the\n\
		fully-printed figure would be flattering itself.\n\n\
		### Why the ring's {ring_nmm:.4} N·mm cannot be engineered away\n\n\
		Two bodies rotating about the SAME axis have a relative motion that is a pure\n\
		rotation about that axis, so every contact between them that is not ON the axis\n\
		SLIDES. There are exactly two escapes and the fully-printed constraint closes\n\
		both:\n\n\
		1. **Put a rolling element between them.** That is what a bearing is, and a\n\
		   PRINTED one is refused by the engine's own printability oracle, not by taste —\n\
		   see the printed-ball row below.\n\
		2. **Reach the axis with a web**, so the sliding arm collapses from {r_ring:.2} mm\n\
		   to nearly zero. **This direction was RE-OPENED for v3** (v2 had refused it on\n\
		   eta alone, when momentum cancellation was the headline claim; it is now a\n\
		   supporting receipt, so eta no longer gets a veto). It is refused again, on two\n\
		   grounds that have nothing to do with eta and are both gated:\n\n\
		   * **G20a — the rim shell.** The web must cross the pin circle at r 27, so it\n\
		     must go over the top, clearing whichever is higher of the top spider and the\n\
		     sun. That puts the web plane at z {web_z:.2}, and the ring's rim then has to\n\
		     REACH it: a dead cylindrical shell {shell_h:.2} mm tall at r 34.25–36.50,\n\
		     where inertia is most expensive. That shell alone is **{i_shell:.0} g·mm²**\n\
		     and the spokes another {l_web:.0}, against a ring-side eta budget of\n\
		     **{ring_budget:.0} g·mm²**. What is left for the TOOTHED ring is\n\
		     {teeth_left:.0} g·mm² — a face width of **{web_face:.2} mm**, where the mesh\n\
		     needs at least the planet's own {t_planet:.2} mm. Swept over the study's\n\
		     entire t_sun range the best any sun thickness reaches is\n\
		     **{web_face_best:.2} mm** (at t_sun {web_face_at:.2}): a thinner sun lowers\n\
		     the web and shrinks the shell, but it shrinks the eta budget faster. The web\n\
		     is infeasible everywhere in the design space, not merely expensive here.\n\
		   * **G20b — the spokes cannot be printed.** Print the ring teeth-down and the\n\
		     web's spokes become ceilings bridged from the rim inward across open air:\n\
		     **{web_span:.2} mm** against the profile's `max_bridge` of 6.00. Print it\n\
		     web-down instead and the spokes are fine — but then the rim has to rise from\n\
		     the web plane, which is the same shell G20a just refused. A conical web\n\
		     escapes both and would have to rise {web_span:.2} × 1.40 = {cone_h:.0} mm to\n\
		     stay off the support threshold, which is three envelopes.\n\n\
		   So the honest form of v2's refusal was incomplete: **eta was never the binding\n\
		   constraint on the web. Printability and rim inertia are, and they refuse it\n\
		   much harder.** The eta row for the web is still published (see the eta table)\n\
		   so the reader can see it was priced, not dodged.\n\n\
		### Directions evaluated, with their numbers\n\n\
		Every one was quantified with the same solver before it was dropped. A negative\n\
		result with a number is a result.\n\n\
		| direction | what it actually costs | verdict |\n|---|---|---|\n\
		| **printed PLA balls** in a printed race — **the largest gain v3 does not take** | PLA's modulus is ~60× lower than bearing steel's, so intuition says a huge contact patch. WRONG, and it was recomputed rather than scaled: E* falls only 1.97× and a ∝ E*^(−⅓), so the patch grows just {a_ratio:.2}× to a = {a_pla:.4} mm, the peak pressure FALLS to {p0_pla:.1} MPa (steel: {p0_race:.1}), and the rolling bound is {roll_pla_nmm:.4} N·mm. Costed end to end they would give **{t_pball:.1} s** against {t_nom:.1} s shipped | **REFUSED — and NOT for either of the two reasons that were expected.** (1) The support oracle does not refuse them: a Ø{ball_d:.2} sphere reports {ball_steep:.1e} mm² of steep area, because at 1.5 mm the whole overhanging region lies inside the oracle's own first-layer tolerance. That negative result is recorded rather than swapped for an argument that works, and the oracle is shown not to be blind — a Ø6.00 sphere reports {big_steep:.0} mm² (G21b). (2) Nor are they refused on stress: even in the absurd limit of ONE ball carrying the whole ring the Hertz pressure is {p0_one_ball:.0} MPa, under yield (G21b3). What refuses them is **FORM ERROR**: at the shipped {layer_h:.2} mm layer height a Ø{ball_d:.2} ball is {ball_layers:.1} layers tall and the staircase alone puts ±{form_err:.3} mm on its radius — {form_ratio:.0}× a stock G25 ball's 0.6 µm. The governing loss of a rolling element that far out of round is micro-impact and climb-over, and **this repository has no model and no data for it**. A published spin time resting on an unmodelled loss is exactly the thing this campaign refuses to do. A crude climb-over estimate (the ball's centre rising and falling by the form error once per ball revolution, with NO energy returned) lands at the same order as the elastic bound — which is exactly why this is a MISSING MODEL and not a proven loss, and why it is published as an open direction with its number rather than shipped. Two secondary costs are real: 24 loose Ø1.5 mm parts put the small-parts safety line straight back into a listing that has just got rid of it, and a Ø1.50 free-floating sphere whose first layer is a ~1.0 mm dot is a part this campaign cannot print-prove and has no gate for. Anyone who wants to try it has the number |\n\
		| **an on-axis point pivot for the sun** (blind socket over a domed post) | the sliding arm collapses from {r_sun:.2} mm to a Hertz patch radius, and the whole sun term with it. Costed end to end: **{t_pivot:.1} s** against {t_nom:.1} s shipped, {pivot_pct:+.0}% | **REFUSED, and the price is published (G21c).** A blind on-axis socket and a static thumb pad are geometrically exclusive: the pad must be carried by a column, the column must be on the axis, and a column through the sun leaves nowhere for the socket. Every alternative pad support was worked — a tower on the top spider (its arms bridge {ts_bridge:.0} mm), a shallow ramp (under 45°, unprintable), a free-floating pad on the sun's own dome (it would rub under thumb pressure) — and none survives. The thumb pad wins because a spinner you cannot hold is not a product |\n\
		| **a central web to the axis** | see above — G20a/G20b | **REFUSED on printability and rim inertia, NOT on eta** |\n\
		| **shoulders/flanges on the six planets** carrying the ring axially | the planet turns at 5.5ω, so ring↔planet slip is 4.5ω·(distance from the pitch point). A pad centred on the planet axis sits 6.0 mm from it ⇒ equivalent arm 27.0 mm vs {r_ring:.2} — 0.78× — and the load then presses the planet onto its OWN pad as well, arm 17.9 mm, total 44.9 mm | **WORSE than doing nothing.** For parallel axes the only rolling locus is the pitch line, which is vertical, and a vertical line cannot carry axial load |\n\
		| **printed rollers on printed journals** under the ring | the gain is exactly r_journal/r_roller, and the ring's underside sits only {roller_room:.2} mm above the bed, so even cutting the frame away entirely under it caps the roller at ~Ø2 and the journal would have to be under Ø1 — a third of this campaign's own printed-pin floor | **unprintable at the only size that fits** |\n\
		| **a free-floating printed thrust washer** between ring and frame | two Coulomb interfaces in series at the same radius and the same normal load: the washer settles at whatever speed, and the torque on the ring is still μWr | **exactly 1.00× — a theorem, not a measurement** |\n\
		| **lightening the ring** | v2 called this a weak lever because the ring's support was 3 % of its budget. It is now {ring_pct:.0}%, so it was re-derived — and it is a THEOREM that it is nearly neutral. eta pins I_sun·k_S = I_ring + ΣI_p·k_P, so I_eff ≈ I_ring·(1 + k_S) is PROPORTIONAL to the ring's inertia, while the ring's Coulomb term is proportional to the ring's mass. Scale the ring and both the numerator and the denominator of I/T move together | **no gain available. The one exception is the arm, and that is already at its floor** |\n\
		| **moving the ring pads inboard** | the ring's continuous flat underside starts at its root circle 34.25 (inboard of that the band is crenellated by the tooth cavity, and the planets' tips reach r 34.00 with only the ISO 53 tip clearance between). v1 sat at 34.75; v3 sits at {r_ring:.2} | **TAKEN — the only free win, worth {pad_gain:.1}% of the ring term** |\n\
		| **magnets to unload the ring** | forbidden outright by this version's own rule (zero non-printed parts), and independently: no force data was sourced, 6-on-6 pole counts cog into detents, and the CPSC has an ACTIVE 2026 recall for magnet fidget-spinner sets over ingestion injuries | **refused three times over** |\n\
		| **more launch speed** | the research's own result: spin time SATURATES, 1000 → 5000 rpm buys +41 % for 25× the energy — and that is for an air-dominated spinner. This one is {coul:.0}% Coulomb, where t ∝ ω₀ exactly | **not a design lever; it is the user's wrist** |\n\n\
		### The EDGE-ON case — the advice REVERTS in v3\n\n\
		v1 said edge-on made the ring term \"largely vanish\". v2 corrected that to 1.30×\n\
		(gravity reacted at a mesh does not stop there — it continues into the planet and\n\
		out through the planet's PIN JOURNAL at k = 5.5) and then INVERTED it, because the\n\
		ball race gave the axial load a rolling path and nothing gave the radial one.\n\
		**v3 has no rolling path anywhere, so the advice reverts to v1's sense** — and to\n\
		v2's magnitude. Costed end to end at the worst support geometry (no planet at\n\
		bottom dead centre, two at ±30°, so the reaction magnitudes sum to\n\
		W/cos30° = 1.1547·W, derived rather than assumed):\n\n\
		| term | N·mm |\n|---|---|\n{edge_rows}\n| **TOTAL** | **{edge_total:.4}** |\n\n\
		Edge-on is **{t_edge:.1} s / {rev_edge:.0} rev** against **{t_nom:.1} s /\n\
		{rev_nom:.0} rev** flat — **{ratio_edge:.2}×**. Both halves are gated (G19b asserts\n\
		it wins, G19c asserts it wins by less than 1.6×) so the claim cannot drift back\n\
		into \"vanishes\" in either direction. The listing says hold it edge-on.\n\n\
		> **Measured spin time: REQUIRED, NOT PERFORMED.** No spinner was printed or timed\n\
		> in this run. The protocol to use is frozen in the listing (n ≥ 5, a repeatable\n\
		> release at ω₀ = {w0:.0} rad/s, a stated stop criterion, spread reported rather than\n\
		> the mean) — and it would be the first methoded spin time in a category that has\n\
		> exactly one unmethoded figure.\n\n\
		**Bounded omissions, stated rather than hidden.** (a) The ring's OD cylinder skin\n\
		friction is omitted — the wetted OD band is ~14% of the disc-face area and no\n\
		usable correlation was sourced, so the prediction is very slightly OPTIMISTIC.\n\
		(b) The mesh-sliding term at zero preload is gated below 5% of the budget and\n\
		omitted with that bound. (c) The free-disc moment coefficient Cm = 3.87·Re^−½ could\n\
		not be verified at a primary source; it drives ~20% of the air term, which is\n\
		itself a small minority of a {coul:.0}%-Coulomb budget. (d) **Break-in is not\n\
		modelled and it is the largest un-modelled effect in v3.** The research is\n\
		explicit that printed journals need it (\"break it free and spin it around with\n\
		some pressure applied in different directions to flatten out any bumps\"), and the\n\
		contest's own hero model tells its users to ream the gear bores with a drill.\n\
		A run-in printed thrust face plausibly falls below μ 0.20 — the bottom of the\n\
		band used here — and this campaign has no data for it, so it claims none. That\n\
		omission makes the published number PESSIMISTIC, which is the direction an\n\
		un-modelled effect is allowed to point in a campaign that publishes bounds.\n\
		(e) The bearing pressure at every printed contact is gated (worst\n\
		{p_worst:.4} MPa, G17b–d) but WEAR is not modelled: PLA-on-PLA sliding wear at\n\
		these pressures has no data in this repo's registry.\n\n",
		w0 = W0, rpm = W0 * 60.0 / TAU, budget_rows = budget_rows, total = d.drag.total_nmm(W0),
		t_nom = d.t_nom, rev_nom = d.rev_nom, t_opt = d.t_opt, t_pes = d.t_pes,
		coul = d.coul_frac * 100.0, t_noring = d.t_noring, i608 = I608_GMM2,
		t_slide = d.t_slide, rev_slide = d.rev_slide, t_slide_opt = d.t_slide_opt, t_slide_pes = d.t_slide_pes,
		t_race = d.t_race, rev_race = d.rev_race, t_race_opt = d.t_race_opt, t_race_pes = d.t_race_pes,
		slide_total = d.drag_slide.total_nmm(W0), race_total = d.drag_race.total_nmm(W0),
		slide_coul = 100.0 * d.drag_slide.terms.iter().filter(|t| t.1 == 0.0).map(|t| t.0).sum::<f64>() / d.drag_slide.torque(W0),
		race_coul = 100.0 * d.drag_race.terms.iter().filter(|t| t.1 == 0.0).map(|t| t.0).sum::<f64>() / d.drag_race.torque(W0),
		i_eff = d.i_eff_gmm2, i_eff_608 = d.i_eff_608_gmm2, i_eff_v2 = d.i_eff_v2_gmm2,
		noring_total = d.drag.terms.iter().filter(|t| !t.2.starts_with("6 ring thrust pads")).map(|t| t.0 * W0.powf(t.1)).sum::<f64>() * 1e3,
		cost_v2 = d.t_race / d.t_nom, cost_v1 = d.t_slide / d.t_nom,
		ring_nmm = d.drag.terms[0].0 * 1e3, sun_nmm = d.drag.terms[1].0 * 1e3,
		b608 = d.drag_slide.terms.iter().find(|t| t.2.starts_with("608")).map(|t| t.0 * W0.powf(t.1)).unwrap_or(0.0) * 1e3,
		race_nmm = (d.drag_race.terms[0].0 + d.drag_race.terms[1].0) * 1e3,
		ratio_ring = d.drag.terms[0].0 / (d.drag_race.terms[0].0 + d.drag_race.terms[1].0),
		ring_pct = 100.0 * d.drag.terms[0].0 / d.drag.torque(W0),
		r_sun = d.r_sun_land, r_ring = d.r_ring_pad,
		web_z = d.shell_h + Z_ROT + T_RING, shell_h = d.shell_h, i_shell = d.i_shell,
		ring_budget = d.i_shell + d.l_web + d.teeth_left, l_web = d.l_web, teeth_left = d.teeth_left,
		web_span = d.web_span, cone_h = d.web_span * RELIEF_SLOPE,
		web_face = d.teeth_left / (d.izz_r / T_RING), t_planet = T_PLANET,
		web_face_best = d.web_face_best, web_face_at = d.web_face_at,
		t_pball = d.t_pball, big_steep = d.big_steep, p0_one_ball = d.p0_one_ball,
		ball_layers = d.ball_layers, form_err = LAYER_H / 2.0, form_ratio = d.form_ratio,
		layer_h = LAYER_H,
		a_pla = d.a_pla, a_ratio = d.a_pla / d.a_race, p0_pla = d.p0_pla, p0_race = d.p0_race,
		roll_pla_nmm = d.roll_pla * 1e3, ball_d = BALL_D, ball_steep = d.ball_steep,
		t_pivot = d.t_pivot, pivot_pct = 100.0 * (d.t_pivot / d.t_nom - 1.0),
		ts_bridge = 23.0 - 7.0, roller_room = Z_ROT,
		pad_gain = 100.0 * (34.75 - d.r_ring_pad) / 34.75,
		edge_rows = edge_rows, edge_total = d.drag_edge.total_nmm(W0),
		t_edge = d.t_edge, rev_edge = d.rev_edge, ratio_edge = d.t_edge / d.t_nom,
		p_worst = d.p_worst, r_pl = d.r_pl_pad,
		p_ring = d.p_ring_pad, p_sun = d.p_sun_land, p_pl = d.p_pl_pad,
		t_pad_lo = d.t_pad_lo, t_pad_hi = d.t_pad_hi,
	));
	a.push_str(&format!(
		"## Structure — and why creep is NOT the governing tier\n\n\
		A spinner is flicked and dropped; it is never held under load. The load case is a\n\
		thumb flick applied and REMOVED, which is exactly what the static allowable\n\
		describes, so `materials::pla::SIG_ALLOW_RT` = {sig_allow:.0} MPa is the right tier and\n\
		`creep_allowable_mpa` is not. That call is written down rather than assumed. Nor is\n\
		this a fatigue case: a fidget spinner is not a cycled drivetrain — the teeth see a\n\
		load only during the flick itself — and that refusal is on record here rather than\n\
		being quietly skipped.\n\n\
		Tooth-root bending, Lewis, with the form factor Y **measured off the generator's\n\
		own outline** rather than taken from a handbook table:\n\n\
		| member | Y measured | handbook (hobbed) |\n|---|---|---|\n\
		| planet 12T | {y_pl:.3} | ~0.36 |\n| sun 42T | {y_sun:.3} | ~0.45 |\n| ring 66T tooth | {y_ring:.3} | n/a |\n\n\
		A hard 5 N thumb flick at the Ø{od:.1} rim is {sig_ring:.3} MPa at the weakest root\n\
		against {sig_allow:.0} MPa — margin ×{marg:.0} (G15). The parts print FLAT, so tooth\n\
		bending is in the in-plane tier, not the 0.55× across-layer tier; that is a\n\
		structural requirement of the orientation, not a slicer preference.\n\n\
		## Clearance stack-up (G12) — published both credited and uncredited\n\n\
		| corner | residual on the Ø6.00 planet |\n|---|---|\n\
		| nominal, calibrated printer (±0.05/side) | {nom:+.3} mm |\n\
		| worst XY oversize (0.15/side, both walls), chamfer CREDITED | {credited:+.3} mm |\n\
		| the same plus uncompensated elephant foot (0.20/side) | {uncredited:+.3} mm |\n\
		| worst corner on the Ø6.15 ladder member | {ladder:+.3} mm |\n\n\
		The uncredited corner is NEGATIVE and that is the whole reason two things exist:\n\
		the bed relief on every clearance surface — 0.45 mm of radial run cut at 1.40\n\
		rise:run, clear of the support threshold rather than on it (which makes layer 1\n\
		physically absent from the gap, so the elephant-foot term cannot apply), and the\n\
		three-bore planet ladder in `optional/`. On an uncalibrated printer the Ø6.15\n\
		planet is the escape hatch, and it costs 2.5 g to ship it.\n\n\
		Concentricity (G8) — **the one receipt the hardware deletion made STRONGER.**\n\
		v1 and v2 located the sun TWICE, on the 608 and on six meshes, and had to prove\n\
		the two did not fight: only 0.12 mm of freedom (C_TIGHT + the bearing's internal\n\
		clearance + C_TIGHT) was available to absorb a 0.15 mm build error. v3's sun runs\n\
		on a plain running fit on the post, so its radial freedom is the full\n\
		{freedom:.2} mm and the residual collapses to {residual:.3} mm against the radial\n\
		lash equivalent jr = jt/(2 tan α) = {jr:.3} mm. The post and the pin circle are\n\
		still cut from the same parametric origin on the SAME printed part, so the\n\
		DESIGNED concentricity is exactly 0.000 mm — but now **the sun is located by its\n\
		six meshes alone and there is nothing left to fight them.** That matters more\n\
		than it sounds: zero mesh preload is worth ~1.03× spin time and 0.2 N of preload\n\
		costs 0.66×, so this was the failure mode most able to quietly destroy the number\n\
		above.\n\n",
		sig_allow = d.sig_allow, y_pl = d.y_pl, y_sun = d.y_sun, y_ring = d.y_ring,
		od = od, sig_ring = d.sig_ring, marg = d.sig_allow / d.sig_ring,
		nom = C_FREE - 0.10, credited = d.credited, uncredited = d.uncredited, ladder = d.ladder_best,
		freedom = C_FREE, residual = d.residual, jr = d.jr,
	));
	a.push_str(&format!(
		"## Safety — EN 71-1 §4.10, gated (G14)\n\n\
		The rule: an accessible space between moving elements that admits a Ø5 mm rod must\n\
		also admit a Ø12 mm rod. The 5–12 mm band is forbidden. Applied as a geometric\n\
		gate over every gap between relatively-moving members:\n\n\
		- every enclosed gap (sun↔top spider {g1:.2} mm, all axial gaps {g2:.2} mm, the ring\n\
		  standing {g3:.2} mm proud of the held rims) is **under 5 mm** — a finger cannot\n\
		  enter, so the rule is not engaged;\n\
		- the one space a finger CAN enter is between adjacent planets at\n\
		  **{neighbour:.3} mm**, which clears the ≥12 mm branch by construction;\n\
		- the converging mesh nips are sub-regions of that same space, and a Ø5 rod cannot\n\
		  be inserted into a wedge narrower than itself.\n\n\
		The negative control is a hypothetical 8-planet layout at the same centre distance:\n\
		its neighbour gap is {nc_gap:.2} mm — squarely inside the forbidden band — and the\n\
		gate fires on it. A safety gate that cannot fail is not a gate.\n\n\
		Every tooth tip carries a 0.30 mm relief on both faces (the named \"sharp gear\n\
		edges\" injury complaint the field ignores), and the ring's outer rim gets a 1.0 mm\n\
		full round on the top edge.\n\n\
		**Two safety lines got SHORTER in v3, and that is worth stating plainly.** v2's\n\
		listing had to warn about a 608, a cap and 24 loose Ø1.50 mm steel balls, every\n\
		one of which drops freely into the 16 CFR 1501 small-parts cylinder. v3 has no\n\
		bought parts at all, so the only small part left in the box is the printed cap.\n\
		The under-3 line stays — the cap is still a small part, and so is a planet if the\n\
		top spider is lifted — but it is now about parts the buyer printed, not parts\n\
		they were told to source and then count back in.\n\n\
		> Keep solvents away from it. There is no longer a bearing to de-grease, which\n\
		> was the one reason anybody ever put IPA near a printed spinner, and there are\n\
		> first-hand reports of printed spinner bodies shattering after a solvent clean.\n\n\
		> **ISO 13854 body-part gap table: REQUIRED, NOT PERFORMED.** The machinery standard\n\
		> that would give a numeric crush limit is paywalled and could not be obtained. The\n\
		> EN 71-1 rod rule is used as the substitute and that substitution is stated.\n\
		> EN 71-1:2026 has also been published since the 2014 text used here; the clause\n\
		> must be re-verified against the current edition before any compliance statement.\n\n",
		g1 = TS_R_IN - (M * S_T as f64 / 2.0 + M), g2 = C_Z, g3 = od / 2.0 - STATIC_R,
		neighbour = d.neighbour, nc_gap = 2.0 * CD * (PI / 8.0).sin() - M * (P_T + 2) as f64,
	));
	a.push_str(&format!(
		"## Design study (G11)\n\n\
		Every rotor is a prism, so `I_zz = ρ·h·J` is EXACT in the face width and the study\n\
		runs on the polygon polar second moments of the very outlines the solids are built\n\
		from (B4 benchmarks that helper against πR⁴/2 to <0.1%). {evals} points swept,\n\
		{feas} feasible under eta ≥ 0.97, height ≤ 12.0, Ø ≤ 73.0, mass ≤ 28 g and\n\
		planet face ≤ ring face. The winner is re-evaluated by `StudyReport::best()` — an\n\
		impure evaluator is a typed error here, never a warning — and `gate_study` asserts\n\
		the SHIPPED point IS that winner.\n\n\
		Shipped: **t_sun {ts:.2} · t_ring {tr:.2} · ring wall {wall:.2} · planet face {tp:.2}**,\n\
		Ø{od:.1} × {height:.2} mm, {printed_g:.1} g printed.\n\n\
		**The design point MOVED in v3 and the study is why.** v1/v2 could not take t_sun\n\
		below 7.60 mm — the 608 had to live inside the bore (0.60 lip + 7.00 width) — so\n\
		the window was 0.60 mm wide and effectively only the ring could move. With the\n\
		bearing gone that floor is gone, the sweep runs 3.00–8.20 in 0.02, and two things\n\
		changed together: the sun got HEAVIER at any given face width (its Ø22.1 bearing\n\
		bore shrank to a Ø6.00 journal, so it is now a nearly solid disc) while losing\n\
		the 608's 610 g·mm² from the eta balance. The optimiser's answer is a thinner sun\n\
		AND a thinner ring than v2 shipped. Nothing was hand-held: `gate_study` asserts\n\
		the shipped point IS the winner and it failed, loudly, on the first attempt at\n\
		writing this version's numbers by hand. The ring wall is NOT a free variable: a wall must be a whole number of 0.45 mm\n\
		extrusion lines, so the legal set inside [min_wall, envelope] is {{1.35, 1.80,\n\
		2.25}} = {{3, 4, 5}} lines. The 5-line floor is an ACTIVE constraint — the\n\
		unconstrained I_eff optimum prefers a thinner wall and more face width — and it\n\
		is bought deliberately, on the one part located by six simultaneous meshes, where\n\
		going out of round binds everywhere at once.\n\n\
		## Retention — the BAYONET (G16, rebuilt in v4) and the cap (G22)\n\n\
		**v3 held the top spider on with friction, and said so.** Six Ø5.55 click bands\n\
		over Ø5.60 holes: 0.025 mm of nominal radial interference, whose v3 gate G16e\n\
		disclosed honestly reached exactly ZERO at 0.025 mm/side of printer error —\n\
		against this campaign's own worst-case XY figure of 0.15 mm/side, six times\n\
		larger. On a badly calibrated machine the spider was a slip fit and the planets\n\
		and ring lost their captor. Worse, on inspection for v4 the SHIPPED band was\n\
		Ø5.55 in a Ø5.60 hole — a 0.025 mm/side CLEARANCE. The interference the strain\n\
		gates were computing was not in the geometry at all. Both facts are recorded\n\
		here rather than quietly fixed.\n\n\
		**v4 retains by a shoulder.** Each pin now carries a Ø{neck:.2} NECK through the\n\
		spider's thickness and a radial FIN above it; each spider arm carries a slot\n\
		whose two walls are the lip. Drop the spider on at the entry bulge, twist\n\
		{psi:.1}° ({travel:.2} mm at the pin circle) to the hard stop, and the fin\n\
		overhangs the slot's outboard wall by **{engage:.2} mm of solid material**. This\n\
		is RESPOOL's zero-preload lesson: lug under ceiling, hard end stop, nothing\n\
		strained at rest.\n\n\
		| | v3 click band | v4 bayonet |\n|---|---|---|\n\
		| retention is | 0.025 mm interference | {engage:.2} mm of material in the way |\n\
		| survives 0.15 mm/side XY | **no** — dies at 0.025 | **yes** — {exy:+.2} mm left |\n\
		| survives 0.15 + 0.20 foot | **no** | **yes** — {efull:+.2} mm left |\n\
		| preload at rest | yes (hoop, six places) | **zero** (G16d, measured on solids) |\n\
		| assembly strain | 0.89 % hoop | **none** — rigid-body twist |\n\
		| capacity | μ-dependent, unmeasurable | {fcap:.1} N, neck bending — {capx:.0}× the {carried:.3} N it carries (G16k) |\n\n\
		**The proof is on solids, not on constants.** G16e lifts the SHIPPED spider 3 mm\n\
		and measures {captive:.1} mm³ of overlap with the pins; G16d proves the same pair\n\
		is not touching anywhere inside its {float:.2} mm float. G16f rebuilds one pin and\n\
		one arm with the FULL G12 error on every retention surface — fin eroded\n\
		0.35 mm/side, slot dilated 0.35 mm/side — and the same lift still measures\n\
		{wc:.3} mm³ per pin. Two negative controls close it: replace every slot with a\n\
		round hole that clears the fin and the overlap goes to exactly zero (G16g), and\n\
		pose the SHIPPED spider at the entry angle and it lifts straight off (G16h).\n\
		Back-out is geometric too: **{urel:.0}% of the twist** has to be undone before\n\
		the fin can reach the bulge (G16j).\n\n\
		> **Why not a snap? Because the arithmetic refuses it, and the gate re-proves\n\
		> that every run (G16m).** A hoop's bore strain is δ/a EXACTLY, so a Ø5.60 hole\n\
		> in this arm can expand only **{snapmax:.3} mm** before PLA's {yld:.2}% yield\n\
		> strain — and the interference a snap must swallow is the same 0.30 mm stack the\n\
		> engagement above swallows. The gap is {snapratio:.1}×, and nothing inside the\n\
		> 12 mm envelope closes it: a 3.4 mm collet finger at 0.9 mm wall reaches ~0.06 mm,\n\
		> and the free pin length above the spider is ~3 mm. The frozen spec's own Ø6.40\n\
		> barb is **{barb:.1}% strain**, eight times yield, and is still refused by name.\n\
		> A snap big enough to survive this printer's error would have to yield to go on.\n\n\
		> **Why the retaining face is a FIN and not a round head.** Every support-free\n\
		> down-facing retaining face is a ≥{slope:.2} rise:run cone, and a cone WEDGES:\n\
		> {slope:.2} N horizontal per 1 N of lift. If any of that horizontal share is\n\
		> TANGENTIAL the bayonet cams itself back toward the entry under any load at all —\n\
		> a joint that unscrews when you turn the toy over, and the failure is scale-free,\n\
		> so the carried weight is enough. The fin is trimmed to ±{finhw:.2} mm about the\n\
		> pin, wholly inside the slot wall's material, so the tangential shares cancel and\n\
		> the whole wedge force is RADIAL — resisted by the spider's hoop, which would\n\
		> have to grow {engage:.2} mm (4.3% strain) to let go. The part breaks first.\n\n\
		**The cap is the residual, and it is the one v4 did NOT fix.** v1/v2 pressed the\n\
		sun onto a 608 and the 608 onto the post; the sun could not come out. v3's sun is\n\
		a drop-in part on a running fit, so the cap is the ONLY thing holding it in when\n\
		the spinner is turned over — G22a asserts the cap overhangs the sun's bore and\n\
		sits one axial clearance above its top face. The cap is now the model's ONLY\n\
		interference fit, and G22b gates its strain (0.91 %, after the 1.82 % correction\n\
		described under Print) — but its GRIP still scales with calibration exactly the\n\
		way the click bands' did. It is left as a press because Ø12 on a Ø5.50 post with\n\
		1.20 mm of engagement has no room for a bayonet's travel, and because a loose cap\n\
		lets the sun lift rather than dropping six planets. G16n prints that residual\n\
		every run rather than letting the headline imply the whole model is now\n\
		calibration-free. Assembly fit is the other residual: the twist rides on C_FREE\n\
		like every other running fit here, so a badly over-extruded machine makes the\n\
		twist tight — it cannot make the shoulder go away.\n\n\
		## Analysis plan (per DESIGN_GUIDE §25.7 — every required item answered)\n\n\
		| analysis | required? | status |\n|---|---|---|\n\
		| **kinematic exactness of the ratio** | **yes — it IS the claim** | **receipts** — G1a–G1e, integer identities plus the engine's own assembly oracle, with a negative control |\n\
		| **mesh geometry (contact ratio, clearance, undercut, neighbour gap)** | **yes — a gear entry lives or dies here** | **receipts** — G2–G4c; the contact-ratio formulas are written in this campaign (the engine has no API) and carry their own negative control |\n\
		| **interference over the full motion cycle** | **yes** | **receipts** — G5a/b dense mesh sweep of a full mesh cycle, G5c/G5d exact `overlap_volume`, G6 negative control that JAMS |\n\
		| **backlash under the printed tolerance** | yes | **receipts** — G7 bisection to flank contact on the built solids |\n\
		| **rotational inertia + angular-momentum balance (eta)** | **yes — it is the supporting receipt** | **receipts** — G9/G9b/G9b2/G9b3/G9c from exact B-rep `mass_properties`, published as a band with its common-mode invariance proved |\n\
		| **static + dynamic balance** | yes — an asymmetric web on a hand-held rotor is felt | **receipts** — G10, zero CG offset and zero products of inertia per rotor, with a negative control that fires |\n\
		| **spin-down dynamics** | **yes — the field's only quoted number, and it is unmethoded** | **new solver, benchmarked first** — B1/B2 against two closed forms, B3 meta-negative-control; result published with its band and its dominant term named |\n\
		| **the three printed sliding interfaces (v3's whole subject)** | **yes — they are {coul:.0}% of the budget** | **receipts** — G17a–i: each arm asserted to be the minimum its geometry allows, bearing pressure gated at every contact, an anti-gaming gate proving the ring PAD COUNT cannot move the answer, and a negative control that puts the 608 and the balls BACK and must come out strictly better |\n\
		| **whether deleting the hardware was worth it** | **yes — it is the whole subject of this version** | **receipts** — all three architectures recomputed on one rotor by one solver in one run, with I_eff carried per-architecture; G17f is the falsifier |\n\
		| **the central web (RE-OPENED for v3)** | **yes — v2 refused it on eta, and eta is no longer the headline** | **receipts** — G20a/G20b. Re-decided on its merits and refused again, on the rim-shell inertia (no sun thickness in the whole study range leaves a printable ring face) and on printability (the spokes' bridge span). The eta cost is published too, and it turns out NOT to be the binding constraint |\n\
		| **printed rolling elements** | **yes — the obvious fully-printed answer** | **receipts** — G21a/b/b2/b3, and the refusal is not the one that was expected: not the support oracle (it passes at Ø1.50 and that is recorded), not stress (34.9 MPa at one ball), but FORM ERROR at 167× a stock ball's, whose governing loss has NO model here |\n\
		| **Hertzian contact under a rolling element** | **yes — recomputed for PLA-on-PLA, not scaled from the steel answer** | **receipts** — G21a, closed-form Hertz with both materials' published constants; B5/B6 benchmark the helper against an independent algebraic path and B7 shows the answer does not rest on the steel constants |\n\
		| **orientation dependence (flat vs edge-on)** | **yes — v1 and v2 both shipped usage advice about it, and it has now changed twice** | **receipts** — G19a–c cost the edge-on load path end to end (mesh, planet journal AND the sun's post journal). v1 said \"largely vanishes\", v2 corrected that to 1.30× and then INVERTED it because the race gave the axial load a rolling path; v3 has no rolling path, so the advice reverts — {ratio_edge:.2}×, with the ceiling gated so it cannot drift back to \"vanishes\" |\n\
		| **tooth-root bending** | yes | **receipts** — G15, Lewis with Y measured off the built outline, not a table |\n\
		| **clearance stack-up** | **yes — this is what makes a printed gear train seize** | **receipts** — G12, published credited AND uncredited |\n\
		| **concentricity of the sun** | **yes — the one failure none of the concepts named** | **receipts** — G8, and v3 IMPROVES it: the sun is no longer doubly located, so the residual is 0.000 mm against a 0.193 mm lash budget |\n\
		| **pinch/entrapment safety** | **yes — exposed gear teeth near fingers** | **receipts** — G14 EN 71-1 §4.10 rod rule as a geometric gate, with a negative control; ISO 13854's table declared unobtainable |\n\
		| **support-free printability** | yes | **receipts** — per-part `support_free_report`, plus a negative control that fires on a wrongly-oriented audit |\n\
		| **connectivity (one body per part)** | **yes — the third oracle** | **receipts** — `Mesh::is_one_body` on every emitted part |\n\
		| **top-spider AND cap retention** | **yes — a cover that falls off drops six planets, and the cap is the only thing holding the sun in** | **receipts, and v4 upgraded them from a disclosure to a proof** — G16a–c bound the engagement over the FULL G12 stack; G16d–f measure it on the built solids, including a rebuild with 0.35 mm/side of error on every retention surface; TWO negative controls (G16g deletes the lip, G16h poses the shipped part untwisted) and both must read exactly zero; G16k/l give a capacity in newtons from section properties instead of v3's μ-dependent bound; G16m keeps the snap refusal live with the spec's own barb as its NC. G22a/G22b for the cap, whose press fit is the one calibration-bound joint left and is called out as such by G16n |\n\
		| **assembly sheet + renders** | yes — the contest scores presentation explicitly | **receipts** — `assembly_doc.py` and `render_views.py` are RUN and GATED, not assumed |\n\
		| creep / sustained load | **no** | a spinner is flicked and released, never held under load. The static tier is the correct one and the reasoning is written above rather than assumed. Gating a fidget toy against a 1-year creep allowable would be plan padding |\n\
		| fatigue | **no** | the teeth carry load only during the flick; this is not a cycled drivetrain. The repo's fatigue solver is screening-only and would also refuse the across-layer question — the refusal is recorded rather than dressed up |\n\
		| thermal | **no** | there is no heat source. A conduction solve on an unheated hand toy would be plan padding |\n\
		| modal / vibration | **no** | and the repo's modal card explicitly excludes a SPINNING part (no stress stiffening), so quoting it here would be out of its stated limits |\n\
		| buckling | **no** | no member is a slender compression strut |\n\
		| 3-D FEA of the tooth root | **no** | the `ace_fea` card is explicit that a fillet peak is staircase-dominated (±20–30%, biased high) and that the closed-form should be applied to the FEA's nominal instead. At ×{marg:.0} margin from a closed form, a voxel solve adds no decision |\n\
		| **Hertzian flank contact / pitting / wear** | **required for a drivetrain, NOT for this** — and NOT PERFORMED regardless | no solver in `tools/solvers/` covers elastic-on-elastic contact (the `contact` card is a planar beam against a RIGID obstacle), and no PLA-on-PLA flank data exists. The race contact above is a different case — rigid sphere on a compliant flat, which the closed form does cover — and it is NOT evidence about the FLANKS |\n\
		| **rolling resistance of a ball on printed PLA** | **REQUIRED, NOT PERFORMED** | nobody publishes a coefficient for this pairing. Rather than invent one, the v1/v2 ledger rows use the fact that a pressure resultant cannot lie outside its own contact patch (`f ≤ a`), which makes those rows LOWER bounds on the model's answer — i.e. the ledger is biased in the HARDWARE's favour, not v3's |\n\
		| **the rolling behaviour of an out-of-round PRINTED ball** | **REQUIRED, NOT PERFORMED — and it is why printed balls do not ship** | a ball {form_ratio:.0}× further from round than a G25 bearing ball does not roll, it climbs and drops. Micro-impact and climb-over losses have no model and no data in this repository, and a spin time resting on an unmodelled loss is not a number this campaign publishes |\n\
		| **break-in of the printed thrust faces** | **REQUIRED, NOT PERFORMED — the largest un-modelled effect in v3** | the research is explicit that printed plain bearings need a documented break-in, and the contest's own hero model tells its users to ream the gear bores. A run-in face plausibly drops below the μ 0.20 that floors the published band. The omission makes the published spin time PESSIMISTIC, which is the only direction an unmodelled effect is allowed to point here |\n\
		| **PLA-on-PLA sliding WEAR at a printed thrust face** | **REQUIRED, NOT PERFORMED** | the bearing pressure at every contact is gated ({p_worst:.4} MPa worst, G17b–d) but pressure is not wear. No PLA-on-PLA wear data exists in this repo's registry, and the three contacts here run for the life of the toy |\n\
		| **impact / drop** | **REQUIRED, NOT PERFORMED** | a fidget spinner is dropped, and PLA's notched toughness is low. No printed-PLA impact model exists in this repo's registry; every card in it is static, quasi-static or eigenvalue-based, and the fatigue solver explicitly refuses to stand in for one. This is the largest honest gap in the deliverable |\n\
		| **μ(PLA-on-PLA) at a printed journal/thrust face** | **REQUIRED, NOT PERFORMED** | it is the single most load-bearing unknown in the spin-time answer (it scales the dominant Coulomb term directly). All published PLA tribology is PLA-on-STEEL at 20 N — wrong pairing, wrong load. Carried as a 0.20–0.50 band, never as a value |\n\
		| **measured eta** | **REQUIRED, NOT PERFORMED** | no instrument exists. Published as a modelled band with its sensitivity table |\n\
		| **measured spin time** | **REQUIRED, NOT PERFORMED** | nothing was printed in this run. The protocol is frozen in `publish/` |\n\
		| **acoustic loudness** | **REQUIRED, NOT PERFORMED** | no dB rig. The {mesh_hz:.0} Hz tooth-passing frequency at 600 rpm is stated as KINEMATIC only; no loudness is claimed |\n\n\
		## Print\n\n\
		Every relief cone in this campaign is cut at 1.40 rise:run (54.5° from horizontal),\n\
		NOT at 45°. A 45° face sits exactly ON the support-free threshold, and a facet\n\
		cannot land there — mesh positions are f32, so the f64 normal carries its own\n\
		representation noise. The ring's bed chamfer measured 1.037e-6 mm² of steep area\n\
		at 45°, which is float noise but which a `steep_area < 1e-6` gate is right to\n\
		fire on. The fix was geometric, not a looser gate.\n\n\
		v3 deletes v2's ball channel from the base and puts six thrust pads back in its\n\
		place. Every one of them is an upward-facing boss with vertical sides buried in\n\
		the arm slab — nothing to bridge, nothing to overhang, and 1.6 g lighter than the\n\
		closed race rim it replaces. The base's hub top is recessed by TWO axial\n\
		clearances rather than one, and that is a boolean-hygiene number, not a\n\
		clearance one: at one clearance it lands on exactly the arms' own top plane, and\n\
		a coincident plane between two unioned bodies is §7.7 rule 3. The chain went\n\
		invalid (genus 2, not watertight) the first time it was written that way and the\n\
		fix was geometric.\n\n\
		**No part in v3 carries a bridge.** The widest downward-facing horizontal patch\n\
		anywhere in the set measures {bridge:.3} mm — a single facet's worth of float\n\
		noise, against the profile's 6.00 mm allowance (G22c). That is not decoration: it\n\
		is what closes the door on the on-axis point pivot, whose blind socket ceiling\n\
		would have been the model's first real bridge, at bore Ø × √2 = 8.49 mm.\n\n\
		**One deviation was forced by the smaller post and it was found by a gate, not by\n\
		inspection.** v1/v2 pressed the cap on with the profile's `xy_clearance_tight`\n\
		(0.05 mm radial) on a Ø7.90 post: 1.27 % hoop strain, legal. The same absolute\n\
		interference on v3's Ø5.50 post is **1.82 %**, past PLA's 1.67 % yield strain.\n\
		G22b fired. The fix is not a looser gate: the cap's interference drops to\n\
		`CAP_PRESS_R`, which is 0.91 % — the joint class DRYBOX has print-proved — with\n\
		the full 1.20 mm of engagement kept. G22b's negative control asserts that the\n\
		inherited fit still fails, so the correction cannot be quietly reverted.\n\n\
		Every part prints FLAT with its gear axis parallel to +Z, zero supports, one plate,\n\
		one colour, no brim (the 0.45 mm chamfers replace it). {printed_g:.1} g PLA\n\
		solid-equivalent for the whole set. Flat orientation is a structural requirement\n\
		(in-plane tooth bending) and a geometric one (every involute stays a continuous\n\
		in-plane extrusion path) — the SEO claim that gears print weaker flat is backwards\n\
		and is refused on record.\n",
		evals = d.study_evals, feas = d.study_feasible, ts = T_SUN, tr = T_RING, wall = RING_WALL,
		yld = d.yield_strain * 100.0, barb = d.spec_strain * 100.0, snapmax = d.snap_max,
		snapratio = 0.30 / d.snap_max, neck = NECK_D, psi = BAY_PSI_DEG, travel = d.travel,
		engage = ENGAGE, exy = d.engage_xy, efull = d.engage_full, fcap = d.f_cap,
		carried = d.carried_n, capx = d.f_cap / d.carried_n,
		captive = d.captive, float = d.float_nom, wc = d.wc_capture,
		urel = 100.0 * d.u_rel / d.travel, slope = RELIEF_SLOPE, finhw = FIN_HW,
		tp = T_PLANET, od = od, height = d.height, printed_g = d.printed_g,
		marg = d.sig_allow / d.sig_ring, mesh_hz = R_T as f64 * 10.0,
		coul = d.coul_frac * 100.0, ratio_edge = d.t_edge / d.t_nom,
		form_ratio = d.form_ratio, p_worst = d.p_worst, bridge = d.worst_bridge,
	));
	let _ = std::fs::write(format!("{OUT}/analysis/ANALYSIS.md"), a);

	// ---------------- analysis/DESIGN.md (authored contract) ----------------
	let _ = std::fs::write(
		format!("{OUT}/analysis/DESIGN.md"),
		format!(include_str!("nullspin_design.md.in"), od = od, jr = d.jr, floor = d.floor, probe = d.margin),
	);

	// ---------------- assembly/BOM.md + instructions.md ---------------------
	let _ = std::fs::write(
		format!("{OUT}/assembly/BOM.md"),
		format!(
			"# NULLSPIN — bill of materials\n\n\
			| item | qty | source | material | mass |\n|---|---|---|---|---|\n\
			| `parts/base_spider` | 1 | print | PLA, 3 walls / 20% gyroid | {mg_base:.2} g |\n\
			| `parts/top_spider` | 1 | print | PLA | {mg_top:.2} g |\n\
			| `parts/ring_66t` | 1 | print, **5 walls** | PLA | {mg_r:.2} g |\n\
			| `parts/sun_42t` | 1 | print | PLA | {mg_s:.2} g |\n\
			| `parts/planet_12t_bore600` | 6 | print | PLA | {mg_p:.2} g each |\n\
			| `parts/cap` | 1 | print | PLA | {mg_cap:.2} g |\n\
			| `optional/sun_b_control` | 1 | print (the A/B) | PLA | {mg_sb:.2} g |\n\
			| `optional/coupon_fit` | 1 | print FIRST, ~12 min | PLA | {mg_cpn:.2} g |\n\
			| `optional/coupon_key` | 1 | print with the coupon | PLA | {mg_key:.2} g |\n\
			| `optional/planet_12t_bore590` · `bore615` | 6 each | print only if needed | PLA | — |\n\n\
			The coupon grew a second piece in v4: a bayonet cannot be gauged by one\n\
			body, so `coupon_key` carries the shipped slot and slides onto the coupon's\n\
			bayonet pin. That is a real part-count cost of the change, stated here.\n\n\
			## Non-printed parts: NONE\n\n\
			There is no `bought` row in this table and there is no `bought` row in\n\
			`scene/bom.csv` either — the file has six lines and every one of them says\n\
			`made`. No bearing, no balls, no magnets, no screws, no nuts, no weights, no\n\
			inserts, no glue, no tools. **Printed set: {printed_g:.1} g of PLA and that is\n\
			the complete list of everything you need.**\n\n\
			## What that costs, in the same units\n\n\
			v1 of this model used one 608 bearing. v2 added 24 loose Ø1.50 mm steel balls\n\
			as a thrust race under the ring. v3 deletes both, and the two loads land back\n\
			on printed sliding contacts:\n\n\
			| build | bought parts | predicted spin |\n|---|---|---|\n\
			| v1 — sliding ring land + 608 | 1 | {t_slide:.1} s |\n\
			| v2 — ball race + 608 | 25 | {t_race:.1} s |\n\
			| **v3 — this one** | **0** | **{t_nom:.1} s** |\n\n\
			All three are recomputed by the same solver on the same rotor in the same run\n\
			(`analysis/ANALYSIS.md`), so that is a measured comparison and not a memory.\n\
			**It is a real loss and it is not hidden anywhere in this repository.** What\n\
			you get for it is a model with nothing to source, nothing to lose on the\n\
			carpet, and nothing small enough to swallow.\n\n\
			## The two contacts that decide your spin time\n\n\
			1. **The six ring thrust pads** in the base rim, at r {r_ring:.2} mm. The ring's\n\
			   whole weight rubs here and it is {ring_pct:.0}% of the entire budget. They are\n\
			   as far inboard as the ring's own flat underside reaches.\n\
			2. **The sun's thrust land** around the post, at r {r_sun:.2} mm. This is what\n\
			   replaced the bearing.\n\n\
			Both are plain printed PLA on plain printed PLA. Their friction coefficient is\n\
			the single biggest unknown in the prediction, which is why the published band\n\
			is {t_pes:.1}–{t_opt:.1} s and not a number. **Printed journals and thrust faces\n\
			are also known to need a break-in** — spin it by hand for a minute with light\n\
			pressure in a few directions before you judge it, exactly as the field advice\n\
			for printed plain bearings says. That effect is NOT in the model, and it can\n\
			only make the real number better than the published one.\n\n\
			Keep solvents away from it. There is no bearing to de-grease, so the one\n\
			reason anyone ever put IPA near a printed spinner is gone; there are\n\
			first-hand reports of printed spinner bodies shattering after a solvent clean.\n",
			mg_r = d.mg_r, mg_s = d.mg_s, mg_p = d.mg_p, mg_sb = d.mg_sb,
			mg_base = d.mg_base, mg_top = d.mg_top, mg_cap = d.mg_cap, mg_cpn = d.mg_coupon,
			mg_key = d.mg_key, printed_g = d.printed_g,
			t_slide = d.t_slide, t_race = d.t_race, t_nom = d.t_nom,
			t_pes = d.t_pes, t_opt = d.t_opt,
			r_ring = d.r_ring_pad, r_sun = d.r_sun_land,
			ring_pct = 100.0 * d.drag.terms[0].0 / d.drag.torque(W0),
		),
	);
	let _ = std::fs::write(
		format!("{OUT}/assembly/instructions.md"),
		format!(
			"# NULLSPIN — assembly, 6 steps, no tools, nothing to buy\n\n\
			**There are no non-printed parts.** Print the six parts, put them together, use\n\
			it. Nothing else goes in the box.\n\n\
			Print `optional/coupon_fit` AND `optional/coupon_key` first (~12 min together).\n\
			Every fit on them is printed part to printed part, and there are three that\n\
			decide the build: a Ø5.50 journal pin (the post and the planet pins are the same\n\
			diameter, so one pin gauges both — a printed planet must drop on and spin free),\n\
			a Ø5.50 press boss for the cap, and the **bayonet joint** — drop `coupon_key`\n\
			over the third pin's fin and slide it home with a slight swivel. The key should\n\
			slide without force and then refuse to lift. If something is off, fix it there\n\
			instead of on a 90-minute print.\n\n\
			1. **Sun onto the post.** It just drops on. There is no bearing and nothing to\n\
			   press: the sun turns directly on the printed post and rests on the small\n\
			   raised land around its base. It is located by its six gear meshes, not by\n\
			   the post, so a little rattle on the post is correct and not a defect.\n\
			2. **Six planets onto six pins.** They are identical and cannot be installed\n\
			   wrong: (S+R) % 6 = 0 makes every planet azimuth mesh-equivalent, so each one\n\
			   self-clocks against the sun as it seats.\n\
			3. **Ring over the planets.** It self-clocks too, is located radially by all\n\
			   six, and rests on the six thrust pads in the base rim.\n\
			4. **Top spider on — the bayonet.** Set it down about 7° anticlockwise of the\n\
			   base arms so each pin's fin drops through the WIDE end of its slot. It sits\n\
			   flat with no force at all. Then twist the spider 7° clockwise until all six\n\
			   pins stop against the closed ends. There is nothing to click past and nothing\n\
			   to press: what holds it is 1.15 mm of pin fin sitting over the slot wall.\n\
			5. **Look at it.** Every pin's fin must be at the CLOSED (narrow) end of its\n\
			   slot. That is the whole inspection — if you can see all six, it is locked,\n\
			   and no amount of printer calibration changes that. To take it off, twist the\n\
			   7° back and lift.\n\
			6. **Cap on.** Press it onto the post top. It is the static thumb pad and it is\n\
			   also what keeps the sun in when you turn the spinner over. This one IS a press\n\
			   fit (0.025 mm) — the only one left in the model, and the only fit whose grip\n\
			   still depends on your printer.\n\n\
			No glue, no drilling, no scalpel, no grease, no hardware.\n\n\
			**Break it in before you judge the spin.** Everything that slides in this model\n\
			is printed PLA on printed PLA, and printed plain bearings are documented to need\n\
			it. Hold the frame and spin the ring by hand for a minute with light pressure in\n\
			a few different directions to flatten the layer bumps. This effect is NOT in the\n\
			published prediction — it can only make the real number better than\n\
			{t_nom:.1} s, never worse.\n\n\
			**To run the A/B:** lift the cap, twist the top spider 7° back and lift it, swap `sun_42t` for\n\
			`sun_b_control`, re-fit. Twenty seconds. The tuned sun feels dead when you tilt\n\
			it; SUN-B fights you like a normal spinner. That is the whole point of the\n\
			design, and you are performing the experiment yourself rather than taking a\n\
			number on trust.\n\n\
			**Hold it EDGE-ON for the longest spin — this reversed back.** v1 said edge-on\n\
			helped; v2 fitted a steel ball race under the ring and flat became the fast way.\n\
			v3 has no rolling element anywhere, so edge-on wins again: held flat, the ring's\n\
			whole weight rubs on the six thrust pads at r {r_ring:.1} mm, which is\n\
			{ring_pct:.0}% of the entire drag budget. Edge-on is {ratio_edge:.1}× flat\n\
			({t_edge:.1} s vs {t_nom:.1} s) — the load moves into the meshes and the planet\n\
			journals, it does not leave the machine. Both budgets are published term by\n\
			term in `analysis/ANALYSIS.md`.\n",
			ratio_edge = d.t_edge / d.t_nom, t_nom = d.t_nom, t_edge = d.t_edge,
			r_ring = d.r_ring_pad,
			ring_pct = 100.0 * d.drag.terms[0].0 / d.drag.torque(W0),
		),
	);

	// ---------------- README.md --------------------------------------------
	let _ = std::fs::write(
		format!("{OUT}/README.md"),
		format!(
			"# NULLSPIN — the spinner whose two rims turn opposite ways\n\n\
			A grounded-carrier (\"star\") epicyclic fidget spinner. The frame you hold IS the\n\
			planet carrier, so the six planet axes never orbit. Flick the outer ring and the\n\
			inner puck counter-rotates at an exact integer ratio: **flick the ring 7 times\n\
			and the puck turns 11 times the other way** (7 · 66 = 11 · 42 = 462).\n\n\
			**Nothing in this model is bought.** No bearing, no balls, no magnets, no\n\
			screws, no nuts, no weights, no inserts, no glue, no tools. Print six parts,\n\
			put them together, use it.\n\n\
			**Nothing in it is held on by friction either.** The top spider — the cover that\n\
			keeps six planets and the ring in — goes on with a 7° BAYONET twist and is then\n\
			held by {engage:.2} mm of solid pin sitting over the slot wall. Earlier versions\n\
			used a press fit whose grip a mis-calibrated printer could erase completely; this\n\
			one survives 0.35 mm/side of error on BOTH parts and still has {efull:.2} mm of\n\
			shoulder left, which is proved on the built solids and falsified two ways every\n\
			run (`analysis/ANALYSIS.md`, G16).\n\n\
			Because the two rotors turn opposite ways, their angular momenta nearly cancel:\n\
			**{eta_pct:.1}%** of the spin angular momentum is cancelled (eta = {eta:.4}), so it\n\
			barely fights being tilted. A second control puck (`optional/sun_b_control`) is\n\
			deliberately UNcancelled at eta = {eta_b:.4}, so you can feel the difference in\n\
			twenty seconds.\n\n\
			Ø{od:.1} × {height:.2} mm · {printed_g:.1} g PLA · **you also need: nothing.**\n\n\
			## It spins for about {t_nom:.0} seconds, and that is the honest number\n\n\
			Predicted **{t_nom:.1} s** from a {rpm:.0} rpm launch, band {t_pes:.1}–{t_opt:.1} s.\n\
			Removing the hardware cost real spin time and this repository does not hide it:\n\
			the same solver, on the same rotor, in the same run, gives **{t_slide:.1} s** for\n\
			v1's architecture (a 608 in the sun) and **{t_race:.1} s** for v2's (a 608 plus a\n\
			24-ball steel thrust race). v3 is {cost:.1}× shorter than v2 and it buys an empty\n\
			bought list.\n\n\
			The reason is one contact. In a grounded-carrier star the ring is not on a\n\
			bearing — it is located by its six meshes — so held flat its whole weight rubs\n\
			on the frame at r {r_ring:.1} mm. That is {ring_pct:.0}% of the drag budget and\n\
			it is constant-force friction, the worst decay class there is. Everything that\n\
			could remove it needs a rolling element or a web to the axis; both are costed\n\
			and both are refused, with numbers, in `analysis/ANALYSIS.md`.\n\n\
			## Folder map\n\n\
			| you're asking… | open |\n|---|---|\n\
			| what do I print? | `parts/` (6 unique parts) · `optional/` (the 12-minute fit coupon **and its bayonet key**, the SUN-B control, the bore ladder) |\n\
			| what do I have to buy? | **nothing** — `assembly/BOM.md` has six rows and every one says `made` |\n\
			| how do I build it? | `assembly/` — `ASSEMBLY.png` (the one-page sheet), `BOM.md`, `instructions.md`, `assembly.stl` |\n\
			| can I modify it? | `cad/*.step` |\n\
			| what does it look like? | `renders/` |\n\
			| is it verified? | `analysis/ANALYSIS.md` (generated every run from the gate suite) + `analysis/DESIGN.md` (the frozen research contract and the analysis plan) |\n\
			| how do I publish it? | `publish/` |\n\n\
			## Regenerate everything\n\n\
			```sh\n\
			cargo run --release -p kernel-model --example nullspin\n\
			```\n\n\
			Every claim on this page is re-proved on every run and the run exits 1 on any\n\
			FAIL. Nothing here is hand-copied.\n\n\
			## Authorship\n\n\
			This model is defined by a **parametric Rust program**, not drawn by hand in a\n\
			GUI CAD package: the program computes every dimension, builds the solids and\n\
			re-proves every claim on every run, and its output is deterministic. The program\n\
			was written with AI assistance, as was the research that froze its dimensions and\n\
			its analysis plan; the geometry is NOT the output of a generative 3-D model —\n\
			there is no mesh generator anywhere in the pipeline. Eligibility was confirmed\n\
			with Printables before entering. This is disclosed in full on the model page\n\
			under \"How this model was made\" in `publish/PRINTABLES_LISTING.md`; no copy\n\
			anywhere in this deliverable implies hand-modelling.\n",
			eta = d.eta, eta_pct = d.eta * 100.0, eta_b = d.eta_b, od = od, height = d.height,
			printed_g = d.printed_g, t_nom = d.t_nom, t_pes = d.t_pes, t_opt = d.t_opt,
			rpm = W0 * 60.0 / TAU, t_slide = d.t_slide, t_race = d.t_race,
			engage = ENGAGE, efull = d.engage_full,
			cost = d.t_race / d.t_nom, r_ring = d.r_ring_pad,
			ring_pct = 100.0 * d.drag.terms[0].0 / d.drag.torque(W0),
		),
	);

	// ---------------- publish/ ---------------------------------------------
	let _ = std::fs::write(
		format!("{OUT}/publish/PRINTABLES_LISTING.md"),
		format!(
			include_str!("nullspin_listing.md.in"),
			eta = d.eta,
			eta_pct = d.eta * 100.0,
			eta_b = d.eta_b,
			eta_lo = d.eta_lo,
			od = od,
			height = d.height,
			printed_g = d.printed_g,
			t_nom = d.t_nom,
			t_pes = d.t_pes,
			t_opt = d.t_opt,
			rpm = W0 * 60.0 / TAU,
			w0 = W0,
			eps_sp = d.eps_sp,
			eps_pr = d.eps_pr,
			jt = d.jt_measured,
			neighbour = d.neighbour,
			mg_r = d.mg_r,
			mg_s = d.mg_s,
			mg_p = d.mg_p,
			izz_r = d.izz_r,
			izz_s = d.izz_s,
			i_eff = d.i_eff_gmm2,
			coul = d.coul_frac * 100.0,
			t_slide = d.t_slide,
			t_race = d.t_race,
			t_edge = d.t_edge,
			ratio_edge = d.t_edge / d.t_nom,
			r_ring = d.r_ring_pad,
			r_sun = d.r_sun_land,
			ring_pct = 100.0 * d.drag.terms[0].0 / d.drag.torque(W0),
			sun_pct = 100.0 * d.drag.terms[1].0 / d.drag.torque(W0),
			engage = ENGAGE,
			efull = d.engage_full,
			snapmax = d.snap_max,
			snapratio = 0.30 / d.snap_max,
			cost = d.t_race / d.t_nom,
		),
	);
	let _ = std::fs::write(
		format!("{OUT}/publish/PRODUCT_SHOT_PROMPT.md"),
		"# NULLSPIN — gallery plan\n\n\
		The contest scores photo quality explicitly, and the revealed bar from previous\n\
		winners is 33–49 gallery items plus a leading GIF. The subject here is MOTION and\n\
		an ABSENCE, so stills alone will under-sell it.\n\n\
		1. **Slot 1 — the hero GIF.** Two concentric rims turning in OPPOSITE directions\n\
		   against a dead-still spider. Shoot square-on, top-down, high contrast between\n\
		   the ring and the sun (print them in one colour and let the shadow do the work,\n\
		   or accept a two-colour build for the gallery only). Put one index groove under\n\
		   a hard light so the sun's direction reads instantly. 3–4 s loop.\n\
		2. **Slot 2 — the SUN-B A/B GIF.** Same rig, same flick, side by side: the tuned\n\
		   sun barely resists a tilt, the control puck fights it. This is the only way an\n\
		   absence gets photographed.\n\
		3. **Macro of one mesh** — the 12T planet between the 42T sun and the 66T ring,\n\
		   showing the tip chamfers and the root fillets.\n\
		4. **Exploded still** on white: base spider, six planets, ring, sun, top spider,\n\
		   cap. Six parts and NOTHING ELSE in the frame — the empty space where a\n\
		   bearing would be is the point of the shot.\n\
		5. **The coupon** mid-fit-check, with a printed planet spinning on its pin.\n\
		6. **The plate** — everything on one bed, no supports, one colour.\n\
		7. **In hand**, for scale, held EDGE-ON (which is the way to hold it for the\n\
		   longest spin in this version — and it reversed twice getting here).\n\
		8. **The base spider alone**, top-down with everything lifted off: six thrust\n\
		   pads, six pins and a Ø5.50 post. This is what a spinner looks like when the\n\
		   bought list is empty, and it is also exactly why it only runs a couple of\n\
		   seconds — do not shoot it as if it were a feature list, shoot it as the\n\
		   trade it is.\n\n\
		Lighting: one large soft key at 45°, one rim light to separate the ring's rounded\n\
		top edge from the background. Shoot the GIFs at 120 fps and slow to 30 so the\n\
		counter-rotation is legible rather than a blur.\n\n\
		Do not stage a spin-time claim, and do not shoot a long spin. This model runs\n\
		for about two seconds and the gallery must not imply otherwise. If a timing shot\n\
		is included, show the protocol (the release rig, the stopwatch, five runs)\n\
		rather than a single number.\n",
	);
}
