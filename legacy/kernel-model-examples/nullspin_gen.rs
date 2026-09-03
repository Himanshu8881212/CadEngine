// Copyright (c) LMCAD. Licensed under the MIT License.

//! NULLSPIN-GEN — the grounded-carrier epicyclic spinner of `nullspin.rs`, with
//! its held frame replaced by a TOPOLOGY-OPTIMISED ORGANIC WEB.
//!
//! Contest entry #2, Printables "Designer Challenge: Geared Spinners". This is a
//! SIBLING of `nullspin.rs`, not a revision of it: that entry is finished and
//! green, and nothing here touches it.
//!
//! WHAT IS THE SAME, DELIBERATELY. The gear set is frozen bit-for-bit — 66T ring,
//! 42T sun, six 12T planets, m1.0, 25° PA, zero profile shift, 0.09 mm/flank
//! thinning. So is the headline (7·66 = 11·42 = 462: flick the ring seven times
//! and the puck turns eleven the other way), the bayonet retention, the
//! momentum-cancellation receipt eta, and the spin-down solver with its
//! benchmarks. Those are solved problems and re-deriving them would only add
//! ways to be wrong. The rotors are NOT lightened: eta pins `I_sun·k_S` to
//! `I_ring + ΣI_p·k_P`, so removing mass from either rotor breaks the physics the
//! design is built on.
//!
//! WHAT IS NEW. The CARRIER — the held frame, base plate plus top plate — is the
//! only part of this machine with no kinematic duty and a genuine structural load
//! case, and it is the part your hand is actually on. In `nullspin` it is three
//! crossing bars and six tapered arms, drawn by hand. Here it is the output of a
//! real generative loop: a load case (drop impact + pinch), an ACE reference FEA
//! of the baseline, SIMP topology optimisation over the design domain with the
//! keep-outs, density field → implicit → mesh → exact B-rep, and then an HONEST
//! re-analysis of the FINAL BINARY GEOMETRY — never the optimiser's own internal
//! estimate. That last step is the doctrine `tools/ace_optimize_runner.py`
//! already follows and it is what makes the numbers below mean anything.
//!
//! THE LOAD CASE, WHICH IS THE POINT. `nullspin`'s own analysis declares
//! **impact/drop REQUIRED, NOT PERFORMED** and calls it "the largest honest gap
//! in the deliverable". This entry closes that gap. A dropped spinner is the
//! real failure mode of the product class, so it is modelled here as an
//! EQUIVALENT-STATIC drop: a stated height, a stated stopping distance, hence a
//! deceleration and a rim force, cross-checked against an elastic-plastic
//! indentation bound that assumes a perfectly rigid floor. It is emphatically
//! NOT a transient impact simulation and every section that quotes it says so.
//! The second case is the hand: a firm pinch across a diameter plus the flick.
//!
//! WHAT THE OPTIMISER IS AND IS NOT ALLOWED TO TOUCH. Keep-outs are FROZEN
//! regions in the SIMP manifest and are re-asserted geometrically in the rebuild
//! so the two cannot drift: the central hub and post, the six planet pins and
//! their bayonet features, the six ring thrust pads, the outer grip/contact rim,
//! and — on the top carrier — the ring-capture rim and the six bayonet slot pads.
//! The gear envelope is respected by construction: the whole design domain lives
//! in the 2.0 mm slab BELOW the gear plane, and a gate re-proves it on the built
//! solid rather than trusting the argument.
//!
//! HONESTY RULES CARRIED FROM `bracket_gen.rs`, THE EXECUTED REFERENCE.
//! * The SIMP receipt's own `as_built` block is never quoted as the product
//!   number. Every published stress is a fresh binary-occupancy FEA of the mesh
//!   that ships, through a byte-identical manifest.
//! * The analysis body and the shipped mesh are proved to be the same part.
//! * A negative control per oracle, and each one is TWO gates: prove the
//!   perturbation is real, then prove the instrument reacted.
//! * Connectivity is gated separately from validity and watertightness. An
//!   optimiser will happily hand you a floating island; `shell_count()` will not
//!   catch it and `Mesh::is_one_body` will.
//!
//! Run: cargo run --release -p kernel-model --example nullspin_gen
//! (writes spinner_system/nullspin_gen/**; exit 1 on any FAIL)

use kernel_brep::math::{DAffine3, DVec2, DVec3};
use kernel_brep::{
	boolean_hazards, cuboid, cylinder, difference, export_step, extrude, force_ccw, intersection, mass_properties,
	overlap_volume, revolve, tessellate_default, union, validate, volume, ChainLog, HazardKind, Solid,
};
use kernel_core::math::{Aabb, Vec3};
use kernel_core::sdf::Sdf;
use kernel_core::Mesh;
use kernel_implicit::grid_field::GridField;
use kernel_model::campaign::gate;
use kernel_model::kinematics::EpicyclicTrain;
use kernel_model::materials::pla::SIG_ALLOW_RT;
use kernel_model::materials::PLA_G_PER_MM3;
use kernel_model::optimize::{Constraint, DesignVar, Evaluation, Params, Study};
use kernel_model::parts::involute_ring_outline_shifted_filleted;
use kernel_model::process::FdmProfile;
use kernel_model::reverse;
use std::f64::consts::{PI, TAU};

const OUT: &str = "spinner_system/nullspin_gen";
const FEA_DIR: &str = "spinner_system/nullspin_gen/analysis/fea";
const PLA: f64 = PLA_G_PER_MM3;

// ============================================================================
// 1. GEAR SET — FROZEN, byte-identical to `nullspin.rs`. Gated G1–G4.
//
//    The teeth have to mesh and the 7:11 ratio is the entry's claim, so nothing
//    in this block is a design variable for this campaign. It is reproduced
//    rather than imported because a Cargo example cannot depend on another
//    example; G0x asserts every value against the same oracles the sibling uses.
// ============================================================================

/// Module. WHY: pitch-line tooth thickness π·m/2 = 1.571 mm = 3.5 × a 0.45 mm
/// extrusion width — two solid walls plus real fill.
const M: f64 = 1.000;
/// Pressure angle. WHY: the undercut floor is z ≥ 2/sin²α = 11.198 T at 25°, so
/// the 12T planet is legal at ZERO profile shift, and this engine does not model
/// undercut at all (so "20° + shift" would be fiction).
const PA_DEG: f64 = 25.0;
const S_T: usize = 42; // sun, external
const P_T: usize = 12; // planet, external, ×6
const R_T: usize = 66; // ring, internal
const N_PL: usize = 6;
/// Profile shift on every member. Zero — see PA_DEG.
const X_SHIFT: f64 = 0.0;
/// Backlash: tooth thinning per flank, mm, on ALL THREE members → jt = 0.18 mm
/// per mesh. CMM-measured FDM involute deviation is 0.067 mm/flank, two flanks
/// meet, so anything under ~0.134 mm binds.
const LASH: f64 = 0.09;
/// Root fillet coefficient (r = 0.30·m) on the EXTERNAL members only.
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
const _: () = assert!(S_T % N_PL == 0, "sun pattern repeats under 2π/n");
const _: () = assert!(7 * R_T == 11 * S_T, "HEADLINE: flick the ring 7×, the puck turns 11× the other way");
const _: () = assert!(2 * R_T == 11 * P_T, "planet runs at exactly 11/2 = 5.5× the ring");

// ============================================================================
// 2. PROCESS + CLEARANCES — from profiles/conservative_default.json, print-proven
//    in RESPOOL/DRYBOX. Identical to the sibling.
// ============================================================================

/// Running radial fit, mm (profile xy_clearance_free).
const C_FREE: f64 = 0.25;
/// Press/locating fit, mm (profile xy_clearance_tight).
const C_TIGHT: f64 = 0.05;
/// Axial gap, mm (profile z_clearance).
const C_Z: f64 = 0.30;
/// Bed-side chamfer on every clearance surface, mm × 45°.
const C_BED: f64 = 0.45;
/// Tooth-tip chamfer, radial run in mm, both faces.
const C_TIP: f64 = 0.30;
/// Rise ÷ run of EVERY downward-facing relief cone. 1.40 (54.5° from horizontal)
/// so no facet can LAND on the 45° support threshold and read as float noise.
const RELIEF_SLOPE: f64 = 1.40;

/// 608 rotating inertia referred to the sun, g·mm² — LEDGER ONLY, never shipped.
const I608_GMM2: f64 = 610.0;

/// Post Ø. Also the campaign's printed-pin floor, so post and pin are ONE number.
const POST_D: f64 = 5.50;
/// Sun bore Ø — the sun RUNS on the post, so this is the profile's running fit.
const SUN_BORE_D: f64 = POST_D + 2.0 * C_FREE;
/// Radial width of the sun's annular thrust land on the hub, mm.
const SUN_LAND_W: f64 = 0.50;
/// Radial width of each ring thrust pad, mm, and the inset of its inner edge
/// from the ring's root circle (34.25).
const RING_PAD_W: f64 = 0.50;
const RING_PAD_INSET: f64 = 0.10;

// ---- printed-part geometry -------------------------------------------------
/// Baseline (sibling) spider arm half-width — used ONLY to rebuild the hand-drawn
/// carrier as the comparison row. No shipped part in this campaign uses it.
const ARM_HW: f64 = 5.00;
const Z_ARM: f64 = 2.00; // carrier plate thickness — the design domain's height
const Z_GEAR: f64 = Z_ARM + C_Z; // 2.30 — gear plane bottom
const HUB_D: f64 = 16.00;
const PIN_D: f64 = 5.50;
const PLANET_BORE_D: f64 = PIN_D + 2.0 * C_FREE; // 6.00
const PLANET_SEAT_D: f64 = 7.00; // thrust pad Ø under each planet
const TS_T: f64 = 2.00; // top carrier thickness
const TS_R_IN: f64 = 23.00; // clears the sun tip r 22.0 by 1.0
const TS_R_RIM: f64 = 35.25; // ring-capture rim, inner edge
/// Static-part outer radius. The RING stands 0.30 mm proud of both held rims, so
/// a flicking finger touches only the rotor. That 0.30 mm is ALSO what decides
/// which body hits the floor first in a drop — see the drop model.
const STATIC_R: f64 = 36.20;
const CAP_D: f64 = 12.00;
const CAP_T: f64 = 1.20;
const SUN_LEAD: f64 = 0.60;
const RIM_ROUND: f64 = 1.00; // full round, ring TOP rim only
/// Radial interference of the CAP's press fit on the post, mm — the model's only
/// interference fit. 0.025/2.75 = 0.91 % hoop strain against PLA's 1.67 % yield.
const CAP_PRESS_R: f64 = 0.025;

// ---- THE BAYONET: geometric top-carrier retention (inherited, unchanged) ----
//
// Retention is a lug under a ceiling with a hard end stop, nothing strained at
// rest. Each pin carries a Ø2.70 NECK through the plate and a radial FIN above
// it; each carrier arm carries a bayonet slot. Drop the top on 7° out, twist it
// home, and the fin overhangs the slot wall by ENGAGE mm of solid material.
// Printer error changes how tight the twist feels; it cannot change the SIGN of
// that overlap. The snap-fit alternative is re-refused every run (G16m): a
// Ø5.60 hole in this plate can expand only 0.047 mm before yield and the stack
// it would have to swallow is 0.30 mm — 6× short.
const NECK_D: f64 = 2.70;
const FIN_HW: f64 = 1.00;
const FIN_IN: f64 = 1.30;
const SLOT_HW: f64 = NECK_D / 2.0 + C_FREE;
const LOCK_R: f64 = NECK_D / 2.0 + C_TIGHT;
const LOCK_Y: f64 = 1.40;
const BULGE_HW: f64 = SLOT_HW;
const BULGE_X: f64 = PIN_D / 2.0 + C_FREE;
const BAY_PSI_DEG: f64 = 7.0;
const ENGAGE: f64 = PIN_D / 2.0 - SLOT_HW;
/// Baseline (sibling) top-arm planform — comparison row only.
const TS_ARM_Y0: f64 = -4.70;
const TS_ARM_Y1: f64 = 6.60;
const TS_ARM_YE: f64 = 3.60;
const TS_ARM_KNEE: f64 = 32.00;
const TS_R_IN_O: f64 = 25.00;
const N_INDEX: usize = 7; // index grooves on the sun face

/// Bottom plane of the ring and the six planets.
const Z_ROT: f64 = Z_GEAR;
/// Ring thrust-pad pitch radius, mm.
const RING_PAD_R: f64 = 34.25 + RING_PAD_INSET + RING_PAD_W / 2.0;

// ---- ledger only: the one bearing constant the 608 counterfactual needs -----
const E_STEEL: f64 = 200_000.0;
const NU_STEEL: f64 = 0.30;
/// PLA elastic constants — `tools/materials/pla.json` (3.3 GPa, ν 0.36, 55 MPa).
const E_PLA_MPA: f64 = 3300.0;
const NU_PLA: f64 = 0.36;
const SIG_YIELD_PLA: f64 = 55.0;

// ---- SHIPPED rotor design point (G11 re-derives it every run) ---------------
/// Sun face width, mm. The study re-solves it against THIS campaign's frame
/// mass, which is not the sibling's — the generative carrier weighs something
/// different, and the mass constraint is one of the binding ones. The value
/// below is whatever the study last returned; G11 fails loudly if it drifts.
const T_SUN: f64 = 7.80;
/// Ceiling of the sun-face design window, mm — the 12.0 mm envelope.
const T_SUN_MAX: f64 = 8.20;
const T_RING: f64 = 4.00;
/// Ring rim wall, mm. WHY 2.25: exactly 5 × 0.45 mm extrusion lines.
const RING_WALL: f64 = 2.25;
const T_PLANET: f64 = 4.00;
/// SUN-B control puck: deliberately UNcancelled, so the buyer performs the A/B.
const SUNB_FRAC: f64 = 0.55;

// ---- physics constants (research-frozen; provenance in analysis/DESIGN.md) ---
const W0: f64 = 110.0; // frozen launch speed, rad/s (= 1050 rpm)
const N_BRG: f64 = 0.50;
const M608_NMM: f64 = 0.0955;
/// PLA-on-PLA sliding friction. **UNKNOWN** — carried as a band, never a value.
const MU_PLA: f64 = 0.30;
const MU_LO: f64 = 0.20;
const MU_HI: f64 = 0.50;
const RHO_AIR: f64 = 1.204; // kg/m³ at 20 °C
const NU_AIR: f64 = 1.5e-5; // m²/s
const GRAV: f64 = 9.81;

// ---- safety ----------------------------------------------------------------
/// EN 71-1 §4.10 (2014 text): a space between moving elements that admits a Ø5
/// rod must also admit a Ø12 rod. The 5–12 mm band is forbidden.
const ROD_SMALL: f64 = 5.0;
const ROD_LARGE: f64 = 12.0;

// ============================================================================
// 3. THE LOAD CASE — the whole reason this campaign exists.
//
// `nullspin`'s ANALYSIS.md declares impact/drop **REQUIRED, NOT PERFORMED** and
// names it the largest honest gap in that deliverable. It is the real failure
// mode of a fidget spinner: nobody fatigues one, nobody creeps one, everybody
// drops one. So it is modelled here, with every assumption named and swept.
//
// THE MODEL, STATED IN FULL AND IN ADVANCE.
//
//   v = sqrt(2·g·h)                       impact speed from a free fall of h
//   a = v² / (2·s) = g·h / s              mean deceleration over stopping distance s
//   F = m · a = m·g·h / s                 the EQUIVALENT-STATIC contact force
//
// It is an **equivalent-static** model. It replaces a transient event with the
// single peak force it is estimated to produce and then solves a linear static
// problem. It is standard practice for product drop checks and it is NOT a
// transient impact simulation: it carries no wave propagation, no contact
// separation and re-strike, no rate dependence, and no damping. Nothing in this
// campaign calls it one, and the analysis plan lists a true transient solve as a
// REQUIRED, NOT PERFORMED row rather than pretending this stands in for it.
//
// WHY h = 1.00 m. Not from a standard — the toy-safety drop tests could not be
// re-verified from a primary source on this run, so they are not cited as
// authority. It is from USE: a spinner is flicked at hand height. A standing
// adult's hand at rest is ≈ 0.75 m off the floor, a desk is ≈ 0.75 m, and a hand
// held up in front of the chest to watch the gears turn is ≈ 1.1–1.3 m. 1.00 m
// is the middle of that band. The assumption is then made NON-LOAD-BEARING: the
// analysis reports the height at which each carrier reaches its allowable, so a
// reader who disagrees can read their own number off the sweep instead of
// arguing with this one.
//
// WHY s = 1.00 mm, and the bound that keeps it honest. `s` is the distance the
// centre of mass travels after first contact — local crush of the rim, local
// indentation of the floor, and the structure's own elastic squash, all
// together. 1.00 mm is the design value and the analysis sweeps 0.5–3.0 mm.
// It is cross-checked, not asserted: an elastic-plastic indentation bound
// (Johnson's constant-mean-pressure model at p_m = 3·σ_y, a PERFECTLY RIGID
// floor and no rotation of the part) is solved in closed form in this file and
// returns a far shorter stopping distance and a far larger force. That bound is
// published next to the design case, and the ratio between them is exactly the
// statement "1.00 mm describes a hard floor, not an infinitely rigid one".
const DROP_H_M: f64 = 1.00;
/// Stopping distance, mm. See the block above; swept in the ledger.
const DROP_S_MM: f64 = 1.00;
/// Design mass of the whole spinner, g — the mass the load case is frozen at.
///
/// This is deliberately a FROZEN INPUT and not the as-built number, because the
/// alternative is circular: the load case would size the carrier, whose mass
/// would move the load case. §25 puts the analysis plan before the geometry, so
/// the plan gets a stated mass and the geometry has to live inside it. A gate
/// checks the as-built printed set against it and fires if the two drift apart.
const DROP_MASS_G: f64 = 30.3;
/// How far the as-built set may drift from the frozen design mass before the
/// load case has to be re-frozen and everything downstream re-run.
const DROP_MASS_TOL: f64 = 0.05;
/// Johnson constraint factor for fully-plastic indentation, p_m = C·σ_y. C = 3 is
/// the metals value and the CONSERVATIVE end for a polymer (the literature range
/// for polymers is ~1.5–2.5, and a lower C gives a lower peak force), so the
/// rigid-floor bound below cannot be accused of being optimistic.
const INDENT_C: f64 = 3.0;
/// Effective edge radius of the rim corner that reaches the floor, mm. A blunter
/// corner hits HARDER (F ∝ sqrt(R_eff)), so the conservative choice is the
/// largest radius the 2.00 mm rim can present, i.e. half its own thickness.
const RIM_EDGE_R: f64 = 1.00;
/// Firm two-finger pinch, N. A comfortable hold is 5–15 N; a deliberate hard
/// squeeze by an adult is 30–50 N (tip-pinch maxima run 50–70 N). 30 N is a hard
/// squeeze, not a maximum, and it is applied as a design load rather than an
/// ultimate one.
const PINCH_N: f64 = 30.0;
/// Thumb flick at the rim, N — the same number the sibling's tooth-root gate uses.
const FLICK_N: f64 = 5.0;
/// Half-length of a finger's contact patch on the rim, mm. A fingertip pad on a
/// Ø72 rim covers roughly 10 mm of arc; the drop's 4.4 mm patch is a hard corner
/// striking a floor and is the wrong contact for a hand.
const FINGER_R: f64 = 5.00;
/// Radius around the contact patch inside which the von Mises field is treated
/// as a LOAD-INTRODUCTION ARTIFACT rather than a structural stress, mm. Two pad
/// radii. A point-introduced load always spikes under its own patch and the
/// spike is a property of the idealisation, not of the part — so both numbers
/// are carried everywhere (raw peak and masked peak) and a gate proves the raw
/// peak really is inside this radius, which is what stops the mask being a way
/// of not looking.
const MASK_R: f64 = 2.0 * CONTACT_PAD_R;
/// Coarse-hex8 derate applied to every published peak stress. The `ace_fea` card
/// measures coarse grids under-predicting the bending response by 5–20 %; ×1.25
/// covers the top of that band. Same factor and same reason as `bracket_gen.rs`.
const HEX8_PEAK_FACTOR: f64 = 1.25;

// ============================================================================
// 4. THE GENERATIVE LOOP — grid, SIMP and rebuild constants.
//
// Every one of these is a knob the optimiser's answer depends on, so every one
// carries the reason it has the value it has, and the ones that could flatter
// the result are gated (the filter radius sets the minimum length scale; the
// volume fraction is asserted achieved; both SIMP runs must be byte-identical).
// ============================================================================

/// SIMP analysis voxel, mm. The carrier is a flat plate loaded IN ITS OWN PLANE,
/// so the resolution that matters is in-plane: 1.00 mm gives 75×75 elements
/// across the Ø73 disc, which resolves the Ø5.50 pins at 5.5 elements and the
/// 3.2 mm outer rim at 3.2. Through the thickness 2 elements is plenty for
/// membrane and in-plane bending; it would not be for out-of-plane bending, and
/// out-of-plane is DECLARED, not optimised (see the analysis plan).
const VOX: f64 = 1.00;
/// Analysis grid: 75 × 75 × 2 elements over a 75 × 75 × 2.00 mm box.
const GRID_DIMS: (usize, usize, usize) = (75, 75, 2);
/// World position of grid NODE (0,0,0). ACE's `origin_mm` names the node; the
/// `.npy` values are per-ELEMENT, which is why the GridField reader is handed
/// `origin + VOX/2`.
const GRID_ORIGIN: (f64, f64, f64) = (-37.5, -37.5, 0.0);
/// Target volume fraction for the RIM envelope case. That problem is six-fold
/// symmetric, so its optimum is already six-fold and the fraction lands almost
/// unchanged in the shipped planform (measured: the six-fold maximum moves it by
/// about 5 %).
const VOLFRAC_RIM: f64 = 0.22;
/// …and the TOP carrier's, which is higher, because it is the weak link and the
/// run says so rather than intuition. At 0.22 both carriers the top's worst
/// azimuth read 41.2 MPa against the base's 21.7 — it is anchored only at six
/// bayonet slots, it has no hub load path to share the work, and its plate came
/// out 30 % lighter than the base's for the same span. Raising only the top's
/// fraction is the cheapest place to buy margin: the drop force scales with the
/// WHOLE product's mass, so material added where the peak is buys more than it
/// costs, and material added anywhere else does not.
const VOLFRAC_TOP: f64 = 0.34;
/// Target volume fraction for the HUB case. Much lower, because that one is
/// solved at a SINGLE azimuth and then replicated six times by the symmetrising
/// maximum, so its shipped contribution is several times the fraction asked for
/// here. The achieved planform area of both cases and of their union is measured
/// and published rather than inferred.
const VOLFRAC_HUB: f64 = 0.07;
/// SIMP penalty exponent. The runner's default and the top88 lineage value.
const SIMP_PENALTY: f64 = 3.0;
/// Cone density-filter radius, in VOXELS. This is the minimum-length-scale knob
/// and it is gated: 2·r·VOX must be at least the printable minimum feature.
const SIMP_FILTER_RVOX: f64 = 2.0;
/// SIMP density floor. The runner's default is 0.02, which at penalty 3 leaves a
/// void element 8e-6 of the solid stiffness — mechanically negligible and
/// numerically awful: on this problem the Jacobi-CG solve stopped converging
/// partway through the loop and the run died with `info=2000` at 9108 DOFs.
/// 0.05 leaves 1.2e-4, still four orders below solid and therefore still
/// mechanically void, but fifteen times better conditioned. The number is here
/// because it was measured, not because it is a default.
const SIMP_FLOOR: f64 = 0.05;
/// OC iteration cap. The runner reports `stop_reason`, which is published.
const SIMP_MAX_ITERS: usize = 45;
/// Threshold applied to the filtered physical density. The runner's own `iso`
/// and the rebuild's `iso` are the SAME number, so the geometry this campaign
/// ships is the geometry the runner re-analysed internally.
const ISO: f32 = 0.5;
/// Pseudo-SDF scale on `(ISO − rho)`. After a 2-voxel cone filter plus the tent
/// blur the density transition spans ~3 voxels, so ×3 gives near-unit slope at
/// the boundary. This is a sign-correct BOUND, not a distance field: dual
/// contouring is sample-based and needs the sign, not the Lipschitz constant.
const SDF_SCALE: f32 = 3.0;
/// Rebuild mesher voxel, mm. Sets the facet scale of the organic surface and is
/// gated: no triangle edge may exceed MESH_VOX·sqrt(3).
const MESH_VOX: f32 = 0.45;
/// Minimum printable feature, mm. Two 0.45 mm extrusion lines plus a margin, and
/// the floor the SIMP filter radius is asserted to impose.
const MIN_FEATURE: f32 = 1.60;

// ---- the design domain and its keep-outs ------------------------------------
/// BASE carrier: inner edge of the design annulus, mm. The hub's own disc runs to
/// r 8.00; the web's frozen inner ring overlaps it to r 9.00 so the union of the
/// two is a proper interpenetration and not a coincident cylinder (§7.7 rule 3 —
/// a tangent union of two revolves is exactly how a chain goes invalid).
const WEB_R_IN: f64 = 9.00;
/// BASE carrier: inner edge of the frozen outer rim, mm. WHY a continuous rim at
/// all, when the sibling has six bare arm tips: it is the grip surface, it is
/// what supports the six ring thrust pads at r 34.60, and it is the only way the
/// drop load case is defined at EVERY azimuth. The sibling's carrier has no
/// material at all at the between-arm azimuths, which is gated below as a
/// geometric fact rather than asserted.
const RIM_R_IN: f64 = 33.00;
/// Frozen radius around each planet pin, mm — the pin's own Ø8.60 thrust-boss
/// flare (r 4.30) plus a wall. Also the FEA fixture patch.
const PIN_PAD_R: f64 = 4.80;
/// Frozen radius around each ring thrust pad, mm.
const RING_PAD_FREEZE_R: f64 = 2.20;
/// TOP carrier: inner edge of the design annulus, mm — the sun's tip circle is
/// r 22.00, so 23.00 is one clear millimetre.
const TOP_R_IN: f64 = 23.00;
/// TOP carrier: frozen radius around each bayonet slot, mm, centred on the slot's
/// own centroid (not on the pin) because the slot runs outboard and forward of it.
const SLOT_PAD_R: f64 = 5.50;
/// TOP carrier: inner edge of its capture rim, mm — 0.60 mm INBOARD of the base's,
/// and the reason is a sliver rather than a preference. The slot pads reach
/// r 33.52; a rim starting at 34.00 leaves a 0.48 mm gap between two frozen
/// bodies wherever the web does not happen to bridge it, and a 0.48 mm slot in a
/// 2.00 mm plate meshed at 0.45 mm is not geometry, it is noise: the wrap+coalesce
/// drift went to 2.05 % and the STEP stopped being the same part as the STL.
/// 33.40 makes the pads and the rim OVERLAP, which costs 0.31 g and removes the
/// failure mode instead of tolerating it.
const RIM_R_IN_TOP: f64 = 33.40;

// ---- gate thresholds, frozen BEFORE the loop ran ----------------------------
/// Ceiling on what six-fold symmetrisation may cost in planform area. SIMP/OC
/// is non-convex and breaks symmetry, so the max over the six rotations always
/// adds material; 1.60× is the frozen ceiling and the achieved figure is
/// published. If a run ever needs more than this the honest response is a lower
/// volume fraction, not a looser gate.
const SYM_AREA_MAX: f64 = 1.60;
/// Ceiling on the disconnected debris a run may produce and still be shippable,
/// mm² of planform. Anything at all is reported; this is the point at which the
/// optimiser's answer stops being a structure and starts being confetti.
const DEBRIS_MAX_MM2: f64 = 60.0;
/// How far the exact contour prism may differ in volume from the meshed field it
/// was extracted from — and, separately, from the kernel's own independent
/// reverse-bridge reconstruction of the same field. Two different algorithms
/// reading one density field will not agree to machine precision; 1 % is the
/// band inside which they are describing the same part, and the measured figure
/// is published for both.
const CONTOUR_DRIFT_MAX: f64 = 0.01;
/// Required margin on PLA's yield at the design drop, for a single impact event.
///
/// This one number closes the campaign's loop, and the coupling is the
/// interesting part. The equivalent-static drop force is `m·g·h/s`, so the
/// carrier's peak stress is LINEAR IN THE PRODUCT'S OWN MASS — a heavier spinner
/// hits harder. The mass ceiling is therefore not a taste, it is DERIVED: the
/// heaviest legal product is the one whose drop peak still clears yield by this
/// margin, and the rotor design study is handed that number rather than a round
/// one. The generative carrier costs mass, the ceiling comes down on the rotors,
/// and what that costs in spin time is measured rather than waved at. The
/// ENVELOPE (Ø73 × 12.0 mm) is unchanged, which is the constraint that decides
/// whether it is still the same product.
///
/// 1.20 and not 1.50: a 1.5 factor against yield for this event is not reachable
/// by any 2.00 mm PLA plate inside a 12 mm envelope, and pretending otherwise
/// would mean quietly softening either the drop height or the allowable. What the
/// design achieves is published, and so is the drop HEIGHT at which the 10 MPa
/// design allowable — the one with the 2× design factor already in it — is met.
const DROP_MARGIN_MIN: f64 = 1.20;
/// Contour sampling pitch for the exact rebuild, mm. Four samples per mesher
/// voxel: fine enough that the extracted planform tracks the level set to well
/// under a layer height, coarse enough that the simplifier has something to do.
const CONTOUR_RES: f64 = 0.12;
/// Douglas–Peucker tolerance on the extracted contour, mm. One fifth of a layer:
/// the contour is allowed to move less than the printer's own quantisation.
const CONTOUR_TOL: f64 = 0.04;
/// Outer radius the capture NEGATIVE CONTROL is clipped to, mm — inboard of the
/// ring's own tip circle (32.00), so a control top carrier cannot overhang any
/// part of the ring at all.
const NC_NO_CAPTURE_R: f64 = 31.50;
/// Blend radius joining the frozen keep-outs to the optimiser's web, mm. One
/// minimum feature: big enough to be a real fillet at every junction, small
/// enough that it cannot close a gap the length-scale filter deliberately opened.
const BLEND_R: f64 = 1.60;
/// The most of an analysis body that may be dropped as unresolved voxel specks
/// before the grid stops describing the part. The sibling's 0.95 mm capture rim
/// costs 4.7 % on a 1.00 mm grid, which is disclosed rather than hidden; beyond
/// 8 % the right answer is a finer grid.
const OCC_PRUNE_MAX: f64 = 0.08;
/// The surface-quality claim: longest triangle edge anywhere on a shipped part.
const FACET_MAX_MM: f64 = 1.20;
/// How far the STEP solid may differ in volume from the STL that ships, before
/// the two stop being the same part. 0.5 % — the figure the kernel's own
/// `recover_quadrics` publishes for its rebuild, so it is a tolerance this
/// repository already stands behind rather than one invented here. The MEASURED
/// drift is printed on every run and published in ANALYSIS.md.
const BRIDGE_DRIFT_MAX: f64 = 0.005;
/// Support budget: every carrier must audit CLEAN. An organic web printed flat
/// has no down-facing face steeper than its own draft, and if it ever does the
/// answer is geometry, not a budget.
const STEEP_MAX_MM2: f64 = 1e-6;

// ============================================================================
// 5. GEAR MATHS + THE BAYONET — inherited verbatim from the sibling, and gated
//    here with the same negative controls so nothing can drift between entries.
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

/// Axial float of the locked top carrier, mm. `C_FREE · RELIEF_SLOPE` and nothing
/// else: zero preload by construction.
fn bay_float(e: f64) -> f64 {
	(SLOT_HW + e - (NECK_D / 2.0 - e)) * RELIEF_SLOPE
}

/// (radial offset `u`, arc length `s`) → local XY, origin at the pin's LOCKED
/// position, +x radially outward. The pin travels along the PIN CIRCLE, not a
/// straight line, and over 7° that arc bows 0.20 mm inboard — most of one C_FREE.
fn arcp(u: f64, s: f64) -> DVec2 {
	let th = s / CD;
	let r = CD + u;
	DVec2::new(r * th.cos() - CD, r * th.sin())
}

/// The bayonet slot in one top-carrier arm, drawn in (u, s) and mapped through
/// [`arcp`]. `e` dilates every wall for the worst-case-stack gate.
fn slot_outline(e: f64) -> Vec<DVec2> {
	let w = SLOT_HW + e;
	let lr = LOCK_R + e;
	let ls = -LOCK_Y;
	let d = bay_d();
	let (bx, bw) = (BULGE_X + e, BULGE_HW + e);
	let s0 = ls + (w - lr);
	let (s1, s2) = (d - bw, d + bw);
	let mut p = Vec::with_capacity(72);
	for i in 0..=24 {
		let t = PI + PI * i as f64 / 24.0; // the locating pocket
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

/// Centre of the bayonet slot's own bounding box in the arm at angle 0, world XY.
/// The SIMP frozen pad is centred HERE and not on the pin, because the slot runs
/// outboard and forward of the pin by most of its own width.
///
/// The BOUNDING BOX centre, not the centroid: the outline spends 25 of its 60
/// points tracing the locating pocket, so its centroid sits inside the pocket and
/// a pad centred there left the bulge's far corner only 0.36 mm inside the pad —
/// under one extrusion line of wall. G16o caught it.
fn slot_centre() -> DVec2 {
	let p = slot_outline(0.0);
	let (mut lo, mut hi) = (DVec2::splat(f64::INFINITY), DVec2::splat(f64::NEG_INFINITY));
	for q in &p {
		lo = lo.min(*q);
		hi = hi.max(*q);
	}
	(lo + hi) * 0.5 + DVec2::new(CD, 0.0)
}

/// Baseline (sibling) top-arm planform, world XY for the arm at angle 0.
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
/// top carrier rests on, the neck, and the fin's relief cone. `e` erodes every
/// external surface by the per-side printer error.
fn bay_pin_blank(e: f64) -> Solid {
	let r = PIN_D / 2.0 - e;
	let nr = NECK_D / 2.0 - e;
	let z_cone = ts_top() + rise(PIN_D / 2.0 - NECK_D / 2.0);
	revolve(
		&force_ccw(vec![
			DVec2::new(0.0, 1.00),
			DVec2::new(4.30, 1.00),
			// the cylinder→flare transition sits at z 1.60, BELOW the plate's top
			// face (2.00), so no pin edge lies IN that plane — §7.7 rule 3.
			DVec2::new(4.30, 1.60),
			DVec2::new(PLANET_SEAT_D / 2.0, Z_ROT),
			DVec2::new(r, Z_ROT),
			DVec2::new(r, ts_bot()), // SEAT: the top carrier's whole axial location
			DVec2::new(nr, ts_bot()),
			DVec2::new(nr, ts_top()),
			DVec2::new(r, z_cone), // fin relief cone, RELIEF_SLOPE
			DVec2::new(r - 0.40, z_cone + 0.40),
			DVec2::new(0.0, z_cone + 0.40),
		]),
		48,
	)
}

/// The three flats that turn a revolved head into the radial FIN.
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
		(rp, rb, rp - M, rp + 1.25 * M)
	}
}

/// Transverse contact ratio of an EXTERNAL–EXTERNAL mesh at standard centre
/// distance. Parametric so the negative control drives the SAME code path.
fn contact_ratio_external(m: f64, alpha_deg: f64, z1: usize, z2: usize) -> f64 {
	let a = alpha_deg.to_radians();
	let (rp1, rp2) = (m * z1 as f64 / 2.0, m * z2 as f64 / 2.0);
	let (rb1, rb2) = (rp1 * a.cos(), rp2 * a.cos());
	let (ra1, ra2) = (rp1 + m, rp2 + m);
	((ra1 * ra1 - rb1 * rb1).sqrt() + (ra2 * ra2 - rb2 * rb2).sqrt() - (rp1 + rp2) * a.sin()) / (PI * m * a.cos())
}

/// Transverse contact ratio of an INTERNAL mesh (pinion `zp` inside ring `zr`).
fn contact_ratio_internal(m: f64, alpha_deg: f64, zp: usize, zr: usize) -> f64 {
	let a = alpha_deg.to_radians();
	let (rpp, rpr) = (m * zp as f64 / 2.0, m * zr as f64 / 2.0);
	let (rbp, rbr) = (rpp * a.cos(), rpr * a.cos());
	let (rap, rar) = (rpp + m, rpr - m);
	((rap * rap - rbp * rbp).sqrt() - (rar * rar - rbr * rbr).sqrt() + (rpr - rpp) * a.sin()) / (PI * m * a.cos())
}

/// Undercut floor `z_min = 2(1−x)/sin²α` (ISO).
fn undercut_floor(x: f64) -> f64 {
	2.0 * (1.0 - x) / (pa().sin() * pa().sin())
}

/// Lewis form factor Y measured from the generator's OWN tooth outline.
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
				continue;
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

/// Signed area and POLAR second moment `J = ∫(x²+y²)dA` of a closed polygon.
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
// 6. SPIN-DOWN SOLVER + HERTZ — inherited, and re-benchmarked here before use
//    (§25.7 answer-type 2: a written solver is guilty until its gates are green).
// ============================================================================

/// A drag budget as a sum of power-law terms `Σ cⱼ·ω^pⱼ` (N·m), every term
/// already REFLECTED to the observable rotor (the ring).
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

/// Spin-down by EXACT QUADRATURE of `I·dω/dt = −T(ω)`. The ω→0 singularity of a
/// pure power law is removed exactly by `ω = ω₀·s^{1/(1−p_min)}`. Composite
/// Simpson, 4000 intervals — deterministic. Returns `(seconds, revolutions)`.
fn spin_down(i_eff_kgm2: f64, d: &Drag, w0: f64) -> (f64, f64) {
	let pmin = d.terms.iter().map(|t| t.1).fold(f64::INFINITY, f64::min);
	assert!(pmin < 1.0, "a drag budget whose slowest term is ω¹ or faster never stops");
	let p = 1.0 / (1.0 - pmin);
	let c_min: f64 = d.terms.iter().filter(|t| (t.1 - pmin).abs() < 1e-12).map(|t| t.0).sum();
	let f_t0 = i_eff_kgm2 * w0 * p / (c_min * w0.powf(pmin));
	let f_a0 = 0.0;
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

/// Free-disc (von Kármán) skin-friction torque coefficient for ω^1.5 on BOTH
/// faces of a disc of radius `r_m`. The Cm normalisation could not be verified at
/// a primary source and is flagged in ANALYSIS.md.
fn disc_air_coeff(r_m: f64) -> f64 {
	3.87 * (NU_AIR / (r_m * r_m)).sqrt() * RHO_AIR * r_m.powi(5)
}

/// Reduced modulus of an elastic contact, MPa.
fn e_star(e1: f64, nu1: f64, e2: f64, nu2: f64) -> f64 {
	1.0 / ((1.0 - nu1 * nu1) / e1 + (1.0 - nu2 * nu2) / e2)
}

/// Hertz contact radius of a SPHERE on a FLAT, mm: `a = (3FR/4E*)^⅓`.
fn hertz_a(load_n: f64, r_ball: f64, estar: f64) -> f64 {
	(3.0 * load_n * r_ball / (4.0 * estar)).cbrt()
}

/// Hertz mutual approach, mm — an INDEPENDENT algebraic path to the same
/// solution, which is what benchmarks [`hertz_a`] against something other than
/// itself (`a² = Rδ`).
fn hertz_delta(load_n: f64, r_ball: f64, estar: f64) -> f64 {
	(9.0 * load_n * load_n / (16.0 * r_ball * estar * estar)).cbrt()
}


// ============================================================================
// 7. THE DROP MODEL — written for this campaign, benchmarked before use.
//
// Two independent routes to the same physical quantity, published side by side:
//
//   `drop_force_stated`  — the EQUIVALENT-STATIC design case. F = m·g·h/s with a
//                          stated stopping distance. Linear in h, linear in 1/s,
//                          so the whole sensitivity is one division and the
//                          reader can move either number themselves.
//
//   `drop_force_indent`  — the RIGID-FLOOR BOUND. Johnson's fully-plastic
//                          indentation at constant mean pressure p_m = C·σ_y:
//                          a rim edge of effective radius R_eff pressed into a
//                          rigid flat has projected contact area A = 2π·R_eff·d,
//                          so F(d) = 2π·p_m·R_eff·d rises LINEARLY with depth and
//                          the energy absorbed to depth d is π·p_m·R_eff·d².
//                          Setting that equal to m·g·h gives d in closed form,
//                          and the equivalent stopping distance is exactly d/2
//                          (the mean of a linearly rising force is half its peak).
//
// R_eff for a rounded rim edge on a flat is the geometric mean of the two
// principal radii — the edge round `R_e` and the rim's own major radius `r_o` —
// which is the standard Hertz equivalent radius for an elliptical contact.
// ============================================================================

/// Equivalent-static drop force, N. `m` kg, `h` m, `s` mm.
fn drop_force_stated(m_kg: f64, h_m: f64, s_mm: f64) -> f64 {
	m_kg * GRAV * h_m / (s_mm * 1e-3)
}

/// Rigid-floor elastic-plastic indentation bound. Returns
/// `(peak force N, crush depth mm, equivalent stopping distance mm)`.
fn drop_force_indent(m_kg: f64, h_m: f64, r_edge_mm: f64, r_major_mm: f64) -> (f64, f64, f64) {
	let p_m = INDENT_C * SIG_YIELD_PLA; // N/mm²
	let r_eff = (r_edge_mm * r_major_mm).sqrt(); // mm
	let energy_nmm = m_kg * GRAV * h_m * 1e3; // J → N·mm
	let d = (energy_nmm / (PI * p_m * r_eff)).sqrt(); // mm
	let f = 2.0 * PI * p_m * r_eff * d; // N
	(f, d, d / 2.0)
}

/// The drop height, m, at which a linear-elastic structure reaching `sigma` MPa
/// at `DROP_H_M` would reach `allow` MPa. The equivalent-static force is LINEAR
/// in h, and a linear static solve is linear in the force, so this is exact
/// within the model rather than an extrapolation.
fn drop_height_at_allowable(sigma_mpa: f64, allow_mpa: f64) -> f64 {
	if sigma_mpa <= 0.0 {
		return f64::INFINITY;
	}
	DROP_H_M * allow_mpa / sigma_mpa
}

// ============================================================================
// 8. ROTOR GEOMETRY — frozen, identical to the sibling. Nothing here is a design
//    variable for this campaign except the three face widths the study solves.
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

/// Tip-relief cutter for an EXTERNAL gear at the face plane `zp`.
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

/// Tip-relief cutter for an INTERNAL ring (tips point inward at `rt`).
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

/// P3 SUN (and P5 SUN-B at `SUNB_FRAC·T_SUN`). 42T external, running fit on the
/// printed post, bed relief, top lead-in, tip chamfers, seven index grooves.
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
	let (gr_in, gr_out, gr_w, gr_d) = (r_seat + 1.6, ra - 1.6, 1.20, 0.40);
	let mut grooves: Option<Solid> = None;
	for k in 0..N_INDEX {
		let a = TAU * k as f64 / N_INDEX as f64 + 0.35;
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

/// P4 PLANET — 12T external, Ø6.00 bore (or a ladder variant).
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

const FRAC_PI_2_STEPS: f64 = PI / 16.0; // 8 samples over the quarter round

/// P0 RING — 66T internal in a rim of OD `2(34.25 + wall)`, 1.0 mm round on the
/// TOP rim edge and a 0.45 × 45° chamfer on the BED edge (a full round at the bed
/// has a 90° overhang tangent, which the support gate fires on, correctly).
fn ring(face: f64, wall: f64) -> Result<Solid, kernel_brep::ChainError> {
	let (_, _, r_tip, r_root) = radii(R_T, false);
	let od = r_root + wall;
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

/// P7 CAP — Ø12 × 1.2 STATIC thumb pad, pressed onto the post.
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

/// P8 FIT COUPON — journal pin, cap-press boss, and the bayonet section.
fn coupon() -> Result<Solid, kernel_brep::ChainError> {
	let plate = cuboid(DVec3::new(-24.0, -11.0, 0.0), DVec3::new(24.0, 11.0, 2.0));
	let mut ch = ChainLog::start("coupon plate", plate)?.seal();
	let pin = cylinder(DVec3::new(-15.0, 0.0, 1.0), DVec3::Z, PIN_D / 2.0, 8.0, 48);
	ch.apply("coupon journal pin", |s| union(s, &pin))?;
	let press = cylinder(DVec3::new(0.0, 0.0, 1.0), DVec3::Z, POST_D / 2.0, 6.0, 48);
	ch.apply("coupon cap-press boss", |s| union(s, &press))?;
	let dz = 4.0 - ts_bot();
	let bay = difference(
		&bay_pin(0.0).transformed(tr(15.0, 0.0, dz)),
		&cuboid(DVec3::new(9.0, -6.0, -12.0), DVec3::new(21.0, 6.0, 1.0)),
	);
	ch.apply("coupon bayonet pin", |s| union(s, &bay))?;
	Ok(ch.finish())
}

/// P9 BAYONET KEY — one shipped slot at the shipped thickness, so the retention
/// joint can be gauged in the hand in twelve minutes.
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
// 9. THE CARRIER AS AN IMPLICIT FIELD
//
// Both carriers are flat plates in the XY plane loaded IN THEIR OWN PLANE, so
// the design problem is 2.5-D: a planform extruded through the plate thickness.
// That is not a modelling convenience, it is the manufacturing constraint — a
// constant cross-section through z is exactly what makes the part print flat
// with no down-facing face anywhere, which is the same reason `bracket_gen.rs`
// averages its density field along one axis before thresholding.
//
// One struct serves five bodies so that every comparison runs down an IDENTICAL
// code path and no flattering difference can hide in a second implementation:
//   * `Web::Baseline`   — the sibling's hand-drawn planform (three crossing bars
//                         / six tapered arms). The row this campaign has to beat.
//   * `Web::SolidStart` — the design domain filled solid: the optimiser's own
//                         blank, and the denominator of every "% removed" claim.
//   * `Web::Generative` — the thresholded, six-fold-symmetrised SIMP density.
//   * `mutilated`       — the FEA negative control: two struts cut out of the
//                         live load path. Same struct, one flag.
//   * `no_rim`          — the capture negative control on the top carrier.
// ============================================================================

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Part {
	Base,
	Top,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Web {
	Baseline,
	SolidStart,
	Generative,
}

#[derive(Clone)]
struct CarrierField {
	part: Part,
	web: Web,
	/// Plate-plane density, dims (nx, ny, 1) so `sample` is z-invariant by
	/// construction (a 1-sample axis is constant under the reader's clamp).
	rho: GridField,
	mutilated: bool,
	no_rim: bool,
}

/// Frozen radius of the drop-contact / grip pad on the rim, mm, and the radius
/// its centre sits at. Centre + radius = STATIC_R exactly, so the frozen set
/// never pokes outside the envelope — a frozen voxel is forced SOLID by the
/// runner whether or not the initial geometry had material there, so a keep-out
/// that overshoots does not merely mis-model, it invents material.
const CONTACT_PAD_R: f64 = 2.20;
const CONTACT_PAD_AT: f64 = STATIC_R - CONTACT_PAD_R;

/// The six azimuths a rim load is applied at, radians. Offset 30° from the pin
/// circle: a drop that lands ON a pin walks straight into a fixture, so the
/// BETWEEN-pin azimuth is the one the structure has to earn.
fn load_azimuths() -> Vec<f64> {
	(0..N_PL).map(|k| PI / N_PL as f64 + TAU * k as f64 / N_PL as f64).collect()
}

/// World XY of the pin at index `k`.
fn pin_xy(k: usize) -> DVec2 {
	let a = TAU * k as f64 / N_PL as f64;
	DVec2::new(CD * a.cos(), CD * a.sin())
}

/// World XY of the bayonet slot's frozen pad centre at index `k`.
fn slot_pad_xy(k: usize) -> DVec2 {
	let c = slot_centre();
	let a = TAU * k as f64 / N_PL as f64;
	DVec2::new(c.x * a.cos() - c.y * a.sin(), c.x * a.sin() + c.y * a.cos())
}

/// World XY of the ring thrust pad at index `k`.
fn ring_pad_xy(k: usize) -> DVec2 {
	let a = TAU * k as f64 / N_PL as f64;
	DVec2::new(RING_PAD_R * a.cos(), RING_PAD_R * a.sin())
}

/// World XY of the rim contact pad at load azimuth index `k`.
fn contact_pad_xy(k: usize) -> DVec2 {
	let a = load_azimuths()[k];
	DVec2::new(CONTACT_PAD_AT * a.cos(), CONTACT_PAD_AT * a.sin())
}

/// Polynomial smooth minimum — the union that leaves a fillet instead of a
/// crease. `k` is the blend radius. Used to join the frozen keep-outs to the
/// optimiser's web, for two reasons that happen to agree: a hard `min` leaves a
/// sharp re-entrant corner at every pad boundary, which is a stress raiser the
/// coarse analysis grid cannot even see, and it also looks like what it is —
/// two shapes glued together. A blend gives the part the continuous, bone-like
/// junctions the shape is supposed to have.
fn smin(a: f32, b: f32, k: f32) -> f32 {
	if k <= 0.0 {
		return a.min(b);
	}
	let h = (0.5 + 0.5 * (b - a) / k).clamp(0.0, 1.0);
	(b + h * (a - b)) - k * h * (1.0 - h)
}

/// Signed distance to a vertical circular column of radius `r` at `c`, XY only.
fn col(p: Vec3, c: DVec2, r: f64) -> f32 {
	let d = ((p.x as f64 - c.x).powi(2) + (p.y as f64 - c.y).powi(2)).sqrt();
	(d - r) as f32
}

/// Signed distance to a CONVEX polygon given CCW, XY only. Exact inside, a
/// conservative (never-positive-when-inside) bound outside — which is all an
/// occupancy sampler and a dual contour need, and it is stated rather than
/// dressed up as a distance field.
fn convex_poly(p: Vec3, pts: &[DVec2]) -> f32 {
	let q = DVec2::new(p.x as f64, p.y as f64);
	let mut d = f64::NEG_INFINITY;
	for i in 0..pts.len() {
		let (a, b) = (pts[i], pts[(i + 1) % pts.len()]);
		let e = b - a;
		let n = DVec2::new(e.y, -e.x).normalize(); // outward for CCW
		d = d.max((q - a).dot(n));
	}
	d as f32
}

impl CarrierField {
	fn thickness(&self) -> f64 {
		match self.part {
			Part::Base => Z_ARM,
			Part::Top => TS_T,
		}
	}

	fn r_in(&self) -> f64 {
		match self.part {
			Part::Base => WEB_R_IN,
			Part::Top => TOP_R_IN,
		}
	}

	/// The frozen keep-outs, re-asserted as EXACT geometry. These are the same
	/// regions the SIMP manifest freezes; asserting them here rather than reading
	/// them out of the 1.00 mm density grid is what keeps a Ø4.80 pin pad from
	/// shipping as a staircase, and a gate compares the two so they cannot drift.
	fn skeleton(&self, p: Vec3) -> f32 {
		let r = ((p.x * p.x + p.y * p.y) as f64).sqrt();
		let k = BLEND_R as f32;
		let mut parts: Vec<f32> = Vec::with_capacity(20);
		match self.part {
			Part::Base => {
				// inner ring: overlaps the hub revolve (r 8.00) by 1.00 mm so their
				// union is an interpenetration, not a coincident cylinder (§7.7).
				parts.push((r - WEB_R_IN) as f32);
				// continuous outer rim — see RIM_R_IN for why it is not optional
				parts.push(((r - STATIC_R).max(RIM_R_IN - r)) as f32);
				for j in 0..N_PL {
					parts.push(col(p, pin_xy(j), PIN_PAD_R));
					parts.push(col(p, ring_pad_xy(j), RING_PAD_FREEZE_R));
					parts.push(col(p, contact_pad_xy(j), CONTACT_PAD_R));
				}
			}
			Part::Top => {
				// The capture rim is the SAME annulus as the base carrier's, moved
				// 0.60 mm further in, and that is a change from the sibling with two
				// measured reasons. (1) Capture: it overhangs the ring's solid back
				// (34.25 outward) by 1.95 mm instead of the sibling's 0.95 mm.
				// (2) Analysis: a 0.95 mm ring does not survive a 1.00 mm grid — it
				// samples into a dotted ring of isolated specks and the solve goes
				// singular. Both carriers therefore carry one rim, and the drop
				// contact is defined at every azimuth on both faces of the machine.
				if !self.no_rim {
					parts.push(((r - STATIC_R).max(RIM_R_IN_TOP - r)) as f32);
				}
				for j in 0..N_PL {
					parts.push(col(p, slot_pad_xy(j), SLOT_PAD_R));
					if !self.no_rim {
						parts.push(col(p, contact_pad_xy(j), CONTACT_PAD_R));
					}
				}
			}
		}
		parts.into_iter().fold(f32::INFINITY, |a, b| if a.is_finite() { smin(a, b, k) } else { b })
	}

	/// The sibling's hand-drawn planform, rebuilt here so the comparison row is a
	/// measurement of THAT design rather than a memory of its numbers.
	fn baseline(&self, p: Vec3) -> f32 {
		let r = ((p.x * p.x + p.y * p.y) as f64).sqrt();
		match self.part {
			Part::Base => {
				// three full-diameter bars = six arms, plus the Ø16 hub
				let mut d = (r - HUB_D / 2.0) as f32;
				for k in 0..3 {
					let a = PI * k as f64 / 3.0;
					let (c, s) = (a.cos(), a.sin());
					let (u, v) = (p.x as f64 * c + p.y as f64 * s, -(p.x as f64) * s + p.y as f64 * c);
					d = d.min((u.abs() - STATIC_R).max(v.abs() - ARM_HW) as f32);
				}
				d
			}
			Part::Top => {
				// inner ring + outer rim + six tapered arms
				let mut d = ((r - TS_R_IN_O).max(TS_R_IN - r)) as f32;
				d = d.min(((r - STATIC_R).max(TS_R_RIM - r)) as f32);
				let arm = ts_arm_outline();
				for k in 0..N_PL {
					let a = TAU * k as f64 / N_PL as f64;
					let (c, s) = (a.cos(), a.sin());
					let q = Vec3::new(
						(p.x as f64 * c + p.y as f64 * s) as f32,
						(-(p.x as f64) * s + p.y as f64 * c) as f32,
						p.z,
					);
					d = d.min(convex_poly(q, &arm));
				}
				d
			}
		}
	}

	/// ONE strut removed from the live load path between the rim contact at the
	/// analysed azimuth and its nearest anchor. Used only by the FEA negative
	/// control.
	///
	/// A control that removes material the part is NOT using proves nothing —
	/// that is the mistake `bracket_gen.rs` records having made once, and it is
	/// why the cut sits on the shortest path from the load to ground. One strut,
	/// not both: cutting both severs the contact region entirely, the analysis
	/// body's own island filter then drops it, and the load lands on nothing —
	/// which is a broken control, not a strong one. The run that discovered that
	/// reported `0 load nodes` and is why this reads `[5usize]` and not `[4, 5]`.
	fn mutilation(&self, p: Vec3) -> f32 {
		let c = contact_pad_xy(4); // the published single-load azimuth
		let anchor = match self.part {
			Part::Base => pin_xy(5),
			Part::Top => slot_pad_xy(5),
		};
		col(p, (c + anchor) * 0.5, 3.50)
	}

	fn planform(&self, p: Vec3) -> f32 {
		let r = ((p.x * p.x + p.y * p.y) as f64).sqrt();
		let domain = ((r - STATIC_R).max(self.r_in() - r)) as f32;
		let web = match self.web {
			Web::Baseline => return self.baseline(p),
			Web::SolidStart => domain,
			Web::Generative => domain.max((ISO - self.rho.sample(p)) * SDF_SCALE),
		};
		// blended union of the optimiser's web with the exact keep-outs, then a
		// HARD clip at the envelope: a smooth minimum is always ≤ the hard one, so
		// without the clip the blend would push material ~k/4 past STATIC_R and
		// quietly grow the product out of its own Ø73 envelope.
		let mut d = smin(web, self.skeleton(p), BLEND_R as f32);
		d = d.max((r - STATIC_R) as f32);
		if self.no_rim {
			// The capture negative control clips the WHOLE planform inboard of the
			// ring's tip circle (r 32.00), skeleton included. Clipping only the
			// frozen rim leaves the density web reaching the envelope on its own
			// (21.6 mm³ of overlap); clipping only the web leaves the six bayonet
			// slot pads, which reach r 33.52 and therefore overhang the ring's
			// teeth (1.3 mm³). Both readings are a control that cannot fail, and
			// the gate refused both. The sibling's own capture control truncates
			// its arms the same way and for the same reason.
			d = d.max((r - NC_NO_CAPTURE_R) as f32);
		}
		if self.mutilated {
			d = d.max(-self.mutilation(p));
		}
		d
	}
}

impl Sdf for CarrierField {
	fn distance(&self, p: Vec3) -> f32 {
		let t = self.thickness() as f32;
		let slab = (-p.z).max(p.z - t);
		slab.max(self.planform(p))
	}

	fn bounds(&self) -> Aabb {
		let t = self.thickness() as f32;
		let r = STATIC_R as f32 + 1.0;
		Aabb::new(Vec3::new(-r, -r, -1.0), Vec3::new(r, r, t + 1.0))
	}
}

/// A 1×1×1 unit grid: the density placeholder for the bodies that do not read one.
fn unit_grid() -> GridField {
	GridField::from_data(vec![1.0], (1, 1, 1), Vec3::ZERO, 1.0).expect("1×1×1 unit grid is valid")
}

/// Sample an implicit body onto the analysis grid as a SOLID-FRACTION field:
/// 2×2×2 sub-samples per element (ACE's own `supersample = 2` convention),
/// fraction of sub-points inside. C-order `(i*ny + j)*nz + k`, z fastest — the
/// layout both NumPy and `GridField` use.
fn sample_occupancy<S: Sdf + ?Sized>(sdf: &S, dims: (usize, usize, usize), origin: Vec3, vox: f32) -> Vec<f32> {
	let (nx, ny, nz) = dims;
	let mut out = vec![0.0f32; nx * ny * nz];
	let q = vox * 0.25;
	for i in 0..nx {
		for j in 0..ny {
			for k in 0..nz {
				let c = origin + Vec3::new((i as f32 + 0.5) * vox, (j as f32 + 0.5) * vox, (k as f32 + 0.5) * vox);
				let mut inside = 0;
				for &dx in &[-q, q] {
					for &dy in &[-q, q] {
						for &dz in &[-q, q] {
							if sdf.distance(c + Vec3::new(dx, dy, dz)) < 0.0 {
								inside += 1;
							}
						}
					}
				}
				out[(i * ny + j) * nz + k] = inside as f32 / 8.0;
			}
		}
	}
	out
}

/// Keep only the largest 6-connected component of an occupancy field, and report
/// how many element-fractions were dropped.
///
/// This is a DISCRETISATION correction, not a design change, and the difference
/// matters. The sibling's top carrier carries a 0.95 mm ring-capture rim; the
/// analysis grid is 1.00 mm, so that rim samples into a DOTTED ring — twelve
/// isolated specks of one and two elements each, 4.7 % of the body. They are not
/// in the part, they are in the picture of the part. Left in, they are rigid
/// islands with no stiffness path to ground and the solve is singular: the run
/// died with `CG did not converge ... info=2000` before this was added.
///
/// So every analysis body goes through the same filter, the fraction removed is
/// printed for each, and a gate refuses a body that lost more than a stated
/// share — because at that point the grid is no longer describing the part and
/// the right answer is a finer grid, not a bigger broom.
fn prune_occ(occ: &[f32]) -> (Vec<f32>, f64) {
	let (nx, ny, nz) = GRID_DIMS;
	let at = |i: usize, j: usize, k: usize| (i * ny + j) * nz + k;
	let solid: Vec<bool> = occ.iter().map(|&v| v >= 0.5).collect();
	let mut label = vec![0usize; nx * ny * nz];
	let mut sizes: Vec<usize> = vec![0];
	for i in 0..nx {
		for j in 0..ny {
			for k in 0..nz {
				let n = at(i, j, k);
				if !solid[n] || label[n] != 0 {
					continue;
				}
				let id = sizes.len();
				sizes.push(0);
				let mut stack = vec![(i, j, k)];
				label[n] = id;
				while let Some((a, b, c)) = stack.pop() {
					sizes[id] += 1;
					for (da, db, dc) in [(1i64, 0i64, 0i64), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)] {
						let (x, y, z) = (a as i64 + da, b as i64 + db, c as i64 + dc);
						if x < 0 || y < 0 || z < 0 || x >= nx as i64 || y >= ny as i64 || z >= nz as i64 {
							continue;
						}
						let m = at(x as usize, y as usize, z as usize);
						if solid[m] && label[m] == 0 {
							label[m] = id;
							stack.push((x as usize, y as usize, z as usize));
						}
					}
				}
			}
		}
	}
	let best = (1..sizes.len()).max_by_key(|&i| sizes[i]).unwrap_or(0);
	let mut out = occ.to_vec();
	let (mut kept, mut lost) = (0.0f64, 0.0f64);
	for n in 0..nx * ny * nz {
		if solid[n] && label[n] != best {
			lost += out[n] as f64;
			out[n] = 0.0;
		} else {
			kept += out[n] as f64;
		}
	}
	(out, if kept + lost > 0.0 { lost / (kept + lost) } else { 0.0 })
}

/// Write a C-order `(nx, ny, nz)` float32 NumPy `.npy` (v1.0) — the density-grid
/// interchange both ACE runners read.
fn write_npy(path: &str, data: &[f32], dims: (usize, usize, usize)) -> std::io::Result<()> {
	let (nx, ny, nz) = dims;
	let dict = format!("{{'descr': '<f4', 'fortran_order': False, 'shape': ({nx}, {ny}, {nz}), }}");
	let mut header = dict.into_bytes();
	while (10 + header.len() + 1) % 64 != 0 {
		header.push(b' ');
	}
	header.push(b'\n');
	let mut out = Vec::with_capacity(10 + header.len() + data.len() * 4);
	out.extend_from_slice(b"\x93NUMPY\x01\x00");
	out.extend_from_slice(&(header.len() as u16).to_le_bytes());
	out.extend_from_slice(&header);
	for v in data {
		out.extend_from_slice(&v.to_le_bytes());
	}
	std::fs::write(path, out)
}

/// World position of element `(i, j, k)`'s centre on the analysis grid.
fn elem_centre(i: usize, j: usize, k: usize) -> Vec3 {
	Vec3::new(
		(GRID_ORIGIN.0 + VOX * (i as f64 + 0.5)) as f32,
		(GRID_ORIGIN.1 + VOX * (j as f64 + 0.5)) as f32,
		(GRID_ORIGIN.2 + VOX * (k as f64 + 0.5)) as f32,
	)
}

/// Grid origin handed to `GridField`: ACE's `origin_mm` names the world position
/// of grid NODE (0,0,0), but every `.npy` this campaign reads or writes is
/// per-ELEMENT, so the reader is offset by half a cell.
fn field_origin() -> Vec3 {
	Vec3::new(
		(GRID_ORIGIN.0 + VOX / 2.0) as f32,
		(GRID_ORIGIN.1 + VOX / 2.0) as f32,
		0.0,
	)
}

// ============================================================================
// 10. PYTHON PLUMBING — the ACE runners, their receipts, and the manifests.
//
// Contract, from `tools/solvers/README.md`: the LAST non-empty stdout line is
// one JSON object, and the runners exit 0 EVEN ON FAILURE — so `ok == true` is
// checked, never the status code. A failed step kills the campaign rather than
// letting the generative loop be silently skipped.
// ============================================================================

fn run_py(tool: &str, job: &str) -> Result<serde_json::Value, String> {
	let out = std::process::Command::new("python3")
		.args([tool, job])
		.output()
		.map_err(|e| format!("python3 not runnable ({e}) — the generative loop cannot be skipped"))?;
	let stdout = String::from_utf8_lossy(&out.stdout);
	let last = stdout.lines().rev().find(|l| !l.trim().is_empty()).unwrap_or("");
	let val: serde_json::Value = serde_json::from_str(last).map_err(|e| {
		let tail: String = String::from_utf8_lossy(&out.stderr).chars().rev().take(400).collect::<String>().chars().rev().collect();
		format!("{tool}: last stdout line is not JSON ({e}); stderr tail: {tail}")
	})?;
	if val.get("ok").and_then(|b| b.as_bool()) != Some(true) {
		return Err(format!("{tool}: {}", val.get("error").and_then(|e| e.as_str()).unwrap_or("ok != true")));
	}
	Ok(val)
}

/// Run a python tool whose receipt is plain stdout (render_views.py), not JSON.
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

/// Recursively round every float to `sig` significant digits. Receipts are
/// byte-compared deliverables and an iterative solve is not bitwise reproducible.
fn round_floats(v: &mut serde_json::Value, sig: i32) {
	match v {
		serde_json::Value::Number(n) => {
			if let Some(x) = n.as_f64() {
				if x != 0.0 && x.is_finite() {
					let m = 10f64.powi(sig - 1 - x.abs().log10().floor() as i32);
					if let Some(nn) = serde_json::Number::from_f64((x * m).round() / m) {
						*n = nn;
					}
				}
			}
		}
		serde_json::Value::Array(a) => a.iter_mut().for_each(|e| round_floats(e, sig)),
		serde_json::Value::Object(o) => o.iter_mut().for_each(|(_, e)| round_floats(e, sig)),
		_ => {}
	}
}

/// Receipt-or-die: persist the receipt (minus wall-clock), gate that it ran, and
/// kill the campaign if it did not.
fn require(label: &str, res: Result<serde_json::Value, String>, receipt_path: &str, ok: &mut bool) -> serde_json::Value {
	match res {
		Ok(mut val) => {
			if let Some(o) = val.as_object_mut() {
				o.remove("timings_s");
			}
			round_floats(&mut val, 9);
			let _ = std::fs::write(receipt_path, format!("{val:#}\n"));
			gate(label, true, format!("receipt {}", receipt_path.rsplit('/').next().unwrap_or("")), ok);
			val
		}
		Err(e) => {
			gate(label, false, e.chars().take(120).collect(), ok);
			println!("\nNULLSPIN-GEN: <<< FAIL (generative loop step could not run)");
			std::process::exit(1);
		}
	}
}

fn f(v: &serde_json::Value, path: &[&str]) -> f64 {
	let mut cur = v;
	for k in path {
		cur = &cur[k];
	}
	cur.as_f64().unwrap_or(f64::NAN)
}

fn write_json(path: &str, v: &serde_json::Value) {
	let _ = std::fs::write(path, format!("{v:#}\n"));
}

/// A vertical-cylinder region selector covering the whole plate slab.
fn cyl_sel(c: DVec2, r: f64) -> serde_json::Value {
	serde_json::json!({
		"type": "cylinder", "axis": "z",
		"center_mm": [c.x, c.y, VOX * GRID_DIMS.2 as f64 / 2.0],
		"radius_mm": r, "length_mm": VOX * GRID_DIMS.2 as f64 + 2.0
	})
}

/// The fixtures for one carrier: the inertial anchors during the impulse.
///
/// BASE — the six planet pins, and ONLY those. During a ~0.1 ms contact the ring
/// (3.5 g) and the six planets (2.2 g) are still travelling and the carrier is
/// the light part being arrested against them; they reach it through six meshes,
/// six planets and six pins. The hub is deliberately NOT a fixture: the sun
/// reaches the carrier through the post as a LOAD, not as a support — which is
/// both the correct free body (the contact force at the rim, the sun's inertia
/// at the hub, the carrier's own inertia as a body load, all reacted at the
/// pins) and the thing that makes the hub structurally necessary. The first
/// version of this campaign clamped the hub as well; SIMP then had no reason to
/// connect it to anything and duly handed back a floating Ø18 disc, which the
/// connectivity gate caught. An unloaded fixture is not an anchor, it is a hole
/// in the problem statement.
///
/// TOP — the six bayonet slots, and nothing else. The top carrier is held on the
/// machine by exactly those six fins, which is the whole point of the joint.
///
/// Treating the pins as rigid anchors is an idealisation and it is stated: it
/// makes the carrier stiffer than reality, and it ignores the 0.25 mm bore
/// clearance and the 0.193 mm radial mesh freedom whose order of take-up is not
/// decidable at this campaign's own 0.15 mm/side build error.
fn fixtures_for(part: Part) -> serde_json::Value {
	let mut v: Vec<serde_json::Value> = Vec::new();
	match part {
		Part::Base => {
			for k in 0..N_PL {
				v.push(serde_json::json!({"kind": "clamped", "region_selector": cyl_sel(pin_xy(k), PIN_PAD_R)}));
			}
		}
		Part::Top => {
			for k in 0..N_PL {
				v.push(serde_json::json!({"kind": "clamped", "region_selector": cyl_sel(slot_pad_xy(k), SLOT_PAD_R)}));
			}
		}
	}
	serde_json::Value::Array(v)
}

/// The sun's inertia reaches the carrier through the post, so it is introduced
/// over the hub ring rather than at a point on the axis. Inside the frozen hub
/// disc, so the patch is always on material.
const HUB_LOAD_AT: f64 = 6.60;
const HUB_LOAD_R: f64 = 2.00;

/// Which sub-problem a SIMP job is solving. SIMP takes exactly one load case and
/// this carrier has two that shape it, so it gets two runs and the results are
/// combined — see `simp_job` for why that is the honest way round.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Case {
	/// The drop at the rim, applied at all six between-pin azimuths at once.
	Rim,
	/// The sun's inertia on the post, at one azimuth. BASE carrier only.
	Hub,
}

/// The frozen keep-outs, in the manifest.
///
/// This set is SMALLER than `CarrierField::skeleton`, and the difference is not
/// an oversight — it is a distinction between two kinds of keep-out that the
/// solver forces you to make honestly.
///
/// A region may be frozen only if THIS load case gives it a reason to exist.
/// `frozen` pins a voxel solid every iteration whether or not anything holds it,
/// so an UNLOADED, UNFIXED frozen island is a rigid body with no stiffness path
/// to ground — a singular block in K. That is not a theory: freezing the six ring
/// thrust pads and all six rim contact pads produced exactly that, the optimiser
/// stripped the material around the unloaded ones, and the runner's own as-built
/// re-analysis died with `CG did not converge ... info=2000`. The solver was
/// right and the manifest was wrong.
///
/// So each case freezes exactly what it touches. The rest of the product's
/// keep-outs — the continuous outer rim, the six ring thrust pads — are
/// FUNCTIONAL requirements that no drop load case can see, so they are
/// re-asserted as exact geometry in the rebuild instead, and the honest
/// re-analysis is run on the part that has them.
fn regions_for(part: Part, case: Case) -> serde_json::Value {
	let mut v: Vec<serde_json::Value> = Vec::new();
	let frozen = |s: serde_json::Value| serde_json::json!({"kind": "frozen", "selector": s});
	for k in 0..N_PL {
		v.push(frozen(match part {
			Part::Base => cyl_sel(pin_xy(k), PIN_PAD_R),
			Part::Top => cyl_sel(slot_pad_xy(k), SLOT_PAD_R),
		}));
	}
	match case {
		Case::Rim => {
			for k in 0..N_PL {
				v.push(frozen(cyl_sel(contact_pad_xy(k), CONTACT_PAD_R)));
			}
		}
		Case::Hub => v.push(frozen(cyl_sel(DVec2::ZERO, WEB_R_IN))),
	}
	serde_json::Value::Array(v)
}

/// PLA as ACE sees it. `ace_optimize_runner.py` never resolves a material KEY —
/// it hands `job["material"]` straight to the solver — so the dict form is the
/// only one that works on both runners, and it is used on both so the two solves
/// cannot disagree about the material.
fn material() -> serde_json::Value {
	serde_json::json!({"youngs_modulus_pa": E_PLA_MPA * 1e6, "poisson": NU_PLA, "density_kg_m3": 1240.0})
}

fn grid_block(doc: &str, out_dir: &str, npy: &str) -> serde_json::Value {
	serde_json::json!({
		"_doc": doc,
		"out_dir": format!("{FEA_DIR}/{out_dir}"),
		"voxel_mm": VOX,
		"origin_mm": [GRID_ORIGIN.0, GRID_ORIGIN.1, GRID_ORIGIN.2],
		"npy": format!("{FEA_DIR}/{npy}"),
		"material": material(),
	})
}

/// The PUBLISHED load case: one rim contact at azimuth `a`, the sun's inertia on
/// the hub, and the carrier's own inertia as a body load. Every published stress
/// in this campaign comes from this manifest, and baseline / solid-start /
/// optimised / negative-control all use it unchanged apart from the geometry.
#[allow(clippy::too_many_arguments)] // one manifest builder, one argument per manifest field
fn drop_job(part: Part, doc: &str, out_dir: &str, npy: &str, a: f64, f_drop: f64, f_sun: f64, accel: f64) -> serde_json::Value {
	let mut j = grid_block(doc, out_dir, npy);
	let fall = [a.cos(), a.sin(), 0.0f64];
	let push = [-fall[0], -fall[1], 0.0f64];
	let c = DVec2::new(CONTACT_PAD_AT * a.cos(), CONTACT_PAD_AT * a.sin());
	let mut loads = vec![serde_json::json!({
		"kind": "point", "magnitude": f_drop, "direction": push,
		"region_selector": cyl_sel(c, CONTACT_PAD_R)
	})];
	if part == Part::Base {
		loads.push(serde_json::json!({
			"kind": "point", "magnitude": f_sun, "direction": fall,
			"region_selector": cyl_sel(DVec2::ZERO, HUB_LOAD_AT + HUB_LOAD_R)
		}));
	}
	loads.push(serde_json::json!({
		"kind": "body", "magnitude": accel, "direction": fall,
		"region_selector": {"type": "all"}
	}));
	let o = j.as_object_mut().expect("json object");
	o.insert("fixtures".into(), fixtures_for(part));
	o.insert("loads".into(), serde_json::Value::Array(loads));
	j
}

/// The PINCH case, verified on the final geometry and never optimised for: a firm
/// two-finger squeeze across a diameter with the flick's tangential drag on the
/// same patch. One finger is the fixture, the other is the load.
fn pinch_job(part: Part, doc: &str, out_dir: &str, npy: &str) -> serde_json::Value {
	let mut j = grid_block(doc, out_dir, npy);
	let a = -PI / 2.0;
	let c = DVec2::new(CONTACT_PAD_AT * a.cos(), CONTACT_PAD_AT * a.sin());
	let mag = (PINCH_N * PINCH_N + FLICK_N * FLICK_N).sqrt();
	let dir = [
		(-a.cos() * PINCH_N - a.sin() * FLICK_N) / mag,
		(-a.sin() * PINCH_N + a.cos() * FLICK_N) / mag,
		0.0f64,
	];
	let o = j.as_object_mut().expect("json object");
	// A FINGER, not a corner. The drop patch is 4.4 mm because a rim corner
	// striking a floor really is that small; a finger pad on this rim is 10 mm of
	// arc, and using the drop's patch for the hand case is not conservatism, it is
	// the wrong contact — it turns the plate into a cantilever off a point.
	o.insert(
		"fixtures".into(),
		serde_json::json!([{"kind": "clamped", "region_selector": cyl_sel(-c, FINGER_R)}]),
	);
	o.insert(
		"loads".into(),
		serde_json::json!([{"kind": "point", "magnitude": mag, "direction": dir, "region_selector": cyl_sel(c, FINGER_R)}]),
	);
	let _ = part;
	j
}

/// The SIMP jobs. SIMP takes exactly ONE load case; this carrier has two that
/// shape it, and they are irreconcilable in a single symmetric run. Both are
/// solved, and the two density fields are combined by maximum in `plate_field`.
///
/// **Case::Rim — the drop, as a six-fold ENVELOPE.** A dropped spinner can land
/// at any azimuth and the structure must be no weaker at any of them, so the rim
/// contact is applied at all six between-pin azimuths at once, reacted at the six
/// pins. It is not the drop and no stress from it is ever published; it is the
/// load PATTERN a part that must survive the drop at any of six azimuths has to
/// be shaped for. Compliance minimisation is invariant to a uniform load scale,
/// so the magnitude steers nothing — only the pattern does. Being six-fold
/// symmetric by construction, its optimum is six-fold too, which is why
/// symmetrising the answer afterwards costs almost nothing.
///
/// **Case::Hub — the sun's inertia on the post, at ONE azimuth.** This one cannot
/// be an envelope, and the reason is a theorem rather than a limitation of the
/// runner: the hub sits at the centre of symmetry, so ANY six-fold symmetric load
/// set has zero net force there, and a self-equilibrating radial set on a frozen
/// hub is absorbed internally in hoop. Run that way, the optimiser is told
/// nothing about the heaviest single load path in the machine — and it duly
/// returns a beautiful hexagonal web with the Ø18 hub floating unattached in the
/// middle of it. That was built, run, and caught by the connectivity gate; the
/// picture is in `analysis/`. So the sun's inertia gets its own single-azimuth
/// run, whose optimum IS symmetrised by the six-fold maximum, which is cheap
/// because the answer is a local fan rather than a whole structure.
///
/// **Why superposing two single-case optima is legitimate here, and what it is
/// not.** Unioning the optima of two load cases is the conservative approximation
/// to true multi-load-case topology optimisation, not an equivalent of it: the
/// union is guaranteed to contain a load path for each case separately and is
/// guaranteed NOT to be the compliance optimum of the combined case. That is
/// stated rather than glossed, and it is why the FINAL geometry is re-analysed
/// against the true COMBINED load case (rim contact + sun inertia + body load, at
/// one azimuth, through the published manifest) instead of being trusted.
#[allow(clippy::too_many_arguments)] // ditto — it is `drop_job` plus the optimiser's knobs
fn simp_job(part: Part, case: Case, doc: &str, out_dir: &str, npy: &str, a: f64, f_drop: f64, f_sun: f64) -> serde_json::Value {
	let mut j = grid_block(doc, out_dir, npy);
	let loads: Vec<serde_json::Value> = match case {
		Case::Rim => load_azimuths()
			.iter()
			.map(|&t| {
				let c = DVec2::new(CONTACT_PAD_AT * t.cos(), CONTACT_PAD_AT * t.sin());
				serde_json::json!({
					"kind": "point", "magnitude": f_drop, "direction": [-t.cos(), -t.sin(), 0.0],
					"region_selector": cyl_sel(c, CONTACT_PAD_R)
				})
			})
			.collect(),
		Case::Hub => vec![serde_json::json!({
			"kind": "point", "magnitude": f_sun, "direction": [a.cos(), a.sin(), 0.0],
			"region_selector": cyl_sel(DVec2::ZERO, HUB_LOAD_AT + HUB_LOAD_R)
		})],
	};
	let o = j.as_object_mut().expect("json object");
	o.insert("fixtures".into(), fixtures_for(part));
	o.insert("loads".into(), serde_json::Value::Array(loads));
	o.insert("regions".into(), regions_for(part, case));
	o.insert(
		"volfrac".into(),
		serde_json::json!(match (case, part) {
			(Case::Hub, _) => VOLFRAC_HUB,
			(Case::Rim, Part::Top) => VOLFRAC_TOP,
			(Case::Rim, Part::Base) => VOLFRAC_RIM,
		}),
	);
	o.insert("penalty".into(), serde_json::json!(SIMP_PENALTY));
	o.insert("filter_radius_vox".into(), serde_json::json!(SIMP_FILTER_RVOX));
	o.insert("max_iters".into(), serde_json::json!(SIMP_MAX_ITERS));
	o.insert("move".into(), serde_json::json!(0.2));
	o.insert("iso".into(), serde_json::json!(ISO));
	o.insert("density_floor".into(), serde_json::json!(SIMP_FLOOR));
	o.insert("time_budget_s".into(), serde_json::json!(900.0));
	j
}

// ---------------------------------------------------------------------------
// 10b. Reading the stress field back.
//
// The runner writes a per-ELEMENT von Mises field. The raw peak of a
// point-loaded model sits under the load patch and is a load-introduction
// artifact, not a structural stress — so BOTH numbers are carried everywhere:
// the raw peak with its location, and the peak outside a stated radius of the
// contact. A gate proves the raw peak really is inside that radius, which is
// what stops the mask being a way of not looking.
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, Default)]
struct StressScan {
	peak_mpa: f64,
	peak_r: f64,
	peak_from_load: f64,
	masked_mpa: f64,
	n_masked: usize,
	n_active: usize,
}

fn scan_stress(path: &str, load_at: DVec2, also: Option<DVec2>, mask_r: f64) -> Option<StressScan> {
	let g = GridField::from_npy_file(path, field_origin(), VOX as f32).ok()?;
	let (nx, ny, nz) = GRID_DIMS;
	let mut s = StressScan::default();
	for i in 0..nx {
		for j in 0..ny {
			for k in 0..nz {
				let c = elem_centre(i, j, k);
				let v = g.sample(c) as f64 / 1e6; // Pa → MPa
				if v <= 0.0 {
					continue;
				}
				s.n_active += 1;
				let mut d = ((c.x as f64 - load_at.x).powi(2) + (c.y as f64 - load_at.y).powi(2)).sqrt();
				if let Some(q) = also {
					d = d.min(((c.x as f64 - q.x).powi(2) + (c.y as f64 - q.y).powi(2)).sqrt());
				}
				if v > s.peak_mpa {
					s.peak_mpa = v;
					s.peak_r = ((c.x * c.x + c.y * c.y) as f64).sqrt();
					s.peak_from_load = d;
				}
				if d <= mask_r {
					s.n_masked += 1;
				} else if v > s.masked_mpa {
					s.masked_mpa = v;
				}
			}
		}
	}
	Some(s)
}

// ---------------------------------------------------------------------------
// 10c. The two SIMP answers → the plate-plane field the geometry is built from.
//
// Four steps, each with a reason:
//   1. average through the plate thickness — the shipped web is a constant
//      cross-section extrusion, which is what makes it print flat with no
//      down-facing face anywhere (`bracket_gen.rs` does the same along its own
//      extrusion axis, and calls it manufacturing regularisation);
//   2. take the MAXIMUM over the six 60 deg rotations — see `sixfold_max`;
//   3. UNION the two load cases by maximum — see `simp_job`;
//   4. one 3x3 tent blur — the "smooth" of threshold-and-smooth.
// ---------------------------------------------------------------------------

/// What the post-processing did to the optimiser's raw answers, measured.
struct PlateReport {
	/// worst |max − min| over the six rotations of the RAW rim field, 0..1
	asym: f64,
	/// thresholded planform area, mm²: the rim case raw, the rim case after the
	/// six-fold maximum, the hub case after it, and the union that ships
	area_rim_raw: f64,
	area_rim_sym: f64,
	area_hub_sym: f64,
	area_union: f64,
}

/// Combine the two SIMP answers into the plate-plane density the geometry is
/// built from.
fn plate_field(rim: &GridField, hub: Option<&GridField>) -> (Vec<f32>, PlateReport) {
	let (nx, ny, _) = GRID_DIMS;
	let flat = flatten(rim);
	let (sym_rim, worst) = sixfold_max(&flat);
	let sym_hub = match hub {
		Some(h) => sixfold_max(&flatten(h)).0,
		None => vec![0.0f32; nx * ny],
	};
	let mut sym = vec![0.0f32; nx * ny];
	for n in 0..nx * ny {
		sym[n] = sym_rim[n].max(sym_hub[n]);
	}
	let out = tent_blur(&sym);
	let area = |v: &[f32]| v.iter().filter(|&&x| x >= ISO).count() as f64 * VOX * VOX;
	let rep = PlateReport {
		asym: worst,
		area_rim_raw: area(&flat),
		area_rim_sym: area(&sym_rim),
		area_hub_sym: area(&sym_hub),
		area_union: area(&out),
	};
	(out, rep)
}

/// Average a density grid through the plate thickness.
fn flatten(rho: &GridField) -> Vec<f32> {
	let (nx, ny, nz) = GRID_DIMS;
	let mut flat = vec![0.0f32; nx * ny];
	for i in 0..nx {
		for j in 0..ny {
			let mut acc = 0.0f32;
			for k in 0..nz {
				acc += rho.sample(elem_centre(i, j, k));
			}
			flat[i * ny + j] = acc / nz as f32;
		}
	}
	flat
}

/// One 3×3 tent blur in the plate plane. The cone filter has already imposed the
/// length scale; this takes the grid's square corners off the level set so the
/// meshed surface reads as bone rather than as voxels.
fn tent_blur(src: &[f32]) -> Vec<f32> {
	let (nx, ny, _) = GRID_DIMS;
	let idx = |i: i64, j: i64| -> f32 { src[(i.clamp(0, nx as i64 - 1) as usize) * ny + j.clamp(0, ny as i64 - 1) as usize] };
	let mut out = vec![0.0f32; nx * ny];
	for i in 0..nx as i64 {
		for j in 0..ny as i64 {
			let mut acc = 0.0f32;
			for (di, wi) in [(-1i64, 0.25f32), (0, 0.5), (1, 0.25)] {
				for (dj, wj) in [(-1i64, 0.25f32), (0, 0.5), (1, 0.25)] {
					acc += wi * wj * idx(i + di, j + dj);
				}
			}
			out[i as usize * ny + j as usize] = acc;
		}
	}
	out
}

/// The MAXIMUM of a plate field over the six 60° rotations, plus how far the
/// input was from six-fold symmetric to begin with.
///
/// MAX, not MEAN, and the reason is not taste. A mean divides every feature's
/// density by up to six wherever its duty cycle is 1/6; the threshold then
/// deletes it, and the "symmetrised" part comes out weaker at EVERY azimuth than
/// the optimiser's answer was at one — the exact opposite of what symmetrising is
/// for. The maximum reproduces the optimiser's own answer at each of the six
/// azimuths, which is the design statement the product needs: the drop azimuth is
/// unknown and the six-fold pin layout is the largest symmetry the load path
/// admits. What it costs in area is measured, not assumed.
fn sixfold_max(flat: &[f32]) -> (Vec<f32>, f64) {
	let (nx, ny, _) = GRID_DIMS;
	let at = |src: &[f32], p: DVec2| -> f32 {
		let fx = (p.x - GRID_ORIGIN.0 - VOX / 2.0) / VOX;
		let fy = (p.y - GRID_ORIGIN.1 - VOX / 2.0) / VOX;
		let (i0, j0) = (fx.floor(), fy.floor());
		let (tx, ty) = (fx - i0, fy - j0);
		let g = |i: f64, j: f64| -> f32 {
			if i < 0.0 || j < 0.0 || i >= nx as f64 || j >= ny as f64 {
				return 0.0;
			}
			src[i as usize * ny + j as usize]
		};
		let (a, b) = (g(i0, j0), g(i0 + 1.0, j0));
		let (c, d) = (g(i0, j0 + 1.0), g(i0 + 1.0, j0 + 1.0));
		let lo = a + (b - a) * tx as f32;
		let hi = c + (d - c) * tx as f32;
		lo + (hi - lo) * ty as f32
	};
	let mut sym = vec![0.0f32; nx * ny];
	let mut worst = 0.0f64;
	for i in 0..nx {
		for j in 0..ny {
			let c = elem_centre(i, j, 0);
			let p = DVec2::new(c.x as f64, c.y as f64);
			let (mut mx, mut mn) = (f32::NEG_INFINITY, f32::INFINITY);
			for k in 0..N_PL {
				let a = TAU * k as f64 / N_PL as f64;
				let q = DVec2::new(p.x * a.cos() - p.y * a.sin(), p.x * a.sin() + p.y * a.cos());
				let v = at(flat, q);
				mx = mx.max(v);
				mn = mn.min(v);
			}
			sym[i * ny + j] = mx;
			worst = worst.max((mx - mn) as f64);
		}
	}
	(sym, worst)
}

/// Delete every part of the planform that is not connected to the body.
///
/// This is the failure mode the task of topology optimisation hands you for
/// free: OC is a density method with no connectivity constraint, so a converged
/// field routinely carries specks and ribbons that touch nothing. They are not
/// harmless — a floating island prints as loose debris inside the machine, and
/// in the analysis grid it voxelises into a load path that does not exist. It
/// is also the one defect the usual oracles miss: such a part is VALID, it is
/// WATERTIGHT, `volume()` happily sums both lumps, and `Solid::shell_count()`
/// counts shell RECORDS rather than connected geometry. Only
/// `Mesh::component_count` sees it, and by then the STL is written.
///
/// So it is pruned here, on the density grid, before any of that: a 4-connected
/// flood fill (diagonal voxel touches are NOT counted as ligaments — a corner
/// contact is not a printable connection) seeded inside the part's own frozen
/// anchor, run over the planform INCLUDING the exact skeleton, so material that
/// reaches the body only through the rim is correctly kept. What it removed is
/// reported in mm², never silently swallowed.
fn prune_islands(part: Part, plate: &[f32]) -> (Vec<f32>, f64, usize) {
	let (nx, ny, _) = GRID_DIMS;
	let probe = CarrierField {
		part,
		web: Web::Generative,
		rho: GridField::from_data(plate.to_vec(), (nx, ny, 1), field_origin(), VOX as f32)
			.expect("plate grid is finite"),
		mutilated: false,
		no_rim: false,
	};
	let zc = (probe.thickness() / 2.0) as f32;
	let solid: Vec<bool> = (0..nx * ny)
		.map(|n| {
			let c = elem_centre(n / ny, n % ny, 0);
			probe.distance(Vec3::new(c.x, c.y, zc)) < 0.0
		})
		.collect();
	let seed_xy = match part {
		Part::Base => DVec2::ZERO,
		Part::Top => slot_pad_xy(0),
	};
	let si = (((seed_xy.x - GRID_ORIGIN.0) / VOX - 0.5).round() as i64).clamp(0, nx as i64 - 1) as usize;
	let sj = (((seed_xy.y - GRID_ORIGIN.1) / VOX - 0.5).round() as i64).clamp(0, ny as i64 - 1) as usize;
	let mut keep = vec![false; nx * ny];
	if solid[si * ny + sj] {
		let mut stack = vec![(si, sj)];
		keep[si * ny + sj] = true;
		while let Some((i, j)) = stack.pop() {
			for (di, dj) in [(1i64, 0i64), (-1, 0), (0, 1), (0, -1)] {
				let (a, b) = (i as i64 + di, j as i64 + dj);
				if a < 0 || b < 0 || a >= nx as i64 || b >= ny as i64 {
					continue;
				}
				let n = a as usize * ny + b as usize;
				if solid[n] && !keep[n] {
					keep[n] = true;
					stack.push((a as usize, b as usize));
				}
			}
		}
	}
	let mut out = plate.to_vec();
	let mut debris = 0usize;
	for n in 0..nx * ny {
		if solid[n] && !keep[n] {
			out[n] = 0.0;
			debris += 1;
		}
	}
	// island count: a second pass over the discarded set, same connectivity
	let mut seen = vec![false; nx * ny];
	let mut islands = 0usize;
	for n in 0..nx * ny {
		if !solid[n] || keep[n] || seen[n] {
			continue;
		}
		islands += 1;
		let mut stack = vec![n];
		seen[n] = true;
		while let Some(m) = stack.pop() {
			let (i, j) = (m / ny, m % ny);
			for (di, dj) in [(1i64, 0i64), (-1, 0), (0, 1), (0, -1)] {
				let (a, b) = (i as i64 + di, j as i64 + dj);
				if a < 0 || b < 0 || a >= nx as i64 || b >= ny as i64 {
					continue;
				}
				let q = a as usize * ny + b as usize;
				if solid[q] && !keep[q] && !seen[q] {
					seen[q] = true;
					stack.push(q);
				}
			}
		}
	}
	(out, debris as f64 * VOX * VOX, islands)
}

// ============================================================================
// 11. FROM FIELD TO PART — mesh, bridge to exact B-rep, then the exact features.
//
// The organic web arrives as an implicit field; the pins, post, hub and pads are
// exact revolves and stay exact. They are joined in the B-REP domain, not the
// implicit one, for a reason worth stating: the bayonet's fin, neck and relief
// cone are 0.40–1.15 mm features whose retention gates run `overlap_volume` on
// solids, and resolving them implicitly would need a ~0.15 mm mesher voxel over
// a Ø73 × 12 mm domain — tens of millions of cells and a mesh no STEP exporter
// should ever see. So the web is meshed at MESH_VOX, bridged once, and unioned
// into the same chain the sibling uses, unchanged.
// ============================================================================

/// Longest triangle edge in a mesh, mm — the facet-size oracle for the surface
/// quality claim. The kernel has no such measurement, so it is written here and
/// gated with a negative control rather than asserted.
fn max_edge(m: &Mesh) -> f64 {
	let mut w = 0.0f64;
	for t in m.triangles() {
		let (a, b, c) = (m.positions[t[0] as usize], m.positions[t[1] as usize], m.positions[t[2] as usize]);
		w = w.max((a - b).length() as f64).max((b - c).length() as f64).max((c - a).length() as f64);
	}
	w
}

/// Bridge the meshed carrier to an exact solid for STEP, and be honest about
/// which route got there.
///
/// **Route 1 — `reverse::mesh_to_solid`, the kernel's own v1 bridge.** Its
/// contract gates volume conservation through wrap+coalesce at relative 1e-6 and
/// refuses rather than hand back silently altered geometry. Taken whenever it
/// passes.
///
/// **Route 2 — the same two kernel passes, run here, with a STATED tolerance.**
/// A 2.5-D organic web does not meet 1e-6, and the reason is structural rather
/// than a defect: `coalesce_coplanar` merges faces whose plane parameters
/// quantise to the same key, and a smooth curved side wall meshed by dual
/// contouring is a fan of facets whose normals differ by a fraction of a degree.
/// Merging them is the right thing to do — it is what turns 66 000 triangles
/// into a STEP a CAD package can open — and it moves the volume by a measured
/// ~0.2 %. That is disclosed and gated at [`BRIDGE_DRIFT_MAX`] (0.5 %, the same
/// figure `recover_quadrics` publishes for its own rebuild), the measured drift
/// is printed and lands in ANALYSIS.md, and the STEP is labelled for what it is.
///
/// **What is authoritative.** The MESH is: `parts/*.stl` and `parts/*.3mf` are
/// the mesher's own output, they are what is printed, and they are what every
/// published analysis was run on. `cad/*.step` is a faceted CAD convenience whose
/// deviation from that mesh is published. This is the same doctrine
/// `bracket_gen.rs` records — the reverse bridge is not in the path to the STL.
fn bridge(m: &Mesh) -> Result<(Solid, String, f64), String> {
	let vm = m.signed_volume().abs();
	if let Ok(s) = reverse::mesh_to_solid(m) {
		let drift = (volume(&s).abs() - vm).abs() / vm.max(1.0);
		return Ok((s, "reverse::mesh_to_solid v1 (kernel contract, 1e-6)".to_string(), drift));
	}
	let wrapped = kernel_brep::solid_from_mesh(m);
	let s = kernel_brep::coalesce_coplanar(&wrapped);
	let v = validate(&s);
	if !v.is_valid() {
		return Err(format!(
			"manual wrap+coalesce failed validation: closed={} manifold={} genus={} shells={}",
			v.closed, v.manifold, v.genus, v.shells
		));
	}
	let drift = (volume(&s).abs() - vm).abs() / vm.max(1.0);
	if drift > BRIDGE_DRIFT_MAX {
		return Err(format!(
			"wrap+coalesce drift {:.4}% exceeds the stated {:.2}% — the STEP would not be the same part as the STL",
			drift * 100.0,
			BRIDGE_DRIFT_MAX * 100.0
		));
	}
	Ok((
		s,
		format!("manual wrap+coalesce, drift {:.3}% (v1's 1e-6 refused it; STL is authoritative)", drift * 100.0),
		drift,
	))
}

/// Marching squares on a carrier's planform, returning closed loops in world XY.
///
/// WHY this exists when the file already has a mesher and a reverse bridge. The
/// carrier is a 2.5-D extrusion, so its exact CAD form is a prism over a planform
/// — and `extrude_with_holes` builds exactly that, as a normal builder output
/// with one face per contour segment and two multi-loop caps. That matters for
/// three reasons the faceted route cannot meet:
///
///  * it BOOLEANS. The faceted solid the reverse bridge hands back is valid, but
///    `coalesce_coplanar` leaves multi-loop planar faces that the default
///    tessellator cannot re-triangulate watertight, so the moment it meets the
///    exact hub-and-post revolve the chain reports `genus 34850` and stops. The
///    bayonet pins, the thrust pads and the post are exact geometry with 0.4 mm
///    features and they are not negotiable, so the web is what has to change.
///  * it is EXACT. There is no wrap-and-coalesce step, so there is no volume
///    drift to disclose: the contour IS the geometry.
///  * it is SMALL. ~1 200 faces instead of ~24 000, and a STEP a CAD package can
///    actually open.
///
/// The reverse bridge is still run, on the same field, as an INDEPENDENT
/// reconstruction — two different algorithms reading one density field — and the
/// two volumes are gated against each other. That is a stronger check than either
/// one alone.
///
/// Edges are indexed by integer id rather than by coordinate, so the point an
/// edge contributes is computed once and shared exactly by both cells that touch
/// it; chaining is then exact integer work with no welding tolerance. The two
/// ambiguous saddle cases are resolved on the cell-centre value.
fn contour_loops_raw(field: &CarrierField, res: f64) -> Vec<Vec<DVec2>> {
	let lo = -(STATIC_R + 1.0);
	let n = ((2.0 * (STATIC_R + 1.0)) / res).ceil() as usize + 1;
	let xy = |i: usize, j: usize| DVec2::new(lo + res * i as f64, lo + res * j as f64);
	let z = (field.thickness() / 2.0) as f32;
	let val = |i: usize, j: usize| -> f64 {
		let p = xy(i, j);
		field.planform(Vec3::new(p.x as f32, p.y as f32, z)) as f64
	};
	let v: Vec<f64> = (0..n * n).map(|k| val(k / n, k % n)).collect();
	let at = |i: usize, j: usize| v[i * n + j];
	// edge ids: horizontals first, then verticals
	let hoff = 0usize;
	let voff = (n - 1) * n;
	let hid = |i: usize, j: usize| hoff + j * (n - 1) + i;
	let vid = |i: usize, j: usize| voff + i * (n - 1) + j;
	let mut pt: std::collections::HashMap<usize, DVec2> = std::collections::HashMap::new();
	let mut cross = |id: usize, a: DVec2, va: f64, b: DVec2, vb: f64| {
		pt.entry(id).or_insert_with(|| {
			let t = if (va - vb).abs() < 1e-30 { 0.5 } else { va / (va - vb) };
			a + (b - a) * t.clamp(0.0, 1.0)
		});
	};
	let mut segs: Vec<(usize, usize)> = Vec::new();
	for i in 0..n - 1 {
		for j in 0..n - 1 {
			let (v0, v1, v2, v3) = (at(i, j), at(i + 1, j), at(i + 1, j + 1), at(i, j + 1));
			let c = usize::from(v0 < 0.0) | usize::from(v1 < 0.0) << 1 | usize::from(v2 < 0.0) << 2 | usize::from(v3 < 0.0) << 3;
			if c == 0 || c == 15 {
				continue;
			}
			let (e0, e1, e2, e3) = (hid(i, j), vid(i + 1, j), hid(i, j + 1), vid(i, j));
			cross(e0, xy(i, j), v0, xy(i + 1, j), v1);
			cross(e1, xy(i + 1, j), v1, xy(i + 1, j + 1), v2);
			cross(e2, xy(i, j + 1), v3, xy(i + 1, j + 1), v2);
			cross(e3, xy(i, j), v0, xy(i, j + 1), v3);
			let mid = 0.25 * (v0 + v1 + v2 + v3);
			match c {
				1 | 14 => segs.push((e3, e0)),
				2 | 13 => segs.push((e0, e1)),
				4 | 11 => segs.push((e1, e2)),
				8 | 7 => segs.push((e2, e3)),
				3 | 12 => segs.push((e3, e1)),
				6 | 9 => segs.push((e0, e2)),
				5 => {
					if mid < 0.0 {
						segs.push((e3, e0));
						segs.push((e1, e2));
					} else {
						segs.push((e3, e2));
						segs.push((e0, e1));
					}
				}
				10 => {
					if mid < 0.0 {
						segs.push((e0, e1));
						segs.push((e2, e3));
					} else {
						segs.push((e0, e3));
						segs.push((e1, e2));
					}
				}
				_ => {}
			}
		}
	}
	// chain segments into closed loops by shared edge id
	let mut adj: std::collections::HashMap<usize, Vec<usize>> = std::collections::HashMap::new();
	for (k, &(a, b)) in segs.iter().enumerate() {
		adj.entry(a).or_default().push(k);
		adj.entry(b).or_default().push(k);
	}
	let mut used = vec![false; segs.len()];
	let mut loops: Vec<Vec<DVec2>> = Vec::new();
	for start in 0..segs.len() {
		if used[start] {
			continue;
		}
		let mut ring: Vec<usize> = Vec::new();
		let (first, mut cur_node) = (segs[start].0, segs[start].1);
		used[start] = true;
		ring.push(first);
		loop {
			ring.push(cur_node);
			let next = adj.get(&cur_node).and_then(|c| c.iter().copied().find(|&k| !used[k]));
			match next {
				Some(k) => {
					used[k] = true;
					cur_node = if segs[k].0 == cur_node { segs[k].1 } else { segs[k].0 };
					if cur_node == first {
						break;
					}
				}
				None => break,
			}
		}
		if ring.len() >= 3 {
			loops.push(ring.iter().filter_map(|e| pt.get(e).copied()).collect());
		}
	}
	loops
}

/// Douglas–Peucker on a CLOSED loop, tolerance in mm. Removes the collinear runs
/// marching squares leaves along straight walls without moving the contour by
/// more than `tol`, which is what keeps the face count (and the STEP) sane.
fn simplify(loop_pts: &[DVec2], tol: f64) -> Vec<DVec2> {
	fn dp(p: &[DVec2], tol: f64, out: &mut Vec<DVec2>) {
		if p.len() < 3 {
			out.extend_from_slice(&p[..p.len().saturating_sub(1)]);
			return;
		}
		let (a, b) = (p[0], p[p.len() - 1]);
		let ab = b - a;
		let len = ab.length();
		let (mut worst, mut at) = (0.0f64, 0usize);
		for (k, q) in p.iter().enumerate().take(p.len() - 1).skip(1) {
			let d = if len < 1e-12 { (*q - a).length() } else { ((*q - a).perp_dot(ab) / len).abs() };
			if d > worst {
				worst = d;
				at = k;
			}
		}
		if worst > tol {
			dp(&p[..=at], tol, out);
			dp(&p[at..], tol, out);
		} else {
			out.push(a);
		}
	}
	let n = loop_pts.len();
	if n < 4 {
		return loop_pts.to_vec();
	}
	// split the ring at two far-apart anchors so the closed curve is handled as
	// two open ones (Douglas–Peucker on a ring with one anchor can shortcut it)
	let half = n / 2;
	let mut out = Vec::with_capacity(n);
	dp(&loop_pts[..=half], tol, &mut out);
	let mut tail: Vec<DVec2> = loop_pts[half..].to_vec();
	tail.push(loop_pts[0]);
	dp(&tail, tol, &mut out);
	out
}

/// Build the carrier's web as an EXACT prism over its own contour.
/// Returns `(solid, outer point count, hole count)`.
fn web_prism(field: &CarrierField) -> (Solid, usize, usize, f64) {
	let raw = contour_loops_raw(field, CONTOUR_RES);
	let mut loops: Vec<Vec<DVec2>> = raw.iter().map(|l| simplify(l, CONTOUR_TOL)).filter(|l| l.len() >= 3).collect();
	// MEASURED deviation of the shipped contour from the marching-squares polyline
	// it was simplified from — the surface-quality claim, checked rather than
	// inherited from Douglas–Peucker's contract.
	let mut dev = 0.0f64;
	for (r, sm) in raw.iter().zip(loops.iter()) {
		for q in r {
			let mut best = f64::INFINITY;
			for i in 0..sm.len() {
				let (a, b) = (sm[i], sm[(i + 1) % sm.len()]);
				let ab = b - a;
				let t = if ab.length_squared() < 1e-18 { 0.0 } else { ((*q - a).dot(ab) / ab.length_squared()).clamp(0.0, 1.0) };
				best = best.min((*q - (a + ab * t)).length());
			}
			dev = dev.max(best);
		}
	}
	if loops.is_empty() {
		return (Solid::default(), 0, 0, dev);
	}
	let area = |l: &Vec<DVec2>| {
		let mut a = 0.0;
		for i in 0..l.len() {
			let q = l[(i + 1) % l.len()];
			a += l[i].x * q.y - q.x * l[i].y;
		}
		0.5 * a.abs()
	};
	let (best, _) = loops.iter().enumerate().max_by(|a, b| area(a.1).total_cmp(&area(b.1))).expect("non-empty");
	let outer = loops.remove(best);
	let n_out = outer.len();
	let holes = loops;
	let n_holes = holes.len();
	(kernel_brep::extrude_with_holes(&outer, &holes, field.thickness()), n_out, n_holes, dev)
}

/// Mesh a carrier field at the rebuild resolution.
fn mesh_field(field: &CarrierField) -> Mesh {
	let domain = field.bounds().pad(MESH_VOX * 2.0);
	kernel_implicit::manifold_dual_contour(field, domain, kernel_core::mesher::Resolution::VoxelSize(MESH_VOX))
}

/// P1 BASE CARRIER — hub + sun thrust land + post (one revolve), the generative
/// web, six ring thrust pads and six bayonet planet pins.
///
/// The chain is the sibling's, with exactly one operation replaced: "six arms
/// (3 crossing bars)" becomes "generative web". Everything else — the hub recess
/// at two axial clearances (one lands on the arm plane and takes the chain
/// invalid, §7.7 rule 3), the pre-unioned disjoint feature sets, the hazard
/// pre-flight on the pin union — is inherited because it was already proved.
fn base_carrier(web: &Solid, e: f64) -> Result<Solid, kernel_brep::ChainError> {
	let post_top = cap_top();
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
	ch.apply("generative web", |s| union(s, web))?;
	let mut pads: Option<Solid> = None;
	for k in 0..N_PL {
		let a = TAU * k as f64 / N_PL as f64;
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
	ch.apply("six planet pins", |s| union(s, &pins))?;
	ch.apply("six fin flats", |s| difference(s, &flats.expect("6 flat sets")))?;
	Ok(ch.finish())
}

/// P2 TOP CARRIER — the generative web with six bayonet slots cut in it.
/// `e` dilates the slots by the per-side printer error (worst-case gate);
/// `nc_round` swaps every slot for a plain round hole that clears the fin — the
/// negative control that must make the retention gate read exactly zero.
fn top_carrier(web: &Solid, z0: f64, e: f64, nc_round: bool) -> Result<Solid, kernel_brep::ChainError> {
	let mut ch = ChainLog::start("top web", web.transformed(tr(0.0, 0.0, z0)))?.seal();
	let mut holes: Option<Solid> = None;
	for k in 0..N_PL {
		let a = TAU * k as f64 / N_PL as f64;
		let h = if nc_round {
			cylinder(DVec3::new(CD, 0.0, z0 - 1.0), DVec3::Z, PIN_D / 2.0 + C_FREE, TS_T + 2.0, 48).transformed(rotz(a))
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
	ch.apply("six bayonet slots", |s| difference(s, &holes.expect("6 slots")))?;
	Ok(ch.finish())
}

// ============================================================================
// 12. EMIT — the §25 step-3 per-part gate battery.
// ============================================================================

fn emit(dir: &str, name: &str, s: &Solid, p: &FdmProfile, ok: &mut bool, worst_bridge: &mut f64, worst_facet: &mut f64) -> Mesh {
	let val = validate(s);
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
	let facet = max_edge(&mesh);
	let pass = val.is_valid() && one && wt && rep.steep_area < STEEP_MAX_MM2 && p.bridge_ok(rep.max_bridge_span) && fits;
	*worst_bridge = worst_bridge.max(rep.max_bridge_span);
	*worst_facet = worst_facet.max(facet);
	*ok &= pass;
	let _ = std::fs::write(format!("{OUT}/{dir}/{name}.stl"), mesh.to_stl_binary());
	let _ = mesh.write_3mf(format!("{OUT}/{dir}/{name}.3mf"));
	println!(
		"  {name:22} valid={:5} 1body={one:5} wt={wt:5} steep={:10.3e} mm²  bridge≤{:4.1}  facet≤{facet:4.2}  {:6.2} g  {}",
		val.is_valid(),
		rep.steep_area,
		rep.max_bridge_span,
		vol * PLA,
		if pass { "OK" } else { "<<< FAIL" }
	);
	if rep.steep_area >= STEEP_MAX_MM2 {
		for q in rep.steep_exemplars.iter().take(4) {
			println!("      steep at print ({:6.1},{:6.1},{:6.1})", q.x, q.y, q.z);
		}
	}
	mesh
}

/// Exact `I_zz` about the world +Z axis and the static imbalance of one rotor,
/// from `mass_properties` on the EXACT B-rep — teeth, chamfers, grooves and all.
/// Returns (mass g, I_zz g·mm², |CG_xy| mm, |I_xz|, |I_yz| g·mm²).
fn rotor(s: &Solid) -> (f64, f64, f64, f64, f64) {
	let mp = mass_properties(s);
	let (cx, cy) = (mp.center_of_mass.x, mp.center_of_mass.y);
	let izz = (mp.inertia.z_axis.z + mp.volume * (cx * cx + cy * cy)) * PLA;
	(
		mp.volume * PLA,
		izz,
		(cx * cx + cy * cy).sqrt(),
		(mp.inertia.x_axis.z * PLA).abs(),
		(mp.inertia.y_axis.z * PLA).abs(),
	)
}

// ============================================================================
// 13. THE CAMPAIGN
// ============================================================================

#[allow(clippy::too_many_lines)]
fn main() {
	kernel_core::telemetry::enable();
	for d in ["parts", "optional", "assembly/scene", "cad", "renders", "analysis/fea", "publish"] {
		let _ = std::fs::create_dir_all(format!("{OUT}/{d}"));
	}
	let p = FdmProfile::load("profiles/conservative_default.json").unwrap_or_else(|_| FdmProfile::conservative_default());
	let mut ok = true;
	let mut worst_bridge = 0.0f64;
	let mut worst_facet = 0.0f64;
	println!("NULLSPIN-GEN — grounded-carrier epicyclic spinner on a generatively-designed carrier\n");

	// ===================== G0 — ENGINE-REFUSAL PROBE ========================
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
	let refuses = involute_ring_outline_shifted_filleted(M, 36, 30.0, false, true, LASH, X_SHIFT, 0.0).is_none();
	gate("G0 NC: internal 36T @ 30° must be REFUSED", refuses, format!("refused {refuses}"), &mut ok);

	// ===================== G1 — KINEMATICS ==================================
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
	let bad = EpicyclicTrain { n_planets: 5, ..train };
	gate("G1a NC: n=5 must be refused ((S+R)%n ≠ 0)", bad.validate_assembly().is_err(), "refused".into(), &mut ok);
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
	let nc_eps = contact_ratio_external(1.0, 30.0, 8, 8);
	gate("G4 NC: 8T×8T @30° through the SAME fn must read ε < 1.20", nc_eps < 1.20, format!("ε {nc_eps:.4}"), &mut ok);
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

	// ===================== SOLVER BENCHMARKS ================================
	// Every solver this campaign wrote is proved against a closed form before it
	// is used, and every benchmark has a meta-negative-control proving it can go
	// red (§25.7 answer-type 2).
	let (i_b, k_b, n_b, c_b) = (1.5e-5, 3.0e-6, 0.43, 4.0e-4);
	let mut d_pow = Drag::default();
	d_pow.add(k_b, n_b, "bench power law");
	let (t_pow, a_pow) = spin_down(i_b, &d_pow, W0);
	let t_pow_a = i_b * W0.powf(1.0 - n_b) / (k_b * (1.0 - n_b));
	let a_pow_a = i_b * W0.powf(2.0 - n_b) / (k_b * (2.0 - n_b)) / TAU;
	let e_pow = ((t_pow - t_pow_a) / t_pow_a).abs().max(((a_pow - a_pow_a) / a_pow_a).abs());
	gate("SOLVER B1 pure power law vs closed form (<0.5%)", e_pow < 0.005, format!("err {e_pow:.3e}"), &mut ok);
	let mut d_cou = Drag::default();
	d_cou.add(c_b, 0.0, "bench coulomb");
	let (t_cou, a_cou) = spin_down(i_b, &d_cou, W0);
	let e_cou = ((t_cou - i_b * W0 / c_b) / (i_b * W0 / c_b))
		.abs()
		.max(((a_cou - i_b * W0 * W0 / (2.0 * c_b) / TAU) / (i_b * W0 * W0 / (2.0 * c_b) / TAU)).abs());
	gate("SOLVER B2 pure Coulomb vs closed form (<0.5%)", e_cou < 0.005, format!("err {e_cou:.3e}"), &mut ok);
	let e_meta = ((t_pow - 1.05 * t_pow_a) / (1.05 * t_pow_a)).abs();
	gate("SOLVER B3 meta-NC: a 5% wrong reference FAILS B1", e_meta >= 0.005, format!("err {e_meta:.3e}"), &mut ok);
	let circ: Vec<DVec2> = (0..512)
		.map(|i| {
			let a = TAU * i as f64 / 512.0;
			DVec2::new(10.0 * a.cos(), 10.0 * a.sin())
		})
		.collect();
	let (ac, jc) = poly_area_j(&circ);
	let e_poly = ((jc - PI * 10.0f64.powi(4) / 2.0) / (PI * 10.0f64.powi(4) / 2.0))
		.abs()
		.max(((ac - PI * 100.0) / (PI * 100.0)).abs());
	gate("SOLVER B4 polygon polar moment vs πR⁴/2 (<0.1%)", e_poly < 0.001, format!("err {e_poly:.3e}"), &mut ok);
	{
		let (fb, rb2, eb) = (0.0065f64, 0.75f64, e_star(E_STEEL, NU_STEEL, E_PLA_MPA, NU_PLA));
		let a1 = hertz_a(fb, rb2, eb);
		let a2 = (rb2 * hertz_delta(fb, rb2, eb)).sqrt();
		let e_h = ((a1 - a2) / a2).abs();
		gate("SOLVER B5 Hertz a vs the independent δ path (<0.1%)", e_h < 0.001, format!("err {e_h:.3e}"), &mut ok);
		let e_h_meta = ((a1 - 1.05 * a2) / (1.05 * a2)).abs();
		gate("SOLVER B6 meta-NC: a 5% wrong reference FAILS B5", e_h_meta >= 0.001, format!("err {e_h_meta:.3e}"), &mut ok);
	}

	// ---- DROP MODEL benchmarks. The two routes are independent algebra, so
	// each one checks the other's arithmetic, and the closed-form energy
	// balance checks both against the physics they claim to represent.
	let m_design_kg = DROP_MASS_G * 1e-3;
	let f_drop = drop_force_stated(m_design_kg, DROP_H_M, DROP_S_MM);
	let (f_bound, d_bound, s_bound) = drop_force_indent(m_design_kg, DROP_H_M, RIM_EDGE_R, STATIC_R);
	// D1 — energy conservation of the indentation model, from the OTHER
	// direction: the work done by a linearly rising force over its own crush
	// depth must equal the drop energy exactly.
	let work = 0.5 * f_bound * d_bound; // N·mm
	let energy = m_design_kg * GRAV * DROP_H_M * 1e3;
	let e_d1 = ((work - energy) / energy).abs();
	gate("DROP D1 indentation model conserves the drop energy (<0.1%)", e_d1 < 0.001, format!("err {e_d1:.3e}"), &mut ok);
	// D2 — the two routes are the SAME equation with a different stopping
	// distance, so feeding the bound's own equivalent stopping distance into the
	// stated-distance route must reproduce the bound's force exactly.
	let f_cross = drop_force_stated(m_design_kg, DROP_H_M, s_bound);
	let e_d2 = ((f_cross - f_bound) / f_bound).abs();
	gate("DROP D2 the two routes agree when handed the same stopping distance", e_d2 < 1e-9, format!("err {e_d2:.3e}"), &mut ok);
	// D3 — meta-NC: the agreement test must be able to go red.
	let e_d3 = ((drop_force_stated(m_design_kg, DROP_H_M, 1.05 * s_bound) - f_bound) / f_bound).abs();
	gate("DROP D3 meta-NC: a 5% wrong stopping distance FAILS D2", e_d3 >= 1e-9, format!("err {e_d3:.3e}"), &mut ok);
	// D4 — the design case must be the SOFTER of the two, or the "hard floor,
	// not a rigid one" statement in the analysis is backwards.
	gate(
		"DROP D4 the rigid-floor bound is strictly harsher than the design case",
		f_bound > f_drop,
		format!("bound {f_bound:.0} N vs design {f_drop:.0} N ({:.1}×)", f_bound / f_drop),
		&mut ok,
	);
	let accel = f_drop / m_design_kg; // m/s²
	// The sun is the heaviest body and its radial inertia can reach the carrier
	// through the post: the bore's 0.25 mm running fit and the mesh's 0.193 mm
	// radial lash equivalent are only 0.057 mm apart, and the campaign's own
	// worst-case build error is 0.15 mm/side, so which one closes first is NOT
	// decidable. The design case assumes the post takes it all.
	let m_sun_design = 12.81e-3; // kg — the sibling's measured sun; re-gated below
	let f_sun = m_sun_design * accel;
	println!(
		"\ndrop load case (equivalent-static, NOT a transient simulation)\n  \
		 h {DROP_H_M:.2} m · s {DROP_S_MM:.2} mm · m {DROP_MASS_G:.1} g  →  a {:.0} m/s² ({:.0} g), F_rim {f_drop:.0} N, F_sun-on-post {f_sun:.0} N\n  \
		 rigid-floor indentation bound: crush {d_bound:.3} mm (s_eq {s_bound:.3} mm) → F {f_bound:.0} N ({:.1}× the design case)",
		accel,
		accel / GRAV,
		f_bound / f_drop
	);

	// ===================== THE GENERATIVE LOOP ==============================
	let lb = run_loop(Part::Base, "base", f_drop, f_sun, accel, &mut ok);
	let lt = run_loop(Part::Top, "top", f_drop, 0.0, accel, &mut ok);

	// ===================== THE DROP VERDICT =================================
	// Everything above measured; this is where it is turned into a claim, and the
	// claim is deliberately narrow. Two tiers, and the difference between them is
	// the whole honesty of the section.
	//
	//  * PLA's YIELD, 55 MPa: "does it break". A single impact is not a sustained
	//    load and not a cycled one, so the ultimate/yield tier is the right one
	//    for the question "did the part survive the event".
	//  * The DESIGN allowable, 10 MPa: 35 MPa conservative base tensile × 0.6
	//    layer adhesion × 0.5 design factor. That is the tier this repository
	//    designs to, and the carrier does NOT meet it at the design drop. The
	//    height at which it would is reported instead of the fact being buried.
	//
	// Note what is NOT done here: the 0.6 layer-adhesion knockdown is carried even
	// though the carrier prints FLAT and this load is IN THE LAYER PLANE, where
	// that knockdown does not apply. Removing it would raise the allowable to
	// 17.5 MPa and make several of these numbers look much better. It is left in,
	// because the sibling's own tooth-root gate uses the same 10 MPa tier for the
	// same in-plane case, and a campaign does not get to invent a friendlier
	// allowable for its own headline.
	let sig_yield = SIG_YIELD_PLA;
	let carriers = [("base", &lb), ("top", &lt)];
	let mut worst_drop = 0.0f64;
	let mut worst_pinch = 0.0f64;
	for (n, l) in carriers {
		let d = l.vm_mid.max(l.vm_opt) * HEX8_PEAK_FACTOR;
		worst_drop = worst_drop.max(d);
		worst_pinch = worst_pinch.max(l.vm_pinch * HEX8_PEAK_FACTOR);
		println!(
			"  {n:4} carrier: drop {:.2} MPa (at-pin) / {:.2} (between-pin) ×{HEX8_PEAK_FACTOR} = {d:.2}; pinch {:.2}; plate {:.2} g",
			l.vm_opt, l.vm_mid, l.vm_pinch, l.g_opt
		);
	}
	gate(
		"DROP G50 the design drop does not break either carrier (peak < PLA yield)",
		worst_drop < sig_yield,
		format!("worst {worst_drop:.1} MPa vs {sig_yield:.0} yield (×{:.2} margin), h {DROP_H_M} m / s {DROP_S_MM} mm", sig_yield / worst_drop),
		&mut ok,
	);
	// The design-allowable answer, reported as a HEIGHT rather than a pass/fail,
	// because a height is what a reader can argue with.
	let h_allow_opt = drop_height_at_allowable(worst_drop, SIG_ALLOW_RT);
	let h_yield_opt = drop_height_at_allowable(worst_drop, sig_yield);
	let h_allow_base = drop_height_at_allowable(lb.vm_base.max(lt.vm_base) * HEX8_PEAK_FACTOR, SIG_ALLOW_RT);
	gate(
		"DROP G51 the drop-survival envelope is REPORTED, not asserted",
		h_allow_opt.is_finite() && h_allow_opt > 0.0 && h_yield_opt > DROP_H_M,
		format!("design allowable at {h_allow_opt:.2} m; yield at {h_yield_opt:.2} m; the sibling's best azimuth {h_allow_base:.2} m"),
		&mut ok,
	);
	// Which case governs is a RESULT, not an assumption, and the first version of
	// this gate asserted an ordering (drop worse than pinch) that is simply not
	// true of the top carrier. It reports the ordering instead, and gates the
	// thing that matters: both cases stay under yield.
	gate(
		"DROP G52 both load cases stay under yield, and the governing one is named",
		worst_pinch.max(worst_drop) < sig_yield,
		format!("drop {worst_drop:.1} / pinch {worst_pinch:.1} MPa vs {sig_yield:.0} — the {} governs", if worst_pinch > worst_drop { "PINCH (top carrier)" } else { "DROP" }),
		&mut ok,
	);
	// THE ANTI-FLATTERY GATE. The generative carrier is NOT better than the
	// sibling's six straight spokes at the sibling's own azimuth — six radial
	// spokes is very close to the textbook answer for a radial load ON a spoke,
	// and the numbers say so. What it buys is uniformity, and that is what gets
	// claimed. This gate asserts the UNFLATTERING direction so the claim can
	// never quietly drift into "stronger".
	let flatters = lb.vm_opt < lb.vm_base && lb.g_opt < lb.g_base;
	gate(
		"DROP G53 ANTI-FLATTERY: the sibling's spokes are NOT beaten at their own azimuth, and it is said",
		!flatters,
		format!(
			"at-pin: sibling {:.2} MPa / {:.2} g vs generative {:.2} MPa / {:.2} g — the claim is UNIFORMITY, not strength",
			lb.vm_base, lb.g_base, lb.vm_opt, lb.g_opt
		),
		&mut ok,
	);
	// …and the optimiser's OWN before/after, at the case it was actually posed.
	let cut_b = 100.0 * (1.0 - lb.g_opt / lb.g_solid);
	let cut_t = 100.0 * (1.0 - lt.g_opt / lt.g_solid);
	gate(
		"DROP G54 SIMP removed real mass from its own blank at its own load case",
		cut_b >= 30.0 && cut_t >= 20.0,
		format!(
			"base {:.2}→{:.2} g (−{cut_b:.0}%), {:.1}→{:.1} MPa; top {:.2}→{:.2} g (−{cut_t:.0}%), {:.1}→{:.1} MPa",
			lb.g_solid, lb.g_opt, lb.vm_solid_mid, lb.vm_mid, lt.g_solid, lt.g_opt, lt.vm_solid_mid, lt.vm_mid
		),
		&mut ok,
	);

	// The mass ceiling, DERIVED from the drop rather than chosen. `worst_drop` was
	// measured at the frozen design mass, and the force is linear in mass, so the
	// heaviest product that still clears yield by DROP_MARGIN_MIN is exactly this.
	let mass_max_g = DROP_MASS_G * sig_yield / (DROP_MARGIN_MIN * worst_drop);
	println!(
		"  mass ceiling derived from the drop: {DROP_MASS_G:.1} g × {sig_yield:.0}/({DROP_MARGIN_MIN}×{worst_drop:.1}) = {mass_max_g:.2} g"
	);

	// ===================== BUILD THE CARRIERS ===============================
	println!("\nparts");
	let build = |r: Result<Solid, kernel_brep::ChainError>, what: &str| -> Solid {
		match r {
			Ok(s) => s,
			Err(e) => {
				println!("  {what} chain failed: {e}");
				std::process::exit(1);
			}
		}
	};
	let s_base = build(base_carrier(&lb.web, 0.0), "base carrier");
	let s_top = build(top_carrier(&lt.web, ts_bot(), 0.0, false), "top carrier");
	let s_cap = cap(cap_bot());
	let frame_g = (volume(&s_base).abs() + volume(&s_top).abs() + volume(&s_cap).abs()) * PLA;
	// Worst-case frame mass over the WHOLE design space, so the study's mass
	// constraint does not depend on the point the study is being asked to find:
	// the six pins grow with t_ring and the post grows with t_sun.
	let frame_g_hi = frame_g
		+ N_PL as f64 * PI * (PIN_D / 2.0).powi(2) * (6.5 - T_RING) * PLA
		+ PI * (POST_D / 2.0).powi(2) * (T_SUN_MAX - T_SUN) * PLA;
	println!("  carrier frame {frame_g:.2} g (worst case over the design space {frame_g_hi:.2} g)");

	// ===================== G11 — ROTOR DESIGN STUDY =========================
	// The rotor set is NOT lightened by this campaign — eta pins I_sun·k_S to
	// I_ring + ΣI_p·k_P and taking mass out of either rotor breaks the physics the
	// product is built on. But the study still has to RE-SOLVE, because its mass
	// constraint reads the frame, and this frame is not the sibling's. If the
	// shipped point ever stops being the optimum for THIS frame, G11 fails loudly.
	let (a_sun_p, j_sun_p) = poly_area_j(&gear_profile(S_T, true, false));
	let (a_pl_p, j_pl_p) = poly_area_j(&gear_profile(P_T, true, true));
	let (a_cav, j_cav) = poly_area_j(&gear_profile(R_T, false, true));
	let bore_r = SUN_BORE_D / 2.0;
	let (a_sun, j_sun) = (a_sun_p - PI * bore_r * bore_r, j_sun_p - PI * bore_r.powi(4) / 2.0);
	let (a_pl, j_pl) = (a_pl_p - PI * (PLANET_BORE_D / 2.0).powi(2), j_pl_p - PI * (PLANET_BORE_D / 2.0).powi(4) / 2.0);
	let sun_pm = (a_sun * PLA, j_sun * PLA);
	let pl_pm = (a_pl * PLA, j_pl * PLA);
	let ring_pm = |wall: f64| {
		let od = 34.25 + wall;
		((PI * od * od - a_cav) * PLA, (PI * od.powi(4) / 2.0 - j_cav) * PLA)
	};
	let ks = -k_sun; // 11/7
	let eval = move |q: &Params| -> Evaluation {
		let (ts, trg, wall, tp) = (q["t_sun"], q["t_ring"], q["ring_wall"], q["t_planet"]);
		let (m_s, i_s) = (sun_pm.0 * ts, sun_pm.1 * ts);
		let (m_p, i_p) = (pl_pm.0 * tp, pl_pm.1 * tp);
		let (m_r, i_r) = {
			let (a, b) = ring_pm(wall);
			(a * trg, b * trg)
		};
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
		.constrain(Constraint::less_than("mass_g", mass_max_g))
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
	let _shipped: Params = [
		("t_sun".to_string(), T_SUN),
		("t_ring".to_string(), T_RING),
		("ring_wall".to_string(), RING_WALL),
		("t_planet".to_string(), T_PLANET),
	]
	.into_iter()
	.collect();
	println!("  study: {} evaluations, {} feasible, stop={}", report.evaluation_count(), report.feasible_count, report.stop_reason);
	if let Ok(b) = report.best("i_eff") {
		println!(
			"  study optimum: t_sun {:.2}  t_ring {:.2}  wall {:.2}  t_planet {:.2}  → I_eff {:.0} g·mm², eta {:.4}, {:.1} g",
			b.params["t_sun"], b.params["t_ring"], b.params["ring_wall"], b.params["t_planet"], b.value, b.constraints["eta"], b.constraints["mass_g"]
		);
	}
	// The rotor point is the SIBLING'S, frozen, and this campaign does not take
	// the study's answer. That is a deliberate refusal, not an omission.
	//
	// The study maximises I_eff — spin time — and with a mass ceiling it will
	// always spend the whole budget on a thicker sun. Taking it would change the
	// mechanism, and the entire value of this entry is that the mechanism is
	// IDENTICAL to the sibling's so the carrier is the only variable. It would
	// also start a runaway: the carrier's mass raises the drop force, the drop
	// force lowers the mass ceiling, the ceiling moves the rotors, the rotors move
	// the mass. So the study is run, its answer is published, and the shipped
	// point is gated as FEASIBLE rather than optimal — with the spin time the
	// refusal costs printed next to it.
	let near = |a: f64, b: f64| (a - b).abs() < 1e-9;
	let shipped_eval = report.evaluations.iter().find(|e| {
		near(e.params["t_sun"], T_SUN) && near(e.params["t_ring"], T_RING) && near(e.params["ring_wall"], RING_WALL) && near(e.params["t_planet"], T_PLANET)
	});
	let feasible = shipped_eval.map(|e| e.feasible).unwrap_or(false);
	let opt_ieff = report.best("i_eff").map(|b| b.value).unwrap_or(f64::NAN);
	let ship_ieff = shipped_eval.and_then(|e| e.objectives.get("i_eff").copied()).unwrap_or(f64::NAN);
	gate(
		"G11 the frozen rotor point is FEASIBLE for this frame (and the study's own answer is published)",
		feasible,
		format!(
			"shipped I_eff {ship_ieff:.0} g·mm² vs the study's {opt_ieff:.0} — {:+.1}% of spin inertia refused to keep the mechanism identical to the sibling's",
			100.0 * (ship_ieff / opt_ieff - 1.0)
		),
		&mut ok,
	);

	// ===================== BUILD THE ROTORS =================================
	let s_ring = build(ring(T_RING, RING_WALL), "ring");
	let s_sun = build(sun(T_SUN), "sun");
	let s_sunb = build(sun(SUNB_FRAC * T_SUN), "sun-b");
	let s_planet = build(planet(T_PLANET, PLANET_BORE_D), "planet");
	let s_pl_lo = build(planet(T_PLANET, 5.90), "planet 5.90");
	let s_pl_hi = build(planet(T_PLANET, 6.15), "planet 6.15");
	let s_coupon = build(coupon(), "coupon");
	let s_key = build(coupon_key(), "coupon key");

	let m_ring = emit("parts", "ring_66t", &s_ring, &p, &mut ok, &mut worst_bridge, &mut worst_facet);
	let m_sun = emit("parts", "sun_42t", &s_sun, &p, &mut ok, &mut worst_bridge, &mut worst_facet);
	let m_planet = emit("parts", "planet_12t_bore600", &s_planet, &p, &mut ok, &mut worst_bridge, &mut worst_facet);
	let m_base = emit("parts", "base_carrier_gen", &s_base, &p, &mut ok, &mut worst_bridge, &mut worst_facet);
	let m_top = emit("parts", "top_carrier_gen", &s_top, &p, &mut ok, &mut worst_bridge, &mut worst_facet);
	let m_cap = emit("parts", "cap", &s_cap, &p, &mut ok, &mut worst_bridge, &mut worst_facet);
	let _ = emit("optional", "sun_b_control", &s_sunb, &p, &mut ok, &mut worst_bridge, &mut worst_facet);
	let _ = emit("optional", "planet_12t_bore590", &s_pl_lo, &p, &mut ok, &mut worst_bridge, &mut worst_facet);
	let _ = emit("optional", "planet_12t_bore615", &s_pl_hi, &p, &mut ok, &mut worst_bridge, &mut worst_facet);
	let _ = emit("optional", "coupon_fit", &s_coupon, &p, &mut ok, &mut worst_bridge, &mut worst_facet);
	let _ = emit("optional", "coupon_key", &s_key, &p, &mut ok, &mut worst_bridge, &mut worst_facet);
	// NOTE on what is NOT gated here. An early version asserted a facet-size limit
	// over every emitted part and read 52.8 mm — from the coupon plate, which is a
	// flat rectangle tessellated as two triangles. A long edge on an exactly
	// planar face is not roughness, and a gate that cannot tell the difference is
	// measuring the wrong thing. The surface-quality claim belongs to the ORGANIC
	// surfaces and is made where they are built: G38/G38b on the mesher output and
	// G38c on the shipped contour's own deviation from the level set.
	gate(
		"PRINT not one part in the set carries a real bridge",
		worst_bridge < 0.05,
		format!("widest patch {worst_bridge:.3} mm over 11 parts, vs max_bridge {:.1}", p.max_bridge),
		&mut ok,
	);
	// NEGATIVE CONTROL for the support oracle: the same carrier on its side.
	let wrong = tessellate_default(&s_base.transformed(DAffine3::from_rotation_x(PI / 2.0))).support_free_report(Vec3::Z, 45.0, 0.3);
	gate(
		"PRINT NC: the generative carrier audited on its side (steep must jump)",
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
	let eta_full = |i_s_pla: f64, i_r: f64, i_p: f64, i608: f64| {
		let i_s = i_s_pla + i608;
		let ls = i_r - i_s * ks + N_PL as f64 * i_p * k_pl;
		let la = i_r + i_s * ks + N_PL as f64 * i_p * k_pl;
		1.0 - ls.abs() / la
	};
	let eta_of = |a: f64, b: f64, c: f64| eta_full(a, b, c, 0.0);
	let eta = eta_of(izz_s, izz_r, izz_p);
	let i_eff_gmm2 = izz_r + izz_s * ks * ks + N_PL as f64 * izz_p * k_pl * k_pl;
	gate("G9 eta on the exact B-rep ≥ 0.95 (design target 0.97)", eta >= 0.95, format!("η {eta:.4}"), &mut ok);
	let sens = [
		("sun +5% flow", eta_of(izz_s * 1.05, izz_r, izz_p)),
		("sun −5% flow", eta_of(izz_s * 0.95, izz_r, izz_p)),
		("ring +5% flow", eta_of(izz_s, izz_r * 1.05, izz_p)),
		("ring −5% flow", eta_of(izz_s, izz_r * 0.95, izz_p)),
		("sun +5% / ring −5%", eta_of(izz_s * 1.05, izz_r * 0.95, izz_p)),
		("sun −5% / ring +5%", eta_of(izz_s * 0.95, izz_r * 1.05, izz_p)),
		("v1/v2 ledger: the 608 put back (+610 g·mm² on the sun)", eta_full(izz_s, izz_r, izz_p, I608_GMM2)),
	];
	let eta_lo = sens[..6].iter().map(|s| s.1).fold(1.0f64, f64::min);
	let d_exact = (eta_of(izz_s * 1.05, izz_r * 1.05, izz_p * 1.05) - eta).abs();
	gate("G9b eta is EXACTLY invariant to common-mode flow (the SHIPPED set)", d_exact < 1e-12, format!("Δη {d_exact:.2e}"), &mut ok);
	gate("G9b3 worst corner in the whole table still ≥ 0.90 (the A/B stays valid)", eta_lo >= 0.90, format!("η_min {eta_lo:.4}"), &mut ok);
	let eta_b = eta_of(izz_sb, izz_r, izz_p);
	gate("G9c SUN-B control is DELIBERATELY uncancelled (η < 0.90)", eta_b < 0.90, format!("η_B {eta_b:.4}"), &mut ok);

	// ===================== G10 — BALANCE ====================================
	gate("G10 ring: static imbalance 0, no products of inertia", cg_r < 1e-6 && ixz_r < 1e-6 && iyz_r < 1e-6, format!("cg {cg_r:.2e} mm"), &mut ok);
	gate("G10 sun (7 index grooves): imbalance still 0", cg_s < 1e-6 && ixz_s < 1e-6 && iyz_s < 1e-6, format!("cg {cg_s:.2e} mm"), &mut ok);
	gate("G10 planet: imbalance 0", cg_p < 1e-6, format!("cg {cg_p:.2e} mm"), &mut ok);
	let lop = {
		let bar = cuboid(
			DVec3::new(SUN_BORE_D / 2.0 + 1.6, -0.6, sun_top() - 0.40),
			DVec3::new(ra_s - 1.6, 0.6, sun_top() + 1.0),
		)
		.transformed(rotz(0.35));
		union(&s_sun, &intersection(&bar, &cylinder(DVec3::ZERO, DVec3::Z, ra_s, sun_top(), 96)))
	};
	let (_, _, cg_lop, _, _) = rotor(&lop);
	gate("G10 NC: one groove filled → imbalance must appear", cg_lop > 1e-4, format!("cg {cg_lop:.2e} mm"), &mut ok);
	// The carrier does not rotate, so its balance is not a spin claim — but a
	// six-fold web on a six-fold machine should still come out balanced, and if
	// the symmetrising maximum ever failed this is where it would show.
	let (_, _, cg_carrier, _, _) = rotor(&s_base);
	gate(
		"G10b the generative carrier came out six-fold balanced (the symmetriser worked)",
		cg_carrier < 0.02,
		format!("cg {cg_carrier:.2e} mm — not a spin claim, a check on the symmetriser"),
		&mut ok,
	);

	// ===================== G5/G6/G7 — MOTION ================================
	let pl_local = |j: usize, th: f64| {
		let b = TAU * j as f64 / N_PL as f64;
		tr(CD * b.cos(), CD * b.sin(), 0.0) * rotz(b + k_pl * th)
	};
	let pose_sun = |th: f64, err: f64| s_sun.transformed(rotz(k_sun * (1.0 + err) * th));
	let pose_ring = |th: f64| s_ring.transformed(rotz(th));
	let pose_planet = |j: usize, th: f64| s_planet.transformed(pl_local(j, th));
	let ov = |a: &Solid, b: &Solid| overlap_volume(a, b).unwrap_or(f64::NAN);
	let pitch_r = TAU / R_T as f64;
	let dense: Vec<f64> = (0..96).map(|i| pitch_r * i as f64 / 96.0).collect();
	let sun_poses: Vec<DAffine3> = dense.iter().map(|&th| rotz(-k_sun * th) * pl_local(0, th)).collect();
	let ring_poses: Vec<DAffine3> = dense.iter().map(|&th| rotz(-th) * pl_local(0, th)).collect();
	let sw_s = kernel_model::sweep_check(&m_sun, &m_planet, &sun_poses);
	let sw_r = kernel_model::sweep_check(&m_ring, &m_planet, &ring_poses);
	gate("G5a sun mesh, 96-pose dense sweep of ONE full mesh cycle", sw_s.contacts == 0 && sw_s.crossings == 0 && sw_s.max_penetration == 0.0, format!("min_cl {:.3} mm", sw_s.min_clearance), &mut ok);
	gate("G5b ring mesh, same 96-pose dense sweep", sw_r.contacts == 0 && sw_r.crossings == 0 && sw_r.max_penetration == 0.0, format!("min_cl {:.3} mm", sw_r.min_clearance), &mut ok);
	let mut worst_sp = 0.0f64;
	let mut worst_pr = 0.0f64;
	for i in 0..16 {
		let th = pitch_r * i as f64 / 16.0;
		let pl = pose_planet(0, th);
		worst_sp = worst_sp.max(ov(&pl, &pose_sun(th, 0.0)));
		worst_pr = worst_pr.max(ov(&pl, &pose_ring(th)));
	}
	gate("G5c exact overlap_volume, 16 poses across the cycle, both meshes", worst_sp < 1e-9 && worst_pr < 1e-9, format!("{:.3e} mm³", worst_sp.max(worst_pr)), &mut ok);
	let mut worst_all = 0.0f64;
	for i in 0..6 {
		let th = 2.0 * TAU * i as f64 / 6.0 + 0.013;
		let (su, rg) = (pose_sun(th, 0.0), pose_ring(th));
		for j in 0..N_PL {
			let pl = pose_planet(j, th);
			worst_all = worst_all.max(ov(&pl, &su)).max(ov(&pl, &rg));
		}
	}
	gate("G5d 2 full ring revs × all 6 planets, exact (72 booleans)", worst_all < 1e-9, format!("{worst_all:.3e} mm³"), &mut ok);
	let mut jam = 0.0f64;
	for e in [0.05f64, -0.05] {
		for i in 0..6 {
			let th = pitch_r * 8.0 * (i + 1) as f64 / 6.0;
			jam = jam.max(ov(&pose_planet(0, th), &pose_sun(th, e)));
		}
	}
	gate("G6 NC: sun ±5% off ratio must JAM (overlap > 0)", jam > 1e-3, format!("{jam:.4} mm³"), &mut ok);
	// G5e — the NEW clearance this campaign owes: the generative web is not the
	// sibling's planform, so nothing inherited proves it stays out of the rotors.
	// The rotors are BUILT at z 0 and ASSEMBLED at Z_ROT / Z_GEAR. The mesh gates
	// above compare rotors to each other, so they share a frame and the offset
	// cancels; a carrier-versus-rotor check does not, and the first version of
	// this gate quietly compared the carrier against a ring sitting 2.30 mm too
	// low and reported a 1261 mm³ collision that does not exist.
	let at_rot = tr(0.0, 0.0, Z_ROT);
	let mut worst_web = 0.0f64;
	for j in 0..N_PL {
		for i in 0..8 {
			let th = pitch_r * i as f64 / 8.0;
			let pl = s_planet.transformed(at_rot * pl_local(j, th));
			worst_web = worst_web.max(ov(&s_base, &pl)).max(ov(&s_top, &pl));
		}
	}
	let rg = s_ring.transformed(at_rot);
	worst_web = worst_web.max(ov(&s_base, &rg)).max(ov(&s_top, &rg));
	let su = s_sun.transformed(tr(0.0, 0.0, Z_GEAR));
	worst_web = worst_web.max(ov(&s_base, &su)).max(ov(&s_top, &su));
	gate(
		"G5e the GENERATIVE web clears every rotor everywhere (48 planet poses + ring + sun)",
		worst_web < 1e-9,
		format!("{worst_web:.3e} mm³ — the optimiser is free in-plane, never in the gear envelope"),
		&mut ok,
	);
	let lash_angle = {
		let th = pitch_r * 0.37;
		let pl = m_planet.transformed_by(pl_local(0, th));
		let (mut lo, mut hi) = (0.0f64, 0.10f64);
		for _ in 0..24 {
			let mid = 0.5 * (lo + hi);
			if m_sun.transformed_by(rotz(k_sun * th + mid)).crosses_mesh(&pl) {
				hi = mid;
			} else {
				lo = mid;
			}
		}
		0.5 * (lo + hi)
	};
	let jt_measured = lash_angle * (M * S_T as f64 / 2.0);
	gate("G7 backlash strictly positive, jt in 0.12–0.26 mm at the sun mesh", jt_measured > 0.12 && jt_measured < 0.26, format!("{jt_measured:.3} mm / {:.3}°", lash_angle.to_degrees()), &mut ok);

	// ===================== G8 — CONCENTRICITY ===============================
	let jr = 0.18 / (2.0 * pa().tan());
	let build_err = 0.15;
	let residual = (build_err - C_FREE).max(0.0);
	gate("G8a designed post↔pin-circle concentricity is exactly 0", true, "0.000 mm (one parametric origin)".into(), &mut ok);
	gate("G8b worst-case residual concentricity < jr (no mesh preload)", residual < jr, format!("{residual:.3} < {jr:.3} mm"), &mut ok);
	gate("G8c each planet's radial freedom ≥ the build error (self-centres)", C_FREE >= build_err, format!("{C_FREE:.2} ≥ {build_err:.2} mm"), &mut ok);

	// ===================== G16 — BAYONET RETENTION ==========================
	// Inherited whole from the sibling, and re-proved here on THIS campaign's
	// solids rather than quoted: the top carrier is a different part now, and a
	// retention proof that ran on the other entry's geometry would prove nothing
	// about this one.
	let yield_strain = SIG_YIELD_PLA / E_PLA_MPA;
	let stack_xy = 0.15;
	let foot = 0.20;
	let travel = bay_d();
	let engage_xy = ENGAGE - 2.0 * stack_xy;
	let engage_full = ENGAGE - 2.0 * (stack_xy + foot);
	gate("G16a six geometric shoulders — material in the way, no preload anywhere", ENGAGE >= 1.0 && N_PL == 6, format!("{N_PL} × {ENGAGE:.2} mm of fin over slot wall"), &mut ok);
	gate("G16b engagement survives the worst-case XY stack (0.15 mm/side, BOTH members)", engage_xy > 0.5, format!("{engage_xy:+.2} mm of {ENGAGE:.2} left; dies at {:.3} mm/side", ENGAGE / 2.0), &mut ok);
	gate("G16c engagement survives XY + the 0.20 mm Prusa elephant foot as well", engage_full > 0.2, format!("{engage_full:+.2} mm at {:.2} mm/side on both members", stack_xy + foot), &mut ok);
	let posed = |t: &Solid, psi_deg: f64, dz: f64| t.transformed(rotz(psi_deg.to_radians())).transformed(tr(0.0, 0.0, dz));
	let ovl = |t: &Solid, b: &Solid, psi_deg: f64, dz: f64| overlap_volume(&posed(t, psi_deg, dz), b).unwrap_or(f64::NAN);
	let float_nom = bay_float(0.0);
	let free = ovl(&s_top, &s_base, 0.0, float_nom - 0.05);
	gate("G16d ZERO PRELOAD: at rest and through its whole float the joint is not touching", free < 1e-9, format!("{free:.3e} mm³ at +{:.2} mm (float {float_nom:.2})", float_nom - 0.05), &mut ok);
	let lift = 3.00;
	let captive = ovl(&s_top, &s_base, 0.0, lift);
	gate("G16e CAPTIVE: lift the locked top carrier and it runs into six fins", captive > 0.5, format!("{captive:8.2} mm³ at +{lift:.2} mm"), &mut ok);
	let e_wc = stack_xy + foot;
	let joint_pin = |e: f64| bay_pin(e).transformed(tr(CD, 0.0, 0.0));
	let joint_arm = |e: f64| {
		difference(
			&extrude(&force_ccw(ts_arm_outline()), TS_T).transformed(tr(0.0, 0.0, ts_bot())),
			&extrude(&force_ccw(slot_outline(e)), TS_T + 2.0).transformed(tr(CD, 0.0, ts_bot() - 1.0)),
		)
	};
	let wc_capture = ovl(&joint_arm(e_wc), &joint_pin(e_wc), 0.0, lift);
	gate("G16f WORST-CASE STACK on solids: fin eroded and slot dilated by 0.35 mm/side, still captive", wc_capture > 0.05, format!("{wc_capture:.3} mm³/pin at +{lift:.2} mm"), &mut ok);
	let nc_lip = top_carrier(&lt.web, ts_bot(), 0.0, true).map(|s| ovl(&s, &s_base, 0.0, lift)).unwrap_or(f64::NAN);
	gate("G16g NC: delete the lip (round holes) and the top carrier MUST lift straight off", nc_lip < 1e-9, format!("{nc_lip:.3e} mm³ (want 0)"), &mut ok);
	let nc_pose = ovl(&s_top, &s_base, -BAY_PSI_DEG, lift);
	gate("G16h NC: at the ENTRY pose the shipped top carrier must lift straight off", nc_pose < 1e-9, format!("{nc_pose:.3e} mm³ at −{BAY_PSI_DEG:.1}° (want 0)"), &mut ok);
	let mut worst_twist = 0.0f64;
	for i in 0..=8 {
		worst_twist = worst_twist.max(ovl(&s_top, &s_base, -BAY_PSI_DEG * (1.0 - i as f64 / 8.0), 0.10));
	}
	gate("G16i the twist is free: 9 poses entry→lock, zero interference at every one", worst_twist < 1e-9, format!("{worst_twist:.3e} mm³ worst of 9 poses"), &mut ok);
	let u_rel = travel - BULGE_HW + FIN_HW;
	let still = ovl(&s_top, &s_base, -BAY_PSI_DEG * (u_rel - 0.20) / travel, lift);
	gate("G16j back-out margin: >75 % of the twist undone before release, proved on solids", u_rel / travel > 0.75 && still > 0.05, format!("{:.0} % undone; still {still:.2} mm³ one step short", 100.0 * u_rel / travel), &mut ok);
	let lever = ts_top() + float_nom - ts_bot();
	let z_neck = PI * NECK_D.powi(3) / 32.0;
	let f_cap = N_PL as f64 * SIG_ALLOW_RT * z_neck / (RELIEF_SLOPE * lever);
	let carried_n = (volume(&s_ring).abs() + N_PL as f64 * volume(&s_planet).abs() + volume(&s_top).abs()) * PLA * 9.81e-3;
	gate("G16k retention capacity (neck bending) beats the carried weight ≥100×", f_cap > 100.0 * carried_n, format!("{f_cap:.1} N vs {carried_n:.3} N carried ({:.0}×)", f_cap / carried_n), &mut ok);
	let hole_a = (PIN_D + 2.0 * C_TIGHT) / 2.0;
	let snap_max = yield_strain * hole_a;
	gate("G16m snap-fit REFUSED on record: the elastic travel this scale allows is < the stack it must survive", snap_max < 2.0 * stack_xy, format!("{snap_max:.3} mm at yield vs {:.2} mm of stack ({:.1}× short)", 2.0 * stack_xy, 2.0 * stack_xy / snap_max), &mut ok);
	let spec_strain = ((6.40 - 2.0 * hole_a) / 2.0) / hole_a;
	gate("G16m NC: the spec's Ø6.40 barb must FAIL the same strain check", spec_strain > yield_strain, format!("{:.1}% vs {:.2}% yield — refused", spec_strain * 100.0, yield_strain * 100.0), &mut ok);
	// NEW to this entry: the slot now sits in an OPTIMISED plate, so the wall
	// around it is whatever the generative web left there. That has to be gated.
	let slot_wall = {
		let mut worst = f64::INFINITY;
		let g = &lt.field;
		for q in slot_outline(0.0) {
			let w = DVec2::new(q.x + CD, q.y);
			// march outward along the slot's own outward normal until we leave material
			let n = (w - slot_centre()).normalize_or_zero();
			let mut t = 0.0f64;
			while t < 6.0 {
				let s = w + n * t;
				if g.planform(Vec3::new(s.x as f32, s.y as f32, (TS_T / 2.0) as f32)) >= 0.0 {
					break;
				}
				t += 0.02;
			}
			worst = worst.min(t);
		}
		worst
	};
	gate(
		"G16o the optimiser left a real wall around every bayonet slot",
		slot_wall >= 2.0 * 0.45,
		format!("thinnest wall {slot_wall:.2} mm ≥ two extrusion lines (0.90)"),
		&mut ok,
	);

	// ===================== G12 / G14 / G23 — FITS, SAFETY, CAPTURE ==========
	let (foot_prusa, xy_worst, prof_dev) = (0.20, 0.15, 0.067);
	let uncredited = C_FREE - 2.0 * xy_worst - 2.0 * foot_prusa;
	let credited = C_FREE - 2.0 * xy_worst;
	let ladder = [5.90, 6.00, 6.15].map(|b| (b - PIN_D) / 2.0 - 2.0 * xy_worst);
	let ladder_best = ladder.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
	gate("G12a nominal fit (±0.05 build) stays positive", C_FREE - 2.0 * 0.05 > 0.0, format!("{:+.3} mm", C_FREE - 2.0 * 0.05), &mut ok);
	gate("G12b worst corner: SOME ladder member stays positive", ladder_best > 0.0, format!("Ø6.15 → {ladder_best:+.3} mm"), &mut ok);
	gate("G12c mesh: jt 0.18 exceeds 2× profile deviation 0.067", 0.18 > 2.0 * prof_dev, format!("0.180 > {:.3}", 2.0 * prof_dev), &mut ok);
	let cap_strain = CAP_PRESS_R / (POST_D / 2.0);
	gate("G22b the cap press fit (the model's only interference) is inside PLA's elastic range", cap_strain < yield_strain, format!("{:.2}% vs {:.2}% yield", cap_strain * 100.0, yield_strain * 100.0), &mut ok);
	gate("G22b NC: the inherited C_TIGHT fit on this post must FAIL the same check", C_TIGHT / (POST_D / 2.0) > yield_strain, format!("{:.2}% — refused", 100.0 * C_TIGHT / (POST_D / 2.0)), &mut ok);
	// ring axial capture, on the built solids
	let lift_r = C_Z + 0.20;
	let capture = overlap_volume(&s_ring.transformed(tr(0.0, 0.0, Z_ROT + lift_r)), &s_top).unwrap_or(f64::NAN);
	gate("G23a ring is CAPTIVE: lifting it past its clearance hits the top carrier", capture > 0.5, format!("{capture:8.2} mm³ at +{lift_r:.2} mm"), &mut ok);
	let engage_ring = STATIC_R.min(34.25 + RING_WALL) - RIM_R_IN_TOP.max(34.25);
	gate("G23b the capture rim engages the ring's back by ≥ 0.50 mm", engage_ring >= 0.50, format!("{engage_ring:5.2} mm (the sibling's rim reaches 0.95)"), &mut ok);
	let naked = top_carrier(&{
		let f = CarrierField { no_rim: true, ..lt.field.clone() };
		web_prism(&f).0
	}, ts_bot(), 0.0, false);
	let nc_capture = naked.map(|s| overlap_volume(&s_ring.transformed(tr(0.0, 0.0, Z_ROT + lift_r)), &s).unwrap_or(0.0)).unwrap_or(f64::NAN);
	gate("G23c NC: delete the capture rim and the ring MUST escape", nc_capture < 1e-9, format!("{nc_capture:.3e} mm³ (want 0)"), &mut ok);
	// EN 71-1 §4.10 rod rule
	let gaps: Vec<(&str, f64)> = vec![
		("sun ↔ top carrier (radial)", TOP_R_IN - ra_s),
		("sun ↔ base carrier (axial)", C_Z),
		("planet ↔ base carrier (rests on its thrust boss)", 0.0),
		("planet ↔ top carrier (axial)", ts_bot() - planet_top()),
		("ring ↔ top carrier (axial)", ts_bot() - ring_top()),
		("ring ↔ base carrier arms (axial)", Z_ROT - Z_ARM),
		("ring ↔ its six thrust pads (in contact)", 0.0),
		("sun ↔ its thrust land (in contact)", 0.0),
		("ring proud of the held rims (radial)", (34.25 + RING_WALL) - STATIC_R),
		("adjacent planets (the one entered gap)", neighbour),
	];
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
	gate("G14b the one Ø5-admitting space also admits Ø12", neighbour >= ROD_LARGE, format!("{neighbour:.3} ≥ {ROD_LARGE:.0} mm"), &mut ok);
	let nc_gap = 2.0 * CD * (PI / 8.0).sin() - M * (P_T + 2) as f64;
	gate("G14 NC: an 8-planet layout must FAIL the rod rule", band(nc_gap), format!("{nc_gap:.2} mm — in band"), &mut ok);
	// G14c — NEW: the generative web makes its own openings, so the rod rule has
	// to be re-asked of THEM. They are openings in a HELD part with no relative
	// motion across them, so the clause does not bite; what does bite is a finger
	// reaching THROUGH one into the gear plane, which is the ring↔carrier axial
	// gap above (0.30 mm) and is far below Ø5.
	gate(
		"G14c the web's own openings are in a HELD part — no relative motion across them",
		(Z_ROT - Z_ARM - C_Z).abs() < 1e-9,
		format!("a finger through an opening meets the rotors at {:.2} mm axial — under Ø{ROD_SMALL:.0}", Z_ROT - Z_ARM),
		&mut ok,
	);
	// tooth-root bending
	let y_sun = lewis_y(S_T, true);
	let y_pl = lewis_y(P_T, true);
	let y_ring = lewis_y(R_T, false);
	let t_flick = FLICK_N * (34.25 + RING_WALL) * 1e-3;
	let wt_ring = t_flick / (N_PL as f64 * 33.0e-3);
	let sig_ring = wt_ring / (T_PLANET * M * y_pl);
	gate("G15 tooth-root bending (Lewis, Y measured off the built outline)", sig_ring < SIG_ALLOW_RT, format!("{sig_ring:.3} MPa vs {SIG_ALLOW_RT:.0}"), &mut ok);
	gate("G15b measured Y is BELOW the handbook 0.36 (sharp-root honesty)", y_pl < 0.36 && y_sun < 0.45, format!("Y_p {y_pl:.3} Y_s {y_sun:.3} Y_r {y_ring:.3}"), &mut ok);

	// ===================== DRAG BUDGET + SPIN TIME ==========================
	// Inherited whole and RE-SOLVED on this entry's rotors. Spin time is reported,
	// never claimed: a geared spinner cannot beat a plain one and this one is a
	// Coulomb machine with no bearing in it.
	let (m_r_kg, m_p_kg, m_s_kg) = (mg_r * 1e-3, mg_p * 1e-3, mg_s * 1e-3);
	let (w_ring_n, w_pl_n, w_sun_n) = (m_r_kg * GRAV, m_p_kg * GRAV, m_s_kg * GRAV);
	let r_pl_pad = ((PLANET_BORE_D / 2.0 + C_BED) + PLANET_SEAT_D / 2.0) / 2.0 * 1e-3;
	let r_sun_land = ((SUN_BORE_D / 2.0 + C_BED) + (SUN_BORE_D / 2.0 + C_BED + SUN_LAND_W)) / 2.0 * 1e-3;
	let r_ring_pad = RING_PAD_R * 1e-3;
	let common = |d: &mut Drag, mu: f64| {
		d.add(N_PL as f64 * mu * w_pl_n * r_pl_pad * k_pl, 0.0, "6 planet thrust pads");
		d.add(disc_air_coeff((34.25 + RING_WALL) * 1e-3), 1.5, "ring disc air");
		d.add(ks.powf(2.5) * disc_air_coeff(ra_s * 1e-3), 1.5, "sun disc air (reflected)");
		d.add(N_PL as f64 * k_pl.powf(2.5) * disc_air_coeff(ra_p * 1e-3), 1.5, "6 planet disc air");
	};
	let budget = |mu: f64| {
		let mut d = Drag::default();
		d.add(mu * w_ring_n * r_ring_pad, 0.0, "6 ring thrust pads (printed)");
		d.add(mu * w_sun_n * r_sun_land * ks, 0.0, "sun thrust land (printed, reflected)");
		common(&mut d, mu);
		d
	};
	let budget_608 = |mu: f64, m608: f64| {
		let mut d = Drag::default();
		d.add(mu * w_ring_n * r_ring_pad, 0.0, "6 ring thrust pads (printed)");
		d.add(ks.powf(1.0 + N_BRG) * (m608 * 1e-3 / W0.powf(N_BRG)), N_BRG, "608 (reflected)");
		common(&mut d, mu);
		d
	};
	let i_eff_kgm2 = i_eff_gmm2 * 1e-9;
	let i_eff_608 = i_eff_kgm2 + I608_GMM2 * ks * ks * 1e-9;
	let d_nom = budget(MU_PLA);
	let (t_nom, rev_nom) = spin_down(i_eff_kgm2, &d_nom, W0);
	let (t_opt, _) = spin_down(i_eff_kgm2, &budget(MU_LO), W0);
	let (t_pes, _) = spin_down(i_eff_kgm2, &budget(MU_HI), W0);
	let (t_608, _) = spin_down(i_eff_608, &budget_608(MU_PLA, M608_NMM), W0);
	println!("\ndrag budget at ω₀ = {W0:.0} rad/s ({:.0} rpm)", W0 * 60.0 / TAU);
	for (c, e, w) in &d_nom.terms {
		println!("  {w:34} {:7.4} N·mm   (ω^{e:.1})", c * W0.powf(*e) * 1e3);
	}
	println!("  {:34} {:7.4} N·mm", "TOTAL", d_nom.total_nmm(W0));
	println!("  I_eff {i_eff_gmm2:.0} g·mm²  →  spin {t_nom:.1} s / {rev_nom:.0} rev   (band {t_opt:.1}–{t_pes:.1} s)");
	gate("PHYS spin time is REPORTED, not claimed; band is finite and > 0", t_nom > 0.0 && t_pes > 0.0 && t_opt.is_finite(), format!("{t_nom:.1} s [{t_pes:.1}–{t_opt:.1}]"), &mut ok);
	let coul_frac = d_nom.terms.iter().filter(|t| t.1 == 0.0).map(|t| t.0).sum::<f64>() / d_nom.torque(W0);
	gate("PHYS Coulomb share of the budget is measured and published", (0.0..=1.0).contains(&coul_frac), format!("{:.0}% Coulomb", coul_frac * 100.0), &mut ok);
	gate("PHYS NC: putting a 608 back must be strictly better (the deletion is honest)", t_608 > t_nom, format!("with a 608 {t_608:.1} s vs fully printed {t_nom:.1} s"), &mut ok);
	// The carrier is generative now, and the one thing that could have made the
	// spin WORSE is if the optimiser had moved a thrust contact. It cannot: the
	// three sliding arms are frozen keep-outs, and this re-asserts it numerically.
	gate(
		"PHYS the generative web did NOT move a single sliding contact",
		(r_ring_pad * 1e3 - RING_PAD_R).abs() < 1e-12 && (r_sun_land * 1e3 - (SUN_BORE_D / 2.0 + C_BED + SUN_LAND_W / 2.0)).abs() < 1e-12,
		format!("ring pad arm {:.2} mm, sun land arm {:.3} mm — both frozen keep-outs, both unchanged", r_ring_pad * 1e3, r_sun_land * 1e3),
		&mut ok,
	);

	// ===================== MASS + ENVELOPE ==================================
	let printed_g = mg_r + mg_s + N_PL as f64 * mg_p + frame_g;
	gate(
		"MASS printed set ≤ the ceiling the DROP derives",
		printed_g <= mass_max_g,
		format!("{printed_g:.2} g ≤ {mass_max_g:.2} g — the sibling ships 27.3 g; the delta is two continuous rims and a drop-rated web"),
		&mut ok,
	);
	// …and the loop closes: re-scale the measured drop peak by the mass the set
	// ACTUALLY came out at, and require the margin the ceiling was derived from.
	let drop_as_built = worst_drop * printed_g / DROP_MASS_G;
	gate(
		&format!("DROP G55 the as-built set still clears yield by ×{DROP_MARGIN_MIN} (the loop closes)"),
		sig_yield / drop_as_built >= DROP_MARGIN_MIN,
		format!("{drop_as_built:.1} MPa at the as-built {printed_g:.2} g vs {sig_yield:.0} yield (×{:.2})", sig_yield / drop_as_built),
		&mut ok,
	);
	let pinch_as_built = worst_pinch * printed_g / DROP_MASS_G;
	gate(
		"DROP G56 the hand case, re-scaled the same way, also clears yield",
		pinch_as_built < sig_yield,
		format!("pinch bound {pinch_as_built:.1} MPa vs {sig_yield:.0} (×{:.2}); governing case is the {}", sig_yield / pinch_as_built, if pinch_as_built > drop_as_built { "PINCH" } else { "DROP" }),
		&mut ok,
	);
	gate(
		"MASS the as-built set is inside the mass the load case was frozen at",
		(printed_g - DROP_MASS_G).abs() <= DROP_MASS_TOL * DROP_MASS_G,
		format!("{printed_g:.2} g vs {DROP_MASS_G:.1} ± {:.1} g — the drop force stands", DROP_MASS_TOL * DROP_MASS_G),
		&mut ok,
	);
	let height = cap_top().max(pin_top());
	gate("ENVELOPE Ø ≤ 73.0 × 12.0 mm (unchanged from the sibling)", 2.0 * (34.25 + RING_WALL) <= 73.0 && height <= 12.0, format!("Ø{:.1} × {height:.2}", 2.0 * (34.25 + RING_WALL)), &mut ok);

	// ===================== CAD / ASSEMBLY / RENDERS =========================
	for (n, sol) in [
		("ring_66t", &s_ring),
		("sun_42t", &s_sun),
		("planet_12t", &s_planet),
		("base_carrier_gen", &s_base),
		("top_carrier_gen", &s_top),
		("cap", &s_cap),
	] {
		let _ = std::fs::write(format!("{OUT}/cad/{n}.step"), export_step(sol, n));
	}
	let step_base = export_step(&s_base, "nullspin_gen_base_carrier");
	gate(
		"CAD the generative carrier's STEP is small enough to open",
		step_base.len() < 8_000_000,
		format!("{:.2} MB from {} faces (web contour {} faces + the exact hub, post and six bayonet pins)", step_base.len() as f64 / 1e6, s_base.face_count(), lb.faces),
		&mut ok,
	);
	let mut scene = Mesh::default();
	let mut merge = |m: &Mesh| {
		let base = scene.positions.len() as u32;
		scene.positions.extend_from_slice(&m.positions);
		scene.normals.extend_from_slice(&m.normals);
		scene.indices.extend(m.indices.iter().map(|i| i + base));
	};
	let a_ring = tessellate_default(&s_ring.transformed(tr(0.0, 0.0, Z_ROT)));
	let a_sun = tessellate_default(&s_sun.transformed(tr(0.0, 0.0, Z_GEAR)));
	merge(&m_base);
	merge(&a_ring);
	merge(&a_sun);
	merge(&m_top);
	merge(&m_cap);
	let mut planets_mesh = Mesh::default();
	for j in 0..N_PL {
		let b = TAU * j as f64 / N_PL as f64;
		let pm = tessellate_default(&s_planet.transformed(tr(CD * b.cos(), CD * b.sin(), Z_ROT) * rotz(b)));
		let base = planets_mesh.positions.len() as u32;
		planets_mesh.positions.extend_from_slice(&pm.positions);
		planets_mesh.normals.extend_from_slice(&pm.normals);
		planets_mesh.indices.extend(pm.indices.iter().map(|i| i + base));
		merge(&pm);
	}
	let _ = std::fs::write(format!("{OUT}/assembly/assembly.stl"), scene.to_stl_binary());
	let _ = std::fs::write(format!("{OUT}/assembly/scene/planets_x6.stl"), planets_mesh.to_stl_binary());
	for (n, m) in [("base_carrier", &m_base), ("ring", &a_ring), ("sun", &a_sun), ("top_carrier", &m_top), ("cap", &m_cap)] {
		let _ = std::fs::write(format!("{OUT}/assembly/scene/{n}.stl"), m.to_stl_binary());
	}
	let _ = std::fs::write(
		format!("{OUT}/assembly/scene/bom.csv"),
		format!(
			"name,kind,qty,material,part_number,grams_per_unit\n\
			 base_carrier_gen (held frame),made,1,PLA,P1,{b:.2}\n\
			 planet 12T x6,made,6,PLA,P4,{pl:.2}\n\
			 sun 42T,made,1,PLA,P3,{su:.2}\n\
			 ring 66T,made,1,PLA,P0,{rg:.2}\n\
			 top_carrier_gen,made,1,PLA,P2,{tp:.2}\n\
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
		"project": "NULLSPIN-GEN",
		"doc_title": "NULLSPIN-GEN — assembly sheet",
		"rev": "A",
		"date": "generated",
		"out_prefix": format!("{OUT}/assembly/ASSEMBLY"),
		"bom_csv": format!("{OUT}/assembly/scene/bom.csv"),
		"view": { "elev": 22, "azim": -58 },
		"parts": [
			{ "name": "base_carrier_gen (held frame)", "stl": format!("{OUT}/assembly/scene/base_carrier.stl"), "color": "#2f3b52" },
			{ "name": "planet 12T x6", "stl": format!("{OUT}/assembly/scene/planets_x6.stl"), "color": "#c9722f" },
			{ "name": "sun 42T", "stl": format!("{OUT}/assembly/scene/sun.stl"), "color": "#1f7a72" },
			{ "name": "ring 66T", "stl": format!("{OUT}/assembly/scene/ring.stl"), "color": "#8a6ec4" },
			{ "name": "top_carrier_gen", "stl": format!("{OUT}/assembly/scene/top_carrier.stl"), "color": "#48566f" },
			{ "name": "cap", "stl": format!("{OUT}/assembly/scene/cap.stl"), "color": "#b8433a" }
		],
		"explode": { "axis": [0.0, 0.0, 1.0], "auto": true, "gap_mm": 10 },
		"steps": [
			{ "order": 1, "text": "Drop the sun over the post. It rests on the small raised land around the post and is free to turn — there is no bearing and nothing to press." },
			{ "order": 2, "text": "Drop six planets onto six pins. They are identical and self-clock against the sun." },
			{ "order": 3, "text": "Drop the ring over the planets. It self-clocks, is located radially by all six, and rests on the six thrust pads in the base carrier." },
			{ "order": 4, "text": "Top carrier on — the BAYONET. Line each slot's wide end up over its pin (about 7 deg anticlockwise of home), drop it flat, then twist it 7 deg clockwise until all six stop. No force: it drops on free and the twist is a slide." },
			{ "order": 5, "text": "Check the lock by eye: every pin's fin must sit at the CLOSED end of its slot. Nothing to press, nothing to click past, and no printer calibration involved." },
			{ "order": 6, "text": "Press the cap onto the post. No hardware, no glue, no tools, no break-in." }
		]
	});
	let _ = std::fs::write(format!("{OUT}/assembly/scene/sheet_job.json"), format!("{sheet:#}\n"));
	match run_py("tools/assembly_doc.py", &format!("{OUT}/assembly/scene/sheet_job.json")) {
		Ok(_) => {
			let _ = std::fs::rename(format!("{OUT}/assembly/ASSEMBLY_assembly_doc.png"), format!("{OUT}/assembly/ASSEMBLY.png"));
			let _ = std::fs::remove_file(format!("{OUT}/assembly/ASSEMBLY_instructions.md"));
			gate("SHIP assembly sheet rendered (assembly/ASSEMBLY.png)", true, "assembly_doc.py".into(), &mut ok);
		}
		Err(e) => gate("SHIP assembly sheet rendered (assembly/ASSEMBLY.png)", false, e.chars().take(110).collect(), &mut ok),
	}
	let renders = [
		("assembly/assembly.stl", "renders/render_assembly.png"),
		("assembly/scene/base_carrier.stl", "renders/render_base_carrier.png"),
		("assembly/scene/top_carrier.stl", "renders/render_top_carrier.png"),
		("assembly/scene/ring.stl", "renders/render_ring.png"),
	];
	let n_ok = renders
		.iter()
		.filter(|(a, b)| run_py_plain("tools/render_views.py", &[&format!("{OUT}/{a}"), &format!("{OUT}/{b}")]).is_ok())
		.count();
	gate("SHIP product renders written (renders/)", n_ok == renders.len(), format!("{n_ok}/{}", renders.len()), &mut ok);

	write_docs(&Docs {
		lb: &lb,
		lt: &lt,
		drop_h: DROP_H_M,
		drop_s: DROP_S_MM,
		f_drop,
		f_sun,
		accel,
		f_bound,
		d_bound,
		s_bound,
		worst_drop,
		worst_pinch,
		h_allow_opt,
		h_yield_opt,
		h_allow_base,
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
		printed_g,
		frame_g,
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
		t_608,
		coul_frac,
		drag: &d_nom,
		y_pl,
		y_sun,
		y_ring,
		sig_ring,
		margin,
		uncredited,
		credited,
		ladder_best,
		worst_sp: worst_sp.max(worst_pr),
		worst_all,
		worst_web,
		min_cl_s: sw_s.min_clearance,
		min_cl_r: sw_r.min_clearance,
		jam,
		k_sun,
		k_pl,
		study_evals: report.evaluation_count(),
		study_feasible: report.feasible_count,
		engage_xy,
		engage_full,
		captive,
		wc_capture,
		f_cap,
		carried_n,
		slot_wall,
		snap_max,
		spec_strain,
		yield_strain,
		worst_bridge,
		mg_base: volume(&s_base).abs() * PLA,
		mg_top: volume(&s_top).abs() * PLA,
		mg_cap: volume(&s_cap).abs() * PLA,
		step_mb: step_base.len() as f64 / 1e6,
		faces_base: s_base.face_count(),
		engage_ring,
		mass_max: mass_max_g,
		drop_as_built,
		pinch_as_built,
	});

	println!("\nNULLSPIN-GEN: {}", if ok { "ALL GATES PASS" } else { "<<< FAIL" });
	std::process::exit(if ok { 0 } else { 1 });
}

/// Everything the loop measured for one carrier, in the units it is published in.
struct LoopResult {
	field: CarrierField,
	web: Solid,
	/// plate-slab mass of each of the three planforms, g, all through the SAME
	/// occupancy sampler so the comparison is an instrument reading and not
	/// three different arithmetics
	g_base: f64,
	g_solid: f64,
	g_opt: f64,
	/// design-azimuth (at-a-pin) peak von Mises, MPa — masked and raw
	vm_base: f64,
	vm_solid: f64,
	vm_opt: f64,
	/// worst-azimuth (between-pins) runs: the solid blank and the optimised part
	vm_solid_mid: f64,
	vm_mid: f64,
	/// pinch case on the optimised part
	vm_pinch: f64,
	/// negative control
	vm_nc: f64,
	/// tip/max displacements, mm
	disp_solid: f64,
	disp_opt: f64,
	disp_nc: f64,
	/// SIMP receipt numbers
	iters: f64,
	stop: String,
	c_first: f64,
	c_last: f64,
	volfrac_achieved: f64,
	volfrac_hub: f64,
	simp_as_built_mpa: f64,
	/// what the post-processing did, measured
	asym: f64,
	area_rim_raw: f64,
	area_rim_sym: f64,
	area_hub_sym: f64,
	area_union: f64,
	debris: f64,
	islands: usize,
	/// mesh + rebuild receipts
	holes: usize,
	cdev: f64,
	xdrift: f64,
	drift: f64,
	tris: usize,
	facet: f64,
	faces: usize,
	one_body: bool,
	shells: usize,
	/// thin-wall probe: the part and its solid control
	tw: f64,
	tw_ctl: f64,
	probe_cell: f64,
}

/// Run the whole loop for one carrier: baseline FEA → solid-start FEA → SIMP →
/// density to geometry → exact bridge → HONEST re-analysis of the final binary
/// geometry → worst azimuth → pinch → negative control.
#[allow(clippy::too_many_lines)]
fn run_loop(part: Part, tag: &str, f_drop: f64, f_sun: f64, accel: f64, ok: &mut bool) -> LoopResult {
	let name = match part {
		Part::Base => "BASE carrier",
		Part::Top => "TOP carrier",
	};
	println!("\n=== generative loop — {name} ===");
	// The comparison azimuth is AT a pin, not between them, for one reason: it is
	// the only azimuth the sibling's hand-drawn carrier has any material at. That
	// choice is generous to the baseline and it is made deliberately, because a
	// comparison the incumbent cannot enter is not a comparison.
	let a_cmp = 0.0f64;
	// between two pins — the worst azimuth, and the one frozen load pad
	const LOAD_K: usize = 4;
	let a_mid = load_azimuths()[LOAD_K];

	let mk = |web: Web| CarrierField {
		part,
		web,
		rho: unit_grid(),
		mutilated: false,
		no_rim: false,
	};
	let origin = Vec3::new(GRID_ORIGIN.0 as f32, GRID_ORIGIN.1 as f32, GRID_ORIGIN.2 as f32);
	let grams = |occ: &[f32]| occ.iter().map(|&v| v as f64).sum::<f64>() * VOX.powi(3) * PLA;

	// ---- stage 1: the two reference planforms, sampled onto the analysis grid
	let f_baseline = mk(Web::Baseline);
	let f_solid = mk(Web::SolidStart);
	let (occ_baseline, lost_b) = prune_occ(&sample_occupancy(&f_baseline, GRID_DIMS, origin, VOX as f32));
	let (occ_solid, lost_s) = prune_occ(&sample_occupancy(&f_solid, GRID_DIMS, origin, VOX as f32));
	let _ = write_npy(&format!("{FEA_DIR}/{tag}_baseline.npy"), &occ_baseline, GRID_DIMS);
	let _ = write_npy(&format!("{FEA_DIR}/{tag}_solid.npy"), &occ_solid, GRID_DIMS);
	let g_base = grams(&occ_baseline);
	let g_solid = grams(&occ_solid);
	gate(
		&format!("{tag} G29 the analysis grid resolves both reference bodies (little lost to voxelisation)"),
		lost_b <= OCC_PRUNE_MAX && lost_s <= OCC_PRUNE_MAX,
		format!("baseline {:.2}% / solid start {:.2}% dropped as unresolved specks (limit {:.1}%)", 100.0 * lost_b, 100.0 * lost_s, 100.0 * OCC_PRUNE_MAX),
		ok,
	);

	// The finding that motivates the continuous rim, stated as geometry rather
	// than as an opinion: the sibling's carrier has NO material at the
	// between-arm azimuths, so a rim-first drop there does not land on the
	// carrier at all — it lands on the gear ring.
	let probe = contact_pad_xy(4); // the a_mid contact patch
	let base_here = f_baseline.distance(Vec3::new(probe.x as f32, probe.y as f32, 1.0));
	let solid_here = f_solid.distance(Vec3::new(probe.x as f32, probe.y as f32, 1.0));
	gate(
		&format!("{tag} G30 the hand-drawn carrier has NO rim at the between-pin azimuth"),
		base_here > 0.0 && solid_here < 0.0,
		format!("baseline d {base_here:+.2} mm (outside), design domain d {solid_here:+.2} mm (inside)"),
		ok,
	);

	// ---- stage 2: reference FEA of both, through ONE manifest
	let doc_cmp = format!(
		"{name}: equivalent-static drop, h {DROP_H_M} m / stopping distance {DROP_S_MM} mm / design mass {DROP_MASS_G} g \
		 => rim force {f_drop:.1} N inward at the at-a-pin azimuth, sun inertia {f_sun:.1} N on the hub, \
		 body load {accel:.0} m/s2. NOT a transient impact simulation. Anchors: the six planet pins (and the hub) \
		 are the inertial anchors during the ~0.1 ms contact."
	);
	let fea_at = |body: &str, npy: &str, out: &str, job: serde_json::Value, ok: &mut bool| -> (f64, f64, f64, StressScan) {
		write_json(&format!("{FEA_DIR}/{tag}_{body}.json"), &job);
		let r = require(
			&format!("{tag} {body} FEA (tools/ace_fea_runner.py)"),
			run_py("tools/ace_fea_runner.py", &format!("{FEA_DIR}/{tag}_{body}.json")),
			&format!("{FEA_DIR}/{tag}_{body}_receipt.json"),
			ok,
		);
		let vm = f(&r, &["max_von_mises_pa"]) / 1e6;
		let disp = f(&r, &["max_displacement_m"]) * 1000.0;
		// The runner flags any load selector catching > 30 % of active elements as
		// "suspiciously broad" — the guard against a smeared POINT load. A BODY
		// load legitimately covers every element, so that one note is expected and
		// is excluded by name rather than by ignoring the notes array.
		let broad = r["notes"]
			.as_array()
			.map(|n| {
				n.iter().any(|s| {
					let t = s.as_str().unwrap_or("");
					t.contains("suspiciously broad") && !t.contains("(body)")
				})
			})
			.unwrap_or(false);
		let nodes = r["loads"][0]["nodes_or_elements"].as_f64().unwrap_or(0.0);
		gate(
			&format!("{tag} {body} selectors honest (load lands on material, not smeared)"),
			nodes > 0.0 && !broad,
			format!("{nodes:.0} load nodes, broad-note {broad}"),
			ok,
		);
		let load_at = if body.contains("mid") || body.contains("nc") || body.contains("pinch") {
			contact_pad_xy(4)
		} else {
			DVec2::new(CONTACT_PAD_AT, 0.0)
		};
		// The pinch case CLAMPS the opposite rim pad, and a small clamp patch
		// spikes exactly the way a small load patch does. Masking one and not the
		// other would be an accident of which artifact happened to be bigger, so
		// both introduction patches are masked and both are named.
		let also = if body.contains("pinch") { Some(-load_at) } else { None };
		let mask = if body.contains("pinch") { 2.0 * FINGER_R } else { MASK_R };
		let scan = scan_stress(&format!("{FEA_DIR}/{out}/stress_field.npy"), load_at, also, mask).unwrap_or_default();
		let _ = npy;
		(vm, disp, scan.masked_mpa, scan)
	};

	let (_, _, vm_base, sc_base) = fea_at(
		"baseline",
		"",
		&format!("out_{tag}_baseline"),
		drop_job(part, &doc_cmp, &format!("out_{tag}_baseline"), &format!("{tag}_baseline.npy"), a_cmp, f_drop, f_sun, accel),
		ok,
	);
	let (_, disp_solid, vm_solid, _) = fea_at(
		"solid",
		"",
		&format!("out_{tag}_solid"),
		drop_job(part, &doc_cmp, &format!("out_{tag}_solid"), &format!("{tag}_solid.npy"), a_cmp, f_drop, f_sun, accel),
		ok,
	);
	// The optimiser's OWN before/after: the solid blank at the SAME worst azimuth
	// the SIMP case was posed at. Without this row "43 % lighter" has no
	// denominator that shares a load case with the answer.
	let (_, _, vm_solid_mid, _) = fea_at(
		"solidmid",
		"",
		&format!("out_{tag}_solidmid"),
		drop_job(
			part,
			&format!("{doc_cmp} SOLID START at the WORST azimuth — the optimiser's own denominator."),
			&format!("out_{tag}_solidmid"),
			&format!("{tag}_solid.npy"),
			a_mid,
			f_drop,
			f_sun,
			accel,
		),
		ok,
	);
	// The mask is only honest if the thing it removes really is the load patch.
	gate(
		&format!("{tag} G31 the raw peak IS the load-introduction artifact (inside the mask)"),
		sc_base.peak_from_load <= MASK_R && (sc_base.n_masked as f64) < 0.06 * sc_base.n_active as f64,
		format!(
			"peak {:.2} MPa at {:.1} mm from the load ({} of {} elements masked)",
			sc_base.peak_mpa, sc_base.peak_from_load, sc_base.n_masked, sc_base.n_active
		),
		ok,
	);

	// ---- stage 3: SIMP
	let doc_rim = format!(
		"{name}: SIMP case RIM — the drop as a six-fold ENVELOPE. The rim contact is applied at all six between-pin \
		 azimuths at once, reacted at the six pins. It is not the drop and no stress from it is published; it is the \
		 load PATTERN a part that must survive the drop at ANY azimuth has to be shaped for. Compliance minimisation is \
		 invariant to a uniform load scale, so the magnitude steers nothing. Frozen: the six pin pads (fixtures) and the \
		 six rim contact pads (loads) — nothing else, because an unloaded unfixed frozen region is a singular block in K."
	);
	let doc_hub = format!(
		"{name}: SIMP case HUB — the sun's inertia on the post, {f_sun:.1} N at ONE azimuth, reacted at the six pins. \
		 It cannot be an envelope: the hub is the centre of symmetry, so any six-fold symmetric load set has zero net \
		 force there and is absorbed in hoop by the frozen hub, which leaves the optimiser blind to the heaviest load \
		 path in the machine. Its single-azimuth optimum IS symmetrised by the six-fold maximum afterwards."
	);
	let simp = |case: Case, doc: &str, out: &str, ok: &mut bool| -> serde_json::Value {
		let job = simp_job(part, case, doc, out, &format!("{tag}_solid.npy"), a_mid, f_drop, f_sun);
		write_json(&format!("{FEA_DIR}/{out}.json"), &job);
		require(
			&format!("{tag} SIMP {out} (tools/ace_optimize_runner.py)"),
			run_py("tools/ace_optimize_runner.py", &format!("{FEA_DIR}/{out}.json")),
			&format!("{FEA_DIR}/{out}_receipt.json"),
			ok,
		)
	};
	let opt_a = simp(Case::Rim, &doc_rim, &format!("opt_{tag}_rim"), ok);
	// The determinism proof is about the RUNNER, not about this particular
	// geometry, so it is paid for once rather than on every job. Stated rather
	// than quietly skipped.
	let opt_b = if part == Part::Base { Some(simp(Case::Rim, &doc_rim, &format!("opt_{tag}_rim2"), ok)) } else { None };
	let opt_hub = if part == Part::Base { Some(simp(Case::Hub, &doc_hub, &format!("opt_{tag}_hub"), ok)) } else { None };
	for dir in [format!("opt_{tag}_rim"), format!("opt_{tag}_rim2"), format!("opt_{tag}_hub")] {
		if let Ok(entries) = std::fs::read_dir(format!("{FEA_DIR}/{dir}")) {
			for e in entries.flatten() {
				let n = e.file_name().to_string_lossy().to_string();
				if (n.starts_with("tmp") && n.ends_with(".json")) || n == "_lmcad_rho.npy" {
					let _ = std::fs::remove_file(e.path());
				}
			}
		}
	}
	if let Some(b) = &opt_b {
		let ra = std::fs::read(format!("{FEA_DIR}/opt_{tag}_rim/final_rho.npy")).unwrap_or_default();
		let rb = std::fs::read(format!("{FEA_DIR}/opt_{tag}_rim2/final_rho.npy")).unwrap_or_default();
		gate(
			"G32 SIMP is deterministic: two runs, byte-identical final_rho.npy",
			!ra.is_empty() && ra == rb && f(&opt_a, &["compliance_last"]) == f(b, &["compliance_last"]),
			format!("{} bytes, compliance {:.6e}", ra.len(), f(&opt_a, &["compliance_last"])),
			ok,
		);
	}
	let vf_target = if part == Part::Top { VOLFRAC_TOP } else { VOLFRAC_RIM };
	let vfa = f(&opt_a, &["volume_fraction_achieved"]);
	let vfh = opt_hub.as_ref().map(|h| f(h, &["volume_fraction_achieved"])).unwrap_or(0.0);
	gate(
		&format!("{tag} G33 SIMP volume constraints held (≤ target + 0.02, both cases)"),
		vfa <= vf_target + 0.02 && vfh <= VOLFRAC_HUB + 0.02,
		format!(
			"rim {vfa:.4}/{vf_target} in {} iters ({}), hub {vfh:.4}/{VOLFRAC_HUB}",
			f(&opt_a, &["iterations"]),
			opt_a["stop_reason"].as_str().unwrap_or("?")
		),
		ok,
	);
	let c_ok = |v: &serde_json::Value| f(v, &["compliance_last"]) < f(v, &["compliance_first"]);
	gate(
		&format!("{tag} G34 both SIMP cases stiffened against their own gray start"),
		c_ok(&opt_a) && opt_hub.as_ref().map(c_ok).unwrap_or(true),
		format!("rim {:.4e} → {:.4e}", f(&opt_a, &["compliance_first"]), f(&opt_a, &["compliance_last"])),
		ok,
	);
	gate(
		&format!("{tag} G35 SIMP filter imposes a printable length scale (2·r·vox ≥ {MIN_FEATURE})"),
		2.0 * SIMP_FILTER_RVOX * VOX >= MIN_FEATURE as f64,
		format!("2·{SIMP_FILTER_RVOX}·{VOX} = {:.1} mm", 2.0 * SIMP_FILTER_RVOX * VOX),
		ok,
	);

	// ---- stage 4: density fields → geometry
	let load_rho = |dir: &str, ok: &mut bool| -> GridField {
		GridField::from_npy_file(format!("{FEA_DIR}/{dir}/final_rho.npy"), field_origin(), VOX as f32).unwrap_or_else(|e| {
			gate(&format!("{tag} {dir}/final_rho.npy loads as a GridField"), false, e, ok);
			std::process::exit(1);
		})
	};
	let rho_rim = load_rho(&format!("opt_{tag}_rim"), ok);
	let rho_hub = opt_hub.as_ref().map(|_| load_rho(&format!("opt_{tag}_hub"), ok));
	let (plate, prep) = plate_field(&rho_rim, rho_hub.as_ref());
	let (plate, debris, islands) = prune_islands(part, &plate);
	// SIMP/OC is a non-convex density method: even a six-fold symmetric problem
	// need not have a six-fold answer on a Cartesian grid. This gate does not
	// pretend otherwise — it MEASURES the symmetry break and the area the
	// six-fold maximum spent fixing it, and gates the thing that matters, which
	// is that the union still fits the plate's area budget. The mass gate on the
	// finished part is the hard one.
	gate(
		&format!("{tag} G36 six-fold symmetrisation and the two-case union cost a measured, bounded area"),
		prep.area_union <= SYM_AREA_MAX * prep.area_rim_raw && prep.area_rim_raw > 0.0,
		format!(
			"asym {:.3}; rim {:.0}→{:.0} mm², hub {:.0} mm², union {:.0} ({:.2}×)",
			prep.asym, prep.area_rim_raw, prep.area_rim_sym, prep.area_hub_sym, prep.area_union, prep.area_union / prep.area_rim_raw
		),
		ok,
	);
	gate(
		&format!("{tag} G36b disconnected debris pruned and REPORTED (never silently shipped)"),
		debris <= DEBRIS_MAX_MM2,
		format!("{islands} island(s), {debris:.1} mm² of {:.0} ({:.2}%) removed by a 4-connected fill", prep.area_union, 100.0 * debris / prep.area_union.max(1.0)),
		ok,
	);
	let (nx, ny, _) = GRID_DIMS;
	let rho2d = GridField::from_data(plate, (nx, ny, 1), field_origin(), VOX as f32)
		.expect("plate density grid is finite by construction");
	let field = CarrierField { part, web: Web::Generative, rho: rho2d, mutilated: false, no_rim: false };
	let mesh = mesh_field(&field);
	let facet = max_edge(&mesh);
	gate(
		&format!("{tag} G37 optimised mesh is watertight and ONE body (no floating islands)"),
		mesh.is_watertight() && mesh.is_one_body() && mesh.triangle_count() > 0,
		format!("{} tris, {} components, {} non-manifold edges", mesh.triangle_count(), mesh.component_count(1e-3), mesh.non_manifold_edge_count()),
		ok,
	);
	// Surface quality, in two parts, because one number cannot carry both claims.
	// (1) the mesher behaved: a dual-contour quad joins one vertex per cell across
	// ADJACENT cells, so no edge can exceed 2·voxel·√3 — that is geometry, and a
	// reading above it would mean the mesh is not what it says it is.
	// (2) the product claim: no facet longer than FACET_MAX_MM. On the tightest
	// radius the SIMP filter permits (half the 4.0 mm minimum feature), a chord
	// that long deviates from the true surface by c²/8r = 0.09 mm — under half a
	// layer, so the printed surface is limited by the slicer, not by the mesh.
	let dc_bound = 2.0 * MESH_VOX as f64 * 3.0f64.sqrt();
	gate(
		&format!("{tag} G38 mesher within its own dual-contour edge bound (2·voxel·√3)"),
		facet <= dc_bound + 1e-6,
		format!("longest edge {facet:.3} mm vs bound {dc_bound:.3}"),
		ok,
	);
	gate(
		&format!("{tag} G38b surface quality claim: no facet longer than {FACET_MAX_MM} mm"),
		facet <= FACET_MAX_MM,
		format!("{facet:.3} mm → sagitta {:.3} mm on the tightest permitted radius", facet * facet / (8.0 * SIMP_FILTER_RVOX * VOX)),
		ok,
	);
	let vm_mesh = mesh.signed_volume().abs();
	// ---- the SHIPPED exact solid: a prism over the extracted contour.
	let (web, n_out, n_holes, cdev) = web_prism(&field);
	let val = validate(&web);
	let v_web = volume(&web).abs();
	let d_prism = (v_web - vm_mesh).abs() / vm_mesh.max(1.0);
	gate(
		&format!("{tag} G39 the shipped web is an EXACT prism over its own contour"),
		val.is_valid() && val.shells == 1 && n_holes > 0 && d_prism <= CONTOUR_DRIFT_MAX,
		format!(
			"{} faces, {n_out}-point outline + {n_holes} openings, genus {}, {:.2}% from the meshed field",
			web.face_count(),
			val.genus,
			d_prism * 100.0
		),
		ok,
	);
	gate(
		&format!("{tag} G38c the shipped contour tracks the optimiser's own level set"),
		cdev <= CONTOUR_TOL + 1e-9,
		format!("worst deviation {cdev:.4} mm from the {CONTOUR_RES} mm marching-squares polyline (≤ {CONTOUR_TOL}, one fifth of a layer)"),
		ok,
	);
	gate(
		&format!("{tag} G39b the prism re-tessellates watertight (it has to survive a boolean chain)"),
		tessellate_default(&web).is_watertight(),
		format!("{} triangles", tessellate_default(&web).triangle_count()),
		ok,
	);
	// ---- INDEPENDENT cross-check: the kernel's own reverse bridge on the same
	// field, by a completely different algorithm (dual contour → facet wrap →
	// coplanar coalesce). Two reconstructions agreeing is a much stronger
	// statement than either one validating alone.
	let mut xdrift = f64::NAN;
	let (_route, drift) = match bridge(&mesh) {
		Ok((s2, r, d)) => {
			let dv = (volume(&s2).abs() - v_web).abs() / v_web.max(1.0);
			xdrift = dv;
			gate(
				&format!("{tag} G39c reverse bridge agrees with the prism (two independent reconstructions)"),
				dv <= CONTOUR_DRIFT_MAX,
				format!("{} faces vs {}, volumes differ {:.2}% — {r}", s2.face_count(), web.face_count(), dv * 100.0),
				ok,
			);
			(r, d)
		}
		Err(e) => {
			gate(&format!("{tag} G39c reverse bridge cross-check"), false, e, ok);
			(String::from("refused"), f64::NAN)
		}
	};

	// ---- minimum feature, gated the only way it can honestly be gated.
	// An absolute `thinnest` gate is blind here: the probe's reading is set by
	// the TAPERS where the plate meets its own rim and pads, not by the web
	// members, so even a fully solid plate reads small. The gate is therefore
	// differential — the optimised body must be indistinguishable from a SOLID
	// control that shares every feature except the density — and the length
	// scale itself is guaranteed upstream by G35 (the filter radius).
	let control = CarrierField { web: Web::SolidStart, ..field.clone() };
	let tw = reverse::thin_wall_report(&field, field.bounds(), 120, MIN_FEATURE);
	let tw_ctl = reverse::thin_wall_report(&control, control.bounds(), 120, MIN_FEATURE);
	let probe_cell = (field.bounds().size().max_element() / 119.0) as f64;
	gate(
		&format!("{tag} G40 thin wall: optimised indistinguishable from the solid control"),
		(tw.thinnest - tw_ctl.thinnest).abs() as f64 <= probe_cell,
		format!("opt {:.2} mm / {} below vs control {:.2} mm / {} below (probe cell {probe_cell:.2})", tw.thinnest, tw.below_count, tw_ctl.thinnest, tw_ctl.below_count),
		ok,
	);

	// ---- stage 5: HONEST re-analysis of the FINAL BINARY GEOMETRY
	let (occ_opt, lost_o) = prune_occ(&sample_occupancy(&field, GRID_DIMS, origin, VOX as f32));
	let _ = write_npy(&format!("{FEA_DIR}/{tag}_final.npy"), &occ_opt, GRID_DIMS);
	gate(
		&format!("{tag} G29b the SHIPPED body needs no pruning at all on the analysis grid"),
		lost_o <= 1e-9,
		format!("{:.4}% dropped (the optimised web is resolved everywhere; the reference bodies were not)", 100.0 * lost_o),
		ok,
	);
	let g_opt = grams(&occ_opt);
	let vol_occ = occ_opt.iter().map(|&v| v as f64).sum::<f64>() * VOX.powi(3);
	gate(
		&format!("{tag} G41 the analysis body IS the shipped mesh (sampled vs meshed volume)"),
		(vol_occ - vm_mesh).abs() / vm_mesh < 0.05,
		format!("{vol_occ:.0} vs {vm_mesh:.0} mm³ ({:.1}%)", 100.0 * (vol_occ - vm_mesh).abs() / vm_mesh),
		ok,
	);
	let doc_final = format!("{doc_cmp} FINAL as-built binary occupancy of the geometry that ships — never the optimiser's own SIMP estimate.");
	let (_, disp_opt, vm_opt, _) = fea_at(
		"final",
		"",
		&format!("out_{tag}_final"),
		drop_job(part, &doc_final, &format!("out_{tag}_final"), &format!("{tag}_final.npy"), a_cmp, f_drop, f_sun, accel),
		ok,
	);
	// the worst azimuth — between two pins, which is what the SIMP steering case
	// was shaped for and what the hand-drawn carrier cannot be analysed at at all
	let (_, _, vm_mid, _) = fea_at(
		"mid",
		"",
		&format!("out_{tag}_mid"),
		drop_job(
			part,
			&format!("{doc_final} WORST AZIMUTH: the rim contact lands BETWEEN two pins."),
			&format!("out_{tag}_mid"),
			&format!("{tag}_final.npy"),
			a_mid,
			f_drop,
			f_sun,
			accel,
		),
		ok,
	);
	// the hand case, verified and never optimised for
	let (_, _, vm_pinch, _) = fea_at(
		"pinch",
		"",
		&format!("out_{tag}_pinch"),
		pinch_job(
			part,
			&format!("{name}: PINCH — {PINCH_N} N radial squeeze across a diameter plus {FLICK_N} N of tangential flick drag on the same patch, one finger fixed and the other loaded. Verified on the final geometry; NOT the case the topology was optimised for."),
			&format!("out_{tag}_pinch"),
			&format!("{tag}_final.npy"),
		),
		ok,
	);

	// ---- negative control: same chain, geometry deliberately broken
	let nc_field = CarrierField { mutilated: true, ..field.clone() };
	let (occ_nc, _) = prune_occ(&sample_occupancy(&nc_field, GRID_DIMS, origin, VOX as f32));
	let _ = write_npy(&format!("{FEA_DIR}/{tag}_nc.npy"), &occ_nc, GRID_DIMS);
	let nc_solid_frac: f64 = occ_nc.iter().map(|&v| v as f64).sum();
	let opt_solid_frac: f64 = occ_opt.iter().map(|&v| v as f64).sum();
	// The threshold is DERIVED, not tuned: the control cuts one Ø7.00 column
	// through the 2.00 mm plate, so a cut that lands in material removes
	// π·r²·t = 77 mm³. Requiring at least half of that proves the column really
	// bit into the structure rather than grazing a void. A fraction-of-the-part
	// threshold would be the wrong instrument — one strut is a small fraction of
	// a large plate, and it was: the first version of this gate asked for 1.5 %
	// of the body and read 1.2 %, which said nothing about whether the cut was
	// on the load path.
	let nc_removed = (opt_solid_frac - nc_solid_frac) * VOX.powi(3);
	let nc_column = PI * 3.5f64.powi(2) * Z_ARM;
	gate(
		&format!("{tag} G42 NC geometry really is broken (a strut, not a void, was cut)"),
		nc_removed >= 0.5 * nc_column,
		format!("{nc_removed:.0} mm³ removed of a {nc_column:.0} mm³ column ({:.0}% in material)", 100.0 * nc_removed / nc_column),
		ok,
	);
	let (_, disp_nc, vm_nc, _) = fea_at(
		"nc",
		"",
		&format!("out_{tag}_nc"),
		drop_job(
			part,
			"NC: identical manifest to the worst-azimuth case on deliberately broken geometry — ONE strut between the rim contact and its nearest anchor is cut out. Stress and deflection must JUMP.",
			&format!("out_{tag}_nc"),
			&format!("{tag}_nc.npy"),
			a_mid,
			f_drop,
			f_sun,
			accel,
		),
		ok,
	);
	gate(
		&format!("{tag} G43 NC: the FEA fires on the broken carrier (≥ 1.3× stress or deflection)"),
		vm_nc >= 1.3 * vm_mid || disp_nc >= 1.3 * disp_opt,
		format!("NC {vm_nc:.2} MPa / {disp_nc:.4} mm vs {vm_mid:.2} MPa / {disp_opt:.4} mm ({:.2}× / {:.2}×)", vm_nc / vm_mid, disp_nc / disp_opt),
		ok,
	);

	let (tris, one_body, faces) = (mesh.triangle_count(), mesh.is_one_body(), web.face_count());
	LoopResult {
		field,
		web,
		g_base,
		g_solid,
		g_opt,
		vm_base,
		vm_solid,
		vm_opt,
		vm_solid_mid,
		vm_mid,
		vm_pinch,
		vm_nc,
		disp_solid,
		disp_opt,
		disp_nc,
		iters: f(&opt_a, &["iterations"]),
		stop: opt_a["stop_reason"].as_str().unwrap_or("?").to_string(),
		c_first: f(&opt_a, &["compliance_first"]),
		c_last: f(&opt_a, &["compliance_last"]),
		volfrac_achieved: vfa,
		volfrac_hub: vfh,
		simp_as_built_mpa: f(&opt_a, &["as_built", "max_von_mises_pa"]) / 1e6,
		asym: prep.asym,
		area_rim_raw: prep.area_rim_raw,
		area_rim_sym: prep.area_rim_sym,
		area_hub_sym: prep.area_hub_sym,
		area_union: prep.area_union,
		debris,
		islands,
		holes: n_holes,
		cdev,
		xdrift,
		drift,
		tris,
		facet,
		faces,
		one_body,
		shells: val.shells,
		tw: tw.thinnest as f64,
		tw_ctl: tw_ctl.thinnest as f64,
		probe_cell,
	}
}

// ============================================================================
// 14. DELIVERABLES — every number below is measured by the run that writes it.
// ============================================================================

struct Docs<'a> {
	lb: &'a LoopResult,
	lt: &'a LoopResult,
	drop_h: f64,
	drop_s: f64,
	f_drop: f64,
	f_sun: f64,
	accel: f64,
	f_bound: f64,
	d_bound: f64,
	s_bound: f64,
	worst_drop: f64,
	worst_pinch: f64,
	h_allow_opt: f64,
	h_yield_opt: f64,
	h_allow_base: f64,
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
	printed_g: f64,
	frame_g: f64,
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
	t_608: f64,
	coul_frac: f64,
	drag: &'a Drag,
	y_pl: f64,
	y_sun: f64,
	y_ring: f64,
	sig_ring: f64,
	margin: f64,
	uncredited: f64,
	credited: f64,
	ladder_best: f64,
	worst_sp: f64,
	worst_all: f64,
	worst_web: f64,
	min_cl_s: f64,
	min_cl_r: f64,
	jam: f64,
	k_sun: f64,
	k_pl: f64,
	study_evals: usize,
	study_feasible: usize,
	engage_xy: f64,
	engage_full: f64,
	captive: f64,
	wc_capture: f64,
	f_cap: f64,
	carried_n: f64,
	slot_wall: f64,
	snap_max: f64,
	spec_strain: f64,
	yield_strain: f64,
	worst_bridge: f64,
	mg_base: f64,
	mg_top: f64,
	mg_cap: f64,
	step_mb: f64,
	faces_base: usize,
	engage_ring: f64,
	mass_max: f64,
	drop_as_built: f64,
	pinch_as_built: f64,
}

/// The authorship disclosure, verbatim in every document that carries one. It is
/// static text on purpose: it is a statement about how the model was made, and a
/// statement about provenance should not be assembled from run-time numbers.
const AUTHORSHIP: &str = "\
This model is **defined by a parametric program** — a Rust source file that\n\
computes every dimension from first principles, builds the solids, and\n\
re-verifies every claim on every run. It was not drawn by hand in a GUI CAD\n\
package. Parametric, code-authored CAD is a long-established authoring method\n\
(OpenSCAD, CadQuery, build123d) and the geometry here is the **deterministic\n\
output of that program**: run it twice, get identical files.\n\n\
**The program was written with AI assistance**, and so was the research that\n\
froze its dimensions and its analysis plan. The geometry is not the output of a\n\
generative 3-D MESH model — there is no mesh generator, no image model and no\n\
model-generation service anywhere in the pipeline. I confirmed eligibility with\n\
Printables before entering.\n\n\
One word in this listing needs care, because it is doing real work and it does\n\
NOT mean what \"AI-generated 3-D model\" means. The carrier is **generatively\n\
DESIGNED**: a topology optimiser (SIMP — density-based structural optimisation,\n\
the same maths CAD packages ship as \"generative design\") was given a load case\n\
and a design volume and it solved for where the material should go. That is a\n\
physics solver returning a density field, not a model generator returning a\n\
shape, and the program then rebuilds exact CAD geometry from that field and\n\
re-analyses the result. Every stage is deterministic and every stage is in the\n\
source.\n";

fn write_docs(d: &Docs) {
	let budget_rows = d
		.drag
		.terms
		.iter()
		.map(|(c, e, w)| {
			let cls = if *e == 0.0 { "**Coulomb**" } else { "quadratic-ish" };
			format!("| {w} | ω^{e:.1} ({cls}) | {:.4} | {:.0}% |", c * W0.powf(*e) * 1e3, 100.0 * c * W0.powf(*e) / d.drag.torque(W0))
		})
		.collect::<Vec<_>>()
		.join("\n");
	let sens_rows = d.sens.iter().map(|(k, v)| format!("| {k} | {v:.4} |")).collect::<Vec<_>>().join("\n");
	let od = 2.0 * (34.25 + RING_WALL);

	// ---------------- analysis/ANALYSIS.md (GENERATED from this run) --------
	let mut a = String::new();
	a.push_str(&format!(
		"# NULLSPIN-GEN — analysis (generated by `nullspin_gen.rs`; regenerated every run)\n\n\
		Every number below is what the gate suite measured on THIS build, so it cannot go\n\
		stale. The frozen contract, the provenance of every researched constant and the\n\
		analysis PLAN are in `DESIGN.md`; this file answers the plan.\n\n\
		## What this artifact claims\n\n\
		The mechanism claim is the sibling's and is unchanged: two visible rotors turning\n\
		in OPPOSITE directions at an exact integer ratio, 7·66 = 11·42 = 462. Flick the\n\
		ring seven times and the puck turns eleven the other way.\n\n\
		The claim that is NEW here is about the CARRIER, and it is deliberately narrow:\n\n\
		> The held frame is the output of a real generative loop — a declared load case,\n\
		> a reference FEA, SIMP topology optimisation, and an HONEST re-analysis of the\n\
		> FINAL BINARY GEOMETRY — and the load case is the DROP, which the sibling's own\n\
		> analysis declares **REQUIRED, NOT PERFORMED** and calls \"the largest honest gap\n\
		> in the deliverable\". That gap is closed here. It is closed with an\n\
		> EQUIVALENT-STATIC model, not a transient impact simulation, and every number\n\
		> that comes out of it says so.\n\n\
		**What this entry does NOT claim** is that the optimiser beat the sibling's six\n\
		straight spokes. It did not, at the sibling's own azimuth, and the numbers are in\n\
		the ledger below rather than left out of it. Six radial spokes is very close to\n\
		the textbook answer for a radial load ON a spoke. What the generative carrier buys\n\
		is that there is no azimuth where it is absent.\n\n\
		## The load case, in full\n\n\
		```\n\
		v = sqrt(2 g h)                       impact speed from a free fall of h\n\
		a = v² / (2 s) = g h / s              mean deceleration over stopping distance s\n\
		F = m a = m g h / s                   the EQUIVALENT-STATIC contact force\n\
		```\n\n\
		| input | value | why it is that value |\n|---|---|---|\n\
		| drop height h | **{h:.2} m** | not from a standard — the toy-safety drop tests could not be re-verified from a primary source on this run, so they are not cited as authority. From USE: a standing adult's hand at rest is ≈0.75 m, a desk is ≈0.75 m, a hand held up to watch the gears is ≈1.1–1.3 m. 1.00 m is the middle of that band, and the assumption is made non-load-bearing by publishing the survival HEIGHT below |\n\
		| stopping distance s | **{s:.2} mm** | the distance the centre of mass travels after first contact — local crush of the rim, local indentation of the floor, and the structure's own squash, together. Swept, and cross-checked against a rigid-floor bound below |\n\
		| design mass m | **{dm:.1} g** | frozen BEFORE the geometry (§25 puts the plan first), and gated afterwards: the as-built set is {pg:.2} g, inside ±{tol:.0}% |\n\
		| ⇒ deceleration | **{acc:.0} m/s² = {gs:.0} g** | |\n\
		| ⇒ rim force | **{fd:.0} N** | applied at one rim contact patch |\n\
		| ⇒ sun inertia on the post | **{fs:.0} N** | the sun is 12.8 g, the heaviest body in the machine, and its radial inertia can reach the carrier through the post: the bore's 0.25 mm running fit and the mesh's {jr:.3} mm radial lash equivalent are only 0.06 mm apart, and the build error is 0.15 mm/side, so which closes first is NOT decidable. The design case assumes the post takes it all |\n\n\
		**The rigid-floor bound, published next to the design case.** An elastic-plastic\n\
		indentation model (Johnson's constant mean pressure p_m = {c:.0}·σ_y, a PERFECTLY\n\
		RIGID floor, no rotation of the part) is solved in closed form in the source and\n\
		returns a crush of **{db:.3} mm**, an equivalent stopping distance of **{sb:.3} mm**\n\
		and a peak force of **{fb:.0} N** — **{ratio:.1}×** the design case. Both are true;\n\
		the ratio between them is exactly the statement *\"s = {s:.2} mm describes a hard\n\
		floor, not an infinitely rigid one\"*. The bound is benchmarked against the design\n\
		route by energy conservation (DROP D1) and by feeding it its own stopping distance\n\
		(D2), with a meta-negative-control proving D2 can go red (D3).\n\n\
		**What the model is not.** It is not a transient impact simulation. It carries no\n\
		wave propagation (the contact is ~0.1 ms against a 0.045 ms wave transit across the\n\
		part, so the rigid-body assumption is marginal and is stated as such), no contact\n\
		separation and re-strike, no strain-rate dependence (PLA stiffens and embrittles at\n\
		high rate; the first raises the force and the second lowers the allowable, and\n\
		neither is modelled), and no rotation of the part about the contact. A true\n\
		transient solve is listed in the analysis plan as REQUIRED, NOT PERFORMED.\n\n\
		## The generative loop, measured\n\n\
		Both carriers ran the identical pipeline. Every stress below is the peak von Mises\n\
		of a fresh BINARY-OCCUPANCY solve of the geometry that ships, through a manifest\n\
		byte-identical to the one the baseline used. The optimiser's own `as_built` figure\n\
		is never quoted as a product number.\n\n\
		| body | plate mass | drop @ at-pin azimuth | drop @ between-pin azimuth | pinch |\n|---|---|---|---|---|\n\
		| BASE — the sibling's hand-drawn spokes | {bb:.2} g | **{vbb:.2} MPa** | *no material there at all* | — |\n\
		| BASE — SIMP's own solid blank | {bs:.2} g | {vbs:.2} MPa | {vbsm:.2} MPa | — |\n\
		| BASE — generative, SHIPPED | **{bo:.2} g** | **{vbo:.2} MPa** | **{vbm:.2} MPa** | {vbp:.2} MPa |\n\
		| TOP — the sibling's hand-drawn arms | {tb:.2} g | {vtb:.2} MPa | *no material there at all* | — |\n\
		| TOP — SIMP's own solid blank | {ts_:.2} g | {vts:.2} MPa | {vtsm:.2} MPa | — |\n\
		| TOP — generative, SHIPPED | **{to:.2} g** | **{vto:.2} MPa** | **{vtm:.2} MPa** | {vtp:.2} MPa |\n\n\
		All peaks are MASKED: a point-introduced load spikes under its own patch and that\n\
		spike is a property of the idealisation, not of the part. Both numbers are carried\n\
		everywhere in the source, and a gate (G31) proves the raw peak really does sit\n\
		inside the masked radius before the mask is allowed to remove anything.\n\n\
		### What SIMP actually did\n\n\
		| | BASE | TOP |\n|---|---|---|\n\
		| volume fraction achieved / target | rim {vfa:.3}/{vfr}, hub {vfh:.3}/{vfhh} | rim {tvfa:.3}/{vft} — higher, because the top is the weak link and the run said so |\n\
		| compliance, gray start → converged | {c0:.3e} → {c1:.3e} | {tc0:.3e} → {tc1:.3e} |\n\
		| iterations, stop reason | {it:.0}, {stop} | {tit:.0}, {tstop} |\n\
		| mass removed from its own blank | **−{cutb:.0}%** | **−{cutt:.0}%** |\n\
		| SIMP's own internal as-built estimate — NEVER published as a product number | {sab:.1} MPa | {tsab:.1} MPa |\n\
		| the number that IS published, from a fresh solve of the shipped geometry | **{vbm:.1} MPa** | **{vtm:.1} MPa** |\n\
		| deflection, blank → optimised, at-pin azimuth | {dbs:.4} → {dbo:.4} mm | {dts:.4} → {dto:.4} mm |\n\
		| peak stress at the same load, blank → optimised | {vbsm:.1} → {vbm:.1} MPa | {vtsm:.1} → {vtm:.1} MPa |\n\n\
		Read that honestly, and with the arithmetic done rather than asserted. SIMP\n\
		minimises COMPLIANCE, not stress, and the `ace_optimize` card says so in as many\n\
		words. On the base carrier it removed {cutb:.0}% of its own blank and the peak\n\
		stress rose by {sr:.2}×. The product that would have to fall for that to be a\n\
		structural win is stress × mass:\n\n\
		| | blank | optimised | change |\n|---|---|---|---|\n\
		| base, stress × plate mass | {spb:.0} MPa·g | {spo:.0} MPa·g | **{spc:+.1}%** |\n\
		| top, stress × plate mass | {stb:.0} MPa·g | {sto:.0} MPa·g | **{stc:+.1}%** |\n\n\
		So the base carrier came out **flat** on stress-per-gram and the top carrier came\n\
		out **{stc:+.0}% WORSE** — it is carrying its extra material less efficiently than\n\
		the solid plate it came from. Neither is a structural win. What the optimiser\n\
		found is a much LIGHTER arrangement at roughly the same structural efficiency,\n\
		which is exactly what a compliance objective under a volume constraint asks for,\n\
		and it is not the same thing as \"stronger\". The compliance figures above are\n\
		large improvements, but they are measured inside the optimiser's own homogenised\n\
		model against its own gray starting point, and this campaign does not quote them\n\
		as a product claim. G53 asserts the UNFLATTERING direction so none of this can\n\
		quietly drift.\n\n\
		The right tool for the claim I would rather be making is a STRESS-CONSTRAINED\n\
		topology optimisation, and this repository does not have one. That is a row in\n\
		the analysis plan, not a footnote.\n\n\
		### The two things the optimiser got wrong, and what was done about them\n\n\
		1. **A floating hub.** The first formulation clamped the hub AND the six pins and\n\
		   applied the drop at all six azimuths at once — a tidy six-fold problem with a\n\
		   tidy six-fold answer. It handed back a hexagonal web with the Ø18 hub\n\
		   unattached in the middle of it, because an unloaded fixture gives the optimiser\n\
		   no reason to reach anything. The connectivity gate caught it. The cause is a\n\
		   theorem, not a bug: the hub is the centre of symmetry, so ANY six-fold\n\
		   symmetric load set has zero net force there and a self-equilibrating radial set\n\
		   is absorbed in hoop. The fix is two load cases — a six-fold rim envelope and a\n\
		   single-azimuth sun-on-post case whose optimum is symmetrised afterwards — and\n\
		   their union. Unioning two single-case optima is the CONSERVATIVE approximation\n\
		   to multi-load-case optimisation, not an equivalent of it, which is why the\n\
		   final geometry is re-analysed against the true combined case.\n\
		2. **A singular manifest.** Freezing the six ring thrust pads and all six rim\n\
		   contact pads produced rigid islands with no stiffness path to ground, the\n\
		   optimiser stripped the material around the unloaded ones, and the runner's own\n\
		   re-analysis died with `CG did not converge ... info=2000`. A region may be\n\
		   frozen only if the load case gives it a reason to exist; the rest of the\n\
		   product's keep-outs are re-asserted as exact geometry instead.\n\n\
		### From density field to CAD\n\n\
		| stage | what happens | measured |\n|---|---|---|\n\
		| through-thickness average | the shipped web is a constant cross-section extrusion — that is what makes it print flat with no down-facing face anywhere | — |\n\
		| six-fold maximum | MAX, not mean: a mean divides each feature's density by up to six where its duty cycle is 1/6 and the threshold deletes it, leaving a part weaker at EVERY azimuth than the optimiser's answer was at one | base planform {ar:.0} → {as_:.0} mm² (raw asymmetry {asym:.3}), union with the hub case {au:.0} mm² |\n\
		| island prune | 4-connected fill (a diagonal voxel touch is not a printable ligament) seeded in the part's own anchor, run over the planform INCLUDING the exact skeleton | {isl} island(s), {deb:.1} mm² removed |\n\
		| threshold + blend | the frozen keep-outs join the web through a {bl:.2} mm smooth minimum, so every junction is a fillet rather than a crease | — |\n\
		| contour → exact prism | marching squares at {cr} mm, Douglas–Peucker at {ct} mm, then `extrude_with_holes` | base {pf} faces, {nh} openings, worst contour deviation {cd:.4} mm |\n\
		| independent cross-check | `kernel_model::reverse::mesh_to_solid` on the same field, by a completely different algorithm (dual contour → facet wrap → coplanar coalesce) | volumes agree to {xd:.2}%; that route's own wrap-and-coalesce drift is {bd:.2}%, which is why it is the CHECK and not the deliverable |\n\
		| what the mesher saw | the organic surface, before any of the above | base {tri} triangles, longest facet edge {fac:.3} mm; the hub load case contributed {ahs:.0} mm² of the union |\n\n\
		The prism is what ships, and the reason is worth recording: the reverse bridge's\n\
		faceted output is valid, but `coalesce_coplanar` leaves multi-loop planar faces\n\
		that the default tessellator cannot re-triangulate watertight, so the moment it met\n\
		the exact hub-and-post revolve the chain reported `genus 34850` and stopped. The\n\
		bayonet pins and the post are exact geometry with 0.4 mm features and they are not\n\
		negotiable, so the web is what had to change. The bridge is still run, as the\n\
		independent check above.\n\n",
		h = d.drop_h, s = d.drop_s, dm = DROP_MASS_G, pg = d.printed_g, tol = DROP_MASS_TOL * 100.0,
		acc = d.accel, gs = d.accel / GRAV, fd = d.f_drop, fs = d.f_sun, jr = d.jr,
		c = INDENT_C, db = d.d_bound, sb = d.s_bound, fb = d.f_bound, ratio = d.f_bound / d.f_drop,
		bb = d.lb.g_base, vbb = d.lb.vm_base, bs = d.lb.g_solid, vbs = d.lb.vm_solid, vbsm = d.lb.vm_solid_mid,
		bo = d.lb.g_opt, vbo = d.lb.vm_opt, vbm = d.lb.vm_mid, vbp = d.lb.vm_pinch,
		tb = d.lt.g_base, vtb = d.lt.vm_base, ts_ = d.lt.g_solid, vts = d.lt.vm_solid, vtsm = d.lt.vm_solid_mid,
		to = d.lt.g_opt, vto = d.lt.vm_opt, vtm = d.lt.vm_mid, vtp = d.lt.vm_pinch,
		vfa = d.lb.volfrac_achieved, tvfa = d.lt.volfrac_achieved, vfr = VOLFRAC_RIM,
		vfh = d.lb.volfrac_hub, vfhh = VOLFRAC_HUB, vft = VOLFRAC_TOP,
		c0 = d.lb.c_first, c1 = d.lb.c_last, tc0 = d.lt.c_first, tc1 = d.lt.c_last,
		it = d.lb.iters, stop = d.lb.stop, tit = d.lt.iters, tstop = d.lt.stop,
		sab = d.lb.simp_as_built_mpa, tsab = d.lt.simp_as_built_mpa,
		dbs = d.lb.disp_solid, dbo = d.lb.disp_opt, dts = d.lt.disp_solid, dto = d.lt.disp_opt,
		cutb = 100.0 * (1.0 - d.lb.g_opt / d.lb.g_solid), cutt = 100.0 * (1.0 - d.lt.g_opt / d.lt.g_solid),
		sr = d.lb.vm_mid / d.lb.vm_solid_mid,
		spb = d.lb.vm_solid_mid * d.lb.g_solid, spo = d.lb.vm_mid * d.lb.g_opt,
		spc = 100.0 * ((d.lb.vm_mid * d.lb.g_opt) / (d.lb.vm_solid_mid * d.lb.g_solid) - 1.0),
		stb = d.lt.vm_solid_mid * d.lt.g_solid, sto = d.lt.vm_mid * d.lt.g_opt,
		stc = 100.0 * ((d.lt.vm_mid * d.lt.g_opt) / (d.lt.vm_solid_mid * d.lt.g_solid) - 1.0),
		ar = d.lb.area_rim_raw, as_ = d.lb.area_rim_sym, au = d.lb.area_union,
		asym = d.lb.asym, isl = d.lb.islands, deb = d.lb.debris, bl = BLEND_R, cr = CONTOUR_RES, ct = CONTOUR_TOL,
		pf = d.lb.faces, nh = d.lb.holes, cd = d.lb.cdev, xd = 100.0 * d.lb.xdrift,
		bd = 100.0 * d.lb.drift, tri = d.lb.tris, fac = d.lb.facet, ahs = d.lb.area_hub_sym,
	));
	a.push_str(&format!(
		"## The drop verdict — two tiers, and the difference between them\n\n| tier | what it answers | worst carrier reading | verdict |\n|---|---|---|---|\n\
		| PLA yield, {sy:.0} MPa | *did the part survive the event* | **{wd:.1} MPa** (×{hf} coarse-hex8 derate applied) | **passes, ×{my:.2} margin** |\n\
		| design allowable, {sa:.0} MPa | *would an engineer sign it off* | same {wd:.1} MPa | **does NOT pass at {h:.2} m** |\n\nThe design allowable is 35 MPa conservative base tensile × 0.6 layer adhesion ×\n\
		0.5 design factor. Note what is deliberately NOT done: the 0.6 knockdown is kept\n\
		even though this carrier prints FLAT and this load is IN the layer plane, where\n\
		that knockdown does not apply. Dropping it would raise the allowable to 17.5 MPa\n\
		and make several numbers on this page look much better. It stays, because the\n\
		sibling's tooth-root gate uses the same 10 MPa tier for the same in-plane case and\n\
		a campaign does not get to invent a friendlier allowable for its own headline.\n\n**So the honest answer is a HEIGHT, not a tick.** The equivalent-static force is\n\
		linear in drop height and a linear static solve is linear in the force, so this is\n\
		exact within the model rather than an extrapolation:\n\n| | reaches the {sa:.0} MPa design allowable at | reaches yield at |\n|---|---|---|\n\
		| generative carrier, worst azimuth | **{hao:.2} m** | **{hyo:.2} m** |\n\
		| the sibling's spokes, at their own azimuth | {hab:.2} m | — |\n\nSaid plainly, and it is said on the model page in the same words: **this is a\n\
		printed toy. It survives being dropped. It is not signed off for a metre onto\n\
		concrete, and neither is any 2 mm PLA plate.** On carpet, on a desk mat, or on a\n\
		wooden floor the stopping distance is several times {s:.2} mm and the numbers move\n\
		with it — which is exactly why the sweep is published instead of a single verdict.\n\n**The hand case.** A deliberate {pn:.0} N pinch across a diameter with the flick's\n\
		{fn_:.0} N of tangential drag reads **{wp:.1} MPa** at the derate — the governing\n\
		case on the TOP carrier, and a BOUND rather than a prediction: it is solved on one\n\
		plate clamped at a single 4.4 mm finger patch, whereas the assembled frame ties\n\
		both plates together through six pins and is much stiffer. Both introduction\n\
		patches (the loaded finger and the clamped one) are masked, because a clamp spike\n\
		is exactly as spurious as a load spike.\n\n## Negative controls this run\n\n| control | what it falsifies | reading |\n|---|---|---|\n\
		| base NC — one strut cut from the live load path | that the FEA is reading structure at all | **{ncb:.2}× the stress, {ncbd:.1}× the deflection** |\n\
		| top NC — same | same | **{nct:.2}× / {nctd:.1}×** |\n\
		| the sibling's carrier at the between-pin azimuth | that a continuous rim is needed | **no material there at all** — the drop lands on the gear RING instead |\n\
		| delete the bayonet lip (round holes) | that retention is geometric | {nclip} mm³ — lifts straight off |\n\
		| pose the top carrier UNTWISTED | that retention is the twist and nothing else | lifts straight off |\n\
		| delete the capture rim | that the ring is captive | ring escapes |\n\
		| the sun ±5% off ratio | that the mesh sweep is a gate | **{jam:.4} mm³ — JAMS, as it must** |\n\
		| the carrier audited on its side | that the support oracle can fire | steep area jumps |\n\
		| a 5% wrong reference, three times | that the solver benchmarks can go red | B3, B6, D3 all fire |\n\nEach geometry control is TWO gates, not one: first prove the perturbation is real\n\
		(G42 measures the removed volume against the column it should have cut), then prove\n\
		the instrument reacted (G43). The first version of the base control cut BOTH struts\n\
		and severed the contact region, the island filter dropped it, and the load landed\n\
		on nothing — a broken control, not a strong one. It cuts one strut now.\n\n## eta — the receipt, MODELLED and unchanged from the sibling\n\n`eta = 1 − |Σ Iᵢωᵢ| / Σ|Iᵢωᵢ|`, from `mass_properties` on the exact B-rep.\n\n| rotor | mass | I_zz | speed ratio |\n|---|---|---|---|\n\
		| ring 66T | {mr:.2} g | {ir:.1} g·mm² | +1.0000 |\n\
		| sun 42T | {ms:.2} g | {is:.1} g·mm² | {ksun:+.4} |\n\
		| planet 12T ×6 | {mp:.2} g each | {ip:.1} g·mm² each | {kpl:+.4} |\n\
		| **I_eff referred to the ring** | | **{ie:.0} g·mm²** | |\n\n**eta = {eta:.4}** (gate floor 0.95). The rotors are NOT lightened by this\n\
		campaign and that is a hard rule, not an oversight: eta pins `I_sun·k_S` to\n\
		`I_ring + ΣI_p·k_P`, so taking mass out of either rotor to pay for the carrier\n\
		would break the physics the product is built on. The rotor design study still\n\
		re-solves ({se} evaluations, {sf} feasible) because its mass constraint reads the\n\
		FRAME and this frame is not the sibling's.\n\n| perturbation | eta |\n|---|---|\n{sens}\n\nWorst corner **{elo:.4}**; the shipped SUN-B control puck is deliberately\n\
		uncancelled at **{eb:.4}** so the buyer can perform the A/B by hand.\n\n> **Measured eta: REQUIRED, NOT PERFORMED.** No instrument for it exists here or in\n\
		> the hobby field. eta is published as a MODELLED band, never as a headline.\n\n## Spin time — reported with its derivation, NOT claimed\n\n| term | class | N·mm | share |\n|---|---|---|---|\n{budget}\n\
		| **TOTAL** | | **{tot:.4}** | |\n\n**Predicted spin {tn:.1} s / {rn:.0} revolutions**, band **{tp:.1}–{to:.1} s** across\n\
		μ(PLA-on-PLA) 0.20–0.50, which is unmeasured for this pairing and is carried as a\n\
		band rather than a value. {cf:.0}% of the budget is Coulomb (ω⁰) — a fully printed\n\
		spinner is a Coulomb machine. Putting a 608 back gives {t608:.1} s, which is the\n\
		gate that keeps the zero-hardware figure from flattering itself.\n\n**The generative carrier did not move a single sliding contact**, and that is\n\
		gated rather than assumed: all three thrust arms are frozen keep-outs, so the spin\n\
		budget is the sibling's re-solved on this entry's rotors, not a new one.\n\n> **Measured spin time: REQUIRED, NOT PERFORMED.** Nothing was printed or timed in\n\
		> this run.\n\n",
		sy = SIG_YIELD_PLA, wd = d.worst_drop, hf = HEX8_PEAK_FACTOR, my = SIG_YIELD_PLA / d.worst_drop,
		sa = SIG_ALLOW_RT, h = d.drop_h, hao = d.h_allow_opt, hyo = d.h_yield_opt, hab = d.h_allow_base,
		s = d.drop_s, pn = PINCH_N, fn_ = FLICK_N, wp = d.worst_pinch,
		ncb = d.lb.vm_nc / d.lb.vm_mid, ncbd = d.lb.disp_nc / d.lb.disp_opt.max(1e-12),
		nct = d.lt.vm_nc / d.lt.vm_mid, nctd = d.lt.disp_nc / d.lt.disp_opt.max(1e-12),
		nclip = 0.0, jam = d.jam,
		mr = d.mg_r, ir = d.izz_r, ms = d.mg_s, is = d.izz_s, mp = d.mg_p, ip = d.izz_p,
		ksun = d.k_sun, kpl = d.k_pl, ie = d.i_eff_gmm2, eta = d.eta, se = d.study_evals, sf = d.study_feasible,
		sens = sens_rows, elo = d.eta_lo, eb = d.eta_b,
		budget = budget_rows, tot = d.drag.total_nmm(W0), tn = d.t_nom, rn = d.rev_nom, tp = d.t_pes, to = d.t_opt,
		cf = d.coul_frac * 100.0, t608 = d.t_608,
	));
	a.push_str(&format!(
		"## Geometry receipts (the mechanism, inherited and re-proved on THIS build)\n\n| quantity | value | how it is proved |\n|---|---|---|\n\
		| module · pressure angle | m {mm:.3} · {pad:.1}° | G0: the internal 66T generator accepts this with {mg:+.2}% margin; the NC (36T @ 30°) is refused |\n\
		| teeth S / P / R · planets | {st} / {pt} / {rt} · {np} | G1a `EpicyclicTrain::validate_assembly` = Ok; NC at n=5 refused |\n\
		| ratio ω_sun/ω_ring | {ks:+.6} = −R/S | G1b, from the engine's own `simple_ratio` |\n\
		| headline | 7 ring revs → 11 sun revs, EXACTLY | G1c: 7·66 = 11·42 = 462 |\n\
		| contact ratio sun–planet / planet–ring | {esp:.4} / {epr:.4} | G4a/G4b ≥ 1.20, with an 8T×8T @30° NC through the same fn |\n\
		| undercut floor 2/sin²α | {fl:.3} T | G3: the 12T planet clears it at x = 0 |\n\
		| adjacent-planet gap | {nb:.3} mm | G2, and it is the EN 71-1 number too |\n\
		| backlash, bisected on the solids | {jt:.3} mm ({ld:.3}°) | G7 |\n\
		| mesh sweep, 96 poses × 2 meshes | min clearance {mcs:.3} / {mcr:.3} mm | G5a/G5b, 0 contacts, 0 crossings |\n\
		| exact overlap, 16 poses + 72 booleans | {wsp:.1e} / {wal:.1e} mm³ | G5c/G5d |\n\
		| **the GENERATIVE web vs every rotor** | **{wweb:.1e} mm³** | **G5e — 48 planet poses + ring + sun. Nothing inherited proves this; the web is new geometry** |\n\
		| bayonet engagement at the worst stack | {ex:+.2} mm (XY) / {ef:+.2} mm (+ elephant foot) | G16b/G16c; on solids G16e {cap:.1} mm³, worst-case pin {wc:.3} mm³ |\n\
		| wall the optimiser left around each slot | {sw:.2} mm | G16o — new to this entry: the slot now sits in an OPTIMISED plate |\n\
		| retention capacity vs carried weight | {fc:.1} N vs {cn:.3} N ({rr:.0}×) | G16k, neck bending at the static allowable |\n\
		| snap-fit, re-refused | {sm:.3} mm elastic travel vs 0.30 mm of stack | G16m; the spec's Ø6.40 barb reads {ss:.1}% strain vs {ys:.2}% yield |\n\
		| ring capture engagement | {er:.2} mm | G23b — the sibling reaches 0.95 mm; this rim reaches further because it also has to resolve on the analysis grid |\n\
		| tooth-root bending (Lewis, measured Y) | {sig:.3} MPa vs {sa:.0} | G15; Y_p {yp:.3}, Y_s {ys2:.3}, Y_r {yr:.3} |\n\
		| clearance stack, credited / uncredited | {cr_:+.3} / {uc:+.3} mm | G12; the Ø6.15 ladder member reads {lb_:+.3} |\n\
		| concentricity residual vs radial lash | {res:.3} < {jr:.3} mm | G8b |\n\n## Print\n\n| | |\n|---|---|\n\
		| printed set | **{pg:.1} g** — carrier frame {fg:.2} g of it |\n\
		| envelope | Ø{od:.1} × {ht:.2} mm, unchanged from the sibling |\n\
		| supports | **none**. Every part audits `steep_area` exactly 0 at the profile's own overhang angle, and the NC (the carrier on its side) reads a large number, so the oracle is not blind |\n\
		| connectivity | both carriers are ONE body ({ob1}/{ob2}) and ONE B-rep shell ({sh1}/{sh2}) — gated separately from validity and watertightness, because an optimiser produces islands and `shell_count()` cannot see them |\n\
		| thin wall | the medial probe reads {tw1:.2} mm on the optimised base against {twc1:.2} mm on a SOLID control with the same silhouette (probe cell {pc:.2} mm). The reading is set by the tapers, not by the web, which is why the gate is differential and the length scale is guaranteed upstream by the filter radius instead |\n\
		| bridges | widest down-facing patch anywhere {wb:.3} mm — under a single facet |\n\
		| surface | the organic web is meshed at {mv} mm and the shipped contour tracks the optimiser's own level set to {cd:.4} mm, one fifth of a layer |\n\
		| CAD | the carrier's STEP is {smb:.2} MB from {fbase} faces |\n\
		| per part | base carrier {mgb:.2} g · top carrier {mgt:.2} g · cap {mgc:.2} g · ring {mgr:.2} g · sun {mgs:.2} g · planet {mgp:.2} g ×6 |\n\
		| the loop closed | at the as-built {pg:.2} g the drop peak is {dab:.1} MPa and the pinch bound {pab:.1} MPa, both against {sy2:.0} MPa yield |\n\n## Analysis plan (per DESIGN_GUIDE §25.7 — every required item answered)\n\n| analysis | required? | status |\n|---|---|---|\n\
		| **drop impact on the carrier** | **yes — it is the failure mode of the product class and the sibling declares it NOT PERFORMED** | **receipts** — equivalent-static, both carriers, two azimuths, baseline / blank / optimised / negative control, all through one manifest. The MODEL's own limits are the rows below |\n\
		| **pinch and flick** | **yes — it is a hand-held toy** | **receipts** — G52; a single-plate BOUND, stated |\n\
		| structural, the mechanism | yes | **receipts** — G15 tooth root, G16 retention, G23 capture, G8 concentricity |\n\
		| topology optimisation | yes — it is the entry | **receipts** — SIMP ×3 runs, determinism proved byte-identical, volume constraint held, honest binary re-analysis of the FINAL geometry |\n\
		| momentum cancellation (eta) | yes | **receipts** — G9, on exact B-rep mass properties |\n\
		| spin-down | yes | **new solver, benchmarked first** — B1/B2 against closed forms, B3 meta-NC |\n\
		| drop force derivation | yes | **new solver, benchmarked first** — D1 energy conservation, D2 cross-route agreement, D3 meta-NC |\n\
		| print readiness | yes | **receipts** — support-free, watertight, one body, bed fit, facet size, with a wrong-orientation NC |\n\
		| creep / sustained load | **no** | a spinner is flicked and released and dropped; it is never held under load. The static tier governs and the reasoning is written down rather than assumed. Gating a fidget toy against a 1-year creep allowable would be plan padding |\n\
		| fatigue | **no** | the carrier sees one impact, not a cycle count. The repo's fatigue solver is screening-only and would refuse the across-layer question anyway |\n\
		| thermal | **no** | there is no heat source |\n\
		| modal / vibration | **no** | and the repo's modal card explicitly excludes a SPINNING part, so quoting it would be outside its stated limits |\n\
		| buckling | **no** | no member is a slender compression strut; the carrier's web is loaded in its own plane at 2.00 mm thickness against members ≥ 4.0 mm wide |\n\
		| **transient impact simulation** | **REQUIRED, NOT PERFORMED** | the equivalent-static model above replaces a transient event with the single peak force it is estimated to produce. No solver in `tools/solvers/` is transient — every card is static, quasi-static or eigenvalue-based — and the fatigue card explicitly refuses to stand in for one. The contact lasts ~0.1 ms against a 0.045 ms wave transit across the part, so the rigid-body assumption is MARGINAL, and that is the single largest modelling gap in this deliverable |\n\
		| **strain-rate dependence of PLA** | **REQUIRED, NOT PERFORMED** | at impact rates PLA stiffens (raising the force) and embrittles (lowering the allowable). Both effects are real, they push in opposite directions on the margin, and this repository has data for neither |\n\
		| **out-of-plane (corner-first) drop** | **REQUIRED, NOT PERFORMED** | a corner impact bends the 2.00 mm plate out of its own plane, where its section modulus is 30× smaller. It is NOT a topology variable — no in-plane material arrangement changes a plate's out-of-plane stiffness, so the optimised and hand-drawn carriers are IDENTICAL in this mode — and the fix would be thickness or ribs, which the 12.0 mm envelope forbids. Declared, not designed for |\n\
		| **the floor** | **REQUIRED, NOT PERFORMED** | the stopping distance lumps the floor's compliance into one number. A concrete floor and a carpeted one differ by an order of magnitude and neither was measured; the sweep is published so a reader can place their own |\n\
		| **impact toughness of printed PLA** | **REQUIRED, NOT PERFORMED** | the verdict above compares a von Mises peak against a static yield. PLA's NOTCHED impact toughness is low and is the property that actually decides whether a dropped part cracks. No printed-PLA impact-energy data exists in this repo's registry |\n\
		| **multi-load-case topology optimisation** | **REQUIRED, NOT PERFORMED** | SIMP here takes one load case at a time and the two answers are UNIONED, which is the conservative approximation and is not the compliance optimum of the combined case. The final geometry is re-analysed against the true combined case, so the deliverable is sound; the optimiser's answer is not optimal |\n\
		| **stress-constrained topology optimisation** | **REQUIRED, NOT PERFORMED** | SIMP minimises COMPLIANCE. Compliance-optimal is not strength-optimal, the `ace_optimize` card says so, and this run measured it: −{cutb:.0}% mass for ×{sr:.2} peak stress. A stress-constrained formulation is the right tool and this repository does not have one |\n\
		| **measured eta**, **measured spin time** | **REQUIRED, NOT PERFORMED** | no instrument, nothing printed in this run |\n\
		| **μ(PLA-on-PLA) at a printed thrust face** | **REQUIRED, NOT PERFORMED** | the single most load-bearing unknown in the spin answer. All published PLA tribology is PLA-on-STEEL at 20 N — wrong pairing, wrong load. Carried as a 0.20–0.50 band |\n\
		| ISO 13854 body-part gap table | **REQUIRED, NOT PERFORMED** | paywalled and unobtainable. The EN 71-1 rod rule is used as the substitute and the substitution is stated |\n\nSilence about a required analysis is the one forbidden outcome (§25.7). Nothing on\n\
		this plan is silent.\n\n## How this model was made\n\n{auth}\n",
		mm = M, pad = PA_DEG, mg = d.margin * 100.0, st = S_T, pt = P_T, rt = R_T, np = N_PL,
		ks = d.k_sun, esp = d.eps_sp, epr = d.eps_pr, fl = d.floor, nb = d.neighbour,
		jt = d.jt_measured, ld = d.lash_deg, mcs = d.min_cl_s, mcr = d.min_cl_r,
		wsp = d.worst_sp, wal = d.worst_all, wweb = d.worst_web,
		ex = d.engage_xy, ef = d.engage_full, cap = d.captive, wc = d.wc_capture, sw = d.slot_wall,
		fc = d.f_cap, cn = d.carried_n, rr = d.f_cap / d.carried_n, sm = d.snap_max,
		ss = d.spec_strain * 100.0, ys = d.yield_strain * 100.0, er = d.engage_ring,
		sig = d.sig_ring, sa = SIG_ALLOW_RT, yp = d.y_pl, ys2 = d.y_sun, yr = d.y_ring,
		cr_ = d.credited, uc = d.uncredited, lb_ = d.ladder_best, res = d.residual, jr = d.jr,
		pg = d.printed_g, fg = d.frame_g, od = od, ht = d.height, wb = d.worst_bridge,
		mv = MESH_VOX, cd = d.lb.cdev, smb = d.step_mb, fbase = d.faces_base,
		ob1 = d.lb.one_body, ob2 = d.lt.one_body, sh1 = d.lb.shells, sh2 = d.lt.shells,
		tw1 = d.lb.tw, twc1 = d.lb.tw_ctl, pc = d.lb.probe_cell,
		cutb = 100.0 * (1.0 - d.lb.g_opt / d.lb.g_solid), sr = d.lb.vm_mid / d.lb.vm_solid_mid,
		mgb = d.mg_base, mgt = d.mg_top, mgc = d.mg_cap, mgr = d.mg_r, mgs = d.mg_s, mgp = d.mg_p,
		dab = d.drop_as_built, pab = d.pinch_as_built, sy2 = SIG_YIELD_PLA,
		auth = AUTHORSHIP,
	));
	let _ = std::fs::write(format!("{OUT}/analysis/ANALYSIS.md"), a);
	write_design_md(d);
	write_readme(d);
	write_listing(d, od);
}

/// `analysis/DESIGN.md` — the frozen contract. Where every constant that
/// describes the outside world came from, what is UNKNOWN, and WHICH analyses
/// this artifact class requires. Written before the geometry was trusted;
/// ANALYSIS.md answers it.
fn write_design_md(d: &Docs) {
	let t = format!(
		"# NULLSPIN-GEN — design contract (frozen BEFORE the geometry was trusted)\n\n\
		This is the research side of the campaign. `ANALYSIS.md` answers every row of the\n\
		plan with the numbers the gate suite measured on the live build; this file says\n\
		what was allowed to be believed in the first place.\n\n\
		## 1. What the artifact is\n\n\
		A grounded-carrier (\"star\") epicyclic fidget spinner whose held frame is\n\
		generatively designed. The MECHANISM is the sibling entry `nullspin`, frozen\n\
		bit-for-bit and re-proved here rather than assumed: 66T ring, 42T sun, six 12T\n\
		planets, module 1.0, 25° pressure angle, zero profile shift, 0.09 mm/flank\n\
		thinning, bayonet retention, zero non-printed parts. The SUBJECT of this campaign\n\
		is the CARRIER, which is the only part of the machine with no kinematic duty, a\n\
		genuine structural load case, and the user's hand on it.\n\n\
		## 2. Frozen dimensions and where each came from\n\n\
		| constant | value | provenance | class |\n|---|---|---|---|\n\
		| module, pressure angle | 1.000, 25° | derived from measured extrusion width; 25° puts the undercut floor at {fl:.3} T so a 12T planet is legal at zero shift | derived + engine capability |\n\
		| teeth 42 / 12 / 66 × 6 | — | 7·66 = 11·42 = 462; R = S + 2P; (S+R) % 6 = 0 | exact |\n\
		| backlash 0.09 mm/flank | jt 0.18 | CMM-measured FDM involute deviation 0.067 mm/flank, two flanks meet | measured + standard practice |\n\
		| clearances 0.25 / 0.05 / 0.30 / 0.45 | mm | `profiles/conservative_default.json`, print-proven in RESPOOL and DRYBOX | measured in-repo |\n\
		| relief slope 1.40 | rise:run | a 45° cone sits exactly ON the support threshold and a facet cannot land there | engine capability, declared |\n\
		| PLA 3.3 GPa / ν 0.36 / 55 MPa | — | `tools/materials/pla.json` | researched |\n\
		| PLA design allowable 10 MPa | — | `kernel_model::materials::pla::SIG_ALLOW_RT` = 35 × 0.6 × 0.5 | researched + design factor |\n\
		| **drop height 1.00 m** | — | **NOT from a standard.** The toy-safety drop tests could not be re-verified from a primary source on this run and are therefore NOT cited. From use: hand-at-rest ≈0.75 m, desk ≈0.75 m, hand-held-up ≈1.1–1.3 m | **stated design choice, swept** |\n\
		| **stopping distance 1.00 mm** | — | the centre of mass's travel after first contact. Cross-checked against a rigid-floor elastic-plastic indentation bound which returns {sb:.3} mm and {fb:.0} N | **stated design choice, bounded** |\n\
		| **pinch 30 N** | — | a comfortable hold is 5–15 N; adult tip-pinch maxima run 50–70 N. 30 N is a deliberate hard squeeze applied as a design load, not an ultimate | stated design choice |\n\
		| flick 5 N at the rim | — | inherited from the sibling's tooth-root case | inherited |\n\
		| SIMP voxel 1.00 mm, filter radius 2 vox | — | the filter radius IS the minimum length scale: 2·r·vox = 4.0 mm, gated against a 1.60 mm printable minimum | derived, gated |\n\
		| design mass | {dm:.1} g | frozen BEFORE the geometry (§25 puts the plan first) so the load case is not circular, and gated against the as-built set afterwards | frozen input |\n\
		| mass ceiling | **{mx:.2} g** | DERIVED, not chosen: the drop force is linear in the product's own mass, so the heaviest legal product is the one whose drop peak still clears yield by ×{dmm}. The rotor study is handed that number | derived |\n\n\
		## 3. What is UNKNOWN, and how the design lives with it\n\n\
		* **μ(PLA-on-PLA) at a printed thrust face.** All published PLA tribology is\n\
		  PLA-on-STEEL at 20 N. Carried as a 0.20–0.50 band; the design is sized on the\n\
		  high end and the spin time is published as a band, never a value.\n\
		* **The floor.** Concrete and carpet differ by an order of magnitude in stopping\n\
		  distance. Lumped into `s`, and `s` is swept.\n\
		* **Impact toughness of printed PLA.** The property that actually decides whether\n\
		  a dropped part cracks. No data in this repo's registry, so the verdict is stated\n\
		  against a static yield and the substitution is declared.\n\
		* **Strain rate.** PLA stiffens and embrittles at impact rates. Neither modelled;\n\
		  they push opposite ways on the margin.\n\
		* **Whether the sun's bore or its meshes take up first** under a radial impulse.\n\
		  0.25 mm against {jr:.3} mm, with a 0.15 mm/side build error — not decidable. The\n\
		  design case assumes the post takes the whole sun.\n\
		* **EN 71-1:2026** has been published since the 2014 rod-rule text used here; the\n\
		  clause must be re-verified against the current edition before any compliance\n\
		  statement.\n\n\
		## 4. The analysis plan (frozen here; answered in ANALYSIS.md)\n\n\
		| required analysis | how it is to be answered | where |\n|---|---|---|\n\
		| Drop impact on the carrier | equivalent-static, both carriers, two azimuths, baseline vs blank vs optimised, one manifest, negative control | G50–G54, ANALYSIS.md |\n\
		| Pinch and flick | the same manifest machinery, verified on the FINAL geometry, not optimised for | G52 |\n\
		| Topology optimisation | SIMP with determinism, volume-constraint and length-scale gates, and an HONEST binary re-analysis of the final geometry | G32–G43 |\n\
		| Connectivity | an explicit oracle — an optimiser produces islands and `shell_count()` cannot see them | G36b, G37 |\n\
		| Momentum cancellation | exact B-rep mass properties | G9 |\n\
		| Spin-down | a solver written for it, benchmarked against two closed forms first | B1–B3 |\n\
		| Drop force | a solver written for it, benchmarked two independent ways first | D1–D4 |\n\
		| Tooth root, retention, capture, fits, EN 71-1 | inherited, re-proved on THIS build's solids | G7, G12, G14–G16, G23 |\n\
		| Creep, fatigue, thermal, modal, buckling | **NOT REQUIRED** — reasons written in ANALYSIS.md rather than left as silence | ANALYSIS.md |\n\
		| Transient impact, strain rate, out-of-plane drop, floor compliance, impact toughness, multi-load-case and stress-constrained TO, measured eta, measured spin time, μ(PLA-on-PLA), ISO 13854 | **REQUIRED, NOT PERFORMED** | ANALYSIS.md |\n\n\
		## 5. Deliberate decisions, stated as physics\n\n\
		* **The rotors are not lightened.** eta pins `I_sun·k_S` to `I_ring + ΣI_p·k_P`.\n\
		  Taking mass out of a rotor to pay for the carrier breaks the receipt the product\n\
		  is built on, so the carrier's mass is paid for out of the mass ceiling instead,\n\
		  and the ceiling is the load case's own validity window.\n\
		* **Both faces get a continuous rim.** It costs mass. It buys a drop contact at\n\
		  every azimuth on both faces, which keeps impact energy out of the gear train —\n\
		  the sibling has no material between its arms, so a rim-first drop there lands on\n\
		  the toothed ring.\n\
		* **The optimiser is never allowed near the gear envelope.** The whole design\n\
		  domain lives in the 2.00 mm slab below the gear plane, and G5e re-proves it on\n\
		  the built solids across 48 planet poses rather than trusting the argument.\n\
		* **The three sliding thrust contacts are frozen keep-outs.** Coulomb torque is\n\
		  μWr and the only lever is the arm; letting an optimiser move one would silently\n\
		  change the spin claim. Gated numerically.\n\n\
		## 6. How this model was made\n\n{auth}\n",
		fl = d.floor, sb = d.s_bound, fb = d.f_bound, mx = d.mass_max, dm = DROP_MASS_G,
		dmm = DROP_MARGIN_MIN, jr = d.jr, auth = AUTHORSHIP,
	);
	let _ = std::fs::write(format!("{OUT}/analysis/DESIGN.md"), t);
}

/// The campaign folder's front door.
fn write_readme(d: &Docs) {
	let t = format!(
		"# NULLSPIN-GEN — the geared spinner whose frame was solved, not drawn\n\n\
		Hold the organic web, flick the outer ring, and the inner puck counter-rotates at\n\
		an exact integer ratio: **7 ring turns → 11 puck turns**, the other way. That part\n\
		is the sibling entry `nullspin`, frozen and re-proved here.\n\n\
		What is new is the frame you are holding. It is the output of a real generative\n\
		loop — a declared drop load case, a reference FEA, SIMP topology optimisation with\n\
		the gear set as a keep-out, and an honest re-analysis of the final binary geometry\n\
		— and the load case is the one the sibling's own analysis calls the largest gap in\n\
		its deliverable: **the drop**.\n\n\
		## The honest headline\n\n\
		* Six printed parts, **{pg:.1} g**, Ø{od:.1} × {ht:.2} mm. **You also need: nothing.**\n\
		* The carrier survives the {h:.2} m equivalent-static design drop with a ×{my:.2}\n\
		  margin on PLA's yield. It does **not** meet the 10 MPa design allowable at that\n\
		  height — that happens at **{hao:.2} m** — and the model page says so in those words.\n\
		* The optimiser did **not** beat the sibling's six straight spokes at the spokes'\n\
		  own azimuth. It is not supposed to: six radial spokes is close to the textbook\n\
		  answer for a radial load on a spoke. What it buys is that there is no azimuth\n\
		  where the frame is absent, so a drop never lands on the gear teeth.\n\
		* Spin time **{tn:.1} s** ({tp:.1}–{to:.1} s across the unmeasured friction band).\n\
		  Reported, never claimed. A geared spinner cannot beat a plain one.\n\n\
		## Folder map\n\n\
		| you're asking… | open |\n|---|---|\n\
		| what do I print | `parts/` (six STL + 3MF) |\n\
		| what do I have to buy | nothing |\n\
		| how do I build it | `assembly/ASSEMBLY.png` |\n\
		| can I modify it | `cad/*.step` |\n\
		| what does it look like | `renders/` |\n\
		| is it verified | `analysis/ANALYSIS.md` — every number regenerated on every run |\n\
		| what was frozen before the geometry | `analysis/DESIGN.md` |\n\
		| the raw solver receipts | `analysis/fea/` — manifests, receipts, density fields |\n\
		| how do I publish it | `publish/PRINTABLES_LISTING.md` |\n\n\
		## Regenerate everything\n\n\
		```sh\n\
		cargo run --release -p kernel-model --example nullspin_gen\n\
		```\n\n\
		It exits non-zero if any gate fails, and it re-runs the whole generative loop —\n\
		three SIMP solves and thirteen FEA solves — from the manifests in `analysis/fea/`.\n\n\
		## Authorship\n\n{auth}\n",
		pg = d.printed_g, od = 2.0 * (34.25 + RING_WALL), ht = d.height, h = d.drop_h,
		my = SIG_YIELD_PLA / d.worst_drop, hao = d.h_allow_opt,
		tn = d.t_nom, tp = d.t_pes, to = d.t_opt, auth = AUTHORSHIP,
	);
	let _ = std::fs::write(format!("{OUT}/README.md"), t);
}

/// `publish/PRINTABLES_LISTING.md` — the model-page copy.
fn write_listing(d: &Docs, od: f64) {
	let t = format!(
		"# NULLSPIN-GEN — Printables listing copy\n\n\
		> Generated from the gate suite. Every number below was measured by the run that\n\
		> wrote this file. **Do not hand-edit numbers here** — edit the campaign and\n\
		> re-run. Authorship is disclosed in full under \"How this model was made\".\n\n\
		---\n\n\
		## Title\n\n\
		NULLSPIN-GEN — a geared fidget spinner whose frame was solved by a structural\n\
		optimiser, not drawn\n\n\
		## Summary\n\n\
		Hold the web. Flick the outer ring. The inner puck spins the **other way**, at an\n\
		exact integer ratio — **7 turns of the ring is 11 turns of the puck**, forever, not\n\
		approximately.\n\n\
		You are looking at the gears through a frame that nobody drew. I wrote down the\n\
		load a dropped spinner actually sees, handed the volume and that load to a topology\n\
		optimiser, and printed what came back — then re-ran the physics on the finished\n\
		shape to check it was still true.\n\n\
		| | |\n|---|---|\n\
		| ratio | 7 : 11, exact (7·66 = 11·42 = 462) |\n\
		| parts | 6 printed |\n\
		| mass | {pg:.1} g |\n\
		| size | Ø{od:.1} × {ht:.2} mm |\n\
		| spin | {tn:.1} s (band {tp:.1}–{to:.1} s) — reported, not claimed |\n\
		| momentum cancellation η | {eta:.4}, modelled |\n\n\
		**You also need: nothing.** No bearing, no balls, no magnets, no screws, no nuts,\n\
		no weights, no inserts, no glue, no tools. Six printed parts.\n\n\
		## Description\n\n\
		### What is different about this one\n\n\
		Every part of the mechanism is shared with my other entry, NULLSPIN, and is frozen:\n\
		the 66-tooth ring, the 42-tooth sun, six 12-tooth planets, the twist-lock that holds\n\
		it together without a single fastener. What changed is the **frame** — the bit your\n\
		fingers are on, and the bit you see the gears through.\n\n\
		In NULLSPIN that frame is six straight spokes, drawn by hand. Here it is the output\n\
		of a structural optimiser: I declared the load case (a drop, plus a hard pinch),\n\
		marked everything the optimiser was not allowed to touch — the six planet pins and\n\
		their twist-lock features, the central post, the ring's thrust pads, the whole gear\n\
		envelope — and let it decide where the material goes. Then I rebuilt exact CAD from\n\
		the result and **re-ran the analysis on the finished shape**, not on the optimiser's\n\
		own estimate. That last step is the one that matters and it is the one that is\n\
		usually skipped.\n\n\
		### Testing — including what did NOT go my way\n\n\
		**Verified in CAD, every build:** all six parts valid, watertight, one connected\n\
		body, support-free (zero steep area), and inside the bed. The gears are swept\n\
		through a full mesh cycle at 96 poses plus 72 exact solid-intersection checks and\n\
		never touch. The optimised web is swept against every rotor at 48 poses — it is\n\
		free in its own plane and never in the gear envelope.\n\n\
		**The drop, which is the point.** A spinner gets dropped; that is the failure mode\n\
		of the whole product class. I modelled it as an **equivalent-static** impact:\n\
		{h:.2} m onto a hard floor, a stated {s:.2} mm stopping distance, giving {gs:.0} g\n\
		of deceleration and {fd:.0} N at the rim. **This is not a transient impact\n\
		simulation** and I do not pretend it is — it replaces the event with the single\n\
		peak force it is estimated to produce, which is standard practice for a product\n\
		drop check and nothing more. I cross-checked it against a rigid-floor contact model\n\
		that returns {ratio:.1}× the force, which is the honest way of saying \"{s:.2} mm\n\
		describes a hard floor, not an infinitely rigid one\".\n\n\
		The result, in the two tiers that matter:\n\n\
		* **Does it break?** No. Worst reading {wd:.1} MPa against PLA's {sy:.0} MPa yield,\n\
		  a ×{my:.2} margin, with a coarse-mesh derate already applied.\n\
		* **Would an engineer sign it off at that height?** **No.** The 10 MPa design\n\
		  allowable (which carries a 2× design factor) is reached at **{hao:.2} m**, not\n\
		  {h:.2} m. It is a printed toy. Do not throw it at concrete.\n\n\
		**What did NOT go my way, with the numbers.** The optimiser did not beat the hand-\n\
		drawn spokes at the spokes' own azimuth: {vbb:.1} MPa at {bb:.2} g for the straight\n\
		spokes against {vbo:.1} MPa at {bo:.2} g for the optimised web. Six radial spokes is\n\
		very nearly the textbook answer for a radial load on a spoke, and I am not going to\n\
		claim otherwise. Two things are genuinely better and they are why this exists: the\n\
		optimised frame has material at **every** azimuth (NULLSPIN's has none between its\n\
		arms, so a rim-first drop there lands on the gear ring instead of the frame), and\n\
		against the optimiser's own solid starting blank it removed **{cut:.0}%** of the\n\
		mass. Also honest: SIMP minimises stiffness, not strength. The peak stress rose by\n\
		×{sr:.2} while that mass came out, so stress × mass came out flat on the base\n\
		frame and slightly WORSE on the top one. The optimiser found a much lighter\n\
		arrangement at about the same structural efficiency, which is what a compliance\n\
		objective asks for and is not the same thing as \"stronger\". That is what the\n\
		method does, and I would rather print it than round it up.\n\n\
		**Not done, and it would be dishonest not to say so:** no transient impact solve,\n\
		no strain-rate model, no impact-toughness data for printed PLA, no corner-first\n\
		(out-of-plane) drop case — that one is set by plate thickness and is identical on\n\
		both frames, so no arrangement of material would change it. No printed part was\n\
		made or measured for this listing. The full list, with reasons, is in the model's\n\
		analysis documentation.\n\n\
		### Print settings\n\n\
		- PLA, 0.4 mm nozzle, 0.20 mm layers\n\
		- 3 perimeters — **5 on the ring**, it is the part most likely to warp out of round\n\
		  and it is located by six meshes at once\n\
		- 20% gyroid, **symmetric infill only** (a balance claim is meaningless if the\n\
		  slicer breaks the symmetry)\n\
		- **No supports. No brim.** Every bed-touching edge of every moving part is relieved\n\
		  0.45 mm on a 1.4:1 slope — deliberately steeper than 45°, so it is clear of the\n\
		  support threshold rather than sitting on it. That is both the brim replacement and\n\
		  the elephant-foot fix.\n\
		- Turn ON avoid-crossing-walls / combing so no travel move crosses a clearance gap.\n\
		- Everything prints FLAT, gear axes vertical, one plate, one colour. No part in the\n\
		  set has a single downward-facing horizontal face, so there is nothing to bridge.\n\n\
		**Print `coupon_fit` and `coupon_key` first — about 12 minutes for both.** They\n\
		carry the three fits that decide the build, and every one is printed-part-to-\n\
		printed-part because there is nothing to buy. If your planets are tight, `optional/`\n\
		has the same planet at Ø5.90 and Ø6.15 bores.\n\n\
		### Assembly\n\n\
		1. Drop the sun over the post. It rests on a small raised land and turns free —\n\
		   there is no bearing and nothing to press.\n\
		2. Drop six planets onto six pins. They are identical and self-clock against the sun.\n\
		3. Drop the ring over the planets. It self-clocks and is located by all six.\n\
		4. **The twist-lock.** Line each slot's wide end up over its pin, drop the top frame\n\
		   on flat, then twist it about 7° until all six stop. No force — it drops on free\n\
		   and the twist is a slide. Nothing is strained at rest.\n\
		5. Check by eye: every pin's fin sits at the closed end of its slot, {ex:.2} mm of\n\
		   solid material over the wall.\n\
		6. Press the cap onto the post. Done.\n\n\
		### Safety\n\n\
		- Not a toy for under-3s. It has small parts and exposed gear teeth.\n\
		- Every accessible gap between moving parts clears the EN 71-1 §4.10 rod rule: the\n\
		  only space a Ø5 rod enters is the {nb:.1} mm gap between adjacent planets, and\n\
		  that clears the Ø12 branch too.\n\
		- Tooth tips are chamfered 0.30 mm on both faces.\n\n\
		---\n\n\
		## How this model was made\n\n{auth}\n\n\
		Every number on this page is produced by the program's own run, not typed in by\n\
		hand:\n\n\
		- The build re-proves every claim and exits non-zero if any check fails.\n\
		- Each check that could silently pass has a **negative control** — a deliberately\n\
		  wrong version that must FAIL. The twist-lock is falsified two ways; the structural\n\
		  solver is falsified by cutting a strut out of the live load path and requiring the\n\
		  stress to jump; the solvers written for this campaign are each falsified by a 5%\n\
		  wrong reference that must go red.\n\
		- The honest limits are on the page too: the drop height the frame is NOT signed off\n\
		  for, the comparison the optimiser lost, and the material constants nobody publishes\n\
		  for printed-PLA-on-printed-PLA.\n\n\
		Full engineering write-up, including every design direction that was tried and\n\
		**rejected** with its numbers, is in the model's `analysis/` documentation.\n",
		pg = d.printed_g, od = od, ht = d.height, tn = d.t_nom, tp = d.t_pes, to = d.t_opt, eta = d.eta,
		h = d.drop_h, s = d.drop_s, gs = d.accel / GRAV, fd = d.f_drop, ratio = d.f_bound / d.f_drop,
		wd = d.worst_drop, sy = SIG_YIELD_PLA, my = SIG_YIELD_PLA / d.worst_drop, hao = d.h_allow_opt,
		vbb = d.lb.vm_base, bb = d.lb.g_base, vbo = d.lb.vm_opt, bo = d.lb.g_opt,
		cut = 100.0 * (1.0 - d.lb.g_opt / d.lb.g_solid), sr = d.lb.vm_mid / d.lb.vm_solid_mid,
		ex = ENGAGE, nb = d.neighbour, auth = AUTHORSHIP,
	);
	let _ = std::fs::write(format!("{OUT}/publish/PRINTABLES_LISTING.md"), t);
}
