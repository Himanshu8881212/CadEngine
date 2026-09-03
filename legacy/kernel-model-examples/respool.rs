//! RESPOOL — a two-part printable reusable spool for Bambu-style 1 kg filament
//! refills (spool-less coils on an ~Ø82 × 60 mm cardboard ring).
//!
//! One part, printed twice: the two halves are IDENTICAL and join with a
//! hermaphroditic 3+3-sector bayonet inside the barrel. Three 42° tongue arcs
//! on each half drop into the other half's socket arcs (the 120° pattern is
//! self-complementary under the flip, so any of three insert positions works),
//! then a +15° twist slides each tongue's lug under the mate's pocket ceiling.
//! Retention is purely GEOMETRIC (lug under ceiling, hard end stop): no snap
//! finger or thread preload is left strained at rest, so there is nothing to
//! creep or wear out in a warm filament dryer — the two community failure
//! modes of twist-lock spools (worn detent bumps, loosening threads) are
//! designed out rather than tolerated. A ride-over detent bump forces a
//! deliberate 0.15 mm axial lift mid-twist (a tactile click), and two Ø2.1
//! witness holes through the flanges line up ONLY in the locked position — a
//! scrap of 1.75 mm filament pushed through them is a free, positive
//! secondary lock (community failure mode #1 is halves separating in the AMS).
//!
//! Envelope (researched 2026-07-28, sources in spool_system/respool/DESIGN.md):
//! flange Ø200.0 × overall width 67.0 × bore Ø55.0 — the official Bambu
//! reusable-spool envelope (AMS window: Ø197–202, width 50–68; AMS-lite bore
//! 53–58). Barrel Ø81.0 with six 0.35-proud grip ribs (envelope Ø81.7) carries
//! the refill's Ø81.5–82 cardboard core: nominal cores slide on with a kiss,
//! tight cores take a 0.1 mm cardboard crush per side — the rib bite that
//! keeps the coil from spinning on the barrel without Bambu's notch-locator
//! (works with notchless third-party refills too). The joint pockets are cut
//! into the barrel wall's INNER face only: a 1.35 mm outer skin stays
//! continuous, so the filament never sees a window it could dip into, and
//! every angular position of the mid-spool seam is backed by a tongue.
//!
//! Support-free by construction in the shipped orientation (flange down):
//! every downward face is bed, a ≤50°-from-horizontal climb, or a dead-flat
//! micro-bridge ≤ 6 mm (pocket ceilings, lug undersides) — gated by
//! `support_free_report` with steep_area < 1e-6, plus a wrong-orientation
//! negative control proving the gate bites.
//!
//! HEAT HONESTY: this file validates geometry, not chemistry. Printed in PLA
//! (Tg 60 °C, HDT 54 °C) a spool WILL soften in a ≥55 °C dryer and has warped
//! at 50 °C settings over dryer hotspots (forum reports cited in DESIGN.md);
//! Bambu's own reusable spool is ABS (70 °C) / ABS+PC (90 °C) for this
//! reason. The zero-preload joint keeps the LOCK creep-safe, but the flanges
//! are still PLA: dry PLA at ≤45–50 °C with the spool lying flat, or reprint
//! the same STL in PETG/ABS/ASA for hotter cycles. Stated in README.md;
//! never silently claimed otherwise.
//!
//! Run: cargo run --example respool -p kernel-model --release
//!   -> spool_system/respool/ (exit 1 on any FAIL)

use kernel_brep::math::{DAffine3, DMat3, DVec2, DVec3};
use kernel_brep::{
	cylinder, cuboid, difference, export_step, export_step_assembly, extrude, import_step_assembly,
	overlap_volume, tessellate_default, union, validate, volume, Mesh, Solid,
};
use kernel_core::math::Vec3;
use kernel_model::campaign::gate;
use std::f64::consts::PI;

// ---- envelope (mm) — researched values, sources in DESIGN.md -------------------
const FLANGE_R: f64 = 100.0; // Ø200 flange (AMS window 197–202)
const SPOOL_W: f64 = 67.0; // overall width (AMS window 50–68; official spool 67)
const HALF_W: f64 = SPOOL_W / 2.0; // 33.5 — the rim/butt plane of one half
const BORE_R: f64 = 27.5; // Ø55 bore (official 55 ± 0.5; AMS-lite window 53–58)
const FLANGE_T: f64 = 3.0; // flange plate thickness
const HUB_T: f64 = 3.2; // bore sleeve wall
const HUB_L: f64 = 10.0; // bore sleeve length (bearing land for rod holders)

// ---- barrel & refill interface -------------------------------------------------
const COIL_ID_NOM: f64 = 82.0; // Bambu refill cardboard core ID (nominal)
const COIL_ID_TIGHT: f64 = 81.5; // tightest reported core ID
const COIL_W: f64 = 60.0; // refill coil width (59.7 measured, 60 spec)
const RO: f64 = 40.5; // barrel outer radius (Ø81.0 body)
const WALL: f64 = 3.2;
const RI: f64 = RO - WALL; // 37.3 barrel inner radius
const RIB_PROUD: f64 = 0.35; // grip ribs: Ø81.7 envelope — kiss at Ø82 core,
const RIB_W: f64 = 3.0; // 0.1/side cardboard crush at Ø81.5 (intentional)

// ---- bayonet joint: radial stack -----------------------------------------------
const C_R: f64 = 0.25; // tongue-outer ↔ mate-wall-inner radial clearance
const T_T: f64 = 2.4; // tongue thickness
const R_TO: f64 = RI - C_R; // 37.05 tongue outer radius
const R_BI: f64 = R_TO - T_T; // 34.65 tongue/band inner radius
const LUG_H: f64 = 1.4; // lug radial protrusion beyond the tongue face
const POCKET_DEPTH: f64 = LUG_H + 0.45; // pocket floor at RI + 1.85
const SKIN: f64 = RO - (RI + POCKET_DEPTH); // 1.35 continuous outer skin over pockets

// ---- bayonet joint: axial stack (local frame, flange bed at z = 0) -------------
const Z_BAND0: f64 = 23.3; // tongue-anchor band ring bottom
const Z_BAND1: f64 = 25.3; // band top = tongue root
const CONE50: f64 = 1.191_753_592_594_209_7; // tan 50° — band underside climb
// Socket cutter floor: EXACTLY flush with the band top. A floor 0.05 below it
// (the first attempt) sliced 0.125 × 0.05 mm terrace slivers into the groove
// floor that later cutters kept crossing — whether the result stayed
// stitchable then depended on revolve facet phase (needle-face class, cousin
// of FRICTION #23). Flush-coincident cutter faces cancel exactly in the
// hardened coplanar pipeline and leave NO extra faces at all.
const Z_CUT: f64 = Z_BAND1;
const Z_TIP: f64 = 2.0 * HALF_W - (Z_CUT + 0.20); // 41.5 tongue tip (0.20 land gap)
// Channel/arm pocket floor: 0.1 EMBEDDED below the band top (the design
// guide's preferred pattern), with the cutters' inner radius pulled past the
// band's inner face into open air — so no cutter edge or face ever lies
// INSIDE a coincident-plane overlap (the partial-coplanar-forest corner that
// broke channel2 when the floors were exactly flush).
const Z_PKT_FLOOR: f64 = Z_CUT - 0.1; // 25.2
const R_PKT_IN: f64 = R_BI - 0.5; // 34.15 — in bore air, clear of every face
const Z_LUG_BOT: f64 = Z_TIP - 2.2; // 39.35 lug flat underside (1.4 mm micro-bridge)
const Z_LUG_TOP: f64 = Z_TIP - 0.8; // 40.75 lug outer face top; 29.7° up-chamfer to tip
const CEIL_CLR: f64 = 0.30; // lug retention face ↔ pocket ceiling axial clearance
const Z_ARM_CEIL: f64 = 2.0 * HALF_W - Z_LUG_BOT + CEIL_CLR; // 27.95 pocket ceiling
const BUMP_LIFT: f64 = 0.15; // detent: forced axial lift riding the bump

// ---- bayonet joint: angular layout (degrees; 120° pattern) ---------------------
const TONGUE_A0: f64 = 9.0; // tongue arcs [9°, 51°] + 120k (42° wide)
const TONGUE_A1: f64 = 51.0;
const LUG_A0: f64 = 11.0; // lug arcs [11°, 19°] + 120k, near the leading edge
const LUG_A1: f64 = 19.0;
const CH_A0: f64 = 340.3; // entry channel [340.3°, 349.7°] + 120k: the flipped
const CH_A1: f64 = 349.7; // lug [341°, 349°] descends with 0.7° side clearance
const ARM_A1: f64 = 364.7; // pocket arm end = hard overtwist stop
const PSI_LOCK: f64 = 15.0; // twist from insert to lock
// Facet-alignment law (established by bisecting this part's own build chain,
// 2026-07-28 — the design-guide §7.4 "least-margin corner" made concrete):
// geometry booleaned into a revolve must respect its facet grid. Observed on
// the pre-hardened variants of this joint: a small embedded union straddling
// a facet-boundary meridian cracked the default tessellation (valid B-rep,
// non-watertight mesh — bump at 113.05° astride 112.5° at SEG=128), and a
// cutter side plane lying exactly ON a meridian degenerated the arrangement
// outright (sector cut [171°,249°] at SEG=120, pitch 3°). A minimal
// tube+pocket+bump repro does NOT crack — the failures needed this part's
// fuller face neighbourhood — so the defence is design-side and layered:
// SEG=126 (divisible by 3 ⇒ the three +120° bump copies share one facet
// phase; pitch 2.857° puts no design angle on the grid), bumps centred
// mid-facet, pocket floors embedded 0.1 below the band top, and pocket
// cutters' inner faces pulled into open air (R_PKT_IN).
const FACET: f64 = 360.0 / SEG as f64;
const BUMP_AC: f64 = 123.5 * FACET; // 352.857° — dead centre of facet 123

// ---- flange furniture ----------------------------------------------------------
const WIN_R: f64 = 66.0; // 6 × Ø30 windows at 60k+37.5° — the 7.5 mod 15 phase
const WIN_HOLE_R: f64 = 15.0; // makes opposite windows line up when locked
const PIN_R: f64 = 90.0; // Ø2.1 witness/lock-pin holes at 7.5° and 187.5°:
const PIN_HOLE_R: f64 = 1.05; // they align through both flanges ONLY at lock
const SLOT_A: f64 = 175.0; // filament-tail tuck slot through the barrel wall

const SEG: usize = 126; // revolve segments — see the facet-alignment law at
                        // BUMP_AC; chord error ~0.031 mm at Ø200, an order of
                        // magnitude under every joint clearance
const PLA_G_PER_MM3: f64 = 0.00124;

// ---- PLA allowables for the strength/thermal arithmetic ------------------------
// Base tensile 35 MPa (low end of published PLA data; Bambu PLA Basic TDS lists
// higher) × 0.6 layer-adhesion knockdown × 0.5 design factor → 10.5 → 10 MPa
// design tension at room temperature; shear taken as 0.58·σ. The HOT tier is
// the honest one for the dryer question: at 50 °C PLA sits just under its own
// HDT (54 °C @ 1.8 MPa, Bambu TDS), so the sustained allowable is cut to
// 2.5 MPa tension / 1.5 MPa shear — roughly "what HDT itself implies", NOT a
// hand-wave. Every load case below must clear the HOT allowable with margin,
// because a spool full of filament lives inside a 45–50 °C dryer for hours.
use kernel_model::materials::pla::{
	creep_allowable_mpa, creep_shear_allowable_mpa, SIG_ALLOW_HOT, SIG_ALLOW_RT, TAU_ALLOW_HOT, TAU_ALLOW_RT,
};
const COIL_KG: f64 = 1.0; // full refill
const HALF_KG_EST: f64 = 0.132; // one printed half (engine volume × PLA ρ; gated ±20%)

// ---- tiny helpers --------------------------------------------------------------

fn v(x: f64, y: f64, z: f64) -> DVec3 {
	DVec3::new(x, y, z)
}
fn tr(x: f64, y: f64, z: f64) -> DAffine3 {
	DAffine3::from_translation(v(x, y, z))
}
fn rotz(deg: f64) -> DAffine3 {
	DAffine3::from_rotation_z(deg.to_radians())
}

/// Force a polygon CCW (extrude() wants CCW).
fn ccw(mut p: Vec<DVec2>) -> Vec<DVec2> {
	let a2: f64 = p.windows(2).map(|w| w[0].x * w[1].y - w[1].x * w[0].y).sum::<f64>()
		+ (p[p.len() - 1].x * p[0].y - p[0].x * p[p.len() - 1].y);
	if a2 < 0.0 {
		p.reverse();
	}
	p
}

/// Arc polyline at radius r from a0 to a1 degrees (inclusive), ~step° facets.
fn push_arc(p: &mut Vec<DVec2>, r: f64, a0: f64, a1: f64, step: f64) {
	let n = (((a1 - a0).abs() / step).ceil() as usize).max(1);
	for i in 0..=n {
		let a = (a0 + (a1 - a0) * i as f64 / n as f64).to_radians();
		p.push(DVec2::new(r * a.cos(), r * a.sin()));
	}
}

/// Pie-wedge prism: axis-centred sector of radius r_out, angles [a0, a1] deg,
/// z ∈ [z0, z1]. Cuts everything inside its radius over the sector.
fn pie_prism(r_out: f64, a0: f64, a1: f64, z0: f64, z1: f64) -> Solid {
	let mut pts = vec![DVec2::new(0.0, 0.0)];
	push_arc(&mut pts, r_out, a0, a1, 2.0);
	extrude(&ccw(pts), z1 - z0).transformed(tr(0.0, 0.0, z0))
}

/// Annular-sector prism: radii [r_in, r_out], angles [a0, a1] deg, z ∈ [z0, z1].
fn ring_prism(r_in: f64, r_out: f64, a0: f64, a1: f64, z0: f64, z1: f64) -> Solid {
	let mut pts = Vec::new();
	push_arc(&mut pts, r_out, a0, a1, 1.0);
	push_arc(&mut pts, r_in, a1, a0, 1.0);
	extrude(&ccw(pts), z1 - z0).transformed(tr(0.0, 0.0, z0))
}

fn mesh_aabb(m: &Mesh) -> (Vec3, Vec3) {
	let mut lo = Vec3::splat(f32::INFINITY);
	let mut hi = Vec3::splat(f32::NEG_INFINITY);
	for p in &m.positions {
		lo = lo.min(*p);
		hi = hi.max(*p);
	}
	(lo, hi)
}

/// Transform a mesh's positions by an affine map (cheap posing for distance checks).
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
	dst.positions.extend_from_slice(&src.positions);
	dst.indices.extend(src.indices.iter().map(|i| i + base));
}

/// Pose of the SECOND (identical) half: flip about X through the rim plane,
/// twist by psi about the spool axis, float dz above nominal closure.
/// Local z=0 (its bed face) lands at world z = SPOOL_W + dz.
fn pose_b(psi_deg: f64, dz: f64) -> DAffine3 {
	tr(0.0, 0.0, SPOOL_W + dz) * rotz(psi_deg) * DAffine3::from_rotation_x(PI)
}

// ---- geometry ------------------------------------------------------------------

/// The revolved half-section (r, z): flange + hub sleeve + barrel wall + the
/// full-360° tongue ring on its anchor band. Every downward-facing segment is
/// bed, ≥50° from horizontal, or dead flat (micro-bridge) — chamfers are drawn
/// steeper than 45° on down-facing edges so the support gate stays exact-zero.
fn half_profile() -> Vec<DVec2> {
	let pts: [(f64, f64); 24] = [
		(BORE_R + 0.6, 0.0),                       // bore bottom chamfer foot (55° climb)
		(FLANGE_R - 0.7, 0.0),                     // bed face
		(FLANGE_R, 1.0),                           // rim lower chamfer (55° from horizontal)
		(FLANGE_R, 2.0),                           // rim land
		(FLANGE_R - 1.0, FLANGE_T),                // rim upper chamfer (up-facing)
		(RO, FLANGE_T),                            // flange top in to the barrel
		(RO, HALF_W - 0.6),                        // barrel outer face
		(RO - 0.6, HALF_W),                        // rim-butt edge chamfer (up-facing)
		(RI, HALF_W),                              // rim butt face
		(RI, Z_BAND1),                             // wall inner face down to the groove
		(R_TO, Z_BAND1),                           // groove floor (mate-tongue slot floor)
		(R_TO, Z_TIP - 0.75),                      // tongue outer face
		(R_TO - 0.6, Z_TIP),                       // tip outer chamfer (up-facing)
		(R_BI + 0.6, Z_TIP),                       // tip flat (1.2 wide)
		(R_BI, Z_TIP - 0.75),                      // tip inner chamfer (51.3° down-face)
		(R_BI, Z_BAND0),                           // tongue + band inner face
		(RI, Z_BAND0 - T_T * CONE50 - C_R * CONE50), // 50° cone under the band ring
		(RI, FLANGE_T),                            // barrel bore down to the flange
		(BORE_R + HUB_T, FLANGE_T),                // flange top in to the hub sleeve
		(BORE_R + HUB_T, HUB_L - 0.8),             // hub outer face
		(BORE_R + HUB_T - 0.8, HUB_L),             // hub top chamfer (up-facing)
		(BORE_R + 0.6, HUB_L),                     // hub top face
		(BORE_R, HUB_L - 0.6),                     // bore top chamfer (up-facing)
		(BORE_R, 0.85),                            // bore face; closes via 55° foot chamfer
	];
	pts.iter().map(|&(r, z)| DVec2::new(r, z)).collect()
}

/// One lug: (r,z) cross-section swept along the tangent chord, mid-plane at
/// `a_mid` degrees. Flat underside (1.4 mm bridge when printed), vertical
/// outer face, 29.7° up-chamfer converging on the tongue tip — after the flip
/// the flat face is the UP-facing retention face pressed square against the
/// mate's pocket ceiling (no wedge action that could cam the tongue inward).
fn lug(a_mid: f64) -> Solid {
	let prof = vec![
		DVec2::new(R_TO - 0.3, Z_LUG_BOT), // embedded 0.3 into the tongue
		DVec2::new(R_TO + LUG_H, Z_LUG_BOT),
		DVec2::new(R_TO + LUG_H, Z_LUG_TOP),
		DVec2::new(R_TO, Z_TIP),
		DVec2::new(R_TO - 0.3, Z_TIP),
	];
	let r_mid = R_TO + LUG_H / 2.0;
	let chord = 2.0 * r_mid * ((LUG_A1 - LUG_A0) / 2.0).to_radians().sin();
	let a = a_mid.to_radians();
	let rhat = v(a.cos(), a.sin(), 0.0);
	let that = v(-a.sin(), a.cos(), 0.0);
	// X→radial, Y→world-Z, Z→(−tangent): a proper rotation (det +1); the prism
	// starts half a chord along +tangent so it straddles the mid-plane.
	let m = DAffine3::from_mat3_translation(DMat3::from_cols(rhat, DVec3::Z, -that), that * (chord / 2.0));
	extrude(&ccw(prof), chord).transformed(m)
}

/// Detent bump on the pocket floor between the insert and lock positions.
/// The mate lug's underside is its 29.7° tip chamfer (a sheet rising with
/// radius), so a flat-topped bump would either never touch it or jam at its
/// inner corner. The bump's top is therefore SLOPED PARALLEL to that chamfer,
/// offset by exactly BUMP_LIFT: riding over it forces a uniform 0.15 mm lift
/// (the tactile click) with full-width contact — no jam, no point-wear. Base
/// is sunk 0.3 below the floor (embedment, not a coincident-face union); the
/// end faces are vertical and the top faces up, so it adds zero overhangs.
fn bump(a_mid: f64) -> Solid {
	let r0 = RI + 0.05; // just inside the pocket, clear of the wall face
	let r1 = R_TO + LUG_H + 0.10; // past the lug's outer face
	let slope = (Z_TIP - Z_LUG_TOP) / LUG_H; // the lug chamfer's dz/dr
	// mate chamfer sheet height at dz=0: z(r) = (2·HALF_W − Z_TIP) + slope·(r − R_TO)
	let zc = |r: f64| (2.0 * HALF_W - Z_TIP) + slope * (r - R_TO) + BUMP_LIFT;
	let prof = vec![
		DVec2::new(r0, Z_PKT_FLOOR - 0.3),
		DVec2::new(r1, Z_PKT_FLOOR - 0.3),
		DVec2::new(r1, zc(r1)),
		DVec2::new(r0, zc(r0)),
	];
	let chord = 1.2; // ±0.91° at the pocket radius vs the ±1.43° facet
	                 // half-pitch: 0.5° clear of both meridians (see BUMP_AC)
	let a = a_mid.to_radians();
	let rhat = v(a.cos(), a.sin(), 0.0);
	let that = v(-a.sin(), a.cos(), 0.0);
	// X→radial, Y→world-Z, Z→(−tangent), det +1; straddles the mid-plane.
	let m = DAffine3::from_mat3_translation(DMat3::from_cols(rhat, DVec3::Z, -that), that * (chord / 2.0));
	extrude(&ccw(prof), chord).transformed(m)
}

/// The revolved body + every bayonet feature — the complete JOINT geometry.
/// This is the solid the expensive posed-boolean gates run on: the flange
/// furniture added later (windows, pin holes, tail slot, marks — all cuts
/// outside the joint band) and the grip ribs (r ≥ 40.2, z ≤ 31.5, shown by
/// the insertion sweep to never approach the mate) cannot change any joint
/// interaction, so gating on this solid is exact for the joint and strictly
/// conservative for "no interpenetration" (it has a superset of material).
/// One full-fidelity locked-pose boolean on the finished part seals the claim.
fn build_joint_half() -> Solid {
	let mut s = kernel_brep::revolve(&half_profile(), SEG);
	// 1) carve the tongue ring down to three 42° tongues (socket sectors open)
	for k in 0..3 {
		let a0 = TONGUE_A1 + 120.0 * k as f64;
		s = difference(&s, &pie_prism(RI - 0.125, a0, a0 + 120.0 - (TONGUE_A1 - TONGUE_A0), Z_CUT, Z_TIP + 1.0));
	}
	// 2) lugs on the three tongues
	for k in 0..3 {
		s = union(&s, &lug((LUG_A0 + LUG_A1) / 2.0 + 120.0 * k as f64));
	}
	// 3) entry channels (through the rim) + pocket arms (blind, ceiling stays)
	for k in 0..3 {
		let o = 120.0 * k as f64;
		s = difference(&s, &ring_prism(R_PKT_IN, RI + POCKET_DEPTH, CH_A0 + o, CH_A1 + o, Z_PKT_FLOOR, HALF_W + 1.0));
		s = difference(&s, &ring_prism(R_PKT_IN, RI + POCKET_DEPTH, CH_A0 + o, ARM_A1 + o, Z_PKT_FLOOR, Z_ARM_CEIL));
	}
	// 4) detent bumps
	for k in 0..3 {
		s = union(&s, &bump(BUMP_AC + 120.0 * k as f64));
	}
	s
}

/// The shipped spool half: joint geometry + coil ribs + flange furniture.
/// Printed exactly as modelled: flange on the bed.
fn build_half(joint: &Solid) -> Solid {
	let mut s = joint.clone();
	// 5) coil grip ribs (embedded 0.3 into the barrel, stop 2 mm short of the rim)
	for k in 0..6 {
		let rib = cuboid(v(-RIB_W / 2.0, RO - 0.3, 2.7), v(RIB_W / 2.0, RO + RIB_PROUD, HALF_W - 2.0));
		s = union(&s, &rib.transformed(rotz(30.0 + 60.0 * k as f64)));
	}
	// 6) flange windows — aligned across the locked spool (7.5 mod 15 phase)
	for k in 0..6 {
		let a = (37.5 + 60.0 * k as f64).to_radians();
		let c = v(WIN_R * a.cos(), WIN_R * a.sin(), -1.0);
		s = difference(&s, &cylinder(c, DVec3::Z, WIN_HOLE_R, FLANGE_T + 2.0, 96));
	}
	// 7) witness / lock-pin holes (align at lock only) — double as tail-park holes
	for a_deg in [7.5_f64, 187.5] {
		let a = a_deg.to_radians();
		let c = v(PIN_R * a.cos(), PIN_R * a.sin(), -1.0);
		s = difference(&s, &cylinder(c, DVec3::Z, PIN_HOLE_R, FLANGE_T + 2.0, 32));
	}
	// 7b) Ø3.5 filament-tail parking holes at r=96 (the official spool's pattern);
	// the 7.5-mod-15 phase makes the pair line up across the locked spool too
	for a_deg in [97.5_f64, 277.5] {
		let a = a_deg.to_radians();
		let c = v(96.0 * a.cos(), 96.0 * a.sin(), -1.0);
		s = difference(&s, &cylinder(c, DVec3::Z, 1.75, FLANGE_T + 2.0, 32));
	}
	// 8) filament-tail tuck slot through the barrel wall near the flange
	let slot = cuboid(v(-1.1, RI - 0.8, 4.5), v(1.1, RO + 0.8, 9.0)).transformed(rotz(SLOT_A));
	s = difference(&s, &slot);
	// 9) bed-face insert/lock marks: ▲ at 0° (insert reference), ● at +15° (lock)
	let tri = extrude(&ccw(vec![DVec2::new(93.0, -2.2), DVec2::new(93.0, 2.2), DVec2::new(96.8, 0.0)]), 1.2)
		.transformed(tr(0.0, 0.0, -0.55));
	s = difference(&s, &tri);
	let a = PSI_LOCK.to_radians();
	let dot = cylinder(v(95.0 * a.cos(), 95.0 * a.sin(), -0.55), DVec3::Z, 1.6, 1.2, 32);
	difference(&s, &dot)
}

/// Spacing shim: a 1.0 mm ring slipped over the barrel to take up axial slack
/// when a coil runs narrower than 60 mm (community failure mode #4: filament
/// diving into the coil↔flange side gap). ID clears the rib envelope.
fn build_shim() -> Solid {
	let prof = vec![
		DVec2::new(41.05, 0.0),
		DVec2::new(60.0, 0.0),
		DVec2::new(60.0, 1.0),
		DVec2::new(41.05, 1.0),
	];
	kernel_brep::revolve(&prof, 128)
}

/// Barrel fit coupon: a 15 mm slice of the ribbed barrel — a 20-minute print
/// to verify the refill core slides on before committing to the full halves.
fn build_coupon_core() -> Solid {
	let prof = vec![
		DVec2::new(RI, 0.0),
		DVec2::new(RO, 0.0),
		DVec2::new(RO, 15.0),
		DVec2::new(RI, 15.0),
	];
	let mut s = kernel_brep::revolve(&prof, SEG);
	for k in 0..6 {
		let rib = cuboid(v(-RIB_W / 2.0, RO - 0.3, 0.3), v(RIB_W / 2.0, RO + RIB_PROUD, 15.0));
		s = union(&s, &rib.transformed(rotz(30.0 + 60.0 * k as f64)));
	}
	s
}

/// Lock fit coupon: the ±60° slice of the joint band around one tongue and one
/// socket. Two prints flip-mate exactly like the full halves (one lug engages)
/// — a fast tolerance check of the bayonet before the big print. Cut from the
/// joint solid (no ribs/windows: the mate never touches those anyway).
fn build_coupon_lock(joint: &Solid) -> Solid {
	kernel_brep::intersection(joint, &pie_prism(110.0, -60.0, 60.0, 19.0, Z_TIP + 1.5))
}

// ---- gates ---------------------------------------------------------------------

fn emit(dir: &str, name: &str, s: &Solid, drop_to_bed: bool) -> bool {
	let val = validate(s);
	let mut printed = s.clone();
	if drop_to_bed {
		let zmin = tessellate_default(&printed)
			.positions
			.iter()
			.map(|p| p.z as f64)
			.fold(f64::INFINITY, f64::min);
		printed = printed.transformed(tr(0.0, 0.0, -zmin));
	}
	let mesh_p = tessellate_default(&printed);
	let rep = mesh_p.support_free_report(Vec3::Z, 45.0, 0.3);
	let (lo, hi) = mesh_aabb(&mesh_p);
	let ext = hi - lo;
	let fits = ext.x <= 250.0 && ext.y <= 250.0 && ext.z <= 220.0;
	let wt = mesh_p.is_watertight();
	let vol = volume(s).abs();
	let ok = val.is_valid() && wt && rep.steep_area < 1e-6 && rep.max_bridge_span <= 6.0 && fits;
	let _ = std::fs::write(format!("spool_system/respool/{dir}/{name}.stl"), mesh_p.to_stl_binary());
	let _ = mesh_p.write_3mf(format!("spool_system/respool/{dir}/{name}.3mf"));
	println!(
		"  {name:16} valid={:5} wt={wt:5} steep={:9.4} mm²  bridge≤{:4.1}  {:4.0} g  {:7.0} mm³  {}",
		val.is_valid(),
		rep.steep_area,
		rep.max_bridge_span,
		vol * PLA_G_PER_MM3,
		vol,
		if ok { "OK" } else { "<<< FAIL" }
	);
	ok
}

fn main() {
	// Campaign runs always contribute to the Level-1 flywheel (telemetry + friction capture).
	kernel_core::telemetry::enable();
	// parts/ holds THE spool part (one file, print it twice); optional/ holds
	// the pre-flight fit coupons and the narrow-coil shim — helpers, not spool
	// pieces. The spool is a 2-piece assembly of two identical halves.
	let _ = std::fs::create_dir_all("spool_system/respool/parts");
	let _ = std::fs::create_dir_all("spool_system/respool/cad");
	let _ = std::fs::create_dir_all("spool_system/respool/analysis");
	let _ = std::fs::create_dir_all("spool_system/respool/assembly/scene");
	let _ = std::fs::create_dir_all("spool_system/respool/optional");
	let _ = std::fs::create_dir_all("spool_system/respool/assembly/scene");
	println!("RESPOOL twist-lock refill spool — parts (STL+3MF in print orientation):\n");

	let joint = build_joint_half();
	let half = build_half(&joint);
	let shim = build_shim();
	let c_core = build_coupon_core();
	let c_lock = build_coupon_lock(&joint);

	let mut ok = true;
	ok &= emit("parts", "spool_half", &half, false);
	ok &= emit("optional", "shim_1.0mm", &shim, false);
	ok &= emit("optional", "coupon_core", &c_core, false);
	ok &= emit("optional", "coupon_lock", &c_lock, true);

	// ---- negative control: the support gate must BITE in a wrong orientation
	let wrong = tessellate_default(&half.transformed(DAffine3::from_rotation_x(PI / 2.0)))
		.support_free_report(Vec3::Z, 45.0, 0.3);
	gate(
		"A-PRINT NC: half audited flange-vertical (steep must jump)",
		wrong.steep_area > 2000.0,
		format!("steep {:8.0} mm²", wrong.steep_area),
		&mut ok,
	);

	// ---- envelope: the researched AMS / holder window ---------------------------
	println!("\nenvelope + refill interface:");
	let m_half = tessellate_default(&half);
	let (lo, hi) = mesh_aabb(&m_half);
	gate(
		"flange Ø == 200.0 (AMS window 197–202)",
		(hi.x as f64 - lo.x as f64 - 200.0).abs() < 0.1 && (hi.y as f64 - lo.y as f64 - 200.0).abs() < 0.1,
		format!("Ø {:6.2}", hi.x as f64 - lo.x as f64),
		&mut ok,
	);
	let m_b_locked = mesh_posed(&m_half, pose_b(PSI_LOCK, 0.0));
	let (_, hib) = mesh_aabb(&m_b_locked);
	gate(
		"assembled width == 67.0 (AMS window 50–68)",
		(hib.z as f64 - 67.0).abs() < 0.05 && lo.z >= -1e-4,
		format!("w {:6.2}", hib.z as f64),
		&mut ok,
	);
	// (window bounds chosen so the bore face's own facet corners at z 0.85/9.4
	// are sampled — mesh vertices only exist at feature corners)
	let bore_r_meas = m_half
		.positions
		.iter()
		.filter(|p| p.z > 0.4 && (p.z as f64) < HUB_L - 0.4)
		.map(|p| (p.x as f64).hypot(p.y as f64))
		.fold(f64::INFINITY, f64::min);
	gate(
		"bore Ø == 55.0 (AMS-lite window 53–58)",
		(bore_r_meas * 2.0 - 55.0).abs() < 0.1,
		format!("Ø {:6.2}", bore_r_meas * 2.0),
		&mut ok,
	);
	// barrel + rib envelope, measured over the coil seat band
	// (mesh vertices only exist at feature corners: the window must contain the
	// rib boxes' top corners at z 31.5 while excluding flange verts (z ≤ 3.0)
	// and the rim chamfer (z ≥ 32.9))
	let rib_env = m_half
		.positions
		.iter()
		.filter(|p| p.z > 3.2 && (p.z as f64) < HALF_W - 1.6)
		.map(|p| (p.x as f64).hypot(p.y as f64))
		.fold(0.0_f64, f64::max);
	gate(
		"rib envelope Ø81.7: kiss at Ø82 core, 0.1/side at Ø81.5",
		(rib_env * 2.0 - (2.0 * RO + 2.0 * RIB_PROUD)).abs() < 0.1
			&& rib_env * 2.0 < COIL_ID_NOM - 0.2
			&& rib_env * 2.0 > COIL_ID_TIGHT,
		format!("Ø {:6.2}", rib_env * 2.0),
		&mut ok,
	);
	gate(
		"coil width seat: inner span 61.0 vs coil 60.0 (+1 shim takes slack)",
		(SPOOL_W - 2.0 * FLANGE_T - COIL_W - 1.0).abs() < 0.01,
		format!("span {:4.1}", SPOOL_W - 2.0 * FLANGE_T),
		&mut ok,
	);
	gate(
		"outer skin over lock pockets (filament never sees the lock)",
		SKIN >= 1.2,
		format!("skin {SKIN:4.2}"),
		&mut ok,
	);

	// refill core gauges: nominal Ø82 tube must ride free over the ribs; the
	// tight Ø81.5 tube overlaps ONLY the rib crowns (intentional cardboard bite)
	let gauge = |id: f64| -> Solid {
		let prof = vec![
			DVec2::new(id / 2.0, FLANGE_T + 0.5),
			DVec2::new(id / 2.0 + 2.5, FLANGE_T + 0.5),
			DVec2::new(id / 2.0 + 2.5, HALF_W - 2.5),
			DVec2::new(id / 2.0, HALF_W - 2.5),
		];
		kernel_brep::revolve(&prof, SEG)
	};
	let g_nom = gauge(COIL_ID_NOM);
	let d_nom = m_half.min_distance(&tessellate_default(&g_nom));
	let ovl_nom = overlap_volume(&half, &g_nom);
	gate(
		"Ø82.0 core gauge rides the ribs free",
		d_nom >= 0.10 && matches!(ovl_nom, Some(vv) if vv.abs() < 0.05),
		format!("gap {d_nom:5.3}"),
		&mut ok,
	);
	let g_tight = gauge(COIL_ID_TIGHT);
	let ovl_tight = overlap_volume(&half, &g_tight).unwrap_or(f64::NAN);
	gate(
		"Ø81.5 core gauge bites ribs only (5–80 mm³ crush)",
		(5.0..=80.0).contains(&ovl_tight),
		format!("crush {ovl_tight:5.1} mm³"),
		&mut ok,
	);
	let d_shim = tessellate_default(&shim).min_distance(&tessellate_default(&build_coupon_core()));
	gate(
		"shim ID 82.1 clears the rib envelope",
		d_shim >= 0.08,
		format!("gap {d_shim:5.3}"),
		&mut ok,
	);

	// ---- the bayonet, machine-proved on posed identical halves ------------------
	// Mesh distance sweeps run on the SHIPPED part; boolean overlap poses run on
	// the joint solid (see build_joint_half doc — exact in the joint band,
	// conservative elsewhere), and the locked pose is re-proved full-vs-full.
	println!("\nbayonet kinematics (half B = same part, flipped; ψ = twist, dz = float):");
	let posed_joint = |psi: f64, dz: f64| joint.transformed(pose_b(psi, dz));
	let posed_mesh = |psi: f64, dz: f64| mesh_posed(&m_half, pose_b(psi, dz));

	// (a) straight-drop insertion sweep at ψ=0
	let mut drop_min = f64::INFINITY;
	let mut dz = 12.0;
	while dz >= 0.6 {
		drop_min = drop_min.min(m_half.min_distance(&posed_mesh(0.0, dz)));
		dz -= 0.6;
	}
	gate(
		"insertion sweep ψ=0, dz 12→0.6 (20 poses) stays clear",
		drop_min >= 0.10,
		format!("min {drop_min:5.3}"),
		&mut ok,
	);
	// (b) fully dropped, pre-twist
	let ovl_seat = overlap_volume(&joint, &posed_joint(0.0, 0.10)).unwrap_or(f64::NAN);
	gate(
		"seated at ψ=0 (dz 0.10): no interpenetration",
		ovl_seat.abs() < 0.05,
		format!("ovl {ovl_seat:6.3}"),
		&mut ok,
	);
	// (c) twist sweep riding the detent window (dz = 0.22 ∈ [bump 0.15, ceiling 0.30])
	let tw_pairs: Vec<(&Solid, Solid)> = (0..=5)
		.map(|k| (&joint, posed_joint(PSI_LOCK * k as f64 / 5.0, 0.22)))
		.collect();
	let mut tw_worst = 0.0_f64;
	let mut tw_refused = false;
	for o in kernel_brep::overlap_volume_many(&tw_pairs) {
		match o {
			Some(o) => tw_worst = tw_worst.max(o.abs()),
			None => tw_refused = true,
		}
	}
	gate(
		"twist sweep ψ 0→15 at dz 0.22 (6 poses): clean pass-through",
		!tw_refused && tw_worst < 0.05,
		format!("worst {tw_worst:6.3}"),
		&mut ok,
	);
	// (d) detent proof: un-lifted mid-twist is BLOCKED by the bump…
	let ovl_bump = overlap_volume(&joint, &posed_joint(7.0, 0.02)).unwrap_or(f64::NAN);
	gate(
		"detent bites: ψ=7 without lift hits the bump",
		ovl_bump > 0.05,
		format!("ovl {ovl_bump:6.3}"),
		&mut ok,
	);
	// (e) locked pose — full-fidelity: the SHIPPED halves, ribs and all
	let ovl_lock = overlap_volume(&half, &half.transformed(pose_b(PSI_LOCK, 0.10))).unwrap_or(f64::NAN);
	gate(
		"locked at ψ=15 (dz 0.10): shipped halves, no interpenetration",
		ovl_lock.abs() < 0.05,
		format!("ovl {ovl_lock:6.3}"),
		&mut ok,
	);
	// (f) RETENTION: pulling the locked halves apart shears three lug sets
	let ovl_pull = overlap_volume(&joint, &posed_joint(PSI_LOCK, 0.75)).unwrap_or(f64::NAN);
	gate(
		"retention: locked + 0.75 pull ⇒ lugs bury into ceilings",
		ovl_pull >= 2.0,
		format!("ovl {ovl_pull:6.2} mm³"),
		&mut ok,
	);
	// …and the unlocked position pulls apart freely (the retention is the twist)
	let free_pull = m_half.min_distance(&posed_mesh(0.0, 0.75));
	gate(
		"counter-proof: at ψ=0 the same pull lifts off freely",
		free_pull >= 0.10,
		format!("min {free_pull:5.3}"),
		&mut ok,
	);
	// (g) overtwist: the arm end is a hard stop just past lock
	let ovl_stop = overlap_volume(&joint, &posed_joint(18.0, 0.22)).unwrap_or(f64::NAN);
	gate(
		"overtwist ψ=18 hits the arm-end stop",
		ovl_stop > 0.30,
		format!("ovl {ovl_stop:6.2}"),
		&mut ok,
	);
	// (h) wrong-angle drop cannot assemble
	let ovl_wrong = overlap_volume(&joint, &posed_joint(30.0, 0.5)).unwrap_or(f64::NAN);
	gate(
		"NC: dropping at ψ=30 collides tongue-on-tongue",
		ovl_wrong > 30.0,
		format!("ovl {ovl_wrong:6.1}"),
		&mut ok,
	);

	// ---- witness holes: align at lock, misalign at insert -----------------------
	println!("\nlock witness / secondary pin:");
	let pa = v(PIN_R * 7.5_f64.to_radians().cos(), PIN_R * 7.5_f64.to_radians().sin(), 0.0);
	let pb_lock = pose_b(PSI_LOCK, 0.0).transform_point3(pa);
	let pb_ins = pose_b(0.0, 0.0).transform_point3(pa);
	let off_lock = (pb_lock.x - pa.x).hypot(pb_lock.y - pa.y);
	let off_ins = (pb_ins.x - pa.x).hypot(pb_ins.y - pa.y);
	gate(
		"Ø2.1 pin holes coaxial at lock (filament scrap = positive lock)",
		off_lock < 0.01,
		format!("off {off_lock:5.3}"),
		&mut ok,
	);
	gate(
		"…and visibly misaligned when not locked",
		off_ins > 5.0,
		format!("off {off_ins:5.1}"),
		&mut ok,
	);

	// ---- as-printed fit robustness ----------------------------------------------
	// The fit must survive a real printer, not just nominal CAD: (a) a hand
	// bringing the halves together crooked, (b) per-part dimensional error.
	println!("\nas-printed fit robustness:");
	let mut mis_min = f64::INFINITY;
	let mut dzm = 8.0;
	while dzm >= 1.0 {
		let pose = tr(0.15, 0.0, 0.0) * pose_b(0.0, dzm);
		mis_min = mis_min.min(m_half.min_distance(&mesh_posed(&m_half, pose)));
		dzm -= 1.0;
	}
	gate(
		"insertion tolerates 0.15 mm lateral misalignment (8 poses)",
		mis_min >= 0.04,
		format!("min {mis_min:5.3}"),
		&mut ok,
	);
	// Tolerance stack: every mating surface pair loses 2·e of clearance when
	// each part errs e toward tight. e=0.05 = a calibrated printer; e=0.10 =
	// adverse worst case. The detent window is the tightest customer — at
	// e=0.10 it goes nominally negative (a stiff first click that the bump's
	// crest polishes in; the 30-min coupon pair exists to check YOUR printer
	// before the long prints) — stated, not hidden.
	let stacks: [(&str, f64, f64); 5] = [
		("tongue↔wall radial", C_R, 2.0),
		("lug↔channel tangential (per side)", 0.44, 2.0),
		("lug↔ceiling axial", CEIL_CLR, 2.0),
		("tip↔landing axial", 0.20, 2.0),
		("detent ride window", CEIL_CLR - BUMP_LIFT, 2.0),
	];
	let mut tight_ok = true;
	let mut worst_ok = true;
	for (name, nom, sens) in stacks {
		let e05 = nom - sens * 0.05;
		let e10 = nom - sens * 0.10;
		if e05 < 0.05 - 1e-9 {
			tight_ok = false;
		}
		// at e=0.10 the three guiding fits must stay open; the two axial
		// end-gaps may close to zero contact but must not invert past the
		// detent's own elastic ride (−0.05). The 1e-9 slack is float noise at
		// the exactly-at-spec boundaries, not a loosened requirement.
		if e10 < if name.contains("radial") || name.contains("tangential") || name.contains("ceiling") { -1e-9 } else { -0.05 - 1e-9 } {
			worst_ok = false;
		}
		println!("    {name:36} nominal {nom:5.2}  e=0.05→{e05:5.2}  e=0.10→{e10:5.2}");
	}
	gate("all fits ≥0.05 with e=0.05 per-part print error", tight_ok, String::new(), &mut ok);
	gate("guiding fits stay open at e=0.10 adverse stack", worst_ok, String::new(), &mut ok);

	// ---- strength + thermal analysis (closed-form, conservative, generated) -----
	// Hand-checkable engineering arithmetic — NOT FEA, and labeled as such in
	// the generated ANALYSIS.md. Every sustained-load case must clear the HOT
	// (50 °C, near-HDT) allowable, because the loaded spool lives in a dryer.
	println!("\nstrength + thermal analysis (design allowables: {SIG_ALLOW_RT}/{TAU_ALLOW_RT} MPa RT, {SIG_ALLOW_HOT}/{TAU_ALLOW_HOT} MPa @50 °C):");
	let w_coil = COIL_KG * 9.81;
	let w_total = (COIL_KG + 2.0 * HALF_KG_EST) * 9.81;

	// LC1 — dryer, spool lying FLAT (the recommended pose): the bottom flange
	// face rests on the dryer floor, the coil presses the flange web in pure
	// through-thickness compression over the (annulus − windows) area.
	let a_web = PI * (92.0_f64.powi(2) - 44.0_f64.powi(2)) - 6.0 * PI * WIN_HOLE_R * WIN_HOLE_R;
	let sig_lc1 = w_coil / a_web;
	// LC2 — dryer, spool UPRIGHT on its rims: the coil hangs on the barrel,
	// a thin tube simply supported between the flange webs (M = WL/8).
	let r_mid = RO - WALL / 2.0;
	let i_tube = PI * r_mid.powi(3) * WALL;
	let sig_lc2 = (w_coil * (SPOOL_W - 2.0 * FLANGE_T) / 8.0) * RO / i_tube;
	// LC3 — hanging on a holder rod through the bore: full weight carried by
	// the hub↔flange junction in annular shear.
	let sig_lc3 = w_total / (2.0 * PI * (BORE_R + HUB_T) * FLANGE_T);
	let sig_hot_max = sig_lc1.max(sig_lc2).max(sig_lc3);
	let m_sustained = (SIG_ALLOW_HOT / sig_lc1)
		.min(SIG_ALLOW_HOT / sig_lc2)
		.min(TAU_ALLOW_HOT / sig_lc3); // LC3 is shear — judged against τ
	gate(
		"LC1–LC3 sustained @50 °C: worst margin ≥20× under allowable",
		m_sustained >= 20.0,
		format!("{sig_hot_max:6.4} MPa, {m_sustained:4.0}×"),
		&mut ok,
	);

	// LC4 — AMS retract torque across the joint. Worst case: 25 N filament
	// tension at the full r=92 coil (T = 2.3 N·m) crossing the joint. Two-stage
	// path: the 3 lug end-faces carry it for ≤3° of free play, then the three
	// 2.4×16.3 mm tongue side-faces bottom out and carry everything.
	let t_apply = 25.0 * 0.092; // N·m
	let a_tongue_side = 3.0 * T_T * (Z_TIP - Z_BAND1);
	// MPa (N/mm²) × mm² × radius (m) → N·m; tongue mid-radius 35.85 mm
	let t_ult_hot = SIG_ALLOW_HOT * a_tongue_side * 35.85e-3;
	let t_ult_rt = t_ult_hot * SIG_ALLOW_RT / SIG_ALLOW_HOT;
	gate(
		"LC4 joint torque: tongue-side capacity ≥4× worst AMS pull (hot)",
		t_ult_hot >= 4.0 * t_apply,
		format!("{t_ult_hot:4.1} vs {t_apply:4.2} N·m"),
		&mut ok,
	);
	// LC5 — pull-apart: 3 engaged lugs in shear over the lug↔ceiling overlap.
	let lug_arc = (R_TO + LUG_H / 2.0) * (LUG_A1 - LUG_A0).to_radians();
	let a_shear = 3.0 * lug_arc * (R_TO + LUG_H - RI);
	let f_pull_rt = a_shear * TAU_ALLOW_RT;
	let f_pull_hot = a_shear * TAU_ALLOW_HOT;
	gate(
		"LC5 pull-apart ≥ 8 kgf at RT design allowables",
		f_pull_rt / 9.81 >= 8.0,
		format!("{a_shear:4.1} mm² ⇒ {:4.1} kgf", f_pull_rt / 9.81),
		&mut ok,
	);
	gate(
		"LC5 hot: ≥2× the full spool's weight even at 50 °C",
		f_pull_hot >= 2.0 * w_total,
		format!("{f_pull_hot:4.0} N vs {w_total:4.1} N"),
		&mut ok,
	);
	// LC6 — overtwist abuse: past the arm-end stop the tongue sides engage
	// after ≤2.3° windup; hand torque ~3 N·m vs the same tongue capacity.
	gate(
		"LC6 overtwist: tongue capacity ≥3× a 3 N·m hand torque (RT)",
		t_ult_rt >= 9.0,
		format!("{t_ult_rt:4.1} N·m"),
		&mut ok,
	);

	// T1 — the material rule this whole campaign is honest about: a PLA spool
	// must stay below PLA's own HDT (54 °C @1.8 MPa, Bambu TDS) with margin →
	// recommended dryer setpoint ≤ 45–50 °C, spool lying flat.
	let pla_hdt = 54.0;
	let setpoint = 50.0;
	gate(
		"T1 PLA spool: 50 °C setpoint sits under PLA HDT 54 °C",
		setpoint + 4.0 <= pla_hdt + 0.01,
		format!("setpoint {setpoint} HDT {pla_hdt}"),
		&mut ok,
	);
	// T2 — expansion: joint is PLA-on-PLA (zero differential); barrel vs the
	// cardboard core grows ΔØ = α·ΔT·Ø, absorbed by the crushable ribs.
	let d_thermal = 2.0 * RO * 6.8e-5 * 30.0;
	gate(
		"T2 thermal Ø growth (+30 K) stays inside the coil clearance",
		d_thermal < (COIL_ID_NOM - 2.0 * RO - 2.0 * RIB_PROUD),
		format!("ΔØ {d_thermal:4.2}"),
		&mut ok,
	);
	// T3 — creep: geometric retention carries ZERO preload; the only sustained
	// stresses are LC1–LC3 (≤ sig_hot_max), a small fraction of the hot
	// allowable. Nothing is left strained to relax in the dryer.
	gate(
		"T3 zero-preload joint: sustained stress ≤2% of hot allowable",
		sig_hot_max <= 0.02 * SIG_ALLOW_HOT,
		format!("{sig_hot_max:6.4} MPa"),
		&mut ok,
	);
	// T3b — the HONEST sustained check (added 2026-07-30 after the field-report
	// re-audit hook flagged T3's citation). SIG_ALLOW_HOT is a STATIC allowable:
	// it describes a load applied and removed, which is not what a spool parked
	// in a warm dryer for weeks experiences. The correct reference for sustained
	// load is the time-derated creep table. Worst realistic exposure: a user who
	// leaves filament in a heated dryer continuously → the 1-YEAR cell, and a
	// 50 °C setpoint rounds UP to the 55 °C row (the lookup is conservative by
	// construction). That cell is flagged in the source data as a BOUND, not a
	// measurement — which is exactly why the design must clear it by a wide
	// margin rather than sit near it. LC1/LC2 are tension/bearing, LC3 is shear.
	let sig_creep = creep_allowable_mpa(50.0, 8760.0);
	let tau_creep = creep_shear_allowable_mpa(50.0, 8760.0);
	let m_creep = (sig_creep / sig_lc1).min(sig_creep / sig_lc2).min(tau_creep / sig_lc3);
	gate(
		"T3b sustained vs 1-YEAR CREEP bound @55 °C tier: margin ≥10×",
		m_creep >= 10.0,
		format!("{m_creep:5.1}× (σ {sig_creep} / τ {tau_creep} MPa)"),
		&mut ok,
	);
	// detent + ligament geometry
	gate(
		"detent window: 0.15 lift vs 0.30 ceiling (0.15 margin)",
		(CEIL_CLR - BUMP_LIFT) >= 0.10,
		format!("margin {:4.2}", CEIL_CLR - BUMP_LIFT),
		&mut ok,
	);
	let spoke_arc = WIN_R * (60.0_f64.to_radians()) - 2.0 * WIN_HOLE_R;
	gate(
		"window spokes ≥ 12 mm wide",
		spoke_arc >= 12.0,
		format!("spoke {spoke_arc:4.1}"),
		&mut ok,
	);

	// ---- ANALYSIS.md — generated from the live numbers above --------------------
	let analysis = format!(
		r#"# RESPOOL — strength & thermal analysis (generated by respool.rs)

Closed-form conservative arithmetic, regenerated on every gated run — **not
FEA**. Loads assume a full 1 kg refill; spool mass 2×{HALF_KG_EST:.3} kg.

## Allowables (PLA, printed)

| tier | tension/bearing | shear | basis |
|---|---|---|---|
| 20 °C design | {SIG_ALLOW_RT} MPa | {TAU_ALLOW_RT} MPa | 35 MPa base × 0.6 layer adhesion × 0.5 design factor |
| 50 °C sustained | {SIG_ALLOW_HOT} MPa | {TAU_ALLOW_HOT} MPa | near-HDT derate (PLA HDT 54 °C @ 1.8 MPa, Bambu TDS) |

## Load cases

| case | model | stress / load | governs vs | margin |
|---|---|---|---|---|
| LC1 dryer, lying flat (recommended) | coil in through-thickness compression on the flange web, area {a_web:.0} mm² | {sig_lc1:.4} MPa | 2.5 MPa hot | {m1:.0}× |
| LC2 dryer, standing upright | barrel = thin tube (I={i_tube:.2e} mm⁴), M=WL/8 | {sig_lc2:.4} MPa | 2.5 MPa hot | {m2:.0}× |
| LC3 hanging on holder rod | hub↔flange annular shear | {sig_lc3:.4} MPa | 1.5 MPa hot | {m3:.0}× |
| LC4 AMS retract torque across the joint | 25 N tension @ r92 → {t_apply:.1} N·m; carried by 3 tongue side-faces ({a_tongue_side:.0} mm²) after ≤3° windup | capacity {t_ult_hot:.1} N·m hot / {t_ult_rt:.1} N·m RT | applied {t_apply:.1} N·m | {m4:.1}× hot |
| LC5 axial pull-apart | 3 lugs in shear, {a_shear:.1} mm² | {f_rt_kg:.1} kgf RT / {f_hot_n:.0} N hot | handling snatch ≈ 25 N | {m5:.1}× RT |
| LC6 overtwist abuse | tongue side-faces after the arm-end stop (≤2.3° windup) | {t_ult_rt:.1} N·m | 3 N·m hand torque | {m6:.0}× |

Transient note (LC4/LC6): before the tongue sides bottom out, the three lug
end-faces see the torque alone — at the full 2.3 N·m that is ~13 MPa local
bearing, at the RT bearing limit. It is a cold, momentary, compressive
contact (the dryer never applies torque); the bounded two-stage path is why
the joint still gates ≥4× on capacity.

## Thermal

- **The joint cannot be the failure point in a dryer**: retention is
  geometric with zero preload — sustained stress anywhere is ≤{sig_hot_max:.3} MPa,
  {pct:.1}% of the 50 °C STATIC allowable. But a static allowable describes a
  load applied and removed, and a spool parked in a warm dryer is a **creep**
  case, so the load-bearing check is the time-derated one: against the
  **1-year sustained bound at the 55 °C tier** ({sig_creep} MPa tension /
  {tau_creep} MPa shear, from `materials::pla::creep_allowable_mpa`; a 50 °C
  setpoint rounds up to the 55 °C row) the worst margin is **{m_creep:.0}×**
  (gate T3b). Honest caveat carried from the source data: that cell is a
  BOUND, not a measurement — no experiment supports any sustained allowable
  above ~0.5 MPa at 55 °C beyond days. The margin is wide enough that the
  conclusion holds either way, which is the point of quoting the bound rather
  than the flattering static number.
- **The material is the limit.** PLA: Tg 60 °C, Vicat 57 °C, HDT 54 °C
  (Bambu PLA TDS). Community reports show PLA spools warping at 50 °C dryer
  setpoints over heater hotspots. Rule printed on the box: **PLA spool →
  dry at ≤45–50 °C, spool lying flat** (LC1: {m1:.0}× margin flat vs {m2:.0}×
  upright — flat also keeps the flange off the heater). For 55–70 °C cycles
  reprint the same STL in PETG/ASA/ABS (Bambu's own spool is ABS, ≤70 °C;
  ABS+PC, ≤90 °C).
- **"Melt and fuse" check**: PLA melts at ~150–170 °C; dryers top out at
  70–85 °C — fusing to the filament is physically out of reach. The real
  failure is slow warping above ~54 °C, which the setpoint rule avoids.
- Expansion: PLA-on-PLA joint → zero differential; barrel vs cardboard core
  grows ΔØ {d_thermal:.2} mm at +30 K, absorbed by the crushable ribs (T2 gate).

## Out of scope (stated, not hidden)

- Drop/impact onto a flange rim (no honest closed form; brittle-PLA risk —
  the official ABS spool survives drops better).
- Long-term creep under the coil's winding tension (winding pre-tension is
  unknown; mitigated by the stiff Ø81 tube and the ≤50 °C rule).
- Fatigue of the detent over thousands of open/close cycles (the bump rides
  on a chamfer with full-width contact and zero rest-state load — wear-
  tolerant by construction, but not life-tested).
"#,
		m1 = SIG_ALLOW_HOT / sig_lc1,
		m2 = SIG_ALLOW_HOT / sig_lc2,
		m3 = TAU_ALLOW_HOT / sig_lc3,
		m4 = t_ult_hot / t_apply,
		m5 = f_pull_rt / 25.0,
		m6 = t_ult_rt / 3.0,
		f_rt_kg = f_pull_rt / 9.81,
		f_hot_n = f_pull_hot,
		pct = 100.0 * sig_hot_max / SIG_ALLOW_HOT,
	);
	let _ = std::fs::write("spool_system/respool/analysis/ANALYSIS.md", analysis);

	// ---- coupons mate exactly like the real parts -------------------------------
	println!("\nlock coupon (two prints of coupon_lock flip-mate):");
	let cl_locked = overlap_volume(&c_lock, &c_lock.transformed(pose_b(PSI_LOCK, 0.10))).unwrap_or(f64::NAN);
	let cl_pull = overlap_volume(&c_lock, &c_lock.transformed(pose_b(PSI_LOCK, 0.75))).unwrap_or(f64::NAN);
	gate(
		"coupon locks clean and retains under pull",
		cl_locked.abs() < 0.05 && cl_pull > 0.5,
		format!("lock {cl_locked:5.2} pull {cl_pull:5.2}"),
		&mut ok,
	);

	// ---- exports: assembly STL/STEP with round-trip volume conservation ---------
	println!("\nexports:");
	let mut asm = Mesh::default();
	merge_into(&mut asm, &m_half);
	merge_into(&mut asm, &m_b_locked);
	let _ = std::fs::write("spool_system/respool/assembly/assembly.stl", asm.to_stl_binary());
	// the merged assembly mesh is itself a deliverable — validate it: closed
	// everywhere and exactly TWO shells (one per half; a merge bug or a posing
	// bug would change either)
	let asm_shells = {
		let mut parent: Vec<u32> = (0..asm.positions.len() as u32).collect();
		fn find(p: &mut [u32], i: u32) -> u32 {
			let mut i = i;
			while p[i as usize] != i {
				p[i as usize] = p[p[i as usize] as usize];
				i = p[i as usize];
			}
			i
		}
		for t in asm.indices.chunks(3) {
			let (a, b, c) = (find(&mut parent, t[0]), find(&mut parent, t[1]), find(&mut parent, t[2]));
			parent[b as usize] = a;
			parent[c as usize] = a;
		}
		let mut roots: Vec<u32> = (0..asm.positions.len() as u32).map(|i| find(&mut parent, i)).collect();
		roots.sort_unstable();
		roots.dedup();
		roots.len()
	};
	gate(
		"ASSEMBLY.stl mesh: watertight, exactly 2 shells",
		asm.is_watertight() && asm_shells == 2,
		format!("wt={} shells={asm_shells}", asm.is_watertight()),
		&mut ok,
	);
	// independent second oracle: the assembly layer's interference checker.
	// Mesh instances route through the winding-number/voxel path, which cannot
	// resolve this joint's 0.1–0.3 mm design gaps at any practical voxel size
	// (the LOCK pose is owned by the exact-boolean gates above, which prove
	// 0.000 overlap) — so the oracle is exercised where its resolution is
	// honest: separated must read CLEAR, a forced 1.0 mm interpenetration
	// must read HIT. Both directions, so the second subsystem provably works
	// on these instances rather than silently agreeing.
	let oracle_pairs = |psi: f64, dz: f64| {
		let mut kasm = kernel_model::Assembly::new();
		kasm.add(kernel_model::Instance::from_mesh(&m_half, kernel_core::math::Affine3A::IDENTITY));
		kasm.add(kernel_model::Instance::from_mesh(
			&mesh_posed(&m_half, pose_b(psi, dz)),
			kernel_core::math::Affine3A::IDENTITY,
		));
		// 2.0 mm voxels: plenty to tell a 12 mm separation from a forced 1 mm
		// overlap, and ~64x cheaper than 0.5 mm on mesh instances (the winding
		// field is evaluated per voxel — resolution here is oracle cost, not truth)
		kasm.interferences(0.05, 2.0f32).len()
	};
	let (apart, bite) = (oracle_pairs(0.0, 12.0), oracle_pairs(PSI_LOCK, -1.0));
	gate(
		"assembly-layer oracle: clear when apart, bites at −1.0 overlap",
		apart == 0 && bite >= 1,
		format!("apart {apart} bite {bite}"),
		&mut ok,
	);
	let _ = std::fs::write("spool_system/respool/assembly/scene/half_a.stl", m_half.to_stl_binary());
	let _ = std::fs::write("spool_system/respool/assembly/scene/half_b.stl", m_b_locked.to_stl_binary());
	// a MOCK of the purchased 1 kg refill coil (not a printed part — scene/BOM
	// prop for the assembly-doc sheet; OD drawn at ~Ø170 of a nominal Ø185 max)
	let coil_mock = kernel_brep::revolve(
		&[
			DVec2::new(41.0, FLANGE_T + 0.5),
			DVec2::new(85.0, FLANGE_T + 0.5),
			DVec2::new(85.0, SPOOL_W - FLANGE_T - 0.5),
			DVec2::new(41.0, SPOOL_W - FLANGE_T - 0.5),
		],
		96,
	);
	let _ = std::fs::write(
		"spool_system/respool/assembly/scene/refill_coil_mock.stl",
		tessellate_default(&coil_mock).to_stl_binary(),
	);
	let _ = std::fs::write(
		"spool_system/respool/cad/spool_half.step",
		export_step(&half, "respool_half"),
	);
	let instances = vec![
		("spool_half".to_string(), half.clone(), DAffine3::IDENTITY),
		("spool_half".to_string(), half.clone(), pose_b(PSI_LOCK, 0.0)),
	];
	match export_step_assembly(&instances, "respool_locked") {
		Ok(step) => {
			let _ = std::fs::write("spool_system/respool/cad/assembly.step", &step);
			match import_step_assembly(&step) {
				Ok(back) => {
					let v_out: f64 = instances.iter().map(|(_, s, _)| volume(s).abs()).sum();
					let v_in: f64 = back.iter().map(|(_, s, _)| volume(s).abs()).sum();
					let dv = (v_out - v_in).abs() / v_out;
					gate(
						"STEP assembly round-trip conserves volume (<2.5%)",
						back.len() == 2 && dv < 0.025,
						format!("dv {:5.2}%", dv * 100.0),
						&mut ok,
					);
				}
				Err(e) => gate("STEP assembly re-import", false, format!("{e:?}"), &mut ok),
			}
		}
		Err(e) => gate("STEP assembly export", false, format!("{e:?}"), &mut ok),
	}

	// assembly/BOM.md — explicit bill of materials, generated from live volumes
	let half_g = volume(&half).abs() * PLA_G_PER_MM3;
	let bom = format!(
		"# RESPOOL — bill of materials\n\n| item | qty | source | material | mass (solid-equiv) |\n|---|---|---|---|---|\n| spool_half (parts/) | 2 | print | PLA, 4 walls 25% | {half_g:.0} g each |\n| 1 kg filament refill, Ø82 core | 1 | purchased | any brand | ~1 kg |\n| shim_1.0mm (optional/) | 0–1 | print | PLA | {shim_g:.0} g |\n| coupon_core / coupon_lock (optional/) | 1 / 2 | print (pre-flight) | PLA | {core_g:.0} / {lock_g:.0} g |\n| lock pin | 0–1 | 75 mm scrap of 1.75 filament | — | — |\n\nNo screws, no inserts, no tools.\n",
		shim_g = volume(&shim).abs() * PLA_G_PER_MM3,
		core_g = volume(&c_core).abs() * PLA_G_PER_MM3,
		lock_g = volume(&c_lock).abs() * PLA_G_PER_MM3,
	);
	let _ = std::fs::write("spool_system/respool/assembly/BOM.md", bom);

	let grams = 2.0 * volume(&half).abs() * PLA_G_PER_MM3;
	gate(
		"analysis mass input matches the engine volume (±20%)",
		(volume(&half).abs() * PLA_G_PER_MM3 / 1000.0 - HALF_KG_EST).abs() <= 0.2 * HALF_KG_EST,
		format!("{:5.3} kg vs {HALF_KG_EST}", volume(&half).abs() * PLA_G_PER_MM3 / 1000.0),
		&mut ok,
	);
	println!("\nspool (two halves): {grams:.0} g of PLA at 100% — official spool is ~250 g");
	println!("\nRESPOOL: {}", if ok { "ALL GATES PASS" } else { "<<< FAIL" });
	std::process::exit(if ok { 0 } else { 1 });
}
