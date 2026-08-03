//! RETRACT26 — retractable USB-cable reel with a PRINTED compliant spiral
//! power spring (badge-reel architecture, doubled cable, both connectors
//! stationary). **Ø72.2 × 29.6 mm, 6 printed parts, ZERO non-printed parts.**
//!
//! Architecture (every choice justified; physics + audit trail in
//! retract26/DESIGN.md and VALIDATION_2026-07-22.md):
//! - DOUBLED cable: the cable's midpoint loops a Ø5 post inside a FLUSH
//!   pocket sunk into the spool core (nothing protrudes into the winding
//!   path — a proud post makes the wraps stack over its bulge and jam
//!   against the wall; the A-CABLE emulation gate exists because of that);
//!   both halves wind together and BOTH connectors stay outside the housing
//!   — no slip ring, no twisting. Reach ≈ 0.47 m per side of a 1 m cable.
//! - PETG spiral clock spring (t 0.75 × h 11.6, 7.55 turns): the user asked
//!   for TPU95; a TPU spiral at this size peaks at ~0.07 N of retraction
//!   (coil PACKING caps the band at t ≤ ~1.3 and TPU's E ≈ 26 MPa is 80×
//!   too soft) — user approved PETG (Prusament TDS: E 1500 MPa, σy 47).
//!   COMPACT solve (user directive): the band sits exactly on the creep
//!   boundary — parked stress σ_p = E·t·θp/2L ⇒ creep SF 2.0, which fixes
//!   L ≥ 120·t·θp and h ≥ 6.2/t²; t is then capped by coil packing
//!   (stroke), giving t 0.75 / h 11.6. Root tapered 2× (joint SCF killed).
//! - stroke honesty: available windup = n(coiled solid at hub) − n(printed);
//!   the printed spiral IS the free state.
//! - inner end: hex ring keyed on the AF8 hex arbor; outer end: Ø3 bead
//!   standing up through a web hole AT THE SPIRAL'S END ANGLE (pure shear,
//!   drop-out blocked by the base plate). Pull-out TIGHTENS the coil.
//! - PRELOAD (1⅓ turns = 8 clicks) is REAL: the arbor is a separate part
//!   whose AF14 hex flange seats 4.0 mm deep in a base pocket (yank-proof).
//!   Push it up with a flat screwdriver through the Ø6.5 hole underneath,
//!   wind, drop back at the nearest 60°. preload_turns is the tuning knob
//!   (NOTE: each extra turn eats creep margin — see DESIGN.md).
//! - ZERO fasteners: the spool stack is retained by a printed QUARTER-TURN
//!   twist-cap. The arbor's Ø4 stub ends in a capsule tab (radial extent
//!   3.1 — it passes the spool's Ø6.4 bore during assembly); the cap drops
//!   over it, twists 90°, and the tab rides an internal ledge (detent bumps
//!   park it). Coin slot on top. The cap overhangs the core's counterbore
//!   step with 0.3 float; core→lugs→cup complete the chain, as before.
//! - open-top housing: the spool's scalloped Ø70.6 grip flange IS the
//!   exposed disc — spin it to assist or rewind manually; it overhangs the
//!   wall top so cable can never escape the open-top exit slots (drop the
//!   cable in at assembly — no connector threading).
//!
//! Print discipline: every part prints support-free in its stated
//! orientation (steep == 0, bridge ≤ 12). retract26/parts/ is the COMPLETE
//! bill of materials — there is no hardware.
//!
//! Run: cargo run --example retract26 -p kernel-model --release  ->  retract26/

use kernel_brep::math::{DAffine3, DVec2, DVec3};
use kernel_brep::{
	cone, cuboid, cylinder, difference, export_step_assembly, extrude, import_step_assembly, revolve, tessellate_default,
	try_difference, union, validate, volume, Mesh, Solid,
};
use kernel_core::math::Vec3;
use std::f64::consts::{PI, TAU};

// ---- parameters (retract26/params.csv overrides) -------------------------------------
#[derive(Clone, Copy)]
struct P {
	cable_d: f64,       // cable diameter (thin USB ≈ 3.5)
	wind_mm: f64,       // doubled-pair length wound on the spool (reach per side)
	preload_turns: f64, // spring windup at cable-fully-in — the retraction-force knob
	band_t: f64,        // spring band thickness
	band_h: f64,        // spring band height (SYNC-ASSERTED to BAND_H: the z-stack is built around it)
	turns: f64,         // printed spiral turns
	e_mpa: f64,         // spring material Young's modulus (Prusament PETG TDS: 1500)
	yield_mpa: f64,     // spring material yield (Prusament PETG TDS: 47, XY)
}
fn load() -> P {
	let mut p = P {
		cable_d: 3.5,
		wind_mm: 470.0,
		preload_turns: 4.0 / 3.0,
		band_t: 0.75,
		band_h: BAND_H,
		turns: 7.32,
		e_mpa: 1500.0,
		yield_mpa: 47.0,
	};
	if let Ok(text) = std::fs::read_to_string("retract26/params.csv") {
		for line in text.lines() {
			let line = line.trim();
			if line.starts_with('#') || line.is_empty() {
				continue;
			}
			let mut it = line.split(',');
			let (Some(k), Some(val)) = (it.next(), it.next()) else { continue };
			let Ok(x) = val.trim().parse::<f64>() else { continue };
			match k.trim() {
				"cable_d" => p.cable_d = x,
				"wind_mm" => p.wind_mm = x,
				"preload_turns" => p.preload_turns = x,
				"band_t" => p.band_t = x,
				"band_h" => p.band_h = x,
				"turns" => p.turns = x,
				"e_mpa" => p.e_mpa = x,
				"yield_mpa" => p.yield_mpa = x,
				_ => {}
			}
		}
	}
	// the z-stack consts are built around BAND_H — params must not silently diverge
	assert!((p.band_h - BAND_H).abs() < 1e-9, "params.csv band_h {} != example BAND_H {BAND_H} — the stack is derived from it", p.band_h);
	p
}

// ---- the assembled stack (mm, z up, housing base bottom at z = 0) ----
// COMPACT rev (2026-07-22, user: "as compact as possible, all printed, least
// parts"): Ø72.2 × 29.6, was Ø75 × 39.4 (−31 % volume), zero hardware.
// Diameter floor = the CABLE (3 layers to r32.5 + 1.0 + wall). Height floor =
// the creep-limited band (h 6.2/t²) + groove + mechanism. Price, documented in
// DESIGN.md: fatigue margin at the 1e6-cycle blanket rule < 1 (≈2×10⁵
// unfactored full pulls) and parked force rides the 0.21 N floor.
const BAND_H: f64 = 11.6; // spring band height — the z-stack is built around it
const BASE_T: f64 = 3.0; // housing base plate 0..3
const BOSS_R: f64 = 12.0; // Ø24 centre boss 3..5 — arbor hex pocket + spring seat
const BOSS_TOP: f64 = 5.0;
const WALL_IR: f64 = 33.5; // wall inner radius (drum r33 + 0.5; wound cable r32.5 + 1.0)
const WALL_OR: f64 = 36.1; // device Ø72.2
const WALL_TOP: f64 = 26.9; // 0.3 under the grip flange
const AFLANGE_AF: f64 = 14.0; // arbor's hex flange (base pocket, 60° preload index)
const AFLANGE_Z0: f64 = 1.0; // flange 1.0..5.0: 4.0 hex engagement (yank-proof)
const HEX_AF: f64 = 8.0; // arbor hex shaft — the spring's key AND the preload winder
const HEX_TOP: f64 = 17.0; // hex 5..17.0; its top face is the web thrust shoulder
const JOUR_R: f64 = 3.0; // Ø6 journal, HEX_TOP..JOUR_TOP — both spool bores ride it
const JOUR_TOP: f64 = 25.6;
const SPRING_Z: f64 = 5.0; // spring ring seats on the boss top, 5..16.6 (h 11.6)
const RING_OR: f64 = 6.0; // spring inner hex-ring outer radius
const CAV_R: f64 = 29.9; // spring outer coil+bead limit (printed state = loosest)
const DRUM_IR: f64 = 30.4; // cup drum cavity radius (CAV_R + 0.5)
const DRUM_OR: f64 = 33.0;
const DRUM_Z0: f64 = 3.6; // drum skirt hangs to 0.6 above the base plate
const WEB_Z0: f64 = 17.0; // cup web (= lower flange, r33) 17.0..19.4, on the hex shoulder
const WEB_Z1: f64 = 19.4; // web top = cable-groove floor plane
const BORE_R: f64 = 3.2; // Ø6.4 over the Ø6 journal (web and core)
const LUG_CIRCLE: f64 = 13.0; // 3× Ø6 core lugs / Ø6.6 web holes at r13
const LUG_D: f64 = 6.0;
// cable-fold anchor: FLUSH pocket in the core + internal Ø5 post (a proud
// post jams the wound stack against the wall — caught by cable emulation)
const PKT_R0: f64 = 10.0;
const PKT_HALF: f64 = 26.0 * PI / 180.0;
const POST_RC: f64 = 16.0;
const POST_R: f64 = 2.5;
const BEAD_RC: f64 = 28.3; // spring outer bead centre radius
const BEAD_D: f64 = 3.0; // web hole Ø3.4, at the spiral's end angle
const CORE_R: f64 = 22.0; // cable-groove floor Ø44
const CORE_Z1: f64 = 27.2; // groove 19.4..27.2 (7.8: Ø3.5 pair side-by-side + 0.8)
const FLANGE_Z1: f64 = 29.6; // grip flange 27.2..29.6 — device top
const FLANGE_R: f64 = 35.3; // overhangs the wall top — cable can't escape the slots
const CB_R: f64 = 8.5; // Ø17 cap counterbore in the core, CB_Z..top
const CB_Z: f64 = 25.3; // step 0.3 below the journal top: the cap overhangs it, 0.3 float
const SLOT_W: f64 = 5.0; // cable exit slots at 0° and 180°, open to the wall top
const SLOT_Z0: f64 = 19.0;
// printed quarter-turn twist-cap (replaces the M3 screw + washer-cap):
const STUB_R: f64 = 2.0; // Ø4 stub above the journal carries the tab
const ARBOR_TOP: f64 = 28.2; // stub 25.6..28.2
const TAB_Z0: f64 = 26.8; // capsule tab 26.8..27.8, radial extent 3.1
const TAB_HL: f64 = 1.9; // tab straight half-length (+ end radius = 3.1 extent)
const TAB_HW: f64 = 1.2;
const CAP_R: f64 = 8.0; // Ø16 disc — overhangs the CB step to retain the core
const CAP_BOT: f64 = 25.6; // nominal cap underside (0.3 above the CB step)
const LEDGE_Z: f64 = 26.8; // internal ledge the twisted tab hangs on
const CHAM_TOP: f64 = 28.3; // tab chamber ceiling
const CAP_TOP: f64 = 29.4; // 0.2 under the flange top; coin slot on top
const PETG: f64 = 0.00127; // g/mm³

const _: () = assert!(WEB_Z0 == HEX_TOP, "web must sit on the hex-top thrust shoulder");
const _: () = assert!(BORE_R < HEX_AF / 2.0, "web bore must NOT pass over the hex (that shoulder is the axial support)");
const _: () = assert!(JOUR_TOP - CB_Z > 0.29 && JOUR_TOP - CB_Z < 0.31, "spool axial float = JOUR_TOP - CB_Z ≈ 0.3");
const _: () = assert!(CAP_R > BORE_R + 1.0 && CAP_R < CB_R, "cap must overhang the bore yet drop into the counterbore");
const _: () = assert!(TAB_HL + TAB_HW < BORE_R - 0.05, "tab capsule extent must pass the spool Ø6.4 bore");
const _: () = assert!(TAB_HL + TAB_HW > STUB_R + 1.0, "tab must overhang the Ø4.4 cap bore enough to bear on the ledge");
const _: () = assert!(CORE_Z1 - WEB_Z1 > 7.79 && CORE_Z1 - WEB_Z1 < 7.81, "cable groove width ≈ 7.8");
const _: () = assert!(FLANGE_R > WALL_IR, "grip flange must overhang the wall top (slot escape cover)");
const _: () = assert!(DRUM_IR >= CAV_R + 0.5, "spring coil clearance to the drum");
const _: () = assert!(BEAD_RC + (BEAD_D + 0.4) / 2.0 + 0.2 < DRUM_IR, "bead web-hole stays clear of the drum wall");
const _: () = assert!(WALL_TOP < CORE_Z1, "wall top under the grip flange");
const _: () = assert!(SLOT_Z0 <= WEB_Z1, "exit slot floor at/below the groove floor");
const _: () = assert!(RING_OR < BOSS_R, "spring ring must seat on the boss top");
const _: () = assert!(AFLANGE_Z0 >= 0.9, "pocket floor thickness under the arbor flange");
const _: () = assert!(BOSS_TOP - AFLANGE_Z0 >= 3.5, "arbor hex index engagement (yank-out resistance)");
const _: () = assert!(POST_RC + POST_R <= CORE_R - 3.5, "fold post + its Ø3.5 loop must stay inside the core surface");
const _: () = assert!(PKT_R0 >= CB_R + 1.4, "pocket inner wall ligament to the cap counterbore");
const _: () = assert!(SPRING_Z + BAND_H <= WEB_Z0 - 0.3, "spring must clear the web underside");
const _: () = assert!(CAP_TOP <= FLANGE_Z1 - 0.2, "cap (and coin slot) recessed below the flange top");
const _: () = assert!(TAB_Z0 + 1.0 + 0.4 <= CHAM_TOP, "tab up-travel headroom inside the chamber");

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
	let mut a = 0.0;
	for i in 0..p.len() {
		let (q, r) = (p[i], p[(i + 1) % p.len()]);
		a += q.x * r.y - q.y * r.x;
	}
	if a < 0.0 {
		p.reverse();
	}
	p
}
fn circle(r: f64, n: usize) -> Vec<DVec2> {
	(0..n).map(|k| { let a = TAU * k as f64 / n as f64; DVec2::new(r * a.cos(), r * a.sin()) }).collect()
}
/// regular hexagon by across-flats
fn hexagon(af: f64) -> Vec<DVec2> {
	let r = af / 2.0 / (PI / 6.0).cos();
	(0..6).map(|k| { let a = TAU * k as f64 / 6.0 + PI / 6.0; DVec2::new(r * a.cos(), r * a.sin()) }).collect()
}
fn bore(s: &Solid, face: DVec3, axis: DVec3, d: f64, len: f64, seg: usize) -> Solid {
	difference(s, &cylinder(face - axis * 0.5, axis, d * 0.5, len + 0.5, seg))
}
/// capsule (stadium) solid: straight half-length hl, half-width hw, z0..z1
fn capsule(hl: f64, hw: f64, z0: f64, z1: f64) -> Solid {
	let mut c = cuboid(v(-hl, -hw, z0), v(hl, hw, z1));
	c = union(&c, &cylinder(v(hl, 0.0, z0), DVec3::Z, hw, z1 - z0, 24));
	union(&c, &cylinder(v(-hl, 0.0, z0), DVec3::Z, hw, z1 - z0, 24))
}

// ---- spring geometry -----------------------------------------------------------------

/// Archimedean band centreline, RING_OR+0.15 → BEAD_RC. The band is TAPERED
/// 2×t → t over the first half turn (bending stress ∝ 1/t² ⇒ the ring
/// junction runs at ~25 % of band stress — kills the joint SCF).
/// Returns (points, per-point thickness, arc length).
fn spring_centreline(p: &P) -> (Vec<DVec2>, Vec<f64>, f64) {
	let theta_end = p.turns * TAU;
	let (r0, r1) = (RING_OR + 0.15, BEAD_RC);
	let c = (r1 - r0) / theta_end;
	let n = (p.turns * 48.0) as usize;
	let mut pts = Vec::with_capacity(n + 1);
	let mut ts = Vec::with_capacity(n + 1);
	let mut len = 0.0;
	let mut prev = DVec2::new(r0, 0.0);
	for k in 0..=n {
		let th = theta_end * k as f64 / n as f64;
		let r = r0 + c * th;
		let q = DVec2::new(r * th.cos(), r * th.sin());
		if k > 0 {
			len += (q - prev).length();
		}
		pts.push(q);
		ts.push(p.band_t * (1.0 + (1.0 - th / PI).max(0.0)));
		prev = q;
	}
	(pts, ts, len)
}

/// ribbon polygon: centreline offset ±t(i)/2 by local normals
fn ribbon(pts: &[DVec2], ts: &[f64]) -> Vec<DVec2> {
	let n = pts.len();
	let normal = |i: usize| {
		let a = if i == 0 { pts[0] } else { pts[i - 1] };
		let b = if i + 1 == n { pts[n - 1] } else { pts[i + 1] };
		let d = (b - a).normalize();
		DVec2::new(-d.y, d.x)
	};
	let mut poly = Vec::with_capacity(2 * n);
	for (i, q) in pts.iter().enumerate() {
		poly.push(*q + normal(i) * (ts[i] / 2.0));
	}
	for i in (0..n).rev() {
		poly.push(pts[i] - normal(i) * (ts[i] / 2.0));
	}
	ccw(poly)
}

/// PETG spiral power spring: hex ring + tapered band + tall end bead (the
/// bead pokes up through the web hole). One flat print.
fn spring(p: &P) -> Solid {
	let (cl, ts, _) = spring_centreline(p);
	let ring = difference(&extrude(&ccw(circle(RING_OR, 96)), p.band_h), &extrude(&ccw(hexagon(HEX_AF + 0.3)), p.band_h));
	let band = extrude(&ribbon(&cl, &ts), p.band_h);
	let end = *cl.last().unwrap();
	let bead = cylinder(v(end.x, end.y, 0.0), DVec3::Z, BEAD_D / 2.0, WEB_Z1 - 0.2 - SPRING_Z, 24);
	union(&union(&ring, &band), &bead).transformed(tr(0.0, 0.0, SPRING_Z))
}

// ---- printed parts -------------------------------------------------------------------

/// Housing: base plate + centre boss with the arbor hex pocket (Ø6.5 driver
/// hole through) + wall with two open-top exit slots. Prints as-is.
fn housing() -> Solid {
	let mut h = cylinder(v(0.0, 0.0, 0.0), DVec3::Z, WALL_OR, BASE_T, 128);
	let wall = difference(
		&cylinder(v(0.0, 0.0, BASE_T - 0.01), DVec3::Z, WALL_OR, WALL_TOP - BASE_T + 0.01, 128),
		&cylinder(v(0.0, 0.0, BASE_T - 0.5), DVec3::Z, WALL_IR, WALL_TOP, 128),
	);
	h = union(&h, &wall);
	for a in [0.0, PI] {
		let cut = cuboid(v(WALL_IR - 1.0, -SLOT_W / 2.0, SLOT_Z0), v(WALL_OR + 1.0, SLOT_W / 2.0, WALL_TOP + 1.0));
		h = difference(&h, &cut.transformed(rotz(a)));
	}
	h = union(&h, &cylinder(v(0.0, 0.0, BASE_T - 0.01), DVec3::Z, BOSS_R, BOSS_TOP - BASE_T + 0.01, 64));
	h = difference(&h, &extrude(&ccw(hexagon(AFLANGE_AF + 0.4)), BOSS_TOP - AFLANGE_Z0 + 0.5).transformed(tr(0.0, 0.0, AFLANGE_Z0)));
	h = bore(&h, v(0.0, 0.0, AFLANGE_Z0), -DVec3::Z, 6.5, AFLANGE_Z0 + 1.0, 24);
	// two flush M4-csk wall-mount holes (OPTIONAL user hardware, not in BOM)
	for a in [PI / 2.0, -PI / 2.0] {
		let (cx, cy) = (30.0 * a.cos(), 30.0 * a.sin());
		h = bore(&h, v(cx, cy, BASE_T), -DVec3::Z, 4.5, BASE_T + 1.0, 24);
		h = difference(&h, &cone(v(cx, cy, BASE_T + 0.01), -DVec3::Z, 4.6, 4.6, 24));
	}
	h
}

/// Arbor: AF14 hex flange (pocket-seated, screwdriver slot underneath) →
/// AF8 hex (spring key / preload winder) → Ø6 journal → Ø4 stub with the
/// twist-lock capsule tab. A separate part precisely SO preload can be
/// wound in. Prints flange-down.
fn arbor() -> Solid {
	let mut a = extrude(&ccw(hexagon(AFLANGE_AF)), BOSS_TOP - AFLANGE_Z0).transformed(tr(0.0, 0.0, AFLANGE_Z0));
	a = union(&a, &extrude(&ccw(hexagon(HEX_AF)), HEX_TOP - BOSS_TOP + 0.01).transformed(tr(0.0, 0.0, BOSS_TOP - 0.01)));
	a = union(&a, &cylinder(v(0.0, 0.0, HEX_TOP - 0.01), DVec3::Z, JOUR_R, JOUR_TOP - HEX_TOP + 0.01, 48));
	a = union(&a, &cylinder(v(0.0, 0.0, JOUR_TOP - 0.01), DVec3::Z, STUB_R, ARBOR_TOP - JOUR_TOP + 0.01, 32));
	// twist-lock tab: capsule, radial extent 3.1 < spool bore r3.2
	a = union(&a, &capsule(TAB_HL, TAB_HW, TAB_Z0, TAB_Z0 + 1.0));
	// screwdriver slot across the flange underside
	difference(&a, &cuboid(v(-8.0, -1.1, AFLANGE_Z0 - 0.5), v(8.0, 1.1, AFLANGE_Z0 + 1.6)))
}

/// Spool cup: drum skirt around the spring + web (= lower flange) with the
/// three lug holes and the bead hole at the spiral's end angle. Prints
/// upside-down (web on the bed, drum wall rising).
fn cup(p: &P) -> Solid {
	let drum = difference(
		&cylinder(v(0.0, 0.0, DRUM_Z0), DVec3::Z, DRUM_OR, WEB_Z0 - DRUM_Z0 + 0.01, 128),
		&cylinder(v(0.0, 0.0, DRUM_Z0 - 0.5), DVec3::Z, DRUM_IR, WEB_Z0 - DRUM_Z0 + 1.0, 128),
	);
	let web = cylinder(v(0.0, 0.0, WEB_Z0), DVec3::Z, DRUM_OR, WEB_Z1 - WEB_Z0, 128);
	let mut c = union(&drum, &web);
	c = bore(&c, v(0.0, 0.0, WEB_Z1), -DVec3::Z, BORE_R * 2.0, WEB_Z1 - WEB_Z0 + 1.0, 48);
	for k in 0..3 {
		let a = TAU * k as f64 / 3.0 + PI / 2.0;
		c = bore(&c, v(LUG_CIRCLE * a.cos(), LUG_CIRCLE * a.sin(), WEB_Z1), -DVec3::Z, LUG_D + 0.6, WEB_Z1 - WEB_Z0 + 1.0, 24);
	}
	// the spring's bead stands up through this hole (torque anchor, pure
	// shear) — at the spiral's end angle so any printed turn count works
	let ba = p.turns.fract() * TAU;
	c = bore(&c, v(BEAD_RC * ba.cos(), BEAD_RC * ba.sin(), WEB_Z1), -DVec3::Z, BEAD_D + 0.4, WEB_Z1 - WEB_Z0 + 1.0, 24);
	c
}

/// Spool core: scalloped grip flange + groove-floor core + three drop-in
/// torque lugs + the flush fold pocket with its Ø5 post. Prints flange-down.
fn core() -> Solid {
	let mut outline = Vec::new();
	let n = 192;
	for k in 0..n {
		let a = TAU * k as f64 / n as f64;
		let bite = 1.2 * (0.5 + 0.5 * (12.0 * a).cos()).powi(3);
		let r = FLANGE_R - bite;
		outline.push(DVec2::new(r * a.cos(), r * a.sin()));
	}
	let flange = extrude(&ccw(outline), FLANGE_Z1 - CORE_Z1).transformed(tr(0.0, 0.0, CORE_Z1));
	let corec = cylinder(v(0.0, 0.0, WEB_Z1 - 0.01), DVec3::Z, CORE_R, CORE_Z1 - WEB_Z1 + 0.02, 96);
	let mut s = union(&flange, &corec);
	// three Ø6 lugs drop through the web holes (torque coupling, no screws) —
	// at 90/210/330° so they clear the fold pocket's ±26° sector at 0°
	for k in 0..3 {
		let a = TAU * k as f64 / 3.0 + PI / 2.0;
		s = union(&s, &cylinder(v(LUG_CIRCLE * a.cos(), LUG_CIRCLE * a.sin(), WEB_Z0 + 0.3), DVec3::Z, LUG_D / 2.0, WEB_Z1 - WEB_Z0 - 0.29, 24));
	}
	// FLUSH fold pocket: sector bite (ceiling 0.25 into the flange), Ø5 post
	// hanging inside — the cable's midpoint U loops it entirely inside Ø44
	let mut sect = vec![DVec2::new(PKT_R0 * (-PKT_HALF).cos(), PKT_R0 * (-PKT_HALF).sin())];
	for k in 0..=16 {
		let a = -PKT_HALF + 2.0 * PKT_HALF * k as f64 / 16.0;
		sect.push(DVec2::new((CORE_R + 1.0) * a.cos(), (CORE_R + 1.0) * a.sin()));
	}
	for k in (0..=16).rev() {
		let a = -PKT_HALF + 2.0 * PKT_HALF * k as f64 / 16.0;
		sect.push(DVec2::new(PKT_R0 * a.cos(), PKT_R0 * a.sin()));
	}
	s = difference(&s, &extrude(&ccw(sect), CORE_Z1 - WEB_Z1 + 0.5).transformed(tr(0.0, 0.0, WEB_Z1 - 0.25)));
	// post reaches PAST the pocket ceiling so it fuses to the flange
	s = union(&s, &cylinder(v(POST_RC, 0.0, WEB_Z1 + 0.2), DVec3::Z, POST_R, CORE_Z1 - WEB_Z1 + 0.3, 24));
	// arbor bore + cap counterbore (step 0.3 below the journal top = float)
	s = bore(&s, v(0.0, 0.0, CB_Z), -DVec3::Z, BORE_R * 2.0, CB_Z - WEB_Z1 + 1.5, 48);
	difference(&s, &cylinder(v(0.0, 0.0, CB_Z), DVec3::Z, CB_R, FLANGE_Z1 - CB_Z + 0.5, 48))
}

/// Twist-cap: the printed quarter-turn lock that replaces all hardware.
/// Drop over the arbor stub (entry slot aligned with the tab), press, twist
/// 90° — the tab hangs on the internal ledge past two detent bumps; the
/// Ø16 disc overhangs the core's counterbore step (0.3 float). Coin slot on
/// top to remove. Prints top-face-down.
fn cap() -> Solid {
	let mut c = cylinder(v(0.0, 0.0, CAP_BOT), DVec3::Z, 5.0, LEDGE_Z + 0.8 - CAP_BOT, 48); // Ø10 boss
	c = union(&c, &cylinder(v(0.0, 0.0, LEDGE_Z + 0.8), DVec3::Z, CAP_R, CAP_TOP - LEDGE_Z - 0.8, 64)); // Ø16 disc
	c = bore(&c, v(0.0, 0.0, CAP_TOP), -DVec3::Z, STUB_R * 2.0 + 0.4, CAP_TOP - CAP_BOT + 1.0, 32); // Ø4.4 over the stub
	// tab chamber above the ledge
	c = difference(&c, &cylinder(v(0.0, 0.0, LEDGE_Z), DVec3::Z, TAB_HL + TAB_HW + 0.4, CHAM_TOP - LEDGE_Z, 32));
	// entry slot below the ledge (capsule + 0.3 clearance)
	c = difference(&c, &capsule(TAB_HL, TAB_HW + 0.3, CAP_BOT - 0.5, LEDGE_Z + 0.01));
	// detent bumps at 45°/225° on the ledge: the twisted tab climbs past
	// them and parks at 90° (0.25 tall — crush features, print as blobs)
	for sgn in [1.0, -1.0] {
		let (bx, by) = (1.95 * sgn, 1.95 * sgn);
		c = union(&c, &cuboid(v(bx - 0.45, by - 0.45, LEDGE_Z), v(bx + 0.45, by + 0.45, LEDGE_Z + 0.25)));
	}
	// coin slot across the top
	difference(&c, &cuboid(v(-8.5, -0.8, CAP_TOP - 0.75), v(8.5, 0.8, CAP_TOP + 0.5)))
}

// ---- emit / audit --------------------------------------------------------------------

fn emit(name: &str, s: &Solid, to_print: DAffine3) -> (bool, f64) {
	let val = validate(s);
	let mut printed = s.transformed(to_print);
	let zmin = tessellate_default(&printed).positions.iter().map(|q| q.z as f64).fold(f64::INFINITY, f64::min);
	printed = printed.transformed(tr(0.0, 0.0, -zmin));
	let mesh = tessellate_default(&printed);
	let rep = mesh.support_free_report(Vec3::Z, 45.0, 0.3);
	let wt = mesh.is_watertight();
	let grams = volume(s).abs() * PETG;
	let ok = val.is_valid() && wt && rep.steep_area < 1e-6 && rep.max_bridge_span <= 12.0;
	let _ = std::fs::write(format!("retract26/parts/{name}.stl"), mesh.to_stl_binary());
	println!(
		"  {name:12} valid={:5} wt={wt:5} {}  {grams:5.1}g  {}",
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
	let _ = std::fs::create_dir_all("retract26/parts");
	if let Ok(dir) = std::fs::read_dir("retract26/parts") {
		for e in dir.flatten() {
			let _ = std::fs::remove_file(e.path());
		}
	}
	let p = load();
	let mut ok = true;

	// ---- A-SPRING: honest stroke (vs the printed free state), strain, torque ----
	let (cl0, ts0, band_l) = spring_centreline(&p);
	let mut band_area = 0.0; // Σ t·ds — what coil packing actually consumes
	let mut compl = 0.0; // C = Σ ds/(E·I(s)), I = h·t³/12
	for i in 1..cl0.len() {
		let ds = (cl0[i] - cl0[i - 1]).length();
		let t = 0.5 * (ts0[i] + ts0[i - 1]);
		band_area += t * ds;
		compl += ds / (p.e_mpa * p.band_h * t.powi(3) / 12.0);
	}
	let r_ip = RING_OR + p.band_t / 2.0;
	let n_arbor = ((r_ip * r_ip + band_area / PI).sqrt() - r_ip) / p.band_t;
	let stroke = n_arbor - p.turns; // printed spiral = free state; windup only tightens
	// cable capacity: pair layers stack radially from the core
	let mut wind_turns = 0.0;
	let mut rem = p.wind_mm;
	let mut layers = 0;
	while rem > 0.0 {
		let r = CORE_R + p.cable_d * (layers as f64 + 0.5);
		let cir = TAU * r;
		wind_turns += (rem / cir).min(1.0);
		rem -= cir;
		layers += 1;
	}
	let cable_out_r = CORE_R + p.cable_d * layers as f64;
	let need = wind_turns + p.preload_turns + 0.3;
	let th_full = (wind_turns + p.preload_turns) * TAU;
	let th_park = p.preload_turns * TAU;
	let m_full = th_full / compl; // N·mm (tapered-band series compliance)
	let m_park = th_park / compl;
	let f_full = m_full / (CORE_R + 0.5 * p.cable_d);
	let f_park = m_park / (cable_out_r - 0.5 * p.cable_d);
	let i_nom = p.e_mpa * p.band_h * p.band_t.powi(3) / 12.0;
	let strain = (p.band_t / 2.0) * m_full / i_nom; // max at the NOMINAL section (root sees ¼)
	let strain_park = (p.band_t / 2.0) * m_park / i_nom; // sustained — the creep case
	let yield_strain = p.yield_mpa / p.e_mpa;
	// creep gate: parked stress vs 0.25·yield (the production_check sustained
	// rule at SF 2) — the COMPACT solve sits right on this boundary
	let creep_sf = 0.25 * p.yield_mpa / (strain_park * p.e_mpa);
	let spring_ok = stroke >= need && strain <= 0.45 * yield_strain && f_park >= 0.2 && f_full >= 0.6 && creep_sf >= 1.95;
	ok &= spring_ok;
	println!(
		"RETRACT26 COMPACT — Ø{:.1}×{:.0} reel, {:.0} mm doubled Ø{} cable ({} layers, {wind_turns:.2} turns), 6 printed parts, ZERO hardware\n\nA-SPRING: band {:.2}×{:.0} L{:.0}, {} printed turns · stroke (n_hub {n_arbor:.2} − printed) {stroke:.2} ≥ need {need:.2} · strain {:.2}% ≤ 45% of yield {:.2}% · parked creep SF {creep_sf:.2} ≥ 1.95 · M {:.1}→{:.1} N·mm · F {f_full:.2} N full / {f_park:.2} N parked  {}",
		WALL_OR * 2.0, FLANGE_Z1, p.wind_mm, p.cable_d, layers,
		p.band_t, p.band_h, band_l, p.turns,
		strain * 100.0, yield_strain * 100.0, m_full, m_park,
		if spring_ok { "OK" } else { "<<< FAIL" }
	);

	// ---- A-CLEAR: every fit derived and asserted numerically ----
	let (cl, ts, _) = spring_centreline(&p);
	let band_max_r = cl.iter().map(|q| q.length()).fold(0.0f64, f64::max) + BEAD_D / 2.0;
	let pitch = (BEAD_RC - RING_OR - 0.15) / p.turns;
	let gap = (0..cl.len().saturating_sub(48)).map(|i| pitch - 0.5 * (ts[i] + ts[i + 48])).fold(f64::INFINITY, f64::min);
	let cable_wall = WALL_IR - cable_out_r;
	let flange_slot_cover = FLANGE_R - WALL_IR;
	let clear_ok = band_max_r <= CAV_R
		&& gap >= 0.8
		&& cable_wall >= 1.0
		&& flange_slot_cover >= 1.5
		&& (CORE_Z1 - WEB_Z1) >= 2.0 * p.cable_d + 0.8;
	ok &= clear_ok;
	println!(
		"A-CLEAR: coil+bead max r {band_max_r:.2} ≤ {CAV_R} · gap {gap:.2} ≥ 0.8 · cable→wall {cable_wall:.2} ≥ 1 · flange covers slots {flange_slot_cover:.1} ≥ 1.5 · groove {:.1} ≥ pair {:.1}  {}",
		CORE_Z1 - WEB_Z1, 2.0 * p.cable_d + 0.8,
		if clear_ok { "OK" } else { "<<< FAIL" }
	);

	// ---- parts (the COMPLETE BOM — nothing non-printed) ----
	let house = housing();
	let arb = arbor();
	let spr = spring(&p);
	let cup_s = cup(&p);
	let core_s = core();
	let cap_s = cap();
	println!("\nprintable parts (retract26/parts is the COMPLETE BOM; spring in PETG, rest PETG/PLA):");
	let flip = DAffine3::from_rotation_x(PI);
	let parts: Vec<(&str, &Solid, DAffine3)> = vec![
		("housing", &house, DAffine3::IDENTITY),
		("arbor", &arb, DAffine3::IDENTITY),
		("spring_petg", &spr, DAffine3::IDENTITY),
		("spool_cup", &cup_s, flip), // prints web-down
		("spool_core", &core_s, flip), // prints flange-down
		("twist_cap", &cap_s, flip), // prints top-face-down
	];
	for (n, s, m) in &parts {
		let (o, _) = emit(n, s, *m);
		ok &= o;
	}

	// ---- A-SPIN: rotating parts must clear the fixed parts at any angle ----
	// (cap is arbor-fixed → part of the fixed set; cup↔core designed contact
	// is asserted numerically, not boolean-probed)
	let fixed = union(&union(&house, &arb), &cap_s);
	let mut spin_ok = true;
	for k in 0..6 {
		let a = TAU * k as f64 / 6.0 + 0.13;
		for (n, s) in [("cup", &cup_s), ("core", &core_s)] {
			let ov = overlap_mm3(&s.transformed(rotz(a)), &fixed);
			if ov.is_nan() || ov >= 0.05 {
				spin_ok = false;
				println!("  {n} ∩ fixed at {:.0}°: {ov:.3} mm³  <<<", a.to_degrees());
			}
		}
	}
	let ov_spring = overlap_mm3(&spr, &core_s);
	spin_ok &= !ov_spring.is_nan() && ov_spring < 0.05;
	ok &= spin_ok;
	println!("\nA-SPIN: cup+core ×6 angles vs housing+arbor+cap, spring vs core — zero interference  {}", if spin_ok { "OK" } else { "<<< FAIL" });

	// ---- A-LOCK: the twist-cap interface, asserted numerically ----
	// pass: tab extent < spool bore r · engage: tab overhang past cap bore ·
	// twist headroom: chamber radius > tab extent · retention chain: cap
	// underside overhangs the CB step with the designed float
	let tab_ext = TAB_HL + TAB_HW;
	let lock_ok = tab_ext < BORE_R - 0.05
		&& tab_ext - (STUB_R + 0.2) >= 0.8
		&& (TAB_HL + TAB_HW + 0.4) - tab_ext >= 0.3
		&& (CAP_BOT - CB_Z - 0.3).abs() < 1e-9;
	ok &= lock_ok;
	println!(
		"A-LOCK: tab extent {tab_ext:.1} passes bore Ø{:.1} · ledge bearing {:.2} · chamber twist clearance 0.4 · cap float 0.3  {}",
		BORE_R * 2.0,
		tab_ext - (STUB_R + 0.2),
		if lock_ok { "OK" } else { "<<< FAIL" }
	);

	// ---- A-CABLE: the cable EMULATED in the worst (fully wound) state ----
	let strand_ring = |rc: f64, zc: f64, sr: f64| -> Solid {
		let prof: Vec<DVec2> = (0..24)
			.map(|k| {
				let a = TAU * k as f64 / 24.0;
				DVec2::new(rc + sr * a.cos(), zc + sr * a.sin())
			})
			.collect();
		revolve(&ccw(prof), 96)
	};
	let mut cable_ok = true;
	let (z_lo, z_hi) = (WEB_Z1 + 0.5 * p.cable_d, WEB_Z1 + 1.5 * p.cable_d);
	let sr = 0.5 * p.cable_d;
	for layer in 0..layers {
		let rc = CORE_R + p.cable_d * (layer as f64 + 0.5);
		assert!(rc - sr >= CORE_R - 1e-9, "wound layer must not sink into the core");
		for zc in [z_lo, z_hi] {
			// core probe uses a 0.1-shrunk strand: layer 0 TOUCHES the core by
			// design and a tangent boolean is the known degeneracy; the exact
			// tangency is the assert above
			for (tn, tgt, r) in [("housing", &house, sr), ("arbor", &arb, sr), ("core", &core_s, sr - 0.1)] {
				let ov = overlap_mm3(&strand_ring(rc, zc, r), tgt);
				if ov.is_nan() || ov >= 0.05 {
					cable_ok = false;
					println!("  wound layer {layer} (r{rc:.1}, z{zc:.1}) ∩ {tn}: {ov:.3} mm³  <<<");
				}
			}
		}
	}
	// exit segments: one strand out of each slot at its winding height
	for (a, zc) in [(0.0, z_lo), (PI, z_hi)] {
		let stub = cylinder(v(cable_out_r + 0.1, 0.0, zc), DVec3::X, 0.5 * p.cable_d, WALL_OR - cable_out_r + 3.0, 24)
			.transformed(rotz(a));
		let ov = overlap_mm3(&stub, &house);
		if ov.is_nan() || ov >= 0.05 {
			cable_ok = false;
			println!("  exit strand at {:.0}° ∩ housing: {ov:.3} mm³  <<<", a.to_degrees());
		}
	}
	// the fold: a U around the Ø5 post inside the flush pocket
	let loop_out = POST_RC + POST_R + p.cable_d;
	let loop_in = POST_RC - POST_R - p.cable_d;
	let t1_chord_edge = (CORE_R + 0.5 * p.cable_d) * PKT_HALF.cos() - 0.5 * p.cable_d;
	let t2_chord_edge = (CORE_R + 1.5 * p.cable_d) * PKT_HALF.cos() - 0.5 * p.cable_d;
	let fold_ok = loop_out <= CORE_R
		&& loop_in >= PKT_R0
		&& t1_chord_edge - (POST_RC + POST_R) >= 0.8
		&& t2_chord_edge >= CORE_R - 0.2;
	cable_ok &= fold_ok;
	ok &= cable_ok;
	println!(
		"A-CABLE: {} wound rings + 2 exit strands — zero interference · fold U r[{loop_in:.1},{loop_out:.1}] inside pocket [{PKT_R0},{CORE_R}] · turn-1 mouth chord {t1_chord_edge:.1} clears post {:.1} by ≥0.8 · turn-2 chord {t2_chord_edge:.1} stays outside Ø44  {}",
		2 * layers,
		POST_RC + POST_R,
		if cable_ok { "OK" } else { "<<< FAIL" }
	);

	// ---- assembly (exact poses) + exploded + STEP ----
	let mut asm = Mesh::new();
	let mut instances: Vec<(String, Solid, DAffine3)> = Vec::new();
	let place = |m: &mut Mesh, list: &mut Vec<(String, Solid, DAffine3)>, name: &str, s: &Solid, x: DAffine3| {
		merge_into(m, &tessellate_default(&s.transformed(x)));
		list.push((name.to_string(), s.clone(), x));
	};
	place(&mut asm, &mut instances, "housing", &house, DAffine3::IDENTITY);
	place(&mut asm, &mut instances, "arbor", &arb, DAffine3::IDENTITY);
	place(&mut asm, &mut instances, "spring_petg", &spr, DAffine3::IDENTITY);
	place(&mut asm, &mut instances, "spool_cup", &cup_s, DAffine3::IDENTITY);
	place(&mut asm, &mut instances, "spool_core", &core_s, DAffine3::IDENTITY);
	place(&mut asm, &mut instances, "twist_cap", &cap_s, rotz(PI / 2.0)); // locked: slot ⊥ tab
	let _ = asm.write_stl_binary("retract26/ASSEMBLY.stl");
	println!("  {} triangles -> retract26/ASSEMBLY.stl", asm.indices.len() / 3);

	let lifts = [0.0, 18.0, 40.0, 62.0, 92.0, 118.0];
	let mut expl = Mesh::new();
	for (i, (_, s, x)) in instances.iter().enumerate() {
		merge_into(&mut expl, &tessellate_default(&s.transformed(tr(0.0, 0.0, lifts[i]) * *x)));
	}
	let _ = expl.write_stl_binary("retract26/ASSEMBLY_EXPLODED.stl");

	match export_step_assembly(&instances, "retract26") {
		Ok(step) => {
			let _ = std::fs::write("retract26/ASSEMBLY.step", &step);
			match import_step_assembly(&step) {
				Ok(back) => {
					let v0: f64 = instances.iter().map(|(_, s, _)| volume(s).abs()).sum();
					let v1: f64 = back.iter().map(|(_, s, _)| volume(s).abs()).sum();
					let dv = (v0 - v1).abs() / v0;
					let sok = back.len() == instances.len() && dv < 0.025;
					ok &= sok;
					println!("A-STEP: {} instances, {} KB, round-trip Δ {:.2}%  {}", instances.len(), step.len() / 1024, dv * 100.0, if sok { "OK" } else { "<<< FAIL" });
				}
				Err(e) => {
					ok = false;
					println!("A-STEP re-import failed: {e:?}  <<< FAIL");
				}
			}
		}
		Err(e) => {
			ok = false;
			println!("A-STEP export failed: {e:?}  <<< FAIL");
		}
	}

	println!("\n{}", if ok { "RETRACT26: ALL GATES PASS" } else { "RETRACT26: GATE FAILURES — see <<< above" });
	if !ok {
		std::process::exit(1);
	}
}
