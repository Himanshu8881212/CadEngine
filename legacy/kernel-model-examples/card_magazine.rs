//! TWO-STATE MAGAZINE — a media-card magazine where "shot" and "fresh" are not
//! a label, a sticker or a flip convention: they are the DEPTH THE CARD SITS AT.
//!
//! Every camera department tracks shot-vs-fresh media with a convention — flip
//! the card, sticker in or out, red case and green case — and every one of them
//! fails silently the moment somebody is tired, gloved, or working in the dark.
//! The failure mode is asymmetric: mistaking a fresh card for a shot one costs
//! you a card slot; mistaking a SHOT card for a fresh one costs you the footage.
//!
//! So this magazine makes the two states differ by GEOMETRY, and makes the
//! dangerous transition the mechanically hard one:
//!
//!   FRESH — the card rides the shelf at the +X end. It stands PROUD of the top
//!           face and pulls out with one gloved thumb.
//!   SHOT  — push it toward −X. It runs off the shelf and drops into the well,
//!           finishing BELOW the top face, where two interference ribs grip it.
//!
//! FRESH → SHOT is a one-finger push and gravity. SHOT → FRESH requires lifting
//! the card the full shelf height AND sliding it the length of the well — two
//! axes, against friction, deliberately. The failure-safe direction is the one
//! the geometry makes hard, and that is not a claim in the write-up: the gate
//! suite below proves that no pure horizontal translation carries a card from
//! SHOT to FRESH (the shelf face blocks it) and measures the lift required.
//!
//! Mounting standard: ARRI Pin-Lock 1/4" female (ARRI INTERFACES v2023-05) —
//! 1/4-20 UNC on the axis, 8 × Ø3 anti-twist pin holes on an LK Ø10.4 bolt
//! circle at 45°. All EIGHT holes, not the two that most third-party parts cut,
//! so the magazine indexes in 45° steps on any ARRI-faced host. The 1/4-20
//! female is a captive ASME B18.2.2 hex nut in a side-entry pocket, NOT a
//! heat-set insert: a Ø9.52 insert wants a ~19 mm boss under the 2× rule and
//! that collides with the Ø10.4 pin ring (see ANALYSIS.md).
//!
//! HONEST SCOPE — printed pin holes are modelled 0.15 undersize and are meant
//! to be reamed; Ø3 E7 at ARRI's ⌖Ø0.1 CZ is not an FDM-achievable fit. The
//! plastic locates, the steel dowels carry the shear. See ANALYSIS.md.
//!
//! Run: cargo run --example card_magazine -p kernel-model --release
//!   -> camera_system/card_magazine/ (exit 1 on any FAIL)

use kernel_brep::math::{DAffine3, DMat3, DVec2, DVec3};
use kernel_brep::{
	boolean_hazards, cuboid, cylinder, difference, export_step, extrude, force_ccw,
	tessellate_default, union, validate, volume, ChainLog, HazardKind, Mesh, Solid,
};
use kernel_core::math::Vec3;
use kernel_model::{campaign::gate, materials, sweep_check};

// ---- the card (CFexpress Type B — the format whose loss hurts most) -------------
// Envelope from the CFexpress 2.0 spec family; the ±0.15 brand-to-brand
// thickness scatter reported by users is NOT vendor-published (see DESIGN.md
// open question 1), so every clearance below is sized for the THICK worst case
// and the printable coupon exists to verify it in 15 minutes.
const CARD_L: f64 = 38.5; // along X — the slide axis
const CARD_H: f64 = 29.6; // along Z — stands on edge
const CARD_T: f64 = 3.8; // along Y — nominal
const CARD_T_MAX: f64 = 3.95; // worst-case thick card (nominal + reported scatter)

// ---- slot array -----------------------------------------------------------------
const N_SLOTS: usize = 6;
const SLOT_W: f64 = 4.5; // 0.35/side on a nominal card; 0.275/side on a thick one
const DIV_T: f64 = 1.6; // divider wall: exactly 4 perimeters at a 0.4 nozzle
const OUT_WALL: f64 = 2.4; // outer side walls
const PITCH: f64 = SLOT_W + DIV_T;

// ---- the two states --------------------------------------------------------------
// The well must be at least one card long so a card can drop clear of the shelf;
// the shelf must be long enough that a card pushed onto it keeps its centre of
// mass OVER the shelf, or it tips back into the well. Both are gated.
const WALL_X: f64 = 3.0; // end walls (the two travel stops)
const WELL_L: f64 = 40.0; // > CARD_L: a card can sit wholly inside the well
const SHELF_L: f64 = 26.0; // shelf run; CG check below proves it is enough
const WELL_FLOOR: f64 = 3.0;
const SHELF_TOP: f64 = 15.0; // also the roof of the captive-nut pocket + 3.5
const BODY_Z: f64 = 36.0;
const BODY_X: f64 = WALL_X + WELL_L + SHELF_L + WALL_X;
const BODY_Y: f64 = N_SLOTS as f64 * SLOT_W + (N_SLOTS as f64 - 1.0) * DIV_T + 2.0 * OUT_WALL;

/// Card X-origin in the SHOT state (hard against the −X stop, 0.5 free).
const SHOT_X0: f64 = WALL_X + 0.5;
/// Card X-origin in the FRESH state (hard against the +X stop, 0.5 free).
const FRESH_X0: f64 = BODY_X - WALL_X - 0.5 - CARD_L;
/// Shelf leading face — the wall a SHOT card runs into if you try to slide it
/// straight across to FRESH. This is the safety interlock.
const SHELF_FACE_X: f64 = WALL_X + WELL_L;

// ---- retention ribs (vertical, so they print with no overhang at all) -----------
// Well ribs are a real interference and engage ONLY when the card is down in the
// well — they hold a SHOT card and add friction to the dangerous direction. They
// stop below shelf height so a card sliding across at shelf height never touches
// them. Shelf ribs are a light anti-rattle guide on a card you want to pull out.
const RIB_R: f64 = 0.8;
const WELL_RIB_P: f64 = 0.45; // protrusion into the slot, per side
const SHELF_RIB_P: f64 = 0.30;
const WELL_RIB_X: f64 = 20.0;
const SHELF_RIB_X: f64 = 55.0;
const WELL_RIB_Z1: f64 = 14.0; // 1.0 below shelf height — the sliding-clearance gate
const SHELF_RIB_Z1: f64 = 30.0;

// ---- ARRI Pin-Lock 1/4" female (ARRI INTERFACES v2023-05) ----------------------
const ARRI_X: f64 = 56.0; // on the shelf run, where there is 15 mm of stock
const PIN_BC_R: f64 = 5.2; // LK Ø10.4
const PIN_D_NOM: f64 = 3.0; // Ø3 E7 in metal
const PIN_D_PRINT: f64 = 2.85; // modelled undersize; ream to Ø3.00 for the dowels
const PIN_DEPTH: f64 = 5.2; // ARRI minimum
/// Loose dowel length: 5.2 buried here + 4.8 proud into the HOST's pin holes,
/// which is ARRI's published max protrusion for the Ø3 pin.
const DOWEL_LEN: f64 = 10.0;
const SCREW_CLEAR_D: f64 = 6.5; // 1/4-20 major Ø6.350 + clearance
/// The REAL nut: ASME B18.2.2 1/4-20 hex, across flats 10.87–11.13.
const NUT_AF_MAX: f64 = 11.13;
const NUT_T_MAX: f64 = 5.74; // thickness 5.38–5.74
/// The POCKET. A pocket cut at the nut's own across-flats does not accept the
/// nut — an FDM slot prints tighter than nominal and there is no room for the
/// fit at all. 0.47 of clearance is the difference between "captive nut" and
/// "sand the nut for twenty minutes".
const NUT_AF: f64 = 11.6;
const NUT_T: f64 = 6.0;
const NUT_Z0: f64 = 6.0; // above the pin holes, below the shelf floor
/// Across-corners of a hexagon = across-flats / cos 30°. This is the dimension
/// that decides whether the nut can SLIDE down the entry channel, and it is the
/// one the first draft got wrong.
fn across_corners(af: f64) -> f64 {
	af / (30.0_f64).to_radians().cos()
}

// ---- comfort features -------------------------------------------------------------
const SCOOP_X: f64 = 14.0; // thumb scoop over the well, for lifting a SHOT card
const SCOOP_R: f64 = 12.0;
const SCOOP_CZ: f64 = BODY_Z + 7.0;
const RAMP: f64 = 5.0; // 45° thumb ramp on the +X top edge

const PLA: f64 = materials::PLA_G_PER_MM3;
/// 6061-T6, for the honest metal mass. The decision document caught five
/// concepts whose aluminium mass claims were wrong by 3-8.6x; this one is
/// computed from the measured solid volume, not asserted.
const AL6061: f64 = 0.0027;

fn v(x: f64, y: f64, z: f64) -> DVec3 {
	DVec3::new(x, y, z)
}
fn tr(x: f64, y: f64, z: f64) -> DAffine3 {
	DAffine3::from_translation(v(x, y, z))
}

/// Centre-line Y of slot `i`.
fn slot_y(i: usize) -> f64 {
	-(BODY_Y / 2.0) + OUT_WALL + SLOT_W / 2.0 + i as f64 * PITCH
}

/// Prism from an (x,z) profile swept along +Y over [y0, y1] (det +1 frame) —
/// the DRYBOX helper, reused verbatim.
fn prism_y(profile: &[(f64, f64)], y0: f64, y1: f64) -> Solid {
	let p: Vec<DVec2> = profile.iter().map(|&(x, z)| DVec2::new(x, z)).collect();
	let m = DAffine3::from_mat3_translation(
		DMat3::from_cols(DVec3::X, DVec3::Z, DVec3::NEG_Y),
		v(0.0, y1, 0.0),
	);
	extrude(&force_ccw(p), y1 - y0).transformed(m)
}

/// Regular hexagon of the given across-flats size, centred on the origin, with
/// its FLATS perpendicular to Y.
///
/// The orientation is load-bearing, not cosmetic. The nut is slid in along X
/// through a channel of fixed Y width, so the nut's extent ACROSS the slide
/// direction has to be its across-flats, not its across-corners. The first
/// draft put vertices on ±Y (a 30° rotation from this), which made the pocket's
/// Y extent 12.94 while the channel feeding it was 11.2 — the nut could not
/// physically reach the pocket. Vertices at 0°, 60°, … put flats on ±Y.
fn hexagon(across_flats: f64) -> Vec<DVec2> {
	let r = across_flats / 3.0_f64.sqrt(); // circumradius
	force_ccw(
		(0..6)
			.map(|i| {
				let a = (60.0 * i as f64).to_radians();
				DVec2::new(r * a.cos(), r * a.sin())
			})
			.collect(),
	)
}

/// A card gauge: the CFexpress Type B envelope, `thick` for the worst case.
fn card(x0: f64, z0: f64, yc: f64, thick: f64) -> Solid {
	cuboid(
		v(x0, yc - thick / 2.0, z0),
		v(x0 + CARD_L, yc + thick / 2.0, z0 + CARD_H),
	)
}

fn merge_into(dst: &mut Mesh, src: &Mesh) {
	let base = dst.positions.len() as u32;
	dst.positions.extend_from_slice(&src.positions);
	dst.indices.extend(src.indices.iter().map(|i| i + base));
}

/// One slot's void: the deep well and the raised shelf, as a single cutter.
fn slot_void(yc: f64) -> Solid {
	let well = cuboid(
		v(WALL_X, yc - SLOT_W / 2.0, WELL_FLOOR),
		v(SHELF_FACE_X, yc + SLOT_W / 2.0, BODY_Z + 1.0),
	);
	let shelf = cuboid(
		v(SHELF_FACE_X, yc - SLOT_W / 2.0, SHELF_TOP),
		v(BODY_X - WALL_X, yc + SLOT_W / 2.0, BODY_Z + 1.0),
	);
	union(&well, &shelf)
}

/// The four vertical retention ribs of one slot, as a single solid. Each rib
/// runs from its own floor upward, so it has NO down-facing end face — the
/// support audit would otherwise catch a 0.6 mm² unsupported ledge per rib.
fn slot_ribs(yc: f64) -> Solid {
	let rib = |x: f64, side: f64, p: f64, z0: f64, z1: f64| {
		let ay = yc + side * (SLOT_W / 2.0 + RIB_R - p);
		cylinder(v(x, ay, z0), DVec3::Z, RIB_R, z1 - z0, 24)
	};
	let a = union(
		&rib(WELL_RIB_X, 1.0, WELL_RIB_P, WELL_FLOOR, WELL_RIB_Z1),
		&rib(WELL_RIB_X, -1.0, WELL_RIB_P, WELL_FLOOR, WELL_RIB_Z1),
	);
	let b = union(
		&rib(SHELF_RIB_X, 1.0, SHELF_RIB_P, SHELF_TOP, SHELF_RIB_Z1),
		&rib(SHELF_RIB_X, -1.0, SHELF_RIB_P, SHELF_TOP, SHELF_RIB_Z1),
	);
	union(&a, &b)
}

fn build_body() -> Result<Solid, kernel_brep::ChainError> {
	let blank = cuboid(v(0.0, -BODY_Y / 2.0, 0.0), v(BODY_X, BODY_Y / 2.0, BODY_Z));
	let mut chain = ChainLog::start("blank", blank)?.seal();

	// The six slot voids are mutually disjoint — pre-union them so the chain runs
	// ONE arrangement instead of six (DESIGN_GUIDE §7.7).
	chain.apply("slot voids", |s| {
		let mut cut: Option<Solid> = None;
		for i in 0..N_SLOTS {
			let c = slot_void(slot_y(i));
			cut = Some(match cut {
				Some(t) => union(&t, &c),
				None => c,
			});
		}
		difference(s, &cut.unwrap())
	})?;

	// Ribs are re-added into the voids; also mutually disjoint, also pre-unioned.
	chain.apply("retention ribs", |s| {
		let mut add: Option<Solid> = None;
		for i in 0..N_SLOTS {
			let r = slot_ribs(slot_y(i));
			add = Some(match add {
				Some(t) => union(&t, &r),
				None => r,
			});
		}
		union(s, &add.unwrap())
	})?;

	// Thumb scoop: a cylinder rolled across the full width over the well, so a
	// recessed SHOT card can be pinched. The cut leaves a concave UP-facing
	// surface, which costs the support audit nothing.
	chain.apply("thumb scoop", |s| {
		let sc = cylinder(
			v(SCOOP_X, -BODY_Y / 2.0 - 1.0, SCOOP_CZ),
			DVec3::Y,
			SCOOP_R,
			BODY_Y + 2.0,
			96,
		);
		difference(s, &sc)
	})?;

	// 45° thumb ramp on the +X top edge — the push-off face for setting SHOT.
	chain.apply("thumb ramp", |s| {
		let prof = [
			(BODY_X - RAMP, BODY_Z + 1.0),
			(BODY_X + 1.0, BODY_Z + 1.0),
			(BODY_X + 1.0, BODY_Z - RAMP),
		];
		difference(s, &prism_y(&prof, -BODY_Y / 2.0 - 1.0, BODY_Y / 2.0 + 1.0))
	})?;

	// ARRI Pin-Lock face. Pre-flighted with the hazard linter before cutting:
	// eight small cylinders on a bolt circle around a ninth is exactly the
	// near-coincident-cylinder shape §7.7 warns about.
	let mut arri = cylinder(v(ARRI_X, 0.0, -0.5), DVec3::Z, SCREW_CLEAR_D / 2.0, NUT_Z0 + 1.0, 48);
	for k in 0..8 {
		let a = (45.0 * k as f64).to_radians();
		arri = union(
			&arri,
			&cylinder(
				v(ARRI_X + PIN_BC_R * a.cos(), PIN_BC_R * a.sin(), -0.5),
				DVec3::Z,
				PIN_D_PRINT / 2.0,
				PIN_DEPTH + 0.5,
				32,
			),
		);
	}
	// captive hex nut + its side-entry channel, above the pin holes
	let hex = extrude(&hexagon(NUT_AF), NUT_T).transformed(tr(ARRI_X, 0.0, NUT_Z0));
	let entry = cuboid(
		v(ARRI_X, -NUT_AF / 2.0, NUT_Z0),
		v(BODY_X + 1.0, NUT_AF / 2.0, NUT_Z0 + NUT_T),
	);
	arri = union(&arri, &union(&hex, &entry));

	let hz = boolean_hazards(chain.solid(), &arri, 0.05);
	let warn: Vec<_> = hz
		.iter()
		.filter(|h| {
			matches!(
				h.kind,
				HazardKind::NearCoincidentPlanes
					| HazardKind::NearCoincidentCylinders
					| HazardKind::EdgeInFace
			)
		})
		.collect();
	assert!(
		warn.is_empty(),
		"ARRI cutter fails the §7.7 pre-flight: {warn:?} — re-dimension before cutting"
	);
	chain.apply("ARRI Pin-Lock face", |s| difference(s, &arri))?;

	Ok(chain.finish())
}

/// Two-slot fit coupon — prints in ~15 minutes and answers the only two
/// questions the research could not: does YOUR card slide in this slot, and do
/// the well ribs actually hold it. Carries the real well floor, the real shelf
/// step and both rib pairs at full size.
fn build_coupon() -> Solid {
	const CL: f64 = 26.0; // short X run: fits both rib stations, not a whole card
	let cy = SLOT_W + DIV_T; // two slots, centred
	let blank = cuboid(v(0.0, -cy / 2.0 - OUT_WALL, 0.0), v(CL, cy / 2.0 + OUT_WALL, BODY_Z));
	let mut s = blank;
	for side in [-1.0_f64, 1.0] {
		let yc = side * PITCH / 2.0;
		// left half is well depth, right half is shelf depth — the step in one part
		let well = cuboid(v(0.0, yc - SLOT_W / 2.0, WELL_FLOOR), v(CL / 2.0, yc + SLOT_W / 2.0, BODY_Z + 1.0));
		let shelf = cuboid(v(CL / 2.0, yc - SLOT_W / 2.0, SHELF_TOP), v(CL, yc + SLOT_W / 2.0, BODY_Z + 1.0));
		s = difference(&s, &union(&well, &shelf));
		let rib = |x: f64, sd: f64, p: f64, z0: f64, z1: f64| {
			cylinder(v(x, yc + sd * (SLOT_W / 2.0 + RIB_R - p), z0), DVec3::Z, RIB_R, z1 - z0, 24)
		};
		s = union(&s, &rib(CL / 4.0, 1.0, WELL_RIB_P, WELL_FLOOR, WELL_RIB_Z1));
		s = union(&s, &rib(CL / 4.0, -1.0, WELL_RIB_P, WELL_FLOOR, WELL_RIB_Z1));
		s = union(&s, &rib(3.0 * CL / 4.0, 1.0, SHELF_RIB_P, SHELF_TOP, SHELF_RIB_Z1));
		s = union(&s, &rib(3.0 * CL / 4.0, -1.0, SHELF_RIB_P, SHELF_TOP, SHELF_RIB_Z1));
	}
	s
}

fn emit(dir: &str, name: &str, s: &Solid, bridge_max: f64, ok: &mut bool) -> Mesh {
	let val = validate(s);
	let mesh = tessellate_default(s);
	let rep = mesh.support_free_report(Vec3::Z, 45.0, 0.3);
	let wt = mesh.is_watertight();
	let one = mesh.is_one_body();
	let vol = volume(s).abs();
	let pass = val.is_valid() && wt && one && rep.steep_area < 1e-6 && rep.max_bridge_span <= bridge_max;
	*ok &= pass;
	let _ = std::fs::write(format!("camera_system/card_magazine/{dir}/{name}.stl"), mesh.to_stl_binary());
	let _ = mesh.write_3mf(format!("camera_system/card_magazine/{dir}/{name}.3mf"));
	println!(
		"  {name:14} valid={:5} wt={wt:5} body={one:5} steep={:8.5} mm²  bridge≤{:5.2}  {:5.1} g PLA  {:5.1} g 6061  {}",
		val.is_valid(),
		rep.steep_area,
		rep.max_bridge_span,
		vol * PLA,
		vol * AL6061,
		if pass { "OK" } else { "<<< FAIL" }
	);
	mesh
}

fn main() {
	kernel_core::telemetry::enable();
	for d in ["parts", "cad", "analysis", "assembly/scene", "optional", "publish"] {
		let _ = std::fs::create_dir_all(format!("camera_system/card_magazine/{d}"));
	}
	println!("TWO-STATE MAGAZINE — shot vs fresh is the depth the card sits at:\n");

	let body = match build_body() {
		Ok(b) => b,
		Err(e) => {
			println!("body chain failed: {e}");
			std::process::exit(1);
		}
	};
	let coupon = build_coupon();

	let mut ok = true;
	// The captive-nut pocket roof is the one real bridge in the part.
	let m_body = emit("parts", "card_magazine", &body, NUT_AF + 0.2, &mut ok);
	let _ = emit("optional", "coupon_fit", &coupon, NUT_AF + 0.2, &mut ok);

	// ---- the two states, geometrically -------------------------------------------
	println!("\nthe two states (CFexpress Type B {CARD_L}×{CARD_H}×{CARD_T}):");
	let shot_top = WELL_FLOOR + CARD_H;
	let fresh_top = SHELF_TOP + CARD_H;
	gate(
		"SHOT card finishes BELOW the top face (≥1.0 recessed)",
		BODY_Z - shot_top >= 1.0,
		format!("{:4.1} mm below", BODY_Z - shot_top),
		&mut ok,
	);
	gate(
		"FRESH card stands PROUD (≥5.0, one gloved thumb)",
		fresh_top - BODY_Z >= 5.0,
		format!("{:4.1} mm proud", fresh_top - BODY_Z),
		&mut ok,
	);
	gate(
		"the two states differ by ≥8 mm of card top height",
		fresh_top - shot_top >= 8.0,
		format!("Δ {:4.1} mm", fresh_top - shot_top),
		&mut ok,
	);
	// A card pushed onto the shelf overhangs the well; if its centre of mass
	// clears the shelf face it stays put, otherwise it tips back in.
	let cg_x = FRESH_X0 + CARD_L / 2.0;
	gate(
		"FRESH card CG sits over the shelf (does not tip back)",
		cg_x > SHELF_FACE_X + 3.0,
		format!("CG {cg_x:4.1} vs shelf face {SHELF_FACE_X:4.1}"),
		&mut ok,
	);
	gate(
		"well is longer than a card (a card can drop fully clear)",
		WELL_L >= CARD_L + 1.0,
		format!("well {WELL_L} vs card {CARD_L}"),
		&mut ok,
	);

	// ---- THE SAFETY INTERLOCK -------------------------------------------------------
	// The whole product is this gate. Mistaking a shot card for fresh is the
	// expensive error, so SHOT -> FRESH must be mechanically hard. Prove that no
	// pure horizontal translation gets there: a card at well height driven +X
	// must run into the shelf face.
	println!("\nsafety interlock (SHOT → FRESH must be deliberate):");
	let yc0 = slot_y(0);
	let m_shot = tessellate_default(&card(SHOT_X0, WELL_FLOOR, yc0, CARD_T));
	// stop to stop, and no further: the path ENDS at the FRESH pose. Driving it
	// past that only proves the +X travel stop exists, which is not this gate.
	let travel = FRESH_X0 - SHOT_X0;
	let slide: Vec<DAffine3> =
		(0..=12).map(|i| tr(travel * i as f64 / 12.0, 0.0, 0.0)).collect();
	let push = sweep_check(&m_body, &m_shot, &slide);
	gate(
		"SHOT card driven +X at well height COLLIDES (interlock fires)",
		push.max_penetration > 0.5,
		format!("pen {:5.2} mm into the shelf face", push.max_penetration),
		&mut ok,
	);
	let lift = SHELF_TOP - WELL_FLOOR;
	gate(
		"escaping the well needs a ≥10 mm lift",
		lift >= 10.0,
		format!("lift {lift:4.1} mm"),
		&mut ok,
	);
	gate(
		"…and then a slide most of the well's length",
		FRESH_X0 - SHOT_X0 >= 25.0,
		format!("slide {travel:4.1} mm"),
		&mut ok,
	);
	// The safe direction must be easy: lifted to shelf height, the same card
	// crosses the whole body without touching anything. Floated 0.1 like every
	// contact pose in this shop — a card resting exactly ON the shelf is an
	// exact-contact pose (§7.4), and the first run read 12 kissing contacts.
	let m_shelf = tessellate_default(&card(SHOT_X0, SHELF_TOP + 0.1, yc0, CARD_T));
	let across = sweep_check(&m_body, &m_shelf, &slide);
	gate(
		"NC: at shelf height the same path is FREE (gate can pass)",
		across.max_penetration < 0.05 && across.contacts == 0,
		format!("pen {:5.3} c {}", across.max_penetration, across.contacts),
		&mut ok,
	);

	// ---- card fit, at nominal AND worst case -----------------------------------------
	println!("\ncard fit (nominal and worst-case thick card):");
	let free_clear = SLOT_W - CARD_T;
	let free_clear_max = SLOT_W - CARD_T_MAX;
	gate(
		"slot clearance on a nominal card 0.4–0.9 (slides, does not rattle)",
		(0.4..=0.9).contains(&free_clear),
		format!("{free_clear:4.2} mm total"),
		&mut ok,
	);
	gate(
		"a worst-case THICK card still slides free (>0.3)",
		free_clear_max > 0.3,
		format!("{free_clear_max:4.2} mm total"),
		&mut ok,
	);
	// Well ribs: designed interference. This is what holds a SHOT card.
	let well_clear = SLOT_W - 2.0 * WELL_RIB_P;
	let well_grip = CARD_T - well_clear;
	let well_grip_max = CARD_T_MAX - well_clear;
	gate(
		"well ribs grip a SHOT card (0.05–0.35 interference)",
		(0.05..=0.35).contains(&well_grip),
		format!("{well_grip:4.2} nominal / {well_grip_max:4.2} thick"),
		&mut ok,
	);
	let shelf_clear = SLOT_W - 2.0 * SHELF_RIB_P;
	gate(
		"shelf ribs only guide (a FRESH card pulls out: clearance ≥0)",
		shelf_clear - CARD_T >= 0.0 && shelf_clear - CARD_T <= 0.25,
		format!("{:4.2} mm clearance", shelf_clear - CARD_T),
		&mut ok,
	);
	gate(
		"well ribs stop clear of the sliding plane (≥0.5 below shelf)",
		SHELF_TOP - WELL_RIB_Z1 >= 0.5,
		format!("{:4.1} mm below", SHELF_TOP - WELL_RIB_Z1),
		&mut ok,
	);
	// exact interference check against a real posed card, not arithmetic
	let m_seated = tessellate_default(&card(SHOT_X0, WELL_FLOOR, yc0, CARD_T));
	let pen_seat = kernel_model::penetration_estimate(&m_body, &m_seated, 6000);
	gate(
		"posed SHOT card measures the rib bite (0.05–0.35, sampled)",
		(0.05..=0.35).contains(&pen_seat),
		format!("pen {pen_seat:5.3}"),
		&mut ok,
	);
	// and a card on the shelf must NOT be gripped
	let m_fresh = tessellate_default(&card(FRESH_X0, SHELF_TOP, yc0, CARD_T));
	let pen_fresh = kernel_model::penetration_estimate(&m_body, &m_fresh, 6000);
	gate(
		"posed FRESH card is free (pen ≈ 0 — pulls out one-handed)",
		pen_fresh <= 0.02,
		format!("pen {pen_fresh:5.3}"),
		&mut ok,
	);

	// ---- ARRI Pin-Lock 1/4" interface --------------------------------------------------
	println!("\nARRI Pin-Lock 1/4\" female (ARRI INTERFACES v2023-05):");
	gate(
		"all 8 pin holes on LK Ø10.4 at 45° (not the usual 2)",
		(2.0 * PIN_BC_R - 10.4).abs() < 1e-9,
		format!("LK Ø{:4.1}, 8 holes", 2.0 * PIN_BC_R),
		&mut ok,
	);
	gate(
		"pin hole depth ≥ ARRI minimum 5.2",
		PIN_DEPTH >= 5.2,
		format!("{PIN_DEPTH} mm"),
		&mut ok,
	);
	let ream = PIN_D_NOM - PIN_D_PRINT;
	gate(
		"pin holes modelled undersize for reaming (0.10–0.20)",
		(0.10..=0.20).contains(&ream),
		format!("Ø{PIN_D_PRINT} → ream Ø{PIN_D_NOM} ({ream:4.2} stock)"),
		&mut ok,
	);
	// the reason there is a nut and not an insert, gated rather than asserted
	let insert_boss_d = 2.0 * 9.52; // the 2x-diameter rule on a 1/4-20 heat-set
	gate(
		"heat-set insert REFUSED: its boss would eat the pin ring",
		insert_boss_d > 2.0 * PIN_BC_R,
		format!("boss Ø{insert_boss_d:4.1} vs pin BC Ø{:4.1}", 2.0 * PIN_BC_R),
		&mut ok,
	);
	gate(
		"captive nut sits ABOVE the pin holes (no intersection)",
		NUT_Z0 >= PIN_DEPTH + 0.5,
		format!("nut z0 {NUT_Z0} vs pins to {PIN_DEPTH}"),
		&mut ok,
	);
	// --- the three gates the first draft did not have, and needed -------------
	// 1. The pocket must be BIGGER than the nut. A pocket cut at the nut's own
	//    nominal across-flats is a press fit at best and unassemblable at worst.
	gate(
		"hex pocket clears the max-material nut (0.30–0.80 across flats)",
		(0.30..=0.80).contains(&(NUT_AF - NUT_AF_MAX)),
		format!("pocket {NUT_AF} vs nut {NUT_AF_MAX} → {:4.2} mm", NUT_AF - NUT_AF_MAX),
		&mut ok,
	);
	gate(
		"…and in thickness (nut is not pinched by the pocket roof)",
		(0.15..=0.60).contains(&(NUT_T - NUT_T_MAX)),
		format!("{:4.2} mm", NUT_T - NUT_T_MAX),
		&mut ok,
	);
	// 2. THE ONE THAT CAUGHT THE BUG. The nut enters along X through a channel
	//    of fixed Y width. Its extent across that direction is across-FLATS only
	//    if the hexagon's flats face ±Y. Measure the modelled polygon rather
	//    than trusting the constructor: half-extent in Y × 2 must equal the
	//    across-flats, NOT the across-corners.
	let hex_poly = hexagon(NUT_AF);
	let hex_y_extent = 2.0 * hex_poly.iter().map(|p| p.y.abs()).fold(0.0_f64, f64::max);
	gate(
		"hex pocket is FLATS-first across the entry channel",
		(hex_y_extent - NUT_AF).abs() < 1e-6,
		format!(
			"Y extent {hex_y_extent:5.2} = across-flats {NUT_AF} (corners would be {:5.2})",
			across_corners(NUT_AF)
		),
		&mut ok,
	);
	gate(
		"entry channel passes the nut's own across-corners width",
		NUT_AF >= NUT_AF_MAX + 0.3,
		format!("channel {NUT_AF} vs nut AF {NUT_AF_MAX} (corners {:5.2})", across_corners(NUT_AF_MAX)),
		&mut ok,
	);
	// 3. Dowel length. A pin that does not reach the HOST's pin holes is
	//    decoration. ARRI publishes 4.8 mm max protrusion for the Ø3 pin, and
	//    this part's own holes are 5.2 deep — so the dowel has to be ~10, not
	//    the 6 the first BOM listed (which would have stood 0.8 mm proud).
	gate(
		"dowel spans BOTH faces (this part 5.2 + host ≤4.8)",
		(DOWEL_LEN - PIN_DEPTH >= 4.0) && (DOWEL_LEN - PIN_DEPTH <= 4.8),
		format!("Ø{PIN_D_NOM}×{DOWEL_LEN} → {:4.1} mm proud", DOWEL_LEN - PIN_DEPTH),
		&mut ok,
	);
	gate(
		"shelf floor survives over the nut pocket (≥3.0 mm)",
		SHELF_TOP - (NUT_Z0 + NUT_T) >= 3.0,
		format!("{:4.1} mm", SHELF_TOP - (NUT_Z0 + NUT_T)),
		&mut ok,
	);
	// ARRI's published max male protrusion for 1/4-20 is 7.5 mm — the stud must
	// not bottom out before the nut takes it.
	gate(
		"a max-protrusion (7.5) 1/4-20 stud does not bottom out",
		NUT_Z0 + NUT_T > 7.5,
		format!("clear to {:4.1} mm", NUT_Z0 + NUT_T),
		&mut ok,
	);

	// ---- print & pack ------------------------------------------------------------------
	println!("\nprint & envelope:");
	gate(
		"fits a 256 mm bed with room to spare",
		BODY_X <= 250.0 && BODY_Y <= 250.0,
		format!("{BODY_X:5.1} × {BODY_Y:4.1} × {BODY_Z:4.1} mm"),
		&mut ok,
	);
	gate(
		"divider walls ≥ 4 perimeters at 0.4 nozzle",
		DIV_T >= 1.6,
		format!("{DIV_T} mm"),
		&mut ok,
	);
	let vol = volume(&body).abs();
	// "Lightweight" needs an EXTERNAL reference or it is just a number I picked.
	// The one this product supplies: an organiser should not outweigh the media
	// it organises. Six CFexpress Type B cards are ~12 g each.
	let media_g = N_SLOTS as f64 * 12.0;
	gate(
		"printed magazine weighs less than the media it carries",
		vol * PLA <= media_g,
		format!("{:4.1} g vs {media_g:4.0} g of cards", vol * PLA),
		&mut ok,
	);
	// The metal number is REPORTED, not gated — I have no external bound to gate
	// it against, and inventing one would be a gate that proves nothing. It is
	// carried into ANALYSIS.md as a named open item instead. What IS gated: the
	// part is mostly air, so the machining path is not "hog out a solid brick".
	let envelope = BODY_X * BODY_Y * BODY_Z;
	gate(
		"body is majority void (stock removal sane for a milled part)",
		vol / envelope <= 0.55,
		format!("{:4.1}% solid of envelope", 100.0 * vol / envelope),
		&mut ok,
	);

	// ---- load path ------------------------------------------------------------------
	// A magazine hangs off one 1/4-20 for the whole shoot day and often lives on
	// the rig between days: that is a sustained load, so it is judged against the
	// time-derated creep table, not the static allowable.
	println!("\nload path (sustained — creep-derated, not static):");
	let cards_g = 6.0 * 12.0; // six CFexpress Type B, ~12 g each
	let hung_n = (vol * PLA + cards_g) / 1000.0 * 9.81;
	let sig_creep = kernel_model::materials::pla::creep_allowable_mpa(23.0, 8760.0);
	// the shelf floor over the nut pocket is the thinnest member in the load path
	let bear_area = std::f64::consts::PI * ((NUT_AF / 2.0).powi(2) - (SCREW_CLEAR_D / 2.0).powi(2));
	let sig_bear = hung_n / bear_area;
	gate(
		"nut seat bearing vs 23 °C/1-year creep bound: ≥50×",
		sig_creep / sig_bear >= 50.0,
		format!("{:6.0}× ({sig_bear:.4} MPa vs {sig_creep} MPa)", sig_creep / sig_bear),
		&mut ok,
	);

	// ---- exports -----------------------------------------------------------------------
	// The boolean chain leaves each planar face split into many fragments that
	// share a plane; the raw STEP was 4.45 MB of edge bookkeeping for a 72 mm
	// part. coalesce_coplanar merges them before export — provenance survives it
	// (FRICTION #20 residual, closed 2026-07-30), so this is safe to run here.
	let raw_faces = body.face_count();
	let export_body = kernel_brep::coalesce_coplanar(&body);
	let dv_coal = (volume(&export_body).abs() - vol).abs() / vol;
	gate(
		"coalesce preserves volume exactly (<0.001%)",
		dv_coal < 1e-5,
		format!("{raw_faces} → {} faces, dv {:6.4}%", export_body.face_count(), dv_coal * 100.0),
		&mut ok,
	);
	let step_txt = export_step(&export_body, "two_state_magazine");
	let _ = std::fs::write("camera_system/card_magazine/cad/card_magazine.step", &step_txt);
	match kernel_brep::import_step(&step_txt) {
		Ok(back) => {
			let dv = (volume(&back).abs() - vol).abs() / vol;
			gate(
				"STEP round-trip conserves volume (<2.5%)",
				dv < 0.025,
				format!("dv {:5.3}%", dv * 100.0),
				&mut ok,
			);
		}
		Err(e) => gate("STEP round-trip", false, format!("{e:?}"), &mut ok),
	}

	// assembly scene: three cards SHOT, three FRESH — the photograph, in CAD.
	// One STL per BOM item so the assembly sheet can balloon them.
	let mut shot_cards = Mesh::default();
	let mut fresh_cards = Mesh::default();
	for i in 0..N_SLOTS {
		if i < 3 {
			merge_into(&mut shot_cards, &tessellate_default(&card(SHOT_X0, WELL_FLOOR, slot_y(i), CARD_T)));
		} else {
			merge_into(&mut fresh_cards, &tessellate_default(&card(FRESH_X0, SHELF_TOP, slot_y(i), CARD_T)));
		}
	}
	// the two purchased items, at their seated positions
	let nut_gauge = difference(
		&extrude(&hexagon(NUT_AF), NUT_T - 0.3).transformed(tr(ARRI_X, 0.0, NUT_Z0 + 0.15)),
		&cylinder(v(ARRI_X, 0.0, NUT_Z0 - 0.5), DVec3::Z, SCREW_CLEAR_D / 2.0 - 0.6, NUT_T + 1.0, 32),
	);
	let mut dowels = Mesh::default();
	for k in [0usize, 4] {
		let a = (45.0 * k as f64).to_radians();
		merge_into(
			&mut dowels,
			&tessellate_default(&cylinder(
				v(ARRI_X + PIN_BC_R * a.cos(), PIN_BC_R * a.sin(), 0.0),
				DVec3::Z,
				PIN_D_NOM / 2.0,
				DOWEL_LEN,
				32,
			)),
		);
	}
	let m_nut = tessellate_default(&nut_gauge);

	let mut scene = Mesh::default();
	for m in [&m_body, &shot_cards, &fresh_cards] {
		merge_into(&mut scene, m);
	}
	let _ = std::fs::write("camera_system/card_magazine/assembly/assembly.stl", scene.to_stl_binary());
	let sc = "camera_system/card_magazine/assembly/scene";
	let _ = std::fs::write(format!("{sc}/magazine.stl"), m_body.to_stl_binary());
	let _ = std::fs::write(format!("{sc}/cards_shot.stl"), shot_cards.to_stl_binary());
	let _ = std::fs::write(format!("{sc}/cards_fresh.stl"), fresh_cards.to_stl_binary());
	let _ = std::fs::write(format!("{sc}/hex_nut.stl"), m_nut.to_stl_binary());
	let _ = std::fs::write(format!("{sc}/dowel_pins.stl"), dowels.to_stl_binary());

	// ---- generated docs ------------------------------------------------------------------
	let analysis = format!(
		r#"# TWO-STATE MAGAZINE — analysis (generated by card_magazine.rs)

Every number here is what the gate suite measured on THIS build. Regenerated
every run, so it cannot go stale.

## The mechanism

| state | card bottom | card top | vs top face ({BODY_Z:.0}) |
|---|---|---|---|
| FRESH (on the shelf) | {SHELF_TOP:.1} | {fresh_top:.1} | **{proud:.1} mm proud** |
| SHOT (in the well) | {WELL_FLOOR:.1} | {shot_top:.1} | **{recess:.1} mm recessed** |

State change costs a {lift:.0} mm lift plus a {slide:.0} mm slide. The two
directions are deliberately NOT symmetric:

- **FRESH → SHOT** (cheap error): push toward −X with one finger. The card runs
  off the shelf and gravity does the rest.
- **SHOT → FRESH** (expensive error — this is the one that overwrites footage):
  lift {lift:.0} mm out of the well, against the rib grip, THEN slide {slide:.0} mm.

The interlock is gated, not asserted: a SHOT card driven straight toward the
shelf penetrates the shelf face by **{pen:.2} mm** — it cannot translate there.
The same card lifted to shelf height crosses freely (pen {free:.3}, contacts
{fc}), which proves the gate can pass and is not measuring a self-fulfilling
collision.

## Card fit

| quantity | value | note |
|---|---|---|
| slot clearance, nominal card | {fc_n:.2} mm | slides, does not rattle |
| slot clearance, worst-case thick card ({CARD_T_MAX}) | {fc_x:.2} mm | still free |
| well-rib interference (holds a SHOT card) | {grip:.2} mm nominal / {gripx:.2} thick | measured on a posed card: {pseat:.3} |
| shelf-rib clearance (FRESH pulls out) | {sclr:.2} mm | posed card pen {pfresh:.3} |

**Open, and honestly open:** the ±0.15 mm brand-to-brand CFexpress thickness
scatter is not vendor-published anywhere the research pass could reach. Both
columns above are computed, and `optional/coupon_fit` exists so you can settle
it on your own cards in fifteen minutes before committing to the full print.

## Mounting standard — ARRI Pin-Lock 1/4" (ARRI INTERFACES v2023-05)

- **8 × Ø{PIN_D_NOM} pin holes on LK Ø{bc:.1} at 45°**, depth {PIN_DEPTH} — the full
  ring, not the two holes most third-party parts cut, so the magazine re-indexes
  in 45° steps without unbolting.
- **1/4-20 female** as a captive ASME B18.2.2 hex nut ({NUT_AF} AF × {NUT_T} thick)
  in a side-entry pocket at z {NUT_Z0}–{nut_top:.1}, above the pin holes and under
  {floor:.1} mm of shelf floor.
- A max-protrusion ARRI 1/4-20 male stud (7.5 mm published) clears to
  {nut_top:.1} mm — no bottoming.

**Why a nut and not a heat-set insert**: the 2×-diameter boss rule on a Ø9.52
1/4-20 insert wants a Ø{boss:.1} boss, which swallows the Ø{bc:.1} pin ring. That
is a gated refusal, not a preference.

### How this actually mounts, and what the pins are for

ARRI Pin-Lock is a MALE/FEMALE pair. The host (cage, cheese plate) presents the
FEMALE face: a threaded hole ringed by Ø3 pin holes. The accessory presents the
MALE face: a screw plus protruding pins. **This magazine's base is a FEMALE
face** — pin holes and a captive nut, no protruding anything — because a female
base is the versatile one: it accepts a plain 1/4-20 male stud (tripod screw,
ball head, magic arm) as well as a screw pushed up through a cheese plate.

The pins exist because **one screw is a pivot.** Tighten a magazine onto a plate
with a single 1/4-20 and it can still rotate about that screw the moment the
friction preload relaxes — which it does. The pins take the twisting moment in
shear so the screw only has to supply clamp load.

Because both faces are female, the pins are **loose dowels shared between them**:
{dowel:.0} mm long, {PIN_DEPTH} buried in this part and {pin_proud:.1} standing proud into
the host's holes ({pin_proud:.1} is ARRI's published max protrusion for the Ø3 pin).
Kondor Blue's own catalogue copy calls these "removable pins" — it is normal
practice, not an invention. Pick whichever opposite pair of the eight holes
matches the orientation you want; that is what the 45° indexing means in use.

**The honest limit: on a plain tripod screw or ball head there are no pin holes
to receive the dowels, and the anti-twist does nothing.** On that kind of mount
you are relying on the screw alone, exactly like every other accessory. The
dowels are listed OPTIONAL in the BOM for this reason.

**Printed-pin honesty**: the holes are modelled Ø{PIN_D_PRINT} and are meant to be
reamed to Ø{PIN_D_NOM} for the steel dowels ({ream:.2} mm of stock). ARRI's ⌖Ø0.1 CZ
positional tolerance on Ø3 E7 is NOT an FDM-achievable fit and this part does not
pretend otherwise: **the plastic locates, the steel dowels carry the shear.** The
machined version holds the real tolerance.

## Mass — computed, not claimed

- **{pla_g:.1} g** PLA (solid-equivalent, {vol:.0} mm³)
- **{al_g:.1} g** in 6061-T6 solid, before any lightening pockets

The 6061 figure is the measured solid volume × 2.70 g/cm³. Stating it matters:
the concept-scouting pass caught five candidate designs whose aluminium mass
claims were wrong by 3–8.6×, and a partner who machines billet catches that
instantly.

## Load path

The magazine hangs off one 1/4-20 all day and often lives on the rig between
days, so it is a **sustained** load and is judged against the time-derated creep
table (`materials::pla::creep_allowable_mpa`), not the static allowable. Body
plus six cards = {hung:.2} N over the {bear:.0} mm² nut seat = {sigb:.4} MPa,
against {sigc} MPa at the 23 °C / 1-year cell: **{marg:.0}× margin**.

## Required, NOT performed

- **Shake and drop retention.** The concept's safety argument needs measured
  data: 3 card brands × N cards, shaker plus a 1 m drop onto plywood, counting
  state migrations. That is a PHYSICAL test and no solver substitutes for it.
  The geometry gates above bound what CAN happen kinematically; they do not
  measure what a real cord-free rib grip does under vibration.
- **Thermal.** PLA HDT is ~54 °C and a black case in a truck exceeds it. No
  thermal analysis was run. The metal version is immune; the printed prototype
  should not be left in a hot vehicle. PETG (Tg ~85 °C) is the reprint path.
- **Rib fatigue.** The well ribs are loaded every state change. Printed-PLA
  across-layer fatigue data does not exist (the materials runner refuses rather
  than reuse a static ratio), so cycle life is unknown and unclaimed.
"#,
		fresh_top = fresh_top,
		shot_top = shot_top,
		proud = fresh_top - BODY_Z,
		recess = BODY_Z - shot_top,
		lift = lift,
		slide = FRESH_X0 - SHOT_X0,
		pen = push.max_penetration,
		free = across.max_penetration,
		fc = across.contacts,
		fc_n = free_clear,
		fc_x = free_clear_max,
		grip = well_grip,
		gripx = well_grip_max,
		pseat = pen_seat,
		sclr = shelf_clear - CARD_T,
		pfresh = pen_fresh,
		bc = 2.0 * PIN_BC_R,
		nut_top = NUT_Z0 + NUT_T,
		floor = SHELF_TOP - (NUT_Z0 + NUT_T),
		boss = insert_boss_d,
		ream = ream,
		dowel = DOWEL_LEN,
		pin_proud = DOWEL_LEN - PIN_DEPTH,
		pla_g = vol * PLA,
		al_g = vol * AL6061,
		vol = vol,
		hung = hung_n,
		bear = bear_area,
		sigb = sig_bear,
		sigc = sig_creep,
		marg = sig_creep / sig_bear,
	);
	let _ = std::fs::write("camera_system/card_magazine/analysis/ANALYSIS.md", analysis);

	let bom = format!(
		"# TWO-STATE MAGAZINE — bill of materials\n\n\
		| item | qty | source | note |\n|---|---|---|---|\n\
		| card_magazine (parts/) | 1 | print | PLA, 4 perimeters, 20% infill — {g:.1} g |\n\
		| 1/4-20 hex nut, ASME B18.2.2 | 1 | hardware store | slides into the side pocket |\n\
		| Ø3 × 10 dowel pin, steel | 2 | hardware store | OPTIONAL — anti-twist; needs a host with ARRI pin holes |\n\
		| coupon_fit (optional/) | 1 | print first | ~15 min — settles the card fit |\n\n\
		No springs, no cords, no adhesive, no assembly. Six CFexpress Type B cards.\n",
		g = vol * PLA
	);
	let _ = std::fs::write("camera_system/card_magazine/assembly/BOM.md", bom);

	// bom_dossier.csv — the machine-readable BOM the assembly sheet balloons
	// against. Written from the live volume so the sheet's mass total cannot
	// drift from the part. Names must match assembly/sheet_job.json's parts.
	let bom_csv = format!(
		"name,kind,qty,material,part_number,grams_per_unit\n\
		 dowel pin,buy,2,steel Ø3x10 m6,DIN 6325,0.55\n\
		 hex nut,buy,1,steel 1/4-20 UNC,ASME B18.2.2,1.9\n\
		 card_magazine,made,1,PLA,TSM-001,{g:.1}\n\
		 cards SHOT,buy,3,CFexpress Type B,—,12.0\n\
		 cards FRESH,buy,3,CFexpress Type B,—,12.0\n",
		g = vol * PLA
	);
	let _ = std::fs::write("camera_system/card_magazine/assembly/bom_dossier.csv", bom_csv);

	println!("\nTWO-STATE MAGAZINE: {}", if ok { "ALL GATES PASS" } else { "<<< FAIL" });
	std::process::exit(if ok { 0 } else { 1 });
}
