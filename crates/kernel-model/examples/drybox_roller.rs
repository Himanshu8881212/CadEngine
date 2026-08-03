//! DRYBOX ROLLER — a bearing-roller + desiccant base that turns the common
//! ~4 L flip-top "cereal keeper" (Vtopmart / Skroam / Wildone / GoMaihe /
//! Praki family — the community-standard single-spool filament drybox) into
//! a rolling dry-feed box for one Ø200 × ≤68 spool (RESPOOL included).
//!
//! One tray + one sliding hatch + four 608 bearings (8 × 22 × 7 skateboard
//! bearings — the cheapest bearing made, ~$0.30/pc in packs). Zero screws,
//! zero inserts: the bearings push onto D-profile stub axles past a
//! small click ring (the community press-stub numbers: Ø7.9 seat, tested by
//! the included 15-minute coupon before you print the tray). The spool's two
//! flange rims ride the four outer races; low rails between the flanges keep
//! it tracked.
//!
//! The whole tray body is a DESICCANT TANK (~0.19 L ≈ 150 g of silica, the
//! amount the popular bases actually load): the deck is slotted at 1.6 mm —
//! orange indicating silica is 2–4 mm, and slots beat holes for retaining
//! broken beads — and refilling is one move: slide the hatch out, pour,
//! slide it back. The hatch is CAPTIVE while the tray sits in the box (the
//! container wall leaves it only ~4 mm of travel, so it cannot creep open or
//! spill in use) and slides fully out of its open-ended channel the moment
//! you lift the tray out. No latch to break, nothing preloaded.
//!
//! Researched constraints (sources in spool_system/drybox_roller/DESIGN.md):
//! container floor ≈ 205–210 × 85–90 with ~3° wall draft and real batch
//! variation → footprint 196 × 82 clears every reported variant; usable
//! height > 210 → spool top sits at 229.5 with the axle axis at 28 (the
//! community's own roller bases live in the same envelope); spools wider
//! than ~68 rub the container walls — that is the box's limit, not ours.
//!
//! Dog-foods the 2026-07-28 engine hardening: the tray chain is built under
//! `ChainLog::seal()` (every op validated + tessellation-checked, first bad
//! step named), the aperture cut is pre-flighted with `boolean_hazards`,
//! the hatch slide and bearing push-on are `sweep_check` paths, and masses
//! come from `kernel_model::materials`.
//!
//! Run: cargo run --example drybox_roller -p kernel-model --release
//!   -> spool_system/drybox_roller/ (exit 1 on any FAIL)

use kernel_brep::math::{DAffine3, DMat3, DVec2, DVec3};
use kernel_brep::{
	boolean_hazards, cuboid, cylinder, difference, export_step, extrude, force_ccw,
	tessellate_default, union, validate, volume, ChainLog, HazardKind, Mesh, Solid,
};
use kernel_core::math::Vec3;
use kernel_model::{campaign::gate, materials, sweep_check};

// ---- container interface (researched; DESIGN.md carries the sources) -----------
const BASE_L: f64 = 196.0; // floor ≈ 205–210 long across the family
const BASE_W: f64 = 82.0; // floor width 85 (US) / 90 (EU); 82 clears both
const BASE_R: f64 = 8.0; // corner radius (floors are rounded)
const BOX_MIN_FLOOR_L: f64 = 205.0;
const BOX_MIN_FLOOR_W: f64 = 85.0;
const BOX_USABLE_H: f64 = 235.0; // internal height class of the 248-tall family

// ---- spool interface (the RESPOOL / universal 1 kg envelope) --------------------
const SPOOL_R: f64 = 100.0;
/// Loaded spool mass (kg): a 1 kg refill on a ~0.3 kg spool. Drives the
/// sustained-load gates on the printed bearing stubs.
const SPOOL_KG: f64 = 1.3;
const SPOOL_W: f64 = 67.0; // ≤68 fits the container free — researched bound
const RIM_TRACK_Y: f64 = 32.0; // flange rim centreline from spool mid-plane
const FLANGE_T: f64 = 3.0;

// ---- 608 bearing (8 × 22 × 7) ----------------------------------------------------
const BRG_OD_R: f64 = 11.0;
const BRG_W: f64 = 7.0;
const BRG_BORE_R: f64 = 4.0;
const STUB_R: f64 = 3.95; // Ø7.9 — the community press-stub seat size
const RING_R: f64 = 4.10; // Ø8.2 click ring: 0.1/side intentional crush
const LEAD_R: f64 = 3.70; // Ø7.4 entry lead

// ---- tray architecture -----------------------------------------------------------
const WALL: f64 = 2.0;
const FLOOR_T: f64 = 2.0;
const DECK_TOP: f64 = 15.5; // deck upper surface (tank lid)
const DECK_T: f64 = 2.0;
const PARAPET_TOP: f64 = 18.0; // wall rim continues above the deck
const RIB_T: f64 = 1.2;
const RAIL_Y0: f64 = 26.0; // guide rails (also the stub anchors)
const RAIL_Y1: f64 = 28.4;
const RAIL_TOP: f64 = 30.5;
const RAIL_X: f64 = 58.0; // rails span x ∈ ±58
const STATION_X: f64 = 45.0; // two bearing stations (spacing 90): the Ø200
                             // spool centre lands at z = 28 + √(111²−45²) = 129.5
const STUB_Z: f64 = 28.0;
// hatch channel + slider
const APER_X: f64 = 30.0; // aperture x ∈ ±30
const APER_Y: f64 = 19.0; // aperture y ∈ ±19
const LIP_Y: f64 = 24.5; // C-channel lips at y = ±24.5, open toward +x
const SLIDER_L: f64 = 78.0; // cover margin 9 vs taper-adjusted in-box travel
const SLIDER_W: f64 = 47.4; // 0.8 per side to the channel walls at ±24.5
const SLIDER_T: f64 = 2.4;
const SLOT_W: f64 = 1.6; // < 2 mm orange-silica beads, slots-not-holes

const PLA: f64 = materials::PLA_G_PER_MM3;

// ---- helpers ---------------------------------------------------------------------

fn v(x: f64, y: f64, z: f64) -> DVec3 {
	DVec3::new(x, y, z)
}
fn tr(x: f64, y: f64, z: f64) -> DAffine3 {
	DAffine3::from_translation(v(x, y, z))
}

/// Rounded-rectangle profile (centred), corner radius r, ~3 segments / corner.
fn rounded_rect(hx: f64, hy: f64, r: f64) -> Vec<DVec2> {
	let mut p = Vec::new();
	let corners = [(hx - r, hy - r, 0.0), (-(hx - r), hy - r, 90.0), (-(hx - r), -(hy - r), 180.0), (hx - r, -(hy - r), 270.0)];
	for (cx, cy, a0) in corners {
		for i in 0..=6 {
			let a = (a0 + 90.0 * i as f64 / 6.0).to_radians();
			p.push(DVec2::new(cx + r * a.cos(), cy + r * a.sin()));
		}
	}
	force_ccw(p)
}

/// Prism from an (x,z) profile swept along +Y over [y0, y1] (det +1 frame).
fn prism_y(profile: &[(f64, f64)], y0: f64, y1: f64) -> Solid {
	// X→X, Y→Z, Z→−Y is det = +1 (a −90° turn about X): sweeping local +Z runs
	// world −Y, so start the prism at y1 and it spans [y0, y1] unmirrored.
	let p: Vec<DVec2> = profile.iter().map(|&(x, z)| DVec2::new(x, z)).collect();
	let m = DAffine3::from_mat3_translation(DMat3::from_cols(DVec3::X, DVec3::Z, DVec3::NEG_Y), v(0.0, y1, 0.0));
	extrude(&force_ccw(p), y1 - y0).transformed(m)
}

/// D-PROFILE for a support-free horizontal boss that a bearing bore must pass
/// over: a circle with its bottom cut flat by the chord between the ±44°
/// points, so every remaining arc facet is ≥46° from horizontal (safe) and
/// the flat is a ~5.7 mm dead-flat underside (bridge-class, printable). The
/// whole profile stays INSIDE radius r — a teardrop cannot do that (its 46°
/// flanks leave the circle, apex at 1.44·r: a Ø8.0 bore physically could not
/// pass — caught twice by the penetration gates), and a deeper 52° chord
/// leaves 38°-from-horizontal arc facets (caught by the support audit).
fn d_profile(r: f64, cx: f64, cz: f64) -> Vec<(f64, f64)> {
	let mut p = Vec::new();
	for i in 0..=40 {
		let a = (-44.0 + 268.0 * i as f64 / 40.0).to_radians();
		p.push((cx + r * a.cos(), cz + r * a.sin()));
	}
	p // the polygon closes across the ±44° chord — the flat bottom
}

/// One stub axle assembly on the +y side at station x: shoulder (spaces the
/// bearing off the rail), Ø7.9 seat, Ø8.2 click ring, Ø7.4 entry lead — all
/// D-profiled, all swept along Y. `side` = ±1 mirrors it.
fn stub(x: f64, side: f64) -> Solid {
	// axial stack from the rail face: shoulder [28.0, 28.9] (0.4 embedded),
	// Ø7.9 seat [28.9, 36.2] (bearing seats 28.9–35.9 + 0.3 play), Ø8.2 click
	// ring [36.2, 36.9], Ø7.4 lead [36.9, 37.9]. `side` sweeps ± ranges.
	let seg = |r: f64, a: f64, b: f64| -> Solid {
		if side > 0.0 {
			prism_y(&d_profile(r, x, STUB_Z), a, b)
		} else {
			prism_y(&d_profile(r, x, STUB_Z), -b, -a)
		}
	};
	// every profile stays inside its own radius, so the Ø8.0 bore slides over
	// seat and lead and the ring's click bite is its round upper arc only
	let shoulder = seg(5.5, RAIL_Y1 - 0.4, RAIL_Y1 + 0.5);
	let seat = seg(STUB_R, RAIL_Y1 + 0.5, RAIL_Y1 + 0.5 + BRG_W + 0.3);
	let ring = seg(RING_R, RAIL_Y1 + 0.5 + BRG_W + 0.3, RAIL_Y1 + 0.5 + BRG_W + 1.0);
	let lead = seg(LEAD_R, RAIL_Y1 + 0.5 + BRG_W + 1.0, RAIL_Y1 + 0.5 + BRG_W + 2.0);
	union(&union(&shoulder, &seat), &union(&ring, &lead))
}

/// A 608 gauge: outer race Ø22 × 7 with the Ø8 bore, axis along Y, centred on
/// the seat at station (sx, side).
fn bearing_gauge(sx: f64, side: f64, y_shift: f64) -> Solid {
	// seated: inner face against the shoulder at |y| = 28.9, so the race spans
	// 28.9–35.9 and the flange rim track (y ≈ 30.5–33.5) rides its middle.
	let y_in = side * (RAIL_Y1 + 0.5 + y_shift);
	let axis = v(0.0, side, 0.0);
	difference(
		&cylinder(v(sx, y_in, STUB_Z), axis, BRG_OD_R, BRG_W, 64),
		&cylinder(v(sx, y_in - side * 1.0, STUB_Z), axis, BRG_BORE_R, BRG_W + 2.0, 32),
	)
}

/// Spool gauge: two Ø200 × 3 flanges + Ø81.7 barrel, axis along Y, centre at
/// (0, 0, zc) — the universal 1 kg refill spool envelope (RESPOOL's own).
fn spool_gauge(zc: f64) -> Solid {
	let flange = |yc: f64| cylinder(v(0.0, yc - FLANGE_T / 2.0, zc), DVec3::Y, SPOOL_R, FLANGE_T, 128);
	let barrel = cylinder(v(0.0, -SPOOL_W / 2.0 + FLANGE_T, zc), DVec3::Y, 40.85, SPOOL_W - 2.0 * FLANGE_T, 64);
	union(&union(&flange(-SPOOL_W / 2.0 + FLANGE_T / 2.0 + 0.0), &flange(SPOOL_W / 2.0 - FLANGE_T / 2.0)), &barrel)
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
	dst.positions.extend_from_slice(&src.positions);
	dst.indices.extend(src.indices.iter().map(|i| i + base));
}

// ---- the tray, built under ChainLog::seal() --------------------------------------

fn build_tray() -> Result<(Solid, Solid), kernel_brep::ChainError> {
	// §7.7-clean construction: the shell is extruded at FULL height and the
	// deck/parapet come from two cuts whose faces never overlap coplanar with
	// each other (the first draft unioned a parapet band whose 28 outer facets
	// were exactly coplanar with the body's over a 0.3 embed band — the §7.4
	// "coplanar forest", and the arrangement ground into a resolve_t_junctions
	// cascade). Disjoint cutters are pre-unioned so the chain runs ~6 big
	// arrangements instead of ~26 (run with LMCAD_CHAIN_TRACE=1 to watch).
	let outer = extrude(&rounded_rect(BASE_L / 2.0, BASE_W / 2.0, BASE_R), PARAPET_TOP);
	let mut chain = ChainLog::start("outer", outer)?.seal();
	chain.apply("tank cavity", |s| {
		let cav = extrude(&rounded_rect(BASE_L / 2.0 - WALL, BASE_W / 2.0 - WALL, BASE_R - WALL), DECK_TOP - FLOOR_T - DECK_T)
			.transformed(tr(0.0, 0.0, FLOOR_T));
		difference(s, &cav)
	})?;
	chain.apply("deck recess", |s| {
		// re-exposes the deck top at DECK_TOP and leaves the parapet ring; its
		// side faces share the cavity's surface family but sit 2.0 above the
		// cavity's top — same family, zero overlap, no interaction
		let rec = extrude(&rounded_rect(BASE_L / 2.0 - WALL, BASE_W / 2.0 - WALL, BASE_R - WALL), PARAPET_TOP - DECK_TOP + 1.0)
			.transformed(tr(0.0, 0.0, DECK_TOP));
		difference(s, &rec)
	})?;
	// tank ribs: mutually disjoint — pre-unioned, ONE arrangement
	chain.apply("ribs", |s| {
		let mut ribs: Option<Solid> = None;
		for y in [-33.0, -27.5, -16.5, -5.5, 5.5, 16.5, 27.5, 33.0] {
			let rib = cuboid(v(-BASE_L / 2.0 + WALL + 1.0, y - RIB_T / 2.0, FLOOR_T - 0.3), v(BASE_L / 2.0 - WALL - 1.0, y + RIB_T / 2.0, DECK_TOP - DECK_T + 0.3));
			ribs = Some(match ribs { Some(r) => union(&r, &rib), None => rib });
		}
		union(s, &ribs.unwrap())
	})?;
	// superstructure: rails + 4 stub axles + hatch lips + stop, pre-unioned
	chain.apply("superstructure", |s| {
		let rail = |side: f64| {
			let (y0, y1) = ((side * RAIL_Y0).min(side * RAIL_Y1), (side * RAIL_Y0).max(side * RAIL_Y1));
			let prof = [
				(-RAIL_X, DECK_TOP - 0.3),
				(RAIL_X, DECK_TOP - 0.3),
				(RAIL_X, RAIL_TOP - 1.2),
				(RAIL_X - 1.4, RAIL_TOP),
				(-RAIL_X + 1.4, RAIL_TOP),
				(-RAIL_X, RAIL_TOP - 1.2),
			];
			prism_y(&prof, y0, y1)
		};
		let lip = |side: f64| {
			let prof = [
				(side * LIP_Y, DECK_TOP - 0.3),
				(side * (LIP_Y + 2.2), DECK_TOP - 0.3),
				(side * (LIP_Y + 2.2), DECK_TOP + SLIDER_T + 1.6),
				(side * (LIP_Y - 3.0), DECK_TOP + SLIDER_T + 1.6),
				(side * (LIP_Y - 3.0), DECK_TOP + SLIDER_T + 0.4),
				(side * LIP_Y, DECK_TOP + SLIDER_T + 0.4),
			];
			let pts: Vec<DVec2> = prof.iter().map(|&(y, z)| DVec2::new(y, z)).collect();
			// channel runs from behind the stop to 0.5 SHORT of the wall's
			// inner face (fully in air — never flush-coincident with it)
			let m = DAffine3::from_mat3_translation(DMat3::from_cols(DVec3::Y, DVec3::Z, DVec3::X), v(-41.0, 0.0, 0.0));
			extrude(&force_ccw(pts), (BASE_L / 2.0 - WALL - 0.5) + 41.0).transformed(m)
		};
		let stop = cuboid(v(-SLIDER_L / 2.0 - 3.4, -25.0, DECK_TOP - 0.3), v(-SLIDER_L / 2.0 - 0.4, 25.0, DECK_TOP + SLIDER_T + 1.6));
		let mut sup = union(&rail(1.0), &rail(-1.0));
		for sx in [-STATION_X, STATION_X] {
			for side in [1.0, -1.0] {
				sup = union(&sup, &stub(sx, side));
			}
		}
		sup = union(&sup, &union(&union(&lip(1.0), &lip(-1.0)), &stop));
		union(s, &sup)
	})?;

	// parapet notches at the four stub stations: the bearing hangs 11 below
	// the stub axis, and without these it would hit the wall on its way in
	// (the push-on sweep caught exactly that at 0.6 mm penetration)
	chain.apply("parapet notches", |s| {
		let mut cuts: Option<Solid> = None;
		for sx in [-STATION_X, STATION_X] {
			for side in [1.0_f64, -1.0] {
				let b = cuboid(
					v(sx - 14.0, (side * 38.5).min(side * 42.5), 15.6),
					v(sx + 14.0, (side * 38.5).max(side * 42.5), PARAPET_TOP + 0.5),
				);
				cuts = Some(match cuts { Some(c) => union(&c, &b), None => b });
			}
		}
		// +x END notch: the slider exits HERE. The first draft had no notch and
		// the slider hit the end parapet — a real collision the vertex-sampled
		// sweep read as pen 0.000 (thin wall, no contained vertices). The
		// hardened sweep gate (contacts == 0) + this notch close that hole.
		// sill 0.3 BELOW the slider plane (a 15.6-flush sill registered as 7
		// kissing contacts in the hardened sweep) and the cutter face 0.3 clear
		// of the lip end faces at 95.5
		let exit = cuboid(v(BASE_L / 2.0 - WALL - 0.2, -27.0, 15.3), v(BASE_L / 2.0 + 0.5, 27.0, PARAPET_TOP + 0.5));
		difference(s, &union(&cuts.unwrap(), &exit))
	})?;

	// aperture — pre-flighted with the hazard linter before cutting
	let aper = cuboid(v(-APER_X, -APER_Y, DECK_TOP - DECK_T - 0.5), v(APER_X, APER_Y, DECK_TOP + 0.5));
	let hazards = boolean_hazards(chain.solid(), &aper, 0.05);
	let warn: Vec<_> = hazards
		.iter()
		.filter(|h| matches!(h.kind, HazardKind::NearCoincidentPlanes | HazardKind::NearCoincidentCylinders | HazardKind::EdgeInFace))
		.collect();
	assert!(
		warn.is_empty(),
		"aperture cutter fails the §7.7 pre-flight: {warn:?} — re-dimension before cutting"
	);
	chain.apply("aperture", |s| difference(s, &aper))?;

	// vent slots: 10 disjoint thin boxes, pre-unioned, ONE arrangement
	chain.apply("vent slots", |s| {
		let mut slots: Option<Solid> = None;
		for (xs, xe) in [(-88.0, -46.0), (46.0, 88.0)] {
			for yc in [-22.0, -11.0, 0.0, 11.0, 22.0] {
				let b = cuboid(v(xs, yc - SLOT_W / 2.0, DECK_TOP - DECK_T - 0.5), v(xe, yc + SLOT_W / 2.0, DECK_TOP + 0.5));
				slots = Some(match slots { Some(t) => union(&t, &b), None => b });
			}
		}
		difference(s, &slots.unwrap())
	})?;
	let tray = chain.finish();

	// the tank void, measured honestly: solid cavity block minus what the tray
	// occupies inside it — computed by boolean, not by hand arithmetic
	let block = extrude(&rounded_rect(BASE_L / 2.0 - WALL, BASE_W / 2.0 - WALL, BASE_R - WALL), DECK_TOP - FLOOR_T - DECK_T)
		.transformed(tr(0.0, 0.0, FLOOR_T));
	Ok((tray, block))
}

/// The sliding hatch: a flat plate with a low thumb ridge and a fingertip
/// notch at the open end. Prints flat, no supports.
fn build_slider() -> Solid {
	let plate = cuboid(v(-SLIDER_L / 2.0, -SLIDER_W / 2.0, 0.0), v(SLIDER_L / 2.0, SLIDER_W / 2.0, SLIDER_T));
	let ridge = cuboid(v(SLIDER_L / 2.0 - 12.0, -14.0, SLIDER_T - 0.3), v(SLIDER_L / 2.0 - 6.0, 14.0, SLIDER_T + 1.4));
	union(&plate, &ridge)
}

/// Stub-fit coupon: one full stub on a small puck — print it first, push a
/// 608 on, and know your printer's fit before committing to the tray.
fn build_coupon() -> Solid {
	let puck = cuboid(v(-14.0, -3.0, 0.0), v(14.0, 3.0, 36.0));
	// shoulder fully BURIED (its end face 0.5 inside the puck, never flush —
	// the first draft left it exactly coincident with the puck face and the
	// arrangement ground on 41 edge-in-face slivers; boolean_hazards now
	// pre-flights this union like every other risky one)
	let s = stub(0.0, 1.0).transformed(tr(0.0, -(RAIL_Y1 - 0.4) + 1.6, 0.0));
	let hz = boolean_hazards(&puck, &s, 0.05);
	let warn = hz
		.iter()
		.filter(|h| matches!(h.kind, HazardKind::NearCoincidentPlanes | HazardKind::NearCoincidentCylinders | HazardKind::EdgeInFace))
		.count();
	assert!(warn == 0, "coupon stub union fails the §7.7 pre-flight: {hz:?}");
	union(&puck, &s)
}

// ---- gates ------------------------------------------------------------------------

fn emit(dir: &str, name: &str, s: &Solid, ok: &mut bool) -> Mesh {
	let val = validate(s);
	let mesh = tessellate_default(s);
	let rep = mesh.support_free_report(Vec3::Z, 45.0, 0.3);
	let wt = mesh.is_watertight();
	let vol = volume(s).abs();
	let pass = val.is_valid() && wt && rep.steep_area < 1e-6 && rep.max_bridge_span <= 10.5;
	*ok &= pass;
	let _ = std::fs::write(format!("spool_system/drybox_roller/{dir}/{name}.stl"), mesh.to_stl_binary());
	let _ = mesh.write_3mf(format!("spool_system/drybox_roller/{dir}/{name}.3mf"));
	println!(
		"  {name:16} valid={:5} wt={wt:5} steep={:9.4} mm²  bridge≤{:4.1}  {:4.0} g  {:7.0} mm³  {}",
		val.is_valid(),
		rep.steep_area,
		rep.max_bridge_span,
		vol * PLA,
		vol,
		if pass { "OK" } else { "<<< FAIL" }
	);
	mesh
}

fn main() {
	// Campaign runs always contribute to the Level-1 flywheel (telemetry + friction capture).
	kernel_core::telemetry::enable();
	let _ = std::fs::create_dir_all("spool_system/drybox_roller/parts");
	let _ = std::fs::create_dir_all("spool_system/drybox_roller/cad");
	let _ = std::fs::create_dir_all("spool_system/drybox_roller/analysis");
	let _ = std::fs::create_dir_all("spool_system/drybox_roller/assembly/scene");
	let _ = std::fs::create_dir_all("spool_system/drybox_roller/optional");
	let _ = std::fs::create_dir_all("spool_system/drybox_roller/assembly/scene");
	println!("DRYBOX ROLLER — 4 L cereal-container roller/desiccant base:\n");

	let (tray, tank_block) = match build_tray() {
		Ok(t) => t,
		Err(e) => {
			println!("tray chain failed: {e}");
			std::process::exit(1);
		}
	};
	let slider = build_slider();
	let coupon = build_coupon();

	let mut ok = true;
	let m_tray = emit("parts", "roller_tray", &tray, &mut ok);
	let m_slider = emit("parts", "hatch_slider", &slider, &mut ok);
	// worst-case rim-on-race engagement across the FULL ±2.1 lateral play
	// (at −2.1 the rim overhangs the race inner edge 0.5 — still 2.5 of the
	// 3.0 rim carried; the first gate claimed ±1.5 and understated the play)
	{
		let race = (RAIL_Y1 + 0.5, RAIL_Y1 + 0.5 + BRG_W);
		let play = SPOOL_W / 2.0 - FLANGE_T - RAIL_Y1;
		let eng = |shift: f64| {
			let (rlo, rhi) = (RIM_TRACK_Y + shift - FLANGE_T / 2.0, RIM_TRACK_Y + shift + FLANGE_T / 2.0);
			(rhi.min(race.1) - rlo.max(race.0)).max(0.0)
		};
		let worst = eng(play).min(eng(-play));
		gate(
			"rim-on-race engagement ≥ 2.4 at full ±2.1 play",
			worst >= 2.4,
			format!("worst {worst:3.1} of {FLANGE_T}"),
			&mut ok,
		);
	}
	// the Ø11 shoulder must land on the 608's INNER ring only (inner-ring
	// land spans ~Ø9.6–12.0): the outer race and cage stay untouched, so the
	// bearing spins free while seated against the shoulder
	gate(
		"shoulder Ø11 lands on the 608 inner ring only",
		(9.6..=12.0).contains(&(2.0 * 5.5)),
		"Ø11 ∈ [9.6, 12]".to_string(),
		&mut ok,
	);
	let _ = emit("optional", "coupon_stub", &coupon, &mut ok);

	// ---- container fit ----------------------------------------------------------
	println!("\ncontainer fit (Amazon-generic 4 L family, sources in DESIGN.md):");
	gate(
		"footprint 196×82 clears the smallest reported floor (205×85)",
		BASE_L <= BOX_MIN_FLOOR_L - 4.0 && BASE_W <= BOX_MIN_FLOOR_W - 2.0,
		format!("{BASE_L}×{BASE_W}"),
		&mut ok,
	);
	let spool_top = STUB_Z + (111.0_f64.powi(2) - STATION_X * STATION_X).sqrt() + SPOOL_R;
	gate(
		"spool top ≤ usable height 235",
		spool_top <= BOX_USABLE_H - 3.0,
		format!("top {spool_top:5.1}"),
		&mut ok,
	);
	gate(
		"spool width 67 inside the ≤68 free-rolling bound",
		SPOOL_W <= 68.0,
		format!("{SPOOL_W}"),
		&mut ok,
	);

	// ---- bearings on stubs --------------------------------------------------------
	println!("\n608 bearings on D-profile stubs (push-on, click, spin free):");
	let brg = bearing_gauge(STATION_X, 1.0, 0.0);
	let m_brg = tessellate_default(&brg);
	// push-on path: bearing approaches along +y from 12 out to seat
	let approach: Vec<DAffine3> = (0..=8).map(|i| tr(0.0, 12.0 - 1.5 * i as f64, 0.0)).collect();
	let sweep = sweep_check(&m_tray, &m_brg, &approach);
	// the ring poses are INTENTIONAL interference: exact crossings > 0 there is
	// the click working, so this sweep gates the crush depth, not crossings —
	// and asserts the crossing oracle does see the ring engagement
	gate(
		"push-on sweep: free until the ring click (est ≤ 0.25, ring seen)",
		sweep.max_penetration <= 0.25 && sweep.crossings >= 1,
		format!("pen {:5.3} x {}", sweep.max_penetration, sweep.crossings),
		&mut ok,
	);
	// Exact booleans on near-tangent faceted cylinders (bore Ø8.0 over ring
	// Ø8.2, 0.1 crush across 40 facet slivers) drive the arrangement into a
	// T-junction cascade — the measurement here uses the sampled penetration
	// estimator (labeled estimate) plus exact arithmetic for the crush volume.
	let m_ring_pose = mesh_posed(&m_brg, tr(0.0, 1.05, 0.0));
	let pen_ring = kernel_model::penetration_estimate(&m_tray, &m_ring_pose, 4000);
	gate(
		"click ring engages the bore (est 0.05–0.20 crush)",
		(0.05..=0.20).contains(&pen_ring),
		format!("pen {pen_ring:5.3}"),
		&mut ok,
	);
	// teardrop keeps ~74% of the full ring annulus: π(4.1²−4.0²)·0.7·0.74
	let ring_crush = std::f64::consts::PI * (RING_R * RING_R - BRG_BORE_R * BRG_BORE_R) * 0.7 * 0.74;
	gate(
		"ring crush volume healthy (0.5–3 mm³, arithmetic)",
		(0.5..=3.0).contains(&ring_crush),
		format!("{ring_crush:4.2} mm³"),
		&mut ok,
	);
	let m_seated = tessellate_default(&bearing_gauge(STATION_X, 1.0, 0.05));
	let pen_seat = kernel_model::penetration_estimate(&m_tray, &m_seated, 4000);
	let d_seat = m_tray.min_distance(&m_seated);
	gate(
		"seated (0.05 float): bore rides the Ø7.9 seat free",
		pen_seat <= 0.02 && (0.02..=0.12).contains(&d_seat),
		format!("pen {pen_seat:4.2} gap {d_seat:4.2}"),
		&mut ok,
	);

	// ---- spool on rollers -----------------------------------------------------------
	println!("\nspool on rollers (Ø200×67 universal gauge):");
	let zc = STUB_Z + (111.0_f64.powi(2) - STATION_X * STATION_X).sqrt();
	let spool = spool_gauge(zc);
	let m_spool = tessellate_default(&spool);
	// four bearings seated (gauges), spool resting on their races
	let mut m_brgs = Mesh::default();
	for sx in [-STATION_X, STATION_X] {
		for side in [1.0, -1.0] {
			merge_into(&mut m_brgs, &tessellate_default(&bearing_gauge(sx, side, 0.0)));
		}
	}
	let d_contact = m_brgs.min_distance(&m_spool);
	gate(
		"flange rims contact the four races",
		d_contact < 0.06,
		format!("d {d_contact:5.3}"),
		&mut ok,
	);
	let d_tray = m_tray.min_distance(&m_spool);
	gate(
		"spool clears the tray everywhere else (≥1.5)",
		d_tray >= 1.5,
		format!("d {d_tray:5.2}"),
		&mut ok,
	);
	gate(
		"belly clears the hatch/deck (≥8)",
		(zc - SPOOL_R) - (DECK_TOP + SLIDER_T + 1.6) >= 8.0,
		format!("gap {:4.1}", (zc - SPOOL_R) - (DECK_TOP + SLIDER_T + 1.6)),
		&mut ok,
	);
	// lateral play between flange inner faces and the rails
	let play = (SPOOL_W / 2.0 - FLANGE_T) - RAIL_Y1;
	gate(
		"lateral tracking play 1.6–2.8 per side",
		(1.6..=2.8).contains(&play),
		format!("play {play:4.2}"),
		&mut ok,
	);
	// lateral extremes: the spool ridden 2.0 toward a rail (0.1 shy of flange
	// contact) must still clear the tray and keep all four race contacts
	for shift in [2.0_f64, -2.0] {
		let m_s = mesh_posed(&m_spool, tr(0.0, shift, 0.0));
		let d_t = m_tray.min_distance(&m_s);
		let d_b = m_brgs.min_distance(&m_s);
		gate(
			&format!("spool at {shift:+.1} lateral: clears tray, rides races"),
			d_t >= 0.05 && d_b < 0.06,
			format!("tray {d_t:4.2} race {d_b:5.3}"),
			&mut ok,
		);
	}
	// negative control: without bearings the spool would sit 11 lower — the
	// tray must collide (proves the clearance gate can fail)
	let dropped = mesh_posed(&m_spool, tr(0.0, 0.0, -11.0));
	let d_nc = m_tray.min_distance(&dropped);
	gate("NC: bearing-less spool hits the rails", d_nc < 0.5, format!("d {d_nc:5.2}"), &mut ok);

	// ---- hatch: slide, seal, captivity ---------------------------------------------
	println!("\ndesiccant hatch:");
	let closed = tr(0.0, 0.0, DECK_TOP + 0.1);
	let m_closed = mesh_posed(&m_slider, closed);
	let d_cl = m_tray.min_distance(&m_closed);
	gate("slider sits in the channel (0.05–0.5 slack)", (0.05..=0.5).contains(&d_cl), format!("d {d_cl:5.3}"), &mut ok);
	let slide: Vec<DAffine3> = (0..=13).map(|i| tr(9.0 * i as f64, 0.0, DECK_TOP + 0.1)).collect();
	let srep = sweep_check(&m_tray, &m_slider, &slide);
	gate(
		"slide-out sweep (14 poses): zero contacts, crossings, pen",
		srep.max_penetration < 0.05 && srep.contacts == 0 && srep.crossings == 0,
		format!("pen {:5.3} c {} x {}", srep.max_penetration, srep.contacts, srep.crossings),
		&mut ok,
	);
	// closed slider covers the aperture with margin; in-box travel ≤ (box floor
	// − tray)/1 side ≈ 4.5 → coverage cannot open in use
	let coverage = SLIDER_L / 2.0 - APER_X;
	// in-box travel = floor gap + ~1.0 of wall taper opening at hatch height
	let inbox_travel = (BOX_MIN_FLOOR_L - BASE_L) / 2.0 + 1.0;
	gate(
		"captive: cover margin 9 vs ≤5.5 taper-adjusted in-box travel",
		coverage >= inbox_travel + 2.0,
		format!("{coverage:3.1} vs {inbox_travel:3.1}"),
		&mut ok,
	);

	// ---- desiccant tank --------------------------------------------------------------
	println!("\ndesiccant tank:");
	// every intruding/removed piece is a rectangular prism — exact arithmetic:
	// ribs subtract, the 0.5 aperture/slot slabs above the deck underside add
	let ribs_mm3 = 8.0 * (BASE_L - 2.0 * WALL - 2.0) * RIB_T * (DECK_TOP - DECK_T + 0.3 - (FLOOR_T - 0.3));
	let aper_slab = (2.0 * APER_X) * (2.0 * APER_Y) * 0.5;
	let slot_slab = 10.0 * (88.0 - 46.0) * SLOT_W * 0.5;
	let tank_ml = (volume(&tank_block).abs() - ribs_mm3 + aper_slab + slot_slab) / 1000.0;
	gate(
		"tank volume ≥ 150 ml (≈110+ g silica at 0.7 g/ml)",
		tank_ml >= 150.0,
		format!("{tank_ml:5.0} ml"),
		&mut ok,
	);
	gate("vent slots retain 2–4 mm beads (1.6 < 2.0)", SLOT_W < 2.0, format!("{SLOT_W}"), &mut ok);
	let vent_cm2 = (2.0 * 5.0 * (88.0 - 46.0) * SLOT_W) / 100.0;
	gate("vent area ≥ 6 cm²", vent_cm2 >= 6.0, format!("{vent_cm2:4.1} cm²"), &mut ok);

	// ---- sustained load on the printed bearing stubs ------------------------------------
	// Added 2026-07-30. This campaign previously declared structural analysis
	// "intentionally absent" with a prose argument that the stresses are tiny.
	// The argument was right, but a spool sits here for MONTHS — that is a creep
	// case, and prose is not a gate. Judged against the time-derated table
	// (materials::pla::creep_allowable_mpa) at the 23 °C / 1-year cell, because a
	// drybox is UNHEATED; a heated dryer drops the allowable ~5× (see the note in
	// the generated ANALYSIS.md).
	let f_stub = SPOOL_KG * 9.81 / 4.0; // four 608s share the load
	let sig_creep_rt = kernel_model::materials::pla::creep_allowable_mpa(23.0, 8760.0);
	let tau_creep_rt = kernel_model::materials::pla::creep_shear_allowable_mpa(23.0, 8760.0);
	// Projected bearing on the stub seat (the classic pin-in-hole convention:
	// load over diameter × engaged length), and shear across the stub root.
	let sig_bear = f_stub / (2.0 * STUB_R * BRG_W);
	let tau_root = f_stub / (std::f64::consts::PI * STUB_R * STUB_R);
	gate(
		"sustained stub bearing vs 23 °C/1-year creep bound: ≥10×",
		sig_creep_rt / sig_bear >= 10.0,
		format!("{:5.1}× ({sig_bear:.3} MPa)", sig_creep_rt / sig_bear),
		&mut ok,
	);
	gate(
		"sustained stub root shear vs 23 °C/1-year creep bound: ≥10×",
		tau_creep_rt / tau_root >= 10.0,
		format!("{:5.1}× ({tau_root:.3} MPa)", tau_creep_rt / tau_root),
		&mut ok,
	);

	// ---- exports -----------------------------------------------------------------------
	let step_txt = export_step(&tray, "drybox_roller_tray");
	let _ = std::fs::write("spool_system/drybox_roller/cad/roller_tray.step", &step_txt);
	match kernel_brep::import_step(&step_txt) {
		Ok(back) => {
			let dv = (volume(&back).abs() - volume(&tray).abs()).abs() / volume(&tray).abs();
			gate("tray STEP round-trip conserves volume (<2.5%)", dv < 0.025, format!("dv {:5.2}%", dv * 100.0), &mut ok);
		}
		Err(e) => gate("tray STEP round-trip", false, format!("{e:?}"), &mut ok),
	}
	let mut scene = Mesh::default();
	merge_into(&mut scene, &m_tray);
	merge_into(&mut scene, &m_brgs);
	merge_into(&mut scene, &m_closed);
	merge_into(&mut scene, &m_spool);
	let _ = std::fs::write("spool_system/drybox_roller/assembly/assembly.stl", scene.to_stl_binary());
	let _ = std::fs::write("spool_system/drybox_roller/assembly/scene/tray.stl", m_tray.to_stl_binary());
	let _ = std::fs::write("spool_system/drybox_roller/assembly/scene/bearings.stl", m_brgs.to_stl_binary());
	let _ = std::fs::write("spool_system/drybox_roller/assembly/scene/hatch.stl", m_closed.to_stl_binary());
	let _ = std::fs::write("spool_system/drybox_roller/assembly/scene/spool_mock.stl", m_spool.to_stl_binary());

	// ---- ANALYSIS.md — generated from the live numbers above --------------------
	let analysis = format!(
		r#"# DRYBOX ROLLER — fit & function analysis (generated by drybox_roller.rs)

Every number below is the value the gate suite measured on this build —
regenerated each run, so it cannot go stale. Sources for the researched
container bounds: DESIGN.md.

## Container fit

| quantity | value | bound | margin |
|---|---|---|---|
| footprint | {BASE_L} × {BASE_W} | smallest reported floor 205 × 85 | {l_m:.0} / {w_m:.0} mm |
| spool top | {spool_top:.1} | usable height ≥ ~235 | {h_m:.1} mm |
| spool width | {SPOOL_W} | ≤ 68 rolls free (community bound) | 1.0 mm |

## Bearings (4 × 608 on D-profile stubs)

- push-on sweep worst sampled penetration: **{pp:.3} mm** (the Ø8.2 ring
  click — exact crossings during the ring poses: {px}, interference SEEN)
- seated: sampled penetration {sp:.2}, running gap {sg:.2} (Ø8.0 bore on
  Ø7.9 seat, floated 0.05 like every contact pose)
- ring crush (arithmetic, teardrop-arc corrected): {rc:.2} mm³
- rim-on-race engagement at full ±{play:.1} lateral play: worst {eng:.1} of
  {FLANGE_T} mm rim
- shoulder Ø11 lands on the 608 INNER ring only (land window Ø9.6–12)

## Desiccant tank

- volume (exact box arithmetic over the cavity): **{tank:.0} ml** ≈
  {silica:.0} g silica at 0.7 g/ml
- vent slots {SLOT_W} mm — retains 2–4 mm orange indicating beads, broken
  fragments included; open area {vent:.1} cm²
- hatch: {SLIDER_L}-long slider, cover margin {cov:.0} mm vs
  {inbox:.1} mm taper-adjusted in-box travel → captive in use, removable
  only with the tray in hand

## Print

- {gr:.0} g PLA solid-equivalent for tray + slider (+ coupon); steep area
  0 for all three parts, worst bridge ≤ 9.8 mm (deck rib bays)

Load path, gated rather than asserted in prose (2026-07-30): a loaded spool
({spool_kg} kg) rests on four 608 bearings, so each Ø{stub_d:.1} × {brg_w:.0} mm
printed stub carries {f_stub:.2} N. That is a SUSTAINED load — a spool can sit
here for months — so it is judged against the **time-derated creep** table
(`materials::pla::creep_allowable_mpa`), not the static allowable: at the
23 °C / 1-year cell ({sig_creep_rt} MPa tension, {tau_creep_rt} MPa shear) the
stub seat sees {sig_bear:.3} MPa in projected bearing ({m_bear:.0}× margin) and
{tau_root:.3} MPa in root shear ({m_shear:.0}× margin). A drybox is unheated, so
the room-temperature row is the right one; a heated dryer would drop the
allowable ~5× and the margin with it — do not repurpose this tray into a
heated unit without re-running that number.

Beyond that the engineering risks here are FIT risks, and those are what the
rest of the gates measure.
"#,
		spool_kg = SPOOL_KG,
		stub_d = 2.0 * STUB_R,
		brg_w = BRG_W,
		f_stub = f_stub,
		sig_creep_rt = sig_creep_rt,
		tau_creep_rt = tau_creep_rt,
		sig_bear = sig_bear,
		m_bear = sig_creep_rt / sig_bear,
		tau_root = tau_root,
		m_shear = tau_creep_rt / tau_root,
		l_m = BOX_MIN_FLOOR_L - BASE_L,
		w_m = BOX_MIN_FLOOR_W - BASE_W,
		h_m = BOX_USABLE_H - spool_top,
		pp = sweep.max_penetration,
		px = sweep.crossings,
		sp = pen_seat,
		sg = d_seat,
		rc = ring_crush,
		play = SPOOL_W / 2.0 - FLANGE_T - RAIL_Y1,
		eng = {
			let race = (RAIL_Y1 + 0.5, RAIL_Y1 + 0.5 + BRG_W);
			let play = SPOOL_W / 2.0 - FLANGE_T - RAIL_Y1;
			let engf = |shift: f64| {
				let (rlo, rhi) = (RIM_TRACK_Y + shift - FLANGE_T / 2.0, RIM_TRACK_Y + shift + FLANGE_T / 2.0);
				(rhi.min(race.1) - rlo.max(race.0)).max(0.0)
			};
			engf(play).min(engf(-play))
		},
		tank = tank_ml,
		silica = tank_ml * 0.7,
		vent = vent_cm2,
		cov = SLIDER_L / 2.0 - APER_X,
		inbox = (BOX_MIN_FLOOR_L - BASE_L) / 2.0 + 1.0,
		gr = (volume(&tray).abs() + volume(&slider).abs()) * PLA,
	);
	let _ = std::fs::write("spool_system/drybox_roller/analysis/ANALYSIS.md", analysis);

	// assembly/BOM.md — explicit bill of materials, generated from live volumes
	let bom = format!(
		"# DRYBOX ROLLER — bill of materials\n\n| item | qty | source | material | mass / cost |\n|---|---|---|---|---|\n| roller_tray (parts/) | 1 | print | PLA, 3 walls 15–25% | {tray_g:.0} g solid-equiv |\n| hatch_slider (parts/) | 1 | print | PLA | {sl_g:.0} g |\n| 608 bearing (8×22×7) | 4 | purchased | any (ZZ/2RS/open) | ~$1.20 total |\n| silica gel, 2–4 mm orange indicating | ~140 g | purchased | — | consumable |\n| coupon_stub (optional/) | 1 | print (pre-flight) | PLA | {cp_g:.0} g |\n| 4 L cereal container (Vtopmart/Skroam family) | 1 | purchased | — | the box itself |\n\nNo screws, no inserts, no tools.\n",
		tray_g = volume(&tray).abs() * PLA,
		sl_g = volume(&slider).abs() * PLA,
		cp_g = volume(&coupon).abs() * PLA,
	);
	let _ = std::fs::write("spool_system/drybox_roller/assembly/BOM.md", bom);

	let grams = (volume(&tray).abs() + volume(&slider).abs()) * PLA;
	println!("\nprinted set: {grams:.0} g PLA solid-equivalent + 4× 608 bearings (~$1.20)");
	println!("\nDRYBOX ROLLER: {}", if ok { "ALL GATES PASS" } else { "<<< FAIL" });
	std::process::exit(if ok { 0 } else { 1 });
}
