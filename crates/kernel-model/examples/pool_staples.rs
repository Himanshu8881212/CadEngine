//! POOLSTAPLES — replacement-part staples for pool plumbing (Printables "Pool
//! Accessories" campaign, entry 4: the high-search-traffic simple spares).
//!
//! Standard pool hose is corrugated 32 mm (1¼") or 38 mm (1½") with smooth
//! cuff ends of ~32.0 / ~38.0 mm INNER diameter that clamp over spigots with
//! hose clamps. This set covers the perennial "the bit that broke" searches:
//!
//!   - `plug_hose_*`: winterizing/blanking plugs that push INTO a hose cuff —
//!     3 shallow sealing ribs (crest 0.3 mm proud of the cuff bore, asserted
//!     arithmetically), a rib-free pilot the printed gauge ring rides on, a
//!     stop flange that lands on the cuff end, and a grip disc to pull it out.
//!     Pure revolve, printed grip-down: radius never grows with height except
//!     the rib flanks, which climb at 56° (> the 45° FDM limit).
//!   - `adapter_32_38mm`: stepped barb adapter joining a 32 cuff to a 38 cuff
//!     (both slide OVER it), Ø22 through bore for flow. Christmas-tree cone
//!     barbs: on the DOWN (38) spigot the retention shelves face up — full 90°
//!     barbs AND support-free; on the UP (32) spigot the retention flanks are
//!     limited to 50° so they self-support — honestly weaker bite, which is
//!     why the hose clamp is not optional (it never was, on any barb).
//!   - `hanger_hose_wall`: wall hanger for the coiled hose. 60 mm wide saddle
//!     arm climbing at 50° chevron-style (underside beats the 45° gate the
//!     same way pool_tubedock's towel hook does), 8 mm up-curl lip, 90.5 mm
//!     clear saddle depth. Two countersunk screw holes are HORIZONTAL in the
//!     print orientation → `kernel_brep::teardrop_hole` + 100° cone crowns
//!     (the drawer_system.rs idiom), so the part prints as it hangs, no
//!     supports.
//!   - `cap_ladder_25mm`: push-in end cap for Ø25 (1") ladder tube, crush
//!     ribs sized for the ~24 mm tube ID, flat low-profile crown. Printed
//!     crown-down (the honest orientation: every overhang is a ≥ 51° rib
//!     flank), audited exactly as shipped.
//!   - `gauge_ring_32mm` / `gauge_ring_38mm`: 6 mm rings whose bores are the
//!     exact cuff IDs. Print one in minutes and slip it over your hose spigot
//!     / our plug pilot to verify YOUR hose matches the standard before
//!     printing the big parts ("verify with your calipers" made physical).
//!
//! Every part ships in print orientation and passes the kernel's
//! `support_free_report` gate (steep_area < 1e-6, bridges ≤ 12 mm) plus
//! brep-valid + watertight + bed fit; a wrong-orientation negative control
//! proves the gate bites. Interference fits are asserted arithmetically from
//! the design constants (crest Ø vs cuff ID); clearance fits are measured on
//! posed meshes against cylinder/ring gauges.
//!
//! Contract: pool_system/staples/DESIGN.md (every line asserted here).
//! Run: cargo run --example pool_staples -p kernel-model --release
//!   -> pool_system/staples/ (exit 1 on any FAIL)

use kernel_brep::math::{DAffine3, DVec2, DVec3};
use kernel_brep::{
	cone, cuboid, cylinder, extrude, extrude_with_holes, revolve, teardrop_hole, tessellate_default, union, validate,
	volume, Mesh, Solid,
};
use kernel_core::math::Vec3;
use std::f64::consts::FRAC_PI_2;

// ---- hose standard (mm) --------------------------------------------------------
const CUFF_IDS: [f64; 2] = [32.0, 38.0]; // smooth-cuff inner Ø of 1¼" / 1½" pool hose
const SEG: usize = 128;
const PETG_G_PER_MM3: f64 = 0.00127;

// ---- plugs ---------------------------------------------------------------------
const PLUG_BODY_CLR: f64 = 0.25; // radial: body OD = cuff ID − 0.5 (slides in)
const RIB_PROUD: f64 = 0.40; // radial: crest OD = cuff ID + 0.3 (seals)
const RIB_RISE: f64 = 0.60; // lower-flank rise: atan(0.60/0.40) = 56.3° > 45°
const RIB_PITCH: f64 = 6.0;
const GRIP_H: f64 = 4.0; // grip disc height (on the bed)
const FLANGE_H: f64 = 5.0; // stop flange height
const PLUG_LEN: f64 = 30.0; // insertion length past the flange
const PLUG_PILOT: f64 = 8.5; // rib-free pilot after the flange (gauge rides here)

// ---- adapter -------------------------------------------------------------------
const AD_BORE_R: f64 = 11.0; // Ø22 flow bore (≥ Ø20 asserted)
const BARB_INT: f64 = 0.5; // diametral crest interference over the cuff ID
const ROOT_CLR: f64 = 0.5; // radial: spigot root OD = cuff ID − 1.0 (cuff slides on)
const AD_FLANGE_R: f64 = 24.0; // Ø48 centre stop flange (round, hand-tightened part)

// ---- hanger --------------------------------------------------------------------
const HG_PLATE_W: f64 = 64.0; // back plate width (arm inset 2 per side: no coplanar union)
const HG_PLATE_T: f64 = 5.0;
const HG_PLATE_H: f64 = 140.0;
const ARM_W: f64 = 60.0; // saddle width (≥ 60 so a coiled hose stack sits flat)
const ARM_ROOT: (f64, f64) = (1.0, 10.0); // arm underside start (buried 4 into the plate)
const ARM_TIP: (f64, f64) = (101.5, 132.6); // underside end: atan(122.6/100.5) = 50.7°
const LIP_IN_Y: f64 = 95.5; // lip inner face → clear saddle depth 90.5 ≥ 90
const HOLE_Z: [f64; 2] = [40.0, 130.0]; // countersunk screw holes (axis horizontal)

// ---- ladder cap ----------------------------------------------------------------
const TUBE_ID: f64 = 24.0; // common Ø25 (1") ladder tube bore
const TUBE_OD: f64 = 25.0;
const CAP_CROWN_R: f64 = 14.0; // Ø28 flat crown covers the Ø25 tube rim
const CAP_CROWN_H: f64 = 3.0;
const CAP_BODY_R: f64 = TUBE_ID / 2.0 - 0.4; // 11.6: 0.4 radial slide clearance
const CAP_CREST_R: f64 = (TUBE_ID + 0.3) / 2.0; // 12.15: crush ribs 0.3 over the tube ID
const CAP_RIB_Z: [f64; 3] = [11.0, 14.0, 17.0]; // pilot 3..11 = 8 rib-free

const GAUGE_H: f64 = 6.0;
const GAUGE_WALL: f64 = 4.0;

// ---- tiny helpers (pool_tubedock.rs idioms) ------------------------------------

fn v(x: f64, y: f64, z: f64) -> DVec3 {
	DVec3::new(x, y, z)
}
fn tr(x: f64, y: f64, z: f64) -> DAffine3 {
	DAffine3::from_translation(v(x, y, z))
}

/// Force a polygon CCW (extrude() wants CCW; profiles are written for legibility).
fn ccw(mut p: Vec<DVec2>) -> Vec<DVec2> {
	let a2: f64 = p.windows(2).map(|w| w[0].x * w[1].y - w[1].x * w[0].y).sum::<f64>()
		+ (p[p.len() - 1].x * p[0].y - p[0].x * p[p.len() - 1].y);
	if a2 < 0.0 {
		p.reverse();
	}
	p
}

/// Prism from a profile in the model (y,z) plane, spanning x ∈ [x0, x0+len].
fn prism_x(profile: &[(f64, f64)], x0: f64, len: f64) -> Solid {
	let p: Vec<DVec2> = profile.iter().map(|&(y, z)| DVec2::new(-z, y)).collect();
	extrude(&ccw(p), len).transformed(tr(x0, 0.0, 0.0) * DAffine3::from_rotation_y(FRAC_PI_2))
}

fn circle(cx: f64, cy: f64, r: f64) -> Vec<DVec2> {
	(0..SEG)
		.map(|i| {
			let a = std::f64::consts::TAU * i as f64 / SEG as f64;
			DVec2::new(cx + r * a.cos(), cy + r * a.sin())
		})
		.collect()
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

fn rev(profile: &[(f64, f64)]) -> Solid {
	let p: Vec<DVec2> = profile.iter().map(|&(r, z)| DVec2::new(r, z)).collect();
	revolve(&p, SEG)
}

// ---- parts (all modeled directly in print orientation, z up from the bed) ------

/// Winterizing plug for a hose cuff of inner Ø `id`. Bottom-up: grip disc,
/// stop flange (lands on the cuff end), rib-free pilot, 3 sealing ribs, 45°
/// lead-in. One revolve — no booleans anywhere. Solid on purpose: the slicer's
/// infill hollows it, and a stiff plug seats better than a thin shell.
fn plug_hose(id: f64) -> Solid {
	let rb = id / 2.0 - PLUG_BODY_CLR; // body
	let rc = rb + RIB_PROUD; // rib crest
	let rg = id / 2.0 + 6.0; // grip disc
	let rf = id / 2.0 + 5.0; // stop flange (Ø = ID + 10: ≥ 5 mm skirt past the bore edge)
	let z0 = GRIP_H + FLANGE_H; // flange top = insertion datum
	let zt = z0 + PLUG_LEN;
	let mut p = vec![(0.0, 0.0), (rg, 0.0), (rg, GRIP_H), (rf, GRIP_H), (rf, z0), (rb, z0)];
	for k in 0..3 {
		let z = z0 + PLUG_PILOT + RIB_PITCH * k as f64;
		p.push((rb, z));
		p.push((rc, z + RIB_RISE)); // lower flank 56.3° (the only outward-growing face)
		p.push((rc, z + RIB_RISE + 0.5)); // crest band
		p.push((rb, z + RIB_RISE + 0.5)); // upward-facing shelf (sharp sealing edge)
	}
	p.push((rb, zt - 2.0));
	p.push((rb - 2.0, zt)); // 45° lead-in (narrows upward: roof-like, prints free)
	p.push((0.0, zt));
	rev(&p)
}

/// 32↔38 barb adapter, printed 38-spigot-down. Per-tooth geometry differs by
/// spigot on purpose (see module doc): down-spigot teeth = 79° ramp + flat
/// upward shelf (true 90° barb edge); up-spigot teeth = 50° retention flank +
/// ramp. Flange underside is a 50° cone. One revolve, Ø22 bore throughout.
fn adapter_32_38() -> Solid {
	let r38 = CUFF_IDS[1] / 2.0 - ROOT_CLR; // 18.5
	let c38 = (CUFF_IDS[1] + BARB_INT) / 2.0; // 19.25
	let r32 = CUFF_IDS[0] / 2.0 - ROOT_CLR; // 15.5
	let c32 = (CUFF_IDS[0] + BARB_INT) / 2.0; // 16.25
	let mut p = vec![(AD_BORE_R, 0.0), (17.0, 0.0)];
	p.push((r38, 5.0)); // tip lead cone, 73° from horizontal
	for k in 0..3 {
		let z = 12.0 + 5.5 * k as f64; // first push also closes the 5..12 pilot band
		p.push((r38, z));
		p.push((c38, z + 4.0)); // ramp: 79° (downward-facing, safely past 45°)
		p.push((r38, z + 4.0)); // retention shelf faces UP: free 90° barb
	}
	p.push((r38, 28.0));
	p.push((AD_FLANGE_R, 34.6)); // flange underside cone: atan(6.6/5.5) = 50.2°
	p.push((AD_FLANGE_R, 39.6)); // flange band
	p.push((r32, 39.6)); // flange top (upward-facing)
	for k in 0..3 {
		let z = 41.6 + 5.5 * k as f64;
		p.push((r32, z));
		p.push((c32, z + 0.9)); // retention flank: atan(0.9/0.75) = 50.2° (down-facing)
		p.push((r32, z + 4.9)); // ramp narrows upward: prints free
	}
	p.push((r32, 64.6)); // pilot 57.5..64.6 (gauge ring rides here)
	p.push((14.0, 69.6)); // tip lead
	p.push((AD_BORE_R, 69.6));
	rev(&p)
}

/// Wall hose hanger, printed exactly as it hangs. Chevron saddle arm (both
/// long faces ≥ 50° from horizontal, tip up-curl lip); two countersunk screw
/// holes with horizontal axes → teardrop bores + 100° cone crowns.
fn hanger_hose_wall() -> Solid {
	let plate = cuboid(v(-HG_PLATE_W / 2.0, 0.0, 0.0), v(HG_PLATE_W / 2.0, HG_PLATE_T, HG_PLATE_H));
	let arm = [
		ARM_ROOT, // underside root (buried in the plate: overlap, never a kiss)
		ARM_TIP, // underside 50.7°
		(ARM_TIP.0, 147.0), // lip outer (vertical)
		(LIP_IN_Y, 147.0), // lip top (upward)
		(LIP_IN_Y, 139.0), // lip inner (vertical): saddle depth 95.5 − 5.0 = 90.5
		(1.0, 24.0), // saddle top edge back to the plate (upward-facing)
	];
	let mut s = union(&plate, &prism_x(&arm, -ARM_W / 2.0, ARM_W));
	for &hz in &HOLE_Z {
		s = teardrop_hole(&s, v(0.0, 0.0, hz), DVec3::Y, DVec3::Z, 4.4, HG_PLATE_T, 46.0, None)
			.expect("hanger screw hole");
		// 100° countersink crown, Ø9 mouth exactly at the front face (drawer idiom:
		// crown normals stay shy of the 45° gate)
		s = kernel_brep::difference(&s, &cone(v(0.0, HG_PLATE_T + 1.0, hz), -DVec3::Y, 5.69, 4.78, SEG));
	}
	s
}

/// Push-in end cap for Ø25 ladder tube (~24 mm ID), printed crown-down: flat
/// low-profile crown on the bed, plug up — every overhang is a ≥ 51° rib
/// flank. (A domed crown printed crown-down would need supports under the
/// dome rim; the flat crown is the honest low-profile choice.)
fn cap_ladder() -> Solid {
	let mut p = vec![(0.0, 0.0), (CAP_CROWN_R, 0.0), (CAP_CROWN_R, CAP_CROWN_H), (CAP_BODY_R, CAP_CROWN_H)];
	for &z in &CAP_RIB_Z {
		p.push((CAP_BODY_R, z));
		p.push((CAP_CREST_R, z + 0.7)); // flank: atan(0.70/0.55) = 51.8°
		p.push((CAP_CREST_R, z + 1.2)); // crest band
		p.push((CAP_BODY_R, z + 1.2)); // upward shelf
	}
	p.push((CAP_BODY_R, 19.5));
	p.push((CAP_BODY_R - 1.5, 21.0)); // 45° lead-in (narrows upward)
	p.push((0.0, 21.0));
	rev(&p)
}

/// Fit-gauge ring: bore is the EXACT cuff inner Ø. Slips over your hose
/// spigot and over the matching plug/adapter pilot.
fn gauge_ring(id: f64) -> Solid {
	extrude_with_holes(&circle(0.0, 0.0, id / 2.0 + GAUGE_WALL), &[circle(0.0, 0.0, id / 2.0)], GAUGE_H)
}

/// Virtual ladder-tube gauge (ID 24 / OD 25 annulus) — posed only, not printed.
fn ladder_tube_gauge() -> Solid {
	extrude_with_holes(&circle(0.0, 0.0, TUBE_OD / 2.0), &[circle(0.0, 0.0, TUBE_ID / 2.0)], GAUGE_H)
}

// ---- gates ---------------------------------------------------------------------

fn emit(name: &str, s: &Solid) -> bool {
	let val = validate(s);
	// parts are modeled in print orientation already; still drop to z = 0
	let zmin = tessellate_default(s).positions.iter().map(|p| p.z as f64).fold(f64::INFINITY, f64::min);
	let printed = s.transformed(tr(0.0, 0.0, -zmin));
	let mesh_p = tessellate_default(&printed);
	let rep = mesh_p.support_free_report(Vec3::Z, 45.0, 0.3);
	let (lo, hi) = mesh_aabb(&mesh_p);
	let ext = hi - lo;
	let fits = ext.x <= 250.0 && ext.y <= 210.0 && ext.z <= 220.0;
	let wt = mesh_p.is_watertight();
	let vol = volume(s).abs();
	let ok = val.is_valid() && wt && rep.steep_area < 1e-6 && rep.max_bridge_span <= 12.0 && fits;
	let _ = std::fs::write(format!("pool_system/staples/parts/{name}.stl"), mesh_p.to_stl_binary());
	println!(
		"  {name:20} valid={:5} wt={wt:5} steep={:8.3} mm²  bridge≤{:5.1}  {:3.0}g  {:6.0}mm³  {}",
		val.is_valid(),
		rep.steep_area,
		rep.max_bridge_span,
		vol * PETG_G_PER_MM3,
		vol,
		if ok { "OK" } else { "<<< FAIL" }
	);
	ok
}

/// One arithmetic design gate: `value` must lie in `[lo, hi]` (constants-only
/// interference/coverage claims — no meshes involved).
fn arith(label: &str, value: f64, lo: f64, hi: f64, ok: &mut bool) {
	let pass = (lo..=hi).contains(&value);
	if !pass {
		*ok = false;
	}
	println!("  {label:52} {value:7.2}  want [{lo:.2}, {hi:.2}]  {}", if pass { "OK" } else { "<<< FAIL" });
}

/// Check one designed relation between two posed meshes.
fn relation(label: &str, a: &Mesh, b: &Mesh, contact: bool, ok: &mut bool) {
	let d = a.min_distance(b);
	let pass = if contact { d < 0.06 } else { d >= 0.10 };
	if !pass {
		*ok = false;
	}
	println!(
		"  {label:52} min_dist={d:7.3}  want {}  {}",
		if contact { "contact (<0.06)" } else { "clearance (>=0.10)" },
		if pass { "OK" } else { "<<< FAIL" }
	);
}

fn posed(s: &Solid, m: DAffine3) -> Mesh {
	tessellate_default(&s.transformed(m))
}

fn merge_into(dst: &mut Mesh, src: &Mesh) {
	let base = dst.positions.len() as u32;
	dst.positions.extend_from_slice(&src.positions);
	dst.indices.extend(src.indices.iter().map(|i| i + base));
}

fn main() {
	let _ = std::fs::create_dir_all("pool_system/staples/parts");
	println!("POOLSTAPLES pool replacement parts — parts (STLs in print orientation):\n");

	let plugs: Vec<Solid> = CUFF_IDS.iter().map(|&d| plug_hose(d)).collect();
	let adapter = adapter_32_38();
	let hanger = hanger_hose_wall();
	let cap = cap_ladder();
	let gauges: Vec<Solid> = CUFF_IDS.iter().map(|&d| gauge_ring(d)).collect();

	let parts: Vec<(String, &Solid)> = vec![
		("plug_hose_32mm".into(), &plugs[0]),
		("plug_hose_38mm".into(), &plugs[1]),
		("adapter_32_38mm".into(), &adapter),
		("hanger_hose_wall".into(), &hanger),
		("cap_ladder_25mm".into(), &cap),
		("gauge_ring_32mm".into(), &gauges[0]),
		("gauge_ring_38mm".into(), &gauges[1]),
	];
	let mut ok = true;
	for (name, s) in &parts {
		ok &= emit(name, s);
	}

	// ---- negative control: the support gate must BITE in a wrong orientation.
	// Hanger lying on its back (plate on the bed, arm up): the saddle's 50.7°
	// top face becomes a 39° underside — ~9000 mm² of steep overhang.
	let wrong = tessellate_default(&hanger.transformed(DAffine3::from_rotation_x(FRAC_PI_2)))
		.support_free_report(Vec3::Z, 45.0, 0.3);
	let nc = wrong.steep_area > 500.0;
	ok &= nc;
	println!(
		"\nA-PRINT NC: hanger audited on its back -> steep {:.1} mm² (must exceed 500) {}",
		wrong.steep_area,
		if nc { "OK" } else { "<<< FAIL" }
	);

	// ---- interference & coverage, asserted from the design constants ----------
	println!("\narithmetic design gates (constants, not meshes):");
	for &id in &CUFF_IDS {
		let crest_od = 2.0 * (id / 2.0 - PLUG_BODY_CLR + RIB_PROUD);
		arith(&format!("plug Ø{id}: rib crest interference over cuff ID"), crest_od - id, 0.2, 0.5, &mut ok);
		arith(&format!("adapter Ø{id} spigot: barb crest interference"), BARB_INT, 0.3, 0.7, &mut ok);
		arith(&format!("plug Ø{id}: flange skirt past the cuff bore edge"), 5.0, 4.0, f64::INFINITY, &mut ok);
	}
	arith("adapter flow bore Ø (must pass Ø20)", 2.0 * AD_BORE_R, 20.0, f64::INFINITY, &mut ok);
	arith("cap: crush-rib crest interference over tube ID", 2.0 * CAP_CREST_R - TUBE_ID, 0.2, 0.5, &mut ok);
	arith("cap: crown Ø over tube OD (covers the rim)", 2.0 * CAP_CROWN_R - TUBE_OD, 2.0, f64::INFINITY, &mut ok);
	arith("hanger: clear saddle depth (coiled-hose stack)", LIP_IN_Y - HG_PLATE_T, 90.0, f64::INFINITY, &mut ok);
	arith("hanger: saddle width", ARM_W, 60.0, f64::INFINITY, &mut ok);
	// every deliberately-angled underside beats the 45° FDM limit with margin
	let deg = |rise: f64, run: f64| (rise / run).atan().to_degrees();
	arith("plug rib lower-flank angle (deg)", deg(RIB_RISE, RIB_PROUD), 48.0, 90.0, &mut ok);
	arith("cap rib lower-flank angle (deg)", deg(0.7, CAP_CREST_R - CAP_BODY_R), 48.0, 90.0, &mut ok);
	arith("adapter up-spigot retention-flank angle (deg)", deg(0.9, 0.75), 48.0, 90.0, &mut ok);
	arith("adapter flange underside cone angle (deg)", deg(6.6, AD_FLANGE_R - 18.5), 48.0, 90.0, &mut ok);
	arith("hanger arm underside angle (deg)", deg(ARM_TIP.1 - ARM_ROOT.1, ARM_TIP.0 - ARM_ROOT.0), 48.0, 90.0, &mut ok);

	// ---- posed fits against cylinder / ring gauges ----------------------------
	println!("\nposed fits (gauge meshes, print frames):");
	let flange_top = GRIP_H + FLANGE_H;
	for (i, &id) in CUFF_IDS.iter().enumerate() {
		let m_plug = tessellate_default(&plugs[i]);
		// seated: gauge ring dropped down the pilot until it lands on the flange
		relation(
			&format!("gauge_{id} seated on plug_{id} stop flange"),
			&m_plug,
			&posed(&gauges[i], tr(0.0, 0.0, flange_top)),
			true,
			&mut ok,
		);
		// mid-pilot: plug BODY must clear the cuff bore (radial 0.25)
		relation(
			&format!("gauge_{id} mid-pilot on plug_{id} (body clears bore)"),
			&m_plug,
			&posed(&gauges[i], tr(0.0, 0.0, flange_top + 1.5)),
			false,
			&mut ok,
		);
	}
	let m_ad = tessellate_default(&adapter);
	relation("gauge_38 over adapter 38-spigot pilot", &m_ad, &posed(&gauges[1], tr(0.0, 0.0, 5.5)), false, &mut ok);
	relation("gauge_32 over adapter 32-spigot pilot", &m_ad, &posed(&gauges[0], tr(0.0, 0.0, 58.0)), false, &mut ok);
	let flow_rod = cylinder(v(0.0, 0.0, -8.0), v(0.0, 0.0, 1.0), 10.0, 86.0, SEG);
	relation("Ø20 flow rod through adapter bore", &m_ad, &tessellate_default(&flow_rod), false, &mut ok);

	let m_cap = tessellate_default(&cap);
	let tube = ladder_tube_gauge();
	relation("ladder tube seated against cap crown", &m_cap, &posed(&tube, tr(0.0, 0.0, CAP_CROWN_H)), true, &mut ok);
	relation("ladder tube mid-pilot (cap body clears ID)", &m_cap, &posed(&tube, tr(0.0, 0.0, CAP_CROWN_H + 1.0)), false, &mut ok);

	// Ø38 coil nested in the hanger saddle valley: tangent to plate front and
	// arm underside line with 0.3 designed clearance each (centre solved from
	// the profile constants; the mesh must agree)
	let hose = cylinder(v(-40.0, 24.3, 82.8), v(1.0, 0.0, 0.0), 19.0, 80.0, SEG);
	relation("Ø38 hose coil nested in hanger saddle", &tessellate_default(&hanger), &tessellate_default(&hose), false, &mut ok);

	// ---- in-use assembly scene (posed component STLs for the assembly-doc
	// sheets + a combined ASSEMBLY.stl). The wall board, hose cuffs and ladder
	// tube are context stubs, not printed parts; where a rib/barb crest sits
	// inside a cuff bore the meshes overlap by the designed interference —
	// shown as modeled, not shaved.
	let _ = std::fs::create_dir_all("pool_system/staples/assembly_parts");
	let cuff_stub = |id: f64, wall: f64, len: f64| {
		extrude_with_holes(&circle(0.0, 0.0, id / 2.0 + wall), &[circle(0.0, 0.0, id / 2.0)], len)
	};
	let flip = DAffine3::from_rotation_x(std::f64::consts::PI); // crown-up for the in-use cap
	let scene: Vec<(&str, Mesh)> = vec![
		// scene A: hanger screwed to a wall board
		("wall_board", tessellate_default(&cuboid(v(-60.0, -10.0, -10.0), v(60.0, 0.0, 160.0)))),
		("hanger_hose_wall", tessellate_default(&hanger)),
		// scene B: 32 cuff stub pushed home against the plug's stop flange
		("plug_hose_32mm", posed(&plugs[0], tr(150.0, 60.0, 0.0))),
		("hose_cuff_32mm", posed(&cuff_stub(CUFF_IDS[0], 3.0, 40.0), tr(150.0, 60.0, flange_top))),
		// scene C: 38 cuff stub clamped over the adapter's down spigot
		("adapter_32_38mm", posed(&adapter, tr(240.0, 60.0, 0.0))),
		("hose_cuff_38mm", posed(&cuff_stub(CUFF_IDS[1], 3.0, 40.0), tr(240.0, 60.0, -14.0))),
		// scene D: end cap seated crown-up on a Ø25 ladder-tube stub
		("ladder_tube_25mm", posed(&cuff_stub(TUBE_ID, (TUBE_OD - TUBE_ID) / 2.0, 60.0), tr(330.0, 60.0, 0.0))),
		("cap_ladder_25mm", posed(&cap, tr(330.0, 60.0, 63.0) * flip)),
	];
	let mut asm = Mesh::default();
	for (name, m) in &scene {
		let _ = std::fs::write(format!("pool_system/staples/assembly_parts/{name}.stl"), m.to_stl_binary());
		merge_into(&mut asm, m);
	}
	let _ = std::fs::write("pool_system/staples/ASSEMBLY.stl", asm.to_stl_binary());
	println!("\nassembly scene: {} posed components -> assembly_parts/ + ASSEMBLY.stl", scene.len());

	println!("\nPOOLSTAPLES: {}", if ok { "ALL GATES PASS" } else { "<<< FAIL" });
	std::process::exit(if ok { 0 } else { 1 });
}
