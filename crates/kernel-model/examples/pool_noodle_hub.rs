//! NOODLEDOCK — pool-noodle hubs with machine-verified buoyancy (Printables
//! "Pool Accessories" flash contest, July 2026 — entry 2 of the POOLDOCK campaign).
//!
//! Standard Ø65 mm pool noodles are the cheapest guaranteed flotation there is;
//! these parts turn them into structure. A **noodle dock** is the POOLDOCK
//! C-clamp re-sized to grip foam (Ø63 bore = 2 mm squeeze), carrying the
//! IDENTICAL vertical dovetail track as `pool_tubedock.rs` — so the towel hook,
//! cup holder and rings from entry 1 drop straight onto a floating noodle (that
//! compatibility is machine-checked here by seating entry-1's rail geometry in
//! this dock's track). A **coupler** joins two noodles end-to-end through a
//! chamfered internal stop; a **raft clip** bridges two parallel noodles at
//! 80 mm spacing into a raft you can dock drinks onto.
//!
//! The flex: buoyancy is ASSERTED, not hoped. Printed-part masses come from the
//! engine's exact volumes (PETG 1.27 g/cm³); noodle displacement and the drink
//! payload are documented constants. Gates: a 2-noodle × 1.2 m raft carrying a
//! docked full 500 ml drink floats with ≥ 1.5× reserve at half submersion, and
//! the hang load stays under half of one noodle's full displacement (a static
//! no-capsize bound — heel presses the loaded noodle deeper long before the
//! raft can roll). Assumptions are constants with comments, so a skeptic can
//! re-derive every number.
//!
//! Every part prints support-free in its shipped orientation (steep_area == 0
//! asserted, negative control included), PETG/ASA for sun + chlorine.
//!
//! Contract: pool_system/noodlehub/DESIGN.md (every line asserted here).
//! Run: cargo run --example pool_noodle_hub -p kernel-model --release
//!   -> pool_system/noodlehub/ (exit 1 on any FAIL)

use kernel_brep::math::{DAffine3, DVec2, DVec3};
use kernel_brep::{cuboid, cylinder, difference, extrude, revolve, tessellate_default, union, validate, volume, Mesh, Solid};
use kernel_core::math::Vec3;
use std::f64::consts::FRAC_PI_2;

// ---- the noodle ----------------------------------------------------------------
const NOODLE_D: f64 = 65.0; // standard pool-noodle diameter
const DOCK_SQUEEZE: f64 = 2.0; // diametral foam squeeze in the dock bore
const CPLR_SQUEEZE: f64 = 1.5; // diametral squeeze in the coupler sockets
const LIP_DEG: f64 = 105.0; // 210° wrap, opening down (same as pool_tubedock)
const LIP_OUT_DEG: f64 = 113.0; // outer lip corner: 8° wedge ramp for snap-on
const WALL: f64 = 3.2;
const CLAMP_LEN: f64 = 40.0;

// ---- the POOLDOCK track — MUST stay byte-identical to pool_tubedock.rs so
// entry-1 attachments fit (the seat/slide gates below prove it) --------------------
const TRK_OPEN: f64 = 3.0;
const TRK_ROOT: f64 = 6.0;
const TRK_DEPTH: f64 = 2.5;
const TRK_TOP: f64 = 12.0;
const TRK_BOT: f64 = -46.0;
const PLATE_T: f64 = 4.0;
const PLATE_BOT: f64 = -50.0;
const PLATE_TOP: f64 = 8.0;
const RAIL_WAIST: f64 = 2.8;
const RAIL_ROOT: f64 = 5.55;
const RAIL_ENG: f64 = 2.35;
const RAIL_LEN: f64 = 40.0;
const FACE_GAP: f64 = 0.2;
const APLATE_T: f64 = 3.5;

// ---- raft geometry -------------------------------------------------------------
const RAFT_PITCH: f64 = 80.0; // noodle centre spacing in the raft clip

// ---- buoyancy scenario constants (documented, not measured by the engine) ------
const PETG_G_PER_MM3: f64 = 0.00127; // printed-part density
const WATER_G_PER_MM3: f64 = 0.001; // fresh water
const FOAM_G_PER_MM3: f64 = 0.00003; // EPE noodle foam ~30 kg/m³
const RAFT_NOODLE_LEN: f64 = 1200.0; // two full-length noodles
const DRINK_G: f64 = 550.0; // full 500 ml can/bottle incl. container

const SEG: usize = 128;

// ---- tiny helpers (shared campaign idiom — see pool_tubedock.rs) ---------------

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

fn prism_x(profile: &[(f64, f64)], x0: f64, len: f64) -> Solid {
	let p: Vec<DVec2> = profile.iter().map(|&(y, z)| DVec2::new(-z, y)).collect();
	extrude(&ccw(p), len).transformed(tr(x0, 0.0, 0.0) * DAffine3::from_rotation_y(FRAC_PI_2))
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

fn push_arc(p: &mut Vec<(f64, f64)>, r: f64, a0_deg: f64, a1_deg: f64, n: usize) {
	for i in 0..=n {
		let a = (a0_deg + (a1_deg - a0_deg) * i as f64 / n as f64).to_radians();
		p.push((r * a.sin(), r * a.cos()));
	}
}

fn dedup(mut p: Vec<(f64, f64)>) -> Vec<(f64, f64)> {
	p.dedup_by(|a, b| (a.0 - b.0).abs() < 1e-9 && (a.1 - b.1).abs() < 1e-9);
	if p.len() > 1 {
		let (f, l) = (p[0], p[p.len() - 1]);
		if (f.0 - l.0).abs() < 1e-9 && (f.1 - l.1).abs() < 1e-9 {
			p.pop();
		}
	}
	p
}

// ---- geometry ------------------------------------------------------------------

fn bore_r() -> f64 {
	(NOODLE_D - DOCK_SQUEEZE) / 2.0
}
fn outer_r() -> f64 {
	bore_r() + WALL
}
fn face_y() -> f64 {
	outer_r() + 3.4
}

/// The Ø63-bore C-ring in the (y,z) plane, noodle axis along X, opening DOWN —
/// same construction as pool_tubedock::dock_profile, sized for foam.
fn noodle_ring_profile(with_plate: bool) -> Vec<(f64, f64)> {
	let (ri, ro) = (bore_r(), outer_r());
	let yf = face_y();
	let pi_face = yf - PLATE_T;
	let mut p: Vec<(f64, f64)> = Vec::new();
	if with_plate {
		let a_top = (ro * ro - PLATE_TOP * PLATE_TOP).sqrt().atan2(PLATE_TOP).to_degrees();
		let zx = -(ro * ro - pi_face * pi_face).sqrt();
		let a_x = pi_face.atan2(zx).to_degrees();
		push_arc(&mut p, ro, -LIP_OUT_DEG, a_top, 128);
		p.push((yf, PLATE_TOP));
		p.push((yf, PLATE_BOT));
		p.push((pi_face, PLATE_BOT));
		p.push((pi_face, zx));
		push_arc(&mut p, ro, a_x, LIP_OUT_DEG, 16);
	} else {
		push_arc(&mut p, ro, -LIP_OUT_DEG, LIP_OUT_DEG, 192);
	}
	p.push((ri * LIP_DEG.to_radians().sin(), ri * LIP_DEG.to_radians().cos()));
	push_arc(&mut p, ri, LIP_DEG, -LIP_DEG, 192);
	dedup(p)
}

fn track_cutter(yf: f64) -> Solid {
	let pts = vec![
		DVec2::new(-TRK_OPEN, yf + 1.0),
		DVec2::new(-TRK_OPEN, yf),
		DVec2::new(-TRK_ROOT, yf - TRK_DEPTH),
		DVec2::new(TRK_ROOT, yf - TRK_DEPTH),
		DVec2::new(TRK_OPEN, yf),
		DVec2::new(TRK_OPEN, yf + 1.0),
	];
	extrude(&ccw(pts), TRK_TOP - TRK_BOT).transformed(tr(0.0, 0.0, TRK_BOT))
}

fn noodle_dock() -> Solid {
	let body = prism_x(&noodle_ring_profile(true), -CLAMP_LEN / 2.0, CLAMP_LEN);
	difference(&body, &track_cutter(face_y()))
}

/// Entry-1 rail on its back plate, LOCAL frame (front face y = 0, bottom z = 0):
/// used here only as the compat gauge proving entry-1 attachments seat.
fn rail_gauge() -> Solid {
	let plate = cuboid(v(-12.0, 0.0, 0.0), v(12.0, APLATE_T, 50.0));
	let rail_pts = vec![
		DVec2::new(-RAIL_WAIST, 0.8),
		DVec2::new(-RAIL_WAIST, -FACE_GAP),
		DVec2::new(-RAIL_ROOT, -FACE_GAP - RAIL_ENG),
		DVec2::new(RAIL_ROOT, -FACE_GAP - RAIL_ENG),
		DVec2::new(RAIL_WAIST, -FACE_GAP),
		DVec2::new(RAIL_WAIST, 0.8),
	];
	union(&plate, &extrude(&ccw(rail_pts), RAIL_LEN))
}

/// End-to-end noodle coupler: a Ø68.3 sleeve whose two Ø63.5 sockets meet at an
/// internal Ø40 stop flange, chamfered 48° on BOTH sides so the tube prints
/// axis-vertical with zero supports. One revolve — no booleans at all.
fn coupler() -> Solid {
	let ri = (NOODLE_D - CPLR_SQUEEZE) / 2.0; // 31.75
	let ro = ri + 2.4;
	let len = 90.0;
	let bore = 20.0; // stop-flange bore radius (water drains, fingers push through)
	let ch = (ri - bore) * (48.0_f64.to_radians()).tan(); // 48° chamfer height
	let half = len / 2.0;
	let fl = 1.5; // flange straight bore half-height
	let profile = vec![
		DVec2::new(ro, 0.0),
		DVec2::new(ro, len),
		DVec2::new(ri, len),
		DVec2::new(ri, half + fl + ch),
		DVec2::new(bore, half + fl),
		DVec2::new(bore, half - fl),
		DVec2::new(ri, half - fl - ch),
		DVec2::new(ri, 0.0),
	];
	revolve(&profile, SEG)
}

/// Raft clip: two plate-less Ø63 C-rings at 80 mm pitch, bridged by a web fused
/// into both rings' centre-facing walls (clear of both bores and both openings).
fn raft_clip() -> Solid {
	let ring = prism_x(&noodle_ring_profile(false), -CLAMP_LEN / 2.0, CLAMP_LEN);
	let a = ring.transformed(tr(0.0, -RAFT_PITCH / 2.0, 0.0));
	let b = ring.transformed(tr(0.0, RAFT_PITCH / 2.0, 0.0));
	// web ±8.0 in y: bore near-edge is at ±8.5 (z = 0), so 0.5 clear of the foam
	let web = cuboid(v(-CLAMP_LEN / 2.0, -8.0, -6.0), v(CLAMP_LEN / 2.0, 8.0, 6.0));
	union(&union(&a, &b), &web)
}

// ---- gates ---------------------------------------------------------------------

fn emit(name: &str, s: &Solid, to_print: DAffine3) -> (bool, f64) {
	let val = validate(s);
	let mut printed = s.transformed(to_print);
	let zmin = tessellate_default(&printed)
		.positions
		.iter()
		.map(|p| p.z as f64)
		.fold(f64::INFINITY, f64::min);
	printed = printed.transformed(tr(0.0, 0.0, -zmin));
	let mesh_p = tessellate_default(&printed);
	let rep = mesh_p.support_free_report(Vec3::Z, 45.0, 0.3);
	let (lo, hi) = mesh_aabb(&mesh_p);
	let ext = hi - lo;
	let fits = ext.x <= 250.0 && ext.y <= 210.0 && ext.z <= 220.0;
	let wt = mesh_p.is_watertight();
	let vol = volume(s).abs();
	let ok = val.is_valid() && wt && rep.steep_area < 1e-6 && rep.max_bridge_span <= 12.0 && fits;
	let _ = std::fs::write(format!("pool_system/noodlehub/parts/{name}.stl"), mesh_p.to_stl_binary());
	println!(
		"  {name:24} valid={:5} wt={wt:5} steep={:8.3} mm²  bridge≤{:5.1}  {:3.0}g  {:6.0}mm³  {}",
		val.is_valid(),
		rep.steep_area,
		rep.max_bridge_span,
		vol * PETG_G_PER_MM3,
		vol,
		if ok { "OK" } else { "<<< FAIL" }
	);
	(ok, vol)
}

fn merge_into(dst: &mut Mesh, src: &Mesh) {
	let base = dst.positions.len() as u32;
	dst.positions.extend_from_slice(&src.positions);
	dst.indices.extend(src.indices.iter().map(|i| i + base));
}

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

fn check(label: &str, pass: bool, detail: String, ok: &mut bool) {
	*ok &= pass;
	println!("  {label:52} {detail}  {}", if pass { "OK" } else { "<<< FAIL" });
}

fn main() {
	let _ = std::fs::create_dir_all("pool_system/noodlehub/parts");
	println!("NOODLEDOCK noodle hubs — parts (STLs in print orientation):\n");

	let dock = noodle_dock();
	let cplr = coupler();
	let clip = raft_clip();
	let coupon = prism_x(&noodle_ring_profile(false), -4.0, 8.0);
	let gauge = rail_gauge();

	let bore_up = DAffine3::from_rotation_y(-FRAC_PI_2);
	let ident = DAffine3::IDENTITY;

	let mut ok = true;
	let mut vols: Vec<(&str, f64)> = Vec::new();
	for (name, s, m) in [
		("dock_65mm_noodle", &dock, bore_up),
		("coupler_65mm", &cplr, ident),
		("clip_raft_2x80mm", &clip, bore_up),
		("coupon_noodle_65mm", &coupon, bore_up),
	] {
		let (o, vol) = emit(name, s, m);
		ok &= o;
		vols.push((name, vol));
	}

	// ---- negative control: the raft clip audited as-used (two bore ceilings) ---
	let wrong = tessellate_default(&clip).support_free_report(Vec3::Z, 45.0, 0.3);
	let nc = wrong.steep_area > 500.0;
	ok &= nc;
	println!(
		"\nA-PRINT NC: raft clip audited as-used -> steep {:.1} mm² (must exceed 500) {}",
		wrong.steep_area,
		if nc { "OK" } else { "<<< FAIL" }
	);

	// ---- foam grip + snap arithmetic -------------------------------------------
	println!();
	let chord = 2.0 * bore_r() * LIP_DEG.to_radians().sin();
	check(
		"dock bore squeeze (foam grip)",
		(0.02..=0.06).contains(&(DOCK_SQUEEZE / NOODLE_D)),
		format!("Ø{:.1} bore on Ø{NOODLE_D} noodle = {:.1}% squeeze (want 2–6%)", 2.0 * bore_r(), 100.0 * DOCK_SQUEEZE / NOODLE_D),
		&mut ok,
	);
	check(
		"dock lip chord vs noodle (snap retention)",
		(0.03..=0.10).contains(&((NOODLE_D - chord) / NOODLE_D)),
		format!("chord {chord:.1} vs Ø{NOODLE_D} = {:.1}% interference (foam band 3–10%)", 100.0 * (NOODLE_D - chord) / NOODLE_D),
		&mut ok,
	);
	check(
		"coupler socket squeeze",
		(0.015..=0.05).contains(&(CPLR_SQUEEZE / NOODLE_D)),
		format!("Ø{:.1} socket = {:.1}% squeeze (want 1.5–5%)", NOODLE_D - CPLR_SQUEEZE, 100.0 * CPLR_SQUEEZE / NOODLE_D),
		&mut ok,
	);

	// ---- entry-1 compatibility: the POOLDOCK rail seats in THIS dock's track ---
	println!("\nposed fits (entry-1 rail gauge in the noodle dock):");
	let yf = face_y();
	let m_dock = tessellate_default(&dock);
	relation("rail gauge seated on stop", &m_dock, &posed(&gauge, tr(0.0, yf + FACE_GAP, TRK_BOT)), true, &mut ok);
	relation("rail gauge mid-slide", &m_dock, &posed(&gauge, tr(0.0, yf + FACE_GAP, TRK_BOT + 20.0)), false, &mut ok);

	// ---- buoyancy report (the load-bearing claim of this entry) ----------------
	// Scenario: 2 noodles × 1.2 m at 80 mm pitch, 2 raft clips, 1 noodle dock,
	// entry-1 cup holder (43.9 cm³ — gated in pool_tubedock.rs) + a full drink.
	println!("\nbuoyancy (engine volumes -> grams; assumptions are the constants at top):");
	// entry-1 cup holder measured 43.9 cm³ / 56 g by its own gates; 65 cm³ here is
	// a deliberate ~50% over-estimate so this gate cannot go stale optimistically
	let cup_vol_mm3 = 65000.0;
	let noodle_vol = std::f64::consts::PI * (NOODLE_D / 2.0) * (NOODLE_D / 2.0) * RAFT_NOODLE_LEN;
	let displacement_half = 2.0 * noodle_vol * 0.5 * WATER_G_PER_MM3;
	let clip_vol = vols.iter().find(|(n, _)| *n == "clip_raft_2x80mm").unwrap().1;
	let dock_vol = vols.iter().find(|(n, _)| *n == "dock_65mm_noodle").unwrap().1;
	let printed_g = (2.0 * clip_vol + dock_vol + cup_vol_mm3) * PETG_G_PER_MM3;
	let foam_g = 2.0 * noodle_vol * FOAM_G_PER_MM3;
	let load_g = printed_g + foam_g + DRINK_G;
	let margin = displacement_half / load_g;
	check(
		"raft floats at half submersion",
		margin >= 1.5,
		format!("supports {displacement_half:.0} g vs load {load_g:.0} g (printed {printed_g:.0} + foam {foam_g:.0} + drink {DRINK_G:.0}) -> {margin:.2}x (want >=1.5x)"),
		&mut ok,
	);
	// Static no-capsize bound: everything hanging on ONE noodle's side must stay
	// under half that noodle's full displacement, so heel just presses it deeper.
	let hang_g = (dock_vol + cup_vol_mm3) * PETG_G_PER_MM3 + DRINK_G;
	let one_full = noodle_vol * WATER_G_PER_MM3;
	check(
		"hang load vs one-noodle displacement",
		hang_g <= 0.5 * one_full,
		format!("{hang_g:.0} g hanging vs {:.0} g reserve (half of one noodle) ", 0.5 * one_full),
		&mut ok,
	);

	// ---- assembly scene for renders + the assembly-doc (posed component STLs) --
	// Scene: a 2-noodle drink raft (clips at ±110), the dock + entry-1 cup holder
	// on the starboard noodle, and the coupler extending the port noodle aft —
	// the whole system story in one exploded sheet (tools/assembly_doc.py).
	let _ = std::fs::create_dir_all("pool_system/noodlehub/assembly_parts");
	let noodle_l = cylinder(v(-200.0, -RAFT_PITCH / 2.0, 0.0), v(1.0, 0.0, 0.0), NOODLE_D / 2.0, 398.5, SEG);
	let noodle_r = cylinder(v(-200.0, RAFT_PITCH / 2.0, 0.0), v(1.0, 0.0, 0.0), NOODLE_D / 2.0, 400.0, SEG);
	// coupler modelled axis-vertical (z 0..90): rotate z->x, centre its stop
	// flange on the port noodle's aft end (x = 200), sockets facing ±x
	let cplr_pose = tr(200.0 - 45.0, -RAFT_PITCH / 2.0, 0.0) * DAffine3::from_rotation_y(FRAC_PI_2);
	let noodle_ext = cylinder(v(201.5, -RAFT_PITCH / 2.0, 0.0), v(1.0, 0.0, 0.0), NOODLE_D / 2.0, 88.5, SEG);
	let scene: Vec<(&str, Mesh)> = vec![
		("noodle_port", tessellate_default(&noodle_l)),
		("noodle_starboard", tessellate_default(&noodle_r)),
		("clip_bow", posed(&clip, tr(-110.0, 0.0, 0.0))),
		("clip_stern", posed(&clip, tr(110.0, 0.0, 0.0))),
		("dock", posed(&dock, tr(0.0, RAFT_PITCH / 2.0, 0.0))),
		("coupler", posed(&cplr, cplr_pose)),
		("noodle_next", tessellate_default(&noodle_ext)),
	];
	let mut asm = Mesh::default();
	for (name, m) in &scene {
		merge_into(&mut asm, m);
		let _ = std::fs::write(format!("pool_system/noodlehub/assembly_parts/{name}.stl"), m.to_stl_binary());
	}
	let _ = std::fs::write("pool_system/noodlehub/ASSEMBLY.stl", asm.to_stl_binary());

	println!("\nNOODLEDOCK: {}", if ok { "ALL GATES PASS" } else { "<<< FAIL" });
	std::process::exit(if ok { 0 } else { 1 });
}
