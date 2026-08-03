//! POOLDOCK — snap-on accessory docks for frame-pool rails (Printables "Pool
//! Accessories" flash contest, July 2026).
//!
//! One printed C-clamp snaps DOWN onto the horizontal top-rail tube of a frame
//! pool (three sizes: Ø25.4 / Ø32 / Ø38 mm — the common Intex/Bestway rail
//! diameters). A vertical dovetail TRACK on its outboard plate accepts drop-in
//! attachments — towel hook, cup holder, pole ring, hose ring — which slide in
//! from the top and rest on a stop, so gravity itself is the latch. Hook loads
//! press the tube into the CLOSED top of the C: a downward pull cannot pop the
//! clamp open (the opening faces down, away from the load path).
//!
//! The track is the POOLDOCK profile: 6.0 mm opening / 12.0 mm root / 2.5 mm
//! deep — the same 6.0 mm opening as DOVESTACK (drawer_system.rs) but with a
//! deliberately STEEPER 50° flank so the dock prints bore-vertical with zero
//! supports (a 31° DOVESTACK flank would be a steep overhang in that
//! orientation). Compat is honest: a DOVESTACK male (root 8.42 > opening 6.0)
//! drops in and is CAPTIVE, riding with ~1.2 mm of flank play; POOLDOCK males
//! fit snug (0.2 mm per flank, 0.15 mm at the root — DOVESTACK numbers).
//!
//! Every part prints support-free in the orientation its STL ships in, and the
//! kernel's `support_free_report` gate (steep_area == 0, bridges ≤ 12 mm) is
//! asserted per part, plus a wrong-orientation negative control proving the
//! gate bites. Fits are machine-checked on posed meshes (tube-in-bore, rail-in-
//! track seated + mid-slide, tumbler-in-cup, hose/pole-in-ring), and snap
//! retention is asserted arithmetically (lip chord 1.5–5 % under tube Ø).
//!
//! Contract: pool_system/pooldock/DESIGN.md (every line asserted here).
//! Run: cargo run --example pool_tubedock -p kernel-model --release
//!   -> pool_system/pooldock/ (exit 1 on any FAIL)

use kernel_brep::math::{DAffine3, DVec2, DVec3};
use kernel_brep::{
	cylinder, cuboid, difference, extrude, overlap_volume, tessellate_default, union, validate, volume, Mesh, Solid,
};
use kernel_core::math::Vec3;
use std::f64::consts::FRAC_PI_2;

// ---- tube sizes (mm) -----------------------------------------------------------
const TUBE_SIZES: [f64; 3] = [25.4, 32.0, 38.0]; // common frame-pool rail tubes
const BORE_CLR: f64 = 0.15; // radial bore clearance (snug ride on the rail)
const WALL: f64 = 3.2; // C-ring wall
const LIP_DEG: f64 = 105.0; // bore lip half-angle from top (210° wrap)
const LIP_OUT_DEG: f64 = 113.0; // outer-surface lip angle: the extra 8° ramps the
                                // lip face so pushing onto the tube wedges it open
const CLAMP_LEN: f64 = 40.0; // clamp length along the tube (X)

// ---- the POOLDOCK track (see module doc for the DOVESTACK relationship) --------
const TRK_OPEN: f64 = 3.0; // track half-width at the face (6.0 opening = DOVESTACK)
const TRK_ROOT: f64 = 6.0; // track half-width at the root (50° flank, support-free
                           // when printed bore-vertical; DOVESTACK's 4.5 would not be)
const TRK_DEPTH: f64 = 2.5; // depth into the plate
const TRK_TOP: f64 = 12.0; // cutter top (above the plate top edge -> open mouth)
const TRK_BOT: f64 = -46.0; // blind bottom: the stop the rail rests on
const PLATE_T: f64 = 4.0; // plate thickness (2.5 track + 1.5 ligament)
const PLATE_BOT: f64 = -50.0; // plate bottom (4.0 of stop under the track)
const PLATE_TOP: f64 = 8.0; // plate top edge (track mouth opens through it)

// ---- male rail (attachment side) -----------------------------------------------
const RAIL_WAIST: f64 = 2.8; // half-width crossing the face plane (0.2/flank clr)
const RAIL_ROOT: f64 = 5.55; // half-width at full engagement (0.27 clr at root width)
const RAIL_ENG: f64 = 2.35; // engagement depth (0.15 axial clearance to track root)
const RAIL_LEN: f64 = 40.0;
const FACE_GAP: f64 = 0.2; // attachment plate face to dock plate face
const APLATE_T: f64 = 3.5; // attachment back-plate thickness

const SEG: usize = 128;
const PETG_G_PER_MM3: f64 = 0.00127;

// ---- tiny helpers --------------------------------------------------------------

fn v(x: f64, y: f64, z: f64) -> DVec3 {
	DVec3::new(x, y, z)
}
fn tr(x: f64, y: f64, z: f64) -> DAffine3 {
	DAffine3::from_translation(v(x, y, z))
}

/// Force a polygon CCW (extrude() wants CCW; profiles below are written for
/// legibility, not winding).
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

/// Prism from a profile in the model (x,z) plane, spanning y ∈ [y0, y1].
fn prism_y(profile: &[(f64, f64)], y0: f64, y1: f64) -> Solid {
	let p: Vec<DVec2> = profile.iter().map(|&(x, z)| DVec2::new(x, z)).collect();
	extrude(&ccw(p), y1 - y0).transformed(tr(0.0, y1, 0.0) * DAffine3::from_rotation_x(FRAC_PI_2))
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

/// Push an arc about the (y,z) origin: y = r·sin φ, z = r·cos φ, φ in degrees
/// (φ = 0 is the top of the tube, +φ is the outboard side).
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

// ---- geometry helpers per size -------------------------------------------------

fn bore_r(d: f64) -> f64 {
	d / 2.0 + BORE_CLR
}
fn outer_r(d: f64) -> f64 {
	bore_r(d) + WALL
}
/// Outboard face plane of the dock plate (the track is cut into this face).
fn face_y(d: f64) -> f64 {
	outer_r(d) + 3.4
}

/// C-ring (+ optional fused track plate) profile in the (y,z) plane, tube axis
/// along X, tube centre at the origin, opening facing DOWN. One simple polygon:
/// there are no booleans anywhere in the snap geometry.
fn dock_profile(d: f64, with_plate: bool) -> Vec<(f64, f64)> {
	let (ri, ro) = (bore_r(d), outer_r(d));
	let yf = face_y(d);
	let pi_face = yf - PLATE_T; // plate inner face: overlaps the ring by 0.6 (a
	                            // proper 2D overlap; a tangent kiss would be degenerate)
	let mut p: Vec<(f64, f64)> = Vec::new();
	if with_plate {
		let a_top = (ro * ro - PLATE_TOP * PLATE_TOP).sqrt().atan2(PLATE_TOP).to_degrees();
		let zx = -(ro * ro - pi_face * pi_face).sqrt();
		let a_x = pi_face.atan2(zx).to_degrees();
		push_arc(&mut p, ro, -LIP_OUT_DEG, a_top, 128); // over the top to z = PLATE_TOP
		p.push((yf, PLATE_TOP)); // plate top edge (track mouth opens through it)
		p.push((yf, PLATE_BOT));
		p.push((pi_face, PLATE_BOT));
		p.push((pi_face, zx)); // plate inner face back up to the ring
		push_arc(&mut p, ro, a_x, LIP_OUT_DEG, 16); // short arc down to the lip
	} else {
		push_arc(&mut p, ro, -LIP_OUT_DEG, LIP_OUT_DEG, 192);
	}
	// +y lip: outer corner at 113°, inner corner at 105° — the 8° stagger makes the
	// lip face a wedge ramp, so the tube pushed up from below levers the lips open.
	p.push((ri * LIP_DEG.to_radians().sin(), ri * LIP_DEG.to_radians().cos()));
	push_arc(&mut p, ri, LIP_DEG, -LIP_DEG, 192); // bore
	// closing edge back to the first point is the mirrored -y lip ramp
	dedup(p)
}

/// The track cutter: a vertical prism whose (x,y) cross-section is the POOLDOCK
/// trapezoid, proud of the face by 1 mm, open at the top, blind at TRK_BOT.
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

fn dock(d: f64) -> Solid {
	let body = prism_x(&dock_profile(d, true), -CLAMP_LEN / 2.0, CLAMP_LEN);
	difference(&body, &track_cutter(face_y(d)))
}

/// Attachment back plate + male rail, in the attachment's LOCAL frame: plate
/// front face at y = 0 (mounts at world y = face_y + FACE_GAP), rail protruding
/// -y into the track, part bottom at z = 0 (= rail bottom, rests on the stop).
fn attach_base(w: f64, h: f64, rail_len: f64) -> Solid {
	let plate = cuboid(v(-w / 2.0, 0.0, 0.0), v(w / 2.0, APLATE_T, h));
	let rail_pts = vec![
		DVec2::new(-RAIL_WAIST, 0.8), // buried 0.8 into the plate: overlap, not kiss
		DVec2::new(-RAIL_WAIST, -FACE_GAP),
		DVec2::new(-RAIL_ROOT, -FACE_GAP - RAIL_ENG),
		DVec2::new(RAIL_ROOT, -FACE_GAP - RAIL_ENG),
		DVec2::new(RAIL_WAIST, -FACE_GAP),
		DVec2::new(RAIL_WAIST, 0.8),
	];
	let body = union(&plate, &extrude(&ccw(rail_pts), rail_len));
	// Production lead-in: the rail enters the track BOTTOM-first, so its bottom
	// ~5 mm tapers to a 3.2-wide, 1.0-shallower tip — the tip self-centres into
	// the 6.0 mouth instead of demanding blind 0.2 mm alignment, and the narrow
	// first layers also keep elephant-foot flare out of the mating envelope.
	// Every taper face is 50° from horizontal, so the upright print stays
	// support-free (a 45°-flat lead-in would be a steep overhang).
	// (cutters start at z = -1, NOT 0: a cutter face coplanar with the part's own
	// bed face is exactly the coincident-face degeneracy the kernel refuses)
	let lead_flank = |s: f64| {
		prism_y(
			&[(s * 1.6, -1.0), (s * 6.5, -1.0), (s * 6.5, 5.84)], // 54.4° cut face
			-3.0,
			-0.05, // rail material only — never nicks the plate at y >= 0
		)
	};
	let lead_root = prism_x(
		&[(-1.55, -1.0), (-3.2, -1.0), (-3.2, 1.97)], // 61° cut face, 0.44 y lead-in at the tip
		-6.5,
		13.0,
	);
	let cutter = union(&union(&lead_flank(1.0), &lead_flank(-1.0)), &lead_root);
	difference(&body, &cutter)
}

/// Chevron towel hook: the prong climbs at ≥ 50° so every downward face beats
/// the 45° support threshold; 30 mm wide for wet towels.
fn hook_towel() -> Solid {
	let base = attach_base(30.0, 55.0, RAIL_LEN);
	let prong = vec![
		(2.5, 6.0),
		(28.0, 36.5), // lower edge: 50.1° from horizontal
		(28.0, 50.0), // tip outer (vertical)
		(23.5, 50.0),
		(23.5, 41.0), // tip inner (vertical)
		(2.5, 16.0), // upper edge (upward-facing, any angle is fine)
	];
	union(&base, &prism_x(&prong, -15.0, 30.0))
}

/// Open C-ring on a plate, axis vertical, opening facing outboard (+y): used at
/// Ø42 as a hose guide and at Ø34 as a pole/brush ring. Pure vertical prism —
/// support-free by construction.
fn ring_attachment(inner_r: f64, gap_half_deg: f64, ring_h: f64) -> Solid {
	let base = attach_base(30.0, 50.0, RAIL_LEN);
	let ro = inner_r + 3.2;
	let yc = APLATE_T + ro - 1.5; // ring overlaps the plate by 1.5
	let n = 96;
	let mut pts: Vec<DVec2> = Vec::new();
	for i in 0..=n {
		let a = (gap_half_deg + (360.0 - 2.0 * gap_half_deg) * i as f64 / n as f64).to_radians();
		pts.push(DVec2::new(ro * a.sin(), yc + ro * a.cos()));
	}
	for i in 0..=n {
		let a = (360.0 - gap_half_deg - (360.0 - 2.0 * gap_half_deg) * i as f64 / n as f64).to_radians();
		pts.push(DVec2::new(inner_r * a.sin(), yc + inner_r * a.cos()));
	}
	union(&base, &extrude(&ccw(pts), ring_h))
}

fn circle(cx: f64, cy: f64, r: f64) -> Vec<DVec2> {
	(0..SEG)
		.map(|i| {
			let a = std::f64::consts::TAU * i as f64 / SEG as f64;
			DVec2::new(cx + r * a.cos(), cy + r * a.sin())
		})
		.collect()
}

/// Cup holder: Ø86 bore (cans, bottles, most tumblers), Ø60 drain in the floor,
/// hung 2 mm off the plate on a low arm. Everything is a vertical wall or sits
/// on the bed — zero supports, zero bridges.
fn holder_cup() -> Solid {
	let base = attach_base(30.0, 50.0, RAIL_LEN);
	let yc = APLATE_T + 2.0 + 46.0; // band outer r = 46, 2 mm off the plate back
	let band = kernel_brep::extrude_with_holes(&circle(0.0, yc, 46.0), &[circle(0.0, yc, 43.0)], 34.0);
	// floor outer radius 45: buried inside the band annulus (43..46), never
	// coincident with its wall, never intruding into the Ø86 cup cavity
	let floor = kernel_brep::extrude_with_holes(&circle(0.0, yc, 45.0), &[circle(0.0, yc, 30.0)], 3.0);
	let arm = cuboid(v(-9.0, 1.0, 0.0), v(9.0, yc - 43.5, 8.0));
	union(&union(&union(&base, &arm), &floor), &band)
}

/// Track fit coupon: a 20-minute print of the female track on a small block.
fn coupon_track() -> Solid {
	let block = cuboid(v(-8.0, 0.0, 0.0), v(8.0, 5.0, 30.0));
	let pts = vec![
		DVec2::new(-TRK_OPEN, 6.0),
		DVec2::new(-TRK_OPEN, 5.0),
		DVec2::new(-TRK_ROOT, 5.0 - TRK_DEPTH),
		DVec2::new(TRK_ROOT, 5.0 - TRK_DEPTH),
		DVec2::new(TRK_OPEN, 5.0),
		DVec2::new(TRK_OPEN, 6.0),
	];
	let cutter = extrude(&ccw(pts), 31.0).transformed(tr(0.0, 0.0, 4.0));
	difference(&block, &cutter)
}

// ---- gates ---------------------------------------------------------------------

fn emit(name: &str, s: &Solid, to_print: DAffine3) -> bool {
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
	let _ = std::fs::write(format!("pool_system/pooldock/parts/{name}.stl"), mesh_p.to_stl_binary());
	println!(
		"  {name:24} valid={:5} wt={wt:5} steep={:8.3} mm²  bridge≤{:5.1}  {:3.0}g  {:6.0}mm³  {}",
		val.is_valid(),
		rep.steep_area,
		rep.max_bridge_span,
		vol * PETG_G_PER_MM3,
		vol,
		if ok { "OK" } else { "<<< FAIL" }
	);
	ok
}

fn merge_into(dst: &mut Mesh, src: &Mesh) {
	let base = dst.positions.len() as u32;
	dst.positions.extend_from_slice(&src.positions);
	dst.indices.extend(src.indices.iter().map(|i| i + base));
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

fn translated_z(m: &Mesh, dz: f32) -> Mesh {
	let mut out = m.clone();
	for p in &mut out.positions {
		p.z += dz;
	}
	out
}

fn main() {
	let _ = std::fs::create_dir_all("pool_system/pooldock/parts");
	println!("POOLDOCK frame-pool rail docks — parts (STLs in print orientation):\n");

	let docks: Vec<Solid> = TUBE_SIZES.iter().map(|&d| dock(d)).collect();
	let coupons: Vec<Solid> = TUBE_SIZES
		.iter()
		.map(|&d| prism_x(&dock_profile(d, false), -4.0, 8.0))
		.collect();
	let hook = hook_towel();
	let cup = holder_cup();
	let ring_pole = ring_attachment(17.0, 50.0, 30.0); // Ø34: telescopic pole / brush handle
	let ring_hose = ring_attachment(21.0, 46.0, 30.0); // Ø42: 32 & 38 mm pool hose
	let c_track = coupon_track();
	let c_rail = attach_base(24.0, 20.0, 20.0);

	// docks + tube coupons print bore-vertical (tube axis X -> print Z);
	// attachments print upright exactly as they hang — every wall vertical.
	let bore_up = DAffine3::from_rotation_y(-FRAC_PI_2);
	let ident = DAffine3::IDENTITY;

	let parts: Vec<(String, &Solid, DAffine3)> = vec![
		("dock_25.4mm_tube".into(), &docks[0], bore_up),
		("dock_32mm_tube".into(), &docks[1], bore_up),
		("dock_38mm_tube".into(), &docks[2], bore_up),
		("hook_towel_30mm".into(), &hook, ident),
		("holder_cup_86mm".into(), &cup, ident),
		("ring_pole_34mm".into(), &ring_pole, ident),
		("ring_hose_42mm".into(), &ring_hose, ident),
		("coupon_tube_25.4mm".into(), &coupons[0], bore_up),
		("coupon_tube_32mm".into(), &coupons[1], bore_up),
		("coupon_tube_38mm".into(), &coupons[2], bore_up),
		("coupon_track".into(), &c_track, ident),
		("coupon_rail".into(), &c_rail, ident),
	];
	let mut ok = true;
	for (name, s, m) in &parts {
		ok &= emit(name, s, *m);
	}

	// ---- negative control: the support gate must BITE in a wrong orientation
	// (dock as-used: the bore ceiling and the track's lower 50° flank both
	// become steep overhangs)
	let wrong = tessellate_default(&docks[2]).support_free_report(Vec3::Z, 45.0, 0.3);
	let nc = wrong.steep_area > 500.0;
	ok &= nc;
	println!(
		"\nA-PRINT NC: dock_38 audited as-used -> steep {:.1} mm² (must exceed 500) {}",
		wrong.steep_area,
		if nc { "OK" } else { "<<< FAIL" }
	);

	// ---- snap retention: lip chord under tube Ø by 1.5–5% — enough bite to stay
	// on, little enough that PETG lips deflect ≤ 0.5 mm each to snap on
	println!();
	for &d in &TUBE_SIZES {
		let chord = 2.0 * bore_r(d) * LIP_DEG.to_radians().sin();
		let margin = (d - chord) / d;
		let pass = (0.015..=0.05).contains(&margin);
		ok &= pass;
		println!(
			"  retention Ø{d:>4}: lip chord {chord:6.2} vs tube {d:5.1} -> interference {:4.1}% (want 1.5–5%) {}",
			margin * 100.0,
			if pass { "OK" } else { "<<< FAIL" }
		);
	}
	// A DOVESTACK male (root half 4.21, drawer_system.rs KEY_ROOT) is CAPTIVE in
	// this track: it cannot pass the 6.0 opening but clears the 12.0 root; ~1.2 mm
	// flank play is the honest cost of the steeper flank.
	let dovestack_root = 4.21_f64;
	let captive = dovestack_root > TRK_OPEN && dovestack_root < TRK_ROOT;
	ok &= captive;
	println!(
		"  DOVESTACK compat: key root {dovestack_root} vs opening {TRK_OPEN}/root {TRK_ROOT} -> captive {}",
		if captive { "OK" } else { "<<< FAIL" }
	);

	// ---- PRODUCTION VALIDATION MATRIX ------------------------------------------
	// Every dock size × every attachment: (a) seated = real contact (min_dist 0)
	// AND zero solid interpenetration at a 0.05 lift (min-distance alone cannot
	// tell touching from overlapping — overlap_volume can); (b) the FULL drop-in
	// path swept in 4 mm steps from above the mouth to the seat, ≥ 0.10 clear at
	// every pose; (c) the rail tube itself, per size: clearance + zero overlap.
	println!("\nproduction validation matrix (3 docks × 4 attachments + tube per size):");
	let atts: [(&str, &Solid); 4] = [("hook", &hook), ("cup", &cup), ("pole", &ring_pole), ("hose", &ring_hose)];
	for (di, &d) in TUBE_SIZES.iter().enumerate() {
		let yf = face_y(d);
		let dock_s = &docks[di];
		let m_dock = tessellate_default(dock_s);
		let tube = cylinder(v(-60.0, 0.0, 0.0), v(1.0, 0.0, 0.0), d / 2.0, 120.0, SEG);
		let t_clr = m_dock.min_distance(&tessellate_default(&tube));
		let t_ovl = overlap_volume(dock_s, &tube);
		let t_ok = t_clr >= 0.10 && matches!(t_ovl, Some(v) if v.abs() < 0.05);
		ok &= t_ok;
		println!(
			"  dock Ø{d:>4} × tube      : clearance {t_clr:5.3}  overlap {:>8}  {}",
			t_ovl.map_or("REFUSED".into(), |v| format!("{v:.3}mm³")),
			if t_ok { "OK" } else { "<<< FAIL" }
		);
		for (aname, att) in atts {
			let seat = tr(0.0, yf + FACE_GAP, TRK_BOT);
			let m_seat = posed(att, seat);
			let seat_d = m_dock.min_distance(&m_seat);
			// interpenetration at 0.05 lift and mid-slide (exact-contact booleans
			// would be a coincident-face degeneracy, so the overlap poses lift off)
			let ovl_lo = overlap_volume(dock_s, &att.transformed(tr(0.0, yf + FACE_GAP, TRK_BOT + 0.05)));
			let ovl_mid = overlap_volume(dock_s, &att.transformed(tr(0.0, yf + FACE_GAP, TRK_BOT + 25.0)));
			// swept insertion path: rail bottom from 2 above the seat to 10 above
			// the mouth (the 0.05-lift pose is the overlap gate's — at that height
			// the bottom face is 0.05 from the stop by construction)
			let mut sweep_min = f64::INFINITY;
			let mut dz = 2.0;
			while dz <= 58.0 {
				sweep_min = sweep_min.min(m_dock.min_distance(&translated_z(&m_seat, dz as f32)));
				dz += 4.0;
			}
			let pass = seat_d < 0.06
				&& matches!(ovl_lo, Some(v) if v.abs() < 0.05)
				&& matches!(ovl_mid, Some(v) if v.abs() < 0.05)
				&& sweep_min >= 0.10;
			ok &= pass;
			println!(
				"  dock Ø{d:>4} × {aname:9}: seat {seat_d:5.3}  overlap {}/{}  sweep(15) min {sweep_min:5.3}  {}",
				ovl_lo.map_or("REF".into(), |v| format!("{v:.2}")),
				ovl_mid.map_or("REF".into(), |v| format!("{v:.2}")),
				if pass { "OK" } else { "<<< FAIL" }
			);
		}
	}

	// ---- snap-on strain bound: lip arm as a straight cantilever (conservative:
	// arm length only the 60° arc from the stiff crown region to the lip, which
	// UNDERSTATES the true compliant length) — peak strain must stay under 1%,
	// well inside PETG/ASA elastic range (~2%), so snapping on cannot whiten or
	// crack the lips even on a cold day
	println!("\nsnap-on lip strain (eps = 1.5·t·delta / L², L = 60° arc, t = wall):");
	for &d in &TUBE_SIZES {
		let (ri, ro) = (bore_r(d), outer_r(d));
		let chord = 2.0 * ri * LIP_DEG.to_radians().sin();
		let delta = (d - chord) / 2.0; // required opening per lip
		let arm = ro * 60.0_f64.to_radians();
		let eps = 1.5 * WALL * delta / (arm * arm);
		let pass = eps <= 0.010;
		ok &= pass;
		println!(
			"  Ø{d:>4}: delta {delta:4.2} mm over arm {arm:4.1} mm -> strain {:.2}% (must be <=1%) {}",
			eps * 100.0,
			if pass { "OK" } else { "<<< FAIL" }
		);
	}

	// ---- seated rattle bound (arithmetic, from the clearance scheme): the rail
	// can translate 0.4 across the track (opening 6.0 vs waist 5.6) and ~0.35 in
	// depth (0.15 root + flank-limited outward travel); over a 40 mm engaged rail
	// that is <=0.6° of wobble — attachments hang snug, they do not clatter
	{
		let x_play = 2.0 * (TRK_OPEN - RAIL_WAIST);
		let wobble_deg = (x_play / RAIL_LEN).atan().to_degrees();
		let pass = x_play <= 0.6 && wobble_deg <= 1.0;
		ok &= pass;
		println!(
			"\nseated play: {x_play:.2} mm across track, {wobble_deg:.2}° wobble over the 40 mm rail (limits 0.6 / 1.0°) {}",
			if pass { "OK" } else { "<<< FAIL" }
		);
	}

	// cup gauge: a Ø84 tumbler seats on the floor (0.05 gap) inside the Ø86 bore
	let yc = APLATE_T + 2.0 + 46.0;
	let tumbler = cylinder(v(0.0, yc, 3.05), v(0.0, 0.0, 1.0), 42.0, 60.0, SEG);
	relation("Ø84 tumbler seated in cup holder", &tessellate_default(&cup), &tessellate_default(&tumbler), true, &mut ok);
	// ring gauges: Ø38 hose and Ø30 pole hang through with ≥ 2 mm of air
	let yc_hose = APLATE_T + 21.0 + 3.2 - 1.5;
	let hose = cylinder(v(0.0, yc_hose, -10.0), v(0.0, 0.0, 1.0), 19.0, 60.0, SEG);
	relation("Ø38 hose through hose ring", &tessellate_default(&ring_hose), &tessellate_default(&hose), false, &mut ok);
	let yc_pole = APLATE_T + 17.0 + 3.2 - 1.5;
	let pole = cylinder(v(0.0, yc_pole, -10.0), v(0.0, 0.0, 1.0), 15.0, 60.0, SEG);
	relation("Ø30 pole through pole ring", &tessellate_default(&ring_pole), &tessellate_default(&pole), false, &mut ok);
	// coupons mate exactly like the real parts
	relation(
		"coupon_rail seated in coupon_track",
		&tessellate_default(&c_track),
		&posed(&c_rail, tr(0.0, 5.0 + FACE_GAP, 4.0)),
		true,
		&mut ok,
	);

	// ---- assembly scene for renders + the assembly-doc (posed component STLs) --
	let yf38 = face_y(38.0);
	let dock38 = &docks[2];
	let _ = std::fs::create_dir_all("pool_system/pooldock/assembly_parts");
	let rail = cylinder(v(-180.0, 0.0, 0.0), v(1.0, 0.0, 0.0), 19.0, 360.0, SEG);
	let hangers: [(&str, &str, &Solid, f64); 4] = [
		("dock_a", "hook_towel", &hook, -120.0),
		("dock_b", "holder_cup", &cup, -40.0),
		("dock_c", "ring_pole", &ring_pole, 40.0),
		("dock_d", "ring_hose", &ring_hose, 120.0),
	];
	let mut scene: Vec<(String, Mesh)> = vec![("rail_38mm".into(), tessellate_default(&rail))];
	for (dname, aname, att, x) in hangers {
		scene.push((dname.into(), posed(dock38, tr(x, 0.0, 0.0))));
		scene.push((aname.into(), posed(att, tr(x, yf38 + FACE_GAP, TRK_BOT))));
	}
	let mut asm = Mesh::default();
	for (name, m) in &scene {
		merge_into(&mut asm, m);
		let _ = std::fs::write(format!("pool_system/pooldock/assembly_parts/{name}.stl"), m.to_stl_binary());
	}
	let _ = std::fs::write("pool_system/pooldock/ASSEMBLY.stl", asm.to_stl_binary());
	// crowded-rail check: at the demo's 80 mm dock pitch, no attachment touches
	// its neighbour (the cup band is the widest at Ø92 — the listing quotes the
	// per-attachment rail budget from these same poses)
	println!("\ncrowded-rail neighbour gaps (80 mm dock pitch):");
	let att_scene: Vec<&Mesh> = scene
		.iter()
		.filter(|(n, _)| !n.starts_with("dock") && n.as_str() != "rail_38mm")
		.map(|(_, m)| m)
		.collect();
	for w in att_scene.windows(2) {
		let g = w[0].min_distance(w[1]);
		let pass = g >= 5.0;
		ok &= pass;
		println!("  neighbour gap {g:6.1} mm (must be >=5) {}", if pass { "OK" } else { "<<< FAIL" });
	}

	println!("\nPOOLDOCK: {}", if ok { "ALL GATES PASS" } else { "<<< FAIL" });
	std::process::exit(if ok { 0 } else { 1 });
}
