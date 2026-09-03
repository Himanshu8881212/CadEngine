//! TRI-SWEEP — a one-piece TPU-95A three-edge floor sweeper (squeegee / ice
//! scraper / moisture sweep), printed flat, support-free, 100% infill (solid).
//!
//! An equilateral plate (260 mm edge, 6 mm thick) with three differently
//! beveled working edges — WIDEST chamfer = water squeegee (1.0 mm lip),
//! NARROWEST/steepest = ice scraper (2.5 mm lip), middle = moisture sweep
//! (1.6 mm lip). The grip is NINE finger holes through a raised central pad,
//! laid out as an INVERTED triangle (3 rows of 4, corner holes shared; each
//! corner points at a working-edge midpoint): whichever edge is on the floor,
//! four fingers go through the far row, the palm lies flat on the solid pad,
//! and the corner hole nearest the working edge doubles as a thumb hole —
//! identical grip for all three edges. Discrete holes instead of a big cutout
//! keep the plate core solid: strictly stiffer and stronger.
//!
//! Everything grows from the bed: bevels are top-face cuts, hole/collar walls
//! are vertical — `support_free_report` (steep_area == 0) is asserted, plus
//! watertight/valid/per-edge-lip/volume/bed-fit gates. Exit non-zero on FAIL.
//!
//! Run: cargo run --example sweeper -p kernel-model --release -> sweeper_out/

use kernel_brep::math::{DAffine3, DVec2, DVec3};
use kernel_brep::{cone, cuboid, cylinder, difference, extrude, revolve, tessellate_default, union, validate, volume, Solid};
use kernel_core::math::Vec3;
use std::f64::consts::PI;

// ---- the part (mm) -------------------------------------------------------------
const EDGE: f64 = 260.0; // outer equilateral edge (user ask: 25–28 cm)
const T: f64 = 6.0; // plate thickness
const RC_OUT: f64 = 8.0; // outer corner rounding
const FINGER_D: f64 = 20.0; // finger hole diameter (TPU flexes; snug is good)
const FINGER_PITCH: f64 = 25.0; // hole pitch along a row (4-finger span 95 mm)
const PAD_R: f64 = 57.3; // raised circular grip pad radius (covers holes + 4 mm)
const PAD_TOP: f64 = 12.0; // pad top height (grip walls = 12 mm tall)
const SEG: usize = 48;
// working edges: (name, lip thickness at edge, chamfer run inboard, rot about Z,
// working line load N/mm — ice 200 N scrape, moisture 130 N, water 75 N wipe)
const EDGES: [(&str, f64, f64, f64, f64); 3] =
	[("ice", 2.5, 5.0, 120.0, 0.77), ("water", 1.0, 12.0, 0.0, 0.30), ("moisture", 1.6, 9.0, 240.0, 0.50)];

const TPU_G_PER_MM3: f64 = 0.00122;

// ---- force validation (conservative TPU-95A data, closed-form; no FEA solver
// is bundled with the kernel, so these are honest analytic gates, not FEM) ----
const TPU_E_MPA: f64 = 25.0; // low-end Young's modulus for TPU 95A
const TPU_ULT_MPA: f64 = 25.0; // ultimate tensile strength, conservative
const F_PUSH_N: f64 = 200.0; // hard two-hand shove into ice through the palm bar
const F_HANG_N: f64 = 200.0; // full pull on the four finger scallops
const W_ABUSE_N_MM: f64 = 0.77; // abuse case: the full 200 N shove on ANY edge

fn v(x: f64, y: f64, z: f64) -> DVec3 {
	DVec3::new(x, y, z)
}
fn tr(x: f64, y: f64, z: f64) -> DAffine3 {
	DAffine3::from_translation(v(x, y, z))
}
fn rad(deg: f64) -> f64 {
	deg * PI / 180.0
}

/// The 9 finger-hole centers: an inverted triangle (vertices at 270°/30°/150°,
/// pointing at the working-edge midpoints), 4 holes per side, corners shared.
fn grip_holes() -> Vec<DVec2> {
	let rv = FINGER_PITCH * 3f64.sqrt(); // side = 3*pitch -> circumradius
	let verts: Vec<DVec2> = (0..3)
		.map(|i| {
			let a = rad(270.0 + 120.0 * i as f64);
			DVec2::new(rv * a.cos(), rv * a.sin())
		})
		.collect();
	let mut c = Vec::new();
	for i in 0..3 {
		let (a, b) = (verts[i], verts[(i + 1) % 3]);
		for k in 0..3 {
			c.push(a + (b - a) * (k as f64 / 3.0)); // t=1 is the next side's t=0
		}
	}
	c
}

/// CCW rounded equilateral triangle centered at the origin, first vertex at
/// `base_deg`, corner rounding `rc`. `edge` is the sharp-corner edge length.
fn tri_profile(edge: f64, base_deg: f64, rc: f64) -> Vec<DVec2> {
	let r_circ = edge / 3f64.sqrt();
	let verts: Vec<DVec2> = (0..3)
		.map(|i| {
			let a = rad(base_deg + 120.0 * i as f64);
			DVec2::new(r_circ * a.cos(), r_circ * a.sin())
		})
		.collect();
	let mut pts: Vec<DVec2> = Vec::new();
	let t = rc * 3f64.sqrt(); // tangent distance from a sharp corner
	for i in 0..3 {
		let (a, b) = (verts[i], verts[(i + 1) % 3]);
		// corner arc at b, center 2*rc along the bisector (toward the origin)
		let d_in = (b - a).normalize();
		let q = b * (1.0 - 2.0 * rc / b.length());
		let start = b - d_in * t - q;
		let a0 = start.y.atan2(start.x);
		for k in 0..=14 {
			let a = a0 + rad(120.0) * k as f64 / 14.0;
			pts.push(q + rc * DVec2::new(a.cos(), a.sin()));
		}
	}
	// insurance: extrude() wants CCW
	let area2: f64 =
		pts.iter().zip(pts.iter().cycle().skip(1)).map(|(p, q)| p.x * q.y - q.x * p.y).sum();
	if area2 < 0.0 {
		pts.reverse();
	}
	pts
}

type Seg = ((f64, f64), (f64, f64));

/// Straight-stroke stencil glyphs (6 wide x 10 tall, mm): segment endpoints.
/// Only the letters the three labels need; angular S/C/R keep every stroke a
/// straight bar so each engraves as one rotated cuboid cut.
fn glyph(ch: char) -> &'static [Seg] {
	match ch {
		'W' => &[((0.0, 10.0), (1.5, 0.0)), ((1.5, 0.0), (3.0, 6.5)), ((3.0, 6.5), (4.5, 0.0)), ((4.5, 0.0), (6.0, 10.0))],
		'A' => &[((0.0, 0.0), (3.0, 10.0)), ((3.0, 10.0), (6.0, 0.0)), ((1.6, 4.0), (4.4, 4.0))],
		'T' => &[((0.0, 10.0), (6.0, 10.0)), ((3.0, 10.0), (3.0, 0.0))],
		'E' => &[((0.0, 0.0), (0.0, 10.0)), ((0.0, 10.0), (6.0, 10.0)), ((0.0, 5.0), (4.5, 5.0)), ((0.0, 0.0), (6.0, 0.0))],
		'R' => &[((0.0, 0.0), (0.0, 10.0)), ((0.0, 10.0), (6.0, 10.0)), ((6.0, 10.0), (6.0, 5.0)), ((6.0, 5.0), (0.0, 5.0)), ((2.5, 5.0), (6.0, 0.0))],
		'I' => &[((3.0, 0.0), (3.0, 10.0))],
		'C' => &[((6.0, 10.0), (0.0, 10.0)), ((0.0, 10.0), (0.0, 0.0)), ((0.0, 0.0), (6.0, 0.0))],
		'M' => &[((0.0, 0.0), (0.0, 10.0)), ((0.0, 10.0), (3.0, 5.0)), ((3.0, 5.0), (6.0, 10.0)), ((6.0, 10.0), (6.0, 0.0))],
		'S' => &[((6.0, 10.0), (0.0, 10.0)), ((0.0, 10.0), (0.0, 5.0)), ((0.0, 5.0), (6.0, 5.0)), ((6.0, 5.0), (6.0, 0.0)), ((6.0, 0.0), (0.0, 0.0))],
		_ => &[],
	}
}

/// Engrave `word` 1.0 mm deep into the top face, letters upright when the edge
/// at 270°+rot is down: block centered at local x=56 (right of the pad skirt),
/// letters spanning y in [-61, -51] (2 mm clear of the widest bevel band,
/// 1.5+ mm clear of the pad rim). One rotated cuboid cut per stroke.
fn engrave_label(mut s: Solid, word: &str, rot_deg: f64) -> Solid {
	const STROKE: f64 = 2.2;
	const PITCH: f64 = 8.5; // 6 wide + 2.5 gap
	let width = 6.0 + (word.len() as f64 - 1.0) * PITCH;
	let x0 = 56.0 - width / 2.0; // centered at 56: first letter clears the pad skirt base
	for (i, ch) in word.chars().enumerate() {
		for &((ax, ay), (bx, by)) in glyph(ch) {
			let a = DVec2::new(x0 + i as f64 * PITCH + ax, -61.0 + ay);
			let b = DVec2::new(x0 + i as f64 * PITCH + bx, -61.0 + by);
			let (mid, len) = ((a + b) * 0.5, (b - a).length() + STROKE);
			let bar = cuboid(v(-len / 2.0, -STROKE / 2.0, T - 1.0), v(len / 2.0, STROKE / 2.0, T + 1.0));
			let place = tr(mid.x, mid.y, 0.0) * DAffine3::from_rotation_z((b - a).y.atan2((b - a).x));
			s = difference(&s, &bar.transformed(DAffine3::from_rotation_z(rad(rot_deg)) * place));
		}
	}
	s
}

/// Top-face chamfer cutter for the outer edge whose midpoint sits at angle
/// 270°+rot: a big cuboid tilted about X so its bottom face is the bevel plane
/// (z = lip at the edge, rising to the plate top `run` mm inboard). The hinge
/// line sits 0.5 mm outboard of the edge so the tool crosses the edge face
/// transversally (no exact edge-on-face tangency), and the inboard extent stops
/// 0.2 mm above the plate top so the collar is never touched.
fn bevel_tool(lip: f64, run: f64, rot_deg: f64) -> Solid {
	let inr = EDGE / (2.0 * 3f64.sqrt());
	let alpha = ((T - lip) / run).atan();
	let reach = 0.5 + (T + 0.2 - lip) * run / (T - lip); // pre-rotation width * cos(alpha)
	let tool = cuboid(v(-EDGE, 0.0, 0.0), v(EDGE, reach / alpha.cos(), 40.0));
	let place = tr(0.0, -inr - 0.5, lip - 0.5 * alpha.tan()) * DAffine3::from_rotation_x(alpha);
	tool.transformed(DAffine3::from_rotation_z(rad(rot_deg)) * place)
}

fn main() {
	let _ = std::fs::create_dir_all("sweeper_out");
	// plate (outer vertex up) + raised circular grip pad, then bevels, then the
	// 9 finger holes last so each bore runs through plate AND pad in one cut.
	let plate = extrude(&tri_profile(EDGE, 90.0, RC_OUT), T);
	// pad = truncated cone: top radius PAD_R at z=12, wall sloping outward at
	// 60° from horizontal (a palm-friendly skirt, 30° from vertical = printable
	// and self-supporting; base 60.8 at the plate top, 2.3 mm clear of the
	// widest bevel band). Built as a full cone (base 3 mm embedded, no coplanar
	// union seam) whose spike above z=12 is sliced off by a slab cut.
	let slope = 1.0 / rad(60.0).tan();
	let r_base = PAD_R + (PAD_TOP - 3.0) * slope;
	let pad = cone(v(0.0, 0.0, 3.0), DVec3::Z, r_base, r_base / slope, SEG * 2);
	let mut s = union(&plate, &pad);
	s = difference(&s, &cuboid(v(-300.0, -300.0, PAD_TOP), v(300.0, 300.0, PAD_TOP + 150.0)));
	// round over the pad-top circumference edge (top face meets the 60° skirt):
	// a 3 mm-radius arc, approximated by 3 chords (chords, not the true arc, so
	// every cutter surface crosses the part transversally — exact tangency
	// mis-stitches; learned on the first rim-chamfer attempt). Fillet circle
	// center (r 55.57, z 9): tangent to the top plane at r 55.57 and to the
	// skirt at (r 58.17, z 10.5). One revolved ring cutter, one cut. (The exact
	// torus fillet_circular_rim honestly refuses here: cylinder walls only.)
	let (fc_r, fc_z, fr) = (55.34, 8.6, 3.4); // max radius: 0.5 mm shy of the corner-hole mouths
	let mut prof: Vec<DVec2> = vec![DVec2::new(fc_r, PAD_TOP + 0.6)];
	for k in 0..=5 {
		let a = rad(12.0 * k as f64);
		prof.push(DVec2::new(fc_r + fr * a.sin(), fc_z + fr * a.cos()));
	}
	prof.push(DVec2::new(59.44, 8.66)); // last chord extended out past the skirt
	prof.push(DVec2::new(PAD_R + 12.0, 9.54));
	prof.push(DVec2::new(PAD_R + 12.0, PAD_TOP + 0.6));
	s = difference(&s, &revolve(&prof, SEG * 2));
	// NOTE: bevel order matters — ice (steepest) must cut first; water-then-ice
	// mis-stitches at their shared corner (validated non-closed). Ice-first is
	// clean through the whole chain, gated below.
	for (_, lip, run, rot, _) in EDGES {
		s = difference(&s, &bevel_tool(lip, run, rot));
	}
	// finger holes, rims eased on BOTH faces (they press into skin): top rim
	// 1.5 mm at 45° (upward-facing cut, trivially printable); bottom rim 1.2 mm
	// radial over 2.0 mm rise — a 31°-from-vertical cone, safely inside the 45°
	// overhang limit so the part stays support-free (gated below). Cutter cones
	// start past the faces so no cut face is coplanar with pad top or bed.
	for c in grip_holes() {
		let r = FINGER_D / 2.0;
		s = difference(&s, &cylinder(v(c.x, c.y, -5.0), DVec3::Z, r, PAD_TOP + 10.0, SEG));
		let rt = r + 1.5 + 1.0; // 45°: radius at z = PAD_TOP + 1
		s = difference(&s, &cone(v(c.x, c.y, PAD_TOP + 1.0), -DVec3::Z, rt, rt, SEG));
		let rb = r + 1.2 + 0.6; // 1.2/2.0 slope: radius at z = -1
		s = difference(&s, &cone(v(c.x, c.y, -1.0), DVec3::Z, rb, rb * 2.0 / 1.2, SEG));
	}
	// engraved edge labels (upright when their edge is down); gate: the text
	// actually removed material, i.e. all 38 stroke cuts landed
	let vol_pre_text = volume(&s).abs();
	for (word, rot) in [("WATER", 0.0), ("ICE", 120.0), ("MIST", 240.0)] {
		s = engrave_label(s, word, rot);
	}
	let text_mm3 = vol_pre_text - volume(&s).abs();

	// ---- gates -----------------------------------------------------------------
	let val = validate(&s);
	let mesh = tessellate_default(&s);
	let wt = mesh.is_watertight();
	let rep = mesh.support_free_report(Vec3::Z, 45.0, 0.3);
	let vol = volume(&s).abs();
	// each working edge must carry ITS lip: max z within 0.4 mm of the edge line
	let inr = EDGE / (2.0 * 3f64.sqrt());
	let mut lips = Vec::new();
	for (name, lip, _, rot, _) in EDGES {
		let a = rad(270.0 + rot);
		let u = (a.cos(), a.sin());
		let maxz = mesh
			.positions
			.iter()
			.filter(|p| p.x as f64 * u.0 + p.y as f64 * u.1 > inr - 0.4)
			.map(|p| p.z as f64)
			.fold(f64::NEG_INFINITY, f64::max);
		lips.push((name, lip, maxz));
	}
	// bed fit: rotated 15° about Z the triangle sits in its minimal square
	let bbox = |deg: f64| {
		let (s15, c15) = (rad(deg).sin(), rad(deg).cos());
		let (mut lo, mut hi) = (DVec2::splat(f64::INFINITY), DVec2::splat(f64::NEG_INFINITY));
		for p in &mesh.positions {
			let q = DVec2::new(c15 * p.x as f64 - s15 * p.y as f64, s15 * p.x as f64 + c15 * p.y as f64);
			lo = lo.min(q);
			hi = hi.max(q);
		}
		hi - lo
	};
	let (e0, e15) = (bbox(0.0), bbox(15.0));

	// ---- force gates (all three use modes + grip) ------------------------------
	// 1) ICE PUSH: palm force crosses the section through the near corner hole.
	//    Net width = 60 mm engaged palm minus one hole, plate thickness only.
	//    Buckling: the 12 mm pad is 8x stiffer (t³), so the governing strip is
	//    the bare plate between pad rim and bevel start — a few mm, not the
	//    whole ligament.
	//    Strip length = pad rim to the floor-supported working edge (t=T assumed
	//    across it — slightly optimistic over the taper, pessimistic in ignoring
	//    pad and floor rotational restraint).
	let sig_push = F_PUSH_N / ((60.0 - FINGER_D) * T);
	let strip = inr - PAD_R;
	let sig_buckle = PI * PI * TPU_E_MPA * T * T / (12.0 * strip * strip); // Euler strip
	let m_rupture = TPU_ULT_MPA / sig_push;
	let m_buckle = sig_buckle / sig_push;
	// 2) EDGE BENDING, per edge: floor line load at the lip tip, tapered cantilever
	//    t(x) = lip + slope*x; peak bending stress sits at x* = lip/slope. Gated
	//    x>=10 at each edge's WORKING load and x>=5 with the full 200 N shove
	//    misapplied to ANY edge (abuse case).
	let mut bend = Vec::new();
	for (name, lip, run, _, w_work) in EDGES {
		let slope = (T - lip) / run;
		let x = (lip / slope).min(run);
		let t = lip + slope * x;
		let sig_per = 6.0 * x / (t * t); // MPa per (N/mm)
		bend.push((name, sig_per * w_work, TPU_ULT_MPA / (sig_per * w_work), TPU_ULT_MPA / (sig_per * W_ABUSE_N_MM)));
	}
	// 3) GRIP: full pull on one 4-hole row -> net-section tension through the pad
	//    at the row line (pad chord minus the four hole diameters, pad height).
	let row_y = FINGER_PITCH * 3f64.sqrt() / 2.0;
	let net = 2.0 * (PAD_R * PAD_R - row_y * row_y).sqrt() - 4.0 * FINGER_D;
	let sig_grip = F_HANG_N / (net * PAD_TOP);
	let m_grip = TPU_ULT_MPA / sig_grip;
	// 4) mesh-derived: no accidental sliver walls. min_thickness is ~0 by
	//    construction (ray-based; every sharp 90° working-edge corner measures 0),
	//    so the gate is bounded sub-0.8 mm AREA: only the inherent narrow strips
	//    along the three edge corners may register, anything more is a real sliver.
	let walls = mesh.wall_thickness(0.8);
	let force_ok = m_rupture >= 10.0
		&& m_buckle >= 2.0
		&& bend.iter().all(|(_, _, mw, ma)| *mw >= 10.0 && *ma >= 5.0)
		&& m_grip >= 10.0
		&& walls.thin_area <= 2500.0;

	let lips_ok = lips.iter().all(|(_, lip, maxz)| (maxz - lip).abs() <= 0.5);
	let ok = val.is_valid()
		&& wt
		&& rep.steep_area < 1e-6
		&& rep.max_bridge_span <= 12.0
		&& lips_ok
		&& force_ok
		&& (300.0..2000.0).contains(&text_mm3)
		&& (150e3..230e3).contains(&vol)
		&& e15.x <= 256.0
		&& e15.y <= 256.0;

	let _ = std::fs::write("sweeper_out/SWEEPER.stl", mesh.to_stl_binary());
	// BOM v2 flat CSV (same columns as kernel-model format.rs), engine volume
	let bom = format!(
		"name,count,params,part_number,material,density_g_cm3,volume_source,unit_mass_g,line_mass_g,make_or_buy\n\
		 tri_sweep,1,edge={EDGE};t={T},TS-001,TPU95A,{:.3},exact,{m:.1},{m:.1},make\n",
		TPU_G_PER_MM3 * 1000.0,
		m = vol * TPU_G_PER_MM3
	);
	let _ = std::fs::write("sweeper_out/BOM.csv", bom);
	// dossier-style flavor of the same line for tools/assembly_doc.py
	let _ = std::fs::write(
		"sweeper_out/bom_dossier.csv",
		format!(
			"name,kind,qty,material,part_number,grams_per_unit\n\
			 tri_sweep,made,1,TPU95A,TS-001,{:.1}\n",
			vol * TPU_G_PER_MM3
		),
	);
	println!("TRI-SWEEP  {} tris -> sweeper_out/SWEEPER.stl (+BOM.csv)", mesh.indices.len() / 3);
	println!("  valid={} genus={} watertight={} steep={:.3}mm² bridge<={:.1}mm", val.is_valid(), val.genus, wt, rep.steep_area, rep.max_bridge_span);
	println!("  volume={:.0}mm³ ({:.0} g TPU95A)  bbox {:.1}x{:.1}  @15°: {:.1}x{:.1}", vol, vol * TPU_G_PER_MM3, e0.x, e0.y, e15.x, e15.y);
	for (name, lip, maxz) in &lips {
		println!("  edge {name:9} lip {lip:.1}mm  measured max-z at edge {maxz:.2}mm");
	}
	println!("  force: ice-push {F_PUSH_N:.0}N -> palm bar {sig_push:.2} MPa (rupture x{m_rupture:.0}, buckling x{m_buckle:.1} pad+floor restraint ignored)");
	for (name, sigma, mw, ma) in &bend {
		println!("  force: edge {name:9} working bend {sigma:.2} MPa (x{mw:.0} working, x{ma:.0} under full 200N abuse)");
	}
	println!("  force: grip hang {F_HANG_N:.0}N on one hole row -> net section {sig_grip:.2} MPa (x{m_grip:.0});  sub-0.8mm wall area {:.0}mm² (edge-corner strips only);  labels engraved {text_mm3:.0}mm³", walls.thin_area);
	assert!(
		ok,
		"TRI-SWEEP gate FAIL: valid={} wt={} steep={} bridge={} lips={:?} force(rupt x{:.0}, buckle x{:.1}, bend {:?}, grip x{:.0}, minwall {:.2}) vol={} bbox15=({:.1},{:.1})",
		val.is_valid(),
		wt,
		rep.steep_area,
		rep.max_bridge_span,
		lips,
		m_rupture,
		m_buckle,
		bend,
		m_grip,
		walls.min_thickness,
		vol,
		e15.x,
		e15.y
	);
	println!("  ALL GATES OK");
}
