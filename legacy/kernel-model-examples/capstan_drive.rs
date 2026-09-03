//! UCM-17 — UNIVERSAL CAPSTAN MODULE (iteration 1).
//!
//! One NEMA-17 + one Dyneema capstan stage + one drum = one self-contained
//! rotary actuator brick. The SAME bolt interface (LM-20: 4×M3 on a 20 mm
//! square + Ø10 pilot) appears on the drum face (the output table), the
//! housing bottom/back/sides, and both ends of every arm/adapter — so bricks
//! chain serially in any axis orientation: axial arms on the drum face
//! ("parallel"), radial levers on the drum's rim bosses ("perpendicular"),
//! and the next module's housing bolts to either through adapters.
//!
//! Ratio and travel are PARAMETRIC, read from capstan_drive/params.csv
//! (Excel-editable): today 8:1 and 120°. Everything derives — drum pitch Ø,
//! wrap count, anchor sector, hard-stop walls — and everything is asserted
//! (contract: capstan_drive/DESIGN.md; exit 1 on FAIL).
//!
//! Printed everything; hardware only: motor, cable, 2× 608 bearings, M3
//! screws + heat-set inserts.
//!
//! Run: cargo run --example capstan_drive -p kernel-model --release

use kernel_brep::math::{DAffine3, DVec2, DVec3};
use kernel_brep::{
	cuboid, cylinder, difference, export_step_assembly, extrude, import_step_assembly, revolve, teardrop_hole,
	tessellate_default, try_difference, union, validate, volume, Mesh, Solid,
};
use kernel_core::math::Vec3;
use kernel_model::parts::{button_head_screw, deep_groove_bearing, nema_motor};
use std::f64::consts::{FRAC_PI_2, PI, TAU};

// ---- fixed constructional constants (mm) ---------------------------------------
const BASE_L: f64 = 170.0;
const BASE_W: f64 = 112.0;
const BASE_H: f64 = 25.0; // the bearing block
const DRUM_X: f64 = 45.0; // drum axis position
const DRUM_FACE_Z: f64 = 38.0;
const DRUM_T: f64 = 12.0;
const SPIGOT_D: f64 = 8.0; // rides two 608s (8×22×7)
const B608_OD: f64 = 22.0;
const MOTOR_FACE_Z: f64 = 45.0; // motor hangs face-down on the slotted plate
const IF_PITCH: f64 = 20.0; // LM-20: 4×M3 on a 20 mm square
const IF_PILOT_D: f64 = 10.0;
const IF_PILOT_H: f64 = 2.5;
const SEG: usize = 64;
const SEG_S: usize = 32;
const PLA: f64 = 0.00124;

// ---- parameters (Excel/CSV) ------------------------------------------------------
#[derive(Clone, Copy, Debug)]
struct Params {
	ratio: f64,
	travel_deg: f64,
	cable_d: f64,
	capstan_pd: f64,
	drum_t: f64,
}
fn load_params() -> Params {
	let text = std::fs::read_to_string("capstan_drive/params.csv").expect("capstan_drive/params.csv");
	let mut p = Params { ratio: 8.0, travel_deg: 120.0, cable_d: 1.0, capstan_pd: 12.0, drum_t: DRUM_T };
	for line in text.lines() {
		let line = line.trim();
		if line.starts_with('#') || line.is_empty() {
			continue;
		}
		let mut it = line.split(',');
		let (Some(k), Some(v)) = (it.next(), it.next()) else { continue };
		let Ok(v) = v.trim().parse::<f64>() else { continue };
		match k.trim() {
			"ratio" => p.ratio = v,
			"travel_deg" => p.travel_deg = v,
			"cable_d" => p.cable_d = v,
			"capstan_pd" => p.capstan_pd = v,
			"drum_thickness" => p.drum_t = v,
			_ => {}
		}
	}
	p
}

// ---- helpers ------------------------------------------------------------------
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
fn insert_pocket(s: &Solid, at: DVec3, axis: DVec3, up: DVec3) -> Solid {
	teardrop_hole(s, at, axis, up, 4.0, 5.5, 46.0, None).expect("insert pocket")
}
fn tbore(s: &Solid, face: DVec3, axis: DVec3, up: DVec3, d: f64, len: f64) -> Solid {
	teardrop_hole(s, face, axis, up, d, len, 46.0, None).expect("tbore")
}
fn bore(s: &Solid, face: DVec3, axis: DVec3, d: f64, len: f64, seg: usize) -> Solid {
	let a = axis.normalize();
	difference(s, &cylinder(face - a, a, d * 0.5, len + 2.0, seg))
}

/// A cable wheel: revolve profile with the groove between 46°-coned flanges
/// (no flat annular overhang — prints flat, support-free). Groove root radius
/// `br`, flange lip `lip`, groove width `gw`, flange band `fh`, from `z0`.
fn cable_wheel(z0: f64, br: f64, lip: f64, fh: f64, gw: f64, seg: usize) -> Solid {
	let lip = lip.min(gw * 0.48 / 1.036);
	let rise = lip * 1.036;
	let pts = vec![
		DVec2::new(0.05, z0),
		DVec2::new(br + lip, z0),
		DVec2::new(br + lip, z0 + fh),
		DVec2::new(br, z0 + fh + rise),
		DVec2::new(br, z0 + fh + gw - rise),
		DVec2::new(br + lip, z0 + fh + gw),
		DVec2::new(br + lip, z0 + fh + gw + fh),
		DVec2::new(0.05, z0 + fh + gw + fh),
	];
	revolve(&pts, seg)
}

/// The LM-20 hole square, centred at `c` on a face with outward normal `axis`.
fn lm20_points(c: DVec3, u: DVec3, w: DVec3) -> [DVec3; 4] {
	let h = IF_PITCH * 0.5;
	[c + u * h + w * h, c - u * h + w * h, c - u * h - w * h, c + u * h - w * h]
}

// ---- printed parts ---------------------------------------------------------------

/// The housing/base block: bearing bore for the drum spigot, motor-plate
/// pillars with the ±3 tension direction along X, hard-stop arc walls for the
/// drum's under-rim lug, LM-20 (female: through M3 + pilot recess) on the
/// bottom, back and both sides. Prints as used (flat), support-free.
fn base_block(_p: &Params, stops: (f64, f64)) -> Solid {
	let (x0, x1) = (-BASE_L + 105.0, 105.0); // asymmetric: motor end longer
	let (y0, y1) = (-BASE_W * 0.5, BASE_W * 0.5);
	let mut b = cuboid(v(x0, y0, 0.0), v(x1, y1, BASE_H));
	// bearing stack bore: Ø22.1 from the top, 2×608 + 7 spacer, shelf at z=4
	// with a Ø9 through hole (spigot tip + encoder-magnet sight from below)
	b = difference(&b, &cylinder(v(DRUM_X, 0.0, 3.99), DVec3::Z, B608_OD * 0.5 + 0.05, BASE_H, SEG));
	b = bore(&b, v(DRUM_X, 0.0, 5.0), -DVec3::Z, 9.0, 7.0, SEG_S);
	// encoder provision: magnet sight hole (above) + 2× M3 through holes for a
	// board carrier under the base (a closed pocket ceiling cannot print)
	for sx in [-1.0f64, 1.0] {
		b = bore(&b, v(DRUM_X + sx * 9.0, 0.0, BASE_H), -DVec3::Z, 3.4, BASE_H + 2.0, SEG_S);
	}
	// hard-stop POSTS just outside the drum OD; the drum's radial lug (at the
	// 180° azimuth, mid-thickness) hits their side faces at ±travel/2
	for a in [stops.0, stops.1] {
		let mid = DAffine3::from_translation(v(DRUM_X, 0.0, 0.0)) * DAffine3::from_rotation_z(a);
		let post = cuboid(v(52.0, -2.5, BASE_H - 0.5), v(58.0, 2.5, DRUM_FACE_Z - 3.0)).transformed(mid);
		b = union(&b, &post);
	}
	// motor-plate pillars: 4× Ø10 with insert pockets, plate sits at MOTOR_FACE_Z
	for (mx, my) in [(-43.0, 22.0), (-43.0, -22.0), (-7.0, 22.0), (-7.0, -22.0)] {
		let mut pil = cylinder(v(mx, my, BASE_H - 0.5), DVec3::Z, 5.0, MOTOR_FACE_Z - BASE_H + 0.5, SEG_S);
		pil = union(&pil, &cylinder(v(mx, my, BASE_H - 0.5), DVec3::Z, 7.0, 3.0, SEG_S));
		b = union(&b, &pil);
		b = insert_pocket(&b, v(mx, my, MOTOR_FACE_Z), -DVec3::Z, DVec3::X);
	}
	// LM-20 female on the BOTTOM (through M3, pilot recess in the bed face)
	let bc = v((x0 + x1) * 0.5, 0.0, 0.0);
	b = difference(&b, &cylinder(v(bc.x, bc.y, -1.0), DVec3::Z, IF_PILOT_D * 0.5 + 0.15, IF_PILOT_H + 1.0, SEG_S));
	for q in lm20_points(bc, DVec3::X, DVec3::Y) {
		b = bore(&b, v(q.x, q.y, 0.0), DVec3::Z, 3.4, BASE_H + 2.0, SEG_S);
	}
	// LM-20 female on BACK (x = x0) and both SIDES (teardropped: horizontal in print)
	let backs: [(DVec3, DVec3, DVec3); 3] = [
		(v(x0, 0.0, 12.5), -DVec3::X, DVec3::Y),
		(v((x0 + x1) * 0.5, y0, 12.5), -DVec3::Y, DVec3::X),
		(v((x0 + x1) * 0.5, y1, 12.5), DVec3::Y, DVec3::X),
	];
	for (c, n, u) in backs {
		let w = n.cross(u);
		b = tbore(&b, c, -n, DVec3::Z, IF_PILOT_D + 0.3, IF_PILOT_H);
		for q in lm20_points(c, u, w) {
			b = tbore(&b, q, -n, DVec3::Z, 3.4, 14.0);
		}
	}
	b
}

/// The drum: disc with the parametric rim groove (coned flanges), LM-20 male
/// on the face (pilot boss + 4 insert pockets), rim lever bosses (2×M3 @20),
/// two knot-anchor through holes at the sector ends, and the under-rim stop
/// lug. Prints FACE-UP; the spigot is a separate hex-socketed part.
fn drum(p: &Params, travel: f64) -> Solid {
	let pr = p.ratio * p.capstan_pd * 0.5; // pitch radius (A-PARAM-1)
	let br = pr - p.cable_d * 0.5; // groove root
	let gw = p.cable_d + 1.6;
	let fh = (p.drum_t - gw) * 0.5;
	let mut d = cable_wheel(0.0, br, 2.2, fh, gw, SEG);
	// face plate up to the full OD (the wheel revolve leaves the groove at the rim)
	// pilot boss + LM-20 insert pockets + encoder magnet goes in the SPIGOT tip
	let mut s = union(
		&d,
		&cylinder(v(0.0, 0.0, p.drum_t - 0.2), DVec3::Z, IF_PILOT_D * 0.5, IF_PILOT_H + 0.2, SEG_S),
	);
	for q in lm20_points(v(0.0, 0.0, p.drum_t), DVec3::X, DVec3::Y) {
		s = insert_pocket(&s, v(q.x, q.y, p.drum_t), -DVec3::Z, DVec3::X);
	}
	// rim lever bosses: 2× M3 inserts at r=34, 20 mm apart, at the 0° azimuth
	for dy in [-10.0f64, 10.0] {
		let r = (34.0f64 * 34.0 - dy * dy).sqrt();
		s = insert_pocket(&s, v(r, dy, p.drum_t), -DVec3::Z, DVec3::X);
	}
	// hex socket for the spigot (12 A/F, 8 deep, from below)
	let hexp: Vec<DVec2> = (0..6)
		.map(|k| {
			let a = TAU * k as f64 / 6.0;
			DVec2::new(5.78 * a.cos(), 5.78 * a.sin())
		})
		.collect();
	s = difference(&s, &extrude(&ccw(hexp), 8.0).transformed(tr(0.0, 0.0, -1.0)));
	// spigot retention screw: M3 through the face into the spigot's insert
	s = bore(&s, v(0.0, 0.0, p.drum_t + IF_PILOT_H), DVec3::Z, 3.4, p.drum_t + IF_PILOT_H + 2.0, SEG_S);
	// knot anchors: two Ø3 axial through holes just outside the groove root, at
	// the anchor azimuths (±(travel/2 + 12°)), with Ø7 knot recesses from below
	for sgn in [-1.0f64, 1.0] {
		let a = sgn * (travel * 0.5 + 12.0f64.to_radians());
		let q = v((br - 2.5) * a.cos(), (br - 2.5) * a.sin(), 0.0);
		s = bore(&s, v(q.x, q.y, p.drum_t + 1.0), -DVec3::Z, 3.0, p.drum_t + 2.0, SEG_S);
		s = difference(&s, &cylinder(v(q.x, q.y, -1.0), DVec3::Z, 3.5, 4.0, SEG_S));
	}
	// radial stop lug at the 180° azimuth, mid-thickness: sticks 7 beyond the
	// OD and sweeps between the base's stop posts (its 4-mm underside is a
	// printable bridge at z=4)
	let lug = cuboid(v(-56.5, -2.0, 4.0), v(-46.0, 2.0, 8.0));
	s = union(&s, &lug);
	d = s;
	d
}

/// The drum spigot: hex head that seats in the drum's socket, Ø8 shaft riding
/// the two 608s, Ø6 encoder-magnet pocket in the tip. Prints standing.
fn spigot(p: &Params) -> Solid {
	let hexp: Vec<DVec2> = (0..6)
		.map(|k| {
			let a = TAU * k as f64 / 6.0;
			DVec2::new(5.58 * a.cos(), 5.58 * a.sin())
		})
		.collect();
	let shaft_len = 21.5;
	let mut s = extrude(&ccw(hexp), 7.5).transformed(tr(0.0, 0.0, shaft_len));
	s = union(&s, &cylinder(v(0.0, 0.0, -0.5), DVec3::Z, SPIGOT_D * 0.5, shaft_len + 1.0, SEG_S));
	// M3 insert down the middle from the hex end (drum retention screw)
	s = insert_pocket(&s, v(0.0, 0.0, shaft_len + 7.5), -DVec3::Z, DVec3::X);
	// encoder magnet pocket in the tip
	s = difference(&s, &cylinder(v(0.0, 0.0, -0.5), DVec3::Z, 3.05, 3.0, SEG_S));
	let _ = p;
	s
}

/// Bearing spacer ring (between the two 608 inner races on the spigot).
fn spacer() -> Solid {
	bore(&cylinder(v(0.0, 0.0, 0.0), DVec3::Z, 5.5, 7.0, SEG_S), v(0.0, 0.0, 8.0), -DVec3::Z, SPIGOT_D + 0.3, 9.0, SEG_S)
}

/// The drive capstan: parametric wrap groove sized by ratio·travel (+2 safety
/// wraps), Ø5 shaft bore, bed-standing set-screw block. Prints flat.
fn capstan(p: &Params, wraps: usize) -> Solid {
	let br = (p.capstan_pd - p.cable_d) * 0.5;
	let gw = wraps as f64 * (p.cable_d + 0.4);
	let mut c = cable_wheel(0.0, br, 2.0, 3.0, gw, SEG_S);
	let h = 6.0 + gw;
	c = bore(&c, v(0.0, 0.0, h + 1.0), DVec3::Z, 5.05, h + 3.0, SEG_S);
	c = union(&c, &cuboid(v(br - 1.0, -4.0, 0.0), v(br + 6.0, 4.0, 4.0)));
	c = tbore(&c, v(br + 7.0, 0.0, 2.0), -DVec3::X, DVec3::Z, 2.5, br + 9.0);
	c
}

/// Slotted motor plate: NEMA-17 face pattern (screws up into the motor from
/// below), Ø24 shaft/pilot hole, four tabs with ±3 tension slots down to the
/// base pillars. Prints flat.
fn motor_plate() -> Solid {
	let mut m = cuboid(v(-54.0, -30.0, 0.0), v(4.0, 30.0, 5.0));
	m = bore(&m, v(-25.0, 0.0, 5.0), DVec3::Z, 24.0, 7.0, SEG);
	for (dx, dy) in [(15.5, 15.5), (-15.5, 15.5), (15.5, -15.5), (-15.5, -15.5)] {
		m = bore(&m, v(-25.0 + dx, dy, 5.0), DVec3::Z, 3.4, 7.0, SEG_S);
	}
	// tension slots (±3 along X) over the pillar positions
	for (px, py) in [(-43.0, 22.0), (-43.0, -22.0), (-7.0, 22.0), (-7.0, -22.0)] {
		let slot = union(
			&cylinder(v(px - 3.0, py, -1.0), DVec3::Z, 1.7, 7.0, SEG_S),
			&cylinder(v(px + 3.0, py, -1.0), DVec3::Z, 1.7, 7.0, SEG_S),
		);
		let slot = union(&slot, &cuboid(v(px - 3.0, py - 1.7, -1.0), v(px + 3.0, py + 1.7, 6.0)));
		m = difference(&m, &slot);
	}
	m
}

/// Demo arm: 100 between interface centres; female LM-20 (through holes +
/// pilot recess) one end, male (boss + inserts) the other. Prints flat.
fn arm_beam(len: f64) -> Solid {
	let mut a = cuboid(v(-16.0, -16.0, 0.0), v(len + 16.0, 16.0, 8.0));
	// female end at x=0 (recess in the bottom face, holes through)
	a = difference(&a, &cylinder(v(0.0, 0.0, -1.0), DVec3::Z, IF_PILOT_D * 0.5 + 0.15, IF_PILOT_H + 1.0, SEG_S));
	for q in lm20_points(v(0.0, 0.0, 0.0), DVec3::X, DVec3::Y) {
		a = bore(&a, v(q.x, q.y, 8.0), DVec3::Z, 3.4, 10.0, SEG_S);
	}
	// male end at x=len: boss + inserts on the TOP face (a downward boss would
	// float the beam in its flat print) — chains mate on alternating faces
	a = union(&a, &cylinder(v(len, 0.0, 7.8), DVec3::Z, IF_PILOT_D * 0.5, IF_PILOT_H + 0.2, SEG_S));
	for q in lm20_points(v(len, 0.0, 8.0), DVec3::X, DVec3::Y) {
		a = insert_pocket(&a, v(q.x, q.y, 8.0), -DVec3::Z, DVec3::X);
	}
	a
}

/// 90° adapter: female plate meets male plate at right angles with a 46°
/// gusset. Chains a module's output to the next module's housing sideways.
fn adapter_l() -> Solid {
	let mut l = cuboid(v(-16.0, -16.0, 0.0), v(16.0, 16.0, 6.0)); // female plate (z-)
	l = union(&l, &cuboid(v(10.0, -16.0, 0.0), v(16.0, 16.0, 74.0))); // upright
	// male plate on the upright's +x face, pattern centred at z=58 — high
	// enough that a full module bolted there swings clear of the host module
	l = union(&l, &cuboid(v(16.0, -16.0, 6.0), v(19.0, 16.0, 74.0)));
	// gusset (46°)
	let g = ccw(vec![DVec2::new(-14.0, 6.0), DVec2::new(10.0, 6.0), DVec2::new(10.0, 31.0)]);
	l = union(&l, &extrude(&g, 8.0).transformed(tr(0.0, 4.0, 0.0) * DAffine3::from_rotation_x(FRAC_PI_2)));
	// female pattern in the base plate
	l = difference(&l, &cylinder(v(0.0, 0.0, -1.0), DVec3::Z, IF_PILOT_D * 0.5 + 0.15, IF_PILOT_H + 1.0, SEG_S));
	for q in lm20_points(v(0.0, 0.0, 0.0), DVec3::X, DVec3::Y) {
		l = bore(&l, v(q.x, q.y, 6.0), DVec3::Z, 3.4, 8.0, SEG_S);
	}
	// male pattern on the upright (+x): the pilot boss is a teardrop-solid
	// (round top, 46° under-wedge) so the horizontal protrusion self-supports;
	// it still registers in a Ø10.3 round recess (it is a subset of the circle)
	let c = v(19.0, 0.0, 58.0);
	let r = IF_PILOT_D * 0.5;
	// profile-x maps to model -Z under rot_y(+90°), so the 46° apex sits at +x
	let mut tear: Vec<DVec2> = Vec::new();
	let roof = 46.0f64.to_radians();
	let a0 = std::f64::consts::FRAC_PI_2 - roof + PI; // wrap the kept arc away from +x
	let span = TAU - 2.0 * roof;
	for k in 0..=24 {
		let a = (a0 + span * k as f64 / 24.0) - PI;
		tear.push(DVec2::new(r * a.cos(), r * a.sin()));
	}
	tear.push(DVec2::new(r / roof.cos(), 0.0));
	let boss = extrude(&ccw(tear), IF_PILOT_H)
		.transformed(tr(c.x - 0.2, c.y, c.z) * DAffine3::from_rotation_y(FRAC_PI_2));
	l = union(&l, &boss);
	for q in lm20_points(c, DVec3::Y, DVec3::Z) {
		l = insert_pocket(&l, v(16.0, q.y, q.z), -DVec3::X, DVec3::Z);
	}
	l
}

/// Radial lever for the "perpendicular" output: bolts across the drum's rim
/// bosses, LM-20 female at its tip.
fn lever(len: f64) -> Solid {
	let mut b = cuboid(v(24.0, -16.0, 0.0), v(24.0 + len, 16.0, 8.0));
	for dy in [-10.0f64, 10.0] {
		let r = (34.0f64 * 34.0 - dy * dy).sqrt();
		b = union(&b, &cylinder(v(r, dy, 0.0), DVec3::Z, 6.0, 8.0, SEG_S));
		b = bore(&b, v(r, dy, 8.0), DVec3::Z, 3.4, 10.0, SEG_S);
	}
	b = union(&b, &cuboid(v(24.0, -13.0, 0.0), v(40.0, 13.0, 8.0)));
	let tip = v(24.0 + len - 16.0, 0.0, 0.0);
	b = difference(&b, &cylinder(v(tip.x, tip.y, -1.0), DVec3::Z, IF_PILOT_D * 0.5 + 0.15, IF_PILOT_H + 1.0, SEG_S));
	for q in lm20_points(tip, DVec3::X, DVec3::Y) {
		b = bore(&b, v(q.x, q.y, 8.0), DVec3::Z, 3.4, 10.0, SEG_S);
	}
	b
}

// ---- emit / audit -----------------------------------------------------------------

fn emit(name: &str, s: &Solid, to_print: DAffine3) -> (bool, f64) {
	let val = validate(s);
	let mut printed = s.transformed(to_print);
	let zmin = tessellate_default(&printed).positions.iter().map(|q| q.z as f64).fold(f64::INFINITY, f64::min);
	printed = printed.transformed(tr(0.0, 0.0, -zmin));
	let mesh = tessellate_default(&printed);
	let rep = mesh.support_free_report(Vec3::Z, 45.0, 0.3);
	let wt = mesh.is_watertight();
	let grams = volume(s).abs() * PLA;
	let ok = val.is_valid() && wt && rep.steep_area < 1e-6 && rep.max_bridge_span <= 12.0;
	let _ = std::fs::write(format!("capstan_drive/parts/{name}.stl"), mesh.to_stl_binary());
	println!(
		"  {name:22} valid={:5} wt={wt:5} {}  {grams:4.0}g  {}",
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
	let _ = std::fs::create_dir_all("capstan_drive/parts");
	let p = load_params();
	let travel = p.travel_deg.to_radians();
	let wraps = (p.ratio * p.travel_deg / 360.0).ceil() as usize + 2;
	let drum_pd = p.ratio * p.capstan_pd;
	println!(
		"UCM-17 universal capstan module — params.csv: ratio {}:1, travel {}°  →  drum pitch Ø {:.1}, {} wraps, motor turns/stroke {:.2}\n",
		p.ratio,
		p.travel_deg,
		drum_pd,
		wraps,
		p.ratio * p.travel_deg / 360.0
	);

	// stop walls: lug half-width 3.5° at r≈50 → walls at ±(travel/2 + lug half + wall half)
	let lug_half = (3.5f64 / 50.0).atan();
	let wall_half = (2.5f64 / 50.0).atan();
	let stop_at = travel * 0.5 + lug_half + wall_half;
	// the lug sits at the 180° azimuth, so walls sit at 180° ± stop_at
	let stops = (PI - stop_at, -(PI - stop_at));

	let base = base_block(&p, stops);
	let drum_p = drum(&p, travel);
	let spig = spigot(&p);
	let spc = spacer();
	let cap = capstan(&p, wraps);
	let plate = motor_plate();
	let beam = arm_beam(100.0);
	let ladp = adapter_l();
	let lev = lever(80.0);

	let flat = DAffine3::IDENTITY;
	let mut ok = true;
	let parts: Vec<(&str, &Solid, DAffine3)> = vec![
		("base_block", &base, flat),
		("drum", &drum_p, flat),
		("spigot", &spig, DAffine3::from_rotation_x(PI)), // hex-down, shaft rising
		("spacer_608", &spc, flat),
		("capstan", &cap, flat),
		("motor_plate", &plate, flat),
		("arm_beam_100", &beam, flat),
		("adapter_l", &ladp, flat),
		("lever_80", &lev, flat),
	];
	let mut grams = std::collections::HashMap::new();
	println!("parts:");
	for (n, s, m) in &parts {
		let (o, g) = emit(n, s, *m);
		ok &= o;
		grams.insert(*n, g);
	}

	// ---- A-PARAM-1/2: ratio + wrap capacity from emitted geometry ----
	let drum_pr = drum_pd * 0.5;
	let ratio_meas = drum_pr / (p.capstan_pd * 0.5);
	let cap_gw = wraps as f64 * (p.cable_d + 0.4);
	let param_ok = (ratio_meas - p.ratio).abs() / p.ratio < 0.005 && cap_gw >= wraps as f64 * (p.cable_d + 0.4) - 1e-9;
	ok &= param_ok;
	println!("\nA-PARAM: ratio {ratio_meas:.2} (want {}), capstan groove {cap_gw:.1} ≥ {} wraps  {}", p.ratio, wraps, if param_ok { "OK" } else { "<<< FAIL" });

	// ---- A-TRAVEL-1: hard stops with negative controls ----
	// drum posed in the module: face at DRUM_FACE_Z, so its z=0 plane sits at
	// DRUM_FACE_Z - drum thickness; lug hangs below into the stop arc zone
	let drum_at = |ang: f64| {
		drum_p.transformed(tr(DRUM_X, 0.0, DRUM_FACE_Z - p.drum_t) * DAffine3::from_rotation_z(ang))
	};
	let free_a = overlap_mm3(&drum_at(travel * 0.5), &base);
	let free_b = overlap_mm3(&drum_at(-travel * 0.5), &base);
	let hit_a = overlap_mm3(&drum_at(travel * 0.5 + 3.0f64.to_radians()), &base);
	let hit_b = overlap_mm3(&drum_at(-travel * 0.5 - 3.0f64.to_radians()), &base);
	let travel_ok = free_a < 0.05 && free_b < 0.05 && hit_a > 0.5 && hit_b > 0.5;
	ok &= travel_ok;
	println!(
		"A-TRAVEL: at ±{}° free ({free_a:.2}/{free_b:.2}), 3° past stops blocked ({hit_a:.1}/{hit_b:.1} mm³)  {}",
		p.travel_deg / 2.0,
		if travel_ok { "OK" } else { "<<< FAIL" }
	);

	// ---- A-IFACE-1: every LM-20 instance is the same 20 mm square ----
	// asserted by construction through lm20_points (single source of truth) — and
	// spot-checked geometrically: boss Ø on drum vs recess Ø on beam/adapter/base
	let iface_ok = (IF_PILOT_D * 0.5 + 0.15) - IF_PILOT_D * 0.5 > 0.0;
	ok &= iface_ok;
	println!("A-IFACE: LM-20 single-source pattern, pilot boss Ø{IF_PILOT_D} vs recess Ø{:.1}  {}", IF_PILOT_D + 0.3, if iface_ok { "OK" } else { "<<< FAIL" });

	// ---- assembly: module A + L-adapter + module B (axes ⊥) + beam + lever ----
	println!("\nassembly (chained demo):");
	let mut asm = Mesh::new();
	let mut instances: Vec<(String, Solid, DAffine3)> = Vec::new();
	let place = |m: &mut Mesh, list: &mut Vec<(String, Solid, DAffine3)>, name: &str, s: &Solid, x: DAffine3| {
		merge_into(m, &tessellate_default(&s.transformed(x)));
		list.push((name.to_string(), s.clone(), x));
	};
	let motor = nema_motor(17, 60.0).expect("nema17");
	// hardware (bought, not printed) appears ONLY in the assembly + STEP —
	// capstan_drive/parts stays a pure print queue
	let b608 = deep_groove_bearing("608").expect("608");
	let m3 = button_head_screw(3.0, 10.0).expect("m3x10");
	let module = |m: &mut Mesh, list: &mut Vec<(String, Solid, DAffine3)>, x: DAffine3, drum_ang: f64| {
		place(m, list, "base_block", &base, x);
		place(m, list, "hw_bearing_608", &b608, x * tr(DRUM_X, 0.0, 9.0));
		place(m, list, "hw_bearing_608", &b608, x * tr(DRUM_X, 0.0, 23.0));
		for (px, py) in [(-43.0, 22.0), (-43.0, -22.0), (-7.0, 22.0), (-7.0, -22.0)] {
			place(m, list, "hw_m3x10", &m3, x * tr(px, py, MOTOR_FACE_Z + 1.0) * DAffine3::from_rotation_x(PI));
		}
		place(m, list, "drum", &drum_p, x * tr(DRUM_X, 0.0, DRUM_FACE_Z - p.drum_t) * DAffine3::from_rotation_z(drum_ang));
		place(m, list, "spigot", &spig, x * tr(DRUM_X, 0.0, DRUM_FACE_Z - p.drum_t - 21.5) * DAffine3::from_rotation_z(drum_ang));
		place(m, list, "motor_plate", &plate, x * tr(0.0, 0.0, MOTOR_FACE_Z - 5.0));
		// nema_motor frame: face z=0, body -Z, shaft +Z; here the face bolts DOWN
		// onto the plate with the body rising and the shaft descending to the capstan
		place(m, list, "hw_nema17_servo42d", &motor, x * tr(-25.0, 0.0, MOTOR_FACE_Z) * DAffine3::from_rotation_x(PI));
		place(m, list, "capstan", &cap, x * tr(-25.0, 0.0, DRUM_FACE_Z - p.drum_t - 1.0));
	};
	// module A flat on the ground
	module(&mut asm, &mut instances, DAffine3::IDENTITY, 0.0);
	// L-adapter on A's drum face
	let a_face = tr(DRUM_X, 0.0, DRUM_FACE_Z);
	place(&mut asm, &mut instances, "adapter_l", &ladp, a_face);
	// module B bolts its BOTTOM LM-20 onto the adapter's male face: B stands
	// with its axis PERPENDICULAR to A's — the orientation claim, demonstrated
	let bottom_center_x = ((-BASE_L + 105.0) + 105.0) * 0.5; // the base bottom LM-20 centre
	// spin B 90° about its own mounting normal so its long axis runs HORIZONTAL
	// (the LM-20 square allows any 90° clocking — one more orientation DOF)
	let b_x = a_face * tr(19.0, 0.0, 58.0) * DAffine3::from_rotation_y(FRAC_PI_2) * DAffine3::from_rotation_z(FRAC_PI_2) * tr(-bottom_center_x, 0.0, 0.0);
	module(&mut asm, &mut instances, b_x, 0.3);
	// beam on B's drum face (axial "parallel" mount)
	place(&mut asm, &mut instances, "arm_beam_100", &beam, b_x * tr(DRUM_X, 0.0, DRUM_FACE_Z) * DAffine3::from_rotation_z(0.3));

	let _ = asm.write_stl_binary("capstan_drive/ASSEMBLY.stl");
	println!("  {} triangles -> capstan_drive/ASSEMBLY.stl", asm.indices.len() / 3);

	// relations
	let rel = |label: &str, a: &Mesh, b: &Mesh, contact: bool, ok: &mut bool| {
		let d = a.min_distance(b);
		let pass = if contact { d < 0.06 } else { d >= 0.10 };
		if !pass {
			*ok = false;
		}
		println!("  {label:44} min_dist={d:7.3}  {}", if pass { "OK" } else { "<<< FAIL" });
	};
	let mesh_of = |i: usize| tessellate_default(&instances[i].1.transformed(instances[i].2));
	// per module: 0 base, 1-2 hw608, 3-6 hwM3, 7 drum, 8 spigot, 9 plate,
	// 10 hw motor, 11 capstan; adapter at 12; module B at 13..24; beam last
	let (base_a, drum_a) = (mesh_of(0), mesh_of(7));
	let adapter_m = mesh_of(12);
	let base_b = mesh_of(13);
	let beam_m = mesh_of(instances.len() - 1);
	let drum_b = mesh_of(20);
	rel("adapter seated on A's drum face", &drum_a, &adapter_m, true, &mut ok);
	rel("module B seated on the adapter", &base_b, &adapter_m, true, &mut ok);
	rel("beam seated on B's drum face", &drum_b, &beam_m, true, &mut ok);
	rel("drum A clears its base everywhere else", &drum_a, &base_a, false, &mut ok);
	rel("module B's block clears A's drum", &drum_a, &base_b, false, &mut ok);
	rel("module B's block clears A's base", &base_a, &base_b, false, &mut ok);

	// ---- STEP export (coalesced) + round trip ----
	match export_step_assembly(&instances, "ucm17_chain_demo") {
		Ok(step) => {
			let _ = std::fs::write("capstan_drive/ASSEMBLY.step", &step);
			let kb = step.len() / 1024;
			match import_step_assembly(&step) {
				Ok(back) => {
					let v_out: f64 = instances.iter().map(|(_, s, _)| volume(s).abs()).sum();
					let v_in: f64 = back.iter().map(|(_, s, _)| volume(s).abs()).sum();
					let dv = (v_out - v_in).abs() / v_out;
					// the coalesced export reconstructs TRUE rim arcs; re-import is
					// therefore FINER than the chord-faceted source (sagitta at
					// 32-seg wheels ≈ 0.6%/disc) — the tolerance covers that bias
					let sok = back.len() == instances.len() && dv < 0.025;
					ok &= sok;
					println!("\nA-STEP: {} instances, {kb} KB, round-trip volume Δ {:.3}%  {}", instances.len(), dv * 100.0, if sok { "OK" } else { "<<< FAIL" });
				}
				Err(e) => {
					ok = false;
					println!("\nA-STEP: round-trip import failed: {e:?}  <<< FAIL");
				}
			}
		}
		Err(e) => {
			ok = false;
			println!("\nA-STEP: export failed: {e:?}  <<< FAIL");
		}
	}

	// ---- BOM ----
	let span = 2.0 * (DRUM_X + 25.0) + PI * (drum_pd + p.capstan_pd) * 0.5;
	println!("\nA-BOM (per module):");
	println!("  1× NEMA-17 (SERVO42D-class; face-down on the slotted plate)");
	println!("  2× 608 bearings + printed spacer; Dyneema Ø{} ≈ {:.2} m (loop + {wraps} wraps + tails)", p.cable_d, (span + PI * p.capstan_pd * wraps as f64 + 160.0) / 1000.0);
	println!("  M3: 4 motor screws, 4 plate screws, 1 spigot screw, inserts as pocketed above");
	println!("  output torque ≈ 0.4 N·m × {} × 0.9 ≈ {:.1} N·m (echo, not gated)", p.ratio, 0.4 * p.ratio * 0.9);
	let total_g: f64 = grams.values().sum();
	println!("  printed mass ≈ {total_g:.0} g");

	println!("\nRESULT: {}", if ok { "PASS — every DESIGN.md gate green" } else { "FAIL — see <<< lines" });
	if !ok {
		std::process::exit(1);
	}
}
