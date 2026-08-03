//! DOVESTACK — a modular dovetail drawer system (Printables Designer Challenge, July 2026).
//!
//! One universal joint does everything: a dovetail SOCKET TRACK (6.0 opening /
//! 9.0 root / 2.5 deep, full-depth groove) on every exterior face of every
//! module. Anything with the matching male profile — a connector key, a wall-dock
//! rail, a foot rail, a hook clip — engages any track: stacking, ganging,
//! wall/pegboard/Skådis mounting, desk mounting, feet and future add-ons are all
//! the SAME interface. Every part prints support-free in its stated orientation,
//! and the kernel's `support_free_report` gate (steep_area == 0) is asserted for
//! every part — plus a wrong-orientation negative control proving the gate bites.
//!
//! Contract: drawer_system/DESIGN.md (every line asserted here; exit 1 on FAIL).
//! Run: cargo run --example drawer_system -p kernel-model --release -> drawer_system/

use kernel_brep::math::{DAffine3, DMat3, DVec2, DVec3};
use kernel_brep::{
	cone, cuboid, cylinder, difference, extrude, teardrop_hole, tessellate_default, try_intersection, union, validate,
	volume, Mesh, Solid,
};
use kernel_core::math::Vec3;
use std::f64::consts::FRAC_PI_2;

// ---- the module grid (mm) ----------------------------------------------------
const WU: f64 = 80.0; // unit exterior width
const HU: f64 = 50.0; // unit exterior height
const D: f64 = 120.0; // module depth (all modules)
const SW: f64 = 4.0; // shell side/top/bottom wall (2.5-deep sockets leave 1.5)
const BW: f64 = 3.0; // shell back wall
const CH: f64 = 1.5; // chamfer on the four long exterior edges

// ---- the one dovetail interface ----------------------------------------------
const SOCK_OPEN: f64 = 3.0; // socket half-width at the face
const SOCK_ROOT: f64 = 4.5; // socket half-width at the root
const SOCK_DEPTH: f64 = 2.5; // socket depth into the wall
const SOCK_OFF: f64 = 20.0; // track offset from unit centre (top/bottom faces)
const KEY_WAIST: f64 = 2.8; // male half-width at the mating plane (0.2/flank clr)
const KEY_ROOT: f64 = 4.21; // male half-width at full engagement
const KEY_ENG: f64 = 2.35; // male engagement depth (0.15 axial clearance)
const KEY_LEN: f64 = 119.0; // full-channel symmetric spline (channel is 120)

// ---- drawer fit ----------------------------------------------------------------
const SIDE_CLR: f64 = 0.35; // drawer body to shell interior, per side
const RAIL_H: f64 = 0.8; // shell floor runner rail height
const TOP_CLR: f64 = 4.5; // drawer body top to shell interior ceiling
const D_WALL: f64 = 2.0; // drawer wall
const D_BACK: f64 = 3.2; // drawer back wall (magnet pocket 2.4 leaves 0.8)
const D_FLOOR: f64 = 2.4;
const BODY_BACK: f64 = 115.8; // drawer body back plane (1.2 to shell interior back)
const PANEL_T: f64 = 3.2; // front panel thickness (face at y = -PANEL_T)
const LUG_OV: f64 = 0.5; // detent lug/rib vertical overlap

const SEG: usize = 32;
const PLA_G_PER_MM3: f64 = 0.00124;

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

/// Prism from a profile in the model (x,z) plane, spanning y ∈ [y0, y1]
/// (used for anything running along the module depth — shells, rails, dock rails).
fn prism_y(profile: &[(f64, f64)], y0: f64, y1: f64) -> Solid {
	let p: Vec<DVec2> = profile.iter().map(|&(x, z)| DVec2::new(x, z)).collect();
	// extrude is along +Z; rot_x(+90°) sends (x, h, d) -> (x, -d, h); shift +y1.
	extrude(&ccw(p), y1 - y0).transformed(tr(0.0, y1, 0.0) * DAffine3::from_rotation_x(FRAC_PI_2))
}

/// Prism from a profile in the model (y,z) plane, spanning x ∈ [x0, x0+len]
/// (ribs, lips, lugs, label rails, hook blades — anything running across width).
fn prism_x(profile: &[(f64, f64)], x0: f64, len: f64) -> Solid {
	let p: Vec<DVec2> = profile.iter().map(|&(y, z)| DVec2::new(-z, y)).collect();
	// rot_y(+90°) sends (px, py, d) -> (d, py, -px): profile (-z, y) lands on (y, z).
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

// ---- shells --------------------------------------------------------------------

/// The shell face profile: outer rectangle with chamfered corners and dovetail
/// socket notches on all four edges. Full-depth extrusion makes every socket an
/// exact planar prism — no booleans involved in the joint geometry at all.
fn shell_profile(uw: usize, uh: usize) -> Vec<DVec2> {
	let (w, h) = (uw as f64 * WU, uh as f64 * HU);
	let (hw, s0, s1, dp) = (w * 0.5, SOCK_OPEN, SOCK_ROOT, SOCK_DEPTH);
	let mut xs: Vec<f64> = Vec::new(); // top/bottom track centres
	for i in 0..uw {
		let cx = -hw + WU * 0.5 + WU * i as f64;
		xs.push(cx - SOCK_OFF);
		xs.push(cx + SOCK_OFF);
	}
	xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
	let zs: Vec<f64> = (0..uh).map(|j| HU * 0.5 + HU * j as f64).collect(); // side tracks

	let mut p: Vec<DVec2> = Vec::new();
	// bottom edge, left -> right
	p.push(DVec2::new(-hw + CH, 0.0));
	for &cx in &xs {
		p.push(DVec2::new(cx - s0, 0.0));
		p.push(DVec2::new(cx - s1, dp));
		p.push(DVec2::new(cx + s1, dp));
		p.push(DVec2::new(cx + s0, 0.0));
	}
	p.push(DVec2::new(hw - CH, 0.0));
	p.push(DVec2::new(hw, CH));
	// right edge, bottom -> top
	for &cz in &zs {
		p.push(DVec2::new(hw, cz - s0));
		p.push(DVec2::new(hw - dp, cz - s1));
		p.push(DVec2::new(hw - dp, cz + s1));
		p.push(DVec2::new(hw, cz + s0));
	}
	p.push(DVec2::new(hw, h - CH));
	p.push(DVec2::new(hw - CH, h));
	// top edge, right -> left
	for &cx in xs.iter().rev() {
		p.push(DVec2::new(cx + s0, h));
		p.push(DVec2::new(cx + s1, h - dp));
		p.push(DVec2::new(cx - s1, h - dp));
		p.push(DVec2::new(cx - s0, h));
	}
	p.push(DVec2::new(-hw + CH, h));
	p.push(DVec2::new(-hw, h - CH));
	// left edge, top -> bottom
	for &cz in zs.iter().rev() {
		p.push(DVec2::new(-hw, cz + s0));
		p.push(DVec2::new(-hw + dp, cz + s1));
		p.push(DVec2::new(-hw + dp, cz - s1));
		p.push(DVec2::new(-hw, cz - s0));
	}
	p.push(DVec2::new(-hw, CH));
	p
}

/// A module shell, `uw` units wide × `uh` units tall: profile extrusion, interior
/// cavity, floor runner rails, detent ceiling rib, magnet pockets, snap recesses.
/// Prints back-face-down (the whole cavity and every socket rises vertically).
fn shell(uw: usize, uh: usize) -> Solid {
	let (w, h) = (uw as f64 * WU, uh as f64 * HU);
	let (iw, ih) = (w - 2.0 * SW, h - 2.0 * SW);
	let ceil = SW + ih;
	let mut s = prism_y(
		&shell_profile(uw, uh).iter().map(|p| (p.x, p.y)).collect::<Vec<_>>(),
		0.0,
		D,
	);
	// interior cavity, cut from the front, leaving the back wall
	s = difference(&s, &cuboid(v(-iw * 0.5, -1.0, SW), v(iw * 0.5, D - BW, ceil)));
	for i in 0..uw {
		let cx = -w * 0.5 + WU * 0.5 + WU * i as f64;
		// floor runner rails (the drawer rides these, not the floor)
		for sx in [-1.0, 1.0] {
			let rx = cx + sx * 18.0;
			s = union(
				&s,
				&prism_y(
					&[(rx - 1.6, SW - 0.3), (rx + 1.6, SW - 0.3), (rx + 0.8, SW + RAIL_H), (rx - 0.8, SW + RAIL_H)],
					2.0,
					115.0,
				),
			);
		}
		// magnet pockets (Ø6.2 × 2.4) in the back wall interior face, one per unit
		// column at each unit-row centre; the pocket axis is vertical in the print.
		for j in 0..uh {
			let cz = HU * 0.5 + HU * j as f64;
			s = difference(&s, &cylinder(v(cx, D - BW - 1.0, cz), DVec3::Y, 3.1, 2.4 + 1.0, SEG));
		}
		// snap recesses (dock / desk-mount finger bump lands here), bottom + top;
		// the ends are 50° ramps, not square walls — a square end is a small flat
		// ceiling in the back-down print (the Python cross-audit flagged them)
		let recess = |face: f64, dir: f64| {
			let prof = [
				(104.9, face - dir),
				(111.1, face - dir),
				(111.1, face),
				(109.9, face + dir),
				(106.1, face + dir),
				(104.9, face),
			];
			prism_x(&prof, cx - 5.0, 10.0)
		};
		s = difference(&s, &recess(0.0, 1.0));
		s = difference(&s, &recess(h, -1.0));
	}
	// detent ceiling rib near the front: flanks 46.8° from PRINT horizontal
	// (the print-up axis here is -Y, so flank rise is measured in y per z).
	let rib = [(4.0, ceil + 0.3), (10.0, ceil + 0.3), (7.55, ceil - 2.0), (6.45, ceil - 2.0)];
	s = union(&s, &prism_x(&rib, -(iw * 0.5 + 0.5), iw + 1.0));
	s
}

// ---- drawers --------------------------------------------------------------------

/// The drawer for a `uw` × `uh` shell: body + oversize front panel + handle lip
/// (46.6° underside), detent lug, divider grooves, teardrop magnet pockets and
/// the integral label rails. Prints bottom-down, as used.
fn drawer(uw: usize, uh: usize) -> Solid {
	let (w, h) = (uw as f64 * WU, uh as f64 * HU);
	let (iw, ih) = (w - 2.0 * SW, h - 2.0 * SW);
	let bw = iw - 2.0 * SIDE_CLR;
	let ride = SW + RAIL_H; // z of the body underside (rides the rails)
	let top = ride + (ih - RAIL_H - TOP_CLR); // body top
	// body, hollowed (top open)
	let mut d = cuboid(v(-bw * 0.5, -0.5, ride), v(bw * 0.5, BODY_BACK, top));
	d = difference(
		&d,
		&cuboid(v(-(bw * 0.5 - D_WALL), 2.0, ride + D_FLOOR), v(bw * 0.5 - D_WALL, BODY_BACK - D_BACK, top + 1.0)),
	);
	// front panel: back face closes on the shell rim at y=0; 0.5 side/top reveal;
	// bottom FLUSH with the body underside so the whole drawer prints flat on the
	// bed (a hanging panel skirt would turn the body floor into a huge bridge —
	// caught by the steep/bridge gate on the first build of this example)
	let pw = w - 1.0;
	d = union(&d, &cuboid(v(-pw * 0.5, -PANEL_T - 0.5, ride), v(pw * 0.5, 0.0, h - 0.5)));
	// handle lip along the panel top; underside chamfer 46.6° for the print
	let lip = [
		(-3.2, h - 0.5),
		(-PANEL_T - 8.5, h - 0.5),
		(-PANEL_T - 8.5, h - 4.5),
		(-3.2, h - 13.5),
	];
	let lw = pw - 24.0;
	d = union(&d, &prism_x(&lip, -lw * 0.5, lw));
	// detent lugs (gentle 30° in, firmer 60° out) — one on each SIDE-WALL top at
	// the back, so the lug base is fully supported by the wall below (a single
	// wide lug on the back wall overhung the open cavity: a 17.7 mm bridge,
	// caught by the print gate). The lug is inset 0.3 from BOTH wall faces: a
	// lug side exactly coplanar with the wall face is the known coincident-face
	// union degeneracy (it broke the taller drawers at some coordinates).
	let lug = [
		(105.0, top - 0.5),
		(BODY_BACK, top - 0.5),
		(BODY_BACK - 2.0, top + LUG_OV + 2.5),
		(BODY_BACK - 4.8, top + LUG_OV + 2.5),
	];
	d = union(&d, &prism_x(&lug, -bw * 0.5 + 0.3, D_WALL - 0.6));
	d = union(&d, &prism_x(&lug, bw * 0.5 - D_WALL + 0.3, D_WALL - 0.6));
	// divider grooves, 20 pitch, both side walls (0.8 deep leaves 1.2 wall);
	// the cutter dips 0.3 below the interior floor so no cutter face is exactly
	// coplanar with it (the known coincident-face degeneracy)
	for k in 0..5 {
		let yg = 25.0 + 20.0 * k as f64;
		for sx in [-1.0, 1.0] {
			let (x_in, x_out) = (sx * (bw * 0.5 - D_WALL + 1.0), sx * (bw * 0.5 - D_WALL - 0.8));
			d = difference(
				&d,
				&cuboid(
					v(x_in.min(x_out), yg - 0.75, ride + D_FLOOR - 0.3),
					v(x_in.max(x_out), yg + 0.75, top + 1.0),
				),
			);
		}
	}
	// magnet pockets in the back face — teardrop (their axis is horizontal in
	// this part's print orientation; kernel_brep::holes::teardrop_hole)
	for i in 0..uw {
		let cx = -w * 0.5 + WU * 0.5 + WU * i as f64;
		for j in 0..uh {
			let cz = HU * 0.5 + HU * j as f64;
			d = teardrop_hole(&d, v(cx, BODY_BACK, cz), -DVec3::Y, DVec3::Z, 6.2, 1.9, 46.0, None)
				.expect("drawer magnet pocket");
		}
	}
	// label rails on the panel front (label slides in sideways; left end closed)
	let rail_top = [(-3.2, 33.2), (-5.9, 33.2), (-5.9, 31.0), (-4.9, 29.9), (-4.9, 31.2)];
	let rail_bot = [(-3.2, 14.2), (-5.9, 17.0), (-5.9, 18.4), (-4.9, 18.4), (-4.9, 17.2), (-3.2, 17.2)];
	d = union(&d, &prism_x(&rail_top, -21.0, 42.0));
	d = union(&d, &prism_x(&rail_bot, -21.0, 42.0));
	d = union(&d, &cuboid(v(-22.0, -4.9, 15.0), v(-20.5, -3.2, 32.0))); // left end stop
	d
}

/// A cross divider for the `uw` × `uh` drawer (slides into the wall grooves).
fn divider(uw: usize, uh: usize) -> Solid {
	let (w, h) = (uw as f64 * WU, uh as f64 * HU);
	let (iw, ih) = (w - 2.0 * SW, h - 2.0 * SW);
	let bw = iw - 2.0 * SIDE_CLR;
	let inner_w = bw - 2.0 * D_WALL;
	let dh = (ih - RAIL_H - TOP_CLR) - D_FLOOR - 0.3;
	cuboid(v(0.0, 0.0, 0.0), v(inner_w + 1.6 - 0.3, dh, 1.2))
}

// ---- the male side of the joint ---------------------------------------------------

/// The connector key: a FULLY SYMMETRIC bowtie spline running the whole channel
/// (engages 2.35 into each of two facing sockets, both mirror planes + both
/// ends identical — so any two touching modules connect the same way up, down,
/// left or right, and the key inserts from either face). It sits flush 0.5
/// inside the joint (invisible); push it through from the other side to remove.
/// Prints lying on a root face (the 59° flanks self-support).
fn key() -> Solid {
	let bowtie = ccw(vec![
		DVec2::new(-KEY_ENG, -KEY_ROOT),
		DVec2::new(0.0, -KEY_WAIST),
		DVec2::new(KEY_ENG, -KEY_ROOT),
		DVec2::new(KEY_ENG, KEY_ROOT),
		DVec2::new(0.0, KEY_WAIST),
		DVec2::new(-KEY_ENG, KEY_ROOT),
	]);
	extrude(&bowtie, KEY_LEN)
}

/// One foot rail: 3-tall chamfered base + the male ridge; slides into a bottom
/// socket track and lifts the lowest module off the desk. Prints as used.
fn feet_rail() -> Solid {
	let prof = [
		(-6.0, 0.0),
		(6.0, 0.0),
		(6.0, 1.2),
		(4.2, 3.0),
		(KEY_WAIST, 3.0),
		(KEY_ROOT, 3.0 + KEY_ENG),
		(-KEY_ROOT, 3.0 + KEY_ENG),
		(-KEY_WAIST, 3.0),
		(-4.2, 3.0),
		(-6.0, 1.2),
	];
	prism_y(&prof, 1.0, 119.0)
}

// ---- wall dock + hook clips --------------------------------------------------------

const DOCK_PLATE_Y: f64 = 125.4; // back plate rear face (grooves open here)
const DOCK_TRACKS: [f64; 2] = [22.0, 62.0]; // horizontal hook tracks, 40 apart

/// The wall dock: a floor with two male rails at the back (the module slides on
/// and its bottom sockets capture vertically), a snap finger that clicks into the
/// module's bottom recess, and a back plate whose REAR face carries two
/// horizontal dovetail tracks for the hook clips (asymmetric profile: the upper
/// flank is 46.1° from horizontal so the floor-down print needs no support) plus
/// three countersunk screw holes. Prints floor-down.
fn dock_wall() -> Solid {
	let mut s = cuboid(v(-42.0, 0.0, -4.0), v(42.0, DOCK_PLATE_Y, 0.0)); // floor
	s = union(&s, &cuboid(v(-40.0, 120.0, -4.0), v(40.0, DOCK_PLATE_Y, 70.0))); // plate
	// module rails at the back of the floor (engage the rear 40 of the sockets)
	for rx in [-20.0f64, 20.0] {
		let prof = [
			(rx - KEY_WAIST, -0.5),
			(rx + KEY_WAIST, -0.5),
			(rx + KEY_WAIST, 0.0),
			(rx + KEY_ROOT, KEY_ENG),
			(rx - KEY_ROOT, KEY_ENG),
			(rx - KEY_WAIST, 0.0),
		];
		s = union(&s, &prism_y(&prof, 80.0, 120.5));
	}
	// snap finger: C-slot through the floor, bump on the free end (ramped both ways)
	s = difference(&s, &cuboid(v(7.0, 86.0, -5.0), v(9.0, 112.0, 1.0)));
	s = difference(&s, &cuboid(v(-9.0, 86.0, -5.0), v(-7.0, 112.0, 1.0)));
	s = difference(&s, &cuboid(v(-9.0, 110.0, -5.0), v(9.0, 112.0, 1.0)));
	let bump = [(105.5, -0.3), (110.5, -0.3), (109.1, 1.0), (106.9, 1.0)];
	s = union(&s, &prism_x(&bump, -5.0, 10.0));
	// hook tracks on the plate rear face — asymmetric dovetail (the upper flank
	// rises 2.6 in z per 2.5 of depth = 46.1° from horizontal in the floor-down
	// print), cut along x; the proud lead-out is a separate vertical segment so
	// it does not flatten the flank angle
	for &tz in &DOCK_TRACKS {
		let prof = [
			(DOCK_PLATE_Y + 0.5, tz - SOCK_OPEN),
			(DOCK_PLATE_Y, tz - SOCK_OPEN),
			(DOCK_PLATE_Y - SOCK_DEPTH, tz - SOCK_ROOT),
			(DOCK_PLATE_Y - SOCK_DEPTH, tz + 5.6),
			(DOCK_PLATE_Y, tz + SOCK_OPEN),
			(DOCK_PLATE_Y + 0.5, tz + SOCK_OPEN),
		];
		s = difference(&s, &prism_x(&prof, -41.0, 82.0));
	}
	// countersunk screw holes (head flush on the module side): teardrop through
	// bore + 100° cone (crown normals stay shy of the 45° gate); the cone mouth
	// is Ø9 exactly at the face, with a full 1.0 overshoot outside it
	for (hx, hz) in [(0.0, 12.0), (-32.0, 40.0), (32.0, 40.0)] {
		s = teardrop_hole(&s, v(hx, 120.0, hz), DVec3::Y, DVec3::Z, 4.4, 5.4, 46.0, None).expect("dock screw hole");
		s = difference(&s, &cone(v(hx, 119.0, hz), DVec3::Y, 5.69, 4.78, SEG));
	}
	s
}

/// Hook-clip blade profile shared bits: the spine (lies against the plate rear
/// face) and its two asymmetric dovetail feet at the track heights. `rear`
/// closes the polygon down the spine's rear edge with prong/lip points inserted.
fn hook_blade(rear: &[(f64, f64)]) -> Solid {
	let mut prof: Vec<(f64, f64)> = vec![(DOCK_PLATE_Y, 14.0)];
	for &tz in &DOCK_TRACKS {
		// foot: waist ±2.8 at the face, asymmetric root (upper +5.15, lower −4.21)
		prof.push((DOCK_PLATE_Y, tz - KEY_WAIST));
		prof.push((DOCK_PLATE_Y - KEY_ENG, tz - KEY_ROOT));
		prof.push((DOCK_PLATE_Y - KEY_ENG, tz + 5.15));
		prof.push((DOCK_PLATE_Y, tz + KEY_WAIST));
	}
	prof.push((DOCK_PLATE_Y, 70.0)); // spine front top
	prof.extend_from_slice(rear); // rear edge top -> bottom (with prong/lip)
	prof.push((DOCK_PLATE_Y + 5.0, 14.0)); // spine rear bottom
	prism_x(&prof, -2.3, 4.6)
}

/// Skådis hook clip: blade prong 4.6 × 12 through the 5 × 15 slot, down-lip
/// behind the ~5.1 board. Prints flat on its face.
fn hook_skadis() -> Solid {
	let (sp, board) = (DOCK_PLATE_Y + 5.0, DOCK_PLATE_Y + 5.0 + 5.4);
	hook_blade(&[
		(sp, 70.0),
		(sp, 68.0),
		(board + 3.0, 68.0), // prong + lip top
		(board + 3.0, 48.0), // lip rear, down 20
		(board, 48.0),
		(board, 56.0), // lip front (behind the board), catch 8
		(sp, 56.0),    // prong underside
	])
}

/// 1/4-inch pegboard hook clip: 5.8-tall prong through the Ø6.35 hole, up-lip
/// behind a board up to 6.4 thick. Prints flat on its face.
fn hook_pegboard() -> Solid {
	let sp = DOCK_PLATE_Y + 5.0;
	let (tip, lipf) = (sp + 10.6, sp + 7.0);
	hook_blade(&[
		(sp, 70.0),
		(sp, 61.8),
		(lipf, 61.8), // prong top out to the lip front
		(lipf, 66.8), // lip front, rising 5
		(tip, 66.8),  // lip top
		(tip, 57.5),  // lip rear / prong tip, chamfered in
		(tip - 1.5, 56.0),
		(sp, 56.0), // prong underside
	])
}

/// Under-desk mount: a plate with two male rails and a back fence on one face
/// (module top sockets slide on until the fence; the snap finger clicks into the
/// module's TOP recess) and four countersunk screw holes. Prints rails-up, then
/// flips to install.
fn desk_mount() -> Solid {
	let mut s = cuboid(v(-50.0, 0.0, 0.0), v(50.0, 130.0, 4.0));
	for rx in [-20.0f64, 20.0] {
		let prof = [
			(rx - KEY_WAIST, 3.5),
			(rx + KEY_WAIST, 3.5),
			(rx + KEY_WAIST, 4.0),
			(rx + KEY_ROOT, 4.0 + KEY_ENG),
			(rx - KEY_ROOT, 4.0 + KEY_ENG),
			(rx - KEY_WAIST, 4.0),
		];
		s = union(&s, &prism_y(&prof, 4.0, 122.0));
	}
	// fence (0.6 shy of the plate back edge — a flush face-on-face union is the
	// known coincident-plane degeneracy)
	s = union(&s, &cuboid(v(-42.0, 122.5, 3.5), v(42.0, 129.4, 12.0)));
	// snap finger + bump (module back face lands at the fence, recess 12 from it)
	s = difference(&s, &cuboid(v(7.0, 84.5, -1.0), v(9.0, 110.5, 5.0)));
	s = difference(&s, &cuboid(v(-9.0, 84.5, -1.0), v(-7.0, 110.5, 5.0)));
	s = difference(&s, &cuboid(v(-9.0, 108.5, -1.0), v(9.0, 110.5, 5.0)));
	let bump = [(104.0, 3.7), (109.0, 3.7), (107.6, 5.0), (105.4, 5.0)];
	s = union(&s, &prism_x(&bump, -5.0, 10.0));
	for (hx, hy) in [(-38.0, 20.0), (38.0, 20.0), (-38.0, 110.0), (38.0, 110.0)] {
		s = difference(&s, &cylinder(v(hx, hy, -0.5), DVec3::Z, 2.2, 5.0, SEG));
		s = difference(&s, &cone(v(hx, hy, 5.0), -DVec3::Z, 5.69, 4.78, SEG)); // Ø9 mouth at the face
	}
	s
}

/// The label plate (40 × 14 × 1.0) that slides into every drawer's panel rails.
fn label_plate() -> Solid {
	cuboid(v(0.0, 0.0, 0.0), v(40.0, 14.0, 1.0))
}

// ---- audit / emit -----------------------------------------------------------------

struct PartCheck {
	ok: bool,
	mesh_use: Mesh,
}

/// Validate + tessellate + audit one part. The STL is written in PRINT
/// orientation (`to_print`), and A-PRINT-1..4 are asserted there: steep_area == 0,
/// max bridge span ≤ 12, watertight, valid, fits the MK4S envelope.
fn emit(name: &str, s: &Solid, to_print: DAffine3) -> PartCheck {
	let val = validate(s);
	let mut printed = s.transformed(to_print);
	// drop the part onto the bed (z = 0) so the emitted STL is print-ready as-is
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
	let _ = std::fs::write(format!("drawer_system/parts/{name}.stl"), mesh_p.to_stl_binary());
	println!(
		"  {name:16} valid={:5} wt={wt:5} steep={:8.3} mm²  bridge≤{:5.1}  {:3.0}g  {:6.0}mm³  {}",
		val.is_valid(),
		rep.steep_area,
		rep.max_bridge_span,
		vol * PLA_G_PER_MM3,
		vol,
		if ok { "OK" } else { "<<< FAIL" }
	);
	PartCheck { ok, mesh_use: tessellate_default(s) }
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
		"  {label:44} min_dist={d:7.3}  want {}  {}",
		if contact { "contact (<0.06)" } else { "clearance (>=0.10)" },
		if pass { "OK" } else { "<<< FAIL" }
	);
}

/// Static interference volume between two posed solids (0.0 when disjoint),
/// computed as vol(a) − vol(a \ b): the difference route stays a single shell
/// even when the overlap region itself is several disjoint slivers (which
/// try_intersection currently mis-builds — engine repro in tests/).
fn overlap_mm3(a: &Solid, b: &Solid) -> f64 {
	match kernel_brep::try_difference(a, b) {
		Ok(rem) => (volume(a).abs() - volume(&rem).abs()).max(0.0),
		Err(e) => {
			println!("  (try_difference refused: {e:?} — treating as gate failure)");
			f64::NAN
		}
	}
}

fn main() {
	let _ = std::fs::create_dir_all("drawer_system/parts");
	println!("DOVESTACK modular drawer system — parts (STLs in print orientation):\n");

	let shell_1x1 = shell(1, 1);
	let shell_2x1 = shell(2, 1);
	let shell_1x2 = shell(1, 2);
	let drawer_1x1 = drawer(1, 1);
	let drawer_2x1 = drawer(2, 1);
	let drawer_1x2 = drawer(1, 2);
	let key_part = key();
	let feet = feet_rail();
	let dock = dock_wall();
	let hook_sk = hook_skadis();
	let hook_pb = hook_pegboard();
	let desk = desk_mount();
	let label = label_plate();
	let div_1x1 = divider(1, 1);
	let div_2x1 = divider(2, 1);
	let div_1x2 = divider(1, 2);

	// print orientations (A-PRINT-*): shells back-down, hooks flat, rest as modelled
	let shell_print = tr(0.0, 0.0, D) * DAffine3::from_rotation_x(-FRAC_PI_2);
	let flat = DAffine3::from_rotation_y(-FRAC_PI_2); // blade thickness (x) -> up
	let ident = DAffine3::IDENTITY;

	let mut ok = true;
	// File names carry the grid size AND the physical envelope, so a user can
	// read the dimensions off the file itself. Mounting hooks are named by the
	// board standard they fit (dimension, not brand).
	let parts: Vec<(&str, &Solid, DAffine3)> = vec![
		("shell_1x1_80x50x120mm", &shell_1x1, shell_print),
		("shell_2x1_160x50x120mm", &shell_2x1, shell_print),
		("shell_1x2_80x100x120mm", &shell_1x2, shell_print),
		("drawer_1x1_80x50x120mm", &drawer_1x1, ident),
		("drawer_2x1_160x50x120mm", &drawer_2x1, ident),
		("drawer_1x2_80x100x120mm", &drawer_1x2, ident),
		("key_spline_119mm", &key_part, flat), // lying on a root face
		("feet_rail_118mm", &feet, ident),
		("wall_dock_84x125x74mm", &dock, ident),
		("hook_slot_5x15mm", &hook_sk, flat),
		("hook_pegboard_6.35mm", &hook_pb, flat),
		("desk_mount_100x130mm", &desk, ident),
		("label_plate_40x14mm", &label, ident),
		("divider_1x1_69x34mm", &div_1x1, ident),
		("divider_2x1_149x34mm", &div_2x1, ident),
		("divider_1x2_69x84mm", &div_1x2, ident),
	];
	let mut meshes: std::collections::HashMap<&str, Mesh> = std::collections::HashMap::new();
	for (name, s, m) in &parts {
		let r = emit(name, s, *m);
		ok &= r.ok;
		meshes.insert(name, r.mesh_use);
	}

	// ---- negative control: the support gate must BITE in a wrong orientation
	// (side-down, where the top/bottom socket flanks become 31°-from-vertical
	// downward faces over the full module depth)
	let wrong = tessellate_default(&shell_1x1).support_free_report(Vec3::new(1.0, 0.0, 0.0), 45.0, 0.3);
	let nc_gate = wrong.steep_area > 500.0;
	ok &= nc_gate;
	println!(
		"\nA-PRINT NC: shell_1x1 audited side-down -> steep {:.1} mm² (must exceed 500) {}",
		wrong.steep_area,
		if nc_gate { "OK" } else { "<<< FAIL" }
	);

	// ---- A-JOINT: constant relations
	let flank_clr = SOCK_OPEN - KEY_WAIST;
	let root_clr = SOCK_DEPTH - KEY_ENG;
	let flare_key = (KEY_ROOT - KEY_WAIST) / KEY_ENG;
	let flare_sock = (SOCK_ROOT - SOCK_OPEN) / SOCK_DEPTH;
	let joint_ok = (flank_clr - 0.2).abs() < 1e-9 && (root_clr - 0.15).abs() < 1e-9 && (flare_key - flare_sock).abs() < 0.001;
	ok &= joint_ok;
	println!(
		"A-JOINT-1: flank clearance {flank_clr:.2} (want 0.20), axial {root_clr:.2} (want 0.15), \
		 flank slopes key {flare_key:.3} vs socket {flare_sock:.3}  {}",
		if joint_ok { "OK" } else { "<<< FAIL" }
	);

	// ---- posed interference gates (negative controls prove retention) ---------
	println!("\nposed retention gates (static interference, mm³):");
	// keyed stack: shell on shell, the symmetric spline through the facing
	// sockets, flush 0.5 inside each face (no contact anywhere when seated).
	// A-SYM-1: the key must work in ALL FOUR insertion orientations — either
	// end first, either root face up — the functional meaning of "symmetric":
	// any two touching modules connect the same way up, down, left or right.
	let top_shell = |dz: f64| shell_1x1.transformed(tr(0.0, 0.0, HU + dz));
	let mid = KEY_LEN * 0.5;
	let orientations: [(&str, DAffine3); 4] = [
		("as-is", DAffine3::IDENTITY),
		("spun 180°", DAffine3::from_rotation_z(std::f64::consts::PI)),
		(
			"end-for-end",
			tr(0.0, 0.0, mid) * DAffine3::from_rotation_x(std::f64::consts::PI) * tr(0.0, 0.0, -mid),
		),
		(
			"spun + flipped",
			DAffine3::from_rotation_z(std::f64::consts::PI)
				* tr(0.0, 0.0, mid) * DAffine3::from_rotation_x(std::f64::consts::PI) * tr(0.0, 0.0, -mid),
		),
	];
	let place_key = DAffine3::from_mat3_translation(
		DMat3::from_cols(v(0.0, 0.0, 1.0), v(1.0, 0.0, 0.0), v(0.0, 1.0, 0.0)),
		v(SOCK_OFF, 0.5, HU),
	);
	for (label, orient) in orientations {
		let key_stack = key_part.transformed(place_key * orient);
		let seated = overlap_mm3(&top_shell(0.02), &key_stack) + overlap_mm3(&shell_1x1, &key_stack);
		let lifted = overlap_mm3(&top_shell(1.0), &key_stack);
		let g1 = seated < 0.05 && lifted > 1.0;
		ok &= g1;
		println!(
			"  key in stack ({label:14}): seated {seated:.3} (want ~0), module lifted 1mm {lifted:.2} (want >1)  {}",
			if g1 { "OK" } else { "<<< FAIL" }
		);
	}

	// drawer detent: free short of the rib, static interference past it
	let lift = tr(0.0, 0.0, 0.05); // break rail coincidence for clean booleans
	let free = overlap_mm3(&drawer_1x1.transformed(lift * tr(0.0, -95.0, 0.0)), &shell_1x1);
	let past = overlap_mm3(&drawer_1x1.transformed(lift * tr(0.0, -106.0, 0.0)), &shell_1x1);
	let closed = overlap_mm3(&drawer_1x1.transformed(lift * tr(0.0, -0.01, 0.0)), &shell_1x1);
	let g2 = free < 0.05 && closed < 0.05 && past > 0.2;
	ok &= g2;
	println!(
		"  drawer detent: closed {closed:.3} / open-95 {free:.3} (want ~0), past-stop {past:.2} (want >0.2)  {}",
		if g2 { "OK" } else { "<<< FAIL" }
	);

	// dock rail retention: seated module free, lifted module captured (poses sit
	// 0.05 off the wall plate so the flush back-face contact is not an exact
	// coincident plane)
	let dock_seated = overlap_mm3(&shell_1x1.transformed(tr(0.0, -0.05, 0.05)), &dock);
	let dock_lifted = overlap_mm3(&shell_1x1.transformed(tr(0.0, -0.05, 1.0)), &dock);
	let g3 = dock_seated < 0.05 && dock_lifted > 1.0;
	ok &= g3;
	println!(
		"  dock dovetail: seated {dock_seated:.3} (want ~0), lifted 1mm {dock_lifted:.2} (want >1)  {}",
		if g3 { "OK" } else { "<<< FAIL" }
	);

	// ---- A-MOUNT: hook prong envelopes from emitted geometry ------------------
	// what actually sits inside the board (spine rear -> board rear) must fit the
	// slot/hole: slice the hook solid with the board volume and measure the slice
	let sp = DOCK_PLATE_Y + 5.0;
	let in_board_env = |s: &Solid, board_t: f64| {
		let window = cuboid(v(-50.0, sp + 0.05, -50.0), v(50.0, sp + board_t - 0.05, 200.0));
		let slice = try_intersection(s, &window).expect("board slice");
		let (lo, hi) = mesh_aabb(&tessellate_default(&slice));
		hi - lo
	};
	let sk = in_board_env(&hook_sk, 5.1); // Skådis board 5.1
	let pb = in_board_env(&hook_pb, 6.4); // 1/4" board 6.4
	// Skådis slot 5×15: prong ≤4.6 thick, ≤12 tall; pegboard Ø6.35 hole: prong ≤5.8 tall.
	let mount_ok = sk.x <= 4.61 && sk.z <= 12.6 && pb.x <= 4.61 && pb.z <= 6.35;
	ok &= mount_ok;
	println!(
		"A-MOUNT-1: in-board envelope skadis {:.1} thick × {:.1} tall (≤4.6×12.6), pegboard {:.1} × {:.1} (≤4.6×6.35)  {}",
		sk.x, sk.z, pb.x, pb.z,
		if mount_ok { "OK" } else { "<<< FAIL" }
	);

	// ---- assembly showcase ------------------------------------------------------
	println!("\nassembly (posed contacts / clearances):");
	let mut asm = Mesh::new();
	let mut posed: Vec<(String, Mesh)> = Vec::new();
	let place = |scene: &mut Mesh, list: &mut Vec<(String, Mesh)>, name: &str, s: &Solid, m: DAffine3| {
		let mesh = tessellate_default(&s.transformed(m));
		merge_into(scene, &mesh);
		list.push((name.to_string(), mesh));
	};

	let up_key = DAffine3::from_mat3_translation(DMat3::from_cols(v(0.0, 0.0, 1.0), v(1.0, 0.0, 0.0), v(0.0, 1.0, 0.0)), DVec3::ZERO);
	let side_key = DAffine3::from_mat3_translation(DMat3::from_cols(v(1.0, 0.0, 0.0), v(0.0, 0.0, -1.0), v(0.0, 1.0, 0.0)), DVec3::ZERO);

	// left tower: D (1x2) | A (1x1) + B (1x1) side by side | C (2x1) on top
	let za = 3.0; // feet lift
	for (mx, module) in [(-80.0, &shell_1x2), (0.0, &shell_1x1), (80.0, &shell_1x1)] {
		for fx in [-20.0, 20.0] {
			place(&mut asm, &mut posed, "feet", &feet, tr(mx + fx, 0.0, 0.0));
		}
		let _ = module;
	}
	place(&mut asm, &mut posed, "D", &shell_1x2, tr(-80.0, 0.0, za));
	place(&mut asm, &mut posed, "A", &shell_1x1, tr(0.0, 0.0, za));
	place(&mut asm, &mut posed, "B", &shell_1x1, tr(80.0, 0.0, za));
	place(&mut asm, &mut posed, "C", &shell_2x1, tr(40.0, 0.0, za + HU));
	for kx in [-20.0, 20.0, 60.0, 100.0] {
		place(&mut asm, &mut posed, "key_top", &key_part, tr(kx, 0.5, za + HU) * up_key);
	}
	place(&mut asm, &mut posed, "key_gangAB", &key_part, tr(40.0, 0.5, za + 25.0) * side_key);
	place(&mut asm, &mut posed, "key_gangDA", &key_part, tr(-40.0, 0.5, za + 25.0) * side_key);
	place(&mut asm, &mut posed, "drw_D", &drawer_1x2, tr(-80.0, 0.0, za));
	place(&mut asm, &mut posed, "drw_A", &drawer_1x1, tr(0.0, 0.0, za));
	place(&mut asm, &mut posed, "drw_B", &drawer_1x1, tr(80.0, 0.0, za));
	place(&mut asm, &mut posed, "drw_C", &drawer_2x1, tr(40.0, -90.0, za + HU)); // open to near the detent
	place(&mut asm, &mut posed, "div_C", &div_2x1, tr(40.0 - (WU * 2.0 - 2.0 * SW - 0.7 - 2.0 * D_WALL + 1.3) * 0.5, -90.0 + 64.25, za + HU + SW + RAIL_H + D_FLOOR) );
	place(&mut asm, &mut posed, "label_B", &label, tr(80.0 - 20.0, -3.7, za + 17.2) * DAffine3::from_rotation_x(FRAC_PI_2));

	// dock vignette at the right: dock + skadis board + hooks + docked module E
	let gd = tr(230.0, 20.0, 26.0);
	place(&mut asm, &mut posed, "dock", &dock, gd);
	place(&mut asm, &mut posed, "E", &shell_1x1, gd);
	place(&mut asm, &mut posed, "drw_E", &drawer_1x1, gd);
	for hx in [-20.0, 20.0] {
		place(&mut asm, &mut posed, "hook", &hook_sk, gd * tr(hx, 0.0, 0.0));
	}
	// board fragment (visual context only — NOT a printed part, no audit)
	let mut board = cuboid(v(-80.0, DOCK_PLATE_Y + 5.0, -46.0), v(80.0, DOCK_PLATE_Y + 10.1, 84.0));
	for bx in [-60.0, -20.0, 20.0, 60.0] {
		for bz in [16.0, 56.0] {
			board = difference(&board, &cuboid(v(bx - 2.5, DOCK_PLATE_Y + 4.0, bz), v(bx + 2.5, DOCK_PLATE_Y + 11.1, bz + 15.0)));
		}
	}
	place(&mut asm, &mut posed, "board", &board, gd);

	let _ = asm.write_stl_binary("drawer_system/ASSEMBLY.stl");
	println!("  scene: {} triangles -> drawer_system/ASSEMBLY.stl", asm.indices.len() / 3);

	// designed contacts + spot clearances (A-ASM-1)
	let get = |n: &str| posed.iter().filter(|(pn, _)| pn == n).collect::<Vec<_>>();
	let pair = |a: &str, ai: usize, b: &str, bi: usize| (get(a)[ai].1.clone(), get(b)[bi].1.clone());
	{
		let (a, b) = pair("A", 0, "B", 0);
		relation("A | B side faces (flush gang)", &a, &b, true, &mut ok);
		let (a, c) = pair("A", 0, "C", 0);
		relation("C stacked on A", &a, &c, true, &mut ok);
		let (a, d) = pair("A", 0, "D", 0);
		relation("D | A side faces", &a, &d, true, &mut ok);
		let (a, f) = pair("A", 0, "feet", 2);
		relation("A rests on its feet rail", &a, &f, true, &mut ok);
		let (a, k) = pair("A", 0, "key_top", 0);
		relation("spline key floats in A's socket (clearance)", &a, &k, false, &mut ok);
		let (c, k) = pair("C", 0, "key_top", 0);
		relation("spline key floats in C's socket (clearance)", &c, &k, false, &mut ok);
		let (a, da) = pair("A", 0, "drw_A", 0);
		relation("drawer A rides shell A rails", &a, &da, true, &mut ok);
		let (c, dc) = pair("C", 0, "drw_C", 0);
		relation("open drawer C still on rails", &c, &dc, true, &mut ok);
		let (b, lb) = pair("B", 0, "label_B", 0);
		relation("label plate seated in B's rails", &b, &lb, false, &mut ok); // label touches DRAWER, clears shell
		let (db, lb2) = pair("drw_B", 0, "label_B", 0);
		relation("label plate against B drawer panel", &db, &lb2, true, &mut ok);
		let (da2, db2) = pair("drw_A", 0, "drw_B", 0);
		relation("adjacent drawer panels never touch", &da2, &db2, false, &mut ok);
		let (e, dk) = pair("E", 0, "dock", 0);
		relation("module E seated on dock floor", &e, &dk, true, &mut ok);
		let (h, dk2) = pair("hook", 0, "dock", 0);
		relation("hook clip spine against dock plate", &h, &dk2, true, &mut ok);
		let (h2, bd) = pair("hook", 0, "board", 0);
		relation("hook prong rests on Skådis slot edge", &h2, &bd, true, &mut ok);
		let (dc2, cdiv) = pair("drw_C", 0, "div_C", 0);
		relation("divider seated in open drawer grooves", &dc2, &cdiv, true, &mut ok);
	}

	println!("\nRESULT: {}", if ok { "PASS — every DESIGN.md gate green" } else { "FAIL — see <<< lines" });
	if !ok {
		std::process::exit(1);
	}
}
