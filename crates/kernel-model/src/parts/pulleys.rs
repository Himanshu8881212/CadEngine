// Copyright (c) LMCAD. Licensed under the MIT License.

//! **GT2 (2 mm pitch) timing-belt pulleys.** The GT2 tooth is a curvilinear profile from the
//! Gates PowerGrip GT2 specification (belt tooth depth 0.75 mm, belt thickness 1.38 mm, pitch
//! line distance PLD 0.254 mm). Pulley grooves here use the de-facto-standard clearance groove
//! polygon published in the droftarts "Parametric pulley" generator (Thingiverse thing:16627,
//! `tooth_profile_GT2_2mm`, groove depth 0.764 / width 1.494) — the profile behind most printed
//! and machined hobby GT2 pulleys — hardcoded verbatim below. That polygon is itself a
//! **piecewise-linear approximation** of the Gates curvilinear tooth with clearance added; this
//! file inherits that approximation and says so rather than pretending to the exact spline.

use super::circle48;
use kernel_brep::math::{DAffine3, DVec2, DVec3};
use kernel_brep::{difference, extrude_with_holes, Solid};
use std::f64::consts::PI;

/// The GT2 2 mm pulley groove polygon, local frame: y = 0 on the pulley outer circle (positive
/// y toward the axis, groove tip at y = 0.764), x tangential spanning the 1.494 mm groove
/// width. Source: `tooth_profile_GT2_2mm` in the droftarts/rbuckland parametric pulley
/// generator (Thingiverse thing:16627 / github rbuckland/openscad.parametric-pulley), verbatim
/// without the two cut-tail points; derived from the Gates PowerGrip GT2 2 mm belt profile.
const GT2_2MM_GROOVE: [(f64, f64); 21] = [
	(0.747183, 0.0),
	(0.647876, 0.037218),
	(0.598311, 0.130528),
	(0.578556, 0.238423),
	(0.547158, 0.343077),
	(0.504649, 0.443762),
	(0.451556, 0.53975),
	(0.358229, 0.636924),
	(0.2484, 0.707276),
	(0.127259, 0.750044),
	(0.0, 0.76447),
	(-0.127259, 0.750044),
	(-0.2484, 0.707276),
	(-0.358229, 0.636924),
	(-0.451556, 0.53975),
	(-0.504797, 0.443762),
	(-0.547291, 0.343077),
	(-0.578605, 0.238423),
	(-0.598311, 0.130528),
	(-0.648009, 0.037218),
	(-0.747183, 0.0),
];

/// GT2 pitch-line differential: the belt's pitch line rides 0.254 mm above the pulley outer
/// surface, so OD = PD − 2 × 0.254 (Gates PowerGrip GT2 2 mm convention).
const GT2_PLD: f64 = 0.254;

/// GT2 belt pitch (mm/tooth) — the "2" of GT2.
const GT2_PITCH: f64 = 2.0;

/// Pitch radius of a GT2 2 mm pulley: pitch Ø = `teeth`·2/π (the belt pitch line wraps the
/// pulley on this circle; the machined OD sits 2 × [`GT2_PLD`] below it).
fn gt2_pitch_radius(teeth: usize) -> f64 {
	teeth as f64 / PI
}

/// Exact pitch-line length of a two-pulley belt loop: straight spans `2·√(C² − (r1−r2)²)` plus
/// the wrap arcs `π(r1+r2) + 2·asin((r1−r2)/C)·(r1−r2)`.
fn belt_loop_length(c: f64, r1: f64, r2: f64) -> f64 {
	let d = r1 - r2;
	2.0 * (c * c - d * d).sqrt() + PI * (r1 + r2) + 2.0 * (d / c).asin() * d
}

/// **GT2 2 mm belt sizing** for a two-pulley drive: the exact pitch-line loop length around
/// `t1`- and `t2`-tooth pulleys at centre distance `center_distance` (standard open-belt
/// geometry: two straight spans tangent to both pitch circles plus the two wrap arcs), and the
/// commercial belt size as that length in 2 mm teeth **rounded to the nearest whole tooth**
/// (closed GT2 loops are sold in integer tooth counts; re-derive the matching exact centre
/// distance with [`gt2_center_distance`]). `None` when either pulley has < 2 teeth or the
/// pitch circles are not strictly separated (`center_distance ≤ r1 + r2`).
pub fn gt2_belt(center_distance: f64, t1: usize, t2: usize) -> Option<(f64, usize)> {
	let (r1, r2) = (gt2_pitch_radius(t1), gt2_pitch_radius(t2));
	// NaN-safe: the conjunction refuses NaN centre distances too.
	if t1 < 2 || t2 < 2 || !(center_distance > r1 + r2 && center_distance.is_finite()) {
		return None;
	}
	let length = belt_loop_length(center_distance, r1, r2);
	Some((length, (length / GT2_PITCH).round() as usize))
}

/// Inverse of [`gt2_belt`]: the exact centre distance at which a closed GT2 belt of
/// `belt_teeth` teeth (pitch length `belt_teeth`·2 mm) wraps `t1`- and `t2`-tooth pulleys
/// taut. Solved by bisection on the strictly increasing loop-length function (converges to
/// machine precision; the bracket is `(r1+r2, L/2]` since `L ≥ 2C` always). `None` when a
/// pulley has < 2 teeth or the belt is too short to clear both pulleys (loop length at
/// touching pitch circles already exceeds it).
pub fn gt2_center_distance(belt_teeth: usize, t1: usize, t2: usize) -> Option<f64> {
	let (r1, r2) = (gt2_pitch_radius(t1), gt2_pitch_radius(t2));
	let target = belt_teeth as f64 * GT2_PITCH;
	if t1 < 2 || t2 < 2 {
		return None;
	}
	let (mut lo, mut hi) = (r1 + r2, target * 0.5);
	if hi <= lo || belt_loop_length(lo + 1e-12, r1, r2) > target {
		return None; // belt shorter than the loop around touching pitch circles
	}
	for _ in 0..200 {
		let mid = 0.5 * (lo + hi);
		if belt_loop_length(mid, r1, r2) < target {
			lo = mid;
		} else {
			hi = mid;
		}
	}
	Some(0.5 * (lo + hi))
}

/// A **GT2 2 mm-pitch timing pulley**: `teeth` grooves on the standard outer diameter
/// `OD = 2·teeth/π − 2·0.254`, a toothed band `belt_width` wide, bored at `bore_d`, optionally
/// with a washer-style retaining **flange** (Ø OD + 3 × 1 mm — a manufacturer-typical
/// proportion, not a standard dimension) on each end.
///
/// Construction: the toothed disc cross-section is one polygon (each groove's published
/// 21-point polyline planted with its mouth chord exactly on the OD circle, tip pointing at
/// the axis, OD arcs between grooves). Unflanged, that profile plus the bore extrude in a
/// single [`extrude_with_holes`] — watertight by construction. Flanged, the belt channel is
/// **turned like a lathe groove**: a full-height bored flange blank (one `extrude_with_holes`)
/// minus one ring cutter whose *hole* is the toothed profile — a single boolean whose every
/// contact is transverse (the cutter's end caps slice the blank's wall mid-height), with no
/// coplanar face-on-face contact anywhere. (Earlier construction — flange discs fused on by
/// coplanar unions, then a through-drill — stitch-exploded depending on the process hash
/// seed.) Genus 1 either way.
pub fn gt2_pulley(teeth: usize, belt_width: f64, bore_d: f64, flanged: bool) -> Solid {
	let od = 2.0 * teeth as f64 / PI - 2.0 * GT2_PLD;
	let half_w = GT2_2MM_GROOVE[0].0; // groove half-width 0.747
	let chord = (od * 0.5) * (od * 0.5) - half_w * half_w;
	// NaN-safe rejection: `!(x > 0)` (not `x <= 0`) so NaN inputs are refused too.
	if !(chord > 0.0 && belt_width > 0.0) || teeth < 2 {
		return Solid::default();
	}
	// Groove mouth chord distance: the polygon's y = 0 corners land exactly on the OD circle.
	let tdc = chord.sqrt();
	let pitch_angle = 2.0 * PI / teeth as f64;
	let delta = half_w.atan2(tdc); // half-angle subtended by a groove mouth

	let mut profile: Vec<DVec2> = Vec::with_capacity(teeth * 24);
	for k in 0..teeth {
		let theta = k as f64 * pitch_angle;
		let (rdir, tdir) = (DVec2::new(theta.cos(), theta.sin()), DVec2::new(-theta.sin(), theta.cos()));
		// Groove polyline, traversed from the −x corner to the +x corner so the walk stays CCW
		// (the published polygon is listed +x → −x).
		for &(px, py) in GT2_2MM_GROOVE.iter().rev() {
			profile.push(rdir * (tdc - py) + tdir * px);
		}
		// OD arc to the next groove (corner points come from the groove polyline itself).
		for j in 1..=3 {
			let a = theta + delta + (pitch_angle - 2.0 * delta) * j as f64 / 4.0;
			profile.push(DVec2::new(od * 0.5 * a.cos(), od * 0.5 * a.sin()));
		}
	}

	if !flanged {
		// Bore cut as an analytic-cylinder boolean (not an extrude_with_holes hole loop):
		// loop-free caps keep the adaptive tessellation watertight → exact STL route
		// (FRICTION #6), and the bore carries the exact cylinder tag for STEP/exact_volume.
		return super::extrude_bored(&profile, belt_width, &[(DVec2::ZERO, bore_d * 0.5, 48)], &[]);
	}
	let (flange_r, flange_t) = (od * 0.5 + 1.5, 1.0);
	// Flanged: bored flange-Ø blank over the full stack height, then turn the belt channel in
	// one transverse difference. The cutter is a prism ring spanning exactly the belt band
	// (z 0…belt_width): outer wall clear outside the blank, inner wall the toothed profile —
	// so the difference leaves the grooves as the channel floor. Its end caps cut the blank's
	// lateral wall mid-height (transverse), never face-on-face. The cutter's outer boundary is
	// a SQUARE (apothem = flange_r + 2, radially clear of the blank everywhere): probing showed
	// the cap-plane arrangement duplicates a wall band when that boundary is a 48-gon running
	// exactly parallel to the blank's wall facets (same phase), while any non-parallel boundary
	// stitches watertight.
	let blank = extrude_with_holes(&circle48(flange_r), &[circle48(bore_d * 0.5)], belt_width + 2.0 * flange_t)
		.transformed(DAffine3::from_translation(DVec3::new(0.0, 0.0, -flange_t)));
	let s = flange_r + 2.0;
	let square = vec![DVec2::new(s, s), DVec2::new(-s, s), DVec2::new(-s, -s), DVec2::new(s, -s)];
	let cutter = extrude_with_holes(&square, &[profile], belt_width);
	difference(&blank, &cutter)
}

#[cfg(test)]
mod tests {
	use super::*;
	use kernel_brep::{tessellate_default, validate, volume, VertexId};

	/// Radii of all solid vertices about the +Z axis.
	fn radii(s: &Solid) -> Vec<f64> {
		(0..s.vertex_count() as u32)
			.map(|i| {
				let p = s.position(VertexId(i));
				(p.x * p.x + p.y * p.y).sqrt()
			})
			.collect()
	}

	#[test]
	fn sixteen_tooth_pulley_keeps_the_gt2_od_convention_and_groove_count() {
		// 16T unflanged: OD must be exactly 2·16/π − 0.508 across the outermost vertices, every
		// groove tip must sit 0.76447 below the mouth chord (2 tip vertices per groove: one per
		// face ring), and the body is a watertight genus-1 ring.
		let (z, w, bore) = (16usize, 9.0, 5.0);
		let p = gt2_pulley(z, w, bore, false);
		let v = validate(&p);
		let od = 2.0 * z as f64 / PI - 0.508;
		let tip_r = ((od * 0.5) * (od * 0.5) - 0.747183_f64.powi(2)).sqrt() - 0.76447;
		let rr = radii(&p);
		let r_max = rr.iter().copied().fold(0.0, f64::max);
		let tips = rr.iter().filter(|r| (**r - tip_r).abs() < 1e-6).count();
		// Volume sanity: between the groove-tip cylinder and the OD cylinder, bore removed.
		let (lo, hi) = ((PI * tip_r * tip_r - PI * 6.25) * w, (PI * od * od / 4.0 - 0.99 * PI * 6.25) * w);
		let vol = volume(&p).abs();
		assert!(
			v.closed
				&& v.manifold && v.genus == 1
				&& tessellate_default(&p).is_watertight()
				&& (r_max - od * 0.5).abs() < 1e-9
				&& tips == 2 * z
				&& vol > lo && vol < hi,
			"16T GT2: want watertight genus-1, OD {od:.4}, {} groove tips, volume in ({lo:.0},{hi:.0}); got {v:?} r_max={r_max:.4} tips={tips} vol={vol:.0}",
			2 * z
		);
	}

	#[test]
	fn belt_math_hits_the_classic_gt2_identities_and_round_trips() {
		// Equal 20T pulleys at C = 100 mm: spans 2·100 plus one full pitch circle
		// 2π·(20/π) = 40 — exactly 240 mm = the off-the-shelf 120-tooth belt. The
		// inverse must return C = 100 to machine precision from that belt; an unequal
		// 60T/20T drive must round-trip through gt2_center_distance the same way; and
		// touching/overlapping pitch circles or a too-short belt are refused.
		let (len_eq, teeth_eq) = gt2_belt(100.0, 20, 20).expect("valid drive");
		let c_eq = gt2_center_distance(120, 20, 20).expect("belt fits");
		let c_uneq = gt2_center_distance(200, 60, 20).expect("belt fits");
		let (len_uneq, teeth_uneq) = gt2_belt(c_uneq, 60, 20).expect("valid drive");
		assert!(
			(len_eq - 240.0).abs() < 1e-9
				&& teeth_eq == 120
				&& (c_eq - 100.0).abs() < 1e-9
				&& (len_uneq - 400.0).abs() < 1e-9
				&& teeth_uneq == 200
				&& gt2_belt(12.0, 20, 20).is_none() // C ≤ r1 + r2 (pitch circles overlap)
				&& gt2_belt(f64::NAN, 20, 20).is_none()
				&& gt2_center_distance(20, 20, 20).is_none() // 40 mm belt cannot wrap 40 mm of arcs
				&& gt2_center_distance(120, 1, 20).is_none(),
			"GT2 belt math: 20T/20T@100 → ({len_eq}, {teeth_eq}) want (240, 120); inverse(120T) → {c_eq} want 100; 60T/20T round-trip → C={c_uneq}, ({len_uneq}, {teeth_uneq}) want (400, 200)"
		);
	}

	#[test]
	fn flanged_twenty_tooth_pulley_fuses_to_one_genus_one_solid() {
		// 20T with both flanges: still a single genus-1 body (one transverse ring-channel
		// difference), outermost vertices on the flange Ø = OD + 3, and the flanges add
		// roughly two 1 mm discs of volume over the unflanged pulley.
		let (z, w, bore) = (20usize, 6.0, 5.0);
		let p = gt2_pulley(z, w, bore, true);
		let bare = gt2_pulley(z, w, bore, false);
		let v = validate(&p);
		let od = 2.0 * z as f64 / PI - 0.508;
		let flange_r = od * 0.5 + 1.5;
		let r_max = radii(&p).iter().copied().fold(0.0, f64::max);
		let added = volume(&p).abs() - volume(&bare).abs();
		let disc2 = 2.0 * (PI * flange_r * flange_r - PI * bore * bore / 4.0) * 1.0;
		assert!(
			v.closed && v.manifold && v.genus == 1 && tessellate_default(&p).is_watertight() && (r_max - flange_r).abs() < 1e-9 && (added - disc2).abs() / disc2 < 0.02,
			"flanged 20T GT2: want watertight genus-1 with flange radius {flange_r:.3} adding ~{disc2:.0}mm³; got {v:?} wt={} r_max={r_max:.3} added={added:.0}",
			tessellate_default(&p).is_watertight()
		);
	}
}
