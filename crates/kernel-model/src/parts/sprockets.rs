// Copyright (c) LMCAD. Licensed under the MIT License.

//! **Roller-chain sprockets** to the ANSI/ASA B29.1 tooth form. The tooth-space geometry
//! follows the American Chain Association construction (as reproduced in GEARS-EdS, "Designing
//! and Drawing a Sprocket", gearseds.com/files/design_draw_sprocket_5.pdf): per tooth space a
//! **seating curve** (radius R about the roller seat on the pitch circle), a **transitional
//! curve** (radius E, internally tangent to the seat), a short straight flank, and a convex
//! **topping curve** (radius F centred 1.4·Dr along the chord to the next seat), truncated at
//! the B29.1 maximum outside diameter `P·(0.6 + cot(180°/N))` into a tip land.

use kernel_brep::math::DVec2;
use kernel_brep::Solid;
use std::f64::consts::PI;

/// 0.0015 in — the seating/topping-curve clearance constant of the ANSI formulas, which are
/// defined in inches — converted once to mm.
const IN_0015: f64 = 0.0015 * 25.4;

/// An **ANSI/ASA B29.1 roller-chain sprocket**: chain `pitch` P and nominal roller diameter
/// `roller_d` Dr in mm (e.g. 6.35 / 3.302 for #25, 9.525 / 5.08 for #35), `teeth` N, bored at
/// `bore_d`. The roller seats lie on the exact pitch circle `PD = P / sin(180°/N)`.
///
/// Tooth form per the ACA/ANSI formulas (inch constants converted; angles in degrees):
/// - seating curve `R = 0.5025·Dr + 0.0015″` spanning `90° − A` each side of the gap bottom,
///   `A = 35° + 60°/N`;
/// - transitional curve `E = 1.3025·Dr + 0.0015″` centred `0.8·Dr` from the seat (internally
///   tangent to it), swept through `B = 18° − 56°/N`;
/// - straight flank `yz = Dr·(1.4·sin(17° − 64°/N) − 0.8·sin B)` tangent to both arcs;
/// - topping curve `F = Dr·(0.8·cos B + 1.4·cos(17° − 64°/N) − 1.3025) − 0.0015″` centred
///   `1.4·Dr` along the chord toward the next seat, **truncated at the B29.1 maximum OD**
///   `P·(0.6 + cot(180°/N))` into a flat-topped tip land (the full topping curve would peak at
///   `PD/2·cos(180°/N) + H` slightly above it; real sprockets are turned to the OD).
///
/// Face width is auto-sized to the B29.1 single-strand tooth `0.93·W − 0.006″` with the chain
/// inner width `W` taken as `P/2` — exact for the small rollerless sizes (#25, #35) this
/// targets, slightly narrow for #40 and up (whose W is wider than P/2).
///
/// Honest deviations from a production sprocket: the published rounded constants make the
/// three curves tangent only to ~µm (the standard itself instructs "force tangency"; we snap
/// the topping-arc start onto its circle), arcs are sampled into a polygon (seat 6 / topping 5
/// segments per flank), and there is no shaft-key or hub boss — just the plate with a plain
/// bore. The caller keeps `bore_d/2` well inside the root circle `PD/2 − R`.
pub fn chain_sprocket(pitch: f64, roller_d: f64, teeth: usize, bore_d: f64) -> Solid {
	let (p, dr, n) = (pitch, roller_d, teeth as f64);
	if !(p > 0.0 && dr > 0.0) || teeth < 6 {
		return Solid::default();
	}
	let deg = PI / 180.0;
	let rp = p / (2.0 * (PI / n).sin()); // pitch radius: PD = P/sin(180°/N)
	let r = 0.5025 * dr + IN_0015; // seating curve radius
	let a_ang = (35.0 + 60.0 / n) * deg;
	let b_ang = (18.0 - 56.0 / n) * deg;
	let e = 1.3025 * dr + IN_0015; // = R + 0.8·Dr exactly (internal tangency)
	let yz = dr * (1.4 * ((17.0 - 64.0 / n) * deg).sin() - 0.8 * b_ang.sin());
	let f = dr * (0.8 * b_ang.cos() + 1.4 * ((17.0 - 64.0 / n) * deg).cos() - 1.3025) - IN_0015;
	let ro = p * (0.6 + 1.0 / (PI / n).tan()) * 0.5; // B29.1 maximum OD / 2

	// --- One gap-to-tooth ascent in seat-local coordinates (u = CCW tangential, v = radial
	// out, origin on the pitch circle at the roller seat centre; sprocket centre at (0, −rp)).
	let mut ascent: Vec<DVec2> = Vec::with_capacity(20);
	// Seating arc: from the gap bottom (−90°) up the CCW side to −A.
	for j in 0..=6 {
		let phi = -PI * 0.5 + (PI * 0.5 - a_ang) * j as f64 / 6.0;
		ascent.push(DVec2::new(r * phi.cos(), r * phi.sin()));
	}
	// Transitional arc about c (internally tangent to the seat at the point above, since
	// |c| = E − R), swept through B.
	let c = DVec2::new(-a_ang.cos(), a_ang.sin()) * (0.8 * dr);
	for j in 1..=4 {
		let phi = -a_ang + b_ang * j as f64 / 4.0;
		ascent.push(c + DVec2::new(e * phi.cos(), e * phi.sin()));
	}
	// Straight flank: tangent to the transitional arc at its end, length yz.
	let phi_y = b_ang - a_ang;
	let z = *ascent.last().expect("transitional points") + DVec2::new(-phi_y.sin(), phi_y.cos()) * yz;
	ascent.push(z);
	// Topping arc about b (1.4·Dr along the chord toward the next seat). The rounded standard
	// constants leave z a few µm off the F-circle, so the arc start is snapped onto it (the
	// construction text itself says to force the tangency).
	let b_c = DVec2::new((PI / n).cos(), -(PI / n).sin()) * (1.4 * dr);
	let centre = DVec2::new(0.0, -rp);
	let h = (f * f - (1.4 * dr - p * 0.5) * (1.4 * dr - p * 0.5)).max(0.0).sqrt();
	let tip = centre + DVec2::new((PI / n).sin(), (PI / n).cos()) * (rp * (PI / n).cos() + h);
	let psi0 = {
		let d = z - b_c;
		d.y.atan2(d.x)
	};
	let psi_tip = {
		let d = tip - b_c;
		d.y.atan2(d.x)
	};
	let wrap = |a: f64| {
		let mut a = a % (2.0 * PI);
		if a > PI {
			a -= 2.0 * PI;
		} else if a < -PI {
			a += 2.0 * PI;
		}
		a
	};
	let sweep_tip = wrap(psi_tip - psi0);
	// Truncation: earliest crossing of the F-arc with the OD circle, if any.
	let to_centre = centre - b_c;
	let d_ob = to_centre.length();
	let m_along = (d_ob * d_ob + ro * ro - f * f) / (2.0 * d_ob);
	let perp2 = ro * ro - m_along * m_along;
	let mut sweep_end = sweep_tip;
	let mut land_from = None;
	if perp2 > 0.0 {
		let u_hat = -to_centre / d_ob; // centre → b direction
		let v_hat = DVec2::new(-u_hat.y, u_hat.x);
		for s in [-1.0, 1.0] {
			let pc = centre + u_hat * m_along + v_hat * (s * perp2.sqrt());
			let d = pc - b_c;
			let sw = wrap(d.y.atan2(d.x) - psi0);
			if sw.signum() == sweep_tip.signum() && sw.abs() < sweep_end.abs() {
				sweep_end = sw;
				land_from = Some(pc);
			}
		}
	}
	for j in 1..=5 {
		let phi = psi0 + sweep_end * j as f64 / 5.0;
		ascent.push(b_c + DVec2::new(f * phi.cos(), f * phi.sin()));
	}
	if let Some(pc) = land_from {
		// Tip land: along the OD circle from the truncation point to the tooth centreline.
		let from = (pc.y + rp).atan2(pc.x);
		let to = PI * 0.5 - PI / n;
		for j in 1..=2 {
			let phi = from + (to - from) * j as f64 / 2.0;
			ascent.push(centre + DVec2::new(ro * phi.cos(), ro * phi.sin()));
		}
	}

	// --- Assemble all N pitches: ascent (seat k → tooth tip), then the mirrored descent into
	// seat k+1 (same list, u-mirrored in the next seat's frame, reversed, sans the shared tip
	// point and the next seat's bottom point).
	let mut profile: Vec<DVec2> = Vec::with_capacity(teeth * 2 * ascent.len());
	let plant = |k: usize, q: DVec2| {
		let theta = 2.0 * PI * k as f64 / n;
		let (rdir, tdir) = (DVec2::new(theta.cos(), theta.sin()), DVec2::new(-theta.sin(), theta.cos()));
		rdir * (rp + q.y) + tdir * q.x
	};
	for k in 0..teeth {
		for &q in &ascent {
			profile.push(plant(k, q));
		}
		for &q in ascent[1..ascent.len() - 1].iter().rev() {
			profile.push(plant((k + 1) % teeth, DVec2::new(-q.x, q.y)));
		}
	}

	// B29.1 single-strand tooth width 0.93·W − 0.006″ with W ≈ P/2 (see doc comment).
	// Bore cut as an analytic-cylinder boolean (not an extrude_with_holes hole loop):
	// loop-free caps keep the adaptive tessellation watertight → exact STL route
	// (FRICTION #6), and the bore carries the exact cylinder tag for STEP/exact_volume.
	let width = 0.93 * p * 0.5 - 0.006 * 25.4;
	super::extrude_bored(&profile, width, &[(DVec2::ZERO, bore_d * 0.5, 48)], &[])
}

#[cfg(test)]
mod tests {
	use super::*;
	use kernel_brep::{tessellate_default, validate, volume, VertexId};

	/// Validate one sprocket geometrically: watertight genus-1; the gap bottoms must measure
	/// the exact pitch circle (deepest non-bore vertices at `PD/2 − R`, one per seat per face
	/// ring); the tips must sit exactly on the truncated B29.1 outside diameter; volume between
	/// the root and OD cylinders.
	fn check_sprocket(label: &str, s: &Solid, p: f64, dr: f64, teeth: usize, bore_d: f64) {
		let n = teeth as f64;
		let pd = p / (PI / n).sin();
		let r = 0.5025 * dr + 0.0015 * 25.4;
		let ro = p * (0.6 + 1.0 / (PI / n).tan()) * 0.5;
		let width = 0.93 * p * 0.5 - 0.006 * 25.4;
		let v = validate(s);
		let outside: Vec<f64> = (0..s.vertex_count() as u32)
			.map(|i| {
				let q = s.position(VertexId(i));
				(q.x * q.x + q.y * q.y).sqrt()
			})
			.filter(|&radius| radius > bore_d * 0.5 + 0.5)
			.collect();
		let seat_floor = outside.iter().copied().fold(f64::INFINITY, f64::min);
		let r_max = outside.iter().copied().fold(0.0, f64::max);
		let seats = outside.iter().filter(|&&radius| (radius - seat_floor).abs() < 1e-9).count();
		let measured_pd = 2.0 * (seat_floor + r);
		let (lo, hi) = ((PI * (pd * 0.5 - r).powi(2) - PI * bore_d * bore_d / 4.0) * width, PI * ro * ro * width);
		let vol = volume(s).abs();
		assert!(
			v.closed
				&& v.manifold && v.genus == 1
				&& tessellate_default(s).is_watertight()
				&& (measured_pd - pd).abs() < 1e-9
				&& seats == 2 * teeth
				&& (r_max - ro).abs() < 1e-9
				&& vol > lo && vol < hi,
			"{label}: want watertight genus-1, measured PD {pd:.6} (P/sin(180/N)), {} seat floors, OD/2 {ro:.4}, volume in ({lo:.0},{hi:.0}); got {v:?} wt={} pd={measured_pd:.6} seats={seats} r_max={r_max:.4} vol={vol:.0}",
			2 * teeth,
			tessellate_default(s).is_watertight()
		);
	}

	#[test]
	fn number25_eighteen_tooth_sprocket_holds_the_b291_pitch_circle() {
		// #25 chain: P = 6.35 mm (1/4″), Dr = 3.302 mm (0.130″ bushing), 18 teeth, Ø8 bore.
		let s = chain_sprocket(6.35, 3.302, 18, 8.0);
		check_sprocket("#25 z18", &s, 6.35, 3.302, 18, 8.0);
	}

	#[test]
	fn number35_eleven_tooth_sprocket_holds_the_b291_pitch_circle() {
		// #35 chain: P = 9.525 mm (3/8″), Dr = 5.08 mm (0.200″ bushing), 11 teeth, Ø10 bore —
		// a small-tooth-count sprocket where the tooth form is strongly curved.
		let s = chain_sprocket(9.525, 5.08, 11, 10.0);
		check_sprocket("#35 z11", &s, 9.525, 5.08, 11, 10.0);
	}
}
