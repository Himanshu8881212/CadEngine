// Copyright (c) LMCAD. Licensed under the MIT License.

//! **Compression springs**: a round wire swept along a helix. Open ends (plain, not
//! closed-and-ground); the wire circle is a 16-gon, so the path sweep's rotation-minimising
//! frame — which precesses around a helix by 2π·sin(lead angle) per turn — is harmless here
//! (a rotated circle is the same circle), unlike for the oriented thread profile in
//! [`super::threads`].

use kernel_brep::math::DVec3;
use kernel_brep::{sweep_solid, Solid};
use std::f64::consts::TAU;

/// A **compression spring**: wire of `wire_d` diameter wound into a helix of `outer_d` outside
/// diameter (coil centreline at `(outer_d − wire_d)/2`), the given coil `pitch` (axial advance
/// per turn) and `active_turns` turns, starting at z = wire_d/2 so the body sits on z = 0.
/// Plain open ends (flat caps perpendicular to the wire, not closed or ground). Returns a
/// closed, manifold, genus-0 solid; `None` for degenerate input — including `pitch ≤ wire_d`,
/// where adjacent coils would touch and the swept solid would self-intersect.
pub fn compression_spring(wire_d: f64, outer_d: f64, pitch: f64, active_turns: f64) -> Option<Solid> {
	let rw = wire_d * 0.5;
	let rh = (outer_d - wire_d) * 0.5; // helix (coil centreline) radius
	if !(rw > 0.0 && rh > rw && active_turns > 0.0) || pitch <= wire_d {
		return None;
	}
	let steps_per_turn = 48;
	let n = (active_turns * steps_per_turn as f64).round() as usize;
	if n < 1 {
		// A sub-step fractional turn count rounds to 0; `path[1]` below would then
		// index out of bounds. Degenerate input -> None (as documented).
		return None;
	}
	let path: Vec<DVec3> = (0..=n)
		.map(|k| {
			let t = k as f64 / steps_per_turn as f64;
			let a = t * TAU;
			DVec3::new(rh * a.cos(), rh * a.sin(), rw + t * pitch)
		})
		.collect();
	// Wire section: a 16-gon in the plane perpendicular to the starting tangent, wound
	// counter-clockwise about the path direction (the loft_solid convention).
	let t0 = (path[1] - path[0]).normalize();
	let e1 = (DVec3::X - t0 * t0.x).normalize(); // radial-ish in-plane axis
	let e2 = t0.cross(e1);
	let profile: Vec<DVec3> = (0..16)
		.map(|i| {
			let a = TAU * i as f64 / 16.0;
			path[0] + (e1 * a.cos() + e2 * a.sin()) * rw
		})
		.collect();
	sweep_solid(&profile, &path)
}

#[cfg(test)]
mod tests {
	use super::*;
	use kernel_brep::{tessellate_default, validate, volume};
	use std::f64::consts::PI;

	#[test]
	fn springs_are_watertight_tubes_of_the_swept_wire_volume() {
		// Two parameter sets. Volume ≈ 16-gon wire area × helix arc length (Pappus for a swept
		// tube; 5% covers the polygonal path/section discretisation), and the coil must reach
		// its outside diameter.
		for (wire_d, outer_d, pitch, turns) in [(2.0, 16.0, 6.0, 5.0), (1.5, 10.0, 4.0, 6.5)] {
			let s = compression_spring(wire_d, outer_d, pitch, turns).expect("non-degenerate spring");
			let v = validate(&s);
			let rw = wire_d * 0.5;
			let rh = (outer_d - wire_d) * 0.5;
			let area = 16.0 * 0.5 * rw * rw * (2.0 * PI / 16.0).sin();
			let len = turns * ((2.0 * PI * rh).powi(2) + pitch * pitch).sqrt();
			let expected = area * len;
			let vol = volume(&s).abs();
			assert!(
				v.closed && v.manifold && v.genus == 0 && tessellate_default(&s).is_watertight() && (vol - expected).abs() / expected < 0.05,
				"spring Ø{wire_d}/Ø{outer_d} p{pitch} × {turns}: want watertight genus-0 ~{expected:.0}mm³; got {v:?} wt={} vol={vol:.0}",
				tessellate_default(&s).is_watertight()
			);
		}
	}

	#[test]
	fn touching_coils_are_refused_instead_of_self_intersecting() {
		// pitch ≤ wire_d would make adjacent coils touch/overlap — the sweep cannot produce a
		// valid solid there, so the function must refuse rather than return garbage.
		assert!(
			compression_spring(2.0, 16.0, 2.0, 5.0).is_none() && compression_spring(2.0, 16.0, 1.5, 5.0).is_none(),
			"coil-bound springs must return None"
		);
	}
}
