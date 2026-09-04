// Copyright (c) LMCAD. Licensed under the MIT License.

//! Modelled **ISO metric threads**: the helical ridge as a watertight B-rep solid, plus a
//! ready-made threaded hex bolt (body + thread pair). The thread cross-section is the ISO 68-1
//! basic profile; coarse pitches come from the ISO 261/262 table.

use super::fasteners::iso4017_head;
use super::hexagon_across_flats;
use kernel_brep::math::{DAffine3, DVec3};
use kernel_brep::{cylinder, extrude, loft_solid, union, Solid};
use std::f64::consts::TAU;

/// ISO 261/262 **coarse** thread pitches, `(nominal Ø d, pitch P)` in mm, M3–M16.
/// Source: ISO 261 general-purpose metric screw thread table (also printed as the `P` column of
/// the DIN 912 table at fasteners.eu/standards/din/912).
const ISO_COARSE: [(f64, f64); 8] = [(3.0, 0.5), (4.0, 0.7), (5.0, 0.8), (6.0, 1.0), (8.0, 1.25), (10.0, 1.5), (12.0, 1.75), (16.0, 2.0)];

/// The ISO 261 coarse pitch for a nominal thread Ø `m` (3, 4, 5, 6, 8, 10, 12, 16), in mm.
/// `None` for sizes outside the table.
pub fn iso_coarse_pitch(m: f64) -> Option<f64> {
	ISO_COARSE.iter().find(|(d, _)| (d - m).abs() < 1e-9).map(|&(_, p)| p)
}

/// An external **ISO metric thread ridge** as a closed, watertight B-rep solid: the ISO 68-1
/// basic profile swept along an exact helix of the given `pitch`, with crests at `major_d`
/// diameter, starting at height `z0` and spanning `length` axially (`length/pitch` turns).
/// Returns `None` for degenerate input (non-positive sizes, or fewer than two turns' worth of
/// path points).
///
/// Geometry, per ISO 68-1 with H = (√3/2)·P:
/// - crest flat P/8 wide at `major_d/2` (the sharp profile truncated by H/8);
/// - 60° included flank angle (each flank at 30° to the thread axis);
/// - root flat P/4 wide at `major_d/2 − (5/8)H` — adjacent turns tile exactly to the pitch;
/// - below the root the section drops straight by an extra `P/4` so the ridge overlaps the
///   minor-diameter shank it is meant to fuse with (see [`threaded_hex_bolt`]).
///
/// The section is planted in the **axial plane** at every helix station (as a lathe tool would
/// cut it) and stitched with [`loft_solid`] — deliberately *not* a rotation-minimising path
/// sweep, whose frame precesses around a helix by 2π·sin(lead angle) per turn and would slowly
/// tilt the profile. Crests therefore sit at exactly `major_d/2` along the whole ridge.
///
/// **Fusion route** (why this returns a separate solid): the ridge pierces the shank wall, so
/// the exact B-rep `union(body, thread)` self-intersects and no exact arrangement can stitch
/// it. Fuse the pair through the voxel half instead: merge the two tessellations and heal via
/// [`crate::watertight_mesh_of`] (winding-number SDF → manifold dual contouring), which is the
/// proven hybrid route for self-intersecting unions.
pub fn iso_thread_solid(major_d: f64, pitch: f64, z0: f64, length: f64) -> Option<Solid> {
	// NaN-safe rejection: `!(x > 0)` (not `x <= 0`) so NaN inputs are refused too.
	if !(major_d > 0.0 && pitch > 0.0 && length > 0.0 && major_d.is_finite() && pitch.is_finite() && length.is_finite()) {
		return None;
	}
	let h = 3.0_f64.sqrt() * 0.5 * pitch; // ISO 68-1 fundamental triangle height
	let r_crest = major_d * 0.5;
	let r_root = r_crest - 0.625 * h; // root-flat radius (5/8·H below the crest)
	let r_buried = r_root - 0.25 * pitch; // burial depth so the ridge overlaps its shank
	if r_buried <= 0.0 {
		return None;
	}
	// The ISO section in the axial (radius, z-offset) plane. Wound so the lofted ridge faces
	// outward (counter-clockwise about the helix direction): buried base → root flat → crest
	// flat → back, the same rotational sense as the proven thread sweep in `hybrid_showcase`.
	// Half-widths: crest P/16; root P/16 + (5/8)H·tan30° = 3P/8 (turns tile to the pitch).
	let section: [(f64, f64); 6] = [
		(r_buried, 0.375 * pitch),
		(r_root, 0.375 * pitch),
		(r_crest, 0.0625 * pitch),
		(r_crest, -0.0625 * pitch),
		(r_root, -0.375 * pitch),
		(r_buried, -0.375 * pitch),
	];

	let steps_per_turn = 96;
	let n = (length / pitch * steps_per_turn as f64).round() as usize;
	if n < 2 {
		return None;
	}
	let sections: Vec<Vec<DVec3>> = (0..=n)
		.map(|k| {
			let t = k as f64 / steps_per_turn as f64; // turns travelled
			let (a, z) = (t * TAU, z0 + t * pitch);
			section.iter().map(|&(radius, dz)| DVec3::new(radius * a.cos(), radius * a.sin(), z + dz)).collect()
		})
		.collect();
	loft_solid(&sections)
}

/// A **threaded ISO 4017 hex bolt** for nominal Ø `m` (M3–M16, coarse pitch) and shank
/// `length`: returns the pair `(body, thread)` of watertight solids.
///
/// - `body` — the exact assembly body: a **minor-diameter** shank (Ø = m − 1.25·H, the ISO
///   root-flat diameter) of `length`, with the ISO 4017 hex head (across-flats and head height
///   from the table) stacked coplanar on top. Closed, manifold, genus 0.
/// - `thread` — the [`iso_thread_solid`] ridge with crests at the nominal Ø `m`, wound from
///   half a pitch above the tip to half a pitch below the head (full-thread ISO 4017 style),
///   its root buried P/4 into the shank so the two bodies overlap.
///
/// They are returned **unfused** because the exact union self-intersects (the ridge pierces
/// the shank wall — no planar arrangement can stitch a self-intersection). Routes to one
/// printable solid, in order of fidelity:
/// 1. merge the two tessellations and heal via [`crate::watertight_mesh_of`] (winding-number
///    SDF → manifold dual contouring) — the proven hybrid fuse from `hybrid_showcase`;
/// 2. keep them separate for display/BOM use — every dimension is already exact.
///
/// `None` for sizes outside the M3–M16 table.
pub fn threaded_hex_bolt(m: f64, length: f64) -> Option<(Solid, Solid)> {
	let pitch = iso_coarse_pitch(m)?;
	let (af, head_h) = iso4017_head(m)?;
	let h = 3.0_f64.sqrt() * 0.5 * pitch;
	let r_root = m * 0.5 - 0.625 * h;
	let shank = cylinder(DVec3::ZERO, DVec3::Z, r_root, length, 48);
	let head = extrude(&hexagon_across_flats(af), head_h).transformed(DAffine3::from_translation(DVec3::new(0.0, 0.0, length)));
	let body = union(&shank, &head);
	// Threaded span inset half a pitch from tip and head so the ridge ends stay clear of the
	// end face and the head seat.
	let thread = iso_thread_solid(m, pitch, 0.5 * pitch, length - pitch)?;
	Some((body, thread))
}

#[cfg(test)]
mod tests {
	use super::*;
	use kernel_brep::{tessellate_adaptive_tol, validate, volume, VertexId};
	use std::f64::consts::PI;

	#[test]
	fn iso_thread_ridge_is_watertight_with_crests_exactly_at_the_major_diameter() {
		// M10×1.5: crest radius must be exactly 5.0 on every crest vertex (the axial-plane
		// section construction guarantees it — a rotation-minimising sweep would precess), the
		// root flat at 5 − 0.625·H, the buried base P/4 deeper, and the loft watertight.
		let (d, p) = (10.0, 1.5);
		let t = iso_thread_solid(d, p, 1.0, 12.0).expect("thread lofts");
		let v = validate(&t);
		let mesh = tessellate_adaptive_tol(&t, 0.01);
		let radii: Vec<f64> = (0..t.vertex_count() as u32)
			.map(|i| {
				let q = t.position(VertexId(i));
				(q.x * q.x + q.y * q.y).sqrt()
			})
			.collect();
		let r_max = radii.iter().copied().fold(0.0, f64::max);
		let r_min = radii.iter().copied().fold(f64::INFINITY, f64::min);
		let r_buried = d * 0.5 - 0.625 * (3.0_f64.sqrt() * 0.5 * p) - 0.25 * p;
		assert!(
			v.closed && v.manifold && v.genus == 0 && mesh.is_watertight() && (r_max - d * 0.5).abs() < 1e-9 && (r_min - r_buried).abs() < 1e-9,
			"M10 ridge: want watertight genus-0 with crest radius exactly {} and base {r_buried:.4}; got {v:?} wt={} r_max={r_max} r_min={r_min}",
			d * 0.5,
			mesh.is_watertight()
		);
	}

	#[test]
	fn threaded_hex_bolt_bodies_are_valid_and_match_the_iso_tables() {
		// M10×30: body = Ø(10 − 1.25·H) root shank + AF16 × 6.4 head per ISO 4017; thread spans
		// 0.75..29.25 with crests at Ø10. Both solids watertight; body volume analytic to 1%.
		let (m, len) = (10.0, 30.0);
		let (body, thread) = threaded_hex_bolt(m, len).expect("M10 is in the tables");
		let vb = validate(&body);
		let vt = validate(&thread);
		let h = 3.0_f64.sqrt() * 0.5 * 1.5;
		let r_root = m * 0.5 - 0.625 * h;
		let expected = PI * r_root * r_root * len + crate::parts::hexagon_area(16.0) * 6.4;
		let crest = (0..thread.vertex_count() as u32)
			.map(|i| {
				let q = thread.position(VertexId(i));
				(q.x * q.x + q.y * q.y).sqrt()
			})
			.fold(0.0, f64::max);
		assert!(
			vb.closed
				&& vb.manifold
				&& vb.genus == 0
				&& (volume(&body).abs() - expected).abs() / expected < 0.01
				&& vt.closed && vt.manifold
				&& tessellate_adaptive_tol(&thread, 0.01).is_watertight()
				&& (crest - m * 0.5).abs() < 1e-9
				&& iso_coarse_pitch(m) == Some(1.5),
			"M10×30 bolt: body {vb:?} vol={:.0} (want ~{expected:.0}); thread {vt:?} crest={crest}",
			volume(&body).abs()
		);
	}
}
