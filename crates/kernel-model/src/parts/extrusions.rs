// Copyright (c) LMCAD. Licensed under the MIT License.

//! **Aluminium T-slot/V-slot extrusion stock** (the 2020/3030 profiles every printer
//! and CNC frame is built from) and the matching 20-series M5 drop-in tee nut.
//!
//! The cross-sections are **honest simplified-but-dimensionally-correct composites**
//! of the published vendor data — real extrusions add fillets, radii and per-vendor
//! lip styling that this profile intentionally omits:
//! - 2020 V-slot: 20 × 20 mm, nominal **6 mm slot** throat with 45° V lips flaring to
//!   9 mm at the face (the OpenBuilds-style V-groove; vendor drawings quote
//!   5.68–6.2 mm throats), ~11 mm internal cavity, 6 mm slot depth, Ø 4.2 mm M5-tap
//!   core. Sources: aluxprofile.com "2020 6 mm" (slot 6, depth 5.5),
//!   amazon.com/dp/B09DTL7G6X (5.68 mm V-slot throat, Ø 4.2 core, 90° V),
//!   us.openbuilds.com/v-slot-20x20-linear-rail.
//! - 3030: 30 × 30 mm, 8 mm slot with 1 mm 45° entry chamfers, 16.5 mm cavity,
//!   8 mm slot depth, Ø 6.8 mm M8-tap core (the generic 30-series numbers).
//!
//! The resulting metal areas land on the published weights (2020 ≈ 0.48 kg/m at
//! 2.7 g/cm³ against the listed 0.46–0.5; 3030 within the 30-series 1.0–1.4 kg/m
//! spread) — asserted in the tests as an external anchor.

use super::extrude_bored;
use kernel_brep::math::{DAffine3, DVec2, DVec3};
use kernel_brep::{difference, extrude_with_holes, Solid};

/// One slotted-extrusion cross-section parameter set (all mm, all half-widths
/// measured from the profile centre).
struct SlotProfile {
	/// Half of the overall square (10 for 2020).
	half: f64,
	/// Half of the slot opening at the throat (3 for the nominal 6 mm slot).
	throat_half: f64,
	/// Half of the opening at the outer face (V-slot flare / entry chamfer).
	face_half: f64,
	/// Depth of the 45° flare from the face down to the throat.
	flare_depth: f64,
	/// Depth from the face to the lip underside (total lip thickness).
	lip_depth: f64,
	/// Half of the internal cavity width.
	cavity_half: f64,
	/// Depth from the face to the slot floor.
	slot_depth: f64,
	/// Half of the flat slot floor (the diagonal webs spring from its ends).
	floor_half: f64,
	/// Central core hole diameter (the tapping bore).
	core_d: f64,
}

/// The 2020 V-slot section (see the module docs for sources).
const P2020: SlotProfile = SlotProfile {
	half: 10.0,
	throat_half: 3.0,
	face_half: 4.5,
	flare_depth: 1.5,
	lip_depth: 2.0,
	cavity_half: 5.5,
	slot_depth: 6.0,
	floor_half: 3.0,
	core_d: 4.2,
};

/// The 3030 T-slot section (see the module docs for sources).
const P3030: SlotProfile = SlotProfile {
	half: 15.0,
	throat_half: 4.0,
	face_half: 5.0,
	flare_depth: 1.0,
	lip_depth: 2.5,
	cavity_half: 8.25,
	slot_depth: 8.0,
	floor_half: 4.5,
	core_d: 6.8,
};

/// The full profile outline: the top-side vertex run (corner → slot detour → next
/// corner, traversed right-to-left so the whole outline is counter-clockwise),
/// repeated under 90° rotations for the four sides.
fn slot_outline(p: &SlotProfile) -> Vec<DVec2> {
	let h = p.half;
	let wall_y = h - p.slot_depth + (p.cavity_half - p.floor_half); // cavity wall foot (45° webs)
	let side = [
		DVec2::new(h, h), // corner
		DVec2::new(p.face_half, h),
		DVec2::new(p.throat_half, h - p.flare_depth), // 45° V flare into the throat
		DVec2::new(p.throat_half, h - p.lip_depth),   // throat land
		DVec2::new(p.cavity_half, h - p.lip_depth),   // lip underside
		DVec2::new(p.cavity_half, wall_y),            // cavity side wall
		DVec2::new(p.floor_half, h - p.slot_depth),   // 45° web diagonal
		DVec2::new(-p.floor_half, h - p.slot_depth),  // slot floor
		DVec2::new(-p.cavity_half, wall_y),
		DVec2::new(-p.cavity_half, h - p.lip_depth),
		DVec2::new(-p.throat_half, h - p.lip_depth),
		DVec2::new(-p.throat_half, h - p.flare_depth),
		DVec2::new(-p.face_half, h),
	];
	(0..4)
		.flat_map(|k| {
			let a = k as f64 * std::f64::consts::FRAC_PI_2;
			let (c, s) = (a.cos(), a.sin());
			side.iter().map(move |q| DVec2::new(q.x * c - q.y * s, q.x * s + q.y * c))
		})
		.collect()
}

/// Shared extrusion builder: outline extruded to `length` along +Z, the core hole
/// then bored by one exact boolean difference with the analytic cylinder primitive
/// (loop-free caps → adaptive tessellation watertight → exact STL route, and the
/// tap core reads back π-exact through `exact_volume`; see `parts::extrude_bored`).
fn extrusion(p: &SlotProfile, length: f64) -> Solid {
	if !(length > 0.0 && length.is_finite()) {
		return Solid::default();
	}
	extrude_bored(&slot_outline(p), length, &[(DVec2::ZERO, p.core_d * 0.5, 48)], &[])
}

/// A length of **2020 V-slot aluminium extrusion**: 20 × 20 mm, four 6 mm slots with
/// 45° V lips, Ø 4.2 mm M5-tap core, along +Z from z = 0. Genus 1 (the core bore;
/// the slots are open channels). Dimensionally-correct simplified composite
/// profile (sharp corners, no extrusion radii) — see the module docs for the cited
/// vendor numbers. Empty for degenerate lengths.
pub fn extrusion_2020(length: f64) -> Solid {
	extrusion(&P2020, length)
}

/// A length of **3030 T-slot aluminium extrusion**: 30 × 30 mm, four 8 mm slots with
/// 1 mm entry chamfers, Ø 6.8 mm M8-tap core, along +Z from z = 0. Same construction
/// and honesty notes as [`extrusion_2020`]. Empty for degenerate lengths.
pub fn extrusion_3030(length: f64) -> Solid {
	extrusion(&P3030, length)
}

/// 20-series drop-in tee-nut dimensions (mm): body length, neck width (fills the
/// 6 mm slot opening), flange width × height (slides in the ~11 mm cavity), overall
/// height. Composite of the common commercial listings (e.g. the 20-series
/// "Drop In/Roll In M5" nuts at kb-3d.com / fluxelectronix.com: ~10 × 6 × 5 with a
/// wider lower flange).
const TNUT_L: f64 = 10.0;
const TNUT_NECK_W: f64 = 5.9;
const TNUT_FLANGE_W: f64 = 9.5;
const TNUT_FLANGE_H: f64 = 2.0;
const TNUT_H: f64 = 4.5;

/// A **2020-series M5 drop-in tee nut**: the T-shaped block that drops into a 2020
/// slot — flange 9.5 × 2 sliding in the cavity, 5.9 mm neck filling the 6 mm
/// opening, 10 mm long, bored Ø 5 (the M5 thread is not modelled, project
/// convention). Built lying in its working pose: flange bottom on z = 0, neck up,
/// nut axis (and bore) along +Z through the centre at (0, 0). Genus 1. The
/// ball-spring retention dimple of the commercial nut is not modelled. The tests
/// assert it fits the [`extrusion_2020`] slot envelope.
pub fn tnut_2020() -> Solid {
	let (hl, hn, hf) = (TNUT_L * 0.5, TNUT_NECK_W * 0.5, TNUT_FLANGE_W * 0.5);
	// T cross-section in the XZ plane (x across the slot, z up), extruded along the
	// slot direction: drawn here in XY and extruded +Z = slot direction, then the
	// bore cut along the original +Y… simpler: build the T in (x, z) as the plan
	// profile with y the extrusion axis, then rotate the result so the bore is +Z.
	let profile = vec![
		DVec2::new(hf, 0.0),
		DVec2::new(hf, TNUT_FLANGE_H),
		DVec2::new(hn, TNUT_FLANGE_H),
		DVec2::new(hn, TNUT_H),
		DVec2::new(-hn, TNUT_H),
		DVec2::new(-hn, TNUT_FLANGE_H),
		DVec2::new(-hf, TNUT_FLANGE_H),
		DVec2::new(-hf, 0.0),
	];
	// Extrude the T along +Z (length axis), then rotate +90° about X — a proper
	// rotation, (x, y, z) → (x, −z, y) — so the part stands flange-down on z = 0
	// with its length centred on y and the bore axis vertical.
	let bar = extrude_with_holes(&profile, &[], TNUT_L).transformed(DAffine3::from_cols(
		DVec3::new(1.0, 0.0, 0.0),
		DVec3::new(0.0, 0.0, 1.0),
		DVec3::new(0.0, -1.0, 0.0),
		DVec3::new(0.0, hl, 0.0),
	));
	let bore = kernel_brep::cylinder(DVec3::new(0.0, 0.0, -1.0), DVec3::Z, 2.5, TNUT_H + 2.0, 32);
	difference(&bar, &bore)
}

#[cfg(test)]
mod tests {
	use super::*;
	use kernel_brep::{tessellate_default, validate, volume, VertexId};

	/// `(min, max)` corner of the solid's vertex bounding box.
	fn bbox(s: &Solid) -> (DVec3, DVec3) {
		let mut lo = DVec3::splat(f64::INFINITY);
		let mut hi = DVec3::splat(f64::NEG_INFINITY);
		for i in 0..s.vertex_count() as u32 {
			let p = s.position(VertexId(i));
			lo = lo.min(p);
			hi = hi.max(p);
		}
		(lo, hi)
	}

	#[test]
	fn extrusion_profiles_hit_their_published_weight_bands() {
		// 100 mm sticks of 2020 and 3030. Volume/length is the exact metal area of
		// the prism; at 2.7 g/cm³ the published listings give 0.46–0.5 kg/m for
		// V-slot 2020 (→ 170–185 mm²) and ~1.0–1.4 kg/m for 30-series 3030
		// (→ 370–520 mm²); both must be genus 1 (core bore), watertight, exactly
		// their nominal envelope, and slot throats must sit at the nominal half-width.
		let checks = [
			("2020", extrusion_2020(100.0), 10.0, 3.0, 160.0, 195.0),
			("3030", extrusion_3030(100.0), 15.0, 4.0, 370.0, 520.0),
		];
		for (label, s, half, throat_half, area_lo, area_hi) in checks {
			let v = validate(&s);
			let (lo, hi) = bbox(&s);
			let area = volume(&s).abs() / 100.0;
			let throat_verts = (0..s.vertex_count() as u32)
				.map(|i| s.position(VertexId(i)))
				.filter(|p| (p.x.abs() - throat_half).abs() < 1e-9 && p.y.abs() > half - 2.6)
				.count();
			assert!(
				v.closed
					&& v.manifold && v.genus == 1
					&& tessellate_default(&s).is_watertight()
					&& (lo.x + half).abs() < 1e-9 && (hi.x - half).abs() < 1e-9
					&& (lo.y + half).abs() < 1e-9 && (hi.y - half).abs() < 1e-9
					&& area > area_lo && area < area_hi
					&& throat_verts > 0,
				"{label}: want watertight genus-1, ±{half} envelope, metal area in ({area_lo}, {area_hi}) mm², throat vertices at ±{throat_half}; got {v:?} bbox=({lo:?},{hi:?}) area={area:.1} throats={throat_verts}"
			);
		}
	}

	#[test]
	fn tee_nut_is_a_bored_t_block_that_fits_the_2020_slot() {
		// The nut must be watertight genus 1 with volume = T-section area × length −
		// Ø5 bore (1% for the 32-gon bore), and every fit dimension must clear the
		// 2020 slot envelope it is sold for: neck through the 6 mm throat, flange in
		// the 11 mm cavity, flange under the lip cavity height, nut under the slot
		// depth — checked against the same constants `extrusion_2020` is built from.
		let nut = tnut_2020();
		let v = validate(&nut);
		let (lo, hi) = bbox(&nut);
		let t_area = TNUT_FLANGE_W * TNUT_FLANGE_H + TNUT_NECK_W * (TNUT_H - TNUT_FLANGE_H);
		let expected = t_area * TNUT_L - std::f64::consts::PI * 2.5 * 2.5 * TNUT_H;
		let fits = TNUT_NECK_W < 2.0 * P2020.throat_half
			&& TNUT_FLANGE_W < 2.0 * P2020.cavity_half
			&& TNUT_H < P2020.slot_depth
			&& TNUT_FLANGE_H < P2020.slot_depth - P2020.lip_depth;
		assert!(
			v.closed
				&& v.manifold && v.genus == 1
				&& tessellate_default(&nut).is_watertight()
				&& fits
				&& (hi.z - TNUT_H).abs() < 1e-9 && lo.z.abs() < 1e-9
				&& (volume(&nut).abs() - expected).abs() / expected < 0.01,
			"M5 tee nut: want watertight genus-1, fits=(true) the 2020 slot, ~{expected:.1}mm³; got {v:?} fits={fits} z=[{},{}] vol={:.1}",
			lo.z,
			hi.z,
			volume(&nut).abs()
		);
	}
}
