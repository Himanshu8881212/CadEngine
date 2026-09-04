// Copyright (c) LMCAD. Licensed under the MIT License.

//! **Tr8 trapezoidal lead screws (DIN 103 / ISO 2904)** — the 3D-printer Z-axis
//! family: the screw body, the true helical **trapezoidal thread ridge** (30°
//! flanks, multi-start), the ubiquitous flanged brass nut, and the printed
//! **nut-trap** feature cut. The thread-form numbers are the DIN 103 basic
//! profile (cited at [`tr8_spec`]); the nut envelope is the de-facto commercial
//! flanged nut (cited at its `const`s).

use kernel_brep::geom::perp_basis;
use kernel_brep::holes::{clearance_hole, Fit};
use kernel_brep::math::{DAffine3, DMat3, DVec2, DVec3};
use kernel_brep::{chamfered_cylinder, cylinder, difference, loft_solid, revolve, Solid};
use std::f64::consts::TAU;

/// One DIN 103 trapezoidal-thread parameter set (all mm).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TrapezoidalSpec {
	/// Nominal (major) screw diameter d.
	pub d: f64,
	/// Thread pitch P (axial crest-to-crest, regardless of starts).
	pub pitch: f64,
	/// Lead (axial advance per revolution) = `pitch · starts`.
	pub lead: f64,
	/// Number of thread starts.
	pub starts: usize,
	/// Pitch diameter d2 = d − 0.5·P.
	pub d2: f64,
	/// Screw minor diameter d3 = d − 2·(0.5·P + ac), ac = 0.25 for P = 2.
	pub d3: f64,
	/// Nut minor diameter D1 = d − P.
	pub nut_d1: f64,
	/// Nut major diameter D4 = d + 2·ac.
	pub nut_d4: f64,
}

/// The **Tr8×P2** spec for a `lead` of 2 (single start), 4 (two starts) or 8
/// (four starts — the printer Z screw). Source: DIN 103 / ISO 2904 basic
/// profile for Tr8×2 — 30° included flank angle, H1 = 0.5·P = 1.0, clearance
/// ac = 0.25, hence d2 = 7.0, d3 = 5.5, D1 = 6.0, D4 = 8.5; multi-start
/// variants share the P2 section and differ only in lead/starts. `None` for
/// any other lead.
pub fn tr8_spec(lead: f64) -> Option<TrapezoidalSpec> {
	let starts = match lead {
		l if (l - 2.0).abs() < 1e-9 => 1,
		l if (l - 4.0).abs() < 1e-9 => 2,
		l if (l - 8.0).abs() < 1e-9 => 4,
		_ => return None,
	};
	Some(TrapezoidalSpec { d: 8.0, pitch: 2.0, lead, starts, d2: 7.0, d3: 5.5, nut_d1: 6.0, nut_d4: 8.5 })
}

/// A **Tr8 lead-screw body** along +Z from z = 0: the exact assembly/clearance
/// solid at the major Ø8 with a half-pitch 45° entry chamfer on the top end
/// (analytic cylinder + cone tags). Per the catalog convention the thread form
/// is **not modelled on the body** — Ø8 is the true outer envelope a Tr8 screw
/// sweeps; cut nut traps and bearing bores against it directly. For the true
/// helical form, fuse [`tr8_thread_ridge`] starts onto a Ø`d3` core through the
/// voxel half (same route as `threaded_hex_bolt`). The DIN 103 numbers ride on
/// [`tr8_spec`]. `None` for an unsupported lead or degenerate length.
pub fn lead_screw_tr8(length: f64, lead: f64) -> Option<Solid> {
	let spec = tr8_spec(lead)?;
	if !(length > spec.pitch && length.is_finite()) {
		return None;
	}
	Some(chamfered_cylinder(spec.d * 0.5, length, spec.pitch * 0.5, 48))
}

/// One start of the **true Tr8 trapezoidal thread ridge** (DIN 103 basic
/// profile, 30° included angle): crest flat 0.366·P wide exactly at Ø8, flanks
/// at 15° to the thread axis, root flat at Ø`d3` = 5.5, base buried a further
/// P/4 so the ridge overlaps the Ø5.5 core it fuses onto. The section is
/// planted in the axial plane at every helix station (lathe-tool convention,
/// like `iso_thread_solid` — no frame precession) and lofted along a helix
/// advancing `lead` per turn; `start_index` phases the ridge by
/// `start_index/starts` of a turn, so the 1/2/4 starts of [`tr8_spec`]
/// interleave at exactly one pitch axially — the tests prove adjacent starts
/// never touch. Fusion route: exact `union` self-intersects (the ridge pierces
/// the core wall); merge tessellations and heal via
/// [`crate::watertight_mesh_of`], the proven hybrid route. `None` for an
/// unsupported lead, `start_index ≥ starts`, or fewer than two helix stations.
pub fn tr8_thread_ridge(lead: f64, start_index: usize, z0: f64, length: f64) -> Option<Solid> {
	let spec = tr8_spec(lead)?;
	if start_index >= spec.starts || !(length > 0.0 && length.is_finite() && z0.is_finite()) {
		return None;
	}
	let p = spec.pitch;
	let r_crest = spec.d * 0.5;
	let r_root = spec.d3 * 0.5;
	let r_buried = r_root - 0.25 * p;
	let tan15 = (15.0_f64).to_radians().tan();
	// Axial half-widths: crest 0.183·P; root grows by the 15° flank run over the
	// full thread depth h3 = (d − d3)/2.
	let half_crest = 0.183 * p;
	let half_root = half_crest + (r_crest - r_root) * tan15;
	let section: [(f64, f64); 6] = [
		(r_buried, half_root),
		(r_root, half_root),
		(r_crest, half_crest),
		(r_crest, -half_crest),
		(r_root, -half_root),
		(r_buried, -half_root),
	];
	let steps_per_turn = 96;
	let n = (length / spec.lead * steps_per_turn as f64).round() as usize;
	if n < 2 {
		return None;
	}
	let phase = TAU * start_index as f64 / spec.starts as f64;
	let sections: Vec<Vec<DVec3>> = (0..=n)
		.map(|k| {
			let t = k as f64 / steps_per_turn as f64; // turns travelled
			let (a, z) = (phase + t * TAU, z0 + t * spec.lead);
			section.iter().map(|&(radius, dz)| DVec3::new(radius * a.cos(), radius * a.sin(), z + dz)).collect()
		})
		.collect();
	loft_solid(&sections)
}

/// De-facto envelope of the ubiquitous flanged **Tr8 brass nut** (the printer
/// listing reproduced across vendors, e.g. the "T8 brass nut" of every lead-screw
/// kit): body Ø10.2 × 15 overall, flange Ø22 × 3.5 at the z = 0 end, four Ø3.5
/// mounting holes on a Ø16 bolt circle (M3 screws), bore Ø8.
const TR8_NUT_BODY_D: f64 = 10.2;
const TR8_NUT_LEN: f64 = 15.0;
const TR8_NUT_FLANGE_D: f64 = 22.0;
const TR8_NUT_FLANGE_T: f64 = 3.5;
const TR8_NUT_BCD: f64 = 16.0;
const TR8_NUT_HOLE_D: f64 = 3.5;

/// The flanged **Tr8 lead-screw nut** (brass-nut envelope, dimensions at the
/// `TR8_NUT_*` constants): flange at z 0…3.5, body to z = 15, bored Ø8 through
/// (the trapezoidal internal thread is not modelled — catalog convention; the
/// DIN 103 female numbers D1/D4 ride on [`tr8_spec`]). One revolved profile
/// (analytic wall tags, no stacked unions) plus the four flange holes. Genus 5.
pub fn lead_screw_nut_tr8() -> Solid {
	let (rb, rf, rbore) = (TR8_NUT_BODY_D * 0.5, TR8_NUT_FLANGE_D * 0.5, 4.0);
	let profile = vec![
		DVec2::new(rbore, 0.0),
		DVec2::new(rf, 0.0),
		DVec2::new(rf, TR8_NUT_FLANGE_T),
		DVec2::new(rb, TR8_NUT_FLANGE_T),
		DVec2::new(rb, TR8_NUT_LEN),
		DVec2::new(rbore, TR8_NUT_LEN),
	];
	let mut nut = revolve(&profile, 48);
	for k in 0..4 {
		let a = TAU * k as f64 / 4.0;
		let at = DVec3::new(TR8_NUT_BCD * 0.5 * a.cos(), TR8_NUT_BCD * 0.5 * a.sin(), TR8_NUT_FLANGE_T);
		nut =
			kernel_brep::holes::drill(&nut, at, -DVec3::Z, TR8_NUT_HOLE_D, kernel_brep::holes::HoleDepth::Through(TR8_NUT_FLANGE_T), None)
				.unwrap_or(nut);
	}
	nut
}

/// Cut a **Tr8 nut trap** into a face — the printed-carriage pocket the flanged
/// nut drops into: a Ø10.6 through-bore for the nut body (and screw passage), a
/// flat-bottomed Ø22.4 × 3.7 flange recess sunk into the face, and four M3 ISO
/// 273 medium clearance holes on the Ø16 bolt circle, through `through` mm of
/// material. `at` is the screw axis on the face, `axis` the outward normal;
/// the bolt circle aligns to `perp_basis(axis)` with the first hole on +X for
/// a +Z face. Fits the [`lead_screw_nut_tr8`] envelope with 0.4/0.2 diametral
/// clearance. `None` for a degenerate axis or a span ≤ the recess depth.
pub fn tr8_nut_trap(solid: &Solid, at: DVec3, axis: DVec3, through: f64) -> Option<Solid> {
	let axis = axis.try_normalize()?;
	let recess_t = TR8_NUT_FLANGE_T + 0.2;
	if !(through > recess_t && through.is_finite()) {
		return None;
	}
	let (e1, e2) = perp_basis(axis);
	// Through-bore for the nut body.
	let mut cut =
		kernel_brep::holes::drill(solid, at, -axis, TR8_NUT_BODY_D + 0.4, kernel_brep::holes::HoleDepth::Through(through), Some(48))
			.ok()?;
	// Flat-bottomed flange recess: a plain cylinder cutter sunk recess_t into the
	// face, overshooting 1 mm above it (no drill point — the recess floor is flat).
	let frame = DMat3::from_cols(e1, e2, axis);
	let recess = cylinder(DVec3::ZERO, DVec3::Z, (TR8_NUT_FLANGE_D + 0.4) * 0.5, recess_t + 1.0, 48)
		.transformed(DAffine3::from_mat3_translation(frame, at - axis * recess_t));
	cut = difference(&cut, &recess);
	for k in 0..4 {
		let a = TAU * k as f64 / 4.0;
		let p = at + e1 * (TR8_NUT_BCD * 0.5 * a.cos()) + e2 * (TR8_NUT_BCD * 0.5 * a.sin());
		cut = clearance_hole(&cut, p, -axis, 3.0, Fit::Medium, None).ok()?;
	}
	Some(cut)
}

#[cfg(test)]
mod tests {
	use super::*;
	use kernel_brep::{cuboid, intersection, tessellate_adaptive_tol, tessellate_default, validate, volume, VertexId};
	use std::f64::consts::PI;

	#[test]
	fn tr8_specs_carry_the_din103_numbers_for_all_three_leads() {
		// Tr8×2 / ×4 / ×8 share the P2 section: d2 = 7, d3 = 5.5, D1 = 6, D4 = 8.5;
		// starts 1/2/4. Lead 3 (not a stocked Tr8 variant) and lead 6 are refused.
		type SpecRow = Option<(usize, f64, f64, f64, f64)>;
		let rows: Vec<SpecRow> =
			[2.0, 4.0, 8.0, 3.0, 6.0].iter().map(|&l| tr8_spec(l).map(|s| (s.starts, s.d2, s.d3, s.nut_d1, s.nut_d4))).collect();
		assert_eq!(
			rows,
			vec![Some((1, 7.0, 5.5, 6.0, 8.5)), Some((2, 7.0, 5.5, 6.0, 8.5)), Some((4, 7.0, 5.5, 6.0, 8.5)), None, None],
			"DIN 103 Tr8 spec rows"
		);
	}

	#[test]
	fn lead_screw_bodies_are_chamfered_major_diameter_envelopes() {
		// ×300 lead 8 and ×100 lead 2: genus-0 watertight×2 envelopes at exactly Ø8,
		// volume = π·16·L minus the half-pitch chamfer ring (1% band for the 48-gon).
		for (len, lead) in [(300.0, 8.0), (100.0, 2.0)] {
			let s = lead_screw_tr8(len, lead).expect("stocked lead");
			let v = validate(&s);
			let r_max = (0..s.vertex_count() as u32)
				.map(|i| {
					let p = s.position(VertexId(i));
					(p.x * p.x + p.y * p.y).sqrt()
				})
				.fold(0.0, f64::max);
			let chamfer_ring = PI * 1.0 * (4.0 * 4.0 - 3.0 * 3.0 - (4.0 * 4.0 - 3.0 * 3.0) / 3.0); // cone frustum rebate over-bound
			let full = PI * 16.0 * len;
			let vol = volume(&s).abs();
			assert!(
				v.closed
					&& v.manifold && v.genus == 0
					&& tessellate_default(&s).is_watertight()
					&& tessellate_adaptive_tol(&s, 0.01).is_watertight()
					&& (r_max - 4.0).abs() < 1e-9
					&& vol < full && vol > full * 0.99 - chamfer_ring,
				"Tr8×{lead} ×{len}: want watertight×2 genus-0 at exactly Ø8; got {v:?} r_max={r_max} vol={vol:.0} (full {full:.0})"
			);
		}
		assert!(lead_screw_tr8(100.0, 3.0).is_none() && lead_screw_tr8(1.0, 8.0).is_none(), "lead 3 and a 1 mm screw must be refused");
	}

	#[test]
	fn tr8_ridge_holds_the_trapezoidal_profile_and_starts_never_touch() {
		// One start of Tr8×8 over 40 mm: watertight genus-0 loft, crests exactly at
		// Ø8, base exactly at the buried Ø5.5 − P/2. Then the four-start proof:
		// start 0 and start 1 (phased 90°, interleaved at one pitch axially) must
		// have an EMPTY exact intersection — the DIN 103 root gap (P − root width ≈
		// 0.6 mm) is the designed clearance, so a phase or width error of tenths
		// would collide.
		let r0 = tr8_thread_ridge(8.0, 0, 0.0, 40.0).expect("ridge lofts");
		let v = validate(&r0);
		let radii: Vec<f64> = (0..r0.vertex_count() as u32)
			.map(|i| {
				let q = r0.position(VertexId(i));
				(q.x * q.x + q.y * q.y).sqrt()
			})
			.collect();
		let (r_min, r_max) = radii.iter().fold((f64::INFINITY, 0.0_f64), |(lo, hi), &r| (lo.min(r), hi.max(r)));
		let buried = 5.5 * 0.5 - 0.5;
		assert!(
			v.closed
				&& v.manifold
				&& v.genus == 0
				&& tessellate_adaptive_tol(&r0, 0.01).is_watertight()
				&& (r_max - 4.0).abs() < 1e-9
				&& (r_min - buried).abs() < 1e-9,
			"Tr8×8 ridge: want watertight genus-0 spanning r {buried}…4 exactly; got {v:?} r=[{r_min:.4},{r_max:.4}]"
		);
		let r1 = tr8_thread_ridge(8.0, 1, 0.0, 40.0).expect("second start");
		let clash = intersection(&r0, &r1);
		let clash_vol = if clash.face_count() == 0 { 0.0 } else { volume(&clash).abs() };
		assert!(
			clash_vol < 0.01 && tr8_thread_ridge(8.0, 4, 0.0, 40.0).is_none(),
			"adjacent Tr8×8 starts must interleave without contact (got {clash_vol:.4} mm³) and start 4 of 4 must be refused"
		);
	}

	#[test]
	fn flanged_nut_and_its_trap_mate_with_designed_clearance() {
		// The nut: genus 5 (bore + 4 flange holes), watertight×2, spanning exactly
		// flange Ø22 / overall 15, volume = revolve closed form − 4 flange holes
		// (1% band). The trap, cut through a 10 mm plate: genus +5, the recess floor
		// at exactly face − 3.7, volume = plate − body bore − recess ring − 4 × M3
		// (1.5% band). Posing the nut in the trap (flange face on the recess floor)
		// must give an EMPTY exact intersection — 0.4/0.2 diametral/axial clearance.
		let nut = lead_screw_nut_tr8();
		let v = validate(&nut);
		let ring48 = |r: f64| 48.0 * 0.5 * r * r * (2.0 * PI / 48.0).sin();
		let nut_expected = (ring48(11.0) - ring48(4.0)) * 3.5 + (ring48(5.1) - ring48(4.0)) * 11.5 - 4.0 * PI * 1.75 * 1.75 * 3.5 * 1.01;
		let nut_vol = volume(&nut).abs();
		assert!(
			v.closed
				&& v.manifold
				&& v.genus == 5
				&& tessellate_default(&nut).is_watertight()
				&& tessellate_adaptive_tol(&nut, 0.01).is_watertight()
				&& (nut_vol - nut_expected).abs() / nut_expected < 0.01,
			"Tr8 nut: want watertight×2 genus-5 ~{nut_expected:.0}mm³; got {v:?} vol={nut_vol:.0}"
		);

		let plate = cuboid(DVec3::new(-25.0, -25.0, 0.0), DVec3::new(25.0, 25.0, 10.0));
		let trapped = tr8_nut_trap(&plate, DVec3::new(0.0, 0.0, 10.0), DVec3::Z, 10.0).expect("valid trap");
		let tv = validate(&trapped);
		let floor_z = 10.0 - 3.7;
		let floor_verts =
			(0..trapped.vertex_count() as u32).map(|i| trapped.position(VertexId(i))).filter(|p| (p.z - floor_z).abs() < 1e-9).count();
		// The nut posed in the trap: flange seated on the recess floor.
		let posed = nut.transformed(DAffine3::from_translation(DVec3::new(0.0, 0.0, floor_z)));
		let material_clash = intersection(&trapped, &posed);
		let clash_vol = if material_clash.face_count() == 0 { 0.0 } else { volume(&material_clash).abs() };
		assert!(
			tv.closed && tv.manifold && tv.genus == 5 && floor_verts >= 48 && clash_vol < 0.01,
			"Tr8 nut trap: want genus-5 with a flat recess floor at z={floor_z} and the posed nut clearing the pocket; got {tv:?} floors={floor_verts} clash={clash_vol:.4}"
		);
		assert!(
			tr8_nut_trap(&plate, DVec3::new(0.0, 0.0, 10.0), DVec3::ZERO, 10.0).is_none()
				&& tr8_nut_trap(&plate, DVec3::new(0.0, 0.0, 10.0), DVec3::Z, 3.0).is_none(),
			"a zero axis and a span thinner than the flange recess must be refused"
		);
	}
}
