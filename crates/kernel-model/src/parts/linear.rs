// Copyright (c) LMCAD. Licensed under the MIT License.

//! **Linear motion**: LM-series linear ball bearings (LM8UU/LM12UU), the SC8UU
//! pillow block, SK8/SHF8 smooth-rod shaft supports, and MGN12 profile-rail /
//! carriage **envelopes**. Sourcing: the LM/SC/SK/SHF dimensions are the de-facto
//! catalog rows reproduced across the CNC/printer ecosystem (Misumi/THK-pattern
//! numbers, cited per table); the MGN12 numbers are the HIWIN MGN-series catalog
//! footprint. Every body here is an honest **envelope** — no balls, races, seals
//! or recirculation paths — built from single profiles plus hole-wizard cuts on
//! planar faces, so everything stays on the exact STL route.

use super::shafts::stadium;
use kernel_brep::holes::{drill, tap_drill_hole, Fit, HoleDepth};
use kernel_brep::math::{DAffine3, DVec2, DVec3};
use kernel_brep::{cuboid, difference, revolve, Solid};

/// One LM-series linear-bearing row (all mm): bore, OD, length, and the two
/// retaining-ring grooves (diameter, width, groove-centre spacing).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LmuuSpec {
	/// Shaft bore diameter.
	pub bore: f64,
	/// Outer diameter.
	pub od: f64,
	/// Overall length.
	pub length: f64,
	/// Retaining-ring groove diameter D1.
	pub groove_d: f64,
	/// Groove width W.
	pub groove_w: f64,
	/// Groove-centre to groove-centre spacing B.
	pub groove_spacing: f64,
}

/// LM-series table `(bore, OD, L, groove Ø D1, groove width W, spacing B)`.
/// Source: the standard LM..UU catalog rows (Misumi/THK pattern, reproduced by
/// every vendor): LM8UU 8 × 15 × 24, grooves Ø14.3 × 1.1 at B 17.5;
/// LM12UU 12 × 21 × 30, grooves Ø19.9 × 1.3 at B 23.
const LMUU: [LmuuSpec; 2] = [
	LmuuSpec { bore: 8.0, od: 15.0, length: 24.0, groove_d: 14.3, groove_w: 1.1, groove_spacing: 17.5 },
	LmuuSpec { bore: 12.0, od: 21.0, length: 30.0, groove_d: 19.9, groove_w: 1.3, groove_spacing: 23.0 },
];

/// The LM-series row for a shaft `bore` of 8 (LM8UU) or 12 (LM12UU), or `None`.
pub fn lmuu_spec(bore: f64) -> Option<LmuuSpec> {
	LMUU.iter().find(|s| (s.bore - bore).abs() < 1e-9).copied()
}

/// An **LM8UU / LM12UU linear ball bearing** envelope: the catalog tube (bore ×
/// OD × length along +Z from z = 0) with its two external retaining-ring grooves
/// turned in — one revolved profile, genus 1, watertight on both tessellation
/// routes. Honest envelope: the recirculating balls, races and rubber seals are
/// not modelled — this is the body you press into a printed block or pair with
/// [`sc8uu_block`]. `None` for a bore outside {8, 12}.
pub fn linear_bearing_lmuu(bore: f64) -> Option<Solid> {
	let s = lmuu_spec(bore)?;
	let (rb, ro, rg) = (s.bore * 0.5, s.od * 0.5, s.groove_d * 0.5);
	let (g0, g1) = (
		(s.length - s.groove_spacing) * 0.5, // centre of the lower groove
		(s.length + s.groove_spacing) * 0.5,
	);
	let hw = s.groove_w * 0.5;
	// Lathe profile (r, z), CCW: bore wall up, then the OD wall descending through
	// the two groove notches.
	let profile = vec![
		DVec2::new(rb, 0.0),
		DVec2::new(ro, 0.0),
		DVec2::new(ro, g0 - hw),
		DVec2::new(rg, g0 - hw),
		DVec2::new(rg, g0 + hw),
		DVec2::new(ro, g0 + hw),
		DVec2::new(ro, g1 - hw),
		DVec2::new(rg, g1 - hw),
		DVec2::new(rg, g1 + hw),
		DVec2::new(ro, g1 + hw),
		DVec2::new(ro, s.length),
		DVec2::new(rb, s.length),
	];
	Some(revolve(&profile, 48))
}

/// De-facto **SC8UU** linear-bearing block dimensions (the aluminium pillow
/// block every printer gantry uses; common listing values): block 34 wide ×
/// 30 long (along the shaft) × 22 tall, shaft centre height 11, Ø15 bearing
/// bore through, four M4 platform taps on a 24 × 18 grid, 6 mm deep.
const SC8UU_W: f64 = 34.0;
const SC8UU_L: f64 = 30.0;
const SC8UU_H: f64 = 22.0;
const SC8UU_CENTER_H: f64 = 11.0;
const SC8UU_HOLE_X: f64 = 24.0;
const SC8UU_HOLE_Y: f64 = 18.0;

/// An **SC8UU linear bearing block** envelope: the block with its Ø15 through
/// bore (press seat for an [`linear_bearing_lmuu`]`(8)`) along +Y at centre
/// height 11, and the four M4 platform tap-drill holes (Ø3.3 × 6 deep, 118°
/// points) on the 24 × 18 top grid. Genus 1 (the bore; the taps are blind).
/// Honest envelope: no end seals, circlip grooves or corner fillets.
pub fn sc8uu_block() -> Solid {
	let block = cuboid(DVec3::new(-SC8UU_W * 0.5, -SC8UU_L * 0.5, 0.0), DVec3::new(SC8UU_W * 0.5, SC8UU_L * 0.5, SC8UU_H));
	let mut s = drill(&block, DVec3::new(0.0, SC8UU_L * 0.5, SC8UU_CENTER_H), -DVec3::Y, 15.0, HoleDepth::Through(SC8UU_L), Some(48))
		.expect("constant geometry: the bore tool is valid");
	for (sx, sy) in [(1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0), (1.0, -1.0)] {
		let at = DVec3::new(sx * SC8UU_HOLE_X * 0.5, sy * SC8UU_HOLE_Y * 0.5, SC8UU_H);
		s = tap_drill_hole(&s, at, -DVec3::Z, 4.0, HoleDepth::Blind(6.0), None).expect("constant geometry: the tap tool is valid");
	}
	s
}

/// De-facto **SK8** shaft-support dimensions (the upright rod clamp): base
/// 42 × 14 × 6 with two Ø5.5 holes 32 apart, tower 20 wide rising to 32.8,
/// Ø8 rod bore at centre height 20, 2 mm clamp slit, M4 cross screw.
const SK8_BASE_W: f64 = 42.0;
const SK8_DEPTH: f64 = 14.0;
const SK8_BASE_H: f64 = 6.0;
const SK8_TOWER_W: f64 = 20.0;
const SK8_H: f64 = 32.8;
const SK8_CENTER_H: f64 = 20.0;
const SK8_BASE_HOLES: f64 = 32.0;

/// An **SK8 shaft support**: the upright clamp block for Ø8 smooth rod — the
/// base-plus-tower profile as one extrusion (no stacked unions), Ø8 rod bore at centre height
/// 20 along +Y, a 2 mm clamp slit from the bore to the top, the M4 clamp
/// clearance screw crossing the slit, and the two Ø5.5 base mounting holes at
/// ±16. Genus 4 (the slit opens the bore; the clamp screw tunnels both lugs;
/// two base holes). Honest envelope: vendor corner radii and the clamp-screw
/// counterbore are omitted.
pub fn shaft_support_sk8() -> Solid {
	let (bw, tw) = (SK8_BASE_W * 0.5, SK8_TOWER_W * 0.5);
	// Front profile in local XY (x across, y up), extruded +Z then rotated so the
	// extrusion depth lies along world Y (the rod axis).
	let profile = vec![
		DVec2::new(bw, 0.0),
		DVec2::new(bw, SK8_BASE_H),
		DVec2::new(tw, SK8_BASE_H),
		DVec2::new(tw, SK8_H),
		DVec2::new(-tw, SK8_H),
		DVec2::new(-tw, SK8_BASE_H),
		DVec2::new(-bw, SK8_BASE_H),
		DVec2::new(-bw, 0.0),
	];
	let body = kernel_brep::extrude(&profile, SK8_DEPTH).transformed(DAffine3::from_cols(
		DVec3::new(1.0, 0.0, 0.0),
		DVec3::new(0.0, 0.0, 1.0),
		DVec3::new(0.0, -1.0, 0.0),
		DVec3::new(0.0, SK8_DEPTH * 0.5, 0.0),
	));
	let mut s = drill(&body, DVec3::new(0.0, SK8_DEPTH * 0.5, SK8_CENTER_H), -DVec3::Y, 8.0, HoleDepth::Through(SK8_DEPTH), Some(48))
		.expect("constant geometry: the rod bore is valid");
	// Clamp slit: from the bore up through the top, full depth.
	s = difference(
		&s,
		&cuboid(DVec3::new(-1.0, -SK8_DEPTH * 0.5 - 1.0, SK8_CENTER_H), DVec3::new(1.0, SK8_DEPTH * 0.5 + 1.0, SK8_H + 1.0)),
	);
	// M4 clamp screw crossing the slit between bore and top.
	s = kernel_brep::holes::clearance_hole(
		&s,
		DVec3::new(SK8_TOWER_W * 0.5, 0.0, (SK8_CENTER_H + 4.0 + SK8_H) * 0.5),
		-DVec3::X,
		4.0,
		Fit::Medium,
		None,
	)
	.expect("constant geometry: the clamp hole is valid");
	for sx in [1.0, -1.0] {
		let at = DVec3::new(sx * SK8_BASE_HOLES * 0.5, 0.0, SK8_BASE_H);
		s = drill(&s, at, -DVec3::Z, 5.5, HoleDepth::Through(SK8_BASE_H), None).expect("constant geometry: the base holes are valid");
	}
	s
}

/// De-facto **SHF8** flange shaft-support dimensions: stadium flange 43 long ×
/// 20 wide × 10 thick, Ø8 rod bore through the centre (along the plate
/// normal), two Ø5.5 ear holes 32 apart, 2 mm clamp slit from the bore out the
/// +X end, M4 clamp screw crossing it.
const SHF8_LEN: f64 = 43.0;
const SHF8_W: f64 = 20.0;
const SHF8_T: f64 = 10.0;
const SHF8_EAR_HOLES: f64 = 32.0;

/// An **SHF8 flange shaft support**: the face-mount rod clamp — a stadium
/// plate with the Ø8 bore along its normal (+Z, plate z 0…10), Ø5.5 ear holes
/// at ±16 on the long axis, a 2 mm slit from the bore out the +Y side (between
/// the ears, as on the real part) and the M4 clamp screw (through-all along X
/// at y = 7) crossing it. Genus 4 (slit-opened bore, clamp screw tunnelling
/// both lips, two ear holes). Honest envelope: no collar boss or corner radii;
/// the clamp bore is clearance Ø both lips (far-lip thread not modelled). The
/// matching SK8 (upright style) is [`shaft_support_sk8`].
pub fn shaft_support_shf8() -> Solid {
	let plate = kernel_brep::extrude(&stadium(SHF8_LEN, SHF8_W), SHF8_T).transformed(DAffine3::from_translation(DVec3::new(
		-SHF8_LEN * 0.5,
		0.0,
		0.0,
	)));
	let mut s = drill(&plate, DVec3::new(0.0, 0.0, SHF8_T), -DVec3::Z, 8.0, HoleDepth::Through(SHF8_T), Some(48))
		.expect("constant geometry: the rod bore is valid");
	for sx in [1.0, -1.0] {
		let at = DVec3::new(sx * SHF8_EAR_HOLES * 0.5, 0.0, SHF8_T);
		s = drill(&s, at, -DVec3::Z, 5.5, HoleDepth::Through(SHF8_T), None).expect("constant geometry: the ear holes are valid");
	}
	// Clamp slit from the bore out the +Y side (clear of both ear holes).
	s = difference(&s, &cuboid(DVec3::new(-1.0, 0.0, -1.0), DVec3::new(1.0, SHF8_W * 0.5 + 1.0, SHF8_T + 1.0)));
	s = kernel_brep::holes::clearance_hole(&s, DVec3::new(SHF8_LEN * 0.5, 7.0, SHF8_T * 0.5), -DVec3::X, 4.0, Fit::Medium, None)
		.expect("constant geometry: the clamp hole is valid");
	s
}

/// HIWIN **MGN12** catalog footprint (mm): rail 12 wide × 8 tall, M3
/// countersunk mounting holes on a 25 pitch; MGN12H carriage block 45.4 × 27,
/// assembly height 13 (block modelled 11 tall riding 6 deep over the rail),
/// four M3 taps on a 20 × 20 grid.
const MGN12_RAIL_W: f64 = 12.0;
const MGN12_RAIL_H: f64 = 8.0;
const MGN12_PITCH: f64 = 25.0;
const MGN12H_L: f64 = 45.4;
const MGN12H_W: f64 = 27.0;
const MGN12H_BLOCK_H: f64 = 11.0;
const MGN12H_CHANNEL_W: f64 = 12.4;
const MGN12H_CHANNEL_D: f64 = 6.0;
const MGN12H_HOLES: f64 = 20.0;

/// An **MGN12 profile-rail envelope**: a 12 × 8 bar along +Y from y = 0 with
/// M3 countersunk mounting holes (ISO 273 clearance + DIN 74 csk) on the
/// catalog 25 mm pitch, the pattern centred along the bar. The raceway profile
/// is intentionally NOT modelled (envelope; the real HIWIN groove geometry is
/// proprietary) — mate with [`mgn12_carriage`], whose channel rides this bar
/// at the catalog 13 mm assembly height. Genus = hole count. `None` when the
/// bar is shorter than one pitch or degenerate.
pub fn mgn12_rail(length: f64) -> Option<Solid> {
	if !(length >= MGN12_PITCH && length.is_finite()) {
		return None;
	}
	let bar = cuboid(DVec3::new(-MGN12_RAIL_W * 0.5, 0.0, 0.0), DVec3::new(MGN12_RAIL_W * 0.5, length, MGN12_RAIL_H));
	let n = ((length - MGN12_PITCH) / MGN12_PITCH).floor() as usize + 1;
	let first = (length - (n - 1) as f64 * MGN12_PITCH) * 0.5;
	let mut s = bar;
	for k in 0..n {
		let at = DVec3::new(0.0, first + k as f64 * MGN12_PITCH, MGN12_RAIL_H);
		// Faceted-frustum countersink (same DIN 74 table) — the analytic-cone
		// cutter would trip the adaptive stitcher and voxel-heal the STL export.
		s = super::countersunk_hole_faceted(&s, at, -DVec3::Z, 3.0, MGN12_RAIL_H)?;
	}
	Some(s)
}

/// The matching **MGN12H carriage envelope**: the 45.4 × 27 block (z 0…11)
/// with the rail channel (12.4 wide × 6 deep) along +Y underneath and four M3
/// platform tap-drill holes (Ø2.5 × 4 deep) on the 20 × 20 grid. Riding an
/// [`mgn12_rail`] with the channel ceiling on the rail top puts the platform
/// at the catalog 13 mm assembly height. Genus 0 (the channel is open, the
/// taps blind). Honest envelope: no ball tracks, end caps or lube ports.
pub fn mgn12_carriage() -> Solid {
	let block = cuboid(DVec3::new(-MGN12H_W * 0.5, -MGN12H_L * 0.5, 0.0), DVec3::new(MGN12H_W * 0.5, MGN12H_L * 0.5, MGN12H_BLOCK_H));
	let channel = cuboid(
		DVec3::new(-MGN12H_CHANNEL_W * 0.5, -MGN12H_L * 0.5 - 1.0, -1.0),
		DVec3::new(MGN12H_CHANNEL_W * 0.5, MGN12H_L * 0.5 + 1.0, MGN12H_CHANNEL_D),
	);
	let mut s = difference(&block, &channel);
	for (sx, sy) in [(1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0), (1.0, -1.0)] {
		let at = DVec3::new(sx * MGN12H_HOLES * 0.5, sy * MGN12H_HOLES * 0.5, MGN12H_BLOCK_H);
		s = tap_drill_hole(&s, at, -DVec3::Z, 3.0, HoleDepth::Blind(4.0), None).expect("constant geometry: the platform taps are valid");
	}
	s
}

#[cfg(test)]
mod tests {
	use super::*;
	use kernel_brep::{tessellate_adaptive_tol, tessellate_default, validate, volume, VertexId};
	use std::f64::consts::PI;

	/// `(closed && manifold && genus == want && watertight on both routes, validity)`.
	fn check(s: &Solid, want_genus: i64) -> (bool, String) {
		let v = validate(s);
		let ok = v.closed
			&& v.manifold
			&& v.genus == want_genus
			&& tessellate_default(s).is_watertight()
			&& tessellate_adaptive_tol(s, 0.01).is_watertight();
		(ok, format!("{v:?} wt={} adaptive_wt={}", tessellate_default(s).is_watertight(), tessellate_adaptive_tol(s, 0.01).is_watertight()))
	}

	#[test]
	fn lm_bearings_are_grooved_catalog_tubes() {
		// LM8UU and LM12UU: genus-1 watertight×2 revolves spanning exactly bore/2 …
		// OD/2 and 0 … L, with groove vertices at exactly D1/2, and volume equal to
		// the 48-gon closed form (tube minus two groove rings) to 1e-6 relative.
		for bore in [8.0, 12.0] {
			let spec = lmuu_spec(bore).expect("table row");
			let b = linear_bearing_lmuu(bore).expect("table size");
			let (ok, diag) = check(&b, 1);
			let ring48 = |r: f64| 48.0 * 0.5 * r * r * (2.0 * PI / 48.0).sin();
			let expected = (ring48(spec.od * 0.5) - ring48(spec.bore * 0.5)) * spec.length
				- 2.0 * (ring48(spec.od * 0.5) - ring48(spec.groove_d * 0.5)) * spec.groove_w;
			let vol = volume(&b).abs();
			let groove_verts = (0..b.vertex_count() as u32)
				.map(|i| {
					let p = b.position(VertexId(i));
					(p.x * p.x + p.y * p.y).sqrt()
				})
				.filter(|r| (r - spec.groove_d * 0.5).abs() < 1e-9)
				.count();
			assert!(
				ok && groove_verts >= 96 && (vol - expected).abs() / expected < 1e-6,
				"LM{bore}UU: want watertight×2 genus-1 with ring grooves at Ø{}, exactly {expected:.2}mm³; got {diag} grooves={groove_verts} vol={vol:.2}",
				spec.groove_d
			);
		}
		assert!(linear_bearing_lmuu(10.0).is_none(), "LM10UU is not in the (8, 12) table subset");
	}

	#[test]
	fn sc8uu_block_seats_the_lm8uu_at_catalog_height() {
		// Genus 1 (Ø15 through-bore; the 4 M4 taps are blind), watertight×2; bore
		// wall vertices at exactly r 7.5 about the (0, *, 11) axis; volume = block −
		// Ø15 bore − 4 taps within 1%.
		let s = sc8uu_block();
		let (ok, diag) = check(&s, 1);
		let bore_verts = (0..s.vertex_count() as u32)
			.map(|i| s.position(VertexId(i)))
			.filter(|p| {
				let r = (p.x * p.x + (p.z - SC8UU_CENTER_H) * (p.z - SC8UU_CENTER_H)).sqrt();
				(r - 7.5).abs() < 1e-9
			})
			.count();
		let expected = SC8UU_W * SC8UU_L * SC8UU_H - PI * 7.5 * 7.5 * SC8UU_L - 4.0 * PI * 1.65 * 1.65 * 7.0;
		let vol = volume(&s).abs();
		assert!(
			ok && bore_verts >= 96 && (vol - expected).abs() / expected < 0.01,
			"SC8UU: want watertight×2 genus-1 with the Ø15 seat at height 11, ~{expected:.0}mm³; got {diag} bore_verts={bore_verts} vol={vol:.0}"
		);
	}

	#[test]
	fn shaft_supports_clamp_a_rod_at_their_catalog_stations() {
		// SK8: genus 4, rod bore at exactly (x, z) = (0, 20), base holes spanning
		// ±16, overall 42 wide × 32.8 tall. SHF8: genus 4, plate 43 × 20 × 10 with
		// the bore on the plate normal. Both watertight on both routes.
		let sk = shaft_support_sk8();
		let (ok_sk, diag_sk) = check(&sk, 4);
		let (mut xmax, mut zmax) = (0.0_f64, 0.0_f64);
		let mut rod_verts = 0;
		for i in 0..sk.vertex_count() as u32 {
			let p = sk.position(VertexId(i));
			xmax = xmax.max(p.x.abs());
			zmax = zmax.max(p.z);
			if ((p.x * p.x + (p.z - SK8_CENTER_H) * (p.z - SK8_CENTER_H)).sqrt() - 4.0).abs() < 1e-9 {
				rod_verts += 1;
			}
		}
		assert!(
			ok_sk && (xmax - 21.0).abs() < 1e-9 && (zmax - SK8_H).abs() < 1e-9 && rod_verts >= 48,
			"SK8: want watertight×2 genus-4, 42 wide × 32.8 tall with the Ø8 bore at height 20; got {diag_sk} xmax={xmax} zmax={zmax} rod_verts={rod_verts}"
		);

		let shf = shaft_support_shf8();
		let (ok_shf, diag_shf) = check(&shf, 4);
		let (mut xmax, mut ymax, mut zmax) = (0.0_f64, 0.0_f64, 0.0_f64);
		for i in 0..shf.vertex_count() as u32 {
			let p = shf.position(VertexId(i));
			xmax = xmax.max(p.x.abs());
			ymax = ymax.max(p.y.abs());
			zmax = zmax.max(p.z);
		}
		assert!(
			ok_shf && (xmax - SHF8_LEN * 0.5).abs() < 1e-9 && (ymax - SHF8_W * 0.5).abs() < 1e-9 && (zmax - SHF8_T).abs() < 1e-9,
			"SHF8: want watertight×2 genus-4 in a {SHF8_LEN} × {SHF8_W} × {SHF8_T} stadium; got {diag_shf} x={xmax} y={ymax} z={zmax}"
		);
	}

	#[test]
	fn mgn12_rail_and_carriage_mate_at_the_catalog_assembly_height() {
		// Rails ×100 (4 holes, genus 4) and ×200 (8 holes, genus 8): countersunk
		// pattern centred, watertight×2. Carriage: genus 0, channel ceiling at z = 6
		// so riding an 8-tall rail puts the 11-tall block's platform at 8 − 6 + 11 =
		// 13 — the catalog assembly height — and the posed pair must not
		// interpenetrate (0.2 mm side clearance per flank).
		for (len, holes) in [(100.0, 4i64), (200.0, 8)] {
			let rail = mgn12_rail(len).expect("valid rail");
			let (ok, diag) = check(&rail, holes);
			assert!(ok, "MGN12 ×{len}: want watertight×2 genus-{holes} (one per csk hole); got {diag}");
		}
		let carriage = mgn12_carriage();
		let (ok_c, diag_c) = check(&carriage, 0);
		assert!(ok_c, "MGN12H carriage: want watertight×2 genus-0; got {diag_c}");

		let rail = mgn12_rail(100.0).expect("valid rail");
		// Pose: rail along +Y at x = 0; carriage channel ceiling (z = 6) on the rail
		// top (z = 8) → carriage base plane at z = 2; platform at 2 + 11 = 13.
		let posed = carriage.transformed(DAffine3::from_translation(DVec3::new(0.0, 50.0, MGN12_RAIL_H - MGN12H_CHANNEL_D)));
		let clash = kernel_brep::intersection(&rail, &posed);
		let clash_vol = if clash.face_count() == 0 { 0.0 } else { volume(&clash).abs() };
		let platform_z = MGN12_RAIL_H - MGN12H_CHANNEL_D + MGN12H_BLOCK_H;
		assert!(
			clash_vol < 0.01 && (platform_z - 13.0).abs() < 1e-9 && mgn12_rail(20.0).is_none(),
			"MGN12H on MGN12: platform must land at the catalog 13 mm with no interpenetration; got clash={clash_vol:.4} platform={platform_z}; a 20 mm rail (under one pitch) must be refused"
		);
	}
}
