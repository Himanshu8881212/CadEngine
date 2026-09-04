// Copyright (c) LMCAD. Licensed under the MIT License.

//! **Bearing bodies** — assembly/display solids for the bearings whose *seats* the
//! hole wizard already cuts: deep-groove ball bearings (603…6804, the kernel's
//! cited d × D × B table), flanged miniatures (F608/F623), thrust bearings
//! (51100/51101), and the KP08 pillow block.
//!
//! Sourcing: deep-groove boundary dimensions come from
//! [`kernel_brep::holes::bearing_specs`] (cited there); flanged/thrust/KP08 rows are
//! cited at their tables below. Every body is an honest **envelope**: one revolve
//! (or one extrusion + hole-wizard drills for KP08) — no balls, cages, races,
//! shields or ISO 15 corner chamfers. So a featureless tube still *reads* as a
//! bearing in exploded views, each body carries shallow **ring-split witness
//! grooves** (the visual line where inner and outer ring — or the two thrust
//! washers — meet); their proportions are display conventions, stated at
//! [`SPLIT_GROOVE_DEPTH`], not standard dimensions.

use kernel_brep::holes::{bearing_spec, drill, HoleDepth};
use kernel_brep::math::{DAffine3, DVec2, DVec3};
use kernel_brep::{revolve, Solid};

/// Depth of the cosmetic ring-split witness groove (mm) — cut into each axial face
/// of radial bearings and into the OD + bore walls of thrust bearings. Display
/// convention (with the groove band width: 25 % of the radial wall for radial
/// bearings, 15 % of the height for thrust), NOT a standard dimension; the load
/// envelope is unchanged apart from these shallow notches.
pub const SPLIT_GROOVE_DEPTH: f64 = 0.4;

/// A **deep-groove ball bearing body** (e.g. `"608"`): the d × D × B annulus of the
/// kernel's cited boundary-dimension table ([`kernel_brep::holes::bearing_specs`] —
/// 603, 608, 625, 688, 6000, 6001, 6804), bore along +Z from z = 0, with the
/// ring-split witness groove on each face at the mid-wall radius. One revolved
/// profile: closed, manifold, genus 1, watertight on both tessellation routes.
/// Drop it into a [`kernel_brep::holes::bearing_seat`] pocket of the same
/// designation for assembly/interference studies. `None` for a designation outside
/// the seat table.
pub fn deep_groove_bearing(designation: &str) -> Option<Solid> {
	let s = bearing_spec(designation)?;
	let (rb, ro, w) = (s.bore * 0.5, s.outer * 0.5, s.width);
	let rm = (rb + ro) * 0.5;
	let hg = (ro - rb) * 0.125; // half of the 25 %-of-wall groove band
	let (g0, g1, d) = (rm - hg, rm + hg, SPLIT_GROOVE_DEPTH);
	// Lathe profile (r, z), CCW: bottom face with its groove notch, OD wall up,
	// top face with its groove notch, bore wall down.
	let profile = vec![
		DVec2::new(rb, 0.0),
		DVec2::new(g0, 0.0),
		DVec2::new(g0, d),
		DVec2::new(g1, d),
		DVec2::new(g1, 0.0),
		DVec2::new(ro, 0.0),
		DVec2::new(ro, w),
		DVec2::new(g1, w),
		DVec2::new(g1, w - d),
		DVec2::new(g0, w - d),
		DVec2::new(g0, w),
		DVec2::new(rb, w),
	];
	Some(revolve(&profile, 48))
}

/// One flanged-miniature-bearing row (all mm): the base d × D × B plus the flange
/// diameter and flange thickness.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FlangedBearingSpec {
	/// Designation, e.g. `"F608"`.
	pub designation: &'static str,
	/// Bore diameter d.
	pub bore: f64,
	/// Outer ring diameter D (below the flange).
	pub outer: f64,
	/// Ring width B.
	pub width: f64,
	/// Flange outer diameter.
	pub flange_d: f64,
	/// Flange thickness.
	pub flange_w: f64,
}

/// Flanged miniature bearings `(d × D × B, flange Ø × thickness)`. Source: the
/// standard flanged listings reproduced across vendors (VXB/BC-Precision style
/// rows): F608ZZ 8 × 22 × 7 with flange Ø25 × 1.5; F623ZZ 3 × 10 × 4 with flange
/// Ø11.5 × 0.6.
const FLANGED: [FlangedBearingSpec; 2] = [
	FlangedBearingSpec { designation: "F608", bore: 8.0, outer: 22.0, width: 7.0, flange_d: 25.0, flange_w: 1.5 },
	FlangedBearingSpec { designation: "F623", bore: 3.0, outer: 10.0, width: 4.0, flange_d: 11.5, flange_w: 0.6 },
];

/// The flanged-bearing table row for `"F608"` or `"F623"`, or `None`.
pub fn flanged_bearing_spec(designation: &str) -> Option<FlangedBearingSpec> {
	FLANGED.iter().find(|s| s.designation.eq_ignore_ascii_case(designation)).copied()
}

/// A **flanged miniature bearing body** (`"F608"` / `"F623"`): the deep-groove
/// annulus with its locating flange at the z = 0 end (flange face down — drop it
/// into a plain bore and the flange registers on the wall), ring-split witness
/// grooves on both faces. One revolve: closed, manifold, genus 1, watertight on
/// both routes. `None` outside the two-row table.
pub fn flanged_bearing(designation: &str) -> Option<Solid> {
	let s = flanged_bearing_spec(designation)?;
	let (rb, ro, rf, w, fw) = (s.bore * 0.5, s.outer * 0.5, s.flange_d * 0.5, s.width, s.flange_w);
	let rm = (rb + ro) * 0.5;
	let hg = (ro - rb) * 0.125;
	let (g0, g1, d) = (rm - hg, rm + hg, SPLIT_GROOVE_DEPTH);
	let profile = vec![
		DVec2::new(rb, 0.0),
		DVec2::new(g0, 0.0),
		DVec2::new(g0, d),
		DVec2::new(g1, d),
		DVec2::new(g1, 0.0),
		DVec2::new(rf, 0.0), // flange face
		DVec2::new(rf, fw),
		DVec2::new(ro, fw), // step back to the ring OD
		DVec2::new(ro, w),
		DVec2::new(g1, w),
		DVec2::new(g1, w - d),
		DVec2::new(g0, w - d),
		DVec2::new(g0, w),
		DVec2::new(rb, w),
	];
	Some(revolve(&profile, 48))
}

/// One single-direction thrust-bearing row (all mm): bore × outer Ø × height.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ThrustBearingSpec {
	/// Designation, e.g. `"51100"`.
	pub designation: &'static str,
	/// Shaft-washer bore d.
	pub bore: f64,
	/// Outer diameter D.
	pub outer: f64,
	/// Overall height T (both washers + ball set).
	pub height: f64,
}

/// Single-direction thrust ball bearings, 511 series `(d × D × T)`. Source: ISO 104
/// boundary dimensions as reproduced in every thrust catalog (SKF/NTN 511xx pages):
/// 51100 10 × 24 × 9; 51101 12 × 26 × 9.
const THRUST: [ThrustBearingSpec; 2] = [
	ThrustBearingSpec { designation: "51100", bore: 10.0, outer: 24.0, height: 9.0 },
	ThrustBearingSpec { designation: "51101", bore: 12.0, outer: 26.0, height: 9.0 },
];

/// The thrust-bearing table row for `"51100"` or `"51101"`, or `None`.
pub fn thrust_bearing_spec(designation: &str) -> Option<ThrustBearingSpec> {
	THRUST.iter().find(|s| s.designation == designation).copied()
}

/// A **thrust ball bearing body** (`"51100"` / `"51101"`): the d × D × T stack of
/// the ISO 104 boundary dimensions as one annular envelope, with the washer-split
/// witness groove around the OD *and* the bore at mid-height (the line where shaft
/// and housing washers meet). One revolve: closed, manifold, genus 1, watertight
/// on both routes. Honest envelope: the two washers + ball cage are one body; the
/// housing washer's slightly larger bore (d + ~0.2) is not modelled. `None`
/// outside the two-row table.
pub fn thrust_bearing(designation: &str) -> Option<Solid> {
	let s = thrust_bearing_spec(designation)?;
	let (rb, ro, h) = (s.bore * 0.5, s.outer * 0.5, s.height);
	let hg = h * 0.075; // half of the 15 %-of-height groove band
	let (h0, h1, d) = (h * 0.5 - hg, h * 0.5 + hg, SPLIT_GROOVE_DEPTH);
	let profile = vec![
		DVec2::new(rb, 0.0),
		DVec2::new(ro, 0.0),
		DVec2::new(ro, h0),
		DVec2::new(ro - d, h0), // OD witness groove
		DVec2::new(ro - d, h1),
		DVec2::new(ro, h1),
		DVec2::new(ro, h),
		DVec2::new(rb, h),
		DVec2::new(rb, h1), // bore witness groove (outward notch)
		DVec2::new(rb + d, h1),
		DVec2::new(rb + d, h0),
		DVec2::new(rb, h0),
	];
	Some(revolve(&profile, 48))
}

/// De-facto **KP08** pillow-block dimensions (the zinc-alloy 8 mm self-aligning
/// pillow block, listing drawing reproduced across CNC/printer vendors): centre
/// height 15, base 55 long × 13 wide × 6 thick, two Ø5.5 bolt holes (M5) 42 apart,
/// housing boss Ø29 → overall height 29.5.
const KP08_BASE_W: f64 = 55.0;
const KP08_DEPTH: f64 = 13.0;
const KP08_BASE_H: f64 = 6.0;
const KP08_CENTER_H: f64 = 15.0;
const KP08_BOSS_R: f64 = 14.5;
const KP08_HOLES: f64 = 42.0;
const KP08_HOLE_D: f64 = 5.5;
const KP08_BORE: f64 = 8.0;

/// A **KP08 pillow block** envelope: base plate plus circular housing boss as ONE
/// extruded profile (base 55 × 13 × 6, boss Ø29 centred at the catalog 15 mm shaft
/// height, overall 29.5 tall), the Ø8 shaft bore through along +Y and the two Ø5.5
/// base bolt holes at ±21. Genus 3 (shaft bore + two bolt holes). Honest envelope:
/// the spherical self-aligning insert, grease nipple and casting fillets are not
/// modelled — the bore is the straight Ø8 shaft pass-through. Dimension source
/// cited at [`KP08_BASE_W`].
pub fn kp08_pillow_block() -> Solid {
	let (bw, rb) = (KP08_BASE_W * 0.5, KP08_BOSS_R);
	// Front profile in local XY (x across, y up): base slab ∪ boss disc, wound CCW.
	// The boss arc meets the base top (y = 6) at x = ±√(r² − (cy − 6)²).
	let cy = KP08_CENTER_H;
	let xj = (rb * rb - (cy - KP08_BASE_H) * (cy - KP08_BASE_H)).sqrt();
	let a0 = (KP08_BASE_H - cy).atan2(xj); // angle of the right junction, < 0
	let a1 = std::f64::consts::PI - a0; // left junction, CCW past the top
									 // Even sample count: the arc is symmetric about 90°, so the midpoint sample
									 // lands exactly on the boss apex (overall height = centre + boss radius).
	let n = ((((a1 - a0) / 4.0_f64.to_radians()).ceil() as usize).max(2) + 1) & !1;
	let mut profile = vec![DVec2::new(-bw, 0.0), DVec2::new(bw, 0.0), DVec2::new(bw, KP08_BASE_H), DVec2::new(xj, KP08_BASE_H)];
	for i in 0..=n {
		let a = a0 + (a1 - a0) * i as f64 / n as f64;
		profile.push(DVec2::new(rb * a.cos(), cy + rb * a.sin()));
	}
	profile.push(DVec2::new(-bw, KP08_BASE_H));
	let body = kernel_brep::extrude(&profile, KP08_DEPTH).transformed(DAffine3::from_cols(
		DVec3::new(1.0, 0.0, 0.0),
		DVec3::new(0.0, 0.0, 1.0),
		DVec3::new(0.0, -1.0, 0.0),
		DVec3::new(0.0, KP08_DEPTH * 0.5, 0.0),
	));
	let mut s =
		drill(&body, DVec3::new(0.0, KP08_DEPTH * 0.5, KP08_CENTER_H), -DVec3::Y, KP08_BORE, HoleDepth::Through(KP08_DEPTH), Some(48))
			.expect("constant geometry: the shaft bore is valid");
	for sx in [1.0, -1.0] {
		let at = DVec3::new(sx * KP08_HOLES * 0.5, 0.0, KP08_BASE_H);
		s = drill(&s, at, -DVec3::Z, KP08_HOLE_D, HoleDepth::Through(KP08_BASE_H), None)
			.expect("constant geometry: the bolt holes are valid");
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

	/// Area of the 48-gon inscribed in radius `r` (the revolve discretisation).
	fn ring48(r: f64) -> f64 {
		48.0 * 0.5 * r * r * (2.0 * PI / 48.0).sin()
	}

	#[test]
	fn deep_groove_bodies_fill_their_seat_table_envelopes() {
		// Every designation in the kernel seat table builds: genus-1 watertight×2
		// revolve spanning exactly bore/2 … D/2 and 0 … B, volume equal to the 48-gon
		// closed form (annulus − two witness-groove rings) to 1e-6 relative.
		let mut all = true;
		let mut diag = String::new();
		for spec in kernel_brep::holes::bearing_specs() {
			let b = deep_groove_bearing(spec.designation).expect("every seat row has a body");
			let (ok, v) = check(&b, 1);
			let (rb, ro) = (spec.bore * 0.5, spec.outer * 0.5);
			let rm = (rb + ro) * 0.5;
			let hg = (ro - rb) * 0.125;
			let expected = (ring48(ro) - ring48(rb)) * spec.width - 2.0 * (ring48(rm + hg) - ring48(rm - hg)) * SPLIT_GROOVE_DEPTH;
			let vol = volume(&b).abs();
			let (mut rmin, mut rmax, mut zmax) = (f64::INFINITY, 0.0_f64, 0.0_f64);
			for i in 0..b.vertex_count() as u32 {
				let p = b.position(VertexId(i));
				let r = (p.x * p.x + p.y * p.y).sqrt();
				rmin = rmin.min(r);
				rmax = rmax.max(r);
				zmax = zmax.max(p.z);
			}
			let row_ok = ok
				&& (rmin - rb).abs() < 1e-9
				&& (rmax - ro).abs() < 1e-9
				&& (zmax - spec.width).abs() < 1e-9
				&& (vol - expected).abs() / expected < 1e-6;
			if !row_ok {
				diag += &format!("{}: {v} r={rmin}..{rmax} z={zmax} vol={vol:.3} want={expected:.3}; ", spec.designation);
			}
			all &= row_ok;
		}
		assert!(
			all && deep_groove_bearing("609").is_none(),
			"every seat-table designation must build a genus-1 watertight×2 body of the exact closed-form volume (and 609 is not stocked); failures: {diag}"
		);
	}

	#[test]
	fn flanged_bearings_step_out_to_their_flange_at_the_flange_face() {
		// F608 and F623: genus-1 watertight×2, flange radius reached only within
		// z ≤ flange_w, ring OD beyond it, volume = closed form to 1e-6.
		for des in ["F608", "F623"] {
			let s = flanged_bearing_spec(des).expect("table row");
			let b = flanged_bearing(des).expect("table size");
			let (ok, diag) = check(&b, 1);
			let (rb, ro, rf) = (s.bore * 0.5, s.outer * 0.5, s.flange_d * 0.5);
			let rm = (rb + ro) * 0.5;
			let hg = (ro - rb) * 0.125;
			let expected = (ring48(ro) - ring48(rb)) * s.width + (ring48(rf) - ring48(ro)) * s.flange_w
				- 2.0 * (ring48(rm + hg) - ring48(rm - hg)) * SPLIT_GROOVE_DEPTH;
			let vol = volume(&b).abs();
			let flange_high = (0..b.vertex_count() as u32)
				.map(|i| b.position(VertexId(i)))
				.filter(|p| ((p.x * p.x + p.y * p.y).sqrt() - rf).abs() < 1e-9)
				.all(|p| p.z < s.flange_w + 1e-9);
			assert!(
				ok && flange_high && (vol - expected).abs() / expected < 1e-6,
				"{des}: want watertight×2 genus-1, flange Ø{} only within z ≤ {}, exactly {expected:.3}mm³; got {diag} flange_low={flange_high} vol={vol:.3}",
				s.flange_d,
				s.flange_w
			);
		}
		assert!(flanged_bearing("F625").is_none(), "F625 is not in the two-row flanged table");
	}

	#[test]
	fn thrust_bearings_carry_the_washer_split_at_mid_height() {
		// 51100 and 51101: genus-1 watertight×2 annuli of the ISO 104 envelope with
		// the mid-height witness grooves on OD and bore; closed-form volume to 1e-6.
		for des in ["51100", "51101"] {
			let s = thrust_bearing_spec(des).expect("table row");
			let b = thrust_bearing(des).expect("table size");
			let (ok, diag) = check(&b, 1);
			let (rb, ro) = (s.bore * 0.5, s.outer * 0.5);
			let gw = s.height * 0.15;
			let expected = (ring48(ro) - ring48(rb)) * s.height
				- (ring48(ro) - ring48(ro - SPLIT_GROOVE_DEPTH)) * gw
				- (ring48(rb + SPLIT_GROOVE_DEPTH) - ring48(rb)) * gw;
			let vol = volume(&b).abs();
			let split_verts = (0..b.vertex_count() as u32)
				.map(|i| b.position(VertexId(i)))
				.filter(|p| {
					let r = (p.x * p.x + p.y * p.y).sqrt();
					((r - (ro - SPLIT_GROOVE_DEPTH)).abs() < 1e-9 || (r - (rb + SPLIT_GROOVE_DEPTH)).abs() < 1e-9)
						&& (p.z - s.height * 0.5).abs() < gw
				})
				.count();
			assert!(
				ok && split_verts >= 192 && (vol - expected).abs() / expected < 1e-6,
				"{des}: want watertight×2 genus-1 with mid-height split grooves on OD and bore, exactly {expected:.3}mm³; got {diag} split_verts={split_verts} vol={vol:.3}"
			);
		}
		assert!(thrust_bearing("51200").is_none(), "the 512 heavy series is not stocked");
	}

	#[test]
	fn kp08_block_carries_its_shaft_at_the_catalog_centre_height() {
		// Genus 3 (shaft bore + 2 bolt holes), watertight×2; 55 wide, 29.5 tall
		// overall, bore wall at exactly r 4 about (0, *, 15); volume = profile area
		// (slab + boss disc above the base) × 13 − bore − bolt holes within 1%.
		let s = kp08_pillow_block();
		let (ok, diag) = check(&s, 3);
		let (mut xmax, mut zmax) = (0.0_f64, 0.0_f64);
		let mut bore_verts = 0;
		for i in 0..s.vertex_count() as u32 {
			let p = s.position(VertexId(i));
			xmax = xmax.max(p.x.abs());
			zmax = zmax.max(p.z);
			if ((p.x * p.x + (p.z - KP08_CENTER_H) * (p.z - KP08_CENTER_H)).sqrt() - KP08_BORE * 0.5).abs() < 1e-9 {
				bore_verts += 1;
			}
		}
		let r = KP08_BOSS_R;
		let d = KP08_CENTER_H - KP08_BASE_H; // chord distance of the base-top cut
		let segment_below = r * r * (d / r).acos() - d * (r * r - d * d).sqrt();
		let profile_area = KP08_BASE_W * KP08_BASE_H + (PI * r * r - segment_below);
		let expected = profile_area * KP08_DEPTH
			- PI * (KP08_BORE * 0.5) * (KP08_BORE * 0.5) * KP08_DEPTH
			- 2.0 * PI * (KP08_HOLE_D * 0.5) * (KP08_HOLE_D * 0.5) * KP08_BASE_H;
		let vol = volume(&s).abs();
		assert!(
			ok && (xmax - KP08_BASE_W * 0.5).abs() < 1e-9
				&& (zmax - (KP08_CENTER_H + KP08_BOSS_R)).abs() < 1e-9
				&& bore_verts >= 96 && (vol - expected).abs() / expected < 0.01,
			"KP08: want watertight×2 genus-3, 55 wide × 29.5 tall with the Ø8 bore at height 15, ~{expected:.0}mm³; got {diag} xmax={xmax} zmax={zmax} bore_verts={bore_verts} vol={vol:.0}"
		);
	}
}
