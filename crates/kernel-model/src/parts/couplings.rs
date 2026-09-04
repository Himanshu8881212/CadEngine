// Copyright (c) LMCAD. Licensed under the MIT License.

//! **Shaft couplings**: jaw (spider) coupling hubs + their elastomer spiders, one-piece
//! set-screw rigid couplings, and one-piece slit **clamp couplings** — the three families
//! every motion build reaches for (motor → lead screw, shaft → shaft).
//!
//! Sourcing honesty: unlike fasteners there is no ISO table for these; the size rows are
//! **de-facto composites of the common commercial listings** (the ubiquitous aluminium
//! couplings sold by StepperOnline/uxcell/Ruland-style catalogs), and the internal
//! proportions (jaw arcs, spigot/jaw radii, screw stations) are stated constants chosen
//! mid-range of those listings — each is documented at its `const`. Tooth/leg engagement
//! geometry is exact: the tests assemble hub + spider + hub and prove zero
//! interpenetration with the designed angular/radial play, the same conjugacy treatment
//! as the gear family.

use kernel_brep::holes::{counterbore_hole, tap_drill_hole, Fit, HoleDepth};
use kernel_brep::math::{DAffine3, DVec2, DVec3};
use kernel_brep::{cuboid, cylinder, difference, Solid};
use std::f64::consts::PI;

/// One jaw-coupling size row (all mm): body OD, assembled overall length, the axial
/// height of the jaw/spider band, and the supported bore range.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JawCouplingSpec {
	/// Body outer diameter.
	pub od: f64,
	/// Assembled overall length (two hubs + interleaved jaw band).
	pub length: f64,
	/// Axial height of the jaw band (spider thickness = this − 0.1 float).
	pub jaw_height: f64,
	/// Smallest stocked bore diameter.
	pub bore_min: f64,
	/// Largest stocked bore diameter.
	pub bore_max: f64,
}

/// Jaw-coupling size table `(OD, L, jaw height, bore min, bore max)`. De-facto rows of
/// the common aluminium flexible jaw couplings (D20L25 / D25L30 / D30L35 / D40L50
/// listings, e.g. StepperOnline & uxcell "flexible shaft coupling"); the D40 max bore is
/// trimmed from the listed 24 to 22 so the bore always leaves a 0.5 mm spigot wall
/// (structural floor of this model, documented at `JAW_SPIGOT_R`).
const JAW: [(f64, f64, f64, f64, f64); 4] =
	[(20.0, 25.0, 4.0, 3.0, 8.0), (25.0, 30.0, 5.0, 4.0, 12.0), (30.0, 35.0, 6.0, 5.0, 16.0), (40.0, 50.0, 8.0, 8.0, 22.0)];

/// Internal proportions of the jaw interface, as fractions of the OD (chosen mid-range
/// of the commercial GR-style geometry; every contact figure below is exercised by the
/// assembled no-interpenetration test):
/// centre spigot radius `0.30·od` (the boss the spider ring wraps),
const JAW_SPIGOT_R: f64 = 0.30;
/// jaw root (inner) radius `0.36·od`,
const JAW_INNER_R: f64 = 0.36;
/// jaw angular width 28° (3 per hub on a 60° station grid → 2° flank play against the
/// 30° spider legs),
const JAW_ARC_DEG: f64 = 28.0;
/// spider leg angular width 30°,
const LEG_ARC_DEG: f64 = 30.0;
/// radial play: spider ring to spigot and to jaw roots 0.2 mm; leg tip to OD 0.5 mm.
const RADIAL_PLAY: f64 = 0.2;

/// The jaw-coupling table row for a body `od` (20, 25, 30, 40), or `None`.
pub fn jaw_coupling_spec(od: f64) -> Option<JawCouplingSpec> {
	JAW.iter().find(|r| (r.0 - od).abs() < 1e-9).map(|&(od, length, jaw_height, bore_min, bore_max)| JawCouplingSpec {
		od,
		length,
		jaw_height,
		bore_min,
		bore_max,
	})
}

/// An annular-sector polygon: radial span `r0..r1`, angular span `a0..a1` (radians,
/// CCW), arcs sampled every ≤ 4°.
fn sector_poly(r0: f64, r1: f64, a0: f64, a1: f64) -> Vec<DVec2> {
	let n = (((a1 - a0) / 4.0_f64.to_radians()).ceil() as usize).max(1);
	let at = |r: f64, a: f64| DVec2::new(r * a.cos(), r * a.sin());
	let mut poly = Vec::with_capacity(2 * n + 2);
	for i in 0..=n {
		poly.push(at(r1, a0 + (a1 - a0) * i as f64 / n as f64)); // outer arc, CCW
	}
	for i in (0..=n).rev() {
		poly.push(at(r0, a0 + (a1 - a0) * i as f64 / n as f64)); // inner arc, CW back
	}
	poly
}

/// One **jaw-coupling hub** of the `od` table size, bored Ø`bore_d`: body cylinder of
/// length `(L − jaw_height)/2`, a centre spigot (Ø `0.60·od`) rising half the band
/// minus play (two assembled spigots leave a 0.5 mm axial gap mid-band), and three
/// 28° jaws (radial span `0.36·od … od/2`, 60° station grid, jaw 0 centred on +X)
/// standing `jaw_height` proud — overall solid length `(L + jaw_height)/2`. Two hubs
/// plus a [`jaw_coupling_spider`] assemble to the table's overall `L` with the mating
/// hub flipped and rotated 60°; the designed play is 2° per flank pair, 0.2 mm
/// radial, 0.5 mm spigot-to-spigot (proven by the assembled no-interpenetration
/// test). Genus 1.
///
/// Construction: a full cylinder minus two transverse band cutters — square slabs
/// whose holes preserve the jaws (islands reaching 0.5 mm past the OD, so no cutter
/// surface is co-cylindrical with the body wall) and, in the lower slab only, the
/// spigot; the slabs overlap 0.5 mm in z so no cutter face is coplanar with a face
/// the earlier cut created — then the analytic-cylinder bore (caps stay free of
/// inner loops; STL exports route exact). Honest simplifications: jaw flanks are
/// flat radial planes (commercial curved-jaw GR flanks crown slightly), vendor
/// chamfers and the hub set screws are omitted (drill them with the hole wizard
/// where your vendor puts them). `None` outside the table or the row's bore range.
pub fn jaw_coupling_hub(od: f64, bore_d: f64) -> Option<Solid> {
	let spec = jaw_coupling_spec(od)?;
	if !(bore_d >= spec.bore_min && bore_d <= spec.bore_max) {
		return None;
	}
	let hub_len = (spec.length - spec.jaw_height) * 0.5;
	let total = hub_len + spec.jaw_height;
	let (r_spig, r_jaw) = (JAW_SPIGOT_R * od, JAW_INNER_R * od);
	let spig_h = (spec.jaw_height - 0.5) * 0.5; // spigot height above the hub face
	let body = cylinder(DVec3::ZERO, DVec3::Z, od * 0.5, total, 48);
	let half_jaw = (JAW_ARC_DEG * 0.5).to_radians();
	let sq = od * 0.5 + 2.0;
	let square = vec![DVec2::new(sq, sq), DVec2::new(-sq, sq), DVec2::new(-sq, -sq), DVec2::new(sq, -sq)];
	let jaw_islands: Vec<Vec<DVec2>> = (0..3)
		.map(|k| {
			let c = 2.0 * PI * k as f64 / 3.0;
			sector_poly(r_jaw, od * 0.5 + 0.5, c - half_jaw, c + half_jaw)
		})
		.collect();
	// Lower band cut (hub face … 0.5 above the spigot top): spare spigot + jaws.
	let mut holes = vec![super::circle48(r_spig)];
	holes.extend(jaw_islands.iter().cloned());
	let lower = kernel_brep::extrude_with_holes(&square, &holes, spig_h + 0.5)
		.transformed(DAffine3::from_translation(DVec3::new(0.0, 0.0, hub_len)));
	// Upper band cut (spigot top … past the jaw tips): spare the jaws only. Its
	// bottom cap trims the spigot at spig_h, 0.5 below any face the lower cut made.
	let upper = kernel_brep::extrude_with_holes(&square, &jaw_islands, (spec.jaw_height - spig_h) + 1.0)
		.transformed(DAffine3::from_translation(DVec3::new(0.0, 0.0, hub_len + spig_h)));
	let s = difference(&difference(&body, &lower), &upper);
	// Bore: analytic cylinder through everything (spigot included).
	Some(difference(&s, &cylinder(DVec3::new(0.0, 0.0, -1.0), DVec3::Z, bore_d * 0.5, total + 2.0, 48)))
}

/// The **elastomer spider** (star insert) of the `od` jaw-coupling size: a centre ring
/// wrapping the hub spigot (bore `2·(0.30·od + 0.2)`, outer `0.36·od − 0.2`) with six
/// 30° legs reaching to `od/2 − 0.5`, `jaw_height − 0.1` thick (0.05 mm axial float
/// per side inside the band), legs centred on 30° + k·60° — exactly the stations left
/// free by two interleaved hubs, with 1° angular play each flank. One star-polygon
/// extrusion plus the analytic bore (genus 1). Real spiders crown their legs and round
/// every corner; this is the flat-flanked envelope, honest for clearance/BOM work.
pub fn jaw_coupling_spider(od: f64) -> Option<Solid> {
	let spec = jaw_coupling_spec(od)?;
	let r_hole = JAW_SPIGOT_R * od + RADIAL_PLAY;
	let r_ring = JAW_INNER_R * od - RADIAL_PLAY;
	let r_leg = od * 0.5 - 0.5;
	let half_leg = (LEG_ARC_DEG * 0.5).to_radians();
	let at = |r: f64, a: f64| DVec2::new(r * a.cos(), r * a.sin());
	let arc = |pts: &mut Vec<DVec2>, r: f64, a0: f64, a1: f64| {
		let n = (((a1 - a0) / 4.0_f64.to_radians()).ceil() as usize).max(1);
		for i in 0..=n {
			pts.push(at(r, a0 + (a1 - a0) * i as f64 / n as f64));
		}
	};
	// Star outline: leg arc at r_leg, drop to the ring, ring arc to the next leg, CCW.
	let mut poly: Vec<DVec2> = Vec::new();
	for k in 0..6 {
		let c = PI / 6.0 + k as f64 * PI / 3.0; // legs at 30° + k·60°
		arc(&mut poly, r_leg, c - half_leg, c + half_leg);
		arc(&mut poly, r_ring, c + half_leg, c + PI / 3.0 - half_leg);
	}
	Some(super::extrude_bored(&poly, spec.jaw_height - 0.1, &[(DVec2::ZERO, r_hole, 48)], &[]))
}

/// One set-screw rigid-coupling size row (all mm): stocked bore, body OD, length, and
/// the DIN 916 set-screw thread size.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RigidCouplingSpec {
	/// Stocked bore diameter.
	pub bore: f64,
	/// Body outer diameter.
	pub od: f64,
	/// Overall length.
	pub length: f64,
	/// Set-screw / clamp-screw metric size.
	pub screw_m: f64,
}

/// Set-screw rigid coupling rows `(bore, OD, L, set-screw M)`. De-facto composite of
/// the common one-piece stainless/aluminium listings (uxcell / generic CNC shaft
/// couplings; Ruland SC-series proportions): 6.35 is the 1/4" stepper shaft.
const SET_SCREW: [(f64, f64, f64, f64); 7] = [
	(4.0, 12.0, 20.0, 3.0),
	(5.0, 14.0, 25.0, 4.0),
	(6.0, 14.0, 25.0, 4.0),
	(6.35, 14.0, 25.0, 4.0),
	(8.0, 16.0, 25.0, 4.0),
	(10.0, 20.0, 30.0, 5.0),
	(12.0, 24.0, 35.0, 6.0),
];

/// Clamp coupling rows `(bore, OD, L, clamp-screw M)`: one-piece slit style (Ruland
/// MSP-proportioned de-facto composite — clamp bodies run larger than set-screw ones
/// to carry the cross screws).
const CLAMP: [(f64, f64, f64, f64); 6] = [
	(4.0, 15.0, 22.0, 3.0),
	(5.0, 16.0, 24.0, 3.0),
	(6.0, 18.0, 25.0, 3.0),
	(8.0, 20.0, 28.0, 4.0),
	(10.0, 23.0, 30.0, 4.0),
	(12.0, 26.0, 32.0, 4.0),
];

fn rigid_spec(table: &[(f64, f64, f64, f64)], bore: f64) -> Option<RigidCouplingSpec> {
	table.iter().find(|r| (r.0 - bore).abs() < 1e-9).map(|&(bore, od, length, screw_m)| RigidCouplingSpec { bore, od, length, screw_m })
}

/// The set-screw coupling table row for a stocked `bore` (4, 5, 6, 6.35, 8, 10, 12), or `None`.
pub fn set_screw_coupling_spec(bore: f64) -> Option<RigidCouplingSpec> {
	rigid_spec(&SET_SCREW, bore)
}

/// The clamp coupling table row for a stocked `bore` (4, 5, 6, 8, 10, 12), or `None`.
pub fn clamp_coupling_spec(bore: f64) -> Option<RigidCouplingSpec> {
	rigid_spec(&CLAMP, bore)
}

/// Bore a possibly **stepped** through-bore: the smaller bore runs the full length,
/// the larger one (if any) opens from its own end to the mid-plane, its floor annulus
/// landing inside the smaller bore's void (no coplanar face pairs). `bore1` enters at
/// z = 0, `bore2` at z = `length`.
///
/// Cutters are 48-gon **prisms**, not analytic cylinders, and the rigid-coupling
/// bodies are prisms too: the cross-drilled couplings must stay all-planar, because
/// the adaptive tessellation stitcher cannot yet seam a drilled hole rim crossing a
/// *tagged curved* face (the kernel's open tessellation frontier) — all-planar bodies
/// ear-clip exactly, keeping the STL export on the `exact` route. The honest trade,
/// documented here: these two families carry no analytic surface tags (`exact_volume`
/// equals the faceted volume; STEP gets the prism), unlike the catalog's plain-bored
/// parts.
fn stepped_bore(body: Solid, bore1: f64, bore2: f64, length: f64) -> Solid {
	let prism = |r: f64, z0: f64, h: f64| {
		kernel_brep::extrude(&super::circle48(r), h).transformed(DAffine3::from_translation(DVec3::new(0.0, 0.0, z0)))
	};
	let (small, large) = (bore1.min(bore2), bore1.max(bore2));
	let s = difference(&body, &prism(small * 0.5, -1.0, length + 2.0));
	if (large - small).abs() < 1e-12 {
		return s;
	}
	let cutter = if (bore2 - large).abs() < 1e-12 {
		prism(large * 0.5, length * 0.5, length * 0.5 + 1.0)
	} else {
		prism(large * 0.5, -1.0, length * 0.5 + 1.0)
	};
	difference(&s, &cutter)
}

/// A one-piece **set-screw rigid shaft coupling** joining Ø`bore1` (entering at z = 0)
/// to Ø`bore2` (entering at z = L) — both stocked table bores; the body OD/L and the
/// DIN 916 screw size come from the larger bore's row. Stepped bores meet at the
/// mid-plane. Four radial set-screw **tap-drill holes** (Ø = M − pitch; the thread
/// itself is not modelled, project convention) reach the bore: two per shaft at the
/// representative stations `L/6` and `L/3` from each end, 90° apart. Genus 5 (the
/// through-bore plus four wall tunnels). `None` for out-of-table bores.
pub fn set_screw_coupling(bore1: f64, bore2: f64) -> Option<Solid> {
	let (s1, s2) = (set_screw_coupling_spec(bore1)?, set_screw_coupling_spec(bore2)?);
	let spec = if s1.bore >= s2.bore { s1 } else { s2 };
	let (od, len, m) = (spec.od, spec.length, spec.screw_m);
	let body = kernel_brep::extrude(&super::circle48(od * 0.5), len);
	let mut s = stepped_bore(body, bore1, bore2, len);
	// Two set screws per shaft side, 90° apart; the hole pierces one wall to the bore.
	for (z, dir) in [(len / 6.0, DVec3::X), (len / 3.0, DVec3::Y), (len - len / 3.0, DVec3::Y), (len - len / 6.0, DVec3::X)] {
		let at = dir * (od * 0.5) + DVec3::new(0.0, 0.0, z);
		s = tap_drill_hole(&s, at, -dir, m, HoleDepth::Through(od * 0.5), None).ok()?;
	}
	Some(s)
}

/// A one-piece **clamp (slit) shaft coupling** joining Ø`bore1` (z = 0) to Ø`bore2`
/// (z = L), both stocked table bores, body row from the larger: a full-length 2 mm
/// (2.5 mm for Ø8/10, 3 mm — table `slit ≈ od/9` rounded — held at 2/2.5 via the row
/// constant below) axial slit on +X severs the bore web, and two DIN 912 M-size cross
/// screws (axis −Y, at `L/4` and `3L/4`, mid-wall of the slit lobe) clamp it shut —
/// cut as ISO 273 clearance bores with DIN 974 counterbores on the +Y lobe. Honest
/// simplifications: the far lobe's thread is not modelled (the cross-bore stays at
/// clearance Ø; project convention), and the counterbore rim follows the curved OD.
/// Genus 4 (the slit opens the bore; each cross screw tunnels both lobes). `None`
/// outside the table.
pub fn clamp_coupling(bore1: f64, bore2: f64) -> Option<Solid> {
	let (s1, s2) = (clamp_coupling_spec(bore1)?, clamp_coupling_spec(bore2)?);
	let spec = if s1.bore >= s2.bore { s1 } else { s2 };
	let (od, len, m) = (spec.od, spec.length, spec.screw_m);
	let slit = if spec.bore <= 6.0 { 2.0 } else { 2.5 };
	let body = kernel_brep::extrude(&super::circle48(od * 0.5), len);
	let mut s = stepped_bore(body, bore1, bore2, len);
	// Full-length slit: from inside the bore void out past the OD on +X.
	let small_r = bore1.min(bore2) * 0.5;
	s = difference(&s, &cuboid(DVec3::new(small_r - 0.5, -slit * 0.5, -1.0), DVec3::new(od * 0.5 + 1.0, slit * 0.5, len + 1.0)));
	// Two cross screws through the slit, mid-wall of the +X lobe.
	let x = (bore1.max(bore2) * 0.5 + od * 0.5) * 0.5;
	let y_surf = (od * 0.5 * od * 0.5 - x * x).sqrt();
	for z in [len * 0.25, len * 0.75] {
		s = counterbore_hole(&s, DVec3::new(x, y_surf, z), -DVec3::Y, m, Fit::Medium, None).ok()?;
	}
	Some(s)
}

#[cfg(test)]
mod tests {
	use super::*;
	use kernel_brep::{intersection, tessellate_adaptive_tol, tessellate_default, validate, volume, VertexId};

	/// Faceted 48-gon disc area of radius `r` (the volume tests' polygon closed form).
	fn disc48(r: f64) -> f64 {
		48.0 * 0.5 * r * r * (2.0 * PI / 48.0).sin()
	}

	#[test]
	fn jaw_coupling_table_and_hub_geometry_hold() {
		// D25L30 (bore 8) and D40L50 (bore 12): genus-1 hubs, watertight on BOTH
		// tessellation routes, spanning exactly z 0…(L+jaw)/2 and r…od/2, volume inside
		// generous smooth bounds (body cylinder minus nothing … minus the full band
		// annulus minus the bore). Out-of-table OD and out-of-range bores refused.
		for (od, bore) in [(25.0, 8.0), (40.0, 12.0)] {
			let spec = jaw_coupling_spec(od).expect("table row");
			let hub = jaw_coupling_hub(od, bore).expect("stocked bore");
			let v = validate(&hub);
			let total = (spec.length + spec.jaw_height) * 0.5;
			let (mut z_max, mut r_max) = (0.0_f64, 0.0_f64);
			for i in 0..hub.vertex_count() as u32 {
				let p = hub.position(VertexId(i));
				z_max = z_max.max(p.z);
				r_max = r_max.max((p.x * p.x + p.y * p.y).sqrt());
			}
			let hub_len = total - spec.jaw_height;
			let body = disc48(od * 0.5) * total - PI * (bore * 0.5).powi(2) * total;
			// Over-bound of the band removal: the full annulus outside the spigot plus
			// the spigot's trimmed top half.
			let spig_h = (spec.jaw_height - 0.5) * 0.5;
			let band_void = (PI * (od * 0.5).powi(2) - PI * (JAW_SPIGOT_R * od).powi(2)) * spec.jaw_height
				+ PI * (JAW_SPIGOT_R * od).powi(2) * (spec.jaw_height - spig_h);
			let vol = volume(&hub).abs();
			assert!(
				v.closed
					&& v.manifold && v.genus == 1
					&& tessellate_default(&hub).is_watertight()
					&& tessellate_adaptive_tol(&hub, 0.01).is_watertight()
					&& (z_max - total).abs() < 1e-9
					&& (r_max - od * 0.5).abs() < 1e-9
					&& vol > body - band_void && vol < body
					&& hub_len > 0.0,
				"D{od} jaw hub Ø{bore}: want watertight×2 genus-1, z to {total}, r to {}, vol in ({:.0}, {body:.0}); got {v:?} z={z_max:.3} r={r_max:.3} vol={vol:.0}",
				od * 0.5,
				body - band_void
			);
		}
		assert!(
			jaw_coupling_hub(22.0, 8.0).is_none() && jaw_coupling_hub(25.0, 14.0).is_none() && jaw_coupling_hub(25.0, 2.0).is_none(),
			"OD 22 (not a row) and bores outside 4–12 on D25 must be refused"
		);
	}

	#[test]
	fn assembled_jaw_coupling_interleaves_without_interpenetration() {
		// The conjugacy proof (same treatment as the gear-mesh test): D25 hub A (bore 8,
		// jaws at 0/120/240°), hub B (bore 10) flipped onto the far end and rotated 60°
		// (jaws at 60/180/300°), spider legs on the 30° stations between them, floated
		// 0.05 mm off each hub face (real spiders float axially). All three pairwise
		// exact intersections must be EMPTY — the designed play is 1° per flank
		// (≈ 0.2 mm at the leg tip) and 0.2 mm radial, so any station/proportion error
		// of a degree or a few tenths would overlap by whole mm³.
		let spec = jaw_coupling_spec(25.0).expect("table row");
		let a = jaw_coupling_hub(25.0, 8.0).expect("hub A");
		let flip = DAffine3::from_cols(DVec3::X, -DVec3::Y, -DVec3::Z, DVec3::new(0.0, 0.0, spec.length));
		let b = jaw_coupling_hub(25.0, 10.0).expect("hub B").transformed(flip * DAffine3::from_rotation_z(PI / 3.0));
		let hub_len = (spec.length - spec.jaw_height) * 0.5;
		let spider =
			jaw_coupling_spider(25.0).expect("spider").transformed(DAffine3::from_translation(DVec3::new(0.0, 0.0, hub_len + 0.05)));
		// The spider is 0.1 thinner than the band: floated 0.05 off each hub face.
		let clash = |a: &Solid, b: &Solid, label: &str| {
			let i = intersection(a, b);
			let vol = if i.face_count() == 0 { 0.0 } else { volume(&i).abs() };
			assert!(vol < 0.01, "{label} must not interpenetrate; got {vol:.4} mm³");
		};
		clash(&a, &b, "hub A × hub B");
		clash(&a, &spider, "hub A × spider");
		clash(&b, &spider, "hub B × spider");
		// And the spider really is genus-1, watertight, ring-to-leg span as designed.
		let v = validate(&spider);
		assert!(
			v.closed && v.manifold && v.genus == 1 && tessellate_adaptive_tol(&spider, 0.01).is_watertight(),
			"D25 spider: want watertight genus-1; got {v:?}"
		);
	}

	#[test]
	fn set_screw_coupling_is_a_stepped_sleeve_with_four_tap_tunnels() {
		// 5×8 (the classic stepper→lead-screw joint, row from the Ø8 side: Ø16×25 M4)
		// and 12×12 (Ø24×35 M6): genus 5 = through-bore + 4 radial wall tunnels,
		// watertight on both routes, volume below the stepped-bore closed form and
		// above it minus the four tap-drill cylinders at full wall depth.
		for (b1, b2) in [(5.0_f64, 8.0_f64), (12.0, 12.0)] {
			let spec = set_screw_coupling_spec(b1.max(b2)).expect("row");
			let c = set_screw_coupling(b1, b2).expect("stocked bores");
			let v = validate(&c);
			let (od_r, len) = (spec.od * 0.5, spec.length);
			let sleeve =
				disc48(od_r) * len - disc48(b1.min(b2) * 0.5) * len - (disc48(b1.max(b2) * 0.5) - disc48(b1.min(b2) * 0.5)) * len * 0.5;
			let pitch = crate::parts::iso_coarse_pitch(spec.screw_m).expect("coarse pitch");
			// The drill tool is a circumscribed 32-gon (apothem-true Ø): area = πr²·1.0033.
			let tap_holes = 4.0 * PI * 1.01 * ((spec.screw_m - pitch) * 0.5).powi(2) * (od_r - b1.min(b2) * 0.5);
			let vol = volume(&c).abs();
			assert!(
				v.closed
					&& v.manifold && v.genus == 5
					&& tessellate_default(&c).is_watertight()
					&& tessellate_adaptive_tol(&c, 0.01).is_watertight()
					&& vol < sleeve && vol > sleeve - tap_holes,
				"set-screw coupling {b1}×{b2}: want watertight×2 genus-5, vol in ({:.0}, {sleeve:.0}); got {v:?} wt={} adaptive_wt={} vol={vol:.0}",
				sleeve - tap_holes,
				tessellate_default(&c).is_watertight(),
				tessellate_adaptive_tol(&c, 0.01).is_watertight()
			);
		}
		assert!(
			set_screw_coupling(7.0, 8.0).is_none() && set_screw_coupling(5.0, 14.0).is_none(),
			"Ø7 and Ø14 are not stocked set-screw coupling bores"
		);
	}

	#[test]
	fn clamp_coupling_is_a_slit_sleeve_with_two_cross_screws() {
		// 5×5 (Ø16×24 M3) and 8×10 (row from Ø10: Ø23×30 M4): the slit opens the bore
		// (genus drops to 0) and each cross screw tunnels both lobes (+2 each) → genus
		// 4; watertight on both routes; the slit gap spans the full length on +X
		// (vertices at y = ±slit/2 with x > bore radius); volume below sleeve-minus-slit
		// and above that minus screw/counterbore stock.
		for (b1, b2, slit) in [(5.0_f64, 5.0_f64, 2.0_f64), (8.0, 10.0, 2.5)] {
			let spec = clamp_coupling_spec(b1.max(b2)).expect("row");
			let c = clamp_coupling(b1, b2).expect("stocked bores");
			let v = validate(&c);
			let (od_r, len) = (spec.od * 0.5, spec.length);
			let small_r = b1.min(b2) * 0.5;
			let sleeve = disc48(od_r) * len - disc48(small_r) * len - (disc48(b1.max(b2) * 0.5) - disc48(small_r)) * len * 0.5;
			let slit_cut = (od_r - small_r) * slit * len; // chord-level over-bound of the web removed
												 // DIN 974 counterbores run ~1 mm over the DIN 912 head Ø dk; bound the screw
												 // stock by two full-length bores at (dk/2 + 1).
			let cbore = crate::parts::din912_dims(spec.screw_m).map(|d| d.0).unwrap_or(2.0 * spec.screw_m);
			let screws = 2.0 * (PI * (cbore * 0.5 + 1.0).powi(2) * 2.0 * od_r);
			let slit_verts = (0..c.vertex_count() as u32)
				.map(|i| c.position(VertexId(i)))
				.filter(|p| (p.y.abs() - slit * 0.5).abs() < 1e-9 && p.x > small_r)
				.count();
			let vol = volume(&c).abs();
			assert!(
				v.closed
					&& v.manifold && v.genus == 4
					&& tessellate_default(&c).is_watertight()
					&& tessellate_adaptive_tol(&c, 0.01).is_watertight()
					&& slit_verts >= 8
					&& vol < sleeve - slit_cut * 0.8
					&& vol > sleeve - slit_cut - screws,
				"clamp coupling {b1}×{b2}: want watertight×2 genus-4 with a ±{} slit, vol in ({:.0}, {:.0}); got {v:?} wt={} adaptive_wt={} slit_verts={slit_verts} vol={vol:.0}",
				slit * 0.5,
				sleeve - slit_cut - screws,
				sleeve - slit_cut * 0.8,
				tessellate_default(&c).is_watertight(),
				tessellate_adaptive_tol(&c, 0.01).is_watertight()
			);
		}
		assert!(
			clamp_coupling(6.35, 6.35).is_none() && clamp_coupling(5.0, 16.0).is_none(),
			"Ø6.35 (set-screw-only row) and Ø16 are not stocked clamp bores"
		);
	}
}
