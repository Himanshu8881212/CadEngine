// Copyright (c) LMCAD. Licensed under the MIT License.

//! **Pins and retaining clips**: ISO 2338 parallel dowel pins, DIN 471/472 circlips
//! (retaining rings) for shafts and bores — the ring itself as a C-shaped exact B-rep —
//! and the matching standard groove cuts. Dimension tables are copied from the published
//! standards with the source cited next to each table; all values mm, all bores/shafts
//! **diameters**.

use super::{circle48, ring_cutter};
use kernel_brep::geom::perp_basis;
use kernel_brep::math::{DAffine3, DMat3, DVec2, DVec3};
use kernel_brep::{difference, extrude_with_holes, revolve, Solid};
use std::f64::consts::PI;

/// ISO 2338 parallel-pin nominal diameters, 1–12 mm (the standard continues to 50).
/// Source: ISO 2338:1997 nominal-diameter series as listed at
/// fastenermart.com/iso-2338-dowel-pins (1, 1.5, 2, 2.5, 3, 4, 5, 6, 8, 10, 12).
const ISO2338_DIAMETERS: [f64; 11] = [1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0];

/// An **ISO 2338 parallel dowel pin** (tolerance class m6 in the standard; the few-µm
/// fit allowance is below model resolution, so the body is built at the nominal Ø):
/// a Ø`d` × `length` cylinder with the standard ~15° insertion chamfer, axial length
/// 0.2·d, at **both** ends.
///
/// Honest approximations (the standard itself marks the end features "≈"):
/// - ISO 2338 figures one chamfered and one crowned end; both ends here carry the 15°
///   chamfer (fastenermart.com lists C = 1.2 mm at Ø6 = 0.2·d, 15°). Ø and length —
///   the controlled, fit-critical dimensions — are exact.
///
/// Built as one revolved profile (64 sectors), watertight by construction, genus 0.
/// `None` for diameters outside the table or a length too short for its two chamfers.
pub fn dowel_pin(d: f64, length: f64) -> Option<Solid> {
	ISO2338_DIAMETERS.iter().find(|&&n| (n - d).abs() < 1e-9)?;
	let c = 0.2 * d; // chamfer axial length per the published C ≈ 0.2·d
				  // NaN-safe rejection: `!(x > y)` via the conjunction so NaN lengths are refused too.
	if !(length > 2.0 * c && length.is_finite()) {
		return None;
	}
	let r = d * 0.5;
	let dr = c * 15.0_f64.to_radians().tan(); // radial drop of the 15° chamfer
	let profile = [
		DVec2::new(0.0, 0.0),
		DVec2::new(r - dr, 0.0),
		DVec2::new(r, c),
		DVec2::new(r, length - c),
		DVec2::new(r - dr, length),
		DVec2::new(0.0, length),
	];
	Some(revolve(&profile, 64))
}

/// One circlip table row: every dimension the ring and its groove need.
/// `d1` nominal (shaft Ø for DIN 471, bore Ø for DIN 472), ring thickness `s`,
/// groove Ø `d2`, groove width `m`, lug width `a`, radial section width `b`,
/// lug-hole Ø `d5`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CirclipSpec {
	/// Nominal shaft (DIN 471) or bore (DIN 472) diameter.
	pub d1: f64,
	/// Ring (and so axial groove-fit) thickness.
	pub s: f64,
	/// Groove diameter — *smaller* than `d1` for shafts, *larger* for bores.
	pub d2: f64,
	/// Groove width (slightly wider than `s`).
	pub m: f64,
	/// Lug (eyelet) width.
	pub a: f64,
	/// Maximum radial section width of the ring body.
	pub b: f64,
	/// Pliers-hole diameter in each lug.
	pub d5: f64,
}

/// DIN 471 external (shaft) retaining-ring table: `(d1, s, d2, m, a, b, d5)`.
/// Source: DIN 471 dimension table as published at fasteners.eu/standards/din/471 (mm).
const DIN471: [CirclipSpec; 7] = [
	CirclipSpec { d1: 8.0, s: 0.8, d2: 7.6, m: 0.9, a: 3.2, b: 1.5, d5: 1.2 },
	CirclipSpec { d1: 10.0, s: 1.0, d2: 9.6, m: 1.1, a: 3.3, b: 1.8, d5: 1.5 },
	CirclipSpec { d1: 12.0, s: 1.0, d2: 11.5, m: 1.1, a: 3.3, b: 1.8, d5: 1.7 },
	CirclipSpec { d1: 15.0, s: 1.0, d2: 14.3, m: 1.1, a: 3.6, b: 2.2, d5: 1.7 },
	CirclipSpec { d1: 20.0, s: 1.2, d2: 19.0, m: 1.3, a: 4.0, b: 2.6, d5: 2.0 },
	CirclipSpec { d1: 25.0, s: 1.2, d2: 23.9, m: 1.3, a: 4.4, b: 3.0, d5: 2.0 },
	CirclipSpec { d1: 30.0, s: 1.5, d2: 28.6, m: 1.6, a: 5.0, b: 3.5, d5: 2.0 },
];

/// DIN 472 internal (bore) retaining-ring table: `(d1, s, d2, m, a, b, d5)`.
/// Source: DIN 472 dimension table as published at fasteners.eu/standards/din/472 (mm).
const DIN472: [CirclipSpec; 8] = [
	CirclipSpec { d1: 16.0, s: 1.0, d2: 16.8, m: 1.1, a: 3.8, b: 2.0, d5: 1.7 },
	CirclipSpec { d1: 20.0, s: 1.0, d2: 21.0, m: 1.1, a: 4.2, b: 2.3, d5: 2.0 },
	CirclipSpec { d1: 22.0, s: 1.0, d2: 23.0, m: 1.1, a: 4.2, b: 2.5, d5: 2.0 },
	CirclipSpec { d1: 26.0, s: 1.2, d2: 27.2, m: 1.3, a: 4.7, b: 2.8, d5: 2.0 },
	CirclipSpec { d1: 32.0, s: 1.2, d2: 33.7, m: 1.3, a: 5.4, b: 3.2, d5: 2.5 },
	CirclipSpec { d1: 35.0, s: 1.5, d2: 37.0, m: 1.6, a: 5.4, b: 3.4, d5: 2.5 },
	CirclipSpec { d1: 42.0, s: 1.75, d2: 44.5, m: 1.85, a: 5.9, b: 4.1, d5: 2.5 },
	CirclipSpec { d1: 47.0, s: 1.75, d2: 49.5, m: 1.85, a: 6.4, b: 4.4, d5: 2.5 },
];

/// The DIN 471 row for a nominal shaft Ø `d` (8, 10, 12, 15, 20, 25, 30), or `None`.
pub fn din471_spec(d: f64) -> Option<CirclipSpec> {
	DIN471.iter().find(|r| (r.d1 - d).abs() < 1e-9).copied()
}

/// The DIN 472 row for a nominal bore Ø `d` (16, 20, 22, 26, 32, 35, 42, 47), or `None`.
pub fn din472_spec(d: f64) -> Option<CirclipSpec> {
	DIN472.iter().find(|r| (r.d1 - d).abs() < 1e-9).copied()
}

/// The C-ring outline of a circlip in the XY plane, gap centred on +X: a body annulus
/// `r_in..r_out` spanning all angles outside the gap (half-angle `gap`), with a lug
/// sector of angular width `lug_arc` and radius reach `r_lug` at each gap end. For
/// external rings `r_lug > r_out` (lugs point outward); for internal rings the caller
/// passes `r_out` as the *body* outer radius and `r_lug < r_in` reaches inward, with
/// `inward = true` flipping which boundary carries the lugs. Arcs are sampled every
/// ~4°; the polygon is wound counter-clockwise.
fn c_ring_outline(r_in: f64, r_out: f64, r_lug: f64, gap: f64, lug_arc: f64, inward: bool) -> Vec<DVec2> {
	let at = |r: f64, a: f64| DVec2::new(r * a.cos(), r * a.sin());
	let arc = |pts: &mut Vec<DVec2>, r: f64, a0: f64, a1: f64| {
		let n = (((a1 - a0).abs() / (4.0_f64).to_radians()).ceil() as usize).max(1);
		for i in 0..=n {
			pts.push(at(r, a0 + (a1 - a0) * i as f64 / n as f64));
		}
	};
	let (a0, a1) = (gap, 2.0 * PI - gap); // gap faces
	let mut pts: Vec<DVec2> = Vec::new();
	if inward {
		// Lugs hang off the inner boundary; the outer boundary is one full arc.
		arc(&mut pts, r_out, a0, a1); // outer body arc, CCW
		pts.push(at(r_lug, a1)); // down the gap face to the lug reach
		arc(&mut pts, r_lug, a1, a1 - lug_arc); // lug B inner arc (CW = interior left)
		pts.push(at(r_in, a1 - lug_arc)); // lug B shoulder up to the body bore
		arc(&mut pts, r_in, a1 - lug_arc, a0 + lug_arc); // body inner arc, CW
		pts.push(at(r_lug, a0 + lug_arc)); // lug A shoulder
		arc(&mut pts, r_lug, a0 + lug_arc, a0); // lug A inner arc
	} else {
		// Lugs stand on the outer boundary; the inner boundary is one full arc.
		arc(&mut pts, r_lug, a0, a0 + lug_arc); // lug A outer arc, CCW
		pts.push(at(r_out, a0 + lug_arc)); // lug A shoulder down to the body
		arc(&mut pts, r_out, a0 + lug_arc, a1 - lug_arc); // body outer arc
		pts.push(at(r_lug, a1 - lug_arc)); // lug B shoulder
		arc(&mut pts, r_lug, a1 - lug_arc, a1); // lug B outer arc
		pts.push(at(r_in, a1)); // down the gap face
		arc(&mut pts, r_in, a1, a0); // inner arc, CW (interior on the left)
	}
	pts
}

/// Shared circlip body builder: outline extruded to thickness `s`, then the two Ø`d5`
/// lug holes (angular centre `gap + lug_arc/2` each side) drilled by exact boolean
/// differences with 24-segment analytic cylinders — not `extrude_with_holes` hole
/// loops, so the caps carry no inner loops, the adaptive tessellation stays watertight
/// and STL export routes exact instead of voxel-healed (FRICTION #6).
#[allow(clippy::too_many_arguments)] // the nine values ARE the part's datum set
fn circlip_body(r_in: f64, r_out: f64, r_lug: f64, r_hole: f64, gap: f64, lug_arc: f64, d5: f64, s: f64, inward: bool) -> Solid {
	let outline = c_ring_outline(r_in, r_out, r_lug, gap, lug_arc, inward);
	let holes: Vec<(DVec2, f64, usize)> = [gap + 0.5 * lug_arc, 2.0 * PI - gap - 0.5 * lug_arc]
		.iter()
		.map(|&a| (DVec2::new(r_hole * a.cos(), r_hole * a.sin()), d5 * 0.5, 24))
		.collect();
	super::extrude_bored(&outline, s, &holes, &[])
}

/// A **DIN 471 external circlip** (retaining ring for shafts) drawn in its *installed*
/// state, seated in the groove of a nominal Ø`shaft_d` shaft: body annulus from the
/// groove radius `d2/2` out to `d2/2 + b`, ring thickness `s`, with the two pliers
/// lugs (width `a`, hole Ø `d5`) flanking the gap on the +X side. Genus 2 (the two
/// lug holes), watertight on both the default and adaptive tessellations.
///
/// Honest approximations: the real ring's section tapers toward the gap and the lug
/// outline is a styled eyelet; here the section is the constant table width `b`, the
/// lugs are annular tabs reaching to `d2/2 + a`, and the gap half-angle is sized so
/// the gap clears ~1.5 lug holes — representative, not die-exact. Ø`d2`, `s`, `b`,
/// `a`, `d5` are the table values. `None` outside the table (Ø8–30).
pub fn circlip_external(shaft_d: f64) -> Option<Solid> {
	let spec = din471_spec(shaft_d)?;
	let r_in = spec.d2 * 0.5;
	let (r_out, r_lug) = (r_in + spec.b, r_in + spec.a);
	let r_hole = r_in + spec.a * 0.5;
	let gap = (0.75 * spec.d5) / r_in; // gap half-angle: clears ~1.5 holes at the bore
	let lug_arc = spec.a / r_hole; // lug angular width ≈ a at the hole radius
	Some(circlip_body(r_in, r_out, r_lug, r_hole, gap, lug_arc, spec.d5, spec.s, false))
}

/// A **DIN 472 internal circlip** (retaining ring for bores) drawn in its *installed*
/// state, seated in the groove of a nominal Ø`bore_d` bore: body annulus from
/// `d2/2 − b` out to the groove radius `d2/2`, ring thickness `s`, with the two pliers
/// lugs reaching *inward* to `d2/2 − a`. Genus 2, watertight on both tessellations. Same
/// honest simplifications as [`circlip_external`]; table Ø16–47, else `None`.
pub fn circlip_internal(bore_d: f64) -> Option<Solid> {
	let spec = din472_spec(bore_d)?;
	let r_out = spec.d2 * 0.5;
	let (r_in, r_lug) = (r_out - spec.b, r_out - spec.a);
	let r_hole = r_out - spec.a * 0.5;
	let gap = (0.75 * spec.d5) / r_lug;
	let lug_arc = spec.a / r_hole;
	Some(circlip_body(r_in, r_out, r_lug, r_hole, gap, lug_arc, spec.d5, spec.s, true))
}

/// Cut the **DIN 471 circlip groove** into a shaft: an annular slot of root Ø `d2`
/// and width `m` (both from the table for the nominal Ø`shaft_d`), spanning
/// `[at, at + m·axis]` along the shaft axis (`at` is the groove face nearer the
/// retained part; `axis` points along the shaft). The cutter is a ring whose end caps
/// cross the shaft wall transversely — the proven lathe-groove boolean route. `None`
/// outside the table.
pub fn circlip_groove_external(solid: &Solid, at: DVec3, axis: DVec3, shaft_d: f64) -> Option<Solid> {
	let spec = din471_spec(shaft_d)?;
	let axis = axis.try_normalize()?;
	Some(difference(solid, &ring_cutter(at, axis, spec.d2 * 0.5, shaft_d * 0.5, spec.m)))
}

/// Cut the **DIN 472 circlip groove** into a bore wall: an annular channel of root Ø
/// `d2` (> bore) and width `m`, spanning `[at, at + m·axis]` (`at` on the bore axis;
/// `axis` along it). The cutter is an annulus from just inside the bore void out to
/// the groove root, so only the bore wall is machined. `None` outside the table.
pub fn circlip_groove_internal(solid: &Solid, at: DVec3, axis: DVec3, bore_d: f64) -> Option<Solid> {
	let spec = din472_spec(bore_d)?;
	let axis = axis.try_normalize()?;
	let inner = (bore_d * 0.5 - 1.0).max(bore_d * 0.25); // strictly inside the bore void
	let (e1, e2) = perp_basis(axis);
	let cutter = extrude_with_holes(&circle48(spec.d2 * 0.5), &[circle48(inner)], spec.m)
		.transformed(DAffine3::from_mat3_translation(DMat3::from_cols(e1, e2, axis), at));
	Some(difference(solid, &cutter))
}

#[cfg(test)]
mod tests {
	use super::*;
	use kernel_brep::{cylinder, tessellate_default, validate, volume, VertexId};

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
	fn dowel_pins_are_chamfered_cylinders_of_the_exact_table_volume() {
		// Two ISO 2338 sizes (Ø6×24, Ø2×8): genus-0 watertight revolve, volume =
		// cylinder minus two chamfer frustum rebates (closed form, 1% covers the
		// 64-gon faceting), and out-of-table / too-short requests refused.
		for (d, len) in [(6.0, 24.0), (2.0, 8.0)] {
			let p = dowel_pin(d, len).expect("table size");
			let v = validate(&p);
			let (r, c) = (d * 0.5, 0.2 * d);
			let dr = c * 15.0_f64.to_radians().tan();
			let frustum = PI * c / 3.0 * (r * r + r * (r - dr) + (r - dr) * (r - dr));
			let expected = PI * r * r * (len - 2.0 * c) + 2.0 * frustum;
			assert!(
				v.closed
					&& v.manifold && v.genus == 0
					&& tessellate_default(&p).is_watertight()
					&& (volume(&p).abs() - expected).abs() / expected < 0.01,
				"Ø{d}×{len} dowel pin: want watertight genus-0 ~{expected:.2}mm³; got {v:?} vol={:.2}",
				volume(&p).abs()
			);
		}
		assert!(
			dowel_pin(7.0, 20.0).is_none() && dowel_pin(6.0, 2.0).is_none(),
			"out-of-table Ø7 and a 2 mm pin (shorter than its chamfers) must be refused"
		);
	}

	#[test]
	fn external_circlips_match_the_din471_table_and_carry_two_lug_holes() {
		// DIN 471 for Ø20 and Ø10 shafts. The ring must be genus 2 (two pliers holes),
		// watertight, span exactly groove radius d2/2 → d2/2 + a (body + lugs), and its
		// volume must sit within 2% of the closed-form sector sum.
		for shaft_d in [20.0, 10.0] {
			let spec = din471_spec(shaft_d).expect("table row");
			let ring = circlip_external(shaft_d).expect("table size");
			let v = validate(&ring);
			let rr = radii(&ring);
			let (r_min, r_max) = rr.iter().fold((f64::INFINITY, 0.0_f64), |(lo, hi), &r| (lo.min(r), hi.max(r)));
			let (r_in, r_out, r_lug) = (spec.d2 * 0.5, spec.d2 * 0.5 + spec.b, spec.d2 * 0.5 + spec.a);
			let r_hole = r_in + spec.a * 0.5;
			let (gap, lug_arc) = ((0.75 * spec.d5) / r_in, spec.a / r_hole);
			let body = (PI - gap - lug_arc) * (r_out * r_out - r_in * r_in);
			let lugs = lug_arc * (r_lug * r_lug - r_in * r_in);
			let expected = (body + lugs - 2.0 * PI * (spec.d5 * 0.5).powi(2)) * spec.s;
			assert!(
				v.closed
					&& v.manifold && v.genus == 2
					&& tessellate_default(&ring).is_watertight()
					&& (r_min - r_in).abs() < 1e-9
					&& (r_max - r_lug).abs() < 1e-9
					&& (volume(&ring).abs() - expected).abs() / expected < 0.02,
				"DIN 471 Ø{shaft_d}: want watertight genus-2 spanning r {r_in:.2}–{r_lug:.2}, ~{expected:.1}mm³; got {v:?} r=[{r_min:.3},{r_max:.3}] vol={:.1}",
				volume(&ring).abs()
			);
		}
		assert!(circlip_external(9.0).is_none(), "Ø9 is not a DIN 471 table size");
	}

	#[test]
	fn internal_circlips_match_the_din472_table_and_reach_inward() {
		// DIN 472 for Ø32 and Ø16 bores: genus 2, watertight, spanning groove radius
		// d2/2 inward to d2/2 − a, volume within 2% of the sector sum.
		for bore_d in [32.0, 16.0] {
			let spec = din472_spec(bore_d).expect("table row");
			let ring = circlip_internal(bore_d).expect("table size");
			let v = validate(&ring);
			let rr = radii(&ring);
			let (r_min, r_max) = rr.iter().fold((f64::INFINITY, 0.0_f64), |(lo, hi), &r| (lo.min(r), hi.max(r)));
			let (r_out, r_in, r_lug) = (spec.d2 * 0.5, spec.d2 * 0.5 - spec.b, spec.d2 * 0.5 - spec.a);
			let r_hole = r_out - spec.a * 0.5;
			let (gap, lug_arc) = ((0.75 * spec.d5) / r_lug, spec.a / r_hole);
			// Sector sum: the body annulus (r_in..r_out) outside the lug arcs, plus the
			// two deeper lug sectors (r_lug..r_out), minus the two pliers holes.
			let body = (PI - gap - lug_arc) * (r_out * r_out - r_in * r_in);
			let lugs = lug_arc * (r_out * r_out - r_lug * r_lug);
			let expected = (body + lugs - 2.0 * PI * (spec.d5 * 0.5).powi(2)) * spec.s;
			assert!(
				v.closed
					&& v.manifold && v.genus == 2
					&& tessellate_default(&ring).is_watertight()
					&& (r_min - r_lug).abs() < 1e-9
					&& (r_max - r_out).abs() < 1e-9
					&& (volume(&ring).abs() - expected).abs() / expected < 0.02,
				"DIN 472 Ø{bore_d}: want watertight genus-2 spanning r {r_lug:.2}–{r_out:.2}, ~{expected:.1}mm³; got {v:?} r=[{r_min:.3},{r_max:.3}] vol={:.1}",
				volume(&ring).abs()
			);
		}
		assert!(circlip_internal(17.0).is_none(), "Ø17 is not a DIN 472 table size");
	}

	#[test]
	fn external_groove_turns_the_table_slot_into_a_shaft() {
		// Ø20 shaft, groove at z = 12: root must be Ø19 (d2), width 1.3 (m), the part
		// stays genus 0 and loses one annulus ring of material (1% band for the
		// faceted 48-gon walls).
		let (d, len, z0) = (20.0, 40.0, 12.0);
		let spec = din471_spec(d).expect("Ø20 row");
		let shaft = cylinder(DVec3::ZERO, DVec3::Z, d * 0.5, len, 48);
		let grooved = circlip_groove_external(&shaft, DVec3::new(0.0, 0.0, z0), DVec3::Z, d).expect("table size");
		let v = validate(&grooved);
		let root_verts = radii(&grooved).iter().filter(|r| (**r - spec.d2 * 0.5).abs() < 1e-9).count();
		let ring48 = |r: f64| 48.0 * 0.5 * r * r * (2.0 * PI / 48.0).sin(); // 48-gon disc area
		let expected = ring48(d * 0.5) * len - (ring48(d * 0.5) - ring48(spec.d2 * 0.5)) * spec.m;
		assert!(
			v.closed
				&& v.manifold && v.genus == 0
				&& tessellate_default(&grooved).is_watertight()
				&& root_verts > 0
				&& (volume(&grooved).abs() - expected).abs() / expected < 0.01,
			"DIN 471 groove in Ø{d} shaft: want watertight genus-0 with root vertices at r={}, ~{expected:.0}mm³; got {v:?} roots={root_verts} vol={:.0}",
			spec.d2 * 0.5,
			volume(&grooved).abs()
		);
	}

	#[test]
	fn internal_groove_channels_the_bore_of_a_housing() {
		// A 40×40×20 block bored Ø16 through, groove at z = 6: the channel root must
		// reach Ø16.8 (d2), the part stays genus 1, and gains the annulus ring volume.
		let bore_d = 16.0;
		let spec = din472_spec(bore_d).expect("Ø16 row");
		let square = vec![DVec2::new(20.0, 20.0), DVec2::new(-20.0, 20.0), DVec2::new(-20.0, -20.0), DVec2::new(20.0, -20.0)];
		let housing = extrude_with_holes(&square, &[circle48(bore_d * 0.5)], 20.0);
		let grooved = circlip_groove_internal(&housing, DVec3::new(0.0, 0.0, 6.0), DVec3::Z, bore_d).expect("table size");
		let v = validate(&grooved);
		let root_verts = radii(&grooved).iter().filter(|r| (**r - spec.d2 * 0.5).abs() < 1e-9).count();
		let ring48 = |r: f64| 48.0 * 0.5 * r * r * (2.0 * PI / 48.0).sin();
		let expected = volume(&housing).abs() + (ring48(spec.d2 * 0.5) - ring48(bore_d * 0.5)) * spec.m;
		assert!(
			v.closed
				&& v.manifold && v.genus == 1
				&& tessellate_default(&grooved).is_watertight()
				&& root_verts > 0
				&& (volume(&grooved).abs() - expected).abs() / expected < 0.01,
			"DIN 472 groove in Ø{bore_d} housing: want watertight genus-1 with channel root at r={}, ~{expected:.0}mm³; got {v:?} roots={root_verts} vol={:.0}",
			spec.d2 * 0.5,
			volume(&grooved).abs()
		);
	}
}
