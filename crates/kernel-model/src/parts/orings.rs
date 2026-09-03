// Copyright (c) LMCAD. Licensed under the MIT License.

//! **O-rings and their machined gland grooves**: AS568 dash sizes with Parker
//! radial (piston) glands, plus **metric cord** sections with **face-seal (axial)
//! glands** — circular and racetrack — for housing lids and ports whose perimeter
//! outruns the AS568 table (campaign/friction/ENGINE.md #10). The ring itself is the exact analytic
//! torus; gland cuts are boolean grooves whose walls cross the host face
//! transversely. Dimension tables are copied from the published standards with the
//! source cited next to each table; all values mm, all bores/shafts **diameters**.

use super::{circle48, ring_cutter};
use kernel_brep::geom::perp_basis;
use kernel_brep::math::{DAffine3, DMat3, DVec2, DVec3};
use kernel_brep::{difference, extrude_with_holes, torus, Solid};
use std::f64::consts::PI;

/// One AS568 O-ring row plus its Parker static-gland dimensions.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct As568Spec {
	/// AS568 dash number (the "-214" of an AS568-214 ring).
	pub dash: u16,
	/// Nominal inside diameter (mm).
	pub id: f64,
	/// Nominal cross-section diameter W (mm).
	pub cs: f64,
	/// Radial gland (groove) depth L for an industrial static seal (mm).
	pub gland_depth: f64,
	/// Groove width G for an industrial static seal (mm).
	pub groove_width: f64,
}

/// Parker industrial **static** radial-gland dimensions by cross-section:
/// `(W, gland depth L, groove width G)`. Source: Parker O-Ring Handbook ORD 5700,
/// Design Chart 4-2 (industrial static O-ring seal glands), chart minima converted
/// from inches — W 0.070/0.103/0.139/0.210/0.275 → L 0.050/0.081/0.111/0.170/0.226,
/// G 0.093/0.140/0.187/0.281/0.375. The resulting squeeze (18–29%) and gland fill
/// (70–83%) sit inside Parker's recommended static bands — asserted in the tests.
const PARKER_GLAND: [(f64, f64, f64); 5] = [
	(1.78, 1.27, 2.36),
	(2.62, 2.06, 3.56),
	(3.53, 2.82, 4.75),
	(5.33, 4.32, 7.14),
	(6.99, 5.74, 9.53),
];

/// AS568 dash-number table: `(dash, ID, W)` in mm. Source: SAE AS568 standard sizes
/// as published in the Parker O-Ring Handbook ORD 5700 size tables (inch ID/W
/// converted; e.g. -214 = 0.984″ × 0.139″ = 24.99 × 3.53 mm). A working subset of
/// the -0xx (W 1.78), -1xx (2.62), -2xx (3.53) and -3xx (5.33) series.
const AS568: [(u16, f64, f64); 15] = [
	(10, 6.07, 1.78),
	(12, 9.25, 1.78),
	(14, 12.42, 1.78),
	(16, 15.60, 1.78),
	(18, 18.77, 1.78),
	(20, 21.95, 1.78),
	(110, 9.19, 2.62),
	(112, 12.37, 2.62),
	(115, 17.12, 2.62),
	(120, 25.07, 2.62),
	(210, 18.64, 3.53),
	(214, 24.99, 3.53),
	(218, 31.34, 3.53),
	(222, 37.69, 3.53),
	(325, 37.47, 5.33),
];

/// The AS568 row (with its Parker static-gland dimensions) for a dash number
/// (10, 12, 14, 16, 18, 20, 110, 112, 115, 120, 210, 214, 218, 222, 325), or `None`.
pub fn as568_spec(dash: u16) -> Option<As568Spec> {
	let (dash, id, cs) = *AS568.iter().find(|r| r.0 == dash)?;
	// Every table cross-section is one of the five gland rows by construction.
	let (_, gland_depth, groove_width) = *PARKER_GLAND.iter().find(|g| (g.0 - cs).abs() < 1e-9)?;
	Some(As568Spec { dash, id, cs, gland_depth, groove_width })
}

/// An **AS568 O-ring** at its free (unsqueezed) nominal size: the exact analytic
/// torus of major radius `(ID + W)/2` and tube radius `W/2`, centred at the origin
/// with its axis along +Z (faceted 48 around the ring × 24 around the tube; the
/// exact torus surface rides on every face tag). Genus 1. `None` for a dash number
/// outside the table.
pub fn o_ring(dash: u16) -> Option<Solid> {
	let spec = as568_spec(dash)?;
	Some(torus(DVec3::ZERO, DVec3::Z, (spec.id + spec.cs) * 0.5, spec.cs * 0.5, 48, 24))
}

/// Cut an **AS568 static O-ring gland groove** into a shaft (male/piston gland): an
/// annular slot of root Ø = the dash's nominal ID, radial depth `L` and width `G`
/// from the cited Parker chart, spanning `[at, at + G·axis]` (`at` on the shaft
/// axis, `axis` along it). Designed for the gland's nominal shaft Ø `ID + 2·L` — at
/// that OD the ring is squeezed by the chart's static design squeeze; the cutter
/// clears a Ø `ID + 2·L + 4` envelope, so oversized stock is not supported. The
/// rectangular groove omits the chart's 0°–5° wall draft and corner break radii
/// (manufacturing detail below model resolution). Parker recommends sizing so the
/// ring ID is stretched 1–5% on the root; seating at the exact nominal ID (0%
/// stretch) keeps the geometry table-exact. `None` outside the table or for a
/// degenerate axis.
pub fn o_ring_groove(solid: &Solid, at: DVec3, axis: DVec3, dash: u16) -> Option<Solid> {
	let spec = as568_spec(dash)?;
	let axis = axis.try_normalize()?;
	let root_r = spec.id * 0.5;
	Some(difference(solid, &ring_cutter(at, axis, root_r, root_r + spec.gland_depth, spec.groove_width)))
}

/// A **metric O-ring cord** section and its static **face-seal (axial) gland**.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MetricCordGland {
	/// Cord cross-section diameter d2 (mm).
	pub cord_d: f64,
	/// Axial gland (groove) depth (mm) — the squeezed cord height.
	pub gland_depth: f64,
	/// Groove width (mm).
	pub groove_width: f64,
}

/// Stocked metric O-ring cord cross-sections (mm): the ISO 3601-1 G-series
/// sections (1.78, 2.62, 3.53, 5.33) plus the common metric cord-stock diameters
/// (1–7 mm) sold by the running metre (e.g. ERIKS / Kremer NBR cord).
const METRIC_CORDS: [f64; 13] = [1.0, 1.5, 1.78, 2.0, 2.5, 2.62, 3.0, 3.53, 4.0, 5.0, 5.33, 6.0, 7.0];

/// The static **face-seal gland** for a metric cord section: depth `0.75·d2`
/// (25% squeeze) and width `π·d2/2.25` (75% gland fill).
///
/// Honest sourcing: Parker's ORD 5700 face-seal charts publish **inch** sections
/// only, so these metric rows are *derived*, not copied — the design squeeze and
/// fill are pinned to the midpoints of Parker's recommended static bands (squeeze
/// 17–30%, fill 60–85%; the derivation is the engineering content and the tests
/// assert both ratios land mid-band for every row). The classic Ø2 cord row works
/// out to 1.5 deep × 2.79 wide — the hand-derived 1.5 × 2.7 of a typical printed
/// housing lid, now table-driven. `None` for a cross-section that is not a
/// stocked cord size.
pub fn metric_cord_gland(cord_d: f64) -> Option<MetricCordGland> {
	METRIC_CORDS.iter().find(|&&c| (c - cord_d).abs() < 1e-9)?;
	Some(MetricCordGland {
		cord_d,
		gland_depth: 0.75 * cord_d,                // 25% squeeze
		groove_width: PI * cord_d / (4.0 * 0.5625), // fill = π·d²/4 / (depth·width) = 75%
	})
}

/// A **metric O-ring / cord ring** at its free nominal size: the exact analytic
/// torus of inside diameter `ring_id` and cord cross-section `cord_d` (a stocked
/// metric size — see [`metric_cord_gland`]), centred at the origin with its axis
/// along +Z (faceted 48 × 24 like [`o_ring`]). This is the display/BOM body for
/// glued-cord and ISO 3601 metric rings whose ID outruns the AS568 table. Genus 1.
/// `None` for an unstocked cord or a degenerate `ring_id`.
pub fn o_ring_cord(ring_id: f64, cord_d: f64) -> Option<Solid> {
	metric_cord_gland(cord_d)?;
	if !(ring_id > 0.0 && ring_id.is_finite()) {
		return None;
	}
	Some(torus(DVec3::ZERO, DVec3::Z, (ring_id + cord_d) * 0.5, cord_d * 0.5, 48, 24))
}

/// Sink a hole-loop-free annular/racetrack channel cutter into a face: `outer` and
/// `inner` are the channel walls in the face plane (CCW), `at` a point on the face,
/// `axis` the outward face normal, `depth` the channel depth. The cutter spans
/// `[at − depth·axis, at + 1·axis]` (1 mm overshoot above the face, so no cutter
/// cap is coplanar with it).
fn face_channel_cut(solid: &Solid, at: DVec3, axis: DVec3, outer: &[DVec2], inner: &[DVec2], depth: f64) -> Solid {
	let (e1, e2) = perp_basis(axis);
	let cutter = extrude_with_holes(outer, &[inner.to_vec()], depth + 1.0)
		.transformed(DAffine3::from_mat3_translation(DMat3::from_cols(e1, e2, axis), at - axis * depth));
	difference(solid, &cutter)
}

/// Cut a **circular face-seal (axial) O-ring gland** into a flat face: an annular
/// channel whose centreline circle has diameter `gland_center_d`, with depth and
/// width from the metric-cord table ([`metric_cord_gland`]) for `cord_d`. `at` is
/// the gland centre **on the face** and `axis` the outward face normal; the groove
/// sinks into the material. Size the cord (or ISO 3601 ring) to the centreline
/// circumference `π·gland_center_d`. Walls are 48-gons (the library's standard
/// boring resolution); both cross the host face transversely. The rectangular
/// section omits wall draft and corner break radii like [`o_ring_groove`]. `None`
/// for an unstocked cord, a degenerate axis, or a centreline too tight for the
/// groove width (`gland_center_d ≤ groove width`).
pub fn o_ring_face_gland(solid: &Solid, at: DVec3, axis: DVec3, gland_center_d: f64, cord_d: f64) -> Option<Solid> {
	let g = metric_cord_gland(cord_d)?;
	let axis = axis.try_normalize()?;
	let (r_in, r_out) = (gland_center_d * 0.5 - g.groove_width * 0.5, gland_center_d * 0.5 + g.groove_width * 0.5);
	if !(r_in > 0.0 && gland_center_d.is_finite()) {
		return None;
	}
	Some(face_channel_cut(solid, at, axis, &circle48(r_out), &circle48(r_in), g.gland_depth))
}

/// A rounded-rectangle (racetrack) outline centred at the origin: overall size
/// `x_len × y_len`, corner radius `r`, wound CCW with 12 segments per quarter
/// corner (the 48-gon resolution). `r == 0` degenerates to the plain rectangle.
fn rounded_rect(x_len: f64, y_len: f64, r: f64) -> Vec<DVec2> {
	let (hx, hy) = (x_len * 0.5, y_len * 0.5);
	if r <= 0.0 {
		return vec![DVec2::new(hx, hy), DVec2::new(-hx, hy), DVec2::new(-hx, -hy), DVec2::new(hx, -hy)];
	}
	let centres = [
		(DVec2::new(hx - r, hy - r), 0.0),
		(DVec2::new(r - hx, hy - r), 0.5 * PI),
		(DVec2::new(r - hx, r - hy), PI),
		(DVec2::new(hx - r, r - hy), 1.5 * PI),
	];
	let mut pts = Vec::with_capacity(52);
	for (c, a0) in centres {
		for i in 0..=12 {
			let a = a0 + 0.5 * PI * i as f64 / 12.0;
			pts.push(c + DVec2::new(r * a.cos(), r * a.sin()));
		}
	}
	pts
}

/// Centreline length of a racetrack (rounded-rectangle) seal path of overall size
/// `x_len × y_len` with corner radius `corner_r`: `2(x + y) − 8r + 2πr` — the cord
/// cut length for a glued-cord face seal (add ~1–2% compression allowance when
/// cutting, per cord-vendor practice; not included). `None` for degenerate sizes
/// or corners that do not fit (`2·corner_r` exceeding either side).
pub fn racetrack_cord_length(x_len: f64, y_len: f64, corner_r: f64) -> Option<f64> {
	if !(x_len > 0.0 && y_len > 0.0 && corner_r >= 0.0 && x_len.is_finite() && y_len.is_finite())
		|| 2.0 * corner_r > x_len + 1e-9
		|| 2.0 * corner_r > y_len + 1e-9
	{
		return None;
	}
	Some(2.0 * (x_len + y_len) - 8.0 * corner_r + 2.0 * PI * corner_r)
}

/// Cut a **racetrack face-seal (axial) O-ring gland** into a flat face — the lid
/// groove of a rectangular housing: the channel centreline is a rounded rectangle
/// of overall size `x_len × y_len` with corner radius `corner_r` (local x/y axes
/// from the face frame of `axis`), centred at `at` on the face, sunk along
/// `-axis`; depth and width from the metric-cord table for `cord_d`. Cut the cord
/// to [`racetrack_cord_length`]`(x_len, y_len, corner_r)`. Corners are 12-segment
/// arcs; the section is rectangular as in [`o_ring_face_gland`]. `None` for an
/// unstocked cord, a degenerate axis, a corner radius below half the groove width
/// (the inner wall corner would self-intersect), or straights too short
/// (`2·corner_r` exceeding either side).
pub fn o_ring_face_gland_racetrack(
	solid: &Solid,
	at: DVec3,
	axis: DVec3,
	x_len: f64,
	y_len: f64,
	corner_r: f64,
	cord_d: f64,
) -> Option<Solid> {
	let g = metric_cord_gland(cord_d)?;
	let axis = axis.try_normalize()?;
	racetrack_cord_length(x_len, y_len, corner_r)?;
	if corner_r < g.groove_width * 0.5 || x_len - g.groove_width <= 0.0 || y_len - g.groove_width <= 0.0 {
		return None;
	}
	let outer = rounded_rect(x_len + g.groove_width, y_len + g.groove_width, corner_r + g.groove_width * 0.5);
	let inner = rounded_rect(x_len - g.groove_width, y_len - g.groove_width, corner_r - g.groove_width * 0.5);
	Some(face_channel_cut(solid, at, axis, &outer, &inner, g.gland_depth))
}

#[cfg(test)]
mod tests {
	use super::*;
	use kernel_brep::{cylinder, tessellate_default, validate, volume, VertexId};
	use std::f64::consts::PI;

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
	fn the_gland_table_keeps_parkers_static_squeeze_and_fill_bands() {
		// Every dash row must resolve to a gland whose design squeeze (W − L)/W lands
		// in Parker's static 17–30% band and whose fill π(W/2)²/(L·G) stays in the
		// recommended 60–85%; spot-check the -214 and -112 rows against the published
		// numbers (24.99 × 3.53 with L 2.82 / G 4.75; 12.37 × 2.62 with L 2.06 / G 3.56).
		let bad: Vec<u16> = AS568
			.iter()
			.filter(|(dash, _, _)| {
				let s = as568_spec(*dash).expect("every table dash resolves");
				let squeeze = (s.cs - s.gland_depth) / s.cs;
				let fill = PI * (s.cs * 0.5).powi(2) / (s.gland_depth * s.groove_width);
				!(0.17..=0.30).contains(&squeeze) || !(0.60..=0.85).contains(&fill)
			})
			.map(|r| r.0)
			.collect();
		let spot: Vec<(f64, f64, f64, f64)> = [214, 112]
			.iter()
			.map(|&d| {
				let s = as568_spec(d).expect("table row");
				(s.id, s.cs, s.gland_depth, s.groove_width)
			})
			.collect();
		assert!(
			bad.is_empty() && spot == vec![(24.99, 3.53, 2.82, 4.75), (12.37, 2.62, 2.06, 3.56)] && as568_spec(15).is_none(),
			"AS568/Parker gland table: every row in the static squeeze/fill bands (violators: {bad:?}), -214/-112 at the published values (got {spot:?}), -015 (not in the subset) refused"
		);
	}

	#[test]
	fn o_rings_are_exact_nominal_tori() {
		// -214 and -010 free rings: watertight genus-1 tori spanning exactly
		// ID/2 .. ID/2 + W across the vertex radii, with the faceted volume inside
		// (0.97, 1.0) × the Pappus closed form 2π²·R·r² (48 × 24-gon inscribed chords).
		for dash in [214u16, 10] {
			let spec = as568_spec(dash).expect("table row");
			let ring = o_ring(dash).expect("table size");
			let v = validate(&ring);
			let rr = radii(&ring);
			let (r_min, r_max) = rr.iter().fold((f64::INFINITY, 0.0_f64), |(lo, hi), &r| (lo.min(r), hi.max(r)));
			let exact = 2.0 * PI * PI * ((spec.id + spec.cs) * 0.5) * (spec.cs * 0.5).powi(2);
			let vol = volume(&ring).abs();
			assert!(
				v.closed
					&& v.manifold && v.genus == 1
					&& tessellate_default(&ring).is_watertight()
					&& (r_min - spec.id * 0.5).abs() < 1e-9
					&& (r_max - (spec.id * 0.5 + spec.cs)).abs() < 1e-9
					&& vol > 0.97 * exact && vol < exact,
				"AS568-{dash}: want watertight genus-1 torus r {:.3}–{:.3}, vol in (0.97, 1)×{exact:.1}; got {v:?} r=[{r_min:.3},{r_max:.3}] vol={vol:.1}",
				spec.id * 0.5,
				spec.id * 0.5 + spec.cs
			);
		}
		assert!(o_ring(999).is_none(), "dash -999 is not an AS568 size");
	}

	#[test]
	fn groove_cuts_the_parker_gland_into_the_nominal_shaft() {
		// -214 and -012 glands turned into their design shafts (Ø = ID + 2·L): the
		// part stays genus 0 and watertight, the groove root lands exactly on Ø ID,
		// and the lost material is one root-to-OD annulus × G (48-gon walls → exact
		// polygon closed form, 1e-6 relative).
		for (dash, len, z0) in [(214u16, 30.0, 12.0), (12u16, 20.0, 8.0)] {
			let spec = as568_spec(dash).expect("table row");
			let shaft_r = spec.id * 0.5 + spec.gland_depth;
			let shaft = cylinder(DVec3::ZERO, DVec3::Z, shaft_r, len, 48);
			let grooved = o_ring_groove(&shaft, DVec3::new(0.0, 0.0, z0), DVec3::Z, dash).expect("table size");
			let v = validate(&grooved);
			let root_verts = radii(&grooved).iter().filter(|r| (**r - spec.id * 0.5).abs() < 1e-9).count();
			let ring48 = |r: f64| 48.0 * 0.5 * r * r * (2.0 * PI / 48.0).sin(); // 48-gon disc area
			let expected = ring48(shaft_r) * len - (ring48(shaft_r) - ring48(spec.id * 0.5)) * spec.groove_width;
			assert!(
				v.closed
					&& v.manifold && v.genus == 0
					&& tessellate_default(&grooved).is_watertight()
					&& root_verts > 0
					&& (volume(&grooved).abs() - expected).abs() / expected < 1e-6,
				"AS568-{dash} gland in Ø{:.2} shaft: want watertight genus-0 with root vertices at r={:.3}, ~{expected:.1}mm³; got {v:?} roots={root_verts} vol={:.1}",
				2.0 * shaft_r,
				spec.id * 0.5,
				volume(&grooved).abs()
			);
		}
		let shaft = cylinder(DVec3::ZERO, DVec3::Z, 10.0, 30.0, 48);
		assert!(
			o_ring_groove(&shaft, DVec3::new(0.0, 0.0, 10.0), DVec3::ZERO, 214).is_none()
				&& o_ring_groove(&shaft, DVec3::new(0.0, 0.0, 10.0), DVec3::Z, 999).is_none(),
			"a zero axis and an out-of-table dash must be refused"
		);
	}

	#[test]
	fn metric_cord_glands_hold_the_design_squeeze_and_fill_for_every_stocked_size() {
		// Every stocked cord must resolve to a gland at exactly the design point —
		// squeeze (d − L)/d = 25%, fill π(d/2)²/(L·G) = 75%, both mid-band of
		// Parker's static 17–30% / 60–85% recommendations — and the Ø2 row must
		// reproduce the classic printed-housing numbers (1.5 deep × 2.79 wide).
		// Unstocked Ø2.3 and the AS568-only Ø6.99 section are refused.
		let bad: Vec<f64> = METRIC_CORDS
			.iter()
			.filter(|&&c| {
				let g = metric_cord_gland(c).expect("every stocked cord resolves");
				let squeeze = (g.cord_d - g.gland_depth) / g.cord_d;
				let fill = PI * (g.cord_d * 0.5).powi(2) / (g.gland_depth * g.groove_width);
				(squeeze - 0.25).abs() > 1e-12 || (fill - 0.75).abs() > 1e-12
			})
			.copied()
			.collect();
		let g2 = metric_cord_gland(2.0).expect("Ø2 cord");
		assert!(
			bad.is_empty()
				&& (g2.gland_depth - 1.5).abs() < 1e-12
				&& (g2.groove_width - PI * 2.0 / 2.25).abs() < 1e-12
				&& metric_cord_gland(2.3).is_none()
				&& metric_cord_gland(6.99).is_none(),
			"metric cord glands: every row at 25% squeeze / 75% fill (violators: {bad:?}); Ø2 → 1.5 × {:.4} (got {:.4} × {:.4}); Ø2.3 and Ø6.99 refused",
			PI * 2.0 / 2.25,
			g2.gland_depth,
			g2.groove_width
		);
	}

	#[test]
	fn metric_cord_rings_are_exact_nominal_tori_at_any_id() {
		// Ø150 ID × Ø3 cord (a housing-lid perimeter far beyond the AS568 table) and
		// Ø20 × Ø2: watertight genus-1 tori spanning exactly ID/2 .. ID/2 + d, faceted
		// volume inside (0.97, 1.0) × the Pappus closed form — the same bands as the
		// AS568 rings. Unstocked cord and degenerate IDs refused.
		for (id, cs) in [(150.0, 3.0), (20.0, 2.0)] {
			let ring = o_ring_cord(id, cs).expect("stocked cord");
			let v = validate(&ring);
			let rr = radii(&ring);
			let (r_min, r_max) = rr.iter().fold((f64::INFINITY, 0.0_f64), |(lo, hi), &r| (lo.min(r), hi.max(r)));
			let exact = 2.0 * PI * PI * ((id + cs) * 0.5) * (cs * 0.5).powi(2);
			let vol = volume(&ring).abs();
			assert!(
				v.closed
					&& v.manifold && v.genus == 1
					&& tessellate_default(&ring).is_watertight()
					&& (r_min - id * 0.5).abs() < 1e-9
					&& (r_max - (id * 0.5 + cs)).abs() < 1e-9
					&& vol > 0.97 * exact && vol < exact,
				"cord ring Ø{id}×{cs}: want watertight genus-1 torus r {:.1}–{:.1}, vol in (0.97, 1)×{exact:.0}; got {v:?} r=[{r_min:.3},{r_max:.3}] vol={vol:.0}",
				id * 0.5,
				id * 0.5 + cs
			);
		}
		assert!(
			o_ring_cord(20.0, 2.3).is_none() && o_ring_cord(0.0, 2.0).is_none() && o_ring_cord(f64::NAN, 2.0).is_none(),
			"an unstocked Ø2.3 cord, a zero ID and a NaN ID must be refused"
		);
	}

	#[test]
	fn circular_face_glands_sink_the_table_channel_into_a_boss_face() {
		// A Ø50 boss face takes a centreline-Ø36 × Ø2-cord gland; a Ø20 boss a
		// centreline-Ø14 × Ø1.5 one. The part must stay genus 0, watertight on BOTH
		// tessellation routes (the FRICTION #6 export gate), and lose exactly the
		// 48-gon annulus × depth (1e-6 relative — all walls are polygon prisms); the
		// groove floor must sit exactly gland_depth below the face.
		for (boss_d, center_d, cord) in [(50.0, 36.0, 2.0), (20.0, 14.0, 1.5)] {
			let g = metric_cord_gland(cord).expect("stocked cord");
			let h = 10.0;
			let boss = cylinder(DVec3::ZERO, DVec3::Z, boss_d * 0.5, h, 48);
			let cut = o_ring_face_gland(&boss, DVec3::new(0.0, 0.0, h), DVec3::Z, center_d, cord).expect("valid gland");
			let v = validate(&cut);
			let ring48 = |r: f64| 48.0 * 0.5 * r * r * (2.0 * PI / 48.0).sin();
			let expected = ring48(boss_d * 0.5) * h - (ring48(center_d * 0.5 + g.groove_width * 0.5) - ring48(center_d * 0.5 - g.groove_width * 0.5)) * g.gland_depth;
			let floor_z = h - g.gland_depth;
			let floor_verts = (0..cut.vertex_count() as u32).filter(|&i| (cut.position(VertexId(i)).z - floor_z).abs() < 1e-9).count();
			assert!(
				v.closed
					&& v.manifold && v.genus == 0
					&& tessellate_default(&cut).is_watertight()
					&& kernel_brep::tessellate_adaptive_tol(&cut, 0.01).is_watertight()
					&& floor_verts >= 96
					&& (volume(&cut).abs() - expected).abs() / expected < 1e-6,
				"Ø{center_d} face gland (Ø{cord} cord) in Ø{boss_d} boss: want watertight genus-0, ≥96 floor verts at z={floor_z}, {expected:.2}mm³; got {v:?} floors={floor_verts} vol={:.2}",
				volume(&cut).abs()
			);
		}
		let boss = cylinder(DVec3::ZERO, DVec3::Z, 25.0, 10.0, 48);
		assert!(
			o_ring_face_gland(&boss, DVec3::new(0.0, 0.0, 10.0), DVec3::Z, 36.0, 2.3).is_none()
				&& o_ring_face_gland(&boss, DVec3::new(0.0, 0.0, 10.0), DVec3::ZERO, 36.0, 2.0).is_none()
				&& o_ring_face_gland(&boss, DVec3::new(0.0, 0.0, 10.0), DVec3::Z, 2.0, 2.0).is_none(),
			"an unstocked cord, a zero axis and a centreline tighter than the groove width must be refused"
		);
	}

	#[test]
	fn racetrack_face_glands_seal_a_rectangular_lid_with_table_driven_cord() {
		// The FRICTION #10 case: a 120×80 lid takes a 100×60 r8 centreline racetrack
		// for Ø2 cord; a 60×40 lid a 46×26 r6 one for Ø3. The lid stays genus 0,
		// watertight on BOTH routes, loses exactly (outer − inner polygon area) ×
		// depth (shoelace closed form, 1e-6), that polygon area matches the smooth
		// G × centreline-length annulus within 0.5% (12-segment corner faceting), and
		// the cord-length helper returns the exact smooth perimeter.
		let shoelace = |poly: &[DVec2]| {
			0.5 * (0..poly.len())
				.map(|i| {
					let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
					a.x * b.y - b.x * a.y
				})
				.sum::<f64>()
		};
		for (lid_x, lid_y, x, y, r, cord) in [(120.0, 80.0, 100.0, 60.0, 8.0, 2.0), (60.0, 40.0, 46.0, 26.0, 6.0, 3.0)] {
			let g = metric_cord_gland(cord).expect("stocked cord");
			let t = 6.0;
			let lid = kernel_brep::cuboid(DVec3::new(-lid_x * 0.5, -lid_y * 0.5, 0.0), DVec3::new(lid_x * 0.5, lid_y * 0.5, t));
			let cut = o_ring_face_gland_racetrack(&lid, DVec3::new(0.0, 0.0, t), DVec3::Z, x, y, r, cord).expect("valid gland");
			let v = validate(&cut);
			let outer = rounded_rect(x + g.groove_width, y + g.groove_width, r + g.groove_width * 0.5);
			let inner = rounded_rect(x - g.groove_width, y - g.groove_width, r - g.groove_width * 0.5);
			let channel = shoelace(&outer) - shoelace(&inner);
			let cord_len = racetrack_cord_length(x, y, r).expect("valid path");
			let smooth = g.groove_width * cord_len;
			let expected = lid_x * lid_y * t - channel * g.gland_depth;
			assert!(
				v.closed
					&& v.manifold && v.genus == 0
					&& tessellate_default(&cut).is_watertight()
					&& kernel_brep::tessellate_adaptive_tol(&cut, 0.01).is_watertight()
					&& (volume(&cut).abs() - expected).abs() / expected < 1e-6
					&& (channel - smooth).abs() / smooth < 0.005
					&& (cord_len - (2.0 * (x + y) - 8.0 * r + 2.0 * PI * r)).abs() < 1e-12,
				"racetrack {x}×{y} r{r} (Ø{cord} cord) in {lid_x}×{lid_y} lid: want watertight genus-0, vol {expected:.2}, channel within 0.5% of {smooth:.2}, cord {cord_len:.2}; got {v:?} vol={:.2} channel={channel:.2}",
				volume(&cut).abs()
			);
		}
		let lid = kernel_brep::cuboid(DVec3::new(-30.0, -20.0, 0.0), DVec3::new(30.0, 20.0, 6.0));
		assert!(
			o_ring_face_gland_racetrack(&lid, DVec3::new(0.0, 0.0, 6.0), DVec3::Z, 46.0, 26.0, 1.0, 3.0).is_none()
				&& o_ring_face_gland_racetrack(&lid, DVec3::new(0.0, 0.0, 6.0), DVec3::Z, 46.0, 10.0, 6.0, 3.0).is_none()
				&& o_ring_face_gland_racetrack(&lid, DVec3::new(0.0, 0.0, 6.0), DVec3::Z, 46.0, 26.0, 6.0, 2.3).is_none(),
			"a corner radius below half the groove width, corners that do not fit the side, and an unstocked cord must be refused"
		);
	}
}
