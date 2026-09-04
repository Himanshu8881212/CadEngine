// Copyright (c) LMCAD. Licensed under the MIT License.

//! Threadless fastener bodies: hex bolts, hex nuts, plain washers and socket-head cap screws,
//! both fully parametric and pre-sized from the published standard tables (ISO 4017 hex-head
//! screws, ISO 4032 hex nuts, ISO 7089 plain washers, DIN 912 / ISO 4762 socket-head cap screws).
//! Threads are not modelled here — these are the analytically exact bodies an assembly needs;
//! for modelled ISO threads see [`super::threads`].

use super::{circle48, hexagon_across_flats};
use kernel_brep::math::{DAffine3, DVec2, DVec3};
use kernel_brep::{cylinder, difference, extrude, extrude_with_holes, revolve, union, Solid};
use std::f64::consts::PI;

/// One row of the ISO 4017 (DIN-EN 24017, ≈ DIN 933) hex-head screw table:
/// `(thread Ø d, across-flats s, head height k)`. Source: ISO 4017 dimension table as published
/// at fasteners.eu/standards/iso/4017 (nominal values, mm).
const ISO4017: [(f64, f64, f64); 8] = [
	(3.0, 5.5, 2.0),
	(4.0, 7.0, 2.8),
	(5.0, 8.0, 3.5),
	(6.0, 10.0, 4.0),
	(8.0, 13.0, 5.3),
	(10.0, 16.0, 6.4),
	(12.0, 18.0, 7.5),
	(16.0, 24.0, 10.0),
];

/// One row of the ISO 4032 (≈ DIN 934, current widths) hex-nut table:
/// `(thread Ø d, across-flats s, thickness m)`. Source: ISO 4032 dimension table as published at
/// fasteners.eu/standards/iso/4032 (max values, mm). Note ISO widths for M10/M12 are 16/18 mm
/// (the superseded DIN 934 used 17/19).
const ISO4032: [(f64, f64, f64); 8] = [
	(3.0, 5.5, 2.4),
	(4.0, 7.0, 3.2),
	(5.0, 8.0, 4.7),
	(6.0, 10.0, 5.2),
	(8.0, 13.0, 6.8),
	(10.0, 16.0, 8.4),
	(12.0, 18.0, 10.8),
	(16.0, 24.0, 14.8),
];

/// One row of the ISO 7089 (≈ DIN 125 A, 200 HV) plain-washer table:
/// `(thread Ø d, inner Ø d1, outer Ø d2, thickness h)`. Source: ISO 7089 dimension table as
/// published at fasteners.eu/standards/iso/7089 (nominal values, mm).
const ISO7089: [(f64, f64, f64, f64); 8] = [
	(3.0, 3.2, 7.0, 0.5),
	(4.0, 4.3, 9.0, 0.8),
	(5.0, 5.3, 10.0, 1.0),
	(6.0, 6.4, 12.0, 1.6),
	(8.0, 8.4, 16.0, 1.6),
	(10.0, 10.5, 20.0, 2.0),
	(12.0, 13.0, 24.0, 2.5),
	(16.0, 17.0, 30.0, 3.0),
];

/// One row of the DIN 912 / ISO 4762 socket-head cap-screw table:
/// `(thread Ø d, head Ø dk, head height k, hex socket across-flats s, socket depth t)`.
/// Source: DIN 912 / ISO 4762 dimension table as published at fasteners.eu/standards/din/912
/// (dk/k max, s nominal, t min; mm). The classic proportions dk = 1.5·d and k = d hold exactly
/// for every row, and t ≈ k/2.
const DIN912: [(f64, f64, f64, f64, f64); 8] = [
	(3.0, 5.5, 3.0, 2.5, 1.3),
	(4.0, 7.0, 4.0, 3.0, 2.0),
	(5.0, 8.5, 5.0, 4.0, 2.5),
	(6.0, 10.0, 6.0, 5.0, 3.0),
	(8.0, 13.0, 8.0, 6.0, 4.0),
	(10.0, 16.0, 10.0, 8.0, 5.0),
	(12.0, 18.0, 12.0, 10.0, 6.0),
	(16.0, 24.0, 16.0, 14.0, 8.0),
];

/// Match a nominal metric size against a table keyed by its first column.
fn lookup<const N: usize, T: Copy>(table: &[T; N], m: f64, key: impl Fn(&T) -> f64) -> Option<T> {
	table.iter().find(|row| (key(row) - m).abs() < 1e-9).copied()
}

/// ISO 4017 hex-head dimensions for a nominal thread size `m` (3, 4, 5, 6, 8, 10, 12, 16):
/// `(across-flats s, head height k)` in mm. `None` for sizes outside the table.
pub fn iso4017_head(m: f64) -> Option<(f64, f64)> {
	lookup(&ISO4017, m, |r| r.0).map(|(_, s, k)| (s, k))
}

/// DIN 912 / ISO 4762 socket-head cap-screw dimensions for a nominal thread size `m`
/// (3, 4, 5, 6, 8, 10, 12, 16): `(head Ø dk, head height k, socket across-flats s, socket
/// depth t)` in mm. `None` for sizes outside the table.
pub fn din912_dims(m: f64) -> Option<(f64, f64, f64, f64)> {
	lookup(&DIN912, m, |r| r.0).map(|(_, dk, k, s, t)| (dk, k, s, t))
}

/// A standard **hex nut**: a hexagonal prism of wrench size `width` (across flats) and `height`,
/// bored concentrically by a clearance hole of `bore` **diameter**. A closed, manifold, genus-1
/// exact B-rep that tessellates watertight via the analytic path.
pub fn hex_nut(width: f64, height: f64, bore: f64) -> Solid {
	let body = extrude(&hexagon_across_flats(width), height);
	// The hole runs past both faces so the cut is clean through the prism.
	let hole = cylinder(DVec3::new(0.0, 0.0, -1.0), DVec3::Z, bore * 0.5, height + 2.0, 48);
	difference(&body, &hole)
}

/// An **ISO 4032 hex nut** (≈ DIN 934) sized from the standard table for the nominal thread Ø
/// `m`: across-flats and thickness from the table, bored at the nominal thread diameter (the
/// thread itself is not modelled — see [`super::threads`] for modelled threads). `None` for
/// sizes outside M3–M16.
pub fn hex_nut_iso4032(m: f64) -> Option<Solid> {
	lookup(&ISO4032, m, |r| r.0).map(|(d, s, thickness)| hex_nut(s, thickness, d))
}

/// A flat round **washer**: a disk of `outer` diameter and `thickness`, bored concentrically by an
/// `inner` diameter. Built by revolving its rectangular cross-section (no boolean), so it is an
/// exact, genus-1 annular ring that tessellates watertight by construction.
pub fn washer(outer: f64, inner: f64, thickness: f64) -> Solid {
	let (ro, ri) = (outer * 0.5, inner * 0.5);
	// Cross-section in the (radius, axial) half-plane: a rectangle from inner to outer radius and
	// 0 to `thickness`, revolved about the axis into the ring.
	revolve(&[DVec2::new(ri, 0.0), DVec2::new(ro, 0.0), DVec2::new(ro, thickness), DVec2::new(ri, thickness)], 64)
}

/// An **ISO 7089 plain washer** (≈ DIN 125 A) sized from the standard table for the nominal
/// thread Ø `m`: inner Ø, outer Ø and thickness from the table. `None` outside M3–M16.
pub fn washer_iso7089(m: f64) -> Option<Solid> {
	lookup(&ISO7089, m, |r| r.0).map(|(_, d1, d2, h)| washer(d2, d1, h))
}

/// A standard **hex-head bolt** body: a cylindrical shank of `shank` **diameter** and
/// `shank_len` length, capped end-to-end by a hexagonal head of wrench size `head_width`
/// (across flats) and `head_height`, fused with a boolean union. A closed, manifold,
/// genus-0 exact B-rep whose volume is the head plus the shank. Threads are modelled as a
/// cosmetic refinement (the demo winds a helical ridge in the voxel half — the exact B-rep
/// thread union self-intersects), so this body is the analytically exact part an assembly
/// needs. For a watertight printable mesh of the curved/planar head-to-shank junction, heal
/// via [`crate::watertight_mesh`].
pub fn hex_bolt(head_width: f64, head_height: f64, shank: f64, shank_len: f64) -> Solid {
	let shaft = cylinder(DVec3::ZERO, DVec3::Z, shank * 0.5, shank_len, 48);
	// The head sits on top of the shank, its base coincident with (and wider than) the
	// shank's top face — a fully-contained coplanar contact that fuses into one solid.
	let head =
		extrude(&hexagon_across_flats(head_width), head_height).transformed(DAffine3::from_translation(DVec3::new(0.0, 0.0, shank_len)));
	union(&shaft, &head)
}

/// An **ISO 4017 hex bolt** body (≈ DIN 933, fully-threaded series) sized from the standard
/// table for the nominal thread Ø `m` and shank `length`: across-flats and head height from
/// the table, shank at the nominal diameter. The thread is not modelled (this is the exact
/// assembly body); for a body+thread pair see [`super::threads::threaded_hex_bolt`]. `None`
/// outside M3–M16.
pub fn hex_bolt_iso4017(m: f64, length: f64) -> Option<Solid> {
	iso4017_head(m).map(|(s, k)| hex_bolt(s, k, m, length))
}

/// A **DIN 912 / ISO 4762 socket-head cap screw** body for nominal thread Ø `m` (M3–M16) and
/// shank `length` (under-head to tip): cylindrical head of Ø dk = 1.5·d and height k = d per
/// the table, with the hexagonal drive socket (across-flats s, depth t ≈ k/2) cut into the top
/// face as a real hexagonal pocket — so the drive geometry is present for clearance checks and
/// rendering. The shank is the nominal Ø `m` cylinder; the thread is not modelled (see
/// [`super::threads`] for modelled ISO threads). Returns a closed, manifold, genus-0 exact
/// B-rep; `None` for sizes outside the table.
pub fn socket_head_cap_screw(m: f64, length: f64) -> Option<Solid> {
	let (dk, k, s, t) = din912_dims(m)?;
	// Machined like the real part is, with no coplanar boolean anywhere (a head primitive
	// seated face-on-face on a coaxial shank fuses seed-dependently — same-phase 48-gon rims
	// are the boolean's known degeneracy): start from a head-Ø blank over the FULL length and
	// TURN the shank down to Ø m with one ring cutter whose hole wall becomes the shank
	// surface. The cutter's outer boundary is a square (half-side dk/2 + 2, radially clear of
	// the blank), its caps at z = −1 (in air) and z = length (a transverse cut through the
	// blank's wall mid-height), so every contact is generic.
	let blank = extrude(&circle48(dk * 0.5), length + k);
	let half = dk * 0.5 + 2.0;
	let square = vec![DVec2::new(half, half), DVec2::new(-half, half), DVec2::new(-half, -half), DVec2::new(half, -half)];
	let ring =
		extrude_with_holes(&square, &[circle48(m * 0.5)], length + 1.0).transformed(DAffine3::from_translation(DVec3::new(0.0, 0.0, -1.0)));
	let turned = difference(&blank, &ring);
	// Hex drive socket: a pocket sunk `t` into the head's top face. The cutting prism extends
	// 1 mm above the face so the cut is clean; the pocket floor stays inside the head (t < k).
	let socket = extrude(&hexagon_across_flats(s), t + 1.0).transformed(DAffine3::from_translation(DVec3::new(0.0, 0.0, length + k - t)));
	Some(difference(&turned, &socket))
}

/// One row of the DIN 127 B **spring (split) lock washer** table: `(thread Ø m,
/// inner Ø d1, radial section width b, section thickness s)`. Source: DIN 127 B
/// dimension table as published at fasteners.eu/standards/din/127 (d1 min, b × s
/// section; mm). The standard's d2 column is a max envelope (d1 + 2b plus the
/// spring opening), not a turned diameter.
const DIN127B: [(f64, f64, f64, f64); 7] = [
	(3.0, 3.1, 1.3, 0.8),
	(4.0, 4.1, 1.5, 0.9),
	(5.0, 5.1, 1.8, 1.2),
	(6.0, 6.1, 2.5, 1.6),
	(8.0, 8.1, 3.0, 2.0),
	(10.0, 10.2, 3.5, 2.2),
	(12.0, 12.2, 4.0, 2.5),
];

/// DIN 127 B dimensions for nominal size `m` (M3–M12): `(inner Ø d1, section
/// width b, section thickness s)`; `None` outside the table.
pub fn din127_dims(m: f64) -> Option<(f64, f64, f64)> {
	DIN127B.iter().find(|r| (r.0 - m).abs() < 1e-9).map(|&(_, d1, b, s)| (d1, b, s))
}

/// Angular width of a spring washer's split gap, and the axial rise of the helix
/// across the sweep (one section thickness → free height 2·s, the catalog's
/// typical uncompressed height). DIN 127 specifies the section and diameters but
/// leaves the working opening free — these two are documented display/function
/// conventions, not standard dimensions.
const SPRING_WASHER_GAP_DEG: f64 = 15.0;

/// A **DIN 127 B spring (split) lock washer** for nominal `m` (M3–M12): the b × s
/// rectangular section swept one turn (minus the 15° split gap) around the d1
/// bore, rising one thickness across the sweep so the free height is 2·s and the
/// split ends stand open — the spring geometry that bites when compressed. The
/// section stays radial/axial (a punched-and-twisted strip, like the real part),
/// built as a 64-station loft: closed, manifold, genus 0, watertight on both
/// routes. Sharp split edges (DIN 127 B's unchamfered form); the standard's d2
/// max envelope is respected by construction (d1 + 2b < d2). `None` outside the
/// table.
pub fn spring_washer(m: f64) -> Option<Solid> {
	let (d1, b, s) = din127_dims(m)?;
	let (r0, r1) = (d1 * 0.5, d1 * 0.5 + b);
	let sweep = 2.0 * PI - SPRING_WASHER_GAP_DEG.to_radians();
	let rise = s;
	let n = 64;
	let sections: Vec<Vec<DVec3>> = (0..=n)
		.map(|i| {
			let t = i as f64 / n as f64;
			let a = t * sweep;
			let (ca, sa) = (a.cos(), a.sin());
			let z = t * rise;
			// Radial/axial rectangle, wound CCW as seen along the direction of travel.
			[(r0, z), (r0, z + s), (r1, z + s), (r1, z)].iter().map(|&(r, z)| DVec3::new(r * ca, r * sa, z)).collect()
		})
		.collect();
	kernel_brep::loft_solid(&sections)
}

#[cfg(test)]
mod tests {
	use super::*;
	use crate::parts::hexagon_area;
	use kernel_brep::{tessellate_default, validate, volume};
	use std::f64::consts::PI;

	#[test]
	fn hex_nut_is_a_valid_genus_one_part_of_the_right_volume() {
		// A wrench-size-16 nut, 8 mm tall, Ø10 bore. Hexagon area for across-flats W is √3/2·W²·…
		// → (3√3/2)·R² with R=(W/2)/cos30°; volume = that·height − π·(bore/2)²·height.
		let (w, h, bore) = (16.0, 8.0, 10.0);
		let nut = hex_nut(w, h, bore);
		let v = validate(&nut);
		let r = (w * 0.5) / (PI / 6.0).cos();
		let hex_area = 1.5 * 3.0_f64.sqrt() * r * r;
		let expected = hex_area * h - PI * (bore * 0.5) * (bore * 0.5) * h;
		assert!(
			v.closed
				&& v.manifold
				&& v.genus == 1
				&& tessellate_default(&nut).is_watertight()
				&& (volume(&nut).abs() - expected).abs() / expected < 0.01,
			"hex_nut must be a watertight genus-1 part ~{expected:.0}mm³: {v:?} wt={} vol={:.0}",
			tessellate_default(&nut).is_watertight(),
			volume(&nut).abs()
		);
	}

	#[test]
	fn washer_is_a_valid_genus_one_ring_of_the_right_volume() {
		// Ø20 outer, Ø10 inner, 3 mm thick → an annular disk, volume π(10²−5²)·3.
		let (outer, inner, t) = (20.0, 10.0, 3.0);
		let washer = washer(outer, inner, t);
		let v = validate(&washer);
		let expected = PI * ((outer * 0.5).powi(2) - (inner * 0.5).powi(2)) * t;
		assert!(
			v.closed
				&& v.manifold
				&& v.genus == 1
				&& tessellate_default(&washer).is_watertight()
				&& (volume(&washer).abs() - expected).abs() / expected < 0.01,
			"washer must be a watertight genus-1 ring ~{expected:.0}mm³: {v:?} wt={} vol={:.0}",
			tessellate_default(&washer).is_watertight(),
			volume(&washer).abs()
		);
	}

	#[test]
	fn hex_bolt_is_a_valid_genus_zero_body_of_the_right_volume() {
		// A wrench-16 head 10 mm tall on a Ø8 shank 30 mm long. Body volume = hex-head prism
		// (area·height) + cylindrical shank (π·r²·length); the two stack with no overlap.
		let (hw, hh, sd, sl) = (16.0, 10.0, 8.0, 30.0);
		let bolt = hex_bolt(hw, hh, sd, sl);
		let v = validate(&bolt);
		let r = (hw * 0.5) / (PI / 6.0).cos();
		let head_area = 1.5 * 3.0_f64.sqrt() * r * r;
		let expected = head_area * hh + PI * (sd * 0.5) * (sd * 0.5) * sl;
		assert!(
			v.closed && v.manifold && v.genus == 0 && (volume(&bolt).abs() - expected).abs() / expected < 0.01,
			"hex_bolt must be a closed genus-0 body ~{expected:.0}mm³: {v:?} vol={:.0}",
			volume(&bolt).abs()
		);
	}

	#[test]
	fn socket_head_cap_screws_match_the_din912_table_and_lose_exactly_the_socket_pocket() {
		// Two sizes spot-checked against the DIN 912 table (dk = 1.5·d, k = d, s, t): the body
		// must be a closed genus-0 solid whose volume is shank + head − the hex socket pocket.
		// The faceted shank/head (48-gon) sit slightly inside the true cylinders, hence 1%.
		for (m, len, dk, k, s, t) in [(6.0, 20.0, 10.0, 6.0, 5.0, 3.0), (10.0, 30.0, 16.0, 10.0, 8.0, 5.0)] {
			assert_eq!(din912_dims(m), Some((dk, k, s, t)), "DIN 912 table row for M{m}");
			let screw = socket_head_cap_screw(m, len).expect("table size");
			let v = validate(&screw);
			let expected = PI * (m * 0.5).powi(2) * len + PI * (dk * 0.5).powi(2) * k - hexagon_area(s) * t;
			assert!(
				v.closed
					&& v.manifold && v.genus == 0
					&& tessellate_default(&screw).is_watertight()
					&& (volume(&screw).abs() - expected).abs() / expected < 0.01,
				"M{m}×{len} SHCS must be a watertight genus-0 body ~{expected:.0}mm³: {v:?} vol={:.0}",
				volume(&screw).abs()
			);
		}
	}

	#[test]
	fn spring_washers_are_open_one_turn_helices_of_the_din127_section() {
		// M5 and M8: genus-0 watertight×2 (the split makes it a ball, not a ring),
		// bore wall exactly at d1/2, OD exactly d1/2 + b (inside the standard's d2
		// max), free height exactly 2·s (the documented rise convention), and volume
		// = Pappus over the polyline sweep (b·s section × centreline path) within 1%.
		use kernel_brep::{tessellate_adaptive_tol, VertexId};
		for m in [5.0, 8.0] {
			let (d1, b, s) = din127_dims(m).expect("table row");
			let w = spring_washer(m).expect("table size");
			let v = validate(&w);
			let wt = tessellate_default(&w).is_watertight() && tessellate_adaptive_tol(&w, 0.01).is_watertight();
			let (r0, r1) = (d1 * 0.5, d1 * 0.5 + b);
			let (mut rmin, mut rmax, mut zmax) = (f64::INFINITY, 0.0_f64, 0.0_f64);
			for i in 0..w.vertex_count() as u32 {
				let p = w.position(VertexId(i));
				let r = (p.x * p.x + p.y * p.y).sqrt();
				rmin = rmin.min(r);
				rmax = rmax.max(r);
				zmax = zmax.max(p.z);
			}
			let sweep = 2.0 * PI - 15.0_f64.to_radians();
			let rm = (r0 + r1) * 0.5;
			let (n, rise) = (64.0, s);
			let chord = (2.0 * rm * (sweep / (2.0 * n)).sin()).hypot(rise / n);
			let expected = b * s * chord * n;
			let vol = volume(&w).abs();
			assert!(
				v.closed && v.manifold
					&& v.genus == 0 && wt && (rmin - r0).abs() < 1e-9
					&& (rmax - r1).abs() < 1e-9 && (zmax - 2.0 * s).abs() < 1e-9
					&& (vol - expected).abs() / expected < 0.01,
				"DIN 127 B M{m}: want a watertight×2 genus-0 split ring Ø{} … Ø{}, 2s = {} free, ~{expected:.2}mm³; got {v:?} wt={wt} r={rmin}…{rmax} z={zmax} vol={vol:.2}",
				d1,
				d1 + 2.0 * b,
				2.0 * s
			);
		}
		assert!(spring_washer(16.0).is_none(), "the DIN 127 B table here stops at M12");
	}

	#[test]
	fn table_sized_nut_washer_and_bolt_constructors_follow_their_standards() {
		// ISO 4032 M10 nut: AF 16 × 8.4 thick, Ø10 bore. ISO 7089 M10 washer: Ø20 × Ø10.5 × 2.
		// ISO 4017 M10 bolt: AF 16 head, 6.4 tall. One snapshot assertion over all three.
		let nut = hex_nut_iso4032(10.0).expect("M10 nut");
		let wash = washer_iso7089(10.0).expect("M10 washer");
		let bolt = hex_bolt_iso4017(10.0, 30.0).expect("M10 bolt");
		let nut_expected = hexagon_area(16.0) * 8.4 - PI * 25.0 * 8.4;
		let wash_expected = PI * (10.0 * 10.0 - 5.25 * 5.25) * 2.0;
		let bolt_expected = hexagon_area(16.0) * 6.4 + PI * 25.0 * 30.0;
		let measured = [volume(&nut).abs(), volume(&wash).abs(), volume(&bolt).abs()];
		let expected = [nut_expected, wash_expected, bolt_expected];
		assert!(
			measured.iter().zip(&expected).all(|(got, want)| (got - want).abs() / want < 0.01)
				&& [&nut, &wash, &bolt].iter().all(|s| {
					let v = validate(s);
					v.closed && v.manifold
				}),
			"table-sized M10 nut/washer/bolt volumes {measured:?} must each be within 1% of {expected:?} and all valid"
		);
	}
}
