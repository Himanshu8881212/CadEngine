// Copyright (c) LMCAD. Licensed under the MIT License.

//! Plain shafts with DIN 6885 keyways, and the matching parallel keys. The DIN 6885-1 size
//! table (key cross-section and keyway depths keyed off the shaft diameter) lives here and is
//! shared by the gear/pulley hub keyways.

use kernel_brep::math::{DAffine3, DVec2, DVec3};
use kernel_brep::{cylinder, difference, extrude, Solid};
use std::f64::consts::PI;

/// One DIN 6885-1 parallel-key size: key cross-section `b × h` (width × height) and the keyway
/// depths `t1` (in the shaft) and `t2` (in the hub), all in mm. Depths are measured on the part
/// centreline: shaft keyway floor at `d/2 − t1`, hub keyway ceiling at `d/2 + t2`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct KeySize {
	/// Key width (the keyway slot width).
	pub b: f64,
	/// Key height.
	pub h: f64,
	/// Keyway depth in the **shaft** (slot floor at `d/2 − t1`).
	pub t1: f64,
	/// Keyway depth in the **hub** (slot ceiling at `d/2 + t2`).
	pub t2: f64,
}

/// The DIN 6885-1 size table: `(shaft Ø over, shaft Ø up-to-and-including, b, h, t1, t2)`.
/// Source: DIN 6885-1 dimension table as published at fullerfasteners.com/tech/
/// din-6885-specifications-parallel-keys-deep-pattern (nominal t1/t2 minima, mm).
const DIN6885: [(f64, f64, f64, f64, f64, f64); 12] = [
	(6.0, 8.0, 2.0, 2.0, 1.2, 1.0),
	(8.0, 10.0, 3.0, 3.0, 1.8, 1.4),
	(10.0, 12.0, 4.0, 4.0, 2.5, 1.8),
	(12.0, 17.0, 5.0, 5.0, 3.0, 2.3),
	(17.0, 22.0, 6.0, 6.0, 3.5, 2.8),
	(22.0, 30.0, 8.0, 7.0, 4.0, 3.3),
	(30.0, 38.0, 10.0, 8.0, 5.0, 3.3),
	(38.0, 44.0, 12.0, 8.0, 5.0, 3.3),
	(44.0, 50.0, 14.0, 9.0, 5.5, 3.8),
	(50.0, 58.0, 16.0, 10.0, 6.0, 4.3),
	(58.0, 65.0, 18.0, 11.0, 7.0, 4.4),
	(65.0, 75.0, 20.0, 12.0, 7.5, 4.9),
];

/// The DIN 6885-1 parallel-key size for a shaft (or hub bore) of diameter `d` mm — the standard
/// auto-selection "key for Ø`d` shaft". `None` outside the table's 6–75 mm range. Range bounds
/// follow the standard's "over a, up to and including b" convention.
pub fn din6885_key_size(d: f64) -> Option<KeySize> {
	DIN6885
		.iter()
		.find(|&&(over, upto, ..)| d > over && d <= upto)
		.map(|&(_, _, b, h, t1, t2)| KeySize { b, h, t1, t2 })
}

/// A keyway slot specification for [`shaft`]: where the DIN 6885 form-A slot sits along the
/// axis. The cross-section (width `b`, depth `t1`) comes from the [`KeySize`], normally
/// auto-selected with [`din6885_key_size`].
#[derive(Clone, Copy, Debug)]
pub struct ShaftKeyway {
	/// Key size (b/h/t1/t2); pass `din6885_key_size(d)` for the standard size.
	pub size: KeySize,
	/// Overall slot length along the axis, including the two semicircular ends.
	pub length: f64,
	/// Distance from the shaft's z = 0 end face to the start of the slot. Keep
	/// `0 < offset` and `offset + length < shaft length` so the slot stays a pocket in the
	/// lateral wall (an end-breaking slot is cut as given, but is a different feature).
	pub offset: f64,
}

/// A closed **stadium** (rounded-ended rectangle) profile: overall length `l` along +X from 0,
/// width `b` centred on y = 0, with semicircular ends of radius b/2 (DIN 6885 form A — the
/// plan shape an end mill leaves). Wound counter-clockwise; each semicircle is sampled with 16
/// segments. (Shared with the linear-motion flange plates.)
pub(crate) fn stadium(l: f64, b: f64) -> Vec<DVec2> {
	let r = b * 0.5;
	let (c0, c1) = (r, l - r); // semicircle centres on y = 0
	let mut pts = Vec::with_capacity(36);
	pts.push(DVec2::new(c0, -r));
	pts.push(DVec2::new(c1, -r));
	// Right cap: −90° → +90° about (c1, 0).
	for i in 1..16 {
		let a = -PI * 0.5 + PI * i as f64 / 16.0;
		pts.push(DVec2::new(c1 + r * a.cos(), r * a.sin()));
	}
	pts.push(DVec2::new(c1, r));
	pts.push(DVec2::new(c0, r));
	// Left cap: +90° → +270° about (c0, 0).
	for i in 1..16 {
		let a = PI * 0.5 + PI * i as f64 / 16.0;
		pts.push(DVec2::new(c0 + r * a.cos(), r * a.sin()));
	}
	pts
}

/// Area of the [`stadium`] **polygon** (the faceted semicircles, not the smooth stadium):
/// rectangle `(l − b)·b` plus a 32-gon disc of radius b/2 (test helper for exact volumes).
#[cfg(test)]
fn stadium_polygon_area(l: f64, b: f64) -> f64 {
	let r = b * 0.5;
	(l - b) * b + 32.0 * 0.5 * r * r * (2.0 * PI / 32.0).sin()
}

/// A **DIN 6885 parallel key** (form A, round-ended): cross-section `b × h`, overall length `l`
/// (over the semicircular ends). The standard machine key that drops into the slot cut by
/// [`shaft`] with a matching [`ShaftKeyway`]. Edge chamfers of the real key are not modelled.
/// Use [`din6885_key_size`] to pick `b × h` for a shaft diameter. Genus-0 exact B-rep,
/// watertight by construction.
pub fn parallel_key(b: f64, h: f64, l: f64) -> Solid {
	extrude(&stadium(l, b), h)
}

/// A plain **shaft**: a Ø`d` × `length` cylinder along +Z (base at the origin), optionally with
/// a DIN 6885 **form-A keyway slot** (rounded ends, width `b`, floor at `d/2 − t1`) milled into
/// its +X side. The slot is cut by subtracting a stadium-profile prism plunged radially — the
/// cut-across-a-curved-wall boolean (bug R3 territory, robust since 2026-06-09). With
/// `keyway: None` this is just the exact cylinder (48 segments).
pub fn shaft(d: f64, length: f64, keyway: Option<ShaftKeyway>) -> Solid {
	let body = cylinder(DVec3::ZERO, DVec3::Z, d * 0.5, length, 48);
	let Some(kw) = keyway else {
		return body;
	};
	// Cutter: the stadium profile is drawn in a local XY plane (x = along the shaft axis,
	// y = across the slot width) and extruded along local +Z by t1 + 1 mm; the affine below maps
	// local (x, y, z) → world (d/2 + 1 − z, y, x + offset), so the prism plunges radially from
	// 1 mm outside the surface down to the slot floor at d/2 − t1.
	let cutter = extrude(&stadium(kw.length, kw.size.b), kw.size.t1 + 1.0).transformed(DAffine3::from_cols(
		DVec3::new(0.0, 0.0, 1.0),
		DVec3::new(0.0, 1.0, 0.0),
		DVec3::new(-1.0, 0.0, 0.0),
		DVec3::new(d * 0.5 + 1.0, 0.0, kw.offset),
	));
	difference(&body, &cutter)
}

#[cfg(test)]
mod tests {
	use super::*;
	use kernel_brep::{tessellate_default, validate, volume, VertexId};

	#[test]
	fn din6885_table_selects_the_standard_key_for_a_shaft_diameter() {
		// Spot rows of the DIN 6885-1 table incl. both ends of a range, plus out-of-range.
		let probe = |d: f64| din6885_key_size(d).map(|k| (k.b, k.h, k.t1, k.t2));
		assert_eq!(
			[probe(10.0), probe(10.5), probe(20.0), probe(38.0), probe(5.0), probe(80.0)],
			[
				Some((3.0, 3.0, 1.8, 1.4)),   // 8 < d ≤ 10 → 3×3
				Some((4.0, 4.0, 2.5, 1.8)),   // 10 < d ≤ 12 → 4×4
				Some((6.0, 6.0, 3.5, 2.8)),   // 17 < d ≤ 22 → 6×6
				Some((10.0, 8.0, 5.0, 3.3)),  // upper bound inclusive → 10×8
				None,                         // below the table
				None,                         // above the table
			],
			"DIN 6885-1 size selection"
		);
	}

	#[test]
	fn keyed_shaft_cuts_the_table_slot_into_the_wall() {
		// Ø20 × 60 shaft with the standard 6×6 keyway (t1 = 3.5), slot 25 long starting at z=10.
		// The slot floor must be the plane x = 10 − 3.5 = 6.5 spanning exactly the key width
		// (|y| ≤ 3), and the removed material is bounded by slot-area × the min/max pocket depth
		// (depth varies 3.02…3.5 across the slot because the cylinder surface curves away).
		let (d, len, t1, b) = (20.0, 60.0, 3.5, 6.0);
		let kw = ShaftKeyway { size: din6885_key_size(d).expect("Ø20 in table"), length: 25.0, offset: 10.0 };
		let s = shaft(d, len, Some(kw));
		let v = validate(&s);
		let floor: Vec<DVec3> = (0..s.vertex_count() as u32)
			.map(|i| s.position(VertexId(i)))
			.filter(|p| (p.x - (d * 0.5 - t1)).abs() < 1e-9)
			.collect();
		let width = floor.iter().map(|p| p.y.abs()).fold(0.0, f64::max);
		let cyl = 48.0 * 0.5 * (2.0 * PI / 48.0).sin() * 100.0 * len; // faceted 48-gon prism
		let cut = stadium_polygon_area(kw.length, b);
		let vol = volume(&s).abs();
		assert!(
			v.closed
				&& v.manifold && v.genus == 0
				&& tessellate_default(&s).is_watertight()
				&& !floor.is_empty() && (width - b * 0.5).abs() < 1e-9
				&& vol > cyl - cut * t1
				&& vol < cyl - cut * 3.0,
			"keyed Ø{d} shaft: want genus-0 watertight, slot floor at x={} spanning ±{}, volume in ({:.0}, {:.0}); got {v:?} floor_pts={} width={width} vol={vol:.0}",
			d * 0.5 - t1,
			b * 0.5,
			cyl - cut * t1,
			cyl - cut * 3.0,
			floor.len()
		);
	}

	#[test]
	fn parallel_key_is_the_exact_stadium_prism() {
		// DIN 6885 A 6×6×25 key (the mate of the Ø20 shaft slot above): volume must equal the
		// stadium polygon area × height to machine precision, and the body is genus 0.
		let (b, h, l) = (6.0, 6.0, 25.0);
		let k = parallel_key(b, h, l);
		let v = validate(&k);
		let expected = stadium_polygon_area(l, b) * h;
		// 1e-6 relative: volume() integrates the divergence over ~70 faces in f64, which
		// accumulates ~1e-8 relative rounding — far below any geometric approximation.
		assert!(
			v.closed && v.manifold && v.genus == 0 && tessellate_default(&k).is_watertight() && (volume(&k).abs() - expected).abs() < 1e-6 * expected,
			"6×6×25 key must be a watertight genus-0 prism of exactly {expected:.6}mm³: {v:?} vol={:.6}",
			volume(&k).abs()
		);
	}
}
