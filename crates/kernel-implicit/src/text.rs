// Copyright (c) LMCAD. Licensed under the MIT License.

//! Single-stroke **text as an SDF** — emboss or engrave part numbers, labels
//! and logos on any implicit solid. A string becomes a union of 3-D capsule
//! segments (round cross-section) tracing Hershey Simplex strokes in the
//! `z = 0` plane, glyphs advanced along `+X`, baseline on `y = 0`, capital
//! height scaled to the requested `height`.
//!
//! # Font provenance & license (embedded data — do not edit by hand)
//!
//! The stroke table below is decoded from the **Hershey Simplex** font
//! (`futural.jhf`, James Hurt's pen-up/pen-down JHF encoding), fetched from
//! the canonical USENET redistribution mirrored at
//! <https://raw.githubusercontent.com/kamalmostafa/hershey-fonts/master/hershey-fonts/futural.jhf>
//! and verified against the classic Simplex ground truth (glyph 'A' =
//! `/\` + crossbar at width 18, cap height 21, baseline 9 in Hershey's
//! y-down grid). Per that distribution's license, the required
//! acknowledgements: *the Hershey Fonts were originally created by Dr. A. V.
//! Hershey while working at the U. S. National Bureau of Standards; the
//! format of the Font data in this distribution was originally created by
//! James Hurt, Cognition, Inc.* (The data may be used by anyone for any
//! purpose, commercial or otherwise, providing those acknowledgements are
//! distributed with it — this comment is that distribution.)
//!
//! Coverage: `A`–`Z` (lowercase folds to uppercase), `0`–`9`, space, `-`,
//! `.` — the part-marking set. Coordinates are stored y-UP with the baseline
//! at 0 and cap top at 21 grid units (the decoder flipped Hershey's y-down
//! rows via `y′ = 9 − y` and folded each glyph's left bearing into `x`);
//! `Q`'s tail honestly descends to −2.
//!
//! # Field contract
//!
//! [`TextSdf`] is `min` over exact capsule SDFs: **exactly 1-Lipschitz** (a
//! min of 1-Lipschitz fields), equal to the true signed distance everywhere
//! outside the ink, and only *understating* depth inside stroke overlaps —
//! the same contract as the strut lattices, so it is safe for narrow-band
//! pruning and is wrapped as a [`crate::FieldQuality::DistanceBound`] leaf.
//!
//! # Emboss / engrave
//!
//! The stroke tube straddles `z = 0` (half-round proud, half-round sunk), so
//! placing the text field with its plane ON a face gives a half-round bead
//! (union) or groove (difference):
//!
//! ```
//! use kernel_core::math::Affine3A;
//! use kernel_implicit::text::text_field;
//! use kernel_implicit::{Cuboid, Node, Sdf, Vec3};
//!
//! let label = || text_field("LM-10", 8.0, 0.6).transform(
//!     Affine3A::from_translation(Vec3::new(5.0, 4.0, 10.0)), // onto the top face z = 10
//! );
//! let plate = || Node::primitive(Cuboid::from_corners(Vec3::ZERO, Vec3::new(40.0, 16.0, 10.0)));
//! let embossed = plate().union(label());      // raised half-round lettering
//! let engraved = plate().difference(label()); // milled half-round groove
//! assert!(embossed.bounds().max.z > 10.0 && engraved.bounds().max.z >= 10.0);
//! ```

use kernel_core::math::{Aabb, DVec3, Vec3};
use kernel_core::sdf::Sdf;

use crate::ops::Node;
use crate::primitives::Capsule;

/// Hershey Simplex capital height in grid units (baseline 0 → cap top 21);
/// the world scale is `height / CAP_HEIGHT`.
const CAP_HEIGHT: f32 = 21.0;

/// Hershey Simplex glyph: `(advance, strokes)` in y-up grid units (see the
/// module doc for provenance and coordinate conventions). `advance` is the
/// pen advance (Hershey right − left); each stroke is one polyline.
type Glyph = (i8, &'static [&'static [(i8, i8)]]);

/// The embedded glyph table (machine-decoded — see module doc; not hand-typed).
fn glyph(c: char) -> Option<Glyph> {
	Some(match c {
		' ' => (16, &[]),
		'-' => (26, &[&[(4, 9), (22, 9)]]),
		'.' => (8, &[&[(4, 5), (3, 4), (4, 3), (5, 4), (4, 5)]]),
		'0' => (20, &[&[(9, 21), (6, 20), (4, 17), (3, 12), (3, 9), (4, 4), (6, 1), (9, 0), (11, 0), (14, 1), (16, 4), (17, 9), (17, 12), (16, 17), (14, 20), (11, 21), (9, 21)]]),
		'1' => (20, &[&[(6, 17), (8, 18), (11, 21), (11, 0)]]),
		'2' => (20, &[&[(4, 16), (4, 17), (5, 19), (6, 20), (8, 21), (12, 21), (14, 20), (15, 19), (16, 17), (16, 15), (15, 13), (13, 10), (3, 0), (17, 0)]]),
		'3' => (20, &[&[(5, 21), (16, 21), (10, 13), (13, 13), (15, 12), (16, 11), (17, 8), (17, 6), (16, 3), (14, 1), (11, 0), (8, 0), (5, 1), (4, 2), (3, 4)]]),
		'4' => (20, &[&[(13, 21), (3, 7), (18, 7)], &[(13, 21), (13, 0)]]),
		'5' => (20, &[&[(15, 21), (5, 21), (4, 12), (5, 13), (8, 14), (11, 14), (14, 13), (16, 11), (17, 8), (17, 6), (16, 3), (14, 1), (11, 0), (8, 0), (5, 1), (4, 2), (3, 4)]]),
		'6' => (20, &[&[(16, 18), (15, 20), (12, 21), (10, 21), (7, 20), (5, 17), (4, 12), (4, 7), (5, 3), (7, 1), (10, 0), (11, 0), (14, 1), (16, 3), (17, 6), (17, 7), (16, 10), (14, 12), (11, 13), (10, 13), (7, 12), (5, 10), (4, 7)]]),
		'7' => (20, &[&[(17, 21), (7, 0)], &[(3, 21), (17, 21)]]),
		'8' => (20, &[&[(8, 21), (5, 20), (4, 18), (4, 16), (5, 14), (7, 13), (11, 12), (14, 11), (16, 9), (17, 7), (17, 4), (16, 2), (15, 1), (12, 0), (8, 0), (5, 1), (4, 2), (3, 4), (3, 7), (4, 9), (6, 11), (9, 12), (13, 13), (15, 14), (16, 16), (16, 18), (15, 20), (12, 21), (8, 21)]]),
		'9' => (20, &[&[(16, 14), (15, 11), (13, 9), (10, 8), (9, 8), (6, 9), (4, 11), (3, 14), (3, 15), (4, 18), (6, 20), (9, 21), (10, 21), (13, 20), (15, 18), (16, 14), (16, 9), (15, 4), (13, 1), (10, 0), (8, 0), (5, 1), (4, 3)]]),
		'A' => (18, &[&[(9, 21), (1, 0)], &[(9, 21), (17, 0)], &[(4, 7), (14, 7)]]),
		'B' => (21, &[&[(4, 21), (4, 0)], &[(4, 21), (13, 21), (16, 20), (17, 19), (18, 17), (18, 15), (17, 13), (16, 12), (13, 11)], &[(4, 11), (13, 11), (16, 10), (17, 9), (18, 7), (18, 4), (17, 2), (16, 1), (13, 0), (4, 0)]]),
		'C' => (21, &[&[(18, 16), (17, 18), (15, 20), (13, 21), (9, 21), (7, 20), (5, 18), (4, 16), (3, 13), (3, 8), (4, 5), (5, 3), (7, 1), (9, 0), (13, 0), (15, 1), (17, 3), (18, 5)]]),
		'D' => (21, &[&[(4, 21), (4, 0)], &[(4, 21), (11, 21), (14, 20), (16, 18), (17, 16), (18, 13), (18, 8), (17, 5), (16, 3), (14, 1), (11, 0), (4, 0)]]),
		'E' => (19, &[&[(4, 21), (4, 0)], &[(4, 21), (17, 21)], &[(4, 11), (12, 11)], &[(4, 0), (17, 0)]]),
		'F' => (18, &[&[(4, 21), (4, 0)], &[(4, 21), (17, 21)], &[(4, 11), (12, 11)]]),
		'G' => (21, &[&[(18, 16), (17, 18), (15, 20), (13, 21), (9, 21), (7, 20), (5, 18), (4, 16), (3, 13), (3, 8), (4, 5), (5, 3), (7, 1), (9, 0), (13, 0), (15, 1), (17, 3), (18, 5), (18, 8)], &[(13, 8), (18, 8)]]),
		'H' => (22, &[&[(4, 21), (4, 0)], &[(18, 21), (18, 0)], &[(4, 11), (18, 11)]]),
		'I' => (8, &[&[(4, 21), (4, 0)]]),
		'J' => (16, &[&[(12, 21), (12, 5), (11, 2), (10, 1), (8, 0), (6, 0), (4, 1), (3, 2), (2, 5), (2, 7)]]),
		'K' => (21, &[&[(4, 21), (4, 0)], &[(18, 21), (4, 7)], &[(9, 12), (18, 0)]]),
		'L' => (17, &[&[(4, 21), (4, 0)], &[(4, 0), (16, 0)]]),
		'M' => (24, &[&[(4, 21), (4, 0)], &[(4, 21), (12, 0)], &[(20, 21), (12, 0)], &[(20, 21), (20, 0)]]),
		'N' => (22, &[&[(4, 21), (4, 0)], &[(4, 21), (18, 0)], &[(18, 21), (18, 0)]]),
		'O' => (22, &[&[(9, 21), (7, 20), (5, 18), (4, 16), (3, 13), (3, 8), (4, 5), (5, 3), (7, 1), (9, 0), (13, 0), (15, 1), (17, 3), (18, 5), (19, 8), (19, 13), (18, 16), (17, 18), (15, 20), (13, 21), (9, 21)]]),
		'P' => (21, &[&[(4, 21), (4, 0)], &[(4, 21), (13, 21), (16, 20), (17, 19), (18, 17), (18, 14), (17, 12), (16, 11), (13, 10), (4, 10)]]),
		'Q' => (22, &[&[(9, 21), (7, 20), (5, 18), (4, 16), (3, 13), (3, 8), (4, 5), (5, 3), (7, 1), (9, 0), (13, 0), (15, 1), (17, 3), (18, 5), (19, 8), (19, 13), (18, 16), (17, 18), (15, 20), (13, 21), (9, 21)], &[(12, 4), (18, -2)]]),
		'R' => (21, &[&[(4, 21), (4, 0)], &[(4, 21), (13, 21), (16, 20), (17, 19), (18, 17), (18, 15), (17, 13), (16, 12), (13, 11), (4, 11)], &[(11, 11), (18, 0)]]),
		'S' => (20, &[&[(17, 18), (15, 20), (12, 21), (8, 21), (5, 20), (3, 18), (3, 16), (4, 14), (5, 13), (7, 12), (13, 10), (15, 9), (16, 8), (17, 6), (17, 3), (15, 1), (12, 0), (8, 0), (5, 1), (3, 3)]]),
		'T' => (16, &[&[(8, 21), (8, 0)], &[(1, 21), (15, 21)]]),
		'U' => (22, &[&[(4, 21), (4, 6), (5, 3), (7, 1), (10, 0), (12, 0), (15, 1), (17, 3), (18, 6), (18, 21)]]),
		'V' => (18, &[&[(1, 21), (9, 0)], &[(17, 21), (9, 0)]]),
		'W' => (24, &[&[(2, 21), (7, 0)], &[(12, 21), (7, 0)], &[(12, 21), (17, 0)], &[(22, 21), (17, 0)]]),
		'X' => (20, &[&[(3, 21), (17, 0)], &[(17, 21), (3, 0)]]),
		'Y' => (18, &[&[(1, 21), (9, 11), (9, 0)], &[(17, 21), (9, 11)]]),
		'Z' => (20, &[&[(17, 21), (3, 0)], &[(3, 21), (17, 21)], &[(3, 0), (17, 0)]]),
		_ => return None,
	})
}

/// Panic message helper for an unsupported character.
fn expect_glyph(c: char) -> Glyph {
	let up = c.to_ascii_uppercase();
	glyph(up).unwrap_or_else(|| {
		panic!(
			"text_field: unsupported character {c:?} — the embedded Hershey Simplex set covers A-Z (case-folded), 0-9, space, '-' and '.'"
		)
	})
}

/// A text string as a single [`Sdf`]: the union (`min`) of exact capsule
/// distances over every stroke segment. Exactly 1-Lipschitz, exact outside
/// the ink, understates only inside stroke overlaps (see the module docs).
/// Built by [`text_field`]; segment count stays small (tens per glyph), so
/// the plain `min` scan needs no acceleration grid.
pub struct TextSdf {
	segs: Vec<Capsule>,
	bounds: Aabb,
}

impl TextSdf {
	/// Lay out `text` (see [`text_field`] for the geometry contract).
	pub fn new(text: &str, height: f32, stroke_radius: f32) -> Self {
		assert!(height.is_finite() && height > 0.0, "TextSdf: height must be finite and > 0, got {height}");
		assert!(
			stroke_radius.is_finite() && stroke_radius > 0.0,
			"TextSdf: stroke_radius must be finite and > 0, got {stroke_radius}"
		);
		let s = height / CAP_HEIGHT;
		let mut segs = Vec::new();
		let mut pts = Vec::new();
		let mut pen = 0.0_f32;
		for c in text.chars() {
			let (advance, strokes) = expect_glyph(c);
			for stroke in strokes {
				let world = |&(x, y): &(i8, i8)| Vec3::new(pen + x as f32 * s, y as f32 * s, 0.0);
				match stroke {
					[] => {}
					[p] => {
						// A single-vertex stroke is a dot: a degenerate capsule (= sphere).
						let a = world(p);
						segs.push(Capsule::new(a, a, stroke_radius));
						pts.push(a);
					}
					_ => {
						for w in stroke.windows(2) {
							let (a, b) = (world(&w[0]), world(&w[1]));
							segs.push(Capsule::new(a, b, stroke_radius));
							pts.push(a);
							pts.push(b);
						}
					}
				}
			}
			pen += advance as f32 * s;
		}
		assert!(
			!segs.is_empty(),
			"TextSdf: {text:?} produced no strokes — text needs at least one non-space glyph"
		);
		Self { segs, bounds: Aabb::from_points(&pts).pad(stroke_radius) }
	}

	/// Number of capsule segments (one per stroke edge; dots count once).
	pub fn segment_count(&self) -> usize {
		self.segs.len()
	}
}

impl Sdf for TextSdf {
	fn distance(&self, p: Vec3) -> f32 {
		self.segs.iter().fold(f32::INFINITY, |d, c| d.min(c.distance(p)))
	}

	fn distance64(&self, p: DVec3) -> f64 {
		self.segs.iter().fold(f64::INFINITY, |d, c| d.min(c.distance64(p)))
	}

	fn bounds(&self) -> Aabb {
		self.bounds
	}
}

/// Single-stroke text as a composable implicit solid: Hershey Simplex strokes
/// become 3-D capsules of radius `stroke_radius` lying in the `z = 0` plane
/// (tube axis in-plane, round cross-section straddling ±`stroke_radius` in
/// `z`), glyphs advanced along `+X` from the origin, baseline on `y = 0`,
/// capitals scaled to exactly `height`. Lowercase folds to uppercase;
/// supported set: `A`–`Z`, `0`–`9`, space, `-`, `.` (unsupported characters
/// panic loudly — no silent tofu). Newlines are not interpreted: one line.
///
/// Place it with the ordinary transform combinators and combine:
/// `body.union(label)` embosses a half-round bead, `body.difference(label)`
/// engraves a half-round groove (module docs show both). The field is exactly
/// 1-Lipschitz (min of exact capsule SDFs) and is wrapped via
/// [`Node::primitive_bound`], per the crate's min-union field contract.
pub fn text_field(text: &str, height: f32, stroke_radius: f32) -> Node {
	Node::primitive_bound(TextSdf::new(text, height, stroke_radius))
}

/// Total pen-advance width of `text` at capital height `height` — the layout
/// width including each glyph's side bearings (the inked extent is a little
/// narrower; the first/last bearings and the capsule radius trade a few
/// percent either way). Useful for centering: translate by half the advance.
/// Same character support (and panics) as [`text_field`].
pub fn text_advance(text: &str, height: f32) -> f32 {
	let s = height / CAP_HEIGHT;
	text.chars().map(|c| expect_glyph(c).0 as f32 * s).sum()
}
