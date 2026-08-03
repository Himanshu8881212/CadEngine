// Copyright (c) LMCAD. Licensed under the MIT License.

//! **Measurement queries** — the metrology an operator or orchestrating AI reads
//! off a finished part before committing it to a process: overall dimensions
//! (L×W×H), the space diagonal, point-to-point linear distance, and whether the
//! part fits a given build volume / stock. These wrap the raw
//! [`Solid::aabb`](crate::Solid::aabb) corner pair in the engineering quantities
//! people actually quote, so callers never re-derive size/centre/fit by hand.

use crate::math::DVec3;
use crate::Solid;

/// An axis-aligned bounding box, plus the quantities derived from it. Construct
/// via [`bounding_box`] / [`bounding_box_of`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoundingBox {
	/// Minimum corner (smallest x, y, z).
	pub min: DVec3,
	/// Maximum corner (largest x, y, z).
	pub max: DVec3,
}

impl BoundingBox {
	/// Overall outside dimensions `(X, Y, Z)` — the L×W×H an operator quotes.
	pub fn size(&self) -> DVec3 {
		self.max - self.min
	}

	/// Geometric centre of the box.
	pub fn center(&self) -> DVec3 {
		(self.min + self.max) * 0.5
	}

	/// Space (corner-to-corner) diagonal length — the smallest sphere/tube bore
	/// the part could pass through axis-aligned, and a quick overall-scale number.
	pub fn diagonal(&self) -> f64 {
		(self.max - self.min).length()
	}

	/// Whether the part fits inside `envelope` **in its current orientation**:
	/// every dimension is ≤ the matching envelope dimension. A build-volume /
	/// stock check before sending a part to a printer or mill.
	pub fn fits_within(&self, envelope: DVec3) -> bool {
		let s = self.size();
		s.x <= envelope.x && s.y <= envelope.y && s.z <= envelope.z
	}

	/// Whether the part fits inside `envelope` **allowing 90° axis re-orientation**
	/// (both dimension triples sorted, then compared) — the practical "can I rotate
	/// it onto the plate / into the stock" check. Uses total ordering, so a
	/// non-finite envelope component simply reads as not-fitting rather than panics.
	pub fn fits_within_rotated(&self, envelope: DVec3) -> bool {
		let mut p = self.size().to_array();
		let mut e = envelope.to_array();
		p.sort_by(f64::total_cmp);
		e.sort_by(f64::total_cmp);
		p[0] <= e[0] && p[1] <= e[1] && p[2] <= e[2]
	}
}

/// The axis-aligned [`BoundingBox`] of a solid (overall L×W×H, centre, diagonal,
/// envelope fit). `None` for a solid with no finite vertices to measure.
pub fn bounding_box(s: &Solid) -> Option<BoundingBox> {
	let (min, max) = s.aabb();
	// `aabb` returns an inverted (+∞, −∞) box for a vertex-less solid; reject that
	// and any non-finite extent rather than hand back a nonsensical size.
	if !min.is_finite() || !max.is_finite() || (max - min).min_element() < 0.0 {
		return None;
	}
	Some(BoundingBox { min, max })
}

/// The combined axis-aligned [`BoundingBox`] over several solids — e.g. an
/// assembly, or a multi-body export's overall footprint. `None` if the slice is
/// empty or every body is vertex-less.
pub fn bounding_box_of(solids: &[&Solid]) -> Option<BoundingBox> {
	let mut min = DVec3::splat(f64::INFINITY);
	let mut max = DVec3::splat(f64::NEG_INFINITY);
	let mut any = false;
	for s in solids {
		if let Some(b) = bounding_box(s) {
			min = min.min(b.min);
			max = max.max(b.max);
			any = true;
		}
	}
	any.then_some(BoundingBox { min, max })
}

/// Straight-line (Euclidean) distance between two points — the linear-dimension
/// measure for clearances, hole spacing, stack heights, feature offsets, etc.
pub fn distance(a: DVec3, b: DVec3) -> f64 {
	(a - b).length()
}

#[cfg(test)]
mod tests {
	use super::*;
	use crate::{cuboid, cylinder};

	#[test]
	fn measure_reports_dimensions_center_diagonal_and_envelope_fit() {
		// A 40×20×10 box centred at the origin: size, centre and diagonal are exact,
		// and the rotated-fit check must accept an envelope that only fits after a
		// 90° turn (e.g. a 25×45×15 plate: 40≤45, 20≤25, 10≤15 only when sorted).
		let b = cuboid(DVec3::new(-20.0, -10.0, -5.0), DVec3::new(20.0, 10.0, 5.0));
		let m = bounding_box(&b).expect("box has geometry");
		let diag = (40.0f64.powi(2) + 20.0f64.powi(2) + 10.0f64.powi(2)).sqrt();
		assert!(
			m.size() == DVec3::new(40.0, 20.0, 10.0)
				&& m.center() == DVec3::ZERO
				&& (m.diagonal() - diag).abs() < 1e-9
				&& !m.fits_within(DVec3::new(25.0, 45.0, 15.0))
				&& m.fits_within_rotated(DVec3::new(25.0, 45.0, 15.0))
				&& m.fits_within(DVec3::new(40.0, 20.0, 10.0)),
			"40×20×10 box: size={:?} center={:?} diag={} (want {diag}); as-is fit must fail but rotated fit succeed",
			m.size(),
			m.center(),
			m.diagonal()
		);
	}

	#[test]
	fn combined_bounding_box_and_point_distance() {
		// Two disjoint unit-ish cylinders 100 mm apart on X: the combined box spans
		// both, and the linear distance between their centres is exactly 100.
		let a = cylinder(DVec3::new(0.0, 0.0, 0.0), DVec3::Z, 5.0, 10.0, 32);
		let c = cylinder(DVec3::new(100.0, 0.0, 0.0), DVec3::Z, 5.0, 10.0, 32);
		let combined = bounding_box_of(&[&a, &c]).expect("two bodies");
		assert!(
			(combined.size().x - 110.0).abs() < 1e-9
				&& (distance(DVec3::new(0.0, 0.0, 0.0), DVec3::new(100.0, 0.0, 0.0)) - 100.0).abs() < 1e-12,
			"combined X span want 110 (two Ø10 cylinders 100 apart), got {}; point distance want 100",
			combined.size().x
		);
	}
}
