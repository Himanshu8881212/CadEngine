// Copyright (c) LMCAD. Licensed under the MIT License.

//! Interference metrics and boolean-hazard pre-scans.
//!
//! Two lessons from production use of the planar-arrangement booleans live here
//! as AI-callable guardrails:
//!
//! - [`overlap_volume`] — measure interference by a **direct intersection**,
//!   never by the `vol(A) − vol(A ∖ B)` subtraction (which fabricated phantom
//!   overlaps on complex solids — BUG CLASS A).
//! - [`detect_coincident_fit`] — pre-scan two solids for near-coincident face
//!   pairs (press fits, flush contacts) that a boolean should not be asked to
//!   resolve at all (a Ø2-pin-in-Ø1.95-pocket boolean ground for 53 CPU-minutes
//!   — BUG CLASS B).

use kernel_core::math::DVec3;

use crate::checked::try_intersection;
use crate::geom::Surface;
use crate::topo::{Solid, VertexId};
use crate::validate::exact_volume;

/// Angular tolerance (rad) for "same direction" in [`detect_coincident_fit`].
const COINCIDENT_ANG_TOL: f64 = 1e-3;
/// Linear tolerance (mm) for "same offset / radius" in [`detect_coincident_fit`].
const COINCIDENT_DIST_TOL: f64 = 0.05;

/// The canonical interference metric: the volume (mm³) of `A ∩ B`, measured by
/// a **direct boolean intersection** — `Some(0.0)` immediately when the solids'
/// (curvature-padded) bounding boxes are disjoint, `None` only when the
/// intersection boolean itself fails to produce a valid solid.
///
/// ## Why not `vol(A) − vol(A ∖ B)`?
/// The subtraction metric routes the interference question through a
/// `difference` on the FULL complexity of `A`, so every face of `A` — not just
/// the contact region — must survive the arrangement round-trip, and any
/// re-stitching residue reads as overlap. In production this FABRICATED phantom
/// overlaps of **0.27–6.4 mm³** when `A` was a complex solid (an octagonal
/// housing already carrying 27 small cylindrical pockets and a register recess
/// from prior booleans), and in the reverse direction UNDERREPORTED a real
/// **~90 mm³** intersection. The direct `intersection(A, B)` discards all of
/// `A`'s non-contact geometry by construction and proved reliable, so it is the
/// only metric this function computes. (The shape class is pinned in
/// `tests/volume_conservation.rs`; on the current kernel both metrics agree to
/// ~1e-6 mm³ there, but the direct metric is structurally immune.)
///
/// The volume is [`exact_volume`] (analytic bulge corrections), so a curved
/// overlap — a pin dipping into a bore — reads at its true value, not the
/// faceted underestimate. The prefilter AABBs come from the solids' vertex
/// data, padded by an upper bound on each curved face's chord sagitta, so a
/// true surface bulging outward past its facet chords can never be declared
/// disjoint by mistake.
pub fn overlap_volume(a: &Solid, b: &Solid) -> Option<f64> {
	match (conservative_aabb(a), conservative_aabb(b)) {
		(Some((alo, ahi)), Some((blo, bhi))) => {
			let disjoint = ahi.x < blo.x
				|| bhi.x < alo.x
				|| ahi.y < blo.y
				|| bhi.y < alo.y
				|| ahi.z < blo.z
				|| bhi.z < alo.z;
			if disjoint {
				return Some(0.0);
			}
		}
		// An empty operand cannot overlap anything.
		_ => return Some(0.0),
	}
	let common = try_intersection(a, b).ok()?;
	Some(exact_volume(&common))
}

/// Pre-scan two solids for **near-coincident face pairs** — the fit geometry
/// (press pins, flush registers) on which a boolean is the wrong tool. Returns
/// `true` when any face of `a` and face of `b` lie on nearly the same analytic
/// surface (within [`COINCIDENT_ANG_TOL`] = 1e-3 rad angular and
/// [`COINCIDENT_DIST_TOL`] = 0.05 mm linear) AND their face extents actually
/// come near each other (face AABBs within the linear tolerance), so two
/// far-apart faces that merely share a supporting plane do not trigger.
///
/// - **Planar** pairs match on normal direction (sign-insensitive — a flush
///   CONTACT has anti-parallel normals) and plane offset.
/// - **Cylindrical** pairs match on axis direction, axis-line separation and
///   radius — this is the press-fit class: a Ø2 pin booleaned against a Ø1.95
///   press pocket (nearly identical axis + radius) ground for **53 CPU-minutes
///   without finishing**, because the arrangement must resolve two long
///   near-parallel curved walls a few hundredths of a millimetre apart into
///   thousands of sliver fragments.
/// - Sphere / cone / torus pairs match on their analytic parameters to the same
///   tolerances (same hazard class, scanned for completeness).
///
/// Callers that get `true` should route the pair through a dedicated fit gate —
/// measure the fit numerically (radius difference, [`overlap_volume`] of an
/// intentionally shrunk tool, clearance checks) — instead of running a boolean
/// across the coincident pair. This is a cheap advisory scan (O(faces of A ×
/// faces of B) parameter comparisons, no arrangement); it never mutates and
/// never false-negatives within its tolerances, but a `true` does not prove the
/// boolean WILL hang — it flags that the operands are in the hazard class.
pub fn detect_coincident_fit(a: &Solid, b: &Solid) -> bool {
	// Solid-level reject: solids further apart than the tolerance share nothing.
	match (conservative_aabb(a), conservative_aabb(b)) {
		(Some((alo, ahi)), Some((blo, bhi))) => {
			let t = COINCIDENT_DIST_TOL;
			if ahi.x + t < blo.x
				|| bhi.x + t < alo.x
				|| ahi.y + t < blo.y
				|| bhi.y + t < alo.y
				|| ahi.z + t < blo.z
				|| bhi.z + t < alo.z
			{
				return false;
			}
		}
		_ => return false,
	}
	let fa = face_scan_data(a);
	let fb = face_scan_data(b);
	for (sa, alo, ahi) in &fa {
		for (sb, blo, bhi) in &fb {
			if !boxes_near(*alo, *ahi, *blo, *bhi, COINCIDENT_DIST_TOL) {
				continue;
			}
			if surfaces_coincident(sa, sb) {
				return true;
			}
		}
	}
	false
}

/// Per-face scan record: analytic surface + outer-loop AABB.
fn face_scan_data(s: &Solid) -> Vec<(Surface, DVec3, DVec3)> {
	s.faces()
		.map(|f| {
			let mut lo = DVec3::splat(f64::INFINITY);
			let mut hi = DVec3::splat(f64::NEG_INFINITY);
			for p in s.face_polygon(f) {
				lo = lo.min(p);
				hi = hi.max(p);
			}
			(s.face(f).surface, lo, hi)
		})
		.collect()
}

/// Whether two AABBs come within `tol` of each other on every axis.
fn boxes_near(alo: DVec3, ahi: DVec3, blo: DVec3, bhi: DVec3, tol: f64) -> bool {
	!(ahi.x + tol < blo.x
		|| bhi.x + tol < alo.x
		|| ahi.y + tol < blo.y
		|| bhi.y + tol < alo.y
		|| ahi.z + tol < blo.z
		|| bhi.z + tol < alo.z)
}

/// Sign-insensitive angle between two directions (rad).
fn direction_angle(u: DVec3, v: DVec3) -> f64 {
	let u = u.normalize_or_zero();
	let v = v.normalize_or_zero();
	u.cross(v).length().atan2(u.dot(v).abs())
}

/// Near-coincidence of two analytic surfaces within the fit tolerances.
fn surfaces_coincident(a: &Surface, b: &Surface) -> bool {
	let ang = COINCIDENT_ANG_TOL;
	let dist = COINCIDENT_DIST_TOL;
	match (*a, *b) {
		(Surface::Plane { origin: oa, normal: na }, Surface::Plane { origin: ob, normal: nb }) => {
			direction_angle(na, nb) < ang && (oa - ob).dot(na.normalize_or_zero()).abs() < dist
		}
		(
			Surface::Cylinder { origin: oa, axis: aa, radius: ra },
			Surface::Cylinder { origin: ob, axis: ab, radius: rb },
		) => {
			let axis = aa.normalize_or_zero();
			let d = ob - oa;
			direction_angle(aa, ab) < ang
				&& (ra - rb).abs() < dist
				&& (d - axis * d.dot(axis)).length() < dist
		}
		(Surface::Sphere { center: ca, radius: ra }, Surface::Sphere { center: cb, radius: rb }) => {
			(ca - cb).length() < dist && (ra - rb).abs() < dist
		}
		(
			Surface::Cone { apex: pa, axis: aa, half_angle: ha },
			Surface::Cone { apex: pb, axis: ab, half_angle: hb },
		) => direction_angle(aa, ab) < ang && (ha - hb).abs() < ang && (pa - pb).length() < dist,
		(
			Surface::Torus { center: ca, axis: aa, major: ma, minor: na },
			Surface::Torus { center: cb, axis: ab, major: mb, minor: nb },
		) => {
			direction_angle(aa, ab) < ang
				&& (ca - cb).length() < dist
				&& (ma - mb).abs() < dist
				&& (na - nb).abs() < dist
		}
		_ => false,
	}
}

/// Vertex AABB of a solid, padded by an upper bound on the chord sagitta of its
/// curved faces (a true cylinder/sphere/cone/torus surface bulges OUTWARD past
/// the facet chords, so the raw vertex box can under-cover it by up to
/// `r − √(r² − (c/2)²)` for the longest facet chord `c` on curvature radius
/// `r`). `None` for an empty solid. The padding makes the disjointness verdict
/// in [`overlap_volume`] conservative: it can only fall through to the real
/// boolean, never wrongly report `0.0`.
fn conservative_aabb(s: &Solid) -> Option<(DVec3, DVec3)> {
	if s.vertex_count() == 0 || s.face_count() == 0 {
		return None;
	}
	let mut lo = DVec3::splat(f64::INFINITY);
	let mut hi = DVec3::splat(f64::NEG_INFINITY);
	for i in 0..s.vertex_count() as u32 {
		let p = s.position(VertexId(i));
		lo = lo.min(p);
		hi = hi.max(p);
	}
	let mut margin = 0.0f64;
	for f in s.faces() {
		let poly = s.face_polygon(f);
		// Smallest curvature radius the face can present to a chord.
		let r = match s.face(f).surface {
			Surface::Plane { .. } => continue,
			Surface::Cylinder { radius, .. } | Surface::Sphere { radius, .. } => radius,
			Surface::Torus { minor, .. } => minor,
			// A cone's circumferential curvature radius at radial distance ρ is
			// ρ/cos α ≥ ρ; the smaller ρ bound only enlarges the sagitta (safe).
			Surface::Cone { apex, axis, .. } => {
				let axis = axis.normalize_or_zero();
				poly.iter()
					.map(|&p| {
						let d = p - apex;
						(d - axis * d.dot(axis)).length()
					})
					.fold(f64::INFINITY, f64::min)
			}
		};
		let mut half_chord = 0.0f64;
		for i in 0..poly.len() {
			half_chord = half_chord.max(poly[i].distance(poly[(i + 1) % poly.len()]) * 0.5);
		}
		// Exact circular-segment sagitta, capped at r (a facet chord of a valid
		// tessellation spans at most half a revolution).
		let sag = if half_chord >= r { r } else { r - (r * r - half_chord * half_chord).sqrt() };
		margin = margin.max(sag);
	}
	let pad = DVec3::splat(margin + 1e-9);
	Some((lo - pad, hi + pad))
}

/// [`overlap_volume`] for many independent pose/pair checks at once, evaluated
/// on scoped threads and returned IN INPUT ORDER (results are independent, so
/// parallelism cannot change them — this is the safe, coarse-grained
/// parallelism; the arrangement inside each boolean stays single-threaded to
/// protect run-to-run bit-determinism). Campaign pose matrices (insertion /
/// twist sweeps) are the intended caller: RESPOOL's gate wall went from
/// serial-sum to slowest-single-boolean wall-clock.
pub fn overlap_volume_many(pairs: &[(&crate::topo::Solid, crate::topo::Solid)]) -> Vec<Option<f64>> {
	kernel_core::par::par_map_indexed(pairs, |_, (a, b)| overlap_volume(a, b))
}

#[cfg(test)]
mod tests {
	use super::*;
	use crate::build::{cuboid, cylinder};
	use crate::checked::try_difference;
	use crate::validate::volume;
	use kernel_core::math::DVec3;

	/// A 10×10×6 block with a vertical through-pocket of radius `pocket_r` at the
	/// origin, plus a pin of radius `pin_r` on the SAME axis protruding 1 mm past
	/// both faces — the press-fit geometry of BUG CLASS B.
	fn pin_in_pocket(pocket_r: f64, pin_r: f64) -> (Solid, Solid) {
		let block = cuboid(DVec3::new(-5.0, -5.0, 0.0), DVec3::new(5.0, 5.0, 6.0));
		let pocket = cylinder(DVec3::new(0.0, 0.0, -1.0), DVec3::Z, pocket_r, 8.0, 32);
		let housing = try_difference(&block, &pocket).expect("pocketed block must validate");
		let pin = cylinder(DVec3::new(0.0, 0.0, -1.0), DVec3::Z, pin_r, 8.0, 32);
		(housing, pin)
	}

	#[test]
	fn coincident_fit_flags_the_press_pin_and_clears_separated_and_loose_pairs() {
		// Ø2 pin in a Ø1.95 press pocket (radii 1.0 vs 0.975: Δr = 0.025 < 0.05,
		// identical axis) — the exact pair that ground a boolean for 53 CPU-minutes.
		let (housing, press_pin) = pin_in_pocket(0.975, 1.0);
		let press = detect_coincident_fit(&housing, &press_pin);
		// Two clearly separated cubes: coplanar TOP faces exist (z = 10 both), but
		// their extents never come near each other — must NOT trigger.
		let cube_a = cuboid(DVec3::ZERO, DVec3::splat(10.0));
		let cube_b = cuboid(DVec3::new(30.0, 0.0, 0.0), DVec3::new(40.0, 10.0, 10.0));
		let separated = detect_coincident_fit(&cube_a, &cube_b);
		// A MODERATELY-close pair — same axis but radius differing by 0.5 (a loose
		// Ø2.95 bore around the Ø2 pin) — is an ordinary boolean, not a fit hazard.
		let (loose_housing, pin) = pin_in_pocket(1.475, 1.0);
		let loose = detect_coincident_fit(&loose_housing, &pin);
		assert!(
			press && !separated && !loose,
			"coincidence guard misfired: press-fit pin-in-pocket={press} (want true), \
			 separated cubes={separated} (want false), radius-gap-0.5 pair={loose} (want false)"
		);
	}

	#[test]
	fn overlap_volume_short_circuits_disjoint_boxes_and_measures_a_real_dip() {
		// Disjoint boxes: no boolean runs, exact zero.
		let cube = cuboid(DVec3::ZERO, DVec3::splat(10.0));
		let far = cylinder(DVec3::new(50.0, 0.0, 0.0), DVec3::Z, 3.0, 10.0, 16);
		let disjoint = overlap_volume(&cube, &far);
		// A pin dipping 2 mm into the cube: true overlap = π·1.5²·2 (analytic,
		// exact_volume's bulge corrections recover it from the 32-gon tool).
		let pin = cylinder(DVec3::new(5.0, 5.0, 8.0), DVec3::Z, 1.5, 10.0, 32);
		let dip = overlap_volume(&cube, &pin).expect("cube∩pin is a plain valid boolean");
		let want = std::f64::consts::PI * 1.5 * 1.5 * 2.0;
		// Empty operand: overlaps nothing.
		let empty = overlap_volume(&cube, &Solid::default());
		assert!(
			disjoint == Some(0.0) && (dip - want).abs() < 1e-6 && empty == Some(0.0),
			"overlap_volume wrong: disjoint={disjoint:?} (want Some(0.0)), dip={dip} (want {want}), \
			 empty={empty:?} (want Some(0.0))"
		);
		// Cross-check: the tessellated intersection volume must be the FACETED disc
		// (32-gon sin-deficit, ~0.64% below analytic) — overlap_volume's exact_volume
		// recovers precisely that deficit via the cylinder bulge correction.
		let vi = volume(&crate::booleans::intersection(&cube, &pin));
		let want_faceted = 16.0 * 1.5 * 1.5 * (std::f64::consts::PI / 16.0).sin() * 2.0;
		assert!(
			(vi - want_faceted).abs() < 1e-6,
			"tessellated intersection volume off its faceted closed form: {vi} vs {want_faceted}"
		);
	}
}
