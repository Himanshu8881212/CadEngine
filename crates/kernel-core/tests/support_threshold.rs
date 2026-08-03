// Copyright (c) LMCAD. Licensed under the MIT License.

//! The 45° knife-edge pin (2026-07-30).
//!
//! `support_free_report` / `overhang_analysis` used to convert the overhang
//! angle to radians and take its sine in **f32** before widening to f64, while
//! facet normals are computed in f64. At 45° the threshold came out
//! −0.7071067690849304 against a true-45° facet normal of −0.70710678118654746
//! — so a surface built EXACTLY on the audit's own threshold was reported as
//! needing support. Measured on the shipped `teardrop_hole` (roof at exactly
//! 45°): 8 mm² of steep area at `overhang_deg = 45`, 0 at 46.
//!
//! This pins the fixed behaviour on a synthetic roof whose angle is exact by
//! construction, so the test says what it means without depending on any
//! part builder:
//!
//! * a facet built AT the threshold is NOT steep (the bug),
//! * a facet 1° past it IS steep (the gate still bites — this is what makes
//!   the fix a correction rather than a loosening),
//! * and the two reports agree, since a part that passes one audit and fails
//!   the other is its own kind of lie.

use kernel_core::math::Vec3;
use kernel_core::mesh::Mesh;

/// One downward-facing triangle whose normal sits exactly `deg` from vertical,
/// built from the angle itself so the geometry carries no rounding of its own.
/// Winding is chosen so the normal points DOWN (−Z component), i.e. it is a
/// ceiling facet the printer would have to bridge or support.
fn roof_facet(deg: f64) -> Mesh {
	// A triangle in the x–z plane extruded along y: its normal is
	// (sin θ, 0, −cos θ) for the winding below, i.e. θ from straight-down.
	// `support_overhang_deg` is measured from VERTICAL, and the classifier
	// tests n·up < −sin(deg) — so a facet at `deg` from vertical has
	// n·up = −sin(deg) exactly.
	let (s, c) = deg.to_radians().sin_cos();
	let n_down = Vec3::new(c as f32, 0.0, -(s as f32));
	// Build a triangle whose plane has that normal: two in-plane directions.
	let u = Vec3::new(-(s as f32), 0.0, -(c as f32)); // ⟂ n, in x–z
	let w = Vec3::Y;
	let p0 = Vec3::ZERO;
	let p1 = u * 10.0;
	let p2 = w * 10.0;
	// Orient so the computed normal matches n_down.
	let (e1, e2) = (p1 - p0, p2 - p0);
	let indices = if e1.cross(e2).dot(n_down) > 0.0 { vec![0, 1, 2] } else { vec![0, 2, 1] };
	Mesh { positions: vec![p0, p1, p2], indices, ..Default::default() }
}

#[test]
fn threshold_facets_are_not_steep_but_one_degree_past_is() {
	// Exactly ON the 45° threshold — must NOT be reported as steep.
	let at = roof_facet(45.0);
	let at_report = at.support_free_report(Vec3::Z, 45.0, 0.3);
	let at_overhang = at.overhang_analysis(Vec3::Z, 45.0);

	// One degree PAST the threshold — must still be caught.
	let past = roof_facet(46.0);
	let past_report = past.support_free_report(Vec3::Z, 45.0, 0.3);
	let past_overhang = past.overhang_analysis(Vec3::Z, 45.0);

	// And a non-round threshold, to prove this is not a 45-specific hack.
	let at_30 = roof_facet(30.0);
	let at_30_report = at_30.support_free_report(Vec3::Z, 30.0, 0.3);

	assert!(
		at_report.steep_area < 1e-6
			&& at_overhang.overhang_area < 1e-6
			&& past_report.steep_area > 1.0
			&& past_overhang.overhang_area > 1.0
			&& at_30_report.steep_area < 1e-6,
		"45° knife-edge regressed — a facet built ON the audit threshold must pass and one past it must fail. \
		 at-45 steep_area={} (want <1e-6) · at-45 overhang_area={} (want <1e-6) · \
		 past-46 steep_area={} (want >1) · past-46 overhang_area={} (want >1) · \
		 at-30 steep_area={} (want <1e-6). If the first two are non-zero the threshold is being \
		 evaluated in f32 again (see the note in mesh/mod.rs).",
		at_report.steep_area,
		at_overhang.overhang_area,
		past_report.steep_area,
		past_overhang.overhang_area,
		at_30_report.steep_area,
	);
}
