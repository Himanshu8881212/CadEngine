// Copyright (c) LMCAD. Licensed under the MIT License.

//! True NURBS–NURBS intersection: the parametric surfaces plug into the same SSI
//! marcher through the closest-point [`NurbsField`] adapter, and the traced curve
//! must lie on both patches and match the geometry we can derive by hand.

use kernel_brep::math::DVec3;
use kernel_brep::ssi::{intersect_surfaces, ImplicitSurface, NurbsField, SsiOptions};
use kernel_brep::NurbsSurface;

/// A degree-1 (bilinear) patch from its four corners — `c{u}{v}`.
fn bilinear(c00: DVec3, c01: DVec3, c10: DVec3, c11: DVec3) -> NurbsSurface {
	NurbsSurface::new(
		1,
		1,
		vec![0.0, 0.0, 1.0, 1.0],
		vec![0.0, 0.0, 1.0, 1.0],
		vec![vec![c00, c01], vec![c10, c11]],
		vec![vec![1.0, 1.0], vec![1.0, 1.0]],
	)
	.expect("valid bilinear patch")
}

#[test]
fn two_flat_nurbs_patches_intersect_in_a_line() {
	// A: the z = 0 patch. B: the z = x patch. Both span x,y ∈ [−2,2].
	let a = bilinear(
		DVec3::new(-2.0, -2.0, 0.0),
		DVec3::new(-2.0, 2.0, 0.0),
		DVec3::new(2.0, -2.0, 0.0),
		DVec3::new(2.0, 2.0, 0.0),
	);
	let b = bilinear(
		DVec3::new(-2.0, -2.0, -2.0),
		DVec3::new(-2.0, 2.0, -2.0),
		DVec3::new(2.0, -2.0, 2.0),
		DVec3::new(2.0, 2.0, 2.0),
	);
	let (fa, fb) = (NurbsField::new(&a, 6), NurbsField::new(&b, 6));
	let opts = SsiOptions { seed_samples: 12, step: 0.05, ..Default::default() };
	let lines = intersect_surfaces(&fa, &fb, DVec3::splat(-2.1), DVec3::splat(2.1), &opts);
	assert!(!lines.is_empty(), "expected an intersection line");
	let pts: Vec<DVec3> = lines.iter().flatten().copied().collect();
	assert!(pts.len() > 40, "line should be traced finely, got {}", pts.len());
	for &p in &pts {
		assert!(fa.value(p).abs() < 1e-5 && fb.value(p).abs() < 1e-5, "off a NURBS patch");
		// Analytic intersection: z = 0 ∧ z = x ⇒ x = z = 0.
		assert!(p.x.abs() < 1e-4 && p.z.abs() < 1e-4, "off the analytic line: {p}");
	}
	let ymin = pts.iter().map(|p| p.y).fold(f64::INFINITY, f64::min);
	let ymax = pts.iter().map(|p| p.y).fold(f64::NEG_INFINITY, f64::max);
	assert!(ymin < -1.0 && ymax > 1.0, "line should span the patch in y ({ymin}..{ymax})");
}

#[test]
fn nurbs_foot_projection_is_scale_invariant() {
	// Regression: the foot-point convergence tolerance must scale with the surface,
	// or a large-coordinate curved patch returns an unconverged foot (a badly wrong
	// signed distance). Saddle z = 4S·u·v at S = 1e8; a point one normal·δ off the
	// centre must read signed distance ≈ δ.
	let s = 1.0e8;
	let saddle = bilinear(
		DVec3::new(-2.0 * s, -2.0 * s, 0.0),
		DVec3::new(-2.0 * s, 2.0 * s, 0.0),
		DVec3::new(2.0 * s, -2.0 * s, 0.0),
		DVec3::new(2.0 * s, 2.0 * s, 4.0 * s),
	);
	let field = NurbsField::new(&saddle, 9);
	let center = saddle.point_at(0.5, 0.5);
	let normal = saddle.normal_at(0.5, 0.5);
	let delta = 1.0e6;
	let d = field.value(center + normal * delta);
	assert!((d - delta).abs() < delta * 1e-3, "scale-variant foot: signed distance {d} vs {delta}");
	assert!(field.value(center).abs() < delta * 1e-3, "on-surface point should read ≈ 0");
}

#[test]
fn saddle_nurbs_meets_a_plane_in_a_curved_intersection() {
	// Saddle (hyperbolic paraboloid) z = (x+2)(y+2)/4, cut by the plane z = 1, gives
	// the curved branch (x+2)(y+2) = 4.
	let saddle = bilinear(
		DVec3::new(-2.0, -2.0, 0.0),
		DVec3::new(-2.0, 2.0, 0.0),
		DVec3::new(2.0, -2.0, 0.0),
		DVec3::new(2.0, 2.0, 4.0),
	);
	let plane = bilinear(
		DVec3::new(-2.0, -2.0, 1.0),
		DVec3::new(-2.0, 2.0, 1.0),
		DVec3::new(2.0, -2.0, 1.0),
		DVec3::new(2.0, 2.0, 1.0),
	);
	let (fs, fp) = (NurbsField::new(&saddle, 7), NurbsField::new(&plane, 5));
	let opts = SsiOptions { seed_samples: 14, step: 0.04, ..Default::default() };
	let lines = intersect_surfaces(&fs, &fp, DVec3::splat(-2.05), DVec3::splat(2.05), &opts);
	assert!(!lines.is_empty(), "expected a curved intersection");
	let pts: Vec<DVec3> = lines.iter().flatten().copied().collect();
	assert!(pts.len() > 30, "curve should be traced, got {}", pts.len());
	let mut curvature_seen = false;
	for &p in &pts {
		assert!(fs.value(p).abs() < 1e-5 && fp.value(p).abs() < 1e-5, "off a NURBS patch");
		assert!((p.z - 1.0).abs() < 1e-4, "not on the z = 1 plane: {p}");
		assert!(((p.x + 2.0) * (p.y + 2.0) - 4.0).abs() < 1e-3, "off (x+2)(y+2)=4: {p}");
		if (p.x - p.y).abs() > 0.5 {
			curvature_seen = true; // a straight line x=y would never get here
		}
	}
	assert!(curvature_seen, "intersection should be genuinely curved, not a line");
}
