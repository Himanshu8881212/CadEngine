// Copyright (c) LMCAD. Licensed under the MIT License.

//! Numerical surface–surface intersection, cross-checked against the exact
//! analytic plane sections: a traced polyline must lie on both fields and, where
//! an analytic answer exists, reproduce its circle / ellipse.

use kernel_brep::math::DVec3;
use kernel_brep::ssi::{intersect_surfaces, ImplicitSurface, SsiOptions};
use kernel_brep::{Curve, Surface};

struct SphereField {
	c: DVec3,
	r: f64,
}
impl ImplicitSurface for SphereField {
	fn value(&self, p: DVec3) -> f64 {
		(p - self.c).length() - self.r
	}
	fn gradient(&self, p: DVec3) -> DVec3 {
		(p - self.c).normalize_or_zero()
	}
}

struct PlaneField {
	o: DVec3,
	n: DVec3, // unit
}
impl ImplicitSurface for PlaneField {
	fn value(&self, p: DVec3) -> f64 {
		(p - self.o).dot(self.n)
	}
	fn gradient(&self, _p: DVec3) -> DVec3 {
		self.n
	}
}

struct CylinderField {
	o: DVec3,
	axis: DVec3, // unit
	r: f64,
}
impl CylinderField {
	fn radial(&self, p: DVec3) -> DVec3 {
		let d = p - self.o;
		d - self.axis * d.dot(self.axis)
	}
}
impl ImplicitSurface for CylinderField {
	fn value(&self, p: DVec3) -> f64 {
		self.radial(p).length() - self.r
	}
	fn gradient(&self, p: DVec3) -> DVec3 {
		self.radial(p).normalize_or_zero()
	}
}

/// Both fields vanish on every traced point.
fn assert_on_both<F: ImplicitSurface, G: ImplicitSurface>(line: &[DVec3], f: &F, g: &G, tol: f64) {
	for &p in line {
		assert!(f.value(p).abs() < tol, "off surface f: {}", f.value(p));
		assert!(g.value(p).abs() < tol, "off surface g: {}", g.value(p));
	}
}

#[test]
fn sphere_plane_ssi_traces_the_analytic_circle() {
	let sphere = SphereField { c: DVec3::ZERO, r: 5.0 };
	let plane = PlaneField { o: DVec3::Z * 3.0, n: DVec3::Z };
	let opts = SsiOptions { seed_samples: 20, step: 0.05, ..Default::default() };
	let lines = intersect_surfaces(&sphere, &plane, DVec3::splat(-6.0), DVec3::splat(6.0), &opts);
	assert_eq!(lines.len(), 1, "sphere ∩ plane is a single circle");
	let circle = &lines[0];
	assert!(circle.len() > 100, "circle should be finely traced, got {}", circle.len());
	assert_on_both(circle, &sphere, &plane, 1e-8);
	// Analytic ground truth: radius √(r²−d²) = √(25−9) = 4 at z = 3.
	for &p in circle {
		assert!((p.z - 3.0).abs() < 1e-8);
		assert!(((p - DVec3::Z * 3.0).length() - 4.0).abs() < 1e-6, "off analytic circle");
	}
	// Closed: the two ends are about one marching step apart.
	assert!((circle[0] - circle[circle.len() - 1]).length() < 2.0 * opts.step);
}

#[test]
fn boundary_grazing_curve_is_still_fully_traced() {
	// Regression: a seed that projects just outside the domain (and fails to trace)
	// must not be marked "covered" and suppress genuine interior seeds. The circle
	// radius 4 at z = 3 grazes the box walls at x,y = ±4 — it must still come back
	// as one complete closed loop.
	let sphere = SphereField { c: DVec3::ZERO, r: 5.0 };
	let plane = PlaneField { o: DVec3::Z * 3.0, n: DVec3::Z };
	let opts = SsiOptions { seed_samples: 20, step: 0.05, ..Default::default() };
	let lo = DVec3::new(-4.0, -4.0, -6.0);
	let hi = DVec3::new(4.0, 4.0, 6.0);
	let lines = intersect_surfaces(&sphere, &plane, lo, hi, &opts);
	assert_eq!(lines.len(), 1, "the grazing circle should be one loop");
	let circle = &lines[0];
	assert!(circle.len() > 400, "full circle should be traced, got {}", circle.len());
	assert!((circle[0] - circle[circle.len() - 1]).length() < 2.0 * opts.step, "loop must close");
	for &p in circle {
		assert!(((p - DVec3::Z * 3.0).length() - 4.0).abs() < 1e-6);
	}
}

#[test]
fn sphere_sphere_ssi_is_a_circle_on_the_radical_plane() {
	let a = SphereField { c: DVec3::ZERO, r: 5.0 };
	let b = SphereField { c: DVec3::X * 6.0, r: 5.0 };
	let opts = SsiOptions { seed_samples: 20, step: 0.05, ..Default::default() };
	let lines = intersect_surfaces(&a, &b, DVec3::splat(-6.0), DVec3::splat(6.0), &opts);
	assert_eq!(lines.len(), 1);
	let circle = &lines[0];
	assert_on_both(circle, &a, &b, 1e-8);
	// Radical plane x = 3, radius √(25−9) = 4 about (3,0,0).
	for &p in circle {
		assert!((p.x - 3.0).abs() < 1e-8);
		assert!(((p - DVec3::X * 3.0).length() - 4.0).abs() < 1e-6);
	}
}

/// Largest distance of any point from the plane through three well-spread points
/// (≈ 0 for a planar curve, large for a genuine space curve).
fn nonplanarity(pts: &[DVec3]) -> f64 {
	let p0 = pts[0];
	let p1 = pts.iter().copied().fold(p0, |best, p| if (p - p0).length() > (best - p0).length() { p } else { best });
	let axis = (p1 - p0).normalize_or_zero();
	let dist_to_line = |p: DVec3| (p - p0 - axis * (p - p0).dot(axis)).length();
	let p2 = pts.iter().copied().fold(p0, |best, p| if dist_to_line(p) > dist_to_line(best) { p } else { best });
	let m = (p1 - p0).cross(p2 - p0).normalize_or_zero();
	pts.iter().map(|p| (*p - p0).dot(m).abs()).fold(0.0_f64, f64::max)
}

#[test]
fn analytic_surface_adapter_sphere_plane_ssi_matches_section() {
	// Drive the SSI from the kernel's own analytic Surface type (signed-distance
	// adapter), and confirm it reproduces the exact section circle.
	let sphere = Surface::Sphere { center: DVec3::ZERO, radius: 5.0 };
	let plane = Surface::Plane { origin: DVec3::Z * 3.0, normal: DVec3::Z };
	let opts = SsiOptions { seed_samples: 20, step: 0.05, ..Default::default() };
	let lines = intersect_surfaces(&sphere, &plane, DVec3::splat(-6.0), DVec3::splat(6.0), &opts);
	assert_eq!(lines.len(), 1);
	for &p in &lines[0] {
		assert!(sphere.unsigned_distance(p) < 1e-7 && plane.unsigned_distance(p) < 1e-7);
		assert!((p.z - 3.0).abs() < 1e-7 && ((p - DVec3::Z * 3.0).length() - 4.0).abs() < 1e-6);
	}
}

#[test]
fn two_perpendicular_cylinders_trace_a_nonplanar_space_quartic() {
	// Unequal radii → a genuine 3-D quartic (no elementary closed form), unlike the
	// degenerate planar ellipses of an equal-radius bicylinder. The SSI reaches what
	// the analytic sections cannot.
	let a = Surface::Cylinder { origin: DVec3::ZERO, axis: DVec3::Z, radius: 3.0 };
	let b = Surface::Cylinder { origin: DVec3::ZERO, axis: DVec3::X, radius: 2.0 };
	let opts = SsiOptions { seed_samples: 20, step: 0.05, ..Default::default() };
	let lines = intersect_surfaces(&a, &b, DVec3::splat(-3.5), DVec3::splat(3.5), &opts);
	assert!(!lines.is_empty(), "expected intersection curve(s)");
	let pts: Vec<DVec3> = lines.iter().flatten().copied().collect();
	assert!(pts.len() > 100, "curve should be finely traced, got {}", pts.len());
	for &p in &pts {
		assert!(a.unsigned_distance(p) < 1e-7, "off cylinder A: {}", a.unsigned_distance(p));
		assert!(b.unsigned_distance(p) < 1e-7, "off cylinder B: {}", b.unsigned_distance(p));
	}
	assert!(nonplanarity(&pts) > 0.3, "intersection should be a non-planar space curve");
}

#[test]
fn cylinder_plane_ssi_matches_the_analytic_ellipse() {
	let r = 3.0;
	let axis = DVec3::Z;
	let cyl = CylinderField { o: DVec3::ZERO, axis, r };
	let n = DVec3::new(0.0, 1.0, 1.0).normalize();
	let plane = PlaneField { o: DVec3::ZERO, n };
	let opts = SsiOptions { seed_samples: 20, step: 0.05, ..Default::default() };
	let lines = intersect_surfaces(&cyl, &plane, DVec3::splat(-7.0), DVec3::splat(7.0), &opts);
	assert_eq!(lines.len(), 1, "cylinder ∩ oblique plane is one ellipse");
	let ellipse = &lines[0];
	assert_on_both(ellipse, &cyl, &plane, 1e-8);

	// Analytic ground truth from step 1.
	let surf = Surface::Cylinder { origin: DVec3::ZERO, axis, radius: r };
	let analytic = surf.plane_section(DVec3::ZERO, n);
	let Curve::Ellipse { center, a, b, .. } = analytic[0] else { panic!("expected analytic Ellipse") };
	// The traced points' extreme radii from the analytic centre recover its semi-axes.
	let (mut max_d, mut min_d) = (0.0_f64, f64::INFINITY);
	for &p in ellipse {
		let d = (p - center).length();
		max_d = max_d.max(d);
		min_d = min_d.min(d);
	}
	assert!((max_d - a).abs() < 0.01, "semi-major {max_d} vs analytic {a}");
	assert!((min_d - b).abs() < 0.01, "semi-minor {min_d} vs analytic {b}");
}
