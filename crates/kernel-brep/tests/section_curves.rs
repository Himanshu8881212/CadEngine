// Copyright (c) LMCAD. Licensed under the MIT License.

//! Exact analytic plane sections: `Surface::plane_section` must return conic
//! curves that lie *exactly* on both the surface and the cutting plane, with the
//! conic type and dimensions matching the closed-form geometry.

use std::f64::consts::{PI, TAU};

use kernel_brep::math::DVec3;
use kernel_brep::{Curve, Surface};

/// Sample a curve densely and assert every point lies on `surface` (unsigned
/// distance ≈ 0) and on the plane through `o` with unit normal `n`.
fn assert_on_surface_and_plane(curve: &Curve, surface: &Surface, o: DVec3, n: DVec3, span: f64, tol: f64) {
	for k in 0..64 {
		let t = -span + (2.0 * span) * (k as f64) / 63.0;
		let p = curve.point_at(t);
		assert!(p.is_finite(), "non-finite curve point at t={t}");
		let ds = surface.unsigned_distance(p);
		let dp = (p - o).dot(n).abs();
		assert!(ds < tol, "point off surface: |d|={ds} at t={t}");
		assert!(dp < tol, "point off plane: |d|={dp} at t={t}");
	}
}

#[test]
fn conic_arc_length_is_exact() {
	// Exact measure of curved edges: line/circle are closed-form; the parabola arc
	// integrates to its closed form ∫₀⁴√(1+(t/2)²)dt = 2√5 + asinh(2) via quadrature.
	let line = Curve::Line { origin: DVec3::ZERO, dir: DVec3::new(3.0, 4.0, 0.0) }; // |dir| = 5
	assert!((line.length(0.0, 2.0) - 10.0).abs() < 1e-12, "line length {}", line.length(0.0, 2.0));

	let circle = Curve::Circle { center: DVec3::new(1.0, 2.0, 3.0), normal: DVec3::Z, radius: 5.0 };
	assert!((circle.length(0.0, TAU) - TAU * 5.0).abs() < 1e-12, "circle perimeter {}", circle.length(0.0, TAU));

	// Ellipse degenerating to a circle (a = b = r) has perimeter 2πr.
	let ell_circle = Curve::Ellipse { center: DVec3::ZERO, normal: DVec3::Z, u: DVec3::X, a: 4.0, b: 4.0 };
	assert!((ell_circle.length(0.0, TAU) - TAU * 4.0).abs() < 1e-9, "degenerate ellipse perimeter {}", ell_circle.length(0.0, TAU));

	let para = Curve::Parabola { vertex: DVec3::ZERO, axis: DVec3::Y, dir: DVec3::X, focal: 1.0 };
	let exact = 2.0 * 5.0_f64.sqrt() + 2.0_f64.asinh();
	assert!((para.length(0.0, 4.0) - exact).abs() < 1e-9, "parabola arc {} vs {exact}", para.length(0.0, 4.0));
}

#[test]
fn plane_meets_plane_in_a_line() {
	let xy = Surface::Plane { origin: DVec3::ZERO, normal: DVec3::Z };
	let curves = xy.plane_section(DVec3::ZERO, DVec3::Y); // xz-plane → the x-axis
	assert_eq!(curves.len(), 1);
	let Curve::Line { dir, .. } = curves[0] else { panic!("expected a Line") };
	assert!(dir.normalize().dot(DVec3::X).abs() > 1.0 - 1e-12, "line should run along x");
	assert_on_surface_and_plane(&curves[0], &xy, DVec3::ZERO, DVec3::Y, 10.0, 1e-12);
}

#[test]
fn parallel_planes_do_not_intersect() {
	let xy = Surface::Plane { origin: DVec3::ZERO, normal: DVec3::Z };
	assert!(xy.plane_section(DVec3::Z * 3.0, DVec3::Z).is_empty());
}

#[test]
fn sphere_section_is_a_circle_of_closed_form_radius() {
	let r = 5.0;
	let d = 3.0; // plane offset from centre along z
	let sphere = Surface::Sphere { center: DVec3::ZERO, radius: r };
	let curves = sphere.plane_section(DVec3::Z * d, DVec3::Z);
	assert_eq!(curves.len(), 1);
	let Curve::Circle { center, radius, .. } = curves[0] else { panic!("expected a Circle") };
	assert!((radius - (r * r - d * d).sqrt()).abs() < 1e-12, "radius {radius} != sqrt(r^2-d^2)");
	assert!((center - DVec3::Z * d).length() < 1e-12);
	assert_on_surface_and_plane(&curves[0], &sphere, DVec3::Z * d, DVec3::Z, PI, 1e-12);
}

#[test]
fn sphere_tangent_plane_is_a_point_beyond_is_empty() {
	let sphere = Surface::Sphere { center: DVec3::ZERO, radius: 4.0 };
	let tangent = sphere.plane_section(DVec3::Z * 4.0, DVec3::Z);
	assert_eq!(tangent.len(), 1);
	let Curve::Circle { radius, .. } = tangent[0] else { panic!("expected a Circle") };
	assert!(radius < 1e-6, "tangent section should be a zero-radius circle, got {radius}");
	assert!(sphere.plane_section(DVec3::Z * 4.5, DVec3::Z).is_empty());
}

#[test]
fn cylinder_perpendicular_section_is_a_circle() {
	let r = 2.5;
	let cyl = Surface::Cylinder { origin: DVec3::ZERO, axis: DVec3::Z, radius: r };
	let curves = cyl.plane_section(DVec3::Z * 1.0, DVec3::Z);
	assert_eq!(curves.len(), 1);
	let Curve::Circle { radius, center, .. } = curves[0] else { panic!("expected a Circle") };
	assert!((radius - r).abs() < 1e-12);
	assert!((center - DVec3::Z).length() < 1e-12);
	assert_on_surface_and_plane(&curves[0], &cyl, DVec3::Z, DVec3::Z, PI, 1e-12);
}

#[test]
fn cylinder_oblique_section_is_an_ellipse_with_closed_form_axes() {
	// A plane tilted 45° from the axis cuts radius-r cylinder in an ellipse with
	// semi-minor b = r and semi-major a = r / |cos(angle(normal, axis))|.
	let r = 3.0;
	let cyl = Surface::Cylinder { origin: DVec3::ZERO, axis: DVec3::Z, radius: r };
	let normal = DVec3::new(0.0, 1.0, 1.0).normalize(); // 45° to z
	let curves = cyl.plane_section(DVec3::ZERO, normal);
	assert_eq!(curves.len(), 1);
	let Curve::Ellipse { a, b, u, normal: en, .. } = curves[0] else { panic!("expected an Ellipse") };
	assert!((b - r).abs() < 1e-12, "semi-minor should equal r, got {b}");
	let cos = DVec3::Z.dot(normal).abs();
	assert!((a - r / cos).abs() < 1e-12, "semi-major should be r/cos, got {a}");
	assert!(a >= b, "convention a >= b violated");
	assert!(u.dot(en).abs() < 1e-12, "major axis must lie in the cutting plane");
	assert_on_surface_and_plane(&curves[0], &cyl, DVec3::ZERO, normal, PI, 1e-12);
}

#[test]
fn cylinder_parallel_plane_cuts_two_lines_or_tangent_or_empty() {
	let r = 2.0;
	let cyl = Surface::Cylinder { origin: DVec3::ZERO, axis: DVec3::Z, radius: r };
	// Plane parallel to the axis, offset 1 < r along x → two lines.
	let two = cyl.plane_section(DVec3::X * 1.0, DVec3::X);
	assert_eq!(two.len(), 2, "secant parallel plane should give two lines");
	for c in &two {
		let Curve::Line { dir, .. } = *c else { panic!("expected Lines") };
		assert!(dir.normalize().dot(DVec3::Z).abs() > 1.0 - 1e-12, "lines run along the axis");
		assert_on_surface_and_plane(c, &cyl, DVec3::X, DVec3::X, 10.0, 1e-12);
	}
	// Tangent plane at x = r → a single line.
	let one = cyl.plane_section(DVec3::X * r, DVec3::X);
	assert_eq!(one.len(), 1, "tangent parallel plane should give one line");
	// Plane outside the cylinder → empty.
	assert!(cyl.plane_section(DVec3::X * (r + 0.5), DVec3::X).is_empty());
}

#[test]
fn cone_perpendicular_section_is_a_circle_growing_with_distance() {
	let half = (0.5_f64).atan(); // tan(half_angle) = 0.5
	let cone = Surface::Cone { apex: DVec3::ZERO, axis: DVec3::Z, half_angle: half };
	let s = 4.0; // axial distance from apex
	let curves = cone.plane_section(DVec3::Z * s, DVec3::Z);
	assert_eq!(curves.len(), 1);
	let Curve::Circle { radius, center, .. } = curves[0] else { panic!("expected a Circle") };
	assert!((radius - s * half.tan()).abs() < 1e-12, "radius {radius} != s*tan(half)");
	assert!((center - DVec3::Z * s).length() < 1e-12);
	assert_on_surface_and_plane(&curves[0], &cone, DVec3::Z * s, DVec3::Z, PI, 1e-9);
	// A plane through / behind the apex yields nothing in closed form here.
	assert!(cone.plane_section(DVec3::ZERO, DVec3::Z).is_empty());
}

#[test]
fn cone_oblique_section_is_an_ellipse_matching_generator_vertices() {
	// Cone apex at origin, axis +Z, tan(half_angle)=0.5. A plane tilted α=30° from
	// horizontal (normal in the x-z plane) cuts a closed ellipse (α < 90°−β). The
	// major-axis vertices are exactly where the plane meets the two x-z generators,
	// which we compute independently in closed form.
	let tanb = 0.5_f64;
	let half = tanb.atan();
	let cone = Surface::Cone { apex: DVec3::ZERO, axis: DVec3::Z, half_angle: half };
	let (alpha, h) = (30.0_f64.to_radians(), 4.0);
	let n = DVec3::new(alpha.sin(), 0.0, alpha.cos());
	let po = DVec3::Z * h;
	let curves = cone.plane_section(po, n);
	assert_eq!(curves.len(), 1);
	let Curve::Ellipse { center, u, a, b, normal: en } = curves[0] else { panic!("expected an Ellipse") };

	// Closed-form vertices: plane ∩ right/left generators (x = ±z·tanβ, z ≥ 0) in y=0.
	let z_r = h * alpha.cos() / (tanb * alpha.sin() + alpha.cos());
	let z_l = h * alpha.cos() / (alpha.cos() - tanb * alpha.sin());
	let vr = DVec3::new(z_r * tanb, 0.0, z_r);
	let vl = DVec3::new(-z_l * tanb, 0.0, z_l);
	let expect_center = (vr + vl) * 0.5;
	let expect_a = (vr - vl).length() * 0.5;
	assert!((center - expect_center).length() < 1e-9, "center {center} != {expect_center}");
	assert!((a - expect_a).abs() < 1e-9, "semi-major {a} != {expect_a}");
	let v0 = center + u * a;
	let v1 = center - u * a;
	let matched = (v0 - vr).length() < 1e-9 && (v1 - vl).length() < 1e-9
		|| (v0 - vl).length() < 1e-9 && (v1 - vr).length() < 1e-9;
	assert!(matched, "major vertices {v0},{v1} != generators {vr},{vl}");
	assert!(a > b && b > 0.0 && u.dot(en).abs() < 1e-12);
	assert_on_surface_and_plane(&curves[0], &cone, po, n, PI, 1e-9);
}

#[test]
fn cone_general_orientation_ellipse_lies_on_surface() {
	// Fully tilted apex/axis and an oblique plane steeper than the generators.
	let half = (0.4_f64).atan();
	let apex = DVec3::new(1.0, -2.0, 0.5);
	let axis = DVec3::new(0.2, 0.3, 1.0).normalize();
	let cone = Surface::Cone { apex, axis, half_angle: half };
	// Nearly along the axis → |cos| close to 1 > sin(half) → ellipse.
	let n = (axis + DVec3::new(0.1, -0.05, 0.0)).normalize();
	let po = apex + axis * 3.0;
	let curves = cone.plane_section(po, n);
	assert_eq!(curves.len(), 1);
	let Curve::Ellipse { a, b, .. } = curves[0] else { panic!("expected an Ellipse") };
	assert!(a >= b && b > 0.0);
	assert_on_surface_and_plane(&curves[0], &cone, po, n, PI, 1e-8);
}

/// Sample a generator ray (t ≥ 0 only — the single-nappe cone has no surface for t < 0).
fn assert_ray_on_surface(curve: &Curve, surface: &Surface, o: DVec3, n: DVec3, tol: f64) {
	for k in 0..40 {
		let t = 0.1 * (k as f64);
		let p = curve.point_at(t);
		assert!(surface.unsigned_distance(p) < tol, "ray off surface at t={t}");
		assert!((p - o).dot(n).abs() < tol, "ray off plane at t={t}");
	}
}

#[test]
fn cone_through_apex_hyperbola_gives_two_generator_rays() {
	let half = (0.5_f64).atan();
	let cone = Surface::Cone { apex: DVec3::ZERO, axis: DVec3::Z, half_angle: half };
	// Plane x = 0 (parallel to the axis) through the apex → two generators.
	let lines = cone.plane_section(DVec3::ZERO, DVec3::X);
	assert_eq!(lines.len(), 2, "hyperbola-through-apex → two generator lines");
	for c in &lines {
		let Curve::Line { origin, dir } = *c else { panic!("expected Lines") };
		assert!(origin.length() < 1e-12, "generator passes through the apex");
		assert!((dir.dot(DVec3::Z) - half.cos()).abs() < 1e-9, "angle to axis = half_angle");
		assert!(dir.dot(DVec3::X).abs() < 1e-12, "generator lies in the plane");
		assert_ray_on_surface(c, &cone, DVec3::ZERO, DVec3::X, 1e-9);
	}
}

#[test]
fn cone_through_apex_parabola_gives_one_generator_ray() {
	let half = (0.5_f64).atan();
	let cone = Surface::Cone { apex: DVec3::ZERO, axis: DVec3::Z, half_angle: half };
	let n = DVec3::new(half.cos(), 0.0, half.sin()); // |cos(angle to axis)| = sin β
	let lines = cone.plane_section(DVec3::ZERO, n);
	assert_eq!(lines.len(), 1, "parabola-through-apex → one tangent generator");
	let Curve::Line { dir, .. } = lines[0] else { panic!("expected a Line") };
	assert!((dir.dot(DVec3::Z) - half.cos()).abs() < 1e-9);
	assert_ray_on_surface(&lines[0], &cone, DVec3::ZERO, n, 1e-9);
}

#[test]
fn cone_through_apex_ellipse_is_just_the_apex_point() {
	let half = (0.5_f64).atan();
	let cone = Surface::Cone { apex: DVec3::ZERO, axis: DVec3::Z, half_angle: half };
	// Plane steeper than the generators (nearly ⟂ axis) through the apex → point only.
	let n = DVec3::new(0.1, 0.0, 1.0).normalize();
	assert!(cone.plane_section(DVec3::ZERO, n).is_empty(), "ellipse-through-apex has no curve");
}

#[test]
fn cone_axis_parallel_plane_is_a_hyperbola_branch_with_closed_form_axes() {
	// Cone apex origin, axis +Z, tan(half)=0.5. Plane x = c (parallel to axis) cuts
	// z²·tan²β − y² = c², i.e. a hyperbola with transverse a = c/tanβ, conjugate b = c,
	// centre (c,0,0), vertex (c,0,c/tanβ) on the real nappe.
	let tanb = 0.5_f64;
	let half = tanb.atan();
	let cone = Surface::Cone { apex: DVec3::ZERO, axis: DVec3::Z, half_angle: half };
	let c = 1.0;
	let curves = cone.plane_section(DVec3::X * c, DVec3::X);
	assert_eq!(curves.len(), 1);
	let Curve::Hyperbola { center, u, a, b, .. } = curves[0] else { panic!("expected a Hyperbola") };
	assert!((a - c / tanb).abs() < 1e-9 && (b - c).abs() < 1e-9, "axes a={a} b={b}");
	assert!((center - DVec3::new(c, 0.0, 0.0)).length() < 1e-9, "center {center}");
	let vertex = center + u * a;
	assert!((vertex - DVec3::new(c, 0.0, c / tanb)).length() < 1e-9, "vertex {vertex}");
	assert!((cone.unsigned_distance(vertex)).abs() < 1e-9, "vertex off cone");
	assert_on_surface_and_plane(&curves[0], &cone, DVec3::X * c, DVec3::X, 2.0, 1e-8);
}

#[test]
fn cone_generator_parallel_plane_is_a_parabola_on_the_surface() {
	// Plane normal (cosβ, 0, sinβ) makes |cos∠(n,axis)| = sinβ exactly → parabola.
	let half = (0.5_f64).atan();
	let cone = Surface::Cone { apex: DVec3::ZERO, axis: DVec3::Z, half_angle: half };
	let n = DVec3::new(half.cos(), 0.0, half.sin());
	let po = DVec3::Z * 2.0;
	let curves = cone.plane_section(po, n);
	assert_eq!(curves.len(), 1, "expected a parabola");
	let Curve::Parabola { vertex, focal, .. } = curves[0] else { panic!("expected a Parabola") };
	assert!(focal > 0.0 && focal.is_finite(), "focal {focal}");
	assert!(cone.unsigned_distance(vertex).abs() < 1e-8, "vertex off cone");
	// On-surface sampling validates vertex, axis, width dir AND focal together: a
	// wrong focal bends the quadratic term off the cone.
	assert_on_surface_and_plane(&curves[0], &cone, po, n, 4.0, 1e-8);
}

#[test]
fn general_orientation_cylinder_section_lies_on_surface() {
	// Adversarial: neither the axis nor the plane is coordinate-aligned. The
	// on-surface + on-plane sampling validates the full ellipse (center and both
	// semi-axis directions), so a swapped major/minor axis would fail here.
	let r = 1.7;
	let axis = DVec3::new(1.0, 2.0, -0.5).normalize();
	let origin = DVec3::new(-2.0, 0.5, 3.0);
	let cyl = Surface::Cylinder { origin, axis, radius: r };
	let normal = DVec3::new(-0.3, 1.0, 0.8).normalize();
	let plane_o = origin + axis * 1.3; // a point on the axis, so the plane surely cuts
	let curves = cyl.plane_section(plane_o, normal);
	assert_eq!(curves.len(), 1);
	let Curve::Ellipse { a, b, .. } = curves[0] else { panic!("expected an Ellipse") };
	let cos = axis.dot(normal).abs();
	assert!((b - r).abs() < 1e-12 && (a - r / cos).abs() < 1e-12, "axes a={a} b={b}");
	assert_on_surface_and_plane(&curves[0], &cyl, plane_o, normal, PI, 1e-9);
}

#[test]
fn section_is_independent_of_plane_normal_sign() {
	// Flipping the plane normal must describe the same plane → the same section.
	let sphere = Surface::Sphere { center: DVec3::new(1.0, 2.0, 3.0), radius: 4.0 };
	let o = DVec3::new(1.0, 2.0, 4.2);
	let up = sphere.plane_section(o, DVec3::Z);
	let down = sphere.plane_section(o, -DVec3::Z);
	let (Curve::Circle { radius: ru, center: cu, .. }, Curve::Circle { radius: rd, center: cd, .. }) =
		(up[0], down[0])
	else {
		panic!("expected circles");
	};
	assert!((ru - rd).abs() < 1e-12 && (cu - cd).length() < 1e-12, "section depends on normal sign");
}

#[test]
fn ellipse_parameterization_closes_and_tangent_is_unit() {
	// Sanity on the new Curve::Ellipse evaluator independent of any surface.
	let e = Curve::Ellipse { center: DVec3::ZERO, normal: DVec3::Z, u: DVec3::X, a: 4.0, b: 2.0 };
	assert!((e.point_at(0.0) - DVec3::X * 4.0).length() < 1e-12);
	assert!((e.point_at(0.0) - e.point_at(TAU)).length() < 1e-12, "ellipse must close");
	for k in 0..16 {
		let t = TAU * (k as f64) / 16.0;
		assert!((e.tangent_at(t).length() - 1.0).abs() < 1e-12, "tangent must be unit");
	}
}

// --- Solid-level section queries: the conic machinery consumed end-to-end ----------------

#[test]
fn solid_cone_oblique_sections_yield_parabola_and_hyperbola() {
	use kernel_brep::cone;
	// The cone∩plane conics consumed at the SOLID level: a Ø6×4 cone solid (apex (0,0,4),
	// half-angle atan(3/4), so sin β = 0.6 exactly). A plane whose unit normal satisfies
	// |n·axis| = sin β is parallel to one generator → exactly one Parabola; a vertical
	// plane (|n·axis| = 0 < sin β) not through the apex → exactly one Hyperbola branch.
	// Every sampled point of each conic must lie ON the solid's tagged cone surface and
	// ON the cutting plane to 1e-9 — exact curves, not approximations.
	let c = cone(DVec3::ZERO, DVec3::Z, 3.0, 4.0, 48);
	let surf = c
		.faces()
		.map(|f| c.face(f).surface)
		.find(|s| matches!(s, Surface::Cone { .. }))
		.expect("the cone solid carries Cone-tagged faces");

	let (po, n) = (DVec3::new(0.0, 0.0, 1.0), DVec3::new(0.0, 0.8, 0.6)); // |n·(−Z)| = 0.6 = sin β
	let sec = c.section_curves(po, n);
	let parabolas: Vec<&Curve> = sec.iter().filter(|k| matches!(k, Curve::Parabola { .. })).collect();
	assert_eq!(parabolas.len(), 1, "generator-parallel plane → one parabola, got {sec:?}");
	assert_on_surface_and_plane(parabolas[0], &surf, po, n.normalize(), 2.5, 1e-9);

	let (po2, n2) = (DVec3::new(0.0, 1.0, 0.0), DVec3::Y); // vertical plane y = 1
	let sec2 = c.section_curves(po2, n2);
	let hyperbolas: Vec<&Curve> = sec2.iter().filter(|k| matches!(k, Curve::Hyperbola { .. })).collect();
	assert_eq!(hyperbolas.len(), 1, "axis-shallow plane → one hyperbola branch, got {sec2:?}");
	assert_on_surface_and_plane(hyperbolas[0], &surf, po2, n2, 1.2, 1e-9);
}

#[test]
fn solid_cylinder_oblique_section_ellipse_has_closed_form_semi_axes() {
	use kernel_brep::cylinder;
	// The solid-level oblique cylinder section is an ellipse with the CLOSED-FORM axes:
	// semi-minor b = r exactly, semi-major a = r/|cos∠(n, axis)| exactly, major direction
	// in the cutting plane. (The surface-level test covers the formula; this pins the
	// SOLID query to the same exactness.)
	let r = 2.0;
	let cyl = cylinder(DVec3::ZERO, DVec3::Z, r, 5.0, 24);
	let n = DVec3::new(0.0, 0.4, 1.0).normalize();
	let sec = cyl.section_curves(DVec3::new(0.0, 0.0, 2.5), n);
	let Some(Curve::Ellipse { a, b, u, normal, .. }) = sec.iter().find(|k| matches!(k, Curve::Ellipse { .. })) else {
		panic!("an oblique cylinder section must include an ellipse, got {sec:?}");
	};
	let cos = n.z.abs();
	assert!(
		(*b - r).abs() < 1e-9 && (*a - r / cos).abs() < 1e-9 && u.dot(*normal).abs() < 1e-9,
		"solid oblique ellipse: a={a} (want {}), b={b} (want {r}), u·n={}",
		r / cos,
		u.dot(*normal)
	);
}

#[test]
fn torus_oblique_section_falls_back_to_chained_polylines() {
	use kernel_brep::{section_curves_with_fallback, torus, SectionCurve};
	// An oblique torus section is a QUARTIC with no closed form in the kernel (only cuts
	// ⟂ the axis are exact circles). The fallback must surface it as chained polylines —
	// here the tilted plane through the centre clips the tube on the ±X sides only, so the
	// section is exactly TWO substantial closed chains, every point exactly on the cutting
	// plane (1e-9) and within chord tolerance of the analytic torus — rather than an
	// empty (silently dropped) result. No exact conic exists for this cut.
	let (maj, min) = (8.0, 2.0);
	let t = torus(DVec3::ZERO, DVec3::Z, maj, min, 64, 32);
	let surf = Surface::Torus { center: DVec3::ZERO, axis: DVec3::Z, major: maj, minor: min };
	let n = DVec3::new(0.0, 0.35, 1.0).normalize();
	let secs = section_curves_with_fallback(&t, DVec3::ZERO, n);
	let polys: Vec<&Vec<DVec3>> = secs
		.iter()
		.filter_map(|k| match k {
			SectionCurve::Polyline(p) => Some(p),
			SectionCurve::Exact(_) => None,
		})
		.collect();
	let exact = secs.len() - polys.len();
	let max_plane = polys.iter().flat_map(|p| p.iter()).map(|p| p.dot(n).abs()).fold(0.0_f64, f64::max);
	let max_surf = polys.iter().flat_map(|p| p.iter()).map(|p| surf.unsigned_distance(*p)).fold(0.0_f64, f64::max);
	assert!(
		exact == 0 && polys.len() == 2 && polys.iter().all(|p| p.len() >= 16) && max_plane < 1e-9 && max_surf < 0.05,
		"oblique torus fallback: exact={exact} polylines={} (lens {:?}) on-plane {max_plane:.2e} on-torus {max_surf:.3}",
		polys.len(),
		polys.iter().map(|p| p.len()).collect::<Vec<_>>()
	);
}
