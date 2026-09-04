// Copyright (c) LMCAD. Licensed under the MIT License.

//! Exact curved cutting: trimming a mesh by an analytic surface must place every
//! new boundary vertex on that surface to (f32-stored) precision — far tighter
//! than the voxel-grid seam of the implicit path.

use std::collections::HashSet;

use kernel_brep::math::{DVec3, Vec3};
use kernel_brep::{
	boundary_loops, drill_cylinder, intersect_sphere, seam_loops, sphere, subtract_cone, subtract_sphere, tessellate_default,
	trim_mesh_by_surface, union_sphere, Keep, Surface,
};
use kernel_core::mesh::Mesh;

/// A cube `[-half, half]³` with each face subdivided into `n × n` quads, so the
/// mesh has vertices on the faces (not only at the corners) for a cut to bite.
fn grid_cube(half: f32, n: usize) -> Mesh {
	let mut m = Mesh::new();
	let faces: [(usize, f32); 6] = [(0, 1.0), (0, -1.0), (1, 1.0), (1, -1.0), (2, 1.0), (2, -1.0)];
	for (axis, sign) in faces {
		let (a1, a2) = ((axis + 1) % 3, (axis + 2) % 3);
		let base = m.positions.len() as u32;
		let row = (n + 1) as u32;
		for i in 0..=n {
			for j in 0..=n {
				let mut p = [0.0f32; 3];
				p[axis] = sign * half;
				p[a1] = -half + 2.0 * half * (i as f32) / (n as f32);
				p[a2] = -half + 2.0 * half * (j as f32) / (n as f32);
				m.positions.push(Vec3::new(p[0], p[1], p[2]));
			}
		}
		for i in 0..n as u32 {
			for j in 0..n as u32 {
				let v00 = base + i * row + j;
				let (v01, v10, v11) = (v00 + 1, v00 + row, v00 + row + 1);
				// Wind outward: the (v00,v10,v11) order yields +axis; flip it for the
				// negative faces so all six are consistently outward and weld into a
				// closed cube (otherwise shared edges aren't proper twins).
				if sign > 0.0 {
					m.push_triangle(v00, v10, v11);
					m.push_triangle(v00, v11, v01);
				} else {
					m.push_triangle(v00, v11, v10);
					m.push_triangle(v00, v01, v11);
				}
			}
		}
	}
	m
}

#[test]
fn cube_trimmed_by_sphere_has_its_cut_exactly_on_the_sphere() {
	let cube = grid_cube(1.0, 10);
	let r = 1.2;
	let ball = Surface::Sphere { center: DVec3::ZERO, radius: r };
	let trimmed = trim_mesh_by_surface(&cube, &ball, Keep::Inside);
	assert!(trimmed.triangle_count() > 0, "cut should keep the inside region");

	// Every kept vertex is inside or on the ball; the cut boundary is on the sphere.
	// No original grid vertex falls within 1e-4 of r, so an on-sphere vertex can only
	// be a Newton-placed crossing — counting them verifies the cut is exact.
	let mut cut_vertices = 0;
	for &v in &trimmed.positions {
		let d = v.length();
		assert!(d <= r as f32 + 1e-4, "vertex outside the ball: {d}");
		if (d - r as f32).abs() < 1e-4 {
			cut_vertices += 1;
		}
	}
	assert!(cut_vertices > 20, "expected a populated cut boundary on the sphere, got {cut_vertices}");
}

#[test]
fn cut_seam_is_one_closed_loop_lying_on_the_cutting_surface() {
	// A ball resting on the top face pokes through it alone, so the exact cut seam
	// is a single closed circle — recovered as one boundary loop on the sphere.
	let cube = grid_cube(1.0, 12);
	let ctr = DVec3::new(0.0, 0.0, 1.0);
	let ball = Surface::Sphere { center: ctr, radius: 0.6 };
	let trimmed = trim_mesh_by_surface(&cube, &ball, Keep::Outside);
	let loops = seam_loops(&trimmed);
	assert_eq!(loops.len(), 1, "ball through one face → one seam loop");
	assert!(loops[0].len() > 8, "seam should be finely sampled, got {}", loops[0].len());
	for &p in &loops[0] {
		assert!(ball.signed_value(p).abs() < 1e-4, "seam vertex off the sphere: {}", ball.signed_value(p));
		assert!((p.z - 1.0).abs() < 1e-4, "seam should lie in the top face plane");
	}
}

#[test]
fn subtract_sphere_is_a_closed_solid_walled_on_the_sphere() {
	// Carve a ball that enters through the top face only (centre below the face, so
	// the seam is a small circle, not an ambiguous great circle).
	let cube = grid_cube(1.0, 12);
	let center = DVec3::new(0.0, 0.0, 0.7);
	let r = 0.5;
	let result = subtract_sphere(&cube, center, r);

	assert!(result.is_watertight(), "result must be a closed 2-manifold");
	// Genus 0 ⇒ Euler characteristic V − E + F = 2.
	let f = result.triangle_count() as i64;
	let mut edges: HashSet<(u32, u32)> = HashSet::new();
	for t in result.indices.chunks_exact(3) {
		for &(a, b) in &[(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
			edges.insert(if a < b { (a, b) } else { (b, a) });
		}
	}
	let (v, e) = (result.positions.len() as i64, edges.len() as i64);
	assert_eq!(v - e + f, 2, "Euler characteristic (V={v} E={e} F={f}) should be 2");

	// Nothing kept inside the removed ball; the carved wall lies on the sphere.
	let surf = Surface::Sphere { center, radius: r };
	let mut on_sphere = 0;
	for &p in &result.positions {
		let sd = surf.signed_value(p.as_dvec3());
		assert!(sd >= -1e-4, "vertex inside the removed ball: {sd}");
		if sd.abs() < 1e-4 {
			on_sphere += 1;
		}
	}
	assert!(on_sphere > 10, "carved wall should lie on the sphere, got {on_sphere}");
	// The ring cap follows the sphere, so the removed dimple matches the true ball∩cube
	// volume (0.469): result ≈ 8 − 0.469 = 7.531. A flat cone cap would give ~7.87.
	let vol = result.signed_volume();
	assert!((vol - 7.531).abs() < 0.05, "dimple should follow the sphere; volume {vol} (expected ≈7.531)");
}

#[test]
fn union_sphere_adds_a_bump_on_the_sphere() {
	// The same ball, unioned, pokes a spherical bump out through the top face.
	let cube = grid_cube(1.0, 12);
	let center = DVec3::new(0.0, 0.0, 0.7);
	let r = 0.5;
	let result = union_sphere(&cube, center, r);

	assert!(result.is_watertight(), "result must be a closed 2-manifold");
	let f = result.triangle_count() as i64;
	let mut edges: HashSet<(u32, u32)> = HashSet::new();
	for t in result.indices.chunks_exact(3) {
		for &(a, b) in &[(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
			edges.insert(if a < b { (a, b) } else { (b, a) });
		}
	}
	let (v, e) = (result.positions.len() as i64, edges.len() as i64);
	assert_eq!(v - e + f, 2, "Euler characteristic (V={v} E={e} F={f}) should be 2");

	// The bump lies on the sphere and pokes out above the top face.
	let surf = Surface::Sphere { center, radius: r };
	let mut on_sphere = 0;
	let mut above = 0;
	for &p in &result.positions {
		if surf.signed_value(p.as_dvec3()).abs() < 1e-4 {
			on_sphere += 1;
		}
		if p.z > 1.0 + 1e-5 {
			above += 1;
		}
	}
	assert!(on_sphere > 10 && above > 5, "expected a sphere bump above the face (on={on_sphere}, above={above})");
	// Volume = cube + the ball cap above z=1 (≈0.0545): ≈8.0545.
	let vol = result.signed_volume();
	assert!((vol - 8.0545).abs() < 0.05, "union should add a bump; volume {vol} (expected ≈8.0545)");
}

#[test]
fn intersect_sphere_is_the_clipped_ball_lens() {
	// cube ∩ ball = the ball clipped to z ≤ 1: a closed lens of volume ball∩cube.
	let cube = grid_cube(1.0, 12);
	let center = DVec3::new(0.0, 0.0, 0.7);
	let r = 0.5;
	let result = intersect_sphere(&cube, center, r);

	assert!(result.is_watertight(), "intersection must be a closed 2-manifold");
	let f = result.triangle_count() as i64;
	let mut edges: HashSet<(u32, u32)> = HashSet::new();
	for t in result.indices.chunks_exact(3) {
		for &(a, b) in &[(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
			edges.insert(if a < b { (a, b) } else { (b, a) });
		}
	}
	let (v, e) = (result.positions.len() as i64, edges.len() as i64);
	assert_eq!(v - e + f, 2, "Euler characteristic (V={v} E={e} F={f}) should be 2");

	// Every vertex is inside-or-on the ball and within the cube (z ≤ 1).
	let surf = Surface::Sphere { center, radius: r };
	for &p in &result.positions {
		assert!(surf.signed_value(p.as_dvec3()) <= 1e-4, "vertex outside the ball");
		assert!(p.z <= 1.0 + 1e-4, "vertex above the cube");
	}
	// Volume = ball ∩ cube ≈ 0.4691. The mesh is ~6% under at this tessellation (the
	// seam N-gon and ring cap sit just inside the sphere); a flat cone cap would give
	// only ≈0.13, so this still confirms the cap follows the sphere.
	let vol = result.signed_volume();
	assert!((vol - 0.4691).abs() < 0.04, "intersection volume {vol} (expected ≈0.469)");
}

#[test]
fn curved_tools_that_miss_or_engulf_degrade_gracefully() {
	let cube = grid_cube(1.0, 8);
	let v0 = cube.signed_volume();

	// A ball far away → no cut, a watertight no-op.
	let miss = subtract_sphere(&cube, DVec3::new(10.0, 0.0, 0.0), 0.5);
	assert!(miss.is_watertight() && (miss.signed_volume() - v0).abs() < 1e-3, "missing ball should be a no-op");

	// A cylinder far from the cube → no bore; drill returns the cube unchanged.
	let no_bore = drill_cylinder(&cube, DVec3::new(10.0, 0.0, 0.0), DVec3::Z, 0.3);
	assert!(no_bore.is_watertight() && (no_bore.signed_volume() - v0).abs() < 1e-3, "missing drill should be a no-op");

	// A ball that engulfs the whole cube → everything removed (empty result).
	let engulfed = subtract_sphere(&cube, DVec3::ZERO, 5.0);
	assert_eq!(engulfed.triangle_count(), 0, "subtracting an engulfing ball leaves nothing");
}

#[test]
fn boundary_loops_splits_a_pinch_into_two() {
	// Two triangles touching only at vertex 0 form two separate boundary loops that
	// meet at the pinch. The edge-consuming walk must recover both, not overwrite one.
	let mut m = Mesh::new();
	for p in [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [-1.0, 1.0, 0.0]] {
		m.positions.push(Vec3::new(p[0], p[1], p[2]));
	}
	m.push_triangle(0, 1, 2);
	m.push_triangle(0, 3, 4);
	let mut loops = boundary_loops(&m);
	loops.sort_by_key(|l| l.len());
	assert_eq!(loops.len(), 2, "a pinch vertex must split into two loops, not merge");
	assert!(loops.iter().all(|l| l.len() == 3));
}

#[test]
fn subtract_cone_carves_a_pit_walled_on_the_cone() {
	// Apex at the cube centre, opening up through the top face: a conical countersink
	// with base radius 0.5 at z=1 (tan(half)=0.5), removing a cone of volume ≈0.262.
	let cube = grid_cube(1.0, 14);
	let apex = DVec3::ZERO;
	let half = (0.5_f64).atan();
	let result = subtract_cone(&cube, apex, DVec3::Z, half);

	assert!(result.is_watertight(), "result must be a closed 2-manifold");
	let f = result.triangle_count() as i64;
	let mut edges: HashSet<(u32, u32)> = HashSet::new();
	for t in result.indices.chunks_exact(3) {
		for &(a, b) in &[(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
			edges.insert(if a < b { (a, b) } else { (b, a) });
		}
	}
	let (v, e) = (result.positions.len() as i64, edges.len() as i64);
	assert_eq!(v - e + f, 2, "Euler characteristic (V={v} E={e} F={f}) should be 2");

	let surf = Surface::Cone { apex, axis: DVec3::Z, half_angle: half };
	let mut on_cone = 0;
	for &p in &result.positions {
		assert!(surf.signed_value(p.as_dvec3()) >= -1e-4, "vertex inside the removed cone");
		if surf.signed_value(p.as_dvec3()).abs() < 1e-4 {
			on_cone += 1;
		}
	}
	assert!(on_cone > 10, "pit wall should lie on the cone, got {on_cone}");
	// Volume = cube − cone((1/3)π·0.25·1 ≈ 0.262) ≈ 7.738 (tessellation-limited).
	let vol = result.signed_volume();
	assert!((vol - 7.738).abs() < 0.03, "conical pit volume {vol} (expected ≈7.738)");
}

#[test]
fn drill_cylinder_makes_a_genus_one_bore_on_the_cylinder() {
	// Drill a radius-0.3 hole straight through the cube along z. The result is a
	// torus-topology (genus-1) solid; its bore wall lies on the cylinder.
	let cube = grid_cube(1.0, 12);
	let r = 0.3;
	let result = drill_cylinder(&cube, DVec3::ZERO, DVec3::Z, r);

	assert!(result.is_watertight(), "drilled solid must be a closed 2-manifold");
	// A through-hole is genus 1 ⇒ Euler characteristic V − E + F = 0.
	let f = result.triangle_count() as i64;
	let mut edges: HashSet<(u32, u32)> = HashSet::new();
	for t in result.indices.chunks_exact(3) {
		for &(a, b) in &[(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
			edges.insert(if a < b { (a, b) } else { (b, a) });
		}
	}
	let (v, e) = (result.positions.len() as i64, edges.len() as i64);
	assert_eq!(v - e + f, 0, "genus-1 Euler characteristic (V={v} E={e} F={f}) should be 0");

	// Bore-wall vertices lie on the cylinder; nothing kept inside it.
	let radial = |p: Vec3| (p.x * p.x + p.y * p.y).sqrt();
	let mut on_cyl = 0;
	for &p in &result.positions {
		assert!(radial(p) >= r as f32 - 1e-4, "vertex inside the bore");
		if (radial(p) - r as f32).abs() < 1e-4 {
			on_cyl += 1;
		}
	}
	assert!(on_cyl > 20, "bore wall should lie on the cylinder, got {on_cyl}");
	// Volume = cube − through-cylinder (π·0.09·2 ≈ 0.565) ≈ 7.435 (tessellation-limited).
	let vol = result.signed_volume();
	assert!((vol - 7.435).abs() < 0.05, "drilled volume {vol} (expected ≈7.435)");
}

#[test]
fn oblique_drill_zippers_unequal_rims_into_a_genus_one_bore() {
	// A tilted axis enters/exits through the top and bottom faces at different
	// positions, so the two seam rims need not have equal vertex counts — exercising
	// the greedy angle zipper rather than the matched ladder.
	let cube = grid_cube(1.0, 12);
	let axis = DVec3::new(0.3, 0.0, 1.0).normalize();
	let r = 0.3;
	let result = drill_cylinder(&cube, DVec3::ZERO, axis, r);

	assert!(result.is_watertight(), "oblique-drilled solid must be a closed 2-manifold");
	let f = result.triangle_count() as i64;
	let mut edges: HashSet<(u32, u32)> = HashSet::new();
	for t in result.indices.chunks_exact(3) {
		for &(a, b) in &[(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
			edges.insert(if a < b { (a, b) } else { (b, a) });
		}
	}
	let (v, e) = (result.positions.len() as i64, edges.len() as i64);
	assert_eq!(v - e + f, 0, "genus-1 Euler characteristic (V={v} E={e} F={f}) should be 0");

	// Bore-wall vertices lie on the (tilted) cylinder: radial distance from the axis ≈ r.
	let radial = |p: Vec3| {
		let d = p.as_dvec3();
		(d - axis * d.dot(axis)).length()
	};
	let mut on_cyl = 0;
	for &p in &result.positions {
		assert!(radial(p) >= r - 1e-4, "vertex inside the bore: {}", radial(p));
		if (radial(p) - r).abs() < 1e-4 {
			on_cyl += 1;
		}
	}
	assert!(on_cyl > 20, "bore wall should lie on the cylinder, got {on_cyl}");
	let vol = result.signed_volume();
	assert!(vol > 7.0 && vol < 8.0, "an oblique bore should remove some material; volume {vol}");
}

#[test]
fn six_face_cut_yields_six_seam_loops() {
	// A ball larger than the half-width but smaller than the half-diagonal pokes
	// through all six faces, leaving six independent seam circles.
	let cube = grid_cube(1.0, 12);
	let ball = Surface::Sphere { center: DVec3::ZERO, radius: 1.2 };
	let trimmed = trim_mesh_by_surface(&cube, &ball, Keep::Inside);
	let loops = seam_loops(&trimmed);
	assert_eq!(loops.len(), 6, "ball through six faces → six seam loops");
	for loop_pts in &loops {
		for &p in loop_pts {
			assert!(ball.signed_value(p).abs() < 1e-4, "seam vertex off the sphere");
		}
	}
}

#[test]
fn sphere_mesh_trimmed_by_cylinder_cut_is_exactly_on_the_cylinder() {
	// A curved mesh cut by a curved surface — impossible to do exactly through the
	// planar boolean. Drill a radius-0.5 cylindrical bore out of a sphere shell.
	let shell = tessellate_default(&sphere(DVec3::ZERO, 1.5, 48, 32));
	let bore = 0.5;
	let drill = Surface::Cylinder { origin: DVec3::ZERO, axis: DVec3::Z, radius: bore };
	let trimmed = trim_mesh_by_surface(&shell, &drill, Keep::Outside);
	assert!(trimmed.triangle_count() > 0);

	let radial = |v: Vec3| (v.x * v.x + v.y * v.y).sqrt();
	let mut cut_vertices = 0;
	for &v in &trimmed.positions {
		assert!(radial(v) >= bore as f32 - 1e-4, "vertex inside the bore: {}", radial(v));
		if (radial(v) - bore as f32).abs() < 1e-4 {
			cut_vertices += 1;
		}
	}
	assert!(cut_vertices > 10, "expected a cut boundary on the bore, got {cut_vertices}");
}
