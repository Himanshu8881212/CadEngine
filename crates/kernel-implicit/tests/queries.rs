// Copyright (c) LMCAD. Licensed under the MIT License.

//! Interaction primitives on a meshed solid: ray picking and closest-point
//! queries — the building blocks of click-to-select, snapping and measurement.

use kernel_core::Ray;
use kernel_implicit::surface_nets;
use kernel_implicit::{Cuboid, Cylinder, Node, Resolution, Sdf, Sphere, Vec3};

/// A tiny deterministic xorshift PRNG for reproducible random queries.
struct Rng(u64);
impl Rng {
	fn next(&mut self) -> u32 {
		self.0 ^= self.0 << 13;
		self.0 ^= self.0 >> 7;
		self.0 ^= self.0 << 17;
		(self.0 >> 32) as u32
	}
	fn f(&mut self, lo: f32, hi: f32) -> f32 {
		lo + (self.next() as f32 / u32::MAX as f32) * (hi - lo)
	}
}

fn sphere_mesh(r: f32) -> kernel_core::Mesh {
	let s = Node::primitive(Sphere::new(Vec3::ZERO, r));
	surface_nets(&s, s.bounds(), Resolution::VoxelSize(0.3))
}

#[test]
fn raycast_hits_the_near_side_of_a_sphere() {
	// A ray fired up the +Z axis at a Ø20 sphere must hit the NEAR cap (z ≈ −10),
	// not the far one, at distance ≈ 10 with an outward normal facing the ray.
	let m = sphere_mesh(10.0);
	let hit = m.raycast(Ray::new(Vec3::new(0.0, 0.0, -20.0), Vec3::Z)).expect("ray should hit");
	assert!((hit.t - 10.0).abs() < 0.4, "hit distance {} vs 10", hit.t);
	assert!((hit.point - Vec3::new(0.0, 0.0, -10.0)).length() < 0.4, "hit point {:?}", hit.point);
	assert!(hit.normal.dot(-Vec3::Z) > 0.7, "normal {:?} should face the ray", hit.normal);
	// The hit lies on the triangle the report names.
	let t = &m.indices[hit.triangle * 3..hit.triangle * 3 + 3];
	assert!(t.iter().all(|&i| (i as usize) < m.positions.len()), "triangle index in range");
}

#[test]
fn raycast_misses_when_offset_beyond_the_radius() {
	let m = sphere_mesh(10.0);
	let miss = m.raycast(Ray::new(Vec3::new(20.0, 0.0, -20.0), Vec3::Z));
	assert!(miss.is_none(), "a ray offset past the radius must miss");
}

#[test]
fn raycast_ignores_geometry_behind_the_origin() {
	// Origin inside the sphere, firing +Z: the only forward hit is the +Z cap at
	// z ≈ +10; the −Z cap is behind the origin and must be ignored.
	let m = sphere_mesh(10.0);
	let hit = m.raycast(Ray::new(Vec3::ZERO, Vec3::Z)).expect("interior ray exits the front");
	assert!((hit.point.z - 10.0).abs() < 0.4, "should exit at +Z cap, got {:?}", hit.point);
}

#[test]
fn closest_point_to_an_exterior_query_lands_on_the_surface() {
	// The nearest surface point to (20,0,0) outside a Ø20 sphere is ≈ (10,0,0) at
	// distance ≈ 10.
	let m = sphere_mesh(10.0);
	let cp = m.closest_point(Vec3::new(20.0, 0.0, 0.0)).expect("non-empty mesh");
	assert!((cp.distance - 10.0).abs() < 0.4, "distance {} vs 10", cp.distance);
	assert!((cp.point - Vec3::new(10.0, 0.0, 0.0)).length() < 0.4, "closest point {:?}", cp.point);
}

#[test]
fn closest_point_to_the_center_is_the_radius_away() {
	// From the center, every surface point is ~r away; the query must return one of
	// them at distance ≈ r (and actually on the surface).
	let m = sphere_mesh(10.0);
	let cp = m.closest_point(Vec3::ZERO).expect("non-empty mesh");
	assert!((cp.distance - 10.0).abs() < 0.5, "distance {} vs ~10", cp.distance);
	assert!((cp.point.length() - 10.0).abs() < 0.5, "point not on the surface: {:?}", cp.point);
}

#[test]
fn closest_point_on_a_box_face_is_the_foot_of_the_perpendicular() {
	// A 20 mm cube and a query straight off one face: the closest point is the foot
	// of the perpendicular on that face, distance = standoff.
	let c = Node::primitive(Cuboid::new(Vec3::ZERO, Vec3::splat(10.0)));
	let m = surface_nets(&c, c.bounds(), Resolution::VoxelSize(0.5));
	let cp = m.closest_point(Vec3::new(0.0, 0.0, 18.0)).expect("non-empty");
	assert!((cp.distance - 8.0).abs() < 0.5, "standoff {} vs 8", cp.distance);
	assert!(cp.point.x.abs() < 0.6 && cp.point.y.abs() < 0.6 && (cp.point.z - 10.0).abs() < 0.5, "closest point {:?}", cp.point);
}

/// Mesh a Ø(2r) sphere centred at `c`.
fn sphere_at(c: Vec3, r: f32) -> kernel_core::Mesh {
	let s = Node::primitive(Sphere::new(c, r));
	surface_nets(&s, s.bounds(), Resolution::VoxelSize(0.6))
}

#[test]
fn clearance_between_two_separated_spheres_is_the_gap() {
	// Two Ø16 spheres, centres 30 apart → surface gap = 30 − 8 − 8 = 14.
	let a = sphere_at(Vec3::new(-15.0, 0.0, 0.0), 8.0);
	let b = sphere_at(Vec3::new(15.0, 0.0, 0.0), 8.0);
	let d = a.min_distance(&b);
	assert!((d - 14.0).abs() < 0.5, "clearance {d} vs 14");
}

#[test]
fn interfering_solids_report_zero_clearance() {
	// Centres 10 apart, radii 8 each → they overlap; the triangle-intersection test
	// must drive the clearance to exactly 0.
	let a = sphere_at(Vec3::new(-5.0, 0.0, 0.0), 8.0);
	let b = sphere_at(Vec3::new(5.0, 0.0, 0.0), 8.0);
	assert_eq!(a.min_distance(&b), 0.0, "overlapping solids must report zero clearance");
}

#[test]
fn clearance_between_a_box_and_a_sphere() {
	// Box spanning x∈[−25,−15] and a sphere of radius 8 at the origin (x∈[−8,8]):
	// the nearest faces are at x=−15 and x=−8, a 7 mm gap.
	let cube = Node::primitive(Cuboid::new(Vec3::new(-20.0, 0.0, 0.0), Vec3::splat(5.0)));
	let m_cube = surface_nets(&cube, cube.bounds(), Resolution::VoxelSize(0.6));
	let m_sphere = sphere_at(Vec3::ZERO, 8.0);
	let d = m_cube.min_distance(&m_sphere);
	assert!((d - 7.0).abs() < 0.6, "box↔sphere clearance {d} vs 7");
}

#[test]
fn bvh_matches_bruteforce_raycast_and_closest_point() {
	// A two-component, non-convex shape so the BVH genuinely branches.
	let shape = Node::primitive(Sphere::new(Vec3::new(-6.0, 0.0, 0.0), 7.0))
		.union(Node::primitive(Cuboid::new(Vec3::new(8.0, 0.0, 0.0), Vec3::splat(5.0))));
	let m = surface_nets(&shape, shape.bounds(), Resolution::VoxelSize(0.5));
	let bvh = m.build_bvh();
	let mut rng = Rng(0xC0FFEE123);

	// Random rays: the accelerated and brute-force results must agree exactly on
	// hit/miss and (when hit) on distance and point.
	let mut hits = 0;
	for _ in 0..150 {
		let o = Vec3::new(rng.f(-30.0, 30.0), rng.f(-30.0, 30.0), rng.f(-30.0, 30.0));
		let target = Vec3::new(rng.f(-12.0, 14.0), rng.f(-8.0, 8.0), rng.f(-8.0, 8.0));
		let dir = (target - o).normalize_or_zero();
		if dir == Vec3::ZERO {
			continue;
		}
		let ray = Ray::new(o, dir);
		match (m.raycast(ray), bvh.raycast(ray)) {
			(None, None) => {}
			(Some(x), Some(y)) => {
				assert!((x.t - y.t).abs() < 1e-3, "ray t {} vs {}", x.t, y.t);
				assert!((x.point - y.point).length() < 1e-2, "ray point mismatch {:?} vs {:?}", x.point, y.point);
				hits += 1;
			}
			(a, b) => panic!("BVH/brute-force disagree on hit/miss: {:?} vs {:?}", a.map(|h| h.t), b.map(|h| h.t)),
		}
	}
	assert!(hits > 20, "expected many hits, got {hits}");

	// Random closest-point queries must match.
	for _ in 0..150 {
		let q = Vec3::new(rng.f(-25.0, 25.0), rng.f(-25.0, 25.0), rng.f(-25.0, 25.0));
		let a = m.closest_point(q).expect("non-empty");
		let b = bvh.closest_point(q).expect("non-empty");
		assert!((a.distance - b.distance).abs() < 1e-3, "closest dist {} vs {}", a.distance, b.distance);
		assert!((a.point - b.point).length() < 1e-2, "closest point mismatch {:?} vs {:?}", a.point, b.point);
	}
}

#[test]
fn bvh_min_distance_matches_bruteforce_and_analytic_gap() {
	// The BVH-accelerated clearance must equal both the brute-force result and the
	// analytic gap, across separated, interfering, and mixed-shape pairs.
	let cases: [(kernel_core::Mesh, kernel_core::Mesh, f64); 3] = [
		(sphere_at(Vec3::new(-15.0, 0.0, 0.0), 8.0), sphere_at(Vec3::new(15.0, 0.0, 0.0), 8.0), 14.0),
		(sphere_at(Vec3::new(-5.0, 0.0, 0.0), 8.0), sphere_at(Vec3::new(5.0, 0.0, 0.0), 8.0), 0.0),
		({
			let c = Node::primitive(Cuboid::new(Vec3::new(-20.0, 0.0, 0.0), Vec3::splat(5.0)));
			surface_nets(&c, c.bounds(), Resolution::VoxelSize(0.6))
		}, sphere_at(Vec3::ZERO, 8.0), 7.0),
	];
	for (a, b, expected) in &cases {
		let brute = a.min_distance(b);
		let fast = a.build_bvh().min_distance(&b.build_bvh());
		assert!((brute - fast).abs() < 1e-4, "BVH {fast} != brute {brute}");
		assert!((fast - expected).abs() < 0.6, "clearance {fast} vs expected {expected}");
	}
}

#[test]
fn draft_analysis_detects_an_undercut_hole() {
	// A 20 mm cube with a Ø8 hole bored perpendicular to the pull direction: the hole
	// walls are trapped between the two mold halves → undercut.
	let cube = Node::primitive(Cuboid::new(Vec3::ZERO, Vec3::splat(10.0)));
	let hole = Node::primitive(Cylinder::new(Vec3::new(-15.0, 0.0, 0.0), Vec3::new(15.0, 0.0, 0.0), 4.0));
	let bounds = cube.bounds();
	let m = surface_nets(&cube.difference(hole), bounds, Resolution::VoxelSize(0.4));
	let r = m.draft_analysis(Vec3::Z, 3.0);
	assert!(r.undercut_area > 200.0, "the perpendicular hole walls should be undercut, got {}", r.undercut_area);
}
