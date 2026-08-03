// Copyright (c) LMCAD. Licensed under the MIT License.

//! Convex-hull acceptance: the hull of a known shape, exclusion of interior
//! points, and the defining property — convex and enclosing every input point.

use kernel_core::{convex_hull, Vec3};

fn cube_corners(h: f32) -> Vec<Vec3> {
	let mut v = Vec::new();
	for &x in &[-h, h] {
		for &y in &[-h, h] {
			for &z in &[-h, h] {
				v.push(Vec3::new(x, y, z));
			}
		}
	}
	v
}

/// Convexity + enclosure: every input point lies on or under every hull face.
fn assert_convex_hull_of(points: &[Vec3], hull: &kernel_core::Mesh) {
	assert!(hull.signed_volume() > 0.0, "hull should be outward-oriented, got {}", hull.signed_volume());
	let eps = 1e-3 * hull.aabb().size().length().max(1.0);
	for t in hull.indices.chunks_exact(3) {
		let a = hull.positions[t[0] as usize];
		let b = hull.positions[t[1] as usize];
		let c = hull.positions[t[2] as usize];
		let n = (b - a).cross(c - a).normalize_or_zero(); // outward
		for &p in points {
			assert!(n.dot(p - a) <= eps, "point {p:?} lies outside a hull face (signed dist {})", n.dot(p - a));
		}
	}
}

#[test]
fn hull_of_a_cube_is_the_cube() {
	let pts = cube_corners(1.0);
	let hull = convex_hull(&pts);
	assert!((hull.signed_volume() - 8.0).abs() < 1e-3, "cube hull volume {} vs 8", hull.signed_volume());
	assert_eq!(hull.vertex_count(), 8, "a cube hull keeps all 8 corners");
	assert_convex_hull_of(&pts, &hull);
}

#[test]
fn interior_points_do_not_change_the_hull() {
	let mut pts = cube_corners(1.0);
	pts.push(Vec3::ZERO);
	pts.push(Vec3::new(0.5, -0.3, 0.2));
	let hull = convex_hull(&pts);
	assert_eq!(hull.vertex_count(), 8, "interior points must not appear on the hull");
	assert!((hull.signed_volume() - 8.0).abs() < 1e-3, "hull volume {} vs 8", hull.signed_volume());
}

#[test]
fn hull_of_a_random_cloud_is_convex_and_encloses_it() {
	// A random point cloud: the hull must be convex and contain every point.
	let mut s = 0x1234_5678u64;
	let mut rng = || {
		s ^= s << 13;
		s ^= s >> 7;
		s ^= s << 17;
		(s >> 40) as f32 / (1u64 << 24) as f32
	};
	let pts: Vec<Vec3> = (0..300).map(|_| Vec3::new(rng() * 4.0 - 2.0, rng() * 3.0 - 1.5, rng() * 5.0 - 2.5)).collect();
	let hull = convex_hull(&pts);
	assert!(hull.is_watertight(), "the hull must be a closed surface");
	assert!(hull.vertex_count() < pts.len(), "the hull should use only the extreme points");
	assert_convex_hull_of(&pts, &hull);
}

#[test]
fn near_coplanar_sliver_cloud_is_empty_or_watertight() {
	// High-aspect-ratio (near-coplanar) clouds previously produced a silently corrupt
	// non-manifold mesh. The hull must now be EITHER empty OR a valid watertight mesh.
	let mut s = 0x9E37_79B9u64;
	let mut rng = || {
		s ^= s << 13;
		s ^= s >> 7;
		s ^= s << 17;
		(s >> 40) as f32 / (1u64 << 24) as f32
	};
	for _ in 0..60 {
		let pts: Vec<Vec3> = (0..40).map(|_| Vec3::new(rng() * 10.0, rng() * 10.0, (rng() - 0.5) * 0.001)).collect();
		let h = convex_hull(&pts);
		assert!(h.is_empty() || h.is_watertight(), "sliver hull is neither empty nor watertight ({} tris)", h.triangle_count());
	}
}

#[test]
fn degenerate_inputs_return_empty() {
	assert!(convex_hull(&[Vec3::ZERO, Vec3::X, Vec3::Y]).is_empty(), "fewer than 4 points has no 3-D hull");
	// All coplanar (z = 0) → no volume → empty.
	let planar: Vec<Vec3> = (0..10).map(|i| Vec3::new(i as f32, (i * i) as f32 % 7.0, 0.0)).collect();
	assert!(convex_hull(&planar).is_empty(), "a coplanar set has no 3-D hull");
}
