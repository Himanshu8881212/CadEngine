// Copyright (c) LMCAD. Licensed under the MIT License.

//! Property-based (fuzz) coverage for the curved boolean. A ball that enters one
//! cube face cleanly (constrained off the great-circle / tangent / multi-face
//! degeneracies) must always carve a *valid* solid: a watertight, genus-0 mesh
//! whose volume is the cube minus a spherical cap. Exercises trim + seam + ring cap
//! across many random tool placements.

use std::collections::HashSet;
use std::f64::consts::PI;

use kernel_brep::math::{DVec3, Vec3};
use kernel_brep::{drill_cylinder, subtract_sphere};
use kernel_core::mesh::Mesh;
use proptest::prelude::*;

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

fn euler(mesh: &Mesh) -> i64 {
	let mut edges: HashSet<(u32, u32)> = HashSet::new();
	for t in mesh.indices.chunks_exact(3) {
		for &(a, b) in &[(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
			edges.insert(if a < b { (a, b) } else { (b, a) });
		}
	}
	mesh.positions.len() as i64 - edges.len() as i64 + (mesh.indices.len() / 3) as i64
}

proptest! {
	#![proptest_config(ProptestConfig::with_cases(40))]

	#[test]
	fn random_single_face_dimple_is_a_valid_solid(
		cx in -0.3f64..0.3, cy in -0.3f64..0.3, r in 0.4f64..0.55,
	) {
		// Centre at z=0.7: the ball pokes only the top face (sides ≥0.7 away > r),
		// with a small-circle seam (not a great circle).
		let cube = grid_cube(1.0, 10);
		let result = subtract_sphere(&cube, DVec3::new(cx, cy, 0.7), r);

		prop_assert!(result.is_watertight(), "dimple must be watertight (c=({cx},{cy}), r={r})");
		prop_assert_eq!(euler(&result), 2, "a simple dimple is genus 0");
		let v = result.signed_volume();
		let ball = 4.0 / 3.0 * PI * r.powi(3);
		prop_assert!(v > 8.0 - ball - 0.1 && v < 8.01, "volume {v} outside (8-ball, 8)");
	}

	#[test]
	fn random_through_drill_is_a_genus_one_solid(
		cx in -0.3f64..0.3, cy in -0.3f64..0.3, r in 0.2f64..0.5,
	) {
		// Axis-aligned bore that clears the side faces (|c|+r < 1): drills cleanly
		// through the top and bottom faces only, giving a torus-topology solid.
		let cube = grid_cube(1.0, 12);
		let result = drill_cylinder(&cube, DVec3::new(cx, cy, 0.0), DVec3::Z, r);

		prop_assert!(result.is_watertight(), "bore must be watertight (c=({cx},{cy}), r={r})");
		prop_assert_eq!(euler(&result), 0, "a through-hole is genus 1 (Euler 0)");
		let v = result.signed_volume();
		let cyl = PI * r * r * 2.0;
		prop_assert!(v > 8.0 - cyl - 0.1 && v < 8.01, "volume {v} outside (8-cyl, 8)");
	}
}
