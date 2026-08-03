// Copyright (c) LMCAD. Licensed under the MIT License.

//! Capstone: the major subsystems compose into one end-to-end pipeline —
//! curved booleans → mesh∩mesh boolean → exact analytic section → f64 mesh+export.

use std::collections::HashSet;

use kernel_brep::math::{DVec3, Vec3};
use kernel_brep::{cuboid, drill_cylinder, mesh_intersection, subtract_sphere, tessellate_default, Curve, Surface};
use kernel_core::mesh::Mesh;
use kernel_core::{surface_nets_f64, MeshF64};

/// A cube `[-half, half]³` subdivided `n × n` per face (consistently wound).
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

#[test]
fn capstone_full_pipeline_composes() {
	// 1. Curved booleans chained: drill a through-hole, then dimple a face.
	let cube = grid_cube(1.0, 14);
	let drilled = drill_cylinder(&cube, DVec3::ZERO, DVec3::Z, 0.3);
	assert!(drilled.is_watertight(), "drilled part must be watertight");
	assert_eq!(euler(&drilled), 0, "a through-hole is genus 1 (Euler 0)");

	let part = subtract_sphere(&drilled, DVec3::new(0.6, 0.0, 1.0), 0.35);
	assert!(part.is_watertight(), "dimpled part must stay watertight");
	assert_eq!(euler(&part), 0, "the dimple does not change genus");

	// 2. The curved-boolean output feeds the mesh∩mesh boolean: keep the lower slab.
	let slab = tessellate_default(&cuboid(DVec3::new(-2.0, -2.0, -2.0), DVec3::new(2.0, 2.0, 0.0)));
	let lower = mesh_intersection(&part, &slab);
	assert!(lower.triangle_count() > 50, "intersection with the slab should be a real solid");
	let vol = lower.signed_volume().abs();
	assert!(vol > 3.0 && vol < 4.0, "lower slab volume {vol} (≈ half the bored cube, ~3.8)");

	// 3. Exact analytic section: the bore cylinder cut by z = 0.5 is a circle r = 0.3.
	let bore = Surface::Cylinder { origin: DVec3::ZERO, axis: DVec3::Z, radius: 0.3 };
	let section = bore.plane_section(DVec3::Z * 0.5, DVec3::Z);
	let Curve::Circle { radius, center, .. } = section[0] else { panic!("expected a circle section") };
	assert!((radius - 0.3).abs() < 1e-12 && (center - DVec3::Z * 0.5).length() < 1e-12);

	// 4. f64 meshing + full-precision OBJ export of an analytic surface.
	let c = DVec3::new(1.0e6, 0.0, 0.0);
	let f64_mesh: MeshF64 = surface_nets_f64(move |p| (p - c).length() - 1.0, c - DVec3::splat(1.5), c + DVec3::splat(1.5), 0.1);
	let obj = f64_mesh.to_obj();
	assert!(obj.lines().filter(|l| l.starts_with("v ")).count() == f64_mesh.positions.len());
	assert!(obj.contains("\nf "), "OBJ should carry faces");
}
