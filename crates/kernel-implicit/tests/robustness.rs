// Copyright (c) LMCAD. Licensed under the MIT License.

//! Robustness tests: degenerate and hostile inputs must never panic, and must
//! never produce NaN/inf distances, gradients, or mesh vertices. Geometric
//! correctness is not asserted for degenerate shapes — only that the kernel
//! stays well-defined.

use kernel_implicit::{surface_nets, Capsule, Cone, Cuboid, Cylinder, Node, Resolution, Sdf, Sphere, Torus, Vec3};
use proptest::prelude::*;

const PROBES: [Vec3; 5] = [
	Vec3::new(0.0, 0.0, 0.0),
	Vec3::new(3.7, -2.1, 5.0),
	Vec3::new(-8.0, 8.0, -8.0),
	Vec3::new(0.01, 0.0, 0.0),
	Vec3::new(50.0, 50.0, 50.0),
];

fn finite_everywhere(sdf: &dyn Sdf, what: &str) {
	for &p in &PROBES {
		assert!(sdf.distance(p).is_finite(), "{what}: non-finite distance at {p:?}");
		assert!(sdf.gradient(p).is_finite(), "{what}: non-finite gradient at {p:?}");
	}
}

#[test]
fn explicit_degenerate_primitives_are_finite() {
	// Zero / collapsed primitives that a naive formula would turn into NaN
	// (0/0, sqrt of negative, divide-by-zero axis length).
	let z = Vec3::ZERO;
	finite_everywhere(&Sphere::new(z, 0.0), "zero-radius sphere");
	finite_everywhere(&Sphere::new(z, -2.0), "negative-radius sphere");
	finite_everywhere(&Cuboid::new(z, Vec3::ZERO), "zero-extent cuboid");
	finite_everywhere(&Cylinder::new(z, z, 3.0), "zero-length cylinder");
	finite_everywhere(&Cylinder::new(z, z, 0.0), "point cylinder");
	finite_everywhere(&Cone::new(z, z, 3.0, 1.0), "zero-length cone");
	finite_everywhere(&Cone::new(z, Vec3::Z * 5.0, 0.0, 0.0), "zero-radius cone");
	finite_everywhere(&Capsule::new(z, z, 2.0), "zero-length capsule");
	finite_everywhere(&Capsule::new(z, z, 0.0), "point capsule");
	finite_everywhere(&Torus::new(z, Vec3::ZERO, 0.0, 0.0), "collapsed torus (zero axis)");

	// Meshing a collapsed primitive must not panic and must yield finite vertices.
	let node = Node::primitive(Sphere::new(z, 0.0));
	let mesh = surface_nets(&node, node.bounds().pad(1.0), Resolution::CellsOnLongestAxis(16));
	assert!(mesh.positions.iter().all(|p| p.is_finite()), "degenerate mesh has non-finite vertices");
}

fn vv(t: (f32, f32, f32)) -> Vec3 {
	Vec3::new(t.0, t.1, t.2)
}

proptest! {
	#![proptest_config(ProptestConfig::with_cases(96))]

	/// Random primitives with hostile parameters (near-zero radii, near-coincident
	/// endpoints) must keep distance and gradient finite everywhere.
	#[test]
	fn primitive_sdf_finite_under_hostile_params(
		k in 0u8..5,
		center in (-6.0f32..6.0, -6.0f32..6.0, -6.0f32..6.0),
		off in (-1.0f32..1.0, -1.0f32..1.0, -1.0f32..1.0), // can be ≈ 0 → coincident
		radius in 0.0f32..5.0, // can be ≈ 0
	) {
		let c = vv(center);
		let e = c + vv(off);
		let prim: Box<dyn Sdf> = match k % 5 {
			0 => Box::new(Sphere::new(c, radius)),
			1 => Box::new(Cuboid::new(c, vv(off).abs())),
			2 => Box::new(Cylinder::new(c, e, radius)),
			3 => Box::new(Capsule::new(c, e, radius)),
			_ => Box::new(Cone::new(c, e, radius, radius * 0.5)),
		};
		for &p in &PROBES {
			prop_assert!(prim.distance(p).is_finite(), "non-finite distance (k={k})");
			prop_assert!(prim.gradient(p).is_finite(), "non-finite gradient (k={k})");
		}
	}

	/// Random CSG over hostile primitives, meshed, must not panic and must yield
	/// only finite vertices.
	#[test]
	fn hostile_csg_meshes_without_nan(
		ka in 0u8..5, ca in (-5.0f32..5.0, -5.0f32..5.0, -5.0f32..5.0), ra in 0.0f32..5.0,
		kb in 0u8..5, cb in (-5.0f32..5.0, -5.0f32..5.0, -5.0f32..5.0), rb in 0.0f32..5.0,
		op in 0u8..3,
	) {
		let mk = |k: u8, c: Vec3, r: f32| -> Node {
			match k % 5 {
				0 => Node::primitive(Sphere::new(c, r)),
				1 => Node::primitive(Cuboid::new(c, Vec3::splat(r))),
				2 => Node::primitive(Cylinder::new(c, c + Vec3::Y * r, r)),
				3 => Node::primitive(Capsule::new(c, c + Vec3::X * r, r)),
				_ => Node::primitive(Cone::new(c, c + Vec3::Z * (r + 0.1), r, 0.0)),
			}
		};
		let a = mk(ka, vv(ca), ra);
		let b = mk(kb, vv(cb), rb);
		let tree = match op % 3 {
			0 => a.union(b),
			1 => a.difference(b),
			_ => a.intersection(b),
		};
		let bounds = tree.bounds();
		prop_assume!(bounds.is_valid() && bounds.size().min_element() > 1e-3 && bounds.size().max_element() < 1e3);
		let mesh = surface_nets(&tree, bounds, Resolution::CellsOnLongestAxis(24));
		prop_assert!(mesh.positions.iter().all(|p| p.is_finite()), "hostile CSG produced a non-finite vertex");
		prop_assert!(mesh.normals.iter().all(|n| n.is_finite()), "hostile CSG produced a non-finite normal");
	}
}
