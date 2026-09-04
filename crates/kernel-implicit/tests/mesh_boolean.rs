// Copyright (c) LMCAD. Licensed under the MIT License.

//! Watertight implicit-path booleans between two closed triangle meshes.

use std::f64::consts::PI;

use kernel_core::math::Vec3;
use kernel_core::{surface_nets_sdf_f64, Mesh, Sdf};
use kernel_implicit::{mesh_boolean_implicit, BoolOp, Sphere};

/// A clean, watertight sphere mesh from the analytic field — a fine curved input
/// of exactly the kind that makes the exact mesh-arrangement boolean produce a
/// non-watertight result.
fn sphere_mesh(center: Vec3, radius: f32, voxel: f64) -> Mesh {
	let s = Sphere::new(center, radius);
	surface_nets_sdf_f64(&s, s.bounds(), voxel).to_mesh()
}

/// A hand-built axis-aligned cube (8 verts, 12 outward triangles) — a
/// non-spherical input with flat faces and sharp edges.
fn box_mesh(min: Vec3, max: Vec3) -> Mesh {
	let p = vec![
		Vec3::new(min.x, min.y, min.z),
		Vec3::new(max.x, min.y, min.z),
		Vec3::new(max.x, max.y, min.z),
		Vec3::new(min.x, max.y, min.z),
		Vec3::new(min.x, min.y, max.z),
		Vec3::new(max.x, min.y, max.z),
		Vec3::new(max.x, max.y, max.z),
		Vec3::new(min.x, max.y, max.z),
	];
	let indices = vec![
		1, 2, 6, 1, 6, 5, // +X
		0, 4, 7, 0, 7, 3, // -X
		3, 7, 6, 3, 6, 2, // +Y
		0, 1, 5, 0, 5, 4, // -Y
		4, 5, 6, 4, 6, 7, // +Z
		0, 3, 2, 0, 2, 1, // -Z
	];
	let mut m = Mesh { positions: p, indices, normals: vec![] };
	m.compute_normals();
	m
}

#[test]
fn implicit_boolean_stays_watertight_on_curved_inputs() {
	// Two r=8 spheres, centres 8 apart — the same configuration whose exact
	// arrangement boolean is non-watertight (349 bad directed edges). The implicit
	// path must return a closed 2-manifold for all three operators, with volumes
	// matching the analytic lens and inclusion–exclusion to voxel accuracy.
	let a = sphere_mesh(Vec3::ZERO, 8.0, 0.7);
	let b = sphere_mesh(Vec3::X * 8.0, 8.0, 0.7);
	assert!(a.is_watertight() && b.is_watertight(), "input spheres should be clean");

	// An explicit voxel keeps the winding-number sampling affordable; the inputs are
	// still smooth curved meshes (~thousands of triangles), not coarse facets.
	let union = mesh_boolean_implicit(&a, &b, BoolOp::Union, 0.7);
	let inter = mesh_boolean_implicit(&a, &b, BoolOp::Intersection, 0.7);
	let diff = mesh_boolean_implicit(&a, &b, BoolOp::Difference, 0.7);

	// The headline guarantee the arrangement path loses: every result is closed.
	assert_eq!(
		(union.is_watertight(), inter.is_watertight(), diff.is_watertight()),
		(true, true, true),
		"implicit-path booleans must be watertight"
	);

	let one = 4.0 / 3.0 * PI * 8.0_f64.powi(3); // single sphere ≈ 2144.66
	let lens = PI * 40.0 * 64.0 / 12.0; // overlap lens ≈ 670.2
	let vol = |m: &Mesh| m.signed_volume().abs();
	let rel = |got: f64, want: f64| (got - want).abs() / want;
	assert!(rel(vol(&inter), lens) < 0.1, "intersection {} vs lens {lens}", vol(&inter));
	assert!(rel(vol(&union), 2.0 * one - lens) < 0.1, "union {} vs {}", vol(&union), 2.0 * one - lens);
	assert!(rel(vol(&diff), one - lens) < 0.1, "difference {} vs {}", vol(&diff), one - lens);
}

#[test]
fn implicit_boolean_disjoint_operands_are_exact() {
	// Bounding boxes far apart (gap ≫ feature): meshing over the empty gap would
	// under-resolve the inputs into garbage, so the boolean must be evaluated
	// exactly — A∪B concatenates, A−B = A, A∩B = ∅. Two r=2 spheres 200 apart.
	let a = sphere_mesh(Vec3::ZERO, 2.0, 0.15);
	let b = sphere_mesh(Vec3::X * 200.0, 2.0, 0.15);
	let one = 4.0 / 3.0 * PI * 8.0; // r=2 sphere ≈ 33.5
	let union = mesh_boolean_implicit(&a, &b, BoolOp::Union, 0.0);
	let diff = mesh_boolean_implicit(&a, &b, BoolOp::Difference, 0.0);
	let inter = mesh_boolean_implicit(&a, &b, BoolOp::Intersection, 0.0);
	assert_eq!(
		(union.is_watertight(), diff.is_watertight(), inter.triangle_count()),
		(true, true, 0),
		"disjoint results: union/diff watertight, intersection empty"
	);
	let rel = |m: &Mesh, want: f64| (m.signed_volume().abs() - want).abs() / want;
	assert!(rel(&union, 2.0 * one) < 0.05, "disjoint union vol {}", union.signed_volume().abs());
	assert!(rel(&diff, one) < 0.05, "disjoint difference vol {}", diff.signed_volume().abs());
}

#[test]
fn implicit_difference_of_contained_solid_is_a_hollow_shell() {
	// B fully inside A: A − B is a closed shell with an internal void — two nested
	// watertight surfaces. Volume = vol(A) − vol(B). r=8 minus a concentric r=3.
	let a = sphere_mesh(Vec3::ZERO, 8.0, 0.7);
	let b = sphere_mesh(Vec3::ZERO, 3.0, 0.4);
	let shell = mesh_boolean_implicit(&a, &b, BoolOp::Difference, 0.6);
	let expected = 4.0 / 3.0 * PI * (512.0 - 27.0);
	assert!(shell.is_watertight(), "hollow shell must be watertight (two nested surfaces)");
	let rel = (shell.signed_volume().abs() - expected).abs() / expected;
	assert!(rel < 0.05, "shell vol {} vs {expected}", shell.signed_volume().abs());
}

#[test]
fn implicit_boolean_on_boxes_is_exact_and_watertight() {
	// Non-spherical inputs with sharp axis-aligned faces stress the dual-contour
	// mesher (faces land on grid planes). A=[-1,1]³, B=[0,2]³ overlap in [0,1]³:
	// inclusion–exclusion gives ∪=15, ∩=1, −=7.
	let a = box_mesh(Vec3::splat(-1.0), Vec3::splat(1.0));
	let b = box_mesh(Vec3::ZERO, Vec3::splat(2.0));
	let u = mesh_boolean_implicit(&a, &b, BoolOp::Union, 0.08);
	let i = mesh_boolean_implicit(&a, &b, BoolOp::Intersection, 0.08);
	let d = mesh_boolean_implicit(&a, &b, BoolOp::Difference, 0.08);
	assert_eq!((u.is_watertight(), i.is_watertight(), d.is_watertight()), (true, true, true), "box booleans must be watertight");
	let v = |m: &Mesh| m.signed_volume().abs();
	assert!(
		(v(&u) - 15.0).abs() < 0.3 && (v(&i) - 1.0).abs() < 0.3 && (v(&d) - 7.0).abs() < 0.3,
		"box booleans ∪={} ∩={} −={} (expect 15/1/7)",
		v(&u),
		v(&i),
		v(&d)
	);
}

#[test]
fn implicit_boolean_normalizes_inward_winding() {
	// MeshSdf signs by winding number, so an inside-out operand would silently
	// vanish unless normalized. A reversed-winding sphere must give the same lens
	// as its outward twin. Two r=2 spheres, centres 2 apart.
	let a = sphere_mesh(Vec3::ZERO, 2.0, 0.2);
	let b = sphere_mesh(Vec3::X * 2.0, 2.0, 0.2);
	let mut a_inward = a.clone();
	for t in a_inward.indices.chunks_exact_mut(3) {
		t.swap(1, 2); // reverse every triangle → inside-out
	}
	let normal = mesh_boolean_implicit(&a, &b, BoolOp::Intersection, 0.2);
	let inward = mesh_boolean_implicit(&a_inward, &b, BoolOp::Intersection, 0.2);
	assert!(inward.is_watertight() && inward.triangle_count() > 50, "inward operand must not vanish");
	let rel = (inward.signed_volume().abs() - normal.signed_volume().abs()).abs() / normal.signed_volume().abs();
	assert!(rel < 0.02, "inward lens {} vs normal {}", inward.signed_volume().abs(), normal.signed_volume().abs());
}

#[test]
fn implicit_boolean_follows_empty_operand_algebra() {
	// A∪∅ = A, A−∅ = A (returned verbatim, so the triangle count is unchanged),
	// A∩∅ = ∅, ∅∪A = A, ∅−A = ∅, ∅∩A = ∅.
	let a = sphere_mesh(Vec3::ZERO, 5.0, 0.4);
	let n = a.triangle_count();
	let empty = Mesh::new();
	let tc = |op, x: &Mesh, y: &Mesh| mesh_boolean_implicit(x, y, op, 0.0).triangle_count();
	let got = (
		tc(BoolOp::Union, &a, &empty),
		tc(BoolOp::Difference, &a, &empty),
		tc(BoolOp::Intersection, &a, &empty),
		tc(BoolOp::Union, &empty, &a),
		tc(BoolOp::Difference, &empty, &a),
		tc(BoolOp::Intersection, &empty, &a),
	);
	assert_eq!(got, (n, n, 0, n, 0, 0));
}
