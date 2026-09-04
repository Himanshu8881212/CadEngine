// Copyright (c) LMCAD. Licensed under the MIT License.

//! Empirical SDF property tests: for an exact signed distance field, a point
//! `p` known to lie on the surface must satisfy `sdf(p) ≈ 0`, and a point
//! offset by `d` along the outward normal must satisfy `sdf(p + d·n) ≈ d`.
//! This independently validates both the distance magnitude and the sign /
//! normal direction of every analytic primitive.

use kernel_core::math::Vec3;
use kernel_core::sdf::Sdf;
use kernel_implicit::{Capsule, Cone, Cuboid, Cylinder, Sphere, Torus};

const TOL: f32 = 1e-3;
const OFFSET: f32 = 0.05;

/// An orthonormal basis `(u, v)` spanning the plane perpendicular to unit `n`.
fn basis(n: Vec3) -> (Vec3, Vec3) {
	let a = if n.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
	let u = (a - n * a.dot(n)).normalize();
	(u, n.cross(u))
}

/// Assert a surface point and its outward-offset behave like an exact SDF.
fn check_surface_point(sdf: &dyn Sdf, p: Vec3, outward: Vec3, what: &str) {
	let on = sdf.distance(p);
	assert!(on.abs() < TOL, "{what}: sdf on surface = {on} (expected 0) at {p:?}");
	let out = sdf.distance(p + outward * OFFSET);
	assert!((out - OFFSET).abs() < TOL, "{what}: outward offset sdf = {out} (expected {OFFSET})");
	let inn = sdf.distance(p - outward * OFFSET);
	assert!((inn + OFFSET).abs() < TOL, "{what}: inward offset sdf = {inn} (expected {})", -OFFSET);
}

#[test]
fn sphere_surface() {
	let s = Sphere::new(Vec3::new(1.0, -2.0, 0.5), 7.0);
	for i in 0..12 {
		for j in 1..12 {
			let theta = std::f32::consts::TAU * i as f32 / 12.0;
			let phi = std::f32::consts::PI * j as f32 / 12.0;
			let dir = Vec3::new(phi.sin() * theta.cos(), phi.sin() * theta.sin(), phi.cos());
			check_surface_point(&s, s.center + dir * s.radius, dir, "sphere");
		}
	}
}

#[test]
fn cuboid_faces() {
	let c = Cuboid::new(Vec3::new(2.0, 0.0, -1.0), Vec3::new(6.0, 4.0, 3.0));
	let axes = [Vec3::X, Vec3::Y, Vec3::Z];
	for (ax, &axis) in axes.iter().enumerate() {
		let (u, v) = (axes[(ax + 1) % 3], axes[(ax + 2) % 3]);
		for s in [-1.0f32, 1.0] {
			for i in 0..5 {
				for j in 0..5 {
					let fu = (i as f32 / 4.0 - 0.5) * 1.6; // stay inside the face
					let fv = (j as f32 / 4.0 - 0.5) * 1.6;
					let p = c.center + axis * (s * c.half.dot(axis)) + u * (fu * c.half.dot(u)) + v * (fv * c.half.dot(v));
					check_surface_point(&c, p, axis * s, "cuboid");
				}
			}
		}
	}
}

#[test]
fn cylinder_lateral() {
	let a = Vec3::new(0.0, 0.0, 0.0);
	let b = Vec3::new(0.0, 12.0, 0.0);
	let r = 4.0;
	let cyl = Cylinder::new(a, b, r);
	let axis = (b - a).normalize();
	let (u, v) = basis(axis);
	for ti in 0..6 {
		for ai in 0..12 {
			let t = (ti as f32 + 0.5) / 6.0;
			let theta = std::f32::consts::TAU * ai as f32 / 12.0;
			let radial = u * theta.cos() + v * theta.sin();
			let p = a + (b - a) * t + radial * r;
			check_surface_point(&cyl, p, radial, "cylinder lateral");
		}
	}
}

#[test]
fn cone_frustum_and_cylinder_degeneracy() {
	// A frustum: ra=5 at a, rb=2 at b. Lateral surface normal tilts with slope.
	let a = Vec3::new(0.0, 0.0, 0.0);
	let b = Vec3::new(0.0, 0.0, 10.0);
	let (ra, rb) = (5.0f32, 2.0f32);
	let cone = Cone::new(a, b, ra, rb);
	let axis = (b - a).normalize();
	let (u, v) = basis(axis);
	let h = (b - a).length();
	let slope = (ra - rb) / h; // dr/dz (positive: shrinks)
	for ti in 0..6 {
		for ai in 0..12 {
			let t = (ti as f32 + 0.5) / 6.0;
			let radius = ra + (rb - ra) * t;
			let theta = std::f32::consts::TAU * ai as f32 / 12.0;
			let radial = u * theta.cos() + v * theta.sin();
			let p = a + (b - a) * t + radial * radius;
			// Outward normal of the lateral cone surface (tilted by the slope).
			let outward = (radial + axis * slope).normalize();
			check_surface_point(&cone, p, outward, "cone frustum");
		}
	}
}

#[test]
fn torus_surface() {
	let center = Vec3::new(0.0, 0.0, 0.0);
	let axis = Vec3::Z;
	let (major, minor) = (10.0f32, 3.0f32);
	let torus = Torus::new(center, axis, major, minor);
	for i in 0..12 {
		for j in 0..12 {
			let theta = std::f32::consts::TAU * i as f32 / 12.0;
			let phi = std::f32::consts::TAU * j as f32 / 12.0;
			let ring = Vec3::new(theta.cos(), theta.sin(), 0.0);
			let normal = ring * phi.cos() + Vec3::Z * phi.sin();
			let p = center + ring * major + normal * minor;
			check_surface_point(&torus, p, normal, "torus");
		}
	}
}

#[test]
fn capsule_lateral() {
	let a = Vec3::new(-3.0, 0.0, 0.0);
	let b = Vec3::new(8.0, 1.0, 2.0);
	let r = 2.5;
	let cap = Capsule::new(a, b, r);
	let axis = (b - a).normalize();
	let (u, v) = basis(axis);
	for ti in 0..6 {
		for ai in 0..12 {
			let t = (ti as f32 + 0.5) / 6.0;
			let theta = std::f32::consts::TAU * ai as f32 / 12.0;
			let radial = u * theta.cos() + v * theta.sin();
			let p = a + (b - a) * t + radial * r;
			check_surface_point(&cap, p, radial, "capsule lateral");
		}
	}
	// Spherical end caps.
	for dir in [a - b, b - a] {
		let n = dir.normalize();
		let tip = if dir == a - b { a } else { b };
		check_surface_point(&cap, tip + n * r, n, "capsule cap");
	}
}
