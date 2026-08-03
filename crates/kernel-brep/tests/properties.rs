// Copyright (c) LMCAD. Licensed under the MIT License.

//! Property-based (fuzz) tests for the B-rep constructors — robustness coverage
//! for `extrude` and `revolve` under random (but valid) profiles. Every result
//! must be a closed, oriented manifold satisfying Euler–Poincaré, tessellate to a
//! watertight mesh, and report the analytically-known volume.

use kernel_brep::math::DVec2;
use kernel_brep::{extrude, revolve, tessellate_default, validate, volume};
use proptest::prelude::*;

/// Shoelace area of a simple polygon.
fn shoelace(profile: &[DVec2]) -> f64 {
	let n = profile.len();
	let mut a = 0.0;
	for i in 0..n {
		let j = (i + 1) % n;
		a += profile[i].x * profile[j].y - profile[j].x * profile[i].y;
	}
	a.abs() * 0.5
}

proptest! {
	#![proptest_config(ProptestConfig::with_cases(64))]

	/// A star-convex polygon (one vertex per equal angular slice, random radius)
	/// is always simple (non-self-intersecting), so extruding it yields a valid
	/// prism. Planar faces ⇒ the volume is exactly shoelace-area × height.
	#[test]
	fn random_star_convex_extrusion_is_valid(
		radii in prop::collection::vec(3.0f64..8.0, 3..9usize),
		height in 2.0f64..10.0,
	) {
		let n = radii.len();
		let profile: Vec<DVec2> = radii
			.iter()
			.enumerate()
			.map(|(i, &r)| {
				let a = std::f64::consts::TAU * i as f64 / n as f64;
				DVec2::new(r * a.cos(), r * a.sin())
			})
			.collect();

		let solid = extrude(&profile, height);
		let v = validate(&solid);
		prop_assert!(v.closed && v.manifold, "extrusion is not a closed manifold");
		prop_assert_eq!(v.euler_characteristic, 2, "prism χ should be 2");
		prop_assert_eq!(v.genus, 0, "prism genus should be 0");

		let mesh = tessellate_default(&solid);
		prop_assert!(mesh.is_watertight(), "extrusion tessellation is not watertight");

		let expected = shoelace(&profile) * height;
		let got = volume(&solid);
		prop_assert!((got - expected).abs() / expected < 1e-6, "extrude vol {got} vs {expected}");
	}

	/// A rectangular cross-section ring (not touching the axis) revolved a full
	/// turn is a genus-1 torus. Pappus: volume = section-area × (2π × centroid r).
	#[test]
	fn random_revolved_ring_is_genus1_torus(
		r0 in 4.0f64..10.0,
		dr in 1.0f64..5.0,
		z0 in -4.0f64..4.0,
		dz in 1.0f64..5.0,
		segments in 16usize..64,
	) {
		let profile = [
			DVec2::new(r0, z0),
			DVec2::new(r0 + dr, z0),
			DVec2::new(r0 + dr, z0 + dz),
			DVec2::new(r0, z0 + dz),
		];
		let solid = revolve(&profile, segments);
		let v = validate(&solid);
		prop_assert!(v.closed && v.manifold, "revolved ring is not a closed manifold");
		prop_assert_eq!(v.euler_characteristic, 0, "torus χ should be 0");
		prop_assert_eq!(v.genus, 1, "revolved ring should be genus 1");

		let mesh = tessellate_default(&solid);
		prop_assert!(mesh.is_watertight(), "torus tessellation is not watertight");

		let expected = (dr * dz) * (2.0 * std::f64::consts::PI * (r0 + dr * 0.5));
		let got = volume(&solid);
		// Faceting under-fills slightly; tolerance scales with the facet count.
		let tol = 0.5 / segments as f64 + 0.01;
		prop_assert!((got - expected).abs() / expected < tol, "torus vol {got} vs {expected} (tol {tol})");
	}

	/// R1 regression, generalised: a random CONCAVE staircase ring section — 2–4 steps,
	/// so several horizontal segments at different radii and a concave corner per step —
	/// revolves to a valid genus-1 solid. (The old centroid-based band orientation broke
	/// exactly this family.) The volume oracle is the honestly-faceted n-gon closed form:
	/// every revolve band quad is planar with offset × area = ½·sin(2π/N)·(rᵢ+rⱼ)(rᵢzⱼ−rⱼzᵢ)
	/// per sector, so the polyhedron volume is exactly N·sin(2π/N)/6 · Σ(rᵢ+rⱼ)(rᵢzⱼ−rⱼzᵢ).
	#[test]
	fn random_concave_staircase_revolve_is_valid_genus1(
		r_in in 2.0f64..6.0,
		steps in prop::collection::vec((0.5f64..4.0, 0.5f64..4.0), 2..5usize),
		segments in 8usize..64,
	) {
		// Staircase: widest at the bottom, stepping inward as z rises (an L for 2 steps).
		// Radii r_in < r_1 < … < r_K, heights 0 < z_1 < … < z_K, both cumulative.
		let k = steps.len();
		let radii: Vec<f64> = std::iter::once(r_in)
			.chain(steps.iter().scan(r_in, |r, &(dr, _)| { *r += dr; Some(*r) }))
			.collect();
		let heights: Vec<f64> = std::iter::once(0.0)
			.chain(steps.iter().scan(0.0, |z, &(_, dz)| { *z += dz; Some(*z) }))
			.collect();
		let mut profile = vec![DVec2::new(radii[0], 0.0), DVec2::new(radii[k], 0.0)];
		for s in 0..k {
			profile.push(DVec2::new(radii[k - s], heights[s + 1]));
			profile.push(DVec2::new(radii[k - s - 1], heights[s + 1]));
		}

		let solid = revolve(&profile, segments);
		let v = validate(&solid);
		prop_assert!(v.closed && v.manifold, "staircase revolve is not a closed manifold: {v:?}");
		prop_assert_eq!(v.genus, 1, "staircase ring should be genus 1");
		prop_assert!(tessellate_default(&solid).is_watertight(), "staircase revolve mesh is not watertight");

		let m: f64 = (0..profile.len())
			.map(|i| {
				let (p, q) = (profile[i], profile[(i + 1) % profile.len()]);
				(p.x + q.x) * (p.x * q.y - q.x * p.y)
			})
			.sum();
		let expected = segments as f64 * (std::f64::consts::TAU / segments as f64).sin() * m / 6.0;
		let got = volume(&solid);
		prop_assert!((got - expected).abs() / expected < 1e-6, "staircase faceted vol {got} vs {expected}");
	}
}
