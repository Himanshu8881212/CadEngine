// Copyright (c) LMCAD. Licensed under the MIT License.

//! Degeneracy validation for the native Voronoi strut generator
//! (`voronoi_struts`) behind `VoronoiLattice`.
//!
//! The generator's honesty contract (see `kernel_implicit::voronoi`) promises
//! that degenerate / near-degenerate seed sets are handled GRACEFULLY: flat
//! (sliver/coplanar) tets are marked inert, so a duplicate / collinear /
//! coplanar / cospherical seed cloud yields a valid-or-empty-but-CLEAN edge
//! graph — never a panic, never a NaN/∞ strut, never an out-of-box strut — and
//! the same seeds always give the same output. This test proves it.

use kernel_core::math::Vec3;
use kernel_implicit::lattice::VoronoiLattice;
use kernel_implicit::voronoi::voronoi_struts;

/// Every emitted strut is finite and inside the clip box (small eps slack).
fn clean(struts: &[(Vec3, Vec3)], lo: Vec3, hi: Vec3) -> bool {
	let eps = Vec3::splat(1e-3);
	struts.iter().all(|&(a, b)| {
		a.is_finite()
			&& b.is_finite()
			&& a.cmpge(lo - eps).all()
			&& a.cmple(hi + eps).all()
			&& b.cmpge(lo - eps).all()
			&& b.cmple(hi + eps).all()
	})
}

/// Bit-identical rebuild from the same seeds (determinism).
fn identical(a: &[(Vec3, Vec3)], b: &[(Vec3, Vec3)]) -> bool {
	a.len() == b.len() && a.iter().zip(b).all(|(x, y)| x == y)
}

/// Deterministic LCG jitter helper (no rand dependency).
fn lcg(seed: &mut u64) -> f32 {
	*seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
	((*seed >> 33) as f32) / ((1u64 << 31) as f32)
}

#[test]
fn degenerate_seed_sets_are_graceful_clean_and_deterministic() {
	let (lo, hi) = (Vec3::splat(-12.0), Vec3::splat(12.0));

	// 5) Cospherical cluster: seeds EXACTLY on a radius-8 sphere (icosahedron
	// vertices). Every 4-subset is cospherical → ambiguous in-sphere tests, which
	// the eps slack resolves consistently. Cleanliness + determinism only.
	let phi = (1.0 + 5.0_f32.sqrt()) / 2.0;
	let ico: Vec<Vec3> = [
		(0.0, 1.0, phi),
		(0.0, 1.0, -phi),
		(0.0, -1.0, phi),
		(0.0, -1.0, -phi),
		(1.0, phi, 0.0),
		(1.0, -phi, 0.0),
		(-1.0, phi, 0.0),
		(-1.0, -phi, 0.0),
		(phi, 0.0, 1.0),
		(phi, 0.0, -1.0),
		(-phi, 0.0, 1.0),
		(-phi, 0.0, -1.0),
	]
	.iter()
	.map(|&(x, y, z)| Vec3::new(x, y, z).normalize() * 8.0)
	.collect();

	// 6) Cocircular ring + a couple of off-plane apex points: 8 seeds on a circle
	// in z = 0 (all cocircular → cospherical with any off-plane apex) plus 2 apex
	// points. Near-degenerate but genuinely 3-D → clean, deterministic.
	let mut ring: Vec<Vec3> = (0..8)
		.map(|i| {
			let a = std::f32::consts::TAU * i as f32 / 8.0;
			Vec3::new(7.0 * a.cos(), 7.0 * a.sin(), 0.0)
		})
		.collect();
	ring.push(Vec3::new(0.0, 0.0, 5.0));
	ring.push(Vec3::new(0.0, 0.0, -5.0));

	// A menu of pathological seed clouds. Each must NOT panic, must produce a
	// clean graph (finite + in-box), and must rebuild bit-identically. `Some(true)`
	// marks a genuinely lower-dimensional cloud (no tetrahedra → empty foam);
	// `None` asserts only cleanliness + determinism.
	let cases: Vec<(&str, Vec<Vec3>, Option<bool>)> = vec![
		// 1) Exact duplicate seeds through a real 3-D cloud: the doubled vertices
		// form flat tets that are marked inert; the rest still tetrahedralizes.
		(
			"duplicates-in-cloud",
			vec![
				Vec3::new(-6.0, -6.0, -6.0),
				Vec3::new(6.0, -6.0, -6.0),
				Vec3::new(-6.0, 6.0, -6.0),
				Vec3::new(6.0, 6.0, 6.0),
				Vec3::new(0.0, 0.0, 0.0),
				Vec3::new(0.0, 0.0, 0.0), // exact duplicate
				Vec3::new(3.0, -2.0, 1.0),
				Vec3::new(3.0, -2.0, 1.0), // another exact duplicate
				Vec3::new(-3.0, 4.0, -2.0),
				Vec3::new(2.0, 2.0, -5.0),
			],
			None,
		),
		// 2) All seeds identical (a full pile-up): no volume anywhere → empty foam.
		("all-coincident", vec![Vec3::new(1.0, 2.0, 3.0); 8], Some(true)),
		// 3) Collinear run: every seed on the x-axis → no tetrahedron → empty.
		("collinear", (0..9).map(|i| Vec3::new(-8.0 + 2.0 * i as f32, 0.0, 0.0)).collect(), Some(true)),
		// 4) Coplanar grid: all seeds on z = 0 → no tetrahedron → empty.
		(
			"coplanar-grid",
			(0..3).flat_map(|i| (0..3).map(move |j| Vec3::new(-5.0 + 5.0 * i as f32, -5.0 + 5.0 * j as f32, 0.0))).collect(),
			Some(true),
		),
		("cospherical-icosahedron", ico, None),
		("cocircular-ring+apexes", ring, None),
	];

	let mut failures: Vec<String> = Vec::new();
	for (label, seeds, expect_empty) in &cases {
		// No panic (the call itself), run twice for determinism.
		let a = voronoi_struts(seeds, lo, hi);
		let b = voronoi_struts(seeds, lo, hi);
		if !clean(&a, lo, hi) {
			failures.push(format!("{label}: produced a non-finite or out-of-box strut ({} struts)", a.len()));
		}
		if !identical(&a, &b) {
			failures.push(format!("{label}: nondeterministic ({} vs {} struts)", a.len(), b.len()));
		}
		if let Some(true) = expect_empty {
			if !a.is_empty() {
				failures.push(format!("{label}: a lower-dimensional cloud must yield an EMPTY foam, got {} struts", a.len()));
			}
		}
	}
	assert!(failures.is_empty(), "voronoi degeneracy handling failed:\n{}", failures.join("\n"));
}

#[test]
fn tiny_jitter_recovers_a_real_graph_from_a_cospherical_cluster() {
	// The flip side of graceful degeneracy: a cospherical cluster is exactly the
	// hard case, yet an infinitesimal jitter (general position) must recover a
	// real, non-empty, clean edge graph — proving the inertness handling does not
	// over-prune away legitimate geometry.
	let (lo, hi) = (Vec3::splat(-12.0), Vec3::splat(12.0));
	let mut seed = 0xD15A_5731_u64;
	let seeds: Vec<Vec3> = (0..20)
		.map(|i| {
			let a = std::f32::consts::TAU * i as f32 / 20.0;
			let b = std::f32::consts::PI * ((i * 7) % 20) as f32 / 20.0;
			// on a sphere of radius ~8, plus sub-0.01 jitter to break cosphericity
			let base = Vec3::new(b.sin() * a.cos(), b.sin() * a.sin(), b.cos()) * 8.0;
			base + Vec3::new(lcg(&mut seed) - 0.5, lcg(&mut seed) - 0.5, lcg(&mut seed) - 0.5) * 0.01
		})
		.collect();
	let struts = voronoi_struts(&seeds, lo, hi);
	assert!(
		!struts.is_empty() && clean(&struts, lo, hi) && identical(&struts, &voronoi_struts(&seeds, lo, hi)),
		"jittered near-cospherical cloud must give a real, clean, deterministic graph: {} struts, clean={}",
		struts.len(),
		clean(&struts, lo, hi)
	);
}

#[test]
fn voronoi_lattice_survives_duplicates_deterministically() {
	// The same degeneracy through the public `VoronoiLattice` constructor: a
	// seed cloud containing exact duplicates must build without panic and yield a
	// deterministic strut count (never NaN struts corrupting the grid).
	let seeds = vec![
		Vec3::new(-5.0, -5.0, -5.0),
		Vec3::new(5.0, -5.0, -5.0),
		Vec3::new(-5.0, 5.0, -5.0),
		Vec3::new(5.0, 5.0, 5.0),
		Vec3::new(0.0, 0.0, 0.0),
		Vec3::new(0.0, 0.0, 0.0), // duplicate
		Vec3::new(2.0, -3.0, 4.0),
		Vec3::new(-4.0, 2.0, -1.0),
	];
	let (lo, hi) = (Vec3::splat(-8.0), Vec3::splat(8.0));
	let a = VoronoiLattice::new(seeds.clone(), 0.7, lo, hi);
	let b = VoronoiLattice::new(seeds, 0.7, lo, hi);
	assert!(
		a.strut_count() == b.strut_count(),
		"duplicate-seed VoronoiLattice must be deterministic: {} vs {} struts",
		a.strut_count(),
		b.strut_count()
	);
}
