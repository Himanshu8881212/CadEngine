// Copyright (c) LMCAD. Licensed under the MIT License.

//! A regular polygon's vertices are ALL cocircular — every `incircle` test
//! ties. The constrained Delaunay triangulation must still be a valid disc
//! (n − 2 triangles, every interior edge shared by exactly two triangles, each
//! ring edge by one) whatever the ring's start vertex or winding: the
//! recovered 240-facet cone's cap ring (a CW-wound 240-gon starting mid-arc)
//! came back with overlapping triangles and 5 non-manifold edges.

use std::collections::HashMap;

use kernel_core::cdt::constrained_delaunay_checked;

fn check(pts: &[[f64; 2]], ring: &[usize], label: &str) {
	let tris = constrained_delaunay_checked(pts, &[ring.to_vec()]).unwrap_or_else(|e| panic!("{label}: refused: {e}"));
	assert_eq!(tris.len(), pts.len() - 2, "{label}: triangle count");
	let mut edges: HashMap<(usize, usize), usize> = HashMap::new();
	for t in &tris {
		for k in 0..3 {
			let (a, b) = (t[k], t[(k + 1) % 3]);
			*edges.entry((a.min(b), a.max(b))).or_insert(0) += 1;
		}
	}
	let n = ring.len();
	for i in 0..n {
		let (a, b) = (ring[i], ring[(i + 1) % n]);
		assert_eq!(edges.get(&(a.min(b), a.max(b))).copied(), Some(1), "{label}: ring edge {a}-{b} must be used once");
	}
	for (&(a, b), &m) in &edges {
		let on_ring = (0..n).any(|i| (ring[i] == a && ring[(i + 1) % n] == b) || (ring[i] == b && ring[(i + 1) % n] == a));
		if !on_ring {
			assert_eq!(m, 2, "{label}: interior edge {a}-{b} used {m}×");
		}
	}
}

#[test]
fn a_regular_240_gon_triangulates_as_a_clean_disc_in_every_ring_order() {
	let n = 240;
	let pts: Vec<[f64; 2]> = (0..n)
		.map(|k| {
			let a = std::f64::consts::TAU * k as f64 / n as f64;
			[9.0 * a.cos(), 9.0 * a.sin()]
		})
		.collect();
	let ccw: Vec<usize> = (0..n).collect();
	check(&pts, &ccw, "ccw from 0");
	let cw: Vec<usize> = (0..n).rev().collect();
	check(&pts, &cw, "cw from 0");
	for start in [1usize, 7, 119, 120, 121, 239] {
		let rot: Vec<usize> = (0..n).map(|i| (i + start) % n).collect();
		check(&pts, &rot, &format!("ccw from {start}"));
		let rotcw: Vec<usize> = rot.iter().rev().copied().collect();
		check(&pts, &rotcw, &format!("cw from {start}"));
	}
	// The recovered cap's exact ring: the 240-gon starting at −1.5° and wound CW.
	let shifted: Vec<[f64; 2]> = (0..n)
		.map(|k| {
			let a = std::f64::consts::TAU * (k as f64 - 1.0) / n as f64;
			[9.0 * a.cos(), 9.0 * a.sin()]
		})
		.collect();
	check(&shifted, &cw, "cw from -1.5°");
}
