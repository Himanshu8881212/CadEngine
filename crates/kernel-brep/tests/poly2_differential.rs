// Copyright (c) LMCAD. Licensed under the MIT License.

//! Differential validation of `kernel_core::poly2::polygon_intersection_area`
//! against the 3D kernel it replaces in the drive kinematic sweeps.
//!
//! Reference route per case: extrude both profiles to height 1.0, run the exact
//! planar-arrangement boolean intersection, take `exact_volume` — for unit height
//! that volume IS the overlap area. The fast 2D path must agree to 0.5% relative
//! (or 1e-3 absolute when the overlap is near zero).
//!
//! This file lives in kernel-brep (not kernel-core) deliberately: kernel-brep
//! depends on kernel-core, so a kernel-core dev-dependency on kernel-brep would
//! be a cycle; here both sides of the differential are visible.

use glam::DVec2;
use kernel_brep::{exact_volume, extrude, try_intersection};
use kernel_core::poly2::polygon_intersection_area;

/// Deterministic xorshift64* in [0, 1). Fixed seeds keep the suite reproducible.
fn rng(state: &mut u64) -> f64 {
	*state ^= *state << 13;
	*state ^= *state >> 7;
	*state ^= *state << 17;
	(*state >> 11) as f64 / (1u64 << 53) as f64
}

/// Random convex polygon: `n` sorted random angles on a random-radii ellipse
/// (a polygon inscribed in an ellipse is convex), translated to `center`.
fn convex_polygon(state: &mut u64, n: usize, center: [f64; 2]) -> Vec<[f64; 2]> {
	let rx = 5.0 + 15.0 * rng(state);
	let ry = 5.0 + 15.0 * rng(state);
	let mut angles: Vec<f64> = (0..n).map(|_| rng(state) * std::f64::consts::TAU).collect();
	angles.sort_by(|a, b| a.partial_cmp(b).unwrap());
	angles.dedup();
	angles.iter().map(|t| [center[0] + rx * t.cos(), center[1] + ry * t.sin()]).collect()
}

/// Random star-shaped (hence simple, generally non-convex) polygon: uniform
/// sorted angles, random radius per vertex in [r_min, r_max].
fn star_polygon(state: &mut u64, n: usize, center: [f64; 2]) -> Vec<[f64; 2]> {
	let r_max = 8.0 + 12.0 * rng(state);
	let r_min = 0.35 * r_max;
	(0..n)
		.map(|k| {
			let t = k as f64 / n as f64 * std::f64::consts::TAU;
			let r = r_min + (r_max - r_min) * rng(state);
			[center[0] + r * t.cos(), center[1] + r * t.sin()]
		})
		.collect()
}

/// Gear-like tooth ring in the style of the `kernel_model` drive sections: a
/// circle at root radius carrying rectangular (radial-flanked) teeth at tip
/// radius. `pts_per_arc` points sample each root gap and each tooth top, so the
/// total vertex count is `teeth * 2 * pts_per_arc` (e.g. 50 teeth × 4 = 400).
fn gear_polygon(teeth: usize, r_root: f64, r_tip: f64, pts_per_arc: usize, center: [f64; 2], phase: f64) -> Vec<[f64; 2]> {
	let mut poly = Vec::with_capacity(teeth * 2 * pts_per_arc);
	let pitch = std::f64::consts::TAU / teeth as f64;
	for i in 0..teeth {
		let a0 = phase + i as f64 * pitch;
		// First half of the pitch: root gap; second half: tooth top. The radius
		// jump between consecutive samples forms the (radial) tooth flanks.
		for (r, lo, hi) in [(r_root, 0.0, 0.5), (r_tip, 0.5, 1.0)] {
			for k in 0..pts_per_arc {
				let t = a0 + pitch * (lo + (hi - lo) * k as f64 / pts_per_arc as f64);
				poly.push([center[0] + r * t.cos(), center[1] + r * t.sin()]);
			}
		}
	}
	poly
}

/// Overlap area via the 3D kernel: extrude to unit height, exact boolean, volume.
fn reference_overlap_area_3d(a: &[[f64; 2]], b: &[[f64; 2]]) -> Result<f64, String> {
	let pa: Vec<DVec2> = a.iter().map(|p| DVec2::new(p[0], p[1])).collect();
	let pb: Vec<DVec2> = b.iter().map(|p| DVec2::new(p[0], p[1])).collect();
	let sa = extrude(&pa, 1.0);
	let sb = extrude(&pb, 1.0);
	match try_intersection(&sa, &sb) {
		Ok(s) => Ok(exact_volume(&s)),
		Err(e) => Err(format!("3D boolean refused: {e:?}")),
	}
}

/// One differential case: label + the two polygons.
type Case = (String, Vec<[f64; 2]>, Vec<[f64; 2]>);

#[test]
fn poly2_matches_3d_extrude_boolean_volume_route() {
	let mut state = 0xC0FFEE1234567u64;
	let mut cases: Vec<Case> = Vec::new();

	// 10 convex pairs (8–24 vertices), random partial offsets.
	for k in 0..10 {
		let na = 8 + (rng(&mut state) * 16.0) as usize;
		let nb = 8 + (rng(&mut state) * 16.0) as usize;
		let off = [4.0 + 20.0 * rng(&mut state), (rng(&mut state) - 0.5) * 16.0];
		let a = convex_polygon(&mut state, na, [0.0, 0.0]);
		let b = convex_polygon(&mut state, nb, off);
		cases.push((format!("convex[{k}] ({na}x{nb} verts, off {off:?})"), a, b));
	}
	// 10 star-shaped (non-convex) pairs, 20–80 vertices.
	for k in 0..10 {
		let na = 20 + (rng(&mut state) * 60.0) as usize;
		let nb = 20 + (rng(&mut state) * 60.0) as usize;
		let off = [3.0 + 14.0 * rng(&mut state), (rng(&mut state) - 0.5) * 12.0];
		let a = star_polygon(&mut state, na, [0.0, 0.0]);
		let b = star_polygon(&mut state, nb, off);
		cases.push((format!("star[{k}] ({na}x{nb} verts, off {off:?})"), a, b));
	}
	// 5 gear-like tooth-ring pairs (the actual drive-simulator shape class),
	// meshed at varying center distances from deep engagement to near-tip.
	for k in 0..5 {
		let ta = 9 + 2 * k;
		let tb = 11 + k;
		let dist = 14.0 + 2.5 * k as f64;
		let a = gear_polygon(ta, 8.0, 10.5, 3, [0.0, 0.0], 0.0);
		let b = gear_polygon(tb, 7.0, 9.5, 3, [dist, 0.0], rng(&mut state) * std::f64::consts::TAU);
		cases.push((format!("gear[{k}] ({ta}x{tb} teeth, center distance {dist})"), a, b));
	}
	assert_eq!(cases.len(), 25, "differential corpus must be exactly the promised 25 cases");

	let mut failures = Vec::new();
	let mut max_rel = 0.0f64;
	let mut sum_rel = 0.0f64;
	let mut rel_count = 0usize;
	for (name, a, b) in &cases {
		let fast = polygon_intersection_area(a, b);
		let reference = match reference_overlap_area_3d(a, b) {
			Ok(v) => v,
			Err(e) => {
				failures.push(format!("{name}: {e}"));
				continue;
			}
		};
		// Near-zero overlaps compare absolutely (a relative bound is meaningless
		// at 0); everything else must agree to 0.5% relative.
		if reference.abs() < 1e-3 || fast.abs() < 1e-3 {
			if (fast - reference).abs() > 1e-3 {
				failures.push(format!("{name}: near-zero mismatch fast={fast} vs 3D={reference}"));
			}
		} else {
			let rel = (fast - reference).abs() / reference.abs();
			max_rel = max_rel.max(rel);
			sum_rel += rel;
			rel_count += 1;
			if rel > 0.005 {
				failures.push(format!("{name}: rel err {:.4}% (fast={fast}, 3D={reference})", rel * 100.0));
			}
		}
	}
	println!(
		"poly2 differential vs 3D route: {} cases, {} compared relatively, max rel err {:.3e}, mean rel err {:.3e}",
		cases.len(),
		rel_count,
		max_rel,
		if rel_count > 0 { sum_rel / rel_count as f64 } else { 0.0 }
	);
	assert!(
		failures.is_empty(),
		"poly2 vs 3D extrude/boolean/volume disagreements ({} of {}):\n{}",
		failures.len(),
		cases.len(),
		failures.join("\n")
	);
}

#[test]
fn bench_poly2_is_at_least_20x_faster_than_3d_route_on_400_vertex_gears() {
	// The workload the module exists for: one pose of two meshing 400-vertex
	// gear sections. 100 poly2 calls vs 10 full 3D-route calls (each 3D call
	// pays extrude + extrude + boolean + volume, exactly what a sweep pose paid).
	let a = gear_polygon(50, 20.0, 23.0, 4, [0.0, 0.0], 0.0);
	let b = gear_polygon(50, 19.0, 22.0, 4, [42.0, 0.0], 0.031);
	assert_eq!((a.len(), b.len()), (400, 400), "bench gear outlines must be the promised 400 vertices");

	// Agreement first — speed of a wrong answer is worthless.
	let fast = polygon_intersection_area(&a, &b);
	let reference = reference_overlap_area_3d(&a, &b).expect("3D reference route must build the bench pose");
	let rel = (fast - reference).abs() / reference.abs();
	assert!(rel < 0.005, "bench pose disagrees before timing: fast={fast} vs 3D={reference} (rel {:.4}%)", rel * 100.0);

	let t0 = std::time::Instant::now();
	for _ in 0..100 {
		std::hint::black_box(polygon_intersection_area(std::hint::black_box(&a), std::hint::black_box(&b)));
	}
	let per_call_2d = t0.elapsed().as_secs_f64() / 100.0;

	let t1 = std::time::Instant::now();
	for _ in 0..10 {
		std::hint::black_box(reference_overlap_area_3d(std::hint::black_box(&a), std::hint::black_box(&b)).expect("3D route"));
	}
	let per_call_3d = t1.elapsed().as_secs_f64() / 10.0;

	let speedup = per_call_3d / per_call_2d;
	println!(
		"poly2 bench (400-vertex gear pair): poly2 {:.3} ms/call (100 calls), 3D route {:.1} ms/call (10 calls) — {speedup:.0}x speedup",
		per_call_2d * 1e3,
		per_call_3d * 1e3
	);
	assert!(
		speedup >= 20.0,
		"poly2 must be at least 20x faster per call than the 3D route: measured {speedup:.1}x ({:.3} ms vs {:.1} ms)",
		per_call_2d * 1e3,
		per_call_3d * 1e3
	);
	// The motivating budget: well under 10 ms per pose for gear-sized outlines.
	assert!(per_call_2d < 0.010, "poly2 per-call budget blown: {:.3} ms >= 10 ms", per_call_2d * 1e3);
}
