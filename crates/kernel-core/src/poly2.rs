// Copyright (c) LMCAD. Licensed under the MIT License.

//! `poly2` — fast 2D polygon overlap, decoupled from the 3D boolean pipeline.
//!
//! **Motivation.** The drive kinematic simulators (planetary / harmonic / cycloidal
//! sweeps) evaluate the overlap of two 2D gear sections through hundreds of poses,
//! but each pose paid the full 3D cost — extrude both profiles, run the planar
//! arrangement boolean, take the volume. A planetary sweep took 631 s and a harmonic
//! one 309 s that way. All those sweeps actually need is the **intersection area** of
//! two simple polygons, which this module computes in well under 10 ms per pose for
//! gear-sized (~300–500 vertex) outlines.
//!
//! **Approach — robustness over cleverness.** Both polygons are triangulated by ear
//! clipping (exact [`crate::predicates::orient2d`] corner tests, so near-collinear
//! corners are classified consistently), then the intersection area is the sum of
//! convex–convex overlaps over all triangle pairs: each pair is clipped by
//! Sutherland–Hodgman and measured by the shoelace formula. Triangle-pair
//! decomposition is exact for simple polygons (the pieces tile each input with no
//! overlap), so no arrangement, snapping, or seam logic is needed. An AABB bin grid
//! over one triangle set prunes the O(nA·nB) pair loop to the pairs that can
//! actually touch.
//!
//! **Contract.** Correct results are promised for **simple** (non-self-intersecting)
//! polygons; winding may be CW or CCW (each is normalized). On garbage input —
//! self-intersecting outlines, repeated vertices, non-finite coordinates — the
//! function still returns *some* finite non-negative number without panicking, but
//! the value is not meaningful (a self-intersecting polygon has no well-defined
//! interior; its ear-clip cover may double-count).

use crate::predicates::orient2d;

/// Signed (shoelace) area of `poly`: positive for CCW winding. Not part of the
/// overlap contract by itself, but public because callers that build sweep
/// ratios (overlap ÷ section area) need the same area convention.
pub fn polygon_area(poly: &[[f64; 2]]) -> f64 {
	let n = poly.len();
	if n < 3 {
		return 0.0;
	}
	let mut s = 0.0;
	for i in 0..n {
		let p = poly[i];
		let q = poly[(i + 1) % n];
		s += p[0] * q[1] - q[0] * p[1];
	}
	0.5 * s
}

/// Area of the intersection of two simple polygons `a` and `b` (CW or CCW; CCW
/// is the documented convention). Non-convex polygons are fully supported.
///
/// Fast path for the drive kinematic sweeps: replaces the per-pose
/// extrude → 3D boolean → volume route (hundreds of ms per pose) with a pure-2D
/// triangle-pair clip (well under 10 ms for ~400-vertex gear outlines) — see the
/// module docs for the measured motivation and the [module contract](self) for
/// what is and is not promised on degenerate input.
pub fn polygon_intersection_area(a: &[[f64; 2]], b: &[[f64; 2]]) -> f64 {
	let ta = triangulate(a);
	let tb = triangulate(b);
	if ta.is_empty() || tb.is_empty() {
		return 0.0;
	}

	// AABB bin grid over B's triangles: for gear-sized inputs (~800 tris/side) this
	// skips the overwhelmingly disjoint pairs of the O(nA·nB) loop.
	let grid = TriGrid::build(&tb);
	let mut stamp = vec![0u32; tb.len()];
	let mut generation = 0u32;
	let mut candidates: Vec<u32> = Vec::new();

	let mut total = 0.0;
	for tri_a in &ta {
		let bb = tri_aabb(tri_a);
		generation += 1;
		grid.gather(bb, &mut stamp, generation, &mut candidates);
		for &j in &candidates {
			let tri_b = &tb[j as usize];
			let bb_b = tri_aabb(tri_b);
			if bb.0[0] > bb_b.1[0] || bb.1[0] < bb_b.0[0] || bb.0[1] > bb_b.1[1] || bb.1[1] < bb_b.0[1] {
				continue;
			}
			// Degenerate / zero-area clip results are clamped to zero inside.
			total += tri_tri_clip_area(tri_a, tri_b);
		}
	}
	// Garbage-input safety net: the contract promises a finite non-negative number.
	if total.is_finite() {
		total.max(0.0)
	} else {
		0.0
	}
}

// --- Triangulation (ear clipping) ---------------------------------------------

type Tri = [[f64; 2]; 3];

/// Sanitize + ear-clip a polygon into CCW triangles. Never panics: non-finite
/// vertices are dropped, consecutive duplicates and exactly-collinear run-ons are
/// drained, a stalled clip (self-intersecting input) falls back to fanning the
/// remainder, and zero-area triangles are discarded.
fn triangulate(poly: &[[f64; 2]]) -> Vec<Tri> {
	// Drop non-finite vertices and consecutive (near-)duplicates.
	let mut pts: Vec<[f64; 2]> = Vec::with_capacity(poly.len());
	for &p in poly {
		if !(p[0].is_finite() && p[1].is_finite()) {
			continue;
		}
		if let Some(&last) = pts.last() {
			if (p[0] - last[0]).abs() < 1e-12 && (p[1] - last[1]).abs() < 1e-12 {
				continue;
			}
		}
		pts.push(p);
	}
	while pts.len() >= 2 {
		let (first, last) = (pts[0], pts[pts.len() - 1]);
		if (first[0] - last[0]).abs() < 1e-12 && (first[1] - last[1]).abs() < 1e-12 {
			pts.pop();
		} else {
			break;
		}
	}
	let n = pts.len();
	if n < 3 {
		return Vec::new();
	}

	let mut idx: Vec<usize> = (0..n).collect();
	if polygon_area(&pts) < 0.0 {
		idx.reverse(); // normalize to CCW for the ear test
	}
	// Drain exactly-collinear corners up front so flat vertices can't stall the clip.
	let mut i = 0;
	while idx.len() > 3 && i < idx.len() {
		let m = idx.len();
		let (ip, ic, inx) = (idx[(i + m - 1) % m], idx[i], idx[(i + 1) % m]);
		if orient2d(pts[ip], pts[ic], pts[inx]) == 0.0 {
			idx.remove(i);
			i = 0;
		} else {
			i += 1;
		}
	}

	let mut tris: Vec<Tri> = Vec::with_capacity(n - 2);
	let mut guard = 0usize;
	while idx.len() > 3 && guard < 100_000 {
		guard += 1;
		let m = idx.len();
		let mut clipped = false;
		for i in 0..m {
			let ip = idx[(i + m - 1) % m];
			let ic = idx[i];
			let inx = idx[(i + 1) % m];
			let (a, b, c) = (pts[ip], pts[ic], pts[inx]);
			// Convex corner (CCW)? Exact orientation classifies near-collinear
			// corners consistently instead of by a rounded f64 sign.
			if orient2d(a, b, c) <= 0.0 {
				continue;
			}
			// No other ring vertex inside OR on the boundary of the candidate
			// ear? Boundary contact must block: a reflex vertex lying exactly on
			// the ear's chord (e.g. an L whose notch corner sits on the corner
			// diagonal) makes the chord exit the polygon even though nothing is
			// strictly inside — clipping such an ear yields an overlapping,
			// area-double-counting cover. Blocking it lets a different ear
			// advance; a full stall falls back to the fan below.
			let mut ok = true;
			for &j in &idx {
				if j == ip || j == ic || j == inx {
					continue;
				}
				let p = pts[j];
				if orient2d(a, b, p) >= 0.0 && orient2d(b, c, p) >= 0.0 && orient2d(c, a, p) >= 0.0 {
					ok = false;
					break;
				}
			}
			if ok {
				tris.push([a, b, c]);
				idx.remove(i);
				clipped = true;
				break;
			}
		}
		if !clipped {
			break; // self-intersecting / degenerate: fan the remainder below
		}
	}
	for w in 1..idx.len().saturating_sub(1) {
		tris.push([pts[idx[0]], pts[idx[w]], pts[idx[w + 1]]]);
	}

	// Normalize every output triangle to CCW and drop zero-area slivers (the fan
	// fallback on garbage input can emit either).
	tris.retain_mut(|t| {
		let s = orient2d(t[0], t[1], t[2]);
		if s < 0.0 {
			t.swap(1, 2);
		}
		s != 0.0
	});
	tris
}

// --- Convex–convex clip (Sutherland–Hodgman) ----------------------------------

/// AABB of a triangle as `(min, max)`.
#[inline]
fn tri_aabb(t: &Tri) -> ([f64; 2], [f64; 2]) {
	let min = [t[0][0].min(t[1][0]).min(t[2][0]), t[0][1].min(t[1][1]).min(t[2][1])];
	let max = [t[0][0].max(t[1][0]).max(t[2][0]), t[0][1].max(t[1][1]).max(t[2][1])];
	(min, max)
}

/// Area of `subject ∩ clip` for two CCW triangles: Sutherland–Hodgman against the
/// three half-planes of `clip`, then shoelace. The output of clipping a triangle
/// by three half-planes has at most 6 vertices, so fixed 8-slot buffers suffice.
/// Degenerate results (empty, sliver, numerically negative) come back as `0.0`.
#[inline]
fn tri_tri_clip_area(subject: &Tri, clip: &Tri) -> f64 {
	let mut cur = [[0.0f64; 2]; 8];
	let mut next = [[0.0f64; 2]; 8];
	cur[..3].copy_from_slice(subject);
	let mut n = 3usize;

	for e in 0..3 {
		let p0 = clip[e];
		let p1 = clip[(e + 1) % 3];
		let ex = p1[0] - p0[0];
		let ey = p1[1] - p0[1];
		let mut m = 0usize;
		if n == 0 {
			return 0.0;
		}
		let mut prev = cur[n - 1];
		let mut prev_side = ex * (prev[1] - p0[1]) - ey * (prev[0] - p0[0]);
		for &pt in cur.iter().take(n) {
			let side = ex * (pt[1] - p0[1]) - ey * (pt[0] - p0[0]);
			if side >= 0.0 {
				if prev_side < 0.0 {
					// Entering: emit the crossing point. Signs are strictly
					// opposite here, so the denominator is nonzero.
					let t = prev_side / (prev_side - side);
					next[m] = [prev[0] + t * (pt[0] - prev[0]), prev[1] + t * (pt[1] - prev[1])];
					m += 1;
				}
				next[m] = pt;
				m += 1;
			} else if prev_side >= 0.0 {
				// Leaving: emit the crossing point.
				let t = prev_side / (prev_side - side);
				next[m] = [prev[0] + t * (pt[0] - prev[0]), prev[1] + t * (pt[1] - prev[1])];
				m += 1;
			}
			prev = pt;
			prev_side = side;
		}
		cur[..m].copy_from_slice(&next[..m]);
		n = m;
	}

	if n < 3 {
		return 0.0;
	}
	let mut s = 0.0;
	for i in 0..n {
		let p = cur[i];
		let q = cur[(i + 1) % n];
		s += p[0] * q[1] - q[0] * p[1];
	}
	// A CCW ∩ CCW clip is CCW; a numerically negative sliver is a degenerate
	// result and reads as zero per the module contract.
	(0.5 * s).max(0.0)
}

// --- AABB bin grid over one triangle set ---------------------------------------

/// Uniform bin grid over a triangle set's AABBs. `gather` returns the indices of
/// every triangle whose AABB might overlap a query AABB, deduplicated with a
/// generation-stamped visited array (no per-query allocation or clearing).
struct TriGrid {
	min: [f64; 2],
	inv_cell: [f64; 2],
	nx: usize,
	ny: usize,
	cells: Vec<Vec<u32>>,
}

impl TriGrid {
	fn build(tris: &[Tri]) -> TriGrid {
		let mut lo = [f64::INFINITY; 2];
		let mut hi = [f64::NEG_INFINITY; 2];
		for t in tris {
			let (a, b) = tri_aabb(t);
			lo = [lo[0].min(a[0]), lo[1].min(a[1])];
			hi = [hi[0].max(b[0]), hi[1].max(b[1])];
		}
		// ~1 triangle per cell on average; clamped so tiny/huge inputs stay sane.
		let n = ((tris.len() as f64).sqrt().ceil() as usize).clamp(1, 64);
		let ext = [(hi[0] - lo[0]).max(1e-12), (hi[1] - lo[1]).max(1e-12)];
		let mut grid = TriGrid {
			min: lo,
			inv_cell: [n as f64 / ext[0], n as f64 / ext[1]],
			nx: n,
			ny: n,
			cells: vec![Vec::new(); n * n],
		};
		for (i, t) in tris.iter().enumerate() {
			let (a, b) = tri_aabb(t);
			let (x0, y0, x1, y1) = grid.cell_range(a, b);
			for cy in y0..=y1 {
				for cx in x0..=x1 {
					grid.cells[cy * grid.nx + cx].push(i as u32);
				}
			}
		}
		grid
	}

	/// Clamped cell-index range covered by an AABB.
	#[inline]
	fn cell_range(&self, lo: [f64; 2], hi: [f64; 2]) -> (usize, usize, usize, usize) {
		let cx0 = (((lo[0] - self.min[0]) * self.inv_cell[0]).floor().max(0.0) as usize).min(self.nx - 1);
		let cy0 = (((lo[1] - self.min[1]) * self.inv_cell[1]).floor().max(0.0) as usize).min(self.ny - 1);
		let cx1 = (((hi[0] - self.min[0]) * self.inv_cell[0]).floor().max(0.0) as usize).min(self.nx - 1);
		let cy1 = (((hi[1] - self.min[1]) * self.inv_cell[1]).floor().max(0.0) as usize).min(self.ny - 1);
		(cx0, cy0, cx1, cy1)
	}

	/// Collect (deduplicated) candidate triangle indices for a query AABB into
	/// `out`. `stamp[i] == generation` marks index `i` as already emitted.
	fn gather(&self, bb: ([f64; 2], [f64; 2]), stamp: &mut [u32], generation: u32, out: &mut Vec<u32>) {
		out.clear();
		let (x0, y0, x1, y1) = self.cell_range(bb.0, bb.1);
		for cy in y0..=y1 {
			for cx in x0..=x1 {
				for &i in &self.cells[cy * self.nx + cx] {
					if stamp[i as usize] != generation {
						stamp[i as usize] = generation;
						out.push(i);
					}
				}
			}
		}
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	/// Deterministic xorshift64* — good enough for adversarial fuzz coordinates.
	fn rng(state: &mut u64) -> f64 {
		*state ^= *state << 13;
		*state ^= *state >> 7;
		*state ^= *state << 17;
		(*state >> 11) as f64 / (1u64 << 53) as f64
	}

	#[test]
	fn known_overlaps_are_exact() {
		let square = |cx: f64, cy: f64, s: f64| -> Vec<[f64; 2]> {
			vec![[cx - s, cy - s], [cx + s, cy - s], [cx + s, cy + s], [cx - s, cy + s]]
		};
		// Non-convex L: a 2×2 square missing its top-right 1×1 quadrant (area 3).
		let ell: Vec<[f64; 2]> = vec![[0.0, 0.0], [2.0, 0.0], [2.0, 1.0], [1.0, 1.0], [1.0, 2.0], [0.0, 2.0]];
		let mut cw = square(0.0, 0.0, 1.0);
		cw.reverse();

		let cases: [(f64, f64, &str); 6] = [
			(polygon_intersection_area(&square(0.0, 0.0, 1.0), &square(1.0, 1.0, 1.0)), 1.0, "quarter-offset squares"),
			(polygon_intersection_area(&square(0.0, 0.0, 1.0), &square(5.0, 0.0, 1.0)), 0.0, "disjoint squares"),
			(polygon_intersection_area(&square(0.0, 0.0, 1.0), &square(0.0, 0.0, 1.0)), 4.0, "identical squares"),
			(polygon_intersection_area(&ell, &ell), 3.0, "L-shape with itself"),
			// [0.5,1.5]² straddles the notch corner: overlap = 1 − the 0.5² notch quadrant.
			(polygon_intersection_area(&ell, &square(1.0, 1.0, 0.5)), 0.75, "L-shape ∩ notch-straddling square"),
			(polygon_intersection_area(&cw, &square(0.5, 0.0, 1.0)), 3.0, "CW-wound input is normalized"),
		];
		let bad: Vec<String> = cases
			.iter()
			.filter(|(got, want, _)| (got - want).abs() > 1e-9)
			.map(|(got, want, name)| format!("{name}: got {got}, want {want}"))
			.collect();
		assert!(bad.is_empty(), "polygon_intersection_area known-answer failures:\n{}", bad.join("\n"));
	}

	#[test]
	fn garbage_input_never_panics_and_stays_finite_nonnegative() {
		// Contract: correctness is only promised for SIMPLE polygons. This test
		// promises the weaker guarantee for everything else: random
		// self-intersecting polygons, repeated vertices, tiny/huge coordinates,
		// NaN/±inf spikes, and empty/degenerate inputs must all return SOME
		// finite non-negative number without panicking.
		let mut state = 0x9E3779B97F4A7C15u64;
		let mut failures = Vec::new();
		for case in 0..500 {
			let na = (rng(&mut state) * 60.0) as usize; // 0..60 vertices — includes degenerate counts
			let nb = (rng(&mut state) * 60.0) as usize;
			let mut mk = |n: usize| -> Vec<[f64; 2]> {
				(0..n)
					.map(|k| {
						let mut x = (rng(&mut state) - 0.5) * 2e3;
						let mut y = (rng(&mut state) - 0.5) * 2e3;
						if case % 7 == 3 && k % 5 == 0 {
							x = [f64::NAN, f64::INFINITY, f64::NEG_INFINITY][k % 3];
						}
						if case % 11 == 5 && k % 4 == 1 {
							y = f64::NAN;
						}
						if case % 5 == 2 && k % 3 == 0 {
							x *= 1e150; // overflow bait for the clip arithmetic
						}
						[x, y]
					})
					.collect()
			};
			let a = mk(na);
			let b = mk(nb);
			let area = polygon_intersection_area(&a, &b);
			if !(area.is_finite() && area >= 0.0) {
				failures.push(format!("case {case}: area = {area} (na={na}, nb={nb})"));
			}
		}
		assert!(
			failures.is_empty(),
			"garbage-input contract violated ({} of 500 cases):\n{}",
			failures.len(),
			failures.join("\n")
		);
	}
}
