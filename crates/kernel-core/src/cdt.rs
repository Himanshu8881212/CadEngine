// Copyright (c) LMCAD. Licensed under the MIT License.

//! Constrained Delaunay triangulation (CDT) of a planar region bounded by
//! polygon rings — the robust planar-face triangulator.
//!
//! **Why this exists.** The B-rep tessellators used to triangulate every planar
//! face by ear clipping a ring into which each hole had been spliced through a
//! zero-width "keyhole" corridor. That construction is fragile exactly where
//! booleans leave the densest geometry: an annular cap whose two rings carry
//! hundreds of near-collinear chord samples, a plate with several countersink
//! rims, a rounded-rectangle loft cap crossed by four notch cutters. The clip
//! stalls on flat corners, fans the remainder into slivers, and the welded
//! export comes back with non-orientable edges and degenerate triangles — so
//! `export_stl` abandoned the exact route on a plain tube (friction: nine
//! campaigns, `export_stl` / `kernel tessellation` surfaces).
//!
//! A CDT has none of those failure modes: it triangulates the ring VERTEX SET
//! (never inventing points), forces every ring edge to be a triangle edge, and
//! then keeps exactly the triangles inside the outer ring and outside every
//! hole by a parity flood. Triangle quality is Delaunay wherever the
//! constraints allow, so no sliver fans; zero-area triangles cannot arise from
//! a simple ring; holes never need a corridor.
//!
//! **Algorithm.** Bowyer–Watson incremental insertion into a super-triangle
//! (visibility walk from the previously inserted point, so ring-ordered input
//! locates in ~O(1)), then Anglada's constraint insertion — the triangles
//! crossed by a missing ring edge are removed and the two resulting
//! pseudo-polygons are re-triangulated with the recursive Delaunay-vertex rule
//! — then the parity flood. Every geometric decision goes through the exact
//! [`orient2d`] / [`incircle`] predicates, so near-collinear and cocircular
//! configurations are classified consistently instead of by rounded
//! determinants.
//!
//! **Contract.** Rings are index lists into `points` (any winding; the first is
//! the outer boundary, the rest are holes). Returns CCW index triangles covering
//! the region to floating-point area, or `None` when the input is not a valid
//! planar region (rings that cross, a ring vertex on another ring's edge, an
//! outer ring with zero area) — the caller falls back to its legacy path and
//! nothing is guessed. Duplicate coordinates are merged (a corridor-spliced
//! ring still triangulates, its doubled corridor edge counted with even parity
//! so it does not partition the face).

use std::collections::{HashMap, HashSet};

use crate::predicates::{incircle, orient2d};

const NONE: usize = usize::MAX;

/// Constrained Delaunay triangulation of the region inside `rings[0]` and
/// outside `rings[1..]`. See the module docs for the contract.
pub fn constrained_delaunay(points: &[[f64; 2]], rings: &[Vec<usize>]) -> Option<Vec<[usize; 3]>> {
	constrained_delaunay_checked(points, rings).ok()
}

/// [`constrained_delaunay`] that names WHY a ring set was refused — for
/// diagnostics and tests; the tessellators only need the `Option`.
pub fn constrained_delaunay_checked(points: &[[f64; 2]], rings: &[Vec<usize>]) -> Result<Vec<[usize; 3]>, String> {
	if rings.is_empty() || rings[0].len() < 3 {
		return Err("no outer ring with at least three vertices".into());
	}
	if points.iter().any(|p| !(p[0].is_finite() && p[1].is_finite())) {
		return Err("non-finite coordinate".into());
	}
	// Canonical index of every point: exact-coordinate duplicates collapse onto
	// their first occurrence, so a ring that revisits a position (a keyhole
	// corridor, a bridge twin) still describes one vertex.
	let mut canon = vec![0usize; points.len()];
	{
		let mut first: HashMap<(u64, u64), usize> = HashMap::with_capacity(points.len());
		for (i, p) in points.iter().enumerate() {
			canon[i] = *first.entry((p[0].to_bits(), p[1].to_bits())).or_insert(i);
		}
	}
	// Ring edges as canonical undirected constraints, with multiplicity: an edge
	// walked twice (a corridor) has even parity and is NOT a region boundary.
	let mut edge_mult: HashMap<(usize, usize), u32> = HashMap::new();
	let mut used: Vec<bool> = vec![false; points.len()];
	for ring in rings {
		let n = ring.len();
		if n < 3 {
			continue;
		}
		for k in 0..n {
			let (a, b) = (canon[ring[k]], canon[ring[(k + 1) % n]]);
			if a >= points.len() || b >= points.len() || a == b {
				continue;
			}
			used[a] = true;
			used[b] = true;
			*edge_mult.entry(key(a, b)).or_insert(0) += 1;
		}
	}
	let region_area = region_area(points, rings);
	if !(region_area > 0.0) {
		return Err(format!("the rings enclose no area ({region_area})"));
	}

	let mut cdt = Cdt::new(points, &used).ok_or("no ring vertices")?;
	// Insert the real vertices in ring order (spatial coherence for the walk).
	let mut last = cdt.tris.len() - 1;
	for ring in rings {
		for &i in ring {
			let c = canon[i];
			if c < points.len() && used[c] && !cdt.inserted[c] {
				last = cdt.insert_point(c, last).ok_or_else(|| format!("point {c} could not be inserted"))?;
			}
		}
	}
	// Force every ring edge.
	let mut constraints: Vec<(usize, usize)> = edge_mult.keys().copied().collect();
	constraints.sort_unstable();
	for (a, b) in constraints {
		if !cdt.insert_constraint(a, b, 0) {
			return Err(format!("ring edge {a}-{b} cannot be forced (rings cross or touch)"));
		}
	}
	let tris = cdt.classify(&edge_mult).ok_or("inconsistent ring parity (a ring vertex on another ring's edge?)")?;
	// The kept triangles must tile the ring region: any drop or double cover
	// (crossing rings, a vertex sitting on another ring's edge) shows up here.
	let covered: f64 = tris
		.iter()
		.map(|t| {
			let (a, b, c) = (points[t[0]], points[t[1]], points[t[2]]);
			0.5 * ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
		})
		.sum();
	if (covered - region_area).abs() > 1e-9 * region_area.max(1.0) {
		return Err(format!("triangles cover {covered} but the rings enclose {region_area}"));
	}
	Ok(tris)
}

/// |outer area| − Σ |hole areas| of the rings, in the input coordinates.
fn region_area(points: &[[f64; 2]], rings: &[Vec<usize>]) -> f64 {
	let shoelace = |ring: &[usize]| -> f64 {
		let n = ring.len();
		let mut s = 0.0;
		for k in 0..n {
			let p = points[ring[k]];
			let q = points[ring[(k + 1) % n]];
			s += p[0] * q[1] - q[0] * p[1];
		}
		(0.5 * s).abs()
	};
	let outer = shoelace(&rings[0]);
	let holes: f64 = rings[1..].iter().filter(|r| r.len() >= 3).map(|r| shoelace(r)).sum();
	outer - holes
}

#[inline]
fn key(a: usize, b: usize) -> (usize, usize) {
	if a < b {
		(a, b)
	} else {
		(b, a)
	}
}

struct Cdt {
	/// Input coordinates followed by the three super-triangle vertices.
	pts: Vec<[f64; 2]>,
	n_real: usize,
	/// CCW triangles; a removed triangle is `[NONE; 3]`.
	tris: Vec<[usize; 3]>,
	/// Directed edge `(a, b)` → the live triangle that walks `a → b`.
	edge_tri: HashMap<(usize, usize), usize>,
	/// Some triangle (possibly stale) that contained the vertex when last set.
	vert_tri: Vec<usize>,
	inserted: Vec<bool>,
	constraints: HashSet<(usize, usize)>,
}

impl Cdt {
	fn new(points: &[[f64; 2]], used: &[bool]) -> Option<Cdt> {
		let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
		let mut any = false;
		for (i, p) in points.iter().enumerate() {
			if !used[i] {
				continue;
			}
			any = true;
			lo = [lo[0].min(p[0]), lo[1].min(p[1])];
			hi = [hi[0].max(p[0]), hi[1].max(p[1])];
		}
		if !any {
			return None;
		}
		let c = [(lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5];
		let m = (hi[0] - lo[0]).max(hi[1] - lo[1]).max(1e-9) * 64.0;
		let mut pts = points.to_vec();
		let s0 = pts.len();
		pts.push([c[0] - m, c[1] - m]);
		pts.push([c[0] + m, c[1] - m]);
		pts.push([c[0], c[1] + m]);
		let mut cdt = Cdt {
			pts,
			n_real: points.len(),
			tris: Vec::new(),
			edge_tri: HashMap::new(),
			vert_tri: vec![NONE; s0 + 3],
			inserted: vec![false; s0],
			constraints: HashSet::new(),
		};
		cdt.add_tri(s0, s0 + 1, s0 + 2);
		Some(cdt)
	}

	fn add_tri(&mut self, a: usize, b: usize, c: usize) -> usize {
		let t = self.tris.len();
		self.tris.push([a, b, c]);
		self.edge_tri.insert((a, b), t);
		self.edge_tri.insert((b, c), t);
		self.edge_tri.insert((c, a), t);
		self.vert_tri[a] = t;
		self.vert_tri[b] = t;
		self.vert_tri[c] = t;
		t
	}

	fn remove_tri(&mut self, t: usize) {
		let [a, b, c] = self.tris[t];
		for e in [(a, b), (b, c), (c, a)] {
			if self.edge_tri.get(&e) == Some(&t) {
				self.edge_tri.remove(&e);
			}
		}
		self.tris[t] = [NONE; 3];
	}

	#[inline]
	fn alive(&self, t: usize) -> bool {
		t < self.tris.len() && self.tris[t][0] != NONE
	}

	/// The live triangle on the other side of the directed edge `a → b`.
	#[inline]
	fn neighbor(&self, a: usize, b: usize) -> Option<usize> {
		self.edge_tri.get(&(b, a)).copied()
	}

	#[inline]
	fn orient(&self, a: usize, b: usize, c: usize) -> f64 {
		orient2d(self.pts[a], self.pts[b], self.pts[c])
	}

	fn any_tri_with(&self, v: usize) -> Option<usize> {
		let t = self.vert_tri[v];
		if self.alive(t) && self.tris[t].contains(&v) {
			return Some(t);
		}
		(0..self.tris.len()).find(|&t| self.alive(t) && self.tris[t].contains(&v))
	}

	/// Visibility walk from `start` to a live triangle containing point `p`.
	fn locate(&self, p: usize, start: usize) -> Option<usize> {
		let mut t = if self.alive(start) { start } else { (0..self.tris.len()).rev().find(|&t| self.alive(t))? };
		let limit = 4 * self.tris.len() + 16;
		'walk: for _ in 0..limit {
			let tri = self.tris[t];
			for k in 0..3 {
				let (u, v) = (tri[k], tri[(k + 1) % 3]);
				if self.orient(u, v, p) < 0.0 {
					t = self.neighbor(u, v)?;
					continue 'walk;
				}
			}
			return Some(t);
		}
		// The walk can only cycle on a non-Delaunay triangulation; scan instead.
		(0..self.tris.len()).find(|&t| self.alive(t) && (0..3).all(|k| self.orient(self.tris[t][k], self.tris[t][(k + 1) % 3], p) >= 0.0))
	}

	/// Bowyer–Watson insertion of vertex `p`. Returns a triangle containing `p`
	/// (the next walk's start).
	fn insert_point(&mut self, p: usize, start: usize) -> Option<usize> {
		let t0 = self.locate(p, start)?;
		// A point coincident with an existing vertex is already inserted.
		if self.tris[t0].contains(&p) {
			self.inserted[p] = true;
			return Some(t0);
		}
		let mut cavity: Vec<usize> = Vec::new();
		let mut in_cav: HashSet<usize> = HashSet::new();
		let mut stack = vec![t0];
		in_cav.insert(t0);
		while let Some(x) = stack.pop() {
			cavity.push(x);
			let tri = self.tris[x];
			for k in 0..3 {
				let (u, v) = (tri[k], tri[(k + 1) % 3]);
				if let Some(n) = self.neighbor(u, v) {
					if !in_cav.contains(&n) {
						let [a, b, c] = self.tris[n];
						if incircle(self.pts[a], self.pts[b], self.pts[c], self.pts[p]) > 0.0 {
							in_cav.insert(n);
							stack.push(n);
						}
					}
				}
			}
		}
		let mut boundary: Vec<(usize, usize)> = Vec::new();
		for &x in &cavity {
			let tri = self.tris[x];
			for k in 0..3 {
				let (u, v) = (tri[k], tri[(k + 1) % 3]);
				match self.neighbor(u, v) {
					Some(n) if in_cav.contains(&n) => {}
					_ => boundary.push((u, v)),
				}
			}
		}
		for &x in &cavity {
			self.remove_tri(x);
		}
		let mut last = NONE;
		for (u, v) in boundary {
			if self.orient(u, v, p) <= 0.0 {
				return None; // not star-shaped: numerically impossible for a Delaunay cavity
			}
			last = self.add_tri(u, v, p);
		}
		self.inserted[p] = true;
		Some(last)
	}

	fn edge_exists(&self, a: usize, b: usize) -> bool {
		self.edge_tri.contains_key(&(a, b)) || self.edge_tri.contains_key(&(b, a))
	}

	/// Force `a – b` to be an edge (Anglada): remove the triangles the segment
	/// crosses and re-triangulate the two pseudo-polygons on either side. A
	/// vertex lying exactly on the segment splits the constraint there.
	fn insert_constraint(&mut self, a: usize, b: usize, depth: usize) -> bool {
		if a == b || depth > 64 {
			return a == b;
		}
		if self.edge_exists(a, b) {
			self.constraints.insert(key(a, b));
			return true;
		}
		// Find the triangle around `a` whose wedge contains the direction to `b`.
		let Some(start) = self.any_tri_with(a) else { return false };
		let mut t = start;
		let (mut eu, mut ew);
		let mut guard = 0usize;
		loop {
			guard += 1;
			if guard > self.tris.len() + 8 {
				return false;
			}
			let tri = self.tris[t];
			let k = match tri.iter().position(|&v| v == a) {
				Some(k) => k,
				None => return false,
			};
			let (u, w) = (tri[(k + 1) % 3], tri[(k + 2) % 3]);
			let ou = self.orient(a, u, b);
			let ow = self.orient(a, w, b);
			if ou == 0.0 && self.between(a, u, b) {
				return self.insert_constraint(a, u, depth + 1) && self.insert_constraint(u, b, depth + 1);
			}
			if ow == 0.0 && self.between(a, w, b) {
				return self.insert_constraint(a, w, depth + 1) && self.insert_constraint(w, b, depth + 1);
			}
			if ou > 0.0 && ow < 0.0 {
				eu = u; // right of a→b
				ew = w; // left of a→b
				break;
			}
			// Rotate CCW around `a`: the triangle across edge w→a walks a→w.
			match self.edge_tri.get(&(a, w)) {
				Some(&n) if n != start => t = n,
				_ => return false,
			}
		}
		let mut crossed = vec![t];
		let mut left = vec![ew];
		let mut right = vec![eu];
		loop {
			if self.constraints.contains(&key(eu, ew)) {
				return false; // crossing constraints: not a valid region
			}
			let Some(n) = self.neighbor(eu, ew) else { return false };
			let tri = self.tris[n];
			let x = *tri.iter().find(|&&v| v != eu && v != ew).unwrap_or(&NONE);
			if x == NONE {
				return false;
			}
			crossed.push(n);
			if x == b {
				break;
			}
			let ox = self.orient(a, b, x);
			if ox == 0.0 {
				// The segment passes through a vertex: split there (nothing has
				// been modified yet).
				return self.between(a, x, b) && self.insert_constraint(a, x, depth + 1) && self.insert_constraint(x, b, depth + 1);
			}
			if ox > 0.0 {
				left.push(x);
				ew = x;
			} else {
				right.push(x);
				eu = x;
			}
			if crossed.len() > self.tris.len() {
				return false;
			}
		}
		for &x in &crossed {
			self.remove_tri(x);
		}
		// Left pseudo-polygon (CCW): a, b, L_k … L_1. Right (CCW): b, a, R_1 … R_k.
		let mut lp = vec![a, b];
		lp.extend(left.iter().rev());
		let mut rp = vec![b, a];
		rp.extend(right.iter());
		self.triangulate_pseudo(&lp, 0);
		self.triangulate_pseudo(&rp, 0);
		self.constraints.insert(key(a, b));
		self.edge_exists(a, b)
	}

	/// Whether `m` lies strictly between `a` and `b` along their common line.
	fn between(&self, a: usize, m: usize, b: usize) -> bool {
		let (pa, pm, pb) = (self.pts[a], self.pts[m], self.pts[b]);
		let d = [(pm[0] - pa[0]) * (pb[0] - pa[0]) + (pm[1] - pa[1]) * (pb[1] - pa[1]), 0.0];
		let e = (pb[0] - pa[0]) * (pb[0] - pa[0]) + (pb[1] - pa[1]) * (pb[1] - pa[1]);
		d[0] > 0.0 && d[0] < e
	}

	/// Recursive Delaunay triangulation of a CCW pseudo-polygon whose base edge
	/// is `poly[0] → poly[1]`.
	fn triangulate_pseudo(&mut self, poly: &[usize], depth: usize) {
		let n = poly.len();
		if n < 3 || depth > 4096 {
			return;
		}
		let (p0, p1) = (poly[0], poly[1]);
		if n == 3 {
			if self.orient(p0, p1, poly[2]) > 0.0 {
				self.add_tri(p0, p1, poly[2]);
			}
			return;
		}
		let mut choice: Option<usize> = None;
		let mut fallback: Option<usize> = None;
		for i in 2..n {
			let c = poly[i];
			if self.orient(p0, p1, c) <= 0.0 {
				continue;
			}
			fallback.get_or_insert(i);
			let (pa, pb, pc) = (self.pts[p0], self.pts[p1], self.pts[c]);
			let empty = (2..n).all(|j| j == i || incircle(pa, pb, pc, self.pts[poly[j]]) <= 0.0);
			if empty {
				choice = Some(i);
				break;
			}
		}
		let Some(ci) = choice.or(fallback) else { return };
		let c = poly[ci];
		self.add_tri(p0, p1, c);
		// Sub-polygon 1: [c, p1, …, poly[ci-1]] with base c → p1.
		let mut sub1 = Vec::with_capacity(ci);
		sub1.push(c);
		sub1.extend_from_slice(&poly[1..ci]);
		self.triangulate_pseudo(&sub1, depth + 1);
		// Sub-polygon 2: [p0, c, poly[ci+1], …] with base p0 → c.
		let mut sub2 = Vec::with_capacity(n - ci + 1);
		sub2.push(p0);
		sub2.extend_from_slice(&poly[ci..]);
		self.triangulate_pseudo(&sub2, depth + 1);
	}

	/// Drop the super-triangle fan and keep the triangles of odd ring parity
	/// (inside the outer ring, outside the holes). Fails on an inconsistent
	/// flood (a region reached with two different parities) or an unlabelled
	/// triangle.
	fn classify(&self, edge_mult: &HashMap<(usize, usize), u32>) -> Option<Vec<[usize; 3]>> {
		let is_super = |v: usize| v >= self.n_real;
		let boundary_edge = |a: usize, b: usize| edge_mult.get(&key(a, b)).is_some_and(|&m| m % 2 == 1);
		let nt = self.tris.len();
		let mut label: Vec<i8> = vec![-1; nt];
		let mut queue: std::collections::VecDeque<usize> = std::collections::VecDeque::new();
		for t in 0..nt {
			if !self.alive(t) {
				continue;
			}
			let tri = self.tris[t];
			if tri.iter().any(|&v| is_super(v)) {
				continue;
			}
			for k in 0..3 {
				let (u, v) = (tri[k], tri[(k + 1) % 3]);
				let outside_neighbour = match self.neighbor(u, v) {
					Some(n) => self.tris[n].iter().any(|&w| is_super(w)),
					None => true,
				};
				if outside_neighbour {
					let l: i8 = if boundary_edge(u, v) { 1 } else { 0 };
					if label[t] == -1 {
						label[t] = l;
						queue.push_back(t);
					} else if label[t] != l {
						return None;
					}
				}
			}
		}
		while let Some(t) = queue.pop_front() {
			let tri = self.tris[t];
			for k in 0..3 {
				let (u, v) = (tri[k], tri[(k + 1) % 3]);
				let Some(n) = self.neighbor(u, v) else { continue };
				if self.tris[n].iter().any(|&w| is_super(w)) {
					continue;
				}
				let l = if boundary_edge(u, v) { 1 - label[t] } else { label[t] };
				if label[n] == -1 {
					label[n] = l;
					queue.push_back(n);
				} else if label[n] != l {
					return None;
				}
			}
		}
		let mut out = Vec::new();
		for t in 0..nt {
			if !self.alive(t) || self.tris[t].iter().any(|&v| is_super(v)) {
				continue;
			}
			match label[t] {
				1 => out.push(self.tris[t]),
				0 => {}
				_ => return None,
			}
		}
		Some(out)
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	fn area(points: &[[f64; 2]], tris: &[[usize; 3]]) -> f64 {
		tris.iter()
			.map(|t| {
				let (a, b, c) = (points[t[0]], points[t[1]], points[t[2]]);
				0.5 * ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
			})
			.sum()
	}

	/// Every ring edge must be a triangle edge, every triangle CCW and
	/// non-degenerate, and no interior edge used more than twice.
	fn check(points: &[[f64; 2]], rings: &[Vec<usize>], tris: &[[usize; 3]]) {
		let mut edges: HashMap<(usize, usize), u32> = HashMap::new();
		for t in tris {
			assert!(orient2d(points[t[0]], points[t[1]], points[t[2]]) > 0.0, "triangle {t:?} not CCW/non-degenerate");
			for k in 0..3 {
				*edges.entry((t[k], t[(k + 1) % 3])).or_insert(0) += 1;
			}
		}
		for (e, c) in &edges {
			assert_eq!(*c, 1, "directed edge {e:?} used {c} times");
		}
		for ring in rings {
			let n = ring.len();
			for k in 0..n {
				let (a, b) = (ring[k], ring[(k + 1) % n]);
				assert!(edges.contains_key(&(a, b)) || edges.contains_key(&(b, a)), "ring edge {a}-{b} missing");
			}
		}
	}

	fn circle(cx: f64, cy: f64, r: f64, n: usize, start: usize) -> (Vec<[f64; 2]>, Vec<usize>) {
		let pts: Vec<[f64; 2]> = (0..n)
			.map(|i| {
				let a = 2.0 * std::f64::consts::PI * i as f64 / n as f64;
				[cx + r * a.cos(), cy + r * a.sin()]
			})
			.collect();
		(pts, (start..start + n).collect())
	}

	#[test]
	fn square_with_square_hole() {
		let points = vec![[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0], [3.0, 3.0], [3.0, 7.0], [7.0, 7.0], [7.0, 3.0]];
		let rings = vec![vec![0, 1, 2, 3], vec![4, 5, 6, 7]];
		let tris = constrained_delaunay(&points, &rings).expect("triangulates");
		check(&points, &rings, &tris);
		assert!((area(&points, &tris) - 84.0).abs() < 1e-12);
		assert_eq!(tris.len(), 8);
	}

	#[test]
	fn dense_annulus_is_exact() {
		let (mut pts, outer) = circle(0.0, 0.0, 55.0, 192, 0);
		let (inner_pts, inner) = circle(0.0, 0.0, 35.0, 192, 192);
		pts.extend(inner_pts);
		let rings = vec![outer, inner];
		let tris = constrained_delaunay(&pts, &rings).expect("annulus triangulates");
		check(&pts, &rings, &tris);
		let want = region_area(&pts, &rings);
		assert!((area(&pts, &tris) - want).abs() < 1e-9 * want);
		// Euler: an annulus of V vertices, with all faces triangles, has 2V triangles.
		assert_eq!(tris.len(), 2 * 192);
	}

	#[test]
	fn concave_outer_with_offset_holes() {
		// An L-shaped outer with two round holes.
		let mut pts = vec![[0.0, 0.0], [40.0, 0.0], [40.0, 15.0], [15.0, 15.0], [15.0, 40.0], [0.0, 40.0]];
		let (h1, r1) = circle(8.0, 8.0, 4.0, 24, pts.len());
		pts.extend(h1);
		let (h2, r2) = circle(30.0, 7.0, 3.0, 16, pts.len());
		pts.extend(h2);
		let rings = vec![vec![0, 1, 2, 3, 4, 5], r1, r2];
		let tris = constrained_delaunay(&pts, &rings).expect("L with holes triangulates");
		check(&pts, &rings, &tris);
		let want = region_area(&pts, &rings);
		assert!((area(&pts, &tris) - want).abs() < 1e-9 * want);
	}

	#[test]
	fn collinear_boundary_runs_and_cw_input() {
		// A rectangle with many collinear subdivision points, wound CW.
		let mut pts = Vec::new();
		for i in 0..=10 {
			pts.push([i as f64, 0.0]);
		}
		for j in 1..=5 {
			pts.push([10.0, j as f64]);
		}
		for i in (0..10).rev() {
			pts.push([i as f64, 5.0]);
		}
		for j in (1..5).rev() {
			pts.push([0.0, j as f64]);
		}
		let n = pts.len();
		let mut ring: Vec<usize> = (0..n).collect();
		ring.reverse();
		let rings = vec![ring];
		let tris = constrained_delaunay(&pts, &rings).expect("triangulates");
		check(&pts, &rings, &tris);
		assert!((area(&pts, &tris) - 50.0).abs() < 1e-12);
	}

	#[test]
	fn keyhole_corridor_ring_triangulates_as_one_face() {
		// A bridged annulus: outer square, corridor to a square hole and back.
		let pts = vec![[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0], [7.0, 7.0], [3.0, 7.0], [3.0, 3.0], [7.0, 3.0]];
		// Corridor from outer vertex 1 into hole vertex 7, the hole walked CW, back out.
		let ring = vec![0, 1, 7, 6, 5, 4, 7, 1, 2, 3];
		let rings = vec![ring];
		let tris = constrained_delaunay(&pts, &rings).expect("corridor ring triangulates");
		assert!((area(&pts, &tris) - 84.0).abs() < 1e-12);
	}

	#[test]
	fn crossing_rings_are_refused() {
		let points = vec![[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0], [5.0, 5.0], [15.0, 5.0], [15.0, 15.0], [5.0, 15.0]];
		let rings = vec![vec![0, 1, 2, 3], vec![4, 5, 6, 7]];
		assert!(constrained_delaunay(&points, &rings).is_none());
	}

	#[test]
	fn fuzz_star_polygons_tile_exactly() {
		let mut state = 0x9E3779B97F4A7C15u64;
		let mut rng = || {
			state ^= state << 13;
			state ^= state >> 7;
			state ^= state << 17;
			(state >> 11) as f64 / (1u64 << 53) as f64
		};
		for case in 0..200 {
			let n = 5 + (rng() * 60.0) as usize;
			let mut pts: Vec<[f64; 2]> = (0..n)
				.map(|i| {
					let a = 2.0 * std::f64::consts::PI * i as f64 / n as f64;
					let r = 10.0 + 8.0 * rng();
					[r * a.cos(), r * a.sin()]
				})
				.collect();
			let mut rings = vec![(0..n).collect::<Vec<usize>>()];
			if case % 2 == 0 {
				let m = 3 + (rng() * 12.0) as usize;
				let start = pts.len();
				pts.extend((0..m).map(|i| {
					let a = 2.0 * std::f64::consts::PI * i as f64 / m as f64;
					let r = 1.0 + 3.0 * rng();
					[r * a.cos(), r * a.sin()]
				}));
				rings.push((start..start + m).collect());
			}
			let tris = constrained_delaunay(&pts, &rings).unwrap_or_else(|| panic!("case {case}: refused"));
			check(&pts, &rings, &tris);
			let want = region_area(&pts, &rings);
			assert!((area(&pts, &tris) - want).abs() < 1e-9 * want, "case {case}: area mismatch");
		}
	}
}
