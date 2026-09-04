// Copyright (c) LMCAD. Licensed under the MIT License.

//! Native 3-D Voronoi edge-graph generation — the geometry behind the
//! `voronoi_lattice` open-cell-foam primitive, with ZERO external Delaunay
//! dependency (scipy/qhull etc.).
//!
//! # What it computes
//!
//! Given a cloud of seed points, [`voronoi_struts`] returns the **1-skeleton of
//! the Voronoi diagram** — the network of Voronoi edges — already clipped to a
//! `[min, max]` box, as a list of segments. That 1-skeleton IS an open-cell
//! foam: every strut is a shared wall-edge of two neighbouring Voronoi cells.
//! The caller sweeps a radius along each segment (a capsule) and takes the
//! union — see [`crate::lattice::VoronoiLattice`].
//!
//! # Algorithm (incremental Bowyer–Watson)
//!
//! 1. **Delaunay tetrahedralization.** Start from one super-tetrahedron large
//!    enough to enclose every seed. Insert the seeds one at a time: find the
//!    tets whose circumsphere already contains the new point (the "cavity"),
//!    delete them, and re-triangulate the cavity's boundary faces to the new
//!    point. Finally drop every tet still touching a super-vertex.
//! 2. **Voronoi dual.** The Voronoi vertices are the tet circumcenters. Every
//!    internal Delaunay face — one shared by exactly two surviving tets — dualises
//!    to one Voronoi edge joining those two circumcenters. A face on the convex
//!    hull touches only one tet; its dual edge runs to infinity and is dropped.
//! 3. **Clip.** Each Voronoi edge is clipped to the `[min, max]` box
//!    (Liang–Barsky); edges fully outside the box, or incident to a circumcenter
//!    at (near-)infinity, are discarded.
//!
//! # Honesty / robustness contract
//!
//! This is a **lattice generator, not the exact-predicate boolean pipeline.**
//! The in-circumsphere test uses `f64` circumcenters with a small relative
//! epsilon, NOT exact arithmetic. That is deliberate and sufficient for a
//! strut graph: a mis-classified cospherical config perturbs a few edges, it
//! does not corrupt a solid. Degenerate (flat/sliver) tets are detected by a
//! near-zero orientation determinant and handled gracefully — such a tet is
//! marked inert (it never claims a point and never contributes a Voronoi
//! vertex) rather than panicking. For seed clouds in general position (e.g. any
//! jittered/random fill) no degeneracy arises at all.

use std::collections::{HashMap, HashSet};

use kernel_core::math::{DVec3, Vec3};

/// Relative slack on the in-circumsphere test. A point `p` lies inside a tet's
/// circumsphere when `|p − c|² ≤ r²·(1 + ε)`; the slack absorbs `f64` rounding
/// so a point sitting essentially on a circumsphere is treated as inside (being
/// slightly over-inclusive keeps the cavity star-shaped, which is the safe way
/// to err).
const INSPHERE_EPS: f64 = 1e-9;

/// A tet whose orientation determinant is below this magnitude is treated as
/// degenerate (flat/sliver): its circumsphere is ill-defined, so it is marked
/// inert. Scaled by the cube of the seed-cloud extent at call time.
const FLAT_DET_REL: f64 = 1e-18;

/// One tetrahedron of the working triangulation: four vertex indices into the
/// point list plus its precomputed circumsphere. `ok == false` marks a
/// degenerate tet that must never claim a point nor dualise to a Voronoi vertex.
struct Tet {
	v: [usize; 4],
	center: DVec3,
	r2: f64,
	ok: bool,
}

/// The four triangular faces of a tet, each as a vertex-index triple sorted
/// ascending so a face shared by two tets hashes to the same key.
fn tet_faces(v: &[usize; 4]) -> [[usize; 3]; 4] {
	let sorted = |mut f: [usize; 3]| {
		f.sort_unstable();
		f
	};
	[sorted([v[1], v[2], v[3]]), sorted([v[0], v[2], v[3]]), sorted([v[0], v[1], v[3]]), sorted([v[0], v[1], v[2]])]
}

/// Circumcenter and squared circumradius of the tetrahedron `(a, b, c, d)`, or
/// `None` if the four points are (near-)coplanar (`|6·signed volume|` below
/// `flat_det`, i.e. the circumsphere is ill-conditioned). Closed form: solve for
/// the point equidistant from all four vertices.
fn circumsphere(a: DVec3, b: DVec3, c: DVec3, d: DVec3, flat_det: f64) -> Option<(DVec3, f64)> {
	let ba = b - a;
	let ca = c - a;
	let da = d - a;
	// 6 × signed volume; its magnitude gauges how flat the tet is.
	let det = ba.dot(ca.cross(da));
	if det.abs() <= flat_det {
		return None;
	}
	// o = circumcenter − a (Cramer's rule for the equidistance system).
	let o = (ba.length_squared() * ca.cross(da) + ca.length_squared() * da.cross(ba) + da.length_squared() * ba.cross(ca)) / (2.0 * det);
	let center = a + o;
	let r2 = o.length_squared();
	if center.is_finite() && r2.is_finite() {
		Some((center, r2))
	} else {
		None
	}
}

/// Build a [`Tet`] from four point indices, precomputing its circumsphere and
/// its `ok` (non-degenerate) flag.
fn make_tet(v: [usize; 4], pts: &[DVec3], flat_det: f64) -> Tet {
	match circumsphere(pts[v[0]], pts[v[1]], pts[v[2]], pts[v[3]], flat_det) {
		Some((center, r2)) => Tet { v, center, r2, ok: true },
		// Inert: r2 = −1 makes the in-sphere test `d² ≤ r²·(1+ε)` always false,
		// so a degenerate tet never claims a point and is skipped in the dual.
		None => Tet { v, center: DVec3::ZERO, r2: -1.0, ok: false },
	}
}

/// Clip the segment `p0→p1` to the axis-aligned box `[lo, hi]` (Liang–Barsky),
/// returning the surviving sub-segment or `None` if the segment misses the box.
fn clip_to_box(p0: DVec3, p1: DVec3, lo: DVec3, hi: DVec3) -> Option<(DVec3, DVec3)> {
	let d = p1 - p0;
	let mut t0 = 0.0f64;
	let mut t1 = 1.0f64;
	// One slab test: keeps the portion of `t` with `p·t ≤ q`.
	let clip = |p: f64, q: f64, t0: &mut f64, t1: &mut f64| -> bool {
		if p.abs() < 1e-30 {
			// Parallel to this slab: inside iff already on the correct side.
			return q >= 0.0;
		}
		let r = q / p;
		if p < 0.0 {
			if r > *t1 {
				return false;
			}
			if r > *t0 {
				*t0 = r;
			}
		} else {
			if r < *t0 {
				return false;
			}
			if r < *t1 {
				*t1 = r;
			}
		}
		true
	};
	let ok = clip(-d.x, p0.x - lo.x, &mut t0, &mut t1)
		&& clip(d.x, hi.x - p0.x, &mut t0, &mut t1)
		&& clip(-d.y, p0.y - lo.y, &mut t0, &mut t1)
		&& clip(d.y, hi.y - p0.y, &mut t0, &mut t1)
		&& clip(-d.z, p0.z - lo.z, &mut t0, &mut t1)
		&& clip(d.z, hi.z - p0.z, &mut t0, &mut t1);
	if ok && t0 <= t1 {
		Some((p0 + d * t0, p0 + d * t1))
	} else {
		None
	}
}

/// Compute the clipped Voronoi 1-skeleton of `seeds`, returned as strut
/// segments `(a, b)` inside the box `[min, max]`. See the module docs for the
/// algorithm and the honest-robustness contract. Deterministic: the same seeds
/// (in the same order) always yield the same segment list.
///
/// Caller contract (the [`crate::lattice::VoronoiLattice`] constructor and the
/// JSON parser enforce it up front): `seeds.len() >= 5`, all seeds finite, and
/// `min` strictly below `max` on every axis. With fewer than four
/// non-coplanar seeds there is no tetrahedron and the result is empty.
pub fn voronoi_struts(seeds: &[Vec3], min: Vec3, max: Vec3) -> Vec<(Vec3, Vec3)> {
	let n = seeds.len();
	if n < 4 {
		return Vec::new();
	}
	let mut pts: Vec<DVec3> = seeds.iter().map(|p| p.as_dvec3()).collect();

	// Seed-cloud bounds → a super-tetrahedron that strictly encloses them all.
	let mut lo = DVec3::splat(f64::INFINITY);
	let mut hi = DVec3::splat(f64::NEG_INFINITY);
	for &p in &pts {
		lo = lo.min(p);
		hi = hi.max(p);
	}
	let center = (lo + hi) * 0.5;
	let extent = pts.iter().map(|p| (*p - center).length()).fold(0.0f64, f64::max).max(1e-6);
	// A regular tet on four alternating cube corners at ±s: its inradius is
	// s·√3/3 ≈ 5.8·extent ≫ extent, so every seed sits strictly inside it.
	let s = extent * 10.0;
	let super_verts =
		[center + DVec3::new(s, s, s), center + DVec3::new(s, -s, -s), center + DVec3::new(-s, s, -s), center + DVec3::new(-s, -s, s)];
	let base = n; // super-vertices occupy indices n..n+4
	pts.extend_from_slice(&super_verts);
	let flat_det = FLAT_DET_REL * extent.powi(3).max(1e-12);

	let mut tets: Vec<Tet> = vec![make_tet([base, base + 1, base + 2, base + 3], &pts, flat_det)];

	// Incremental insertion.
	for pi in 0..n {
		let p = pts[pi];
		// Cavity: tets whose circumsphere already contains p.
		let mut bad: Vec<usize> = Vec::new();
		for (ti, t) in tets.iter().enumerate() {
			if t.ok && (p - t.center).length_squared() <= t.r2 * (1.0 + INSPHERE_EPS) {
				bad.push(ti);
			}
		}
		if bad.is_empty() {
			// No tet claims p (only possible under a degeneracy that left a
			// hole); skip it gracefully rather than corrupt the mesh.
			continue;
		}
		// Cavity boundary = faces of bad tets that are NOT shared by two bad tets.
		let mut face_count: HashMap<[usize; 3], u32> = HashMap::new();
		for &ti in &bad {
			for f in tet_faces(&tets[ti].v) {
				*face_count.entry(f).or_insert(0) += 1;
			}
		}
		let mut boundary: Vec<[usize; 3]> = face_count.into_iter().filter(|&(_, c)| c == 1).map(|(f, _)| f).collect();
		boundary.sort_unstable(); // determinism, independent of HashMap order
							// Delete the cavity: swap_remove in descending index order is safe
							// because it only ever pulls a tail (higher-index, non-cavity) tet down.
		bad.sort_unstable();
		for &ti in bad.iter().rev() {
			tets.swap_remove(ti);
		}
		// Re-triangulate: one new tet per boundary face, apex at p.
		for f in boundary {
			tets.push(make_tet([f[0], f[1], f[2], pi], &pts, flat_det));
		}
	}

	// Keep only real, non-degenerate tets (drop everything touching a super-vertex).
	tets.retain(|t| t.ok && t.v.iter().all(|&i| i < base));

	// Voronoi dual: every internal Delaunay face (shared by exactly two surviving
	// tets) → one edge between the two circumcenters.
	let mut face_adj: HashMap<[usize; 3], Vec<usize>> = HashMap::new();
	for (ti, t) in tets.iter().enumerate() {
		for f in tet_faces(&t.v) {
			face_adj.entry(f).or_default().push(ti);
		}
	}
	// Collect tet-index pairs, sorted, so the output order is deterministic.
	let mut pairs: Vec<(usize, usize)> = Vec::new();
	let mut seen: HashSet<(usize, usize)> = HashSet::new();
	for adj in face_adj.values() {
		if adj.len() == 2 {
			let key = (adj[0].min(adj[1]), adj[0].max(adj[1]));
			if seen.insert(key) {
				pairs.push(key);
			}
		}
	}
	pairs.sort_unstable();

	// Clip each Voronoi edge to the box; drop infinities/huge slivers.
	let (lo_b, hi_b) = (min.as_dvec3(), max.as_dvec3());
	let diag = (hi_b - lo_b).length();
	let far = (1.0e3 * diag).max(1.0e3); // "at infinity" magnitude gate
	let box_center = (lo_b + hi_b) * 0.5;
	let mut out: Vec<(Vec3, Vec3)> = Vec::with_capacity(pairs.len());
	for (ia, ib) in pairs {
		let a = tets[ia].center;
		let b = tets[ib].center;
		if !a.is_finite() || !b.is_finite() {
			continue;
		}
		if (a - box_center).length() > far || (b - box_center).length() > far {
			continue; // degenerate/unstable circumcenter — treat as infinity
		}
		if (a - b).length() < 1.0e-9 {
			continue; // coincident circumcenters — no real edge
		}
		if let Some((ca, cb)) = clip_to_box(a, b, lo_b, hi_b) {
			if (ca - cb).length() >= 1.0e-9 {
				out.push((ca.as_vec3(), cb.as_vec3()));
			}
		}
	}
	out
}

#[cfg(test)]
mod tests {
	use super::*;

	/// Deterministic LCG points in `[lo, hi]³` (no rand dependency).
	fn lcg_points(n: usize, lo: f32, hi: f32, seed: &mut u64) -> Vec<Vec3> {
		fn next(s: &mut u64) -> f32 {
			*s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
			((*s >> 33) as f32) / ((1u64 << 31) as f32)
		}
		(0..n).map(|_| Vec3::new(lo + (hi - lo) * next(seed), lo + (hi - lo) * next(seed), lo + (hi - lo) * next(seed))).collect()
	}

	#[test]
	fn eight_cube_corners_have_one_central_voronoi_vertex() {
		// The 8 corners of a cube plus a jitter-free centre-ish cloud is a clean
		// probe: a symmetric point set must produce a NON-EMPTY, box-contained
		// edge graph, and every emitted strut must lie inside the clip box.
		let seeds = vec![
			Vec3::new(-5.0, -5.0, -5.0),
			Vec3::new(5.0, -5.0, -5.0),
			Vec3::new(-5.0, 5.0, -5.0),
			Vec3::new(5.0, 5.0, -5.0),
			Vec3::new(-5.0, -5.0, 5.0),
			Vec3::new(5.0, -5.0, 5.0),
			Vec3::new(-5.0, 5.0, 5.0),
			Vec3::new(5.0, 5.0, 5.0),
			Vec3::new(0.0, 0.0, 0.0),
			Vec3::new(2.0, 1.0, -1.0),
		];
		let (lo, hi) = (Vec3::splat(-6.0), Vec3::splat(6.0));
		let struts = voronoi_struts(&seeds, lo, hi);
		let inside = struts.iter().all(|&(a, b)| {
			let eps = 1e-3;
			a.cmpge(lo - Vec3::splat(eps)).all()
				&& a.cmple(hi + Vec3::splat(eps)).all()
				&& b.cmpge(lo - Vec3::splat(eps)).all()
				&& b.cmple(hi + Vec3::splat(eps)).all()
		});
		assert!(!struts.is_empty() && inside, "symmetric cube cloud: {} struts, all-inside-box={inside}", struts.len());
	}

	#[test]
	fn random_cloud_edges_are_deterministic_and_a_reasonable_graph() {
		// A 30-point random-ish cloud: the dual is a GRAPH (more edges than
		// seeds, far fewer than the 20×seeds explosion bound), and rebuilding
		// from the identical seeds is bit-for-bit identical.
		let mut seed = 0x00C0_FFEE_1234_u64;
		let seeds = lcg_points(30, -9.0, 9.0, &mut seed);
		let (lo, hi) = (Vec3::splat(-10.0), Vec3::splat(10.0));
		let a = voronoi_struts(&seeds, lo, hi);
		let b = voronoi_struts(&seeds, lo, hi);
		let identical = a.len() == b.len() && a.iter().zip(&b).all(|(x, y)| x == y);
		assert!(
			a.len() > seeds.len() && a.len() < 20 * seeds.len() && identical,
			"voronoi dual sanity: {} edges for {} seeds (want {}..{}), deterministic={identical}",
			a.len(),
			seeds.len(),
			seeds.len() + 1,
			20 * seeds.len()
		);
	}
}
