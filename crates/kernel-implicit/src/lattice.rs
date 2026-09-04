// Copyright (c) LMCAD. Licensed under the MIT License.

//! Strut-based implicit primitives — the PicoGK-parity workhorses for
//! computational engineering (bionic lattices, conformal cooling, tube routing).
//!
//! A *strut* here is a **cone-capsule**: the convex hull of two end spheres
//! `(a, ra)` and `(b, rb)` (Inigo Quilez's exact "round cone"), so struts may
//! taper. [`BeamLattice`] is an arbitrary node/strut graph of them.
//!
//! # Lipschitz / exactness contract (load-bearing for narrow-band meshing)
//!
//! Each strut distance is the **exact Euclidean signed distance** to its convex
//! hull, hence 1-Lipschitz. The combined field is `min` over struts: a min of
//! 1-Lipschitz fields is 1-Lipschitz, and it equals the exact signed distance of
//! the union everywhere **outside** the union; inside an overlap of several
//! struts it can only *understate* the depth (|value| ≤ true distance), never
//! overstate it. Both properties keep the field safe for
//! [`crate::narrow_band`]'s Lipschitz block pruning, whose correctness requires
//! that a sampled value never overstates the distance to the field's zero set.
//!
//! # Acceleration
//!
//! `min` over `N` struts is `O(N)` per query, which makes a 10k-strut lattice
//! unmeshable. At construction a uniform spatial grid is built over the struts'
//! radius-inflated AABBs (the kernel-core BVH is triangle-specific, and a grid
//! is a better fit for ~equal-sized struts). A query expands Chebyshev rings of
//! cells around the query point and stops only when *no unvisited strut can
//! possibly be nearer than the best found* (see [`Struts::ring_lower_bound`]),
//! so the accelerated result equals the brute-force `min` to floating-point
//! rounding — acceleration never changes the field.

use std::cell::RefCell;
use std::collections::{HashMap, HashSet};

use kernel_core::math::{Aabb, DVec3, Vec3};
use kernel_core::sdf::Sdf;

thread_local! {
	/// Per-thread, epoch-stamped "strut already evaluated this query" scratch.
	/// A strut registered in several grid cells would otherwise be re-evaluated
	/// once per visited cell (a helix segment can span ~8 cells); the stamp
	/// makes each strut cost one distance evaluation per query. Thread-local so
	/// `distance(&self)` stays immutable and meshers can sample in parallel;
	/// epoch-stamping makes clearing O(1) per query.
	static VISITED_STAMPS: RefCell<(Vec<u32>, u32)> = const { RefCell::new((Vec::new(), 0)) };
}

/// One tapered strut: the convex hull of end spheres `(a, ra)` and `(b, rb)`.
#[derive(Clone, Copy, Debug)]
struct Strut {
	a: Vec3,
	b: Vec3,
	ra: f32,
	rb: f32,
}

impl Strut {
	/// Tight AABB of the strut (segment box inflated by the larger radius).
	fn aabb(&self) -> Aabb {
		Aabb::from_points(&[self.a, self.b]).pad(self.ra.max(self.rb))
	}

	/// Exact signed Euclidean distance to the convex hull of the two end
	/// spheres (after Inigo Quilez's `sdRoundCone`). With `ra == rb` this is
	/// algebraically the capsule distance. When one end sphere contains the
	/// other (`|ra − rb| ≥ |b − a|`, incl. zero-length axes) the hull *is* the
	/// larger sphere, and the sphere distance is returned — still exact.
	fn distance(&self, p: Vec3) -> f32 {
		let ba = self.b - self.a;
		let l2 = ba.dot(ba);
		let rr = self.ra - self.rb;
		let a2 = l2 - rr * rr;
		if a2 <= 0.0 || l2 < 1e-12 {
			return if self.ra >= self.rb { (p - self.a).length() - self.ra } else { (p - self.b).length() - self.rb };
		}
		let il2 = 1.0 / l2;
		let pa = p - self.a;
		let y = pa.dot(ba);
		let z = y - l2;
		let x2 = (pa * l2 - ba * y).length_squared();
		let y2 = y * y * l2;
		let z2 = z * z * l2;
		let k = rr.signum() * rr * rr * x2;
		if z.signum() * a2 * z2 > k {
			(x2 + z2).sqrt() * il2 - self.rb
		} else if y.signum() * a2 * y2 < k {
			(x2 + y2).sqrt() * il2 - self.ra
		} else {
			((x2 * a2 * il2).sqrt() + y * rr) * il2 - self.ra
		}
	}

	/// `f64` mirror of [`Strut::distance`] (same branch structure).
	fn distance64(&self, p: DVec3) -> f64 {
		let (a, b) = (self.a.as_dvec3(), self.b.as_dvec3());
		let (ra, rb) = (self.ra as f64, self.rb as f64);
		let ba = b - a;
		let l2 = ba.dot(ba);
		let rr = ra - rb;
		let a2 = l2 - rr * rr;
		if a2 <= 0.0 || l2 < 1e-12 {
			return if ra >= rb { (p - a).length() - ra } else { (p - b).length() - rb };
		}
		let il2 = 1.0 / l2;
		let pa = p - a;
		let y = pa.dot(ba);
		let z = y - l2;
		let x2 = (pa * l2 - ba * y).length_squared();
		let y2 = y * y * l2;
		let z2 = z * z * l2;
		let k = rr.signum() * rr * rr * x2;
		if z.signum() * a2 * z2 > k {
			(x2 + z2).sqrt() * il2 - rb
		} else if y.signum() * a2 * y2 < k {
			(x2 + y2).sqrt() * il2 - ra
		} else {
			((x2 * a2 * il2).sqrt() + y * rr) * il2 - ra
		}
	}
}

/// A set of struts with a uniform spatial grid for ~O(1) nearest-strut queries.
///
/// Cells store the indices of every strut whose inflated AABB overlaps them; a
/// strut may appear in several cells, so the f32 query deduplicates with the
/// thread-local [`VISITED_STAMPS`] scratch (each strut costs one evaluation per
/// query). The f64 query skips the dedup — `min` is idempotent so duplicates
/// only cost time, and that path is off the meshing hot loop.
struct Struts {
	struts: Vec<Strut>,
	/// Union of the inflated strut AABBs — every strut lies inside this box.
	bounds: Aabb,
	origin: Vec3,
	/// Cell edge length.
	h: f32,
	dims: [usize; 3],
	cells: Vec<Vec<u32>>,
}

impl Struts {
	/// Per-axis cell count ceiling — bounds grid memory for pathological inputs.
	const MAX_DIM: usize = 256;

	fn new(struts: Vec<Strut>) -> Self {
		let mut bounds = Aabb::empty();
		let mut mean_ext = 0.0f32;
		for s in &struts {
			let bb = s.aabb();
			bounds = bounds.union(bb);
			mean_ext += bb.size().max_element();
		}
		if struts.is_empty() || !bounds.is_valid() {
			return Self { struts, bounds: Aabb::empty(), origin: Vec3::ZERO, h: 1.0, dims: [1, 1, 1], cells: vec![Vec::new()] };
		}
		mean_ext /= struts.len() as f32;
		let size = bounds.size().max(Vec3::splat(1e-6));
		// Cell size: ~one strut per cell on average (volume / N), but no smaller
		// than half the mean strut extent (so a strut spans O(1) cells) and no
		// finer than MAX_DIM cells per axis (memory bound).
		let target = (size.x * size.y * size.z / struts.len().max(1) as f32).cbrt();
		let h = target.max(0.5 * mean_ext).max(size.max_element() / Self::MAX_DIM as f32).max(1e-6);
		let dims = [
			((size.x / h).ceil() as usize).clamp(1, Self::MAX_DIM),
			((size.y / h).ceil() as usize).clamp(1, Self::MAX_DIM),
			((size.z / h).ceil() as usize).clamp(1, Self::MAX_DIM),
		];
		let mut cells = vec![Vec::new(); dims[0] * dims[1] * dims[2]];
		let clampi = |v: f32, n: usize| (v.floor().max(0.0) as usize).min(n - 1);
		for (i, s) in struts.iter().enumerate() {
			let bb = s.aabb();
			let lo = (bb.min - bounds.min) / h;
			let hi = (bb.max - bounds.min) / h;
			let (x0, x1) = (clampi(lo.x, dims[0]), clampi(hi.x, dims[0]));
			let (y0, y1) = (clampi(lo.y, dims[1]), clampi(hi.y, dims[1]));
			let (z0, z1) = (clampi(lo.z, dims[2]), clampi(hi.z, dims[2]));
			for z in z0..=z1 {
				for y in y0..=y1 {
					for x in x0..=x1 {
						cells[x + dims[0] * (y + dims[1] * z)].push(i as u32);
					}
				}
			}
		}
		Self { struts, bounds, origin: bounds.min, h, dims, cells }
	}

	/// Grid cell containing the (clamped) point.
	#[inline]
	fn cell_of(&self, q: Vec3) -> [isize; 3] {
		let l = (q - self.origin) / self.h;
		[
			(l.x.floor().max(0.0) as usize).min(self.dims[0] - 1) as isize,
			(l.y.floor().max(0.0) as usize).min(self.dims[1] - 1) as isize,
			(l.z.floor().max(0.0) as usize).min(self.dims[2] - 1) as isize,
		]
	}

	/// Visit every in-range cell at Chebyshev ring `k` around `c0`, passing the
	/// cell's strut list and its world-space box.
	fn for_each_cell_in_ring(&self, c0: [isize; 3], k: isize, mut f: impl FnMut(&[u32], Aabb)) {
		let [nx, ny, nz] = [self.dims[0] as isize, self.dims[1] as isize, self.dims[2] as isize];
		let visit = |x: isize, y: isize, z: isize, f: &mut dyn FnMut(&[u32], Aabb)| {
			if x >= 0 && y >= 0 && z >= 0 && x < nx && y < ny && z < nz {
				let lo = self.origin + Vec3::new(x as f32, y as f32, z as f32) * self.h;
				f(&self.cells[(x + nx * (y + ny * z)) as usize], Aabb::new(lo, lo + Vec3::splat(self.h)));
			}
		};
		if k == 0 {
			visit(c0[0], c0[1], c0[2], &mut f);
			return;
		}
		for dz in -k..=k {
			for dy in -k..=k {
				if dz.abs() == k || dy.abs() == k {
					// Full rows on the z/y shell faces.
					for dx in -k..=k {
						visit(c0[0] + dx, c0[1] + dy, c0[2] + dz, &mut f);
					}
				} else {
					// Otherwise only the two x shell faces.
					visit(c0[0] - k, c0[1] + dy, c0[2] + dz, &mut f);
					visit(c0[0] + k, c0[1] + dy, c0[2] + dz, &mut f);
				}
			}
		}
	}

	/// Lower bound on the distance from `p` to ANY strut not yet visited when
	/// ring `k` is about to be expanded.
	///
	/// A strut first encountered at ring `k` has an inflated AABB disjoint from
	/// the box of rings `0..k`, so its distance from the clamped point `q` is at
	/// least `(k − 1)·h`. All struts lie inside [`Struts::bounds`], and `q` is
	/// the per-axis clamp of `p` onto that box, so for any strut point `x`:
	/// `|p − x|² ≥ |p − q|² + |q − x|²` (on clamped axes the two differences
	/// share a sign). Hence `dist(p, strut) ≥ √(dq² + ((k−1)h)²)`.
	#[inline]
	fn ring_lower_bound(&self, dq: f32, k: isize) -> f32 {
		let qk = ((k - 1).max(0) as f32) * self.h;
		(dq * dq + qk * qk).sqrt()
	}

	/// Exact `min` of strut distances at `p` via the ring search (f32).
	///
	/// Within a ring, a cell whose box is at least `best` away is skipped: any
	/// strut registered there is either also registered in (and evaluated at) a
	/// nearer cell, or lies entirely ≥ that box distance away — `best` only
	/// decreases, so a strut whose every cell was skipped satisfies
	/// `dist(p, strut) ≥ dist(p, its AABB) ≥ min over its cells' box distances
	/// ≥ best` and can never win. Pruning therefore never changes the result.
	fn distance(&self, p: Vec3) -> f32 {
		if self.struts.is_empty() {
			return f32::INFINITY;
		}
		let q = p.clamp(self.bounds.min, self.bounds.max);
		let dq = (p - q).length();
		let c0 = self.cell_of(q);
		let max_ring = *self.dims.iter().max().unwrap() as isize;
		VISITED_STAMPS.with(|scratch| {
			let (stamps, epoch) = &mut *scratch.borrow_mut();
			if stamps.len() < self.struts.len() {
				stamps.resize(self.struts.len(), 0);
			}
			*epoch = epoch.wrapping_add(1);
			if *epoch == 0 {
				stamps.fill(0);
				*epoch = 1;
			}
			let e = *epoch;
			let mut best = f32::INFINITY;
			for k in 0..=max_ring {
				if self.ring_lower_bound(dq, k) >= best {
					break;
				}
				self.for_each_cell_in_ring(c0, k, |cell, cell_box| {
					if cell.is_empty() || cell_box.distance_squared(p) >= best * best.max(0.0) {
						return;
					}
					for &si in cell {
						if stamps[si as usize] != e {
							stamps[si as usize] = e;
							best = best.min(self.struts[si as usize].distance(p));
						}
					}
				});
			}
			best
		})
	}

	/// Exact `min` of strut distances at `p` via the ring search (f64 struts;
	/// the ring geometry itself is the stored f32 grid, so the conservative
	/// stop bound is slackened by a whole cell to absorb f32 rounding).
	fn distance64(&self, p: DVec3) -> f64 {
		if self.struts.is_empty() {
			return f64::INFINITY;
		}
		let pf = p.as_vec3();
		let q = pf.clamp(self.bounds.min, self.bounds.max);
		let dq = (pf - q).length();
		let c0 = self.cell_of(q);
		let max_ring = *self.dims.iter().max().unwrap() as isize;
		let mut best = f64::INFINITY;
		for k in 0..=max_ring {
			if (self.ring_lower_bound(dq, k) - self.h) as f64 >= best {
				break;
			}
			self.for_each_cell_in_ring(c0, k, |cell, _| {
				for &si in cell {
					best = best.min(self.struts[si as usize].distance64(p));
				}
			});
		}
		best
	}
}

/// A beam lattice: a graph of `nodes` connected by tapered cone-capsule
/// `struts`, evaluated as the exact `min`-union of the strut distances with a
/// uniform-grid acceleration so queries stay ~O(1) at 10k+ struts (see the
/// module docs for the exactness/Lipschitz contract).
///
/// Construct from an explicit graph ([`BeamLattice::new`]) or fill a box with a
/// standard cell ([`BeamLattice::from_cells`]). A lattice is an [`Sdf`], so it
/// drops into any CSG [`Node`](crate::ops::Node) via `Node::primitive` —
/// intersect it with a shroud solid to shape it, or
/// [`fillet_union`](crate::features::fillet_union) it onto a skin.
///
/// **Extraction note (measured):** mesh junction-rich lattices with
/// [`manifold_dual_contour`](crate::manifold_dc::manifold_dual_contour). The
/// one-vertex-per-cell duals (narrow-band DC / Surface Nets) fold the saddle
/// cells where several strut creases meet at a node into non-manifold fins —
/// a structural limitation that does not vanish with refinement (see
/// `octet_block_meshes_watertight_with_sane_volume`). They remain useful as
/// the fast closed-volume route.
pub struct BeamLattice {
	nodes: Vec<Vec3>,
	struts: Struts,
}

/// Unit-cell topology for [`BeamLattice::from_cells`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LatticeCell {
	/// Struts along the 12 cube edges (bending-dominated, simple).
	Cubic,
	/// The octet truss (Deshpande et al.): cube corners + face centres,
	/// corner↔face-centre struts plus the inner octahedron edges — all of
	/// length `cell/√2`. Stretch-dominated, the standard structural lattice.
	Octet,
	/// Body-centred cubic: a node at the cell centre strutted to the 8 cube
	/// corners (an X through the cell, struts of length `cell·√3/2`). Cells share
	/// corners so the graph is connected; bending-dominated → compliant/springy.
	Bcc,
}

impl BeamLattice {
	/// Build from an explicit graph: `struts` are `(node_a, node_b, radius_a,
	/// radius_b)` — differing end radii make a tapered strut.
	///
	/// Contract (asserted): node indices in range, radii positive and finite,
	/// node positions finite. An empty strut list is allowed and yields an
	/// empty solid (distance `+∞`, invalid bounds — meshers reject it cleanly).
	pub fn new(nodes: Vec<Vec3>, struts: Vec<(u32, u32, f32, f32)>) -> Self {
		assert!(nodes.iter().all(|n| n.is_finite()), "BeamLattice: non-finite node position");
		let list: Vec<Strut> = struts
			.iter()
			.map(|&(ia, ib, ra, rb)| {
				assert!(
					(ia as usize) < nodes.len() && (ib as usize) < nodes.len(),
					"BeamLattice: strut ({ia}, {ib}) references a node out of range (have {})",
					nodes.len()
				);
				assert!(
					ra > 0.0 && rb > 0.0 && ra.is_finite() && rb.is_finite(),
					"BeamLattice: strut ({ia}, {ib}) has non-positive or non-finite radius ({ra}, {rb})"
				);
				Strut { a: nodes[ia as usize], b: nodes[ib as usize], ra, rb }
			})
			.collect();
		Self { nodes, struts: Struts::new(list) }
	}

	/// Fill `region` with a regular lattice of `cell` topology: whole cells of
	/// edge `cell_size` starting at `region.min` (`floor(size/cell_size)`, at
	/// least one, per axis — a region that is not a whole multiple is filled
	/// from `min` and the remainder is left empty). All struts get the uniform
	/// `radius`. Shared nodes and shared face struts are deduplicated, so the
	/// graph is a single connected lattice.
	pub fn from_cells(region: Aabb, cell: LatticeCell, cell_size: f32, radius: f32) -> Self {
		assert!(region.is_valid() && region.min.is_finite() && region.max.is_finite(), "BeamLattice::from_cells: invalid region");
		assert!(cell_size > 0.0 && cell_size.is_finite(), "BeamLattice::from_cells: cell_size must be positive");
		assert!(radius > 0.0 && radius.is_finite(), "BeamLattice::from_cells: radius must be positive");
		let size = region.size();
		let n = [
			((size.x / cell_size).floor() as usize).max(1),
			((size.y / cell_size).floor() as usize).max(1),
			((size.z / cell_size).floor() as usize).max(1),
		];

		// Nodes live on a half-cell lattice (doubled integer coordinates), so
		// cube corners (even coords) and face centres (mixed parity) share one
		// exact dedup key.
		let mut key_to_id: HashMap<(i64, i64, i64), u32> = HashMap::new();
		let mut nodes: Vec<Vec3> = Vec::new();
		let mut node_at = |key: (i64, i64, i64), nodes: &mut Vec<Vec3>| -> u32 {
			*key_to_id.entry(key).or_insert_with(|| {
				let p = region.min + Vec3::new(key.0 as f32, key.1 as f32, key.2 as f32) * (cell_size * 0.5);
				nodes.push(p);
				(nodes.len() - 1) as u32
			})
		};
		let mut seen: HashSet<(u32, u32)> = HashSet::new();
		let mut struts: Vec<(u32, u32, f32, f32)> = Vec::new();
		let mut push = |a: u32, b: u32, struts: &mut Vec<(u32, u32, f32, f32)>| {
			let key = (a.min(b), a.max(b));
			if a != b && seen.insert(key) {
				struts.push((a, b, radius, radius));
			}
		};

		for k in 0..n[2] {
			for j in 0..n[1] {
				for i in 0..n[0] {
					let (x, y, z) = (2 * i as i64, 2 * j as i64, 2 * k as i64);
					match cell {
						LatticeCell::Bcc => {
							// A cell-centre node strutted to the 8 cube corners (an X-cross).
							// Corners (even coords) are shared, so cells bond into one graph.
							let center = node_at((x + 1, y + 1, z + 1), &mut nodes);
							for c in [(0, 0, 0), (2, 0, 0), (0, 2, 0), (2, 2, 0), (0, 0, 2), (2, 0, 2), (0, 2, 2), (2, 2, 2)] {
								let nc = node_at((x + c.0, y + c.1, z + c.2), &mut nodes);
								push(center, nc, &mut struts);
							}
						}
						LatticeCell::Cubic => {
							// The 12 cube edges (shared edges dedup via `seen`).
							for (ca, cb) in [
								((0, 0, 0), (1, 0, 0)),
								((0, 1, 0), (1, 1, 0)),
								((0, 0, 1), (1, 0, 1)),
								((0, 1, 1), (1, 1, 1)),
								((0, 0, 0), (0, 1, 0)),
								((1, 0, 0), (1, 1, 0)),
								((0, 0, 1), (0, 1, 1)),
								((1, 0, 1), (1, 1, 1)),
								((0, 0, 0), (0, 0, 1)),
								((1, 0, 0), (1, 0, 1)),
								((0, 1, 0), (0, 1, 1)),
								((1, 1, 0), (1, 1, 1)),
							] {
								let na = node_at((x + 2 * ca.0, y + 2 * ca.1, z + 2 * ca.2), &mut nodes);
								let nb = node_at((x + 2 * cb.0, y + 2 * cb.1, z + 2 * cb.2), &mut nodes);
								push(na, nb, &mut struts);
							}
						}
						LatticeCell::Octet => {
							// Face centres ordered [x−, x+, y−, y+, z−, z+];
							// opposite faces are the consecutive pairs.
							let fc_keys = [
								(x, y + 1, z + 1),
								(x + 2, y + 1, z + 1),
								(x + 1, y, z + 1),
								(x + 1, y + 2, z + 1),
								(x + 1, y + 1, z),
								(x + 1, y + 1, z + 2),
							];
							let mut fc = [0u32; 6];
							for (id, &key) in fc.iter_mut().zip(fc_keys.iter()) {
								*id = node_at(key, &mut nodes);
							}
							// Each face centre to its 4 face corners.
							for (f, &(fx, fy, fz)) in fc_keys.iter().enumerate() {
								for s in 0..4u8 {
									let (s0, s1) = ((s & 1) as i64 * 2 - 1, ((s >> 1) & 1) as i64 * 2 - 1);
									let corner = match f / 2 {
										0 => (fx, fy + s0, fz + s1),
										1 => (fx + s0, fy, fz + s1),
										_ => (fx + s0, fy + s1, fz),
									};
									let nc = node_at(corner, &mut nodes);
									push(fc[f], nc, &mut struts);
								}
							}
							// The inner octahedron: every non-opposite face-centre pair.
							for a in 0..6usize {
								for b in (a + 1)..6 {
									if b != a + 1 || a % 2 != 0 {
										push(fc[a], fc[b], &mut struts);
									}
								}
							}
						}
					}
				}
			}
		}
		Self::new(nodes, struts)
	}

	/// Number of (deduplicated) lattice nodes.
	pub fn node_count(&self) -> usize {
		self.nodes.len()
	}

	/// Number of struts.
	pub fn strut_count(&self) -> usize {
		self.struts.struts.len()
	}

	/// Total naive strut volume: the sum of conical-frustum volumes
	/// `π·L/3·(ra² + ra·rb + rb²)` — ignores junction overlap (which removes
	/// material) and the spherical end caps (which add). A sanity yardstick,
	/// not a measurement.
	pub fn strut_volume_estimate(&self) -> f64 {
		self.struts
			.struts
			.iter()
			.map(|s| {
				let l = (s.b - s.a).length() as f64;
				let (ra, rb) = (s.ra as f64, s.rb as f64);
				std::f64::consts::PI * l / 3.0 * (ra * ra + ra * rb + rb * rb)
			})
			.sum()
	}
}

impl Sdf for BeamLattice {
	fn distance(&self, p: Vec3) -> f32 {
		self.struts.distance(p)
	}

	fn distance64(&self, p: DVec3) -> f64 {
		self.struts.distance64(p)
	}

	fn bounds(&self) -> Aabb {
		self.struts.bounds
	}
}

/// A tube swept along a polyline: each consecutive point pair becomes a
/// cone-capsule strut whose end radii are the per-vertex `radii`, so the wall
/// tapers linearly vertex-to-vertex (up to the sphere-tangency offset of the
/// hull — second order in the radius step, zero for constant radius). Shares
/// the strut grid and the exactness/Lipschitz contract of [`BeamLattice`]
/// (see the module docs).
///
/// This is the conformal-cooling-channel primitive: **subtract** a `Pipe` from
/// a wall to carve a channel, or union it for external tubing. A smooth pipe
/// has no strut junctions, so — unlike a junction-rich beam lattice — it also
/// meshes cleanly with the narrow-band marchers, not only Manifold DC.
pub struct Pipe {
	struts: Struts,
}

impl Pipe {
	/// Build from a polyline `path` (≥ 2 points) and per-vertex `radii` (same
	/// length, all positive and finite — asserted). Consecutive duplicate
	/// points are legal: the degenerate segment is its larger end sphere.
	pub fn new(path: Vec<Vec3>, radii: Vec<f32>) -> Self {
		assert!(path.len() >= 2, "Pipe: path needs at least 2 points, got {}", path.len());
		assert_eq!(path.len(), radii.len(), "Pipe: one radius per path vertex");
		assert!(path.iter().all(|p| p.is_finite()), "Pipe: non-finite path point");
		assert!(radii.iter().all(|r| *r > 0.0 && r.is_finite()), "Pipe: radii must be positive and finite");
		let struts = path.windows(2).zip(radii.windows(2)).map(|(p, r)| Strut { a: p[0], b: p[1], ra: r[0], rb: r[1] }).collect();
		Self { struts: Struts::new(struts) }
	}

	/// A circular helix around `axis`: starts at `center + r_helix·û` (û is a
	/// deterministic perpendicular of `axis`), advances `pitch` along the axis
	/// per turn for `turns` revolutions, with a constant tube `radius`.
	///
	/// The path is a polyline with `samples_per_turn` samples per turn (≥ 8 —
	/// asserted); its chord sagitta is `r_helix·(1 − cos(π/n))`, ≈ 0.5 % of
	/// `r_helix` at n = 32, so the tube wall deviation is negligible at the
	/// defaults used for cooling channels (n = 64). For a multi-start channel,
	/// rotate the node of a second helix about the axis.
	pub fn helix(center: Vec3, axis: Vec3, r_helix: f32, pitch: f32, turns: f32, samples_per_turn: usize, radius: f32) -> Self {
		assert!(
			r_helix > 0.0 && radius > 0.0 && turns > 0.0 && r_helix.is_finite() && pitch.is_finite() && turns.is_finite(),
			"Pipe::helix: r_helix, radius and turns must be positive and finite"
		);
		assert!(samples_per_turn >= 8, "Pipe::helix: need >= 8 samples per turn, got {samples_per_turn}");
		let w = axis.try_normalize().expect("Pipe::helix: axis must be non-zero");
		let u = w.any_orthonormal_vector();
		let v = w.cross(u);
		let segs = ((turns * samples_per_turn as f32).ceil() as usize).max(1);
		let path: Vec<Vec3> = (0..=segs)
			.map(|i| {
				let t = turns * i as f32 / segs as f32; // position in turns
				let a = t * std::f32::consts::TAU;
				center + u * (r_helix * a.cos()) + v * (r_helix * a.sin()) + w * (pitch * t)
			})
			.collect();
		let n = path.len();
		Self::new(path, vec![radius; n])
	}

	/// Number of polyline segments.
	pub fn segment_count(&self) -> usize {
		self.struts.struts.len()
	}

	/// Analytic tube volume: per-segment conical frustums plus the two
	/// hemispherical end caps. By the 3-D tube formula a constant-radius tube
	/// around a non-self-intersecting curve has volume exactly `π·r²·L`
	/// (the curvature terms vanish in odd dimension), so for smooth polylines
	/// (shallow joint angles) this estimate is sub-percent accurate; sharp
	/// kinks add small uncounted joint wedges. Used to verify carved cooling
	/// channels are really hollow.
	pub fn volume_estimate(&self) -> f64 {
		let s = &self.struts.struts;
		let frustums: f64 = s
			.iter()
			.map(|s| {
				let l = (s.b - s.a).length() as f64;
				let (ra, rb) = (s.ra as f64, s.rb as f64);
				std::f64::consts::PI * l / 3.0 * (ra * ra + ra * rb + rb * rb)
			})
			.sum();
		let caps = 2.0 / 3.0 * std::f64::consts::PI * ((s[0].ra as f64).powi(3) + (s[s.len() - 1].rb as f64).powi(3));
		frustums + caps
	}
}

impl Sdf for Pipe {
	fn distance(&self, p: Vec3) -> f32 {
		self.struts.distance(p)
	}

	fn distance64(&self, p: DVec3) -> f64 {
		self.struts.distance64(p)
	}

	fn bounds(&self) -> Aabb {
		self.struts.bounds
	}
}

/// A 3-D Voronoi open-cell foam: the 1-skeleton of the Voronoi diagram of a
/// cloud of seed points, swept as a uniform-radius strut network. Unlike
/// [`BeamLattice`] (which is handed an explicit graph) this computes the Voronoi
/// edge graph **natively in Rust** from just the seeds — no external Delaunay
/// library — via an incremental Bowyer–Watson tetrahedralization (see
/// [`crate::voronoi`]). The resulting struts are evaluated identically to a
/// [`BeamLattice`]: the exact `min`-union of cone-capsule distances over the
/// same accelerated strut grid, so the whole Lipschitz/exactness contract in
/// the module docs carries over unchanged.
///
/// The Voronoi cells are unbounded, so the edge graph is **clipped to the
/// `[min, max]` box** at construction; edges running to infinity (convex-hull
/// faces) and numerically unstable slivers are dropped. This is a lattice
/// generator, NOT the exact-predicate boolean pipeline: the in-circumsphere
/// test is `f64`-approximate (honest note in [`crate::voronoi`]), which is the
/// right trade for a strut graph — for seeds in general position it is a moot
/// point, and a mis-classified degeneracy perturbs an edge rather than
/// corrupting a solid.
///
/// Mesh it exactly like a beam lattice: junction-rich, so extract with
/// [`manifold_dual_contour`](crate::manifold_dc::manifold_dual_contour), and
/// intersect it with a shroud solid (a ball, a shell) to shape the foam.
pub struct VoronoiLattice {
	seed_count: usize,
	struts: Struts,
}

impl VoronoiLattice {
	/// Build the clipped Voronoi foam of `seeds` with uniform strut `radius`,
	/// clipped to the box `[min, max]`.
	///
	/// Contract (asserted): at least 5 finite seeds (four define a single tet —
	/// a foam needs more), a positive finite radius, and `min` strictly below
	/// `max` on every axis. An all-coplanar seed set yields no tetrahedra and
	/// hence an empty foam (distance `+∞`, invalid bounds — meshers reject it
	/// cleanly), which is honest rather than a panic.
	pub fn new(seeds: Vec<Vec3>, radius: f32, min: Vec3, max: Vec3) -> Self {
		assert!(seeds.len() >= 5, "VoronoiLattice: need at least 5 seed points, got {}", seeds.len());
		assert!(seeds.iter().all(|s| s.is_finite()), "VoronoiLattice: non-finite seed position");
		assert!(radius > 0.0 && radius.is_finite(), "VoronoiLattice: radius must be positive and finite, got {radius}");
		assert!(
			min.is_finite() && max.is_finite() && min.x < max.x && min.y < max.y && min.z < max.z,
			"VoronoiLattice: 'min' {min:?} must be finite and strictly below 'max' {max:?} on every axis"
		);
		let segments = crate::voronoi::voronoi_struts(&seeds, min, max);
		let list: Vec<Strut> = segments.iter().map(|&(a, b)| Strut { a, b, ra: radius, rb: radius }).collect();
		Self { seed_count: seeds.len(), struts: Struts::new(list) }
	}

	/// Number of seed generator points the foam was built from.
	pub fn seed_count(&self) -> usize {
		self.seed_count
	}

	/// Number of (clipped) Voronoi-edge struts in the foam.
	pub fn strut_count(&self) -> usize {
		self.struts.struts.len()
	}
}

impl Sdf for VoronoiLattice {
	fn distance(&self, p: Vec3) -> f32 {
		self.struts.distance(p)
	}

	fn distance64(&self, p: DVec3) -> f64 {
		self.struts.distance64(p)
	}

	fn bounds(&self) -> Aabb {
		self.struts.bounds
	}
}

#[cfg(test)]
mod tests {
	use super::*;
	use crate::narrow_band::dual_contour_narrowband;
	use crate::ops::Node;
	use crate::primitives::{Capsule, Cuboid, Sphere};
	use kernel_core::check_mesh;
	use kernel_core::mesher::Resolution;

	/// Deterministic LCG points in `bounds` padded by `pad` (no rand dep).
	fn lcg_points(n: usize, bounds: Aabb, pad: f32, seed: &mut u64) -> Vec<Vec3> {
		fn next(s: &mut u64) -> f32 {
			*s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
			((*s >> 33) as f32) / ((1u64 << 31) as f32)
		}
		let (lo, hi) = (bounds.min - Vec3::splat(pad), bounds.max + Vec3::splat(pad));
		(0..n)
			.map(|_| {
				let t = Vec3::new(next(seed), next(seed), next(seed));
				lo + (hi - lo) * t
			})
			.collect()
	}

	#[test]
	fn single_strut_equals_capsule_distance() {
		// Equal end radii make the cone-capsule algebraically a capsule; the
		// lattice (through its grid) must reproduce the exact capsule distance
		// at probes inside, outside, beyond the caps and FAR away (the far
		// probes exercise the ring-search termination path).
		let (a, b, r) = (Vec3::new(-4.0, 1.0, 0.5), Vec3::new(5.0, -2.0, 3.0), 1.25);
		let lat = BeamLattice::new(vec![a, b], vec![(0, 1, r, r)]);
		let cap = Capsule::new(a, b, r);
		for p in [
			Vec3::new(0.0, 0.0, 1.0),       // near the middle, inside
			Vec3::new(0.6, -0.4, 1.7),      // on-axis-ish
			Vec3::new(8.0, -4.0, 5.0),      // beyond the b cap
			Vec3::new(-9.0, 6.0, -2.0),     // beyond the a cap
			Vec3::new(0.0, 30.0, 0.0),      // far lateral
			Vec3::new(-200.0, 150.0, 90.0), // very far (termination bound)
		] {
			let (got, want) = (lat.distance(p), cap.distance(p));
			assert!((got - want).abs() < 1e-6, "lattice vs capsule at {p:?}: {got} vs {want}");
			let got64 = lat.distance64(p.as_dvec3());
			assert!((got64 - want as f64).abs() < 1e-4, "lattice f64 vs capsule at {p:?}: {got64} vs {want}");
		}
	}

	#[test]
	fn tapered_strut_is_hull_of_end_spheres() {
		// A strongly tapered strut: beyond each end the distance must be the
		// exact end-sphere distance (the hull's caps ARE the spheres), and a
		// degenerate strut (one sphere inside the other) must collapse to the
		// larger sphere everywhere.
		let (a, b) = (Vec3::ZERO, Vec3::new(10.0, 0.0, 0.0));
		let lat = BeamLattice::new(vec![a, b], vec![(0, 1, 3.0, 1.0)]);
		let pa = Vec3::new(-5.0, 0.0, 0.0);
		let pb = Vec3::new(14.0, 0.0, 0.0);
		assert!((lat.distance(pa) - ((pa - a).length() - 3.0)).abs() < 1e-6, "a-cap is the r=3 sphere");
		assert!((lat.distance(pb) - ((pb - b).length() - 1.0)).abs() < 1e-6, "b-cap is the r=1 sphere");
		// Lateral mid probe: the hull wall slope is sin θ = (ra−rb)/|ba|, so at
		// the midpoint the wall radius is the lerp of radii MINUS the tangency
		// shift; assert against the closed form r(t) = lerp − d·sinθ·(…): use
		// the plane tangent to both spheres: distance from axis point q to the
		// wall = (lerp_r − (q−mid)·…) — simplest exact check: the point on the
		// tangent line between tangency circles must be ON the surface.
		let sin_t = (3.0f32 - 1.0) / 10.0;
		let cos_t = (1.0 - sin_t * sin_t).sqrt();
		// Tangency points of the upper tangent line in the xz=0, y>0 half-plane:
		let ta = a + Vec3::new(3.0 * sin_t, 3.0 * cos_t, 0.0);
		let tb = b + Vec3::new(1.0 * sin_t, 1.0 * cos_t, 0.0);
		let on_wall = (ta + tb) * 0.5;
		assert!(lat.distance(on_wall).abs() < 1e-5, "tangent-line midpoint must lie on the hull surface, got {}", lat.distance(on_wall));
		let degen = BeamLattice::new(vec![Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0)], vec![(0, 1, 5.0, 1.0)]);
		let p = Vec3::new(7.0, 2.0, -1.0);
		assert!((degen.distance(p) - (p.length() - 5.0)).abs() < 1e-6, "contained end sphere: hull is the larger sphere");
	}

	#[test]
	fn from_cells_builds_the_expected_graph() {
		// 2×2×2 cubic cells: (2+1)³ = 27 shared nodes, 3 axes · 2·3·3 = 54
		// unique edge struts. Octet on one cell: 8 corners + 6 face centres =
		// 14 nodes, 24 corner↔face + 12 octahedron = 36 struts.
		let region = Aabb::new(Vec3::ZERO, Vec3::splat(20.0));
		let cubic = BeamLattice::from_cells(region, LatticeCell::Cubic, 10.0, 1.0);
		assert_eq!((cubic.node_count(), cubic.strut_count()), (27, 54), "cubic 2³ graph");
		let one = Aabb::new(Vec3::ZERO, Vec3::splat(10.0));
		let octet = BeamLattice::from_cells(one, LatticeCell::Octet, 10.0, 1.0);
		assert_eq!((octet.node_count(), octet.strut_count()), (14, 36), "octet single-cell graph");
		// All octet struts have length cell/√2.
		let want = 10.0 / 2.0f32.sqrt();
		for s in &octet.struts.struts {
			assert!(((s.b - s.a).length() - want).abs() < 1e-4, "octet strut length {} != {want}", (s.b - s.a).length());
		}
		// BCC on one cell: 8 corners + 1 centre = 9 nodes, 8 centre↔corner struts of
		// length cell·√3/2; cells share corners, so multi-cell blocks stay connected.
		let bcc = BeamLattice::from_cells(one, LatticeCell::Bcc, 10.0, 1.0);
		assert_eq!((bcc.node_count(), bcc.strut_count()), (9, 8), "bcc single-cell graph");
		let want_bcc = 10.0 * 3.0f32.sqrt() / 2.0;
		for s in &bcc.struts.struts {
			assert!(((s.b - s.a).length() - want_bcc).abs() < 1e-4, "bcc strut length {} != {want_bcc}", (s.b - s.a).length());
		}
	}

	#[test]
	fn octet_block_meshes_watertight_with_sane_volume() {
		// The signature workflow: an octet block at adequate resolution (voxel
		// ≈ radius/4) meshes FULLY WATERTIGHT — zero non-manifold and zero
		// boundary edges — via Manifold Dual Contouring, with a volume in a
		// sane band around the naive strut-volume sum (junction overlap removes
		// material, node-ball rounding adds a little; measured ratio here is
		// ~0.65 — the band is a sanity gate, not a precision claim).
		//
		// HONEST mesher note (measured, not assumed): the one-vertex-per-cell
		// duals (narrow-band DC / Surface Nets) fold the saddle cells where
		// many strut creases meet at a junction into non-manifold fins — 1212
		// nme at 0.35 mm, GROWING to 8556 at 0.2 mm (structural, not an
		// under-resolution artefact; `make_manifold` cannot snip them apart
		// either). `manifold_dual_contour` resolves those cells by placing one
		// vertex per surface patch and is clean at EVERY tested resolution —
		// it is the designated extraction path for junction-rich lattices. The
		// narrow-band march remains valuable as the fast volume/closure route:
		// it must agree on volume and stay hole-free (boundary edges = 0).
		let region = Aabb::new(Vec3::ZERO, Vec3::splat(20.0));
		let lat = BeamLattice::from_cells(region, LatticeCell::Octet, 10.0, 1.4);
		let naive = lat.strut_volume_estimate();
		let domain = lat.bounds().pad(1.0);

		let mesh = crate::manifold_dual_contour(&lat, domain, Resolution::VoxelSize(0.35));
		let r = check_mesh(&mesh);
		let vol = mesh.signed_volume();
		assert!(
			mesh.is_watertight() && r.non_manifold_edges == 0 && r.boundary_edges == 0 && vol > 0.5 * naive && vol < 1.05 * naive,
			"octet block (MDC): watertight={} nme={} bnd={} tris={} vol={vol:.1} naive={naive:.1} (ratio {:.3})",
			mesh.is_watertight(),
			r.non_manifold_edges,
			r.boundary_edges,
			mesh.triangle_count(),
			vol / naive
		);

		let nb = dual_contour_narrowband(&lat, domain, Resolution::VoxelSize(0.35));
		let rnb = check_mesh(&nb);
		let nbv = nb.signed_volume();
		assert!(
			rnb.boundary_edges == 0 && (nbv - vol).abs() / vol < 0.01,
			"narrow-band DC must be closed with matching volume: bnd={} vol={nbv:.1} vs MDC {vol:.1}",
			rnb.boundary_edges
		);
	}

	#[test]
	fn grid_query_is_exact_and_fast_at_5k_struts() {
		// Exactness oracle: the accelerated query must equal the brute-force
		// min over ALL struts (to f32 rounding) at random probes in AND far
		// outside the bounds — this falsifies any wrong ring-termination bound.
		// Cost: 1e5 queries against ~5.6k struts must stay far under a
		// brute-force budget (~5×10⁸ cone evaluations); the wall-time gate is
		// generous to absorb CI noise and is enforced in release builds.
		let region = Aabb::new(Vec3::ZERO, Vec3::splat(48.0));
		let lat = BeamLattice::from_cells(region, LatticeCell::Octet, 8.0, 0.8);
		assert!(lat.strut_count() > 5000, "want >5k struts, have {}", lat.strut_count());

		let mut seed = 0x5eed_1a77_1ceb_eef0_u64;
		for p in lcg_points(200, lat.bounds(), 40.0, &mut seed) {
			let brute = lat.struts.struts.iter().map(|s| s.distance(p)).fold(f32::INFINITY, f32::min);
			let got = lat.distance(p);
			assert!((got - brute).abs() <= 1e-6 * brute.abs().max(1.0), "grid {got} != brute {brute} at {p:?}");
		}

		let queries = lcg_points(100_000, lat.bounds(), 2.0, &mut seed);
		let t0 = std::time::Instant::now();
		let mut acc = 0.0f32;
		for &p in &queries {
			acc += lat.distance(p);
		}
		let dt = t0.elapsed();
		println!("5.6k-strut lattice: 1e5 queries in {dt:?} (checksum {acc})");
		assert!(acc.is_finite());
		if cfg!(not(debug_assertions)) {
			assert!(dt.as_secs_f64() < 10.0, "1e5 queries took {dt:?}; grid acceleration is not O(1) per query");
		}
	}

	#[test]
	fn straight_pipe_equals_capsule_distance() {
		// A 3-point COLLINEAR constant-radius pipe is exactly one capsule —
		// the accelerated min over its two segments must reproduce the closed
		// form at probes inside, near the interior joint, beyond the ends and
		// far away.
		let (a, m, b, r) = (Vec3::new(-6.0, 0.0, 1.0), Vec3::new(-1.0, 0.0, 1.0), Vec3::new(8.0, 0.0, 1.0), 1.5);
		let pipe = Pipe::new(vec![a, m, b], vec![r, r, r]);
		let cap = Capsule::new(a, b, r);
		for p in [Vec3::new(0.0, 0.0, 1.0), Vec3::new(-1.0, 0.2, 2.0), Vec3::new(12.0, 3.0, -1.0), Vec3::new(-40.0, 25.0, 60.0)] {
			let (got, want) = (pipe.distance(p), cap.distance(p));
			assert!((got - want).abs() < 1e-6, "pipe vs capsule at {p:?}: {got} vs {want}");
			let got64 = pipe.distance64(p.as_dvec3());
			assert!((got64 - want as f64).abs() < 1e-4, "pipe f64 vs capsule at {p:?}: {got64} vs {want}");
		}
	}

	#[test]
	fn helical_pipe_meshes_watertight() {
		// A smooth tube has no strut junctions, so narrow-band dual contouring
		// must mesh it fully watertight (nme = 0, no boundary). The helix must
		// also span the expected envelope (radial r_helix + r, axial 0 …
		// pitch·turns ± r) and its meshed volume must match the analytic tube
		// formula estimate.
		let pipe = Pipe::helix(Vec3::ZERO, Vec3::Z, 8.0, 6.0, 3.0, 64, 1.5);
		let bb = pipe.bounds();
		let mesh = dual_contour_narrowband(&pipe, bb.pad(1.0), Resolution::VoxelSize(0.25));
		let r = check_mesh(&mesh);
		assert!(
			mesh.is_watertight() && r.non_manifold_edges == 0 && r.boundary_edges == 0,
			"helix pipe: watertight={} nme={} bnd={}",
			mesh.is_watertight(),
			r.non_manifold_edges,
			r.boundary_edges
		);
		assert!(
			(bb.min.z + 1.5).abs() < 1e-4 && (bb.max.z - 19.5).abs() < 1e-4 && (bb.max.x - 9.5).abs() < 0.05,
			"helix envelope wrong: {bb:?}"
		);
		let est = pipe.volume_estimate();
		let vol = mesh.signed_volume();
		assert!((vol - est).abs() / est < 0.02, "helix volume {vol:.1} vs analytic estimate {est:.1}");
	}

	#[test]
	fn block_minus_helical_pipe_removes_pipe_volume() {
		// The conformal-cooling workflow: carve a fully-embedded helical
		// channel out of a block. The carved solid must stay watertight and
		// the removed material must equal the analytic pipe volume (tube
		// formula + end caps) within a documented 5 % — both meshes share the
		// same grid so the outer-surface quantization error cancels in the
		// difference.
		let pipe = Pipe::helix(Vec3::new(15.0, 15.0, 4.0), Vec3::Z, 8.0, 6.0, 3.0, 64, 1.5);
		let est = pipe.volume_estimate();
		let block = || Node::primitive(Cuboid::new(Vec3::new(15.0, 15.0, 13.0), Vec3::new(15.0, 15.0, 13.0)));
		let domain = block().bounds().pad(1.0);

		let solid = dual_contour_narrowband(&block(), domain, Resolution::VoxelSize(0.25));
		let carved_node = block().difference(Node::primitive(pipe));
		let carved = dual_contour_narrowband(&carved_node, domain, Resolution::VoxelSize(0.25));
		let removed = solid.signed_volume() - carved.signed_volume();
		// `is_watertight` covers boundary AND non-manifold edges (every edge
		// used exactly twice); the full `check_mesh` self-intersection scan is
		// skipped here — it costs ~10 s on this 200k-tri mesh and is not what
		// this test gates.
		assert!(
			carved.is_watertight() && (removed - est).abs() / est < 0.05,
			"carved block: watertight={} removed={removed:.1} vs pipe estimate {est:.1} ({:+.1}%)",
			carved.is_watertight(),
			(removed / est - 1.0) * 100.0
		);
	}

	#[test]
	#[should_panic(expected = "one radius per path vertex")]
	fn pipe_rejects_mismatched_radii() {
		let _ = Pipe::new(vec![Vec3::ZERO, Vec3::X], vec![1.0]);
	}

	#[test]
	fn voronoi_lattice_foam_ball_meshes_watertight() {
		// The payoff: ~30 FIXED seeds (deterministic LCG — no rand/Date at test
		// time) in a cube feed the NATIVE Voronoi generator (in-kernel
		// Bowyer–Watson, zero scipy). The 1-skeleton must be a real GRAPH — more
		// edges than seeds, well under the 20×seeds explosion bound — and,
		// intersected with a solid ball and extracted by Manifold Dual Contouring
		// (the designated junction-rich path), a valid, fully WATERTIGHT foam
		// ball with positive volume.
		//
		// HONEST mesher note (measured): as with EVERY strut lattice, a saddle
		// cell where several Voronoi struts meet at a junction can leave MDC one
		// non-manifold fin (here nme = 1 at voxel 0.3 — a structural artefact
		// that does not reliably vanish with refinement, same class as the octet
		// junction fins). The production `run_program` pipeline snips those with
		// `make_manifold`; this test runs that identical two-step path and
		// asserts the RESULT is fully watertight (nme = 0, bnd = 0).
		let mut seed = 0x5EED_F0A3_u64;
		let seeds = lcg_points(30, Aabb::new(Vec3::splat(-10.0), Vec3::splat(10.0)), 0.0, &mut seed);
		let foam = VoronoiLattice::new(seeds, 1.0, Vec3::splat(-12.0), Vec3::splat(12.0));
		assert!(
			foam.strut_count() > foam.seed_count() && foam.strut_count() < 20 * foam.seed_count(),
			"voronoi dual must be a graph: {} struts for {} seeds (want {}..{})",
			foam.strut_count(),
			foam.seed_count(),
			foam.seed_count() + 1,
			20 * foam.seed_count()
		);
		let ball = Node::primitive(foam).intersection(Node::primitive(Sphere::new(Vec3::ZERO, 9.0)));
		let domain = Aabb::new(Vec3::splat(-9.5), Vec3::splat(9.5));
		let raw = crate::manifold_dual_contour(&ball, domain, Resolution::VoxelSize(0.3));
		let raw_tris = raw.triangle_count();
		let mut mesh = raw;
		if !mesh.is_watertight() || check_mesh(&mesh).non_manifold_edges > 0 {
			mesh = kernel_core::make_manifold(&mesh);
		}
		let r = check_mesh(&mesh);
		assert!(
			raw_tris > 0
				&& mesh.triangle_count() > 0
				&& mesh.is_watertight()
				&& r.non_manifold_edges == 0
				&& r.boundary_edges == 0
				&& mesh.signed_volume() > 0.0,
			"voronoi foam ball (MDC+heal): raw_tris={raw_tris} tris={} watertight={} nme={} bnd={} vol={:.1}",
			mesh.triangle_count(),
			mesh.is_watertight(),
			r.non_manifold_edges,
			r.boundary_edges,
			mesh.signed_volume()
		);
	}

	#[test]
	fn voronoi_lattice_strut_count_is_deterministic() {
		// Same seeds (same LCG stream) → identical strut count, twice. Guards the
		// generator against HashMap iteration-order nondeterminism (the sort in
		// `voronoi::voronoi_struts` is what makes this hold).
		let mut s1 = 0xABCD_1234_5678_u64;
		let mut s2 = 0xABCD_1234_5678_u64;
		let region = Aabb::new(Vec3::splat(-8.0), Vec3::splat(8.0));
		let (lo, hi) = (Vec3::splat(-10.0), Vec3::splat(10.0));
		let a = VoronoiLattice::new(lcg_points(40, region, 0.0, &mut s1), 0.6, lo, hi);
		let b = VoronoiLattice::new(lcg_points(40, region, 0.0, &mut s2), 0.6, lo, hi);
		assert!(
			a.strut_count() == b.strut_count() && a.strut_count() > 0,
			"identical seeds must give identical strut count: {} vs {}",
			a.strut_count(),
			b.strut_count()
		);
	}
}
