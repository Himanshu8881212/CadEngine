// Copyright (c) LMCAD. Licensed under the MIT License.

//! A median-split AABB bounding-volume hierarchy over a mesh's triangles, for
//! fast *repeated* ray and closest-point queries — `O(log n)` typical versus the
//! `O(n)` brute force of [`Mesh::raycast`] / [`Mesh::closest_point`]. Build it
//! once with [`Mesh::build_bvh`], then query many times (picking, snapping,
//! measurement in an interactive viewer). Results are identical to the
//! brute-force methods; only the cost differs.

use crate::math::{Aabb, Ray, Vec3};
use crate::mesh::{closest_point_on_triangle, ray_triangle, triangle_triangle_distance, ClosestPoint, Mesh, RayHit};

/// Maximum triangles in a leaf.
const LEAF: usize = 4;

/// One node: a leaf owns a `[start, start + count)` slice of the triangle array;
/// an internal node owns two child node indices `(left, right)`.
struct Node {
	bounds: Aabb,
	a: u32, // leaf: start triangle slot;  internal: left child index
	b: u32, // leaf: triangle count;       internal: right child index
	leaf: bool,
}

/// A bounding-volume hierarchy built over the triangles of a [`Mesh`].
pub struct MeshBvh {
	tris: Vec<[Vec3; 3]>, // triangles in leaf order
	ids: Vec<usize>,      // original mesh-triangle index per slot
	nodes: Vec<Node>,
}

impl Mesh {
	/// Build a [`MeshBvh`] for fast repeated ray / closest-point queries. The cost
	/// is `O(n log n)` once; amortize it across many queries.
	pub fn build_bvh(&self) -> MeshBvh {
		MeshBvh::new(self)
	}
}

impl MeshBvh {
	fn new(mesh: &Mesh) -> Self {
		let tris: Vec<[Vec3; 3]> = mesh
			.indices
			.chunks_exact(3)
			.map(|t| [mesh.positions[t[0] as usize], mesh.positions[t[1] as usize], mesh.positions[t[2] as usize]])
			.collect();
		let mut order: Vec<usize> = (0..tris.len()).collect();
		let mut nodes = Vec::new();
		if !tris.is_empty() {
			build(&tris, &mut order, 0, tris.len(), &mut nodes);
		}
		// Reorder so each leaf's `[start, start+count)` indexes contiguous triangles.
		let ordered: Vec<[Vec3; 3]> = order.iter().map(|&i| tris[i]).collect();
		MeshBvh { tris: ordered, ids: order, nodes }
	}

	/// Nearest forward intersection of `ray` with the surface, or `None`.
	/// Identical result to [`Mesh::raycast`], but pruned by the hierarchy.
	pub fn raycast(&self, ray: Ray) -> Option<RayHit> {
		if self.nodes.is_empty() {
			return None;
		}
		let mut best: Option<RayHit> = None;
		let mut stack = vec![0u32];
		while let Some(ni) = stack.pop() {
			let node = &self.nodes[ni as usize];
			let tmax = best.map_or(f32::INFINITY, |h| h.t);
			if !node.bounds.ray_hits(ray, tmax) {
				continue;
			}
			if node.leaf {
				for s in node.a..node.a + node.b {
					if let Some((t, pt, nrm)) =
						ray_triangle(ray, self.tris[s as usize][0], self.tris[s as usize][1], self.tris[s as usize][2])
					{
						if best.is_none_or(|h| t < h.t) {
							best = Some(RayHit { t, point: pt, normal: nrm, triangle: self.ids[s as usize] });
						}
					}
				}
			} else {
				stack.push(node.a);
				stack.push(node.b);
			}
		}
		best
	}

	/// Nearest point on the surface to `query`, or `None`. Identical result to
	/// [`Mesh::closest_point`], but pruned by the hierarchy (children visited
	/// nearest-first so the bound tightens early).
	pub fn closest_point(&self, query: Vec3) -> Option<ClosestPoint> {
		if self.nodes.is_empty() {
			return None;
		}
		let mut best: Option<ClosestPoint> = None;
		let mut stack = vec![0u32];
		while let Some(ni) = stack.pop() {
			let node = &self.nodes[ni as usize];
			let bound = best.map_or(f32::INFINITY, |c| c.distance);
			if node.bounds.distance_squared(query) >= bound * bound {
				continue;
			}
			if node.leaf {
				for s in node.a..node.a + node.b {
					let t = self.tris[s as usize];
					let cp = closest_point_on_triangle(query, t[0], t[1], t[2]);
					let d = (cp - query).length();
					if best.is_none_or(|c| d < c.distance) {
						best = Some(ClosestPoint { point: cp, distance: d, triangle: self.ids[s as usize] });
					}
				}
			} else {
				// Push the farther child first so the nearer is popped (visited) first.
				let dl = self.nodes[node.a as usize].bounds.distance_squared(query);
				let dr = self.nodes[node.b as usize].bounds.distance_squared(query);
				if dl < dr {
					stack.push(node.b);
					stack.push(node.a);
				} else {
					stack.push(node.a);
					stack.push(node.b);
				}
			}
		}
		best
	}
}

impl MeshBvh {
	/// Count the forward crossings of `ray` with the surface (every triangle hit
	/// with `t > 0`, not just the nearest). For a *closed* mesh an odd count means
	/// the ray origin is inside the solid — the parity primitive behind
	/// [`contains_point`](Self::contains_point). Grazing hits exactly on a shared
	/// edge or vertex can be double-counted; cast several differently-oriented
	/// rays and vote if a single ray's parity would be load-bearing.
	pub fn ray_crossings(&self, ray: Ray) -> usize {
		if self.nodes.is_empty() {
			return 0;
		}
		let mut count = 0usize;
		let mut stack = vec![0u32];
		while let Some(ni) = stack.pop() {
			let node = &self.nodes[ni as usize];
			if !node.bounds.ray_hits(ray, f32::INFINITY) {
				continue;
			}
			if node.leaf {
				for s in node.a..node.a + node.b {
					let t = self.tris[s as usize];
					if ray_triangle(ray, t[0], t[1], t[2]).is_some() {
						count += 1;
					}
				}
			} else {
				stack.push(node.a);
				stack.push(node.b);
			}
		}
		count
	}

	/// Whether `p` lies inside the solid bounded by this (assumed **closed**)
	/// mesh, by ray-crossing parity: three fixed, deliberately non-axis-aligned
	/// rays are cast and the majority parity wins, so a single ray grazing an
	/// edge or vertex of typical axis-aligned CAD geometry cannot flip the
	/// answer. Points exactly *on* the surface are ambiguous (either answer may
	/// come back); callers that care should treat near-surface points — where
	/// [`closest_point`](Self::closest_point) is ~0 — as "on", not "in". For an
	/// open mesh the result is meaningless.
	pub fn contains_point(&self, p: Vec3) -> bool {
		// Irrational-ish directions: no zero components, no two equal, so rays
		// from axis-aligned geometry do not run parallel to faces or through
		// aligned edge lattices.
		const DIRS: [Vec3; 3] = [
			Vec3::new(0.542_425_1, 0.783_406_2, 0.303_254_5),
			Vec3::new(-0.671_088_6, 0.258_770_3, 0.694_721),
			Vec3::new(0.178_309_9, -0.544_203_5, 0.819_735_4),
		];
		DIRS.iter().filter(|&&d| self.ray_crossings(Ray::new(p, d)) % 2 == 1).count() >= 2
	}

	/// All pairs of triangles — `(index in self, index in other)`, original mesh
	/// triangle indices — whose triangles intersect (Möller's test, which counts
	/// coplanar overlap and touching contact as intersecting). Found by a
	/// simultaneous descent of the two hierarchies, so only box-overlapping
	/// candidates are tested exactly. The pair order is deterministic. Beware the
	/// output size: two largely-coincident surfaces intersect in O(n) pairs (and
	/// pathological inputs in O(n·m)).
	pub fn intersecting_triangle_pairs(&self, other: &MeshBvh) -> Vec<(usize, usize)> {
		let mut pairs = Vec::new();
		if self.nodes.is_empty() || other.nodes.is_empty() {
			return pairs;
		}
		let mut stack = vec![(0u32, 0u32)];
		while let Some((na, nb)) = stack.pop() {
			let a = &self.nodes[na as usize];
			let b = &other.nodes[nb as usize];
			if a.bounds.distance_squared_box(b.bounds) > 0.0 {
				continue;
			}
			if a.leaf && b.leaf {
				for sa in a.a..a.a + a.b {
					for sb in b.a..b.a + b.b {
						if crate::meshcheck::tri_tri_intersect(self.tris[sa as usize], other.tris[sb as usize]) {
							pairs.push((self.ids[sa as usize], other.ids[sb as usize]));
						}
					}
				}
			} else if b.leaf || (!a.leaf && a.bounds.size().length() >= b.bounds.size().length()) {
				stack.push((a.a, nb));
				stack.push((a.b, nb));
			} else {
				stack.push((na, b.a));
				stack.push((na, b.b));
			}
		}
		pairs
	}

	/// Minimum separation between this surface and `other`, found by a simultaneous
	/// descent of the two hierarchies with bounding-box pruning (scales far better
	/// than the brute-force pair sweep for large assemblies). `0.0` on touch or
	/// interference.
	///
	/// NOTE — currently UNUSED, and NOT a verified drop-in for
	/// [`Mesh::min_distance`]. The two agree on simple convex cases (parallel /
	/// crossed cylinders, to f32 precision) but DIVERGED on the 37-part gearbox
	/// acceptance: routing assembly clearance through this under-reported some
	/// shaft↔housing pairs toward 0, flipping a must-clear gap to "touching"
	/// (see the reverted commit "BVH-accelerate assembly clearance (O2)"). Root
	/// cause not yet isolated. Before any caller adopts it, validate it against
	/// `Mesh::min_distance` on representative assembly meshes and pin the
	/// equivalence with a curved-mesh / engulfed-part test — a flat-grid
	/// equivalence test is NOT sufficient.
	pub fn min_distance(&self, other: &MeshBvh) -> f64 {
		if self.nodes.is_empty() || other.nodes.is_empty() {
			return f64::INFINITY;
		}
		// Seed a bound from one vertex of each mesh against the other.
		let mut best = f64::INFINITY;
		if let Some(cp) = other.closest_point(self.tris[0][0]) {
			best = best.min(cp.distance as f64);
		}
		if let Some(cp) = self.closest_point(other.tris[0][0]) {
			best = best.min(cp.distance as f64);
		}
		// Descend the pair of trees, always splitting the larger node.
		let mut stack = vec![(0u32, 0u32)];
		while let Some((na, nb)) = stack.pop() {
			let a = &self.nodes[na as usize];
			let b = &other.nodes[nb as usize];
			if (a.bounds.distance_squared_box(b.bounds) as f64) >= best * best {
				continue;
			}
			if a.leaf && b.leaf {
				for sa in a.a..a.a + a.b {
					for sb in b.a..b.a + b.b {
						let d = triangle_triangle_distance(self.tris[sa as usize], other.tris[sb as usize]) as f64;
						if d < best {
							best = d;
							if best <= 0.0 {
								return 0.0;
							}
						}
					}
				}
			} else if b.leaf || (!a.leaf && a.bounds.size().length() >= b.bounds.size().length()) {
				stack.push((a.a, nb));
				stack.push((a.b, nb));
			} else {
				stack.push((na, b.a));
				stack.push((na, b.b));
			}
		}
		best
	}
}

/// Recursively build the subtree over `order[start..end]`, returning its node
/// index. Splits at the median of the axis with the widest centroid spread.
fn build(tris: &[[Vec3; 3]], order: &mut [usize], start: usize, end: usize, nodes: &mut Vec<Node>) -> u32 {
	let bounds = bounds_of(tris, &order[start..end]);
	let idx = nodes.len() as u32;
	nodes.push(Node { bounds, a: start as u32, b: (end - start) as u32, leaf: true });
	if end - start <= LEAF {
		return idx;
	}
	let axis = widest_centroid_axis(tris, &order[start..end]);
	let mid = (start + end) / 2;
	order[start..end]
		.select_nth_unstable_by(mid - start, |&x, &y| centroid(tris[x]).to_array()[axis].total_cmp(&centroid(tris[y]).to_array()[axis]));
	let l = build(tris, order, start, mid, nodes);
	let r = build(tris, order, mid, end, nodes);
	let node = &mut nodes[idx as usize];
	node.leaf = false;
	node.a = l;
	node.b = r;
	idx
}

fn centroid(t: [Vec3; 3]) -> Vec3 {
	(t[0] + t[1] + t[2]) / 3.0
}

fn bounds_of(tris: &[[Vec3; 3]], idxs: &[usize]) -> Aabb {
	let mut bb = Aabb::empty();
	for &i in idxs {
		for &v in &tris[i] {
			bb = bb.expand_point(v);
		}
	}
	bb
}

/// The axis (0/1/2) along which the triangle centroids are most spread.
fn widest_centroid_axis(tris: &[[Vec3; 3]], idxs: &[usize]) -> usize {
	let mut bb = Aabb::empty();
	for &i in idxs {
		bb = bb.expand_point(centroid(tris[i]));
	}
	let s = bb.size().to_array();
	let mut axis = 0;
	if s[1] > s[axis] {
		axis = 1;
	}
	if s[2] > s[axis] {
		axis = 2;
	}
	axis
}
