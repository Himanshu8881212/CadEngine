// Copyright (c) LMCAD. Licensed under the MIT License.

//! Mesh → SDF bridge.
//!
//! [`MeshSdf`] turns an arbitrary triangle mesh (e.g. a B-rep tessellation, or
//! an imported STL) into an [`Sdf`], so any solid can enter the implicit / CSG
//! world. Unsigned distance comes from a BVH nearest-triangle query; the **sign**
//! comes from the generalized winding number (Jacobson et al.), which is robust
//! even for imperfect meshes — no reliance on consistent normals.
//!
//! The winding number is evaluated with the **fast** BVH-accelerated scheme of
//! Barill et al. (*Fast Winding Numbers for Soups and Clouds*): each BVH node
//! precomputes an aggregate dipole (`Σ area·normal`), the **second-order mixed
//! moment** `Σ area·normal·(centroid_t − centroid)ᵀ`, an area-weighted centroid,
//! and a bounding radius. A query approximates a node by its second-order
//! multipole when the point is far (`dist > BETA · radius`) and otherwise
//! recurses to the exact Van Oosterom–Strackee per-triangle solid angle at the
//! leaves. That makes the per-query sign cost ≈ O(log n) instead of O(n), so
//! voxelizing a large mesh (`VoxelGrid::from_sdf(&mesh_sdf, …)`) scales.

use kernel_core::math::{Aabb, DMat3, DVec3, Vec3};
use kernel_core::mesh::Mesh;
use kernel_core::closest_point_on_triangle;
use kernel_core::sdf::Sdf;

/// Far-field threshold: a node is approximated by its multipole only when the
/// query point is farther than `BETA` times the node's bounding radius. With the
/// second-order (mixed-moment) term carried here, Barill et al.'s recommended
/// ≈2.0 holds the approximation error under 1e-2 (the bound the
/// `fast_winding_matches_brute_force` test enforces; the sphere probe measures
/// 6.4e-3, and a 4000-query hex-nut B-rep probe touched 1.01e-2 at a far-field
/// point where the true winding is ≈ 0 — sign decisions keep two orders of
/// magnitude of margin because near-surface nodes fail this gate and recurse to
/// the exact leaves; measured sweeps in `BENCH.md`). The previous first-order
/// dipole needed BETA = 4.0 for the same bound, which at part scale shadowed the
/// whole tree and degraded near-surface queries to brute force — the dominant
/// cost of the hybrid heal.
const BETA: f64 = 2.0;

/// Maximum triangles in a BVH leaf.
const LEAF: usize = 4;

/// Triangles whose longest edge exceeds this fraction of the mesh diagonal are
/// recursively bisected before the BVH is built. B-rep tessellations mix huge
/// planar facets (a whole prism wall as two triangles) with fine curved ones;
/// a part-sized triangle gives every enclosing node a part-sized radius, so the
/// `dist > BETA·radius` far-field test never fires for queries near the part
/// and the winding number degrades to brute force. Splitting changes **no
/// geometry**: the children exactly tile the parent, solid angles and closest
/// points are additive/identical (up to one f32 midpoint rounding ≤ ulp).
const REFINE_FRAC: f32 = 0.05;

/// Fixed traversal-stack capacity. `build_node` median-splits (`mid = count / 2`),
/// so the tree is balanced and its depth is `ceil(log2(n / LEAF)) + 1` — at most
/// 31 levels for `u32`-indexed triangles. DFS holds one deferred sibling per
/// level, so 64 slots can never overflow; a fixed array keeps the per-query
/// traversal allocation-free (these queries run hundreds of thousands of times
/// per heal).
const STACK: usize = 64;

/// Squared distance from a point to an AABB (0 if inside).
fn aabb_dist2(p: Vec3, b: Aabb) -> f32 {
	let q = p.clamp(b.min, b.max);
	(p - q).length_squared()
}

/// Exact signed solid angle (steradians) subtended at the origin by the triangle
/// with corners `a`, `b`, `c` expressed *relative to the query point*
/// (Van Oosterom–Strackee). Accumulated in `f64`.
fn tri_solid_angle(a: DVec3, b: DVec3, c: DVec3) -> f64 {
	let la = a.length();
	let lb = b.length();
	let lc = c.length();
	let numer = a.dot(b.cross(c));
	let denom = la * lb * lc + a.dot(b) * lc + b.dot(c) * la + c.dot(a) * lb;
	2.0 * numer.atan2(denom)
}

/// A BVH node carrying both the spatial bounds (for nearest-triangle queries) and
/// the aggregate dipole + higher-order moments (for the fast winding number).
#[derive(Clone, Copy)]
struct BvhNode {
	aabb: Aabb,
	/// Aggregate dipole `Σ areaₜ · unit_normalₜ` over the node's triangles.
	area_normal: DVec3,
	/// First mixed moment `M = Σ aₜ·n̂ₜ·(p̄ₜ − centroid)ᵀ` (`p̄ₜ` the triangle
	/// centroid; exact, since `∫_T (x − c) dA = aₜ(p̄ₜ − c)`): the Jacobian term
	/// of the far-field expansion.
	moment: DMat3,
	/// Trace of `moment`, pre-folded for the query loop.
	moment_trace: f64,
	/// Second mixed moments `T_ijk = Σₜ n̂ₜᵢ ∫_T (x−c)_j (x−c)_k dA` for the
	/// Hessian term, stored per symmetric pair `(jk)` ∈ xx,yy,zz,xy,xz,yz as a
	/// vector over `i` (triangle quadratic integrals via the exact 3-midpoint
	/// rule). Together with `moment` this is the order-2 Taylor data of Barill
	/// et al., which holds the far-field error within the tested 1e-2 at
	/// `BETA = 2` (measured margins on [`BETA`]) — the first-order dipole alone
	/// needed `BETA = 4`.
	t2: [DVec3; 6],
	/// Pre-folded linear Hessian coefficient `−3a − 1.5u` (see
	/// [`MeshSdf::winding_number`]).
	h1: DVec3,
	/// Area-weighted centroid of the node's triangles.
	centroid: DVec3,
	/// Distance from `centroid` to the farthest contained vertex (the radius
	/// beyond which the multipole approximation is trustworthy).
	radius: f64,
	/// Inner node: index of the left child (right = left + 1); leaf: `u32::MAX`.
	left: u32,
	/// Leaf triangle range `[start, start + count)` into `tri_order`.
	start: u32,
	count: u32,
}

impl BvhNode {
	fn empty() -> Self {
		BvhNode {
			aabb: Aabb::empty(),
			area_normal: DVec3::ZERO,
			moment: DMat3::ZERO,
			moment_trace: 0.0,
			t2: [DVec3::ZERO; 6],
			h1: DVec3::ZERO,
			centroid: DVec3::ZERO,
			radius: 0.0,
			left: u32::MAX,
			start: 0,
			count: 0,
		}
	}
}

/// Outer product `n dᵀ` (rows from `n`, columns scaled by `d`).
#[inline]
fn outer(n: DVec3, d: DVec3) -> DMat3 {
	DMat3::from_cols(n * d.x, n * d.y, n * d.z)
}

/// Recursively build the BVH subtree at `node_idx` over `order[start..start+count]`
/// (median split on the longest axis). Dipole fields are filled afterwards.
fn build_node(
	nodes: &mut Vec<BvhNode>,
	node_idx: usize,
	order: &mut [u32],
	start: usize,
	count: usize,
	aabbs: &[Aabb],
	centroids: &[Vec3],
) {
	let mut bb = Aabb::empty();
	for &t in &order[start..start + count] {
		bb = bb.union(aabbs[t as usize]);
	}
	if count <= LEAF {
		nodes[node_idx] = BvhNode { aabb: bb, left: u32::MAX, start: start as u32, count: count as u32, ..BvhNode::empty() };
		return;
	}
	let ext = bb.size();
	let axis = if ext.x >= ext.y && ext.x >= ext.z {
		0
	} else if ext.y >= ext.z {
		1
	} else {
		2
	};
	order[start..start + count].sort_unstable_by(|&a, &b| {
		centroids[a as usize][axis].partial_cmp(&centroids[b as usize][axis]).unwrap_or(std::cmp::Ordering::Equal)
	});
	let mid = count / 2;
	let left_idx = nodes.len();
	nodes.push(BvhNode::empty());
	nodes.push(BvhNode::empty());
	nodes[node_idx] = BvhNode { aabb: bb, left: left_idx as u32, ..BvhNode::empty() };
	build_node(nodes, left_idx, order, start, mid, aabbs, centroids);
	build_node(nodes, left_idx + 1, order, start + mid, count - mid, aabbs, centroids);
}

/// A triangle mesh exposed as a signed distance field.
pub struct MeshSdf {
	verts: Vec<Vec3>,
	tris: Vec<[u32; 3]>,
	nodes: Vec<BvhNode>,
	tri_order: Vec<u32>,
	bounds: Aabb,
}

impl MeshSdf {
	/// Build from a triangle mesh. The mesh should be (approximately) closed; the
	/// winding number tolerates small gaps and inconsistent normals.
	pub fn new(mesh: &Mesh) -> Self {
		let mut s = MeshSdf {
			verts: mesh.positions.clone(),
			tris: mesh.triangles().collect(),
			nodes: Vec::new(),
			tri_order: Vec::new(),
			bounds: mesh.aabb(),
		};
		s.refine_oversized();
		s.build_bvh();
		s
	}

	/// Recursively bisect (longest edge, at its midpoint) every triangle whose
	/// longest edge exceeds `REFINE_FRAC` of the mesh diagonal, so BVH node
	/// radii stay small relative to the part and the far-field winding test can
	/// fire near the surface. Deterministic (stack order is a function of the
	/// input order only); represents exactly the same surface.
	fn refine_oversized(&mut self) {
		let diag = self.bounds.diagonal();
		if !diag.is_finite() || diag <= 0.0 {
			return;
		}
		let max_len2 = (diag * REFINE_FRAC) * (diag * REFINE_FRAC);
		let mut out: Vec<[u32; 3]> = Vec::with_capacity(self.tris.len());
		let mut work: Vec<([u32; 3], u32)> = Vec::new();
		for &t in &self.tris {
			work.push((t, 0));
			while let Some((tri, depth)) = work.pop() {
				let [i, j, k] = tri;
				let (a, b, c) = (self.verts[i as usize], self.verts[j as usize], self.verts[k as usize]);
				let e = [(b - a).length_squared(), (c - b).length_squared(), (a - c).length_squared()];
				let longest = if e[0] >= e[1] && e[0] >= e[2] { 0 } else if e[1] >= e[2] { 1 } else { 2 };
				// Depth cap: 12 halvings shrink any finite edge below the
				// threshold (2¹² ≫ 1/REFINE_FRAC); it only guards degenerate
				// arithmetic (e.g. midpoint rounding onto an endpoint).
				if e[longest] <= max_len2 || depth >= 12 {
					out.push(tri);
					continue;
				}
				let (u, v) = match longest {
					0 => (i, j),
					1 => (j, k),
					_ => (k, i),
				};
				let mid = (self.verts[u as usize] + self.verts[v as usize]) * 0.5;
				let m = self.verts.len() as u32;
				self.verts.push(mid);
				let split = match longest {
					0 => [[i, m, k], [m, j, k]],
					1 => [[i, j, m], [i, m, k]],
					_ => [[i, j, m], [m, j, k]],
				};
				work.push((split[0], depth + 1));
				work.push((split[1], depth + 1));
			}
		}
		self.tris = out;
	}

	fn tri_aabb(&self, t: usize) -> Aabb {
		let [i, j, k] = self.tris[t];
		Aabb::from_points(&[self.verts[i as usize], self.verts[j as usize], self.verts[k as usize]])
	}

	fn tri_centroid(&self, t: usize) -> Vec3 {
		let [i, j, k] = self.tris[t];
		(self.verts[i as usize] + self.verts[j as usize] + self.verts[k as usize]) / 3.0
	}

	/// Area (`f64`) and `area · unit_normal` dipole for triangle `t`
	/// (`0.5 · (b−a)×(c−a)`).
	fn tri_dipole(&self, t: usize) -> (f64, DVec3) {
		let [i, j, k] = self.tris[t];
		let a = self.verts[i as usize].as_dvec3();
		let b = self.verts[j as usize].as_dvec3();
		let c = self.verts[k as usize].as_dvec3();
		let cross = (b - a).cross(c - a);
		(0.5 * cross.length(), cross * 0.5)
	}

	fn build_bvh(&mut self) {
		let n = self.tris.len();
		self.nodes.clear();
		let mut order: Vec<u32> = (0..n as u32).collect();
		if n == 0 {
			self.tri_order = order;
			return;
		}
		let aabbs: Vec<Aabb> = (0..n).map(|t| self.tri_aabb(t)).collect();
		let centroids: Vec<Vec3> = (0..n).map(|t| self.tri_centroid(t)).collect();
		let dipoles: Vec<(f64, DVec3)> = (0..n).map(|t| self.tri_dipole(t)).collect();
		self.nodes.push(BvhNode::empty());
		build_node(&mut self.nodes, 0, &mut order, 0, n, &aabbs, &centroids);
		self.tri_order = order;
		self.compute_dipoles(&dipoles);
	}

	/// Fill in each node's aggregate dipole, second-order moment, area-weighted
	/// centroid and radius, processing children before parents.
	fn compute_dipoles(&mut self, dipoles: &[(f64, DVec3)]) {
		let count = self.nodes.len();
		let mut total_area = vec![0.0f64; count];
		let mut weighted_centroid = vec![DVec3::ZERO; count];
		// Children are pushed after their parent, so reverse index order processes
		// leaves/deeper nodes first.
		for ni in (0..count).rev() {
			let node = self.nodes[ni];
			if node.left == u32::MAX {
				let mut area_normal = DVec3::ZERO;
				let mut area = 0.0f64;
				let mut wc = DVec3::ZERO;
				for &t in &self.tri_order[node.start as usize..(node.start + node.count) as usize] {
					let (a, dip) = dipoles[t as usize];
					area_normal += dip;
					area += a;
					wc += self.tri_centroid(t as usize).as_dvec3() * a;
				}
				self.nodes[ni].area_normal = area_normal;
				total_area[ni] = area;
				weighted_centroid[ni] = wc;
			} else {
				let (l, r) = (node.left as usize, node.left as usize + 1);
				self.nodes[ni].area_normal = self.nodes[l].area_normal + self.nodes[r].area_normal;
				total_area[ni] = total_area[l] + total_area[r];
				weighted_centroid[ni] = weighted_centroid[l] + weighted_centroid[r];
			}
			let area = total_area[ni];
			self.nodes[ni].centroid = if area > 0.0 {
				weighted_centroid[ni] / area
			} else {
				self.nodes[ni].aabb.center().as_dvec3()
			};
			// Order-1 and order-2 moments about THIS node's centroid. The triangle
			// first moment `∫_T (x − c) dA = aₜ(p̄ₜ − c)` is exact; the quadratic
			// integrals use the exact 3-midpoint rule. A child's moments shift to
			// the parent centroid `c` by (`s = c_child − c`, `m_j` = column `j` of
			// the child's `M`, `AN` its dipole):
			//   M  += M_child + AN·sᵀ
			//   T(jk) += T_child(jk) + m_j·s_k + m_k·s_j + AN·s_j·s_k
			let c = self.nodes[ni].centroid;
			let mut m = DMat3::ZERO;
			let mut t2 = [DVec3::ZERO; 6];
			// Symmetric pair table (j, k) matching the `t2` layout xx,yy,zz,xy,xz,yz.
			const PAIRS: [(usize, usize); 6] = [(0, 0), (1, 1), (2, 2), (0, 1), (0, 2), (1, 2)];
			if node.left == u32::MAX {
				for &t in &self.tri_order[node.start as usize..(node.start + node.count) as usize] {
					let (_, dip) = dipoles[t as usize];
					let [i, j, k] = self.tris[t as usize];
					let (a, b, cc) = (
						self.verts[i as usize].as_dvec3(),
						self.verts[j as usize].as_dvec3(),
						self.verts[k as usize].as_dvec3(),
					);
					m += outer(dip, (a + b + cc) / 3.0 - c);
					let w = dip / 3.0;
					for md in [(a + b) * 0.5 - c, (b + cc) * 0.5 - c, (cc + a) * 0.5 - c] {
						for (p, &(pj, pk)) in PAIRS.iter().enumerate() {
							t2[p] += w * (md[pj] * md[pk]);
						}
					}
				}
			} else {
				for child in [node.left as usize, node.left as usize + 1] {
					let n = self.nodes[child];
					let s = n.centroid - c;
					m += n.moment + outer(n.area_normal, s);
					let cols = [n.moment.x_axis, n.moment.y_axis, n.moment.z_axis];
					for (p, &(pj, pk)) in PAIRS.iter().enumerate() {
						t2[p] += n.t2[p] + cols[pj] * s[pk] + cols[pk] * s[pj] + n.area_normal * (s[pj] * s[pk]);
					}
				}
			}
			// Pre-fold the query-time linear Hessian coefficient −3a − 1.5u, where
			// a_k = Σᵢ T_iik and u = T(xx) + T(yy) + T(zz).
			let a_vec = DVec3::new(
				t2[0].x + t2[3].y + t2[4].z,
				t2[3].x + t2[1].y + t2[5].z,
				t2[4].x + t2[5].y + t2[2].z,
			);
			let u_vec = t2[0] + t2[1] + t2[2];
			self.nodes[ni].moment = m;
			self.nodes[ni].moment_trace = m.x_axis.x + m.y_axis.y + m.z_axis.z;
			self.nodes[ni].t2 = t2;
			self.nodes[ni].h1 = a_vec * -3.0 - u_vec * 1.5;
		}
		for ni in 0..count {
			let c = self.nodes[ni].centroid;
			let (start, end) = self.subtree_range(ni);
			let mut r2 = 0.0f64;
			for &t in &self.tri_order[start..end] {
				let [i, j, k] = self.tris[t as usize];
				for &v in &[i, j, k] {
					r2 = r2.max((self.verts[v as usize].as_dvec3() - c).length_squared());
				}
			}
			self.nodes[ni].radius = r2.sqrt();
		}
	}

	/// `[start, end)` of the `tri_order` range covered by subtree `ni`.
	fn subtree_range(&self, ni: usize) -> (usize, usize) {
		let node = self.nodes[ni];
		if node.left == u32::MAX {
			(node.start as usize, (node.start + node.count) as usize)
		} else {
			let (ls, _) = self.subtree_range(node.left as usize);
			let (_, re) = self.subtree_range(node.left as usize + 1);
			(ls, re)
		}
	}

	/// Fast generalized winding number at `p` (≈ 1 inside, ≈ 0 outside): far nodes
	/// contribute their order-2 multipole, near nodes recurse to the exact
	/// per-triangle solid angle. Accumulated in `f64`.
	///
	/// The far-field expansion of a node's contribution about its area-weighted
	/// centroid `c` (Barill et al. 2018; `d = c − p`, `ℓ = |d|`, `g(x) =
	/// (x − p)/|x − p|³`): dipole `N·d/ℓ³`, Jacobian term `tr(M)/ℓ³ − 3dᵀMd/ℓ⁵`,
	/// and Hessian term `½⟨∇²g, T⟩ = (d·h1 + 7.5·d·w/ℓ²)/ℓ⁵` where `w(jk-fold) =
	/// Σ_pairs T(jk)·d_j·d_k` and `h1` pre-folds the linear coefficients.
	pub fn winding_number(&self, p: Vec3) -> f64 {
		if self.tris.is_empty() {
			return 0.0;
		}
		let pd = p.as_dvec3();
		let mut omega = 0.0f64;
		let mut stack = [0u32; STACK];
		let mut top = 1usize; // stack[0] is already the root index 0
		while top > 0 {
			top -= 1;
			let node = &self.nodes[stack[top] as usize];
			let to_center = node.centroid - pd;
			let dist2 = to_center.length_squared();
			if dist2 > (BETA * node.radius) * (BETA * node.radius) && dist2 > 0.0 {
				let dist = dist2.sqrt();
				let inv3 = 1.0 / (dist2 * dist);
				let d = to_center;
				let second = node.moment_trace - 3.0 * d.dot(node.moment * d) / dist2;
				let t2 = &node.t2;
				let w_vec = t2[0] * (d.x * d.x)
					+ t2[1] * (d.y * d.y)
					+ t2[2] * (d.z * d.z)
					+ (t2[3] * (d.x * d.y) + t2[4] * (d.x * d.z) + t2[5] * (d.y * d.z)) * 2.0;
				let third = (d.dot(node.h1) + 7.5 * d.dot(w_vec) / dist2) / dist2;
				omega += (node.area_normal.dot(d) + second + third) * inv3;
				continue;
			}
			if node.left == u32::MAX {
				for &t in &self.tri_order[node.start as usize..(node.start + node.count) as usize] {
					let [i, j, k] = self.tris[t as usize];
					omega += tri_solid_angle(
						self.verts[i as usize].as_dvec3() - pd,
						self.verts[j as usize].as_dvec3() - pd,
						self.verts[k as usize].as_dvec3() - pd,
					);
				}
			} else {
				debug_assert!(top + 2 <= STACK, "balanced BVH depth exceeds the fixed stack");
				stack[top] = node.left;
				stack[top + 1] = node.left + 1;
				top += 2;
			}
		}
		omega / (4.0 * std::f64::consts::PI)
	}

	/// Inside test (`winding > ½`), with a provable short-circuit: from any point
	/// outside the mesh's AABB the whole surface subtends less than the solid
	/// angle of the AABB itself, which is below 2π from outside it, so the
	/// winding number is below ½ no matter how the surface is wound — the
	/// padding ring of a voxelization grid never pays for a winding traversal.
	#[inline]
	fn inside(&self, p: Vec3) -> bool {
		if aabb_dist2(p, self.bounds) > 0.0 {
			return false;
		}
		self.winding_number(p) > 0.5
	}

	/// Brute-force exact winding number (O(n) sum of solid angles), for comparison
	/// against the fast path in tests.
	pub fn winding_number_exact(&self, p: Vec3) -> f64 {
		let pd = p.as_dvec3();
		let mut omega = 0.0f64;
		for t in &self.tris {
			omega += tri_solid_angle(
				self.verts[t[0] as usize].as_dvec3() - pd,
				self.verts[t[1] as usize].as_dvec3() - pd,
				self.verts[t[2] as usize].as_dvec3() - pd,
			);
		}
		omega / (4.0 * std::f64::consts::PI)
	}

	/// Nearest point on the surface via BVH traversal (children visited
	/// nearest-first so the bound tightens early): unsigned squared distance,
	/// the closest point itself, and the triangle it lies on.
	fn nearest_on_surface(&self, p: Vec3) -> (f32, Vec3, u32) {
		let mut best = f32::INFINITY;
		let mut best_cp = p;
		let mut best_tri = 0u32;
		let mut stack = [0u32; STACK];
		let mut top = 1usize; // stack[0] is already the root index 0
		while top > 0 {
			top -= 1;
			let node = &self.nodes[stack[top] as usize];
			if aabb_dist2(p, node.aabb) >= best {
				continue;
			}
			if node.left == u32::MAX {
				for &t in &self.tri_order[node.start as usize..(node.start + node.count) as usize] {
					let [i, j, k] = self.tris[t as usize];
					let cp = closest_point_on_triangle(p, self.verts[i as usize], self.verts[j as usize], self.verts[k as usize]);
					let d2 = (p - cp).length_squared();
					if d2 < best {
						best = d2;
						best_cp = cp;
						best_tri = t;
					}
				}
			} else {
				let (l, r) = (node.left, node.left + 1);
				let dl = aabb_dist2(p, self.nodes[l as usize].aabb);
				let dr = aabb_dist2(p, self.nodes[r as usize].aabb);
				debug_assert!(top + 2 <= STACK, "balanced BVH depth exceeds the fixed stack");
				if dl < dr {
					stack[top] = r;
					stack[top + 1] = l;
				} else {
					stack[top] = l;
					stack[top + 1] = r;
				}
				top += 2;
			}
		}
		(best, best_cp, best_tri)
	}
}

impl Sdf for MeshSdf {
	fn distance(&self, p: Vec3) -> f32 {
		if self.tris.is_empty() {
			return f32::INFINITY;
		}
		let unsigned = self.nearest_on_surface(p).0.sqrt();
		if self.inside(p) {
			-unsigned
		} else {
			unsigned
		}
	}

	/// Outward unit normal, computed analytically instead of by the default
	/// six-sample central difference (which costs 6 nearest + 6 winding queries —
	/// it dominated the hybrid heal). Away from the surface the signed-distance
	/// gradient is exactly `sign · (p − closest_point)/|p − closest_point|`
	/// (one nearest + one winding query, and exact across mesh edges/vertices
	/// where a central difference blurs facets). Within direction-noise range of
	/// the surface (`|p − cp|` under ~1e-5 of the mesh diagonal, where the `f32`
	/// closest-point roundoff would dominate the direction), the one-sided limit
	/// is the closest triangle's geometric normal, oriented outward by one
	/// winding probe just off the surface.
	fn gradient(&self, p: Vec3) -> Vec3 {
		if self.tris.is_empty() {
			return Vec3::Z;
		}
		let (d2, cp, tri) = self.nearest_on_surface(p);
		let diag = self.bounds.diagonal().max(1e-6);
		let degen = diag * 1e-5;
		if d2 > degen * degen {
			let sign = if self.inside(p) { -1.0 } else { 1.0 };
			return (p - cp) * (sign / d2.sqrt());
		}
		let [i, j, k] = self.tris[tri as usize];
		let (a, b, c) = (self.verts[i as usize], self.verts[j as usize], self.verts[k as usize]);
		let n = (b - a).cross(c - a).normalize_or_zero();
		if n.length_squared() < 0.5 {
			return Vec3::Z; // degenerate closest triangle: no meaningful direction
		}
		// Probe clearly outside direction-noise range but far below feature size.
		if self.inside(cp + n * (degen * 100.0)) {
			-n
		} else {
			n
		}
	}

	fn bounds(&self) -> Aabb {
		self.bounds.pad(self.bounds.diagonal() * 0.02 + 1e-3)
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	/// A UV sphere of `radius` at the origin, CCW outward winding.
	fn sphere_mesh(radius: f32, stacks: usize, slices: usize) -> Mesh {
		let mut m = Mesh::new();
		let pi = std::f32::consts::PI;
		for i in 0..=stacks {
			let theta = pi * i as f32 / stacks as f32;
			let (st, ct) = theta.sin_cos();
			for j in 0..=slices {
				let phi = 2.0 * pi * j as f32 / slices as f32;
				let (sp, cp) = phi.sin_cos();
				m.push_vertex(Vec3::new(radius * st * cp, radius * ct, radius * st * sp));
			}
		}
		let stride = (slices + 1) as u32;
		for i in 0..stacks as u32 {
			for j in 0..slices as u32 {
				let a = i * stride + j;
				let b = (i + 1) * stride + j;
				m.push_triangle(a, b, i * stride + j + 1);
				m.push_triangle(b, (i + 1) * stride + j + 1, i * stride + j + 1);
			}
		}
		m.ensure_outward();
		m
	}

	struct Rng(u64);
	impl Rng {
		fn next_f32(&mut self) -> f32 {
			self.0 ^= self.0 << 13;
			self.0 ^= self.0 >> 7;
			self.0 ^= self.0 << 17;
			(self.0 >> 40) as f32 / (1u32 << 24) as f32
		}
		fn range(&mut self, lo: f32, hi: f32) -> f32 {
			lo + (hi - lo) * self.next_f32()
		}
	}

	#[test]
	fn fast_winding_matches_brute_force() {
		let s = MeshSdf::new(&sphere_mesh(1.0, 24, 48));
		let mut rng = Rng(0x1234_5678_9abc_def0);
		let mut max_err = 0.0f64;
		for _ in 0..400 {
			let p = Vec3::new(rng.range(-2.5, 2.5), rng.range(-2.5, 2.5), rng.range(-2.5, 2.5));
			max_err = max_err.max((s.winding_number(p) - s.winding_number_exact(p)).abs());
		}
		assert!(max_err < 1e-2, "fast vs brute winding err {max_err} exceeds 1e-2");
	}

	#[test]
	fn sign_inside_outside_and_distance() {
		let s = MeshSdf::new(&sphere_mesh(1.0, 32, 64));
		assert!(s.distance(Vec3::ZERO) < 0.0, "centre is inside");
		assert!(s.distance(Vec3::new(5.0, 0.0, 0.0)) > 0.0, "far point is outside");
		// Unsigned distance tracks the analytic sphere SDF away from the surface.
		let d = s.distance(Vec3::new(2.0, 0.0, 0.0));
		assert!((d - 1.0).abs() < 0.02, "sphere SDF: got {d}, expected ~1.0");
	}

	#[test]
	fn empty_mesh_is_well_defined() {
		let s = MeshSdf::new(&Mesh::new());
		assert_eq!(s.winding_number(Vec3::ZERO), 0.0);
		assert!(s.distance(Vec3::ZERO).is_infinite());
	}
}
