// Copyright (c) LMCAD. Licensed under the MIT License.

//! Thorough triangle-[`Mesh`] validator.
//!
//! [`check_mesh`] inspects a mesh for the defects that matter to a 3D-printing /
//! solid-modeling pipeline and reports them in a [`MeshReport`]:
//!
//! - **boundary edges** — undirected edges used by exactly one triangle (holes),
//! - **non-manifold edges** — undirected edges used by more than two triangles,
//! - **non-manifold vertices** — vertices whose incident triangles do not form a
//!   single edge-connected fan (an "umbrella"),
//! - **degenerate triangles** — triangles whose area is effectively zero,
//! - **self-intersections** — pairs of triangles that intersect geometrically
//!   without sharing a vertex, located with a simple BVH over the triangles plus
//!   the Möller triangle–triangle overlap test.
//!
//! A mesh is **watertight** when it has no boundary edges, no non-manifold edges
//! and no non-manifold vertices. (Self-intersections and degeneracies are
//! reported separately: a mesh can be edge-watertight yet still self-intersect.)

use std::collections::HashMap;

use crate::math::{Aabb, Vec3};
use crate::mesh::Mesh;

/// Relative tolerance for the degenerate-triangle (near-zero area) test.
///
/// A triangle is considered degenerate when twice its area (the cross-product
/// magnitude of two edges) is below this absolute threshold. Kept small so that
/// only genuinely collapsed triangles are flagged.
const DEGENERATE_AREA_EPSILON: f32 = 1e-12;

/// A summary of mesh-validity defects produced by [`check_mesh`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MeshReport {
	/// `true` when there are no boundary, non-manifold, or non-orientable edges and
	/// no non-manifold vertices (the mesh bounds a closed, orientable 2-manifold).
	pub watertight: bool,
	/// Number of undirected edges incident to exactly one triangle.
	pub boundary_edges: usize,
	/// Number of undirected edges incident to more than two triangles.
	pub non_manifold_edges: usize,
	/// Number of edges shared by exactly two triangles that traverse the edge in
	/// the SAME direction — a flipped/inconsistent winding (non-orientable
	/// adjacency). Zero for a consistently-oriented surface.
	pub non_orientable_edges: usize,
	/// Number of vertices whose incident triangles form more than one
	/// edge-connected fan.
	pub non_manifold_vertices: usize,
	/// Number of triangles with effectively zero area.
	pub degenerate_triangles: usize,
	/// Number of geometrically intersecting triangle pairs that share no vertex.
	pub self_intersections: usize,
}

/// Run the full validation suite over `mesh`.
///
/// Degenerate triangles are excluded from the self-intersection search (their
/// collapsed geometry would otherwise produce spurious hits), but they are still
/// counted toward the edge / vertex topology so that holes left by a collapsed
/// triangle are reported faithfully.
pub fn check_mesh(mesh: &Mesh) -> MeshReport {
	let (boundary_edges, non_manifold_edges, non_orientable_edges) = edge_report(mesh);
	let non_manifold_vertices = non_manifold_vertex_count(mesh);
	let degenerate_triangles = degenerate_triangle_count(mesh);
	let self_intersections = self_intersection_count(mesh);

	let watertight = mesh.triangle_count() > 0
		&& boundary_edges == 0
		&& non_manifold_edges == 0
		&& non_orientable_edges == 0
		&& non_manifold_vertices == 0;

	MeshReport {
		watertight,
		boundary_edges,
		non_manifold_edges,
		non_orientable_edges,
		non_manifold_vertices,
		degenerate_triangles,
		self_intersections,
	}
}

/// Topological 2-manifold test — the rigorous `watertight` (no boundary,
/// non-manifold, or non-orientable edges and no non-manifold vertices) WITHOUT
/// the costly self-intersection search. Self-intersection is a separate
/// geometric property (see [`MeshReport::self_intersections`]); this is the
/// closure/manifold guarantee that [`crate::Mesh::is_watertight`] relies on.
pub(crate) fn is_two_manifold(mesh: &Mesh) -> bool {
	if mesh.triangle_count() == 0 {
		return false;
	}
	let (boundary, non_manifold, non_orientable) = edge_report(mesh);
	boundary == 0 && non_manifold == 0 && non_orientable == 0 && non_manifold_vertex_count(mesh) == 0
}

/// Canonical (unordered) representation of an undirected edge.
fn edge_key(a: u32, b: u32) -> (u32, u32) {
	if a < b {
		(a, b)
	} else {
		(b, a)
	}
}

/// Count boundary edges (used once) and non-manifold edges (used >2 times).
fn edge_report(mesh: &Mesh) -> (usize, usize, usize) {
	// Directed half-edge counts: `dir[(a, b)]` is how many triangles traverse a→b.
	let mut dir: HashMap<(u32, u32), u32> = HashMap::new();
	for t in mesh.triangles() {
		for &(a, b) in &[(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
			*dir.entry((a, b)).or_insert(0) += 1;
		}
	}
	let (mut boundary, mut non_manifold, mut non_orientable) = (0, 0, 0);
	let mut seen: std::collections::HashSet<(u32, u32)> = std::collections::HashSet::new();
	for &(a, b) in dir.keys() {
		let key = edge_key(a, b);
		if !seen.insert(key) {
			continue; // count each undirected edge once
		}
		let fwd = dir.get(&key).copied().unwrap_or(0);
		let bwd = dir.get(&(key.1, key.0)).copied().unwrap_or(0);
		match fwd + bwd {
			1 => boundary += 1,
			2 if fwd == 2 || bwd == 2 => non_orientable += 1, // both same direction
			n if n > 2 => non_manifold += 1,
			_ => {}
		}
	}
	(boundary, non_manifold, non_orientable)
}

/// True when a triangle's two index pairs are effectively collapsed (zero area).
fn is_degenerate(mesh: &Mesh, tri: [u32; 3]) -> bool {
	let n = mesh.positions.len();
	if tri[0] as usize >= n || tri[1] as usize >= n || tri[2] as usize >= n {
		return true;
	}
	// A repeated index can never form a triangle with area.
	if tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0] {
		return true;
	}
	let a = mesh.positions[tri[0] as usize];
	let b = mesh.positions[tri[1] as usize];
	let c = mesh.positions[tri[2] as usize];
	let twice_area = (b - a).cross(c - a).length();
	twice_area <= DEGENERATE_AREA_EPSILON
}

/// Count triangles that are degenerate (near-zero area or repeated indices).
fn degenerate_triangle_count(mesh: &Mesh) -> usize {
	mesh.triangles().filter(|&t| is_degenerate(mesh, t)).count()
}

/// Disjoint-set forest (union-find) with path halving and union by size.
struct UnionFind {
	parent: Vec<usize>,
	size: Vec<usize>,
}

impl UnionFind {
	fn new(n: usize) -> Self {
		Self { parent: (0..n).collect(), size: vec![1; n] }
	}

	fn find(&mut self, mut x: usize) -> usize {
		while self.parent[x] != x {
			self.parent[x] = self.parent[self.parent[x]];
			x = self.parent[x];
		}
		x
	}

	fn union(&mut self, a: usize, b: usize) {
		let (ra, rb) = (self.find(a), self.find(b));
		if ra == rb {
			return;
		}
		let (big, small) = if self.size[ra] >= self.size[rb] { (ra, rb) } else { (rb, ra) };
		self.parent[small] = big;
		self.size[big] += self.size[small];
	}

	/// Number of distinct roots among the first `n` elements.
	fn component_count(&mut self, n: usize) -> usize {
		let mut roots = std::collections::HashSet::new();
		for i in 0..n {
			let r = self.find(i);
			roots.insert(r);
		}
		roots.len()
	}
}

/// Count vertices whose incident triangles do not form a single edge-connected
/// fan.
///
/// For each vertex `v` we gather the triangles touching `v`. Two such triangles
/// belong to the same fan if they share an edge through `v` (i.e. they share a
/// second vertex `w != v`). We union them on that shared neighbour and count the
/// vertices that end up with more than one component. Degenerate / out-of-range
/// triangles are skipped so they cannot create phantom components.
fn non_manifold_vertex_count(mesh: &Mesh) -> usize {
	let vcount = mesh.positions.len();
	if vcount == 0 {
		return 0;
	}

	// For each vertex, the list of (local triangle id) it participates in, where
	// the local id indexes into `incident`'s own per-vertex list.
	let mut incident: Vec<Vec<usize>> = vec![Vec::new(); vcount];
	// Parallel store of the two *other* corners of each incident triangle, in the
	// same order as `incident[v]`.
	let mut others: Vec<Vec<(u32, u32)>> = vec![Vec::new(); vcount];

	for t in mesh.triangles() {
		if is_degenerate(mesh, t) {
			continue;
		}
		for k in 0..3 {
			let v = t[k] as usize;
			let o1 = t[(k + 1) % 3];
			let o2 = t[(k + 2) % 3];
			incident[v].push(others[v].len());
			others[v].push((o1, o2));
		}
	}

	let mut non_manifold = 0;
	for tris in &others {
		let m = tris.len();
		if m <= 1 {
			// 0 or 1 incident triangle is trivially a single (or empty) fan.
			continue;
		}
		// Union triangles that share a neighbour vertex (an edge through `v`).
		let mut uf = UnionFind::new(m);
		// Map a neighbour-vertex id -> the first local triangle that referenced it.
		let mut neighbour_first: HashMap<u32, usize> = HashMap::new();
		for (local, &(o1, o2)) in tris.iter().enumerate() {
			for nb in [o1, o2] {
				match neighbour_first.get(&nb) {
					Some(&first) => uf.union(first, local),
					None => {
						neighbour_first.insert(nb, local);
					}
				}
			}
		}
		if uf.component_count(m) > 1 {
			non_manifold += 1;
		}
	}
	non_manifold
}

// --- BVH + triangle–triangle intersection ------------------------------------

/// A node of the median-split BVH built over the mesh triangles.
struct BvhNode {
	bounds: Aabb,
	/// For a leaf: range `[start, end)` into the reordered triangle index list.
	/// For an interior node: `start == end` is unused; children are valid.
	start: usize,
	end: usize,
	left: Option<usize>,
	right: Option<usize>,
}

impl BvhNode {
	fn is_leaf(&self) -> bool {
		self.left.is_none() && self.right.is_none()
	}
}

/// A flat BVH over a set of triangles (referenced by their triangle index).
struct TriangleBvh {
	nodes: Vec<BvhNode>,
	/// Reordered list of triangle indices grouped per leaf.
	order: Vec<usize>,
}

/// Maximum triangles in a BVH leaf before splitting.
const BVH_LEAF_SIZE: usize = 4;

impl TriangleBvh {
	/// Build a BVH over the given triangle indices, using `bounds` (per triangle,
	/// indexed by triangle id) and `centroids`.
	fn build(tri_ids: Vec<usize>, bounds: &[Aabb], centroids: &[Vec3]) -> Self {
		let mut bvh = TriangleBvh { nodes: Vec::new(), order: tri_ids };
		if !bvh.order.is_empty() {
			let n = bvh.order.len();
			bvh.build_range(0, n, bounds, centroids);
		}
		bvh
	}

	/// Recursively build the node covering `order[start..end]`, returning its
	/// node index.
	fn build_range(
		&mut self,
		start: usize,
		end: usize,
		bounds: &[Aabb],
		centroids: &[Vec3],
	) -> usize {
		let mut node_bounds = Aabb::empty();
		for &tid in &self.order[start..end] {
			node_bounds = node_bounds.union(bounds[tid]);
		}

		let count = end - start;
		let node_index = self.nodes.len();
		self.nodes.push(BvhNode {
			bounds: node_bounds,
			start,
			end,
			left: None,
			right: None,
		});

		if count <= BVH_LEAF_SIZE {
			return node_index;
		}

		// Split along the longest axis of the centroid bounds at the median.
		let mut cb = Aabb::empty();
		for &tid in &self.order[start..end] {
			cb = cb.expand_point(centroids[tid]);
		}
		let extent = cb.size();
		let axis = if extent.x >= extent.y && extent.x >= extent.z {
			0
		} else if extent.y >= extent.z {
			1
		} else {
			2
		};

		let axis_val = |tid: usize| -> f32 {
			let c = centroids[tid];
			match axis {
				0 => c.x,
				1 => c.y,
				_ => c.z,
			}
		};

		let mid = start + count / 2;
		// Partial sort: place the median element so [start..mid] <= [mid..end].
		self.order[start..end]
			.select_nth_unstable_by(count / 2, |&a, &b| {
				axis_val(a).partial_cmp(&axis_val(b)).unwrap_or(std::cmp::Ordering::Equal)
			});

		// Guard against a degenerate split (all centroids equal) → keep as leaf.
		if mid == start || mid == end {
			return node_index;
		}

		let left = self.build_range(start, mid, bounds, centroids);
		let right = self.build_range(mid, end, bounds, centroids);
		self.nodes[node_index].left = Some(left);
		self.nodes[node_index].right = Some(right);
		node_index
	}

	/// Collect all triangle-id pairs whose leaf bounding boxes overlap, invoking
	/// `visit(a, b)` for each candidate pair with `a < b` (triangle ids).
	fn for_each_candidate_pair(&self, mut visit: impl FnMut(usize, usize)) {
		if self.nodes.is_empty() {
			return;
		}
		// Stack of node-index pairs to test for overlap (self-pairs allowed for
		// within-leaf testing).
		let mut stack: Vec<(usize, usize)> = vec![(0, 0)];
		while let Some((a, b)) = stack.pop() {
			let na = &self.nodes[a];
			let nb = &self.nodes[b];
			if a != b && !aabb_overlap(na.bounds, nb.bounds) {
				continue;
			}
			match (na.is_leaf(), nb.is_leaf()) {
				(true, true) => {
					if a == b {
						// All unordered pairs within one leaf.
						let s = na.start;
						let e = na.end;
						for i in s..e {
							for j in (i + 1)..e {
								let (ti, tj) = (self.order[i], self.order[j]);
								let (lo, hi) = if ti < tj { (ti, tj) } else { (tj, ti) };
								visit(lo, hi);
							}
						}
					} else {
						for i in na.start..na.end {
							for j in nb.start..nb.end {
								let (ti, tj) = (self.order[i], self.order[j]);
								if ti == tj {
									continue;
								}
								let (lo, hi) = if ti < tj { (ti, tj) } else { (tj, ti) };
								visit(lo, hi);
							}
						}
					}
				}
				_ => {
					// Descend the larger (non-leaf-preferred) side.
					let descend_a = !na.is_leaf() && (nb.is_leaf() || a != b);
					if a == b {
						// Self-pair on an interior node: expand into child pairs.
						let (l, r) = (na.left.unwrap(), na.right.unwrap());
						stack.push((l, l));
						stack.push((r, r));
						stack.push((l, r));
					} else if descend_a {
						let (l, r) = (na.left.unwrap(), na.right.unwrap());
						stack.push((l, b));
						stack.push((r, b));
					} else {
						let (l, r) = (nb.left.unwrap(), nb.right.unwrap());
						stack.push((a, l));
						stack.push((a, r));
					}
				}
			}
		}
	}
}

/// True if two AABBs overlap (touching counts as overlap).
fn aabb_overlap(a: Aabb, b: Aabb) -> bool {
	a.min.cmple(b.max).all() && b.min.cmple(a.max).all()
}

/// Count intersecting triangle pairs that share no vertex.
///
/// Builds a BVH over the non-degenerate triangles and runs the Möller
/// triangle–triangle overlap test on every candidate pair whose leaf boxes
/// overlap. Pairs that share a vertex index are skipped (adjacent triangles are
/// expected to touch along shared edges).
fn self_intersection_count(mesh: &Mesh) -> usize {
	let tri_count = mesh.triangle_count();
	if tri_count < 2 {
		return 0;
	}

	let tris: Vec<[u32; 3]> = mesh.triangles().collect();

	// Canonicalize vertex identity by POSITION so adjacency is detected even on
	// unwelded meshes (e.g. raw per-face B-rep output, where coincident corners
	// carry distinct indices). Without this, every shared edge between duplicated
	// vertices would register as a false self-intersection.
	let canon = canonical_ids(mesh);
	let tris_canon: Vec<[u32; 3]> = tris
		.iter()
		.map(|t| [canon[t[0] as usize], canon[t[1] as usize], canon[t[2] as usize]])
		.collect();

	// Per-triangle vertex positions, bounds and centroids; degenerate triangles
	// get an empty box so the BVH never visits them.
	let mut verts: Vec<[Vec3; 3]> = Vec::with_capacity(tri_count);
	let mut bounds: Vec<Aabb> = Vec::with_capacity(tri_count);
	let mut centroids: Vec<Vec3> = Vec::with_capacity(tri_count);
	let mut live: Vec<usize> = Vec::new();

	for (tid, &t) in tris.iter().enumerate() {
		if is_degenerate(mesh, t) {
			verts.push([Vec3::ZERO; 3]);
			bounds.push(Aabb::empty());
			centroids.push(Vec3::ZERO);
			continue;
		}
		let a = mesh.positions[t[0] as usize];
		let b = mesh.positions[t[1] as usize];
		let c = mesh.positions[t[2] as usize];
		verts.push([a, b, c]);
		let bb = Aabb::from_points(&[a, b, c]);
		bounds.push(bb);
		centroids.push((a + b + c) / 3.0);
		live.push(tid);
	}

	if live.len() < 2 {
		return 0;
	}

	let bvh = TriangleBvh::build(live, &bounds, &centroids);

	let mut count = 0usize;
	bvh.for_each_candidate_pair(|a, b| {
		// Skip pairs that share a vertex by position (adjacent / fan triangles).
		if shares_vertex(tris_canon[a], tris_canon[b]) {
			return;
		}
		if tri_tri_intersect(verts[a], verts[b]) {
			count += 1;
		}
	});
	count
}

/// Whether the mesh has a PROPER self-intersection: two triangles sharing no
/// vertex index whose interiors cross (segment–triangle test; edge/vertex grazes
/// and coplanar overlap excluded). Same predicate and adjacency rule as the
/// historic O(T²) pair sweep, but candidate pairs come from the triangle BVH —
/// a clean mesh (the common B-rep validation case) costs ~O(T log T), not O(T²).
/// The BVH yields a superset of the box-overlapping pairs the sweep tested, and
/// a crossing requires overlapping boxes, so the boolean result is identical.
/// Backs [`Mesh::has_self_intersection`](crate::Mesh::has_self_intersection).
pub(crate) fn has_proper_self_intersection(mesh: &Mesh) -> bool {
	let tri_count = mesh.triangle_count();
	if tri_count < 2 {
		return false;
	}
	let tris: Vec<[u32; 3]> = mesh.triangles().collect();
	// Per-triangle corner positions, bounds and centroids. Degenerate triangles
	// are kept (real boxes) so the candidate set matches the old all-pairs sweep;
	// `triangles_cross` returns false for them via its determinant guard.
	let mut verts: Vec<[Vec3; 3]> = Vec::with_capacity(tri_count);
	let mut bounds: Vec<Aabb> = Vec::with_capacity(tri_count);
	let mut centroids: Vec<Vec3> = Vec::with_capacity(tri_count);
	for &t in &tris {
		let a = mesh.positions[t[0] as usize];
		let b = mesh.positions[t[1] as usize];
		let c = mesh.positions[t[2] as usize];
		verts.push([a, b, c]);
		bounds.push(Aabb::from_points(&[a, b, c]));
		centroids.push((a + b + c) / 3.0);
	}
	let bvh = TriangleBvh::build((0..tri_count).collect(), &bounds, &centroids);
	let mut hit = false;
	bvh.for_each_candidate_pair(|a, b| {
		// Skip adjacent triangles (shared vertex index); once a crossing is found
		// the enumerator keeps draining but every later visit returns immediately.
		if hit || shares_vertex(tris[a], tris[b]) {
			return;
		}
		if triangles_cross(verts[a], verts[b]) {
			hit = true;
		}
	});
	hit
}

/// Does segment `o`→`e` cross the *interior* of triangle `abc`? Epsilon margins
/// exclude edge/vertex grazes so only proper crossings count (Möller–Trumbore).
fn seg_hits_tri(o: Vec3, e: Vec3, a: Vec3, b: Vec3, c: Vec3) -> bool {
	let eps = 1e-6_f32;
	let dir = e - o;
	let ab = b - a;
	let ac = c - a;
	let pv = dir.cross(ac);
	let det = ab.dot(pv);
	if det.abs() < eps {
		return false; // segment parallel to the triangle plane
	}
	let inv = 1.0 / det;
	let tv = o - a;
	let u = tv.dot(pv) * inv;
	if u <= eps || u >= 1.0 - eps {
		return false;
	}
	let qv = tv.cross(ab);
	let v = dir.dot(qv) * inv;
	if v <= eps || u + v >= 1.0 - eps {
		return false;
	}
	let t = ac.dot(qv) * inv;
	t > eps && t < 1.0 - eps
}

/// Two triangles cross iff an edge of one pierces the interior of the other (six
/// segment–triangle tests). Coplanar overlap is intentionally not a crossing.
fn triangles_cross(t1: [Vec3; 3], t2: [Vec3; 3]) -> bool {
	let [a0, a1, a2] = t1;
	let [b0, b1, b2] = t2;
	seg_hits_tri(a0, a1, b0, b1, b2)
		|| seg_hits_tri(a1, a2, b0, b1, b2)
		|| seg_hits_tri(a2, a0, b0, b1, b2)
		|| seg_hits_tri(b0, b1, a0, a1, a2)
		|| seg_hits_tri(b1, b2, a0, a1, a2)
		|| seg_hits_tri(b2, b0, a0, a1, a2)
}

/// Map each vertex to a canonical id shared by all vertices at the same position
/// (quantized to a small fraction of the mesh size). Lets adjacency be tested by
/// position rather than index, so unwelded meshes don't yield false positives.
fn canonical_ids(mesh: &Mesh) -> Vec<u32> {
	use std::collections::HashMap;
	let tol = (mesh.aabb().diagonal() as f64 * 1e-6).max(1e-9);
	let inv = 1.0 / tol;
	let mut map: HashMap<(i64, i64, i64), u32> = HashMap::new();
	let mut ids = vec![0u32; mesh.positions.len()];
	for (i, p) in mesh.positions.iter().enumerate() {
		let key = (
			(p.x as f64 * inv).round() as i64,
			(p.y as f64 * inv).round() as i64,
			(p.z as f64 * inv).round() as i64,
		);
		let next = map.len() as u32;
		ids[i] = *map.entry(key).or_insert(next);
	}
	ids
}

/// True if the two triangles reference any common vertex index.
fn shares_vertex(a: [u32; 3], b: [u32; 3]) -> bool {
	for & va in &a {
		for &vb in &b {
			if va == vb {
				return true;
			}
		}
	}
	false
}

/// The Möller (1997) triangle–triangle overlap test in 3D.
///
/// Returns `true` if triangles `t1` and `t2` intersect (including coplanar
/// overlap and shared-edge / touching contact). Both triangles are assumed
/// non-degenerate by the caller.
pub(crate) fn tri_tri_intersect(t1: [Vec3; 3], t2: [Vec3; 3]) -> bool {
	let [v0, v1, v2] = t1;
	let [u0, u1, u2] = t2;

	// Plane of triangle 1: n1 · x + d1 = 0.
	let n1 = (v1 - v0).cross(v2 - v0);
	let d1 = -n1.dot(v0);

	// The signed distances below use the UN-normalized normal `n` (length 2·area),
	// so a fixed absolute epsilon would be size-dependent. Scaling the snap
	// threshold by |n| makes it a true point-to-plane distance tolerance —
	// scale-invariant across tiny and large meshes.
	const REL: f32 = 1e-7;
	let n1len = n1.length();
	let e1 = REL * n1len;

	// Signed distances of triangle 2's vertices to plane 1.
	let (du0, du1, du2) = (n1.dot(u0) + d1, n1.dot(u1) + d1, n1.dot(u2) + d1);
	let du0c = clamp_eps(du0, e1);
	let du1c = clamp_eps(du1, e1);
	let du2c = clamp_eps(du2, e1);

	// Triangle 2 entirely on one side of plane 1 → no overlap.
	if du0c * du1c > 0.0 && du0c * du2c > 0.0 {
		return false;
	}

	// Plane of triangle 2.
	let n2 = (u1 - u0).cross(u2 - u0);
	let d2 = -n2.dot(u0);
	let n2len = n2.length();
	let e2 = REL * n2len;

	let (dv0, dv1, dv2) = (n2.dot(v0) + d2, n2.dot(v1) + d2, n2.dot(v2) + d2);
	let dv0c = clamp_eps(dv0, e2);
	let dv1c = clamp_eps(dv1, e2);
	let dv2c = clamp_eps(dv2, e2);

	if dv0c * dv1c > 0.0 && dv0c * dv2c > 0.0 {
		return false;
	}

	// Direction of the line of intersection of the two planes.
	let dir = n1.cross(n2);

	// Coplanar when |n1 × n2| = |n1|·|n2|·sin(θ) is below the relative tolerance.
	if dir.length() <= REL * n1len * n2len {
		return coplanar_tri_tri(n1, t1, t2);
	}

	// Project onto the largest component of `dir` (a 1-D parameterisation).
	let abs = dir.abs();
	let index = if abs.x >= abs.y && abs.x >= abs.z {
		0
	} else if abs.y >= abs.z {
		1
	} else {
		2
	};
	let proj = |p: Vec3| -> f32 {
		match index {
			0 => p.x,
			1 => p.y,
			_ => p.z,
		}
	};

	let (vp0, vp1, vp2) = (proj(v0), proj(v1), proj(v2));
	let (up0, up1, up2) = (proj(u0), proj(u1), proj(u2));

	// Intervals on the intersection line for each triangle.
	let isect1 = compute_interval(vp0, vp1, vp2, dv0c, dv1c, dv2c);
	let isect2 = compute_interval(up0, up1, up2, du0c, du1c, du2c);

	let (a0, a1) = isect1;
	let (b0, b1) = isect2;
	let (lo1, hi1) = (a0.min(a1), a0.max(a1));
	let (lo2, hi2) = (b0.min(b1), b0.max(b1));

	// Overlap of the two parameter intervals (touching counts).
	hi1 >= lo2 && hi2 >= lo1
}

/// Snap a near-zero signed distance to exactly zero so co-planar / on-plane
/// vertices are treated consistently.
fn clamp_eps(d: f32, eps: f32) -> f32 {
	if d.abs() < eps {
		0.0
	} else {
		d
	}
}

/// Compute the 1-D interval (two endpoints on the intersection line) cut out of a
/// triangle, given its projected vertices and their signed distances to the other
/// plane. The single vertex on the opposite side of the plane is the pivot.
fn compute_interval(vp0: f32, vp1: f32, vp2: f32, d0: f32, d1: f32, d2: f32) -> (f32, f32) {
	// Reorder so the odd-one-out vertex (the one alone on its side) is in the
	// middle, matching the canonical Möller formulation.
	if d0 * d1 > 0.0 {
		// v2 is the odd one out.
		interval_from(vp2, vp0, vp1, d2, d0, d1)
	} else if d0 * d2 > 0.0 {
		// v1 is the odd one out.
		interval_from(vp1, vp0, vp2, d1, d0, d2)
	} else if d1 * d2 > 0.0 || d0 != 0.0 {
		// v0 is the odd one out.
		interval_from(vp0, vp1, vp2, d0, d1, d2)
	} else if d1 != 0.0 {
		interval_from(vp1, vp0, vp2, d1, d0, d2)
	} else if d2 != 0.0 {
		interval_from(vp2, vp0, vp1, d2, d0, d1)
	} else {
		// All three on the plane: caller handled coplanar separately, but be safe.
		(vp0.min(vp1).min(vp2), vp0.max(vp1).max(vp2))
	}
}

/// Two intersection-line parameters from a pivot vertex `p` (distance `dp`) and
/// the two opposite vertices `a`, `b` (distances `da`, `db`).
fn interval_from(p: f32, a: f32, b: f32, dp: f32, da: f32, db: f32) -> (f32, f32) {
	let t0 = lerp_zero(p, a, dp, da);
	let t1 = lerp_zero(p, b, dp, db);
	(t0, t1)
}

/// Parameter where the segment `p`–`q` crosses the plane, given signed distances
/// `dp`, `dq`. Falls back to the midpoint if the distances coincide (segment in
/// the plane).
fn lerp_zero(p: f32, q: f32, dp: f32, dq: f32) -> f32 {
	let denom = dp - dq;
	if denom.abs() <= f32::MIN_POSITIVE {
		(p + q) * 0.5
	} else {
		p + (q - p) * (dp / denom)
	}
}

/// Coplanar triangle–triangle overlap test (both triangles lie in plane `n`).
fn coplanar_tri_tri(n: Vec3, t1: [Vec3; 3], t2: [Vec3; 3]) -> bool {
	// Project onto the plane's dominant axis to a 2-D problem.
	let abs = n.abs();
	let (i0, i1) = if abs.x >= abs.y && abs.x >= abs.z {
		(1usize, 2usize)
	} else if abs.y >= abs.z {
		(0, 2)
	} else {
		(0, 1)
	};
	let to2d = |p: Vec3| -> [f32; 2] {
		let arr = [p.x, p.y, p.z];
		[arr[i0], arr[i1]]
	};
	let a = [to2d(t1[0]), to2d(t1[1]), to2d(t1[2])];
	let b = [to2d(t2[0]), to2d(t2[1]), to2d(t2[2])];

	// Edge-cross tests: any edge of one triangle crossing any edge of the other.
	for i in 0..3 {
		let a0 = a[i];
		let a1 = a[(i + 1) % 3];
		for j in 0..3 {
			let b0 = b[j];
			let b1 = b[(j + 1) % 3];
			if segments_intersect_2d(a0, a1, b0, b1) {
				return true;
			}
		}
	}

	// Containment: a vertex of one triangle inside the other.
	if point_in_triangle_2d(a[0], b) || point_in_triangle_2d(b[0], a) {
		return true;
	}
	false
}

/// 2-D orientation sign of the ordered triple (`a`, `b`, `c`).
///
/// Routed through the exact [`crate::predicates::orient2d`] (the `f32`→`f64`
/// promotion is lossless, so the sign is the exact orientation of the original
/// points). Returns a clean `±1`/`0` so a near-degenerate triangle classifies
/// consistently — naive `f32` could return a wrong sign and make the
/// self-intersection test disagree with itself across the triangle's edges.
fn orient2d(a: [f32; 2], b: [f32; 2], c: [f32; 2]) -> f32 {
	let s = crate::predicates::orient2d(
		[a[0] as f64, a[1] as f64],
		[b[0] as f64, b[1] as f64],
		[c[0] as f64, c[1] as f64],
	);
	if s > 0.0 {
		1.0
	} else if s < 0.0 {
		-1.0
	} else {
		0.0
	}
}

/// True if 2-D segments `p1`–`p2` and `q1`–`q2` intersect (including touching).
fn segments_intersect_2d(p1: [f32; 2], p2: [f32; 2], q1: [f32; 2], q2: [f32; 2]) -> bool {
	let d1 = orient2d(q1, q2, p1);
	let d2 = orient2d(q1, q2, p2);
	let d3 = orient2d(p1, p2, q1);
	let d4 = orient2d(p1, p2, q2);
	if ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
		&& ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
	{
		return true;
	}
	// Colinear / touching endpoints.
	(d1 == 0.0 && on_segment_2d(q1, q2, p1))
		|| (d2 == 0.0 && on_segment_2d(q1, q2, p2))
		|| (d3 == 0.0 && on_segment_2d(p1, p2, q1))
		|| (d4 == 0.0 && on_segment_2d(p1, p2, q2))
}

/// True if colinear point `p` lies within the bounding box of segment `a`–`b`.
fn on_segment_2d(a: [f32; 2], b: [f32; 2], p: [f32; 2]) -> bool {
	p[0] >= a[0].min(b[0])
		&& p[0] <= a[0].max(b[0])
		&& p[1] >= a[1].min(b[1])
		&& p[1] <= a[1].max(b[1])
}

/// True if 2-D point `p` lies inside (or on the border of) triangle `t`.
fn point_in_triangle_2d(p: [f32; 2], t: [[f32; 2]; 3]) -> bool {
	let d0 = orient2d(t[0], t[1], p);
	let d1 = orient2d(t[1], t[2], p);
	let d2 = orient2d(t[2], t[0], p);
	let has_neg = d0 < 0.0 || d1 < 0.0 || d2 < 0.0;
	let has_pos = d0 > 0.0 || d1 > 0.0 || d2 > 0.0;
	!(has_neg && has_pos)
}

#[cfg(test)]
mod tests {
	use super::*;

	/// A closed regular-ish tetrahedron with outward winding: 4 vertices, 4
	/// faces, every edge shared by exactly two faces.
	fn tetrahedron() -> Mesh {
		let mut m = Mesh::new();
		m.positions = vec![
			Vec3::new(0.0, 0.0, 0.0),
			Vec3::new(1.0, 0.0, 0.0),
			Vec3::new(0.0, 1.0, 0.0),
			Vec3::new(0.0, 0.0, 1.0),
		];
		// Faces wound consistently (the exact orientation is irrelevant for
		// topology, but we keep it sane).
		m.indices = vec![
			0, 2, 1, // bottom
			0, 1, 3, // front
			0, 3, 2, // left
			1, 2, 3, // hypotenuse face
		];
		m
	}

	#[test]
	fn closed_tetrahedron_is_clean() {
		let report = check_mesh(&tetrahedron());
		assert_eq!(
			report,
			MeshReport {
				watertight: true,
				boundary_edges: 0,
				non_manifold_edges: 0,
				non_orientable_edges: 0,
				non_manifold_vertices: 0,
				degenerate_triangles: 0,
				self_intersections: 0,
			}
		);
	}

	#[test]
	fn flipped_face_is_non_orientable() {
		// A closed tetrahedron with one face's winding reversed: still closed and
		// 2-manifold by undirected counts, but its 3 edges now traverse the same
		// way as their neighbours — a non-orientable adjacency the validator must
		// catch (and therefore report not-watertight).
		let mut m = tetrahedron();
		// Reverse the winding of the last triangle.
		let n = m.indices.len();
		m.indices.swap(n - 1, n - 2);
		let report = check_mesh(&m);
		assert_eq!(report.boundary_edges, 0, "still closed");
		assert_eq!(report.non_manifold_edges, 0, "still 2-manifold by undirected count");
		assert_eq!(report.non_orientable_edges, 3, "the flipped face's 3 edges are non-orientable");
		assert!(!report.watertight, "a non-orientable mesh is not watertight");
	}

	#[test]
	fn unwelded_mesh_has_no_false_self_intersections() {
		// "Unweld" the tetrahedron: give every triangle its own three vertices
		// (coincident positions, distinct indices), as raw per-face B-rep output
		// would. Self-intersection must stay 0 — adjacency is by position, not
		// index. (Edge/vertex manifold checks legitimately change for unwelded
		// input, so we only assert the self-intersection count here.)
		let welded = tetrahedron();
		let mut unwelded = Mesh::new();
		for t in welded.triangles() {
			let base = unwelded.positions.len() as u32;
			for &idx in &t {
				unwelded.positions.push(welded.positions[idx as usize]);
			}
			unwelded.push_triangle(base, base + 1, base + 2);
		}
		assert_eq!(check_mesh(&unwelded).self_intersections, 0, "unwelded coincident faces are not self-intersections");
	}

	#[test]
	fn bvh_self_intersection_scales_without_false_positives() {
		// A 16×16 grid of coplanar, edge-adjacent triangles (512 tris) builds a
		// multi-level BVH and MUST report no self-intersection — coplanar and
		// shared-vertex pairs are excluded. Then one upright triangle that pierces
		// a grid-cell interior (sharing no vertex) must flip the result to true.
		// Guards the BVH candidate-pair path against false positives at scale and
		// against missed crossings.
		const N: usize = 16;
		let mut m = Mesh::new();
		for j in 0..=N {
			for i in 0..=N {
				m.positions.push(Vec3::new(i as f32, j as f32, 0.0));
			}
		}
		let idx = |i: usize, j: usize| (j * (N + 1) + i) as u32;
		for j in 0..N {
			for i in 0..N {
				m.push_triangle(idx(i, j), idx(i + 1, j), idx(i + 1, j + 1));
				m.push_triangle(idx(i, j), idx(i + 1, j + 1), idx(i, j + 1));
			}
		}
		assert!(
			!m.has_self_intersection(),
			"a flat {N}×{N} triangle grid must report no self-intersection — the BVH must not false-positive at scale"
		);

		// Upright triangle whose vertical edge pierces the interior of grid cell
		// (8,8) at (8.5, 8.3, 0); shares no vertex with the grid.
		let base = m.positions.len() as u32;
		m.positions.push(Vec3::new(8.5, 8.3, -1.0));
		m.positions.push(Vec3::new(8.5, 8.3, 1.0));
		m.positions.push(Vec3::new(8.7, 8.9, 0.0));
		m.push_triangle(base, base + 1, base + 2);
		assert!(
			m.has_self_intersection(),
			"an upright triangle piercing the grid interior must be detected by the BVH path"
		);
	}

	#[test]
	fn single_open_triangle_has_three_boundary_edges() {
		let mut m = Mesh::new();
		m.positions = vec![Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0)];
		m.indices = vec![0, 1, 2];
		let report = check_mesh(&m);
		assert_eq!(
			report,
			MeshReport {
				watertight: false,
				boundary_edges: 3,
				non_manifold_edges: 0,
				non_orientable_edges: 0,
				non_manifold_vertices: 0,
				degenerate_triangles: 0,
				self_intersections: 0,
			}
		);
	}

	#[test]
	fn crossing_triangles_report_self_intersection() {
		// A "+" shape: one triangle in the XY plane, one in the XZ plane, sharing
		// no vertex, crossing through the middle.
		let mut m = Mesh::new();
		m.positions = vec![
			// Triangle 0 — lies in z = 0, spans the X axis around the origin.
			Vec3::new(-1.0, -1.0, 0.0),
			Vec3::new(1.0, -1.0, 0.0),
			Vec3::new(0.0, 1.0, 0.0),
			// Triangle 1 — lies in x = 0, spans both sides of z = 0, piercing
			// triangle 0 through the middle.
			Vec3::new(0.0, -0.5, -1.0),
			Vec3::new(0.0, -0.5, 1.0),
			Vec3::new(0.0, 0.8, 0.0),
		];
		m.indices = vec![0, 1, 2, 3, 4, 5];
		let report = check_mesh(&m);
		assert!(
			report.self_intersections >= 1,
			"crossing '+' triangles must report a self-intersection, got {:?}",
			report
		);
	}

	#[test]
	fn degenerate_triangle_is_counted() {
		let mut m = Mesh::new();
		// Three colinear points → zero-area triangle.
		m.positions = vec![
			Vec3::new(0.0, 0.0, 0.0),
			Vec3::new(1.0, 0.0, 0.0),
			Vec3::new(2.0, 0.0, 0.0),
		];
		m.indices = vec![0, 1, 2];
		let report = check_mesh(&m);
		assert_eq!(report.degenerate_triangles, 1, "colinear triangle must be degenerate: {:?}", report);
	}

	#[test]
	fn non_manifold_vertex_from_two_disjoint_fans() {
		// Two triangles meeting only at a single shared apex vertex (an
		// hourglass / bowtie at vertex 0): the incident triangles around vertex 0
		// form two separate fans.
		let mut m = Mesh::new();
		m.positions = vec![
			Vec3::new(0.0, 0.0, 0.0), // 0 — shared apex
			Vec3::new(1.0, 1.0, 0.0), // 1
			Vec3::new(1.0, -1.0, 0.0), // 2
			Vec3::new(-1.0, 1.0, 0.0), // 3
			Vec3::new(-1.0, -1.0, 0.0), // 4
		];
		m.indices = vec![
			0, 1, 2, // fan A around 0 (neighbours 1,2)
			0, 3, 4, // fan B around 0 (neighbours 3,4) — disjoint from A
		];
		let report = check_mesh(&m);
		assert_eq!(
			report.non_manifold_vertices, 1,
			"the shared apex must be flagged as non-manifold: {:?}",
			report
		);
	}

	#[test]
	fn non_manifold_edge_from_three_triangles() {
		// Three triangles sharing one edge (0-1): a "fin" / T-junction edge.
		let mut m = Mesh::new();
		m.positions = vec![
			Vec3::new(0.0, 0.0, 0.0), // 0
			Vec3::new(1.0, 0.0, 0.0), // 1
			Vec3::new(0.0, 1.0, 0.0), // 2
			Vec3::new(0.0, 0.0, 1.0), // 3
			Vec3::new(0.0, -1.0, 0.0), // 4
		];
		m.indices = vec![
			0, 1, 2,
			0, 1, 3,
			0, 1, 4,
		];
		let report = check_mesh(&m);
		assert_eq!(report.non_manifold_edges, 1, "edge 0-1 shared by 3 faces: {:?}", report);
	}
}

