// Copyright (c) LMCAD. Licensed under the MIT License.

//! Topological repair to make a closed triangle mesh a 2-manifold.
//!
//! Dual meshers (Surface Nets / Dual Contouring) place a single vertex per cell,
//! so a cell straddled by two surface sheets (sub-voxel features, near-tangent
//! solids) can leave an edge shared by more than two triangles — closed, but not
//! 2-manifold. [`make_manifold`] repairs this by **patch separation**:
//!
//! 1. Group the directed half-edges by undirected edge. An edge with exactly two
//!    opposite half-edges is an unambiguous *manifold* edge; anything else
//!    (more than two half-edges, or two of the same direction) is non-manifold.
//! 2. Union the shared corners only across the unambiguous manifold edges. The
//!    connected components of that union are the surface *patches*.
//! 3. Emit one vertex per (vertex, patch), so a vertex shared by several patches
//!    splits into one copy per patch.
//!
//! A non-manifold edge between two distinct patches then resolves automatically:
//! each patch gets its own copy of the edge's endpoints, so the edge becomes two
//! manifold edges. Because we never synthesise adjacency across a non-manifold
//! edge, the repair can't create non-orientable (flipped) twins, and only
//! connectivity changes — every new vertex copies an existing position, so the
//! geometry, volume, and (for a closed input) closedness are all preserved.
//!
//! **Scope.** This fully separates *globally-separable* patches (distinct
//! components, or components meeting along an edge or at a point) regardless of
//! how they are arranged around the shared edge. A non-manifold pinch on a single
//! *globally-connected* surface (e.g. a thin neck from a coarse boolean) keeps the
//! two sides in one patch — separating them would open a boundary or move
//! geometry — so it is left unchanged. The repair is therefore monotone: it never
//! increases the non-manifold count. Eliminating connected pinches needs
//! source-level Manifold Dual Contouring (tracked separately); meshing such a
//! model a little finer also avoids them.

use std::collections::HashMap;

use crate::mesh::Mesh;

/// Union-find over corner ids.
struct Uf(Vec<u32>);

impl Uf {
	fn new(n: usize) -> Self {
		Uf((0..n as u32).collect())
	}
	fn find(&mut self, mut x: u32) -> u32 {
		while self.0[x as usize] != x {
			self.0[x as usize] = self.0[self.0[x as usize] as usize];
			x = self.0[x as usize];
		}
		x
	}
	fn union(&mut self, a: u32, b: u32) {
		let (ra, rb) = (self.find(a), self.find(b));
		if ra != rb {
			self.0[ra as usize] = rb;
		}
	}
}

/// Undirected edge counts: `(boundary_edges, non_manifold_edges)` — edges used by
/// exactly one / more than two triangles. Cheap O(n) health metric.
fn edge_health(mesh: &Mesh) -> (usize, usize) {
	let mut counts: HashMap<(u32, u32), u32> = HashMap::new();
	for t in mesh.triangles() {
		for &(a, b) in &[(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
			*counts.entry(if a < b { (a, b) } else { (b, a) }).or_insert(0) += 1;
		}
	}
	let boundary = counts.values().filter(|&&c| c == 1).count();
	let non_manifold = counts.values().filter(|&&c| c > 2).count();
	(boundary, non_manifold)
}

/// Return a 2-manifold mesh equivalent to `mesh` where possible, separating
/// distinct surface patches that meet at non-manifold edges/vertices.
///
/// Guaranteed **never worse than the input**: the separated mesh is accepted only
/// if it neither opens a boundary nor increases the non-manifold-edge count
/// (separation can isolate a face when non-manifold edges cluster), otherwise the
/// input is returned unchanged. Geometry/volume are always preserved. Idempotent
/// on an already-manifold mesh.
pub fn make_manifold(mesh: &Mesh) -> Mesh {
	let separated = patch_separate(mesh);
	let (b0, n0) = edge_health(mesh);
	let (b1, n1) = edge_health(&separated);
	if b1 <= b0 && n1 <= n0 {
		separated
	} else {
		mesh.clone()
	}
}

/// The patch-separation core (see module docs): one vertex per (vertex, patch),
/// patches being the connected components over unambiguous manifold edges.
fn patch_separate(mesh: &Mesh) -> Mesh {
	let tris: Vec<[u32; 3]> = mesh.triangles().collect();
	let nt = tris.len();
	if nt == 0 {
		return mesh.clone();
	}
	let hec = 3 * nt;
	// Half-edge `h` runs from corner `h%3` to `(h%3+1)%3` of triangle `h/3`.
	let src = |h: usize| tris[h / 3][h % 3];
	let dst = |h: usize| tris[h / 3][(h % 3 + 1) % 3];
	// Corner of triangle `t` located at vertex `x` (its half-edge id).
	let corner_at = |t: usize, x: u32| -> usize { 3 * t + tris[t].iter().position(|&v| v == x).expect("vertex belongs to triangle") };

	// 1. Group half-edges by undirected edge.
	let mut edge_map: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
	for h in 0..hec {
		let (a, b) = (src(h), dst(h));
		edge_map.entry(if a < b { (a, b) } else { (b, a) }).or_default().push(h);
	}

	// 2. Union corners ONLY across unambiguous manifold edges (exactly two
	// half-edges, opposite directions). Non-manifold edges (>2) and same-direction
	// pairs are left un-unioned, so distinct patches stay separate. This never
	// synthesises a twin, so it cannot introduce non-orientable adjacency.
	let mut uf = Uf::new(hec);
	for ((a, b), hes) in &edge_map {
		if hes.len() != 2 {
			continue;
		}
		let (h0, h1) = (hes[0], hes[1]);
		if (src(h0) == *a) == (src(h1) == *a) {
			continue; // same direction → not a clean manifold edge
		}
		let (t0, t1) = (h0 / 3, h1 / 3);
		uf.union(corner_at(t0, *a) as u32, corner_at(t1, *a) as u32);
		uf.union(corner_at(t0, *b) as u32, corner_at(t1, *b) as u32);
	}

	// 3. One output vertex per patch-corner; copy positions/normals; rebuild.
	let has_normals = mesh.normals.len() == mesh.positions.len();
	let mut root_to_new: HashMap<u32, u32> = HashMap::new();
	let mut out = Mesh::new();
	let mut corner_new = vec![0u32; hec];
	for (h, slot) in corner_new.iter_mut().enumerate() {
		let root = uf.find(h as u32);
		let id = *root_to_new.entry(root).or_insert_with(|| {
			let id = out.positions.len() as u32;
			out.positions.push(mesh.positions[src(h) as usize]);
			if has_normals {
				out.normals.push(mesh.normals[src(h) as usize]);
			}
			id
		});
		*slot = id;
	}
	for t in 0..nt {
		out.push_triangle(corner_new[3 * t], corner_new[3 * t + 1], corner_new[3 * t + 2]);
	}
	out
}

#[cfg(test)]
mod tests {
	use super::*;
	use crate::math::Vec3;
	use crate::meshcheck::check_mesh;

	/// A closed outward-oriented tetrahedron from four corner positions, indices
	/// offset by `base` into a shared vertex list.
	fn push_tetra(m: &mut Mesh, corners: [Vec3; 4]) {
		let base = m.positions.len() as u32;
		for c in corners {
			m.push_vertex(c);
		}
		for f in [[0u32, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]] {
			m.push_triangle(base + f[0], base + f[1], base + f[2]);
		}
	}

	#[test]
	fn never_opens_a_boundary_on_open_input() {
		// An OPEN fixture: edge (0,1) shared by four triangles whose other edges
		// are all boundary. Naively separating would isolate the four faces and
		// open more boundary, so the safety gate must return the input unchanged
		// (never worse) rather than "resolve" it into loose triangles.
		let mut m = Mesh::new();
		for p in [
			Vec3::new(0.0, 0.0, 0.0),
			Vec3::new(1.0, 0.0, 0.0),
			Vec3::new(0.5, 1.0, 0.0),
			Vec3::new(0.5, -1.0, 0.0),
			Vec3::new(0.5, 0.0, 1.0),
			Vec3::new(0.5, 0.0, -1.0),
		] {
			m.push_vertex(p);
		}
		m.push_triangle(0, 1, 2);
		m.push_triangle(1, 0, 3);
		m.push_triangle(0, 1, 4);
		m.push_triangle(1, 0, 5);
		let before = check_mesh(&m);
		let fixed = make_manifold(&m);
		let after = check_mesh(&fixed);
		assert!(after.boundary_edges <= before.boundary_edges, "must not open a boundary");
		assert!(after.non_manifold_edges <= before.non_manifold_edges, "must not worsen");
	}

	#[test]
	fn separates_two_tetrahedra_sharing_an_edge() {
		// Two independent closed tetrahedra sharing ONLY edge (0,1), with apexes
		// radially INTERLEAVED around the shared axis — the case the previous
		// consecutive-pairing repair got wrong. Vertices 0,1 are shared.
		let mut m = Mesh::new();
		// Shared edge along +z.
		m.push_vertex(Vec3::new(0.0, 0.0, 0.0)); // 0
		m.push_vertex(Vec3::new(0.0, 0.0, 2.0)); // 1
										   // Tetra A apexes interleaved with B's around the axis.
		let polar = |deg: f32, z: f32| Vec3::new(deg.to_radians().cos(), deg.to_radians().sin(), z);
		m.push_vertex(polar(30.0, 0.5)); // 2  (A)
		m.push_vertex(polar(90.0, 1.5)); // 3  (A)
		m.push_vertex(polar(150.0, 0.5)); // 4 (B)
		m.push_vertex(polar(330.0, 1.5)); // 5 (B)
									// Tetra A on {0,1,2,3}, tetra B on {0,1,4,5}.
		for tet in [[0u32, 1, 2, 3], [0, 1, 4, 5]] {
			for f in [[tet[0], tet[2], tet[1]], [tet[0], tet[1], tet[3]], [tet[0], tet[3], tet[2]], [tet[1], tet[2], tet[3]]] {
				m.push_triangle(f[0], f[1], f[2]);
			}
		}
		assert!(check_mesh(&m).non_manifold_edges >= 1, "shared edge should be non-manifold");

		let fixed = make_manifold(&m);
		let report = check_mesh(&fixed);
		assert_eq!(report.non_manifold_edges, 0, "interleaved separable patches must fully separate");
		// The shared endpoints duplicate (one copy per tetra): 6 → 8 vertices.
		assert_eq!(fixed.vertex_count(), 8, "shared endpoints should split per patch");
		assert!((fixed.signed_volume() - m.signed_volume()).abs() < 1e-9, "volume preserved");
	}

	#[test]
	fn idempotent_on_manifold_mesh() {
		let mut m = Mesh::new();
		push_tetra(&mut m, [Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0)]);
		let v0 = m.signed_volume();
		let fixed = make_manifold(&m);
		assert_eq!(check_mesh(&fixed).non_manifold_edges, 0);
		assert_eq!(fixed.vertex_count(), 4, "manifold mesh is unchanged");
		assert!((fixed.signed_volume() - v0).abs() < 1e-9, "volume must be preserved");
	}
}
