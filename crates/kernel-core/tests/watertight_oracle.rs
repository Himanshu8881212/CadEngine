//! Regression: the rigorous `Mesh::is_two_manifold()` rejects a closed-edge but
//! non-2-manifold mesh (bowtie / pinched vertex) that `is_watertight()` (edge
//! closure only) accepts — documenting the distinction the two methods draw.

use kernel_core::{Mesh, Vec3};

#[test]
fn is_watertight_rejects_bowtie_vertex() {
	// Two tetrahedra sharing ONLY vertex 3. Every edge is still used exactly twice,
	// so non_manifold_edge_count()==0 (the old oracle's blind spot), but vertex 3's
	// incident triangles form two disjoint fans -> not a 2-manifold.
	let positions = vec![
		Vec3::new(1.0, 0.0, 0.0),
		Vec3::new(0.0, 1.0, 0.0),
		Vec3::new(0.0, 0.0, 1.0),
		Vec3::ZERO,
		Vec3::new(-1.0, 0.0, 0.0),
		Vec3::new(0.0, -1.0, 0.0),
		Vec3::new(0.0, 0.0, -1.0),
	];
	let indices = vec![
		0u32, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3, // tet A (closed, orientable)
		3, 5, 4, 3, 4, 6, 3, 6, 5, 4, 5, 6, // tet B sharing vertex 3
	];
	let m = Mesh { positions, indices, normals: Vec::new() };
	assert_eq!(m.non_manifold_edge_count(), 0, "all edges are shared exactly twice");
	assert!(
		m.is_watertight() && !m.is_two_manifold(),
		"edge-closed (is_watertight=true) but NOT a 2-manifold (is_two_manifold must be false): \
		 is_watertight={} is_two_manifold={}",
		m.is_watertight(),
		m.is_two_manifold()
	);
}
