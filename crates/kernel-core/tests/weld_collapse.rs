//! `Mesh::weld` must DROP triangles it collapses. A needle triangle whose two
//! corners weld into one vertex has its two long edges become the SAME segment;
//! keeping it double-counts that edge (incidence 4 with the two legitimate
//! neighbours) and a closed surface stops reading watertight. Found in the wild:
//! the boolean pipeline's face recovery can emit a zero-area sliver face whose
//! two far corners differ by ~1e-9 (sub-weld), and the desk-mount plate of the
//! drawer_system example meshed non-manifold exactly this way.

use kernel_core::{Mesh, Vec3};

#[test]
fn weld_drops_collapsed_needle_triangles() {
	// A unit right triangle split into two triangles along a mid edge, PLUS a
	// zero-width needle between them whose far corners differ by 1e-9 (below the
	// weld tolerance): before the fix the needle survived as (a, m, m) and its
	// long edge counted 4x.
	let a = Vec3::new(0.0, 0.0, 0.0);
	let b = Vec3::new(10.0, 0.0, 0.0);
	let c = Vec3::new(0.0, 10.0, 0.0);
	let m = Vec3::new(5.0, 5.0, 0.0); // midpoint of bc
	let m2 = Vec3::new(5.0, 5.0 + 1e-9, 0.0); // sub-weld twin

	let mut mesh = Mesh::new();
	for tri in [[a, b, m], [a, m2, c], [a, m, m2]] {
		let base = mesh.positions.len() as u32;
		mesh.positions.extend_from_slice(&tri);
		mesh.indices.extend_from_slice(&[base, base + 1, base + 2]);
	}
	mesh.weld(1e-4);

	// The needle must be gone; the two real triangles must survive with the
	// shared edge a-m appearing exactly twice (mesh of an open square: not
	// watertight, but every interior edge manifold).
	let mut incidence: std::collections::HashMap<(u32, u32), u32> = std::collections::HashMap::new();
	for t in mesh.indices.chunks_exact(3) {
		for (p, q) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
			*incidence.entry((p.min(q), p.max(q))).or_insert(0) += 1;
		}
	}
	let max_incidence = incidence.values().max().copied().unwrap_or(0);
	assert!(
		mesh.indices.len() == 6 && mesh.positions.len() == 4 && max_incidence == 2,
		"weld must drop the collapsed needle: {} indices (want 6), {} verts (want 4), max edge incidence {} (want 2)",
		mesh.indices.len(),
		mesh.positions.len(),
		max_incidence,
	);
}
