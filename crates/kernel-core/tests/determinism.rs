// Copyright (c) LMCAD. Licensed under the MIT License.

//! Bit-level determinism of the kernel-core mesh-repair pipeline.
//!
//! `Mesh::fill_holes` used to collect boundary edges by iterating a `HashSet`,
//! whose order is seeded per-instance: identical input meshes were repaired
//! differently run to run (loop start vertices, cap emission order, and — at a
//! pinch vertex shared by two holes — even WHICH edges were spliced into one
//! cap, changing the vertex count). This test rebuilds the same defective mesh
//! 40 times in one process (every rebuild allocates fresh, differently-seeded
//! hash maps) and demands a byte-identical result from the full repair chain
//! `weld` → `make_manifold` → `fill_holes`.

use kernel_core::{make_manifold, Mesh, Vec3};

/// A deliberately defective fixture, built directly from triangles:
///
/// - a 4×4-cell planar grid pushed as an UNWELDED soup (per-triangle vertices,
///   exercising `weld`), with cells (1,1) and (2,2) omitted — two square holes
///   whose rims share the pinch vertex (2,2,0), the case where `fill_holes`'
///   greedy walk has a genuine successor choice;
/// - two closed tetrahedra sharing only the edge (0,0,10)–(0,0,12) with their
///   apexes radially interleaved — a non-manifold edge between two separable
///   patches, exercising `make_manifold`'s patch separation.
fn defective_fixture() -> Mesh {
	let mut m = Mesh::new();
	// Plate soup: grid corner (i, j) sits at (i, j, 0), spacing 1.
	let corner = |i: u32, j: u32| Vec3::new(i as f32, j as f32, 0.0);
	let mut soup_tri = |a: Vec3, b: Vec3, c: Vec3| {
		let i0 = m.push_vertex(a);
		let i1 = m.push_vertex(b);
		let i2 = m.push_vertex(c);
		m.push_triangle(i0, i1, i2);
	};
	for i in 0..4u32 {
		for j in 0..4u32 {
			if (i, j) == (1, 1) || (i, j) == (2, 2) {
				continue; // the two holes, pinched at grid corner (2, 2)
			}
			soup_tri(corner(i, j), corner(i + 1, j), corner(i + 1, j + 1));
			soup_tri(corner(i, j), corner(i + 1, j + 1), corner(i, j + 1));
		}
	}
	// Two tetrahedra sharing one edge, apexes interleaved around it (away from
	// the plate so welding cannot touch them).
	let e0 = m.push_vertex(Vec3::new(0.0, 0.0, 10.0));
	let e1 = m.push_vertex(Vec3::new(0.0, 0.0, 12.0));
	let polar = |deg: f32, z: f32| Vec3::new(deg.to_radians().cos(), deg.to_radians().sin(), z);
	let a0 = m.push_vertex(polar(30.0, 10.5));
	let a1 = m.push_vertex(polar(90.0, 11.5));
	let b0 = m.push_vertex(polar(150.0, 10.5));
	let b1 = m.push_vertex(polar(330.0, 11.5));
	for tet in [[e0, e1, a0, a1], [e0, e1, b0, b1]] {
		for f in [[tet[0], tet[2], tet[1]], [tet[0], tet[1], tet[3]], [tet[0], tet[3], tet[2]], [tet[1], tet[2], tet[3]]] {
			m.push_triangle(f[0], f[1], f[2]);
		}
	}
	m
}

/// Order-sensitive FNV-1a over the exact output bytes that matter: vertex and
/// index counts, every position's IEEE bit pattern in index order, then every
/// triangle index. Any reordering, renumbering, or coordinate change flips it.
fn content_hash(m: &Mesh) -> u64 {
	let mut h: u64 = 0xcbf2_9ce4_8422_2325;
	let mut eat = |v: u64| h = (h ^ v).wrapping_mul(0x0000_0100_0000_01b3);
	eat(m.positions.len() as u64);
	eat(m.indices.len() as u64);
	for p in &m.positions {
		eat(p.x.to_bits() as u64);
		eat(p.y.to_bits() as u64);
		eat(p.z.to_bits() as u64);
	}
	for &i in &m.indices {
		eat(i as u64);
	}
	h
}

/// One full repair run over a fresh fixture. Two branches:
///
/// - the canonical chain `weld` → `make_manifold` → `fill_holes` (note
///   `make_manifold` splits the 2-fan pinch vertex, so the two hole rims join
///   into one topologically-forced 8-edge loop before filling);
/// - `weld` → `fill_holes` directly, where the pinch is still intact and the
///   greedy walk's successor choice at it decides splice-into-one-cap versus
///   two separate caps — pre-fix this varied the *vertex and fill counts*, not
///   just the bytes.
///
/// The snapshot is every count the pipeline emits plus a content hash of each
/// final mesh.
type Snapshot = (usize, usize, usize, usize, usize, usize, usize, u64, usize, usize, usize, u64);

fn repair_snapshot() -> Snapshot {
	let mut m = defective_fixture();
	m.weld(1e-3);
	let (vw, tw) = (m.vertex_count(), m.triangle_count());
	let mut chained = make_manifold(&m);
	let (vm, tm) = (chained.vertex_count(), chained.triangle_count());
	let filled = chained.fill_holes();

	let mut direct = m.clone();
	let direct_filled = direct.fill_holes();

	(
		vw,
		tw,
		vm,
		tm,
		filled,
		chained.vertex_count(),
		chained.triangle_count(),
		content_hash(&chained),
		direct_filled,
		direct.vertex_count(),
		direct.triangle_count(),
		content_hash(&direct),
	)
}

#[test]
fn mesh_repair_chain_is_bit_deterministic_in_process() {
	let runs: Vec<Snapshot> = (0..40).map(|_| repair_snapshot()).collect();
	let distinct: std::collections::BTreeSet<Snapshot> = runs.iter().copied().collect();
	// Printed so repeated test-binary invocations can be diffed for
	// cross-process determinism as well.
	println!("repair snapshot (chain v/t/filled/hash, then direct-fill filled/v/t/hash): {:?}", runs[0]);
	assert_eq!(
		distinct.len(),
		1,
		"40 in-process repairs of the identical mesh must be bit-identical, got {} distinct snapshots \
		 (v_weld, t_weld, v_manifold, t_manifold, holes_filled, v_final, t_final, content_hash, \
		 direct_filled, direct_v, direct_t, direct_hash): {:?}",
		distinct.len(),
		distinct
	);
}
