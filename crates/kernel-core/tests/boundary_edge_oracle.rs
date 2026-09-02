//! Regression: `Mesh::boundary_edge_count()` counts OPENINGS, and agrees with
//! `check_mesh` — the only other implementation of the same question in the tree.
//!
//! The old rule was "a directed edge `a→b` whose reverse `b→a` is absent". That
//! is a different question. Two triangles that share an edge but wind the SAME
//! way close that edge perfectly — there is no rim and nothing to fill — yet
//! `b→a` is absent, so every one of them was reported as a boundary edge.
//! `kernel_core::check_mesh` has always separated the two (boundary = used once,
//! non-orientable = used twice in the same direction), so the tree carried two
//! oracles for one property that disagreed by construction on exactly the meshes
//! where the answer matters.
//!
//! It matters because "the measurement surface is not closed" is a gate: the
//! kernel-api connectivity oracle refuses to report a body count when the
//! tessellation has boundary edges, on the (correct) grounds that a count taken
//! on a cracked surface counts cracks. Fed the wrong boundary count it refused
//! 11 shipped part programs across 8 campaigns whose tessellations are closed
//! and whose body count was right.

use kernel_core::math::Vec3;
use kernel_core::mesh::Mesh;
use kernel_core::check_mesh;

/// An axis-aligned closed box as 12 outward-wound triangles.
fn box_mesh() -> Mesh {
	let v = |x: f32, y: f32, z: f32| Vec3::new(x, y, z);
	let positions = vec![
		v(0.0, 0.0, 0.0), v(10.0, 0.0, 0.0), v(10.0, 10.0, 0.0), v(0.0, 10.0, 0.0),
		v(0.0, 0.0, 10.0), v(10.0, 0.0, 10.0), v(10.0, 10.0, 10.0), v(0.0, 10.0, 10.0),
	];
	let indices = vec![
		0, 2, 1, 0, 3, 2, // -Z
		4, 5, 6, 4, 6, 7, // +Z
		0, 1, 5, 0, 5, 4, // -Y
		3, 7, 6, 3, 6, 2, // +Y
		0, 4, 7, 0, 7, 3, // -X
		1, 2, 6, 1, 6, 5, // +X
	];
	Mesh { positions, indices, ..Default::default() }
}

/// A closed tetrahedron with `flipped` of its four triangles wound inside-out.
/// Every edge is still used by exactly two triangles, so nothing is open.
fn tet(flipped: usize) -> Mesh {
	let positions =
		vec![Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.0, 0.0, 0.0)];
	let faces = [[0u32, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]];
	let mut indices = Vec::new();
	for (i, f) in faces.iter().enumerate() {
		if i < flipped {
			indices.extend_from_slice(&[f[0], f[2], f[1]]);
		} else {
			indices.extend_from_slice(f);
		}
	}
	Mesh { positions, indices, normals: Vec::new() }
}

/// An open tetrahedron: the same body with one face removed, leaving a real
/// three-edge rim.
fn open_tet() -> Mesh {
	let mut m = tet(0);
	m.indices.truncate(9);
	m
}

#[test]
fn a_closed_surface_has_no_boundary_edges_however_it_winds() {
	for flipped in 0..=4 {
		let m = tet(flipped);
		let r = check_mesh(&m);
		assert_eq!(r.boundary_edges, 0, "check_mesh sees no opening with {flipped} faces flipped");
		assert_eq!(
			m.boundary_edge_count(),
			0,
			"a closed tetrahedron with {flipped} of 4 faces wound inside-out has NO opening, but \
			 boundary_edge_count() reported {} (check_mesh: boundary {}, non-orientable {})",
			m.boundary_edge_count(),
			r.boundary_edges,
			r.non_orientable_edges
		);
	}
	// The winding defect is still visible — it is just not called an opening.
	let m = tet(1);
	assert_eq!(check_mesh(&m).non_orientable_edges, 3, "one flipped face makes its three edges non-orientable");
	assert!(!m.is_two_manifold(), "a non-orientable surface is not a 2-manifold");
}

#[test]
fn a_real_opening_is_still_counted() {
	let m = open_tet();
	assert_eq!(m.boundary_edge_count(), 3, "a missing face leaves a three-edge rim");
	assert_eq!(check_mesh(&m).boundary_edges, 3);
}

#[test]
fn the_two_oracles_agree_on_every_shape() {
	let mut cases: Vec<(String, Mesh)> = Vec::new();
	for flipped in 0..=4 {
		cases.push((format!("tet flipped={flipped}"), tet(flipped)));
	}
	cases.push(("open tet".into(), open_tet()));
	cases.push(("box".into(), box_mesh()));
	let mut flipped_box = box_mesh();
	flipped_box.indices.swap(1, 2); // one face wound inside-out: closed, non-orientable
	cases.push(("box with one flipped triangle".into(), flipped_box));
	let mut open_box = box_mesh();
	open_box.indices.truncate(open_box.indices.len() - 3);
	cases.push(("box minus one triangle".into(), open_box));
	for (name, m) in &cases {
		let r = check_mesh(m);
		assert_eq!(
			m.boundary_edge_count(),
			r.boundary_edges,
			"{name}: Mesh::boundary_edge_count and check_mesh must be one oracle"
		);
		assert_eq!(
			m.non_orientable_edge_count(),
			r.non_orientable_edges,
			"{name}: Mesh::non_orientable_edge_count and check_mesh must be one oracle"
		);
	}
}

#[test]
fn fill_holes_does_not_cap_a_closed_surface() {
	// The repair is driven by the same rule as the measure, so a closed but
	// badly-wound surface has nothing to fill. Capping one would push a third
	// triangle onto an edge that already had two — a winding defect promoted to
	// a non-manifold one.
	let mut m = tet(1);
	let before = m.triangle_count();
	assert_eq!(m.fill_holes(), 0, "nothing to fill on a closed surface");
	assert_eq!(m.triangle_count(), before, "fill_holes invented geometry on a closed surface");
	assert_eq!(check_mesh(&m).non_manifold_edges, 0);

	// ...and a genuine hole is still filled.
	let mut open = open_tet();
	assert_eq!(open.fill_holes(), 1);
	assert_eq!(open.boundary_edge_count(), 0, "the rim is closed after the repair");
}
