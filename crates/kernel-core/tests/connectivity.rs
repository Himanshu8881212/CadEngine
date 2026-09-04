// Copyright (c) LMCAD. Licensed under the MIT License.

//! Connectivity as a THIRD oracle (promoted 2026-07-31).
//!
//! Earned by the cold-start exam: a session building `hook_system/drill_hook`
//! produced a part severed into two floating lumps that passed `validate`,
//! `is_watertight`, `volume`, every clearance/stress/sweep/export gate, AND
//! rendered convincingly. `Solid::shell_count()` reported 1 for it too, because
//! it counts B-rep shell records rather than connected geometry. Nothing in the
//! engine's validity vocabulary could see the defect.
//!
//! These pins assert the property that makes `component_count` worth having:
//! it must disagree with the other oracles exactly when they are blind.

use kernel_core::math::Vec3;
use kernel_core::mesh::Mesh;

/// An axis-aligned closed box as 12 triangles, offset by `dx` in X.
fn box_mesh(dx: f32) -> Mesh {
	let (a, b) = (Vec3::new(dx, 0.0, 0.0), Vec3::new(dx + 10.0, 10.0, 10.0));
	let v = |x: f32, y: f32, z: f32| Vec3::new(x, y, z);
	let positions = vec![
		v(a.x, a.y, a.z),
		v(b.x, a.y, a.z),
		v(b.x, b.y, a.z),
		v(a.x, b.y, a.z),
		v(a.x, a.y, b.z),
		v(b.x, a.y, b.z),
		v(b.x, b.y, b.z),
		v(a.x, b.y, b.z),
	];
	// Outward-facing winding, 2 triangles per face.
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

/// Two boxes merged into ONE mesh object but not touching — the shape of the
/// defect: a single mesh that is secretly two bodies.
fn two_disjoint_boxes() -> Mesh {
	let a = box_mesh(0.0);
	let b = box_mesh(50.0);
	let off = a.positions.len() as u32;
	let mut m = a;
	m.positions.extend_from_slice(&b.positions);
	m.indices.extend(b.indices.iter().map(|i| i + off));
	m
}

#[test]
fn connectivity_sees_what_validity_and_watertightness_cannot() {
	let one = box_mesh(0.0);
	let two = two_disjoint_boxes();

	// The severed mesh is watertight and correctly measured -- that is exactly
	// why it slipped through every gate in the field.
	let two_watertight = two.is_watertight();
	let one_vol = one.signed_volume().abs();
	let two_vol = two.signed_volume().abs();
	let volume_looks_plausible = (two_vol - 2.0 * one_vol).abs() < 1e-3;

	// ...and connectivity is the only oracle that disagrees.
	let one_n = one.component_count(1e-3);
	let two_n = two.component_count(1e-3);

	assert!(
		two_watertight && volume_looks_plausible && one_n == 1 && two_n == 2 && one.is_one_body() && !two.is_one_body(),
		"connectivity oracle broken. The whole point is that the SEVERED mesh still looks fine to the other \
		 oracles and only component_count objects: severed.is_watertight()={two_watertight} (expect true — \
		 this is the trap) · severed volume={two_vol} vs 2x single {one_vol} plausible={volume_looks_plausible} \
		 (expect true) · single component_count={one_n} (expect 1) · severed component_count={two_n} (expect 2) \
		 · is_one_body single={} severed={} (expect true/false)",
		one.is_one_body(),
		two.is_one_body(),
	);
}

#[test]
fn component_count_edge_cases_are_defined() {
	// Empty is 0 bodies, not 1 -- an empty result is a failed op, and must not
	// read as "one nice connected part".
	let empty = Mesh::default().component_count(1e-3);

	// Two boxes sharing a face weld into ONE body at the house tolerance.
	let mut touching = box_mesh(0.0);
	let b = box_mesh(10.0); // starts exactly where the first ends
	let off = touching.positions.len() as u32;
	touching.positions.extend_from_slice(&b.positions);
	touching.indices.extend(b.indices.iter().map(|i| i + off));
	let touching_n = touching.component_count(1e-3);

	// A stray unreferenced vertex is not a body.
	let mut stray = box_mesh(0.0);
	stray.positions.push(Vec3::new(999.0, 999.0, 999.0));
	let stray_n = stray.component_count(1e-3);

	assert!(
		empty == 0 && touching_n == 1 && stray_n == 1,
		"component_count edge cases: empty={empty} (want 0 — an empty op result must not read as one body) \
		 face-touching pair={touching_n} (want 1 — coincident vertices weld at the house tolerance) \
		 unreferenced-vertex box={stray_n} (want 1 — a stray position is not a body)"
	);
}
