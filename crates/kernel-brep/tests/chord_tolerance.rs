// Copyright (c) LMCAD. Licensed under the MIT License.

//! Watertight chord-tolerance tessellation: facets bounded to a deviation tolerance
//! while shared edges stay crack-free.

use kernel_brep::math::DVec3;
use kernel_brep::{cylinder, sphere, tessellate_adaptive_tol, Surface};

#[test]
fn adaptive_tol_is_watertight_and_refines_with_tolerance() {
	// Mixed curved/planar solids (cylinder = curved sides + planar caps) are the
	// case where naive per-face refinement cracks. `tessellate_adaptive_tol` must
	// stay watertight at every tolerance while adding triangles as `tol` tightens.
	let cyl = || cylinder(DVec3::ZERO, DVec3::Z, 20.0, 40.0, 6);
	let sph = || sphere(DVec3::ZERO, 20.0, 8, 6);

	let mut prev_cyl = 0usize;
	for &tol in &[1.0, 0.5, 0.1, 0.02] {
		for s in [cyl(), sph()] {
			let m = tessellate_adaptive_tol(&s, tol);
			assert!(
				m.is_watertight(),
				"adaptive_tol must stay watertight at tol={tol} ({} non-manifold edges)",
				m.non_manifold_edge_count()
			);
		}
		let n = tessellate_adaptive_tol(&cyl(), tol).triangle_count();
		assert!(n >= prev_cyl, "tighter tolerance must not reduce triangles: {prev_cyl} -> {n}");
		prev_cyl = n;
	}
	// A tight tolerance genuinely refines beyond the control facet.
	assert!(prev_cyl > tessellate_adaptive_tol(&cyl(), 100.0).triangle_count(), "tight tol should add triangles");
}

#[test]
fn adaptive_tol_bounds_the_chord_deviation() {
	// Every lateral facet of a tolerance-tessellated cylinder lies within ~tol of
	// the true surface (caps excluded by their constant z; their rim verts also sit
	// on the cylinder).
	let base = DVec3::ZERO;
	let (radius, height, tol) = (20.0, 40.0, 0.05);
	let surf = Surface::Cylinder { origin: base, axis: DVec3::Z, radius };
	let m = tessellate_adaptive_tol(&cylinder(base, DVec3::Z, radius, height, 6), tol);

	let mut max_dev = 0.0f64;
	for t in m.indices.chunks_exact(3) {
		let p = [
			m.positions[t[0] as usize].as_dvec3(),
			m.positions[t[1] as usize].as_dvec3(),
			m.positions[t[2] as usize].as_dvec3(),
		];
		let on_cyl = p.iter().all(|&v| surf.signed_value(v).abs() < 1e-3);
		let cap = p.iter().all(|v| v.z.abs() < 1e-3) || p.iter().all(|v| (v.z - height).abs() < 1e-3);
		if on_cyl && !cap {
			for &(i, j) in &[(0, 1), (1, 2), (2, 0)] {
				max_dev = max_dev.max(surf.signed_value((p[i] + p[j]) * 0.5).abs());
			}
		}
	}
	assert!(max_dev > 0.0 && max_dev <= tol * 1.5, "lateral chord deviation {max_dev} should be within ~{tol}");
}

#[test]
fn adaptive_tol_reaches_micron_smoothness() {
	// Resin-print precision from the EXACT path: at a 5-micron chord tolerance the cylinder
	// tessellation is watertight, fine, and every lateral facet lies within ~5 micron of the
	// true surface — a smoothness a voxel grid cannot reach on a part this size. This is what
	// lets a precision shank/shaft mesh crisply without the voxel half.
	let base = DVec3::ZERO;
	let (radius, height, tol) = (4.0, 20.0, 0.005);
	let surf = Surface::Cylinder { origin: base, axis: DVec3::Z, radius };
	let m = tessellate_adaptive_tol(&cylinder(base, DVec3::Z, radius, height, 12), tol);
	assert!(m.is_watertight(), "micron-tol cylinder must be watertight");

	let mut max_dev = 0.0f64;
	for t in m.indices.chunks_exact(3) {
		let p = [
			m.positions[t[0] as usize].as_dvec3(),
			m.positions[t[1] as usize].as_dvec3(),
			m.positions[t[2] as usize].as_dvec3(),
		];
		let on_cyl = p.iter().all(|&v| surf.signed_value(v).abs() < 1e-3);
		let cap = p.iter().all(|v| v.z.abs() < 1e-3) || p.iter().all(|v| (v.z - height).abs() < 1e-3);
		if on_cyl && !cap {
			for &(i, j) in &[(0, 1), (1, 2), (2, 0)] {
				max_dev = max_dev.max(surf.signed_value((p[i] + p[j]) * 0.5).abs());
			}
		}
	}
	assert!(
		m.triangle_count() > 800 && max_dev <= tol * 1.5,
		"micron tessellation: {} tris (want >800, refined beyond the 12-seg control), chord dev {max_dev} should be within ~{tol}",
		m.triangle_count()
	);
}
