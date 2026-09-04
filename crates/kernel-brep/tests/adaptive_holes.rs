// Copyright (c) LMCAD. Licensed under the MIT License.

//! The sealed-hole family (campaign themes T6(b)/(c) + T15): the ADAPTIVE
//! tessellation path — the one every export and measurement mesh rides — used to
//! assemble a face from its OUTER loop only, dropping `Face::inner` entirely.
//! A plate with a hole tessellated as if the hole were skin: the hole's tube was
//! disconnected from the caps (boundary edges, `components` over-count), a
//! trivially exact part was demoted to `voxel_healed` on export, and
//! `support_report` measured hole area as face area. These tests pin the fix:
//! adaptive-tessellated holed faces must produce CLOSED, one-body, orientable
//! meshes whose area/volume match the analytic truth.

use glam::{DVec2, DVec3};
use kernel_brep::{cylinder, difference, extrude_with_holes, tessellate_adaptive_tol};

/// 40x20x5 plate with two square 6x6 through-holes (the fix-phase t6b repro).
fn holed_plate() -> kernel_brep::Solid {
	let outer = vec![DVec2::new(0.0, 0.0), DVec2::new(40.0, 0.0), DVec2::new(40.0, 20.0), DVec2::new(0.0, 20.0)];
	let sq = |x0: f64| vec![DVec2::new(x0, 8.0), DVec2::new(x0 + 6.0, 8.0), DVec2::new(x0 + 6.0, 14.0), DVec2::new(x0, 14.0)];
	extrude_with_holes(&outer, &[sq(8.0), sq(24.0)], 5.0)
}

#[test]
fn adaptive_mesh_of_holed_plate_is_closed_one_body_and_orientable() {
	let s = holed_plate();
	let m = tessellate_adaptive_tol(&s, 0.05);

	// The three independent oracles the campaigns gate on, all on the SAME mesh
	// the exports ship: closed (no boundary edges), one body, consistently wound.
	assert_eq!(
		m.boundary_edge_count(),
		0,
		"holed plate's measurement mesh must be CLOSED — dropped inner loops leave the hole tubes unstitched"
	);
	assert_eq!(m.component_count(1e-3), 1, "one body — the hole tubes must be connected to the caps");
	assert_eq!(m.non_orientable_edge_count(), 0, "consistently wound — bridged hole caps must not double-cover");
	assert!(m.is_watertight(), "watertight in the edge-closure sense");
}

#[test]
fn adaptive_mesh_of_holed_plate_measures_the_true_volume_and_area() {
	let s = holed_plate();
	let m = tessellate_adaptive_tol(&s, 0.05);

	// Closed form: (40*20 - 2 * 6*6) * 5 = 3640 mm^3. All faces planar, so the
	// tessellation must be exact to float tolerance — a sealed hole reads 4000.
	let v = m.signed_volume().abs();
	assert!((v - 3640.0).abs() < 1e-4 * 3640.0, "faceted volume {v} must equal the closed form 3640 (sealed holes read 4000)");

	// Total surface area, closed form: caps 2*(800-72) = 1456, outer walls
	// 2*(40+20)*5 = 600, hole walls 2 * 4*6*5 = 240 -> 2296 mm^2.
	let area: f64 = m
		.indices
		.chunks_exact(3)
		.map(|t| {
			let (a, b, c) =
				(m.positions[t[0] as usize].as_dvec3(), m.positions[t[1] as usize].as_dvec3(), m.positions[t[2] as usize].as_dvec3());
			(b - a).cross(c - a).length() * 0.5
		})
		.sum();
	assert!(
		(area - 2296.0).abs() < 1e-4 * 2296.0,
		"total area {area} must equal the closed form 2296 (a sealed hole reads 2344 and support_report inherits the lie)"
	);
}

#[test]
fn annular_cap_from_a_boolean_stays_clean_through_the_adaptive_path() {
	// A washer: cylinder minus coaxial through-cylinder. The boolean's loop-aware
	// result carries the bore as an INNER loop on both caps — the T15 shape
	// family (there it was a rotated tube; cap-with-inner-loop is the essence).
	let outer = cylinder(DVec3::ZERO, DVec3::Z, 12.0, 6.0, 96);
	let bore = cylinder(DVec3::new(0.0, 0.0, -1.0), DVec3::Z, 5.0, 8.0, 96);
	let washer = difference(&outer, &bore);
	let m = tessellate_adaptive_tol(&washer, 0.05);

	assert_eq!(m.boundary_edge_count(), 0, "washer mesh must be closed");
	assert_eq!(m.component_count(1e-3), 1, "washer is one body");
	assert_eq!(
		m.non_orientable_edge_count(),
		0,
		"annular caps must triangulate without double-cover (T15: the hole-bridge used to overlap itself)"
	);
	assert!(!m.has_self_intersection(), "no crossing triangles on the annular caps");
}

#[test]
fn fine_tolerance_washer_grid_stays_orientable() {
	// The same washer at a chord tolerance fine enough to push the curved-face
	// grids to `segs > 1`. The grid used to wind every interior triangle
	// against the face ring's single Newell normal — near-degenerate for a
	// half-barrel bore wall, flipping interior grid rows on the far side of
	// the arc (measured: 24 non-orientable edges at tol 0.005 on exactly this
	// solid). Winding now uses the analytic normal at each triangle's own
	// centroid with a per-face aggregate sign vote; this pins it.
	let outer = cylinder(DVec3::ZERO, DVec3::Z, 8.0, 6.0, 96);
	let bore = cylinder(DVec3::new(0.0, 0.0, -1.0), DVec3::Z, 5.0, 8.0, 96);
	let washer = difference(&outer, &bore);
	let m = tessellate_adaptive_tol(&washer, 0.005);

	assert_eq!(m.boundary_edge_count(), 0, "fine washer mesh must be closed");
	assert_eq!(
		m.non_orientable_edge_count(),
		0,
		"interior grid rows of the bore wall must wind with the LOCAL surface normal, not the ring's average"
	);
	assert_eq!(m.component_count(1e-3), 1, "fine washer is one body");
}
