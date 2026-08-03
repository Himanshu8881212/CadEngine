// Copyright (c) LMCAD. Licensed under the MIT License.

//! Full-f64 Surface Nets: it must resolve a fine feature held far from the origin,
//! where the f32 grid mesher loses all precision.

use std::f64::consts::PI;

use kernel_core::math::DVec3;
use kernel_core::{dual_contour_f64, surface_nets_f64, MeshF64};

fn sphere_field(center: DVec3, r: f64) -> impl Fn(DVec3) -> f64 {
	move |p| (p - center).length() - r
}

fn box_field(center: DVec3, half: DVec3) -> impl Fn(DVec3) -> f64 {
	move |p| {
		let q = (p - center).abs() - half;
		q.max(DVec3::ZERO).length() + q.max_element().min(0.0)
	}
}

#[test]
fn f64_surface_nets_resolves_a_unit_sphere_at_a_huge_offset() {
	// At a 1e7 offset a 1 mm grid step is below f32's ~7 significant digits, so the
	// f32 mesher cannot represent the lattice; f64 resolves the unit sphere cleanly.
	let center = DVec3::new(1.0e7, 0.0, 0.0);
	let r = 1.0;
	let mesh = surface_nets_f64(sphere_field(center, r), center - DVec3::splat(1.5), center + DVec3::splat(1.5), 0.1);

	assert!(mesh.triangle_count() > 200, "should finely mesh the sphere, got {}", mesh.triangle_count());
	// Surface-nets vertices sit within ~one voxel of the true surface — and that
	// accuracy survives the 1e7 offset only because the arithmetic is f64.
	for &p in &mesh.positions {
		let off = ((p - center).length() - r).abs();
		assert!(off < 0.12, "vertex off the unit sphere at 1e7 offset: {off}");
	}
	// Centroid-relative volume stays accurate despite the offset: 4/3·π ≈ 4.18879.
	let vol = mesh.signed_volume();
	assert!((vol - 4.0 / 3.0 * PI).abs() < 0.2, "sphere volume {vol} (expected ≈4.189)");
}

#[test]
fn f64_dual_contour_keeps_box_corners_sharper_than_surface_nets() {
	// Box off the grid so its faces don't fall on lattice points. DC's QEF places
	// vertices on the edges/corners (volume ≈ exact 8); Surface Nets rounds them.
	let half = DVec3::splat(1.0);
	let center = DVec3::new(0.05, 0.03, 0.07);
	let (lo, hi, vox) = (DVec3::splat(-1.5), DVec3::splat(1.5), 0.15);
	let dc = dual_contour_f64(box_field(center, half), lo, hi, vox);
	let sn = surface_nets_f64(box_field(center, half), lo, hi, vox);

	assert!(dc.triangle_count() > 100);
	let (vdc, vsn) = (dc.signed_volume(), sn.signed_volume());
	assert!((vdc - 8.0).abs() < 0.3, "DC box volume {vdc} (expected ≈8)");
	assert!(vsn < vdc - 0.05, "DC should preserve sharper corners than SN: dc={vdc} sn={vsn}");
}

#[test]
fn f64_dual_contour_works_at_a_huge_offset() {
	let center = DVec3::new(0.0, 1.0e7, 0.0);
	let dc = dual_contour_f64(box_field(center, DVec3::splat(1.0)), center - DVec3::splat(1.5), center + DVec3::splat(1.5), 0.15);
	assert!(dc.triangle_count() > 100, "DC must mesh the box at 1e7 offset");
	assert!((dc.signed_volume() - 8.0).abs() < 0.4, "box volume {} at 1e7 offset", dc.signed_volume());
}

#[test]
fn f64_obj_export_round_trips_vertices_at_full_precision() {
	let center = DVec3::new(1.0e7, 0.0, 0.0);
	let mesh = surface_nets_f64(sphere_field(center, 1.0), center - DVec3::splat(1.5), center + DVec3::splat(1.5), 0.2);
	let obj = mesh.to_obj();
	let v_lines: Vec<&str> = obj.lines().filter(|l| l.starts_with("v ")).collect();
	let f_count = obj.lines().filter(|l| l.starts_with("f ")).count();
	assert_eq!(v_lines.len(), mesh.positions.len());
	assert_eq!(f_count, mesh.triangle_count());

	// The first vertex parses back to its exact f64 value despite the 1e7 offset —
	// an f32 export would lose ~0.5 of resolution at this magnitude.
	let p: Vec<f64> = v_lines[0].split_whitespace().skip(1).map(|t| t.parse().unwrap()).collect();
	let orig = mesh.positions[0];
	assert!((p[0] - orig.x).abs() < 1e-6 && (p[1] - orig.y).abs() < 1e-6 && (p[2] - orig.z).abs() < 1e-6);
	assert!((p[0] - 1.0e7).abs() < 2.0, "x {} should be a surface point near 1e7", p[0]);
}

#[test]
fn f64_stl_export_is_well_formed() {
	let m = surface_nets_f64(sphere_field(DVec3::ZERO, 1.0), DVec3::splat(-1.5), DVec3::splat(1.5), 0.2);
	let stl = m.to_stl_binary();
	// Binary STL: 80-byte header + u32 count + 50 bytes per triangle.
	assert_eq!(stl.len(), 84 + 50 * m.triangle_count());
	let n = u32::from_le_bytes([stl[80], stl[81], stl[82], stl[83]]) as usize;
	assert_eq!(n, m.triangle_count());
}

#[test]
fn f64_surface_nets_rejects_degenerate_domains() {
	let bad = surface_nets_f64(sphere_field(DVec3::ZERO, 1.0), DVec3::ZERO, DVec3::ZERO, 0.1);
	assert_eq!(bad.triangle_count(), 0, "zero-size domain yields an empty mesh");
	let nan = surface_nets_f64(sphere_field(DVec3::ZERO, 1.0), DVec3::splat(-2.0), DVec3::splat(2.0), f64::NAN);
	assert_eq!(nan.triangle_count(), 0, "non-finite voxel yields an empty mesh");
	assert_eq!(MeshF64::default().signed_volume(), 0.0);
}
