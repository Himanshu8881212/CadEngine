//! `Mesh::support_free_report` — the bed/bridge-aware refinement of
//! `overhang_analysis`. A printer needs support only for *steep* downward
//! surface: first-layer bed contact and flat bridged ceilings print fine, and
//! the naive overhang flag lumps all three together.

use kernel_core::{Mesh, Vec3};

/// Push one triangle wound so its geometric normal matches the intent.
fn tri(mesh: &mut Mesh, a: Vec3, b: Vec3, c: Vec3) {
	let base = mesh.positions.len() as u32;
	mesh.positions.extend_from_slice(&[a, b, c]);
	mesh.indices.extend_from_slice(&[base, base + 1, base + 2]);
}

#[test]
fn support_free_report_classifies_bed_bridge_and_steep() {
	let mut m = Mesh::new();
	let v = |x: f32, y: f32, z: f32| Vec3::new(x, y, z);

	// 20×20 floor at z=0, facing down: BED contact (2 triangles, 400 mm²).
	tri(&mut m, v(0.0, 0.0, 0.0), v(20.0, 20.0, 0.0), v(20.0, 0.0, 0.0));
	tri(&mut m, v(0.0, 0.0, 0.0), v(0.0, 20.0, 0.0), v(20.0, 20.0, 0.0));
	// The same square as a flat ceiling at z=10: one connected BRIDGE patch
	// (400 mm²; true span = its min-direction extent = 20 — the metric reports
	// the narrow bridging direction, not the AABB diagonal).
	tri(&mut m, v(0.0, 0.0, 10.0), v(20.0, 20.0, 10.0), v(20.0, 0.0, 10.0));
	tri(&mut m, v(0.0, 0.0, 10.0), v(0.0, 20.0, 10.0), v(20.0, 20.0, 10.0));
	// A ceiling tilted 30° off horizontal (normal 60° from vertical, n·up ≈
	// −0.866): STEEP — needs support. Area = ½·10·10 = 50 mm².
	// Plane basis: e1 = +X, e2 = n × e1 = (0, −0.866, −0.5).
	tri(&mut m, v(30.0, 0.0, 8.0), v(40.0, 0.0, 8.0), v(30.0, -8.6603, 3.0));
	// A vertical wall (n·up = 0): never flagged. Area 50 mm².
	tri(&mut m, v(50.0, 0.0, 0.0), v(50.0, 10.0, 0.0), v(50.0, 0.0, 10.0));

	m.weld(1e-6); // patch connectivity is by shared vertex — weld like real meshes
	let r = m.support_free_report(Vec3::Z, 45.0, 0.3);
	let steep_flags = r.steep.iter().filter(|s| **s).count();
	assert!(
		(r.bed_area - 400.0).abs() < 0.5
			&& (r.bridge_area - 400.0).abs() < 0.5
			&& (r.steep_area - 50.0).abs() < 0.5
			&& (r.total_area - 900.0).abs() < 1.0
			&& (r.max_bridge_span - 20.0).abs() < 0.05
			&& steep_flags == 1
			&& r.steep.len() == 6,
		"support-free classification wrong: bed={:.2} (want 400) bridge={:.2} (want 400) steep={:.2} (want 50) \
		 total={:.2} (want 900) span={:.3} (want 20) steep_flags={steep_flags}/6 (want exactly the 30° ceiling)",
		r.bed_area,
		r.bridge_area,
		r.steep_area,
		r.total_area,
		r.max_bridge_span,
	);
}
