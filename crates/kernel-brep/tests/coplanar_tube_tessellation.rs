//! A hollow TUBE abutting a planar face EXACTLY coplanar (annular footprint = an
//! inner loop coplanar with the plane) used to tessellate NON-watertight while
//! the B-rep stayed valid — a FRICTION #20 / R2–R3 class gap found building a
//! NEMA-17 harmonic-drive housing. FIXED 2026-07-02 by weld hygiene:
//! `Mesh::weld` now drops triangles it collapses (the coplanar seam's recovery
//! left zero-area needle fragments whose long edges double-counted after
//! welding — the same root cause as the drawer_system desk-dock plate, see
//! `recovery_needle_weld.rs`). This test pins the FIXED behaviour, keeps the
//! box/solid-cylinder controls, and keeps the overlap case green.

use kernel_brep::math::DVec3;
use kernel_brep::{cuboid, cylinder, difference, tessellate_default, union, validate};

fn tube(z0: f64) -> kernel_brep::Solid {
	difference(
		&cylinder(DVec3::new(0.0, 0.0, z0), DVec3::Z, 12.0, 10.0, 64),
		&cylinder(DVec3::new(0.0, 0.0, z0 - 1.0), DVec3::Z, 8.0, 12.0, 64),
	)
}

#[test]
fn coplanar_tube_on_a_plane_tessellates_watertight() {
	let base = cuboid(DVec3::new(-20.0, -20.0, 0.0), DVec3::new(20.0, 20.0, 5.0));

	// Controls: box-on-box and a SOLID cylinder-on-box, both coplanar at z=5, are fine.
	let solid_cyl = cylinder(DVec3::new(0.0, 0.0, 5.0), DVec3::Z, 8.0, 10.0, 64);
	assert!(
		tessellate_default(&union(&base, &cuboid(DVec3::new(-8.0, -8.0, 5.0), DVec3::new(8.0, 8.0, 15.0)))).is_watertight()
			&& tessellate_default(&union(&base, &solid_cyl)).is_watertight(),
		"coplanar box-on-box and solid-cylinder-on-box must tessellate watertight (the issue is specifically the annular tube footprint)"
	);

	// Previously the bug: a TUBE abutting the plane exactly coplanar (bottom at
	// z=5) meshed non-manifold. Since the weld drops collapsed needle triangles
	// the seam meshes watertight — pin the fix.
	let abut = union(&base, &tube(5.0));
	assert!(
		validate(&abut).is_valid() && tessellate_default(&abut).is_watertight(),
		"a coplanar tube-on-plane must be a valid B-rep AND tessellate watertight (fixed 2026-07-02, weld needle hygiene): \
		 valid={} watertight={}",
		validate(&abut).is_valid(),
		tessellate_default(&abut).is_watertight()
	);

	// The workaround: overlap the tube 1 mm into the base (bottom at z=4) — watertight.
	let overlap = union(&base, &tube(4.0));
	assert!(
		validate(&overlap).is_valid() && tessellate_default(&overlap).is_watertight(),
		"overlapping the tube into the base (not coplanar-abutting) must tessellate watertight: valid={} watertight={}",
		validate(&overlap).is_valid(),
		tessellate_default(&overlap).is_watertight()
	);
}
