//! Mesh I/O round-trip fidelity for the formats users exchange.
//!
//! - STL, OBJ and 3MF all preserve triangle count and enclosed volume exactly.
//! - OBJ and 3MF are indexed (shared vertices), so a closed mesh round-trips as
//!   still watertight.
//! - STL stores per-triangle vertices with NO sharing, so `from_stl_bytes`
//!   returns an UNWELDED mesh that reads as non-watertight — until `weld()`
//!   rebuilds the shared topology. This pins the FDM/STL gotcha and its remedy
//!   (import STL, then weld before any watertight-dependent op).

use kernel_brep::math::DVec3;
use kernel_brep::{cuboid, cylinder, difference, tessellate_default};
use kernel_core::mesh::Mesh;

#[test]
fn mesh_io_round_trips_preserve_geometry_and_stl_needs_weld_for_watertight() {
	let part = difference(
		&cuboid(DVec3::new(-8.0, -8.0, -3.0), DVec3::new(8.0, 8.0, 3.0)),
		&cylinder(DVec3::new(0.0, 0.0, -4.0), DVec3::Z, 3.0, 8.0, 48),
	);
	let mesh = tessellate_default(&part);
	assert!(mesh.is_watertight(), "the tessellation must be watertight to begin with");
	let (tris, vol) = (mesh.triangle_count(), mesh.signed_volume());

	// per-process names: two concurrently-running suites must not race on the
	// same temp files (observed 2026-07-28 when two workspace test runs
	// overlapped — one deleted the fixed-name files while the other read them)
	let dir = std::env::temp_dir();
	let pid = std::process::id();
	let obj_p = dir.join(format!("lmcad_rt_test_{pid}.obj"));
	let mf_p = dir.join(format!("lmcad_rt_test_{pid}.3mf"));
	mesh.write_obj(&obj_p).expect("write obj");
	mesh.write_3mf(&mf_p).expect("write 3mf");
	let obj = Mesh::from_obj_bytes(&std::fs::read(&obj_p).unwrap()).expect("obj imports");
	let mf = Mesh::from_3mf_bytes(&std::fs::read(&mf_p).unwrap()).expect("3mf imports");
	let _ = (std::fs::remove_file(&obj_p), std::fs::remove_file(&mf_p));
	let stl = Mesh::from_stl_bytes(&mesh.to_stl_binary()).expect("stl imports");

	let vol_ok = |m: &Mesh| (m.signed_volume() - vol).abs() / vol.abs() < 1e-4 && m.triangle_count() == tris;

	// Every format preserves triangle count and enclosed volume.
	assert!(
		vol_ok(&stl) && vol_ok(&obj) && vol_ok(&mf),
		"all formats must preserve tri count {tris} and volume {vol:.3}: stl[{},{:.3}] obj[{},{:.3}] mf[{},{:.3}]",
		stl.triangle_count(), stl.signed_volume(), obj.triangle_count(), obj.signed_volume(), mf.triangle_count(), mf.signed_volume()
	);

	// Indexed formats keep the closed topology; raw STL does not until welded.
	assert!(
		obj.is_watertight() && mf.is_watertight() && !stl.is_watertight(),
		"OBJ/3MF must round-trip watertight and raw STL must not (unwelded): obj={} mf={} stl={}",
		obj.is_watertight(), mf.is_watertight(), stl.is_watertight()
	);
	let mut welded = stl.clone();
	welded.weld(1e-4);
	assert!(
		welded.is_watertight() && (welded.signed_volume() - vol).abs() / vol.abs() < 1e-4,
		"weld() must restore watertightness (and preserve volume) after STL import: wt={} vol={:.3}",
		welded.is_watertight(), welded.signed_volume()
	);
}
