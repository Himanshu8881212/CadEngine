// Copyright (c) LMCAD. Licensed under the MIT License.

//! `import_mesh {heal: "remesh"}` (l12 F2): a soup the topological repair cannot
//! fix — two overlapping shells, wound INWARD as a vendor export was — re-meshes
//! watertight through its winding-number field, with the receipt saying so.

use std::path::PathBuf;

use kernel_api::run_program;
use kernel_brep::build::cuboid;
use kernel_brep::math::DVec3;
use kernel_core::Mesh;

fn out_dir(tag: &str) -> PathBuf {
	let d = std::env::temp_dir().join(format!("lmcad_remesh_{tag}_{}", std::process::id()));
	let _ = std::fs::remove_dir_all(&d);
	std::fs::create_dir_all(&d).unwrap();
	d
}

#[test]
fn an_inward_wound_overlapping_soup_remeshes_watertight() {
	let d = out_dir("soup");
	// Two 20-cubes overlapping by 10: as separate shells (a soup with 24
	// self-intersections), every triangle wound inward.
	let mut soup = Mesh::new();
	for (lo, hi) in [(DVec3::ZERO, DVec3::splat(20.0)), (DVec3::splat(10.0), DVec3::splat(30.0))] {
		let m = kernel_brep::tessellate_default(&cuboid(lo, hi));
		let base = soup.positions.len() as u32;
		soup.positions.extend(m.positions.iter().copied());
		soup.normals.extend(m.normals.iter().copied());
		soup.indices.extend(m.indices.iter().map(|i| i + base));
	}
	soup.reverse_winding();
	assert!(soup.signed_volume() < 0.0, "the fixture is wound inward");
	soup.write_stl_binary(&d.join("soup.stl")).unwrap();

	let program = serde_json::json!({"ops": [
		{"id": "plain", "op": "import_mesh", "file": "soup.stl"},
		{"id": "m", "op": "import_mesh", "file": "soup.stl", "heal": "remesh", "voxel": 0.5, "out": "healed.stl"},
		{"id": "v", "op": "volume", "in": "m"}
	]});
	let r = run_program(&program.to_string(), &d);
	assert!(r.ok, "{r:#?}");
	let plain = r.ops.iter().find(|o| o.id == "plain").unwrap().measures.clone().unwrap();
	// Two closed shells that cross each other: each shell is watertight on its
	// own, the soup crosses itself 18 times and is wound inward (negative volume).
	assert!(plain["self_intersections"].as_u64().unwrap_or(0) > 0, "the soup crosses itself: {plain}");
	assert!(plain["volume"].as_f64().unwrap_or(0.0) < 0.0, "the soup is wound inward: {plain}");
	let m = r.ops.iter().find(|o| o.id == "m").unwrap().measures.clone().unwrap();
	assert_eq!(m["healed"], serde_json::json!("remesh"), "{m}");
	assert_eq!(m["watertight"], serde_json::json!(true), "{m}");
	assert_eq!(m["remesh_voxel"], serde_json::json!(0.5));
	assert!(m["remesh_mesher"].is_string(), "{m}");
	// Union of the two cubes: 8000 + 8000 − 1000 = 15000 mm³, to the voxel's resolution.
	let vol = r.ops.iter().find(|o| o.id == "v").unwrap().measures.clone().unwrap()["volume"].as_f64().unwrap();
	assert!((vol - 15000.0).abs() < 0.03 * 15000.0, "remeshed volume {vol} vs 15000");
	let _ = std::fs::remove_dir_all(&d);
}
