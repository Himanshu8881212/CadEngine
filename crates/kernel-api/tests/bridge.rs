//! ACE-bridge round trip: solid → solid_fraction.npy → gated watertight mesh.
use kernel_api::run_program;

#[test]
fn density_grid_round_trips_a_bracket_volume() {
	let dir = std::env::temp_dir().join("lmcad_bridge_test");
	let _ = std::fs::create_dir_all(&dir);
	// an L-bracket-ish solid: plate + upright, minus a bore — B-rep booleans
	let program = r##"{"ops": [
		{"id": "plate", "op": "box", "min": [0, 0, 0], "max": [40, 20, 6]},
		{"id": "wall",  "op": "box", "min": [0, 0, 0], "max": [6, 20, 30]},
		{"id": "l",     "op": "union", "a": "plate", "b": "wall"},
		{"id": "bore",  "op": "cylinder", "base": [30, 10, -1], "axis": [0, 0, 1], "radius": 4.0, "height": 8.0},
		{"id": "part",  "op": "difference", "a": "l", "b": "bore"},
		{"id": "vol",   "op": "volume", "in": "part"},
		{"id": "grid",  "op": "sample_density_grid", "in": "part",
		 "origin": [-1, -1, -1], "voxel": 0.5, "shape": [84, 44, 64], "supersample": 2,
		 "file": "solid_fraction.npy"},
		{"id": "back",  "op": "mesh_density_grid", "npy": "solid_fraction.npy",
		 "origin": [-1, -1, -1], "voxel": 0.5, "file": "roundtrip.stl"}
	]}"##;
	let report = run_program(program, &dir);
	let txt = serde_json::to_string_pretty(&report).unwrap();
	assert!(txt.contains("\"ok\": true") || report_ok(&txt), "bridge program must succeed end-to-end:\n{txt}");
	// exact volume vs the round-tripped voxel volume: one-voxel-skin agreement
	let v_exact = extract_num(&txt, "vol");
	let v_back = extract_num_key(&txt, "volume_mm3");
	let rel = (v_exact - v_back).abs() / v_exact;
	assert!(
		rel < 0.05,
		"round-trip volume must agree within the voxel skin (5% at h=0.5): exact {v_exact:.1} vs back {v_back:.1} ({:.1}%)\n{txt}",
		rel * 100.0
	);
	// the npy must be loadable by the ACE contract check: magic + header basics
	let bytes = std::fs::read(dir.join("solid_fraction.npy")).unwrap();
	assert!(
		bytes.starts_with(b"\x93NUMPY") && {
			let h = String::from_utf8_lossy(&bytes[10..128]);
			h.contains("'<f4'") && h.contains("(84, 44, 64)") && h.contains("False")
		},
		"npy header must match the ACE contract (float32, C-order, (84,44,64))"
	);
}

fn report_ok(txt: &str) -> bool {
	!txt.contains("\"error\"")
}
fn extract_num(txt: &str, op_id: &str) -> f64 {
	// the volume op reports {"id":"vol", ... "volume": N}
	let at = txt.find(&format!("\"{op_id}\"")).expect("op in report");
	let sub = &txt[at..];
	let key = sub.find("\"volume\"").expect("volume in measures");
	parse_after(&sub[key..])
}
fn extract_num_key(txt: &str, key: &str) -> f64 {
	let at = txt.find(&format!("\"{key}\"")).expect("key in report");
	parse_after(&txt[at..])
}
fn parse_after(s: &str) -> f64 {
	let colon = s.find(':').unwrap();
	s[colon + 1..]
		.trim_start()
		.chars()
		.take_while(|c| c.is_ascii_digit() || *c == '.' || *c == '-' || *c == 'e' || *c == 'E' || *c == '+')
		.collect::<String>()
		.parse()
		.unwrap()
}
