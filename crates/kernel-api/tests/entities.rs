//! M4 loop, first slice (CAD Code): `list_faces` / `list_edges` enumerate a solid's entities as
//! references the agent can act on — analytic surface descriptors + witness points read from the
//! existing kernel topology, so an agent can SELECT geometry instead of guessing coordinates.

use kernel_api::{run_program, Report};
use serde_json::json;
use std::path::Path;

fn measures<'a>(r: &'a Report, id: &str) -> Option<&'a serde_json::Value> {
	r.ops.iter().find(|o| o.id == id).and_then(|o| o.measures.as_ref())
}
fn run(dir: &Path, ops: serde_json::Value) -> Report {
	run_program(&serde_json::to_string(&json!({ "ops": ops })).unwrap(), dir)
}

#[test]
fn list_faces_and_edges_enumerate_entities_with_types_and_witnesses() {
	let dir = std::env::temp_dir().join(format!("cadcode_entities_{}", std::process::id()));
	std::fs::create_dir_all(&dir).unwrap();

	// A box: exactly 6 planar faces and 12 edges, each face carrying a normal descriptor + witness.
	let r = run(
		&dir,
		json!([
			{"id":"b","op":"box","min":[0,0,0],"max":[10,10,10]},
			{"id":"lf","op":"list_faces","in":"b"},
			{"id":"le","op":"list_edges","in":"b"}
		]),
	);
	let lf = measures(&r, "lf").unwrap_or_else(|| panic!("list_faces measures — {r:#?}"));
	assert_eq!(lf["count"].as_u64(), Some(6), "a box has 6 faces — {r:#?}");
	let faces = lf["faces"].as_array().unwrap();
	assert!(faces.iter().all(|f| f["type"] == "plane"), "every box face is a plane — {r:#?}");
	assert!(
		faces[0]["descriptor"]["normal"].is_array() && faces[0]["witness"].is_array() && faces[0]["area"].as_f64() == Some(100.0),
		"a face carries a normal descriptor, a witness point, and (planar) exact area — {r:#?}"
	);
	let le = measures(&r, "le").unwrap_or_else(|| panic!("list_edges measures — {r:#?}"));
	assert_eq!(le["count"].as_u64(), Some(12), "a box has 12 edges — {r:#?}");
	assert!(
		le["edges"][0]["midpoint"].is_array() && le["edges"][0]["length"].as_f64() == Some(10.0),
		"an edge carries a midpoint witness and its chord length — {r:#?}"
	);

	// A primitive cylinder: its wall is enumerated as an analytic cylinder face with a radius.
	let r = run(
		&dir,
		json!([
			{"id":"c","op":"cylinder","base":[0,0,0],"axis":[0,0,1],"radius":4,"height":10},
			{"id":"lf","op":"list_faces","in":"c"}
		]),
	);
	let faces = measures(&r, "lf").and_then(|m| m["faces"].as_array()).unwrap_or_else(|| panic!("cylinder faces — {r:#?}"));
	let wall = faces.iter().find(|f| f["type"] == "cylinder");
	assert!(
		wall.map(|w| (w["descriptor"]["radius"].as_f64().unwrap_or(0.0) - 4.0).abs() < 1e-9).unwrap_or(false),
		"the cylinder wall is enumerated as an analytic cylinder face of radius 4 — {r:#?}"
	);

	let _ = std::fs::remove_dir_all(&dir);
}
