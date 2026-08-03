//! GLB (binary glTF 2.0) export must be a spec-valid container whose accessor
//! counts and layout match the mesh — the web / 3D-viewer interchange format.
//! The existing coverage only checks the 4 magic bytes; this validates the whole
//! container and the glTF JSON it carries.

use kernel_brep::math::DVec3;
use kernel_brep::{cuboid, tessellate_default};

fn u32le(b: &[u8], o: usize) -> u32 {
	u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]])
}

#[test]
fn glb_export_is_a_valid_gltf2_container_matching_the_mesh() {
	let m = tessellate_default(&cuboid(DVec3::ZERO, DVec3::new(10.0, 20.0, 5.0)));
	let (nv, ni) = (m.positions.len(), m.indices.len());
	assert_eq!((nv, ni), (8, 36), "a cuboid tessellates to 8 verts / 36 indices");
	let glb = m.to_glb();

	// 12-byte header: magic, version 2, declared length == actual.
	assert_eq!(&glb[0..4], b"glTF", "GLB magic");
	assert_eq!(u32le(&glb, 4), 2, "glTF container version 2");
	assert_eq!(u32le(&glb, 8) as usize, glb.len(), "declared length must equal the byte length");

	// JSON chunk then BIN chunk; the two chunks plus headers must tile the file.
	let json_len = u32le(&glb, 12) as usize;
	assert_eq!(&glb[16..20], b"JSON", "chunk 0 is the JSON chunk");
	let bin_off = 20 + json_len;
	let bin_len = u32le(&glb, bin_off) as usize;
	assert_eq!(&glb[bin_off + 4..bin_off + 8], b"BIN\0", "chunk 1 is the BIN chunk");
	assert_eq!(12 + 8 + json_len + 8 + bin_len, glb.len(), "header + JSON chunk + BIN chunk must tile the file exactly");

	// The glTF JSON must declare TRIANGLES, version 2.0, the POSITION attribute,
	// the accessor counts that match the mesh, and the POSITION min/max bounds.
	let json = std::str::from_utf8(&glb[20..20 + json_len]).expect("JSON chunk is UTF-8");
	assert!(
		json.contains("\"version\":\"2.0\"")
			&& json.contains("\"mode\":4")
			&& json.contains("\"POSITION\":")
			&& json.contains(&format!("\"count\":{nv}"))
			&& json.contains(&format!("\"count\":{ni}"))
			&& json.contains("\"min\":")
			&& json.contains("\"max\":"),
		"glTF JSON must be a TRIANGLES mesh with POSITION/index accessor counts {nv}/{ni} and POSITION bounds: {json}"
	);

	// BIN buffer = positions (nv*3*f32) + normals (nv*3*f32) + indices (ni*u32).
	assert_eq!(bin_len, nv * 12 + nv * 12 + ni * 4, "BIN buffer length must equal positions+normals+indices bytes");
}
