// Copyright (c) LMCAD. Licensed under the MIT License.

//! Mesh file I/O — STL / OBJ / 3MF / PLY / glTF readers and writers.
//! Split out of the monolithic mesh.rs (2026-07-28 cleanup wave); behavior
//! is byte-identical — the impl block moved verbatim.

use std::io::{self, Read, Write};
use std::path::Path;

use super::*;

/// Wrap a glTF JSON chunk and binary buffer into a `.glb` container (12-byte
/// header + padded JSON chunk + BIN chunk).
pub(crate) fn glb_container(json: &[u8], bin: &[u8]) -> Vec<u8> {
	let mut json_bytes = json.to_vec();
	while !json_bytes.len().is_multiple_of(4) {
		json_bytes.push(b' '); // glTF requires the JSON chunk padded with spaces
	}
	let total = 12 + 8 + json_bytes.len() + 8 + bin.len();
	let mut out = Vec::with_capacity(total);
	out.extend_from_slice(&0x4654_6C67u32.to_le_bytes()); // magic "glTF"
	out.extend_from_slice(&2u32.to_le_bytes()); // version 2
	out.extend_from_slice(&(total as u32).to_le_bytes());
	out.extend_from_slice(&(json_bytes.len() as u32).to_le_bytes());
	out.extend_from_slice(&0x4E4F_534Au32.to_le_bytes()); // chunk type "JSON"
	out.extend_from_slice(&json_bytes);
	out.extend_from_slice(&(bin.len() as u32).to_le_bytes());
	out.extend_from_slice(&0x004E_4942u32.to_le_bytes()); // chunk type "BIN\0"
	out.extend_from_slice(bin);
	out
}

impl Mesh {
	pub fn read_stl(path: impl AsRef<Path>) -> io::Result<Mesh> {
		Self::from_stl_bytes(&std::fs::read(path)?)
	}

	/// Read a Wavefront `.obj` file into a triangle mesh — the reading half of OBJ
	/// interchange (companion to [`Mesh::write_obj`]).
	pub fn read_obj(path: impl AsRef<Path>) -> io::Result<Mesh> {
		Self::from_obj_bytes(&std::fs::read(path)?)
	}

	/// Read a `.3mf` package into a triangle mesh — the reading half of 3MF interchange
	/// (companion to [`Mesh::write_3mf`]). Lets authored / printed 3MF parts enter the kernel
	/// via the mesh→SDF bridge.
	pub fn read_3mf(path: impl AsRef<Path>) -> io::Result<Mesh> {
		Self::from_3mf_bytes(&std::fs::read(path)?)
	}

	/// Read an ASCII `.ply` (Stanford Polygon) file into a triangle mesh — the format scanners
	/// and tools like MeshLab export. Lets scanned / research meshes enter the kernel.
	pub fn read_ply(path: impl AsRef<Path>) -> io::Result<Mesh> {
		Self::from_ply_bytes(&std::fs::read(path)?)
	}

	/// Parse a Wavefront OBJ mesh from bytes: `v x y z` vertices and `f` faces. Every face
	/// index form is accepted (`a`, `a/b`, `a/b/c`, `a//c`); indices are 1-based, and a negative
	/// index counts back from the current vertex list (per the OBJ spec). Polygon faces are
	/// fan-triangulated; texture/normal/other records are ignored, recovering a position-only
	/// triangle mesh (welded vertices are preserved as written — call [`Mesh::weld`] if the source
	/// duplicated them). Lets external/authored OBJ meshes enter the kernel.
	pub fn from_obj_bytes(bytes: &[u8]) -> io::Result<Mesh> {
		let text = std::str::from_utf8(bytes).map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;
		let mut mesh = Mesh::new();
		for line in text.lines() {
			let mut tok = line.split_whitespace();
			match tok.next() {
				Some("v") => {
					let c: Vec<f32> = tok.filter_map(|t| t.parse::<f32>().ok()).collect();
					if c.len() >= 3 {
						mesh.positions.push(Vec3::new(c[0], c[1], c[2]));
					}
				}
				Some("f") => {
					let nv = mesh.positions.len() as i64;
					// Each face vertex is `v[/vt][/vn]`; take the leading vertex index, resolve
					// 1-based / negative, and keep only in-range references.
					let idx: Vec<u32> = tok
						.filter_map(|t| t.split('/').next().and_then(|s| s.parse::<i64>().ok()))
						.filter_map(|i| {
							let r = if i < 0 { nv + i } else { i - 1 };
							(0..nv).contains(&r).then_some(r as u32)
						})
						.collect();
					for k in 1..idx.len().saturating_sub(1) {
						mesh.push_triangle(idx[0], idx[k], idx[k + 1]);
					}
				}
				_ => {}
			}
		}
		Ok(mesh)
	}

	/// Parse a 3MF package from bytes (an OPC zip): the 3D model part — located by its
	/// `.model` extension — is read and its `<vertex>` / `<triangle>` elements parsed into a
	/// position-only triangle mesh. Triangles referencing out-of-range vertices are dropped.
	/// Reads the core mesh geometry; materials, colours and component transforms are ignored.
	/// Welded vertices are preserved as written (call [`Mesh::weld`] if a source duplicated
	/// them). Companion to [`Mesh::write_3mf`].
	pub fn from_3mf_bytes(bytes: &[u8]) -> io::Result<Mesh> {
		let mut zip = zip::ZipArchive::new(io::Cursor::new(bytes)).map_err(io::Error::other)?;
		// The 3D model lives in an OPC part with a `.model` extension (conventionally
		// `3D/3dmodel.model`); find it by extension rather than parsing the relationships.
		let model_part = zip
			.file_names()
			.find(|n| Path::new(n).extension().is_some_and(|e| e.eq_ignore_ascii_case("model")))
			.map(str::to_string)
			.ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "3MF: no .model part in package"))?;
		let mut xml = String::new();
		zip.by_name(&model_part).map_err(io::Error::other)?.read_to_string(&mut xml)?;

		let mut mesh = Mesh::new();
		for v in xml_elements(&xml, "vertex") {
			if let (Some(x), Some(y), Some(z)) = (xml_attr_f32(v, "x"), xml_attr_f32(v, "y"), xml_attr_f32(v, "z")) {
				mesh.positions.push(Vec3::new(x, y, z));
			}
		}
		let nv = mesh.positions.len() as u32;
		for t in xml_elements(&xml, "triangle") {
			if let (Some(a), Some(b), Some(c)) = (xml_attr_u32(t, "v1"), xml_attr_u32(t, "v2"), xml_attr_u32(t, "v3")) {
				if a < nv && b < nv && c < nv {
					mesh.push_triangle(a, b, c);
				}
			}
		}
		Ok(mesh)
	}

	/// Parse an **ASCII PLY** mesh from bytes. The header's `element vertex N` / `element face M`
	/// counts drive the parse; each vertex line contributes its first three floats as `x y z`
	/// (extra per-vertex properties — normals, colours — are ignored), and each face line
	/// `k i₀ … i_{k-1}` is fan-triangulated. Out-of-range face indices are dropped. Binary PLY
	/// is not handled (returns whatever the ASCII pass recovers). Companion to authored meshes.
	pub fn from_ply_bytes(bytes: &[u8]) -> io::Result<Mesh> {
		let text = std::str::from_utf8(bytes).map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;
		let mut lines = text.lines();
		let (mut n_verts, mut n_faces) = (0usize, 0usize);
		for line in lines.by_ref() {
			let l = line.trim();
			if l == "end_header" {
				break;
			}
			let mut t = l.split_whitespace();
			if t.next() == Some("element") {
				match (t.next(), t.next().and_then(|s| s.parse::<usize>().ok())) {
					(Some("vertex"), Some(n)) => n_verts = n,
					(Some("face"), Some(n)) => n_faces = n,
					_ => {}
				}
			}
		}
		let mut mesh = Mesh::new();
		for _ in 0..n_verts {
			let line = lines.next().ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "PLY: truncated vertices"))?;
			let c: Vec<f32> = line.split_whitespace().take(3).filter_map(|s| s.parse().ok()).collect();
			if c.len() == 3 {
				mesh.positions.push(Vec3::new(c[0], c[1], c[2]));
			}
		}
		let nv = mesh.positions.len() as i64;
		for _ in 0..n_faces {
			let line = lines.next().ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "PLY: truncated faces"))?;
			let nums: Vec<i64> = line.split_whitespace().filter_map(|s| s.parse().ok()).collect();
			let k = nums.first().copied().unwrap_or(0) as usize;
			if k < 3 || nums.len() < 1 + k {
				continue;
			}
			let idx = &nums[1..1 + k];
			for w in 1..k - 1 {
				let tri = [idx[0], idx[w], idx[w + 1]];
				if tri.iter().all(|&i| (0..nv).contains(&i)) {
					mesh.push_triangle(tri[0] as u32, tri[1] as u32, tri[2] as u32);
				}
			}
		}
		Ok(mesh)
	}

	/// Parse STL bytes (binary or ASCII). Binary is detected by the exact
	/// `84 + 50·count` length; otherwise the input is parsed as ASCII.
	pub fn from_stl_bytes(bytes: &[u8]) -> io::Result<Mesh> {
		if bytes.len() >= 84 {
			let count = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize;
			if bytes.len() == 84 + 50 * count {
				// Guard the rare case where an ASCII file's length coincides with the
				// binary formula: a real binary record region is raw floats and never
				// spells the `facet` keyword, so its presence means the file is ASCII.
				let looks_ascii = bytes[..bytes.len().min(512)].windows(5).any(|w| w.eq_ignore_ascii_case(b"facet"));
				if !looks_ascii {
					return Self::parse_binary_stl(bytes, count);
				}
			}
		}
		Self::parse_ascii_stl(bytes)
	}

	fn parse_binary_stl(bytes: &[u8], count: usize) -> io::Result<Mesh> {
		let mut mesh = Mesh::with_capacity_for(count);
		let f = |o: usize| f32::from_le_bytes([bytes[o], bytes[o + 1], bytes[o + 2], bytes[o + 3]]);
		let mut off = 84;
		for _ in 0..count {
			let n = Vec3::new(f(off), f(off + 4), f(off + 8)).normalize_or_zero();
			let base = mesh.positions.len() as u32;
			for k in 0..3 {
				let b = off + 12 + k * 12;
				mesh.positions.push(Vec3::new(f(b), f(b + 4), f(b + 8)));
				mesh.normals.push(n);
			}
			mesh.push_triangle(base, base + 1, base + 2);
			off += 50;
		}
		Ok(mesh)
	}

	fn parse_ascii_stl(bytes: &[u8]) -> io::Result<Mesh> {
		let text = std::str::from_utf8(bytes)
			.map_err(|_| io::Error::new(io::ErrorKind::InvalidData, "STL is neither valid binary nor ASCII"))?;
		let mut mesh = Mesh::new();
		let mut verts: Vec<Vec3> = Vec::with_capacity(3);
		let mut normal = Vec3::ZERO;
		for line in text.lines() {
			let mut it = line.split_whitespace();
			match it.next() {
				Some("facet") => {
					it.next(); // "normal"
					let c: Vec<f32> = it.filter_map(|t| t.parse().ok()).collect();
					normal = if c.len() == 3 { Vec3::new(c[0], c[1], c[2]).normalize_or_zero() } else { Vec3::ZERO };
				}
				Some("vertex") => {
					let c: Vec<f32> = it.filter_map(|t| t.parse().ok()).collect();
					if c.len() == 3 {
						verts.push(Vec3::new(c[0], c[1], c[2]));
					}
				}
				Some("endfacet") => {
					if verts.len() == 3 {
						let base = mesh.positions.len() as u32;
						for &v in &verts {
							mesh.positions.push(v);
							mesh.normals.push(normal);
						}
						mesh.push_triangle(base, base + 1, base + 2);
					}
					verts.clear();
				}
				_ => {}
			}
		}
		Ok(mesh)
	}

	fn with_capacity_for(triangles: usize) -> Mesh {
		Mesh {
			positions: Vec::with_capacity(triangles * 3),
			indices: Vec::with_capacity(triangles * 3),
			normals: Vec::with_capacity(triangles * 3),
		}
	}

	// --- Exporters -----------------------------------------------------------

	/// Binary STL bytes.
	pub fn to_stl_binary(&self) -> Vec<u8> {
		let tri = self.triangle_count();
		let mut buf = Vec::with_capacity(84 + tri * 50);
		buf.extend_from_slice(&[0u8; 80]); // 80-byte header
		buf.extend_from_slice(&(tri as u32).to_le_bytes());
		let push = |v: Vec3, buf: &mut Vec<u8>| {
			buf.extend_from_slice(&v.x.to_le_bytes());
			buf.extend_from_slice(&v.y.to_le_bytes());
			buf.extend_from_slice(&v.z.to_le_bytes());
		};
		for t in self.indices.chunks_exact(3) {
			let a = self.positions[t[0] as usize];
			let b = self.positions[t[1] as usize];
			let c = self.positions[t[2] as usize];
			let n = (b - a).cross(c - a).normalize_or_zero();
			push(n, &mut buf);
			push(a, &mut buf);
			push(b, &mut buf);
			push(c, &mut buf);
			buf.extend_from_slice(&[0u8; 2]); // attribute byte count
		}
		buf
	}

	/// Write a binary STL file.
	pub fn write_stl_binary(&self, path: impl AsRef<Path>) -> io::Result<()> {
		std::fs::write(path, self.to_stl_binary())
	}

	/// Write a Wavefront OBJ file (handy for quick inspection / diffing).
	pub fn write_obj(&self, path: impl AsRef<Path>) -> io::Result<()> {
		let mut w = io::BufWriter::new(std::fs::File::create(path)?);
		let f = finite_or_zero; // a non-finite coordinate would write an invalid token
		for p in &self.positions {
			writeln!(w, "v {} {} {}", f(p.x), f(p.y), f(p.z))?;
		}
		for n in &self.normals {
			writeln!(w, "vn {} {} {}", f(n.x), f(n.y), f(n.z))?;
		}
		// Only reference `vn i` when there is exactly one normal per vertex, else a
		// `f a//a` would index out of (or mismatch) the normal array.
		let has_normals = self.normals.len() == self.positions.len();
		for t in self.indices.chunks_exact(3) {
			let (a, b, c) = (t[0] + 1, t[1] + 1, t[2] + 1);
			if has_normals {
				writeln!(w, "f {a}//{a} {b}//{b} {c}//{c}")?;
			} else {
				writeln!(w, "f {a} {b} {c}")?;
			}
		}
		w.flush()
	}

	/// Serialize to a self-contained binary glTF (`.glb`) with POSITION + NORMAL
	/// attributes and `UNSIGNED_INT` indices — the standard interchange for web /
	/// Three.js / Blender viewers. Normals are computed if not already present.
	pub fn to_glb(&self) -> Vec<u8> {
		let n_verts = self.positions.len();
		let n_idx = self.indices.len();
		// glTF 2.0 forbids accessors with count 0; emit a minimal valid asset (no
		// meshes/accessors) for an empty mesh instead of count:0 accessors.
		if n_verts == 0 || n_idx == 0 {
			// `asset` is the only required top-level property; emit just that (empty
			// `scenes`/`nodes` arrays would violate the glTF 2.0 minItems:1 schema).
			return glb_container(b"{\"asset\":{\"version\":\"2.0\",\"generator\":\"LMCAD hybrid kernel\"}}", &[]);
		}
		let normals = if self.normals.len() == n_verts && n_verts > 0 {
			self.normals.clone()
		} else {
			self.area_weighted_normals()
		};

		// Binary buffer: [positions][normals][indices] (all naturally 4-aligned).
		let mut bin: Vec<u8> = Vec::with_capacity(n_verts * 24 + n_idx * 4);
		let push3 = |bin: &mut Vec<u8>, v: Vec3| {
			// Sanitize so the binary POSITION/NORMAL data stays within the (also
			// sanitized) accessor min/max and never carries NaN/inf.
			bin.extend_from_slice(&finite_or_zero(v.x).to_le_bytes());
			bin.extend_from_slice(&finite_or_zero(v.y).to_le_bytes());
			bin.extend_from_slice(&finite_or_zero(v.z).to_le_bytes());
		};
		for &p in &self.positions {
			push3(&mut bin, p);
		}
		let (pos_off, pos_len) = (0usize, bin.len());
		let nrm_off = bin.len();
		for &n in &normals {
			push3(&mut bin, n);
		}
		let nrm_len = bin.len() - nrm_off;
		let idx_off = bin.len();
		for &i in &self.indices {
			bin.extend_from_slice(&i.to_le_bytes());
		}
		let idx_len = bin.len() - idx_off;
		while !bin.len().is_multiple_of(4) {
			bin.push(0);
		}

		let bb = if n_verts > 0 { self.aabb() } else { Aabb::new(Vec3::ZERO, Vec3::ZERO) };
		// A non-finite vertex would make the AABB non-finite and emit bare `NaN`/`inf`
		// tokens into the JSON — invalid JSON that breaks every glTF parser. Sanitize
		// the accessor min/max to finite values.
		let san = |v: f32| if v.is_finite() { v } else { 0.0 };
		let (mn, mx) = (Vec3::new(san(bb.min.x), san(bb.min.y), san(bb.min.z)), Vec3::new(san(bb.max.x), san(bb.max.y), san(bb.max.z)));
		let json = format!(
			"{{\"asset\":{{\"version\":\"2.0\",\"generator\":\"LMCAD hybrid kernel\"}},\
			\"scene\":0,\"scenes\":[{{\"nodes\":[0]}}],\"nodes\":[{{\"mesh\":0}}],\
			\"meshes\":[{{\"primitives\":[{{\"attributes\":{{\"POSITION\":0,\"NORMAL\":1}},\"indices\":2,\"mode\":4}}]}}],\
			\"accessors\":[\
			{{\"bufferView\":0,\"componentType\":5126,\"count\":{nv},\"type\":\"VEC3\",\"min\":[{mnx},{mny},{mnz}],\"max\":[{mxx},{mxy},{mxz}]}},\
			{{\"bufferView\":1,\"componentType\":5126,\"count\":{nv},\"type\":\"VEC3\"}},\
			{{\"bufferView\":2,\"componentType\":5125,\"count\":{ni},\"type\":\"SCALAR\"}}],\
			\"bufferViews\":[\
			{{\"buffer\":0,\"byteOffset\":{po},\"byteLength\":{pl},\"target\":34962}},\
			{{\"buffer\":0,\"byteOffset\":{no},\"byteLength\":{nl},\"target\":34962}},\
			{{\"buffer\":0,\"byteOffset\":{io},\"byteLength\":{il},\"target\":34963}}],\
			\"buffers\":[{{\"byteLength\":{bl}}}]}}",
			nv = n_verts, ni = n_idx, po = pos_off, pl = pos_len, no = nrm_off, nl = nrm_len,
			io = idx_off, il = idx_len, bl = bin.len(),
			mnx = mn.x, mny = mn.y, mnz = mn.z, mxx = mx.x, mxy = mx.y, mxz = mx.z,
		);
		glb_container(json.as_bytes(), &bin)
	}

	/// Write a binary glTF (`.glb`) file.
	pub fn write_glb(&self, path: impl AsRef<Path>) -> io::Result<()> {
		std::fs::write(path, self.to_glb())
	}

	/// Write a 3MF file (OPC zip with a single mesh object, millimeter units).
	///
	/// 3MF is the preferred export: it preserves manifoldness and units, unlike
	/// STL which is a convenience only.
	pub fn write_3mf(&self, path: impl AsRef<Path>) -> io::Result<()> {
		use zip::write::SimpleFileOptions;

		let file = std::fs::File::create(path)?;
		let mut zw = zip::ZipWriter::new(file);
		let opts = SimpleFileOptions::default().compression_method(zip::CompressionMethod::Deflated);
		let zip_err = |e: zip::result::ZipError| io::Error::other(e);

		const CONTENT_TYPES: &str = concat!(
			"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n",
			"<Types xmlns=\"http://schemas.openxmlformats.org/package/2006/content-types\">",
			"<Default Extension=\"rels\" ContentType=\"application/vnd.openxmlformats-package.relationships+xml\"/>",
			"<Default Extension=\"model\" ContentType=\"application/vnd.ms-package.3dmanufacturing-3dmodel+xml\"/>",
			"</Types>"
		);
		const RELS: &str = concat!(
			"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n",
			"<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">",
			"<Relationship Target=\"/3D/3dmodel.model\" Id=\"rel0\" ",
			"Type=\"http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel\"/>",
			"</Relationships>"
		);

		let mut model = String::with_capacity(256 + self.positions.len() * 48 + self.indices.len() * 16);
		model.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
		model.push_str(
			"<model unit=\"millimeter\" xml:lang=\"en-US\" \
			 xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\">",
		);
		model.push_str("<resources><object id=\"1\" type=\"model\"><mesh><vertices>");
		for p in &self.positions {
			let (x, y, z) = (finite_or_zero(p.x), finite_or_zero(p.y), finite_or_zero(p.z));
			model.push_str(&format!("<vertex x=\"{x}\" y=\"{y}\" z=\"{z}\"/>"));
		}
		model.push_str("</vertices><triangles>");
		for t in self.indices.chunks_exact(3) {
			model.push_str(&format!("<triangle v1=\"{}\" v2=\"{}\" v3=\"{}\"/>", t[0], t[1], t[2]));
		}
		model.push_str("</triangles></mesh></object></resources>");
		model.push_str("<build><item objectid=\"1\"/></build></model>");

		zw.start_file("[Content_Types].xml", opts).map_err(zip_err)?;
		zw.write_all(CONTENT_TYPES.as_bytes())?;
		zw.add_directory("_rels/", opts).map_err(zip_err)?;
		zw.start_file("_rels/.rels", opts).map_err(zip_err)?;
		zw.write_all(RELS.as_bytes())?;
		zw.add_directory("3D/", opts).map_err(zip_err)?;
		zw.start_file("3D/3dmodel.model", opts).map_err(zip_err)?;
		zw.write_all(model.as_bytes())?;
		zw.finish().map_err(zip_err)?;
		Ok(())
	}
}


/// Each `<{tag} …/>`-or-`<{tag} …>` element of `xml`, as the substring from the tag to its
/// closing `>`. A trailing boundary check (the char after the name must be whitespace, `/`
/// or `>`) keeps `<vertex` from matching inside `<vertices>`. Tolerant enough for the
/// machine-generated 3MF mesh section without a full XML parser.
pub(crate) fn xml_elements<'a>(xml: &'a str, tag: &str) -> Vec<&'a str> {
	let open = format!("<{tag}");
	let mut out = Vec::new();
	let mut from = 0;
	while let Some(rel) = xml[from..].find(&open) {
		let start = from + rel;
		let after = start + open.len();
		let ends = xml.as_bytes().get(after).is_none_or(|&c| matches!(c, b' ' | b'\t' | b'\r' | b'\n' | b'/' | b'>'));
		let end = xml[start..].find('>').map_or(xml.len(), |e| start + e + 1);
		if ends {
			out.push(&xml[start..end]);
		}
		from = end.max(start + 1);
	}
	out
}

/// Value of the whitespace-delimited `name="value"` attribute within an element substring,
/// or `None` if absent. The preceding-whitespace guard stops a name from matching as the
/// suffix of a longer attribute.
pub(crate) fn xml_attr<'a>(elem: &'a str, name: &str) -> Option<&'a str> {
	let needle = format!("{name}=\"");
	let mut from = 0;
	while let Some(rel) = elem[from..].find(&needle) {
		let at = from + rel;
		let boundary = at == 0 || elem.as_bytes()[at - 1].is_ascii_whitespace();
		let vstart = at + needle.len();
		let vend = vstart + elem[vstart..].find('"')?;
		if boundary {
			return Some(&elem[vstart..vend]);
		}
		from = vend + 1;
	}
	None
}

pub(crate) fn xml_attr_f32(elem: &str, name: &str) -> Option<f32> {
	xml_attr(elem, name).and_then(|s| s.trim().parse().ok())
}

pub(crate) fn xml_attr_u32(elem: &str, name: &str) -> Option<u32> {
	xml_attr(elem, name).and_then(|s| s.trim().parse().ok())
}



