// Copyright (c) LMCAD. Licensed under the MIT License.

//! `kernel-wasm` — the WebAssembly surface that exposes the hybrid geometry
//! kernel to a browser (Three.js) viewer.
//!
//! The crate has two layers:
//! - A plain, non-`wasm` [`build_demo`] that assembles a smooth-blended CSG
//!   model and meshes it with `kernel_implicit::manifold_dual_contour` (watertight
//!   even across the hard boolean's concave crease). This is ordinary Rust and is
//!   exercised by the host-side unit tests.
//! - A thin `#[wasm_bindgen]` layer ([`MeshBuffers`] + [`demo`]) that flattens
//!   the resulting [`Mesh`] into the contiguous `f32`/`u32` buffers a WebGL
//!   `BufferGeometry` expects, with no per-call allocation crossing the FFI
//!   boundary beyond the three returned vectors.
//!
//! # Building for the browser
//!
//! `wasm-bindgen` compiles fine on the host, so `cargo build -p kernel-wasm`
//! and `cargo test -p kernel-wasm` both work without a wasm toolchain. To
//! produce the browser artifact and run the viewer:
//!
//! ```text
//! # from this crate directory (crates/kernel-wasm):
//! wasm-pack build --target web --out-dir web/pkg
//! # then serve the `web/` folder over HTTP (modules require http://, not file://):
//! python3 -m http.server --directory web 8080
//! # open http://localhost:8080/
//! ```
//!
//! `web/viewer.js` imports `./pkg/kernel_wasm.js`, calls [`demo`], and draws the
//! returned mesh with orbit controls (Three.js is pulled from a CDN).

use kernel_core::{Mesh, Resolution, Sdf, Vec3};
use kernel_implicit::{manifold_dual_contour, Cuboid, Cylinder, Node, Sphere};

use wasm_bindgen::prelude::*;

/// Assemble the demo model and mesh it at the given voxel size (world units).
///
/// The model is a deliberately "kernel-y" shape: a rounded body formed by a
/// **smooth union** of a sphere and a cuboid (so the blend region shows off the
/// fillet), with a cylindrical bore **subtracted** straight through it. This
/// exercises a smooth boolean, a hard boolean, and the analytic primitives in a
/// single watertight result.
///
/// A non-finite or non-positive `voxel` falls back to a 0.5mm default, and any
/// positive value is floored at 0.1mm, so a bad argument from the browser
/// cannot stall the mesher with an unbounded or pathologically dense grid.
pub fn build_demo(voxel: f32) -> Mesh {
	let voxel = if voxel.is_finite() && voxel > 0.0 { voxel.max(0.1) } else { 0.5 };

	// Rounded body: sphere blended into a cuboid with a 6mm fillet.
	let body = Node::primitive(Cuboid::new(Vec3::new(6.0, 0.0, 0.0), Vec3::splat(10.0)))
		.smooth_union(Node::primitive(Sphere::new(Vec3::new(-8.0, 0.0, 0.0), 9.0)), 6.0);

	// Through-bore along Y. End-caps reach well outside the body so the cut is
	// a clean tunnel rather than a blind pocket.
	let bore = Node::primitive(Cylinder::new(
		Vec3::new(0.0, -20.0, 0.0),
		Vec3::new(0.0, 20.0, 0.0),
		4.0,
	));

	let model = body.difference(bore);
	// Manifold Dual Contouring, not plain Surface Nets: the hard `difference` carves
	// a concave crease at the bore rims where Surface Nets leaves non-manifold edges
	// at many voxel sizes — MDC keeps the result a watertight 2-manifold throughout.
	manifold_dual_contour(&model, model.bounds(), Resolution::VoxelSize(voxel))
}

/// Flattened mesh buffers laid out for a Three.js `BufferGeometry`.
///
/// `positions` and `normals` are `xyz`-interleaved (length `3 * vertex_count`);
/// `indices` is a flat triangle list (length `3 * triangle_count`). The
/// `#[wasm_bindgen]` getters hand each vector to JS as a typed array
/// (`Float32Array` / `Uint32Array`) by value.
#[wasm_bindgen]
pub struct MeshBuffers {
	positions: Vec<f32>,
	normals: Vec<f32>,
	indices: Vec<u32>,
}

#[wasm_bindgen]
impl MeshBuffers {
	/// Interleaved `xyz` vertex positions (`Float32Array`).
	#[wasm_bindgen(getter)]
	pub fn positions(&self) -> Vec<f32> {
		self.positions.clone()
	}

	/// Interleaved `xyz` unit vertex normals (`Float32Array`).
	#[wasm_bindgen(getter)]
	pub fn normals(&self) -> Vec<f32> {
		self.normals.clone()
	}

	/// Flat triangle index list (`Uint32Array`).
	#[wasm_bindgen(getter)]
	pub fn indices(&self) -> Vec<u32> {
		self.indices.clone()
	}

	/// Number of vertices (`positions.len() / 3`).
	#[wasm_bindgen(getter, js_name = vertexCount)]
	pub fn vertex_count(&self) -> usize {
		self.positions.len() / 3
	}

	/// Number of triangles (`indices.len() / 3`).
	#[wasm_bindgen(getter, js_name = triangleCount)]
	pub fn triangle_count(&self) -> usize {
		self.indices.len() / 3
	}
}

impl MeshBuffers {
	/// Flatten a [`Mesh`] into interleaved buffers, computing vertex normals if
	/// the mesh did not carry one normal per vertex.
	fn from_mesh(mut mesh: Mesh) -> Self {
		if mesh.normals.len() != mesh.positions.len() {
			mesh.compute_normals();
		}

		let mut positions = Vec::with_capacity(mesh.positions.len() * 3);
		for p in &mesh.positions {
			positions.extend_from_slice(&[p.x, p.y, p.z]);
		}

		let mut normals = Vec::with_capacity(mesh.normals.len() * 3);
		for n in &mesh.normals {
			normals.extend_from_slice(&[n.x, n.y, n.z]);
		}

		Self { positions, normals, indices: mesh.indices }
	}
}

/// Build the demo model and return it as flattened buffers for the viewer.
///
/// This is the single entry point the browser calls; it simply forwards to
/// [`build_demo`] and flattens the result.
#[wasm_bindgen]
pub fn demo(voxel: f32) -> MeshBuffers {
	MeshBuffers::from_mesh(build_demo(voxel))
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn demo_meshes_to_a_watertight_solid() {
		// Drive the plain Rust path (never the wasm-bound `demo`), so the test runs
		// on the host without a wasm runtime. Sweep a range of voxel sizes: the hard
		// boolean's concave crease is non-manifold under plain Surface Nets at several
		// of these (0.3 → 78 bad edges, 0.4 → 4), so this guards the Manifold Dual
		// Contouring meshing across resolutions, not just one lucky value.
		for v in [0.3f32, 0.35, 0.4, 0.5, 0.7] {
			let mesh = build_demo(v);
			assert!(mesh.triangle_count() > 0, "demo produced no triangles at voxel {v}");
			assert!(
				mesh.is_watertight(),
				"demo mesh must be a closed manifold at voxel {v}, {} non-manifold edges",
				mesh.non_manifold_edge_count()
			);
		}
	}

	#[test]
	fn flatten_preserves_counts_and_normalizes() {
		let mesh = build_demo(0.6);
		let tris = mesh.triangle_count();
		let verts = mesh.vertex_count();

		let buffers = MeshBuffers::from_mesh(mesh);
		assert_eq!(buffers.vertex_count(), verts, "vertex count must survive flattening");
		assert_eq!(buffers.triangle_count(), tris, "triangle count must survive flattening");
		assert_eq!(buffers.positions().len(), verts * 3);
		assert_eq!(buffers.normals().len(), verts * 3);
		assert_eq!(buffers.indices().len(), tris * 3);

		// compute_normals yields unit (or zero) normals; check the first is unit.
		let n = &buffers.normals();
		let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
		assert!((len - 1.0).abs() < 1e-3, "leading normal should be unit length, got {len}");
	}

	#[test]
	fn nonpositive_voxel_is_clamped_not_hung() {
		// A zero/negative voxel size must be clamped so meshing still terminates.
		let mesh = build_demo(0.0);
		assert!(mesh.triangle_count() > 0, "clamped voxel size should still mesh");
	}
}
