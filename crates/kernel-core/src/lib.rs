// Copyright (c) LMCAD. Licensed under the MIT License.

//! `kernel-core` — foundation layer of the hybrid B-rep + voxel geometry kernel.
//!
//! Provides the shared math vocabulary, the unifying [`Sdf`] trait, the triangle
//! [`Mesh`] output type with exporters, and the Surface Nets mesher. Both the
//! implicit/voxel half and the exact B-rep half build on these contracts.

pub mod bvh;
pub mod par;
pub mod clearance;
pub mod hull;
pub mod manifold;
pub mod marching;
pub mod math;
pub mod mesh;
pub mod meshcheck;
pub mod mesher;
pub mod mesher_f64;
pub mod poly2;
pub mod predicates;
pub mod sdf;
pub mod telemetry;

pub use bvh::MeshBvh;
pub use clearance::radial_wave_field;
pub use hull::convex_hull;
pub use manifold::make_manifold;
pub use math::{Aabb, Obb, Ray, Vec3};
pub use mesh::{
	closest_point_on_triangle, ClosestPoint, DraftReport, MassProperties, Mesh, OverhangReport, PrincipalAxes, RayHit,
	SectionProperties, SupportFreeReport, ThicknessReport,
};
pub use meshcheck::{check_mesh, MeshReport};
pub use mesher::{surface_nets, Resolution};
pub use mesher_f64::{dual_contour_f64, dual_contour_sdf_f64, surface_nets_f64, surface_nets_sdf_f64, MeshF64};
pub use poly2::{polygon_area, polygon_intersection_area};
pub use predicates::{incircle, incircle_exact, orient2d, orient2d_exact, orient3d, orient3d_exact};
pub use sdf::{central_difference, Sdf};

#[cfg(test)]
mod tests {
	use super::*;

	/// A bare analytic sphere, used to exercise the core meshing path.
	struct Sphere {
		center: Vec3,
		radius: f32,
	}

	impl Sdf for Sphere {
		fn distance(&self, p: Vec3) -> f32 {
			(p - self.center).length() - self.radius
		}
		fn bounds(&self) -> Aabb {
			Aabb::from_center_half_extent(self.center, Vec3::splat(self.radius + 0.5))
		}
		fn gradient(&self, p: Vec3) -> Vec3 {
			(p - self.center).normalize_or_zero()
		}
	}

	#[test]
	fn sphere_meshes_to_correct_volume_and_is_watertight() {
		let s = Sphere { center: Vec3::ZERO, radius: 10.0 };
		let mesh = surface_nets(&s, s.bounds(), Resolution::VoxelSize(0.5));

		let exact = 4.0 / 3.0 * std::f64::consts::PI * 10.0f64.powi(3);
		let rel_err = (mesh.signed_volume() - exact).abs() / exact;

		assert!(!mesh.is_empty(), "mesh should not be empty");
		assert!(mesh.is_watertight(), "surface nets output must be watertight");
		assert!(rel_err < 0.01, "volume rel err {rel_err} too large (got {})", mesh.signed_volume());
	}

	#[test]
	fn sphere_mass_properties_match_closed_form() {
		// An off-center sphere: CoM tracks the center, and the inertia about the CoM
		// is the isotropic 2/5·m·r² regardless of where the sphere sits (so the
		// parallel-axis shift is exercised). Tessellated → agreement to a few percent.
		let center = Vec3::new(4.0, -3.0, 2.0);
		let s = Sphere { center, radius: 8.0 };
		let mp = surface_nets(&s, s.bounds(), Resolution::VoxelSize(0.4)).mass_properties();

		assert!((mp.center_of_mass - center.as_dvec3()).length() < 0.1, "CoM {:?} vs {center:?}", mp.center_of_mass);
		let exact = 0.4 * mp.volume * 64.0;
		for i in [mp.inertia.x_axis.x, mp.inertia.y_axis.y, mp.inertia.z_axis.z] {
			assert!((i - exact).abs() / exact < 0.02, "sphere principal inertia {i} vs {exact}");
		}
	}

	#[test]
	fn bad_voxel_size_yields_empty_mesh_not_overflow() {
		// 0 / negative / NaN / inf voxel sizes were laundered to 1e-6, blowing the
		// grid up to billions of cells and overflowing. They must now mesh to empty.
		let s = Sphere { center: Vec3::ZERO, radius: 5.0 };
		// 0 / negative / NaN / inf, plus a finite-but-tiny size whose lattice would
		// overflow `nx*ny*nz` (~1e21 cells) — all must mesh to empty, never panic/OOM.
		for v in [0.0f32, -1.0, f32::NAN, f32::INFINITY, 1e-7] {
			let m = surface_nets(&s, s.bounds(), Resolution::VoxelSize(v));
			assert!(m.is_empty(), "VoxelSize({v}) must yield an empty mesh, got {} tris", m.triangle_count());
		}
	}

	#[test]
	fn empty_mesh_is_not_watertight() {
		// check_mesh's watertight flag must agree with Mesh::is_watertight on the empty
		// mesh (a mesh that bounds nothing is not a closed solid).
		assert!(!check_mesh(&Mesh::new()).watertight, "an empty mesh must not report watertight");
		assert!(!Mesh::new().is_watertight());
	}

	#[test]
	fn fill_holes_restores_watertightness() {
		// A watertight sphere, then punch a hole by deleting the triangle umbrella of
		// one vertex. Filling the resulting boundary loop must close it again and
		// restore (approximately) the volume.
		let s = Sphere { center: Vec3::ZERO, radius: 10.0 };
		let whole = surface_nets(&s, s.bounds(), Resolution::VoxelSize(0.4));
		let v0 = whole.signed_volume();
		assert!(whole.is_watertight(), "the source sphere is watertight");

		let mut holed = whole.clone();
		holed.indices = holed.indices.chunks_exact(3).filter(|t| !t.contains(&0)).flatten().copied().collect();
		assert!(!holed.is_watertight(), "removing a vertex umbrella opens a hole");

		let n = holed.fill_holes();
		assert_eq!(n, 1, "exactly one hole to fill");
		assert!(holed.is_watertight(), "the filled mesh is watertight again");
		assert!((holed.signed_volume() - v0).abs() / v0.abs() < 0.02, "volume restored {} vs {v0}", holed.signed_volume());
	}

	#[test]
	fn decimate_reduces_triangles_and_stays_finite() {
		let s = Sphere { center: Vec3::ZERO, radius: 10.0 };
		let dense = surface_nets(&s, s.bounds(), Resolution::VoxelSize(0.4));
		let low = dense.decimate(1.5);

		assert!(low.triangle_count() < dense.triangle_count() / 2, "decimated to far fewer triangles");
		assert!(!low.is_empty(), "decimation should keep a surface");
		assert!(low.positions.iter().all(|p| p.is_finite()), "no non-finite vertices");
		// No degenerate triangles remain.
		for t in low.indices.chunks_exact(3) {
			assert!(t[0] != t[1] && t[1] != t[2] && t[0] != t[2], "no degenerate triangle");
		}
		// Volume is approximately preserved (clustering at 1.5mm on a Ø20 sphere).
		let (dv, lv) = (dense.signed_volume(), low.signed_volume());
		assert!((lv - dv).abs() / dv.abs() < 0.2, "decimated volume {lv} vs {dv}");
	}

	#[test]
	fn stl_binary_roundtrip() {
		let s = Sphere { center: Vec3::ZERO, radius: 8.0 };
		let original = surface_nets(&s, s.bounds(), Resolution::VoxelSize(0.6));
		let bytes = original.to_stl_binary();
		let mut read = Mesh::from_stl_bytes(&bytes).expect("parse binary STL");
		assert_eq!(read.triangle_count(), original.triangle_count(), "triangle count preserved");
		read.weld(1e-4); // STL is a soup; weld to recover shared topology
		assert!(read.is_watertight(), "re-read + welded mesh should be watertight");
		let (v0, v1) = (original.signed_volume(), read.signed_volume());
		assert!((v1 - v0).abs() / v0.abs() < 1e-4, "round-trip volume {v1} vs {v0}");
	}

	#[test]
	fn stl_import_never_panics_on_malformed_input() {
		// Targeted malformed inputs: the external-input parser must return
		// Ok/Err, never panic.
		let mut targeted: Vec<Vec<u8>> = vec![
			vec![],
			vec![0u8; 83],          // shorter than the 84-byte binary header
			vec![0u8; 84],          // header only, count = 0
			b"solid x\n".to_vec(),  // ASCII with no facets
			b"facet normal 0 0\n vertex 1 2\nendfacet\n".to_vec(), // truncated fields
			b"\xff\xfe garbage \x00\x01 not utf8".to_vec(),
		];
		// A binary header claiming a huge count with no body (size mismatch → ASCII).
		let mut huge = vec![0u8; 84];
		huge[80..84].copy_from_slice(&u32::MAX.to_le_bytes());
		targeted.push(huge);
		for c in &targeted {
			let _ = Mesh::from_stl_bytes(c); // must not panic
		}

		// Correctly-sized binary with arbitrary (garbage) body bytes must parse to
		// exactly `count` triangles without panicking.
		for count in [0usize, 1, 3, 17] {
			let mut v = vec![0u8; 84 + 50 * count];
			v[80..84].copy_from_slice(&(count as u32).to_le_bytes());
			for (i, b) in v[84..].iter_mut().enumerate() {
				*b = (i as u8).wrapping_mul(37).wrapping_add(13);
			}
			let m = Mesh::from_stl_bytes(&v).expect("sized binary parses");
			assert_eq!(m.triangle_count(), count, "binary count honored");
		}

		// Pseudo-random byte arrays (xorshift) — none may panic the parser.
		let mut s = 0x2545_f491_4f6c_dd1du64;
		let mut next = || {
			s ^= s << 13;
			s ^= s >> 7;
			s ^= s << 17;
			s
		};
		for _ in 0..3000 {
			let len = (next() % 400) as usize;
			let bytes: Vec<u8> = (0..len).map(|_| (next() & 0xff) as u8).collect();
			let _ = Mesh::from_stl_bytes(&bytes); // must not panic
		}
	}

	#[test]
	fn glb_export_structure() {
		let s = Sphere { center: Vec3::ZERO, radius: 6.0 };
		let m = surface_nets(&s, s.bounds(), Resolution::VoxelSize(0.8));
		let glb = m.to_glb();
		let u32at = |o: usize| u32::from_le_bytes(glb[o..o + 4].try_into().unwrap());

		assert_eq!(&glb[0..4], b"glTF", "GLB magic");
		assert_eq!(u32at(4), 2, "glTF version 2");
		assert_eq!(u32at(8) as usize, glb.len(), "header length == total");

		let json_len = u32at(12) as usize;
		assert_eq!(&glb[16..20], b"JSON", "first chunk is JSON");
		let json = std::str::from_utf8(&glb[20..20 + json_len]).unwrap();
		assert!(json.contains(&format!("\"count\":{}", m.vertex_count())), "POSITION/NORMAL accessor count");
		assert!(json.contains(&format!("\"count\":{}", m.indices.len())), "index accessor count");

		let bin_hdr = 20 + json_len;
		let bin_len = u32at(bin_hdr) as usize;
		assert_eq!(&glb[bin_hdr + 4..bin_hdr + 8], b"BIN\0", "second chunk is BIN");
		assert_eq!(bin_hdr + 8 + bin_len, glb.len(), "BIN chunk fills to end");
	}

	#[test]
	fn stl_ascii_parse() {
		let ascii = "solid tri\n\
			facet normal 0 0 1\n outer loop\n  vertex 0 0 0\n  vertex 1 0 0\n  vertex 0 1 0\n endloop\nendfacet\n\
			endsolid tri\n";
		let m = Mesh::from_stl_bytes(ascii.as_bytes()).expect("parse ASCII STL");
		assert_eq!(m.triangle_count(), 1);
		assert_eq!(m.positions.len(), 3);
		assert!((m.positions[1] - Vec3::new(1.0, 0.0, 0.0)).length() < 1e-6, "vertex parsed");
		assert!((m.normals[0] - Vec3::Z).length() < 1e-6, "facet normal parsed");
	}

	#[test]
	fn weld_merges_within_tolerance_across_cell_boundary() {
		let mut m = Mesh::new();
		// Points 0 and 1 are 0.2e-3 apart (< 1e-3) but straddle the quantization
		// cell boundary at 0.5e-3; point 2 is far away.
		m.positions = vec![
			Vec3::new(0.6e-3, 0.0, 0.0),
			Vec3::new(0.4e-3, 0.0, 0.0),
			Vec3::new(1.0, 0.0, 0.0),
		];
		m.indices = vec![0, 1, 2];
		m.weld(1e-3);
		// The pair merges — and the triangle it collapsed is DROPPED (a collapsed
		// needle double-counts its long edge and breaks watertightness; see
		// tests/weld_collapse.rs).
		assert_eq!(m.vertex_count(), 2, "near-coincident pair should weld to one vertex");
		assert!(m.indices.is_empty(), "the triangle collapsed by the weld must be dropped, indices = {:?}", m.indices);
	}

	#[test]
	fn weld_keeps_points_beyond_tolerance() {
		let mut m = Mesh::new();
		m.positions = vec![Vec3::ZERO, Vec3::new(0.01, 0.0, 0.0)]; // 0.01 apart, tol 1e-3
		m.indices = vec![0, 0, 1];
		m.weld(1e-3);
		assert_eq!(m.vertex_count(), 2, "points beyond tolerance must not merge");
	}

	#[test]
	fn aabb_union_and_contains() {
		let a = Aabb::new(Vec3::ZERO, Vec3::splat(1.0));
		let b = Aabb::new(Vec3::splat(2.0), Vec3::splat(3.0));
		let u = a.union(b);
		assert_eq!(u, Aabb::new(Vec3::ZERO, Vec3::splat(3.0)));
		assert!(u.contains(Vec3::splat(1.5)));
		assert!(!a.contains(Vec3::splat(2.5)));
	}
}
