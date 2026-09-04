// Copyright (c) LMCAD. Licensed under the MIT License.

//! End-to-end cross-crate integration test exercising the spec's full data flow:
//! **author** exactly in B-rep, round-trip through mesh I/O, **bridge** into the
//! implicit world, **combine** robustly via CSG, **output** a watertight mesh,
//! and drive it all from the parametric model. Guards against integration
//! regressions that no single-crate unit test would catch.

use kernel_brep::math::DVec3;
use kernel_brep::{cuboid, tessellate_default};
use kernel_core::{check_mesh, make_manifold, Mesh, Resolution};
use kernel_implicit::{dual_contour_narrowband, surface_nets, Cylinder, MeshSdf, Node, Sdf, Sphere, Vec3, VoxelGrid};
use kernel_model::{BooleanOp, Dim, Document, Feature};

#[test]
fn full_pipeline_author_combine_output() {
	// 1) AUTHOR a flat tab in exact B-rep, tessellate it.
	let tab = cuboid(DVec3::new(-10.0, -6.0, -2.0), DVec3::new(10.0, 6.0, 2.0));
	let tab_mesh = tessellate_default(&tab);
	assert!(tab_mesh.is_watertight(), "B-rep tessellation is watertight");

	// Round-trip through binary STL (write → read) and validate GLB structure.
	let stl = tab_mesh.to_stl_binary();
	let reread = Mesh::from_stl_bytes(&stl).expect("re-read STL");
	assert_eq!(reread.triangle_count(), tab_mesh.triangle_count(), "STL round-trip count");
	let glb = tab_mesh.to_glb();
	assert_eq!(&glb[0..4], b"glTF", "GLB is structurally valid");

	// 2) BRIDGE the re-read mesh (an unwelded soup) into the implicit world via
	// the winding-number SDF, then voxelize it once for fast CSG sampling.
	let tab_sdf = MeshSdf::new(&reread);
	let tab_grid = VoxelGrid::from_sdf(&tab_sdf, tab_sdf.bounds(), 0.4);
	// The bridge preserves the tab's volume (20 × 12 × 4 = 960 mm³).
	let bridged = surface_nets(&tab_grid, tab_grid.bounds(), Resolution::VoxelSize(0.4));
	assert!((bridged.signed_volume() - 960.0).abs() / 960.0 < 0.05, "bridge round-trip volume {}", bridged.signed_volume());

	// 3) COMBINE: a smooth body ∪ the exact tab, minus a drilled hole — one CSG tree.
	let body = Node::primitive(Sphere::new(Vec3::ZERO, 8.0));
	let hole = Node::primitive(Cylinder::new(Vec3::new(0.0, 0.0, -12.0), Vec3::new(0.0, 0.0, 12.0), 3.0));
	let part = body.smooth_union(Node::primitive(tab_grid), 3.0).difference(hole);

	// 4) OUTPUT: mesh with area-scaling Dual Contouring, repair, validate.
	let mut mesh = dual_contour_narrowband(&part, part.bounds().pad(1.0), Resolution::VoxelSize(0.4));
	mesh = make_manifold(&mesh);
	let report = check_mesh(&mesh);
	assert!(mesh.positions.iter().all(|p| p.is_finite()), "no non-finite vertices");
	assert_eq!(report.boundary_edges, 0, "closed (no holes)");
	assert_eq!(report.non_orientable_edges, 0, "orientable");
	assert!(mesh.signed_volume() > 0.0, "outward, positive volume");

	// 5) PARAMETRIC: a plate with a parameter-driven hole; editing the parameter
	// re-evaluates the feature tree and changes the volume.
	let mut doc = Document::new();
	doc.set_param("hole_r", 3.0);
	let plate = doc.add(Feature::Box {
		center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
		size: [Dim::Literal(30.0), Dim::Literal(20.0), Dim::Literal(5.0)],
	});
	let hole_feat = doc.add(Feature::Cylinder {
		center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
		radius: Dim::param("hole_r"),
		height: Dim::Literal(20.0),
	});
	let result = doc.add(Feature::Boolean { op: BooleanOp::Difference, a: plate, b: hole_feat });
	doc.set_root(result);
	let v_small = doc.mesh(0.5f32).signed_volume();
	doc.set_param("hole_r", 6.0);
	let v_big = doc.mesh(0.5f32).signed_volume();
	assert!(v_big < v_small, "enlarging the hole removes more material ({v_big} < {v_small})");
}
