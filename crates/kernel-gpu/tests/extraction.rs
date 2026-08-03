// Copyright (c) LMCAD. Licensed under the MIT License.

//! GPU Surface Nets extraction tests: meshes are `check_mesh`'d and compared
//! against the CPU `surface_nets` ground truth (same algorithm, same lattice
//! layout — see `extract.rs` module docs for what is and is not promised).
//!
//! RUNTIME SKIP: every test here needs a GPU adapter; without one a LOUD skip
//! message is printed and the test passes vacuously (documented in
//! NUMERICS.md).

use kernel_core::math::{Aabb, Vec3};
use kernel_core::{check_mesh, surface_nets, Resolution};
use kernel_gpu::{gpu_surface_nets, GpuContext, GpuError, GpuNode, GpuSurfaceNets};

fn gpu_or_skip(test: &str) -> Option<GpuContext> {
	match GpuContext::new() {
		Ok(ctx) => Some(ctx),
		Err(e) => {
			eprintln!("==================================================================");
			eprintln!("SKIPPED {test}: NO GPU ADAPTER — this test verified NOTHING here.");
			eprintln!("  reason: {e}");
			eprintln!("==================================================================");
			None
		}
	}
}

#[test]
fn gpu_extracts_sphere_watertight_at_analytic_volume() {
	let Some(ctx) = gpu_or_skip("gpu_extracts_sphere_watertight_at_analytic_volume") else { return };
	let tree = GpuNode::sphere(Vec3::new(0.5, -0.25, 1.0), 8.0);
	let domain = tree.bounds();
	let mesh = gpu_surface_nets(&ctx, &tree, domain, Resolution::VoxelSize(0.25)).expect("gpu extraction");
	let cpu = surface_nets(&tree.to_node(), domain, Resolution::VoxelSize(0.25));
	let r = check_mesh(&mesh);
	let (vol, cpu_vol) = (mesh.signed_volume(), cpu.signed_volume());
	let want = 4.0 / 3.0 * std::f64::consts::PI * 512.0;
	assert!(
		mesh.is_watertight()
			&& r.non_manifold_edges == 0
			&& r.boundary_edges == 0
			&& (vol - want).abs() / want < 0.01
			&& (vol - cpu_vol).abs() / cpu_vol < 1e-3
			&& (mesh.vertex_count() as f64 - cpu.vertex_count() as f64).abs() / (cpu.vertex_count() as f64) < 0.01,
		"gpu sphere: watertight={} nme={} bnd={} vol={vol:.2} (analytic {want:.2}, cpu {cpu_vol:.2}) verts={}/{}",
		mesh.is_watertight(),
		r.non_manifold_edges,
		r.boundary_edges,
		mesh.vertex_count(),
		cpu.vertex_count()
	);
}

#[test]
fn gpu_matches_cpu_on_a_drilled_csg_part() {
	let Some(ctx) = gpu_or_skip("gpu_matches_cpu_on_a_drilled_csg_part") else { return };
	// 20 mm cube minus a through-hole, with a fillet seam — a real CSG part
	// through the full compile+extract path at two resolutions.
	let part = || {
		GpuNode::cuboid(Vec3::ZERO, Vec3::splat(10.0))
			.fillet_union(GpuNode::cylinder(Vec3::new(0.0, 0.0, 10.0), Vec3::new(0.0, 0.0, 14.0), 4.0), 2.0)
			.difference(GpuNode::cylinder(Vec3::new(0.0, -11.0, 0.0), Vec3::new(0.0, 11.0, 0.0), 4.0))
	};
	let tree = part();
	let domain = tree.bounds().pad(0.5);
	let extractor = GpuSurfaceNets::compile(&ctx, &tree).expect("compile");
	for vs in [0.4f32, 0.25] {
		let mesh = extractor.extract(domain, Resolution::VoxelSize(vs)).expect("gpu extraction");
		let cpu = surface_nets(&tree.to_node(), domain, Resolution::VoxelSize(vs));
		let (vol, cpu_vol) = (mesh.signed_volume(), cpu.signed_volume());
		let r = check_mesh(&mesh);
		assert!(
			mesh.is_watertight() && r.boundary_edges == 0 && (vol - cpu_vol).abs() / cpu_vol < 1e-3,
			"drilled part @ {vs}: watertight={} bnd={} vol gpu={vol:.2} cpu={cpu_vol:.2}",
			mesh.is_watertight(),
			r.boundary_edges
		);
	}
}

#[test]
fn gpu_gyroid_lattice_block_is_closed_and_matches_cpu_volume() {
	let Some(ctx) = gpu_or_skip("gpu_gyroid_lattice_block_is_closed_and_matches_cpu_volume") else { return };
	// The signature TPMS workload: a 0.6-thick gyroid shell ∩ a 40 mm cube.
	// Surface Nets (CPU or GPU) is the one-vertex-per-cell dual, so we assert
	// what IS true of it: a CLOSED surface (zero boundary edges) whose volume
	// matches the CPU march; full 2-manifoldness on TPMS shells is Manifold
	// DC's job (the CPU watertight authority), not this preview path's.
	let half = 20.0f32;
	let region = Aabb::from_center_half_extent(Vec3::ZERO, Vec3::splat(half));
	let tree = GpuNode::gyroid(region, 0.35, 0.6).intersection(GpuNode::cuboid(Vec3::ZERO, Vec3::splat(half)));
	let mesh = gpu_surface_nets(&ctx, &tree, region, Resolution::VoxelSize(0.4)).expect("gpu extraction");
	let cpu = surface_nets(&tree.to_node(), region, Resolution::VoxelSize(0.4));
	let r = check_mesh(&mesh);
	let rc = check_mesh(&cpu);
	let (vol, cpu_vol) = (mesh.signed_volume(), cpu.signed_volume());
	assert!(
		r.boundary_edges == 0
			&& rc.boundary_edges == 0
			&& mesh.triangle_count() > 100_000
			&& (vol - cpu_vol).abs() / cpu_vol < 2e-3,
		"gyroid block: gpu bnd={} cpu bnd={} tris={} vol gpu={vol:.1} cpu={cpu_vol:.1} (nme gpu={} cpu={} — informational)",
		r.boundary_edges,
		rc.boundary_edges,
		mesh.triangle_count(),
		r.non_manifold_edges,
		rc.non_manifold_edges
	);
	println!(
		"gyroid 40mm @ 0.4: gpu {} tris vol {vol:.1} (nme {}), cpu {} tris vol {cpu_vol:.1} (nme {})",
		mesh.triangle_count(),
		r.non_manifold_edges,
		cpu.triangle_count(),
		rc.non_manifold_edges
	);
}

#[test]
fn gpu_extracts_the_branchy_beam_lattice_closed() {
	let Some(ctx) = gpu_or_skip("gpu_extracts_the_branchy_beam_lattice_closed") else { return };
	// A small explicit truss (tapered + capsule + degenerate struts) through
	// the strut storage buffer: the extraction must be CLOSED and match the
	// CPU march's volume. (Junction-rich lattices can pinch non-manifold
	// edges in ANY one-vertex-per-cell dual — the honest lattice caveat from
	// kernel-implicit applies equally here; closure and volume are the gates.)
	let nodes = vec![
		Vec3::new(-6.0, 0.0, 0.0),
		Vec3::new(6.0, 0.0, 0.0),
		Vec3::new(0.0, 8.0, 0.0),
		Vec3::new(0.0, 3.0, 7.0),
	];
	let struts = vec![
		(0, 1, 1.5, 1.0),
		(1, 2, 1.0, 1.0),
		(2, 0, 1.0, 1.4),
		(0, 3, 1.2, 0.8),
		(1, 3, 1.2, 0.8),
		(2, 3, 1.2, 0.8),
	];
	let tree = GpuNode::lattice(nodes, struts);
	let domain = tree.bounds().pad(1.0);
	let mesh = gpu_surface_nets(&ctx, &tree, domain, Resolution::VoxelSize(0.2)).expect("gpu extraction");
	let cpu = surface_nets(&tree.to_node(), domain, Resolution::VoxelSize(0.2));
	let r = check_mesh(&mesh);
	let (vol, cpu_vol) = (mesh.signed_volume(), cpu.signed_volume());
	assert!(
		r.boundary_edges == 0 && (vol - cpu_vol).abs() / cpu_vol < 1e-3 && vol > 0.0,
		"truss: bnd={} vol gpu={vol:.2} cpu={cpu_vol:.2}",
		r.boundary_edges
	);
}

#[test]
fn gpu_extraction_is_deterministic_run_to_run() {
	let Some(ctx) = gpu_or_skip("gpu_extraction_is_deterministic_run_to_run") else { return };
	// Prefix-sum compaction (no atomics) makes the extraction bit-stable on
	// the same device/driver — the GPU analogue of the meshers' determinism
	// note in NUMERICS.md. Two fresh extractions must agree to the BIT.
	let tree = GpuNode::sphere(Vec3::ZERO, 6.0).smooth_union(GpuNode::cuboid(Vec3::new(5.0, 0.0, 0.0), Vec3::splat(4.0)), 1.5);
	let domain = tree.bounds().pad(0.5);
	let extractor = GpuSurfaceNets::compile(&ctx, &tree).expect("compile");
	let a = extractor.extract(domain, Resolution::VoxelSize(0.3)).expect("first extraction");
	let b = extractor.extract(domain, Resolution::VoxelSize(0.3)).expect("second extraction");
	let pos_bits = |m: &kernel_core::Mesh| m.positions.iter().flat_map(|p| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]).collect::<Vec<u32>>();
	assert!(
		pos_bits(&a) == pos_bits(&b) && a.indices == b.indices,
		"two GPU extractions of the same tree must be bit-identical ({} vs {} verts, {} vs {} tris)",
		a.vertex_count(),
		b.vertex_count(),
		a.triangle_count(),
		b.triangle_count()
	);
}

#[test]
fn gpu_extraction_guards_mirror_cpu_and_refuse_oversize_loudly() {
	let Some(ctx) = gpu_or_skip("gpu_extraction_guards_mirror_cpu_and_refuse_oversize_loudly") else { return };
	// CPU-mirrored guards: A − A has no surface (empty mesh, not an error);
	// a bare half-space has an unbounded domain (empty mesh — same as the
	// CPU's bare_plane_is_not_meshable). Over the dense-lattice cap the CPU
	// silently returns empty (documented sharp edge) — the GPU api instead
	// refuses with a structured TooLarge error.
	let cube = || GpuNode::cuboid(Vec3::ZERO, Vec3::splat(10.0));
	let nothing = cube().difference(cube());
	let mesh = gpu_surface_nets(&ctx, &nothing, cube().bounds(), Resolution::VoxelSize(0.5)).expect("empty extraction");
	assert!(mesh.is_empty(), "A - A must extract to an empty mesh, got {} tris", mesh.triangle_count());

	let plane = GpuNode::plane(Vec3::ZERO, Vec3::Y);
	let mesh = gpu_surface_nets(&ctx, &plane, plane.bounds(), Resolution::VoxelSize(1.0)).expect("unbounded domain");
	assert!(mesh.is_empty(), "a bare half-space is not meshable (CPU parity)");

	let big = GpuSurfaceNets::compile(&ctx, &cube()).expect("compile");
	let err = big.extract(cube().bounds(), Resolution::VoxelSize(1e-4)).expect_err("over-cap extraction must refuse");
	assert!(matches!(err, GpuError::TooLarge { .. }), "expected TooLarge, got: {err}");
}
