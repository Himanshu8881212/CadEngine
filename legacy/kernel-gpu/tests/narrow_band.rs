// Copyright (c) LMCAD. Licensed under the MIT License.

//! GPU narrow-band extraction gates: parity against the dense GPU extractor
//! AND the CPU `surface_nets` oracle, sparsity receipts on the work counters,
//! multi-shell closure, the documented band clamp, and the TPMS egg-crate
//! (where naive banding cracks).
//!
//! RUNTIME SKIP: every test here needs a GPU adapter; without one a LOUD skip
//! message is printed and the test passes vacuously (the established
//! kernel-gpu pattern — see tests/extraction.rs and NUMERICS.md).

use std::time::Instant;

use kernel_core::math::{Aabb, Vec3};
use kernel_core::{check_mesh, surface_nets, Resolution};
use kernel_gpu::{
	extract_narrow_band_with_stats, gpu_surface_nets, GpuContext, GpuError, GpuNarrowBand, GpuNode, GpuSurfaceNets,
};

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

/// Connected shells of a triangle mesh via union-find over shared vertices.
/// (`check_mesh`'s `MeshReport` carries no shell count, so the multi-shell
/// gate computes it here — Surface Nets shares vertices within a shell and
/// disjoint solids share none, so vertex connectivity IS shell connectivity.)
fn shell_count(m: &kernel_core::Mesh) -> usize {
	let n = m.vertex_count();
	let mut parent: Vec<u32> = (0..n as u32).collect();
	fn find(parent: &mut [u32], mut x: u32) -> u32 {
		while parent[x as usize] != x {
			parent[x as usize] = parent[parent[x as usize] as usize];
			x = parent[x as usize];
		}
		x
	}
	let mut used = vec![false; n];
	for t in m.triangles() {
		for k in 0..3 {
			used[t[k] as usize] = true;
			let (a, b) = (find(&mut parent, t[k]), find(&mut parent, t[(k + 1) % 3]));
			if a != b {
				parent[a as usize] = b;
			}
		}
	}
	let mut roots = std::collections::HashSet::new();
	for v in 0..n as u32 {
		if used[v as usize] {
			roots.insert(find(&mut parent, v));
		}
	}
	roots.len()
}

/// Position bits + indices, for bit-identity pins.
fn mesh_bits(m: &kernel_core::Mesh) -> (Vec<u32>, Vec<u32>) {
	(
		m.positions.iter().flat_map(|p| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]).collect(),
		m.indices.clone(),
	)
}

/// The drilled CSG part from tests/extraction.rs (cube + filleted boss minus
/// a through-hole) — the same real-part falsifier, so results are directly
/// comparable across the dense and narrow-band suites.
fn drilled_part() -> GpuNode {
	GpuNode::cuboid(Vec3::ZERO, Vec3::splat(10.0))
		.fillet_union(GpuNode::cylinder(Vec3::new(0.0, 0.0, 10.0), Vec3::new(0.0, 0.0, 14.0), 4.0), 2.0)
		.difference(GpuNode::cylinder(Vec3::new(0.0, -11.0, 0.0), Vec3::new(0.0, 11.0, 0.0), 4.0))
}

#[test]
fn narrow_band_matches_dense_gpu_and_cpu_on_sphere_and_drilled_part() {
	let Some(ctx) = gpu_or_skip("narrow_band_matches_dense_gpu_and_cpu_on_sphere_and_drilled_part") else { return };
	// GATE 1a: sphere — narrow band vs dense GPU vs CPU vs analytic volume.
	let tree = GpuNode::sphere(Vec3::new(0.5, -0.25, 1.0), 8.0);
	let domain = tree.bounds();
	let vs = 0.25f32;
	let (nb, stats) = extract_narrow_band_with_stats(&ctx, &tree, domain, Resolution::VoxelSize(vs), 0.0).expect("narrow-band extraction");
	let dense = gpu_surface_nets(&ctx, &tree, domain, Resolution::VoxelSize(vs)).expect("dense extraction");
	let cpu = surface_nets(&tree.to_node(), domain, Resolution::VoxelSize(vs));
	let r = check_mesh(&nb);
	let (nbv, dv, cv) = (nb.signed_volume(), dense.signed_volume(), cpu.signed_volume());
	let want = 4.0 / 3.0 * std::f64::consts::PI * 512.0;
	assert!(
		nb.is_watertight()
			&& r.boundary_edges == 0
			&& r.non_manifold_edges == 0
			&& (nbv - dv).abs() / dv.abs() < 1e-3
			&& (nbv - cv).abs() / cv.abs() < 1e-3
			&& (nbv - want).abs() / want < 0.01,
		"nb sphere: watertight={} bnd={} nme={} vol nb={nbv:.3} dense={dv:.3} cpu={cv:.3} analytic={want:.3} \
		 (|nb-dense|/dense={:.2e}, |nb-cpu|/cpu={:.2e}); {stats:?}",
		nb.is_watertight(),
		r.boundary_edges,
		r.non_manifold_edges,
		(nbv - dv).abs() / dv.abs(),
		(nbv - cv).abs() / cv.abs(),
	);
	println!(
		"nb sphere @ {vs}: vol nb={nbv:.4} dense={dv:.4} cpu={cv:.4} (analytic {want:.4}), \
		 active {}/{} blocks, {} samples (dense lattice {})",
		stats.active_blocks, stats.total_blocks, stats.samples_evaluated, stats.dense_samples
	);

	// GATE 1b: the drilled CSG part (fillet seam + through-hole) at the same
	// resolution the dense suite pins.
	let part = drilled_part();
	let pdomain = part.bounds().pad(0.5);
	let (pnb, pstats) = extract_narrow_band_with_stats(&ctx, &part, pdomain, Resolution::VoxelSize(vs), 0.0).expect("narrow-band extraction");
	let pdense = gpu_surface_nets(&ctx, &part, pdomain, Resolution::VoxelSize(vs)).expect("dense extraction");
	let pcpu = surface_nets(&part.to_node(), pdomain, Resolution::VoxelSize(vs));
	let pr = check_mesh(&pnb);
	let (pnbv, pdv, pcv) = (pnb.signed_volume(), pdense.signed_volume(), pcpu.signed_volume());
	assert!(
		pnb.is_watertight() && pr.boundary_edges == 0 && (pnbv - pdv).abs() / pdv.abs() < 1e-3 && (pnbv - pcv).abs() / pcv.abs() < 1e-3,
		"nb drilled part: watertight={} bnd={} vol nb={pnbv:.3} dense={pdv:.3} cpu={pcv:.3} \
		 (|nb-dense|/dense={:.2e}, |nb-cpu|/cpu={:.2e}); {pstats:?}",
		pnb.is_watertight(),
		pr.boundary_edges,
		(pnbv - pdv).abs() / pdv.abs(),
		(pnbv - pcv).abs() / pcv.abs(),
	);
	println!(
		"nb drilled part @ {vs}: vol nb={pnbv:.4} dense={pdv:.4} cpu={pcv:.4}, active {}/{} blocks, {} samples (dense {})",
		pstats.active_blocks, pstats.total_blocks, pstats.samples_evaluated, pstats.dense_samples
	);
}

#[test]
fn narrow_band_sparsity_receipts_on_a_big_fine_sphere() {
	let Some(ctx) = gpu_or_skip("narrow_band_sparsity_receipts_on_a_big_fine_sphere") else { return };
	// GATE 2: a 60 mm sphere at a fine voxel — the "big domain, thin feature"
	// case the narrow band exists for. The assertions are on the WORK COUNTERS
	// (active-block ratio, field samples), which are deterministic; wall-clock
	// speedup is MEASURED AND PRINTED but deliberately NOT asserted — machine
	// load makes timing assertions flaky, while the counters cannot lie.
	let tree = GpuNode::sphere(Vec3::ZERO, 30.0);
	let domain = tree.bounds();
	let vs = 0.15f32;
	let nbx = GpuNarrowBand::compile(&ctx, &tree).expect("nb compile");
	let dx = GpuSurfaceNets::compile(&ctx, &tree).expect("dense compile");

	let t0 = Instant::now();
	let (nb, stats) = nbx.extract_with_stats(domain, Resolution::VoxelSize(vs), 0.0).expect("narrow-band extraction");
	let t_nb = t0.elapsed();
	let t0 = Instant::now();
	let dense = dx.extract(domain, Resolution::VoxelSize(vs)).expect("dense extraction");
	let t_dense = t0.elapsed();

	let ratio = stats.active_blocks as f64 / stats.total_blocks as f64;
	let sample_gain = stats.dense_samples as f64 / stats.samples_evaluated as f64;
	let (nbv, dv) = (nb.signed_volume(), dense.signed_volume());
	// is_watertight (edge/vertex topology) rather than full check_mesh here:
	// the self-intersection BVH on a ~1M-triangle sphere adds minutes and the
	// closure gates already run full check_mesh on the smaller-part tests.
	// Pins carry ~20% headroom over the measured receipts on Apple M-series /
	// Metal: ratio 0.0828, samples 11 630 951 vs 65 450 827 (5.63x fewer).
	assert!(
		nb.is_watertight()
			&& (nbv - dv).abs() / dv.abs() < 1e-3
			&& ratio < 0.10
			&& stats.samples_evaluated * 5 < stats.dense_samples,
		"nb sparsity: watertight={} vol nb={nbv:.2} dense={dv:.2} (rel {:.2e}); \
		 active {}/{} blocks (ratio {ratio:.4}, pinned < 0.10); \
		 samples {} vs dense lattice {} ({sample_gain:.2}x fewer, pinned > 5x); \
		 wall-clock nb {t_nb:?} vs dense {t_dense:?} (informational only)",
		nb.is_watertight(),
		(nbv - dv).abs() / dv.abs(),
		stats.active_blocks,
		stats.total_blocks,
		stats.samples_evaluated,
		stats.dense_samples,
	);
	println!(
		"nb sparsity 60mm sphere @ {vs}: ratio {ratio:.4} ({}/{} blocks), samples {} vs {} ({sample_gain:.2}x), \
		 wall nb {t_nb:?} vs dense {t_dense:?} ({:.2}x)",
		stats.active_blocks,
		stats.total_blocks,
		stats.samples_evaluated,
		stats.dense_samples,
		t_dense.as_secs_f64() / t_nb.as_secs_f64().max(1e-9)
	);

	// GATE 2b: beyond the dense cap. At 0.09 mm the conceptual lattice
	// (670³ ≈ 3.0e8 cells) exceeds the dense 2²⁸ cap — the dense GPU extractor
	// must refuse loudly, and the narrow band must still deliver a closed
	// sphere at the analytic volume: the ledgered "big domains at fine voxels
	// become fast previews" capability, re-proven every run.
	let fine = 0.09f32;
	let dense_err = dx.extract(domain, Resolution::VoxelSize(fine)).expect_err("dense must refuse over its cap");
	let t0 = Instant::now();
	let (fine_nb, fine_stats) = nbx.extract_with_stats(domain, Resolution::VoxelSize(fine), 0.0).expect("narrow-band extraction over dense cap");
	let t_fine = t0.elapsed();
	let fine_v = fine_nb.signed_volume();
	let want = 4.0 / 3.0 * std::f64::consts::PI * 27_000.0;
	assert!(
		matches!(dense_err, GpuError::TooLarge { .. })
			&& fine_nb.is_watertight()
			&& (fine_v - want).abs() / want < 0.01,
		"beyond dense cap @ {fine}: dense refused with {dense_err}; nb watertight={} vol={fine_v:.1} \
		 (analytic {want:.1}), active {}/{} blocks, {} samples in {t_fine:?}",
		fine_nb.is_watertight(),
		fine_stats.active_blocks,
		fine_stats.total_blocks,
		fine_stats.samples_evaluated,
	);
	println!(
		"nb beyond-cap @ {fine}: {} tris, vol {fine_v:.1} vs analytic {want:.1}, active {}/{} blocks, {} samples, {t_fine:?}",
		fine_nb.triangle_count(),
		fine_stats.active_blocks,
		fine_stats.total_blocks,
		fine_stats.samples_evaluated
	);
}

#[test]
fn narrow_band_keeps_two_disjoint_shells() {
	let Some(ctx) = gpu_or_skip("narrow_band_keeps_two_disjoint_shells") else { return };
	// GATE 3: two disjoint spheres — the band must find and close BOTH
	// components (block flags are per-block field tests, so disconnected
	// surface cannot hide the way a seed-and-flood scheme could drop it).
	let tree = GpuNode::sphere(Vec3::new(-15.0, 0.0, 0.0), 6.0).union(GpuNode::sphere(Vec3::new(15.0, 0.0, 0.0), 6.0));
	let domain = tree.bounds().pad(0.5);
	let vs = 0.25f32;
	let (nb, stats) = extract_narrow_band_with_stats(&ctx, &tree, domain, Resolution::VoxelSize(vs), 0.0).expect("narrow-band extraction");
	let dense = gpu_surface_nets(&ctx, &tree, domain, Resolution::VoxelSize(vs)).expect("dense extraction");
	let r = check_mesh(&nb);
	let shells = shell_count(&nb);
	let (nbv, dv) = (nb.signed_volume(), dense.signed_volume());
	assert!(
		nb.is_watertight() && r.boundary_edges == 0 && r.non_manifold_edges == 0 && shells == 2 && (nbv - dv).abs() / dv.abs() < 1e-3,
		"two disjoint spheres: watertight={} bnd={} nme={} shells={shells} (want 2) vol nb={nbv:.2} dense={dv:.2}; {stats:?}",
		nb.is_watertight(),
		r.boundary_edges,
		r.non_manifold_edges,
	);
	println!(
		"nb two shells @ {vs}: shells={shells}, vol nb={nbv:.3} dense={dv:.3}, active {}/{} blocks",
		stats.active_blocks, stats.total_blocks
	);
}

#[test]
fn narrow_band_band_floor_is_a_documented_clamp_and_band_independent_above_it() {
	let Some(ctx) = gpu_or_skip("narrow_band_band_floor_is_a_documented_clamp_and_band_independent_above_it") else { return };
	// GATE 4: the band-safety negative control. A band below the safe floor
	// (0, negative, even NaN) is DOCUMENTED-CLAMPED up to 2 voxels — pinned
	// here as bit-identical output and identical work receipts vs the explicit
	// floor. And ABOVE the floor the band only adds work, never changes the
	// mesh: band = +inf refines every block (active == total) yet reproduces
	// the floor mesh bit for bit (prefix-sum ids are invocation-order- and
	// band-independent for the contributing cells).
	let tree = GpuNode::sphere(Vec3::ZERO, 5.0);
	let domain = tree.bounds();
	let vs = 0.5f32;
	let nbx = GpuNarrowBand::compile(&ctx, &tree).expect("nb compile");
	let floor_band = 2.0 * vs;
	let (m_floor, s_floor) = nbx.extract_with_stats(domain, Resolution::VoxelSize(vs), floor_band).expect("floor-band extraction");
	let (m_zero, s_zero) = nbx.extract_with_stats(domain, Resolution::VoxelSize(vs), 0.0).expect("zero-band extraction");
	let (m_neg, s_neg) = nbx.extract_with_stats(domain, Resolution::VoxelSize(vs), -3.0).expect("negative-band extraction");
	let (m_nan, s_nan) = nbx.extract_with_stats(domain, Resolution::VoxelSize(vs), f32::NAN).expect("nan-band extraction");
	let (m_inf, s_inf) = nbx.extract_with_stats(domain, Resolution::VoxelSize(vs), f32::INFINITY).expect("inf-band extraction");
	let bits = mesh_bits(&m_floor);
	assert!(
		m_floor.is_watertight()
			&& !m_floor.is_empty()
			&& s_zero == s_floor
			&& s_neg == s_floor
			&& s_nan == s_floor
			&& mesh_bits(&m_zero) == bits
			&& mesh_bits(&m_neg) == bits
			&& mesh_bits(&m_nan) == bits
			&& s_inf.active_blocks == s_inf.total_blocks
			&& mesh_bits(&m_inf) == bits,
		"band clamp: floor watertight={} ({} tris); band 0/-3/NaN must clamp to the 2-voxel floor \
		 (stats zero={s_zero:?} neg={s_neg:?} nan={s_nan:?} vs floor={s_floor:?}, bit-identical meshes) \
		 and band=inf must refine all blocks ({}/{}) yet produce the identical mesh",
		m_floor.is_watertight(),
		m_floor.triangle_count(),
		s_inf.active_blocks,
		s_inf.total_blocks,
	);
	println!(
		"nb band clamp: floor {} tris (active {}/{}); inf-band active {}/{}; all outputs bit-identical",
		m_floor.triangle_count(),
		s_floor.active_blocks,
		s_floor.total_blocks,
		s_inf.active_blocks,
		s_inf.total_blocks
	);
}

#[test]
fn narrow_band_gyroid_block_is_closed_and_matches_cpu_volume() {
	let Some(ctx) = gpu_or_skip("narrow_band_gyroid_block_is_closed_and_matches_cpu_volume") else { return };
	// GATE 5: the TPMS egg-crate — surface everywhere, sheets in almost every
	// block, the case where a naive band cracks. Same 40 mm gyroid ∩ cube
	// workload the dense suite pins (Gyroid divides by √3·scale exactly so the
	// field stays ≤ 1-Lipschitz — the property the coarse flag test relies
	// on). Same honest assertion as the dense suite: a CLOSED surface (zero
	// boundary edges) at the CPU volume; non-manifold edges are the known
	// one-vertex-per-cell dual caveat and reported informationally.
	let half = 20.0f32;
	let region = Aabb::from_center_half_extent(Vec3::ZERO, Vec3::splat(half));
	let tree = GpuNode::gyroid(region, 0.35, 0.6).intersection(GpuNode::cuboid(Vec3::ZERO, Vec3::splat(half)));
	let vs = 0.4f32;
	let (nb, stats) = extract_narrow_band_with_stats(&ctx, &tree, region, Resolution::VoxelSize(vs), 0.0).expect("narrow-band extraction");
	let dense = gpu_surface_nets(&ctx, &tree, region, Resolution::VoxelSize(vs)).expect("dense extraction");
	let cpu = surface_nets(&tree.to_node(), region, Resolution::VoxelSize(vs));
	let r = check_mesh(&nb);
	let (nbv, dv, cv) = (nb.signed_volume(), dense.signed_volume(), cpu.signed_volume());
	let ratio = stats.active_blocks as f64 / stats.total_blocks as f64;
	assert!(
		r.boundary_edges == 0
			&& nb.triangle_count() > 100_000
			&& (nbv - dv).abs() / dv.abs() < 1e-3
			&& (nbv - cv).abs() / cv.abs() < 2e-3,
		"nb gyroid block: bnd={} tris={} vol nb={nbv:.1} dense={dv:.1} cpu={cv:.1} \
		 (|nb-dense|/dense={:.2e}, |nb-cpu|/cpu={:.2e}); active {}/{} blocks (nme={} — informational)",
		r.boundary_edges,
		nb.triangle_count(),
		(nbv - dv).abs() / dv.abs(),
		(nbv - cv).abs() / cv.abs(),
		stats.active_blocks,
		stats.total_blocks,
		r.non_manifold_edges,
	);
	println!(
		"nb gyroid 40mm @ {vs}: {} tris, vol nb={nbv:.1} dense={dv:.1} cpu={cv:.1}, \
		 active {}/{} blocks (ratio {ratio:.3} — egg-crate, honestly dense-ish), nme={}",
		nb.triangle_count(),
		stats.active_blocks,
		stats.total_blocks,
		r.non_manifold_edges
	);
}
