// Copyright (c) LMCAD. Licensed under the MIT License.

//! GPU baseline benchmark — the numbers behind BENCH.md's "GPU (kernel-gpu)"
//! section.
//!
//! Workloads:
//! 1. **Field evaluation throughput** (Mcells/s): the gyroid lattice tree and
//!    the rocket-style jacket truss (320 struts, brute-force min on the GPU vs
//!    the grid-accelerated CPU path), sampled on dense point grids. GPU times
//!    are END-TO-END (point upload + dispatch + readback) — stated cold
//!    (first dispatch) and warm (second dispatch, same buffers re-created;
//!    the difference is mostly Metal shader warm-up).
//! 2. **Surface-nets extraction, gyroid 40 mm @ 0.15 mm voxel**: GPU pipeline
//!    (corner sampling + classify + prefix-sum compaction + emit + readback)
//!    vs the CPU `surface_nets` on the identical domain/lattice — same
//!    algorithm, directly comparable output (volumes/counts printed).
//!
//! Run: `cargo run --example bench_gpu -p kernel-gpu --release`
//! Exits 0 with a LOUD message when no GPU adapter is present.

use std::time::Instant;

use kernel_core::math::{Aabb, Vec3};
use kernel_core::sdf::Sdf;
use kernel_core::{check_mesh, surface_nets, Resolution};
use kernel_gpu::{GpuContext, GpuField, GpuNode, GpuSurfaceNets};
use rayon::prelude::*;

/// The signature TPMS workload (kernel-implicit's watertight gyroid config).
fn gyroid_tree(half: f32) -> GpuNode {
	let region = Aabb::from_center_half_extent(Vec3::ZERO, Vec3::splat(half));
	GpuNode::gyroid(region, 0.35, 0.6).intersection(GpuNode::cuboid(Vec3::ZERO, Vec3::splat(half)))
}

/// The rocket_demo-style jacket truss (6 rings x 20 spokes, tapered pins +
/// X-braces = 320 struts) — the lattice brute-force-vs-grid comparison.
fn jacket_truss() -> GpuNode {
	let r_inner = |z: f32| 14.0 + 3.0 * ((z - 25.0) / 12.0).tanh();
	let rings = [15.0f32, 19.0, 23.0, 27.0, 31.0, 35.0];
	let spokes = 20u32;
	let mut nodes = Vec::new();
	let mut struts = Vec::new();
	for (i, &z) in rings.iter().enumerate() {
		for j in 0..spokes {
			let a = (j as f32 + 0.5 * (i % 2) as f32) * std::f32::consts::TAU / spokes as f32;
			let dir = Vec3::new(a.cos(), a.sin(), 0.0);
			nodes.push(dir * (r_inner(z) + 3.2) + Vec3::new(0.0, 0.0, z));
			nodes.push(dir * (r_inner(z) + 9.8) + Vec3::new(0.0, 0.0, z));
			let (wa, sa) = ((nodes.len() - 2) as u32, (nodes.len() - 1) as u32);
			struts.push((wa, sa, 1.2, 0.9));
			if i > 0 {
				struts.push((wa, sa - 2 * spokes, 0.8, 0.8));
				struts.push((wa - 2 * spokes, sa, 0.8, 0.8));
			}
		}
	}
	GpuNode::lattice(nodes, struts)
}

/// A dense `n^3` point grid over `bounds`.
fn point_grid(n: usize, bounds: Aabb) -> Vec<Vec3> {
	let size = bounds.size();
	let step = size / (n as f32 - 1.0);
	let mut pts = Vec::with_capacity(n * n * n);
	for k in 0..n {
		for j in 0..n {
			for i in 0..n {
				pts.push(bounds.min + Vec3::new(i as f32, j as f32, k as f32) * step);
			}
		}
	}
	pts
}

fn mcells(n: usize, secs: f64) -> f64 {
	n as f64 / secs / 1e6
}

fn bench_field_eval(ctx: &GpuContext, name: &str, tree: &GpuNode, grid_n: usize) {
	let pts = point_grid(grid_n, tree.bounds().pad(1.0));
	let n = pts.len();

	let t0 = Instant::now();
	let field = GpuField::compile(ctx, tree).expect("compile");
	let compile_s = t0.elapsed().as_secs_f64();

	let t0 = Instant::now();
	let gpu_cold = field.eval(&pts);
	let cold_s = t0.elapsed().as_secs_f64();
	let t0 = Instant::now();
	let gpu_warm = field.eval(&pts);
	let warm_s = t0.elapsed().as_secs_f64();

	let node = tree.to_node();
	let t0 = Instant::now();
	let cpu: Vec<f32> = pts.par_iter().map(|&p| node.distance(p)).collect();
	let cpu_s = t0.elapsed().as_secs_f64();

	// Honest cross-check while we're here: the bench inputs satisfy the
	// declared parity tolerance (and warm == cold bitwise — determinism).
	let mut max_rel = 0.0f32;
	for i in 0..n {
		max_rel = max_rel.max((gpu_cold[i] - cpu[i]).abs() / (1.0 + cpu[i].abs()));
	}
	assert!(max_rel < 1e-4, "{name}: bench probes exceed the parity tolerance: {max_rel:.3e}");
	assert!(gpu_cold.iter().zip(&gpu_warm).all(|(a, b)| a.to_bits() == b.to_bits()), "{name}: GPU eval must be run-to-run deterministic");

	println!(
		"| field eval: {name} | {n} pts | compile {:.0} ms | GPU cold {:.0} ms ({:.0} Mcells/s) | GPU warm {:.0} ms ({:.0} Mcells/s) | CPU rayon {:.0} ms ({:.1} Mcells/s) | warm speedup {:.1}x | max scaled err {max_rel:.1e} |",
		compile_s * 1e3,
		cold_s * 1e3,
		mcells(n, cold_s),
		warm_s * 1e3,
		mcells(n, warm_s),
		cpu_s * 1e3,
		mcells(n, cpu_s),
		cpu_s / warm_s
	);
}

fn bench_extraction(ctx: &GpuContext, half: f32, voxel: f32) {
	let tree = gyroid_tree(half);
	let region = Aabb::from_center_half_extent(Vec3::ZERO, Vec3::splat(half));

	let t0 = Instant::now();
	let extractor = GpuSurfaceNets::compile(ctx, &tree).expect("compile");
	let compile_s = t0.elapsed().as_secs_f64();
	let t0 = Instant::now();
	let gpu_mesh = extractor.extract(region, Resolution::VoxelSize(voxel)).expect("gpu extract");
	let gpu_cold_s = t0.elapsed().as_secs_f64();
	let t0 = Instant::now();
	let gpu_mesh2 = extractor.extract(region, Resolution::VoxelSize(voxel)).expect("gpu extract warm");
	let gpu_warm_s = t0.elapsed().as_secs_f64();

	let node = tree.to_node();
	let t0 = Instant::now();
	let cpu_mesh = surface_nets(&node, region, Resolution::VoxelSize(voxel));
	let cpu_s = t0.elapsed().as_secs_f64();

	let rg = check_mesh(&gpu_mesh);
	let rc = check_mesh(&cpu_mesh);
	let (vg, vc) = (gpu_mesh.signed_volume(), cpu_mesh.signed_volume());
	println!(
		"| surface nets: gyroid {:.0} mm @ {voxel} | GPU compile {:.0} ms, extract cold {:.0} ms / warm {:.0} ms | CPU {:.0} ms | warm speedup {:.1}x | GPU {} tris bnd={} nme={} vol {vg:.1} | CPU {} tris bnd={} nme={} vol {vc:.1} | dVol {:.2e} |",
		2.0 * half,
		compile_s * 1e3,
		gpu_cold_s * 1e3,
		gpu_warm_s * 1e3,
		cpu_s * 1e3,
		cpu_s / gpu_warm_s,
		gpu_mesh.triangle_count(),
		rg.boundary_edges,
		rg.non_manifold_edges,
		cpu_mesh.triangle_count(),
		rc.boundary_edges,
		rc.non_manifold_edges,
		(vg - vc).abs() / vc.abs().max(1.0)
	);
	assert_eq!(gpu_mesh2.triangle_count(), gpu_mesh.triangle_count(), "warm extraction must reproduce");
}

fn main() {
	let ctx = match GpuContext::new() {
		Ok(c) => c,
		Err(e) => {
			eprintln!("==================================================================");
			eprintln!("bench_gpu: NO GPU ADAPTER — nothing measured ({e}).");
			eprintln!("==================================================================");
			return;
		}
	};
	let info = &ctx.adapter_info;
	println!("# kernel-gpu bench — adapter: {} ({:?}), backend {:?}", info.name, info.device_type, info.backend);
	println!("# threads: {} (rayon), release profile", rayon::current_num_threads());

	// Field-eval throughput (256^3 = 16.78M points for the gyroid; 128^3 =
	// 2.1M for the strut-loop lattice, whose GPU path is brute-force min).
	bench_field_eval(&ctx, "gyroid lattice tree", &gyroid_tree(20.0), 256);
	bench_field_eval(&ctx, "jacket truss (320 struts, GPU brute-force)", &jacket_truss(), 128);

	// Extraction: the BENCH.md row — gyroid 40 mm @ 0.15.
	bench_extraction(&ctx, 20.0, 0.15);
}
