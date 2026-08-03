// Copyright (c) LMCAD. Licensed under the MIT License.

//! Kernel performance baseline — five representative workloads spanning both halves
//! of the hybrid kernel, timed with `std::time::Instant` (median of 3 runs, no
//! dependencies). The published numbers live in `BENCH.md` at the repo root; re-run
//! this example to refresh them:
//!
//! ```text
//! cargo run --example bench_kernel -p kernel-model --release
//! ```
//!
//! The five workloads:
//! 1. **boolean_union_cyl_box** — exact planar boolean of a 64-segment cylinder with
//!    an overlapping cuboid (the arrangement/stitch pipeline).
//! 2. **adaptive_tessellation** — `tessellate_adaptive_tol` of a 64-segment filleted
//!    cylinder at 10 µm chord tolerance (the exact analytic meshing path).
//! 3. **gyroid_mdc** — Manifold Dual Contouring of the watertight gyroid lattice
//!    (params of kernel-implicit's watertight-lattice test: 40 mm cube, 0.6 shell,
//!    0.8 voxel — the implicit half's signature workload).
//! 4. **hybrid_heal_hex_nut** — `watertight_mesh` of a hex nut at 0.5 voxel (B-rep →
//!    winding-number SDF → MDC re-mesh, the hybrid's core move).
//! 5. **flange_extrude_7_holes** — `extrude_with_holes` of a 96-gon flange with a
//!    centre bore and a 6-hole bolt circle, plus `validate` (multi-loop construction
//!    + the validity oracle).
//!
//! Each row prints a sanity column (sizes, watertightness, validity) so the timings
//! are demonstrably measuring real completed work, not dead code.
//!
//! A separate **scale** section (opt-in: pass `--scale`; run once each, not
//! median-of-3 — these are capacity evidence, not interactive-latency rows, and
//! the TPMS case runs ~2.5 minutes within ~11 GB) follows the table:
//!
//! ```text
//! cargo run --example bench_kernel -p kernel-model --release -- --scale
//! ```
//!
//! - **tess_102k_face_revolve** — `tessellate_adaptive_tol` of a 102 400-face
//!   B-rep (200-point corrugated ring profile revolved in 512 sectors), the
//!   Level-9 "interactive rebuilds on 100k-face models" probe.
//! - **gyroid_200mm_narrowband** — narrow-band Surface Nets of a 200 mm TPMS
//!   shell at sub-voxel-cap resolution: the conceptual lattice is *billions* of
//!   cells — far beyond the dense meshers' 2²⁸ allocation cap — and the
//!   surface-tracking march visits only the active band (count printed).

use std::f64::consts::TAU;
use std::time::Instant;

use kernel_brep::math::{DVec2, DVec3};
use kernel_brep::{
	cuboid, cylinder, extrude_with_holes, filleted_cylinder, revolve, tessellate_adaptive_tol, union, validate,
};
use kernel_implicit::narrow_band::surface_nets_narrowband_with_visited;
use kernel_implicit::{manifold_dual_contour, Aabb, Cuboid as VoxCuboid, Gyroid, Node, Resolution, Vec3};
use kernel_model::parts::hex_nut;
use kernel_model::watertight_mesh;

/// A circle sampled as an `n`-gon centred at `(cx, cy)` — the profile vocabulary of
/// the extrusion builders.
fn circle(r: f64, n: usize, cx: f64, cy: f64) -> Vec<DVec2> {
	(0..n)
		.map(|i| {
			let a = i as f64 * TAU / n as f64;
			DVec2::new(cx + r * a.cos(), cy + r * a.sin())
		})
		.collect()
}

/// Run `f` three times, returning (last result, the three wall times in ms, median ms).
/// `black_box` keeps the optimizer from eliding the work.
fn bench<T>(mut f: impl FnMut() -> T) -> (T, [f64; 3], f64) {
	let mut times = [0.0_f64; 3];
	let mut out = None;
	for t in &mut times {
		let start = Instant::now();
		let r = std::hint::black_box(f());
		*t = start.elapsed().as_secs_f64() * 1e3;
		out = Some(r);
	}
	let mut sorted = times;
	sorted.sort_by(f64::total_cmp);
	(out.expect("three runs always produce a result"), times, sorted[1])
}

fn main() {
	let mut rows: Vec<(&str, f64, [f64; 3], String)> = Vec::new();

	// (a) Exact planar boolean: 64-seg cylinder ∪ overlapping cuboid.
	{
		let cyl = cylinder(DVec3::new(0.0, 0.0, -20.0), DVec3::Z, 15.0, 40.0, 64);
		let box_ = cuboid(DVec3::new(5.0, -15.0, -15.0), DVec3::new(35.0, 15.0, 15.0));
		let (solid, runs, med) = bench(|| union(&cyl, &box_));
		let v = validate(&solid);
		rows.push((
			"boolean_union_cyl_box",
			med,
			runs,
			format!("{} faces, valid={}", solid.face_count(), v.is_valid()),
		));
	}

	// (b) Adaptive exact tessellation of a filleted cylinder at 10 µm.
	{
		let solid = filleted_cylinder(20.0, 40.0, 3.0, 64, 16);
		let (mesh, runs, med) = bench(|| tessellate_adaptive_tol(&solid, 0.01));
		rows.push((
			"adaptive_tessellation",
			med,
			runs,
			format!("{} tris, watertight={}", mesh.triangle_count(), mesh.is_watertight()),
		));
	}

	// (c) Manifold Dual Contouring of the watertight gyroid lattice (40 mm cube,
	// 0.6 shell, 0.8 voxel — the kernel-implicit watertight-lattice test's params).
	{
		let half = 20.0;
		let region = Aabb::from_center_half_extent(Vec3::ZERO, Vec3::splat(half));
		let lattice = Node::primitive(Gyroid::new(region, 0.35, 0.6))
			.intersection(Node::primitive(VoxCuboid::new(Vec3::ZERO, Vec3::splat(half))));
		let (mesh, runs, med) = bench(|| manifold_dual_contour(&lattice, region, Resolution::VoxelSize(0.8)));
		rows.push((
			"gyroid_mdc",
			med,
			runs,
			format!("{} tris, watertight={}", mesh.triangle_count(), mesh.is_watertight()),
		));
	}

	// (d) Hybrid heal: hex nut B-rep → winding-number SDF → MDC, 0.5 voxel.
	{
		let nut = hex_nut(16.0, 8.0, 10.0);
		let (mesh, runs, med) = bench(|| watertight_mesh(&nut, 0.5));
		rows.push((
			"hybrid_heal_hex_nut",
			med,
			runs,
			format!("{} tris, watertight={}", mesh.triangle_count(), mesh.is_watertight()),
		));
	}

	// (e) Multi-loop flange: 96-gon outer r40, centre bore 48-gon r10, six 24-gon r3
	// holes on a R30 bolt circle, h=8 — construction plus the validity oracle.
	{
		let outer = circle(40.0, 96, 0.0, 0.0);
		let mut holes = vec![circle(10.0, 48, 0.0, 0.0)];
		for k in 0..6 {
			let a = k as f64 * TAU / 6.0;
			holes.push(circle(3.0, 24, 30.0 * a.cos(), 30.0 * a.sin()));
		}
		let (out, runs, med) = bench(|| {
			let solid = extrude_with_holes(&outer, &holes, 8.0);
			let v = validate(&solid);
			(solid, v)
		});
		let (solid, v) = out;
		rows.push((
			"flange_extrude_7_holes",
			med,
			runs,
			format!("{} faces, genus={}, valid={}", solid.face_count(), v.genus, v.is_valid()),
		));
	}

	println!("=== kernel performance baseline (median of 3, release) ===");
	println!("| bench | median (ms) | runs (ms) | sanity |");
	println!("|---|---|---|---|");
	for (name, med, runs, sanity) in &rows {
		println!(
			"| {name} | {med:.1} | {:.1} / {:.1} / {:.1} | {sanity} |",
			runs[0], runs[1], runs[2]
		);
	}

	if !std::env::args().any(|a| a == "--scale") {
		println!("\n(scale evidence skipped — pass `--scale` to run the 102k-face and 200 mm narrow-band cases)");
		return;
	}
	println!("\n=== scale evidence (run once each) ===");

	// (f) 102k-face B-rep, adaptive-tessellated: a corrugated ring (outer wall
	// rippled by 8 sine periods, straight bore) revolved in 512 sectors → 200
	// profile edges × 512 = 102 400 band faces, genus 1.
	{
		let mut profile: Vec<DVec2> = Vec::with_capacity(200);
		for i in 0..100 {
			let t = i as f64 / 99.0;
			profile.push(DVec2::new(40.0 + 2.0 * (t * 8.0 * TAU).sin(), t * 40.0));
		}
		for i in 0..100 {
			let t = i as f64 / 99.0;
			profile.push(DVec2::new(30.0, 40.0 - t * 40.0));
		}
		let t0 = Instant::now();
		let solid = revolve(&profile, 512);
		let t_build = t0.elapsed().as_secs_f64() * 1e3;
		let t0 = Instant::now();
		let mesh = std::hint::black_box(tessellate_adaptive_tol(&solid, 0.05));
		let t_tess = t0.elapsed().as_secs_f64() * 1e3;
		println!(
			"tess_102k_face_revolve: build {t_build:.0} ms, adaptive tess (50 µm) {t_tess:.0} ms — {} faces → {} tris, watertight={}",
			solid.face_count(),
			mesh.triangle_count(),
			mesh.is_watertight()
		);
	}

	// (g) 200 mm TPMS shell, narrow-band Surface Nets at 0.1 mm voxel. The
	// CONCEPTUAL lattice is (200/0.1 + 3)³ ≈ 8.0e9 cells — ~30× beyond the dense
	// meshers' 2²⁸ cap (the narrow-band path indexes up to 2⁴⁴) — while the
	// surface-tracking march touches only the printed active-band count. The
	// Gyroid field is 1-Lipschitz by construction (√3-normalized), as the
	// block-scan seeding requires.
	{
		let half = 100.0;
		let vs = 0.1f32;
		let region = Aabb::from_center_half_extent(Vec3::ZERO, Vec3::splat(half));
		let part = Node::primitive(Gyroid::new(region, 0.02, 1.5))
			.intersection(Node::primitive(VoxCuboid::new(Vec3::ZERO, Vec3::splat(half))));
		let dims = (2.0 * half / vs).ceil() as f64 + 3.0;
		let t0 = Instant::now();
		let (mesh, visited) = std::hint::black_box(surface_nets_narrowband_with_visited(
			&part,
			region,
			Resolution::VoxelSize(vs),
		));
		let t_march = t0.elapsed().as_secs_f64() * 1e3;
		let t0 = Instant::now();
		let wt = mesh.is_watertight();
		let t_wt = t0.elapsed().as_secs_f64() * 1e3;
		println!(
			"gyroid_200mm_narrowband: march {t_march:.0} ms — conceptual {:.2e} cells, visited {visited} ({:.2}%), {} tris, watertight={wt} (check {t_wt:.0} ms)",
			dims * dims * dims,
			100.0 * visited as f64 / (dims * dims * dims),
			mesh.triangle_count()
		);
	}
}
