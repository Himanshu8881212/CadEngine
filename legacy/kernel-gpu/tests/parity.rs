// Copyright (c) LMCAD. Licensed under the MIT License.

//! CPU↔GPU field parity: every probe must satisfy the declared GPU tolerance
//! `|gpu − cpu_f32| ≤ 1e-4 · (1 + |cpu_f32|)` (NUMERICS.md "GPU evaluation").
//! The CPU oracle is `GpuNode::to_node()` — the bit-authoritative tree the CPU
//! meshers consume.
//!
//! RUNTIME SKIP: every test here needs a GPU adapter. Without one (headless
//! CI) the tests print a LOUD skip message and pass vacuously — they verify
//! nothing in that environment. The full list of runtime-skipping tests is
//! documented in NUMERICS.md.

use std::f32::consts::TAU;
use std::sync::Arc;

use kernel_core::math::{Aabb, Quat, Vec3};
use kernel_core::sdf::Sdf;
use kernel_gpu::{GpuContext, GpuField, GpuNode};
use kernel_implicit::expr_sdf::Expr;
use kernel_implicit::grid::VoxelGrid;
use kernel_implicit::primitives::Sphere;

/// Acquire a GPU or skip LOUDLY (see module docs).
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

/// Deterministic LCG probe points in `bounds` padded by `pad`.
fn lcg_points(n: usize, bounds: Aabb, pad: f32, seed: &mut u64) -> Vec<Vec3> {
	fn next(s: &mut u64) -> f32 {
		*s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
		((*s >> 33) as f32) / ((1u64 << 31) as f32)
	}
	let (lo, hi) = (bounds.min - Vec3::splat(pad), bounds.max + Vec3::splat(pad));
	(0..n)
		.map(|_| {
			let t = Vec3::new(next(seed), next(seed), next(seed));
			lo + (hi - lo) * t
		})
		.collect()
}

/// Evaluate `tree` on CPU (authoritative) and GPU at `probes`; assert the
/// declared tolerance with one snapshot-style report of the worst offenders.
fn assert_parity(ctx: &GpuContext, name: &str, tree: &GpuNode, probes: &[Vec3]) {
	let field = GpuField::compile(ctx, tree).unwrap_or_else(|e| panic!("{name}: GPU compile failed: {e}"));
	let gpu = field.eval(probes);
	let node = tree.to_node();
	let mut worst: Vec<(f32, Vec3, f32, f32)> = Vec::new(); // (rel, p, cpu, gpu)
	let mut max_rel = 0.0f32;
	for (i, &p) in probes.iter().enumerate() {
		let cpu = node.distance(p);
		let err = (gpu[i] - cpu).abs();
		let rel = err / (1.0 + cpu.abs());
		max_rel = max_rel.max(rel);
		if rel >= 1e-4 {
			worst.push((rel, p, cpu, gpu[i]));
		}
	}
	worst.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
	worst.truncate(5);
	assert!(
		worst.is_empty(),
		"{name}: {} of {} probes exceed |gpu-cpu| < 1e-4*(1+|d|); worst: {:?}",
		worst.len(),
		probes.len(),
		worst
	);
	println!("{name}: {} probes, max scaled error {max_rel:.3e} (tolerance 1e-4)", probes.len());
}

/// An explicit strut graph covering every cone-capsule branch: a tapered
/// strut, an equal-radius (capsule) strut, and a degenerate strut whose hull
/// collapses to the larger end sphere.
fn branchy_lattice() -> GpuNode {
	let nodes = vec![
		Vec3::new(-6.0, 0.0, 0.0),
		Vec3::new(6.0, 1.0, 2.0),
		Vec3::new(0.0, 7.0, -1.0),
		Vec3::new(0.5, 7.2, -1.1), // nearly coincident with node 2 (degenerate hull)
	];
	let struts = vec![
		(0, 1, 2.0, 0.8), // tapered
		(1, 2, 1.2, 1.2), // capsule
		(2, 3, 3.0, 0.5), // one end sphere contains the other
	];
	GpuNode::lattice(nodes, struts)
}

/// A manually sampled helical tube (the GPU tree and the CPU `Pipe` get the
/// SAME polyline, so this tests strut-buffer parity, not helix construction).
fn helix_pipe(center: Vec3, r_helix: f32, pitch: f32, turns: f32, n: usize, radius: f32) -> (Vec<Vec3>, Vec<f32>) {
	let path: Vec<Vec3> = (0..=n)
		.map(|i| {
			let t = turns * i as f32 / n as f32;
			let a = t * TAU;
			center + Vec3::new(r_helix * a.cos(), r_helix * a.sin(), pitch * t)
		})
		.collect();
	let radii = vec![radius; path.len()];
	(path, radii)
}

#[test]
fn parity_each_primitive_leaf() {
	let Some(ctx) = gpu_or_skip("parity_each_primitive_leaf") else { return };
	let mut seed = 0x5eed_0001_u64;

	let grid_src = VoxelGrid::from_sdf(&Sphere::new(Vec3::ZERO, 6.0), Aabb::from_center_half_extent(Vec3::ZERO, Vec3::splat(7.0)), 0.5);
	let (hpath, hradii) = helix_pipe(Vec3::ZERO, 8.0, 6.0, 2.0, 96, 1.5);
	let sphere_expr = Arc::new(Expr::Sub(
		Box::new(Expr::Length3(Box::new(Expr::X), Box::new(Expr::Y), Box::new(Expr::Z))),
		Box::new(Expr::Const(8.0)),
	));

	// The 12 lowerable primitive leaves, each probed inside, near and far.
	let leaves: Vec<(&str, GpuNode, Aabb)> = vec![
		("sphere", GpuNode::sphere(Vec3::new(1.0, -2.0, 0.5), 7.0), Aabb::from_center_half_extent(Vec3::new(1.0, -2.0, 0.5), Vec3::splat(7.0))),
		("cuboid", GpuNode::cuboid(Vec3::new(0.5, 0.0, -1.0), Vec3::new(5.0, 3.0, 8.0)), Aabb::from_center_half_extent(Vec3::new(0.5, 0.0, -1.0), Vec3::new(5.0, 3.0, 8.0))),
		("cylinder", GpuNode::cylinder(Vec3::new(-4.0, 0.0, 1.0), Vec3::new(5.0, 2.0, 3.0), 2.5), Aabb::from_points(&[Vec3::new(-4.0, 0.0, 1.0), Vec3::new(5.0, 2.0, 3.0)]).pad(2.5)),
		("cone", GpuNode::cone(Vec3::new(0.0, 0.0, -5.0), Vec3::new(1.0, 0.5, 6.0), 4.0, 1.5), Aabb::from_points(&[Vec3::new(0.0, 0.0, -5.0), Vec3::new(1.0, 0.5, 6.0)]).pad(4.0)),
		("plane", GpuNode::plane(Vec3::new(0.5, 0.0, 1.0), Vec3::new(1.0, 2.0, 0.5)), Aabb::from_center_half_extent(Vec3::ZERO, Vec3::splat(20.0))),
		("torus", GpuNode::torus(Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.3, 0.4, 1.0), 8.0, 2.0), Aabb::from_center_half_extent(Vec3::new(1.0, 0.0, 0.0), Vec3::splat(10.0))),
		("capsule", GpuNode::capsule(Vec3::new(-5.0, 1.0, 0.0), Vec3::new(5.0, -1.0, 2.0), 2.0), Aabb::from_points(&[Vec3::new(-5.0, 1.0, 0.0), Vec3::new(5.0, -1.0, 2.0)]).pad(2.0)),
		("gyroid", GpuNode::gyroid(Aabb::from_center_half_extent(Vec3::ZERO, Vec3::splat(10.0)), 0.4, 0.3), Aabb::from_center_half_extent(Vec3::ZERO, Vec3::splat(10.0))),
		("beam_lattice", branchy_lattice(), Aabb::from_center_half_extent(Vec3::ZERO, Vec3::splat(10.0))),
		("pipe", GpuNode::pipe(hpath, hradii), Aabb::from_center_half_extent(Vec3::new(0.0, 0.0, 6.0), Vec3::splat(12.0))),
		("voxel_grid", GpuNode::grid(Arc::new(grid_src)), Aabb::from_center_half_extent(Vec3::ZERO, Vec3::splat(8.0))),
		("expr_sdf", GpuNode::expr(sphere_expr, 1.0, Some(Aabb::from_center_half_extent(Vec3::ZERO, Vec3::splat(8.0)))), Aabb::from_center_half_extent(Vec3::ZERO, Vec3::splat(8.0))),
	];
	assert_eq!(leaves.len(), 12, "the 12 primitive leaves");
	for (name, tree, probe_box) in &leaves {
		// Near probes plus a far band (10x pad) to exercise cap/termination branches.
		let mut probes = lcg_points(160, *probe_box, 2.0, &mut seed);
		probes.extend(lcg_points(40, *probe_box, 25.0, &mut seed));
		assert_parity(&ctx, &format!("leaf::{name}"), tree, &probes);
	}
}

#[test]
fn parity_every_combinator_in_one_tree() {
	let Some(ctx) = gpu_or_skip("parity_every_combinator_in_one_tree") else { return };
	// One kitchen-sink tree through ALL 18 combinators (every Node variant +
	// the 4 feature seam operators), so a wrong WGSL mirror of any of them
	// fails this single falsifier.
	let s = |c: Vec3, r: f32| GpuNode::sphere(c, r);
	let base = s(Vec3::new(-3.0, 0.0, 0.0), 5.0)
		.union(GpuNode::cuboid(Vec3::new(3.0, 0.0, 0.0), Vec3::splat(4.0)))
		.intersection(GpuNode::cylinder(Vec3::new(0.0, 0.0, -8.0), Vec3::new(0.0, 0.0, 8.0), 7.0));
	let blends = GpuNode::cone(Vec3::new(0.0, -6.0, 0.0), Vec3::new(0.0, 6.0, 0.0), 3.0, 1.0)
		.smooth_union(GpuNode::capsule(Vec3::new(-4.0, -4.0, 0.0), Vec3::new(4.0, 4.0, 0.0), 1.5), 2.0)
		.smooth_intersection(s(Vec3::ZERO, 6.5), 1.5)
		.smooth_difference(GpuNode::torus(Vec3::ZERO, Vec3::Z, 4.0, 1.0), 1.0);
	let featured = base
		.fillet_union(blends, 1.2)
		.chamfer_union(GpuNode::cuboid(Vec3::new(0.0, 0.0, 6.0), Vec3::new(2.0, 2.0, 2.0)), 0.8)
		.fillet_difference(GpuNode::cylinder(Vec3::new(0.0, -9.0, 0.0), Vec3::new(0.0, 9.0, 0.0), 1.5), 0.6)
		.chamfer_difference(GpuNode::cylinder(Vec3::new(-9.0, 0.0, 0.0), Vec3::new(9.0, 0.0, 0.0), 1.0), 0.5);
	let shaped = featured
		.offset(0.3)
		.shell(2.5)
		.translate(Vec3::new(1.0, -0.5, 0.5))
		.rotate(Quat::from_axis_angle(Vec3::new(0.0, 1.0, 0.0).normalize(), 0.4));
	let patterned = shaped
		.linear_pattern(Vec3::new(18.0, 0.0, 0.0), 2)
		.circular_pattern(Vec3::new(9.0, 0.0, 0.0), Vec3::Z, TAU / 3.0, 3)
		.mirror(Vec3::new(0.0, -14.0, 0.0), Vec3::Y);
	let ramp = Arc::new(Expr::Mul(Box::new(Expr::Const(0.02)), Box::new(Expr::Z)));
	let weight = Arc::new(Expr::Mul(Box::new(Expr::Add(Box::new(Expr::Z), Box::new(Expr::Const(10.0)))), Box::new(Expr::Const(0.05))));
	let tree = patterned.offset_by(ramp, 1.0).lerp(s(Vec3::ZERO, 12.0), weight);

	let mut seed = 0x5eed_0002_u64;
	let probes = lcg_points(200, tree.bounds(), 3.0, &mut seed);
	assert_parity(&ctx, "all_18_combinators", &tree, &probes);
}

#[test]
fn parity_smooth_blend_chain() {
	let Some(ctx) = gpu_or_skip("parity_smooth_blend_chain") else { return };
	// The organic-modelling composite: a metaball-style chain of smooth unions
	// with a smooth difference bite and a zero-radius fillet (the r <= 0 seam
	// fallback branch must also mirror).
	let mut tree = GpuNode::sphere(Vec3::new(-6.0, 0.0, 0.0), 4.0);
	for (i, c) in [Vec3::new(-2.0, 1.5, 0.5), Vec3::new(2.0, -1.0, -0.5), Vec3::new(6.0, 0.5, 0.2)].iter().enumerate() {
		tree = tree.smooth_union(GpuNode::sphere(*c, 3.5 - 0.4 * i as f32), 2.0);
	}
	let tree = tree
		.smooth_difference(GpuNode::cylinder(Vec3::new(0.0, 0.0, -6.0), Vec3::new(0.0, 0.0, 6.0), 1.8), 1.2)
		.fillet_union(GpuNode::capsule(Vec3::new(-8.0, -3.0, 0.0), Vec3::new(8.0, -3.0, 0.0), 1.0), 0.0);
	let mut seed = 0x5eed_0003_u64;
	let probes = lcg_points(220, tree.bounds(), 2.0, &mut seed);
	assert_parity(&ctx, "smooth_blend_chain", &tree, &probes);
}

#[test]
fn parity_rocket_style_jacket_lattice() {
	let Some(ctx) = gpu_or_skip("parity_rocket_style_jacket_lattice") else { return };
	// The rocket_demo jacket workload shape: 6 rings x 20 spokes of tapered
	// radial pins + X-braces (320 struts), conformal to a smooth r(z) profile,
	// unioned with a helical pipe and welded to a shell with a fillet — the
	// W3 lattice workhorses in one composite.
	let r_inner = |z: f32| 14.0 + 3.0 * ((z - 25.0) / 12.0).tanh();
	let rings = [15.0f32, 19.0, 23.0, 27.0, 31.0, 35.0];
	let spokes = 20u32;
	let mut nodes = Vec::new();
	let mut struts = Vec::new();
	for (i, &z) in rings.iter().enumerate() {
		for j in 0..spokes {
			let a = (j as f32 + 0.5 * (i % 2) as f32) * TAU / spokes as f32;
			let dir = Vec3::new(a.cos(), a.sin(), 0.0);
			nodes.push(dir * (r_inner(z) + 3.2) + Vec3::new(0.0, 0.0, z));
			nodes.push(dir * (r_inner(z) + 9.8) + Vec3::new(0.0, 0.0, z));
			let (wa, sa) = ((nodes.len() - 2) as u32, (nodes.len() - 1) as u32);
			struts.push((wa, sa, 1.2, 0.9));
			if i > 0 {
				let (wb, sb) = (wa - 2 * spokes, sa - 2 * spokes);
				struts.push((wa, sb, 0.8, 0.8));
				struts.push((wb, sa, 0.8, 0.8));
			}
		}
	}
	assert_eq!(struts.len(), 320, "jacket truss strut count (6x20 pins + 5x20x2 braces)");
	let truss = GpuNode::lattice(nodes, struts);
	let (hpath, hradii) = helix_pipe(Vec3::new(0.0, 0.0, 12.0), r_inner(12.0) + 6.0, 8.0, 3.0, 192, 1.4);
	let shell = GpuNode::cylinder(Vec3::new(0.0, 0.0, 13.0), Vec3::new(0.0, 0.0, 37.0), 26.0)
		.difference(GpuNode::cylinder(Vec3::new(0.0, 0.0, 12.0), Vec3::new(0.0, 0.0, 38.0), 24.0));
	let tree = truss.union(GpuNode::pipe(hpath, hradii)).fillet_union(shell, 1.2);

	let mut seed = 0x5eed_0004_u64;
	let mut probes = lcg_points(300, tree.bounds(), 2.0, &mut seed);
	probes.extend(lcg_points(60, tree.bounds(), 30.0, &mut seed));
	assert_parity(&ctx, "rocket_style_jacket_lattice", &tree, &probes);
}

#[test]
fn parity_helical_thread_expr_field() {
	let Some(ctx) = gpu_or_skip("parity_helical_thread_expr_field") else { return };
	// A sinusoidal screw-thread field as pure Expr data: r - r0 + a*sin(z*k +
	// theta) (single start, so the atan2 branch cut cancels inside the
	// periodic sin). Composed with a bounding cylinder and a graded offset —
	// the AI-facing "custom field" workflow end to end.
	let b = |e: Expr| Box::new(e);
	let theta = Expr::Atan2 { y: b(Expr::Y), x: b(Expr::X) };
	let phase = Expr::Add(b(Expr::Mul(b(Expr::Z), b(Expr::Const(TAU as f64 / 4.0)))), b(theta));
	let thread = Expr::Sub(
		b(Expr::Length2(b(Expr::X), b(Expr::Y))),
		b(Expr::Add(b(Expr::Const(9.0)), b(Expr::Mul(b(Expr::Const(0.8)), b(Expr::Sin(b(phase))))))),
	);
	// |grad| <= 1 + 0.8*sqrt(k^2 + (1/r)^2) ~ 2.6 at r >= 5; declare 4.
	let leaf = GpuNode::expr(Arc::new(thread), 4.0, Some(Aabb::from_center_half_extent(Vec3::new(0.0, 0.0, 10.0), Vec3::new(10.0, 10.0, 12.0))));
	let tree = leaf
		.intersection(GpuNode::cylinder(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 20.0), 10.0))
		.offset_by(Arc::new(Expr::Mul(b(Expr::Const(-0.01)), b(Expr::Z))), 0.5);
	let mut seed = 0x5eed_0005_u64;
	// Keep probes off the screw axis (atan2(0,0) pole; CPU returns 0, GPU is
	// indeterminate there — a documented Expr pole, not a parity target).
	let probes: Vec<Vec3> = lcg_points(400, tree.bounds(), 1.5, &mut seed)
		.into_iter()
		.filter(|p| p.x.hypot(p.y) > 0.5)
		.collect();
	assert!(probes.len() > 300, "probe filter kept enough points");
	assert_parity(&ctx, "helical_thread_expr", &tree, &probes);
}

#[test]
fn parity_expr_operator_coverage() {
	let Some(ctx) = gpu_or_skip("parity_expr_operator_coverage") else { return };
	// Every one of the 21 Expr operators in one finite-on-the-box expression
	// (poles kept off the probe domain: Div by 4+z^2, Sqrt of abs+1, Mod with
	// positive operands, Atan2 with x shifted positive).
	let b = |e: Expr| Box::new(e);
	let sphere_term = Expr::Sub(b(Expr::Length3(b(Expr::X), b(Expr::Y), b(Expr::Z))), b(Expr::Const(6.0)));
	let plane_term = Expr::Max(b(Expr::Neg(b(Expr::Y))), b(Expr::Mul(b(Expr::Const(0.25)), b(Expr::X))));
	let base = Expr::Min(b(sphere_term), b(plane_term));
	let wobble_mod = Expr::Sin(b(Expr::Mod(b(Expr::Add(b(Expr::X), b(Expr::Const(50.0)))), b(Expr::Const(7.3)))));
	let wobble_atan = Expr::Cos(b(Expr::Atan2 { y: b(Expr::Y), x: b(Expr::Add(b(Expr::X), b(Expr::Const(40.0)))) }));
	let root_term = Expr::Sqrt(b(Expr::Add(b(Expr::Abs(b(Expr::Z))), b(Expr::Const(1.0)))));
	let div_term = Expr::Clamp {
		value: b(Expr::Div(
			b(Expr::Length2(b(Expr::X), b(Expr::Y))),
			b(Expr::Add(b(Expr::Const(4.0)), b(Expr::Mul(b(Expr::Z), b(Expr::Z))))),
		)),
		lo: b(Expr::Const(-2.0)),
		hi: b(Expr::Const(2.0)),
	};
	let detail = Expr::Add(b(Expr::Add(b(wobble_mod), b(wobble_atan))), b(Expr::Add(b(root_term), b(div_term))));
	let expr = Expr::Add(b(base), b(Expr::Mul(b(Expr::Const(0.05)), b(detail))));
	let tree = GpuNode::expr(Arc::new(expr), 3.0, Some(Aabb::from_center_half_extent(Vec3::ZERO, Vec3::splat(12.0))));
	let mut seed = 0x5eed_0006_u64;
	let probes = lcg_points(300, Aabb::from_center_half_extent(Vec3::ZERO, Vec3::splat(12.0)), 0.0, &mut seed);
	assert_parity(&ctx, "expr_operator_coverage", &tree, &probes);
}

#[test]
fn lowering_rejects_the_unlowerable_loudly() {
	let Some(ctx) = gpu_or_skip("lowering_rejects_the_unlowerable_loudly") else { return };
	// Structured errors, not silent wrong fields: empty lattices (CPU distance
	// is +inf — unrepresentable in WGSL), non-finite parameters, and bad
	// Lipschitz declarations must all refuse to compile.
	let cases: Vec<(&str, GpuNode)> = vec![
		("empty lattice", GpuNode::lattice(vec![Vec3::ZERO], vec![])),
		("nan center", GpuNode::sphere(Vec3::new(f32::NAN, 0.0, 0.0), 1.0)),
		("bad lipschitz", GpuNode::expr(Arc::new(Expr::X), 0.0, None)),
		("bad strut index", GpuNode::lattice(vec![Vec3::ZERO], vec![(0, 3, 1.0, 1.0)])),
	];
	let mut got = Vec::new();
	for (name, tree) in &cases {
		got.push((name, GpuField::compile(&ctx, tree).is_err()));
	}
	assert_eq!(
		got.iter().filter(|(_, e)| *e).count(),
		cases.len(),
		"every unlowerable tree must yield Err: {got:?}"
	);
}
