// Copyright (c) LMCAD. Licensed under the MIT License.

//! R5 — run-to-run determinism of the boolean pipeline.
//!
//! The boolean arrangement is pure f64 geometry with no randomness of its own, so
//! the SAME recipe must produce the bit-identical solid every time. Historically it
//! did not: `std::collections::HashMap`/`HashSet` give every instance a fresh
//! `RandomState` seed, so any geometry decision fed by map *iteration order*
//! (triangle-soup order out of coincident-facet cancellation, region order out of
//! face recovery, …) flipped between runs — the same flange recipe validated in one
//! process run and stitch-exploded in the next, and the fuzz corpus scored 99.0%
//! vs 99.5% on identical seeds.
//!
//! A test cannot observe a *different process*, but it does not need to: each
//! in-process repeat constructs fresh `HashMap`s with fresh `RandomState` seeds, so
//! a surviving order dependence shows up as varying topology counts or volume bits
//! across in-process repeats. Pre-fix, this test failed within a handful of
//! repeats; it now pins the pipeline to one bit-exact outcome.

use kernel_brep::math::{DVec2, DVec3};
use kernel_brep::{
	cylinder, difference, fillet_circular_rim, revolve, union, validate, volume, Solid,
};

/// The historically flaky recipe (the `parts_gallery` flange, distilled):
/// L-profile ring revolve ∪ rim-filleted boss, then 7 drills (bore + 6 bolt holes)
/// — every op lands on caps already carrying the previous cuts' healed seams, the
/// regime where iteration-order-sensitive stitching used to flip outcomes.
fn build_flange() -> Solid {
	let profile = [
		DVec2::new(10.0, 0.0),
		DVec2::new(40.0, 0.0),
		DVec2::new(40.0, 7.0),
		DVec2::new(39.0, 8.0),
		DVec2::new(10.0, 8.0),
	];
	let ring = revolve(&profile, 64);
	let boss = cylinder(DVec3::new(0.0, 0.0, 8.0), DVec3::Z, 18.0, 28.0, 64);
	// Fillet the boss's top rim BEFORE the union (torus band entering the boolean).
	let boss = fillet_circular_rim(&boss, DVec3::new(18.0, 0.0, 36.0), 2.5, 8)
		.expect("boss rim fillet is in scope (convex primitive rim)");
	let mut body = union(&ring, &boss);
	// Through-bore, then a 6-bolt circle: 7 chained differences.
	body = difference(&body, &cylinder(DVec3::new(0.0, 0.0, -1.0), DVec3::Z, 10.0, 38.5, 64));
	for k in 0..6 {
		let a = std::f64::consts::TAU * k as f64 / 6.0;
		let bolt = cylinder(DVec3::new(30.0 * a.cos(), 30.0 * a.sin(), -1.0), DVec3::Z, 3.0, 10.0, 24);
		body = difference(&body, &bolt);
	}
	body
}

/// Everything we require to be bit-identical run to run: full topology counts and
/// the volume's exact bit pattern (float summation order is part of determinism).
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
struct Snapshot {
	closed: bool,
	manifold: bool,
	faces: usize,
	edges: usize,
	vertices: usize,
	half_edges: usize,
	shells: usize,
	genus: i64,
	euler: i64,
	volume_bits: u64,
}

fn snapshot(s: &Solid) -> Snapshot {
	let v = validate(s);
	Snapshot {
		closed: v.closed,
		manifold: v.manifold,
		faces: s.face_count(),
		edges: s.edge_count(),
		vertices: s.vertex_count(),
		half_edges: s.half_edge_count(),
		shells: v.shells,
		genus: v.genus,
		euler: v.euler_characteristic,
		volume_bits: volume(s).to_bits(),
	}
}

/// Serializes the tests in this binary around the process-global
/// `LMCAD_BREP_THREADS` variable: the threaded variant below flips it, and the
/// harness runs sibling tests on concurrent threads. (Outputs are schedule-
/// invariant by contract, so an overlap could not change a PASSING result —
/// but a FAILURE under overlap would be confusing to attribute.)
static ENV_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

#[test]
fn flange_recipe_is_bit_identical_over_40_runs() {
	let _env = ENV_LOCK.lock().unwrap();
	let first = snapshot(&build_flange());
	// Printed so 10 separate `--nocapture` invocations of this binary can be diffed
	// for CROSS-PROCESS bit-identity (an assertion can only compare within a process).
	println!("flange snapshot: {first:?}");
	// The deterministic outcome must also be the CORRECT one: a closed manifold
	// flange with 7 through-holes (bore + 6 bolts), in one shell.
	assert!(
		first.closed && first.manifold && first.shells == 1 && first.genus == 7,
		"the flange recipe must produce one valid genus-7 shell, got {first:?}"
	);
	for run in 1..40 {
		let snap = snapshot(&build_flange());
		assert!(
			snap == first,
			"run {run} diverged from run 0 — the boolean pipeline made an \
			 iteration-order-dependent decision:\n run 0: {first:?}\n run {run}: {snap:?}\n \
			 (volume run 0 = {}, run {run} = {})",
			f64::from_bits(first.volume_bits),
			f64::from_bits(snap.volume_bits)
		);
	}
}

/// The THREADED twin of the 40× pin (2026-07-30 intra-arrangement parallelism):
/// with `LMCAD_BREP_THREADS=4` the pipeline's pure-map stages run on scoped
/// workers, and 40 rebuilds must still be bit-identical — to each other AND to
/// the sequential (`=1`) schedule, since R5 promises one bit-exact outcome per
/// platform, not one per thread count. The engagement counter proves the
/// threaded path genuinely ran (an always-sequential stub cannot pass).
#[test]
fn flange_recipe_threaded_40_runs_match_sequential_bits() {
	let _env = ENV_LOCK.lock().unwrap();
	std::env::set_var("LMCAD_BREP_THREADS", "1");
	let sequential = snapshot(&build_flange());

	std::env::set_var("LMCAD_BREP_THREADS", "4");
	let engaged_before = kernel_brep::booleans::par_items_processed();
	for run in 0..40 {
		let snap = snapshot(&build_flange());
		assert!(
			snap == sequential,
			"threaded run {run} diverged from the sequential schedule — thread scheduling \
			 leaked into an observable byte (R5):\n sequential: {sequential:?}\n threaded run \
			 {run}: {snap:?}\n (volume sequential = {}, threaded = {})",
			f64::from_bits(sequential.volume_bits),
			f64::from_bits(snap.volume_bits)
		);
	}
	let engaged = kernel_brep::booleans::par_items_processed() - engaged_before;
	std::env::remove_var("LMCAD_BREP_THREADS");
	assert!(
		engaged > 0,
		"the 40 threaded rebuilds never dispatched an item to the worker pool — the threaded \
		 40× pin would be vacuously identical to the sequential one"
	);
}
