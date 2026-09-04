// Copyright (c) LMCAD. Licensed under the MIT License.

//! Gates for `kernel_model::optimize` — the general design-space harness.
//!
//! Every gate here is an **analytic problem with a hand-computable answer**, so
//! the harness is measured against arithmetic rather than against itself: a
//! quadratic bowl with a known minimum, the same bowl with a constraint whose
//! optimum is a hand-projected point, a two-objective trade-off with a closed-
//! form Pareto front. Plus the three that matter most for honesty: the
//! bit-determinism of a parallel run, the NEGATIVE CONTROL proving the impure-
//! evaluator net actually fires, and the loud refusal when nothing is feasible.
//!
//! The last gate leaves arithmetic behind and runs a real study over exact
//! B-rep geometry (a ribbed plate: wall thickness × rib count, mass from
//! `exact_volume`, stiffness from the exact mid-span section) — proving the
//! harness composes with the geometry kernel, and, because the honest
//! re-evaluation defaults to BIT-identical, proving the geometry evaluator is
//! bit-reproducible too.
//!
//! Run: `cargo test --release -p kernel-model --test optimize`

use kernel_brep::math::DVec3;
use kernel_brep::{cuboid, exact_volume, section_properties, union, Solid};
use kernel_model::materials::PLA_G_PER_MM3;
use kernel_model::optimize::{gate_study, Constraint, DesignVar, Evaluation, Params, SearchOptions, Strategy, Study, StudyError};
use std::collections::BTreeMap;

/// `[("x", 1.0), ("y", 2.0)]` → [`Params`].
fn params(pairs: &[(&str, f64)]) -> Params {
	pairs.iter().map(|(k, v)| (k.to_string(), *v)).collect()
}

// ===========================================================================
// GATE 1 — known optimum: sweep and local search both find it, local is cheaper
// ===========================================================================

/// Bowl with the minimum at (2, 3): `f = (x−2)² + 2(y−3)² + 1`, `f_min = 1`.
fn bowl(p: &Params) -> Evaluation {
	let (x, y) = (p["x"], p["y"]);
	Evaluation::new().objective("f", (x - 2.0).powi(2) + 2.0 * (y - 3.0).powi(2) + 1.0)
}

#[test]
fn known_optimum_found_by_sweep_and_by_local_search_with_measured_costs() {
	// Declaration: x, y ∈ [0, 5] continuous, swept at 21 levels each — the
	// lattice step is exactly 5/20 = 0.25, so the analytic minimum (2, 3) is a
	// sampled point and the sweep can hit it EXACTLY.
	let study =
		Study::new(bowl).var(DesignVar::continuous("x", 0.0, 5.0)).var(DesignVar::continuous("y", 0.0, 5.0)).grid_levels(21).minimize("f");

	let grid = study.full_factorial().expect("grid runs");
	let grid_best = grid.best("f").expect("the unconstrained bowl has a feasible optimum");
	assert!(
		grid.evaluation_count() == 441 && grid.strategy == Strategy::FullFactorial && grid.stop_reason == "exhaustive",
		"the exhaustive sweep must cost exactly 21×21 = 441 evaluations: got {} ({:?}, stop={})",
		grid.evaluation_count(),
		grid.strategy,
		grid.stop_reason
	);
	assert!(
		grid_best.params["x"] == 2.0 && grid_best.params["y"] == 3.0 && grid_best.value == 1.0,
		"sweep must land bit-exactly on the analytic minimum (x=2, y=3, f=1): got x={}, y={}, f={} \
		 (re-evaluated; recorded {})",
		grid_best.params["x"],
		grid_best.params["y"],
		grid_best.value,
		grid_best.recorded_value
	);

	// Same declaration, derivative-free local search from the corner (0, 0).
	let opts = SearchOptions { max_evaluations: 400, step_tolerance: 1e-4, ..SearchOptions::default() };
	let local = study.pattern_search(&params(&[("x", 0.0), ("y", 0.0)]), opts).expect("pattern search runs");
	let local_best = local.best("f").expect("local search finds a feasible optimum");
	let (dx, dy, df) = ((local_best.params["x"] - 2.0).abs(), (local_best.params["y"] - 3.0).abs(), (local_best.value - 1.0).abs());
	assert!(
		dx < 1e-3 && dy < 1e-3 && df < 1e-6,
		"pattern search must reach the analytic minimum within the stated tolerance (|Δx|,|Δy| < 1e-3, |Δf| < 1e-6 \
		 at step_tolerance 1e-4): got x={:.9} (Δ{dx:.2e}), y={:.9} (Δ{dy:.2e}), f={:.12} (Δ{df:.2e}), stop={}",
		local_best.params["x"],
		local_best.params["y"],
		local_best.value,
		local.stop_reason
	);
	// THE EFFICIENCY CLAIM, MEASURED — not asserted in prose.
	assert!(
		local.evaluation_count() == 125 && local.evaluation_count() < grid.evaluation_count(),
		"pattern search must reach the same optimum in FEWER evaluations than the sweep, pinned at 125 (3.5× cheaper): \
		 got {} vs {} (stop={})",
		local.evaluation_count(),
		grid.evaluation_count(),
		local.stop_reason
	);

	// Sensitivity: y is the stiffer direction (weight 2 vs 1) over an identical
	// [0,5] box, so its range-normalized main effect must be the larger one.
	// Balanced only for the exhaustive sample — the harness says which it has.
	let (sx, sy) = (grid.sensitivity["x"]["f"], grid.sensitivity["y"]["f"]);
	assert!(
		grid.sensitivity_balanced && !local.sensitivity_balanced && sy > sx && sx > 0.0,
		"main effects on a balanced sweep must rank y (weight 2) above x (weight 1) and flag balance: \
		 x={sx:.6}, y={sy:.6}, grid balanced={}, local balanced={}",
		grid.sensitivity_balanced,
		local.sensitivity_balanced
	);
}

// ===========================================================================
// GATE 2 — constrained: the unconstrained optimum is infeasible
// ===========================================================================

/// Bowl centred at (3, 4) with the linear quantity `sum = x + y` exposed as a
/// constrainable measure: `f = (x−3)² + (y−4)²`.
fn offset_bowl(p: &Params) -> Evaluation {
	let (x, y) = (p["x"], p["y"]);
	Evaluation::new().objective("f", (x - 3.0).powi(2) + (y - 4.0).powi(2)).constraint("sum", x + y)
}

#[test]
fn constrained_optimum_sits_on_the_boundary_and_infeasible_points_are_kept() {
	// Hand calculation: the unconstrained minimum (3, 4) has x+y = 7 and is
	// therefore INFEASIBLE under x+y ≤ 5. For an isotropic bowl the constrained
	// optimum is the orthogonal projection of the centre onto the active line:
	// (3,4) − ((7−5)/2)·(1,1) = (2, 3), with f = 1² + 1² = 2 exactly.
	let study = Study::new(offset_bowl)
		.var(DesignVar::continuous("x", 0.0, 6.0))
		.var(DesignVar::continuous("y", 0.0, 6.0))
		.grid_levels(25) // lattice step 6/24 = 0.25 — (2,3) AND (3,4) are both sampled
		.minimize("f")
		.constrain(Constraint::less_than("sum", 5.0));

	let grid = study.full_factorial().expect("grid runs");
	let best = grid.best("f").expect("the constrained problem has feasible designs");
	assert!(
		best.params["x"] == 2.0 && best.params["y"] == 3.0 && best.value == 2.0 && best.violation == 0.0,
		"constrained optimum must be the hand-projected point (2, 3) with f = 2 exactly: got x={}, y={}, f={} \
		 (recorded {}), violation {}",
		best.params["x"],
		best.params["y"],
		best.value,
		best.recorded_value,
		best.violation
	);
	let sum = best.constraints["sum"];
	assert!(
		sum == 5.0,
		"the returned best must sit ON the active constraint boundary (x+y = 5, inclusive bound ⇒ feasible): got \
		 sum = {sum} (Δ{:.2e})",
		(sum - 5.0).abs()
	);

	// Infeasible designs are RETAINED and MARKED — never silently dropped.
	let total = grid.evaluation_count();
	let infeasible = grid.infeasible().count();
	let unconstrained = grid
		.evaluations
		.iter()
		.find(|r| r.params["x"] == 3.0 && r.params["y"] == 4.0)
		.expect("the unconstrained optimum must still appear in the report");
	assert!(
		total == 625 && grid.feasible_count + infeasible == total && infeasible == 394 && !unconstrained.feasible,
		"the report must keep every point and mark feasibility: {total} evaluations = {} feasible + {infeasible} \
		 infeasible (want 625 = 231 + 394); the unconstrained optimum (3,4) has f={:.6}, sum={:.6}, violation={:.6}, \
		 feasible={}",
		grid.feasible_count,
		unconstrained.objectives["f"],
		unconstrained.constraints["sum"],
		unconstrained.violation,
		unconstrained.feasible
	);

	// STATED LIMITATION, PINNED (not hidden): the poll is axis-aligned, so a
	// DIAGONAL active constraint can only be followed as a staircase — the
	// search stays feasible and ends on the boundary, but it can only approach
	// the constrained optimum to within its final step, never land on it the
	// way the sweep does (measured miss below: Δf = 2.4e-4 at step_tolerance
	// 1e-4, i.e. 0.012%). What is asserted is what is true: feasible, on the
	// boundary, and NEVER better than the exhaustive answer — a local search
	// that beat the exhaustive optimum would mean the harness is lying.
	let local = study.pattern_search(&params(&[("x", 0.0), ("y", 0.0)]), SearchOptions::default()).expect("pattern search runs");
	let local_best = local.best("f").expect("local search stays feasible");
	let local_sum = local_best.constraints["sum"];
	assert!(
		local_best.violation == 0.0 && (local_sum - 5.0).abs() < 1e-3 && local_best.value >= best.value,
		"pattern search under an active diagonal constraint must stay feasible, stall on the boundary, and never \
		 beat the sweep: got x={:.6}, y={:.6}, sum={local_sum:.6}, f={:.6} (sweep optimum f={:.6}), violation={}",
		local_best.params["x"],
		local_best.params["y"],
		local_best.value,
		best.value,
		local_best.violation
	);
	assert!(
		local_best.value > 2.0 && (local_best.value - 2.000244155526161).abs() < 1e-12,
		"the staircase stall point is pinned so a change in the search is visible: f = {:.15} (want 2.000244155526161, \
		 strictly above the exhaustive optimum 2.0)",
		local_best.value
	);
}

// ===========================================================================
// GATE 3 — Pareto front against an analytic trade-off
// ===========================================================================

#[test]
fn pareto_front_matches_the_analytic_trade_off_and_excludes_dominated_points() {
	// Minimize f1 = x and f2 = (x−1)² over x ∈ [0, 2] on the 0.1 lattice.
	// Analytically: for x ≤ 1 lowering x improves f1 but worsens f2, so every
	// such point is non-dominated; for x > 1 BOTH objectives are worse than at
	// x = 1, so every such point is dominated. Front = {x ∈ [0,1]} = 11 points,
	// lying exactly on f2 = (1 − f1)².
	let study = Study::new(|p: &Params| {
		let x = p["x"];
		Evaluation::new().objective("f1", x).objective("f2", (x - 1.0).powi(2))
	})
	.var(DesignVar::stepped("x", 0.0, 2.0, 0.1))
	.minimize("f1")
	.minimize("f2");

	let report = study.full_factorial().expect("study runs");
	assert!(
		report.evaluation_count() == 21 && report.pareto_front == (0..=10).collect::<Vec<_>>(),
		"the front must be exactly the 11 sampled points with x ≤ 1: got {} evaluations, front {:?}",
		report.evaluation_count(),
		report.pareto_front
	);

	// On the analytic curve, and monotone (a real trade-off: f2 falls as f1 rises).
	let mut worst_curve = 0.0f64;
	let mut monotone = true;
	let mut previous: Option<(f64, f64)> = None;
	for &i in &report.pareto_front {
		let (f1, f2) = (report.evaluations[i].objectives["f1"], report.evaluations[i].objectives["f2"]);
		worst_curve = worst_curve.max((f2 - (1.0 - f1).powi(2)).abs());
		if let Some((pf1, pf2)) = previous {
			monotone &= f1 > pf1 && f2 < pf2;
		}
		previous = Some((f1, f2));
	}
	assert!(
		worst_curve < 1e-12 && monotone,
		"the extracted front must lie on the analytic curve f2 = (1 − f1)² and fall monotonically: worst deviation \
		 {worst_curve:.3e} (band 1e-12), monotone = {monotone}"
	);

	// Non-domination, checked directly against every sampled point.
	let dominated_by_any = report.pareto_front.iter().any(|&i| {
		report.evaluations.iter().any(|o| {
			let (a1, a2) = (o.objectives["f1"], o.objectives["f2"]);
			let (b1, b2) = (report.evaluations[i].objectives["f1"], report.evaluations[i].objectives["f2"]);
			a1 <= b1 && a2 <= b2 && (a1 < b1 || a2 < b2)
		})
	});
	// The negative control: x = 1.5 IS sampled, IS dominated by x = 1.0
	// (f1 1.0 < 1.5 and f2 0.0 < 0.25), and MUST NOT be on the front.
	let fifteen = &report.evaluations[15];
	assert!(
		!dominated_by_any && !report.pareto_front.contains(&15) && fifteen.params["x"] == 1.5,
		"no front point may be dominated, and the dominated point x=1.5 (f1={:.6}, f2={:.6}; beaten by x=1.0 with \
		 f1=1.0, f2=0.0) must be excluded: dominated_by_any={dominated_by_any}, front={:?}",
		fifteen.objectives["f1"],
		fifteen.objectives["f2"],
		report.pareto_front
	);
}

// ===========================================================================
// GATE 4 — determinism, with parallel evaluation ENABLED
// ===========================================================================

#[test]
fn two_identical_studies_produce_bit_identical_reports() {
	// Deliberately float-heavy and non-associative so any scheduling leak would
	// show up in the low bits; run over 3 variables (5×5×5 = 125 points) so the
	// parallel map really does fan out.
	let evaluator = |p: &Params| {
		let (x, y, z) = (p["x"], p["y"], p["z"]);
		let f = (x * 0.7).sin() * (y * 1.3).cos() + (1.0 + z * z).ln() + x * y * z / 7.0;
		Evaluation::new().objective("f", f).objective("g", x + y - z).constraint("hull", x * x + y * y + z * z)
	};
	fn declare(study: Study<'_>) -> Study<'_> {
		study
			.var(DesignVar::continuous("x", -1.0, 2.0))
			.var(DesignVar::continuous("y", 0.5, 3.5))
			.var(DesignVar::continuous("z", 0.0, 2.0))
			.minimize("f")
			.maximize("g")
			.constrain(Constraint::less_than("hull", 8.0))
	}
	let a = declare(Study::new(evaluator));
	let b = declare(Study::new(evaluator));

	let (ra, rb) = (a.full_factorial().expect("a runs"), b.full_factorial().expect("b runs"));
	let (ca, cb) = (ra.canonical(), rb.canonical());
	let first_diff = ca.lines().zip(cb.lines()).position(|(l, r)| l != r);
	// Anti-vacuity: the comparison must be over a substantial dump, not two
	// empty strings. Measured 24 201 bytes / 144 lines = 8 declaration lines +
	// 125 evaluations + feasible_count + 2 bests + pareto + 6 sensitivities +
	// the balance flag.
	assert!(
		ca.len() > 10_000 && ca.lines().count() == 144 && ca.contains("obj:f=0x"),
		"the canonical dump must carry every evaluation with f64 bit patterns: {} bytes, {} lines",
		ca.len(),
		ca.lines().count()
	);
	assert!(
		ca == cb && ra.digest() == rb.digest() && ra.evaluation_count() == 125,
		"two identical sweeps must serialize bit-identically (f64 bit patterns included): {} evaluations, digests \
		 {:#018x} vs {:#018x}, first differing canonical line {:?}",
		ra.evaluation_count(),
		ra.digest(),
		rb.digest(),
		first_diff
	);

	// The same for the iterative strategy, whose candidate sets depend on the
	// search state — the harder determinism case.
	let start = params(&[("x", 0.0), ("y", 1.0), ("z", 1.0)]);
	let (sa, sb) = (
		a.pattern_search(&start, SearchOptions::default()).expect("a searches"),
		b.pattern_search(&start, SearchOptions::default()).expect("b searches"),
	);
	assert!(
		sa.canonical() == sb.canonical() && sa.digest() == sb.digest() && sa.evaluation_count() > 1,
		"two identical pattern searches must serialize bit-identically: {} vs {} evaluations, digests {:#018x} vs \
		 {:#018x}",
		sa.evaluation_count(),
		sb.evaluation_count(),
		sa.digest(),
		sb.digest()
	);
	// Parallel evaluation was genuinely available for those maps.
	assert!(
		std::thread::available_parallelism().map(|n| n.get()).unwrap_or(1) >= 1 && ra.evaluation_count() > 1,
		"the determinism claim is only interesting with the parallel map engaged: {} cores, {} evaluations",
		std::thread::available_parallelism().map(|n| n.get()).unwrap_or(0),
		ra.evaluation_count()
	);
}

// ===========================================================================
// GATE 5 — NEGATIVE CONTROL: the honest re-evaluation actually fires
// ===========================================================================

#[test]
fn impure_evaluator_is_caught_by_the_honest_re_evaluation() {
	use std::sync::atomic::{AtomicUsize, Ordering};

	// A one-point design space, so the search makes exactly one call and
	// `best()` makes exactly one more. The evaluator returns a DIFFERENT value
	// on the second call — the classic hidden-state bug this net exists for.
	let calls = AtomicUsize::new(0);
	let impure = Study::new(|p: &Params| {
		let n = calls.fetch_add(1, Ordering::SeqCst) as f64;
		Evaluation::new().objective("f", p["x"] + n)
	})
	.var(DesignVar::stepped("x", 1.0, 1.0, 1.0))
	.minimize("f");

	let report = impure.full_factorial().expect("the sweep itself succeeds — the lie only shows on re-evaluation");
	let err = report.best("f").expect_err("an impure evaluator must NOT yield a best design");
	let text = err.to_string();
	match &err {
		StudyError::ImpureEvaluator { name, recorded, reevaluated, tolerance, .. } => assert!(
			name == "f" && *recorded == 1.0 && *reevaluated == 2.0 && *tolerance == 0.0,
			"the mismatch must be reported with both numbers: name={name}, recorded={recorded}, \
			 reevaluated={reevaluated}, tolerance={tolerance}"
		),
		other => panic!("expected StudyError::ImpureEvaluator, got {other:?}"),
	}
	assert!(
		text.contains("IMPURE EVALUATOR")
			&& text.contains("NOT reproducible")
			&& text.contains("must not be quoted")
			&& text.contains("1.0")
			&& text.contains("2.0"),
		"the refusal must be loud and self-explaining, got: {text}"
	);

	// POSITIVE CONTROL — the same shape with a pure evaluator returns a best
	// design whose re-evaluated value is bit-identical to the recorded one, so
	// the gate above is proving a real net, not an always-on refusal.
	let pure =
		Study::new(|p: &Params| Evaluation::new().objective("f", p["x"] * 3.0)).var(DesignVar::stepped("x", 1.0, 1.0, 1.0)).minimize("f");
	let best = pure.full_factorial().expect("runs").best("f").expect("a pure evaluator re-evaluates cleanly");
	assert!(
		best.value == 3.0 && best.value.to_bits() == best.recorded_value.to_bits() && best.evaluator_calls == 2,
		"a pure evaluator must re-evaluate bit-identically: value {} (bits {:#018x}) vs recorded {} (bits \
		 {:#018x}), {} evaluator calls (1 search + 1 honest re-run)",
		best.value,
		best.value.to_bits(),
		best.recorded_value,
		best.recorded_value.to_bits(),
		best.evaluator_calls
	);
}

// ===========================================================================
// GATE 6 — nothing is feasible: refuse loudly, with the closest miss
// ===========================================================================

#[test]
fn a_study_with_no_feasible_design_refuses_with_the_closest_miss() {
	// x, y ∈ [0, 2] on a 0.5 lattice (25 points); the constraint demands
	// x + y ≥ 10, which the box cannot reach — the closest point is the last
	// one swept, (2, 2), missing by exactly 6.
	let study = Study::new(|p: &Params| {
		let (x, y) = (p["x"], p["y"]);
		Evaluation::new().objective("f", x * y).constraint("sum", x + y)
	})
	.var(DesignVar::stepped("x", 0.0, 2.0, 0.5))
	.var(DesignVar::stepped("y", 0.0, 2.0, 0.5))
	.maximize("f")
	.constrain(Constraint::greater_than("sum", 10.0));

	let report = study.full_factorial().expect("the sweep runs — infeasibility is not a run failure");
	assert!(
		report.evaluation_count() == 25 && report.feasible_count == 0 && report.infeasible().count() == 25,
		"every infeasible point must still be reported: {} evaluations, {} feasible, {} infeasible",
		report.evaluation_count(),
		report.feasible_count,
		report.infeasible().count()
	);
	assert!(
		report.pareto_front.is_empty() && report.best_per_objective["f"].is_none(),
		"with nothing feasible there is no front and no recorded winner: front {:?}, best {:?}",
		report.pareto_front,
		report.best_per_objective["f"]
	);

	let err = report.best("f").expect_err("best() must refuse rather than return an infeasible design");
	let text = err.to_string();
	match &err {
		StudyError::NoFeasibleDesign { evaluated, closest_index, closest_params, closest_violation, .. } => assert!(
			*evaluated == 25
				&& *closest_index == 24
				&& closest_params["x"] == 2.0
				&& closest_params["y"] == 2.0
				&& *closest_violation == 6.0,
			"the refusal must carry the closest miss: evaluated={evaluated}, index={closest_index}, \
			 params={closest_params:?}, violation={closest_violation}"
		),
		other => panic!("expected StudyError::NoFeasibleDesign, got {other:?}"),
	}
	assert!(
		text.contains("no feasible design in 25 evaluations")
			&& text.contains("closest miss is #24")
			&& text.contains("x=2.000000, y=2.000000")
			&& text.contains("violation 6.000000")
			&& text.contains("'sum' wants ≥ 10.000000, got 4.000000"),
		"the refusal must say what to relax, got: {text}"
	);
}

// ===========================================================================
// GATE 7 — real geometry: a ribbed plate through the exact B-rep kernel
// ===========================================================================

const PLATE_L: f64 = 60.0; // along +X (the beam span)
const PLATE_W: f64 = 40.0; // along +Y
const RIB_W: f64 = 3.0; // rib thickness in Y
const RIB_H: f64 = 6.0; // rib height ABOVE the plate top
const RIB_INSET: f64 = 2.0; // X inset at each end — no coplanar end faces in the union
const RIB_EMBED: f64 = 0.5; // rib root sunk into the plate — a clean overlapping union

/// Ribbed plate: a `PLATE_L × PLATE_W × t` base with `ribs` full-length ribs on
/// top, evenly spaced across the width. Every rib overlaps the plate by
/// `RIB_EMBED` and is inset in X, so each union is a proper overlapping boolean
/// (DESIGN_GUIDE §7.7: no face-coincident adders, no coplanar end faces).
fn ribbed_plate(t: f64, ribs: usize) -> Solid {
	let mut s = cuboid(DVec3::ZERO, DVec3::new(PLATE_L, PLATE_W, t));
	for i in 0..ribs {
		let c = PLATE_W * (i + 1) as f64 / (ribs + 1) as f64;
		let rib =
			cuboid(DVec3::new(RIB_INSET, c - 0.5 * RIB_W, t - RIB_EMBED), DVec3::new(PLATE_L - RIB_INSET, c + 0.5 * RIB_W, t + RIB_H));
		s = union(&s, &rib);
	}
	s
}

/// Hand calculation for the mid-span section: composite second moment of area
/// about the horizontal centroidal axis (plate rectangle + `ribs` rectangles,
/// parallel-axis theorem), mm⁴.
fn analytic_i_zz(t: f64, ribs: usize) -> f64 {
	let (a1, z1) = (PLATE_W * t, 0.5 * t);
	let i1 = PLATE_W * t.powi(3) / 12.0;
	let n = ribs as f64;
	let (a2, z2) = (n * RIB_W * RIB_H, t + 0.5 * RIB_H);
	let i2 = n * RIB_W * RIB_H.powi(3) / 12.0;
	let a = a1 + a2;
	let zbar = if a > 0.0 { (a1 * z1 + a2 * z2) / a } else { 0.0 };
	i1 + a1 * (z1 - zbar).powi(2) + i2 + a2 * (z2 - zbar).powi(2)
}

/// Measure one candidate plate with the geometry kernel: mass from the EXACT
/// volume, bending stiffness from the mid-span cross-section (all faces planar,
/// so the sectioned polygon is exact).
fn measure_plate(p: &Params) -> Evaluation {
	let t = p["wall"];
	let ribs = p["ribs"].round() as usize;
	let solid = ribbed_plate(t, ribs);
	let mass_g = exact_volume(&solid) * PLA_G_PER_MM3;
	let sp = section_properties(&solid, DVec3::new(0.5 * PLATE_L, 0.0, 0.0), DVec3::X).expect("the mid-span plane cuts the plate");
	// The section basis is chosen by the kernel; take the moment about the
	// horizontal axis, i.e. the one measured along whichever in-plane axis is
	// the vertical (Z) one.
	let i_zz = if sp.v_axis.z.abs() > sp.u_axis.z.abs() { sp.i_vv } else { sp.i_uu };
	Evaluation::new().objective("mass_g", mass_g).objective("i_zz_mm4", i_zz).constraint("mass_g", mass_g)
}

#[test]
fn real_geometry_study_picks_the_stiffest_plate_under_a_mass_budget() {
	// wall ∈ {1.6, 2.0, 2.4, 2.8, 3.2} mm × ribs ∈ {0, 1, 2, 3} = 20 exact
	// B-rep builds. Maximize section stiffness under a 10 g PLA budget.
	let study = Study::new(measure_plate)
		.var(DesignVar::stepped("wall", 1.6, 3.2, 0.4))
		.var(DesignVar::stepped("ribs", 0.0, 3.0, 1.0))
		.maximize("i_zz_mm4")
		.minimize("mass_g")
		.constrain(Constraint::less_than("mass_g", 10.0));

	let report = study.full_factorial().expect("the geometry sweep runs");
	assert!(
		report.evaluation_count() == 20 && report.feasible_count == 14,
		"the sweep must build all 20 candidates and mark the over-budget ones: {} evaluations, {} feasible, {} \
		 infeasible",
		report.evaluation_count(),
		report.feasible_count,
		report.infeasible().count()
	);

	// The honest winner — re-evaluated through the geometry kernel at the
	// DEFAULT bit-identical tolerance, so this also proves the B-rep evaluator
	// is bit-reproducible.
	let best = report.best("i_zz_mm4").expect("a feasible plate exists");
	let (wall, ribs) = (best.params["wall"], best.params["ribs"]);
	let mass = best.objectives["mass_g"];
	assert!(
		wall == 2.0 && ribs == 3.0 && best.value.to_bits() == best.recorded_value.to_bits(),
		"the stiffness-optimal plate under the 10 g budget is 2.0 mm wall × 3 ribs (thicker walls must drop a rib \
		 to fit the budget, and ribs buy far more I than wall does): got wall={wall}, ribs={ribs}, I={:.4} mm⁴ \
		 (recorded {:.4}), mass={mass:.4} g",
		best.value,
		best.recorded_value
	);
	// Against the hand calculation, and against the budget.
	let analytic = analytic_i_zz(2.0, 3);
	let mass_analytic = (PLATE_L * PLATE_W * 2.0 + 3.0 * (PLATE_L - 2.0 * RIB_INSET) * RIB_W * RIB_H) * PLA_G_PER_MM3;
	// Band: 1e-5 relative on I, because the sectioning path runs on the f32 mesh
	// vertices (docs/NUMERICS.md f32/f64 split ⇒ ~1e-7 relative per coordinate);
	// mass comes from the f64 exact volume and is pinned to 1e-9 g.
	assert!(
		(best.value - analytic).abs() / analytic < 1e-5 && (mass - mass_analytic).abs() < 1e-9 && mass <= 10.0 && best.violation == 0.0,
		"the measured optimum must match the composite-section hand calc and the exact volume: I = {:.6} mm⁴ vs \
		 analytic {analytic:.6} (Δ{:.3e} relative, band 1e-5), mass = {mass:.6} g vs analytic {mass_analytic:.6} g, \
		 budget 10 g, violation {}",
		best.value,
		(best.value - analytic).abs() / analytic,
		best.violation
	);

	// The two objectives genuinely conflict: the lightest feasible plate is the
	// bare 1.6 mm sheet, which is nowhere near the stiffest.
	let lightest = report.best("mass_g").expect("a feasible plate exists");
	// The front is pinned by MEMBERSHIP: indices are wall_level·4 + rib_level,
	// so 0..7 are every 1.6 and 2.0 mm plate, plus 10 = (2.4 mm, 2 ribs) which
	// is lighter than (2.0, 3) and stiffer than everything lighter. 8 = (2.4, 0)
	// and 9 = (2.4, 1) are correctly dominated by thinner-but-ribbed plates.
	assert!(
		lightest.params["wall"] == 1.6
			&& lightest.params["ribs"] == 0.0
			&& (lightest.value - PLATE_L * PLATE_W * 1.6 * PLA_G_PER_MM3).abs() < 1e-9
			&& report.pareto_front == vec![0, 1, 2, 3, 4, 5, 6, 7, 10],
		"mass and stiffness must trade off: lightest = wall {} / ribs {} at {:.4} g (want 1.6 / 0 / 4.7616 g), \
		 front {:?} (want [0,1,2,3,4,5,6,7,10])",
		lightest.params["wall"],
		lightest.params["ribs"],
		lightest.value,
		report.pareto_front
	);
	// Rib count dominates stiffness over this box — the screening indicator
	// agreeing with the mechanics is the point of reporting it.
	let (s_wall, s_ribs) = (report.sensitivity["wall"]["i_zz_mm4"], report.sensitivity["ribs"]["i_zz_mm4"]);
	assert!(
		(s_ribs - 0.728872).abs() < 5e-4 && (s_wall - 0.272311).abs() < 5e-4 && s_ribs > s_wall,
		"rib count must screen as the dominant stiffness driver: ribs {s_ribs:.6} (want 0.728872) vs wall \
		 {s_wall:.6} (want 0.272311), range-normalized main effects, balanced = {}",
		report.sensitivity_balanced
	);

	// CAMPAIGN INTEGRATION — the shipped design is re-proven to BE the study's
	// optimum on every run, and the gate bites when it is not.
	let mut ok = true;
	let shipped = params(&[("wall", 2.0), ("ribs", 3.0)]);
	let gated = gate_study("shipped plate = study optimum", &report, "i_zz_mm4", &shipped, 1e-9, &mut ok)
		.expect("the gate returns the re-evaluated winner");
	assert!(
		ok && gated.value == best.value,
		"gate_study must pass for the shipped optimum and hand back the re-evaluated value: ok={ok}, gated I={:.4}, \
		 best I={:.4}",
		gated.value,
		best.value
	);
	let mut ok_bad = true;
	let wrong = params(&[("wall", 2.8), ("ribs", 3.0)]);
	gate_study("negative control: wrong wall", &report, "i_zz_mm4", &wrong, 1e-9, &mut ok_bad);
	assert!(!ok_bad, "gate_study must FAIL a shipped design that is not the study optimum (wall 2.8 vs 2.0)");
}

// ===========================================================================
// Declaration guards — refusals a caller can act on
// ===========================================================================

#[test]
fn unusable_declarations_and_queries_refuse_with_reasons() {
	let ev = |_: &Params| Evaluation::new().objective("f", 1.0);
	let cases: Vec<(Study<'_>, &str)> = vec![
		(Study::new(ev).minimize("f"), "no design variables declared"),
		(Study::new(ev).var(DesignVar::continuous("x", 0.0, 1.0)), "no objectives declared"),
		(
			Study::new(ev).var(DesignVar::continuous("x", 0.0, 1.0)).var(DesignVar::continuous("x", 0.0, 1.0)).minimize("f"),
			"duplicate design variable 'x'",
		),
		(Study::new(ev).var(DesignVar::continuous("x", 2.0, 1.0)).minimize("f"), "has max 1 below min 2"),
		(Study::new(ev).var(DesignVar::stepped("x", 0.0, 1.0, 0.0)).minimize("f"), "non-positive step 0"),
	];
	for (study, want) in cases {
		let text = study.full_factorial().expect_err("must refuse").to_string();
		assert!(text.contains(want), "declaration refusal should mention {want:?}, got: {text}");
	}

	// A sweep that would be enormous is refused with its own cost, not run.
	let huge = Study::new(ev)
		.var(DesignVar::stepped("x", 0.0, 1000.0, 1.0))
		.var(DesignVar::stepped("y", 0.0, 1000.0, 1.0))
		.minimize("f")
		.evaluation_cap(10_000);
	let text = huge.full_factorial().expect_err("must refuse").to_string();
	assert!(text.contains("1002001 evaluations, above the cap of 10000"), "the sweep must state its cost when refusing, got: {text}");

	// An out-of-bounds start, an unknown objective, and an evaluator that
	// forgets a declared name each refuse by name.
	let study = Study::new(ev).var(DesignVar::continuous("x", 0.0, 1.0)).minimize("f");
	let text = study.pattern_search(&params(&[("x", 5.0)]), SearchOptions::default()).expect_err("must refuse").to_string();
	assert!(text.contains("'x' at 5, outside its declared [0, 1]"), "start-bounds refusal, got: {text}");
	let report = study.full_factorial().expect("runs");
	let text = report.best("nope").expect_err("must refuse").to_string();
	assert!(text.contains("no objective 'nope' in this study — declared: f"), "unknown-objective refusal, got: {text}");

	let forgetful = Study::new(|_: &Params| Evaluation::new().objective("g", 1.0)).var(DesignVar::continuous("x", 0.0, 1.0)).minimize("f");
	let text = forgetful.full_factorial().expect_err("must refuse").to_string();
	assert!(
		text.contains("evaluator returned no objective 'f'") && text.contains("it returned: g"),
		"missing-value refusal must name what was and was not returned, got: {text}"
	);

	let nan = Study::new(|_: &Params| Evaluation::new().objective("f", f64::NAN)).var(DesignVar::continuous("x", 0.0, 1.0)).minimize("f");
	let text = nan.full_factorial().expect_err("must refuse").to_string();
	assert!(text.contains("non-finite objective 'f' = NaN"), "NaN refusal, got: {text}");
}

/// Lattice arithmetic is the ground everything else stands on: levels are
/// `min + k·step` (no accumulation drift), `max` is included only when it lands
/// on the lattice, and every candidate is snapped back into the declaration.
#[test]
fn design_variable_lattice_and_snapping_are_exact() {
	let v = DesignVar::stepped("t", 1.6, 3.2, 0.4);
	let levels = v.levels(5);
	assert!(
		levels.len() == 5 && levels[0] == 1.6 && levels[2] == 2.4000000000000004 && levels[4] == 3.2,
		"stepped levels must be min + k·step exactly: {levels:?}"
	);
	let c = DesignVar::continuous("x", 0.0, 5.0);
	let cl = c.levels(21);
	assert!(
		cl.len() == 21 && cl[8] == 2.0 && cl[12] == 3.0 && cl[20] == 5.0,
		"continuous levels must include both endpoints exactly: {} levels, [8]={}, [12]={}, [20]={}",
		cl.len(),
		cl[8],
		cl[12],
		cl[20]
	);
	// Below/above the 1.8 midpoint the snap must fall to 1.6 / rise to 2.0, and
	// out-of-range values clamp to the declared ends.
	let snapped: Vec<f64> = [-1.0, 1.79, 1.81, 9.0].iter().map(|&x| v.snap(x)).collect();
	assert!(snapped == vec![1.6, 1.6, 2.0, 3.2], "snap must clamp into [min, max] and land on the nearest lattice point: {snapped:?}");
	let dup: BTreeMap<u64, ()> = cl.iter().map(|x| (x.to_bits(), ())).collect();
	assert!(dup.len() == cl.len(), "sweep levels must be distinct bit patterns: {} of {}", dup.len(), cl.len());
}
