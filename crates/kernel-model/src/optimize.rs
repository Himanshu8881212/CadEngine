// Copyright (c) LMCAD. Licensed under the MIT License.

//! **Design-space studies** — the general optimization harness: declared design
//! variables, objectives and constraints, deterministic derivative-free search,
//! and a winner that is **re-evaluated before anyone may quote it**.
//!
//! This codifies what campaigns used to do ad hoc — the BEYBLADE's "+31% I_z
//! under mass/envelope constraints" spreadsheet, the drives' `tools/
//! param_optimize.py` Nelder-Mead runs — into one surface that composes with
//! [`crate::campaign::gate`], so an optimum is **re-proven every run** like
//! every other campaign claim instead of being a one-off table in a README.
//!
//! ## The doctrine: an optimizer that lies about its optimum is the worst bug
//! `tools/ace_optimize_runner.py` (SIMP topology optimization) never reports
//! the homogenized proxy compliance it optimized; it runs ONE more honest
//! binary-occupancy re-analysis of the thresholded design — the part that would
//! actually be printed — and reports THAT. This module generalizes the same
//! rule to arbitrary evaluators:
//!
//! * [`StudyReport::best`] calls the evaluator **again** on the winning point
//!   and returns [`BestDesign::value`] from that re-run, not from the cached
//!   search record;
//! * any disagreement between the recorded value and the re-run is a loud typed
//!   [`StudyError::ImpureEvaluator`] — never a warning field someone can skip;
//! * infeasible designs are **retained** in [`StudyReport::evaluations`] and
//!   marked, because the near-miss designs are exactly what a designer wants to
//!   look at, and [`StudyReport::best`] still refuses to return one — with the
//!   closest-miss distance, so "no feasible design" is actionable.
//!
//! ## Determinism (hard contract, `docs/NUMERICS.md` R5)
//! Same declaration + same start point ⇒ **bit-identical** report. There is no
//! randomness anywhere in this module: every candidate set is a pure function of
//! the declaration and the search state, evaluations run through
//! [`kernel_core::par::par_map_indexed`] (results are returned BY INDEX, so
//! thread scheduling cannot reorder them), every map/summary iterates
//! [`std::collections::BTreeMap`]s in key order, and the first error in **index**
//! order wins. [`StudyReport::canonical`] serializes the whole report with f64
//! bit patterns so equality is checkable rather than believable.
//!
//! **The evaluator must be pure**: a function of `params` alone (no clocks, no
//! unseeded randomness, no mutable captured state, no caches keyed on anything
//! but `params`). Parallel evaluation makes an impure evaluator's result
//! schedule-dependent, and the honest re-evaluation exists to catch exactly
//! that — see the negative control in `tests/optimize.rs`.
//!
//! ## Strategies and their cost (stated, not implied)
//!
//! | strategy | evaluations | when to use |
//! |---|---|---|
//! | [`Study::full_factorial`] | `∏ levels(var)` — exhaustive | ≲4 variables and an affordable evaluator. The honest baseline: it cannot miss an optimum that lies on its own lattice, and it feeds a *balanced* sample to [`StudyReport::sensitivity`]. |
//! | [`Study::pattern_search`] | ≤ `2·k` per poll, ≈ `2·k·log2(step₀/tol)` total for `k` variables | expensive evaluator, local refinement from a known-good start. Derivative-free (compass/coordinate poll), so no smoothness is assumed — but it polls **axis-aligned directions only** and therefore stalls on an active constraint that is not axis-aligned (pinned as a limitation in `tests/optimize.rs`, not hidden). |
//! | [`pareto_front`] | `O(n²·m)` over `n` sampled points, `m` objectives | ≥2 objectives. **Extraction, not search**: the front is only as good as the sample handed to it. |
//!
//! ## Worked shape
//! ```no_run
//! use kernel_model::optimize::{Constraint, DesignVar, Evaluation, Params, Study};
//!
//! let study = Study::new(|p: &Params| {
//!     let (t, n) = (p["wall"], p["ribs"]);
//!     let mass = 2400.0 * t + 1008.0 * n;
//!     Evaluation::new().objective("mass_g", mass).constraint("mass_g", mass)
//! })
//! .var(DesignVar::stepped("wall", 1.6, 3.2, 0.4))
//! .var(DesignVar::stepped("ribs", 0.0, 3.0, 1.0))
//! .minimize("mass_g")
//! .constrain(Constraint::less_than("mass_g", 10.0));
//! let report = study.full_factorial().expect("study runs");
//! let best = report.best("mass_g").expect("a feasible design exists");
//! println!("{:?} -> {} (re-evaluated)", best.params, best.value);
//! ```

use kernel_core::par::par_map_indexed;
use std::collections::BTreeMap;
use std::fmt;

/// One point in the design space: variable name → value. A [`BTreeMap`] so
/// iteration is in name order on every platform and every run.
pub type Params = BTreeMap<String, f64>;

// ---------------------------------------------------------------------------
// Declaration
// ---------------------------------------------------------------------------

/// A declared design variable: an inclusive `[min, max]` interval, optionally
/// **discretized** by `step` (printed wall thicknesses come in 0.4 mm lattice
/// steps; rib counts are integers).
///
/// `step` also decides the sweep lattice: with `Some(s)` the levels are
/// `min + k·s` for every `k ≥ 0` with `min + k·s ≤ max` — note that `max`
/// itself is sampled only if it lands on that lattice (declare a `max` that
/// does, or leave `step` unset). With `None` the sweep uses
/// [`Study::grid_levels`] evenly spaced levels including both endpoints.
#[derive(Clone, Debug, PartialEq)]
pub struct DesignVar {
	/// Variable name — the key the evaluator reads out of [`Params`].
	pub name: String,
	/// Inclusive lower bound.
	pub min: f64,
	/// Inclusive upper bound.
	pub max: f64,
	/// Lattice step, or `None` for a continuous variable.
	pub step: Option<f64>,
}

impl DesignVar {
	/// A continuous variable on `[min, max]`.
	pub fn continuous(name: &str, min: f64, max: f64) -> Self {
		Self { name: name.to_string(), min, max, step: None }
	}

	/// A variable discretized to the `min + k·step` lattice inside `[min, max]`.
	pub fn stepped(name: &str, min: f64, max: f64, step: f64) -> Self {
		Self { name: name.to_string(), min, max, step: Some(step) }
	}

	/// The sweep levels of this variable, ascending — see the type docs for the
	/// lattice rule. Computed as `min + k·step` (multiplication, never
	/// accumulation) so the k-th level has no drift and is bit-reproducible.
	pub fn levels(&self, continuous_levels: usize) -> Vec<f64> {
		// NaN-safe: a degenerate or unusable interval collapses to one level
		// rather than emitting NaN sample points.
		if !self.min.is_finite() || !self.max.is_finite() || self.max <= self.min {
			return vec![self.min];
		}
		match self.step {
			Some(s) if s > 0.0 => {
				// 1e-9 relative slack so a level that lands on `max` up to
				// round-off is still sampled.
				let n = ((self.max - self.min) / s + 1e-9).floor().max(0.0) as usize;
				(0..=n).map(|k| self.min + k as f64 * s).collect()
			}
			_ => {
				let n = continuous_levels.max(2);
				(0..n)
					.map(|k| {
						if k + 1 == n {
							self.max
						} else {
							self.min + k as f64 * (self.max - self.min) / (n - 1) as f64
						}
					})
					.collect()
			}
		}
	}

	/// Clamp to `[min, max]` and, for a discretized variable, snap to the
	/// nearest lattice point. Every candidate the harness evaluates passes
	/// through here, so a search can never report a value the declaration
	/// forbids.
	pub fn snap(&self, value: f64) -> f64 {
		let v = value.clamp(self.min, self.max);
		match self.step {
			Some(s) if s > 0.0 => {
				let k = ((v - self.min) / s).round().max(0.0);
				(self.min + k * s).min(self.max)
			}
			_ => v,
		}
	}
}

/// Which way is better for an objective.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Sense {
	/// Lower is better (mass, cost, compliance).
	Minimize,
	/// Higher is better (stiffness, inertia, margin).
	Maximize,
}

/// A declared objective: a name the evaluator must return, plus its [`Sense`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Objective {
	/// Objective name — the key the evaluator must fill in [`Evaluation`].
	pub name: String,
	/// Minimize or maximize.
	pub sense: Sense,
}

impl Objective {
	/// An objective to minimize.
	pub fn minimize(name: &str) -> Self {
		Self { name: name.to_string(), sense: Sense::Minimize }
	}

	/// An objective to maximize.
	pub fn maximize(name: &str) -> Self {
		Self { name: name.to_string(), sense: Sense::Maximize }
	}
}

/// The admissible band of a constraint. All bounds are **inclusive**: a design
/// sitting exactly on the bound is FEASIBLE (that is where constrained optima
/// live, and nudging it out with an epsilon would be a silent lie).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ConstraintKind {
	/// `value ≤ bound`.
	LessThan(f64),
	/// `value ≥ bound`.
	GreaterThan(f64),
	/// `lo ≤ value ≤ hi`.
	Between(f64, f64),
}

/// A declared constraint on a named quantity the evaluator returns.
#[derive(Clone, Debug, PartialEq)]
pub struct Constraint {
	/// Constrained quantity's name — the key the evaluator must fill.
	pub name: String,
	/// The admissible band.
	pub kind: ConstraintKind,
}

impl Constraint {
	/// `value ≤ bound`.
	pub fn less_than(name: &str, bound: f64) -> Self {
		Self { name: name.to_string(), kind: ConstraintKind::LessThan(bound) }
	}

	/// `value ≥ bound`.
	pub fn greater_than(name: &str, bound: f64) -> Self {
		Self { name: name.to_string(), kind: ConstraintKind::GreaterThan(bound) }
	}

	/// `lo ≤ value ≤ hi`.
	pub fn between(name: &str, lo: f64, hi: f64) -> Self {
		Self { name: name.to_string(), kind: ConstraintKind::Between(lo, hi) }
	}

	/// Distance from `value` to the admissible band, in the quantity's own
	/// units — exactly `0.0` when satisfied (including on the bound), so
	/// feasibility needs no epsilon.
	pub fn violation(&self, value: f64) -> f64 {
		match self.kind {
			ConstraintKind::LessThan(b) => (value - b).max(0.0),
			ConstraintKind::GreaterThan(b) => (b - value).max(0.0),
			ConstraintKind::Between(lo, hi) => (lo - value).max(value - hi).max(0.0),
		}
	}

	/// Human phrasing of the band, e.g. `"wants ≤ 10.000000"`.
	pub fn describe(&self) -> String {
		match self.kind {
			ConstraintKind::LessThan(b) => format!("wants ≤ {b:.6}"),
			ConstraintKind::GreaterThan(b) => format!("wants ≥ {b:.6}"),
			ConstraintKind::Between(lo, hi) => format!("wants {lo:.6} ≤ · ≤ {hi:.6}"),
		}
	}
}

/// What the user's evaluator returns for one design point: the value of every
/// declared objective and of every constrained quantity.
///
/// Every declared name must be present — a missing one is a loud
/// [`StudyError::MissingValue`], never a zero silently substituted. Non-finite
/// values are refused too ([`StudyError::NonFiniteValue`]): a NaN objective
/// would make the ranking order-dependent.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Evaluation {
	/// Objective name → value.
	pub objectives: BTreeMap<String, f64>,
	/// Constrained-quantity name → value.
	pub constraints: BTreeMap<String, f64>,
}

impl Evaluation {
	/// An empty evaluation to fill with [`objective`](Self::objective) /
	/// [`constraint`](Self::constraint).
	pub fn new() -> Self {
		Self::default()
	}

	/// Record an objective value (builder style).
	pub fn objective(mut self, name: &str, value: f64) -> Self {
		self.objectives.insert(name.to_string(), value);
		self
	}

	/// Record a constrained quantity's value (builder style).
	pub fn constraint(mut self, name: &str, value: f64) -> Self {
		self.constraints.insert(name.to_string(), value);
		self
	}
}

// ---------------------------------------------------------------------------
// Results
// ---------------------------------------------------------------------------

/// One evaluated design point, kept in the report **whether or not it is
/// feasible** — the near-miss designs are half the value of a study.
#[derive(Clone, Debug, PartialEq)]
pub struct EvaluationRecord {
	/// Position in [`StudyReport::evaluations`] (also the evaluation order).
	pub index: usize,
	/// The point that was evaluated (already snapped into the declared bounds).
	pub params: Params,
	/// Declared objectives, by name.
	pub objectives: BTreeMap<String, f64>,
	/// Declared constrained quantities, by name.
	pub constraints: BTreeMap<String, f64>,
	/// `violation == 0.0` — inclusive bounds, so on-the-bound is feasible.
	pub feasible: bool,
	/// Sum of every constraint's [`Constraint::violation`], in mixed units:
	/// a *relative* ranking of how badly a point misses, not a metric.
	pub violation: f64,
}

/// Which search produced a report.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Strategy {
	/// Exhaustive sweep of the declared lattice ([`Study::full_factorial`]).
	FullFactorial,
	/// Derivative-free axis-aligned poll ([`Study::pattern_search`]).
	PatternSearch,
}

impl Strategy {
	/// Stable lowercase label used in reports and canonical dumps.
	pub fn label(self) -> &'static str {
		match self {
			Strategy::FullFactorial => "full_factorial",
			Strategy::PatternSearch => "pattern_search",
		}
	}
}

/// Knobs of [`Study::pattern_search`]. All deterministic; none is a seed,
/// because there is nothing random to seed.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SearchOptions {
	/// Hard cap on evaluator calls (the search stops with
	/// `stop_reason == "evaluation_budget"`).
	pub max_evaluations: usize,
	/// Stop once every continuous variable's poll step is below this (absolute,
	/// in the variable's own units).
	pub step_tolerance: f64,
	/// Step multiplier applied to continuous variables after a failed poll.
	pub shrink: f64,
	/// Initial poll step of a continuous variable as a fraction of its range
	/// (`max − min`). Discretized variables always poll at their own `step`.
	pub initial_step_fraction: f64,
}

impl Default for SearchOptions {
	fn default() -> Self {
		Self { max_evaluations: 400, step_tolerance: 1e-4, shrink: 0.5, initial_step_fraction: 0.25 }
	}
}

/// Why a study refused. Every variant carries the numbers needed to fix the
/// study; none of them is a warning a caller can read past.
#[derive(Clone, Debug, PartialEq)]
pub enum StudyError {
	/// The declaration itself is unusable (no variables, no objectives,
	/// duplicate names, inverted or non-finite bounds, non-positive step).
	BadDeclaration {
		/// What exactly is wrong.
		reason: String,
	},
	/// The exhaustive sweep would exceed [`Study::evaluation_cap`].
	DesignSpaceTooLarge {
		/// Lattice size the sweep would need.
		combinations: usize,
		/// The configured cap.
		cap: usize,
	},
	/// A report was queried for an objective that was never declared.
	UnknownObjective {
		/// The requested name.
		name: String,
		/// The names that do exist.
		declared: Vec<String>,
	},
	/// The evaluator omitted a declared objective/constraint at some point.
	MissingValue {
		/// The point being evaluated.
		params: Params,
		/// `"objective"` or `"constraint"`.
		kind: &'static str,
		/// The declared name that was missing.
		name: String,
		/// What the evaluator did return.
		returned: Vec<String>,
	},
	/// The evaluator returned a NaN or infinite value — ranking would become
	/// order-dependent, so the study refuses instead of guessing.
	NonFiniteValue {
		/// The point being evaluated.
		params: Params,
		/// `"objective"` or `"constraint"`.
		kind: &'static str,
		/// The offending name.
		name: String,
		/// The offending value.
		value: f64,
	},
	/// A pattern-search start point named a variable outside its declared
	/// bounds (snapping it silently would hide a typo).
	StartOutOfBounds {
		/// Variable name.
		var: String,
		/// The value supplied.
		value: f64,
		/// Declared lower bound.
		min: f64,
		/// Declared upper bound.
		max: f64,
	},
	/// Nothing sampled satisfies the constraints, so there is no best design to
	/// return — reported with the closest miss so the refusal is actionable.
	NoFeasibleDesign {
		/// How many points were evaluated.
		evaluated: usize,
		/// Index of the least-violating point.
		closest_index: usize,
		/// Its parameters.
		closest_params: Params,
		/// Its total violation.
		closest_violation: f64,
		/// The single worst constraint at that point, phrased for humans.
		worst: String,
	},
	/// **The safety net firing.** Re-evaluating the winning design did not
	/// reproduce the value recorded during the search: the evaluator is not a
	/// pure function of its parameters (or something cached went stale), so the
	/// study's optimum is not reproducible and must not be quoted.
	ImpureEvaluator {
		/// The winning point that was re-evaluated.
		params: Params,
		/// `"objective"` or `"constraint"`.
		kind: &'static str,
		/// Which value disagreed.
		name: String,
		/// The value recorded during the search.
		recorded: f64,
		/// The value the re-run produced.
		reevaluated: f64,
		/// The relative tolerance in force (0.0 = bit-identical required).
		tolerance: f64,
	},
}

impl fmt::Display for StudyError {
	fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
		match self {
			StudyError::BadDeclaration { reason } => write!(f, "unusable study declaration: {reason}"),
			StudyError::DesignSpaceTooLarge { combinations, cap } => write!(
				f,
				"exhaustive sweep needs {combinations} evaluations, above the cap of {cap} — coarsen a variable's \
				 step, lower grid_levels, raise evaluation_cap, or use pattern_search"
			),
			StudyError::UnknownObjective { name, declared } => {
				write!(f, "no objective '{name}' in this study — declared: {}", declared.join(", "))
			}
			StudyError::MissingValue { params, kind, name, returned } => write!(
				f,
				"evaluator returned no {kind} '{name}' at [{}] — it returned: {}",
				fmt_params(params),
				if returned.is_empty() { "nothing".to_string() } else { returned.join(", ") }
			),
			StudyError::NonFiniteValue { params, kind, name, value } => {
				write!(f, "evaluator returned non-finite {kind} '{name}' = {value} at [{}]", fmt_params(params))
			}
			StudyError::StartOutOfBounds { var, value, min, max } => {
				write!(f, "start point puts '{var}' at {value}, outside its declared [{min}, {max}]")
			}
			StudyError::NoFeasibleDesign { evaluated, closest_index, closest_params, closest_violation, worst } => {
				write!(
					f,
					"no feasible design in {evaluated} evaluations: closest miss is #{closest_index} at [{}] with \
					 total constraint violation {closest_violation:.6} ({worst}) — best() never returns an infeasible \
					 design; relax a bound or widen a design variable",
					fmt_params(closest_params)
				)
			}
			StudyError::ImpureEvaluator { params, kind, name, recorded, reevaluated, tolerance } => write!(
				f,
				"IMPURE EVALUATOR: re-evaluating the winning design at [{}] changed {kind} '{name}' from {recorded:?} \
				 (recorded during the search) to {reevaluated:?} (re-run), |Δ| = {:e}, tolerance {tolerance:e} — the \
				 reported optimum is NOT reproducible and must not be quoted; the evaluator must be a pure function \
				 of its parameters (no clock, no unseeded randomness, no mutable capture, no stale cache)",
				fmt_params(params),
				(recorded - reevaluated).abs()
			),
		}
	}
}

impl std::error::Error for StudyError {}

/// `name=value, name=value` in name order.
fn fmt_params(p: &Params) -> String {
	p.iter().map(|(k, v)| format!("{k}={v:.6}")).collect::<Vec<_>>().join(", ")
}

// ---------------------------------------------------------------------------
// Study
// ---------------------------------------------------------------------------

/// A declared design-space study: variables, objectives, constraints and the
/// evaluator that turns a point into numbers.
///
/// The evaluator is arbitrary user code — geometry, physics, a shell-out — and
/// the harness assumes nothing about it except **purity** (see the module
/// docs). Build with [`Study::new`] plus the builder methods, then run a
/// strategy; the returned [`StudyReport`] borrows the study so it can
/// re-evaluate the winner honestly.
pub struct Study<'e> {
	vars: Vec<DesignVar>,
	objectives: Vec<Objective>,
	constraints: Vec<Constraint>,
	grid_levels: usize,
	cap: usize,
	reeval_tol: f64,
	evaluator: Box<dyn Fn(&Params) -> Evaluation + Sync + 'e>,
}

impl fmt::Debug for Study<'_> {
	fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
		f.debug_struct("Study")
			.field("vars", &self.vars)
			.field("objectives", &self.objectives)
			.field("constraints", &self.constraints)
			.field("grid_levels", &self.grid_levels)
			.field("evaluation_cap", &self.cap)
			.field("reeval_tolerance", &self.reeval_tol)
			.finish_non_exhaustive()
	}
}

impl<'e> Study<'e> {
	/// A study driven by `evaluator`, which **must be pure** — a function of
	/// its [`Params`] alone. It is called from several threads at once
	/// ([`kernel_core::par::par_map_indexed`]), hence the `Sync` bound; results
	/// are collected by index, so scheduling cannot change the report.
	pub fn new(evaluator: impl Fn(&Params) -> Evaluation + Sync + 'e) -> Self {
		Self {
			vars: Vec::new(),
			objectives: Vec::new(),
			constraints: Vec::new(),
			grid_levels: 5,
			cap: 100_000,
			reeval_tol: 0.0,
			evaluator: Box::new(evaluator),
		}
	}

	/// Declare a design variable (builder style; declaration order fixes the
	/// sweep's odometer order and the poll order).
	pub fn var(mut self, v: DesignVar) -> Self {
		self.vars.push(v);
		self
	}

	/// Declare an objective (builder style).
	pub fn objective(mut self, o: Objective) -> Self {
		self.objectives.push(o);
		self
	}

	/// Shorthand for `objective(Objective::minimize(name))`.
	pub fn minimize(self, name: &str) -> Self {
		self.objective(Objective::minimize(name))
	}

	/// Shorthand for `objective(Objective::maximize(name))`.
	pub fn maximize(self, name: &str) -> Self {
		self.objective(Objective::maximize(name))
	}

	/// Declare a constraint (builder style).
	pub fn constrain(mut self, c: Constraint) -> Self {
		self.constraints.push(c);
		self
	}

	/// Levels per **continuous** variable in the exhaustive sweep (default 5,
	/// minimum 2 — both endpoints always included). Discretized variables
	/// ignore this and use their own lattice.
	pub fn grid_levels(mut self, n: usize) -> Self {
		self.grid_levels = n;
		self
	}

	/// Refuse an exhaustive sweep larger than `cap` points (default 100 000) —
	/// a stated cost ceiling instead of an accidental overnight run.
	pub fn evaluation_cap(mut self, cap: usize) -> Self {
		self.cap = cap;
		self
	}

	/// Relative tolerance of the honest re-evaluation in [`StudyReport::best`].
	///
	/// **Default `0.0` = bit-identical**, because determinism is a hard contract
	/// here (`docs/NUMERICS.md`) and the exact B-rep pipeline honours it.
	/// Raising it is a deliberate, documented weakening for evaluators that
	/// genuinely cannot be bit-stable (an external solver with a wall-clock
	/// iteration budget, say) — and it weakens exactly the check that catches a
	/// lying optimum, so say so wherever you set it.
	pub fn reeval_tolerance(mut self, relative: f64) -> Self {
		self.reeval_tol = relative;
		self
	}

	/// The declared variables, in declaration order.
	pub fn vars(&self) -> &[DesignVar] {
		&self.vars
	}

	/// The declared objectives, in declaration order.
	pub fn objectives(&self) -> &[Objective] {
		&self.objectives
	}

	/// The declared constraints, in declaration order.
	pub fn constraints(&self) -> &[Constraint] {
		&self.constraints
	}

	fn check(&self) -> Result<(), StudyError> {
		let bad = |reason: String| Err(StudyError::BadDeclaration { reason });
		if self.vars.is_empty() {
			return bad("no design variables declared".to_string());
		}
		if self.objectives.is_empty() {
			return bad("no objectives declared".to_string());
		}
		for (i, v) in self.vars.iter().enumerate() {
			if self.vars[..i].iter().any(|w| w.name == v.name) {
				return bad(format!("duplicate design variable '{}'", v.name));
			}
			if !v.min.is_finite() || !v.max.is_finite() {
				return bad(format!("variable '{}' has non-finite bounds [{}, {}]", v.name, v.min, v.max));
			}
			if v.max < v.min {
				return bad(format!("variable '{}' has max {} below min {}", v.name, v.max, v.min));
			}
			if let Some(s) = v.step {
				if !s.is_finite() || s <= 0.0 {
					return bad(format!("variable '{}' has non-positive step {s}", v.name));
				}
			}
		}
		for (i, o) in self.objectives.iter().enumerate() {
			if self.objectives[..i].iter().any(|p| p.name == o.name) {
				return bad(format!("duplicate objective '{}'", o.name));
			}
		}
		for (i, c) in self.constraints.iter().enumerate() {
			if self.constraints[..i].iter().any(|d| d.name == c.name) {
				return bad(format!("duplicate constraint '{}'", c.name));
			}
			if let ConstraintKind::Between(lo, hi) = c.kind {
				if hi < lo {
					return bad(format!("constraint '{}' has hi {hi} below lo {lo}", c.name));
				}
			}
		}
		Ok(())
	}

	/// Evaluate ONE point through the declared contract: every declared
	/// objective/constraint must come back finite, and the constraint violation
	/// is summed in declaration order. Public so a campaign can re-check a
	/// shipped design against the same contract the study used.
	pub fn evaluate_at(&self, params: &Params) -> Result<EvaluationRecord, StudyError> {
		self.record(0, params)
	}

	fn record(&self, index: usize, params: &Params) -> Result<EvaluationRecord, StudyError> {
		let ev = (self.evaluator)(params);
		let mut objectives = BTreeMap::new();
		for o in &self.objectives {
			let Some(&v) = ev.objectives.get(&o.name) else {
				return Err(StudyError::MissingValue {
					params: params.clone(),
					kind: "objective",
					name: o.name.clone(),
					returned: ev.objectives.keys().cloned().collect(),
				});
			};
			if !v.is_finite() {
				return Err(StudyError::NonFiniteValue {
					params: params.clone(),
					kind: "objective",
					name: o.name.clone(),
					value: v,
				});
			}
			objectives.insert(o.name.clone(), v);
		}
		let mut constraints = BTreeMap::new();
		let mut violation = 0.0f64;
		for c in &self.constraints {
			let Some(&v) = ev.constraints.get(&c.name) else {
				return Err(StudyError::MissingValue {
					params: params.clone(),
					kind: "constraint",
					name: c.name.clone(),
					returned: ev.constraints.keys().cloned().collect(),
				});
			};
			if !v.is_finite() {
				return Err(StudyError::NonFiniteValue {
					params: params.clone(),
					kind: "constraint",
					name: c.name.clone(),
					value: v,
				});
			}
			constraints.insert(c.name.clone(), v);
			violation += c.violation(v);
		}
		Ok(EvaluationRecord { index, params: params.clone(), objectives, constraints, feasible: violation == 0.0, violation })
	}

	/// Evaluate a batch in parallel, results in index order; the FIRST error in
	/// index order wins, so a failing study fails the same way every run.
	fn evaluate_batch(&self, offset: usize, points: &[Params]) -> Result<Vec<EvaluationRecord>, StudyError> {
		let results = par_map_indexed(points, |i, p| self.record(offset + i, p));
		let mut out = Vec::with_capacity(results.len());
		for r in results {
			out.push(r?);
		}
		Ok(out)
	}

	/// **Exhaustive sweep** of the declared lattice — `∏ levels(var)`
	/// evaluations, the honest baseline. Points are enumerated as an odometer
	/// with the LAST declared variable varying fastest, so the evaluation order
	/// is a pure function of the declaration.
	///
	/// This is the only strategy whose sample is *balanced*, which is what makes
	/// [`StudyReport::sensitivity`] a fair main-effect screen (see
	/// [`StudyReport::sensitivity_balanced`]).
	#[doc(alias = "grid")]
	pub fn full_factorial(&self) -> Result<StudyReport<'_>, StudyError> {
		self.check()?;
		let levels: Vec<Vec<f64>> = self.vars.iter().map(|v| v.levels(self.grid_levels)).collect();
		let total = levels.iter().try_fold(1usize, |acc, l| acc.checked_mul(l.len())).unwrap_or(usize::MAX);
		if total > self.cap {
			return Err(StudyError::DesignSpaceTooLarge { combinations: total, cap: self.cap });
		}
		let mut points = Vec::with_capacity(total);
		for k in 0..total {
			let mut rem = k;
			let mut p = Params::new();
			for (v, ls) in self.vars.iter().zip(&levels).rev() {
				p.insert(v.name.clone(), ls[rem % ls.len()]);
				rem /= ls.len();
			}
			points.push(p);
		}
		let records = self.evaluate_batch(0, &points)?;
		Ok(self.report(Strategy::FullFactorial, records, "exhaustive", true))
	}

	/// **Derivative-free local refinement** from `start`: a compass (coordinate)
	/// pattern search.
	///
	/// Each poll evaluates the ≤ `2k` axis neighbours `x ± step_v` (clipped and
	/// snapped into the declaration, duplicates dropped) **in parallel**, then
	/// moves to the best one if it beats the incumbent, else shrinks every
	/// continuous step by [`SearchOptions::shrink`]. Discretized variables never
	/// poll below their own lattice step; a failed poll marks them exhausted
	/// (reset the moment any move succeeds). The search stops on
	/// `"step_tolerance"`, `"no_improving_move"` (a fully polled discrete
	/// space), or `"evaluation_budget"` — recorded in
	/// [`StudyReport::stop_reason`].
	///
	/// Candidates are ranked **feasibility-first** (see [`better`]), so the
	/// incumbent never leaves the feasible region once it is in it.
	///
	/// Cost ≈ `2k·log2(step₀/tol)` evaluations for `k` continuous variables:
	/// dramatically cheaper than the exhaustive sweep, and the tests pin that
	/// claim with measured counts.
	///
	/// **Stated limitation**: the poll is axis-aligned, so on an *active*
	/// constraint whose boundary is not axis-aligned every neighbour is either
	/// infeasible or worse and the search stalls on the boundary short of the
	/// constrained optimum. Use the sweep when a constraint is expected to be
	/// active — this is pinned as a limitation in `tests/optimize.rs`, not
	/// papered over.
	#[doc(alias = "coordinate_descent")]
	pub fn pattern_search(&self, start: &Params, opts: SearchOptions) -> Result<StudyReport<'_>, StudyError> {
		self.check()?;
		let objective = self.objectives[0].clone();
		// The start point must be inside the declaration: snapping a typo
		// silently would hide it.
		let mut point = Params::new();
		for v in &self.vars {
			let x = start.get(&v.name).copied().unwrap_or(0.5 * (v.min + v.max));
			if !x.is_finite() || x < v.min || x > v.max {
				return Err(StudyError::StartOutOfBounds { var: v.name.clone(), value: x, min: v.min, max: v.max });
			}
			point.insert(v.name.clone(), v.snap(x));
		}

		let mut steps: Vec<f64> = self
			.vars
			.iter()
			.map(|v| match v.step {
				Some(s) => s,
				None => (v.max - v.min) * opts.initial_step_fraction,
			})
			.collect();
		let mut exhausted: Vec<bool> = vec![false; self.vars.len()];
		let has_continuous = self.vars.iter().any(|v| v.step.is_none());

		let mut records = self.evaluate_batch(0, std::slice::from_ref(&point))?;
		let mut current = 0usize;
		let stop_reason = loop {
			let converged = self.vars.iter().enumerate().all(|(i, v)| match v.step {
				Some(_) => exhausted[i],
				None => steps[i] < opts.step_tolerance,
			});
			if converged {
				break if has_continuous { "step_tolerance" } else { "no_improving_move" };
			}
			// Poll set: a pure function of (point, steps, declaration).
			let mut poll: Vec<Params> = Vec::new();
			for (i, v) in self.vars.iter().enumerate() {
				if exhausted[i] {
					continue;
				}
				for sign in [1.0f64, -1.0] {
					let mut cand = point.clone();
					cand.insert(v.name.clone(), v.snap(point[&v.name] + sign * steps[i]));
					if cand != point && !poll.contains(&cand) {
						poll.push(cand);
					}
				}
			}
			if poll.is_empty() {
				// Every neighbour collapsed onto the incumbent (bounds or
				// lattice): shrink and retry, or declare the poll finished.
				self.shrink_steps(&mut steps, &mut exhausted, &opts);
				if !has_continuous {
					break "no_improving_move";
				}
				continue;
			}
			if records.len() + poll.len() > opts.max_evaluations {
				break "evaluation_budget";
			}
			let first = records.len();
			let batch = self.evaluate_batch(first, &poll)?;
			records.extend(batch);
			// Best of the poll, ties to the lowest index (declaration order).
			let mut best = first;
			for i in first + 1..records.len() {
				if better(&records[i], &records[best], &objective.name, objective.sense) {
					best = i;
				}
			}
			if better(&records[best], &records[current], &objective.name, objective.sense) {
				current = best;
				point = records[best].params.clone();
				exhausted.iter_mut().for_each(|e| *e = false);
			} else {
				self.shrink_steps(&mut steps, &mut exhausted, &opts);
				if !has_continuous {
					break "no_improving_move";
				}
			}
		};
		Ok(self.report(Strategy::PatternSearch, records, stop_reason, false))
	}

	/// Shrink continuous steps and exhaust discrete ones after a failed poll.
	fn shrink_steps(&self, steps: &mut [f64], exhausted: &mut [bool], opts: &SearchOptions) {
		for (i, v) in self.vars.iter().enumerate() {
			match v.step {
				Some(_) => exhausted[i] = true,
				None => steps[i] *= opts.shrink,
			}
		}
	}

	fn report<'s>(
		&'s self,
		strategy: Strategy,
		evaluations: Vec<EvaluationRecord>,
		stop_reason: &str,
		balanced: bool,
	) -> StudyReport<'s> {
		let feasible_count = evaluations.iter().filter(|r| r.feasible).count();
		let mut best_per_objective = BTreeMap::new();
		for o in &self.objectives {
			let mut best: Option<usize> = None;
			for r in evaluations.iter().filter(|r| r.feasible) {
				match best {
					Some(b) => {
						if better(r, &evaluations[b], &o.name, o.sense) {
							best = Some(r.index);
						}
					}
					None => best = Some(r.index),
				}
			}
			best_per_objective.insert(o.name.clone(), best);
		}
		let front = pareto_front(&evaluations, &self.objectives);
		let sensitivity = main_effects(&self.vars, &self.objectives, &evaluations);
		StudyReport {
			strategy,
			evaluations,
			feasible_count,
			best_per_objective,
			pareto_front: front,
			sensitivity,
			sensitivity_balanced: balanced,
			stop_reason: stop_reason.to_string(),
			study: self,
		}
	}
}

/// Strict "a is better than b" under the harness's **feasibility-first** rule:
/// a feasible point beats any infeasible one; two feasible points compare on the
/// objective under its sense; two infeasible ones compare on total violation
/// (less bad wins). Never `true` for equal points, so ties resolve to whichever
/// was evaluated first — the deterministic tiebreak.
pub fn better(a: &EvaluationRecord, b: &EvaluationRecord, objective: &str, sense: Sense) -> bool {
	match (a.feasible, b.feasible) {
		(true, false) => true,
		(false, true) => false,
		(true, true) => {
			let (va, vb) = (a.objectives[objective], b.objectives[objective]);
			match sense {
				Sense::Minimize => va < vb,
				Sense::Maximize => va > vb,
			}
		}
		(false, false) => a.violation < b.violation,
	}
}

// ---------------------------------------------------------------------------
// Pareto + sensitivity
// ---------------------------------------------------------------------------

/// Non-dominated (Pareto-optimal) indices among the **feasible** records, in
/// ascending index order.
///
/// `a` dominates `b` when `a` is at least as good on every objective (under
/// that objective's [`Sense`]) and strictly better on at least one. Duplicated
/// objective vectors therefore do not dominate each other and both stay on the
/// front. Infeasible records are excluded from the front but stay in the report
/// — a front of designs that cannot be built would be worthless.
///
/// Cost `O(n²·m)`; this is an **extraction over a given sample**, not a search:
/// the front can only be as good as the points it is handed.
pub fn pareto_front(records: &[EvaluationRecord], objectives: &[Objective]) -> Vec<usize> {
	let feasible: Vec<&EvaluationRecord> = records.iter().filter(|r| r.feasible).collect();
	let dominates = |a: &EvaluationRecord, b: &EvaluationRecord| {
		let mut strictly = false;
		for o in objectives {
			let (va, vb) = (a.objectives[&o.name], b.objectives[&o.name]);
			let (as_good, better_than) = match o.sense {
				Sense::Minimize => (va <= vb, va < vb),
				Sense::Maximize => (va >= vb, va > vb),
			};
			if !as_good {
				return false;
			}
			strictly |= better_than;
		}
		strictly
	};
	feasible
		.iter()
		.filter(|r| !feasible.iter().any(|other| dominates(other, r)))
		.map(|r| r.index)
		.collect()
}

/// Range-normalized **main effect** of every variable on every objective.
///
/// Method: group the sampled evaluations by the variable's level, average each
/// objective within a level, and report
/// `(max level mean − min level mean) / (max objective − min objective)` over
/// the whole sample — a dimensionless 0…1 screening indicator.
///
/// Limits, stated because they matter: it is a **screening indicator, not a
/// derivative** — (1) it averages over all other variables, so it is blind to
/// interactions (a variable that only matters when another is high can read
/// near 0); (2) it is only fair on a *balanced* sample, i.e. the exhaustive
/// sweep — on a pattern-search sample the levels are visited unevenly and the
/// number is a crude hint at best ([`StudyReport::sensitivity_balanced`] says
/// which you have); (3) it describes the sampled BOX, not the neighbourhood of
/// the optimum; (4) it is normalized by this study's own objective range, so it
/// compares variables within one study and nothing across studies.
fn main_effects(
	vars: &[DesignVar],
	objectives: &[Objective],
	records: &[EvaluationRecord],
) -> BTreeMap<String, BTreeMap<String, f64>> {
	let mut out = BTreeMap::new();
	for v in vars {
		// Group indices by the variable's exact level (bit key, first-appearance
		// order — deterministic without hashing).
		let mut keys: Vec<u64> = Vec::new();
		let mut groups: Vec<Vec<usize>> = Vec::new();
		for (i, r) in records.iter().enumerate() {
			let Some(x) = r.params.get(&v.name) else { continue };
			let k = x.to_bits();
			match keys.iter().position(|&kk| kk == k) {
				Some(j) => groups[j].push(i),
				None => {
					keys.push(k);
					groups.push(vec![i]);
				}
			}
		}
		let mut per_objective = BTreeMap::new();
		for o in objectives {
			let value = |i: usize| records[i].objectives.get(&o.name).copied().unwrap_or(0.0);
			let mut lo = f64::INFINITY;
			let mut hi = f64::NEG_INFINITY;
			for i in 0..records.len() {
				lo = lo.min(value(i));
				hi = hi.max(value(i));
			}
			let range = hi - lo;
			let mut mean_lo = f64::INFINITY;
			let mut mean_hi = f64::NEG_INFINITY;
			for g in &groups {
				let mut sum = 0.0;
				for &i in g {
					sum += value(i);
				}
				let mean = sum / g.len() as f64;
				mean_lo = mean_lo.min(mean);
				mean_hi = mean_hi.max(mean);
			}
			let effect =
				if groups.len() >= 2 && range > 0.0 && mean_hi.is_finite() { (mean_hi - mean_lo) / range } else { 0.0 };
			per_objective.insert(o.name.clone(), effect);
		}
		out.insert(v.name.clone(), per_objective);
	}
	out
}

// ---------------------------------------------------------------------------
// Report
// ---------------------------------------------------------------------------

/// Everything a study measured, plus the machinery to quote it honestly.
///
/// The report borrows its [`Study`], which is what lets [`best`](Self::best)
/// re-run the evaluator on the winning point instead of trusting a cached
/// number.
pub struct StudyReport<'s> {
	/// Which search produced this report.
	pub strategy: Strategy,
	/// Every evaluated point in evaluation order — **infeasible ones included**
	/// and flagged by [`EvaluationRecord::feasible`].
	pub evaluations: Vec<EvaluationRecord>,
	/// How many of them satisfy every constraint.
	pub feasible_count: usize,
	/// Objective name → index of the best FEASIBLE record, or `None` when
	/// nothing feasible was sampled. A recorded winner: nothing here may be
	/// quoted until [`best`](Self::best) has re-evaluated it.
	pub best_per_objective: BTreeMap<String, Option<usize>>,
	/// Indices of the non-dominated feasible records ([`pareto_front`]).
	pub pareto_front: Vec<usize>,
	/// Variable → objective → range-normalized main effect. **Method and
	/// limits**: see the module's sensitivity notes — level means differenced
	/// and divided by the sampled objective range; a screening indicator, not a
	/// derivative, blind to interactions, fair only on a balanced sample.
	pub sensitivity: BTreeMap<String, BTreeMap<String, f64>>,
	/// Whether [`sensitivity`](Self::sensitivity) came from a balanced
	/// (exhaustive) sample. `false` ⇒ read it as a hint only.
	pub sensitivity_balanced: bool,
	/// `"exhaustive"`, `"step_tolerance"`, `"no_improving_move"` or
	/// `"evaluation_budget"`.
	pub stop_reason: String,
	study: &'s Study<'s>,
}

impl fmt::Debug for StudyReport<'_> {
	fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
		f.debug_struct("StudyReport")
			.field("strategy", &self.strategy.label())
			.field("evaluations", &self.evaluations.len())
			.field("feasible_count", &self.feasible_count)
			.field("best_per_objective", &self.best_per_objective)
			.field("pareto_front", &self.pareto_front)
			.field("sensitivity", &self.sensitivity)
			.field("sensitivity_balanced", &self.sensitivity_balanced)
			.field("stop_reason", &self.stop_reason)
			.finish_non_exhaustive()
	}
}

impl StudyReport<'_> {
	/// Number of evaluator calls the search made (the honest cost of this run;
	/// [`best`](Self::best) adds exactly one more).
	pub fn evaluation_count(&self) -> usize {
		self.evaluations.len()
	}

	/// Evaluations that violate at least one constraint — retained on purpose.
	pub fn infeasible(&self) -> impl Iterator<Item = &EvaluationRecord> {
		self.evaluations.iter().filter(|r| !r.feasible)
	}

	/// The study this report came from (declaration + evaluator).
	pub fn study(&self) -> &Study<'_> {
		self.study
	}

	/// **The honest winner.** Picks the best FEASIBLE point for `objective`,
	/// then calls the evaluator ONE more time on its parameters and returns
	/// [`BestDesign::value`] from that re-run.
	///
	/// Refuses loudly instead of guessing:
	/// * [`StudyError::NoFeasibleDesign`] when nothing sampled is feasible —
	///   carrying the closest miss and its violation, so the refusal tells you
	///   what to relax;
	/// * [`StudyError::ImpureEvaluator`] when the re-run disagrees with the
	///   recorded value beyond [`Study::reeval_tolerance`] (default:
	///   bit-identical). That is the safety net for a non-pure evaluator or a
	///   stale cache, and a study whose optimum does not reproduce has no
	///   optimum.
	pub fn best(&self, objective: &str) -> Result<BestDesign, StudyError> {
		let Some(obj) = self.study.objectives.iter().find(|o| o.name == objective) else {
			return Err(StudyError::UnknownObjective {
				name: objective.to_string(),
				declared: self.study.objectives.iter().map(|o| o.name.clone()).collect(),
			});
		};
		let Some(Some(idx)) = self.best_per_objective.get(objective).copied() else {
			return Err(self.no_feasible_error());
		};
		let recorded = &self.evaluations[idx];
		let rerun = self.study.record(recorded.index, &recorded.params)?;
		let tol = self.study.reeval_tol;
		let mismatched = |a: f64, b: f64| (a - b).abs() > tol * a.abs().max(b.abs());
		for (name, &was) in &recorded.objectives {
			let now = rerun.objectives[name];
			if mismatched(was, now) {
				return Err(StudyError::ImpureEvaluator {
					params: recorded.params.clone(),
					kind: "objective",
					name: name.clone(),
					recorded: was,
					reevaluated: now,
					tolerance: tol,
				});
			}
		}
		for (name, &was) in &recorded.constraints {
			let now = rerun.constraints[name];
			if mismatched(was, now) {
				return Err(StudyError::ImpureEvaluator {
					params: recorded.params.clone(),
					kind: "constraint",
					name: name.clone(),
					recorded: was,
					reevaluated: now,
					tolerance: tol,
				});
			}
		}
		Ok(BestDesign {
			objective: objective.to_string(),
			sense: obj.sense,
			index: idx,
			params: recorded.params.clone(),
			value: rerun.objectives[objective],
			recorded_value: recorded.objectives[objective],
			objectives: rerun.objectives.clone(),
			constraints: rerun.constraints.clone(),
			violation: rerun.violation,
			evaluator_calls: self.evaluations.len() + 1,
		})
	}

	fn no_feasible_error(&self) -> StudyError {
		let mut closest: Option<&EvaluationRecord> = None;
		for r in &self.evaluations {
			if closest.is_none_or(|c| r.violation < c.violation) {
				closest = Some(r);
			}
		}
		let Some(c) = closest else {
			return StudyError::NoFeasibleDesign {
				evaluated: 0,
				closest_index: 0,
				closest_params: Params::new(),
				closest_violation: f64::INFINITY,
				worst: "nothing was evaluated".to_string(),
			};
		};
		let mut worst: Option<(&str, f64, f64, String)> = None;
		for con in &self.study.constraints {
			let v = c.constraints[&con.name];
			let viol = con.violation(v);
			if viol > worst.as_ref().map_or(0.0, |w| w.1) {
				worst = Some((&con.name, viol, v, con.describe()));
			}
		}
		let worst_text = match worst {
			Some((name, _, value, band)) => format!("'{name}' {band}, got {value:.6}"),
			None => "no constraint violated — the study declared none".to_string(),
		};
		StudyError::NoFeasibleDesign {
			evaluated: self.evaluations.len(),
			closest_index: c.index,
			closest_params: c.params.clone(),
			closest_violation: c.violation,
			worst: worst_text,
		}
	}

	/// Canonical text serialization of the whole report — every f64 as its
	/// **bit pattern**, every map in key order. Two runs of the same declaration
	/// must produce byte-identical output; diffing two of these localizes a
	/// determinism break to the exact evaluation and value.
	pub fn canonical(&self) -> String {
		let hex = |x: f64| format!("0x{:016x}", x.to_bits());
		let mut s = String::new();
		s.push_str("study\n");
		for v in &self.study.vars {
			s.push_str(&format!(
				"\tvar {} min={} max={} step={}\n",
				v.name,
				hex(v.min),
				hex(v.max),
				v.step.map(hex).unwrap_or_else(|| "none".to_string())
			));
		}
		for o in &self.study.objectives {
			let sense = match o.sense {
				Sense::Minimize => "minimize",
				Sense::Maximize => "maximize",
			};
			s.push_str(&format!("\tobjective {} {sense}\n", o.name));
		}
		for c in &self.study.constraints {
			let band = match c.kind {
				ConstraintKind::LessThan(b) => format!("lt {}", hex(b)),
				ConstraintKind::GreaterThan(b) => format!("gt {}", hex(b)),
				ConstraintKind::Between(lo, hi) => format!("between {} {}", hex(lo), hex(hi)),
			};
			s.push_str(&format!("\tconstraint {} {band}\n", c.name));
		}
		s.push_str(&format!("\tstrategy {} stop={}\n", self.strategy.label(), self.stop_reason));
		for r in &self.evaluations {
			s.push_str(&format!("eval {}", r.index));
			for (k, v) in &r.params {
				s.push_str(&format!(" {k}={}", hex(*v)));
			}
			for (k, v) in &r.objectives {
				s.push_str(&format!(" obj:{k}={}", hex(*v)));
			}
			for (k, v) in &r.constraints {
				s.push_str(&format!(" con:{k}={}", hex(*v)));
			}
			s.push_str(&format!(" feasible={} violation={}\n", u8::from(r.feasible), hex(r.violation)));
		}
		s.push_str(&format!("feasible_count {}\n", self.feasible_count));
		for (name, idx) in &self.best_per_objective {
			let which = idx.map(|i| i.to_string()).unwrap_or_else(|| "none".to_string());
			s.push_str(&format!("best {name} {which}\n"));
		}
		let front = self.pareto_front.iter().map(|i| i.to_string()).collect::<Vec<_>>().join(",");
		s.push_str(&format!("pareto {front}\n"));
		for (var, per) in &self.sensitivity {
			for (obj, e) in per {
				s.push_str(&format!("sens {var}/{obj} {}\n", hex(*e)));
			}
		}
		s.push_str(&format!("sensitivity_balanced {}\n", u8::from(self.sensitivity_balanced)));
		s
	}

	/// FNV-1a 64 digest of [`canonical`](Self::canonical) — a short number to
	/// print in a gate line or a README when quoting a study result.
	pub fn digest(&self) -> u64 {
		let mut h: u64 = 0xcbf2_9ce4_8422_2325;
		for b in self.canonical().as_bytes() {
			h ^= *b as u64;
			h = h.wrapping_mul(0x1000_0000_01b3);
		}
		h
	}
}

/// The winner of a study, with its objective value **taken from the honest
/// re-evaluation** — [`BestDesign::value`] is the number you are allowed to
/// quote; [`recorded_value`](Self::recorded_value) is what the search had
/// cached, kept only so the two can be shown to agree.
#[derive(Clone, Debug, PartialEq)]
pub struct BestDesign {
	/// Which objective this is the best of.
	pub objective: String,
	/// That objective's sense.
	pub sense: Sense,
	/// Index of the winning record in [`StudyReport::evaluations`].
	pub index: usize,
	/// The winning parameters.
	pub params: Params,
	/// **Re-evaluated** value of `objective` — quote this one.
	pub value: f64,
	/// The value recorded during the search (equal to `value` within
	/// [`Study::reeval_tolerance`], else `best()` would have refused).
	pub recorded_value: f64,
	/// All objectives from the re-evaluation.
	pub objectives: BTreeMap<String, f64>,
	/// All constrained quantities from the re-evaluation.
	pub constraints: BTreeMap<String, f64>,
	/// Total constraint violation of the re-evaluation — `0.0`, or `best()`
	/// would not have returned this point.
	pub violation: f64,
	/// Total evaluator calls charged to this answer: the search plus the one
	/// honest re-evaluation.
	pub evaluator_calls: usize,
}

// ---------------------------------------------------------------------------
// Campaign integration
// ---------------------------------------------------------------------------

/// Campaign gate: **the shipped design IS the study's chosen optimum** — the
/// idiom that turns a one-off optimization into a claim re-proven on every run,
/// exactly like every other campaign gate.
///
/// Runs [`StudyReport::best`] (so the winner is re-evaluated, and an impure
/// evaluator or an infeasible study FAILS the gate with its own loud message),
/// then checks every declared design variable of the shipped design against the
/// winner within `tol`. Prints through [`crate::campaign::gate`], folding the
/// verdict into `ok`, and returns the [`BestDesign`] so the campaign can quote
/// its re-evaluated numbers.
///
/// ```no_run
/// # use kernel_model::optimize::*;
/// # let study = Study::new(|_: &Params| Evaluation::new().objective("m", 1.0))
/// #     .var(DesignVar::stepped("wall", 1.6, 3.2, 0.4)).minimize("m");
/// # let report = study.full_factorial().unwrap();
/// let mut ok = true;
/// let shipped: Params = [("wall".to_string(), 2.0)].into_iter().collect();
/// let best = gate_study("shipped wall = study optimum", &report, "m", &shipped, 1e-9, &mut ok);
/// # let _ = best;
/// ```
pub fn gate_study(
	label: &str,
	report: &StudyReport<'_>,
	objective: &str,
	shipped: &Params,
	tol: f64,
	ok: &mut bool,
) -> Option<BestDesign> {
	let best = match report.best(objective) {
		Ok(b) => b,
		Err(e) => {
			crate::campaign::gate(label, false, format!("study refused: {e}"), ok);
			return None;
		}
	};
	// Report the WORST-matching variable, so one wrong dimension cannot hide
	// behind the others.
	let mut worst: Option<(String, f64, f64, f64)> = None;
	for v in report.study.vars() {
		let want = best.params[&v.name];
		let got = shipped.get(&v.name).copied().unwrap_or(f64::NAN);
		let d = (got - want).abs();
		if !worst.as_ref().is_some_and(|w| d <= w.3) {
			worst = Some((v.name.clone(), got, want, d));
		}
	}
	let (name, got, want, d) = worst.unwrap_or_else(|| (String::new(), 0.0, 0.0, 0.0));
	let pass = d <= tol;
	let detail = if pass {
		format!("{objective}={:.4} @{} evals", best.value, best.evaluator_calls)
	} else {
		format!("'{name}' ships {got:.4} vs optimum {want:.4} (Δ{d:.4} > {tol:.4})")
	};
	crate::campaign::gate(label, pass, detail, ok);
	Some(best)
}
