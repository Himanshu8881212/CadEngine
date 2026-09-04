# param_optimize — one derivative-free optimizer over ANY receipted analyzer

**Runner**: `tools/analyzers/param_optimize.py` ·
`python3 tools/analyzers/param_optimize.py <job.json> [--out PATH]`
**Gates**: `python3 tools/validation/param_optimize_validation.py` (four analytic
pins; hermetic, stdlib only — also runs under
`python3 tools/analyzer_registry.py --run-pins`) ·
`python3 tools/tests/param_optimize_drift_test.py` (the witness-selection drift
detector; needs the release `kernel-api` binary) ·
`python3 tools/tests/test_checkers.py` (its `test_param_optimize` block:
constraint-bound, quantization, path, refusal and expression-safety contracts).
**No benchmark gate suite is registered.**
**Status**: pinned, no gate suite **Tier**: Validated
(`python3 tools/analyzer_registry.py --tier param_optimize`; manifest
`tools/manifests/param_optimize.manifest.json` + pin
`tools/validation/param_optimize_validation.py` are both present).

This is not a solver and it has no physics of its own. It is the search loop that
turns "what should this dimension be" from an argument into a receipt. It never
touches geometry: it substitutes named numeric parameters into a template, runs
something that emits a receipt, reads a dotted expression out of that receipt,
and iterates — so the SAME optimizer drives a kernel work-order program, an ACE
physics runner, or an agent-derived model
(`tools/analyzers/derived_model.py`), and it is representation-agnostic by
construction (B-rep dimensions, implicit/TPMS constants, anything expressible as
ops).

The failure it exists to prevent is the hand-tuned dimension nobody can defend
later. The failures it can still commit — a local optimum, a discretised search
space, a witness edge that silently moved mid-sweep — are the reason most of the
receipt is diagnostics rather than the answer.

## What it actually computes

A scalarized cost, minimized by Nelder-Mead simplex over box-bounded parameters
(`tools/manifests/param_optimize.manifest.json`):

    min_x   sum_i s_i w_i f_i(x)                        # objectives, s_i = -1 when maximized
          + sum_t w_t ((g_t(x) - v_t) / tol_t)^2        # convergence targets
          + 10 (1 + |obj|) sum_c viol_c(x)              # constraint penalty

    viol = v/max - 1   (above a max)  |  min/v - 1   (below a min)

The violation is DIMENSIONLESS and relative, scaled by `10*(1+|objective|)` so a
10% violation dominates any objective magnitude. Zero and negative bounds are
first-class: `{"max": 0.0}` is the natural spelling of "no steep area" / "no
warnings" and no longer divides by zero, and a negative bound no longer inverts
the penalty sign (both pinned in `tools/tests/test_checkers.py`).

**Selection is FEASIBILITY-FIRST**: the reported best is the best candidate that
satisfied every constraint whenever one was ever seen. `constraint_ok: false`
means the whole search never found a feasible point, and the reported best is
then the least-bad infeasible one, said loudly.

Search machinery:

- **Nelder-Mead** (scipy), `xatol` 1e-3, `fatol` 1e-9, bound clipping (candidates
  are clamped, never rejected, at the bounds). Without scipy it degrades to a
  deterministic coordinate sweep — still honest, just slower, and the receipt's
  `residual_or_convergence.method` names which ran.
- **Deterministic multi-start**: `multi_start: N` adds starts from a bit-reversed
  lattice across the bounds. **No RNG and no clock anywhere** — that is what makes
  the byte-identical determinism pin possible. `max_evals` is the TOTAL budget,
  split across starts.
- **Robust (worst-case) mode**: `robust: {tols: {...}, aggregate: "worst"}` also
  evaluates every candidate at the tolerance extremes of the named parameters and
  scores it on the WORST corner; constraints must hold at every corner. The
  scheme used is stated in the receipt — full 2^k corners for k <= 3, else the 2k
  axis extremes.
- **Weighted multi-objective** (`objectives: [...]`) is a single scalarization,
  reported per term. It is **NOT a Pareto front**, and the manifest says so.

Assumptions baked in: parameters are CONTINUOUS in finite `[min, max]` boxes; the
evaluator is a PURE function of the parameters (a nondeterministic analyzer
breaks the determinism guarantee and can mislead the simplex); one scalarization
per run; derivative-free LOCAL search — multi-start mitigates but does not
eliminate local minima, and no global-optimality claim is made.

**Three diagnostics that make a wrong answer visible instead of plausible:**

1. **Quantization** (`quantization`). When two evals return a bit-identical score
   at different parameter vectors, the evaluator is discretising the search space
   (a voxel grid, a mesh seed, a rounded input). The receipt then carries a
   measured per-parameter LOWER BOUND on that effective resolution plus the
   evidence pair, and a loud stderr warning — a converged-looking optimum can be
   a plateau of the discretiser. A declared `"resolution": 4.0` on a parameter
   additionally flags search steps below it. A smooth evaluator must NOT be
   accused (a symmetric objective is a level set, not a plateau) — both directions
   are pinned in `tools/tests/test_checkers.py`.
2. **Witness-selection drift** (`selection_unstable`, `selection_evidence`). If a
   witness-based feature op (e.g. `fillet_edge_near`) resolves to more than one
   distinct `EdgeName` across the sweep, the candidates' objectives are not
   mutually comparable. Drifted candidates are REJECTED from the search, the run
   is stamped `selection_unstable: true`, and the evidence names every distinct
   edge with the parameters where it was first seen.
3. **Provenance honesty** (`optimizer_validation_status` vs
   `result_validation_status`). The OPTIMIZER is Validated; the RESULT is only as
   validated as the analyzer behind it. A nested `analysis_envelope` from the
   evaluator is inherited; otherwise the result is published as Demonstrated,
   never promoted by association.

## The contract

```
{template: {ops: [ ... any string "$name" is replaced by the param value ... ]},
 params:   {"rim_t": {"min": 2, "max": 8, "init": 4, "resolution"?: 0.1}, ...},
 objective?: "mp.inertia_diag[2] / mp.volume",     # dotted <op_id>.<key>; MAXIMIZES by default
 maximize?: true,                                   # legacy default is true — pass false to minimize
 objectives?: [{expr, weight, maximize}],           # weighted sum, reported per term
 targets?:    [{expr, value, tol, weight}],         # quadratic; with targets, `objective` is optional
 constraints?:[{expr, min?, max?}],
 max_evals?: 40, multi_start?: 4,
 robust?: {tols: {...}, aggregate: "worst"},
 evaluator?: {kind: "engine"}                       # default: the kernel-api CLI, one run per eval
            | {kind: "command", argv: [..., "$JOB"], job_template: {...}, timeout?: 300, cwd?},
 program_dir? | out_dir?}
```

**`timeout` is per candidate and defaults to 300 s.** Any real physics-in-the-loop
evaluator takes longer than that, and every candidate dies silently-as-a-failed-eval
if it is not raised (campaigns use 1800). This is the single most common way to
get a meaningless run.

**PATHS matter and are not a temp dir.** A candidate program/job is materialised
under `program_dir`, else `out_dir`, else the JOB FILE's own directory — so a
template's relative `import_step` / `load_part` paths resolve exactly as they do
in the job that carries them. The scratch file is removed after each eval; that
it lands in the caller's directory and is cleaned up is pinned in
`tools/tests/test_checkers.py`.

Receipt (last non-empty stdout line; logging on stderr): `best_params`,
`best_objective` (the single-`objective` value when one was given, else the
minimized combined score), `best_score`, `constraint_ok`, `best_measures`,
`evals` and `n_evals` (the same COUNT under an unambiguous name — a roll-up doing
`len(receipt["evals"])` used to raise `TypeError`), `history_first`,
`history_last`, `selection_unstable`, plus `targets`/`targets_met`, `objectives`,
`robust`, `multi_start`, `declared_resolution`, `quantization`,
`selection_evidence` when applicable, and `geometry_hash`,
`residual_or_convergence`, `optimizer_validation_status`,
`result_validation_status`, `analysis_envelope`.

Exit contract: the shared one in `tools/_receipt.py` — **0** `ok:true`, **1** the
tool could not run the request, **2** it RAN and REFUSED or the analysis failed.
The domain refusal is `refusal.no_successful_evaluation`: a run where every
evaluation failed or was rejected refuses, PERSISTS a receipt, and exits nonzero
rather than printing a bare line and exiting 0 (pinned in
`tools/tests/test_checkers.py`). Evaluator receipts with `ok:false` are failed
evals, never silently scored; an evaluator that prints success JSON and then
exits nonzero is rejected; non-finite evaluator outputs are rejected instead of
entering the search.

**Objective/constraint/target expressions are a safe AST over receipt data —
selectors and arithmetic, never a Python execution surface.** A sandbox-escape
expression is refused and touches no file (pinned, same test file).

## Benchmark gates (measured, frozen)

The registry records `gate: no` for `param_optimize`
(`python3 tools/analyzer_registry.py --tier param_optimize` returns
`"gate_suite": null`). The evidence is the validation pin plus two acceptance
tests.

The pin's ground truth is ANALYTIC known-optimum problems driven through a
hermetic pure-python command evaluator (no engine, no ACE), measured 2026-07-17:

| gate | what it pins | where |
|---|---|---|
| **Pin 1 — known optimum** | minimize `(x-3)^2 + (y+1)^2` on `[-10,10]^2`; analytic argmin (3, -1). Gate `best_score <= 1e-4` and params within +/-0.02; **measured best_score 6.44e-08 at (2.99996, -1.00025)**. Also pins that a bare `objective` still MAXIMIZES by default (the run is explicit `maximize:false`, and a silent flip fails loudly). | `tools/validation/param_optimize_validation.py`; frozen in `tools/manifests/param_optimize.manifest.json` → `validation.error_band.known_optimum` |
| **Pin 2 — target convergence** | `x^3 + 13x = 40` has one real root, x* = 2.0912942; the target machinery must land inside the declared tol 0.05. **Measured achieved 40.000000, miss 1.14e-07.** | same pin; `error_band.target_convergence` |
| **Pin 3 — active constraint** | maximize `x` s.t. `x <= 7` on `[0,10]` must end ON the cap without a violated candidate. Gate `[6.9, 7.0]` and `constraint_ok`; **measured 6.999878**. This pin CAUGHT A REAL DEFECT during authoring: ulp-scale cap violations underflowed the relative penalty and an infeasible point won by a float ulp — fixed by feasibility-first selection. | same pin; `error_band.active_constraint` |
| **Pin 4 — R5 determinism** | two runs of the identical job produce **byte-identical receipts** (no clocks, no RNG, bit-reversed multi-start lattice). | same pin |
| witness-selection drift | the detector's own silent-wrong scenario, end to end: before detection the optimizer returned `ok:true` with `best_objective` **23985.514** at h=30 — a fillet silently rounded on the WRONG (decoy) edge, reported as full success. The test now requires `selection_unstable: true`, `distinct_edges == 2`, and that the decoy value is NOT reported as best. | `tools/tests/param_optimize_drift_test.py` (needs `target/release/kernel-api`) |
| behavioural contracts | zero and negative constraint bounds, quantization detected / not falsely accused, candidate jobs materialised under the caller's dir and cleaned up, `n_evals` published, no-successful-evaluation refusal + persisted receipt, exit-9-after-success rejected, non-finite rejected, expression sandbox | `tools/tests/test_checkers.py` → `test_param_optimize` |

Direction of error: none systematic — the pin's own note is "no systematic bias;
direct search on the analyzer's own receipts".

## Validity limits / out of scope

- **The pins are analytic toy problems, and that is all they establish.** They
  prove the simplex, the target cost, the penalty sign, the feasibility-first
  selection and the determinism are correct. **They do not bound the optimality
  gap on a real objective**, and no study in this repo does. Quote what the
  receipt says (best score, evals, constraint status), never a
  "percent-optimal".
- **Convergence budget is not characterised.** Nothing here establishes how many
  evaluations a given class of problem needs, or what `max_evals` is enough. The
  manifest's limit is a budget statement, not an accuracy statement: `max_evals`
  caps total analyzer runs and robust mode multiplies each eval by the corner
  count.
- **Local optimizer, non-convex problems.** A multi-modal objective can converge
  to a local optimum; raise `multi_start` and inspect the per-start bests. No
  global claim, ever.
- **Continuous parameters only, roughly <= 8 of them** (manifest
  `limits_of_validity.regime`). Discrete and categorical choices — hole count,
  which bearing, thread pitch as a catalogue item — are OUT OF SCOPE. Sweep those
  outside the optimizer.
- **A weighted sum is not a Pareto front.** One scalarization is explored per run;
  trade-off exploration needs multiple runs at different weights.
- **The result is only as good as the evaluator.** The optimizer's Validated tier
  transfers to the SEARCH, not to the physics: an engine-evaluator objective
  reads the kernel's validated measures, a command-evaluator objective inherits
  the tier of whatever analyzer it drove (check that analyzer's own card and
  manifest). The receipt separates `optimizer_validation_status` from
  `result_validation_status` precisely so this cannot be blurred.
- **A nondeterministic evaluator breaks the guarantee.** The determinism pin
  covers the optimizer; it cannot cover an analyzer that returns different
  numbers for the same input.
- **`quantization` reports a LOWER BOUND on effective resolution**, measured from
  the run's own evals — not the discretiser's true step. Absence of the key is
  not proof of a smooth evaluator; it is absence of evidence within this run's
  sampled pairs.
- **`selection_unstable` covers witness-resolved edges only.** Other silent
  identity drift in a template (a face index, a boolean order) is not detected.

## When to use it

When a dimension, a wall thickness, a cell size or a lattice constant is about to
be chosen by feel, and there is a receipted number that says whether the choice
is good — kernel measures (`mp.volume`, `mp.inertia_diag[2]`), a physics runner's
receipt, or a derived model's envelope. Also when the question is a TARGET rather
than an extremum ("land the first mode at 40 +/- 1 Hz"), which is `targets`, and
when the honest requirement is that the part works at the tolerance extremes and
not merely at nominal, which is `robust`.

Do not use it to explore a trade-off surface (one scalarization per run), to pick
between discrete alternatives, or as a substitute for the campaign's closed-form
gate — an optimum that was never gated is still ungated.

Run: `python3 tools/analyzers/param_optimize.py job.json` ·
prove: `python3 tools/validation/param_optimize_validation.py && python3 tools/tests/param_optimize_drift_test.py`
