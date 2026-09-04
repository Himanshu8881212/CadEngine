# damped_oscillator — how far does a 2nd-order system overshoot, and when does it settle?

**Runner**: `tools/analyzers/derived_model.py` (shim: `tools/derived_model.py`), class
`DampedOscillator` · `python3 tools/analyzers/derived_model.py job.json` ·
`--selftest` · `--manifest OUT`
**Manifest**: `tools/manifests/derived/damped_oscillator.manifest.json`
(`derived_model: true`, `model_file: derived_model.py`)
**Gates**: three inline self-check gates that re-run on EVERY invocation
(refuse-before-run) · `python3 tools/analyzers/derived_model.py --selftest`
**Status**: **no committed validation pin and no benchmark gate suite** — the
registry records `pin: no`, `gate: no`, and the manifest's
`validation.pin_file` is the empty string
(`python3 tools/analyzer_registry.py --tier damped_oscillator`). The inline gates
pass: the selftest run here on 2026-09-04 printed *"3 gates (worst rel 1.29e-09),
envelope deterministic + guardrail-accepted, refusals loud"*.
**Tier**: **Demonstrated** — the registry's own reason string: *"Validated
requires a present manifest AND a present validation pin."*

**This is a DERIVED model, not a solve and not a validated solver.** It is the
worked exemplar of the `derived_model.py` scaffold: a closed-form / lumped model
an agent wrote down with citations, implemented, and gated against textbook
ground truth, per the tier-(b) contract in `docs/ANALYSIS_DOMAINS.md`. Every
result it emits is stamped `validation_status: synthesized_inloop` inside the
`lmcad.analysis.v1` provenance envelope and **can never claim `validated`** —
that word requires a committed manifest *and* a pin on disk
(`docs/MANIFEST_SCHEMA.md`). Committing its manifest to
`tools/manifests/derived/` is what auto-registered it on the graduation ledger,
at Demonstrated, below the validated line.

What it answers: for any single-degree-of-freedom second-order system — a
mass-spring-damper, an RLC analogue, a servo loop — driven by a unit step, how
much does it overshoot, when does it peak, when does it stay inside ±2 %, and at
what damped frequency does it ring. It is also the canonical target for
`tools/analyzers/param_optimize.py`: `docs/ANALYSIS_DOMAINS.md` records the
end-to-end run *"find the damping ratio for 10 % step overshoot"* → `zeta* =
0.59116` in 50 evals, agreeing to all five printed decimals with the closed form
`ζ = −ln(0.1)/√(π² + ln²(0.1))`.

## The physics

    ODE:        x'' + 2 zeta omega_n x' + omega_n^2 x = omega_n^2 u(t)     (unit step, zero ICs)
    overshoot:  Mp = exp(-pi zeta / sqrt(1 - zeta^2))                      (gate 1 ground truth)
    peak time:  tp = pi / (omega_n sqrt(1 - zeta^2))                       (gate 2 ground truth)
    period:     T  = 2 pi / omega_n between consecutive undamped peaks     (gate 3 ground truth)
    damped:     omega_d = omega_n sqrt(1 - zeta^2) ,  reported as Hz

Sources are mandatory for a derived model — `__init_subclass__` refuses a
source-less class at import time — and these are the ones declared: **Ogata,
*Modern Control Engineering*, 5th ed., sec. 5-3** (ISBN 978-0136156734) for `Mp`
and `tp`, and **Rao, *Mechanical Vibrations*, 6th ed., ch. 2** (ISBN
978-0134361307) for the natural period and the damped-frequency relation.

Discretization: **RK4, fixed step**, default `dt = T_d/400`, default horizon 10
damped periods. Outputs are read off the sampled trajectory (`overshoot_pct`
from the trajectory maximum; `settling_time_2pct_s` from the LAST sample outside
the ±2 % band), so peak and settling times are dt-quantized — the receipt carries
`dt`. Every run also computes the same metrics at `dt/2` and reports
`dt_refinement_rel_change`, with `dt_converged` true below 1e-3.

Assumptions: linear, time-invariant, single DOF, constant coefficients; zero
initial conditions and an ideal unit step; underdamped `0 <= zeta < 1`, which is
**enforced** in `evaluate()`, not merely documented.

## The contract

Job: `{"zeta": 0.15, "omega_n_rad_s": 25.0, "t_end_s"?, "dt_s"?}` —
`zeta` dimensionless in [0, 1), `omega_n_rad_s` > 0, times in seconds.

The LAST stdout line is the whole `lmcad.analysis.v1` envelope, which is why
`param_optimize`'s command evaluator can drive it directly
(`"targets": [{"expr": "values.overshoot_pct", ...}]`). It carries:

- `values{overshoot_pct (% of final), peak_time_s, settling_time_2pct_s, damped_freq_hz}`
- `provenance{validation_status: "synthesized_inloop", analyzer_name, analyzer_version,
  geometry_hash (over the model name + sorted job inputs), material_version, manifest_ref}`
- `residual_or_convergence{integrator: "RK4 fixed-step", dt_s, t_end_s,
  dt_refinement_rel_change, dt_converged, reported, gates[]}` — bare scalars are
  rejected by `provenance.stamp`; the residual must be structured
- `self_check{gate, gates_run, limit, expected, obtained, passed}` — the worst gate

Order is enforced by the scaffold: **gates → evaluate → stamp → re-check**. If
any gate fails the model returns `{ok:false, error: "validation gates FAILED —
model refuses to evaluate: ..."}` and never computes an answer. An out-of-limits
input raises through to `{ok:false, error: "ValueError: invalid_param: ..."}`
(e.g. `zeta = 1.4`, `omega_n_rad_s <= 0`). `provenance.check_synthesized`
re-verifies the envelope before anything prints.

**Exit codes differ from the analyzers**: `DerivedModel.main` returns **0** when
`ok` is true and **1** otherwise. It does not use `tools/_receipt.py`, so there
is no exit-2 "ran and refused" code here — a refusal and an internal error both
exit 1.

## Benchmark gates (measured, frozen)

**None. This model has no committed validation pin and no benchmark gate
suite.** The registry records `pin: no` / `gate: no`, and the manifest's
`validation.pin_file` is `""`. What exists instead is the scaffold's
**refuse-before-run inline gates** — three comparisons against the closed forms
above, re-executed on every single invocation, which block evaluation if any of
them drifts. Their limits are frozen in the manifest; the errors are recomputed
each run rather than stored.

| gate | ground truth (source) | frozen limit | where |
|---|---|---|---|
| `overshoot_vs_closed_form` | `Mp = exp(-pi*zeta/sqrt(1-zeta^2))` at zeta = 0.2, omega_n = 10 (Ogata 5-3) | ≤ 0.5 % rel | `DampedOscillator.run_gates`, `tools/analyzers/derived_model.py`; `validation.error_band` in the manifest |
| `peak_time_vs_closed_form` | `tp = pi/(omega_n*sqrt(1-zeta^2))` at zeta = 0.2 (Ogata 5-3) | ≤ 0.5 % rel | same |
| `undamped_period` | `T = 2*pi/omega_n` between consecutive undamped peaks (Rao ch. 2) | ≤ 0.1 % rel | same |
| selftest (contract, not a benchmark) | exemplar gates pass; a zeta = 0.15 / omega_n = 25 run matches `Mp` to < 5e-3 rel; `validation_status == "synthesized_inloop"`; `dt_converged` true; envelope byte-identical across two runs and accepted by the guardrail; `zeta = 1.4` refuses with `invalid_param`; a source-less subclass is impossible to define | `selftest()` in the same file |

Measured values: the manifest's `last_measured` is **2026-07-17**, and its
`error_band` entries are the gate **limits**, not measured errors — no measured
error is stored in the repo for this model. Running `--selftest` here on
2026-09-04 printed a worst relative gate error of **1.29e-09** against a limit of
5e-3; that is a number this card measured by running the command, not a frozen
band. The only other end-to-end figure on record is the `param_optimize` run
quoted above (`zeta* = 0.59116`, 50 evals, `targets_met: true`) in
`docs/ANALYSIS_DOMAINS.md`.

## Validity limits / out of scope

- **Underdamped only, `0 <= zeta < 1`.** At `zeta >= 1` the gates' closed forms
  do not exist and the model refuses. Critically-damped and overdamped responses
  are a DECLARED GAP: they would need their own gates and their own citations,
  i.e. a new derived model.
- **Not validated, and cannot be.** `synthesized_inloop` is the ceiling until a
  committed pin lands at `validation.pin_file`. Quote its numbers as a derived
  model with citations, never as a validated solver result, and never launder
  them into a campaign claim without the envelope.
- **Inline gates are not a pin.** They prove the implementation reproduces three
  textbook closed forms at ONE operating point each (zeta = 0.2 and zeta = 0);
  they do not establish accuracy across the parameter range. **Not
  characterised:** no study in this repo bounds the error at other zeta or
  omega_n, so treat the model's error away from the gate points as unbounded and
  do not gate a design claim on it.
- **dt-quantized outputs.** `peak_time_s` and `settling_time_2pct_s` land on
  sample times; `settling_time_2pct_s` is the LAST ±2 % band exit found on the
  sampled trajectory, which a coarse `dt_s` or a short `t_end_s` can misreport.
  Check `dt_converged` in the receipt.
- Single DOF, linear, constant coefficients, ideal unit step, zero initial
  conditions. No Coulomb friction, no backlash, no nonlinear damping, no forcing
  other than the step, no multi-mode coupling — for real structural modes use
  `modal`, which solves `K phi = omega^2 M phi` on the geometry.

## When to use it

When a design question is genuinely a lumped second-order one and you want the
transient, not the mode: how much a spring-loaded latch or a servo-driven axis
will overshoot its target, how long a damped arm rings before it is inside ±2 %,
what damping ratio hits a required overshoot (pair it with `param_optimize`, as
`docs/ANALYSIS_DOMAINS.md` does). Use it, second, as the **template** for any new
derived model: copy its shape — equations, assumptions, units, limits, real
citations, gates against an independent closed form — and start with
`python3 tools/analyzers/derived_model.py --new my_domain`. If you cannot write a
gate against a closed form or a known limit, the scaffold's rule applies: the
model must not run.
