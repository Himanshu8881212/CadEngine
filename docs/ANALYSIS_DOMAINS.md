# Analysis domains — what an agent can honestly analyze here (and what not)

This is the capability CONTRACT for physics analysis in this repo. It exists so
an AI agent (or a human) can answer, before starting: *is this analysis inside
the validated surface, derivable with citations, or out of scope and in need of
an external solver?* Every claim below is checkable against a file in-tree.

The intended loop — the reason the analysis stack has the shape it has:

1. **Research** the physics (find the governing equations AND their sources).
2. **Write the equations down** with citations — `tools/derived_model.py`
   refuses a source-less model at import time.
3. **Implement** them as a runnable model (or pick an in-tree solver).
4. **Validate** against closed forms / known limits — gates re-run on every
   invocation and REFUSE to evaluate if they fail ("refuse-before-run").
5. **Optimize to targets** — `tools/param_optimize.py` drives ANY receipted
   analyzer (engine programs and physics models share one optimizer) with
   first-class convergence targets, multi-start, and worst-case tolerance
   corners.

Worked end-to-end (run 2026-07-17, receipts quoted verbatim): *"find the
damping ratio for 10% step overshoot"* — `param_optimize` over the
`damped_oscillator` derived model, target `values.overshoot_pct = 10 ± 0.1`:

```json
{"evaluator": {"kind": "command",
               "argv": ["python3", "tools/derived_model.py", "$JOB"],
               "job_template": {"zeta": "$zeta", "omega_n_rad_s": 20.0}},
 "params":  {"zeta": {"min": 0.05, "max": 0.95, "init": 0.3}},
 "targets": [{"expr": "values.overshoot_pct", "value": 10.0, "tol": 0.1}],
 "max_evals": 60}
```

Result: `zeta* = 0.59116` in 50 evals, `targets_met: true` — the closed form
`ζ = −ln(0.1)/√(π² + ln²(0.1)) = 0.59116` agrees to all five printed decimals.
Every eval's receipt carried the model's 3 validation gates and an RK4
dt-refinement convergence receipt inside the `lmcad.analysis.v1` envelope.

---

## Tier (a) — in-tree pinned solvers (the validated surface)

Hex8 voxel-grid solvers over LMCAD geometry (`ops+solid` or a density `.npy`),
each with a committed manifest (`tools/manifests/`) and a ground-truth pin that
CI (`analysis-gate.yml`, self-hosted job) actually executes:

| capability | tool | pinned against | band |
|---|---|---|---|
| linear-elastic static (stress, deflection, compliance) | `ace_fea` | Euler-Bernoulli + shear cantilever | −20%..0 coarse, −10%..0 fine (under-predicts, converges from below) |
| free-vibration modes | `ace_modal` | E-B cantilever frequencies | +4.0% / +0.9% |
| linear (eigenvalue) buckling | `ace_buckling` | Euler column | +7.3% / +3.0% |
| SIMP topology optimization | `ace_optimize` | exact inequalities (descent ≤0.9×, material-removal monotonicity ≥1.0, volume ±0.02, watertight STL) | measured 0.15× / 1.58 / exact |
| parametric optimization over ANY receipted analyzer | `param_optimize` | analytic optima (paraboloid argmin, cubic-root target, active cap) + byte-identical determinism | measured 6.4e-8 / 1.1e-7 / feasible-at-cap |

Known, named limitation (not hidden): peak stress at fillets/notches from the
voxel FEA is ±20–30%, biased HIGH, and does not converge under refinement —
the Kt pin (`ace_fea_kt_validation.py`) makes this visible; body-fitted meshing
would be a build, not a wiring fix.

## Tier (b) — derivable domains (`tools/derived_model.py`)

Any domain where the agent can CITE ground truth is one scaffold subclass away:
1-D / lumped / closed-form models — acoustics (transfer matrices, Helmholtz
resonators, Thiele-Small alignments), thermal RC networks and 1-D conduction,
beam/plate/shaft sizing formulas, linkage kinematics/dynamics, magnetic
circuits, RC/RLC electrical analogues. The criterion is falsifiability, not
domain: **if you cannot write a gate against an independent closed form or
known limit, the model must not run.** The scaffold enforces the rest:
citations required at import, gates re-run per invocation, results stamped
`synthesized_inloop` (never `validated`) inside the provenance envelope, and
`provenance.check_synthesized` re-verified before anything prints.

Committing the model's manifest to `tools/manifests/derived/` puts it on the
graduation ledger automatically — at Demonstrated, below the validated line,
until a real pin lands (`docs/MANIFEST_SCHEMA.md`). Start a new one with
`python3 tools/derived_model.py --new my_domain`.

## Tier (c) — rules and tables (Cataloged, or Validated-arithmetic)

Deterministic arithmetic over published tables: ISO 286 fits, tolerance stacks,
FDM production derating, joint/fastener checks, BOM/plate bookkeeping. Correct
by construction relative to their cited sources; not physics simulations, never
presented as such (`docs/ANALYSIS_TIERS.md`). Three of them —
`tolerance_stack`, `production_check`, `production_dossier` — carry a manifest
and a hand-derived validation pin since 2026-09-02 and the registry reports
them **Validated**; that word there means "the arithmetic is proven against an
independent hand derivation with a stated error band", and their `kind` stays
rules_engine / reporting. `joint_check` remains Cataloged (no pin).

## NOT covered — refuse and say so

There is **no in-tree solver** for: 3-D fluid flow / aerodynamic fields (CFD),
3-D electromagnetic fields, transient 3-D thermal fields, nonlinear FEA
(plasticity, contact, large deformation, hyperelastics), or fatigue-life
prediction beyond cited S-N table arithmetic. An agent asked for these must
say so and either (i) reduce honestly to a tier-(b) 1-D/lumped model with
stated limits, or (ii) bridge to an external solver — below. Presenting a
lumped estimate as a field solution is a contract violation, not a shortcut.

---

## External-solver bridge (the escape hatch)

Geometry OUT, numbers BACK IN — inside the same honesty envelope:

**Geometry out.** `export_stl` / `export_step` ops for mesh/B-rep consumers;
`tools/voxelize_stl.py` (STL → occupancy `.npy`) for grid solvers — the same
`.npy` contract the ACE runners consume (`nx,ny,nz` float density, `voxel_mm`,
`origin_mm`). Deterministic; the geometry content-hash
(`provenance.geometry_hash`) keys the result to what was actually exported,
and the STL relation to the B-rep program is `derived_from` with a stated
chord tolerance — not `equality`.

**Numbers back in.** External results do not surface bare. Wrap them with
`provenance.stamp(...)`: a structured residual/convergence receipt (bare
scalars are rejected), a manifest reference (write one with the
`derived_model` scaffold describing the external tool + config + sources),
and an honest `validation_status` — `synthesized_inloop` only if an inline
self-check against a known limit PASSED, else `synthesized_unvalidated`
(which `provenance.check_synthesized` refuses to let surface unmarked), or
`research` for frontier work. External results can never claim `validated`
without a committed manifest + pin, same as everything else.

**Optimization over external solvers.** `param_optimize`'s command evaluator
runs any argv, so an external solver wrapped in a script that prints one JSON
receipt line is immediately optimizable — same targets, same robust corners,
same receipts as the in-tree loop above. A receipt with `ok:false` is a failed
eval, never silently scored.

---

## Where the surface stands

`python3 tools/analyzer_registry.py` prints the live ledger; the CI gate
(`analysis-gate.yml`) blocks any Validated claim lacking manifest+pin and any
unparseable derived manifest. As of 2026-09-02: 18 registered surfaces, 9
Validated (50.0% above the line — the four structural solvers, both
optimizers, and the three pinned rules/bookkeeping engines), the rest honestly
below it.
