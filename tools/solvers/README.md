# Solver registry

One card per solver in this directory. A card states, in a fixed shape: runner
+ gates, the PHYSICS and its governing equations, the discretization, the JSON
manifest -> receipt contract, a table of benchmark gates with **measured**
numbers and the bands frozen from them, the validity limits / out of scope, and
when to use it. A solver is guilty until its own gate suite is green
(DESIGN_GUIDE §25.7) — the card exists so the next agent can tell what a number
from that solver is worth without reading the source.

| solver | card | physics | runner · gates | status | when to use |
|---|---|---|---|---|---|
| **ace_fea** | [ace_fea.md](ace_fea.md) | linear static elasticity, `K u = F`, von Mises | `ace_fea_runner.py` · `ace_fea_validation.py`, `ace_fea_kt_validation.py` | pinned (needs the ACE package) | stiffness/deflection and load paths in 3-D; the reference stress field other solvers consume |
| **thermal** | [thermal.md](thermal.md) | heat conduction, steady + transient (`rho cp dT/dt = div(k grad T) + q`) | `ace_thermal_runner.py` · `test_ace_thermal.py` | green, in-house | service-temperature questions: heat soak, bearing seats near a heat source, PLA hot side vs `softening_c` |
| **modal** | [modal.md](modal.md) | undamped free vibration, `K phi = omega^2 M phi` | `ace_modal_runner.py` · `test_ace_modal_buckling.py` | green (ACE K/M, local eigensolve) | resonance: is a bracket's first mode near the printer/motor excitation band |
| **buckling** | [buckling.md](buckling.md) | linearised bifurcation, `K phi = -lambda K_g phi` | `ace_buckling_runner.py` · `test_ace_modal_buckling.py` | green (ACE) | slender compression members; UPPER bound — apply the card's 0.5 knockdown |
| **contact** | [contact.md](contact.md) | geometrically-nonlinear planar beam + rigid-obstacle penalty contact, Newton-Raphson | `ace_contact_runner.py` · `test_ace_contact_fatigue.py` | green, in-house | snap-fits, latches, living hinges, spring clips: insertion/retention force curves and peak strain at large deflection |
| **fatigue** | [fatigue.md](fatigue.md) | stress-life: Basquin S-N + mean-stress correction + Palmgren-Miner damage | `ace_fatigue_runner.py` · `test_ace_contact_fatigue.py` | green, in-house · **screening only** | repeated actuation / cyclic duty; REFUSES any material without credible printed S-N data (PLA is the only one that has it) |
| **creep** | [creep.md](creep.md) | time × temperature allowable LOOKUP (not a solve): `sig_allow(T, t)` from the record's own tabulated cells, rounded UP on both axes by default; interpolation between bracketing cells is OPT-IN, labelled `basis: "interpolated"`, Python-only | `materials.py --creep` / `production_check.py` · `materials_crosslang_test.py`, `materials_creep_crosslang.py` gates | green, in-house · **table lookup, PLA only** | any load HELD rather than applied: a static margin says nothing about a load that never comes off. REFUSES above the hottest tabulated tier, for a material with no table, and when no duration is stated |
| **air_topology_audit** | [air_topology_audit.md](air_topology_audit.md) | voxel parity-fill + 6-connected flood label of the INTERNAL AIR; connectivity between named seeds, not a flow model | `analyzers/air_topology_audit.py` · none (3 regressions in `tests/test_aux_tools.py`) | **no gate suite** · Demonstrated | parts whose function lives in the void — speaker lines, manifolds, ducts, channels: watertight/geometric_ok gate the MATERIAL, not the path |
| **sweep_check** | [sweep_check.md](sweep_check.md) | sampled parameter sweep over the engine's `clearance`/`coincident_fit` measures; free-motion proof only | `analyzers/sweep_check.py` · none (behavioural pins in `tests/test_checkers.py`, SKIPPED without `target/release/kernel-api`) | **no gate suite** · Demonstrated | insertion and motion ranges: does the mechanism move through its travel clear. NEVER for a must-NOT-fit claim — that is `intersection` + `exact_volume` |
| **balance_check** | [balance_check.md](balance_check.md) | rigid-body mass properties about a declared spin axis: CG offset, static imbalance `U = m r`, couple terms via parallel axis, `F = m r omega^2` | `analyzers/balance_check.py` · none (**no test in `tools/tests/` at all**) | **no gate suite** · Demonstrated | rotating parts — props, flywheels, rotors, spool hubs. MEASURES only: `ok:true` is not a pass, and this repo carries no balance-grade allowable |
| **joint_check** | [joint_check.md](joint_check.md) | fastener rules engine over published-typical capacity tables: heat-set pull-out, thread strip, bearing/shear, ISO 898-1 steel, min engagement, `1/sqrt((T/Tc)^2+(S/Sc)^2)` | `analyzers/joint_check.py` · none (behavioural pins in `tests/test_checkers.py`) | **no gate suite** · **Cataloged** (lowest tier) | choosing and sizing a joint: heat-set vs threaded-into-plastic vs captive nut, boss engagement. Screening ranking, never a capacity — no pull test backs it |
| **ace_fea_tet** | [ace_fea_tet.md](ace_fea_tet.md) | the SAME linear static elasticity as ace_fea (`K u = F`, von Mises) on a gmsh BODY-FITTED tet10 mesh with nodal stress recovery — true curved surfaces, no voxel staircase | `analyzers/ace_fea_tet_runner.py` · `validation/ace_fea_kt_tet_validation.py` (+ `analyzer_registry.py --check-contract`) | pinned, **no gate suite** · Validated | fillet/notch PEAK stress, where the voxel path's Kt does not converge: measured Kt 1.545 → 1.610 → 1.652 vs Peterson 1.667, monotone from below. Fields are per-NODE, so no GridField hand-off |
| **ace_optimize** | [ace_optimize.md](ace_optimize.md) | SIMP topology optimization over the ace_fea hex8 solve: `min F^T u` at fixed volfrac, cone density filter + optimality-criteria update, then a BINARY as-built re-analysis | `analyzers/ace_optimize_runner.py` · `validation/ace_optimize_validation.py` (+ `analyzer_registry.py --check-contract`) | pinned, **no gate suite** · Validated | where material has to be when a rib pattern is about to be guessed. Compliance-optimal is NOT strength-optimal, and the STL is a mesh only — re-gate the result |
| **param_optimize** | [param_optimize.md](param_optimize.md) | not physics: Nelder-Mead direct search over a scalarized cost (objectives + quadratic targets + relative constraint penalty), feasibility-first, deterministic multi-start, optional worst-case tolerance corners | `analyzers/param_optimize.py` · `validation/param_optimize_validation.py`, `tests/param_optimize_drift_test.py`, `tests/test_checkers.py` | pinned, **no gate suite** · Validated | choosing a dimension against a receipted number instead of by feel — drives the engine OR any receipt-emitting analyzer. Pins are analytic toys: they prove the search, never the optimality gap |
| **graded_infill** | [graded_infill.md](graded_infill.md) | geometry synthesis, not a solve: percentile remap of a prior ace_fea von Mises field to gyroid wall thickness, volume-calibrated `alpha(t)` band, solid skin, kernel-gated meshing | `analyzers/graded_infill_runner.py` · none — only `--selftest` (calibration round-trip, measured 0.0093 vs a 5% gate) | **no gate suite, no pin** · Demonstrated | hollowing a part that is already stiff enough: thicker lattice walls where the stress is. Graded stiffness is NOT characterised — re-analyse the output and gate on that |
| **tolerance_stack** | [tolerance_stack.md](tolerance_stack.md) | closed-form 1-D chain arithmetic, not physics: worst-case band about the TRUE nominal (`sum lo/hi`) AND RSS (`+/-t` = 3 sigma, asymmetric mid-shift), plus bore/shaft fit extremes | `analyzers/tolerance_stack.py` · `validation/tolerance_stack_validation.py`, `tests/test_checkers.py` | pinned, gate suite green · Validated (arithmetic) | will it still close at the printer's extremes: bearing seats, shaft-in-bore, lid lips, min engagement. Worst-case is a guarantee, an RSS-only pass is a statistics claim; the 0.15 mm printer default is a placeholder, not a measurement |
| **production_check** | [production_check.md](production_check.md) | a RULES engine over another solver's number, not a solve: static / creep-table / fatigue / temperature / anisotropy derating of one `ace_fea` peak stress against `material_db.json` cells | `analyzers/production_check.py` · `validation/production_check_validation.py`, `--selftest`, `tests/materials_crosslang_test.py` | pinned, **no gate suite** (registry `gate: no`) · Validated (arithmetic) | the last gate before a part is called shippable — and the only one that asks how long the load is HELD (`duration_h` required when sustained; the creep number itself is [creep.md](creep.md)) |
| **production_dossier** | [production_dossier.md](production_dossier.md) | bookkeeping, not physics: divergence-theorem mesh volume/area, shell+infill printed-mass model, grams-per-hour time heuristic, FFD plate packing | `publish/production_dossier.py` · `validation/production_dossier_validation.py` (`tests/test_aux_tools.py` covers only `--help`) | pinned, gate suite covers `--help` only · Validated (bookkeeping) | the campaign BOM, cost, plate count and print hours — and the thick-section warning that says a part wants hollowing. The +/-30% mass / +/-50% time bands are DECLARED, never measured against a slicer |
| **damped_oscillator** | [damped_oscillator.md](damped_oscillator.md) | a DERIVED closed-form model with citations (Ogata 5-3, Rao ch. 2), not a solve: `x'' + 2 zeta wn x' + wn^2 x = wn^2 u(t)` by RK4, gated on Mp / tp / period | `analyzers/derived_model.py` (`DampedOscillator`) · none — 3 inline gates re-run on EVERY invocation (refuse-before-run) + `--selftest` | **no pin, no gate suite** · Demonstrated | lumped 2nd-order transients (overshoot, peak, ±2% settling) and the template for any new derived model. Results are stamped `synthesized_inloop` and can never claim `validated` |

Notes that apply across the registry:

- **Receipt contract**: the LAST non-empty stdout line of every runner is one
  JSON object; logging goes to stderr. Since 2026-08-08 there is ONE exit
  contract for all of them (`tools/_receipt.py`): **0** = `ok:true`, **1** = the
  tool could not run the request (usage / unreadable job / internal error),
  **2** = it RAN and REFUSED, or the analysis failed. The ACE-bridge runners
  used to exit 0 on failure; they no longer do, and
  `LMCAD_RUNNER_EXIT=legacy` (env) or `"legacy_exit_zero": true` (job key)
  restores the old behaviour and records `exit_contract.mode = "legacy"` on the
  receipt. Parsing `ok` is unchanged and still correct; `$?` now agrees with it.
  Every failure receipt also carries a machine-matchable `error_kind`.
- **Status vs TIER**: the `status` column is a GATE-SUITE status ("green" = its
  own suite passes). It is not an analysis tier. The tier lives in the registry:
  `python3 tools/analyzer_registry.py --tier <name>` returns it as JSON
  (`ace_thermal` Demonstrated, `ace_contact` Demonstrated, `ace_fatigue`
  Cataloged — deliberately below Demonstrated, because proving the Miner
  arithmetic is not proving the life).
- **Materials**: `tools/analyzers/materials.py` is the one source of truth for material
  records (`tools/materials/<key>.json`). `tools/materials/fatigue.json` is a
  SIDECAR table (`meta.schema_kind = "fatigue_table"`), not a record.
- **Everything is as-designed, not as-printed**: no solver here models
  printed-layer anisotropy inside the solve. Apply
  `tools/analyzers/materials.py derated()` to the ALLOWABLE, not to `E`.
- **Adding a solver**: write the runner, write the gate suite, run it, freeze
  the bands from the MEASURED numbers, add a meta-negative-control that proves
  the suite can go red, then add the card and a row here.
