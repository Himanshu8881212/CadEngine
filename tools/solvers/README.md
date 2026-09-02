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
| **creep** | [creep.md](creep.md) | time × temperature allowable LOOKUP (not a solve): `sig_allow(T, t)` from the record's own tabulated cells, rounded UP on both axes, never interpolated | `materials.py --creep` / `production_check.py` · `materials_crosslang_test.py`, `materials_creep_crosslang.py` gates | green, in-house · **table lookup, PLA only** | any load HELD rather than applied: a static margin says nothing about a load that never comes off. REFUSES above the hottest tabulated tier, for a material with no table, and when no duration is stated |

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
- **Materials**: `tools/materials.py` is the one source of truth for material
  records (`tools/materials/<key>.json`). `tools/materials/fatigue.json` is a
  SIDECAR table (`meta.schema_kind = "fatigue_table"`), not a record.
- **Everything is as-designed, not as-printed**: no solver here models
  printed-layer anisotropy inside the solve. Apply
  `tools/materials.py derated()` to the ALLOWABLE, not to `E`.
- **Adding a solver**: write the runner, write the gate suite, run it, freeze
  the bands from the MEASURED numbers, add a meta-negative-control that proves
  the suite can go red, then add the card and a row here.
