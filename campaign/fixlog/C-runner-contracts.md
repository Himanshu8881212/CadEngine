# C-runner-contracts — fixlog (T3 + T13 + T7 + registry tier)   [COMPLETE]

Owner: C-runner-contracts. Owned files ONLY:
`tools/ace_fea_runner.py`, `ace_fea_tet_runner.py`, `ace_modal_runner.py`,
`ace_buckling_runner.py`, `ace_thermal_runner.py`, `ace_contact_runner.py`,
`ace_fatigue_runner.py`, `ace_optimize_runner.py`, `graded_infill_runner.py`,
`_ace.py`, `_receipt.py`, `analyzer_registry.py`.

Repro dir: `<scratchpad>/repro`. Negative-control copy: `<scratchpad>/prefix`.

---

## THE CONTRACT (one, for every runner I own)

    exit 0  ok:true   analysis ran, receipt usable
    exit 1  ok:false  the tool could not run the request (usage / unreadable job
                      / internal error). NO analysis was performed.
    exit 2  ok:false  the tool RAN and REFUSED, or the analysis failed.

Any nonzero => do not quote. `error_kind` is a machine-matchable slug
(`refusal.*`, `timeout`, `killed.*`, `internal`, `usage`,
`receipt_path_conflict`). The receipt also carries `exit_code` and, on failure,
`exit_contract {mode, code, meaning, contract, opt_out}` — so a failed analysis
is detectable from the receipt ALONE as well as from `$?`.

LOUD OPT-OUT: `LMCAD_RUNNER_EXIT=legacy` (env) or `"legacy_exit_zero": true`
(job key) restores exit-0-always AND records `exit_contract.mode = "legacy"`
plus `suppressed_code`. Strict is the default everywhere.

Also, on every runner:
- `--out PATH` — atomic (temp + fsync + rename) receipt write.
- a job `receipt` key that disagrees with `--out` is REFUSED **before the
  solve** (exit 1, `receipt_path_conflict`); the shipped file is untouched.
- `LMCAD_RECEIPT_DRY_RUN=1` — no on-disk write at all (safe what-if runs).
- `wall_budget_s` / `LMCAD_WALL_BUDGET_S` + SIGTERM/SIGINT/SIGHUP handlers →
  a synthesized `ok:false` receipt naming the limit/signal.
- `determinism {nondeterministic_paths, core_sig_figs, core_digest,
  solver_reproducibility}` — the T7 fix.

---

## REPRODUCED before fixing (verbatim)

1. **T13 buckling accepts pure TENSION.** `din_rail refusal_buckling_tension.json`
   (12x12x16 PLA prism, clamp z<=0.5, +40 N at z>=15.5, 4.0 s):
   `ok:true`, `buckling_load_factor 18437.37126639016`,
   `critical_load_N 737494.8506555998`, `design_critical_load_n 368747.43`,
   exit 0. Only note: "material would YIELD before elastic buckling ~7566 MPa".
2. **T13 zero-catch fixture.** ACE library call directly, fixture bbox over air:
   `reference_fea` RETURNS `max_disp 3.86e-07 m`, notes
   `fixture[1] (slider): selector matched no active nodes.` — a field for a
   model missing a boundary condition, `ok` true. (rotor F6, verbatim shape.)
3. **T13 contact row 0.** Plane obstacle 0.3385 mm above the tip at lambda=0:
   row 0 `insertion_force_n 16925.0 N`; rows 1..n peak `1.9242 N`.
   Pre-fix `insertion.peak_force_n` = max over ALL rows = **16925.0 N**, i.e.
   **8795x** the real peak (wrist F7 measured 425x on its own geometry).
4. **T7 receipts not byte-comparable.** Two runs of `buckling_bolt_cam.json`:
   files differ; `buckling_load_factor` 367.3955977296438 / ...435 (shipped
   ...439); `timings_s` differs every run.
5. **T3 two contracts in one directory.** `ace_fea/tet/modal/buckling/optimize/
   graded_infill` had `sys.exit(0)` on the failure path; `thermal/contact/
   fatigue` `sys.exit(1)`. `ace_fea_tet_runner` additionally `return`ed (exit 0)
   on an explicit `tet solver refused` receipt.
6. **Registry.** `ace_contact_runner.py / ace_fatigue_runner.py /
   ace_thermal_runner.py` listed as "analyzer-shaped tools not in the registry"
   while `tools/solvers/README.md` calls all three "green".

## BACKWARD-COMPAT SCANS (done BEFORE changing behaviour)

- 82 shipped receipts carry `fixtures[].nodes_or_elements`; **none is 0** →
  the zero-catch refusal breaks no shipped campaign.
- **No** `ace_*` job in any campaign carries a job-level `receipt` key →
  honouring it in the runners creates no new files for existing campaigns.
- All 7 portfolio buckling jobs re-run. 4 preserved, 3 refused (below).

---

## CHANGED

| file | change |
|---|---|
| `_receipt.py` | rewritten as the shared receipt + exit-code contract: exit codes, `Refusal`, atomic write, `--out`, dry run, signal + wall-budget receipts, `load_job` / `finish` / `run_cli`, `determinism_block`. Old `emit(receipt, job, tool)` / `receipt_path(job, tool)` signatures preserved. |
| `_ace.py` | re-exports the contract; adds `selector_catch_audit` + `refuse_empty_selectors`, `compression_check` + `refuse_tensile_load_case`, `mesh_resolution_receipt` (+warning), `validated_range_check` (+warning), `apply_warnings`. |
| all 9 runners | routed through `load_job` / `finish` / `run_cli`; docstrings state the new contract; `determinism` block added. |
| `ace_fea_runner.py` | zero-catch refusal, `mesh_resolution`, `validated_range`, `warnings` — all BEFORE the solve. |
| `ace_buckling_runner.py` | + `compression_check` and the tensile refusal, before the solve. |
| `ace_modal_runner.py` | zero-catch refusal, mesh/validated-range warnings. |
| `ace_fea_tet_runner.py` | the `tet solver refused` path now exits 2, not 0. |
| `ace_contact_runner.py` | curve row 0 labelled `curve_rows.row_0` + `initial_state`; every receipt statistic (`path_max`, `insertion`) reads rows 1..n; `contact.initial_state_not_equilibrated` warning. |
| `ace_thermal/contact/fatigue` | their own `JobError` / `DataRefusal` / `ConvergenceError` map to exit 2; internal errors to exit 1. |
| `analyzer_registry.py` | registers `ace_thermal` (Demonstrated), `ace_contact` (Demonstrated), `ace_fatigue` (Cataloged) with machine-readable `tier_reason` + `gate_suite`; new `--tier NAME` query; new **`--check-contract`** executable gate suite (31 gates). |

## PROVED

Suites, all green after the change:
`test_ace_thermal.py` 21/21 · `test_ace_contact_fatigue.py` 46/46 ·
`test_ace_modal_buckling.py` PASS · `ace_fea_validation.py` /
`ace_modal_validation.py` / `ace_buckling_validation.py` /
`ace_optimize_validation.py` all PASS · `analyzer_registry.py --check` PASS ·
`analyzer_registry.py --check-contract` **PASS 31/31** (3 consecutive runs).

Negative control — `<scratchpad>/prefix` is a copy of `tools/` with the pre-fix
contract restored (exit 0 on failure, no zero-catch refusal, no tensile
refusal, no wall budget, job `receipt` key wins over `--out`):
**`--check-contract` FAILS 24/31, 7 gates red**, including
`8 job 'receipt' vs --out ... shipped_intact=False` — the singulator F14 clobber
reproduced and caught. Running that copy against the ORIGINAL runners also
KeyErrors on `determinism` (the key does not exist pre-fix).

Physics preserved (same job, new runner vs shipped receipt):

| receipt | shipped | re-run | delta |
|---|---|---|---|
| `din_rail buckling_bolt_cam` λ | 367.3955977296439 | 367.3955977296438 | 3e-16 |
| `cubesat a5_buckling_frame_2p0` λ | 0.5378860166137663 | 0.5378860166134093 | 7e-13 |
| `gripper buckling_slice_v04` λ | 3.938200053100366 | 3.9382000532343473 | 3e-11 |
| `din_rail fea_foot_lc1` max_vm_pa | 592071.2438111951 | 592071.2438111951 | **bit-identical** |
| `ball modal_frame_v1p6` f1 | 541.6485954092929 | 541.6485954092924 | 9e-16 |

T7 demonstrated: two back-to-back `buckling_bolt_cam` runs — receipt bytes
DIFFER, `determinism.core_digest` is EQUAL.

---

## BEHAVIOUR CHANGE THAT AFFECTS A SHIPPED CAMPAIGN — say it loudly

`agriculture_system/jar_top_seed_singulator/programs/buckling_neck.json` now
**REFUSES** (`refusal.no_compressive_load_path`, load alignment **+0.892**).
It shipped `ok:true`, λ 844.5, `design_critical_load_n` 15201 N, and its README
line 119 + ANALYSIS §4 quote that as "**422x the 36 N design load**".

It is the same defect as din_rail F7: the 36 N reference load pulls the housing
top AWAY from the clamped base — a tie, not a strut — and the receipt's own
note already said the material would yield at **1072 MPa vs a 55 MPa yield**,
i.e. 20x, before any bifurcation. The old behaviour IS the bug. The verdict's
§6 did not catch this one; the fix did.

`marine_system/folding_deck_cleat/programs/jobs/refusal_buckling_horn_tension.json`
also now refuses (alignment +0.862) — which is what that campaign WANTED; it is
named "refusal" and was recorded as a non-refusal.
`electronics_system/din_rail_pi4_enclosure/receipts/refusal_buckling_tension_NONREFUSAL.json`
is now a historical artefact: the tool refuses.

Escape hatch, recorded not hidden: `"allow_tensile_load_case": true` forces the
solve and stamps `compression_check.override: true` + a note in `notes`.

## CROSS-OWNER REQUESTS

1. `campaign/OPERATOR_BRIEF.md` + `campaign/digests/tools_cookbook.md`: the rule
   "ACE runners exit 0 even on failure — parse `ok`, never `$?`" is now WRONG in
   its second half. Replace with the 0/1/2 table and the `LMCAD_RUNNER_EXIT=legacy`
   opt-out; and replace the `runner.py job | tail -1 > receipt.json` idiom with
   `runner.py job --out receipt.json` (the redirect truncates at launch — turgo F8).
2. `campaign/DELIVERABLE_SPEC.md` §3 Determinism: it names only STLs/PNGs.
   State the solver contract: receipts are reproducible to
   `determinism.core_sig_figs` significant figures, compare
   `determinism.core_digest`, never receipt bytes.
3. `tools/manifests/ace_fea.manifest.json` (+ modal/buckling/tet):
   `validation.direction` and `error_band` read as unqualified analyzer
   properties. The runners now emit `validated_range` + an
   `manifest.outside_validated_discretization` warning, but the manifest text
   itself should be re-scoped to its specimen (rotor F14).
4. `tools/solvers/README.md`: the "green" column is a GATE-SUITE status. Add the
   registry TIER beside it (`analyzer_registry.py --tier <name>`), now that
   thermal/contact/fatigue are registered at Demonstrated/Demonstrated/Cataloged.
5. `tools/audit_docs.py` is the one remaining registry catalogue-drift warning —
   register it or declare it non-analysis.
6. `air_topology_audit.py` calls `_receipt.emit(...)` then exits 0 regardless
   (cubesat F11 + the exit-code class). It should use
   `_receipt.load_job/finish/run_cli` like the other tools now do.
