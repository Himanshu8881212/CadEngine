# D-checkers-optimizer — fixlog (COMPLETE)

Owner D. Owned: tools/tolerance_stack.py, tools/param_optimize.py,
tools/joint_check.py, tools/sweep_check.py, tools/balance_check.py,
tools/dim_suggest.py, tools/param_optimize_validation.py,
tools/param_optimize_drift_test.py.
NEW file I created: **tools/test_checkers.py** (30 falsifiable pins).

Mid-work, the owner of `tools/_receipt.py` landed the shared
receipt + exit-code + `--out` contract (`Refusal`, `load_job`, `finish`,
`run_cli`, exits 0/1/2, `LMCAD_RECEIPT_DRY_RUN`). I had written a parallel
`tools/_checker_cli.py`; I **deleted it** and rewired all six checkers onto the
shared module. One contract, one module.

## Reproduced BEFORE any edit (transcripts run 2026-08-08)

1. **T11-a asymmetric worst-case double count** (ball F4).
   `{"chain":[{"name":"A","nominal":10.0,"tol":{"plus":0.0,"minus":0.10},"dir":1},
   {"name":"B","nominal":9.0,"tol":0.0,"dir":-1}],"closes":{"min_required":0.0,"max_allowed":5.0}}`
   -> `nominal_gap 0.95, worst_min 0.85, worst_max 0.95`. Hand: A in
   [9.90,10.00], B == 9.0 => gap [0.90,1.00]. CONFIRMED.
2. **T11-b CHAIN raw KeyError** (din_rail F1): one-sided `closes` ->
   `{"ok": false, "error": "KeyError: 'max_allowed'"}`. CONFIRMED.
3. **T11-c receipt clobber** (singulator F14 / cleat F7): a copied job with a
   baked `receipt` path rewrote the original's receipt. CONFIRMED byte-diff.
4. **T11-d exit 0 on ok:false** (gripper F9) for tolerance_stack AND
   sweep_check. CONFIRMED.
5. **joint_check exit INVERTED** (ball F5): M6 -> `KeyError: 'M6'` exit 1;
   M5 failing `min_engagement_rule` -> exit 0. CONFIRMED both halves.
6. **T12-a `max: 0.0` crash** (turgo F3): `float division by zero`. CONFIRMED.
   Plus an unreported sibling: a **negative** `max` makes the penalty NEGATIVE,
   rewarding violation — repro drove `x` to the far bound at `ok:true`.
7. **T12-b silent quantization** (rotor F8): a floor(x/4)*4 command evaluator
   returned bit-identical scores for x=6.0 and 6.3; receipt said nothing.
8. **T12-c `evals` int under a plural name** (din_rail F9).
9. **T12-d / T4 station programs to system temp** (gripper F4/turgo F7/rotor F11).

## What changed

- `tolerance_stack.py`: worst-case band is about the TRUE nominal; the RSS
  mid-shift moved to a new `rss_nominal_gap` (+ `asymmetric_note`). One-sided
  `closes` is first class; unknown `closes` key refused; every KeyError path
  replaced by a named `Refusal`. Additive `band_contribution` / `pct_of_band`.
- `joint_check.py`: per-joint refusal for out-of-table size/material/type;
  other joints keep their evidence; exit code now agrees with the verdict.
- `param_optimize.py`: `relative_violation()` (legacy formula bit-for-bit for
  positive bounds, defined + correctly signed for zero/negative);
  `quantization_report()` (single-coordinate dead pairs, contradiction filter,
  live-step yardstick); `n_evals`; `station_dir()`/`_materialize()`;
  `no_successful_evaluation` refusal; `timeout`/`cwd`/`program_dir` documented.
- `sweep_check.py`: `sweep_semantics` in every receipt, per-watch
  `all_stations_interfering`, always-present `failed_stations`,
  `interfering_watches`, and a `refusal.no_free_station` when no watch ever saw
  a clear station.
- `balance_check.py`, `dim_suggest.py`: shared contract + `program_dir`.

## Proved

- `python3 tools/test_checkers.py` -> **ALL 30 OK** (rc 0).
- `param_optimize_validation.py` (4 pins incl. byte-identical determinism) rc 0.
- `param_optimize_drift_test.py` rc 0. `analyzer_registry.py` rc 0.
- Independent brute-force hand computation of the two affected shipped chains
  matches the FIXED tool exactly and differs from the shipped receipts.
- Portfolio sweep, dry-run, no campaign file touched: **91 shipped
  tolerance_stack receipts regenerated; 89 identical on every legacy numeric
  field; exactly 2 change** (both correct, no verdict flips):
  - `energy_system/turgo_runner/receipts/tol_2_nut.json`
    nominal 0.45 -> 0.30, worst [0.30, 0.90] -> [0.15, 0.75]; rss unchanged;
    pass_worst/pass_rss stay true.
  - `automotive_system/rotor_runout_gauge_bridge/receipts/tol_clamp_closure.json`
    nominal 0.2455 -> 0.2550, worst [0.0315, 0.4405] -> [0.0410, 0.4500];
    rss unchanged; verdicts stay true.
  All 9 other asymmetric jobs are FIT mode, which never used the mid-shift and
  is bit-unchanged.
