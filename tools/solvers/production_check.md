# production_check — does this peak stress survive being PRINTED, held, cycled and warmed?

**Runner**: `tools/analyzers/production_check.py` (shim: `tools/production_check.py`) ·
`python3 tools/analyzers/production_check.py job.json` · `--selftest`
**Gates**: `python3 tools/validation/production_check_validation.py`
(shim: `tools/production_check_validation.py`) ·
`python3 tools/analyzers/production_check.py --selftest` ·
`python3 tools/tests/materials_crosslang_test.py`
**Status**: pins green, **no benchmark gate suite** — the registry records
`gate: no` / `gate_suite: null` for it
(`python3 tools/analyzer_registry.py --tier production_check`). Both pins re-run
2026-09-04 in this worktree: *"production_check validation: ALL PINS OK"* and
*"SELFTEST PASS"*.
**Tier**: **Validated** (registry; `kind` = rules_engine — *Validated
arithmetic*, meaning the rule maths is pinned to an independent hand derivation
of the table cells, **not** that any physics was validated).

**This is a rules engine, not a solve.** It computes no field, meshes nothing,
and integrates nothing. It takes ONE number another solver already produced —
`max_von_mises_pa`, the peak von Mises stress from a prior `ace_fea` run — plus
the design's stated intent (material, load character, service temperature,
duration, print orientation) and grades that number against derated allowables
read out of `tools/material_db.json` and `tools/materials/pla.json`. It is the
last gate before a part is called shippable, and it is where a physics receipt
becomes a verdict.

It exists because a raw FEA margin is not a printed-part margin. The same
10 MPa peak is comfortable in a bracket loaded once, marginal in one loaded
across the layers, and *fatal* in one that holds the load for a year — and the
tool's job is to make you say which of those you meant before it will answer.

## What it actually computes

Five rules, each a division whose full arithmetic lands on the receipt row.
`demand_mpa = max_von_mises_pa / 1e6` throughout; `derate` = the material's
`layer_adhesion_factor` when the anisotropy rule fires, else 1.

    static      SF = (yield_mpa * derate) / demand_mpa                    (always)
    creep       SF = (creep_cell(T, t) * z_ratio_if_across_layer) / demand_mpa   (sustained)
    fatigue     SF = (ultimate_mpa * fatigue_knockdown * derate) / demand_mpa    (cyclic)
    temp        SF = service_limit_c / service_temp_c                     (always)
    anisotropy  fires when the primary load is inclined > 30 deg out of the layer
                plane; then derate = layer_adhesion_factor multiplies EVERY stress
                allowable above, and an explicit across-layer row is reported.
    pass iff SF >= safety_factor_required (default 2.0); ok = every evaluated rule passes.

**The creep rule is a table lookup, and it can refuse.** It reads
`creep.sig_allow_mpa` through `tools/analyzers/materials.py` — the one reader,
shared with the Rust contract — at the stated temperature and duration. Do not
restate the table here: its cells, its round-up semantics, the opt-in
interpolation, the refusal kinds and the recorded conflict with the legacy
`yield x creep_sustained_fraction` scalar all live in **[creep.md](creep.md)**,
which is the card for that number. What this card owes you is where it lands:
`allowable_mpa` on the creep row, with a `creep_cell` block naming the exact
cell and how it was reached, and `legacy_scalar_mpa` beside it — reported for
visibility, **never** served as the allowable.

The anisotropy rule is a **scalar-tier heuristic on the load DIRECTION only**
(angle between `primary_load_dir` and the layer plane). It is not a layer-normal
stress-tensor check; the receipt says so in `notes` on every run that fires it.
With no `orientation` in the job the rule is SKIPPED **with a reason** — never
silently.

## The contract

Job:

```
{"material": "PLA|PETG|ABS|ASA|TPU95A|PC|PA",      # REQUIRED (TPU→TPU95A, NYLON→PA)
 "max_von_mises_pa": 10e6,                          # REQUIRED — from a prior ace_fea
 "load_character"?: {"sustained": bool, "cyclic": bool},
 "duration_h": 8760,                                # REQUIRED WHEN sustained (no default)
 "service_temp_c"?: 25, "safety_factor_required"?: 2.0,
 "orientation"?: {"build_dir": [0,0,1], "primary_load_dir": [0,0,1]},
 "creep_interpolation"?: false}                     # opt-in, see creep.md
```

Receipt (LAST non-empty stdout line): `{ok, material, safety_factor_required,
anisotropy_derate_applied, rules: [{rule, allowable_mpa, demand_mpa, SF, pass,
detail, ...}], skipped: [{rule, reason}], notes, disclaimer}`. `ok` is the
OVERALL verdict: a structurally failing part still answers with the full
per-rule receipt.

Exit codes follow the shared contract: **0** ok · **1** the tool could not run
the request (unknown material → `error_kind: "internal"`, exit 1) · **2** it ran
and the verdict failed (`error_kind: "gate_failed"`) or a rule refused. The
three creep refusals surface as `error_kind` `refusal.creep_duration_required`,
`refusal.creep_temp_above_tabulated`, `refusal.creep_no_table`, each with
`allowable_mpa: 0.0`, `pass: false`, `refused: true` and the named
`refusal_kind` on the row.

## Benchmark gates (measured, frozen)

**There is no benchmark gate suite.** The registry records `gate: no` for this
analyzer (`python3 tools/analyzer_registry.py --tier production_check` returns
`"gate_suite": null`). What exists is (i) a hand-derived **validation pin**,
`tools/validation/production_check_validation.py`, which drives the real CLI and
re-derives every expected number in-file from a named table cell, (ii) the
tool's own `--selftest`, which is *self-consistency* — it proves the rules agree
with the tool's own reader, not with an independent reference — and (iii) the
540-probe cross-language creep pin, which covers the DEFAULT creep reader only.
Together they prove the **arithmetic and the refusal contract**. They prove
nothing about whether the table cells describe your filament.

| gate | what it pins | where |
|---|---|---|
| static + temp | PLA yield 55 / demand 10 → SF 5.5; temp 55/25 → SF 2.2; creep, fatigue and anisotropy SKIPPED with reasons; exit 0 | pin 1, `tools/validation/production_check_validation.py` |
| creep cells | 23 °C / 8760 h → cell `[23C][1y]` = 2.5 MPa (SF 2.5 at 1 MPa demand, `cell_match: "exact"`); 23 °C / 24 h → 5.0 MPa; 25 °C / 720 h rounds UP to `[55C][30d]` = 0.5 MPa → fails, exit 2, `gate_failed`; legacy 11.0 MPa reported and not used | pin 2, same file (cells: `tools/materials/pla.json`; card: [creep.md](creep.md)) |
| temperature limit | PLA at 60 °C vs limit 55 °C → temp row fails, SF 55/60, static still passes, `ok:false`, exit 2 | pin 3, same file |
| anisotropy derate | load along the build direction → across-layer static 55 × 0.55 = 30.25 MPa (SF 3.025 at 10 MPa) applied to the static row too; sustained cell 2.5 × 0.55 = 1.375 MPa; an in-plane load (0.0 deg) SKIPS the rule with a note and leaves yield at 55 | pin 4, same file |
| fatigue knockdown | PLA ultimate 60 × 0.3 = 18 MPa → SF 3.6 at 5 MPa | pin 5, same file |
| the three refusals | sustained with no `duration_h` → `creep_duration_required`; 70 °C sustained → `creep_temp_above_tabulated` (the 55 °C row is NOT served); PETG sustained → `creep_no_table`. Each: allowable 0.0, `pass:false`, exit 2, full per-rule receipt, legacy scalar reported never served (PLA 11.0, PETG 11.75 MPa) | pin 6, same file |
| could-not-run ≠ refusal | unknown material exits **1** with `error_kind: "internal"`, distinct from a refusal's 2 | pin 7, same file |
| opt-in interpolation | `creep_interpolation:true` at 30 °C / 24 h → 5.0 + (30−23)/(55−23) × (1.5−5.0) = **4.234375 MPa** with `basis: "interpolated"`, both bracketing cells and the default bucket (1.5) named — while the DEFAULT still reads 1.5 MPa and fails; 23 °C / 12 h log-linear → 5.545261 MPa; 70 °C still refuses | pin 9, same file |
| determinism | two runs of one job produce a byte-identical receipt | pin 8, same file |
| error band | rule arithmetic exact to 1e-9 MPa (compared at 1e-4 against the receipt's 4-decimal rounding) — **measured 0.0**; interpolation measured 2.5e-5 (rounding only); refusal contract measured as specified; `last_measured` 2026-09-02 | `validation.error_band`, `tools/manifests/production_check.manifest.json` |

## Validity limits / out of scope

- **It is not an analysis.** Every number it returns is a division of one
  upstream number by a table cell. Quoting a `production_check` SF as a
  structural result without the FEA receipt behind it is a category error.
- **The demand inherits its solver's error.** `ace_fea` under-predicts peak
  bending stress by ~20 % on coarse meshes and is ±20–30 %, biased HIGH, at
  fillets and notches (`docs/ANALYSIS_DOMAINS.md`). This tool applies no
  correction for that and does not know which regime the demand came from.
- **The allowables are datasheet-class typicals, not your filament.** The
  material rows are "typical desktop-FDM datasheet values (verify per brand)" by
  the db's own statement; the creep/fatigue knockdowns are engineering rules of
  thumb. **Not characterised:** no measurement in this repo ties any of these
  cells to a spool anyone printed, so there is no error band on the allowable
  side at all — the pinned band above covers the arithmetic only.
- **Anisotropy is scalar-tier**: a load-direction angle, not a layer-normal
  stress. A tensor check needs an ACE solver change, which would be a build.
- **Only PLA has a creep table**; every other material REFUSES a sustained
  verdict outright. See [creep.md](creep.md) — that is a declared gap, not a
  place to substitute the legacy 0.2-of-yield scalar.
- One scalar peak per job: no stress distribution, no location, no multi-load
  combination, no notch/size/surface factors.

## When to use it

Last, on every part, once you have an FEA (or measured) peak stress and know how
the part will be used: run it before you call anything shippable. Reach for it
specifically when the answer depends on something the stress field cannot see —
the load is HELD (state `duration_h`; the creep rule will refuse without it),
the load CYCLES, the part runs warm, or the load pulls across the layers rather
than along them. If it refuses, the refusal is the answer: state the duration,
declare the temperature honestly, or change material.
