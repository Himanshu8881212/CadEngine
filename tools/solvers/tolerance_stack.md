# tolerance_stack — will this chain still close, and this bore still clear, at the printer's extremes?

**Runner**: `tools/analyzers/tolerance_stack.py` (shim: `tools/tolerance_stack.py`) ·
`python3 tools/analyzers/tolerance_stack.py job.json [--out PATH]`
**Gates**: `python3 tools/validation/tolerance_stack_validation.py`
(shim: `tools/tolerance_stack_validation.py`) ·
`python3 tools/tests/test_checkers.py`
**Status**: green — both re-run 2026-09-04 in this worktree: *"tolerance_stack
validation: ALL PINS OK"* and *"CHECKER PINS: ALL 30 OK"*.
**Tier**: **Validated** (`python3 tools/analyzer_registry.py --tier tolerance_stack`;
`kind` = rules_engine, so this is *Validated arithmetic*, not physics — the pin
proves the stack maths against a hand derivation, not the printer).

The question is not "does the CAD model fit" — in CAD everything fits at
nominal. It is "does it still fit when the housing came out 0.2 mm deep, the
spacer 0.15 mm thin and the shaft 0.15 mm fat, all at once". That is the
question that decides whether an assembly binds, rattles, or has to be reprinted,
and it is arithmetic nobody does reliably in their head: the shipped portfolio
contains a bore/shaft pair (8.2 over 8.0, 0.2 mm nominal clearance) that reads
as generous and **interferes** at default FDM tolerance. The tool answers it
twice — once as a guaranteed bound (worst case) and once as a 3-sigma
statistical band (RSS) — and never lets the two verdicts be confused.

It is also the analyzer that most often has to say "I refuse": a chain with no
functional limit, a misspelled `closes` key, an asymmetric `tol` missing a side.
Each of those used to be a `KeyError` in a persisted receipt (`din_rail F1`);
each is now a named refusal.

## What it actually computes

Closed-form arithmetic over a 1-D chain. No geometry, no engine call, no
sampling — no Monte Carlo anywhere.

    gap:          g_nom  = sum_i dir_i * n_i                       (dir_i = +1 | -1)
    worst case:   g_min  = g_nom - sum_i lo_i ,  g_max = g_nom + sum_i hi_i
                  hi_i   = plus_i  if dir_i = +1 else minus_i
                  lo_i   = minus_i if dir_i = +1 else plus_i       (about the TRUE nominal)
    RSS:          t_eq_i = (plus_i + minus_i)/2 ,  shift = sum_i dir_i (plus_i - minus_i)/2
                  g_rss  = g_nom + shift ,  sigma_g = sqrt( sum_i (t_eq_i/3)^2 )
                  band   = g_rss +/- 3 sigma_g
    fit:          c_min  = (bore_nom - bore_minus) - (shaft_nom + shaft_plus)
                  c_max  = (bore_nom + bore_plus)  - (shaft_nom - shaft_minus)
    ranking:      band_contribution_i = plus_i + minus_i, as a % of the band width

Baked-in assumptions, from `tools/manifests/tolerance_stack.manifest.json`: the
chain is one-dimensional and linear (no angular, projected or form-tolerance
terms); worst case assumes all extremes can coincide and claims **no**
probability; RSS assumes independent contributors whose +/-t is a 3-sigma band
of a normal process centred on the mid-shifted nominal. The two conventions are
kept apart in the receipt on purpose — before 2026-08-08 the RSS mid-shift was
applied to the shared nominal, which moved the *worst-case* band by
(plus-minus)/2: pessimistic on the low side and **optimistic on the high side**
(`ball F4`).

An element with no `tol` takes `printer_tol_default`, **0.15 mm** — a typical
well-tuned desktop-FDM figure and, as the manifest says in as many words, *"a
stand-in for a measurement, not a measurement"*.

## The contract

Job (`chain`, `fit`, or both in one job):

```
{"chain": [{"name", "nominal", "tol": t | {"plus","minus"}, "dir": +1|-1}, ...],
 "closes": {"min_required"?, "max_allowed"?},      # either side may be omitted (one-sided)
 "fit":    {"bore": {"nominal","tol"?}, "shaft": {"nominal","tol"?}},
 "printer_tol_default"?: 0.15}
```

Receipt (LAST non-empty stdout line; logging on stderr):
`chain{nominal_gap, worst_min, worst_max, rss_nominal_gap, rss_sigma_gap,
rss_min, rss_max, rss_convention, closes{min_required, max_allowed,
sides_checked}, pass_worst, pass_rss, contributors[] (ranked by band width, with
pct_of_band / pct_of_worst), asymmetric_note?}` and
`fit{nominal_clearance, min_clearance, max_clearance, extremes{max_shaft,
min_bore}, interference_at_extremes, pass}`. Values are rounded to 9 decimals.
`ok` = every requested mode passes.

Exit codes are the shared contract (`tools/_receipt.py`): **0** ok · **1** the
tool could not run the request · **2** it ran and refused *or* the stack failed.
A failed verdict carries `error_kind: "internal"` with `exit_code: 2` (measured
here on the 1.4 +/-0.15 vs `min_required` 3.0 job); refusals carry
`refusal.<kind>`: `refusal.missing_closes`, `refusal.bad_closes`,
`refusal.empty_closes`, `refusal.bad_tol`, `refusal.bad_dir`,
`refusal.missing_nominal`, `refusal.bad_element`, `refusal.empty_chain`,
`refusal.bad_fit`, `refusal.empty_job`, plus `receipt_path_conflict` when
`--out` disagrees with a job `receipt` key. `LMCAD_RECEIPT_DRY_RUN=1` suppresses
every on-disk write.

## Benchmark gates (measured, frozen)

| gate | what it pins | where |
|---|---|---|
| symmetric 3-element chain | 20.0+/-0.20 − 12.0+/-0.10 − 7.5+/-0.15 → nominal 0.5, worst [0.05, 0.95], RSS 0.5 +/- sqrt(0.0725) = +/-0.269258240 → [0.230741760, 0.769258240], sigma 0.089752747; contributors rank 44.4 / 33.3 / 22.2 % of the band | pin 1, `tools/validation/tolerance_stack_validation.py` |
| asymmetric mid-shift | 10.0 +0/−0.10 (dir +1) − 9.0 exact → worst [0.90, 1.00] about the TRUE nominal 1.00, while `rss_nominal_gap` = 0.95 and sigma = 0.05/3; `asymmetric_note` present | pin 2, same file; `ball F4` row in `tools/tests/test_checkers.py` |
| the dangerous direction | 10.0 +0.10/−0 − 9.0 → `worst_max` 1.1 against `max_allowed` 1.05: `pass_worst` false, exit 2 (pre-2026-08-08 this passed) | `tools/tests/test_checkers.py` |
| worst fails / RSS passes | same chain at `min_required` 0.10: `pass_worst` false (0.05 < 0.10), `pass_rss` true (0.230742 ≥ 0.10), `ok:false`, exit 2 | pin 3, validation script |
| fit extremes | bore 8.2 +/-0.15 vs shaft 8.0 +/-0.15 → clearance [−0.10, +0.50], `interference_at_extremes` true, exit 2; bore 8.5 (tol omitted → 0.15 default) → [0.20, 0.80], exit 0 | pin 4, validation script |
| one-sided `closes` | `{min_required: 1.0}` alone is first class: `sides_checked = ["min_required"]`, `worst_min` 1.25 (1.4 − 0.15); a misspelled key is `refusal.bad_closes`, never an unbounded limit | `tools/tests/test_checkers.py` (`din_rail F1`) |
| refusal + exit contract | chain with no `closes` → `refusal.missing_closes`, exit 2; failing verdict exits 2 and a passing one 0; a mistyped flag is refused; a dry run cannot clobber a shipped receipt; `--out` vs job `receipt` → `receipt_path_conflict` and neither file written | pin 5 + `tools/tests/test_checkers.py` (`gripper F9`, `cleat F7`, `singulator F14`) |
| determinism | two runs of one job produce a byte-identical stdout receipt | pin 6, validation script |
| error band | chain arithmetic and fit arithmetic exact to 1e-9 mm — **measured 0.0**; determinism measured identical; `last_measured` 2026-09-02 | `validation.error_band`, `tools/manifests/tolerance_stack.manifest.json` |

## Validity limits / out of scope

- **1-D only.** No angular stack-up, no form tolerance, no datum shift, no
  projected chains. A tilted mating face is a chain the user must build by hand —
  or a new solver.
- **RSS is only as good as independence + normality.** A printer whose whole
  batch runs 0.1 mm large violates both. `pass_rss` true with `pass_worst` false
  is a STATISTICAL pass and must be quoted that way; the manifest states the
  ~0.27 % per-3-sigma-side tail *conditional on those assumptions holding*, and
  nothing in this repo measures whether they hold for any real printer.
- **The 0.15 mm default is not a measurement.** No study in this repo
  characterises the dimensional tolerance of any printer. Treat it as a
  placeholder for your own calibration data, and do not gate a claim on the
  default when the part is a press fit.
- **Fit mode is two cylinders' extremes**, not a fit CLASS: it does not consult
  ISO 286 tables, and it says nothing about roughness, ovality, or the elastic
  interference of a press fit.
- **As-designed, not as-printed**: no elephant's foot, no first-layer squish, no
  shrinkage model. Those are inputs you must fold into the tolerances yourself.

## When to use it

Any time two printed parts have to meet at a dimension you cannot adjust after
printing: a bearing seat depth, a shaft through a bore, a lid lip in a groove, a
minimum thread engagement, a snap-fit's residual gap. Run it BEFORE the first
print — the contributor ranking names the one dimension worth tightening — and
quote which band you passed on: a worst-case pass is a guarantee, an RSS-only
pass is a statistics claim about a batch you have not measured.
