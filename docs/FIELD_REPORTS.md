# Field reports — the physical half of the flywheel

What happened to the part **in the real world**. "The lug cracked after two
months in the dryer." "The hinge whitened after 200 cycles." "It warped off
the bed." Until this stream existed the shop could not learn from field
failure: the engine captured software friction automatically
(`kernel_core::telemetry::log_friction`) and the
calibration wave captured printer FIT reality (`tools/ingest_calibration.py`),
but nothing ingested what the physical object did once it left the printer.

- **Corpus**: `docs/field_reports.jsonl` — append-only JSONL, one report per
  line, line 1 a self-describing `_schema` header that readers skip.
- **Intake**: `tools/field_report.py` (`--new`, `--list`, `--show`, `--stats`).
- **Triage**: `tools/field_triage.py` (`--id`, `--all`, `--campaign`).
- **Doctrine**: the five-step procedure
  an agent follows when the user says "it broke". Its law: **a field failure
  that does not become a gate is a lesson lost.**

## THE CORPUS IS EMPTY OF REAL DATA

Nothing in this repo is a real field observation. The three records shipped in
`docs/field_reports.jsonl` are **synthetic illustrations** whose only purpose is
to exercise and pin the pipeline. They are unmistakable by construction:

- `"example": true` on every one,
- ids prefixed `EXAMPLE-` (the intake REFUSES a record where the flag and the
  id prefix disagree — provenance cannot be fudged in either direction),
- every `observation.text` opens with `EXAMPLE (SYNTHETIC — not a real
  observation …)`,
- `--list` and `--stats` **exclude** them unless `--include-examples` is
  passed, and both print `0 REAL report(s)` regardless.

`python3 tools/field_report.py --stats` today prints "The REAL corpus is
EMPTY". It will keep saying that until the user reports something. Any future
claim about failure rates, MTBF, or "we see X% creep failures" that cites this
file before then is fabricated.

## Record schema (schema_version 1)

| field | required | notes |
|---|---|---|
| `id` | yes | stable handle, `[A-Z0-9][A-Z0-9._-]*`, unique in the corpus |
| `example` | yes | boolean; `true` ⇔ id starts with `EXAMPLE-` |
| `reported` | yes | ISO date `YYYY-MM-DD` |
| `schema_version` | yes | `1` |
| `part.family` / `part.entry` | yes | the campaign: `<family>_system/<entry>/` |
| `part.part` | no | which piece failed (free text) |
| `part.commit` | no | git short SHA / version the part was built from (auto-filled from HEAD) |
| `process.material` | yes | key into `tools/materials/*.json` (PLA, PETG, ABS, ASA, PC, PA, TPU95A) |
| `process.process` | no | `fdm` (default) |
| `process.layer_h_mm`, `.walls`, `.infill_pct` | no | print settings |
| `process.orientation_note` | conditional | REQUIRED for `layer_delamination` — which face was on the bed |
| `service.environment` | yes | where the part lived, free text |
| `service.temp_c` | conditional | REQUIRED for `warping`; drives the envelope check |
| `service.duration_h` | conditional | REQUIRED for `creep_deformation`, `chemical_uv` |
| `service.cycles` | conditional | REQUIRED for `fatigue_crack` |
| `service.load_n` | conditional | REQUIRED for `fracture` |
| `service.humidity_pct` | no | 0–100 |
| `observation.failure_mode` | yes | controlled vocabulary, below |
| `observation.location` | conditional | REQUIRED for `fit_loose` / `fit_tight` |
| `observation.first_observed_h` | no | must not exceed `service.duration_h` |
| `observation.text` | yes | the reporter's own words, ≥15 chars (≥60 for `other`) |
| `severity` | yes | `cosmetic` \| `degraded` \| `functional_loss` \| `safety` |
| `photo` | no | path to a photo |
| `classification` | derived | `design_failure` \| `condition_violation` — set by the intake, never by hand |
| `condition_violation` | derived | present only on a condition violation; carries the material limit that was exceeded |

### `failure_mode` vocabulary

| mode | means |
|---|---|
| `fracture` | broke apart under load, one event |
| `creep_deformation` | slowly changed shape under a sustained load |
| `layer_delamination` | split ALONG a layer line / interlayer bond |
| `wear` | material removed by rubbing over use |
| `fatigue_crack` | crack grew under repeated cycles, no single overload |
| `warping` | distorted from heat or residual stress |
| `chemical_uv` | environmental attack — solvent, oil, moisture, sunlight, ozone |
| `fit_loose` | mating fit looser than intended |
| `fit_tight` | mating fit tighter than intended |
| `other` | none of the above — requires ≥60 characters of description |

## Refusals (exit 1, `{"ok": false, "error": …, "errors": […]}`)

The intake is loud on purpose: a half-recorded failure produces a
half-remediation. It refuses missing required fields; an unknown
`failure_mode`, `severity` or material (each printing the valid set);
mode-specific evidence gaps (creep without a duration, fatigue without cycles,
fracture without a load, delamination without an orientation, a fit report
without a location, warping without a temperature); out-of-range numbers; and
self-contradictions — cycles accumulated in zero elapsed time, a failure first
observed after the part left service, an `example` flag disagreeing with the
id prefix.

### CONDITION VIOLATION — the loudest one

If `service.temp_c` exceeds the material record's `thermal.softening_c` (PLA
55 °C, PETG 68 °C, ABS 87 °C …), the part was run **outside its material
envelope**. That is not a design failure and must never be laundered into one.
The intake refuses by default with the arithmetic printed, and records the
report only under `--ack-condition-violation`, which stamps
`classification: "condition_violation"`. Triage then suppresses the campaign
re-audit (an out-of-envelope run cannot falsify an in-envelope gate), re-derates
nothing, and prescribes a stated + gated maximum service temperature instead.

## Triage: report → engineering consequence

`tools/field_triage.py` is deterministic — closed-form, no ML. Per report:

1. **Remediation plan.** `failure_mode` → the analysis that would have caught
   it, why it was missed, the data source, the solver to run (with a
   `present: true/false` flag, so a missing runner is stated rather than
   pretended), the allowable to re-derate, the gate to add, and the permanent
   design rule.

   | mode | analysis | solver | change |
   |---|---|---|---|
   | `creep_deformation` | sustained load at the SERVICE temperature and duration | `tools/analyzers/production_check.py` | re-derate against `creep.sig_allow_mpa[T][t]` |
   | `fatigue_crack` | cyclic life vs a cycle-count allowable | `tools/analyzers/ace_fatigue_runner.py`, `tools/analyzers/ace_fea_runner.py` | `sn_curve` knockdown; UNKNOWN off the tabulated point |
   | `fracture` | static strength margin at the break | `tools/analyzers/ace_fea_runner.py`, `tools/analyzers/production_check.py` | the reported load becomes the design load |
   | `layer_delamination` | interlayer allowable in the AS-PRINTED orientation | `tools/analyzers/production_check.py` | × `z_vs_xy_strength_ratio` |
   | `warping` | service temperature vs the softening envelope + DFM | `tools/analyzers/ace_thermal_runner.py`, `tools/analyzers/production_check.py` | stated + gated service limit |
   | `wear` | none — no wear solver exists in this tree | — | design rule + replaceable wear part |
   | `chemical_uv` | none — material selection against cited data | — | declared environment drives material choice |
   | `fit_loose` / `fit_tight` | tolerance stack vs a MEASURED profile | `tools/analyzers/tolerance_stack.py`, `tools/ingest_calibration.py` | tighten/open against measured deviation, ship a coupon |
   | `other` | UNCLASSIFIED — triage refuses to guess | — | classify first |

2. **Campaign re-audit.** The generated `<family>_system/<entry>/analysis/
   ANALYSIS.md` is parsed into labelled claims — table rows (`section / first
   cell`), bullets (`section / bolded span`), bolded prose — and each is tagged
   as a green ADEQUACY claim or, if it sits under an "out of scope" /
   "not performed" heading, as a declared GAP. A claim is reported as
   **contradicted** when it asserts adequacy and matches the failure mode
   strongly: two or more distinct mode words, OR one mode word plus a word
   naming the same part/location as the report. One generic word with nothing
   tying it to the part is demoted to `weak_matches` — shown, never hidden,
   never counted. Matching claims under a gap heading become
   `acknowledged_gaps`: honest in advance, and now due.

   Every contradiction is a **candidate**. The matcher reads vocabulary and
   adequacy wording, not physics; a human adjudicates each one and dismisses
   none silently.

3. **Priority.** `P0` = a green gate is contradicted (or `safety` severity) —
   the highest-value signal in the whole system. `P1` = a design failure with
   no contradicted gate (a missing analysis, not a wrong one), including one
   that lands on a declared gap. `P2` = condition violation or cosmetic.

Both a human-readable block and a machine JSON block are printed;
`--json-only` gives the JSON alone.

## Verify

```sh
python3 tools/field_report.py --self-test   # intake gates + every refusal
python3 tools/field_triage.py --self-test   # full pipeline on the EXAMPLE records
```

`field_report.py --self-test` also re-validates every shipped record and
asserts that all of them are labelled examples — so a real report can never be
committed as one of the synthetic seeds, and a synthetic seed can never drift
into looking real.
