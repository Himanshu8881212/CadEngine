# Analyzer manifest schema (`lmcad.manifest.v1`)

A **manifest** is the falsifiable spec of an analyzer: the governing equations it
claims to solve, the assumptions under which that claim holds, how boundary
conditions are interpreted, the units on the wire, and the pointer to the
validation pin that proves the claim against ground truth. It is the document a
result's `manifest_ref` points at (see `tools/provenance.py`).

A manifest is a prerequisite — **not** a substitute — for a validation pin. An
analyzer may be reported as tier `Validated` (by `tools/analyzer_registry.py`)
**only if it has BOTH a manifest AND at least one present validation pin**. A
manifest with no pin is a promise; a pin with no manifest is an unexplained
number. The graduation line requires both.

Manifests live in `tools/manifests/<analyzer_name>.manifest.json` where
`<analyzer_name>` is the registry name (e.g. `ace_fea`, not `ace_fea_runner`).
They are **data**, deterministic, and carry no wall-clock fields — the only
dates present are the fixed `last_measured` stamps copied from the validation
pins.

## Required top-level fields

| field | type | meaning |
|---|---|---|
| `schema` | string | must be `"lmcad.manifest.v1"` |
| `analyzer` | string | registry name, matches the key used by the registry |
| `analyzer_version` | string | semver of the analyzer contract this manifest describes |
| `title` | string | one-line human title |
| `governing_equations` | array of `{name, expr, description}` | the equations actually solved |
| `assumptions` | array of string | the modelling assumptions the result depends on |
| `boundary_conditions` | object | semantics of each BC kind the analyzer accepts (how selectors/fixtures/loads map to the math) |
| `units` | object `{inputs, outputs}` | unit of every named input/output quantity |
| `discretization` | object `{method, element, notes}` | how the continuum is discretised |
| `validation` | object | see below — the pin, the ground truth, the error band |
| `caveats` | array of string | honest known-limitations echoed by the tool description |
| `limits_of_validity` | object | parameter ranges outside which the pin does not vouch for the result |

## Optional fields

| field | type | meaning |
|---|---|---|
| `sources` | array of `{title, ref, used_for}` | the CITATIONS behind the equations — `ref` is a book+edition+section, DOI, standard number, or URL; `used_for` names the equation/gate it backs. **Required in practice for derived models** (`tools/analyzers/derived_model.py` refuses a source-less model at import time); recommended for everything else. |
| `model_file` | string | for derived-model manifests: the runnable model file the manifest describes (relative to `tools/`) |
| `derived_model` | bool | marks a manifest produced by the `tools/analyzers/derived_model.py` scaffold |

### `validation` sub-object

| field | type | meaning |
|---|---|---|
| `pin_file` | string | path to the `*_validation.py` that runs the check (the primary pin) — `tools/validation/<analyzer>_validation.py` since the 2026-09-02 layout |
| `additional_pins` | array of string (optional) | further pins (may be `pending` until a parallel agent lands them) |
| `ground_truth` | string | the closed-form / measured reference the pin compares against |
| `error_band` | object | the pinned error interval(s), e.g. `{"coarse": "-20%..0", "fine": "-10%..0"}` |
| `direction` | string | which way the discretisation error runs (e.g. "under-predicts, converges from below") |
| `last_measured` | string (date) | fixed date the band was last measured (NOT a clock read) |

## Status vocabulary (must match `tools/provenance.py`)

`validated` · `demonstrated` · `cataloged` · `synthesized_inloop` ·
`synthesized_unvalidated` · `research`. A manifest describes an analyzer whose
committed tier is one of `validated` / `demonstrated` / `cataloged`; the two
`synthesized_*` statuses are for on-the-fly analysis and are governed by the
synthesis guardrail in `docs/ANALYSIS_TIERS.md`, not by a committed manifest.

## Minimal validity check

`tools/analyzer_registry.py --check` enforces that every manifest present:
parses as JSON, has `schema == "lmcad.manifest.v1"`, and carries every required
top-level field above with a non-empty value. A `Validated` claim additionally
requires the `validation.pin_file` to exist on disk. Reference examples that
pass this check ship for the four structural analyzers
(`tools/manifests/ace_fea.manifest.json`, `ace_fea_tet.manifest.json`,
`ace_modal.manifest.json`, `ace_buckling.manifest.json`), both optimizers
(`param_optimize.manifest.json`, `ace_optimize.manifest.json`), and the three
rules/bookkeeping engines (`tolerance_stack.manifest.json`,
`production_check.manifest.json`, `production_dossier.manifest.json` — closed
forms with a hand-derived pin; `discretization.method` is "closed form" and the
`sources` block cites the textbook / table each rule is arithmetic over).

## Derived-model manifests (`tools/manifests/derived/`)

Manifests written by the `tools/analyzers/derived_model.py` scaffold (an agent's
on-the-fly physics model — see the 5-step loop in its docstring). Same schema,
plus `sources` (mandatory there), `model_file`, and `derived_model: true`.
Committing one to `tools/manifests/derived/<name>.manifest.json` AUTO-REGISTERS
the model on the graduation ledger at tier **Demonstrated** (manifest + inline
self-check gates that re-run on every invocation). It may claim **Validated**
only once `validation.pin_file` names a committed pin that exists on disk —
runtime results from a derived model always carry `validation_status:
synthesized_inloop`, never `validated`. An unparseable derived manifest fails
`--check` loudly; it is never silently skipped. Worked exemplar:
`tools/manifests/derived/damped_oscillator.manifest.json`.
