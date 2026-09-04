# Analysis tiers, synthesis guardrails, and the research fence

*(Companion: [`ANALYSIS_DOMAINS.md`](ANALYSIS_DOMAINS.md) — the capability
contract: which physics domains are pinned in-tree, which are derivable with
citations via `tools/analyzers/derived_model.py`, and how the external-solver bridge
works.)*

This is the graduation pipeline's constitution. It says what it means for an
analysis number to be trustworthy, forbids confident-but-unvalidated numbers
from reaching a user unmarked, and draws a hard boundary around the physics we
deliberately do NOT ship yet.

Three moving parts implement it:

- **`tools/provenance.py`** — the result envelope (`lmcad.analysis.v1`), the
  deterministic geometry content-hash, and the checkers (`check_envelope`,
  `check_synthesized`).
- **`tools/analyzer_registry.py`** — the ledger of every analyzer, its tier, its
  manifest/pin status, and the system health metric.
- **`.github/workflows/analysis-gate.yml`** — CI enforcement.

Manifests are specified in `docs/MANIFEST_SCHEMA.md`.

---

## 1. The analysis-result contract (the envelope)

Every analyzer result that surfaces to a human should be wrapped by
`provenance.stamp(...)` into one envelope:

```
{
  "schema": "lmcad.analysis.v1",
  "values": <the analyzer's receipt>,
  "residual_or_convergence": <structured receipt — NEVER a bare scalar>,
  "self_check": <{limit, expected, obtained, passed} | null>,
  "manifest_ref": "tools/manifests/<analyzer>.manifest.json" | null,
  "geometry_relation": <equality | derived_from | null>,
  "provenance": {
    "geometry_hash":     "program:sha256:<hex>" | "mesh:sha256:<hex>",
    "material_version":  "<version string>",
    "analyzer_name":     "ace_fea",
    "analyzer_version":  "1.0.0",
    "validation_status": "validated" | "demonstrated" | "cataloged"
                       | "synthesized_inloop" | "synthesized_unvalidated" | "research"
  }
}
```

`stamp()` **refuses** a bare-scalar `residual_or_convergence` and an unknown
`validation_status` — the contract is enforced at construction, not by
convention.

The physics runners additionally record the clean/dirty state and commit of the
LMCAD checkout, Python/platform and numerical-package versions, a SHA-256 for the
hash-locked dependency file, hashes of the exact runner/solver source files, and
the exact sampled grid or Tet10 node/connectivity hash. The supported environment
is Python 3.11 plus `tools/requirements-analysis.lock`. A dirty checkout, missing
lock, non-empty `PYTHONPATH`, package imported outside the selected Python prefix,
or validation-range miss automatically downgrades a requested `validated` claim to
`demonstrated`. `LMCAD_REQUIRE_REPRODUCIBLE_ANALYSIS=1` makes those same conditions
a refusal before meshing or solver artifacts are produced.

Until 2026-09-04 the solvers lived in a separate ACE checkout and a second commit
pin (`tools/ACE_REVISION`) had to match, which no hosted runner could satisfy. The
solver source now lives in this repository at `tools/analyzers/physics/` (still
Apache-2.0 — see its `NOTICE`), so **`lmcad.commit` IS the solver's revision** and
one clean checkout is the whole reproducibility claim.

### Geometry content-hash (determinism)

`provenance.geometry_hash(...)` is a pure function of geometry **content**:

- **program** — a work-order program is canonicalised with **sorted JSON keys**
  and compact separators, so object-key order is irrelevant (list/op order is
  preserved because it is semantic). Yields `program:sha256:<hex>`.
- **mesh** — a binary STL is canonicalised **order-independently**: the header
  and per-facet normals are dropped, each triangle's vertices are cyclically
  rotated to start at the lexicographically smallest vertex (orientation
  preserved), and the triangle list is sorted, so facet emission order and
  vertex-listing order do not change the hash. Yields `mesh:sha256:<hex>`.

No wall-clock time ever enters an envelope or the hash.

### Equality vs derived-from (the honest relation distinction)

Two representations of "the same" solid do **not** share a hash. The relation
between them is stated as data:

- **`equality`** (`equality_relation`) — the analysis ran on exactly the
  referenced geometry: same rep, same hash, no error bound.
- **`derived_from`** (`derived_from_relation`) — the analysis geometry was
  produced by a bridge (B-rep program to STL tessellation to voxel occupancy),
  with a **stated `error_bound`** (chord tolerance, voxel size, ...). The two
  hashes differ on purpose.

Both types exist today as data even though the tessellation/voxelisation bridges
do not yet emit the relation automatically — the honest structure is in place
first.

---

## 2. The tiers

A number is only as trustworthy as the evidence behind the analyzer that made
it. Three tiers, and the registry **cannot inflate** them: a `Validated` claim
with no committed manifest+pin is auto-downgraded and recorded as a violation
the CI gate fails on.

| tier | bar | what backs the number |
|---|---|---|
| **Validated** | manifest **and** >= 1 present validation pin | the result is checked against an independent closed-form / measured ground truth, with a documented error band and a demonstrated convergence direction |
| **Demonstrated** | runs end-to-end + a built-in self-check/sanity gate | confidence comes from the engine's own validated measures and internal consistency — **not** from a ground-truth pin |
| **Cataloged** | deterministic rules/arithmetic over cited tables/formulas | correct-by-construction relative to its sources; neither a physics simulation nor pinned; only as good as the tables and the (usually 1-D) assumptions |

The corresponding envelope `validation_status` values are `validated`,
`demonstrated`, `cataloged` (plus the two synthesis statuses in section 3 and
the research status in section 4).

### The current ledger (run `python3 tools/analyzer_registry.py`)

Pins live in `tools/validation/`, the analyzers they pin in `tools/analyzers/`
(renderers/emitters in `tools/publish/`, gate suites in `tools/tests/`; the
2026-09-02 layout, mapped by `tools/_layout.py`, with a forwarding shim at
every old flat `tools/<name>.py` path). The table names pins by basename.

| analyzer | tier | manifest | pin |
|---|---|---|---|
| ace_fea | Validated | yes | yes (`ace_fea_validation.py`; Kt pin `ace_fea_kt_validation.py`) |
| ace_fea_tet | Validated | yes | yes (`ace_fea_kt_tet_validation.py` — Peterson/Pilkey stepped-bar Kt, converging from below) |
| ace_modal | Validated | yes | yes (`ace_modal_validation.py`) |
| ace_buckling | Validated | yes | yes (`ace_buckling_validation.py`) |
| ace_optimize | Validated | yes | yes (`ace_optimize_validation.py` — exact inequalities: OC descent, material-removal monotonicity, volume honesty, watertight STL) |
| ace_thermal | Demonstrated | no | no (gate suite `test_ace_thermal.py`, not a registered pin) |
| ace_contact | Demonstrated | no | no (gate suite `test_ace_contact_fatigue.py`) |
| ace_fatigue | Cataloged | no | no (gate suite proves the Miner arithmetic, not the life) |
| graded_infill | Demonstrated | no | no |
| param_optimize | Validated | yes | yes (`param_optimize_validation.py` — analytic known optimum, target convergence, active constraint, byte-identical determinism) |
| air_topology_audit | Demonstrated | no | no |
| sweep_check | Demonstrated | no | no |
| balance_check | Demonstrated | no | no |
| joint_check | Cataloged | no | no |
| tolerance_stack | Validated | yes | yes (`tolerance_stack_validation.py` — hand-derived textbook worst-case + RSS stacks, asymmetric mid-shift, fit extremes; exact) |
| production_check | Validated | yes | yes (`production_check_validation.py` — material-table cells x the documented rules, the three creep refusals, exit contract; exact) |
| production_dossier | Validated | yes | yes (`production_dossier_validation.py` — analytic box STLs: exact volume/area, the shell mass model by hand, thick-section warning, packing, refusal) |
| damped_oscillator | Demonstrated | yes | no (derived-model exemplar, auto-registered from `tools/manifests/derived/`) |

**System health metric — "% of analysis surface below the validated line":**

- analysis surface (registered analyzers): **18**
- Validated: **9** => **50.0% above** the validated line
- **50.0% below** the validated line (9 of 18 analyzers not yet validated)

This number is intentionally, honestly conservative. The four structural
solvers are pinned to closed-form physics and both optimizers are pinned to
analytic optima / exact physics inequalities (2026-07-17 — the tools that
DRIVE design decisions sit above the line); the three rules/bookkeeping
engines campaigns gate on (`tolerance_stack`, `production_check`,
`production_dossier`) were graduated on 2026-09-02 by pinning their arithmetic
to hand derivations — **Validated there means "the arithmetic is proven
against an independent hand derivation", not "a physics simulation"; their
`kind` column still reads rules_engine / reporting.** The remaining audit and
rules surfaces are useful and deterministic but have not been graduated. Agent-derived
physics models (`tools/analyzers/derived_model.py`) enter the ledger automatically when
their manifest is committed under `tools/manifests/derived/` — at Demonstrated,
BELOW the line, until someone lands a ground-truth pin (see
`docs/MANIFEST_SCHEMA.md`). The metric is
count-weighted (each analyzer = one unit of surface — a stated simplification;
surfaces are not weighted by usage or blast radius). It is the scoreboard the
pipeline exists to move.

Renderers, codegen, and geometry bridges (`render_sheet`, `analysis_sheet`,
`assembly_doc`, `motion_gif`, `voxelize_stl`, `gen_discover`, `bom_audit`,
`make_all_plate`) are **not** analysis surface and are excluded from the
denominator with a documented reason (see `NON_ANALYSIS` in the registry).

---

## 3. Synthesis guardrails — the mandatory pattern for on-the-fly analysis

When the model **synthesises a new analysis on the fly** (writes and runs solver
code for a question no catalogued analyzer answers), the danger is a
confident-looking number with no provenance. The following pattern is
**MANDATORY**. `provenance.check_synthesized(envelope)` is the lightweight
checker that enforces the envelope-visible parts of it, and **nothing that fails
it may surface to a user unmarked.**

A synthesised result MUST, in order:

1. **Emit a manifest BEFORE reporting.** State the governing equations and
   assumptions first (a `manifest_ref` to a written manifest, per
   `docs/MANIFEST_SCHEMA.md`). No equations stated => do not report a number.
2. **Run a self-check against a known limit.** Reduce the synthesised model to a
   case with a known answer (a coarse/fine convergence pair, a far-field limit
   that must recover the un-featured value, a zero-load/rigid-body sanity case)
   and record it as `self_check = {limit, expected, obtained, passed}`. `passed`
   must be `True`.
3. **Report a residual / convergence receipt, never a bare number.** Emit the
   structured `residual_or_convergence` (iteration count + relative residual, or
   the coarse-vs-fine deltas). A lone scalar is refused by `stamp()`.
4. **Execute sandboxed.** Run synthesised solver code in an isolated process /
   sandbox (e.g. the ACE runner subprocess boundary, or a Vercel-sandbox-style
   microVM), never in-process with the kernel — a synthesised solver is
   untrusted code.
5. **Pin solver versions and seeds.** Record the solver/library versions in the
   manifest and set deterministic seeds; a synthesised result that cannot be
   reproduced bit-for-bit is not a result.
6. **Stamp `validation_status` on every synthesised result.** Use
   `synthesized_inloop` only when the self-check passed; otherwise
   `synthesized_unvalidated`, which `check_synthesized` treats as *must not
   surface without a warning banner*. A synthesised analysis can **never** claim
   `validated` — that status is reserved for a committed manifest+pin that
   passed CI.

`check_synthesized` returns `(ok, problems)` and is `ok` only when: the envelope
is structurally valid, `validation_status` is a synthesised status (never
`validated`), a `manifest_ref` is present, a `self_check` object is present and
`passed is True`, and a real `residual_or_convergence` receipt is present. The
CI gate and any surfacing layer should call it and refuse to display a result it
rejects.

**Graduation path:** a synthesised analysis becomes a first-class analyzer only
by landing (a) a committed manifest in `tools/manifests/`, (b) a committed
`tools/validation/*_validation.py` pin against ground truth, and (c) a registry row — at which
point the registry (and CI) can report it `Validated`. Until all three exist, it
stays synthesised and marked.

---

## 4. The research fence — physics we deliberately do NOT ship yet

Some analysis domains are **out of scope on purpose**. They are declared
`research`-tier: **hard-gated, ship-nothing-unmarked.** This is a scope boundary,
not an accident or an oversight — writing them down here IS the implementation
of "don't build them yet, fence them."

The fenced domains, and why each is off-limits until it earns a manifest+pin:

- **Aerodynamics / external flow** — drag, lift, boundary layers. No validated
  solver exists in this repo; hand-wavy CFD numbers are worse than none.
- **Thermal / conjugate-heat / thermal-CFD** — transient thermal, natural/forced
  convection, coupled fluid-thermal. Not built; the FDM material data carries
  temperature deratings (a Cataloged rule in `production_check`), which is the
  ONLY temperature claim allowed today.
- **Electromagnetics / motor magnetics** — B-fields, torque-from-flux, eddy
  losses. The drives (cyclo/harmonic/planetary) are analysed kinematically and
  structurally only; magnetic performance is fenced.
- **NURBS / freeform-surface field analysis** — running FEA/CFD directly on
  trimmed NURBS or freeform B-rep faces. The validated path is
  occupancy-voxelised hex8; freeform-exact field analysis is fenced.

Rules for anything inside the fence:

1. **No number surfaces unmarked.** Any `research`-tier output must carry
   `validation_status = "research"` in its envelope and be presented as
   exploratory, with the fence cited.
2. **No auto-promotion.** A `research` domain cannot be lifted to `Demonstrated`
   or `Validated` by a demo alone — it requires the full graduation path
   (section 3): a committed manifest, a committed ground-truth pin, a registry
   row, and CI.
3. **The registry and CI enforce the ceiling.** `research` is not an
   auto-inflatable status; like `synthesized_*` there is no code path from it to
   `validated` without the committed evidence.

The fence is a feature. It keeps the below-the-line honesty of section 2 (50%
of the surface as of 2026-09-02) from quietly eroding into confident numbers
about physics we have not validated.
