# OPERATOR BRIEF — read this FIRST before any part campaign

Operating orders for design agents driving LMCAD. Everything here is condensed
from the five digests in `campaign/digests/` — exact JSON shapes live THERE,
not here. Repo root has a space: **always quote**
`"/Users/himanshu/Work/New-LMCAD/cad engine"`.

---

## 1. Engine mental model (10 lines)

1. LMCAD is one Rust kernel with two geometry halves: an **exact B-rep** side
   (analytic surface tags, persistent naming, loop-aware booleans, STEP) and an
   **implicit/SDF** side (expression trees, TPMS lattices, dual contouring).
2. You drive it with a JSON program: `{"ops":[{"id","op",...}]}` — ops run in
   order, first failure stops the run, exit 0 iff every op passed.
3. Only geometry-producing ops bind ids; measures/exports/asserts/lookups bind
   nothing (referencing one is a loud `missing_ref`) — but **every measure op
   is now its own gate** via the universal `require` param (2026-08-08):
   `{"op":"export_stl", …, "require":{"watertight":true,"route":"exact"}}`,
   `{"op":"support_report", …, "require":{"steep_area":0.0}}`,
   `{"op":"bounding_box", …, "require":{"fits_within":true}}`. An unmet
   expectation FAILS the run with `assert_failed` and exits 1; a met one
   echoes a `required` block into the measures. Expectation = scalar
   (equality), array (element-wise), or `{equals|min|max|within|not_null}`;
   keys may be dotted paths. Four mandatory SPEC §2 gates that used to be
   unexpressible in-program are now expressible — use them.
4. Units are mm; JSON-surface angles are degrees; bores are diameters.
5. Every measure carries `provenance` (`analytic` vs `faceted`); every export
   names its `route` (`exact` vs `voxel_healed`) and `watertight`.
6. The implicit→exact bridge is one-directional: fields leave only as meshes,
   never as B-reps (`HybridFuse` route `exact` is the one re-entry path).
7. The op catalogue is **161 ops** — `{"op":"describe"}` is authoritative and
   cannot drift; DESIGN_GUIDE Parts I–II and old error strings ("116 ops") are
   stale in places. When guide and binary disagree, trust the binary + API.md.
8. Physics lives OUTSIDE the JSON surface: Python runners in `tools/` consume
   exported/voxelized geometry. No FEA/load-case ops exist in programs.
9. Refusal is a first-class answer: when the kernel can't do a thing exactly,
   it refuses with a machine-matchable error `kind` instead of degrading.
10. **THE trap, now closed (2026-08-10)**: unknown/misspelled op-level params
    **FAIL the op** (`invalid_param`) — the engine refuses rather than letting
    a typo silently select a default. Keys starting with `_` are the comment
    convention and never warn; the universal `require` key is accepted
    everywhere. Keep the `exact_volume_within` gates anyway (they also catch
    geometric silent modes like absolutized negative radii).

## 2. The doctrine — receipts, not claims

- **No claim without a receipt.** Prose may only restate what a machine report
  says. Quote the number AND its provenance tag. Never quote the mesh volume as
  the analytic one or vice versa — they measure different objects.
- **Refusals are data.** `assert_failed` means fix the geometry or the wrong
  expectation — **never delete the gate**. A solver refusal (fatigue-PETG,
  modal-no-fixtures, snap-through) is recorded in the analysis record and
  routed around visibly, never laundered into a skipped analysis.
- **Silence is forbidden.** Every required analysis ends as: a receipt, a
  newly-gated derived model, or a bold **"required, NOT performed"** row
  (DESIGN_GUIDE §25.7). Physical tests have no solver substitute — unclaimed
  is the honest state.
- **Provenance discipline.** Analysis results carry `validation_status`; only
  registry-`validated` surfaces (ace_fea, ace_fea_tet, ace_modal,
  ace_buckling, ace_optimize, param_optimize, and the three pinned
  rules/bookkeeping engines tolerance_stack / production_check /
  production_dossier — "validated arithmetic", never "validated physics") may
  be quoted as validated. thermal/contact/fatigue are registered at
  Demonstrated / Demonstrated / Cataloged and in-house-gated — say so.
- **Negative controls.** Any retention/security/"can't fail" claim needs the
  failure attitude built and measured to interfere, plus the legal-path twin
  measured to clear. A gate that cannot fail is not a gate.

## 3. How to run things (exact commands)

```sh
# JSON program (the `run` subcommand is REQUIRED — bare path fails):
"/Users/himanshu/Work/New-LMCAD/cad engine/target/release/kernel-api" run prog.json --out-dir out/

# assembly file:
"/Users/himanshu/Work/New-LMCAD/cad engine/target/release/kernel-api" asm a.lmcasm --out-dir out/ [--tol MM] [--voxel MM] [--window MM]

# Python tools (plain python3; tools/_ace.py resolves ~/Work/ACE itself):
python3 "/Users/himanshu/Work/New-LMCAD/cad engine/tools/<tool>.py" job.json
```

- Report = stdout JSON, top level `{"api_version":"cadcode.v1","ok",...}`.
  `kernel-api run` exits **1** whenever `ok` is false — its exit code is
  trustworthy.

### 3.1 The exit-code contract — THREE codes, and `ok` and `$?` finally agree

The LAST non-empty stdout line is ONE JSON receipt; logs go to stderr. Since
**2026-08-08** every runner in `tools/` shares one contract
(`tools/_receipt.py` — read its docstring, it is the authority):

| exit | `ok` | meaning | `error_kind` |
|---|---|---|---|
| **0** | `true` | the analysis ran and the receipt is usable | — |
| **1** | `false` | the tool **could not run the request** — usage, unreadable job, internal error. **No analysis was performed.** | `usage`, `internal`, `receipt_path_conflict` |
| **2** | `false` | the tool **RAN and REFUSED**, or the analysis failed | `refusal.*`, `timeout`, `killed.*` |

**Any nonzero exit means "do not quote this receipt."** Both signals always
agree now — that is the whole point. Branch on `error_kind` (a machine-matchable
slug), never on prose. This replaces the old two-family split in which the six
ACE runners exited 0 on failure while an internal `KeyError` exited 1; the old
"parse `ok`, never `$?`" advice **still works unchanged** — `ok` never moved —
but `$?` now works too, and `1` vs `2` tells you whether anything was attempted.
Verified: `ace_fatigue_runner.py` with a malformed job → exit **2**,
`error_kind: "refusal.JobError"`; `tolerance_stack.py` on an interfering fit →
exit **2**; an unknown flag → exit **1**, `error_kind: "refusal.usage"`.

A caller that genuinely depends on exit-0-on-failure sets
`LMCAD_RUNNER_EXIT=legacy` (env) or `"legacy_exit_zero": true` (job key); the
receipt then carries `exit_contract.mode = "legacy"`, so the opt-out is itself
on the record. **Strict is the default everywhere. Do not set the opt-out in a
campaign** — an old script that needs it is the thing to fix.

The four shapes of silence this closed, and what you must still do:
- `tool.py job.json | tail -1 > receipt.json` — **never use this idiom.** The
  redirect truncates the target at LAUNCH, so an interrupted solve leaves a
  zero-byte file where a good receipt used to be. Use `--out PATH`, which
  writes atomically (temp + rename) at the END of the run.
- a job-level `"receipt"` key that disagrees with `--out` is now **REFUSED**
  (exit 1, `error_kind: receipt_path_conflict`) instead of quietly winning and
  clobbering a shipped file.
- a timeout or kill used to leave **no receipt at all**. Set
  `"wall_budget_s"` in the job (or `LMCAD_WALL_BUDGET_S`); SIGTERM/SIGINT and
  the budget both synthesize an honest `ok:false` receipt naming the limit
  instead of dying with a bare traceback. SIGKILL — what
  `subprocess.run(timeout=)` sends — cannot be caught by anyone, which is
  exactly why the budget belongs INSIDE the runner. **Put a wall budget on
  every long solve.**
- `LMCAD_RECEIPT_DRY_RUN=1` suppresses every on-disk write, so a what-if run
  against a shipped campaign can no longer mutate its evidence.

Still write automation as all three checks — the last stdout line parses as
JSON, `ok` is true, and the artefacts the receipt names exist on disk.
`kernel-api run` / `asm` exit 1 whenever `ok` is false (two codes, not three).

### 3.2 Path roots are NOT symmetric — the #1 cause of "reproducing does not reproduce"

| what | resolves against | refuses |
|---|---|---|
| `export_stl` / `export_step` / `export_*` `file` | **`--out-dir`** (sandboxed) | absolute paths, `..` |
| `import_step` / `import_mesh` / `load_part` `file` | **the PROGRAM file's directory, then `--out-dir`** (fallback = the T4 heal, 2026-08) | `..` — verbatim: `path '../out/b.step' must not contain '..' (it would escape the sandbox)` |
| `library_*` `dir` | `--out-dir` | — |
| every Python tool's job paths (`stl`, `out`, `npy`, …) | **the CWD you launched from** | — |

A STEP round-trip (§2.12) CAN be written as export-then-import with the same
relative name: the import looks beside the program first, then falls back to
`--out-dir` (the T4 heal). Program-relative files keep priority when both
exist. Doc contract updated 2026-08-27.

More consequences to plan around:
- `sweep_check` and `param_optimize.call_engine` write their generated station
  programs to a **system temp dir**, so `load_part`/`import_*` inside a station
  program resolves against `/var/folders/...`, not your campaign. Station
  programs must reference geometry they construct themselves, or take absolute
  inputs the runner passes in.
- Reports and receipts **echo the RESOLVED ABSOLUTE path** (kernel `io` errors,
  `render_sheet`'s `receipt.out`, the checkers' `receipt`). Those files are
  therefore not byte-comparable across machines or across two different
  `--out-dir`s. See §3 of DELIVERABLE_SPEC for what determinism actually
  guarantees.
- Always launch Python tools **from the repo root** and give job paths relative
  to it, so a reader can copy your "Reproducing" line verbatim.

- Doc/render tools take `date` as a string input, never the clock —
  re-renders and rebuilt STLs must be byte-identical.
- `production_dossier.py` / `render_sheet.py` / `assembly_doc.py` have no
  `--help` (they crash treating it as a file); read each tool's docstring.
  The docstring at the top of each `tools/*.py` outranks any digest.

## 4. Op-surface map — what exists, when to use which

| surface | use for | key facts | shapes in |
|---|---|---|---|
| **Exact B-rep ops** (constructors, sketches, booleans, fillet/chamfer, hole wizard, patterns, pose/mirror, measures, asserts, STEP) | everything dimensional: fits, bores, mating faces, anything you'll gate | validate-gated booleans (empty result = loud failure); persistent naming; `exact_volume` is π-exact; loft/sweep/patterns/bounding_box/measure_dimension ALL exist as ops (guide is stale) | `digests/ops_core.md` |
| **Implicit `implicit` op** (12 leaves / 20 combinators / 16 scalar ops, `expr_sdf`) | lattices, graded walls, real threads, organic blends — things exact booleans cannot do | binds NO solid — products are file + measures; `expr_sdf` requires `lipschitz_bound`; under-declared bounds = silent holes → `"mesher":"manifold"`; **pillow trap**: fillet/smooth unions bulge parallel/buried faces with green receipts — fuse first, cut last, probe datums | `digests/implicit_recipes.md` |
| **`.lmcpart` Document** | parametric parts, configs, persistent fillet EdgeNames, library admission | Dims are `{"Literal"}/{"Param"}`, no arithmetic; RADIAN corners (`CircularPattern.angle`, `ExtrudeSketch.draft`); Box is center+size; voxel-half roots refuse `load_part` | `digests/implicit_recipes.md` §6 |
| **`HybridFuse`** | lattice/pipe fused into an exact part that stays loadable/STEP-able | `route:"exact"` keeps exact faces verbatim; >50k-tri operand self-demotes to heal — coarsen the fuse voxel to re-enter exact | `digests/implicit_recipes.md` §7 |
| **`.lmcasm` + `asm_*` ops** | multi-part placement, mates, contacts, BOM | mates are authority, poses are seeds; mate residual >1e-6 FAILS the run but non-watertight instance exports exit 0 — gate the receipt yourself | `digests/implicit_recipes.md` §8 |
| **Catalog: 48 parts, 13 feature cuts, design-math lookups** | any standard hardware — check FIRST before modelling | origin/+Z, diameters, NO threads on bodies; out-of-table sizes refuse loudly; `iso286_fit`, `thread_spec`, `heatset_spec` return numbers as measures | `digests/implicit_recipes.md` §10 |

Rules of thumb: model in exact B-rep unless the feature is impossible there
(threads, TPMS, organic blends). Real threads only via `thread_ridge` /
`export_threaded` or the implicit groove idiom — exact booleans on helices
self-intersect. Prove disjointness with `union` + `assert shells==N` (the
tessellation-independent proof) or `assert_disjoint` (faceted, fixed for
nested pairs 2026-08-08 but see §8 for its under-read) — never
intersect-and-hope.

## 5. Analysis stack map (validity limits condensed)

| solver | what it is | error band / hard limits | tier |
|---|---|---|---|
| `ace_fea` | hex8 voxel linear-elastic | deflection −11%/−6% (coarse/fine, from below); **fillet PEAK stress ±20–30% high, never converges to Kt** — use closed-form Kt × FEA nominal | Validated |
| `ace_fea_tet` | body-fitted tet10 | resolves concentrations; fields UNSTRUCTURED (not grid-compatible); point loads only | Validated |
| `ace_modal` | frequencies + shapes | +1–3% high; refuses no-fixtures without `free_free:true`; YOU identify modes via participation receipts | Validated |
| `ace_buckling` | linear eigenvalue | λ is an UPPER bound; **mandatory 0.5 knockdown** → gate on `design_critical_load_n`; pair with a strength gate | Validated |
| `ace_thermal` | voxel conduction, steady+transient | conduction + user-supplied Robin film ONLY — no convection network/radiation/CFD; refuses unanchored components | in-house gated, NOT registered |
| `ace_contact` | planar corotational beam + rigid contact | snap-fits/latches; PLANAR only; friction untested; snap-through refuses | in-house gated, NOT registered |
| `ace_fatigue` | S-N screening | **PLA only**; refuses PETG/ABS/ASA/PA/PC/TPU and across-layer for ANY material; life scatter 3.7×–90×, quote it every time; ≤2e6 cycles | in-house gated, NOT registered |

### 5.1 READ THIS BEFORE YOU PLAN AN ANALYSIS: the body-fitted (tet) route is not a fallback

**8 of 10 campaigns lost wall-clock to `ace_fea_tet`, and none of them got the
number they planned it for.** It is the only independent check on a voxel
stress result, so it is worth attempting — but budget it as a *maybe*, plan the
closed-form bracket as your primary, and never make a deliverable depend on it.

What it actually does on kernel-exported geometry:
- **It refuses watertight, `valid`, `manifold`, `components==1` STLs the kernel
  itself signs off.** Observed refusals, verbatim:
  `Exception: PLC Error: A segment and a facet intersect at point` (slender
  facets left by an exact boolean — down to 0.00087 mm² in one case);
  `AssertionError: body-fitted mesh has a non-positive corner Jacobian
  (min -2.718e-03 mm^3) — inverted/degenerate element`.
- **One failure mode is not a refusal at all — it is a process abort**:
  `libc++abi: terminating due to uncaught exception of type std::runtime_error:
  Failed to reach critical value in pass 0 for measure(s): ScaledJac`, with NO
  stdout and no receipt. That breaks the wire contract; treat "empty receipt
  file" as a tet failure, not as your bug.
- **Sub-3 mm features usually have no usable `elem_size_mm` window.** Coarse
  enough to afford → inverted elements (refusal); fine enough to mesh → past
  `direct_max_dof` (250 000) onto the iterative path. A 2.2 mm flexure arm
  meshed cleanly at 1.6 mm (63 965 tet10 / 112 304 nodes / ~337 k DOF) and did
  not finish in ~45 min. Another campaign pinned a core at ~79 % for >28 min
  before being killed. 360 k DOF exceeded 10 GB.
- It is **not bit-deterministic** (superlu), and its fields are UNSTRUCTURED —
  they are not grid-compatible with `ace_fea` / `graded_infill`.

Operating orders:
1. **Probe across element sizes before concluding anything about the geometry.**
   A refusal at one `elem_size_mm` says nothing about the next.
2. Mesh the **sub-model**, not the assembly. Cutting a blade off its disk turned
   a hard abort into a clean 8 748-tet solve — and say which support bound the
   sub-model represents (a rigid clamp at the cut is the LOWER-support, i.e.
   conservative, bound).
3. Give every tet attempt a wall-clock budget you decided in advance, and
   **ship the refusal as a result** (`receipts/..._ATTEMPTED.json` +
   `refusals.json`) rather than letting the campaign block on it.
4. The peak from a rigid clamp is nodal-recovered and mesh-dependent; the
   receipt carries no convergence field, so a single tet run is not a converged
   peak. Two element sizes or no convergence claim.

MUST-REFUSE domains (no in-tree solver, fenced): CFD/aero, 3-D EM,
convection/thermal-CFD, nonlinear 3-D FEA, crack growth, freeform-face field
analysis. New physics only via `tools/derived_model.py` (citations at import,
gates re-run every invocation, capped at `synthesized_inloop`).

Anisotropy: no solver models layers — derate the ALLOWABLE via
`materials.derated()` (z/xy = 0.55 above 30° out-of-plane), never E.
Details + job schemas: `digests/analysis_honesty.md`, `digests/tools_cookbook.md`.

## 6. Optimization paths — when each is justified

- **`param_optimize.py`** (Validated, stdlib): `"$name"` substitution over
  template ops, dotted-expression objectives/constraints over receipts,
  `targets` / `multi_start` / `robust` worst-case modes, and a v2 command
  evaluator that can drive ANY receipt-emitting analyzer. **Default choice**
  whenever the design has named parameters — cheap, deterministic,
  feasibility-first. This is the easiest way to satisfy the mandatory
  optimization gate honestly.
- **`ace_optimize`** (SIMP topology, Validated): justified only when the
  material layout itself is the unknown (bracket webbing, stiffness/mass).
  Mark load/fixture regions `frozen` or SIMP eats them. Result is a MESH only
  — no B-rep reconstruction; downstream is print-only.
- **`graded_infill`** (Demonstrated): stress-graded gyroid infill from an
  `ace_fea` field ON THE SAME GRID (shape mismatch refused, never resampled —
  pin the frame with `voxelize_stl.py`). Justified for interior mass savings
  on parts whose envelope is frozen. Verify supports in a slicer.

## 7. Print-readiness — PLA on a Bambu-class printer

Stock reality: 0.4 mm nozzle, **256 mm bed** (gate `bounding_box` with
`envelope: [256,256,256]`), PLA-only filament, HDT ~54 °C, creep is real.

- **Walls**: gate `wall_thickness {flag_below: 1.6}` (4 perimeters at 0.4);
  judge `thin_area` + `p05_thickness` — `min_thickness` is corner noise.
- **Overhangs**: `support_report {build_dir}` per intended orientation; gate
  `steep_area == 0.0` exactly for a "support-free" claim. Never set
  `overhang_deg` equal to a modelled face angle (f32 knife edge) — use the
  printer's real limit and treat readings within 1° of a modelled angle as
  unresolved. Horizontal bores → `teardrop_hole`; counterbores facing down →
  `bridged_counterbore` (drill the 0.3 mm membrane after printing).
- **Fits**: house FDM tolerance is ±0.15 mm. A 0.2 mm nominal clearance
  INTERFERES at extremes (verified) — design running clearances ≥ 0.3 mm and
  prove every interface with `tolerance_stack.py`. Bearing seats are nominal
  line-to-line: apply `iso286_fit` allowance to YOUR bore.
- **Thermal**: only allowed temperature claims are conduction-solver results +
  softening_c = 55 °C. PLA near/above 55 °C is a condition violation, not a
  design case. State service temperature on every part.
- **Creep — the surface, and the trap in it.** Sustained load is a CREEP case,
  never a yield case. One reader, one table:

  ```python
  import sys; sys.path.insert(0, "tools")
  import materials as M
  M.creep_allowable_mpa("PLA", 23, 8760)                      # -> 2.5   (bare scalar)
  M.creep_allowable_mpa("PLA", 23, 8760, across_layer=True)   # -> 1.375 (x0.55 applied FOR you)
  M.creep_lookup("PLA", 25, 720)                              # -> the RECEIPT; ship this
  ```

  `tools/materials/pla.json#creep.sig_allow_mpa` is a **two-row step table**,
  and this is the whole trap:

  | | 1 h | 24 h | 30 d | 1 y |
  |---|---|---|---|---|
  | **23 °C** | 7.5 | 5.0 | 3.5 | 2.5 |
  | **55 °C** | 3.0 | 1.5 | 0.5 | 0.5 |

  There is **nothing between 23 °C and 55 °C**, and the reader rounds the
  service temperature **UP to the next tabulated row**. So a 25 °C declared
  ambient reads at the **55 °C** row: `creep_lookup("PLA", 25, 720)` returns
  `0.5` MPa with `row_used_c: 55.0`, `cell_match: "rounded_up_conservative"`
  and the note *"state 55C, not 25.0 C, as the temperature this margin holds
  at"*. A campaign that declares 25 °C ambient and quotes the 23 °C row is
  wrong by 7×. **Declare 23 °C or accept the 55 °C row — there is no middle.**
  Above 55 °C the reader **REFUSES** (`refusal_kind:
  "creep_temp_above_tabulated"`, no fallback to the 55 °C row).
  - **Always ship the `creep_lookup` receipt, not the scalar.** It carries
    `row_used_c` / `col_used_h` / `cell_match` / `extrapolated` / `refused` /
    `anisotropy_factor` / `material_hash` — i.e. *which cell your margin was
    read at*, which is the number a reviewer actually needs.
  - `across_layer=True` applies the 0.55 ratio inside the lookup; do **not**
    also multiply by 0.55 yourself.
  - The legacy `thermal.creep_sustained_fraction` = 0.2 (≈11 MPa) is a
    recorded conflict, surfaced as `legacy_scalar` in the receipt. **The table
    governs**; the scalar exists to quote the conflict, never to gate on.
  - `production_check.py`'s verdict is a *static-strength* verdict. Do not
    read it as a creep verdict — run the creep gate separately, with a stated
    design duration.
- **Strength**: static allowable 55 MPa yield (XY, datasheet-class); fatigue
  UTS 40.9 MPa; margin FEA demand by its ~20% under-read. Run
  `production_check.py` on every FEA result.
- Voxel choice for implicit/heal routes: 0.3–0.4 mm FDM production;
  ≥3 voxels across every wall/strut; fine threads 0.06–0.12.

## 8. Failure playbook, condensed

- Error kinds are machine-matchable: `parse, unknown_op, duplicate_id,
  missing_ref, wrong_type, invalid_param, feature_failed, sketch_failed,
  invalid_geometry, admission_rejected, dependents_exist, assert_failed, io,
  internal`. Execution stops at the first failure — one root cause.
- Empty boolean = loud `invalid_param`, never an empty solid → check poses
  before retrying. But see the next bullet: for a *nested* pair the pose
  checkers lie.
- **`clearance` on NESTED pairs was fixed 2026-08-08 — and it UNDER-reads.**
  It used to return `distance: 0.0` for nested/coaxial/enclosed pairs with a
  real gap. It now returns a number, but a `faceted` one: a Ø11.4 pin coaxial
  in a Ø12 bore (true 0.300 mm radial gap) reads **0.2711 mm**, a −9.6 %
  under-read from inscribed polygonal facets (≈ `r·(1−cos π/n)`).
  `assert_disjoint` passes the same pair now. Publish the faceted number with
  its provenance tag for "does it clear"; when a few percent decides the fit,
  use the **grown-gauge bracket** — grow a copy of the moving body by δ and
  intersect; `intersection` REFUSES while δ < clearance and binds an
  `exact_volume` once δ > clearance, so two runs bracket the gap with
  `analytic` provenance. Worked example in `DELIVERABLE_SPEC` §2.11.
- `feature_failed` on fillets/chamfers: convex straight edges between planar
  faces only; concave junctions refused; rim fillets convex rims only and the
  picker has NO distance guard — measure after. **Ease edges on primitives
  first, boolean last.**
- Boolean hygiene: cut bores on virgin primitives; fuse coplanar bodies last;
  overshoot cutters past faces; embed ≥0.1 mm or coincide exactly — never the
  1e-6–0.1 mm sliver; polygon segments ≥ ~1.5 mm; float contact poses 0.1 mm.
- Silent modes (exit stays 0) → tripwires: misspelled optional param → volume
  window; negative radius absolutized → volume window; hole loop crossing
  outer → volume window; wizard cut near a wall → `wall_thickness` + volume
  window; rim fillet snap → measure after; blend pillow → datum probe;
  under-declared Lipschitz → manifold mesher; assembly export
  `watertight:false` at exit 0 → gate the receipt.
- **Hardest silent failure: a part severed into floating lumps passes
  validity, watertight and the volume window.** Gate EVERY part with **BOTH**
  `{"op":"assert","in":X,"shells":1}` **and**
  `{"op":"assert","in":X,"components":1}`, and use the `mesh_components`
  measure to diagnose. They are complementary, not redundant — and **not in
  the direction older docs claimed.** Measured: a `difference`-severed bar
  reads `shells 2` *and* `components 2` (both fire); two boxes 0.0005 mm apart
  read `shells 2` but `components 1` (only `shells` fires — `mesh_components`
  welds any sever narrower than `weld_tol` 0.001 mm shut);
  `extrude_with_holes` with 2 holes reads `shells 1` while `mesh_components`
  REFUSES (`invalid_geometry`: the tessellated surface is not closed, so a
  count would be faceter cracks, not bodies — it used to return a false 3). Full
  measured table, the weld-scale band, and the two constructible
  oracle-negative-controls: `DELIVERABLE_SPEC` §2.2 and §2.13. Keep
  tapered-cutter apexes strictly inside material, and keep any designed
  ligament well clear of the 0.001 mm weld scale.
- Sweeps prove free motion ONLY — blind to steady interference. Must-NOT-fit
  claims belong on exact `overlap_volume` in the posed failure attitude.
- Hit an engine/tool/doc bug? Log it per the friction protocol in
  `DELIVERABLE_SPEC.md`. **Never edit engine or tools source.**

## 9. Deep-dive pointer table

| topic | go to |
|---|---|
| exact ops: constructors, sketches, booleans, fillets, hole wizard, patterns, measures, asserts, exports | `campaign/digests/ops_core.md`; API.md; DESIGN_GUIDE §4–§10, §21, §23 |
| implicit trees, expr_sdf/Lipschitz, lattices, threads, pillow trap | `campaign/digests/implicit_recipes.md` §1–§5; DESIGN_GUIDE §11–§15 |
| `.lmcpart` grammar, HybridFuse, `.lmcasm`, library, catalog | `campaign/digests/implicit_recipes.md` §6–§10; DESIGN_GUIDE §16–§20 |
| solver cards, trust tiers, refusal lists, material/creep data | `campaign/digests/analysis_honesty.md`; docs/ANALYSIS_TIERS.md, docs/ANALYSIS_DOMAINS.md; tools/solvers/*.md |
| every tool's job schema + verified examples | `campaign/digests/tools_cookbook.md`; docstring at top of each `tools/*.py` |
| finished-campaign layouts, gate taxonomy, negative controls, process lessons | `campaign/digests/exemplars.md`; showcase/squatchee_spin/; camera_system/card_magazine/; docs/FRICTION.md |
| what each campaign must ship | `campaign/DELIVERABLE_SPEC.md` (the contract) |
| the two connectivity oracles, the weld-scale limit, the two constructible oracle-NCs | `DELIVERABLE_SPEC` §2.2 + §2.13; `digests/ops_core.md` "ENGINE UPDATE" |
| `support_report` semantics (`describe` ships empty docs) | `digests/ops_core.md` §11a; `DELIVERABLE_SPEC` §2.5; DESIGN_GUIDE §22 |
| `clearance` on nested pairs + the grown-gauge bracket | `digests/ops_core.md` §11b; `DELIVERABLE_SPEC` §2.11 |
| exit codes, receipt persistence, wall budgets | §3.1 above; `tools/_receipt.py` docstring (the authority) |
| what determinism actually guarantees (`determinism.core_digest`) | `DELIVERABLE_SPEC` §3; `digests/analysis_honesty.md` |
| **are these docs still true?** | `python3 docs/test_doc_contracts.py` — 22 executable doc contracts run against the live binary and tools; if one fails it names the doc section that has gone stale |
| numerics, determinism, f32/f64, Lipschitz contract | docs/NUMERICS.md |
| op param truth at runtime | `{"op":"describe","name":"<op>"}` |
