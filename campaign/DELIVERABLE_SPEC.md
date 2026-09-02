# DELIVERABLE SPEC — the contract every part campaign must ship

Binding on every design agent. A campaign that does not meet this spec is not
done, whatever its geometry looks like. Read `campaign/OPERATOR_BRIEF.md`
first. Repo root (quote it): `"/Users/himanshu/Work/New-LMCAD/cad engine"`.

---

## 1. Naming and directory layout

Campaigns live at `<domain>_system/<part_name>/` under the repo root
(e.g. `camera_system/card_magazine/`, `printer_system/spool_hub/`).
Names: lowercase snake_case; the part directory is the campaign unit.

Required layout (card_magazine convention):

```
<domain>_system/<part_name>/
├── README.md            # mechanism, contents table, print settings,
│                        #   "What has NOT been done" section (mandatory)
├── parts/               # final print files: <part_name>.stl (+ .3mf where useful)
│                        #   MUST regenerate byte-identical from programs/
├── cad/                 # <part_name>.step (exact AP203 export, round-trip gated)
├── programs/            # EVERYTHING needed to regenerate and re-verify:
│                        #   part program(s), assembly program, tol_*.json,
│                        #   sweep_*.json, balance/dossier/sheet/asmdoc jobs,
│                        #   physics job.json files
├── receipts/            # curated verdict-relevant receipts: gate outputs,
│                        #   sweep CSVs, solver receipt JSONs, bom_dossier.{csv,json}
├── analysis/
│   ├── DESIGN.md        # written BEFORE geometry is trusted (§25.7):
│   │                    #   problem, dimension-freeze table (every constant
│   │                    #   sourced), the analysis PLAN, defects-caught log,
│   │                    #   recorded refusals, open questions
│   └── ANALYSIS.md      # regenerated EVERY run from live receipts — measured
│                        #   numbers only; may never be hand-edited stale prose
├── renders/             # render_sheet/render_views PNGs
├── assembly/            # MANDATORY, single-part campaigns included:
│                        #   ASSEMBLY_assembly_doc.png (exploded/ballooned
│                        #   diagram), ASSEMBLY_instructions.md,
│                        #   bom_dossier.{csv,json}; scene/ STLs where the
│                        #   part has distinguishable bodies
└── README.md "Reproducing" section: exact command lines to rebuild everything
```

**Every campaign ships `assembly/` — single-part campaigns included**
(maintainer directive, 2026-08-27). The BOM and the ballooned diagram are
the artifacts a builder actually opens; a single part still has a mounting
or unfolding sequence, a coupon, and a mass line, so generate them with
`tools/production_dossier.py` (BOM into `assembly/`) + `tools/assembly_doc.py`
(the diagram + instructions), and wire both into `run_all.sh`. For a
print-in-place part, export the interlocked bodies as `assembly/scene/*.stl`
(print pose) and balloon those — state in the steps that the exploded view
is documentation, not an assembly task. Follow
`school_system/rated_desk_hook/` (single part) and
`school_system/folding_book_stand/` (print-in-place) as exemplars, and
`showcase/squatchee_spin/` / card_magazine for true multi-part.

Embed the design record IN the programs: top-level `"part"`, `"notes"`,
`"receipts"` keys are ignored by the engine — use them for rationale,
load-bearing warnings (boolean order, polygon density) and expected receipt
values so a re-run can be diffed against the shipped claim.

## 2. Minimum gate checklist — every part, no exceptions

Each gate is an in-program `assert`/measure or a tool receipt saved under
`receipts/`. A recorded-but-unchecked measure is worthless.

**Use `require` — every measure op is now its own gate (2026-08-08).** Until
this landed, `assert` spoke only `volume_within / exact_volume_within / genus /
shells / components / closed / manifold / valid`, so four mandatory gates below
(§2.4 route/watertight, §2.5 `steep_area == 0.0`, §2.6 `thin_area`, §2.7
`fits_within`) **could not be in-program gates at all** and had to be checked
out of band. They can now. `require` is a universal optional param on every op
that emits measures:

```json
{"op":"export_stl",      "in":"part", "file":"part.stl",
                          "require":{"watertight":true, "route":"exact"}},
{"op":"support_report",  "in":"part", "build_dir":[0,0,1],
                          "require":{"steep_area":0.0, "max_bridge_span":{"max":5.0}}},
{"op":"wall_thickness",  "in":"part", "flag_below":1.6,
                          "require":{"thin_area":0.0, "p05_thickness":{"min":1.6}}},
{"op":"bounding_box",    "in":"part", "envelope":[256,256,256],
                          "require":{"fits_within":true}},
{"op":"mesh_components", "in":"part", "weld_tol":0.0001,
                          "require":{"components":1}}
```

`describe` (authoritative): *"Universal gate: `{"<measure key>": expectation}`
checked against this op's own measures; the op FAILS (`assert_failed`) when an
expectation is unmet. Expectation = scalar (equality), array (element-wise), or
`{equals|min|max|within|not_null}`. Keys may be dotted paths."*

Verified 2026-08-08: an unmet `require` fails the run with
`assert_failed: require failed: max_bridge_span: measured 10.0, expected <= 5`
and the process exits **1**; a met one echoes a `required` block into the
measures, so the receipt records what was gated, not just what was measured.
**A gate you only record is not a gate — put the expectation in the program.**

1. **Validity**: `validate` → `valid`, `closed`, `manifold`, expected `genus`
   and `shells` asserted (not just recorded).
2. **Connectedness — assert BOTH `shells` and `components`.** They are two
   *complementary* oracles computed from two different objects. Neither
   subsumes the other, and a campaign that ships only one has an unmeasured
   blind spot.

   ```json
   {"op":"assert","in":"<part>","shells":1},
   {"op":"assert","in":"<part>","components":1}
   ```

   | | `assert shells:N` | `assert components:N` |
   |---|---|---|
   | source | `validate()` on the **exact B-rep** — counts closed shells | `mesh_components` on the **tessellated mesh** (`tol` 0.05) after a vertex weld at `weld_tol` 0.001 mm |
   | provenance | topological / exact | `faceted` |
   | catches what the other misses | a sever **narrower than the weld scale** (the mesh welds it shut and `components` reads 1) | disconnection that only exists in the *mesh you actually print or voxelize* — the faceted body is the object every downstream STL, slicer and voxel solver sees |
   | when it cannot be trusted | — | **REFUSES loudly** (`invalid_geometry`) rather than over-counting — see below |

   **Measured on this kernel (2026-08-08, `target/release/kernel-api`):**

   - A `difference`-severed bar reads `shells 2` **and** `components 2` — both
     gates fire. Earlier revisions of this spec claimed a severed part passes
     the `shells` gate; that claim was **wrong** and is retracted. Five
     campaigns copied it into their own docs as if it were their own
     measurement; all five receipts in fact said `shells 2`.
   - The regime where the two disagree is the opposite one: two boxes with a
     **0.0005 mm** gap read `shells 2` but `components 1`. In that regime
     **`components` is the weaker check.**
   - `extrude_with_holes` with 2 holes: `shells 1`, `genus 2`, `valid true` —
     and `mesh_components` (and `assert components:1` with it) **REFUSES**
     with `invalid_geometry` rather than silently returning 3. Verbatim
     (2026-08-08): *"the connectivity oracle cannot be trusted on this solid —
     tessellating it at tol 0.05 mm left 16 boundary edges (28 triangles), so
     the measurement surface is NOT closed and its component count (3) counts
     faceter cracks, not severed bodies. … Gate this part with `validate`
     (closed / manifold / shells) meanwhile, and/or `export_stl` it and run
     this measure on the export's bound mesh — the exported mesh IS what
     prints."* This is refuse-never-degrade working correctly: the op used to
     return a false 3. **Follow the refusal's own instruction** — keep
     `shells:1` live, and run the connectivity walk on the exported mesh,
     which is the object that actually prints. Record the refusal verbatim in
     ANALYSIS.md; do not delete the gate. **Update 2026-08-27** (fixlog
     2026-08-27-pose-robustness): after the f64 piercing-predicate fix the
     2-hole plate's tessellation closes (boundary_edges 0), so
     `mesh_components` returns the TRUE count (1) and the gate passes — the
     refusal channel remains for genuinely unclosed tessellations, and the
     doc contract pins the new behavior.

   **The weld-scale limit — state it if your part has any thin ligament.**
   `mesh_components` welds near-coincident vertices before walking
   connectivity, so a sever narrower than `weld_tol` is welded shut and
   invisible. Measured (two 10 mm boxes separated by `gap`, `union_all`;
   `shells` = 2 in every cell):

   | gap (mm) | 0.0001 | 0.0005 | 0.0009 | **0.0010** | 0.0011 | 0.0015 |
   |---|---|---|---|---|---|---|
   | components, faces at x = 0 | 1 | 1 | 1 | 1 | 2 | 2 |
   | components, faces at x = 10 | 1 | 1 | 1 | **2** | 2 | 2 |
   | components, faces at x = 100 | 1 | 1 | 1 | 1 | 2 | 2 |

   Read it as: **welded (invisible to `components`) strictly below `weld_tol`
   = 0.001 mm, separate strictly above it; exactly AT the tolerance the f32
   mesh coordinates decide, so it is coordinate-dependent.** Any designed
   ligament, sever or tapered-cutter apex at or below 0.001 mm is invisible to
   the connectivity gate and must be gated on `shells` (and on a volume
   window). Do not design a feature whose correctness depends on the exact
   boundary.

   `mesh_components` accepts `tol` (chord tolerance of the measurement
   tessellation, default 0.05) and `weld_tol` (default 0.001 — its `describe`
   doc now reads *"Position-weld scale (mm) for vertex identity … the house
   weld scale"*). Verified: `weld_tol 0.0001` turns the 0.0005 mm pair from
   `components 1` into `components 2`, and the receipt echoes the value used.
   **`assert` takes the same two knobs** (since 2026-08-08), with defaults
   identical to `mesh_components`' — both read them through one shared helper,
   so the GATE and the DIAGNOSTIC printed beside it can never be tuned apart.
   Omit them and `assert components:1` runs exactly as it always has. Either
   spelling gates a tighter walk; use whichever reads better:

   ```json
   {"op":"assert","in":"<part>","components":1,"weld_tol":0.0001}
   {"op":"mesh_components","in":"<part>","weld_tol":0.0001,"require":{"components":1}}
   ```

   One more thing the connectivity oracle will tell you rather than guess: it
   needs a CLOSED measurement surface. If the tessellation of the bound solid
   has boundary edges, the count is counting faceter cracks rather than bodies,
   so the op FAILS `invalid_geometry` and names the two routes that still
   work — `validate` (shells) and the same measure on the exported mesh. A
   *winding* defect (`non_orientable_edges` > 0 while `boundary_edges` is 0)
   leaves every triangle in place and every vertex shared, so it cannot move the
   count: it is reported in the measures, never refused.
3. **Volume window**: `assert exact_volume_within` against a closed-form
   target you computed yourself (band, never equality). This is the tripwire
   for geometric silent modes (absolutized radii, hole loops crossing outer).
3b. **Zero warnings**: since 2026-08-06 every report entry carries a
   `warnings` array when the op was given params it does not accept. A shipped
   campaign has ZERO warnings across every program report — grep your receipts.
4. **Watertight export**: `export_stl` receipt `watertight: true`, `route`
   quoted (`exact` preferred; `voxel_healed` is honest but must be noted).
   Assembly instance exports exit 0 even when leaky — gate the receipt.

   Know what you are gating. `watertight` here is **edge closure** — every
   undirected edge used by exactly two triangles — which is the property a
   slicer needs and the one the op refuses without. It does **not** test that
   the triangles agree on which way is out. The receipt says so itself
   (`watertight_means`) and reports `boundary_edges`, `non_orientable_edges`
   and `two_manifold` beside it. History: measured 2026-08-08, 26 of 41
   export ops wrote files carrying 3–395 non-orientable edges (the boolean
   tessellator's winding defect). **Fixed in engine round 4** (per-triangle
   analytic winding + fold-refusal, `campaign/fixlog/H-census-round4.md`) and
   the portfolio re-baselined: **measured 2026-08-24, 0 of 65 shipped export
   files carry any non-orientable edge**. Residual, disclosed: sliver facets
   along transverse intersection curves can still mis-wind at very fine chord
   tolerances (the boolean's carved-face decomposition family); exports demote
   to the voxel heal honestly and `mesh_components` now emits
   `non_orientable_witness` midpoints so any recurrence is locatable. Quote
   `watertight`; gate `two_manifold` when downstream depends on consistent
   normals — it now holds across the shipped portfolio.
5. **Support/overhang report**: `support_report` with the explicit intended
   `build_dir`; a "support-free" claim requires `steep_area == 0.0` exactly;
   quote `max_bridge_span`. `describe` ships **empty `doc` strings** for both
   parameters, so the measured semantics are pinned down in
   `digests/ops_core.md` — read them before you write orientation prose. The
   two that cost four campaigns wall-clock:
   - **`build_dir` points AWAY from the bed** (it is the layer-growth
     direction). `build_dir [0,0,1]` puts the bed at min-Z. Verified: an
     L-bracket with a 5×10 foot at z=0 reports `bed_area 50.0` at
     `[0,0,1]` and `bed_area 200.0` (its two top faces) at `[0,0,-1]`.
     `render_sheet`'s `build_dir` uses the same convention, so a sign error
     here ships a render of the wrong bed.
   - **A LARGER `overhang_deg` is MORE permissive, not stricter** (default
     45). A downward face is counted in `steep_area` iff its tilt from
     `build_dir` **exceeds** `overhang_deg`. Verified on lofted frusta: a wall
     tilted 63.4° from the build direction is steep at `overhang_deg` 63 and
     clean at 64; a 45° wall is steep at 44 and clean at 45. The comparison is
     strict, so a value equal to a modelled face angle sits exactly on an f32
     knife edge — never set `overhang_deg` to a modelled face angle, and treat
     any reading within 1° of one as unresolved. A "second, stricter reading"
     means a **smaller** number.
6. **Wall thickness**: `wall_thickness {flag_below: 1.6}` (0.4 nozzle, 4
   perimeters); judge `thin_area` + `p05_thickness`. Mandatory after every
   hole-wizard cut (the wizard has zero wall-proximity awareness).
7. **Mass properties + bed fit**: `mass_properties` receipt shipped;
   `bounding_box` with `envelope: [256,256,256]` → `fits_within` true.
8. **At least one honest physics analysis** with its error band stated in
   ANALYSIS.md: e.g. ace_fea (state the −20% coarse under-read; use Kt ×
   nominal for notches), modal, buckling (0.5 knockdown mandatory), thermal,
   contact, or a gated derived model. Quote `validation_status`/tier honestly.
   If load is sustained: gate against `creep_allowable_mpa(T, hours)` and
   state the design duration — never the static allowable.
9. **At least one optimization with receipts**: `param_optimize` (default),
   `ace_optimize`, or `graded_infill` — job + receipt in `programs/` +
   `receipts/`, and the chosen optimum's constraints re-verified by the
   normal gate suite (an optimizer receipt is not a validity receipt).
10. **Fit stacks at ±0.15 mm printer extremes for EVERY mating interface**:
    `tolerance_stack.py` FIT and/or CHAIN mode; pass = no interference at
    extremes (or the designed interference in its stated band). Running fits
    need ≥ 0.3 mm nominal clearance at default tolerance.
11. **Negative controls for any retention/security/"cannot" claim**: pose the
    failure attitude and measure interference (`overlap_volume > 0`), pose
    the legal path and measure clearance. Float exact-contact poses by
    0.1 mm. Sweeps prove free motion only — must-NOT-fit claims live on exact
    overlap volume. Ship both numbers.

    **`clearance` on nested pairs was FIXED on 2026-08-08 — and it is
    `faceted`, so it UNDER-reads.** It used to return `distance: 0.0` for
    nested/coaxial/enclosed pairs with a real gap (six campaigns fell back to
    other measures because of it). Re-verified on the same case that broke it —
    a Ø11.4 pin coaxial inside a Ø12 bore, a true 0.300 mm radial gap:

    ```
    clearance(tube, pin)      -> {"distance": 0.2711080312728882,
                                  "interfering": false, "overlap_volume": 0.0,
                                  "provenance": "faceted"}
    assert_disjoint(tube, pin) -> PASSES (it used to false-fail)
    ```

    **Quote it, but quote the provenance with it.** The reading is 0.2711 mm
    against a true 0.300 mm — a **−9.6 %** under-read, because the measure runs
    on inscribed polygonal facets (the error scales as
    `r·(1 − cos(π/n))` ≈ 0.029 mm here). That is the *conservative* direction
    for a clearance claim, so it is publishable as-is — but never call it the
    analytic gap, and never quote it as the design clearance for a fit where
    3 % matters. `tol` does not materially move it (0.271108 at default vs
    0.271108 at `tol` 0.001).

    **When you need an ANALYTIC number: the grown-gauge bracket.**
    `intersection` on a genuinely disjoint pair refuses
    (`invalid_param: intersection produced an empty solid`), so an empty
    intersection is itself a machine-checkable receipt. Grow a copy of the
    moving body by δ and bracket the clearance between a δ that still refuses
    and a δ that binds:

    | δ (radius growth) | result | reading |
    |---|---|---|
    | 5.99 − 5.7 = 0.29 mm | `intersection` **refused**, `invalid_param`, exit 1 | radial clearance > 0.29 mm |
    | 6.01 − 5.7 = 0.31 mm | binds; `exact_volume` = 6.0369 mm³, `provenance: analytic` | radial clearance < 0.31 mm |

    Ship both runs — the refusing one is evidence, not a failure to hide. The
    clearance is bracketed to `[0.29, 0.31]` mm with **analytic** provenance,
    which is what a tight fit deserves; the faceted 0.2711 is what a coarse
    "does it clear at all" question deserves.
12. **STEP round-trip**: `export_step` then `import_step`; volume conserved
    within 2.5% (asserted). Run coplanar coalescing before export where
    boolean chains fragmented faces.

### 2.13 Oracle negative controls — one per ORACLE, not just per part

A gate that has never been observed to fail is not evidence. For each oracle
you lean on, ship a deliberately-broken twin program whose ONLY difference is
the defect, and show it **exits 1** with the `assert_failed` message quoted.
Put the twins in `programs/` and their reports in `receipts/`.

The two connectivity oracles are non-redundant in **opposite** directions, so
proving them takes **two** twins. Both of these are verified to run on this
binary (2026-08-08) — copy them and swap in your own geometry.

**NC-A — `components` fires, `shells` would have fired too** (a coarse sever;
the ordinary severed-part case):

```json
{"ops":[{"id":"bar","op":"box","min":[-20,-5,-5],"max":[20,5,5]},
        {"id":"knife","op":"box","min":[-1,-10,-10],"max":[1,10,10]},
        {"id":"cut","op":"difference","a":"bar","b":"knife"},
        {"id":"c","op":"mesh_components","in":"cut"},
        {"id":"g_comp","op":"assert","in":"cut","components":1}]}
```
→ `c: components 2, is_one_body false`; `g_comp` fails
`assert_failed: components: measured 2, expected 1`; **exit 1**.

**NC-B — `components` PASSES on a severed body and only `shells` fires**
(the sub-weld-tol sever; this is the twin that proves the two gates are not
redundant, and it is the replacement for the un-constructible example this
spec used to give):

```json
{"ops":[{"id":"a","op":"box","min":[0,0,0],"max":[10,10,10]},
        {"id":"b","op":"box","min":[10.0005,0,0],"max":[20,10,10]},
        {"id":"u","op":"union_all","in":["a","b"]},
        {"id":"c","op":"mesh_components","in":"u"},
        {"id":"g_comp","op":"assert","in":"u","components":1},
        {"id":"g_shell","op":"assert","in":"u","shells":1}]}
```
→ `c: components 1, is_one_body true`; `g_comp` **passes**; `g_shell` fails
`assert_failed: shells: measured 2, expected 1`; **exit 1**.

Use the same 0.0005 mm offset on your own sever plane to make NC-B
part-specific. Note the sever must sit at a coordinate whose magnitude puts it
inside the weld band (see the table in §2.2) — at a face on x = 0 the same
0.0005 mm gap already reads `components 2`, which turns NC-B back into NC-A.
Check the measured `components` in the receipt; do not assume.

Other oracles that must have a twin if you rely on them: the volume window
(perturb one dimension past the band), `steep_area == 0.0` (re-run
`support_report` at the orientation you rejected), and any must-NOT-fit
interference claim (§2.11).

## 3. Honesty rules

- **No claim without a receipt.** Every number in README/ANALYSIS/listings is
  copied from a file in `receipts/` or a program report — cite which.
- **Provenance tags quoted.** `analytic` vs `faceted`, export `route`,
  `volume_source`, solver `validation_status` — verbatim, never paraphrased
  upward. Only registry-Validated surfaces may be called validated; check
  `python3 tools/analyzer_registry.py`, not the solver's own README.
- **Validity limits stated** next to every physics number: error band,
  refused regimes, anisotropy handling (allowable derated ×0.55 out-of-plane,
  E untouched), fatigue scatter band (3.7×–90×) whenever a life is quoted.
- **Refusals recorded, never laundered.** A solver/kernel refusal goes into
  DESIGN.md/ANALYSIS.md with the verbatim reason and the visible route
  around. `assert_failed` → fix geometry or expectation, never the gate.
- **"What has NOT been done"** — mandatory README section naming every
  unperformed-but-relevant analysis and physical test (drop/shake, thermal
  cycling, across-layer fatigue = unknowable, etc.). Silence is the one
  forbidden outcome.
- **Determinism — what is actually guaranteed.** Re-run your own "Reproducing"
  commands before declaring done, and hold each artefact to the right standard.
  A blanket "byte-identical receipts" rule is **silently unmeetable** on this
  toolchain, so it is not the rule.

  | artefact | standard | how you check it |
  |---|---|---|
  | STLs / STEPs from `kernel-api` | **byte-identical** — mandatory | `cmp` |
  | PNGs/GIFs from `render_sheet` / `assembly_doc` / `analysis_sheet` / `motion_gif` | **byte-identical** — mandatory, and this is why every doc job carries a literal `date` **string**, never the clock | `cmp` |
  | any tool receipt (checkers AND solvers) | **`determinism.core_digest` identical** — mandatory | compare that one field, never the receipt bytes |
  | tool receipt BYTES | **not comparable, ever** | don't |

  **Never `cmp` a receipt.** Every runner receipt now carries a
  `determinism` block (`tools/_receipt.py`, schema `lmcad.determinism.v1`):

  ```json
  "determinism": {
    "schema": "lmcad.determinism.v1",
    "nondeterministic_paths": ["timings_s"],
    "core_sig_figs": 12,
    "core_digest": "sha256:5b790443f2…",
    "solver_reproducibility": "<what this particular solver guarantees>",
    "how_to_compare": "compare `determinism.core_digest` between runs, NOT the receipt bytes…"
  }
  ```

  `core_digest` is a sha256 over the payload with the named
  `nondeterministic_paths` stripped and every float quantized to
  `core_sig_figs` significant figures. **That is the byte-comparison campaigns
  were told to make and could not.** Verified 2026-08-08: two runs of the same
  `ace_fatigue` job produced identical `core_digest` while `timings_s`
  differed.

  Two structural reasons a raw receipt diff will always fail, both now
  disclosed in the receipt itself rather than discovered the hard way:
  1. every ACE receipt embeds a wall-clock `timings_s` block;
  2. the numerics are not bit-reproducible — `ace_modal` reruns differ by
     ~1e-13 to 2.6e-12 relative, and `ace_buckling` and the
     `ace_fea_tet`/superlu path likewise.

  So the honest reproducibility claim for a solver number is **"`core_digest`
  reproduces; the payload reproduces to `core_sig_figs` significant
  figures"** — never "byte identical". Read
  `determinism.solver_reproducibility` and quote it verbatim in ANALYSIS.md;
  it states per-solver whether the digest is expected stable across *machines*
  or only across runs on this one. A "Reproducing" section that tells a reader
  to `cmp` a receipt is a defect.
- Condition violations (PLA above 55 °C, out-of-envelope loads) are stated as
  condition limits, never converted into design margins.

## 4. FRICTION PROTOCOL — engine/tool/doc issues

When you hit an engine bug, tool crash, doc drift, or surprising refusal:

- **Append** a structured entry to
  `"/Users/himanshu/Work/New-LMCAD/cad engine/campaign/friction/<part>.md"`
  (create the file on first entry; never delete prior entries):

```markdown
## F<N> — <one-line title> (<date>)
- symptom: what happened, verbatim error/receipt line
- minimal repro: smallest program.json / job.json + exact command line
- expected vs actual: what the docs/digests promised vs what the binary did
- workaround used: how the campaign proceeded (or "blocked")
```

- **You MUST NOT edit engine source or tools source.** Not `crates/`, not
  `tools/`, not the docs they gate. Workarounds live in YOUR campaign
  directory; fixes are the maintainer's job, informed by your friction file.
- Doc-vs-binary contradictions count as friction (cite doc section and the
  verified behavior). Silent-ignore near-misses that cost you time count too.

## 5. Final self-check before declaring the campaign done

Run through in order; any "no" means not done:

1. Fresh-shell rebuild: every command in README "Reproducing" runs from the
   repo root and exits per its contract; rebuilt STLs byte-identical to
   `parts/` (`cmp`).
2. All programs exit 0 and every gate in §2 is present, asserted, and its
   receipt is in `receipts/`.
3. Negative controls executed THIS run: failure attitude interferes
   (number quoted), release/legal path clears (number quoted), and at least
   one oracle-NC proves a gate can fail.
4. ANALYSIS.md regenerated from this run's receipts — zero hand-typed
   measurement numbers; DESIGN.md dimension-freeze table has a source per row.
5. Every prose number traced to a receipt file; provenance/route/tier tags
   quoted; error bands stated.
6. Fit stack receipt exists for every mating interface, at ±0.15 mm extremes.
7. Physics: at least one analysis + `production_check.py` verdict on its
   stress result; sustained loads gated on the creep table; refused analyses
   recorded.
8. Optimization receipt exists and the selected optimum passed the full gate
   suite afterward.
9. Print pack: bed fit (256), wall gate, support report with declared build
   orientation, `route`/`watertight` receipts green (or noted).
10. "What has NOT been done" section present and honest; publish copy (if
    any) written FROM receipts with both control numbers included.
11. Friction entries appended for every issue hit; engine/tools source
    untouched (`git status` on `crates/` and `tools/` is clean).
12. Directory matches §1 layout; no orphan scratch files; programs contain
    the embedded design record.
