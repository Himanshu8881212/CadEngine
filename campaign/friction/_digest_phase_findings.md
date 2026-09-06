# Friction found during the digest phase (pre-campaign), 2026-08-06

Engine/tool/doc issues surfaced by the 5 digest readers while verifying the docs
against the binary. Logged by the orchestrator for the fix phase. All are
GENERAL issues — none block campaigns (workarounds are in OPERATOR_BRIEF.md).

## F1 — DESIGN_GUIDE Parts I–II stale vs binary (doc drift)
- severity: minor
- surface: campaign/DESIGN_GUIDE.md
- status: fixed — F1 FIXED (docs), fix log 2026-08-06
- symptom: guide §5.4/§9.2/§10.4 claim `loft`, `sweep`, `mirror`, `rotate_x/y`,
  `linear_pattern`, `polar_pattern`, `bounding_box`, `measure_dimension` do not
  exist on the JSON surface; all verified present and working (160 ops via
  `describe`).
- expected vs actual: guide says missing; binary executes them.
- note: audit_docs.py gates CI yet did not catch this — the audit's coverage of
  "op X does not exist" prose claims is a gap.

## F2 — `unknown_op` error string quotes stale op count
- severity: papercut
- surface: campaign/DESIGN_GUIDE.md
- status: fixed — F2 FIXED (docs), §21.3/§23 now match the live binary message
- symptom: error text says "116 supported ops"; `describe` reports 160.
- fix shape: derive the count in the error message from the op table.

## F3 — DESIGN_GUIDE §18 stale on assembly runner (doc drift)
- severity: minor
- surface: campaign/DESIGN_GUIDE.md
- status: fixed — F3 FIXED (docs), §18.2 gains the mates DOF and export:assembly_step rows
- symptom: mates receipt now carries a DOF block (`rank`, `free_dof`,
  `verdict`) not in §18.1 body; runner auto-exports AP214 STEP
  (`export:assembly_step`) missing from the §18.2 table.

## F4 — analyzer registry vs solver cards tension
- severity: minor
- surface: tools/analyzer_registry.py
- status: fixed — OPERATOR_BRIEF §5 now quotes the registry tiers (ace_thermal / ace_contact **Demonstrated**, ace_fatigue **Cataloged**, from tools/analyzer_registry.py) instead of "NOT registered"; the registry is the single authority
  (Demonstrated / Cataloged / Demonstrated), and as of 2026-09-04 every one of
  the 18 registered analyzers has a card in `tools/solvers/` whose Status line
  states its registry tier verbatim, so "green suite" can no longer be read as
  a tier. `audit_docs.py` is still unregistered — deliberately: it audits docs,
  it does not analyze a part.
- symptom: ace_contact / ace_fatigue / ace_thermal / audit_docs are
  analyzer-shaped with green benchmark suites but NOT registered in
  `tools/analyzer_registry.py` (its own warning). Solver cards read "green"
  while the trust ledger has no Validated row — easy for an operator to
  over-claim tier.
- fix shape: either register them at their honest tier or make the cards state
  the tier explicitly.

## F5 — `bom_audit.py` is project-hardcoded, not generic
- severity: minor
- surface: tools/publish/bom_audit.py
- status: fixed — bom_audit.py reads the part-name rows from the job (`_NAME_ROW`) and shares the doc_cli `--out`/receipt contract; nothing project-specific is hard-coded
- symptom: hardcoded cyclo26/harmonic26/planetary26 STEP trees + fixed hardware
  table; unusable for any new campaign without editing source.
- fix shape: job-file-driven generic tool (parts list + hardware table as
  input), keeping the old project file as an example job.

## F6 — silent acceptance of unknown op-level params (THE trap)
- severity: major
- surface: kernel-api cli
- status: fixed — F6 FIXED 2026-08-06 (per-op warnings), made fatal invalid_param by the 2026-08-14 ruling
- symptom: misspelled/unknown optional params are ignored; default stays in
  force; exit 0. Verified with `"bogus_param": 42`. Every digest flagged it as
  the biggest operator hazard; workaround is volume-window tripwires.
- expected vs actual: expected a loud `invalid_param` (matches the engine's own
  refuse-don't-degrade doctrine); actual silent default.
- fix shape (general): report-level `warnings: ["unknown param 'bogus_param'
  on op 'box' (id p)"]` at minimum, or an opt-in strict mode
  (`--strict-params` / program-level flag) that fails the op. Must not break
  the documented embedded-design-record idiom (top-level `part`/`notes`/
  `receipts` keys and doc-comment keys inside ops, e.g. `"_comment"`).

## F7 — path resolution inconsistency across surfaces
- severity: minor
- surface: kernel-api cli
- status: fixed — one path rule on every surface: OUT ops resolve against `--out-dir`, IN ops against the program directory first then `--out-dir` (a miss names both roots); verified with the export→import repros (`step_rt`, `import_mesh_path`)
- symptom (verified): `load_part.file` resolves relative to the program JSON's
  dir; `library_*` `dir` resolves relative to `--out-dir`; `.lmcasm` sources
  resolve against each asm file's own dir.
- fix shape: document as contract or unify; at minimum echo the resolved
  absolute path in each receipt so operators can see where it landed.

## F8 — assembly instance exports exit 0 when leaky (gate asymmetry)
- severity: major
- surface: export_stl
- status: fixed — per-instance assembly exports take the strict manufacturing path (a leaky instance FAILS the op); the merged file is labelled a diagnostic SCENE and carries its quality counters; `kernel-api asm` instance rows report `route`/`watertight`
- symptom: part-program `export_stl` fails the run if heal fails; assembly
  instance exports return exit 0 with `watertight:false` in the receipt.
- fix shape: per-run flag or per-op param to promote leaky instance exports to
  failures; default behavior unchanged if that asymmetry is intentional.

## F9 — doc tools crash on `--help`
- severity: papercut
- surface: tools/publish/production_dossier.py
- status: fixed — the document tools share `tools/_receipt.py::doc_cli`; `--help` prints usage and exits 0
- symptom: `production_dossier.py` / `render_sheet.py` / `assembly_doc.py`
  treat `--help` as a job file path and crash with a stack trace.
- fix shape: argparse-standard `--help` printing the docstring job schema.

## F10 — `support_report` threshold knife-edge at modelled angles
- severity: minor
- surface: support_report
- status: fixed — `support_report` reports `near_threshold_area` + `threshold_margin_deg` (1°), with `near_threshold_witness` and a note when > 0; verified: a 45° teardrop roof reads 40.0 mm² near-threshold at `overhang_deg` 45 and 0 at 48
- symptom: f32 comparison at `overhang_deg` exactly equal to a modelled face
  angle flickers (45° teardrop roof at default 45).
- fix shape: documented epsilon band on the threshold + a receipt note when
  any face sits within ~1° of the threshold ("unresolved at this threshold").

## F11 — no JSON-surface single-body/connectivity gate
- severity: major
- surface: mesh_components
- status: fixed — F11 FIXED 2026-08-06, mesh_components op plus assert {components: N}
- symptom: the hardest silent failure (part severed into floating lumps) passes
  validate/watertight/volume and `shells==1`; the only in-tree check
  (`Mesh::is_one_body` / union-find component count) is Rust-only. JSON
  campaigns have no first-class oracle; FRICTION #24 documents shipped near-miss.
- fix shape: a `mesh_components` measure op (component count + per-component
  volume) and an `assert` key (`components == N`) on the JSON surface.

## F12 — README creep-margin discrepancy uncaught by audit
- severity: minor
- surface: tools/audit_docs.py
- status: fixed — `tools/audit_docs.py` gained the `receipt` class and `--also DIR`: a prose line ending in `<!-- receipt: receipts/x.json dotted.key [tol=N%] -->` is checked against the receipt's number (any number on the line within tol passes; a missing file, non-numeric key or drifted value is an ERROR). Verified on a fixture: 124 quoted vs 138.2 in the receipt → error; 3.4 vs 3.41 at 5 % → pass; a missing receipt → error. DELIVERABLE_SPEC §3 states the anchor rule and §5 the self-check line
- symptom: card_magazine README quotes 124× in one place, generated ANALYSIS
  computes ~138× (gated ≥50×). Doc-audit does not diff prose numbers against
  generated receipts.

## F13 — kernel-api CLI usage error messaging (minor)
- severity: minor
- surface: kernel-api cli
- status: fixed — the CLI usage text names both subcommands and every flag, and a bad flag is reported with the usage instead of a bare error
- symptom: bare `kernel-api prog.json` exits 1/2 without pointing at the `run`
  subcommand; three independent readers tripped on it.
- fix shape: top-level usage error suggesting `run`/`asm` subcommands.

---

## Fix log (orchestrator, 2026-08-06 — pre-campaign)

- **F6 FIXED (general)**: unknown op-level params now emit per-op report
  `warnings` (non-fatal; `_`-prefix exempt as the comment convention). Impl:
  `run_one` diffs raw keys against the generated `OP_PARAMS` table.
  Tests: `tests/measure.rs::unknown_params_warn_without_failing_...`. Shipped
  showcase programs re-run: 0 warnings, byte-identical STL rebuild confirmed.
- **F11 FIXED (general)**: new `mesh_components` measure op + `assert
  {"components": N}` key expose `Mesh::component_count` on the JSON surface.
  Op count 160 → 161; discover.rs regenerated via `tools/gen_discover.py`;
  API.md/README/DESIGN_GUIDE Appendix A updated; `tools/audit_docs.py` exit 0.
  Tests: `tests/measure.rs::mesh_components_is_the_single_body_oracle_...`.
- **F2 partially moot**: the binary already derives the count (`OP_COUNT`);
  the stale "116" lives only in DESIGN_GUIDE §23 prose (still open, fix phase).
  The one hardcoded test pin ("160 supported ops") now derives from `OP_COUNT`.
- Remaining findings (F1, F3–F5, F7–F10, F12, F13) stay open for the
  post-campaign fix phase.
- **F1 FIXED (docs)**: DESIGN_GUIDE §5/§5.4/§9.2/§10.4 no longer claim
  loft/sweep/patterns/mirror/bounding_box/measure_dimension are absent from the
  op surface; each section now routes one-shot op forms to API.md and keeps the
  parametric-Document doctrine. audit_docs exit 0.
- **F2 FIXED (docs)**: §21.3 "names exactly 116" and the §23 stale error quote
  now match the live binary message (161, describe pointer).
- **F3 FIXED (docs)**: §18.2 table gains the mates DOF block row and the
  `export:assembly_step` row — values quoted from a real re-run of the §18.1
  tour file (residual 1.34e-12 matches the originally documented run).

---

## Fix log (orchestrator, 2026-08-14 — engine fix round 2)

### PROTOCOL BREACH found and adjudicated
An UNLOGGED engine change landed 2026-08-10 11:20-11:22 (interp.rs, program.rs,
tests/measure.rs edited; both binaries rebuilt 12:05-12:09) — during the
assembly campaigns, outside any sanctioned fix phase, with no fixlog entry, no
doc update, no compat note. What it did: (a) unknown op params became FATAL
(`invalid_param`) instead of the documented non-fatal warnings, and the
regression test pinning the documented contract was silently rewritten to match;
(b) a `manufacturing_ready` refusal was added to every mesh export, which broke
`asm_ops` end-to-end and every negative-control fail-pose scene export (SLAS F9:
programs unchanged since 2026-08-08 now exit 1 at the export op).

Maintainer ruling, on merits:
- **KEEP fail-closed unknown params** — refuse-never-degrade is this engine's
  doctrine and the original digest F6 expectation; zero in-tree programs relied
  on the leniency (111-program regression, 0 warnings). Docs now updated to
  match (API.md report table, OPERATOR_BRIEF §1.10). The breach was the
  PROCESS, not the direction.
- **KEEP the manufacturing guard for print files; EXEMPT diagnostic scenes.**
  New `write_mesh_scene` path: the merged `asm_export` file is a posed SCENE
  (an NC failure attitude interpenetrates BY DESIGN) — it now exports with
  `scene: true`, `cross_instance_self_intersections` ON THE RECORD, while
  per-instance part files stay strict. Tests: asm_ops::merged_scene_export_...
  (pins report-not-refuse) + the restored full_loop test.

### T6(b)/(c)/T15 root FIXED — the sealed-hole tessellation family
`tessellate_adaptive.rs` dropped `Face::inner` entirely (face_boundary walked
the outer loop only). Every export/measurement mesh of a part with a holed
planar face shipped with the hole sealed as skin: unclosed measurement meshes
(mesh_components refusals), `voxel_healed` demotions on trivially exact parts,
support_report counting hole area as face area. Fix: `loop_boundary` refactor +
holes routed through the dense shared-seam sampling into the (pre-existing,
bridged) `tessellate_planar_with_holes`. Tests:
`crates/kernel-brep/tests/adaptive_holes.rs` — closed/one-body/orientable,
volume 3640 exact vs sealed 4000, area 2296 exact, and the boolean annular-cap
(T15 family) clean through the adaptive path.
NOTE: this legitimately CHANGES STL bytes for affected shipped parts —
scheduled for the deliberate re-baseline after the assembly campaigns complete.

### T5 verified already fixed (degenerate-triangle distance-0), owner A, with
tests; no action.

---

## Fix log (orchestrator, 2026-08-23 — engine fix round 3, dsh-era)

Aug-10 stealth-guard casualties found by running the FULL suite (the guard
measures honestly what used to ship invisibly; each fix keeps the honesty and
restores function):
- **hybrid_boolean TPMS fusion**: route-aware write policy (Strict/Healed/Scene)
  — healed-route files refuse only on true breakage; crossings/pinch vertices
  REPORT (require-gateable). Voxel-domain writers (tpms, import_mesh,
  mesh_carve, export_threaded, hybrid healed) moved to the Healed policy;
  `manufacturing_ready` in receipts is now the computed truth, not a constant.
- **fail-closed vs serde aliases**: the unknown-param check refused documented
  `bore_d` aliases. gen_discover now emits `aliases` into ParamSpec; the check
  and `describe` both honour them.
- **min_ligament message pin**: needle updated to the centralized validator's
  wording (invariant core pinned).
- **wave3 gear route pin**: RETIRED the route=exact expectation — root-caused
  the bored-gear cap↔wall seam-phase frontier (cap ring and curved wall top
  ring sample the rim at different phases; the ear clip's endgame wedge grazes
  the wall: ONE raw crossing pair, measured at (r=6.000, z=10)). The honest
  route is voxel_healed until the seam is phase-locked; test now pins that,
  with the goal documented.

Ear-clipper hardening landed while root-causing (all general, defense in
depth): the documented-but-missing collinear-drain on stall now exists; the
stall fan is gated on convexity (concave remainders take a centroid fan);
ear validity now also rejects ears properly crossed by non-incident ring
edges (long boolean seam edges could slice through vertex-clean ears);
`tessellate_planar` un-bakes keyhole-bridged rings into the verified
hole-aware path; `tessellate_planar_with_holes` gained a ranked-anchor retry
ladder + 2D hole-coverage verification + a star-shaped annulus strip fallback.

## RESOLUTIONS (2026-09-05 fix round)

Engine (`crates/`) and `tools/` fixes made at the maintainer's request (2026-09-05); every `fixed` above names its receipt (a repro in the fix-round scratch set, a re-run of this campaign's own program, or a unit test). Entries above are unchanged except their `status` line.

- **F4** — F4: the brief and the registry no longer disagree — §5's tier column is the registry's word for word.
- **F5** — F5: `bom_audit.py` is generic (job-driven name rows, doc_cli receipt persistence).
- **F7** — F7: path resolution is one documented rule (ops_core.md §path table) and the round-trip repros pass.
- **F8** — F8: instance exports can no longer exit 0 on a leaky body (asmops export policy Strict per instance, Scene only for the merged diagnostic).
- **F9** — F9: `--help` no longer crashes any doc tool (doc_cli).
- **F10** — F10: the knife-edge is on the receipt — `near_threshold_area` names the area the threshold cannot resolve, so "support-free at 45°" is quoted with its ±1° check (kernel-core `SupportFreeReport::near_threshold_area`, measure.rs support_report).
- **F12** — F12: stays open on the audit side (prose semantics are the campaign's); the checker now gives the exact number to quote.
- **F13** — F13: usage messaging (kernel-api main.rs USAGE).

### Second pass (2026-09-05, the 13 items left open or partial)

- **F12** — F12: prose numbers can be anchored to receipts and audited (`audit_docs.py --also <campaign> --only receipt`).
