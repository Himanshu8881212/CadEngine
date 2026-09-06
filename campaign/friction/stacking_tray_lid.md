# friction — stacking_tray_lid (2026-09-05)

Drop-in lid for a MakerWorld stacking tray, seated on the tray's own 45 degree
rim funnel. Campaign: `Workspace/storage_system/stacking_tray_lid/`.

Five of the six items below are one family: **the exact STL tessellator leaves
the exact route on geometry every topological gate calls clean.** Each has a
cheap, reproducible workaround, and all five workarounds are now baked into
`programs/gen_programs.py` with the reason written beside them.

## F1 — four chained `difference` cuts demote the STL export; folding them with `union_all` does not (2026-09-05)
- severity: major
- surface: kernel tessellation
- status: fixed — four chained `difference` cuts export exact (`lid` re-run 2026-09-05: x_stl/x_3mf/x_scene exact)
- recurrence of: campaign/friction/folding_deck_cleat.md#F2 (two `countersink_hole` cuts in one plate make the exact tessellation leak)
- symptom: the lid is cut by four identical, mutually disjoint notch cutters.
  Removing them one after another (`difference` x4) produced
  `"route": "voxel_healed"`, `demotion.reason: "degenerate_triangles"`,
  `degenerate_triangles: 1`, `exact_triangles: 1778` -> a **543 024-triangle,
  27 MB** healed STL. The witness sat exactly on a cutter/face crossing:
  `[5.187485218048096, 48.29999923706055, 20.93333371480306]`.
  `validate` reported valid / closed / manifold / genus 0 / shells 1 throughout,
  and `assert components: 1` passed. Only the export receipt knew.
- minimal repro: build the lid with `programs/gen_programs.py`, then take the
  four `cut{i}` bodies out with four chained `difference` ops instead of one
  `union_all` + one `difference`:
  `"$K" run lid.json --out-dir .` -> `x_stl` route `voxel_healed`.
- expected vs actual: cutting N disjoint bodies one at a time and cutting their
  union once are the same set operation; only the second tessellates exactly.
  Measured side by side, same cutters, same part: chained -> `voxel_healed`
  543 024 tris; folded -> `exact` 2 158 tris.
- workaround used: `union_all` the four cutters, one `difference`. In
  `lid_ops()`.

## F2 — `rotate_x 180` and `mirror` both demote the export of a solid that exports exactly in place (2026-09-05)
- severity: major
- surface: kernel tessellation
- status: fixed — `rotate_x 180` and `mirror` of an exact-exporting solid stay exact (`rot_mirror` repro)
- recurrence of: campaign/friction/folding_book_stand.md#F4 (`export_stl` of POSED solids hangs) — same family, different symptom: this one demotes rather than hangs
- symptom: the lid must ship top-face-down. Posing it with
  `{"op":"rotate_x","degrees":180}` + `translate` gave `route: "voxel_healed"`,
  `reason: "non_orientable_edges"` at scale 1.7 while the SAME solid exported
  `exact` in its own frame at the same scale. Replacing the rotation with
  `{"op":"mirror","plane":{"point":[0,0,z],"normal":[0,0,1]}}` demoted at scale
  1.0 as well. `rotate_x 180` multiplies every coordinate by
  `sin(180 deg) = 1.2246e-16`, so the pose perturbs the part at ~1e-14 and
  turns exactly-coplanar faces into near-coplanar ones.
- minimal repro:
  ```
  lid_ops(1.7) + [{"id":"p","op":"rotate_x","in":"lid","degrees":180},
                  {"id":"t","op":"translate","in":"p","offset":[0,0,40.63]},
                  {"id":"x","op":"export_stl","in":"t","file":"z.stl"}]
  ```
  -> `voxel_healed`, 637 328 triangles. Export `lid` directly -> `exact`, 1 184.
- expected vs actual: a rigid 180 degree rotation about a world axis is exact in
  f64 for the three coordinates that matter; the op should special-case
  multiples of 90 degrees rather than going through `cos`/`sin`. `mirror` is
  documented as "orientation-safe reflection" and was not safe here either.
- workaround used: `lid_ops(flip=True)` authors the print body directly in print
  coordinates. No transform op is used anywhere in the shipped programs.

## F3 — a corner annulus between two near-parallel arcs mis-winds; making the arcs concentric fixes it (2026-09-05)
- severity: major
- surface: kernel tessellation
- status: fixed — the corner annulus between near-parallel arcs winds correctly (CDT planar tessellation)
- symptom: the lid's neck steps 1.20 mm inboard of the taper's top, leaving a
  narrow annular face. Built with both rounded-rect profiles at corner radius
  3.00 mm — two arcs of equal radius whose centres are 1.70 mm apart along the
  diagonal — the export demoted with `reason: "non_orientable_edges"`, witness
  on the corner arc at the step plane
  (`[-46.783742904663086, 47.84733772277832, 19.5]`).
  It was **scale- and frame-dependent**, which is the tell that it sits on a
  predicate boundary: measured `exact` at s=1.7 unflipped and s=1.0 flipped,
  `voxel_healed` at s=1.0 unflipped and s=1.7 flipped, same program.
- minimal repro: set `DIMS["neck_r"] = 3.0` in `programs/gen_programs.py` and
  export `lid` at s=1.0 and s=1.7 in both frames.
- expected vs actual: the four combinations should agree; they did not.
- workaround used: `neck_r = 1.80`, chosen so the neck's corner centre
  (48.30 - 1.80 = 46.50) is the SAME point as the taper top's corner centre —
  the two arcs become concentric instead of near-parallel. All four
  combinations then export `exact`, and so do s=0.5 and s=2.5.

## F4 — two stacked solids whose shared plane differs by 1.8e-15 make `union_all` refuse (2026-09-05)
- severity: minor
- surface: union_all
- status: fixed — boolean-entry snap rounding (1e-12 grid) makes the 1.8e-15 plane difference bit-equal; `stack_ulp` repro unions
- recurrence of: campaign/friction/folding_book_stand.md#F1 (union with a coincident face fails)
- symptom: the neck sits between the taper and the plate. Its `extrude` height
  came out as `1.3999999999999986` (from `20.9 - 19.5` in f64) and its translate
  offset as `3.0`, so its top face landed at `4.399999999999999` while the
  taper's bottom section was at `4.4`. `union_all` refused:
  `union_all failed validate(): closed=false manifold=false genus=5
  euler_characteristic=-7 shells=2 — refusing to bind an invalid solid`.
- minimal repro: stack two prisms with `(offset, height)` pairs derived by
  subtraction such that `offset + height` differs from the neighbour's plane by
  one ulp.
- expected vs actual: the engine's own guidance is "embed >= 0.1 mm or coincide
  exactly — never the sliver between". A campaign cannot *reach* exact
  coincidence through an `(offset, height)` API when the two numbers come from
  different subtractions, so the only reachable state is the forbidden one.
  A weld tolerance on the union, or a documented "give every stacked solid an
  embedment" rule, would close it.
- workaround used: `neck_pad = 0.20` — the neck is embedded 0.20 mm into both
  neighbours, whose profiles strictly contain it there, so the union is
  geometrically unchanged.

## F5 — `loft` with sections ordered descending in z binds an INSIDE-OUT solid that passes every topological gate (2026-09-05)
- severity: major
- surface: loft
- status: fixed — a loft whose sections descend in z is re-skinned outward (`loft_desc` repro: positive `exact_volume`)
- symptom: the same two sections lofted in the two possible orders both bind.
  Ascending gives `exact_volume` **+11468.247848723888**; descending gives
  **-11468.247848723891**. Both report `valid: true, closed: true,
  manifold: true, shells: 1, genus: 0`.
- minimal repro:
  ```json
  {"ops":[{"id":"L","op":"loft","sections":[<rounded rect at z=5.6>,
                                            <rounded rect at z=4.4>]},
          {"id":"v","op":"validate","in":"L"},
          {"id":"vol","op":"exact_volume","in":"L"}]}
  ```
- expected vs actual: `ops_core.md` section 4 says sections must be "ordered along the
  loft", and the op does not check it. A negative volume is an inverted
  orientation — the op should refuse, or at minimum the sign should fail
  `validate`. Nothing in the section 2 gate suite catches it except an
  `exact_volume_within` window against a positive target, which is why that
  gate is worth keeping even on geometry with no cones in it.
- workaround used: the section order is asserted by construction in
  `lid_ops()`, and the closed-form volume window is the tripwire.

## F6 — `production_dossier.py` refuses the `--out` flag every other runner takes (2026-09-05)
- severity: papercut
- surface: tools/publish/production_dossier.py
- status: fixed — production_dossier takes `--out`
- symptom: `python3 tools/publish/production_dossier.py job.json --out r.json`
  -> `{"ok": false, "error": "usage: production_dossier.py job.json"}`, exit 1.
- minimal repro: as above.
- expected vs actual: `OPERATOR_BRIEF.md` section 3.1 says to use `--out PATH` on every
  runner and explicitly forbids the stdout-redirect idiom because a redirect
  truncates its target at launch. The tool's own docstring says
  `Usage: python3 production_dossier.py job.json` and `sys.argv` is checked with
  `len(sys.argv) != 2`, so the brief's rule cannot be followed for this one tool
  and the cookbook entry that shows the shared contract is misleading here.
- workaround used: `run_all.sh` redirects stdout for this tool only, with the
  exception written on the line above it.

## F7 — "every runner receipt now carries a `determinism` block" is true only of the ACE physics runners (2026-09-05)
- severity: minor
- surface: campaign/DELIVERABLE_SPEC.md
- status: fixed — every runner receipt carries the `determinism` block (ensure_determinism_block on the shared receipt path)
- symptom: `DELIVERABLE_SPEC.md` section 3 says *"Every runner receipt now carries a
  `determinism` block (`tools/_receipt.py`, schema `lmcad.determinism.v1`)"* and
  makes `determinism.core_digest` the mandatory comparison for **"any tool
  receipt (checkers AND solvers)"**, with receipt bytes declared "not comparable,
  ever". Of the six tool receipts this campaign ships, exactly one has the
  block:

  | receipt | tool | registry tier | `determinism` present |
  |---|---|---|---|
  | `fea_pry.json` | `ace_fea_runner.py` | Validated | **yes** |
  | `opt_mass.json` | `param_optimize.py` | Validated | no |
  | `tol_*.json` | `tolerance_stack.py` | Validated | no |
  | `prodcheck_pry.json` | `production_check.py` | Validated | no |
  | `dossier.json` | `production_dossier.py` | Validated | no |
  | `sheet_*.json` | `render_sheet.py` | — | no |

- minimal repro:
  `python3 tools/analyzers/tolerance_stack.py job.json --out r.json`, then
  `python3 -c "import json;print('determinism' in json.load(open('r.json')))"`
  -> `False`. Same for `param_optimize.py`, `production_check.py`,
  `production_dossier.py`, `render_sheet.py`.
- expected vs actual: `grep -l determinism tools/**/*.py` looks like wide
  adoption, but in `param_optimize.py` and `render_sheet.py` the word only
  appears in a docstring — `determinism_block()` (`tools/_receipt.py:488`) is
  called by the ACE runners and `derived_model.py` only. A campaign following
  section 3 to the letter has **no sanctioned way to compare five of its six tool
  receipts between runs**: bytes are forbidden and the digest does not exist.
- workaround used: this campaign quotes `core_digest` for the one receipt that
  has it and says plainly, in README "Reproducing", which artefacts are held to
  `cmp` and which are not comparable at all.

## RESOLUTIONS (2026-09-05 fix round)

Engine (`crates/`) and `tools/` fixes made at the maintainer's request (2026-09-05); every `fixed` above names its receipt (a repro in the fix-round scratch set, a re-run of this campaign's own program, or a unit test). Entries above are unchanged except their `status` line.

- **F1** — F1: exact route.
- **F2** — F2: poses stay exact.
- **F3** — F3: annulus winding.
- **F4** — F4: ulp-stacked solids union.
- **F5** — F5: inside-out loft.
- **F6** — F6: --out.
- **F7** — F7: claim now true.
