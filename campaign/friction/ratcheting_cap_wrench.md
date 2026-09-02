# Friction log — ratcheting_cap_wrench

## F1 — OPERATOR_BRIEF §5 says ace_contact/ace_fatigue/ace_thermal are "NOT registered"; the registry registers all three (2026-08-08)

- symptom: `campaign/OPERATOR_BRIEF.md` §2 ("Provenance discipline") and the §5 solver table both state:
  *"thermal/contact/fatigue are in-house gated but NOT registered — say so."*
  The live registry disagrees. Verbatim from `python3 tools/analyzer_registry.py`:

  ```
  ace_thermal         Demonstrated   no   no  yes   ok
  ace_contact         Demonstrated   no   no  yes   ok
  ace_fatigue         Cataloged      no   no  yes   ok
  ```

  They are registered, and they carry tiers (Demonstrated / Demonstrated / Cataloged).
  What is true is the weaker statement that they are **below the Validated line**
  (the registry's own footer: "% BELOW the validated line (not yet validated): 66.7%").
- minimal repro: `cd "/Users/himanshu/Work/New-LMCAD/cad engine" && python3 tools/analyzer_registry.py`
- expected vs actual: OPERATOR_BRIEF §5 promised "NOT registered" (i.e. absent from the
  ledger); the binary/tool lists all three with an explicit tier. DELIVERABLE_SPEC §3
  says *"Only registry-Validated surfaces may be called validated; check
  `python3 tools/analyzer_registry.py`, not the solver's own README"* — which makes the
  registry the authority and the brief the stale document.
- workaround used: this campaign quotes the REGISTRY tier verbatim for every solver
  (`ace_contact` = **Demonstrated**, `ace_fatigue` = **Cataloged**,
  `tolerance_stack` = **Cataloged**, `production_check` = **Cataloged**,
  `param_optimize`/`ace_fea` = **Validated**) and never uses the phrase "not registered".
  The brief's sentence is recorded here as the conflict it is rather than repeated.

## F2 — `iso286_fit` has no free-running class wide enough for a printed sliding rail (2026-08-08)

- symptom: the catalog digest (`digests/implicit_recipes.md` §10) lists the supported
  classes as `H7/g6, H7/h6, H7/k6, H7/n6, H7/p6, H7/s6, H8/f7`. The loosest of these,
  `H8/f7`, returns for a Ø110 nominal:
  `{"clearance": [0.036, 0.125], "hole": [0.0, 0.054], "shaft": [-0.071, -0.036]}`
  — a **0.036–0.125 mm** diametral clearance band. The house FDM tolerance is ±0.15 mm,
  i.e. roughly **five times the entire ISO band**, so the ISO number cannot be used as a
  fit; it can only be quoted as the machined-surface reference it is.
  There is no `H9/d9` (the free-running class the concept card named) and no `H11/c11`.
- minimal repro:
  ```sh
  "…/target/release/kernel-api" run assistive_system/ratcheting_cap_wrench/programs/lookups.json \
      --out-dir assistive_system/ratcheting_cap_wrench/receipts/lookups_out
  ```
  → op `fit_journal_110` measures as above. Requesting `"fit":"H9/d9"` is refused as
  out-of-table.
- expected vs actual: the concept card planned to source the journal and the dovetail rail
  from `iso286_fit` at `H9/d9`. Expected a free-running band comparable to FDM reality;
  actual is a precision-machining band two orders of magnitude tighter than the printer.
- workaround used: `H8/f7` is recorded as the **machined-surface reference only**, tagged
  as such in the DESIGN.md dimension-freeze table, and the *governing* clearance for every
  printed-to-printed running fit is the house FDM minimum (>= 0.3 mm radial nominal),
  proven by `tolerance_stack.py` at ±0.15 mm rather than by the ISO band. No ISO fit class
  is claimed for a printed running surface anywhere in this campaign.

## F3 — difference of coaxial ANALYTIC cylinders exports voxel_healed, never exact (2026-08-14)
- symptom: `export_stl` receipt `route: "voxel_healed", watertight: true, triangles: 345712` for a plain Ø110/Ø70 tube; the exact tessellation path is abandoned silently (exit stays 0). With `segments: 180` on both cylinders the same boolean exports `route: "exact", triangles: 2310`.
- minimal repro: {"ops":[{"id":"a","op":"cylinder","base":[0,0,0],"axis":[0,0,1],"radius":55,"height":6},{"id":"b","op":"cylinder","base":[0,0,-0.5],"axis":[0,0,1],"radius":35,"height":7},{"id":"t","op":"difference","a":"a","b":"b"},{"id":"x","op":"export_stl","in":"t","file":"tube.stl"}]} — `kernel-api run tube.json --out-dir out/`; op x measures `route:"voxel_healed"`.
- expected vs actual: OPERATOR_BRIEF §4 calls exact B-rep the default surface for dimensional work and DELIVERABLE_SPEC §2.4 wants `route: exact` preferred; a coaxial cylinder difference is the most elementary annulus and demotes every downstream export of the part (ring/frame/plate all carry coaxial bores). Also makes exports ~150x heavier.
- workaround used: every cylinder in this campaign's geometry is emitted with an explicit `segments` count (180 for r>=20, 96 mid, 48 small), i.e. polygonal prisms; closed-form volume targets use the exact n-gon area 0.5*n*r^2*sin(2pi/n). Verified `route: "exact"` after the change. Side effect (documented in DESIGN): bores are inscribed 180-gons, sagitta 0.008 mm at r55.35 — negligible against the ±0.15 mm print band.

## F4 — seg-180 coaxial difference demotes export route near r 66 regardless of phase; seg 360 rescues (2026-08-14)
- symptom: `export_stl` receipt `route: "voxel_healed", triangles: 316844` for a plain segmented tube difference at plate radii. F3's fix (explicit `segments`) stops working at larger radii.
- minimal repro (all cylinders explicit segments, coaxial difference, export):
  r66/53.5 h5 seg180 -> voxel_healed; seg96 -> voxel_healed; cutter phase-rotated 1.0 deg -> voxel_healed; **seg360 -> exact (4806 tris)**; r64/53.5 seg180 -> exact; r60/50 seg180 -> exact; r55/35 seg180 (F3's case) -> exact. Also height-dependent noise: r66/53.5 h6 seg180 voxel_healed.
- expected vs actual: F3 workaround documented "segments => exact"; actual: the exact route dies again somewhere in r 64..66 at 180 segments, independent of phase/height.
- workaround used: `geom_lib.seg_for()` gains a 360-segment tier for r >= 56. Plate (r66) and frame head (r62) now export `route: "exact"` (6574 / 17466 tris). A RELATED numerology: pawl prism z1 = 9.4 or 9.3 demotes its export while 9.35/9.45 stay exact (same program otherwise) — worked around by choosing 9.35, recorded in geom_lib.
- note: kernel-side; no crates/tools touched.

## F5 — paired small-cylinder unions across a segmented head corrupt the exact tessellation; heal can fail or not terminate in reasonable time (2026-08-14)
- symptom: frame head (r62, seg 360) + Ø11.2 boss cylinders (seg 96) at az 60/120/240/300: export refuses `mesh is not manufacturing-ready even after the voxel heal (voxel 0.3 mm): ... self_intersections=10` (exit 1), or demotes.
- measured matrix (head + bosses only, union_all, export):
  single boss az60 -> exact; az120 -> exact; az240 alone -> exact (28788 tris); az240+az300 (south pair) -> self-intersections, heal FAILS; az60+az120 (north pair) -> voxel_healed 723856 tris; all four -> `serialized stl failed strict round-trip validation: boundary_edges=3`; boss pair phase-rotated 1.875 deg -> still fails; centers nudged 61.5->61.45 -> still fails; bosses as rounded-coordinate 96-gon prisms -> still fails. Circle-circle crossing angle is 82 deg (transversal) — not a tangency problem.
- also (F5b): with bosses replaced by boxes, the POCKET sector cut before the 4 pilot cuts leaves self_intersections=1 (export refused); identical cuts with the pocket LAST -> route exact 17466 tris. Boolean ORDER is load-bearing.
- expected vs actual: OPERATOR_BRIEF §8 boolean hygiene has no rule against paired unions or cut order; validity gates all pass (valid/closed/manifold/shells 1) while the export-side tessellation is corrupt — a green-receipt silent mode until the export gate fires.
- workaround used: (1) boss cylinders replaced by two rectangular boss BARS (box-vs-circle unions are clean), pilots unchanged; (2) frame_ops order frozen: unions first, cylinder cuts, pilot cuts, pocket sector cut LAST; both recorded in DESIGN.md §11 (S2-D6/D7).

## F6 — ace_contact `supports[].dofs` silently accepts booleans as PRESCRIBED 1.0 displacements (2026-08-14)
- symptom: a "clamp" written as `{"node":"root","dofs":{"ux":true,"uy":true,"rz":true}}` produced a curve whose row-1 tip displacement JUMPED 17.9 mm on a 20 mm cantilever at lambda 0.025 with near-zero incremental stiffness — the runner had interpreted `true` as a prescribed displacement of 1.0 mm (ux, uy) and 1.0 RADIAN (rz) ramped with lambda. Exit stayed 0, receipt green.
- minimal repro: 20x5x2 mm PLA cantilever, 1 N tip load, supports dofs booleans -> tip_uy 17.85 mm at lambda 0.25 (closed form 0.24 mm at full load). Same job with `{"ux":0.0,"uy":0.0,"rz":0.0}` -> matches PL^3/3EI to <2%.
- expected vs actual: `digests/tools_cookbook.md` documents the field only as `dofs:{ux?,uy?,rz?}` with no type/semantics; the runner's own gate suite (46/46 green) uses `0.0` floats. Expected: a type refusal for booleans (the engine-side convention is refuse-don't-guess); actual: bool quietly coerced to 1.0 — a silent unit trap of the exact class OPERATOR_BRIEF §1.10 says was closed on the op surface.
- workaround used: all campaign contact jobs write explicit `0.0` prescribed displacements; the mis-clamped receipts were discarded and the calibration case retained in scratch. No tools edited.

## F7 — 46+ short rack teeth across the ring's annular wall: voxel heal FAILS (F3/F4 family boundary) (2026-08-14)
- symptom: after baking rack_pitch 1.0 over span [-16,50] (66 teeth), `export_stl` on the ring REFUSED: `mesh is not manufacturing-ready even after the voxel heal (voxel 0.3 mm): ... self_intersections=1` (exit 1). The same construction at 33 teeth (pitch 2.0) healed to route voxel_healed; at 46 teeth (span [-6,40]) it heals again.
- minimal repro: ring_ops from programs/geom_lib.py with rack_pitch 1.0, rack_x0 -16, rack_x1 50 -> export refusal; rack_x0 -6, rack_x1 40 -> voxel_healed, watertight true, 589284 tris.
- expected vs actual: F3 documented the demote-to-voxel_healed behaviour for features crossing the annular wall; expected the heal to keep absorbing it; actual: enough short prisms crossing the wall push the healer past what it can fix — the failure is count/extent dependent.
- workaround used: rack span derived from the jaw-flank travel band (the only region teeth are functional), 46 teeth; recorded as S3-D2. No tools edited.

## F8 — the three document tools have no `--out`, so the only way to persist their receipt is the idiom OPERATOR_BRIEF §3.1 forbids (2026-08-23)
- symptom: `render_sheet.py`, `assembly_doc.py` and `production_dossier.py` print their receipt as the last stdout line and accept no `--out PATH`. `tolerance_stack.py`, the ACE runners and `joint_check.py` all do accept it. A campaign that wants those receipts on disk must either redirect stdout (`tool.py job.json > receipt.json`) or re-implement the atomic write itself.
- minimal repro: `python3 tools/render_sheet.py <job>.json --out receipts/x.json` -> the flag is treated as a second positional/unknown arg; `grep -n -- '--out' tools/render_sheet.py tools/assembly_doc.py tools/production_dossier.py` returns nothing, while `tools/joint_check.py:8` documents `[--out PATH]`.
- expected vs actual: OPERATOR_BRIEF §3.1 says "never use `tool.py job.json > receipt.json` — the redirect truncates the target at LAUNCH ... Use `--out PATH`, which writes atomically", and presents that as the shared contract of every runner in `tools/`. Actual: three of the tools a campaign is REQUIRED to run (renders + BOM are DELIVERABLE_SPEC §1 deliverables) do not implement the escape hatch the doctrine names.
- consequence observed in this campaign: the one op with the same gap is `kernel-api run` itself, and it bit — a killed S4 assembly re-run left `receipts/assembly_report.json` at ZERO BYTES (DESIGN §13, S4-D3).
- workaround used: `programs/run_gates.py` and `programs/run_tools.py` capture stdout in memory and write the receipt with `mkstemp` + `os.replace` after the child exits, so an interrupted run leaves the previous receipt intact. No tools edited.

## F9 — `production_dossier.py` refuses a part that PASSES the bed-fit gate, because `spacing_mm` is also an edge margin (2026-08-23)
- symptom: `ValueError: part 'frame' footprint 255.0 x 124.0 mm cannot fit the 256.0 x 256.0 bed in any 0/90 rotation with 5.0 mm spacing: REFUSED` (exit 1), for a part whose in-program `bounding_box` gate `{"envelope":[256,256,256],"require":{"fits_within":true}}` passes with measured size [255.0, 124.0, 16.0].
- minimal repro: `programs/doc/dossier_spacing5.json` (job identical to the shipped dossier except `spacing_mm: 5`).
- expected vs actual: the docstring does say `spacing_mm ... gap between parts AND to bed edges`, so the behaviour is documented — but the two gates a campaign is told to run (`bounding_box fits_within` in §2.7 and the dossier in §1) disagree about what "fits the bed" means, and only the second one knows about skirts. This is a doctrine gap rather than a bug: nothing tells a campaign that the envelope gate is not the packing gate.
- workaround used: BOTH runs ship — the refusal as `receipts/dossier_spacing5_REFUSAL.json` and the 0.5 mm-spacing dossier as the deliverable — and the README carries the resulting condition limit (frame = one-part plate, no skirt). Recorded as DESIGN §13 S4-D1. No tools edited.

## F10 — `assembly_doc.py` prints an OVERALL DIMENSION computed from a 1/N subsample, and under-reported this assembly by 48% (2026-08-23)
- symptom: the shipped sheet's ASSEMBLED panel reads `overall 132 × 132 × 26 mm (W × D × H)` for an assembly whose true bounding box is **255 × 132 × 25.8 mm**. The 145 mm lever — the whole point of the tool — is missing from the number, and the fitted view is scaled as if the handle were not there (it is drawn outside the axis limits).
- root cause, read from the source (`tools/assembly_doc.py` ~line 615): the ASSEMBLED panel builds `fit_pts_a` from every triangle, then decimates with `fit_pts_a[::ceil(len/30000)]` for speed, and takes BOTH the view fit AND the printed `ext = ahi - alo` from that decimated cloud.
- minimal repro (this campaign, 14 instances, 1 865 358 vertices, stride 63):
  ```
  full bbox            [255.0, 132.0, 25.8]
  bbox of pts[::63]    [131.96, 131.8, 25.8]
  frame vertices with x < -66 : 18 of 52 482   ->  sampled: 0
  ```
  A slender feature whose extreme is defined by a handful of vertices (a box handle has ~18 vertices past the head) is missed by a uniform stride with probability ≈ (1 − 1/63)^18 ≈ 0.75. The decimation is a rendering optimisation; using it for a REPORTED DIMENSION makes the number stochastic in the mesh's triangle count.
- expected vs actual: the tool's own docstring calls this panel "assembled view with scale bar + overall dimensions", i.e. an engineering callout; DELIVERABLE_SPEC §3 requires every published number to be traceable and correct. Expected: extents from the full cloud (an O(n) min/max, cheap even at 1.9 M points) or no callout at all. Actual: a plausible-looking wrong dimension on the face of the document, with no warning in the receipt (`receipts/asmdoc_receipt.json` reports `parts`, `px`, `bom_rows` — nothing about decimation).
- workaround used: the campaign does not quote the sheet's overall label anywhere. README and ANALYSIS quote the MEASURED envelope from `receipts/part_frame_report.json` (`bounding_box` → size, the gated number) and the assembly instance bbox, and the render note says the sheet's own label is wrong and by how much. No tools edited.

## F11 — `bom_audit.py` is a fourth tool in the F8 family, and its `receipt` path is resolved against the CWD, not the job file (2026-08-23)
- symptom: `bom_audit.py job.json` accepts EXACTLY two argv entries (`if len(argv) != 2: usage`), so there is no `--out PATH`; the only in-tool way to persist the receipt is the job's own `"receipt"` key, and that path is resolved by `tools/_receipt.py` against the PROCESS CWD. Launch the same job from the campaign directory instead of the repo root and the receipt lands at `<campaign>/assistive_system/ratcheting_cap_wrench/receipts/bom_audit.json` — a nested fossil tree, silently, with exit 0.
- minimal repro: `cd assistive_system/ratcheting_cap_wrench && python3 ../../tools/bom_audit.py programs/doc/bomaudit.json` -> stderr says `receipt written: assistive_system/ratcheting_cap_wrench/receipts/bom_audit.json`, relative to the campaign dir. This is the same trap `param_optimize.py` set in S4 (BUILD_LOG 2026-08-23), and the same fossil directory shape: an empty `assistive_system/ratcheting_cap_wrench/programs/_opt_scratch` tree was found INSIDE this campaign during the S5 audit and removed.
- expected vs actual: OPERATOR_BRIEF §3.1 presents `--out PATH` as the shared contract of every runner in `tools/`; `bom_audit.py`'s own docstring documents `receipt` but says nothing about which root it is resolved against. `assemblies[].step` by contrast IS resolved against `base_dir` -> the job file's directory -> the CWD, and refuses naming the roots it tried. The two path families in one job file follow different rules.
- workaround used: the job carries `"base_dir": REPO` for the STEP path, and `programs/run_tools.py` lists `bom_audit.py` in both `FROM_REPO` (launch from the repo root, so the repo-root-relative receipt path lands correctly) and `NO_OUT` (capture the last stdout JSON line and rewrite it atomically with mkstemp+os.replace). No tools edited.
