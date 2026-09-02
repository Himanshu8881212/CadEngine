# FRICTION — slas_microplate_row_index_stage

## F1 — the boolean tessellator self-intersects on a tangential union, silently (2026-08-08)
- symptom: `validate` on `guide_bridge` returned
  `{"valid": true, "closed": true, "manifold": true, "shells": 1,
    "geometric_ok": false,
    "self_intersection": {"pairs": 39, "point": [49.973724365234375,
     -15.49154281616211, 51.0], "triangles": [145252, 584830]}}`
  and `export_stl` on the SAME solid reported `route: "exact"`,
  `watertight: true`, `two_manifold: true`, `non_orientable_edges: 0`.
  Nothing in the DELIVERABLE_SPEC §2 gate list except `geometric_ok` sees this.
- minimal repro: a 200 x 30 x 12 mm box with 12 teardrop cutters differenced
  out of it, each cutter = `cylinder(r=3.25, segments=128)` unioned with a
  triangular prism whose two base vertices lie ON the true circle at +/-50 deg
  from the peak axis (the textbook tangent teardrop).
  `kernel-api run prog.json --out-dir out/` -> exit 0, `validate` on the
  result gives `geometric_ok: false, self_intersection.pairs: 16`.
- expected vs actual: DESIGN_GUIDE / ops_core describe `cylinder` as a solid
  with exact analytic surface tags, which invites treating a point at radius r
  as being ON its surface. It is not: the tessellation is an INSCRIBED n-gon,
  so such a point sits up to r*(1-cos(pi/n)) OUTSIDE the facets and creates
  exactly the 1e-6..0.1 mm sliver the boolean-hygiene rule in
  OPERATOR_BRIEF §8 forbids. The hygiene rule is right; what is missing from
  the docs is that a *tangency to a curved primitive* is an instance of it.
- workaround used: every guide bore is built as ONE exact circumscribed
  polygon (`parts_bridge.teardrop_polygon`) instead of a boolean union, and
  `geometric_ok: true` was added to the `validate` gate of every part in this
  campaign. Sliver gone, area exact by shoelace.

## F2 — polygon arc pitch: FINER tessellation makes booleans WORSE, ~120x slower (2026-08-08)
- symptom: with the polygon teardrop above, cutting 12 bores through the beam:
  `arc_deg 5.0` -> `geometric_ok false`, `self_intersection.pairs 6`, 87 s;
  `arc_deg 10.0` -> `geometric_ok false`, pairs 1;
  `arc_deg 13.0 / 16.0 / 20.0` -> `geometric_ok true`, 0.7 s.
- minimal repro: same beam, `teardrop_polygon(dia=6.5, arc_deg=A)` for
  A in {5, 10, 13, 16, 20}, `linear_pattern` count 12 step [9,0,0],
  `difference`, `validate`.
- expected vs actual: nothing in the digests warns that a finer polygon can
  turn a clean boolean into a self-intersecting one. The trigger is a
  near-collinear vertex: at arc_deg 5 the gable flank (40.0 deg from the up
  axis) meets the first arc facet (37.5 deg) at a 2.5 deg kink, next to a
  0.284 mm edge. Coarser facets widen that kink.
- workaround used: arc_deg 13.0 (260 deg / 20 facets), circumscribed so the
  INSCRIBED bore diameter stays exactly nominal. Both the measurement table
  and the reason are shipped in `programs/parts_bridge.py`.

## F3 — `support_report` reports a flat ceiling as BRIDGING, so `steep_area == 0.0` cannot falsify it (2026-08-08)
- symptom: `guide_bridge` at `build_dir [0,0,1]` (upright: a 144 mm flat arch
  ceiling) reports `steep_area: 0.0, support_free: true`, together with
  `bridge_area: 4851.324590233379, max_bridge_span: 19.428146362304688`.
  An oracle negative control built on `steep_area == 0.0` therefore PASSES at
  the orientation it was written to reject (exit 0).
- minimal repro: `{"op":"support_report","in":"guide_bridge",
  "build_dir":[0,0,1],"overhang_deg":45,"require":{"steep_area":0.0}}` on the
  shipped `programs/part_guide_bridge.json` geometry -> exit 0.
- expected vs actual: DELIVERABLE_SPEC §2.5 says *"a 'support-free' claim
  requires `steep_area == 0.0` exactly"* and §2.13 names
  `steep_area == 0.0` as an oracle that must have a twin. On this geometry
  that oracle is NOT sufficient on its own: the worst orientation passes it.
  `max_bridge_span` is what fires. Suggest the spec say both.
- workaround used: TWO twins shipped —
  `nc_oracle_D1_support_steep` ([0,1,0], fires `steep_area` at 639.86 mm^2)
  and `nc_oracle_D2_support_bridge_span` ([0,0,1], fires `max_bridge_span` at
  19.428 mm against a 5.0 limit). Both exit 1. Every part in this campaign
  gates `max_bridge_span` as well as `steep_area`.

## F4 — `kernel-api asm` on a mesh-sourced `.lmcasm` is very slow (2026-08-08)
- symptom: `asm_save` writes instance sources as MESHES
  (`{"source": {"mesh": "parts/base_rail.stl"}}`, 88 888 triangles for the
  rail). Re-running the saved file with
  `kernel-api asm assembly/slas_row_index_stage.lmcasm --out-dir …` had not
  produced its stdout receipt after >15 minutes of wall clock, although it
  had already written `bom.csv`, `bom.json`, a per-instance mesh directory and
  a 128 MB merged assembly STL into the out-dir. The process was terminated at
  ~30 min to return the core to the shared budget; the two 128 MB mesh
  artefacts were deleted and only the BOM kept. See
  `receipts/lmcasm_run_ATTEMPTED.md`. The equivalent in-program route (`asm_instance` on bound
  solids + `asm_solve` + `asm_contacts` at window 30) completes in 27 s.
- minimal repro: `programs/assembly.json` (in-program, 27 s) vs
  `kernel-api asm assembly/slas_row_index_stage.lmcasm` (>15 min) on the same
  six instances.
- expected vs actual: implicit_recipes §8.2 documents the `asm` runner as the
  ground-truth route and does not warn that a mesh-sourced save re-solves and
  re-contacts against raw meshes rather than against the exact B-reps the
  in-program ops hold.
- workaround used: the campaign's PRIMARY assembly receipt is the in-program
  one (`receipts/assembly.json`), which carries the mates, the DOF block, the
  contacts and the interference proofs. The saved `.lmcasm` ships as an
  interchange artefact; its re-run is left running out of band and is NOT on
  the critical path of any claim.

## F5 — ace_fea_tet reports a gmsh PLC refusal as `internal` / exit 1, not a refusal / exit 2 (2026-08-08)
- symptom: `python3 tools/ace_fea_tet_runner.py <job> --out <receipt>` on a
  kernel-exported STL that the kernel itself signs off (`route: exact`,
  `watertight: true`, `two_manifold: true`, `shells 1`, `components 1`) returns
  verbatim `"error": "Exception: Invalid boundary mesh (overlapping facets) on
  surface 2 surface 3"`, `"error_kind": "internal"`, and **exit 1**.
- minimal repro: `laboratory_system/slas_microplate_row_index_stage/programs/job_tet_bridge_3p0.json`
  and `..._2p0.json` (identical but for `elem_size_mm`), against
  `laboratory_system/slas_microplate_row_index_stage/parts/guide_bridge.stl`.
  Both element sizes give the same message.
- expected vs actual: OPERATOR_BRIEF s3.1 defines exit **1** as "the tool could
  not run the request — usage, unreadable job, internal error. NO analysis was
  performed" and exit **2** as "the tool RAN and REFUSED". The runner DID run:
  it loaded the STL, invoked the mesher, and the mesher rejected the geometry.
  That is a refusal (`refusal.*`), and s5.1 of the brief lists exactly this
  class of gmsh/PLC message as a refusal. Classifying it `internal` / exit 1
  makes a caller that branches on `error_kind` treat a real geometry refusal as
  a broken request, and hides it from any "record every refusal" sweep.
- workaround used: the ATTEMPTED receipts are shipped as the result of analysis
  row A6 (`receipts/stage3/tet_bridge_{3p0,2p0}_ATTEMPTED.json`) and
  ANALYSIS.md s4.1 records the message verbatim together with this
  misclassification. No deliverable depends on A6; it was budgeted as a MAYBE.

## F6 — a design dimension was silently acting as a printer rule in our own gate suite (2026-08-08)
- symptom: `geom.gate_suite` defaulted `wall_thickness.flag_below` and the
  `p05_thickness` minimum to `F["pawl_arm_t"]`. That was 1.60 mm in stage 2 -
  numerically identical to the house four-perimeter rule - so the coupling was
  invisible. The stage-3 optimizer moved `pawl_arm_t` to 1.74 mm and EVERY
  OTHER part's wall gate tightened to 1.74 with it, silently and with exit 0.
- minimal repro: bake `programs/stage3_optimum.json`, run
  `programs/gen_parts.py`, and read `guide_bridge_wall.flag_below` in
  `receipts/part_guide_bridge.json`: 1.74, not 1.60.
- expected vs actual: not an engine bug - a campaign bug, logged here because
  it is a general trap and the class of defect the portfolio verdict keeps
  finding. A gate threshold must never be spelled as a design dimension: a
  design change then moves the gate instead of being caught by it.
- workaround used: `geom.HOUSE_WALL = 1.60` named explicitly and used by
  `gate_suite`; all seven bodies re-run. Recorded in ANALYSIS.md s2.

## F7 — `bom_audit` matches ANY quoted string in the STEP, so a natural `name_pattern` sweeps up AP203 keywords (2026-08-08)
- symptom: `bom_audit.py` with `"name_pattern": "[a-z][a-z0-9_]+"` on a
  6-body assembly reported 8 findings, none of them about our parts:
  `station_A: axis x347 is NOT in the unified BOM`,
  `... refdir x318 ...`, `... config_control_design x-3 ...`,
  `... distance_accuracy_value x-3 ...`, `... design x10 ...`,
  `... placement x2 ...`, `... mechanical x3 ...`, `... assembly x0 ...`.
  Note the NEGATIVE counts: the calibrator subtracts a per-name metadata
  overhead that these keywords do not have, so they read x-3.
- minimal repro:
  `python3 tools/bom_audit.py laboratory_system/slas_microplate_row_index_stage/programs/doc/bom_audit.json`
  with `name_pattern` set to `[a-z][a-z0-9_]+` against
  `assembly/scene_station_A.step`.
- expected vs actual: the docstring says the pattern selects "the quoted
  instance names the audit counts", and the digest calls them "NAUO instance
  names". The implementation is `re.findall(rf"'({pattern})'", step_text)`
  over the WHOLE file, so it counts every quoted token in the AP203 header,
  `PRODUCT_DEFINITION_CONTEXT`, `AXIS2_PLACEMENT_3D` names and so on — not
  instance names. The default `hw_[a-z0-9_]+` hides this because no AP203
  keyword starts with `hw_`; any campaign that does not use that prefix walks
  straight into it. A restriction to NAUO/PRODUCT rows would make the tool do
  what it says.
- workaround used: a pattern that cannot collide with the AP203 vocabulary —
  `(?:plate_gauge_[a-z0-9]+|[a-z0-9]+_[a-z0-9]+)` (two-token snake_case plus
  our gauge family). Its limit — it would not see an undeclared body whose
  name has three or more tokens outside `plate_gauge_` — is declared in
  `programs/doc/bom_audit.json` under `_pattern_limit`, and the audit is
  oracle-checked (`bom_audit_ORACLE_missing_pawl.json`, exit 1) so it is
  proven able to fail.

## F8 — `assembly_doc` title block clips a long `project` string instead of shrinking or wrapping it (2026-08-08)
- symptom: with `"project": "slas_microplate_row_index_stage"` (31 chars) the
  PROJECT cell of the title block renders the text overflowing the left page
  border — the first characters are cut off by the sheet edge in
  `renders/slas_row_index_stage_assembly_doc.png`. Nothing in the receipt
  mentions it; `{"ok": true, ...}` either way.
- minimal repro: `python3 tools/assembly_doc.py
  laboratory_system/slas_microplate_row_index_stage/programs/doc/asmdoc.json`
  and look at the bottom-left title-block cell.
- expected vs actual: the DOC TITLE cell in the same title block autoshrinks
  to fit; the PROJECT cell does not. A campaign directory name is the natural
  value for `project`, and campaign names in this repo routinely exceed 30
  characters, so the field is clipped by default rather than by accident.
- workaround used: none — the TRUE campaign name is kept rather than
  abbreviated to make a render look tidy, and the cosmetic clip is recorded
  here. Purely presentational: no BOM row, balloon, step or dimension is
  affected.

## F9 — kernel rebuilt 2026-08-10 now refuses `asm_export` on intentionally-interpenetrating NC scenes, breaking end-to-end re-run of the fail poses (2026-08-14)
- symptom: `kernel-api run programs/pose_nc3_halfstep_fail.json` (unchanged
  since 2026-08-08, receipt on disk `ok true`) now exits **1**: every measure
  op still passes and `iv_pawl_rack` reproduces `overlap_volume 17.82` mm3
  EXACTLY, but the final `asm_export` op returns verbatim
  `"kind": "invalid_geometry", "message": "op 'export': refusing manufacturing
  output: boundary_edges=0, non_manifold_edges=0, non_orientable_edges=0,
  non_manifold_vertices=0, degenerate_triangles=0, proper_self_intersections=8"`.
  The larger fail scenes are also far slower: `pose_nc1_lift_fail.json`
  (88k-triangle rail in the scene) ran >2 min where the 2026-08-08 pass
  completed the whole 20-pose suite quickly.
- minimal repro: `"target/release/kernel-api" run
  "laboratory_system/slas_microplate_row_index_stage/programs/pose_nc3_halfstep_fail.json"
  --out-dir <scratch>` with the binary dated 2026-08-10 12:09 (receipts are
  dated 2026-08-08; `pose_nc3_halfstep_control.json`, non-interpenetrating,
  still exits 0).
- expected vs actual: for MANUFACTURING output the refusal is right — a
  self-intersecting scene STL is not printable. But an NC failure attitude is
  DESIGNED to interpenetrate (`overlap_volume > 0` is the whole claim), so the
  new check makes any diagnostic scene export in a fail pose fail the run.
  There is no per-op opt-out (e.g. `allow_self_intersections` for
  diagnostic/scene exports) visible in `describe`.
- workaround used: none applied to shipped artifacts — the shipped 2026-08-08
  receipts remain the evidence and every OVERLAP number still reproduces
  before the export op runs (verified live for NC3: 17.82 exact). Noted for
  the docs: under kernel >= 2026-08-10 the four `pose_nc*_fail.json` programs
  exit 1 AT THE EXPORT OP ONLY, and that refusal itself corroborates the
  interpenetration claim. If a future stage regenerates receipts wholesale, it
  should drop or fence the `asm_export` op in the four fail poses (a program
  change, to be re-receipted then, not silently now).

- RETIRED 2026-08-24: engine round 4 (2026-08-23,
  `campaign/fixlog/H-census-round4.md`) exports merged scenes as
  diagnostics — the refusal this entry pinned no longer fires. The four
  fail poses run end-to-end again (fresh NC3 receipt: exit 0,
  `overlap_volume 17.82` reproduced, export op `ok` with `scene: true`,
  `cross_instance_self_intersections: 8` — the very count the refusal
  quoted). All shipped pose/assembly receipts re-earned wholesale; the
  refusal-era evidence is kept frozen in the two `*_ATTEMPTED` receipts.
  See the campaign BUILD_LOG 2026-08-24 entry.

## F10 — F9's scope is wider: the 2026-08-10 kernel refuses scene export on the LEGAL poses and the assembly too, not just the NC fail attitudes (2026-08-14)
- symptom: during the stage-4 self-check fresh re-run, `kernel-api run` on
  `programs/assembly.json` and on 18 of the 19 non-NC3-control pose programs
  (all unchanged since 2026-08-08, shipped receipts all `ok true`) now exits
  **1 at the final scene/export op only**. Verbatim from the assembly re-run:
  `"kind": "invalid_geometry", "message": "op 'scene': refusing manufacturing
  output: boundary_edges=0, non_manifold_edges=0, non_orientable_edges=0,
  non_manifold_vertices=1, degenerate_triangles=0,
  proper_self_intersections=8"`; from `pose_station_A`: same shape with
  `non_manifold_vertices=1, proper_self_intersections=6`. Every op BEFORE the
  final export passes in every program, and every overlap gate reproduces its
  2026-08-08 number exactly (NC fails 270.918 / 949.590 / 17.82 mm3, legal
  twins 0.0, overcentre designed sweep 80.73). Only
  `pose_nc3_halfstep_control.json` (two-body floated scene) still exits 0.
  Station poses now take ~45-60 s each; the NC1/NC2 scenes minutes.
- minimal repro: `"target/release/kernel-api" run
  "laboratory_system/slas_microplate_row_index_stage/programs/pose_station_A.json"
  --out-dir <scratch>` with the binary dated 2026-08-10 12:09.
- expected vs actual: an assembled attitude legitimately contains exact-contact
  faces and the designed I8 press-seat, so its merged scene mesh always
  carries coincident/intersecting facets (`proper_self_intersections` 6-8,
  `non_manifold_vertices` 1) even though every pairwise `overlap_volume`
  gate measures 0.0. The 2026-08-08 kernel exported these diagnostic scene
  STLs; the new manufacturing-validity check refuses them, with no per-op
  opt-out for diagnostic/scene exports visible in `describe`.
- workaround used: none applied to shipped artifacts. The shipped 2026-08-08
  receipts remain the evidence; the stage-4 self-check ran every pose to a
  scratch dir, confirmed the failure is at the LAST op only and that every
  measure gate reproduces exactly, and shipped two ATTEMPTED receipts
  (`receipts/stage4/pose_nc3_fail_kernel20260810_ATTEMPTED.json`,
  `receipts/stage4/pose_station_A_kernel20260810_ATTEMPTED.json`). The
  Reproducing sections in README/ANALYSIS carry the annotation. A wholesale
  receipt regeneration now requires fencing/dropping the final scene-export
  op in assembly.json and every pose program (a program change, to be
  re-receipted then, not silently now).

- RETIRED 2026-08-24 with F9: the round-4 engine runs `assembly.json` and
  ALL 20 pose programs end-to-end (station A fresh receipt: exit 0,
  export `ok`, `cross_instance_self_intersections: 6` on the record —
  matching the refusal-era counter). No program was fenced or changed —
  the 2026-08-08 programs run as authored; receipts and scene STLs
  re-baselined under round 4; `gen_analysis.py` now guards BOTH ways
  (frozen ATTEMPTED history must keep its refusal; re-earned receipts
  must stay green through their scene export). See BUILD_LOG 2026-08-24.

## F11 — tool receipts under the post-2026-08-10 toolchain carry a new envelope schema, so `core_digest` no longer matches the shipped 2026-08-08 receipts even though every measured value is bit-identical (2026-08-14)
- symptom: the stage-4 self-check re-ran 37 tool jobs (6 production_check, 4
  fatigue, 2 contact, 3 fea, 2 optimize, 20 tolerance_stack) with `--out` to a
  scratch dir and compared `determinism.core_digest` against the shipped
  receipts: fea digests MISMATCH, and most older receipts have no
  `determinism` block at all. Field-level diff shows the cause is the
  toolchain, not the physics: the new receipts add
  `runtime_environment`, `validation_applicability`,
  `validated_range.job_discretization_*`, change `geometry_hash` from
  `program:sha256:...` to `sampled-program:sha256:...`, and report
  `validation_status` `"demonstrated"` with verbatim reason
  `"solver/runtime checkout or dependency environment is not
  release-reproducible"` where the same solve on 2026-08-08 reported
  `"validated"`.
- minimal repro: `python3 tools/ace_fea_runner.py
  "laboratory_system/slas_microplate_row_index_stage/programs/job_fea_bridge_v10.json"
  --out <scratch>/fea_bridge_v10.json` then compare against
  `receipts/stage3/fea_bridge_v10.json`: `max_von_mises_pa
  912722.869293238` and `max_displacement_m 3.83898430247751e-05` are
  bit-identical, `determinism.core_digest` differs.
- expected vs actual: DELIVERABLE_SPEC s3 promises `core_digest` is the
  cross-run comparison. That holds within one toolchain; across the
  2026-08-08 -> 2026-08-10+ update the digest covers the new envelope
  fields, so it cannot certify receipt equivalence across tool versions.
- workaround used: `programs/verify_toolchain_drift.py` strips the volatile /
  envelope fields and deep-compares everything else; receipt
  `receipts/stage4/toolchain_drift_payload_compare.json`: 37 pairs, 32
  payload-identical, 5 with ADDED-schema-fields only, 0 measured-value
  diffs. Shipped receipts stand; docs annotated.
