# FRICTION — cubesat_1u_dev_frame

## F1 — `difference` refuses a solid that carries BOTH end chamfers and a vertical-edge fillet (2026-08-07)
- symptom: `{"kind":"invalid_geometry","message":"op 'n': difference failed validate():
  closed=false manifold=false genus=2 euler_characteristic=-1 shells=2 — refusing to
  bind an invalid solid"}`.  The cutter is a plain box removing a corner prism from a
  rail; it never comes near the treated edges.
- minimal repro (`kernel-api run n_full.json --out-dir out`):
  ```json
  {"ops":[
   {"id":"r","op":"box","min":[0,0,0],"max":[8.5,8.5,113.5]},
   {"id":"c00","op":"chamfer_edge_near","in":"r","witness":[4.25,0,0],"radius":0.5},
   {"id":"c01","op":"chamfer_edge_near","in":"c00","witness":[8.5,4.25,0],"radius":0.5},
   {"id":"c02","op":"chamfer_edge_near","in":"c01","witness":[4.25,8.5,0],"radius":0.5},
   {"id":"c03","op":"chamfer_edge_near","in":"c02","witness":[0,4.25,0],"radius":0.5},
   {"id":"c10","op":"chamfer_edge_near","in":"c03","witness":[4.25,0,113.5],"radius":0.5},
   {"id":"c11","op":"chamfer_edge_near","in":"c10","witness":[8.5,4.25,113.5],"radius":0.5},
   {"id":"c12","op":"chamfer_edge_near","in":"c11","witness":[4.25,8.5,113.5],"radius":0.5},
   {"id":"c13","op":"chamfer_edge_near","in":"c12","witness":[0,4.25,113.5],"radius":0.5},
   {"id":"f","op":"fillet_edge_near","in":"c13","witness":[0,0,56.75],"radius":1.0},
   {"id":"nb","op":"box","min":[1.655,4.515,62.0],"max":[10.5,10.5,115.5]},
   {"id":"n","op":"difference","a":"f","b":"nb"}]}
  ```
  Isolation matrix (all executed 2026-08-07):
  | operand | cutter | result |
  |---|---|---|
  | 8 end chamfers only | corner box | **OK** (valid, shells 1) |
  | R1.0 vertical fillet only | corner box | **OK** (valid, shells 1) |
  | 8 end chamfers + R1.0 vertical fillet | corner box | **FAIL** invalid_geometry |
  | 8 end chamfers + 1.0 vertical *chamfer* | corner box | **FAIL** invalid_geometry |
  | 8 end chamfers + R1.0 fillet | box fully INSIDE the solid | OK |
  So it is the pair "pad chamfers meeting a treated vertical edge" + a cutter that
  crosses the boundary faces; either treatment alone is fine.
- expected vs actual: OPERATOR_BRIEF §8 and ops_core §7 warn that *edge features after
  booleans* fail on fragmented faces, and prescribe "ease edges on primitives first,
  boolean last".  Following that prescription exactly is what breaks here — the boolean
  after the eased primitive is the failing op.  Doc says the order is safe; binary
  refuses it.
- workaround used: build the notched rail WITHOUT any cutter.  Each rail is two stacked
  prisms — a full 8.5x8.5 box below the board seat and an extruded L-section above it —
  each edge-treated on its own virgin primitive and then `union`ed on the coincident
  z=62 plane.  No difference op touches a rail.  (`programs/gen_part.py`, rails section.)

## F2 — `chamfer_edge_near` refuses the two top edges incident to a reflex corner (2026-08-07)
- symptom: `{"kind":"invalid_geometry","message":"op 'tc2': chamfer_edge_near failed
  validate(): closed=false manifold=false genus=1 ..."}` when chamfering the top-face
  edges of an extruded L-section at the edges that meet the concave (reflex) vertex.
- minimal repro: extrude profile `[[0,0],[8.5,0],[8.5,4.515],[1.655,4.515],[1.655,8.5],[0,8.5]]`
  height 51.5, then `chamfer_edge_near` at witness `[5.08,4.515,51.5]` radius 0.5.
  Chamfering the four edges NOT incident to the reflex vertex succeeds (verified:
  valid, shells 1, exact_volume 2301.9802); doing the reflex-incident pair first fails
  identically, so it is not an ordering artefact.
- expected vs actual: ops_core §7 scopes `chamfer_edge_near` to "CONVEX straight edges
  between two PLANAR faces" and refuses *concave junctions*.  These edges ARE convex
  (top face against a side face) — only the vertex they terminate at is reflex.  The
  refusal is correct-ish but the documented scope does not predict it.
- workaround used: the four guide-channel top edges are left sharp; the four outer pad
  edges carry the 0.5x45 chamfer.  The board lead-in chamfer named in DESIGN.md F20 is
  therefore NOT modelled — recorded as amendment A9 and as an open item, not silently
  dropped.

## F3 — `solid_from_implicit` has no `mesher` parameter, so TPMS fields cannot reliably
enter the exact solid environment (2026-08-07)
- symptom: `{"kind":"invalid_geometry","message":"op 'g0': mesh_to_solid: mesh is not
  watertight even after weld(0.00001): 91 non-manifold/boundary edges remain (was 91
  before weld, 309200 triangles)"}` for a gyroid sheet at cell 5.0 mm / voxel 0.4, and
  the same failure for a `offset_by`-graded gyroid at voxel 0.5 and 0.7.
- minimal repro: `{"id":"g","op":"solid_from_implicit","voxel":0.5,"expr":
  {"op":"intersection","a":{"op":"offset_by","max_abs":0.75,"in":{"shape":"gyroid",...},
  "field":{...}},"b":{"shape":"box",...}}}`
- expected vs actual: implicit_recipes §1 "Mesher choice" states TPMS/gyroid/junction-rich
  fields **require** `"mesher":"manifold"`, and the `implicit` op accepts it (`describe`
  lists `mesher` for `implicit`).  `describe` for `solid_from_implicit` lists only
  `expr`, `voxel`, `domain` — no `mesher` — so the one op that BINDS a solid from a
  field is locked to the narrowband mesher that the docs say must not be used for TPMS.
  `hybrid_boolean` is not an escape: it binds no geometry (verified — `validate` on its
  id returns `missing_ref: 'hb' is a measure/export op and binds no geometry`) and its
  own exact route self-demoted (`"route":"voxel_healed","healed_reason":"exact
  arrangement failed validation (closed=false manifold=false genus=65 shells=8)"`).
- workaround used: TPMS content is restricted to fields the narrowband mesher survives
  (uniform-thickness gyroid, cell >= 8 mm, voxel >= 0.5, wall >= 3 voxels) and to a size
  the exact-solid route can carry — see DESIGN.md amendment A10 for the measured
  face-count/STEP-size budget that forced corner gussets instead of full-height panels.

## F4 — parity-fill voxelizers report a phantom 9-voxel "sealed void" inside solid material (2026-08-07)
- symptom: `air_topology_audit.py` on the shipped, exact-route, watertight STL returns
  `{"ok": true, "components": 2, "sizes_cm3": [1058.79, 0.01], ...}` — a second internal-air
  component. The same phantom appears in `voxelize_stl.py`'s independent 3-D parity grid:
  `scipy.ndimage.label(~solid)` finds 9 cells at index bbox `min [91 96 4] max [99 96 4]`
  (world x [91,100], y [96,97], z [4,5]) — i.e. 9 mm^3 of "air" buried inside the +X/+Y
  corner rail, 2.75 mm BELOW the body face. Re-running the audit at `voxel_mm: 0.7`
  (incommensurate with every 0.5 mm design plane) still reports `components: 2`
  (second size rounds to 0.00 cm^3), so it is not a single-pitch alignment accident.
- minimal repro:
  `python3 tools/air_topology_audit.py aerospace_system/cubesat_1u_dev_frame/programs/physics/a10_airtopo_frame.json`
  (stl = parts/cubesat_1u_dev_frame.stl, voxel_mm 1.0, wall_margin_mm 0).
- expected vs actual: tools_cookbook §3 presents `air_topology_audit` as the gate on the
  VOID and its `components` count as the finding. Taken at face value the receipt says the
  part has a sealed void. The exact B-rep disagrees: `g_valid` on the same body reads
  `shells: 1, closed: true, manifold: true, genus: 9` — an enclosed void would be a second
  shell. The artifact sits exactly on the coincident face where the cutter-free rail's
  L-prism meets its corner block (the construction forced by F1), which is where a
  scanline parity count is most fragile.
- workaround used: do not trust a voxel void census against an exact body — ADJUDICATE it.
  `programs/gen_void_probe.py` -> `programs/void_probe_program.json` drops a 4.0 x 0.6 x 0.6
  = 1.44 mm^3 probe box into the suspect cells and measures `clearance` against the shipped
  solid: `v_solid overlap_volume 1.4400 mm^3` (= 100% of the probe, i.e. fully buried in
  material), `v_solid2` 1.4400 at the diagonally opposite rail, and the control probe in the
  open bay `v_air overlap_volume 0.0, distance 43.908`. Verdict: artifact, no sealed void.
  The A10 receipt is shipped WITH this adjudication attached; the audit's `connected` pairs
  (both harness teardrops <-> board bay = true) are the part of it that is load-bearing.

## F5 — ace_fea's default Jacobi-CG cannot solve a 3e5-DOF frame, and a hung solve is indistinguishable from a slow one (2026-08-07)
- symptom: `ace_fea_runner.py` on the shipped frame at the campaign's declared 1.0 mm grid
  (shape [100,100,114], 81198 active elements, 346938 DOF, `direct_solver_max_dof: 0` = the
  documented default "always Jacobi-CG") produced **no output and no receipt after 30 minutes**
  of single-threaded wall clock. The only line ever written to stderr was
  `loaded density grid (100, 100, 114) from .../frame_vox_1p0.npy`. A 1.5 mm retry (~99k DOF)
  with `direct_solver_max_dof: 200000` also failed to land inside the budget. The SAME job at
  2.0 mm (43650 DOF) converges in **24.5 s** (`receipts/fea_railload.json` `timings_s.fea_s`).
  `ace_modal` on the IDENTICAL 1.0 mm grid solves in 151 s, so the grid itself is tractable —
  it is specifically the static CG that does not.
- minimal repro: `python3 tools/ace_fea_runner.py
  aerospace_system/cubesat_1u_dev_frame/programs/physics/a3_fea_railload.json`
  (npy geometry, 4 x 300 N bbox point loads on the +Z rail ends, clamped plane z<=0.5 side '-').
- expected vs actual: `digests/tools_cookbook.md` documents `direct_solver_max_dof: 0` as the
  default and warns only that "SuperLU needs ~10 GB at 237k DOF" — i.e. it steers you toward CG
  at exactly the size where CG stops working, and gives no guidance on the Jacobi-CG iteration
  budget for a slender frame (condition number is terrible for a 113.5 mm column made of 3 mm
  plates). Worse for an agent: the runner emits **no iteration counter, no residual trace and no
  heartbeat**, so "still converging" and "wedged" look identical, and the receipt's own
  `residual_source` field admits "Per-iteration count is not exposed by ACE's return (read-only)".
  Two half-hour blocks of a shared 6-agent box were spent finding this out.
- workaround used: DESIGN.md amendment A14 — the campaign declares **two** grids instead of one,
  1.0 mm for modal (A1/A2) and 2.0 mm for every static solve (A3/A4/A5 prestress, A7), both on
  `origin_mm [0,0,0]` so selectors keep world coordinates, with the grid-sensitivity cost of the
  coarse static grid measured and published in `receipts/opt_grid_calibration.json` rather than
  assumed away.

## F6 — doc drift: ace_fatigue's stress block must be NESTED under "stress", and param_optimize's command timeout is undocumented (2026-08-07)
- symptom (a): a fatigue job written exactly as the cookbook's schema line reads
  (`"sigma_ref_mpa": 4.15` at the top level) is refused with
  `{"ok": false, "error": "JobError: stress block required: {npy,...} or {sigma_ref_mpa} or {sigma_ref_pa}"}`.
  The runner reads `job.get("stress")` (`tools/ace_fatigue_runner.py:283`); the key must be
  `"stress": {"sigma_ref_mpa": 4.15}`.
- symptom (b): `param_optimize.py`'s v2 command evaluator wraps every candidate in
  `subprocess.run(..., timeout=float(ev.get("timeout", 300)))` (`tools/param_optimize.py:166`).
  A physics-in-the-loop evaluator that takes longer than 300 s per candidate — which any real
  solver loop does — silently dies unless you set `evaluator.timeout`. The cookbook's v2
  description lists `argv` and `job_template` but not `timeout`.
- minimal repro (a): `python3 tools/ace_fatigue_runner.py` on
  `{"out_dir":"/tmp/f","material":"PLA","load_orientation":"in_plane","sigma_ref_mpa":4.15,
    "spectrum":[{"cycles":100000,"r_ratio":0.0}]}`.
- expected vs actual: `digests/tools_cookbook.md` section "ace_fatigue_runner.py" writes the stress
  alternatives as bare braces in a list of top-level job keys, which reads as top-level keys; and
  its param_optimize section documents the v2 evaluator without the timeout knob.
- workaround used: nest the stress block (`programs/physics/r4_fatigue_across_layer.json`,
  `r4b_fatigue_in_plane_control.json`) and set `"evaluator": {..., "timeout": 1800}` in
  `programs/physics/opt_param_optimize.json`. Both are one-line fixes once you read the source;
  the cost is that the first failure looks like a broken job rather than a doc gap.

## F7 — analysis_sheet.py view panels crash with a bare KeyError when a load has no `label` (2026-08-07)
- symptom: `python3 tools/analysis_sheet.py job.json` died with
  `File ".../tools/analysis_sheet.py", line 155, in view_panel: lw_px = rs.text_w_px(ld["label"], ...)`
  → `KeyError: 'label'`. No job-validation message, no hint which panel or which load; the tool
  still exited 0 (it prints the traceback and returns without a receipt), so a shell gate keyed on
  `$?` would have called this a success with no PNG written.
- minimal repro: a job with
  `{"panels":[{"kind":"view","stl":"x.stl","loads":[{"at":[0,0,0],"dir":[0,0,-1]}]}], "results":[], "out":"o.png"}`
  → same traceback. Adding `"label":"300 N"` to the load fixes it.
- expected vs actual: `campaign/digests/tools_cookbook.md` §5 documents the view panel as
  `{kind:"view", caption, stl, loads, fixture}` and does not say `loads[].label` is REQUIRED
  (the neighbouring `fixture.label` genuinely is optional — `spec["fixture"].get("label","clamped")`
  — so the asymmetry is invisible from the docs). Expected an optional annotation; got a hard crash.
- workaround used: `programs/gen_docs.py` emits a `label` on every load
  (`programs/docs/analysis_sheet_job.json`). Renders clean: `renders/frame_analysis_sheet.png`,
  receipt `{"ok": true, "panels": 4}`.

## F8 — analysis_sheet field panels have no unit conversion of their own (2026-08-07)
- symptom: the A3 stress panel's colour bar read `1.92e+07 MPa` while the panel's declared
  `"unit": "MPa"` was taken verbatim — the ace_fea `stress_field.npy` is in **Pa**, and `unit` is a
  label only, not a conversion. A sheet published without noticing would have overstated every
  stress on it by 1e6 while looking self-consistent.
- minimal repro: any field panel pointing at an `ace_fea_runner.py` `stress_field.npy` with
  `"unit":"MPa"` and no `"scale"`.
- expected vs actual: the cookbook lists `unit` and `scale?` as independent optional keys without
  saying that the voxel runners emit SI (Pa, m) and that `scale` is the only thing that makes
  `unit` true. Expected `unit` to be declarative; it is decorative.
- workaround used: `"scale": 1e-6` on both stress panels in `programs/gen_docs.py`; the thermal
  panel needs none (`T_field.npy` is already °C). Cross-checked against the receipt:
  panel max now reads 19.2 MPa vs `receipts/fea_railload.json` `max_von_mises_pa` 19228245.6.

## F9 — `ace_buckling` load_factors are not bit-reproducible, so a receipt-generated document cannot be byte-stable (2026-08-08, independent verification pass)
- symptom: two runs of the identical job on identical geometry return
  `load_factors[0] = 0.5378860166137663` and `0.5378860166135455` (rel 4e-13).
  Every derived headline number (`critical_load_N`, `design_critical_load_n`)
  rounds identically, but `analysis/ANALYSIS.md` §5 prints the raw list at full
  repr precision, so the generated document changes on every re-run:
  `- | load_factors | [0.5378860166137663, ...]`
  `+ | load_factors | [0.5378860166135455, ...]`
- minimal repro:
  `python3 tools/ace_buckling_runner.py aerospace_system/cubesat_1u_dev_frame/programs/physics/a5_buckling_frame_2p0.json`
  run twice; diff the two receipts' `load_factors`.
- expected vs actual: DELIVERABLE_SPEC §3 Determinism asks that generated
  documents regenerate identically. The kernel geometry path IS bit-exact
  (STL/3MF/STEP/PNG all `cmp`-clean across a full rebuild); the ARPACK-class
  eigenvalue path is not.
- workaround used: none needed for the verdict — the campaign's headline numbers
  are stable to 6+ significant figures. Suggested campaign-side fix is to print
  `load_factors` at a fixed precision (e.g. `%.6f`) in `gen_analysis_body.py` so
  the document is byte-stable; suggested tool-side fix is a fixed random seed /
  deterministic starting vector in the eigensolver.

## F10 — `rerun_physics.py` progress is invisible when stdout is redirected (2026-08-08, verification pass)
- symptom: `python3 programs/rerun_physics.py > log 2>&1` writes nothing to `log`
  until the whole ~25-minute run ends (Python block-buffers a pipe), so a
  long run is indistinguishable from a hung one — the same failure mode the
  campaign already logged as F5 for ace_fea.
- minimal repro: the command above; watch `log` stay 0 bytes for 20+ minutes.
- expected vs actual: the script prints one line per row as it goes; that is
  only true on a tty.
- workaround used: polled receipt mtimes under `receipts/` and `pgrep -f` to
  track progress. `python3 -u` would fix it campaign-side.

## F11 — `air_topology_audit.py` ignores the job's `receipt` key (2026-08-08, repair pass)
- symptom: `programs/physics/a10_airtopo_nc6_plugged.json` carries a top-level
  `"receipt": "<abs path>"` key, exactly like the `tolerance_stack` /
  `joint_check` / `production_check` jobs in this campaign do. Those tools honour
  it and write the file; `air_topology_audit.py` does not. The run prints the
  JSON receipt on stdout and exits 0, and **no file appears** — verbatim:
  `head: receipts/airtopo_nc6_plugged.json: No such file or directory`
  after `python3 tools/air_topology_audit.py .../a10_airtopo_nc6_plugged.json`
  returned `{"ok": false, "components": 2, ...}` on stdout.
- minimal repro:
  ```sh
  cd "/Users/himanshu/Work/New-LMCAD/cad engine"
  python3 tools/air_topology_audit.py \
    aerospace_system/cubesat_1u_dev_frame/programs/physics/a10_airtopo_nc6_plugged.json
  ls aerospace_system/cubesat_1u_dev_frame/receipts/airtopo_nc6_plugged.json   # ENOENT
  ```
- expected vs actual: the `receipt` key is the campaign-wide convention for "write
  the receipt here" and is honoured by at least three other tools; this one
  silently ignores it while still exiting 0. That is the dangerous shape — a
  campaign that relies on the key gets an ok=true run with no receipt on disk,
  and any gate reading that receipt fails on a missing file rather than on a
  measurement. (Same family as the note that `rerun_physics.py` passes
  `out=None` for the tools that DO honour the key: the two conventions only
  coexist because every job happens to carry both.)
- workaround used: the receipt is written by `rerun_physics.py`'s own `run(..., out=...)`
  path instead, which captures the last stdout line and dumps it. The `receipt`
  key is left in the job file as documentation of intent. No tool source touched.

## F12 — `field_triage.creep_allowable()` takes a material DICT, not a material NAME (2026-08-08, repair pass)
- symptom: the obvious call from the campaign side,
  `field_triage.creep_allowable("pla", 23.0, 720.0)`, raises
  `AttributeError: 'str' object has no attribute 'get'` at
  `tools/field_triage.py:424` (`table = (mat.get("creep") or {}).get(...)`).
- minimal repro:
  ```sh
  cd "/Users/himanshu/Work/New-LMCAD/cad engine"
  python3 -c "import sys;sys.path.insert(0,'tools');import field_triage as ft;ft.creep_allowable('pla',23.0,720.0)"
  ```
- expected vs actual: every other material-facing entry point in the campaign's
  reach is addressed by material NAME (`production_check` jobs say
  `"material": "PLA"`), so a name is the natural argument; the function wants the
  parsed `tools/materials/pla.json` dict and says so only in the body, not in the
  signature or docstring. Cheap fix on the tool side would be to accept either.
- workaround used: `programs/gen_analysis.py` loads
  `tools/materials/pla.json` itself and passes the dict, so ANALYSIS.md §4 can
  print the creep cell's `basis` and verbatim `confidence` string next to the
  allowable it derives (DELIVERABLE_SPEC §3 validity limits). No tool source touched.
