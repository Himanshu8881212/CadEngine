# Friction — iso9409_wedge_flexure_gripper

## F1 — `mesh_components` counts one extra component per `extrude_with_holes` hole loop (2026-08-07)
- symptom: a topologically perfect solid fails the mandatory single-body gate.
  `extrude_with_holes` (outer U-frame + 3 hole loops) reports
  `validate -> closed:true manifold:true valid:true genus:3 shells:1
  euler_characteristic:-4`, but
  `{"op":"assert","in":"wedge","components":1}` fails with
  `assert failed: components: measured 4, expected 1`.
  The count is exactly 1 + (number of hole loops).
- minimal repro (a rectangular plate with ONE rectangular through hole — an
  object that is unarguably one body):

```json
{"ops":[
 {"id":"c","op":"extrude","profile":[[-52.5,24],[52.5,24],[52.5,144],[20,144],[20,100],[40,100],[40,42],[-40,42],[-40,100],[-20,100],[-20,144],[-52.5,144]],"height":11.4},
 {"id":"mcc","op":"mesh_components","in":"c"},
 {"id":"a","op":"extrude_with_holes","outer":[[-52.5,24],[52.5,24],[52.5,144],[20,144],[20,100],[40,100],[40,42],[-40,42],[-40,100],[-20,100],[-20,144],[-52.5,144]],"holes":[[[-8.0,27.2],[18.0,27.2],[18.0,37.5],[-8.0,37.5]]],"height":11.4},
 {"id":"mca","op":"mesh_components","in":"a"}]}
```
  `"/Users/himanshu/Work/New-LMCAD/cad engine/target/release/kernel-api" run repro.json --out-dir out/`
  -> `mcc.components = 1, triangles 44` (correct) but
     `mca.components = 2, triangles 52` (wrong; same solid plus one hole).
  Verified stable at `tol` 0.05 (default) and 0.01, so it is not a chord-tolerance effect.
- expected vs actual: ops_core.md "ENGINE UPDATE 2026-08-06" and
  DELIVERABLE_SPEC 2.2 make `assert components:1` the mandatory single-body
  oracle on EVERY part, and say `shells==1` cannot catch severance that
  `components` can. Here the two disagree in the opposite direction: the solid
  is closed/manifold/one-shell with the correct genus, yet `mesh_components`
  reports the hole-wall surfaces as separate faceted components. The measurement
  tessellation of a capping face that carries inner loops appears not to share
  edges between the outer wall and the hole walls.
- consequence if unfixed: any campaign that uses `extrude_with_holes` on a
  gated part cannot satisfy the mandatory gate, and a real severance inside such
  a part would be masked by the constant offset.
- workaround used: build the wedge as `extrude` (outer profile only) followed by
  three `difference` cuts against extruded prism cutters that overshoot both
  faces. The boolean re-tessellation welds correctly and `components` reads 1
  with identical genus/volume. Recorded in analysis/DESIGN.md defects-caught log
  as D6 (engine-side, not a geometry defect). Engine and tools source untouched.

## F2 — `clearance` reports distance 0.0 / interfering:true / overlap_volume:null on provably disjoint curved-face pairs (2026-08-07)
- symptom: for the posed palm+wedge pair (designed 0.30 mm radial gap between an
  O14 follower pin and its O14.6 cam-slot cap, and 0.30 mm between the wedge
  rails and the guide walls) `clearance` returns
  `{"distance": 0.0, "interfering": false, "overlap_volume": 0.0}`, so
  `assert_disjoint {min_clearance: 0.05}` fails on geometry that is disjoint.
  For the posed palm + O6 dowel pin sitting inside its O6.25 bore it returns
  `{"distance": 0.0, "interfering": true, "overlap_volume": null}` — i.e. it
  claims interference AND declines to quantify it.
  The exact route disagrees and is right: `union` + `assert {"shells": 2}`
  PASSES for both pairs, and the same programs' plane-vs-plane pairs read
  correctly (guide wall vs wedge rail measures 0.2999992 mm; adapter float
  measures 0.1000000).
- minimal repro: `programs/poses_nc.json` in this campaign, ops
  `p1_wedge_gap` / `p1_union` / `p1_wedge_disjoint` and
  `nc1_legal` / `nc1_legal_union` / `nc1_legal_gate`; the pair is reproducible
  from two `import_step` bodies plus one `pose`. A cut-down repro is
  `clearance` between a `cylinder` r=7 and a solid containing a co-axial
  r=7.3 through slot.
- expected vs actual: ops_core.md 9 documents `clearance` as "the interference
  measure that does NOT fail on overlap", with `overlap_volume` in mm3 — the
  campaign doctrine (DELIVERABLE_SPEC 2.11) then hangs every must-NOT-fit claim
  on that number. Here the number is either absent (`null`) or the sign is
  wrong, and only on pairs where a curved face is involved; box-vs-box pairs in
  the same program behave correctly (`nc2_fail` -> overlap_volume 72.8 mm3).
- workaround used: all disjointness proofs in this campaign moved to the
  tessellation-independent exact route (`union` + `assert shells == N`), and the
  must-NOT-fit interference claims moved to `intersection` + `exact_volume`
  (a boolean that BINDS is interference; a disjoint pair refuses it with
  `invalid_param ... produced an empty solid`). `clearance` numbers are still
  recorded as receipts but are never the authority. Engine/tools untouched.

## F3 — `tools_cookbook.md` puts the fatigue stress spec at the job top level; the runner requires a `stress` block (2026-08-07)
- symptom: a job written from the cookbook's §1 ace_fatigue paragraph
  ("stress one of `{npy, unit?}` | `{sigma_ref_mpa}` | `{sigma_ref_pa}`") fails with
  `JobError: stress block required: {npy,...} or {sigma_ref_mpa} or {sigma_ref_pa}`
  even though `sigma_ref_mpa` IS present — because the runner reads
  `job.get("stress")` (tools/ace_fatigue_runner.py:283) and the cookbook's wording
  reads as a top-level alternative.
- minimal repro:
```json
{"out_dir":"/tmp/f","material":"PLA","curve":"design","sigma_ref_mpa":10.0,
 "spectrum":[{"cycles":1000,"load_factor":1.0,"r_ratio":0.0}]}
```
  `python3 tools/ace_fatigue_runner.py repro.json` -> the JobError above, exit 1.
  Wrapping as `"stress": {"sigma_ref_mpa": 10.0}` succeeds.
- expected vs actual: the digest lists the three stress forms without naming the
  key they live under; every other block in the same paragraph (`spectrum`,
  `curve`) IS top-level, so the natural reading is wrong.
- consequence if unfixed: a silent-looking hard failure on the first fatigue job
  of every campaign; costs one debug cycle each time.
- workaround used: `"stress": {"sigma_ref_mpa": ...}` in
  `programs/gen_jobs.py`. Docs and tools untouched.

## F4 — `sweep_check` / `param_optimize.call_engine` write station programs to a SYSTEM temp dir, which makes `import_step` unusable in any swept template (2026-08-07)
- symptom: a sweep template containing
  `{"op":"import_step","file":"../_rt/wedge.step"}` fails at EVERY station with
  `invalid_param: op 'wedge': path '../_rt/wedge.step' must not contain '..'
  (it would escape the sandbox)`. There is no spelling that works: `import_step`
  resolves `file` against the PROGRAM FILE's directory and refuses both absolute
  paths and `..`, while `param_optimize.call_engine`
  (tools/param_optimize.py:83) writes the substituted program to
  `tempfile.NamedTemporaryFile(suffix=".json")` in the system temp dir. The two
  contracts are individually reasonable and jointly exclude the feature.
- minimal repro: `programs/jobs/sweep_wedge.json` in this campaign with the
  template's first op set back to
  `{"id":"wedge","op":"import_step","file":"_rt/wedge.step"}`;
  `python3 tools/sweep_check.py programs/jobs/sweep_wedge.json` ->
  `ok:false`, 24/24 `failed_stations`, all with the message above.
- expected vs actual: `digests/exemplars.md` and DELIVERABLE_SPEC 2.11 push
  sweeps onto the SHIPPED geometry; the shipped geometry of an exact-B-rep
  campaign is a STEP file; but a swept program can never load one.
- consequence if unfixed: every sweep must re-model its geometry from ops
  instead of loading what ships, which is exactly the "re-modelled lookalike"
  failure mode the poses/NC doctrine warns against.
- workaround used: `programs/gen_sweep.py` lifts the wedge's construction ops
  verbatim out of the shipped `programs/wedge.json` (slice up to and including
  the solid id `wedge`) instead of importing the STEP — same generator, same
  numbers, no second model. Engine and tools source untouched.

## F5 — `ace_fea_tet` aborts on a watertight, validate-clean STL when the surface carries slender triangles (2026-08-07)
- symptom: `AssertionError: body-fitted mesh has a non-positive corner Jacobian
  (min -1.223e-04 mm^3) — inverted/degenerate element; ref-mesh` on an STL that
  the kernel itself certifies: `validate` -> `valid/closed/manifold`, `genus 0`,
  `shells 1`, `components 1`, `export_stl` -> `route "exact", watertight true`.
  Receipt: `ok:false`, everything else null; exit 0 is NOT the signal (this
  runner follows the ACE convention).
- minimal repro: `programs/gen_kt_specimen.py` with `SEG = 32` and
  `export_stl tol 0.02` (the 3 mm blade-root fillet then tessellates into
  0.147 mm chords next to 6 mm flats), then
  `python3 tools/ace_fea_tet_runner.py programs/jobs/tet_blade_root.json`
  with `elem_size_mm` 0.5 or 1.0.
- expected vs actual: the cookbook's tet row says the geometry input is
  "`stl` (watertight)" with no quality qualifier, and the tool is the prescribed
  route for exactly this job (fillet concentrations the voxel path under-reads).
  A watertight kernel export is not sufficient input.
- consequence if unfixed: the Validated tet path silently becomes unavailable
  for any part whose fillets are finely faceted — i.e. most real fillets — with
  an assertion rather than a diagnosis (which triangle, what to change).
- workaround used: coarsened the specimen's arc to `SEG = 16` and exported at
  `tol 0.05`; the same solid then meshed cleanly (26280 tets,
  `min_corner_jacobian_mm3 1.234e-04`, volume error 0.056 %). Recorded in
  analysis/DESIGN.md 18 as D12's route, not laundered.

## F6 — blind `drill` leaves a 118 deg drill POINT that can breach the far face (2026-08-08)
- symptom: `{"op":"drill","at":[18,26.5,6],"axis":[0,0,-1],"d":20.6,"depth":2.5}` on a
  6.0 mm plate returned `ok:true` with measures
  `{"d":20.6,"depth":2.5,"kind":"blind","point_depth":8.688864375983872}` — the
  conical drill point runs 8.689 mm below the entry face, i.e. 2.689 mm PAST the
  6.0 mm plate. The pocket therefore became a through opening: `validate` read
  `genus 7` where the blind-pocket closed form predicts 6, and `exact_volume`
  read 34677.932 mm3 vs the 35309.109 mm3 target (an extra 631.18 mm3 = exactly
  the clipped point cone pi*3.5/3*(10.3^2+10.3*4.475+4.475^2)).
- minimal repro: `programs/gen_coupon.py` at RECESS depth 2.5 in a T=6.0 plate;
  `"/Users/himanshu/Work/New-LMCAD/cad engine/target/release/kernel-api" run
  optional/fit_coupon.json --out-dir .`
- expected vs actual: `{"op":"describe","name":"drill"}` documents
  `depth?:number` with NO doc string and no mention of a drill point; the digests'
  hole-wizard section describes `depth` as a blind depth. The binary models a real
  118 deg twist drill, so the effective depth is `point_depth`, not `depth`, and a
  large-diameter shallow pocket silently becomes a through hole (the drill op
  itself stays ok:true — only the genus/volume gates catch it).
- workaround used: cut flat-bottomed recesses with an explicit `cylinder` +
  `difference` cutter that overshoots the ENTRY face by 1 mm (a printed pocket is
  flat bottomed anyway). `drill` is kept for through bores only. No engine or
  tools source touched.

## F7 — `tpms` writes its mesh under --out-dir but `hybrid_boolean` reads relative to the PROGRAM dir (2026-08-08)
- symptom: a two-op program `{"op":"tpms",...,"file":"probe_lat.stl"}` followed by
  `{"op":"hybrid_boolean","in":"plate","file":"probe_lat.stl",...}` fails with
  `{"kind":"io","message":"op 'fuse': cannot read '/tmp/probe_lat.stl': No such
  file or directory (os error 2)"}` even though the `tpms` op reported
  `ok:true, watertight:true, triangles:34864` one op earlier. The file exists —
  at `<out-dir>/probe_lat.stl`, not at `<program dir>/probe_lat.stl`.
- minimal repro: program at `/tmp/tpmsout/sub/probe2.json`, run with
  `--out-dir /tmp/tpmsout`; `tpms.file` must be written as `"sub/probe_lat.stl"`
  (out-dir relative) while the very next op's `hybrid_boolean.file` must be
  written as `"probe_lat.stl"` (program-dir relative) to name the SAME file.
- expected vs actual: OPERATOR_BRIEF 3 documents export `file` as joining
  `--out-dir` and `load_part.file` as resolving against the program directory;
  `hybrid_boolean.file` is an INPUT mesh and follows the load_part rule, but its
  natural producer (`tpms`, `implicit`, `gyroid_block`) follows the export rule.
  Any single-program lattice pipeline therefore needs two different spellings of
  one path, and the failure only appears at the consuming op.
- workaround used: none needed in the shipped campaign — the graded-TPMS jaw pad
  was not built (declared unclaimed in README "What has NOT been done" and
  DESIGN.md 28). Recorded so the next campaign does not pay the same cost. No
  engine or tools source touched.

## F8 — `voxelize_stl.py` has no staleness guard: `ace_fea`/`ace_modal`/`ace_thermal` silently consume an out-of-date density field (2026-08-08, independent verification)
- symptom: the shipped `analysis/fields/palm_v15.npy` carries **62552** solid voxels.
  Re-running the README's own command on the byte-identical `parts/palm.stl`
  (`python3 tools/voxelize_stl.py .../jobs/voxelize_palm_v15.json`) prints
  `{"ok": true, "solid_voxels": 64328, ...}` — twice in a row, so the tool is
  deterministic and the FIELD was stale. 4178 cells differ (+2977 / −1201),
  localized to x[−52.8,50.7] y[0,45] z[120.0,160.5] mm (the finger/jaw region
  moved by amendment A13). Every voxel consumer then re-ran with different
  answers: `ace_fea` 2 g peak 3.4292 → 3.3520 MPa and disp 0.29988 → 0.31764 mm,
  `ace_modal` f1 70.5379 → 65.8356 Hz, `ace_thermal` n_solid_voxels 62552 → 64328.
- minimal repro: `python3 tools/voxelize_stl.py \
  "robotics_system/iso9409_wedge_flexure_gripper/programs/jobs/voxelize_palm_v15.json"`
  then compare `solid_voxels` against `n_active_elements` in any shipped
  `receipts/fea_wrench_*_v15.json` / `modal_global_v15.json`.
- expected vs actual: a density field is an INPUT artifact, but nothing in
  `voxelize_stl.py` or the ACE runners records the source STL's hash into the
  `.npy` (or checks it), so a consumer cannot tell a fresh field from one built
  before the last geometry amendment. `ace_fea` DOES emit a `geometry_hash`
  (`density:sha256:…`) but it hashes the density array, not the STL, so it
  changes silently rather than refusing.
- workaround used: none available in-campaign — a verifier has to re-run
  `voxelize_stl.py` and diff `solid_voxels` by hand. Suggested fix: stamp the
  input STL sha256 into the `.npy` sidecar and have the ACE voxel runners refuse
  (or loudly warn) when the STL on disk no longer matches.

## F9 — `tolerance_stack.py` and `production_check.py` exit 0 on `ok: false` (2026-08-08, independent verification)
- symptom: `python3 tools/tolerance_stack.py .../jobs/tol_i7_heatset.json; echo $?`
  prints `{"ok": false, ...}` then `0`. Same for `tol_i5_guide_centred`,
  `tol_i1_grip_a2`, and for `tools/production_check.py` on
  `production_check_blade_root` and `production_check_crank_pin`
  (both `"ok": false`, both `exit 0`).
- minimal repro: as above; five jobs, five `ok:false` receipts, five `exit=0`.
- expected vs actual: `DESIGN.md`'s plan table and this campaign's README
  ("`tolerance_stack.py` exits 1 on `tol_i7_heatset` **by design** … and
  `production_check.py` exits 1 on `production_check_blade_root` **by design**")
  both assume a nonzero exit. The binaries exit 0. The OPERATOR_BRIEF rule
  "ACE runners exit 0 even on failure — parse `ok`, never `$?`" evidently extends
  to the Cataloged tools too, but no doc says so.
- workaround used: parse the `ok` field of the last stdout line; never `$?`.
  (Doc-vs-binary contradiction; the campaign README needs correcting, not the tool.)

## F10 — `render_sheet.py` resolves job-relative paths against the CWD, so the README's repo-root command line cannot rebuild the renders (2026-08-08, independent verification)
- symptom: from the repo root,
  `python3 tools/render_sheet.py "$P/programs/jobs/render_palm.json"` →
  `{"ok": false, "error": "FileNotFoundError: [Errno 2] No such file or directory: 'parts/palm.stl'"}`
  and `exit 1`, for all five sheets. The job file carries `"stl": "parts/palm.stl"`,
  `"out": "renders/sheet_palm.png"`.
- minimal repro: `cd "<repo root>" && python3 tools/render_sheet.py \
  robotics_system/iso9409_wedge_flexure_gripper/programs/jobs/render_palm.json`
- expected vs actual: `kernel-api run` resolves program-relative paths against
  `--out-dir`, and `voxelize_stl.py` / the ACE runners take absolute paths, so a
  job file that carries relative paths is ambiguous. `render_sheet.py` resolves
  them against `os.getcwd()`, which is the one directory the README does NOT
  tell you to stand in. Same family as F7.
- workaround used: run the five render commands from the PART directory
  (`cd robotics_system/iso9409_wedge_flexure_gripper`), where all five exit 0 and
  reproduce every PNG byte-identically (md5 unchanged).

## F11 — two receipts meshed from byte-identical program geometry shipped different `geometry_hash` values, and nothing noticed (2026-08-08, repair pass)
- symptom: `receipts/modal_finger_v04.json` shipped
  `n_active_elements 29544`, `geometry_hash program:sha256:bbb26f8d9c9c…`
  while `receipts/buckling_slice_v04.json` shipped
  `n_active_elements 30054`, `geometry_hash program:sha256:18f4f65f689e…`.
  The `ops`/`solid`/`shape`/`voxel_mm`/`origin_mm` blocks of
  `programs/jobs/modal_finger_v04.json` and `buckling_slice_v04.json` are
  character-for-character identical (both describe the same 4 mm finger slice).
  Re-running `ace_modal_runner modal_finger_v04` reproduces buckling's numbers
  exactly: 30054 elements, `geometry_hash …18f4f65f…`, f₁ 139.5425 → 132.1303 Hz.
  The modal receipt had simply never been re-run after amendment A13 moved the
  blade length 60 → 62 mm.
- minimal repro: from the repo root, with the two job files side by side —
  `python3 -c "import json;a=json.load(open('.../modal_finger_v04.json'));b=json.load(open('.../buckling_slice_v04.json'));print(a['ops']==b['ops'], a['shape']==b['shape'])"`
  → `True True`, then compare `geometry_hash` in the two shipped receipts.
- expected vs actual: `geometry_hash` is the field an operator would reach for to
  answer "was this receipt computed on the current geometry?", and it is doing
  its job — the two hashes DIFFER, which is exactly the alarm. But nothing reads
  it. No runner refuses on a mismatch, no tool cross-checks sibling receipts that
  share a geometry block, and the hash is not compared against anything at
  regeneration time. This is the program-meshed twin of F8 (which covers the
  `.npy` voxel path): the stale-input hazard is not specific to `voxelize_stl.py`,
  it is general to every analyzer receipt that outlives its input.
- expected vs actual, second half: the hash is also not comparable ACROSS
  analyzers by construction — `ace_fea`'s is `density:sha256:…` over the density
  array, `ace_modal`/`ace_buckling`'s is `program:sha256:…` over the meshed
  program — so an operator cannot diff a voxel receipt against a program receipt
  even when both describe the same part.
- suggested fix (maintainer's call): have the runners record the hash of the
  INPUT as given (STL sha256 for `stl` jobs, canonicalised ops blob for program
  jobs) in a uniformly named field, and add a `--verify` mode that refuses when a
  receipt's recorded input hash does not match a freshly computed one.
- workaround used: `programs/selfcheck.py` now re-derives the voxel field on every
  run and fails on any mismatch against the shipped `.npy` or against the element
  count in any voxel-consuming receipt. That closes the F8 path. The general
  program-meshed path is NOT closed by the campaign — a stale program-meshed
  receipt is still only caught by re-running the analyzer, which is minutes to
  ~10 min per job and is therefore not in the self-check. Carried as an open
  limitation in `analysis/DESIGN.md` §31 (repair R14).
