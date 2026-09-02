# Friction log — rotor_runout_gauge_bridge

## F1 — OPERATOR_BRIEF describe example omits required `id` field (2026-08-06)
- symptom: running the brief's §9 pointer-table form `{"op":"describe","name":"iso286_fit"}` fails:
  `"kind": "invalid_param", "message": "op #0: missing required string field 'id'"` (exit report ok:false)
- minimal repro: `{"ops":[{"op":"describe","name":"iso286_fit"}]}` →
  `"/Users/himanshu/Work/New-LMCAD/cad engine/target/release/kernel-api" run prog.json --out-dir out/`
- expected vs actual: OPERATOR_BRIEF §9 ("op param truth at runtime | `{"op":"describe","name":"<op>"}`") and §1 line 7 (`{"op":"describe"}` is authoritative) show describe without an `id`; the binary requires `id` on every op, including describe.
- workaround used: added an id (`{"id":"d1","op":"describe","name":"iso286_fit"}`) — works. Cost: one retry. Doc-side fix suggestion: show the id in the brief's describe examples.

## F2 — `union_all` of two cylinders + a tangent waist box never returns (2026-08-07)
- symptom: no error, no report — the process ran past 110 s and was killed twice. The op was an
  obround stud-slot cutter: `union_all` of two Ø15.5 cylinders (segments 64) and an `extrude` of
  the rectangular waist whose two long edges are EXACTLY tangent to both cylinders. Every earlier
  op in the same program completes in < 2 s.
- minimal repro (in-campaign): the pre-fix `programs/gen_bridge.py` step 6 —
  `{"op":"cylinder","base":[28.797,39.642,-1],"axis":[0,0,1],"radius":7.75,"height":10,"segments":64}`
  and its twin at `[35.855,49.350,...]`, unioned with
  `{"op":"extrude","profile":[[35.067,35.087],[42.125,44.795],[29.585,53.905],[22.527,44.197]],"height":10}`
  → `kernel-api run prog.json --out-dir out/`
- expected vs actual: `ops_core.md` §6 warns about coplanar overlap and about keeping cutter side
  planes off revolve facet meridians, but says nothing about plane/cylinder TANGENCY. Expected a
  refusal or a slow-but-finite union; got an apparent hang (no exit inside 110 s).
- workaround used: replaced the union with ONE circumscribed 26-gon prism (`geom.slot_polygon`),
  flats tangent at r = 7.75 so the slot is never undersize, min chord 2.04 mm. Runs in < 1 s and
  the stadium area is exact by shoelace. Cost: ~2 killed runs plus the rewrite.

## F3 — `import_step` resolves against the PROGRAM directory, not `--out-dir` (2026-08-07)
- symptom: `export_step {"file":"rotor_runout_gauge_bridge.step"}` writes to `<out-dir>/`, then
  `import_step {"file":"rotor_runout_gauge_bridge.step"}` in the SAME program fails:
  `{"kind":"io","message":"op 'step_back': cannot read 'programs/rotor_runout_gauge_bridge.step':
  No such file or directory (os error 2)"}`
- minimal repro: any program under `programs/` that exports a STEP and immediately re-imports it,
  run with `--out-dir` pointing anywhere other than `programs/`.
- expected vs actual: `ops_core.md` §10 documents the export path rule and §11 documents the
  `load_part` rule, but the STEP round-trip (DELIVERABLE_SPEC §2.12) is a same-program pattern and
  the two path rules are incompatible for it. Not a bug so much as an undocumented sharp edge.
- workaround used: export the STEP TWICE — once to the shipped `cad/…step` and once to
  `programs/_rt_<piece>.step` — then `import_step "_rt_<piece>.step"`, running with
  `--out-dir <part dir>`. The `_rt_*.step` probes are kept (they are also what
  `poses_program.json` imports) and are named in the README.

## F4 — `validate.geometric_ok` flips false on the second of two mirror-image cuts (2026-08-07)
- symptom: `receipts/carriage_gate_report.json` op `g_validate` reports
  `{"closed":true,"manifold":true,"valid":true,"shells":1,"genus":4,"geometric_ok":false}`.
  Bisecting every boolean: `c_m5h_r` (M5 clearance cylinder at x = +13.5) → `geometric_ok true`;
  the mirror-image `c_m5h_l` (same op, x = −13.5) → `geometric_ok false`. Exit code stays 0, the
  op does not fail, and no `warnings` are emitted.
- minimal repro: `programs/carriage_program.json`, ops `c_m5h_r` / `c_m5h_l`; bisect with
  `{"op":"validate"}` after each boolean.
- expected vs actual: two geometrically mirror-image differences on a mirror-symmetric solid
  should give identical validity receipts. `geometric_ok` is not documented in `ops_core.md` §9
  (the `validate` row lists closed/manifold/euler/genus/shells/valid only), so its contract and
  severity are unknown to a campaign author. Tried segments 48 and 64 on the cutter — no change.
- workaround used: none needed to ship — every gate that IS documented passes (`valid`, `closed`,
  `manifold`, `shells 1`, `components 1`, exact-volume window to 1e-8 %, `export_stl` route
  `exact` + `watertight true`, STEP round-trip 0.007 %). The flag is recorded verbatim in
  `analysis/DESIGN.md` §16 and in the README rather than being dropped. Requested: document
  `geometric_ok` (what it tests, whether it is shippable) and check the mirror asymmetry.

## F5 — `assert_disjoint` / `clearance` report `distance: 0.0` for a pin threading a hole (2026-08-07)
- symptom: an M14 stud (Ø14 cylinder) posed through the Ø15.5 bridge stud slot — a true 0.75 mm
  radial clearance — measures `{"distance":0.0,"interfering":false,"overlap_volume":0.0}`, and
  `assert_disjoint {min_clearance:0.05}` therefore FAILS with
  `"surface distance 0 mm ≤ required clearance 0.05 mm — 'bridge' and 'studs100_0' touch or
  interfere"`. Reproduced for all four claimed PCDs (100 / 108 / 114.3 / 120) and for a Ø8.000
  stem in a Ø8.10 bore. The `interfering` and `overlap_volume` fields are CORRECT; only
  `distance` (and hence `assert_disjoint`) is wrong. The same pair of ops works correctly when the
  two bodies are side-by-side rather than one-through-the-other (carriage seated in the rail
  measures `distance: 0.100`).
- minimal repro: `import_step` the bridge, add
  `{"op":"cylinder","base":[29.389,40.451,-2],"axis":[0,0,1],"radius":7.0,"height":12,"segments":48}`,
  then `{"op":"clearance","a":"bridge","b":"stud"}`.
- expected vs actual: `ops_core.md` §9 documents `clearance.distance` as "the interference measure
  that does NOT fail on overlap"; nothing says the distance degenerates for enclosed/threading
  poses. `assert_disjoint` is advertised as the positive non-overlap proof.
- workaround used: every legal-twin gate in `programs/poses_program.json` is now
  `union_all` + `{"assert":{"shells":N}}` (the tessellation-independent proof `ops_core.md` §6
  recommends), with the `clearance` receipt kept alongside for its `interfering` /
  `overlap_volume` fields. Cost: one failed run plus an 8-pose diagnostic.

## F6 — ace_fea `slider` fixture silently degrades to "no fixture" when its selector catches only inactive voxels (2026-08-07)
- symptom: a `{"kind":"slider","dof_constrained":["uz"],"region_selector":{"type":"bbox",...,"max_mm":[43,40,0.3]}}`
  returned `{"kind":"slider","nodes_or_elements":0}` and the note
  `fixture[2] (slider): selector matched no active nodes.` The run still exited
  `ok:true` and produced a receipt identical to the no-fixture case — a
  0.036793 mm answer that LOOKS like a converged result for a model that
  quietly lost 4 of its 6 boundary conditions.
- minimal repro: any bbox whose z band only covers the half-voxel of air below a
  part sitting on z = 0. With `origin_mm z = -1.0`, `voxel_mm 1.5`, element
  centers are at z = -0.25 (air) and 1.25 (solid); a bbox `max_mm z = 0.3` plus
  the resolver's 0.5-voxel tolerance reaches 1.05 and so catches ONLY the air
  layer. Command: `python3 tools/ace_fea_runner.py jobs/fea_L1_c.json`.
- expected vs actual: `campaign/digests/tools_cookbook.md` §Selectors documents
  "A selector catching 0 nodes errors" — that is true for LOADS but not for
  FIXTURES: a 0-node fixture is a note, not an error, and `ok` stays true.
- workaround used: the campaign gates the receipt itself —
  `programs/opt_eval.py` refuses any eval whose fixture/load node counts sum to
  zero, and every shipped A1/A5 receipt has its per-fixture
  `nodes_or_elements` list quoted in ANALYSIS.md. The bbox z band was moved to
  0.9 mm so exactly one SOLID layer is caught on both pinned frames.

## F7 — `assert` op reports `exact_volume_within` results under `measures.exact_volume`, not under the assert key (2026-08-07)
- symptom: reading a candidate's volume back out of a report with
  `op.get("exact_volume")` returned `None`, and the optimizer evaluator crashed
  with `TypeError: unsupported operand type(s) for *: 'NoneType' and 'float'`.
  The actual report entry is
  `{"id":"g_volume","ok":true,"measures":{"exact_volume":190074.7526918904}}`.
- minimal repro: `{"op":"assert","in":X,"exact_volume_within":{"target":T,"percent":P}}`
  then read the report entry.
- expected vs actual: `campaign/digests/ops_core.md` describes asserts as
  binding nothing and being pass/fail; it does not say that an
  `exact_volume_within` assert ALSO publishes the measured volume, nor under
  which key. Useful behaviour, undocumented — cost one debug cycle.
- workaround used: all campaign readers go through `measures` (see
  `programs/opt_eval.py::measures`).

## F8 — a coarse in-loop voxel silently QUANTIZES an optimizer parameter, and nothing in the receipt says so (2026-08-07)
- symptom: driving `param_optimize.py` through a command evaluator that
  voxelizes the candidate geometry, two candidates that differ in a real
  dimension returned a BIT-IDENTICAL objective:
  `eval 1: {'SPAR_H': 44.0, 'EAR_T': 8.0} -> score 0.00865668`
  `eval 3: {'SPAR_H': 44.0, 'EAR_T': 8.4} -> score 0.00865668`
  At `voxel_mm 4.0` the 0.4 mm change in foot thickness does not move a single
  voxel, so the FEA sees the same body. Nelder-Mead reads that as a flat
  direction and can stall on it.
- minimal repro: any `voxelize_stl.py` -> `ace_fea_runner.py` chain where a
  parameter's step is smaller than `voxel_mm`. `tools/voxelize_stl.py` reports
  `solid_voxels`, which DOES change for larger steps, but neither it nor the FEA
  receipt carries a "geometry unchanged from the previous call" signal, and
  `param_optimize`'s history shows only the score.
- expected vs actual: `campaign/digests/tools_cookbook.md` warns that struts
  under ~4 cells should be quoted as approximate; it does not warn that an
  OPTIMIZER's search space is silently discretized to the voxel pitch, which is
  a different and more dangerous failure — the run looks converged.
- workaround used: the campaign states the in-loop voxel and its measured bias
  against the pinned frame in the job file itself
  (`programs/jobs/param_optimize.json._in_loop_voxel_note`), keeps the parameter
  ranges wide enough that the coarse grid can resolve the ends (EAR_T 8..16 at
  4.0 mm voxel), and re-verifies the selected optimum on the pinned 1.5 mm frame
  plus the full Stage-2 gate suite before anything is shipped.

## F9 — ace_fea_tet is unaffordable at the mesh size this part needs, and refuses at the size that is affordable (2026-08-07)
- symptom: at `elem_size_mm 3.5` on a thin-walled clamp (2.2 mm flexure arms,
  0.6 mm slit, 0.30 mm land) the runner returns
  `{"ok": false, "error": "AssertionError: body-fitted mesh has a non-positive corner Jacobian (min -2.718e-03 mm^3) — inverted/degenerate element; ref-mesh"}`.
  At `elem_size_mm 1.6` it meshes cleanly (63 965 tet10, 112 304 nodes,
  `volume_rel_err 7.7e-4`) but the solve is ~337 k DOF on the iterative path and
  did not complete in ~45 min of wall clock on a machine at load average 40-66
  (six agents, eight cores). `elem_size_mm 2.8` was killed mid-run before it
  reported.
- minimal repro: `python3 tools/ace_fea_tet_runner.py programs/jobs/fea_tet_L2.json`
  with `elem_size_mm` 3.5 on `parts/rotor_runout_gauge_carriage.stl`.
- expected vs actual: `campaign/digests/tools_cookbook.md` documents the tet path
  as the curved-geometry twin that "resolves fillet stress concentrations", and
  documents `direct_max_dof` (250 000) — but gives no guidance that a part with
  sub-3 mm features has no usable elem_size window: too coarse inverts the mesh,
  fine enough exceeds the direct-solver threshold and falls onto CG. There is no
  documented cost model to plan around.
- workaround used: the refusal is SHIPPED as the A2 result
  (`receipts/fea_tet_L2.json`, `receipts/tet_L2_scaled.json`), and the governing
  clamp stress stays the closed-form bracket 1.535-4.786 MPa vs the 5.0 MPa
  creep cell (`receipts/closure_arithmetic.json`). DESIGN §18.1 asked the tet to
  arbitrate between the two bounds; it could not, so the CONSERVATIVE bound
  governs and the arbitration stays open. Nothing is claimed from a run that did
  not happen.

## F10 — the mandatory `components:1` gate does NOT survive `import_step` (2026-08-07)
- symptom: the carriage asserts `components 1` on the natively-built body
  (`receipts/carriage_gate_report.json` op `g_components_measure`:
  `{"components": 1, "is_one_body": true, "triangles": 8260, "tol": 0.05,
  "weld_tol": 0.001}`). Export it to STEP and import it straight back and the
  SAME solid reports
  `op 'posed_components': assert failed: components: measured 8, expected 1`
  while `validate` on the imported body still reads
  `{"valid": true, "closed": true, "manifold": true, "shells": 1, "genus": 4}`
  and `volume 42874.917` vs the native `exact_volume 42871.992` (+0.0068 %).
- minimal repro (2 ops + a measure sweep, `--out-dir` anywhere):
  `{"ops":[{"id":"c","op":"import_step","file":"_rt_carriage.step"},
           {"id":"m","op":"mesh_components","in":"c","tol":0.05,"weld_tol":0.001}]}`
  -> `components 8, triangles 1464`. Repeated at `tol` 0.02 / 0.05 / 0.1 and
  `weld_tol` 0.001 / 0.01 / 0.05: **components stays 8 and triangles stays 1464
  at every combination** — neither knob moves the reading, so this is not a weld
  tolerance the caller can tune out.
- expected vs actual: `OPERATOR_BRIEF` §8 makes `components:1` the first-class
  gate for the hardest silent failure ("a part severed into floating lumps
  passes validity/watertight/volume and shells==1"), and `DELIVERABLE_SPEC` §2.2
  requires it on every part body. Nothing says the measure is faceter-dependent.
  The imported body is tessellated at 1464 triangles vs the native 8260 — 5.6x
  coarser — and the resulting per-face triangulations do not share vertices
  along shared edges, so vertex welding cannot connect them at any tolerance and
  the mesh splits into 8 face groups. The measure is reading the FACETER, not
  the solid: an honest gate returns a false FAIL on a provably single body.
- workaround used: build render/pose inputs NATIVELY instead of re-importing the
  round-trip STEP — `programs/gen_renders.py` replays
  `programs/carriage_program.json`'s ops up to the `carriage` body, translates
  that, and asserts `components 1` on the translated native solid (passes).
  `programs/gen_poses.py` is unaffected because it gates imported bodies on
  `shells` and on exact `overlap_volume`, never on `components`.
  Consequence for anyone reading the campaign: `components:1` is only meaningful
  on a natively-built body in the same program; on an imported one it is
  unusable in its current form, and a campaign that gates only after a STEP
  round trip would be gating on nothing.

## F11 — tools/sweep_check.py cannot sweep any assembly whose bodies come from `import_step` (2026-08-08)
- symptom: `sweep_check.py` substitutes `"$t"` into a template and runs it through
  `param_optimize.call_engine`, which writes the program to a **tempfile**
  (`tools/param_optimize.py` ~line 83: `tempfile.NamedTemporaryFile(suffix=".json")`).
  `import_step.file` resolves against the PROGRAM FILE's directory
  (OPERATOR_BRIEF §3), so in the tempfile it resolves under `/var/folders/...`
  and the import fails. The obvious escape — an absolute path — is rejected too:
  `{"ops":[{"id":"b","op":"import_step","file":"/abs/path/_rt_bridge.step"}]}`
  run through `kernel-api run` returns `ok:false` on op `b`.
- minimal repro: any `sweep_check.py` job whose `template.ops` contain an
  `import_step`, e.g. a two-body clearance sweep of a carriage along a rail.
- expected vs actual: `campaign/digests/tools_cookbook.md` presents sweep_check as
  the generic tool for "insertion sweeps" and "motion sweeps" over a normal LMCAD
  work order, and DELIVERABLE_SPEC §2.11 leans on sweeps for free-motion claims.
  Nothing warns that the one thing multi-body assembly sweeps need — loading
  previously-built bodies — is exactly what the tempfile wire forbids. The two
  documented file-resolution rules (program-relative, no absolutes) are
  individually correct and jointly make the tool unusable for this class.
- workaround used: `programs/gen_sweep.py` writes ONE program containing all 24
  stations (translate + union/assert shells==2 + clearance per station) and runs
  it with `kernel-api run` from `programs/`, where `_rt_*.step` resolves.
  Cheaper too — one engine run of 3.3 s instead of 24. Receipt:
  `receipts/sweep_travel.json` (24/24 stations disjoint, min distance
  0.100002 mm, warnings 0).

## F12 — `ace_modal` at the frame every other analysis uses is unaffordable, with no cost model to plan around (2026-08-08)
- symptom: `programs/jobs/modal_L3.json` on the SAME pinned 1.5 mm frame that
  every `ace_fea` row in this campaign uses (54 372 active elements, 209 601 DOF,
  6 modes) printed one stderr line —
  `loaded density grid (69, 134, 31) from .../fea_out/bridge_c.npy` — and then
  produced nothing for **90 minutes of wall clock** while accumulating only
  **7 min of CPU** (`ps`: 48 % of one core, 3.0 GB RSS) at load average 34-81 on
  8 cores. `receipts/modal_L3.json` stayed a zero-byte file across two stages.
  The same job at 2.5 mm (12 494 elements, 54 891 DOF, 53 435 free) finished in
  **132.9 s** of `modal_s`.
- minimal repro: `python3 tools/ace_modal_runner.py programs/jobs/modal_L3.json`
  with `voxel_mm 1.5`, `n_modes 6`, 6 fixtures, on this part's `bridge_c.npy`.
- expected vs actual: the same 1.5 mm frame solves `ace_fea` in ~170 s, and
  `campaign/digests/analysis_honesty.md` documents ace_modal's accuracy band
  (+1-3 %) and its refusal rules but gives no cost guidance at all. The receipt
  itself shows why: the path is
  `eigsh_shift_invert(sigma=0.000e+00)`, i.e. a shift-invert factorisation of the
  full stiffness matrix — a very different cost curve from the CG path ace_fea
  uses, and a ~4x DOF increase moves it from 2 min to "did not finish". Nothing
  warns that the frame you pin for FEA may be unusable for modal, which is
  exactly the trap when the doctrine tells you to pin ONE frame and reuse it.
- workaround used: `programs/gen_modal_coarse.py` re-poses the identical fixture
  set on a 2.5 mm twin frame (`programs/jobs/modal_L3_c25.json`). A4 lands as a
  real measurement — first mode **171.47 Hz** bare, **96.06 Hz** after the
  closed-form tip-mass correction, versus the ≥ 30 Hz requirement — and
  `analysis/ANALYSIS.md` §4 names the coarse receipt as its source, states the
  extra unquantified coarse-mesh bias, and never quotes it as the fine result.
  The fine job is left in the tree so it can be run on an idle machine.

## F13 — `support_report.overhang_deg` polarity is undocumented and inverted from intuition (2026-08-08, hostile verification)
- symptom: `describe {"name":"support_report"}` returns `overhang_deg` with
  `"doc": ""` — no units, no sign convention, no statement of which direction is
  stricter. Determined empirically by bisection on the shipped bridge:
  `overhang_deg` 30/40/44 → `steep_area 2720.947 mm²`, `support_free false`;
  46/49/50/60/80 → `steep_area 0.0`, `support_free true`. So a LARGER
  `overhang_deg` is MORE permissive, and the whole gate is a cliff at the
  modelled 45° gable angle.
- minimal repro:
  `{"ops":[{"id":"s","op":"support_report","in":<body>,"build_dir":[0,0,1],"overhang_deg":44.0}]}`
  vs the same with `60.0`, run through `kernel-api run`.
- expected vs actual: DELIVERABLE_SPEC §2.5 tells a campaign to gate
  `steep_area == 0.0` and warns "never set `overhang_deg` to a modelled face
  angle", which reads as if a *second, larger* value is a *stricter* reading.
  It is the opposite. This campaign's own `part_program.json` op
  `g_support_bridges` carries `_why: "second, stricter reading so the 45 deg
  gables are resolved on both sides of 45"` — 60° is looser than 50°, and both
  sit on the permissive side of the 45° cliff, so the second reading adds
  nothing. An empty `doc` string on a gate parameter is how that happens.
- workaround used: none needed for verification; reported as a campaign finding
  (the 50°/60° pair is not two-sided evidence). A one-line `doc` on
  `overhang_deg` stating the polarity would have prevented it.

## F14 — `ace_fea` manifest `validation.direction` does not transfer between geometries, and nothing warns you (2026-08-08, repair pass)
- symptom: `tools/manifests/ace_fea.manifest.json` states, as an unqualified
  property of the analyzer, `"direction": "under-predicts peak/tip response;
  hex8 is stiff; error converges from below toward the analytic value under
  refinement."` with `error_band {coarse_voxel_1.0: "-20%..0", fine_voxel_0.5:
  "-10%..0"}`. Read as written, that licenses deflating a deflection gate by the
  band. On this campaign's part it is **false for deflection**. Identical job,
  identical fixtures/loads/origin/material, resolution the only change:

      voxel 1.5 mm  ->  tip_displacement_m 2.658970508201094e-05
      voxel 1.0 mm  ->  tip_displacement_m 2.471493987588807e-05   (-7.05 %)

  Refinement LOWERED the deflection: it converges from ABOVE. The stress caveat
  in the same manifest DID transfer (max_von_mises_pa 993840.96 -> 1221594.51,
  i.e. the coarse mesh reads 18.6 % low, versus the caveat's "roughly 20%").
- root cause (measured, not guessed): the manifest's pin is
  `tools/ace_fea_validation.py`, whose ground truth is an 8 x 8 mm cantilever at
  voxel 1.0/0.5 — a pure-bending specimen at 8 voxels across the section. This
  part is 34 x 44 mm at voxel 1.5 (23 x 29 voxels across the section) and 81 %
  of its compliance is rigid rotation of two 8 mm foot plates, not spar bending.
  The two error mechanisms (shear locking in bending vs boundary resolution of a
  thin bearing plate) have OPPOSITE signs, and the manifest exposes only one.
- minimal repro:
      cd "<repo>/automotive_system/rotor_runout_gauge_bridge"
      python3 "<repo>/tools/voxelize_stl.py"   programs/jobs/vox_bridge_c.json
      python3 "<repo>/tools/voxelize_stl.py"   programs/jobs/vox_bridge_f.json
      python3 "<repo>/tools/ace_fea_runner.py" programs/jobs/fea_L1_c.json | tail -1
      python3 "<repo>/tools/ace_fea_runner.py" programs/jobs/fea_L1_f.json | tail -1
  Compare `tip_displacement_m` and `max_von_mises_pa`. ~10 min for the fine run
  (681 303 DOF, Jacobi-CG, rtol 1e-8, converged).
- expected vs actual: the manifest promises a one-sided band ("-20%..0") on
  "peak/tip response"; the binary delivers +7.05 % on tip response and -18.6 %
  on peak stress for the same model. Also note the campaign runs at voxel 1.5,
  which is COARSER than the manifest's coarsest pinned point (1.0), so even the
  magnitude was an extrapolation with no stated support.
- workaround used: solved the refinement in-campaign and shipped it as
  `receipts/fea_L1_f_run.json`; ANALYSIS.md gained a §1.2a that reports the
  measured trend and explicitly withdraws "converges from below" for deflection
  on this geometry. The 0.89 gate deflation was KEPT (it is conservative in
  either direction) but is no longer justified as a correction for an under-read.
  No Richardson extrapolation is claimed from two grids.
- suggested manifest change (maintainer's call, not made here): qualify
  `validation.direction` with the specimen it was pinned on, and say that the
  sign is a property of the dominant compliance mechanism, not of hex8.
