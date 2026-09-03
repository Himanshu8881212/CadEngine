# Friction log — folding_deck_cleat (marine)

## F1 — Concept card names catalog parts that do not exist in the binary (2026-08-06)
- symptom: CONCEPTS.md §6 (marine) claims criterion (c) rests on "3 catalog items (4× ISO 10642 M5
  countersunk screws, ISO 2341 clevis pin, ISO 1234 split pin)". The binary's op catalogue (161 ops,
  enumerated via `{"op":"describe"}`) has NO `clevis_pin` and NO `split_pin` op. Fastener family ops
  present: `flat_head_screw`, `socket_head_cap_screw`, `button_head_screw`, `set_screw`, `hex_bolt`,
  `hex_nut`, `lock_nut`, `washer`, `spring_washer`, `dowel_pin`, `threaded_rod`, `standoff`,
  `shoulder_bolt`, `circlip_external`, `circlip_internal`.
- minimal repro: `{"ops":[{"id":"d","op":"describe"}]}` →
  `"/Users/himanshu/Work/New-LMCAD/cad engine/target/release/kernel-api" run describe.json --out-dir out`
  → grep the op list for `clevis` / `split` (absent). Receipt kept at
  `marine_system/folding_deck_cleat/receipts/stage1_catalog_probe_receipt.json` (probe program in
  `programs/stage1_catalog_probe.json`) showing the substitutes that DO build.
- expected vs actual: card (frozen after engine-fit review) promises catalog clevis-pin and split-pin
  envelopes; binary provides neither. Digest `implicit_recipes.md` §10's 48-part list is consistent
  with the binary (no clevis/split pin) — the drift is in the concept card, not the digest.
- workaround used: BOM keeps the purchased ISO 2341 B 6×40 pin and ISO 1234 1.6 split pin as real
  hardware; CAD-side clearance/NC envelopes use `dowel_pin {d:6}` (ISO 2338 table size, verified
  builds, validate green) as the pin-shank proxy plus an explicitly modelled Ø10×2 head cylinder, and
  a modelled Ø1.4 wire torus/cylinder for the split pin where a posed NC needs it. Criterion (c)
  hardware branch still stands on: hole-wizard `countersink_hole` M5 ×4, catalog `flat_head_screw`
  m5 envelopes, catalog `dowel_pin` d6 envelope, plus design-math lookups (`iso286_fit` receipt in
  the same probe). Recorded in DESIGN.md §2 and §10.

## F2 — Two `countersink_hole` cuts in one plate make the exact tessellation leak (2026-08-07)
- symptom: with the DEFAULT `export_stl` tol (0.01) a plate carrying two or more M5 countersinks
  exports `{"route":"voxel_healed","triangles":108640}` (base plate: 382 436 triangles, 19 MB) even
  though `validate` says `closed=true manifold=true valid=true`. One countersink alone exports
  `{"route":"exact","triangles":1454}`. `validate` also reports `geometric_ok:false` for a single
  countersink in some plates and `true` in others (40x40x8 box: false; 95x60x8 box + 2 csk: true),
  so that flag is noisy and does not predict the export route.
- minimal repro (verified):
  `{"ops":[{"id":"b","op":"box","min":[0,0,0],"max":[40,40,8]},
           {"id":"c","op":"countersink_hole","in":"b","at":[12,20,8],"axis":[0,0,-1],"m":5},
           {"id":"d","op":"countersink_hole","in":"c","at":[28,20,8],"axis":[0,0,-1],"m":5},
           {"id":"e","op":"export_stl","in":"d","file":"two_csk.stl"}]}`
  -> `"route":"voxel_healed"`. Same program with `"tol":0.05` -> `"route":"exact"`, 2372 triangles.
  A hand-built equivalent (drill Ø5.5 + 90 deg `cone` difference) leaks identically at tol 0.01, so
  the fault is in the cone/cylinder tangency tessellation, not in the hole wizard.
- expected vs actual: ops_core S10 says the exact adaptive tessellation is tried first and heal is
  the fallback for genuinely leaky solids; here a valid, manifold, analytic solid leaks only at the
  finer tolerance (finer tol -> MORE leakage, which is backwards).
- workaround used: every shipped `export_stl` carries `"tol": 0.05` (0.05 mm chord is far below the
  0.4 mm nozzle). Base: route `exact`, 19 346 triangles. Recorded in gen_common.EXPORT_TOL and in
  the emitted programs' `_why` comments.

## F3 — `clearance.distance` / `assert_disjoint` read 0 mm for a nested-but-disjoint body (2026-08-07)
- symptom: the raised-latched cleat pose (horn sitting inside the base channel, provably apart)
  reports `{"distance":0.0,"interfering":false,"overlap_volume":0.0}`, and
  `assert_disjoint {a:base,b:horn,min_clearance:0.02}` FAILS with
  `"surface distance 0 mm <= required clearance 0.02 mm - 'base' and 'latched' touch or interfere"`.
  The same pair proves disjoint through the tessellation-independent route:
  `union` -> `validate` -> `shells: 2`. For a body OUTSIDE the other's envelope (Ø6 rope on the horn
  groove) the same measure returns a correct `0.3807 mm`.
- minimal repro: import the two shipped STEP files, pose the horn at
  `rotate{axis:[1,1,1],240}` + `translate[-18,-7.8,11]`, then `clearance{a:base,b:horn}` (distance
  0.0, interfering false) vs `union{a,b}` + `validate` (shells 2).
- expected vs actual: ops_core S9 documents `distance` as the measured surface distance and
  `assert_disjoint` as "passes iff measured surface distance EXCEEDS min_clearance"; a 0.1-0.3 mm
  gap should read 0.1-0.3, not 0.
- workaround used: every legal-pose gate in this campaign is `union` + `assert {"shells": 2}`;
  `clearance` is still recorded, but only its `interfering` / `overlap_volume` pair is quoted as a
  number (those two agree with the union oracle in every pose we ran).

## F4 — Gauge cylinder built in place along the bore axis refuses to union; the same cylinder POSED works (2026-08-07)
- symptom: `{"op":"cylinder","base":[-18,-20,11],"axis":[0,1,0],"radius":3.0,...}` unioned with the
  base (Ø6.6 teardrop bores on the same axis, 0.284 mm clearance) fails
  `union failed validate(): closed=false manifold=false genus=17 euler_characteristic=-30 shells=2`.
  Segment counts 24 / 32 all fail. Building the identical cylinder on +Z and posing it
  (`rotate_x -90`) onto the same axis unions cleanly (`shells: 2`, `geometric_ok: true`), as does the
  catalog `dowel_pin` (also built on +Z and posed).
- minimal repro: the two variants above against `cad/folding_deck_cleat_base.step`.
- expected vs actual: the two constructions are the same solid to within float; only the facet
  meridian PHASE relative to the bore differs. DESIGN_GUIDE S7.7's hygiene note ("keep cutter side
  planes off revolve facet meridians") predicts the hazard but the failure is a hard refusal on a
  0.284 mm-clearance pair that should never touch.
- workaround used: all gauge pins in `programs/nc_interference.json` are built on +Z and posed.

## F5 — ace_fea_tet: gmsh refuses ONE element size on a valid watertight STL (2026-08-07)
- symptom: `{"ok": false, "error": "Exception: Wrong topology of boundary mesh for parametrization"}`
  from `tools/ace_fea_tet_runner.py` on `parts/folding_deck_cleat_horn.stl` (1120 triangles,
  `export_stl` route `exact`, `watertight: true`, kernel `validate` closed/manifold/genus 1/shells 1)
  at `elem_size_mm: 1.2`. Fails in seconds, before any solve.
- minimal repro: the shipped job `programs/jobs/tet_horn_lc1_teeth.json` with `elem_size_mm` set to
  1.2; `python3 tools/ace_fea_tet_runner.py programs/jobs/tet_horn_lc1_teeth.json`.
  The SAME file at `elem_size_mm` 1.5, 1.0 and 0.8 meshes and solves.
- expected vs actual: the cookbook (digests/tools_cookbook.md 1) documents the tet path as
  "GEOMETRY one of `stl` (watertight)" with no element-size admissibility condition; a watertight,
  manifold, genus-1 STL that the kernel validates is expected to mesh at any reasonable size.
  In fact one specific size hits a gmsh surface-parametrization failure. The receipt is honest
  (`ok:false` + the verbatim gmsh message) — this is a usability report, not a correctness bug.
- workaround used: element size moved 1.2 -> 1.0 and recorded in the job's `_friction_F5` key;
  the campaign notes that a tet refusal must be probed across sizes before concluding the geometry
  is un-meshable.

## F6 — `mesh_components` / `assert components:1` reports a STEP-IMPORTED body as many bodies (2026-08-07)

- symptom: a solid re-entered with `import_step` reports `components: 10` (horn) and
  `components: 12` (base) while `validate` on the SAME id reports
  `{"valid":true,"closed":true,"manifold":true,"shells":1,"genus":1}` and `volume` matches the
  exporting run to 6e-4. `mesh_components` receipt verbatim:
  `{"components": 10, "is_one_body": false, "provenance": "faceted", "tol": 0.05,
    "triangles": 758, "weld_tol": 0.001}`.
  The body is NOT severed — the same STEP file, exported from a program whose native body
  asserted `components: 1`, re-imports as a tessellation whose triangles do not weld at
  `weld_tol 0.001`. Posing it changes nothing (rotate 240 about [1,1,1], rotate 90 about Z, and
  a pure translate all still read 10), so the pose is innocent; the import tessellation is the
  cause. Note the triangle count: the shipped horn STL has 1120 triangles, the re-imported body
  tessellates to 758.

- minimal repro (must live in the directory holding `_gen/`, because `import_step.file`
  resolves against the PROGRAM file's directory):
  `marine_system/folding_deck_cleat/programs/probe_pose_components.json`
  ```json
  {"ops":[{"id":"h","op":"import_step","file":"_gen/folding_deck_cleat_horn.step"},
          {"id":"mc0","op":"mesh_components","in":"h"},
          {"id":"p240","op":"pose","in":"h","rotate":{"axis":[1,1,1],"degrees":240}},
          {"id":"mc1","op":"mesh_components","in":"p240"},
          {"id":"v1","op":"validate","in":"p240"}]}
  ```
  `"/Users/himanshu/Work/New-LMCAD/cad engine/target/release/kernel-api" run probe_pose_components.json --out-dir .`

- expected vs actual: OPERATOR_BRIEF S8 and DELIVERABLE_SPEC S2.2 make
  `{"op":"assert","components":1}` the mandatory single-body gate on EVERY part and describe
  `mesh_components` as the diagnostic for a severed body. Actual: on any `import_step` body the
  gate fires a FALSE POSITIVE — the campaign's own shipped, valid, watertight, genus-correct
  horn "fails" the connectivity gate purely because the importer's tessellation is unwelded.
  A designer who follows the doctrine literally on a posed/imported assembly gets a red gate on
  good geometry, and (worse) may learn to distrust the one gate that catches real severing.
  Interestingly a body that is imported and then BOOLEANED re-tessellates through the boolean
  path and welds correctly: this campaign's oracle negative control
  (`programs/nc_oracle_severed.json`, import_step -> difference) reads `components: 2` exactly
  as intended. So the defect is narrow: raw `import_step` output (and poses of it).

- workaround used: on imported/posed bodies this campaign gates
  `valid/closed/manifold/genus/shells` + `volume_within` and does NOT assert `components`
  (see `programs/asm_pose.json`, which carries the reason inline). The connectivity gate is kept
  where it is meaningful and trustworthy — on the natively built parts in `programs/part_base.json`
  and `programs/part_horn.json`, both of which assert `components: 1` and pass. The interference
  negative controls continue to use `union` + `assert shells == N`, which is unaffected.

## F7 — `tolerance_stack.py` writes the receipt path baked into the job, with no dry-run (2026-08-08, hostile-verification pass)
- symptom: while probing gate falsifiability I copied `programs/tol_dog_stroke_chain.json` to
  `programs/_vfy_tol.json`, changed only `closes.min_required` 0.3 -> 3.0, and ran
  `python3 tools/tolerance_stack.py _vfy_tol.json`. The tool correctly returned
  `"ok": false, "pass_worst": false` — and ALSO silently overwrote the SHIPPED receipt
  `receipts/tol_dog_stroke_chain.json` with the deliberately-broken result, because the receipt
  destination is carried inside the job body and my copy inherited it.
- minimal repro:
  `cp programs/tol_dog_stroke_chain.json /tmp/x.json` (edit `closes.min_required` to 3.0) then
  `python3 "/Users/himanshu/Work/New-LMCAD/cad engine/tools/tolerance_stack.py" /tmp/x.json`
  -> `receipts/tol_dog_stroke_chain.json` now reads `"ok": false`.
- expected vs actual: DELIVERABLE_SPEC §2.10 treats `receipts/tol_*.json` as the shipped evidence;
  a *what-if* run of a copied job should not be able to mutate that evidence. There is no
  `--dry-run`/`--no-receipt` flag and no stdout warning that a file was written.
- workaround used: restored by re-running the unmodified `tol_dog_stroke_chain.json` and
  confirming `cmp` against a pre-run snapshot of the whole campaign directory. Anyone doing
  falsifiability probes on tolerance stacks must snapshot `receipts/` first.

## F8 — solver receipts embed wall-clock `timings_s`, so they can never be byte-identical on re-run (2026-08-08, hostile-verification pass)
- symptom: re-running the README "Reproducing" chain end to end reproduced every physical
  number exactly (`max_von_mises_pa` 39353001.39556958 identical to 16 digits on
  `fea_lc1_deck`), but `cmp` still reports the receipt files as different. The only differing
  keys are `/timings_s/fea_s` (51.935 vs the shipped 47.x) and `/timings_s/sample_s`.
  `ace_buckling` additionally differs in the last 1-2 ULP of `buckling_load_factor`
  (724.4507876459232 vs 724.4507876459217) — a CG/ARPACK reduction-order effect, not a
  timing one.
- minimal repro: `python3 tools/ace_fea_runner.py <any fea job> | tail -1 > a.json` twice,
  then `cmp a.json b.json`.
- expected vs actual: DELIVERABLE_SPEC §3 "Determinism" asks that committed artifacts
  regenerate byte-identical; STLs, STEPs, PNGs, GIFs, BOM JSON/CSV and the optimizer receipt
  all did, but solver receipts structurally cannot. A verifier therefore has to diff receipts
  key-by-key with `timings_s` excluded, which is not documented anywhere.
- workaround used: flattened both JSONs and compared all leaf keys except `timings_s`;
  reported "reproduces exactly" on that basis.

## F9 — `offset_solid` is unusably slow at a fine voxel on a 30 mm part (2026-08-08)
- symptom: no error — the run simply does not finish. A three-op probe
  (`import_step` horn → `offset_solid delta 0.1/0.2/0.3, voxel 0.15` → pose/union/validate)
  was killed at **120 s** with no output. The same program with the default `voxel 0.3` was not
  attempted after that, because at 0.3 mm the re-extraction error is the same order as the
  0.3 mm clearance being measured, which defeats the purpose.
- minimal repro (from `marine_system/folding_deck_cleat/programs`):
  ```json
  {"ops":[{"id":"h","op":"import_step","file":"_gen/folding_deck_cleat_horn.step"},
          {"id":"o","op":"offset_solid","in":"h","delta":0.1,"voxel":0.15}]}
  ```
  `"/Users/himanshu/Work/New-LMCAD/cad engine/target/release/kernel-api" run probe.json --out-dir .`
  — the horn is ~30 × 18 × 20 mm, i.e. ~1.6 M cells at 0.15 mm. `describe {"name":"offset_solid"}`
  documents `voxel` with no guidance on cost, and there is no progress output or budget refusal.
- expected vs actual: `describe` presents `voxel` as a free accuracy knob ("Voxel size (mm) of the
  re-extraction lattice (default 0.3)"). Actual: cost is cubic in 1/voxel with no stated limit and
  no early refusal, so the only signal is a wall-clock timeout.
- workaround used: the shrunken/grown-proxy route to a measured fold-path clearance was abandoned
  and replaced with an **exact** one that uses only pose + union + assert — a DISPLACEMENT PROBE.
  `programs/gen_poses.py` now poses the horn 0.25 mm off the pin axis in ±X and ±Z at all five
  fold stations (union still reads `shells 2`) and 0.30 mm in −Z (union fuses to `shells 1`),
  bracketing the binding radial clearance of the legal path to [0.25, 0.30) mm. 137 ops, 3.6 s,
  zero warnings — and it is exact B-rep arithmetic rather than a voxel re-extraction, which is
  strictly better evidence than the proxy would have been.
