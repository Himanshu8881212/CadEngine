# Friction log — screw_on_exponential_horn

## F1 — Frozen concept card names a nonexistent thread standard (2026-08-06)
- symptom: CONCEPTS.md §10 (acoustics card, frozen 2026-08-06) specifies the
  throat interface as "1-3/8"-27 TPI screw-on compression-driver thread" and
  builds its negative controls around a "1-3/8"-18 gauge" as the WRONG-pitch
  imposter. Stage-1 standards research shows the industry screw-on standard is
  **1-3/8"-18 TPI (1 3/8-18 UNEF/NEF)**: Eminence official PSD:2002 sheet says
  "Mounting Thread (PSD:2002S) 1 3/8" 18 NEF ext." (pispeakers.com/PSD2002.pdf);
  Parts Express sells the driver as "…1-3/8"-18" (SKU 290-446) and the S2B-A
  adapter as "1-3/8"-18 TPI Screw-On". No 1-3/8"-27 audio product exists;
  27 TPI is the mic-stand pitch (5/8"-27), apparently conflated. Two further
  card facts failed research: driver mass "~1.55 kg / 3.4 lb" (published:
  4.7 lb/2.1 kg current sheet, 5 lb/2.27 kg Bluearan) and class reference
  "GRS PT2522-8" (a planar tweeter, not a compression driver).
- minimal repro: compare CONCEPTS.md §10 "Interfaces" bullet 1 against
  https://pispeakers.com/PSD2002.pdf and the Parts Express 290-446 title.
- expected vs actual: CONCEPTS.md freeze header promises both adversarial
  reviews were applied and the card is buildable as written; actual card pins
  its headline interface to a pitch that does not exist in the wild (its
  source-before-headline ledger did flag the driver mass as to-be-sourced).
- workaround used: campaign proceeds on the researched standard. DESIGN.md §2
  carries the erratum + unfreeze note; all thread dims re-derived at 18 TPI
  (Machining Doctor ASME B1.1 table); negative-control gauges inverted (legal
  = 1-3/8-18 UNEF-2A; imposters = 1-3/8"-27 and M35×1.5); headline creep
  moment recomputed at 2.3 kg → 0.90 N·m. CONCEPTS.md itself NOT edited
  (frozen doc, maintainer's call).

## F2 — `thread_spec` op is metric-only; card implies it covers the inch-thread engagement arithmetic (2026-08-06)
- symptom: `{"op":"describe","name":"thread_spec"}` → params: `m` (number,
  required) only. `{"op":"thread_spec","m":4}` returns metric data (pitch 0.7,
  minor_d 3.242 …). No form accepts diameter+TPI or any inch designation, so
  neither 1-3/8"-18 UNEF nor 5/8"-27 UNS can be looked up.
- minimal repro: `"/Users/himanshu/Work/New-LMCAD/cad engine/target/release/kernel-api" run desc.json --out-dir out/`
  with `{"ops":[{"id":"d1","op":"describe","name":"thread_spec"}]}`.
- expected vs actual: CONCEPTS.md §10 repair R1 prescribes
  "thread_spec/tolerance_stack engagement arithmetic" for the posed-station
  thread proof — on this card both load-bearing threads are inch series, so
  thread_spec cannot supply any of the numbers the repair names. Docs don't
  claim inch support (this is a capability gap surfaced by the card's wording,
  not a binary bug).
- workaround used: ASME B1.1 closed-form hand calcs recorded as their own
  receipts in DESIGN.md §3.3 (cross-checked against Machining Doctor's
  published 1 3/8-18 UNEF table to 0.003 mm), feeding tolerance_stack FIT/CHAIN
  jobs at stage 3. Engagement arithmetic stays fully receipted, just not
  engine-cataloged.

## F3 — custom inch threads ARE expressible via `thread_ridge`, but no exact boolean can use the result (2026-08-07)
- symptom: `{"op":"thread_ridge","major_d":35.225,"pitch":1.4111,"z0":-0.5,"length":13}`
  succeeds and binds a solid whose own `validate` is clean
  (`closed=true manifold=true genus=0 shells=1`, measures `minor_d 33.69743944089973,
  turns 9.212670965913118`). The very next op refuses:
  `invalid_geometry: op 'cut': difference failed validate(): closed=false
  manifold=false genus=1 euler_characteristic=-1 shells=1 — refusing to bind
  an invalid solid`.
- minimal repro: `"…/kernel-api" run t2.json --out-dir out/` with
  `{"ops":[{"id":"ring","op":"cylinder","base":[0,0,0],"axis":[0,0,1],"radius":22,"height":14,"segments":96},
           {"id":"borec","op":"cylinder","base":[0,0,-1],"axis":[0,0,1],"radius":16.85,"height":16,"segments":96},
           {"id":"blank","op":"difference","a":"ring","b":"borec"},
           {"id":"ridge","op":"thread_ridge","major_d":35.225,"pitch":1.4111,"z0":-0.5,"length":13},
           {"id":"cut","op":"difference","a":"blank","b":"ridge"}]}`
- expected vs actual: API.md documents the self-intersection for the EXTERNAL
  `union(body, ridge)` case and routes it to `export_threaded`. It does not say
  the INTERNAL `difference(bore, ridge)` case fails too — but it does, and
  `export_threaded` is ISO-metric-only (`m`), so for a custom `major_d`+`pitch`
  thread there is NO sanctioned route at all. `thread_ridge`'s custom-thread
  form is therefore inspect-only in practice.
- workaround used: the DESIGN_GUIDE §15 `expr_sdf` groove idiom
  (`programs/threads.py`), meshed by `implicit`. Because `implicit` meshes its
  whole domain at one voxel, and the 180 mm horn body has 1.295e5 mm2 of
  surface, this forced the part into three printed pieces so the threads live
  on small dedicated bodies (DESIGN.md §13 A1). Not a workaround we would have
  chosen: it added a joint to the headline creep path.

## F4 — `export_step` writes under `--out-dir` but `import_step` resolves against the PROGRAM's directory and refuses `..` (2026-08-07)
- symptom: a program that exports `cad/horn_body.step` and then imports
  `horn_body.step` fails
  `io: op 'rt': cannot read 'horn_body.step': No such file or directory`;
  importing `../cad/horn_body.step` fails
  `invalid_param: path '../cad/horn_body.step' must not contain '..' (it would
  escape the sandbox)`.
- minimal repro: two 4-op programs in `scratchpad/pd/`, verified both ways —
  export lands in `--out-dir/sub/t.step`, import only finds
  `<program dir>/sub/t.step`.
- expected vs actual: DELIVERABLE_SPEC §2.12 requires `export_step` then
  `import_step` with volume conserved, which reads as a single-program gate.
  The two ops resolve paths against DIFFERENT roots (digest ops_core §2 states
  both rules separately but does not flag that they make the round-trip
  impossible in one program unless `--out-dir` == the program's directory).
- workaround used: each part program exports a THIRD, byte-identical STEP to
  `programs/roundtrip/<part>.step` (legal: that path joins `--out-dir`, which
  is the part directory) and imports `roundtrip/<part>.step` (legal: relative
  to `programs/`). One program, one run, gate asserted. The `programs/roundtrip/`
  directory is a declared gate artifact, not scratch.

## F5 — `implicit` binds no solid, so no in-program `assert` can gate a voxel-route part (2026-08-07)
- symptom: `implicit` returns `volume`, `triangles`, `watertight`, `healed` in
  `measures`, but binds nothing, so `{"op":"assert","in":"<implicit id>"}` is a
  loud `missing_ref`. Every shipped threaded piece and every thread negative
  control on this part is an implicit measure.
- minimal repro: any `implicit` op followed by `assert` referencing its id.
- expected vs actual: this is documented behaviour ("binds NO solid — products
  are the file + measures", implicit_recipes §1), not a bug — but it collides
  with DELIVERABLE_SPEC §2's "a recorded-but-unchecked measure is worthless"
  for every part whose final geometry can only come from the voxel half.
  `import_mesh` does not help: it also binds nothing.
- workaround used: `programs/check_receipts.py` (stdlib) re-reads the shipped
  run reports and asserts the same numbers the generators wrote into each
  program's own `"receipts"` block — watertight, not-healed, implicit-vs-exact
  volume agreement, and a closed-form thread volume-delta band. It exits 1 on
  failure and writes `receipts/check_receipts_verdict.json`.

## F6 — `clearance` returns `overlap_volume: null` on helical meshes (2026-08-07)
- symptom: `{"op":"clearance","a":"<thread_ridge>","b":"<thread_ridge>"}` returns
  `{"coincident_fit_hazard": true, "distance": 0.0, "interfering": true,
  "overlap_volume": null, "provenance": "faceted"}`. The digest's own example
  (overlapping boxes) returns a real `overlap_volume: 27.0`, so the field is
  computable in general — it silently degrades to null here.
- minimal repro: two `thread_ridge` solids at the same z0, e.g.
  `major_d 35.225 / pitch 1.4111` and `major_d 34.889 / pitch 0.9407`, both
  `length 12`, then `clearance` between them.
- expected vs actual: DELIVERABLE_SPEC §2.11 requires must-NOT-fit claims to
  live on exact `overlap_volume`. On the geometry class where a thread campaign
  most needs it, `overlap_volume` is null and `interfering` alone cannot
  distinguish "nested and free" from "jammed".
- workaround used: the interference is measured through the implicit half
  instead — `volume(gauge) - volume(intersection(gauge, available_space))`,
  where `available_space` is our groove UNION our pilot bore. Both terms are
  `implicit` volume measures at one voxel, so the subtraction is honest.
  `programs/nc_threads.json`, `programs/nc_micthread.json`.

## F7 — `clearance` and `assert_disjoint` both report surface distance 0 for a NESTED coaxial pair that has a real 0.30 mm gap (2026-08-07)
- symptom: horn body (Ø34.200 counterbore) and collar (Ø33.600 spigot) posed
  with the spigot inside the counterbore and the faces 1.0 mm apart:
  `clearance` -> `{"distance": 0.0, "interfering": false, "overlap_volume": 0.0}`
  and `assert_disjoint {min_clearance: 0.05}` FAILS with
  `surface distance 0 mm <= required clearance 0.05`. The same ops measure the
  NON-nested pair in the same program correctly (stand bracket floated 0.1 mm
  reads `distance 0.10000038`; driver gauge reads 16.0 and 24.1).
- minimal repro: `programs/assembly.json` ops `c_register` / `c_collar_face`
  against `c_stand_face` in the same run
  (`receipts/assembly_report.json`).
- expected vs actual: `interfering: false` with `overlap_volume: 0.0` says the
  solids do not touch; `distance: 0.0` says they do. The two fields contradict
  each other, and `assert_disjoint` trusts the wrong one, so a correctly
  clearing register cannot be gated by distance.
- workaround used: the register is proved ANALYTICALLY instead —
  `measure_dimension {kind: "diameter"}` on both features
  (`d_cbore.value 34.2`, `d_spigot.value 33.6`, both `provenance: "analytic"`,
  which is a stronger receipt than a faceted distance anyway) — and
  non-contact of the three assembled pieces is proved tessellation-independently
  by `union_all` + `assert {"shells": 3}` (`g_three_bodies`, passing).

## F8 — air_topology_audit.py silently seals a wide-open bore when a slice centre lands on a vertex ring (2026-08-07)
- symptom: `air_topology_audit.py` on the SHIPPED, gated, watertight
  `parts/horn_body.stl` at the documented `voxel_mm: 1.0` returns
  `{"ok": false, "components": 13, "seed_labels": {"throat": 0, "mouth": 2},
  "connected": {"throat<->mouth": false}}` — i.e. it reports the horn's air
  column as SEVERED. The bore is Ø25.4 and completely open (the same STL
  passes at voxel 1.003).
- root cause (read from `tools/air_topology_audit.py:voxelize`): the per-slice
  parity fill builds contour segments with
  `da, db = a[2]-z, b[2]-z; if da*db < 0: pts.append(...)`, then keeps the
  triangle only `if len(pts) == 2`. A triangle with a vertex EXACTLY on the
  slice plane has `da == 0`, contributes at most one crossing and is DISCARDED.
  Exact-B-rep STL exports emit a whole VERTEX RING at the mid-height of every
  cylindrical face, so at `voxel_mm 1.0` the slice centres z = 1.5 and z = 5.5
  land exactly on the mid rings of the Ø34.2 counterbore (z 0→3) and the Ø25.4
  throat bore (z 3→8); every bore triangle on that slice is dropped, the bore
  contour vanishes and the slice reads FULLY SOLID.
- minimal repro (measured, 2026-08-07):
  ```
  python3 - <<'PY'
  import sys; sys.path.insert(0,"<repo>/tools")
  from _stl import load_stl; import numpy as np
  tris = load_stl("acoustics_system/screw_on_exponential_horn/parts/horn_body.stl")
  zmin, zmax = tris[:,:,2].min(1), tris[:,:,2].max(1)
  for z in (0.5, 1.5, 2.5):
      sel = tris[(zmin < z) & (zmax > z)]; segs = 0
      for t in sel:
          pts = [1 for i in range(3)
                 if (t[i][2]-z)*(t[(i+1)%3][2]-z) < 0]
          segs += (len(pts) == 2)
      print(z, "triangles", len(sel), "usable segments", segs)
  PY
  ```
  → `0.5 triangles 2407 usable 2407` / `1.5 triangles 1895 usable 1859` /
  `2.5 triangles 2347 usable 2347`; the 36 lost triangles are the entire
  counterbore contour, and the x-ray at y=0.4845 goes from
  `[-55, -17.092, 17.092, 55]` (4 crossings, correct) to `[-55, 55]`.
- expected vs actual: `campaign/digests/tools_cookbook.md` §3 documents the
  audit as the void-side gate with `voxel_mm: 1.0` as the worked value and says
  nothing about slice-plane degeneracy; the tool's own docstring calls the
  result an air-topology fact. Actual: at some pitches it is a tessellation
  artefact, and it fails CLOSED (reports a sealed channel that is open), which
  is the safe direction for a gate but produces a false alarm that costs a
  campaign a day if believed.
- workaround used (engine/tools untouched): `programs/gen_air_jobs.py` reads
  every vertex z from the STL and picks the voxel pitch CLOSEST TO TARGET whose
  slice centres all clear every vertex ring by > 1e-3 mm, shipping
  `_safe_pitch.min_slice_centre_to_vertex_z_mm` on the job as the proof.
  Chosen: 1.003 mm (horn, min clearance 0.00107 mm) and 0.500 mm (NC4 control
  and its fair-comparison horn twin, 0.00128 mm). Independent cross-check on
  the result: the reported air-column volume 981.8 cm³ (@1.003) / 981.47 cm³
  (@0.500) matches the closed-form bore integral
  π·r_t²·(e^{m_S·L}−1)/m_S = 986.6 cm³ to 0.5 %, which a falsely-severed column
  could not do.
- SECOND, SEPARATE LIMIT recorded here (not a bug, a resolution property): a
  septum thinner than the voxel pitch is invisible to a slice-based voxeliser.
  The NC4 control's 0.3 mm septum spans z ∈ [5.0, 5.3]; only a pitch that puts
  a slice centre INSIDE that band can see it. `gen_air_jobs.py` enforces that
  condition explicitly for the control. Any future air audit of a thin-membrane
  defect must state the pitch-vs-feature relation or the control is vacuous.

## F9 — ace_fea converges cleanly on an inclined thin wall that is a kinematic hinge chain (2026-08-08)
- symptom: `ace_fea_runner.py` on the shipped, gated, watertight P1 horn body
  (306 271 mm^3, wall 2.503-3.358 mm) at `voxel_mm 2.0` returned
  `{"ok": true, "max_displacement_m": 0.09635232542257907,
  "max_von_mises_pa": 335948030.17}` — 96 mm of deflection under a 27 N load and
  six times PLA's yield — with
  `residual_or_convergence.converged: true, target_rtol: 1e-08` and no warning
  other than the (unrelated) "selector catches 100% of active elements" note on
  the body load.
- minimal repro: `programs/job_voxelize_fea_horn_v20.json` then
  `programs/job_fea_horn_v20.json` in this campaign (grid 117 x 117 x 132,
  38 498 active elements, 220 413 DOF).
- root cause: the wall is 1.25 elements thick at this pitch, and the wall is
  inclined at 43 deg from the build axis. Parity-filled voxels of an inclined
  one-element wall form a STAIRCASE CHAIN — cubes joined one face at a time —
  which in hex8 has almost no bending stiffness. It is NOT a severance: 6-
  connected labelling gives 1 component, largest fraction 1.000
  (`receipts/fea_grid_connectivity.json`), so `mesh_components`-style checks and
  the runner's own selectors all read healthy.
- expected vs actual: `campaign/digests/analysis_honesty.md` and the cookbook
  state ace_fea's error band as "deflection -11%/-6%, peak stress +20-30%", i.e.
  bounded and signed. Actual, outside the resolution regime, the error is
  UNBOUNDED and of the opposite sign for deflection (500x too soft), and nothing
  in the receipt says so. The digests give a voxel rule for IMPLICIT/heal
  exports (">= 3 voxels across every wall/strut") but do not restate it as a
  precondition for the voxel FEA runners, where it matters just as much.
- what caught it: not the solver. Two Validated surfaces disagreeing —
  `ace_modal` on the same STL implies a lateral stiffness ~500x higher than
  27 N / 96 mm — plus a physical sanity check on the displacement magnitude.
- workaround used: the whole-shell FEA is REFUSED and its receipt is kept as
  evidence (with a `check_receipts.py` gate that ASSERTS the artefact is still
  present, so the refusal cannot quietly become a quoted result). LC1 is
  answered instead by a PLATE SUB-MODEL on the same 1.5 mm grid, where the 8 mm
  plate is 5.3 elements through thickness (`programs/gen_plate_submodel.py`),
  run at both fixity bounds, plus the closed-form couple bound from the A1
  derived model. Suggested doc fix for the maintainer: add ">= 3 elements across
  the thinnest load-bearing wall, and beware INCLINED thin walls at any pitch"
  to the ace_fea/ace_modal cards, and consider a receipt-side warning when the
  active-element count implies fewer than ~3 elements across the local
  thickness.

## F10 — `mesh_components` shatters on an `import_step` body: components 24 on a solid that is shells 1 / genus 5 (2026-08-08)
- symptom: `{"op":"assert","in":<imported>,"components":1}` on a STEP that this
  campaign itself exported fails with
  `assert_failed: op 'g_horn': assert failed: components: measured 24, expected 1`.
  The diagnosing measure agrees and is worse for the smaller parts:
  `mesh_components` → `{"components":24,"is_one_body":false,"provenance":"faceted","tol":0.05,"triangles":46373,"weld_tol":0.001}` (horn_body),
  `components 12 / triangles 2930` (throat_collar_prethread),
  `components 7 / triangles 1773` (stand_mount_prethread).
  The SAME solids read `shells 1`, `genus 5` in the same report, and their
  NATIVE (pre-export) twins assert `components == 1` green in
  `receipts/part_program_report.json` / `collar_exact_report.json` /
  `stand_exact_report.json`.
- minimal repro (from the part dir; the STEP files are this campaign's own
  round-trip exports, written by the shipped programs):
```json
{"ops":[{"id":"h","op":"import_step","file":"roundtrip/horn_body.step"},
        {"id":"mc","op":"mesh_components","in":"h"}]}
```
  `"/Users/himanshu/Work/New-LMCAD/cad engine/target/release/kernel-api" run programs/_probe_mc.json --out-dir .`
- expected vs actual: OPERATOR_BRIEF §8 and DELIVERABLE_SPEC §2.2 make
  `components:1` THE single-body gate and say `shells==1` does not prove it.
  On an imported body the implication inverts: `shells` is right and
  `components` is wrong. The tessellation an imported STEP carries is not
  welded across face boundaries at `weld_tol` 0.001, so every analytic face
  becomes its own island; the count tracks face count, not connectivity
  (43841 faces → 24 islands, 1616 → 12, 589 → 7).
- consequence if unnoticed: the inverse is the dangerous direction — a
  connectivity gate that ALWAYS fails is loud, but any campaign that decides
  "components is unreliable on imports, drop the gate" then has no severance
  gate on assembly-frame geometry at all.
- workaround used: the `components:1` gate is asserted ONLY on native solids
  (it is, in all three part programs). `programs/assembly_scene.json` gates
  its imported bodies on `shells == 1` plus an `exact_volume_within` window
  against the native closed-form volume, and says so in its `notes`; the
  existing `programs/assembly.json` was already clean (it gates
  `union_all → shells == 3`, never `components`, on imported bodies).
- SECOND FACE OF THE SAME DEFECT (same session): `export_stl` of an
  `import_step` body REFUSES —
  `invalid_geometry: op 'x_horn': mesh is not watertight even after the voxel
  heal (voxel 0.3 mm) — refusing to export a leaky mesh`. The unwelded facet
  islands are literally a leaky mesh, so the heal cannot close them. Net
  effect: a STEP this engine wrote cannot be re-exported as STL by this
  engine. Round-tripped bodies are usable for `clearance` / `assert_disjoint`
  / `volume` (all analytic) but NOT for meshing.
  Workaround: assembly-frame meshes are produced by `programs/make_scene.py`,
  a stdlib rigid-transform of the ALREADY-EXPORTED meshes (exact float64
  rotate+translate of each vertex, binary STL in and out), so the scene is a
  posed copy of the shipped/gated meshes rather than a re-mesh.

## F11 — `assembly_doc.py` `view` is a DICT, not a list, and a list crashes with an unrelated AttributeError (2026-08-08)
- symptom: `{"ok": false, "error": "AttributeError: 'list' object has no attribute 'get'"}`
  — the whole sheet fails and the message names neither the field nor the job.
- minimal repro: any valid assembly_doc job with `"view": [22, -60]`
  (elev, azim in the obvious order), run as
  `python3 tools/assembly_doc.py programs/job_assembly_doc.json`.
- expected vs actual: `campaign/digests/tools_cookbook.md` §5 lists the job
  fields as `... out_prefix*, date?, rev?, project?, doc_title?, view?,
  max_px? (1800)` and never says what shape `view` is. Every other
  view-ish field in the render family that I met is a LIST
  (`build_dir: [0,0,1]`, `explode.axis: [0,0,1]`, `size_px: [900,640]`), so a
  list is the natural guess. The tool actually does
  `view = job.get("view") or {}; elev, azim = float(view.get("elev", 18.0)),
  float(view.get("azim", -60.0))` (tools/assembly_doc.py:438-439), i.e. it
  wants `{"elev":..., "azim":...}`.
- workaround used: `programs/gen_render_jobs.py` emits
  `"view": {"elev": 22, "azim": -60}` with an inline comment saying why.
  Suggested doc fix for the maintainer: spell the shape in the cookbook
  (`view? {elev, azim}`) and/or type-check the field so the receipt names it.

## F12 — self-inflicted, recorded because it cost a run: editing a running `sh` script corrupts its parse (2026-08-08)
- symptom: `programs/run_physics.sh: line 37: _check.json: command not found`,
  emitted after the air-topology block had already produced correct receipts.
  `_check.json` is a FRAGMENT of `programs/job_prodcheck_$t.json` — a token
  sliced in half.
- root cause: `sh` reads a script incrementally by BYTE OFFSET, not as a parsed
  whole. I inserted four lines into `run_physics.sh` while that same script was
  executing, which shifted every offset after the insertion point; the shell
  resumed reading mid-token.
- expected vs actual: nothing in the engine or the tools misbehaved. This is a
  POSIX-shell property and it is not called out anywhere in the digests, so it
  is logged here as a process hazard for the next agent: **never edit a run
  script while it is running.** The failure is loud but its message points at a
  line that is not the problem, which is what makes it expensive.
- workaround used: re-ran the whole of `run_physics.sh` after the edit landed
  (it costs ~4 minutes, so nothing was lost but time). No results from the
  corrupted run were kept: the receipts written before the corruption were
  overwritten by the clean re-run.

## F13 — `air_topology_audit` receipt: `seed_labels` and `sizes_cm3` cannot be joined, and the wrong join hides on passing parts (2026-08-08)
- symptom: the obvious reading of the receipt — "the seed's component volume is
  `sizes_cm3[seed_label - 1]`" — is wrong, and it silently returns a plausible
  number. On the SHIPPED horn the seeds sit in label 2 and `sizes_cm3[1]` is
  1947.17 cm3, which IS the bore, so the join looks correct. On the NC4
  membraned control the throat seed sits in label 7 of 11 components and
  `sizes_cm3[6]` is 0.21 cm3 — an unrelated speck, not the severed throat stub.
  Two independent passes of this campaign (stage 3's `gen_analysis.py` and
  stage 4's first `gen_readme.py`) both made a version of this mistake and both
  produced numbers that read fine.
- root cause, from the tool's own source (tools/air_topology_audit.py:86):
  `"sizes_cm3": [round(float(s)*h**3/1000, 2) for s in sorted(sizes[1:], reverse=True)[:8]]`
  — a TRUNCATED, DESCENDING-SORTED list — while line 87 emits
  `"seed_labels": seeds`, which are raw label ids from the labelling pass.
  There is no mapping between them, and when `components > 8` the list does not
  even contain every component.
- expected vs actual: `campaign/digests/tools_cookbook.md` documents the
  receipt as `{"ok", "components", "sizes_cm3", "seed_labels", "connected",
  "openings_mm2"}` without saying that `sizes_cm3` is sorted+truncated or that
  `seed_labels` are not indices into it. Two adjacent keys that look joinable
  and are not.
- workaround used: nothing in this campaign attributes a volume to a seed. The
  air column is identified by AGREEMENT WITH THE CLOSED-FORM BORE INTEGRAL —
  1947.17 cm3 measured vs 1946.9 cm3 integrated, gated at 2 % by
  `check_receipts.py`'s `A6:air_volume_matches_closed_form` — and the NC4 row
  in both shipped documents quotes LABELS and COMPONENT COUNTS
  (throat label 7 vs mouth label 2; components 10 -> 11) instead of volumes.
  Suggested doc fix for the maintainer: either emit
  `seed_sizes_cm3: {throat: ..., mouth: ...}` directly, or rename the field to
  `largest_sizes_cm3` so the truncation and the sort are visible in the name.

## F14 — `ace_modal_runner.py` is not bit-reproducible: eigenvalues move in the last ~10 digits between identical runs (2026-08-08, found by independent verification)
- symptom: re-running `sh programs/run_physics.sh` with no source change
  reproduces every `ace_fea`, `ace_thermal`, `tolerance_stack`, `joint_check`,
  `air_topology_audit`, `production_check` and `param_optimize` receipt
  byte-for-byte (only `timings_s` moves), but the three `ace_modal` receipts
  change in the last digits of every eigenvalue and of `effective_mass_kg`:
  `modal_horn_v20.first_mode_hz` 88.54995720290233 -> 88.54995720267024
  (2.6e-12 relative), `modal_horn_v20_lug` 65.04261773951878 ->
  65.04261773938353, `modal_horn_v30` 85.01827485786734 -> 85.01827485787805.
- minimal repro:
  `python3 "$T/ace_modal_runner.py" programs/job_modal_horn_v20.json | tail -1`
  twice, diff the two last stdout lines.
- expected vs actual: DELIVERABLE_SPEC §3 "Determinism" asks committed
  artefacts to regenerate byte-identical; the geometry half of this engine
  does exactly that (all three `parts/*.stl`, including the 46.0 MB and
  50.5 MB voxel meshes, and all 13 PNGs, `cmp`-identical across a full
  rebuild). The modal solver alone is only reproducible to ~1e-11, presumably
  a threaded/Lanczos reduction whose summation order is not pinned.
- workaround used: none needed for this campaign — every shipped modal number
  is quoted to 2 decimal places (88.55 / 65.04 / 85.02 Hz) and the
  `check_receipts.py` gate `A3:*_below_half_fc` compares against 240.737 Hz,
  so the drift is 10 orders of magnitude below anything asserted. Recorded so
  no future campaign writes a `cmp`-based gate on a modal receipt.

## F15 — `assert` accepts only topology/volume, so `support_report` / `wall_thickness` / `bounding_box` / `mass_properties` cannot be gated in-program (2026-08-08)
- symptom: `crates/kernel-api/src/interp.rs` `OpKind::Assert` takes exactly
  `volume_within / exact_volume_within / genus / shells / components / closed /
  manifold / valid`, and `discover.rs` lists no others. Adding
  `{"op":"assert","in":"tcut3","steep_area":0.0}` produces
  `warnings: ["unknown param 'steep_area' — 'assert' does not accept it..."]`
  and the assert then fires with *no checks*, i.e.
  `op 'g': assert has no checks — give at least one of volume_within /
  exact_volume_within / genus / shells / components / closed / manifold /
  valid`. The audit ops themselves (`support_report`, `wall_thickness`,
  `bounding_box`, `mass_properties`) take no expectation parameter either.
- minimal repro: any program with
  `{"op":"support_report","in":"<solid>","build_dir":[0,0,1],"overhang_deg":45}`
  followed by an attempt to assert its `steep_area`.
- expected vs actual: DELIVERABLE_SPEC §2 opens "A recorded-but-unchecked
  measure is worthless" and §2.5/§2.6/§2.7 make `steep_area == 0.0`, the wall
  percentiles and `fits_within` mandatory GATES. The engine can produce all
  three measures but cannot assert any of them, so a program that satisfies
  §2.5 in letter (the measure is recorded) can still be a program in which the
  gate cannot fail. This campaign shipped exactly that state for four stages —
  `part_program.json` has four asserts and none of them touch the support,
  wall or bbox measures — and it took an external verifier to notice.
- workaround used: the measures are gated OUT of program, in
  `programs/check_receipts.py::exact_body_measures()`, reading the same report
  files (`P1/P2/P3:support_free`, `:wall_p05_above_1p6`,
  `:thin_area_not_growing`, `:fits_the_bed`, `:mass_properties_present`).
  Falsification-tested by mutating each measure in memory; every gate flips.
  A `min`/`max`/`equals` expectation on the audit ops themselves — or letting
  `assert` read a named measure off a prior op — would let this live where it
  belongs, next to the geometry that produces it.

## F16 — `creep_allowable_mpa` is a two-row step lookup with no cell between 23 °C and 55 °C, and nothing in the gate surface makes the temperature visible (2026-08-08)
- symptom: `crates/kernel-model/src/lib.rs` `creep_allowable_mpa(temp_c, hours)`
  computes `row = if temp_c <= CREEP_TEMPS_C[0] { 0 } else { 1 }` over
  `CREEP_TEMPS_C = [23.0, 55.0]`. `creep_allowable_mpa(23.0, 8760)` = **2.5**;
  `creep_allowable_mpa(23.000001, 8760)` = **0.5**. A five-fold cliff at a
  temperature nobody controls a room to.
- minimal repro: the two calls above (reproduced from
  `tools/materials/pla.json` by `programs/creep.py` in this campaign).
- expected vs actual: the docstring says the lookup "rounds the temperature UP
  to the next tabulated tier … so an in-between request never reads a rosier
  cell", which is correct and deliberate. The friction is that a design whose
  own declared ambient is 25 °C reads the **55 °C** row, and nothing in the
  receipt surface says so: `creep_allowable_mpa` is a bare function, no tool
  receipt records which cell was used, and a campaign that writes
  "gated against creep_allowable_mpa(23 C, 1 year)" in prose looks fully
  compliant with DELIVERABLE_SPEC §2.8 while having silently designed to a
  temperature it does not hold. This campaign made exactly that error and
  shipped it through four stages.
- workaround used: `programs/creep.py` re-derives the lookup from
  `tools/materials/pla.json` and is the single source for both document
  generators and `check_receipts.py`; every sustained table now ships BOTH
  cells; the >23 °C column is published as a condition limit and two gates
  ASSERT its SF stays below 1.0 so it cannot become a margin. What would fix
  this upstream is a receipt: have the creep lookup (or `production_check.py`)
  emit `{temp_c_requested, row_used_c, hours_requested, col_used, sigma_mpa}`
  so the temperature a margin was read at is a gateable number rather than a
  sentence in a README.
