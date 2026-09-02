# FRICTION — prosthetic_wrist_quick_disconnect

Engine/tool/doc issues hit while driving LMCAD. Append-only; no entry is ever deleted.
Engine and tools source were NOT touched — every workaround lives inside
`biomedical_system/prosthetic_wrist_quick_disconnect/`.

---

## F1 — `wall_thickness` reports a facet-dependent `thin_area` at a convex feature crossing (2026-08-07)

- symptom: on the receptacle, the detent nose (a small cylinder unioned onto the
  counterbore wall so that it protrudes into the bore) makes `wall_thickness
  {flag_below: 1.6}` report sub-1.6 mm material that does not physically exist. The
  reading is not stable under geometrically equivalent changes. A 4x4 sweep of nose
  radius R and cylinder `segments`, with the nose TIP radius held fixed at 11.30 mm in
  every case (so the mechanism is identical), gave:

  | R (mm) | segments | `thin_area` (mm^2) | `min_thickness` (mm) |
  |---|---|---|---|
  | 1.20 | 32 | 1.5740 | 0.2445 |
  | 1.30 | 32 | **0.1918** | 0.1228 |
  | 1.30 | 64 | 0.8090 | 0.2123 |
  | 1.35 | 64 | 1.0938 | 0.0029 |
  | 1.60 | 48 | **0.0000** | 1.6822 |

  The material behind the nose is the 1.70 mm detent finger plus the nose itself
  (>= 2.60 mm total); nothing in that region is 0.003 mm thick. The two circles cross
  at ~86 degrees, so this is not a tangency case.
- minimal repro: `programs/part_receptacle.json` as shipped, ops `s6` -> `nose1` ->
  `nose2` -> `receptacle` -> `g_wall`; re-run with `nose1.radius` 1.20/1.30/1.35/1.60
  and `segments` 32/48/64/96.
  `"/Users/himanshu/Work/New-LMCAD/cad engine/target/release/kernel-api" run programs/part_receptacle.json --out-dir .`
- expected vs actual: OPERATOR_BRIEF section 7 says to judge `thin_area` + `p05_thickness`
  and treat `min_thickness` as "oblique-corner-ray noise". Actual: `thin_area` — the
  measure the brief tells you to JUDGE ON — is itself facet-noise at a convex feature
  intersection, swinging 0.00 -> 1.64 mm^2 across geometrically equivalent parts.
- workaround used: chose the (R 1.30, 32-segment) minimum, 0.1918 mm^2 = 0.0017% of the
  part's surface, and reported it as a measurement artifact with this sweep as evidence
  rather than chasing a zero that a facet count happens to produce. `p05_thickness`
  3.209 mm and `median_thickness` 6.000 mm are quoted as the real wall evidence.

---

## F2 — `clearance` / `assert_disjoint` return `distance: 0.0` for INTERLOCKED but non-overlapping bodies (2026-08-07)

- symptom: with the slider pressed to full travel, the insert stud (O16.50) passes
  through the slider's large lobe (O17.20) with a true 0.35 mm radial gap everywhere.
  `clearance` reports:
  `{"distance": 0.0, "interfering": false, "overlap_volume": 0.0, "coincident_fit_hazard": false}`
  and `assert_disjoint {min_clearance: 0.1}` therefore FAILS with
  `"surface distance 0 mm <= required clearance 0.1 mm — 'sld_press' and 'ins_pull' touch or interfere"`
  even though the same receipt says `interfering: false` and `overlap_volume: 0.0`.
  Reproduced against a plain O16.50 cylinder too (op `c1` of the diagnostic), so it is
  not an artefact of the insert's threads or notches.
- minimal repro: `programs/controls_posed.json` ops `sld_press` (slider translated
  [-4, 0, 26]) and `stud_plain` (cylinder r 8.25, z 24..39), then
  `{"op": "clearance", "a": "sld_press", "b": "stud_plain"}`.
- expected vs actual: `digests/ops_core.md` section 9 documents `clearance` as returning
  `distance` + `interfering` + `overlap_volume`, and `assert_disjoint` as passing "iff
  measured surface distance EXCEEDS min_clearance". For two bodies that are linked
  (one threaded through a hole in the other) but nowhere closer than 0.35 mm, the
  reported distance is 0.0 and `assert_disjoint` is unusable. `distance: 0.0` and
  `interfering: false` are also mutually inconsistent as a receipt.
- workaround used: every must-CLEAR gate on an interlocked pair (N2 release twin, N4
  legal twin, the assembled receptacle+slider pose) was moved to the exact,
  tessellation-independent route the digest recommends — `union_all` + `assert
  {"shells": 2}` — and the clearance NUMBER is quoted from two analytic
  `measure_dimension` diameters ((17.20 - 16.50)/2 = 0.35 mm radial) rather than from a
  faceted distance. `clearance` is still recorded for its `overlap_volume`, which is
  correct (it matched the exact `intersection` volume to all digits: 32.26875959605661).

---

## F3 — `thread_ridge` overshoots its declared axial span by ~0.48 mm at EACH end (2026-08-07)

- symptom: `{"op": "thread_ridge", "major_d": 13.15, "pitch": 1.27, "z0": 1.0,
  "length": 12.0}` produces a solid whose `bounding_box` is z = 0.524 .. 13.475 — i.e.
  0.476 mm below `z0` and 0.475 mm above `z0 + length`. Because the insert's blind
  thread bore bottoms at z = 13.40, the top overshoot punched through the bore's floor
  into solid stock and the union with the body went from genus 0 to genus 1. The
  overshoot is ~0.375 x pitch, so it scales with pitch and cannot be treated as a
  constant fudge.
- minimal repro: `{"ops": [{"id":"r","op":"thread_ridge","major_d":13.15,"pitch":1.27,
  "z0":1.0,"length":12.0}, {"id":"b","op":"bounding_box","in":"r"}]}` — reported
  `min[2] = 0.524`, `max[2] = 13.475` against a declared span of 1.0 .. 13.0.
- expected vs actual: `describe` documents `z0` as "Axial start of the ridge (default 0)"
  and `length` as "Axial span of the ridge (`length/pitch` turns)". Nothing in
  `describe`, API.md or `digests/implicit_recipes.md` section 5 warns that the produced
  solid extends beyond `[z0, z0+length]`, and the measures the op returns
  (`z0`, `length`, `turns`, `major_d`, `minor_d`, `pitch`) all echo the DECLARED span,
  so the receipt cannot reveal it either — only a `bounding_box` can.
- workaround used: `bounding_box` the ridge, then size the span from the measured
  extent: the insert now uses `z0 1.0 / length 11.6` (measured 0.524 .. 13.075) inside a
  13.40 mm bore, and `programs/cal_thread_ridge.json` gates the ridge's own
  `exact_volume` against a closed form so any future change to the form is caught.

---

## F4 — `revolve`/`extrude` polygon helpers: `extrude` refuses CW profiles with a topology error, not a winding error (2026-08-07)

- symptom: an annular-sector cutter polygon generated inner-arc-then-outer-arc-reversed
  is clockwise. `extrude` refused it with
  `invalid_geometry: op 'rl1': extrude failed validate(): closed=false manifold=false
  genus=40 euler_characteristic=-74 shells=3 — refusing to bind an invalid solid`.
- minimal repro: `{"op":"extrude","profile":[[12.35,0],[0,12.35],[0,15.35],[15.35,0]],
  "height":1.5}` (any CW loop).
- expected vs actual: `digests/ops_core.md` section 4 says "CW extrude fails
  `invalid_geometry` loudly", which is true — but the message reports genus/shells of a
  garbage solid rather than naming the winding, so the first read looks like a
  self-intersecting profile. Cost: one wrong-diagnosis cycle.
- workaround used: `ensure_ccw()` in `programs/gen_common.py` normalises every generated
  profile's winding via the shoelace sign before it reaches the engine. Low severity —
  recorded per the protocol's "silent-ignore near-misses that cost you time count too".

## F5 — ace_fea `cylinder` region_selector needs a 3-vector `center_mm` + `length_mm`, not `center_mm`+`z_min/z_max` (2026-08-07)
- symptom: `{"ok": false, "error": "ValueError: cylinder 'center_mm' must be a 3-vector"}` from
  `tools/ace_fea_runner.py`; the job had `{"type":"cylinder","axis":"z","center_mm":[0.0,0.0],
  "radius_mm":7.025,"z_min_mm":1.0,"z_max_mm":13.4}`.
- minimal repro: any ace_fea job whose load `region_selector` is a finite z-cylinder.
  `python3 tools/ace_fea_runner.py fea_L1_insert_0p35.json`
- expected vs actual: `campaign/digests/tools_cookbook.md` "Selectors (voxel physics runners)"
  lists the selector TYPES (`all|bbox|plane|cylinder|sphere`) and says "Geometry keys in mm",
  but never gives the cylinder's key set. The truth is in `~/Work/ACE/engine/verify/selectors.py`
  `_resolve_cylinder`: `axis` ('x'|'y'|'z'), `center_mm` = a 3-vector POINT ON THE AXIS,
  `radius_mm`, and an OPTIONAL `length_mm` that makes the extent finite and CENTRED on
  `center_mm`. `z_min_mm`/`z_max_mm` are silently ignored (unknown keys), so a job that
  "looks" bounded is actually an INFINITE cylinder if the 3-vector guess happens to be right.
- workaround used: `"center_mm":[0,0,0.5*(z0+z1)], "length_mm": z1-z0`. Cost: one failed FEA
  run (~2 min). Suggest the cookbook gain a one-line key list per selector type.

## F6 — ace_fatigue requires a nested `"stress": {...}` block; the cookbook line reads as top-level keys (2026-08-07)
- symptom: `JobError: stress block required: {npy,...} or {sigma_ref_mpa} or {sigma_ref_pa}`
  from a job that had a top-level `"sigma_ref_mpa": 24.68`.
- minimal repro: `{"out_dir":..., "material":"PLA", "sigma_ref_mpa": 24.68,
  "spectrum":[{"cycles":100}]}` -> exit 1.
- expected vs actual: the cookbook's ace_fatigue line is
  "stress one of `{npy, unit?}` | `{sigma_ref_mpa}` | `{sigma_ref_pa}`", which parses naturally
  as "one of these keys, at the top level" (every other key on that line IS top-level). The
  runner wants `"stress": {"sigma_ref_mpa": ...}`.
- workaround used: nested the block. Cost: one failed run.

## F7 — ace_contact curve row 0 is the UN-EQUILIBRATED initial state and reports a fabricated-looking force when the start configuration penetrates the obstacle (2026-08-07)
- symptom: a detent-finger job whose beam tip starts 0.3385 mm inside the rigid profile (the
  physically correct SEATED state of a preloaded detent) returned, verbatim, row 0 of
  `curve.npy`: `lambda 0.0, insertion_force_n -674.8568, total_normal_force_n 982.1124,
  tip_uy_mm 0.0000, max_penetration_mm 0.2455, n_contact_nodes 1`. Row 1 (lambda 0.025) is a
  clean equilibrium: `insertion_force_n 1.5857, max_penetration_mm 0.0006`. The receipt's
  top-level `ok` is true and nothing flags row 0, so `max(|insertion_force_n|)` over the curve
  -- the obvious way to read a peak insertion force -- is wrong by 425x.
- minimal repro: any `kind:"profile"` obstacle whose surface is above the beam's initial tip
  position at lambda = 0.
- expected vs actual: the docstring says "there is no silent last-iterate" and "a step that does
  not converge raises", both true; but it does not say that ROW 0 is not a solved step. A
  penalty force of kappa x initial_penetration is written into the curve as if it were an
  actuator force.
- workaround used: (a) every statistic in `programs/eval_detent.py` skips row 0; (b) the seated
  state is measured by its OWN converged run with `motion.travel_mm = 0.0`. Suggest either
  solving lambda = 0 or marking row 0 in `curve_columns`.

## F8 — ace_fea is killed with no receipt at all under concurrent-agent memory pressure (2026-08-07)
- symptom: `tools/ace_fea_runner.py` on a 177,714-element / 590,556-DOF job produced stderr
  `loaded density grid (64, 64, 76) from ...` and then NOTHING -- no stdout line, no traceback,
  no non-zero-exit JSON. The identical job re-run solo finished in 292.9 s with
  `max_von_mises_pa 7827707.68`. Cause: ~6 design agents sharing 8 cores / 24 GB.
- minimal repro: run two >500k-DOF ace_fea jobs plus other agents' jobs at once.
- expected vs actual: the wire contract says the ACE runners "exit 0 even on failure -- parse
  `ok`". When the process is SIGKILLed there is no `ok` to parse, so a harness that trusts the
  contract sees an empty stdout and must invent an error. Not a solver bug -- an environment
  limit -- but it means "parse ok, never the exit code" is insufficient advice on a shared box.
- workaround used: the receptacle ships at 0.60 mm with NO refinement pair, and the campaign
  says so in `programs/gen_fea.py` and in ANALYSIS/DESIGN rather than quoting a convergence
  claim it did not measure; runs were serialised one at a time thereafter.

## F9 — `translate` takes `offset`, not `delta` (the warnings fence worked) (2026-08-07)
- symptom: `{"kind":"invalid_param","message":"op 's_00' ('translate'): bad params: missing
  field `offset`"}` plus the warning `unknown param 'delta' - 'translate' does not accept it`.
- expected vs actual: no doc claimed `delta`; this is logged only as EVIDENCE THAT THE 2026-08-06
  warnings fence does exactly its job -- the report named the offending key AND pointed at
  `describe {"name":"translate"}`, so the fix took one edit. Recorded as a positive control on
  the fence, not as a complaint.

## F10 — ace_fea_tet (gmsh) refuses a watertight, exact-route STL that carries a `thread_ridge` union (2026-08-07)
- symptom: `{"ok": false, "error": "Exception: Invalid boundary mesh (overlapping facets) on
  surface 45 surface 116"}` from `tools/ace_fea_tet_runner.py` at `elem_size_mm: 0.9` on
  `parts/insert.stl`.
- minimal repro: `python3 tools/ace_fea_tet_runner.py programs/tet_groove_insert.json`, where
  the STL is the shipped insert (route `exact`, `watertight: true`, `validate` valid/closed/
  manifold, genus 0, shells 1, components 1, STEP round-trip -0.065%).
- expected vs actual: the cookbook says the tet runner's geometry is `stl` (watertight) and
  calls it "the curved-geometry twin". Every in-tree gate the kernel offers says this mesh is
  clean; gmsh still rejects it. The offending surfaces are on the helical `thread_ridge` band,
  where the ridge is unioned into the bore with a 0.15 mm embedment (AMEND A6/A12) -- the union
  leaves slivers that are watertight but not gmsh-admissible.
- workaround used: NONE. The A3 row ships as **required, NOT PERFORMED** with this error
  quoted verbatim in DESIGN s17.2. A de-threaded proxy insert was deliberately NOT substituted:
  a tet number from geometry that is not the shipped part would be a fabricated cross-check.
  Consequence recorded: the groove root has one numerical read and one closed form, and they
  disagree by 48% (DESIGN s17.4 D-P2).

## F11 — `mesh_components` reports a one-body part as 9 bodies after a STEP round-trip, while `validate` still says shells 1 / closed / manifold (2026-08-08)
- symptom: the shipped receptacle measures `components: 1, is_one_body: true, triangles: 11434`
  natively. Re-imported from its own exact AP203 export it measures
  `{"components": 9, "is_one_body": false, "triangles": 4690, "tol": 0.05, "weld_tol": 0.001}`
  while `validate` on the SAME imported body still returns
  `{"valid": true, "closed": true, "manifold": true, "shells": 1, "genus": 1}` —
  with one flip: `geometric_ok` goes `true` (native) -> `false` (imported).
  Insert: 1 -> 5 components (30242 -> 29722 tri). Slider: 1 -> 2 components (450 -> 290 tri).
  Raising the measure's `tol` 0.05 -> 0.2 does not change the count (still 9).
- minimal repro:
  `{"ops":[{"id":"rec","op":"import_step","file":"rt/receptacle.step"},
           {"id":"v","op":"validate","in":"rec"},
           {"id":"mc","op":"mesh_components","in":"rec"}]}`
  run with `kernel-api run prog.json --out-dir .` against a STEP written by `export_step`
  from the same session.
- expected vs actual: DELIVERABLE_SPEC §2.2 makes `assert components:1` the first-class
  single-body gate and OPERATOR_BRIEF §8 calls a severed part "the hardest silent failure".
  Applying that gate to an `import_step` body — which is the doctrine-recommended way to
  re-bind shipped geometry for posed controls — fires `assert_failed` on a part that is
  provably one body. The cause looks like tessellation, not topology: STEP re-import
  re-facets at a much coarser density (11434 -> 4690 tri) and adjacent analytic faces do not
  share vertices at the 0.001 mm weld tolerance, so the FACETED component walk sees cracks.
  `mesh_components` is tagged `provenance: faceted`, so this is arguably in-contract — but
  the gate that the spec calls mandatory then cannot be run on round-tripped geometry, and
  nothing in the docs says so.
- workaround used: the `components: 1` gate is asserted ONLY on natively built bodies (it is
  in all three `part_*.json` programs and passes). Programs that re-bind via `import_step`
  (`controls_posed.json`, `asm_scene.json`, `sweep_motion.json`) assert `validate` /
  `shells` instead, and record `mesh_components` as a MEASURE with this finding attached.
  The discrepancy is published in ANALYSIS.md §1 rather than hidden.

## F12 — `export_stl` `tol` has no observable effect on a STEP-reimported body (2026-08-08, low severity)
- symptom: `{"op":"export_stl","in":<import_step body>,"file":...}` writes a 15,166,484-byte
  (303,329-triangle) STL for the receptacle whose NATIVE export is 18,382 triangles. Adding
  `"tol": 0.05` — a parameter `describe {"name":"export_stl"}` lists as a real optional number,
  so it raises no `warnings` entry — produces a byte-identical 15,166,484-byte file.
- minimal repro: import a STEP written by `export_step`, then `export_stl` it twice, once with
  and once without `tol`; `cmp` the two outputs.
- expected vs actual: a chord-tolerance parameter that is accepted and documented should change
  the tessellation, or the op should say it does not apply on this path. Neither happens. The
  root cause looks like the STEP round trip: the exported AP203 carries 1604 fragmented faces
  (boolean chains are not coalesced on export, and no coalesce op exists on the surface), and
  each face gets its own minimum triangle budget regardless of `tol`.
- workaround used: none needed — the affected files are RENDER artefacts (`assembly/pose_*.stl`)
  and the renderers cope. Recorded because it cost time and because a campaign that trusted
  `tol` to control an output size would be quietly wrong. `parts/*.stl` are exported from the
  native bodies and are unaffected.

## F13 — `export_stl` silently downgrades to `route: voxel_healed` with ZERO warnings, so the zero-warnings gate is blind to it (2026-08-08, found by independent verification)
- symptom: `{"op":"export_stl","in":"c3","file":"optional/c3_thread_ring.stl"}` returns
  `{"route": "voxel_healed", "triangles": 38120, "watertight": true}` and `warnings: []`, while the
  structurally identical `thread_ridge` union in `part_insert.json` returns `route: "exact"`. The
  program report is `ok: true` either way. Nothing in the op result distinguishes "I exported your
  exact B-rep" from "I gave up and voxelised it", except a field nobody is forced to read.
- minimal repro: `python3 programs/gen_coupons.py`, then
  `python3 -c "import json;print([o['measures'] for o in json.load(open('receipts/coupons.report.json'))['ops'] if o['id']=='c3_stl'])"`
  → `route: voxel_healed`. Same op on `programs/part_insert.json#stl` → `route: exact`.
- expected vs actual: DELIVERABLE_SPEC §2.4 makes `route` a gated, quotable provenance tag, but the
  surface offers no way to REQUIRE a route: there is no `assert`-able key for it and no warning when
  the exact path is abandoned. A campaign can therefore pass "ZERO warnings across every report"
  (§2 gate 3b) while shipping voxel-healed geometry for a fit-critical coupon. Three of this
  campaign's shipped STLs (`assembly/pose_*.stl`) plus `optional/c3_thread_ring.stl` are on the
  healed route and none of the shipped prose says so.
- workaround used: none available in-program. Verification had to read every `export_stl` measure
  block by hand:
  `python3 -c "...[o['measures'].get('route') for o in report['ops']]"`. Suggested engine fix: allow
  `{"op":"assert","in":<body>,"export_route":"exact"}` or emit a warning when `export_stl` falls back.

## F14 — `export_stl`'s route flips between `exact` and `voxel_healed` on the FACET DENSITY of the operands, not on anything the program declares (2026-08-08, found while repairing F13)
- symptom: repairing F13 by making the C3 coupon blind-bored (topologically identical to the
  shipped insert) was necessary but NOT sufficient. The same union of a `cylinder`-difference shell
  with a `thread_ridge`, at the same diameters and the same 0.15 mm embedment, exports
  `route: "voxel_healed"` at `segments: 64` and `route: "exact"` at `segments: 128` or `256`:
  ```
  seg  64 -> genus 0, route voxel_healed
  seg 128 -> genus 0, route exact
  seg 256 -> genus 0, route exact
  ```
  Same op, same topology, same validity (`valid/closed/manifold`, genus 0, shells 1) in every case.
- minimal repro: a ring OD 22 / H 8, blind bore Ø13.00 from z=-1 to z=6.6, `thread_ridge`
  `major_d 13.30 / pitch 1.27 / z0 1.0 / length 4.8`, unioned; export at `segments` 64 vs 128.
  Full driver: `programs/gen_coupons.py#c3()`.
- expected vs actual: `segments` is documented as a TESSELLATION density knob. Here it silently
  decides whether the export is the exact B-rep or a voxel approximation of it — i.e. a rendering
  parameter changes the provenance class of a print file. Nothing warns; `ok` is true both ways.
  Combined with F13 (no assertable route key) this means a campaign can lose the exact route by
  tuning a facet count for speed and never find out.
- workaround used: C3 now sets `segments: 128` explicitly, matching the shipped insert's revolve
  density, with the reason written into `gen_coupons.py`; and `gen_coupons.py` gates every export
  route out-of-program (hard fail on anything but `exact`), with the mirror-image declaration in
  `gen_render.py` for the three render-artefact pose exports (all `voxel_healed`, by declaration).
  `programs/selfcheck.py` item 4h re-checks both against the declaration.

## F15 — `production_check.py` has no time-dependent creep mode, so its creep verdict is derived from the STATIC yield it is supposed to replace (2026-08-08)
- symptom: `{"load_character": {"sustained": true}}` produces
  `"allowable = yield 55.00 x creep_sustained_fraction 0.20 = 11.00 MPa"` — a scalar fraction of the
  static number, with no duration input and no reference to `tools/materials/pla.json#creep`. The
  job schema accepts no `duration_h`, so there is no way to ask the tool for the 30 d or 1 y cell.
  A campaign that gates a sustained load with this tool and stops there has, in effect, gated it on
  yield: this campaign's detent row shipped `creep SF 3.3029 PASS` off the 11.0 MPa allowable while
  the governing 23 °C table gives SF 1.50 / 1.05 / 0.75 at 24 h / 30 d / 1 y — the 1-year cell over
  the allowable outright.
- minimal repro: `python3 tools/production_check.py programs/prodcheck_detent_seated_creep.json`
  → `rules[].creep.allowable_mpa 11.0`. Compare
  `tools/materials/pla.json#creep.sig_allow_mpa["23C"]` = `{1h 7.5, 24h 5.0, 30d 3.5, 1y 2.5}`.
- expected vs actual: OPERATOR_BRIEF §7 says the pla.json creep TABLE governs and the 0.2-fraction
  is "a recorded conflict"; DELIVERABLE_SPEC §2.8 requires a sustained load to be gated on
  `creep_allowable_mpa(T, hours)` with the design duration stated. The only in-tree implementation
  of that lookup is `creep_allowable()` inside `tools/field_triage.py`, which is reachable only
  through a field REPORT, not as a design-time gate. So the tool a campaign is told to use for the
  verdict cannot produce the allowable the doctrine says governs.
- workaround used: `programs/gen_prodcheck.py#creep_table_gate()` reads
  `tools/materials/pla.json#creep.sig_allow_mpa` directly and emits
  `receipts/creep_table_gate.receipt.json` — every sustained row × every tabulated duration, with
  the anisotropy factor applied per row and an explicit design duration on each. The lenient
  `production_check` SF is carried alongside so the conflict stays visible, and
  `selfcheck.py` item 6f fails if a sustained row has no design duration. Suggested engine fix:
  accept `service.duration_h` in the production_check job and route it through the same
  `creep_allowable()` the triage tool already has.
