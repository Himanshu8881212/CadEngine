# Friction — graham_deadbeat_escapement

## F1 — deep_groove_bearing catalog has no 623 (2026-08-14)
- symptom: `{"op":"deep_groove_bearing","designation":"623"}` fails, verbatim:
  `op 'b623': deep_groove_bearing: '623' is not in the seat table (603, 608, 625, 688, 6000, 6001, 6804)`
- minimal repro: `programs/catalog_refusals/refusal_bearing_623.json` →
  `"target/release/kernel-api" run programs/catalog_refusals/refusal_bearing_623.json --out-dir receipts/catalog_refusals_out`
  (receipt: `receipts/refusal_bearing_623.json`, exit 1)
- expected vs actual: ISO 15:2017 lists 623 (3×10×4) in the general plan; the
  frozen concept named it. The catalog seat table skips it while carrying the
  FLANGED variant `flanged_bearing` `F623` (probe `cat_bearing_f623` in
  `receipts/freeze_probe.json` binds fine, bbox 11.5×11.5×4.0).
- workaround used: design change, recorded in DESIGN.md — both bearing seats
  use `flanged_bearing F623` (same 3×10×4 boundary + Ø11.5 flange). The flange
  is an assembly IMPROVEMENT (axial register against the frame face). Not blocked.

## F2 — circlip_external / circlip_groove_external have no Ø3 (2026-08-14)
- symptom: verbatim: `op 'clip': circlip_external: Ø3 is not in the DIN 471 table
  (supported: Ø8, 10, 12, 15, 20, 25, 30)` and
  `op 'grv': circlip_groove_external: Ø3 must be a DIN 471 size (Ø8, 10, 12, 15, 20, 25, 30) and the axis non-zero`
- minimal repro: `programs/catalog_refusals/refusal_circlip_d3.json`,
  `programs/catalog_refusals/refusal_circlip_groove_d3.json` (receipts of the
  same names under `receipts/`, exit 1 each)
- expected vs actual: DIN 471 itself tabulates shafts down to Ø3 (groove
  d2 = 2.8, m = 0.5 for Ø3); the catalog table starts at Ø8. The frozen concept
  named 2× circlip Ø3 for arbor retention.
- workaround used: design change, recorded in DESIGN.md — circlips DELETED from
  the BOM. Axial retention re-routed: dowel arbor pressed in the bearing inner
  ring (ISO 2338 m6 on ISO 492 P0 bore = 0.002–0.016 mm interference, metal-metal),
  hubs carry D-flat + M3 set screw each (2 set screws instead of 1), outer ring
  captured by printed bearing cap + M3 screws. Not blocked.

## F3 — export_threaded is coarse-pitch-only; M8×0.75 cannot go through it (2026-08-14)
- symptom: `export_threaded` takes `m` only (no `pitch` param); with `m: 8` it
  SUCCEEDS but produces pitch 1.25 (receipt `receipts/refusal_export_threaded_m8_fine.json`:
  `"pitch": 1.25`, `"m": 8.0`) — the wrong thread for the M8×0.75 rating pair.
- minimal repro: `programs/catalog_refusals/refusal_export_threaded_m8_fine.json`
  (exit 0 — the receipt is the evidence: the produced pitch is 1.25, not 0.75)
- expected vs actual: concept requires the ISO 261 fine pitch M8×0.75 for rate
  resolution (0.75 mm/turn). No refusal fires — a silent wrong-pitch mode if
  unchecked; only a `require {pitch: 0.75}` gate would catch it (and did, in the
  freeze probe design).
- workaround used: `thread_ridge` (takes `major_d` + `pitch` explicitly) probed
  green in `receipts/freeze_probe.json` id `thread_ridge_m8x075`
  (pitch 0.75, minor Ø7.188, gated by `require`). That is the campaign's thread
  route; every thread op carries `require {pitch: 0.75}`. Not blocked.

## F4 — anchor export refuses: one self-intersection survives the voxel heal (2026-08-23)
- symptom: verbatim, `op 'g_stl': mesh is not manufacturing-ready even after the
  voxel heal (voxel 0.3 mm): boundary_edges=0, non_manifold_edges=0,
  non_orientable_edges=0, non_manifold_vertices=0, degenerate_triangles=0,
  self_intersections=1 — refusing export`, exit 1, while `validate` on the SAME
  solid reports `valid true, closed true, manifold true, shells 1, genus 1`.
- minimal repro: `kernel-api run programs/part_deadbeat_anchor.json --out-dir
  build/deadbeat_anchor` at the stage-2 revision of partlib.py (D-flat cut as a
  BOX spanning x[1.05,3], y[-3,3] after a Ø3.1 `drill`).
- expected vs actual: DESIGN_GUIDE/OPERATOR_BRIEF present the voxel heal as the
  fallback that always yields a manufacturing-ready mesh; here it fails at voxel
  0.3, and REFUSES OUTRIGHT at 0.12 (finer voxel, strictly worse outcome).
  `tol` 0.01 / 0.002 and voxel 0.1 / 0.05 all reproduce the refusal.
- workaround used: the D-bore is now cut with an ANALYTIC cutter,
  `intersection(cylinder Ø3.1, box x<=1.05)` — the solid the closed-form volume
  model already assumed.  Exports again; volume moved 14464.007 -> 14604.394
  against a 14598.641 target (-0.92 % -> +0.04 %).  Evidence:
  receipts/probe_exit_strip.json.

## F5 — anchor cannot reach the `exact` export route at all (2026-08-23)
- symptom: `export_stl` on the anchor reports `route: "voxel_healed"` from the
  exit dead-face cut (`a3`) onward; `a1` and `a2` export `exact`.  The shipped
  STL is therefore a 216k-triangle REMESH of a part whose smallest working
  feature is a 0.60 mm lock.
- minimal repro: programs/probe_exit_strip.py (mode A exports `a3`).
- expected vs actual: probed across exit-strip start radius (40.55 / 40.00 /
  41.30 / 41.90 mm), ring-cutter outer radius (42 / 43 mm) and azimuth window
  (303-318 / 304.8-316.5 deg) — the route never moved.  Boolean hygiene alone
  does not explain it; the exit ring cutter crosses the bow's inner arc at a
  shallow angle and the exact tessellator does not survive it.
- workaround used: SHIPPED AS voxel_healed and gated as such — `require`
  {watertight, route: "voxel_healed", self_intersections: 0, two_manifold: true,
  boundary_edges: 0, non_manifold_edges: 0}, so a silent route change fails the
  run in either direction.  Every dimensional measure in this campaign runs on
  the exact B-rep, never on this STL, and cad/deadbeat_anchor.step is the
  dimensional authority (round-trip 14603.835 vs 14604.394 exact).

## F6 — `union` fails validate() on a near-tangent posed pair (2026-08-23)
- symptom: verbatim, `op 'u_S10': union failed validate(): closed=false
  manifold=false genus=3 euler_characteristic=-5 shells=1 — refusing to bind an
  invalid solid`, at station S10 only, while the same two solids union cleanly
  at the other eleven stations.
- minimal repro: build/prove_stations/st_S10.json.
- expected vs actual: the refusal is correct behaviour for an invalid result,
  but it makes the union-identity overlap measure (|AnB| = |A|+|B|-|AuB|)
  station-fragile.
- workaround used: prove_stations.py / opt_anchor_eval.py run ONE PROGRAM PER
  STATION and fall back to `intersection` when the union refuses; an
  `intersection` that refuses with "produced an empty solid" is itself the
  machine-checkable receipt for zero overlap.

## F7 — the "merged scene export refuses" note reads wider than the binary behaves (2026-08-23)
- symptom: the campaign brief's KNOWN KERNEL DRIFT note says a post-2026-08-10
  kernel "refuses the FINAL merged scene-export op of an assembled attitude"
  when exact-contact seats or designed interference are present, and this
  assembly does refuse: `op 'scene': refusing manufacturing output: ...
  proper_self_intersections=1` (receipts/refusal_scene_merged_export.json).
  Read as written, that sounds like the assembled attitude cannot be exported
  at all — three earlier stages of this campaign proceeded on that assumption
  and shipped no assembly CAD.
- minimal repro: the SAME solved attitude, one op changed:
  `{"id":"astep","op":"asm_export_step","file":"..._assembly.step"}` instead of
  `{"id":"scene","op":"asm_export","file":"...stl","parts_dir":"scene_parts"}`.
  Command: `"target/release/kernel-api" run
  horology_system/graham_deadbeat_escapement/programs/asm_build.json
  --out-dir horology_system/graham_deadbeat_escapement/build/asm`
- expected vs actual: expected a refusal by analogy; actual **exit 0**,
  `{"parts": 20, "solved": true, "skipped": [], "bytes": 41854341}`.
  `asm_export_step` writes one STEP PRODUCT per instance and never builds the
  merged triangle soup, so the self-intersection that stops `asm_export` never
  arises. The refusal is a property of the MERGE, not of the attitude.
- workaround used: none needed — this is a capability that was being left on
  the table. `asm_export_step` is now in asm_build.json gated
  `require {parts: 20, solved: true, skipped: []}`, and the STEP it writes is
  the input to `tools/bom_audit.py` (receipts/bom_audit.json), which is how
  this campaign audits its BOM at all. Suggest the drift note say "the merged
  scene-export op" explicitly and name asm_export_step as the route that still
  works.

## F8 — production_dossier silently ignores a job-level `receipt` key (2026-08-23)
- symptom: `programs/dossier.json` carried
  `"receipt": ".../receipts/production_dossier.json"` (the key documented for
  `tools/bom_audit.py` and the _receipt.py rules). No such file was written and
  no warning was emitted; the outputs landed only at the `out_dir` names
  `bom_dossier.json` / `bom_dossier.csv`.
- minimal repro: `python3 tools/production_dossier.py
  horology_system/graham_deadbeat_escapement/programs/dossier.json` with a
  `receipt` key present; then `ls` for the named path.
- expected vs actual: the tool's own `--help` lists `out_dir` as REQUIRED and
  never mentions `receipt`, so the tool is behaving to its documented contract;
  the friction is that an unrecognised job key is accepted in SILENCE, which is
  exactly the near-miss class the kernel's `warnings` array was added to close.
  Cost: one wrong-path assumption in the doc generator.
- workaround used: the `receipt` key removed from dossier.json; gen_docs.py
  reads `receipts/bom_dossier.json`. Suggest tools echo unknown job keys the
  way ops echo `warnings`.
