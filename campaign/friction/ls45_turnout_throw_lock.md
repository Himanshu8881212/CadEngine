# Friction — ls45_turnout_throw_lock

## F1 — ACE runners leak a `tmp*.json` scratch program into `out_dir`, one per run, forever (2026-08-08)

- symptom: after each `rebuild.sh`, `receipts/fea/<job>/` gains one more
  unreferenced, mode-0600 file named `tmp<random>.json`. Eight had accumulated
  in `receipts/fea/buckling/` alone. Verbatim content of one
  (`receipts/fea/buckling/tmpxes32f5r.json`):

  ```json
  {"ops": [{"id": "pil", "op": "cylinder", "base": [0, 0, 0], "axis": [0, 0, 1],
    "radius": 4.0, "height": 27.51, "segments": 64},
   {"id": "_ace_grid", "op": "sample_density_grid", "in": "pil",
    "origin": [-4.8, -4.8, -0.8], "voxel": 0.8, "shape": [12, 12, 37],
    "supersample": 2, "file": "solid_fraction.npy"}]}
  ```

  It is the generated LMCAD program that `engine.lmcad.sample_part` runs to
  voxelise the geometry — a genuine intermediate, written next to its output
  and never unlinked. The random name means it is never overwritten, so the
  directory grows without bound.

- minimal repro, from the repo root:

  ```sh
  ls rail_system/ls45_turnout_throw_lock/receipts/fea/blade_coarse/ | wc -l
  python3 tools/ace_fea_runner.py \
      rail_system/ls45_turnout_throw_lock/programs/fea_blade_coarse.json \
      --out rail_system/ls45_turnout_throw_lock/receipts/fea_blade_coarse.json
  ls rail_system/ls45_turnout_throw_lock/receipts/fea/blade_coarse/ | wc -l   # +1
  ```

  The write happens under `tools/_ace.py:load_geometry` ->
  `engine.lmcad.sample_part(..., out_dir / "solid_fraction.npy", ...)`, i.e.
  inside the ACE package, not in `tools/`.

- expected vs actual: `solid_fraction.npy`, `disp_field.npy` and
  `stress_field.npy` land in `out_dir` with FIXED names and are overwritten on
  re-run, which is what DELIVERABLE_SPEC §3's determinism rule needs. The
  program JSON breaks that pattern: it is randomly named, never cleaned, and
  never referenced by any receipt. DELIVERABLE_SPEC §5.12 requires the shipped
  directory to contain "no orphan scratch files", so an accumulating temp file
  inside `receipts/` is a rule the campaign cannot satisfy without a
  workaround.

  It also has a second-order effect worth recording: `analysis/ANALYSIS.md`
  states how many receipts were walked when scanning for `warnings` arrays.
  With the leak, that count grew by 3 on every rebuild, so a document that is
  gated on regenerating BYTE-IDENTICALLY could never converge. The leak was
  found by that gate failing, not by inspection.

- workaround used: `programs/rebuild.sh` step 6 deletes
  `receipts/fea/*/tmp*.json` immediately after the physics step, with a comment
  pointing at this entry. Nothing under `crates/` or `tools/` was touched. The
  fix belongs in `sample_part`, which should either use a fixed name (like the
  `.npy` files beside it) or unlink the program after the run.

## F2 — 2026-08-10 engine rebuild breaks exact-route STL export of multi-loop planar faces (2026-08-14)
- context: the campaign was green through Stage 4 on 2026-08-08 (2 consecutive
  byte-identical rebuilds). `target/release/kernel-api` was REBUILT 2026-08-10
  12:09 from a working tree with uncommitted crates/ modifications (git status
  lists `kernel-brep/src/tessellate.rs`, `tessellate_adaptive.rs`,
  `kernel-core/src/mesh/*` among ~20 modified files). Under the new binary the
  UNCHANGED `part_crank_handle.json` fails its `export_stl` gate; the other
  four members still pass.
- symptom (deterministic, 3 identical runs, byte-identical receipts), verbatim:
  `op 'g_stl': mesh is not manufacturing-ready even after the voxel heal
  (voxel 0.3 mm): boundary_edges=0, non_manifold_edges=0,
  non_orientable_edges=0, non_manifold_vertices=0, degenerate_triangles=0,
  self_intersections=1 — refusing export`
  while `validate` on the SAME solid still reports geometric_ok TRUE — the
  exact B-rep is fine; only the exact-route tessellation self-intersects.
- minimal repros (scratchpad receipts, all exit 1 on the new binary):
  1. `extrude_with_holes` of a 360-gon disc (r 21.1) with ONE 96-gon hole
     (r 4.225), height 4 → `self_intersections=2` even AFTER voxel heal.
     The op is broken natively, independent of booleans.
  2. same with two holes → `self_intersections=2`; square outer with two
     holes → heal succeeds but route degrades to `voxel_healed`.
  3. boolean route: 360-gon disc extrude, MINUS a 98-vertex sector-polygon
     prism (the crank arc slot), MINUS a 96-seg cylinder →
     `self_intersections=3` after heal. Every pairwise subset (disc+slot,
     disc+bore, disc+two CYLINDER holes in any outer shape) exports
     exact-OK — the trigger needs a polygon-prism cut AND a cylinder cut
     AND (for the full part) a tangent coplanar union on the same face set.
  Also observed while bisecting: boolean ORDER now changes `validate`
  geometric_ok (bore-cut-first on the full C6 part reads geometric_ok FALSE),
  and several variants run 30-60+ s where C6 ran in ~5 s.
- expected vs actual: a binary rebuild must not turn a valid solid
  (`geometric_ok: true`) into an unexportable one on the exact route; the
  DELIVERABLE_SPEC section 2.4 gate `require {route: "exact"}` exists
  precisely to catch this. Nothing in the campaign changed between the green
  receipts and the failure — only the binary.
- workaround used (inside the campaign, revision C7, defect DC-29 in
  DESIGN.md section 14): sink the drive pin 0.2 mm into the plate (kills the
  exact coplanar tangent union — OPERATOR_BRIEF section 8 hygiene applied to
  a PLANAR tangency) and cut the pivot bore BEFORE the arc slot. No frozen
  dimension changed; the closed-form ledger is unchanged by construction
  (the sunk pin displaces solid plate exactly). Full gate suite exit 0 with
  `route: "exact"` restored. crates/ and tools/ untouched.
- residual risk for other campaigns: any part authored with
  `extrude_with_holes`, or with a tangent-union + polygon-cut + cylinder-cut
  face combination, will fail its export gate on the current binary even
  though it passed before 2026-08-10.

## F3 — same 2026-08-10 rebuild: asm_export refuses the merged scene of any SEATED assembly (2026-08-14)
- symptom: all five kinematic state programs, unchanged since the 2026-08-08
  green run, now exit 1 at `asm_export`. Verbatim (S0):
  `op 'export': refusing manufacturing output: boundary_edges=0,
  non_manifold_edges=0, non_orientable_edges=0, non_manifold_vertices=0,
  degenerate_triangles=0, proper_self_intersections=1362`
  (S1 1362, M 1575, S3 1376, S4 1376.) The per-instance STLs in `parts_dir`
  are written BEFORE the refusal; the refused op's receipt carries NO
  `measures` block, so the per-instance watertight/route receipts are lost.
- minimal repro: any assembly whose solved mates put two instances in exact
  designed contact (slide face on its station stop, distance mate 0), then
  `asm_export` with a merged `file`. The coincident triangles of the seat are
  counted as `proper_self_intersections` by the new manufacturing-output
  check, which cannot be disabled and has no per-op override param
  (`AsmExport { file, parts_dir, tol, voxel }` — nothing else).
- expected vs actual: a seated mechanism ALWAYS has coincident surfaces at
  its designed seats — that is what a seat is. The op's documented behaviour
  (and this campaign's 2026-08-08 receipts) treated the merged scene as
  documentation, with mesh-quality gating on the instances; the new check
  makes merged-scene export of any functional assembly state impossible.
  A `require`-style opt-in would be the right shape; a hard refusal is not.
- workaround used (campaign-side only): `gen_asm.py` moves `asm_export` to
  the program's LAST op (so `asm_export_step`, `asm_save`, solve, contacts
  and mass all complete); new `programs/run_asm.py` pins the refusal
  signature with a regex that requires EVERY other defect counter to be 0
  (a refusal that also reports open edges is a real defect and hard-fails),
  quotes it verbatim, and merges the kernel's own per-instance binary STLs
  into `assembly/scene_<state>.stl` by triangle-record concatenation
  (contributing no geometry); `check_asm.py` tolerates ok:false IFF the sole
  failing op is `export`, and replaces the lost per-instance route receipts
  with file-count gating plus each part program's own
  `require {watertight, route: "exact"}` export gate. If a future binary
  exports the merged scene again it lands as `scene_<state>_kernel.stl` and
  run_asm.py reports the workaround retirable — self-retiring, not
  self-hiding. crates/ and tools/ untouched.
- RETIRED 2026-08-24: the round-4 engine (2026-08-23,
  `campaign/fixlog/H-census-round4.md`; merged-scene policy
  `campaign/REBASELINE_RUNBOOK.md` item 2, pinned by
  `asm_ops::merged_scene_export_reports_...`) exports the merged scene as a
  diagnostic — `scene: true`, designed seats reported as
  `cross_instance_self_intersections` (fresh receipts: S0/S1 1362, M 1575,
  S3/S4 1376 — the very counts this entry's refusal quoted). The campaign
  workaround is fully retired: run_asm.py requires 5/5 green (the old pin
  kept only to NAME a regression), the Python scene merge is deleted,
  `assembly/scene_<state>.stl` is the kernel's own export, check_asm.py
  re-gates per-instance watertight/route from the restored receipts, and
  gen_docs.py's guard now fires if the refusal ever returns. See the
  campaign BUILD_LOG 2026-08-24 entry.
