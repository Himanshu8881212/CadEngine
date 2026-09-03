# Re-baseline runbook — engine fix round 2 (prepared 2026-08-14)

The pending kernel changes legitimately alter shipped artifact BYTES, so the
swap-in is a deliberate re-baseline, not a silent rebuild. Do NOT run this
while any campaign workflow is live.

## What is staged in source (isolated-target tested, binaries NOT yet swapped)

1. **Sealed-hole tessellation fix** (`tessellate_adaptive.rs`): planar faces
   with inner loops now triangulate their holes through the dense shared-seam
   sampling + bridged ear-clip. Pinned by `kernel-brep/tests/adaptive_holes.rs`
   (closed / one-body / orientable; volume 3640 vs sealed 4000; area 2296;
   boolean annular cap clean).
   **Byte impact: every shipped STL/3MF whose part has a holed planar face
   re-tessellates differently — and correctly.** Receipts that measured the
   sealed lie (support areas, `voxel_healed` routes on trivially exact parts,
   mesh_components refusals) change to the truth.
2. **Diagnostic scene export policy** (`write_mesh_scene`): the merged
   `asm_export` file reports `scene: true` + `cross_instance_self_intersections`
   instead of refusing designed-interference NC poses (SLAS F9 healed; the four
   `pose_nc*_fail.json` programs exit 0 again). Per-instance part files stay
   strict. Pinned by `asm_ops::merged_scene_export_reports_...`.
3. **T4 input-path fallback** (`resolve_input_or_out`): import_step /
   load_part / import_mesh / npy reads resolve beside the program FIRST, then
   under `--out-dir`; a total miss names both roots. Pinned by
   `imports::step_round_trip_resolves_across_the_out_dir_root`.
   Heals every `_rt_*.step` round-trip program under any out dir.
4. **2026-08-10 stealth-change adjudication** (see friction ledger): unknown
   params stay FAIL-CLOSED (docs updated to match); the manufacturing guard
   stays for print files.

## Still open (deliberately)

- `union_all` non-termination at ~13 disjoint bodies (arrangement cost doubles
  per shell; fix requires either a disjoint-append fast path with verbatim
  topology concatenation, or arrangement surgery — needs quiet CPU).
- Curved faces with inner loops (a bore through a barrel wall) are still
  tessellated from the outer loop only on BOTH paths — same family, rarer;
  needs UV-domain triangulation per quadric class.
- The residual winding defect on faces WITHOUT holes, if any remains after the
  hole fix — measure via the census below before assuming.

## Procedure (after the assembly workflow completes)

1. `cargo build -p kernel-api --release` (REAL target dir this
   time) + full `cargo test --workspace --release` + clippy + audit_docs.
2. **Census first**: re-run the 111-program regression harness EXPECTING byte
   changes; record per-part which artifacts changed and why (holed faces) in
   this file. Any change NOT explained by the hole fix is a stop-the-line bug.
3. Re-baseline each affected campaign by its own README "Reproducing" commands
   (programs → receipts → generated ANALYSIS/README), so docs regenerate in
   lockstep with geometry. Never hand-edit campaign files.
4. Re-run the non-orientable-edge census (26/41 pre-fix) and update the
   `watertight_means` disclosure in DELIVERABLE_SPEC §2.4 with the post-fix
   number.
5. Re-run `campaign/history/ORCHESTRATOR_VERIFICATION.md` checks end to end and append
   the round-2 results.

---

# Round-4 record (executed 2026-08-23/24)

Procedure above executed for the round-4 kernel (winding rebuild + heal-empty
refusal + witness diagnostics; `campaign/fixlog/H-census-round4.md`).

1. Build + full workspace suites green (brep all, core+model 409/0, api 124/0
   + new pins), clippy 0.
2. Census (clone-based, pre-re-baseline): 267 programs — 222 passed, 44
   NC-as-designed (after the classifier honored declared "expected REFUSAL"
   headers and the `_refus` substring), 0 real failures, 0 warnings, 1
   timeout (biomedical `sweep_motion`, the standing union_all cost item).
   Byte-changed: the SAME 24 artifacts as the round-2 expected class — the
   winding fix added zero new churn; doc byte-stability held everywhere, so
   receipt-quoted numbers were preserved (winding-order-only changes).
3. Re-baselined in place by each campaign's own programs: the 8 byte-changed
   campaigns + marine (whose `_gen -> cp -> parts` pipeline hides artifact
   regeneration from the census — its base STL was the portfolio's last
   non-orientable file). Biomedical exception stated loudly in its BUILD_LOG:
   `sweep_motion` (probe-only, zero artifacts) not re-run — 7 h without
   finishing vs ~7 min historic; union_all arrangement cost, amplified by
   round-4 tessellation work.
4. Non-orientable census: **0 of 65 portfolio export files** (was 26/41 ops,
   3–395 edges). DELIVERABLE_SPEC §2.4 updated. Density note: the cleat base
   re-tessellates 22x denser at the same tol (fold-refusal reroutes its
   countersink-family warped grids to interior refinement) — correctness
   first, disclosed in the campaign BUILD_LOG; no other density change
   observed (census doc-stability + historic receipt counts).
5. Papercut fixed along the way: bare-filename `run <prog>.json` invocations
   (empty input base) refused on sandbox canonicalize — fixed in
   `confined_join` (empty base == cwd), pinned by
   `imports::bare_program_filename_empty_input_base_still_resolves`.

Still open (unchanged): union_all disjoint-body cost; curved faces with inner
loops (UV-domain triangulation); the transverse-curve sliver family at fine
tolerances (witness-locatable now).
