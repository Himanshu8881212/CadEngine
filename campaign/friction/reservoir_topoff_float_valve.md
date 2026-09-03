## F1 — export route silently demotes to voxel_healed for any cutter ending inside an existing void, and for intersection operands sharing coincident end planes (2026-08-14)
- symptom: `export_stl` receipts flip from `route: "exact"` to `route: "voxel_healed"` (exit stays 0) after a `difference` whose cutter's end face lies inside an already-cut void, e.g. a guide-bore cutter overshooting into a chamber cavity. Bisect receipt: `hydroponics_system/reservoir_topoff_float_valve/receipts/defects/vb_gcut_heal_demotion_bisect.json` — `x_b1` exact, `x_b2` voxel_healed with no other change. The 0.3 mm heal then manufactured self-intersections out of legitimate 0.1 mm embeds (which are the documented boolean-hygiene idiom), failing exports that the exact route would have passed.
- minimal repro: box minus cylinder A (a cavity), then minus cylinder B whose end face lies inside cavity A; `export_stl` the result. Compare route against cutting B before A (stays exact). Program shape as in the bisect receipt above; run `"target/release/kernel-api" run prog.json --out-dir out/`.
- expected vs actual: OPERATOR_BRIEF §8 prescribes "overshoot cutters past faces; embed >=0.1 mm" — following exactly that idiom silently forfeits the exact route when the overshoot lands in a prior void, and 0.1 embeds are sub-voxel for the 0.3 heal that then takes over. No doc names void-piercing cutters as an exact-route hazard.
- workaround used: cut order re-planned (bores on virgin material first) where topology allows; embeds enlarged to >=0.4 (> heal voxel) elsewhere; the threaded valve_body ships route=voxel_healed, disclosed. Campaign proceeded; ~2 h of bisection to find the trigger.

## F2 — tolerance_stack labels a ran-and-failed analysis error_kind "internal" while exiting 2 (2026-08-14)
- symptom: a FIT job with designed interference (bore 3.90/shaft 4.008 m6) exits 2 with `exit_contract.code: 2, meaning "ok:false — tool ran and REFUSED, or the analysis failed"` but the receipt's top-level `error_kind` is `"internal"`. Receipt: `hydroponics_system/reservoir_topoff_float_valve/receipts/tol_pin_boss.json` (also tol_key.json, tol_gland_openloop.json).
- minimal repro: `{"fit":{"bore":{"nominal":3.9,"tol":0.15},"shaft":{"nominal":4.008,"tol":0.004}}}` -> `python3 tools/tolerance_stack.py job.json` -> exit 2, error_kind "internal".
- expected vs actual: OPERATOR_BRIEF §3.1 / tools_cookbook exit table map exit 2 to `error_kind: refusal.*|timeout|killed.*` and exit 1 to `usage|internal`. Here exit code and `ok` behave per contract but the kind string contradicts the documented family, so branch-on-error_kind automation would misclassify a legitimate analysis failure as a tool bug.
- workaround used: campaign automation branches on exit code + `ok` + the mode-level `pass` fields, not on `error_kind`, and the three designed-interference receipts are quoted by their `fit`/`chain` blocks. No block; cosmetic-but-contract-relevant.

## F3 — ace_fea_tet aborts the process (SIGABRT 134, no stdout, no receipt) on a kernel-signed watertight STL; wall_budget_s cannot catch it (2026-08-14)
- symptom: `python3 tools/ace_fea_tet_runner.py tet_lever_a.json` on the stage-3 baked lever STL (watertight, route exact, components 1) terminates with exit 134 and stderr `libc++abi: terminating due to uncaught exception of type std::runtime_error: Failed to reach critical value in pass 0 for measure(s): ScaledJac`. No last-line JSON receipt is emitted, breaking the wire contract; the in-runner wall budget never engages because the abort is in native code. The PRE-bake lever (same builder, arm_s 10) instead refused cleanly with `Exception: Invalid boundary mesh (overlapping facets) on surface 68 surface 70` (exit 1) at both elem sizes 1.6/2.2.
- minimal repro: `hydroponics_system/reservoir_topoff_float_valve/programs/tet_lever_a.json` (elem 1.6, wall_budget_s 900) against `parts/lever_arm.stl`.
- expected vs actual: OPERATOR_BRIEF §5.1 documents this abort class; _receipt.py synthesizes honest receipts for SIGTERM/SIGINT/timeout but a native SIGABRT still yields nothing. Expected per the exit contract: some receipt, any receipt.
- workaround used: campaign synthesized `receipts/tet_lever_*_ATTEMPTED.json` + `refusals_tet.json` marked "synthesized_by campaign" with the verbatim stderr; A1 hex8 + closed-form Kt band remain the stress surface (A7 was budgeted as MAYBE in DESIGN).

## F4 — tools/derived_model.py accepts a foreign job and silently runs its OWN exemplar instead of refusing (2026-08-23)
- symptom: `python3 tools/derived_model.py hydroponics_system/reservoir_topoff_float_valve/programs/orifice_model_job.json` (job = `{"model":"orifice_flow","out":"…/receipts/orifice_model.json"}`) prints `{"ok": false, "error": "KeyError: 'zeta'", "self_check": {…"gate": "overshoot_vs_closed_form"…}}` and exits 1. The `"model"` key is never read: the tool ran its worked exemplar `DampedOscillator` against our job and failed on the exemplar's own parameter. The self_check block in the failure receipt reports the EXEMPLAR's gates ("overshoot_vs_closed_form"), which reads like our model's gates passed.
- minimal repro: `echo '{"model":"orifice_flow"}' > /tmp/j.json && python3 tools/derived_model.py /tmp/j.json` -> exit 1, `KeyError: 'zeta'`.
- expected vs actual: OPERATOR_BRIEF §1.10 is that unknown/misspelled params FAIL rather than silently select a default; the kernel enforces this. This tool does the opposite — an unrecognised `"model"` key selects the exemplar. Expected: `invalid_param`-class refusal naming the unknown key (or a registry lookup), not a KeyError from a different model. The docstring does say `job.json  # run the worked exemplar`, so this is doc-consistent but contract-inconsistent, and the emitted error names the wrong model.
- workaround used: campaign derived models are subclasses driven through `programs/rtfv_models.py <job.json>` (as their own docstring says). This campaign's stage-4 self-check driver `programs/verify_all.py` had inherited the wrong invocation; the driver's own expected-exit check CAUGHT it (`orifice_model exit 1 (want 0) *** MISMATCH`), the driver was corrected, and the README "Reproducing" section now carries the correct line plus a warning. Cost ~15 min. No block.

## F5 — posed-instance strict export refuses a solid whose part program exports exact/clean (found re-verifying D-AS2 under engine round 4) (2026-08-24)
- symptom: in-program `asm_export` of the MAIN assembly still exits 1 under
  the round-4 kernel with verbatim `op '<id>': refusing manufacturing
  output: boundary_edges=0, non_manifold_edges=0, non_orientable_edges=0,
  non_manifold_vertices=0, degenerate_triangles=0,
  proper_self_intersections=5` — the same count stage 2 recorded as D-AS2
  and attributed to the two designed press fits.
- minimal repro: asm_main.json ops with `x_step`/`save` replaced by
  `{"op":"asm_export","file":"scene.stl","parts_dir":"scene_parts"}`
  (scratch probe, deleted). Instance write order is valve_body, rail_mount,
  dowel_pin_4m6x20, lever_arm, seat_poppet, o_ring_AS568_010, float_cage,
  ittf_ball_gauge; the first SIX per-instance STLs land in `scene_parts/`
  and float_cage's does not — the op dies writing the SEVENTH instance,
  before any merge exists.
- expected vs actual: round 4 (campaign/fixlog/H-census-round4.md; scene
  policy per campaign/REBASELINE_RUNBOOK.md item 2) exports merged scenes
  as diagnostics — designed cross-instance contacts land as
  `cross_instance_self_intersections` (proven the same day on rail ls45's
  five seated states and the cap wrench's flush-seat scene). Expected this
  assembly to export with its two press fits ON THE RECORD; actual: the
  PER-INSTANCE strict path refuses float_cage's posed tessellation with 5
  proper self-intersections, while `programs/part_float_cage.json` exports
  the IDENTICAL solid `route: exact, self_intersections: 0, watertight:
  true` at the same default tolerance. A rigid pose must not change mesh
  validity — this looks like the H4 sliver family flipping under the posed
  transform.
- consequence for the record: D-AS2's press-fit attribution is SUPERSEDED
  (docs regenerated 2026-08-24 to say so); the in-program merged export
  stays refused for THIS campaign, now for the measured per-instance
  reason. NOT an F3-family scene-policy pin. The .lmcasm runner route
  remains the shipped scene surface.
- workaround used: none needed (the runner route already ships, gated by
  check_asm.py incl. merged-scene watertight). Left for an engine fix
  phase: posed-instance tessellation parity with the part-program export.
