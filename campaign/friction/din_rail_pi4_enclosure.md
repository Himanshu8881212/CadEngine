# Friction log — din_rail_pi4_enclosure

Stage 2 logged none. Stage 3 hit two rough edges, both worked around inside the
campaign directory. Engine and tools source untouched.

## F1 — tolerance_stack.py CHAIN mode leaks a raw KeyError instead of refusing (2026-08-07)
- symptom: a CHAIN job with `"closes": {"min_required": 1.0}` (no
  `max_allowed`) returns, as its whole receipt,
  `{"ok": false, "error": "KeyError: 'max_allowed'"}` — and the persisted
  receipt file contains only those two keys, so the chain numbers are lost.
- minimal repro:
  `{"chain":[{"name":"d","nominal":1.4,"tol":0.15,"dir":1}],
    "closes":{"min_required":1.0}}`
  → `python3 tools/tolerance_stack.py job.json` (exit 1)
- expected vs actual: the tool already raises a reasoned
  `ValueError("chain mode needs \`closes\` {min_required, max_allowed}")` when
  `closes` is absent entirely (source line ~143), so a partially-specified
  `closes` is the one path that escapes the reasoned-refusal contract stated in
  its own docstring. `campaign/digests/tools_cookbook.md` shows both keys in the
  CHAIN example but never says both are MANDATORY, and several natural stacks
  are one-sided (a minimum engagement with no meaningful upper bound).
- workaround used: every one-sided chain in this campaign carries an explicit,
  physically-justified `max_allowed` (e.g. the nose-engagement chain is capped
  at the 4.0 mm width of flange underside actually available). See
  `programs/gen_tolerance_stacks.py`. Cost: one full 19-job re-run, ~10 min.

## F2 — production_check.py reports a temperature RATIO in the same `SF` field as stress rules (2026-08-07)
- symptom: for every job in this campaign the `temp` rule reports
  `"SF": 1.375` (= 55 C softening / 40 C service) with `"allowable_mpa": 0.0`.
  Selecting "the governing rule" as `min(rules, key=SF)` — the obvious reading —
  therefore returns `temp` with a 0.0 allowable on EVERY part, hiding the real
  strength margin (anisotropy/static, SF 5.6 here).
- minimal repro: any job with `service_temp_c: 40.0`, e.g.
  `{"material":"PLA","max_von_mises_pa":5.4e6,"service_temp_c":40.0,
    "orientation":{"build_dir":[0,1,0],"primary_load_dir":[0,1,0]},
    "load_character":{"sustained":false,"cyclic":false}}`
- expected vs actual: the cookbook documents the receipt as
  `rules:[{rule, allowable_mpa, demand_mpa, SF, pass, detail}]` without saying
  that `SF` changes UNITS between rules. Mixing a dimensionless temperature
  headroom into a stress-margin field invites exactly this misreading; it cost
  a wrong "governing" line in a first draft of this campaign's summary.
- workaround used: the campaign selects the governing rule only from
  `{"static","creep","anisotropy"}` and reports the `temp` row separately as a
  condition check. See the roll-up in `programs/gen_stage3_summary.py` and
  `receipts/production_check_summary.json`.

## Non-friction observations (recorded in the campaign, not here)
- ace_thermal's refusal of a 1.0 mm voxel model whose parity-fill severed the
  shell into 14 components is CORRECT behaviour, not friction; it is kept as a
  receipt (`receipts/refusal_thermal_voxel1p0_severed.json`).
- `union(base, blocked_lid)` exporting via route `voxel_healed` (1.05 M
  triangles) while `union(base, shipped_lid)` stays route `exact` (56.7 k) is
  recorded as campaign finding F-S3-7 with the route quoted, not as an engine
  defect — the healed route is a documented, honestly-labelled outcome.

## F3 — sweep_check.py reports `failed_stations: null` on a FAILING sweep (2026-08-07)
- symptom: a sweep that genuinely interferes returns a receipt whose top-level
  fields read `{"ok": false, ... }` with `failed_stations` **null**, e.g.
  verbatim from `receipts/sweep_lid_slide.json` (pre-fix run):
  `"ok": false` and the runner's own stdout line
  `sweep_lid_slide    ok=False failed_stations=None`.
  The information that actually localises the failure is one level down, in
  `watches.<id>.first_interfering_t` (5.0) and
  `watches.<id>.interfering_t_ranges` ([[0.0, 90.0]] on the first attempt).
  A caller that branches on `failed_stations` — the obviously-named field —
  sees `None` on a failing run and can read it as "nothing failed".
- minimal repro: any `sweep_check.py` job whose watched `clearance` interferes.
  Ours: `programs/sweep_lid_slide.json` built by `programs/gen_sweeps.py`
  (base + lid, lid translated along +X), run as
  `python3 "/Users/himanshu/Work/New-LMCAD/cad engine/tools/sweep_check.py" \
     "<part>/programs/sweep_lid_slide.json"`.
- expected vs actual: the field name promises the failing stations; the digests
  describe sweeps as a pass/fail free-motion proof. Actual: `ok:false` is the
  only top-level signal and the station data lives per-watch. Not wrong, but
  the null in a same-level sibling field is a near-miss that cost time.
- workaround used: the campaign reads `watches[*].first_interfering_t` and
  `min_distance` instead of `failed_stations`, and — because a sweep proves
  free motion only — the actual defect was localised by an exact-geometry
  probe (`programs/gen_s4_key_probe.py`: `intersection` + `exact_volume` +
  `bounding_box`), which named the interfering feature from its bbox.

## F4 — render_sheet.py header: overlay legend collides with the meta line (2026-08-07)
- symptom: on a 5-STL overlay with a long combined title the swatch legend
  drawn INSIDE the header band runs underneath the right-aligned meta string,
  so `base_shell`, `lid` and `th35_gauge` overprint
  `bbox 92.0 x 49.6 x 88.3 mm - 53044 tris - units mm - 2026-08-07`.
  Cosmetic only; every panel below is correct.
- minimal repro: `programs/sheet_assembly.json` in this campaign —
  `{"stls": [din_foot, latch_bolt, base_shell, lid, th35_gauge], ...}` ->
  `python3 "/Users/himanshu/Work/New-LMCAD/cad engine/tools/render_sheet.py"
   "<part>/programs/sheet_assembly.json"`; see renders/assembly_sheet.png.
- expected vs actual: the sheet's own presentation contract says "nothing
  floats, nothing touches a border" and the legend is placed at
  `mg + text_w_px(name) + 30` with no collision test against the meta text,
  which is right-aligned from the other side. Actual: the two overlap once the
  title is long enough.
- workaround used: none needed — shipped as-is and noted here rather than
  silently cropping the legend or shortening the part list. The per-body
  sheets (1 STL each) have no legend and are unaffected.

## F5 — kernel-api report echoes the `--out-dir`-resolved file path, so program reports are not byte-reproducible across equivalent out-dir spellings (2026-08-08, found by independent verification)
- symptom: re-running the README "Reproducing" step 1 exactly as documented
  (`"$K" run "$PART/programs/part_program.json" --out-dir "$PART"`) produces a
  `receipts/part_program_report.json` that differs from the committed one on
  every export op:
  `"file": "/Users/.../din_rail_pi4_enclosure/parts/din_foot.stl"` (re-run) vs
  `"file": "./parts/din_foot.stl"` (committed). Same for `controls_report.json`,
  `oracle_nc6_severed_report.json`, `oracle_nc4_blocked_lid_report.json`.
  Every measure in the reports is bit-identical; only the echoed path differs.
- minimal repro:
  `cd <part> && kernel-api run programs/part_program.json --out-dir . > a.json`
  `kernel-api run "<abs part>/programs/part_program.json" --out-dir "<abs part>" > b.json`
  `diff a.json b.json`  → differs only in `ops[*].file`.
- expected vs actual: DELIVERABLE_SPEC §3 (Determinism) requires committed
  artifacts to regenerate byte-identical from the documented command lines.
  The exported STL/STEP payloads DO (cmp-clean); the *report* does not, because
  the report stores the caller's out-dir spelling rather than a path relative
  to the out-dir. A campaign that generated its receipts from inside the part
  directory can never reproduce them with the absolute-path commands its own
  README documents.
- workaround used: verification compared reports with the `file` fields
  normalised; the geometric/measure content was confirmed identical. No engine
  or tool source touched.

## F5 addendum — resolved campaign-side by pinning the CWD (2026-08-08)
- The campaign's README "Reproducing" block now opens with `cd "$PART"` and
  passes `--out-dir "."` on every `kernel-api run`. That is the spelling the
  committed reports were generated with, so the documented commands now
  reproduce `receipts/*_report.json` byte-identically (verified: 21/21 files
  under `parts/ cad/ renders/ assembly/` plus `part_program_report.json` and
  `controls_report.json` cmp-clean on a full re-run). The engine-side issue
  stands as filed — a report that echoes the caller's out-dir spelling still
  cannot be byte-compared across equivalent invocations, and pinning the CWD is
  a workaround, not a fix.

## F6 — campaign-side defect, not engine: a job's relative `out` path resolved against the CWD (2026-08-08)
- symptom: `python3 "$PART/programs/latch_statics.py" "$PART/programs/latch_statics_job.json"`
  run from the repo root died with
  `FileNotFoundError: [Errno 2] No such file or directory: 'receipts/latch_statics.json'`,
  even though the script already computed `HERE`/`ROOT` from `__file__` for
  every other path. The job's `"out": "receipts/latch_statics.json"` was opened
  against the CWD.
- minimal repro: `cd "$ROOT" && python3 "$PART/programs/latch_statics.py" "$PART/programs/latch_statics_job.json"`.
- expected vs actual: README documents the command as runnable from the repo
  root; it only worked with the CWD set to the part directory.
- workaround used: none — FIXED at the root in campaign code.
  `latch_statics.py` gained `_resolve()`, which resolves any relative job path
  (both `out` and the new `measured_areas_from`) against the PART directory.
  Recorded here because it is exactly the class of defect the friction protocol
  exists to surface, and because it is the reason the whole Reproducing block
  now pins its CWD.

## F7 — ace_buckling_runner.py accepts a purely TENSILE load case and returns a positive buckling factor instead of refusing (2026-08-08)
- symptom: `programs/refusal_buckling_tension.json` clamps a 12 x 12 x 16 mm
  prism at `z <= 0.5` and applies 40 N along `[0,0,1]` at `z >= 15.5` — pure
  tension, no compressive stress anywhere in the applied direction. The runner
  exits 0, `ok: true`, `error: null`, and returns
  `buckling_load_factor` 18437.371266390153, `critical_load_N` 737494.85, plus a
  full knockdown block with `design_critical_load_n` 368747.4 N. Its own note
  shows what the eigenvalue is riding on: "most-compressive principal pre-stress
  -1.154e+05 Pa" — the Poisson transverse contraction, not the applied axis.
- minimal repro:
  `python3 tools/ace_buckling_runner.py "<part>/programs/refusal_buckling_tension.json"`
- expected vs actual: a buckling solve with no compressive pre-stress in the
  applied direction has no physical bifurcation to find. Expected either a
  refusal ("no compressive pre-stress in the applied direction") or at minimum a
  warning on the receipt. Actual: a clean positive result with a knockdown block,
  which reads exactly like a design margin.
- consequence in this campaign (the reason this is filed rather than shrugged
  off): this very receipt was promoted into ANALYSIS.md §2.5 and DESIGN.md §3.3
  as a buckling margin "on the LC1 load path" — wrong geometry AND wrong sign —
  and survived a full rebuild because no program regenerated it. Caught by
  independent verification, not by any gate.
- workaround used: the campaign now runs `ace_buckling` on the SHIPPED bolt
  solid under a genuinely compressive reference load
  (`programs/gen_buckling.py` -> `receipts/buckling_bolt_cam.json`; the receipt
  carries `_compression_check` and the most-compressive principal pre-stress is
  -2.364e+06 Pa on the loaded axis). The tension probe is retained as a recorded
  NON-refusal (`receipts/refusals.json` row `A-R6b_buckling_no_compression`,
  `refused: false`, with `recorded_as` and `not_a_design_number`), and its full
  output ships as `receipts/refusal_buckling_tension_NONREFUSAL.json`.
  The tool-side fix is the maintainer's.

## F8 — DELIVERABLE_SPEC §2's connectivity-oracle example is not constructible with this kernel (2026-08-08)
- symptom: §2 asks for "a split-body variant that the connectivity gate catches
  while `shells` still reads 1". Every route tried reports `shells: 2` as well:
  - boolean-severing the shipped foot (`programs/oracle_severed_program.json`)
    gives `s_val` `{valid true, closed true, manifold true, genus 1, shells 2}`
    and `s_mc` `{components 2}` — both gates fire.
  - the obvious way to get one B-rep shell out of two lumps, a vertex-touching
    union, is REFUSED by the kernel:
    `{"id":"u","op":"union","a":<box 0..10>,"b":<box 10..20 offset in x,y,z>}` ->
    `invalid_geometry: op 'u': union failed validate(): closed=true manifold=false genus=1 euler_characteristic=3 shells=2 — refusing to bind an invalid solid`
    (the refusal is correct; it just closes the route).
- minimal repro: the two programs above; second one is 6 ops.
- expected vs actual: the spec's parenthetical implies such a body is buildable.
  With this kernel's `validate` (shell count on the bound solid) and `union`
  (rejects non-manifold results), it is not — at least not by any construction
  this campaign found.
- workaround used: the oracle is shipped as it is (it DOES prove the gate can
  fail: `assert components: 1` -> "measured 2, expected 1", exit 1, while
  validity/watertight/wall gates all stay green), and ANALYSIS.md §8.1 now reads
  both `shells` and `components` out of the receipt and states the general rule
  — `components` is guaranteed to catch a split body, `shells` is not — instead
  of the false specific claim that `shells` read 1 here. Doc-vs-binary
  contradiction filed per §4.

## F9 — tools/param_optimize.py writes `evals` as an int while the surrounding roll-ups treat it as a list (2026-08-08, reported by independent verification)
- symptom: `receipts/optimize_latch_receipt.json` carries `"evals": 72` — a
  count. An auditor's roll-up script that did `len(receipt["evals"])`, which is
  the natural reading for a key named `evals` and the shape other receipt
  roll-ups in this tree use for per-evaluation records, raised
  `TypeError: object of type 'int' has no len()`.
- minimal repro: `python3 -c "import json;print(len(json.load(open('receipts/optimize_latch_receipt.json'))['evals']))"`
- expected vs actual: either the key should be `n_evals` (a count) or `evals`
  should be the list of evaluations. Today it is a count under a plural name,
  and the per-evaluation trace is only on stdout, not in the receipt.
- workaround used: the campaign reads it as a scalar
  (`gen_stage3_summary.py` -> `"evals": opt["evals"]`). No campaign-side
  problem; filed so the maintainer can pick one convention.
