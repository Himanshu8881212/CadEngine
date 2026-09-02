# Friction log — `jar_top_seed_singulator`

Structured per `campaign/DELIVERABLE_SPEC.md` §4. Engine and tools source
untouched; every workaround lives inside the campaign directory.

---

## F1 — `creep_allowable_mpa(T, hours)` is unreachable from the surface the brief tells campaigns to use (2026-08-06)

- **symptom**: `OPERATOR_BRIEF.md` §7 and `DELIVERABLE_SPEC.md` §2 gate 8 both
  instruct campaigns to "gate against `creep_allowable_mpa(T, hours)`". No such
  callable exists on the JSON op surface or in `tools/`:

  ```
  $ grep -rn "creep_allowable_mpa" tools/ docs/ *.md
  docs/CHANGELOG.md:341: `kernel_model::materials::pla::{creep_allowable_mpa, ...
  DESIGN_GUIDE.md:3455:  ... gate against `kernel_model::materials::pla::creep_allowable_mpa`
  $ python3 -c "import sys;sys.path.insert(0,'tools');import production_check as P;P.creep_allowable_mpa"
  AttributeError: module 'production_check' has no attribute 'creep_allowable_mpa'
  ```

  The only implementation is Rust, `crates/kernel-model/src/lib.rs:4232`, not
  exported as an op (`{"op":"describe"}` lists 161 ops; none is creep-related).
- **minimal repro**:
  ```sh
  cd "/Users/himanshu/Work/New-LMCAD/cad engine"
  grep -rn "def creep_allowable_mpa" tools/          # no hits
  python3 -c "import sys;sys.path.insert(0,'tools');import materials as M;M.creep_allowable_mpa"
  # AttributeError
  ```
- **expected vs actual**: the brief promises a callable gate for the mandatory
  sustained-load check. Actual: a design agent driving the JSON + Python surface
  has no way to call it; the nearest Python equivalent is
  `tools/field_triage.py:418  def creep_allowable(mat, temp_c, duration_h)`,
  which is undocumented in OPERATOR_BRIEF, DELIVERABLE_SPEC and
  `digests/tools_cookbook.md` (field_triage is not in the tools cookbook at all).
- **workaround used**: `programs/design_material_gates.py` in this campaign
  imports `tools/field_triage.creep_allowable` and reads the same
  `creep.sig_allow_mpa` table from the in-tree PLA record, emitting
  `receipts/design_material_gates.json`. No tools source was edited.

---

## F2 — Python and Rust readers of the SAME creep table disagree above 55 °C; the Python one is the non-conservative direction (2026-08-06)

- **symptom**: `kernel_model::materials::pla::creep_allowable_mpa` documents and
  implements a hard refusal above the hot tier —
  `if ... temp_c > CREEP_TEMPS_C[1] { return 0.0; }` with
  `CREEP_TEMPS_C = [23.0, 55.0]` (`crates/kernel-model/src/lib.rs:4187,4233`),
  whose docstring says *"Returns 0.0 — i.e. 'no sustained load is defensible' —
  above the hot tier … A gate written as `stress <= creep_allowable_mpa(..)`
  therefore FAILS loudly in exactly the regime where no data exists, which is
  the intended behavior."*

  `tools/field_triage.py:436` instead falls back to the **last tabulated row**:
  ```python
  pick_t = next((k for v, k in temps if temp_c is not None and v >= temp_c), temps[-1][1])
  ```
  so it keeps returning the 55 °C cell for any temperature, flagging only
  `extrapolated: True`.

  Measured:
  ```
  23 C / 24 h -> field_triage 5.0 MPa (bucket 23C, extrapolated=False)
  40 C / 24 h -> field_triage 1.5 MPa (bucket 55C, extrapolated=False)
  55 C / 24 h -> field_triage 1.5 MPa (bucket 55C, extrapolated=False)
  70 C / 24 h -> field_triage 1.5 MPa (bucket 55C, extrapolated=True)
  120 C / 24 h -> field_triage 1.5 MPa (bucket 55C, extrapolated=True)
  ```
  Rust at 70 °C and 120 °C returns **0.0**. A campaign that gates on the Python
  helper at any temperature above 55 °C gets a **silent 1.5 MPa allowable where
  the Rust contract refuses outright** — the wrong direction for a safety gate,
  and `extrapolated: True` is a field a gate can easily not read.
- **minimal repro**:
  ```sh
  cd "/Users/himanshu/Work/New-LMCAD/cad engine"
  python3 -c "
  import sys; sys.path.insert(0,'tools')
  import materials as M, field_triage as F
  print(F.creep_allowable(M.get('pla').record, 120.0, 24.0))"
  # {'known': True, 'sig_allow_mpa': 1.5, 'temperature_bucket': '55C', ..., 'extrapolated': True}
  # Rust creep_allowable_mpa(120.0, 24.0) == 0.0
  ```
- **expected vs actual**: expected — one table, one refusal contract. Actual —
  two readers of `creep.sig_allow_mpa` with different out-of-range behaviour,
  and the surface campaigns are actually able to call is the permissive one.
- **workaround used**: this campaign's service ceiling is 40 °C, which is inside
  the table, so the divergence does not change any shipped number. To be safe,
  `programs/design_material_gates.py` enumerates the temperature bucket that was
  selected (`temperature_bucket`) and the `extrapolated` flag on every case, and
  `analysis/DESIGN.md` LC1/LC7 states 55 °C as a **condition limit**, never as a
  design temperature. No tools source was edited.

---

## F3 — `production_check.py`'s creep rule contradicts the material record's creep table (2026-08-06, pre-recorded by the brief, re-confirmed here)

- **symptom**: `production_check.py` computes the sustained allowable as
  `yield × thermal.creep_sustained_fraction = 55.0 × 0.2 = 11.0 MPa`,
  time-blind, while the same PLA record carries
  `creep.sig_allow_mpa["23C"]["24h"] = 5.0` and `["23C"]["1y"] = 2.5`.
  Measured `legacy_scalar_creep_mpa: 11.0` in
  `receipts/design_material_gates.json`.
- **minimal repro**: `python3 programs/design_material_gates.py` in this campaign
  dir; compare `legacy_scalar_creep_mpa` against `creep_cases`.
- **expected vs actual**: `OPERATOR_BRIEF.md` §7 already names this ("The legacy
  0.2-fraction (11–12 MPa) is a recorded conflict; **the table governs**"), so
  this entry is a re-confirmation with numbers rather than a new discovery — but
  the tool itself still emits the 11.0 MPa row with no pointer to the table, so a
  campaign that reads only the `production_check` receipt would ship a 2.2×
  optimistic sustained margin without knowing it.
- **workaround used**: `analysis/DESIGN.md` LC1 gates on the **table** cells
  (5.0 MPa at 23 °C/24 h, 1.5 MPa at the 40 °C ceiling) and quotes 11.0 MPa only
  as the named conflicting legacy value, never as a margin. Stage 2/3 will run
  `production_check.py` as the spec requires and annotate its creep row with this
  entry.

## F4 — `union_all` over many mutually-disjoint cutter bodies does not complete (2026-08-07)
- symptom: a program folding 13 mutually-disjoint cylinders into one cutter with
  `{"op":"union_all","in":[13 ids]}` produced no output in >120 s (killed twice;
  no error, no progress). The 13 bodies are pairwise disjoint except the 3
  bayonet notches, which overlap the central bore.
- minimal repro: `scratchpad/tseg_64.json` — disc r28 h9 seg64; 8 cylinders r5
  h11 at radius 21 (seg 32); bore r10.3 (seg 48); 3 notches r3 (seg 24); groove
  r14.3 (seg 32); then `union_all` of those 13, then one `difference`.
  `"…/target/release/kernel-api" run tseg_64.json --out-dir out` → no completion.
- expected vs actual: ops_core digest §6 documents `union_all` as an ordinary
  left fold and explicitly blesses the disjoint case ("Disjoint bodies keep
  their own shells through a union: `union_all` + `assert {"shells": N}` = exact
  N-body no-contact proof"). Actual: the disjoint fold is the pathological case.
  The SAME solid built as a chain of 13 `difference` ops against the disc
  completes in **0.38 s** (`scratchpad/tseq.json`, verified exit 0, genus 8).
- workaround used: every part in this campaign cuts with a CHAIN of
  `difference` ops on virgin primitives instead of one union_all cutter.
  Recorded in `programs/gen_seed_plate.py`'s docstring so the choice is not
  mistaken for arbitrary style.

## F5 — export_step and import_step resolve `file` against DIFFERENT roots (2026-08-07)
- symptom: `{"op":"export_step","in":X,"file":"cad/p.step"}` with
  `--out-dir .` writes `<partdir>/cad/p.step`, but the very next
  `{"op":"import_step","file":"cad/p.step"}` fails
  `io: cannot read 'programs/cad/p.step': No such file or directory (os error 2)`.
  Adding `..` is refused: `invalid_param: path '../cad/_probe.step' must not
  contain '..' (it would escape the sandbox)`.
- minimal repro: `programs/_probe_rt.json` (deleted after the probe):
  cylinder → `export_step file "cad/_probe.step"` → `import_step file
  "../cad/_probe.step"`, run with `--out-dir .` from the part directory.
- expected vs actual: OPERATOR_BRIEF §3 says export `file` joins `--out-dir`
  and that `load_part.file` resolves against the program file's directory;
  ops_core §10 lists `import_step` under Exports & imports where the header
  states "Paths join `--out-dir`". Actual: `import_step` behaves like
  `load_part` (program-file-relative), not like the other export/import rows.
  Consequence: the DELIVERABLE_SPEC §2.12 round-trip gate cannot read the
  shipped `cad/<n>.step` from a program living in `programs/`.
- workaround used: each part program exports the STEP twice — the shipped
  `cad/<n>.step` and a byte-identical witness `programs/rt/<n>.step` — and the
  round-trip gate imports the witness. Documented in `lib_dims.gate_ops()` and
  in README so the duplicate is not read as sloppiness.

## F6 — `clearance` returns `overlap_volume: null` on high-face-count STEP-imported operands (2026-08-07)
- symptom: `{"op":"clearance","a":<wheel from import_step>,"b":<driver from
  import_step>,"tol":0.01}` returned
  `{"coincident_fit_hazard": true, "distance": 0.0, "interfering": true,
  "overlap_volume": null, "provenance": "faceted"}`.
  `overlap_volume: null` on an op that reported `ok: true`. The wheel STEP has
  23308 faces, the driver 659.
- minimal repro: build `part_geneva_wheel.json` and `part_driver_crank.json`
  (both exit 0), then
  `{"ops":[{"id":"w","op":"import_step","file":"rt/geneva_wheel.step"},
           {"id":"d","op":"import_step","file":"rt/driver_crank.step"},
           {"id":"d0","op":"translate","in":"d","offset":[52,0,0]},
           {"id":"c","op":"clearance","a":"w","b":"d0","tol":0.01}]}`
  run with `--out-dir .` from the part directory.
  CONTROL: the same op on two small STEP-imported boxes returns a normal
  `{"distance":1.0,"overlap_volume":0.0}` (scratchpad/clr.json), so it is not
  `import_step` per se — it is the operand size/complexity.
  SECOND CONTROL: a hand-built cylinder probe of the very same pin/slot
  interface against the same imported wheel returns
  `{"interfering": false, "distance": 0.2999999523162842}` — the true answer.
- expected vs actual: ops_core §9 documents `clearance` as returning
  `distance`, `interfering` and `overlap_volume` (mm³) and stresses that it
  "does NOT fail on overlap". Actual: it neither fails nor produces the number.
  `interfering: true` alongside `overlap_volume: null` is worse than a refusal,
  because a policy layer that reads `interfering` gets a verdict that the
  op could not actually compute — the exact silent-mode class the campaign
  gates against.
- workaround used: `programs/asm_stations.json` no longer imports the shipped
  STEPs. It rebuilds SIMPLIFIED CONJUGATE bodies in-program carrying exactly the
  two surfaces the claim is about (the wheel's 8 slot walls + 8 lock arcs, the
  driver's 215° crescent + drive pin). All six stations then return real
  numbers: `interfering: false`, `distance` 0.2999–0.3068 mm, `overlap_volume`
  0.0. The simplification is stated on the receipt, not hidden.

## F7 — `cone` is a true cone, never a frustum, and there is no frustum constructor (2026-08-07)
- symptom: `{"op":"cone","base":[21,0,-9.6],"axis":[0,0,-1],"radius":6.0,
  "height":5.5}` was written to open a chute OUT from r 6.00 to r 10.975. It
  actually cut a spike tapering to a POINT. Exit 0, `valid: true`, genus as
  expected — only the `exact_volume_within` gate caught it, at +1.39 %.
- minimal repro: any `cone` used as a draft/relief cutter where a frustum is
  meant; compare `exact_volume` against `(pi h/3)(R²+Rr+r²)`.
- expected vs actual: ops_core §4 lists `cone {base, axis, radius, height}` and
  the implicit `cone` LEAF as `{a, b, ra, rb}` ("capped frustum; `rb: 0` =
  sharp"). The two surfaces share a name but not a shape, and the exact side has
  no `rb`. Reading the leaf table first (as the implicit-heavy parts of this
  campaign do) makes the exact `cone` look like a frustum.
- workaround used: every drafted opening is built with `revolve` over an
  explicit (r, z) profile and then `translate`d off the axis, since `revolve`
  is about world Z only. Documented in `gen_housing_bottom.py` as defect D8.

## F8 — export/import path asymmetry also bites `import_mesh` (2026-08-07)
- symptom: `{"op":"hybrid_boolean", ..., "out":"parts/housing_top_threaded.stl"}`
  wrote the file correctly, and the next op
  `{"op":"import_mesh","file":"../parts/housing_top_threaded.stl"}` failed
  `invalid_param: path '../parts/housing_top_threaded.stl' must not contain
  '..' (it would escape the sandbox)`.
- minimal repro: as above, run from the part directory with `--out-dir .` and a
  program living in `programs/`.
- expected vs actual: identical in kind to F5 but on a different op pair, which
  is what makes it worth a separate entry: the rule is not "STEP is special", it
  is "every WRITE joins --out-dir and every READ joins the program directory".
  ops_core §10 tabulates `import_mesh` under Exports & imports, whose header
  says paths join `--out-dir`.
- workaround used: `hybrid_boolean` writes to `programs/rt/` (which both roots
  can name) and the README "Reproducing" section copies the result to `parts/`.

## F9 — `ace_fea_tet_runner` refuses a kernel-exported watertight STL with an opaque gmsh error (2026-08-07)
- symptom: `python3 tools/ace_fea_tet_runner.py programs/fea_tet_pin.json` prints
  exactly `{"ok": false, "error": "Exception: Singular matrix 3x3"}` and nothing on
  stderr. The STL is the campaign's own shipped `parts/driver_crank.stl`, which the
  kernel exported `route: "exact", watertight: true` and which `check_mesh` reports
  with boundary_edges 0 / non_manifold_edges 0. Reproduced at `elem_size_mm` 1.5.
- minimal repro: job `{"out_dir":..., "elem_size_mm":1.5, "stl":"<part>/parts/driver_crank.stl",
  "material":"PLA", "fixtures":[{"kind":"clamped","region_selector":{"type":"box",
  "min_mm":[-30,-30,13],"max_mm":[30,30,30]}}], "loads":[{"kind":"point","magnitude":10.05,
  "direction":[0,-1,0],"region_selector":{"type":"box","min_mm":[-23.5,-3.5,0],"max_mm":[-16.3,3.5,8]}}]}`
- expected vs actual: tools_cookbook says the tet runner's STL path takes "a WATERTIGHT
  surface STL (mesh_stl)" and ACE's `mesh_stl` docstring promises "a non-watertight STL
  will fail loudly in gmsh's surface-loop -> volume step rather than silently produce
  garbage". This STL IS watertight and the failure is not in that step — the message
  comes from gmsh's `classifySurfaces`/`createGeometry` reparametrisation, with no hint
  of which surface or why.
- second-order issue: `mesh_stl` exposes `high_order_optimize` and `second_order_linear`
  precisely for meshes gmsh struggles with, but `ace_fea_tet_runner.py`'s job schema has
  no key for either, so a campaign that must not edit `tools/` cannot reach the documented
  workaround.
- workaround used: the pin-root concentration was measured instead on the runner's OWN
  `specimen: "shouldered_bar"` Kt geometry, dimensioned to the pin root (d 6.00,
  l_small 12.30, D 20.00, r 1.00) — `programs/fea_tet_pin_specimen.json`. The refusal is
  quoted verbatim in `receipts/fea_tet_pin.json` and in ANALYSIS.md; it is not laundered
  into "the tet analysis was skipped".

## F10 — a 0.04 mm change to one cutter makes a LATER, 27 mm-distant boolean fail validate (2026-08-07)
- symptom: `part_geneva_wheel.json` builds (exit 0) with drop-window cutters of Ø10.70 and
  FAILS at op `s47` with `invalid_geometry: op 's47': difference failed validate():
  closed=false manifold=false genus=9 euler_characteristic=-17 shells=1 — refusing to bind
  an invalid solid` when the same 8 cutters are Ø10.74 or Ø10.78. `s47` is a Ø5.00
  rim-tip blunting cylinder at r 48.04 — **27 mm away from the nearest drop window**, and
  its own parameters are byte-identical between the passing and failing runs.
- minimal repro: `programs/gen_geneva_wheel.py`, change `WINDOW_D` from 10.70 to 10.74,
  regenerate, `kernel-api run programs/part_geneva_wheel.json --out-dir .` → exit 1.
  Sweep measured in this campaign: 10.70 exit 0, 10.74 exit 1, 10.78 exit 1.
- expected vs actual: a difference against a cutter that does not touch the failing feature
  should not change that feature's validity. The failure is also silent about WHICH loop
  went bad, so the only diagnosis available to a campaign is a bisection sweep.
- workaround used: the drop window is held at its frozen Ø10.70 over the optimised Ø10.08
  pocket (0.31 mm of radial relief, still above the design's own 0.30 rule) instead of
  growing with it. Recorded in `programs/gen_geneva_wheel.py` at the constant, and as
  amendment A14 in DESIGN.md §18.

## F11 — STEP round trip refuses a body the kernel itself calls valid, and the threshold is a 0.1 mm geometry change (2026-08-07)
- symptom: `part_housing_bottom.json` — the body passes `validate` (`valid:true,
  closed:true, manifold:true, genus:6, shells:1`), `export_stl` gives
  `route:"exact", watertight:true`, `export_step` succeeds — and then `import_step` on that
  very file fails: `invalid_geometry: op 'housing_bottom_rt': import_step failed validate():
  closed=false manifold=false genus=28 euler_characteristic=-37 shells=10`.
- minimal repro: same program, only `CHUTE_R_TOP` (the top radius of the chute frustum,
  a `revolve` profile) changed. Measured with `--out-dir .` so both halves of the
  round trip see the same file: 6.00 → exit 0; **6.04 → exit 1**; 6.10 → exit 0;
  **6.20 → exit 1**. Nothing else in the program differs.
- expected vs actual: DELIVERABLE_SPEC §2.12 makes the STEP round trip a mandatory gate,
  so a campaign is forced to tune an unrelated dimension until the exporter/importer pair
  happens to agree. The failure is in the round trip, not in the design: the same body
  exports an exact watertight STL either way.
- caution for others: a first attempt to bisect this with `--out-dir /tmp/...` gave six
  identical failures for six different radii, because (friction F5) `export_step` resolves
  `file` against `--out-dir` while `import_step` resolves it against the PROGRAM's
  directory — every test was re-importing one stale STEP. Bisect this class of bug only
  with `--out-dir .`.
- workaround used: `CHUTE_R_TOP = 6.10` (1.06 mm of chute lip beyond the optimised Ø10.08
  pocket, above the design's own 1.00 mm requirement), chosen by the measured sweep above
  and commented as such at the constant.

## F12 — `assembly_doc.py` refuses a legal job outright when the step prose is long, and the digest's `explode.axis` example is a string the tool cannot parse (2026-08-08)
- symptom: two separate stops in one tool.
  (a) `campaign/digests/tools_cookbook.md` §"assembly_doc.py" documents
  `explode` as `{axis, auto:true, gap_mm:8}` / `{axis, spacing_mm}` /
  `{axis, offsets:{...}}` without ever saying what `axis` IS. Passing the
  obvious `"axis": "z"` dies with the verbatim receipt
  `{"ok": false, "error": "ValueError: could not convert string to float: 'z'"}`.
  The tool's own docstring (line 21) does say `{axis: [x,y,z], ...}` — the
  digest is the drifted copy.
  (b) With the axis fixed, an 8-step / 12-BOM-row job is REFUSED:
  `{"ok": false, "error": "ValueError: steps do not fit the sequence panel even
  at 6 pt (8 steps, 12 BOM rows) — shorten the steps or split the doc"}`.
  There is no `steps_md_only`, no continuation page and no `sequence_panel`
  sizing knob, so a documentation-rich assembly cannot be documented at all
  until its prose is cut to fit a fixed panel.
- minimal repro:
  ```sh
  cd "/Users/himanshu/Work/New-LMCAD/cad engine/agriculture_system/jar_top_seed_singulator"
  python3 programs/gen_docs.py            # writes programs/asmdoc.json
  python3 "/Users/himanshu/Work/New-LMCAD/cad engine/tools/assembly_doc.py" programs/asmdoc.json
  ```
  (a) reproduces with `"explode": {"axis": "z", "offsets": {...}}`;
  (b) reproduces with the pre-2026-08-08 `steps` list in `gen_docs.py`
  (8 steps averaging ~230 characters against a 12-row BOM).
- expected vs actual: the cookbook implies any of the three explode forms works
  as written, and nothing anywhere warns that step LENGTH is a hard constraint
  coupled to BOM ROW COUNT. Actual: `axis` must be a 3-vector, and the doc
  refuses (correctly, rather than overlapping text) once
  steps × prose-length + bom_rows exceeds the fixed panel at 6 pt.
- workaround used: `explode.axis` is now the vector `[0,0,1]`; the eight steps
  in `programs/gen_docs.py` were cut to ~110 characters each, and the reasoning
  that was cut out of them (why each fit is what it is) was MOVED to
  `assembly/singulator_instructions.md` and `README.md` rather than deleted.
  The refusal is recorded here and in ANALYSIS.md; no number was dropped.

## F13 — `kernel-api asm` re-tessellates `mesh` instances ~18× denser on export, which makes `asm_contacts` intractable on a 5-part assembly (2026-08-08)
- symptom: `kernel-api asm assembly/singulator.lmcasm --out-dir assembly/ --window 2.0`
  ran for **55 minutes wall / 33 minutes CPU** without producing its report, and
  was stopped. It got as far as writing every instance export and the merged
  assembly mesh; the `contacts` entry never landed and the report JSON stayed
  0 bytes. The instance exports show why: the shipped meshes go IN at one size
  and come OUT ~18× larger.

  | instance | `parts/*.stl` (source) | `assembly/parts/NN_*.stl` (runner export) | ratio |
  |---|---|---|---|
  | housing_bottom  | 1 961 984 B |  17 553 284 B | 8.9× |
  | geneva_wheel    |   844 184 B |   9 532 884 B | 11.3× |
  | seed_plate_corn |   479 784 B |   5 551 284 B | 11.6× |
  | housing_top     | 2 178 984 B |  20 113 484 B | 9.2× |
  | driver_crank    |   312 884 B |   5 766 284 B | 18.4× |

  The merged `jar_top_seed_singulator_assembly.stl` is 58 516 884 B ≈ **1.17 M
  triangles** from a 150 k-triangle input set. The all-pairs `asm_contacts`
  proximity scan then runs against that inflated soup, and does not finish.
- minimal repro:
  ```sh
  cd "/Users/himanshu/Work/New-LMCAD/cad engine/agriculture_system/jar_top_seed_singulator"
  python3 programs/gen_docs.py           # writes assembly/singulator.lmcasm
  "/Users/himanshu/Work/New-LMCAD/cad engine/target/release/kernel-api" \
      asm assembly/singulator.lmcasm --out-dir assembly/ --window 2.0
  ```
  Five `{"source": {"mesh": "../parts/<name>.stl"}}` instances, identity or
  pure-translation poses, no mates.
- expected vs actual: `campaign/digests/implicit_recipes.md` §8.1 says a `mesh`
  source is "welded, measured honestly as a mesh", and §8.2 describes the
  per-instance exports as "one world-posed STL per instance" — i.e. the same
  mesh, posed. Nothing warns that a mesh instance is re-tessellated an order of
  magnitude finer on export, nor that `--window` does not bound the scan cost.
  There is also no way to ask the runner for `load`/`bom`/`export` WITHOUT the
  contacts scan, so one intractable op blocks the whole (otherwise cheap)
  report.
- workaround used: none available inside the campaign. `assembly/singulator.lmcasm`
  still ships and still loads; the artefacts the run DID produce ship
  (`assembly/bom.csv`, `assembly/bom.json`, `assembly/parts/*.stl`, the merged
  assembly mesh). The assembly documentation is carried instead by
  `assembly/singulator_assembly_doc.png` + `_instructions.md` from
  `assembly_doc.py`, and the interference/clearance claims were never resting on
  `asm_contacts` in the first place — they live on exact `overlap_volume` in
  `programs/nc*.json` (see DESIGN.md §9). README.md and ANALYSIS.md both state,
  in place, that `receipts/asm_singulator.report.json` is ABSENT and why.
  (Follow-up, same day: the 116 MB of inflated instance exports and the merged
  assembly mesh were deleted from the campaign after their sizes were recorded
  above — the table IS the evidence, and `parts/*.stl` already are the scene
  meshes at their correct density. `assembly/bom.csv` and `assembly/bom.json`
  are kept.)

## F14 — `tolerance_stack.py`'s job-level `receipt` path silently overrides the caller's output path and CLOBBERS a shipped receipt (2026-08-08, hostile-verifier pass)
- symptom: re-running the documented wire contract but sending the output somewhere
  else for diffing —
  `python3 programs/run_job.py "$T/tolerance_stack.py" programs/tol_wiper_gap.json /tmp/vr/tol_wiper_gap.json` —
  still REWROTE `receipts/tol_wiper_gap.json`. The tool's own `_receipt.emit(out, job,
  "tolerance_stack")` honours the job's `"receipt"` key (an absolute path into
  `receipts/`) regardless of what the caller asked for, and it writes a version WITHOUT
  run_job.py's `_job` / `_tool` / `_returncode` provenance keys. All 12 shipped
  `receipts/tol_*.json` were silently mutated by what was meant to be a read-only
  verification run.
- minimal repro: any `tolerance_stack.py` job carrying `"receipt": "<abs path>"`, invoked
  with a different destination argument. The destination argument is honoured too — you
  get TWO files, and the one inside `receipts/` is the un-provenanced one.
- expected vs actual: DELIVERABLE_SPEC §3 treats `receipts/` as the audit trail; a tool
  invoked with an explicit alternative output path should not write into it. Actual: the
  job file's `receipt` key wins and the campaign's committed receipt is overwritten.
- workaround used: re-ran all 12 stacks with the README's exact destination
  (`receipts/$n.json`) and confirmed byte-identity against the pre-run md5 baseline —
  ALL 12 restored byte-identical. Take an md5 baseline before ANY verification run.

## F15 — ACE solver receipts embed a wall-clock timing field, so they can never be byte-reproducible (2026-08-08)
- symptom: `receipts/fea_*.json` and `receipts/buckling_neck.json` differ on every run.
  Two back-to-back identical runs of `programs/fea_wheel_lc4.json` diff by exactly one
  line: `"fea_s": 28.662` vs `"fea_s": 41.358`. Every physics number (`max_von_mises_pa`
  7868914.925518583, `n_dof` 102777, `geometry_hash`) is bit-identical.
- minimal repro: `python3 programs/run_job.py "$T/ace_fea_runner.py"
  programs/fea_wheel_lc4.json receipts/fea_wheel_lc4.json` twice; `cmp` the two outputs.
- expected vs actual: DELIVERABLE_SPEC §3 determinism asks committed artefacts to
  regenerate byte-identical. STLs/PNGs do; solver receipts structurally cannot, because
  the runner records its own runtime inside the receipt.
- workaround used: verify solver receipts by numeric field comparison, never `cmp`.
  Reserve byte-identity checks for STL/STEP/PNG/CSV and for the deterministic tools.

## F16 — `describe {"name":"support_report"}` ships empty `doc` strings, so the `build_dir` sign convention is undocumented (2026-08-08)
- symptom: `describe` returns `{"name":"build_dir","type":"[x,y,z]","required":false,"doc":""}`.
  Nothing states whether `build_dir` is the print-up direction or the bed-normal
  direction, and the campaign's own files disagree (`analysis/DESIGN.md` §10 declares
  `[0,0,1]` for all five pieces while `programs/part_housing_{top,bottom}.json` gate on
  `[0,0,-1]`).
- minimal repro: a mushroom solid (Ø6×10 post under a Ø40×5 cap) →
  `build_dir [0,0,1]` reads `bed_area 28.23` (= π·3², the post foot) and
  `bridge_area 1226.39`; `build_dir [0,0,-1]` reads `bed_area 1254.62` (= π·20², the cap
  face) and `bridge_area 0.0`. So `build_dir` IS the print-up direction and `[0,0,-1]`
  means the modelled part is printed upside down.
- expected vs actual: docs promise `describe` is authoritative; for this op the parameter
  semantics that decide whether a support claim is even about the right orientation are
  simply absent.
- workaround used: determined empirically with the mushroom probe above before trusting
  any `steep_area`/`max_bridge_span` reading.

## F17 — a `hybrid_boolean` result cannot be gated: it binds no geometry, and nothing can re-bind it (2026-08-08)
- symptom: `parts/housing_top_threaded.stl` is the file this campaign actually prints
  (exact body + real 70-450 helical thread). It could not carry a single `assert`.
  Attaching `validate` to the fuse result gives, verbatim:
  `op 'p_validate' param 'in': 'housing_top_thread_fuse' is a measure/export op and
  binds no geometry`
  Re-importing the mesh it just wrote gives the identical refusal:
  `op 'p_validate' param 'in': 'hcheck' is a measure/export op and binds no geometry`
- minimal repro:
  `{"ops":[{"id":"hcheck","op":"import_mesh","file":"rt/housing_top_threaded.stl"},
           {"id":"p_validate","op":"validate","in":"hcheck"}]}`
  `"$KA" run prog.json --out-dir .`   -> exit 1, `missing_ref`
- expected vs actual: DELIVERABLE_SPEC §2.2/§2.4 require `assert components: 1` and a
  gated receipt on the body that ships — and §2.2 exists precisely for the case where
  "a severed part passes every other validity gate". The one body in this campaign that
  came off a `voxel_healed` route (72 self-intersections) is the one body the assert
  suite cannot reach. `describe {"name":"hybrid_boolean"}` makes `out` REQUIRED and
  documents no id-binding result; `import_mesh` is check-only.
- routes tried and why each failed:
  * `import_mesh` -> measure/export op, binds nothing (above).
  * `solid_from_implicit` DOES bind a solid, but speaks the CSG-combinator grammar,
    not the raw-math `expr_sdf` grammar a helix needs:
    `op 'thr': at expr: unknown combinator 'max' — supported combinators: union,
    intersection, difference, smooth_union, smooth_intersection, smooth_difference,
    displace, offset, shell, translat...`
    A helical thread is `mod(z - k*atan2(y,x))`; that grammar cannot express it.
  * No mesh->solid op exists anywhere in the 161-op surface (checked the full
    `describe` list: only `import_step` binds an imported body).
- workaround used: campaign-local `programs/gate_threaded_print.py`, which computes
  from the shipped mesh what is honestly computable — connected components by
  union-find (reads 1 on the print file, and cross-checks at 2 on
  `optional/nc6_severed_wheel.stl` and 1 on `parts/geneva_wheel.stl`), genus by Euler
  characteristic (reads 1 / 2 / 9, matching the kernel's own asserted genus on all
  three bodies), closed/manifold by edge parity, a two-sided CLOSED-FORM volume window
  (a union is monotone: V_body <= V_fused <= V_body + V_ridge), the standalone ridge
  volume against Pappus, and bed fit. It exits 1 on failure.
  `wall_thickness` and `support_report` are left OPEN and explicitly NOT faked —
  re-implementing a kernel gate in campaign Python would ship a different gate under
  the same name. Both are stated as open in README, DESIGN §19 and the receipt itself.
- ask: either let `hybrid_boolean` bind its result as a solid, or add a mesh->solid
  op. Today the engine can produce a print file it cannot gate.

## F18 — `clearance` on coarse inscribed cylinders reports an exact-contact 0.0 that is a faceting artefact (2026-08-08)
- symptom: NC3's legal twin (`nc3_pass`, lock column r 16.00 inside a concave scallop
  cut at r 16.5999 — a 0.5999 mm design clearance) measured
  `{"distance": 0.0, "interfering": false, "overlap_volume": 0.0}`.
  A `distance` of exactly 0.0 with `interfering: false` reads as a tangency and trips
  the campaign's own "float exact-contact poses by 0.1 mm" rule, so it looks like a
  bad pose. It is not: it is the two inscribed polygons touching.
- minimal repro: same three ops at three densities, pose untouched:
  `segments 36  -> distance 0.0`
  `segments 128 -> distance 0.0`
  `segments 360 -> distance 0.5999023914337158`
- expected vs actual: `provenance: "faceted"` is reported honestly, but there is no
  signal that the faceted reading has collapsed a real 0.60 mm gap to zero. A
  `coincident_fit_hazard: false` alongside `distance: 0.0` is actively misleading —
  the number that would have flagged it (the facet-induced uncertainty, ~sagitta of
  the coarser operand) is not reported.
- workaround used: raised the three NC3 bodies to 360 segments in `gen_controls.py`
  with the measurement across densities recorded in-source. The failure attitude
  still interferes (overlap 60.853 mm³), so the control did not weaken.
- ask: report a facet-uncertainty bound next to `distance`, or warn when
  `distance == 0.0` on faceted operands.
