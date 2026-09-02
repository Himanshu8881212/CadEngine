# FRICTION — turgo_runner (energy_system)

## F1 — `validate.geometric_ok` false-positives on polar patterns of off-axis tubes (2026-08-07)

- **symptom**: the shipped part reports
  `"geometric_ok": false` inside `validate` while every other check on the same
  solid is clean:
  `{"closed": true, "euler_characteristic": -10, "genus": 6, "geometric_ok": false, "manifold": true, "shells": 1, "valid": true}`
  (`energy_system/turgo_runner/receipts/part_report.json`, op `topology`).
  `crates/kernel-api/src/interp.rs:1448` defines the flag as
  `!kernel_brep::self_intersects(s)` and `crates/agent-bench` asserts
  `geometric_ok:true` for "geometrically sound", so a false reading looks like
  a severed/self-intersecting part. Everything independent says it is not.

- **minimal repro** (no campaign geometry involved — a plain off-axis tube
  patterned about Z; saved as
  `energy_system/turgo_runner/programs/repro_geometric_ok.json`):

```json
{"ops": [
 {"id":"o","op":"cylinder","base":[24,0,7],"axis":[1,0,0],"radius":14.2,"height":24,"segments":64},
 {"id":"i","op":"cylinder","base":[23,0,7],"axis":[1,0,0],"radius":12.0,"height":26,"segments":64},
 {"id":"d","op":"difference","a":"o","b":"i"},
 {"id":"vd","op":"validate","in":"d"},
 {"id":"p3","op":"polar_pattern","in":"d","count":3,"center":[0,0,0],"axis":[0,0,1]},
 {"id":"v3","op":"validate","in":"p3"}
]}
```
```sh
"/Users/himanshu/Work/New-LMCAD/cad engine/target/release/kernel-api" run \
  energy_system/turgo_runner/programs/repro_geometric_ok.json --out-dir /tmp/o
```

- **expected vs actual**: the digest (`ops_core.md` §9) documents `validate` as
  recording `closed / manifold / euler_characteristic / genus / shells / valid`
  and says nothing about `geometric_ok`; `agent-bench` treats it as the
  geometric-soundness oracle. Expected: a pattern of well-separated,
  individually sound bodies is sound. Actual, measured on the repro:

  | count | geometric_ok | shells | genus |
  |---|---|---|---|
  | 1 (unpatterned) | **true** | 1 | 1 |
  | 2 | true | 2 | 2 |
  | 3 | **false** | 3 | 3 |
  | 4 | true | 4 | 4 |
  | 5 | **false** | 5 | 5 |

  Non-monotonic in the copy count, which no real self-intersection can be.
  Control: the same `polar_pattern` at count 14 on a `sphere` and on a `box`
  reads `geometric_ok: true`, so it is not a generic multi-shell bug — it needs
  the off-axis cylinder pair (a tube). A single copy rotated by the same
  arbitrary angle (`rotate_z 25.714286`) reads **true**, so it is not the
  rotation alone either.

- **evidence the geometry is in fact sound** (turgo_runner blade, count 5):
  `exact_volume` 4882.954745913263 = exactly 5 x the single-blade
  976.5909491826512 (analytic); `mesh_components` = 5; `export_stl`
  `route:"exact"`, `watertight:true`; `clearance` between neighbours
  distance 6.394 mm, `overlap_volume` 0.0, `interfering` false;
  `assert_disjoint min_clearance 1.0` passes.

- **workaround used**: proceeded. The campaign does not and cannot assert
  `geometric_ok` (the `assert` op has no such check — only
  `valid/closed/manifold/genus/shells/components/volume`), so the flag never
  gated anything. The part is instead proven sound by the five independent
  receipts listed above, and the false reading is quoted verbatim in
  `analysis/DESIGN.md` §11 and in ANALYSIS.md rather than being suppressed.
  No engine or tools source was touched.

- **doc drift also noted**: `campaign/digests/ops_core.md` §9's `validate` row
  omits `geometric_ok` entirely, so an agent has no documented way to know
  whether the flag is load-bearing. Suggest documenting it (and, if it is meant
  to be load-bearing, exposing it in `assert`).

## F2 — ace_fea_tet: sliver facets refuse the mesher, and the retry ABORTS the process (2026-08-07)
- symptom: two distinct failures meshing the same stall sub-model.
  (a) true geometry (bucket + hub patch with the real r=30 cylindrical disk rim):
      `{"ok": false, "error": "Exception: PLC Error:  A segment and a facet intersect at point"}`
      The bucket crosses the disk rim at a shallow angle, so the exact boolean leaves
      facets as small as 0.00087 mm2 at x = 29.9..30.0 (measured on the exported STL).
  (b) rim idealised as a flat slab to remove those slivers: the runner produced NO stdout at
      all and the process died with an uncaught C++ exception:
      `libc++abi: terminating due to uncaught exception of type std::runtime_error: Failed to
      reach critical value in pass 0 for measure(s): ScaledJac`
      This BREAKS the documented wire contract (cookbook: "exit 0 always, {ok:false,error} line
      is the contract") — the caller gets an empty receipt file and no reason.
- minimal repro: energy_system/turgo_runner/programs/coupon_program.json (writes
  physics/coupon_blade.stl and physics/coupon_tetA.stl), then
  `python3 tools/ace_fea_tet_runner.py energy_system/turgo_runner/programs/tet_stall_a.json`
- expected vs actual: cookbook 1 sells ace_fea_tet as "the curved-geometry twin: true surfaces,
  resolves fillet stress concentrations". Actual: it cannot ingest a watertight, valid,
  manifold, components==1 exact-route STL that the kernel itself signs off, and one of the two
  failure modes is a hard abort rather than a receipt.
- workaround used: the stall case is analysed on the BUCKET ALONE (physics/blade_only.stl,
  which meshes fine: 8748 tets, mesh volume 974.886 mm3 vs exact 976.591 = -0.17%), rigidly
  clamped at x = 27.5 mm — the true disk-rim radius at the bucket's -y side. That is the
  LOWER-SUPPORT bound, i.e. the conservative one; the refusal of the upper-support twin is
  recorded in receipts/refusals.json and quoted in ANALYSIS.md rather than hidden.

## F3 — param_optimize: a constraint with `max: 0.0` crashes the whole run (2026-08-07)
- symptom: `{"ok": false, "error": "float division by zero"}` after two successful evals.
  Source: tools/param_optimize.py line 217, `penalty += v / c["max"] - 1.0`.
- minimal repro: any job with `"constraints":[{"expr":"<anything>","max":0.0}]`.
- expected vs actual: DELIVERABLE_SPEC 2.5 REQUIRES `steep_area == 0.0` exactly for a
  support-free claim, and 2.3b requires zero warnings — both are naturally written as
  `max: 0.0`. The optimizer cannot express either.
- workaround used: bounds raised to 0.01 mm2 (steep area) and 0.5 (integer warning count) with
  the reason recorded inline in programs/opt_job.json; the exact `== 0.0` assertions are re-run
  in the full gate suite on the chosen optimum, which is where they belong anyway.

## F4 — derived_model.py: `--selftest` is hard-wired to the exemplar (2026-08-07)
- symptom: `python3 programs/jet_force_model.py --selftest` ->
  `KeyError: 'overshoot_pct'` at tools/derived_model.py line 411, AFTER correctly running and
  passing all four of MY model's gates.
- minimal repro: any DerivedModel subclass whose `evaluate()` does not return a value named
  `overshoot_pct`; `DerivedModel.main()` routes `--selftest` to the module-level `selftest()`,
  which asserts on the DampedOscillator exemplar's own output keys.
- expected vs actual: the docstring advertises `--selftest` as "gates + envelope + determinism"
  for the scaffold generally; it only works for the one exemplar.
- workaround used: gate verification is read from the real run's receipt instead
  (receipts/jet_force_receipt.json -> residual_or_convergence.gates, 4/4 passed, and
  self_check.passed true). No claim depends on --selftest.

## F5 — ace_modal: no memory-lean eigensolver path; 360k dof needs >10 GB (2026-08-07)
- symptom: ace_modal on the full runner at voxel 0.75 mm (84575 active elements, ~360k dof)
  reached 9.8 GB RSS and 50% CPU after 25 min wall / 7.6 min CPU — i.e. it was swapping, on a
  24 GB machine shared by 6 agents. Killed. At 1.1 mm (111k dof) the same job takes 64 s.
- minimal repro: energy_system/turgo_runner/programs/modal_075.json (kept in the campaign).
- expected vs actual: ACE fea.py line 976 uses `spla.eigsh(A, k, sigma=0.0, which="LM")`, whose
  shift-invert needs a full sparse LU of K. There is no job key to request a factorization-free
  path (LOBPCG / `which="SA"`, which the same file has at line 984 as a fallback only after
  eigsh raises) and no key to cap memory. cookbook 1 documents `direct_solver_max_dof` for
  ace_fea but ace_modal has no equivalent.
- workaround used: a 3-point convergence ladder at 1.1 / 0.9 mm on the FULL part plus a 0.45 mm
  SUB-MODEL of one bucket + its hub patch, which puts 4.9 elements across the 2.2 mm cup wall —
  the resolution DESIGN.md 5 row 2 asked for — on the feature the gate is actually about. The
  0.55 mm full-part grid is still pinned (physics/turgo_055.npy) for the ace_fea path.

## F6 — `clearance`: `distance` reads 0.0 for an ENCLOSED, non-touching pair (2026-08-08)
- symptom: a solid fully inside a tube, nowhere near it, reports
  `{"distance": 0.0, "interfering": false, "overlap_volume": 0.0}`. The boolean verdict is
  right; the NUMBER a free-motion receipt would quote is wrong (it should be the annular gap).
  Verbatim, from the repro below:
  `c1 {"coincident_fit_hazard": false, "distance": 0.0, "interfering": false, "overlap_volume": 0.0, "provenance": "faceted"}`
  while the same `inner` against a disjoint side-by-side cylinder correctly gives
  `c2 {... "distance": 25.0 ...}`.
- minimal repro (2 s, no campaign files):
  ```json
  {"ops":[
   {"id":"inner","op":"cylinder","base":[0,0,0],"axis":[0,0,1],"radius":10.0,"height":5.0},
   {"id":"to","op":"cylinder","base":[0,0,-1],"axis":[0,0,1],"radius":23.0,"height":8.0},
   {"id":"ti","op":"cylinder","base":[0,0,-2],"axis":[0,0,1],"radius":20.0,"height":10.0},
   {"id":"tube","op":"difference","a":"to","b":"ti"},
   {"id":"c1","op":"clearance","a":"inner","b":"tube"},
   {"id":"solid_far","op":"cylinder","base":[40,0,0],"axis":[0,0,1],"radius":5.0,"height":5.0},
   {"id":"c2","op":"clearance","a":"inner","b":"solid_far"}]}
  ```
  `"/Users/himanshu/Work/New-LMCAD/cad engine/target/release/kernel-api" run repro.json --out-dir /tmp/out`
  Expected `c1.distance == 10.0` (bore 20.0 − part 10.0); actual `0.0`.
- expected vs actual: API.md / ops_core describe `clearance` as surface separation with
  `overlap_volume` for the interference branch. Nothing says `distance` collapses to 0 when one
  body's bbox is contained in the other's. A campaign that quoted `min_distance` from a
  containment sweep would ship a false "0.0 mm clearance" number with a green `ok`.
- workaround used: the 360 deg housing sweep keeps the casing TUBE watch as the CONTAINMENT
  gate (`overlap_volume`/`interfering` are correct) and adds `clr_case_feeler` — 12 D2.0 pins
  whose inner surface lies exactly on the casing bore R53.0 — as the witness that yields a true
  separation number. Both watches ship; programs/gen_stage4.py records the reason inline.

## F7 — `import_step` cannot survive template-materialising tools (sweep_check, dim_suggest) (2026-08-08)
- symptom: every station of a sweep_check job failed with
  `op 'part_raw': cannot read '/var/folders/t0/1mzcwf550j5bjb7ntwp817zr0000gn/T/roundtrip_source.step': No such file or directory (os error 2)`
  — 73/73 stations, `"stations": 0` in every watch. dim_suggest.py fails identically on the same
  program (`ValueError: program failed at op 'rt'`).
- minimal repro: any sweep_check/dim_suggest job whose template contains
  `{"op":"import_step","file":"something.step"}` with the .step file next to the REAL program.
  `python3 tools/sweep_check.py programs/sweep_rotation.json` (pre-fix version).
- expected vs actual: `import_step`'s `file` resolves against the PROGRAM FILE's directory
  (same rule OPERATOR_BRIEF 3 gives for `load_part.file`), but both tools route through
  `param_optimize.call_engine`, which writes the substituted program to
  `tempfile.NamedTemporaryFile(suffix=".json")` — so the program's directory becomes `$TMPDIR`
  and every relative geometry input is orphaned. Passing an absolute path instead is refused
  outright: `path '...' must be relative to the sandbox (absolute paths ...)`. The two rules
  together leave NO way to feed an existing STEP/part file into a swept template.
- workaround used: the sweep template no longer imports anything — programs/gen_stage4.py
  imports gen_part.build() and inlines the 47 geometry ops (trimmed at the `part` id), so the
  station program is self-contained. Cost: the full part is rebuilt at every station (~21 s),
  turning a 4 min sweep into ~25 min. dim_suggest got the same treatment via
  programs/dims_source.json (part_program.json minus the export/import tail).

## F8 — the documented `runner.py job | tail -1 > receipt.json` idiom DESTROYS a good receipt when a solve is interrupted (2026-08-08)
- symptom: `tools_cookbook.md` prescribes capturing ACE-runner receipts as
  `python3 tools/ace_modal_runner.py job.json | tail -1 > receipts/x_receipt.json`.
  The shell truncates the target at LAUNCH and `tail` writes only at EOF, so a solve that is
  starved, killed or OOMed leaves a **ZERO-BYTE receipt where a valid one used to be**. Observed
  here: `receipts/modal_090_receipt.json` went 6 825 bytes -> 0 bytes the moment the re-run
  started, and stayed 0 for the ~40 min the eigensolve was starved (load average 77-91 from six
  concurrent agents; the process accumulated 2:24 of CPU in ~20 min of wall clock).
- minimal repro: `python3 tools/ace_modal_runner.py programs/modal_090.json | tail -1 > r.json`
  then Ctrl-C after a few seconds. `r.json` is 0 bytes.
- expected vs actual: the cookbook presents the idiom as the standard capture pattern and the
  ACE-runner contract is "exit 0 even on failure — parse `ok`". Neither warns that the redirect
  is destructive before the tool has produced anything. A campaign that re-runs its physics as
  part of a final self-check can therefore END WORSE THAN IT STARTED, and a `-s` (non-empty)
  test is the only thing standing between that and a silently truncated deliverable.
- workaround used: never redirect over a good receipt. Write to a TEMP path and copy in only
  after a non-empty, `ok:true` parse. The starved rung's receipt was restored from a
  pre-run backup and its provenance re-proved rather than assumed: the density grid was
  re-voxelized from the shipped STL in this session and
  `tools/provenance.geometry_hash(density_path=physics/modal_090/voxelized.npy)` reproduces
  `density:sha256:6417c816b9f74239d228282e223463f24c79b8694ea84b8400cd61fa8b544995` — byte-for-byte
  the `geometry_hash` inside the carried-forward receipt. `programs/rebuild.sh` still uses the
  cookbook idiom (it is the documented one); the hazard is recorded here and in README.

## F9 — `assert` accepts only 8 checks, so four DELIVERABLE_SPEC §2 gates cannot be in-program gates (2026-08-08, independent verification pass)
- symptom: adding `{"id":"gate_supports","op":"assert","in":"part","steep_area":0.0}` to
  `programs/part_program.json` returns, verbatim:
  `unknown param 'steep_area' — 'assert' does not accept it, so it was IGNORED and the default
  (if any) stayed in force; call describe {"name":"assert"} for the accepted params`
  followed by `{'kind': 'invalid_param', 'message': "op 'gate_supports': assert has no checks —
  give at least one of volume_within / exact_volume_within / genus / shells / components /
  closed / manifold / valid"}`.
- minimal repro: append the op above to any program with a `support_report` and run
  `kernel-api run prog.json --out-dir /tmp/o`.
- expected vs actual: DELIVERABLE_SPEC §2.4 ("gate the receipt": `watertight`/`route`), §2.5
  (`steep_area == 0.0` exactly), §2.6 (`thin_area`/`p05_thickness`) and §2.7 (`fits_within`
  true) all read as gates, and §2's preamble says "a recorded-but-unchecked measure is
  worthless" — but the kernel has no assert predicate for any of them. Those four gates can
  therefore only ever be *recorded*: a regression that filled the part with steep overhangs,
  broke watertightness, or overflowed the 256 mm envelope would still exit 0 and still print
  `DONE` from `programs/rebuild.sh`.
- workaround used (verification only, no source touched): the four measures were read out of
  `receipts/part_report.json` by hand and cross-checked. A campaign-side workaround would be a
  post-run python check in `rebuild.sh` (it already does this for `ok` and `warnings`); the
  engine-side fix is an `assert` that accepts the support/wall/bbox/export measures.

## F10 — `ace_fea_tet` (superlu_direct) is not bit-deterministic, so derived receipts cannot regenerate byte-identically (2026-08-08, independent verification pass)
- symptom: re-running the shipped stall job reproduces the peak to 1.6e-15 relative but not to
  the bit: `max_von_mises_pa` = `12853964.502047122` on the re-run vs `12853964.502047101` in
  `receipts/tet_stall_b_receipt.json` (and `12853964.502047331` in the run that produced the
  shipped `receipts/refusals.json`). Mesh volume is bit-identical (`1254.0821906354504`).
- minimal repro: `python3 tools/ace_fea_tet_runner.py programs/tet_stall_b.json | tail -1`
  twice; diff the `max_von_mises_pa` field.
- expected vs actual: DELIVERABLE_SPEC §3 requires committed STLs/PNGs to regenerate
  byte-identical (they do — verified) but says nothing about solver receipts. Anything that
  *derives* from a tet stress (here `programs/gen_refusals.py` → `receipts/refusals.json`,
  which carries `sigma_stall_mpa`, `sigma_bep_mpa`, `cycles_to_failure`, `damage`) therefore
  cannot be `cmp`-checked; only a tolerance diff is meaningful. Observed drift is ~2e-14
  relative, i.e. physically irrelevant, but it makes `cmp` the wrong reproducibility oracle for
  that class of receipt.
- workaround used: compared `receipts/refusals.json` field-by-field with a numeric tolerance
  instead of `cmp`; all 12 differing fields agree to <2e-13 relative.

## F11 — `shells == 1` with `components >= 2` is not constructible, and `mesh_components` is the WEAKER of the two (2026-08-08, repair pass)

- symptom: `DELIVERABLE_SPEC` §2.2 tells every campaign to prove the
  connectivity gate on "a split-body variant that the connectivity gate
  catches while `shells` still reads 1". That variant cannot be built on this
  kernel, and the real blind spot runs the opposite way.

  Two facts, both measured:

  1. **A valid solid cannot have `shells 1` and `components >= 2`.** A closed
     2-manifold surface shell bounds exactly one connected volume, and every
     construction that would break that link is refused before it can be
     measured. Boxes sharing exactly one edge:
     `union failed validate(): closed=false manifold=false genus=2
     euler_characteristic=1 shells=2 — refusing to bind an invalid solid`.
     Boxes sharing exactly one vertex:
     `union_all failed validate(): closed=true manifold=false genus=1
     euler_characteristic=3 shells=2 — refusing to bind an invalid solid`.
     Boxes sharing a full face fuse into one body (`shells 1`,
     `components 1`). So `shells` never under-reads relative to `components`
     for anything `validate` will accept.

  2. **`mesh_components` under-reads instead.** It runs on the faceted mesh
     and welds at `weld_tol` 0.001 mm, so any sever narrower than that is
     welded shut. On the real shipped part, the identical sever plane at two
     gap widths:

     | gap | `validate` → `shells` | `mesh_components` |
     |---|---|---|
     | 0.25 mm | 2 | `components 2`, `is_one_body false` |
     | **0.0005 mm** | **2** | **`components 1`, `is_one_body true`** |

     i.e. at a sub-weld-tol sever the *connectivity* gate reports the part is
     one body and the *topology* gate is the only one that fires.

- minimal repro (toy, ~1 s):
  ```json
  {"ops":[{"id":"a","op":"box","min":[0,0,0],"max":[10,10,10]},
          {"id":"b","op":"box","min":[10.0005,0,0],"max":[20,10,10]},
          {"id":"u","op":"union_all","in":["a","b"]},
          {"id":"v","op":"validate","in":"u"},
          {"id":"c","op":"mesh_components","in":"u"}]}
  ```
  `kernel-api run p.json --out-dir .` → `v: shells 2`, `c: components 1,
  is_one_body true, weld_tol 0.001`. Campaign repro on the real geometry:
  `programs/nc_shells_vs_components.json` →
  `receipts/nc_shells_vs_components_report.json` (exits 1).

- expected vs actual: DELIVERABLE_SPEC §2.2 says "`shells==1` does NOT prove
  it — a severed part passes every other validity gate", and offers the
  shells-blind variant as the oracle to build. On this binary a severed part
  does NOT pass the shells gate; the gate that a severed part can slip past is
  `components`, whenever the sever is thinner than `weld_tol`. The spec's
  example is therefore unbuildable and its stated rationale is inverted. Both
  gates are still needed — they are non-redundant in the direction opposite to
  the one the spec describes.

- workaround used: none needed for the part. The campaign ships
  `programs/nc_shells_vs_components.json` as a second single-body oracle (the
  same blade, the same sever plane, 0.0005 mm gap, `assert shells:1`, exits 1)
  alongside `nc_connectivity.json`, and `analysis/ANALYSIS.md` §10a prints the
  measured 2×2 table. README and ANALYSIS previously repeated the spec's
  framing as if it had been observed; that claim is retracted in both
  documents and in `analysis/DESIGN.md` A19. Suggested spec change: replace
  the §2.2 parenthetical with the `weld_tol` case, or expose `weld_tol` as a
  `mesh_components` parameter so a campaign can tighten it.

## F12 — the `ace_fea_tet` rigid-clamp peak does not converge, and nothing in the tooling says so at the point of use (2026-08-08, repair pass)

- symptom: `tools/ace_fea_tet_runner.py`'s docstring warns that "the reported
  peak is still nodal-recovered and mesh-dependent — refine elem_size_mm to
  confirm convergence", but the receipt itself carries no convergence field,
  no warning, and no indication of where the peak sits relative to the
  fixtures. A campaign that reads only the receipt has no signal at all. Here
  the peak sits 0.224 mm from a rigid clamp plane, i.e. it is a
  boundary-condition singularity, and refining it does not converge it:

  | `elem_size_mm` | tets | nodes | max von Mises |
  |---|---|---|---|
  | 0.85 | 11 498 | 19 540 | 12.854 MPa |
  | 0.67 | 21 284 | 34 758 | **14.087 MPa (+9.6 %)** |

  Excluding the clamp boundary layer, the same field is converged to a few
  percent (at +1.0 mm: 8.824 → 8.986 MPa, +1.8 %). So the *physics* is fine
  and the *headline number* is not a bound.

- minimal repro: `programs/tet_stall_b.json` and `programs/tet_stall_b_fine.json`
  (identical but for `elem_size_mm`), `python3 tools/ace_fea_tet_runner.py
  <job> | tail -1`. ~25 s and ~45 s respectively.

- expected vs actual: a `max_von_mises_pa` next to `validation_status`
  Validated reads as a converged quantity. It is not one when a `clamped`
  fixture creates a re-entrant or edge singularity — which is the normal case
  for a sub-model. Suggestion: have the runner report the peak node's distance
  to the nearest fixture region, and/or a `peak_on_fixture_boundary: true`
  flag, so the artefact is visible in the receipt rather than only to a
  campaign that goes looking with its own probe script.

- workaround used: shipped the two-point study
  (`receipts/tet_stall_b_fine_receipt.json`, matched-exclusion comparison in
  `receipts/tet_stall_b_field_probe.json` → `mesh_convergence`), ran
  `production_check` on BOTH readings, and stated in README and ANALYSIS §3
  that the quoted 12.854 MPa is a mesh-dependent reading and not an upper
  bound. No number was revised downward.
