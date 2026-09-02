# ENGINE FIX REPORT — integration of the six-owner fix pass

**Date:** 2026-08-08 · **Role:** integrator (owner G) · **Scope:** prove the tree is
healthy after owners A–F landed changes in parallel, and that nothing regressed for the
10 shipped campaigns or the showcase.

**Headline.** The tree is green, and it was not green when I started. The parallel pass
introduced one severe regression — **11 of 30 shipped part programs across 8 of the 10
campaigns exited 1** — plus a broken Rust test, two clippy warnings and a stale doc
contract. All are fixed, root-caused to general defects rather than patched at the
symptom, and pinned by tests that fail without the fix. Every committed STL, 3MF and
print file in the portfolio now rebuilds **byte-identical**.

---

## 1. Verification evidence

| gate | result |
|---|---|
| `cargo build --workspace --release` | clean, 0 warnings |
| `cargo test --workspace --release` | **160 suites / 1073 tests, 0 failed** (was 1 failed) |
| `cargo clippy --workspace --all-targets --release` | **0 warnings** (was 2) |
| `python3 tools/gen_discover.py` then compare | **regeneration changed nothing** — the generated table is in step with `program.rs` |
| `python3 tools/audit_docs.py` | **exit 0** — 18 findings, 0 at or above `error`; `[path] [section] [symbol] [claim]` passes all clean |
| Python contract gates (12 scripts) | **12/12 exit 0** (was 11/12) |
| **Campaign regression: 30 part programs, 39 committed artefacts** | **39/39 byte-identical, 0 warnings, 0 non-zero exits** |
| Negative controls (30 NC/oracle/refusal programs) | every severance oracle still fires `assert_failed`; `ball/nc_oracle.json` still exits 1 as its README requires |
| Representative physics job (din_rail `fea_foot_lc1`) | receipt **purely additive**; `max_von_mises_pa` 592071.2438111951 and `geometry_hash` bit-identical to the shipped receipt |
| Representative tolerance stacks (turgo ×2, din_rail ×2) | additive; the one intended numeric correction is present and no verdict flips |

### Per-part regression detail (all `exit=0 ok=true 0 warnings`)

| campaign | programs | committed artefacts rebuilt | result |
|---|---|---|---|
| `aerospace_system/cubesat_1u_dev_frame` | 2 | `cubesat_1u_dev_frame.stl/.3mf`, `gauge_pc104_board.stl`, `gauge_pocket_section.stl` | identical |
| `biomedical_system/prosthetic_wrist_quick_disconnect` | 3 | `insert.stl`, `receptacle.stl`, `slider.stl` | identical |
| `robotics_system/iso9409_wedge_flexure_gripper` | 4 | `crank.stl`, `flange_adapter.stl`, `palm.stl`, `wedge.stl` | identical |
| `electronics_system/din_rail_pi4_enclosure` | 3 | `din_foot.stl`, `latch_bolt.stl`, `base_shell.stl`, `lid.stl`, `th35_gauge.stl`, `lid_blocked_louver_CONTROL.stl` | identical |
| `automotive_system/rotor_runout_gauge_bridge` | 2 | `rotor_runout_gauge_bridge.stl`, `..._carriage.stl` | identical |
| `marine_system/folding_deck_cleat` | 2 | `folding_deck_cleat_base.stl`, `..._horn.stl` | identical |
| `optics_system/ball_kinematic_mirror_mount` | 2 | `frame.stl/.3mf`, `platform.stl/.3mf` | identical |
| `energy_system/turgo_runner` | 1 | `turgo_runner.stl/.3mf` | identical |
| `agriculture_system/jar_top_seed_singulator` | 7 | 7 print STLs | identical |
| `acoustics_system/screw_on_exponential_horn` | 1 | `horn_body.stl/.3mf` | identical |
| `showcase/squatchee_spin` | 3 | `squatchee_spin_mount/prop/retainer.stl` | identical |

Run twice end to end against the final binary — the same 30/39/0/0 both times, so the
result is not an artefact of one build.

Method: each campaign tree is copied to scratch and rebuilt there, so no shipped
deliverable is ever written. The produced artefact is **deleted from the copy before the
run**, so a comparison can never pass against a file that was not actually rebuilt —
`kernel-api` aborts at the first failing op, and my first harness pass compared stale
copies for exactly that reason. That flaw is worth recording: it briefly reported the
regression below as "byte-identical", i.e. the harness itself was silently degrading.

---

## 2. What was fixed

### 2.1 THE REGRESSION — `Mesh::boundary_edge_count()` counted NON-ORIENTABLE edges as boundary edges

**Symptom.** 11 of 30 shipped part programs exited 1 with `invalid_geometry`:
wrist `part_insert` + `part_receptacle`, gripper `palm`, din_rail `part_program`, cleat
`part_base`, ball `part_frame` + `part_platform`, turgo `part_program`, singulator
`part_driver_crank` + `part_geneva_wheel` + `part_housing_top`. Owner A's T6 trust guard
refused the connectivity count, reporting "the faceter dropped geometry". In every case
the count it would have reported was **1** — the passing value. Owner A's compat note
("every case that trips the guard read `components > 1` before, so those gates already
failed") does not hold; these campaigns shipped with 0 warnings and passing gates.

**General defect (not the guard).** The guard's premise is right: a connectivity count on
a surface that is not closed counts cracks. It was fed a wrong closure test.
`Mesh::boundary_edge_count()` asked *"is the reverse directed edge absent?"*. Two
triangles that share an edge but wind the **same** way close that edge perfectly — no rim,
nothing to fill — yet `b→a` is absent, so every one of them was reported as a boundary
edge. `kernel_core::meshcheck::edge_report` has always separated **boundary** (undirected
edge used once) from **non-orientable** (used twice, same direction). Two oracles for one
property, disagreeing by construction, on exactly the meshes where the answer matters —
the same class owner A filed for the self-intersection pair.

**Reproduction (smallest).** The din_rail lid chain `lid_pl … lid_dl3` (a union of
plate/skirts/tongues/key, then three rotated louver cutters; the `lid_key` tab is
required). 176 triangles. `validate` on the bound export mesh: `boundary_edges: 0`,
`non_orientable_edges: 3`, `closed`. `mesh_components` on the solid: refused, "left 3
boundary edges". Bisected from the full program with a per-op probe harness.

**Fix.** `crates/kernel-core/src/mesh/mod.rs`:
- `directed_edges()` — one pass producing per-directed-edge use counts plus the edges in
  triangle order (so boundary walks are deterministic rather than hash-seeded).
- `is_boundary_edge()` — an undirected edge is a boundary edge **iff it is used exactly
  once**. `boundary_edge_count()` and `largest_boundary_loop()` both use it.
- `fill_holes()` uses the same rule. It previously fanned a cap onto an already-closed
  edge, promoting a winding defect into a non-manifold one (a third triangle on an edge
  that already had two). The repair now cannot disagree with the measure it exists to
  drive to zero.
- New `non_orientable_edge_count()` so the two defects can be reported apart without
  paying for `check_mesh`'s self-intersection sweep.

**Test that fails without it.** `crates/kernel-core/tests/boundary_edge_oracle.rs`
(4 tests): a closed tetrahedron with 0–4 faces wound inside-out has no opening at any
count; a real opening is still counted; `Mesh::boundary_edge_count` /
`non_orientable_edge_count` agree with `check_mesh` on every shape in a battery;
`fill_holes` invents no geometry on a closed surface. **Negative control run:** with the
old rule restored, 3 of the 4 fail (`left: 3, right: 0`).

**Also fixed downstream, so the receipts stop contradicting themselves:**
- `mesh_components` now reports `non_orientable_edges` beside `boundary_edges`. Without
  it, `watertight: false` next to `boundary_edges: 0` is unexplained.
- The guard now only fires on a genuine opening, and says so in the source comment and in
  API.md, so the next reader does not re-introduce the conflation.

### 2.2 `validate` on a bound mesh contradicted itself

`closed` was `check_mesh().watertight`, which folds in orientability **and** bowtie
vertices, so a mesh with a flipped triangle reported `closed: false` next to
`boundary_edges: 0` in the same receipt. Now `closed` is closure and nothing else;
`manifold` carries orientability + non-manifold edges + non-manifold vertices.
`valid` (= closed AND manifold) is arithmetically **unchanged** — everything the old
`closed` covered still gates, just under the right name. `non_manifold_vertices` added
to the receipt. This path is new this pass (owner A's `EnvValue::Mesh`), so no campaign
can depend on the old spelling.

### 2.3 `export_stl` / `export_3mf` claimed a watertightness it does not test

`export_stl` reports `watertight: true`, and DELIVERABLE_SPEC §2.4 makes that a mandatory
gate. It is `Mesh::is_watertight()` = edge closure only — it does **not** test
orientability. **Measured across the portfolio: 26 of 41 export ops write files carrying
3–395 non-orientable edges while reporting `route: "exact", watertight: true`** — every
campaign except cubesat and rotor, and the shipped showcase. The bytes are exactly what
shipped; the property was simply never reported.

Fix is additive and changes no value: the export receipt now also carries
`watertight_means` (a one-line statement of which sense is meant), `boundary_edges`,
`non_orientable_edges` and `two_manifold`. A campaign that needs consistent normals can
gate `two_manifold`; one that needs printability keeps gating `watertight`. API.md
`export_stl` and DELIVERABLE_SPEC §2.4 both rewritten to say which sense is meant and to
carry the 26/41 census, so a future campaign gates it knowingly rather than assuming.
**The winding defect itself is NOT fixed — see §5.**

### 2.4 `studio-server` pinned an API.md documentation GAP by name

`apidoc::tests::extracts_real_sections_and_reports_missing_ones_honestly` asserted
`extract_section(&md, "clearance").is_none()` as "the honest known-missing case". Owner A
then documented `clearance` — the improvement the auditor asks for — and the test broke.
The op name was never the contract. Replaced with the **biconditional over the whole live
`describe` surface**: `extract_section` returns a section starting with the op's heading
iff API.md carries one, for every op, with the expectation computed by an independent
`"### \`op\`"` line-prefix scan (deliberately not reusing the module's fence/depth
machinery). The `None` branch stays exercised by whatever is genuinely undocumented,
without naming it, so closing the last gap can never fail the test again.

### 2.5 `docs/test_doc_contracts.py` pinned `assert`'s parameter set by enumeration

Owner F's contract asserted `assert` has **no** `tol`/`weld_tol`. Owner A then added both
— which is precisely what owner F's own cross-owner request asked for. The contract failed
for being right about yesterday. Same defect class as 2.4: an enumeration is not a
property, and it breaks on every additive parameter. Repinned on what the spec actually
promises: the gate exposes the knobs, its **defaults equal the measure's defaults**, a
plain `assert components: 1` still passes, and a tightened one **fails** the 0.0005 mm
sever the default passes (`assert_failed`, exit 1). `campaign/DELIVERABLE_SPEC.md` §2.2
rewritten to match, including the connectivity oracle's closed-surface requirement and the
boundary-vs-winding distinction.

### 2.6 `tools/nightly.sh` watched none of the Python contracts

The repo has a nightly self-exercise. It ran `cargo test`, clippy and two campaign
examples. It ran **zero** of the tools/ contracts — so the ~150 executable pins the six
owners just wrote (checker pins, aux-tool pins, doc contracts, the 540-probe cross-language
creep vectors, the 31-gate runner exit contract, the doc-drift audit) were unwatched, and a
regression there would surface only when a campaign tripped over it. That is the same
silence this repo forbids in its tools, one level up.

Added, **discovered rather than enumerated** so a suite added tomorrow is watched tomorrow
without editing the file: every `tools/test_*.py` and `docs/test_*.py`, plus the gates that
take arguments (`audit_docs.py`, `materials_crosslang_test.py`, `analyzer_registry.py
--check` and `--check-contract`, `materials.py --selftest`, `production_check.py
--selftest`). Plus a new step asserting `discover.rs` regenerates unchanged from
`program.rs` — hand-editing one without the other is invisible to every other gate.
Report table, failing-gate block and `history.jsonl` fields updated. Verified under `sh`:
12 gates discovered, 12 exit 0.

### 2.7 `field_triage.py --self-test` reported a missing FIXTURE as 11 failed checks

`spool_system/respool/analysis/ANALYSIS.md` is absent from the tree, so every
claim-parsing check returned 0 claims and was reported **FAILED**. Owner B correctly
identified this as pre-existing and out of their theme, but the general defect is worse
than filed: three of those checks assert an **empty** result ("contradicts NOTHING"), so a
missing fixture made them pass **vacuously**. A suite that is permanently red is a suite
nobody can gate on, and its noise hides a real regression.

Fix: checks declare `needs=<fixture>`; a check whose fixture is absent is **SKIPPED**, not
failed. `_report` now distinguishes three outcomes — PASS (exit 0), `self_test_failed`
(exit 1), and `fixture_missing` (exit 1, "nothing failed; nothing was proved either", with
the missing paths and the skipped list). Result: **28 passed, 0 failed, 13 skipped** (was
30 passed, 11 failed). All 12 of owner B's creep checks pass. Negative control run: a
deliberately broken assertion still yields `error_kind: self_test_failed`.

### 2.8 Two clippy warnings

`clippy::type_complexity` on my own `directed_edges` return type (fixed with a
`DirectedEdgeUses` alias that also documents the 1-vs-2-vs-non-orientable rule), and
`clippy::neg_cmp_op_on_partial_ord` in owner B's `materials_creep_crosslang.rs`. The
latter's `!(x < 1e-12)` was buying NaN-safety; rewritten as
`delta.is_nan() || delta.abs() >= 1e-12` so the intent is explicit and a NaN allowable
still counts as a mismatch rather than agreement.

### 2.9 Cross-owner requests closed (files no single owner could touch)

- **`studio/mcp/src/lib.rs`** — the `production_check` MCP tool exposed no `duration_h`,
  so after owner B's T14 fix **every** MCP caller with `sustained: true` would receive
  `refusal.creep_duration_required` with no way to answer. Added the property with the
  full contract in its description (the table semantics, the round-UP rule, the three
  refusal kinds, the 25 °C-reads-the-55 °C-row trap) and corrected the module header table,
  which still described the superseded `yield × creep_sustained_fraction` rule.
- **`tools/solvers/README.md`** — added the `creep` row for owner B's new card; replaced
  the exit-code note (still said "the ACE-bridge runners exit 0 even on failure") with
  owner C's 0/1/2 contract and the `LMCAD_RUNNER_EXIT=legacy` opt-out; added a note that
  the `status` column is a gate-suite status, not a tier, pointing at
  `analyzer_registry.py --tier`.
- **`tools/analyzer_registry.py`** — the last catalogue-drift warning (`audit_docs.py`)
  plus five other unclassified tools declared in `NON_ANALYSIS` with reasons.
  `audit_docs.py` analyses **documents**: it computes no physical quantity and has no
  manifest or pin; it was caught only because its filename contains "audit". Drift
  warning is now empty.

---

## 3. Refuted / corrected claims from the owner reports

- **Owner A, T6(b)/(c) compat: "No pass→fail flips: every case that trips the guard read
  `components > 1` before, so those gates already failed."** Refuted by measurement. All
  11 tripping programs measured `components: 1` and had shipped green. The sweep of "the
  repo's 251 real programs" evidently did not execute the part programs; running them is
  the check that catches this.
- **Owner A, T6(b)/(c) root cause: "a planar face carrying inner/hole loops is the known
  case."** True for the `components > 1` cases they reproduced, but it is not what fired
  on the campaigns. Those had `boundary_edges` 3–258 with `components: 1` — a *winding*
  defect, not a dropped hole loop. The tessellation fix owner A filed as a cross-owner
  request remains valid and remains open; it is a different phenomenon from the regression.
- **Owner B's count of affected `production_check` jobs (12 across 7 campaigns).** The
  measured figure is **15 across 8** — the gripper's `production_check_palm_wrench.json`
  and cubesat's `a6_production_check_creep.json` were missed, and cleat has 3 rather than
  the implied count. Nothing about the fix changes; the disclosure was simply short.
- **Owner B: "`field_triage.py --self-test` is RED before and after, and the failing set
  is byte-identical."** The observation is right and the baseline discipline was right. The
  conclusion that it was not fixable in their paths is wrong — the fix is in
  `field_triage.py`, which they owned: report the missing fixture as a fixture, not as
  eleven engineering failures. Also, three of the "failing" checks were in fact passing
  vacuously, so the byte-identical failing set was not the whole story.
- **Owner F, DELIVERABLE_SPEC §2.2 "`assert` has no `tol`/`weld_tol`."** True when
  written, false by the end of the pass. Both the doc and its executable contract are
  updated. Owner F predicted this handoff explicitly and it is the reason
  `docs/test_doc_contracts.py` earned its keep — it named the stale section for me.
- **My own error, recorded so a successor does not repeat it:** my first regression
  harness copied each campaign tree *including* `parts/`, so a program that aborted before
  its export was compared against the shipped file it had never overwritten, and reported
  "identical". It briefly hid an 11-program regression behind a green result. Any rebuild
  check must delete the artefact before rebuilding it.

---

## 4. Backward-compatibility impact

**Byte-level: none.** All 39 committed STL/3MF artefacts across the 10 campaigns and the
showcase rebuild byte-identical. No geometry, tessellation or export path changed.

**Receipt shapes: additive only.** No key was removed or renamed anywhere.

| surface | change | risk |
|---|---|---|
| `export_stl` / `export_3mf` measures | **+** `watertight_means`, `boundary_edges`, `non_orientable_edges`, `two_manifold` | none — `route`, `triangles`, `watertight` unchanged in name and value |
| `mesh_components` measures | **+** `non_orientable_edges` | none |
| `validate` on a bound **mesh** | `closed` now means closure only; `manifold` now also covers bowtie vertices; **+** `non_manifold_vertices` | none — the path is new this pass; `valid` is arithmetically identical |
| `mesh_components` / `assert components` on a solid | **stops refusing** a closed-but-non-orientable tessellation | fail→pass only, and only where the refusal was wrong. A genuine opening still refuses; all 30 negative controls still fire |
| `Mesh::fill_holes` | no longer caps an already-closed edge | changes output only on a mesh with non-orientable edges, where the old output was a non-manifold edge |
| MCP `production_check` | **+** `duration_h` input property | none — required only when `sustained: true`, which currently refuses outright without it |

**The behaviour changes the owners declared loudly stand and are confirmed here**: ACE
runner exit codes now 0/1/2 (`analyzer_registry --check-contract` 31/31), shipped
`production_check` jobs will refuse until a `duration_h` is added — **the count is 15 jobs
across 8 campaigns, not the 12 across 7 owner B reported**; the scan is
`load_character.sustained == true` with no `duration_h`, `service.duration_h` or
`load_character.duration_h` anywhere in the job (ball ×2, wrist ×2, cleat ×3, horn ×4,
cubesat ×1, turgo ×1, singulator ×1, gripper ×1) —
`singulator/buckling_neck.json` and `cleat/refusal_buckling_horn_tension.json` now refuse
as tensile load cases, and `tolerance_stack` corrects the asymmetric worst-case band. I
verified the last one end to end: `turgo/tol_2_nut` moves `nominal_gap` 0.45 → 0.30 and the
worst band [0.30, 0.90] → [0.15, 0.75] with **no verdict flip**, while the symmetric
`din_rail/tol_i1_nose_engagement` is leaf-for-leaf identical apart from additive fields.

---

## 5. Still open

1. **The winding defect itself (26 of 41 shipped exports).** The adaptive tessellation
   emits triangles wound inside-out on boolean results — 3 to 395 non-orientable edges per
   file. It is now on every export receipt, and the STLs are byte-identical to what
   shipped, so nothing regressed; but a file reported `watertight: true` genuinely is not a
   consistently-oriented 2-manifold. Not fixed here **because fixing it changes the bytes
   of 26 shipped print files**, which is exactly the red flag this pass was told to guard.
   It needs a deliberate re-baseline: fix `crates/kernel-brep` winding, regenerate all
   print STLs, re-run every campaign. Owner of that decision is the maintainer, not this
   pass.
2. **The tessellation defect owner A root-caused** (`tessellate_adaptive.rs::face_boundary`
   ignoring `Face::inner` hole loops; `tessellate.rs` self-overlapping hole-bridge
   triangles). Still open, still correctly diagnosed, and now clearly separable from the
   regression above. It is what makes a STEP-round-tripped pocketed solid read
   `boundary_edges: 1576, components: 30` — reproduced here on
   `din_rail/part_program.json:base_rt`.
3. **`union_all` non-termination** at n≈13 mostly-disjoint bodies (owner A, bisected,
   documented in API.md). Untouched.
4. **`spool_system/respool/analysis/ANALYSIS.md` is missing.** 13 `field_triage` checks
   cannot run. Now named rather than mis-reported. Deliberately **not** wired into
   `nightly.sh`: a permanently-red gate is a gate nobody reads. Restore the file (or
   repoint the fixture) and add it.
5. **15 ops have no `### <op>` heading in API.md** (`sample_density_grid`,
   `mesh_density_grid`, the 11 `asm_*`, `gear_train_poses`). These are the audit's 15 INFO
   findings. The `studio-server` test now measures the gap structurally instead of pinning
   one name, so writing them is a pure improvement with no test to update.
6. **Campaign-side follow-through.** Four campaign READMEs still restate the retracted
   "shells==1 does not prove single-bodyness" claim; `singulator`'s README quotes a
   buckling number the runner now refuses; two shipped tolerance receipts predate the
   asymmetric-band correction; two campaigns carry workarounds for the parity-fill defect
   owner E fixed. The `*_system/` trees are shipped deliverables and were not touched.

---

## 6. Files changed by this pass

```
crates/kernel-core/src/mesh/mod.rs          boundary/orientability oracles + fill_holes
crates/kernel-core/tests/boundary_edge_oracle.rs   NEW — 4 pinning tests
crates/kernel-api/src/interp.rs             validate-on-mesh, mesh_components, export measures
crates/kernel-model/tests/materials_creep_crosslang.rs   clippy + explicit NaN handling
studio/server/src/apidoc.rs                 biconditional doc-coverage contract
studio/mcp/src/lib.rs                       production_check duration_h + creep contract
tools/nightly.sh                            python contract gates + discover.rs in-step gate
tools/field_triage.py                       fixture_missing vs self_test_failed
tools/analyzer_registry.py                  NON_ANALYSIS classifications
tools/solvers/README.md                     creep row, exit contract, tier note
docs/test_doc_contracts.py                  repinned assert tol/weld_tol contract
API.md                                      export_stl + mesh_components sections
campaign/DELIVERABLE_SPEC.md                §2.2 assert knobs + closed-surface rule;
                                            §2.4 what `watertight` actually tests + census
campaign/fixlog/G-integrator.md             working log
```

`crates/kernel-api/src/discover.rs` is regenerated, not hand-edited — `gen_discover.py`
reproduces it byte for byte from `program.rs`, verified as part of this pass and now a
nightly gate.

No file under any `*_system/` campaign directory or `showcase/` was modified.
