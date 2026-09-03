# F-docs-contract — fixlog

Owner F. Owned paths: `DESIGN_GUIDE.md`, `README.md`, `docs/**`,
`campaign/OPERATOR_BRIEF.md`, `campaign/DELIVERABLE_SPEC.md`, `campaign/digests/**`.

Binary under test: `target/release/kernel-api` (mtime 2026-08-06 21:32; no
`crates/**/src` file is newer, so it reflects current kernel source at the time
of measurement). Re-verify anything marked VERIFY-AFTER-FIX-PHASE after other
owners rebuild.

---

## A. §2.2 connectivity oracle — REPRODUCED, spec is wrong (fixing)

### A1. Difference-severed bar reads BOTH gates (spec's example unbuildable)
```json
{"ops":[{"id":"bar","op":"box","min":[-20,-5,-5],"max":[20,5,5]},
        {"id":"knife","op":"box","min":[-1,-10,-10],"max":[1,10,10]},
        {"id":"cut","op":"difference","a":"bar","b":"knife"},
        {"id":"v","op":"validate","in":"cut"},
        {"id":"mc","op":"mesh_components","in":"cut"}]}
```
→ `v: shells 2, genus 0, valid true, closed true, manifold true`
→ `mc: components 2, is_one_body false, tol 0.05, weld_tol 0.001`

Both fire. DELIVERABLE_SPEC §2.2 claims `shells==1` while `components` catches
it — NOT constructible this way.

### A2. Sub-weld-tol sever: `components` is the WEAKER check (inversion confirmed)
```json
{"ops":[{"id":"a","op":"box","min":[0,0,0],"max":[10,10,10]},
        {"id":"b","op":"box","min":[10.0005,0,0],"max":[20,10,10]},
        {"id":"u","op":"union_all","in":["a","b"]},
        {"id":"c","op":"mesh_components","in":"u"},
        {"id":"g_comp","op":"assert","in":"u","components":1},
        {"id":"g_shell","op":"assert","in":"u","shells":1}]}
```
→ `c: components 1, is_one_body true`; `g_comp` **PASSES**; `g_shell` FAILS
`assert_failed: shells: measured 2, expected 1`; process exit **1**.

This IS a constructible oracle-negative-control and it is the replacement
example for §2.2.

### A3. Weld-scale limit — measured, position-dependent (NOT a clean 0.001)
Two 10 mm boxes, faces at `base` and `base+gap`, `union_all`:

| gap (mm) | base 0 | base 10 | base 100 |
|---|---|---|---|
| 0.0001 | 1 | 1 | 1 |
| 0.0002 | 1 | 1 | 1 |
| 0.0003 | 1 | 1 | 1 |
| 0.0005 | **2** | **1** | **1** |
| 0.0007 | 2 | 2 | 2 |
| 0.0009 | 2 | 2 | 2 |
| 0.0010 | 2 | 2 | 2 |
| 0.0015 | 2 | 2 | 2 |

(`shells` = 2 in every cell.) So the merge distance is a **grid snap at
`weld_tol` on an f32 mesh**, not a distance test: guaranteed-welded ≲0.0003 mm,
guaranteed-separate ≳0.0007 mm, position-dependent in between. Documenting the
band, not a single number.

### A4. `weld_tol` IS tunable on the measure — T6's "fixed, non-tunable" is REFUTED
`{"op":"mesh_components","in":"u","weld_tol":0.0001}` → `components 2`,
receipt echoes `weld_tol: 0.0001`. Confirmed by the misspelling control
`"wled_tol"`, which returns `components 1` **plus** a `warnings` entry.
BUT `describe assert` lists only
`in / volume_within / exact_volume_within / genus / shells / components /
closed / manifold / valid` — **no `tol`, no `weld_tol`** — so the *gate* is
pinned at 0.05 / 0.001 even though the *measure* is tunable. That asymmetry is
the real defect; documented as such.

### A5. `components` over-counts hole loops (false-positive direction)
```json
{"id":"p","op":"extrude_with_holes","outer":[[0,0],[40,0],[40,20],[0,20]],
 "holes":[[[5,5],[10,5],[10,10],[5,10]],[[25,5],[30,5],[30,10],[25,10]]],"height":5}
```
→ `validate: shells 1, genus 2, valid true`; `mesh_components: components 3`
(1 + one per hole loop). `weld_tol: 0.01` does not change it.
So `assert components:1` cannot be satisfied on a legal single-body part with
through-holes made this way. Must be stated in §2.2.

---

## B. Digest regeneration (in progress)

## C. Determinism contract (in progress)

---

## D. RE-MEASURE 2026-08-08 ~06:00 — other owners landed fixes MID-PASS

`docs/test_doc_contracts.py` (my new gate) caught it: 8 of 21 contracts flipped
between my first measurement pass and the second. Binary rebuilt 05:58; ~20
files under `tools/` and 5 under `crates/` changed. **Everything below is
re-measured against the NEW state and the docs were rewritten to match.**

| what changed | old (my first pass) | NEW (authoritative) |
|---|---|---|
| `assert` param surface | 9 params | **+ `require`** — and `require` is now a UNIVERSAL param on every measure op |
| weld scale | position-dependent, f32 grid snap, band 0.0003–0.0007 | **clean distance test at `weld_tol`**: welded < 0.001, separate > 0.001, f32-fuzzy exactly AT 0.001 (base 0 → 1, base 10 → 2, base 100 → 1) |
| `clearance` on a nested pair | `distance: 0.0` (FALSE) | **FIXED — `distance: 0.2711`** on a true 0.30 mm gap (faceted under-read ≈ r(1−cos π/n)) |
| `assert_disjoint` nested | false-FAILED | **passes** |
| tool exit codes | 0/1 split, ACE always 0 | **three-code contract in `tools/_receipt.py`**: 0 ok:true · 1 could-not-run · 2 ran-and-REFUSED. ACE runners now strict too. `legacy_exit_zero` / `LMCAD_RUNNER_EXIT=legacy` opt-out |
| `_checker_cli.py` | existed, `--dry-run`/`--force` | **gone**; replaced by `_receipt.py`. `--out` only; `LMCAD_RECEIPT_DRY_RUN=1` env |
| solver determinism | nothing in the receipt | **`determinism.core_digest`** (sha256 over the payload minus `nondeterministic_paths`, floats quantized to `core_sig_figs` 12) — verified equal across two runs while `timings_s` differed |
| `assembly_doc.explode.axis` | vector only | **vector OR axis name** ("z"/"+z"/"-z") |
| `support_report` semantics | build_dir away-from-bed; larger overhang_deg = permissive; default 45; max_bridge_span = short way | **UNCHANGED — re-verified identical.** `describe` still ships empty `doc` for `build_dir`/`overhang_deg` → cross-owner request stands |
| `extrude_with_holes` over-count | shells 1 / components 3 | **UNCHANGED** |
| severed bar | shells 2 / components 2 | **UNCHANGED** |
| path-root asymmetry | export→out-dir, import→program dir, `..` refused | **UNCHANGED** |
| `ace_fatigue` stress nesting | top-level refused | **UNCHANGED** (now exit **2**, not 1) |

**T10 is CLOSED by `require`.** `{"op":"support_report",...,"require":{"steep_area":0.0}}`
fails the run with `assert_failed`; verified also on `export_stl`
(`{"watertight":true,"route":"exact"}`), `bounding_box` (`{"fits_within":true}`)
and `wall_thickness` (`{"thin_area":0.0}`). The receipt echoes a `required`
block. Four §2 gates that were previously unexpressible are now in-program.

---

## E. SECOND re-measure — `mesh_components` now REFUSES instead of over-counting

`extrude_with_holes` + 2 holes: `mesh_components` (and `assert components:1`
with it) now fails `invalid_geometry` verbatim:

> op 'c': the connectivity oracle cannot be trusted on this solid — tessellating
> it at tol 0.05 mm left 16 boundary edges (28 triangles), so the measurement
> surface is NOT closed and its component count (3) counts faceter cracks, not
> severed bodies. … Gate this part with `validate` (closed / manifold / shells)
> meanwhile, and/or `export_stl` it and run this measure on the export's bound
> mesh — the exported mesh IS what prints

Refuse-never-degrade working as designed. All connectivity prose rewritten
across SPEC §2.2, OPERATOR_BRIEF §8, ops_core, analysis_honesty, exemplars.

## F. WHAT I SHIPPED

**Files changed (all owned):**
- `campaign/DELIVERABLE_SPEC.md` — §2 `require` preamble (new); §2.2 rewritten
  (complementary oracles + measured tables + weld-scale limit); §2.5
  `support_report` semantics; §2.11 clearance + grown-gauge bracket; §2.13
  oracle-NC section (new, two constructible NCs); §3 determinism contract.
- `campaign/OPERATOR_BRIEF.md` — §1.3 `require`; §3.1 three-code exit contract
  (new); §3.2 path-root asymmetry (new); §4 disjointness; §5.1 tet-route
  reality (new); §7 creep surface; §8 connectivity + clearance.
- `campaign/digests/ops_core.md` — §10 export/import roots; §11a
  `support_report` measured semantics (new); §11b clearance (new); ENGINE
  UPDATE connectivity block rewritten; `assert_disjoint` + `clearance` rows.
- `campaign/digests/tools_cookbook.md` — wire contract + receipt persistence
  rewritten; `ace_fea` body-load N/kg; `ace_fatigue` stress nesting;
  `assembly_doc.explode.axis`; `param_optimize` evaluator `timeout`;
  per-tool exit lines.
- `campaign/digests/analysis_honesty.md` — §2 exit contract; connectivity;
  determinism (`core_digest`); creep surface + the 25 °C trap.
- `campaign/digests/exemplars.md` — connectivity oracle pair; oracle-NC rule.
- `DESIGN_GUIDE.md` §22 — `support_report` conventions.
- `docs/test_doc_contracts.py` — **NEW.** 22 executable doc contracts.

**Gates:** `python3 tools/audit_docs.py` → exit 0 (19 findings, 0 ≥ error).
`python3 docs/test_doc_contracts.py` → 22 passed, 0 failed, 0 skipped.
