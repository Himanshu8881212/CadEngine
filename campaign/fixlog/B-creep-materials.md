# B-creep-materials — T14 fix log

Owner: B-creep-materials. Files owned: `tools/materials.py`, `tools/material_db.json`,
`tools/materials/**`, `tools/production_check.py`, `tools/field_triage.py`,
`tools/materials_crosslang_test.py`, `tools/solvers/*.md` (material/creep cards),
`crates/kernel-model/src/lib.rs` (creep/material fns only).

## HARD CONSTRAINT discovered first (read this before editing any record)

`tools/materials/pla.json`'s `meta.hash`
(`dbdd2b78bfa1e93ca36c9574739df44d01b4c716587fdb2052285602c9d1be78`) is quoted
verbatim inside **shipped campaign receipts** in at least 9 of the 10 campaign
trees (thermal receipts, contact receipts, `design_material_gates.json`,
`creep_gate.json`, plus prose in two ANALYSIS.md / DESIGN.md).
**Therefore no existing `tools/materials/*.json` record may be edited** — any
content change re-hashes the record and breaks byte-identical rebuild of the
shipped deliverables. Every fix below is CODE reading the unchanged record.
New sidecar files in `tools/materials/` are fine (they opt out of the record
schema via `meta.schema_kind`, like the existing `fatigue.json`).

## Reproduced (before any edit)

1. **Two readers, one table, Python non-conservative** (singulator F2)
   ```
   $ python3 -c "import sys;sys.path.insert(0,'tools');import materials as M,field_triage as F;\
     rec=M.get('pla').record; print(F.creep_allowable(rec,120.0,24.0))"
   {'known': True, 'sig_allow_mpa': 1.5, 'temperature_bucket': '55C', ... 'extrapolated': True}
   ```
   Rust `kernel_model::materials::pla::creep_allowable_mpa(120.0, 24.0)` == **0.0**.
   Same at 70 C. Also divergent: `hours = -5` -> Python 7.5 MPa, Rust 0.0;
   `temp_c = NaN`/`None` -> Python 1.5 MPa (silently the hot row), Rust 0.0.
2. **`creep_allowable_mpa` unreachable from Python** (singulator F1)
   `hasattr(materials, 'creep_allowable_mpa')` -> `False`. Only
   `field_triage.creep_allowable(record_dict, ...)` existed, reachable only via a
   field REPORT, undocumented in the brief/spec/cookbook.
3. **`field_triage.creep_allowable("pla", ...)`** (cubesat F12)
   -> `AttributeError: 'str' object has no attribute 'get'`.
4. **`production_check.py` has no duration input** (wrist F15, singulator F3)
   PLA sustained -> `allowable = yield 55.00 x creep_sustained_fraction 0.20 = 11.00 MPa`
   vs the record's `creep.sig_allow_mpa["23C"]["1y"] = 2.5` — 4.4x optimistic,
   and the job schema has nowhere to state a duration.
   12 shipped prodcheck jobs across 7 campaigns set `sustained: true`; **none**
   carries a duration.
5. **No cell provenance anywhere** (horn F16): nothing in any receipt records
   which (temperature row, duration column) a margin was read at.

Baseline of the tools I own, before my edits:
* `python3 tools/production_check.py --selftest` -> PASS
* `python3 tools/materials.py --selftest` -> PASS (7 records)
* `python3 tools/materials_crosslang_test.py` -> PASS
* `cargo test -p kernel-model --release --test materials_creep` -> 2 passed
* `python3 tools/field_triage.py --self-test` -> **already RED (exit 1)** for a
  reason outside this theme: `spool_system/respool/analysis/ANALYSIS.md` does not
  exist in the tree, so 10 checks that parse it fail. All four E-001 *creep*
  checks pass. Recorded here so the pre-existing red is not attributed to me.

## Changes

### 1. `tools/materials.py` — ONE reader, and it refuses
New: `creep_lookup(material, temp_c, hours, across_layer=)` (full receipt) and
`creep_allowable_mpa(...)` (bare scalar, exactly `lookup["sig_allow_mpa"]`).
This is the callable OPERATOR_BRIEF §7 and DELIVERABLE_SPEC §2 gate 8 promised
and that did not exist in Python. Semantics are the Rust contract's:
round T and duration UP; **REFUSE above the hottest tier / on non-finite,
missing or negative input / for a material with no table**; reuse the last
duration column beyond 1 y, flagged. `sig_allow_mpa` is 0.0 on every refusal so
`demand <= allowable` fails on its own. Accepts a NAME or a record dict.
Receipt carries `row_used_c`, `col_used_h`, `temperature_bucket`,
`duration_bucket`, `cell_match` (exact / rounded_up_conservative /
extrapolated_beyond_last_duration / refused), `refusal_kind`, `across_layer`,
`anisotropy_factor`, per-cell `confidence`, material `hash`.
Supporting: `creep_cells`, `creep_temp_key_c`, `creep_duration_key_hours`
(RAISE on an unparseable key — the old reader coerced one to 0.0 h, which sorts
FIRST and would then be picked for every request), `legacy_creep_scalar`.
`validate()` now also proves (a) every `conflicts[].field` resolves and a scalar
`canonical` equals the live value — this is what makes the recorded PLA
cp 1200 vs 1800 J/kgK entry and the creep_sustained_fraction 0.2 vs table entry
machine-checked instead of prose; (b) a creep table is non-increasing in BOTH
axes, without which "round up" is not conservative.
New CLI `--creep MAT T HOURS [--across-layer]`, exit 1 on refusal.

### 2. `tools/field_triage.py` — delegates, keeps its keys
`creep_allowable(mat, temp_c, duration_h, across_layer=False)` is now a thin
wrapper over `materials.creep_lookup`; accepts a NAME as well as a record dict
(closes cubesat F12). All legacy receipt keys kept. **Behavior change**: at
> 55 °C it now refuses instead of returning the 55 °C row.

### 3. `tools/production_check.py` — a duration, and the table
New job field `duration_h` (also `service.duration_h`, `load_character.duration_h`).
The `creep` rule reads `creep.sig_allow_mpa` through `materials.creep_lookup` at
the stated T and duration; the row carries a full `creep_cell` provenance block
plus `legacy_scalar_mpa` (reported, never used). Refuses — as a FAILING row with
a machine-matchable `refusal_kind`, never an exception and never a number — when
the duration is missing, T is above the table, or the material has no table.
Also: `__main__` now uses `_receipt.load_job` + `_receipt.finish`, so a failed
or refused run exits 2 instead of the previous `emit(...); sys.exit(0)`.

### 4. `crates/kernel-model/src/lib.rs` (creep fns only) — additive provenance
New `CreepCell` / `CreepCellMatch` / `CreepRefusal` and
`pla::creep_lookup(temp_c, hours, across_layer)`. `creep_allowable_mpa` is now
`creep_lookup(..).sig_allow_mpa` — **numerically unchanged**, proven by the
pre-existing `tests/materials_creep.rs` passing untouched. Enum `as_str()` slugs
are the SAME strings the Python receipts use, so one vocabulary spans both.

### 5. THE PIN — `tools/materials/creep_crosslang_vectors.json` (new sidecar)
540 probes x {allowable, in-plane value, row, column, cell_match, refusal_kind}
covering every tier boundary, both sides of every boundary (22.999 / 23.0 /
23.000001 / 54.999 / 55.0 / 55.000001), above the hot tier (56 / 70 / 120 / 1e6),
every duration column exactly, between columns, beyond 1 y, and nan / +-inf /
negative / -0.0 inputs, x {in-plane, across-layer}.
`tools/materials_crosslang_test.py` checks the PYTHON reader against it, plus
six hand-written (not generated) doctrine pins; it also runs the RUST leg live
(`cargo test ... --test materials_creep_crosslang`) and treats a missing cargo
as a FAILURE, not a skip (`--no-rust` opts out loudly).
`crates/kernel-model/tests/materials_creep_crosslang.rs` checks the RUST reader
against the SAME file, plus its own hand-written doctrine pins.

### 6. `tools/solvers/creep.md` (new card)
The material/creep card: physics, the table, the six lookup rules, the receipt
fields, the recorded 0.2-fraction conflict, the benchmark gates, validity limits.

## Proved

* `python3 tools/materials.py --selftest` — PASS (9 new creep checks).
* `python3 tools/production_check.py --selftest` — PASS (8 new creep checks).
* `python3 tools/materials_crosslang_test.py` — PASS, 540/540 probes + live Rust leg.
* `cargo test -p kernel-model --release --test materials_creep` — 2 passed
  (UNCHANGED file: proof `creep_allowable_mpa` did not move numerically).
* `cargo test -p kernel-model --release --test materials_creep_crosslang` — 2 passed.
* `python3 tools/field_triage.py --self-test` — 11 failures, the SAME 11 as the
  pre-existing baseline (missing `spool_system/respool/analysis/ANALYSIS.md`);
  all 12 creep checks, 11 of them new, pass.

### Negative proofs (the tests FAIL without the fix)

* Python: monkeypatched `materials.creep_lookup` back to the old
  "fall back to the LAST tabulated row" semantics ->
  `3 / 7` creep checks fail, including `probe mismatch at 120 of 540`.
  (`scratchpad/neg_proof.py`, transcript in the report.)
* Rust: temporarily deleted the `TempAboveTabulated` refusal ->
  `rust_and_python_agree_at_every_creep_cell_boundary` FAILED with
  "DISAGREE at 120 of 540 probes", first mismatch
  `probe[300] T=55.000001 ... rust sigma=3 ... python sigma=0 refused`;
  `creep_refusal_doctrine_is_hand_pinned` FAILED too. Reverted.
* production_check: the shipped horn job `job_prodcheck_stand_sustained.json`
  (PLA, 25 °C, sustained, across-layer, no duration) previously produced
  `creep allowable 11.0 x 0.55 = 6.05 MPa, PASS`; it now returns
  `refusal.creep_duration_required`, exit 2. With `duration_h: 8760` added it
  returns `creep.sig_allow_mpa[55C][1y] 0.5 x 0.55 = 0.275 MPa, SF 0.35 FAIL`
  — a 22x change in the allowable, in the conservative direction.

## LOUD backward-compatibility notes

1. `production_check.py` with `sustained: true` and **no `duration_h` now
   REFUSES** (`ok:false`, exit 2). **12 shipped jobs across 7 campaigns** are in
   that state: horn x4, singulator x1, wrist x2, din_rail x2, turgo x1, ball x2.
   Re-running them without adding a duration will fail. This is the T14 defect
   itself — the old path derived a creep verdict from the static yield — so the
   old behavior IS the bug. Campaign dirs were not touched.
2. `field_triage.creep_allowable` above 55 °C now refuses instead of returning
   1.5 MPa. No shipped field report is above the table (E-003 is 70 °C but is
   classified a condition violation and takes no creep number), so no shipped
   receipt changes.
3. A material with no creep table (everything except PLA) now refuses instead of
   serving `yield x 0.2`. `production_check --selftest`'s old "PETG creep
   allowable 12.5 MPa" pin was itself asserting the defect and is replaced.
4. **No `tools/materials/*.json` record was edited** — see the hard constraint
   above. Only a new sidecar was added. Every record hash is unchanged.
5. `tools/material_db.json` gained ONE additive top-level key,
   `creep_sustained_fraction_is_SUPERSEDED`. The `disclaimer` string that
   production_check receipts embed is byte-unchanged.

## Cross-owner requests (files I do not own)

* `studio/mcp/src/lib.rs` — the `production_check` MCP tool schema still
  describes the OLD creep rule ("creep (yield x creep_sustained_fraction …)")
  and exposes no `duration_h` property, so an MCP caller cannot state a
  duration and will always get the refusal. Needs: a `duration_h` number
  property, and the description rewritten to the table semantics.
  Same for the module-header table at `lib.rs:29`.
* `tools/solvers/README.md` — the solver registry table needs a row for the new
  `creep.md` card. I own only the card, not the registry.
* `campaign/OPERATOR_BRIEF.md` §7 / `campaign/DELIVERABLE_SPEC.md` §2 gate 8 —
  should now point at the reachable Python entry points
  (`tools/materials.py --creep MAT T HOURS`, `materials.creep_allowable_mpa`,
  `production_check.py` `duration_h`) instead of a Rust-only symbol, and should
  state that a sustained gate must quote the CELL (`cell_match`,
  `temperature_bucket`, `duration_bucket`) it was read at.
* `tools/field_triage.py --self-test` is red for a reason outside this theme:
  `spool_system/respool/analysis/ANALYSIS.md` is absent from the tree, so 11
  claim-parsing checks fail. Baseline before and after my change is identical.
* `docs/` / `DESIGN_GUIDE.md` mention `creep_allowable_mpa` as Rust-only; they
  can now point at the Python mirror.

## Status: COMPLETE. All owned gates green (see Proved).

