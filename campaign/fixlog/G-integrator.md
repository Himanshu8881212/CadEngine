# G — Integrator fixlog

> **Structure note (2026-09-03):** the `studio/` tree — HTTP server, web IDE,
> `lmcad-tui`, and the `lmcad-mcp` MCP server — was removed from the repository.
> Paths under `studio/` named below are historical; the engine is reached through
> the `kernel-api` CLI, and the analysis layer through `python3 tools/<tool>.py`.

Goal: prove the tree is healthy after six parallel owners landed changes, and that
NOTHING regressed for the 10 shipped campaigns + the showcase.

## Step ledger (update as you go — a successor continues from here)

- [x] 0. Orient. `git status` = 44 modified + 13 new files. Build green.
- [x] 1a. `cargo build --workspace --release` — GREEN (1m14s)
- [x] 1b. `cargo test --workspace --release` — 160 suites / 1073 tests, 0 failed
      (1 failure found and fixed first: studio-server apidoc, see FINDING 2)
- [x] 1c. `cargo clippy --workspace --all-targets --release` — 0 warnings
      (2 found and fixed: type_complexity in my own code, neg_cmp_op in owner B's test)
- [x] 2. gen_discover regeneration changed NOTHING — generator in step with program.rs
- [x] 3. `python3 tools/audit_docs.py` exit 0 (18 INFO, 0 error/warn)
- [x] 4. Python tool health — 12 gate scripts, all exit 0 after fixes
- [x] 5. CAMPAIGN REGRESSION — 30 programs, 39 committed artefacts, ALL byte-identical,
      0 warnings, 0 non-zero exits (after FINDING 1 fixed; 11 were failing before)
- [x] 6. din_rail FEA receipt + 4 tolerance stacks from 2 campaigns — additive only
- [x] 7. Wrote campaign/ENGINE_FIX_REPORT.md

## Notes

### FINDING 1 (fixed) — `Mesh::boundary_edge_count()` counted NON-ORIENTABLE edges as boundary edges
Owner A's new T6 trust guard (`connectivity_measures`, interp.rs ~163) refuses
`mesh_components`/`assert components` when the measurement tessellation of a SOLID has
boundary edges. Correct premise; it was fed a WRONG boundary count.

`Mesh::boundary_edge_count()` asked "is the reverse directed edge absent?". Two triangles
that share an edge but wind the SAME way close that edge perfectly — no rim — yet `b→a` is
absent, so every one of them was called a boundary edge. `kernel_core::meshcheck::edge_report`
has always separated boundary (used once) from non-orientable (used twice, same direction):
two oracles for one property, disagreeing by construction.

Repro (minimal, from din_rail part_program lid chain — `lid_pl..lid_dl3`, needs `lid_key` in
the union): 176 triangles. `validate` on the bound export mesh → `boundary_edges: 0,
non_orientable_edges: 3`. `mesh_components` → refused, "left 3 boundary edges".

Blast radius BEFORE fix: 11 part programs across 8 of the 10 campaigns exited 1 (wrist
insert+receptacle, gripper palm, din_rail part_program, cleat part_base, ball frame+platform,
turgo part_program, singulator crank+geneva+housing_top). All had `component count (1)` in the
refusal message — the passing value.

Fix: crates/kernel-core/src/mesh/mod.rs — `directed_edges()` + `is_boundary_edge()`; an edge is
a boundary edge iff the UNDIRECTED edge is used exactly once. `fill_holes` now uses the same
rule (it would otherwise fan a cap onto an already-closed edge = a winding defect promoted to a
non-manifold one). New `non_orientable_edge_count()` so the two defects can be reported apart
without paying for check_mesh's self-intersection sweep. Boundary walks now iterate in triangle
order (determinism).
Test: crates/kernel-core/tests/boundary_edge_oracle.rs (4 tests). Negative control run: with the
old rule restored, 3 of 4 FAIL.

### FINDING 2 (fixed) — studio-server apidoc test pinned a doc GAP by name
`extract_section(&md, "clearance").is_none()` — owner A documented `clearance` in API.md, so
the test broke. The name was never the contract. Replaced with the biconditional over the live
`describe` surface, expectation computed by an independent prefix scan.

### FINDING 3 (fixed) — `validate` on a bound mesh contradicted itself
Reported `closed: false` beside `boundary_edges: 0` (its `closed` was check_mesh's `watertight`,
which folds in orientability + bowtie vertices). Now `closed` = closure, `manifold` = the rest;
`valid` is arithmetically unchanged.

### FINDING 4 (fixed) — 2 clippy warnings
`type_complexity` on my own `directed_edges` return (fixed with a documented type alias) and
`neg_cmp_op_on_partial_ord` in owner B's materials_creep_crosslang.rs (its `!(x < eps)` was
NaN-safety; rewritten as `delta.is_nan() || delta.abs() >= eps` so the intent is explicit).

### FINDING 5 (fixed) — nightly.sh watched none of the Python contracts
`cargo test` + clippy + 2 examples only. The ~150 Python pins six owners just wrote were
unwatched. Added a DISCOVERED (glob) `tools/test_*.py` + `docs/test_*.py` step plus named
gates, and a "discover.rs is in step with program.rs" step.

### FINDING 6 (fixed) — docs/test_doc_contracts.py pinned `assert`'s param set by enumeration
Owner A added `tol`/`weld_tol` to `assert` — exactly what owner F had asked for — and owner F's
contract failed for being right about yesterday. Repinned on the PROPERTY (gate is tunable,
defaults == the measure's, a tightened gate catches a sever the default passes), and rewrote
DELIVERABLE_SPEC §2.2 to match.

### FINDING 7 (fixed) — field_triage --self-test reported a missing FIXTURE as 11 failed checks
`spool_system/respool/analysis/ANALYSIS.md` is absent; every claim-parsing check returned 0
claims and was reported FAILED, and three checks that assert an EMPTY result passed VACUOUSLY.
Now: `needs=` on a check, SKIP not FAIL, and a named `fixture_missing` refusal distinct from
`self_test_failed`. 28 pass / 0 fail / 13 skip (was 30 pass / 11 fail).

### FINDING 8 (reported, not fixed) — `watertight: true` on 26 of 41 shipped exports means only EDGE CLOSURE
`Mesh::is_watertight()` = every edge used twice. It does NOT test orientability. Census of the
10 campaigns + showcase: 26 export ops write files carrying 3–395 non-orientable edges while
reporting `route: exact, watertight: true`. Bytes unchanged from what shipped. `export_stl` now
also reports `watertight_means`, `boundary_edges`, `non_orientable_edges`, `two_manifold`.
NOT fixed in the tessellator: that would change 26 shipped STLs' bytes.

### FINDING 9 (fixed) — cross-owner requests no single owner could close
- studio/mcp/src/lib.rs: `production_check` had no `duration_h`, so EVERY MCP caller with
  sustained:true would get `refusal.creep_duration_required` with no way to answer.
- tools/solvers/README.md: creep row; exit-code note still said "ACE runners exit 0 on failure";
  status-vs-tier note.
- tools/analyzer_registry.py: `audit_docs.py` + 5 others declared in NON_ANALYSIS; drift warning
  now empty.

### Method note for a successor
My FIRST regression harness copied each campaign tree INCLUDING parts/, so a program that
aborted before its export was compared against the shipped file it never overwrote and reported
"identical". It hid the 11-program regression behind a green result for one pass. Delete the
artefact before rebuilding it. kernel-api ABORTS at the first failing op.
