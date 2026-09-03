# Fixlog — pose/boolean/export robustness round (2026-08-27)

> **Structure note (2026-09-03):** the `studio/` tree — HTTP server, web IDE,
> `lmcad-tui`, and the `lmcad-mcp` MCP server — was removed from the repository.
> Paths under `studio/` named below are historical; the engine is reached through
> the `kernel-api` CLI, and the analysis layer through `python3 tools/<tool>.py`.

Trigger: the Back to School 2026 campaigns' friction files
(`rated_desk_hook.md`, `folding_book_stand.md`); fixes maintainer-directed.

## Engine changes

1. **f64 piercing predicate** (`kernel-core/src/mesh/measure.rs`
   `segment_pierces_triangle`, `kernel-core/src/meshcheck.rs` `seg_hits_tri`):
   Möller–Trumbore now evaluates in f64 with a scale-aware parallel guard.
   At f32, rotated meshes' last-ulp noise manufactured self-intersection
   witnesses; the false positives (a) demoted posed exports to the voxel
   heal (the "hangs") and (b) misrouted the tessellator's cap-cleanliness
   check during boolean face rebuilds.
2. **Heal time budget** (`kernel-api/src/interp.rs` `heal_voxel_for_budget`):
   the winding-number heal lattice is capped at 2²² cells by auto-coarsening
   the heal voxel; the export receipt carries `heal_voxel_mm` on the healed
   route, so coarsening is on the record. (`MAX_LATTICE_CELLS` = 2²⁸ remains
   the memory bound.)
3. **Rotation snapping** (`kernel-api/src/interp.rs` `snap_rotation`, wired
   into pose / rotate_x/y/z / polar_pattern): rotation-matrix entries within
   1e-12 of {0, ±1} snap exact, so axis-permutation rotations are exact
   signed permutation matrices and coplanar classification survives posing.
4. **union_all fold order** (`kernel-api/src/interp.rs`): operands fold in
   ascending AABB-contact-degree order (disjoint first, hub last) instead of
   argument order — repeated arrangements against the same rebuilt face were
   failing position-dependently. Union is associative; result unchanged;
   deterministic (ties keep argument order).
5. **clearance overlap fallback** (`kernel-api/src/interp.rs`): exact
   intersection failure now yields the FACETED mesh-boolean overlap volume
   with an explanatory `overlap_volume_reason`, never a bare null.
6. **render_views path resolution** (`studio/mcp/src/lib.rs`): `stl` resolves
   against out dir then repo root (read-only); the miss error names both.

## Regression tests

- `crates/kernel-brep/tests/pose_tessellation.rs` — rigid rotations preserve
  manufacturing readiness (9 angles incl. the two measured failures); the
  −118° witness must be None.

## Verified against the original failures

- posed panel export: >8 min heal grind → **exact route, 0.6 s**
- deployed 3-body scene export: hang → **exact route, 0.46 s**
- 4 posed carriers coincide-abutting a hole wall: `union_all`
  invalid_geometry at #3 → **ok + exact-route export**
- interfering posed pair: `overlap_volume: null` → numeric faceted value

## Docs

- `campaign/digests/tools_cookbook.md`: production_check `duration_h` +
  creep-table governance (was stale vs the tool).

## Doc contracts (docs/test_doc_contracts.py) — 22/22 after updates

Two contracts were pinning superseded behavior:
- "components REFUSES on the holed plate": the f64 fix closes that
  tessellation, so the oracle now returns the TRUE count (1, boundary_edges
  0) — contract re-pinned to the improvement (DELIVERABLE_SPEC §2.2 carries
  an update note; the refusal channel remains for genuinely open surfaces).
- "import resolves against the PROGRAM dir (only)": stale since the earlier
  T4 heal added the --out-dir fallback; contract re-pinned, and ops_core §10
  + OPERATOR_BRIEF §3.2 prose updated to the program-dir-first + fallback
  truth (same-name STEP round-trips now work).
