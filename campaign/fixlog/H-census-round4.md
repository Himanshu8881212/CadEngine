# Round 4 — census follow-up (2026-08-23)

Scope: the post-rebaseline census's ONE real engine flag
(`screw_on_exponential_horn/part_program.json` → `invalid_geometry` with every
counter zero) plus the census classifier's misfiled negative controls. All
fixes general; probes were scratch (deleted); pins live in the test suites.

---

## H1 — adaptive curved-grid winding used ONE global normal per face

**Repro (minimal):** two coaxial 96-seg cylinders, `difference`, washer.
`mesh_components tol 0.005` → **24 non-orientable edges**, all at interior
grid rows of the bore wall (z = 1.5 / 3.0 / 4.5 on an h=6 washer).

**Cause:** `tessellate_curved` wound every grid triangle against the face
ring's single Newell normal — near-degenerate for a wide-arc (half-barrel)
face — and sign-corrected vertex normals per-vertex, so two vertices of one
cell could disagree.

**Fix (`crates/kernel-brep/src/tessellate_adaptive.rs`):** the same recipe
`push_refined_tris` already used: ONE aggregate sign vote over the ring
(Σ n_analytic·newell), then each triangle winds against the analytic normal at
its OWN centroid, `sigma`-corrected; vertex normals share the same sigma.
Quad/tri grids additionally REFUSE a folded grid (mixed winding against local
outward — a warped intersection-curve side folds the bilinear interior) and
fall through to the chart-based paths; `ear_clip_ring` gained a `_wound`
variant with per-vertex/per-triangle references for the warped-ring path.

**Pin:** `kernel-brep/tests/adaptive_holes.rs::fine_tolerance_washer_grid_stays_orientable`
(washer at 0.005 → NOE 0). Suite green.

## H2 — voxel heal silently returns EMPTY over the lattice budget

**Repro:** horn `export_stl tol 0.01` → exact mesh not manufacturing-ready
(H4 family) → heal at default voxel 0.3 → `manifold_dual_contour` needs
~785×785×886 ≈ 5.5e8 cells > `MAX_LATTICE_CELLS` (2.68e8) → **returns
`Mesh::new()`** → downstream refusal prints "boundary_edges=0, …,
self_intersections=0" (all counters of an EMPTY mesh) — the least actionable
message in the engine.

**Fix (`crates/kernel-api/src/interp.rs`):** the export policy detects the
empty healed mesh from a non-empty solid and refuses with the real cause and
the fix: part extent, the cell budget, and the MINIMUM workable voxel
(mirrors the mesher's pad+margin arithmetic, 5% headroom). Horn's message now
says "Re-export with voxel ≥ 0.37 mm, or at a tol where the exact route is
manufacturing-ready" — and voxel 0.4 was verified to heal it watertight
(3.47M tris, 40 min).

## H3 — `mesh_components` counts were not locatable

**Fix:** `Mesh::non_orientable_edge_witnesses(cap)` (kernel-core) + a
`non_orientable_witness` array (midpoints, cap 8) in the `mesh_components`
receipt whenever the count is nonzero. This is what localized H1 (witnesses on
a through-hole rim at z=0 — NOT the curved barrel everyone suspected) in one
probe run.

## H4 — residual DISCLOSED family (not fixed, bounded + routed honestly)

The boolean's carved-face decomposition emits sliver facets along warped
intersection curves; at fine chord tolerances their winding is
orientation-unstable. Localized repros: transverse-bored cylinder 56 NOE (all
within a hair of the intersection curve), 12-hole plate 115 NOE (hole-rim
slivers; the faces arrive as many small pieces — `tessellate_planar_with_holes`
never runs). The keyhole ladder ALSO gained `cap_rim_true` (a cap must not
duplicate a directed edge or invent a rim the bridged ring never had) so a
dirty clip now drives the retry ladder instead of shipping. Exports demote to
the voxel heal with receipts; the horn re-baselined to tol 0.05 (exact route)
instead. Full fix = decomposition-side, out of round-4 scope.

## H5 — census classifier misfiled declared refusals

10 of the census's 12 "FAILs" were programs whose HEADER declares
"expected REFUSAL" (graham `asm_scene_probe` on designed press fits, `*_refuse`,
`backlash_*`, `probe_oring_refusal`, `catalog_refusal_circlip4`).
`campaign/workflows/regress.py` + `lmcad-dsh/campaign_audit.py` now classify
NC by declared intent in the `part` header as well as by filename.

## Horn re-baseline (campaign artifact, engineering rationale)

`part_program.json` stl/3mf tol 0.01 → 0.05 (0.05 mm chord ≪ 0.2 mm nozzle;
rides the EXACT route). Full program re-run: exit 0, 0 warnings, watertight,
51,584 tris. Logged in the part's BUILD_LOG.
