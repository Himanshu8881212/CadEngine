# Fixlog — export demotion receipt + wall_thickness wedges (2026-09-02)

Trigger: friction `l12_mini_case.md` F3 (exact-route export demotes with no
reason on the receipt — recurring) and F4 (`wall_thickness` reads mirror-image
dovetail grooves 5× apart and counts the lip's knife edge as a thin wall);
`uphill_roller.md` F2/F3 are the same two defects. Fixes maintainer-directed.

## Engine changes

1. **Demotion receipt** (`kernel-api/src/interp.rs` `solid_mesh_routed` /
   `exact_route_demotion`): whenever the exact route is abandoned,
   `export_stl` / `export_3mf` (and `asm_export`'s per-instance entries) carry
   `demotion: {reason, boundary_edges, non_manifold_edges,
   non_orientable_edges, non_manifold_vertices, degenerate_triangles,
   self_intersections, exact_triangles, witness}`. `reason` is the first
   failing check of the exact route's manufacturing predicate (boundary →
   non-manifold edges → non-orientable → non-manifold vertices → degenerate →
   self-intersection; `tessellation_failed` for an empty tessellation), the
   counts are the abandoned EXACT tessellation's, `self_intersections` is
   `null` when the sweep never ran, and `witness` holds up to 8 points of the
   named defect in the body's frame. The decision logic is untouched (same
   predicate, same evaluation order); exact exports carry no new field.
   Witness machinery: `Mesh::boundary_edge_witnesses` /
   `non_manifold_edge_witnesses` (one traversal with the existing
   `non_orientable_edge_witnesses`), `kernel_core::degenerate_triangle_witnesses`,
   `non_manifold_vertex_witnesses`.
2. **`wall_thickness` sampler** (`kernel-core/src/mesh/thickness.rs`):
   area-uniform stratified sampling — a 65 536-sample budget spread by
   surface area, each triangle above one budget cell split into `m²`
   barycentric sub-triangles with one hash-jittered sample each (weights sum
   to the area exactly; deterministic, no RNG state). Triangles at or below
   one cell keep the single centroid sample, so fine voxel-route meshes read
   byte-identically. Per-triangle `thickness` keeps the centroid ray.
3. **`exclude_wedge_deg`** (`wall_thickness` op, `ThicknessOptions`): a
   flagged reading whose ray exits through a face that shares an edge with
   the sample's own face, at a CONVEX material dihedral below the threshold,
   is an acute-wedge reading — counted under `thin_area_wedge`, reported with
   `thin_area_total` and `thin_wedge_witness`. Faces are groups of
   edge-connected triangles with normals within ≈1° (boolean-sliver
   tolerant; zero-area slivers are transparent to adjacency). Parallel faces
   never share an edge, so a thin plate / drafted wall is never a wedge; a
   concave notch is never a wedge (far-corner convexity test). Absent, no
   wedge fields are emitted. `thin_witness` (≤ 8 thinnest counted flagged
   samples) and `samples` are always reported.

## Regression tests

- `crates/kernel-core/src/mesh/thickness.rs` unit tests — weights partition
  the area (plate reads exactly 800 mm²), determinism, a 30° wedge prism's
  apex band moves to `thin_area_wedge` only when asked (area conserved).
- `crates/kernel-api/tests/wall_thickness_wedge.rs` — the F4 dovetail block
  (neck 4 / tip 6 / depth 2.5 in a 4.5 mm floor, exact boolean): `thin_area
  0` with `exclude_wedge_deg 75` and `thin_area_wedge` ≈ 77 mm² (analytic
  2 lips × (0.64 + 0.64) × 30); mirror in X agrees within 5 %; witnesses sit on
  the lips; a 0.5 mm plate stays 800 mm² of `thin_area`; angle validation.
- `crates/kernel-api/tests/export_demotion.rs` — Ø8.5/48-segment boss with a
  concentric blind Ø2.4/24-segment pocket (the campaign's pin-pocket case):
  `mesh_components` calls it clean, the export demotes on
  `degenerate_triangles` (1 of 896) with the witness on the Ø2.4 wall at
  r = 1.2; STL and 3MF receipts identical; a box exports exact with no field.

## Verified against the original failures

- Pin pocket Ø2.4/24 inside a Ø8.5/48 boss on a plate: `mesh_components`
  reports 3 non-orientable edges → `demotion.reason: "non_orientable_edges"`,
  3 witnesses on one vertical line of the pocket wall (r = 1.2, z 6..8).
- The same pocket in a bare boss (`mesh_components` clean) →
  `degenerate_triangles` (1), witness mid-wall.
- VESA keyhole (round counterbore + grazing slot counterbore) →
  `degenerate_triangles` (2), witnesses on the Ø4.5 through-hole wall.
- Dovetail block, plain census: original 76.63 mm² vs mirrored 76.82 mm²
  (0.25 % apart; the campaign measured 19.6 vs 101). With
  `exclude_wedge_deg 75`: both `thin_area 0.0`, `min_thickness 2.0` (the
  groove floor), the lip bands under `thin_area_wedge`.
- The API.md example tray (single extrusion with the dovetail notch):
  `thin_area 0.0`, `thin_area_wedge 75.9`, its `require` passes.

## Receipt values that change for existing bodies

Baseline = `cleanup-2026-09` (492e02e) release binary, same programs, no
`exclude_wedge_deg`. Rule of thumb: anything the centroid sampler could see
is unchanged; thin bands it missed now count; `median_thickness` is now an
AREA median (it was the median over triangles, so a plate read its 60 mm
side length); `min_thickness` on an acute body is the knife edge.

| body (flag_below) | measure | baseline | now | why |
|---|---|---|---|---|
| 60×60×8 plate (1.0) | thin_area / min / p05 | 0 / 8.0 / 8.0 | same | exact either way |
| 60×60×8 plate (1.0) | median_thickness | 60.0 | 8.0 | area median, not triangle-count median |
| dovetail block (1.6) | thin_area | 0.0 | 76.6 (mirror 76.8) | the 0.64 mm lip bands, missed by every centroid; analytic ≈ 76.8 |
| dovetail block (1.6) | min_thickness | 2.0 | 0.0006 | the knife edge (use `exclude_wedge_deg`: → 2.0, thin_area 0) |
| dovetail block (1.6) | median | 30.0 | 4.5 | area median (the floor) |
| flange Ø80/8, 2×Ø7 holes (3.0) | thin_area / p05 | 0 / 8.0 | 0 / 8.0 | unchanged |
| flange (3.0) | min_thickness | 6.52 | 5.56 | real ligament hole-wall → top chamfer at z ≈ 8 (witnessed) |
| flange (3.0) | median | 10.38 | 8.0 | area median |
| uphill_roller cone (1.6) | thin_area | 0.0 | 458.9 | the 59° rim band, 0.96 mm wide × 2 faces (analytic ≈ 464); `exclude_wedge_deg 75` → thin_area 0, wedge 458.9, min 12.5 |
| uphill_roller cone (1.6) | p05 | 21.4 | 2.41 | the rim band is ≈ 5 % of the area; with the exclusion p05 = 18.7 |
| any mesh whose triangles are all ≤ one budget cell (voxel-route meshes) | everything | — | byte-identical | one centroid sample per triangle, as before |

## Docs

- `API.md` `wall_thickness` (rewritten: params, measures, the wedge rule,
  determinism) and `export_stl` (the `demotion` object; the routing paragraph).
- `campaign/digests/ops_core.md` rows for `wall_thickness` and `export_stl`.
- `docs/CHANGELOG.md` dated entry.

## Not covered

- The `.lmcasm` CLI (`kernel-api asm …`, `asm.rs`) routes parts through
  `kernel_model`'s `mesh_instance_exact_routed`, a separate routing path; its
  per-part `route` receipt does not carry `demotion`.
- `thin_wall` (the implicit-side sampled census) is a different sampler and
  is unchanged.
