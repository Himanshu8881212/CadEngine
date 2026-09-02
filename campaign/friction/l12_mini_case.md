# Friction log — l12_mini_case (Framework Laptop 12 mainboard case)

Engine and tools were read-only; these are the things that cost time or forced a design
detour. Numbered F1… for the maintainer; the campaign's own defects log (design-side) is
in `framework_system/l12_mini_case/analysis/DESIGN.md`.

## F1 — `import_step` refuses real vendor STEPs (three out of three)

Framework's mainboard STEP (45.6 MB, 168 solids) fails with "trim vertex does not lie on
B-spline patch"; the battery with "inner loops on a curved analytic face"; the Expansion
Card enclosure with "periodic sphere/torus region". Worked around by extracting per-solid
names + bounding boxes with OpenCascade (`vendor/occ_extract.py`) and modelling the board
as an inflated-box envelope. A `bbox_only`/`tolerant` import mode that returns solids' AABBs
and names without needing the B-rep would have saved a day.

## F2 — `import_mesh {heal}` cannot heal the vendor STL

"still not watertight after healing (non_manifold_edges=1070)" on the OpenCascade mesh of
the mainboard. Used for renders only (`assembly/scene/board_mesh.stl`, built by
`vendor/board_mesh.py` outside the engine).

## F3 — exact-route export is facet-luck sensitive (again), now with two clean bisections

- Tray: the catch **ridges** on the long walls at crest bottom z ≥ 3.0 demote the export to
  `voxel_healed` *only when the plug windows in the end walls also exist* — 43 mm apart,
  every ridge shape/angle/length/embed variant, every x position, deterministic (11 parallel
  runs). One ridge alone passes; ridges at z ≤ 2.8 pass. `mesh_components` reports the mesh
  clean (0 non-orientable, watertight) — the demotion reason is not surfaced anywhere in the
  receipt. Design fix: catch **recesses** (cuts) instead of ridges. Please emit the reason
  (the leaking edge/triangle) in the `export_stl` receipt when the exact route is abandoned.
- Lid: tab length 20 + the fan grille at x 28..80 demotes; 18 or 22 pass; moving the grille
  ±3 mm doesn't help. Same request.
- Pin pockets Ø2.4 with 24 segments inside a 48-segment Ø8.5 boss leave 3 non-orientable
  edges; 30 segments (or Ø2.6/24) are clean.
- VESA keyholes: a round counterbore concentric with the entry hole (0.5 mm ring) plus a
  slot counterbore box whose sides graze the round one → healed. One rectangular counterbore
  covering the whole entry circle is clean.

## F4 — `wall_thickness` reads mirror-image dovetail grooves 5× apart

Four identical floor grooves at x ±22/±90: the whole-tray thin_area reads 19.6 mm² with the
±22 pair, 101 mm² with the −90 groove and 19.6 with the +90 groove alone (min 0.037 vs 1.08).
The lip of a female dovetail is a knife-edge wedge at the bed, so *some* thin reading is
physical; the 5× asymmetry is the sampler. Adding a 1.0-mm vertical land to the neck moved
the reading to the male rails (343/709 mm²) and demoted the tray export. Resolution: gate
the tray minus the lip bands (`g_walls_nolip`, 0.0 mm²) and report the whole-body number
under a loose bound. A `wall_thickness` option to exclude wedge edges (dihedral < 90°) or
to report the thin patches' locations would make this a one-liner. `thin_wall`'s `at` is
dominated by edges too.

## F5 — `clearance` on complex bodies: `overlap_volume` null

Same as CONEJURE: require `interfering` only, then an exact `intersection` + `exact_volume`
for the must-interfere controls.

## F6 — `ace_contact_runner` plane obstacle never engaged

A plane at the beam tip with `normal [0,-1]` and `motion [0,1]` reported penalty force
400 N and zero tip motion/stress (the beam sat 0.02 inside the solid side and the plane
did not move it). The prescribed-tip-displacement path (`supports: {node: tip, dofs: {uy},
ramped: true}`) works and is what the campaign uses; the tip force is derived from the
receipt's peak stress (Roark 8.1) in `programs/contact_eval.py`. A receipt field
`tip_reaction_n` for prescribed-displacement supports would remove that derivation.

## F7 — `production_check` creep buckets 30 °C to the 55 °C cell

`creep_lookup('PLA', 30, 8760)` → 0.5 MPa (bucket 55C). Conservative by design, but a
23 → 55 °C jump with nothing in between turns a 30 °C wall mount into a fail. A 35 or 40 °C
cell in `tools/materials/pla.json` would help every enclosure campaign.

## F8 — `render_views` on the assembled scene is tiny

Four views of a 288 × 128 × 24 mm assembly render the model at ~15 % of the panel; a
`zoom`/`fit` option or auto-fit to the largest view would make the hero usable directly.
