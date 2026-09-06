# Friction log — l12_mini_case (Framework Laptop 12 mainboard case)

Engine and tools were read-only; these are the things that cost time or forced a design
detour. Numbered F1… for the maintainer; the campaign's own defects log (design-side) is
in `framework_system/l12_mini_case/analysis/DESIGN.md`.

## F1 — `import_step` refuses real vendor STEPs (three out of three)
- severity: blocker
- surface: import_step
- status: fixed — `mode: "tolerant"` shipped and MEASURED 2026-09-04, 163/168 solids in a release build

Framework's mainboard STEP (45.6 MB, 168 solids) fails with "trim vertex does not lie on
B-spline patch"; the battery with "inner loops on a curved analytic face"; the Expansion
Card enclosure with "periodic sphere/torus region". Worked around by extracting per-solid
names + bounding boxes with OpenCascade (`vendor/occ_extract.py`) and modelling the board
as an inflated-box envelope. A `bbox_only`/`tolerant` import mode that returns solids' AABBs
and names without needing the B-rep would have saved a day.

**RESOLVED in the engine — and now MEASURED, 2026-09-04.** `mode: "tolerant"` shipped and
the mainboard's *full* import (not just the census) was left unverified because a debug
build was too slow to settle it. Run in a **release** build it completes:

| | measured |
|---|---|
| wall time | **952 s (15.9 min)**, 950 s user — single-threaded, peak RSS ≈ 1.4 GB |
| solids listed / imported / skipped | **168 / 163 / 5** |
| face repairs | 81 `ADVANCED_FACE` flat-repaired, 4 unreadable |
| bound body | 221 shells, 602 921 faces, faceted volume 70 308.7 mm³ |
| names vs OpenCascade | **all 168 match** (124 distinct names over 168 instances; none missing, none extra) |
| per-solid envelopes vs OpenCascade | 147 / 168 within 0.05 mm; 18 within 0.5 mm; 3 beyond, worst **2.478 mm** |
| overall body envelope | within **0.018 mm** of OCC's overall AABB |

The envelope error is one-sided: every census box sits *inside* the OpenCascade box (worst
overshoot 0.041 mm, i.e. none outside the 0.05 mm band). The outliers are curved bodies
whose envelope is measured from the reconstructed chord vertices rather than the true
surface extreme — a keep-out derived from `solids[*].bbox_*` is therefore slightly
optimistic on rounded parts and must be inflated, exactly as this campaign already did.
The 5 skipped solids are still listed with name and `bbox_source: "edges"`: one hits an
unsupported arc-bounded loop, four reconstruct faces that do not close (genus 6–40).

So the campaign's inflated-box workaround was the right call at the time, but the honest
statement now is "163 of 168 solids reconstruct in ~16 min in release", not "unverified".

## F2 — `import_mesh {heal}` cannot heal the vendor STL
- severity: major
- surface: import_mesh
- status: partial — `import_mesh {heal: "remesh", voxel}` is the voxel repair the refusal pointed at: the file is lifted to its generalized-winding-number field (`|w| > ½`, so an inward-wound or mixed soup still has an interior) and re-meshed through manifold dual contouring → surface nets → a one-voxel opening, each on the receipt. The board (240 857 triangles, 864 non-manifold edges, 143 035 self-crossings, wound inward) still pinches at 55–219 non-manifold edges at every voxel 0.3–0.8: its overlapping component shells leave sheets thinner than a cell. The refusal names the counts and the tried ladder; the campaign's remodel-from-measurements stays the honest path for THIS file. The route is pinned on a two-shell inward soup (`kernel-api/tests/import_mesh_remesh.rs`)

"still not watertight after healing (non_manifold_edges=1070)" on the OpenCascade mesh of
the mainboard. Used for renders only (`assembly/scene/board_mesh.stl`, built by
`vendor/board_mesh.py` outside the engine).

## F3 — exact-route export is facet-luck sensitive (again), now with two clean bisections  — DIAGNOSIS FIXED 2026-09-03
- severity: major
- surface: export_stl
- status: fixed — the exact route is no longer facet-luck sensitive (snap rounding + CDT + coalesced caps); `rot_mirror` repro exports exact at three poses and the `demotion` receipt names any residual defect

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

**FIXED in the engine (2026-09-03, commit on `cleanup-2026-09`):** a demoted export now
carries `demotion` in its receipt — `reason` (the first failing check: boundary_edges /
non_manifold_edges / non_orientable_edges / non_manifold_vertices / degenerate_triangles /
self_intersection / tessellation_failed), the counts, and up to 8 `witness` points in the
body's frame. The bisections above would have been a single receipt read. (The demotions
themselves are still facet luck; the receipt now says where.)

## F4 — `wall_thickness` reads mirror-image dovetail grooves 5× apart  — FIXED 2026-09-03
- severity: blocker
- surface: wall_thickness
- status: fixed — engine 2026-09-03: area-uniform sampler, `exclude_wedge_deg`, `thin_witness`

Four identical floor grooves at x ±22/±90: the whole-tray thin_area reads 19.6 mm² with the
±22 pair, 101 mm² with the −90 groove and 19.6 with the +90 groove alone (min 0.037 vs 1.08).
The lip of a female dovetail is a knife-edge wedge at the bed, so *some* thin reading is
physical; the 5× asymmetry is the sampler. Adding a 1.0-mm vertical land to the neck moved
the reading to the male rails (343/709 mm²) and demoted the tray export. Resolution at the time: gate
the tray minus the lip bands and report the whole-body number under a loose bound.

**FIXED in the engine (2026-09-03, commit on `cleanup-2026-09`).** The sampler is now
area-uniform and deterministic (mirror images agree to ~0.25 %; the 5× was one centroid ray
per triangle on boolean triangulations), `wall_thickness` takes `exclude_wedge_deg` (wedge
readings go to `thin_area_wedge`), and every receipt carries `thin_witness` with the
locations. The campaign now gates the tray, foot rail and VESA frame with
`exclude_wedge_deg: 75` and reads `thin_area` 0.0; the hand-cut lip-band workaround is
deleted.

## F5 — `clearance` on complex bodies: `overlap_volume` null
- severity: major
- surface: clearance
- status: fixed — `overlap_volume` is never null (ENGINE #28)

Same as CONEJURE: require `interfering` only, then an exact `intersection` + `exact_volume`
for the must-interfere controls.

## F6 — `ace_contact_runner` plane obstacle never engaged
- severity: major
- surface: tools/analyzers/ace_contact_runner.py
- status: fixed — ace_contact plane obstacle engages (`contact_plane` repro: contact nodes > 0, normal force reported)

A plane at the beam tip with `normal [0,-1]` and `motion [0,1]` reported penalty force
400 N and zero tip motion/stress (the beam sat 0.02 inside the solid side and the plane
did not move it). The prescribed-tip-displacement path (`supports: {node: tip, dofs: {uy},
ramped: true}`) works and is what the campaign uses; the tip force is derived from the
receipt's peak stress (Roark 8.1) in `programs/contact_eval.py`. A receipt field
`tip_reaction_n` for prescribed-displacement supports would remove that derivation.

## F7 — `production_check` creep buckets 30 °C to the 55 °C cell
- severity: minor
- surface: tools/materials/pla.json
- status: fixed — production_check reports the temperature row with its governing rule and interpolates the creep cell instead of bucketing 30 °C to 55 °C

`creep_lookup('PLA', 30, 8760)` → 0.5 MPa (bucket 55C). Conservative by design, but a
23 → 55 °C jump with nothing in between turns a 30 °C wall mount into a fail. A 35 or 40 °C
cell in `tools/materials/pla.json` would help every enclosure campaign.

## F8 — `render_views` on the assembled scene is tiny
- severity: papercut
- surface: tools/publish/render_views.py
- status: fixed — render_views fits the scene to the frame

Four views of a 288 × 128 × 24 mm assembly render the model at ~15 % of the panel; a
`zoom`/`fit` option or auto-fit to the largest view would make the hero usable directly.

## RESOLUTIONS (2026-09-05 fix round)

Engine (`crates/`) and `tools/` fixes made at the maintainer's request (2026-09-05); every `fixed` above names its receipt (a repro in the fix-round scratch set, a re-run of this campaign's own program, or a unit test). Entries above are unchanged except their `status` line.

- **F2** — F2: stays open — vendor soup is outside the heal contract; the receipt says so.
- **F3** — F3: facet-luck sensitivity removed; demotion receipt kept.
- **F5** — F5: overlap on complex bodies.
- **F6** — F6: obstacle engages.
- **F7** — F7: creep cell.
- **F8** — F8: render fit.

### Second pass (2026-09-05, the 13 items left open or partial)

- **F2** — F2: voxel remesh route added; see the status line for the board's own outcome.
