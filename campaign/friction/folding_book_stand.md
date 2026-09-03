# friction — folding_book_stand

## F1 — union with a face coincident to a hole's inner wall fails; the same coincide on an outer face succeeds (2026-08-27)
- symptom: `union_all` including a carrier block whose face exactly
  coincided with the inner wall of an `extrude_with_holes` window failed
  `invalid_geometry: union_all failed validate(): closed=false
  manifold=false genus=9 ... shells=1`. The identical coincide pattern
  against an OUTER plate face (base carriers on the base front face)
  unioned fine.
- minimal repro: extrude_with_holes plate; box abutting the hole wall
  exactly; union_all → invalid_geometry. Bisect transcript in the session.
- expected vs actual: digest ops_core §6 says "Coplanar contact IS
  supported (boss-on-plate, flush stacks)"; the hole-wall case is not.
- workaround used: embed 1.0 mm into the plate instead of coinciding.

## F2 — shallow (0.35 mm) embed slivers HANG the exact STL tessellator; no timeout, no receipt (2026-08-27)
- symptom: `export_stl` (exact route) on a panel whose knuckles embedded
  0.35 mm into the plate edge ran > 8 CPU-minutes at 100 % without
  producing output; kill left no report. Same geometry with a 1.5 mm embed
  exports in 0.04 s. Chord `tol` 0.01/0.02/0.05 made no difference.
- minimal repro: plate + posed prism embedded 0.35 mm; export_stl.
- expected vs actual: OPERATOR_BRIEF §8 warns the 1e-6–0.1 mm sliver band
  "mints needle faces" but promises loud failure ("a program never writes
  garbage"); an infinite hang with no wall-clock guard is a third mode the
  docs do not name. `export_stl` has no internal budget the way tools/
  runners do.
- workaround used: deep (≥ 0.9 mm) embeds everywhere; features merged into
  single profiles where possible.

## F3 — box-vs-posed-prism unions fail position-dependently; sequential unions fragment until one dies (2026-08-27)
- symptom: identical knuckle+neck unions at different x offsets: first
  succeeded, second failed `invalid_geometry` (genus 1, shells 1). An
  arc-relieved carrier profile that exported fine alone and a plate that
  exported fine alone HUNG the tessellator when unioned. Sequential
  union of 16 attachments onto one plate died at the 5th; restructuring
  to union all mutually-disjoint attachments first, then ONE union with
  the plate, moved the failure but did not fix box-vs-prism cases.
- minimal repro: axis-aligned box overlapping a `pose`-rotated extrude
  prism (rotation [1,1,1] 120°) 1.0 deep; union; repeat at x+27.6.
- expected vs actual: float dirt from the pose (~1e-14 · coordinate) makes
  near-coincident planes classify differently per instance; digest §7.7
  boolean-hygiene list does not cover "posed prism vs axis-aligned box".
- workaround used: merged the boxes into the prism PROFILES (single
  polygon per knuckle including its plate-joint neck / grab flag) — zero
  extra booleans; and for the H2 carriers, clearance was created by
  chamfering the LEG plate corner instead of arc-relieving the carrier.

## F4 — export_stl of POSED solids hangs (2026-08-27)
- symptom: exporting the deployed-configuration union (bodies rotated
  -111..-233° about x) or even a SINGLE posed body hung > 60 s at 100 %
  CPU; the same bodies in print pose export in < 0.3 s.
- minimal repro: any use_pose_*.json body + export_stl.
- expected vs actual: no documented limitation on exporting posed solids.
- workaround used: the posed programs gate geometry (shells, clearances)
  without exporting; deployed views are left to photographs. Campaign
  proceeded.

## F5 — `clearance.overlap_volume` is null for some interfering posed pairs (2026-08-27)
- symptom: capture-NC clearance ops on interfering posed bodies returned
  `interfering: true` with `overlap_volume: null`, so a
  `require {overlap_volume: {min: ...}}` failed with "min/max/within need
  a numeric measure, but 'overlap_volume' measured null".
- expected vs actual: ops_core §9 documents overlap_volume (mm³) as a
  clearance measure (verified 27.0 on overlapping boxes); on these pairs
  it is null at exit 0.
- workaround used: gate capture NCs on `interfering: true` alone.

---

## RESOLUTIONS (maintainer-directed engine fixes, 2026-08-27)

The maintainer asked for these to be fixed in-engine; campaign rules were
lifted for that work. Root causes found and fixed:

- **F2 + F4 (the export "hangs") — one root cause, fixed.** The exact-route
  gate `self_intersection_witness` evaluated Möller–Trumbore in **f32**; on
  posed (rotated) meshes last-ulp coordinate noise manufactured a crossing
  that does not exist at double precision (verified: the −118° witness pair
  [139,143] vanishes in f64). The false positive demoted the export to the
  voxel heal, whose winding-number lattice on a panel-sized body (~19M cells
  at voxel 0.3) grinds for many minutes — indistinguishable from a hang.
  Fixes: the piercing predicate (both copies: `mesh/measure.rs`,
  `meshcheck.rs`) now evaluates in f64 with a scale-aware parallel guard;
  and the heal voxel auto-coarsens to a 2²² cell TIME budget, with
  `heal_voxel_mm` reported in the export receipt. Regression:
  `crates/kernel-brep/tests/pose_tessellation.rs`. Measured after: the posed
  panel and the full deployed scene export on the EXACT route in <1 s.
- **F3 + F1 (position-dependent boolean failures) — two fixes.**
  (a) Axis-permutation rotations ([1,1,1]/120°, ±90° about axes, …) built by
  axis-angle carried ~1e-16 residue in their "zero" entries, putting
  exactly-coplanar face pairs into near-coplanar limbo inside the exact
  arrangement. The op layer now snaps rotation entries within 1e-12 of
  {0, ±1} (`snap_rotation`, kernel-api interp.rs) — a posed prism abutting a
  hole wall now unions cleanly. (b) `union_all` used a left fold in argument
  order, re-arranging the SAME rebuilt face once per contacting operand
  (died at the third of four carriers); it now folds in ascending
  AABB-contact-degree order — mutually-disjoint operands first, the
  touch-everything operand once, last. Both verified on the reconstructed
  failing geometry (all four carriers coincide-abutting the window wall:
  union_all ok, exact-route export).
- **F5 (`overlap_volume: null`) — fixed.** When the exact intersection fails
  on a pair, `clearance` now falls back to the mesh-boolean volume of the
  already-tessellated operands, labelled as a FACETED estimate in
  `overlap_volume_reason`, instead of a null.

Residual, disclosed: genuinely oblique rotations (no snappable entries)
combined with exact face-coincidence remain the thinnest corner of the exact
arrangement — embeds ≥ 0.9 mm are still the recommended practice there.
