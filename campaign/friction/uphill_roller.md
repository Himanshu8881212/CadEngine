# Friction — uphill_roller (magic_system, 2026-09-01)

## F1 — `require` "within" expectation shape is undocumented and array-refused (2026-09-01)
- symptom: `{"require": {"distance": {"within": [0.08, 0.30]}}}` → `invalid_param: op 'clr_a': require 'distance': 'within' must be {"target": t, "abs": a} or {"target": t, "percent": p}`
- minimal repro: any measure op with `"require": {"<key>": {"within": [lo, hi]}}`
- expected vs actual: OPERATOR_BRIEF §1.3 and DELIVERABLE_SPEC §2 list `{equals|min|max|within|not_null}` without the shape of `within`; the natural reading is a range. The binary wants the `assert` volume-window form `{target, abs|percent}`.
- workaround used: a `win(lo, hi)` helper in gen_programs.py builds `{target, abs}`. Suggest the digests spell out the shape (one line) and/or the engine accept a 2-array.

## F2 — exact-route export is position-sensitive for a faceted bore with several cutters (2026-09-01)
- symptom: `export_stl` `route: "voxel_healed"` (190k triangles, 9.5 MB) for a cone half with a socket (bore + roof + 2 channels + 2 pockets); `mesh_components` at tol 0.01 reports 3–40 `non_orientable_edges` with witnesses at the cutter/bore-facet junctions. The SAME construction with an unrelated dimension changed (base chamfer 0.9 vs 0.95 vs 0.98 mm) flips between exact and healed. 20+ variants measured; the log is in `magic_system/uphill_roller/analysis/DESIGN.md` (defects).
- minimal repro: `magic_system/uphill_roller/programs/cone_half.json` with `BASE_CH_R` set to 0.95 in gen_programs.py (healed) vs 0.98 (exact).
- expected vs actual: DELIVERABLE_SPEC §2.4 says the residual sliver mis-winding is "disclosed" and demotes honestly — it does, and the witnesses are excellent for locating the spot. But there is no construction rule a campaign can follow to AVOID it: keeping cutter planes mid-facet, chaining differences, circumscribing arcs, distinct radii, and even single-outline stacked prisms (which fail `validate()` with either abutting or overlapping bands) all left some sliver. Nearly a day of the campaign went into this.
- workaround used: revert to the one construction that measured exact, then scan the seam-chamfer size and freeze the value that exports exact with `thin_area 0.0` (deterministic, so reproducible; fragile to edits). Suggest: (a) a `coalesce`/re-triangulate pass on the exact tessellation before the winding check, or a tolerance for sliver facets below the chord tolerance; (b) documenting that abutting/overlapping cutters with a shared facet fail the boolean.

## F3 — `wall_thickness` attributes ~23 mm² "thin" to a bare obtuse rim (2026-09-01)
- symptom: cone half with NO base chamfer (base plane meets a 31-deg cone wall, 121-deg dihedral): `thin_area 23.02 mm2`, `min_thickness 0.092`, `p05 13.87`; with a 0.98 mm chamfer cut into the same cone: `thin_area 0.0`. Reading identical across unrelated socket changes.
- minimal repro: `revolve` of `[[0,0],[38.5,0],[3.43,58.45],...nose...,[0,60.4]]` → `wall_thickness flag_below 1.6`.
- expected vs actual: "min_thickness is corner noise" is documented; `thin_area` on a 240 mm obtuse rim is not — it consumes the gate that SPEC §2.6 asks for (`thin_area: 0.0`).
- workaround used: seam chamfer (also wanted for elephant-foot relief). Suggest the measure exclude samples whose ray leaves through a face adjacent to the source edge, or report the flagged area per face pair.
