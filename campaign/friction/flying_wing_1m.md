# flying_wing_1m — friction log

Campaign: `Workspace/aerospace_system/flying_wing_1m/` (1 m PETG-CF flying wing, 2208 motor, 4S, single carbon spar per side, single-wall printed segments). Written during the campaign under DELIVERABLE_SPEC §4; engine and tools source untouched.

## F1 — `ace_optimize_runner.py` hands a string `material` to the solver unresolved (2026-09-06)
- severity: minor
- surface: tools/analyzers/ace_optimize_runner.py
- status: open
- symptom: a SIMP job with `"material": "PETG"` (the string-key form every other voxel runner accepts) dies with `{"ok": false, "error": "TypeError: string indices must be integers, not 'str'", "error_kind": "internal"}` after loading the density grid; no traceback reaches the receipt.
- minimal repro: any `ace_optimize_runner.py` job whose `material` is a registry key, e.g. `programs/simp_section_job.json` with `"material": "PETG"`; `python3 tools/analyzers/ace_optimize_runner.py job.json` → exit 1, `error_kind: internal`.
- expected vs actual: `tools_cookbook.md` §ace_optimize says the job is "ace_fea job + volfrac …", and `ace_fea_runner.py` resolves a string key through `_ace.resolve_material` (`tools/analyzers/_ace.py:109`); `ace_optimize_runner.py:171` calls `reference_fea(rho, kind, voxel, job["material"], …)` with the raw job value, so the FEA indexes a string. A pasted `{youngs_modulus_pa, poisson, density_kg_m3}` dict works.
- workaround used: `programs/gen_simp_section.py` resolves the record itself via `materials.get("PETG").fea_material()` (same one-source record, hash echoed in the job) — no claim weakened, ~20 min lost to the hidden traceback.

## F2 — XFOIL 6.99 (external bridge) crashes when appending to a polar save file under a current gfortran (2026-09-06)
- severity: note
- surface: xfoil (external, ~/.local/bin, not an LMCAD surface)
- status: fixed — polar accumulated in memory (`PACC` with no file) and written once with `PWRT 1 <file>`; the bridge also refuses to reuse a stale file
- symptom: `Fortran runtime error: … use REWIND or BACKSPACE`, exit 2, polar file left with a header and no rows; and the binary would not start at all until Homebrew `gcc` (libgfortran 5) and `libx11` were reinstalled.
- workaround used: `programs/xfoil_polar.py` (PACC-in-memory + PWRT); `brew install gcc libx11` restored the runtime. Kept in the log so the next campaign that reaches for XFOIL does not lose the hour.

## F3 — `union_all` severs a tube where two thin plates pierce it, silently (2026-09-06)
- severity: major
- surface: union_all
- status: open
- symptom: the centre section's two spar-socket tubes (Ø16.3 × 80 mm, meeting at the centreline) pierced by two 0.9 mm bulkhead plates at y = ±48 mm, unioned with the skin: `validate` reports `valid: true, shells: 2`, `mesh_components` reports 2 bodies, and splitting the exported mesh shows the second body is the socket pair's MIDDLE portion between the two plates (bbox x 77–104, y ±47.6) — the union cut the tubes at the plate faces and lost the continuity through the 0.9 mm plates. No error, no warning.
- minimal repro: `Workspace/aerospace_system/flying_wing_1m/programs/friction_repro/center_u5.json` (skin loft + two plates + two sockets, `union_all`, `export_stl`) — `skin+bulkR+sockR+sockL` → 1 shell; `skin+bulkL+bulkR+sockR` → 1 shell; all five → 2 shells. Fold-order dependent.
- expected vs actual: `ops_core.md` §6 says a union of overlapping bodies is one shell (the `shells` count is the disjointness proof); a tube overlapping a plate by the plate's full thickness (0.9 mm, far above the 0.1 mm sliver band) came back as two bodies while `valid` stayed true — the DELIVERABLE_SPEC §2.2 "severed into floating lumps" class, caught by the mandatory `shells`/`components` gates.
- workaround used: the plates get a Ø17.3 clearance hole around the socket (no piercing), and the sockets are tied to the skins by two 0.9 mm spar-box webs that overlap the socket walls along their whole length (embedded overlaps, no thin-plate crossings). No shipped claim weakened; ~1 h lost.
- recurrence of: campaign/friction/ENGINE.md#24 (validity does not imply connectedness — this is a new way to reach it: a union that produces the severance rather than a cutter)

## F4 — `audit_docs.py --only receipt`: the anchor grammar is discoverable only by failing it (2026-09-06)
- severity: note
- surface: tools/audit_docs.py
- status: open
- symptom: 94 of 103 anchors in a freshly generated `analysis/ANALYSIS.md` failed at once, for four unrelated reasons that no document states together: (1) an anchor path resolves from the DOCUMENT's directory (or the repo root), so a file under `analysis/` must write `../receipts/…`; (2) booleans are "no numeric value"; (3) list indices (`cg_mm[0]`, `values.panels[3].margin`, `rules[0].SF`) are not part of the key grammar; (4) the number quoted must match the receipt IN THE RECEIPT'S UNIT — a value shown in mm from a receipt in metres fails even with `tol=200%` (the tolerance is relative, a 1000× unit change is 99 900 %).
- minimal repro: a line `| tip | 3.96 mm <!-- receipt: receipts/fea_spar_receipt.json tip_displacement_m tol=200% --> |` in `<campaign>/analysis/ANALYSIS.md`; `python3 tools/audit_docs.py --also <campaign> --only receipt` → "names a file that does not exist" (path) and, once rebased, "prose quotes 3.96 but … is 0.00395578".
- expected vs actual: DELIVERABLE_SPEC §5 ("every number … carries a `<!-- receipt: path key -->` anchor") reads as if any quoted number can be anchored; in practice only scalar numbers in receipt units on dotted keys, resolved from the document's own directory. The failure messages are precise, so this cost ~20 min, not a claim.
- workaround used: `programs/gen_analysis.py` `row()` emits an anchor only for a non-bool number on a bracket-free key and, when the displayed value is unit-converted, quotes the receipt-unit value right before the anchor ("3.96 mm (receipt value 0.00395578 <!-- … -->)"); list-indexed values are shown without an anchor and the mass budget gained a scalar `cg_x_mm`. Suggest: one paragraph of anchor grammar in DELIVERABLE_SPEC §3/§5 (or a `--explain` line in the audit's first failure).

## F5 — `ace_optimize_runner.py`: a failed as-built check throws away the finished optimisation (2026-09-06)
- severity: minor
- surface: tools/analyzers/ace_optimize_runner.py
- status: open
- symptom: the SIMP section study ran its full 60 iterations (log: `iter 60: compliance 2.152817e-06, change 0.2000, vol 0.0600`) and wrote `receipts/simp_section/final_rho.npy`, then the as-built verification FEA on the iso-thresholded field died (`RuntimeError: reference_fea: CG did not converge (Jacobi then AMG, info=2000) at 54536 DOFs`) and the receipt came back as `{"ok": false, "error_kind": "internal", "iterations": null, "compliance_first": null, "compliance_last": null, "final_rho_npy": null}` — the sixty iterations, the compliance trace and the path of the field it had just written are all gone from the receipt, and a solver refusal (an unconverged CG on a fragmented 0.5 mm field — the same one-voxel-wall fragmentation `ace_fea_runner` reports as `refusal.solver.unconverged` with a `grid_connectivity` block) is reported as `internal`.
- minimal repro: `programs/simp_section_job.json` (0.5 mm grid, 0.45 mm frozen skins, volfrac 0.06, 60 iterations, 2700 s) with `python3 tools/analyzers/ace_optimize_runner.py … --out r.json` → exit 2, `error_kind: internal`, `iterations: null`; `receipts/simp_section/final_rho.npy` exists with the run's timestamp.
- expected vs actual: an as-built check that refuses should come back as a refusal (`refusal.solver.unconverged` or `refusal.manufacturing_mesh_invalid`, as the 2600 s run on the earlier 14 % section did) WITH the optimisation history and the unreleased field path, so the operator can record "optimised, not released" instead of "internal error, nothing". The run before it (1500 s budget, 42 iterations) refused cleanly as `refusal.optimization_incomplete`, so the history-loss is specific to the as-built failure path.
- workaround used: both receipts are kept (`receipts/simp_section_refusal_1500s.json`, `receipts/simp_section_internal_2700s.json` = the cited receipt); the last iterate is rendered from the npy the optimiser wrote and stamped UNRELEASED in `renders/simp_section.png`; no compliance number is claimed; the truss layout for the plane-strain comparison is a schematic tracing of that picture. ~50 min of compute, no claim weakened.

## F6 — `clearance` reports `interfering: true` for a coplanar face-to-face contact (2026-09-06)
- severity: note
- surface: clearance
- status: open
- symptom: the SG90 body resting with its ear top faces exactly on the cradle's recess ceiling (coplanar, no penetration by construction) measures `contact: true, distance: 0.0, interfering: true, overlap_volume: 0.0084` — the 0.008 mm³ is the boolean's coincident-face residue, but the `interfering` flag reads as a real interference, so a `require: {"interfering": false}` gate on a designed contact fails.
- minimal repro: `programs/assembly_scene.json` op `c_servo_in_cradle_R` with `"require": {"interfering": false}` (the shipped op requires `overlap_volume <= 0.05` instead).
- expected vs actual: ops_core.md lists `coincident_fit_hazard` as the flag for face-on-face fits; a designed bearing contact should read `contact: true, interfering: false` (or the residue should be below the op's own `tol` 0.01) — here the residue volume trips the interference flag.
- workaround used: gate the overlap volume, not the flag; no claim weakened.

## F7 — `export_step` writes every lofted surface as planar patches; CAD importers flag faults (2026-09-06)
- severity: minor
- surface: export_step
- status: open
- symptom: `cad/center_section.step` (AP203, one `MANIFOLD_SOLID_BREP`, 1437 `ADVANCED_FACE`: 1111 `PLANE`, 8 `CYLINDRICAL_SURFACE`) imports into Onshape as "translated with errors — parts with faults have been imported": the lofted airfoil skins are thousands of tiny planar facets with sub-0.01 mm slivers at the leading/trailing edges, which the translator's tolerance heals as faults. The geometry is the engine's exact faceted B-rep; it is not a spline loft.
- minimal repro: `kernel-api run programs/center_section.json --out-dir .` then open `cad/center_section.step` in Onshape (or count `PLANE` vs `B_SPLINE_SURFACE` in the file: 1111 vs 0).
- expected vs actual: ops_core.md says export_step writes "EXACT analytic surfaces (plane/cylinder/sphere/cone/torus) … untagged faces export as planar patches" — so this is as documented; the gap is that a `loft` has no analytic surface class and cannot be tagged, so any airfoil/organic part is a faceted STEP. A ruled loft between two polylines is a set of bilinear/ruled patches that could be written as `B_SPLINE_SURFACE_WITH_KNOTS` of degree 1x1 with far fewer faces (one per polyline segment pair), which importers accept without faults.
- workaround used: none needed for printing (the STL carries the same information); the user was told the STEP is faceted. Suggest: a `ruled`/`bspline` face tag for lofts in export_step.

## F8 — `assert_disjoint` reports contact when one solid merely crosses a face PLANE of the other through a void (2026-09-06)
- severity: minor
- surface: assert_disjoint
- status: open
- symptom: the nose pod's tongue (a box) passes through the plane of the centre section's nose flat (x = 8) into the OPEN nose cavity; `assert_disjoint(pod, centre, min 0.15)` fails with "surface distance 0 mm", and `intersection(pod, centre)` returns a 3.6e-7 mm³ sliver whose bounding box is a 0.009 x 20 x 0.01 mm line exactly on the crossing (x = 8.000..8.009, z = 5.378..5.389). The same happened for the motor mount's tongue crossing the trailing-edge plane into its pocket.
- minimal repro: the campaign's `programs/assembly_scene.json` with the pod proof on `np_pod` (the shipped scene proves `np_shell` disjoint and the tongue by `overlap_volume <= 0.01` instead); or: a hollow box with an open face, and a bar entering through that opening whose side faces are parallel to the open face's plane — `assert_disjoint` → distance 0.
- expected vs actual: two solids that do not share material should measure a positive distance (here the tongue is 1 mm from every cavity wall); the crossing of a face plane through a void is not a contact. The boolean's tolerance mints a sliver on the crossing line that the distance test then sees as contact.
- workaround used: split the proof — the part of the body that does not cross the plane is proven with `assert_disjoint`, the crossing part with `clearance` + `require overlap_volume <= 0.01` (the sliver is 1e-7). No claim weakened; ~20 min.

## F9 — `validate`/`assert manifold` pass a solid whose mesh `export_stl` refuses (2092 non-manifold edges); the same solid POSED exports watertight (2026-09-07)
- severity: minor
- surface: validate, assert (closed/manifold), export_stl, pose
- status: open
- symptom: the pinched-tip wing segment (`programs/wing_outer2_R.json`, body = skin − two overlapping cavity cutters whose faces coincide over 1 mm) passes `validate`, `assert {closed, manifold, shells 1}`, `wall_thickness` and `support_report`, and its 90°-posed copy exports watertight through the exact route (the shipped part gate). Exporting the UNPOSED body (the assembly scene does) fails on every route and tolerance: "mesh is not manufacturing-ready even after the voxel heal (voxel 0.3 mm): non_manifold_edges=2092, self_intersections=108". A 0° pose fails the same way; only the 90° rotation exports.
- minimal repro: `programs/friction_repro/outer2_coincident_cutters.json` (the segment ops with the second cutter at the SAME offset as the first, then `export_stl` of `body` with `require.route exact`; then `pose` 90° about x and export again — the second passes).
- expected vs actual: a solid that `validate`/`assert manifold` accept should export on every route, or `validate` should report the non-manifold edges; a rigid rotation should not change manufacturability.
- workaround used: the second cutter is offset 0.03 mm deeper so its faces lie strictly inside the first cavity over the overlap (no coincident faces); both the posed and the unposed bodies then export. Related observation: the unposed MIRRORED elevon (`mirror` op output) exports only through the voxel-heal route (`route: voxel_healed`) while its posed twin exports exact — so the scene exports require `watertight` but do not pin the route.

## F10 — `wall_thickness` files ~1 000 mm² of "thin" readings on the centre body that its own witnesses put on edges (2026-09-07)
- severity: minor
- surface: wall_thickness
- status: open
- symptom: on the REV C centre body (a 0.9 mm skin, median 0.90, p05 0.87) `thin_area` reads 950–1 170 mm² with `flag_below 0.8` and `exclude_wedge_deg 75`; every `thin_witness` sits at |y| = 80.0 (the two OPEN span ends of the loft), on the motor hump's 0.45 mm flank steps, at the trailing-edge closure wedge or on the cut face — readings of 0.01–0.2 mm where no wall is thinner than 0.85. The REV B centre section (same ends, no pod, no hump) read 73 mm². Clipping the skin to |y| ≤ 70 moved the same readings to the clip planes (1 192 mm²); a 30 mm slab of the plain wing read 71 genuine + 305 "wedge".
- minimal repro: `Workspace/aerospace_system/flying_wing_1m/programs/center_body.json` up to `center_body_wall` with the `require` removed; compare `thin_witness` positions with the loft ends.
- expected vs actual: ops_core §wall_thickness says `min_thickness` is edge noise and `thin_area` + percentiles are the judge; here the edge noise reaches the AREA figure (1 % of the surface), so the area gate cannot be set at the value a real thin panel would produce without also failing on edges.
- workaround used: the part's `thin_area` budget is 1 200 with the reason in the program's notes, and the skin is gated a second way — `programs/check_body_wall.py` ray-casts thickness on the EXPORTED print mesh (numpy Möller–Trumbore, area-weighted samples away from the ends, per span band, p05 ≥ 0.85 and ≤ 3 % under 0.8) and writes `receipts/body_wall_raycast.json`. No claim weakened; ~2 h lost locating the readings.

## F11 — `support_report`'s angle convention has to be reverse-engineered from the digest's two examples (2026-09-07)
- severity: papercut
- surface: support_report
- status: open
- symptom: designing a fused motor hump whose flanks must clear the 47° gate needed the exact rule; `describe` and ops_core §11a give two worked cases ("a 45° wall is steep at 44 and clean at 45", "63.4° at 63/64") but not the formula. Derived by matching both: a downward face is steep iff the angle between its normal and straight down is < 90° − `overhang_deg` (i.e. the face leans more than `overhang_deg` from the vertical); `near_threshold_area` is the band within 1° of that. A doubly-sloped face (a flank that also slopes chordwise) is LESS steep than its spanwise slope suggests (the x-slope tilts the normal further from down), which is what made the hump's raked front printable.
- minimal repro: any loft with a flank at a known slope, `support_report` at two thresholds around it.
- expected vs actual: one sentence of definition in the op's `doc` would have saved the probes.
- workaround used: probes at the modelled angles; the sentence above is now in DESIGN.md §5.

## F12 — `clearance`/`assert_disjoint` read a glued cradle as touching the cut skin when its recess walls were coplanar with the cut-out's walls (papercut, worked around)

- **Where:** `Workspace/aerospace_system/flying_wing_1m/programs/assembly_scene.json`, proof `g_cradle_vs_o2R` (the servo cradle `crR_placed` against the outer segment `o2R_body_servo` = the lofted skin minus the servo cut-out prism), 2026-09-07.
- **What happened:** the cradle's z = 0 face sits 0.7 mm inside the skin's inner surface (glue line); the exported meshes (tol 0.005) put the nearest wing vertex 0.698 mm from the cradle, `clearance` against the UNCUT segment reads 0.613 mm, yet against the cut segment it reads `distance 0, contact true, interfering false, overlap 0` and `assert_disjoint 0.1` refuses. Splitting the cut segment into regions (intersection with boxes) localised the phantom contact to the region holding the cut-out's aft wall and to the slab below the cradle's face. The cradle's ear recess (12.8 × 33.1, the same outline as the skin cut-out, aligned by the same psi/theta pose) had its side walls COPLANAR with the cut-out's walls, 0.7 mm apart along the plane.
- **Repro / not repro:** a 9-op toy (a 0.45 slab with a slot, a block 1.15 above it with a coplanar recess) reads the correct 0.7 — the false contact needs the lofted skin + posed prism cutter of the scene. The scene program before the fix is the run-2 `assembly_scene.json` with `RECESS_EXTRA = 0` in `gen_wing.servo_cradle`.
- **Workaround:** the recess is 0.2 mm wider than the cut-out on every side (`gen_wing.RECESS_EXTRA`), so no face pair is coplanar; the proof then reads 0.613 mm. The ear-length stack `tol_servo_length` carries the 33.5 recess.
- **Ask:** when `contact` is reported with `overlap_volume 0`, name the face pair (ids or a point) in the measures, so a phantom contact can be told from a real graze without bisecting the solid by hand.

## F13 — `teardrop_hole`: `through` is the depth from the SURFACE, not from `at` (papercut, documentation)

- **Where:** `Workspace/aerospace_system/flying_wing_1m/programs/joint_clip_outer.json` (rev C.1 joint clips), 2026-09-07.
- **What happened:** the pilots were written the way the campaign's other teardrops are (the motor plate's holes, the hatch pilot): `at` 0.5 mm OUTSIDE the face, `through` = wanted depth + 0.5. Measured by the closed-form volume against the kernel's (`through` 3.5 removed 7.52 mm³ per Ø1.6 teardrop = 3.5 × 2.148 mm²; 4.5 removed 4.5 × …), the hole is `through` deep from the surface the mouth point projects onto — the 0.5 mm of air in front of the face does not count. A blind pilot meant to stop 0.47 mm under the saddle groove went 0.5 mm deeper, broke into the groove and turned the part's genus from 0 to 2 (and, with the pilots exiting into the groove, the exact STL route reported open edges and fell back to `voxel_healed`).
- **Ask:** say so in the op's doc line (`through: depth below the surface at `at`'s projection`), or take `at` literally. Every existing teardrop in this campaign is therefore 0.5 mm deeper than its author intended — harmless there (they all exit into air), corrected on the clips.


## F14 — `difference` refuses "tangential contact" between operands that are 0.44 mm apart (diagnostic)

- **Where:** `Workspace/aerospace_system/flying_wing_1m/programs/wing_inner_R.json`, op `skin` = difference(outer loft, cavity loft), 2026-09-08 (rev E build attempts 1–9). Reproduced in a 3-op probe: the two lofts and their difference, nothing else.
- **What happened:** the difference refused with `closed=false manifold=false genus=3 shells=1`, four bad edges — two sub-segments of the cavity's rear closure, one in each of the part's two end-cap planes (y = 80.0000 and y = 290.0000 exactly), each reported twice, with the message "an edge used by 4 faces where the operands only TOUCH … separate the bodies by ≥ 1e-3 mm or overlap them". The operands do not touch: both lofts validate alone (`shells 1, genus 0, valid, closed, manifold`), every section polygon is simple, the cavity is strictly inside the skin at all 71 sampled stations AND on the ruled surfaces between span knots, and the minimum boundary-to-boundary distance is **0.4361 mm** — 436× the tolerance the message asks for. The trigger was a trailing-edge construction that stepped the section onto a constant two-line strip; moving the step (0.90–0.95 chord) and the closure (1–6 mm ahead of it) changed the reported edge by microns but never cleared the refusal.
- **Repro / not repro:** present whenever the skin loft carried the step and the cavity closed ahead of it; absent with the step removed and the same cavity (green), and the difference itself succeeded with the step present and the older gap-based closure. So it needs both operands, not either alone.
- **Workaround:** the trailing-edge strip was abandoned for a four-line blunt trailing edge (`te_thick_mm` 1.8), which reaches a better printability number by a construction the boolean accepts. A separate, genuine instance of the same message was self-inflicted and worth recording next to it: ruling the cavity on the skin's own span knots puts a cavity vertex ring exactly ON the skin's cap plane, which IS a vertex-on-face contact — the fix there is to keep the two knot sets offset.
- **Ask:** when the refusal is a tangential contact, report the two faces (or the point pair) and the measured separation. Here the separation was 0.44 mm and the message sent the campaign looking for a graze that does not exist; nine builds went into bisecting it by hand. If the real cause is a tessellation artefact rather than model geometry, say which triangle.

## F15 — `audit_docs.py` walks `revisions/` and re-litigates frozen history (rule collision, blocking)

- **Where:** `python3 tools/audit_docs.py --also <campaign> --only receipt`, the last gate of `flying_wing_1m/run_all.sh`, 2026-09-08.
- **What happened:** 43 receipt findings, **every one of them inside `revisions/`** — `C.1/analysis/ANALYSIS.md` (7), `D.0-spiral-wings/README.md` (5), `D.0-spiral-wings/analysis/ANALYSIS.md` (31). The LIVE documents are clean: auditing `<campaign>/analysis` alone reports 0 findings over 101 references. `SKIP_DIRS` is `{.git, target, __pycache__, node_modules, .venv}` and the `--also` path does a plain `rglob("*.md")` with no skip filtering at all, so a snapshot's frozen documents are audited as if they were live claims about the current receipts.
- **Why it cannot be satisfied:** DELIVERABLE_SPEC §2.15 snapshots the campaign at moments the auditor's contract does not hold. `D.0-spiral-wings` was taken deliberately BEFORE a redesign, so its `parts/` and `plates/` are rev D while its documents are rev C.1's — run 7 never reached the document steps, and the snapshot's README and ANALYSIS contain zero mentions of the V45 profile they sit beside. That is an accurate record of an unfinished state, and no amount of correctness in the live campaign will make those two files agree with a receipt. Rev C.1, a properly finished green revision, still fails 7: a snapshot cannot be expected to self-audit.
- **Workaround (not applied — `tools/` is read-only during campaigns):** audit `<campaign>/analysis` instead of `<campaign>`. That covers ANALYSIS.md's 151 anchors but drops README.md's 21, because `--also` only accepts a directory (a file path yields 0 references) and README.md sits at the campaign root next to `revisions/`. Weakening a gate to make a run green is exactly what the honesty rule forbids, so this campaign is holding at the refusal instead.
- **Ask:** add `revisions` to `SKIP_DIRS`, and have the `--also` walk honour `SKIP_DIRS` as the `--root` walk already does (`os.walk` at line 227 filters it; `rglob` at line 1108 does not). A frozen snapshot is a historical artefact, never a live claim; auditing one asks yesterday's document to describe today's receipt. This will otherwise fail on every campaign that adopts §2.15, at every future revision.
- **Resolved 2026-09-08** (maintainer approved the engine fix): `revisions` added to `SKIP_DIRS` and the `--also` walk now honours `SKIP_DIRS` as the `--root` walk already did. Campaign audit: 0 findings over 113 receipt references, 0 across all nine check classes, corpus 12 live files. Noted separately while testing it: `audit_docs.py --self-test` crashes with `FileNotFoundError: …/inj/AGENTS.md` — verified identical on the unmodified tool with `git stash`, so it is PRE-EXISTING and not from this change, but a tool whose docstring says each check is "independently self-tested" currently has no working self-test.
