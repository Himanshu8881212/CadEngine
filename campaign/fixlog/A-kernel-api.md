# Owner A — kernel-api (T10, T6, T5, T15, T8)

Ownership: `crates/kernel-api/**`, `crates/kernel-core/src/mesh/**` (if strictly
required), `API.md`. Everything else is a cross-owner request.

Scratch repros live in
`/private/tmp/claude-501/-Users-himanshu-Work-New-LMCAD/eef3ec62-58a7-4682-b52d-7dd8bef08b42/scratchpad/repro/`.

---

## Stage 1 — reproductions (all done BEFORE any edit)

### T6(a) — `weld_tol` is a grid cell, not a tolerance; and it is not exposed on `assert`
`repro/t6a.json`: two boxes with a **0.0005 mm** gap.
```
v   shells 2      (severance seen)
mc  components 1  weld_tol 1e-3   (severance MISSED)
mc2 components 2  weld_tol 1e-6   (severance seen)
```
`assert { components: 1 }` has **no** `weld_tol`/`tol` param at all, so the gate
cannot be tuned. VERIFIED as described in the brief.

**New, worse finding** (`repro/t6a2.json`): `Mesh::component_count` quantizes
vertices onto a `1/weld_tol` grid with `round()` and unions only *identical*
cells. That makes the answer depend on ABSOLUTE POSITION, not on the gap:

| gap | geometry | components @ weld_tol 1e-3 |
|---|---|---|
| 0.0004 mm | `[0,10]` + `[10.0004,20]` | **1** |
| 0.0004 mm | `[0,10.0003]` + `[10.0007,20]` | **2** |

Same gap, same tolerance, different answer. `weld_tol` is not a tolerance today.

### T6(b) + T6(c) — ONE root cause, and it is not the one the campaigns guessed
`repro/t6b.json`: `extrude_with_holes` with 2 hole loops → `validate` genus 2,
shells 1, valid; `mesh_components` = **3** at every `weld_tol`.
`repro/t6c.json`+`t6c2.json`: a tube exported to STEP and re-imported →
`components 2` at `weld_tol` 1e-6 / 1e-3 / 1e-2 (weld tolerance is irrelevant).
`repro/t6e.json`: control — a plain **cylinder** STEP round-trip reads
`components 1`; a **box with a pocket** (top face carries an inner loop) reads 2.

Root cause: `crates/kernel-brep/src/tessellate_adaptive.rs::face_boundary`
assembles a face's boundary from `solid.face(f).outer` ONLY and never touches
`solid.face(f).inner`. Every planar face with a hole loop is tessellated as if
the hole were not there, so the hole's tube is disconnected from the rest of the
mesh and the measurement mesh is not closed.
Corroborating: `export_stl` of a 40×20×5 plate with ONE square hole reports
`route: "voxel_healed"` (`repro/t6d.json`) — a trivially exact part cannot take
the exact route — and `support_report` on it returns `total_area 2300` where the
true area is 2250 (the hole is missing from both caps).

**REFUTES** the campaigns' stated cause ("adjacent analytic faces do not share
vertices, so nothing welds at any tolerance", wrist F11 / cleat F6 / horn F10 /
rotor F10). Welding is innocent; inner loops are the cause.

`crates/kernel-brep/**` is NOT my file → cross-owner request (see below).
In-scope mitigation: the connectivity oracle must not report a count measured on
a non-closed measurement mesh.

### T5 — the silent `distance: 0.0` is caused by DEGENERATE TRIANGLES
`repro/t5a.json`: a coaxial spigot (r 16.8) inside a counterbore (r 17.1) with a
real 0.3 mm radial gap and a 1.0 mm axial gap →
`{"distance": 0.0, "interfering": false, "overlap_volume": 0.0}` — the exact
horn-F7 contradiction.
`repro/t5b.json` control: the same nesting with a small spigot reads `2.0`
correctly, so nesting per se is fine. `repro/t5c.json` bisection: spigot radius
14 → 2.0, radius 15 → **0.0**.

Isolated with `tests/zz_scratch_probe.rs` (deleted before finishing): 216 of the
812×380 triangle pairs return distance 0. The first offender is
```
A [(-9.500251, 14.21813, 20.0), (-9.500251, 14.21813, 15.0), (-9.500251, 14.21813, 10.0)]
```
— three COLLINEAR points, i.e. a zero-area triangle from the bore wall's
subdivided seam. In `triangle_triangle_distance` the first thing done is
`tri_tri_intersect(a, b)`; for a degenerate triangle `n1 = 0`, so the
plane-side rejection never fires and `dir.length() <= REL*n1len*n2len`
degenerates to `0 <= 0` → the CO-PLANAR branch is taken with a zero normal and
returns true → distance 0.

So T5 is not "out of scope for the current algorithm". It is a robustness bug in
`crates/kernel-core/src/mesh/measure.rs` (MY file).

### T15 — `geometric_ok:false` is a TRUE POSITIVE, and the campaigns were wrong to ignore it
`repro/t15.json` is turgo F1 verbatim. Probed with `zz_scratch_probe.rs`:
32 proper triangle crossings in `tessellate_default`, and
`tessellate_adaptive_tol(.., 0.05).has_self_intersection() == true` as well —
i.e. **the STL that `export_stl` writes with `route:"exact", watertight:true`
really does contain crossing triangles.** The offending pairs all lie on ONE
planar annular end cap (all six vertices satisfy `n·p = 24.0` for the rotated
tube's cap plane), one of them a long bridge sliver: the hole-bridging
triangulation of a planar face with an inner loop overlaps itself.
Same FAMILY as T6(b)/(c) — planar-face-with-inner-loop tessellation.
Actionable fix in my scope: make `validate` say WHERE.

---

## Stage 2 — design decided

**T10 — one assertive vocabulary, not eight.** Added a UNIVERSAL `require`
parameter accepted by every op, applied to that op's own measures. This is the
only design that does not duplicate op parameters (`envelope`, `build_dir`,
`flag_below`, `weld_tol`, `tol`) onto `assert`, and it works for every op that
exists now or later. `assert` keeps its current solid-topology vocabulary
unchanged (backward compatible) and additionally gains the `tol`/`weld_tol` knobs
T6(a) demands.

**T10 second half — mesh values bind.** `EnvValue::Mesh` added; every
mesh-valued op binds its result mesh, and the measure ops accept a mesh. The
implicit→exact contract is untouched: a mesh value is a mesh forever, and the
only mesh→B-rep route stays the explicit `solid_from_implicit` reverse bridge.

---

## Stage 3 — changes applied (all built, all tested)

### T10 — the universal `require` gate  (`crates/kernel-api/src/require.rs`, new)
`require` is accepted on EVERY op and checked against that op's OWN measures;
unmet ⇒ `assert_failed`, so the op (and the program) fails. Chosen over adding
keys to `assert` because the mandatory gates are measured by ops that own their
own parameters (`envelope`, `build_dir`, `flag_below`, `weld_tol`, the export
path) — putting them on `assert` would duplicate every one of those and
guarantee drift. Wired in `interp.rs::run_one` (one call site, so it applies to
present AND future ops), added to the unknown-param allow-list, advertised by
`describe` (`universal_params`, and appended to every per-op param list).

Grammar: scalar = equality; array = element-wise (`null` = don't care);
object = any of `equals` / `min` / `max` / `within{target,abs|percent}` /
`not_null`; keys may be dotted paths (`bbox.size`, `size.2`).
Refuses (never silently passes): empty `require`, a key naming no measure (the
refusal LISTS the real keys), a numeric bound on a non-numeric measure, an
unknown clause, `within` without exactly one of abs/percent, `require` on an op
with no measures. On success the measures gain a `required` echo.

### T10 second half — mesh values  (`interp.rs`)
`EnvValue::Mesh` added. Binding ops: `export_stl`, `export_3mf`, `import_mesh`,
`implicit`, `tpms`, `gyroid_block`, `shell`, `mesh_carve`, `hybrid_boolean`,
`export_threaded`. Mesh-accepting measures: `validate`, `volume`,
`bounding_box`, `mesh_components`, `support_report`, `clearance`,
`assert_disjoint`, `assert`. All measures gained a `source` field
(`"solid"` / `"mesh"`).
The implicit→exact contract is intact: no op turns a mesh into a B-rep;
`solid_from_implicit` remains the only (explicit, `route:"voxel"`-labelled)
reverse bridge. `assert genus/shells/exact_volume_within` on a mesh REFUSES
(`wrong_type`); `volume` on a leaky mesh REFUSES.

### T6(a) — `crates/kernel-core/src/mesh/mod.rs` + `program.rs`/`interp.rs`
- `component_count` was rewritten so the union-find nodes are VERTICES and the
  grid is only an accelerator: two vertices are merged iff their true distance
  is `<= weld_tol` (same cell or any of the 26 neighbours). Cell membership is
  no longer itself the merge rule, so `weld_tol` is an exact tolerance and the
  answer no longer depends on the part's absolute position in space.
  Deterministic: union-find connectivity is independent of cell visit order.
- `assert` gained `tol` and `weld_tol` (defaults identical to today), shared
  with `mesh_components` through one `connectivity_tolerances` helper.

### T6(b)/(c) — the trust guard  (`interp.rs::connectivity_measures`)
A connectivity count taken on a measurement surface that is NOT CLOSED is
counting faceter cracks. A bound solid is closed by construction, so the op now
REFUSES (`invalid_geometry`) with the boundary-edge count, the count it would
have reported, the cause, and two named alternatives. A bound MESH is never
refused (openness there is data, reported as `watertight:false`).
No pass→fail flips: every case that trips the guard read `components > 1`
before, i.e. already failed.

### T5 — `crates/kernel-core/src/mesh/measure.rs`
`triangle_triangle_distance` routes DEGENERATE (zero-area) triangles straight to
the feature distances instead of through the Möller predicate, which is
undefined on them. `clearance`/`assert_disjoint` also gained `tol`, `source`,
and a mandatory `overlap_volume_reason` whenever `overlap_volume` is null.

### T15 — `mesh/measure.rs` + `interp.rs`
`Mesh::self_intersection_witness()` (sweep-and-prune, ~O(T log T), deterministic
lowest pair). `validate.geometric_ok` is now DERIVED from it, so flag and
witness can never contradict; `tests/self_intersection_witness.rs` pins the new
predicate to the historic `kernel_brep::self_intersects` over a 14-solid battery.
Verdict: TRUE POSITIVE — reported as `self_intersection {triangles, point, pairs}`.

### T8 — `cone.top_radius` (frustum), `interp.rs` + `program.rs`
Additive; omitted or 0 = the historic true cone byte-for-byte. Built by revolving
the trapezoid so the lateral band keeps its exact `Surface::Cone` tag —
`exact_volume` is π-exact to 1e-12 relative, and STEP export stays analytic.
`top_radius == radius` REFUSES (that solid is a cylinder).

### One self-inflicted regression, found and fixed
The first cut of the T6 trust guard called `kernel_core::check_mesh` to get
`boundary_edges` — but `check_mesh` ALSO runs the self-intersection sweep, which
is the expensive half of that call and answers a question the connectivity gate
does not ask. That put an O(T log T) tri-tri sweep on every `mesh_components` /
`assert components` / `volume`-on-mesh. Replaced with the two edge-hash passes
that actually answer the question (`Mesh::boundary_edge_count`,
`Mesh::is_two_manifold`). `validate` still pays for the sweep — that is the op
whose whole job is the full diagnosis.

Not a regression (checked, pre-existing): `sweep_motion.json` in the wrist
campaign does not complete. It is `union_all` over 17 heavily-overlapping
translated copies of an imported STEP body plus 27 more `union`s — the T8
`union_all` pathology (cross-owner request 4). Its `assert`s are `shells: 2`
only, which tessellates nothing, so no code I touched runs on that path.

### Docs / generated
- `API.md`: new *Gating a program with `require`* and *Mesh values* sections; a
  `### clearance` reference section (the op had none, and it is the T5 measure);
  `union_all`'s non-termination hazard with the measured n=12/n=13 boundary;
  rewrites of `cone`, `validate`, `mesh_components` (incl. the
  components-vs-shells complementarity table) and `assert`.
  `tools/audit_docs.py`: 0 errors.
- `discover.rs` regenerated with `tools/gen_discover.py` (idempotent).

### Tests added
- `crates/kernel-api/tests/require_gate.rs` (6)
- `crates/kernel-api/tests/gate_oracles.rs` (10)
- `crates/kernel-api/tests/self_intersection_witness.rs` (3)
- `crates/kernel-core/src/mesh/measure.rs` unit tests (3)

---

## Cross-owner requests (do NOT edit these myself)

1. **`crates/kernel-brep/src/tessellate_adaptive.rs`** — `face_boundary()` (line
   ~206) ignores `solid.face(f).inner`. Fix: build the inner loops' polylines
   from the same `EdgePoints` and hand outer+holes to the hole-aware planar
   triangulator that `crates/kernel-brep/src/tessellate.rs` already has
   (`tessellate_planar_with_holes`, line ~234). Repros above. Blast radius:
   `mesh_components`, `support_report`, `clearance`, `assert_disjoint`, and the
   `export_stl` route (a 40x20x5 plate with ONE square hole cannot take the
   exact route today). Once fixed, my T6 trust guard simply stops firing.
2. **`crates/kernel-brep/src/tessellate.rs`** — the hole-bridging planar
   triangulation produces SELF-OVERLAPPING triangles on the annular end caps of
   a boolean result (T15 repro: 20-32 crossing pairs, all six vertices on the
   same cap plane `n.p = 24.0`). Needs a bridge-edge validity test or a proper
   constrained triangulation. This is a REAL defect that ships into STLs.
3. **`crates/kernel-core/src/meshcheck.rs`** — two self-intersection oracles
   disagree by construction: `self_intersection_count` canonicalizes vertex
   identity BY POSITION and skips degenerate triangles;
   `has_proper_self_intersection` uses RAW indices and keeps them. They should
   share one adjacency rule; and once they do, fold
   `has_proper_self_intersection` into `Mesh::self_intersection_witness` so
   there is one predicate rather than two kept in step by a test.
4. **`crates/kernel-brep` booleans** — `union_all` non-termination, bisected:
   folding 13 mostly-disjoint cutters, n=9 0.11 s / n=10 0.22 s / n=11 0.42 s /
   n=12 0.85 s / **n=13 no completion in 100 s**. Cost doubles per body and then
   blows up. A BALANCED (tournament) fold of the same 13 hangs identically, so
   it is the arrangement over a many-shell accumulator, not the fold order;
   every individual pair unions in <1 s. Repro:
   `scratchpad/repro/t8_unionall.json` (regenerate from the fixlog recipe).
   Documented as a hazard in API.md `union_all` meanwhile.
5. **`docs/ROBUSTNESS.md`** — the two T8 items I could not root-cause in the
   timebox and did not touch: `difference` refusing a solid carrying both end
   chamfers and a vertical-edge fillet (singulator F7/F10, cleat F2), and a
   0.04 mm change to one cutter making a 27 mm-distant later boolean fail
   `validate` (cubesat F1/F2). Both are arrangement-robustness, not API.
6. **`campaign/DELIVERABLE_SPEC.md` §2.2** — its stated rationale is inverted on
   this kernel. `components` is not strictly stronger than `shells`; they are
   complementary (see the table now in API.md `mesh_components`). Also §2.4-§2.7
   should now point at `require`, which makes those gates expressible.
