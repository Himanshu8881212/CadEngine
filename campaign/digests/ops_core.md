# ops_core.md — Exact-B-rep Work-Order Cookbook (LMCAD `kernel-api`)

Digest of DESIGN_GUIDE.md §1–§10 + §21/§23 and API.md, cross-checked against the
live binary on 2026-08-06 (`describe` reports **160 ops**). Every snippet marked
"VERIFIED" was executed against
`"/Users/himanshu/Work/New-LMCAD/cad engine/target/release/kernel-api"` during
digest preparation.

> **DOC-DRIFT WARNING (verified live):** DESIGN_GUIDE Parts I–II are stale in
> three places. The binary HAS ops the guide says don't exist:
> `loft`, `sweep` (guide §5.4 says "Document-only" — wrong, both run as JSON ops),
> `linear_pattern`, `polar_pattern`, `mirror`, `rotate_x`, `rotate_y` (guide §9.2
> says "no B-rep array op by design" — wrong), and `bounding_box` +
> `measure_dimension` (guide §10.4 says "no bounding-box or linear-dimension
> measure" — wrong). API.md is the accurate op reference; when the two disagree,
> trust API.md, and when in doubt run `{"op": "describe", "name": "<op>"}` —
> that catalogue is compile-forced complete and cannot drift.

---

## 1. Running programs

```bash
# from anywhere (QUOTE the path — it contains a space):
"/Users/himanshu/Work/New-LMCAD/cad engine/target/release/kernel-api" run program.json --out-dir out/
"/Users/himanshu/Work/New-LMCAD/cad engine/target/release/kernel-api" asm assembly.lmcasm --out-dir out/asm
```

- Report is **stdout JSON**; exit code **0 iff every op succeeded**. Parse the
  report, never logs.
- Top level: `{"api_version": "cadcode.v1", "ok": bool, "ops": [...]}` (VERIFIED).
- Per-op entry: `id` (always; `$program` for file-level failures), `ok`,
  `measures` (op-specific), `file` (exports), `error: {"kind", "message"}` on
  failure.
- Execution **stops at the first failing op** — one root-cause error, no cascades.
- `asm` flags: `--base-dir` (source resolution), `--tol 0.05`, `--voxel 0.4`,
  `--window 1.0`.

## 2. Program envelope & grammar rules

```json
{"ops": [
  {"id": "plate", "op": "box", "min": [0, 0, 0], "max": [60, 40, 8]},
  {"id": "vol",   "op": "volume", "in": "plate"},
  {"id": "out",   "op": "export_stl", "in": "plate", "file": "plate.stl"}
]}
```

- Ops run in order; each `id` unique (`duplicate_id` otherwise); later ops
  reference earlier via `in` / `a` / `b` / `sketch` / `in: [ids]`.
- **Only geometry-producing ops bind.** Measures, exports, asserts,
  design-math lookups bind nothing — referencing them is a loud `missing_ref`.
- Units **mm**; angles on the JSON surface are **degrees** everywhere
  (`degrees`, `*_deg`). Bores/shanks are **diameters**; hex sizes across flats.
  (The `.lmcpart` Document grammar is the exception: `CircularPattern.angle`
  and `ExtrudeSketch.draft` are RADIANS.)
- `extrude` profiles must be **CCW** simple polygons (CW fails
  `invalid_geometry` loudly). `extrude_with_holes`, `extrude_tapered`,
  `revolve` and sketch sweeps re-wind automatically.
- **THE trap: unknown/misspelled fields are silently ignored.** A misspelled
  *optional* param leaves the default in force with exit 0 (verified in the
  guide: `"segemnts": 64` on a cylinder → 32-segment volume, no error).
  Misspelled *required* params are loud `invalid_param`. Defenses: gate on
  `exact_volume_within` (immune to faceting typos), assert any measure the
  param drives, or check the param exists via `describe`.
- Top-level unknown keys tolerated (e.g. `"_comment"`).
- Export `file` paths join `--out-dir` and are **confined** to it: absolute
  paths and `..` are refused `invalid_param`. `load_part` / import paths
  resolve against the **program file's own directory** (also confined).

## 3. Refusal semantics & error kinds

| kind | when | what to do |
|---|---|---|
| `parse` | not JSON / not `{"ops": [...]}` (id `$program`) | fix envelope |
| `unknown_op` | op name not in catalogue | check spelling; run `describe` |
| `duplicate_id` | reused id | rename |
| `missing_ref` | ref names nothing, or names a non-binding op | reference a binding op |
| `wrong_type` | e.g. sketch where solid needed | `sketch_extrude` first |
| `invalid_param` | missing/malformed required param; degenerate input; **EMPTY boolean result**; out-of-table standard size; unloadable `.lmcpart`; pattern caps | message names it; see below for empty booleans |
| `feature_failed` | fillet/chamfer witness matched nothing or edge out of scope | right op for the edge kind (§7 below); move witness; shrink radius |
| `sketch_failed` | constraints conflict (no convergence) or open profile | read residual + which dims disagree |
| `invalid_geometry` | op ran but result non-manifold/non-closed; or export leaky even after heal | wind CCW; refine voxel; check hazards |
| `admission_rejected` | `library_add` gate: a sampled param-range corner failed to build/rebuild deterministically | shrink declared ranges |
| `dependents_exist` | `library_remove` refused (assemblies reference it) | deprecate, or `"force": true` |
| `assert_failed` | declared expectation unmet — measured vs expected in message | fix geometry or wrong expectation; never delete the gate |
| `io` | unreadable/unwritable path | check paths |
| `internal` | caught kernel panic | report as engine bug |

**Empty-boolean doctrine (VERIFIED):** a disjoint `intersection` or a
`difference` that consumes everything is an **op failure** (`invalid_param`,
message "...produced an empty solid..."), never an empty solid — there is no
empty-solid value in this engine. To *prove* non-overlap positively, use
`assert_disjoint` (gives you the distance) or the exact route
`union`/`union_all` + `assert {"shells": N}` (tessellation-independent).

**`try_*` booleans do NOT exist on the JSON surface.** Every JSON boolean is
already checked (validate-gated; empty → failure). `try_union` /
`try_difference` / `try_intersection` / `try_*_sealed` / `ChainLog` /
`boolean_hazards` / `coalesce_coplanar` are **Rust-surface** APIs
(`kernel_brep`). The JSON shortcut for the hazard pre-scan is the
`coincident_fit` op (§9 below).

## 4. Solid constructors — 11 on the op surface

All validate their result and refuse to bind broken/empty solids. Curved walls
are faceted but every facet carries its exact analytic surface tag —
`exact_volume` and STEP read the tag; `volume` reads facets. Segments buy
silhouette only, never measurement accuracy.

Minimal valid JSON, one per constructor (all shapes verified against API.md;
loft/sweep/patterns executed live):

```json
{"id": "b",  "op": "box", "min": [0,0,0], "max": [60,40,8]}
{"id": "c",  "op": "cylinder", "base": [0,0,0], "axis": [0,0,1], "radius": 3.5, "height": 10, "segments": 32}
{"id": "s",  "op": "sphere", "center": [0,0,20], "radius": 6, "u": 32, "v": 16}
{"id": "co", "op": "cone", "base": [0,0,0], "axis": [0,0,1], "radius": 5, "height": 12, "segments": 32}
{"id": "t",  "op": "torus", "center": [0,0,0], "axis": [0,0,1], "major": 20, "minor": 5, "ring_segments": 48, "tube_segments": 24}
{"id": "e",  "op": "extrude", "profile": [[0,0],[30,0],[30,10],[10,10],[10,25],[0,25]], "height": 6}
{"id": "ew", "op": "extrude_with_holes", "outer": [[0,0],[40,0],[40,30],[0,30]], "holes": [[[10,10],[20,10],[20,20],[10,20]]], "height": 6}
{"id": "et", "op": "extrude_tapered", "profile": [[0,0],[30,0],[30,20],[0,20]], "height": 10, "draft_deg": 2}
{"id": "r",  "op": "revolve", "profile": [[10,0],[40,0],[40,7],[39,8],[10,8]], "segments": 64}
{"id": "l",  "op": "loft", "sections": [[[-20,-20,0],[20,-20,0],[20,20,0],[-20,20,0]], [[-10,-10,30],[10,-10,30],[10,10,30],[-10,10,30]]]}
{"id": "sw", "op": "sweep", "profile": [[-4,-4,0],[4,-4,0],[4,4,0],[-4,4,0]], "path": [[0,0,0],[0,0,25],[20,0,25]]}
```

Constructor notes:
- Defaults: cylinder/cone segments 32; sphere u32/v16; torus 48/24; revolve 64.
- `extrude`: negative `height` extrudes down. `extrude_with_holes`: hole loops
  must lie **strictly inside** `outer` (see silent trap below); genus = hole
  count. `extrude_tapered`: **convex profiles only**, no holes.
- `revolve`: profile is `[radius, z]` pairs about **world Z**, r ≥ 0; isolated
  on-axis apex refused. Each profile edge carries exact cylinder/cone/plane tag.
- `loft` (op form): 3D sections, ≥ 2 sections, **same point count** (≥ 3),
  consistent winding, ordered along the loft; result is honestly faceted.
  `sweep` (op form): closed 3D profile ≥ 3 pts along open path ≥ 2 pts,
  rotation-minimizing frame; for helical pitch use implicit `helix_pipe`/`pipe`.
  VERIFIED live: loft frustum volume 28000.0, sweep bend 2464.5.
  (`LoftSolid`/`SweepSolid` also exist as Document features for parametric use.)

Gate catches (loud): zero-thickness box, zero axis, zero-height cone,
minor > major torus, CW extrude, self-intersecting tapered inset, on-axis
revolve apex. Gate does NOT catch (silent, gate = topology check only):
1. `sphere` `radius: -2` → binds an r=2 sphere (sign absolutized).
2. Concave profile + shallow draft in `extrude_tapered` → binds outside the
   documented domain.
3. Hole loop crossing `outer` → valid topology, **wrong volume** (can exceed
   the blank!).
Tripwire for all three: an `assert` volume window. **Measure what you mean.**

## 5. Sketches & the constraint solver

`sketch` solves on creation (Levenberg–Marquardt + rank-revealing DOF
analysis). Receipt: `residual`, `iterations`, `converged`, `dof` (=2×points),
`rank`, `free_dof` (=dof−rank), `redundant`, `state` ∈
`under_constrained` / `well_constrained` / `over_constrained`.

```json
{"id": "profile", "op": "sketch",
 "points": [[1,2],[55,3],[58,44],[-2,38]],
 "segments": [[0,1],[1,2],[2,3],[3,0]],
 "constraints": [
   {"kind": "fixed", "point": 0, "at": [0,0]},
   {"kind": "horizontal", "a": 0, "b": 1},
   {"kind": "distance", "a": 0, "b": 1, "distance": 60},
   {"kind": "vertical", "a": 0, "b": 3},
   {"kind": "distance", "a": 0, "b": 3, "distance": 40},
   {"kind": "horizontal", "a": 3, "b": 2},
   {"kind": "vertical", "a": 1, "b": 2}]},
{"id": "plate", "op": "sketch_extrude", "sketch": "profile", "height": 8}
```

11 constraint kinds (exact field names):
`fixed {point, at:[x,y]}` · `coincident {a,b}` · `horizontal {a,b}` (y_a=y_b) ·
`vertical {a,b}` (x_a=x_b) · `distance {a,b,distance}` · `parallel {a,b,c,d}` ·
`perpendicular {a,b,c,d}` · `equal_length {a,b,c,d}` ·
`tangent {line_a,line_b,center,radius_point}` · `angle {a,b,c,d,degrees}`
(magnitude — may settle ±) · `symmetric {a,b,line_a,line_b}`.

- Under-constrained is **allowed and labelled** — extrude still runs from seed
  positions. Discipline: drive `free_dof` to 0 before a dimension matters.
- Over-constrained/conflicting fails `sketch_failed` with residual + state.
- `circles: [{center, radius_point}]` — a single standalone circle extrudes to
  an **exact analytic cylinder** (tag-exact). Solver-driven dims are converged
  numerics: gate with windows ≥ 1e-6, never bit-equality.
- `arcs: [{a, b, center, ccw?}]` (center is a construction point) — arcs facet
  at sketch tessellation (not tag-exact).
- `sketch_revolve {sketch, segments?}` — sketch (x,y) read as (r,z), r ≥ 0.

## 6. Booleans

`union {a,b}` · `difference {a,b}` · `intersection {a,b}` ·
`union_all {in: [ids...]}` (≥ 2; folds in ascending AABB-contact-degree order since 2026-08-27 — mutually-disjoint operands merge first, the touch-everything operand arranges once, last; result identical, robustness against repeated re-arrangement of one face). Exact planar-arrangement booleans
with **persistent face naming** (fillet witnesses still resolve after cuts;
tags survive so `exact_volume` stays π-exact — but a bored plate measured 6e-5
relative off closed-form once, so volume-gate with a band, never equality).

- Coplanar contact IS supported (boss-on-plate, flush stacks: verified exact
  volumes, one shell). But a *forest* of coplanar partial overlaps is the
  thinnest-margin corner: prefer ≥ 0.1 mm embedment or exact coincidence —
  **never the sliver between** (~1e-6–0.1 mm gaps mint needle faces).
- Tool-building convention: overshoot cutters past both faces (drill from −1
  to height+1) so no coplanar membrane is left.
- Disjoint bodies keep their own shells through a union:
  `union_all` + `assert {"shells": N}` = exact N-body no-contact proof.
- Boolean hygiene (from the RESPOOL post-mortem, §7.7): keep cutter side
  planes OFF revolve facet meridians (`k·360°/segments`); pick `segments`
  divisible by pattern count; keep cutter edges out of coincident-plane
  overlap regions; valid ≠ tessellatable — check the export `route`.
- Pre-flight from JSON: `coincident_fit {a, b}` → bool. `true` = some face
  pair lies on nearly the same analytic surface (1e-3 rad / 0.05 mm) AND
  extents near — a hazard CLASS, not a verdict (flush stacks are true and
  fine; press fits are true and may hang the boolean). `false` ≠ safe.
- On an empty-result refusal: it's your poses/dimensions. Re-check with
  `clearance` or `assert_disjoint`, don't retry blindly.

## 7. Fillets, chamfers, hole wizard — scope limits

### `fillet_edge_near` / `chamfer_edge_near`
`{in, witness: [x,y,z], radius, max_distance?}`. No edge IDs cross the JSON
surface — you point NEAR an edge. Witness farther than `max_distance` (default
10% of bbox diagonal) from every edge → `feature_failed` (never grabs a far
edge). Get exact witnesses from `list_edges` (midpoints).

**Honest scope:** CONVEX straight edges between two PLANAR faces only.
Concave junctions (inside corners) refused for BOTH ops — model the cove
explicitly. Curved-face edges refused → use `fillet_circular_rim`.
Radius must fit both adjacent faces.

**Trap:** on boolean results, the first chamfer works, the NEXT may fail
("not a straight edge...") — re-tessellation fragments the flat faces.
**Ease edges on primitives first, boolean last.**

### `fillet_circular_rim`
`{in, witness, radius, arc_segments?=8}`. Exact torus band on a circular
**convex** rim (cylinder wall meets planar cap), incl. bosses fused by
boolean. Scope: convex rims only; full-circle cap, no inner loops; radius <
cap radius; wall ≥ radius. **Trap:** the rim picker has NO distance guard —
it snaps to the nearest *qualifying* rim silently (verified: witness at a
concave root filleted the top rim 12 mm away, exit 0). Check a measure after.

Persistence: op-surface witnesses re-resolve on every run (nearest-edge
semantics); Document `Fillet`/`Chamfer` features store persistent EdgeNames
that survive re-dimensioning — use those when the fillet must survive edits.

### Hole wizard — 7 cuts, real ISO/DIN tables
Shared: `at` on the entry face, `axis` INTO material, diameters everywhere,
cutters overshoot 0.5 mm, `segments?=32`, exact surface tags kept. Metric
table M2–M12 (countersink starts M3; off-table → loud `invalid_param`).

| op | key params | echo (the table row actually cut) |
|---|---|---|
| `drill` | `d`, exactly one of `depth` (blind, 118° point) / `through` | `kind`, `depth`+`point_depth` (point reaches DEEPER — plan walls on `point_depth`) or `through` |
| `clearance_hole` | `m`, `fit?` close/medium(default)/coarse — always cut through entire extent | `clearance_d` (ISO 273: M5 → 5.3/5.5/5.8) |
| `counterbore_hole` | `m`, `fit?` | `counterbore_d`, `counterbore_depth` (DIN 974-1; M5 → Ø10×5.8) |
| `countersink_hole` | `m` ≥ 3, `fit?` | `countersink_d` (DIN 74-1 form F 90°; M5 → Ø12.5) |
| `tap_drill_hole` | `m`, `depth`/`through` | `pilot_d` = m − pitch, `pitch`. Thread NOT modelled (real threads: `thread_ridge`+`export_threaded`, voxel route) |
| `bolt_circle` | `center`, `axis`, `circle_d` (BCD), `n`, `start_deg?=0`, `hole: {kind: "drill"\|"clearance"\|"counterbore"\|"countersink"\|"tap_drill", ...}` | `hole` echo with full table dims |
| `bearing_seat` | `bearing`: "603","608","625","688","6000","6001","6804" | `bore_d`, `outer_d`, `width`, `pocket_d`, `pocket_depth`, `shoulder_d` (608 → Ø22×7 pocket, Ø15 shoulder) |

Genus arithmetic: each THROUGH cut adds 1 genus; blind holes add none.
**Wizard has zero edge-proximity awareness** — a countersink tangent to a wall
raises nothing. Follow every wizard cut with `wall_thickness` + a volume window.

## 8. Placement & patterns

- `translate {in, offset: [x,y,z]}`
- `rotate_x` / `rotate_y` / `rotate_z` `{in, degrees}` — about the world axis
  through the ORIGIN.
- `pose {in, rotate?: {axis, degrees, center?}, translate?}` — arbitrary-axis
  rotation (about `center`, default origin) THEN translate; at least one part
  required (empty pose = `invalid_param`). Chain poses for composed rotations.
  A `.lmcasm` instance pose is exactly rotate+translate.
- `mirror {in, plane: {point, normal}}` — orientation-safe reflection.
  **Reflects in place; does NOT union with the original** — `union` the two
  ids for a symmetric part.
- `linear_pattern {in, count, step: [x,y,z]}` — count INCLUDES the original
  (instance 0 at offset 0); folded into ONE solid by exact union; disjoint
  clones = honest multi-shell (`shells == count`, verified). Zero step refused.
  Caps: count 2..=500, count × faces ≤ 100 000.
- `polar_pattern {in, count, center, axis, step_deg? = 360/count}` — same fold,
  same caps; 360°-multiple step refused. VERIFIED: 6 spokes → shells 6.
- All transforms re-validate and carry analytic tags.
- Hole patterns → `bolt_circle`. Parametric arrays that must re-evaluate →
  Document `LinearPattern`/`CircularPattern`/`Mirror` (`Mirror` there DOES
  union original+reflection; `CircularPattern.angle` is RADIANS). Repeated
  parts → `.lmcasm` instances / `asm_instance` ops.
- Pattern hygiene: keep copies from sharing face planes with EACH OTHER;
  copies seated ON a base plane are the supported coplanar case.

## 9. Measures & assertions — the complete receipt vocabulary

All measure ops bind nothing and need a bound solid (`wrong_type` on a sketch).
Provenance fields are carried per receipt (VERIFIED live).

| op | params | measures (provenance) |
|---|---|---|
| `validate` | `in` | `closed`, `manifold`, `euler_characteristic`, `genus`, `shells`, `valid` — RECORDS topology (every solid op already gates on it). Genus = through-tunnels: strongest one-number shape check |
| `volume` | `in` | `volume` (`provenance: "faceted"`) — exact for planar solids, segment-dependent on curved |
| `exact_volume` | `in` | `exact_volume` (`provenance: "analytic"`) — π-exact from surface tags; falls back to facets on untagged faces. The default volume gate; band, not equality, after booleans |
| `mass_properties` | `in` | `volume`, `center_of_mass`, `inertia_diag` [Ixx,Iyy,Izz], full `inertia_tensor` — UNIT density, about CoM in model axes; analytic 2nd moments for cyl/sphere/cone faces (torus: tessellation-level) |
| `bounding_box` | `in`, `envelope?: [x,y,z]` | `min`, `max`, `size`, `center`, `diagonal`, + `fits_within`, `fits_within_rotated` with envelope — the "fits the printer bed" check |
| `measure_dimension` | `in`, `kind: "point_point"\|"face_face"\|"diameter"`, `a`/`b` (points or face witnesses), `near` (diameter witness) | `value`, `provenance` (`analytic` for face_face/diameter from plane eqns / surface tags; `coordinates` for point_point), face descriptors. Non-parallel/non-planar face_face and cone/torus diameter are LOUD `invalid_param`, never a wrong number |
| `wall_thickness` | `in`, `flag_below` (required) | `min_thickness`, `p05_thickness`, `median_thickness`, `thin_area`, `sampled_triangles`. Judge by `thin_area` + percentiles; `min_thickness` is oblique-corner-ray noise |
| `draft_analysis` | `in`, `pull`, `min_deg` | `min_draft_deg`, `low_draft_area`, `undercut_area`; walls parallel to pull = 0° |
| `coincident_fit` | `a`, `b` | `coincident_fit` (bool) — near-coincident-face hazard CLASS pre-scan (1e-3 rad / 0.05 mm), O(faces²), safe on pairs that would hang a boolean |
| `clearance` | `a`, `b`, `tol?` | `distance`, `interfering` (bool), `overlap_volume` (mm³), `coincident_fit_hazard`, `provenance: "faceted"` — the interference measure that does NOT fail on overlap (VERIFIED: overlapping boxes → interfering true, overlap_volume 27.0, exit 0). **`distance` is only trustworthy for SEPARATED pairs — see §11b for the nested-pair failure and the blessed fallback** |
| `support_report` | `in`, `build_dir? = [0,0,1]`, `overhang_deg? = 45` | `support_free`, `bed_area`, `bridge_area`, `steep_area`, `total_area`, `max_bridge_span`, `provenance: "faceted"`. One orientation per call; areas only, no locations. **`describe` ships EMPTY `doc` strings for both params — the semantics below are measured, not documented by the binary.** See §11a |

Discovery (bind nothing, VERIFIED):
- `describe` → no-arg: `count: 160` + all op names; `{name}`: `params:
  [{name, type, required, doc}]`, `exists: false` for typos. **The
  authoritative param source — use it before using any op you haven't run.**
- `list_faces {in}` → `faces: [{index, type: plane|cylinder|sphere|cone|torus,
  descriptor (exact surface), witness (centroid — feed to measure_dimension /
  asm_mate_*), area (facet polygon; null for curved)}]`.
- `list_edges {in}` → `edges: [{index, midpoint, length, curved}]` — midpoint
  is a ready-made fillet/chamfer witness (`witness_distance: 0.0`). Counts are
  the topology the kernel HOLDS (booleans add seam edges; a bored plate reads
  44, not 36).

### Assertions (enforce; `assert_failed`, exit 1)
- `assert {in, ...}` — at least one check (empty = `invalid_param`). Checks:
  `volume_within` / `exact_volume_within` (each `{"target", "abs"|"percent"}`
  — exactly one tolerance form), `genus` (int), `shells` (int), `closed` /
  `manifold` / `valid` (bool). All present checks evaluated; every failure
  listed in one message; on pass the measured values are echoed.
- `assert_disjoint {a, b, min_clearance?=0, tol?=0.01}` — passes iff measured
  surface distance EXCEEDS `min_clearance`; reports `distance`. Measured on
  raw exact tessellations, accurate ~`tol`: keep `min_clearance` ≳ `tol` for
  hard proofs; **for a NESTED pair it FALSE-FAILS — see §11b** — so for any
  tessellation-independent or enclosed-geometry proof use `union` + `assert
  shells`.

### Assertion patterns worth copying
- **Assertion-first**: write the gates (topology + closed-form volume window +
  wall_thickness + clearances) before the geometry; build until exit 0. The
  acceptance criteria ride with the design.
- **Witness fixture**: to prove a Ø5.2 bore at exactly z=14, pose a Ø5.0×40
  gauge pin on that axis, `union`, `assert shells == 2` — one assertion proves
  diameter AND position AND that the bore is open. (bounding_box now exists,
  but the fixture remains the strongest placement proof.)
- **Mesh interleave**: pose two gears at center distance, union,
  `assert shells == 2` — teeth interleave without contact.

### Silent failure modes → tripwires (none flip the exit code)
| silent mode | tripwire |
|---|---|
| misspelled optional param → default used | `exact_volume_within` / assert the driven measure |
| negative sphere radius → absolutized | volume window |
| hole loop crossing outer → wrong volume, valid topology | volume window |
| rim fillet snapping to a far qualifying rim | measure after the fillet |
| wizard cut tangent/too close to a wall | `wall_thickness` + volume window |
| edge features after booleans failing on fragmented planes | ease primitives first, boolean last |
| valid B-rep, leaky tessellation | check export `route`/`watertight` receipt |
| assembly instance export `watertight: false` with exit 0 | gate on the per-instance receipt yourself |

## 10. Exports & imports

**Export and import paths do NOT share a root.** Both are sandboxed and both
refuse `..` and absolute paths — but they resolve against *different*
directories, which is the direct cause of "reproducing does not reproduce":

| direction | ops | resolves against |
|---|---|---|
| **out** | `export_stl`, `export_3mf`, `export_step`, `export_threaded`, `mesh_carve.out`, `library_*` `dir` | **`--out-dir`** |
| **in** | `import_step`, `import_mesh`, `load_part`, `mesh_carve.file` | **the PROGRAM file's own directory FIRST, then `--out-dir`** (fallback added 2026-08 — the T4 heal; a total miss names BOTH tried roots) |

Parents are created on the out side; report `file` = the path actually written.
Verified 2026-08-08: a program in `prog/` run with `--out-dir out/` writes
`out/b.step`, and a second program in `prog/` doing
`import_step {"file":"b.step"}` fails
`io: cannot read '<…>/prog/b.step': No such file or directory`, while
`{"file":"../out/b.step"}` fails
`invalid_param: path '../out/b.step' must not contain '..' (it would escape the sandbox)`.

**A §2.12 STEP round-trip CAN now be written as export-then-import with the
same relative name** (the import falls back to `--out-dir` when the file is
not beside the program — the T4 heal). Program-relative inputs keep priority,
so a file that exists in BOTH places resolves beside the program; keep the
names distinct when that ambiguity matters.

Note also that `io` and `invalid_param` messages **echo the resolved ABSOLUTE
path**, so reports containing them are not byte-comparable across machines or
across two different `--out-dir`s.

| op | params | measures | semantics |
|---|---|---|---|
| `export_stl` | `in`, `file`, `tol?=0.01`, `voxel?=0.3` | `route`, `triangles`, `watertight` | binary STL. **Route is honest**: exact adaptive tessellation at `tol`; watertight → `"route": "exact"`; leaky → winding-number-SDF heal remesh at `voxel` → `"route": "voxel_healed"`; STILL leaky → op FAILS `invalid_geometry` (a program never writes garbage) |
| `export_3mf` | same | same | same mesh routing, 3MF (mm units explicit) |
| `export_step` | `in`, `file` | — | STEP **AP203** with EXACT analytic surfaces (plane/cylinder/sphere/cone/torus, circular edges as CIRCLE) — not a mesh; no tessellation, no routing. Product name = file stem. Untagged faces export as planar patches |
| `export_threaded` | `in`, `m`, `length`, `z0?`, `internal?`, `voxel?=pitch/8`, `file` | `route`, `volume_delta_vs_body`, ... | the ONLY way to fuse/cut a real ISO thread (exact union would self-intersect). Thread axis is world +Z through origin. `voxel` > pitch/6 refused. Internal is a print-practical male-form+0.4mm-crest-clearance approximation, NOT ISO female form |
| `import_step` | `file` | `shells`, `genus`, `faces`, `volume`, `freeform_faces` | BINDS an exact B-rep (tags kept); multi-solid merges to one multi-shell solid |
| `import_mesh` | `file` (.stl/.obj/.3mf/.ply), `heal?`, `out?` | full check_mesh receipt; `volume` only iff watertight | **binds nothing** — meshes never enter the solid environment |
| `mesh_carve` | `in`, `file`, `bool`, `voxel?=0.3`, `out` | `route: "voxel_implicit"`, ... | boolean a solid vs a mesh FILE through the voxel half; writes a file, binds nothing |

Program exports fail on leaky; **assembly instance exports do NOT** (receipt
carries `watertight: false`, exit stays 0 — your policy layer must gate).
OBJ/glTF/AP242: Rust surface only.

## 11. `load_part` + minimal `.lmcpart`

```json
{"id": "spacer", "op": "load_part", "file": "spacer.lmcpart"}
```
Relative path resolves against the PROGRAM FILE's directory. Measures: `name`,
`units`, `created_with`. Refused (`invalid_param`) if the recipe's root is a
voxel-half feature (shell/gyroid/smooth booleans can't enter the solid
environment). Minimal hand-written recipe (all numbers are `{"Literal": x}` or
`{"Param": "name"}` Dims; feature refs are indices; `Boolean.op` is
`"Union"|"Difference"|"Intersection"`):

```json
{"format": "lmc-part", "version": 1, "units": "mm", "name": "spacer",
 "created_with": "by hand",
 "document": {
   "params": {"h": 8.0},
   "features": [
     {"Box": {"center": [{"Literal": 15.0}, {"Literal": 10.0}, {"Literal": 4.0}],
              "size": [{"Literal": 30.0}, {"Literal": 20.0}, {"Param": "h"}]}, "label": "blank"},
     {"Cylinder": {"center": [{"Literal": 15.0}, {"Literal": 10.0}, {"Literal": 4.0}],
                   "radius": {"Literal": 4.0}, "height": {"Literal": 12.0}}},
     {"Boolean": {"op": "Difference", "a": 0, "b": 1}}],
   "root": 2, "suppressed": []}}
```
Note `Box` is CENTER+SIZE here (op-surface `box` is min/max) — one of the
Document grammar's three shape asymmetries (with radians in `CircularPattern.
angle` and `ExtrudeSketch.draft`).

## 11a. `support_report` — the measured semantics (`describe` ships empty docs)

`describe {"name":"support_report"}` returns `build_dir` and `overhang_deg`
with `"doc": ""`. Everything below was measured on
`target/release/kernel-api`, 2026-08-08. Orientation prose was wrong in four
campaigns — one shipped a render of the wrong bed — so read this before you
write any "prints support-free in orientation X" sentence.

**`build_dir` points AWAY from the bed.** It is the layer-growth direction, so
`build_dir: [0,0,1]` puts the bed at **min-Z** and `bed_area` counts faces
whose outward normal is anti-parallel to it. Verified on an L-bracket (5×10
foot at z = 0, a 15×10 arm roofed at z = 20):

| `build_dir` | `bed_area` | `bridge_area` | reading |
|---|---|---|---|
| `[0,0,1]` | **50.0** (the 5×10 foot) | 150.0 (the arm underside) | bed at min-Z |
| `[0,0,-1]` | **200.0** (the two top faces) | 0.0 | bed at max-Z |

`render_sheet`'s `build_dir` uses the same convention (it "rotates the part so
this vector points +Z and rests it on a drawn bed"), which is the cross-check:
if your `render_sheet` bed view looks upside-down, your `support_report`
orientation claim is also backwards.

**`overhang_deg` — a LARGER value is MORE permissive** (default **45**). A
downward-facing face lands in `steep_area` iff its **tilt from `build_dir`
EXCEEDS `overhang_deg`**. Verified on lofted square frusta of known wall tilt:

| wall tilt from `build_dir` | steep at `overhang_deg` … | clean at `overhang_deg` … |
|---|---|---|
| 45.0° | ≤ 44 | ≥ 45 |
| 63.435° | ≤ 63 | ≥ 64 |
| default (unset) | steep at 46° tilt, clean at 44° tilt → **default = 45** | |

The comparison is strict on f32, so `overhang_deg` equal to a modelled face
angle sits exactly on a knife edge. **Never set `overhang_deg` to a modelled
face angle**, and treat any reading within 1° of one as unresolved. A "second,
stricter reading" is a **smaller** number — one campaign shipped 50° and 60°
believing they bracketed its 45° gables; both are on the permissive side and
the pair proved nothing.

**Face classes** (three disjoint subsets of `total_area`, not a partition of
it — upward and vertical faces are in none of them; only `steep_area` is the
support gate):
- `bed_area` — faces flat against the bed (normal = −`build_dir`, at the
  minimum extent along `build_dir`);
- `bridge_area` — downward faces at 90° tilt (horizontal roofs/undersides) that
  are not on the bed;
- `steep_area` — downward faces tilted more than `overhang_deg` from
  `build_dir`. A "support-free" claim requires `steep_area == 0.0` **exactly**.
  Note a horizontal underside is `bridge_area`, **not** `steep_area`, so
  `support_free: true` does **not** mean "no bridging" — quote
  `max_bridge_span` alongside it, always.

**`max_bridge_span` is the SHORT way across**, i.e. the minimum bounding-box
extent of the bridging region — the distance the printer actually has to span.
Verified: a deck between two posts leaving a 30 × 8 mm underside reports
`8.0`; the same deck leaving 30 × 50 mm reports `30.0`. For a true **cantilever**
this under-reads the unsupported reach (a 15 × 10 mm cantilevered arm reports
`10.0`, not 15) — a cantilever is not a bridge and needs its own judgement.

## 11b. `clearance` on NESTED pairs — fixed, but `faceted` and therefore an UNDER-read

`clearance` used to report `distance: 0.0` for nested, interlocked, enclosed
and coaxial pairs with a real gap, and `assert_disjoint` false-FAILED them
(six campaigns routed around it). **Fixed 2026-08-08.** Re-verified on the
case that broke it — a Ø11.4 pin coaxial inside a Ø12 bore, a true 0.300 mm
radial gap:

```
clearance(tube, pin)       -> {"distance": 0.2711080312728882, "interfering": false,
                               "overlap_volume": 0.0, "coincident_fit_hazard": false,
                               "provenance": "faceted"}
assert_disjoint(tube, pin) -> PASSES   (it used to fail this pair)
clearance(tube, far_box)   -> {"distance": 147.83, ...}
```

**Quote it with its provenance.** 0.2711 against a true 0.300 mm is a
**−9.6 %** under-read, because the measure runs on inscribed polygonal facets;
the error scales as `r·(1 − cos(π/n))` ≈ 0.029 mm here. That is the
*conservative* direction for a clearance claim, so publish it as-is for
"does it clear at all" — but it is not the analytic gap, and `tol` does not
materially move it (0.271108 at default vs 0.271108 at `tol` 0.001).

**When the number must be ANALYTIC — the grown-gauge bracket.** Still the
strongest available measure, and the only one with `analytic` provenance.
`intersection` on a genuinely
disjoint pair refuses (`invalid_param: intersection produced an empty solid`),
so the refusal is itself a machine-checkable receipt. Grow a copy of the moving
body by δ and bracket the clearance between a δ that still refuses and a δ that
binds. Verified on the same pin/bore:

| gauge radius | δ vs the pin | result | reading |
|---|---|---|---|
| 5.99 | 0.29 mm | `intersection` **refused** (`invalid_param`) | radial clearance **> 0.29 mm** |
| 6.01 | 0.31 mm | binds; `exact_volume` = 6.0369 mm³, `provenance: analytic` | radial clearance **< 0.31 mm** |
| 6.05 | 0.35 mm | binds; `exact_volume` = 30.2850 mm³ | (monotone — a sanity check on the bracket) |

Ship both programs and both reports; the refusing one exits 1 and its report
is the evidence, not a failure to hide. The result is `[0.29, 0.31]` mm with
**analytic** provenance — tighter than the faceted 0.2711 and on the right
side of the truth. Use the faceted `distance` for "does it clear"; use the
bracket whenever a few percent decides the fit.

## 12. Design-math lookups (bind nothing; numbers in `measures`)

`iso286_fit {d ≤ 120, fit: "H7/g6"|"H7/h6"|"H7/k6"|"H7/n6"|"H7/p6"|"H7/s6"|"H8/f7"}`
→ `hole`/`shaft`/`clearance` `[lower, upper]` mm (negative clearance =
interference) · `thread_spec {m: 3..16 coarse}` → `pitch`, `minor_d`,
`tap_drill_d` · `heatset_spec {m: 2..6}` → `pilot_d`, `pocket_depth`, `boss_d`
(Ruthex) · `gt2_belt {center_distance, t1, t2}` → `pitch_length`, `belt_teeth`
· `gt2_center_distance {belt_teeth, t1, t2}` · `metric_cord_gland {cord_d}` ·
`racetrack_cord_length {x_len, y_len, corner_r}` · `pipe_thread_g
{designation: "G1/8".."G1/2"}`. Also 48 catalog part constructors (`spur_gear`,
`hex_bolt`, `deep_groove_bearing`, `nema_motor`, `shaft`, ...) and 13 standard
feature cuts (`teardrop_hole`, `bridged_counterbore`, `heatset_insert_boss`,
`o_ring_groove`, `nema_mount_cut`, ...) — check the catalog BEFORE modelling a
standard part. In-program assembly ops exist too (`asm_instance`, `asm_mate`,
`asm_mate_axis`/`asm_mate_face`, `asm_solve`, `asm_contacts`,
`asm_interference_volume`, `asm_mass_properties`, `asm_export`,
`asm_export_step`, `asm_save`, `gear_train_poses`).

## 13. Canonical gated-part skeleton (copy this shape)

```json
{"ops": [
  {"id": "wall", "op": "box", "min": [0,0,0], "max": [40,40,12]},
  {"id": "seat", "op": "bearing_seat", "in": "wall", "at": [20,20,12], "axis": [0,0,-1], "bearing": "608"},
  {"id": "bolts", "op": "bolt_circle", "in": "seat", "center": [20,20,12], "axis": [0,0,-1],
   "circle_d": 31, "n": 4, "start_deg": 45, "hole": {"kind": "clearance", "m": 3}},
  {"id": "gate_topology", "op": "assert", "in": "bolts", "genus": 5, "valid": true},
  {"id": "gate_volume", "op": "assert", "in": "bolts", "exact_volume_within": {"target": 15219.7, "percent": 0.1}},
  {"id": "gate_walls", "op": "wall_thickness", "in": "bolts", "flag_below": 2},
  {"id": "gate_print", "op": "support_report", "in": "bolts"},
  {"id": "stl", "op": "export_stl", "in": "bolts", "file": "bearing_wall.stl"}
]}
```
(Executed in the guide: exit 0, exact_volume 15219.6964 vs closed form
15219.70 — write the closed-form target yourself.)

## 14. Deep-dive pointers

| topic | where |
|---|---|
| mental model, receipts doctrine | DESIGN_GUIDE §1 |
| quickstart work orders (catalog, sketch, recipe+wizard, library, gearbox) | DESIGN_GUIDE §3 |
| grammar rules, verified | DESIGN_GUIDE §4; API.md "The program model" |
| constructors + degenerate traps | DESIGN_GUIDE §5 (§5.2 misspell trap, §5.3 traps); API.md "Solid constructors" (incl. loft/sweep OPS — guide §5.4 is stale) |
| sketch solver, constraint table | DESIGN_GUIDE §6; API.md "Sketch ops" |
| booleans, shells proofs, coplanar history, hygiene checklist | DESIGN_GUIDE §7 (esp. §7.7); API.md "Booleans" |
| fillet/chamfer/rim scope + hole wizard tables | DESIGN_GUIDE §8; API.md "Features & transforms", "Hole wizard" |
| placement, patterns (op-surface patterns: API.md ONLY) | DESIGN_GUIDE §9; API.md `mirror`/`linear_pattern`/`polar_pattern` |
| measures/assertions, witness fixture | DESIGN_GUIDE §10; API.md "Measures", "Assertions", "Discovery & introspection" |
| implicit half (lattices, threads, expr_sdf, pillow trap) | DESIGN_GUIDE §11–§15; API.md "Implicit / hybrid" |
| `.lmcpart` Document grammar (all 30 features, radian corners) | DESIGN_GUIDE §16; API.md "Native formats" |
| hybrid fuse, voxel-size table, complexity rails | DESIGN_GUIDE §17 |
| assemblies `.lmcasm`, mates, BOM, contacts | DESIGN_GUIDE §18; API.md "The assembly surface", "Assembly ops (in-program)" |
| library admission gate | DESIGN_GUIDE §19; API.md "Parts library" |
| catalog (48 parts, 13 cuts, 7+ lookups) | DESIGN_GUIDE §20; API.md "Standard parts catalog" onward |
| exports/STEP scope | DESIGN_GUIDE §21; API.md "Exports" |
| print-readiness method | DESIGN_GUIDE §22 |
| failure playbook (verbatim error text) | DESIGN_GUIDE §23 |
| limits ledger / known frictions | DESIGN_GUIDE §24; docs/FRICTION.md |
| ACE voxel-physics bridge (`sample_density_grid`/`mesh_density_grid`) | API.md top; tools/ runners |

---

## ENGINE UPDATE 2026-08-06 (orchestrator) — two new first-class fences

The catalogue is now **161 ops**. Two general fixes landed before the campaigns:

1. **`mesh_components` measure + `assert {"components": N}`** — the single-body
   oracle, and the **second half** of the connectivity gate.
   `{"id":"mc","op":"mesh_components","in":"part","tol"?:0.05,"weld_tol"?:0.001}`
   returns `{components, is_one_body, triangles, tol, weld_tol,
   provenance:"faceted"}`.

   **Gate BOTH `shells` and `components`.** The claim in earlier revisions that
   "`shells==1` cannot catch this class" is **WRONG and is retracted** —
   measured 2026-08-08 on `target/release/kernel-api`:

   | construction | `validate.shells` | `mesh_components.components` | who fires |
   |---|---|---|---|
   | bar cut clean through by `difference` | 2 | 2 | **both** |
   | two boxes, 0.0005 mm gap at x=10, `union_all` | 2 | **1** | **only `shells`** |
   | `extrude_with_holes`, outer + 2 hole loops | 1 (genus 2, valid) | **REFUSES** `invalid_geometry` | only `shells` is usable |

   So the two are **complementary oracles measured on different objects**:
   `shells` is exact-B-rep topology, `components` is a connectivity walk on the
   *faceted* mesh — the object your STL, slicer and voxel solvers actually see.
   - What `shells` catches that `components` misses: **any sever narrower than
     the weld scale.** `mesh_components` welds near-coincident vertices first,
     so a sever thinner than `weld_tol` (default 0.001 mm) is welded shut and
     invisible. Measured at bases x = 0 / 10 / 100: welded strictly below
     0.001 mm, separate strictly above; exactly AT 0.001 the f32 mesh
     coordinates decide (1 / 2 / 1 respectively), so never design a feature
     whose correctness depends on the boundary itself.
   - What `components` catches that `shells` misses: disconnection that exists
     only in the tessellation, and severance surviving routes with no B-rep
     shell structure.
   - **When it cannot be trusted it REFUSES** (fixed 2026-08-08 — it used to
     over-count one component per `extrude_with_holes` hole loop, and many
     after `import_step`). On a planar face carrying inner/hole loops the
     measure now fails `invalid_geometry`: *"tessellating it at tol 0.05 mm
     left 16 boundary edges (28 triangles), so the measurement surface is NOT
     closed and its component count (3) counts faceter cracks, not severed
     bodies … Gate this part with `validate` (closed / manifold / shells)
     meanwhile, and/or `export_stl` it and run this measure on the export's
     bound mesh — the exported mesh IS what prints."* Do exactly that: keep
     `shells:1` live and re-run the walk on the exported mesh. Record the
     refusal verbatim; never delete the gate.

   `tol` (chord tolerance of the measurement tessellation, default 0.05) and
   `weld_tol` (default 0.001, "the house weld scale") are real parameters of
   the **measure** — verified: `weld_tol 0.0001` turns the 0.0005 mm pair from
   `components 1` into `components 2`, and the receipt echoes the value used.
   `assert` takes neither, so `assert components:1` always runs at the
   defaults. **To gate at a tightened tolerance, use `require` on the measure
   itself** (§9 "the universal gate"):

   ```json
   {"op":"mesh_components","in":"part","weld_tol":0.0001,"require":{"components":1}}
   ```
   Verified: on the 0.0005 mm pair this fails with
   `assert_failed: require failed: components: measured 2, expected 1`, exit 1
   — while the plain `assert components:1` beside it passes.

   The two constructible oracle-negative-controls (one per direction) are in
   `campaign/DELIVERABLE_SPEC.md` §2.13; both are verified to exit 1.
2. **Unknown-param warnings** — a report entry now carries
   `"warnings": ["unknown param 'segmnets' — 'cylinder' does not accept it, …"]`
   when an op is given params it does not accept. Non-fatal (exit unchanged);
   `_`-prefixed keys (in-op comments) never warn. Campaign rule: ship with
   ZERO warnings.

Both verified by `crates/kernel-api/tests/measure.rs` (last two tests) and live
smoke; clean reports keep their exact historical bytes (determinism holds).
