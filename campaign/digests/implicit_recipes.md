# Implicit / Hybrid / Recipe / Assembly Cookbook (LMCAD)

Digest of DESIGN_GUIDE.md §11–§20 for design agents. Every JSON block marked
**[VERIFIED]** was executed against the release binary on 2026-08-06; quoted
error messages are the binary's own. Source docs: `DESIGN_GUIDE.md` (repo
root), `API.md` (op parameter tables). Deep-dive pointers at the end.

---

## 0. How to run anything (get this right first)

```
"/Users/himanshu/Work/New-LMCAD/cad engine/target/release/kernel-api" run <program.json> --out-dir <dir>
"/Users/himanshu/Work/New-LMCAD/cad engine/target/release/kernel-api" asm <assembly.lmcasm> [--base-dir DIR] [--out-dir DIR] [--tol MM] [--voxel MM] [--window MM]
```

- The subcommand (`run` / `asm`) is **required** — bare `kernel-api program.json`
  exits 2 with a usage message. The repo path contains a space: always quote it.
- Output JSON goes to stdout; `ok: true/false` per op and overall; exit 0/1.
- **Path resolution rules** [VERIFIED]:
  - `load_part.file` resolves relative to the **program JSON's directory** (works from any cwd).
  - `library_*` ops' `dir` resolves relative to **`--out-dir`** (the vault landed in `out/vault/`, not cwd). Plan for this or pass structure through out-dir.
  - op `file` outputs (`implicit`, `export_stl`) land under `--out-dir`.
  - `asm` runner: part sources resolve against each `.lmcasm` file's own directory (`--base-dir` overrides).
- **THE ENGINE IGNORES UNKNOWN OP-LEVEL PARAMS SILENTLY** [VERIFIED: added
  `"bogus_param": 42` to an op — exit 0, no warning]. A typoed param name =
  default silently used. Inside `expr` trees, unknown *fields/shapes/ops* DO
  fail loudly with a JSON path (`at expr.b.a: …`). So: expression trees are
  typo-safe, op parameter lists are not — double-check op param spellings
  against API.md.

---

## 1. The `implicit` op — CSG expression trees

`implicit` meshes a recursive tree watertight at `voxel` resolution and
optionally writes a file (`.stl`/`.3mf` by extension). **It binds no solid** —
products are the file + measures (`triangles`, `watertight`, `healed`,
`volume`). Minimal runnable anchor [VERIFIED: 26,784 triangles, watertight,
volume 5547.53]:

```json
{"ops": [
  {"id": "blob", "op": "implicit", "voxel": 0.4,
   "expr": {"op": "smooth_union", "k": 3,
    "a": {"shape": "sphere", "center": [0, 0, 12], "radius": 8},
    "b": {"shape": "box", "min": [-10, -10, 0], "max": [10, 10, 10]}},
   "file": "blob.stl"}
]}
```

A tree node is either a **leaf** `{"shape": …}` or a **combinator** `{"op": …}`
with children `a`/`b` (or `in` for single-child ops). The complete surface
(quoted from the interpreter's own rejection messages, so it cannot drift):

- **12 leaves**: `sphere, box, cylinder, cone, capsule, torus, plane, gyroid,
  beam_lattice, pipe, helix_pipe, expr_sdf`
- **20 combinators**: `union, intersection, difference, smooth_union,
  smooth_intersection, smooth_difference, fillet_union, fillet_difference,
  chamfer_union, chamfer_difference, offset, shell, translate, rotate, scale,
  mirror, linear_pattern, circular_pattern, offset_by, lerp`
- **16 scalar ops**: `add, sub, mul, div, min, max, mod, atan2, neg, abs,
  sqrt, sin, cos, clamp, length2, length3` plus variables `"x"/"y"/"z"` and
  bare-number constants.

Parse errors carry the JSON path to the bad subtree (`at expr.b.a: …`).

### Leaf parameter cheat-sheet (all forms executed in the guide)

| leaf | params | notes |
|---|---|---|
| `sphere` | `center, radius` | |
| `box` | `min, max` | |
| `cylinder` | `a, b, radius` | endpoints, any axis |
| `cone` | `a, b, ra, rb` | capped frustum; `rb: 0` = sharp; `ra == rb` = cylinder |
| `capsule` | `a, b, radius` | sphere-swept segment |
| `torus` | `center, axis, major, minor` | tangent-contact unions **pinch** (non-manifold) — bury or gap, never kiss |
| `plane` | `point, normal` | UNBOUNDED half-space; inside = opposite normal; legal only under intersection with something bounded or an explicit op-level `domain` |
| `gyroid` | `min, max, scale, thickness` | TPMS; `scale` rad/mm, cell period 2π/scale; `thickness` = wall half-thickness; use `"mesher": "manifold"` |
| `beam_lattice` | cell form: `min, max, cell` ("cubic"/"octet"), `cell_size, radius` (≤ 16384 cells, fills whole cells from low corner) — or graph form: `nodes` + `struts` `[a, b, ra, rb]` (tapering) | junction-rich ⇒ manifold mesher |
| `pipe` | `path` (polyline) + `radius` (constant) XOR `radii` (one per vertex, tapers) | |
| `helix_pipe` | `center, axis, r_helix, pitch, turns, radius`, opt `samples_per_turn` (default 64, 8..1024) | springs, cooling channels |
| `expr_sdf` | see §2 | |

### Blend-family combinators (seams and cut mouths)

`fillet_union/fillet_difference {r}`: TRUE constant-radius blend (dimension
it on a drawing). `chamfer_union/chamfer_difference {r}`: 45° flat collar
(adds the most material). `smooth_union/smooth_difference/smooth_intersection
{k}`: polynomial blob, organic, dimension-vague. Hard `union/difference/
intersection` never bulge and never fail. Rule: fillet/chamfer when a drawing
would dimension the blend; smooth when you want flesh. **Read §4 (pillow
trap) before using any of them near parallel surfaces.**

### Transforms & patterns

`translate {offset}`, `rotate {axis, degrees, center?}` (**degrees** here),
`scale {factor}` (uniform, about origin), `mirror {point, normal}` (result =
child ∪ reflection). `linear_pattern {step, count}`, `circular_pattern
{center, axis, count, step_degrees?}` (default 360/count). Patterns nest
(3-stack × 8-ring = 24 copies). Count cap is loud: `field 'count' must be in
1..=4096`. Cost = domain repetition: each copy costs one child evaluation per
query.

### Mesher choice

- `"mesher": "narrowband"` (default): fast, surface-area-scaled, **requires
  ≤ 1-Lipschitz fields**.
- `"mesher": "manifold"`: dense sampling, only assumes continuity; use for
  TPMS/gyroids, octet lattices, junction-rich geometry, steep modulation
  fields, or any field you cannot bound.

### `gyroid_block` (separate op)

One-shot TPMS cube: `center`, `half`, `scale`, `thickness` — meshes via
Manifold DC straight to STL, heals once if needed (`healed: true` in measures
= marginal-resolution warning), `invalid_geometry` if still leaky. Binds
nothing. For graded walls or lattice ∩ part, use the `gyroid` leaf or the
Document features instead.

---

## 2. Scalar expression language + the `expr_sdf` contract

Used inside `expr_sdf.expr` and the `field` of `offset_by`/`lerp`.

- Constants are **bare JSON numbers** (`1.25`, never `{"const": 1.25}`).
- The query point is the **bare strings** `"x"`, `"y"`, `"z"` (mm).
  `{"var": "x"}` is refused (`missing required field 'op'`).
- Operator argument names (exact — wrong keys are refused loudly):
  - `add/sub/mul/div/min/max/mod {a, b}` — `mod` is Euclidean, result in `[0, b)`
  - `neg/abs/sqrt/sin/cos {arg}`
  - `clamp {value, lo, hi}` (never errors, even lo > hi)
  - `length2 {a, b}` — cylindrical radius is `{"op":"length2","a":"x","b":"y"}`
  - `atan2 {y, x}` — named keys, radians
  - `length3 {a, b, c}`
- SDF-land: `max` = intersection, `min` = union, negate = invert.

### The `expr_sdf` leaf contract (every clause is enforced)

```json
{"shape": "expr_sdf", "expr": <scalar tree>,
 "lipschitz_bound": L, "min": [x,y,z], "max": [x,y,z]}
```

- `lipschitz_bound` is **REQUIRED** [VERIFIED: omitting it →
  `invalid_param: at expr: missing required field 'lipschitz_bound'`].
  Declare a truthful `L ≥ sup|∇expr|`; the kernel evaluates `expr / L`
  (zero set unchanged, slope normalized).
- `min`/`max` come **together or not at all**; they feed the automatic
  meshing domain (tree bounds padded by 3·voxel). Omitting both is legal only
  if something else bounds the tree, else:
  `invalid_param: the expression tree is unbounded … intersect it with a
  bounded shape, give the expr_sdf leaf min/max bounds, or pass an explicit
  'domain'`.
- A 5×5×5 **degeneracy probe** runs pre-extraction; a pole (e.g. `1/z`) in the
  domain fails with `expression evaluates to inf at probe point …`. The probe
  is a tripwire, not a proof — clamp denominators anyway.

Minimal runnable expr_sdf [VERIFIED: watertight, volume 2146.35 ≈ analytic
2144.7 for r=8 sphere at voxel 0.5]:

```json
{"ops": [
  {"id": "s", "op": "implicit", "voxel": 0.5,
   "expr": {"shape": "expr_sdf", "lipschitz_bound": 1.0,
    "min": [-9,-9,-9], "max": [9,9,9],
    "expr": {"op": "sub", "a": {"op": "length3", "a": "x", "b": "y", "c": "z"}, "b": 8}}}
]}
```

### The Lipschitz honesty trap (measured spectrum in the guide)

Over-declaring L is safe (slower). **Under-declaring is NOT reliably caught**:
the guide measured a truth-1.0 sphere declared at 0.9/0.5/0.2/0.1 → silently
correct *that time*; at 0.05 → loud `invalid_geometry` (surface pruned away).
The dangerous middle ground is **partial pruning: silent holes**. Do the
gradient arithmetic every time (sum term slopes, declare with margin). If no
bound is practical → `"mesher": "manifold"`.

---

## 3. Field modulation — `offset`, `shell`, `lerp`, `offset_by`, graded lattices

| combinator | shape | semantics |
|---|---|---|
| `offset {t, in}` | uniform inflate (+) / deflate (−); rounds convex corners |
| `shell {t, in}` | hollow wall of **total thickness 2·t** straddling the surface |
| `lerp {a, b, field}` | pointwise distance blend, weight clamp(field, 0, 1) — round-to-square transitions, solid-to-lattice gradients |
| `offset_by {in, field, max_abs}` | surface moves outward by field(p) mm, clamped to ±max_abs |

**Modulation caveat**: the result is only `(1 + |∇field|)`-Lipschitz. Keep
graded fields to a few % per mm for the narrowband mesher, or use
`"mesher": "manifold"`.

Graded gyroid workhorse [VERIFIED: watertight, volume 6512.55 at voxel 0.5,
0.3 s] — wall thickness +0.6 mm at z=0 fading to nominal at z=30:

```json
{"ops": [
  {"id": "graded", "op": "implicit", "voxel": 0.5, "mesher": "manifold",
   "domain": {"min": [-15, -15, 0], "max": [15, 15, 30]},
   "expr": {"op": "intersection",
    "a": {"op": "offset_by", "max_abs": 0.6,
      "in": {"shape": "gyroid", "min": [-15, -15, 0], "max": [15, 15, 30],
             "scale": 0.45, "thickness": 0.7},
      "field": {"op": "mul", "a": 0.02, "b": {"op": "sub", "a": 30, "b": "z"}}},
    "b": {"shape": "cylinder", "a": [0, 0, 0], "b": [0, 0, 30], "radius": 14}}}
]}
```

(The uniform twin measured 3703.5 mm³ — the grade added 1.76× material biased
to the loaded end.) The persistable Document twin is `GyroidLattice` +
`LinearGrade` (§6.5). An op-level `"domain": {min, max}` is how you bound
trees containing unbounded leaves.

---

## 4. THE PILLOW TRAP (the #1 silent-failure mode of blends)

`fillet_union`/`smooth_union` are **global field operations**: everywhere the
two operands' surfaces sit within the blend radius of each other, material is
added — including buried faces and parallel walls far from the visible seam.
**Receipts stay green** (watertight, valid). The guide's measured minimal
repro: a riser whose buried bottom face sits 2.5 mm from the bed plane, fused
with `fillet_union r=5` → bed face pillowed **0.443 mm below z=0** while
reporting `ok: true, watertight: true` (volume +731 mm³ over the hard union).

**Mitigation — fuse first, cut last**: apply soft unions early, then re-clamp
every datum plane with a *hard* op as the final shaping step, e.g.

```json
{"op": "intersection",
 "a": {"op": "fillet_union", "r": 5, "a": …, "b": …},
 "b": {"shape": "plane", "point": [0, 0, 0], "normal": [0, 0, -1]}}
```

restored STL z_min to exactly 0.0000. **And probe datums**: a bbox/z_min check
on the export, a `volume_within` window against the hard-union volume —
receipts alone will smile through a pillow. Same rule applies to `HybridFuse`
parts carrying datum faces (§7).

---

## 5. Threads and helical features

Real helical threads self-intersect when buried in a shank — no exact boolean
can build them; only the implicit extraction fuses them watertight. Reusable
idiom (guide-executed: watertight, 125,328 tris, volume 728.30 vs plain-stud
804.25): cut a helical V-groove from a cylinder via `difference` with an
`expr_sdf`. The anatomy:

1. **Helical unwrap**: `u = mod(z − (P/2π)·atan2(y,x), P)` — continuous across
   the atan2 branch cut because the jump is exactly one pitch. (P = pitch;
   P/2π as a literal constant, e.g. 0.198943678865 for P=1.25.)
2. **Recenter branchlessly**: `abs(u − P/2)` = distance from groove centreline.
3. **Flank slope**: half-width grows `s` per mm of radius above root radius
   (`length2(x,y) − r_root`), included angle `2·atan(s)`; keep crest
   half-width < P/2 so lands remain.
4. **Span/root clamps** as `max` terms: `z ∈ [z0, z1]`, `r ≥ r_root`.
5. **Lipschitz accounting**: `|∇(z − kθ)| ≤ √(1 + (k/r)²)` over the band;
   add the flank slope; clamps are 1-Lipschitz; declare with margin
   (guide: sup ≤ 1.602, declared 1.7).
6. Thread **ridge** (bolt): build the trapezoid flanks in the (r, u) plane and
   `union` onto the shank — full M10×1.5 program in API.md ("The I6 proof",
   matched the Rust reference to <0.0001%).

Full runnable stud program: DESIGN_GUIDE.md §15. Voxel: ≥ ~3 voxels across
thread depth (0.12 for a 0.65-deep groove; 0.06–0.08 = resin-grade).
Catalog fastener bodies (§10) carry **no** threads — they are clearance
envelopes; model threads only when you're printing them.

### Voxel-size selection (measured anchors)

| intent | voxel (mm) |
|---|---|
| draft/iteration | 0.5–0.8 |
| FDM production (~100 mm part, 0.2 mm layers) | 0.3–0.4 |
| resin/SLA, fine threads | 0.06–0.12 |
| sub-voxel fits | don't — use B-rep (voxel meshes honest only to ~the voxel) |

Keep ≥ ~3 voxels across every wall/strut. `healed: true` on a passing op =
marginal resolution warning. Complexity rails (all loud): pattern count ≤
4096, beam_lattice ≤ 16384 cells, pipe/helix ≤ 100k segments,
`samples_per_turn` 8..1024, dense meshers cap at 2^28 cells. Model near the
origin in mm.

---

## 6. `.lmcpart` — the recipe grammar

### 6.1 Envelope + Document fields

```json
{
  "format": "lmc-part", "version": 1, "units": "mm",
  "name": "…", "created_with": "…",
  "document": {
    "params": {"name": 40.0},
    "features": [ {"<Variant>": {…}, "label": "…", "notes": "…"} ],
    "root": 1,
    "suppressed": [],
    "configs": {"variant_name": {"param": 60.0}},
    "active_config": "variant_name"
  }
}
```

- `format`/`version`/`units` are contract fields; non-mm is refused, never rescaled.
- Dimensions are `Dim`s: `{"Literal": 4.0}` or `{"Param": "h"}` — **no
  arithmetic in the file**; derive in your head or add a param.
- `root`: which feature is the result (omit = last). `suppressed`: modifier
  features (fillet/hole/transform/…) fall back to their input when suppressed.
  `configs`/`active_config`: named parameter-override sets (unknown name =
  no overrides). `label`/`notes` beside the variant are inert to geometry.
- Saves are byte-stable (sorted keys) — recipes git-diff like code.
- Optional `meta` block (BOM v2): `{"part_number", "material": {"name",
  "density_g_cm3"}, "make_or_buy"}` — feeds mass rollups in assemblies.

Complete working example [VERIFIED: exact_volume 7453.805 = 7680 − π·3²·8,
genus 1]:

```json
{
  "format": "lmc-part", "version": 1, "units": "mm",
  "name": "drilled_pad", "created_with": "agent",
  "document": {
    "params": {"pad_w": 40.0, "bore_d": 6.0},
    "features": [
      {"Box": {"center": [{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": 4.0}],
               "size": [{"Param": "pad_w"}, {"Literal": 24.0}, {"Literal": 8.0}]},
       "label": "pad"},
      {"Hole": {"input": 0, "kind": "Drill", "m_or_d": {"Param": "bore_d"},
                "at": [{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": 8.0}],
                "axis": [{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": -1.0}]},
       "label": "axle bore"}
    ],
    "root": 1, "suppressed": [],
    "configs": {"wide": {"pad_w": 60.0}}
  }
}
```

Load and gate:

```json
{"ops": [
  {"id": "pad", "op": "load_part", "file": "pad.lmcpart"},
  {"id": "xv", "op": "exact_volume", "in": "pad"},
  {"id": "topo", "op": "validate", "in": "pad"}
]}
```

Guide-measured edit semantics: `"active_config": "wide"` → 11293.805;
`"suppressed": [1]` → 7680.0 genus 0 (hole fell back to its input);
`"root": 0` → un-drilled feature 0. The whole edit model is file edits +
deterministic rebuild.

### 6.2 Document grammar ≠ op grammar — the three corners that bite

1. Document sketches wrap indices in objects (`"segments": [{"a": 0, "b": 1}]`,
   `"arcs": [{"a","b","center","ccw"}]`, `"circles": [{"center","radius_point"}]`)
   and constraints are externally-tagged PascalCase
   (`{"Fixed": {"point": 0, "at": [0, 0]}}`, `{"Distance": {"a": 0, "b": 1,
   "distance": 50.0}}`) — unlike the op surface's bare pairs.
2. The op surface is **degrees** everywhere; the Document's
   `ExtrudeSketch.draft` and `CircularPattern.angle` are **RADIANS**.
3. `Transform.xform` is 12 floats, **COLUMN-major** (x-axis, y-axis, z-axis
   columns, then translation): `[1,0,0, 0,1,0, 0,0,1, 20,0,0]` = translate
   +20 X. `.lmcasm` quaternions are `[x, y, z, w]`.

Feature variant names and enums are PascalCase (`"Box"`, `"Hole"`,
`"kind": "Drill"`, `"Boolean": {"op": "Difference"}`), unlike the snake_case
op surface.

### 6.3 All 30 feature variants (half: B = exact B-rep, V = voxel-only, B+V = both)

| variant | half | notes |
|---|---|---|
| `Box`, `Sphere`, `Cylinder` | B+V | Cylinder axis +Z, `center` = body centroid (spans center.z ± h/2) |
| `FilletedCylinder`, `ChamferedCylinder` | B | `radius/height/fillet|chamfer`, base at local origin |
| `Boolean` | B+V | `{"op": "Union"/"Difference"/"Intersection", "a": id, "b": id}` |
| `SmoothUnion/SmoothDifference/SmoothIntersection` | V | `blend` radius |
| `Gyroid` | V | legacy center+size form; prefer `GyroidLattice` |
| `Transform` | B+V | 12-float column-major, rigid + uniform scale |
| `Fillet`, `Chamfer` | B | persistent EdgeName = pair of face names, optional `near` Dim-triple disambiguator; survives re-dimensioning |
| `ExtrudeSketch` | B | Document sketch + `height`, `dims` `[[constraint_idx, Dim]]` overrides (re-target a constraint from params BEFORE solving), radian `draft` (outer boundary only) |
| `LinearPattern`, `Mirror`, `CircularPattern` | B+V | pattern fused; radian `angle` |
| `Shell` | V | wall `thickness`, outer faces preserved |
| `Hole` | B | wizard: `kind` Drill/Clearance/Counterbore/Countersink/Tap; `fit` only clearance-family, `depth` only drill/tap — wrong combos fail loudly |
| `CircularRimFillet` | B | exact torus rim; `concave: true` = bore-exit lip (cap structure must be what a boolean bore cut emits — a Hole-wizard drill through a plain cylinder cap does NOT qualify yet, fails loudly) |
| `LoftSolid`, `SweepSolid` | B | section stack / profile-along-path |
| `CatalogPart` | B | any §10 standard part as a feature |
| `ORingGroove`, `CirclipGroove`, `HeatsetBoss` | B | catalog cut twins |
| `GyroidLattice` | V | corner-form `region` [[min],[max]], `scale`, `thickness`, optional `grade` = LinearGrade `{axis, per_unit, offset, max_abs}` (field = offset + per_unit·(axis·p), clamped; keep slope a few %/mm) |
| `BeamLatticeFill` | V | cubic/octet cell fill of a region |
| `PipeFeat` | V | polyline `path` + per-vertex `radii` (arrays of Dim) |
| `HybridFuse` | B+V | §7 |

Neither half fakes the other: `Document::evaluate` (voxel) passes fillets
through unrounded and skips sketches; `evaluate_brep` omits voxel-only
features.

### 6.4 Voxel-half recipes cannot enter the solid environment [VERIFIED]

A Shell/Gyroid/smooth-boolean/pipe-rooted recipe has no exact B-rep;
`load_part` refuses with exactly:

> `invalid_param: op '…': the part's feature tree produced no exact B-rep
> (voxel-half-only features — shell, gyroid, smooth booleans — cannot enter
> the solid environment)`

Such recipes live in **assemblies** (the `.lmcasm` runner meshes them via
`Document::mesh` and names the route) or behind a `HybridFuse` that re-enters
the exact world.

### 6.5 Graded-lattice recipe pattern

`GyroidLattice` with `grade` + `Cylinder` + `Boolean Intersection` (full
`damper.lmcpart` in DESIGN_GUIDE §16.8). Measured behavior when run via a
one-instance `.lmcasm`: at runner `--voxel 0.4` it exported
`route: "voxel_healed"`, **`watertight: false` — and the asm run still exits
0**. Re-run at `--voxel 0.25` → watertight true. **Gate on the receipt, not
the exit code**, for assembly instance exports.

---

## 7. Hybrid parts — `HybridFuse`, routes, the voxel knob

```json
{"HybridFuse": {"brep": <feature id>, "field": <feature id>,
                "op": "Union"|"Difference"|"Intersection", "voxel": {"Literal": 0.5}}}
```

The field side is meshed at `voxel` for the seam; `voxel ≤ 0` auto-picks
≈ 1/96 of the relevant bounding diagonal. Route semantics:

- **`ExactStitch`** (`route: "exact"`): untouched exact faces kept verbatim
  (bit-identical, provenance-tagged); result is a real Solid that feeds
  downstream B-rep features and `load_part`.
- **`Healed`** (`route: "voxel_healed"`): everything voxel-resampled; the
  exact half returns nothing → `load_part` refuses (§6.4 message), but the
  `.lmcasm` route still delivers the watertight mesh with the route named.
- **50k-triangle operand rail** (`HYBRID_EXACT_MAX_OPERAND_TRIS`): an operand
  mesh denser than 50,000 triangles self-demotes to the heal ("operand mesh
  too dense for the exact arrangement … re-mesh coarser or accept the heal").
  The fuse `voxel` controls operand density — **coarsen to re-enter the exact
  route**.

Working exact-stitch example [VERIFIED: genus 1, `route: "exact"`, 5,492
triangles, watertight — an implicit pipe became part of a loadable, cuttable,
STEP-able exact solid]: Box + PipeFeat handle + HybridFuse Union at voxel 0.5
(full file in §17.1 of the guide; also re-executed for this digest).

**Remedy chain** when a fuse misbehaves:
1. Wanted exact, got healed → coarsen fuse `voxel` (50k rail), simplify the
   seam region, or keep the lattice as its own `.lmcasm` instance.
2. Healed and leaky → refine the *runner's* `--voxel`; junction-rich fields
   get the manifold mesher automatically on the Document mesh path.
3. Never infer the route from looks — read `route` in the receipts.

Datum rule for fused parts: same as §4 — fuse first, cut last, probe datums.

---

## 8. `.lmcasm` — assemblies, complete grammar

Note: since 2026-07-17 you can build assemblies with **in-program ops**
(`asm_instance`, `asm_instance_mesh`, `asm_mate`, `asm_mate_axis`,
`asm_mate_face`, `asm_solve` [DOF-honest, fails on non-convergence],
`asm_contacts`, `asm_export`, `asm_export_step`, `asm_save` which writes the
`.lmcasm` for you, `gear_train_poses`). In-program mate kinds add `angle`,
`axis_distance` (gear center-distance), `fixed`. See API.md "Assembly ops
(in-program)". The file format below remains ground truth and everything
loads and runs.

### 8.1 File anatomy

```json
{
  "format": "lmc-asm", "version": 1, "units": "mm", "name": "…",
  "instances": [
    {"name": "base", "source": {"path": "spacer.lmcpart"},
     "pose": {"translation": [0.0, 0.0, 0.0]}},
    {"name": "pin", "source": {"part": { …full inline .lmcpart envelope… }},
     "pose": {"translation": [14.2, 9.5, 0.4],
              "rotation": [0.0, 0.0, 0.0, 1.0]}},
    {"name": "sub", "source": {"asm_path": "asm/shaft_input.lmcasm"}, "pose": …},
    {"name": "scan", "source": {"mesh": "part.stl"}, "pose": …},
    {"name": "spare", "suppressed": true, "source": …, "pose": …}
  ],
  "mates": [ … ],
  "states": {"service": {"poses": [ …one per instance… ], "suppressed": [2]}}
}
```

- **Sources** (4 kinds): `path` (relative `.lmcpart`), `part` (inline
  envelope), `asm_path` (nested sub-assembly, path only — no inline asm),
  `mesh` (`.stl`/`.obj`/`.3mf`/`.ply`, welded, measured honestly as a mesh).
- **Poses**: `{"translation": [x,y,z], "rotation": [qx,qy,qz,qw]}` —
  quaternion `[x,y,z,w]`, omitted = identity; rigid only (scale refused on
  save, `BadPose`). Rx(−90°) = `[-0.7071068, 0, 0, 0.7071068]`.
- **Mates** (file-format set, externally-tagged PascalCase over instance
  indices, geometry in each instance's **LOCAL** frame, dirs normalized
  defensively):
  - `Coincident {a, a_point, b, b_point}` — point-on-point
  - `Distance {a, a_point, b, b_point, distance}` — separation ≥ 0
  - `Parallel {a, a_dir, b, b_dir}` — parallel OR anti-parallel (solver picks the closer, never forces a 180° flip)
  - `Concentric {a, a_axis_point, a_axis_dir, b, b_axis_point, b_axis_dir}` — axes collinear
  - **Stored poses are only the solver's seed; the mates are the authority** — re-solved on every load.
- **States**: pose snapshots, one pose per instance (count-checked on save),
  plus optional `suppressed` index list. Instance-level `"suppressed": true`
  keeps it in the file but out of geometry, BOM and contacts.

### 8.2 Runner report anatomy [VERIFIED end-to-end]

`kernel-api asm demo.lmcasm --out-dir out/` produces ordered op entries:

| entry | receipt highlights |
|---|---|
| `load` | `instances, mates, states, suppressed, top_level, units` |
| `mates` | `residual` (runner FAILS above 1e-6 — `assert_failed`), `per_mate` residuals, and a **`dof` block**: `constraint_rows, rank, redundant_rows, free_dof, grounded_instances, verdict` e.g. `"under_constrained (1 free DOF)"` [VERIFIED] |
| `bom` | schema `bom/2`: `flat` (grouped by name+params), `tree` (structure with rollups), plus `bom.json` and fixed-column `bom.csv` files; byte-identical across runs |
| `export:NN:name` | one world-posed STL per instance under `parts/`, with honest per-instance `route` (`"exact"` / `"voxel_healed"`) and `watertight`; suppressed instances report `{"suppressed": true}` and write nothing |
| `export:assembly` | merged mesh STL: `instances, triangles, watertight` |
| `export:assembly_step` | AP214 STEP of solid-backed instances (mesh instances listed as `skipped`) [VERIFIED — the runner writes STEP too] |
| `contacts` | see below |
| `state:<name>` | state applied + exported, then assembled poses restored |

### 8.3 Contacts = clearance/interference checks [VERIFIED]

```json
{"pairs": [{"a": "base", "b": "pin", "distance": 0.4976, "i": 0, "j": 1,
            "touching": false}],
 "tol": 0.05, "touching": 0, "window": 1.0}
```

- All-pairs proximity scan within `--window` (default 1.0 mm); `touching` =
  distance ≤ ~1e-6. B-rep parts measured on their **raw exact tessellation**
  at `--tol` (0.05 default), so a designed 0.5 mm radial fit reads 0.4976 —
  sub-voxel fits are real measurements.
- `touching: true` is a *class*: the scan cannot distinguish a designed seat
  from an interference overlap. Distinguish via the exact route (`union` +
  `shells` count) or an intent layer.
- **Contacts ≠ connections**: touching parts can still fall apart. Doctrine:
  (1) recess/spigot registration so parts locate each other, (2) real mating
  faces for lattices (union solid end discs — a raw TPMS edge is not a
  joint), (3) a fastener path ("what holds this together?"). Then assert
  intent in a caller-policy check: allowlist designed contacts, fail on
  unexpected touches, prove `MUST_CLEAR` pairs positive-distance (model:
  `gearbox/check_asm.py`, 52/52 designed contacts).
- Gate policy [MEASURED]: a non-watertight *instance export* does NOT fail
  the asm run (exit stays 0) — your pipeline must gate on the receipts.
  The `mates` residual gate (1e-6) DOES fail the run.

### 8.4 Nested assemblies + BOM v2

- `{"source": {"asm_path": "sub.lmcasm"}}` to any depth; each file resolves
  sources against its own directory. A sub-assembly solves its own mates,
  then enters the parent as **one rigid unit**. Parent mates/states address
  top-level instances only (no mating to a sub's internal member). Include
  cycles refused loudly, naming the chain. Leaf parts get hierarchical names
  (`stack_in/spacer9_0`, `parts/02_stack_in_shaft_in.stl`).
- BOM groups by `(part name, param values)` — same recipe at h=8 and h=10 =
  two lines; suppressed instances excluded. With part `meta`, flat lines gain
  `part_number, material, unit_mass_g, line_mass_g, make_or_buy` and an honest
  `volume_source`: `"exact"` (analytic B-rep) vs `"mesh"` (voxel route).
  No revisions/suppliers/cost — that is PLM-land, out of scope.

---

## 9. The library — admission-gated personal vocabulary

A library is a plain directory: one `{name}-v{version}.lmcpart` per admitted
entry + byte-stable `index.json` (git it; every admission/removal is
auditable). Five ops: `library_add`, `library_search`, `library_instantiate`,
`library_deprecate`, `library_remove`.

**Pitfall [VERIFIED]**: the ops' `dir` resolves relative to `--out-dir`, not
cwd or the program directory.

### Admission gate (`library_add`)

Payload: `part` (full envelope) + `meta` `{name, version, category?, tags?,
description?, provenance {author, date}, params: [{name, units, default, min,
max}]}`. Dates are **caller-supplied** — the kernel never stamps clock time
(deterministic bytes).

The gate builds the candidate at declared **defaults**, sampled **range
corners** (all 2^n min/max combos, capped at 16 by a deterministic spread
keeping all-min, all-max, each single-param extreme) and the **midpoint**;
each sample must be a closed manifold AND rebuild volume-bit-deterministically
(two evaluations, identical to the last bit). [VERIFIED: 3-param bushing →
`gate_samples: 10, gate_rebuilds: 2, volume_at_defaults: 3995.4498`;
`library_instantiate` with `params: {"h": 20}` → exact_volume 8042.477.]

Honest scope: the gate proves the sampled points, not everything between —
declare ranges you intend, not the widest imaginable. A degenerate corner
(e.g. bore_r.max ≥ outer_r.min) is refused loudly, exit 1, kind
`admission_rejected`, naming the corner:
`sample corner_lh (bore_r=12, outer_r=8) failed to build: the rebuild
produced an EMPTY solid`. Fix the declaration, not the gate.

### Curation rules

- `(name, version)` is **immutable** once admitted; changed geometry = new
  version; unversioned instantiate takes the highest.
- `library_instantiate` rejects unknown/out-of-range params loudly.
- Retirement is two-stage: `library_deprecate` (hidden from search, still
  builds, instantiate carries `"deprecated": true` + warning; idempotent) →
  `library_remove` (refuses with kind `dependents_exist` while any `.lmcasm`
  in the directory references the stored file by path; `"force": true`
  overrides — git is your undo).

---

## 10. Standard-parts catalog

48 parametric parts + 13 standard feature cuts + 7 hole-wizard cuts + 7
design-math lookups, from published ISO/DIN/ANSI/vendor tables. Conventions:

- Parts build **at the origin along +Z** — place with `pose`. Bores and
  shanks are **diameters**. `bore_d` is accepted as an alias for `bore` on
  `spur_gear`/`gt2_pulley`/`chain_sprocket`.
- **No threads on catalog bodies** — they are exact assembly/clearance
  envelopes (clearance studies are exactly what they're for).
- Out-of-table sizes are loud [VERIFIED]:
  `hex_bolt: M7 is not in the ISO 4017 table (supported: M3, M4, M5, M6, M8,
  M10, M12, M16)`.
- Invocation is just an op [VERIFIED — nuts/inserts/dowels all present]:

```json
{"ops": [
  {"id": "nut", "op": "hex_nut", "m": 5},
  {"id": "ins", "op": "heatset_spec", "m": 4},
  {"id": "pin", "op": "dowel_pin", "d": 4, "length": 20}
]}
```

(`heatset_spec` M4 → `pilot_d 5.6, pocket_depth 9.1, boss_d 11.2,
insert_length 8.1`; dowel Ø4×20 exact_volume 249.249.)

### The 48 parts by family (op names exact)

- **Fasteners (15)**: `hex_bolt` (m, length) · `hex_nut` (m) · `washer` (m) ·
  `socket_head_cap_screw` (m, length) · `flat_head_screw` · `button_head_screw` ·
  `set_screw` · `lock_nut` (m) · `threaded_rod` (m, length) · `standoff` ·
  `shoulder_bolt` (shoulder_d, shoulder_len) · `spring_washer` (m) ·
  `dowel_pin` (d, length) · `circlip_external` (shaft_d) · `circlip_internal` (bore_d)
- **Power transmission (13)**: `spur_gear` (module, teeth, face_width, bore,
  keyway?) · `internal_gear` (module, teeth, face_width, rim_od) · `gear_rack`
  (module, length, width) · `gt2_pulley` (teeth, belt_width, bore, flanged?) ·
  `chain_sprocket` (pitch, roller_d, teeth, bore) · `shaft` (d, length,
  keyway?) · `parallel_key` (b, h, l) · `jaw_coupling_hub` (od, bore) ·
  `jaw_coupling_spider` (od) · `set_screw_coupling` (bore1, bore2) ·
  `clamp_coupling` (bore1, bore2) · `lead_screw_tr8` (length, lead) ·
  `lead_screw_nut_tr8` ()
- **Bearings & linear (10)**: `deep_groove_bearing` (designation:
  603/608/625/688/6000/6001/6804) · `flanged_bearing` (F608/F623) ·
  `thrust_bearing` (51100/51101) · `kp08_pillow_block` () ·
  `linear_bearing_lmuu` (bore: 8/12) · `sc8uu_block` () · `shaft_support_sk8` () ·
  `shaft_support_shf8` () · `mgn12_rail` (length) · `mgn12_carriage` ()
- **Motors (2)**: `nema_motor` (frame: 17/23, body_len) · `nema_mount_plate`
  (frame, thickness, margin)
- **Sealing & fluid (4)**: `o_ring` (dash) · `o_ring_cord` (ring_id, cord_d) ·
  `pipe_boss_g` (designation G1/8…G1/2, wall, length) · `hose_barb` (hose_id, barbs)
- **Springs & structure (4)**: `compression_spring` (wire_d, outer_d, pitch,
  turns — refused if coils touch: pitch ≤ wire_d) · `extrusion_2020` (length) ·
  `extrusion_3030` (length) · `tnut_2020` ()

### The 13 standard feature cuts (machined into a prior solid via `in`/`at`/`axis`)

`heatset_insert_boss`, `circlip_groove_external`, `circlip_groove_internal`,
`o_ring_groove`, `o_ring_face_gland`, `o_ring_face_gland_racetrack`,
`nema_mount_cut`, `servo_pocket`, `tr8_nut_trap`, `pc4_port`,
`teardrop_hole` (45°-crowned, prints unsupported; needs `up`), `board_mount`
(rpi/arduino patterns — mirror in y when cut from a top face, documented
face-frame caveat), `bridged_counterbore` (0.3 mm membrane, deliberately NOT
through — drill after printing; genus 0 vs the wizard's genus 1). Parameter
tables + runnable example each: API.md "Standard feature cuts".

### The 7 design-math lookups (bind no geometry — numbers come back as measures)

`iso286_fit` (d, fit ∈ H7/g6, H7/h6, H7/k6, H7/n6, H7/p6, H7/s6, H8/f7;
negative clearance = interference) · `gt2_belt` (center_distance, t1, t2) ·
`gt2_center_distance` (belt_teeth, t1, t2) · `heatset_spec` (m) ·
`metric_cord_gland` (cord_d) · `racetrack_cord_length` (x_len, y_len,
corner_r) · `pipe_thread_g` (designation). Referencing one as a solid is
`missing_ref`.

Catalog assembly facts worth stealing: bearing seats are **nominal
line-to-line** by design (union of dropped-in bearing + wall merges to ONE
shell) — apply the `iso286_fit` allowance to *your bore*, not the catalog
body. Gear meshes: assert *designed* backlash (e.g. flank gap 0.083 mm at
C = conjugate − 0.4), never the theoretical center distance (involutes touch
within µm and the union welds). Topology contracts double as fingerprints
(rigid coupling genus 5, clamp coupling genus 4, 75 mm MGN12 rail genus 3).

---

## 11. Refusal-kind vocabulary seen in this half

| kind | meaning / example |
|---|---|
| `invalid_param` | bad/missing field, out-of-table size, voxel-half `load_part`, unbounded tree, missing lipschitz_bound |
| `invalid_geometry` | didn't mesh watertight (tangent pinch `non_manifold_edges`, over-pruned Lipschitz, leaky heal on program-level `export_stl`) |
| `admission_rejected` | library gate corner failed |
| `dependents_exist` | `library_remove` while an `.lmcasm` references the entry |
| `missing_ref` | referencing a non-solid op (e.g. a design-math lookup) as input |
| `assert_failed` | asm mates residual > 1e-6; any `assert` op miss |

---

## 12. Deep-dive pointers

| topic | where |
|---|---|
| implicit tree node params, per-leaf tables | API.md "Implicit / hybrid"; DESIGN_GUIDE §11 |
| expr_sdf contract clauses, Lipschitz trap | DESIGN_GUIDE §12 |
| offset_by / lerp / graded gyroid | DESIGN_GUIDE §13; Document twin §16.8 |
| pillow trap repro + fix | DESIGN_GUIDE §14; field case `iphone_stand/DESIGN.md` §4, §8 |
| thread idiom, M10 reference program | DESIGN_GUIDE §15; API.md "The I6 proof" |
| .lmcpart envelope, feature table, Document grammar corners | DESIGN_GUIDE §16 (all 30 variants §16.4) |
| HybridFuse routes, 50k rail, remedy chain | DESIGN_GUIDE §17; voxel table §17.4; rails §17.5 + docs/NUMERICS.md |
| .lmcasm grammar, runner, contacts, nesting, BOM v2 | DESIGN_GUIDE §18; in-program ops API.md "Assembly ops (in-program)" |
| joinery doctrine / intent checks | DESIGN_GUIDE §18.6; `gearbox/check_asm.py`; `legacy/kernel-model-examples/tri_benchmark.rs` (uncompiled) |
| library ops + gate | DESIGN_GUIDE §19; API.md "Parts library" |
| catalog parts / cuts / design math params | DESIGN_GUIDE §20; API.md "Standard parts catalog", "Standard feature cuts" |
| exports, STEP, print-readiness, failure playbook, limits | DESIGN_GUIDE §21–§24 |
| B-rep half (ops 1–10: sketches, booleans, fillets, measures) | DESIGN_GUIDE §4–§10 (separate digest) |
