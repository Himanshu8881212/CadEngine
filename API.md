# LMCAD JSON Binding — the AI Cookbook (`kernel-api`)

This is the op-by-op reference for driving the LMCAD hybrid kernel **without
writing Rust**: you send a JSON *program*, the kernel executes it, and you get a
JSON *report* back. It is written for an LLM (or any external process) that
plans CAD work as data. Every example below is a complete runnable program.

```bash
# Build once, then run programs:
cargo run -p kernel-api --release -- run program.json --out-dir out/
# or after `cargo build -p kernel-api --release`:
target/release/kernel-api run program.json --out-dir out/
# the .lmcasm assembly surface (load/mates/BOM/exports/contacts — see below):
target/release/kernel-api asm assembly.lmcasm --out-dir out/asm
```

The report prints to **stdout**; the exit code is **0 iff every op succeeded**.
Rust embedders call `kernel_api::run_program(json_text, out_dir) -> Report` and
`kernel_api::run_assembly(path, out_dir, &AsmOptions) -> Report`.

## ACE / voxel-physics bridge

Two ops connect LMCAD to voxel physics pipelines (topology optimization,
hex8 FEA) that speak the `solid_fraction.npy` contract — float32, C-order,
shape `(nx, ny, nz)` indexed `rho[i,j,k]` with `i↔x`, voxel CENTERS at
`origin + (index + 0.5) · voxel` (mm):

```json
{"id": "grid", "op": "sample_density_grid", "in": "part",
 "origin": [-1, -1, -1], "voxel": 0.5, "shape": [84, 44, 64],
 "supersample": 2, "file": "solid_fraction.npy"}
```
`in` (a bound solid, sampled through the winding-number MeshSdf bridge) or
`expr` (an implicit tree) — exactly one. `supersample`³ stratified
sub-points per voxel give true boundary fractions (SIMP-friendly), 1..=4.
Measures: `voxels`, `shape`, `solid_fraction_mean`, `bytes`.

```json
{"id": "stl", "op": "mesh_density_grid", "npy": "final_rho.npy",
 "origin": [-1, -1, -1], "voxel": 0.5, "iso": 0.5, "file": "part.stl"}
```
Reads a density `.npy` (`<f4`/`<f8`, C-order), thresholds at `iso`,
REDISTANCES to a true level-set and meshes it watertight through the
narrow-band pipeline (heal fallback; the op FAILS rather than emit a leaky
mesh). Measures mirror the ACE `emit_stl` contract: `ok`, `volume_mm3`,
`num_triangles`, `watertight` (+ `healed`).

Round-trip property (pinned in `tests/bridge.rs`): B-rep → grid at h →
mesh recovers the exact volume within the one-voxel skin (<5% at h=0.5 on
an L-bracket).

## The program model

```json
{"ops": [
  {"id": "plate", "op": "box", "min": [0, 0, 0], "max": [60, 40, 8]},
  {"id": "vol",   "op": "volume", "in": "plate"},
  {"id": "out",   "op": "export_stl", "in": "plate", "file": "plate.stl"}
]}
```

- Ops run **in order**. Each op has a unique `id` and an `op` kind.
- Geometry-producing ops **bind** their result to their `id`; later ops
  reference it via `in`, `a`, `b`, or `sketch`. There are three value kinds:
  **solid** (an exact B-rep), **sketch** (a solved 2D sketch), and **mesh**
  (triangles — see *Mesh values* below). Pure measure ops bind nothing —
  referencing their id is a `missing_ref` error.
- Execution **stops at the first failing op**: the report covers every attempted
  op and names exactly one root-cause failure (no error cascades).
- **No silent invalidity**: every solid-producing op is gated through the
  kernel's `validate()`. A result that is not a closed manifold (or is empty)
  fails the op with the topology details instead of being bound.
- Every op that reports measures accepts the universal **`require`** gate — see
  *Gating a program with `require`* below.

### Conventions
- Units are **millimetres**; angles in the JSON surface are **degrees**,
  always (`degrees`, `*_deg`). (The `.lmcpart` Document grammar has ONE radian
  field — `ExtrudeSketch.draft`; see the Native formats section.)
- 3D vectors are `[x, y, z]`; 2D profile points are `[x, y]` arrays.
- Profiles for `extrude` must be **counter-clockwise** simple polygons (a CW
  profile builds inside-out and fails the validity gate loudly).
  `extrude_with_holes`, `extrude_tapered`, `revolve`, and the sketch sweeps
  re-wind input automatically.
- Unknown JSON fields are ignored, but the op reports a `warnings` entry naming
  the key it did not accept. A missing/malformed **required** param is a loud
  `invalid_param`; misspelling an **optional** param leaves its default in
  effect — read the warnings, and `describe {"name": "<op>"}` for the accepted
  set.

## Gating a program with `require`

`require` is accepted on **every op** and is checked against **that op's own
measures**. An unmet expectation fails the op with `assert_failed`, so a
mandatory gate lives in the program instead of in an external grep over the
report:

```json
{"id": "stl", "op": "export_stl", "in": "part", "file": "part.stl",
 "require": {"route": "exact", "watertight": true}}
```

Why here and not on `assert`: `assert` takes a bound solid, so it can only gate
what a solid answers with no further parameters. The other mandatory gates are
measured by ops that own their own parameters — the build direction and overhang
angle (`support_report`), the thin-wall threshold (`wall_thickness`), the build
envelope (`bounding_box`), the export path and route (`export_*`), the weld
scale (`mesh_components`). Hanging the expectation off the op that already owns
those parameters keeps **one** way to express a gate and needs no vocabulary of
its own. `assert` stays the topology gate.

**Keys** name a measure of that op, and may be dotted paths into a nested
measure (`bbox.size`, `stress.max`); an integer segment indexes an array
(`size.2`).

**Expectations**:

| form | meaning |
|---|---|
| a scalar (`true`, `1`, `"exact"`, `null`) | must be EQUAL |
| an array | element-wise against an array measure; `null` skips an element |
| `{"min": x}` / `{"max": x}` | INCLUSIVE bound |
| `{"equals": v}` | equality (the object form, for combining with a bound) |
| `{"within": {"target": t, "abs": a}}` | \|measured − t\| ≤ a |
| `{"within": {"target": t, "percent": p}}` | half-width is `\|t\|·p/100` |
| `{"not_null": true}` | the measure must have been computed |

Several clauses may be combined in one object; **all** must hold.

On success the op's measures gain a `required` echo of the expectation, so the
receipt records the gate that was applied and not merely that the op passed.

**Refusals** (`invalid_param`, never a silent pass):

- `require` present but empty — a gate that checks nothing must not report success.
- a key that names no measure — the refusal lists the keys that exist, so a typo
  can never become a gate that quietly checks nothing.
- `min`/`max`/`within` against a non-numeric or `null` measure.
- `require` on an op that reports no measures.

The four `DELIVERABLE_SPEC` §2 gates that had no in-program expression before:

```json
{"id": "stl", "op": "export_stl", "in": "p", "file": "p.stl",
 "require": {"route": "exact", "watertight": true}},
{"id": "sup", "op": "support_report", "in": "p", "build_dir": [0, 0, 1],
 "require": {"steep_area": {"max": 0.0}}},
{"id": "wall", "op": "wall_thickness", "in": "p", "flag_below": 1.2,
 "require": {"thin_area": {"max": 0.0}, "p05_thickness": {"min": 1.2}}},
{"id": "bed", "op": "bounding_box", "in": "p", "envelope": [256, 256, 256],
 "require": {"fits_within": true}}
```

## Mesh values

The voxel / implicit route produces **meshes**, not B-reps, and so do the
exports. Those meshes are the files that get printed, so they bind to their op's
`id` and are measurable:

| op | binds |
|---|---|
| `export_stl` / `export_3mf` | the mesh actually written (on the healed route this is **not** the solid's tessellation) |
| `import_mesh` | the mesh read from the file |
| `implicit` / `tpms` / `gyroid_block` / `shell` / `mesh_carve` / `hybrid_boolean` / `export_threaded` | the extracted mesh |

Ops that accept a bound mesh wherever they accept a solid: `validate`,
`volume`, `bounding_box`, `mesh_components`, `support_report`, `clearance`,
`assert_disjoint`, and `assert` (its mesh-meaningful checks). Their measures
carry `"source": "solid"` or `"source": "mesh"` so the two are never confused.

This does **not** add a mesh→B-rep conversion. A mesh value stays a mesh; the
only field→exact route remains the explicit `solid_from_implicit` reverse
bridge, which says `route: "voxel"` on its own receipt. Handing a mesh to an op
that needs exact geometry is a loud `wrong_type`.

A complete gate on a print file:

```json
{"id": "stl", "op": "export_stl", "in": "part", "file": "part.stl",
 "require": {"watertight": true}},
{"id": "one", "op": "mesh_components", "in": "stl", "require": {"components": 1}},
{"id": "bed", "op": "bounding_box", "in": "stl", "envelope": [256, 256, 256],
 "require": {"fits_within": true}},
{"id": "sup", "op": "support_report", "in": "stl",
 "require": {"steep_area": {"max": 0.0}}}
```

## The report

```json
{
  "ok": true,
  "ops": [
    {"id": "plate", "ok": true},
    {"id": "vol",   "ok": true, "measures": {"volume": 19200.0}},
    {"id": "out",   "ok": true, "file": "out/plate.stl",
     "measures": {"route": "exact", "triangles": 12, "watertight": true}}
  ]
}
```

Per-op entry fields:

| field | present | meaning |
|---|---|---|
| `id` | always | the op's id (`$program` for whole-file failures, `#<index>` if an op had no id) |
| `ok` | always | whether this op succeeded |
| `measures` | per op | op-specific numbers/flags (documented per op below) |
| `warnings` | when non-empty | hazards the interpreter noticed while running the op. Unknown params **fail closed** (2026-08-10): a param the op does not accept fails the op with `invalid_param` — a misspelled manufacturing dimension must never silently select a default and still return an apparently valid part — and the report entry carries the offending keys in `warnings` beside the error. Keys starting with `_` are the in-op comment convention and never warn. The universal `require` key is accepted on every op |
| `file` | export ops | the path actually written |
| `error` | on failure | `{"kind": <error kind>, "message": "..."}` — the message names the op id and the offending parameter/values |

A failing program looks like:

```json
{
  "ok": false,
  "ops": [
    {"id": "u", "ok": false,
     "error": {"kind": "missing_ref",
               "message": "op 'u' param 'a': no result named 'nope' — it must be the id of an earlier geometry-producing op"}}
  ]
}
```

### Error kinds

| kind | when |
|---|---|
| `parse` | the program file is not valid JSON, or not `{"ops": [...]}` shaped (reported on id `$program`) |
| `unknown_op` | `op` names none of the 161 operations below |
| `duplicate_id` | two ops share an `id` |
| `missing_ref` | `in`/`a`/`b`/`sketch` names no prior result (or names a measure/export op, which binds nothing) |
| `wrong_type` | the reference resolved to the wrong kind (e.g. a sketch where a solid is needed) |
| `invalid_param` | required param missing/malformed, the kernel rejected degenerate inputs (incl. an EMPTY boolean result, e.g. a disjoint intersection), a standard size outside its ISO/DIN table, or a file that is not a loadable `.lmcpart` |
| `feature_failed` | fillet/chamfer/rim-fillet could not apply: witness matched nothing, radius does not fit, edge outside the supported scope |
| `sketch_failed` | constraints did not converge (conflicting), or the profile is open/degenerate |
| `invalid_geometry` | the op ran but produced a non-manifold/non-closed solid (full `Validity` details in the message), or a mesh that stayed leaky after healing |
| `admission_rejected` | `library_add`'s admission gate refused the candidate: a sample of its declared parameter ranges failed to build, was not a closed manifold, or did not rebuild volume-bit-deterministically — the message names the exact sample and values; nothing was admitted |
| `dependents_exist` | `library_remove` refused: `.lmcasm` assemblies in the library directory still reference the entry by path (the message lists them); pass `"force": true` to remove anyway |
| `assert_failed` | a declared expectation was not met by the measured geometry (an `assert`/`assert_disjoint` check, or the `asm` runner's mate-residual gate) — measured vs expected values in the message |
| `io` | a file could not be read/written |
| `internal` | a kernel panic, caught and surfaced (treat as a kernel bug; the message carries the panic text) |

---

# The assembly surface — `kernel-api asm`

The second subcommand executes a **`.lmcasm` assembly file** (the native
assembly format — instances of `.lmcpart` parts, `.lmcasm` sub-assemblies,
**or triangle-mesh files** (`{"mesh": "part.stl"}` — the bridge that lets
program-built / imported / scanned parts join a mated assembly, measured
honestly on their welded mesh) at rigid poses, mates, optional named states;
see `kernel_model::format`) end-to-end. Prefer AUTHORING assemblies with the
in-program `asm_*` ops (see *Assembly ops* in the op reference) and `asm_save`
— hand-writing `.lmcasm` is still supported, this subcommand executes either.
Mate kinds: `Coincident`, `Distance`, `Parallel`, `Concentric`, `Angle`
(directional, 0–180°), `AxisDistance` (parallel axes at a center distance —
the gear mate), `Fixed` (grounds any instance, not just index 0).

```bash
kernel-api asm gearbox.lmcasm [--base-dir DIR] [--out-dir DIR] \
                              [--tol MM] [--voxel MM] [--window MM]
```

| flag | default | meaning |
|---|---|---|
| `--base-dir` | the `.lmcasm` file's directory | where the **top file's** `path` / `asm_path` sources resolve (every nested `.lmcasm` resolves its own sources against its *own* directory — the same contract at every level) |
| `--out-dir` | `.` | where exports, `bom.json` and `bom.csv` are written |
| `--tol` | `0.05` | chord tolerance (mm) for exact B-rep tessellation |
| `--voxel` | `0.4` | voxel size (mm) for organic parts / the watertight heal — also the mesh-route volume for BOM masses |
| `--window` | `1.0` | proximity window (mm): pairs closer than this are listed with distances |

The output is the same JSON report shape as `run` (stdout; exit 0 iff every
step passed), with these steps in order:

| entry id | does | measures / failure |
|---|---|---|
| `load` | parse + instantiate (parts rebuilt from their documents, sub-assemblies resolved recursively and flattened to leaf parts, poses applied) | `name`, `units`, `instances` (leaf parts), `top_level` (the file's own instance count — equal for a flat assembly), `suppressed` (indices), `mates`, `states` (names); fails loudly on any format/part error, including a named `asm_path` include cycle |
| `mates` | report the on-load mate re-solve residual (the **max across all nesting levels** — each sub-assembly solves its own mates, then the file's mates place the units) with the honesty bundle | `residual`, `max_residual` (1e-6), `per_mate`: `[{index, kind, residual}]` (WHICH mate is unsatisfied), `dof`: numeric DOF report (`verdict` `well_constrained` / `under_constrained (N free DOF)`, `rank`, `redundant_rows`); statically broken mates (bad index, self-mate, zero direction) fail `invalid_param` instead of being silently skipped; `assert_failed` naming the worst offenders when the mates did not converge |
| `bom` | **BOM v2** → `bom.json` + (flat view) `bom.csv` | `schema: "bom/2"`, `flat`: `[{name, count, params, part_number?, material?, volume_source?, unit_mass_g?, line_mass_g?, make_or_buy?}]` (suppressed instances excluded; the optional fields come from each part envelope's `meta` block, mass = density × engine volume with the honest `volume_source` `"exact"`/`"mesh"`), `tree`: the per-instance nesting with rolled-up leaf counts, `csv`: the written CSV path |
| `export:<NN>:<name>` | one world-posed STL per leaf instance → `parts/NN_name.stl` | `part`, `triangles`, `watertight`, `route` (`exact` / `voxel_healed` — the honest routing verdict per part); sub-assembly members are named hierarchically (`stage1/bearing_l` → `parts/NN_stage1_bearing_l.stl`); suppressed instances report `{"suppressed": true}` and write nothing; an instance that produces NO geometry **fails** (`invalid_geometry`) |
| `export:assembly` | the merged assembled mesh → `<name>_assembly.stl` | `instances`, `triangles`, `watertight` |
| `export:assembly_step` | the AP214 STEP assembly (NAUO product tree, solved poses, volume-conserving) → `<name>_assembly.step` for every B-rep-backed instance | `parts`, `bytes`, `skipped`: mesh/organic instances listed with why (STEP carries no tessellation here); with NO B-rep instance the step reports that instead of writing an empty file |
| `contacts` | the proximity scan over all unsuppressed **leaf** pairs (B-rep parts measured on their **raw exact tessellation** at `--tol`, so sub-voxel fits like a 0.05 mm gear-flank gap are real measurements) | `window`, `tol`, `pairs`: `[{a, b, i, j, distance, touching}]` with hierarchical names, `touching` (count at distance ≤ 1e-6 — the designed-contact / interference class) |
| `state:<name>` | each named state applied + exported → `<name>_state_<state>.stl` | `triangles`, `watertight`, `suppressed` (count); the assembled poses are restored afterwards |

**Assembly nesting (v2) in one paragraph.** An instance whose source is
`{"asm_path": "sub.lmcasm"}` loads that file recursively (any depth; two
sibling instances of the same file are fine), lets it solve its **own** mates,
then places it as **one rigid unit** at the instance pose. The parent's mates
and states address **top-level instances only** — a mate's geometry for a
sub-assembly unit is written in the *sub-assembly's* frame, and mating to (or
un-suppressing) an internal member from the parent is out of scope in v2.
Suppressing a sub-assembly instance drops its entire branch from geometry,
contacts and BOM. Include cycles are refused loudly: `sub-assembly cycle:
'…/a.lmcasm' is already being loaded by this include chain (… -> …)`.
Sub-assemblies are path-referenced only (no inline assembly envelope).

Proving design intent on top of the scan is the caller's policy: e.g. the
gearbox pipeline (`gearbox/run_all.sh`) pipes both the flat and the nested
report through `gearbox/check_asm.py`, which allowlists the designed contacts
(52 in that assembly, classified by leaf name at any nesting level), pins the
nested BOM v2 tree rollup, and fails on any unexpected touching pair.

---

# Op reference

161 ops (mechanical count of `OpKind` — `describe` enumerates them all,
including the 13 assembly ops): solid constructors, sketch ops, booleans,
features/transforms, measures, assertions, exports/imports, implicit/hybrid
ops (including the voxel-route solid ops `offset_solid` / `shell_solid` /
`solid_from_implicit` and the interrogation probes `thin_wall` /
`min_ligament`), the native-format loader, the curated library, 48 standard
parts, 13 standard feature cuts, design-math lookups, the hole wizard — and
the **in-program assembly surface** (`asm_*` + `gear_train_poses`, next
section after Implicit/hybrid).

## Solid constructors

All constructors validate their result and fail rather than bind a broken or
empty solid. Segment counts control faceting of curved walls (the analytic
surface is still carried exactly on each face tag, which is what
`exact_volume` and STEP export read).

### `box`
Axis-aligned cuboid from two opposite corners.

| param | type | required | meaning |
|---|---|---|---|
| `min` | `[x,y,z]` | yes | low corner |
| `max` | `[x,y,z]` | yes | high corner (each component > `min`) |

```json
{"ops": [{"id": "plate", "op": "box", "min": [0, 0, 0], "max": [60, 40, 8]}]}
```

### `cylinder`
Right circular cylinder: base-cap center, axis direction, faceted side wall
tagged with the exact cylinder surface.

| param | type | required | meaning |
|---|---|---|---|
| `base` | `[x,y,z]` | yes | center of the base cap |
| `axis` | `[x,y,z]` | yes | extrusion direction (normalized internally) |
| `radius` | number | yes | radius > 0 |
| `height` | number | yes | height > 0 |
| `segments` | int | no (32) | wall facet count (≥ 3) |

```json
{"ops": [{"id": "pin", "op": "cylinder", "base": [0, 0, 0], "axis": [0, 0, 1],
          "radius": 3.5, "height": 10, "segments": 24}]}
```

### `sphere`
UV sphere (`u` segments around, `v` pole-to-pole), faces tagged with the exact
sphere.

| param | type | required | meaning |
|---|---|---|---|
| `center` | `[x,y,z]` | yes | center |
| `radius` | number | yes | radius > 0 |
| `u` | int | no (32) | segments around the axis |
| `v` | int | no (16) | segments pole to pole |

```json
{"ops": [{"id": "ball", "op": "sphere", "center": [0, 0, 20], "radius": 6, "u": 24, "v": 12}]}
```

### `cone`
Base disc tapering to an apex `height` along `axis` — or, with `top_radius`, the
**frustum** that same taper cuts at `height` (a draughted boss, a chamfered
spigot, a tapered stand-off).

| param | type | required | meaning |
|---|---|---|---|
| `base` | `[x,y,z]` | yes | center of the base disc |
| `axis` | `[x,y,z]` | yes | apex direction |
| `radius` | number | yes | base radius > 0 |
| `height` | number | yes | apex distance > 0 |
| `segments` | int | no (32) | wall facet count |
| `top_radius` | number | no (0) | flat top radius; 0 = a true cone (apex) |

The frustum's lateral band carries the same exact `Surface::Cone` tag as the
un-truncated cone, so `exact_volume` / `mass_properties` / `export_step` stay
analytic. `top_radius == radius` is refused (`invalid_param`): that solid is a
cylinder, and a cone surface with no apex is not representable — use `cylinder`.

```json
{"ops": [{"id": "tip", "op": "cone", "base": [0, 0, 0], "axis": [0, 0, 1], "radius": 5, "height": 12}]}
```

```json
{"ops": [{"id": "boss", "op": "cone", "base": [0, 0, 0], "axis": [0, 0, 1],
          "radius": 10, "height": 20, "top_radius": 4}]}
```

### `torus`
Ring torus about `axis` (`minor` must be < `major`).

| param | type | required | meaning |
|---|---|---|---|
| `center` | `[x,y,z]` | yes | ring center |
| `axis` | `[x,y,z]` | yes | ring plane normal |
| `major` | number | yes | ring radius |
| `minor` | number | yes | tube radius (< `major`) |
| `ring_segments` | int | no (48) | facets around the axis |
| `tube_segments` | int | no (24) | facets around the tube |

```json
{"ops": [{"id": "ring", "op": "torus", "center": [0, 0, 0], "axis": [0, 0, 1], "major": 20, "minor": 5}]}
```

### `extrude`
Linear extrusion of a closed **counter-clockwise** XY profile along +Z (a
negative `height` extrudes downward). CW input fails the validity gate.

| param | type | required | meaning |
|---|---|---|---|
| `profile` | `[[x,y], ...]` | yes | ≥ 3 distinct points, CCW, non-self-intersecting |
| `height` | number | yes | non-zero sweep distance |

```json
{"ops": [{"id": "lbar", "op": "extrude",
          "profile": [[0,0], [30,0], [30,10], [10,10], [10,25], [0,25]], "height": 6}]}
```

### `extrude_with_holes`
Plate with through-holes: outer loop + hole loops (each loop is re-wound
automatically; genus of the result = number of holes).

| param | type | required | meaning |
|---|---|---|---|
| `outer` | `[[x,y], ...]` | yes | outer boundary |
| `holes` | `[[[x,y], ...], ...]` | yes | zero or more hole loops strictly inside `outer` |
| `height` | number | yes | non-zero sweep distance |

```json
{"ops": [{"id": "washer_sq", "op": "extrude_with_holes",
          "outer": [[0,0], [40,0], [40,30], [0,30]],
          "holes": [[[10,10], [20,10], [20,20], [10,20]]], "height": 6}]}
```

### `extrude_tapered`
Drafted extrusion: every wall slopes inward by `draft_deg` so the part releases
from a mold. **Convex profiles only** (a concave vertex can self-intersect
under inset — rejected as `invalid_param` via the empty-solid gate). Holes are
not supported on the drafted path.

| param | type | required | meaning |
|---|---|---|---|
| `profile` | `[[x,y], ...]` | yes | convex CCW/CW polygon (re-wound automatically) |
| `height` | number | yes | non-zero sweep distance |
| `draft_deg` | number | yes | inward wall slope in degrees (0 = plain prism) |

```json
{"ops": [{"id": "boss", "op": "extrude_tapered",
          "profile": [[0,0], [30,0], [30,20], [0,20]], "height": 10, "draft_deg": 2}]}
```

### `revolve`
Full 360° revolution of a closed `(r, z)` profile about the **Z axis** —
`profile[i] = [radius, z]`, radii ≥ 0. Any simple polygon works (concave
included; winding is normalized). A profile point with `r ≈ 0` becomes a pole;
an isolated on-axis apex (both neighbours off-axis) is rejected — it would
pinch. Each profile edge carries its exact surface (cylinder/plane/cone).

| param | type | required | meaning |
|---|---|---|---|
| `profile` | `[[r,z], ...]` | yes | cross-section in the half-plane, r ≥ 0 |
| `segments` | int | no (64) | sectors around the axis |

```json
{"ops": [{"id": "flange_body", "op": "revolve",
          "profile": [[10,0], [40,0], [40,7], [39,8], [10,8]], "segments": 64}]}
```

### `loft`
Skin a closed solid through a stack of closed **3D** section loops. Every section
is `[[x,y,z], ...]` and **all sections must share the same point count** (≥ 3),
be ordered along the loft direction, and wind consistently (CCW seen from the
+direction). Adjacent sections are joined by triangulated lateral faces and the
ends are centroid-fan capped — the morph is **faceted** (honest tessellation; the
faceted volume equals the prismatoid integral exactly). Use it for transitions a
single profile can't make: square→round reducers, tapered ducts, twisted blends.

| param | type | required | meaning |
|---|---|---|---|
| `sections` | `[[[x,y,z], ...], ...]` | yes | ≥ 2 equal-length closed loops |

```json
{"ops": [{"id": "frustum", "op": "loft", "sections": [
  [[-20,-20,0], [20,-20,0], [20,20,0], [-20,20,0]],
  [[-10,-10,30], [10,-10,30], [10,10,30], [-10,10,30]]]}]}
```

### `sweep`
Sweep a closed **3D** `profile` along a 3D `path` polyline with a
rotation-minimising frame (the profile is planted at the path start and carried
along without twisting). Needs ≥ 3 profile points and ≥ 2 path points — a bent
square tube, an L-channel, a routed conduit. For a helical pitch use the implicit
`helix_pipe`/`pipe`; for a constant straight pull prefer `extrude`.

| param | type | required | meaning |
|---|---|---|---|
| `profile` | `[[x,y,z], ...]` | yes | closed section loop, ≥ 3 points |
| `path` | `[[x,y,z], ...]` | yes | centreline polyline, ≥ 2 points |

```json
{"ops": [{"id": "bend", "op": "sweep",
          "profile": [[-4,-4,0], [4,-4,0], [4,4,0], [-4,4,0]],
          "path": [[0,0,0], [0,0,25], [20,0,25]]}]}
```

## Sketch ops

### `sketch`
A constrained 2D sketch. It is **solved on creation** (Levenberg–Marquardt) and
the report carries the solve + degree-of-freedom analysis. Conflicting
constraints (no convergence) fail the op with `sketch_failed`;
under-constrained sketches are allowed and labelled. Point indices reference
the `points` array; all indices are bounds-checked.

| param | type | required | meaning |
|---|---|---|---|
| `points` | `[[x,y], ...]` | yes | initial point positions (the solver moves them) |
| `segments` | `[[i,j], ...]` | no | straight edges between point indices |
| `arcs` | `[{a,b,center,ccw?}, ...]` | no | arc edges; `center` is a construction point, `ccw` defaults true |
| `circles` | `[{center,radius_point}, ...]` | no | standalone full circle (only as the sketch's single profile) |
| `constraints` | array | no | see kinds below |

Constraint kinds (`"kind"` plus the listed fields, all point indices):

| kind | fields | holds |
|---|---|---|
| `fixed` | `point`, `at: [x,y]` | pins a point to a position |
| `coincident` | `a`, `b` | two points share one position |
| `horizontal` | `a`, `b` | `y_a == y_b` |
| `vertical` | `a`, `b` | `x_a == x_b` |
| `distance` | `a`, `b`, `distance` | point separation |
| `parallel` | `a`, `b`, `c`, `d` | direction `a→b` ∥ `c→d` |
| `perpendicular` | `a`, `b`, `c`, `d` | direction `a→b` ⟂ `c→d` |
| `equal_length` | `a`, `b`, `c`, `d` | length of `a→b` equals length of `c→d` |
| `tangent` | `line_a`, `line_b`, `center`, `radius_point` | line tangent to circle |
| `angle` | `a`, `b`, `c`, `d`, `degrees` | angle between `a→b` and `c→d` (magnitude — may settle at ±) |
| `symmetric` | `a`, `b`, `line_a`, `line_b` | `a`,`b` mirror across the line |

Measures reported: `residual`, `iterations`, `converged`, `dof`, `rank`,
`free_dof`, `redundant`, `state` (`under_constrained` / `well_constrained` /
`over_constrained`).

```json
{"ops": [{"id": "rect", "op": "sketch",
  "points": [[1,2], [55,3], [58,44], [-2,38]],
  "segments": [[0,1], [1,2], [2,3], [3,0]],
  "constraints": [
    {"kind": "fixed", "point": 0, "at": [0, 0]},
    {"kind": "horizontal", "a": 0, "b": 1},
    {"kind": "distance", "a": 0, "b": 1, "distance": 60},
    {"kind": "vertical", "a": 0, "b": 3},
    {"kind": "distance", "a": 0, "b": 3, "distance": 40},
    {"kind": "horizontal", "a": 3, "b": 2},
    {"kind": "vertical", "a": 1, "b": 2}
  ]}]}
```

### `sketch_extrude`
Extrude a solved sketch along +Z. A single standalone circle extrudes to an
**exact analytic cylinder**; multiple closed loops become outer + holes
automatically. Open chains fail with `sketch_failed`.

| param | type | required | meaning |
|---|---|---|---|
| `sketch` | id | yes | a prior `sketch` op |
| `height` | number | yes | non-zero sweep distance |

```json
{"ops": [
  {"id": "sq", "op": "sketch", "points": [[0,0], [20,0], [20,20], [0,20]],
   "segments": [[0,1], [1,2], [2,3], [3,0]]},
  {"id": "block", "op": "sketch_extrude", "sketch": "sq", "height": 10}
]}
```

### `sketch_revolve`
Revolve a solved sketch about Z, reading its `(x, y)` as `(r, z)` — same domain
rules as `revolve` (r ≥ 0).

| param | type | required | meaning |
|---|---|---|---|
| `sketch` | id | yes | a prior `sketch` op |
| `segments` | int | no (64) | sectors |

```json
{"ops": [
  {"id": "section", "op": "sketch", "points": [[10,0], [16,0], [16,6], [10,6]],
   "segments": [[0,1], [1,2], [2,3], [3,0]]},
  {"id": "bush", "op": "sketch_revolve", "sketch": "section", "segments": 32}
]}
```

## Booleans

Exact planar-arrangement booleans with persistent face naming (the result's
faces remember which operand they came from, which is what keeps fillet
witnesses resolvable on boolean results). An **empty** result (e.g. a disjoint
`intersection`, or a `difference` that consumes everything) is a loud
`invalid_param` — there is nothing to bind. Coplanar contact, faces with inner
loops, and cuts across previously-cut curved walls are supported (the R1–R4
robustness work).

### `union`

| param | type | required | meaning |
|---|---|---|---|
| `a`, `b` | id | yes | prior solids |

```json
{"ops": [
  {"id": "plate", "op": "box", "min": [0,0,0], "max": [30,20,10]},
  {"id": "dome", "op": "sphere", "center": [15,10,10], "radius": 6, "u": 24, "v": 12},
  {"id": "body", "op": "union", "a": "plate", "b": "dome"}
]}
```

### `difference`

```json
{"ops": [
  {"id": "plate", "op": "box", "min": [0,0,0], "max": [30,20,10]},
  {"id": "drill", "op": "cylinder", "base": [15,10,-1], "axis": [0,0,1], "radius": 3, "height": 12},
  {"id": "holed", "op": "difference", "a": "plate", "b": "drill"}
]}
```

### `intersection`

```json
{"ops": [
  {"id": "a", "op": "box", "min": [0,0,0], "max": [30,20,10]},
  {"id": "b", "op": "box", "min": [-5,-5,-5], "max": [8,8,8]},
  {"id": "common", "op": "intersection", "a": "a", "b": "b"}
]}
```

### `union_all`
n-ary union: folds every solid listed in `in` (≥ 2 ids) into one result — the
one-op form of a chained `union` ladder. Since 2026-08-27 the fold order is
robustness-aware rather than left-to-right: operands merge in ascending order
of how many other operands' AABBs they touch (ties keep argument order, so
the fold stays deterministic). Mutually-disjoint operands combine first as a
cheap multi-shell union and a touch-everything "hub" operand is arranged
once, last — a hub whose same face was re-arranged once per contacting
operand used to fail `invalid_geometry` mid-ladder. Union is associative, so
the resulting solid is unchanged. Disjoint bodies keep their own shells, so
`union_all` + `assert {"shells": N}` is an N-body no-contact proof.

| param | type | required | meaning |
|---|---|---|---|
| `in` | array of ids | yes | at least two prior solids |

```json
{"ops": [
  {"id": "a", "op": "box", "min": [0,0,0], "max": [5,5,5]},
  {"id": "b", "op": "box", "min": [10,0,0], "max": [15,5,5]},
  {"id": "c", "op": "box", "min": [20,0,0], "max": [25,5,5]},
  {"id": "all", "op": "union_all", "in": ["a", "b", "c"]},
  {"id": "no_contact", "op": "assert", "in": "all", "shells": 3}
]}
```

**Known hazard — a long fold of mostly-disjoint bodies can fail to terminate.**
Measured on a 13-cutter fold (a Ø56 disc's cutters: 8 × Ø10 cylinders on a Ø42
bolt circle, a Ø20.6 bore, 3 × Ø6 notches centred ON the bore wall, and a Ø28.6
groove ring), folding the first *n* of those thirteen:

| n | 2 | 9 | 10 | 11 | 12 | 13 |
|---|---|---|---|---|---|---|
| wall time | 0.44 s | 0.11 s | 0.22 s | 0.42 s | 0.85 s | **no completion in 100 s** |

The cost doubles per body from n = 9 and then does not finish, and a BALANCED
(tournament) fold of the same thirteen behaves identically — so this is the
boolean arrangement over a many-shell accumulator, not the fold order. Every
individual pair unions in well under a second.

Until that is fixed: build the same solid as a CHAIN of `difference` ops against
virgin primitives (the identical 13-cutter part completes in 0.38 s that way),
and keep `union_all` for short folds. `assert {"shells": N}` still works, and is
still the tessellation-independent no-contact proof, for folds that complete.

## Features & transforms

### `fillet_edge_near`
Round the **straight** edge nearest `witness` with a constant-radius
cylindrical fillet. Witness resolution: the nearest *named* edge of the solid
is selected; if it lies farther than `max_distance` (default **10% of the
solid's bounding-box diagonal**) the op fails with `feature_failed` — a witness
must actually point at an edge, never silently grab a far one.

HONEST SCOPE: a **convex** straight edge shared by two **planar** faces — any
convex dihedral angle (every box edge, prism edges), with simple 3-face
corners at both ends. **Concave junctions are out of scope** (inside corners,
e.g. where a wall meets a base plate: the round would *add* material — a cove,
not a fillet) and are refused with `feature_failed` for both this op and
`chamfer_edge_near`; model the cove explicitly instead (difference a cylinder
from a corner bar to leave a quarter-round strip, union it into the junction).
Curved-face edges are rejected with `feature_failed` — for the rim of a
cylinder/boss use `fillet_circular_rim` (convex rims only). The radius must
fit inside both adjacent faces.

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid |
| `witness` | `[x,y,z]` | yes | a point on/near the edge to round |
| `radius` | number | yes | fillet radius > 0 |
| `max_distance` | number | no (10% of bbox diagonal) | witness rejection distance |

```json
{"ops": [
  {"id": "plate", "op": "box", "min": [0,0,0], "max": [30,20,10]},
  {"id": "soft", "op": "fillet_edge_near", "in": "plate", "witness": [15, 0, 10], "radius": 2}
]}
```

### `chamfer_edge_near`
Flat-bevel sibling of `fillet_edge_near` (same witness rule, same scope);
`radius` is the setback of the two tangent lines.

```json
{"ops": [
  {"id": "plate", "op": "box", "min": [0,0,0], "max": [30,20,10]},
  {"id": "beveled", "op": "chamfer_edge_near", "in": "plate", "witness": [15, 20, 10], "radius": 2}
]}
```

### `fillet_circular_rim`
Fillet a **circular convex rim** (cylindrical wall meeting a planar end cap)
with the exact rolling-ball **torus** band — works on bare cylinders and on
bosses fused onto other bodies. `witness` picks the rim when several qualify
(a bare cylinder has two).

HONEST SCOPE (otherwise `feature_failed`): convex boss rims only (no concave
wall-meets-plate junctions, no bore exit lips); the cap must be a full circle
with no inner loops and trivalent rim vertices; `radius` strictly less than
the cap radius, wall at least `radius` long.

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid |
| `witness` | `[x,y,z]` | yes | a point near the rim circle |
| `radius` | number | yes | fillet radius > 0 |
| `arc_segments` | int | no (8) | facets along the quarter arc |

```json
{"ops": [
  {"id": "boss", "op": "cylinder", "base": [0,0,0], "axis": [0,0,1], "radius": 6, "height": 12, "segments": 48},
  {"id": "soft_rim", "op": "fillet_circular_rim", "in": "boss", "witness": [6, 0, 12], "radius": 1.5, "arc_segments": 6}
]}
```

### `translate`
Rigid translation (re-validated like every solid op; analytic surface tags and
edge curves are transformed too).

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid |
| `offset` | `[x,y,z]` | yes | translation vector |

```json
{"ops": [
  {"id": "b", "op": "box", "min": [0,0,0], "max": [10,10,10]},
  {"id": "moved", "op": "translate", "in": "b", "offset": [100, 0, 0]}
]}
```

### `rotate_z`
Rigid rotation about the **world Z axis** through the origin.

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid |
| `degrees` | number | yes | counter-clockwise angle (viewed from +Z) |

```json
{"ops": [
  {"id": "b", "op": "box", "min": [0,0,0], "max": [10,10,10]},
  {"id": "turned", "op": "rotate_z", "in": "b", "degrees": 30}
]}
```

### `rotate_x`
Rigid rotation about the **world X axis** through the origin — the exact
sibling of `rotate_z` (identical to a `pose` with `rotate.axis = [1,0,0]`).

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid |
| `degrees` | number | yes | counter-clockwise angle (viewed from +X) |

```json
{"ops": [
  {"id": "b", "op": "box", "min": [0,0,0], "max": [10,10,10]},
  {"id": "tipped", "op": "rotate_x", "in": "b", "degrees": 90}
]}
```

### `rotate_y`
Rigid rotation about the **world Y axis** through the origin — the exact
sibling of `rotate_z` (identical to a `pose` with `rotate.axis = [0,1,0]`).

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid |
| `degrees` | number | yes | counter-clockwise angle (viewed from +Y) |

```json
{"ops": [
  {"id": "b", "op": "box", "min": [0,0,0], "max": [10,10,10]},
  {"id": "rolled", "op": "rotate_y", "in": "b", "degrees": -45}
]}
```

### `pose`
**General rigid pose**: rotation about an ARBITRARY axis (through an optional
center point), THEN a translation — the full `Rx(-90°)·Rz(phase)`-style
placement an assembly needs (a `.lmcasm` instance pose is exactly
`rotate` + `translate`). At least one of the two parts is required (an empty
pose is a loud `invalid_param`). Chain two `pose` ops for composed rotations.

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid |
| `rotate` | object | no | `{"axis": [x,y,z], "degrees": d, "center": [x,y,z]?}` — right-handed about `axis` (any non-zero vector) through `center` (default the origin) |
| `translate` | `[x,y,z]` | no | translation applied AFTER the rotation |

```json
{"ops": [
  {"id": "gear", "op": "spur_gear", "module": 1.25, "teeth": 12, "face_width": 12, "bore": 8},
  {"id": "posed", "op": "pose", "in": "gear",
    "rotate": {"axis": [1, 0, 0], "degrees": -90},
    "translate": [0, -6, 48]}
]}
```

### `mirror`
Reflect a solid across a plane given as a point + normal. **Orientation-safe
by construction**: the kernel's dedicated `Solid::mirrored` rebuilds every face
loop reversed, so the reflected copy is a correctly-oriented (outward-normal)
valid solid — never the inside-out shape a raw reflection matrix would produce.
Inner hole loops are carried, so mirroring a part with a pocket or bore keeps
the hole. A zero/non-finite `plane.normal` is a loud `invalid_param` (the op
never silently returns the unreflected input).

`mirror` reflects in place; it does NOT union the original with its image —
`union` the two ids for a symmetric part.

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid |
| `plane` | object | yes | `{"point": [x,y,z], "normal": [x,y,z]}` — the mirror plane; `normal` is any non-zero vector, normalized internally |

```json
{"ops": [
  {"id": "half", "op": "extrude", "profile": [[0,0], [30,0], [30,10], [8,10], [0,22]], "height": 5},
  {"id": "other", "op": "mirror", "in": "half", "plane": {"point": [0,0,0], "normal": [1,0,0]}},
  {"id": "sym", "op": "union", "a": "half", "b": "other"}
]}
```

### `linear_pattern`
`count` clones of a solid at offsets `i·step` (i = 0 … count−1, so the original
is instance 0), folded into ONE solid with the exact boolean union.

**Disjoint clones are an honest multi-shell solid** (verified behavior of the
exact union, pinned in the kernel's boolean tests): `validate().shells ==
count`, closed, manifold, volume = count × the single volume. Overlapping
clones fuse into fewer shells with the overlap counted once — either way the
result passes the same validate gate as every solid op. A zero `step` is
rejected (`invalid_param`): coincident clones are a boolean degeneracy, not a
pattern.

Caps (structured `invalid_param`, checked before any allocation): `count`
2..=500, and `count × per-clone face count` ≤ 100 000.

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid |
| `count` | int | yes | instances INCLUDING the original, 2..=500 |
| `step` | `[x,y,z]` | yes | per-instance offset (mm), non-zero |

```json
{"ops": [
  {"id": "post", "op": "cylinder", "base": [0,0,0], "axis": [0,0,1], "radius": 3, "height": 20},
  {"id": "fence", "op": "linear_pattern", "in": "post", "count": 5, "step": [15, 0, 0]},
  {"id": "gate", "op": "assert", "in": "fence", "valid": true, "shells": 5}
]}
```

### `polar_pattern`
`count` clones of a solid rotated `k·step_deg` (k = 0 … count−1) about `axis`
through `center`, folded into ONE solid with the exact boolean union — same
disjoint-shells / overlap-fuse behavior and the same 500-count / 100 000-face
caps as `linear_pattern`. `step_deg` defaults to `360 / count` (a full evenly
spaced ring); a multiple of 360° is rejected — the clones would coincide.

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid |
| `count` | int | yes | instances INCLUDING the original, 2..=500 |
| `center` | `[x,y,z]` | yes | a point on the rotation axis |
| `axis` | `[x,y,z]` | yes | rotation axis, any non-zero vector |
| `step_deg` | number | no (`360/count`) | angular pitch between instances, right-handed about `axis` |

```json
{"ops": [
  {"id": "spoke", "op": "box", "min": [8, -1.5, 0], "max": [28, 1.5, 4]},
  {"id": "wheel_spokes", "op": "polar_pattern", "in": "spoke", "count": 6, "center": [0, 0, 0], "axis": [0, 0, 1]},
  {"id": "gate", "op": "assert", "in": "wheel_spokes", "valid": true, "shells": 6}
]}
```

## Measures

Measure ops bind no value — they attach numbers to the report. They require a
solid input (`wrong_type` on a sketch).

### `validate`
Topological health. Note: every solid-producing op already gates on this; the
op exists so a program can RECORD the topology (e.g. assert genus) in its
report. Accepts a solid or a bound **mesh**.

Measures (solid): `closed`, `manifold`, `euler_characteristic`, `genus`,
`shells`, `valid`, `geometric_ok`, `source: "solid"`.
Measures (mesh): `closed`, `manifold`, `valid`, `triangles`, `boundary_edges`,
`non_manifold_edges`, `non_orientable_edges`, `geometric_ok`, `source: "mesh"`.

`geometric_ok` is the GEOMETRIC validity flag: `false` means two triangles of
the tessellation properly cross, i.e. the surface passes through itself. A solid
can be closed, manifold and watertight and still be geometrically invalid, with
a silently-wrong volume — and the exported STL inherits the crossing.

When `geometric_ok` is `false` the report carries the **witness**, so the flag is
actionable rather than something to learn to ignore:

```json
"self_intersection": {"triangles": [732, 1206],
                      "point": [-21.7768, 15.1400, -1.5870],
                      "pairs": 20}
```

`triangles` are the two crossing triangle indices in the measurement
tessellation, `point` is a point on the crossing, `pairs` is how many crossing
pairs exist. The witness is deterministic (the lexicographically lowest pair).

```json
{"ops": [
  {"id": "b", "op": "box", "min": [0,0,0], "max": [10,10,10]},
  {"id": "check", "op": "validate", "in": "b"}
]}
```

### `volume`
Signed enclosed volume of the **faceted** B-rep (positive for a valid outward
solid; exact for planar-faced solids, faceted-approximate for curved walls).

Measures: `volume`.

```json
{"ops": [
  {"id": "b", "op": "box", "min": [0,0,0], "max": [20,10,5]},
  {"id": "v", "op": "volume", "in": "b"}
]}
```

### `exact_volume`
Analytic volume recovered from the faces' exact surface tags — π-exact on
tagged cylinders/spheres/cones (e.g. a drilled hole subtracts `π r² h`, not a
24-gon prism). Falls back to the faceted contribution for untagged faces.

Measures: `exact_volume`.

```json
{"ops": [
  {"id": "pin", "op": "cylinder", "base": [0,0,0], "axis": [0,0,1], "radius": 3, "height": 10},
  {"id": "xv", "op": "exact_volume", "in": "pin"}
]}
```

### `mass_properties`
Unit-density rigid-body properties (multiply by your material density).
Inertia is about the center of mass in model axes; the report carries the
tensor **diagonal** `[Ixx, Iyy, Izz]`.

Measures: `volume`, `center_of_mass`, `inertia_diag`.

```json
{"ops": [
  {"id": "b", "op": "box", "min": [0,0,0], "max": [20,10,5]},
  {"id": "mp", "op": "mass_properties", "in": "b"}
]}
```

### `bounding_box`
Axis-aligned bounding box of the part, plus the quantities an operator/AI reads
off it: overall `size` = L×W×H, `center`, and the corner-to-corner `diagonal`.
Give an optional `envelope` `[x,y,z]` (a printer bed, a mill envelope, a stock
block) and the report adds `fits_within` (current orientation) and
`fits_within_rotated` (allowing a 90° axis re-orientation) — the practical
"does this part fit?" release check.

| param | type | required | meaning |
|---|---|---|---|
| `in` | string | yes | id of the solid to measure |
| `envelope` | `[x,y,z]` | no | build volume / stock to test the fit against |

Measures: `min`, `max`, `size`, `center`, `diagonal` (+ `fits_within`,
`fits_within_rotated` when `envelope` is given).

```json
{"ops": [
  {"id": "b", "op": "box", "min": [0,0,0], "max": [40,20,10]},
  {"id": "bb", "op": "bounding_box", "in": "b", "envelope": [25,45,15]}
]}
```

### `measure_dimension`
Measure **one dimension** of a bound solid, exactly where the analytic
geometry allows — the drawing-callout measure op (FRICTION #21). Three kinds:

- `point_point`: distance between two given points (provenance
  `coordinates` — the points are the caller's claim, the distance is exact
  arithmetic on them).
- `face_face`: perpendicular distance between two **parallel planar faces**,
  each selected by a witness point (nearest face centroid — the same anchor
  `list_faces` reports). Provenance `analytic`: computed from the plane
  equations, not the mesh. Non-planar or non-parallel selections fail
  `invalid_param` LOUDLY with the selected face types / the measured angle —
  never a silently-wrong number.
- `diameter`: Ø of the cylindrical or spherical face nearest `near` —
  provenance `analytic`, exactly `2·radius` from the surface tag. Cones and
  tori are refused by design (their Ø varies; the error says what was hit
  and what to do instead).

The measures carry everything a drawing needs (value, provenance, selected
faces' descriptors and witness anchors); `render_sheet.py` consumes them as
dimension callouts, and `tools/dim_suggest.py` auto-drafts a callout set from
`list_faces` + this op.

| param | type | required | meaning |
|---|---|---|---|
| `in` | string | yes | id of the solid to measure |
| `kind` | string | yes | `"point_point"` / `"face_face"` / `"diameter"` |
| `a` | `[x,y,z]` | point_point / face_face | first point, or witness selecting the first face |
| `b` | `[x,y,z]` | point_point / face_face | second point, or witness selecting the second face |
| `near` | `[x,y,z]` | diameter | witness selecting the measured cylinder/sphere face |

Measures: `kind`, `value` (mm), `provenance` (`analytic` / `coordinates`),
plus per-kind anchors (`face`/`face_a`/`face_b` descriptors, `delta`).

```json
{"ops": [
  {"id": "plate", "op": "box", "min": [0,0,0], "max": [60,40,8]},
  {"id": "bore", "op": "drill", "in": "plate", "at": [30,20,8], "axis": [0,0,-1], "d": 6, "through": 8},
  {"id": "thick", "op": "measure_dimension", "in": "bore", "kind": "face_face", "a": [30,20,0], "b": [30,20,8]},
  {"id": "dia", "op": "measure_dimension", "in": "bore", "kind": "diameter", "near": [33,20,4]}
]}
```

### `wall_thickness`
Ray-based wall thickness (inward ray per facet to the opposite wall).
`min_thickness` reads oblique distances on sharp-corner facets — judge thin
walls by `thin_area` against your `flag_below`, not the raw minimum alone.

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid |
| `flag_below` | number | yes | thinness threshold (mm) |

Measures: `min_thickness`, `p05_thickness`, `median_thickness`, `thin_area`,
`flag_below`, `sampled_triangles`. In practice `min_thickness` is corner noise
(oblique rays at sharp corners can read near-zero); the robust signals are the
percentiles (`p05_thickness` / `median_thickness`, over the finite per-triangle
samples) and `thin_area` against `flag_below`.

```json
{"ops": [
  {"id": "b", "op": "box", "min": [0,0,0], "max": [30,20,10]},
  {"id": "wt", "op": "wall_thickness", "in": "b", "flag_below": 1}
]}
```

### `draft_analysis`
Moldability against a pull direction: minimum draft angle, area below `min_deg`,
and undercut area (faces trapped between both mold halves). Walls parallel to
pull have 0° draft — a plain box reports `min_draft_deg: 0`.

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid |
| `pull` | `[x,y,z]` | yes | mold pull direction (non-zero) |
| `min_deg` | number | yes | required draft in degrees |

Measures: `min_draft_deg`, `low_draft_area`, `undercut_area`.

```json
{"ops": [
  {"id": "boss", "op": "extrude_tapered", "profile": [[0,0],[30,0],[30,20],[0,20]],
   "height": 10, "draft_deg": 2},
  {"id": "da", "op": "draft_analysis", "in": "boss", "pull": [0, 0, 1], "min_deg": 1}
]}
```

### `mesh_components`
Connected-body count of the tessellated solid — the **single-body oracle** the
other gates cannot give. `shells` counts B-rep shell *records*, which can still
read 1 on a part severed into floating lumps (docs/FRICTION.md #24: a tapered
cutter's apex run out through a wall leaves a free-floating panel that passes
`validate`, watertightness, volume, sweeps and STEP round-trip). This measure
tessellates the exact surfaces, position-welds vertices at `weld_tol` (so
coincident-but-unshared boolean vertices count as one point), and union-finds
actual triangle connectivity.

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid **or a bound mesh** |
| `tol` | number | no | chord tolerance (mm) of the measurement tessellation (default 0.05); ignored for a bound mesh, which IS its triangles |
| `weld_tol` | number | no | position-weld scale (mm) for vertex identity (default 1e-3, the house weld scale) |

Measures: `components`, `is_one_body`, `triangles`, `tol`, `weld_tol`,
`watertight`, `boundary_edges`, `non_orientable_edges`, `source`,
`provenance: "faceted"`. Gate it with `require {"components": 1}` (or
`assert {"components": 1}`) — any campaign that subtracts a tapered or tapering
cutter must.

`watertight` here is the rigorous 2-manifold test, so it can be `false` while
`boundary_edges` is 0. When it is, `non_orientable_edges` is why: some triangles
are wound inside-out. That closes every edge and shares every vertex, so it
cannot change the component count — it is reported, never refused.

#### `components` and `shells` are COMPLEMENTARY — neither dominates

| | catches | misses |
|---|---|---|
| `components` | a part severed into floating lumps that the B-rep still records as ONE shell | a severance **narrower than `weld_tol`** — the two faces weld together and read as one body |
| `shells` | a severance of any width, down to sub-micron, because the boolean records a new shell | a severance the B-rep does not record as a shell split |

Run both. A worked case: two boxes with a **0.0005 mm** gap read `shells: 2`
(severance seen) and `components: 1` at the default `weld_tol` of 1e-3
(severance welded shut). Dropping `weld_tol` to `1e-6` makes `components` see it
too — which is why `weld_tol` is exposed on `assert` as well as here. A hard
severance proof needs `weld_tol` **below** the gap being ruled out.

`weld_tol` is a true tolerance, not a grid pitch: any two vertices no farther
apart than `weld_tol` are one point, wherever the part sits in space.

#### Refusal: an untrustworthy measurement surface

A bound **solid** is closed and manifold by construction, so if its measurement
tessellation has boundary edges the faceter has dropped geometry and the
component count is counting faceter cracks, not bodies. The op then FAILS with
`invalid_geometry` rather than report the number — the message gives the
boundary-edge count and the count it would have reported. Gate `validate`
(`closed` / `manifold` / `shells`) meanwhile, and/or `export_stl` the part and
run this measure on the export's bound mesh, which is what actually prints. A
bound **mesh** is never refused: openness there is a property of the data, and
is reported as `watertight: false` for `require` to gate.

**Only an OPENING trips this.** A boundary edge is an undirected edge used by
exactly ONE triangle. A *winding* defect — two triangles sharing an edge and
traversing it the same way — closes that edge perfectly and is counted under
`non_orientable_edges`, not here. The distinction is load-bearing: until
2026-08-08 `Mesh::boundary_edge_count` asked "is the reverse directed edge
absent?", which is true for every non-orientable edge, and this refusal
therefore fired on 11 shipped part programs across 8 campaigns whose
tessellations are closed and whose component count was correct.

```json
{"ops": [
  {"id": "a", "op": "box", "min": [0,0,0], "max": [10,10,10]},
  {"id": "b", "op": "box", "min": [20,0,0], "max": [30,10,10]},
  {"id": "u", "op": "union", "a": "a", "b": "b"},
  {"id": "mc", "op": "mesh_components", "in": "u"}
]}
```

reports `components: 2, is_one_body: false` — the disjoint union is one bound
solid in two bodies.

### `coincident_fit`
Advisory **pre-scan for the near-coincident-face hazard class**
(`kernel_brep::detect_coincident_fit`) — the question to ask BEFORE booleaning
two solids that may share a flush or press-fit face pair. `true` means some
face of `a` and some face of `b` lie on nearly the same analytic surface
(within **1e-3 rad** of direction and **0.05 mm** of offset/radius) **and**
their extents actually come near each other (the two face AABBs within that
same 0.05 mm), so faces that merely share a supporting plane from far apart do
not trigger. Planes match sign-insensitively on normal + offset (a flush
CONTACT has anti-parallel normals); cylinders on axis direction, axis-line
separation and radius — the press-fit class the kernel note records as a Ø2
pin against a Ø1.95 pocket that ground for 53 CPU-minutes without finishing;
sphere / cone / torus on their analytic parameters at the same tolerances.

**A `true` is a CLASS, not a verdict** — the same honesty as the assembly
runner's `contacts.touching`. A designed flush stack answers `true` and unions
in milliseconds; a press fit answers `true` and may never finish. The scan
mutates nothing, refuses nothing and proves nothing about one particular
boolean: it says which question to ask next — measure the fit numerically (the
`clearance` op, `assert_disjoint`) or move the surfaces apart, instead of
unioning across the coincident pair. Symmetrically a `false` means "outside
THIS class at these tolerances", never "this boolean is safe": a 0.2 mm slip
fit reads `false`. Cost is O(faces of `a` × faces of `b`) parameter
comparisons with no arrangement, so it is safe to run on the very pair that
would hang. (Deeper pre-flight — cutter embedment, facet-meridian alignment —
is `kernel_brep::boolean_hazards` on the Rust surface, DESIGN_GUIDE §7.7; this
op is its single-question shortcut.)

| param | type | required | meaning |
|---|---|---|---|
| `a`, `b` | ids | yes | two prior solids |

Measures: `coincident_fit` (bool).

Executed — four questions on one plate: a Ø4 pin against a Ø3.95 press pocket
(radius difference 0.025 mm, inside the 0.05 mm tolerance), the same pin
against a Ø4.4 slip pocket (0.2 mm, outside it), a plate/cap pair sharing the
z = 6 plane (flush contact — in the class, and perfectly safe to union), and
the press pocket against a pin parked 40 mm away (killed by the solid-level
AABB reject):

```json
{"ops": [
  {"id": "plate", "op": "box", "min": [-10, -10, 0], "max": [10, 10, 6]},
  {"id": "press_pocket", "op": "drill", "in": "plate", "at": [0, 0, 6], "axis": [0, 0, -1], "d": 3.95, "through": 6},
  {"id": "slip_pocket", "op": "drill", "in": "plate", "at": [0, 0, 6], "axis": [0, 0, -1], "d": 4.4, "through": 6},
  {"id": "pin", "op": "cylinder", "base": [0, 0, -3], "axis": [0, 0, 1], "radius": 2, "height": 12},
  {"id": "press", "op": "coincident_fit", "a": "pin", "b": "press_pocket"},
  {"id": "slip", "op": "coincident_fit", "a": "pin", "b": "slip_pocket"},
  {"id": "cap", "op": "box", "min": [-10, -10, 6], "max": [10, 10, 10]},
  {"id": "flush", "op": "coincident_fit", "a": "plate", "b": "cap"},
  {"id": "far_pin", "op": "cylinder", "base": [40, 0, 0], "axis": [0, 0, 1], "radius": 2, "height": 12},
  {"id": "apart", "op": "coincident_fit", "a": "far_pin", "b": "press_pocket"}
]}
```

```json
{"id": "press", "ok": true, "measures": {"coincident_fit": true}}
{"id": "slip",  "ok": true, "measures": {"coincident_fit": false}}
{"id": "flush", "ok": true, "measures": {"coincident_fit": true}}
{"id": "apart", "ok": true, "measures": {"coincident_fit": false}}
```

### `support_report`
FDM **support-necessity** audit of a bound solid printed as-oriented with
`build_dir` up (`kernel_core::Mesh::support_free_report`, read off the
adaptive tessellation at a fixed 0.05 mm chord — hence
`provenance: "faceted"`). Every facet whose normal points down past
`overhang_deg` **from vertical** (a wall is 0°, a flat ceiling 90°; 45° is the
usual FDM limit) is sorted into three honest buckets:

- `bed_area` — facets lying entirely within 0.2 mm of the lowest point along
  `build_dir`: the first layer on the plate, never needs support.
- `bridge_area` — ceilings within 1° of dead flat: the printer bridges them.
  `max_bridge_span` is the widest patch's TRUE span — 2 × the deepest interior
  point's distance to that patch's boundary — so the 20 × 10 mm lintel below
  spans **10** (the short way across), not 20 — and by the same construction
  an annular ceiling spans its radial width, not its diameter. FDM handles
  ~5–10 mm trivially; long spans droop.
- `steep_area` — everything else facing down: the area that would need support
  material. `support_free` is exactly `steep_area < 1e-6`.

**`steep_area` is a CLASS, not a defect count** — the same honesty as the
assembly runner's `contacts.touching`: a deliberately supported overhang and a
modelling accident land in the same bucket, and the op reports the AREA, never
an intent. It audits ONE orientation per call (the bored wall below is
`support_free: false` upright and `true` laid on its side), and it reports
areas ONLY — the per-triangle flags, the steep exemplar points and the
per-patch spans exist on the Rust surface's `SupportFreeReport` but are not
carried into JSON, so this op answers "how much", never "where".

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid |
| `build_dir` | `[x,y,z]` | no (`[0, 0, 1]`) | print-up direction (normalized internally) |
| `overhang_deg` | number | no (`45`) | steepest overhang that still prints unsupported, in degrees from vertical |

Measures: `support_free`, `bed_area`, `bridge_area`, `steep_area`,
`total_area`, `max_bridge_span`, `provenance` (`"faceted"`).

Executed — a 20 × 10 × 30 wall with a Ø8 horizontal bore (the classic sagging
ceiling: 62.7 mm² steep), the same wall bored as a `teardrop_hole`, and a
20 mm-wide portal whose flat lintel is the bridge case:

```json
{"ops": [
  {"id": "wall", "op": "box", "min": [-10, -5, 0], "max": [10, 5, 30]},
  {"id": "round", "op": "drill", "in": "wall", "at": [0, 5, 15], "axis": [0, -1, 0], "d": 8, "through": 10},
  {"id": "as_bored", "op": "support_report", "in": "round"},
  {"id": "on_side", "op": "support_report", "in": "round", "build_dir": [0, 1, 0]},
  {"id": "tear", "op": "teardrop_hole", "in": "wall", "at": [0, 5, 15], "axis": [0, -1, 0], "up": [0, 0, 1], "d": 8, "through": 10},
  {"id": "teardrop_45", "op": "support_report", "in": "tear"},
  {"id": "teardrop_46", "op": "support_report", "in": "tear", "overhang_deg": 46},
  {"id": "block", "op": "box", "min": [40, 0, 0], "max": [80, 10, 20]},
  {"id": "gap", "op": "box", "min": [50, -1, -1], "max": [70, 11, 12]},
  {"id": "portal", "op": "difference", "a": "block", "b": "gap"},
  {"id": "lintel", "op": "support_report", "in": "portal"}
]}
```

```json
{"id": "as_bored", "ok": true,
 "measures": {"bed_area": 200.0, "bridge_area": 0.0, "max_bridge_span": 0.0,
              "provenance": "faceted", "steep_area": 62.74012088147694,
              "support_free": false, "total_area": 2351.0559145559673}}
{"id": "on_side", "ok": true,
 "measures": {"bed_area": 550.0568787817854, "bridge_area": 0.0, "max_bridge_span": 0.0,
              "provenance": "faceted", "steep_area": 0.0,
              "support_free": true, "total_area": 2351.0559145559673}}
{"id": "teardrop_45", "ok": true,
 "measures": {"bed_area": 200.0, "bridge_area": 0.0, "max_bridge_span": 0.0,
              "provenance": "faceted", "steep_area": 7.9999978400371035,
              "support_free": false, "total_area": 2361.1779336378518}}
{"id": "teardrop_46", "ok": true,
 "measures": {"bed_area": 200.0, "bridge_area": 0.0, "max_bridge_span": 0.0,
              "provenance": "faceted", "steep_area": 0.0,
              "support_free": true, "total_area": 2361.1779336378518}}
{"id": "lintel", "ok": true,
 "measures": {"bed_area": 200.0, "bridge_area": 200.0, "max_bridge_span": 10.0,
              "provenance": "faceted", "steep_area": 0.0,
              "support_free": true, "total_area": 2560.0}}
```

**The threshold is a knife edge — never set it to a modelled face angle.** The
test is `n·up < −sin(overhang_deg)`, with the threshold evaluated in f32
against facet normals computed from the mesh's f32 vertices, so a face
modelled EXACTLY at the limit falls on whichever side the rounding lands.
`teardrop_hole`'s roof is exactly 45°: at the default `overhang_deg` its six
roof triangles compute `n·up` within 1.2e-8 of the f32 threshold
(−0.7071067690849304), two of them — 4 mm² each — landing on the steep side,
which is the whole of the `teardrop_45` reading above. The same solid reads
`"steep_area": 80.00001548959973` at `"overhang_deg": 44` (both 40 mm² flanks,
executed) and `0.0` at `46`. So `teardrop_45` is a FALSE alarm, not a sagging
roof: set the threshold to your printer's real limit and treat any reading
within a degree of a modelled face angle as unresolved.

### `clearance`
Non-asserting gap / interference between two bound solids or meshes — the
MEASURING twin of `assert_disjoint` (this op never fails a program; gate it with
`require`).

| param | type | required | meaning |
|---|---|---|---|
| `a`, `b` | ids | yes | two prior solids **or bound meshes** |
| `tol` | number | no | measurement chord tolerance in mm (default `0.01`) |

Measures: `distance` (minimum surface gap, mm), `interfering`,
`overlap_volume`, `coincident_fit_hazard`, `tol`, `source`,
`provenance: "faceted"`, and `overlap_volume_reason` whenever `overlap_volume`
is `null`.

```json
{"id": "gap", "op": "clearance", "a": "bore", "b": "pin",
 "require": {"distance": {"min": 0.20}}}
```

**What `distance` is, exactly.** It is the minimum distance between the two
TESSELLATED surfaces, computed from exact triangle–triangle feature distances.
Both operands are faceted with **inscribed** polygons, so for two nominally
coaxial round features the measured gap is SMALLER than the nominal gap by the
sagitta of both facetings: a Ø34.2 bore against a Ø33.6 spigot (0.30 mm nominal
radial gap) measures ≈ 0.227 mm at the default `tol`, converging upward as `tol`
tightens. That is not an error — it is the honest gap between the surfaces the
kernel actually holds. For a nominal-geometry number use `measure_dimension`
with `kind: "diameter"` on both features (`provenance: "analytic"`), which is a
stronger receipt than any faceted distance.

**`interfering` and `overlap_volume`.** `overlap_volume` is the volume of the
exact boolean intersection and needs two exact SOLIDS. It is `null` — with
`overlap_volume_reason` saying which of the three cases applies — when:

- `coincident_fit_hazard` is true (the operands share a flush/press-fit face
  pair, and the exact intersection across it is the known boolean-hang case);
- the exact intersection produced no measurable body for the pair;
- at least one operand is a bound mesh.

When `overlap_volume` is `null`, `interfering` degrades to `distance < 1e-6`,
which reads CONTACT as interference. Read `distance` in that case, or gate
`exact_volume` on an explicit `intersection` body — the tessellation-independent
route.

## Assertions

Measures *record*; assertions *enforce*. An `assert` op fails its program
(kind `assert_failed`, exit 1) when any declared expectation is unmet — so the
acceptance criteria live **in the program**, not in an external grep over the
report. On success the measured values are echoed as measures.

### `assert`
Declarative **topology** checks against a bound solid (or mesh). Give at least
one check (an assertion with nothing to assert is a loud `invalid_param`). All
present checks are evaluated and every failure is listed in the one error
message.

Every OTHER kind of gate — export route, watertightness, support-freeness, wall
thickness, bed fit, mass — is expressed with the universal `require` parameter on
the op that measures it (see *Gating a program with `require`*). `assert` is not
a second vocabulary for those.

On a bound **mesh**, `genus` / `shells` / `exact_volume_within` are refused
(`wrong_type`): a mesh carries no B-rep topology and no analytic surfaces, and
inventing those numbers from triangles is exactly the plausible-looking answer
this surface refuses to give. `components` / `closed` / `manifold` / `valid` /
`volume_within` are answered from the mesh itself.

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid **or a bound mesh** |
| `volume_within` | object | no | `{"target": v, "abs": a}` or `{"target": v, "percent": p}` — faceted volume must land in `target ± tolerance` (exactly one of `abs`/`percent`) |
| `exact_volume_within` | object | no | same window applied to the analytic `exact_volume` |
| `genus` | integer | no | topological genus must equal this |
| `shells` | integer | no | shell count must equal this (e.g. `2` proves a union kept two disjoint bodies) |
| `components` | integer | no | mesh connected-component count must equal this — the single-body gate (`components: 1`), measured exactly like `mesh_components`; `shells` alone cannot catch a severed part, and `components` alone cannot catch a severance narrower than `weld_tol` |
| `tol` | number | no | chord tolerance (mm) of the `components` measurement tessellation (default 0.05) |
| `weld_tol` | number | no | position-weld scale (mm) for `components` vertex identity (default 1e-3) — a severance narrower than this welds shut and reads as one body, so a hard severance proof needs `weld_tol` below the gap being ruled out |
| `closed` / `manifold` / `valid` | bool | no | the corresponding `validate()` flag must equal this |

```json
{"ops": [
  {"id": "b", "op": "box", "min": [0,0,0], "max": [20,10,5]},
  {"id": "gate", "op": "assert", "in": "b",
   "volume_within": {"target": 1000, "percent": 0.1}, "genus": 0, "shells": 1, "components": 1, "valid": true}
]}
```

### `assert_disjoint`
Prove two solids do **not** touch: passes iff their measured surface distance
**exceeds** `min_clearance` — the exit-0 proof of non-interference that an
empty `intersection` (which is an op *failure*) cannot give. Both solids are
measured on their raw exact tessellations (vertices on the true analytic
surfaces; never the voxel heal), so the distance is accurate to about `tol` —
for hard proofs keep `min_clearance` at or above `tol`, or pair this with the
exact-boolean route (`union` + `assert shells == 2`), which is
tessellation-independent.

| param | type | required | meaning |
|---|---|---|---|
| `a`, `b` | ids | yes | two prior solids |
| `min_clearance` | number | no | required gap in mm (default `0`: any positive measured gap passes) |
| `tol` | number | no | measurement chord tolerance in mm (default `0.01`) |

Measures (on pass): `distance`, `min_clearance`.

```json
{"ops": [
  {"id": "g1", "op": "spur_gear", "module": 1.25, "teeth": 12, "face_width": 12, "bore": 8},
  {"id": "g2", "op": "spur_gear", "module": 1.25, "teeth": 60, "face_width": 10, "bore": 8},
  {"id": "g2_meshed", "op": "pose", "in": "g2",
    "rotate": {"axis": [0, 0, 1], "degrees": 3}, "translate": [45.15, 0, 0]},
  {"id": "no_clash", "op": "assert_disjoint", "a": "g1", "b": "g2_meshed", "tol": 0.005}
]}
```

## Discovery & introspection

Three read-only ops that let a program ask the kernel what it can do and what
it just built, instead of guessing. They bind nothing, change no geometry and
read the topology as it already stands (no rebuild, no tessellation).

### `describe`
Self-describe the op surface from the single authoritative catalogue
(`crates/kernel-api/src/discover.rs`), which is compile-forced complete through
the `op_tag` match — it cannot drift from what actually runs. No-arg it answers
the whole catalogue; with `name` it answers that op's parameter specs
(name / type / required, plus `doc` where the spec carries one — the
machine-readable half of the tables in this document), and `exists: false` for
anything else (the basis of did-you-mean).

| param | type | required | meaning |
|---|---|---|---|
| `name` | string | no | one op to describe; omit for the whole catalogue |

Measures: no-arg `count`, `ops` (every name), `params_available`; with `name`,
`name` + `exists`, plus `params`: `[{name, type, required, doc}]` for a real
op (`doc` is `""` where the spec carries no prose).

Executed — the no-arg form answers `"count": 160`, `"params_available": true`
and the 160-name `ops` array (`"box"`, `"cylinder"`, … `"thread_ridge"`,
`"export_threaded"`); the per-op form is the authoritative parameter table:

```json
{"ops": [
  {"id": "all", "op": "describe"},
  {"id": "spec", "op": "describe", "name": "support_report"},
  {"id": "typo", "op": "describe", "name": "support_reprot"}
]}
```

```json
{"id": "spec", "ok": true,
 "measures": {"exists": true, "name": "support_report",
              "params": [{"doc": "", "name": "in", "required": true, "type": "id-ref"},
                         {"doc": "", "name": "build_dir", "required": false, "type": "[x,y,z]"},
                         {"doc": "", "name": "overhang_deg", "required": false, "type": "number"}]}}
{"id": "typo", "ok": true, "measures": {"exists": false, "name": "support_reprot"}}
```

### `list_faces`
Enumerate a solid's FACES as references (the M4 loop): `count` plus
`faces: [{index, type, descriptor, witness, area}]`. `type` is
`plane`/`cylinder`/`sphere`/`cone`/`torus`; `descriptor` carries the exact
analytic surface (plane normal + point, cylinder axis + radius, …); `witness`
is the face polygon's centroid — the anchor `measure_dimension`'s `face_face`
and `diameter` selections and the `asm_mate_face` / `asm_mate_axis` witnesses
resolve against (for an EDGE witness — fillets, chamfers — use `list_edges`).

Honest about `area`: it is the area of the polygon the kernel actually holds
(so a coarsely faceted end cap reports the FACET polygon — 18 mm² per cap on
the 4-segment Ø6 cylinder below, not the analytic π·3² ≈ 28.27), and it is
`null` for every curved face. The `descriptor` stays exact either way
(`"radius": 3.0`), which is why `exact_volume` and STEP export read it rather
than the polygon.

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid |

```json
{"ops": [
  {"id": "pin", "op": "cylinder", "base": [0, 0, 0], "axis": [0, 0, 1], "radius": 3, "height": 10, "segments": 4},
  {"id": "faces", "op": "list_faces", "in": "pin"}
]}
```

```json
{"id": "faces", "ok": true, "measures": {"count": 6, "faces": [
  {"area": 18.0, "descriptor": {"normal": [-0.0, -0.0, -1.0], "point": [0.0, 0.0, 0.0]},
   "index": 0, "type": "plane", "witness": [-1.1102230246251565e-16, 1.1102230246251565e-16, 0.0]},
  {"area": 18.0, "descriptor": {"normal": [0.0, 0.0, 1.0], "point": [0.0, 0.0, 10.0]},
   "index": 1, "type": "plane", "witness": [-1.3777276490407724e-16, 1.1102230246251565e-16, 10.0]},
  {"area": null, "descriptor": {"axis": [0.0, 0.0, 1.0], "point": [0.0, 0.0, 0.0], "radius": 3.0},
   "index": 2, "type": "cylinder", "witness": [1.5, 1.5, 5.0]}]}}
```

(Indices 3–5 repeat that same cylinder descriptor at the other three wall
witnesses — `[-1.5, 1.5000000000000002, 5.0]`, `[-1.5000000000000002, -1.5,
5.0]`, `[1.4999999999999998, -1.5, 5.0]`.)

### `list_edges`
Enumerate a solid's EDGES as references: `count` plus
`edges: [{index, midpoint, length, curved}]`. `midpoint` is the witness you
hand straight back to `fillet_edge_near` / `chamfer_edge_near`; `midpoint` and
`length` are the exact chord for a straight edge (`curved: false`) and an
approximation for a curved one (`curved: true` — the chord of the underlying
edge curve, not its arc length).

Honest about `count`: it is the topology the kernel HOLDS, not the edges you
would draw by hand. On a post-boolean solid the arrangement's own vertices and
seams are in the list (see the bored plate below), and after a chain of
booleans coplanar fragmentation inflates it further — the finishing pass for
that is `kernel_brep::coalesce_coplanar` on the Rust surface, not this op.

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid |

Executed — the 12 edges of a plate, one of whose midpoints is fed straight back
as a fillet witness (`"witness_distance": 0.0` — an exact hit, no near-miss),
and the same plate after a Ø6 through-bore at 8 segments:

```json
{"ops": [
  {"id": "plate", "op": "box", "min": [0, 0, 0], "max": [30, 20, 10]},
  {"id": "edges", "op": "list_edges", "in": "plate"},
  {"id": "soft", "op": "fillet_edge_near", "in": "plate", "witness": [15, 20, 10], "radius": 2},
  {"id": "bore", "op": "drill", "in": "plate", "at": [15, 10, 10], "axis": [0, 0, -1], "d": 6, "through": 10, "segments": 8},
  {"id": "bored_edges", "op": "list_edges", "in": "bore"}
]}
```

```json
{"id": "edges", "ok": true, "measures": {"count": 12, "edges": [
  {"curved": false, "index": 0, "length": 30.0, "midpoint": [15.0, 20.0, 0.0]},
  {"curved": false, "index": 1, "length": 20.0, "midpoint": [30.0, 10.0, 0.0]},
  {"curved": false, "index": 2, "length": 30.0, "midpoint": [15.0, 0.0, 0.0]},
  {"curved": false, "index": 3, "length": 20.0, "midpoint": [0.0, 10.0, 0.0]},
  {"curved": false, "index": 4, "length": 30.0, "midpoint": [15.0, 0.0, 10.0]},
  {"curved": false, "index": 5, "length": 20.0, "midpoint": [30.0, 10.0, 10.0]},
  {"curved": false, "index": 6, "length": 30.0, "midpoint": [15.0, 20.0, 10.0]},
  {"curved": false, "index": 7, "length": 20.0, "midpoint": [0.0, 10.0, 10.0]},
  {"curved": false, "index": 8, "length": 10.0, "midpoint": [30.0, 0.0, 5.0]},
  {"curved": false, "index": 9, "length": 10.0, "midpoint": [0.0, 0.0, 5.0]},
  {"curved": false, "index": 10, "length": 10.0, "midpoint": [30.0, 20.0, 5.0]},
  {"curved": false, "index": 11, "length": 10.0, "midpoint": [0.0, 20.0, 5.0]}]}}
{"id": "soft", "ok": true,
 "measures": {"resolved_edge": {"faces": [{"operand": "Primitive", "source_face": 1},
                                          {"operand": "Primitive", "source_face": 3}],
                                "max_distance": 3.7416573867739418, "witness_distance": 0.0}}}
{"id": "bored_edges", "ok": true, "measures": {"count": 44, "edges": [ … 44 entries, 12 of them "curved": true … ]}}
```

The bored plate's `44` is the honest topology, not the 12 + 16 + 8 = 36 you
would draw: on each annular face the arrangement also laid **two diagonal seam
edges** (plate corner → bore ring, 15.202404483724985 mm each) and split two of
the eight 2.2961 mm ring chords into 0.5997503725091069 + 1.6963502216814212
halves at their feet — which is why only 12 of the 16 ring chords carry
`curved: true`. The rest is what you expect: 12 straight verticals (4 plate
pillars + 8 bore-wall edges) and the 4 + 4 outer face edges.

## Exports

File paths are joined onto the CLI's `--out-dir` and **confined** to it:
absolute paths and any `..` component are refused with `invalid_param` (the
sandbox rule — a program can only write under its output directory). Parent
directories are created. The report's `file` field is the path actually
written.

STL/3MF routing is **honest**: the solid is tessellated on the exact adaptive
path (chord tolerance `tol`); if that mesh is watertight it ships
(`"route": "exact"`), otherwise the solid is healed through the voxel half
(winding-number SDF → manifold re-mesh at `voxel` mm,
`"route": "voxel_healed"`). A mesh that stays leaky even after healing fails
with `invalid_geometry` rather than exporting garbage.

### `export_stl`

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid |
| `file` | string | yes | output path (binary STL) |
| `tol` | number | no (0.01) | chord tolerance in mm |
| `voxel` | number | no (0.3) | heal-fallback voxel size in mm |

Measures: `route`, `triangles`, `watertight`, `watertight_means`,
`boundary_edges`, `non_orientable_edges`, `two_manifold`. The op also **binds
the mesh it wrote**, so the print file itself can be gated (`mesh_components`,
`support_report`, `bounding_box`, `validate`, `require`) rather than the solid
that stands in for it — on the `voxel_healed` route those are two different
surfaces.

**What `watertight` means here.** It is EDGE CLOSURE: every undirected edge is
used by exactly two triangles. That is the property the op refuses without, and
the one a slicer needs to fill a solid. It is *not* the rigorous
closed-orientable-2-manifold property, and on this kernel the difference is
real: a boolean result routinely carries a few triangles wound inside-out, which
close their edges perfectly and leave the file non-orientable. Those show up as
`non_orientable_edges > 0` with `boundary_edges: 0` and `two_manifold: false`,
and they cannot be seen from `watertight` alone — which is why all four are
reported. Gate `watertight` for printability; gate `two_manifold` as well if a
downstream tool needs consistent normals.

```json
{"ops": [
  {"id": "b", "op": "box", "min": [0,0,0], "max": [20,10,5]},
  {"id": "out", "op": "export_stl", "in": "b", "file": "part.stl", "tol": 0.01,
   "require": {"watertight": true, "route": "exact", "boundary_edges": 0}}
]}
```

### `export_step`
STEP AP203 with **exact analytic surfaces** (plane / cylindrical / spherical /
conical / toroidal entities, circular edges as CIRCLE) — not a mesh dump. The
product name is the file stem.

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid |
| `file` | string | yes | output path |

```json
{"ops": [
  {"id": "pin", "op": "cylinder", "base": [0,0,0], "axis": [0,0,1], "radius": 3, "height": 10},
  {"id": "out", "op": "export_step", "in": "pin", "file": "pin.step"}
]}
```

### `export_3mf`
Same tessellation/heal routing as `export_stl`, written as 3MF.

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid |
| `file` | string | yes | output path |
| `tol` | number | no (0.01) | chord tolerance in mm |
| `voxel` | number | no (0.3) | heal-fallback voxel size in mm |

```json
{"ops": [
  {"id": "b", "op": "box", "min": [0,0,0], "max": [20,10,5]},
  {"id": "out", "op": "export_3mf", "in": "b", "file": "part.3mf"}
]}
```

## Imports

The reading half of interchange. Input paths resolve like `load_part` (against
the program file's own directory through the CLI, against `out_dir` through
the bare `run_program`) and are **confined** to that base — absolute paths and
any `..` component are refused with `invalid_param`, same sandbox rule as
outputs.

### `import_step`
Import a STEP physical file through the kernel's analytic importer and **bind
the reconstructed exact B-rep** as a solid — cut it, measure it, re-export it
like any other. Faces keep their analytic surface tags (plane / cylinder /
sphere / cone / torus); trimmed-NURBS faces enter as their chord facets and
are counted in `freeform_faces`. A **multi-solid file merges into ONE
multi-shell solid** (each `MANIFOLD_SOLID_BREP` keeps its own shell — the
`shells` measure is the honest count; a measured export→import round trip on
the in-tree corpus conserves volume to ~2 × 10⁻¹⁰ relative).

Failure mapping is verbatim from the kernel: parse / dangling-reference /
unsupported-construct problems are `invalid_param` with the importer's exact
reason; faces that do not form a usable solid are `invalid_geometry`.

| param | type | required | meaning |
|---|---|---|---|
| `file` | string | yes | path to a STEP file (confined to the input base) |
| `mode` | string | no | `"strict"` (default) or `"tolerant"` — see below |

Measures (both modes): `source` (`"step"`), `shells`, `genus`, `faces`,
`volume` (faceted), `freeform_faces`.

```json
{"ops": [
  {"id": "housing", "op": "import_step", "file": "housing.step"},
  {"id": "check", "op": "assert", "in": "housing", "valid": true},
  {"id": "out", "op": "export_stl", "in": "housing", "file": "housing.stl"}
]}
```

Every trim vertex of a B-spline face is projected onto its patch within the
file's own asserted `UNCERTAINTY_MEASURE_WITH_UNIT` (typically 4–50 µm from
Creo/SolidWorks/Onshape) — a vertex the producer itself declares "on" the
patch is accepted, not refused. Holes on curved analytic faces (a bore
through a cylinder wall, a pocket in a spherical dome) and periodic
sphere/torus regions the ring-grid resampler cannot phase (a corner ball
bounded by three arcs, a torus band whose rims start at different
longitudes, a half-torus wall) are tessellated on their exact surface through
the parameter-patch path.

#### `"mode": "tolerant"` — real vendor files

Strict mode refuses the whole file on the first face it cannot read, and
imports every `MANIFOLD_SOLID_BREP` in its LOCAL frame (assembly placements
are ignored). Tolerant mode is the read path for a vendor assembly (a
mainboard, a battery pack) where one odd face must not cost the other 167
solids and where the campaign needs the parts' *placed* envelopes:

- **per-face containment**: a face the exact routes refuse is rolled back and
  **flat-repaired** (its loops ear-clipped on their own Newell plane — the
  boundary chords stay verbatim, so the shell stays welded and closed; only
  that face's interior geometry is approximated). A face that cannot even be
  repaired is **skipped**, which skips its whole solid (an open shell can
  never bind);
- **per-solid census with placements**: EVERY solid of the file is listed,
  one record per assembly **instance** (the NAUO tree is walked with its
  `ITEM_DEFINED_TRANSFORMATION` placements — the count OpenCascade's XCAF
  reader gives), named by its `PRODUCT`, with a world-space envelope from the
  entity geometry (vertices, exact conic-arc extremes, B-spline control
  points) — so a solid whose B-rep could not be built still reports its
  envelope; an imported solid's envelope also folds in its reconstructed
  vertices;
- **a looser trim-vertex snap**: 10× the file's uncertainty (strict: 1×);
  every vertex accepted beyond the uncertainty is reported.

The bound body is the **compound** of every imported instance, placed
(mirrored placements rebuild the instance outward-wound), a valid multi-shell
solid. If NO solid imports the op fails `invalid_geometry` with the counts and
the first skip reasons in the message.

Additional measures in tolerant mode:

| measure | meaning |
|---|---|
| `mode` | `"tolerant"` |
| `uncertainty_mm` | the file's asserted uncertainty (`null` when it states none) |
| `solids_total` / `solids_imported` / `solids_skipped` | instance counts |
| `faces_skipped` / `faces_repaired` | face-level counts across all breps |
| `solids` | one record per instance: `name` (PRODUCT name), `path` (product names root → instance, `/`-joined), `entity` (the brep id), `status` (`"imported"` \| `"skipped"`), `bbox_min` / `bbox_max` (placed, mm), `bbox_source` (`"brep"`: reconstructed vertices folded in; `"edges"`: entity geometry only), `faces`, `faces_repaired`, `faces_skipped`, `reason` (skipped solids only, verbatim) |
| `skipped` | `[{entity, kind, solid, reason}]` — faces (`ADVANCED_FACE`) that could not be read even flat, and solids (`MANIFOLD_SOLID_BREP` / `BREP_WITH_VOIDS`) that were skipped as a consequence or failed validation |
| `repaired` | `[{entity, kind, solid, reason}]` — flat-repaired faces (reason ends `…repaired: <surface type> face approximated by N flat facets…`), trim vertices projected onto their patch beyond the uncertainty, unparseable statements skipped (`kind: "statement"`), an unreadable assembly structure (`kind: "assembly"`) |

```json
{"ops": [
  {"id": "board", "op": "import_step", "file": "vendor/mainboard.step", "mode": "tolerant"},
  {"id": "env", "op": "bounding_box", "in": "board"}
]}
```

Read `solids[*].bbox_*` for per-part envelopes (a keep-out for a case), and
gate on `solids_skipped` / `faces_repaired` when the exact geometry matters:
a flat-repaired face is inside the body's envelope but not its true surface.

### `import_mesh`
Import a triangle-mesh file — `.stl`, `.obj`, `.3mf` or `.ply`, sniffed by
extension (the kernel has **no glTF reader**) — weld it (STL and many
exporters store an unshared soup), and report the full `check_mesh` receipt.
**Binds nothing**: meshes never enter the solid environment (it holds exact
solids and sketches only); to cut with an imported mesh, hand its FILE to
`mesh_carve`. `volume` is reported **only when the welded mesh is watertight**
— a leaky mesh has no defined enclosed volume, so the key is omitted rather
than guessed.

With `"heal": true` the kernel's deterministic import repair runs first:
boundary loops are capped (`fill_holes`) and non-manifold junctions split
(`make_manifold`). If the mesh is STILL leaky afterwards the op fails
`invalid_geometry` (route it through `mesh_carve`, which re-meshes watertight
through the voxel half, or repair upstream).

| param | type | required | meaning |
|---|---|---|---|
| `file` | string | yes | mesh path (`.stl` / `.obj` / `.3mf` / `.ply`) |
| `heal` | bool | no (false) | cap holes + split non-manifold junctions before the receipt |
| `out` | string | no | re-write the welded/healed mesh (`.stl` / `.3mf` by extension) |

Measures: `format`, `triangles`, `healed`, the full receipt (`watertight`,
`boundary_edges`, `non_manifold_edges`, `non_orientable_edges`,
`non_manifold_vertices`, `degenerate_triangles`, `self_intersections`),
`bbox_min` / `bbox_max`, and `volume` iff watertight.

```json
{"ops": [
  {"id": "scan", "op": "import_mesh", "file": "scan.stl", "heal": true, "out": "scan_healed.3mf"}
]}
```

### `mesh_carve`
Boolean a bound solid against a **mesh file** through the winding-number voxel
boolean: the solid is meshed on the honest exact-else-heal route, the file is
read and welded, both are lifted into winding-number SDFs and the result is
re-meshed by dual contouring. The output is **guaranteed a closed 2-manifold**,
but the seam is **voxel-resampled** — accurate to `voxel`, never exact — hence
route `"voxel_implicit"`, always stated in the measures. This is the bridge
that lets a scanned/downloaded mesh cut (or join) exact geometry. Writes
`out`; binds nothing. An empty result (e.g. an intersection of disjoint
parts) fails `invalid_geometry` instead of writing an empty file.

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid (the `a` operand) |
| `file` | string | yes | the mesh operand (`.stl` / `.obj` / `.3mf` / `.ply`) |
| `bool` | string | yes | `"union"` / `"difference"` / `"intersection"` |
| `voxel` | number | no (0.3) | resampling lattice size in mm |
| `out` | string | yes | output mesh path (`.stl` / `.3mf` by extension) |

Measures: `route` (`"voxel_implicit"`), `triangles`, `watertight`, `volume`,
`voxel`.

```json
{"ops": [
  {"id": "stock", "op": "box", "min": [0,0,0], "max": [60,40,20]},
  {"id": "carved", "op": "mesh_carve", "in": "stock", "file": "relief.stl",
   "bool": "difference", "voxel": 0.2, "out": "plaque.stl"}
]}
```

## Implicit / hybrid

**The one-way street, stated as a contract:** the two halves convert freely in
ONE direction. Any exact solid (or mesh) can *enter* the implicit world
losslessly enough to compute with — the winding-number bridge (`MeshSdf`) lifts
it to a field, and `hybrid_boolean` can even keep the untouched exact faces
verbatim. Coming *back* there is now exactly ONE door, and it is honest about
what it is: `solid_from_implicit` (reverse bridge v1, below) wraps a field-born
mesh into a **faceted** B-rep `Solid` — one planar face per surviving triangle,
volume-conservation gated, route `"voxel"` — so a lattice can re-enter exact
planar booleans and `export_step`. What NEVER happens is analytic recovery: a
voxelized cylinder comes back as N planar facets, not a `Surface::Cylinder`
(field → analytic boundary reconstruction is the industry-wide unsolved
problem; recovery is the ledgered v2), so curvature-reading ops downstream of
the bridge (fillet/chamfer witnesses, `measure_dimension diameter`, hole
seats on curved walls) have no analytic tags to read. Every mesh-side result
still carries route `"voxel_implicit"` / `"voxel_healed"`, every bridged solid
route `"voxel"` — the crossing is NAMED in every receipt rather than blurred.
Plan programs accordingly: do exact work first, cross to the implicit half
late, and bridge back only what genuinely needs solid-side machinery.

### `gyroid_block`
A gyroid TPMS lattice cube (half-extent `half` at `center`), built on the
implicit half of the kernel and meshed **watertight** by Manifold Dual
Contouring at `voxel` resolution, then written as binary STL. If the mesh is
not watertight/manifold it is healed once (`make_manifold`); if it STILL is
not, the op fails with `invalid_geometry` (try a smaller `voxel` or thicker
wall). Binds no value — the artifact is the file.

Field notes (the gyroid is an implicit field, only roughly metric): `scale` is
the angular frequency in rad/mm — the cell period is `2π / scale` (0.35 ≈
18 mm cells); `thickness` is the wall half-thickness parameter in mm.

| param | type | required | meaning |
|---|---|---|---|
| `center` | `[x,y,z]` | yes | block center |
| `half` | number | yes | cube half-extent (mm) |
| `scale` | number | yes | gyroid frequency (rad/mm) |
| `thickness` | number | yes | wall half-thickness (mm) |
| `voxel` | number | yes | dual-contour voxel size (mm) — wall should span ≥ ~3 voxels |
| `file` | string | yes | output STL path |

Measures: `triangles`, `watertight`, `healed`.

```json
{"ops": [{"id": "lattice", "op": "gyroid_block", "center": [0, 0, 0], "half": 20,
          "scale": 0.35, "thickness": 0.6, "voxel": 0.15, "file": "gyroid.stl"}]}
```

### `tpms`
A bounded TPMS lattice block in any of the **six families** — gyroid,
Schwarz-P, diamond, Neovius, Schoen I-WP, Fischer-Koch S — as a NAMED op, so
lattices are discoverable in the catalogue instead of hidden inside the
`implicit` tree. It is the exact named-op twin of the tree's `tpms` leaf: the
parameters go through the SAME parser and the field is wrapped
`primitive_bound` per the FieldQuality contract (a downstream offset/shell is
honestly flagged approximate). `mode: "network"` (default) gives the open
double-labyrinth (`level` is the iso-level; 0 ≈ 50% solid, negative thins);
`mode: "sheet"` gives the wall lattice (`level` is the wall half-thickness in
mm, > 0, required). The block is **closed by construction**: a raw TPMS is an
open labyrinth (the region box cuts its tubes), so the field is clamped by the
`min`/`max` box — walls cap at the boundary. Meshed watertight by Manifold Dual
Contouring at `voxel` (healed once if needed; still-leaky fails
`invalid_geometry`). Binds no
value — the artifact is the file; the measures carry route
`"voxel_implicit"`.

| param | type | required | meaning |
|---|---|---|---|
| `kind` | string | yes | `gyroid` / `schwarz_p` / `diamond` / `neovius` / `schoen_iwp` / `fischer_koch_s` |
| `min` | `[x,y,z]` | yes | lattice block corner (mm) |
| `max` | `[x,y,z]` | yes | opposite corner (mm) |
| `cell` | number | yes | unit-cell edge length (mm) |
| `mode` | string | no (`"network"`) | `"network"` or `"sheet"` |
| `level` | number | network: no (0) · sheet: yes | network iso-level / sheet wall half-thickness (mm) |
| `voxel` | number | no (0.3) | extraction voxel size (mm) — walls should span ≥ ~3 voxels |
| `file` | string | yes | output mesh path (`.stl` / `.3mf` by extension) |

Measures: `route` (`"voxel_implicit"`), `kind`, `mode`, `triangles`,
`watertight`, `healed`, `volume` (mm³), `voxel`.

```json
{"ops": [{"id": "core", "op": "tpms", "kind": "schoen_iwp", "min": [0, 0, 0],
          "max": [40, 40, 20], "cell": 8, "mode": "sheet", "level": 0.5,
          "voxel": 0.25, "file": "iwp_core.stl"}]}
```

### `hybrid_boolean`
**The flagship convergence op** (docs/BAR.md Level 9, "true convergence"):
boolean a bound **exact B-rep** solid against a **non-B-rep operand** — an
implicit CSG tree (`field`, the same grammar as `implicit`) or a mesh file
(`file`) — in one call, without hand-building representation twins. The exact
side **stays exact wherever the operand does not touch it**:

- **Exact route** (`route: "exact_stitch"`): untouched input faces appear in
  the result **verbatim** — identical loops and vertices, analytic surface
  tags (cylinder/sphere/cone/torus) intact — and the seam is exact against
  the operand's facets. The per-face receipts prove it, **measured on the
  result**: every input face lands in exactly one of `kept_exact` (verbatim;
  `kept_exact_curved` counts those with curved analytic tags), `retiled`
  (full area survives, face subdivision changed), `trimmed` (genuinely cut
  back at the seam), `consumed` (swallowed by the operand).
- **Healed route** (`route: "voxel_healed"` + `healed_reason`): when the
  exact stitch cannot be trusted (the measured reason says why — e.g. a
  still-pinched lattice operand or an over-budget operand mesh), the result
  is re-meshed through the winding-number voxel twin — watertight,
  voxel-approximate everywhere, and the first three face counts are 0
  because nothing survives resampling. Nothing degrades silently.

The field operand is meshed at `voxel` before the boolean (it must have
finite bounds — clamp an unbounded TPMS/plane by intersecting with a box
node) and Manifold DC's documented pinch case is snipped and re-verified at
intake. Either route's result is **verified watertight and 2-manifold**, or
the op fails `invalid_geometry` with the measured edge counts instead of
writing a degraded body. Writes `out`; binds nothing (meshes never enter the
solid environment).

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | the exact B-rep operand (a bound solid) |
| `field` | tree | one of | implicit operand — nestable CSG tree with finite bounds (exclusive with `file`) |
| `file` | string | one of | mesh-file operand (`.stl` / `.obj` / `.3mf` / `.ply`; exclusive with `field`) |
| `bool` | string | yes | `"union"` / `"difference"` / `"intersection"` |
| `voxel` | number | no (0.3) | field meshing + healed-fallback lattice size (mm) |
| `out` | string | yes | output mesh path (`.stl` / `.3mf` by extension) |

Measures: `route` (`"exact_stitch"` / `"voxel_healed"`), `healed_reason`
(healed route only), `operand` (`"implicit_field"` / `"mesh_file"`),
`brep_faces`, `kept_exact`, `kept_exact_curved`, `retiled`, `trimmed`,
`consumed`, `operand_triangles`, `triangles`, `watertight`, `volume`, `voxel`.

```json
{"ops": [
  {"id": "bracket", "op": "cylinder", "base": [0,0,0], "axis": [0,0,1], "radius": 20, "height": 12},
  {"id": "latticed", "op": "hybrid_boolean", "in": "bracket", "bool": "intersection",
   "field": {"shape": "tpms", "kind": "gyroid", "min": [-20,-20,0], "max": [20,20,12],
             "cell": 6, "mode": "sheet", "level": 0.6},
   "voxel": 0.25, "out": "latticed_bracket.stl"}
]}
```

### `implicit` — nestable expression trees (runtime composability)

The implicit half's full CSG `Node` algebra as **data** (docs/BAR.md I6): `expr` is
a RECURSIVE JSON tree of leaf shapes and combinators — including user-authored
scalar fields (`expr_sdf`, `offset_by`, `lerp`) written in a small math
expression language the kernel evaluates itself. A deployed AI composes new
geometry kinds (threads, graded lattices, blends) without anyone writing Rust.

The tree is extracted watertight at `voxel` resolution. A mesh that is not
watertight/manifold is healed once (`make_manifold`); if it STILL is not, the
op fails with `invalid_geometry` (refine the voxel — thin walls need ≥ ~3
voxels — or switch mesher). Binds no value; the measures carry `triangles`,
`watertight`, `healed`, `volume` (mm³), and `file` is written when requested.

| param | type | required | meaning |
|---|---|---|---|
| `expr` | tree | yes | the recursive expression tree (grammar below) |
| `voxel` | number | yes | extraction voxel size (mm) |
| `mesher` | string | no (`"narrowband"`) | `"narrowband"` (fast, surface-area-scaled, REQUIRES ≤ 1-Lipschitz fields) or `"manifold"` (dense, no Lipschitz assumption, resolves lattice/TPMS pinch saddles) |
| `domain` | `{min, max}` | no | explicit meshing box; default: the tree's bounds padded by 3·voxel |
| `file` | string | no | output path — the extension picks the format (`.stl` / `.3mf`) |

Parse errors are `invalid_param` and carry the **JSON path to the bad
subtree** (e.g. `op 'bolt': at expr.b.a: unknown shape 'sphre' — …`).

#### Tree grammar

A tree node is either a **leaf** `{"shape": "...", ...}` or a **combinator**
`{"op": "...", ...}` whose children (`a`, `b`, `in`) are themselves tree
nodes. Leaves (all lengths mm; every parameter is validated loudly):

| shape | params | notes |
|---|---|---|
| `sphere` | `center`, `radius` | |
| `box` | `min`, `max` | axis-aligned corners |
| `cylinder` | `a`, `b`, `radius` | capped, endpoint form |
| `cone` | `a`, `b`, `ra`, `rb` | capped frustum; `rb: 0` is a sharp cone, `ra == rb` a cylinder |
| `capsule` | `a`, `b`, `radius` | segment swept by a sphere |
| `torus` | `center`, `axis`, `major`, `minor` | |
| `plane` | `point`, `normal` | a half-space (inside = below the normal); UNBOUNDED — intersect it with a bounded shape or pass `domain` |
| `gyroid` | `min`, `max`, `scale`, `thickness` | TPMS shell bounded to the box; `scale` rad/mm (cell period 2π/scale), `thickness` = wall half-thickness |
| `tpms` | `kind`, `min`, `max`, `cell`, `mode`, `level` | TPMS lattice in six families — `kind`: `gyroid`/`schwarz_p`/`diamond`/`neovius`/`schoen_iwp`/`fischer_koch_s`. `mode` `"network"` (solid labyrinth; `level` iso, 0 ≈ 50%, default) or `"sheet"` (thickened wall; `level` = half-thickness > 0). `cell` = unit-cell period (mm). A distance BOUND ⇒ downstream `offset`/`shell` is flagged approximate |
| `beam_lattice` | `nodes` + `struts` — or `min`, `max`, `cell` (`"cubic"`/`"octet"`), `cell_size`, `radius` | graph form: `struts` are `[node_a, node_b, radius_a, radius_b]` (tapering allowed); cell form fills the box. Junction-rich ⇒ use `"mesher": "manifold"` |
| `strut_lattice` | `kind`, `cell`, `radius`, `min`, `max` | **triply-periodic** strut lattice — `kind`: `bcc` (compliant/springy) / `fcc` / `octet` (stretch-dominated workhorse); `cell` = period (mm), uniform strut `radius`. The FIELD is periodic over all of space; `min`/`max` is only the bounds hint (exactly like `tpms`) — **intersect with a box/shroud to close it**, or the meshing domain cuts the struts open. A distance BOUND (min-union) ⇒ downstream `offset`/`shell` flagged approximate. Junction-rich ⇒ `"mesher": "manifold"` |
| `pipe` | `path`, `radius` or `radii` | tube along a polyline; `radii` (one per point) tapers the wall |
| `pipe_path` | `points`, `radius` | uniform-radius capsule chain along a polyline (the strut vocabulary's skeleton/pipe convenience — same field as `pipe` with a constant radius) |
| `helix_pipe` | `center`, `axis`, `r_helix`, `pitch`, `turns`, `radius`, `samples_per_turn` (64) | circular helix tube — cooling channels, springs |
| `text` | `text`, `height`, `stroke_radius` | single-stroke **Hershey Simplex** text as capsule strokes in the `z = 0` plane (tube straddles ±`stroke_radius` in z): baseline on `y = 0`, glyphs advance +X from the origin, capitals scaled to `height`. Union onto a face = embossed bead, difference = engraved groove. Charset: `A`–`Z` (lowercase folds up), `0`–`9`, space, `-`, `.` — anything else is a loud `invalid_param` naming the character (validated BEFORE the kernel; ≤ 256 chars). Exactly 1-Lipschitz, distance BOUND |
| `expr_sdf` | `expr`, `lipschitz_bound`, optional `min` + `max` | a user scalar field as an SDF leaf — see the contract below |

Combinators (children `a`/`b`, or `in` for single-child ops; angles degrees):

| op | params | meaning |
|---|---|---|
| `union` / `intersection` / `difference` | `a`, `b` | hard booleans (`min`/`max` on distance — never fail) |
| `smooth_union` / `smooth_intersection` / `smooth_difference` | `a`, `b`, `k` | polynomial blend of radius-ish `k` (a *blob*: blend size tracks field magnitude) |
| `fillet_union` / `fillet_difference` | `a`, `b`, `r` | TRUE constant-radius quarter-round on the seam |
| `chamfer_union` / `chamfer_difference` | `a`, `b`, `r` | 45° flat bevel on the seam |
| `displace` | `in`, `amplitude`, `texture` | **procedural surface texture**: `d′ = (d − amplitude·t(p)) / L′` with texture field `t ∈ [0, 1]` — positive `amplitude` raises it proud (grip), negative recesses. `texture.kind`: `knurl` (`pitch`, `depth_frac` [0..1] def 1 — crossed ±45° ridges, peak-to-valley `amplitude·depth_frac/2`), `stipple` (`cell`, `coverage` [0..1] def 0.5 — hashed raised dots), `noise` (`cell`, `seed` def 0 — deterministic trilinear value-noise). The kernel divides by the derived bound `L′ = 1 + |amplitude|·L_texture`, so the **zero set (your geometry) is preserved and the field stays ≤ 1-Lipschitz** for a ≤ 1-Lipschitz child — narrow-band pruning stays sound. Grazing ridge saddles can still pinch the narrow-band extraction on curved bodies — the refusal names it; use `"mesher": "manifold"` then. Distance BOUND |
| `offset` | `in`, `t` | inflate (`t > 0`) / deflate (`t < 0`) the surface |
| `shell` | `in`, `t` | hollow shell of total wall `2·t` around the surface |
| `translate` | `in`, `offset` | |
| `rotate` | `in`, `axis`, `degrees`, optional `center` (origin) | rotation about the axis line through `center` |
| `scale` | `in`, `factor` | uniform, about the origin |
| `mirror` | `in`, `point`, `normal` | the child ∪ its reflection across the plane |
| `linear_pattern` | `in`, `step`, `count` | copy *i* at `i·step` (count ≤ 4096) |
| `circular_pattern` | `in`, `center`, `axis`, `count`, optional `step_degrees` (360/count) | polar pattern |
| `offset_by` | `in`, `field`, `max_abs` | **graded offset**: the surface moves outward by `field(p)` mm (clamped to ±`max_abs`) — graded wall thickness / lattice inflation. `field` is a scalar expression **or** a `{"grid": …}` NPY source (below) |
| `lerp` | `a`, `b`, `field` | pointwise blend, weight = `clamp(field, 0, 1)` — solid-to-lattice transitions. Same two `field` forms |

#### The scalar expression language (`expr_sdf.expr`, `field`)

A scalar expression is a JSON **number** (constant), one of the **variables**
`"x"` / `"y"` / `"z"` (the query point, mm), or `{"op": ...}`:

| op | params | meaning |
|---|---|---|
| `add`, `sub`, `mul`, `div`, `min`, `max` | `a`, `b` | arithmetic |
| `mod` | `a`, `b` | Euclidean remainder (`rem_euclid`: result in `[0, b)`) — the periodic-pattern / helical-unwrap workhorse |
| `atan2` | `y`, `x` | angle of the point `(x, y)`, radians — note the named, order-proof params |
| `neg`, `abs`, `sqrt`, `sin`, `cos` | `arg` | |
| `clamp` | `value`, `lo`, `hi` | `min(max(value, lo), hi)` — never errors, even for `lo > hi` |
| `length2` | `a`, `b` | `√(a² + b²)` — e.g. the cylindrical radius `length2(x, y)` |
| `length3` | `a`, `b`, `c` | `√(a² + b² + c²)` |

**The `expr_sdf` Lipschitz contract (honest, load-bearing).** The narrow-band
mesher prunes blocks assuming a sampled value never overstates the distance to
the surface — guaranteed by fields with `|∇| ≤ 1`. An arbitrary expression
cannot be auto-normalized, so `lipschitz_bound` is REQUIRED: declare a truthful
bound `L ≥ sup|∇expr|` and the kernel evaluates `expr / L` — the **zero set
(your geometry) is unchanged**, the slope is normalized. Over-declaring is
safe; **under-declaring can prune away real surface** (holes). If no bound is
practical, use `"mesher": "manifold"` (samples every cell, needs continuity
only). `min`/`max` bounds declare where the surface lives (they feed the
automatic domain); omit them only under an intersection with a bounded shape
or an explicit `domain`.

**Degeneracy guard.** Every scalar expression is probed on a 5×5×5 lattice
over the meshing domain BEFORE extraction; a NaN/∞ value (division pole, √ of
a negative, `mod 0`) fails loudly with the expression's JSON path and the
probe point. The probe is a heuristic tripwire, not a proof — keep your
denominators clamped.

`offset_by`/`lerp` field caveat (same as the Rust API): the modulated result
is only `(1 + |∇field|)`-Lipschitz, so keep graded fields slowly varying
(a few % per mm) for the narrow-band mesher, or extract with `"manifold"`.

#### Simulation fields as grade sources — `{"grid": …}`

The `field` of `offset_by` / `lerp` alternatively takes a **sampled grid**
instead of a math expression — the simulation→geometry bridge
(`kernel_implicit::grid_field::GridField`): an FEA stress field remapped to a
density (`tools/stress_to_density.py`), a `sample_density_grid` output, any
nodal scalar — loaded from a NumPy `.npy` file and evaluated by trilinear
interpolation, **border-clamped** (outside the grid the nearest border value
extends, so the law is total and continuous wherever the mesher samples):

```json
"field": {"grid": {"path": "stress_density.npy", "origin": [0, 0, 0],
                   "cell": 20, "normalize": [0, 1], "law": [-0.6, 0.9]}}
```

| key | required | meaning |
|---|---|---|
| `path` | yes | the `.npy` file — `<f4`/`<f8`, C-order, shape exactly `(nx, ny, nz)`. Resolves confined under the input base like every input path (absolute / `..` refused) |
| `origin` | yes | world position (mm) of sample `(0, 0, 0)` — the grid frame is YOURS to declare, `.npy` carries none (for `ace_fea` per-element fields pass `origin_mm + cell/2` so samples land on element centers) |
| `cell` | yes | sample spacing (mm), cubic on all axes |
| `normalize` | no | affine remap `[lo, hi] → [0, 1]`, clamped (e.g. `[0, allowable_stress]`) |
| `law` | no | **grade law**: sampled value clamped to `[0, 1]` then mapped `0 → law[0]`, `1 → law[1]` (mm — the offset `offset_by` applies; positive inflates). Omit to feed the raw values through unmapped |

Loading is strict so grading stays honest: a missing file is `io`, and a
malformed one — wrong dtype, Fortran order, non-3-D shape, **any non-finite
value** (one NaN would silently poison every nearby trilinear sample) — is
`invalid_param` carrying the kernel's precise reason. Executed refusal
(`missing_field.npy` absent):

```json
{"kind": "io",
 "message": "op 'graded': at expr.field.grid: cannot read 'missing_field.npy': No such file or directory (os error 2)"}
```

The Lipschitz caveat above applies doubly: a grid law's slope is
`(law[1] − law[0]) / (value-change distance)` — cell-scale value jumps make
steep laws, so mesh grid-graded trees with `"mesher": "manifold"` (as every
executed example here does). Worked example — a ±ramp density (2×1×1 grid,
`0` at `x = 0` → `1` at `x = 20`) grading a 20×20×10 pad from −0.6 mm (thin
the idle end) to +0.9 mm (fatten the loaded end), measured 4321.9 mm³
vs 4000 ungraded (executed; the two-sample `.npy` is 134 bytes):

```json
{"ops": [
  {"id": "graded", "op": "implicit", "voxel": 0.4, "mesher": "manifold",
   "domain": {"min": [-2, -2, -2], "max": [22, 22, 12]},
   "expr": {"op": "offset_by", "max_abs": 1.0,
     "in": {"shape": "box", "min": [0, 0, 0], "max": [20, 20, 10]},
     "field": {"grid": {"path": "stress_density.npy", "origin": [0, 0, 0], "cell": 20,
                        "law": [-0.6, 0.9]}}},
   "file": "graded_pad.stl"}
]}
```

Receipt (real run): `{"triangles": 22292, "volume": 4321.90015324077,
"watertight": true, "healed": false}`.

#### The periodic lattices, textures and text — executed tour

**`strut_lattice`** — the octet truss as an infill core, clipped closed by the
same box that bounds it (the leaf is periodic over ALL of space; unshrouded it
is refused as unbounded by ops that need bounds, and an oversized meshing
domain would cut its struts open — same doctrine as `tpms`):

```json
{"ops": [
  {"id": "core", "op": "implicit", "voxel": 0.25, "mesher": "manifold",
   "expr": {"op": "intersection",
     "a": {"shape": "strut_lattice", "kind": "octet", "cell": 8, "radius": 0.8,
           "min": [0, 0, 0], "max": [32, 32, 16]},
     "b": {"shape": "box", "min": [0, 0, 0], "max": [32, 32, 16]}},
   "file": "octet_core.stl"}
]}
```

Receipt (real run): `{"triangles": 682084, "volume": 6222.250240555123,
"watertight": true, "healed": false}` — 38.0% solid fraction at cell 8 /
r 0.8. For calibration: ONE octet cell (cell 10, r 1) through this exact
grammar measures 379.2 mm³ = 37.9% (pinned in `tests/implicit_wave.rs`),
vs the kernel's 39.3% field-fraction pin for the same family — the box clip
shaves the boundary strut bulges, which is the honest difference between the
periodic field's fraction and a clipped, meshed block. Executed refusal
(`"kind": "hexcomb"`):

```json
{"kind": "invalid_param",
 "message": "op 'lat': at expr: 'kind' must be one of bcc|fcc|octet, got Some(\"hexcomb\")"}
```

**`pipe_path`** — a coolant channel skeleton (three segments, uniform Ø5):

```json
{"ops": [
  {"id": "channel", "op": "implicit", "voxel": 0.3,
   "expr": {"shape": "pipe_path", "points": [[0, 0, 0], [30, 0, 0], [30, 20, 0], [30, 20, 15]], "radius": 2.5},
   "file": "coolant_path.stl"}
]}
```

Receipt (real run): `{"triangles": 30400, "volume": 1334.217212451825,
"watertight": true}` (capsule chain: ~65 mm of Ø5 tube + end caps − elbow
overlaps). Given identical points, `pipe_path` and `pipe` produce the SAME
field — volume agreement is pinned to 1e-6 in `tests/implicit_wave.rs`.

**`text`** — a part number milled into a name plate (the stroke tube straddles
`z = 0`, so translating the text onto the top face and differencing engraves a
half-round groove; union instead to emboss):

```json
{"ops": [
  {"id": "tag", "op": "implicit", "voxel": 0.12, "mesher": "manifold",
   "expr": {"op": "difference",
     "a": {"shape": "box", "min": [0, 0, 0], "max": [40, 12, 4]},
     "b": {"op": "translate", "offset": [4, 2, 4],
       "in": {"shape": "text", "text": "LM-10", "height": 8, "stroke_radius": 0.6}}},
   "file": "name_plate.stl"}
]}
```

Receipt (real run): `{"triangles": 202300, "volume": 1878.8347443143955,
"watertight": true}` (plate 1920 mm³ minus the engraved half-round strokes).
Executed refusal — the charset is validated BEFORE the kernel, so a JSON
program can never hit the kernel's panic:

```json
{"kind": "invalid_param",
 "message": "op 'tag': at expr: unsupported character 'Ø' in 'text' — the embedded Hershey Simplex set covers A-Z (lowercase folds to uppercase), 0-9, space, '-' and '.'"}
```

**`displace`** — a Ø16 grip post with a 2 mm crossed knurl (amplitude 0.4 →
peak-to-valley 0.2 mm; the zero set is preserved and the emitted field stays
≤ 1-Lipschitz, so the geometry is exactly the raw displaced surface):

```json
{"ops": [
  {"id": "grip", "op": "implicit", "voxel": 0.2, "mesher": "manifold",
   "expr": {"op": "displace", "amplitude": 0.4,
     "texture": {"kind": "knurl", "pitch": 2.0, "depth_frac": 1.0},
     "in": {"shape": "cylinder", "a": [0, 0, 0], "b": [0, 0, 30], "radius": 8}},
   "file": "knurled_post.stl"}
]}
```

Receipt (real run): `{"triangles": 133196, "volume": 6422.528941795253,
"watertight": true}` — +6.5% over the plain post's 6031.9 mm³, the ridges'
material. `"manifold"` is deliberate: this exact tree on the narrow-band
mesher was measured to pinch (3 non-manifold edges after heal — grazing ridge
saddles on the curved wall) and refuses with `invalid_geometry` naming the
counts; on a flat-faced box the narrow-band route runs clean and its volume
agrees with the dense mesher to <1% (pinned in `tests/implicit_wave.rs` — the
operational proof the ≤ 1-Lipschitz renormalization tears nothing).

A first taste — a sphere smooth-blended onto a box, written straight to STL:

```json
{"ops": [
  {"id": "blob", "op": "implicit", "voxel": 0.4,
   "expr": {"op": "smooth_union", "k": 3,
     "a": {"shape": "sphere", "center": [0, 0, 12], "radius": 8},
     "b": {"shape": "box", "min": [-10, -10, 0], "max": [10, 10, 10]}},
   "file": "blob.stl"}
]}
```

A helical spring inside a capsule cage — `helix_pipe` plus a `circular_pattern`:

```json
{"ops": [
  {"id": "spring", "op": "implicit", "voxel": 0.25,
   "expr": {"op": "union",
     "a": {"shape": "helix_pipe", "center": [0, 0, 0], "axis": [0, 0, 1],
           "r_helix": 8, "pitch": 6, "turns": 3, "radius": 1.5},
     "b": {"op": "circular_pattern", "center": [0, 0, 0], "axis": [0, 0, 1], "count": 6,
       "in": {"shape": "capsule", "a": [14, 0, -3], "b": [14, 0, 21], "radius": 1.2}}},
   "file": "spring_cage.stl"}
]}
```

#### Graded lattice: expression-driven wall thickness

The nTop-style workflow as pure data: a gyroid whose wall half-thickness ramps
0.6 → 1.4 mm bottom-to-top, driven by the field `0.02·(z + 20)` through
`offset_by` (field gradient 0.02 ≪ 1, per the contract), clipped to a box and
extracted dense (`manifold` — a TPMS shell is junction-rich). Measured: the
grading holds 2.3× the ungraded lattice's volume, watertight at voxel 0.8
(pinned by `graded_gyroid_lattice_program_matches_rust_reference`):

```json
{"ops": [
  {"id": "lattice", "op": "implicit", "voxel": 0.8, "mesher": "manifold",
   "domain": {"min": [-20, -20, -20], "max": [20, 20, 20]},
   "expr": {"op": "intersection",
     "a": {"op": "offset_by", "max_abs": 0.8,
       "in": {"shape": "gyroid", "min": [-20, -20, -20], "max": [20, 20, 20],
              "scale": 0.35, "thickness": 0.6},
       "field": {"op": "mul", "a": 0.02, "b": {"op": "add", "a": "z", "b": 20.0}}},
     "b": {"shape": "box", "min": [-20, -20, -20], "max": [20, 20, 20]}},
   "file": "graded_lattice.stl"}
]}
```

#### The I6 proof: an M10×1.5 machine bolt with a REAL helical thread, pure JSON

The `hybrid_showcase` flagship bolt — Ø10 shank, AF16 hex head, ISO-form
helical thread (pitch 1.5, depth 0.85, threaded z 2…28) — needed two custom
Rust `Sdf` structs when it shipped. This program rebuilds it with **zero
Rust**: the thread is the helical-coordinate trapezoid written in the scalar
language. The idiom, piece by piece:

- cylindrical radius — `rad = {"op": "length2", "a": "x", "b": "y"}`;
- helix unwrap — `θ = {"op": "atan2", "y": "y", "x": "x"}`, then the axial
  offset to the nearest thread turn, recentered branchlessly into
  `[−P/2, P/2)`: `u = mod(z − P·θ/2π + P/2 − z0, P) − P/2` (continuous across
  the atan2 branch cut because the jump is exactly one pitch `P`);
- the swept trapezoid is a FIXED convex quad in the `(rad, u)` plane — its
  field is the `max` of four edge half-planes
  `(rad − aᵣ)·êᵤ − (u − aᵤ)·êᵣ` (unit edge normals as constants), `max`-ed
  with the two span planes `z0 − z` and `z − z1`;
- `lipschitz_bound: 1.5` — each half-plane is `α·rad + β·u + c` with
  `α² + β² = 1`, `|∇rad| = 1` and `|∇u| ≤ √(1 + (P/2πr)²) ≈ 1.003` over the
  thread's radial band, so `|α| + |β|·1.003 ≤ √2·1.003 < 1.5` honestly bounds
  the slope (the kernel divides by it — geometry unchanged, pruning safe).

The hex head is six half-planes in an `expr_sdf` (`max(|x|, |x/2 ± y·√3/2|) −
AF/2`, 1-Lipschitz exactly), and the shank is a plain `cylinder` leaf. The
union self-intersects (the thread root is buried 0.3 mm in the shank), which
no exact B-rep boolean can stitch — the implicit extraction fuses it into ONE
watertight manifold. Pinned by `pure_json_helical_thread_bolt_matches_rust_reference`:
watertight, volume within 2% of the Rust-built reference (measured agreement
at voxel 0.08: **4870.82 vs 4870.82 mm³, Δ < 0.0001%**; 0.06 is the
showcase's resin-grade choice if you have a minute to spare).

```json
{
  "ops": [
    {
      "id": "bolt",
      "op": "implicit",
      "voxel": 0.08,
      "domain": {"min": [-9.3, -9.3, -0.2], "max": [9.3, 9.3, 46.6]},
      "expr": {
        "op": "union",
        "a": {"shape": "cylinder", "a": [0, 0, 0], "b": [0, 0, 40], "radius": 5},
        "b": {
          "op": "union",
          "a": {
            "shape": "expr_sdf",
            "expr": {
              "op": "max",
              "a": {
                "op": "max",
                "a": {"op": "sub", "a": {"op": "abs", "arg": "x"}, "b": 8.0},
                "b": {
                  "op": "max",
                  "a": {
                    "op": "sub",
                    "a": {
                      "op": "abs",
                      "arg": {
                        "op": "add",
                        "a": {"op": "mul", "a": "x", "b": 0.5},
                        "b": {"op": "mul", "a": "y", "b": 0.866025403784}
                      }
                    },
                    "b": 8.0
                  },
                  "b": {
                    "op": "sub",
                    "a": {
                      "op": "abs",
                      "arg": {
                        "op": "add",
                        "a": {"op": "mul", "a": "x", "b": -0.5},
                        "b": {"op": "mul", "a": "y", "b": 0.866025403784}
                      }
                    },
                    "b": 8.0
                  }
                }
              },
              "b": {"op": "max", "a": {"op": "sub", "a": 40.0, "b": "z"}, "b": {"op": "sub", "a": "z", "b": 46.4}}
            },
            "lipschitz_bound": 1.0,
            "min": [-9.2376, -9.2376, 40.0],
            "max": [9.2376, 9.2376, 46.4]
          },
          "b": {
            "shape": "expr_sdf",
            "expr": {
              "op": "max",
              "a": {
                "op": "max",
                "a": {
                  "op": "max",
                  "a": {
                    "op": "sub",
                    "a": {
                      "op": "mul",
                      "a": {"op": "sub", "a": {"op": "length2", "a": "x", "b": "y"}, "b": 4.7},
                      "b": 0.41529234959
                    },
                    "b": {
                      "op": "mul",
                      "a": {
                        "op": "sub",
                        "a": {
                          "op": "sub",
                          "a": {
                            "op": "mod",
                            "a": {
                              "op": "add",
                              "a": {
                                "op": "sub",
                                "a": "z",
                                "b": {"op": "mul", "a": {"op": "atan2", "y": "y", "x": "x"}, "b": 0.238732414638}
                              },
                              "b": -1.25
                            },
                            "b": 1.5
                          },
                          "b": 0.75
                        },
                        "b": -0.645
                      },
                      "b": 0.909688003863
                    }
                  },
                  "b": {
                    "op": "sub",
                    "a": {"op": "mul", "a": {"op": "sub", "a": {"op": "length2", "a": "x", "b": "y"}, "b": 5.85}, "b": 1.0},
                    "b": {
                      "op": "mul",
                      "a": {
                        "op": "sub",
                        "a": {
                          "op": "sub",
                          "a": {
                            "op": "mod",
                            "a": {
                              "op": "add",
                              "a": {
                                "op": "sub",
                                "a": "z",
                                "b": {"op": "mul", "a": {"op": "atan2", "y": "y", "x": "x"}, "b": 0.238732414638}
                              },
                              "b": -1.25
                            },
                            "b": 1.5
                          },
                          "b": 0.75
                        },
                        "b": -0.12
                      },
                      "b": 0.0
                    }
                  }
                },
                "b": {
                  "op": "max",
                  "a": {
                    "op": "sub",
                    "a": {
                      "op": "mul",
                      "a": {"op": "sub", "a": {"op": "length2", "a": "x", "b": "y"}, "b": 5.85},
                      "b": 0.41529234959
                    },
                    "b": {
                      "op": "mul",
                      "a": {
                        "op": "sub",
                        "a": {
                          "op": "sub",
                          "a": {
                            "op": "mod",
                            "a": {
                              "op": "add",
                              "a": {
                                "op": "sub",
                                "a": "z",
                                "b": {"op": "mul", "a": {"op": "atan2", "y": "y", "x": "x"}, "b": 0.238732414638}
                              },
                              "b": -1.25
                            },
                            "b": 1.5
                          },
                          "b": 0.75
                        },
                        "b": 0.12
                      },
                      "b": -0.909688003863
                    }
                  },
                  "b": {
                    "op": "sub",
                    "a": {"op": "mul", "a": {"op": "sub", "a": {"op": "length2", "a": "x", "b": "y"}, "b": 4.7}, "b": -1.0},
                    "b": {
                      "op": "mul",
                      "a": {
                        "op": "sub",
                        "a": {
                          "op": "sub",
                          "a": {
                            "op": "mod",
                            "a": {
                              "op": "add",
                              "a": {
                                "op": "sub",
                                "a": "z",
                                "b": {"op": "mul", "a": {"op": "atan2", "y": "y", "x": "x"}, "b": 0.238732414638}
                              },
                              "b": -1.25
                            },
                            "b": 1.5
                          },
                          "b": 0.75
                        },
                        "b": 0.645
                      },
                      "b": 0.0
                    }
                  }
                }
              },
              "b": {"op": "max", "a": {"op": "sub", "a": 2.0, "b": "z"}, "b": {"op": "sub", "a": "z", "b": 28.0}}
            },
            "lipschitz_bound": 1.5,
            "min": [-5.85, -5.85, 2.0],
            "max": [5.85, 5.85, 28.0]
          }
        }
      },
      "file": "bolt_fused.stl"
    }
  ]
}
```

### `shell`
Hollow a bound solid into a closed wall of thickness `wall` mm, **preserving
its outer surface** (the enclosure workflow: model the outside, shell it, cut
openings in a later program stage on the mesh side). Named `shell` to match the
kernel's own `Feature::Shell` in the `.lmcpart` grammar — one vocabulary across
both surfaces (do not confuse it with the `shells` *count* in `validate` /
`assert`, which is a topology measure).

**This is a voxel-route op by construction — the result is `voxel_healed`,
never exact.** The solid is lifted into a winding-number SDF (the same
machinery as the exact-else-heal export fallback), the inward-offset copy is
subtracted (`outer − offset(inner, −wall)`, exactly the kernel's
`Feature::Shell` semantics), and the result is re-meshed watertight by Manifold
Dual Contouring at `voxel` resolution. Accuracy is on the order of the voxel
size (a 30 mm cube at the 0.3 default measured 0.004% off the analytic wall
volume, but sharp-feature fidelity is *voxel-grade*, not B-rep-grade).

Like `gyroid_block` / `implicit`, `shell` **binds no solid** — no op in the
vocabulary consumes a mesh (to keep hollowing INSIDE the solid environment
use `shell_solid`, below). The mesh goes to `file` (`.stl`/`.3mf`) and the
measures carry `route: "voxel_healed"`, `volume`, `triangles`, `watertight`.
A hollow deeper than the grid resolves is rejected up front: `wall` must be
at least `2 × voxel` (shrink `voxel` or thicken the wall). The interior void is
fully enclosed (the wall also spans top and bottom); volume ≈ outer − inner.

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid |
| `wall` | number | yes | wall thickness (mm), > 0 and ≥ 2·`voxel` |
| `voxel` | number | no (0.3) | SDF re-mesh voxel size (mm); grid capped at 5·10⁷ cells |
| `file` | string | no | output mesh path; extension picks `.stl`/`.3mf` |

```json
{"ops": [
  {"id": "case", "op": "box", "min": [0,0,0], "max": [60, 40, 25]},
  {"id": "hollowed", "op": "shell", "in": "case", "wall": 2, "voxel": 0.3, "file": "case_shell.stl"}
]}
```

### `offset_solid`
Signed surface offset that **binds a solid**: grow (`delta > 0` — the
Minkowski sum with a ball, so convex edges and corners gain a genuine
`delta`-radius round; that round is true offset geometry, not an artifact) or
shrink (`delta < 0` — erosion; any region thinner than `2·|delta|` vanishes,
and eroding past the part's inradius is a loud refusal, never an empty bind).
Wire to `kernel_model::shell::offset_to_solid`.

**Voxel route by construction — the receipts say `route: "voxel"` and
`faceted: true`, never exact.** The solid is lifted into a winding-number SDF,
the shifted level set is re-extracted by Manifold Dual Contouring at `voxel`,
and the mesh is wrapped back into a B-rep with **one planar face per
triangle** — no analytic surface tags survive, so downstream curvature-reading
ops (fillet witnesses, `diameter` measures) have nothing to read; exact
*planar* booleans, `validate`/`volume`/`mass_properties`, STL/3MF/STEP exports
all work. Surface placement is accurate to ~`voxel/2` plus the input
tessellation's chord error.

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid |
| `delta` | number | yes | signed offset (mm): positive grows, negative shrinks |
| `voxel` | number | no (0.3) | re-extraction voxel size (mm); grid capped at 5·10⁷ cells |

Measures: `route` (`"voxel"`), `faceted` (true), `voxel`, `delta`, `faces`,
`volume`, `closed`, `manifold`, `shells`, `genus`.

Executed — a 20³ pad grown +2 mm; the analytic Steiner volume of the rounded
result is `a³ + 6a²r + 3πr²a + 4πr³/3 = 13587.5 mm³` and the receipt reads
13589.9 (+0.02%, voxel 0.4):

```json
{"ops": [
  {"id": "pad",   "op": "box", "min": [0, 0, 0], "max": [20, 20, 20]},
  {"id": "grown", "op": "offset_solid", "in": "pad", "delta": 2, "voxel": 0.4},
  {"id": "v",     "op": "volume", "in": "grown"}
]}
```

```json
{"id": "grown", "ok": true,
 "measures": {"closed": true, "delta": 2.0, "faces": 42948, "faceted": true,
              "genus": 0, "manifold": true, "route": "voxel", "shells": 1,
              "volume": 13589.928053419462, "voxel": 0.4}}
```

(Shrinking is the same call with `delta: -2` — a 20³ cube erodes to the sharp
16³ = 4096 mm³ cube, measured within 1%, pinned in `tests/implicit_wave.rs`.)
Executed refusal — total erosion:

```json
{"kind": "invalid_param",
 "message": "op 'gone': offset_solid produced an empty result — a negative delta (-6 mm) at or beyond the part's inradius erodes it away entirely (regions thinner than 2·|delta| vanish); shrink |delta|"}
```

### `shell_solid`
Hollow a bound solid into a closed wall of `thickness` mm — outer surface
preserved, cavity sealed — and **bind the result as a solid** (the
solid-environment sibling of the file-writing `shell`; wire to
`kernel_model::shell::shell_to_solid`). **Voxel route by construction**
(`route: "voxel"`, faceted — same contract and caveats as `offset_solid`; the
cavity's concave corners round at ~`voxel` scale). The hollow topology
survives the bridge: the cavity arrives as a second nested shell —
`shells: 2` / `cavity: true` in the measures, and `assert {shells: 2}` gates
it in-program. `shells: 1` / `cavity: false` means the wall met itself
(`thickness` ≥ the part's inradius left no cavity) — stated, never hidden.

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid |
| `thickness` | number | yes | wall thickness (mm), > 0 and ≥ 2·`voxel` |
| `voxel` | number | no (0.3) | re-extraction voxel size (mm); grid capped at 5·10⁷ cells |

Measures: `route` (`"voxel"`), `faceted`, `voxel`, `thickness`, `cavity`,
`faces`, `volume`, `closed`, `manifold`, `shells`, `genus`.

Executed — a 30×30×20 case hollowed at 2 mm; the analytic wall volume
(erosion cavity has sharp corners for a box) is `18000 − 26·26·16 =
7184 mm³`, measured 7183.9 (−0.002%, voxel 0.4), cavity proven by the gate:

```json
{"ops": [
  {"id": "case",   "op": "box", "min": [0, 0, 0], "max": [30, 30, 20]},
  {"id": "hollow", "op": "shell_solid", "in": "case", "thickness": 2, "voxel": 0.4},
  {"id": "gate",   "op": "assert", "in": "hollow", "shells": 2, "valid": true}
]}
```

```json
{"id": "hollow", "ok": true,
 "measures": {"cavity": true, "closed": true, "faces": 90200, "faceted": true,
              "genus": 0, "manifold": true, "route": "voxel", "shells": 2,
              "thickness": 2.0, "volume": 7183.907142157295, "voxel": 0.4}}
```

Executed refusals — what `kernel_model::shell` cannot deliver is refused
up front, deterministically:

```json
{"kind": "invalid_param",
 "message": "op 'bad': thickness must be a positive wall thickness in mm"}
```
```json
{"kind": "invalid_param",
 "message": "op 'bad': thickness 0.5 mm is under 2 × voxel (0.4 mm) — the grid cannot resolve the wall; shrink 'voxel' or thicken it"}
```

### `solid_from_implicit`
**Reverse bridge v1** (`kernel_model::reverse::implicit_to_solid`) — the one
door from the field world back into the solid environment, honest about what
it is. The implicit `expr` tree (same grammar as `implicit`) is meshed
**dense** (Manifold Dual Contouring — no Lipschitz assumption, so `expr_sdf`
bounds are not narrow-band-verified here) at `voxel` over `domain` (default:
the tree's own finite bounds), then wrapped into a validated **faceted**
B-rep: one planar face per triangle, adjacent exactly-coplanar facets
coalesced into multi-loop faces. The wrap is gated on **volume conservation**
(`|solid − mesh| ≤ 1e-6` relative) — a coalesce/stitch that altered geometry
is a refusal (`invalid_geometry` with both measured volumes), never a quiet
corruption; success stamps `volume_conserved: true` in the receipts.

What v1 buys: a TPMS/strut lattice, smooth blend, or textured body becomes a
real bound `Solid` — STEP export and exact planar booleans downstream. What
it does NOT do: analytic curved-surface recovery (a voxelized cylinder is N
planar facets, never a `Surface::Cylinder`) — that is the ledgered v2. Every
face is chord-accurate to `voxel`, and a fine voxel on a large part means a
very large faceted solid (and STEP file) — inherent to the faceted contract.

| param | type | required | meaning |
|---|---|---|---|
| `expr` | tree | yes | the implicit expression tree (grammar above) |
| `voxel` | number | yes | extraction voxel size (mm) — also every face's chord fidelity |
| `domain` | `{min, max}` | no | explicit meshing box; default: the tree's own (finite) bounds |

Measures: `route` (`"voxel"`), `faceted`, `voxel`, `faces`, `volume`,
`volume_conserved`, `approximate_offset`, `closed`, `manifold`, `shells`,
`genus`.

Executed — **the round trip**: a BCC strut lattice, clipped closed, bridged
to a solid, gated, and written as STEP (2×2×2 cells at cell 10 / r 1.6;
the receipts carry the lattice's real topology — genus 30):

```json
{"ops": [
  {"id": "bridged", "op": "solid_from_implicit", "voxel": 0.5,
   "expr": {"op": "intersection",
     "a": {"shape": "strut_lattice", "kind": "bcc", "cell": 10, "radius": 1.6,
           "min": [0, 0, 0], "max": [20, 20, 20]},
     "b": {"shape": "box", "min": [0, 0, 0], "max": [20, 20, 20]}}},
  {"id": "gate", "op": "assert", "in": "bridged", "valid": true},
  {"id": "step", "op": "export_step", "in": "bridged", "file": "bcc_core.step"}
]}
```

```json
{"id": "bridged", "ok": true,
 "measures": {"approximate_offset": false, "closed": true, "faces": 39647,
              "faceted": true, "genus": 30, "manifold": true, "route": "voxel",
              "shells": 1, "volume": 3184.2053803993526,
              "volume_conserved": true, "voxel": 0.5}}
```

(39.8% solid; the whole chain is pinned end-to-end in
`tests/implicit_wave.rs::roundtrip_strut_lattice_to_solid_to_step`.)
Executed refusal — a domain the field never crosses:

```json
{"kind": "invalid_param",
 "message": "op 'ghost': implicit_to_solid: the field has no surface inside Aabb { min: Vec3(50.0, 50.0, 50.0), max: Vec3(60.0, 60.0, 60.0) } at voxel 0.5 (or the lattice exceeded the mesher's cell cap) — nothing to bridge"}
```

### `thin_wall`
SAMPLED thin-wall census (`kernel_model::reverse::thin_wall_report`) of a
bound solid (`in`, lifted through the winding-number MeshSdf) **or** an
implicit `expr` tree — exactly one. A `samples`³ lattice spans the census box;
at each interior point that is a local |distance| maximum along ±gradient (a
medial-surface point) the local wall thickness is estimated as `2·|d|`.
Receipts: `thinnest` (mm), `at` (its location), `below_count` (medial samples
under `t_min` — a **sample census**, not a defect count: one thin wall yields
many samples). Binds nothing.

**An ESTIMATE, stated bluntly.** It can **under-report by up to ~one lattice
cell** (an accepted sample sits up to half a cell off the true mid-surface —
conservative for a minimum-wall warning) and can **miss entirely** a wall
thinner than the cell. On a CSG *bound* (smooth blends, TPMS, offsets of
booleans) `2·|d|` is a lower bound, not the exact thickness. And on a
sharp-edged body the medial axis includes the **edge-wedge bisectors** — near
a 90° edge the material wedge genuinely thins toward zero, so a whole-part
census of an exact-B-rep box reads the edge sliver, not a wall (measured:
0.60 mm on a 3 mm plate at 48 samples — pinned as the documented caveat).
Interrogate walls with an interior `domain`, or use the ray-based
`wall_thickness` for face-to-face readings on exact solids; field-born
(voxel-extracted) geometry has ~voxel-rounded edges and does not hit the
sharp-edge case. Use `thin_wall` to WARN; gate final claims on finer sampling.

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | one of | a bound solid (exclusive with `expr`) |
| `expr` | tree | one of | an implicit tree (exclusive with `in`) |
| `t_min` | number | yes | census threshold (mm) for `below_count` |
| `samples` | int | no (64) | lattice points per axis, 8..=256 (cost ~samples³ field evaluations) |
| `domain` | `{min, max}` | no | census box; default: the solid's aabb (nudged half a step off its own faces) / the tree's bounds |

Measures: `status` (`"measured"` / `"no_interior_samples"` — an empty census
is an explicit status, never a raw ∞ in JSON), `basis`
(`"sampled_medial_estimate"`), `thinnest` (null when no interior sample),
`at`, `below_count`, `t_min`, `samples`.

Executed — a Ø20 sphere shelled to a 1.4 mm wall (`shell` `t: 0.7`),
censused at 64³ against a 1.6 mm rule: `thinnest` reads 1.05 — the true
1.4 minus exactly the documented one-cell under-report (cell ≈ 0.34 mm) —
and 11544 samples fall under the rule:

```json
{"ops": [
  {"id": "census", "op": "thin_wall", "t_min": 1.6, "samples": 64,
   "expr": {"op": "shell", "t": 0.7,
     "in": {"shape": "sphere", "center": [0, 0, 0], "radius": 10}}}
]}
```

```json
{"id": "census", "ok": true,
 "measures": {"at": [-0.5095243453979492, 3.90634822845459, 9.001585960388184],
              "basis": "sampled_medial_estimate", "below_count": 11544,
              "samples": 64, "status": "measured", "t_min": 1.6,
              "thinnest": 1.0517390966415405}}
```

### `min_ligament`
Advisory **minimum-ligament echo** (`kernel_brep::holes::min_ligament`,
FRICTION #21): the thinnest remaining material between a PLANNED Ø`d`
through-bore at `at` + `axis` (hole-wizard convention: `axis` points INTO the
material) and the solid's existing boundary — measured BEFORE any cut, so a
program can gate a drilling plan on a wall-thickness rule instead of
discovering a torn web after the fact. Purely a measurement; nothing is cut,
nothing binds.

What is measured, exactly: 64 stations on the would-be bore wall (one ring at
the **mid-span** of the material extent along the axis), each measured to the
current boundary by exact per-triangle closest point on the default
tessellation; the echo is the minimum. Honest caveats: faces the bore will
pierce are part of the boundary, so the echo is **clamped above by ~half the
material span** (a value near `span/2` means "no lateral ligament thinner
than the mid-depth", which is what the thin-web warning regime needs); it is
a 64-station sample on ONE ring (features strictly above/below the ring are
not seen); the bore is treated as a through hole (blind-floor ligaments are
out of scope). An unanswerable question is an explicit **status**, never a
raw NaN/∞ in JSON: `no_material` (no material along `+axis` from `at`) or
`no_boundary` (empty solid — unreachable for a bound solid).

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid |
| `at` | `[x,y,z]` | yes | planned hole center on the entry face |
| `axis` | `[x,y,z]` | yes | drilling direction, INTO the material |
| `d` | number | yes | planned bore **diameter** (mm) |

Measures: `status` (`"measured"` / `"no_material"` / `"no_boundary"`),
`ligament` (mm; null unless measured), `basis`
(`"mid_span_ring_64_stations"`), and the echoed `d` / `at` / `axis`.

Executed — a Ø6 bore planned 5 mm from a plate edge leaves `5 − 3 = 2 mm` of
web, and the echo reads exactly that:

```json
{"ops": [
  {"id": "plate", "op": "box", "min": [0, 0, 0], "max": [30, 30, 12]},
  {"id": "web",   "op": "min_ligament", "in": "plate", "at": [5, 15, 12], "axis": [0, 0, -1], "d": 6}
]}
```

```json
{"id": "web", "ok": true,
 "measures": {"at": [5.0, 15.0, 12.0], "axis": [0.0, 0.0, -1.0],
              "basis": "mid_span_ring_64_stations", "d": 6.0,
              "ligament": 2.0, "status": "measured"}}
```

(The same plate asked with `axis: [0, 0, 1]` — pointing OUT of the material —
answers `{"status": "no_material", "ligament": null}`; pinned in
`tests/implicit_wave.rs`.)

## Assembly ops (in-program)

The assembly surface as first-class ops — build parts, instance them, mate
them, solve, verify, export and persist, all in ONE program (no hand-authored
`.lmcasm` needed; `asm_save` writes one for you). Instances are referenced by
their `asm_instance*` op ids; nothing moves until `asm_solve`. The first
instance is the GROUND frame (`asm_mate {kind:"fixed"}` grounds others).

| op | does |
|---|---|
| `asm_instance` | place a bound solid at a seed pose (`solid`, `name?`, `translate?`, `rotate?` `{axis, degrees, center?}`, `material?` `{name, density_g_cm3}`) |
| `asm_instance_mesh` | place a mesh FILE (`.stl`/`.obj`/`.3mf`/`.ply`) — welded, measured honestly as a mesh |
| `asm_mate` | raw mate from explicit LOCAL geometry; `kind`: `coincident` · `distance` · `parallel` · `concentric` · `angle` (0–180°, directional) · `axis_distance` (parallel axes at center `distance` — the gear mate) · `fixed` |
| `asm_mate_axis` | mate DERIVED from real B-rep faces: each witness picks the nearest cylindrical/conical/toric face; its exact analytic axis becomes the mate (concentric, or `axis_distance` with `distance`). Receipts echo the derived axes |
| `asm_mate_face` | derived face seat: coincident + parallel from each witness face's plane, `offset` mm apart (0 = flush). CENTROID-ON-CENTROID (slide is constrained), and a boolean can fragment a big plane — the seat centers on the PICKED fragment (echoed); use raw `asm_mate` for a specific landing point |
| `asm_solve` | relax poses to satisfy the mates; receipts: `residual`, `converged`, `per_mate` residuals, `dof` (numeric rank — `under_constrained (N free DOF)` says what is honestly unmated), solved `poses`. FAILS on non-convergence naming the worst mates (`allow_unconverged` to inspect anyway); statically broken mates refuse up front |
| `asm_contacts` | all-pairs proximity at current poses (`window` 1.0, exact adaptive tessellation at `tol` 0.05 for solids); `touching` = ≤1e-6 |
| `asm_interference_volume` | shared material (mm³) of two instances, voxel-sampled (`voxel` 0.3) |
| `asm_mass_properties` | per-instance volume (`volume_source` `exact`/`mesh`, never conflated) × material density; total mass NULL + `mass_complete:false` when any instance lacks a material (honest omission) |
| `asm_export` | merged mesh (+ optional per-instance files), route named per instance |
| `asm_export_step` | AP214 STEP assembly of the solid-backed instances; mesh instances listed as skipped |
| `asm_save` | write a re-executable `.lmcasm`: `load_part`-sourced instances keep their `.lmcpart` path; program-built geometry exports next to the file as `{"mesh": …}` sources; mates serialized; solved poses become the stored seeds |
| `gear_train_poses` | epicyclic (Wolfrom) member poses at an input angle: validates assemblability (loud), returns ratio + per-member rotations and planet translations at `module·(S+Pa)/2` — feed straight into `asm_instance` so a posed train meshes exactly |

One program, the whole loop (abridged from `kernel-api/tests/asm_ops.rs`, which
also asserts every receipt):

```json
{"ops": [
  {"id": "blank",  "op": "box", "min": [-20, -20, 0], "max": [20, 20, 8]},
  {"id": "bore",   "op": "cylinder", "base": [0, 0, 0], "axis": [0, 0, 1], "radius": 4.0, "height": 8.0},
  {"id": "plate",  "op": "difference", "a": "blank", "b": "bore"},
  {"id": "shaft",  "op": "cylinder", "base": [0, 0, 0], "axis": [0, 0, 1], "radius": 3.8, "height": 20.0},

  {"id": "i_plate", "op": "asm_instance", "solid": "plate", "material": {"name": "PLA", "density_g_cm3": 1.24}},
  {"id": "i_shaft", "op": "asm_instance", "solid": "shaft",
   "translate": [9, -6, 3], "rotate": {"axis": [0, 1, 0], "degrees": 30}},

  {"id": "m_axis", "op": "asm_mate_axis",
   "a": "i_plate", "a_witness": [4, 0, 4], "b": "i_shaft", "b_witness": [3.8, 0, 10]},
  {"id": "m_seat", "op": "asm_mate", "kind": "distance",
   "a": "i_plate", "a_point": [0, 0, 0], "b": "i_shaft", "b_point": [0, 0, 0], "distance": 4.0},

  {"id": "solve",    "op": "asm_solve"},
  {"id": "contacts", "op": "asm_contacts"},
  {"id": "save",     "op": "asm_save", "file": "pin_plate.lmcasm"}
]}
```

The shaft is seeded 9 mm off-axis and tilted 30°; `solve` pulls it concentric
into the bore (derived from the REAL bore-wall axis) 4 mm deep, reports
`under_constrained (1 free DOF)` — the spin nothing mates — and `contacts`
measures the designed 0.2 mm ring gap. `save` writes a `.lmcasm` (shaft +
plate as mesh sources) that `kernel-api asm` re-executes bit-honestly.

## Native formats

### `load_part`
Load a **`.lmcpart`** file — the kernel's native parametric format (a
self-describing envelope around the full feature/parameter tree; geometry is
never stored) — evaluate its feature tree to the exact B-rep, and bind the
result as a solid. This is how a program consumes a part that an earlier
session (human or AI) saved or hand-edited; cut it, measure it, export it like
any other solid. Path resolution: a relative path resolves against the
**program file's own directory** when run through the CLI (so a program
references its parts relative to itself and stays relocatable — exactly how a
`.lmcasm` resolves its `path` sources), and against `out_dir` through the bare
library call `run_program` (pass the base explicitly with
`run_program_with_input_base`). Input paths are **confined** to that base just
like outputs: absolute paths and any `..` component are refused.

Fails with `io` when the file cannot be read, and `invalid_param` when it is
not a loadable `.lmcpart` (wrong/missing `format` tag, unsupported `version`,
non-`mm` `units`, malformed document — the message carries the precise reason)
or when its tree has no exact B-rep result (voxel-half-only features such as
shell / gyroid / smooth booleans cannot enter the solid environment).

| param | type | required | meaning |
|---|---|---|---|
| `file` | string | yes | path to a `.lmcpart` file |

Measures: `name`, `units`, `created_with` (the file's envelope header).

A minimal `.lmcpart` is plain JSON you can write (or edit) by hand — a drilled
spacer whose thickness is driven by the parameter `h`:

```json
{
  "format": "lmc-part",
  "version": 1,
  "units": "mm",
  "name": "spacer",
  "created_with": "written by hand",
  "document": {
    "params": {"h": 8.0},
    "features": [
      {"Box": {"center": [{"Literal": 15.0}, {"Literal": 10.0}, {"Literal": 4.0}],
               "size": [{"Literal": 30.0}, {"Literal": 20.0}, {"Param": "h"}]},
       "label": "blank"},
      {"Cylinder": {"center": [{"Literal": 15.0}, {"Literal": 10.0}, {"Literal": 4.0}],
                    "radius": {"Literal": 4.0}, "height": {"Literal": 12.0}}},
      {"Boolean": {"op": "Difference", "a": 0, "b": 1}}
    ],
    "root": 2,
    "suppressed": []
  }
}
```

Saved as `spacer.lmcpart` next to the program file, this program loads,
measures and exports it:

```json
{"ops": [
  {"id": "spacer", "op": "load_part", "file": "spacer.lmcpart"},
  {"id": "v", "op": "volume", "in": "spacer"},
  {"id": "out", "op": "export_stl", "in": "spacer", "file": "spacer.stl"}
]}
```

## Parts library — curated, admission-gated (docs/BAR.md I7)

A **library** is a plain directory: one stored `.lmcpart` per admitted entry
plus a byte-stable, sorted-key `index.json` — git-version the directory and
every admission, deprecation and removal is auditable and reversible. The five
`library_*` ops are how an AI grows its own reusable vocabulary **without
polluting it**:

- **Admission gate**: `library_add` accepts a candidate only after building it
  at the declared interface **defaults**, the sampled **range corners** (all
  2ⁿ min/max combinations, capped at 16 by a deterministic spread that always
  keeps all-min, all-max and each single-parameter extreme), and the
  **midpoint** — each sample must be a closed manifold AND rebuild
  **volume-bit-deterministically** (two full evaluations must agree to the
  last bit). Per-sample measures are recorded in the index as evidence. A
  failure is a loud `admission_rejected` naming the sample; nothing is
  admitted. (Honest scope: the gate proves the sampled points, not every value
  between them.)
- **Curation**: `library_deprecate` hides an entry from search while existing
  references keep building (instantiate carries a warning);
  `library_remove` refuses with the dependent list while any `.lmcasm` in the
  directory references the entry by path, unless forced.
- **Dates are caller-supplied** (`meta.provenance.date`): the kernel never
  stamps clock time into library data, so identical programs write identical
  bytes.

`dir` resolves like file params (relative joins `--out-dir`) and is created on
demand. Versions are integers ≥ 1; one `(name, version)` is immutable once
admitted — changed geometry goes in as a new version. Unversioned lookups use
the highest admitted version.

### `library_add`
Admit a candidate part. The candidate is a full `.lmcpart` envelope, passed
either **inline** (`part` — the feature tree is plain JSON, see `load_part`)
or by path (`part_file`); exactly one of the two is required. Every parameter
named in `meta.params` must exist in the candidate document's parameter table
(a typo'd interface would drive nothing — that is a loud `invalid_param`).

| param | type | required | meaning |
|---|---|---|---|
| `dir` | string | yes | library directory |
| `part` | object | one of | the candidate `.lmcpart` envelope, inline |
| `part_file` | string | one of | path to the candidate `.lmcpart` |
| `meta.name` | string | yes | entry name (1–64 chars `A–Z a–z 0–9 . _ -`, starts alphanumeric; becomes the stored file stem) |
| `meta.version` | int | yes | entry version, ≥ 1 (immutable once admitted) |
| `meta.category` | string | no | coarse search grouping |
| `meta.tags` | [string] | no | search tags (sorted + deduplicated) |
| `meta.description` | string | no | free text (searched) |
| `meta.provenance.author` | string | yes | who authored the part |
| `meta.provenance.date` | string | yes | caller-supplied date (never a clock) |
| `meta.provenance.created_with` | string | no | producing tool (default: this binding's version stamp) |
| `meta.params[]` | array | no | declared interface: `{name, units, default, min, max, description?}` per parameter |

Measures: `name`, `version`, `file` (the stored `.lmcpart`), `gate_samples`,
`gate_rebuilds`, `volume_at_defaults`. Fails with `admission_rejected` (gate),
`invalid_param` (bad meta / duplicate version / not a loadable `.lmcpart`), or
`io`.

```json
{"ops": [
  {"id": "admit", "op": "library_add", "dir": "lib",
   "part": {
     "format": "lmc-part", "version": 1, "units": "mm", "name": "bushing",
     "created_with": "cookbook",
     "document": {
       "params": {"outer_r": 12.0, "bore_r": 4.0, "h": 10.0},
       "features": [
         {"Cylinder": {"center": [{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": 0.0}],
                       "radius": {"Param": "outer_r"}, "height": {"Param": "h"}}},
         {"Cylinder": {"center": [{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": 0.0}],
                       "radius": {"Param": "bore_r"}, "height": {"Literal": 200.0}}},
         {"Boolean": {"op": "Difference", "a": 0, "b": 1}}
       ],
       "root": 2, "suppressed": []
     }
   },
   "meta": {
     "name": "bushing", "version": 1, "category": "spacers",
     "tags": ["bearing", "sleeve"],
     "description": "parametric plain bushing with a through bore",
     "provenance": {"author": "cookbook-ai", "date": "2026-06-10"},
     "params": [
       {"name": "outer_r", "units": "mm", "default": 12.0, "min": 8.0, "max": 16.0},
       {"name": "bore_r",  "units": "mm", "default": 4.0,  "min": 2.0, "max": 5.0},
       {"name": "h",       "units": "mm", "default": 10.0, "min": 4.0, "max": 40.0}
     ]
   }}
]}
```

### `library_search`
Search the curated view (deprecated entries are hidden). Matches are returned
with their full declared interface, so a caller can go straight to
`library_instantiate` without reading `index.json`.

| param | type | required | meaning |
|---|---|---|---|
| `dir` | string | yes | library directory |
| `text` | string | no | case-insensitive substring over name/category/description/tags (empty: all) |
| `tags` | [string] | no | tags the entry must all carry (case-insensitive) |

Measures: `matches` — `[{name, version, category, tags, description,
params: [{name, units, default, min, max}]}]`.

```json
{"ops": [{"id": "find", "op": "library_search", "dir": "lib", "text": "bushing", "tags": ["bearing"]}]}
```

### `library_instantiate`
Rebuild a library entry as a solid: interface **defaults** are applied first,
then the caller's `params` — an unknown parameter or an out-of-range value is
a loud `invalid_param` naming the declared interface/range. A **deprecated**
entry still builds, but the measures carry `"deprecated": true` plus a
`warning` string (docs/BAR.md I7: existing refs keep building, instantiate warns).

| param | type | required | meaning |
|---|---|---|---|
| `dir` | string | yes | library directory |
| `name` | string | yes | entry name |
| `version` | int | no | entry version (default: highest admitted) |
| `params` | object | no | `{name: value}` overrides within the declared ranges |

Measures: `name`, `version`, `deprecated`, `params` (the overrides applied),
and `warning` when deprecated.

```json
{"ops": [
  {"id": "bush", "op": "library_instantiate", "dir": "lib", "name": "bushing",
   "params": {"outer_r": 14.0, "bore_r": 3.0, "h": 20.0}},
  {"id": "plate", "op": "box", "min": [0, 0, 0], "max": [40, 30, 8]},
  {"id": "placed", "op": "translate", "in": "bush", "offset": [20, 15, 12]},
  {"id": "product", "op": "union", "a": "plate", "b": "placed"},
  {"id": "check", "op": "validate", "in": "product"},
  {"id": "out", "op": "export_stl", "in": "product", "file": "product.stl"}
]}
```

### `library_deprecate`
Deprecate every version of a name: hidden from `library_search`, files stay on
disk, `.lmcasm` path references keep loading, and `library_instantiate` keeps
working with a warning. Idempotent. Fails `invalid_param` on an unknown name.

| param | type | required | meaning |
|---|---|---|---|
| `dir` | string | yes | library directory |
| `name` | string | yes | entry name |

Measures: `name`, `deprecated_versions`.

```json
{"ops": [{"id": "retire", "op": "library_deprecate", "dir": "lib", "name": "bushing"}]}
```

### `library_remove`
Remove every version of a name — part files and index rows. Without `force`
the op first scans the directory's `.lmcasm` files and **refuses with kind
`dependents_exist`** (the message lists the referencing assemblies) when any
still references the entry by path. With `force` it removes anyway; keep the
library directory under git so every removal stays recoverable.

| param | type | required | meaning |
|---|---|---|---|
| `dir` | string | yes | library directory |
| `name` | string | yes | entry name |
| `force` | bool | no | skip the dependents refusal (default false) |

Measures: `name`, `removed_files`, `forced`.

```json
{"ops": [{"id": "rm", "op": "library_remove", "dir": "lib", "name": "bushing"}]}
```
### The `.lmcpart` Document grammar — the corners you would otherwise read out of source

The Document (feature-tree) JSON inside a `.lmcpart` is a *different grammar*
from the op surface above. The three places hand-authors have been bitten:

- **Document `Sketch` schema** (inside `ExtrudeSketch.sketch`): point indices
  are wrapped in objects, not pairs —
  `"segments": [{"a": 0, "b": 1}, …]`,
  `"arcs": [{"a": i, "b": j, "center": k, "ccw": true}, …]`,
  `"circles": [{"center": i, "radius_point": j}, …]`,
  with `"points": [[x, y], …]` and `"constraints"` as in the solver. (The op
  surface's `sketch` uses bare `[i, j]` index pairs instead.)
- **Angle units**: the op surface is **degrees everywhere**; the Document's
  `ExtrudeSketch.draft` is a `Dim` in **RADIANS** (e.g. 1.5° =
  `{"Literal": 0.0261799…}`). This is the one known unit asymmetry between the
  two grammars — convert explicitly when translating a program into a part
  file.
- **`Transform.xform` layout**: a flat array of **12 floats, column-major**
  — the linear part's three basis columns then the translation:
  `[x_axis·3, y_axis·3, z_axis·3, translation·3]`. Identity + raise by 82 mm:

  ```json
  {"Transform": {"input": 0,
   "xform": [1,0,0, 0,1,0, 0,0,1, 0,0,82]}}
  ```

  Quaternions in `.lmcasm` poses are `[x, y, z, w]` (e.g. Rx(−90°) =
  `[-0.7071068, 0, 0, 0.7071068]`).

## Standard parts catalog

McMaster-style parametric standard parts, built from the published ISO/DIN/ANSI
dimension tables (see `kernel-model/src/parts/` for the cited `const` tables).
Conventions: dimensions in **mm**, bores and shanks are **diameters**, hex
sizes are **across flats**. Parts build at the origin along **+Z**; place them
with `pose` / `translate` / `rotate_z`. Bore-diameter params are canonically
`bore` on this surface; the Document (`.lmcpart`) field name `bore_d` is
accepted as an alias on `spur_gear` / `gt2_pulley` / `chain_sprocket` so the
two grammars interchange. A size outside its standard's table is a loud
`invalid_param` naming the supported sizes. Threads are not modelled on the
catalog bodies (these are the exact assembly/clearance solids).

### `spur_gear`
ISO 53 involute spur gear: true involute flanks, extruded to `face_width`,
bored, optionally with the DIN 6885-1 hub keyway auto-sized for the bore.
Genus 1 (bore through). A plain bore carries the exact cylinder surface
(π-exact `exact_volume`, true cylinder in STEP), and `export_stl` takes the
`exact` route — no voxel-heal fallback. Honest approximations (documented in
the kernel, not silent): no trochoidal root fillet, no undercut below ~17
teeth.

| param | type | required | meaning |
|---|---|---|---|
| `module` | number | yes | gear module `m` (tooth size), mm |
| `teeth` | int | yes | tooth count `z` |
| `face_width` | number | yes | axial width, mm |
| `bore` | number | yes | bore **diameter**, mm (keep `bore/2 + t2 < m(z/2 − 1.25)`); `bore_d` — the `.lmcpart` Document field name — is accepted as an alias |
| `pressure_angle_deg` | number | no (20) | pressure angle in degrees |
| `keyway` | bool | no (false) | cut the DIN 6885-1 hub keyway for `bore` (bore must be in the 6–75 mm table) |

```json
{"ops": [{"id": "gear", "op": "spur_gear", "module": 2, "teeth": 20,
          "face_width": 10, "bore": 8, "keyway": true}]}
```

### `hex_bolt`
ISO 4017 hex-head bolt body: across-flats and head height from the standard
table, shank at the nominal diameter. Supported sizes: M3, M4, M5, M6, M8,
M10, M12, M16.

| param | type | required | meaning |
|---|---|---|---|
| `m` | number | yes | nominal thread size (the 10 of M10) |
| `length` | number | yes | shank length, mm |

```json
{"ops": [{"id": "bolt", "op": "hex_bolt", "m": 10, "length": 30}]}
```

### `hex_nut`
ISO 4032 hex nut, bored at the nominal thread diameter. Sizes M3–M16 as
`hex_bolt`.

| param | type | required | meaning |
|---|---|---|---|
| `m` | number | yes | nominal thread size |

```json
{"ops": [{"id": "nut", "op": "hex_nut", "m": 5}]}
```

### `washer`
ISO 7089 plain washer (≈ DIN 125 A). Sizes M3–M16.

| param | type | required | meaning |
|---|---|---|---|
| `m` | number | yes | nominal thread size |

```json
{"ops": [
  {"id": "washer", "op": "washer", "m": 5},
  {"id": "v", "op": "volume", "in": "washer"}
]}
```

### `socket_head_cap_screw`
DIN 912 / ISO 4762 socket-head cap screw body with the hexagonal drive socket
cut into the head (real pocket — present for clearance checks). Sizes M3–M16.

| param | type | required | meaning |
|---|---|---|---|
| `m` | number | yes | nominal thread size |
| `length` | number | yes | under-head shank length, mm |

```json
{"ops": [{"id": "shcs", "op": "socket_head_cap_screw", "m": 5, "length": 16}]}
```

### `gt2_pulley`
GT2 2 mm-pitch timing pulley: `teeth` grooves on the standard outer diameter
`OD = 2·teeth/π − 0.508`, toothed band `belt_width` wide, bored; optionally a
retaining flange on each end. Genus 1.

| param | type | required | meaning |
|---|---|---|---|
| `teeth` | int | yes | groove count (≥ 2) |
| `belt_width` | number | yes | toothed band width, mm |
| `bore` | number | yes | bore **diameter**, mm |
| `flanged` | bool | no (false) | add Ø(OD+3) × 1 mm flanges |

```json
{"ops": [{"id": "pulley", "op": "gt2_pulley", "teeth": 20, "belt_width": 6,
          "bore": 5, "flanged": true}]}
```

### `chain_sprocket`
ANSI/ASA B29.1 roller-chain sprocket plate: roller seats on the exact pitch
circle, ACA/ANSI tooth form, face width auto-sized to the B29.1 single-strand
tooth. Pass the chain's pitch and roller diameter (e.g. #25: 6.35 / 3.302;
#35: 9.525 / 5.08). Keep `bore/2` well inside the root circle.

| param | type | required | meaning |
|---|---|---|---|
| `pitch` | number | yes | chain pitch P, mm |
| `roller_d` | number | yes | nominal roller diameter Dr, mm |
| `teeth` | int | yes | tooth count (≥ 6) |
| `bore` | number | yes | bore **diameter**, mm |

```json
{"ops": [{"id": "sprocket", "op": "chain_sprocket", "pitch": 6.35,
          "roller_d": 3.302, "teeth": 12, "bore": 5}]}
```

### `jaw_coupling_hub`
One hub of a GR-style jaw (spider) coupling: body cylinder, half-height centre
spigot, three 28° jaws on the 60° station grid (jaw 0 on +X), analytic bore.
Two hubs (the second flipped and rotated 60°) plus a `jaw_coupling_spider`
assemble to the size row's overall length with 2°/0.2 mm designed play —
proven by an assembled no-interpenetration test in the kernel. Genus 1.
Size rows are de-facto composites of the common aluminium-coupling listings
(documented in the kernel): OD 20 (L25), 25 (L30), 30 (L35), 40 (L50).

| param | type | required | meaning |
|---|---|---|---|
| `od` | number | yes | body outer **diameter** (20, 25, 30, 40) |
| `bore` | number | yes | bore **diameter**, within the row's range (e.g. D25: 4-12) |

```json
{"ops": [{"id": "hub", "op": "jaw_coupling_hub", "od": 25, "bore": 8}]}
```

### `jaw_coupling_spider`
The elastomer star insert mating two `jaw_coupling_hub`s: centre ring wrapping
the spigots, six 30° legs, 0.1 mm thinner than the jaw band (axial float).
Flat-flanked envelope of the crowned commercial spider (documented). Genus 1.

| param | type | required | meaning |
|---|---|---|---|
| `od` | number | yes | coupling body **diameter** (20, 25, 30, 40) |

```json
{"ops": [{"id": "spider", "op": "jaw_coupling_spider", "od": 25}]}
```

### `set_screw_coupling`
One-piece set-screw rigid shaft coupling joining Ø`bore1` (entering at z = 0)
to Ø`bore2` (entering at z = L); stepped bores meet at the mid-plane. Four
radial DIN 916 set-screw tap-drill holes (two per shaft, 90° apart; threads
not modelled, project convention). All-planar prism construction so the
cross-drilled body exports STL on the exact route (documented trade: no
analytic surface tags). Genus 5. Stocked bores: 4, 5, 6, 6.35, 8, 10, 12;
body OD/L and screw size from the larger bore's row.

| param | type | required | meaning |
|---|---|---|---|
| `bore1` | number | yes | bore **diameter** at z = 0 (stocked size) |
| `bore2` | number | yes | bore **diameter** at z = L (stocked size) |

```json
{"ops": [{"id": "rigid", "op": "set_screw_coupling", "bore1": 5, "bore2": 8}]}
```

### `clamp_coupling`
One-piece slit clamp coupling: full-length axial slit on +X severs the bore
web; two DIN 912 cross screws (counterbored clearance bores along -Y at L/4
and 3L/4) clamp it shut. Far-lobe threads are not modelled (clearance Ø all
through, documented). Genus 4. Stocked bores: 4, 5, 6, 8, 10, 12.

| param | type | required | meaning |
|---|---|---|---|
| `bore1` | number | yes | bore **diameter** at z = 0 (stocked size) |
| `bore2` | number | yes | bore **diameter** at z = L (stocked size) |

```json
{"ops": [{"id": "clamp", "op": "clamp_coupling", "bore1": 8, "bore2": 10}]}
```

### `linear_bearing_lmuu`
LM-series linear ball-bearing envelope: the catalog tube with its two
retaining-ring grooves (one revolve; balls/races/seals not modelled —
documented). LM8UU 8 × 15 × 24 (grooves Ø14.3 × 1.1 at 17.5), LM12UU
12 × 21 × 30 (Ø19.9 × 1.3 at 23) — the de-facto rows every vendor stocks.
Genus 1.

| param | type | required | meaning |
|---|---|---|---|
| `bore` | number | yes | shaft **diameter**: 8 or 12 |

```json
{"ops": [{"id": "lm8uu", "op": "linear_bearing_lmuu", "bore": 8}]}
```

### `sc8uu_block`
SC8UU pillow-block envelope: 34 × 30 × 22 block, Ø15 bearing seat through at
the catalog 11 mm centre height (along +Y), four blind M4 platform taps on the
24 × 18 grid. Genus 1. No parameters.

```json
{"ops": [{"id": "block", "op": "sc8uu_block"}]}
```

### `shaft_support_sk8`
SK8 upright shaft support for Ø8 smooth rod: base 42 × 14 × 6 (two Ø5.5 holes
at ±16), tower to 32.8 with the rod bore at the catalog 20 mm centre height,
2 mm clamp slit and M4 cross screw. Genus 4. No parameters.

```json
{"ops": [{"id": "sk8", "op": "shaft_support_sk8"}]}
```

### `shaft_support_shf8`
SHF8 flange shaft support: 43 × 20 × 10 stadium plate, Ø8 bore on the plate
normal, Ø5.5 ear holes at ±16, slit + M4 clamp screw between the ears.
Genus 4. No parameters.

```json
{"ops": [{"id": "shf8", "op": "shaft_support_shf8"}]}
```

### `mgn12_rail`
HIWIN MGN12 profile-rail **envelope**: 12 × 8 bar along +Y with M3 countersunk
mounting holes on the catalog 25 mm pitch (pattern centred; the countersink is
a plane-faceted frustum so the STL export stays on the exact route —
documented in the kernel). The proprietary raceway profile is intentionally
not modelled. Genus = hole count.

| param | type | required | meaning |
|---|---|---|---|
| `length` | number | yes | rail length, mm (≥ 25) |

```json
{"ops": [{"id": "rail", "op": "mgn12_rail", "length": 200}]}
```

### `mgn12_carriage`
MGN12H carriage envelope: 45.4 × 27 block with the rail channel underneath and
four M3 platform taps on the 20 × 20 grid; riding an `mgn12_rail` puts the
platform at the catalog 13 mm assembly height (the kernel test poses the pair
and proves no interpenetration). Genus 0. No parameters.

```json
{"ops": [{"id": "carriage", "op": "mgn12_carriage"}]}
```

### `deep_groove_bearing`
Deep-groove ball-bearing **body**: the d × D × B annulus of the kernel's cited
seat table (the same designations `bearing_seat` cuts pockets for), bore along
+Z from z = 0, with shallow ring-split witness grooves on each face (display
convention — balls/cages/shields/chamfers not modelled, documented). Genus 1.
Drop it into a `bearing_seat` pocket of the same designation for
assembly/interference studies.

| param | type | required | meaning |
|---|---|---|---|
| `designation` | string | yes | "603", "608", "625", "688", "6000", "6001" or "6804" |

```json
{"ops": [{"id": "b608", "op": "deep_groove_bearing", "designation": "608"}]}
```

### `flanged_bearing`
Flanged miniature bearing body, flange face at z = 0 (drop into a plain bore;
the flange registers on the wall): F608 8 × 22 × 7 with flange Ø25 × 1.5, or
F623 3 × 10 × 4 with flange Ø11.5 × 0.6 (the standard vendor rows). Same
witness-groove display convention as `deep_groove_bearing`. Genus 1.

| param | type | required | meaning |
|---|---|---|---|
| `designation` | string | yes | "F608" or "F623" |

```json
{"ops": [{"id": "f", "op": "flanged_bearing", "designation": "F623"}]}
```

### `thrust_bearing`
Thrust ball-bearing body, 511 series (ISO 104 boundary dims): 51100 10 × 24 × 9
or 51101 12 × 26 × 9, modelled as one annular envelope with the washer-split
witness groove around OD and bore at mid-height (the two washers + ball cage are
one body — documented). Genus 1.

| param | type | required | meaning |
|---|---|---|---|
| `designation` | string | yes | "51100" or "51101" |

```json
{"ops": [{"id": "t", "op": "thrust_bearing", "designation": "51100"}]}
```

### `kp08_pillow_block`
KP08 pillow block envelope (de-facto zinc-alloy listing dims): base 55 × 13 × 6
with Ø5.5 bolt holes at ±21, housing boss Ø29, the Ø8 shaft bore through along
+Y at the catalog 15 mm centre height (overall 29.5 tall). The self-aligning
insert is not modelled. Genus 3. No parameters.

```json
{"ops": [{"id": "kp", "op": "kp08_pillow_block"}]}
```

### `pipe_boss_g`
G-series (ISO 228-1 / BSPP) pipe-thread port boss: round boss Ø `major + 2·wall`
× `length` along +Z, bored straight through at the standard **tap-drill Ø**
(8.8 / 11.8 / 15.25 / 19.0 for G1/8…G1/2) with a 45° mouth chamfer opening to
the thread major Ø; the flat mouth annulus outside the chamfer is the sealing
face. Union onto a tank/manifold wall and tap the thread — the helix is not
modelled (tap-drill convention, documented). Genus 1.

| param | type | required | meaning |
|---|---|---|---|
| `designation` | string | yes | "G1/8", "G1/4", "G3/8" or "G1/2" |
| `wall` | number | yes | radial wall beyond the major Ø, mm (≥ 1) |
| `length` | number | yes | boss length, mm (> chamfer + one pitch) |

```json
{"ops": [{"id": "port", "op": "pipe_boss_g", "designation": "G1/4", "wall": 2.5, "length": 12}]}
```

### `hose_barb`
Parametric hose-barb stem for a hose of inner Ø `hose_id`: base at z = 0 (union
it onto your boss or wall), bore Ø `0.6·hose_id` through, `barbs` sawtooth teeth
at the documented de-facto catalog proportions (crest 118 % of hose ID, gentle
ramp toward the tip, square retention shoulder toward the base; the first tooth
doubles as the 85 % tip lead-in). Genus 1.

| param | type | required | meaning |
|---|---|---|---|
| `hose_id` | number | yes | hose inner **diameter**, mm |
| `barbs` | integer | yes | sawtooth tooth count (≥ 1) |

```json
{"ops": [{"id": "barb", "op": "hose_barb", "hose_id": 6, "barbs": 3}]}
```

### `lead_screw_tr8`
Tr8 trapezoidal lead-screw body (DIN 103 / ISO 2904): the exact Ø8 envelope a
Tr8 screw sweeps, half-pitch entry chamfer, along +Z from z = 0. Leads 2
(1-start), 4 (2-start), 8 (4-start — the printer Z screw), all pitch 2. The
thread form is not modelled on the body (catalog convention); the DIN 103
numbers (d2 7.0, d3 5.5, D1 6.0, D4 8.5) are documented in the kernel's
`tr8_spec`, and the true helical trapezoidal ridge exists Rust-side
(`tr8_thread_ridge`) for voxel-fused showcase parts. Genus 0.

| param | type | required | meaning |
|---|---|---|---|
| `length` | number | yes | screw length, mm (> 2) |
| `lead` | number | yes | 2, 4 or 8 |

```json
{"ops": [{"id": "screw", "op": "lead_screw_tr8", "length": 300, "lead": 8}]}
```

### `lead_screw_nut_tr8`
The ubiquitous flanged Tr8 brass-nut envelope (de-facto listing dims, cited in
the kernel): body Ø10.2 × 15, flange Ø22 × 3.5 at z = 0, four Ø3.5 holes on a
Ø16 bolt circle, bore Ø8 (internal thread not modelled). Genus 5. No
parameters. Mate with `tr8_nut_trap`.

```json
{"ops": [{"id": "nut", "op": "lead_screw_nut_tr8"}]}
```

### `nema_motor`
Simplified NEMA stepper body for assembly/clearance work: chamfered-corner
square body below the faceplate (face at z = 0, body in −z), pilot register
boss and output shaft along +Z. NEMA ICS 16 frame dimensions (inch-converted,
cited in the kernel): N17 — face 42.3, bolts 31.0 square M3, pilot Ø22 × 2,
shaft Ø5 × 24; N23 — face 56.4, bolts 47.14 square M5, pilot Ø38.1 × 1.6,
shaft Ø6.35 × 21. Honest envelope: no wiring box, ribs or rear shaft. Genus 0.

| param | type | required | meaning |
|---|---|---|---|
| `frame` | int | yes | NEMA frame number (17 or 23) |
| `body_len` | number | yes | body length below the faceplate, mm (e.g. 40) |

```json
{"ops": [{"id": "motor", "op": "nema_motor", "frame": 17, "body_len": 40}]}
```

### `nema_mount_plate`
The minimal motor bracket: a square plate (`face + 2·margin` on a side) with
the NEMA pilot register bore (pilot + 0.2) and the four ISO 273 medium
clearance holes through it. Genus 5.

| param | type | required | meaning |
|---|---|---|---|
| `frame` | int | yes | NEMA frame number (17 or 23) |
| `thickness` | number | yes | plate thickness, mm |
| `margin` | number | yes | extra width beyond the motor face per side, mm (≥ 0) |

```json
{"ops": [{"id": "plate", "op": "nema_mount_plate", "frame": 17,
          "thickness": 5, "margin": 4}]}
```

### `shaft`
Plain Ø`d` shaft along +Z (base at the origin), optionally with a DIN 6885
form-A keyway slot (rounded ends) milled into its +X side, width/depth
auto-sized from the DIN 6885-1 table for `d` (table covers over 6 up to
75 mm). Keep `0 < offset` and `offset + length <` shaft length so the slot
stays a lateral pocket.

| param | type | required | meaning |
|---|---|---|---|
| `d` | number | yes | shaft **diameter**, mm |
| `length` | number | yes | shaft length, mm |
| `keyway` | `{length, offset}` | no | slot length / start offset along the axis, mm |

```json
{"ops": [{"id": "shaft", "op": "shaft", "d": 8, "length": 40,
          "keyway": {"length": 20, "offset": 5}}]}
```

### `parallel_key`
DIN 6885 form-A parallel key (round ends): a `b` × `h` × `l` bar lying flat on
z = 0, length along +X. Pass the table size for a shaft from `din6885` data
(e.g. Ø20 shaft → 6 × 6); the function builds exactly what is asked.

| param | type | required | meaning |
|---|---|---|---|
| `b` | number | yes | key width, mm |
| `h` | number | yes | key height, mm |
| `l` | number | yes | overall length incl. the semicircular ends, mm (keep `l > b`) |

```json
{"ops": [{"id": "key", "op": "parallel_key", "b": 6, "h": 6, "l": 25}]}
```

### `dowel_pin`
ISO 2338 parallel dowel pin: Ø`d` × `length` with the standard ~15° insertion
chamfers (0.2·d) at both ends. Diameters 1, 1.5, 2, 2.5, 3, 4, 5, 6, 8, 10,
12; `length` must exceed the two chamfers. Genus 0.

| param | type | required | meaning |
|---|---|---|---|
| `d` | number | yes | pin **diameter**, mm (table size) |
| `length` | number | yes | overall length, mm |

```json
{"ops": [{"id": "pin", "op": "dowel_pin", "d": 6, "length": 24}]}
```

### `circlip_external`
DIN 471 external retaining ring for a nominal Ø`shaft_d` shaft, drawn in its
installed state (seated at the groove Ø from the table) with the two pliers
lugs. Genus 2 (the lug holes). Sizes Ø8, 10, 12, 15, 20, 25, 30. Cut the
matching groove with `circlip_groove_external`.

| param | type | required | meaning |
|---|---|---|---|
| `shaft_d` | number | yes | nominal shaft **diameter**, mm (table size) |

```json
{"ops": [{"id": "clip", "op": "circlip_external", "shaft_d": 20}]}
```

### `circlip_internal`
DIN 472 internal retaining ring for a nominal Ø`bore_d` bore (lugs reach
inward). Genus 2. Sizes Ø16, 20, 22, 26, 32, 35, 42, 47. Cut the matching
groove with `circlip_groove_internal`.

| param | type | required | meaning |
|---|---|---|---|
| `bore_d` | number | yes | nominal bore **diameter**, mm (table size) |

```json
{"ops": [{"id": "clip", "op": "circlip_internal", "bore_d": 32}]}
```

### `flat_head_screw`
ISO 10642 countersunk (flat-head) socket screw body: 90° conical head to the
table Ø, hex socket pocket cut in. `length` is overall (tip to head top) and
must contain the head cone + socket. Sizes M3–M16. Pairs with
`countersink_hole`.

| param | type | required | meaning |
|---|---|---|---|
| `m` | number | yes | nominal thread size |
| `length` | number | yes | overall length, mm |

```json
{"ops": [{"id": "fhs", "op": "flat_head_screw", "m": 5, "length": 16}]}
```

### `button_head_screw`
ISO 7380 button-head socket screw body: spherical-cap head over a Ø`m` shank,
hex socket through the crown. `length` is the under-head shank. Sizes M3–M12.

| param | type | required | meaning |
|---|---|---|---|
| `m` | number | yes | nominal thread size |
| `length` | number | yes | under-head shank length, mm |

```json
{"ops": [{"id": "bhs", "op": "button_head_screw", "m": 5, "length": 16}]}
```

### `set_screw`
DIN 916 cup-point set screw (grub screw): headless body with the hex socket in
the top face and the cup recess (modelled as a 120° conical recess at the
table's mouth Ø) in the bottom. Sizes M3–M12; `length` must hold cup + socket
+ a 0.5 mm web.

| param | type | required | meaning |
|---|---|---|---|
| `m` | number | yes | nominal thread size |
| `length` | number | yes | overall length, mm |

```json
{"ops": [{"id": "grub", "op": "set_screw", "m": 6, "length": 10}]}
```

### `lock_nut`
DIN 985 nyloc (nylon-insert) lock nut body: hex wrench section + insert collar
with crown chamfer, bored through. Note the DIN widths (M10 → 17 AF, M12 →
19 AF — wider than ISO 4032). Sizes M3–M16. Genus 1.

| param | type | required | meaning |
|---|---|---|---|
| `m` | number | yes | nominal thread size |

```json
{"ops": [{"id": "nyloc", "op": "lock_nut", "m": 10}]}
```

### `threaded_rod`
Metric threaded rod (studding, DIN 976-1 style): Ø`m` body with half-pitch 45°
end chamfers. The thread itself is not modelled (catalog bodies are the exact
assembly envelopes). Sizes M3–M16 (ISO 261 coarse table); any length above the
two chamfers.

| param | type | required | meaning |
|---|---|---|---|
| `m` | number | yes | nominal thread size |
| `length` | number | yes | rod length, mm |

```json
{"ops": [{"id": "stud", "op": "threaded_rod", "m": 8, "length": 60}]}
```

### `standoff`
Female–female hex standoff (spacer) at the conventional wrench size for `m`
(M2 → AF 4 … M6 → AF 10), bored through at the nominal Ø (internal thread not
modelled). Sizes M2, M2.5, M3, M4, M5, M6. Genus 1.

| param | type | required | meaning |
|---|---|---|---|
| `m` | number | yes | nominal thread size |
| `length` | number | yes | standoff length, mm |

```json
{"ops": [{"id": "spacer", "op": "standoff", "m": 3, "length": 12}]}
```

### `shoulder_bolt`
ISO 7379 hexagon-socket shoulder screw: thread tip at z = 0 (stem at the thread
major Ø, helix not modelled), the ground Ø`shoulder_d` shoulder of your ordered
`shoulder_len`, the table's socket head on top (socket cut to k/2 — the
standard's socket-depth column is not reproduced; documented). Sizes are the
standard's distinctive 6.5 / 8 / 10 / 13 / 16 (threads M5–M12). Genus 0.

| param | type | required | meaning |
|---|---|---|---|
| `shoulder_d` | number | yes | shoulder **diameter**: 6.5, 8, 10, 13 or 16 |
| `shoulder_len` | number | yes | ground shoulder length, mm |

```json
{"ops": [{"id": "pivot", "op": "shoulder_bolt", "shoulder_d": 8, "shoulder_len": 20}]}
```

### `spring_washer`
DIN 127 B spring (split) lock washer: the b × s rectangular section swept one
turn minus a 15° split gap around the d1 bore, rising one thickness so the free
height is 2·s and the split ends stand open (gap/rise are documented
conventions; section and diameters are the standard's). Sizes M3–M12. Genus 0
(split ring).

| param | type | required | meaning |
|---|---|---|---|
| `m` | number | yes | nominal thread size: 3, 4, 5, 6, 8, 10 or 12 |

```json
{"ops": [{"id": "lock", "op": "spring_washer", "m": 5}]}
```

### `compression_spring`
Compression spring: round wire (16-gon section) swept along a helix, plain
open ends, body seated on z = 0. Refused when coils would touch
(`pitch ≤ wire_d`) or `outer_d ≤ 2·wire_d`. Genus 0.

| param | type | required | meaning |
|---|---|---|---|
| `wire_d` | number | yes | wire **diameter**, mm |
| `outer_d` | number | yes | coil outside **diameter**, mm |
| `pitch` | number | yes | axial advance per turn, mm (> `wire_d`) |
| `turns` | number | yes | active turns (may be fractional) |

```json
{"ops": [{"id": "spring", "op": "compression_spring", "wire_d": 2,
          "outer_d": 16, "pitch": 6, "turns": 5}]}
```

### `extrusion_2020`
2020 V-slot aluminium extrusion stock: 20 × 20 mm, four 6 mm slots with 45° V
lips, Ø4.2 M5-tap core, along +Z. Honest simplified-but-dimensionally-correct
composite profile (sharp corners, no extrusion radii); the metal area lands on
the published ~0.48 kg/m. Genus 1 (the core bore).

| param | type | required | meaning |
|---|---|---|---|
| `length` | number | yes | stick length, mm |

```json
{"ops": [
  {"id": "rail", "op": "extrusion_2020", "length": 100},
  {"id": "v", "op": "volume", "in": "rail"}
]}
```

### `extrusion_3030`
3030 T-slot extrusion stock: 30 × 30 mm, 8 mm slots, Ø6.8 M8-tap core. Same
conventions and honesty notes as `extrusion_2020`.

| param | type | required | meaning |
|---|---|---|---|
| `length` | number | yes | stick length, mm |

```json
{"ops": [{"id": "rail", "op": "extrusion_3030", "length": 80}]}
```

### `tnut_2020`
2020-series M5 drop-in tee nut (9.5 × 2 flange, 5.9 neck, 10 long, Ø5 bore —
fits the `extrusion_2020` slot envelope; thread and retention dimple not
modelled). No parameters. Built flange-down on z = 0, bore along +Z. Genus 1.

```json
{"ops": [{"id": "tnut", "op": "tnut_2020"}]}
```

### `o_ring`
AS568 O-ring at its free nominal size: the exact analytic torus ID × W for the
dash number, axis +Z. Supported dashes: 010, 012, 014, 016, 018, 020 (W 1.78);
110, 112, 115, 120 (W 2.62); 210, 214, 218, 222 (W 3.53); 325 (W 5.33).
Genus 1. Cut the mating shaft gland with `o_ring_groove`.

| param | type | required | meaning |
|---|---|---|---|
| `dash` | int | yes | AS568 dash number (e.g. `214` for AS568-214) |

```json
{"ops": [{"id": "seal", "op": "o_ring", "dash": 214}]}
```

### `o_ring_cord`
Metric O-ring / glued-cord ring at its free nominal size: the exact analytic
torus, **any** inside diameter (housing lids outrun the AS568 table) with a
stocked metric cord cross-section. Genus 1. Cut the mating face gland with
`o_ring_face_gland` / `o_ring_face_gland_racetrack`. Stocked cords: Ø1, 1.5,
1.78, 2, 2.5, 2.62, 3, 3.53, 4, 5, 5.33, 6, 7.

| param | type | required | meaning |
|---|---|---|---|
| `ring_id` | number | yes | ring inside **diameter**, mm (free) |
| `cord_d` | number | yes | cord cross-section **diameter**, mm (stocked size) |

```json
{"ops": [{"id": "seal", "op": "o_ring_cord", "ring_id": 150, "cord_d": 3}]}
```

### `gear_rack`
ISO 53 / DIN 867 basic-rack gear rack — straight flanks at the pressure angle
(the exact involute limit): pitch π·m, addendum m, dedendum 1.25·m, whole
teeth only (pattern centred, root-level lands at the ends). Bar along +X from
x = 0, teeth +Y, extruded `width` along +Z; back face y = 0, **pitch line
y = 3·m** (section height 4·m). Root/tip fillets left sharp (documented).
Refused when no whole tooth fits or the pressure angle is outside (0°, 32°).

| param | type | required | meaning |
|---|---|---|---|
| `module` | number | yes | module `m`, mm |
| `length` | number | yes | bar length, mm |
| `width` | number | yes | face width (extrusion), mm |
| `pressure_angle_deg` | number | no (20) | pressure angle in degrees |

```json
{"ops": [{"id": "rack", "op": "gear_rack", "module": 2, "length": 100,
          "width": 10}]}
```

### `internal_gear`
Internal (ring) gear: involute tooth spaces cut into the bore of a rim of
outer Ø `rim_od` — tip circle m(z − 2), root circle m(z + 2.5). Exact
conjugate of a `spur_gear` pinion of the same module/angle at centre distance
`(z_ring − z_pinion)·m/2` (verified by a meshed no-interference test in the
kernel). Genus 1. Refused when the rim is thinner than the root circle,
`teeth < 8`, or the root land pinches shut (high pressure angles). Tip fouling
for small tooth differences (≲ 10) is not checked.

| param | type | required | meaning |
|---|---|---|---|
| `module` | number | yes | module `m`, mm |
| `teeth` | int | yes | ring tooth count (≥ 8) |
| `face_width` | number | yes | axial width, mm |
| `rim_od` | number | yes | rim outer **diameter**, mm (> m·(teeth + 2.5)) |
| `pressure_angle_deg` | number | no (20) | pressure angle in degrees |

```json
{"ops": [{"id": "ring", "op": "internal_gear", "module": 2, "teeth": 36,
          "face_width": 8, "rim_od": 84}]}
```

## Standard feature cuts

Catalog-driven features machined into a prior solid (same spirit as the hole
wizard): `at` / `axis` place the cut, the standard's table supplies every
dimension. Grooves use the proven lathe-style transverse ring cutter; the host
solid should be the part's **nominal** diameter for the standard (oversized
stock is outside the cutters' clearance envelopes — each op documents its
envelope).

### `heatset_insert_boss`
Grow a heat-set insert boss on a printed part: a boss (Ø 2× pilot) out of the
face at `at` along the outward normal `axis`, with the correctly **undersized**
insert pocket (the Ruthex pilot-drill table — e.g. M3 → Ø4.0 pocket, NOT the
thread Ø) bored back down, melt-pool room below. Sizes M2, M2.5, M3, M4, M5,
M6. The boss base must land fully on the host face.

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid |
| `at` | `[x,y,z]` | yes | boss centre on the host face |
| `axis` | `[x,y,z]` | yes | outward face normal |
| `m` | number | yes | insert thread size |

```json
{"ops": [
  {"id": "lid", "op": "box", "min": [0,0,0], "max": [30,30,6]},
  {"id": "boss", "op": "heatset_insert_boss", "in": "lid", "at": [15, 15, 6],
   "axis": [0, 0, 1], "m": 3}
]}
```

### `circlip_groove_external`
Cut the DIN 471 circlip groove into a shaft: root Ø `d2` and width `m` from
the table for the nominal Ø`shaft_d`, spanning `[at, at + m·axis]` (`at` on
the shaft axis). Designed for a shaft at the nominal Ø; the cutter clears a
Ø(`shaft_d` + 4) envelope. Sizes as `circlip_external`.

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid (the shaft) |
| `at` | `[x,y,z]` | yes | groove start point on the shaft axis |
| `axis` | `[x,y,z]` | yes | shaft axis direction |
| `shaft_d` | number | yes | nominal shaft **diameter**, mm (table size) |

```json
{"ops": [
  {"id": "axle", "op": "shaft", "d": 20, "length": 40},
  {"id": "grooved", "op": "circlip_groove_external", "in": "axle",
   "at": [0, 0, 32], "axis": [0, 0, 1], "shaft_d": 20}
]}
```

### `circlip_groove_internal`
Cut the DIN 472 circlip channel into a bore wall: root Ø `d2` (> bore) and
width `m` from the table for the nominal Ø`bore_d`, spanning
`[at, at + m·axis]` (`at` on the bore axis). The annular cutter machines only
the bore wall. Sizes as `circlip_internal`.

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid (the housing) |
| `at` | `[x,y,z]` | yes | channel start point on the bore axis |
| `axis` | `[x,y,z]` | yes | bore axis direction |
| `bore_d` | number | yes | nominal bore **diameter**, mm (table size) |

```json
{"ops": [
  {"id": "block", "op": "box", "min": [-20,-20,0], "max": [20,20,20]},
  {"id": "bored", "op": "drill", "in": "block", "at": [0, 0, 20],
   "axis": [0, 0, -1], "d": 16, "through": 20},
  {"id": "grooved", "op": "circlip_groove_internal", "in": "bored",
   "at": [0, 0, 6], "axis": [0, 0, 1], "bore_d": 16}
]}
```

### `o_ring_groove`
Cut an AS568 static O-ring gland into a shaft (male/piston gland): root Ø =
the dash's nominal ID, depth and width from the Parker O-Ring Handbook static
chart, spanning `[at, at + G·axis]`. Designed for the gland's nominal shaft
Ø = ID + 2·depth (e.g. dash 214 → Ø30.63 shaft); the cutter clears that Ø + 4.
Wall draft and corner breaks are omitted (documented in the kernel). Dashes as
`o_ring`.

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid (the shaft/piston) |
| `at` | `[x,y,z]` | yes | groove start point on the shaft axis |
| `axis` | `[x,y,z]` | yes | shaft axis direction |
| `dash` | int | yes | AS568 dash number |

```json
{"ops": [
  {"id": "piston", "op": "cylinder", "base": [0,0,0], "axis": [0,0,1],
   "radius": 15.315, "height": 30, "segments": 48},
  {"id": "gland", "op": "o_ring_groove", "in": "piston", "at": [0, 0, 12],
   "axis": [0, 0, 1], "dash": 214}
]}
```

### `o_ring_face_gland`
Cut a circular **face-seal (axial)** O-ring gland into a flat face: an annular
channel of centreline Ø `gland_center_d`, depth/width from the metric-cord
gland table (25% squeeze / 75% fill — mid-band of Parker's static
recommendations; the chart publishes inch sections only, so the metric rows
are derived, honestly documented in the kernel). `at` is the gland centre ON
the face, `axis` the outward normal; the groove sinks into the material.
Measures echo what the table chose: `gland_depth`, `groove_width`, `squeeze`,
`fill`, and `cord_length` (= π·gland_center_d, the centreline circumference).
Cords as `o_ring_cord`. Refused for unstocked cords or a centreline tighter
than the groove width.

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid (the housing/boss) |
| `at` | `[x,y,z]` | yes | gland centre on the face |
| `axis` | `[x,y,z]` | yes | outward face normal |
| `gland_center_d` | number | yes | channel centreline **diameter**, mm |
| `cord_d` | number | yes | cord cross-section **diameter**, mm |

```json
{"ops": [
  {"id": "boss", "op": "cylinder", "base": [0,0,0], "axis": [0,0,1],
   "radius": 25, "height": 10, "segments": 48},
  {"id": "gland", "op": "o_ring_face_gland", "in": "boss", "at": [0, 0, 10],
   "axis": [0, 0, 1], "gland_center_d": 36, "cord_d": 2}
]}
```

Report measures: `{"gland_depth": 1.5, "groove_width": 2.7925,
"squeeze": 0.25, "fill": 0.75, "cord_length": 113.097}`.

### `o_ring_face_gland_racetrack`
The FRICTION-bred lid seal: a **racetrack** (rounded-rectangle) face-seal
gland for rectangular housings — centreline `x_len × y_len` with corner
radius `corner_r` (≥ half the groove width), centred at `at` on the face,
sunk along `-axis`; depth/width from the metric-cord table as
`o_ring_face_gland`. Corners are 12-segment arcs. Measures echo the gland
dimensions plus `cord_length` (the centreline perimeter — the cord cut
length). Refused when the corners don't fit the sides.

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid (the lid) |
| `at` | `[x,y,z]` | yes | racetrack centre on the face |
| `axis` | `[x,y,z]` | yes | outward face normal |
| `x_len` | number | yes | centreline overall length, face-frame x, mm |
| `y_len` | number | yes | centreline overall length, face-frame y, mm |
| `corner_r` | number | yes | centreline corner radius, mm |
| `cord_d` | number | yes | cord cross-section **diameter**, mm |

```json
{"ops": [
  {"id": "lid", "op": "box", "min": [-60, -40, 0], "max": [60, 40, 6]},
  {"id": "gland", "op": "o_ring_face_gland_racetrack", "in": "lid",
   "at": [0, 0, 6], "axis": [0, 0, 1], "x_len": 100, "y_len": 60,
   "corner_r": 8, "cord_d": 2}
]}
```

Report measures: `{"gland_depth": 1.5, "groove_width": 2.7925,
"squeeze": 0.25, "fill": 0.75, "cord_length": 306.265}`.

### `tr8_nut_trap`
The printed-carriage pocket for the flanged Tr8 nut: Ø10.6 through-bore (nut
body + screw passage), flat-bottomed Ø22.4 × 3.7 flange recess sunk into the
face, and four M3 ISO 273 medium clearance holes on the Ø16 bolt circle —
0.4/0.2 mm designed clearance to `lead_screw_nut_tr8` (the kernel test seats
the nut in the trap and proves zero interpenetration).

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid (the carriage) |
| `at` | `[x,y,z]` | yes | screw axis on the face |
| `axis` | `[x,y,z]` | yes | outward face normal |
| `through` | number | yes | material span, mm (> 3.7) |

```json
{"ops": [
  {"id": "carriage", "op": "box", "min": [-25, -25, 0], "max": [25, 25, 10]},
  {"id": "trap", "op": "tr8_nut_trap", "in": "carriage", "at": [0, 0, 10],
   "axis": [0, 0, 1], "through": 10}
]}
```

### `nema_mount_cut`
Machine a NEMA motor mount into any face: the pilot register through-bore
(pilot Ø + 0.2) plus the four ISO 273 medium clearance holes on the frame's
bolt square, all through `through` mm of material. The bolt square aligns to
the face frame of `axis` (`perp_basis`; for ±Z that is the world X/Y).

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid |
| `at` | `[x,y,z]` | yes | motor axis position on the face |
| `axis` | `[x,y,z]` | yes | outward face normal |
| `frame` | int | yes | NEMA frame number (17 or 23) |
| `through` | number | yes | material span the holes cut through, mm |

```json
{"ops": [
  {"id": "bracket", "op": "box", "min": [-30, -30, 0], "max": [30, 30, 6]},
  {"id": "mount", "op": "nema_mount_cut", "in": "bracket", "at": [0, 0, 6],
   "axis": [0, 0, 1], "frame": 17, "through": 6}
]}
```

### `servo_pocket`
Drop-in hobby-servo mount: the rectangular case cutout (case + 0.4 mm fit)
plus the ear-screw pilot holes, through `through` mm of panel. Long side along
the face frame's first axis. Models (datasheet dims cited in the kernel):
`"sg90"` (23.0 × 12.2 case, 2 pilots Ø1.8 at 27.5), `"mg996r"` (40.7 × 19.7,
4 pilots Ø3.5 on 49.5 × 10). The wire-exit notch is not cut (vendor-specific).

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid (the panel) |
| `at` | `[x,y,z]` | yes | pocket centre on the face |
| `axis` | `[x,y,z]` | yes | outward face normal |
| `model` | string | yes | `"sg90"` or `"mg996r"` |
| `through` | number | yes | material span the pocket cuts through, mm |

```json
{"ops": [
  {"id": "panel", "op": "box", "min": [-40, -20, 0], "max": [40, 20, 4]},
  {"id": "mount", "op": "servo_pocket", "in": "panel", "at": [0, 0, 4],
   "axis": [0, 0, 1], "model": "sg90", "through": 4}
]}
```

### `pc4_port`
Push-fit pneumatic port (the bowden/airline standard): the fitting's
flat-bottomed tap-drill pocket — Ø5.0 × 6 for PC4-M6 (M6×1), Ø9.0 × 7 for
PC4-M10 (M10×1) — plus the Ø4.2 tube-pass bore continuing through the rest of
the material, so the 4 mm OD tube seats straight through. Tap the fine thread
in the pocket; the helix is not modelled. Adds one tunnel (genus +1).

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid |
| `at` | `[x,y,z]` | yes | port centre on the face |
| `axis` | `[x,y,z]` | yes | outward face normal |
| `m` | number | yes | fitting thread: 6 or 10 |
| `through` | number | yes | total material depth, mm (> pocket depth) |

```json
{"ops": [
  {"id": "manifold", "op": "box", "min": [-30, -15, 0], "max": [30, 15, 10]},
  {"id": "port", "op": "pc4_port", "in": "manifold", "at": [0, 0, 10],
   "axis": [0, 0, 1], "m": 6, "through": 10}
]}
```

### `teardrop_hole`
The printable horizontal hole: a Ø`d` bore through `through` mm whose crown
continues as two 45° roof lines to a teardrop apex `√2·d/2` above centre along
the build direction `up` — no overhang past 45°, no supports. The bore keeps
the exact nominal circle over its lower 270° (pins still locate on it); the
teardrop only adds clearance above. Hole-wizard conventions: `at` on the entry
face, `axis` INTO the material. Adds one tunnel (genus +1).

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid |
| `at` | `[x,y,z]` | yes | hole centre on the entry face |
| `axis` | `[x,y,z]` | yes | drilling direction, into the material |
| `up` | `[x,y,z]` | yes | build direction (not parallel to `axis`) |
| `d` | number | yes | bore **diameter**, mm |
| `through` | number | yes | material span, mm |

```json
{"ops": [
  {"id": "wall", "op": "box", "min": [-10, -5, 0], "max": [10, 5, 30]},
  {"id": "axle", "op": "teardrop_hole", "in": "wall", "at": [0, 5, 15],
   "axis": [0, -1, 0], "up": [0, 0, 1], "d": 8, "through": 10}
]}
```

### `board_mount`
One-call board mounting pattern: the published clearance-hole positions for a
Raspberry Pi B-family board (4 × M2.5, 58 × 49 from (3.5, 3.5) on the vendor
drawing origin = board bottom-left corner), an Arduino Uno R3 (4 × M3 at the
reference-drawing inch-grid positions, same corner datum), or a VESA FDMI
MIS-D square (75 × 75 / 100 × 100 M4, measured from the pattern centre — the
NUC/monitor standard). Holes cut through all material. Face-frame caveat:
`axis` +Z maps pattern x/y to world (X, Y); −Z gives (X, −Y), so the
corner-anchored rpi/arduino patterns mirror in y on a top face — cut from the
underside or account for it. Genus +4 on a plate.

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid (the panel) |
| `at` | `[x,y,z]` | yes | pattern datum on the face (see above) |
| `axis` | `[x,y,z]` | yes | drilling direction, into the material |
| `board` | string | yes | "rpi", "arduino_uno", "vesa75" or "vesa100" |

```json
{"ops": [
  {"id": "panel", "op": "box", "min": [0, 0, 0], "max": [130, 130, 4]},
  {"id": "pi", "op": "board_mount", "in": "panel", "at": [20, 20, 0],
   "axis": [0, 0, 1], "board": "rpi"}
]}
```

### `bridged_counterbore`
The printable counterbore: the DIN 974-1 pocket for an M-`m` cap screw, with
the ISO 273 medium clearance bore started only `bridge` mm below the pocket
floor — a thin sacrificial membrane bridges the pocket ceiling flat so it
prints without supports, and you **drill the membrane out afterwards**. The
as-printed solid is intentionally NOT a through hole: genus is unchanged (the
kernel test asserts genus 0 against the hole wizard's genus 1). Sizes M2–M12.

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid |
| `at` | `[x,y,z]` | yes | hole centre on the entry face |
| `axis` | `[x,y,z]` | yes | drilling direction, into the material |
| `m` | number | yes | nominal screw size (2–12) |
| `through` | number | yes | total material depth, mm (> pocket + bridge) |
| `bridge` | number | yes | membrane thickness, mm (one layer, e.g. 0.25) |

```json
{"ops": [
  {"id": "plate", "op": "box", "min": [-15, -15, 0], "max": [15, 15, 10]},
  {"id": "cb", "op": "bridged_counterbore", "in": "plate", "at": [0, 0, 10],
   "axis": [0, 0, -1], "m": 5, "through": 10, "bridge": 0.3}
]}
```

## Design-math lookups

Pure cited-table/closed-form calculations — they bind no geometry (referencing
their id is a `missing_ref` error, like any measure op); the numbers come back
in `measures`.

### `gt2_belt`
Size the belt of a two-pulley GT2 2 mm drive: exact pitch-line loop length at
the given centre distance, plus the commercial belt size as the nearest whole
tooth. Measures: `pitch_length` (mm), `belt_teeth`. Refused when the pitch
circles are not strictly separated.

| param | type | required | meaning |
|---|---|---|---|
| `center_distance` | number | yes | pulley centre distance, mm |
| `t1` | int | yes | first pulley tooth count (≥ 2) |
| `t2` | int | yes | second pulley tooth count (≥ 2) |

```json
{"ops": [{"id": "belt", "op": "gt2_belt", "center_distance": 100,
          "t1": 20, "t2": 20}]}
```

Report: `{"pitch_length": 240.0, "belt_teeth": 120}`.

### `gt2_center_distance`
Inverse of `gt2_belt`: the exact centre distance at which a closed belt of
`belt_teeth` teeth runs taut on the two pulleys. Measures: `center_distance`
(mm). Refused when the belt is too short to wrap the pulleys.

| param | type | required | meaning |
|---|---|---|---|
| `belt_teeth` | int | yes | belt tooth count (pitch length = 2·teeth mm) |
| `t1` | int | yes | first pulley tooth count (≥ 2) |
| `t2` | int | yes | second pulley tooth count (≥ 2) |

```json
{"ops": [{"id": "c", "op": "gt2_center_distance", "belt_teeth": 120,
          "t1": 20, "t2": 20}]}
```

### `iso286_fit`
Resolve an ISO 286 hole-basis **preferred fit** to numeric limits for a
nominal diameter ≤ 120 mm. Supported fits: `"H7/g6"`, `"H7/h6"`, `"H7/k6"`,
`"H7/n6"`, `"H7/p6"`, `"H7/s6"`, `"H8/f7"` (case-insensitive). Measures:
`hole`, `shaft`, `clearance` — each `[lower, upper]` deviations from the
nominal in mm; negative clearance is interference.

| param | type | required | meaning |
|---|---|---|---|
| `d` | number | yes | nominal diameter, mm (0 < d ≤ 120) |
| `fit` | string | yes | fit designation, e.g. `"H7/g6"` |

```json
{"ops": [{"id": "fit", "op": "iso286_fit", "d": 8, "fit": "H7/g6"}]}
```

Report: `{"hole": [0.0, 0.015], "shaft": [-0.014, -0.005],
"clearance": [0.005, 0.029]}`.

### `heatset_spec`
Heat-set insert table lookup (Ruthex M2–M6, sources cited in the kernel): the
pilot/pocket sizing a flush insert pocket needs — for when the boss grown by
`heatset_insert_boss` is not wanted (e.g. pockets sunk into a flange face: cut
them with `drill` at `pilot_d` × `pocket_depth`).

| param | type | required | meaning |
|---|---|---|---|
| `m` | number | yes | insert thread size: 2, 2.5, 3, 4, 5, 6 |

Measures: `m`, `pilot_d` (the undersized pocket Ø), `insert_length`,
`pocket_depth` (insert length + 1 mm melt room), `boss_d` (the 2×pilot rule,
if you do want a boss).

```json
{"ops": [{"id": "m4", "op": "heatset_spec", "m": 4}]}
```

Report: `{"m": 4.0, "pilot_d": 5.6, "insert_length": 8.1, "pocket_depth": 9.1,
"boss_d": 11.2}`.
### `metric_cord_gland`
The static face-seal gland for a stocked metric O-ring cord cross-section:
depth `0.75·d` (25% squeeze) and width `π·d/2.25` (75% fill) — the midpoints
of Parker's recommended static bands (derived, since the Parker charts publish
inch sections only; the kernel documents and tests the derivation). Measures:
`gland_depth`, `groove_width`, `squeeze`, `fill`. Stocked cords as
`o_ring_cord`.

| param | type | required | meaning |
|---|---|---|---|
| `cord_d` | number | yes | cord cross-section **diameter**, mm |

```json
{"ops": [{"id": "g", "op": "metric_cord_gland", "cord_d": 2}]}
```

Report: `{"gland_depth": 1.5, "groove_width": 2.7925, "squeeze": 0.25,
"fill": 0.75}`.

### `racetrack_cord_length`
Cord cut length for a racetrack (rounded-rectangle) seal path:
`2(x + y) − 8r + 2πr` over the centreline. Add the cord vendor's 1–2%
compression allowance yourself when cutting. Measures: `cord_length` (mm).
Refused when `2·corner_r` exceeds either side.

| param | type | required | meaning |
|---|---|---|---|
| `x_len` | number | yes | centreline overall length, x, mm |
| `y_len` | number | yes | centreline overall length, y, mm |
| `corner_r` | number | yes | centreline corner radius, mm (≥ 0) |

```json
{"ops": [{"id": "cord", "op": "racetrack_cord_length", "x_len": 100,
          "y_len": 60, "corner_r": 8}]}
```

Report: `{"cord_length": 306.265}`.

### `pipe_thread_g`
ISO 228-1 parallel pipe thread (G/BSPP) lookup: the cited thread row for a port
drawing or a `pipe_boss_g`. Measures: `major_d`, `tpi`, `pitch` (25.4/TPI),
`tap_drill_d` (mm).

| param | type | required | meaning |
|---|---|---|---|
| `designation` | string | yes | "G1/8", "G1/4", "G3/8" or "G1/2" |

```json
{"ops": [{"id": "g14", "op": "pipe_thread_g", "designation": "G1/4"}]}
```

Report: `{"major_d": 13.157, "tpi": 19.0, "pitch": 1.337, "tap_drill_d": 11.8}`.

## Hole wizard

Standard machining-style holes cut straight into a prior solid, with the real
ISO/DIN dimension tables hardcoded in the kernel (`kernel-brep/src/holes.rs`,
sources cited there). Shared conventions: `at` is a point on the entry face,
`axis` points **into** the material, depths are measured from `at` along
`axis`, all diameters are diameters. Cutters overshoot entry/exit faces by
0.5 mm so cuts never leave coplanar membranes; hole walls are faceted
(`segments`, default 32) but carry their exact analytic surface tags (which is
what `exact_volume` and STEP export read). Metric table: M2, M2.5, M3, M4, M5,
M6, M8, M10, M12 (countersinks start at M3). Out-of-table sizes and degenerate
axes/depths are loud `invalid_param`s.

### `drill`
Plain Ø`d` hole. Exactly ONE of `depth` (blind — ends in the standard 118°
drill-point cone) or `through` (the material span to pierce) is required.

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid |
| `at` | `[x,y,z]` | yes | entry point |
| `axis` | `[x,y,z]` | yes | drilling direction, into the material |
| `d` | number | yes | hole **diameter**, mm |
| `depth` | number | one of | full-diameter depth of a blind hole |
| `through` | number | one of | material span of a through hole |
| `segments` | int | no (32) | tool facet count |

Measures: `d`, `kind` (`"blind"`/`"through"`), `depth` + `point_depth` (the
118° point reaches this far) for blind, `through` for through.

```json
{"ops": [
  {"id": "block", "op": "box", "min": [0,0,0], "max": [30,20,10]},
  {"id": "pocket", "op": "drill", "in": "block", "at": [15, 10, 10],
   "axis": [0, 0, -1], "d": 6, "depth": 5}
]}
```

### `clearance_hole`
ISO 273:1979 clearance hole for an M-`m` screw — always cut through the
solid's **entire** extent along `axis` (a clearance hole passes the screw
through). E.g. M5 → Ø5.3 / 5.5 / 5.8 for fit `close` / `medium` / `coarse`.

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid |
| `at` | `[x,y,z]` | yes | entry point |
| `axis` | `[x,y,z]` | yes | hole direction |
| `m` | number | yes | nominal thread size |
| `fit` | `"close"` / `"medium"` / `"coarse"` | no (`"medium"`) | ISO 273 series |
| `segments` | int | no (32) | tool facet count |

Measures: `m`, `fit`, `clearance_d` (the table diameter actually cut).

```json
{"ops": [
  {"id": "plate", "op": "box", "min": [0,0,0], "max": [30,20,8]},
  {"id": "pass", "op": "clearance_hole", "in": "plate", "at": [15, 10, 8],
   "axis": [0, 0, -1], "m": 5}
]}
```

### `counterbore_hole`
ISO 273 clearance hole plus the DIN 974-1 counterbore that recesses a DIN 912
socket-head cap screw flush (e.g. M5 → Ø10 pocket, 5.8 deep). The counterbore
depth is measured from `at`, so put `at` on the entry face.

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid |
| `at` | `[x,y,z]` | yes | entry point on the face |
| `axis` | `[x,y,z]` | yes | hole direction |
| `m` | number | yes | nominal thread size |
| `fit` | string | no (`"medium"`) | clearance series |
| `segments` | int | no (32) | tool facet count |

Measures: `m`, `fit`, `clearance_d`, `counterbore_d`, `counterbore_depth` —
the DIN 974-1 row the cut used, so mating hardware can be posed without
reading the kernel tables (e.g. M4 → Ø8.0 × 4.8 deep).

```json
{"ops": [
  {"id": "plate", "op": "box", "min": [0,0,0], "max": [40,30,10]},
  {"id": "seat", "op": "counterbore_hole", "in": "plate", "at": [20, 15, 10],
   "axis": [0, 0, -1], "m": 5, "fit": "close"}
]}
```

### `countersink_hole`
ISO 273 clearance hole plus the DIN 74-1 form F 90° countersink that seats an
ISO 10642 countersunk screw flush (e.g. M5 → sink to Ø12.5 at the entry
plane). Sizes M3–M12 (form F starts at M3 — M2/M2.5 fail loudly).

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid |
| `at` | `[x,y,z]` | yes | entry point on the face |
| `axis` | `[x,y,z]` | yes | hole direction |
| `m` | number | yes | nominal thread size (≥ 3) |
| `fit` | string | no (`"medium"`) | clearance series |
| `segments` | int | no (32) | tool facet count |

Measures: `m`, `fit`, `clearance_d`, `countersink_d` (the DIN 74-1 form F
entry-plane diameter).

```json
{"ops": [
  {"id": "panel", "op": "box", "min": [0,0,0], "max": [40,30,6]},
  {"id": "flush", "op": "countersink_hole", "in": "panel", "at": [20, 15, 6],
   "axis": [0, 0, -1], "m": 5}
]}
```

### `tap_drill_hole`
Tap-drill pilot bore for an ISO **coarse** M-`m` thread: pilot Ø = `m − pitch`
(the 100%-thread tapping size, e.g. M6×1 → Ø5). Blind pilots end in the 118°
drill point. The thread itself is manufacturing detail and is **not**
modelled. Same `depth`/`through` rule as `drill`.

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid |
| `at` | `[x,y,z]` | yes | entry point |
| `axis` | `[x,y,z]` | yes | hole direction |
| `m` | number | yes | nominal thread size |
| `depth` | number | one of | full-diameter depth of a blind pilot |
| `through` | number | one of | material span of a through pilot |
| `segments` | int | no (32) | tool facet count |

Measures: `m`, `pitch`, `pilot_d` (= `m − pitch`), plus the same depth facts
as `drill`.

```json
{"ops": [
  {"id": "boss", "op": "box", "min": [0,0,0], "max": [20,20,16]},
  {"id": "pilot", "op": "tap_drill_hole", "in": "boss", "at": [10, 10, 16],
   "axis": [0, 0, -1], "m": 6, "depth": 12}
]}
```

### `bolt_circle`
Repeat ONE hole-wizard cut at `n` equally spaced positions on a bolt circle of
**diameter** `circle_d` (the drawing's BCD), centred at `center` in the plane
perpendicular to `axis`. `start_deg` offsets the first hole from the
deterministic in-plane reference direction (+X for a Z axis), increasing
right-handed about `axis`. The cut is pure data under `hole`, tagged by
`kind`:

| `hole.kind` | extra params | cut |
|---|---|---|
| `"drill"` | `d`, `depth` or `through` | plain bore |
| `"clearance"` | `m`, `fit?` | ISO 273 through clearance hole |
| `"counterbore"` | `m`, `fit?` | clearance + DIN 974-1 counterbore |
| `"countersink"` | `m`, `fit?` | clearance + DIN 74-1 form F 90° sink |
| `"tap_drill"` | `m`, `depth` or `through` | ISO coarse pilot bore |

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid |
| `center` | `[x,y,z]` | yes | bolt-circle centre |
| `axis` | `[x,y,z]` | yes | hole direction (into the material) |
| `circle_d` | number | yes | bolt-circle **diameter**, mm |
| `n` | int | yes | hole count (≥ 1) |
| `start_deg` | number | no (0) | first-hole angle, degrees |
| `hole` | object | yes | the repeated cut (see table above) |
| `segments` | int | no (32) | tool facet count |

Measures: `n`, `circle_d`, `start_deg`, and `hole` — the cut's echo including
its ISO/DIN table dimensions (same fields as the standalone hole ops).

```json
{"ops": [
  {"id": "plate", "op": "box", "min": [0,0,0], "max": [60,60,8]},
  {"id": "pattern", "op": "bolt_circle", "in": "plate", "center": [30, 30, 8],
   "axis": [0, 0, -1], "circle_d": 40, "n": 4, "start_deg": 45,
   "hole": {"kind": "counterbore", "m": 4}},
  {"id": "gate", "op": "assert", "in": "pattern", "genus": 4}
]}
```

### `bearing_seat`
Cut the seat for a standard deep-groove ball bearing: a flat-bottom pocket of
the bearing's outer Ø and width (nominal — press/slip allowance is the
caller's offset, e.g. via `iso286_fit`) plus a concentric **shoulder bore**
through the rest of the material at the mean `(d + D)/2`, which still seats
the outer ring on a `(D − d)/4` ledge. Supported designations: 603, 608, 625,
688, 6000, 6001, 6804.

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | a prior solid |
| `at` | `[x,y,z]` | yes | seat centre on the entry face |
| `axis` | `[x,y,z]` | yes | pocket direction (into the material) |
| `bearing` | string | yes | designation, e.g. `"608"` |
| `segments` | int | no (32) | tool facet count |

Measures: `bearing`, `bore_d`, `outer_d`, `width`, `pocket_d`, `pocket_depth`,
`shoulder_d` — e.g. `"608"` → Ø22 × 7 pocket, Ø15 shoulder bore.

```json
{"ops": [
  {"id": "wall", "op": "box", "min": [0,0,0], "max": [40,40,20]},
  {"id": "seat", "op": "bearing_seat", "in": "wall", "at": [20, 20, 20],
   "axis": [0, 0, -1], "bearing": "608"}
]}
```

## Modelled ISO threads

Real thread GEOMETRY (unlike `tap_drill_hole`, which only drills the pilot):
the ISO 68-1 basic profile swept on an exact helix, coarse pitches from the
ISO 261/262 table (M3, M4, M5, M6, M8, M10, M12, M16). The load-bearing truth
of this family: the ridge's root is deliberately buried P/4 below the minor Ø
so it overlaps the shank it fuses with, which means the **exact boolean
`union(body, ridge)` self-intersects and no planar arrangement can stitch it**.
The honest route to one printable solid is the voxel half — `export_threaded`
— which is why the ridge and the fuse are separate ops. The thread axis is
always **world +Z through the origin**: pose the body's shank/bore onto it
first.

### `thread_spec`
Measures-only ISO lookup — the numbers a design needs before any geometry.

| param | type | required | meaning |
|---|---|---|---|
| `m` | number | yes | nominal size (M3–M16, coarse) |

Measures: `pitch`, `h` (the ISO 68-1 fundamental triangle height (√3/2)·P),
`minor_d` (basic minor Ø = m − 1.25·h), `tap_drill_d` (= m − pitch). An
off-table size is `invalid_param` naming the supported sizes.

```json
{"ops": [
  {"id": "m6", "op": "thread_spec", "m": 6}
]}
```

### `thread_ridge`
The external thread ridge as an **exact, watertight B-rep solid** (closed,
manifold, genus 0 — it passes the bind gate like any solid): the ISO 68-1
profile planted in the axial plane at 96 helix stations per turn, so crests
sit exactly at the major Ø along the whole ridge (a rotation-minimising sweep
would precess). Give either `m` (coarse pitch from the table) or BOTH
`major_d` and `pitch` for a custom thread. Spans over 200 turns are refused
before any loft (allocation guard). Bind it for inspection/measurement — but
fuse it with `export_threaded`, never the exact `union` (see above).

| param | type | required | meaning |
|---|---|---|---|
| `m` | number | one of | nominal ISO size; exclusive with `major_d`+`pitch` |
| `major_d` | number | one of | crest **diameter** in mm; requires `pitch` |
| `pitch` | number | with `major_d` | thread pitch in mm |
| `z0` | number | no (0) | axial start of the ridge |
| `length` | number | yes | axial span (`length/pitch` turns, ≤ 200) |

Measures: `major_d`, `pitch`, `minor_d`, `z0`, `length`, `turns`.

```json
{"ops": [
  {"id": "ridge", "op": "thread_ridge", "m": 6, "length": 10},
  {"id": "check", "op": "assert", "in": "ridge", "valid": true, "genus": 0}
]}
```

### `export_threaded`
Fuse (external) or cut (internal) an ISO thread onto a bound body and write
the mesh — through the **voxel half**, the proven hybrid route for the
self-intersecting exact union. Accurate to `voxel`, never exact; the measures
state the route honestly.

- **External** (`internal: false`, the default): the body and ridge
  tessellations are merged as a soup and healed via the winding-number SDF
  (route `"voxel_healed"`). `volume_delta_vs_body` — measured against the body
  alone healed at the SAME voxel — is **asserted > 0**: a thread that adds no
  material fails `invalid_geometry` instead of shipping a bald stud (the
  in-tree regression's guard). A measured M6 × 11 fuse adds ≈ 44 mm³.
- **Internal** (`internal: true`): a male-profile ridge enlarged to crest
  Ø (m + 0.4) — 0.2 mm **radial** crest clearance — is voxel-subtracted from
  the bore wall (route `"voxel_implicit"`), and the delta is asserted < 0.
  This is a **print-practical approximation** of a female thread (the male
  form + clearance, sized for FDM tapping/forming), **NOT the ISO D1/D4 basic
  female form** — said here so nobody mistakes it for a gauge-accurate nut
  thread. A measured M6 × 8 cut into a Ø5 bore removes ≈ 49 mm³.

Deterministic guards, refused up front: `voxel` coarser than **pitch/6**
(`invalid_param` — the lattice would smear the crests into a smooth band; the
default is pitch/8), spans over 200 turns, and a body whose bounding box does
not even overlap the thread span (`invalid_param` naming the +Z placement
rule — a floating ridge would otherwise still "add volume" as its own shell).

| param | type | required | meaning |
|---|---|---|---|
| `in` | id | yes | the body (shank/bore on the +Z axis at the origin) |
| `m` | number | yes | nominal ISO size (M3–M16, coarse) |
| `z0` | number | no (0) | axial start of the threaded span |
| `length` | number | yes | axial span (`length/pitch` turns, ≤ 200) |
| `internal` | bool | no (false) | cut a female thread instead of fusing a male one |
| `voxel` | number | no (pitch/8) | lattice size in mm; > pitch/6 is refused |
| `file` | string | yes | output mesh path (`.stl` / `.3mf` by extension) |

Measures: `route`, `m`, `pitch`, `internal`, `voxel`, `triangles`,
`watertight`, `volume`, `volume_delta_vs_body`.

```json
{"ops": [
  {"id": "shank", "op": "cylinder", "base": [0,0,0], "axis": [0,0,1],
   "radius": 2.4588, "height": 12, "segments": 48},
  {"id": "stud", "op": "export_threaded", "in": "shank", "m": 6,
   "z0": 0.5, "length": 11, "file": "m6_stud.stl"}
]}
```

---

# A complete worked example: the flange

One revolved cross-section, six drilled bolt holes, validation, measurement,
two exports — the exact program checked in as the binding's end-to-end test
(`crates/kernel-api/tests/programs.rs::flange_program_end_to_end`).

```json
{"ops": [
  {"id": "body", "op": "revolve",
   "profile": [[10,0], [40,0], [40,7], [39,8], [10,8]], "segments": 64},
  {"id": "hole0", "op": "cylinder", "base": [30, 0, -1], "axis": [0,0,1], "radius": 3.5, "height": 10, "segments": 24},
  {"id": "cut0", "op": "difference", "a": "body", "b": "hole0"},
  {"id": "hole1", "op": "cylinder", "base": [15, 25.98076211353316, -1], "axis": [0,0,1], "radius": 3.5, "height": 10, "segments": 24},
  {"id": "cut1", "op": "difference", "a": "cut0", "b": "hole1"},
  {"id": "hole2", "op": "cylinder", "base": [-15, 25.98076211353316, -1], "axis": [0,0,1], "radius": 3.5, "height": 10, "segments": 24},
  {"id": "cut2", "op": "difference", "a": "cut1", "b": "hole2"},
  {"id": "hole3", "op": "cylinder", "base": [-30, 0, -1], "axis": [0,0,1], "radius": 3.5, "height": 10, "segments": 24},
  {"id": "cut3", "op": "difference", "a": "cut2", "b": "hole3"},
  {"id": "hole4", "op": "cylinder", "base": [-15, -25.98076211353316, -1], "axis": [0,0,1], "radius": 3.5, "height": 10, "segments": 24},
  {"id": "cut4", "op": "difference", "a": "cut3", "b": "hole4"},
  {"id": "hole5", "op": "cylinder", "base": [15, -25.98076211353316, -1], "axis": [0,0,1], "radius": 3.5, "height": 10, "segments": 24},
  {"id": "cut5", "op": "difference", "a": "cut4", "b": "hole5"},
  {"id": "check", "op": "validate", "in": "cut5"},
  {"id": "vol", "op": "volume", "in": "cut5"},
  {"id": "stl", "op": "export_stl", "in": "cut5", "file": "flange.stl", "tol": 0.01},
  {"id": "step", "op": "export_step", "in": "cut5", "file": "flange.step"}
]}
```

Actual report (trimmed to the measuring/export entries; the 13 build ops report
`{"id": ..., "ok": true}`):

```json
{
  "ok": true,
  "ops": [
    {"id": "check", "ok": true, "measures": {
      "closed": true, "manifold": true, "euler_characteristic": -12,
      "genus": 7, "shells": 1, "valid": true}},
    {"id": "vol", "ok": true, "measures": {"volume": 35687.93833732343}},
    {"id": "stl", "ok": true, "file": "out/flange.stl",
     "measures": {"route": "exact", "triangles": 10656, "watertight": true}},
    {"id": "step", "ok": true, "file": "out/flange.step"}
  ]
}
```

Genus 7 = the through-bore plus six bolt holes; `volume` is the faceted value
(64-gon body, 24-gon holes — within 0.006% of the closed form 35686 mm³), and
an `exact_volume` op on the same solid reports 35727.24 mm³ (Pappus body minus
six exact `π r² h` holes — the surface tags survive all six booleans).
