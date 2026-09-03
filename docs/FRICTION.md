# FRICTION.md — dogfood log: parametric 2-stage gearbox through public surfaces only

Context: a design engineer (not a kernel developer) built `gearbox/` end-to-end using
ONLY the public surfaces: the `kernel-api` CLI, JSON programs (API.md), and hand-authored
`.lmcpart` / `.lmcasm` files. No crate source was modified. Every papercut hit on the way
is logged here with severity and the exact JSON/evidence. Severities: **blocker** (no way
through this surface; workaround required), **major** (wrong/silent/missing behavior that
cost real time or risks a bad part), **minor** (papercut/doc gap).

Where a workaround exists it is named. The `gearbox/` campaign directory this log was
written against was **retired from the repository on 2026-09-03** (design campaigns now
live outside this checkout). Nothing it proved was lost: the 20 `.lmcpart` recipes are
kept as the pre-W6 backward-compatibility corpus in `crates/kernel-model/tests/fixtures/pre_w6_parts/`, and the assembly files, the
design-intent allowlist and the evidence programs are kept as the reference example in
`reference/assembly/` (which has its own README and `run_all.sh`). Paths below have been repointed at
those homes; a path still written as `gearbox/…` names a file that went with the
campaign and is called out as such.

## Disposition after the w6 friction pass (2026-06-11)

| # | severity | item | disposition |
|---|---|---|---|
| 1 | blocker | `.lmcasm` had no executable surface | **RESOLVED-w6** — `kernel-api asm` |
| 2 | major | assembly checks ignored B-rep-only parts | **RESOLVED-w6** |
| 3 | major | no general rigid pose op | **RESOLVED-w6** — `pose` |
| 4 | major | non-interference proofs exited 1 | **RESOLVED-w6** — `assert_disjoint` |
| 5 | major | measures recorded, never checked | **RESOLVED-w6** — `assert` family |
| 6 | major | spur-gear STL ships `voxel_healed` | **DEFERRED** — triangulator owner, tracked with #19 |
| 7 | major | the two grammars unequal | **PARTIAL** — op aliases + all docs done; Document features → catalog agent |
| 8 | major | `bearing_seat`/`bolt_circle` unreachable | **RESOLVED-w6 (ops)** — bearing catalog PART → catalog agent |
| 9 | minor | hole cuts reported no table dims | **RESOLVED-w6** |
| 10 | minor | no face-seal O-ring gland | **DEFERRED** — `parts/**`, catalog agent |
| 11 | minor | heat-set inserts boss-only | **PARTIAL** — `heatset_spec` op; pocket feature → catalog agent |
| 12 | minor | `iso286_fit` hole-basis only | **DEFERRED** — `parts/**`, catalog agent |
| 13 | minor | `load_part` resolved against `--out-dir` | **RESOLVED-w6** — program-relative |
| 14 | minor | `Transform.xform` serde undocumented | **RESOLVED-w6** |
| 15 | minor | gear bores carry no surface tags | **DEFERRED** — gear builders, catalog agent |
| 16 | minor | no n-ary union op | **RESOLVED-w6** — `union_all` |
| 17 | minor | `min_thickness` is corner noise | **RESOLVED-w6 (reporting)** — percentiles |
| 18 | note | what worked well | NOTE — kept as balancing evidence |
| 19 | major (found w6) | housing exact tessellation went leaky post-Wave-5 | **OPEN** — kernel tessellation, triangulator owner |
| 20 | major (cold-start audit) | edge features after booleans fragment witness resolution | **CLOSED 2026-07-30** — coalesce + provenance-preserving rebuild |
| 21 | minor (cold-start audit) | hole wizard has no edge-proximity awareness | **CLOSED 2026-07-29** — `holes::min_ligament` advisory echo |

---

## 1. [BLOCKER] `.lmcasm` has no executable public surface

> **STATUS: RESOLVED-w6.** `kernel-api asm <file.lmcasm> [--base-dir D]
> [--out-dir O] [--tol] [--voxel] [--window]` loads the file, re-solves mates
> (residual gated at 1e-6), writes the BOM (`bom.json`), one exact-routed STL
> per instance (route named per part), the merged assembled STL, every named
> state's STL, and runs the B-rep-aware contact scan — all as one `run`-shaped
> JSON report with the exit-0 contract (see API.md "The assembly surface").
> the `asmcheck` and gearbox-STL example workaround harnesses are retired, fully
> replaced; the pipeline (now `reference/assembly/run_all.sh`) drives `kernel-api asm` +
> `check_asm.py` (the design-intent allowlist, the only assembly-specific
> remainder).
> On the gearbox: 37 instances, residual ~1e-12, 52 designed contacts found,
> tightest must-clear gap 0.050 mm — identical to the retired harness.

The CLI is `kernel-api run <program.json>` only, and the op list has exactly one native
loader, `load_part` (.lmcpart). There is **no way to load, validate, solve, BOM, or
clearance-check an assembly file** through the CLI/JSON surface — the `.lmcasm` format
exists (kernel-model/src/format.rs, `load_assembly`, BOM, mates, states) but only Rust
callers can reach it. An assembly-centric product (this gearbox) cannot be machine-checked
end-to-end with the shipped binary.

Evidence: `kernel-api` usage string (`usage: kernel-api run <program.json> [--out-dir DIR]`);
API.md op table ("1 native-format loader"). A program op `{"op": "load_assembly"}` is
`unknown_op`.

Workaround (HISTORICAL — the former gearbox/tools/asmcheck directory is no longer in the tree; the
recipe is kept because it still works): a ~150-line read-only downstream crate consuming
the public Rust API (`kernel_model::format::load_assembly`) to verify the file loads, the
mates re-solve (residual 1.4e-12), the BOM groups, and contacts are as designed.

Suggestion: `kernel-api check-asm <file.lmcasm>` (load + residual + BOM + report JSON), or
`load_assembly` as an op binding instances as solids.

## 2. [MAJOR] Assembly clearance/interference APIs silently ignore B-rep-only parts

> **STATUS: RESOLVED-w6.** `Assembly::clearance`/`interferences` now mesh every
> instance through the exact-preferring route (B-rep documents tessellated
> analytically at an ⅛-voxel chord tolerance; `mesh_instance_exact` is the new
> public per-instance accessor), and `interference_volume`/`mesh_all`/`bounds`
> bridge B-rep-only documents through the winding-number `MeshSdf` instead of an
> empty mesh. Regression: `assembly_checks_see_brep_only_parts` (kernel-model).
> The gearbox's 52-contact class is discoverable through `kernel-api asm`
> (see #1); `reference/assembly/run_all.sh` asserts it.

`Assembly::clearance` / `interferences` / `interference_volume` mesh each instance through
the **implicit half** (`Instance::mesh` → `Document::evaluate` → SDF dual-contour). Every
feature that is B-rep-only — `CatalogPart` (gears, shafts, screws, pins), `ExtrudeSketch`,
`Hole`, `CirclipGroove`, `ORingGroove`, `HeatsetBoss` — evaluates to `None` on that path,
so the instance contributes an **empty mesh**: `clearance` returns `inf`, `interferences`
finds nothing, **silently**. That is precisely the parts a gearbox is made of.

Evidence (gearbox/out/asmcheck_report.txt): official API vs hand-rolled B-rep measurement
of the same posed pair:

```
g1p <-> g1w   official     inf   brep   0.050 mm
g1w <-> base  official     inf   brep   4.215 mm
```

Of the 52 real designed contacts in the assembly, the official scan finds 6 (the only
pairs whose parts are plain cylinder/boolean documents — bearings vs spacers).

Workaround (in `asmcheck`): `doc.evaluate_brep()` → `kernel_brep::tessellate_adaptive_tol`
→ transform by `instance.pose` → `Mesh::min_distance`, AABB-prefiltered pairwise scan.

Suggestion: route `Instance::mesh` through the hybrid helper (B-rep tessellation when the
document is B-rep-only), or at minimum return an error/flag instead of an empty mesh.

## 3. [MAJOR] No general rigid pose in the program surface (only `rotate_z` + `translate`)

> **STATUS: RESOLVED-w6.** New `pose` op: `{"op": "pose", "in": X, "rotate":
> {"axis": [x,y,z], "degrees": d, "center": [x,y,z]?}, "translate": [x,y,z]?}`
> — rotation about any axis through any point, then translation (exactly the
> `.lmcasm` instance pose form; chain two poses for composed rotations). The
> gearbox's `Rx(−90°)`-posed parts are now writable as programs:
> `reference/assembly/check_artifacts.json` poses the real spacers/bolts verbatim from the
> assembly file and proves them disjoint from the housing.

A gearbox's gears/shafts lie along Y; their assembly pose is `Rx(-90°)·Rz(phase)`. No op
can produce that orientation, so **posed multi-part interference checks cannot be written
as programs** — e.g. "load housing_base.lmcpart and the posed wheel, intersect". The gear
mesh checks had to be done in the gears' local XY frame, and the housing-vs-rotating-parts
check had to use axis-aligned `cylinder` envelope solids (whose `axis` parameter IS
general — the asymmetry is the trap: constructors take any axis, transforms don't).

Evidence: op list (5 features/transforms: fillet/chamfer/rim/translate/rotate_z); there is
no `rotate`/`rotate_x`/`pose` op (`unknown_op`).

Workaround: `programs/check_mesh_stage*.json` (gear-local frame), `check_envelopes.json`
(swept-envelope cylinders along Y), `tools/asmcheck` for true posed contacts.

Suggestion: a `transform` op taking the same 12-float affine the `.lmcpart` `Transform`
feature already uses.

## 4. [MAJOR] Proving NON-interference exits 1: empty boolean results are op failures

> **STATUS: RESOLVED-w6.** `assert_disjoint {a, b, min_clearance?, tol?}` is the
> exit-0 non-interference proof (measured surface distance must EXCEED
> `min_clearance`; the API.md example measures the two posed gearbox stage-1
> gears at the designed 0.050 mm flank gap). For tessellation-independent
> proofs the exact route is now also assertable in-program:
> `union` + `assert {"shells": 2}`. Empty booleans remain loud failures by
> design — the assertion ops are the intended way to state emptiness intent.
> `reference/assembly/programs/check_clash_expected_fail.json` stays in-tree as the
> documented historical evidence.

The natural check "these two posed gears must NOT intersect" is an `intersection` whose
EMPTY result is the pass condition — but an empty boolean is a loud `invalid_param`
**failure**, and "execution stops at the first failing op", so the check program exits 1
on success-of-intent and there is no way to continue past it.

Evidence — `reference/assembly/programs/check_clash_expected_fail.json` (kept in-tree as the
documented expected-fail program; exits 1 with):

```json
{"id": "clash", "ok": false, "error": {"kind": "invalid_param",
 "message": "op 'clash': intersection produced an empty solid — degenerate input, ..."}}
```

Workaround: union + `validate` instead — disjoint bodies keep separate shells, so
`union; validate; shells == 2` is an exit-0 no-contact proof (used by
`check_mesh_stage1/2.json` and `check_envelopes.json`). The report records `shells`, but
nothing *asserts* it — a wrong shell count still exits 0 (see item 5).

Suggestion: an `assert_disjoint {a, b}` measure op, or `"allow_empty": true` on booleans
binding an explicit empty + `"empty": true` measure.

## 5. [MAJOR] No assertions in programs: measures are recorded, never checked

> **STATUS: RESOLVED-w6.** New `assert` op (kind `assert_failed` on unmet
> intent, execution stops, exit 1): `volume_within {target, abs|percent}`,
> `exact_volume_within`, `genus`, `shells`, `closed`, `manifold`, `valid` — all
> present checks evaluated, every failure named with measured vs expected in
> one message, measured values echoed as measures on pass. The gearbox
> acceptance-style "shells == 2" greps are now in-program assertions
> (`reference/assembly/check_artifacts.json`).

Programs can *measure* (`validate`, `volume`, `wall_thickness`…) but cannot *assert*. The
gearbox acceptance ("genus == 6", "shells == 2", "volume within x%") lives in my runner
script's `grep`, outside the surface. An AI/CI loop wants the program itself to fail when
a measured value drifts.

Suggestion: optional `"expect"` block per measure op (e.g. `{"op": "validate",
"expect": {"genus": 6, "shells": 1}}`) failing the op on mismatch.

## 6. [MAJOR] Catalog spur gears export STL via `voxel_healed` (exact tessellation leaks)

> **STATUS: RESOLVED for gears (measured 2026-06-11 on post-w6 main).** The
> z=60 wheel now exports `route: exact` at 5 534 triangles; in the full
> `kernel-api asm` gearbox run 36/37 instances route exact. The one residual
> `voxel_healed` is `housing_base` — i.e. the situation in the original
> finding below has inverted, and what remains is exactly #19 (still open).
> Original finding kept below as the historical record.

Every `spur_gear` STL export reports `route: voxel_healed` — the exact adaptive
tessellation of the involute `extrude_with_holes` solid is not watertight, so the shipped
mesh is a 0.3 mm-voxel remesh: 331 624 triangles for the z=60 wheel (a flat-flanked prism
should be a few thousand), with flank fidelity bounded by the voxel, not the involute.
Honest routing and honestly reported — but the flagship catalog part shouldn't need the
fallback for a plain polygon extrusion.

Evidence — the run report for `p_gear_s1_wheel.json` (a campaign output, not committed):

```json
{"id": "stl", "measures": {"route": "voxel_healed", "triangles": 331624, "watertight": true}}
```

(housing_base, a 64-feature part, exports `route: exact` at 63 856 triangles.)

## 7. [MAJOR] The two authoring surfaces are unequal where it hurts

> **STATUS: PARTIALLY RESOLVED-w6 / rest DEFERRED.** Done on the op/doc side:
> `bore_d` (the Document field name) is now a serde alias on the
> `spur_gear`/`gt2_pulley`/`chain_sprocket` ops; API.md states the canonical
> conventions (degrees everywhere on the op surface, `bore` canonical with the
> alias) and now documents the Document `Sketch` schema (`{"a": i, "b": j}`
> segments), the radians-only `ExtrudeSketch.draft` caveat, and the
> `Transform.xform` 12-float layout, so nothing here requires reading kernel
> source anymore. DEFERRED (feature-tree/Document side, owned by the
> catalog/library agent, not the w6 friction pass): `CatalogPart::Shaft`
> keyway field, a `ParallelKey`/`CirclipExternal` catalog feature, a Document
> `Revolve` feature, and switching `ExtrudeSketch.draft` to degrees.

One engine, two public grammars — and capabilities don't line up. Hit while building:

- `shaft` **op** has a DIN 6885 form-A `keyway`; the Document `CatalogPart::Shaft` has
  none → shaft keyseats in `.lmcpart` had to be hand-cut Boxes (form B), and the matching
  `parallel_key` op has **no Document twin at all** → keys modeled as plain Boxes.
- `circlip_external` (the ring itself) is op-only → the DIN 471 clips appear in the BOM
  but cannot be instanced in the `.lmcasm` (their *grooves* are fine: `CirclipGroove` is a
  feature). STL exported via `programs/p_circlip_din471_8.json` instead.
- `revolve` / `sketch_revolve` are op-only; Documents have no revolve feature → bearing
  envelope and spacer tubes built as cylinder booleans.
- Document `ExtrudeSketch.draft` is **radians**; the op surface (`extrude_tapered
  .draft_deg`, all angles) is **degrees**. Same engine, two units.
- `spur_gear` op bore param is `bore`; Document field is `bore_d`.
- The op `sketch` JSON (segments as `[i, j]` pairs) and the Document `Sketch` serde
  (segments as `{"a": i, "b": j}`) are different schemas for the same concept; only the
  op one is documented in API.md — the Document one I had to read out of `sketch.rs`.

Evidence: compare `crates/kernel-model/tests/fixtures/pre_w6_parts/shaft_input.lmcpart` (Box keyway cuts) with the one-line
op form `{"op": "shaft", "d": 8, "length": 73, "keyway": {...}}`.

## 8. [MAJOR] `bearing_seat` and `bolt_circle` exist in the kernel but on no public surface

> **STATUS: RESOLVED-w6 (ops).** `bearing_seat {in, at, axis, bearing,
> segments?}` (603/608/625/688/6000/6001/6804; echoes pocket Ø/depth and
> shoulder Ø as measures) and `bolt_circle {in, center, axis, circle_d, n,
> start_deg?, hole: {kind, …}}` (hole as pure data: drill / clearance /
> counterbore / countersink / tap_drill) are JSON ops, documented in API.md
> with runnable examples. Still open from this item's tail: a rolling-bearing
> catalog PART (the envelope ring itself) — `parts/**`, catalog agent.

`kernel-brep/src/holes.rs` ships `bolt_circle(...)` and `bearing_seat(solid, at, axis,
"608", ...)` with a cited bearing table (603/608/625/688/6000/6001/6804) — exactly what a
gearbox housing needs — and neither is an op nor a Document feature. I rebuilt the 608
seat by hand (Ø22×7 pocket + Ø16 web bore from a boss) ×6, and the bolt pattern as 8
explicit holes. Relatedly there is no rolling-bearing catalog **part** on either surface
(the table exists for seats only), so the six 608s are hand-modelled envelope rings.

Evidence: `grep -c bearing kernel-api/src/interp.rs` → 0.

## 9. [MINOR] Hole-wizard cuts don't report the table dimensions they used

> **STATUS: RESOLVED-w6.** Every hole-wizard op now echoes its ISO/DIN table
> row as measures: `clearance_hole` → `clearance_d`; `counterbore_hole` → +
> `counterbore_d`, `counterbore_depth` (the M4 → Ø8.0 × 4.8 that cost a 0.4 mm
> screw float); `countersink_hole` → + `countersink_d`; `tap_drill_hole` →
> `pitch`, `pilot_d`; `drill` → blind `point_depth` (118° tip reach);
> `bolt_circle` echoes its repeated cut's row; `bearing_seat` echoes pocket and
> shoulder dimensions. Pinned by `hole_wizard_ops_echo_table_rows` (and the
> repeated-cut echo by `bolt_circle_and_bearing_seat_ops`).

`counterbore_hole` (and friends) bind the cut solid but attach **no measures** — not the
counterbore Ø, not its depth. To pose the DIN 912 screws seated on the counterbore floor I
needed DIN 974-1 depth for M4 (4.8 mm) and had to read `kernel-brep/src/holes.rs`. My
first guess (4.4) left all 8 screws floating 0.4 mm — caught only by the asmcheck contact
scan (the expected `bolt <-> lid` seatings were missing from the designed-contact list).

Suggestion: echo the spec as measures, like `iso286_fit` already does:
`{"clearance_d": 4.5, "counterbore_d": 8.0, "counterbore_depth": 4.8}`.

## 10. [MINOR] No face-seal O-ring gland; AS568-only and the table is too small for housings

> **STATUS: DEFERRED (w6).** Needs a new gland feature + metric-cord table in
> `kernel_model::parts` (`parts/**` is owned by the catalog agent, not the w6
> friction pass). The racetrack-groove workaround in `housing_lid.lmcpart`
> remains the documented route.

`o_ring_groove` cuts only radial/piston glands, and the largest supported dash (325) is
far below a ~150 mm housing perimeter; metric cord is not supported. The lid face seal had
to be a hand-built racetrack groove (two rounded-rect prisms differenced, 2.7 × 1.5 for a
Ø2 cord at ~25 % squeeze) — Parker chart numbers hand-copied, not table-driven.

Evidence: `crates/kernel-model/tests/fixtures/pre_w6_parts/housing_lid.lmcpart` features "groove ring outer/inner".

## 11. [MINOR] Heat-set inserts: boss-only, no pocket-only variant

> **STATUS: PARTIALLY RESOLVED-w6 / rest DEFERRED.** The Ruthex table is now
> queryable: `heatset_spec {m}` returns `pilot_d`, `insert_length`,
> `pocket_depth` (+1 mm melt room) and the 2×pilot `boss_d` rule, so flange
> pockets are `drill` cuts driven by table data instead of hardcoded numbers.
> DEFERRED: a true pocket-only feature variant (geometry feature in
> `parts/**`, catalog agent).

`heatset_insert_boss` always grows a boss out of the face. Inserts sunk flush into a flange
top face (the standard lid-screw joint on printed boxes) need a *pocket without the boss*;
the Ruthex pilot table (M4 → Ø5.6) lives in the kernel but is not queryable, so the 8
flange pockets are plain `drill` features with the pilot Ø hardcoded from reading
`inserts.rs`. (The boss op itself worked nicely for the 4 accessory bosses on the floor.)

## 12. [MINOR] `iso286_fit` covers 7 hole-basis fits only

> **STATUS: DEFERRED (w6).** Shaft-basis and bearing-class fits are new rows in
> `kernel_model::parts::fits` (`parts/**` is owned by the catalog agent, not
> the w6 friction pass). The H7/k6 + H7/h6 proxies stay documented in
> `programs/check_fits.json`.

No shaft-basis fits, no bearing-class fits (k5/j5, N7/P7 housings). Bearing seats were
documented with H7/k6 and H7/h6 as nearest proxies (`programs/check_fits.json`).

## 13. [MINOR] `load_part` resolves relative paths against `--out-dir`, not the program file

> **STATUS: RESOLVED-w6.** Through the CLI, relative `load_part` paths now
> resolve against the PROGRAM FILE's directory — programs are relocatable and
> the two native formats agree (`.lmcasm` `path` sources already resolved
> against the assembly's directory). Library callers keep the old behavior via
> `run_program` (base = out_dir) or state it explicitly with the new
> `run_program_with_input_base`. Pinned by
> `cli_load_part_resolves_against_program_dir`; API.md updated.

Documented in API.md, but it still reads as a trap: a program is not relocatable —
`gearbox/programs/*.json` must encode `../parts/...` knowing the runner's `--out-dir`.
Suggestion: resolve against the program file's directory (like `.lmcasm` `path` sources
resolve against the assembly's directory — the two native formats already disagree).

## 14. [MINOR] `Transform.xform` (Affine3A) serde format is undocumented

> **STATUS: RESOLVED-w6.** API.md's Native formats section now documents the
> 12-float column-major layout (`[x_axis·3, y_axis·3, z_axis·3,
> translation·3]`) with a worked example, the `.lmcasm` quaternion order
> `[x, y, z, w]`, and (per #7) the Document `Sketch` schema — extracted from a
> real gearbox part, not guessed.

The 12-float column-major layout (`[x_axis, y_axis, z_axis, translation]`) is discoverable
only from glam source; API.md's `.lmcpart` example shows no Transform. First authoring
attempt was a guess that happened to work; an AI without source access would brute-force
it. Same for quaternion order (`[x,y,z,w]`) in `.lmcasm` poses (that one IS shown in
format.rs docs).

## 15. [MINOR] Catalog gear bores/keyways carry no analytic surface tags

> **STATUS: DEFERRED (w6).** Fix belongs in the gear builders
> (`kernel_model::parts::gears` — tag the bore polygon's facets with their
> `Surface::Cylinder`); `parts/**` is owned by the catalog agent, not the w6
> friction pass.

`exact_volume` on the gears equals the faceted `volume` bit-for-bit (1398.90 == 1398.90)
while shafts recover π-exact (`3613.04 → 3623.27`): the gear's bore is a raw 48-gon
polygon, so STEP export and exact measures get a prism, not a cylinder. Minor fidelity
loss on the catalog's flagship part.

Evidence: `out/report_p_gear_s1_pinion.txt` (`vol == xvol`) vs `out/report_p_shaft_input.txt`.

## 16. [MINOR] No n-ary union / group op

> **STATUS: RESOLVED-w6.** `union_all {in: [ids…]}` folds any number of solids
> (≥ 2, loud otherwise); with `assert {shells: N}` an N-body disjointness
> check is two ops. Documented with a runnable example in API.md.

The swept-envelope clearance check unions 8 bodies through 7 chained binary `union` ops
with bookkeeping ids (`u0..u6`) — `programs/check_envelopes.json`. A `union_all {in:
[...]}` (or a `shells`-aware `group`) would make disjointness checks one op.

## 17. [MINOR] `wall_thickness.min_thickness` is corner noise in practice

> **STATUS: RESOLVED-w6 (reporting).** `wall_thickness` now reports
> `p05_thickness` and `median_thickness` over the finite per-triangle samples
> alongside the (documented-noisy) `min_thickness`; API.md names the
> percentiles + `thin_area` as the robust signals. Ray clamping itself
> (kernel-side sampling change) is not taken in the w6 pass.

On the housing it reports `min_thickness: 0.0024 mm` (sharp-corner oblique rays; API.md
does warn). The usable signal is `thin_area` against `flag_below`. Suggestion: clamp rays
to near-antiparallel surface pairs, or report a percentile alongside the min.

Evidence: `out/report_p_housing_base.txt` (`min_thickness 0.0024`, `thin_area 2156` —
the 2156 mm² is real: the M3 boss walls are 2.0 < my 2.4 flag, by table design).

## 18. [NOTE] What worked better than expected (for balance)

> **STATUS: NOTE (no action needed).** Kept as the balancing evidence.

- Document-side booleans on a 64-feature housing (drafted sketch extrudes, 6 boss unions,
  9 transformed-cylinder cuts, 10 hole-wizard features, 4 heat-set bosses): first try,
  genus 6 exactly as predicted, `route: exact` STL, valid STEP.
- `extrude_tapered`/`ExtrudeSketch` accept negative heights (sweep down from the parting
  plane) — undocumented but exactly what drafted housings want.
- Disjoint `union` keeps shells separate and `validate` reports `shells` — the workable
  no-contact proof (items 4/5 notwithstanding).
- The zero-backlash ISO 53 tooth convention (tooth 0 centred on +X, keyway on +X) made
  mesh phasing pure arithmetic, and the measured flank gap (0.050 mm at ΔC = +0.15 mm)
  matches involute theory (half the normal backlash, 2·ΔC·tanα·cosα/2 ≈ 0.051) to 1 µm.
- `.lmcasm` round-trip: 37 instances, 4 mates re-solved on load to residual 1.4e-12, BOM
  grouped correctly (spacer variants split by their `len` parameter).

## 19. [MAJOR, found w6] housing_base exact tessellation regressed to leaky post-Wave-5

> **STATUS: OPEN (kernel tessellation — outside the w6 friction-pass ownership;
> needs the triangulator owner).** Found while replacing `asmcheck` with the
> official `kernel-api asm` surface, dated 2026-06-10.

At dogfood time `housing_base` exported `route: exact` (63 856 triangles,
watertight). On current main its adaptive tessellation is **not watertight**
(46 954 raw triangles, `is_watertight() == false`), so the part now ships
`route: voxel_healed` (971 736 triangles) — honestly reported, but a fidelity
regression on the flagship dogfood part, bisected to the Wave-5
parameter-space-triangulator era kernel changes (the part file itself is
byte-identical; the retired byte-identical `asmcheck` binary reproduces the
same failure against today's kernel, so this is NOT a w6 scan change).

Observable consequence: the leaky region's crack triangles cross two corner
M4-insert pockets and three web bores, so any mesh-distance contact scan
(official or the retired harness) reads a phantom `0.0 mm` for
`base↔bolt_0/bolt_3/spacer9_0/spacer10_0/spacer21_0` while the EXACT boolean
proves clear air (bolt↔pilot 0.8 mm radial, spacer↔web 2 mm; probe: bolt-shank
∩ base = empty). Handling: `reference/assembly/check_asm.py` tolerates exactly these five
pairs as named artifacts, and `reference/assembly/check_artifacts.json` re-proves each
disjoint EVERY pipeline run through the exact layer (`pose` + `union` +
`assert shells == 2`) — if one ever truly touches, the pipeline fails.

## 20. [MAJOR, found by the cold-start audit] Edge features after booleans fragment the next witness resolution

> **STATUS: CLOSED 2026-07-30.** Both halves are done. The fragmentation half
> landed 2026-07-28 as `coalesce_coplanar` (plane groups merged across shared
> edges via a region-boundary half-edge walk; pads-on-plate 65 faces → 16,
> volume-exact). The residual half — the rebuild RESET provenance names, so the
> pass was finishing-only and could not sit mid-chain, which is exactly where
> this bug bites — closed 2026-07-30: `FaceName` provenance now survives both
> `coalesce_coplanar` and `recover_quadrics` (an unmerged face keeps its name
> exactly; a merged face inherits the lexicographically-least constituent name,
> policy documented on `FaceName`; edge curves re-attach when both endpoints
> survive). Pinned: 16/16 faces named after rebuild, **58 same-name
> fragmentation seams → 0**, corner edges that previously resolved to NOTHING
> re-resolve by name+witness to the same segment, and a boolean → coalesce →
> witness-fillet chain evaluates bit-identically across two runs with correct
> material removal (1.0793 mm³ vs the closed form 1.0730). The §8.4 workaround
> ("ease edges on primitives first, boolean last") is now advice, not a
> requirement.
>
> Honest remainder, and it is correct behaviour rather than a limitation: the
> names of fully-consumed interior fragments — and the non-least names of a
> multi-name merge — no longer resolve, because they name geometry that no
> longer exists as a face.

`chamfer_edge_near`/`fillet_edge_near` on a *boolean result*: the first call
succeeds, the **next** fails with *"not a straight edge between two
perpendicular planar faces"* — even for a clean corner nowhere near the cut.
Each chamfer/boolean re-tessellation fragments the flat side faces, so edge
resolution no longer finds one straight edge between two whole planar faces.
On a pristine box, four sequential corner chamfers work fine. Found by a
docs-only `claude -p` Opus session (2026-06-11) building a GoPro mount; it
called this its biggest time sink. Workaround (now in DESIGN_GUIDE §8.4):
ease edges on primitives first, boolean last. Engine fix direction: coplanar
face re-coalescing after booleans/features — same family as the open
"curved-face re-tagging after cuts" frontier item.

## 21. [MINOR, found by the cold-start audit] Hole wizard has zero edge-proximity awareness

An M4 countersink placed tangent to the plate edge and 0.25 mm from a prong
raised nothing — no warning measure, no failure. The cut is honest geometry;
the silence is the trap (a thin-wall break would only surface in
`wall_thickness` or print). Tripwired in DESIGN_GUIDE §23 (wall_thickness
gate + volume window after every wizard cut). Fix direction: a
`min_ligament` measure in the wizard echo.

## 22. [NOTE, found building Studio Wave IDE-1] Kernel-surface gaps the IDE hit

Five findings from wrapping the kernel in a server, one positive:
1. Catalog-built `.lmcpart` recipes bake every dimension as a `Literal` —
   `shaft_input.lmcpart` has zero `Dim`s; only spacers/keys carry params.
   Catalog emitters should surface their natural parameters as Dims.
2. No machine-readable param schema for `kernel_model::parts` — Studio's
   34-family catalog is hand-curated against API.md prose, kept honest only
   by instantiate-from-defaults tests. Export the spec tables.
3. `Document` lacks a public feature iterator (kind/label/suppressed);
   Studio reads the serde JSON form instead. Workable; an accessor is cleaner.
4. `OpReport.file` is the absolute resolved path; servers strip the out-dir
   prefix to build URLs. An out-dir-relative field would help.
5. `run_program` has no per-op progress callback — chat tool calls block
   until a whole program finishes; streaming per-op status needs a hook.
6. POSITIVE: `Document::export_mesh(tol) -> (Mesh, RouteReport)` was exactly
   the right one-call surface for honest viewport meshes.

## #23 — notch-sliver boolean overlap mis-stitches (checked ops refuse) — 2026-07-02

Isolated repro (`kernel-brep/tests/recovery_needle_weld.rs::notch_sliver_overlap_refuses_honestly`):
a dovetail-notched plate overlapping a bowtie key as two disjoint 0.4-wide
parallel-flank sliver strips. The arrangement mis-stitches the A/B fragments in
every op (union/difference/intersection); `validate` catches it and every
`try_*` REFUSES — honest failure, no silent garbage. The SAME joint geometry
embedded in the real drawer_system module shell (big profile, key piercing the
front face) resolves correctly and is asserted numerically by the example's
retention gates on every run. Found while gating DOVESTACK dovetail retention.
Related fix landed at the same time: `Mesh::weld` now drops collapsed needle
triangles (a boolean-recovery zero-area sliver face previously double-counted
its long edge after welding — non-manifold mesh from a valid B-rep; see
`kernel-core/tests/weld_collapse.rs`).

---

# Open frontier — detailed notes (2026-07-29)

Still open:
- **Tangent/coincident planar-face-on-a-curved-wall degeneracy (2026-06-19, repro
  `tests/keyed_pulley_acceptance.rs`):** a planar box face placed EXACTLY tangent
  to a cylindrical wall (e.g. a keyway slot starting at y = bore radius) is a
  coincident-face degeneracy the planar arrangement can't resolve — refused by
  `try_difference`. Same class as the sub-tolerance near-coplanar cut. NOTE: this
  is a degenerate placement, NOT a practical pulley bug — a REAL keyway overlaps
  the bore (cut into it), and a realistic keyed V-pulley with lightening holes
  builds cleanly (corrects an earlier overstatement that called it a keyed-bore +
  lightening-holes bug; the "4 holes" was a red herring — the tangent keyway left
  the topology fragile so a later hole tripped). The general coincident-face
  robustness lives in the fuzz-hardened `booleans.rs` — supervised pass.
- Edge-feature witness resolution after booleans (FRICTION #20, cold-start
  audit 2026-06-11): re-tessellation fragments planar faces; coplanar face
  re-coalescing LANDED 2026-07-28 (`coalesce_coplanar`) and its quadric
  generalization 2026-07-30 (`recover::recover_quadrics`) — both remain
  geometry-FINISHING passes (the rebuild resets provenance names), so
  mid-chain witness re-resolution is still the open half. The side wants
  are DONE: `bounding_box`/`measure_dimension` ops exist, and the
  hole-wizard `min_ligament` echo landed 2026-07-29 (FRICTION #21 closed).
- SSI seam snapping (`ssi.rs`) into booleans — CORRECTED 2026-07-12 (the previous
  "cut seams still land on chords" note was stale): DONE per BAR.md **Level 7 CLOSED**
  (Wave-4/5) — plane∩cylinder cut seams exact to 1e-15, quadric∩quadric seams snap to
  ≤1e-9 (was ~1.7e-2 chords) via parameter-space charts; cut curved fragments
  re-tagged. Residual only: the warp-aware bulge follow-up (named in BAR.md).
- Wired 2026-06-09: `rim_fillet_band` now drives `fillet_circular_rim` (exact torus
  rim fillets on non-primitive bosses, convex rims only); conic plane-sections are
  consumed by `section_curves_with_fallback` (polyline fallback for no-closed-form
  cuts); inertia is analytic for cylinder, cone AND sphere faces (lens
  second-moment corrections — `cylinder`/`cone`/`sphere_second_moment`; verified
  coarse==fine in `tests/mass_properties_analytic.rs`) — torus second moments are
  now analytic too (CORRECTED 2026-07-12; the previous "~18% off / tessellation-level"
  note was stale): `torus_lens_moments` (`validate.rs`), coarse==fine asserted in
  `tests/mass_properties_analytic.rs` and `tests/mass_properties_torus.rs`.
- STEP: trimmed NURBS faces, sphere poles/periodic regions, third-party
  exporter corpus. Assemblies DONE 2026-07-02 (export_step_assembly +
  import_step_assembly round-trip, volume-conserving). CORRECTED 2026-07-09
  (the previous note here was stale): the exporter has ALWAYS emitted analytic
  CYLINDRICAL/SPHERICAL/CONICAL/TOROIDAL_SURFACE entities, and same-surface
  facet COALESCING for planes + full-wrap cylinders landed 2026-07-03
  (0086f26 + b32047a, round-trip self-gated per solid, LMCAD_COAL_* env
  hatches). CORRECTED AGAIN 2026-07-30 (this note itself had gone stale, and
  the doc auditor did not catch it because a prose "in progress" is not a
  mechanical claim — a human read it): cone-frusta coalescing + CIRCLE/surface
  entity dedup **SHIPPED 2026-07-09** (`ee063f6`), not "in progress". Measured
  against the true pre-change parent (`618fce5`, built in a throwaway worktree,
  byte-identical measurement code both sides): revolve-heavy synthetic
  **1 466 032 → 381 245 B = 3.85× bytes, 29 763 → 7 830 entities = 3.80×**
  (CONICAL_SURFACE 1024 → 4 per-facet → per-band); round-trip re-import volume
  conserved to **3.94e-11 rel**. On cyclo26 the coalescing-only contribution is
  **1.75× bytes / 1.78× entities** — BELOW the 2–4× expectation, cause recorded:
  153 band groups whose boolean-rotated rims have no angle-matched seam
  vertices. (The old "12.0 MB / 231k" baseline is not comparable to today's
  10.7 MB — the exporter is unchanged since `ee063f6`, so the cyclo26 MODEL
  grew.) Pinned by `kernel-brep/tests/step_size.rs` with raise-only floors.
  Remaining headroom, genuinely open: rim-arc chain merging (256 chord arcs →
  2 per half-rim) changes exported vertex topology and would break edge-pairing
  with the adjacent disc face — topology surgery, not a coalescing tweak. Also
  open: sphere caps; torus stays faceted (own importer refuses partial-turn
  tori — widening that is unsanctioned importer work). Post-change the file is
  **topology-dominated and at its irreducible floor** for a faceted-boundary
  B-rep (1538 VERTEX_POINT = 6 profile rings × 256 + 2 poles).


---

## #24 — `valid` + `watertight` + every dimensional gate can all pass on a part that is in TWO PIECES — 2026-07-31

**Severity: major.** Found building the DRILL HOOK campaign
(`drill_hook.rs`, then under `crates/kernel-model/examples/`, parked since 2026-09
in `legacy/kernel-model-examples/`).

The hook is a prism with a tapered cutter through its cradle: a hexagonal
prism whose two ends ramp inward at 46.6° so the grip channel closes without
needing supports. An early draft truncated the part's end faces INSIDE the
cutter's apex, to avoid leaving a knife edge. The void therefore ran clean out
through both end faces, and the channel's front wall became a **separate
floating body**.

Everything green: `validate(&solid).is_valid()` = true, `is_watertight()` =
true, `volume()` returned a plausible number (it simply summed both lumps),
and the campaign's whole gate suite — support-free audit, shelf-band rock,
tool keep-out, insertion sweep, retention overlaps, contact pressure, the
closed-form stress sections, STEP round-trip — passed. The four-view render
looked like a hook.

`Solid::shell_count()` **also reports 1** for that geometry, so it is not the
oracle either (it counts B-rep shell records, not connected geometry).

Caught by eye, on the render, by noticing a gap between the arm and the
cradle in the top view — then confirmed with a union-find over the STL's
welded vertices: **2 components**.

**What closed it (campaign-side, and it should be the idiom):**

```rust
// legacy/kernel-model-examples/drill_hook.rs (then crates/kernel-model/examples/)
let parts_n = mesh_components(&tessellate_default(&hook));
gate("the hook is ONE connected body", parts_n == 1, format!("{parts_n} bodies"), &mut ok);
```

plus a negative control that rebuilds the truncated-ramp variant and asserts
it splits (it reports **2 bodies while `shell_count` still says 1** — that
contrast is the whole lesson, and the NC pins it).

**The general rule this earns:** connectivity is a SEPARATE oracle from
validity and watertightness, and no campaign that subtracts a tapered or
tapering cutter should ship without it. Any cutter whose cross-section shrinks
to a point must have that apex strictly INSIDE the material, with a stated
solid margin — `drill_hook.rs` now carries that as the named const `END_TIE`
with the failure written into its doc comment.

**Engine-side follow-up, not done here:** a `Mesh::component_count()` (or a
`connected` flag on `SupportFreeReport`-class reports) would make this a
one-liner for every campaign instead of a copied helper. The campaign-local
`mesh_components` is 20 lines of union-find and is the obvious thing to
promote under the rule-of-two.

---

## #25 — `overlap_volume` refuses at ONE offset while its neighbours resolve — 2026-07-31

**Severity: minor (characterised, worked around honestly).**

Sweeping a drill-housing keep-out box back toward the shelf in 4 mm steps and
taking `overlap_volume(&hook, &box)` at each:

```
dx  -6.0 ->     10.664      dx -18.0 ->  None  (NaN)
dx -10.0 ->  14058.284      dx -22.0 ->  48223.139
dx -14.0 ->  28170.284      dx -26.0 ->  51247.139
```

Only −18 refuses. At that offset the box's rear plane lands **1.0 mm** from
the hook's slot back face — outside §7.7's ~0.1 mm sliver band, so the usual
explanation does not cover it. Not reduced to a minimal repro (the arrangement
needs the full face neighbourhood, per the RESPOOL precedent).

Handled by choosing negative-control offsets that resolve (−6 for the tight
control, −10 for the loose one) and **recording the refusal in the campaign
source and in `analysis/DESIGN.md`** rather than silently stepping around it.
Repro: restore the drill-hook source from `legacy/kernel-model-examples/` into
`crates/kernel-model/examples/` (see the legacy README), then
`cargo run --release -p kernel-model --example drill_hook` with the
diagnostic loop restored around `body_keepout`.

---

## #26 — a structural tie the exact B-rep has, the FEA's voxel grid can lose — and that is a DESIGN signal, not just a solver artefact — 2026-07-31

**Severity: note (a useful heuristic, earned the hard way).**

After fixing #24 the DRILL HOOK's channel ramps closed inside the part, tied
by a solid slab outboard of the ramp apex. At a 2.5 mm slab the exact geometry
is unambiguously one connected body — but `ace_fea_runner.py` on a 2.0 mm
occupancy grid reported the hook **five times softer** (tip deflection 2.24 mm
vs 0.42 mm) and its peak stress doubled, because binary occupancy could not
resolve the tie. Widening the slab to 5.0 mm restored both.

The tell that it was not simply "coarse mesh" noise: the deliberately
UNDER-BUILT negative control (every member at 40 % thickness) came out
**stiffer and less stressed** than the shipped part. A negative control that
inverts is a signal that the shipped geometry, not the control, is wrong.

The rule taken from it, and now written into the campaign's `END_TIE` doc
comment: **a tie a voxel grid can lose is a tie a slicer's perimeters can lose
too.** Sizing structural ties at least ~2 cells of whatever grid will be used
to analyse them is cheap insurance, and the disagreement between the exact and
discretized models is worth treating as a design finding rather than a
modelling nuisance.

Two related solver behaviours from the same campaign, both correct and both
recorded: `ace_fea` **refuses to converge** at 1.6 mm and 3.0 mm voxels on
this geometry while 2.0 mm solves (`CG did not converge (Jacobi then AMG,
info=2000) … refusing an unconverged solution`) — the 5 mm lip falls under two
cells. The refusal is the right behaviour; the cost is that this campaign has
no grid-convergence pair and had to cross-check with two boundary-condition
idealisations instead, which is documented in its `ANALYSIS.md` rather than
skipped.

---

## #27 — `sweep_check` cannot see a STEADY interference that never produces a near pose — 2026-07-31

**Severity: minor (documented limitation, concrete repro).**

`SweepReport`'s doc already warns that vertex-sampled penetration can read 0.0
through a thin wall. This is the sibling case, and it is worth a named repro:
`sweep_check` evaluates its exact crossing test only on poses whose mesh
distance is under 0.05, so an interference that is present at **every** pose
and never produces a "near" transition is invisible to it.

Repro (DRILL HOOK negative control): a grip gauge 4 mm thicker than the
channel is lowered through 13 poses. It interferes by 1 mm per side at every
pose from the moment it enters. `sweep_check` reports **crossings = 0**;
`overlap_volume` on the seated pose reports **1680 mm³**.

The campaign gates that negative control on `overlap_volume` and prints the
sweep's verdict beside it so the blind spot is visible in the run log rather
than inferred. General rule, consistent with the existing guidance: a sweep is
for FREE-RUN proofs (`contacts == 0 && crossings == 0`); anything asserting
that something does NOT fit belongs on the exact oracle.
