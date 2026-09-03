# Numerical Contracts

The Level-9 requirement (BAR.md: "tolerance model, units, ranges, failure modes")
made explicit. Every claim below is sourced from code (`file:identifier`); nothing
here is aspirational. When code and this document disagree, the code is the bug —
update whichever is wrong, but never let them drift silently.

## Units

- **Millimetres, everywhere.** All lengths — coordinates, tolerances, voxel
  sizes, chord tolerances — are model units, and the model unit is mm
  (`kernel-core/src/math.rs:EPSILON` "Default linear tolerance (mm)";
  `kernel-brep/src/booleans.rs:EPS` "in model units (mm)").
- The native file formats make the unit a **contract field**: a `.lmcpart` /
  `.lmcasm` whose `units` is missing or not `"mm"` is *refused* at load, not
  silently rescaled (`kernel-model/src/format.rs` — "refusing beats silently
  mis-scaling").
- Angles are radians throughout (`std::f64::consts` use across builders).
- Mass properties follow: volumes are mm³, inertia mm⁵·density.

## Coordinate ranges

- **B-rep (f64) working range: |x| ≲ 1e7 mm un-centred; beyond that, booleans
  re-centre automatically.** The arrangement's absolute tolerances (`EPS` 1e-9,
  `WELD_EPS` 1e-7) fall below the f64 ulp once coordinates reach ~1e8 (ulp(1e8)
  ≈ 1.5e-8), so coincidence tests would collapse. `boolean()` measures the
  operands' bbox centre and, when `|centre| > 1e7`, translates both operands to
  the origin, runs the arrangement in full precision, and shifts the kept
  fragments back (`kernel-brep/src/booleans.rs:154`). In place (`centre =
  ZERO`) the translate is an exact no-op, so near-origin results are
  byte-identical to the un-centred path.
- **Implicit/voxel (f32) honest range: ~1e6 mm is already degraded.** The f32
  SDF path suffers catastrophic cancellation at metre scale: a point 0.03 mm
  outside a 1e6 mm sphere rounds ONTO the surface in f32, while the `distance64`
  path recovers it — this is pinned, with numbers, by
  `kernel-implicit/src/lib.rs::f64_distance_is_correct_and_precise_at_large_scale`.
  Mesh vertex positions are f32 (`kernel-core/src/mesh.rs:Mesh::positions`), so
  meshing precision at coordinate magnitude `X` is ~`X · 1e-7` (f32 ulp);
  `Mesh::weld` computes its spatial-hash keys in f64 specifically "to stay exact
  at large coordinate magnitudes" (`kernel-core/src/mesh.rs:weld`).
- Practical guidance encoded by those two facts: model parts near the origin in
  mm; assemblies that must live far away are translated by the f64 pose layer
  (`kernel-model` `Instance::pose`), not by baking offsets into geometry.

## Tolerance model

One table, from code. "Absolute" means mm, not relative.

| constant | value | governs | source |
|---|---|---|---|
| `EPS` | 1e-9 | boolean arrangement: on-plane / coincidence tests; degenerate-triangle filter (`area ≤ EPS²`) | `kernel-brep/src/booleans.rs:57` |
| `WELD_EPS` | 1e-7 | stitching boolean fragments back into a half-edge solid (vertex identification) | `kernel-brep/src/booleans.rs:59` |
| `TJUNCTION_EPS` | `4·WELD_EPS` = 4e-7 | T-junction healing: a vertex this close to another face's edge is inserted into that edge; the sliver filter in `stitch` MUST use the same value or a thin triangle folds its own boundary (the R2/R3 non-manifold seed — see the comment) | `kernel-brep/src/booleans.rs:66` |
| `EPSILON` (f32) | 1e-5 | default linear tolerance for geometric comparisons on the mesh/voxel side | `kernel-core/src/math.rs:16` |
| `SURFACE_EPSILON` (f32) | 1e-6 | SDF zero-crossing tests along a grid edge | `kernel-core/src/math.rs:19` |
| `EPS` (solver) | 1e-12 | assembly constraint solver: directions/displacements below this are treated as zero | `kernel-model/src/constraints.rs:48` |
| winding threshold | 0.5 | `MeshSdf` inside/outside decision (generalized winding number > ½ ⇒ inside) | `kernel-implicit/src/meshsdf.rs:Sdf::distance` |
| `BETA` | 2.0 | fast-winding far-field gate: a BVH node is multipole-approximated only when `dist > BETA · node radius`; with the order-2 (Jacobian + Hessian) moments the in-tree test bound is 1e-2 (sphere probe measures 6.4e-3; a 4000-query hex-nut B-rep probe touched 1.01e-2 at a far-field point where winding ≈ 0 — sign decisions are unaffected because near-surface nodes fail the gate and recurse to exact leaves; measured numbers in `BENCH.md`) | `kernel-implicit/src/meshsdf.rs:BETA` |
| `REFINE_FRAC` | 0.05 | triangles with a longest edge above this fraction of the mesh diagonal are bisected before BVH build (geometry-identical; keeps node radii part-relative) | `kernel-implicit/src/meshsdf.rs:REFINE_FRAC` |

**Predicates are not epsilon-based.** Orientation signs (`orient2d`, `orient3d`,
`incircle`) are **exact**: a fast f64 evaluation guarded by Shewchuk's
conservative error bounds (`CCWERRBOUND_A = (3 + 16ε)ε`, `O3DERRBOUND_A`,
`ICCERRBOUND_A`, with ε = 2⁻⁵³) falls back to exact floating-point expansions
when the filter cannot certify the sign
(`kernel-core/src/predicates.rs:25-36`). Everything downstream that needs a
*consistent* side-of decision (ear clipping, arrangements, hulls) uses these,
so a sign can be slow but never wrong.

**Chord tolerances are parameters, not constants**: `tessellate_adaptive_tol(solid,
tol)` takes the chord tolerance in mm (`kernel-brep/src/tessellate_adaptive.rs:82`);
`precise_mesh(solid, tol)` prefers the exact path and falls back to a voxel heal at
`(tol · 20).clamp(0.1, 0.5)` mm (`kernel-model/src/lib.rs:precise_mesh`).

## f32 / f64 split

The convention is declared at the top of the shared math module
(`kernel-core/src/math.rs:5-8`) and holds across the workspace:

- **f64 (`DVec3`) — the exact B-rep side**: all of `kernel-brep` (topology,
  arrangements, booleans `booleans.rs:50`, SSI, NURBS, validation, mass
  properties, STEP), because coincidence decisions at 1e-9 on ~1e2-mm parts need
  ~1e-11 relative precision.
- **f32 (`Vec3`) — the implicit/voxel side**: SDF evaluation, voxel grids, mesh
  vertices (`kernel-core/src/sdf.rs:Sdf::distance -> f32`,
  `mesh.rs:Mesh::positions`), for memory and speed across multi-million-sample
  grids.
- **Deliberate f64 islands inside the f32 side**, where accumulation or
  cancellation would otherwise destroy the result:
  - `Sdf::distance64` (`kernel-core/src/sdf.rs:30`) — opt-in f64 evaluation for
    large-coordinate queries; every primitive implements it
    (`kernel-implicit/src/primitives.rs`).
  - The winding number accumulates solid angles in f64 and stores all BVH
    moments in f64 (`kernel-implicit/src/meshsdf.rs:tri_solid_angle`,
    `BvhNode`), because a watertight verdict is a SUM of ~1e5 signed terms.
  - `Mesh::weld` hashes positions via f64 keys (`kernel-core/src/mesh.rs:weld`).
  - Signed volume / mass-property reductions are f64 (`kernel-core/src/mesh.rs:
    signed_volume`).
  - An f64 mesher exists for geometry that must be sampled in f64 end-to-end
    (`kernel-core/src/mesher_f64.rs`).

## Determinism guarantees

What is **bit-stable run-to-run and process-to-process** (same platform/libm):

- **The boolean pipeline.** Pure f64 geometry with no iteration-order
  dependence: the three historical `HashMap`/`HashSet` order leaks
  (`cancel_coincident` drain order, `recover_faces` region order,
  `boundary_loops` pinch successors) were fixed (R5) and the regression is
  pinned by `kernel-brep/tests/determinism.rs` — a flange built by revolve ∪
  rim-filleted boss minus 7 drills, rebuilt 40×, asserted **bit-identical**
  (volume bits and topology counts).
  **Threaded since 2026-07-30, byte-identical BY CONSTRUCTION**: co-refine
  and fragment classification run as chunked pure flat-maps
  (`kernel_core::par::par_flat_map_chunks` — chunk boundaries are a pure
  function of item count, never of scheduling; results concatenate in
  ascending chunk order; per-item float sequences unchanged), everything
  order-sensitive (the whole stitch phase) stays sequential. Control:
  `LMCAD_BREP_THREADS` (unset = on, `1` = the identical code on one
  thread); nested-pool guard keeps booleans sequential inside
  `kernel_core::par` worker threads. Pinned by
  `kernel-brep/tests/threading_parity.rs` (11-case corpus, full canonical
  dumps byte-identical across schedules) and a threaded 40× variant in
  `determinism.rs`. Heavy booleans measure ~1.3–2.2× (stitch is the
  sequential Amdahl floor — stated, not hidden).
- **Mesh repair.** `Mesh::fill_holes` walks boundary edges in triangle order,
  never `HashSet` order, so identical input yields the identical repaired mesh
  (`kernel-core/src/mesh.rs:fill_holes` doc note).
- **The meshers, despite rayon.** Dense Surface Nets / DC / Manifold DC fan
  per-cell work across threads but assign vertex ids in a serial phase in cell
  order ("Vertex ids are assigned in Phase B so output is deterministic
  regardless of thread scheduling", `kernel-implicit/src/manifold_dc.rs`); the
  per-edge Hermite pass is collected in index order. The narrow-band mesher's
  flood fill is a deterministic BFS in discovery order
  (`kernel-implicit/src/narrow_band.rs:flood_fill`).
- **The fuzz corpus.** Chains derive entirely from explicit xorshift64 seeds;
  the whole 2000-chain report is byte-identical run to run post-R5
  (`kernel-brep/tests/fuzz_chains.rs` ratchet history, "3/3 runs
  byte-identical"), and the 10 000-chain Level-9 corpus reproduced
  byte-identically across 2 runs including its 2 failing seeds
  (`ROBUSTNESS.md`).
- **Feature rebuilds / persistence.** `Document` re-evaluation is a pure
  function of the feature tree; `.lmcpart` saves are byte-stable and the
  load→evaluate round-trip is bit-identical (BAR.md I3, `kernel-model/src/
  persist.rs` tests).

What is **not** promised: bit-identity *across platforms or libm versions*
(`sin`/`cos` differ; the fuzz ratchet keeps 2 points of cross-platform headroom
— `fuzz_chains.rs:PASS_RATE_FLOOR`), and floating-point associativity across
*future code changes* (the determinism tests pin behavior so a change is loud,
not silent).

## Lipschitz contracts (SDF side)

The narrow-band mesher's seeding prunes blocks by `|d(centre)| > block
half-diagonal`, which is only sound for **1-Lipschitz** fields (`|∇d| ≤ 1`); a
field that overstates distance can step over a surface
(`kernel-implicit/src/narrow_band.rs:find_seeds`). The contracts:

- **Exact primitive SDFs are 1-Lipschitz** (true Euclidean distances):
  Sphere/Cuboid/Cylinder/Capsule/Cone/Plane/Torus
  (`kernel-implicit/src/primitives.rs`).
- **CSG `min`/`max` (union/intersection/difference), `Offset` (d − t) and
  `Shell` (|d| − t) are non-expansive** — they preserve the Lipschitz bound of
  their children (`kernel-implicit/src/ops.rs:Node::distance`; relied on
  explicitly by `narrow_band.rs`'s "exact SDFs and the non-expansive CSG
  min/max/offset/shell of them").
- **`Gyroid` is normalized to honour the contract**: the raw sine-sum's gradient
  reaches √3·scale, so the field divides by `scale·√3`, guaranteeing `|∇d| ≤ 1`
  at an unchanged zero set (`kernel-implicit/src/primitives.rs:Gyroid::distance`
  — comment documents the bound and why).
- **`MeshSdf` magnitudes are true unsigned distances** (BVH nearest-triangle),
  hence 1-Lipschitz in magnitude; the winding-number sign can only flip across
  the surface itself (`kernel-implicit/src/meshsdf.rs`).
- **Smooth blends and transforms are the caller's risk**: `smin`-style blends
  and `Transform` with uniform scale `s` rescale distances by `s`
  (`ops.rs:Node::Transform` multiplies by `x.scale`) — a heavily non-metric
  field (e.g. post-blend) must be re-normalized via
  `kernel-implicit/src/redistance.rs` before narrow-band meshing, as the
  narrow-band module doc instructs.

## Known failure modes and their errors

Loud-by-design (machine-readable error or explicit empty, never a silent lie):

- **Checked booleans withhold invalid output.** `try_union` / `try_difference`
  / `try_intersection` run the identical boolean, validate, and return
  `Err(BooleanError { op, validity })` (closed/manifold/genus/shells report)
  instead of an invalid `Solid` (`kernel-brep/src/checked.rs`). The unchecked
  spellings still return whatever the arrangement produced — callers are
  expected to `validate` (the AI-facing JSON binding routes through the checked
  forms).
- **STEP import refuses what it cannot represent**: unsupported surfaces/curves
  (revolution, offset, parabola, trimmed…), periodic sphere/torus regions, and
  pole-spanning caps raise `StepError::Unsupported(entity)` — a uniform, loud
  error enumerated per entity class in the module table
  (`kernel-brep/src/step_import.rs:19-27,54`).
- **Degenerate construction inputs yield an empty `Solid`, not a panic or a
  broken manifold**: `extrude`/`revolve` sanitize profiles (drop duplicate
  points, reject <3-point/zero-area input, reject negative radius and isolated
  on-axis apexes that would pinch) (`kernel-brep/src/build.rs:revolve` doc and
  body).
- **Native format loads refuse contract violations**: wrong `format`, newer
  `version`, non-mm `units`, unreadable referenced part files
  (`kernel-model/src/format.rs` error enum).
- **Dense-mesher lattice cap (sharp edge, documented honestly):** all dense
  meshers refuse a conceptual lattice above `MAX_LATTICE_CELLS = 2²⁸` points
  (~1 GB of f32) by returning an **empty mesh** — a *silent* degradation at the
  call site (`kernel-core/src/mesher.rs:23`,
  `kernel-implicit/src/manifold_dc.rs:235`, `dual_contour.rs:67`). Two escapes
  exist: `VoxelGrid` construction *clamps resolution* (uniformly coarser, never
  empty — `kernel-implicit/src/grid.rs:48`), and the narrow-band meshers index
  a conceptual lattice up to `2⁴⁴` because they never allocate it
  (`kernel-implicit/src/narrow_band.rs:NARROWBAND_MAX_LATTICE_CELLS`, which also
  records the silent-empty incident that motivated the dedicated cap). Callers
  needing finer-than-2²⁸ resolution route through the narrow band — see the
  200 mm / 0.1 mm scale benchmark in `BENCH.md` (8.04e9 conceptual cells,
  watertight).
- **Manifold Dual Contouring's honest guarantee is "closed, never worse than
  Surface Nets"** — full 2-manifoldness can fail on connected-pinch CSG
  *differences* and does **not** vanish with refinement; validate with
  `check_mesh` and remedy with `make_manifold` when it matters
  (`kernel-implicit/src/manifold_dc.rs:manifold_dual_contour` doc, and the
  `difference_pinch_stays_closed_and_no_worse_than_naive` test that asserts
  exactly the honest bound).
- **Winding-number sign on open geometry**: `MeshSdf` tolerates small gaps and
  inconsistent normals (that is its purpose), but a *grossly* open soup has no
  well-defined inside; the winding field degrades smoothly rather than erroring
  (`kernel-implicit/src/meshsdf.rs` module doc). Heals of open inputs should be
  validated by the caller (`is_watertight` on the result, as
  `watertight_mesh_of`'s tests do).
- **Fuzz-measured robustness rate is published, not asserted away**: the chain
  corpus pass rate is ratcheted in-test (raise on fixes, never lower —
  `kernel-brep/tests/fuzz_chains.rs:MEASURED_PASS_RATE` history) and published
  in `ROBUSTNESS.md`.

## Mixed-operand hybrid booleans

The Level-9 "true convergence" operation
(`kernel-model/src/hybrid.rs:hybrid_boolean`): one boolean between an exact
B-rep `Solid` and a mesh or implicit-field operand. Its numerical contract:

- **Accuracy is split by side and stated per call.** On the `ExactStitch`
  route, every input B-rep face the operand does not touch is in the result
  **verbatim** — bit-identical vertices, analytic surface tags — verified
  geometrically (cyclic loop equality to 1e-9 mm), never assumed; the seam is
  the exact arrangement against the operand's facets, so its accuracy is the
  operand's own (a scan's facets verbatim; a field meshed at `voxel` by
  Manifold DC). On the `Healed` route everything is voxel-resampled and the
  result says so (`solid: None`, reason string, zero kept faces).
- **The watertight guarantee is checked**: both routes verify zero
  boundary/non-manifold edges and return `HybridError::NotWatertight` instead
  of a leaky mesh.
- **Result tessellation uses a 1e-7 weld**, not the 1e-5 default: a stitched
  solid's shared edges are exact f64 coincidences (identical f32 bits), so no
  geometric welding is needed — at 1e-5 the weld over-merges sub-1e-5 seam
  fragments and breaks 2-manifoldness (measured: 2 over-used edges in 24k
  triangles on the flange∪gyroid flagship; 0 at 1e-7).
- **Exact-route ceiling (measured, routed honestly):** f32 mesh operands whose
  seam chords densely cross one B-rep face can generate over-split splinters
  *thinner than `WELD_EPS`* (measured ≈ 4.8e-7 mm: torus scan / MDC sphere
  crossing a 40 mm face) — the arrangement then fails validation and the call
  self-demotes to `Healed` with that reason. Box/cylinder scans, enclosed
  scans, and field operands crossing only part-scale faces (the gyroid flange)
  stitch exactly; both behaviors are pinned in `hybrid.rs` tests.

## Tolerant modeling (opt-in)

The Level-9 "imports with gaps/slivers heal instead of failing" capability,
first slice (`kernel-brep/src/heal.rs`). The contract:

- **The caller owns the tolerance.** `Solid::heal_tolerant(tol)` takes `tol`
  in mm, absolute. It welds near-coincident vertices within `tol` (first-seen
  representative, scanned in vertex-index order through a `tol`-cell spatial
  hash — **deterministic**, no iteration-order dependence), collapses the
  duplicate runs the weld creates, drops loops left with < 3 distinct
  vertices and faces whose remaining polygon area is ≤ `tol²`, then rebuilds
  the half-edge solid — so cracks of width ≤ `tol` close because the rewritten
  loops share vertices and the twin-matcher pairs them again.
- **Loud, machine-readable healing.** Every call returns a `HealReport`:
  welded-vertex / dropped-face / dropped-inner-loop counts, unpaired half-edge
  counts before and after (`open_edges_*` — the crack measure), and the full
  `Validity` before and after. Nothing is repaired silently; a no-op heal
  reports zeros (`healed_anything() == false`) and is geometrically exact (the
  rebuild reuses the original coordinates — on a clean solid the volume is
  bit-identical, asserted in-tree).
- **Sane `tol` range.** The heal is meaningful for `tol` above the boolean's
  stitch tolerances (`WELD_EPS` 1e-7 / `TJUNCTION_EPS` 4e-7 — gaps below those
  heal inside the boolean already) and far below the model's feature size:
  features at or below `tol` are *legitimately collapsed* — that is what the
  tolerance means. `tol = 0` still merges exactly-coincident duplicates.
- **What does NOT heal (stated, not implied):** no geometry is invented — a
  hole wider than `tol` (e.g. an entire missing face) stays open and is
  reported via `open_edges_after > 0`; T-junction cracks (a vertex on another
  face's edge *interior*) are not healed by a standalone heal (the boolean
  pipeline heals its own T-junctions internally); long slivers (sub-`tol`
  width but area > `tol²`) survive; self-intersections and overlapping shells
  are out of scope.
- **Tolerant booleans are opt-in, strict paths unchanged.**
  `boolean_tolerant(a, b, op, tol)` heals both operands, runs the *identical*
  exact boolean, validates, and returns the result **only if well-formed** —
  else the same machine-readable `BooleanError` as the checked API. On clean
  operands it matches the strict path to the bit (asserted in-tree). The
  strict spellings (`union`/`try_union`/…) never heal: gapped input still
  fails loudly there, by design — the in-tree acceptance asserts BOTH
  behaviors on the same cracked operand pair
  (`heal.rs:gapped_boolean_fails_strict_and_succeeds_tolerant`).

## GPU evaluation and extraction (kernel-gpu) — tolerance-equivalent, never authoritative

> **2026-09:** `kernel-gpu` is parked, unbuilt, in `legacy/kernel-gpu/` (restore
> steps in `legacy/README.md`). This section records its contract as last built
> and tested; the `kernel-gpu/...` paths below now live under `legacy/`.

The wgpu/WGSL half (`legacy/kernel-gpu`, Metal on this machine) re-evaluates
the implicit side on the GPU. Its numerical position, stated once and enforced
in-tree:

- **The CPU stays bit-authoritative.** A GPU tree (`kernel-gpu/src/tree.rs:
  GpuNode`) is a single source of truth producing BOTH the ordinary CPU `Node`
  (`GpuNode::to_node` — the evaluation every CPU mesher consumes) and the WGSL
  (string codegen mirroring each CPU distance formula **branch-for-branch**,
  same guards and constants — `kernel-gpu/src/codegen.rs`). The GPU never
  redefines geometry; it re-evaluates it. (A mirror tree is used because
  `Node::Prim` boxes `dyn Sdf` without `Any`, so a built CPU tree cannot be
  introspected back into primitive parameters.)
- **Declared GPU tolerance: `|gpu − cpu_f32| ≤ 1e-4 · (1 + |cpu_f32|)`** at
  every probe, enforced by `kernel-gpu/tests/parity.rs`: per-leaf probe sweeps
  for all 12 lowerable leaves (Sphere, Cuboid, Cylinder, Cone, Plane, Torus,
  Capsule, Gyroid, BeamLattice, Pipe, VoxelGrid, ExprSdf), one tree through
  all 18 combinators (the 14 `Node` variants + the 4 fillet/chamfer seam
  operators), a smooth-blend chain, a rocket-style jacket truss (320 struts)
  ∪ helical pipe composite, a helical-thread `Expr` field, and an
  all-21-operator `Expr`. Measured headroom on Apple M3 / Metal: max scaled
  error 1.1e-7 … 1.0e-6 — two to three orders inside the bound (the tests
  print the per-tree maxima).
- **The GPU evaluates f32 only** (WGSL has no f64): `Expr` leaves, which the
  CPU walks in f64, run in f32 on the GPU — inside the tolerance at part
  scale (measured); at large coordinates the f32 honest range above (~1e6 mm
  already degraded) applies to the whole GPU path. The IEEE poles of
  `Div`/`Sqrt`/`Mod`/`Atan2(0,0)` are *indeterminate* in WGSL rather than the
  CPU's defined IEEE results — keep fields off their poles, exactly as the
  `expr_sdf` docs already require.
- **Lattices/pipes evaluate brute-force on the GPU** (`min` over a strut
  storage buffer): the CPU's spatial grid "never changes the field", so both
  sides compute the same min to rounding. The GPU wins by parallelism, not
  pruning; for ≫10k-strut lattices with few queries the CPU grid remains the
  scalable path.
- **Extraction: GPU = preview/bulk, CPU Manifold DC = watertight authority.**
  `GpuSurfaceNets` mirrors `kernel_core::surface_nets` (identical lattice
  layout, cube-edge tables generated from `kernel_core::marching`, identical
  vertex/quad rules; prefix-sum compaction instead of serial emission), so the
  output is CLOSED by the same shared-corner-buffer argument. It is NOT
  promised bit-equal to the CPU mesh: vertices shift within ~tolerance/|∇d|
  and a corner sample within 1e-4 of zero may classify differently in marginal
  cells (in the in-tree gyroid-block test GPU and CPU happened to agree
  exactly — 426 660 triangles and 18 non-manifold edges each — but only
  closure and volume are asserted). Production watertightness remains
  `manifold_dual_contour` + `check_mesh` on the CPU.
- **Determinism:** GPU extraction assigns vertex ids and triangle slots by
  exclusive prefix sums (no atomics anywhere), so two runs on the same
  device/driver are bit-identical — pinned by
  `gpu_extraction_is_deterministic_run_to_run` and by the field evaluator's
  cold-vs-warm bit check in the bench. The cross-platform caveat above applies
  with "GPU driver" added to the list of variables.
- **Failure modes are loud:** no adapter → `GpuError::NoAdapter`; an
  unlowerable tree (empty lattice — its CPU distance is +∞, which has no WGSL
  spelling — non-finite parameter, non-positive Lipschitz bound, out-of-range
  strut index) → `GpuError::Lower`; a generated-WGSL validation failure →
  `GpuError::Shader` carrying the full source; an extraction beyond the dense
  2²⁸ cap or the device's buffer limits → `GpuError::TooLarge`, deliberately
  LOUDER than the CPU dense meshers' documented silent-empty over-cap edge.
  Domain guards (degenerate/non-finite domain, `A − A`, bare half-space)
  mirror the CPU exactly and yield an empty mesh, not an error.
- **Narrow-band extraction (2026-07-30):** `extract_narrow_band`
  — Lipschitz-safe coarse block scan → prefix-sum compaction → refine active
  blocks only, splicing the SAME cube-edge unroll as the dense path (shared
  `edge_unroll()`, one source of truth). Corner coordinates come from one
  global-index expression, so neighbouring blocks evaluate identical f32
  bits at shared lattice points and the dense closure argument carries over.
  Same honesty contract (preview path, CPU authoritative), same determinism
  statement (prefix sums, no atomics). Band floor `max(band, 2·voxel)`
  absorbs cross-pipeline rounding; sub-floor/NaN bands clamp (pinned
  bit-identical to the explicit floor). Delivers domains beyond the dense
  2²⁸-cell cap (pinned: 3.0e8-cell sphere, watertight, volume −0.001% vs
  analytic) with active-block and samples-evaluated receipts in
  `NarrowBandStats`; wall-clock printed, work counters asserted.
- **Runtime-skipping tests:** all 18 GPU tests (7 in
  `kernel-gpu/tests/parity.rs`, 6 in `kernel-gpu/tests/extraction.rs`, 5 in
  `kernel-gpu/tests/narrow_band.rs`) need an
  adapter; without one each prints a loud `SKIPPED <name>: NO GPU ADAPTER …
  verified NOTHING` banner and passes vacuously. On this Mac (Metal present)
  they all RUN — a green from a headless CI is a skip, not a verification, and
  the banner is the tell.
