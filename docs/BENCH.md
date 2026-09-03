# Kernel Performance Baseline

Measured **2026-06-10** with `cargo run --example bench_kernel -p kernel-model --release`
(the harness now lives uncompiled in `legacy/kernel-model-examples/bench_kernel.rs`; see that
folder's README to restore it before re-measuring)
(`std::time::Instant`, median of 3 runs per workload, `black_box`-guarded; release profile:
opt-level 3, thin LTO).

**Machine**: Apple M3 (8 cores: 4 performance + 4 efficiency), 24 GB RAM, macOS
(Darwin 25.4.0), rustc 1.95.0.

> **Honesty note on load**: this machine is shared with other agents' builds and
> test suites; load average swung 6–43 during the session. The table below is the
> quietest invocation observed (load ~6, after the other agents' suites drained;
> its unchanged rows — boolean, adaptive, flange — reproduce the 2026-06-09 quiet
> baseline, which is the tell that the window was genuinely quiet). A second
> independent invocation in the same window agreed (heal 58.9 ms median), and an
> earlier verification pass during the optimization session published 62.9 ms —
> the optimized heal sits in the high-50s-to-low-60s quiet. Contended invocations
> of the *identical binary* showed `hybrid_heal_hex_nut` at 225–295 ms and
> `gyroid_mdc` at 50–209 ms (~4–20× swings). **Re-run on an idle machine before
> drawing tuning conclusions**; treat cross-run deltas under ~2× as possible
> noise until reproduced quiet.

## Results (median of 3, ms)

| bench | median (ms) | runs (ms) | sanity | 2026-06-09 baseline |
|---|---|---|---|---|
| boolean_union_cyl_box | 5.6 | 5.9 / 5.6 / 5.4 | 103 faces, valid=true | 5.0 |
| adaptive_tessellation | 9.0 | 9.2 / 9.0 / 9.0 | 8956 tris, watertight=true | 10.1 |
| gyroid_mdc | 9.2 | 9.2 / 9.8 / 8.6 | 101388 tris, watertight=true | 24.1 |
| hybrid_heal_hex_nut | 55.7 | 55.7 / 55.0 / 55.7 | 9144 tris, watertight=true | 821.8 |
| flange_extrude_7_holes | 0.1 | 0.2 / 0.1 / 0.1 | 290 faces, genus=7, valid=true | 0.2 |

## What each workload measures

1. **boolean_union_cyl_box** — exact planar boolean (arrangement + stitch) of a
   64-segment cylinder (r 15, h 40) with an overlapping 30×30×30 cuboid.
2. **adaptive_tessellation** — `tessellate_adaptive_tol(…, 0.01)` (10 µm chord
   tolerance) of `filleted_cylinder(20.0, 40.0, 3.0, 64, 16)`: the exact analytic
   meshing path over plane/cylinder/torus faces.
3. **gyroid_mdc** — Manifold Dual Contouring of the watertight gyroid lattice from
   kernel-implicit's test (`Gyroid::new(region, 0.35, 0.6)` ∩ 40 mm cube, voxel 0.8):
   the implicit half's signature TPMS workload, ~101 k triangles.
4. **hybrid_heal_hex_nut** — `kernel_model::watertight_mesh(&hex_nut(16.0, 8.0, 10.0), 0.5)`:
   B-rep → tessellate → winding-number `MeshSdf` → MDC re-mesh, the hybrid's core move.
5. **flange_extrude_7_holes** — `extrude_with_holes` of a 96-gon flange (r 40) with a
   48-gon r 10 centre bore plus six 24-gon r 3 holes on a R30 bolt circle, h 8, plus
   `validate` (multi-loop construction + the validity oracle; genus 7 confirms 7 holes).

## The 2026-06-10 heal-path optimization (822 ms → 56 ms, ~15×)

The Level-9 frontier flagged in the previous baseline is closed. Same-day,
same-load A/B on this machine: pristine 519 ms → optimized 63 ms (8.3×); vs the
published 2026-06-09 quiet baseline: 821.8 → 55.7 ms (14.8×) in the quietest
window (62.9 ms in the optimization session's own quietest window — see the
honesty note). `watertight_mesh` semantics are unchanged (all 503 workspace
tests pass, including the heal-volume and watertightness assertions;
`parts_gallery`, `catalog`, `hybrid_showcase` all exit 0). What changed
(profiled first — winding-number evaluation was ~83% of every SDF query, and
the default gradient cost 6 queries):

- **`MeshSdf` fast-winding upgraded to the order-2 Barill expansion**
  (Jacobian + Hessian moments per BVH node) with `BETA` 4.0 → 2.0
  (`kernel-implicit/src/meshsdf.rs`). Accuracy on the final tree (max
  fast-vs-brute error, 4000-query sweeps, deterministic harness): sphere
  24×48: 6.41e-3 (6.7e-3 before the change); sphere 64×128: 6.58e-3 (9.5e-3
  before); hex-nut B-rep tess: 1.01e-2 (8.7e-4 before). The spheres improve
  and stay comfortably inside the 1e-2 bound the in-tree
  `fast_winding_matches_brute_force` test enforces (sphere 24×48, 400-query
  subset: 6.41e-3 worst case). The hex-nut probe is the honest trade: its
  4000-query max touches 1.01e-2 — at a far-field point in the bore where the
  true winding is ≈ 0, so the inside verdict (threshold ½) has two orders of
  magnitude of margin; near-surface queries never use the expansion (the
  `dist > BETA·radius` gate forces exact per-triangle recursion there), which
  is why every heal volume/watertightness assertion is unaffected.
- **Oversized-triangle refinement before BVH build** (`REFINE_FRAC` 0.05):
  B-rep tessellations carry part-sized facets whose node radii made the
  far-field test never fire; bisecting them (geometry-identical) restores the
  O(log n) descent.
- **Analytic `MeshSdf::gradient`** — `sign·(p − closest)/|p − closest|` (one
  nearest + one sign query) instead of the 6-query central difference; exact
  for a distance field, with a triangle-normal fallback within f32 noise of the
  surface.
- **AABB sign short-circuit** — a query outside the mesh AABB can never be
  inside (the whole surface subtends < 2π from outside its hull), so the
  padding ring of the voxel grid skips the winding traversal entirely.
- **Manifold DC: Hermite data once per unique lattice edge** instead of
  per-cell (a shared minimal edge was refined 4×), **lazy face-centre saddle
  samples** (only ambiguous 4-crossing faces pay), dropped the cube-centre
  sample that fed only the disabled body merge, and replaced the per-cell
  `HashMap` patch map with a fixed-array scan
  (`kernel-implicit/src/manifold_dc.rs`). Output is bit-identical to the
  per-cell recomputation (same `refine_crossing`/`gradient` calls on the same
  inputs, consumed in the same order). This is also what took **gyroid_mdc
  24.1 → ~9 ms** — it shares the dense-MDC path.

Per-query `MeshSdf` costs on the hex-nut heal lattice (51 324 points,
single-threaded, measured via a temporary stage-profiling harness — deleted
after the work — under comparable load ~17–22, so ratios are meaningful, not
the absolute floor): `distance` 8.9 → 3.1 µs, `gradient` 48.8 → 3.1 µs
(15.7×), winding alone 7.4 → 2.3 µs; the heal's MDC stage went 546 → 157 ms
under that load and the whole heal measures 55.7 ms in the quiet window above.

## Scale evidence (run-once, opt-in: `-- --scale`)

Measured 2026-06-10 (single invocations — these are capacity probes, not
interactive-latency medians). The 0.1 mm TPMS row ran under load ~15–21
(another agent active; the same invocation's table rows were ~10–25% above the
quiet medians, so treat the march wall time as an upper bound):

| case | result |
|---|---|
| tess_102k_face_revolve | build 51 ms, adaptive tess (50 µm) **302 ms** (246 ms in an earlier, quieter window) — 102 400 B-rep faces → 204 800 tris, watertight=true |
| gyroid_200mm_narrowband | march **149.7 s** — conceptual lattice 8.04e9 cells (≈30× the dense 2²⁸ cap), visited 143.4 M (1.78%), **50.3 M tris, watertight=true** (check itself 14.6 s); RSS sampled every 10 s stayed under 11 GB |

- **tess_102k_face_revolve** — a 200-point corrugated ring profile revolved in
  512 sectors: 102 400 analytic band faces, adaptive-tessellated at 50 µm. The
  Level-9 "interactive rebuilds on 100k-face models" probe: construction plus
  exact tessellation lands well under a second.
- **gyroid_200mm_narrowband** — a 200 mm TPMS shell (`Gyroid(scale 0.02,
  shell 1.5)` ∩ 200 mm cube) at 0.1 mm voxel via `surface_nets_narrowband`.
  The point is the *conceptual* grid: 2003³ ≈ 8.04e9 cells, ~30× beyond the
  dense meshers' 2²⁸ allocation cap (`NARROWBAND_MAX_LATTICE_CELLS` = 2⁴⁴
  indexes it without allocating), while the surface-tracking march touches only
  1.78% of it and still returns a watertight 50.3 M-triangle mesh. (An earlier
  0.15 mm probe on the same tree: 2.39e9 conceptual, 53.2 M visited (2.22%),
  22.4 M tris watertight, march 46.3 s, ~5.5 GB peak RSS — the visited-fraction
  drop at finer voxels is the area-proportional scaling doing its job.) Honest
  scope notes: the march is single-threaded (the corner-sample cache is a
  sequential flood fill — parallelizing it is open work), and at period
  ≈ 314 mm this is a single TPMS wall through the part, not an nTop-scale
  multi-thousand-cell lattice; the *fixed-period* lattice rows above
  (gyroid_mdc) cover density, this row covers resolution capacity.

## Reading of the numbers (2026-06-10)

- The hybrid heal is no longer the outlier: a single fastener at 0.5 mm voxel
  heals in ~56 ms — interactive territory — and the heal now scales with
  surface area as the winding tree intends, not with brute-force sign sums.
- Exact-path operations remain fast at interactive sizes, and the 100k-face
  adaptive tessellation at ~250–300 ms gives the first large-model exact-path
  datum.
- The narrow band turns the 2²⁸ dense cap into a soft limit: 8 *billion*
  conceptual cells (200 mm part at 0.1 mm) are reachable today at
  area-proportional cost; multi-million-cell *lattices* at fine voxels remain
  out of reach until the band march is parallelized (open work, noted above).

## Reproducing

```
# bench_kernel.rs is parked in legacy/kernel-model-examples/ — restore it first
# (legacy/kernel-model-examples/README.md), then:
cargo run --example bench_kernel -p kernel-model --release            # the median-of-3 table
cargo run --example bench_kernel -p kernel-model --release -- --scale # + the scale section
```

Update this file (date, machine, table) whenever the kernel's meshing/boolean
internals change; keep old numbers in git history rather than editing them in place.

## GPU baseline (kernel-gpu, wgpu → Metal) — measured 2026-06-10

`cargo run --example bench_gpu -p kernel-gpu --release` — adapter **Apple M3
(IntegratedGpu), Metal**; CPU comparisons on the same machine's 8 cores via
rayon. Load note: measured at load ~11–19 (other agents' suites draining);
the bench asserts the parity tolerance and bit-determinism on every row, so
correctness is load-independent — re-run quiet before tuning on the timings.
GPU eval times are **end-to-end** (probe upload + dispatch + readback, no
caching); extraction times cover all compute passes + prefix sums + mesh
readback (pipeline compile stated separately).

| workload | GPU | CPU (rayon, 8 threads) | speedup | agreement |
|---|---|---|---|---|
| field eval — gyroid lattice tree, 256³ = 16.78 M pts | compile 50 ms; cold 187 ms / warm 206 ms (**81–90 Mcells/s**) | 144 ms (116 Mcells/s) | **0.7×** | max scaled err 5.4e-7; warm bit-identical to cold |
| field eval — rocket-style jacket truss (320 struts; GPU is brute-force min, CPU is the accelerated grid), 128³ = 2.10 M pts | compile 31 ms; cold 64 ms / warm 49 ms (**43 Mcells/s**) | 408 ms (5.1 Mcells/s) | **8.3×** | max scaled err 1.2e-6 |
| surface-nets extraction — gyroid 40 mm @ 0.15 mm (19.7 M corner samples) | compile 106 ms; extract cold 118 ms / warm 120 ms | 663 ms (`kernel_core::surface_nets`, same lattice) | **5.5×** | **identical 3 175 116 tris both sides**, bnd=0, nme=0, vol 8595.0 vs 8595.0 (dVol 2.2e-7) |

Reading of the numbers (honest):

- **A cheap field is transfer-bound on unified memory.** The gyroid evaluates
  in ~9 transcendentals; uploading 268 MB of probe points and reading 67 MB
  back costs more than 8 CPU cores just computing it — the GPU *loses* (0.7×)
  on the probe-eval API for this field. The probe evaluator earns its keep on
  expensive fields (the 320-strut truss: 8.3× even though the GPU brute-forces
  what the CPU prunes with a spatial grid).
- **Extraction is where the GPU pays off**: probe positions are generated
  in-shader (nothing shipped up), and only the compacted mesh comes back —
  5.5× on the BENCH gyroid at 0.15 mm, 3.2 M triangles in ~120 ms after a
  one-time ~106 ms pipeline compile. At this resolution the GPU and CPU
  marches agreed *exactly* in topology (same triangle count, zero boundary /
  non-manifold edges on both) and to 2.2e-7 in volume — consistent with the
  parity suite's 1e-7-ish field errors being far below the 0.15 mm cell size.
- Role statement stands (NUMERICS.md): GPU = preview/bulk extraction; the CPU
  Manifold DC remains the watertight authority for production output.
