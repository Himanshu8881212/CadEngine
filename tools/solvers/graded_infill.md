# graded_infill — stress-graded gyroid lattice: thicker walls where the part works

**Runner**: `tools/analyzers/graded_infill_runner.py` ·
`python3 tools/analyzers/graded_infill_runner.py <job.json> [--out PATH]` ·
`python3 tools/analyzers/graded_infill_runner.py --selftest`
**Gates**: none that pin the result — see Benchmark gates. What runs:
`python3 tools/analyzers/graded_infill_runner.py --selftest` (the gyroid
calibration round-trip) and `python3 tools/analyzer_registry.py --check-contract`
(the runner-contract gate; `analyzers/graded_infill_runner.py` is in its
`CONTRACT_RUNNERS` list).
**Status**: no gate suite, no validation pin **Tier**: Demonstrated
(`python3 tools/analyzer_registry.py --tier graded_infill` returns
`"manifest": null`, `"pins": []`, `"gate_suite": null`; the registry's rule is
that Validated needs a manifest AND a present pin, and this analyzer has
neither. Also `docs/ANALYSIS_TIERS.md`.)

It re-skins a solid and fills its interior with a sheet-gyroid lattice whose wall
thickness follows a prior `ace_fea` von Mises field. The question it answers is
the one a slicer's uniform infill percentage cannot: given that the load path is
not uniform, where should the material go — thicker walls where the part works
hardest, thin walls where it coasts, at the same or lower mass.

The gyroid (not Voronoi, not gyroid-by-accident) is chosen because sheet-gyroid
walls are continuous-curvature and short-self-buttressed, so they are expected to
print without internal supports. **That expectation is not gated here** — see
Validity limits.

This is a GEOMETRY SYNTHESIS tool, not a solver. It consumes a stress field and
emits a mesh. It computes no stresses, and it does not check that the graded part
still passes: re-analyse the result.

## What it actually computes

Three deterministic stages, then the kernel's own mesher.

**1. Occupancy and skin.** `occupancy = solid_fraction >= iso`;
`interior = binary_erosion(occupancy, iterations = round(shell_mm / voxel))`
(`scipy.ndimage.binary_erosion`); `skin = occupancy AND NOT interior`. The skin
stays fully solid, so the part keeps a real outer wall.

**2. Stress → wall thickness, by percentile.** Over SOLID voxels only:

    t(x) = wall.min + (wall.max - wall.min) * clip( (vm(x) - P_lo) / (P_hi - P_lo), 0, 1 )

where `P_lo`, `P_hi` are the `stress_map.lo_pct` / `hi_pct` percentiles of von
Mises over the solid (defaults 20 and 95). This is a **linear percentile remap,
not an optimality condition**: it is a defensible heuristic for putting material
where stress is, and it is not derived from, nor equivalent to, a compliance
optimum (that is [ace_optimize.md](ace_optimize.md)). Degenerate percentiles
(`P_lo == P_hi`) fall back to the mid thickness everywhere and say so in `notes`.

**3. Gyroid band at the calibrated threshold.** The sheet gyroid is the band
`|g| <= alpha` of

    g(x,y,z) = sin X cos Y + sin Y cos Z + sin Z cos X,     X = 2*pi*x / cell_mm

evaluated at voxel CENTERS. The threshold `alpha` is NOT the thickness — `|g|`
grows at the local gradient rate, which varies over the surface — so `alpha(t)`
is calibrated numerically on one unit cell (`GyroidCalibration`, 64^3 =
262 144 cell-centered samples by default):

    to first order, dist(x) = (c / 2pi) * g / |grad g|          (gradient analytic)
    a true wall of thickness t  <=>  |g| <= pi (t/c) |grad g|
    vf(t) = sample mean of that indicator                        (wall_fraction)
    alpha(t) = F^-1( vf(t) ),   F(a) = sample mean of [ |g| <= a ]

So the single global threshold reproduces the target VOLUME of the wall by
construction. **It does not reproduce the local thickness pointwise**, and the
class docstring is explicit about that.

The graded density is 1.0 on skin, the band indicator on interior, 0 outside; it
is written to `graded.npy` and meshed BY THE KERNEL — one `kernel-api run` with
`mesh_density_grid` (dual contour + heal, watertight-or-fail), escalating to
voxel/2 and voxel/3 with one voxel of air padding when thin walls pinch at native
resolution. At each finer rung **the field is re-evaluated analytically, not
interpolated** (`build_graded(..., f)`), so the escalation adds real resolution
rather than smoothing. Padded grids over 48 000 000 cells are skipped with a
note.

## The contract

```
{out_dir, voxel_mm,                        # REQUIRED
 origin_mm?,                               # world coord of grid node (0,0,0); default [0,0,0]
 GEOMETRY exactly one of:
   ops + solid + shape [+ supersample],    # LMCAD ops sampled onto the grid (same route as ace_fea)
   npy,                                    # an existing (nx,ny,nz) float density .npy
 stress_npy,                               # REQUIRED: stress_field.npy of a prior ace_fea ON THE SAME GRID
 cell_mm?,                                 # gyroid cell size, default 8.0
 wall?: {min, max},                        # thickness range in mm, default {0.8, 2.4}
 stress_map?: {lo_pct, hi_pct},            # percentiles mapped to wall.min/max, default {20, 95}
 shell_mm?,                                # solid skin depth, default 1.5
 iso?,                                     # occupancy threshold, default 0.5
 file?}                                    # output mesh name inside out_dir, default "graded_infill.stl"
```

**The grid must match exactly.** A `stress_npy` whose shape differs from the
geometry grid is REFUSED, never resampled — same voxel, same origin, same shape
as the `ace_fea` run that produced it. And the stress field must come from the
VOXEL runner: `ace_fea_tet`'s per-node unstructured fields are not a grid and
cannot be used here ([ace_fea_tet.md](ace_fea_tet.md),
`campaign/digests/tools_cookbook.md`).

Receipt (last non-empty stdout line; logging on stderr): `file`, `volume_mm3`,
`solid_volume_mm3`, `volume_fraction`, `skin_voxels`, `interior_voxels`,
`cell_mm`, `wall_range_applied`, `stress_pcts_used {lo_pct, hi_pct, lo_pa,
hi_pa}`, `watertight`, `healed`, `triangles`, `mesh_upsample`, `mesh_voxel_mm`,
`graded_npy`, `notes`, `timings_s`, `determinism`. Artifacts: `out_dir/graded.npy`
and the mesh named by `file`.

`--selftest` emits the calibration receipt instead:
`{ok, roundtrip_max_rel_err, wall_map_monotone, samples}`.

Exit contract: the shared one in `tools/_receipt.py` — **0** `ok:true`, **1** the
tool could not run the request, **2** it RAN and REFUSED or the analysis failed.
**This runner declares no `refusal.*` kinds of its own.** Its domain rejections
are raised as plain `ValueError` / `RuntimeError`, so they land as
`error_kind: "internal"` with exit **1**, not as a refusal with exit 2 — verified
2026-09-04 by running it with `stress_npy` omitted:
`{"ok": false, "error": "ValueError: stress_npy is required …", "error_kind": "internal", "exit_code": 1}`.
Match on `ok`, and do not write a gate that keys on exit 2 for this tool. The
rejections it makes are: no geometry route or both routes given; missing
`stress_npy`; a stress/geometry shape mismatch; bad parameters (`cell_mm <= 0`,
not `0 < wall.min <= wall.max`, `shell_mm < 0`, not
`0 <= lo_pct < hi_pct <= 100`); empty occupancy at `iso`; `shell_mm` eroding the
whole part so no interior is left; and the kernel refusing to mesh the graded
field watertight at every rung.

Determinism: the runner's own note is that this is pure numpy field synthesis
plus the kernel's meshing, so `core_digest` is expected to be stable across runs
AND machines; `timings_s` is the only declared non-deterministic path.

## Benchmark gates (measured, frozen)

**None. This analyzer has no benchmark gate suite and no validation pin.** The
registry records `gate: no`, `pin: no`, `man: no` for it
(`python3 tools/analyzer_registry.py --tier graded_infill`), with the reason
stated there: "ships a --selftest and a support-necessity audit, but no
ground-truth pin on the graded stiffness."

What exists, and exactly what it proves:

| check | what it pins | where |
|---|---|---|
| `--selftest` calibration round-trip | that the calibration's own inverse is self-consistent — `threshold -> fraction -> threshold` recovers the threshold, and the wall-thickness map `alpha(t)` is MONOTONE. Gate: 5% and monotone. **Measured here 2026-09-04: `roundtrip_max_rel_err` 0.009312740322609814, `wall_map_monotone` true, `samples` 262144** (`python3 tools/analyzers/graded_infill_runner.py --selftest`). | `GyroidCalibration.roundtrip_check` in `tools/analyzers/graded_infill_runner.py` |
| runner contract | routes through `run_cli`, has no bare `sys.exit(0)` failure path, and the old flat path `tools/graded_infill_runner.py` still forwards here | `python3 tools/analyzer_registry.py --check-contract` |
| watertightness of the deliverable | per-run, not a frozen gate: the mesh comes through the kernel's `mesh_density_grid` watertight-or-fail path, and the receipt reports `watertight`, `healed` and the `mesh_upsample` rung that succeeded | `tools/analyzers/graded_infill_runner.py` → `mesh_through_engine` |

That round-trip is a **self-consistency check of a numerical inverse**. It does
NOT pin: that the realised wall thickness equals `t(x)` anywhere in the part,
that the graded part's stiffness or strength matches any prediction, that the
grading improves anything over uniform infill, or that the mesh is printable.
Any stiffness, mass-saving or thickness number this tool is used to justify is
unvalidated against an independent reference.

## Validity limits / out of scope

- **Graded stiffness is NOT characterised.** No study in this repo compares a
  graded-infill part against a measured or independently-computed stiffness, and
  nothing establishes that the percentile map produces a better structure than a
  uniform lattice at the same mass. Until one exists, treat any performance claim
  for the grading as unsupported, and gate the RESULT by re-analysing it (voxelize
  the mesh with `tools/analyzers/voxelize_stl.py`, re-run
  [ace_fea.md](ace_fea.md)) rather than by citing this runner.
- **Local wall thickness is NOT pinned.** `alpha(t)` matches the target VOLUME by
  construction; the class docstring states local thickness varies ~+/-10%
  (p10-p90) over the surface and that the band thins where sheets merge, plus a
  thin-wall relation `vf(t) ~= 3.106 t/c` (the literature gyroid area constant).
  **Those three figures are docstring claims with no gate behind them** — the
  `--selftest` asserts only the round-trip and monotonicity above. Do not quote
  the +/-10% as a measured tolerance.
- **Self-supporting printing is a claim to verify per part, not a gate.** The
  runner's docstring says to check the exported mesh in a slicer's support
  preview, and points at the ops-surface `support_report`. Note the repo is not
  self-consistent on whether that op can even see this output: the runner's
  docstring says `support_report` "refuses imported meshes", while `API.md` lists
  it among the ops that accept a bound mesh from `export_stl` / `import_mesh`.
  Either way, **nothing in this repo gates the self-supporting claim for a graded
  gyroid**, so verify it in the slicer for the part in hand.
- **Resolution.** Walls under ~2 voxels only resolve on the upsampled rungs; the
  runner adds a `notes` line when `wall.min < 2 * voxel_mm` and names the rung it
  actually used (`mesh_upsample`, `mesh_voxel_mm`). Trust that rung, not the
  requested `wall.min`.
- **It inherits every limit of the stress field it is fed.** The grading is only
  as meaningful as the `ace_fea` run behind it — one static load case, hex8 voxel
  discretization, feature stresses approximate below ~4 voxels, and (per
  ace_fea.md) fillet/notch peaks that do not converge. A grading driven by a
  non-converged peak concentrates material at a mesh artifact.
- **Single stress field, single load case.** No envelope over multiple load
  cases, no fatigue, no thermal, no anisotropy. The solve behind it is
  as-designed, not as-printed.
- **The output is a MESH ONLY** — no B-rep reconstruction exists. It is print /
  simulation geometry, not a parametric solid.
- **`tools/analyzers/stress_to_density.py` is NOT part of this pipeline.**
  Checked: `graded_infill_runner.py` neither imports nor calls it, and no
  campaign or gate wires the two together. It is a SIBLING hinge in the same
  FEA-driven grading family — it maps a von Mises `.npy` to a `[floor, ceil]`
  density `.npy` (`density = floor + (ceil - floor) * clip(vm/vm_clip, 0, 1)^gamma`,
  `vm_clip` = the `--clip-percentile` of POSITIVE voxels, non-finite voxels
  refused) for `kernel_implicit::grid_field::GridField` to consume as a grade law
  for `Node::offset_by` (`crates/kernel-implicit/src/grid_field.rs`). That is the
  IMPLICIT/`GridField` grading route; `graded_infill` is the standalone
  gyroid-lattice route and does its own percentile mapping internally. Use one or
  the other knowingly, and do not describe them as stages of one pipeline.
  `stress_to_density.py` is a preprocessor, not a registered analyzer — it has no
  tier and no card.

## When to use it

When a part is already stiff enough as a solid but too heavy or too slow to
print, the loads are known, and an `ace_fea` field on a matching grid already
exists: the design question is "which interior can I hollow out, and by how
much". Fill with the graded gyroid, then re-analyse the result and gate on THAT,
not on this runner's receipt.

Reach for [ace_optimize.md](ace_optimize.md) instead when the load PATH itself is
the open question — where material belongs, not merely how much of it. Reach for
`stress_to_density.py` + `GridField` instead when the geometry is implicit and
the grading should drive `offset_by` on an existing implicit node. And when the
governing case is sustained load, resonance or buckling, the grading answers none
of it ([creep.md](creep.md), [modal.md](modal.md), [buckling.md](buckling.md)).

Run: `python3 tools/analyzers/graded_infill_runner.py job.json` ·
what can be proved: `python3 tools/analyzers/graded_infill_runner.py --selftest`
(calibration self-consistency only — there is no result gate)
