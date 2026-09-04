# ace_optimize — SIMP topology optimization: where does the material actually have to be

**Runner**: `tools/analyzers/ace_optimize_runner.py` (SIMP + cone density filter +
optimality-criteria loop, top88 lineage, driving the in-tree
`physics.reference_fea` in SIMP mode) ·
`python3 tools/analyzers/ace_optimize_runner.py <job.json> [--out PATH]`
**Gates**: `python3 tools/validation/ace_optimize_validation.py` (the four
inequality pins; also runs under `python3 tools/analyzer_registry.py --run-pins`,
the `physics-gate` workflow) · `python3 tools/analyzer_registry.py --check-contract`
(runner-contract gate — `analyzers/ace_optimize_runner.py` is in its
`CONTRACT_RUNNERS` list). **No benchmark gate suite of its own.**
**Status**: pinned, no gate suite **Tier**: Validated
(`python3 tools/analyzer_registry.py --tier ace_optimize`; manifest
`tools/manifests/ace_optimize.manifest.json` + pin
`tools/validation/ace_optimize_validation.py` are both present, which is the
registry's rule).

A designer's guess at where to put ribs is a guess. This runner answers the
stiffness question directly: at a fixed material budget, which voxels carry the
load. It is the only tool here that CHANGES the geometry rather than judging it,
and that makes its honesty problem specific — a density-based optimizer is very
easy to read as more finished than it is. Two things in this runner exist for
that reason. First, after the SIMP loop it re-analyses the THRESHOLDED, binary
part — the thing that would actually print — instead of reporting the
gray homogenized proxy's numbers. Second, the STL is emitted through LMCAD's
watertight-or-fail mesh gate, and a design that cannot pass it produces no file
at all rather than a mesh you discover is broken in the slicer.

It optimizes COMPLIANCE. It does not optimize strength, and the manifest says so
outright.

## The physics

Minimum-compliance density-based topology optimization
(`tools/manifests/ace_optimize.manifest.json`):

    min_rho  c(rho) = F^T u(rho)
    s.t.     K(rho) u = F,   mean(rho | design) = volfrac,   0 <= rho <= 1

    SIMP:    E_e = floor + rho_e^p (E0 - floor),          p = 3 by default
    filter:  rho_phys = H rho / (H 1)                     (cone kernel, radius filter_radius_vox)
    OC:      rho_new = clip( rho * (dC/drho / lambda)^0.5, move ),  lambda by bisection

with the SIMP sensitivity `dC/drho_e = -(p / rho_e) * 2 * U_e` and the density
filter's chain rule applied to both the objective and the volume gradient.

**The elasticity underneath is `ace_fea`'s and nothing else** — trilinear hex8 on
the voxel grid, linear elastic, small strain, isotropic, single static load case.
Everything in [ace_fea.md](ace_fea.md)'s validity section applies here, including
that SIMP-mode stress is homogenized (`rho_eff^p * D B u`), not solid-material
stress. That is precisely why the runner ends with a binary re-analysis.

**Discretization**: one design variable per voxel. Design domain = voxels solid in
the INITIAL sampled geometry (`rho0 >= 0.5`) AND region kind `design`; `frozen`
and `fixed` voxels are re-pinned to 1.0 and `void` to 0.0 every iteration. The
cone filter is masked to the design region so weight cannot bleed in from
outside. Default move limit 0.2, density floor 0.02, penalty 3.0, filter radius
1.5 voxels.

**The loop stops** at `max_iters`, at max design-variable change < 0.01
(`stop_reason: "converged"`), or at 0.8x `time_budget_s`
(`stop_reason: "time_budget"`). A budget stop is a REFUSAL by default
(`refusal.optimization_incomplete`, no release artifact) unless the job sets
`allow_incomplete_optimization: true`, in which case the receipt carries a
`optimize.stopped_before_convergence` warning object.

**It is a local optimizer on a non-convex problem.** No global-optimality claim
is made anywhere, and none should be quoted.

## The contract

Job = the `ace_fea` job (geometry via `ops`+`solid`+`shape` or `npy`, `material`,
`fixtures`, `loads`, `regions`, `voxel_mm`, `origin_mm`) plus:

```
{volfrac,                 # REQUIRED, strictly in (0,1) — target mean density over the design region
 penalty?,                # SIMP p, default 3.0 (>= 1.0)
 filter_radius_vox?,      # cone filter radius in voxels, default 1.5 (> 0)
 max_iters?,              # default 60
 move?,                   # OC move limit, default 0.2, in (0,1]
 density_floor?,          # default 0.02, in (0,1)
 iso?,                    # threshold for the as-built check + STL, default 0.5, in (0,1)
 time_budget_s?,          # default 600; the loop stops at 0.8x
 allow_incomplete_optimization?,
 direct_solver_max_dof?}
```

**Mark load and fixture regions `frozen`.** The solver applies loads only on
ACTIVE elements, so an unprotected load region can be optimized away — a silent
way to get a beautiful, load-free answer. The validation pin freezes the clamp
root and the loaded tip slab for exactly this reason.

Receipt (last non-empty stdout line; logging on stderr): `iterations`,
`stop_reason`, `compliance_first`, `compliance_last`,
`volume_fraction_achieved`, `final_rho_npy`, `as_built {max_von_mises_pa,
max_displacement_m, n_active_elements}`, `stl {ok, watertight, volume_mm3,
num_triangles, path, mesh_upsample, mesh_voxel_mm, issues}`, `timings_s`,
`validated_range`, the `lmcad.analysis.v1` provenance envelope, and a
`determinism` block. Artifacts: `out_dir/final_rho.npy` (float32 filtered
physical density) and `out_dir/optimized.stl`.

Read `compliance_first` correctly: it is the UNIFORM-`volfrac` GRAY start, not
the solid part (manifest `caveats`). "6.6x stiffer" means than that start.

**The STL gate escalates, then refuses.** The raw optimizer field commonly
pinches at native voxel resolution (diagonal voxel contacts), so the runner pads
one voxel of air and retries at 2x and 3x grid-aligned upsampling
(`grid_mode=True`, so origin and voxel/f stay exact — no silent rescale), capped
at 48 000 000 voxels. The factor actually used is on the receipt as
`mesh_upsample`. If every rung fails, the file is deleted and the run refuses
with the kernel's own text in `mesh_issues`.

Exit contract: the shared one in `tools/_receipt.py` — **0** `ok:true`, **1** the
tool could not run the request, **2** it RAN and REFUSED or the analysis failed.
Refusal kinds: `refusal.invalid_parameter` (a non-finite control, or a control
outside its range), `refusal.optimization_incomplete`,
`refusal.manufacturing_mesh_invalid`, plus the shared `refusal.usage` /
`refusal.job_unreadable` / `refusal.job_malformed`, `timeout`, `killed.SIG*`,
`internal`. **Two parameter-validation paths, two exit codes** — verified by
running the runner 2026-09-04: `penalty: 0.5` gives
`{"error_kind": "refusal.invalid_parameter", "exit_code": 2}`, while
`volfrac: 1.5` raises a bare `ValueError` and gives
`{"error": "ValueError: volfrac must be in (0,1), got 1.5", "error_kind": "internal", "exit_code": 1}`.
Both are nonzero and both carry a receipt; match on `ok`/`error_kind`, not on
which of the two a bad number happens to take. (The runner's older header
paragraph still reads "failure => {ok:false, error}, exit 0"; that line is stale
— its own WIRE section and the measured behaviour above govern.)

Determinism: `timings_s` is the only declared non-deterministic path, but the
runner's `determinism.solver_note` is explicit that `time_budget_s` can end the
loop at a different iteration on a loaded machine and thereby move `iterations`,
`stop_reason`, `compliance_last` and the exported STL — the ANSWER, not just the
timings. Those fields are deliberately left INSIDE `core_digest`. Pin `max_iters`
and raise `time_budget_s` for a reproducible run.

## Benchmark gates (measured, frozen)

The registry records `gate: no` for `ace_optimize`
(`python3 tools/analyzer_registry.py --tier ace_optimize` returns
`"gate_suite": null`). There is no benchmark gate suite; the evidence is the
validation pin below plus the runner-contract gate.

**There is no closed-form optimal topology to pin against** — so the pin does not
claim one. It pins exact, falsifiable INEQUALITIES that any correct SIMP/OC loop
must satisfy, on a 40x8x8 mm cantilever at 1 mm voxels (clamped x=0, 10 N tip
load at x=40, tip and clamp slabs frozen, volfrac 0.4, 25 OC iterations,
E=2.2 GPa, nu=0.37), whose underlying FEA is itself pinned by
`tools/validation/ace_fea_validation.py`.

| gate | what it pins | where |
|---|---|---|
| **A — OC descent** | `compliance_last <= 0.9 x compliance_first`; **measured 3.671e-02 -> 5.554e-03 J = 0.151x** (a 6.61x compliance improvement over the uniform-0.4 gray start), 2026-07-17. A broken OC update or a flipped SIMP sensitivity sign trips it. | `tools/validation/ace_optimize_validation.py`; frozen in `tools/manifests/ace_optimize.manifest.json` → `validation.error_band.descent` |
| **B — material-removal monotonicity** | the thresholded as-built part (a strict subset of the solid domain, same grid, same load) must deflect >= the full solid beam; **measured ratio 1.58** vs the required >= 1.0. A violation means the reduced-stiffness assembly is wrong or a load landed on void. | same pin; `validation.error_band.as_built_vs_solid_deflection` |
| **C — volume honesty** | `volume_fraction_achieved` within 0.02 of the asked `volfrac`; **measured 0.400 vs asked 0.400**. Pins the OC volume bisection. | same pin; `validation.error_band.volume_fraction` |
| **D — deliverable honesty** | the receipt's STL is `ok` AND `watertight`, or the receipt says why not. | same pin (`stl.ok`, `stl.watertight`) |
| runner contract | routes through `run_cli`, no bare `sys.exit(0)` failure path, and `tools/ace_optimize_runner.py` still forwards to the moved file | `python3 tools/analyzer_registry.py --check-contract` |

Note what these four do and do not establish: they prove the loop improves,
respects physics, hits its budget and ships a valid mesh. **They do not measure
how close the result is to an optimum**, because nothing here can.

## Validity limits / out of scope

- **Compliance-optimal is NOT strength-optimal.** Nothing in the objective sees
  stress. Read `as_built.max_von_mises_pa`, and re-run [ace_fea.md](ace_fea.md)
  on the exported STL (via `tools/analyzers/voxelize_stl.py`) for the load cases
  you actually have — including the ones the optimization did not see.
- **Single static load case, compliance objective only.** No stress-constrained,
  frequency, buckling, thermal or multi-load objectives exist here. A part with
  two real load directions optimized for one of them is optimized for one of
  them.
- **Distance-to-optimum is not characterised.** No study in this repo bounds how
  far an `ace_optimize` result sits from the true optimum of its own problem, and
  the OC method is a local optimizer on a non-convex problem. Do not quote a
  percentage-optimal figure; quote the compliance improvement over the stated
  start, which is what the pin measures.
- **Resolution.** Everything in `ace_fea`'s limits carries over: features below
  ~4 voxels across are unreliable, and thin members near the voxel size are
  resolution-limited. Refine `voxel_mm` rather than trusting a sub-voxel strut.
  The pin's own geometry is a single L/h = 5 cantilever; extreme aspect grids
  inherit the FEA's discretization limits and are not separately pinned.
- **The STL is a MESH ONLY.** No density-to-B-rep reconstruction exists in this
  repo. Treat it as print/simulation geometry, not a parametric solid, and do not
  expect to fillet or dimension it downstream.
- **The as-built check is at one `iso`.** It re-analyses the thresholded design at
  `iso` (default 0.5); the sensitivity of the answer to that threshold is not
  characterised anywhere here.
- **Printability is not in the objective.** No overhang, support, minimum-feature
  or anisotropy constraint is applied during the loop; a compliance-optimal
  lattice can be unprintable. Audit the STL separately, and remember the solve is
  as-designed, not as-printed (`tools/analyzers/materials.py derated()` on the
  ALLOWABLE, not on `E`).

## When to use it

When the load path itself is the open question: a bracket, hub, mount or plate
where you know the fixtures, the loads and the mass budget but not where the
material belongs — and a rib pattern is about to be guessed. Use it to find the
structure, then gate the RESULT with the closed-form case and a fresh
[ace_fea.md](ace_fea.md) run on the exported mesh; the optimizer's own compliance
number is a search score, not a margin.

Do not reach for it to make an already-working part lighter by a few percent (the
resolution limits eat that), to satisfy a stress constraint (it has none), or when
the governing case is sustained load ([creep.md](creep.md)), resonance
([modal.md](modal.md)) or buckling ([buckling.md](buckling.md)) — compliance is
none of those.

Run: `python3 tools/analyzers/ace_optimize_runner.py job.json` ·
prove: `python3 tools/validation/ace_optimize_validation.py`
