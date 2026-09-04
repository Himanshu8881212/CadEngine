# ace_fea_tet — body-fitted tet10 elasticity: the fillet stress the voxel path cannot see

**Runner**: `tools/analyzers/ace_fea_tet_runner.py` (bridge to the in-tree
`physics.fea_tet.reference_fea_tet` + `physics.mesh_ir`, `tools/analyzers/physics/` —
numpy/scipy + gmsh) · `python3 tools/analyzers/ace_fea_tet_runner.py <job.json> [--out PATH]`
**Gates**: `python3 tools/validation/ace_fea_kt_tet_validation.py` (the Kt convergence
pin; also runs under `python3 tools/analyzer_registry.py --run-pins`, the
`physics-gate` workflow) · `python3 tools/analyzer_registry.py --check-contract`
(the runner-contract gate — `analyzers/ace_fea_tet_runner.py` is in its
`CONTRACT_RUNNERS` list). **No benchmark gate suite of its own.**
**Status**: pinned, no gate suite **Tier**: Validated
(`python3 tools/analyzer_registry.py --tier ace_fea_tet`; the registry's rule is
manifest + >=1 present validation pin, and both are present:
`tools/manifests/ace_fea_tet.manifest.json`, `tools/validation/ace_fea_kt_tet_validation.py`).

**Read [ace_fea.md](ace_fea.md) first — this card only states what the tet path
CHANGES.** Same equations, same material/fixture/load/selector job schema, same
receipt-and-exit contract; a different mesh. `ace_fea` samples geometry onto a
regular voxel grid, so a conic fillet becomes a staircase of re-entrant corners
and its peak stress is a mesh artifact: the voxel Kt pin measured scatter of
-6%..+44% that refinement never fixes (ace_fea.md, `tools/validation/ace_fea_kt_validation.py`).
`ace_fea_tet` meshes the TRUE surface with gmsh and solves quadratic tetrahedra,
so the same specimen converges monotonically to the published Kt. That is the
whole reason this second FEA exists: a campaign that gates a notch, a shoulder,
a bearing seat or any filleted transition on the voxel peak is quoting a number
the voxel pin says does not converge.

What it costs: geometry must be gmsh-meshable and watertight, the field outputs
are UNSTRUCTURED (per-node, not `(nx,ny,nz)`), and only point loads with
clamped/pinned fixtures are wired.

## The physics

Identical governing equations to `ace_fea` — linear static elasticity, small
strain, small displacement, isotropic homogeneous, no plasticity/contact/
geometric nonlinearity, static (inertia and damping ignored):

    K u = F
    sigma = D(E, nu) : eps,  eps = sym(grad u),   reduced to von Mises

**What changes is the discretization** (`tools/manifests/ace_fea_tet.manifest.json`):

- **Element**: 10-node quadratic C0 tetrahedron (tet10) on a gmsh OCC/STL
  body-fitted conforming mesh at `elem_size_mm`, `ElementOrder=2` with
  `HighOrderOptimize` to untangle curved elements. `ace_fea` uses trilinear hex8
  on the voxel grid with binary occupancy `rho >= 0.5`.
- **Boundary representation**: curved boundaries are meshed as real surfaces,
  NOT occupancy-sampled. There is no staircase, so no staircase artifact.
- **Stress recovery**: `sigma_node = mean over incident elements of D B(x_node) u_e`
  — stress evaluated at each element's 10 nodes and averaged across the elements
  meeting a node, so a surface peak is not smeared into the interior. `ace_fea`
  reports per-ELEMENT von Mises instead.
- **Solve**: SuperLU direct below `direct_max_dof` (default 250 000), Jacobi-CG
  above it (`cg_tol` default 1e-9, `cg_maxiter` default 20 000). `ace_fea`'s CG
  rtol is 1e-8.
- **No SIMP mode.** Density-based topology work stays on the voxel path
  ([ace_optimize.md](ace_optimize.md)).

Mesh quality is a real gate, not a comment: `MeshIR.check()` raises on an
inverted element before the solve, and its result (`n_tets`, `n_nodes`,
`min_corner_jacobian_mm3`, `volume_mm3`) is carried on the receipt. Pass
`volume_ref_mm3` and the mesh is cross-checked against the analytic volume.

## The contract

Job (geometry in mm, physics in SI). Material, `fixtures`, `loads` and the
selector grammar are `ace_fea`'s; the geometry block is not:

```
{out_dir, elem_size_mm,                          # REQUIRED; elem_size_mm = target tet edge (mm)
 GEOMETRY exactly one of:
   stl: "<abs path to a WATERTIGHT surface STL>",           # mesh_stl
   specimen:"shouldered_bar" + d, D, r, l_small, l_large,   # mesh_shouldered_bar (the Kt benchmark)
   specimen:"box" + lx, ly, lz,                             # mesh_box
 material: "PLA" | {youngs_modulus_pa, poisson, density_kg_m3},   # key resolves via tools/analyzers/materials.py
 fixtures: [{kind: clamped|pinned, region_selector, dof_constrained?}],   # REQUIRED
 loads?:   [{kind: "point", magnitude, direction, region_selector}],      # POINT ONLY
 volume_ref_mm3?, direct_max_dof?, cg_tol?, cg_maxiter?}
```

**Selectors are NODE-geometric and a strict subset of `ace_fea`'s**:
`{type:"all"}`, `{type:"plane", axis, value_mm, side}`, `{type:"box", min_mm, max_mm}`.
Cylinder and sphere selectors — which the voxel runner accepts — are **refused
loudly**, not silently approximated. A point load's resultant is spread equally
over the selected surface nodes (same convention as hex8).

Receipt (last non-empty stdout line; logging on stderr): `max_von_mises_pa`,
`max_displacement_m`, `n_nodes`, `n_tets`, `method`, `solver`, the `mesh` block
above, per-selector node-count receipts (`fixtures`, `loads`,
`selector_count_unit: "nodes"`), `field_layout`, `validated_range`, the
`lmcad.analysis.v1` provenance envelope and a `determinism` block.

**Field outputs are UNSTRUCTURED — the honest break from `ace_fea`:**

| file | shape | field |
|---|---|---|
| `stress_field.npy` | `(n_nodes,)` | nodal von Mises, Pa |
| `disp_field.npy` | `(n_nodes, 3)` | displacement, m |
| `nodes_mm.npy` | `(n_nodes, 3)` | node coordinates, mm |

There is **no GridField hand-off**. `ace_fea`'s per-element grids feed
`kernel_implicit::grid_field` and every grid-shaped consumer
(`tools/analyzers/stress_to_density.py`, `graded_infill`'s `stress_npy`, the
`analysis_sheet` field panels); tet fields cannot, and
`campaign/digests/tools_cookbook.md` says so explicitly. Route grading work
through the voxel runner.

Exit contract: the shared one in `tools/_receipt.py` — **0** `ok:true`, **1** the
tool could not run the request (usage / unreadable job / internal error), **2** it
RAN and REFUSED or the analysis failed. `error_kind` is machine-matchable:
`refusal.tet_solver` when the tet solver itself refuses (0-node selector,
singular system, inverted tet, unsupported load — the refusal receipt still
carries the `mesh` block), `refusal.usage` / `refusal.job_unreadable` /
`refusal.job_malformed`, `timeout` under `wall_budget_s`, `killed.SIG*`,
`internal` for anything else (a geometry block missing entirely raises
`ValueError` and therefore lands as `internal`, exit 1). `LMCAD_RUNNER_EXIT=legacy`
or `"legacy_exit_zero": true` restores exit-0-always and records
`exit_contract.mode = "legacy"`.

The receipt's `geometry_hash` is a sha256 over the EXACT solved mesh
(`nodes_mm`, `tets`, `surf_tris`, `surf_group`), not the STL or the specimen
parameters gmsh happened to generate it from — so two receipts that claim the
same geometry solved the same geometry.

## Benchmark gates (measured, frozen)

The registry records `gate: no` for `ace_fea_tet`
(`python3 tools/analyzer_registry.py --tier ace_fea_tet` returns
`"gate_suite": null`) — there is no benchmark gate suite. What exists is the
validation PIN below, plus the source-and-behaviour runner-contract gate.

| gate | what it pins | where |
|---|---|---|
| Kt convergence ladder (measured 2026-07-18) | shouldered round bar in axial tension, d=16, D=24, r=2.4 mm (D/d=1.5, r/d=0.15, h/r=1.667) vs Peterson/Pilkey chart 3.4 **Kt = 1.667**: measured Kt **1.545 (-7.3%) -> 1.610 (-3.4%) -> 1.652 (-0.9%)** at fillet element sizes 2.0 / 1.5 / 1.0 mm — monotone UP, from below, no overshoot | `tools/validation/ace_fea_kt_tet_validation.py`; the same three numbers are frozen in `tools/manifests/ace_fea_tet.manifest.json` → `validation.error_band` |
| far-field nominal | median nodal von Mises in the small shaft (3 < z < 15 mm) equals `sigma_nom = P/(pi d^2/4)` **within 2% at every element size** | same pin, gate `far_field_nominal_within_2pct_all_sizes` |
| monotonicity + no overshoot | the pin FAILS unless Kt strictly increases with refinement and the finest value lands within +/-5% of 1.667 | same pin, gates `Kt_increases_monotonically_with_refinement`, `finest_Kt_within_5pct_of_theory_no_overshoot` |
| the contrast that justifies the runner | the voxel hex8 path on the IDENTICAL specimen scatters -6%..+44%, biased high, non-convergent | `tools/validation/ace_fea_kt_validation.py`, quoted in [ace_fea.md](ace_fea.md) |
| runner contract | routes through `run_cli`, has no bare `sys.exit(0)` failure path, and the old flat path `tools/ace_fea_tet_runner.py` still forwards here | `python3 tools/analyzer_registry.py --check-contract` |

## Validity limits / out of scope

- **No deflection/stiffness pin for this path.** The cantilever pin that gives
  `ace_fea` its -11.2% / -5.9% band is a VOXEL pin
  (`tools/validation/ace_fea_validation.py`); nothing in this repo measures the
  tet path's displacement error against a closed form. The Kt pin measures a
  STRESS ratio, and its far-field gate is a nominal-stress check, not a
  deflection check. **Displacement accuracy here is not characterised** — do not
  transfer `ace_fea`'s band to a tet run.
- **Mesh dependence of the peak is real and only bounded at the pinned sizes.**
  Stress is nodal-AVERAGED recovery, not superconvergent patch recovery. The
  ladder stops at `elem_size_mm` 1.0 mm (~r/2.4) for runtime; the manifest's
  `limits_of_validity` says the fillet peak is trustworthy to a few percent at
  `elem_size <= r/2.4` and that coarser meshes under-read it. Outside that
  ratio, refine and confirm convergence yourself — an un-refined tet peak is a
  lower bound, i.e. UNconservative, the opposite of the voxel path's bias.
- **Geometry robustness is gmsh's.** A non-watertight STL fails loudly at the
  surface-loop-to-volume step rather than producing garbage; a solid gmsh cannot
  mesh is a refusal, not a result.
- **Point loads and clamped/pinned fixtures only.** Body loads and pressure
  loads are not implemented in the tet path (the voxel runner has them).
- **Cylinder/sphere selectors are unavailable**, so a job ported from `ace_fea`
  with a cylinder selector must be re-expressed as planes/boxes — a re-expression
  that is not equivalent is a DIFFERENT boundary condition, not a formatting
  change.
- **No SIMP / density mode, no thermal coupling, no dynamics.** Modal and
  buckling live on the voxel path ([modal.md](modal.md), [buckling.md](buckling.md)).
- **Isotropic, as-designed, not as-printed** — like every solver here. Layer
  anisotropy is not in the solve; apply `tools/analyzers/materials.py derated()`
  to the ALLOWABLE, never to `E`.
- Determinism: the mesh is bit-identical run to run, but the SuperLU reduction
  order is not pinned, so the runner's own `determinism.solver_note` states peak
  stress moves ~2e-14 relative between runs. Compare `core_digest`, not receipt
  bytes.

## When to use it

When the number you are about to gate on is a **peak stress at a curved feature**:
a shoulder fillet, a notch, a bolt-hole edge, a radiused root under bending, a
bearing-seat transition — anywhere the voxel Kt pin's "does not converge, scatters
-6%..+44%" verdict makes the `ace_fea` peak unquotable. Also when the part exists
only as a watertight STL from the kernel tessellator and you want its true
surface solved rather than re-occupancy-sampled.

Stay on [ace_fea.md](ace_fea.md) for: global stiffness and load paths, anything
that must hand a field to `GridField` / `graded_infill` / `stress_to_density`,
anything needing body or pressure loads or cylinder/sphere selectors, and every
SIMP topology pass. Neither runner replaces the campaign's closed-form gate
(DESIGN_GUIDE §25.7) — both sharpen it.

Run: `python3 tools/analyzers/ace_fea_tet_runner.py job.json` ·
prove: `python3 tools/validation/ace_fea_kt_tet_validation.py`
