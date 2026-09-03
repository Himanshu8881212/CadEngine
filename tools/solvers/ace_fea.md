# ace_fea — hex8 linear-elastic voxel FEA (the registry precedent)

- **Runner**: `tools/analyzers/ace_fea_runner.py` (bridge to ACE `engine.verify.reference_fea`; needs the ACE package, `ACE_ROOT`/`ACE_PYTHON`) · **Pins**: `tools/validation/ace_fea_validation.py`, `tools/validation/ace_fea_kt_validation.py` · registry manifest: `tools/manifests/ace_fea.manifest.json`
- **Physics**: linear static elasticity — small strain, small displacement, isotropic homogeneous; no plasticity, contact, or geometric nonlinearity (dynamics live in ace_modal/ace_buckling, cards owned by mission 2).
- **Governing equations**: `K u = F`; `sigma = D(E,nu) : sym(grad u)` reduced to von Mises; optional SIMP `E_eff = rho_eff^p * E0` (then reported stress is homogenized, NOT solid-material stress).
- **Discretization**: trilinear hex8 elements on the regular voxel grid; binary as-built occupancy rho >= 0.5 (or SIMP density mode). Jacobi-preconditioned CG rtol 1e-8, raises on non-convergence; direct SuperLU below opt-in DOF cap.

## I/O contract (JSON job -> .npy fields + JSON receipt on last stdout line)
```
{out_dir, voxel_mm, origin_mm?,
 ops+solid+shape[+supersample] | npy,                # LMCAD JSON ops sampled by ACE, or a density .npy (tools/analyzers/voxelize_stl.py output)
 material: "PLA" | {youngs_modulus_pa, poisson, density_kg_m3},   # string key resolves via tools/analyzers/materials.py (Unit 3)
 fixtures: [{kind: clamped|pinned|slider, region_selector, dof_constrained?}],   # REQUIRED; ACE's own selector engine
 loads?: [{kind: point|body|pressure, magnitude, direction?, region_selector}],
 regions?: [{kind: frozen|fixed|design|void, selector}], simp_penalty?, density_floor?, direct_solver_max_dof?}
```
Outputs: `stress_field.npy` (von Mises, per-ELEMENT), `disp_field.npy`. Receipt: `max_von_mises_pa`, `max_displacement_m`, `tip_displacement_m`, `n_active_elements/n_dof`, per-selector node-count receipts (loads catching > 30% of active elements are flagged "suspiciously broad"), structured convergence receipt + `lmcad.analysis.v1` provenance envelope.
**GridField hand-off**: fields are per-element; pass `origin = origin_mm + voxel/2` per `kernel_implicit::grid_field` doc.
Failure = `{ok:false, error}` + **exit 0** — the JSON line is the contract, not the exit code (MCP-bridge convention; the thermal runner deliberately differs).

## Benchmark record (pinned validations; measured values as of their pin dates)
| pin | closed form | measured | band asserted |
|---|---|---|---|
| cantilever displacement (`ace_fea_validation.py`, 2026-07-08) | Euler-Bernoulli + shear: delta = PL^3/3EI + 1.2PL/GA = 0.2934 mm (L=40, b=h=8 mm, P=10 N, E=2.2 GPa) | voxel 1.0: -11.2% · voxel 0.5: -5.9% — under-predicts (hex8 is stiff), converges from below | coarse (-20%,0), fine (-10%,0), monotone toward analytic |
| stress concentration (`ace_fea_kt_validation.py`) | Peterson/Pilkey shouldered bar in tension, Kt = 1.667 (D/d=1.5, r/d=0.15) | far-field nominal < 1% error; fillet peak scatters -6%..+44%, plateaus +20..29% under refinement — **does NOT converge to Kt** (staircase artifact, biased high = conservative) | nominal trustworthy; peak non-convergence itself pinned |

## Validity limits / out of scope
- Coarse grids under-predict bending stiffness response ~5-20% (see band above); quote feature stresses as approximate below ~4 voxels across the feature.
- Fillet/notch PEAK stress is staircase-dominated: trust only to roughly +/-20-30%, biased high. Use the closed-form Kt on the FEA's (accurate) nominal stress instead of the voxel peak.
- SIMP-mode stress is homogenized (`rho_eff^p * D B u`), not a solid-material stress.
- Isotropic material: printed-layer anisotropy is NOT in the solve — apply `tools/analyzers/materials.py derated()` to the allowable, not to E.

## When to use
Sharpening a campaign's closed-form load case (DESIGN_GUIDE §25.7): stiffness/deflection checks, load paths through brackets/hubs/lattices (voxelize_stl bridges fused hybrid meshes), SIMP topology passes via ace_optimize. Not a substitute for the closed-form gate — it sharpens, never replaces.

Run: `ACE_PYTHON tools/analyzers/ace_fea_runner.py job.json` · prove: `ACE_PYTHON tools/validation/ace_fea_validation.py && ACE_PYTHON tools/validation/ace_fea_kt_validation.py`
