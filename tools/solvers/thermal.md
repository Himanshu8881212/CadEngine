# thermal — voxel heat conduction (steady + transient)

- **Runner**: `tools/ace_thermal_runner.py` · **Gates**: `tools/test_ace_thermal.py` · in-house (NumPy/SciPy only, no ACE dependency)
- **Physics**: heat conduction in an isotropic solid; convection only as a Robin film coefficient. No radiation, no internal convection, no temperature-dependent k.
- **Governing equations**: steady `div(k grad T) + q_vol = 0`; transient `rho*cp*dT/dt = div(k grad T) + q_vol`.
- **Discretization**: cell-centered finite volume on the binary voxel grid (solid = rho >= 0.5). Interior face conductance `k*h`; Dirichlet half-cell `2k*h`; Robin series film `A/(1/h_c + h/2k)`; flux into rhs. SPD system, Jacobi-CG rtol 1e-10 (SuperLU opt-in). Transient: implicit backward Euler — **unconditionally stable** for any dt (first-order in dt); gate-probed at Fo=2.3, ~14x the 3-D explicit limit.

## I/O contract (JSON manifest -> .npy fields + JSON receipt on last stdout line)
```
{out_dir, voxel_mm, origin_mm?,                      # geometry frame in mm (origin = grid NODE (0,0,0), ACE convention)
 npy | stl+shape | shape+solid:"full",               # density grid; STL goes through tools/voxelize_stl.py (one parity-fill)
 material: "PLA" | {k_w_mk[, density_kg_m3, cp_j_kgk]},   # registry key reads tools/materials/*.json thermal block; null k refused
 bcs: [{kind:"fixed_t", t_c | kind:"flux", q_w_m2 | kind:"convection", h_w_m2k, t_inf_c,
        box_mm:[[..],[..]], faces?:"any"|["+x",...]}],    # exposed-face sets by axis-aligned box over face CENTERS;
                                                          # list order = first claim wins; 0 faces claimed = error; unclaimed = adiabatic
 sources?: [{q_w | q_w_m3, box_mm}], probes_mm?: [[x,y,z],..],
 transient?: {t_initial_c, dt_s, t_end_s, snapshot_times_s?},  # t_end_s must be an integer multiple of dt_s
 void_fill_c?, solver?: {rtol, maxiter, direct_max_dof}}
```
Outputs: `T_field.npy` (+ `T_t<t>s.npy` snapshots) — C-order float32 (nx,ny,nz), all-finite (void = `void_fill_c`, default mean solid T). Receipt: `t_min/max_c`, trilinear `probes`, per-BC face counts/areas/powers, energy balance (`residual_rel`), CG iteration counts + true residual, `grid_field{npy, origin_mm, cell_mm, ...}`.
**GridField hand-off**: field is per-VOXEL; receipt `grid_field.origin_mm` = job origin + voxel/2 = world position of sample (0,0,0) per `kernel_implicit::grid_field` — pass straight to `GridField::from_npy_file`.
Failure = `{ok:false, error}` + **exit 1** (deliberate deviation from the ACE runners' exit-0: this runner doubles as a shell gate; the negative controls pin it).

## Benchmark gates (measured 2026-07-30, all green; bands frozen from these numbers)
| gate | closed form | measured | gate band |
|---|---|---|---|
| 1a slab, fixed T both faces | linear T, Q=kA dT/L | profile 0.0e0, flux 5.0e-16 rel | <= 1e-6 |
| 1b slab + uniform source | T=(q/2k)x(L-x) | err ratios 4.05, 4.01 per h/2 (order 2.02, 2.00); nx=32 err 9.8e-4 | ratio in [3.3,4.8]; <=1.5e-3 |
| 2 cylinder wall (voxelized annulus) | ln(r/r1) profile, Q=2*pi*k*L*dT/ln(r2/r1) | h=0.25: profile 8.4e-3, flux 1.0e-2; ~O(h) staircase-limited | <=1.3e-2 / <=1.5e-2 + must improve with h |
| 3 Robin slab, Bi=4.62 | series resistance L/kA + 1/h_c A | profile 5.1e-8, flux 1.2e-12 rel | <=1e-5 / <=1e-8 |
| 4 semi-infinite step | erfc(x/2sqrt(alpha*t)) at 3 depths x 3 times | h=.25,dt=.05: 6.7e-3 of dT; energy residual 5.5e-9; stable+monotone at Fo=2.3 | <=1.2e-2; energy <=1e-6 |
| 5 negative controls | no-BC / zero-k / empty / unanchored island | all exit 1 with pointed errors | exit != 0 |
| 6 materials registry | "PLA" -> tools/materials/pla.json thermal block | k=0.13, cp=1200 resolve + solve; energy 1.0e-9 | exact record values + <=1e-6 |

## Validity limits / out of scope
- Curved Dirichlet/Robin surfaces staircase: expect ~O(h) errors, a few % at ~2 voxels per feature radius (gate 2 numbers); planar axis-aligned faces are exact to solver tolerance.
- Backward Euler is first-order in dt: early-time step responses need dt << t of interest (gate 4: dt=t/40 gave 0.7% of dT).
- Single material per job; k constant (no T-dependence); no radiation/enclosure exchange, no phase change, no contact resistance between parts, no convection network (h_c is user-supplied).
- Steady problems need every connected solid component anchored by a fixed_t/convection bc (refused otherwise, not guessed).

## When to use
Printed-part service-temperature questions: heat-soak time of a spool hub in a drybox, temperature at a bearing seat near a heat source, whether a PLA bracket's hot side exceeds `thermal.softening_c` (pair with the creep table in `tools/materials/pla.json`). Feed the field back into geometry via GridField grade laws (e.g. thicken hot zones).

Run: `python3 tools/ace_thermal_runner.py job.json` · prove: `python3 tools/test_ace_thermal.py`
