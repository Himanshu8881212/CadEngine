# LMCAD tools/ cookbook — analysis, optimization, checks, documentation

Reference for AI design agents running part-design campaigns. Everything here was read from
source and the marked (VERIFIED) examples were actually executed on 2026-08-06.

- Repo root: `/Users/himanshu/Work/New-LMCAD/cad engine` — **the path has a space, always quote it**.
- Engine CLI: `"/Users/himanshu/Work/New-LMCAD/cad engine/target/release/kernel-api" run <program.json> --out-dir <dir>`
  (note the `run` subcommand; there is also `asm` for assemblies).
- All Python tools: `python3 "/Users/himanshu/Work/New-LMCAD/cad engine/tools/<tool>.py" <job.json>`
  — plain `python3` works; the runners put `~/Work/ACE` on `sys.path` themselves (`ACE_ROOT` env
  overrides) and default `LMCAD_KERNEL_API` to the repo's `target/release/kernel-api`.
- **Where the tools live (re-organised 2026-09-02, map in `tools/_layout.py`):**
  `tools/analyzers/` (every registered analysis surface: the `ace_*_runner.py` solvers,
  `graded_infill_runner.py`, `param_optimize.py`, the checkers `tolerance_stack` /
  `production_check` / `joint_check` / `sweep_check` / `balance_check` / `air_topology_audit`,
  `derived_model.py`, `materials.py`, `_ace.py`, `voxelize_stl.py`, `stress_to_density.py`),
  `tools/publish/` (`render_sheet`, `render_views`, `analysis_sheet`, `assembly_doc`,
  `motion_gif`, `production_dossier`, `document_bundle`, `make_all_plate`, `bom_audit`),
  `tools/validation/` (the `*_validation.py` pins), `tools/tests/` (the gate suites), and
  at the top level the shared contracts (`_receipt.py`, `_stl.py`, `provenance.py`,
  `analyzer_registry.py`, `check_ci_security.py`) plus data (`manifests/`, `materials/`,
  `material_db.json`). **Every old flat path still works**: `tools/<name>.py` is a forwarding
  shim that runs the real file with the same argv, stdout and exit code (and hands back the
  real module on `import`), so `python3 tools/tolerance_stack.py job.json` and
  `python3 tools/analyzers/tolerance_stack.py job.json` are the same command with the same
  receipt (proven byte-identical on 2026-09-02; CI re-proves one shim on every PR). Edit the
  real file, never the shim. `tools/_parked/` holds orphaned tools that are off the surface.
- **Engine op args are FLAT, not nested under `params`**: `{"id":"b","op":"box","min":[0,0,0],"max":[40,10,6]}`.
  `export_stl` takes `file` (not `path`); `cylinder` takes `base`+`axis` (3-vector)+`radius`+`height`.
- The engine and several tools **ignore unknown JSON keys silently** — a typoed param name is not an
  error, it is a silently-applied default. Copy field names from this cookbook exactly.

## Universal wire contract

Every tool: **the LAST non-empty stdout line is ONE JSON receipt**; all logging goes to stderr.
Parse only that line.

**ONE contract for every runner in `tools/`** since 2026-08-08 — defined and implemented in
`tools/_receipt.py`, whose docstring is the authority:

| exit | `ok` | meaning | `error_kind` |
|---|---|---|---|
| **0** | `true` | analysis ran, receipt usable | — |
| **1** | `false` | the tool **could not run the request** (usage / unreadable job / internal). **No analysis performed.** | `usage`, `internal`, `receipt_path_conflict` |
| **2** | `false` | the tool **RAN and REFUSED**, or the analysis failed | `refusal.*`, `timeout`, `killed.*` |

Any nonzero exit means "do not quote this receipt". Both signals always agree. Every receipt
carries the contract inline so it is self-describing:

```json
"exit_code": 2,
"error_kind": "refusal.JobError",
"exit_contract": {"mode":"strict","code":2,
  "meaning":"ok:false — tool ran and REFUSED, or the analysis failed; see error_kind",
  "contract":"0 = ok:true; 1 = tool could not run the request; 2 = ran and REFUSED / analysis failed. Any nonzero: do not quote this receipt.",
  "opt_out":"LMCAD_RUNNER_EXIT=legacy (env) or \"legacy_exit_zero\": true (job key) …"}
```

Verified transcript (2026-08-08): `ace_fatigue_runner.py` on a top-level `sigma_ref_mpa` →
`{"ok": false, "error": "JobError: stress block required…", "error_kind": "refusal.JobError"}`,
**exit 2**. `tolerance_stack.py` on a zero-clearance fit → `ok:false`, **exit 2**; on
`--nope` → `error_kind: "refusal.usage"`, **exit 1**; on a passing fit → **exit 0**.

**This is a behaviour change, deliberately.** The six ACE runners used to exit 0 on `ok:false` by
design. The old rule "parse `ok`, never `$?`" still works — `ok` never moved — and now `$?` works
too, with `1` vs `2` telling you whether anything was attempted. A caller that truly needs the old
behaviour sets `LMCAD_RUNNER_EXIT=legacy` or `"legacy_exit_zero": true`, and the receipt records
`exit_contract.mode = "legacy"` so the opt-out is on the record. Campaigns should never set it.

Branch on `error_kind`, never on prose. Still gate on all three: the last stdout line parses as
JSON, `ok` is true, and the artefacts the receipt names exist on disk.

Receipt persistence (`tools/_receipt.py`): the stdout line stays the wire contract; the checker
tools (tolerance_stack, balance_check, joint_check, sweep_check, production_check, dim_suggest,
document_bundle) ALSO write the receipt to disk. Destination order: `--out PATH` > job key
`"receipt": "<path>"` (relative joins `out_dir`, else CWD) > `<out_dir>/<tool>_receipt.json`
(checkers only — the physics runners keep this default OFF) > stdout only.

- `--out PATH` writes **atomically** (temp + rename) at the END of the run, so it can never
  truncate a good receipt the way `| tail -1 >` does. **Use it instead of a shell redirect.**
- `--out` and a job `"receipt"` key that DISAGREE are **REFUSED** (exit 1,
  `error_kind: receipt_path_conflict`) instead of the job key quietly winning — that silent
  clobber destroyed a shipped receipt in one campaign.
- `LMCAD_RECEIPT_DRY_RUN=1` suppresses every on-disk write: a what-if run against a shipped
  campaign can no longer mutate its evidence.
- `"wall_budget_s"` in the job (or `LMCAD_WALL_BUDGET_S`) plus SIGTERM/SIGINT synthesize an
  honest `ok:false` receipt naming the limit (`error_kind: "timeout"` / `"killed.<signal>"`)
  instead of a bare traceback and a missing file. SIGKILL — what `subprocess.run(timeout=)`
  sends — is uncatchable, which is exactly why the budget belongs INSIDE the runner. **Put a
  wall budget on every long solve** (especially `ace_fea_tet`).
- The on-disk form is indented and human-diffable; the stdout form stays one line.

Every receipt also carries a `determinism` block — see "Determinism" in `analysis_honesty.md`.
**Compare `determinism.core_digest` between runs, never the receipt bytes.**
The ACE physics runners do NOT self-persist their receipt (their field `.npy` files land in
`out_dir`; capture stdout yourself).

**Refusals are first-class answers.** These tools refuse loudly instead of guessing: modal with no
fixtures and no `free_free:true`; buckling with zero/no-compressive load; fatigue for any material
whose printed S-N status is not `measured` (currently only **PLA** is `measured`) and for
`load_orientation:"across_layer"` (always); thermal for a material with null conductivity; the tet
FEA for cylinder/sphere selectors. Treat `{ok:false}` + a reasoned error as evidence, not a crash.

## Materials — one source of truth

`tools/analyzers/materials.py` reads `tools/materials/{pla,petg,abs,asa,pc,pa,tpu95a}.json`.
Anywhere a job wants `"material"`, a **string key** (`"PLA"`, case-insensitive, aliases
TPU→TPU95A, NYLON→PA, PET-G→PETG) resolves to the registry record; a pasted dict
`{youngs_modulus_pa, poisson, density_kg_m3}` passes through unchanged.
Verified: `materials.get("PLA").fea_material()` → `{youngs_modulus_pa: 3.3e9, poisson: 0.36, density_kg_m3: 1240.0}`; `yield_mpa = 55.0`.
`production_check` uses a separate `tools/material_db.json` (keys PLA|PETG|ABS|ASA|TPU95A|PC|PA).
Fatigue data lives in `tools/materials/fatigue.json` (statuses: PLA=measured, PETG/ABS=insufficient,
ASA/PA/PC/TPU95A=unknown → refused).

## Selectors (voxel physics runners)

`region_selector` types: `all` | `bbox` | `plane` | `cylinder` | `sphere` (`shell` deliberately
raises NotImplementedError — unverifiable). **Plane side is `"+"` or `"-"`, NOT "above"/"below"**
(VERIFIED refusal: `plane selector side must be '+'|'-', got 'above'`). Geometry keys in mm.
The tet runner supports only `all` | `plane` (`{axis,value_mm,side}`) | `box` (`{min_mm,max_mm}`).
Any load selector catching >30% of active elements gets a "suspiciously broad" note in the receipt
(the smeared-load mistake behind an earlier 3x-wrong benchmark). A selector catching 0 nodes errors.

## Geometry blocks (shared by voxel runners)

One of:
- `ops` + `solid` (op id) + `shape` `[nx,ny,nz]` [+ `supersample` default 2] — LMCAD ops sampled via ACE's `sample_part`;
- `npy` — absolute path of an existing `(nx,ny,nz)` float density grid;
- `stl` + `shape` — (modal/buckling/thermal only) watertight STL parity-filled via `tools/analyzers/voxelize_stl.py`.
Plus `voxel_mm` (REQUIRED) and `origin_mm` (default `[0,0,0]`, world coord of grid node (0,0,0)).

---

# 1. PHYSICS

## ace_fea_runner.py — hex8 voxel linear-elastic FEA (needs full ACE)  [tier: Validated]

`python3 tools/analyzers/ace_fea_runner.py job.json`

Job: `out_dir`*, `voxel_mm`*, `origin_mm`?, GEOMETRY (ops+solid+shape | npy), `regions`?
(`[{kind: frozen|fixed|design|void, selector}]`), `material`* (key or dict), `fixtures`*
(`[{kind: clamped|pinned|slider, region_selector, dof_constrained?}]`), `loads`?
(`[{kind: point|body|pressure, magnitude, direction (unit 3-vec, point/body), region_selector}]`
— **units: `point` = N (total force over the selected nodes); `body` = N/kg, i.e. an ACCELERATION,
NOT force-per-volume; `pressure` = Pa**), `simp_penalty`? (null=binary occupancy), `density_floor`? (0.02),

> **`body` load unit — the 1240× trap.** The solver computes per-node force =
> `magnitude` × direction × **tributary MASS** (`~/Work/ACE/engine/verify/fea.py:1037`:
> *"Body load: per-node force = magnitude(N/kg) * direction * tributary mass"*; `fea_tet.py:326`
> states the same). So for a **2 g** handling case pass `magnitude = 19.62` (= 2 × 9.81 N/kg),
> **not** 24328.8 (= 2 g × ρ 1240 kg/m³, which earlier revisions of this digest implied by writing
> the unit as N·m⁻³). Passing the density-multiplied value over-loads by exactly ρ — one campaign
> read 92.9 MPa for a 2 g shake, `ok:true`, exit 0, nothing warned. Sanity-check every body-load
> receipt: implied total force ≈ `magnitude` × mass(kg), and mass = ρ × volume.
`direct_solver_max_dof`? (0 = always Jacobi-CG; SuperLU needs ~10 GB at 237k DOF).

Receipt: `{ok, max_von_mises_pa, max_displacement_m, tip_displacement_m, n_active_elements, n_dof,
method, fixtures/loads node-count receipts, selector_count_unit:"nodes", notes, stress_field_npy,
disp_field_npy, timings_s, compliance?, geometry_hash, residual_or_convergence, analysis_envelope}`.
Fields: `stress_field.npy`/`disp_field.npy` are structured `(nx,ny,nz)` grids in out_dir.

VERIFIED (40×10×6 mm PLA cantilever, voxel 1.0, 10 N tip): `max_von_mises_pa≈5.23e6`,
`tip_displacement_m≈2.90e-4`, 0.14 s total.

Honesty: coarse hex8 **under-predicts** peak bending stress ~20% (converges from below); SIMP-mode
stress is homogenised, not solid-material; stair-step voxel boundaries add spurious surface
concentrations. Manifest: `tools/manifests/ace_fea.manifest.json`.

Minimal job (VERIFIED):
```json
{"out_dir":"/abs/fea_out","voxel_mm":1.0,
 "ops":[{"id":"b","op":"box","min":[0,0,0],"max":[40,10,6]}],"solid":"b","shape":[40,10,6],
 "material":"PLA",
 "fixtures":[{"kind":"clamped","region_selector":{"type":"plane","axis":"x","value_mm":0.5,"side":"-"}}],
 "loads":[{"kind":"point","magnitude":10.0,"direction":[0,0,-1],
           "region_selector":{"type":"plane","axis":"x","value_mm":39.5,"side":"+"}}]}
```

## ace_fea_tet_runner.py — body-fitted tet10 FEA (needs ACE + gmsh)  [Validated]

`python3 tools/analyzers/ace_fea_tet_runner.py job.json` — the curved-geometry twin: true surfaces, resolves
fillet stress concentrations the voxel path under-reads.

Job: `out_dir`*, `elem_size_mm`*, GEOMETRY one of `stl` (watertight) | `specimen:"shouldered_bar"`
(+`d,D,r,l_small,l_large`) | `specimen:"box"` (+`lx,ly,lz`), `material`*, `fixtures`*
(clamped|pinned only), `loads`? (**point only** — body/pressure unsupported), `volume_ref_mm3`?,
`direct_max_dof`? (250000), `cg_tol`? (1e-9), `cg_maxiter`? (20000).

Receipt adds `mesh` `{n_tets,n_nodes,min_corner_jacobian_mm3,volume_mm3}` and `field_layout`:
**fields are UNSTRUCTURED** — `stress_field.npy (N_nodes,)` nodal von Mises Pa,
`disp_field.npy (N_nodes,3)` m, `nodes_mm.npy (N_nodes,3)` mm. Do not feed these to grid-shaped
consumers (analysis_sheet field panels, stress_to_density) — those want the voxel runner's grids.

## ace_modal_runner.py — hex8 modal (frequencies + mode shapes) (needs ACE)  [Validated]

`python3 tools/analyzers/ace_modal_runner.py job.json`

Job: `out_dir`*, `voxel_mm`*, GEOMETRY (ops|npy|stl+shape), `regions`?, `material`* (**density must
be > 0**), `fixtures`* UNLESS `free_free:true` (fixtures+free_free together = refused; neither =
refused, never a silent fallback), `n_modes`? (6). `simp_penalty` is ignored with a note.

Receipt: `{ok, frequencies_hz (elastic ascending), first_mode_hz, rigid_body_modes_hz, boundary,
participation [{mode,f_hz,effective_mass_kg/fraction,kinetic_fraction per xyz}],
total_active_mass_kg, mode_shapes {layout, files: mode_shape_NN.npy}, eigensolve, n_free_dof, ...}`.
Mode shapes are `(nx,ny,nz)` float32 unit-max magnitude grids, GridField-compatible.
Honesty: no damping/preload; lumped-mass hex8 reads a few % HIGH, converges down.

## ace_buckling_runner.py — hex8 linear (eigenvalue) buckling (needs ACE)  [Validated]

`python3 tools/analyzers/ace_buckling_runner.py job.json`

Job: like ace_fea plus geometry via ops|npy|stl+shape; `loads`* REQUIRED (the reference load case;
all-zero refused; moment loads skipped — no rotational DOF), `n_modes`? (4), `knockdown`? (0<k≤1,
default 0.5), `direct_solver_max_dof`?.

Receipt: `{ok, caveat, load_factors, buckling_load_factor, applied_reference_load_N,
critical_load_N, knockdown {recommended_factor, design_critical_load_n, why, sources},
prestress {max_von_mises_pa, ..., disp/stress_field_npy}, ...}`.
**Interpretation is mandatory**: linear buckling is an UPPER bound; design load = knockdown ×
critical (default 0.5 for FDM; AISC/EN/NASA sources cited in receipt). A yield-before-buckling note
is appended when the scaled prestress exceeds yield (registry-key materials only). Refuses honestly
when no compressive stress / no positive eigenvalue. Error band measured: +2..+6% on slender
columns, ACE docstring says up to +10..30% coarse.

## ace_thermal_runner.py — voxel heat conduction, steady + transient (numpy/scipy ONLY)  [not yet in registry]

`python3 tools/analyzers/ace_thermal_runner.py job.json` — in-house FV solver, no ACE import. **Exit 1 on failure.**

Job (temps °C): `out_dir`*, `voxel_mm`*, `origin_mm`?, GEOMETRY (npy | stl+shape | shape+`solid:"full"`),
`material`* (key with non-null `thermal.conductivity_w_mk`, or dict `{k_w_mk[,density_kg_m3,cp_j_kgk]}`
— latter two required for transient), `bcs`* `[{kind, box_mm:[[x0,y0,z0],[x1,y1,z1]], faces?}]` with
kinds `fixed_t {t_c}` | `flux {q_w_m2, +into solid}` | `convection {h_w_m2k, t_inf_c}` — applied to
EXPOSED voxel faces whose centers fall in box_mm; first claim wins; a bc claiming 0 faces errors;
unclaimed faces adiabatic. `sources`? `[{q_w|q_w_m3, box_mm}]`, `probes_mm`?, `transient`?
`{t_initial_c, dt_s, t_end_s, snapshot_times_s?}` (t_end must be integer multiple of dt),
`void_fill_c`?, `solver`? `{rtol 1e-10, maxiter 20000, direct_max_dof 0}`.

Receipt: `{ok, mode, n_solid_voxels, n_dof, material, method, cg_iters(_total/_max_per_step),
true_residual_rel, bc_receipts (power per bc), energy {balance receipt}, snapshots?, t_min_c,
t_max_c, probes, grid_field {origin_mm shifted +voxel/2 — cell centers, ready for
GridField::from_npy_file}, timings_s}`. Fields: `T_field.npy`, `T_t<t>s.npy`.

## ace_contact_runner.py — 2-D corotational beam + rigid contact (numpy ONLY)  [not yet in registry]

`python3 tools/analyzers/ace_contact_runner.py job.json` — snap-fits, latch arms, living hinges: large
rotation load PATHS that linear voxel FEA gets wrong. **Exit 1 on failure**; no silent last-iterate.

Job (mm, N, N·mm): `out_dir`*, `beam`* `{length_mm, n_elements}|{nodes_mm:[[x,y],..]}` +
`section {width_mm, thickness_mm}` or taper `{width_mm, root_thickness_mm, tip_thickness_mm}`,
`material`* ("PLA" or `{youngs_modulus_pa[,yield_mpa,ultimate_mpa]}`), `supports`*
`[{node:"root"|"tip"|int|{at_mm}, dofs:{ux?,uy?,rz?}, ramped?}]`, `loads`? `[{node, fx_n?, fy_n?,
mz_nmm?}]` (λ-scaled, fixed direction), `obstacles`? `[{kind: plane|cylinder|profile, ...,
penalty_n_per_mm, friction?:{mu,k_t_n_per_mm}, motion?:{dir,travel_mm}, nodes?}]`,
`steps`? `{n:20, max_iter:30, tol:1e-8, ...}`, `linear_reference`? (true).

Receipt includes `curve_columns` (16 named columns: lambda, insertion_force_n, tip_ux/uy_mm,
max_abs_stress_mpa, max_penetration_mm, ..., reaction_fx_n/reaction_fy_n/reaction_mz_nmm,
tip_reaction_n) for `curve.npy` `(n_steps+1, n_cols)`.
**Reactions (2026-09-02):** `reactions` = one row per support node at λ = 1
`{node, dofs_constrained, ramped, prescribed, fx_n, fy_n, mz_nmm}` (force BY the support ON the
beam — `reaction_convention` spells it out). For a prescribed displacement
(`{"node":"tip","dofs":{"uy":-1.0},"ramped":true}`) that row IS the actuator force, and
`tip_reaction_n` is the scalar (null under load control; its curve column is NaN there, never a
silent 0). `linear.tip_reaction_n` sits beside it. Pinned: 3EId/L³ within 2 % at d/L = 0.01
(gate 1b, measured 1e-4; linear exact).
Limits: planar, Euler-Bernoulli, node-based contact, quasi-static, isotropic.

## ace_fatigue_runner.py — S-N (Basquin + Miner) post-processor (numpy ONLY)  [not yet in registry]

`python3 tools/analyzers/ace_fatigue_runner.py job.json` — consumes a prior ace_fea `stress_field.npy` or a
scalar. **Exit 1 on failure/refusal.** SCREENING tool: ranks variants, never certifies life.

Job: `out_dir`*, `material`* (registry key — only PLA currently has `measured` printed S-N data;
everything else REFUSED — or inline `{curve:{a_mpa,b,stress_measure,n_valid?}, sigma_uts_mpa}`),
`curve`? ("design" PS≥90% default | "median"), `load_orientation`? ("in_plane" default;
"across_layer" ALWAYS refused),

> **`stress` is a NESTED BLOCK, not top-level keys** — the single most-hit line in this cookbook
> (three campaigns). The runner reads `job.get("stress")` (`tools/analyzers/ace_fatigue_runner.py:283`).
> ```json
> "stress": {"sigma_ref_mpa": 4.15}          ✅
> "sigma_ref_mpa": 4.15                       ❌ → {"ok": false, "error":
>                                                "JobError: stress block required:
>                                                 {npy,...} or {sigma_ref_mpa} or {sigma_ref_pa}",
>                                                "error_kind": "refusal.JobError"}   (exit 2)
> ```
> The block is one of `{"npy": <path>, "unit"?: "pa"|"mpa"}` | `{"sigma_ref_mpa": <f>}` |
> `{"sigma_ref_pa": <f>}`. Both forms above verified 2026-08-08.

`stress`* (block, above), `spectrum`* `[{cycles, load_factor?, r_ratio?}]` or `[{cycles, sigma_a_mpa,
sigma_m_mpa?}]` (field mode must declare r_ratio; default 0.0 zero-tension), `mean_stress`?
(goodman|gerber|none|intrinsic; stacking on a curve that absorbs mean stress = refused;
compressive mean gets NO credit), `sigma_uts_mpa`?, `damage_limit`? (1.0).
A block above printed UTS is refused (that's static failure).

Receipt: `{ok, method, curve, blocks[...], damage {total_at_critical_location, life_status,
spectrum_repeats_to_failure, cycles_to_failure, critical_index}, confidence {life_scatter_factor_90_10
— printed-PLA band spans 3.7x..90x in life, statement}, damage_field_npy? }`.

## ace_optimize_runner.py — SIMP topology optimization (needs ACE)  [Validated]

`python3 tools/analyzers/ace_optimize_runner.py job.json` — top88-lineage SIMP + OC over ACE hex8 FEA, then
one honest binary-occupancy re-analysis of the thresholded design + watertight-or-fail STL.

Job = ace_fea job + `volfrac`* (0..1), `penalty`? (3.0), `filter_radius_vox`? (1.5), `max_iters`?
(60), `move`? (0.2), `density_floor`? (0.02), `iso`? (0.5), `time_budget_s`? (600, stops at 0.8×).
**Mark load/fixture regions `frozen`** or they can be optimized away.

Receipt: `{ok, iterations, stop_reason, compliance_first/last, volume_fraction_achieved,
final_rho_npy, as_built {max_von_mises_pa, max_displacement_m, n_active_elements}, stl {ok,
watertight, volume_mm3, num_triangles, path, mesh_upsample, issues}, timings_s}`. STL is a MESH
ONLY — no B-rep reconstruction exists.

## graded_infill_runner.py — stress-graded gyroid infill (needs ACE + the kernel-api binary)  [Demonstrated]

`python3 tools/analyzers/graded_infill_runner.py job.json` (or `--selftest`)

Job: `out_dir`*, `voxel_mm`*, GEOMETRY (ops+solid+shape | npy), `stress_npy`* (ace_fea field ON THE
SAME GRID — shape mismatch refused, never resampled), `cell_mm`? (8.0), `wall`? `{min:0.8, max:2.4}`,
`stress_map`? `{lo_pct:20, hi_pct:95}`, `shell_mm`? (1.5), `iso`? (0.5), `file`? ("graded_infill.stl").
Receipt: `{ok, volume_mm3, ...}`; meshed by the kernel (dual contour, watertight-or-fail, escalates
voxel/2, voxel/3). Wall thickness is volume-calibrated ±10% local variation; verify supports in a slicer.

---

# 2. OPTIMIZATION

## param_optimize.py — universal parametric optimizer (stdlib; drives the engine or ANY analyzer)  [Validated]

`python3 tools/analyzers/param_optimize.py job.json`

Any string `"$name"` anywhere in the template ops is replaced by the param value. Nelder-Mead with
bound clipping + constraint penalties; feasibility-first selection.

```json
{"template":{"ops":[{"id":"b","op":"box","min":[0,0,0],"max":["$w",10,6]},
                    {"id":"mp","op":"mass_properties","in":"b"}]},
 "params":{"w":{"min":10,"max":60,"init":30}},
 "objective":"mp.volume","maximize":false,
 "constraints":[{"expr":"mp.volume","min":1200}],
 "max_evals":12}
```
VERIFIED → `{ok:true, best_params:{w:21.0}, best_objective:1260.0, constraint_ok:true, evals:12, ...}`.

Objectives/constraints are dotted expressions over op measures (`mp.volume`,
`mp.inertia_diag[2]`). v2 additions (all optional): `evaluator {kind:"command", argv:[...,"$JOB"],
job_template:{...}, timeout: <seconds, default 300>}` to optimize over ANY receipt-emitting
analyzer (physics-in-the-loop) — **set `timeout` explicitly.** Every candidate is wrapped in
`subprocess.run(..., timeout=float(ev.get("timeout", 300)))` (`tools/analyzers/param_optimize.py`), so a
physics-in-the-loop evaluator that takes longer than 300 s per candidate — which any real solver
loop does — dies unless you raise it. Campaigns use 1800;
`targets [{expr,value,tol,weight}]`; `objectives [...]` (weighted sum — reported per-term, NOT a
Pareto front); `multi_start N` (deterministic); `robust {tols:{...}, aggregate:"worst"}` (worst-case
over tolerance corners). Uses `kernel-api` CLI per eval (uncapped reports; MCP fallback caps at 60 KiB).

## derived_model.py — scaffold for agent-derived physics models  [exemplar: Demonstrated]

`python3 tools/analyzers/derived_model.py job.json | --selftest | --manifest OUT | --new NAME`

Subclass `DerivedModel`: cannot exist without equations/assumptions/units/limits/SOURCES
(`__init_subclass__` refuses at import). Every run re-executes validation gates; failing gates =
refuse to evaluate. Emits the `lmcad.analysis.v1` envelope with status `synthesized_inloop` (can
NEVER claim `validated` — that needs a committed manifest + pin). Last stdout line is the envelope
JSON, so param_optimize's command evaluator drives it directly. `--new NAME` scaffolds a model file.

## stress_to_density.py — stress .npy → graded density .npy (numpy)

`python3 tools/analyzers/stress_to_density.py stress.npy [--floor 0.15] [--ceil 1.0] [--gamma 1.0] [--clip-percentile 99]`
(flags, not a job file). Writes `<stem>_density.npy` (float32, same shape) next to input.
Receipt: `{ok, in, out, shape, stress_min, stress_max_clipped, floor, ceil, gamma, clip_percentile,
density_mean}`. Non-finite voxels refused. Feeds `GridField` grading (`Node::offset_by`).

## voxelize_stl.py — watertight STL → binary occupancy .npy on an EXPLICIT grid (numpy)

`python3 tools/analyzers/voxelize_stl.py job.json`;
job `{stl, origin_mm:[x,y,z], voxel_mm, shape:[nx,ny,nz], out}`.
VERIFIED: box STL → `{ok:true, solid_voxels:2320, solid_fraction_mean:0.967, bytes:9600}`.
The bridge for full-geometry physics on hybrid/mesh parts; keeps the same grid frame as a FEA job
so selectors keep world coordinates. Struts under ~4 cells: quote stresses as approximate.

---

# 3. FIT / MOTION / ASSEMBLY CHECKS

## tolerance_stack.py — 1-D stack-up + bore/shaft fit (pure stdlib)  [Validated — arithmetic pinned, not physics]

`python3 tools/analyzers/tolerance_stack.py job.json [--out PATH]` — 0/1/2 exit contract; persists receipt.

CHAIN mode: `{"chain":[{name, nominal, tol (±t or {plus,minus}), dir:±1}...],
"closes":{min_required, max_allowed}, "printer_tol_default":0.15}` — worst-case AND RSS (±t = 3σ).
FIT mode: `{"fit":{"bore":{nominal,tol?},"shaft":{nominal,tol?}}}` — omitted tol takes the
0.15 mm FDM default. Receipt: chain `{nominal_gap, worst_min/max, rss_min/max, pass_worst,
pass_rss, contributors ranked}`; fit `{min/max_clearance, interference_at_extremes, pass}`.
`ok` = all requested modes pass.

VERIFIED gotcha: bore 8.2 / shaft 8.0 with default ±0.15 tolerances → `interference_at_extremes:
true`, `ok:false`. A 0.2 mm nominal clearance is NOT enough at default FDM tolerance — design
clearances ≥ 0.3 mm or state tighter tols.

## sweep_check.py — parameter-sweep interference check (stdlib + engine)  [Demonstrated]

`python3 tools/analyzers/sweep_check.py job.json`. Template ops contain `"$t"`; at each station the engine
runs and each watched `clearance`/`coincident_fit` op is collected.
Job: `{template:{ops:[...]}, t:{from,to,steps<=200}, watch:[op ids], out_dir}`.
Receipt: `{ok (false iff any watch interferes anywhere or a station failed), watches:{id:
{min_distance, min_distance_t, first_interfering_t, interfering_t_ranges, table_path csv}},
failed_stations}`.

## balance_check.py — rotating-assembly imbalance (stdlib + engine)  [Demonstrated]

`python3 tools/analyzers/balance_check.py job.json`
Job: form 1 `{ops, solid, density_kg_m3}` or form 2 `{parts:[{name,ops,solid,density_kg_m3}...]}`,
plus `spin_axis {point, dir}`, `spin_rpm`?.
Receipt: `{ok, mass_g, cg_mm, cg_offset_mm, static_imbalance_g_mm, couple_terms {I_uw_g_mm2,
I_vw_g_mm2, magnitude}, est_wobble_force_N_at_rpm?, per_part}`. Measures only — imposes no grade.
VERIFIED on a centered box: all zeros, `ok:true`.

## joint_check.py — fastener/insert rules engine (pure stdlib)  [Cataloged]

`python3 tools/analyzers/joint_check.py job.json`
Job: `{joints:[{name, type: machine_screw_into_heatset|screw_into_plastic_thread|bolt_through_nut,
size: M3|M4|M5, material: pla|petg|abs|asa|pc|nylon (the PLASTIC side), loads:{tension_N, shear_N,
sustained}, engagement_mm, insert_len_mm?}], safety_factor: 2.0}`.
Modes checked: heat-set pull-out, thread strip, bearing/shear, steel screw allowables, min
engagement (≥2d plastic / ≥ insert length), sustained ×0.25 creep derate, combined
`1/sqrt((T/Tc)²+(S/Sc)²)`. Receipt: `{ok, joints:[{governing_mode, SF_actual, pass, modes, notes}]}`.
All numbers TYPICAL published values — verify per brand.
VERIFIED: M3 heatset in PLA, 100 N tension + 50 N shear → SF 3.39, pass.

## air_topology_audit.py — internal-air connectivity gate (numpy + scipy)  [Demonstrated]

`python3 tools/analyzers/air_topology_audit.py job.json` — the TL-91 lesson: watertight/geometric_ok gate the
MATERIAL, not the VOID. Voxelizes an STL, flood-labels internal air, verifies the functional air
path is one component between named seeds.
Job: `{stl, voxel_mm:1.0, wall_margin_mm:7, open_faces:["y+"], seeds:{name:[x,y,z]},
require_connected:[["chamber","port"]], front_openings_face:"y-"|null}`.
Receipt: `{ok, components, sizes_cm3, seed_labels, connected:{...}, openings}` — ok:false if any
required pair is disconnected.

---

# 4. PRINT / PRODUCTION GATES

## production_check.py — FDM derated-allowables gate on FEA results (pure stdlib)  [Validated — table arithmetic pinned, not physics]

`python3 tools/analyzers/production_check.py job.json` (also `--selftest`)
Job: `{material* (PLA|PETG|ABS|ASA|TPU95A|PC|PA), max_von_mises_pa* (from ace_fea),
load_character?:{sustained,cyclic}, **duration_h — REQUIRED when sustained**
(also accepted as service.duration_h / load_character.duration_h),
service_temp_c?:25, orientation?:{build_dir, primary_load_dir},
safety_factor_required?:2.0, creep_interpolation?:false}`.
`creep_interpolation:true` (2026-09-02, opt-in) reads the creep allowable INTERPOLATED between
the bracketing table cells (linear in T, log-linear in time: 30 °C/24 h → 4.234375 MPa instead of
the default 55 °C-row 1.5 MPa); the receipt says so (top-level flag, the creep row's
`creep_interpolation`, `creep_cell.basis: "interpolated"`, both cells, the formula, and the
default bucket beside it). Never extrapolates (70 °C still refuses); quote it as interpolated.
Rules: static (yield), creep (the material's `creep.sig_allow_mpa`
temperature×duration TABLE at the stated service_temp_c and duration_h,
sustained — the legacy `yield × creep_sustained_fraction` scalar is reported
as `legacy_scalar_mpa` but NEVER gates), fatigue (ultimate × knockdown,
cyclic), temp limit, anisotropy (>30° out-of-plane load →
layer_adhesion_factor on everything; skipped WITH a note if orientation
absent). Receipt: `{ok (overall verdict), rules:[{rule, allowable_mpa,
demand_mpa, SF, pass, detail}], skipped, notes, disclaimer}`.

VERIFIED gotcha (updated 2026-08-27, rated_desk_hook friction F1):
`sustained: true` with **no `duration_h`** → the creep rule REFUSES
(`refusal_kind: "creep_duration_required"`, `allowable_mpa: 0.0`, `ok:false`,
exit 2) — a creep allowable is a function of temperature AND time, and the
time-blind 11 MPa scalar this tool used to apply is non-conservative at long
duration (PLA table: 5.0 MPa at 23 °C/24 h, 2.5 at 1 y). State the design
duration; sustained-load PLA still fails early — design to ~5 MPa (24 h) /
~2.5 MPa (1 y) or switch material. Demand also inherits ace_fea's ~20%
under-prediction: margin accordingly.

## production_dossier.py — BOM cost + print-time + plate packing (numpy)  [Validated — bookkeeping pinned to analytic boxes, not physics]

`python3 tools/publish/production_dossier.py job.json` — 0/1/2 exit contract.
Job: `out_dir`*, `parts`* (MADE `{name, stl, material, qty?, material_required?, print_notes?,
print_params?}` | BUY `{name, buy:true, qty?, part_number?, unit_price?, source?}`),
`bed`? `{x:220,y:220,z:250}`, `density_kg_m3`? overrides, `print_params`? `{perimeters:3,
line_width:0.45, layer_h:0.2, top_bottom_layers:4, infill:0.20}`, `filament_price_per_kg`? (25),
`print_speed_factor`?, `spacing_mm`? (5).
Emits `bom_dossier.json` + `.csv`. Printed-mass model (the 4.6 kg lesson): printed_g = shell +
infill×core, ±30% band; time heuristic printed_g/12 h ±50% — planning figures, slicer is truth.
Thick-section warning when solid_g > 2×printed_g. Parts that can't fit the bed refuse the job.

## bom_audit.py — HARDCODED project script, not generic

Audits `{cyclo26,harmonic26,planetary26}/ASSEMBLY.step` against a hardcoded unified hardware BOM by
counting STEP instance names. Run from a directory containing those trees: `python3 tools/publish/bom_audit.py`.
Not reusable for other campaigns without editing the `UNIFIED` table. Exit 0/1 = PASS/FAIL.

---

# 5. DOCUMENTATION / RENDERING (all matplotlib headless; motion_gif also needs PIL)

## render_views.py — quick 4-view PNG (positional args, no job file)

`python3 tools/publish/render_views.py in.stl out.png [iso|joint z0 z1]` — iso = top/iso/front/bottom;
joint = z-clipped band 3-view. Prints `out: N tris` (not JSON). VERIFIED.

## render_sheet.py — the 12-panel vision contact sheet

`python3 tools/publish/render_sheet.py job.json` — 0/1/2 exit contract. Job paths resolve against the CWD; launch from the repo root.
Job: `stl` (one) or `stls` (overlay), `out`*, `dimensions`? (`[{kind:"linear",a,b,label?,view?}|
{kind:"diameter",center,axis,radius,label?}]` — take values from dim_suggest/measure_dimension so
numbers are analytic), `build_dir`? ([0,0,1]; drives the bed-view print orientation), `sections`?
(`{x,y,z}` cut overrides), `date`? (string; NEVER the clock — determinism), `max_px`? (1600).
12 panels: 6 orthos, 2 isos, bed view, 3 true section cuts with parity hatch. Binary STL only.
Receipt: `{ok, out, panels:12, px, triangles, ...}`. VERIFIED on the box (1600×1200 px).

## analysis_sheet.py — how the part PERFORMED (domain-agnostic panels)

`python3 tools/publish/analysis_sheet.py job.json`
Modern form: `{title, panels:[{kind:"view", caption, stl, loads, fixture} | {kind:"field", stl, npy,
origin_mm, voxel_mm, cmap, unit, scale?, vmax?, hotspot?} | {kind:"curve", series, xlabel, ylabel,
targets?} | {kind:"image", png}], results (dict or [[label,value]] — verbatim receipts), gates
{name:bool} chips, date, out}`. Legacy structural form auto-converts (`stl, stress_npy, disp_npy,
origin_mm, voxel_mm, case, loads, fixture, results, gates`). Field panels expect STRUCTURED
(nx,ny,nz) grids (the voxel runners' outputs, not the tet runner's).

## assembly_doc.py — exploded-view assembly sheet + instructions.md

`python3 tools/publish/assembly_doc.py job.json`
Job: `parts`* `[{name, stl, color?}]` in assembly order, `explode`*
(`{axis: [x,y,z], auto:true, gap_mm:8}` | `{axis: [x,y,z], spacing_mm}` |
`{axis: [x,y,z], offsets:{name:[dx,dy,dz]}}`), `steps`* (`[{order, text, fasteners?}]`
or `auto_steps:true` draft), `bom_csv`? (production_dossier's csv — row order = balloon numbers),
`out_prefix`*, `date`?, `rev`?, `project`?, `doc_title`?, `view`?, `max_px`? (1800).

> **`explode.axis` now accepts BOTH forms** (fixed 2026-08-08, ball F6 / singulator F12):
> a 3-vector `[0,0,1]` **or** an axis name `"z"` / `"+z"` / `"-z"` (`parse_axis`,
> `tools/publish/assembly_doc.py`). It used to be vector-only and a letter died with
> `ValueError: could not convert string to float: 'z'` — a message that never named the key.
> An unknown name now refuses by name. Prefer the explicit vector in shipped jobs: it is the
> form every other direction field in the toolchain takes (`motion_gif`'s `spin.axis` /
> `spin.center`, `render_sheet`'s and `support_report`'s `build_dir`).
> `view` accepts **either** `{elev, azim}` **or** `[elev, azim]` (both legal since 2026-08-08;
> it used to accept only the dict and died on a list with a bare `AttributeError`).
Writes `<out_prefix>_assembly_doc.png` + `_instructions.md`. A scene part with no BOM row is a hard
error. Receipt: `{ok, png, md, px, parts, steps, bom_rows}`.

## motion_gif.py — motion study / assembly-sequence GIF (+MP4 via ffmpeg)

`python3 tools/publish/motion_gif.py job.json`
Job: `title, meta, parts:[{stl, name, color?, spin:{axis,center,turns}? | keyframes:[{at:0..1,
translate?, rotate:{axis,center,degrees}?}]?, visible_from?}], sequence?:{axis?, distance?, hold?}
(auto fly-in assembly), frames:48, fps:24, elev, azim, azim_sweep?, size_px:[900,640], ground:true,
date, out (.gif), contact_out?, poster_out?, mp4_out?`. Gear trains encode true ratios in `turns`.
ffmpeg absent → `{"mp4_skipped": reason}`, everything else still succeeds.
Proven pattern: convert a sweep_check receipt's t stations into keyframes → clearance proof becomes
a visible motion.

## dim_suggest.py — analytic dimension callouts from list_faces (stdlib + engine)

`python3 tools/dim_suggest.py job.json`
Job: `{program:{...}|program_file, solid, out (dims fragment json), max:10, receipt?}`.
Suggests Ø callouts (exact 2·radius, grouped per bore) and 2–6-rung axis step ladders; refuses to
guess cones/tori/freeform. Output file holds `{"dimensions":[...]}` for direct merge into a
render_sheet job. VERIFIED: box+bore → `{ok:true, suggested:1, dimensions:[{kind:"diameter",...,
label:"Ø6.00"}]}`.

## document_bundle.py — ONE command → full deliverable bundle

`python3 tools/publish/document_bundle.py job.json` — orchestrates everything above into
`<out_dir>/{print,docs,receipts,programs,README.md,manifest.json}` (sha256-signed manifest).
Job (paths relative to the JOB FILE's directory): `{name, title, out_dir, date* (never the clock),
rev, changelog, bed?, print_params?, parts:[{name, program_file|program, solid, material, qty?,
formats? ["stl","3mf","step"], sheet?:{auto_dimensions?, max_dims?, dimensions?}}],
assembly:{parts:[{name,stl,color?}] (names MUST equal parts[].name), program_file?, explode?,
steps?, motion?, sequence?}, checks:[{name, tool, job}] with tool ∈ tolerance_stack|sweep_check|
balance_check|joint_check|production_check|air_topology_audit, fits:[{label,op,params}],
templates:[{src,dst}] ({{dotted.path:.2f}} injected from the merged receipt tree; unresolvable
keys FAIL the bundle)}`.
Receipt: `{ok, out_dir, artifacts, gates, ...}`. Nothing is claimed a receipt does not state.

---

# Dependency matrix

| needs | tools |
|---|---|
| full ACE (`~/Work/ACE` importable) | ace_fea, ace_fea_tet (+gmsh), ace_modal, ace_buckling, ace_optimize, graded_infill (+kernel-api binary) |
| numpy/scipy only | ace_thermal, air_topology_audit, voxelize_stl (numpy) |
| numpy only | ace_contact, ace_fatigue, stress_to_density, production_dossier |
| pure stdlib (+ engine binary for some) | tolerance_stack, joint_check, production_check, param_optimize, sweep_check, balance_check, dim_suggest, derived_model, document_bundle (orchestrates all) |
| matplotlib (+numpy) | render_views, render_sheet, analysis_sheet, assembly_doc, motion_gif (+PIL) |

# Trust tiers (tools/analyzer_registry.py — run it to see the live ledger)

Validated (manifest+pin): ace_fea, ace_fea_tet, ace_modal, ace_buckling, ace_optimize,
param_optimize, and — since 2026-09-02, pinned to hand-derived arithmetic rather than physics —
tolerance_stack, production_check, production_dossier (quote them as "Validated arithmetic",
their registry `kind` is rules_engine / reporting). Demonstrated: ace_thermal, ace_contact,
graded_infill, air_topology_audit, sweep_check, balance_check, damped_oscillator. Cataloged:
ace_fatigue (deliberately — the gate suite proves the Miner arithmetic, not the life), joint_check.
ace_thermal / ace_contact / ace_fatigue ARE registered (Demonstrated / Demonstrated / Cataloged,
`--tier <name>` returns the reason); their solver cards' "green" is a gate-suite status, not a tier.

# Pitfalls checklist (each one observed or source-documented)

1. Quote the repo path — it contains a space.
2. Engine ops: flat args (`min`/`max` on box), NOT `params:{}`; `export_stl` wants `file`;
   `cylinder` wants `base` + 3-vector `axis`.
3. Plane selector `side` is `"+"`/`"-"` — "above"/"below" is refused.
4. ONE exit contract for every runner: 0 = ok:true, 1 = could not run the request, 2 = ran and
   REFUSED / analysis failed. `ok` and `$?` always agree; branch on `error_kind`. (The ACE runners
   used to exit 0 on failure — they no longer do; `LMCAD_RUNNER_EXIT=legacy` restores it, on the
   record, and campaigns should never set it.)
5. Unknown job keys are silently ignored almost everywhere — misspelled optional params become
   silent defaults. Diff your job against this cookbook's field names.
6. Modal: no fixtures without `free_free:true` = refusal; density ≤ 0 = refusal; simp_penalty ignored.
7. Buckling: report knockdown×critical (default 0.5), never raw critical_load_N; check the
   yield-before-buckling note.
8. Fatigue: only PLA passes the data gate today; across-layer always refused; von Mises fields need
   an explicit `r_ratio` per block.
9. ace_fea voxel stress reads ~20% LOW at coarse voxels; tet path reads concentrations but its
   fields are unstructured (different downstream consumers).
10. graded_infill / stress-field consumers refuse grid-shape mismatches — keep origin/voxel/shape
    identical across the pipeline; voxelize_stl exists to pin the frame.
11. ace_optimize: freeze load/fixture regions or they get optimized away.
12. Default FDM tolerance ±0.15 mm: a 0.2 mm nominal bore-shaft clearance interferes at extremes.
13. PLA sustained-load allowable is 11 MPa (creep fraction 0.2) — most "static-OK" parts fail this gate.
14. Documentation tools take `date` as a string and never read the clock (byte-identical
    re-renders); omitting it is allowed but different tools default differently ("—").
15. Binary STL only for all doc/render tools; ASCII STL is refused.
16. `receipt`/`out_dir` job keys make checker receipts persist; ACE runner receipts are stdout-only
    — capture them.

# Deep-dive pointers

| topic | source |
|---|---|
| solver validation pins + error bands | `tools/manifests/*.manifest.json` (fea, fea_tet, modal, buckling, optimize, param_optimize); `tools/ace_*_validation.py`, `tools/tests/test_ace_modal_buckling.py`, `tools/tests/test_ace_thermal.py`, `tools/tests/test_ace_contact_fatigue.py` |
| solver registry cards (why each exists, limits) | `tools/solvers/{ace_fea,modal,buckling,thermal,contact,fatigue}.md`, `tools/solvers/README.md` |
| trust tiers & graduation rules | `tools/analyzer_registry.py` (runnable), `docs/ANALYSIS_TIERS.md`, `docs/MANIFEST_SCHEMA.md` |
| provenance envelope (`lmcad.analysis.v1`) | `tools/provenance.py`, `tools/analyzers/_ace.py` (provenance_fields) |
| materials records & derating | `tools/analyzers/materials.py`, `tools/materials/*.json`, `tools/material_db.json`, `tools/materials/fatigue.json` |
| ACE integration & selector engine | `docs/ACE_INTEGRATION.md`; ACE source `~/Work/ACE/engine/verify/selectors.py`, `fea.py`, `fea_tet.py` |
| derived on-the-fly models | `tools/analyzers/derived_model.py` docstring + `DampedOscillator` exemplar; `tools/manifests/derived/` |
| engine op vocabulary | `crates/kernel-api/src/discover.rs`, `crates/agent-bench/src/lib.rs` (worked op JSON), `describe` op at runtime (`{"op":"describe","name":"box"}`) |
| doc design system | `tools/publish/render_sheet.py` STYLE dict (assembly_doc/analysis_sheet/motion_gif import it) |
| bundle layout convention | `tools/publish/document_bundle.py` docstring |
| numerics/robustness doctrine | `docs/NUMERICS.md`, `docs/ROBUSTNESS.md`, `campaign/friction/ENGINE.md` |
