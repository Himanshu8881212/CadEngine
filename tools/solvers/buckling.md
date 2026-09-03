# buckling — hex8 linear (eigenvalue) buckling, critical load factor

- **Runner**: `tools/analyzers/ace_buckling_runner.py` (bridge to ACE `engine.verify.reference_buckling`, plus a prestress receipt pass through `engine.verify.reference_fea` — the exact function behind ace_fea) · **Gates**: `tools/tests/test_ace_modal_buckling.py` · legacy pin `tools/validation/ace_buckling_validation.py` · manifest `tools/manifests/ace_buckling.manifest.json`
- **Physics**: classical linearised pre-buckling (bifurcation) analysis. Two passes: (1) static `K u = F` under the manifest's reference load; (2) geometric stiffness from the recovered 2x2x2 Gauss-point Cauchy stresses, `K_g = integral G^T diag(S,S,S) G dV`, then `K phi = -lambda K_g phi` for the smallest POSITIVE factor; `critical_load = lambda x applied reference load`.
- **THE CAVEAT (first field of every receipt)**: this is a bifurcation estimate on the PERFECT geometry — no imperfections, no plasticity, no large-displacement path. Real structures buckle EARLIER; lambda is an UPPER bound and a DESIGN-LOOP number.
- **Knockdown (recommended, cited)**: receipts carry `knockdown.recommended_factor = 0.5` -> `design_critical_load_n = 0.5 x critical_load_N`. Basis: AISC 360 §E3 keeps only 0.877 x the elastic critical stress for straight steel columns (crookedness ~L/1000 + residual stress); EN 1993-1-1 §6.3.1.2 buckling curves knock intermediate slenderness down by imperfection factors alpha = 0.13-0.76; NASA SP-8007-2020/REV 2 gives 0.32-0.65 for thin-walled cylinders. FDM parts are more imperfect than any of those calibration sets, hence flat 0.5 — and for SHELL-LIKE modes (inspect the mode) even 0.5 can be unconservative: use SP-8007-class knockdowns there.
- **Discretization**: trilinear hex8, binary occupancy; static pass direct sparse; eigensolve via Cholesky-reduced dense (<= 1500 free DOF) or generalised `eigsh(-K_g, M=K, which='LA')`, positive factors only, honest refusals otherwise.

## I/O contract (JSON job -> .npy prestress fields + JSON receipt on last stdout line)
```
{out_dir, voxel_mm, origin_mm?,
 ops+solid+shape[+supersample] | npy | stl+shape,    # STL parity-filled via tools/analyzers/voxelize_stl.py
 material: "PLA" | {youngs_modulus_pa, poisson[, density_kg_m3]},   # string key also enables the yield-before-buckling note
 fixtures: [{kind, region_selector, dof_constrained?}],             # REQUIRED
 loads: [{kind: point|body|pressure, magnitude, direction?, region_selector}],   # the REFERENCE load; zero/absent load = refusal
 n_modes?: 4, knockdown?: 0.5, direct_solver_max_dof?: 0, regions?}
```
Outputs: `prestress_disp_field.npy` / `prestress_stress_field.npy` (ace_fea layout). Receipt: `caveat`, `load_factors`, `buckling_load_factor`, `applied_reference_load_N`, `critical_load_N`, `knockdown{...sources}`, `prestress{max_von_mises_pa, tip_displacement_m, ...}`, a loud note when von Mises at the critical load exceeds the registry yield (buckling then isn't the governing failure), selector receipts (loads > 30% of active elements flagged), provenance envelope. Failure — including the solver's honest refusals (no load, no compressive stress anywhere, no positive eigenvalue) = `{ok:false, error}` + **exit 0**.

## Benchmark gates (measured 2026-07-30, all green; bands frozen from these numbers)
| gate | closed form / expectation | measured | band asserted |
|---|---|---|---|
| Euler column (45x4.5x3 mm, fixed-free, weak axis, L_eff/r ~ 100) | `P_cr = pi^2 E I / (4 L_eff^2)`, `L_eff = L - voxel` (plane clamp fixes the first element layer) | vox 0.75: +6.31% · vox 0.5: +3.36% (dev 3-pt: +2.23% at 0.375) | (4.5%, 8.5%) / (1.8%, 5.2%), over-prediction only; must converge |
| convergence order | between O(h) stairstep and O(h^2) element limits | p = 1.56 (dev 3-point: 1.56 / 1.42) | p in [0.8, 2.5] |
| linearity: 2E -> 2 lambda | sigma = D B K^-1 F is E-independent, so K_g fixed while K doubles | ratio 2.000000000 | <= 1e-6 rel |
| linearity: 2F -> lambda/2 | K_g doubles while K fixed; `lambda x P` invariant | ratio 0.500000000; lambda*P equal | <= 1e-6 rel |
| cross-solver (static pass vs ace_fea) | both runners call the same `reference_fea` with the same Jacobi-CG settings on the same manifest | disp fields + tip + max vM: max rel diff 0.00e+00 | <= 1e-8 rel |
| negative control | zero / absent reference load | refuses in-process AND through the subprocess JSON contract | must refuse |

## Validity limits / out of scope
- UPPER bound by construction; apply the knockdown before design use. Never quote raw lambda as a margin.
- Slender ELASTIC members only (gate pinned at L_eff/r ~ 100). Stocky columns fail by yield first — the runner's yield note fires when it can (registry-key material), but the strength check (ace_fea + closed form) is the caller's job.
- Moment loads are skipped (C0 hex8 has no rotational DOFs — noted in the receipt); pure-tension states refuse ("no compressive stress").
- A point load's local contact compression can seed a compressive region even in globally tensile states — read the receipts, don't fish for a factor.
- Isotropic, perfect geometry: printed seams, infill and layer anisotropy are exactly the imperfections the knockdown exists for.

## When to use
Compression members in campaigns: spool-holder posts, drybox roller axles under belt/band tension, thin lattice struts, long thin walls under clamp loads. Use `design_critical_load_n` (knocked down) in gates; pair every buckling gate with a strength gate; inspect `prestress_stress_field.npy` to see WHERE the compression lives.

Run: `ACE_PYTHON tools/analyzers/ace_buckling_runner.py job.json` · prove: `python3 tools/tests/test_ace_modal_buckling.py`
