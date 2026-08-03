# contact — geometrically-nonlinear beam + rigid-obstacle contact (snap-fits, latches, hinges)

- **Runner**: `tools/ace_contact_runner.py` · **Gates**: `tools/test_ace_contact_fatigue.py` · in-house (NumPy only, no ACE dependency)
- **Why not ace_fea**: `ace_fea` is LINEAR (small strain AND small displacement, no contact). A snap-fit arm deflecting 1-5 mm on a 15 mm arm is 10-30% of span at 10-30 deg tip rotation — linear FEA over-predicts stiffness there because it never shortens the moment arm (gate 2: linear says delta/L = 1.00, truth is 0.603). This is a **different discretization for a different job**: a planar corotational BEAM, <300 DOF, with an exact closed-form benchmark (the elastica). Use `ace_fea` for 3-D stress fields; use this for load PATHS with large rotation and/or contact.
- **Physics**: planar corotational Euler-Bernoulli beam — exact for arbitrarily large RIGID-BODY rotation with small LOCAL strain (the printed-polymer snap-fit regime). Local `N = EA/L0 (Ln-L0)`, `M1,M2 = EI/L0 (4,2 / 2,4)(theta_i - alpha, theta_j - alpha)`, `alpha = beta - beta0`; `f_int = B^T q`, `K_t = B^T k_l B + (N/Ln) zz zz^T + ((M1+M2)/Ln^2)(r zz^T + zz r^T)` (Crisfield). The geometric group is what makes the answer STIFFER than linear.
- **Contact**: node-to-analytic-rigid-surface PENALTY — `plane`, `cylinder` (in/out), `profile` (piecewise-linear rigid terrain, the snap-fit catch), each optionally TRANSLATING with the load parameter (that is what produces an insertion curve). Normal `p_n = kappa max(0,-gap)`; optional regularized Coulomb friction (elastic slip, approximate slipping tangent). Penetration `= p_n/kappa` is reported every step, never hidden.
- **Solution**: Newton-Raphson, `steps.n` increments of load / prescribed displacement / obstacle travel, Crisfield line search on the ENERGY merit `|du.R|` (the residual norm is NOT a usable merit: a healthy corotational predictor multiplies `||R||` ~2300x while penalty active-set chatter multiplies it ~9500x — indistinguishable). Convergence = force ratio `<= tol` OR Bathe energy ratio `<= tol_energy`; `converged_by` is in the receipt. **Non-convergence RAISES** — `{ok:false, error}` + exit 1, never a silent last iterate.

## I/O contract (JSON manifest -> curve .npy + JSON receipt on last stdout line)
```
{out_dir,
 beam: {length_mm, n_elements | nodes_mm:[[x,y],..],
        section:{width_mm, thickness_mm | root_thickness_mm+tip_thickness_mm}},   # mm; taper = the usual snap arm
 material: "PLA" | {youngs_modulus_pa[, yield_mpa, ultimate_mpa]},
 supports: [{node:"root"|"tip"|int|{at_mm}, dofs:{ux?,uy?,rz?}, ramped?}],        # ramped => value*lambda = displacement control
 loads?:   [{node, fx_n?, fy_n?, mz_nmm?}],                                        # scaled by lambda; FIXED direction (no follower loads)
 obstacles?: [{kind:"plane", point_mm, normal | "cylinder", center_mm, radius_mm, side
               | "profile", points_mm (x ascending), side:"above"|"below",
               penalty_n_per_mm, friction?:{mu,k_t_n_per_mm}, motion?:{dir,travel_mm}, nodes?}],
 steps?: {n:20, max_iter:30, tol:1e-8, tol_energy:1e-14, line_search:true, min_alpha:1/1024, ls_eta:0.8},
 linear_reference?: true}
```
Outputs: `curve.npy` — float64 `(n_steps+1, 12)` C-order, columns in receipt `curve_columns`: lambda, obstacle_travel_mm, **insertion_force_n**, total_normal_force_n, tip_ux/uy_mm, max_disp_mm, max_abs_stress_mpa, max_penetration_mm, n_contact_nodes, newton_iters, residual_norm_n. Receipt also carries `nonlinear` and `linear` side by side (so the linear over-prediction is visible), `path_max` (worst stress/strain over the WHOLE path — a latch that springs back ends unstressed), `insertion{peak_force_n, peak_at_travel_mm, min_force_n, final_force_n, max_penetration_mm}`, `nodes_final_mm`, and per-step `convergence.per_step` with iterations, residual history, `converged_by`, reactions and `equilibrium_residual_n`. Failure = `{ok:false, error}` + **exit 1** (thermal-runner convention; the negative controls pin it).

## Benchmark gates (measured 2026-07-30, all green; bands frozen from these numbers)
| gate | closed form | measured | band asserted |
|---|---|---|---|
| 1 linear limit | `delta = PL^3/3EI`, `M_root = PL` | linear ref 2.73e-13 rel, root moment 2.05e-12 rel; `delta_nl/delta_lin` 0.99886 / 0.999989 / 0.9999999 at alpha = 1e-1/1e-2/1e-3 | <= 1e-11 / 1e-10; \|1-ratio\| <= 1e-6 at alpha=1e-3 and shrinking ~100x per load decade (O(alpha^2)) |
| 2 large deflection | exact elastica (elliptic integrals, re-derived + Gauss-quadratured in the gate file) at `alpha = PL^2/EI = 3`: `delta/L = 0.6032534411`, `x/L = 0.7455798154` | 10/20/40/80 el: 8.622e-4, 2.149e-4, 5.436e-5, **1.431e-5**; ratios 4.012/3.953/3.798 | <= 2e-5 (x: 6e-6); ratios in [3.4,4.6] |
| 2 PHYSICS SIGN | linear `delta/L = alpha/3 = 1.000` vs elastica 0.6033 | 0.60326 (ratio 0.6033) | < 0.65 x linear — a linear-kinematics bug lands at 1.0 |
| 2 extensibility | the elastica is INEXTENSIBLE; this beam is not, by `O((t/L)^2)` | err at 80 el: t=1.0 3.764e-5, t=0.5 1.942e-5, t=0.2 1.431e-5 -> excess ratio 4.57 (t^2 predicts 4) | ratio in [3.0,6.0] — the fine-mesh residual is PHYSICS, measured, not hidden |
| 3a plane contact | penetration `= p_n/kappa`; force balance is an identity | pen 1.755e-4/1.755e-5/1.755e-6 mm at kappa 1e4/1e5/1e6; `pen*kappa` 1.755443 -> 1.755485 N (10x convergence per decade); no node deeper than the tolerance; equilibrium 8.6e-12 rel | spread <= 1e-4 + converging; equilibrium <= 1e-10 |
| 3b roller statics | simply-supported beam, `R_roller = P a/L = 0.6666667 N` | 0.6666124 N, 8.14e-5 rel (second-order moment-arm shortening) | <= 1e-3 |
| 4 insertion curve | snap-fit guides: `F = P (mu + tan a)/(1 - mu tan a)`, `P = 3EI y/L^3`; frictionless `F = P tan a` | peak **3.0069 N** at 2.175 mm travel vs 3.2380 N naive (-7.14%), vs 3.0663 N beam-column-corrected (-1.94%); retention -8.3335 N vs `P tan 60 = 8.1450` (+2.31%); force >= 0 through engagement, exactly 0 after release; path-max root strain 1.205% | peak +-12% of naive AND below it AND +-6% of corrected; retention +-10%; \|force\| <= 1e-9 after release |
| 4 PHYSICS | 3 N of contact force compresses the arm at 4.9% of its Euler load 61.07 N -> beam-column softening | peak lands BELOW the small-deflection closed form | must be below |
| 6 negative controls | — | absurd single-step engagement (2-iteration budget) and shallow-arch SNAP-THROUGH past the limit point both exit 1 with "did not converge"; no supports / zero penalty refuse | exit != 0 |
| 6 stated, not faked | — | an absurd PENALTY alone (1e14 N/mm = 2e13x the arm's tip stiffness, whole engagement in ONE step) does **not** break it: converges, penetration 6.1e-14 mm | gated as the positive result it is |
| 7 meta-control | break one constant in a scratch copy | corotational rotation removed -> gates 1-4 all raise; Pa->MPa 1e-6 -> 1e-5 -> 8 value-level failures incl. the physics sign | suite must go RED |

## Validity limits / out of scope
- **PLANAR beam only.** No 3-D, no torsion, no out-of-plane buckling of the arm, no width effects (a wide latch is modelled as a beam of that `width_mm`, i.e. plane-stress-ish, not a plate).
- **Euler-Bernoulli**: no transverse shear. Below L/t ~ 10 the arm reads a few % stiff; short stubby latches need the caveat stated.
- **Contact is NODE-based**: the contact patch resolves only to the node spacing, and the reported penetration IS the penalty compliance `p_n/kappa` — quote it. Sharp (zero-radius) obstacle corners make the normal jump and can stall Newton; fillet them (a printed catch has a radius anyway) or the runner will refuse.
- **Friction** is a regularized elastic-slip Coulomb model with an APPROXIMATE slipping tangent and a monotone tangential parameter (not exact arc length on curved obstacles). Default is frictionless; every gate here is frictionless, so friction is **untested capability** — treat `mu > 0` results as indicative and say so.
- **Quasi-static**: no dynamics, so the snap-back "click" (the energy released when the latch clears the catch) is not resolved; the curve jumps to zero instead.
- **Elastic, isotropic**: no yielding, no creep, no printed-layer anisotropy. Check the reported `path_max.strain` against the material's allowable strain and `tools/materials/pla.json` `creep` for held deflections.
- **Load control only**: no arc-length continuation, so a genuine limit point (snap-through) is a REFUSAL, not a traced path — see gate 6.

## When to use
Any printed feature whose whole point is large elastic deflection against something rigid: snap-fit / cantilever latches (insertion force, retention force, peak strain), living hinges, press-fit lips and spring clips, a spool pawl riding a ratchet. The receipt's `insertion_force_n` curve is the number a designer actually specifies ("assembles under 15 N by hand, retains above 40 N"), and `path_max.strain` is what decides whether the arm survives the first click. Pair with `tools/solvers/fatigue.md` for repeated actuation, and with `ace_fea` when you need the 3-D stress field at the root fillet.

Run: `python3 tools/ace_contact_runner.py job.json` · prove: `python3 tools/test_ace_contact_fatigue.py`
