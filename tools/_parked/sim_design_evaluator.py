#!/usr/bin/env python3
"""sim_design_evaluator.py — one physics evaluation for the simulation-driven
design loop (a `command` evaluator for tools/param_optimize.py).

The closed loop is: param_optimize proposes design PARAMETERS -> this script
builds the candidate part, meshes it BODY-FITTED, runs the trustworthy tet10
FEA (engine.verify.fea_tet, validated convergent at stress concentrations) ->
returns a flat receipt -> param_optimize scores the objective/constraints and
proposes the next candidate, until the physics objective converges.

This is the piece that makes the loop's output MEAN something: the objective is
computed from a simulation that actually converges to the true fillet stress,
not the +/-20-30% non-convergent voxel proxy. A min-mass-subject-to-stress
optimum found against a lying evaluator is the wrong part with false
confidence; found against this one, it is the real physics optimum.

Candidate part (the acceptance specimen): a stepped round shaft (shouldered
bar) — the classic stress-concentration part. Human-fixed conditions come in
the job; the design variables are substituted by param_optimize ($d, $r).

Usage:  <ACE_PYTHON> sim_design_evaluator.py <job.json>
Job JSON (all lengths mm, physics SI; $-substituted fields already resolved):
    d, r              design vars: small dia + shoulder fillet radius (mm)
    D                 large dia (mm, fixed)
    l_small, l_large  segment lengths (mm, fixed)
    load_n            axial resultant on the small end (N)
    material          {youngs_modulus_pa, poisson, density_g_cm3}
    elem_size_mm      optional; default clamp(r/2, 0.9, 1.5) — moderate in-loop
                      resolution (accurate enough for the trend + a ~few-%%
                      conservative fillet peak; the converged design is
                      re-checked fine in the validation pin)

Receipt (LAST stdout line, one JSON object; all logging to stderr):
    {ok, mass_g, fillet_peak_pa, far_field_pa, kt, d_mm, r_mm, n_tets, ...}
    ok:false + error on any failure (param_optimize treats it as an infeasible
    eval, per its command-evaluator contract).
"""

import json
import math
import os
import sys

ACE_ROOT = os.environ.get("ACE_ROOT", os.path.expanduser("~/Work/ACE"))
sys.path.insert(0, ACE_ROOT)


def log(*a):
    print(*a, file=sys.stderr, flush=True)


def emit(obj):
    print(json.dumps(obj), flush=True)


def main():
    try:
        job = json.load(open(sys.argv[1]))
    except Exception as e:  # noqa: BLE001
        emit({"ok": False, "error": f"bad job json: {e}"})
        return 0
    try:
        import numpy as np
        from engine.verify.mesh_ir import mesh_shouldered_bar
        from engine.verify.fea_tet import reference_fea_tet

        d = float(job["d"])
        r = float(job["r"])
        D = float(job["D"])
        ls = float(job["l_small"])
        ll = float(job["l_large"])
        P = float(job["load_n"])
        mat = job["material"]
        rho = float(mat["density_g_cm3"])
        # moderate in-loop mesh; fine re-check is the validation pin's job
        elem = float(job.get("elem_size_mm", min(1.5, max(0.9, r / 2.0))))

        # geometric feasibility the optimizer must respect (fillet must fit the
        # step, small dia must be below large dia): refuse loudly if violated so
        # param_optimize scores it infeasible instead of meshing garbage.
        if not (d + 1e-6 < D):
            emit({"ok": False, "error": f"d({d}) must be < D({D})"})
            return 0
        step = (D - d) / 2.0
        if r > step - 1e-6:
            emit({"ok": False, "error": f"fillet r({r}) exceeds the step ({step:.3f})"})
            return 0

        m = mesh_shouldered_bar(d, D, r, ls, ll, elem_size_mm=elem)
        rec = m.check(volume_ref_mm3=None)  # asserts positive Jacobians
        res = reference_fea_tet(
            m, {"youngs_modulus_pa": float(mat["youngs_modulus_pa"]),
                "poisson": float(mat["poisson"]),
                "density_kg_m3": rho * 1000.0},
            loads=[{"kind": "point", "magnitude": P, "direction": [0, 0, -1],
                    "region_selector": {"type": "plane", "axis": "z",
                                        "value_mm": 0.0, "side": "-"}}],
            fixtures=[{"kind": "clamped",
                       "region_selector": {"type": "plane", "axis": "z",
                                           "value_mm": ls + ll, "side": "+"}}],
            direct_max_dof=0, cg_tol=1e-9, cg_maxiter=60000)
        if not res.get("ok"):
            emit({"ok": False, "error": f"fea: {res.get('error')}"})
            return 0

        vm = res["vm_nodal"]
        z = m.nodes_mm[:, 2]
        # governing stress = the fillet peak (a real convergent concentration),
        # measured in a z-band around the shoulder — NOT the global max (which
        # is the loaded-end point-load singularity, an artifact).
        band = np.abs(z - ls) < 1.5 * r
        fillet_peak = float(vm[band].max())
        ff_mask = (z > 3.0) & (z < ls - 3.0)
        far_field = float(np.median(vm[ff_mask])) if ff_mask.any() else float("nan")
        sig_nom = P / (math.pi * (d ** 2) / 4.0)  # MPa (N / mm^2)
        kt = fillet_peak / (sig_nom * 1e6) if sig_nom > 0 else float("nan")

        mass_g = m.volume_mm3() / 1000.0 * rho  # mm^3 -> cm^3 -> g

        # Mesh-convergence design margin: the in-loop mesh is moderate for
        # speed and reads a still-converging fillet peak a few %% LOW, so a
        # design optimized to the cap at this fidelity sits over the cap at
        # truth. `stress_margin` (a documented knockdown, calibrated from the
        # measured fine-vs-in-loop residual ~7%%) inflates the peak the loop
        # constrains on, so the TRUE peak lands within the cap. This is the
        # honest fix the earlier trust-delta finding pointed to — a standard
        # FEA mesh-convergence factor, stated not hidden.
        stress_margin = float(job.get("stress_margin", 0.0))
        fillet_peak_design = fillet_peak * (1.0 + stress_margin)

        emit({"ok": True,
              "mass_g": mass_g,
              "fillet_peak_pa": fillet_peak,
              "fillet_peak_design_pa": fillet_peak_design,
              "stress_margin": stress_margin,
              "far_field_pa": far_field,
              "sig_nom_pa": sig_nom * 1e6,
              "kt": kt,
              "d_mm": d, "r_mm": r,
              "volume_mm3": m.volume_mm3(),
              "n_tets": int(m.n_tets),
              "elem_size_mm": elem})
        return 0
    except Exception as e:  # noqa: BLE001
        import traceback
        log(traceback.format_exc())
        emit({"ok": False, "error": f"{type(e).__name__}: {e}"})
        return 0


if __name__ == "__main__":
    sys.exit(main())
