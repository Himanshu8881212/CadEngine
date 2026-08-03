#!/usr/bin/env python3
"""Physics validation pin: ACE's hex8 FEA vs the closed-form cantilever solution.

A correct FEA implementation must CONVERGE toward the analytic answer under mesh
refinement; a wrong one won't. This pins that property (and the error band we
quote in the ace_fea tool description) against Euler-Bernoulli + shear:

	beam L=40, b=8, h=8 mm · clamped at x=0 · 10 N tip load
	delta = PL^3/(3EI) + 1.2·PL/(GA) = 0.2934 mm

Measured 2026-07-08 (pinned here): voxel 1.0 → −11.2%, voxel 0.5 → −5.9%,
both UNDER-predicting (hex8 is stiff — the documented, conservative direction).

Run:  ACE_PYTHON (default ~/miniconda3/bin/python3) this file.
Exit: 0 iff all assertions hold; nonzero with a message otherwise.
"""
import os, sys
import numpy as np

sys.path.insert(0, os.environ.get("ACE_ROOT", os.path.expanduser("~/Work/ACE")))
from engine.verify import reference_fea  # noqa: E402

L, b, h = 40.0, 8.0, 8.0
P, E, NU = 10.0, 2.2e9, 0.37

I = b * h**3 / 12.0
G = E / (2 * (1 + NU))
d_analytic = P * L**3 / (3 * (E * 1e-6) * I) + 1.2 * P * L / ((G * 1e-6) * b * h)


def solve(vox: float) -> float:
	nx, ny, nz = int(L / vox), int(b / vox), int(h / vox)
	res = reference_fea(
		np.ones((nx, ny, nz), dtype=np.float32),
		np.full((nx, ny, nz), "design", dtype=object),
		vox,
		{"youngs_modulus_pa": E, "poisson": NU, "density_kg_m3": 1270.0},
		loads=[{"kind": "point", "magnitude": P, "direction": [0, 0, -1],
			"region_selector": {"type": "plane", "axis": "x", "value_mm": L, "side": "+"}}],
		fixtures=[{"kind": "clamped", "region_selector": {"type": "plane", "axis": "x", "value_mm": 0.0, "side": "-"}}],
		direct_solver_max_dof=0,
	)
	return res["max_displacement_m"] * 1000.0


coarse, fine = solve(1.0), solve(0.5)
e_coarse = (coarse - d_analytic) / d_analytic
e_fine = (fine - d_analytic) / d_analytic
print(f"analytic {d_analytic:.4f} mm · coarse {coarse:.4f} ({e_coarse:+.1%}) · fine {fine:.4f} ({e_fine:+.1%})")

failures = []
if not (-0.20 < e_coarse < 0.0):
	failures.append(f"coarse error {e_coarse:+.1%} outside the pinned (-20%, 0) band")
if not (-0.10 < e_fine < 0.0):
	failures.append(f"fine error {e_fine:+.1%} outside the pinned (-10%, 0) band")
if not abs(e_fine) < abs(e_coarse):
	failures.append("no convergence: refinement did not move toward the analytic solution")
if failures:
	print("VALIDATION FAIL:", "; ".join(failures))
	sys.exit(1)
print("VALIDATION PASS: converges toward closed-form physics; error band as documented (hex8 under-predicts).")
