#!/usr/bin/env python3
"""Physics validation pin: ACE's hex8 modal solver vs the closed-form cantilever.

A correct modal implementation must CONVERGE toward the analytic answer under
mesh refinement; a wrong one won't. This pins that property (and the error band
quoted in the ace_modal tool description) against the Euler-Bernoulli
clamped-free first bending frequency:

	beam L=40, b=8, h=8 mm · clamped at x=0 · E=2.2e9 Pa · rho=1270 kg/m^3
	f1 = (1.875104^2 / 2pi) · sqrt(E·I / (rho·A)) / L^2 = 1063.06 Hz

Measured 2026-07-09 (pinned here): voxel 1.0 → +4.0%, voxel 0.5 → +0.9%,
both OVER-predicting (hex8 + the one-voxel clamp layer are stiff — frequencies
come out slightly high and refine DOWNWARD toward the analytic value). Note the
analytic pin is pure Euler-Bernoulli: at L/h = 5 shear flexibility (Timoshenko)
would LOWER the exact answer a few percent, so the true discretization
stiffness is a little larger than the raw deltas below — the pinned band is on
the EB reference because that is the formula the tool description quotes.

Run:  ACE_PYTHON (default ~/miniconda3/bin/python3) this file.
Exit: 0 iff all assertions hold; nonzero with a message otherwise.
"""
import math
import os, sys
import numpy as np

sys.path.insert(0, os.environ.get("ACE_ROOT", os.path.expanduser("~/Work/ACE")))
from engine.verify import reference_modal  # noqa: E402

L, b, h = 40.0, 8.0, 8.0
E, RHO, NU = 2.2e9, 1270.0, 0.37

I = (b * 1e-3) * (h * 1e-3) ** 3 / 12.0
A = (b * 1e-3) * (h * 1e-3)
f_analytic = (1.875104**2 / (2 * math.pi)) * math.sqrt(E * I / (RHO * A)) / (L * 1e-3) ** 2


def solve(vox: float) -> float:
	nx, ny, nz = int(L / vox), int(b / vox), int(h / vox)
	res = reference_modal(
		np.ones((nx, ny, nz), dtype=np.float32),
		None,
		vox,
		{"youngs_modulus_pa": E, "poisson": NU, "density_kg_m3": RHO},
		[{"kind": "clamped", "region_selector": {"type": "plane", "axis": "x", "value_mm": 0.0, "side": "-"}}],
		n_modes=2,
	)
	return res["first_mode_hz"]


coarse, fine = solve(1.0), solve(0.5)
e_coarse = (coarse - f_analytic) / f_analytic
e_fine = (fine - f_analytic) / f_analytic
print(f"analytic {f_analytic:.2f} Hz · coarse {coarse:.2f} ({e_coarse:+.1%}) · fine {fine:.2f} ({e_fine:+.1%})")

failures = []
if not (0.0 < e_coarse < 0.10):
	failures.append(f"coarse error {e_coarse:+.1%} outside the pinned (0, +10%) band")
if not (0.0 < e_fine < 0.05):
	failures.append(f"fine error {e_fine:+.1%} outside the pinned (0, +5%) band")
if not abs(e_fine) < abs(e_coarse):
	failures.append("no convergence: refinement did not move toward the analytic solution")
if failures:
	print("VALIDATION FAIL:", "; ".join(failures))
	sys.exit(1)
print("VALIDATION PASS: converges toward closed-form physics; error band as documented (hex8 over-predicts frequency).")
