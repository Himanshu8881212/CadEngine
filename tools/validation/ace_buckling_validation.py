#!/usr/bin/env python3
"""Physics validation pin: ACE's hex8 buckling solver vs the Euler column.

A correct linear-buckling implementation must CONVERGE toward the analytic
critical load under mesh refinement; a wrong one won't. This pins that property
(and the error band quoted in the ace_buckling tool description) against the
clamped-free (fixed-free) Euler column:

	strut L=60, b=6, h=6 mm · clamped at x=0 · 100 N axial compression at x=L
	Pcr = pi^2·E·I / (4·L^2) = 162.85 N  (E=2.2e9 Pa, I=b·h^3/12)

Slenderness check: r = h/sqrt(12) = 1.73 mm, effective Le/r = 2L/r ≈ 69 — a
genuinely slender elastic column, and the solver handled it cleanly, so no
stockier fallback case was needed. Measured 2026-07-09 (pinned here):
voxel 1.0 → +7.3%, voxel 0.5 → +3.0%, both OVER-predicting (linear eigenvalue
buckling on a stiff hex8 mesh is an upper bound on the elastic critical load;
ACE's own docstring says 10-30% high on coarse meshes — this geometry measures
better than that). The first factor is a degenerate PAIR (square section — two
identical bending planes); the pin uses the smallest.

Run:  python3 this file (numpy + scipy only; the solver is in-tree).
Exit: 0 iff all assertions hold; nonzero with a message otherwise.
"""
import math
import os, sys
import numpy as np

sys.path.insert(0, os.path.join(  # tools/analyzers: the in-tree solver package
	os.path.dirname(os.path.abspath(__file__)), os.pardir, "analyzers"))
from physics import reference_buckling  # noqa: E402

L, b, h = 60.0, 6.0, 6.0
E, NU, P = 2.2e9, 0.37, 100.0

I = (b * 1e-3) * (h * 1e-3) ** 3 / 12.0
P_analytic = math.pi**2 * E * I / (4 * (L * 1e-3) ** 2)


def solve(vox: float) -> float:
	nx, ny, nz = int(L / vox), int(b / vox), int(h / vox)
	res = reference_buckling(
		np.ones((nx, ny, nz), dtype=np.float32),
		None,
		vox,
		{"youngs_modulus_pa": E, "poisson": NU, "density_kg_m3": 1270.0},
		[{"kind": "point", "magnitude": P, "direction": [-1, 0, 0],
			"region_selector": {"type": "plane", "axis": "x", "value_mm": L, "side": "+"}}],
		[{"kind": "clamped", "region_selector": {"type": "plane", "axis": "x", "value_mm": 0.0, "side": "-"}}],
		n_modes=2,
	)
	return res["critical_load_n"]


coarse, fine = solve(1.0), solve(0.5)
e_coarse = (coarse - P_analytic) / P_analytic
e_fine = (fine - P_analytic) / P_analytic
print(f"analytic {P_analytic:.2f} N · coarse {coarse:.2f} ({e_coarse:+.1%}) · fine {fine:.2f} ({e_fine:+.1%})")

failures = []
if not (0.0 < e_coarse < 0.15):
	failures.append(f"coarse error {e_coarse:+.1%} outside the pinned (0, +15%) band")
if not (0.0 < e_fine < 0.08):
	failures.append(f"fine error {e_fine:+.1%} outside the pinned (0, +8%) band")
if not abs(e_fine) < abs(e_coarse):
	failures.append("no convergence: refinement did not move toward the analytic solution")
if failures:
	print("VALIDATION FAIL:", "; ".join(failures))
	sys.exit(1)
print("VALIDATION PASS: converges toward closed-form physics; error band as documented (linear buckling over-predicts).")
