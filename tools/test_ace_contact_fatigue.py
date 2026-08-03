#!/usr/bin/env python3
"""test_ace_contact_fatigue.py — benchmark gate suite for solvers #5 and #6.

  runner A: tools/ace_contact_runner.py   (nonlinear beam + rigid contact)
  runner B: tools/ace_fatigue_runner.py   (Basquin / mean-stress / Miner)

Both solvers are GUILTY until these gates are green (DESIGN_GUIDE §25.7; the
precedent is tools/test_ace_thermal.py). Every gate re-derives its analytic
truth in a comment, drives the REAL CLI contract (subprocess, JSON receipt on
the last stdout line), and asserts against bands frozen from MEASURED behaviour
(2026-07-30, this machine — measured values quoted per gate, headroom stated).

Run:  python3 tools/test_ace_contact_fatigue.py            -> exit 0 iff green
      python3 tools/test_ace_contact_fatigue.py --gates 1,2 -> subset (used by
                                                               the meta control)

Gates:
  1  LINEAR LIMIT — the nonlinear solver must reproduce PL^3/3EI as the load
     goes to zero (ratio -> 1 monotonically); the linear reference is asserted
     EXACT (a cubic-Hermite beam element is exact for a tip-loaded cantilever)
     and the root moment is asserted = P*L.
  2  LARGE DEFLECTION — cantilever under a fixed-direction tip load at
     alpha = PL^2/EI = 3 against the EXACT elastica (elliptic-integral solution
     re-derived and quadratured in this file); mesh convergence measured; and
     the PHYSICS SIGN asserted: the nonlinear answer must be STIFFER than the
     linear one.
  3  CONTACT — (a) cantilever pressed onto a rigid plane: penetration is the
     penalty compliance and must scale as 1/kappa, no node passes the surface,
     and the global force balance is pinned at machine precision; (b) a rigid
     roller under a simply-supported beam must carry exactly P*a/L (statics).
  4  INSERTION CURVE — a latch arm riding a 30-deg rigid ramp: force
     non-negative through engagement, ~0 after release, and the peak checked
     against the textbook snap-fit closed form F = P tan(alpha).
  5  FATIGUE ARITHMETIC — Miner on a hand-computed 2-block spectrum (exact
     dyadic rationals, machine-precision gate), Basquin round-trip on synthetic
     data, Goodman and Gerber against hand calculations, and the PLA registry
     end-to-end.
  6  NEGATIVE CONTROLS — non-converging contact must exit 1 loudly; a material
     with no credible printed S-N data must REFUSE by name; across-layer
     fatigue must REFUSE; a zero-amplitude spectrum must return an explicit
     no-damage status; a mean-stress correction stacked on a max-stress curve
     must refuse (double counting).
  7  META-NEGATIVE CONTROL — deliberately break one constant in a scratch copy
     of each runner and prove this suite turns RED. A suite that cannot fail is
     not a suite.
"""
from __future__ import annotations

import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

TOOLS = Path(__file__).resolve().parent
CONTACT = Path(os.environ.get("ACE_CONTACT_RUNNER", TOOLS / "ace_contact_runner.py"))
FATIGUE = Path(os.environ.get("ACE_FATIGUE_RUNNER", TOOLS / "ace_fatigue_runner.py"))

E_PLA_PA = 3.3e9          # tools/materials/pla.json mechanical.youngs_modulus_pa
E_PLA_MPA = E_PLA_PA * 1e-6

results: list[tuple[str, bool, str]] = []


def gate(label: str, ok: bool, detail: str) -> None:
	results.append((label, ok, detail))
	print(f"  {'PASS' if ok else '<<< FAIL'}  {label}: {detail}")


def run_job(runner: Path, job: dict, tmp: Path, name: str, expect_ok: bool = True):
	"""Drive the real CLI. Returns (receipt, returncode)."""
	job = dict(job)
	job.setdefault("out_dir", str(tmp / name))
	path = tmp / f"{name}.json"
	path.write_text(json.dumps(job), encoding="utf-8")
	proc = subprocess.run([sys.executable, str(runner), str(path)],
	                      capture_output=True, text=True)
	lines = [l for l in proc.stdout.strip().splitlines() if l.strip()]
	rec = json.loads(lines[-1]) if lines else {"ok": False, "error": "no stdout receipt"}
	if expect_ok and not rec.get("ok"):
		raise AssertionError(f"{name}: runner failed unexpectedly: {rec.get('error')}\n"
		                     f"{proc.stderr[-2000:]}")
	return rec, proc.returncode


# ---------------------------------------------------------------------------
# Exact elastica for a cantilever with a fixed-direction transverse tip load.
#
# Derivation (Bisshopp & Drucker 1945). Inextensible planar rod, arc length s,
# tangent angle theta(s), clamped at s=0, tip load P perpendicular to the
# undeformed axis at s=L. Bending moment at s is the load times the remaining
# horizontal arm, so
#       EI theta'  = -P (x_L - x(s))     ->    EI theta'' = P cos theta.
# One integration with theta'(L) = 0 (zero moment at the free end) gives
#       (EI/2) theta'^2 = P (sin theta - sin theta_L).
# Writing phi = -theta (positive downward) and alpha = P L^2 / EI:
#       sqrt(2 alpha) = I(phi_L),   I = integral_0^{phi_L} dphi/sqrt(sin phi_L - sin phi)
#       delta/L       = J/I,        J = integral_0^{phi_L} sin phi dphi/sqrt(...)
#       x_tip/L       = 2 sqrt(sin phi_L)/I     (the phi-integral of cos is exact)
# The integrands have an inverse-square-root endpoint singularity; substituting
# sin phi = sin(phi_L) (1 - v^2) removes it exactly and leaves an analytic
# integrand on v in [0,1], which fixed Gauss-Legendre nails to ~1e-15.
# PARAMETERISING BY phi_L means alpha = I^2/2 comes out directly — no root
# find, no table, no interpolation error.
# ---------------------------------------------------------------------------
_GL_X, _GL_W = np.polynomial.legendre.leggauss(400)
_GL_V = 0.5 * (_GL_X + 1.0)          # map [-1,1] -> [0,1]
_GL_HW = 0.5 * _GL_W


def elastica(phi_l: float):
	"""-> (alpha = PL^2/EI, delta_v/L, x_tip/L) for tip angle phi_l (rad)."""
	sp = math.sin(phi_l)
	den = np.sqrt(1.0 - sp ** 2 * (1.0 - _GL_V ** 2) ** 2)
	I = math.sqrt(sp) * float(np.sum(_GL_HW * 2.0 / den))
	J = sp ** 1.5 * float(np.sum(_GL_HW * 2.0 * (1.0 - _GL_V ** 2) / den))
	return I * I / 2.0, J / I, 2.0 * math.sqrt(sp) / I


def elastica_at_alpha(alpha: float):
	"""Invert alpha(phi_l) by bisection (alpha is strictly increasing in phi_l)."""
	lo, hi = 1e-9, 0.5 * math.pi - 1e-7
	for _ in range(200):
		mid = 0.5 * (lo + hi)
		if elastica(mid)[0] < alpha:
			lo = mid
		else:
			hi = mid
	phi = 0.5 * (lo + hi)
	a, dv, xt = elastica(phi)
	return phi, a, dv, xt


def slender_beam(length_mm: float, n_el: int, t_mm: float = 1.0, w_mm: float = 1.0):
	return {"length_mm": length_mm, "n_elements": n_el,
	        "section": {"width_mm": w_mm, "thickness_mm": t_mm}}


def EI_of(w_mm: float, t_mm: float) -> float:
	return E_PLA_MPA * w_mm * t_mm ** 3 / 12.0


CLAMP = [{"node": "root", "dofs": {"ux": 0.0, "uy": 0.0, "rz": 0.0}}]


# ---------------------------------------------------------------------------
# Gate 1 — LINEAR LIMIT.
# Closed form: a cantilever, length L, tip transverse load P:
#     delta_lin = P L^3 / (3 EI),   M_root = P L,   sigma_root = M c / I.
# A 2-node cubic-Hermite (Euler-Bernoulli) element reproduces the cubic
# deflection of a tip-loaded cantilever EXACTLY, so the LINEAR reference must
# match to round-off regardless of mesh — a strong pin on the element algebra.
# The nonlinear solve is the same equations plus geometric terms of order
# alpha^2, so delta_nl/delta_lin -> 1 as alpha = PL^2/EI -> 0.
# Measured 2026-07-30 (L=100, w=t=1 mm, E=3.3 GPa, 20 elements):
#     alpha 1e-1 -> nl/lin 0.9988604749 (|1-r| 1.140e-3)
#     alpha 1e-2 -> nl/lin 0.9999885807 (|1-r| 1.142e-5)
#     alpha 1e-3 -> nl/lin 0.9999998858 (|1-r| 1.142e-7)
# i.e. |1 - ratio| falls exactly 100x per decade of load = O(alpha^2), the
# leading elastica correction (the exact elastica gives delta/L = alpha/3 *
# (1 - alpha^2/... )). Linear-reference error 2.725e-13 relative and root-moment
# error <= 2.05e-12 (linear-solve conditioning, not element algebra).
# Gates: linear == closed form <= 1e-11; root moment <= 1e-10; |1-ratio| <= 1e-6
# at alpha=1e-3; the deviation must SHRINK ~quadratically as the load -> 0.
# ---------------------------------------------------------------------------
def gate_1(tmp: Path) -> None:
	L, w, t, n_el = 100.0, 1.0, 1.0, 20
	EI = EI_of(w, t)
	devs, rows = [], []
	for alpha in (1e-1, 1e-2, 1e-3):
		P = alpha * EI / L ** 2
		rec, _ = run_job(CONTACT, {
			"beam": slender_beam(L, n_el, t, w),
			"material": {"youngs_modulus_pa": E_PLA_PA},
			"supports": CLAMP,
			"loads": [{"node": "tip", "fy_n": -P}],
			"steps": {"n": 4},
		}, tmp, f"g1_a{alpha:g}")
		lin, nl = rec["linear"], rec["nonlinear"]
		exact = -P * L ** 3 / (3.0 * EI)
		lin_err = abs(lin["tip_uy_mm"] - exact) / abs(exact)
		# extreme-fibre stress from M_root = P L: sigma = P L (t/2) / I
		sig_exact = P * L * (t / 2.0) / (w * t ** 3 / 12.0)
		sig_err = abs(lin["max_abs_stress_mpa"] - sig_exact) / sig_exact
		ratio = nl["tip_uy_mm"] / lin["tip_uy_mm"]
		devs.append(abs(1.0 - ratio))
		rows.append((alpha, lin_err, sig_err, ratio))
		print(f"        alpha={alpha:.0e}  P={P:.4e} N  linear err {lin_err:.2e}  "
		      f"root-moment err {sig_err:.2e}  nl/lin {ratio:.10f}")
	worst_lin = max(r[1] for r in rows)
	worst_sig = max(r[2] for r in rows)
	gate("1 linear reference == PL^3/3EI (exact element)", worst_lin <= 1e-11,
	     f"max rel err {worst_lin:.3e} over alpha 1e-1..1e-3 (gate 1e-11; measured 2.73e-13)")
	gate("1 linear root moment == P*L", worst_sig <= 1e-10,
	     f"max rel err {worst_sig:.3e} on sigma_root = P L c / I (gate 1e-10; measured 2.05e-12)")
	gate("1 nonlinear -> linear as load -> 0", devs[-1] <= 1e-6,
	     f"|1 - delta_nl/delta_lin| = {devs[-1]:.3e} at alpha=1e-3 "
	     f"(gate 1e-6; measured 1.14e-7)")
	gate("1 deviation shrinks with load (O(alpha^2))",
	     devs[0] > devs[1] > devs[2] and devs[0] / devs[1] > 50.0 and devs[1] / devs[2] > 50.0,
	     f"|1-ratio| {devs[0]:.2e} -> {devs[1]:.2e} -> {devs[2]:.2e} "
	     f"(per-decade factors {devs[0] / devs[1]:.0f}, {devs[1] / devs[2]:.0f}; gate > 50 "
	     f"~ quadratic; measured ~100)")


# ---------------------------------------------------------------------------
# Gate 2 — LARGE DEFLECTION vs the EXACT elastica (derivation above).
# At alpha = PL^2/EI = 3 the exact solution is
#     phi_L = 0.9860169467 rad (56.4946 deg), delta/L = 0.6032534411,
#     x_tip/L = 0.7455798154,
# while LINEAR theory says delta/L = alpha/3 = 1.000 — a 66% over-prediction of
# deflection, i.e. the nonlinear structure is far STIFFER. That sign is the
# physics a broken corotational implementation gets wrong (drop the element
# rotation from the local kinematics and the solver degenerates to the linear
# answer), so it is asserted explicitly.
# The elastica is INEXTENSIBLE while this beam has finite EA, so there is a
# mesh-INDEPENDENT modelling difference of order (t/L)^2: the axial strain at
# alpha = 3 is ~ alpha t^2 / (12 L^2). The convergence study therefore runs at
# L/t = 500 (t = 0.2 mm), where that floor is ~1e-6 — two decades below the
# finest discretisation error. What is NOT true is hidden nowhere: the last
# sub-gate MEASURES the extensibility difference by sweeping t at fixed mesh
# and asserts it scales as t^2, proving the fine-mesh residual is physics, not
# a solver defect.
# Measured 2026-07-30 (delta/L relative error vs exact, L=100 mm, t=0.2 mm):
#     10 el 8.622e-4 · 20 el 2.149e-4 · 40 el 5.436e-5 · 80 el 1.431e-5
#     -> per-doubling ratios 4.012, 3.953, 3.798 (order 2.00, 1.98, 1.93)
#     x_tip/L errors 2.414e-4 / 6.133e-5 / 1.562e-5 / 4.153e-6
#   extensibility sweep at 80 elements: t=1.0 3.764e-5, t=0.5 1.942e-5,
#     t=0.2 1.431e-5 -> excess over the t=0.2 baseline 2.333e-5 -> 5.10e-6,
#     ratio 4.57 for a 2x change in t (t^2 predicts 4.0)
# Gates: 80-element error <= 2e-5; x error <= 6e-6; each halving ratio in
# [3.4, 4.6]; extensibility excess ratio in [3.0, 6.0]; delta_nl < 0.65 delta_lin.
# ---------------------------------------------------------------------------
def gate_2(tmp: Path) -> None:
	L, w, t = 100.0, 1.0, 0.2
	alpha = 3.0
	phi_x, a_chk, dv_exact, xt_exact = elastica_at_alpha(alpha)
	assert abs(a_chk - alpha) < 1e-12, f"elastica inversion failed: {a_chk}"

	def solve(t_mm: float, n_el: int, name: str):
		P = alpha * EI_of(w, t_mm) / L ** 2
		rec, _ = run_job(CONTACT, {
			"beam": slender_beam(L, n_el, t_mm, w),
			"material": {"youngs_modulus_pa": E_PLA_PA},
			"supports": CLAMP,
			"loads": [{"node": "tip", "fy_n": -P}],
			"steps": {"n": 20},
		}, tmp, name)
		nl = rec["nonlinear"]
		dv = -nl["tip_uy_mm"] / L
		xt = (L + nl["tip_ux_mm"]) / L
		return rec, dv, xt, abs(dv - dv_exact) / dv_exact, abs(xt - xt_exact) / xt_exact

	errs, x_errs, last = [], [], None
	for n_el in (10, 20, 40, 80):
		rec, dv, xt, ed, ex = solve(t, n_el, f"g2_n{n_el}")
		errs.append(ed)
		x_errs.append(ex)
		last = (rec, dv, xt)
		print(f"        {n_el:3d} el (t={t} mm): delta/L {dv:.8f} (err {ed:.3e})  "
		      f"x/L {xt:.8f} (err {ex:.3e})  phi_L {-rec['nonlinear']['tip_rz_rad']:.7f}")
	ratios = [errs[i] / errs[i + 1] for i in range(len(errs) - 1)]
	rec, dv, xt = last
	dl_lin = -rec["linear"]["tip_uy_mm"] / L
	print(f"        exact: delta/L {dv_exact:.10f}  x/L {xt_exact:.10f}  "
	      f"phi_L {phi_x:.7f} rad;  LINEAR delta/L {dl_lin:.6f}")
	gate("2 elastica delta/L @ 80 elements", errs[-1] <= 2.0e-5,
	     f"rel err {errs[-1]:.3e} vs exact {dv_exact:.10f} (gate 2e-5; measured 1.431e-5)")
	gate("2 elastica x_tip/L @ 80 elements", x_errs[-1] <= 6.0e-6,
	     f"rel err {x_errs[-1]:.3e} vs exact {xt_exact:.10f} (gate 6e-6; measured 4.15e-6)")
	gate("2 mesh CONVERGENCE ORDER ~ 2", all(3.4 <= r <= 4.6 for r in ratios),
	     f"per-doubling error ratios {['%.3f' % r for r in ratios]} "
	     f"(gate [3.4,4.6] ~ order 2; measured 4.012/3.953/3.798)")
	gate("2 PHYSICS SIGN: nonlinear STIFFER than linear", dv < 0.65 * dl_lin,
	     f"delta_nl/L {dv:.6f} vs delta_lin/L {dl_lin:.6f} "
	     f"(ratio {dv / dl_lin:.4f}; gate < 0.65; exact elastica ratio "
	     f"{dv_exact / (alpha / 3.0):.4f}) — a linear-kinematics bug lands at 1.0")

	# What is NOT true: the fine-mesh residual is not zero, because the solver's
	# beam is EXTENSIBLE and the elastica is not. Measure that difference
	# instead of hiding it: at fixed 80 elements it must scale as t^2.
	_r1, _d1, _x1, e_t10, _ = solve(1.0, 80, "g2_ext_t10")
	_r2, _d2, _x2, e_t05, _ = solve(0.5, 80, "g2_ext_t05")
	base = errs[-1]
	ex10, ex05 = e_t10 - base, e_t05 - base
	rat = ex10 / ex05 if ex05 > 0 else float("inf")
	print(f"        extensibility @80 el: err(t=1.0) {e_t10:.3e}, err(t=0.5) {e_t05:.3e}, "
	      f"err(t=0.2) {base:.3e} -> excess {ex10:.3e} / {ex05:.3e}, ratio {rat:.2f}")
	gate("2 fine-mesh residual IS extensibility (excess ~ t^2)",
	     e_t10 > e_t05 > base and 3.0 <= rat <= 6.0,
	     f"excess over the L/t=500 baseline falls {rat:.2f}x when t halves "
	     f"(gate [3.0,6.0]; t^2 predicts 4.0; measured 4.57) — the remaining "
	     f"{base:.2e} is discretisation, NOT a solver defect")


# ---------------------------------------------------------------------------
# Gate 3a — CONTACT: cantilever pressed onto a rigid plane.
# A penalty constraint is exact only in the limit kappa -> inf; what it
# guarantees is  penetration = p_n / kappa  at every contacting node. That is
# the honest statement, so it is what gets gated: (i) the deepest penetration
# equals max(p_n)/kappa to solver tolerance, (ii) it therefore falls as 1/kappa
# under refinement, and (iii) no node lies deeper than that below the surface.
# Global force balance (reactions + contact + applied = 0) is an algebraic
# IDENTITY of the assembled system, so it is pinned near machine precision — if
# it ever moves, the reaction/contact bookkeeping in the receipt is wrong. It is
# gated RELATIVE to the force scale because the contact force is kappa * (-g)
# and round-off in g (~eps * |y|) is amplified by kappa.
# NOTE the honest subtlety: a cantilever resting on a plane is statically
# INDETERMINATE, so the contact force itself depends slightly on kappa and
# pen*kappa is not a constant — it CONVERGES as kappa -> inf. That convergence
# is what gets gated, not a fake exact invariance.
# Case: L=60 mm, w=8, t=2 mm PLA cantilever, 2 N down at the tip (linear tip
# deflection 8.18 mm), rigid plane at y = -1.0 mm.
# Measured 2026-07-30: max penetration 1.755e-4 / 1.755e-5 / 1.755e-6 mm at
# kappa 1e4 / 1e5 / 1e6 N/mm; pen*kappa = 1.755443 / 1.755481 / 1.755485 N
# (successive changes 3.8e-5 then 4.0e-6 N — 9.5x per decade of kappa, i.e.
# O(1/kappa)); relative equilibrium residual <= 8.6e-12.
# ---------------------------------------------------------------------------
def gate_3a(tmp: Path) -> None:
	L, w, t, n_el = 60.0, 8.0, 2.0, 30
	gap = 1.0
	prods, pens, eqs = [], [], []
	for kappa in (1e4, 1e5, 1e6):
		rec, _ = run_job(CONTACT, {
			"beam": slender_beam(L, n_el, t, w),
			"material": {"youngs_modulus_pa": E_PLA_PA},
			"supports": CLAMP,
			"loads": [{"node": "tip", "fy_n": -2.0}],
			"obstacles": [{"kind": "plane", "point_mm": [0.0, -gap], "normal": [0.0, 1.0],
			               "penalty_n_per_mm": kappa}],
			"steps": {"n": 12},
		}, tmp, f"g3a_k{kappa:g}")
		pen = rec["insertion"]["max_penetration_mm"]
		last = rec["convergence"]["per_step"][-1]
		fn = max(o["total_normal_force_n"] for o in last["obstacles"])
		# deepest node below the plane, straight from the geometry receipt
		ys = np.array([p[1] for p in rec["nodes_final_mm"]])
		deepest = float(max(0.0, -gap - ys.min()))
		scale = 2.0 + fn                      # applied + carried contact force
		eq = max(abs(v) for v in last["equilibrium_residual_n"]) / scale
		prods.append(pen * kappa)
		pens.append(pen)
		eqs.append(eq)
		print(f"        kappa {kappa:.0e}: max_pen {pen:.3e} mm  pen*kappa {pen * kappa:.6f} N  "
		      f"sum p_n {fn:.4f} N  deepest node {deepest:.3e} mm  eq_resid_rel {eq:.2e}")
		gate(f"3a no node passes the plane @ kappa={kappa:.0e}",
		     deepest <= pen * (1.0 + 1e-9) + 1e-12,
		     f"deepest node {deepest:.3e} mm vs penalty tolerance {pen:.3e} mm")
	d1, d2 = abs(prods[1] - prods[0]), abs(prods[2] - prods[1])
	gate("3a penetration = p_n/kappa, converging as kappa -> inf",
	     max(prods) / min(prods) - 1.0 <= 1e-4 and d2 < d1 / 5.0,
	     f"pen*kappa = {['%.6f' % p for p in prods]} N across kappa 1e4..1e6 "
	     f"(spread {max(prods) / min(prods) - 1.0:.2e} <= 1e-4; successive change "
	     f"{d1:.2e} -> {d2:.2e} N, {d1 / d2:.1f}x per decade, gate > 5) -> penetration "
	     f"{pens[0]:.2e} -> {pens[-1]:.2e} mm; the residual kappa-dependence is real "
	     f"(the cantilever-on-plane problem is statically indeterminate), not numerical")
	gate("3a global force balance (reactions+contact+applied)", max(eqs) <= 1e-10,
	     f"max |sum F| / force scale = {max(eqs):.2e} (gate 1e-10; measured 8.6e-12 — an "
	     f"algebraic identity up to kappa-amplified round-off, so drift means the "
	     f"receipt bookkeeping is wrong)")


# ---------------------------------------------------------------------------
# Gate 3b — CONTACT EQUILIBRIUM against a NON-trivial closed form.
# Simply-supported beam: pin at x=0 (ux, uy fixed; rotation free), rigid
# cylinder acting as a roller under the tip at x=L, point load P down at x=a.
# Moments about the pin:   R_roller * L = P * a   ->   R_roller = P a / L,
# INDEPENDENT of the support stiffness (the system is statically determinate),
# so the penalty compliance cannot bias the answer — only geometric
# nonlinearity can, at order (delta/L)^2.
# L=60, a=20, P=2 N -> R_roller = 0.666667 N exactly. The cylinder starts
# 1e-4 mm inside the tip so contact is ACTIVE at u=0 (a tangent, non-penetrating
# start would leave the pinned beam a free-rotation mechanism -> singular
# tangent, which the runner would correctly refuse).
# Measured 2026-07-30: R_roller = 0.6666124 N, rel err 8.14e-5 (second-order
# moment-arm shortening), equilibrium residual 2.0e-13 N. Gate: <= 1e-3.
# ---------------------------------------------------------------------------
def gate_3b(tmp: Path) -> None:
	L, w, t, n_el = 60.0, 8.0, 2.0, 30
	a, P, R = 20.0, 2.0, 2.0
	rec, _ = run_job(CONTACT, {
		"beam": slender_beam(L, n_el, t, w),
		"material": {"youngs_modulus_pa": E_PLA_PA},
		"supports": [{"node": "root", "dofs": {"ux": 0.0, "uy": 0.0}}],
		"loads": [{"node": {"at_mm": a}, "fy_n": -P}],
		"obstacles": [{"kind": "cylinder", "center_mm": [L, -(R - 1e-4)], "radius_mm": R,
		               "side": "outside", "penalty_n_per_mm": 1e4, "nodes": ["tip"]}],
		"steps": {"n": 10},
	}, tmp, "g3b_roller")
	last = rec["convergence"]["per_step"][-1]
	fy = -last["obstacles"][0]["force_on_obstacle_n"][1]   # upward force ON the beam
	exact = P * a / L
	err = abs(fy - exact) / exact
	tip_uy = rec["nonlinear"]["tip_uy_mm"]
	eq = max(abs(v) for v in last["equilibrium_residual_n"])
	print(f"        roller force {fy:.7f} N vs statics P a/L = {exact:.7f} N  "
	      f"(tip uy {tip_uy:.4f} mm, eq_resid {eq:.2e} N)")
	gate("3b roller reaction == P*a/L (statics)", err <= 1e-3,
	     f"{fy:.7f} vs {exact:.7f} N, rel err {err:.3e} (gate 1e-3; measured 8.14e-5 — "
	     f"second-order moment-arm shortening, not a solver error)")
	gate("3b equilibrium identity holds with a curved obstacle", eq <= 1e-12,
	     f"max |sum F| = {eq:.2e} N (gate 1e-12)")


# ---------------------------------------------------------------------------
# Gate 4 — INSERTION FORCE CURVE for a ramped snap feature.
#
# Geometry: a latch arm (L=15, w=6, t=1.5 mm PLA cantilever, EI = 5568.75
# N mm^2) whose TIP rides a rigid catch that translates in -x — flat, 30-deg
# lead ramp rising 1.25 mm, 2 mm plateau, 60-deg retention face, flat. Every
# corner is filleted r = 0.5 mm: a printed catch has no zero-radius corner, and
# a zero-radius corner makes node-to-surface contact chatter (measured: the
# sharp-corner version failed to converge at the ramp crest, and the runner
# correctly refused rather than reporting the last iterate). Contact is
# restricted to the TIP node — a rounded latch nose, which is also what the
# closed form assumes.
#
# Closed form (standard snap-fit design guides, e.g. BASF/Bayer): with lead
# angle a and friction mu,  F_insert = P (mu + tan a)/(1 - mu tan a), where
# P = 3 EI y / L^3 is the force to deflect the arm by y. Frictionless it is
# F = P tan a, and that is EXACT for this contact model: the normal force is
# p_n * n_hat with n_hat = (-sin a, cos a), the beam supplies the vertical
# component P, so the horizontal component is P tan a identically.
# At the top of the STRAIGHT ramp y = 1.13301 mm (rise minus the crest-fillet
# rise), so P = 5.6084 N and F = 3.2380 N.
#
# The measured peak is BELOW that, and that sign is physics, not error: the
# contact pushes the tip toward the root, so the arm carries ~3 N of AXIAL
# COMPRESSION, and a compressive axial load softens a cantilever in the ratio
# ~N/P_cr with P_cr = pi^2 EI/(4L^2) = 61.08 N -> ~5% less force for the same
# imposed deflection. Both the magnitude and the sign are gated.
#
# The retention face gives a second closed form for free: released over a
# 60-deg face at the deflection the arm actually has there (0.95 mm, i.e. the
# plateau rise minus the fillet's r(1-cos 60)), |F| = 3 EI y/L^3 tan(60 deg) =
# 4.7024 * 1.7321 = 8.1447 N of PULL-IN — the number a designer wants for
# "how hard does this latch resist being pulled back out".
#
# Measured 2026-07-30 (80 steps): peak +3.0069 N at travel 2.1750 mm (-7.14%
# vs the naive closed form, -1.94% vs the beam-column-corrected 3.0663 N);
# force exactly 0 through engagement's plateau and >= 0 everywhere before the
# catch; retention (most negative) -8.3335 N at travel 4.8750 mm (+2.32% vs
# the closed form); final force 0.0 N; path-max root strain 1.205%; max
# penetration 2.005e-4 mm at kappa = 5e4 N/mm.
# ---------------------------------------------------------------------------
def catch_profile(x_start, y0, rise, lead_deg, plateau_mm, back_deg, r, nf=8):
	"""Rigid catch terrain as a filleted polyline (x ascending).

	Walks the surface tangent angle: flat -> +lead -> plateau -> -back -> flat,
	inserting a radius-r arc at every angle change. A CONCAVE turn (angle
	increasing) puts the arc centre above the surface, a CONVEX turn below;
	either way the arc starts exactly at the current point, so the polyline is
	C1 to within the arc discretisation."""
	pts = [[-100.0, y0]]
	x, y, th = float(x_start), float(y0), 0.0
	pts.append([x, y])

	def arc(th1):
		nonlocal x, y, th
		concave = th1 > th
		if concave:
			cx, cy = x - r * math.sin(th), y + r * math.cos(th)
			for a in np.linspace(th, th1, nf + 1)[1:]:
				pts.append([cx + r * math.sin(a), cy - r * math.cos(a)])
		else:
			cx, cy = x + r * math.sin(th), y - r * math.cos(th)
			for a in np.linspace(th, th1, nf + 1)[1:]:
				pts.append([cx - r * math.sin(a), cy + r * math.cos(a)])
		x, y = pts[-1]
		th = th1

	def straight(dx):
		nonlocal x, y
		x, y = x + dx, y + dx * math.tan(th)
		pts.append([x, y])

	lead, back = math.radians(lead_deg), math.radians(back_deg)
	arc(lead)
	# straight rise = total rise minus what the two lead-ramp fillets already gave
	straight((rise - 2.0 * r * (1.0 - math.cos(lead))) / math.tan(lead))
	ramp_top = (x, y)
	arc(0.0)
	straight(plateau_mm)
	arc(-back)
	face_top = (x, y)          # start of the STRAIGHT retention face
	# descending: dx stays POSITIVE (x must ascend), tan(th) < 0 carries y down
	straight((rise - 2.0 * r * (1.0 - math.cos(back))) / math.tan(back))
	arc(0.0)
	pts.append([200.0, y])
	return pts, ramp_top, face_top


def gate_4(tmp: Path) -> None:
	L, w, t, n_el = 15.0, 6.0, 1.5, 20
	EI = EI_of(w, t)
	lead_deg, back_deg, rise, flat_y, r = 30.0, 60.0, 1.25, -0.05, 0.5
	pts, ramp_top, face_top = catch_profile(L, flat_y, rise, lead_deg, 2.0, back_deg, r)
	travel = 6.0
	rec, _ = run_job(CONTACT, {
		"beam": slender_beam(L, n_el, t, w),
		"material": "PLA",
		"supports": CLAMP,
		"obstacles": [{"kind": "profile", "points_mm": pts, "side": "above",
		               "penalty_n_per_mm": 5e4, "nodes": ["tip"],
		               "motion": {"dir": [-1.0, 0.0], "travel_mm": travel}}],
		"steps": {"n": 80},
	}, tmp, "g4_snap")
	curve = np.load(rec["curve_npy"])
	cols = rec["curve_columns"]
	trav = curve[:, cols.index("obstacle_travel_mm")]
	force = curve[:, cols.index("insertion_force_n")]
	uy = curve[:, cols.index("tip_uy_mm")]
	ins = rec["insertion"]

	# travel at which the straight ramp ends, and at which the plateau ends
	trav_ramp_top = ramp_top[0] - L
	trav_plateau_end = trav_ramp_top + r * math.sin(math.radians(lead_deg)) + 2.0
	y_ramp_top = ramp_top[1]
	P_cf = 3.0 * EI * y_ramp_top / L ** 3
	f_cf = P_cf * math.tan(math.radians(lead_deg))
	p_cr = math.pi ** 2 * EI / (4.0 * L ** 2)          # cantilever Euler load
	f_bc = f_cf * (1.0 - f_cf / p_cr)                  # beam-column-softened estimate
	peak = ins["peak_force_n"]
	err, err_bc = (peak - f_cf) / f_cf, (peak - f_bc) / f_bc
	# Retention: same construction as the lead ramp — the face angle only reaches
	# `back_deg` AFTER the plateau-to-face fillet, by which point the arm has
	# already relaxed by r(1 - cos back). Use the deflection there, not the full
	# plateau deflection, or the closed form is comparing against a geometry the
	# part never occupies.
	f_ret_cf = 3.0 * EI * face_top[1] / L ** 3 * math.tan(math.radians(back_deg))
	f_ret = -ins["min_force_n"]
	engaged = trav <= trav_plateau_end + 1e-9
	after = trav >= trav[-1] - 0.5
	print(f"        peak {peak:.4f} N at travel {ins['peak_at_travel_mm']:.4f} mm "
	      f"(tip up {float(uy[int(np.argmax(force))]):.4f} mm); closed form P tan(30) = "
	      f"{f_cf:.4f} N (P = {P_cf:.4f} N), beam-column corrected {f_bc:.4f} N "
	      f"(P_cr = {p_cr:.2f} N)")
	print(f"        retention {f_ret:.4f} N at travel {ins['peak_at_travel_mm']:.2f}->"
	      f"{float(trav[int(np.argmin(force))]):.4f} mm vs P tan(60) = {f_ret_cf:.4f} N; "
	      f"final {ins['final_force_n']:.2e} N; path-max strain "
	      f"{rec['path_max']['strain'] * 100:.3f}% at travel "
	      f"{rec['path_max']['at_obstacle_travel_mm']:.3f} mm; max penetration "
	      f"{ins['max_penetration_mm']:.3e} mm")
	gate("4 peak insertion force vs snap-fit closed form", abs(err) <= 0.12,
	     f"measured {peak:.4f} N vs F = P tan(30 deg) = {f_cf:.4f} N, {err * 100:+.2f}% "
	     f"(gate +-12%; measured -7.65%)")
	gate("4 PHYSICS: peak below the small-deflection closed form (beam-column)",
	     peak < f_cf and abs(err_bc) <= 0.06,
	     f"the {peak:.3f} N contact force compresses the arm at {peak / p_cr * 100:.1f}% "
	     f"of its Euler load {p_cr:.2f} N, softening it; corrected estimate {f_bc:.4f} N, "
	     f"measured {err_bc * 100:+.2f}% off it (gate: below f_cf AND within +-6% of f_bc)")
	gate("4 force NON-NEGATIVE through engagement",
	     float(force[engaged].min()) >= -1e-9,
	     f"min over the {int(engaged.sum())} steps up to travel "
	     f"{trav_plateau_end:.3f} mm (end of plateau) = {float(force[engaged].min()):.3e} N "
	     f"(gate >= -1e-9)")
	gate("4 retention (negative) lobe vs P tan(60 deg)",
	     abs(f_ret - f_ret_cf) / f_ret_cf <= 0.10,
	     f"pull-in peak {f_ret:.4f} N vs closed form 3EI y/L^3 tan(60) = {f_ret_cf:.4f} N "
	     f"at the face-top deflection {face_top[1]:.4f} mm "
	     f"({(f_ret - f_ret_cf) / f_ret_cf * 100:+.2f}%, gate +-10%; measured +2.32%) — "
	     f"the RETENTION force, and the reason 'non-negative' is asserted only through "
	     f"ENGAGEMENT: releasing over the catch pulls the latch in.")
	gate("4 force returns to ~0 after full engagement",
	     abs(ins["final_force_n"]) <= 1e-9 and float(np.abs(force[after]).max()) <= 1e-9,
	     f"|force| <= {float(np.abs(force[after]).max()):.2e} N over the last 0.5 mm of "
	     f"travel (past the retention face); final {ins['final_force_n']:.2e} N (gate 1e-9)")
	gate("4 peak occurs ON the lead ramp, not on the plateau",
	     ins["peak_at_travel_mm"] <= trav_ramp_top + r + 1e-9,
	     f"peak at travel {ins['peak_at_travel_mm']:.4f} mm; straight ramp ends at "
	     f"{trav_ramp_top:.4f} mm (a horizontal plateau can carry no insertion force)")


# ---------------------------------------------------------------------------
# Gate 5 — FATIGUE ARITHMETIC (exact by construction, machine-precision gates).
#
# 5a Miner, hand-computed. Synthetic curve sigma_a = 100 N^-0.1, i.e.
#    N = (sigma_a/100)^-10. Block 1: sigma_a = 50 MPa -> N1 = 2^10 = 1024
#    (exactly). Block 2: sigma_a = 25 MPa -> N2 = 4^10 = 1048576 (exactly).
#    n1 = 100, n2 = 10000 ->
#      D = 100/1024 + 10000/1048576
#        = 0.09765625 + 0.0095367431640625
#        = 0.1071929931640625    (an exact dyadic rational — no rounding)
#      repeats to failure = 1/D = 9.328855140186916...
#    Both terms are exactly representable in binary64, so this gate is a pure
#    arithmetic pin at 1e-15 relative.
#
# 5b Basquin round-trip. Plant a = 137.5 MPa, b = -0.0917, sample 12 decades-
#    spread lives, fit in log-log, recover the coefficients. Gate 1e-12
#    relative (the mission asked for 1e-6; the fit is a straight line through
#    exactly-collinear points, so 1e-6 would be 6 decades of slack).
#
# 5c Goodman / Gerber hand calculations. sigma_a = 12, sigma_m = 8,
#    sigma_u = 40 MPa:
#      Goodman sigma_ar = 12 / (1 - 8/40)     = 12 / 0.8  = 15.0   MPa exactly
#      Gerber  sigma_ar = 12 / (1 - (8/40)^2) = 12 / 0.96 = 12.5   MPa exactly
#    Verified END-TO-END through the CLI: the runner is fed sigma_a/sigma_m and
#    a curve whose life at 15.0 (resp. 12.5) MPa is known, so a wrong
#    correction cannot hide inside the life number.
#
# 5d PLA registry end-to-end: the design curve must resolve from
#    tools/materials/fatigue.json, re-derive its own (a, b) from the stored
#    primitives (k = 5.5, sigma_max = 0.1 x 40.9 MPa at 2e6), and produce the
#    life the closed form predicts.
# ---------------------------------------------------------------------------
def gate_5(tmp: Path) -> None:
	inline = {"name": "synthetic", "sigma_uts_mpa": 1000.0,
	          "curve": {"a_mpa": 100.0, "b": -0.1, "stress_measure": "amplitude"}}
	rec, _ = run_job(FATIGUE, {
		"material": inline, "mean_stress": "none",
		"stress": {"sigma_ref_mpa": 1.0},
		"spectrum": [{"name": "hi", "cycles": 100, "sigma_a_mpa": 50.0},
		             {"name": "lo", "cycles": 10000, "sigma_a_mpa": 25.0}],
	}, tmp, "g5a_miner")
	d = rec["damage"]["total_at_critical_location"]
	d_exact = 100.0 / 1024.0 + 10000.0 / 1048576.0
	n1 = rec["blocks"][0]["cycles_to_failure_at_critical"]
	n2 = rec["blocks"][1]["cycles_to_failure_at_critical"]
	rep = rec["damage"]["spectrum_repeats_to_failure"]
	err_d = abs(d - d_exact) / d_exact
	print(f"        N1 {n1:.10f} (exact 1024)  N2 {n2:.10f} (exact 1048576)  "
	      f"D {d:.17f} (exact {d_exact:.17f})  repeats {rep:.10f}")
	gate("5a Miner 2-block damage == hand calculation",
	     err_d <= 1e-15 and abs(n1 - 1024.0) <= 1e-9 and abs(n2 - 1048576.0) <= 1e-6,
	     f"D = {d:.17f} vs exact {d_exact:.17f}, rel err {err_d:.2e} (gate 1e-15); "
	     f"N1 {n1:.6f}/1024, N2 {n2:.4f}/1048576")
	gate("5a spectrum repeats to failure == 1/D",
	     abs(rep - 1.0 / d_exact) / (1.0 / d_exact) <= 1e-15,
	     f"{rep:.12f} vs 1/D = {1.0 / d_exact:.12f}")

	# 5b Basquin round-trip (in-process — the fit is library API, not CLI)
	sys.path.insert(0, str(TOOLS))
	import importlib.util
	spec_m = importlib.util.spec_from_file_location("_fat_gate", FATIGUE)
	fat = importlib.util.module_from_spec(spec_m)
	spec_m.loader.exec_module(fat)
	a_true, b_true = 137.5, -0.0917
	N = np.logspace(2.0, 6.5, 12)
	S = a_true * N ** b_true
	a_fit, b_fit, r2 = fat.basquin_fit(N, S)
	ea, eb = abs(a_fit - a_true) / a_true, abs(b_fit - b_true) / abs(b_true)
	gate("5b Basquin fit round-trip recovers planted coefficients",
	     ea <= 1e-12 and eb <= 1e-12 and r2 >= 1.0 - 1e-12,
	     f"a {a_fit:.12f}/{a_true} (err {ea:.2e}), b {b_fit:.12f}/{b_true} "
	     f"(err {eb:.2e}), R^2 {r2:.15f} (gate 1e-12; mission asked 1e-6)")

	# 5c Goodman / Gerber end-to-end through the CLI
	for model, expect in (("goodman", 15.0), ("gerber", 12.5)):
		rec, _ = run_job(FATIGUE, {
			"material": {"name": "synthetic", "sigma_uts_mpa": 40.0,
			             "curve": {"a_mpa": 100.0, "b": -0.1, "stress_measure": "amplitude"}},
			"mean_stress": model,
			"stress": {"sigma_ref_mpa": 1.0},
			"spectrum": [{"cycles": 1000, "sigma_a_mpa": 12.0, "sigma_m_mpa": 8.0}],
		}, tmp, f"g5c_{model}")
		s_eff = rec["blocks"][0]["sigma_effective_mpa_max"]
		n_life = rec["blocks"][0]["cycles_to_failure_at_critical"]
		n_exact = (expect / 100.0) ** (-10.0)
		ok = abs(s_eff - expect) <= 1e-12 and abs(n_life - n_exact) / n_exact <= 1e-12
		gate(f"5c {model} correction == hand calculation", ok,
		     f"sigma_ar {s_eff:.15f} vs exact {expect} (sigma_a 12, sigma_m 8, "
		     f"sigma_u 40 MPa); life {n_life:.6f} vs {n_exact:.6f} (gate 1e-12)")

	# 5d PLA registry end-to-end
	rec, code = run_job(FATIGUE, {
		"material": "PLA", "curve": "design",
		"stress": {"sigma_ref_mpa": 6.0},
		"spectrum": [{"name": "duty", "cycles": 20000, "load_factor": 1.0, "r_ratio": 0.0}],
	}, tmp, "g5d_pla")
	c = rec["curve"]
	# NB: 4.09 exactly as the record stores it, NOT 0.1*40.9 — those two differ
	# by 1 ulp in binary64 and the exponent 1/b = -5.5 amplifies that into 1.6e-12
	# of life, which would make a machine-precision gate a lie about the source
	# of the discrepancy.
	a_exact = 4.09 * 2.0e6 ** (1.0 / 5.5)
	b_exact = -1.0 / 5.5
	n_life = rec["blocks"][0]["cycles_to_failure_at_critical"]
	n_exact = (6.0 / a_exact) ** (1.0 / b_exact)
	band = rec["confidence"]["life_scatter_factor_90_10"]
	print(f"        PLA design curve a {c['a_mpa']:.10f} MPa (exact {a_exact:.10f}), "
	      f"b {c['b']:.12f}; life at 6 MPa max = {n_life:.1f} cycles (exact {n_exact:.1f}); "
	      f"scatter band {band['min']:.2f}x..{band['max']:.2f}x")
	ok = (code == 0 and rec["material"]["name"] == "PLA"
	      and rec["material"]["status"] == "measured"
	      and c["a_mpa"] == a_exact and c["b"] == b_exact
	      and abs(n_life - n_exact) / n_exact <= 1e-14
	      and rec["sigma_uts_mpa"] == 40.9
	      and rec["confidence"]["mean_stress_model"] == "intrinsic")
	gate("5d PLA registry curve end-to-end", ok,
	     f"a {c['a_mpa']:.6f} MPa / b {c['b']:.9f} re-derived from k=5.5 and "
	     f"sigma_max=0.1x40.9 MPa at 2e6 (Ezeh & Susmel 2019 eq. 5-6); life at "
	     f"sigma_max 6 MPa = {n_life:.1f} cycles, D = "
	     f"{rec['damage']['total_at_critical_location']:.6f}")
	gate("5d life-scatter band recomputed from the source table",
	     band is not None and abs(band["min"] - 1.185 ** 7.7) <= 1e-9
	     and abs(band["max"] - 2.174 ** 5.8) <= 1e-9,
	     f"90/10 LIFE band {band['min']:.3f}x .. {band['max']:.2f}x "
	     f"(= T_sigma^k over Ezeh & Susmel Table 1; best row (7.7, 1.185), "
	     f"worst row (5.8, 2.174)) — quoted in every receipt")


# ---------------------------------------------------------------------------
# Gate 6 — NEGATIVE CONTROLS. A gate that cannot fail is not a gate; a solver
# that answers an ill-posed request is worse than no solver. Each control must
# exit NONZERO with ok:false and a POINTED error. Positive controls prove exit
# 0 is reachable, so a nonzero exit is informative.
# ---------------------------------------------------------------------------
def gate_6(tmp: Path) -> None:
	def refuses(runner, name, job, needle):
		rec, code = run_job(runner, job, tmp, name, expect_ok=False)
		err = str(rec.get("error", ""))
		ok = code != 0 and rec.get("ok") is False and needle.lower() in err.lower()
		gate(f"6 {name} refuses", ok,
		     f"exit {code}, error {err[:130]!r} (needs substring {needle!r})")

	_r, code = run_job(CONTACT, {
		"beam": slender_beam(30.0, 10),
		"material": {"youngs_modulus_pa": E_PLA_PA},
		"supports": CLAMP, "loads": [{"node": "tip", "fy_n": -0.01}],
		"steps": {"n": 2},
	}, tmp, "g6_positive_contact")
	gate("6 contact positive control exits 0", code == 0, f"returncode {code}")

	# (a) NON-CONVERGENT CONTACT, absurd STEP: drive the latch arm 2.1 mm up the
	# rigid ramp — the whole nonlinear engagement — in ONE increment with a
	# 2-iteration budget. Two Newton iterations cannot solve it, and the runner
	# must SAY SO and exit 1 rather than report the last iterate (its residual
	# is 4.8e-5 relative, small enough to look plausible if it were reported).
	pts, _rt, _ft = catch_profile(15.0, -0.05, 1.25, 30.0, 2.0, 60.0, 0.5)
	ramp_job = {
		"beam": slender_beam(15.0, 20, 1.5, 6.0),
		"material": {"youngs_modulus_pa": E_PLA_PA},
		"supports": CLAMP,
		"obstacles": [{"kind": "profile", "points_mm": pts, "side": "above",
		               "penalty_n_per_mm": 5e4, "nodes": ["tip"],
		               "motion": {"dir": [-1.0, 0.0], "travel_mm": 2.1}}],
		"steps": {"n": 1, "max_iter": 2},
	}
	refuses(CONTACT, "absurd single-step contact engagement", ramp_job, "converge")

	# (b) PHYSICAL non-convergence: a shallow arch (40 mm span, 1 mm rise, pinned
	# both ends) loaded at the apex past its SNAP-THROUGH LIMIT POINT. Beyond the
	# limit load there is NO equilibrium on the fundamental path, so no iteration
	# budget can help — the honest answer is a refusal. Measured 2026-07-30:
	# 0.5 N fails at lambda = 0.75 (i.e. a limit load near 0.375 N).
	xs = np.linspace(0.0, 40.0, 11)
	arch_nodes = [[float(x), float(1.0 * (1.0 - ((x - 20.0) / 20.0) ** 2))] for x in xs]
	refuses(CONTACT, "snap-through past a limit point", {
		"beam": {"nodes_mm": arch_nodes, "section": {"width_mm": 1.0, "thickness_mm": 1.0}},
		"material": {"youngs_modulus_pa": E_PLA_PA},
		"supports": [{"node": 0, "dofs": {"ux": 0.0, "uy": 0.0}},
		             {"node": 10, "dofs": {"ux": 0.0, "uy": 0.0}}],
		"loads": [{"node": 5, "fy_n": -0.5}],
		"steps": {"n": 20},
	}, "converge")

	# (a') WHAT IS *NOT* TRUE, stated instead of faked: an absurd PENALTY on its
	# own does NOT break this solver. 1e14 N/mm against a latch arm whose tip
	# stiffness is 4.95 N/mm is 2e13x over-stiff, and driving the tip over the
	# entire catch in ONE increment still converges, because the Crisfield energy
	# line search absorbs the free-flight step that penalty contact provokes.
	# Gated as the positive result it is — with penetration bounded by p_n/kappa.
	rec, code = run_job(CONTACT, dict(ramp_job,
	                                  obstacles=[dict(ramp_job["obstacles"][0],
	                                                  penalty_n_per_mm=1e14)],
	                                  steps={"n": 1, "max_iter": 30}),
	                    tmp, "g6_absurd_penalty")
	pen = rec["insertion"]["max_penetration_mm"]
	gate("6 absurd PENALTY alone does not break the solver (stated, not faked)",
	     code == 0 and rec["ok"] and pen <= 1e-9,
	     f"kappa = 1e14 N/mm (2e13x the arm's 4.95 N/mm tip stiffness), whole "
	     f"engagement in one step: exit {code}, converged, max penetration "
	     f"{pen:.2e} mm — the non-convergence control above therefore uses an "
	     f"absurd STEP, not an absurd penalty")
	refuses(CONTACT, "unsupported beam (rigid-body modes)", {
		"beam": slender_beam(30.0, 4),
		"material": {"youngs_modulus_pa": E_PLA_PA},
		"supports": [], "loads": [{"node": "tip", "fy_n": -0.01}],
	}, "supports required")
	refuses(CONTACT, "zero penalty stiffness", {
		"beam": slender_beam(30.0, 4),
		"material": {"youngs_modulus_pa": E_PLA_PA}, "supports": CLAMP,
		"obstacles": [{"kind": "plane", "point_mm": [0.0, -1.0], "normal": [0.0, 1.0],
		               "penalty_n_per_mm": 0.0}],
	}, "not a constraint")

	# (b) FATIGUE: no credible printed S-N data -> refuse BY NAME.
	base = {"stress": {"sigma_ref_mpa": 5.0}, "spectrum": [{"cycles": 1000, "r_ratio": 0.0}]}
	refuses(FATIGUE, "PETG (insufficient printed S-N data)",
	        dict(base, material="PETG"), "insufficient")
	refuses(FATIGUE, "ABS (insufficient printed S-N data)",
	        dict(base, material="ABS"), "insufficient")
	refuses(FATIGUE, "TPU95A (unknown printed S-N data)",
	        dict(base, material="TPU95A"), "unknown")
	refuses(FATIGUE, "across-layer fatigue (no Z data anywhere)",
	        dict(base, material="PLA", load_orientation="across_layer"), "across_layer")
	refuses(FATIGUE, "goodman stacked on a max-stress curve (double count)",
	        dict(base, material="PLA", curve="design", mean_stress="goodman"), "double-count")
	refuses(FATIGUE, "peak stress above the printed ultimate",
	        {"material": "PLA", "curve": "design", "stress": {"sigma_ref_mpa": 60.0},
	         "spectrum": [{"cycles": 10, "r_ratio": 0.0}]}, "static failure")

	# (c) ZERO-AMPLITUDE SPECTRUM: implemented as INFINITE life with an explicit
	# status, not as a refusal — Basquin genuinely gives N = inf at sigma_a = 0.
	# What is gated is that the status is EXPLICIT and the life number is null
	# rather than a silently-huge integer.
	rec, code = run_job(FATIGUE, {
		"material": {"name": "synthetic", "sigma_uts_mpa": 40.0,
		             "curve": {"a_mpa": 100.0, "b": -0.1, "stress_measure": "amplitude"}},
		"mean_stress": "none", "stress": {"sigma_ref_mpa": 0.0},
		"spectrum": [{"cycles": 1e9, "load_factor": 1.0, "r_ratio": -1.0}],
	}, tmp, "g6_zero_amp")
	dmg = rec["damage"]
	ok = (code == 0 and dmg["life_status"] == "no_damage"
	      and dmg["total_at_critical_location"] == 0.0
	      and dmg["cycles_to_failure"] is None
	      and dmg["spectrum_repeats_to_failure"] is None
	      and rec["blocks"][0]["zero_amplitude"] is True
	      and rec["blocks"][0]["cycles_to_failure_at_critical"] is None)
	gate("6 zero-amplitude spectrum -> explicit no-damage status", ok,
	     f"life_status {dmg['life_status']!r}, D {dmg['total_at_critical_location']}, "
	     f"cycles_to_failure {dmg['cycles_to_failure']!r}, "
	     f"zero_amplitude flag {rec['blocks'][0]['zero_amplitude']} "
	     f"(implemented as infinite-life-within-model, NOT a refusal; "
	     f"note: {dmg['note'][:60]}...)")


# ---------------------------------------------------------------------------
# Gate 7 — META-NEGATIVE CONTROL (the ace_thermal precedent, adopted).
# Break ONE constant in a scratch copy of each runner and re-run the relevant
# gates against the broken copy: the suite MUST go red. This proves the gates
# can fail, i.e. that green means something.
#   contact: delete the element rotation `alpha` from the local rotation
#            measure (`th[i] - alpha` -> `th[i]`). That is EXACTLY the
#            corotational bug — the solver degenerates toward linear kinematics
#            and the large-deflection physics sign is lost.
#   fatigue: flip the sign of the Basquin exponent used for life
#            (`(s/a)**(1/b)` -> `(s/a)**(-1/b)`), so life GROWS with stress.
# Both scratch copies are deleted before this function returns.
# ---------------------------------------------------------------------------
def gate_7(tmp: Path) -> None:
	scratch = tmp / "meta"
	scratch.mkdir(exist_ok=True)

	def broken_run(label, src: Path, dst: Path, old: str, new: str, env_key: str, gates: str):
		text = src.read_text(encoding="utf-8")
		if text.count(old) != 1:
			gate(f"7 meta-control setup ({label})", False,
			     f"expected exactly one occurrence of {old!r} in {src.name}, "
			     f"found {text.count(old)}")
			return None
		dst.write_text(text.replace(old, new), encoding="utf-8")
		env = dict(os.environ)
		env[env_key] = str(dst)
		proc = subprocess.run([sys.executable, str(Path(__file__).resolve()), "--gates", gates],
		                      capture_output=True, text=True, env=env)
		fired = [l.strip() for l in proc.stdout.splitlines() if "<<< FAIL" in l]
		dst.unlink(missing_ok=True)
		return proc.returncode, fired

	out = broken_run("contact-kinematics", CONTACT, scratch / "broken_contact_a.py",
	                 "th_i = th[i] - alpha", "th_i = th[i]",
	                 "ACE_CONTACT_RUNNER", "1,2,3,4")
	if out is not None:
		code, fired = out
		names = [f.split(":")[0].replace("<<< FAIL", "").strip() for f in fired]
		gate("7 broken contact KINEMATICS turns the suite RED", code != 0 and len(fired) > 0,
		     f"scratch copy with the corotational rotation removed "
		     f"(th_i = theta_i - alpha -> theta_i): gates 1-4 exit {code} with "
		     f"{len(fired)} red gate(s): {names}")

	# Second break, chosen to produce WRONG NUMBERS rather than a refusal: a
	# 10x-wrong unit conversion leaves the tangent perfectly consistent, so the
	# solver converges happily to the wrong answer. That is the failure mode a
	# gate suite exists to catch.
	out = broken_run("contact-units", CONTACT, scratch / "broken_contact_b.py",
	                 'out["youngs_modulus_mpa"] = float(e_pa) * 1e-6',
	                 'out["youngs_modulus_mpa"] = float(e_pa) * 1e-5',
	                 "ACE_CONTACT_RUNNER", "1,2,3,4")
	if out is not None:
		code, fired = out
		names = [f.split(":")[0].replace("<<< FAIL", "").strip() for f in fired]
		gate("7 broken contact UNIT CONSTANT turns the suite RED",
		     code != 0 and len(fired) > 0,
		     f"scratch copy with Pa->MPa scaled 1e-6 -> 1e-5 (converges fine, answers "
		     f"10x stiff): gates 1-4 exit {code} with {len(fired)} red gate(s): {names}")

	out = broken_run("fatigue", FATIGUE, scratch / "broken_fatigue.py",
	                 "out[pos] = (s[pos] / a_mpa) ** (1.0 / b)",
	                 "out[pos] = (s[pos] / a_mpa) ** (-1.0 / b)",
	                 "ACE_FATIGUE_RUNNER", "5")
	if out is not None:
		code, fired = out
		names = [f.split(":")[0].replace("<<< FAIL", "").strip() for f in fired]
		gate("7 broken fatigue solver turns the suite RED", code != 0 and len(fired) > 0,
		     f"scratch copy with the Basquin exponent sign flipped "
		     f"(N = (sigma/a)^(1/b) -> ^(-1/b)): gate 5 exits {code} with "
		     f"{len(fired)} red gate(s): {names}")
	shutil.rmtree(scratch, ignore_errors=True)
	gate("7 scratch copies deleted", not scratch.exists(),
	     f"{scratch} removed (no temporary diagnostics left behind)")


GATES = {"1": gate_1, "2": gate_2, "3": lambda t: (gate_3a(t), gate_3b(t)),
         "4": gate_4, "5": gate_5, "6": gate_6, "7": gate_7}


def main(argv: list[str]) -> int:
	want = list("1234567")
	if "--gates" in argv:
		want = [g.strip() for g in argv[argv.index("--gates") + 1].split(",") if g.strip()]
	print(f"ace_contact + ace_fatigue benchmark gates")
	print(f"  contact runner: {CONTACT}")
	print(f"  fatigue runner: {FATIGUE}")
	with tempfile.TemporaryDirectory(prefix="ace_contact_fatigue_gates_") as td:
		tmp = Path(td)
		for g in want:
			if g not in GATES:
				print(f"unknown gate {g!r}; known {sorted(GATES)}")
				return 2
			try:
				GATES[g](tmp)
			except Exception as exc:  # noqa: BLE001 — an exception IS a red gate
				gate(f"{g} gate raised", False, f"{type(exc).__name__}: {exc}")
	n_fail = sum(1 for _, ok, _ in results if not ok)
	print(f"\n{'ALL GATES GREEN' if n_fail == 0 else f'{n_fail} GATE(S) RED'} "
	      f"({len(results) - n_fail}/{len(results)} pass)")
	return 0 if n_fail == 0 else 1


if __name__ == "__main__":
	sys.exit(main(sys.argv[1:]))
