#!/usr/bin/env python3
"""test_ace_thermal.py — benchmark gate suite for tools/ace_thermal_runner.py.

The solver is guilty until these gates are green (DESIGN_GUIDE §25.7: a
written solver must first be proven on closed-form benchmarks + a convergence
check). Every gate re-derives its analytic truth in a comment, runs the REAL
CLI contract (subprocess, JSON receipt on the last stdout line), and asserts
against bands frozen from MEASURED behavior (2026-07-30, this machine —
measured values quoted per gate; headroom stated, never silent).

Run:  python3 tools/test_ace_thermal.py     -> exit 0 iff all gates pass.

Gates:
  1a  1-D slab, fixed T both faces  -> exact linear profile + exact flux
  1b  1-D slab + uniform volumetric source -> quadratic profile; CONVERGENCE
      ORDER asserted (error ~4x down per voxel halving = 2nd order)
  2   radial cylinder-wall conduction vs ln-profile (staircase-limited ~O(h);
      band from measured convergence)
  3   Robin-cooled slab (Bi ~ 4.6) vs series-resistance closed form
  4   transient semi-infinite solid vs erfc solution at 3 depths x 3 times;
      discrete energy balance < 1e-6 relative; backward-Euler unconditional-
      stability probe at ~14x the explicit dt limit
  5   negative controls: no-BC, zero-k, empty-domain, unanchored-component
      manifests must refuse loudly (exit != 0 + ok:false + pointed error)
"""
from __future__ import annotations

import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

TOOLS = Path(__file__).resolve().parent
RUNNER = TOOLS / "ace_thermal_runner.py"

K_PLA = 0.13          # W/(m*K) — PLA-class conductivity used across the gates
BIG = 1e9             # "whole domain" box bound (mm)

results: list[tuple[str, bool, str]] = []


def gate(label: str, ok: bool, detail: str) -> None:
	results.append((label, ok, detail))
	print(f"  {'PASS' if ok else '<<< FAIL'}  {label}: {detail}")


def run_job(job: dict, tmp: Path, name: str, expect_ok: bool = True):
	"""Run the runner CLI; return (receipt, returncode). The LAST non-empty
	stdout line is the receipt (the ACE-family contract)."""
	job_path = tmp / f"{name}.json"
	job.setdefault("out_dir", str(tmp / name))
	job_path.write_text(json.dumps(job), encoding="utf-8")
	proc = subprocess.run([sys.executable, str(RUNNER), str(job_path)],
	                      capture_output=True, text=True)
	lines = [l for l in proc.stdout.strip().splitlines() if l.strip()]
	rec = json.loads(lines[-1]) if lines else {"ok": False, "error": "no stdout receipt"}
	if expect_ok and not rec.get("ok"):
		raise AssertionError(f"{name}: runner failed unexpectedly: {rec.get('error')}\n{proc.stderr[-2000:]}")
	return rec, proc.returncode


def slab_job(nx: int, h: float, bcs, extra=None) -> dict:
	job = {"voxel_mm": h, "shape": [nx, 4, 4], "solid": "full",
	       "material": {"k_w_mk": K_PLA}, "bcs": bcs}
	job.update(extra or {})
	return job


def x_faces(nx: int, h: float):
	"""(box, faces) selectors for the x=0 and x=L slab end faces."""
	L = nx * h
	lo = {"box_mm": [[-1, -1, -1], [1e-3 * h, BIG, BIG]], "faces": ["-x"]}
	hi = {"box_mm": [[L - 1e-3 * h, -1, -1], [L + 1, BIG, BIG]], "faces": ["+x"]}
	return lo, hi


# ---------------------------------------------------------------------------
# Gate 1a — 1-D slab, Dirichlet both ends.
# Derivation: steady 1-D, no source: d/dx(k dT/dx) = 0 -> T linear:
#   T(x) = T_hot + (T_cold - T_hot) x / L,   flux Q = k A (T_hot - T_cold) / L.
# The cell-centered FV scheme (interior stencil + half-cell Dirichlet closure)
# reproduces ANY linear field exactly, so the only residual is the linear
# solve. Measured 2026-07-30: profile 7.1e-17, flux 5.0e-16 relative.
# Gate: <= 1e-6 relative (mission band <= 1% subsumed with 4 decades margin;
# 1e-6 leaves ~10 decades over measured for platform/scipy variation).
# ---------------------------------------------------------------------------
def gate_1a(tmp: Path) -> None:
	nx, h = 20, 0.5
	L = nx * h
	t_hot, t_cold = 100.0, 0.0
	lo, hi = x_faces(nx, h)
	rec, _ = run_job(slab_job(nx, h, [
		dict(kind="fixed_t", t_c=t_hot, **lo),
		dict(kind="fixed_t", t_c=t_cold, **hi),
	]), tmp, "slab_linear")
	T = np.load(rec["grid_field"]["npy"]).astype(np.float64)
	x = (np.arange(nx) + 0.5) * h
	exact = t_hot + (t_cold - t_hot) * x / L
	prof_err = float(np.abs(T[:, 2, 2] - exact).max() / (t_hot - t_cold))
	A = (4 * h * 1e-3) ** 2
	q_exact = K_PLA * A * (t_hot - t_cold) / (L * 1e-3)
	q_meas = rec["bc_receipts"][0]["power_w_into_solid"]
	flux_err = abs(q_meas - q_exact) / q_exact
	gate("1a slab linear profile", prof_err <= 1e-6,
	     f"max|T-exact|/dT = {prof_err:.3e} (gate 1e-6; mission 1e-2; measured-band 7e-17)")
	gate("1a slab exact flux Q=kA dT/L", flux_err <= 1e-6,
	     f"Q meas {q_meas:.6e} W vs exact {q_exact:.6e} W, rel err {flux_err:.3e} (gate 1e-6)")
	gate("1a energy balance", rec["energy"]["residual_rel"] <= 1e-9,
	     f"steady net-power residual {rec['energy']['residual_rel']:.3e} (gate 1e-9)")


# ---------------------------------------------------------------------------
# Gate 1b — slab + uniform volumetric source, CONVERGENCE ORDER.
# Derivation: k T'' + q = 0, T(0)=T(L)=0
#   -> T(x) = (q / 2k) x (L - x),  T_max = q L^2 / 8k  (exact quadratic).
# A quadratic has curvature, so the discretization error is finite and must
# shrink ~4x per voxel halving if the scheme is 2nd order. Measured
# 2026-07-30 (max-norm, relative to T_max): nx=8 1.587e-2, nx=16 3.922e-3,
# nx=32 9.776e-4 -> ratios 4.05, 4.01 (order 2.02, 2.00).
# Gate: each halving ratio in [3.3, 4.8] (order 1.72..2.26) + finest <= 1.5e-3.
# ---------------------------------------------------------------------------
def gate_1b(tmp: Path) -> None:
	q_w_m3 = 5.0e4
	L_mm = 8.0
	errs = []
	for nx in (8, 16, 32):
		h = L_mm / nx
		lo, hi = x_faces(nx, h)
		rec, _ = run_job(slab_job(nx, h, [
			dict(kind="fixed_t", t_c=0.0, **lo),
			dict(kind="fixed_t", t_c=0.0, **hi),
		], extra={"sources": [{"q_w_m3": q_w_m3, "box_mm": [[-1, -1, -1], [BIG, BIG, BIG]]}]}),
			tmp, f"slab_quad_{nx}")
		T = np.load(rec["grid_field"]["npy"]).astype(np.float64)
		x = (np.arange(nx) + 0.5) * h * 1e-3
		Lm = L_mm * 1e-3
		exact = q_w_m3 / (2 * K_PLA) * x * (Lm - x)
		errs.append(float(np.abs(T[:, 2, 2] - exact).max() / exact.max()))
	ratios = [errs[i] / errs[i + 1] for i in range(len(errs) - 1)]
	orders = [math.log2(r) for r in ratios]
	print(f"        errors {['%.3e' % e for e in errs]}  ratios {['%.2f' % r for r in ratios]}"
	      f"  orders {['%.2f' % o for o in orders]}")
	gate("1b convergence ORDER (source slab)",
	     all(3.3 <= r <= 4.8 for r in ratios),
	     f"per-halving ratios {['%.2f' % r for r in ratios]} (gate [3.3, 4.8] ~ order 2; "
	     f"measured 4.05/4.01)")
	gate("1b finest-grid accuracy", errs[-1] <= 1.5e-3,
	     f"nx=32 rel err {errs[-1]:.3e} (gate 1.5e-3; measured 9.78e-4)")


# ---------------------------------------------------------------------------
# Gate 2 — radial conduction through a cylinder wall (voxelized annulus).
# Derivation: axisymmetric steady conduction, (1/r) d/dr(r k dT/dr) = 0
#   -> T(r) = T1 + (T2 - T1) ln(r/r1) / ln(r2/r1)
#   -> Q = 2 pi k L (T1 - T2) / ln(r2/r1)   per wall of height L.
# The curved surfaces STAIRCASE on the voxel grid, so the Dirichlet boundary
# lands within +-h/2 of the true radius: convergence is ~O(h), not O(h^2) —
# stated, not hidden. Measured 2026-07-30 (band r1+2h..r2-2h, normalized by
# dT): profile 1.815e-2 (h=0.5) -> 8.426e-3 (h=0.25); flux 2.346e-2 ->
# 1.000e-2. Gate: h=0.25 profile <= 1.3e-2, flux <= 1.5e-2, and both errors
# strictly improve under refinement.
# ---------------------------------------------------------------------------
def gate_2(tmp: Path) -> None:
	r1, r2 = 4.0, 10.0
	t1, t2 = 80.0, 20.0
	nz = 4
	out = {}
	for h in (0.5, 0.25):
		pad = 2
		n = int(round(2 * (r2 + pad * h) / h))
		n += n % 2
		c = n * h / 2.0
		xy = (np.arange(n) + 0.5) * h - c
		X, Y = np.meshgrid(xy, xy, indexing="ij")
		rr = np.sqrt(X ** 2 + Y ** 2)
		rho = np.repeat(((rr >= r1) & (rr <= r2))[:, :, None], nz, axis=2).astype(np.float32)
		npy = tmp / f"annulus_{h}.npy"
		np.save(npy, rho)
		rm = (r1 + r2) / 2.0
		lateral = ["-x", "+x", "-y", "+y"]  # z faces stay adiabatic (pure radial)
		rec, _ = run_job({
			"voxel_mm": h, "npy": str(npy), "material": {"k_w_mk": K_PLA},
			"bcs": [
				# first-claim-wins: inner faces (centers inside the mid-radius box) ...
				{"kind": "fixed_t", "t_c": t1, "faces": lateral,
				 "box_mm": [[c - rm, c - rm, -1], [c + rm, c + rm, BIG]]},
				# ... then every remaining lateral exposed face = outer surface.
				{"kind": "fixed_t", "t_c": t2, "faces": lateral,
				 "box_mm": [[-1, -1, -1], [BIG, BIG, BIG]]},
			]}, tmp, f"annulus_{h}")
		T = np.load(rec["grid_field"]["npy"]).astype(np.float64)[:, :, nz // 2]
		band = (rr >= r1 + 2 * h) & (rr <= r2 - 2 * h)
		exact = t1 + (t2 - t1) * np.log(rr / r1) / math.log(r2 / r1)
		prof = float(np.abs(T[band] - exact[band]).max() / (t1 - t2))
		q_exact = 2 * math.pi * K_PLA * (nz * h * 1e-3) * (t1 - t2) / math.log(r2 / r1)
		q_meas = rec["bc_receipts"][0]["power_w_into_solid"]
		out[h] = (prof, abs(q_meas - q_exact) / q_exact)
		print(f"        h={h}: profile {prof:.3e}, flux {out[h][1]:.3e} "
		      f"(Q {q_meas:.4e} vs {q_exact:.4e} W)")
	gate("2 cylinder ln-profile @ h=0.25", out[0.25][0] <= 1.3e-2,
	     f"rel err {out[0.25][0]:.3e} (gate 1.3e-2; measured 8.43e-3; staircase-limited ~O(h))")
	gate("2 cylinder flux @ h=0.25", out[0.25][1] <= 1.5e-2,
	     f"rel err {out[0.25][1]:.3e} (gate 1.5e-2; measured 1.00e-2)")
	gate("2 cylinder refinement improves", out[0.25][0] < out[0.5][0] and out[0.25][1] < out[0.5][1],
	     f"profile {out[0.5][0]:.3e}->{out[0.25][0]:.3e}, flux {out[0.5][1]:.3e}->{out[0.25][1]:.3e}")


# ---------------------------------------------------------------------------
# Gate 3 — Robin-cooled slab (the Biot problem with a clean closed form).
# Derivation: slab 0..L, T(0)=T_hot fixed, convection h_c to T_inf at x=L.
# 1-D series resistance (per area A): R = L/(kA) + 1/(h_c A)
#   -> Q = (T_hot - T_inf) / R,  T(x) = T_hot - Q x/(k A)  (linear inside),
#      surface film drop T_s - T_inf = Q/(h_c A).
# Biot Bi = h_c L / k = 60 * 0.010 / 0.13 = 4.62 — the film resistance is
# ~18% of the total, so a wrong Robin closure cannot hide. Linear exact
# solution -> FV is exact up to solve tolerance. Measured 2026-07-30:
# profile 5.1e-8, flux-in 1.2e-12, flux-out 5.5e-12 relative.
# Gate: profile <= 1e-5, both fluxes <= 1e-8.
# ---------------------------------------------------------------------------
def gate_3(tmp: Path) -> None:
	nx, h = 20, 0.5
	L_m = nx * h * 1e-3
	t_hot, t_inf, h_c = 90.0, 20.0, 60.0
	lo, hi = x_faces(nx, h)
	rec, _ = run_job(slab_job(nx, h, [
		dict(kind="fixed_t", t_c=t_hot, **lo),
		dict(kind="convection", h_w_m2k=h_c, t_inf_c=t_inf, **hi),
	]), tmp, "robin_slab")
	A = (4 * h * 1e-3) ** 2
	r_tot = L_m / (K_PLA * A) + 1.0 / (h_c * A)
	q_exact = (t_hot - t_inf) / r_tot
	T = np.load(rec["grid_field"]["npy"]).astype(np.float64)
	x = (np.arange(nx) + 0.5) * h * 1e-3
	exact = t_hot - q_exact * x / (K_PLA * A)
	prof = float(np.abs(T[:, 2, 2] - exact).max() / (t_hot - t_inf))
	q_in = rec["bc_receipts"][0]["power_w_into_solid"]
	q_out = -rec["bc_receipts"][1]["power_w_into_solid"]
	bi = h_c * L_m / K_PLA
	gate("3 Robin slab profile (Bi=%.2f)" % bi, prof <= 1e-5,
	     f"rel err {prof:.3e} (gate 1e-5; measured 5.1e-8)")
	gate("3 Robin flux both surfaces", abs(q_in - q_exact) / q_exact <= 1e-8
	     and abs(q_out - q_exact) / q_exact <= 1e-8,
	     f"Q exact {q_exact:.6e} W, in-err {abs(q_in - q_exact) / q_exact:.2e}, "
	     f"out-err {abs(q_out - q_exact) / q_exact:.2e} (gate 1e-8; measured ~1e-12)")


# ---------------------------------------------------------------------------
# Gate 4 — transient: semi-infinite solid step response.
# Derivation: rho cp dT/dt = k T'' on x>0, T(x,0)=T0, T(0,t)=Ts
#   -> T(x,t) = T0 + (Ts - T0) erfc( x / (2 sqrt(alpha t)) ),  alpha = k/(rho cp).
# Domain 40 mm >> penetration sqrt(alpha*10s) ~ 0.76 mm, so the far end is
# provably undisturbed (semi-infinite assumption holds). Backward Euler is
# first-order in dt and unconditionally stable. Measured 2026-07-30
# (max over depths {1,2,4} mm x times {2,5,10} s, normalized by dT):
#   h=0.5  dt=0.1  : 2.895e-2      h=0.25 dt=0.05 : 6.650e-3
# Gate: fine case <= 1.2e-2; coarse->fine refinement shrinks error by > 2x;
# energy-balance residual <= 1e-6 relative (measured ~6e-9); stability probe
# at Fo = alpha dt / h^2 = 2.3 (~14x the 3-D explicit limit 1/6) stays
# bounded and spatially monotone to solver tolerance.
# ---------------------------------------------------------------------------
def gate_4(tmp: Path) -> None:
	from scipy.special import erfc

	rho, cp = 1240.0, 1800.0
	alpha = K_PLA / (rho * cp)
	t0c, tsc = 20.0, 100.0
	times = (2.0, 5.0, 10.0)
	depths = (1.0, 2.0, 4.0)
	mat = {"k_w_mk": K_PLA, "density_kg_m3": rho, "cp_j_kgk": cp}

	def run_case(h: float, dt: float):
		nx = int(round(40.0 / h))
		lo, _hi = x_faces(nx, h)
		rec, _ = run_job({
			"voxel_mm": h, "shape": [nx, 4, 4], "solid": "full", "material": mat,
			"bcs": [dict(kind="fixed_t", t_c=tsc, **lo)],
			"probes_mm": [[x, 2 * h, 2 * h] for x in depths],
			"transient": {"t_initial_c": t0c, "dt_s": dt, "t_end_s": times[-1],
			              "snapshot_times_s": list(times[:-1])},
		}, tmp, f"semi_{h}_{dt}")
		fields = {s["t_s"]: s["npy"] for s in rec["snapshots"]}
		fields[times[-1]] = rec["grid_field"]["npy"]
		errs = []
		for t in times:
			T = np.load(fields[t]).astype(np.float64)
			for xp in depths:
				i = int(xp / h - 0.5)          # nearest cell center at or below xp
				xm = (i + 0.5) * h * 1e-3      # compare AT the cell center (no interp)
				ex = t0c + (tsc - t0c) * erfc(xm / (2 * math.sqrt(alpha * t)))
				errs.append(abs(float(T[i, 2, 2]) - ex) / (tsc - t0c))
		# the receipt's trilinear probes at t_end, vs erfc at the probe point
		perrs = []
		for pr, xp in zip(rec["probes"], depths):
			ex = t0c + (tsc - t0c) * erfc(xp * 1e-3 / (2 * math.sqrt(alpha * times[-1])))
			perrs.append(abs(pr["t_c"] - ex) / (tsc - t0c))
		return max(errs), max(perrs), rec

	e_coarse, _, _ = run_case(0.5, 0.1)
	e_fine, e_probe, rec_f = run_case(0.25, 0.05)
	print(f"        coarse(h=0.5,dt=0.1) {e_coarse:.3e} · fine(h=0.25,dt=0.05) "
	      f"{e_fine:.3e} · trilinear probes {e_probe:.3e}")
	gate("4 erfc response @ h=0.25 dt=0.05", e_fine <= 1.2e-2,
	     f"max err/dT {e_fine:.3e} over depths {depths} mm x times {times} s "
	     f"(gate 1.2e-2; measured 6.65e-3)")
	gate("4 erfc refinement (h,dt)/2 => err/2+", e_fine < e_coarse / 2.0,
	     f"coarse {e_coarse:.3e} -> fine {e_fine:.3e} (ratio {e_coarse / e_fine:.2f}, gate > 2)")
	gate("4 trilinear probe path", e_probe <= 1.2e-2,
	     f"receipt probes at {depths} mm err/dT {e_probe:.3e} (gate 1.2e-2)")
	gate("4 energy balance (mission bound)", rec_f["energy"]["residual_rel"] <= 1e-6,
	     f"|E_in - E_stored|/scale = {rec_f['energy']['residual_rel']:.3e} (gate 1e-6; measured ~6e-9)")

	# Stability probe: ONE implicit step at dt = 10 s on h = 0.5 mm:
	# Fo = alpha dt / h^2 = 5.83e-8 * 10 / 2.5e-7 = 2.33 >> 1/6 (3-D explicit
	# limit) — an explicit step would oscillate/diverge; backward Euler must
	# stay inside [T0, Ts] and be spatially monotone (M-matrix maximum
	# principle), up to linear-solve tolerance (CG rtol 1e-10 -> ~1e-8 slack;
	# gate uses 1e-6).
	nx = 80
	lo, _hi = x_faces(nx, 0.5)
	rec, _ = run_job({
		"voxel_mm": 0.5, "shape": [nx, 4, 4], "solid": "full", "material": mat,
		"bcs": [dict(kind="fixed_t", t_c=tsc, **lo)],
		"transient": {"t_initial_c": t0c, "dt_s": 10.0, "t_end_s": 10.0},
	}, tmp, "stability_probe")
	T = np.load(rec["grid_field"]["npy"]).astype(np.float64)[:, 2, 2]
	fo = alpha * 10.0 / (0.5e-3) ** 2
	tol = 1e-6
	bounded = rec["t_min_c"] >= t0c - tol and rec["t_max_c"] <= tsc + tol
	monotone = bool(np.all(np.diff(T) <= tol))
	gate("4 unconditional stability @ Fo=%.1f" % fo, bounded and monotone,
	     f"T in [{rec['t_min_c']:.6f}, {rec['t_max_c']:.6f}] within [{t0c},{tsc}]±{tol}, "
	     f"monotone={monotone} (explicit limit Fo=1/6 exceeded {fo / (1 / 6):.0f}x)")


# ---------------------------------------------------------------------------
# Gate 5 — negative controls. A gate that cannot fail is not a gate; a solver
# that answers an ill-posed manifest is worse than no solver. Each control
# must exit NONZERO with ok:false and a pointed error. A well-posed control
# run proves exit 0 is reachable (so nonzero exits are informative).
# ---------------------------------------------------------------------------
def gate_5(tmp: Path) -> None:
	nx, h = 8, 1.0
	lo, hi = x_faces(nx, h)

	# positive control first: exit code 0 for a well-posed job
	_rec, code = run_job(slab_job(nx, h, [
		dict(kind="fixed_t", t_c=50.0, **lo),
		dict(kind="fixed_t", t_c=0.0, **hi),
	]), tmp, "neg_positive_control")
	gate("5 positive control exits 0", code == 0, f"returncode {code}")

	def refuses(name: str, job: dict, needle: str) -> None:
		rec, code = run_job(job, tmp, name, expect_ok=False)
		ok = code != 0 and rec.get("ok") is False and needle.lower() in str(rec.get("error", "")).lower()
		gate(f"5 {name} refuses", ok,
		     f"exit {code}, error: {str(rec.get('error'))[:110]!r} (needs substring {needle!r})")

	refuses("no-BC steady", slab_job(nx, h, []), "fixed_t or convection")
	refuses("zero-k material", slab_job(nx, h, [
		dict(kind="fixed_t", t_c=50.0, **lo), dict(kind="fixed_t", t_c=0.0, **hi),
	], extra={"material": {"k_w_mk": 0.0}}), "k_w_mk")
	empty = tmp / "empty.npy"
	np.save(empty, np.zeros((6, 6, 6), np.float32))
	refuses("empty domain", {"voxel_mm": 1.0, "npy": str(empty),
	                         "material": {"k_w_mk": K_PLA},
	                         "bcs": [dict(kind="fixed_t", t_c=1.0,
	                                      box_mm=[[-1, -1, -1], [BIG, BIG, BIG]])]},
	        "zero solid voxels")
	two = tmp / "two_islands.npy"
	rho = np.zeros((12, 4, 4), np.float32)
	rho[:4] = 1.0
	rho[8:] = 1.0
	np.save(two, rho)
	refuses("unanchored component", {
		"voxel_mm": 1.0, "npy": str(two), "material": {"k_w_mk": K_PLA},
		"bcs": [dict(kind="fixed_t", t_c=50.0,
		             box_mm=[[-1, -1, -1], [1e-3, BIG, BIG]], faces=["-x"])]},
	        "singular")


# ---------------------------------------------------------------------------
# Gate 6 — materials-registry integration. The solver's data is part of the
# capability: "PLA" must resolve through tools/materials.py to the researched
# thermal block (k 0.13 / cp 1200, filled 2026-07-30 with cited sources) and
# solve end-to-end. If someone re-nulls the record, this goes red — correctly.
# ---------------------------------------------------------------------------
def gate_6(tmp: Path) -> None:
	nx, h = 10, 1.0
	lo, hi = x_faces(nx, h)
	job = slab_job(nx, h, [
		dict(kind="fixed_t", t_c=50.0, **lo),
		dict(kind="fixed_t", t_c=20.0, **hi),
	], extra={"material": "PLA",
	          "transient": {"t_initial_c": 20.0, "dt_s": 1.0, "t_end_s": 10.0}})
	rec, code = run_job(job, tmp, "registry_pla")
	m = rec.get("material", {})
	ok = (code == 0 and m.get("name") == "PLA" and m.get("k_w_mk") == 0.13
	      and m.get("cp_j_kgk") == 1200.0 and isinstance(m.get("hash"), str)
	      and rec["energy"]["residual_rel"] <= 1e-6)
	gate("6 registry material PLA end-to-end", ok,
	     f"k={m.get('k_w_mk')} cp={m.get('cp_j_kgk')} hash={str(m.get('hash'))[:12]}... "
	     f"energy_res={rec['energy']['residual_rel']:.2e} (record thermal block, "
	     f"researched 2026-07-30)")


def main() -> int:
	print(f"ace_thermal benchmark gates — runner {RUNNER}")
	with tempfile.TemporaryDirectory(prefix="ace_thermal_gates_") as td:
		tmp = Path(td)
		gate_1a(tmp)
		gate_1b(tmp)
		gate_2(tmp)
		gate_3(tmp)
		gate_4(tmp)
		gate_5(tmp)
		gate_6(tmp)
	n_fail = sum(1 for _, ok, _ in results if not ok)
	print(f"\n{'ALL GATES GREEN' if n_fail == 0 else f'{n_fail} GATE(S) RED'} "
	      f"({len(results) - n_fail}/{len(results)} pass)")
	return 0 if n_fail == 0 else 1


if __name__ == "__main__":
	sys.exit(main())
