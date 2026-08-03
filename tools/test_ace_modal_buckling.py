#!/usr/bin/env python3
"""test_ace_modal_buckling.py — benchmark gates for ace_modal / ace_buckling.

A solver is guilty until its benchmark gates are green (DESIGN_GUIDE §25.7).
This standalone suite re-proves, on every run, every claim the two runners
make: closed-form agreement with honest measured bands, convergence under
refinement, the free-free path, linearity pins, negative controls (refusals
that actually fire), and cross-solver consistency of the buckling pre-stress
pass against ace_fea. Exit 0 iff ALL gates hold; exit 1 otherwise.

Run:  python3 tools/test_ace_modal_buckling.py   (any interpreter with numpy/
scipy and the ACE package importable — same requirement as the runners).

Closed forms used (derivations at each gate):
  Euler-Bernoulli beam      f_i = (b_i L)^2 / (2 pi L^2) * sqrt(E I / (rho A))
	cantilever  b_i L = 1.875104, 4.694091, 7.854757   (cos*cosh = -1 roots)
	free-free   b_i L = 4.730041, 7.853205, 10.995608  (cos*cosh = +1 roots)
  Euler fixed-free column   P_cr = pi^2 E I / (4 L^2)
Effective length: an ACE plane clamp at x=0 fixes the whole first ELEMENT
layer (selector band = one voxel), so the flexible span is L - voxel; every
closed form below uses L_eff = L - voxel. Free-free cases have no clamp and
use the full L.

Why the bands are where they are (stated, not hidden): hex8 + lumped mass
over-predict frequencies/critical loads and converge DOWN; the EB closed form
itself omits Timoshenko shear + rotary inertia, which would LOWER the exact
answer by ~ (b_i L)^2 (r/L_eff)^2 (1 + E/(kappa G))/2 (kappa = 5/6, r = h/sqrt(12))
— at L/h = 20 that is ~0.2% (mode 1), ~1.0% (mode 2), ~2.8% (mode 3), so
higher-mode errors can honestly cross zero as the mesh refines while mode 1
stays positive. Every band below brackets a MEASURED value (printed at run
time) with stated margin; bands are never widened to hide a regression.
"""
from __future__ import annotations

import json
import math
import os
import shutil
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
import ace_buckling_runner  # noqa: E402
import ace_modal_runner  # noqa: E402

# PLA-class constants pinned HERE (not read from materials/) so the closed
# forms in this file can never drift silently against a materials edit; the
# materials-registry path itself is exercised by the STL/contract gate below.
E, NU, RHO = 3.3e9, 0.36, 1240.0
G_MOD = E / (2.0 * (1.0 + NU))

CANT_BL = (1.875104, 4.694091, 7.854757)
FREE_BL = (4.730041, 7.853205, 10.995608)
# Cantilever EB effective-modal-mass fractions (Gamma_i^2 / m_total for the
# base-excitation direction; standard closed-form values, e.g. any modal
# survey table): 61.3% / 18.8% / 6.5% for the first three bending modes.
EFF_MASS_EB = (0.6131, 0.1883, 0.0646)

FAILURES: list[str] = []


def gate(name: str, ok: bool, detail: str) -> None:
	print(f"  [{'ok  ' if ok else 'FAIL'}] {name}: {detail}")
	if not ok:
		FAILURES.append(f"{name}: {detail}")


def eb_freq(bl: float, L_eff_mm: float, b_mm: float, h_mm: float) -> float:
	"""Euler-Bernoulli beam frequency (Hz) for weak-axis (thickness h) bending."""
	L, b, h = L_eff_mm * 1e-3, b_mm * 1e-3, h_mm * 1e-3
	I = b * h ** 3 / 12.0
	A = b * h
	return (bl ** 2 / (2.0 * math.pi * L ** 2)) * math.sqrt(E * I / (RHO * A))


def euler_pcr(L_eff_mm: float, b_mm: float, h_mm: float) -> float:
	"""Euler fixed-free critical load (N), weak axis (thickness h)."""
	L, b, h = L_eff_mm * 1e-3, b_mm * 1e-3, h_mm * 1e-3
	I = b * h ** 3 / 12.0
	return math.pi ** 2 * E * I / (4.0 * L ** 2)


def beam_npy(tmp: Path, name: str, L: float, b: float, h: float, vox: float) -> str:
	grid = np.ones((round(L / vox), round(b / vox), round(h / vox)), dtype=np.float32)
	path = tmp / f"{name}.npy"
	np.save(path, grid)
	return str(path)


CLAMP_X0 = [{"kind": "clamped", "region_selector":
	{"type": "plane", "axis": "x", "value_mm": 0.0, "side": "-"}}]
MAT = {"youngs_modulus_pa": E, "poisson": NU, "density_kg_m3": RHO}


def modal_job(tmp: Path, name: str, L: float, b: float, h: float, vox: float,
              **extra) -> dict:
	job = {
		"out_dir": str(tmp / name),
		"voxel_mm": vox,
		"npy": beam_npy(tmp, name, L, b, h, vox),
		"material": dict(MAT),
		"n_modes": 8,
	}
	job.update(extra)
	return job


def z_bending_modes(payload: dict, kin_min: float, eff_min: float) -> list[dict]:
	"""Identify weak-axis (z) bending modes from the participation receipts:
	dominant z kinetic energy AND nonzero net z effective mass (torsion has
	high z kinetic energy for a b/h=2 section — exactly b^2/(b^2+h^2) = 0.8 —
	but ~zero net translation, so the effective-mass cut rejects it)."""
	out = []
	for i, p in enumerate(payload["participation"]):
		if (p["kinetic_fraction"]["z"] > kin_min
				and p["effective_mass_fraction"]["z"] > eff_min):
			out.append({"idx": i, **p})
	return out


def run_json_runner(script: str, job: dict) -> dict:
	"""Invoke a runner as a subprocess and parse its last-line JSON receipt
	(the stdout contract every ace_* runner promises)."""
	with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
		json.dump(job, f)
		job_file = f.name
	try:
		proc = subprocess.run([sys.executable, str(TOOLS / script), job_file],
		                      capture_output=True, text=True, timeout=300)
		lines = [ln for ln in proc.stdout.splitlines() if ln.strip()]
		if not lines:
			return {"ok": False, "error": f"no stdout receipt; stderr: {proc.stderr[-200:]}"}
		return json.loads(lines[-1])
	finally:
		os.unlink(job_file)


def write_box_stl(path: Path, lx: float, ly: float, lz: float) -> None:
	"""Minimal binary STL of an axis-aligned box [0,lx]x[0,ly]x[0,lz] (mm)."""
	v = [(x, y, z) for x in (0.0, lx) for y in (0.0, ly) for z in (0.0, lz)]
	quads = [  # (outward normal axis, 4 corner indices as 2 triangles each)
		((0, 1, 3), (0, 3, 2)), ((4, 6, 7), (4, 7, 5)),  # x = 0, x = lx
		((0, 4, 5), (0, 5, 1)), ((2, 3, 7), (2, 7, 6)),  # y = 0, y = ly
		((0, 2, 6), (0, 6, 4)), ((1, 5, 7), (1, 7, 3)),  # z = 0, z = lz
	]
	tris = [t for pair in quads for t in pair]
	with open(path, "wb") as f:
		f.write(b"\0" * 80)
		f.write(struct.pack("<I", len(tris)))
		for a, b, c in tris:
			pa, pb, pc = (np.array(v[i]) for i in (a, b, c))
			n = np.cross(pb - pa, pc - pa)
			nn = n / max(np.linalg.norm(n), 1e-30)
			f.write(struct.pack("<3f", *nn))
			for p in (pa, pb, pc):
				f.write(struct.pack("<3f", *p))
			f.write(struct.pack("<H", 0))


def main() -> int:  # noqa: PLR0915 — a linear benchmark script reads best linear
	t_suite = time.monotonic()
	tmp = Path(tempfile.mkdtemp(prefix="modal_buckling_gates_"))
	print(f"benchmark scratch: {tmp}")

	# =====================================================================
	# GATE 1 — modal cantilever vs Euler-Bernoulli closed form + convergence
	# Geometry 60 x 6 x 3 mm (L/h = 20, chosen so EB applies: the Timoshenko
	# shear correction is ~0.2/1.0/2.8% for modes 1/2/3 — see module docstring)
	# at voxels 0.75 (h/4) and 0.5 (h/6). Bands bracket the measured errors
	# (printed): hex8+lumped mass over-predicts, converging down; mode 3's
	# band reaches below zero because the omitted shear is ~2.8% there.
	# =====================================================================
	print("== G1: cantilever modal vs Euler-Bernoulli (L/h = 20) ==")
	L, B, H = 60.0, 6.0, 3.0
	errs: dict[float, list[float]] = {}
	payloads: dict[float, dict] = {}
	for vox in (0.75, 0.5):
		t0 = time.monotonic()
		p = ace_modal_runner.run_modal_job(
			modal_job(tmp, f"cant_{vox}", L, B, H, vox, fixtures=CLAMP_X0))
		payloads[vox] = p
		zb = z_bending_modes(p, kin_min=0.5, eff_min=0.01)
		gate(f"G1 vox {vox}: >= 3 z-bending modes identified", len(zb) >= 3,
		     f"found {len(zb)} in {p['n_modes']} modes ({time.monotonic()-t0:.1f}s, "
		     f"n_dof {p['n_dof']})")
		if len(zb) < 3:
			continue
		errs[vox] = []
		for i in range(3):
			ref = eb_freq(CANT_BL[i], L - vox, B, H)
			err = (zb[i]["f_hz"] - ref) / ref
			errs[vox].append(err)
			print(f"      mode {i+1}: {zb[i]['f_hz']:8.2f} Hz vs EB(L_eff) {ref:8.2f} -> {err:+.2%}")
	if 0.75 in errs and 0.5 in errs:
		e_c, e_f = errs[0.75], errs[0.5]
		# Measured 2026-07-30 (re-printed above every run):
		#   coarse +2.58 / +1.31 / -0.56 %,  fine +1.42 / +0.19 / -1.64 %.
		# Bands bracket those with ~+-1.3% margin and keep the SIGN structure:
		# modes 1-2 over-predict (stiff hex8), mode 3 sits below EB because the
		# omitted Timoshenko shear (~2.8% at L/h=20) outweighs the mesh stiffness
		# as the mesh refines — the fine mode-3 band is therefore NEGATIVE.
		bands_c = [(0.012, 0.040), (0.002, 0.028), (-0.020, 0.010)]
		bands_f = [(0.005, 0.028), (-0.010, 0.014), (-0.030, -0.002)]
		for i in range(3):
			gate(f"G1 coarse mode {i+1} in band {bands_c[i]}",
			     bands_c[i][0] <= e_c[i] <= bands_c[i][1], f"measured {e_c[i]:+.2%}")
			gate(f"G1 fine mode {i+1} in band {bands_f[i]}",
			     bands_f[i][0] <= e_f[i] <= bands_f[i][1], f"measured {e_f[i]:+.2%}")
		# convergence toward the closed form: modes 1-2 have a clean EB limit
		# (shear < 1%); mode 3's EB reference is itself ~2.8% off (shear), so a
		# strict monotone gate there would test the WRONG limit — stated, not gated.
		for i in range(2):
			gate(f"G1 mode {i+1} converges toward EB", abs(e_f[i]) < abs(e_c[i]),
			     f"|{e_f[i]:+.2%}| < |{e_c[i]:+.2%}|")

	# =====================================================================
	# GATE 2 — mode-shape artifacts: GridField-compatible layout + physics
	# mode 1 must be root-quiet/tip-loud; mode 2 must have an interior node
	# (EB node of mode 2 sits at 0.774 L). Uses the coarse-cantilever receipts.
	# =====================================================================
	print("== G2: mode-shape .npy artifacts ==")
	p = payloads.get(0.75)
	if p is None:
		gate("G2 mode shapes", False, "no coarse cantilever payload")
	else:
		zb = z_bending_modes(p, 0.5, 0.01)
		f1 = np.load(p["mode_shapes"]["files"][zb[0]["idx"]])
		f2 = np.load(p["mode_shapes"]["files"][zb[1]["idx"]])
		nx = f1.shape[0]
		gate("G2 layout", f1.dtype == np.float32 and f1.flags["C_CONTIGUOUS"]
		     and f1.shape == (80, 8, 4) and abs(float(f1.max()) - 1.0) < 1e-6,
		     f"dtype {f1.dtype}, C {f1.flags['C_CONTIGUOUS']}, shape {f1.shape}, max {f1.max():.6f}")
		root = float(f1[: int(0.15 * nx)].max())
		tip = float(f1[-2:].max())
		gate("G2 mode 1 shape (root quiet, tip loud)", root < 0.1 and tip > 0.9,
		     f"root max {root:.3f}, tip max {tip:.3f}")
		axial_max = f2.max(axis=(1, 2))  # per-x-slice peak magnitude
		interior = axial_max[int(0.55 * nx): int(0.95 * nx)]
		gate("G2 mode 2 interior node (EB: x = 0.774 L)", float(interior.min()) < 0.25,
		     f"interior per-slice peak dips to {interior.min():.3f} of unit max")

	# =====================================================================
	# GATE 3 — participation physics: cantilever EB effective-mass fractions
	# (0.613 / 0.188 / 0.065) and exact lumped-mass conservation rho*h^3*n.
	# =====================================================================
	print("== G3: participation receipts ==")
	if p is not None and len(zb) >= 3:
		bands = [(0.55, 0.67), (0.15, 0.22), (0.04, 0.09)]
		for i in range(3):
			em = zb[i]["effective_mass_fraction"]["z"]
			gate(f"G3 mode {i+1} eff-mass z ~ EB {EFF_MASS_EB[i]:.3f}",
			     bands[i][0] <= em <= bands[i][1], f"measured {em:.4f}")
		vol_mass = RHO * (0.75e-3) ** 3 * 80 * 8 * 4
		gate("G3 lumped mass conserved exactly",
		     abs(p["total_active_mass_kg"] - vol_mass) < 1e-12 * vol_mass,
		     f"{p['total_active_mass_kg']:.6e} vs rho*V {vol_mass:.6e} kg")

	# =====================================================================
	# GATE 4 — cross-pin vs ACE's own reference_modal on the same case: the
	# runner's local eigensolve must reproduce the house solver wherever both
	# apply (same imported K/M assembly, independent eigensolve layer).
	# =====================================================================
	print("== G4: runner eigensolve == ACE reference_modal ==")
	from engine.verify import reference_modal
	rho_grid = np.ones((80, 8, 4), dtype=np.float32)
	t0 = time.monotonic()
	ace = reference_modal(rho_grid, None, 0.75, MAT, CLAMP_X0, n_modes=8)
	mine = payloads[0.75]["frequencies_hz"] if 0.75 in payloads else []
	n_cmp = min(len(mine), len(ace["frequencies_hz"]))
	if n_cmp:
		rel = max(abs(a - b) / b for a, b in
		          zip(mine[:n_cmp], ace["frequencies_hz"][:n_cmp]))
		gate("G4 frequency agreement <= 1e-6 rel", rel <= 1e-6,
		     f"max rel diff {rel:.2e} over {n_cmp} modes ({time.monotonic()-t0:.1f}s)")
	else:
		gate("G4 frequency agreement", False, "no modes to compare")

	# =====================================================================
	# GATE 5 — free-free beam: 6 rigid-body modes at ~0 Hz then elastic modes
	# vs the free-free EB closed form (full L — no clamp layer). Proves the
	# negative-shift eigensolve path. Measured: z1 +2.4% (vox 1.0) / +1.0%
	# (vox 0.75); z2 crosses zero (shear ~1%): bands bracket both.
	# =====================================================================
	print("== G5: free-free beam ==")
	ff_err1 = {}
	for vox in (1.0, 0.75):
		t0 = time.monotonic()
		pf = ace_modal_runner.run_modal_job(
			modal_job(tmp, f"ff_{vox}", L, B, H, vox, free_free=True, n_modes=6))
		rig = pf["rigid_body_modes_hz"]
		f_el1 = pf["frequencies_hz"][0]
		gate(f"G5 vox {vox}: exactly 6 rigid-body modes", len(rig) == 6,
		     f"{len(rig)} rigid ({time.monotonic()-t0:.1f}s, {pf['eigensolve']['path']})")
		gate(f"G5 vox {vox}: rigid modes ~ 0 Hz", max(rig, default=1e9) < 1e-3 * f_el1,
		     f"max rigid {max(rig, default=0):.2e} Hz vs first elastic {f_el1:.1f} Hz")
		# free-free z-bending: kinetic-z > 0.9 keeps bending, drops torsion (0.8)
		zff = [q for q in pf["participation"] if q["kinetic_fraction"]["z"] > 0.9]
		gate(f"G5 vox {vox}: >= 2 free-free z-bending modes", len(zff) >= 2,
		     f"found {len(zff)}")
		# Measured 2026-07-30: vox 1.0 z1 +2.37% z2 +0.68%; vox 0.75 z1 +0.97%
		# z2 -0.64% (shear crossing on z2, as in G1 mode 3). Bands ~+-1.3% margin.
		ff_bands = {1.0: [(0.012, 0.036), (-0.006, 0.020)],
		            0.75: [(0.002, 0.022), (-0.020, 0.008)]}
		if len(zff) >= 2:
			for i, band in enumerate(ff_bands[vox]):
				ref = eb_freq(FREE_BL[i], L, B, H)
				err = (zff[i]["f_hz"] - ref) / ref
				if i == 0:
					ff_err1[vox] = err
				gate(f"G5 vox {vox} z-mode {i+1} in band {band}",
				     band[0] <= err <= band[1],
				     f"{zff[i]['f_hz']:.1f} Hz vs FF-EB {ref:.1f} -> {err:+.2%}")
	if len(ff_err1) == 2:
		gate("G5 free-free mode 1 converges toward EB",
		     abs(ff_err1[0.75]) < abs(ff_err1[1.0]),
		     f"|{ff_err1[0.75]:+.2%}| < |{ff_err1[1.0]:+.2%}|")

	# =====================================================================
	# GATE 6 — modal negative controls: refusals must actually fire.
	# =====================================================================
	print("== G6: modal negative controls ==")
	base = modal_job(tmp, "neg_modal", 30.0, 4.5, 3.0, 0.75)
	try:
		ace_modal_runner.run_modal_job(dict(base))
		gate("G6 no-fixture refusal", False, "solved without fixtures or free_free flag")
	except ValueError as exc:
		gate("G6 no-fixture refusal", "free_free" in str(exc),
		     f"refused, names the explicit opt-out: {str(exc)[:60]}...")
	try:
		bad = dict(base, free_free=True,
		           material={"youngs_modulus_pa": E, "poisson": NU, "density_kg_m3": 0.0})
		ace_modal_runner.run_modal_job(bad)
		gate("G6 zero-density refusal", False, "solved with rho = 0")
	except ValueError as exc:
		gate("G6 zero-density refusal", "density" in str(exc),
		     f"refused: {str(exc)[:60]}...")
	sub = run_json_runner("ace_modal_runner.py", dict(base))
	gate("G6 subprocess contract on refusal", sub.get("ok") is False
	     and "free_free" in sub.get("error", ""),
	     f"ok={sub.get('ok')}, error mentions free_free")

	# =====================================================================
	# GATE 7 — Euler column (fixed-free): P_cr = pi^2 E I / (4 L_eff^2),
	# weak axis; slenderness (L_eff/r ~ 100, r = h/sqrt(12)) keeps it truly
	# elastic. Measured: +6.3% (vox 0.75) -> +3.4% (vox 0.5), and +2.2% at
	# 0.375 in development — observed convergence order ~1.5 (between the
	# O(h) surface-stairstep and O(h^2) element limits, as expected for a
	# voxelated boundary). Bands bracket measurement; over-prediction only.
	# =====================================================================
	print("== G7: Euler column buckling + convergence order ==")
	Lc, Bc, Hc = 45.0, 4.5, 3.0
	buck_err = {}
	for vox in (0.75, 0.5):
		t0 = time.monotonic()
		job = {
			"out_dir": str(tmp / f"buck_{vox}"),
			"voxel_mm": vox,
			"npy": beam_npy(tmp, f"col_{vox}", Lc, Bc, Hc, vox),
			"material": dict(MAT),
			"fixtures": CLAMP_X0,
			"loads": [{"kind": "point", "magnitude": 10.0, "direction": [-1, 0, 0],
				"region_selector": {"type": "plane", "axis": "x", "value_mm": Lc, "side": "+"}}],
			"n_modes": 2,
		}
		pb = ace_buckling_runner.run_buckling_job(job)
		ref = euler_pcr(Lc - vox, Bc, Hc)
		err = (pb["critical_load_N"] - ref) / ref
		buck_err[vox] = err
		print(f"      vox {vox}: Pcr {pb['critical_load_N']:.3f} N vs Euler(L_eff) "
		      f"{ref:.3f} -> {err:+.2%} ({time.monotonic()-t0:.1f}s, n_dof {pb['n_dof']})")
		# measured 2026-07-30: +6.31% (0.75), +3.36% (0.5); bands ~+-2% margin,
		# over-prediction only (linear buckling on a stiff mesh is an upper bound)
		band = (0.045, 0.085) if vox == 0.75 else (0.018, 0.052)
		gate(f"G7 vox {vox} in band {band}", band[0] <= err <= band[1],
		     f"measured {err:+.2%}")
		if vox == 0.75:
			gate("G7 caveat + knockdown present",
			     pb["caveat"].startswith("LINEAR") and
			     pb["knockdown"]["recommended_factor"] == 0.5 and
			     len(pb["knockdown"]["sources"]) == 3,
			     f"design load {pb['knockdown']['design_critical_load_n']:.1f} N "
			     f"= 0.5 x {pb['critical_load_N']:.1f} N")
	if len(buck_err) == 2:
		gate("G7 converges toward Euler", buck_err[0.5] < buck_err[0.75],
		     f"{buck_err[0.5]:+.2%} < {buck_err[0.75]:+.2%}")
		order = math.log(buck_err[0.75] / buck_err[0.5]) / math.log(0.75 / 0.5)
		gate("G7 observed order in [0.8, 2.5]", 0.8 <= order <= 2.5,
		     f"p = {order:.2f} (dev 3-point check: 1.56 / 1.42)")

	# =====================================================================
	# GATE 8 — buckling linearity pins: lambda = eig(K, -K_g(sigma(u))) with
	# u = K^-1 F gives sigma independent of E at fixed F scaled by... work it:
	# 2E => K x2, u x1/2, sigma = D(2E) B u/2 unchanged => K_g unchanged =>
	# lambda x2 exactly. 2F => sigma x2 => K_g x2 => lambda x1/2 exactly
	# (critical load lambda*P invariant). Measured residual ~1e-10.
	# =====================================================================
	print("== G8: buckling linearity pins ==")
	def small_col(e_pa: float, p_n: float, name: str) -> dict:
		return ace_buckling_runner.run_buckling_job({
			"out_dir": str(tmp / name),
			"voxel_mm": 0.75,
			"npy": beam_npy(tmp, name, 24.0, 3.0, 3.0, 0.75),
			"material": {"youngs_modulus_pa": e_pa, "poisson": NU, "density_kg_m3": RHO},
			"fixtures": CLAMP_X0,
			"loads": [{"kind": "point", "magnitude": p_n, "direction": [-1, 0, 0],
				"region_selector": {"type": "plane", "axis": "x", "value_mm": 24.0, "side": "+"}}],
			"n_modes": 1,
		})
	lam0 = small_col(E, 1.0, "lin0")["buckling_load_factor"]
	lam_2e = small_col(2 * E, 1.0, "lin2E")["buckling_load_factor"]
	lam_2p = small_col(E, 2.0, "lin2P")["buckling_load_factor"]
	gate("G8 doubling E doubles lambda", abs(lam_2e / lam0 - 2.0) <= 1e-6,
	     f"ratio {lam_2e / lam0:.9f}")
	gate("G8 doubling load halves lambda", abs(lam_2p / lam0 - 0.5) <= 1e-6,
	     f"ratio {lam_2p / lam0:.9f}")
	gate("G8 critical load invariant under load scaling",
	     abs(2.0 * lam_2p - lam0) <= 1e-6 * lam0,
	     f"lambda*P: {2.0 * lam_2p:.6f} vs {lam0:.6f}")

	# =====================================================================
	# GATE 9 — buckling negative control: zero load must refuse loudly.
	# =====================================================================
	print("== G9: buckling negative controls ==")
	try:
		small_col(E, 0.0, "neg_zero_load")
		gate("G9 zero-load refusal", False, "returned a factor for zero load")
	except ValueError as exc:
		gate("G9 zero-load refusal", "load" in str(exc).lower(),
		     f"refused: {str(exc)[:60]}...")
	zjob = {
		"out_dir": str(tmp / "neg_zero_load_sub"),
		"voxel_mm": 0.75,
		"npy": beam_npy(tmp, "neg_col", 24.0, 3.0, 3.0, 0.75),
		"material": dict(MAT), "fixtures": CLAMP_X0, "loads": [], "n_modes": 1,
	}
	sub = run_json_runner("ace_buckling_runner.py", zjob)
	gate("G9 subprocess contract on refusal", sub.get("ok") is False
	     and "load" in sub.get("error", "").lower(),
	     f"ok={sub.get('ok')}, error mentions load")

	# =====================================================================
	# GATE 10 — cross-solver consistency: the buckling pre-stress pass vs
	# ace_fea_runner on the SAME manifest. Both invoke engine.verify.
	# reference_fea with identical inputs and solver settings (consistency by
	# construction), and this gate re-proves it at the RUNNER level: fields
	# and scalars must agree to solver determinism.
	# =====================================================================
	print("== G10: buckling prestress == ace_fea on the same manifest ==")
	shared_npy = beam_npy(tmp, "xcheck", 30.0, 4.5, 3.0, 0.75)
	shared = {
		"voxel_mm": 0.75, "npy": shared_npy, "material": dict(MAT),
		"fixtures": CLAMP_X0,
		"loads": [{"kind": "point", "magnitude": 10.0, "direction": [-1, 0, 0],
			"region_selector": {"type": "plane", "axis": "x", "value_mm": 30.0, "side": "+"}}],
	}
	fea = run_json_runner("ace_fea_runner.py", dict(shared, out_dir=str(tmp / "x_fea")))
	buck = ace_buckling_runner.run_buckling_job(dict(shared, out_dir=str(tmp / "x_buck"), n_modes=1))
	if not fea.get("ok"):
		gate("G10 ace_fea run", False, f"ace_fea failed: {fea.get('error')}")
	else:
		d_fea = np.load(fea["disp_field_npy"])
		d_pre = np.load(buck["prestress"]["disp_field_npy"])
		denom = max(float(np.abs(d_fea).max()), 1e-300)
		rel = float(np.abs(d_fea - d_pre).max()) / denom
		gate("G10 displacement fields agree <= 1e-8 rel", rel <= 1e-8,
		     f"max rel diff {rel:.2e} (same reference_fea, same Jacobi-CG settings)")
		tip_rel = abs(buck["prestress"]["tip_displacement_m"] - fea["tip_displacement_m"]) \
			/ max(abs(fea["tip_displacement_m"]), 1e-300)
		vm_rel = abs(buck["prestress"]["max_von_mises_pa"] - fea["max_von_mises_pa"]) \
			/ max(abs(fea["max_von_mises_pa"]), 1e-300)
		gate("G10 tip displacement + max vM agree <= 1e-8 rel",
		     tip_rel <= 1e-8 and vm_rel <= 1e-8,
		     f"tip rel {tip_rel:.2e}, vM rel {vm_rel:.2e}")

	# =====================================================================
	# GATE 11 — STL route + materials-registry key + full runner contract:
	# a box STL parity-filled by tools/voxelize_stl.py must give EXACTLY the
	# frequencies of the equivalent all-ones grid (identical occupancy), with
	# the full receipt (participation, mode shapes on disk, envelope).
	# =====================================================================
	print("== G11: STL route + contract ==")
	stl_path = tmp / "box.stl"
	write_box_stl(stl_path, 12.0, 4.5, 3.0)
	stl_job = {
		"out_dir": str(tmp / "stl_modal"), "voxel_mm": 0.75,
		"stl": str(stl_path), "shape": [16, 6, 4],
		"material": "PLA",  # exercises tools/materials/pla.json resolution
		"fixtures": CLAMP_X0, "n_modes": 3,
	}
	sub = run_json_runner("ace_modal_runner.py", stl_job)
	if not sub.get("ok"):
		gate("G11 STL modal run", False, f"failed: {sub.get('error')}")
	else:
		import materials
		pla = materials.get("PLA").fea_material()
		ref = ace_modal_runner.run_modal_job({
			"out_dir": str(tmp / "stl_ref"), "voxel_mm": 0.75,
			"npy": beam_npy(tmp, "stl_ref_grid", 12.0, 4.5, 3.0, 0.75),
			"material": pla, "fixtures": CLAMP_X0, "n_modes": 3,
		})
		rel = max(abs(a - b) / b for a, b in
		          zip(sub["frequencies_hz"], ref["frequencies_hz"]))
		gate("G11 STL == ones-grid frequencies", rel <= 1e-9,
		     f"max rel diff {rel:.2e} (identical occupancy by parity fill)")
		shapes_exist = all(os.path.exists(f) for f in sub["mode_shapes"]["files"])
		gate("G11 receipt contract complete",
		     shapes_exist and len(sub["participation"]) == 3
		     and "analysis_envelope" in sub and sub["boundary"] == "fixed",
		     f"{len(sub['mode_shapes']['files'])} mode shapes on disk, "
		     f"participation + envelope present")

	# =====================================================================
	dt = time.monotonic() - t_suite
	print(f"\n{len(FAILURES)} failure(s); suite wall time {dt:.1f}s")
	if FAILURES:
		for f in FAILURES:
			print(f"  FAILED: {f}")
		print(f"SUITE: FAIL (scratch kept for inspection: {tmp})")
		return 1
	shutil.rmtree(tmp, ignore_errors=True)  # receipts are the printout; no scratch left behind
	print("SUITE: PASS — every claim above was re-measured this run.")
	return 0


if __name__ == "__main__":
	sys.exit(main())
