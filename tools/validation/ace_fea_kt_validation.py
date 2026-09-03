#!/usr/bin/env python3
"""Physics validation pin: how wrong is our voxel hex8 FEA at a STRESS CONCENTRATION?

Every other ACE FEA pin (ace_fea_validation, ...) tests SMOOTH geometry — a
cantilever, a plate, a column — where the hex8 grid has no curved boundary to
staircase. Those pins are green. But peak stress in a real part lives at a
FILLET, and our only meshing path is voxel-derived binary hex8
(voxelize_stl -> ace_fea_runner, OR sample_part with rho>=0.5 occupancy — BOTH
binarize, so both staircase a curved surface into cubic steps). This pin
MEASURES the error there against a case with a PUBLISHED theoretical Kt.

SPECIMEN — shouldered (stepped) round bar in AXIAL TENSION (the classic
Kt benchmark; tension, not bending, so the ~20% hex8 bending under-read of
ace_fea_validation does NOT confound the fillet measurement):
    small dia d = 16 mm, large dia D = 24 mm  (D/d = 1.5)
    shoulder fillet r = 2.4 mm                (r/d = 0.15, h/r = 1.667)
    axial load P = 1000 N on the small end, large end clamped
    sigma_nom = P / (pi d^2/4)  at the small section
    peak_theory = Kt * sigma_nom

THEORETICAL Kt — Peterson's / Pilkey shoulder-fillet-in-tension fit
    Kt = C1 + C2(2h/D) + C3(2h/D)^2 + C4(2h/D)^3,  h = (D-d)/2,
    C1..C4(h/r) for 0.1 <= h/r <= 2.0 (below).  For d=16,D=24,r=2.4:
    h/r = 1.667  ->  Kt = 1.667.
    Source: W. D. Pilkey, "Peterson's Stress Concentration Factors" (chart
    3.4, stepped bar of circular cross section with a shoulder fillet, axial
    tension); coefficient fit as published e.g. in Pilkey, "Formulas for
    Stress, Strain, and Structural Matrices," 2nd ed., Wiley 2005, and
    reproduced by the AMESWeb shaft-shoulder-fillet SCF calculator.

WHAT WE MEASURE (voxelize the specimen THE WAY WE ACTUALLY DO, at
r/2, r/3, r/4, r/6, r/8 voxels across the fillet radius): the peak von Mises
in a z-band around the fillet, divided by the analytic sigma_nom, gives a
measured Kt to compare against 1.667.

FINDING (pinned, receipts below): the far-field nominal stress is dead-on
(<1%), but the fillet PEAK does NOT converge to the theoretical Kt. It is
dominated by the sharp re-entrant corners of the STAIRCASED fillet surface,
which is a mesh artifact, not the real fillet stress. Measured error scatters
-6% (very coarse, blunted) to +44% (r/3), then plateaus / grows to +20..29%
under refinement (r/6..r/10) instead of approaching 0. NO tested voxel
resolution reliably lands within 10% of the true Kt, and refining the mesh
does not fix it. So a peak/fillet stress our FEA reports is trustworthy only
to roughly +/-20-30%, biased HIGH by staircasing (the conservative direction
for a strength check, but wrong, and non-convergent).

Run:  ~/miniconda3/bin/python tools/ace_fea_kt_validation.py
Exit: 0 iff the characterization holds (geometry valid+watertight, nominal
      trustworthy, fillet peak provably non-convergent to Kt); nonzero
      otherwise. Deterministic: fixed geometry, fixed solver rtol, no clock.
"""
import json
import math
import os
import subprocess
import sys

import numpy as np

ROOT = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(ROOT, "..", ".."))  # tools/validation/<this> -> repo root
sys.path.insert(0, os.path.join(REPO, "tools"))
import _layout  # noqa: E402
ENGINE = os.path.join(REPO, "target", "release", "kernel-api")
PY = os.environ.get("ACE_PYTHON", os.path.expanduser("~/miniconda3/bin/python"))
OUT = os.path.join(REPO, "engine_out", "kt_validation")
os.makedirs(OUT, exist_ok=True)

# ---------------- specimen (mm) ----------------
d, D, r = 16.0, 24.0, 2.4          # small dia, large dia, shoulder fillet radius
L_large, L_small = 20.0, 22.0      # cylinder lengths (Saint-Venant clearance each end)
z_sh = L_large                     # shoulder plane z
z_tan = z_sh + r                   # fillet -> small-cylinder tangent z
z_top = z_tan + L_small            # loaded end z
P = 1000.0                         # axial load (N); linear-elastic, absolute value irrelevant
E, NU = 200e9, 0.30                # steel; Kt is defined near nu~0.3
A_small = math.pi * (d / 2.0) ** 2                 # mm^2
SIG_NOM = P / (A_small * 1e-6)                      # Pa  (= P / A, analytic nominal)

# ---------------- theoretical Kt (Peterson/Pilkey, axial tension) ----------------
def peterson_kt_tension(D, d, r):
	"""Kt for a stepped round bar with a shoulder fillet in axial tension.
	Pilkey / Peterson's SCF fit; coefficients for 0.1 <= h/r <= 2.0 and
	2.0 <= h/r <= 20.0.  h = (D-d)/2.  Returns (Kt, h/r)."""
	h = (D - d) / 2.0
	hr = h / r
	s = math.sqrt(hr)
	t2D = 2.0 * h / D
	if hr <= 2.0:                                  # 0.1 <= h/r <= 2.0
		C1 = 0.926 + 1.157 * s - 0.099 * hr
		C2 = 0.012 - 3.036 * s + 0.961 * hr
		C3 = -0.302 + 3.977 * s - 1.744 * hr
		C4 = 0.365 - 2.098 * s + 0.878 * hr
	else:                                          # 2.0 <= h/r <= 20.0
		C1 = 1.200 + 0.860 * s - 0.022 * hr
		C2 = -1.805 - 0.346 * s - 0.038 * hr
		C3 = 2.198 - 0.486 * s + 0.165 * hr
		C4 = -0.593 - 0.028 * s - 0.106 * hr
	return C1 + C2 * t2D + C3 * t2D ** 2 + C4 * t2D ** 3, hr


KT, HR = peterson_kt_tension(D, d, r)
PEAK_THEORY = KT * SIG_NOM


# ---------------- build the specimen (B-rep revolve) ----------------
def arc(cx, cz, rad, a0, a1, n):
	return [[cx + rad * math.cos(math.radians(a)), cz + rad * math.sin(math.radians(a))]
	        for a in np.linspace(a0, a1, n)]


def build_specimen():
	"""Revolve a (radius, z) profile about Z: two coaxial cylinders joined by an
	exact 90-deg fillet arc tangent to the shoulder plane and the small wall.
	STL faceted FINER than the finest voxel so voxelization staircasing — not
	facet coarseness — is the only discretization of the curved surface."""
	prof = ([[0.0, 0.0], [D / 2, 0.0], [D / 2, z_sh], [d / 2 + r, z_sh]]
	        + arc(d / 2 + r, z_sh + r, r, 270.0, 180.0, 32)      # concave fillet
	        + [[d / 2, z_top], [0.0, z_top]])
	prof = [[round(p[0], 5), round(p[1], 5)] for p in prof]
	program = {"ops": [
		{"id": "shaft", "op": "revolve", "profile": prof, "segments": 360},
		{"id": "v", "op": "validate", "in": "shaft"},
		{"id": "m", "op": "mass_properties", "in": "shaft", "density": 1.0},
		{"id": "x", "op": "export_stl", "in": "shaft", "file": "shaft.stl"}]}
	json.dump(program, open(os.path.join(OUT, "kt_program.json"), "w"), indent=1)
	rr = subprocess.run([ENGINE, "run", os.path.join(OUT, "kt_program.json"), "--out-dir", OUT],
	                    capture_output=True, text=True)
	rep = json.loads(rr.stdout)
	by = {o["id"]: o for o in rep["ops"]}
	bad = [o for o in rep["ops"] if o.get("error")]
	assert not bad, f"specimen build FAILED: {json.dumps(bad)[:800]}"
	return by["v"]["measures"], by["x"]["measures"]


# ---------------- voxelize + FEA at one resolution ----------------
def grid_for(N):
	vox = r / N
	margin = 2.0 * vox
	ox = oy = -D / 2.0 - margin
	oz = -margin
	nx = int(math.ceil((D + 2 * margin) / vox))
	nz = int(math.ceil((z_top + 2 * margin) / vox))
	return vox, [ox, oy, oz], [nx, nx, nz]


def solve(N, keep_field=None):
	"""voxelize_stl.py -> ace_fea_runner.py at N voxels across the fillet radius.
	Returns a dict of measured quantities plus the VERBATIM ace_fea payload."""
	vox, origin, shape = grid_for(N)
	rho_npy = os.path.join(OUT, "_rho.npy")
	vjob = {"stl": os.path.join(OUT, "shaft.stl"), "origin_mm": origin,
	        "voxel_mm": vox, "shape": shape, "out": rho_npy}
	json.dump(vjob, open(os.path.join(OUT, "_vjob.json"), "w"))
	rv = subprocess.run([PY, str(_layout.find_tool("voxelize_stl.py")), os.path.join(OUT, "_vjob.json")],
	                    capture_output=True, text=True)
	vrec = json.loads([l for l in rv.stdout.splitlines() if l.strip()][-1])

	fea_dir = os.path.join(OUT, "_fea")
	fjob = {"out_dir": fea_dir, "voxel_mm": vox, "origin_mm": origin, "shape": shape, "npy": rho_npy,
	        "material": {"youngs_modulus_pa": E, "poisson": NU, "density_kg_m3": 7850.0},
	        "fixtures": [{"kind": "clamped",
	                      "region_selector": {"type": "plane", "axis": "z", "value_mm": vox, "side": "-"}}],
	        "loads": [{"kind": "point", "magnitude": P, "direction": [0, 0, 1],
	                   "region_selector": {"type": "plane", "axis": "z", "value_mm": z_top - vox, "side": "+"}}],
	        "direct_solver_max_dof": 0}
	json.dump(fjob, open(os.path.join(OUT, "_fjob.json"), "w"))
	rf = subprocess.run([PY, str(_layout.find_tool("ace_fea_runner.py")), os.path.join(OUT, "_fjob.json")],
	                    capture_output=True, text=True)
	fea = json.loads([l for l in rf.stdout.splitlines() if l.strip()][-1])
	assert fea.get("ok"), f"N={N}: ace_fea FAILED: {json.dumps(fea)[:400]}\n{rf.stderr[-500:]}"

	stress = np.load(os.path.join(fea_dir, "stress_field.npy"))
	ox, oy, oz = origin
	# peak von Mises in a z-band that BRACKETS the fillet (excludes the clamped
	# end and the loaded end so a boundary artifact can't masquerade as the peak)
	klo = max(0, int((z_sh - r - oz) / vox))
	khi = int((z_sh + 2 * r - oz) / vox)
	fmax = float(stress[:, :, klo:khi].max())
	# far-field nominal: median von Mises over the solid voxels of a mid-slice of
	# the small section (uniaxial tension => von Mises == axial stress == sigma_nom)
	kmid = int((z_tan + 0.5 * L_small - oz) / vox)
	sl = stress[:, :, kmid]
	nom = float(np.median(sl[sl > 0]))
	# where is the GLOBAL max? (transparency: confirm it sits on the fillet)
	gi = np.unravel_index(int(np.argmax(stress)), stress.shape)
	gz = oz + (gi[2] + 0.5) * vox
	if keep_field is not None:                       # persist one field for the sheet
		np.save(keep_field, stress)
	return {
		"N": N, "vox_mm": vox, "grid": shape, "solid_voxels": vrec["solid_voxels"],
		"n_dof": fea["n_dof"], "n_active": fea["n_active_elements"], "fea_s": fea["timings_s"]["fea_s"],
		"peak_pa": fmax, "nominal_pa": nom, "global_max_pa": float(stress.max()), "global_max_z_mm": gz,
		"peak_at_fillet": bool(z_sh - r < gz < z_sh + 2 * r),
		"Kt_meas": fmax / SIG_NOM, "err": (fmax - PEAK_THEORY) / PEAK_THEORY,
		"nom_err": (nom - SIG_NOM) / SIG_NOM,
		"origin_mm": origin, "fea_payload_verbatim": fea,
	}


def render_analysis_sheet():
	"""One analysis sheet from receipts.json + the saved r/6 von Mises field:
	load case + fillet stress field + the (non-)convergence curve. Best-effort —
	a matplotlib hiccup must never sink the validation verdict."""
	rec = json.load(open(os.path.join(OUT, "receipts.json")))
	res = rec["resolutions"]
	kt = rec["theoretical_Kt"]
	_, origin6, _ = grid_for(6)
	peak6 = next(x["peak_pa"] for x in res if x["N"] == 6) / 1e6
	job = {
		"title": "ACE hex8 FEA at a fillet (Kt validation)",
		"meta_note": f"stepped shaft, axial tension - voxelize -> ace_fea - "
		f"Peterson Kt={kt:.3f} - D/d={D/d:.2f}, r/d={r/d:.3f}",
		"date": "kt validation", "out": os.path.join(OUT, "kt_validation_sheet.png"),
		"panels": [
			{"kind": "view", "caption": "load case - P axial on small end, large end clamped",
			 "stl": os.path.join(OUT, "shaft.stl"), "elev": 6.0, "azim": -72.0,
			 "loads": [{"label": f"P = {P:.0f} N", "at": [0.0, 0.0, z_top], "dir": [0, 0, 1]}],
			 "fixture": {"label": "clamped (large end)"}},
			{"kind": "field", "caption": "von Mises - r/6 voxel (staircased fillet ring reads HIGH)",
			 "stl": os.path.join(OUT, "shaft.stl"), "npy": os.path.join(OUT, "stress_field.npy"),
			 "origin_mm": origin6, "voxel_mm": r / 6, "cmap": "turbo", "unit": "MPa",
			 "scale": 1e-6, "vmax": peak6, "hotspot": True, "elev": 6.0, "azim": -72.0},
			{"kind": "curve", "caption": "measured Kt vs voxels across fillet - NO convergence to Kt",
			 "series": [{"x": [x["N"] for x in res], "y": [x["Kt_meas"] for x in res],
			             "label": "measured Kt (peak/sigma_nom)"}],
			 "xlabel": "voxels across fillet radius  (r / voxel)", "ylabel": "K t",
			 "targets": [{"y": kt, "label": f"Peterson Kt = {kt:.3f}"},
			             {"y": 1.10 * kt, "label": "+10%"}, {"y": 0.90 * kt, "label": "-10%"}],
			 "ylim": [1.3, 2.6]},
		],
		"results": [
			["theoretical Kt (tension)", f"{kt:.3f}"],
			["sigma_nom = P/(pi d^2/4)", f"{SIG_NOM/1e6:.3f} MPa"],
			["measured Kt  r/2 .. r/8", " -> ".join(f"{x['Kt_meas']:.2f}" for x in res)],
			["peak error  r/2 .. r/8", " -> ".join(f"{x['err']*100:+.0f}%" for x in res)],
			["far-field nominal error (max)", f"{rec['analysis']['nominal_max_abs_err']*100:.2f}%"],
			["resolution for reliable <10%", "NONE (non-convergent)"
			 if rec["analysis"]["resolution_for_lt10pct"] is None
			 else f"r/{rec['analysis']['resolution_for_lt10pct']}"],
			["Kt source", "Peterson's SCF (Pilkey), stepped-bar shoulder fillet, tension"],
		],
		"gates": rec["gates"],
	}
	jp = os.path.join(OUT, "kt_sheet_job.json")
	json.dump(job, open(jp, "w"), indent=1)
	rs = subprocess.run([PY, str(_layout.find_tool("analysis_sheet.py")), jp],
	                    capture_output=True, text=True)
	last = [l for l in rs.stdout.splitlines() if l.strip()]
	print(f"analysis sheet: {json.loads(last[-1])['out'] if last else '(failed) ' + rs.stderr[-300:]}")


def main():
	print("=" * 78)
	print("SPECIMEN  shouldered round bar, AXIAL TENSION")
	print(f"  d={d} mm  D={D} mm  (D/d={D/d:.2f})   fillet r={r} mm  (r/d={r/d:.3f}, h/r={HR:.3f})")
	print(f"  P={P:.0f} N   sigma_nom = P/(pi d^2/4) = {SIG_NOM/1e6:.4f} MPa")
	print(f"THEORETICAL Kt (Peterson/Pilkey, tension) = {KT:.4f}"
	      f"   =>  peak_theory = Kt*sigma_nom = {PEAK_THEORY/1e6:.4f} MPa")
	print(f"  source: Pilkey, Peterson's Stress Concentration Factors (shoulder-fillet-"
	      f"in-tension chart 3.4); C1..C4(h/r) polynomial fit.")

	vmeas, xmeas = build_specimen()
	geom_ok = bool(vmeas["geometric_ok"] and vmeas["valid"] and vmeas["closed"] and vmeas["manifold"])
	watertight = bool(xmeas["watertight"])
	print(f"\nGEOMETRY  valid={vmeas['valid']} geometric_ok={vmeas['geometric_ok']} "
	      f"manifold={vmeas['manifold']} closed={vmeas['closed']} watertight={watertight} "
	      f"tris={xmeas['triangles']}")

	# resolution ladder: N voxels across the fillet radius (vox = r/N)
	ladder = [2, 3, 4, 6, 8]
	sheet_field = os.path.join(OUT, "stress_field.npy")
	rows = []
	print(f"\n{'N=r/vox':>8} {'vox':>6} {'vox/r':>6} {'grid':>15} {'ndof':>9} {'t_s':>6} "
	      f"{'peakMPa':>8} {'nomMPa':>7} {'Kt_meas':>8} {'err%':>7} {'@fillet':>8}")
	for N in ladder:
		R = solve(N, keep_field=sheet_field if N == 6 else None)
		rows.append(R)
		print(f"{N:>8} {R['vox_mm']:>6.3f} {'r/%d'%N:>6} {str(R['grid']):>15} {R['n_dof']:>9} "
		      f"{R['fea_s']:>6.1f} {R['peak_pa']/1e6:>8.3f} {R['nominal_pa']/1e6:>7.3f} "
		      f"{R['Kt_meas']:>8.3f} {R['err']*100:>+7.1f} {str(R['peak_at_fillet']):>8}")

	# determinism: re-solve one resolution, require an identical peak
	R2 = solve(4)
	base = next(x for x in rows if x["N"] == 4)
	det_rel = abs(R2["peak_pa"] - base["peak_pa"]) / base["peak_pa"]
	print(f"\nDETERMINISM  N=4 re-run peak {R2['peak_pa']/1e6:.6f} vs {base['peak_pa']/1e6:.6f} MPa"
	      f"  (rel delta {det_rel:.2e})")

	# ---- analysis ----
	errs = {x["N"]: x["err"] for x in rows}
	nom_errs = [abs(x["nom_err"]) for x in rows]
	finest = max(ladder)
	err_fine = errs[finest]
	# "resolution needed for <10%": scan the ladder for a resolution that lands within
	# 10% AND stays within 10% for all finer resolutions (a real convergence would).
	def converged_from(n):
		return all(abs(errs[k]) < 0.10 for k in ladder if k >= n)
	need = next((n for n in ladder if converged_from(n)), None)
	coarse_typ = [x for x in rows if x["N"] in (2, 3, 4)]           # how we actually mesh a fillet
	coarse_lo = min(x["err"] for x in coarse_typ)
	coarse_hi = max(x["err"] for x in coarse_typ)

	print("\n" + "-" * 78)
	print("ANALYSIS")
	print(f"  far-field NOMINAL stress: max |error| vs analytic P/A = {max(nom_errs)*100:.2f}%  "
	      f"-> the FEA gets the NOMINAL right; the error is entirely in the PEAK.")
	print(f"  fillet PEAK vs Kt*sigma_nom: coarse/typical (r/2..r/4) error spans "
	      f"{coarse_lo*100:+.1f}%..{coarse_hi*100:+.1f}%  (sign not even fixed).")
	print(f"  under refinement (r/6, r/8) error is {errs[6]*100:+.1f}%, {errs[8]*100:+.1f}% — it does "
	      f"NOT approach 0; it plateaus/grows HIGH (staircase re-entrant-corner over-read).")
	if need is None:
		print("  resolution needed for a RELIABLE <10% peak: NONE in r/2..r/8 — voxel hex8 does "
		      "not converge to the fillet Kt at any tested (or usable) resolution.")
	else:
		print(f"  resolution needed for <10% peak (converged): r/{need}.")

	# ---- honest gate (characterization pin: PASS = reproduced the known error) ----
	non_monotone = not all(abs(errs[ladder[i + 1]]) <= abs(errs[ladder[i]]) for i in range(len(ladder) - 1))
	min_fine_err = min(abs(errs[k]) for k in ladder if k >= 3)     # excl. lucky coarse N=2
	gates = {
		"geometry_valid_watertight": geom_ok and watertight,
		"peak_is_at_the_fillet": all(x["peak_at_fillet"] for x in rows),
		"nominal_trustworthy_lt_3pct": max(nom_errs) < 0.03,
		"deterministic": det_rel < 1e-6,
		"fillet_peak_NOT_converged_gt_10pct": abs(err_fine) > 0.10,
		"refinement_does_not_fix_it": min_fine_err > 0.10 and non_monotone,
	}
	print("\nGATES")
	for k, v in gates.items():
		print(f"  [{'PASS' if v else 'FAIL'}] {k}")

	receipts = {
		"specimen": {"d_mm": d, "D_mm": D, "fillet_r_mm": r, "Dd": D / d, "rd": r / d, "hr": HR,
		             "load_P_N": P, "sigma_nom_MPa": SIG_NOM / 1e6, "material": {"E_pa": E, "nu": NU}},
		"theoretical_Kt": KT, "Kt_source": "Pilkey, Peterson's Stress Concentration Factors, "
		"chart 3.4 (stepped circular bar, shoulder fillet, axial tension); C1..C4(h/r) fit "
		"(also Pilkey, Formulas for Stress/Strain/Structural Matrices 2e, Wiley 2005).",
		"geometry": {"validate": vmeas, "export": xmeas},
		"resolutions": [{k: x[k] for k in ("N", "vox_mm", "grid", "n_dof", "solid_voxels", "fea_s",
		                                    "peak_pa", "nominal_pa", "Kt_meas", "err", "nom_err",
		                                    "global_max_z_mm", "peak_at_fillet")} for x in rows],
		"fea_payloads_verbatim": {str(x["N"]): x["fea_payload_verbatim"] for x in rows},
		"analysis": {"nominal_max_abs_err": max(nom_errs), "coarse_err_range": [coarse_lo, coarse_hi],
		             "err_r6": errs[6], "err_r8": errs[8], "resolution_for_lt10pct": need,
		             "determinism_rel_delta": det_rel},
		"gates": gates,
	}
	jn = lambda o: o.item() if hasattr(o, "item") else str(o)
	json.dump(receipts, open(os.path.join(OUT, "receipts.json"), "w"), indent=1, default=jn)

	try:
		render_analysis_sheet()
	except Exception as exc:                                        # noqa: BLE001
		print(f"analysis sheet: skipped ({type(exc).__name__}: {exc})")

	# ---- verdict ----
	print("\n" + "=" * 78)
	verdict_ok = all(gates.values())
	if verdict_ok:
		print(f"VALIDATION PASS (characterization): voxel hex8 does NOT converge to the "
		      f"theoretical fillet Kt={KT:.3f}.")
		print(f"  Nominal stress is trustworthy (<{max(nom_errs)*100:.1f}%). The fillet PEAK "
		      f"over-reads by {errs[8]*100:+.0f}% at r/8 and scatters {coarse_lo*100:+.0f}%.."
		      f"{coarse_hi*100:+.0f}% at r/2..r/4,")
		print(f"  because it is measuring the STAIRCASED surface (a mesh artifact), not the "
		      f"real fillet. No usable resolution is reliably within 10%; refinement does not fix it.")
		print(f"  => Trust our reported fillet/peak stresses to only ~+/-20-30%, biased HIGH.")
	else:
		print("VALIDATION FAIL: the measured behavior no longer matches the pinned characterization "
		      "(a gate above is FAIL) — re-read the table and UPDATE the docs; do not silently "
		      "re-green. If the peak now CONVERGES to Kt, the voxel-staircase limitation was fixed "
		      "(great — rewrite this pin to assert convergence).")
	print(f"  receipts: {os.path.join(OUT, 'receipts.json')}")
	print("=" * 78)
	sys.exit(0 if verdict_ok else 1)


if __name__ == "__main__":
	main()
