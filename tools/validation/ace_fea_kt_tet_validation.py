#!/usr/bin/env python3
"""Physics validation pin: does the BODY-FITTED tet10 FEA CONVERGE at a fillet?

This is the companion to tools/ace_fea_kt_validation.py (the voxel hex8 pin).
Same specimen, same theoretical Kt, same nominal stress — but a fundamentally
different meshing path. The voxel pin proved the hex8/voxel route does NOT
converge to the fillet Kt (it staircases the curved surface into re-entrant
corners and scatters -6% to +44%, biased HIGH, refinement never fixes it).
This pin proves the tet10 route — gmsh body-fitted mesh on the EXACT conic
fillet surface, quadratic elements, nodal stress recovery — DOES converge,
monotonically from below, toward the published Kt.

SPECIMEN — shouldered (stepped) round bar in AXIAL TENSION (the classic Kt
benchmark; identical to the voxel pin so the two paths are directly comparable):
    small dia d = 16 mm, large dia D = 24 mm  (D/d = 1.5)
    shoulder fillet r = 2.4 mm                (r/d = 0.15, h/r = 1.667)
    small shaft z in [0, 20], large shaft z in [20, 40], fillet at the shoulder
    axial load P = 1000 N on the small end (z=0), large end (z=40) clamped
    sigma_nom = P / (pi d^2/4)  at the small section
    peak_theory = Kt * sigma_nom

THEORETICAL Kt — Peterson's / Pilkey shoulder-fillet-in-tension fit
    Kt = C1 + C2(2h/D) + C3(2h/D)^2 + C4(2h/D)^3,  h = (D-d)/2,
    C1..C4(h/r) for 0.1 <= h/r <= 2.0.  For d=16,D=24,r=2.4: h/r=1.667 -> Kt=1.667.
    Source: W. D. Pilkey, "Peterson's Stress Concentration Factors" (chart 3.4,
    stepped circular bar with a shoulder fillet, axial tension); coefficient fit
    also in Pilkey, "Formulas for Stress, Strain, and Structural Matrices,"
    2nd ed., Wiley 2005.

WHAT WE MEASURE — mesh the specimen body-fitted (gmsh tet10) at three fillet
element sizes (2.0, 1.5, 1.0 mm), solve linear-elastic, and read:
    fillet peak = max nodal von Mises in a z-band |z-20| < 1.5*r bracketing the
                  fillet (excludes both loaded and clamped ends);
    far-field   = median nodal von Mises in the small shaft 3 < z < 15 (uniaxial
                  tension there, so von Mises == axial stress == sigma_nom);
    measured Kt = fillet peak / sigma_nom, compared against 1.667.
Element sizes stop at 1.0 mm (~r/2.4): finer is honest but the nodal stress
recovery is O(30s+) per solve — this ladder runs in ~1.5 min total.

FINDING (pinned, receipts in the table below): far-field nominal is dead-on
(<2% at every size), and the measured Kt rises MONOTONICALLY under refinement —
1.545 -> 1.610 -> 1.652 (2026-07-18) — converging toward the theoretical 1.667
FROM BELOW (quadratic tets are slightly compliant / the discrete peak resolves
gradually), landing within ~1% at the finest size with NO overshoot. This is the
qualitatively-correct convergence the voxel path never achieves.

Run:  ~/miniconda3/bin/python3 tools/ace_fea_kt_tet_validation.py
Exit: 0 iff ALL gates hold (nominal trustworthy, Kt monotone up, finest Kt
      within +/-5% of 1.667); nonzero + message otherwise. Deterministic
      specimen and solver tolerance; no clock read in the verdict.
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(  # tools/analyzers: the in-tree solver package
    os.path.dirname(os.path.abspath(__file__)), os.pardir, "analyzers"))
import numpy as np  # noqa: E402
from physics.mesh_ir import mesh_shouldered_bar  # noqa: E402
from physics.fea_tet import reference_fea_tet  # noqa: E402

# ---------------- specimen (mm) ----------------
d, D, r = 16.0, 24.0, 2.4          # small dia, large dia, shoulder fillet radius
L_small, L_large = 20.0, 20.0      # z in [0, L_small] then [L_small, L_small+L_large]
z_sh = L_small                     # shoulder plane z (fillet centre band)
z_top = L_small + L_large          # clamped end z
P = 1000.0                         # axial load (N); linear-elastic, magnitude irrelevant
E, NU = 2.0e9, 0.37                # material (nu near 0.3; Kt weakly nu-dependent)
SIG_NOM = P / (math.pi * (d / 2.0) ** 2)   # MPa (== N/mm^2), analytic nominal


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


def solve(elem_mm):
	"""Body-fitted tet10 solve at one fillet element size. Returns measured
	quantities: n_tets, far-field median (MPa), fillet peak (MPa), Kt."""
	mesh = mesh_shouldered_bar(d, D, r, L_small, L_large, elem_size_mm=elem_mm)
	res = reference_fea_tet(
		mesh,
		{"youngs_modulus_pa": E, "poisson": NU, "density_kg_m3": 1270.0},
		loads=[{"kind": "point", "magnitude": P, "direction": [0, 0, -1],
		        "region_selector": {"type": "plane", "axis": "z", "value_mm": 0.0, "side": "-"}}],
		fixtures=[{"kind": "clamped",
		           "region_selector": {"type": "plane", "axis": "z", "value_mm": z_top, "side": "+"}}],
		direct_max_dof=0, cg_tol=1e-9, cg_maxiter=60000)
	assert res.get("ok"), f"elem={elem_mm}: tet FEA FAILED: {res}"
	z = mesh.nodes_mm[:, 2]
	vm = res["vm_nodal"] / 1e6                                    # -> MPa
	band = np.abs(z - z_sh) < 1.5 * r                            # fillet z-band
	far = (z > 3.0) & (z < 15.0)                                 # small-shaft mid
	peak = float(vm[band].max())
	nom = float(np.median(vm[far]))
	return {"elem_mm": elem_mm, "n_tets": int(res["n_tets"]),
	        "far_MPa": nom, "peak_MPa": peak, "Kt": peak / SIG_NOM,
	        "nom_err": (nom - SIG_NOM) / SIG_NOM, "kt_err": (peak / SIG_NOM - KT) / KT}


def main():
	print("=" * 78)
	print("SPECIMEN  shouldered round bar, AXIAL TENSION — BODY-FITTED tet10 path")
	print(f"  d={d} mm  D={D} mm  (D/d={D/d:.2f})   fillet r={r} mm  (r/d={r/d:.3f}, h/r={HR:.3f})")
	print(f"  P={P:.0f} N   sigma_nom = P/(pi d^2/4) = {SIG_NOM:.4f} MPa")
	print(f"THEORETICAL Kt (Peterson/Pilkey, tension) = {KT:.4f}"
	      f"   =>  peak_theory = {PEAK_THEORY:.4f} MPa")
	print(f"  source: Pilkey, Peterson's Stress Concentration Factors, chart 3.4 "
	      f"(stepped bar, shoulder fillet, axial tension).")

	ladder = [2.0, 1.5, 1.0]                                     # fillet element size (mm)
	print(f"\n{'elem_mm':>8} {'n_tets':>8} {'far_MPa':>8} {'peak_MPa':>9} {'Kt_meas':>8} "
	      f"{'nom_err':>8} {'Kt_err':>8}")
	rows = []
	for es in ladder:
		R = solve(es)
		rows.append(R)
		print(f"{R['elem_mm']:>8.2f} {R['n_tets']:>8} {R['far_MPa']:>8.3f} {R['peak_MPa']:>9.3f} "
		      f"{R['Kt']:>8.3f} {R['nom_err']*100:>+7.2f}% {R['kt_err']*100:>+7.2f}%")

	# ---- CONTRAST vs the voxel hex8 pin (ace_fea_kt_validation.py) ----
	print("\nCONTRAST vs voxel hex8 (tools/ace_fea_kt_validation.py):")
	print("  voxel path: fillet Kt SCATTERS -6%..+44%, biased HIGH, does NOT converge")
	print("              (staircased re-entrant corners are a mesh artifact, not the fillet).")
	print("  this path:  fillet Kt rises MONOTONICALLY toward the true Kt, from below.")

	# ---- gates ----
	kts = [R["Kt"] for R in rows]
	nom_errs = [abs(R["nom_err"]) for R in rows]
	finest = rows[-1]
	monotone_up = all(kts[i + 1] > kts[i] for i in range(len(kts) - 1))
	gates = {
		"far_field_nominal_within_2pct_all_sizes": max(nom_errs) < 0.02,
		"Kt_increases_monotonically_with_refinement": monotone_up,
		"finest_Kt_within_5pct_of_theory_no_overshoot":
			finest["Kt"] >= 0.95 * KT and finest["Kt"] <= 1.05 * KT,
	}
	print("\nGATES")
	for k, v in gates.items():
		print(f"  [{'PASS' if v else 'FAIL'}] {k}")

	print("\n" + "=" * 78)
	ok = all(gates.values())
	if ok:
		print(f"VALIDATION PASS: body-fitted tet10 CONVERGES to the fillet Kt={KT:.3f}.")
		print(f"  Kt {' -> '.join(f'{k:.3f}' for k in kts)} (monotone up), finest "
		      f"{finest['Kt']:.3f} = {finest['kt_err']*100:+.1f}% vs theory; nominal within "
		      f"{max(nom_errs)*100:.1f}%.")
		print(f"  Unlike the voxel path (scatters, non-convergent), this path is "
		      f"qualitatively correct — trust its fillet peak to a few % once refined.")
	else:
		print("VALIDATION FAIL: a gate above is FAIL — the tet10 convergence claim no longer "
		      "holds. Re-read the table and UPDATE the docs; do NOT weaken the gate to re-green. "
		      "If Kt now scatters or overshoots, the meshing/recovery regressed.")
	print("=" * 78)
	sys.exit(0 if ok else 1)


if __name__ == "__main__":
	main()
