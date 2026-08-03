#!/usr/bin/env python3
"""Optimizer validation pin: ace_optimize (SIMP/OC) vs exact physics inequalities.

No closed-form OPTIMAL TOPOLOGY exists to pin a topology optimizer against —
what CAN be pinned are exact, falsifiable properties any correct SIMP/OC loop
must satisfy on a cantilever whose FEA is itself pinned (ace_fea_validation.py):

	Pin A  descent: compliance_last < compliance_first — the OC update must
	       IMPROVE the design it was given, not wander.
	Pin B  monotonicity of material removal: the thresholded as-built part
	       (a strict SUBSET of the solid domain, same grid, same load) must
	       deflect MORE than the full solid — removing material never
	       stiffens a structure. A violated inequality = broken assembly of
	       the reduced stiffness matrix or a load dropped onto void.
	Pin C  volume honesty: the OC bisection must actually deliver the asked
	       volume fraction (volume_fraction_achieved within 0.02 of volfrac).
	Pin D  deliverable honesty: the receipt's STL is watertight (the runner
	       escalates resolution until the gated mesh passes, or says so).

Setup: 40x8x8 mm cantilever @ 1 mm voxels, clamped x=0, 10 N tip load at
x=40 (tip + clamp slabs frozen), volfrac 0.4, 25 OC iterations. Geometry is
passed as a density .npy — the pin needs no LMCAD kernel build for physics
(the STL gate uses the kernel binary; absent binary => stl.ok false => pin D
reports it honestly).

Measured 2026-07-17 (pinned here): compliance 3.671e-02 -> 5.554e-03 J
(x6.61 stiffer than the uniform-0.4 start), as-built/solid deflection
ratio 1.58, volume_fraction_achieved 0.400.

Run:  ACE_PYTHON (default ~/miniconda3/bin/python3) this file.
Exit: 0 iff all assertions hold; nonzero with a message otherwise.
"""
import json
import os
import subprocess
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.environ.get("ACE_ROOT", os.path.expanduser("~/Work/ACE")))
from engine.verify import reference_fea  # noqa: E402

TOOLS = os.path.dirname(os.path.abspath(__file__))
L, B, H = 40, 8, 8  # mm, voxel 1.0 => grid shape (40, 8, 8)
P, E, NU = 10.0, 2.2e9, 0.37
MATERIAL = {"youngs_modulus_pa": E, "poisson": NU, "density_kg_m3": 1270.0}
LOADS = [{"kind": "point", "magnitude": P, "direction": [0, 0, -1],
          "region_selector": {"type": "plane", "axis": "x", "value_mm": float(L), "side": "+"}}]
FIXTURES = [{"kind": "clamped",
             "region_selector": {"type": "plane", "axis": "x", "value_mm": 0.0, "side": "-"}}]
VOLFRAC = 0.4


def main() -> None:
	work = tempfile.mkdtemp(prefix="ace_opt_pin_")

	# Reference: the FULL SOLID beam on the identical grid/load/fixture —
	# solved with the same pinned reference_fea the runner uses internally.
	solid = reference_fea(
		np.ones((L, B, H), dtype=np.float32),
		np.full((L, B, H), "design", dtype=object),
		1.0, MATERIAL, loads=LOADS, fixtures=FIXTURES, direct_solver_max_dof=0)
	d_solid = float(solid["max_displacement_m"])

	npy = os.path.join(work, "solid.npy")
	np.save(npy, np.ones((L, B, H), dtype=np.float32))
	job = {
		"out_dir": os.path.join(work, "opt"),
		"voxel_mm": 1.0,
		"npy": npy,
		"material": MATERIAL, "loads": LOADS, "fixtures": FIXTURES,
		# Freeze the clamp root and the loaded tip slab so the load path can
		# never be optimized away (the runner's documented contract).
		"regions": [
			{"kind": "frozen", "selector": {"type": "plane", "axis": "x", "value_mm": 1.0, "side": "-"}},
			{"kind": "frozen", "selector": {"type": "plane", "axis": "x", "value_mm": float(L - 1), "side": "+"}},
		],
		"volfrac": VOLFRAC, "max_iters": 25, "time_budget_s": 300.0,
	}
	job_path = os.path.join(work, "job.json")
	with open(job_path, "w") as f:
		json.dump(job, f)
	out = subprocess.run([sys.executable, os.path.join(TOOLS, "ace_optimize_runner.py"), job_path],
	                     capture_output=True, text=True, timeout=600)
	last = ""
	for line in out.stdout.splitlines():
		if line.strip():
			last = line
	assert last, f"runner produced no receipt; stderr tail: {out.stderr[-400:]!r}"
	r = json.loads(last)
	assert r.get("ok"), f"runner refused: {json.dumps(r)[:300]}"

	c0, c1 = float(r["compliance_first"]), float(r["compliance_last"])
	vfrac = float(r["volume_fraction_achieved"])
	d_built = float(r["as_built"]["max_displacement_m"])
	print(f"compliance {c0:.3e} -> {c1:.3e} J (x{c0 / c1:.2f}) · "
	      f"as-built/solid deflection {d_built / d_solid:.2f} · vol {vfrac:.3f}")

	failures = []
	# Pin A — descent. A healthy OC run on this problem improves compliance
	# several-fold; require at least strict 10% improvement so noise can't pass.
	if not c1 <= 0.9 * c0:
		failures.append(
			f"no descent: compliance {c0:.3e} -> {c1:.3e} (needs <=0.9x) — the OC "
			f"update or the SIMP sensitivity sign is broken")
	# Pin B — removing material must NOT stiffen: as-built (subset of solid
	# domain, same hex8 grid, same load) deflects at least as much as solid.
	if not d_built >= d_solid:
		failures.append(
			f"as-built deflection {d_built:.3e} m < solid {d_solid:.3e} m — a part "
			f"with 60% of the material cannot be stiffer; reduced-stiffness "
			f"assembly or load application is wrong")
	# Pin C — volume bisection honesty.
	if not abs(vfrac - VOLFRAC) <= 0.02:
		failures.append(
			f"volume_fraction_achieved {vfrac:.3f} vs asked {VOLFRAC} (tol 0.02) — "
			f"the OC volume bisection is not converging")
	# Pin D — the deliverable is a watertight mesh, or the receipt says why not.
	if not (r.get("stl", {}).get("ok") and r["stl"].get("watertight")):
		failures.append(
			f"STL gate: ok={r.get('stl', {}).get('ok')} watertight="
			f"{r.get('stl', {}).get('watertight')} issues={r.get('stl', {}).get('issues')}")

	if failures:
		print("VALIDATION FAIL:", "; ".join(failures))
		sys.exit(1)
	print("VALIDATION PASS: descent, material-removal monotonicity, volume honesty, watertight deliverable.")


if __name__ == "__main__":
	main()
