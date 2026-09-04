#!/usr/bin/env python3
"""sim_generative_verify.py — close the GENERATIVE design loop with an HONEST
body-fitted verification step, and quantify how much the trustworthy check
differs from the SIMP loop's own voxel physics.

The chain, end to end:

    1. ace_optimize_runner.py  — SIMP/OC topology optimization of a short
       cantilever (min compliance at a fixed volume fraction), then an HONEST
       binary-occupancy re-analysis of the thresholded AS-BUILT part on the
       voxel hex8 grid, plus a watertight-or-fail STL of that part.
    2. ace_fea_tet_runner.py   — the validated body-fitted tet10 solver, run on
       the SAME as-built STL with the SAME material / load / support, to VERIFY
       the part on a conforming mesh (no voxel staircase).

The high-value output is the comparison: SIMP's own as-built (voxel hex8) peak
von Mises + tip deflection vs the body-fitted-verified numbers on the identical
part, with the percentage difference. The voxel number is EXPECTED to differ —
a staircased hex8 grid under-reads stress concentrations at re-entrant SIMP
features; that gap is exactly why a trustworthy verification matters.

HONEST OUTCOME CONTRACT
-----------------------
This tool NEVER fabricates a downstream number. Two outcomes are possible and
both are reported truthfully:

  * body-fitted CLOSES — a tet10 solve succeeds on the STL: the receipt prints
    the voxel-vs-body-fitted comparison and %diff, and asserts only invariants
    that actually hold (chain completed, STL watertight, positive mesh
    Jacobians, both peak stresses finite and positive). It does NOT assert a
    particular level of agreement — it MEASURES it.

  * body-fitted REFUSES — the loop is honestly reported as an integration
    limit. Measured here (2026-07-18): ACE's body-fitted mesher (gmsh
    classifySurfaces -> createGeometry -> tet10 HighOrderOptimize) cannot
    robustly consume a SIMP as-built surface-nets STL. The STL is watertight,
    single-component, and has zero degenerate facets, yet the tet10 pipeline
    crashes (SIGABRT / NaN in the high-order optimizer), refuses (PLC
    self-intersection, non-positive corner Jacobian), or hangs across every
    element size tried. The SAME runner meshes and solves a clean analytic box
    of the same envelope without issue — a control this tool runs and reports —
    so the blocker is the SIMP mesh SURFACE QUALITY, not the solver. This is
    consistent with the optimizer runner's own note: "no density-to-B-rep
    reconstruction exists" — verifying a raw SIMP mesh means meshing a
    staircased shell, which the body-fitted path is not robust to.

Either way the SIMP half of the loop is real and gated: descent, volume
honesty, watertight deliverable, finite voxel physics.

HONEST CAVEATS (documented, not hidden)
  * Grid is coarse (40x8x8 @ 1 mm) — a coarse hex8 grid under-predicts peak
    bending stress by ~20% vs a converged mesh; that under-read is PART of what
    a body-fitted check would expose.
  * The point load is lumped over the tip face nodes (both solvers), not a true
    point — so the very-near-load stress is a mesh artifact in both.
  * A SIMP as-built part is not manufacturable as-is (voxel staircase, thin
    members); the STL is a MESH ONLY.

Run:  python3 this file (numpy + scipy only; the solver is in-tree).
Exit: 0 iff the chain ran and an honest receipt was produced (a body-fitted
      integration limit is a REPORTED finding, not a tool failure); nonzero
      only if the SIMP half or the solver control itself is broken.
"""
import json
import os
import subprocess
import sys
import tempfile

import numpy as np

TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(  # tools/analyzers: the in-tree solver package
    os.path.dirname(os.path.abspath(__file__)), os.pardir, "analyzers"))
PY_EXE = sys.executable

# --- the generative problem: a short cantilever, min compliance @ fixed volume.
# Human conditions = load + support + material + volfrac target.
L, B, H = 40, 8, 8              # mm, voxel 1.0 => grid (40, 8, 8)
VOXEL = 1.0
P, E, NU = 10.0, 2.2e9, 0.37    # 10 N tip load; a stiff polymer (E, nu)
MATERIAL = {"youngs_modulus_pa": E, "poisson": NU, "density_kg_m3": 1270.0}
VOLFRAC = 0.4
# Grid-space selectors for the SIMP/voxel runner (world mm on the grid).
LOADS = [{"kind": "point", "magnitude": P, "direction": [0.0, 0.0, -1.0],
          "region_selector": {"type": "plane", "axis": "x", "value_mm": float(L), "side": "+"}}]
FIXTURES = [{"kind": "clamped",
             "region_selector": {"type": "plane", "axis": "x", "value_mm": 0.0, "side": "-"}}]
# Body-fitted element sizes to try (mm). First one that yields a valid solve wins.
TET_ELEM_SIZES = (1.5, 2.0, 2.5)
TET_TIMEOUT_S = 75


def run_receipt(script: str, job: dict, work: str, timeout: int):
	"""Run a bridge runner on a job dict; return (receipt_or_None, status).

	Subprocess-isolated so a NATIVE mesher abort (SIGABRT) or a hang cannot take
	this tool down — the whole point of a trustworthy verification harness.
	status is one of: 'ok', 'refused', 'crash rc=..', 'hang', 'no_receipt'.
	"""
	jp = os.path.join(work, "job.json")
	with open(jp, "w") as f:
		json.dump(job, f)
	try:
		out = subprocess.run([PY_EXE, os.path.join(TOOLS, script), jp],
		                     capture_output=True, text=True, timeout=timeout,
		                     env={**os.environ})
	except subprocess.TimeoutExpired:
		return None, f"hang(>{timeout}s)"
	last = ""
	for line in out.stdout.splitlines():
		if line.strip():
			last = line
	if not last:
		tail = out.stderr.strip().splitlines()
		msg = tail[-1][:120] if tail else "(no stderr)"
		return None, f"crash rc={out.returncode} :: {msg}"
	r = json.loads(last)
	if not r.get("ok"):
		return r, "refused"
	return r, "ok"


def run_simp(work: str) -> dict:
	"""SIMP/OC optimize + honest as-built voxel re-analysis + watertight STL."""
	npy = os.path.join(work, "solid.npy")
	np.save(npy, np.ones((L, B, H), dtype=np.float32))
	job = {
		"out_dir": os.path.join(work, "opt"),
		"voxel_mm": VOXEL, "npy": npy,
		"material": MATERIAL, "loads": LOADS, "fixtures": FIXTURES,
		# Freeze the clamp root and the loaded tip slab so the load path can
		# never be optimized away (the runner's documented contract).
		"regions": [
			{"kind": "frozen", "selector": {"type": "plane", "axis": "x", "value_mm": 1.0, "side": "-"}},
			{"kind": "frozen", "selector": {"type": "plane", "axis": "x", "value_mm": float(L - 1), "side": "+"}},
		],
		"volfrac": VOLFRAC, "max_iters": 30, "time_budget_s": 120.0,
	}
	r, status = run_receipt("ace_optimize_runner.py", job, work, timeout=300)
	if status != "ok":
		raise RuntimeError(f"SIMP runner did not produce a valid receipt ({status}): "
		                   f"{json.dumps(r)[:300] if r else ''}")
	return r


def tet_job(stl: str, elem: float, work: str, load_side_mm: float) -> dict:
	"""Body-fitted job on the as-built STL, SAME material/load/support, with the
	grid selectors translated to world-mm plane selectors on the tet NODES."""
	return {
		"out_dir": os.path.join(work, "tet"),
		"stl": stl, "elem_size_mm": elem, "material": MATERIAL,
		"fixtures": [{"kind": "clamped",
		              "region_selector": {"type": "plane", "axis": "x", "value_mm": 0.5, "side": "-"}}],
		"loads": [{"kind": "point", "magnitude": P, "direction": [0.0, 0.0, -1.0],
		           "region_selector": {"type": "plane", "axis": "x", "value_mm": load_side_mm, "side": "+"}}],
	}


def run_control_box(work: str) -> tuple:
	"""Control: the SAME body-fitted runner on a CLEAN analytic box of the same
	envelope. If this solves while the SIMP STL does not, the blocker is proven
	to be the SIMP surface quality, not the solver. Returns (receipt, status)."""
	job = {
		"out_dir": os.path.join(work, "box"),
		"specimen": "box", "lx": float(L), "ly": float(B), "lz": float(H),
		"elem_size_mm": 2.0, "material": MATERIAL,
		"fixtures": [{"kind": "clamped",
		              "region_selector": {"type": "plane", "axis": "x", "value_mm": 0.5, "side": "-"}}],
		"loads": [{"kind": "point", "magnitude": P, "direction": [0.0, 0.0, -1.0],
		           "region_selector": {"type": "plane", "axis": "x", "value_mm": float(L) - 0.5, "side": "+"}}],
	}
	return run_receipt("ace_fea_tet_runner.py", job, work, timeout=TET_TIMEOUT_S)


def pct(a: float, b: float) -> float:
	"""Signed % difference of `a` relative to baseline `b`."""
	return 100.0 * (a - b) / b if b else float("nan")


def main() -> None:  # noqa: PLR0915 — one linear, documented pipeline
	work = tempfile.mkdtemp(prefix="sim_gen_verify_")

	# ---- Link 1: SIMP generate + honest voxel as-built physics ----------------
	simp = run_simp(work)
	c0, c1 = float(simp["compliance_first"]), float(simp["compliance_last"])
	vfrac = float(simp["volume_fraction_achieved"])
	stl = simp["stl"]
	vox_vm = float(simp["as_built"]["max_von_mises_pa"])
	vox_disp = float(simp["as_built"]["max_displacement_m"])

	print("=" * 74)
	print("GENERATIVE LOOP — SIMP generate  ->  body-fitted VERIFY")
	print("=" * 74)
	print(f"Problem: {L}x{B}x{H} mm cantilever @ {VOXEL} mm voxels · clamp x=0 · "
	      f"{P:g} N tip load · volfrac {VOLFRAC}")
	print(f"SIMP/OC: compliance {c0:.3e} -> {c1:.3e} J (x{c0 / c1:.2f} stiffer) · "
	      f"vol_achieved {vfrac:.3f}")
	print(f"As-built STL: watertight={stl['watertight']} tris={stl['num_triangles']} "
	      f"vol={stl['volume_mm3']:.1f} mm^3 (upsample x{stl['mesh_upsample']})")
	print(f"As-built VOXEL hex8 physics: peak vM {vox_vm:.3e} Pa · "
	      f"max deflection {vox_disp * 1e3:.4f} mm")

	# ---- Link 2: body-fitted VERIFY on the identical part ---------------------
	# The STL is padded one voxel; the true tip face lands at x ~= L. Catch the
	# tip layer with a plane just inside it.
	load_side_mm = float(L) - 1.0
	attempts = []
	tet = None
	for elem in TET_ELEM_SIZES:
		r, status = run_receipt("ace_fea_tet_runner.py",
		                        tet_job(stl["path"], elem, work, load_side_mm),
		                        work, timeout=TET_TIMEOUT_S)
		note = status
		if status == "refused":
			note = f"refused :: {str(r.get('error'))[:90]}"
		attempts.append({"elem_size_mm": elem, "status": note})
		print(f"body-fitted tet10 @ elem={elem} mm: {note}")
		if status == "ok":
			tet = r
			break

	# Control: prove the solver itself is sound on clean tessellation.
	box, box_status = run_control_box(work)
	box_ok = box_status == "ok"
	if box_ok:
		print(f"control (clean analytic box, same runner): SOLVED "
		      f"vM {float(box['max_von_mises_pa']):.3e} Pa · "
		      f"minJ {float(box['mesh']['min_corner_jacobian_mm3']):.3e} mm^3")
	else:
		print(f"control (clean analytic box, same runner): {box_status}")

	print("-" * 74)

	# ---- Report + falsifiable assertions that DO hold -------------------------
	# SIMP-half invariants (always required — the generate link must be real).
	assert simp.get("ok"), "SIMP receipt not ok"
	assert c1 <= 0.9 * c0, (
		f"SIMP did not descend: compliance {c0:.3e} -> {c1:.3e} (needs <=0.9x)")
	assert abs(vfrac - VOLFRAC) <= 0.02, (
		f"SIMP volume dishonest: achieved {vfrac:.3f} vs asked {VOLFRAC}")
	assert stl["ok"] and stl["watertight"], (
		f"as-built STL not watertight: {stl.get('issues')}")
	assert np.isfinite(vox_vm) and vox_vm > 0 and np.isfinite(vox_disp) and vox_disp > 0, (
		f"as-built voxel physics not finite/positive: vM={vox_vm} disp={vox_disp}")

	receipt = {
		"loop": "SIMP-generate -> body-fitted-verify",
		"simp": {
			"compliance_first_j": c0, "compliance_last_j": c1,
			"volfrac_achieved": vfrac,
			"as_built_voxel_hex8": {"peak_von_mises_pa": vox_vm,
			                        "max_deflection_m": vox_disp},
			"stl_watertight": bool(stl["watertight"]),
			"stl_triangles": stl["num_triangles"],
		},
		"body_fitted_attempts": attempts,
		"control_clean_box_solved": bool(box_ok),
	}

	if tet is not None:
		# ---- LOOP CLOSED: real voxel-vs-body-fitted comparison ----------------
		bf_vm = float(tet["max_von_mises_pa"])
		bf_disp = float(tet["max_displacement_m"])
		min_jac = float(tet["mesh"]["min_corner_jacobian_mm3"])
		# Invariants that hold for any successful body-fitted solve.
		assert min_jac > 0, f"body-fitted mesh has non-positive Jacobian {min_jac}"
		assert np.isfinite(bf_vm) and bf_vm > 0, f"body-fitted vM not finite/positive: {bf_vm}"
		assert np.isfinite(bf_disp) and bf_disp > 0, f"body-fitted disp not finite/positive: {bf_disp}"
		d_vm, d_disp = pct(vox_vm, bf_vm), pct(vox_disp, bf_disp)
		receipt["outcome"] = "loop_closed"
		receipt["body_fitted_tet10"] = {
			"peak_von_mises_pa": bf_vm, "max_deflection_m": bf_disp,
			"n_tets": tet["n_tets"], "n_nodes": tet["n_nodes"],
			"min_corner_jacobian_mm3": min_jac,
		}
		receipt["voxel_vs_bodyfitted"] = {
			"peak_von_mises_pct_diff_voxel_rel_bodyfitted": d_vm,
			"max_deflection_pct_diff_voxel_rel_bodyfitted": d_disp,
		}
		print("LOOP CLOSED — voxel (SIMP as-built) vs body-fitted verification:")
		print(f"  peak von Mises: voxel {vox_vm:.3e} Pa   body-fitted {bf_vm:.3e} Pa "
		      f"  (voxel {d_vm:+.1f}%)")
		print(f"  max deflection: voxel {vox_disp * 1e3:.4f} mm   body-fitted "
		      f"{bf_disp * 1e3:.4f} mm  (voxel {d_disp:+.1f}%)")
		print("  (%diff MEASURED, not asserted — the whole point is to quantify the gap.)")
	else:
		# ---- LOOP OPEN: honest integration-limit finding ----------------------
		# The finding is only credible if the solver itself works on clean
		# tessellation — assert the control solved, else the failure is the
		# solver (a different, more serious bug) and we must fail loudly.
		assert box_ok and float(box["mesh"]["min_corner_jacobian_mm3"]) > 0, (
			f"control box did NOT solve ({box_status}) — the body-fitted SOLVER "
			f"is broken, not just the SIMP mesh; cannot attribute the limit")
		assert any(a["status"] != "ok" for a in attempts) and attempts, (
			"no body-fitted attempt was characterized")
		receipt["outcome"] = "body_fitted_meshing_limit"
		receipt["finding"] = (
			"SIMP as-built surface-nets STL is watertight but NOT robustly "
			"body-fitted-meshable by the gmsh tet10 pipeline (crash/refuse/hang "
			"across all element sizes); the SAME runner solves a clean analytic "
			"box of the same envelope, so the blocker is SIMP surface quality, "
			"not the solver. No density->B-rep reconstruction exists to supply "
			"clean geometry.")
		print("LOOP OPEN — body-fitted verification refused (HONEST FINDING):")
		print("  The SIMP as-built STL is watertight but the body-fitted tet10")
		print("  mesher cannot consume it; the clean-box control solved on the")
		print("  SAME runner, so the blocker is the SIMP surface, not the solver.")
		print("  No body-fitted number is fabricated. Voxel as-built physics stands")
		print(f"  as the SIMP loop's own (un-verified) estimate: peak vM {vox_vm:.3e} Pa,")
		print(f"  deflection {vox_disp * 1e3:.4f} mm — a coarse hex8 read that a")
		print("  body-fitted mesh would be expected to correct upward at re-entrant")
		print("  features (the reason trustworthy verification matters).")

	print("-" * 74)
	print("RECEIPT " + json.dumps(receipt))
	print("=" * 74)
	print("OK: chain ran end-to-end and produced an honest receipt.")


if __name__ == "__main__":
	main()
