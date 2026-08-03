#!/usr/bin/env python3
"""sim_generative_reconstruct.py — CLOSE the generative simulation-driven design
loop that sim_generative_verify.py leaves OPEN.

sim_generative_verify.py proved the loop was OPEN: SIMP produces a watertight
STL, but the STL is a STAIRCASED surface-nets shell that the body-fitted gmsh
tet10 pipeline cannot robustly consume (SIGABRT / non-positive Jacobian / PLC
self-intersection). The missing link is a RECONSTRUCTION step: turn the SIMP
CONTINUOUS density field (rho in [0,1] on the voxel grid) into a SMOOTH
watertight iso-surface that the body-fitted mesher CAN consume.

The chain here:

    1. SIMP/OC optimize the same short cantilever (reused verbatim from
       sim_generative_verify.run_simp) -> final_rho.npy (graded density) +
       grid frame (origin_mm, voxel_mm, shape).
    2. RECONSTRUCT a smooth surface at iso=0.5 of the CONTINUOUS density via
       skimage.measure.marching_cubes (interpolated iso-surface, NOT the binary
       staircase), padded so it closes into a watertight manifold, then a few
       volume-preserving Taubin smoothing passes to shed residual voxel ridges.
    3. Body-fitted mesh it to a STRAIGHT-SIDED tet10 volume (gmsh
       classifySurfaces -> createGeometry -> tet10 with HighOrderOptimize OFF —
       the curved-element untangler SIGABRTs on organic topology — and
       SecondOrderLinear ON so mid-side nodes stay at straight edge midpoints
       instead of curving into solver-rejected slivers). mesh.check() gates
       positive Jacobians. See tet_solve_probe.
    4. If it meshes, run reference_fea_tet with the SAME load/support as the
       SIMP problem (grid selectors translated to world-mm planes on the tet
       nodes).
    5. Print an HONEST receipt: SIMP voxel-predicted vs body-fitted-verified
       peak von Mises + tip deflection + %diff — MEASURED, not assumed.

HONESTY CONTRACT (this repo forbids gaming metrics): no downstream number is
fabricated. If the smooth surface still will not mesh/solve, that is reported as
the real limit, never papered over. Only invariants that actually hold are
asserted (reconstruction watertight + single-component, mesh positive-Jacobian,
solve finite). The %diff is MEASURED and printed, never asserted. Honest caveat:
the verified solve is straight-sided tet10 (curvature-untangling off), so the
peak stress is marginally less sharp than fully-curved elements would read —
still a real body-fitted conforming-mesh verification, vastly better than the
voxel staircase.

Run:  ACE_ROOT=~/Work/ACE <ACE_PYTHON> sim_generative_reconstruct.py
Exit: 0 iff the chain ran and produced an honest receipt (a residual meshing
      limit is a REPORTED finding, not a tool failure); nonzero only if the
      SIMP half or the reconstruction invariants themselves break.
"""
import json
import os
import subprocess
import sys
import tempfile

import numpy as np

TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TOOLS)
ACE_ROOT = os.environ.get("ACE_ROOT", os.path.expanduser("~/Work/ACE"))
os.environ.setdefault("ACE_ROOT", ACE_ROOT)
PY_EXE = sys.executable

# Reuse the SIMP-run helper, subprocess-isolated runner, and problem constants
# from the loop-OPEN tool verbatim (do NOT re-implement the generate link).
from sim_generative_verify import (  # noqa: E402
	L, B, H, VOXEL, P, MATERIAL, VOLFRAC, run_simp, pct,
)

# Body-fitted tet10 element sizes to try; first POSITIVE-Jacobian solve wins.
# The mesh is built straight-sided (see tet_solve_probe) so the solver
# accepts it on this organic topology where the untangler would SIGABRT.
TET_ELEM_SIZES = (1.5, 1.0)
TET_TIMEOUT_S = 120
SMOOTH_ITERS = 40          # volume-preserving Taubin passes
SMOOTH_PASSBAND = 0.05
# Upsampling the density (denser surface) made gmsh WORSE (reparametrization
# hangs + tetgen PLC self-intersections), so reconstruct at native resolution.
DENSITY_UPSAMPLE = 1


def reconstruct_surface(rho_npy: str, out_stl: str) -> dict:
	"""Iso=0.5 marching-cubes reconstruction of the CONTINUOUS density field,
	volume-preserving Taubin-smoothed, written watertight to out_stl.

	Returns a receipt dict with watertight / single-component / normals /
	volume facts. Raises only on a hard reconstruction failure."""
	import pyvista as pv
	from scipy.ndimage import zoom
	from skimage.measure import marching_cubes

	rho = np.load(rho_npy).astype(np.float64)          # (nx,ny,nz) graded density
	f = DENSITY_UPSAMPLE
	if f > 1:
		rho = zoom(rho, f, order=3)                    # cubic -> smoother iso
	spacing = VOXEL / f
	# Pad one layer of VOID so the iso-surface closes into a watertight manifold
	# wherever the part touches the grid boundary (frozen full-height end slabs).
	padded = np.pad(rho, 1, mode="constant", constant_values=0.0)
	# Smooth INTERPOLATED iso-surface of the graded field — not the binary
	# staircase. spacing carries the physical mm scale so element sizes are
	# meaningful; the constant origin offset is immaterial (planes from bounds).
	verts_world, faces, _normals, _vals = marching_cubes(
		padded, level=0.5, spacing=(spacing, spacing, spacing))

	pv_faces = np.hstack(
		[np.full((faces.shape[0], 1), 3, dtype=np.int64), faces.astype(np.int64)]
	).ravel()
	mesh = pv.PolyData(verts_world, pv_faces)
	mesh = mesh.clean()                                # merge coincident verts

	raw_open = int(mesh.n_open_edges)
	# Volume-preserving Taubin smoothing to shed voxel ridges without shrinking.
	vol_before = float(mesh.volume)
	mesh = mesh.smooth_taubin(n_iter=SMOOTH_ITERS, pass_band=SMOOTH_PASSBAND)
	mesh = mesh.clean()
	vol_after = float(mesh.volume)

	# Single largest connected component (the frozen load path is connected; any
	# tiny SIMP debris islands are dropped and REPORTED, never silently kept).
	conn = mesh.connectivity("all")
	n_regions = int(conn["RegionId"].max()) + 1 if conn.n_cells else 0
	if n_regions > 1:
		mesh = mesh.connectivity("largest")
		mesh = mesh.clean()

	# Consistent outward normals for a clean STL surface classification.
	mesh = mesh.compute_normals(
		consistent_normals=True, auto_orient_normals=True,
		non_manifold_traversal=False)
	tri = mesh.triangulate()

	open_edges = int(tri.n_open_edges)
	tri.save(out_stl)
	xmin, xmax = float(tri.bounds[0]), float(tri.bounds[1])
	return {
		"path": out_stl,
		"n_triangles": int(tri.n_cells),
		"n_points": int(tri.n_points),
		"raw_open_edges": raw_open,
		"open_edges": open_edges,
		"watertight": open_edges == 0,
		"n_components_before_prune": n_regions,
		"volume_mm3_before_smooth": vol_before,
		"volume_mm3": vol_after,
		"volume_shrink_pct": pct(vol_after, vol_before),
		"x_min_mm": xmin, "x_max_mm": xmax,
	}


def tet_solve_probe(stl: str, elem: float, clamp_x: float, load_x: float) -> None:
	"""Subprocess-isolated body-fitted mesh+solve subcommand (so a gmsh abort
	can't sink the parent). Meshes a STRAIGHT-SIDED tet10 via the sanctioned
	mesh_stl (order=2, high_order_optimize=False to dodge the untangler SIGABRT,
	second_order_linear=True so mid-side nodes stay at straight edge midpoints
	instead of curving into solver-rejected slivers + gmsh Optimize/OptimizeNetgen
	quality passes), gates positive Jacobians via mesh.check(), then
	reference_fea_tet with the SAME material/load/support. Prints ONE JSON line
	(the contract), exit 0."""
	from engine.verify.fea_tet import reference_fea_tet
	from engine.verify.mesh_ir import mesh_stl

	try:
		mesh = mesh_stl(stl, elem_size_mm=elem, order=2,
		                high_order_optimize=False, second_order_linear=True)
		chk = mesh.check()                              # raises on inverted element
		fixtures = [{"kind": "clamped",
		             "region_selector": {"type": "plane", "axis": "x",
		                                 "value_mm": clamp_x, "side": "-"}}]
		loads = [{"kind": "point", "magnitude": P, "direction": [0.0, 0.0, -1.0],
		          "region_selector": {"type": "plane", "axis": "x",
		                              "value_mm": load_x, "side": "+"}}]
		res = reference_fea_tet(mesh, MATERIAL, loads, fixtures,
		                        direct_max_dof=0, cg_tol=1e-9, cg_maxiter=60000)
		if not res.get("ok"):
			print(json.dumps({"ok": False, "error": f"solver: {res.get('error')}"}),
			      flush=True)
			return
		print(json.dumps({
			"ok": True,
			"max_von_mises_pa": res["max_von_mises_pa"],
			"max_displacement_m": res["max_disp_m"],
			"n_tets": res["n_tets"], "n_nodes": res["n_nodes"],
			"min_corner_jacobian_mm3": chk.get("min_corner_jacobian_mm3"),
		}), flush=True)
	except Exception as exc:  # noqa: BLE001 — JSON line is the contract
		print(json.dumps({"ok": False, "error": f"{type(exc).__name__}: {exc}"}),
		      flush=True)


def main() -> None:  # noqa: PLR0915 — one linear, documented pipeline
	work = tempfile.mkdtemp(prefix="sim_gen_recon_")

	# ---- Link 1: SIMP generate (reused verbatim) ------------------------------
	simp = run_simp(work)
	c0, c1 = float(simp["compliance_first"]), float(simp["compliance_last"])
	vfrac = float(simp["volume_fraction_achieved"])
	rho_npy = simp["final_rho_npy"]
	vox_vm = float(simp["as_built"]["max_von_mises_pa"])
	vox_disp = float(simp["as_built"]["max_displacement_m"])

	print("=" * 74)
	print("GENERATIVE LOOP — SIMP generate -> smooth RECONSTRUCT -> body-fitted VERIFY")
	print("=" * 74)
	print(f"Problem: {L}x{B}x{H} mm cantilever @ {VOXEL} mm voxels · clamp x=0 · "
	      f"{P:g} N tip load · volfrac {VOLFRAC}")
	print(f"SIMP/OC: compliance {c0:.3e} -> {c1:.3e} J (x{c0 / c1:.2f} stiffer) · "
	      f"vol_achieved {vfrac:.3f}")
	print(f"As-built VOXEL hex8 physics: peak vM {vox_vm:.3e} Pa · "
	      f"max deflection {vox_disp * 1e3:.4f} mm")

	# ---- Link 2: smooth reconstruction of the CONTINUOUS density --------------
	smooth_stl = os.path.join(work, "reconstructed.stl")
	rec = reconstruct_surface(rho_npy, smooth_stl)
	print("-" * 74)
	print(f"RECONSTRUCT (marching_cubes iso=0.5 + Taubin x{SMOOTH_ITERS}): "
	      f"tris={rec['n_triangles']} watertight={rec['watertight']} "
	      f"components={rec['n_components_before_prune']}")
	print(f"  volume {rec['volume_mm3_before_smooth']:.1f} -> {rec['volume_mm3']:.1f} mm^3 "
	      f"(smoothing shrink {rec['volume_shrink_pct']:+.2f}%) · "
	      f"x-extent [{rec['x_min_mm']:.2f}, {rec['x_max_mm']:.2f}] mm")

	# Reconstruction invariants that MUST hold (the point of the step).
	assert rec["watertight"], (
		f"reconstructed surface not watertight: {rec['open_edges']} open edges")
	assert rec["n_components_before_prune"] == 1, (
		f"reconstruction split into {rec['n_components_before_prune']} components "
		f"(kept largest, but the load path should be single) — reported, not hidden")
	assert abs(rec["volume_shrink_pct"]) < 15.0, (
		f"Taubin smoothing collapsed features: {rec['volume_shrink_pct']:+.2f}% "
		f"volume change (needs |Δ|<15%)")

	# Clamp/load planes just inside the reconstructed x-extremes (root & tip
	# frozen slabs are fully solid, so a plane 1 mm in catches their nodes).
	clamp_x = rec["x_min_mm"] + 1.0
	load_x = rec["x_max_mm"] - 1.0

	# ---- Link 3: body-fitted mesh + solve, SAME load/support ------------------
	# Bypass the runner: call mesh_stl(order=2, high_order_optimize=False) +
	# reference_fea_tet directly, in a subprocess so a gmsh abort is captured as
	# an honest failure (not a tool crash). First positive-Jacobian solve wins.
	attempts = []
	tet = None
	for elem in TET_ELEM_SIZES:
		try:
			proc = subprocess.run(
				[PY_EXE, os.path.abspath(__file__), "--tet-solve", smooth_stl,
				 str(elem), str(clamp_x), str(load_x)],
				capture_output=True, text=True, timeout=TET_TIMEOUT_S,
				env={**os.environ})
			try:
				r = json.loads((proc.stdout.strip().splitlines() or ["{}"])[-1])
			except json.JSONDecodeError:
				err = (proc.stderr.strip().splitlines() or ["(no stderr)"])[-1][:110]
				r = {"ok": False, "error": f"crash rc={proc.returncode} :: {err}"}
		except subprocess.TimeoutExpired:
			r = {"ok": False, "error": f"hang(>{TET_TIMEOUT_S}s)"}
		note = "ok" if r.get("ok") else f"refused :: {str(r.get('error'))[:100]}"
		attempts.append({"elem_size_mm": elem, "status": note})
		print(f"body-fitted tet10 (straight-sided, no untangler) @ elem={elem} mm: {note}")
		if r.get("ok"):
			tet = r
			break

	print("-" * 74)

	# ---- Report + falsifiable assertions that DO hold -------------------------
	assert simp.get("ok"), "SIMP receipt not ok"
	assert c1 <= 0.9 * c0, (
		f"SIMP did not descend: compliance {c0:.3e} -> {c1:.3e} (needs <=0.9x)")
	assert abs(vfrac - VOLFRAC) <= 0.02, (
		f"SIMP volume dishonest: achieved {vfrac:.3f} vs asked {VOLFRAC}")
	assert np.isfinite(vox_vm) and vox_vm > 0 and np.isfinite(vox_disp) and vox_disp > 0, (
		f"as-built voxel physics not finite/positive: vM={vox_vm} disp={vox_disp}")

	receipt = {
		"loop": "SIMP-generate -> smooth-reconstruct -> body-fitted-verify",
		"simp": {
			"compliance_first_j": c0, "compliance_last_j": c1,
			"volfrac_achieved": vfrac,
			"as_built_voxel_hex8": {"peak_von_mises_pa": vox_vm,
			                        "max_deflection_m": vox_disp},
		},
		"reconstruction": rec,
		"body_fitted_attempts": attempts,
	}

	if tet is not None:
		# ---- LOOP CLOSED: real voxel-vs-body-fitted comparison ----------------
		bf_vm = float(tet["max_von_mises_pa"])
		bf_disp = float(tet["max_displacement_m"])
		min_jac = float(tet["min_corner_jacobian_mm3"])
		assert rec["watertight"], "reconstruction must be watertight (loop invariant)"
		assert min_jac > 0, f"body-fitted mesh has non-positive Jacobian {min_jac}"
		assert np.isfinite(bf_vm) and bf_vm > 0, f"body-fitted vM not finite/positive: {bf_vm}"
		assert np.isfinite(bf_disp) and bf_disp > 0, f"body-fitted disp not finite/positive: {bf_disp}"
		d_vm, d_disp = pct(vox_vm, bf_vm), pct(vox_disp, bf_disp)
		elem_used = next(a["elem_size_mm"] for a in attempts if a["status"] == "ok")
		caveat = ("STRAIGHT-SIDED tet10 (curvature-untangling OFF to avoid the "
		          "gmsh SIGABRT on organic topology): a real body-fitted conforming "
		          "solve, but marginally less sharp than fully-curved elements would "
		          "read — still vastly better than the voxel staircase.")
		receipt["outcome"] = "loop_closed"
		receipt["body_fitted_tet10"] = {
			"elem_size_mm": elem_used,
			"peak_von_mises_pa": bf_vm, "max_deflection_m": bf_disp,
			"n_tets": tet["n_tets"], "n_nodes": tet["n_nodes"],
			"min_corner_jacobian_mm3": min_jac,
			"caveat": caveat,
		}
		receipt["voxel_vs_bodyfitted"] = {
			"peak_von_mises_pct_diff_voxel_rel_bodyfitted": d_vm,
			"max_deflection_pct_diff_voxel_rel_bodyfitted": d_disp,
		}
		print("LOOP CLOSED — the smooth reconstruction meshes AND solves body-fitted")
		print(f"where the staircased STL could not. tet10 @ elem={elem_used} mm: "
		      f"{tet['n_tets']} tets, minJ {min_jac:.3e} mm^3 (positive).")
		print("voxel (SIMP as-built) vs body-fitted verification:")
		print(f"  peak von Mises: voxel {vox_vm:.3e} Pa   body-fitted {bf_vm:.3e} Pa "
		      f"  (voxel {d_vm:+.1f}%)")
		print(f"  max deflection: voxel {vox_disp * 1e3:.4f} mm   body-fitted "
		      f"{bf_disp * 1e3:.4f} mm  (voxel {d_disp:+.1f}%)")
		print("  (%diff MEASURED, not asserted — the whole point is to quantify the gap.)")
		print(f"  caveat: {caveat}")
	else:
		# ---- RESIDUAL LIMIT: honest report of how far the smooth surface got --
		assert attempts, "no body-fitted attempt was characterized"
		assert any(a["status"] != "ok" for a in attempts), "inconsistent attempt log"
		receipt["outcome"] = "reconstruction_ok_bodyfitted_solve_limit"
		receipt["finding"] = (
			"The smooth marching-cubes reconstruction IS watertight, single-"
			"component and Taubin-cleaned, but even the straight-sided tet10 path "
			"(mesh_stl order=2, high_order_optimize=False) refused/crashed at every "
			f"element size tried ({', '.join(str(a['elem_size_mm']) for a in attempts)}"
			" mm) — exact failures in body_fitted_attempts. No body-fitted stress "
			"number is fabricated.")
		print("RESIDUAL LIMIT — reconstruction watertight but the straight-sided tet10")
		print("  solve still refused/crashed at every element size (HONEST FINDING, no")
		print("  number fabricated). Exact failures are in the receipt. Voxel as-built")
		print(f"  stands as the SIMP estimate: vM {vox_vm:.3e} Pa, deflection "
		      f"{vox_disp * 1e3:.4f} mm.")

	print("-" * 74)
	print("RECEIPT " + json.dumps(receipt))
	print("=" * 74)
	print("OK: chain ran end-to-end and produced an honest receipt.")


if __name__ == "__main__":
	if len(sys.argv) > 2 and sys.argv[1] == "--tet-solve":
		# --tet-solve <stl> <elem> <clamp_x> <load_x>
		tet_solve_probe(sys.argv[2], float(sys.argv[3]),
		                float(sys.argv[4]), float(sys.argv[5]))
	else:
		main()
