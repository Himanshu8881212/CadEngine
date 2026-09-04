#!/usr/bin/env python3
"""ace_buckling_runner.py — one-shot hex8 linear buckling on LMCAD geometry.

Standalone job runner (``python3 tools/ace_buckling_runner.py job.json``) —
and importable as a library (:func:`run_buckling_job`) — running ACE's independent hex8 eigenvalue-buckling solver
(``physics.reference_buckling``) on geometry built by the LMCAD kernel:

  1. static pre-stress solve  K u = F  under the manifest's reference load;
  2. geometric (initial-stress) stiffness K_g assembled from the recovered
	 2x2x2 Gauss-point Cauchy stresses (K_g = integral G^T diag(S,S,S) G dV);
  3. eigenproblem  K phi = -lambda K_g phi  for the smallest POSITIVE
	 buckling load factor; critical load = lambda x applied reference load.

THE LINEAR-BUCKLING CAVEAT (also the first receipt field): this is a
BIFURCATION estimate on the perfect geometry — no imperfections, no
plasticity, no large-displacement path following. Real structures buckle
EARLIER. Treat the factor as a DESIGN-LOOP number and apply a knockdown
before trusting it: the receipt carries a recommended knockdown block
(default 0.5, i.e. design load = half the computed critical load) with the
cited sources — AISC 360 keeps only 0.877 x the elastic critical stress even
for straight steel columns (initial crookedness ~L/1000 + residual stress);
EN 1993-1-1 buckling curves knock intermediate-slenderness members down by
imperfection factors alpha = 0.13-0.76; NASA SP-8007-2020/REV 2 knocks
thin-walled cylindrical shells down to 0.32-0.65 of the linear prediction.
FDM-printed parts carry larger geometric imperfections and anisotropy than
any of those calibration sets, hence the flat 0.5 recommendation — and for
shell-like modes (inspect the mode!) even 0.5 can be UNconservative.

CROSS-SOLVER CONSISTENCY BY CONSTRUCTION: after the buckling solve this
runner re-runs the SAME static pass through ``physics.reference_fea``
— the exact function behind tools/ace_fea_runner.py — with the same
geometry/loads/fixtures/solver settings, and ships its displacement/stress
fields as prestress receipts (prestress_disp_field.npy /
prestress_stress_field.npy). tools/test_ace_modal_buckling.py gates that
these receipts agree with an ace_fea_runner.py run of the same manifest.

Usage:  python3 ace_buckling_runner.py <job.json>

Job JSON (all geometry in mm, physics in SI):
	out_dir            REQUIRED  directory for job artifacts
	voxel_mm           REQUIRED  cubic voxel edge (mm)
	origin_mm          optional  world coord of grid node (0,0,0); default [0,0,0]
	GEOMETRY, one of:
	  ops + solid + shape [+ supersample=2]   LMCAD JSON ops via
											  physics.sampling.sample_part
	  npy                                     absolute path of an existing
											  (nx,ny,nz) float density .npy
	  stl + shape                             watertight STL, parity-filled onto
											  the job grid via tools/voxelize_stl.py
	regions            optional  [{kind: frozen|fixed|design|void, selector}]
	material           REQUIRED  "PLA"-style key (tools/materials/*.json) or a
								 pasted {youngs_modulus_pa, poisson
								 [, density_kg_m3 — needed for body loads]}
	fixtures           REQUIRED  [{kind: clamped|pinned|slider, region_selector,
								   dof_constrained?}]
	loads              REQUIRED  the reference load case —
								 [{kind: point|body|pressure, magnitude,
								   direction (unit 3-vec, point/body only),
								   region_selector}]; moment loads are NOT
								 applied (C0 hex8 has no rotational DOFs — the
								 solver notes this and skips them); an all-zero
								 load is REFUSED, not returned as lambda=inf
	n_modes            optional  number of lowest positive factors, default 4
	knockdown          optional  override the recommended factor (0 < k <= 1)
	direct_solver_max_dof  optional  forwarded to the prestress reference_fea
								 pass (default 0 = Jacobi-CG, matching
								 ace_fea_runner so the receipts are comparable)

Output contract: the LAST non-empty stdout line is ONE JSON object; all
logging goes to stderr. Success => {ok:true, caveat, load_factors,
buckling_load_factor, applied_reference_load_N, critical_load_N,
knockdown {recommended_factor, design_critical_load_n, why, sources},
prestress {max_von_mises_pa, max_displacement_m, tip_displacement_m,
disp_field_npy, stress_field_npy, method}, n_active_elements, n_dof,
n_free_dof, method, fixtures/loads receipts, notes, timings_s, provenance
envelope}; any failure — including the solver's honest refusals (no load, no
compressive stress anywhere, no positive eigenvalue) => {ok:false, error}
and exit 2 (a refused analysis) or exit 1 (a broken request) — see THE WIRE
+ EXIT CONTRACT below. The JSON line and the exit code now AGREE.

Honest error band, measured (tools/test_ace_modal_buckling.py re-proves it
every run): Euler clamped-free column 45x4.5x3 mm, closed form at the
solver's effective length L - voxel (a plane clamp fixes the whole first
element layer): voxel 0.75 -> +6.3%, voxel 0.5 -> +3.4%, voxel 0.375 ->
+2.2% (observed order ~1.5, converging DOWN toward the analytic value —
linear buckling on a stiff hex8 mesh over-predicts). ACE's own docstring
says 10-30% high on coarse meshes; these slender-beam cases measure better.

THE WIRE + EXIT CONTRACT (shared; see tools/_receipt.py for the full rules):
    python3 <runner>.py <job.json> [--out PATH]
  The LAST non-empty stdout line is ONE JSON receipt; all logging goes to
  stderr. The exit code AGREES with the receipt, always:
    exit 0  ok:true   analysis ran, receipt usable
    exit 1  ok:false   the tool could not run the request (usage, unreadable
                       job, internal error) — NO analysis was performed
    exit 2  ok:false   the tool RAN and REFUSED, or the analysis failed
  `error_kind` is a machine-matchable slug (`refusal.*`, `timeout`, `killed.*`,
  `internal`, `usage`, `receipt_path_conflict`). CHANGED 2026-08-08: this runner
  used to exit 0 on ok:false by design. Parsing `ok` still works and is still
  correct; `$?` now works too. `LMCAD_RUNNER_EXIT=legacy` or a job
  `"legacy_exit_zero": true` restores exit-0-always and records the opt-out in
  `exit_contract.mode`.
  `--out PATH` writes the receipt atomically (temp+rename) so an interrupted run
  can never leave a zero-byte file where a good receipt was; a job-level
  `receipt` key that disagrees with `--out` is REFUSED, not silently preferred.
  `LMCAD_RECEIPT_DRY_RUN=1` suppresses every on-disk write (safe what-if runs).
  `"wall_budget_s"` (or `LMCAD_WALL_BUDGET_S`), SIGTERM and SIGINT all produce
  an honest ok:false receipt instead of a vanished run.
  `determinism` names the receipt's wall-clock fields and carries `core_digest`,
  a sha256 over the rest at 12 significant figures — compare THAT between runs,
  never the receipt bytes.
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # tools/: the shared contracts + the layout map
import _layout  # noqa: E402
_layout.add_import_paths()  # tools/, tools/analyzers, tools/publish — sibling-style imports keep working after the 2026-09-02 move
from _ace import (  # noqa: E402  — importing runs the boot side effects (physics package on path, kernel-api env)
	PHYSICS_INSTALL_HINT,
	apply_warnings,
	build_region_kind,
	compression_check,
	determinism_block,
	emit,
	finish,
	load_geometry,
	load_job,
	log,
	mesh_resolution_receipt,
	mesh_resolution_warning,
	provenance_fields,
	refuse_empty_selectors,
	refuse_tensile_load_case,
	resolve_material,
	run_cli,
	selector_catch_audit,
	validated_range_check,
	validated_range_warning,
)

ANALYZER_VERSION = "reference_buckling/hex8/v2+prestress-receipts"

CAVEAT = (
	"LINEAR (eigenvalue) buckling: a bifurcation estimate on the perfect "
	"geometry — no imperfections, no plasticity, no large-displacement path. "
	"Real structures buckle EARLIER; this is an UPPER bound and a DESIGN-LOOP "
	"number. Apply the knockdown block before trusting critical_load_N."
)

KNOCKDOWN_DEFAULT = 0.5
KNOCKDOWN_WHY = (
	"flat 0.5 (design load = half the linear critical load) for FDM-printed "
	"members: printed parts carry larger geometric imperfection + anisotropy "
	"than the steel/aerospace calibration sets behind the cited standards, "
	"which already knock ideal elastic buckling down to 0.877 (straight steel "
	"columns), 0.24-0.87 equivalent (EN 1993-1-1 curves at intermediate "
	"slenderness), and 0.32-0.65 (thin cylindrical shells). Shell-like modes "
	"(inspect the mode shape) may need MORE than 0.5 — use SP-8007 class "
	"knockdowns there."
)
KNOCKDOWN_SOURCES = [
	"AISC 360 §E3: elastic-range column strength Fcr = 0.877·Fe (initial "
	"crookedness ~L/1000 + residual stresses on the Euler load)",
	"EN 1993-1-1 §6.3.1.2: buckling curves a0-d, imperfection factors "
	"alpha = 0.13-0.76 (chi reduction vs the elastic critical load)",
	"NASA SP-8007-2020/REV 2 (Buckling of Thin-Walled Circular Cylinders): "
	"empirical knockdown factors ~0.32-0.65 for axially compressed cylinders",
]


def stl_to_npy(job: dict, out_dir: Path) -> None:
	"""Bridge a ``stl`` geometry key to ``npy`` via tools/voxelize_stl.py
	(subprocess; last stdout line is the receipt). Twin of the helper in
	ace_modal_runner.py — kept per-runner, the runners are standalone."""
	if not job.get("stl"):
		return
	if "shape" not in job:
		raise ValueError("stl geometry needs an explicit grid: provide 'shape' [nx,ny,nz]")
	vox_job = {
		"stl": job["stl"],
		"origin_mm": list(job.get("origin_mm", (0.0, 0.0, 0.0))),
		"voxel_mm": float(job["voxel_mm"]),
		"shape": [int(n) for n in job["shape"]],
		"out": str(out_dir / "voxelized.npy"),
	}
	job_file = out_dir / "voxelize_job.json"
	job_file.write_text(json.dumps(vox_job), encoding="utf-8")
	tool = Path(__file__).resolve().parent / "voxelize_stl.py"
	proc = subprocess.run([sys.executable, str(tool), str(job_file)],
						  capture_output=True, text=True)
	lines = [ln for ln in proc.stdout.splitlines() if ln.strip()]
	receipt = json.loads(lines[-1]) if lines else {"ok": False, "error": "no receipt"}
	if not receipt.get("ok"):
		raise ValueError(f"voxelize_stl failed: {receipt.get('error', proc.stderr[-300:])}")
	log(f"voxelized {job['stl']} -> {vox_job['out']} ({receipt.get('solid_voxels')} solid voxels)")
	job["npy"] = vox_job["out"]


def selector_receipts(job: dict, rho, kind, voxel: float, origin):
	"""Per-selector node-count receipts (same recipe as ace_fea_runner's,
	including the suspiciously-broad load note)."""
	from physics.fea import _occupancy
	from physics.selectors import element_mask_to_node_ids, resolve_selector

	occ = _occupancy(rho, kind, simp_floor=None)
	n_active = int(occ.sum())

	def count(entry):
		sel = entry.get("region_selector", {"type": "all"})
		mask = resolve_selector(sel, rho.shape, voxel, origin) & occ
		return int(mask.sum()), int(element_mask_to_node_ids(mask).size)

	fixtures, loads, notes = [], [], []
	for fx in job.get("fixtures", []) or []:
		_, nodes = count(fx)
		fixtures.append({"kind": fx.get("kind"), "nodes_or_elements": nodes})
	for li, ld in enumerate(job.get("loads", []) or []):
		elems, nodes = count(ld)
		loads.append({
			"kind": ld.get("kind"),
			"nodes_or_elements": nodes,
			"magnitude": ld.get("magnitude"),
		})
		if n_active > 0 and elems > 0.30 * n_active:
			notes.append(
				f"load[{li}] ({ld.get('kind')}): selector catches {elems}/{n_active} "
				f"active elements ({elems / n_active:.0%}) — suspiciously broad — "
				f"verify the selector"
			)
	return fixtures, loads, notes


def _yield_mpa_if_known(job_material) -> float | None:
	"""Yield strength when the job named a materials-registry key (the fea
	material dict deliberately has no strength fields)."""
	if isinstance(job_material, str):
		try:
			import materials
			return float(materials.get(job_material).yield_mpa)
		except Exception:  # noqa: BLE001 — a yield note must never sink a solve
			return None
	return None


def run_buckling_job(job: dict) -> dict:
	"""Run one buckling job dict (schema in the module docstring); return the
	receipt payload. Raises on the solver's honest refusals (zero load, no
	compressive stress, no positive eigenvalue, no fixtures) — main() turns
	those into the {ok:false} JSON line. AI-callable: the benchmark suite
	(tools/test_ace_modal_buckling.py) imports and pins this function."""
	import numpy as np
	from physics import reference_buckling
	from physics.fea import reference_fea

	job = dict(job)
	yield_mpa = _yield_mpa_if_known(job.get("material"))
	job["material"] = resolve_material(job["material"])  # Unit 3: single materials source
	out_dir = Path(job["out_dir"])
	out_dir.mkdir(parents=True, exist_ok=True)
	stl_to_npy(job, out_dir)

	rho, origin, voxel, sample_s = load_geometry(job, out_dir)
	kind = build_region_kind(job, rho.shape, voxel, origin)

	# --- ADMISSIBILITY, BEFORE the solve (T13) --------------------------------
	# Refusals are cheap and must fire before minutes of eigensolve, and before
	# any receipt that could be mistaken for a design number exists.
	from physics.fea import _occupancy
	occ = _occupancy(rho, kind, simp_floor=None)
	catch = selector_catch_audit(job, occ, voxel, origin)
	refuse_empty_selectors(catch)
	comp = compression_check(job, occ, voxel, origin)
	refuse_tensile_load_case(comp)
	mesh_res = mesh_resolution_receipt(occ, voxel)
	vrange = validated_range_check(job, "tools/manifests/ace_buckling.manifest.json")

	t0 = time.monotonic()
	res = reference_buckling(
		rho, kind, voxel, job["material"],
		job.get("loads", []), job.get("fixtures", []),
		n_modes=int(job.get("n_modes", 4)),
		origin_mm=origin,
	)
	buckling_s = time.monotonic() - t0

	# --- prestress receipts: the SAME static pass through reference_fea -------
	# (the exact function behind ace_fea_runner.py, same solver default) so the
	# pre-buckling state is inspectable and cross-checkable against ace_fea.
	t0 = time.monotonic()
	pre = reference_fea(
		rho, kind, voxel, job["material"],
		job.get("loads", []), job.get("fixtures", []),
		origin_mm=origin,
		direct_solver_max_dof=int(job.get("direct_solver_max_dof", 0)),
	)
	prestress_s = time.monotonic() - t0
	pre_disp_npy = out_dir / "prestress_disp_field.npy"
	pre_stress_npy = out_dir / "prestress_stress_field.npy"
	np.save(pre_disp_npy, pre["disp_field"])
	np.save(pre_stress_npy, pre["stress_field"])

	notes = list(res["notes"])
	try:
		fixtures, loads, broad = selector_receipts(job, rho, kind, voxel, origin)
		notes.extend(broad)
	except Exception as exc:  # noqa: BLE001 — receipts must never sink a good solve
		fixtures, loads = [], []
		notes.append(f"selector receipts unavailable: {type(exc).__name__}: {exc}")

	# --- knockdown recommendation (the design-loop number) --------------------
	kd = float(job.get("knockdown", KNOCKDOWN_DEFAULT))
	if not (0.0 < kd <= 1.0):
		raise ValueError(f"knockdown must be in (0, 1], got {kd}")
	critical = res["critical_load_n"]
	knockdown = {
		"recommended_factor": kd,
		"design_critical_load_n": (kd * critical) if critical is not None else None,
		"why": KNOCKDOWN_WHY,
		"sources": KNOCKDOWN_SOURCES,
	}

	# --- yield-before-buckling honesty note -----------------------------------
	# At the critical load the pre-stress field scales by lambda; if that
	# already exceeds yield, elastic buckling is NOT the governing failure.
	if yield_mpa is not None and pre["max_von_mises_pa"] > 0:
		vm_at_critical = res["buckling_load_factor"] * pre["max_von_mises_pa"]
		if vm_at_critical >= yield_mpa * 1e6:
			notes.append(
				f"material would YIELD before elastic buckling: von Mises at the "
				f"critical load ~ {vm_at_critical / 1e6:.1f} MPa >= yield "
				f"{yield_mpa:.1f} MPa — the linear factor is not the governing "
				f"failure mode; check strength (ace_fea) first.")

	payload = {
		"ok": True,
		"caveat": CAVEAT,
		"load_factors": res["buckling_load_factors"],
		"buckling_load_factor": res["buckling_load_factor"],
		"applied_reference_load_N": res["applied_reference_load_n"],
		"critical_load_N": critical,
		"knockdown": knockdown,
		"prestress": {
			"max_von_mises_pa": pre["max_von_mises_pa"],
			"max_displacement_m": pre["max_displacement_m"],
			"tip_displacement_m": pre["tip_displacement_m"],
			"disp_field_npy": str(pre_disp_npy),
			"stress_field_npy": str(pre_stress_npy),
			"method": pre["method"],
		},
		"n_modes": res["n_modes"],
		"n_active_elements": res["n_active_elements"],
		"n_dof": res["n_dof"],
		"n_free_dof": res["n_free_dof"],
		"method": res["method"],
		"fixtures": fixtures,
		"loads": loads,
		"selector_count_unit": "nodes",
		"selector_catch_audit": catch,
		"compression_check": comp,
		"mesh_resolution": mesh_res,
		"validated_range": vrange,
		"notes": notes,
		"timings_s": {"sample_s": round(sample_s, 3),
					  "buckling_s": round(buckling_s, 3),
					  "prestress_s": round(prestress_s, 3)},
	}
	if comp.get("verdict") == "tensile" and comp.get("override"):
		payload["notes"].append(
			"OVERRIDE: allow_tensile_load_case is set — this load case has NO "
			"compressive load path and the factor below is not a design margin.")
	apply_warnings(payload, job, [
		mesh_resolution_warning(mesh_res),
		validated_range_warning(vrange),
	])
	# Provenance envelope, added alongside the scalar fields (Rule 4).
	payload.update(provenance_fields(
		job, res, analyzer_name="ace_buckling", analyzer_version=ANALYZER_VERSION,
		values={"critical_load_N": res["critical_load_n"],
				"buckling_load_factor": res["buckling_load_factor"]},
		manifest_ref="tools/manifests/ace_buckling.manifest.json",
		validation_applicability=vrange))
	payload["determinism"] = determinism_block(
		payload, nondeterministic_paths=["timings_s"],
		solver_note=("dense/sparse symmetric eigensolve (Cholesky reduction + "
					 "ARPACK-class iteration): reproducible to ~1e-12 relative, "
					 "NOT to the bit — the reduction order is not pinned. Geometry, "
					 "mesh, DOF counts and all integer fields ARE bit-identical."))
	return payload


def main() -> None:
	job, out = load_job()
	finish(run_buckling_job(job), job=job, tool="ace_buckling", out=out)


if __name__ == "__main__":
	run_cli("ace_buckling", main, install_hint=PHYSICS_INSTALL_HINT)
