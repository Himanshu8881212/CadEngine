#!/usr/bin/env python3
"""ace_modal_runner.py — one-shot hex8 modal analysis (frequencies + mode shapes).

Standalone job runner (``python3 tools/ace_modal_runner.py job.json``)
— and importable as a library (:func:`run_modal_job`) — solving the undamped
free-vibration eigenproblem  K·phi = omega^2·M·phi  on LMCAD geometry.

STIFFNESS MACHINERY IS REUSED, NOT REPLICATED: the mesh, the hex8 element
stiffness, the lumped mass, and the fixture/selector handling are imported
from ACE's benchmark-validated solver (``engine.verify.fea``: ``_assemble_mesh``,
``_hex8_Ke``, ``_hex8_lumped_mass_diag``, ``_collect_fixed_dofs``) — the exact
matrices ``reference_fea``/``reference_modal`` assemble. Only the EIGENSOLVE
layer is local to this runner, for two stated reasons ACE cannot cover:
(1) ``reference_modal`` returns frequencies only — no eigenvectors, so no mode
shapes and no participation factors; (2) it refuses free-free systems by
design. tools/test_ace_modal_buckling.py pins this runner's frequencies
against ``reference_modal`` on an identical fixed case (agreement gated).

Mass matrix: LUMPED (row-sum / HRZ) diagonal — chosen over consistent mass
because it is positive-definite by construction, it turns the generalised
problem into a standard symmetric one via the exact scaling
A = M^-1/2 K M^-1/2 (so eigsh eigenvectors come out mass-normalised for
free), and on a regular hex grid its low-mode accuracy error is the same
order as the hex8 stiffness discretisation error itself (both push
frequencies slightly HIGH, converging down under refinement).

Eigensolve (stated method):
  fixed      A = M^-1/2 K_ff M^-1/2 is SPD -> ``scipy.sparse.linalg.eigsh``
			 shift-invert about sigma = 0, ``which='LM'`` (targets the lowest
			 modes); fallback to ``which='SA'`` then dense on failure; dense
			 ``eigh`` outright for n_free <= 400.
  free-free  A is singular (6 rigid-body modes), so sigma = 0 cannot be
			 factorised. We shift-invert about a small NEGATIVE
			 sigma = -(2*pi*1e-3*f_long)^2 with f_long = sqrt(E/rho)/(2*d_bbox)
			 (the longitudinal-wave fundamental over the bounding-box
			 diagonal — a guaranteed over-estimate of the first elastic
			 frequency scale, so 1e-3 of it sits far below every elastic
			 eigenvalue): K - sigma*M is then SPD and factorisable, and both
			 the rigid cluster (~0) and the elastic cluster map to comparable
			 shift-invert magnitudes. Eigenvalues <= 1e-6 * max(returned) are
			 classified rigid-body and reported separately.

Usage:  <ACE_PYTHON> ace_modal_runner.py <job.json>

Job JSON (all geometry in mm, physics in SI):
	out_dir            REQUIRED  directory for artifacts (mode_shape_*.npy, ...)
	voxel_mm           REQUIRED  cubic voxel edge (mm)
	origin_mm          optional  world coord of grid node (0,0,0); default [0,0,0]
	GEOMETRY, one of:
	  ops + solid + shape [+ supersample=2]   LMCAD JSON ops via
											  engine.lmcad.sample_part
	  npy                                     absolute path of an existing
											  (nx,ny,nz) float density .npy
	  stl + shape                             watertight STL, parity-filled onto
											  the job grid by invoking
											  tools/voxelize_stl.py (the house
											  STL bridge — not reimplemented)
	regions            optional  [{kind: frozen|fixed|design|void, selector}]
	material           REQUIRED  "PLA"-style key (tools/materials/*.json via
								 materials.py) or a pasted {youngs_modulus_pa,
								 poisson, density_kg_m3}; density MUST be > 0
	fixtures           [{kind: clamped|pinned|slider, region_selector (bbox/
								 plane/cylinder/sphere/all), dof_constrained?}]
								 — REQUIRED unless free_free
	free_free          optional  true = explicit free-free (unsupported
								 combination with fixtures). Absent fixtures
								 WITHOUT free_free:true is REFUSED, never
								 silently fallen back.
	n_modes            optional  number of lowest ELASTIC modes, default 6

Output contract: the LAST non-empty stdout line is ONE JSON object; all
logging goes to stderr. Success => {ok:true, frequencies_hz (elastic,
ascending), first_mode_hz, rigid_body_modes_hz, boundary, participation
(per mode: effective_mass_kg/fraction + kinetic_fraction per x/y/z),
total_active_mass_kg, mode_shapes, eigensolve, n_modes, n_active_elements,
n_dof, n_free_dof, method, fixtures receipts, notes, timings_s, provenance
envelope}; any failure => {ok:false, error} and a NONZERO exit (see THE WIRE
+ EXIT CONTRACT below).

Mode-shape artifact layout (GridField-compatible): one ``mode_shape_NN.npy``
per elastic mode in out_dir — (nx, ny, nz) float32, C-order, per-VOXEL modal
displacement magnitude (max |phi| over the element's 8 corner nodes of the
mass-normalised shape, then rescaled to max = 1.0), exact zeros in void
voxels. This is byte-layout-identical to ace_fea's disp_field.npy, i.e.
directly loadable by ``kernel_implicit::grid_field::GridField::from_npy_file``
for simulation->geometry grading.

Honest caveats: LINEAR modal analysis — no damping, no preload / stress
stiffening, no plasticity; lumped-mass hex8 is slightly STIFF. Measured on
the Euler-Bernoulli cantilever pin (60x6x3 mm, L/h = 20, closed form at the
solver's effective length L - voxel because a plane clamp fixes the whole
first element layer): see tools/test_ace_modal_buckling.py for the live
bands; frequencies over-predict by a few percent and converge downward.
Higher modes carry Timoshenko shear the EB closed form lacks (~2.7% at mode
3, L/h = 20), so mode-3 error can legitimately cross zero.

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
import math
import os
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # tools/: the shared contracts + the layout map
import _layout  # noqa: E402
_layout.add_import_paths()  # tools/, tools/analyzers, tools/publish — sibling-style imports keep working after the 2026-09-02 move
from _ace import (  # noqa: E402  — importing runs the boot side effects (ACE on path, kernel-api env)
	ACE_INSTALL_HINT,
	Refusal,
	apply_warnings,
	build_region_kind,
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
	resolve_material,
	run_cli,
	selector_catch_audit,
	validated_range_check,
	validated_range_warning,
)

ANALYZER_VERSION = "lmcad_modal/hex8-lumped-eigsh/v2 (K,M via ACE engine.verify.fea)"
METHOD = "lmcad_hex8_modal_lumped_eigsh"
RIGID_REL_TOL = 1e-6  # lambda <= RIGID_REL_TOL * max(lambda) => rigid-body
MODE_SHAPE_LAYOUT = (
	"(nx,ny,nz) float32 C-order per-voxel modal displacement magnitude "
	"(unit-max, mass-normalised shape; zeros in void) — same layout as "
	"ace_fea disp_field.npy; GridField::from_npy_file-compatible"
)


def stl_to_npy(job: dict, out_dir: Path) -> None:
	"""Bridge a ``stl`` geometry key to the ``npy`` path via tools/voxelize_stl.py.

	The house STL->occupancy bridge is invoked as a subprocess (its receipt
	contract: last stdout line is one JSON object) rather than reimplemented,
	so STL parity-fill semantics stay in exactly one file. Mutates ``job``:
	sets ``job["npy"]`` on success. (Twin of the same helper in
	ace_buckling_runner.py — kept per-runner because the runners are
	standalone stdout-receipt scripts.)
	"""
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


def _eigensolve(A, k: int, free_free: bool, sigma_neg: float, notes: list) -> tuple:
	"""Lowest-k eigenpairs of the SPD/PSD standard-form operator A.

	Returns (eigvals ascending, eigvecs matching, path_string). Fixed path:
	shift-invert sigma=0 ('LM'), 'SA' fallback, dense fallback. Free-free
	path: shift-invert about the negative sigma (see module docstring).
	Dense eigh outright for n <= 400 (faster AND exact for tiny systems).
	"""
	import numpy as np
	import scipy.sparse.linalg as spla

	n = A.shape[0]
	k = max(1, min(k, n - 1))
	if n <= 400:
		w, v = np.linalg.eigh(A.toarray())
		return w[:k], v[:, :k], "dense_eigh"
	sigma = -abs(sigma_neg) if free_free else 0.0
	try:
		w, v = spla.eigsh(A, k=k, sigma=sigma, which="LM")
		order = np.argsort(w)
		return w[order], v[:, order], f"eigsh_shift_invert(sigma={sigma:.3e})"
	except Exception as exc:  # noqa: BLE001 — fall back to a robust path
		notes.append(f"shift-invert eigensolve failed ({type(exc).__name__}); retrying which='SA'.")
	try:
		w, v = spla.eigsh(A, k=k, which="SA")
		order = np.argsort(w)
		return w[order], v[:, order], "eigsh_SA_fallback"
	except Exception as exc:  # noqa: BLE001
		notes.append(f"sparse eigensolve failed ({type(exc).__name__}); dense fallback.")
		w, v = np.linalg.eigh(A.toarray())
		return w[:k], v[:, :k], "dense_eigh_fallback"


def run_modal_job(job: dict) -> dict:
	"""Run one modal job dict (schema in the module docstring); return the
	receipt payload. Raises ValueError on refusals (no fixtures and no
	explicit free_free, zero density, degenerate mesh, ...) — main() turns
	those into the {ok:false} JSON line. AI-callable: the benchmark suite
	(tools/test_ace_modal_buckling.py) imports and pins this function.
	"""
	import numpy as np
	import scipy.sparse as sp
	from engine.verify.fea import (
		_assemble_mesh,
		_collect_fixed_dofs,
		_hex8_Ke,
		_hex8_lumped_mass_diag,
	)

	job = dict(job)
	job["material"] = resolve_material(job["material"])  # Unit 3: single materials source
	material = job["material"]
	E = float(material["youngs_modulus_pa"])
	nu = float(material["poisson"])
	density = float(material.get("density_kg_m3", 0.0))
	if density <= 0.0:
		raise ValueError(
			f"material['density_kg_m3'] must be > 0 for a mass matrix; got "
			f"{material.get('density_kg_m3')!r} — refusing to fabricate frequencies.")

	out_dir = Path(job["out_dir"])
	out_dir.mkdir(parents=True, exist_ok=True)
	stl_to_npy(job, out_dir)

	fixtures = job.get("fixtures") or []
	free_free = bool(job.get("free_free", False))
	if fixtures and free_free:
		raise ValueError("free_free:true with non-empty fixtures is contradictory — pick one.")
	if not fixtures and not free_free:
		raise ValueError(
			"no fixtures constrain any DOF and free_free was not requested — "
			"an unconstrained spectrum has 6 rigid-body modes and is usually a "
			"manifest mistake. Pass fixtures, or set free_free:true to request "
			"the free-free spectrum explicitly (never a silent fallback).")
	n_modes = int(job.get("n_modes", 6))
	if n_modes < 1:
		raise ValueError(f"n_modes must be >= 1, got {n_modes}")

	rho, origin, voxel, sample_s = load_geometry(job, out_dir)
	kind = build_region_kind(job, rho.shape, voxel, origin)

	# --- ADMISSIBILITY, BEFORE the eigensolve (T13) -------------------------
	from engine.verify.fea import _occupancy
	occ = _occupancy(rho, kind, simp_floor=None)
	catch = selector_catch_audit(job, occ, voxel, origin)
	refuse_empty_selectors(catch)
	mesh_res = mesh_resolution_receipt(occ, voxel)
	vrange = validated_range_check(job, "tools/manifests/ace_modal.manifest.json")
	h = float(voxel) * 1e-3
	notes: list[str] = []
	if job.get("simp_penalty") is not None:
		notes.append("simp_penalty ignored: modal analysis is binary-occupancy by design "
					 "(penalised stiffness + linear mass makes spurious low-rho local modes).")

	t0 = time.monotonic()
	(occ, ei, ej, ek, node_id, n_active, n_nodes, n_dof,
	 elem_node_ids, elem_dofs) = _assemble_mesh(rho, kind, voxel)
	if n_active < 2:
		raise ValueError(f"too few active elements ({n_active}) for a meaningful modal analysis.")

	# --- K and lumped M: the exact assembly reference_fea/reference_modal use --
	Ke = _hex8_Ke(E, nu, h)
	rows = np.repeat(elem_dofs, 24, axis=1).reshape(-1)
	cols = np.tile(elem_dofs, (1, 24)).reshape(-1)
	K = sp.coo_matrix((np.tile(Ke.reshape(-1), n_active), (rows, cols)),
					  shape=(n_dof, n_dof)).tocsr()
	me_diag = _hex8_lumped_mass_diag(density, h)
	m_global = np.zeros(n_dof)
	np.add.at(m_global, elem_dofs.reshape(-1), np.tile(me_diag, n_active))
	total_mass = density * (h ** 3) * n_active  # lumped mass conserves this exactly

	# --- fixtures -> free DOFs ------------------------------------------------
	if fixtures:
		fixed_dofs = _collect_fixed_dofs(fixtures, occ, node_id, rho.shape,
										 voxel, origin, notes)
		if not fixed_dofs:
			raise ValueError("fixtures matched no active DOFs — selectors miss the part; "
							 "refusing (this is not a free-free request).")
		free = np.setdiff1d(np.arange(n_dof), np.fromiter(fixed_dofs, dtype=np.int64))
	else:
		free = np.arange(n_dof)
	n_free = int(free.size)
	n_expected_rigid = 6 if free_free else 0
	if n_free <= n_modes + n_expected_rigid:
		raise ValueError(f"only {n_free} free DOFs for {n_modes} elastic modes"
						 f"{' + 6 rigid-body modes' if free_free else ''}; mesh too small.")

	# --- standard form: A = M^-1/2 K M^-1/2 (lumped M diagonal => exact) ------
	Kff = K[free][:, free].tocsc()
	mf = m_global[free]
	if not np.all(mf > 0):
		raise ValueError("a free DOF has zero lumped mass — geometry/occupancy is degenerate.")
	dinv = 1.0 / np.sqrt(mf)
	Dinv = sp.diags(dinv)
	A = (Dinv @ Kff @ Dinv).tocsc()
	A = 0.5 * (A + A.T)

	# free-free negative shift from the longitudinal-wave scale (see docstring)
	active_idx = np.stack([ei, ej, ek], axis=1)
	span_mm = (active_idx.max(axis=0) - active_idx.min(axis=0) + 1) * float(voxel)
	bbox_diag_m = float(np.linalg.norm(span_mm)) * 1e-3
	f_long = math.sqrt(E / density) / (2.0 * bbox_diag_m)
	sigma_neg = (2.0 * math.pi * 1e-3 * f_long) ** 2

	k_req = n_modes + n_expected_rigid + 4  # over-request to survive filtering
	lam, y, eig_path = _eigensolve(A, min(k_req, n_free - 1), free_free, sigma_neg, notes)
	eig_s = time.monotonic() - t0

	# back-transform: phi = M^-1/2 y  => phi^T M phi = y^T y = 1 (mass-normalised)
	phi = np.zeros((n_dof, lam.size))
	phi[free] = dinv[:, None] * y

	# --- rigid / elastic split ------------------------------------------------
	lam_max = float(np.max(np.abs(lam))) if lam.size else 1.0
	rigid_tol = max(RIGID_REL_TOL * lam_max, 1e-12)
	rigid_mask = lam <= rigid_tol
	rigid_hz = [float(math.sqrt(max(v, 0.0)) / (2 * math.pi)) for v in lam[rigid_mask]]
	if free_free and len(rigid_hz) != 6:
		raise Refusal(
			"invalid_rigid_mode_count",
			f"free-free solve found {len(rigid_hz)} rigid-body modes, expected exactly 6. "
			"The mesh may contain disconnected components or a mechanism; elastic "
			"frequencies would be ambiguously indexed, so they are not published.",
			rigid_body_modes_hz=rigid_hz,
			expected=6)
	if not free_free and rigid_hz:
		raise Refusal(
			"under_constrained_model",
			f"fixed-boundary solve still has {len(rigid_hz)} near-zero mode(s). "
			"The fixtures leave rigid motion or a mechanism; silently discarding those "
			"modes would make the reported first mode incorrect. Add sufficient independent "
			"constraints or explicitly request a valid free_free model.",
			rigid_body_modes_hz=rigid_hz)
	elastic = np.where(~rigid_mask)[0][:n_modes]
	if elastic.size == 0:
		raise ValueError("no positive elastic eigenvalues extracted — under-constrained "
						 "or degenerate system.")
	freqs = np.sqrt(lam[elastic]) / (2.0 * math.pi)

	# --- participation: Gamma_d = phi^T M r_d (phi mass-normalised) ----------
	m_node = m_global.reshape(-1, 3)[:, 0]
	participation = []
	for mi, col in enumerate(elastic):
		p = phi[:, col].reshape(-1, 3)
		gam = (m_node[:, None] * p).sum(axis=0)          # participation factors
		eff = gam * gam                                   # effective modal mass (kg)
		ke = (m_node[:, None] * p * p).sum(axis=0)        # kinetic split, sums to 1
		ke_tot = max(float(ke.sum()), 1e-300)
		participation.append({
			"mode": mi + 1,
			"f_hz": float(freqs[mi]),
			"effective_mass_kg": {a: float(eff[i]) for i, a in enumerate("xyz")},
			"effective_mass_fraction": {a: float(eff[i] / total_mass) for i, a in enumerate("xyz")},
			"kinetic_fraction": {a: float(ke[i] / ke_tot) for i, a in enumerate("xyz")},
		})

	# --- mode-shape artifacts (GridField-compatible; layout in docstring) ----
	shape_files = []
	for mi, col in enumerate(elastic):
		node_mag = np.linalg.norm(phi[:, col].reshape(-1, 3), axis=1)
		elem_mag = node_mag[elem_node_ids].max(axis=1)
		field = np.zeros(rho.shape, dtype=np.float32)
		field[ei, ej, ek] = elem_mag.astype(np.float32)
		peak = float(field.max())
		if peak > 0:
			field /= peak
		f = out_dir / f"mode_shape_{mi + 1:02d}.npy"
		np.save(f, np.ascontiguousarray(field))
		shape_files.append(str(f))

	notes.append(f"lumped (row-sum) hex8 mass, total active mass {total_mass:.4e} kg; "
				 f"standard-form eigensolve ({eig_path}) on {n_free} free DOFs.")

	res = {  # provenance convergence receipt reads method/notes/n_dof/n_active_elements
		"method": METHOD, "notes": notes, "n_dof": int(n_dof),
		"n_active_elements": int(n_active),
	}
	fixtures_receipt = fixture_receipts(job, rho, kind, voxel, origin, notes)
	payload = {
		"ok": True,
		"frequencies_hz": [float(f) for f in freqs],
		"first_mode_hz": float(freqs[0]),
		"rigid_body_modes_hz": rigid_hz,
		"boundary": "free-free" if free_free else "fixed",
		"participation": participation,
		"total_active_mass_kg": float(total_mass),
		"mode_shapes": {"layout": MODE_SHAPE_LAYOUT, "files": shape_files},
		"eigensolve": {
			"path": eig_path,
			"k_requested": int(min(k_req, n_free - 1)),
			"sigma_strategy": ("negative shift sigma=-(2pi*1e-3*f_long)^2, f_long="
							   f"{f_long:.1f} Hz (longitudinal-wave scale)") if free_free
							  else "shift-invert sigma=0 (SPD fixed system)",
		},
		"n_modes": int(freqs.size),
		"n_active_elements": int(n_active),
		"n_dof": int(n_dof),
		"n_free_dof": n_free,
		"method": METHOD,
		"fixtures": fixtures_receipt,
		"selector_count_unit": "nodes",
		"selector_catch_audit": catch,
		"mesh_resolution": mesh_res,
		"validated_range": vrange,
		"notes": notes,
		"timings_s": {"sample_s": round(sample_s, 3), "modal_s": round(eig_s, 3)},
	}
	apply_warnings(payload, job, [
		mesh_resolution_warning(mesh_res),
		validated_range_warning(vrange),
	])
	payload.update(provenance_fields(
		job, res, analyzer_name="ace_modal", analyzer_version=ANALYZER_VERSION,
		values={"frequencies_hz": payload["frequencies_hz"],
				"first_mode_hz": payload["first_mode_hz"]},
		manifest_ref="tools/manifests/ace_modal.manifest.json",
		validation_applicability=vrange))
	payload["determinism"] = determinism_block(
		payload, nondeterministic_paths=["timings_s"],
		solver_note=("ARPACK/Lanczos shift-invert eigensolve: eigenvalues reproduce to "
					 "~1e-12..1e-13 relative between identical runs, NOT to the bit "
					 "(the Krylov reduction order is not pinned). 12 significant "
					 "figures agree; compare core_digest, never receipt bytes, and "
					 "never write a cmp-based gate on a modal receipt."))
	return payload


def fixture_receipts(job: dict, rho, kind, voxel: float, origin, notes: list) -> list:
	"""Per-fixture node-count receipts (same recipe as ace_fea_runner's,
	loads omitted — modal analysis takes none). Never sinks a good solve."""
	try:
		from engine.verify.fea import _occupancy
		from engine.verify.selectors import element_mask_to_node_ids, resolve_selector

		occ = _occupancy(rho, kind, simp_floor=None)
		fixtures = []
		for fx in job.get("fixtures", []) or []:
			sel = fx.get("region_selector", {"type": "all"})
			mask = resolve_selector(sel, rho.shape, voxel, origin) & occ
			nodes = int(element_mask_to_node_ids(mask).size)
			fixtures.append({"kind": fx.get("kind"), "nodes_or_elements": nodes})
		return fixtures
	except Exception as exc:  # noqa: BLE001 — receipts must never sink a good solve
		notes.append(f"selector receipts unavailable: {type(exc).__name__}: {exc}")
		return []


def main() -> None:
	job, out = load_job()
	finish(run_modal_job(job), job=job, tool="ace_modal", out=out)


if __name__ == "__main__":
	run_cli("ace_modal", main, install_hint=ACE_INSTALL_HINT)
