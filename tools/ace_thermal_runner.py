#!/usr/bin/env python3
"""ace_thermal_runner.py — voxel heat-conduction solver (steady + transient).

The shop's SECOND permanent solver (registry card: tools/solvers/thermal.md;
the elastic precedent is tools/solvers/ace_fea.md). Written in-house, NumPy/
SciPy only — no ACE package dependency — and guilty until its benchmark gates
(tools/test_ace_thermal.py) are green.

Usage:  python3 ace_thermal_runner.py <job.json>

Physics: steady  div(k grad T) + q_vol = 0
         transient  rho*cp*dT/dt = div(k grad T) + q_vol, implicit backward
         Euler (unconditionally stable for any dt; first-order in time).
Discretization: cell-centered finite volume on the voxel grid. Interior face
conductance k*h; boundary closures — Dirichlet 2k*h (half-cell), Robin
series resistance 1/(1/h_c + h/(2k)) per unit area, Neumann q*A into the rhs.
The matrix is SPD; solved by Jacobi-preconditioned CG (rtol 1e-10 default)
or SuperLU below an opt-in DOF cap.

Job JSON (geometry in mm, physics SI, temperatures degC):
    out_dir            REQUIRED  directory for field .npy outputs
    voxel_mm           REQUIRED  cubic voxel edge (mm)
    origin_mm          optional  world coord of grid NODE (0,0,0); default [0,0,0]
                                 (same convention as ace_fea_runner)
    GEOMETRY, one of:
      npy                        path of an (nx,ny,nz) float density .npy;
                                 solid = rho >= 0.5 (binary occupancy, as ACE)
      stl + shape                binary STL parity-filled onto this exact frame
                                 by tools/voxelize_stl.py (subprocess — ONE
                                 source of truth for voxelization)
      shape + solid:"full"       full rectangular domain (benchmarks, plates)
    material           REQUIRED  string key resolved via tools/materials.py
                                 (thermal.conductivity_w_mk etc; null = loud
                                 error) or a pasted dict {k_w_mk
                                 [, density_kg_m3, cp_j_kgk]} — the latter two
                                 REQUIRED for transient runs
    bcs                REQUIRED  [{kind, box_mm:[[x0,y0,z0],[x1,y1,z1]],
                                   faces?: "any" | ["+x","-x",...], ...}]
                                 kinds: fixed_t {t_c} · flux {q_w_m2, positive
                                 INTO the solid} · convection {h_w_m2k, t_inf_c}
                                 Applied to EXPOSED voxel faces (solid next to
                                 void or domain edge) whose face CENTERS fall in
                                 box_mm (eps = 1e-6*voxel). bcs claim faces in
                                 list order — first claim wins; a bc claiming 0
                                 faces is an error. Unclaimed faces = adiabatic.
    sources            optional  [{q_w | q_w_m3, box_mm}] volumetric heat over
                                 the solid voxels whose CENTERS fall in box_mm
    probes_mm          optional  [[x,y,z], ...] — trilinear samples of the
                                 cell-centered field, clamped at the borders
                                 (identical semantics to kernel_implicit
                                 GridField::sample)
    transient          optional  {t_initial_c, dt_s, t_end_s,
                                  snapshot_times_s?: [...]}; t_end_s must be an
                                 integer multiple of dt_s; snapshot times are
                                 rounded to the nearest step (actual time
                                 reported). Absent => steady state.
    void_fill_c        optional  value written into VOID voxels of the output
                                 field (default: mean solid temperature) so the
                                 .npy stays all-finite — GridField::from_data
                                 refuses non-finite values
    solver             optional  {rtol (default 1e-10), maxiter (default 20000),
                                  direct_max_dof (default 0 = always CG)}

Output contract: mirrors the ACE runners — the LAST non-empty stdout line is
ONE JSON receipt; all logging goes to stderr. T_field.npy (steady / final)
and T_t<time>s.npy snapshots land in out_dir as C-order float32 (nx,ny,nz).

GridField hand-off (receipt key ``grid_field``): kernel_implicit::GridField
reads C-order f32 with ``origin`` = world position of SAMPLE (0,0,0). This
field is per-VOXEL (cell-centered), so the receipt's grid_field.origin_mm is
job origin_mm + voxel/2 on each axis — the CENTER of voxel (0,0,0) — ready to
pass straight to GridField::from_npy_file. (Same +h/2 shift grid_field.rs
documents for ace_fea's per-element fields.)

Failure paths: {ok:false, error} AND a nonzero exit — 2 for a JobError
refusal, 1 for a broken request (this used to be the deviation from the
ACE runners' exit-0 contract — this runner is also a shell-gate primitive;
the negative controls in test_ace_thermal.py pin the nonzero exit).

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

TOOLS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS_DIR))

ANALYZER_VERSION = "ace_thermal/fv-voxel-cg/v1"
FACE_LABELS = ("-x", "+x", "-y", "+y", "-z", "+z")


def log(msg: str) -> None:
	print(msg, file=sys.stderr, flush=True)


from _receipt import (  # noqa: E402  — the shared receipt + exit-code contract
	determinism_block,
	emit,
	finish,
	load_job,
	run_cli,
)


class JobError(ValueError):
	"""A manifest/physics refusal with a user-actionable message."""


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------
def load_geometry(job: dict, out_dir: Path):
	"""Resolve the job's geometry block to a boolean solid mask.

	Returns (solid, origin_mm, voxel_mm, sample_seconds).
	"""
	import numpy as np

	voxel = float(job["voxel_mm"])
	if not (voxel > 0.0) or not np.isfinite(voxel):
		raise JobError(f"voxel_mm must be finite and > 0, got {voxel}")
	origin = tuple(float(v) for v in job.get("origin_mm", (0.0, 0.0, 0.0)))
	t0 = time.monotonic()
	if job.get("npy"):
		rho = np.load(job["npy"]).astype(np.float32)
		if rho.ndim != 3:
			raise JobError(f"npy density must be 3-D (nx,ny,nz), got shape {rho.shape}")
		if not np.isfinite(rho).all():
			raise JobError("npy density contains non-finite values")
		log(f"loaded density grid {rho.shape} from {job['npy']}")
	elif job.get("stl"):
		shape = tuple(int(n) for n in job["shape"])
		vox_out = out_dir / "solid_fraction.npy"
		vox_job = out_dir / "voxelize_job.json"
		vox_job.write_text(json.dumps({
			"stl": job["stl"], "origin_mm": list(origin), "voxel_mm": voxel,
			"shape": list(shape), "out": str(vox_out),
		}), encoding="utf-8")
		# ONE parity-fill implementation in the shop: tools/voxelize_stl.py.
		proc = subprocess.run(
			[sys.executable, str(TOOLS_DIR / "voxelize_stl.py"), str(vox_job)],
			capture_output=True, text=True,
		)
		if proc.returncode != 0:
			raise JobError(f"voxelize_stl failed: {proc.stderr.strip() or proc.stdout.strip()}")
		rho = np.load(vox_out).astype(np.float32)
		log(f"voxelized {job['stl']} onto grid {shape}")
	elif job.get("solid") == "full":
		shape = tuple(int(n) for n in job["shape"])
		rho = np.ones(shape, dtype=np.float32)
		log(f"full solid domain {shape}")
	else:
		raise JobError("geometry required: one of npy | stl+shape | shape+solid:'full'")
	solid = rho >= 0.5  # binary as-built occupancy, same rule as ACE binary mode
	return solid, origin, voxel, time.monotonic() - t0


# ---------------------------------------------------------------------------
# Material
# ---------------------------------------------------------------------------
def resolve_thermal_material(job_material, *, need_transient: bool) -> dict:
	"""Resolve to {k_w_mk [, density_kg_m3, cp_j_kgk], name?, hash?}.

	A STRING is a key into tools/materials.py (the one source of truth); its
	thermal.conductivity_w_mk / specific_heat_j_kgk must be non-null — a null
	is refused loudly, never silently defaulted (fill the record instead).
	"""
	import numpy as np

	if isinstance(job_material, str):
		import materials
		mat = materials.get(job_material)
		th = mat.record.get("thermal", {})
		out = {
			"k_w_mk": th.get("conductivity_w_mk"),
			"density_kg_m3": mat.density_kg_m3,
			"cp_j_kgk": th.get("specific_heat_j_kgk"),
			"name": mat.name, "hash": mat.hash,
		}
		if out["k_w_mk"] is None:
			raise JobError(
				f"material '{mat.name}': thermal.conductivity_w_mk is null in "
				f"tools/materials/ — research and fill the record; the solver "
				f"never invents a conductivity"
			)
		if need_transient and out["cp_j_kgk"] is None:
			raise JobError(
				f"material '{mat.name}': thermal.specific_heat_j_kgk is null — "
				f"transient runs need cp; fill the record"
			)
	elif isinstance(job_material, dict):
		out = dict(job_material)
		if "k_w_mk" not in out:
			raise JobError("pasted material dict needs k_w_mk (W/(m*K))")
		if need_transient and (out.get("density_kg_m3") is None or out.get("cp_j_kgk") is None):
			raise JobError("transient runs need material density_kg_m3 and cp_j_kgk")
	else:
		raise JobError(f"material must be a registry key string or a dict, got {type(job_material).__name__}")

	k = out["k_w_mk"]
	if not isinstance(k, (int, float)) or not np.isfinite(k) or k <= 0.0:
		raise JobError(f"conductivity k_w_mk must be finite and > 0, got {k!r} — "
		               f"a zero/negative k is not a conductor, refusing")
	for key in ("density_kg_m3", "cp_j_kgk"):
		v = out.get(key)
		if v is not None and (not isinstance(v, (int, float)) or not np.isfinite(v) or v <= 0.0):
			raise JobError(f"material {key} must be finite and > 0, got {v!r}")
	return out


# ---------------------------------------------------------------------------
# Exposed faces + BC claiming
# ---------------------------------------------------------------------------
def exposed_faces(solid, origin, voxel):
	"""Enumerate exposed voxel faces (solid cell next to void or domain edge).

	Returns (cell_lin, dir_idx, centers_mm): parallel arrays over faces —
	linear C-order index of the owning solid cell, direction 0..5 per
	FACE_LABELS, and the world face-center coordinate.
	"""
	import numpy as np

	nx, ny, nz = solid.shape
	cells, dirs = [], []
	for axis in range(3):
		for sign in (0, 1):  # 0 = minus face, 1 = plus face
			nb_void = np.ones_like(solid)
			sl_src = [slice(None)] * 3
			sl_dst = [slice(None)] * 3
			if sign == 1:
				sl_dst[axis], sl_src[axis] = slice(None, -1), slice(1, None)
			else:
				sl_dst[axis], sl_src[axis] = slice(1, None), slice(None, -1)
			nb_void[tuple(sl_dst)] = ~solid[tuple(sl_src)]
			mask = solid & nb_void
			ijk = np.argwhere(mask)
			cells.append(ijk)
			dirs.append(np.full(len(ijk), axis * 2 + sign, dtype=np.int8))
	ijk = np.concatenate(cells, axis=0)
	dir_idx = np.concatenate(dirs, axis=0)
	centers = (ijk + 0.5) * voxel + np.asarray(origin)
	axis_of = dir_idx // 2
	off = np.where(dir_idx % 2 == 1, 0.5 * voxel, -0.5 * voxel)
	centers[np.arange(len(ijk)), axis_of] += off
	cell_lin = np.ravel_multi_index(ijk.T, solid.shape)
	return cell_lin, dir_idx, centers


def _faces_filter(spec) -> set:
	if spec in (None, "any"):
		return set(range(6))
	names = [spec] if isinstance(spec, str) else list(spec)
	out = set()
	for n in names:
		if n not in FACE_LABELS:
			raise JobError(f"faces entry {n!r} invalid; use any of {FACE_LABELS} or 'any'")
		out.add(FACE_LABELS.index(n))
	return out


def _box(spec, what: str):
	import numpy as np
	b = np.asarray(spec, dtype=np.float64)
	if b.shape != (2, 3) or not np.isfinite(b).all():
		raise JobError(f"{what}.box_mm must be [[x0,y0,z0],[x1,y1,z1]] finite, got {spec!r}")
	lo, hi = np.minimum(b[0], b[1]), np.maximum(b[0], b[1])
	return lo, hi


def claim_bc_faces(bcs, cell_lin, dir_idx, centers, voxel):
	"""Assign exposed faces to bcs in list order (first claim wins).

	Returns per-bc face index arrays. A bc claiming zero faces is an error —
	same doctrine as ACE selectors (a silent no-op BC is a wrong answer)."""
	import numpy as np

	eps = 1e-6 * voxel
	claimed = np.zeros(len(cell_lin), dtype=bool)
	per_bc = []
	for bi, bc in enumerate(bcs):
		kind = bc.get("kind")
		if kind not in ("fixed_t", "flux", "convection"):
			raise JobError(f"bcs[{bi}].kind must be fixed_t|flux|convection, got {kind!r}")
		lo, hi = _box(bc.get("box_mm"), f"bcs[{bi}]")
		allowed = _faces_filter(bc.get("faces"))
		inside = np.all(centers >= lo - eps, axis=1) & np.all(centers <= hi + eps, axis=1)
		dir_ok = np.isin(dir_idx, list(allowed))
		sel = inside & dir_ok & ~claimed
		idx = np.nonzero(sel)[0]
		if len(idx) == 0:
			raise JobError(
				f"bcs[{bi}] ({kind}) claims ZERO exposed faces — box {bc.get('box_mm')} "
				f"faces={bc.get('faces', 'any')} misses the solid surface (or every "
				f"face there was already claimed by an earlier bc)"
			)
		claimed[idx] = True
		per_bc.append(idx)
	return per_bc


# ---------------------------------------------------------------------------
# Assembly (SPD finite-volume system)
# ---------------------------------------------------------------------------
def assemble(job, solid, origin, voxel, mat):
	"""Assemble K (W/K), rhs f (W), plus bc bookkeeping for receipts.

	Returns dict with K (csr), f, dof index map, per-bc data, source power."""
	import numpy as np
	from scipy import sparse

	n = int(solid.sum())
	if n == 0:
		raise JobError("empty domain: geometry contains zero solid voxels (rho >= 0.5)")
	h_m = float(voxel) * 1e-3
	k = float(mat["k_w_mk"])

	idx = np.full(solid.shape, -1, dtype=np.int64)
	idx[solid] = np.arange(n)
	flat_idx = idx.reshape(-1)

	diag = np.zeros(n)
	rows, cols, vals = [], [], []
	g_int = k * h_m  # k*A/d with A=h^2, d=h
	for axis in range(3):
		sl_lo = [slice(None)] * 3
		sl_hi = [slice(None)] * 3
		sl_lo[axis], sl_hi[axis] = slice(None, -1), slice(1, None)
		pair = solid[tuple(sl_lo)] & solid[tuple(sl_hi)]
		ii = idx[tuple(sl_lo)][pair]
		jj = idx[tuple(sl_hi)][pair]
		np.add.at(diag, ii, g_int)
		np.add.at(diag, jj, g_int)
		rows.extend((ii, jj))
		cols.extend((jj, ii))
		vals.extend((np.full(len(ii), -g_int), np.full(len(jj), -g_int)))

	f = np.zeros(n)
	cell_lin, dir_idx, centers = exposed_faces(solid, origin, voxel)
	bcs = job.get("bcs") or []
	per_bc = claim_bc_faces(bcs, cell_lin, dir_idx, centers, voxel)

	area = h_m * h_m
	bc_data = []  # per bc: dict for receipts + energy accounting
	for bc, faces in zip(bcs, per_bc):
		dof = flat_idx[cell_lin[faces]]
		entry = {"kind": bc["kind"], "n_faces": int(len(faces)),
		         "area_mm2": round(float(len(faces)) * float(voxel) ** 2, 9),
		         "dof": dof}
		if bc["kind"] == "fixed_t":
			t_fix = float(bc["t_c"])
			if not np.isfinite(t_fix):
				raise JobError(f"fixed_t t_c must be finite, got {bc['t_c']!r}")
			g = 2.0 * k * h_m  # half-cell closure: k*A/(h/2)
			np.add.at(diag, dof, g)
			np.add.at(f, dof, g * t_fix)
			entry.update(g_w_k=g, t_c=t_fix)
		elif bc["kind"] == "convection":
			h_c = float(bc["h_w_m2k"])
			t_inf = float(bc["t_inf_c"])
			if not (np.isfinite(h_c) and h_c > 0.0) or not np.isfinite(t_inf):
				raise JobError(f"convection needs finite h_w_m2k > 0 and finite t_inf_c, got {bc!r}")
			g = area / (1.0 / h_c + h_m / (2.0 * k))  # series: film + half cell
			np.add.at(diag, dof, g)
			np.add.at(f, dof, g * t_inf)
			entry.update(g_w_k=g, t_inf_c=t_inf, h_w_m2k=h_c)
		else:  # flux
			q = float(bc["q_w_m2"])
			if not np.isfinite(q):
				raise JobError(f"flux q_w_m2 must be finite, got {bc['q_w_m2']!r}")
			np.add.at(f, dof, q * area)
			entry.update(q_w_m2=q, power_w=float(q * area * len(faces)))
		bc_data.append(entry)

	# volumetric sources
	src_power = 0.0
	vol = h_m ** 3
	cell_centers_frame = None
	for si, src in enumerate(job.get("sources") or []):
		lo, hi = _box(src.get("box_mm"), f"sources[{si}]")
		if cell_centers_frame is None:
			ijk_all = np.argwhere(solid)
			cell_centers_frame = ((ijk_all + 0.5) * voxel + np.asarray(origin), idx[solid.nonzero()])
		cc, dofs_all = cell_centers_frame
		eps = 1e-6 * voxel
		inside = np.all(cc >= lo - eps, axis=1) & np.all(cc <= hi + eps, axis=1)
		dof = dofs_all[inside]
		if len(dof) == 0:
			raise JobError(f"sources[{si}] selects zero solid voxels — box {src.get('box_mm')}")
		if src.get("q_w") is not None:
			q_cell = float(src["q_w"]) / len(dof)
		elif src.get("q_w_m3") is not None:
			q_cell = float(src["q_w_m3"]) * vol
		else:
			raise JobError(f"sources[{si}] needs q_w (total W) or q_w_m3")
		if not np.isfinite(q_cell):
			raise JobError(f"sources[{si}] power is non-finite")
		np.add.at(f, dof, q_cell)
		src_power += q_cell * len(dof)

	K = sparse.coo_array(
		(np.concatenate([np.concatenate(vals), diag]) if vals else diag,
		 (np.concatenate([np.concatenate(rows), np.arange(n)]) if rows else np.arange(n),
		  np.concatenate([np.concatenate(cols), np.arange(n)]) if cols else np.arange(n))),
		shape=(n, n)).tocsr()
	return {"K": K, "f": f, "n": n, "idx": idx, "bc_data": bc_data,
	        "src_power_w": src_power, "h_m": h_m}


def check_wellposed(job, solid, sysd, transient: bool):
	"""Steady conduction is singular without a temperature-anchoring bc on
	EVERY connected solid component (pure-Neumann leaves T defined only up to
	a constant). Refuse loudly instead of letting CG wander."""
	import numpy as np
	from scipy import ndimage

	bcs = job.get("bcs") or []
	if transient:
		if not bcs and not (job.get("sources") or []):
			raise JobError("no BCs and no sources: nothing drives the transient — refusing "
			               "(add bcs[] or sources[] to the manifest)")
		return
	if not any(bc.get("kind") in ("fixed_t", "convection") for bc in bcs):
		raise JobError(
			"steady conduction needs at least one fixed_t or convection bc "
			"(a flux-only/no-bc problem is singular: temperature is undefined "
			"up to a constant)"
		)
	labels, n_comp = ndimage.label(solid)  # 6-connectivity default structure
	if n_comp > 1:
		anchored = set()
		# dof i corresponds to the i-th solid cell in C-order (assembly numbering)
		comp_of_dof = labels.reshape(-1)[np.flatnonzero(solid.reshape(-1))]
		for bc, entry in zip(bcs, sysd["bc_data"]):
			if bc.get("kind") in ("fixed_t", "convection"):
				anchored.update(np.unique(comp_of_dof[entry["dof"]]).tolist())
		missing = [c for c in range(1, n_comp + 1) if c not in anchored]
		if missing:
			sizes = {c: int((labels == c).sum()) for c in missing}
			raise JobError(
				f"steady problem is singular: {len(missing)} of {n_comp} connected "
				f"solid components have no fixed_t/convection bc (component sizes "
				f"{sizes} voxels) — anchor every component or remove the floaters"
			)


# ---------------------------------------------------------------------------
# Linear solve
# ---------------------------------------------------------------------------
def make_solver(A, solver_cfg: dict):
	"""Return (solve(b, x0) -> (x, n_iter), method_str). SPD assumed."""
	import numpy as np
	from scipy.sparse.linalg import LinearOperator, cg, splu

	n = A.shape[0]
	rtol = float(solver_cfg.get("rtol", 1e-10))
	maxiter = int(solver_cfg.get("maxiter", 20000))
	direct_max = int(solver_cfg.get("direct_max_dof", 0))
	if direct_max and n <= direct_max:
		lu = splu(A.tocsc())

		def solve_direct(b, x0=None):
			return lu.solve(b), 0
		return solve_direct, f"superlu direct (n_dof {n} <= direct_max_dof {direct_max})"

	dinv = 1.0 / A.diagonal()
	M = LinearOperator((n, n), matvec=lambda x: dinv * x)

	def solve_cg(b, x0=None):
		count = [0]

		def cb(_xk):
			count[0] += 1
		x, info = cg(A, b, x0=x0, rtol=rtol, atol=0.0, maxiter=maxiter, M=M, callback=cb)
		if info != 0:
			raise RuntimeError(
				f"CG did not converge (info={info}, {count[0]} iters, rtol={rtol}) — "
				f"refusing to report an unconverged temperature field"
			)
		return x, count[0]
	return solve_cg, f"jacobi-cg rtol={rtol}"


# ---------------------------------------------------------------------------
# Post: energy receipts, probes, field output
# ---------------------------------------------------------------------------
def bc_powers(sysd, T):
	"""Per-bc power INTO the solid (W), from the discrete solution — the same
	conductances the solve used, so steady-state balance is an algebraic
	identity up to linear-solve residual."""
	import numpy as np

	out = []
	for entry in sysd["bc_data"]:
		if entry["kind"] == "fixed_t":
			p = float(np.sum(entry["g_w_k"] * (entry["t_c"] - T[entry["dof"]])))
		elif entry["kind"] == "convection":
			p = float(np.sum(entry["g_w_k"] * (entry["t_inf_c"] - T[entry["dof"]])))
		else:
			p = entry["power_w"]
		out.append({"kind": entry["kind"], "n_faces": entry["n_faces"],
		            "area_mm2": entry["area_mm2"], "power_w_into_solid": p})
	return out


def energy_receipt(sysd, T):
	powers = bc_powers(sysd, T)
	p_net = sum(p["power_w_into_solid"] for p in powers) + sysd["src_power_w"]
	p_in = sum(max(p["power_w_into_solid"], 0.0) for p in powers) + max(sysd["src_power_w"], 0.0)
	p_out = -sum(min(p["power_w_into_solid"], 0.0) for p in powers) - min(sysd["src_power_w"], 0.0)
	scale = max(p_in, p_out, 1e-30)
	return powers, {
		"p_in_w": p_in, "p_out_w": p_out, "p_source_w": sysd["src_power_w"],
		"residual_w": p_net, "residual_rel": abs(p_net) / scale,
	}


def probe_field(T_grid, origin, voxel, probes_mm):
	"""Clamped trilinear sample of the CELL-CENTERED field — the exact
	semantics of kernel_implicit GridField::sample with origin = voxel center."""
	import numpy as np

	out = []
	nx, ny, nz = T_grid.shape
	for p in probes_mm or []:
		q = (np.asarray(p, dtype=np.float64) - np.asarray(origin) - 0.5 * voxel) / voxel
		qc = np.clip(q, 0.0, np.array([nx - 1, ny - 1, nz - 1], dtype=np.float64))
		i0 = np.minimum(np.floor(qc), [nx - 2 if nx > 1 else 0,
		                               ny - 2 if ny > 1 else 0,
		                               nz - 2 if nz > 1 else 0]).astype(int)
		i0 = np.maximum(i0, 0)
		t = qc - i0
		i1 = np.minimum(i0 + 1, [nx - 1, ny - 1, nz - 1])
		c = 0.0
		for dx, wx in ((0, 1 - t[0]), (1, t[0])):
			for dy, wy in ((0, 1 - t[1]), (1, t[1])):
				for dz, wz in ((0, 1 - t[2]), (1, t[2])):
					ii = (i1[0] if dx else i0[0], i1[1] if dy else i0[1], i1[2] if dz else i0[2])
					c += wx * wy * wz * float(T_grid[ii])
		out.append({"p_mm": [float(v) for v in p], "t_c": c})
	return out


def field_to_grid(T, solid, void_fill):
	import numpy as np
	grid = np.full(solid.shape, float(void_fill), dtype=np.float64)
	grid[solid] = T
	return grid


def save_field(grid, path: Path):
	import numpy as np
	np.save(path, np.ascontiguousarray(grid.astype(np.float32)))


def grid_field_receipt(npy_path, origin, voxel, shape, void_fill):
	return {
		"npy": str(npy_path),
		"origin_mm": [float(o) + 0.5 * float(voxel) for o in origin],
		"cell_mm": float(voxel),
		"shape": list(int(s) for s in shape),
		"order": "C", "dtype": "float32",
		"void_fill_c": float(void_fill),
		"convention": (
			"values are per-VOXEL (cell-centered); origin_mm above is the world "
			"position of sample (0,0,0) = the CENTER of voxel (0,0,0) = job "
			"origin_mm + cell/2 per axis — pass straight to "
			"kernel_implicit GridField::from_npy_file(path, origin, cell)"
		),
	}


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def run(job: dict) -> dict:
	import numpy as np

	out_dir = Path(job["out_dir"])
	out_dir.mkdir(parents=True, exist_ok=True)
	tr = job.get("transient")
	transient = tr is not None

	solid, origin, voxel, sample_s = load_geometry(job, out_dir)
	mat = resolve_thermal_material(job.get("material"), need_transient=transient)

	t0 = time.monotonic()
	sysd = assemble(job, solid, origin, voxel, mat)
	check_wellposed(job, solid, sysd, transient)
	assemble_s = time.monotonic() - t0
	K, f, n = sysd["K"], sysd["f"], sysd["n"]
	log(f"assembled: {n} solid voxels / {n} dof, {K.nnz} nnz")

	solver_cfg = job.get("solver") or {}
	payload = {
		"ok": True,
		"mode": "transient" if transient else "steady",
		"n_solid_voxels": n, "n_dof": n,
		"material": {k: v for k, v in mat.items() if k in ("name", "hash", "k_w_mk", "density_kg_m3", "cp_j_kgk")},
		"analyzer_version": ANALYZER_VERSION,
	}

	t1 = time.monotonic()
	if not transient:
		solve, method = make_solver(K, solver_cfg)
		T, iters = solve(f)
		res_true = float(np.linalg.norm(K @ T - f) / max(np.linalg.norm(f), 1e-30))
		powers, energy = energy_receipt(sysd, T)
		payload.update(method=f"steady {method}", cg_iters=iters,
		               true_residual_rel=res_true, bc_receipts=powers, energy=energy)
		final_T = T
	else:
		dt = float(tr["dt_s"])
		t_end = float(tr["t_end_s"])
		t_init = float(tr["t_initial_c"])
		if not (dt > 0.0 and np.isfinite(dt)):
			raise JobError(f"transient.dt_s must be finite and > 0, got {dt}")
		if not (t_end > 0.0 and np.isfinite(t_end)) or not np.isfinite(t_init):
			raise JobError("transient needs finite t_end_s > 0 and finite t_initial_c")
		n_steps = int(round(t_end / dt))
		if n_steps < 1 or abs(n_steps * dt - t_end) > 1e-9 * max(t_end, 1.0):
			raise JobError(f"t_end_s ({t_end}) must be an integer multiple of dt_s ({dt})")
		c_vol = float(mat["density_kg_m3"]) * float(mat["cp_j_kgk"]) * sysd["h_m"] ** 3  # J/K per voxel
		from scipy import sparse
		A = (K + sparse.identity(n, format="csr") * (c_vol / dt)).tocsr()
		solve, method = make_solver(A, solver_cfg)

		snap_req = list(tr.get("snapshot_times_s") or [])
		snap_steps = {}
		for s in snap_req:
			step = int(round(float(s) / dt))
			if step < 1 or step > n_steps:
				raise JobError(f"snapshot time {s}s is outside (0, t_end={t_end}s]")
			snap_steps.setdefault(step, float(s))

		T = np.full(n, t_init)
		e_in_j = 0.0
		iters_total, iters_max = 0, 0
		snapshots = []
		for step in range(1, n_steps + 1):
			b = f + (c_vol / dt) * T
			T, it = solve(b, x0=T)
			iters_total += it
			iters_max = max(iters_max, it)
			powers_step = bc_powers(sysd, T)
			p_net = sum(p["power_w_into_solid"] for p in powers_step) + sysd["src_power_w"]
			e_in_j += p_net * dt  # backward-Euler flux integral (fluxes at t^{n+1})
			if step in snap_steps:
				t_now = step * dt
				vf = float(job.get("void_fill_c", float(T.mean())))
				grid = field_to_grid(T, solid, vf)
				p = out_dir / f"T_t{t_now:g}s.npy"
				save_field(grid, p)
				snapshots.append({"t_s": t_now, "requested_t_s": snap_steps[step],
				                  "npy": str(p), "void_fill_c": vf,
				                  "t_min_c": float(T.min()), "t_max_c": float(T.max())})
		e_stored_j = float(c_vol * np.sum(T - t_init))
		scale = max(abs(e_in_j), abs(e_stored_j), 1e-30)
		powers, _ = energy_receipt(sysd, T)
		payload.update(
			method=f"transient backward-euler (unconditionally stable) {method}",
			n_steps=n_steps, dt_s=dt, t_end_s=t_end,
			cg_iters_total=iters_total, cg_iters_max_per_step=iters_max,
			bc_receipts_final=powers, snapshots=snapshots,
			energy={
				"e_in_j": e_in_j, "e_stored_j": e_stored_j,
				"residual_j": e_in_j - e_stored_j,
				"residual_rel": abs(e_in_j - e_stored_j) / scale,
				"note": "discrete backward-Euler balance: boundary+source flux integral "
				        "vs stored rho*cp*dT — an identity up to linear-solve residual",
			},
		)
		final_T = T
	solve_s = time.monotonic() - t1

	void_fill = float(job.get("void_fill_c", float(final_T.mean())))
	grid = field_to_grid(final_T, solid, void_fill)
	field_path = out_dir / "T_field.npy"
	save_field(grid, field_path)

	payload.update(
		t_min_c=float(final_T.min()), t_max_c=float(final_T.max()),
		probes=probe_field(grid, origin, voxel, job.get("probes_mm")),
		grid_field=grid_field_receipt(field_path, origin, voxel, solid.shape, void_fill),
		timings_s={"sample_s": round(sample_s, 3), "assemble_s": round(assemble_s, 3),
		           "solve_s": round(solve_s, 3)},
	)
	return payload


def main() -> None:
	job, out = load_job()
	payload = run(job)
	payload["determinism"] = determinism_block(
		payload, nondeterministic_paths=["timings_s"],
		solver_note=("finite-volume conduction with a Jacobi-CG (steady) / backward-Euler "
	             "(transient) solve at a pinned tolerance: every reported field is a "
	             "deterministic function of the inputs and measured bit-identical "
	             "across re-runs. Compare core_digest, not receipt bytes."))
	finish(payload, job=job, tool="ace_thermal", out=out)


if __name__ == "__main__":
	run_cli("ace_thermal", main, refusal_types=(JobError,))
