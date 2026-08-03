#!/usr/bin/env python3
"""ace_contact_runner.py — geometrically-nonlinear beam + rigid-obstacle contact.

The shop's FIFTH permanent solver (registry card: tools/solvers/contact.md;
precedents: tools/solvers/ace_fea.md linear-elastic, tools/solvers/thermal.md
in-house). Written in-house, NumPy only — no ACE package dependency — and
guilty until its benchmark gates (tools/test_ace_contact_fatigue.py) are green.

WHY A BEAM AND NOT THE HOUSE VOXEL FEA
--------------------------------------
`tools/ace_fea_runner.py` (ACE `engine.verify.reference_fea`) is LINEAR: small
strain AND small displacement, no contact. The features this solver exists for
— snap-fit cantilevers, living hinges, latch arms, press-fit lips — routinely
deflect 3-5 mm on a 15 mm arm (~10-30% of span, tip rotations 10-30 deg). At
that amplitude linear FEA over-predicts stiffness because it never shortens the
moment arm. Retro-fitting geometric nonlinearity + contact onto the hex8 voxel
grid would need a 3-D contact search and ~1e5 DOF Newton solves per load step
to resolve a 1.5 mm-thick arm; a planar corotational BEAM gets the same physics
in <300 DOF with an exact closed-form benchmark (the elastica). So this runner
does NOT import ACE — it is a different discretization for a different job, and
the card says so. Use ace_fea for 3-D stress fields, this for load PATHS with
large rotation and/or contact.

Usage:  python3 ace_contact_runner.py <job.json>

Physics: planar (2-D) corotational Euler-Bernoulli beam. Exact for arbitrarily
large RIGID-BODY rotations with small LOCAL strains (the snap-fit regime for
PLA/PETG: strains under a few %). Element internal force from Crisfield's
corotational formulation:
    local deformations  u_bar = Ln - L0,  th_i = theta_i - alpha,
                        th_j = theta_j - alpha,   alpha = beta - beta0
    local forces        N = EA/L0 * u_bar
                        M1 = EI/L0 (4 th_i + 2 th_j),  M2 = EI/L0 (2 th_i + 4 th_j)
    f_int = B^T q  with  B = [r^T ; e3^T - z^T ; e6^T - z^T]
                        r = [-c,-s,0, c,s,0]^T,  zz = [s,-c,0, -s,c,0]^T,  z = zz/Ln
    K_t   = B^T k_l B + (N/Ln) zz zz^T + ((M1+M2)/Ln^2) (r zz^T + zz r^T)
The geometric (second) group is what makes the nonlinear answer STIFFER than
the linear one under a transverse tip load — the physics sign gate 2 pins.

Contact: node-to-analytic-rigid-surface PENALTY (plane / cylinder / piecewise-
linear profile "terrain"), optional regularized Coulomb friction (elastic slip).
Normal force  p_n = kappa * max(0, -gap) along the outward surface normal;
the penetration you accept IS p_n/kappa, reported every step, never hidden.

Newton-Raphson with load/motion incrementation (lambda 0 -> 1 in `steps.n`), a
Crisfield backtracking line search on the ENERGY merit |du . R| (the residual
norm is NOT a usable merit here — see newton_step), and a LOUD failure: a step
that does not converge raises -> {ok:false, error} + exit 1. There is no silent
last-iterate.

Job JSON (lengths mm, forces N, moments N*mm, stresses reported in Pa AND MPa):
    out_dir            REQUIRED  directory for curve .npy output
    beam               REQUIRED  {length_mm, n_elements} | {nodes_mm: [[x,y],..]}
                                 + section: {width_mm, thickness_mm}
                                        or {width_mm, root_thickness_mm,
                                            tip_thickness_mm}   (linear taper)
    material           REQUIRED  "PLA" (tools/materials.py key) or
                                 {youngs_modulus_pa [, yield_mpa, ultimate_mpa]}
    supports           REQUIRED  [{node, dofs:{ux?,uy?,rz?}, ramped?:false}]
                                 node = "root" | "tip" | int | {at_mm: x}
                                 dof values in mm / rad; ramped => value * lambda
                                 (this is the prescribed-displacement path)
    loads              optional  [{node, fx_n?, fy_n?, mz_nmm?}] — scaled by
                                 lambda. Fixed direction (NOT follower loads).
    obstacles          optional  [{kind, ..., penalty_n_per_mm,
                                   friction?: {mu, k_t_n_per_mm},
                                   motion?: {dir:[dx,dy], travel_mm},
                                   nodes?: ["tip"|int|...]}]
                       kind "plane"    point_mm:[x,y], normal:[nx,ny]
                                       (solid side = where (x-p0).n >= 0)
                       kind "cylinder" center_mm:[x,y], radius_mm,
                                       side:"outside"|"inside"
                       kind "profile"  points_mm:[[x,y],..] (x ascending),
                                       side:"above"|"below" — a rigid terrain;
                                       outside its x-range the end height is
                                       extended horizontally
    steps              optional  {n: 20, max_iter: 30, tol: 1e-8,
                                  tol_energy: 1e-14, line_search: true,
                                  min_alpha: 1/1024, ls_eta: 0.8}
                                 tol is RELATIVE to the step's opening unbalance
                                 (frozen per step, so the history cannot be
                                 renormalised into looking converged);
                                 tol_energy is Bathe's energy ratio — see the
                                 note in newton_step for why both exist
    linear_reference   optional  true (default) — also solve K_0 u = F once at
                                 lambda=1 (initial tangent, contact IGNORED) so
                                 the receipt shows the linear vs nonlinear gap

Output contract: mirrors ace_thermal_runner — the LAST non-empty stdout line is
ONE JSON receipt; all logging goes to stderr. `curve.npy` (float64,
(n_steps+1, n_cols) C-order, column names in receipt `curve_columns`) lands in
out_dir. Failure = {ok:false, error} + **exit 1**.

Honest limits (also in tools/solvers/contact.md): planar only; Euler-Bernoulli
(no transverse shear — beams under L/t ~ 10 read a few % stiff); contact is
NODE-based, so contact-patch resolution equals the node spacing; friction is a
regularized (elastic-slip) Coulomb model with an approximate slipping tangent;
quasi-static (no dynamics, so the snap-back "click" energy release is not
resolved); isotropic material, printed-layer anisotropy is NOT in the solve.
"""
from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS_DIR))

ANALYZER_VERSION = "ace_contact/corotational-beam2d-penalty/v1"

CURVE_COLUMNS = [
	"lambda",
	"obstacle_travel_mm",      # travel of obstacle 0 (0.0 if none / no motion)
	"insertion_force_n",       # actuator force along obstacle 0's motion dir
	"total_normal_force_n",    # sum |p_n| over every obstacle
	"tip_ux_mm", "tip_uy_mm",
	"max_disp_mm",
	"max_abs_stress_mpa",
	"max_penetration_mm",
	"n_contact_nodes",
	"newton_iters",
	"residual_norm_n",
]


def log(msg: str) -> None:
	print(msg, file=sys.stderr, flush=True)


def emit(payload: dict) -> None:
	print(json.dumps(payload), flush=True)


class JobError(ValueError):
	"""A manifest/physics refusal with a user-actionable message."""


class ConvergenceError(RuntimeError):
	"""Newton failed. NEVER downgraded to a warning — the last iterate is not
	an equilibrium state and reporting it would be a fabricated answer."""


# ---------------------------------------------------------------------------
# Material
# ---------------------------------------------------------------------------
def resolve_structural_material(job_material) -> dict:
	"""Resolve to {youngs_modulus_mpa, name?, hash?, yield_mpa?, ultimate_mpa?}.

	A STRING is a key into tools/materials.py (the one source of truth). A null
	modulus is refused loudly — the solver never invents an E."""
	import numpy as np

	if isinstance(job_material, str):
		import materials
		mat = materials.get(job_material)
		mech = mat.record.get("mechanical", {})
		e_pa = mech.get("youngs_modulus_pa")
		out = {"name": mat.name, "hash": mat.hash, "youngs_modulus_pa": e_pa,
		       "yield_mpa": mech.get("yield_mpa"), "ultimate_mpa": mech.get("ultimate_mpa")}
	elif isinstance(job_material, dict):
		out = dict(job_material)
		e_pa = out.get("youngs_modulus_pa")
	else:
		raise JobError(f"material must be a registry key string or a dict, got "
		               f"{type(job_material).__name__}")
	if e_pa is None:
		raise JobError("material youngs_modulus_pa is null/missing — fill the record; "
		               "the solver never invents a modulus")
	if not isinstance(e_pa, (int, float)) or not np.isfinite(e_pa) or e_pa <= 0.0:
		raise JobError(f"youngs_modulus_pa must be finite and > 0, got {e_pa!r}")
	out["youngs_modulus_mpa"] = float(e_pa) * 1e-6   # Pa -> N/mm^2 (internal units)
	return out


# ---------------------------------------------------------------------------
# Beam geometry + section properties
# ---------------------------------------------------------------------------
def build_beam(job, e_mpa: float):
	"""Return (X0 (n_nodes,2) mm, props dict of per-element arrays).

	props: L0, beta0, EA (N), EI (N*mm^2), area_mm2, inertia_mm4, c_mm (half
	thickness, for the extreme-fibre stress)."""
	import numpy as np

	spec = job.get("beam")
	if not isinstance(spec, dict):
		raise JobError("beam block required: {length_mm, n_elements} or {nodes_mm}")
	if spec.get("nodes_mm") is not None:
		X0 = np.asarray(spec["nodes_mm"], dtype=np.float64)
		if X0.ndim != 2 or X0.shape[1] != 2 or len(X0) < 2:
			raise JobError(f"beam.nodes_mm must be [[x,y],...] with >= 2 nodes, got shape {X0.shape}")
	else:
		length = float(spec.get("length_mm", 0.0))
		n_el = int(spec.get("n_elements", 20))
		if not (length > 0.0 and np.isfinite(length)):
			raise JobError(f"beam.length_mm must be finite and > 0, got {length}")
		if n_el < 1:
			raise JobError(f"beam.n_elements must be >= 1, got {n_el}")
		s = np.linspace(0.0, length, n_el + 1)
		X0 = np.stack([s, np.zeros_like(s)], axis=1)
	if not np.isfinite(X0).all():
		raise JobError("beam node coordinates contain non-finite values")

	n_el = len(X0) - 1
	d = X0[1:] - X0[:-1]
	L0 = np.hypot(d[:, 0], d[:, 1])
	if not (L0 > 0.0).all():
		bad = int(np.argmin(L0))
		raise JobError(f"beam element {bad} has zero length (duplicate nodes "
		               f"{X0[bad].tolist()} / {X0[bad + 1].tolist()})")
	beta0 = np.arctan2(d[:, 1], d[:, 0])

	sec = spec.get("section")
	if not isinstance(sec, dict):
		raise JobError("beam.section required: {width_mm, thickness_mm} or "
		               "{width_mm, root_thickness_mm, tip_thickness_mm}")
	w = float(sec.get("width_mm", 0.0))
	if not (w > 0.0 and np.isfinite(w)):
		raise JobError(f"beam.section.width_mm must be finite and > 0, got {w}")
	if sec.get("thickness_mm") is not None:
		t = np.full(n_el, float(sec["thickness_mm"]))
	elif sec.get("root_thickness_mm") is not None and sec.get("tip_thickness_mm") is not None:
		# linear taper evaluated at the element MIDPOINT (the standard snap-fit
		# arm: root thick, tip thin, so bending strain is spread along the arm)
		xi = (np.arange(n_el) + 0.5) / n_el
		t = float(sec["root_thickness_mm"]) * (1.0 - xi) + float(sec["tip_thickness_mm"]) * xi
	else:
		raise JobError("beam.section needs thickness_mm, or root_thickness_mm + tip_thickness_mm")
	if not (np.isfinite(t).all() and (t > 0.0).all()):
		raise JobError(f"beam.section thickness must be finite and > 0, got {t.tolist()}")

	area = w * t
	inertia = w * t ** 3 / 12.0
	props = {
		"L0": L0, "beta0": beta0,
		"EA": e_mpa * area, "EI": e_mpa * inertia,
		"area_mm2": area, "inertia_mm4": inertia, "c_mm": 0.5 * t,
		"thickness_mm": t, "width_mm": w,
	}
	return X0, props


def resolve_node(spec, X0, what: str) -> int:
	"""'root' | 'tip' | int | {'at_mm': x} -> node index."""
	import numpy as np

	n = len(X0)
	if spec == "root":
		return 0
	if spec == "tip":
		return n - 1
	if isinstance(spec, bool):
		raise JobError(f"{what}: node must be 'root'|'tip'|int|{{at_mm}}, got {spec!r}")
	if isinstance(spec, int):
		if not (-n <= spec < n):
			raise JobError(f"{what}: node index {spec} out of range for {n} nodes")
		return spec % n
	if isinstance(spec, dict) and "at_mm" in spec:
		x = float(spec["at_mm"])
		i = int(np.argmin(np.abs(X0[:, 0] - x)))
		tol = 0.51 * float(np.max(np.abs(np.diff(X0[:, 0])))) if n > 1 else 0.0
		if abs(X0[i, 0] - x) > max(tol, 1e-9):
			raise JobError(f"{what}: no node within half a node spacing of x={x} mm "
			               f"(nearest node {i} at x={X0[i, 0]}) — add nodes or move the selector")
		return i
	raise JobError(f"{what}: node must be 'root'|'tip'|int|{{at_mm}}, got {spec!r}")


# ---------------------------------------------------------------------------
# Corotational element kernel
# ---------------------------------------------------------------------------
def internal_force_and_tangent(X0, u, props, alpha_ref):
	"""Assemble f_int (n_dof,) and K_t (n_dof,n_dof) at displacement state u.

	`alpha_ref` is the per-element accumulated element rotation from the LAST
	CONVERGED state; it only un-wraps atan2's (-pi,pi] branch so rotations past
	+-180 deg stay continuous. Also returns per-element local forces
	(N, M1, M2) for stress recovery."""
	import numpy as np

	n_nodes = len(X0)
	n_dof = 3 * n_nodes
	f_int = np.zeros(n_dof)
	K = np.zeros((n_dof, n_dof))
	d = u.reshape(n_nodes, 3)
	th = d[:, 2]

	L0, beta0 = props["L0"], props["beta0"]
	EA, EI = props["EA"], props["EI"]
	loc = np.zeros((len(L0), 3))
	alpha_now = np.zeros(len(L0))

	for e in range(len(L0)):
		i, j = e, e + 1
		# Current chord as UNDEFORMED chord + displacement difference. Forming
		# (X0_j + u_j) - (X0_i + u_i) instead would round the small displacement
		# against the large absolute coordinate, and eps*|X0| (not eps*|L0|)
		# then sets the residual floor: measured 2.3e-11 N that way on the
		# gate-1 cantilever at x ~ 100 mm, 1.1e-12 N this way.
		d0x, d0y = X0[j] - X0[i]
		ddx, ddy = d[j, 0] - d[i, 0], d[j, 1] - d[i, 1]
		dx, dy = d0x + ddx, d0y + ddy
		Ln = math.hypot(dx, dy)
		if Ln <= 1e-12 * L0[e]:
			raise JobError(f"element {e} collapsed to zero length during the solve "
			               f"(Ln={Ln:.3e} mm) — the load path is not physical at this step")
		c, s = dx / Ln, dy / Ln
		# alpha = beta - beta0 through the shortest branch, then un-wrapped
		# against the last converged value so |alpha| may exceed pi.
		c0, s0 = math.cos(beta0[e]), math.sin(beta0[e])
		raw = math.atan2(c0 * s - s0 * c, c0 * c + s0 * s)
		alpha = raw + 2.0 * math.pi * round((alpha_ref[e] - raw) / (2.0 * math.pi))
		alpha_now[e] = alpha

		# Axial deformation WITHOUT catastrophic cancellation: naive (Ln - L0)
		# subtracts two nearly-equal numbers and EA/L0 amplifies the lost bits.
		# Ln^2 - L0^2 = 2 d0.dd + |dd|^2 exactly, so
		#   u_bar = (2 d0.dd + |dd|^2) / (Ln + L0)   — algebraically identical
		# to Ln - L0, numerically clean.
		u_bar = (2.0 * (d0x * ddx + d0y * ddy) + ddx * ddx + ddy * ddy) / (Ln + L0[e])
		th_i = th[i] - alpha
		th_j = th[j] - alpha
		N = EA[e] / L0[e] * u_bar
		M1 = EI[e] / L0[e] * (4.0 * th_i + 2.0 * th_j)
		M2 = EI[e] / L0[e] * (2.0 * th_i + 4.0 * th_j)
		loc[e] = (N, M1, M2)

		r = np.array([-c, -s, 0.0, c, s, 0.0])
		zz = np.array([s, -c, 0.0, -s, c, 0.0])
		z = zz / Ln
		B = np.zeros((3, 6))
		B[0] = r
		B[1] = -z
		B[1, 2] += 1.0
		B[2] = -z
		B[2, 5] += 1.0

		q = np.array([N, M1, M2])
		fe = B.T @ q
		kl = np.array([
			[EA[e] / L0[e], 0.0, 0.0],
			[0.0, 4.0 * EI[e] / L0[e], 2.0 * EI[e] / L0[e]],
			[0.0, 2.0 * EI[e] / L0[e], 4.0 * EI[e] / L0[e]],
		])
		ke = B.T @ kl @ B
		ke += (N / Ln) * np.outer(zz, zz)
		ke += ((M1 + M2) / Ln ** 2) * (np.outer(r, zz) + np.outer(zz, r))

		dofs = np.array([3 * i, 3 * i + 1, 3 * i + 2, 3 * j, 3 * j + 1, 3 * j + 2])
		f_int[dofs] += fe
		K[np.ix_(dofs, dofs)] += ke
	return f_int, K, loc, alpha_now


def element_stress_mpa(loc, props):
	"""Extreme-fibre |sigma| per element: |N|/A + max(|M1|,|M2|) c / I.

	The corotational element carries a LINEAR moment between its ends, so the
	extreme moment inside an element is at one of its ends — no interior
	sampling needed."""
	import numpy as np

	N = np.abs(loc[:, 0])
	M = np.maximum(np.abs(loc[:, 1]), np.abs(loc[:, 2]))
	return N / props["area_mm2"] + M * props["c_mm"] / props["inertia_mm4"]


# ---------------------------------------------------------------------------
# Rigid obstacles (analytic surfaces) + penalty contact
# ---------------------------------------------------------------------------
def _vec2(spec, what: str):
	import numpy as np
	v = np.asarray(spec, dtype=np.float64)
	if v.shape != (2,) or not np.isfinite(v).all():
		raise JobError(f"{what} must be a finite 2-vector [x,y], got {spec!r}")
	return v


class Obstacle:
	"""One rigid analytic surface + its penalty/friction parameters."""

	def __init__(self, idx: int, spec: dict, X0):
		import numpy as np

		self.idx = idx
		self.kind = spec.get("kind")
		if self.kind not in ("plane", "cylinder", "profile"):
			raise JobError(f"obstacles[{idx}].kind must be plane|cylinder|profile, got {self.kind!r}")
		self.kappa = float(spec.get("penalty_n_per_mm", 0.0))
		if not (np.isfinite(self.kappa) and self.kappa > 0.0):
			raise JobError(f"obstacles[{idx}].penalty_n_per_mm must be finite and > 0, "
			               f"got {spec.get('penalty_n_per_mm')!r} — a zero penalty is not a constraint")
		fr = spec.get("friction") or {}
		self.mu = float(fr.get("mu", 0.0))
		self.kt = float(fr.get("k_t_n_per_mm", self.kappa))
		if self.mu < 0.0 or not np.isfinite(self.mu):
			raise JobError(f"obstacles[{idx}].friction.mu must be finite and >= 0, got {self.mu}")
		if self.mu > 0.0 and not (np.isfinite(self.kt) and self.kt > 0.0):
			raise JobError(f"obstacles[{idx}].friction.k_t_n_per_mm must be finite and > 0")
		mo = spec.get("motion") or {}
		if mo:
			d = _vec2(mo.get("dir", [1.0, 0.0]), f"obstacles[{idx}].motion.dir")
			nrm = float(np.linalg.norm(d))
			if nrm <= 0.0:
				raise JobError(f"obstacles[{idx}].motion.dir is the zero vector")
			self.mdir = d / nrm
			self.travel = float(mo.get("travel_mm", 0.0))
			if not np.isfinite(self.travel):
				raise JobError(f"obstacles[{idx}].motion.travel_mm must be finite")
		else:
			self.mdir = np.array([1.0, 0.0])
			self.travel = 0.0

		nodes = spec.get("nodes")
		if nodes is None:
			self.nodes = np.arange(len(X0))
		else:
			self.nodes = np.array([resolve_node(n, X0, f"obstacles[{idx}].nodes") for n in nodes])
			if len(self.nodes) == 0:
				raise JobError(f"obstacles[{idx}].nodes selects zero nodes")

		if self.kind == "plane":
			self.p0 = _vec2(spec.get("point_mm"), f"obstacles[{idx}].point_mm")
			nv = _vec2(spec.get("normal"), f"obstacles[{idx}].normal")
			nn = float(np.linalg.norm(nv))
			if nn <= 0.0:
				raise JobError(f"obstacles[{idx}].normal is the zero vector")
			self.normal = nv / nn
		elif self.kind == "cylinder":
			self.center = _vec2(spec.get("center_mm"), f"obstacles[{idx}].center_mm")
			self.radius = float(spec.get("radius_mm", 0.0))
			if not (np.isfinite(self.radius) and self.radius > 0.0):
				raise JobError(f"obstacles[{idx}].radius_mm must be finite and > 0")
			self.side = spec.get("side", "outside")
			if self.side not in ("outside", "inside"):
				raise JobError(f"obstacles[{idx}].side must be outside|inside, got {self.side!r}")
		else:
			pts = np.asarray(spec.get("points_mm"), dtype=np.float64)
			if pts.ndim != 2 or pts.shape[1] != 2 or len(pts) < 2 or not np.isfinite(pts).all():
				raise JobError(f"obstacles[{idx}].points_mm must be >= 2 finite [x,y] pairs")
			if not (np.diff(pts[:, 0]) > 0.0).all():
				raise JobError(f"obstacles[{idx}].points_mm must have strictly ascending x "
				               f"(it is a height field, not a closed shape)")
			self.pts = pts
			self.side = spec.get("side", "above")
			if self.side not in ("above", "below"):
				raise JobError(f"obstacles[{idx}].side must be above|below, got {self.side!r}")

	def offset(self, lam: float):
		return self.mdir * (lam * self.travel)

	def evaluate(self, x, lam: float):
		"""Per selected node: (gap, n_hat, t_hat, s_tangential, curv_over_r).

		gap > 0 = separated. n_hat points from the surface toward the allowed
		half-space, so grad(gap) w.r.t. the node position IS n_hat for all three
		kinds. `curv_over_r` is the d n_hat/d x factor (0 for flat surfaces)."""
		import numpy as np

		off = self.offset(lam)
		p = x[self.nodes] - off
		m = len(self.nodes)
		if self.kind == "plane":
			gap = (p - self.p0) @ self.normal
			nh = np.tile(self.normal, (m, 1))
			s = (p - self.p0) @ np.array([-self.normal[1], self.normal[0]])
			curv = np.zeros(m)
		elif self.kind == "cylinder":
			d = p - self.center
			r = np.hypot(d[:, 0], d[:, 1])
			r_safe = np.maximum(r, 1e-12)
			if self.side == "outside":
				gap = r - self.radius
				nh = d / r_safe[:, None]
				curv = 1.0 / r_safe
			else:
				gap = self.radius - r
				nh = -d / r_safe[:, None]
				curv = -1.0 / r_safe
			s = self.radius * np.arctan2(d[:, 1], d[:, 0])
		else:
			xs, ys = self.pts[:, 0], self.pts[:, 1]
			xl = np.clip(p[:, 0], xs[0], xs[-1])
			y_surf = np.interp(p[:, 0], xs, ys)          # clamps outside the range
			seg = np.clip(np.searchsorted(xs, xl, side="right") - 1, 0, len(xs) - 2)
			slope = (ys[seg + 1] - ys[seg]) / (xs[seg + 1] - xs[seg])
			slope = np.where((p[:, 0] < xs[0]) | (p[:, 0] > xs[-1]), 0.0, slope)
			inv = 1.0 / np.sqrt(1.0 + slope ** 2)
			if self.side == "above":
				gap = (p[:, 1] - y_surf) * inv
				nh = np.stack([-slope * inv, inv], axis=1)
			else:
				gap = (y_surf - p[:, 1]) * inv
				nh = np.stack([slope * inv, -inv], axis=1)
			s = p[:, 0]
			curv = np.zeros(m)
		th = np.stack([-nh[:, 1], nh[:, 0]], axis=1)
		return gap, nh, th, s, curv


def contact_forces(obstacles, X0, u, lam, fric_state):
	"""External contact force vector on the beam + its tangent contribution.

	Returns (f_c, dfc_du, receipts). `receipts` carries per-obstacle penetration,
	normal force, the actuator (insertion) force along the motion direction, and
	the contact node count. dfc_du is d f_c / d u — the Newton tangent uses
	K_t - dfc_du."""
	import numpy as np

	n_nodes = len(X0)
	n_dof = 3 * n_nodes
	f_c = np.zeros(n_dof)
	dfc = np.zeros((n_dof, n_dof))
	x = X0 + u.reshape(n_nodes, 3)[:, :2]
	recs = []
	for ob in obstacles:
		gap, nh, th, s, curv = ob.evaluate(x, lam)
		active = gap < 0.0
		p_n = np.where(active, ob.kappa * (-gap), 0.0)
		f_obs = np.zeros(2)
		pen_max = float(np.max(-gap[active])) if active.any() else 0.0
		for k, node in enumerate(ob.nodes):
			if not active[k]:
				fric_state[(ob.idx, int(node))] = None
				continue
			n_hat, t_hat = nh[k], th[k]
			f_node = p_n[k] * n_hat
			blk = -ob.kappa * np.outer(n_hat, n_hat) + p_n[k] * curv[k] * (
				np.eye(2) - np.outer(n_hat, n_hat))
			if ob.mu > 0.0:
				anchor = fric_state.get((ob.idx, int(node)))
				if anchor is None:
					anchor = float(s[k])
					fric_state[(ob.idx, int(node))] = anchor
				slip = float(s[k]) - anchor
				t_trial = -ob.kt * slip
				limit = ob.mu * p_n[k]
				if abs(t_trial) <= limit:
					f_node = f_node + t_trial * t_hat
					blk = blk - ob.kt * np.outer(t_hat, t_hat)
				else:
					sgn = math.copysign(1.0, t_trial)
					f_node = f_node + sgn * limit * t_hat
					# d(mu p_n)/dx = -mu kappa n  (p_n = kappa*(-gap), grad gap = n)
					blk = blk - sgn * ob.mu * ob.kappa * np.outer(t_hat, n_hat)
			f_c[3 * node:3 * node + 2] += f_node
			dfc[3 * node:3 * node + 2, 3 * node:3 * node + 2] += blk
			f_obs -= f_node   # Newton's third law: force ON the rigid obstacle
		f_total_nodes = -f_obs
		recs.append({
			"index": ob.idx, "kind": ob.kind,
			"n_contact_nodes": int(active.sum()),
			"max_penetration_mm": pen_max,
			"total_normal_force_n": float(np.sum(p_n)),
			"force_on_obstacle_n": [float(f_obs[0]), float(f_obs[1])],
			# actuator force needed to DRIVE the obstacle along its motion dir
			"insertion_force_n": float(f_total_nodes @ ob.mdir),
			"travel_mm": float(lam * ob.travel),
		})
	return f_c, dfc, recs


# ---------------------------------------------------------------------------
# Boundary conditions + external load
# ---------------------------------------------------------------------------
DOF_NAMES = {"ux": 0, "uy": 1, "rz": 2}


def build_bcs(job, X0):
	"""-> (constrained dof indices, base values, ramped flags)."""
	import numpy as np

	sup = job.get("supports")
	if not sup:
		raise JobError("supports required: an unsupported beam has a singular stiffness "
		               "matrix (rigid-body modes) — declare at least one support")
	dofs, vals, ramped = [], [], []
	for si, spec in enumerate(sup):
		node = resolve_node(spec.get("node", "root"), X0, f"supports[{si}]")
		dd = spec.get("dofs")
		if not isinstance(dd, dict) or not dd:
			raise JobError(f"supports[{si}].dofs must be a non-empty dict of "
			               f"{{ux|uy|rz: value}}, got {dd!r}")
		ramp = bool(spec.get("ramped", False))
		for name, value in dd.items():
			if name not in DOF_NAMES:
				raise JobError(f"supports[{si}].dofs key {name!r} invalid; use ux|uy|rz")
			v = float(value)
			if not np.isfinite(v):
				raise JobError(f"supports[{si}].dofs.{name} must be finite, got {value!r}")
			d = 3 * node + DOF_NAMES[name]
			if d in dofs:
				raise JobError(f"supports[{si}]: dof {name} of node {node} constrained twice")
			dofs.append(d)
			vals.append(v)
			ramped.append(ramp)
	return np.array(dofs, dtype=int), np.array(vals), np.array(ramped, dtype=bool)


def build_loads(job, X0):
	import numpy as np

	n_dof = 3 * len(X0)
	f = np.zeros(n_dof)
	recs = []
	for li, spec in enumerate(job.get("loads") or []):
		node = resolve_node(spec.get("node", "tip"), X0, f"loads[{li}]")
		fx = float(spec.get("fx_n", 0.0))
		fy = float(spec.get("fy_n", 0.0))
		mz = float(spec.get("mz_nmm", 0.0))
		if not all(np.isfinite(v) for v in (fx, fy, mz)):
			raise JobError(f"loads[{li}] components must be finite, got {spec!r}")
		f[3 * node] += fx
		f[3 * node + 1] += fy
		f[3 * node + 2] += mz
		recs.append({"node": int(node), "fx_n": fx, "fy_n": fy, "mz_nmm": mz})
	return f, recs


# ---------------------------------------------------------------------------
# Newton driver
# ---------------------------------------------------------------------------
def newton_step(X0, props, u, lam, con, con_val, f_ext, obstacles, fric_state,
                alpha_ref, cfg):
	"""Solve one load step to equilibrium. Returns (u, iters, res, res_hist, ls_used).

	Raises ConvergenceError if the residual criterion is not met inside
	`max_iter` Newton iterations — the caller turns that into {ok:false} +
	exit 1. No silent last-iterate, ever."""
	import numpy as np

	n_dof = len(u)
	free = np.ones(n_dof, dtype=bool)
	free[con] = False
	u = u.copy()
	u[con] = con_val

	tol = float(cfg["tol"])
	max_iter = int(cfg["max_iter"])
	use_ls = bool(cfg["line_search"])
	min_alpha = float(cfg["min_alpha"])
	eta = float(cfg["ls_eta"])

	def residual(uu):
		f_int, K, loc, a_now = internal_force_and_tangent(X0, uu, props, alpha_ref)
		f_c, dfc, recs = contact_forces(obstacles, X0, uu, lam, fric_state)
		R = f_int - f_c - f_ext
		return R, K - dfc, f_int, f_c, loc, a_now, recs

	tol_e = float(cfg["tol_energy"])

	res_hist = []
	ls_used = 0
	E1 = None
	R, Kt, f_int, f_c, loc, a_now, recs = residual(u)
	# ONE reference for the whole step, computed ONCE from the step's OPENING
	# state: its unbalance, the applied load, and the forces the structure is
	# already carrying. Every term is fixed before the first iteration, so a
	# diverging iterate CANNOT inflate it (re-evaluating ||f_int|| each
	# iteration would let divergence read ~1.0 forever — a metric that hides
	# the failure). The ||f_int|| / ||f_c|| terms are what make the criterion
	# usable when a step opens with almost no unbalance — e.g. a rigid obstacle
	# sliding along its own flat plateau, where ||R_0|| is round-off and a
	# purely unbalance-relative tolerance would be unreachable by construction.
	ref = max(float(np.linalg.norm(R[free])), float(np.linalg.norm(f_ext[free])),
	          float(np.linalg.norm(f_int[free])), float(np.linalg.norm(f_c[free])), 1e-30)
	for it in range(1, max_iter + 1):
		rn = float(np.linalg.norm(R[free]))
		res_hist.append(rn / ref)
		if rn <= tol * ref:
			return (u, it - 1, rn / ref, res_hist, ls_used, f_int, f_c, loc, a_now, recs,
			        ref, "force", (None if E1 is None else 0.0))
		A = Kt[np.ix_(free, free)]
		try:
			du = np.linalg.solve(A, -R[free])
		except np.linalg.LinAlgError as exc:
			raise ConvergenceError(
				f"tangent stiffness is singular at lambda={lam:.6g}, Newton iteration {it} "
				f"({exc}) — the structure has an unconstrained mechanism or the penalty "
				f"stiffness has destroyed the conditioning") from exc
		if not np.isfinite(du).all():
			raise ConvergenceError(
				f"Newton increment is non-finite at lambda={lam:.6g}, iteration {it} — "
				f"the solve has diverged (check penalty_n_per_mm and steps.n)")
		# Bathe's ENERGY criterion (Finite Element Procedures, §8.4.4): the work
		# done by the pending increment against the current unbalance, relative
		# to the step's first such work. It exists because ||R|| MIXES UNITS —
		# forces (N) and moments (N*mm) — so on a fine mesh the moment entries
		# dominate ||R|| while ||f_ext|| is pure force, and the force ratio
		# stalls near 1e-8 by construction (measured: 1.2e-8 .. 3.1e-8 on
		# 80-160-element slender beams, entirely round-off). The energy ratio is
		# dimensionally consistent and has no such floor. Convergence = EITHER
		# criterion; both numbers go in the receipt, so neither can hide.
		E = abs(float(du @ R[free]))
		if E1 is None:
			E1 = max(E, 1e-300)
		# Crisfield LINE SEARCH on the same energy merit s(a) = du . R(u + a du),
		# NOT on ||R||. The predictor of a corotational step legitimately
		# multiplies ||R|| (measured 2300x on a healthy cantilever step) while s
		# stays small, so a residual-norm search throttles good solves — and a
		# residual-norm guard cannot even separate the healthy case from the
		# pathological one (penalty active-set chatter measured 9500x growth,
		# the same order). The pathology this exists for: a contact node that
		# OPENS at the start of a step takes a free-flight Newton step (measured
		# 1.18 mm of spring-back on the snap-fit gate), lands ~1.2 mm inside the
		# obstacle at p_n = 5.7e4 N, is thrown back out, and limit-cycles
		# forever. Backtracking on s parks it just inside the surface; the
		# contact tangent then converges in one more iteration. That needs ~10
		# halvings, hence min_alpha = 1/1024.
		s0 = max(E, 1e-300)
		alpha, best = 1.0, None
		while True:
			trial = u.copy()
			trial[free] += alpha * du
			state = residual(trial)
			sa = abs(float(du @ state[0][free]))
			if best is None or (np.isfinite(sa) and sa < best[1]):
				best = (trial, sa, state)
			ok = np.isfinite(sa) and sa <= eta * s0
			if not use_ls or ok or alpha <= min_alpha:
				if ok:
					best = (trial, sa, state)
				break
			alpha *= 0.5
			ls_used += 1
		trial, _sa, state = best
		R, Kt, f_int, f_c, loc, a_now, recs = state
		u = trial
		if E <= tol_e * E1:
			rn2 = float(np.linalg.norm(R[free]))
			res_hist.append(rn2 / ref)
			return (u, it, rn2 / ref, res_hist, ls_used, f_int, f_c, loc, a_now, recs,
			        ref, "energy", E / E1)
	rn = float(np.linalg.norm(R[free]))
	raise ConvergenceError(
		f"Newton did not converge at lambda={lam:.6g}: {max_iter} iterations, final "
		f"relative force residual {rn / ref:.6e} > tol {tol:.3e} AND energy ratio "
		f"{(E / E1 if E1 else float('nan')):.6e} > tol_energy {tol_e:.3e} (residual "
		f"history {['%.2e' % v for v in res_hist[-6:]]}). REFUSING to report the last "
		f"iterate — it is not an equilibrium state. Reduce penalty_n_per_mm, raise "
		f"steps.n, or raise steps.max_iter.")


def linear_local_forces(X0, u, props):
	"""Local (N, M1, M2) from SMALL-displacement kinematics — B evaluated at the
	UNDEFORMED configuration. Used only for the linear-reference receipt: feeding
	a linear displacement field through the corotational (finite-rotation)
	recovery would mix the two theories and report a nonsense stress."""
	import numpy as np

	d = u.reshape(len(X0), 3)
	loc = np.zeros((len(props["L0"]), 3))
	for e in range(len(props["L0"])):
		i, j = e, e + 1
		c0, s0 = math.cos(props["beta0"][e]), math.sin(props["beta0"][e])
		L0 = props["L0"][e]
		de = np.concatenate([d[i], d[j]])
		r0 = np.array([-c0, -s0, 0.0, c0, s0, 0.0])
		z0 = np.array([s0, -c0, 0.0, -s0, c0, 0.0]) / L0
		u_bar = r0 @ de
		th_i = de[2] - z0 @ de
		th_j = de[5] - z0 @ de
		loc[e] = (props["EA"][e] / L0 * u_bar,
		          props["EI"][e] / L0 * (4.0 * th_i + 2.0 * th_j),
		          props["EI"][e] / L0 * (2.0 * th_i + 4.0 * th_j))
	return loc


def linear_reference(X0, props, con, con_val, f_ext):
	"""One small-displacement solve with the INITIAL tangent at lambda=1.

	This is exactly what tools/ace_fea_runner.py-class linear analysis answers.
	Contact is IGNORED here on purpose: the point of the receipt is to show the
	designer how much the linear answer over-predicts stiffness (or ignores the
	obstacle) versus the nonlinear one."""
	import numpy as np

	n_dof = 3 * len(X0)
	u0 = np.zeros(n_dof)
	_f0, K0, _loc0, _a0 = internal_force_and_tangent(X0, u0, props, np.zeros(len(props["L0"])))
	free = np.ones(n_dof, dtype=bool)
	free[con] = False
	u = np.zeros(n_dof)
	u[con] = con_val
	rhs = f_ext[free] - K0[np.ix_(free, con)] @ con_val if len(con) else f_ext[free]
	u[free] = np.linalg.solve(K0[np.ix_(free, free)], rhs)
	return u, linear_local_forces(X0, u, props)


# ---------------------------------------------------------------------------
# Run
# ---------------------------------------------------------------------------
def run(job: dict) -> dict:
	import numpy as np

	out_dir = Path(job["out_dir"])
	out_dir.mkdir(parents=True, exist_ok=True)
	t0 = time.monotonic()

	mat = resolve_structural_material(job.get("material"))
	X0, props = build_beam(job, mat["youngs_modulus_mpa"])
	n_nodes = len(X0)
	n_dof = 3 * n_nodes
	con, con_base, con_ramped = build_bcs(job, X0)
	f_ext_full, load_recs = build_loads(job, X0)
	obstacles = [Obstacle(i, s, X0) for i, s in enumerate(job.get("obstacles") or [])]

	cfg = dict(n=20, max_iter=30, tol=1e-8, tol_energy=1e-14, line_search=True,
	           min_alpha=1.0 / 1024.0, ls_eta=0.8)
	cfg.update(job.get("steps") or {})
	n_steps = int(cfg["n"])
	if n_steps < 1:
		raise JobError(f"steps.n must be >= 1, got {n_steps}")
	if not (float(cfg["tol"]) > 0.0):
		raise JobError(f"steps.tol must be > 0, got {cfg['tol']}")
	if not (float(cfg["tol_energy"]) > 0.0):
		raise JobError(f"steps.tol_energy must be > 0, got {cfg['tol_energy']}")
	setup_s = time.monotonic() - t0

	fric_state: dict = {}
	alpha_ref = np.zeros(len(props["L0"]))
	u = np.zeros(n_dof)
	u[con] = np.where(con_ramped, 0.0, con_base)

	curve = []
	steps_rec = []
	tip = n_nodes - 1

	def snapshot(lam, iters, res, loc, recs, f_c):
		disp = u.reshape(n_nodes, 3)
		mag = np.hypot(disp[:, 0], disp[:, 1])
		sig = element_stress_mpa(loc, props)
		pen = max([r["max_penetration_mm"] for r in recs], default=0.0)
		nct = sum(r["n_contact_nodes"] for r in recs)
		curve.append([
			lam,
			recs[0]["travel_mm"] if recs else 0.0,
			recs[0]["insertion_force_n"] if recs else 0.0,
			float(sum(r["total_normal_force_n"] for r in recs)),
			float(disp[tip, 0]), float(disp[tip, 1]),
			float(mag.max()),
			float(sig.max()),
			pen, float(nct), float(iters), float(res),
		])

	# lambda = 0 reference row (already-satisfied state, no iteration)
	f0, _K0, loc0, _a0 = internal_force_and_tangent(X0, u, props, alpha_ref)
	fc0, _d0, recs0 = contact_forces(obstacles, X0, u, 0.0, dict(fric_state))
	snapshot(0.0, 0, 0.0, loc0, recs0, fc0)

	t1 = time.monotonic()
	iters_total, ls_total, by_energy = 0, 0, 0
	worst_res = 0.0
	for k in range(1, n_steps + 1):
		lam = k / n_steps
		con_val = np.where(con_ramped, lam * con_base, con_base)
		(u, iters, res, hist, ls, f_int, f_c, loc, a_now, recs, ref,
		 why, e_ratio) = newton_step(
			X0, props, u, lam, con, con_val, lam * f_ext_full, obstacles,
			fric_state, alpha_ref, cfg)
		alpha_ref = a_now                      # converged element rotations
		iters_total += iters
		ls_total += ls
		worst_res = max(worst_res, res)
		by_energy += int(why == "energy")
		# Reactions at constrained dofs: R = f_int - f_c - f_ext there.
		# Global force balance is then an IDENTITY (internal forces are
		# self-equilibrated), so `equilibrium_residual_n` measures the solve's
		# arithmetic, not the physics — which is exactly what a receipt should
		# pin: sum(reactions) + sum(contact) + sum(applied) = 0.
		R_all = f_int - f_c - lam * f_ext_full
		react = R_all[con]
		eq = np.zeros(2)
		for ax in (0, 1):
			eq[ax] = float(react[np.array([(d % 3) == ax for d in con], dtype=bool)].sum()
			               + f_c[ax::3].sum() + lam * f_ext_full[ax::3].sum())
		steps_rec.append({
			"step": k, "lambda": lam, "newton_iters": iters,
			"residual_rel": res, "residual_ref_n": ref,
			"converged_by": why,
			"energy_ratio": e_ratio,
			"residual_history_rel": [float(v) for v in hist],
			"line_search_halvings": ls,
			"obstacles": recs,
			"reactions_n": [float(v) for v in react],
			"reaction_dofs": [int(d) for d in con],
			"equilibrium_residual_n": [float(eq[0]), float(eq[1])],
		})
		snapshot(lam, iters, res, loc, recs, f_c)
	solve_s = time.monotonic() - t1

	curve = np.asarray(curve, dtype=np.float64)
	curve_path = out_dir / "curve.npy"
	np.save(curve_path, np.ascontiguousarray(curve))

	disp = u.reshape(n_nodes, 3)
	sig_nl = element_stress_mpa(loc, props)
	e_max = int(np.argmax(sig_nl))
	# PATH maxima, not just the final state: a latch that springs back to zero
	# ends the run unstressed, and the number a designer needs is the WORST
	# stress/strain anywhere on the insertion path.
	sig_col = curve[:, CURVE_COLUMNS.index("max_abs_stress_mpa")]
	i_worst = int(np.argmax(sig_col))
	payload = {
		"ok": True,
		"analyzer_version": ANALYZER_VERSION,
		"method": (f"corotational Euler-Bernoulli beam2d, Newton-Raphson, "
		           f"{n_steps} load steps, tol {cfg['tol']:.1e} (relative), "
		           f"penalty contact"),
		"n_nodes": n_nodes, "n_elements": n_nodes - 1, "n_dof": n_dof,
		"material": {k: v for k, v in mat.items()
		             if k in ("name", "hash", "youngs_modulus_pa", "yield_mpa", "ultimate_mpa")},
		"loads": load_recs,
		"nonlinear": {
			"tip_ux_mm": float(disp[tip, 0]), "tip_uy_mm": float(disp[tip, 1]),
			"tip_rz_rad": float(disp[tip, 2]),
			"max_disp_mm": float(np.hypot(disp[:, 0], disp[:, 1]).max()),
			"max_abs_stress_mpa": float(sig_nl.max()),
			"max_abs_stress_pa": float(sig_nl.max()) * 1e6,
			"max_stress_element": e_max,
			"max_stress_x_mm": float(0.5 * (X0[e_max, 0] + X0[e_max + 1, 0])),
			"max_strain": float(sig_nl.max() / mat["youngs_modulus_mpa"]),
		},
		"path_max": {
			"abs_stress_mpa": float(sig_col[i_worst]),
			"abs_stress_pa": float(sig_col[i_worst]) * 1e6,
			"strain": float(sig_col[i_worst] / mat["youngs_modulus_mpa"]),
			"at_lambda": float(curve[i_worst, 0]),
			"at_obstacle_travel_mm": float(curve[i_worst, 1]),
			"max_disp_mm": float(curve[:, CURVE_COLUMNS.index("max_disp_mm")].max()),
			"note": "worst value over the WHOLE load path (the final state may be unloaded)",
		},
		"convergence": {
			"steps": n_steps, "iterations_total": iters_total,
			"line_search_halvings_total": ls_total,
			"worst_step_residual_rel": worst_res,
			"tol_rel": float(cfg["tol"]), "tol_energy": float(cfg["tol_energy"]),
			"steps_converged_by_energy": by_energy,
			"criterion": ("EITHER ||R_free||_2 <= tol * ref, with ref = "
			              "max(||R_0||, ||f_ext||, ||f_int||, ||f_contact||) evaluated ONCE "
			              "at the step's opening state (so divergence cannot inflate it), "
			              "OR the Bathe energy ratio |du.R| <= tol_energy * |du_1.R_1| "
			              "(needed because ||R|| mixes N and N*mm). "
			              "per_step.converged_by names which fired."),
			"per_step": steps_rec,
		},
		"nodes_initial_mm": [[float(p[0]), float(p[1])] for p in X0],
		"nodes_final_mm": [[float(X0[i, 0] + disp[i, 0]), float(X0[i, 1] + disp[i, 1])]
		                   for i in range(n_nodes)],
		"curve_npy": str(curve_path),
		"curve_columns": CURVE_COLUMNS,
		"curve_shape": [int(curve.shape[0]), int(curve.shape[1])],
		"obstacles_final": steps_rec[-1]["obstacles"] if steps_rec else [],
		"timings_s": {"setup_s": round(setup_s, 4), "solve_s": round(solve_s, 4)},
	}
	if obstacles:
		ins = curve[:, CURVE_COLUMNS.index("insertion_force_n")]
		payload["insertion"] = {
			"peak_force_n": float(np.max(ins)),
			"peak_at_travel_mm": float(curve[int(np.argmax(ins)), 1]),
			"min_force_n": float(np.min(ins)),
			"final_force_n": float(ins[-1]),
			"max_penetration_mm": float(np.max(curve[:, CURVE_COLUMNS.index("max_penetration_mm")])),
			"note": ("insertion_force_n is the ACTUATOR force along obstacle[0]'s "
			         "motion direction (positive = resisting insertion). Penetration "
			         "is the penalty compliance p_n/kappa — it is reported, not hidden."),
		}
	if job.get("linear_reference", True):
		u_lin, loc_lin = linear_reference(X0, props, con,
		                                  np.where(con_ramped, con_base, con_base), f_ext_full)
		dl = u_lin.reshape(n_nodes, 3)
		sig_lin = element_stress_mpa(loc_lin, props)
		nl_uy = payload["nonlinear"]["tip_uy_mm"]
		payload["linear"] = {
			"tip_ux_mm": float(dl[tip, 0]), "tip_uy_mm": float(dl[tip, 1]),
			"max_disp_mm": float(np.hypot(dl[:, 0], dl[:, 1]).max()),
			"max_abs_stress_mpa": float(sig_lin.max()),
			"note": ("small-displacement solve with the INITIAL tangent at lambda=1, "
			         "contact IGNORED — this is what a linear FEA (ace_fea) answers"),
		}
		if abs(float(dl[tip, 1])) > 1e-14:
			payload["linear"]["nonlinear_over_linear_tip_uy"] = float(nl_uy / dl[tip, 1])
	return payload


def main() -> None:
	if len(sys.argv) != 2:
		emit({"ok": False, "error": "usage: ace_contact_runner.py <job.json>"})
		sys.exit(1)
	job = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
	emit(run(job))


if __name__ == "__main__":
	try:
		main()
	except Exception as exc:  # noqa: BLE001 — the JSON line is the contract...
		# ...and, like ace_thermal_runner, this one ALSO exits 1: a solver that
		# quietly reports a non-converged iterate is worse than no solver.
		emit({"ok": False, "error": f"{type(exc).__name__}: {exc}"})
		sys.exit(1)
