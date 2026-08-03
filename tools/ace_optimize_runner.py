#!/usr/bin/env python3
"""ace_optimize_runner.py — SIMP topology optimization on LMCAD geometry.

Bridge runner spawned by the LMCAD MCP server (``lmcad-mcp`` tool
``ace_optimize``): the standard SIMP + density-filter + optimality-criteria
loop (top88 lineage) driven by ACE's hex8 reference FEA in SIMP mode, then an
HONEST final check — one binary-occupancy re-analysis of the thresholded
design (the as-built part, not the homogenized proxy) — and a watertight-or-
fail STL through LMCAD's gated meshing pipeline.

Usage:  <ACE_PYTHON> ace_optimize_runner.py <job.json>

Job JSON = the same geometry/grid/regions/material/fixtures/loads block as
ace_fea_runner.py (see its header), plus:
    volfrac            REQUIRED  target volume fraction of the design region (0-1)
    penalty            optional  SIMP penalty p, default 3.0
    filter_radius_vox  optional  cone filter radius in voxels, default 1.5
    max_iters          optional  default 60
    move               optional  OC move limit, default 0.2
    density_floor      optional  default 0.02
    iso                optional  threshold for the as-built check + STL, default 0.5
    time_budget_s      optional  default 600; the loop stops at 0.8x budget

Design domain = voxels whose INITIAL sampled geometry is solid (rho0 >= 0.5)
AND region kind `design`. frozen/fixed voxels are re-pinned to 1.0 and void
to 0.0 every iteration. Mark load/fixture regions `frozen` — the solver only
applies loads on ACTIVE elements, so an unprotected load region can be
optimized away.

Output contract: LAST non-empty stdout line is ONE JSON object (logging on
stderr); failure => {ok:false, error}, exit 0 — the JSON is the contract.
Produces out_dir/final_rho.npy (the filtered physical density) and the gated
STL. The STL is a MESH ONLY — no density-to-B-rep reconstruction exists.
"""
from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

sys.dont_write_bytecode = True  # keep tools/ free of __pycache__ litter
REPO_ROOT = Path(__file__).resolve().parent.parent
ACE_ROOT = os.environ.get("ACE_ROOT", os.path.expanduser("~/Work/ACE"))
sys.path.insert(0, ACE_ROOT)
os.environ.setdefault(
    "LMCAD_KERNEL_API", str(REPO_ROOT / "target" / "release" / "kernel-api")
)

ACE_INSTALL_HINT = (
    "ACE package not importable — install it into the interpreter named by "
    "ACE_PYTHON: `pip install -e ~/Work/ACE` (or set ACE_ROOT)."
)


def log(msg: str) -> None:
    print(msg, file=sys.stderr, flush=True)


def emit(payload: dict) -> None:
    print(json.dumps(payload), flush=True)


def main() -> None:  # noqa: PLR0915 — one linear, documented pipeline
    t_start = time.monotonic()
    job = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
    out_dir = Path(job["out_dir"])
    out_dir.mkdir(parents=True, exist_ok=True)

    import numpy as np
    from scipy.ndimage import convolve
    from engine.lmcad import emit_stl_gated
    from engine.verify.fea import reference_fea

    # Shared job plumbing (geometry sampling + region kinds) lives in the FEA
    # runner; import it as a sibling module.
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from ace_fea_runner import build_region_kind, load_geometry

    volfrac = float(job["volfrac"])
    if not 0.0 < volfrac < 1.0:
        raise ValueError(f"volfrac must be in (0,1), got {volfrac}")
    p = float(job.get("penalty", 3.0))
    r_filt = float(job.get("filter_radius_vox", 1.5))
    max_iters = int(job.get("max_iters", 60))
    move = float(job.get("move", 0.2))
    floor = float(job.get("density_floor", 0.02))
    iso = float(job.get("iso", 0.5))
    budget = float(job.get("time_budget_s", 600.0))

    rho0, origin, voxel, sample_s = load_geometry(job, out_dir)
    kind = build_region_kind(job, rho0.shape, voxel, origin)
    frozen = np.zeros(rho0.shape, bool) if kind is None else np.isin(kind, ("frozen", "fixed"))
    void = np.zeros(rho0.shape, bool) if kind is None else (kind == "void")
    design = (rho0 >= 0.5) & ~frozen & ~void
    if kind is not None:
        design &= kind == "design"
    n_design = int(design.sum())
    if n_design == 0:
        raise ValueError("no design voxels: initial geometry solid ∩ region 'design' is empty")

    # Cone kernel for the top88-style density filter (masked so weight never
    # bleeds in from outside the design region).
    R = int(np.ceil(r_filt - 1e-9))
    c = np.arange(-R, R + 1, dtype=float)
    dx, dy, dz = np.meshgrid(c, c, c, indexing="ij")
    w = np.maximum(0.0, r_filt - np.sqrt(dx * dx + dy * dy + dz * dz))
    den = convolve(design.astype(float), w, mode="constant", cval=0.0)
    den[den <= 1e-12] = 1.0  # only read at design voxels, where den >= w(0)

    def filt(field: np.ndarray) -> np.ndarray:
        """H·field / H·1 over the design mask (zero elsewhere)."""
        out = convolve(np.where(design, field, 0.0), w, mode="constant", cval=0.0) / den
        return np.where(design, out, 0.0)

    def physical(x: np.ndarray) -> np.ndarray:
        """Design vars -> the physical density grid the FEA sees."""
        xp = filt(x)
        xp[frozen] = 1.0
        xp[void] = 0.0
        return np.clip(xp, 0.0, 1.0).astype(np.float64)

    def fea(rho: np.ndarray, simp: float | None) -> dict:
        return reference_fea(
            rho, kind, voxel, job["material"],
            job.get("loads", []), job.get("fixtures", []),
            simp_penalty=simp, density_floor=floor, origin_mm=origin,
            direct_solver_max_dof=int(job.get("direct_solver_max_dof", 0)),
        )

    x = np.where(design, volfrac, 0.0)
    compliance_first = compliance_last = float("nan")
    stop_reason = "max_iters"
    iters_done = 0
    t_opt = time.monotonic()
    for it in range(1, max_iters + 1):
        if time.monotonic() - t_start > 0.8 * budget:
            stop_reason = "time_budget"
            break
        x_phys = physical(x)
        res = fea(x_phys, simp=p)
        C = float(res["compliance"])
        compliance_last = C
        if it == 1:
            compliance_first = C
        # SIMP sensitivity: dC/drho_e = -(p/rho_e) * 2 * U_e, then the density-
        # filter chain rule dC/dx_i = H^T (dC/drho / H·1).
        dc = -(p / np.clip(x_phys, floor, 1.0)) * 2.0 * res["element_energy"]
        dc_x = np.where(design, convolve(np.where(design, dc / den, 0.0), w, mode="constant", cval=0.0), 0.0)
        dv_x = np.where(design, convolve(np.where(design, 1.0 / den, 0.0), w, mode="constant", cval=0.0), 0.0)
        # Optimality-criteria bisection on the volume multiplier.
        l1, l2 = 0.0, 1e9
        x_new = x
        while (l2 - l1) / (l1 + l2 + 1e-30) > 1e-3:
            lmid = 0.5 * (l1 + l2)
            B = np.sqrt(np.maximum(0.0, -dc_x) / (lmid * np.maximum(dv_x, 1e-30)))
            x_new = np.clip(np.clip(x * B, x - move, x + move), floor, 1.0)
            x_new = np.where(design, x_new, 0.0)
            if physical(x_new)[design].mean() > volfrac:
                l1 = lmid
            else:
                l2 = lmid
        change = float(np.abs(x_new - x)[design].max())
        x = x_new
        iters_done = it
        log(f"iter {it:3d}: compliance {C:.6e}, change {change:.4f}, vol {physical(x)[design].mean():.4f}")
        if change < 0.01:
            stop_reason = "converged"
            break
    opt_s = time.monotonic() - t_opt

    x_phys = physical(x)
    final_npy = out_dir / "final_rho.npy"
    np.save(final_npy, x_phys.astype(np.float32))

    # HONEST as-built re-analysis: binary occupancy of the thresholded design
    # (what would actually be printed), not the homogenized SIMP proxy.
    t0 = time.monotonic()
    binary = fea(np.where(x_phys >= iso, 1.0, 0.0), simp=None)
    as_built_s = time.monotonic() - t0

    # Gated STL: watertight-or-fail through LMCAD's redistance + narrow-band
    # pipeline. The raw optimizer field commonly pinches at native voxel
    # resolution (diagonal voxel contacts), so escalate: pad one voxel of air
    # (closes level sets that touch the grid boundary; the iso crossing still
    # lands on the true face) and retry at 2x/3x grid-aligned upsampling
    # (`grid_mode=True` keeps origin and voxel/f exact — no silent rescale).
    # The factor used is reported; if every rung fails, stl.ok stays false
    # with the kernel's own refusal text.
    t0 = time.monotonic()
    stl_path = out_dir / "optimized.stl"
    from scipy.ndimage import zoom

    padded = np.pad(x_phys.astype(np.float32), 1, mode="constant")
    pad_origin = tuple(v - voxel for v in origin)
    stl = {"ok": False, "watertight": False, "volume_mm3": 0.0,
           "num_triangles": 0, "issues": ["no mesh attempt ran"]}
    upsample = 0
    for factor in (1, 2, 3):
        if padded.size * factor ** 3 > 48_000_000:
            stl["issues"] = stl.get("issues", []) + [
                f"upsample x{factor} skipped: grid would exceed 48M voxels"]
            break
        field = padded if factor == 1 else zoom(
            padded, factor, order=1, grid_mode=True, mode="grid-constant")
        stl = emit_stl_gated(field.astype(np.float32), voxel / factor,
                             pad_origin, stl_path, iso=iso)
        upsample = factor
        if stl["ok"]:
            break
        log(f"gated mesh refused at voxel/{factor}; escalating")
    stl_s = time.monotonic() - t0

    emit({
        "ok": True,
        "iterations": iters_done,
        "stop_reason": stop_reason,
        "compliance_first": compliance_first,
        "compliance_last": compliance_last,
        "volume_fraction_achieved": float(x_phys[design].mean()),
        "final_rho_npy": str(final_npy),
        "as_built": {
            "max_von_mises_pa": binary["max_von_mises_pa"],
            "max_displacement_m": binary["max_displacement_m"],
            "n_active_elements": binary["n_active_elements"],
        },
        "stl": {
            "ok": bool(stl["ok"]),
            "watertight": bool(stl["watertight"]),
            "volume_mm3": stl["volume_mm3"],
            "num_triangles": stl["num_triangles"],
            "path": str(stl_path),
            "mesh_upsample": upsample,
            "mesh_voxel_mm": voxel / max(upsample, 1),
            "issues": stl.get("issues", []),
        },
        "timings_s": {
            "sample_s": round(sample_s, 3),
            "opt_s": round(opt_s, 3),
            "as_built_s": round(as_built_s, 3),
            "stl_s": round(stl_s, 3),
        },
    })


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:  # noqa: BLE001 — the JSON line IS the contract
        error = f"{type(exc).__name__}: {exc}"
        if isinstance(exc, (ImportError, ModuleNotFoundError)) and "engine" in str(exc):
            error += f" | hint: {ACE_INSTALL_HINT}"
        emit({"ok": False, "error": error})
        sys.exit(0)
