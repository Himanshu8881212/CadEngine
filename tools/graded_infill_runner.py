#!/usr/bin/env python3
"""graded_infill_runner.py — stress-graded gyroid lattice infill on LMCAD geometry.

Bridge runner spawned by the LMCAD MCP server (``lmcad-mcp`` tool
``graded_infill``): re-skins a solid and fills its interior with a sheet-gyroid
lattice whose wall thickness follows a prior ``ace_fea`` von Mises field —
thicker walls where the part works hardest, thin walls where it coasts. The
gyroid (not Voronoi) is the lattice because sheet-gyroid walls are
self-supporting for FDM (continuous curvature, short self-buttressed
overhangs, no internal supports) — a claim to verify per part: the
ops-surface ``support_report`` audits B-rep solids and refuses imported
meshes, so check the exported mesh in a slicer support preview.

Usage:  <ACE_PYTHON> graded_infill_runner.py <job.json>
        <ACE_PYTHON> graded_infill_runner.py --selftest

Job JSON (geometry in mm):
    out_dir       REQUIRED  directory for .npy/.stl outputs
    voxel_mm      REQUIRED  cubic voxel edge (mm) of the grid
    origin_mm     optional  world coord of grid node (0,0,0); default [0,0,0]
    GEOMETRY, exactly one of:
      ops + solid + shape [+ supersample]   LMCAD ops; `solid` names the op id
                                            sampled onto the grid (via ACE's
                                            sample_part, like ace_fea)
      npy                                   absolute path of an existing
                                            (nx,ny,nz) float density .npy —
                                            e.g. the solid_fraction.npy a prior
                                            ace_fea run saved
    stress_npy    REQUIRED  stress_field.npy of a prior ace_fea ON THE SAME
                            GRID — a shape mismatch is refused, never resampled
    cell_mm       optional  gyroid cell size, default 8.0
    wall          optional  {min, max} wall thickness range (mm), default
                            {min: 0.8, max: 2.4}
    stress_map    optional  {lo_pct, hi_pct} percentiles of von Mises over
                            SOLID voxels mapped linearly wall.min -> wall.max
                            (clamped outside), default {20, 95}
    shell_mm      optional  solid skin depth preserved at every outer surface
                            (binary erosion of the occupancy), default 1.5
    iso           optional  occupancy / meshing threshold, default 0.5
    file          optional  output mesh name (.stl/.3mf) inside out_dir,
                            default "graded_infill.stl"

Algorithm: occupancy = solid_fraction >= iso; skin = occupancy minus its
erosion by round(shell_mm/voxel) (scipy.ndimage.binary_erosion); interior =
the erosion. Per-voxel wall thickness t = wall.min + (wall.max - wall.min) *
clip((vm - P_lo)/(P_hi - P_lo), 0, 1). The graded density is 1.0 on skin,
the gyroid-band indicator |g| <= alpha(t) on interior, 0 outside; it is
written to graded.npy and meshed BY THE KERNEL (one-shot ``lmcad-mcp``
``run_program`` with ``mesh_density_grid`` — dual contour + heal, watertight-
or-fail), escalating to voxel/2 and voxel/3 with one voxel of air padding
when thin walls pinch at native resolution (the field is re-evaluated
analytically at each finer grid, not interpolated).

Output contract: the LAST non-empty stdout line is ONE JSON object; all
logging goes to stderr. Success => {ok:true, volume_mm3, ...}; any failure =>
{ok:false, error} and STILL exit 0 — the JSON line is the contract.

Honest caveats (echoed by the MCP tool description): the wall thickness is
VOLUME-calibrated (see GyroidCalibration) — local thickness varies ~+/-10%
(p10-p90) over the surface and the band thins where sheets merge; walls under
~2 voxels only resolve on the upsampled rungs; the result is a MESH ONLY, no
B-rep reconstruction exists.
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from pathlib import Path

sys.dont_write_bytecode = True  # keep tools/ free of __pycache__ litter
REPO_ROOT = Path(os.environ.get("LMCAD_ROOT", Path(__file__).resolve().parent.parent))
ENGINE_BIN = REPO_ROOT / "target" / "release" / "lmcad-mcp"

MAX_MESH_CELLS = 48_000_000  # padded-grid ceiling per rung (same as ace_optimize)


def log(msg: str) -> None:
    print(msg, file=sys.stderr, flush=True)


def emit(payload: dict) -> None:
    print(json.dumps(payload), flush=True)


class GyroidCalibration:
    """Numeric |g|-threshold <-> wall-thickness calibration for the gyroid.

    The sheet gyroid of local wall thickness t is the band ``|g| <= alpha``
    of g(x,y,z) = sin X cos Y + sin Y cos Z + sin Z cos X (X = 2*pi*x/c). The
    threshold alpha is NOT the thickness — |g| grows at the local gradient
    rate, which varies over the surface — so alpha(t) is calibrated
    numerically on one unit cell (n^3 cell-centered samples, default 64^3):

    1. A true wall of thickness t is |dist to g=0| <= t/2. To first order
       dist = (c/2pi) * g/|grad g| (gradient analytic), so the wall indicator
       is |g| <= pi*(t/c)*|grad g| and its target volume fraction vf(t) is
       that indicator's sample mean (``wall_fraction``).
    2. The single global threshold reproducing that volume is the empirical
       |g|-quantile at vf(t): alpha(t) = F^-1(vf(t)) where F(a) = sample mean
       of [|g| <= a] (``threshold_for_fraction`` / ``fraction_below``).

    Measured properties at n=64 (checked by ``roundtrip_check`` — the
    --selftest gate): threshold -> fraction -> threshold round-trips within
    1% (gate: 5%); for thin walls vf(t) ~= 3.106*t/c, the literature gyroid
    area constant, rolling off as sheets merge; the single-alpha band matches
    the target VOLUME by construction, while local thickness varies ~+/-10%
    (p10-p90) over the surface, redistributed toward low-gradient regions.
    """

    def __init__(self, n: int = 64) -> None:
        import numpy as np

        u = (np.arange(n) + 0.5) * (2.0 * np.pi / n)
        x, y, z = np.meshgrid(u, u, u, indexing="ij")
        g = np.sin(x) * np.cos(y) + np.sin(y) * np.cos(z) + np.sin(z) * np.cos(x)
        gx = np.cos(x) * np.cos(y) - np.sin(z) * np.sin(x)
        gy = -np.sin(x) * np.sin(y) + np.cos(y) * np.cos(z)
        gz = -np.sin(y) * np.sin(z) + np.cos(z) * np.cos(x)
        grad = np.sqrt(gx * gx + gy * gy + gz * gz)
        self._n_samples = g.size
        self._abs_g = np.sort(np.abs(g).ravel())
        # First-order |distance| to the surface in normalized units: |g|/|grad|.
        self._dist = np.sort((np.abs(g) / np.maximum(grad, 1e-12)).ravel())

    def fraction_below(self, alpha):
        """F(alpha): volume fraction of the cell with |g| <= alpha."""
        import numpy as np

        return np.searchsorted(self._abs_g, alpha, side="right") / self._n_samples

    def threshold_for_fraction(self, vf):
        """F^-1(vf): the |g| threshold whose band has volume fraction vf."""
        import numpy as np

        idx = np.clip((np.asarray(vf) * self._n_samples).astype(int), 0, self._n_samples - 1)
        return self._abs_g[idx]

    def wall_fraction(self, t_over_c):
        """Target volume fraction of a true wall of thickness t on cell c."""
        import numpy as np

        tau = np.pi * np.asarray(t_over_c)  # (t/2)*(2pi/c) in normalized units
        return np.searchsorted(self._dist, tau, side="right") / self._n_samples

    def threshold_for_wall(self, t_over_c):
        """alpha(t): the |g| threshold matching a wall t's volume on cell c."""
        return self.threshold_for_fraction(self.wall_fraction(t_over_c))

    def roundtrip_check(self) -> dict:
        """The --selftest gate: threshold -> fraction -> threshold within 5%,
        plus monotonicity of the wall-thickness map."""
        import numpy as np

        alphas = np.linspace(0.05, 1.4, 28)
        recovered = self.threshold_for_fraction(self.fraction_below(alphas))
        rel_err = float(np.max(np.abs(recovered - alphas) / alphas))
        walls = self.threshold_for_wall(np.linspace(0.02, 0.5, 25))
        monotone = bool(np.all(np.diff(walls) >= 0.0))
        return {
            "ok": rel_err <= 0.05 and monotone,
            "roundtrip_max_rel_err": rel_err,
            "wall_map_monotone": monotone,
            "samples": self._n_samples,
        }


def call_engine(program: dict, out_dir: Path, timeout_s: float = 300.0) -> dict:
    """One-shot ``lmcad-mcp`` run_program (the param_optimize.py exchange).

    The child is pointed at the job's out_dir for BOTH input resolution and
    exports (LMCAD_ROOT + CADCODE_OUT_DIR), so the program reads graded .npy
    files from and writes meshes into out_dir with plain relative names.
    """
    if not ENGINE_BIN.exists():
        raise RuntimeError(f"engine binary not found at {ENGINE_BIN} — build with `cargo build --release -p studio-mcp`")
    lines = "\n".join([
        json.dumps({"jsonrpc": "2.0", "id": 1, "method": "initialize",
                    "params": {"protocolVersion": "2025-06-18", "capabilities": {}}}),
        json.dumps({"jsonrpc": "2.0", "method": "notifications/initialized"}),
        json.dumps({"jsonrpc": "2.0", "id": 2, "method": "tools/call",
                    "params": {"name": "run_program", "arguments": {"program": program}}}),
    ]) + "\n"
    env = {**os.environ, "LMCAD_ROOT": str(out_dir), "CADCODE_OUT_DIR": str(out_dir)}
    out = subprocess.run([str(ENGINE_BIN)], input=lines, capture_output=True,
                         text=True, timeout=timeout_s, env=env, cwd=REPO_ROOT)
    for line in out.stdout.splitlines():
        if not line.strip():
            continue
        try:
            m = json.loads(line)
        except json.JSONDecodeError:
            continue
        if m.get("id") == 2:
            return json.loads(m["result"]["content"][0]["text"])
    raise RuntimeError(f"engine gave no response: {out.stderr[:300]}")


def write_npy_f32(path: Path, arr) -> None:
    import numpy as np

    np.save(path, np.ascontiguousarray(arr, dtype=np.float32))


def gyroid_band(shape, origin, voxel: float, cell: float, alpha_local):
    """Indicator |g| <= alpha_local at the voxel CENTERS of the given grid."""
    import numpy as np

    two_pi_c = 2.0 * np.pi / cell
    ax = [(origin[k] + (np.arange(shape[k]) + 0.5) * voxel) * two_pi_c for k in range(3)]
    x = ax[0][:, None, None]
    y = ax[1][None, :, None]
    z = ax[2][None, None, :]
    g = np.sin(x) * np.cos(y) + np.sin(y) * np.cos(z) + np.sin(z) * np.cos(x)
    return np.abs(g) <= alpha_local


def upsample_nearest(mask, f: int):
    """Exact nearest-neighbour f-times upsampling of a per-voxel field."""
    if f == 1:
        return mask
    return mask.repeat(f, axis=0).repeat(f, axis=1).repeat(f, axis=2)


def build_graded(occ, t_local, origin, voxel: float, cell: float,
                 shell_mm: float, calib: GyroidCalibration, f: int):
    """The graded density grid at refinement f (voxel/f), re-evaluated
    analytically: occupancy/thickness upsampled nearest-neighbour, the skin
    re-eroded at the fine voxel, the gyroid sampled at the fine centers.

    Returns (graded float32 array, skin_voxels, interior_voxels) — counts at
    the FINE resolution.
    """
    import numpy as np
    from scipy.ndimage import binary_erosion

    h = voxel / f
    occ_f = upsample_nearest(occ, f)
    n_erode = int(round(shell_mm / h))
    interior = binary_erosion(occ_f, iterations=n_erode) if n_erode > 0 else occ_f.copy()
    skin = occ_f & ~interior
    alpha = calib.threshold_for_wall(upsample_nearest(t_local, f) / cell)
    band = gyroid_band(occ_f.shape, origin, h, cell, alpha)
    graded = np.where(skin | (interior & band), 1.0, 0.0).astype(np.float32)
    return graded, int(skin.sum()), int(interior.sum())


def mesh_through_engine(graded, origin, voxel: float, out_dir: Path, file: str, iso: float) -> dict:
    """Pad one voxel of air, write the field, and mesh it via the kernel's
    gated ``mesh_density_grid`` (watertight-or-fail). Returns the op receipt
    dict on success, or {"ok": False, "error": ...} with the kernel's own
    refusal text."""
    import numpy as np

    padded = np.pad(graded, 1, mode="constant")
    pad_origin = [v - voxel for v in origin]
    npy_name = "graded_padded.npy"
    write_npy_f32(out_dir / npy_name, padded)
    program = {"ops": [{
        "id": "mesh", "op": "mesh_density_grid", "npy": npy_name,
        "origin": pad_origin, "voxel": voxel, "iso": iso, "file": file,
    }]}
    report = call_engine(program, out_dir)
    op = next((o for o in report.get("ops", []) if o.get("id") == "mesh"), {})
    if not op.get("ok"):
        message = (op.get("error") or {}).get("message", "no mesh op in the engine report")
        return {"ok": False, "error": message}
    m = op.get("measures") or {}
    return {
        "ok": True,
        "volume_mm3": m.get("volume_mm3"),
        "triangles": m.get("num_triangles"),
        "watertight": bool(m.get("watertight")),
        "healed": bool(m.get("healed")),
        "file": op.get("file"),
    }


def main() -> None:
    job = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
    out_dir = Path(job["out_dir"])
    out_dir.mkdir(parents=True, exist_ok=True)

    import numpy as np

    # Shared geometry plumbing (ops-sampling and npy loading) lives in the FEA
    # runner; import it as a sibling module, exactly like ace_optimize_runner.
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from ace_fea_runner import load_geometry

    has_ops = bool(job.get("ops"))
    if has_ops == bool(job.get("npy")):
        raise ValueError("give exactly one geometry route: ops+solid+shape OR npy")
    if not job.get("stress_npy"):
        raise ValueError("stress_npy is required — the stress_field.npy of a prior ace_fea on the SAME grid")
    cell = float(job.get("cell_mm", 8.0))
    wall = job.get("wall") or {}
    t_min = float(wall.get("min", 0.8))
    t_max = float(wall.get("max", 2.4))
    smap = job.get("stress_map") or {}
    lo_pct = float(smap.get("lo_pct", 20.0))
    hi_pct = float(smap.get("hi_pct", 95.0))
    shell_mm = float(job.get("shell_mm", 1.5))
    iso = float(job.get("iso", 0.5))
    file = str(job.get("file", "graded_infill.stl"))
    if not (cell > 0.0 and 0.0 < t_min <= t_max and shell_mm >= 0.0 and 0.0 <= lo_pct < hi_pct <= 100.0):
        raise ValueError(
            f"bad parameters: need cell_mm > 0 (got {cell}), 0 < wall.min <= wall.max "
            f"(got {t_min}..{t_max}), shell_mm >= 0 (got {shell_mm}), 0 <= lo_pct < hi_pct <= 100 "
            f"(got {lo_pct}..{hi_pct})"
        )

    rho, origin, voxel, sample_s = load_geometry(job, out_dir)
    stress = np.load(job["stress_npy"]).astype(np.float64)
    if stress.shape != rho.shape:
        raise ValueError(
            f"stress grid {stress.shape} does not match geometry grid {rho.shape} — "
            "stress_npy must come from an ace_fea run on the SAME grid (same shape/voxel/origin)"
        )

    t0 = time.monotonic()
    notes: list[str] = []
    occ = rho >= iso
    if not occ.any():
        raise ValueError(f"occupancy is empty at iso {iso} — nothing to infill")
    solid_voxels = int(occ.sum())
    solid_volume = solid_voxels * voxel**3

    vm = stress[occ]
    lo_pa = float(np.percentile(vm, lo_pct))
    hi_pa = float(np.percentile(vm, hi_pct))
    if hi_pa <= lo_pa:
        notes.append(
            f"stress percentiles degenerate (P{lo_pct:g} == P{hi_pct:g} == {lo_pa:.3g} Pa) — "
            "using the mid wall thickness everywhere"
        )
        t_local = np.full(rho.shape, 0.5 * (t_min + t_max))
    else:
        t_local = t_min + (t_max - t_min) * np.clip((stress - lo_pa) / (hi_pa - lo_pa), 0.0, 1.0)

    calib = GyroidCalibration()
    graded, skin_voxels, interior_voxels = build_graded(occ, t_local, origin, voxel, cell, shell_mm, calib, f=1)
    if interior_voxels == 0:
        raise ValueError(
            f"shell_mm {shell_mm} erodes the whole part at voxel {voxel} — no interior left to "
            "lattice (reduce shell_mm or refine the grid)"
        )
    graded_npy = out_dir / "graded.npy"
    write_npy_f32(graded_npy, graded)
    if t_min < 2.0 * voxel:
        notes.append(
            f"wall.min {t_min} mm is under 2 voxels ({voxel} mm grid) — native-resolution walls "
            "quantize to >= 1 voxel; trust the upsampled rung the receipt names"
        )
    grade_s = time.monotonic() - t0

    # Kernel-gated meshing, escalating exactly like ace_optimize: native voxel
    # first, then the field RE-EVALUATED (not interpolated) at voxel/2, voxel/3.
    t0 = time.monotonic()
    mesh: dict = {"ok": False, "error": "no mesh attempt ran"}
    upsample = 0
    for f in (1, 2, 3):
        cells = (graded.shape[0] * f + 2) * (graded.shape[1] * f + 2) * (graded.shape[2] * f + 2)
        if cells > MAX_MESH_CELLS:
            notes.append(f"upsample x{f} skipped: padded grid would exceed {MAX_MESH_CELLS} voxels")
            break
        field = graded if f == 1 else build_graded(occ, t_local, origin, voxel, cell, shell_mm, calib, f)[0]
        mesh = mesh_through_engine(field, origin, voxel / f, out_dir, file, iso=0.5)
        upsample = f
        if mesh["ok"]:
            break
        log(f"gated mesh refused at voxel/{f}: {mesh['error']}; escalating")
    mesh_s = time.monotonic() - t0
    if not mesh["ok"]:
        raise RuntimeError(f"the kernel refused to mesh the graded field watertight at every rung — last refusal: {mesh['error']}")

    emit({
        "ok": True,
        "file": mesh["file"],
        "volume_mm3": mesh["volume_mm3"],
        "solid_volume_mm3": solid_volume,
        "volume_fraction": mesh["volume_mm3"] / solid_volume,
        "skin_voxels": skin_voxels,
        "interior_voxels": interior_voxels,
        "cell_mm": cell,
        "wall_range_applied": {"min": t_min, "max": t_max},
        "stress_pcts_used": {"lo_pct": lo_pct, "hi_pct": hi_pct, "lo_pa": lo_pa, "hi_pa": hi_pa},
        "watertight": mesh["watertight"],
        "healed": mesh["healed"],
        "triangles": mesh["triangles"],
        "mesh_upsample": upsample,
        "mesh_voxel_mm": voxel / max(upsample, 1),
        "graded_npy": str(graded_npy),
        "notes": notes,
        "timings_s": {
            "sample_s": round(sample_s, 3),
            "grade_s": round(grade_s, 3),
            "mesh_s": round(mesh_s, 3),
        },
    })


if __name__ == "__main__":
    try:
        if len(sys.argv) > 1 and sys.argv[1] == "--selftest":
            emit(GyroidCalibration().roundtrip_check())
        else:
            main()
    except Exception as exc:  # noqa: BLE001 — the JSON line IS the contract
        emit({"ok": False, "error": f"{type(exc).__name__}: {exc}"})
        sys.exit(0)
