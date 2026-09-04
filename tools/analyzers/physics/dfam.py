"""dfam.py — independent reference Design-for-Additive-Manufacturing checks.

A clean-room, benchmark-validated geometric checker that the ACE verifier runs
on the **as-built occupancy** (``rho >= 0.5`` plus ``region_kind`` overrides),
exactly the solid that would actually be printed. It is independent of any
per-part geometry/analysis code and uses only the voxel field.

Two checks, both purely geometric (no FEA):

* **Minimum wall thickness.** A Euclidean-distance-transform (EDT) of the
  occupancy gives, for every solid voxel, the distance (in voxels) to the
  nearest empty voxel. The thinnest feature is governed by the smallest such
  interior distance. CRITICAL boundary pitfall (designer.system.md §10): EDT
  treats the *array boundary* as infinite distance, NOT as empty space, so a
  voxel sitting against the grid edge would be mis-reported as deep interior.
  We therefore pad the occupancy with one ``False`` cell on every side, run the
  EDT on the padded field, and slice back. The reported wall thickness is
  ``2 * (min interior EDT) * voxel_size`` — twice the minimum half-thickness.

* **Overhang / self-support.** Powder-bed and FFF processes can only print a
  downward-facing surface if its inclination from horizontal is at least the
  self-supporting limit ``overhang_deg`` (typically ~45 deg). A surface
  shallower than that (closer to horizontal) needs support. We find every
  *down-facing* surface voxel — solid, with the neighbour in the
  ``-build_axis`` direction empty — estimate the local outward surface normal
  from the gradient of a Gaussian-smoothed occupancy, measure the angle the
  surface makes with the horizontal build plane, and flag the voxel when that
  surface angle is below ``overhang_deg`` (i.e. the face is too shallow to be
  self-supporting). A flat horizontal roof (angle 0) is the worst case and is
  always flagged.

Conventions match :mod:`physics.fea`: SI units (N, m, Pa), edge length
``h = voxel_size_mm * 1e-3``, occupancy ``rho >= 0.5`` with ``region_kind`` in
``{frozen, fixed}`` forced solid and ``void`` forced empty (reuses
:func:`physics.fea._occupancy`).

Dependencies: numpy + scipy only. No network, no ``agents.*`` imports.
"""

from __future__ import annotations

import numpy as np
from scipy import ndimage

from .fea import _occupancy

__all__ = ["reference_dfam"]

METHOD = "reference_dfam_edt_minwall_gradient_overhang"

_AXIS_INDEX = {"x": 0, "y": 1, "z": 2}


def _build_axis_index(build_axis: str) -> int:
    a = str(build_axis).lower()
    if a not in _AXIS_INDEX:
        raise ValueError(
            f"build_axis must be one of 'x','y','z'; got {build_axis!r}")
    return _AXIS_INDEX[a]


def reference_dfam(rho: np.ndarray,
                   region_kind: np.ndarray | None,
                   voxel_size_mm: float,
                   *,
                   build_axis: str = "z",
                   overhang_deg: float = 45.0,
                   min_wall_mm: float = 0.8,
                   **kwargs) -> dict:
    """Independent reference DfAM checker on a voxel occupancy field.

    Parameters
    ----------
    rho : (nx, ny, nz) array
        Density / occupancy field. Solid voxels are ``rho >= 0.5``.
    region_kind : (nx, ny, nz) string array or None
        ``{frozen, fixed, design, void}`` per voxel. ``frozen``/``fixed`` are
        forced solid; ``void`` forced empty. ``None`` skips this override.
    voxel_size_mm : float
        Cube edge length in mm.
    build_axis : {'x','y','z'}, keyword-only
        Print/growth direction. Down-facing surfaces are the solid voxels whose
        neighbour in the ``-build_axis`` direction is empty. Default ``'z'``.
    overhang_deg : float, keyword-only
        Self-supporting limit, measured as the surface inclination from the
        horizontal build plane. A down-facing surface whose angle from
        horizontal is **below** this needs support. Default ``45.0``.
    min_wall_mm : float, keyword-only
        Minimum acceptable wall thickness in mm. Default ``0.8``.

    Returns
    -------
    dict with keys: ``min_wall_mm_found``, ``overhang_voxel_count``,
    ``overhang_area_mm2``, ``passes_dfam``, ``build_axis``, ``thresholds``,
    ``method``, ``notes``.
    """
    notes: list[str] = []

    rho = np.asarray(rho)
    if rho.ndim != 3:
        raise ValueError(f"rho must be 3-D, got shape {rho.shape}")

    vsz = float(voxel_size_mm)
    if vsz <= 0.0:
        raise ValueError(f"voxel_size_mm must be > 0, got {voxel_size_mm!r}")

    axis = _build_axis_index(build_axis)
    overhang_deg = float(overhang_deg)
    min_wall_mm = float(min_wall_mm)

    occ = _occupancy(rho, region_kind)
    n_solid = int(occ.sum())
    if n_solid == 0:
        notes.append("no solid voxels (occupancy empty after region_kind); "
                     "DfAM checks are vacuous.")
        return {
            "min_wall_mm_found": 0.0,
            "overhang_voxel_count": 0,
            "overhang_area_mm2": 0.0,
            "passes_dfam": False,
            "build_axis": str(build_axis).lower(),
            "thresholds": {"min_wall_mm": min_wall_mm,
                           "overhang_deg": overhang_deg},
            "method": METHOD,
            "notes": notes,
        }

    # ------------------------------------------------------------------
    # Minimum wall thickness via padded EDT (designer.system.md §10).
    # ------------------------------------------------------------------
    # EDT returns, per solid voxel, the Euclidean distance (in voxels) to the
    # nearest background (empty) voxel. PAD with one False cell on every side
    # so voxels against the array boundary are correctly treated as adjacent to
    # empty space, then slice back. Without the pad, EDT treats the array edge
    # as infinite distance and a boundary-touching voxel lies about its depth.
    padded = np.pad(occ.astype(bool), 1, mode="constant", constant_values=False)
    dist = ndimage.distance_transform_edt(padded)
    dist = dist[1:-1, 1:-1, 1:-1]  # back to original (nx, ny, nz)

    interior = dist[occ]  # EDT only over solid voxels
    min_half_vox = float(interior.min())  # min half-thickness, in voxels
    # Wall thickness ~= twice the minimum half-thickness. For a t-voxel-thick
    # wall every voxel is 1 cell from background (EDT = 1), so 2*1*vsz = 2*vsz.
    #
    # CONSERVATIVE-BY-DESIGN: min(EDT) is the distance from the *nearest* solid
    # voxel to free space, so it reports the thinnest point ANYWHERE — including
    # the part's own corners and edges, which on a voxel grid are always ~1
    # voxel half-width. This is a deliberate lower bound for a printability
    # gate (it errs toward flagging, never toward passing an un-printable thin
    # feature). It is NOT a feature-segmented per-wall thickness; a part whose
    # only thin point is a sharp corner will read ~2*vsz even if every true
    # wall is thick. Integrators needing per-feature wall thickness should use
    # a local-thickness / medial-axis measure instead.
    min_wall_mm_found = 2.0 * min_half_vox * vsz

    # ------------------------------------------------------------------
    # Overhang / self-support check.
    # ------------------------------------------------------------------
    # A down-facing surface voxel is a solid voxel whose neighbour in the
    # -build_axis direction is empty (or off the grid). Roll the occupancy by
    # +1 along the build axis: a voxel's -axis neighbour comes from index k-1,
    # i.e. occ shifted toward +axis. Cells rolled in from the boundary are
    # treated as empty (the part sits on the build plate => its underside is
    # exposed), which is the conservative DfAM reading.
    below = np.zeros_like(occ)
    sl_dst = [slice(None)] * 3
    sl_src = [slice(None)] * 3
    sl_dst[axis] = slice(1, None)   # fill k = 1..end
    sl_src[axis] = slice(0, -1)     # from k-1
    below[tuple(sl_dst)] = occ[tuple(sl_src)]
    # below[...,k] is True iff the -axis neighbour (k-1) is solid. The k=0 face
    # has no -axis neighbour on the grid => stays False => treated as exposed.
    downface = occ & (~below)
    n_downface = int(downface.sum())

    overhang_voxel_count = 0
    overhang_area_mm2 = 0.0
    face_area_mm2 = vsz * vsz

    if n_downface == 0:
        notes.append("no down-facing surface voxels found for the given "
                     "build_axis; overhang check is vacuous.")
    else:
        # Local outward surface normal from the gradient of a smoothed
        # occupancy. The gradient of a smoothed solid-indicator points from
        # empty toward solid (uphill, into the material); the OUTWARD normal is
        # the negative gradient. Pad with empty so the smoothing/gradient sees
        # free space outside the part (consistent with the EDT pad above).
        occ_f = occ.astype(np.float64)
        occ_pad = np.pad(occ_f, 2, mode="constant", constant_values=0.0)
        smooth = ndimage.gaussian_filter(occ_pad, sigma=1.0, mode="constant",
                                         cval=0.0)
        gx, gy, gz = np.gradient(smooth)
        gx = gx[2:-2, 2:-2, 2:-2]
        gy = gy[2:-2, 2:-2, 2:-2]
        gz = gz[2:-2, 2:-2, 2:-2]

        di, dj, dk = np.where(downface)
        # Outward normal = -grad(smooth). The surface INCLINATION from the
        # horizontal build plane equals the angle between the surface normal
        # and the build axis: a horizontal surface has a vertical (axial)
        # normal => inclination 0; a vertical wall has a horizontal normal =>
        # inclination 90. Hence
        #   cos(inclination) = |n . build_axis| / |n|   ->   alpha = arccos(.).
        nvec = np.stack([-gx[di, dj, dk], -gy[di, dj, dk], -gz[di, dj, dk]],
                        axis=1)  # (m, 3) outward normals
        nmag = np.linalg.norm(nvec, axis=1)

        # A horizontal flat roof has a degenerate (near-zero) smoothed gradient
        # tangentially but a clean axial component; guard against zero-norm
        # normals by falling back to a purely axial down normal there, which is
        # the worst (0-deg) overhang and must be flagged.
        bad = nmag < 1e-12
        if bad.any():
            ax_comp = np.zeros((bad.sum(), 3))
            ax_comp[:, axis] = -1.0  # pointing down the build axis
            nvec[bad] = ax_comp
            nmag[bad] = 1.0

        axial = np.abs(nvec[:, axis]) / nmag          # = cos(inclination)
        axial = np.clip(axial, 0.0, 1.0)
        surface_angle_deg = np.degrees(np.arccos(axial))  # inclination from horiz

        # Flag faces shallower than the self-supporting limit. Use a tiny
        # epsilon so a face *exactly* at the limit is considered supported.
        eps = 1e-6
        flagged = surface_angle_deg < (overhang_deg - eps)
        overhang_voxel_count = int(flagged.sum())
        overhang_area_mm2 = float(overhang_voxel_count) * face_area_mm2
        notes.append(
            f"overhang: {n_downface} down-facing voxels examined, "
            f"{overhang_voxel_count} below the {overhang_deg:.1f} deg "
            f"self-supporting limit (surface inclination from horizontal).")

    min_wall_ok = min_wall_mm_found >= (min_wall_mm - 1e-9)
    if not min_wall_ok:
        notes.append(
            f"min wall {min_wall_mm_found:.4f} mm < required {min_wall_mm:.4f} "
            f"mm (thinnest feature is {min_half_vox:.3g} voxel half-widths).")

    passes_dfam = bool(min_wall_ok and overhang_voxel_count == 0)

    return {
        "min_wall_mm_found": float(min_wall_mm_found),
        "overhang_voxel_count": int(overhang_voxel_count),
        "overhang_area_mm2": float(overhang_area_mm2),
        "passes_dfam": passes_dfam,
        "build_axis": str(build_axis).lower(),
        "thresholds": {"min_wall_mm": min_wall_mm,
                       "overhang_deg": overhang_deg},
        "method": METHOD,
        "notes": notes,
    }
