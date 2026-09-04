"""selectors.py — region selectors for the independent reference FEA verifier.

Clean-room implementation of the ACE selector schema (see
``agents/_schema.md`` §3). A *selector* is a dict that names a region of the
voxel envelope; ``resolve_selector`` turns it into a boolean **element** mask
over the ``(nx, ny, nz)`` voxel grid.

Coordinate convention
----------------------
* The grid has ``nx*ny*nz`` hex8 elements (voxels) and
  ``(nx+1)*(ny+1)*(nz+1)`` nodes at the grid corners.
* Element ``(i, j, k)`` occupies the axis-aligned box

      [origin + (i,   j,   k  ) * h ,  origin + (i+1, j+1, k+1) * h]

  where ``h = voxel_size_mm`` (all selector geometry is in **mm**, matching
  the schema's ``*_mm`` keys). Its **center** is at
  ``origin + (i+0.5, j+0.5, k+0.5) * h``.
* Node ``(a, b, c)`` (a in 0..nx, etc.) sits at ``origin + (a, b, c) * h``.

This module is dependency-light: numpy only. No network, no imports of
``agents.*`` or any per-part code.
"""

from __future__ import annotations

from typing import Iterable

import numpy as np

__all__ = [
    "resolve_selector",
    "element_mask_to_node_ids",
    "element_centers_mm",
    "node_coords_mm",
]


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def element_centers_mm(shape: tuple[int, int, int],
                       voxel_size_mm: float,
                       origin_mm: Iterable[float] = (0.0, 0.0, 0.0),
                       ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return broadcastable (cx, cy, cz) element-center coordinate grids in mm.

    Each output has shape ``(nx, ny, nz)`` so they can be compared elementwise
    against selector bounds.
    """
    nx, ny, nz = shape
    ox, oy, oz = (float(v) for v in origin_mm)
    h = float(voxel_size_mm)
    cx = ox + (np.arange(nx) + 0.5) * h
    cy = oy + (np.arange(ny) + 0.5) * h
    cz = oz + (np.arange(nz) + 0.5) * h
    return np.meshgrid(cx, cy, cz, indexing="ij")


def node_coords_mm(shape: tuple[int, int, int],
                   voxel_size_mm: float,
                   origin_mm: Iterable[float] = (0.0, 0.0, 0.0),
                   ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return broadcastable (x, y, z) node coordinate grids in mm.

    Each output has shape ``(nx+1, ny+1, nz+1)``.
    """
    nx, ny, nz = shape
    ox, oy, oz = (float(v) for v in origin_mm)
    h = float(voxel_size_mm)
    x = ox + np.arange(nx + 1) * h
    y = oy + np.arange(ny + 1) * h
    z = oz + np.arange(nz + 1) * h
    return np.meshgrid(x, y, z, indexing="ij")


# ---------------------------------------------------------------------------
# Element mask -> node ids
# ---------------------------------------------------------------------------

def element_mask_to_node_ids(elem_mask: np.ndarray,
                             node_id: np.ndarray | None = None,
                             ) -> np.ndarray:
    """Convert a boolean element mask to the node ids touching those elements.

    Parameters
    ----------
    elem_mask : (nx, ny, nz) bool
        True where an element is selected.
    node_id : (nx+1, ny+1, nz+1) int array, optional
        Mapping from grid-corner index to a global (compacted) node id, with
        ``-1`` for nodes that are not part of the active model (the FEA module
        numbers only nodes touching an active element). If supplied, the
        returned ids are global ids drawn from this map, and corners mapped to
        ``-1`` are dropped. If omitted, the returned ids are flat indices into
        the full ``(nx+1, ny+1, nz+1)`` corner grid (C order).

    Returns
    -------
    np.ndarray
        Sorted, unique 1-D int array of node ids touching the selected
        elements.
    """
    nx, ny, nz = elem_mask.shape
    # Mark the 8 corners of every selected element on the node grid.
    touched = np.zeros((nx + 1, ny + 1, nz + 1), dtype=bool)
    ii, jj, kk = np.where(elem_mask)
    for di in (0, 1):
        for dj in (0, 1):
            for dk in (0, 1):
                touched[ii + di, jj + dj, kk + dk] = True

    if node_id is None:
        flat = np.ravel_multi_index(np.where(touched),
                                    (nx + 1, ny + 1, nz + 1))
        return np.unique(flat)

    ids = node_id[touched]
    ids = ids[ids >= 0]
    return np.unique(ids)


# ---------------------------------------------------------------------------
# Individual selector kinds
# ---------------------------------------------------------------------------

def _resolve_all(shape: tuple[int, int, int]) -> np.ndarray:
    return np.ones(shape, dtype=bool)


def _resolve_bbox(sel: dict,
                  shape: tuple[int, int, int],
                  voxel_size_mm: float,
                  origin_mm: Iterable[float]) -> np.ndarray:
    if "min_mm" not in sel or "max_mm" not in sel:
        raise ValueError("bbox selector requires 'min_mm' and 'max_mm'")
    lo = np.asarray(sel["min_mm"], dtype=float)
    hi = np.asarray(sel["max_mm"], dtype=float)
    if lo.shape != (3,) or hi.shape != (3,):
        raise ValueError("bbox 'min_mm'/'max_mm' must be 3-vectors")
    # Allow caller to pass min/max in either order.
    lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)

    cx, cy, cz = element_centers_mm(shape, voxel_size_mm, origin_mm)
    # Half-voxel tolerance so a degenerate box (min == max, as used for a
    # single-point load in the benchmarks) still catches the element whose
    # center is nearest. Without it a min==max==99 box selects nothing.
    h = float(voxel_size_mm)
    tol = 0.5 * h + 1e-9
    inside = (
        (cx >= lo[0] - tol) & (cx <= hi[0] + tol) &
        (cy >= lo[1] - tol) & (cy <= hi[1] + tol) &
        (cz >= lo[2] - tol) & (cz <= hi[2] + tol)
    )
    return inside


def _resolve_plane(sel: dict,
                   shape: tuple[int, int, int],
                   voxel_size_mm: float,
                   origin_mm: Iterable[float]) -> np.ndarray:
    axis = sel.get("axis")
    if axis not in ("x", "y", "z"):
        raise ValueError(f"plane selector axis must be 'x'|'y'|'z', got {axis!r}")
    if "value_mm" not in sel:
        raise ValueError("plane selector requires 'value_mm'")
    value = float(sel["value_mm"])
    side = sel.get("side", "+")
    if side not in ("+", "-"):
        raise ValueError(f"plane selector side must be '+'|'-', got {side!r}")

    centers = element_centers_mm(shape, voxel_size_mm, origin_mm)
    ax = {"x": 0, "y": 1, "z": 2}[axis]
    c = centers[ax]
    h = float(voxel_size_mm)

    # One-voxel tolerance band: a clamp specified at value_mm = 0 on the "-"
    # side must still catch the first layer of elements (centers at 0.5*h),
    # which sit on the "+" side of the plane geometrically. The band of width
    # h around the plane guarantees the boundary layer is always captured.
    band = h + 1e-9
    if side == "-":
        # On the negative side OR within one voxel of the plane.
        return c <= value + band
    else:
        return c >= value - band


def _resolve_cylinder(sel: dict,
                      shape: tuple[int, int, int],
                      voxel_size_mm: float,
                      origin_mm: Iterable[float]) -> np.ndarray:
    """Solid cylinder: elements whose center lies inside a finite cylinder.

    Recognised keys (mm): ``axis`` ('x'|'y'|'z'), ``center_mm`` [x,y,z] (a
    point on the axis), ``radius_mm``. Optional ``length_mm`` + the axial
    coordinate of ``center_mm`` define a finite extent (centered on
    ``center_mm`` along ``axis``); omitted ``length_mm`` means infinite.
    """
    axis = sel.get("axis")
    if axis not in ("x", "y", "z"):
        raise ValueError(f"cylinder selector axis must be 'x'|'y'|'z', got {axis!r}")
    if "center_mm" not in sel or "radius_mm" not in sel:
        raise ValueError("cylinder selector requires 'center_mm' and 'radius_mm'")
    center = np.asarray(sel["center_mm"], dtype=float)
    if center.shape != (3,):
        raise ValueError("cylinder 'center_mm' must be a 3-vector")
    radius = float(sel["radius_mm"])

    cx, cy, cz = element_centers_mm(shape, voxel_size_mm, origin_mm)
    comps = {"x": cx, "y": cy, "z": cz}
    ax = {"x": 0, "y": 1, "z": 2}[axis]
    radial_axes = [k for k in ("x", "y", "z") if k != axis]
    r2 = np.zeros(shape, dtype=float)
    for k in radial_axes:
        kk = {"x": 0, "y": 1, "z": 2}[k]
        r2 = r2 + (comps[k] - center[kk]) ** 2
    mask = r2 <= radius * radius

    length = sel.get("length_mm")
    if length is not None:
        half = 0.5 * float(length)
        axial = comps[axis]
        mask = mask & (np.abs(axial - center[ax]) <= half)
    return mask


def _resolve_sphere(sel: dict,
                    shape: tuple[int, int, int],
                    voxel_size_mm: float,
                    origin_mm: Iterable[float]) -> np.ndarray:
    """Solid sphere: elements whose center lies inside a sphere.

    Keys (mm): ``center_mm`` [x,y,z], ``radius_mm``.
    """
    if "center_mm" not in sel or "radius_mm" not in sel:
        raise ValueError("sphere selector requires 'center_mm' and 'radius_mm'")
    center = np.asarray(sel["center_mm"], dtype=float)
    if center.shape != (3,):
        raise ValueError("sphere 'center_mm' must be a 3-vector")
    radius = float(sel["radius_mm"])

    cx, cy, cz = element_centers_mm(shape, voxel_size_mm, origin_mm)
    r2 = (cx - center[0]) ** 2 + (cy - center[1]) ** 2 + (cz - center[2]) ** 2
    return r2 <= radius * radius


# ---------------------------------------------------------------------------
# Public dispatcher
# ---------------------------------------------------------------------------

def resolve_selector(selector: dict,
                     shape: tuple[int, int, int],
                     voxel_size_mm: float,
                     origin_mm: Iterable[float] = (0.0, 0.0, 0.0),
                     ) -> np.ndarray:
    """Resolve a selector dict to a boolean ``(nx, ny, nz)`` element mask.

    Supported ``type`` values: ``"all"``, ``"bbox"``, ``"plane"``,
    ``"cylinder"``, ``"sphere"``.

    ``"shell"`` is intentionally **not** implemented: extracting a surface
    shell of a given offset depends on the actual occupancy field (which this
    pure-geometry resolver does not see) and on the part's outer-surface
    definition, so attempting it here would silently produce a wrong region.
    It raises ``NotImplementedError`` so the verifier can treat any criterion
    keyed off a shell selector as *unverifiable* rather than as a pass.

    Parameters
    ----------
    selector : dict
        Must contain a ``"type"`` key. Geometry keys are in mm.
    shape : (nx, ny, nz)
        Voxel-grid (element) shape.
    voxel_size_mm : float
        Cube edge length in mm.
    origin_mm : 3-vector, default (0, 0, 0)
        World coordinate of node (0, 0, 0).

    Returns
    -------
    np.ndarray
        Boolean array of shape ``(nx, ny, nz)``.
    """
    if not isinstance(selector, dict):
        raise TypeError(f"selector must be a dict, got {type(selector).__name__}")
    if len(shape) != 3:
        raise ValueError(f"shape must be a 3-tuple, got {shape!r}")
    t = selector.get("type")

    if t == "all":
        return _resolve_all(shape)
    if t == "bbox":
        return _resolve_bbox(selector, shape, voxel_size_mm, origin_mm)
    if t == "plane":
        return _resolve_plane(selector, shape, voxel_size_mm, origin_mm)
    if t == "cylinder":
        return _resolve_cylinder(selector, shape, voxel_size_mm, origin_mm)
    if t == "sphere":
        return _resolve_sphere(selector, shape, voxel_size_mm, origin_mm)
    if t == "shell":
        raise NotImplementedError(
            "shell selector is not supported by the reference verifier: a "
            "surface shell of a given offset depends on the as-built occupancy "
            "field and the part's outer-surface definition, which this "
            "pure-geometry resolver cannot reconstruct without risking a wrong "
            "region. Treat any criterion keyed off a 'shell' selector as "
            "unverifiable."
        )
    raise NotImplementedError(
        f"selector type {t!r} is not recognised; supported types are "
        f"'all', 'bbox', 'plane', 'cylinder', 'sphere' (and 'shell' which is "
        f"deliberately unverifiable)."
    )
