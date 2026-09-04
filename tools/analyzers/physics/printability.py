"""printability.py — independent print-readiness verifier for the FINAL STL.

Where :mod:`physics.dfam` audits the voxel *field*, this module audits
the **delivered mesh** — the exact geometry a slicer will see. It is pure
Python on the verifier's existing dependency set (trimesh + numpy + scipy),
independent of any per-part agent code and of how the LMCAD kernel produced
the STL.

Checks (all geometric, no FEA):

* **Watertight + single solid body.** A non-watertight or multi-body STL is
  not a printable solid, period. ``bodies`` counts face-adjacency-connected
  components (``mesh.split(only_watertight=False)``); winding consistency is
  reported alongside.

* **Overhang audit.** For a build direction (default ``+z``), a *down-facing*
  triangle (outward normal with a negative build-axis component) whose surface
  inclination from the horizontal build plane is **below** ``overhang_deg``
  (default 45 deg) needs support. Inclination equals the angle between the
  outward normal and the straight-down direction: a flat roof (normal
  straight down) is 0 deg — the worst case; a vertical wall is 90 deg — always
  self-supporting. Faces resting on the build plate (within
  ``bed_contact_tol_mm`` of the mesh minimum along the build axis) are
  bed-supported and excluded. ``overhang_fraction`` is flagged down-facing
  area / total down-facing area (bed contact excluded); 0.0 when there is no
  down-facing area.

* **Bridge-span estimate.** Near-horizontal down-facing patches (inclination
  below ``bridge_flat_deg``, default 20 deg) are clustered by face adjacency;
  each patch's vertices are projected onto the build plane and the patch's
  **minor principal extent** is reported as its bridge span.
  HONEST CAVEAT: this assumes the slicer bridges along the patch's short
  direction and that both ends in that direction are anchored — an
  *optimistic* (lower-bound) estimate; a tunnel ceiling open at its short
  ends really spans its LONG direction. The default limit
  (``bridge_span_limit_mm`` = 10 mm) is therefore deliberately tight for FDM.

* **Minimum wall from the mesh.** The STL is voxelized (surface + interior
  fill) at a pitch fine enough to resolve the threshold (``min_wall_mm/4``,
  memory-capped), then a *medial* EDT reading is taken: the minimum EDT over
  the EDT's local maxima (26-neighbourhood), times two. This measures true
  feature thickness — e.g. a 2-voxel slab reads ~2 pitches, a 10 mm cube
  reads ~10 mm. It deliberately does NOT reuse ``reference_dfam``'s
  min-over-ALL-solid-voxels reading, because that reading equals 2x the
  voxel pitch on any solid with an exposed surface (a resolution statement,
  meaningful on the physics grid it was designed for but meaningless on an
  arbitrary re-voxelization pitch). The raw reading is debiased by the
  2-voxel surface-voxelization dilation (slab-calibrated), leaving a
  <= +0 / -1 voxel error — conservative for a >= gate; sharp knife-edges
  and acute tapers honestly read as ~0 mm thin features. ``None`` when
  unmeasurable (non-watertight
  mesh — interior fill is undefined — or the memory cap forces a pitch too
  coarse to resolve the threshold); an unmeasured wall is reported as such
  and makes ``printable`` False — never silently counted as a pass.

``printable`` is True only when EVERY check above was measured and passed at
the given thresholds — i.e. "support-free print-ready as-is". Callers with a
support strategy may treat overhang/bridge/wall as informational (that policy
decision lives in :mod:`agents._verify`, keyed on
``designer_brief.support_strategy``).

Default thresholds (documented, overridable per call):
  overhang_deg = 45.0           # classic FDM self-supporting limit
  min_wall_mm = 0.8             # 2 perimeters of a 0.4 mm nozzle
  bridge_span_limit_mm = 10.0   # tight, compensating the optimistic estimate
  overhang_area_tol_mm2 = 1.0   # ignores sub-mm^2 tessellation debris only
  bridge_flat_deg = 20.0        # "near-horizontal" patch cut-off
  bed_contact_tol_mm = 0.5      # first-layer band treated as bed-supported

Dependencies: numpy + scipy + trimesh. No network, no ``agents.*`` imports.
"""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np
from scipy import ndimage, sparse
from scipy.sparse.csgraph import connected_components

__all__ = ["printability_report"]

METHOD = "printability_mesh_overhang_bridge_medialwall"

_AXIS_INDEX = {"x": 0, "y": 1, "z": 2}
_EPS_DEG = 1e-6


def _max_voxel_cells() -> int:
    """Cell cap for the min-wall voxelization (memory guard). Override with
    ACE_PRINTABILITY_MAX_CELLS."""
    raw = os.environ.get("ACE_PRINTABILITY_MAX_CELLS")
    if raw:
        try:
            return max(1000, int(raw))
        except ValueError:
            pass
    return 2_000_000


def _load_mesh(stl):
    """Path/str → trimesh.Trimesh (or the object itself). Returns
    (mesh_or_None, error_string_or_None). Never raises on bad geometry —
    a failed load is a verdict (unprintable), not a crash."""
    import trimesh
    if isinstance(stl, trimesh.Trimesh):
        return stl, None
    try:
        mesh = trimesh.load(str(stl), force="mesh")
    except Exception as e:  # noqa: BLE001 — unloadable STL is a finding
        return None, f"STL failed to load: {type(e).__name__}: {e}"
    if not isinstance(mesh, trimesh.Trimesh):
        return None, f"STL loaded as {type(mesh).__name__}, not a Trimesh"
    if mesh.is_empty or len(mesh.faces) == 0:
        return None, "STL contains no triangles"
    return mesh, None


def _min_wall_from_mesh(mesh, min_wall_mm: float, max_cells: int,
                        notes: list[str]) -> tuple[float | None, float | None]:
    """Medial-EDT minimum wall of a watertight mesh via voxelization.

    Returns (min_wall_mm_found | None, pitch_mm | None). None when the
    measurement is not possible at an honest resolution (documented in
    ``notes``).
    """
    if not mesh.is_watertight:
        notes.append("min-wall not measured: mesh is not watertight, so the "
                     "voxel interior fill is undefined.")
        return None, None
    # Pitch fine enough to resolve the threshold (4 cells across a wall at
    # the limit), floored at 0.05 mm, then coarsened to fit the cell cap.
    pitch = max(float(min_wall_mm) / 4.0, 0.05)
    ext = np.maximum(np.asarray(mesh.extents, dtype=float), 1e-9)

    def cells_at(p: float) -> float:
        return float(np.prod(np.ceil(ext / p) + 3.0))

    if cells_at(pitch) > max_cells:
        # smallest pitch that fits the budget (cube-root scaling), + margin
        pitch = float((np.prod(ext) / max_cells) ** (1.0 / 3.0)) * 1.05
        while cells_at(pitch) > max_cells:
            pitch *= 1.25
        notes.append(
            f"min-wall voxelization coarsened to pitch {pitch:.3f} mm to fit "
            f"the {max_cells:,}-cell memory cap.")
    if pitch > float(min_wall_mm) / 2.0:
        notes.append(
            f"min-wall NOT measurable: achievable pitch {pitch:.3f} mm cannot "
            f"resolve the {min_wall_mm:.3f} mm threshold (needs <= "
            f"threshold/2). Raise ACE_PRINTABILITY_MAX_CELLS to measure.")
        return None, float(pitch)

    occ = np.asarray(mesh.voxelized(pitch).fill().matrix, dtype=bool)
    if not occ.any():
        notes.append("min-wall not measured: voxelization produced no "
                     "solid cells.")
        return None, float(pitch)
    padded = np.pad(occ, 1, mode="constant", constant_values=False)
    dist = ndimage.distance_transform_edt(padded)[1:-1, 1:-1, 1:-1]
    # Medial voxels = local maxima of the EDT (plateaus included). The
    # thinnest feature's medial plane carries the smallest local-max EDT;
    # thickness ~= 2 * that half-width. Plain surface voxels (EDT=1 with a
    # larger inward neighbour) are NOT maxima, so this does not collapse to
    # 2*pitch the way min-over-all-solid-voxels does.
    medial = (ndimage.maximum_filter(dist, size=3) == dist) & occ
    vals = dist[medial]
    if vals.size == 0:  # cannot happen for a nonempty grid, but stay honest
        notes.append("min-wall not measured: no medial voxels found.")
        return None, float(pitch)
    # DEBIAS: surface voxelization marks every surface-intersecting cell
    # solid, dilating a wall by up to one voxel per side — measured on
    # calibration slabs the raw medial reading is true thickness + ~2
    # voxels, the UNSAFE direction for a >= wall gate (a 0.4 mm slab would
    # read 0.8 mm). Subtract the 2-voxel dilation; residual error is
    # <= +0 / -1 voxel, i.e. conservative (never reads thicker than truth).
    return float(max(2.0 * vals.min() - 2.0, 0.0) * pitch), float(pitch)


def _bridge_spans(mesh, bridge_mask: np.ndarray,
                  axis: int) -> tuple[float, int, list[float]]:
    """Cluster near-horizontal down-facing faces by adjacency; per patch,
    report the minor principal extent of the vertices projected onto the
    build plane. Returns (max_span_mm, n_patches, spans)."""
    if not bridge_mask.any():
        return 0.0, 0, []
    n_faces = len(mesh.faces)
    fa = np.asarray(mesh.face_adjacency)
    if fa.size:
        keep = bridge_mask[fa[:, 0]] & bridge_mask[fa[:, 1]]
        fa = fa[keep]
    if fa.size:
        graph = sparse.coo_matrix(
            (np.ones(len(fa)), (fa[:, 0], fa[:, 1])),
            shape=(n_faces, n_faces))
        _, labels = connected_components(graph, directed=False)
    else:
        labels = np.arange(n_faces)  # every flagged face its own patch
    spans: list[float] = []
    keep_axes = [a for a in range(3) if a != axis]
    for lbl in np.unique(labels[bridge_mask]):
        fidx = np.where(bridge_mask & (labels == lbl))[0]
        pts = mesh.vertices[np.unique(mesh.faces[fidx].ravel())]
        pts2 = pts[:, keep_axes].astype(float)
        pts2 -= pts2.mean(axis=0)
        # principal axes of the projected patch; extents along each
        cov = pts2.T @ pts2
        _, vecs = np.linalg.eigh(cov)
        proj = pts2 @ vecs
        extents = proj.max(axis=0) - proj.min(axis=0)
        spans.append(float(extents.min()))
    return (max(spans) if spans else 0.0), len(spans), spans


def printability_report(stl,
                        *,
                        build_axis: str = "z",
                        overhang_deg: float = 45.0,
                        min_wall_mm: float = 0.8,
                        bridge_span_limit_mm: float = 10.0,
                        overhang_area_tol_mm2: float = 1.0,
                        bridge_flat_deg: float = 20.0,
                        bed_contact_tol_mm: float = 0.5,
                        max_voxel_cells: int | None = None) -> dict:
    """Print-readiness report for a final STL (path or trimesh.Trimesh).

    Parameters mirror the module docstring's documented thresholds; units
    are mm / mm^2 / degrees. ``build_axis`` in {'x','y','z'} (default 'z').

    Returns a JSON-serializable dict:
      watertight            : bool
      bodies                : int — TOP-LEVEL bodies (containment-aware:
                              enclosed cavity shells are internal voids,
                              reported separately as enclosed_cavities)
      overhang_area_mm2     : float — down-facing area below overhang_deg
      overhang_fraction     : float — that area / total down-facing area
      max_bridge_span_mm    : float — worst patch span (see caveat above)
      min_wall_mm           : float | None — medial-EDT reading (None =
                              unmeasurable; documented in notes)
      printable             : bool — ALL checks measured AND passed
      issues                : list[str] — one entry per failed/unmeasured check
      plus: winding_consistent, down_facing_area_mm2, total_area_mm2,
      n_bridge_patches, bridge_spans_mm, min_wall_resolution_mm, build_axis,
      thresholds, method, notes.
    """
    if build_axis not in _AXIS_INDEX:
        raise ValueError(
            f"build_axis must be one of 'x','y','z'; got {build_axis!r}")
    axis = _AXIS_INDEX[build_axis]
    thresholds = {
        "overhang_deg": float(overhang_deg),
        "min_wall_mm": float(min_wall_mm),
        "bridge_span_limit_mm": float(bridge_span_limit_mm),
        "overhang_area_tol_mm2": float(overhang_area_tol_mm2),
        "bridge_flat_deg": float(bridge_flat_deg),
        "bed_contact_tol_mm": float(bed_contact_tol_mm),
    }
    notes: list[str] = []
    issues: list[str] = []
    out: dict = {
        "watertight": False, "bodies": 0,
        "overhang_area_mm2": 0.0, "overhang_fraction": 0.0,
        "max_bridge_span_mm": 0.0, "min_wall_mm": None,
        "printable": False, "issues": issues,
        "winding_consistent": False,
        "down_facing_area_mm2": 0.0, "total_area_mm2": 0.0,
        "n_bridge_patches": 0, "bridge_spans_mm": [],
        "min_wall_resolution_mm": None,
        "build_axis": build_axis, "thresholds": thresholds,
        "method": METHOD, "notes": notes,
    }

    mesh, err = _load_mesh(stl)
    if mesh is None:
        issues.append(err or "STL unusable")
        return out

    # --- solidity -------------------------------------------------------
    out["watertight"] = bool(mesh.is_watertight)
    out["winding_consistent"] = bool(mesh.is_winding_consistent)
    try:
        pieces = mesh.split(only_watertight=False)
        # Containment-aware count: a shell whose centroid lies inside a
        # LARGER shell is an enclosed CAVITY (a legitimate internal void —
        # e.g. a wing bay behind a spar under a closed skin), not a separate
        # print body. Floaters — disconnected shells NOT enclosed by anything
        # — still count as bodies and still fail the gate. The naive count
        # failed a valid skin+spar wing whose aft bay was a clean void.
        tops, cavities = 0, 0
        for i, p in enumerate(pieces):
            inside = False
            for j, q in enumerate(pieces):
                if i != j and len(q.faces) > len(p.faces):
                    try:
                        if bool(q.contains([p.centroid])[0]):
                            inside = True
                            break
                    except Exception:  # noqa: BLE001
                        pass
            if inside:
                cavities += 1
            else:
                tops += 1
        out["bodies"] = int(tops)
        out["enclosed_cavities"] = int(cavities)
        out["shells_raw"] = int(len(pieces))
    except Exception:  # noqa: BLE001 — split can fail on degenerate meshes
        out["bodies"] = -1
        notes.append("body count unavailable (mesh.split failed).")
    if not out["watertight"]:
        issues.append("not watertight — mesh has open edges; not a solid")
    if out["bodies"] != 1:
        issues.append(f"bodies={out['bodies']} (expected exactly 1 solid body)")
    if not out["winding_consistent"]:
        notes.append("face winding is inconsistent; normals re-oriented "
                     "before the overhang audit.")
        try:
            mesh.fix_normals()
        except Exception:  # noqa: BLE001
            notes.append("fix_normals failed; overhang normals may be "
                         "unreliable.")

    normals = np.asarray(mesh.face_normals, dtype=float)
    areas = np.asarray(mesh.area_faces, dtype=float)
    centers = np.asarray(mesh.triangles_center, dtype=float)
    out["total_area_mm2"] = float(areas.sum())

    # --- overhang audit --------------------------------------------------
    # inclination-from-horizontal = angle(outward normal, straight-down):
    # flat roof -> 0 deg (worst), vertical wall -> 90 deg (self-supporting).
    n_axis = normals[:, axis]
    down = n_axis < -1e-8
    axis_min = float(mesh.bounds[0][axis])
    bed = centers[:, axis] <= (axis_min + float(bed_contact_tol_mm))
    down_eff = down & ~bed
    inclination_deg = np.degrees(np.arccos(np.clip(-n_axis, -1.0, 1.0)))
    flagged = down_eff & (inclination_deg < (float(overhang_deg) - _EPS_DEG))

    down_area = float(areas[down_eff].sum())
    overhang_area = float(areas[flagged].sum())
    out["down_facing_area_mm2"] = down_area
    out["overhang_area_mm2"] = overhang_area
    out["overhang_fraction"] = (overhang_area / down_area
                                if down_area > 0.0 else 0.0)
    notes.append(
        f"overhang: {int(down_eff.sum())} down-facing triangles "
        f"({down_area:.1f} mm^2, bed-contact excluded), "
        f"{int(flagged.sum())} below the {float(overhang_deg):.1f} deg "
        f"self-supporting limit ({overhang_area:.1f} mm^2).")
    if overhang_area > float(overhang_area_tol_mm2):
        issues.append(
            f"overhang area {overhang_area:.1f} mm^2 exceeds the "
            f"{float(overhang_area_tol_mm2):.1f} mm^2 tolerance at "
            f"{float(overhang_deg):.1f} deg — needs supports")

    # --- bridge spans ----------------------------------------------------
    bridge_mask = down_eff & (inclination_deg
                              < (float(bridge_flat_deg) - _EPS_DEG))
    max_span, n_patches, spans = _bridge_spans(mesh, bridge_mask, axis)
    out["max_bridge_span_mm"] = float(max_span)
    out["n_bridge_patches"] = int(n_patches)
    out["bridge_spans_mm"] = [round(float(s), 3) for s in spans]
    if n_patches:
        notes.append(
            f"bridge: {n_patches} near-horizontal down-facing patch(es); "
            f"worst minor-extent span {max_span:.1f} mm (optimistic "
            f"estimate — assumes bridging along the short, anchored "
            f"direction).")
    if max_span > float(bridge_span_limit_mm) + 1e-9:
        issues.append(
            f"bridge span {max_span:.1f} mm exceeds the "
            f"{float(bridge_span_limit_mm):.1f} mm limit")

    # --- minimum wall ----------------------------------------------------
    cap = max_voxel_cells if max_voxel_cells is not None else _max_voxel_cells()
    wall, pitch = _min_wall_from_mesh(mesh, float(min_wall_mm), int(cap),
                                      notes)
    out["min_wall_mm"] = wall
    out["min_wall_resolution_mm"] = pitch
    if wall is None:
        issues.append("min wall UNMEASURED (see notes) — cannot claim the "
                      "part meets the wall-thickness minimum")
    elif wall < float(min_wall_mm) - 1e-9:
        issues.append(
            f"min wall {wall:.3f} mm < required {float(min_wall_mm):.3f} mm "
            f"(measured at {pitch:.3f} mm voxel pitch, accuracy ±1 voxel)")

    out["printable"] = not issues
    return out
