"""convergence.py — measured discretization-error bound for the ACE verifier.

ACE historically inflated under-predicted stress/deflection by a *blunt, fixed*
1.2 "analysis-uncertainty factor" (see ``agents/_verify.py._stress_uncertainty``).
That number is a guess. This module replaces the guess with a **measured**
two-grid discretization-error estimate for a *specific* part + load case.

What it does
------------
1. Run :func:`physics.reference_fea` on the NATIVE voxel field
   (``stress_fine = max_von_mises_pa``).
2. Build a **coarsened** field by 2x2x2 block-reduction:
     * ``rho`` -> mean rho per 2-block, re-thresholded ``>= 0.5`` (i.e. a
       coarse element is solid iff the average density of its 8 sub-voxels is
       at least 0.5 — the same binary-occupancy rule ``reference_fea`` uses);
     * ``region_kind`` -> most-conservative label per 2-block (see
       :func:`_coarsen_region_kind`): ``void`` is preserved only if the WHOLE
       block is void, ``fixed``/``frozen`` propagate if ANY sub-voxel carries
       them, so the coarse mesh never *gains* material that the fine mesh
       declared empty and never *loses* a clamp anchor;
     * ``voxel_size_mm`` doubled.
   Then run ``reference_fea`` again (``stress_coarse``).
3. Richardson-style extrapolation assuming O(h^p) convergence. Linear hex8
   stress converges at ``p ~ 1`` (first order in the peak surface stress on a
   bending problem). With a refinement ratio ``r = h_coarse / h_fine = 2`` the
   Richardson extrapolate of the fine/coarse pair is

       S_exact ~= S_fine + (S_fine - S_coarse) / (r**p - 1)
                = S_fine + (S_fine - S_coarse) / (2**p - 1).

   For ``p = 1`` this is ``S_exact ~= 2*S_fine - S_coarse``.
4. Report a RELATIVE discretization-error band

       rel_err = |S_fine - S_coarse| / max(S_fine, eps)

   and a measured uncertainty factor ``1 + rel_err`` (floored at 1.0, capped
   sensibly so a pathological coarsening cannot produce an absurd factor).

IMPORTANT CAVEAT — this is NOT a rigorous GCI
---------------------------------------------
A 2x2x2 block-reduction does not just refine the *mesh* — it also **coarsens
the geometry**. Re-thresholding ``mean rho >= 0.5`` can delete thin features
(a 1-voxel web, a single-voxel fillet) entirely, which changes the *part*, not
only the discretization. So the two FEA runs are not a clean h-refinement pair
of the *same* solid. Treat the result as an **approximate h-refinement
indicator** — a measured, part-specific replacement for the flat 1.2 factor —
NOT as a formally valid Grid Convergence Index. The returned ``notes`` flag
when coarsening changed the active-element count materially (a sign geometry
was lost), and the factor is capped so a degenerate coarsening cannot silently
inflate the bound without bound.

SI throughout (N, m, Pa); edge ``h = voxel_size_mm * 1e-3``; canonical material
keys ``youngs_modulus_pa`` / ``poisson`` / ``density_kg_m3`` — matching
``fea.py``. Dependencies: numpy + scipy only. No network, no ``agents.*``.
"""

from __future__ import annotations

import numpy as np

from .fea import reference_fea, _occupancy

__all__ = ["convergence_study", "coarsen_field"]

METHOD = "two_grid_richardson_h_refinement_indicator"

# Order of convergence assumed for the peak von Mises stress of a linear (Q1)
# hex8 element on a bending-dominated problem. Stress is one derivative less
# accurate than displacement, so the peak stress is ~first order even though
# displacement is ~second order. p=1 is the documented, conservative default.
_DEFAULT_P = 1.0

# Refinement ratio for a 2x2x2 block-reduction: the coarse edge is twice the
# fine edge.
_REFINEMENT_RATIO = 2.0

# Convergence threshold on the relative discretization error.
_CONVERGED_REL_ERR = 0.10

# Sensible cap on the recommended uncertainty factor. A two-grid estimate on a
# geometry-altering coarsening can blow up if coarsening deletes the load path;
# we cap the *recommended* factor so a degenerate study cannot return an absurd
# multiplier. The raw rel_err is always reported uncapped for transparency.
_MAX_UNCERTAINTY_FACTOR = 2.0

_EPS = 1.0e-30  # guards division by ~zero stress


# ---------------------------------------------------------------------------
# Coarsening
# ---------------------------------------------------------------------------

def _block_reduce_mean(field: np.ndarray) -> np.ndarray:
    """Mean of each 2x2x2 block, trimming any odd trailing layer.

    A field of shape ``(nx, ny, nz)`` reduces to ``(nx//2, ny//2, nz//2)``.
    Odd trailing layers (the last index along an odd-length axis) are dropped
    *before* reduction so every coarse cell aggregates a full 2x2x2 block of
    fine cells — partial blocks would bias the mean and, worse, silently keep
    or drop a half-block of material. The dropped sliver is reported by the
    caller via the active-element-count check.
    """
    nx, ny, nz = field.shape
    cx, cy, cz = nx // 2, ny // 2, nz // 2
    trimmed = field[: 2 * cx, : 2 * cy, : 2 * cz]
    # reshape into (cx,2, cy,2, cz,2) and average the size-2 block axes
    blocks = trimmed.reshape(cx, 2, cy, 2, cz, 2)
    return blocks.mean(axis=(1, 3, 5))


def _coarsen_region_kind(region_kind: np.ndarray) -> np.ndarray:
    """Coarsen a (nx,ny,nz) string region_kind field by 2x2x2 blocks.

    Most-conservative-per-block rule, chosen so the coarse occupancy never
    *gains* solid the fine field declared void and never *loses* a fixed/frozen
    anchor that a fixture selector might rely on:

      * ``void``  : a coarse cell is ``void`` ONLY if every sub-voxel is void
                    (so we never resurrect deleted material as solid).
      * ``fixed`` / ``frozen`` : propagate if ANY sub-voxel carries the label
                    (fixed beats frozen beats design), so a clamp's frozen/fixed
                    anchor survives coarsening.
      * otherwise ``design``.

    Priority (high to low): fixed > frozen > design > void. A block resolves to
    the highest-priority label present, EXCEPT that ``void`` only wins when it
    is unanimous (handled by giving it the lowest priority but treating a
    fully-void block specially).
    """
    nx, ny, nz = region_kind.shape
    cx, cy, cz = nx // 2, ny // 2, nz // 2
    trimmed = region_kind[: 2 * cx, : 2 * cy, : 2 * cz]
    blocks = trimmed.reshape(cx, 2, cy, 2, cz, 2)

    out = np.full((cx, cy, cz), "design", dtype=object)

    has_fixed = np.zeros((cx, cy, cz), dtype=bool)
    has_frozen = np.zeros((cx, cy, cz), dtype=bool)
    has_design = np.zeros((cx, cy, cz), dtype=bool)
    n_void = np.zeros((cx, cy, cz), dtype=np.int64)

    # Iterate the 8 sub-positions; vectorised over the coarse grid.
    for a in range(2):
        for b in range(2):
            for c in range(2):
                sub = blocks[:, a, :, b, :, c]  # (cx,cy,cz) of labels
                has_fixed |= (sub == "fixed")
                has_frozen |= (sub == "frozen")
                has_design |= (sub == "design")
                n_void += (sub == "void").astype(np.int64)

    fully_void = n_void == 8
    out[fully_void] = "void"
    # design wins over a non-unanimous void (design has higher priority)
    out[has_design & ~fully_void] = "design"
    # frozen beats design
    out[has_frozen & ~fully_void] = "frozen"
    # fixed beats frozen (highest priority)
    out[has_fixed & ~fully_void] = "fixed"
    return out


def coarsen_field(rho: np.ndarray,
                  region_kind: np.ndarray | None,
                  voxel_size_mm: float):
    """Block-reduce a voxel field by 2x2x2.

    Returns ``(rho_coarse, region_kind_coarse, voxel_size_mm_coarse)``:

      * ``rho_coarse`` = mean rho per 2-block (the binary ``>= 0.5`` threshold
        is applied downstream by ``reference_fea`` / ``_occupancy``, exactly as
        for the fine field — we deliberately return the *mean* density, not a
        re-thresholded mask, so the canonical occupancy rule is applied in
        exactly one place);
      * ``region_kind_coarse`` = most-conservative label per block (or ``None``
        if the input was ``None``);
      * ``voxel_size_mm_coarse`` = ``2 * voxel_size_mm``.

    Raises ``ValueError`` if the field is too small to coarsen (any axis < 2).
    """
    rho = np.asarray(rho)
    if rho.ndim != 3:
        raise ValueError(f"rho must be 3-D, got shape {rho.shape}")
    if min(rho.shape) < 2:
        raise ValueError(
            f"field too small to coarsen: shape {rho.shape} has an axis < 2; "
            f"a 2x2x2 block-reduction needs every axis >= 2.")

    rho_coarse = _block_reduce_mean(rho.astype(float))

    if region_kind is None:
        rk_coarse = None
    else:
        region_kind = np.asarray(region_kind)
        if region_kind.shape != rho.shape:
            raise ValueError(
                f"region_kind shape {region_kind.shape} != rho shape "
                f"{rho.shape}")
        rk_coarse = _coarsen_region_kind(region_kind)

    return rho_coarse, rk_coarse, 2.0 * float(voxel_size_mm)


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def convergence_study(rho: np.ndarray,
                      region_kind: np.ndarray | None,
                      voxel_size_mm: float,
                      material: dict,
                      loads: list[dict],
                      fixtures: list[dict],
                      **kwargs) -> dict:
    """Measured two-grid discretization-error bound for one part + load case.

    Runs :func:`physics.reference_fea` on the native field and on a
    2x2x2 block-coarsened field, then forms a Richardson-style estimate of the
    converged peak von Mises stress and a relative discretization-error band.

    Parameters
    ----------
    rho, region_kind, voxel_size_mm, material, loads, fixtures
        As in :func:`physics.reference_fea`. ``origin_mm`` and any other
        kwargs are forwarded to BOTH FEA runs unchanged (loads/fixtures resolve
        via selectors in mm, so doubling ``voxel_size_mm`` keeps the part the
        same physical size with half the voxels per axis).
    convergence_order : float, optional kwarg
        Assumed order ``p`` for the O(h^p) Richardson extrapolation. Default
        ``1.0`` (linear-hex8 peak stress is ~first order).

    Returns
    -------
    dict with keys::

        stress_fine_pa              float   native-grid max von Mises
        stress_coarse_pa            float   coarse-grid max von Mises
        stress_extrapolated_pa      float   Richardson O(h^p) extrapolate
        rel_discretization_error    float   |fine-coarse| / max(fine, eps)
        recommended_uncertainty_factor float >= 1.0, = clamp(1 + rel_err)
        converged                   bool    rel_err < ~0.10
        convergence_order           float   p actually used
        refinement_ratio            float   2.0
        n_active_fine               int
        n_active_coarse             int
        method                      str
        notes                       list[str]

    Notes
    -----
    Coarsening loses geometry (see the module docstring): this is an
    *approximate h-refinement indicator*, NOT a rigorous GCI. If the field is
    too small to coarsen, no coarse run is attempted; the function returns a
    conservative fallback (``recommended_uncertainty_factor`` = the legacy 1.2)
    with a clear note rather than raising, so a verifier can still proceed.
    """
    notes: list[str] = []
    p = float(kwargs.pop("convergence_order", _DEFAULT_P))
    if p <= 0.0:
        notes.append(f"convergence_order {p} <= 0 is invalid; using p=1.0.")
        p = _DEFAULT_P

    # --- fine (native) run -------------------------------------------------
    fine = reference_fea(rho, region_kind, voxel_size_mm, material,
                         loads, fixtures, **kwargs)
    stress_fine = float(fine["max_von_mises_pa"])
    n_active_fine = int(fine["n_active_elements"])

    # --- guard: too small to coarsen --------------------------------------
    rho_arr = np.asarray(rho)
    if min(rho_arr.shape) < 2:
        notes.append(
            f"field shape {rho_arr.shape} has an axis < 2 — too small to "
            f"coarsen by 2x2x2; no two-grid estimate is possible. Falling back "
            f"to the legacy flat 1.2 analysis-uncertainty factor.")
        return _too_small_result(stress_fine, n_active_fine, p, notes)

    # --- build coarse field + run -----------------------------------------
    rho_c, rk_c, vsz_c = coarsen_field(rho, region_kind, voxel_size_mm)

    # A coarse field can collapse if 2x2x2 averaging deletes a thin load path
    # or all the anchor material. Both reference_fea (no active elements / no
    # fixed DOFs) and our own occupancy check below surface that as a fallback.
    occ_c = _occupancy(rho_c, rk_c)
    n_active_coarse = int(occ_c.sum())
    if n_active_coarse == 0:
        notes.append(
            "coarsened field has NO active elements (2x2x2 re-thresholding "
            "deleted all material) — cannot run the coarse grid. Falling back "
            "to the legacy flat 1.2 analysis-uncertainty factor.")
        return _too_small_result(stress_fine, n_active_fine, p, notes,
                                 n_active_coarse=0)

    try:
        coarse = reference_fea(rho_c, rk_c, vsz_c, material,
                               loads, fixtures, **kwargs)
    except ValueError as exc:
        # e.g. coarsening removed the clamp anchor -> "no DOFs constrained",
        # or otherwise made the coarse model unsolvable. Be honest, don't guess.
        notes.append(
            f"coarse-grid FEA failed ({exc}); the 2x2x2 coarsening changed the "
            f"part enough to break the load/fixture path. Falling back to the "
            f"legacy flat 1.2 analysis-uncertainty factor.")
        return _too_small_result(stress_fine, n_active_fine, p, notes,
                                 n_active_coarse=n_active_coarse)
    except Exception as exc:  # noqa: BLE001 — surface any other solver failure
        notes.append(
            f"coarse-grid FEA raised {type(exc).__name__}: {exc}; falling back "
            f"to the legacy flat 1.2 analysis-uncertainty factor.")
        return _too_small_result(stress_fine, n_active_fine, p, notes,
                                 n_active_coarse=n_active_coarse)

    stress_coarse = float(coarse["max_von_mises_pa"])
    n_active_coarse = int(coarse["n_active_elements"])

    # Flag material geometry change. Exact 8:1 would be a perfect block-reduce
    # of a fully-solid field; real parts differ. A large deviation means the
    # coarse mesh is NOT the same solid (geometry was lost) -> indicator only.
    expected_coarse = n_active_fine / 8.0
    if expected_coarse > 0:
        geom_dev = abs(n_active_coarse - expected_coarse) / expected_coarse
        if geom_dev > 0.10:
            notes.append(
                f"coarsening changed the active-element count by "
                f"{geom_dev:.0%} vs the ideal 8:1 ({n_active_fine} -> "
                f"{n_active_coarse}); thin geometry was altered, so this is an "
                f"approximate h-refinement indicator, not a clean GCI.")

    # --- Richardson-style estimate ----------------------------------------
    # S_exact ~= S_fine + (S_fine - S_coarse) / (r**p - 1), r = 2.
    r = _REFINEMENT_RATIO
    denom = r ** p - 1.0  # = 1.0 for p=1, r=2
    stress_extrap = stress_fine + (stress_fine - stress_coarse) / denom

    rel_err = abs(stress_fine - stress_coarse) / max(stress_fine, _EPS)

    raw_factor = 1.0 + rel_err
    rec_factor = float(min(max(raw_factor, 1.0), _MAX_UNCERTAINTY_FACTOR))
    if raw_factor > _MAX_UNCERTAINTY_FACTOR:
        notes.append(
            f"raw uncertainty factor {raw_factor:.3f} exceeds the cap "
            f"{_MAX_UNCERTAINTY_FACTOR}; recommended factor clamped. A factor "
            f"this large usually means coarsening altered the part rather than "
            f"a true discretization error — inspect the two stress values.")

    converged = bool(rel_err < _CONVERGED_REL_ERR)

    notes.append(
        f"two-grid h-refinement: h_fine={voxel_size_mm} mm "
        f"(n_active={n_active_fine}), h_coarse={vsz_c} mm "
        f"(n_active={n_active_coarse}); assumed O(h^{p:g}); "
        f"refinement ratio r={r:g}.")
    notes.append(
        "APPROXIMATE indicator: 2x2x2 block-coarsening loses geometry "
        "(re-thresholding mean rho can delete thin features), so the two runs "
        "are not a clean same-solid h-refinement pair. NOT a rigorous GCI.")
    # Carry through any solver notes from the runs that flag unverifiable loads.
    for tag, sub in (("fine", fine), ("coarse", coarse)):
        for n in sub.get("notes", []):
            if any(k in n for k in ("unverifiable", "moment", "unknown kind")):
                notes.append(f"[{tag}] {n}")

    return {
        "stress_fine_pa": stress_fine,
        "stress_coarse_pa": stress_coarse,
        "stress_extrapolated_pa": float(stress_extrap),
        "rel_discretization_error": float(rel_err),
        "recommended_uncertainty_factor": rec_factor,
        "converged": converged,
        "convergence_order": p,
        "refinement_ratio": float(r),
        "n_active_fine": n_active_fine,
        "n_active_coarse": n_active_coarse,
        "method": METHOD,
        "notes": notes,
    }


def _too_small_result(stress_fine: float,
                      n_active_fine: int,
                      p: float,
                      notes: list[str],
                      n_active_coarse: int | None = None) -> dict:
    """Conservative fallback when no usable coarse grid is available.

    Returns the legacy flat 1.2 factor (the value this module is meant to
    replace) so the verifier can still proceed, but with ``converged=False``
    and a clear note that no measured bound was obtained.
    """
    LEGACY_FACTOR = 1.2
    return {
        "stress_fine_pa": stress_fine,
        "stress_coarse_pa": float("nan"),
        "stress_extrapolated_pa": stress_fine,
        "rel_discretization_error": float("nan"),
        "recommended_uncertainty_factor": LEGACY_FACTOR,
        "converged": False,
        "convergence_order": p,
        "refinement_ratio": float(_REFINEMENT_RATIO),
        "n_active_fine": n_active_fine,
        "n_active_coarse": (0 if n_active_coarse is None else n_active_coarse),
        "method": METHOD + "_fallback_no_coarse_grid",
        "notes": notes,
    }
