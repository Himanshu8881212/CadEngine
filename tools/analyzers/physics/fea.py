"""fea.py — independent reference structural FEA for the ACE verifier.

A clean-room, benchmark-validated **hex8 linear-elastic** solver. The
orchestrator uses this to *verify* delivered parts; it is deliberately
independent of any per-part ``analysis.py`` (those are known to be wrong) and
of any SIMP density weighting.

Key design choices
-------------------
* **As-built occupancy (default).** Active elements are ``occ = rho >= 0.5``
  — a binary threshold, NOT a SIMP-penalised continuous density.
  ``region_kind`` in ``{frozen, fixed}`` is forced occupied and ``void``
  forced empty. This verifies the solid that would actually be printed,
  independent of how the optimizer weighted densities.
* **Optional SIMP mode.** Passing ``simp_penalty=p`` switches
  :func:`reference_fea` to true topology-optimization semantics: active
  elements are ``rho > density_floor`` (same frozen/fixed/void region
  overrides), each element stiffness is scaled by ``rho_eff**p`` with
  ``rho_eff = clip(rho, density_floor, 1)`` (frozen/fixed at 1.0), and the
  result carries ``element_energy`` (the SIMP sensitivity kernel) and
  ``compliance``. When ``simp_penalty`` is None the binary path is
  bit-for-bit unchanged.
* **Q1 hex8.** 8-node trilinear brick, 24 DOF/element, isotropic linear
  elasticity, 2x2x2 Gauss-Legendre quadrature. Cube edge ``h`` in metres.
* **SI throughout.** Forces in N, lengths in m, stress in Pa.

The element-stiffness derivation, B-matrix, and von Mises formula are
self-contained and validated against two independent closed-form cases in
``tests/test_reference_fea.py`` (end-loaded cantilever; pure axial tension).

Dependencies: numpy + scipy only. No network, no ``agents.*`` imports.
"""

from __future__ import annotations

import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla

from .selectors import resolve_selector

__all__ = ["reference_fea", "reference_modal"]

METHOD = "reference_hex8_linear_elastic_binary_occupancy"
METHOD_SIMP = "reference_hex8_linear_elastic_simp_density"
METHOD_MODAL = "reference_hex8_modal_lumped"

# Local corner ordering used throughout. Element (i,j,k) corner `n` sits at
# grid node (i+CORNER[n][0], j+CORNER[n][1], k+CORNER[n][2]). The order is the
# lexicographic (di, dj, dk) order matching the analyzer.system.md §5 pattern.
_CORNER_OFFSETS = np.array(
    [(di, dj, dk) for di in (0, 1) for dj in (0, 1) for dk in (0, 1)],
    dtype=np.int64,
)  # shape (8, 3); natural coords for corner n are (2*di-1, 2*dj-1, 2*dk-1)


# ---------------------------------------------------------------------------
# Constitutive matrix (isotropic 3-D, Voigt order [xx,yy,zz,yz,xz,xy])
# ---------------------------------------------------------------------------

def _elastic_D(E: float, nu: float) -> np.ndarray:
    """6x6 isotropic stiffness in Voigt order [xx, yy, zz, yz, xz, xy]."""
    c = E / ((1.0 + nu) * (1.0 - 2.0 * nu))
    D = np.zeros((6, 6))
    D[0, 0] = D[1, 1] = D[2, 2] = c * (1.0 - nu)
    D[0, 1] = D[0, 2] = D[1, 0] = D[1, 2] = D[2, 0] = D[2, 1] = c * nu
    g = c * (1.0 - 2.0 * nu) / 2.0  # shear modulus term = E / (2(1+nu))
    D[3, 3] = D[4, 4] = D[5, 5] = g
    return D


# ---------------------------------------------------------------------------
# hex8 shape-function gradients and B-matrix
# ---------------------------------------------------------------------------

def _shape_grad_natural(xi: float, eta: float, zeta: float) -> np.ndarray:
    """dN/d(xi,eta,zeta) for the 8 trilinear shape functions.

    Returns an (8, 3) array; row n is (dN_n/dxi, dN_n/deta, dN_n/dzeta).
    Corner n has natural coords s = 2*offset - 1 (each component +/-1).
    """
    signs = 2 * _CORNER_OFFSETS - 1  # (8,3) of +/-1
    sx, sy, sz = signs[:, 0], signs[:, 1], signs[:, 2]
    dN = np.empty((8, 3))
    dN[:, 0] = 0.125 * sx * (1.0 + sy * eta) * (1.0 + sz * zeta)
    dN[:, 1] = 0.125 * sy * (1.0 + sx * xi) * (1.0 + sz * zeta)
    dN[:, 2] = 0.125 * sz * (1.0 + sx * xi) * (1.0 + sy * eta)
    return dN


def _B_matrix(dN_xyz: np.ndarray) -> np.ndarray:
    """Strain-displacement matrix B (6 x 24) from dN/d(x,y,z) (8 x 3).

    DOF ordering within the element: [u0x,u0y,u0z, u1x,u1y,u1z, ...]. Voigt
    strain order is [eps_xx, eps_yy, eps_zz, gamma_yz, gamma_xz, gamma_xy].
    """
    B = np.zeros((6, 24))
    for n in range(8):
        dx, dy, dz = dN_xyz[n]
        col = 3 * n
        # eps_xx
        B[0, col + 0] = dx
        # eps_yy
        B[1, col + 1] = dy
        # eps_zz
        B[2, col + 2] = dz
        # gamma_yz = du_y/dz + du_z/dy
        B[3, col + 1] = dz
        B[3, col + 2] = dy
        # gamma_xz = du_x/dz + du_z/dx
        B[4, col + 0] = dz
        B[4, col + 2] = dx
        # gamma_xy = du_x/dy + du_y/dx
        B[5, col + 0] = dy
        B[5, col + 1] = dx
    return B


def _hex8_Ke(E: float, nu: float, h: float) -> np.ndarray:
    """24x24 element stiffness for a cube of edge `h` via 2x2x2 Gauss.

    For an axis-aligned cube of edge h the Jacobian is constant:
    J = (h/2) I, so dN/dx = (2/h) dN/dxi and det(J) = (h/2)^3.
    """
    D = _elastic_D(E, nu)
    Ke = np.zeros((24, 24))
    g = 1.0 / np.sqrt(3.0)  # 2-point Gauss abscissa; weights are all 1
    gp = (-g, g)
    detJ = (h / 2.0) ** 3
    inv = 2.0 / h  # d/dx = (2/h) d/dxi
    for xi in gp:
        for eta in gp:
            for zeta in gp:
                dN_nat = _shape_grad_natural(xi, eta, zeta)
                dN_xyz = dN_nat * inv  # (8,3) constant-Jacobian map
                B = _B_matrix(dN_xyz)
                Ke += (B.T @ D @ B) * detJ  # weights = 1
    return Ke


def _hex8_lumped_mass_diag(density: float, h: float) -> np.ndarray:
    """Diagonal of the 24x24 lumped (row-sum / HRZ) hex8 mass matrix.

    The element mass is ``m = density * h**3``. For a trilinear brick the
    consistent mass matrix rows sum to ``m/8`` per node (the integral of each
    shape function over the element is the element volume / 8). Lumping by
    row-sum therefore places ``m/8`` of mass on each of the 8 corner nodes,
    distributed identically across the three translational DOFs of that node.

    A lumped (diagonal) mass matrix is used rather than the consistent mass
    matrix because: (1) it is positive-definite by construction, (2) it makes
    the generalised eigenproblem K phi = lambda M phi reduce to a standard
    symmetric problem after an M^{-1/2} scaling (cheap, robust), and (3) for
    the lowest natural frequencies a row-sum lumped mass on a regular hex grid
    is accurate to within the coarse-mesh discretisation error of K itself
    (it slightly *over*-estimates the lowest frequency, the same order as the
    stiffness over-prediction of a coarse hex8 mesh). Returns a length-24
    vector (every entry equals ``m/8``).
    """
    m_node = density * (h ** 3) / 8.0
    return np.full(24, m_node, dtype=float)


def _B_centroid(h: float) -> np.ndarray:
    """B-matrix evaluated at the element centroid (natural 0,0,0)."""
    dN_nat = _shape_grad_natural(0.0, 0.0, 0.0)
    dN_xyz = dN_nat * (2.0 / h)
    return _B_matrix(dN_xyz)


def _von_mises(sigma: np.ndarray) -> float:
    """von Mises from a Voigt stress [xx, yy, zz, yz, xz, xy]."""
    sxx, syy, szz, syz, sxz, sxy = sigma
    return float(np.sqrt(
        0.5 * ((sxx - syy) ** 2 + (syy - szz) ** 2 + (szz - sxx) ** 2)
        + 3.0 * (syz ** 2 + sxz ** 2 + sxy ** 2)
    ))


# ---------------------------------------------------------------------------
# Occupancy
# ---------------------------------------------------------------------------

def _occupancy(rho: np.ndarray, region_kind: np.ndarray | None,
               *, simp_floor: float | None = None) -> np.ndarray:
    """Active-element mask.

    Binary rule (``simp_floor is None``): ``rho >= 0.5``. SIMP rule:
    ``rho > simp_floor`` (strict — an element AT the floor carries no
    meaningful stiffness and is excluded). In both modes ``region_kind``
    ``frozen``/``fixed`` are forced occupied and ``void`` forced empty.
    """
    if simp_floor is None:
        occ = np.asarray(rho) >= 0.5
    else:
        occ = np.asarray(rho) > simp_floor
    if region_kind is not None:
        rk = np.asarray(region_kind)
        if rk.shape != occ.shape:
            raise ValueError(
                f"region_kind shape {rk.shape} != rho shape {occ.shape}")
        occ = occ | (rk == "frozen") | (rk == "fixed")
        occ = occ & ~(rk == "void")
    if simp_floor is not None:
        # SINGULARITY GUARD (SIMP mode): the strict activity threshold can
        # strand active islands whose only neighbours sit at the optimizer's
        # density floor (excluded) — free-floating components add rigid-body
        # modes that stall every Krylov solver (Jacobi/ILU/AMG all hit
        # maxiter on a production wing field before this guard existed).
        # Physically, an island not connected to the frozen/fixed structure
        # carries no load: exclude it from the solve; its zero strain energy
        # then prunes it in the next optimizer update. 6-connectivity.
        from scipy import ndimage as _ndi
        lab, _n = _ndi.label(occ, structure=_ndi.generate_binary_structure(3, 1))
        if region_kind is not None:
            anchor = ((np.asarray(region_kind) == "frozen")
                      | (np.asarray(region_kind) == "fixed")) & occ
        else:
            anchor = np.zeros_like(occ)
        if not anchor.any():
            # no frozen/fixed material (e.g. an all-design test slab):
            # anchor the LARGEST component — fixtures resolve later and a
            # disconnected sliver would still be caught by the solver guard
            sizes = np.bincount(lab.ravel())
            sizes[0] = 0
            if sizes.max() > 0:
                anchor = lab == sizes.argmax()
        keep_ids = np.unique(lab[anchor])
        keep_ids = keep_ids[keep_ids > 0]
        occ = occ & np.isin(lab, keep_ids)
    return occ


# ---------------------------------------------------------------------------
# Exposed faces (for pressure loads)
# ---------------------------------------------------------------------------

# Face directions and the local corner indices that bound each face, in the
# _CORNER_OFFSETS ordering. Corner index = 4*di + 2*dj + dk.
def _corner_index(di: int, dj: int, dk: int) -> int:
    return 4 * di + 2 * dj + dk


_FACES = {
    # (axis, +/-1): (neighbour offset, [4 local corner indices on that face])
    ("x", -1): ((-1, 0, 0), [_corner_index(0, dj, dk) for dj in (0, 1) for dk in (0, 1)]),
    ("x", +1): ((+1, 0, 0), [_corner_index(1, dj, dk) for dj in (0, 1) for dk in (0, 1)]),
    ("y", -1): ((0, -1, 0), [_corner_index(di, 0, dk) for di in (0, 1) for dk in (0, 1)]),
    ("y", +1): ((0, +1, 0), [_corner_index(di, 1, dk) for di in (0, 1) for dk in (0, 1)]),
    ("z", -1): ((0, 0, -1), [_corner_index(di, dj, 0) for di in (0, 1) for dj in (0, 1)]),
    ("z", +1): ((0, 0, +1), [_corner_index(di, dj, 1) for di in (0, 1) for dj in (0, 1)]),
}
_AXIS_VEC = {"x": np.array([1.0, 0.0, 0.0]),
             "y": np.array([0.0, 1.0, 0.0]),
             "z": np.array([0.0, 0.0, 1.0])}


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def reference_fea(rho: np.ndarray,
                  region_kind: np.ndarray | None,
                  voxel_size_mm: float,
                  material: dict,
                  loads: list[dict],
                  fixtures: list[dict],
                  simp_penalty: float | None = None,
                  density_floor: float = 0.02,
                  return_element_energy: bool = False,
                  **kwargs) -> dict:
    """Independent reference hex8 linear-elastic FEA on a voxel field.

    Parameters
    ----------
    rho : (nx, ny, nz) array
        Density / occupancy field. Active elements are ``rho >= 0.5``
        (binary mode) or ``rho > density_floor`` (SIMP mode).
    region_kind : (nx, ny, nz) string array or None
        ``{frozen, fixed, design, void}`` per voxel. ``frozen``/``fixed`` are
        forced solid; ``void`` forced empty. ``None`` skips this override.
    voxel_size_mm : float
        Cube edge length in mm. Internally converted to metres.
    material : dict
        Canonical keys: ``youngs_modulus_pa``, ``poisson``, ``density_kg_m3``.
    loads : list[dict]
        Each: ``kind`` in {point, body, pressure, moment}, ``magnitude``,
        ``direction`` (3-vector, for point/body), ``region_selector``.
    fixtures : list[dict]
        Each: ``kind`` in {clamped, pinned, slider}, ``region_selector``,
        optional ``dof_constrained``.
    simp_penalty : float or None
        ``None`` (default) keeps the exact binary as-built behaviour —
        bit-for-bit identical to calls that omit the kwarg. A float ``p``
        (typically 3.0) enables SIMP semantics: active elements are
        ``rho > density_floor`` plus frozen/fixed (always solid), minus void
        (always excluded); each element stiffness is scaled by
        ``rho_eff**p`` where ``rho_eff = clip(rho, density_floor, 1.0)`` for
        design elements and exactly 1.0 for frozen/fixed. Point and pressure
        loads are unchanged; **body loads use the physical (unpenalised)
        rho** for the tributary mass, so gravity/self-weight sees real mass
        while stiffness sees the penalised interpolation.
    density_floor : float
        SIMP-mode activity threshold and lower clip for ``rho_eff``
        (default 0.02). Ignored in binary mode. Elements at or below the
        floor are excluded (strict ``>``); active design elements never fall
        below ``rho_eff = density_floor`` so the stiffness matrix stays
        non-degenerate.
    return_element_energy : bool
        Binary-mode only flag (default False) to also compute
        ``element_energy``/``compliance`` on the binary path. In SIMP mode
        these keys are always returned.
    origin_mm : 3-vector, optional kwarg
        World coordinate of node (0,0,0). Default (0,0,0).

    Returns
    -------
    dict with keys: ``max_von_mises_pa``, ``max_displacement_m``,
    ``tip_displacement_m``, ``stress_field`` (nx,ny,nz float32),
    ``disp_field`` (nx,ny,nz float32), ``n_active_elements``, ``n_dof``,
    ``method``, ``notes``. In SIMP mode (or with
    ``return_element_energy=True``) two more keys:

    ``element_energy`` : (nx, ny, nz) float64
        Per-element strain energy ``U_e = 0.5 * u_e^T K_e(rho_eff) u_e``
        (zeros for inactive elements), computed element-wise from the solved
        displacement — no extra global matrices. This is THE SIMP
        sensitivity kernel: the optimizer forms
        ``dC/drho_e = -(p / rho_e) * 2 * U_e`` from it.
    ``compliance`` : float
        Total compliance ``f . u`` (== ``2 * sum(U_e)`` by the work-energy
        identity; the pair is a built-in consistency check).
    """
    notes: list[str] = []
    origin_mm = kwargs.get("origin_mm", (0.0, 0.0, 0.0))

    rho = np.asarray(rho)
    if rho.ndim != 3:
        raise ValueError(f"rho must be 3-D, got shape {rho.shape}")
    shape = rho.shape
    nx, ny, nz = shape

    E = float(material["youngs_modulus_pa"])
    nu = float(material["poisson"])
    density = float(material.get("density_kg_m3", 0.0))
    h = float(voxel_size_mm) * 1e-3  # metres
    voxel_vol = h ** 3

    simp = simp_penalty is not None
    if simp:
        p = float(simp_penalty)
        if not np.isfinite(p) or p <= 0.0:
            raise ValueError(
                f"reference_fea: simp_penalty must be a positive finite "
                f"float or None, got {simp_penalty!r}")
        floor = float(density_floor)
        if not (0.0 < floor < 1.0):
            raise ValueError(
                f"reference_fea: density_floor must be in (0, 1), got "
                f"{density_floor!r}")

    occ = _occupancy(rho, region_kind,
                     simp_floor=(floor if simp else None))
    n_active = int(occ.sum())
    if n_active == 0:
        rule = (f"rho > {floor:g}" if simp else "rho >= 0.5")
        raise ValueError(
            f"reference_fea: no active elements ({rule} is empty after "
            f"applying region_kind). Nothing to analyse.")

    # --- number nodes touching an active element -------------------------
    node_id = -np.ones((nx + 1, ny + 1, nz + 1), dtype=np.int64)
    occ_corner = np.zeros((nx + 1, ny + 1, nz + 1), dtype=bool)
    ei, ej, ek = np.where(occ)
    for di, dj, dk in _CORNER_OFFSETS:
        occ_corner[ei + di, ej + dj, ek + dk] = True
    active_nodes = np.where(occ_corner)
    n_nodes = active_nodes[0].size
    node_id[active_nodes] = np.arange(n_nodes, dtype=np.int64)
    n_dof = 3 * n_nodes

    # element -> 8 global node ids (n_active, 8)
    elem_node_ids = np.empty((n_active, 8), dtype=np.int64)
    for n, (di, dj, dk) in enumerate(_CORNER_OFFSETS):
        elem_node_ids[:, n] = node_id[ei + di, ej + dj, ek + dk]

    # element -> 24 global dofs (n_active, 24)
    elem_dofs = (3 * elem_node_ids[:, :, None]
                 + np.arange(3)[None, None, :]).reshape(n_active, 24)

    # --- SIMP density interpolation ---------------------------------------
    # stiff_scale (n_active,): per-element stiffness multiplier rho_eff**p.
    # rho_phys (grid): physical density in [0,1] for body-load mass — NOT
    # penalised, so self-weight sees real mass while stiffness sees SIMP.
    # Both are None in binary mode, keeping that path bit-for-bit unchanged.
    stiff_scale: np.ndarray | None = None
    rho_phys: np.ndarray | None = None
    if simp:
        rho_f = np.asarray(rho, dtype=np.float64)
        rho_eff = np.clip(rho_f[ei, ej, ek], floor, 1.0)
        rho_phys = np.clip(rho_f, 0.0, 1.0)
        if region_kind is not None:
            rk = np.asarray(region_kind)
            solid = (rk == "frozen") | (rk == "fixed")
            rho_eff = np.where(solid[ei, ej, ek], 1.0, rho_eff)
            rho_phys = np.where(solid, 1.0, rho_phys)
        stiff_scale = rho_eff ** p
        notes.append(
            f"SIMP mode: active = rho > {floor:g} (frozen/fixed forced "
            f"solid, void excluded); element stiffness scaled by rho_eff^"
            f"{p:g}, rho_eff = clip(rho, {floor:g}, 1) (frozen/fixed at "
            f"1.0); body loads use physical rho; stress reported as the "
            f"homogenized rho_eff^p * D B u.")

    # --- element stiffness + global assembly -----------------------------
    Ke = _hex8_Ke(E, nu, h)  # identical for every cube (up to SIMP scaling)
    # Vectorised COO triplets: 24x24 block per element.
    rows = np.repeat(elem_dofs, 24, axis=1).reshape(-1)          # (n*576,)
    cols = np.tile(elem_dofs, (1, 24)).reshape(-1)               # (n*576,)
    if stiff_scale is None:
        vals = np.tile(Ke.reshape(-1), n_active)                 # (n*576,)
    else:
        vals = (stiff_scale[:, None] * Ke.reshape(-1)[None, :]).reshape(-1)
    K = sp.coo_matrix((vals, (rows, cols)),
                      shape=(n_dof, n_dof)).tocsr()

    # --- node world coordinates (m) for load distribution ----------------
    ox, oy, oz = (float(v) for v in origin_mm)
    # global node index -> (a,b,c) corner index
    corner_idx = np.array(active_nodes).T  # (n_nodes, 3)

    # --- loads -----------------------------------------------------------
    F = np.zeros(n_dof)
    loaded_nodes: set[int] = set()
    for li, load in enumerate(loads or []):
        kind = load.get("kind")
        sel = load.get("region_selector", {"type": "all"})
        mag = float(load.get("magnitude", 0.0))
        try:
            elem_mask = resolve_selector(sel, shape, float(voxel_size_mm),
                                         origin_mm)
        except NotImplementedError as exc:
            notes.append(f"load[{li}] ({kind}): selector unverifiable — {exc}")
            continue
        elem_mask = elem_mask & occ  # only active elements bear load

        if kind == "point":
            _apply_point_load(load, mag, elem_mask, node_id, F, loaded_nodes,
                              notes, li)
        elif kind == "body":
            _apply_body_load(load, mag, elem_mask, node_id, F, density,
                             voxel_vol, loaded_nodes, notes, li,
                             elem_rho=rho_phys)
        elif kind == "pressure":
            _apply_pressure_load(mag, sel, elem_mask, occ, node_id, h, F,
                                 loaded_nodes, notes, li, shape,
                                 float(voxel_size_mm), origin_mm)
        elif kind == "moment":
            notes.append(
                f"load[{li}] (moment): C0 hex8 has no rotational DOFs; moment "
                f"loads are NOT applied by the reference solver. Treat any "
                f"criterion depending on this load as unverifiable.")
        else:
            notes.append(f"load[{li}]: unknown kind {kind!r}; skipped.")

    # --- fixtures --------------------------------------------------------
    fixed_dofs: set[int] = set()
    for fi, fx in enumerate(fixtures or []):
        kind = fx.get("kind")
        sel = fx.get("region_selector", {"type": "all"})
        try:
            elem_mask = resolve_selector(sel, shape, float(voxel_size_mm),
                                         origin_mm)
        except NotImplementedError as exc:
            notes.append(
                f"fixture[{fi}] ({kind}): selector unverifiable — {exc}")
            continue
        elem_mask = elem_mask & occ
        node_ids = _selected_node_ids(elem_mask, node_id)
        if node_ids.size == 0:
            notes.append(
                f"fixture[{fi}] ({kind}): selector matched no active nodes.")
            continue

        dof_constrained = fx.get("dof_constrained")
        comps = _fixture_components(kind, sel, dof_constrained, notes, fi)
        for c in comps:
            fixed_dofs.update((3 * node_ids + c).tolist())

    if not fixed_dofs:
        raise ValueError(
            "reference_fea: no DOFs constrained by any fixture — the system "
            "is rigid-body singular. Check that the fixtures' selectors match "
            "active material.")

    free = np.setdiff1d(np.arange(n_dof), np.fromiter(fixed_dofs, dtype=np.int64))
    if free.size == 0:
        raise ValueError("reference_fea: all DOFs are fixed; nothing to solve.")

    # --- solve -----------------------------------------------------------
    Kff = K[free][:, free].tocsc()
    # Large systems: a direct factorization of >~250k free DOFs needs more
    # RAM than a typical workstation has free (a production SIMP wing solve
    # thrashed a 16 GB machine into swap at 450k DOF). Use Jacobi-
    # preconditioned CG above the threshold — standard practice for SIMP-
    # scale voxel elasticity (top3d lineage). Honest failure: CG that does
    # not converge RAISES; it never silently returns a bad solution.
    _n_free = int(Kff.shape[0])
    _cg_threshold = int(kwargs.get("direct_solver_max_dof", 250_000))
    Ff = F[free]
    if not np.any(F):
        notes.append("no nonzero nodal forces were assembled — check loads.")
    try:
        if _n_free <= _cg_threshold:
            uf = spla.spsolve(Kff, Ff)
        else:
            _diag = Kff.diagonal()
            if not np.all(_diag > 0):
                raise RuntimeError(
                    "reference_fea: non-positive stiffness diagonal — cannot "
                    "precondition; check density floor / region encoding.")
            _M = sp.diags(1.0 / _diag)
            uf, _info = spla.cg(Kff, Ff, M=_M, rtol=1e-8, atol=0.0,
                                maxiter=20_000)
            if _info != 0:
                # High-contrast SIMP elasticity defeats one-level
                # preconditioners (Jacobi stalled at 20k iters, ILU at 5k on a
                # production wing field). Escalate to smoothed-aggregation
                # ALGEBRAIC MULTIGRID — the standard solver class for
                # voxel-elasticity/SIMP systems — as a CG preconditioner.
                try:
                    import pyamg
                    # elasticity AMG NEEDS the rigid-body near-nullspace:
                    # 6 modes (3 translations + 3 rotations) evaluated at the
                    # free DOFs' node coordinates — without B, smoothed
                    # aggregation stalls exactly like a one-level method.
                    _free_nodes = free // 3
                    _comp = free % 3
                    # node coordinates from the active-node grid indices
                    _nix = np.asarray(active_nodes[0], dtype=float)
                    _niy = np.asarray(active_nodes[1], dtype=float)
                    _niz = np.asarray(active_nodes[2], dtype=float)
                    _coords = np.stack([
                        ox + _nix * h, oy + _niy * h, oz + _niz * h,
                    ], axis=1)[_free_nodes]  # (n_free, 3) mm
                    _B = np.zeros((_n_free, 6))
                    for _d in range(3):
                        _B[_comp == _d, _d] = 1.0
                    _x, _y, _z = _coords[:, 0], _coords[:, 1], _coords[:, 2]
                    _rot = {0: (1, _z, 2, -_y), 1: (2, _x, 0, -_z), 2: (0, _y, 1, -_x)}
                    for _m, (_c1, _v1, _c2, _v2) in _rot.items():
                        _sel1 = _comp == _c1
                        _sel2 = _comp == _c2
                        _B[_sel1, 3 + _m] = _v1[_sel1]
                        _B[_sel2, 3 + _m] = _v2[_sel2]
                    _ml = pyamg.smoothed_aggregation_solver(
                        Kff.tocsr(), B=_B, max_coarse=500, strength="symmetric")
                    _Mamg = _ml.aspreconditioner(cycle="V")
                    uf, _info = spla.cg(Kff, Ff, M=_Mamg, rtol=1e-8, atol=0.0,
                                        maxiter=2_000)
                except ImportError as _exc:
                    raise RuntimeError(
                        f"reference_fea: Jacobi-CG stalled at {_n_free} DOFs "
                        f"and pyamg is unavailable ({_exc}) — install pyamg or "
                        "reduce the grid.") from _exc
                except MemoryError as _exc:
                    raise RuntimeError(
                        f"reference_fea: AMG setup exceeded memory at "
                        f"{_n_free} DOFs: {_exc}") from _exc
                if _info != 0:
                    raise RuntimeError(
                        f"reference_fea: CG did not converge (Jacobi then AMG, "
                        f"info={_info}) at {_n_free} DOFs — refusing an "
                        "unconverged solution. Reduce the grid or raise "
                        "density_floor.")
                notes.append(
                    f"iterative solve: AMG-CG at {_n_free} free DOFs "
                    f"(Jacobi stalled), rtol 1e-8.")
            else:
                notes.append(
                    f"iterative solve: Jacobi-CG at {_n_free} free DOFs "
                    f"(direct threshold {_cg_threshold}), rtol 1e-8.")
    except RuntimeError:
        raise
    except Exception as exc:  # noqa: BLE001 — surface any solver failure
        raise RuntimeError(f"reference_fea: linear solve failed: {exc}") from exc
    if not np.all(np.isfinite(uf)):
        raise RuntimeError(
            "reference_fea: solution contains non-finite values — the system "
            "is likely under-constrained / singular (insufficient fixtures to "
            "remove rigid-body modes).")

    u = np.zeros(n_dof)
    u[free] = uf

    # --- per-element peak stress at the 2x2x2 Gauss points ----------------
    # Sampling at the Gauss points (not the element centroid) captures the
    # bending-stress gradient: the centroid under-predicts peak surface
    # stress by ~20% on coarse meshes — the UNSAFE direction for a verifier
    # (it would call a near-yield part safe). Gauss points sit inboard of
    # the corners, so they avoid the spurious spikes that nodal/corner
    # sampling produces at point-load and re-entrant singularities. We take
    # the max von Mises over the 8 points per element.
    D = _elastic_D(E, nu)
    u_elem = u[elem_dofs]               # (n_active, 24)
    g = 1.0 / np.sqrt(3.0)
    vm = np.zeros(n_active)
    for xi in (-g, g):
        for eta in (-g, g):
            for zeta in (-g, g):
                Bg = _B_matrix(_shape_grad_natural(xi, eta, zeta) * (2.0 / h))
                s = (u_elem @ Bg.T) @ D.T                # (n_active, 6) Voigt
                vm_gp = np.sqrt(
                    0.5 * ((s[:, 0] - s[:, 1]) ** 2
                           + (s[:, 1] - s[:, 2]) ** 2
                           + (s[:, 2] - s[:, 0]) ** 2)
                    + 3.0 * (s[:, 3] ** 2 + s[:, 4] ** 2 + s[:, 5] ** 2))
                vm = np.maximum(vm, vm_gp)
    if stiff_scale is not None:
        # SIMP homogenized stress: sigma_e = rho_eff^p * D B u_e. Reporting
        # the un-scaled solid-material stress at floor-density elements would
        # show huge fictitious peaks (their strains are large precisely
        # because they carry almost no load).
        vm = vm * stiff_scale

    stress_field = np.zeros(shape, dtype=np.float32)
    stress_field[ei, ej, ek] = vm.astype(np.float32)

    # --- nodal displacement magnitude back-projected to elements ---------
    u_nodes = u.reshape(n_nodes, 3)
    node_disp_mag = np.linalg.norm(u_nodes, axis=1)  # per active node
    # element disp = max over its 8 corner nodes (conservative)
    elem_disp = node_disp_mag[elem_node_ids].max(axis=1)  # (n_active,)
    disp_field = np.zeros(shape, dtype=np.float32)
    disp_field[ei, ej, ek] = elem_disp.astype(np.float32)

    max_vm = float(vm.max()) if vm.size else 0.0
    max_disp = float(node_disp_mag.max()) if node_disp_mag.size else 0.0

    if loaded_nodes:
        loaded_arr = np.fromiter(loaded_nodes, dtype=np.int64)
        tip_disp = float(node_disp_mag[loaded_arr].max())
    else:
        tip_disp = 0.0
        notes.append("no loaded nodes recorded; tip_displacement_m is 0.")

    result = {
        "max_von_mises_pa": max_vm,
        "max_displacement_m": max_disp,
        "tip_displacement_m": tip_disp,
        "stress_field": stress_field,
        "disp_field": disp_field,
        "n_active_elements": n_active,
        "n_dof": int(n_dof),
        "method": METHOD_SIMP if simp else METHOD,
        "notes": notes,
    }

    # --- per-element strain energy (SIMP sensitivity kernel) --------------
    # U_e = 0.5 * u_e^T K_e(rho_eff) u_e, evaluated element-wise from the
    # already-solved displacement — no additional global matrices. Always
    # returned in SIMP mode; opt-in on the binary path.
    if simp or return_element_energy:
        Ue = 0.5 * np.einsum("ei,ei->e", u_elem @ Ke, u_elem)
        if stiff_scale is not None:
            Ue = Ue * stiff_scale
        element_energy = np.zeros(shape, dtype=np.float64)
        element_energy[ei, ej, ek] = Ue
        result["element_energy"] = element_energy
        # Compliance f.u from the assembled load vector; by the work-energy
        # identity this equals 2*sum(U_e) (u == 0 on fixed DOFs, so reaction
        # forces do no work) — the pair is a built-in consistency check.
        result["compliance"] = float(F @ u)

    return result


# ---------------------------------------------------------------------------
# Shared mesh assembly (used by both the static and modal reference solvers)
# ---------------------------------------------------------------------------

def _assemble_mesh(rho: np.ndarray,
                   region_kind: np.ndarray | None,
                   voxel_size_mm: float):
    """Build the active-element mesh: occupancy, node numbering, element->DOF.

    Returns ``(occ, ei, ej, ek, node_id, n_active, n_nodes, n_dof,
    elem_node_ids, elem_dofs)``. Uses the *same* binary-occupancy rule,
    corner ordering, and node-numbering as :func:`reference_fea` so the global
    stiffness produced here is identical to the static solver's.
    """
    rho = np.asarray(rho)
    if rho.ndim != 3:
        raise ValueError(f"rho must be 3-D, got shape {rho.shape}")
    nx, ny, nz = rho.shape

    occ = _occupancy(rho, region_kind)
    n_active = int(occ.sum())
    if n_active == 0:
        raise ValueError(
            "no active elements (rho >= 0.5 is empty after applying "
            "region_kind). Nothing to analyse.")

    node_id = -np.ones((nx + 1, ny + 1, nz + 1), dtype=np.int64)
    occ_corner = np.zeros((nx + 1, ny + 1, nz + 1), dtype=bool)
    ei, ej, ek = np.where(occ)
    for di, dj, dk in _CORNER_OFFSETS:
        occ_corner[ei + di, ej + dj, ek + dk] = True
    active_nodes = np.where(occ_corner)
    n_nodes = active_nodes[0].size
    node_id[active_nodes] = np.arange(n_nodes, dtype=np.int64)
    n_dof = 3 * n_nodes

    elem_node_ids = np.empty((n_active, 8), dtype=np.int64)
    for n, (di, dj, dk) in enumerate(_CORNER_OFFSETS):
        elem_node_ids[:, n] = node_id[ei + di, ej + dj, ek + dk]
    elem_dofs = (3 * elem_node_ids[:, :, None]
                 + np.arange(3)[None, None, :]).reshape(n_active, 24)

    return (occ, ei, ej, ek, node_id, n_active, n_nodes, n_dof,
            elem_node_ids, elem_dofs)


def _collect_fixed_dofs(fixtures, occ, node_id, shape, voxel_size_mm,
                        origin_mm, notes) -> set[int]:
    """Resolve fixtures to a set of constrained global DOF indices.

    Reuses :func:`reference_fea`'s fixture logic verbatim: selector resolution,
    intersection with active occupancy, node-id gathering, and
    :func:`_fixture_components` for the per-kind translation components.
    """
    fixed_dofs: set[int] = set()
    for fi, fx in enumerate(fixtures or []):
        kind = fx.get("kind")
        sel = fx.get("region_selector", {"type": "all"})
        try:
            elem_mask = resolve_selector(sel, shape, float(voxel_size_mm),
                                         origin_mm)
        except NotImplementedError as exc:
            notes.append(
                f"fixture[{fi}] ({kind}): selector unverifiable — {exc}")
            continue
        elem_mask = elem_mask & occ
        node_ids = _selected_node_ids(elem_mask, node_id)
        if node_ids.size == 0:
            notes.append(
                f"fixture[{fi}] ({kind}): selector matched no active nodes.")
            continue
        dof_constrained = fx.get("dof_constrained")
        comps = _fixture_components(kind, sel, dof_constrained, notes, fi)
        for c in comps:
            fixed_dofs.update((3 * node_ids + c).tolist())
    return fixed_dofs


# ---------------------------------------------------------------------------
# Modal (natural-frequency) reference solver
# ---------------------------------------------------------------------------

def reference_modal(rho: np.ndarray,
                    region_kind: np.ndarray | None,
                    voxel_size_mm: float,
                    material: dict,
                    fixtures: list[dict],
                    *,
                    n_modes: int = 6,
                    **kwargs) -> dict:
    """Independent reference hex8 free-vibration (modal) analysis.

    Solves the generalised eigenproblem ``K phi = lambda M phi`` on the FREE
    DOFs (fixtures removed) for the smallest ``n_modes`` eigenvalues, and
    reports natural frequencies ``f_i = sqrt(lambda_i) / (2*pi)`` in Hz.

    The global stiffness ``K`` is assembled identically to
    :func:`reference_fea` (same binary occupancy, same hex8 ``Ke``, same node
    numbering). The mass matrix ``M`` is a **lumped (row-sum) diagonal** hex8
    mass using ``density = material["density_kg_m3"]`` — see
    :func:`_hex8_lumped_mass_diag` for the rationale (positive-definite,
    robust, accurate for the lowest modes on a regular grid).

    Spurious / rigid-body modes: only the FREE DOFs are passed to the
    eigensolver, so a properly constrained part has no rigid-body modes. To be
    robust against residual numerical zeros (under-constrained directions,
    eigensolver noise) we discard any eigenvalue ``lambda <= tol`` where
    ``tol`` is a small positive fraction of the spectrum scale, and keep only
    the genuine positive-frequency modes.

    Parameters
    ----------
    rho, region_kind, voxel_size_mm, material, fixtures
        As in :func:`reference_fea` (no loads — modal analysis needs none).
    n_modes : int, keyword-only
        Number of (lowest, positive) modes to return. Default 6.
    origin_mm : 3-vector, optional kwarg
        World coordinate of node (0,0,0). Default (0,0,0).
    simp_penalty, density_floor, return_element_energy : optional kwargs
        Accepted for call-site symmetry with :func:`reference_fea` but
        **ignored** — modal analysis stays binary-occupancy. A SIMP-penalised
        modal solve would need a consistent interpolation of BOTH stiffness
        (rho**p) and mass (linear in rho), and at low rho that mismatch
        produces spurious localized low-frequency modes unless a dedicated
        low-density mass interpolation is added; that is out of scope for
        this verifier. Passing a non-None ``simp_penalty`` adds a note to
        the result instead of failing.

    Returns
    -------
    dict with keys ``frequencies_hz`` (ascending list of floats),
    ``first_mode_hz`` (float), ``n_modes`` (int, number actually returned),
    ``method`` (``"reference_hex8_modal_lumped"``), ``n_active_elements``,
    ``n_dof``, ``n_free_dof``, ``notes``.

    Raises
    ------
    ValueError
        If there are no active elements, no fixtures constrain any DOF, the
        free system is too small, or no positive modes can be extracted.
    """
    notes: list[str] = []
    origin_mm = kwargs.get("origin_mm", (0.0, 0.0, 0.0))
    if kwargs.get("simp_penalty") is not None:
        notes.append(
            "simp_penalty/density_floor ignored: reference_modal is "
            "binary-occupancy by design (penalized-stiffness + linear-mass "
            "modal analysis produces spurious low-rho local modes and is out "
            "of scope for this verifier).")

    rho = np.asarray(rho)
    if rho.ndim != 3:
        raise ValueError(f"rho must be 3-D, got shape {rho.shape}")
    shape = rho.shape

    E = float(material["youngs_modulus_pa"])
    nu = float(material["poisson"])
    density = float(material.get("density_kg_m3", 0.0))
    if density <= 0.0:
        raise ValueError(
            "reference_modal: material['density_kg_m3'] must be > 0 for a "
            "mass matrix; got {!r}.".format(material.get("density_kg_m3")))
    h = float(voxel_size_mm) * 1e-3  # metres

    try:
        n_modes = int(n_modes)
    except (TypeError, ValueError):
        raise ValueError(f"n_modes must be a positive int, got {n_modes!r}")
    if n_modes < 1:
        raise ValueError(f"n_modes must be >= 1, got {n_modes}")

    (occ, ei, ej, ek, node_id, n_active, n_nodes, n_dof,
     elem_node_ids, elem_dofs) = _assemble_mesh(rho, region_kind,
                                                 voxel_size_mm)
    if n_active < 2:
        raise ValueError(
            f"reference_modal: too few active elements ({n_active}) for a "
            f"meaningful modal analysis.")

    # --- global stiffness K (identical assembly to reference_fea) ----------
    Ke = _hex8_Ke(E, nu, h)
    rows = np.repeat(elem_dofs, 24, axis=1).reshape(-1)
    cols = np.tile(elem_dofs, (1, 24)).reshape(-1)
    kvals = np.tile(Ke.reshape(-1), n_active)
    K = sp.coo_matrix((kvals, (rows, cols)), shape=(n_dof, n_dof)).tocsr()

    # --- global lumped mass M (diagonal) -----------------------------------
    # Each element deposits m/8 on each of its 24 element DOFs; assembling
    # (summing) over elements gives the global lumped mass per DOF directly.
    me_diag = _hex8_lumped_mass_diag(density, h)            # (24,)
    m_global = np.zeros(n_dof, dtype=float)
    np.add.at(m_global, elem_dofs.reshape(-1),
              np.tile(me_diag, n_active))
    total_mass = density * (h ** 3) * n_active
    # Sanity: assembled lumped mass must equal sum over elements of m
    # (each DOF-direction carries the full element mass once).

    # --- fixtures -> fixed DOFs (reuse reference_fea logic) ----------------
    fixed_dofs = _collect_fixed_dofs(fixtures, occ, node_id, shape,
                                     voxel_size_mm, origin_mm, notes)
    if not fixed_dofs:
        raise ValueError(
            "reference_modal: no DOFs constrained by any fixture — the system "
            "has rigid-body modes and free-vibration frequencies are "
            "ill-defined. Provide at least one fixture.")

    free = np.setdiff1d(
        np.arange(n_dof), np.fromiter(fixed_dofs, dtype=np.int64))
    n_free = int(free.size)
    if n_free == 0:
        raise ValueError("reference_modal: all DOFs are fixed; nothing to solve.")
    if n_free <= n_modes:
        raise ValueError(
            f"reference_modal: only {n_free} free DOFs but {n_modes} modes "
            f"requested; mesh is too small / too constrained.")

    Kff = K[free][:, free].tocsr()
    Mff_diag = m_global[free]
    if not np.all(Mff_diag > 0):
        # A free DOF with zero lumped mass would make M singular and the
        # generalised problem ill-posed. With binary occupancy every active
        # node touches an active element, so this should not happen; guard
        # anyway with a tiny positive floor relative to the mean mass.
        floor = 1e-12 * float(Mff_diag[Mff_diag > 0].mean())
        nz = int((Mff_diag <= 0).sum())
        notes.append(
            f"{nz} free DOF(s) had zero lumped mass; floored to {floor:.3e} kg "
            f"to keep M positive-definite.")
        Mff_diag = np.where(Mff_diag > 0, Mff_diag, floor)

    # --- standard-form reduction: lumped M is diagonal, so the generalised
    # symmetric problem K phi = lambda M phi maps exactly to the standard
    # symmetric eigenproblem A y = lambda y with A = D^{-1/2} K D^{-1/2},
    # phi = D^{-1/2} y, D = diag(M). A is symmetric positive (semi-)definite,
    # which is well conditioned for shift-invert about sigma=0.
    inv_sqrt_m = 1.0 / np.sqrt(Mff_diag)
    Dinv = sp.diags(inv_sqrt_m)
    A = (Dinv @ Kff @ Dinv).tocsc()
    A = 0.5 * (A + A.T)  # symmetrise away round-off asymmetry

    k_solve = min(n_modes + 4, n_free - 1)  # over-request to survive filtering
    eigvals = _smallest_eigs(A, k_solve, notes)

    # --- filter spurious / rigid-body / negative eigenvalues ---------------
    # Eigenvalues are omega^2 (rad/s)^2. Genuine modes are strictly positive;
    # discard anything <= a small positive tol scaled by the spectrum so we
    # don't admit numerical zeros as ~0 Hz "modes".
    scale = float(np.max(np.abs(eigvals))) if eigvals.size else 1.0
    tol = max(scale * 1e-8, 1e-9)
    positive = np.sort(eigvals[eigvals > tol])
    n_discarded = int(eigvals.size - positive.size)
    if n_discarded:
        notes.append(
            f"discarded {n_discarded} eigenvalue(s) <= {tol:.3e} "
            f"(rigid-body / numerical-noise modes).")
    if positive.size == 0:
        raise ValueError(
            "reference_modal: no positive eigenvalues found — the constrained "
            "system appears to have no genuine vibration modes (likely "
            "under-constrained or degenerate).")

    omega = np.sqrt(positive)
    freqs = (omega / (2.0 * np.pi)).astype(float)
    freqs = np.sort(freqs)[:n_modes]

    notes.append(
        f"lumped (row-sum) hex8 mass; total active mass {total_mass:.4e} kg; "
        f"standard-form eigensolve on {n_free} free DOFs.")

    return {
        "frequencies_hz": [float(f) for f in freqs],
        "first_mode_hz": float(freqs[0]),
        "n_modes": int(freqs.size),
        "method": METHOD_MODAL,
        "n_active_elements": n_active,
        "n_dof": int(n_dof),
        "n_free_dof": n_free,
        "notes": notes,
    }


def _smallest_eigs(A: sp.spmatrix, k: int, notes: list[str]) -> np.ndarray:
    """Smallest-magnitude eigenvalues of a symmetric PSD sparse matrix.

    Prefers shift-invert about sigma=0 (``eigsh(..., sigma=0, which='LM')``),
    which targets eigenvalues nearest zero — exactly the lowest natural
    frequencies — and converges far faster/more reliably than ``which='SM'``.
    Falls back to ``which='SA'`` (smallest algebraic) if the shift-invert
    factorisation fails (e.g. a near-singular A), and finally to a dense
    eigensolve for very small systems.
    """
    n = A.shape[0]
    k = max(1, min(k, n - 1))
    # For tiny systems a dense solve is both faster and more robust.
    if n <= 400:
        w = np.linalg.eigvalsh(A.toarray())
        return np.sort(w)[:k]
    try:
        w = spla.eigsh(A, k=k, sigma=0.0, which="LM",
                       return_eigenvectors=False)
        return np.sort(w)
    except Exception as exc:  # noqa: BLE001 — fall back to a robust path
        notes.append(
            f"shift-invert eigensolve failed ({type(exc).__name__}); "
            f"retrying with which='SA'.")
    try:
        w = spla.eigsh(A, k=k, which="SA", return_eigenvectors=False)
        return np.sort(w)
    except Exception as exc:  # noqa: BLE001
        notes.append(
            f"sparse eigensolve failed ({type(exc).__name__}); "
            f"falling back to dense eigvalsh.")
        w = np.linalg.eigvalsh(A.toarray())
        return np.sort(w)[:k]


# ---------------------------------------------------------------------------
# Load application helpers
# ---------------------------------------------------------------------------

def _selected_node_ids(elem_mask: np.ndarray, node_id: np.ndarray) -> np.ndarray:
    """Global node ids touching the selected (and active) elements."""
    nx, ny, nz = elem_mask.shape
    touched = np.zeros((nx + 1, ny + 1, nz + 1), dtype=bool)
    ii, jj, kk = np.where(elem_mask)
    if ii.size == 0:
        return np.empty(0, dtype=np.int64)
    for di, dj, dk in _CORNER_OFFSETS:
        touched[ii + di, jj + dj, kk + dk] = True
    ids = node_id[touched]
    return np.unique(ids[ids >= 0])


def _unit(vec) -> np.ndarray:
    v = np.asarray(vec, dtype=float)
    n = np.linalg.norm(v)
    if n == 0:
        return v
    return v / n


def _apply_point_load(load, mag, elem_mask, node_id, F, loaded_nodes,
                      notes, li):
    """Total force = magnitude * unit(direction), split equally over the
    selected nodes."""
    direction = _unit(load.get("direction", [0.0, 0.0, 0.0]))
    node_ids = _selected_node_ids(elem_mask, node_id)
    if node_ids.size == 0:
        notes.append(f"load[{li}] (point): selector matched no active nodes.")
        return
    f_total = mag * direction
    f_per = f_total / node_ids.size
    for c in range(3):
        np.add.at(F, 3 * node_ids + c, f_per[c])
    loaded_nodes.update(node_ids.tolist())


def _apply_body_load(load, mag, elem_mask, node_id, F, density, voxel_vol,
                     loaded_nodes, notes, li, elem_rho=None):
    """Body load: per-node force = magnitude(N/kg) * direction * tributary
    mass, where tributary mass at a node = density * voxel_vol/8 summed over
    the active elements that touch it.

    ``elem_rho`` (grid-shaped float array or None): SIMP mode passes the
    PHYSICAL density field here so each element's mass is
    ``density * voxel_vol * rho_phys`` — unpenalised, per the SIMP contract
    that mass is linear in rho while stiffness is rho**p. ``None`` (binary
    mode) keeps the original full-density behaviour untouched.
    """
    direction = _unit(load.get("direction", [0.0, 0.0, 0.0]))
    nx, ny, nz = elem_mask.shape
    ii, jj, kk = np.where(elem_mask)
    if ii.size == 0:
        notes.append(f"load[{li}] (body): selector matched no active elements.")
        return
    if density <= 0.0:
        notes.append(
            f"load[{li}] (body): material density_kg_m3 is 0; body force is 0.")
    nodal_mass = np.zeros((nx + 1, ny + 1, nz + 1))
    if elem_rho is None:
        m_per_corner = density * voxel_vol / 8.0
    else:
        m_per_corner = (density * voxel_vol / 8.0
                        * np.asarray(elem_rho, dtype=np.float64)[ii, jj, kk])
    for di, dj, dk in _CORNER_OFFSETS:
        np.add.at(nodal_mass, (ii + di, jj + dj, kk + dk), m_per_corner)
    touched = nodal_mass > 0
    gids = node_id[touched]
    masses = nodal_mass[touched]
    keep = gids >= 0
    gids, masses = gids[keep], masses[keep]
    for c in range(3):
        np.add.at(F, 3 * gids + c, mag * direction[c] * masses)
    loaded_nodes.update(gids.tolist())


def _apply_pressure_load(mag, sel, elem_mask, occ, node_id, h, F,
                         loaded_nodes, notes, li, shape, voxel_size_mm,
                         origin_mm):
    """Pressure on the selected region's *exposed* faces.

    Approximation: for every active element in the selected region, the faces
    that are exposed (no active neighbour across them) carry a uniform pressure
    ``mag`` (Pa) acting along the *inward* face normal. The face force
    ``p * h^2`` is lumped equally onto its 4 corner nodes (the consistent nodal
    forces for a constant traction on a bilinear quad face). Sign convention:
    positive magnitude pushes inward (compressive), matching the usual
    "external pressure" meaning; a negative magnitude pulls outward (tension).

    Plane-selector restriction: when the selector is a ``plane``, only the
    exposed faces whose outward normal is along that plane's axis (and on the
    plane's ``side``) are loaded. This prevents a thin end-cap selection from
    spuriously pressurising the lateral boundary faces of the same elements.
    For any other selector kind, all exposed faces of the selected elements are
    loaded (correct for a closed shell/region). This is a reasonable
    engineering approximation; integrators needing exact tractions on a curved
    boundary should not rely on it.
    """
    nx, ny, nz = shape
    ii, jj, kk = np.where(elem_mask)
    if ii.size == 0:
        notes.append(f"load[{li}] (pressure): selector matched no active elements.")
        return

    restrict_axis = None
    restrict_sign = None
    if isinstance(sel, dict) and sel.get("type") == "plane":
        restrict_axis = sel.get("axis")
        restrict_sign = +1 if sel.get("side", "+") == "+" else -1

    face_force = mag * h * h  # N per exposed face
    n_faces = 0
    for (axis, sign), (offset, local_corners) in _FACES.items():
        if restrict_axis is not None and (
                axis != restrict_axis or sign != restrict_sign):
            continue
        ni, nj, nk = ii + offset[0], jj + offset[1], kk + offset[2]
        in_bounds = (
            (ni >= 0) & (ni < nx) & (nj >= 0) & (nj < ny) &
            (nk >= 0) & (nk < nz)
        )
        neighbour_active = np.zeros(ii.shape, dtype=bool)
        neighbour_active[in_bounds] = occ[ni[in_bounds], nj[in_bounds],
                                          nk[in_bounds]]
        exposed = ~neighbour_active  # boundary faces are exposed too
        if not exposed.any():
            continue
        # Inward normal = -outward normal. Outward normal for (axis, sign) is
        # sign * axis_unit. Pressure pushes inward => force along -sign*axis.
        normal = -sign * _AXIS_VEC[axis]
        f_per_node = face_force * normal / 4.0
        exi, exj, exk = ii[exposed], jj[exposed], kk[exposed]
        n_faces += exi.size
        for lc in local_corners:
            di, dj, dk = _CORNER_OFFSETS[lc]
            gids = node_id[exi + di, exj + dj, exk + dk]
            keep = gids >= 0
            gg = gids[keep]
            for c in range(3):
                np.add.at(F, 3 * gg + c, f_per_node[c])
            loaded_nodes.update(gg.tolist())
    if n_faces == 0:
        notes.append(
            f"load[{li}] (pressure): no exposed faces found in selected region.")
    else:
        notes.append(
            f"load[{li}] (pressure): applied over {n_faces} exposed faces "
            f"(uniform-traction lumped-to-corners approximation, inward normal).")


# ---------------------------------------------------------------------------
# Fixture DOF inference
# ---------------------------------------------------------------------------

_DOF_INDEX = {"ux": 0, "uy": 1, "uz": 2}


def _fixture_components(kind, sel, dof_constrained, notes, fi) -> list[int]:
    """Translation DOF components (0,1,2) to fix for a fixture.

    C0 hex8 has only translational DOFs, so rotational constraints (rx,ry,rz)
    in ``dof_constrained`` are ignored (with a note). ``clamped`` and
    ``pinned`` both fix all three translations (no rotational DOFs to add).
    ``slider`` fixes only the normal translation, inferred from the plane
    selector's axis (or a documented fallback).
    """
    if dof_constrained:
        comps = []
        rot_seen = False
        for d in dof_constrained:
            if d in _DOF_INDEX:
                comps.append(_DOF_INDEX[d])
            else:
                rot_seen = True
        if rot_seen:
            notes.append(
                f"fixture[{fi}]: rotational DOFs in dof_constrained ignored "
                f"(C0 hex8 has translational DOFs only).")
        if comps:
            return sorted(set(comps))
        # dof_constrained listed only rotations -> fall through to kind default.

    if kind in ("clamped", "pinned"):
        return [0, 1, 2]
    if kind == "slider":
        # The slider's free plane is the surface it sits on; the constrained
        # DOF is the translation NORMAL to that plane. For a 'plane' selector
        # the normal is its axis ('x'->ux=0, etc.).
        axis = sel.get("axis") if isinstance(sel, dict) else None
        axis_to_comp = {"x": 0, "y": 1, "z": 2}
        if axis in axis_to_comp:
            notes.append(
                f"fixture[{fi}] (slider): fixed normal translation u{axis} "
                f"inferred from the plane selector axis.")
            return [axis_to_comp[axis]]
        notes.append(
            f"fixture[{fi}] (slider): no plane axis to infer the normal from; "
            f"defaulting to fixing uz. Provide a 'plane' selector or "
            f"dof_constrained for a definite normal.")
        return [2]
    # Unknown kind: be conservative, fix all translations.
    notes.append(f"fixture[{fi}]: unknown kind {kind!r}; fixed all translations.")
    return [0, 1, 2]
