"""buckling.py — independent linear (eigenvalue) buckling verifier for ACE.

A clean-room, benchmark-validated **hex8 linear buckling** analysis. The
orchestrator uses this to *verify* that a delivered part has an adequate
margin against elastic (Euler-type) instability under a given load, fully
independent of any per-part ``analysis.py`` and of any SIMP weighting.

Method (classical linearised pre-buckling / eigenvalue buckling)
----------------------------------------------------------------
1. Solve the linear static problem ``K u = F`` for the pre-stress
   displacement field, reusing the *exact* mesh + ``K`` assembly + load /
   fixture handling of :mod:`physics.fea` (no reimplementation).
2. Recover the per-element Cauchy stress state ``S`` (3x3) at each Gauss
   point and assemble the **geometric (initial-stress) stiffness** ``K_g``:

       Kg_e = integral_V ( G^T * S_hat * G ) dV

   where ``G`` (9x24) maps the 24 element DOFs to the 9 displacement-gradient
   components ``du_i/dx_j`` and ``S_hat`` is the 9x9 block-diagonal
   ``diag(S, S, S)``. 2x2x2 Gauss-Legendre quadrature on the cube of edge
   ``h = voxel_size_mm * 1e-3`` m.
3. Solve the generalised eigenproblem ``K phi = -lambda K_g phi`` for the
   smallest **positive** ``lambda`` (the buckling load factor). The critical
   load is ``applied_load * lambda``. Non-positive / spurious eigenvalues are
   filtered out.

Sign convention
---------------
``K_g`` is built from the actual Cauchy stress (compression negative). With
the eigenproblem written ``K phi = -lambda K_g phi``, a *compressive*
pre-stress (negative-definite contribution from ``K_g`` along the buckling
mode) yields a *positive* ``lambda``. We therefore keep the smallest strictly
positive ``lambda``; that root is, by construction, the compressive buckling
mode. A structure with no compressive stress state produces no positive
``lambda`` and we raise rather than report a meaningless number.

Dependencies: numpy + scipy only. No network, no ``agents.*`` imports.
"""

from __future__ import annotations

import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla

from . import fea
from .fea import (
    _CORNER_OFFSETS,
    _assemble_mesh,
    _collect_fixed_dofs,
    _elastic_D,
    _hex8_Ke,
    _shape_grad_natural,
    _B_matrix,
    _apply_point_load,
    _apply_body_load,
    _apply_pressure_load,
)
from .selectors import resolve_selector

__all__ = ["reference_buckling"]

METHOD = "reference_hex8_linear_eigenvalue_buckling"


# ---------------------------------------------------------------------------
# Geometric-stiffness building blocks
# ---------------------------------------------------------------------------

def _G_matrix(dN_xyz: np.ndarray) -> np.ndarray:
    """Displacement-gradient operator G (9 x 24) from dN/d(x,y,z) (8 x 3).

    Element DOF order matches :func:`fea._B_matrix`:
    ``[u0x, u0y, u0z, u1x, u1y, u1z, ...]``.

    The 9 output rows are the displacement gradients ordered as

        [ du_x/dx, du_x/dy, du_x/dz,
          du_y/dx, du_y/dy, du_y/dz,
          du_z/dx, du_z/dy, du_z/dz ]

    i.e. three consecutive rows per displacement component. This ordering is
    consistent with the 9x9 ``S_hat = diag(S, S, S)`` block layout (one 3x3
    Cauchy block acting on the gradient triple of each displacement
    component).
    """
    G = np.zeros((9, 24))
    for n in range(8):
        dx, dy, dz = dN_xyz[n]
        col = 3 * n
        # du_x/d{x,y,z}
        G[0, col + 0] = dx
        G[1, col + 0] = dy
        G[2, col + 0] = dz
        # du_y/d{x,y,z}
        G[3, col + 1] = dx
        G[4, col + 1] = dy
        G[5, col + 1] = dz
        # du_z/d{x,y,z}
        G[6, col + 2] = dx
        G[7, col + 2] = dy
        G[8, col + 2] = dz
    return G


def _S_hat(sigma_voigt: np.ndarray) -> np.ndarray:
    """9x9 block-diagonal initial-stress matrix diag(S, S, S).

    ``sigma_voigt`` is the Voigt stress [xx, yy, zz, yz, xz, xy] (the
    convention used throughout :mod:`physics.fea`). The 3x3 Cauchy
    tensor is

        S = [[sxx, sxy, sxz],
             [sxy, syy, syz],
             [sxz, syz, szz]]
    """
    sxx, syy, szz, syz, sxz, sxy = sigma_voigt
    S = np.array([[sxx, sxy, sxz],
                  [sxy, syy, syz],
                  [sxz, syz, szz]])
    Sh = np.zeros((9, 9))
    Sh[0:3, 0:3] = S
    Sh[3:6, 3:6] = S
    Sh[6:9, 6:9] = S
    return Sh


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def reference_buckling(rho: np.ndarray,
                       region_kind: np.ndarray | None,
                       voxel_size_mm: float,
                       material: dict,
                       loads: list[dict],
                       fixtures: list[dict],
                       *,
                       n_modes: int = 4,
                       **kwargs) -> dict:
    """Independent reference hex8 linear (eigenvalue) buckling analysis.

    Solves ``K u = F`` for the pre-stress state, assembles the geometric
    stiffness ``K_g`` from the recovered element stresses, and solves the
    generalised eigenproblem ``K phi = -lambda K_g phi`` for the smallest
    positive buckling load factor ``lambda``.

    Parameters
    ----------
    rho, region_kind, voxel_size_mm, material, loads, fixtures
        Identical meaning and conventions to :func:`physics.fea.reference_fea`.
        SI throughout (N, m, Pa); ``h = voxel_size_mm * 1e-3`` m. Material keys
        ``youngs_modulus_pa`` / ``poisson`` (``density_kg_m3`` only needed for
        body loads).
    n_modes : int, keyword-only
        Number of (lowest, positive) buckling factors to return. Default 4.
    origin_mm : 3-vector, optional kwarg
        World coordinate of node (0, 0, 0). Default (0, 0, 0).

    Returns
    -------
    dict with keys ``buckling_load_factor`` (smallest positive lambda),
    ``buckling_load_factors`` (ascending list), ``applied_reference_load_n``
    (sum of applied external force magnitude, if derivable; else None),
    ``critical_load_n`` (factor * applied load, if applied load is known),
    ``n_modes`` (number actually returned), ``method``, ``n_active_elements``,
    ``n_dof``, ``n_free_dof``, ``notes``.

    Raises
    ------
    ValueError
        On no active elements, no constraining fixtures, no applied load, no
        compressive stress anywhere, or no positive eigenvalue (no buckling
        mode under the given load — typically a tensile / shear-only state).
    """
    notes: list[str] = []
    origin_mm = kwargs.get("origin_mm", (0.0, 0.0, 0.0))

    rho = np.asarray(rho)
    if rho.ndim != 3:
        raise ValueError(f"rho must be 3-D, got shape {rho.shape}")
    shape = rho.shape

    E = float(material["youngs_modulus_pa"])
    nu = float(material["poisson"])
    density = float(material.get("density_kg_m3", 0.0))
    h = float(voxel_size_mm) * 1e-3  # metres
    voxel_vol = h ** 3

    try:
        n_modes = int(n_modes)
    except (TypeError, ValueError):
        raise ValueError(f"n_modes must be a positive int, got {n_modes!r}")
    if n_modes < 1:
        raise ValueError(f"n_modes must be >= 1, got {n_modes}")

    # --- mesh assembly (identical occupancy / numbering as reference_fea) ---
    (occ, ei, ej, ek, node_id, n_active, n_nodes, n_dof,
     elem_node_ids, elem_dofs) = _assemble_mesh(rho, region_kind,
                                                 voxel_size_mm)
    if n_active < 2:
        raise ValueError(
            f"reference_buckling: too few active elements ({n_active}) for a "
            f"meaningful buckling analysis.")

    # --- global stiffness K (identical assembly to reference_fea) ----------
    Ke = _hex8_Ke(E, nu, h)
    rows = np.repeat(elem_dofs, 24, axis=1).reshape(-1)
    cols = np.tile(elem_dofs, (1, 24)).reshape(-1)
    kvals = np.tile(Ke.reshape(-1), n_active)
    K = sp.coo_matrix((kvals, (rows, cols)), shape=(n_dof, n_dof)).tocsr()

    # --- loads -> global force vector F (reuse reference_fea load helpers) --
    F = np.zeros(n_dof)
    loaded_nodes: set[int] = set()
    applied_force_vec = np.zeros(3)  # net externally-applied force (for ref load)
    applied_force_abs = 0.0          # sum of per-load |magnitude| applied
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
        elem_mask = elem_mask & occ

        F_before = F.copy()
        if kind == "point":
            _apply_point_load(load, mag, elem_mask, node_id, F, loaded_nodes,
                              notes, li)
        elif kind == "body":
            _apply_body_load(load, mag, elem_mask, node_id, F, density,
                             voxel_vol, loaded_nodes, notes, li)
        elif kind == "pressure":
            _apply_pressure_load(mag, sel, elem_mask, occ, node_id, h, F,
                                 loaded_nodes, notes, li, shape,
                                 float(voxel_size_mm), origin_mm)
        elif kind == "moment":
            notes.append(
                f"load[{li}] (moment): C0 hex8 has no rotational DOFs; moment "
                f"loads are NOT applied. Buckling under this load is "
                f"unverifiable by the reference solver.")
            continue
        else:
            notes.append(f"load[{li}]: unknown kind {kind!r}; skipped.")
            continue

        # Net applied force contributed by this load (sum of nodal forces it
        # added). Used to report an absolute critical load.
        dF = F - F_before
        applied_force_vec += dF.reshape(-1, 3).sum(axis=0)
        applied_force_abs += float(np.linalg.norm(dF.reshape(-1, 3).sum(axis=0)))

    if not np.any(F):
        raise ValueError(
            "reference_buckling: no nonzero nodal forces were assembled — a "
            "buckling load factor requires a reference load. Check the loads / "
            "selectors (moment loads are not applied by the C0 hex8 solver).")

    # --- fixtures -> fixed DOFs (reuse reference_fea logic) ----------------
    fixed_dofs = _collect_fixed_dofs(fixtures, occ, node_id, shape,
                                     voxel_size_mm, origin_mm, notes)
    if not fixed_dofs:
        raise ValueError(
            "reference_buckling: no DOFs constrained by any fixture — the "
            "system is rigid-body singular and buckling is ill-defined. "
            "Provide at least one fixture.")

    free = np.setdiff1d(
        np.arange(n_dof), np.fromiter(fixed_dofs, dtype=np.int64))
    n_free = int(free.size)
    if n_free == 0:
        raise ValueError(
            "reference_buckling: all DOFs are fixed; nothing to solve.")
    if n_free <= n_modes:
        raise ValueError(
            f"reference_buckling: only {n_free} free DOFs but {n_modes} modes "
            f"requested; mesh is too small / too constrained.")

    # --- static pre-stress solve K u = F -----------------------------------
    Kff = K[free][:, free].tocsc()
    Ff = F[free]
    try:
        uf = spla.spsolve(Kff, Ff)
    except Exception as exc:  # noqa: BLE001 — surface any solver failure
        raise RuntimeError(
            f"reference_buckling: pre-stress linear solve failed: {exc}"
        ) from exc
    if not np.all(np.isfinite(uf)):
        raise RuntimeError(
            "reference_buckling: pre-stress solution contains non-finite "
            "values — the system is likely under-constrained / singular "
            "(insufficient fixtures to remove rigid-body modes).")
    u = np.zeros(n_dof)
    u[free] = uf

    # --- per-element stresses at the 8 Gauss points ------------------------
    # We need the full Voigt stress at every Gauss point (not just von Mises)
    # to build S_hat. Recover sigma = D @ B(gp) @ u_elem for each element.
    D = _elastic_D(E, nu)
    u_elem = u[elem_dofs]                       # (n_active, 24)
    g = 1.0 / np.sqrt(3.0)
    gauss = [(xi, eta, zeta)
             for xi in (-g, g) for eta in (-g, g) for zeta in (-g, g)]
    detJ = (h / 2.0) ** 3                       # weights are all 1

    # Pre-compute G and B at each Gauss point (geometry is the same cube for
    # every element, so these 8 (G, B) pairs are reused across all elements).
    inv = 2.0 / h
    G_gp = []
    B_gp = []
    for (xi, eta, zeta) in gauss:
        dN_xyz = _shape_grad_natural(xi, eta, zeta) * inv
        G_gp.append(_G_matrix(dN_xyz))
        B_gp.append(_B_matrix(dN_xyz))

    # Track whether any compressive (negative) normal stress exists; a purely
    # tensile state can never buckle and must raise rather than return junk.
    min_principal = np.inf  # most-negative principal stress seen anywhere

    # --- assemble geometric stiffness K_g ----------------------------------
    # Kg_e (24x24) per element, summed over the 8 Gauss points:
    #     Kg_e += (G^T @ S_hat @ G) * detJ
    Kg_elem = np.zeros((n_active, 24, 24))
    for gp in range(8):
        G = G_gp[gp]
        B = B_gp[gp]
        # Voigt stress at this Gauss point for every element: (n_active, 6)
        sig = (u_elem @ B.T) @ D.T
        # Vectorised most-negative principal stress across all elements at
        # this gp (eigvalsh of the per-element 3x3 Cauchy tensor).
        S3 = np.zeros((n_active, 3, 3))
        S3[:, 0, 0] = sig[:, 0]
        S3[:, 1, 1] = sig[:, 1]
        S3[:, 2, 2] = sig[:, 2]
        S3[:, 1, 2] = S3[:, 2, 1] = sig[:, 3]  # yz
        S3[:, 0, 2] = S3[:, 2, 0] = sig[:, 4]  # xz
        S3[:, 0, 1] = S3[:, 1, 0] = sig[:, 5]  # xy
        eigs = np.linalg.eigvalsh(S3)           # (n_active, 3) ascending
        min_principal = min(min_principal, float(eigs[:, 0].min()))

        # Build S_hat for every element and accumulate G^T S_hat G * detJ.
        # G^T S_hat G with S_hat = diag(S,S,S) decomposes into a sum over the
        # three displacement components, each using the SAME 3x3 S block on
        # its gradient triple. Implement as a batched einsum.
        # G has shape (9,24); split into three (3,24) gradient blocks.
        Gx = G[0:3, :]   # gradients of u_x
        Gy = G[3:6, :]   # gradients of u_y
        Gz = G[6:9, :]   # gradients of u_z
        # For each element e:  Gx^T S_e Gx + Gy^T S_e Gy + Gz^T S_e Gz
        # = (Gx^T + Gy^T + Gz^T paths) ; but S_e differs per element, so batch.
        # contrib_e = sum_{blk in {x,y,z}} Gblk^T @ S_e @ Gblk
        # Use einsum over the element axis with the shared Gblk geometry.
        for Gblk in (Gx, Gy, Gz):
            # (n,24,24) = Gblk^T (24,3) @ S3 (n,3,3) @ Gblk (3,24)
            tmp = np.einsum('ai,nab->nib', Gblk, S3)      # (n,24,3)
            Kg_elem += np.einsum('nib,bj->nij', tmp, Gblk) * detJ

    if not np.isfinite(min_principal) or min_principal >= 0.0:
        raise ValueError(
            "reference_buckling: no compressive stress anywhere in the "
            "pre-stress state (most-negative principal stress = "
            f"{min_principal:.3e} Pa >= 0). A purely tensile / shear-balanced "
            "state cannot buckle under this load; criterion is unverifiable "
            "as a buckling case.")

    # Assemble global K_g (sparse, same triplet pattern as K).
    kgvals = Kg_elem.reshape(-1)
    Kg = sp.coo_matrix((kgvals, (rows, cols)), shape=(n_dof, n_dof)).tocsr()
    Kgff = Kg[free][:, free].tocsc()
    Kgff = 0.5 * (Kgff + Kgff.T)  # symmetrise round-off asymmetry
    Kff_sym = 0.5 * (Kff + Kff.T)

    # --- generalised eigenproblem  K phi = -lambda K_g phi -----------------
    # We want the smallest POSITIVE buckling factor lambda. For a compressive
    # pre-stress, the relevant mode has the elastic strain energy (positive)
    # balanced by the destabilising geometric term, so positive lambda is the
    # compressive buckling mode; a tensile pre-stress gives no positive lambda
    # (and we raise). The solve is delegated to _solve_buckling_eigs which
    # reduces to a standard symmetric problem and returns positive factors.
    k_solve = min(n_modes + 6, n_free - 2)
    lambdas = _solve_buckling_eigs(Kff_sym, Kgff, k_solve, notes)

    # --- filter to genuine positive buckling factors -----------------------
    finite = lambdas[np.isfinite(lambdas)]
    scale = float(np.max(np.abs(finite))) if finite.size else 1.0
    tol = max(scale * 1e-8, 1e-12)
    positive = np.sort(finite[finite > tol])
    n_discarded = int(lambdas.size - positive.size)
    if n_discarded:
        notes.append(
            f"discarded {n_discarded} non-positive / spurious eigenvalue(s) "
            f"(<= {tol:.3e}); these are tension-mode or numerical-noise roots, "
            f"not compressive buckling factors.")
    if positive.size == 0:
        raise ValueError(
            "reference_buckling: no positive eigenvalue found — the load does "
            "not produce a compressive buckling mode in this constrained "
            "configuration (or the mesh is too coarse to resolve one).")

    factors = positive[:n_modes]
    bf = float(factors[0])

    # --- reportable absolute loads -----------------------------------------
    applied_ref_load = None
    critical_load = None
    net_mag = float(np.linalg.norm(applied_force_vec))
    if net_mag > 0.0:
        applied_ref_load = net_mag
        critical_load = bf * net_mag
    elif applied_force_abs > 0.0:
        # Self-equilibrated or multi-axis load: net force ~0 but a real load
        # magnitude exists. Report the summed magnitude as the reference.
        applied_ref_load = applied_force_abs
        critical_load = bf * applied_force_abs
        notes.append(
            "applied loads have ~zero net resultant (self-equilibrated); "
            "applied_reference_load_n reports the summed applied magnitude.")

    notes.append(
        f"hex8 eigenvalue buckling on {n_free} free DOFs; geometric stiffness "
        f"from 2x2x2 Gauss recovered stresses; smallest positive factor = "
        f"{bf:.4e} (most-compressive principal pre-stress {min_principal:.3e} "
        f"Pa). Coarse hex8 meshes typically over-predict the critical factor "
        f"by 10-30%; treat the factor as approximate.")

    return {
        "buckling_load_factor": bf,
        "buckling_load_factors": [float(x) for x in factors],
        "applied_reference_load_n": applied_ref_load,
        "critical_load_n": critical_load,
        "n_modes": int(factors.size),
        "method": METHOD,
        "n_active_elements": n_active,
        "n_dof": int(n_dof),
        "n_free_dof": n_free,
        "notes": notes,
    }


# ---------------------------------------------------------------------------
# Generalised eigensolve for the buckling problem
# ---------------------------------------------------------------------------

def _solve_buckling_eigs(K: sp.spmatrix, Kg: sp.spmatrix, k: int,
                         notes: list[str]) -> np.ndarray:
    """Positive buckling factors of ``K phi = -lambda K_g phi`` (ascending).

    ``K`` is the elastic stiffness (symmetric positive-definite on the free
    DOFs) and ``K_g`` is the geometric (initial-stress) stiffness (symmetric,
    indefinite in general). We want the *smallest positive* ``lambda``.

    Reduction to a standard symmetric eigenproblem
    ----------------------------------------------
    ``K`` is SPD, so factor ``K = L L^T`` (Cholesky). Substituting
    ``phi = L^{-T} y`` into ``K phi = -lambda K_g phi`` and left-multiplying by
    ``L^{-1}`` gives

        y = -lambda ( L^{-1} K_g L^{-T} ) y     <=>     A y = (1/lambda) y

    with ``A = -L^{-1} K_g L^{-T}`` symmetric. The eigenvalues of ``A`` are
    ``eta = 1/lambda``; the **largest positive** ``eta`` correspond to the
    **smallest positive** ``lambda`` — the lowest compressive buckling factors.
    A tensile / non-buckling pre-stress makes ``A`` have no positive
    eigenvalues, so no positive ``lambda`` is returned and the caller raises.

    For large systems the dense Cholesky/eig is replaced by a sparse
    shift-invert about a large positive ``eta`` (``which='LA'``) using a sparse
    Cholesky-free reduction; we fall back to the dense path on any failure.
    """
    n = K.shape[0]
    k = max(1, min(k, n - 2))

    # Small/medium systems: dense symmetric reduction is robust and exact.
    if n <= 1500:
        return _dense_reduced(K, Kg, k, notes)

    # Large systems: sparse path. A = -L^{-1} K_g L^{-T} via sparse Cholesky is
    # not available in scipy, so solve the generalised standard-form problem
    # (-K_g) y = eta K y for the LARGEST algebraic eta with K as the SPD M
    # (shift-invert handled internally by eigsh's generalised mode), then
    # lambda = 1/eta. Largest positive eta -> smallest positive lambda.
    try:
        eta = spla.eigsh((-Kg).tocsc(), k=k, M=K.tocsc(),
                         which="LA", return_eigenvectors=False)
        eta = eta[np.isfinite(eta)]
        pos = eta[eta > 0]
        if pos.size:
            lam = 1.0 / pos
            return np.sort(lam[np.isfinite(lam)])
        notes.append("sparse buckling eigensolve returned no positive eta; "
                     "falling back to dense reduction.")
    except Exception as exc:  # noqa: BLE001 — fall back to dense
        notes.append(
            f"sparse buckling eigensolve failed ({type(exc).__name__}); "
            f"falling back to dense reduction.")
    return _dense_reduced(K, Kg, k, notes)


def _dense_reduced(K: sp.spmatrix, Kg: sp.spmatrix, k: int,
                   notes: list[str]) -> np.ndarray:
    """Dense Cholesky-reduced solve of ``K phi = -lambda K_g phi``.

    Returns the smallest ``k`` positive buckling factors ``lambda`` in
    ascending order (possibly fewer if the geometric stiffness admits fewer
    positive modes). See :func:`_solve_buckling_eigs` for the derivation.
    """
    import scipy.linalg as sla
    Kd = np.asarray(K.todense())
    Kgd = np.asarray(Kg.todense())
    Kd = 0.5 * (Kd + Kd.T)
    Kgd = 0.5 * (Kgd + Kgd.T)
    try:
        L = np.linalg.cholesky(Kd)
    except np.linalg.LinAlgError:
        # K should be SPD on the free DOFs; if a tiny indefiniteness sneaks in
        # (round-off on a nearly-singular constraint), regularise minimally.
        eps = 1e-12 * float(np.trace(Kd) / Kd.shape[0])
        notes.append(
            "elastic stiffness not numerically SPD; added a tiny diagonal "
            f"jitter ({eps:.2e}) to factorise.")
        L = np.linalg.cholesky(Kd + eps * np.eye(Kd.shape[0]))
    Linv = sla.solve_triangular(L, np.eye(L.shape[0]), lower=True)
    A = -Linv @ Kgd @ Linv.T
    A = 0.5 * (A + A.T)
    eta = np.linalg.eigvalsh(A)                      # eta = 1/lambda, ascending
    # Largest positive eta -> smallest positive lambda. Drop ~0 eta (lambda=inf:
    # modes with no geometric softening) and negative eta (tensile/anti-buckling
    # modes, lambda < 0).
    scale = float(np.max(np.abs(eta))) if eta.size else 1.0
    tol = max(scale * 1e-9, 1e-300)
    pos = eta[eta > tol]
    if pos.size == 0:
        return np.empty(0)
    lam = np.sort(1.0 / pos)                          # ascending lambda
    return lam[:k]
