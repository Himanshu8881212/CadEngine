"""fea_tet.py — unstructured quadratic-tetrahedron (tet10) linear-elastic FEA.

The body-fitted counterpart to the structured hex8 :func:`physics.fea.
reference_fea`. Where the hex8 solver assembles ONE identical element stiffness
on a voxel grid (and therefore staircases every curved boundary), this solver
integrates a *per-element* isoparametric stiffness over a conforming tet10 mesh
(:class:`physics.mesh_ir.MeshIR`) whose faces follow the true surface — so
peak stress at a fillet can actually converge.

It is deliberately consistent with the hex8 solver so results are comparable:
same isotropic constitutive matrix (:func:`physics.fea._elastic_D`), same
Voigt order ``[xx, yy, zz, yz, xz, xy]``, same von Mises (:func:`_von_mises`),
same mm-in / SI-internal convention (node coords mm, E in Pa, force in N,
stress in Pa).

Element: 10-node quadratic tetrahedron, 30 DOF, 4-point degree-3 Gauss rule,
per-element numerical Jacobian (curved elements have a non-constant J). The
shape functions are written in the exact gmsh type-11 node ordering documented
in :mod:`physics.mesh_ir`.

Dependencies: numpy + scipy. No gmsh at solve time.
"""

from __future__ import annotations

import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla

from .fea import _elastic_D, _von_mises
from .mesh_ir import MeshIR

METHOD_TET = "reference_tet10_linear_elastic_body_fitted"

# 4-point degree-3 Gauss rule on the reference tet (corners (0,0,0),(1,0,0),
# (0,1,0),(0,0,1), volume 1/6). Integrates cubics exactly; weights sum to 1/6.
_GA = 0.5854101966249685
_GB = 0.1381966011250105
_TET_GP = np.array([[_GB, _GB, _GB], [_GA, _GB, _GB],
                    [_GB, _GA, _GB], [_GB, _GB, _GA]])
_TET_W = np.full(4, (1.0 / 6.0) / 4.0)

# Natural coords of the 10 nodes (gmsh type-11 ordering) — for nodal stress
# recovery (evaluate stress AT the nodes, then average across elements).
_TET10_NODE_NAT = np.array([
    [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0],
    [0.5, 0.0, 0.0], [0.5, 0.5, 0.0], [0.0, 0.5, 0.0],
    [0.0, 0.0, 0.5], [0.0, 0.5, 0.5], [0.5, 0.0, 0.5],
])


def _shape_grad(r: float, s: float, t: float):
    """tet10 shape functions N (10,) and natural gradients dN/d(r,s,t) (10,3),
    gmsh type-11 ordering. Barycentric L1=1-r-s-t, L2=r, L3=s, L4=t."""
    L1, L2, L3, L4 = 1.0 - r - s - t, r, s, t
    # dL/d(r,s,t)
    dL1 = np.array([-1.0, -1.0, -1.0])
    dL2 = np.array([1.0, 0.0, 0.0])
    dL3 = np.array([0.0, 1.0, 0.0])
    dL4 = np.array([0.0, 0.0, 1.0])
    N = np.array([
        L1 * (2 * L1 - 1), L2 * (2 * L2 - 1), L3 * (2 * L3 - 1), L4 * (2 * L4 - 1),
        4 * L1 * L2, 4 * L2 * L3, 4 * L1 * L3, 4 * L1 * L4, 4 * L3 * L4, 4 * L2 * L4,
    ])
    dN = np.empty((10, 3))
    dN[0] = (4 * L1 - 1) * dL1
    dN[1] = (4 * L2 - 1) * dL2
    dN[2] = (4 * L3 - 1) * dL3
    dN[3] = (4 * L4 - 1) * dL4
    dN[4] = 4 * (dL1 * L2 + L1 * dL2)
    dN[5] = 4 * (dL2 * L3 + L2 * dL3)
    dN[6] = 4 * (dL1 * L3 + L1 * dL3)
    dN[7] = 4 * (dL1 * L4 + L1 * dL4)
    dN[8] = 4 * (dL3 * L4 + L3 * dL4)
    dN[9] = 4 * (dL2 * L4 + L2 * dL4)
    return N, dN


def _B_from_dNxyz(dN_xyz: np.ndarray) -> np.ndarray:
    """Strain-displacement B (6 x 30) from dN/d(x,y,z) (10,3). Same Voigt order
    and same DOF layout [u0x,u0y,u0z, u1x,...] as the hex8 solver."""
    B = np.zeros((6, 30))
    for n in range(10):
        dx, dy, dz = dN_xyz[n]
        c = 3 * n
        B[0, c + 0] = dx
        B[1, c + 1] = dy
        B[2, c + 2] = dz
        B[3, c + 1] = dz
        B[3, c + 2] = dy
        B[4, c + 0] = dz
        B[4, c + 2] = dx
        B[5, c + 0] = dy
        B[5, c + 1] = dx
    return B


def _element_matrices(coords_m: np.ndarray, D: np.ndarray):
    """Return (Ke 30x30, [(gp_B, gp_detJ_w)...]) for one tet10, coords in metres.
    Also returns the per-Gauss-point B and weight for later stress recovery."""
    Ke = np.zeros((30, 30))
    gp_data = []
    for (r, s, t), w in zip(_TET_GP, _TET_W):
        _, dN = _shape_grad(r, s, t)
        # Jacobian J[k,i] = dx_k/dxi_i  (physical row, natural col). dN is
        # dN/d(xi); coords_m is (10,3) node positions. J = coords^T @ dN.
        # Then dN/dx = dN/dxi . dxi/dx = dN @ inv(J). Writing J the transposed
        # way (dN^T @ coords) and using inv(J) silently gives inv(J)^T for any
        # non-axis-aligned element — correct only when J is symmetric — which
        # destroys the strain on rotated elements (all real tets). Keep this.
        J = coords_m.T @ dN           # (3,3)
        detJ = np.linalg.det(J)
        if detJ <= 0:
            return None, None         # inverted element -> caller rejects
        dN_xyz = dN @ np.linalg.inv(J)   # (10,3)
        B = _B_from_dNxyz(dN_xyz)
        Ke += (B.T @ D @ B) * (detJ * w)
        gp_data.append((B, detJ * w))
    return Ke, gp_data


# ---------------------------------------------------------------------------
# tri6 surface faces (pressure / traction loads)
# ---------------------------------------------------------------------------

# Dunavant degree-4, 6-point rule on the reference triangle (corners (0,0),
# (1,0),(0,1), area 1/2). Integrates quartics exactly — the tri6 pressure
# integrand N_i*(dx/dxi x dx/deta) is quartic on a curved (quadratic) face, so
# this makes the CONSISTENT nodal load vector exact, not lumped. Weights below
# already fold in the 1/2 reference area, so sum(TRI_W) == 1/2.
_TA, _TB = 0.445948490915965, 0.108103018168070
_TC, _TD = 0.091576213509771, 0.816847572980459
_TRI_GP = np.array([
    [_TA, _TA], [_TB, _TA], [_TA, _TB],
    [_TC, _TC], [_TD, _TC], [_TC, _TD],
])
_TRI_W = 0.5 * np.array([0.223381589678011] * 3 + [0.109951743655322] * 3)


def _tri6_shape(xi: float, eta: float):
    """tri6 (gmsh type-9) shape functions N (6,) and natural gradients (6,2).
    Node order: 0,1,2 corners; 3=mid(0,1) 4=mid(1,2) 5=mid(2,0). Area coords
    L1=1-xi-eta, L2=xi, L3=eta."""
    L1, L2, L3 = 1.0 - xi - eta, xi, eta
    dL1 = np.array([-1.0, -1.0])
    dL2 = np.array([1.0, 0.0])
    dL3 = np.array([0.0, 1.0])
    N = np.array([L1 * (2 * L1 - 1), L2 * (2 * L2 - 1), L3 * (2 * L3 - 1),
                  4 * L1 * L2, 4 * L2 * L3, 4 * L3 * L1])
    dN = np.empty((6, 2))
    dN[0] = (4 * L1 - 1) * dL1
    dN[1] = (4 * L2 - 1) * dL2
    dN[2] = (4 * L3 - 1) * dL3
    dN[3] = 4 * (dL1 * L2 + L1 * dL2)
    dN[4] = 4 * (dL2 * L3 + L2 * dL3)
    dN[5] = 4 * (dL3 * L1 + L3 * dL1)
    return N, dN


def _face_opposite_corner(tets: np.ndarray) -> dict:
    """Map frozenset of a tet's 3 corner-node ids -> the 4th (interior) corner
    id, over every tet face. A boundary face appears in exactly one tet, so the
    lookup gives an interior reference point to orient the outward face normal
    for pressure sign. Built only when a pressure load is present."""
    opp = {}
    tc = tets[:, :4]
    faces = [(0, 1, 2, 3), (0, 1, 3, 2), (0, 2, 3, 1), (1, 2, 3, 0)]
    for row in tc:
        for i, j, k, o in faces:
            opp[frozenset((int(row[i]), int(row[j]), int(row[k])))] = int(row[o])
    return opp


# ---------------------------------------------------------------------------
# Geometric selectors on arbitrary points (the mesh selector bridge). Mirrors
# the geometric MEANING of physics.selectors.resolve_selector (plane /
# box(=bbox) / cylinder), but tests true mesh NODE coordinates directly instead
# of voxel-element centres — so none of the voxel-grid half-/one-voxel
# tolerance bands apply here (a node either is or isn't in the region).
# ---------------------------------------------------------------------------

_AXIS = {"x": 0, "y": 1, "z": 2}


def nodes_in_selector(nodes_mm: np.ndarray, sel: dict) -> np.ndarray:
    """Boolean mask of nodes satisfying a geometric selector (mm coords).

    Supported ``type`` (alias ``kind``): ``all``, ``plane`` (axis, value_mm,
    side in '+'/'-'/'both', tol_mm), ``box``/``bbox`` (min_mm, max_mm),
    ``cylinder`` (axis, center_mm, radius_mm, optional length_mm — a finite
    axial extent centred on ``center_mm`` along ``axis``; omitted => infinite).
    """
    typ = sel.get("type", sel.get("kind"))
    if typ in ("all", None):
        return np.ones(nodes_mm.shape[0], dtype=bool)
    if typ == "plane":
        axis = _AXIS[sel["axis"]]
        v = float(sel["value_mm"])
        side = sel.get("side", "both")
        tol = float(sel.get("tol_mm", 1e-4))
        coord = nodes_mm[:, axis]
        if side == "-":
            return coord <= v + tol
        if side == "+":
            return coord >= v - tol
        return np.abs(coord - v) <= tol
    if typ in ("box", "bbox"):
        lo = np.asarray(sel["min_mm"], float)
        hi = np.asarray(sel["max_mm"], float)
        # accept min/max in either order (matches selectors._resolve_bbox)
        lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)
        return np.all((nodes_mm >= lo - 1e-6) & (nodes_mm <= hi + 1e-6), axis=1)
    if typ == "cylinder":
        axis = _AXIS[sel["axis"]]
        center = np.asarray(sel["center_mm"], float)
        radius = float(sel["radius_mm"])
        radial = [a for a in range(3) if a != axis]
        r2 = ((nodes_mm[:, radial] - center[radial]) ** 2).sum(axis=1)
        mask = r2 <= radius * radius + 1e-6
        length = sel.get("length_mm")
        if length is not None:
            half = 0.5 * float(length)
            mask &= np.abs(nodes_mm[:, axis] - center[axis]) <= half + 1e-6
        return mask
    raise ValueError(f"tet selector: unsupported type {typ!r}")


# ---------------------------------------------------------------------------
# Main solve
# ---------------------------------------------------------------------------

def reference_fea_tet(mesh: MeshIR, material: dict,
                      loads: list[dict], fixtures: list[dict],
                      *, direct_max_dof: int = 250_000,
                      cg_tol: float = 1e-9, cg_maxiter: int = 20000) -> dict:
    """Body-fitted tet10 linear-elastic solve on a MeshIR.

    Parameters mirror :func:`physics.fea.reference_fea` where it makes
    sense. ``material``: {youngs_modulus_pa, poisson, density_kg_m3}.
    ``fixtures``: [{kind: clamped|pinned, region_selector, dof_constrained?}].
    ``loads``: [{kind: point, magnitude, direction, region_selector}] — a
    ``point`` load's resultant is spread equally over the selected surface
    nodes (same convention as the hex8 path).

    Returns {ok, max_von_mises_pa, max_disp_m, disp (N,3) m, vm_nodal (N,) Pa,
    n_nodes, n_tets, method, notes}.
    """
    E = float(material["youngs_modulus_pa"])
    nu = float(material["poisson"])
    D = _elastic_D(E, nu)
    nodes_mm = mesh.nodes_mm
    coords_m = nodes_mm * 1e-3
    tets = mesh.tets
    nn = nodes_mm.shape[0]
    ndof = 3 * nn
    notes: list[str] = []

    # -- assemble K (COO -> CSR) ------------------------------------------
    ne = tets.shape[0]
    rows = np.empty(ne * 900, dtype=np.int64)
    cols = np.empty(ne * 900, dtype=np.int64)
    data = np.empty(ne * 900, dtype=np.float64)
    gp_store: list = [None] * ne
    p = 0
    inverted = 0
    for e in range(ne):
        conn = tets[e]
        Ke, gp = _element_matrices(coords_m[conn], D)
        if Ke is None:
            inverted += 1
            continue
        gp_store[e] = (conn, gp)
        edof = (3 * conn[:, None] + np.arange(3)).ravel()  # (30,)
        rr = np.repeat(edof, 30)
        cc = np.tile(edof, 30)
        rows[p:p + 900] = rr
        cols[p:p + 900] = cc
        data[p:p + 900] = Ke.ravel()
        p += 900
    if inverted:
        return {"ok": False, "error": f"{inverted} inverted tet10 elements "
                f"(non-positive Jacobian at a Gauss point)"}
    K = sp.coo_matrix((data[:p], (rows[:p], cols[:p])),
                      shape=(ndof, ndof)).tocsr()

    # -- fixtures (Dirichlet) ---------------------------------------------
    fixed = np.zeros(ndof, dtype=bool)
    for fx in fixtures:
        mask = nodes_in_selector(nodes_mm, fx["region_selector"])
        nidx = np.nonzero(mask)[0]
        if nidx.size == 0:
            return {"ok": False, "error": f"fixture {fx.get('kind')} selected 0 nodes"}
        dofs = fx.get("dof_constrained")  # e.g. [0,1,2] or None => all 3
        comps = range(3) if not dofs else dofs
        for c in comps:
            fixed[3 * nidx + c] = True
    n_fixed = int(fixed.sum())
    if n_fixed == 0:
        return {"ok": False, "error": "no DOFs constrained — singular system"}

    # -- loads (RHS) -------------------------------------------------------
    F = np.zeros(ndof)
    load_receipts = []
    density = float(material.get("density_kg_m3", 0.0))
    surf_tris = mesh.surf_tris
    # outward-normal reference, built once, only if a pressure load needs it
    face_opp = (_face_opposite_corner(tets)
                if any(l.get("kind") == "pressure" for l in loads) else None)
    for ld in loads:
        kind = ld.get("kind", "point")

        if kind == "point":
            mask = nodes_in_selector(nodes_mm, ld["region_selector"])
            nidx = np.nonzero(mask)[0]
            if nidx.size == 0:
                return {"ok": False, "error": f"load {kind} selected 0 nodes"}
            direction = np.asarray(ld["direction"], float)
            direction = direction / (np.linalg.norm(direction) or 1.0)
            total = float(ld["magnitude"]) * direction  # N resultant
            per = total / nidx.size
            for c in range(3):
                np.add.at(F, 3 * nidx + c, per[c])
            load_receipts.append({"kind": "point", "nodes": int(nidx.size)})

        elif kind == "body":
            # Consistent volume load: per-DOF force = mag(N/kg) * dir * rho *
            # integral(N_i dV) over selected elements. Same physical meaning as
            # the hex8 body load (magnitude is an acceleration in N/kg), but the
            # tet10 integral is CONSISTENT (Gauss-integrated), not lumped — so
            # some corner entries are legitimately negative while the resultant
            # is exactly mag*rho*V*dir. Elements selected by their centroid.
            direction = np.asarray(ld["direction"], float)
            direction = direction / (np.linalg.norm(direction) or 1.0)
            mag = float(ld["magnitude"])
            centroids = coords_m[tets[:, :4]].mean(axis=1) * 1e3  # mm
            emask = nodes_in_selector(centroids, ld["region_selector"])
            esel = np.nonzero(emask)[0]
            if esel.size == 0:
                return {"ok": False, "error": "load body selected 0 elements"}
            for e in esel:
                ce = coords_m[tets[e]]
                fvol = np.zeros(10)
                for (r, s, t), w in zip(_TET_GP, _TET_W):
                    N, dN = _shape_grad(r, s, t)
                    detJ = np.linalg.det(ce.T @ dN)
                    fvol += N * (detJ * w)
                fnodal = mag * density * np.outer(fvol, direction)  # (10,3) N
                gd = (3 * tets[e][:, None] + np.arange(3)).ravel()
                np.add.at(F, gd, fnodal.ravel())
            load_receipts.append({"kind": "body", "elements": int(esel.size),
                                  "density_kg_m3": density, "consistent": True})

        elif kind in ("pressure", "traction"):
            if surf_tris.shape[0] == 0:
                return {"ok": False, "error":
                        f"load {kind}: mesh has no surf_tris boundary faces"}
            nmask = nodes_in_selector(nodes_mm, ld["region_selector"])
            fsel = np.nonzero(nmask[surf_tris].all(axis=1))[0]
            if fsel.size == 0:
                return {"ok": False,
                        "error": f"load {kind} selected 0 boundary faces"}
            mag = float(ld["magnitude"])
            if kind == "traction":
                direction = np.asarray(ld["direction"], float)
                direction = direction / (np.linalg.norm(direction) or 1.0)
            total_area = 0.0
            for f in fsel:
                face = surf_tris[f]
                fc = coords_m[face]  # (6,3) m
                if kind == "pressure":
                    # Outward normal from the interior corner; positive
                    # magnitude pushes INWARD (compressive) — matches hex8.
                    opp = face_opp.get(frozenset(int(n) for n in face[:3]))
                    Nc, dNc = _tri6_shape(1 / 3, 1 / 3)
                    cross_c = np.cross(dNc[:, 0] @ fc, dNc[:, 1] @ fc)
                    if opp is not None:
                        outward = Nc @ fc - coords_m[opp]
                        s = 1.0 if np.dot(cross_c, outward) >= 0 else -1.0
                    else:
                        s = 1.0  # unshared face: use gmsh face winding as-is
                fvec = np.zeros((6, 3))
                for (xi, eta), w in zip(_TRI_GP, _TRI_W):
                    N, dN = _tri6_shape(xi, eta)
                    cross = np.cross(dN[:, 0] @ fc, dN[:, 1] @ fc)  # n*dA (m^2)
                    if kind == "pressure":
                        fvec += w * np.outer(N, (-mag * s) * cross)
                    else:
                        total_area += w * np.linalg.norm(cross)
                        fvec += w * np.outer(N, mag * np.linalg.norm(cross)
                                             * direction)
                gd = (3 * face[:, None] + np.arange(3)).ravel()
                np.add.at(F, gd, fvec.ravel())
            rec = {"kind": kind, "faces": int(fsel.size), "consistent": True}
            if kind == "traction":
                rec["area_m2"] = float(total_area)
            load_receipts.append(rec)

        else:
            return {"ok": False,
                    "error": f"tet path: load kind {kind!r} not supported"}

    # -- reduce & solve ----------------------------------------------------
    free = ~fixed
    Kff = K[free][:, free]
    Ff = F[free]
    u = np.zeros(ndof)
    if free.sum() <= direct_max_dof:
        uf = spla.spsolve(Kff.tocsc(), Ff)
        solver = "superlu_direct"
    else:
        diag = Kff.diagonal()
        diag[diag == 0] = 1.0
        M = spla.LinearOperator(Kff.shape, matvec=lambda x: x / diag)
        uf, info = spla.cg(Kff, Ff, rtol=cg_tol, maxiter=cg_maxiter, M=M)
        solver = f"jacobi_cg(info={info})"
        if info != 0:
            notes.append(f"CG did not converge cleanly (info={info})")
    u[free] = uf
    disp = u.reshape(nn, 3)

    # -- stress recovery: evaluate stress at each element's 10 NODES (not the
    #    element average — that smears a surface peak into the interior and
    #    under-reads a fillet) then average across elements meeting each node.
    #    This is nodal stress recovery; the surface peak it yields converges to
    #    the true concentration under mesh refinement. ----------------------
    #    Vectorised over elements: for each of the 10 node natural positions we
    #    batch inv(J) with np.linalg.inv on a stacked (E,3,3) array and build
    #    the (E,6,30) B in bulk — numerically identical to the old per-element
    #    double loop (verified <1e-6*max on a small mesh) but ~10x faster.
    node_grads = np.array([_shape_grad(r, s, t)[1]
                           for (r, s, t) in _TET10_NODE_NAT])  # (10,10,3)
    ce_all = coords_m[tets]                                    # (E,10,3)
    edof_all = (3 * tets[:, :, None] + np.arange(3)).reshape(ne, 30)
    ue_all = u[edof_all]                                       # (E,30)
    col = 3 * np.arange(10)
    vm_count = np.zeros(nn)
    sig_accum = np.zeros((nn, 6))
    for a in range(10):
        dN = node_grads[a]                                    # (10,3)
        J = np.einsum("enk,ni->eki", ce_all, dN)              # (E,3,3)
        dN_xyz = np.einsum("ni,eij->enj", dN, np.linalg.inv(J))  # (E,10,3)
        B = np.zeros((ne, 6, 30))
        dx, dy, dz = dN_xyz[..., 0], dN_xyz[..., 1], dN_xyz[..., 2]
        B[:, 0, col + 0] = dx
        B[:, 1, col + 1] = dy
        B[:, 2, col + 2] = dz
        B[:, 3, col + 1] = dz
        B[:, 3, col + 2] = dy
        B[:, 4, col + 0] = dz
        B[:, 4, col + 2] = dx
        B[:, 5, col + 0] = dy
        B[:, 5, col + 1] = dx
        sig = np.einsum("ij,ej->ei", D, np.einsum("eij,ej->ei", B, ue_all))
        g = tets[:, a]
        np.add.at(sig_accum, g, sig)
        np.add.at(vm_count, g, 1.0)
    good = vm_count > 0
    sig_accum[good] /= vm_count[good, None]
    vm_nodal = np.array([_von_mises(sig_accum[i]) for i in range(nn)])

    return {
        "ok": True,
        "method": METHOD_TET,
        "solver": solver,
        "n_nodes": nn,
        "n_tets": int(ne),
        "n_fixed_dof": n_fixed,
        "max_disp_m": float(np.abs(disp).max()),
        "max_von_mises_pa": float(vm_nodal.max()),
        "disp": disp,
        "vm_nodal": vm_nodal,
        "sig_nodal": sig_accum,
        "loads": load_receipts,
        "notes": notes,
    }
