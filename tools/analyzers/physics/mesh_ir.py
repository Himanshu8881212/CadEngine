"""mesh_ir.py — body-fitted tetrahedral mesh intermediate representation.

The structured hex8 path (:mod:`physics.fea`) binarises geometry onto a
voxel grid, which STAIRCASES every curved boundary — so peak stress at a fillet
never converges (see ``tools/ace_fea_kt_validation.py``). This module is the
first half of the body-fitted alternative: a **conforming quadratic-tetrahedron
(tet10) mesh** whose element faces follow the true curved surface.

Design contract (deliberately mesher-agnostic)
----------------------------------------------
:class:`MeshIR` is the ONLY thing the solver (:mod:`physics.fea_tet`) sees.
The concrete mesher (Gmsh, here) sits behind :func:`mesh_stl` / the specimen
builders, so a different backend (TetGen, a native mesher) can be swapped in
without touching the solver, SPR recovery, or the validation pins.

Node ordering (gmsh element type 11, verified from
``gmsh.model.mesh.getElementProperties(11)`` — NOT guessed)::

    corners : 0=(0,0,0) 1=(1,0,0) 2=(0,1,0) 3=(0,0,1)
    edges   : 4=mid(0,1) 5=mid(1,2) 6=mid(0,2) 7=mid(0,3) 8=mid(2,3) 9=mid(1,3)

Units: node coordinates are in **mm** (matching the mm-in / SI-internal
convention of :func:`physics.fea.reference_fea`). The solver converts to
metres itself.

Dependencies: numpy + gmsh. gmsh is already present in the ACE python env
(4.15.2); it is used ONLY as a meshing backend, never at solve time.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# gmsh tet10 corner->edge-node topology, in this module's canonical ordering
# (matches gmsh type 11). Each edge node k (4..9) is the midpoint of two corners.
GMSH_TET10_EDGES = ((0, 1), (1, 2), (0, 2), (0, 3), (2, 3), (1, 3))


@dataclass
class MeshIR:
    """A conforming tet10 volume mesh in millimetre coordinates.

    Attributes
    ----------
    nodes_mm : (N, 3) float64
        Node coordinates in mm.
    tets : (E, 10) int32
        tet10 connectivity into ``nodes_mm``, gmsh type-11 node ordering.
    surf_tris : (F, 6) int32
        Boundary tri6 faces (quadratic triangles) into ``nodes_mm`` — the
        conforming surface, used for pressure/traction loads and for surface
        stress recovery. Empty array allowed.
    surf_group : (F,) int32
        Physical-group / source-face tag per boundary face (0 if untagged),
        so selectors and stress readouts can name a face.
    meta : dict
        Free-form provenance: backend, element size, gmsh version, receipts.
    """

    nodes_mm: np.ndarray
    tets: np.ndarray
    surf_tris: np.ndarray = field(default_factory=lambda: np.empty((0, 6), np.int32))
    surf_group: np.ndarray = field(default_factory=lambda: np.empty((0,), np.int32))
    meta: dict = field(default_factory=dict)

    # -- invariants / receipts ------------------------------------------------

    @property
    def n_nodes(self) -> int:
        return int(self.nodes_mm.shape[0])

    @property
    def n_tets(self) -> int:
        return int(self.tets.shape[0])

    def volume_mm3(self) -> float:
        """Exact mesh volume via the tet10 corner sub-tets (curved-face
        contribution is O(h^3) and folded in by the quadrature in the solver;
        this corner estimate is the reported geometric volume)."""
        p = self.nodes_mm[self.tets[:, :4]]  # (E,4,3) straight-edge corners
        v = np.einsum("ei,ei->e",
                      np.cross(p[:, 1] - p[:, 0], p[:, 2] - p[:, 0]),
                      p[:, 3] - p[:, 0]) / 6.0
        return float(np.abs(v).sum())

    def min_corner_jacobian(self) -> float:
        """Smallest signed corner-tet volume (a sliver/inversion sentinel).
        Negative => an inverted element the solver must reject."""
        p = self.nodes_mm[self.tets[:, :4]]
        v = np.einsum("ei,ei->e",
                      np.cross(p[:, 1] - p[:, 0], p[:, 2] - p[:, 0]),
                      p[:, 3] - p[:, 0]) / 6.0
        return float(v.min())

    def check(self, *, volume_ref_mm3: float | None = None,
              volume_tol: float = 5e-3) -> dict:
        """Falsifiable mesh receipt. Raises AssertionError on an inverted
        element; optionally checks volume against an analytic reference."""
        vol = self.volume_mm3()
        min_jac = self.min_corner_jacobian()
        assert min_jac > 0.0, (
            f"body-fitted mesh has a non-positive corner Jacobian "
            f"(min {min_jac:.3e} mm^3) — inverted/degenerate element; ref-mesh")
        rec = {"n_nodes": self.n_nodes, "n_tets": self.n_tets,
               "volume_mm3": vol, "min_corner_jacobian_mm3": min_jac,
               "n_surf_tris": int(self.surf_tris.shape[0])}
        if volume_ref_mm3 is not None:
            rel = abs(vol - volume_ref_mm3) / volume_ref_mm3
            rec["volume_ref_mm3"] = volume_ref_mm3
            rec["volume_rel_err"] = rel
            assert rel <= volume_tol, (
                f"body-fitted mesh volume {vol:.4f} mm^3 differs from analytic "
                f"{volume_ref_mm3:.4f} by {rel:.2%} > {volume_tol:.2%}")
        return rec


# ---------------------------------------------------------------------------
# Gmsh backend (the concrete mesher, behind the MeshIR contract)
# ---------------------------------------------------------------------------

def _gmsh_extract(model) -> MeshIR:
    """Pull a MeshIR out of the CURRENT gmsh model after a 3-D order-2 mesh."""
    import gmsh

    # nodes: gmsh tags are 1-based and possibly sparse -> compact to 0-based
    ntags, ncoords, _ = gmsh.model.mesh.getNodes()
    ntags = np.asarray(ntags, dtype=np.int64)
    coords = np.asarray(ncoords, dtype=np.float64).reshape(-1, 3)
    remap = np.full(int(ntags.max()) + 1, -1, dtype=np.int64)
    remap[ntags] = np.arange(ntags.size)
    nodes_mm = coords

    # tet10 volume elements (type 11)
    etags, enodes = gmsh.model.mesh.getElementsByType(11)
    tets = remap[np.asarray(enodes, dtype=np.int64).reshape(-1, 10)].astype(np.int32)

    # tri6 boundary faces (type 9), tagged by their surface physical/entity id
    surf_tris_list, surf_group_list = [], []
    for dim, tag in gmsh.model.getEntities(2):
        ftags, fnodes = gmsh.model.mesh.getElementsByType(9, tag)
        if len(ftags) == 0:
            continue
        f = remap[np.asarray(fnodes, dtype=np.int64).reshape(-1, 6)].astype(np.int32)
        surf_tris_list.append(f)
        surf_group_list.append(np.full(f.shape[0], tag, dtype=np.int32))
    surf_tris = (np.concatenate(surf_tris_list) if surf_tris_list
                 else np.empty((0, 6), np.int32))
    surf_group = (np.concatenate(surf_group_list) if surf_group_list
                  else np.empty((0,), np.int32))

    return MeshIR(nodes_mm=nodes_mm, tets=tets,
                  surf_tris=surf_tris, surf_group=surf_group,
                  meta={"backend": "gmsh", "gmsh_version": gmsh.option.getString(
                      "General.Version") if hasattr(gmsh.option, "getString") else "?"})


def mesh_stl(stl_path: str, *, elem_size_mm: float,
             min_size_mm: float | None = None, order: int = 2,
             high_order_optimize: bool = True,
             second_order_linear: bool = False) -> MeshIR:
    """Mesh a WATERTIGHT surface STL into a conforming tet10 volume.

    The kernel's own tessellation output (watertight, manifold) is the clean
    input this relies on; a non-watertight STL will fail loudly in gmsh's
    surface-loop -> volume step rather than silently produce garbage.

    ``high_order_optimize`` (default True) runs gmsh's curved-element untangler,
    which sharpens fillet-on-analytic-surface elements — but it ABORTS
    (SIGABRT, "failed to reach critical ScaledJac") on organic thin-member
    topologies such as a reconstructed topology-optimization iso-surface. Pass
    ``high_order_optimize=False`` to mesh those without the untangler.

    ``second_order_linear`` (default False) places tet10 mid-side nodes at the
    STRAIGHT edge midpoints instead of projecting them onto the curved
    reparametrised surface. On an organic reconstructed surface that projection
    (with the untangler off) leaves ~tens of non-positive-Jacobian slivers the
    solver rejects; genuinely straight-sided tet10 + mesh optimisation removes
    them. Use ``order=2, high_order_optimize=False, second_order_linear=True``
    for a robust body-fitted mesh of an arbitrary organic part; the analytic
    specimens keep the curved default.
    """
    import gmsh

    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.add("stl_tet")
        gmsh.merge(stl_path)
        # classify -> reparametrise the discrete surface -> bounded volume
        gmsh.model.mesh.classifySurfaces(np.deg2rad(40.0), True, True, np.deg2rad(180.0))
        gmsh.model.mesh.createGeometry()
        surfs = gmsh.model.getEntities(2)
        loop = gmsh.model.geo.addSurfaceLoop([s[1] for s in surfs])
        gmsh.model.geo.addVolume([loop])
        gmsh.model.geo.synchronize()
        gmsh.option.setNumber("Mesh.MeshSizeMax", elem_size_mm)
        if min_size_mm is not None:
            gmsh.option.setNumber("Mesh.MeshSizeMin", min_size_mm)
        gmsh.option.setNumber("Mesh.ElementOrder", order)
        gmsh.option.setNumber("Mesh.HighOrderOptimize",
                              1 if high_order_optimize else 0)
        if second_order_linear:
            gmsh.option.setNumber("Mesh.SecondOrderLinear", 1)
            gmsh.option.setNumber("Mesh.Optimize", 1)
            gmsh.option.setNumber("Mesh.OptimizeNetgen", 1)
        gmsh.model.mesh.generate(3)
        m = _gmsh_extract(gmsh.model)
        m.meta.update({"source": stl_path, "elem_size_mm": elem_size_mm,
                       "order": order, "high_order_optimize": high_order_optimize})
        return m
    finally:
        gmsh.finalize()


def mesh_shouldered_bar(d_mm: float, D_mm: float, r_mm: float,
                        l_small_mm: float, l_large_mm: float,
                        *, elem_size_mm: float, order: int = 2) -> MeshIR:
    """Exact stepped round bar with a TRUE (curved) shoulder fillet — the
    Kt-benchmark specimen, meshed body-fitted so the fillet is a real conic
    surface, not a voxel staircase. Small dia over ``l_small`` (z in
    [0, l_small]) then large dia over ``l_large`` (z in [l_small, l_small+
    l_large]); fillet radius ``r`` at the shoulder."""
    import gmsh

    rs, rl = d_mm / 2.0, D_mm / 2.0
    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.add("shouldered_bar")
        occ = gmsh.model.occ
        c1 = occ.addCylinder(0, 0, 0, 0, 0, l_small_mm, rs)
        c2 = occ.addCylinder(0, 0, l_small_mm, 0, 0, l_large_mm, rl)
        out, _ = occ.fuse([(3, c1)], [(3, c2)])
        occ.synchronize()
        vol = gmsh.model.getEntities(3)[0][1]
        # select ONLY the re-entrant shoulder circle (z==l_small, radius==rs)
        shoulder = []
        for _, tag in gmsh.model.getBoundary([(3, vol)], recursive=True):
            x0, y0, z0, x1, y1, z1 = gmsh.model.getBoundingBox(1, abs(tag))
            if abs(z0 - l_small_mm) < 1e-6 and abs(z1 - l_small_mm) < 1e-6 \
                    and abs(x1 - rs) < 0.5:
                shoulder.append(abs(tag))
        assert len(shoulder) == 1, (
            f"shoulder-edge selection found {len(shoulder)} edges, expected 1 "
            f"— specimen geometry changed; fix the selector before meshing")
        occ.fillet([vol], shoulder, [r_mm], removeVolume=True)
        occ.synchronize()
        # refine across the fillet: size ~ r/... is set by the caller via elem_size
        gmsh.option.setNumber("Mesh.MeshSizeMax", elem_size_mm)
        gmsh.option.setNumber("Mesh.MeshSizeMin", elem_size_mm * 0.5)
        gmsh.option.setNumber("Mesh.ElementOrder", order)
        gmsh.option.setNumber("Mesh.HighOrderOptimize", 1)
        gmsh.model.mesh.generate(3)
        m = _gmsh_extract(gmsh.model)
        m.meta.update({"specimen": "shouldered_bar",
                       "d_mm": d_mm, "D_mm": D_mm, "r_mm": r_mm,
                       "l_small_mm": l_small_mm, "l_large_mm": l_large_mm,
                       "elem_size_mm": elem_size_mm})
        return m
    finally:
        gmsh.finalize()


def mesh_box(lx_mm: float, ly_mm: float, lz_mm: float,
             *, elem_size_mm: float, order: int = 2) -> MeshIR:
    """Axis-aligned box [0,lx]x[0,ly]x[0,lz] as a tet10 mesh — the smooth
    validation specimen (cantilever / axial patch test)."""
    import gmsh

    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.add("box")
        gmsh.model.occ.addBox(0, 0, 0, lx_mm, ly_mm, lz_mm)
        gmsh.model.occ.synchronize()
        gmsh.option.setNumber("Mesh.MeshSizeMax", elem_size_mm)
        gmsh.option.setNumber("Mesh.MeshSizeMin", elem_size_mm * 0.5)
        gmsh.option.setNumber("Mesh.ElementOrder", order)
        gmsh.model.mesh.generate(3)
        m = _gmsh_extract(gmsh.model)
        m.meta.update({"specimen": "box", "lx_mm": lx_mm, "ly_mm": ly_mm,
                       "lz_mm": lz_mm, "elem_size_mm": elem_size_mm})
        return m
    finally:
        gmsh.finalize()


def analytic_shouldered_bar_volume(d_mm, D_mm, r_mm, l_small_mm, l_large_mm) -> float:
    """Nominal two-cylinder volume IGNORING the small fillet fillet-scoop — used
    only as a loose (>=1% tol) volume sanity ref, not an exactness claim."""
    import math
    rs, rl = d_mm / 2.0, D_mm / 2.0
    return math.pi * rs**2 * l_small_mm + math.pi * rl**2 * l_large_mm
