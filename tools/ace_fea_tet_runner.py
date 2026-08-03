#!/usr/bin/env python3
"""ace_fea_tet_runner.py — one-shot body-fitted tet10 reference FEA.

Bridge runner spawned by the LMCAD MCP server (``lmcad-mcp`` tool ``ace_fea``
with ``mesh:"body_fitted"``) to run ACE's validated body-fitted tet10
linear-elastic solver (``engine.verify.fea_tet.reference_fea_tet``) on a
conforming tet10 mesh (``engine.verify.mesh_ir.MeshIR``). This is the
CURVED-GEOMETRY twin of ``ace_fea_runner.py`` (hex8 voxel grid): a true conic
fillet is a real surface here, not a voxel staircase, so it resolves stress
concentrations the voxel path under-reads.

Usage:  <ACE_PYTHON> ace_fea_tet_runner.py <job.json>

Job JSON (geometry in mm, physics in SI). Same material/fixtures/loads/selector
schema as ace_fea_runner.py; the ONLY differences are the GEOMETRY block and
the unstructured field outputs (documented below).

    out_dir            REQUIRED  directory for field .npy outputs
    elem_size_mm       REQUIRED  target tet edge length (mm)
    GEOMETRY, one of:
      stl                        absolute path of a WATERTIGHT surface STL
                                 (mesh_stl)
      specimen:"shouldered_bar"  + d, D, r, l_small, l_large (mm)
                                 (mesh_shouldered_bar — the Kt benchmark)
      specimen:"box"             + lx, ly, lz (mm)   (mesh_box)
    material           REQUIRED  {youngs_modulus_pa, poisson, density_kg_m3}
                                 (or a material-key string, resolved like the
                                 voxel runner)
    fixtures           REQUIRED  [{kind:'clamped'|'pinned', region_selector,
                                   dof_constrained?}]
    loads              optional  [{kind:'point', magnitude, direction (unit
                                   3-vec), region_selector}]
    volume_ref_mm3     optional  analytic volume to cross-check the mesh against
    direct_max_dof     optional  default 250000 (SuperLU below, Jacobi-CG above)
    cg_tol/cg_maxiter  optional  CG tolerance / iteration cap

    Selectors are GEOMETRIC on the mesh NODES (world mm). The tet path supports
    {type:'all'} | {type:'plane',axis,value_mm,side} | {type:'box',min_mm,
    max_mm} — cylinder/sphere are NOT supported by the tet selector engine and
    are refused loudly (the voxel runner accepts them).

Output contract (IDENTICAL to ace_fea_runner.py): the LAST non-empty stdout
line is ONE JSON object; all logging goes to stderr. Success =>
{ok:true, max_von_mises_pa, ...}; any failure => {ok:false, error} and STILL
exit 0 — the JSON line is the contract, not the exit code. Success also carries
a mesh receipt {n_tets, n_nodes, min_corner_jacobian_mm3, volume_mm3} from
MeshIR.check() and per-selector node-count receipts (selector_count_unit:
"nodes").

FIELD OUTPUTS ARE UNSTRUCTURED (this is the honest difference from the voxel
runner's structured (nx,ny,nz) grids): the mesh is body-fitted tetrahedra, so
disp_field.npy is (N_nodes, 3) displacement in metres, stress_field.npy is
(N_nodes,) nodal von Mises in Pa, and nodes_mm.npy is (N_nodes, 3) node
coordinates in mm to interpret them. The receipt's ``field_layout`` says so.

Honest caveats: body-fitted tet10 resolves the fillet peak the voxel grid
misses, but the reported peak is still nodal-recovered and mesh-dependent —
refine elem_size_mm to confirm convergence. Only point loads and clamped/pinned
fixtures are wired in the tet solver.
"""
from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _ace import (  # noqa: E402  — importing runs the boot side effects (ACE on path, kernel-api env)
    ACE_INSTALL_HINT,
    emit,
    log,
    resolve_material,
)

ANALYZER_VERSION = "reference_fea_tet/tet10-body-fitted/v1"


def build_mesh(job: dict):
    """Resolve the job's geometry block to a MeshIR. Returns (mesh, seconds)."""
    from engine.verify import mesh_ir as M

    elem = float(job["elem_size_mm"])
    specimen = job.get("specimen")
    t0 = time.monotonic()
    if job.get("stl"):
        mesh = M.mesh_stl(job["stl"], elem_size_mm=elem)
        log(f"meshed STL {job['stl']} -> {mesh.n_tets} tets")
    elif specimen == "shouldered_bar":
        mesh = M.mesh_shouldered_bar(
            float(job["d"]), float(job["D"]), float(job["r"]),
            float(job["l_small"]), float(job["l_large"]), elem_size_mm=elem)
        log(f"meshed shouldered_bar -> {mesh.n_tets} tets")
    elif specimen == "box":
        mesh = M.mesh_box(float(job["lx"]), float(job["ly"]), float(job["lz"]),
                          elem_size_mm=elem)
        log(f"meshed box -> {mesh.n_tets} tets")
    else:
        raise ValueError(
            "job needs a geometry block: 'stl' path, or specimen "
            "'shouldered_bar' (d,D,r,l_small,l_large) or 'box' (lx,ly,lz)")
    return mesh, time.monotonic() - t0


def selector_receipts(job: dict, mesh):
    """Per-selector NODE-count receipts, resolved with the tet solver's OWN
    ``nodes_in_selector`` so the counts match exactly what the solve pinned or
    loaded. Mirrors the voxel runner's receipt shape (selector_count_unit
    "nodes")."""
    from engine.verify.fea_tet import nodes_in_selector

    def count(entry):
        return int(nodes_in_selector(mesh.nodes_mm, entry["region_selector"]).sum())

    fixtures = [{"kind": fx.get("kind"), "nodes_or_elements": count(fx)}
                for fx in job.get("fixtures", []) or []]
    loads = [{"kind": ld.get("kind"), "nodes_or_elements": count(ld),
              "magnitude": ld.get("magnitude")}
             for ld in job.get("loads", []) or []]
    return fixtures, loads


def main() -> None:
    job = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
    job["material"] = resolve_material(job["material"])  # Unit 3: single materials source
    out_dir = Path(job["out_dir"])
    out_dir.mkdir(parents=True, exist_ok=True)

    import numpy as np
    from engine.verify.fea_tet import reference_fea_tet

    mesh, mesh_s = build_mesh(job)
    # MeshIR.check() raises on an inverted element — a real mesh-quality gate;
    # it is the mesh receipt the success payload carries.
    mesh_receipt = mesh.check(volume_ref_mm3=job.get("volume_ref_mm3"))

    t0 = time.monotonic()
    res = reference_fea_tet(
        mesh, job["material"],
        job.get("loads", []), job.get("fixtures", []),
        direct_max_dof=int(job.get("direct_max_dof", 250_000)),
        cg_tol=float(job.get("cg_tol", 1e-9)),
        cg_maxiter=int(job.get("cg_maxiter", 20000)),
    )
    fea_s = time.monotonic() - t0

    if not res.get("ok"):
        # The solver refused (0-node selector, singular system, inverted tet,
        # unsupported load) — surface it on the same {ok:false,error} contract.
        emit({"ok": False, "error": f"tet solver refused: {res.get('error')}",
              "mesh": mesh_receipt})
        return

    stress_npy = out_dir / "stress_field.npy"   # (N_nodes,) nodal von Mises, Pa
    disp_npy = out_dir / "disp_field.npy"        # (N_nodes,3) displacement, m
    nodes_npy = out_dir / "nodes_mm.npy"         # (N_nodes,3) node coords, mm
    np.save(stress_npy, res["vm_nodal"])
    np.save(disp_npy, res["disp"])
    np.save(nodes_npy, mesh.nodes_mm)

    notes = list(res.get("notes", []))
    try:
        fixtures, loads = selector_receipts(job, mesh)
    except Exception as exc:  # noqa: BLE001 — receipts must never sink a good solve
        fixtures, loads = [], []
        notes.append(f"selector receipts unavailable: {type(exc).__name__}: {exc}")

    payload = {
        "ok": True,
        "max_von_mises_pa": res["max_von_mises_pa"],
        "max_displacement_m": res["max_disp_m"],
        "n_nodes": res["n_nodes"],
        "n_tets": res["n_tets"],
        "method": res["method"],
        "solver": res.get("solver"),
        "mesh": mesh_receipt,
        "fixtures": fixtures,
        "loads": loads,
        "selector_count_unit": "nodes",
        "field_layout": {
            "kind": "unstructured_tet10",
            "stress_field_npy": {"path": str(stress_npy), "shape": "(n_nodes,)",
                                 "field": "nodal von Mises", "units": "Pa"},
            "disp_field_npy": {"path": str(disp_npy), "shape": "(n_nodes,3)",
                               "field": "displacement", "units": "m"},
            "nodes_mm_npy": {"path": str(nodes_npy), "shape": "(n_nodes,3)",
                             "units": "mm"},
            "note": "body-fitted tet mesh — fields are per-node, NOT a "
                    "structured (nx,ny,nz) grid like the voxel runner's",
        },
        "notes": notes,
        "analyzer_version": ANALYZER_VERSION,
        "timings_s": {"mesh_s": round(mesh_s, 3), "fea_s": round(fea_s, 3)},
    }
    emit(payload)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:  # noqa: BLE001 — the JSON line IS the contract
        error = f"{type(exc).__name__}: {exc}"
        if isinstance(exc, (ImportError, ModuleNotFoundError)) and "engine" in str(exc):
            error += f" | hint: {ACE_INSTALL_HINT}"
        emit({"ok": False, "error": error})
        sys.exit(0)
