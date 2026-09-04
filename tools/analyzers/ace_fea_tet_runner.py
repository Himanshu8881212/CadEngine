#!/usr/bin/env python3
"""ace_fea_tet_runner.py — one-shot body-fitted tet10 reference FEA.

Standalone job runner (``python3 tools/ace_fea_tet_runner.py job.json``, i.e.
``ace_fea`` with ``mesh:"body_fitted"``) running ACE's validated body-fitted tet10
linear-elastic solver (``physics.fea_tet.reference_fea_tet``) on a
conforming tet10 mesh (``physics.mesh_ir.MeshIR``). This is the
CURVED-GEOMETRY twin of ``ace_fea_runner.py`` (hex8 voxel grid): a true conic
fillet is a real surface here, not a voxel staircase, so it resolves stress
concentrations the voxel path under-reads.

Usage:  python3 ace_fea_tet_runner.py <job.json>

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
a NONZERO exit (see THE WIRE + EXIT CONTRACT below). Success also carries
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

THE WIRE + EXIT CONTRACT (shared; see tools/_receipt.py for the full rules):
    python3 <runner>.py <job.json> [--out PATH]
  The LAST non-empty stdout line is ONE JSON receipt; all logging goes to
  stderr. The exit code AGREES with the receipt, always:
    exit 0  ok:true   analysis ran, receipt usable
    exit 1  ok:false   the tool could not run the request (usage, unreadable
                       job, internal error) — NO analysis was performed
    exit 2  ok:false   the tool RAN and REFUSED, or the analysis failed
  `error_kind` is a machine-matchable slug (`refusal.*`, `timeout`, `killed.*`,
  `internal`, `usage`, `receipt_path_conflict`). CHANGED 2026-08-08: this runner
  used to exit 0 on ok:false by design. Parsing `ok` still works and is still
  correct; `$?` now works too. `LMCAD_RUNNER_EXIT=legacy` or a job
  `"legacy_exit_zero": true` restores exit-0-always and records the opt-out in
  `exit_contract.mode`.
  `--out PATH` writes the receipt atomically (temp+rename) so an interrupted run
  can never leave a zero-byte file where a good receipt was; a job-level
  `receipt` key that disagrees with `--out` is REFUSED, not silently preferred.
  `LMCAD_RECEIPT_DRY_RUN=1` suppresses every on-disk write (safe what-if runs).
  `"wall_budget_s"` (or `LMCAD_WALL_BUDGET_S`), SIGTERM and SIGINT all produce
  an honest ok:false receipt instead of a vanished run.
  `determinism` names the receipt's wall-clock fields and carries `core_digest`,
  a sha256 over the rest at 12 significant figures — compare THAT between runs,
  never the receipt bytes.
"""
from __future__ import annotations

import hashlib
import json
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # tools/: the shared contracts + the layout map
import _layout  # noqa: E402
_layout.add_import_paths()  # tools/, tools/analyzers, tools/publish — sibling-style imports keep working after the 2026-09-02 move
from _ace import (  # noqa: E402  — importing runs the boot side effects (physics package on path, kernel-api env)
    PHYSICS_INSTALL_HINT,
    apply_warnings,
    determinism_block,
    emit,
    finish,
    load_job,
    log,
    provenance_fields,
    resolve_material,
    runtime_provenance,
    run_cli,
    validated_range_check,
    validated_range_warning,
)

ANALYZER_VERSION = "reference_fea_tet/tet10-body-fitted/v1"


def build_mesh(job: dict):
    """Resolve the job's geometry block to a MeshIR. Returns (mesh, seconds)."""
    from physics import mesh_ir as M

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
    from physics.fea_tet import nodes_in_selector

    def count(entry):
        return int(nodes_in_selector(mesh.nodes_mm, entry["region_selector"]).sum())

    fixtures = [{"kind": fx.get("kind"), "nodes_or_elements": count(fx)}
                for fx in job.get("fixtures", []) or []]
    loads = [{"kind": ld.get("kind"), "nodes_or_elements": count(ld),
              "magnitude": ld.get("magnitude")}
             for ld in job.get("loads", []) or []]
    return fixtures, loads


def main() -> None:
    job, out = load_job()
    runtime_provenance(job)  # release strictness is checked before meshing/artifacts
    job["material"] = resolve_material(job["material"])  # Unit 3: single materials source
    out_dir = Path(job["out_dir"])
    out_dir.mkdir(parents=True, exist_ok=True)

    import numpy as np
    from physics.fea_tet import reference_fea_tet

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
        # It used to `return` here, i.e. EXIT 0 on a refused analysis; the
        # exit code now agrees with `ok` (T3).
        finish({"ok": False, "error": f"tet solver refused: {res.get('error')}",
                "mesh": mesh_receipt}, job=job, tool="ace_fea_tet", out=out,
               kind="refusal.tet_solver")

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
    vrange = validated_range_check(job, "tools/manifests/ace_fea_tet.manifest.json")
    payload["validated_range"] = vrange
    apply_warnings(payload, job, [validated_range_warning(vrange)])

    # Bind the receipt to the exact body-fitted mesh solved, not merely the STL
    # or specimen parameters from which gmsh happened to generate it.
    mesh_digest = hashlib.sha256()
    for array in (mesh.nodes_mm, mesh.tets, mesh.surf_tris, mesh.surf_group):
        a = np.ascontiguousarray(array)
        mesh_digest.update(str(a.dtype).encode("ascii"))
        mesh_digest.update(json.dumps(list(a.shape)).encode("ascii"))
        mesh_digest.update(a.tobytes(order="C"))
    exact_mesh_hash = "tet10-mesh:sha256:" + mesh_digest.hexdigest()
    payload.update(provenance_fields(
        job, res, analyzer_name="ace_fea_tet", analyzer_version=ANALYZER_VERSION,
        values={"max_von_mises_pa": res["max_von_mises_pa"],
                "max_displacement_m": res["max_disp_m"]},
        manifest_ref="tools/manifests/ace_fea_tet.manifest.json",
        geometry_hash=exact_mesh_hash,
        validation_applicability=vrange))
    payload["determinism"] = determinism_block(
        payload, nondeterministic_paths=["timings_s"],
        solver_note=("SuperLU direct (or CG) on a gmsh tet10 mesh: the mesh itself is "
                     "bit-identical run to run, but the factorisation's reduction order "
                     "is not pinned, so peak stress moves ~2e-14 relative. Compare "
                     "core_digest, not receipt bytes."))
    finish(payload, job=job, tool="ace_fea_tet", out=out)


if __name__ == "__main__":
    run_cli("ace_fea_tet", main, install_hint=PHYSICS_INSTALL_HINT)
