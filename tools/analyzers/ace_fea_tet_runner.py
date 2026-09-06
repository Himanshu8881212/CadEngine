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
    max_mm} | {type:'cylinder',axis,center_mm,radius_mm[,length_mm]}.

    high_order_optimize  optional  default true — gmsh's curved-element
                                 untangler; set false when it aborts
    second_order_linear  optional  default false — straight-sided tet10
    dof_budget           optional  default 1.5e6 — the job is REFUSED before
                                 meshing when the STL-volume estimate exceeds it
                                 (receipt `cost_estimate` carries the model)

    gmsh runs in an ISOLATED child process: its native abort (SIGABRT on the
    high-order optimiser) becomes a refusal receipt instead of a vanished run.
    Every mesher refusal (PLC, parametrisation, inverted element, budget) is
    exit 2 / `refusal.MeshRefusal`. The receipt's `peak` block says where the
    reported maximum sits relative to the fixtures (`on_fixture_boundary`).

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

ANALYZER_VERSION = "reference_fea_tet/tet10-body-fitted/v2 (isolated gmsh, cost model, peak receipt)"


class MeshRefusal(Exception):
    """The mesher RAN and could not produce a usable tet mesh from this
    geometry at this element size: gmsh's PLC / parametrisation / boundary
    refusals, its high-order optimiser abort, an inverted-element mesh, or a
    request above the DOF budget. A refusal (exit 2, `refusal.MeshRefusal`),
    never `internal` (slas F5: a gmsh PLC refusal exited 1 as a tool bug)."""


# Tet-count heuristic for a gmsh tet10 mesh of a volume V at element size h,
# calibrated on the in-tree specimens (a Ø132/Ø107 × 5 annulus: 23 463 mm³ at
# h = 5 → 3 683 tets, 7 846 nodes): n_tets ≈ V / (0.06 h³) ± 40 %, n_nodes ≈
# 2.1 n_tets (tet10), DOF = 3 n_nodes. A planning number, stated as such.
TET_FILL = 0.06
NODES_PER_TET10 = 2.1


def estimate_cost(stl_volume_mm3: float, elem: float) -> dict:
    n_tets = stl_volume_mm3 / (TET_FILL * elem ** 3)
    n_nodes = NODES_PER_TET10 * n_tets
    return {
        "basis": "n_tets ≈ V / (0.06 h³), n_nodes ≈ 2.1 n_tets (tet10), DOF = 3 n_nodes — calibrated "
                 "on in-tree specimens, ±40 %; a planning figure, not a measurement",
        "stl_volume_mm3": round(stl_volume_mm3, 3),
        "elem_size_mm": elem,
        "estimated_tets": int(n_tets),
        "estimated_nodes": int(n_nodes),
        "estimated_dof": int(3 * n_nodes),
    }


def _mesh_stl_child(stl, elem, hoo, sol, out_npz, err_txt):
    """Body of the isolated gmsh process (see `mesh_stl_isolated`)."""
    import json as _json
    import numpy as _np
    try:
        from physics import mesh_ir as M
        m = M.mesh_stl(stl, elem_size_mm=elem, high_order_optimize=hoo, second_order_linear=sol)
        _np.savez(out_npz, nodes_mm=m.nodes_mm, tets=m.tets, surf_tris=m.surf_tris,
                  surf_group=m.surf_group, meta=_json.dumps(m.meta, default=str))
    except BaseException as exc:  # noqa: BLE001 — the parent classifies it
        with open(err_txt, "w", encoding="utf-8") as f:
            f.write(f"{type(exc).__name__}: {exc}")


def mesh_stl_isolated(stl: str, elem: float, high_order_optimize: bool, second_order_linear: bool):
    """Run gmsh in a CHILD process. Its high-order optimiser can raise an
    uncaught C++ exception ("Failed to reach critical value in pass 0 for
    measure(s): ScaledJac") that terminates the whole process with SIGABRT —
    no stdout, no receipt, and a wall budget cannot catch native code
    (reservoir F3, turgo F2). In a child that abort becomes a refusal receipt
    in the parent, with the hint the docstring of `mesh_ir.mesh_stl` gives."""
    import multiprocessing as mp
    import tempfile

    import numpy as np
    from physics import mesh_ir as M

    ctx = mp.get_context("spawn")
    with tempfile.TemporaryDirectory() as td:
        out_npz = os.path.join(td, "mesh.npz")
        err_txt = os.path.join(td, "error.txt")
        proc = ctx.Process(target=_mesh_stl_child,
                           args=(stl, elem, high_order_optimize, second_order_linear, out_npz, err_txt))
        proc.start()
        proc.join()
        if os.path.exists(err_txt):
            with open(err_txt, encoding="utf-8") as f:
                msg = f.read()
            raise MeshRefusal(
                f"gmsh could not mesh {stl!r} at elem_size_mm {elem}: {msg}. This is the mesher "
                f"refusing the surface, not a tool fault — try another elem_size_mm (one specific "
                f"size can hit a parametrisation failure while its neighbours mesh), re-export the "
                f"STL at a coarser chord tolerance (tol 0.05) to remove sliver facets, or set "
                f"high_order_optimize: false / second_order_linear: true in the job")
        if proc.exitcode != 0 or not os.path.exists(out_npz):
            raise MeshRefusal(
                f"gmsh TERMINATED the meshing process (exit code {proc.exitcode}; SIGABRT = -6/134) "
                f"while meshing {stl!r} at elem_size_mm {elem} — its high-order optimiser aborts on "
                f"slender / organic surface topologies ('Failed to reach critical value ... ScaledJac'). "
                f"Set high_order_optimize: false (and second_order_linear: true for straight-sided "
                f"tet10) or change elem_size_mm; the parent runner survived because gmsh ran isolated")
        z = np.load(out_npz, allow_pickle=False)
        meta = json.loads(str(z["meta"]))
        return M.MeshIR(nodes_mm=z["nodes_mm"], tets=z["tets"], surf_tris=z["surf_tris"],
                        surf_group=z["surf_group"], meta=meta)


def build_mesh(job: dict):
    """Resolve the job's geometry block to a MeshIR. Returns (mesh, seconds)."""
    from physics import mesh_ir as M

    elem = float(job["elem_size_mm"])
    specimen = job.get("specimen")
    t0 = time.monotonic()
    if job.get("stl"):
        # `high_order_optimize` / `second_order_linear` are gmsh's own escape
        # hatches for surfaces it otherwise refuses (singulator F9: the mesher
        # exposed them, the job schema did not).
        mesh = mesh_stl_isolated(job["stl"], elem,
                                 bool(job.get("high_order_optimize", True)),
                                 bool(job.get("second_order_linear", False)))
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
    from physics.fea_tet import nodes_in_selector, reference_fea_tet

    # COST MODEL before any meshing (rotor F9: a part with sub-3 mm features had
    # no usable elem_size window and no number to plan around): the STL volume
    # gives an expected tet/node/DOF count; above `dof_budget` (default 1.5e6)
    # the job is refused with the element size that would fit.
    elem = float(job["elem_size_mm"])
    cost = None
    if job.get("stl"):
        try:
            from _stl import load_stl
            tris = load_stl(job["stl"])
            v = float(abs(np.einsum("ij,ij->i", tris[:, 0], np.cross(tris[:, 1], tris[:, 2])).sum()) / 6.0)
            cost = estimate_cost(v, elem)
            budget = int(job.get("dof_budget", 1_500_000))
            cost["dof_budget"] = budget
            direct_max = int(job.get("direct_max_dof", 250_000))
            cost["expected_solver_path"] = "SuperLU direct" if cost["estimated_dof"] <= direct_max else "Jacobi-CG (above direct_max_dof)"
            if cost["estimated_dof"] > budget:
                fit = elem * (cost["estimated_dof"] / budget) ** (1.0 / 3.0)
                raise MeshRefusal(
                    f"estimated {cost['estimated_dof']:,} DOF at elem_size_mm {elem} exceeds dof_budget "
                    f"{budget:,} — elem_size_mm ≥ {fit:.2f} would fit (DOF ∝ 1/h³), or raise dof_budget "
                    f"knowingly; estimate: {cost['basis']}")
            log(f"cost estimate: ~{cost['estimated_tets']:,} tets, ~{cost['estimated_dof']:,} DOF ({cost['expected_solver_path']})")
        except MeshRefusal:
            raise
        except Exception as exc:  # noqa: BLE001 — an estimate must never sink a run
            log(f"cost estimate unavailable: {type(exc).__name__}: {exc}")

    mesh, mesh_s = build_mesh(job)
    # MeshIR.check() raises on an inverted element — a real mesh-quality gate;
    # it is the mesh receipt the success payload carries. An inverted element
    # is the MESHER's refusal of this surface at this size (gripper F5), not
    # a tool fault.
    try:
        mesh_receipt = mesh.check(volume_ref_mm3=job.get("volume_ref_mm3"))
    except AssertionError as exc:
        raise MeshRefusal(
            f"{exc} — the tet mesh gmsh produced at elem_size_mm {elem} contains an inverted / "
            f"degenerate element. Slender facets on the STL (a finely faceted fillet next to a "
            f"flat) are the usual cause: re-export the STL at chord tol 0.05, change elem_size_mm, "
            f"or set second_order_linear: true (straight-sided tet10)") from exc
    if cost is not None:
        cost["actual_tets"] = int(mesh.n_tets)
        cost["actual_nodes"] = int(mesh.n_nodes)
        cost["actual_dof"] = int(3 * mesh.n_nodes)

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

    # WHERE the peak is (turgo F12): a peak within ~1.5 element sizes of a
    # clamped/pinned node set is a boundary-condition singularity — it does not
    # converge under refinement and is not an upper bound. Say so in the
    # receipt, at the point of use, instead of leaving it to a probe script.
    peak = None
    peak_warning = None
    try:
        vm = np.asarray(res["vm_nodal"], dtype=float)
        ipk = int(np.argmax(vm))
        ppk = np.asarray(mesh.nodes_mm[ipk], dtype=float)
        fmask = np.zeros(mesh.n_nodes, dtype=bool)
        for fx in job.get("fixtures", []) or []:
            fmask |= np.asarray(nodes_in_selector(mesh.nodes_mm, fx["region_selector"]), dtype=bool)
        dist = float(np.min(np.linalg.norm(mesh.nodes_mm[fmask] - ppk, axis=1))) if fmask.any() else None
        on_boundary = dist is not None and dist <= 1.5 * elem
        peak = {
            "node": ipk,
            "at_mm": [round(float(c), 4) for c in ppk],
            "distance_to_fixture_mm": None if dist is None else round(dist, 4),
            "on_fixture_boundary": bool(on_boundary),
            "criterion": "on_fixture_boundary = within 1.5 x elem_size_mm of a fixture node",
        }
        if on_boundary:
            peak_warning = {
                "kind": "stress.peak_on_fixture_boundary",
                "message": (f"max_von_mises_pa is read at node {ipk}, {dist:.3f} mm from a fixture "
                            f"(elem_size_mm {elem}): a rigid-clamp edge singularity. It does not "
                            f"converge under refinement (measured +9.6 % between two grids on a "
                            f"sub-model) and is NOT an upper bound — quote the field away from the "
                            f"clamp, or refine and compare, before gating on it."),
            }
    except Exception as exc:  # noqa: BLE001
        notes.append(f"peak-location receipt unavailable: {type(exc).__name__}: {exc}")

    payload = {
        "ok": True,
        "max_von_mises_pa": res["max_von_mises_pa"],
        "max_displacement_m": res["max_disp_m"],
        "n_nodes": res["n_nodes"],
        "n_tets": res["n_tets"],
        "method": res["method"],
        "solver": res.get("solver"),
        "mesh": mesh_receipt,
        "peak": peak,
        "cost_estimate": cost,
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
    apply_warnings(payload, job, [validated_range_warning(vrange), peak_warning])

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
    run_cli("ace_fea_tet", main, install_hint=PHYSICS_INSTALL_HINT, refusal_types=(MeshRefusal,))
