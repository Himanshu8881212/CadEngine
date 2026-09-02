#!/usr/bin/env python3
"""ace_fea_runner.py — one-shot hex8 reference FEA on LMCAD geometry.

Bridge runner spawned by the LMCAD MCP server (``lmcad-mcp`` tool ``ace_fea``)
to run ACE's benchmark-validated hex8 linear-elastic solver
(``engine.verify.reference_fea``) on geometry built by the LMCAD kernel.

Usage:  <ACE_PYTHON> ace_fea_runner.py <job.json>

Job JSON (all geometry in mm, physics in SI):
    out_dir            REQUIRED  directory for field .npy outputs
    voxel_mm           REQUIRED  cubic voxel edge (mm)
    origin_mm          optional  world coord of grid node (0,0,0); default [0,0,0]
    GEOMETRY, one of:
      ops + solid + shape [+ supersample=2]   LMCAD JSON ops; `solid` names the
                                              op id sampled onto the grid via
                                              engine.lmcad.sample_part
      npy                                     absolute path of an existing
                                              (nx,ny,nz) float density .npy
    regions            optional  [{kind: frozen|fixed|design|void, selector}]
                                 resolved by the FEA's own selector engine;
                                 absent => whole grid is `design` (no override)
    material           REQUIRED  {youngs_modulus_pa, poisson, density_kg_m3}
    fixtures           REQUIRED  [{kind: clamped|pinned|slider, region_selector,
                                   dof_constrained?}]
    loads              optional  [{kind: point|body|pressure, magnitude,
                                   direction (unit 3-vec, point/body only),
                                   region_selector}]
    simp_penalty       optional  null (default) = binary as-built occupancy
                                 (rho >= 0.5); float p = SIMP density mode
    density_floor      optional  SIMP activity floor, default 0.02
    direct_solver_max_dof  optional, default 0 = always Jacobi-CG (the direct
                                 SuperLU path needs ~10 GB at 237k DOF)

Output contract: the LAST non-empty stdout line is ONE JSON object; all
logging goes to stderr. Success => {ok:true, max_von_mises_pa, ...}; any
failure => {ok:false, error} and a NONZERO exit (see THE WIRE + EXIT
CONTRACT below). stress_field.npy / disp_field.npy land in
out_dir. Success payloads also carry per-selector receipts —
``fixtures: [{kind, nodes_or_elements}]`` and ``loads: [{kind,
nodes_or_elements, magnitude}]`` (counts are ACTIVE grid NODES touching the
selected active elements, ``selector_count_unit: "nodes"``) — plus a
"suspiciously broad" note whenever a load selector catches > 30% of all
active elements (the smeared-load mistake behind an earlier 3x-wrong
benchmark).

Honest caveats (echoed by the MCP tool description): coarse hex8 grids
under-predict peak bending stress by roughly 20% vs a converged mesh; in
SIMP mode the reported stress is the homogenized rho_eff^p * D B u, not a
solid-material stress.

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

import json
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _ace import (  # noqa: E402  — importing runs the boot side effects (ACE on path, kernel-api env)
    ACE_INSTALL_HINT,
    apply_warnings,
    build_region_kind,
    determinism_block,
    emit,
    finish,
    load_geometry,
    load_job,
    log,
    mesh_resolution_receipt,
    mesh_resolution_warning,
    provenance_fields,
    refuse_empty_selectors,
    resolve_material,
    run_cli,
    selector_catch_audit,
    validated_range_check,
    validated_range_warning,
)

ANALYZER_VERSION = "reference_fea/hex8-jacobi-cg/v1"


def selector_receipts(job: dict, rho, kind, voxel: float, origin):
    """Per-selector node-count receipts: (fixtures, loads, extra notes).

    Resolves every fixture/load ``region_selector`` with ACE's own selector
    engine (``engine.verify.selectors.resolve_selector``), intersects it with
    the same active-element mask the solve uses (``engine.verify.fea._occupancy``
    — reused directly so the receipt counts EXACTLY what the FEA loaded or
    clamped), and counts the grid nodes touching the selected active elements
    via the public ``element_mask_to_node_ids``. A selector catching 0 already
    errors inside ACE; the value here is catching ACCIDENTALLY-HUGE selections
    (a load smeared over the whole part — the mistake behind an earlier
    3x-wrong benchmark), so any load selector covering > 30% of all active
    elements gets a "suspiciously broad" note appended to the receipts.
    """
    from engine.verify.fea import _occupancy  # the solve's own activity rule
    from engine.verify.selectors import element_mask_to_node_ids, resolve_selector

    simp = job.get("simp_penalty")
    floor = float(job.get("density_floor", 0.02)) if simp is not None else None
    occ = _occupancy(rho, kind, simp_floor=floor)
    n_active = int(occ.sum())

    def count(entry):
        sel = entry.get("region_selector", {"type": "all"})
        mask = resolve_selector(sel, rho.shape, voxel, origin) & occ
        return int(mask.sum()), int(element_mask_to_node_ids(mask).size)

    fixtures, loads, notes = [], [], []
    for fx in job.get("fixtures", []) or []:
        _, nodes = count(fx)
        fixtures.append({"kind": fx.get("kind"), "nodes_or_elements": nodes})
    for li, ld in enumerate(job.get("loads", []) or []):
        elems, nodes = count(ld)
        loads.append({
            "kind": ld.get("kind"),
            "nodes_or_elements": nodes,
            "magnitude": ld.get("magnitude"),
        })
        if n_active > 0 and elems > 0.30 * n_active:
            notes.append(
                f"load[{li}] ({ld.get('kind')}): selector catches {elems}/{n_active} "
                f"active elements ({elems / n_active:.0%}) — suspiciously broad — "
                f"verify the selector"
            )
    return fixtures, loads, notes


def main() -> None:
    job, out = load_job()
    job["material"] = resolve_material(job["material"])  # Unit 3: single materials source
    out_dir = Path(job["out_dir"])
    out_dir.mkdir(parents=True, exist_ok=True)

    import numpy as np
    from engine.verify.fea import _occupancy, reference_fea

    rho, origin, voxel, sample_s = load_geometry(job, out_dir)
    kind = build_region_kind(job, rho.shape, voxel, origin)

    # --- ADMISSIBILITY, BEFORE the solve (T13) -----------------------------
    # A selector catching zero ACTIVE elements is a broken model: ACE turns a
    # 0-node FIXTURE into a note and keeps ok:true, so a run that quietly lost
    # 4 of its 6 boundary conditions still returns a converged-looking field
    # (rotor F6). Refuse at the zero end, exactly as the existing
    # "suspiciously broad" note guards the >30% end.
    simp_floor = (float(job.get("density_floor", 0.02))
                  if job.get("simp_penalty") is not None else None)
    occ = _occupancy(rho, kind, simp_floor=simp_floor)
    catch = selector_catch_audit(job, occ, voxel, origin)
    refuse_empty_selectors(catch)
    mesh_res = mesh_resolution_receipt(occ, voxel)
    vrange = validated_range_check(job, "tools/manifests/ace_fea.manifest.json")

    t0 = time.monotonic()
    res = reference_fea(
        rho, kind, voxel, job["material"],
        job.get("loads", []), job.get("fixtures", []),
        simp_penalty=job.get("simp_penalty"),
        density_floor=float(job.get("density_floor", 0.02)),
        origin_mm=origin,
        direct_solver_max_dof=int(job.get("direct_solver_max_dof", 0)),
    )
    fea_s = time.monotonic() - t0

    stress_npy = out_dir / "stress_field.npy"
    disp_npy = out_dir / "disp_field.npy"
    np.save(stress_npy, res["stress_field"])
    np.save(disp_npy, res["disp_field"])

    notes = list(res["notes"])
    try:
        fixtures, loads, broad = selector_receipts(job, rho, kind, voxel, origin)
        notes.extend(broad)
    except Exception as exc:  # noqa: BLE001 — receipts must never sink a good solve
        fixtures, loads = [], []
        notes.append(f"selector receipts unavailable: {type(exc).__name__}: {exc}")

    payload = {
        "ok": True,
        "max_von_mises_pa": res["max_von_mises_pa"],
        "max_displacement_m": res["max_displacement_m"],
        "tip_displacement_m": res["tip_displacement_m"],
        "n_active_elements": res["n_active_elements"],
        "n_dof": res["n_dof"],
        "method": res["method"],
        "fixtures": fixtures,
        "loads": loads,
        "selector_count_unit": "nodes",
        "selector_catch_audit": catch,
        "mesh_resolution": mesh_res,
        "validated_range": vrange,
        "notes": notes,
        "stress_field_npy": str(stress_npy),
        "disp_field_npy": str(disp_npy),
        "timings_s": {"sample_s": round(sample_s, 3), "fea_s": round(fea_s, 3)},
    }
    if "compliance" in res:
        payload["compliance"] = res["compliance"]
    apply_warnings(payload, job, [
        mesh_resolution_warning(mesh_res),
        validated_range_warning(vrange),
    ])
    # Provenance envelope: geometry hash + structured convergence receipt +
    # the lmcad.analysis.v1 envelope, ADDED alongside the scalar fields above.
    payload.update(provenance_fields(
        job, res, analyzer_name="ace_fea", analyzer_version=ANALYZER_VERSION,
        values={"max_von_mises_pa": res["max_von_mises_pa"],
                "max_displacement_m": res["max_displacement_m"]},
        manifest_ref="tools/manifests/ace_fea.manifest.json",
        validation_applicability=vrange))
    payload["determinism"] = determinism_block(
        payload, nondeterministic_paths=["timings_s"],
        solver_note=("Jacobi-preconditioned CG at rtol 1e-8 (or SuperLU direct): the "
                     "physics fields measured bit-identical across re-runs on one "
                     "machine, but the reduction order is not pinned across builds/"
                     "thread counts — compare core_digest, not receipt bytes."))
    finish(payload, job=job, tool="ace_fea", out=out)


if __name__ == "__main__":
    run_cli("ace_fea", main, install_hint=ACE_INSTALL_HINT)
