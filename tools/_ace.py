"""Shared harness for the ACE physics runners (fea / modal / buckling).

Every ace_*_runner.py is a standalone stdout-receipt script, but the boot block
(ACE on the path, the LMCAD kernel-api env default), the log/emit helpers, and
the geometry-loading contract are byte-identical across all three — they were
"kept in sync by hand" and had already drifted in comments. This is the ONE
source of truth. Importing it runs the boot side effects (ACE_ROOT onto sys.path,
LMCAD_KERNEL_API default), so a runner must `from _ace import ...` before it
touches `engine.*`. Receipt/job schemas are unchanged: this only de-duplicates.
`selector_receipts` is deliberately NOT here — fea's and buckling's versions
genuinely differ, so each keeps its own.
"""
from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
ACE_ROOT = os.environ.get("ACE_ROOT", os.path.expanduser("~/Work/ACE"))
sys.path.insert(0, ACE_ROOT)
os.environ.setdefault(
    "LMCAD_KERNEL_API", str(REPO_ROOT / "target" / "release" / "kernel-api")
)

ACE_INSTALL_HINT = (
    "ACE package not importable — install it into the interpreter named by "
    "ACE_PYTHON: `pip install -e ~/Work/ACE` (or set ACE_ROOT)."
)


def log(msg: str) -> None:
    print(msg, file=sys.stderr, flush=True)


def emit(payload: dict) -> None:
    print(json.dumps(payload), flush=True)


def load_geometry(job: dict, out_dir: Path):
    """Resolve the job's geometry block to a density grid.

    Returns (rho, origin_mm, voxel_mm, sample_seconds).
    """
    import numpy as np

    voxel = float(job["voxel_mm"])
    origin = tuple(float(v) for v in job.get("origin_mm", (0.0, 0.0, 0.0)))
    t0 = time.monotonic()
    if job.get("npy"):
        rho = np.load(job["npy"]).astype(np.float32)
        log(f"loaded density grid {rho.shape} from {job['npy']}")
    else:
        from engine.lmcad import sample_part

        shape = tuple(int(n) for n in job["shape"])
        rho = sample_part(
            job["ops"], job["solid"], origin, voxel, shape,
            out_dir / "solid_fraction.npy",
            supersample=int(job.get("supersample", 2)),
        )
        log(f"sampled LMCAD solid '{job['solid']}' onto grid {shape}")
    return rho, origin, voxel, time.monotonic() - t0


def build_region_kind(job: dict, shape, voxel: float, origin):
    """regions -> string region_kind grid; None (all-design) when absent."""
    regions = job.get("regions")
    if not regions:
        return None  # the solver skips the override — same as all-design
    from engine.lmcad import region_kind_from_regions

    return region_kind_from_regions(regions, shape, voxel, origin)


def resolve_material(job_material):
    """Point the ACE runners at the materials source of truth (Unit 3).
    Backward-compatible: a pasted {youngs_modulus_pa, poisson, density_kg_m3}
    dict passes through unchanged; a STRING is a material key resolved via
    tools/materials.py to the same record Rust reads for density — one source,
    no re-keyed constants. A range assertion in materials.py catches a kg/m^3 <->
    g/cm^3 mixup on the way in."""
    if isinstance(job_material, str):
        import materials
        return materials.get(job_material).fea_material()
    return job_material


# --- provenance envelope (shared by fea/modal/buckling; see tools/provenance.py) ---
# The three runners are pinned/Validated in the analyzer registry, so their
# results are stamped with the lmcad.analysis.v1 envelope: the content-hash of
# the geometry analysed, the material identity, and a STRUCTURED convergence
# receipt pulled from ACE's real output (never fabricated). Existing scalar
# fields are kept; the envelope is ADDED alongside.

def _hash_material(material: dict) -> str:
    """A stable identity for the pasted FEA material dict (E/nu/rho). Unit 3 will
    replace this with a real materials-record version; until then the exact
    material used is identified by the content hash of its dict."""
    canon = json.dumps(material, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return "material:sha256:" + __import__("hashlib").sha256(canon).hexdigest()[:16]


def geometry_hash_for_job(job: dict) -> str | None:
    """Content-hash of whatever geometry this job analysed: the work-order
    program (ops+solid) if present, else the voxel density .npy."""
    import provenance
    if job.get("ops"):
        return provenance.geometry_hash(program={"ops": job["ops"], "solid": job.get("solid")})
    if job.get("npy"):
        return provenance.geometry_hash(density_path=job["npy"])
    return None


def convergence_receipt(res: dict) -> dict:
    """An HONEST structured convergence receipt from ACE's actual return.
    ACE's static/modal solvers use scipy CG at rtol 1e-8 and RAISE on
    non-convergence, so a returned result provably meets that tolerance; the
    solver method + DOF count + the solver note are all real. Per-iteration
    count is not exposed by ACE's return — stated, not invented."""
    notes = res.get("notes", []) or []
    solver_notes = [n for n in notes if "solve" in n.lower() or "cg" in n.lower()]
    iterative = any(("iterative" in n.lower()) or ("cg" in n.lower()) for n in solver_notes)
    return {
        "solver": res.get("method", "unknown"),
        "converged": True,  # ACE raises RuntimeError on non-convergence
        "target_rtol": 1e-8 if iterative else None,  # CG tolerance met; None for direct solve
        "n_dof": res.get("n_dof"),
        "n_active_elements": res.get("n_active_elements"),
        "solver_notes": solver_notes,
        "residual_source": (
            "scipy.sparse.linalg.cg rtol=1e-8; ACE raises on non-convergence, so a "
            "returned solution provably meets rtol. Per-iteration count is not exposed "
            "by ACE's return (read-only) — reported honestly, not fabricated."
        ),
    }


def provenance_fields(job: dict, res: dict, *, analyzer_name: str,
                      analyzer_version: str, values: dict, manifest_ref: str) -> dict:
    """Fields to merge into a runner payload: the geometry hash, the structured
    convergence receipt, and the full lmcad.analysis.v1 envelope. Additive —
    the caller keeps all existing scalar fields (Rule 4)."""
    import provenance
    geom = geometry_hash_for_job(job)
    conv = convergence_receipt(res)
    envelope = provenance.stamp(
        values=values,
        geometry_hash=geom or "unknown:none",
        material_version=_hash_material(job.get("material", {})),
        analyzer_name=analyzer_name,
        analyzer_version=analyzer_version,
        validation_status=provenance.STATUS_VALIDATED,
        residual_or_convergence=conv,
        manifest_ref=manifest_ref,
    )
    return {"geometry_hash": geom, "residual_or_convergence": conv, "analysis_envelope": envelope}
