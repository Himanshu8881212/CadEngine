"""Shared harness for the physics runners (fea / modal / buckling).

Every ace_*_runner.py is a standalone stdout-receipt script, but the boot block
(the solver package on the path, the LMCAD kernel-api env default), the log/emit
helpers, and the geometry-loading contract are byte-identical across all three —
they were "kept in sync by hand" and had already drifted in comments. This is the
ONE source of truth. Importing it runs the boot side effects (tools/ and
tools/analyzers onto sys.path, LMCAD_KERNEL_API default), so a runner must
`from _ace import ...` before it touches `physics.*`. Receipt/job schemas are
unchanged: this only de-duplicates. `selector_receipts` is deliberately NOT here
— fea's and buckling's versions genuinely differ, so each keeps its own.

The solvers themselves live IN THIS REPO at `tools/analyzers/physics/` (moved
out of the ACE project on 2026-09-04; they stay Apache-2.0 — see that
directory's NOTICE and LICENSE-APACHE-2.0). There is no external checkout and
no `ACE_ROOT` any more: the pinned revision of the physics is this repository's
own history.
"""
from __future__ import annotations

import hashlib
import importlib
import importlib.metadata
import json
import os
import platform
import re
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]  # tools/analyzers/_ace.py -> repo root
PHYSICS_ROOT = Path(__file__).resolve().parent / "physics"  # the in-tree solver package
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # tools/analyzers: `import physics`
sys.path.insert(0, str(REPO_ROOT / "tools"))  # _receipt / provenance live at tools/ top level
os.environ.setdefault(
    "LMCAD_KERNEL_API", str(REPO_ROOT / "target" / "release" / "kernel-api")
)

import _receipt  # noqa: E402  — the shared receipt + exit-code contract
from _receipt import (  # noqa: E402,F401  — re-exported so runners import one module
    EXIT_ERROR,
    EXIT_OK,
    EXIT_REFUSED,
    Refusal,
    determinism_block,
    finish,
    load_job,
    run_cli,
)

PHYSICS_INSTALL_HINT = (
    "the in-tree solver package tools/analyzers/physics/ could not be imported — "
    "it needs numpy and scipy: `pip install --require-hashes -r "
    "tools/requirements-analysis.lock`. (Nothing external is required any more; "
    "the solvers moved into this repository on 2026-09-04.)"
)


def log(msg: str) -> None:
    print(msg, file=sys.stderr, flush=True)


def emit(payload: dict) -> None:
    """Legacy stdout-only emit. New code uses `finish()`, which ALSO persists
    the receipt and exits with the contracted code."""
    print(json.dumps(payload), flush=True)
    _receipt._CTX["emitted"] = True


def load_geometry(job: dict, out_dir: Path):
    """Resolve the job's geometry block to a density grid.

    Returns (rho, origin_mm, voxel_mm, sample_seconds).
    """
    runtime_provenance(job)  # release strictness is checked before sampling/solve/artifacts
    import numpy as np

    voxel = float(job["voxel_mm"])
    origin = tuple(float(v) for v in job.get("origin_mm", (0.0, 0.0, 0.0)))
    t0 = time.monotonic()
    if job.get("npy"):
        rho = np.load(job["npy"]).astype(np.float32)
        log(f"loaded density grid {rho.shape} from {job['npy']}")
    else:
        from physics.sampling import sample_part

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
    from physics.sampling import region_kind_from_regions

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
    """Hash the exact sampled analysis domain, including grid metadata.

    A work-order hash without voxel/origin/shape/supersample describes a CAD
    model, not the numerical grid actually solved. NPY jobs likewise include
    placement and pitch alongside the file bytes.
    """
    if job.get("ops"):
        descriptor = {
            "ops": job["ops"], "solid": job.get("solid"),
            "voxel_mm": job.get("voxel_mm"), "origin_mm": job.get("origin_mm", [0, 0, 0]),
            "shape": job.get("shape"), "supersample": job.get("supersample", 2),
        }
        canon = json.dumps(descriptor, sort_keys=True, separators=(",", ":"),
                           allow_nan=False).encode("utf-8")
        return "sampled-program:sha256:" + hashlib.sha256(canon).hexdigest()
    if job.get("npy"):
        h = hashlib.sha256()
        with open(job["npy"], "rb") as f:
            for chunk in iter(lambda: f.read(1024 * 1024), b""):
                h.update(chunk)
        metadata = {
            "voxel_mm": job.get("voxel_mm"), "origin_mm": job.get("origin_mm", [0, 0, 0]),
            "shape": job.get("shape"),
        }
        h.update(json.dumps(metadata, sort_keys=True, separators=(",", ":"),
                            allow_nan=False).encode("utf-8"))
        return "density-grid:sha256:" + h.hexdigest()
    return None


def _git_identity(path: str | Path) -> dict:
    path = str(path)
    out = {"path": path, "commit": None, "dirty": None, "available": False}
    try:
        sha = subprocess.run(["git", "-C", path, "rev-parse", "HEAD"],
                             capture_output=True, text=True, timeout=5)
        status = subprocess.run(["git", "-C", path, "status", "--porcelain"],
                                capture_output=True, text=True, timeout=5)
        if sha.returncode == 0 and status.returncode == 0:
            out.update(commit=sha.stdout.strip(), dirty=bool(status.stdout.strip()), available=True)
    except (OSError, subprocess.TimeoutExpired):
        pass
    return out


def runtime_provenance(job: dict) -> dict:
    """Exact solver/runtime identity.

    The solver used to live in a separate ACE checkout, so this pinned its
    commit against `tools/ACE_REVISION`. Since 2026-09-04 the physics is in
    THIS repository (`tools/analyzers/physics/`), so the solver's revision IS
    `lmcad.commit` and the external pin is retired — a clean LMCAD checkout is
    now the whole reproducibility claim, which is also what lets a hosted
    runner make it.
    """
    if isinstance(job.get("_runtime_provenance"), dict):
        return job["_runtime_provenance"]
    lmcad = _git_identity(REPO_ROOT)
    packages = {}
    package_sources = {}
    for name in ("numpy", "scipy", "gmsh", "matplotlib"):
        try:
            packages[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            packages[name] = None
        try:
            module = importlib.import_module(name)
            packages[name] = str(getattr(module, "__version__", packages[name]))
            package_sources[name] = str(getattr(module, "__file__", "")) or None
        except Exception:
            package_sources[name] = None
    lock = REPO_ROOT / "tools" / "requirements-analysis.lock"
    lock_hash = None
    if lock.exists():
        lock_hash = "sha256:" + hashlib.sha256(lock.read_bytes()).hexdigest()
    source_hashes = {}
    for path in (
        Path(__file__), REPO_ROOT / "tools" / "provenance.py",
        PHYSICS_ROOT / "fea.py",
        PHYSICS_ROOT / "fea_tet.py",
        PHYSICS_ROOT / "mesh_ir.py",
    ):
        if path.exists():
            source_hashes[str(path)] = "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()
    prefix = str(Path(sys.prefix).resolve())
    modules_in_prefix = all(
        path is not None and str(Path(path).resolve()).startswith(prefix + os.sep)
        for path in package_sources.values()
    )
    pythonpath = os.environ.get("PYTHONPATH", "").strip()
    reproducible = bool(
        lmcad.get("available") and not lmcad.get("dirty")
        and lock_hash and modules_in_prefix and not pythonpath
    )
    result = {
        "python": sys.version.split()[0], "python_executable": sys.executable,
        "python_implementation": platform.python_implementation(),
        "platform": platform.platform(), "packages": packages,
        "package_sources": package_sources, "python_prefix": prefix,
        "pythonpath_set": bool(pythonpath), "modules_in_python_prefix": modules_in_prefix,
        # `ace` / `expected_ace_commit` were removed on 2026-09-04 when the
        # solver moved in-tree: there is no second checkout to identify, and a
        # key that always said "not available" would be worse than none.
        # `physics` names where the solver now is; `lmcad.commit` IS its
        # revision, and `source_hashes` pins its exact bytes.
        "physics": {
            "package": str(PHYSICS_ROOT.relative_to(REPO_ROOT)),
            "in_tree": PHYSICS_ROOT.is_dir(),
            "license": "Apache-2.0 (see tools/analyzers/physics/NOTICE)",
        },
        "lmcad": lmcad,
        "dependency_lock": lock_hash, "source_hashes": source_hashes,
        "reproducible": reproducible,
    }
    strict = bool(job.get("require_reproducible_environment")) or os.environ.get(
        "LMCAD_REQUIRE_REPRODUCIBLE_ANALYSIS") == "1"
    if strict and not reproducible:
        raise Refusal(
            "non_reproducible_environment",
            "release-grade analysis requires a clean LMCAD checkout (which now "
            "carries the solver itself, tools/analyzers/physics/) plus "
            "tools/requirements-analysis.lock; this runtime does not satisfy that contract.",
            runtime_environment=result)
    job["_runtime_provenance"] = result
    return result


def convergence_receipt(res: dict) -> dict:
    """An HONEST structured convergence receipt from the solver's actual return.
    The static/modal solvers use scipy CG at rtol 1e-8 and RAISE on
    non-convergence, so a returned result provably meets that tolerance; the
    solver method + DOF count + the solver note are all real. Per-iteration
    count is not exposed by the solver's return — stated, not invented."""
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
                      analyzer_version: str, values: dict, manifest_ref: str,
                      geometry_hash: str | None = None,
                      validation_applicability: dict | None = None) -> dict:
    """Fields to merge into a runner payload: the geometry hash, the structured
    convergence receipt, and the full lmcad.analysis.v1 envelope. Additive —
    the caller keeps all existing scalar fields (Rule 4)."""
    import provenance
    geom = geometry_hash or geometry_hash_for_job(job)
    conv = convergence_receipt(res)
    runtime = runtime_provenance(job)
    status = provenance.STATUS_VALIDATED
    reasons = []
    if validation_applicability and validation_applicability.get("band_transfers") is False:
        status = provenance.STATUS_DEMONSTRATED
        reasons.append("job discretization lies outside the validated pin range")
    if not runtime["reproducible"]:
        status = provenance.STATUS_DEMONSTRATED
        reasons.append("solver/runtime checkout or dependency environment is not release-reproducible")
    envelope = provenance.stamp(
        values=values,
        geometry_hash=geom or "unknown:none",
        material_version=_hash_material(job.get("material", {})),
        analyzer_name=analyzer_name,
        analyzer_version=analyzer_version,
        validation_status=status,
        residual_or_convergence=conv,
        manifest_ref=manifest_ref,
    )
    envelope["validation_applicability"] = validation_applicability
    envelope["runtime_environment"] = runtime
    envelope["validation_status_reasons"] = reasons
    return {
        "geometry_hash": geom,
        "residual_or_convergence": conv,
        "analysis_envelope": envelope,
        "runtime_environment": runtime,
        "validation_status": status,
        "validation_status_reasons": reasons,
    }


# ===========================================================================
# T13 — PHYSICAL ADMISSIBILITY OF A JOB.
#
# The portfolio's own words: "silent acceptance — THE trap". Four campaigns
# were handed a confident number for a question the solver could not answer:
#   * a purely TENSILE load case returned a positive buckling factor plus a
#     knockdown block that reads exactly like a design margin (din_rail F7,
#     and — found while fixing it — the SHIPPED singulator buckling_neck
#     receipt, promoted into its README as "422x the design load");
#   * a `slider` fixture whose selector caught only INACTIVE voxels silently
#     degraded to "no fixture", and the run still said ok:true (rotor F6);
#   * an inclined wall 1.25 elements thick converged cleanly to 96 mm of
#     deflection under 27 N — 500x too soft, unbounded error, no warning
#     (horn F9);
#   * the analyzer manifest's `validation.direction` was read as an unqualified
#     property and did not transfer to a different geometry / a coarser grid
#     than the pin ever measured (rotor F14).
#
# The helpers below turn each of those into either a REFUSAL (when the request
# is provably unanswerable) or a machine-matchable WARNING on the receipt
# (when the answer exists but its error band does not). Nothing is a note in
# prose only — every one has a slug a gate can match.
# ===========================================================================

TENSILE_ALIGNMENT_THRESHOLD = 0.7  # cos 45.6 deg; see compression_check()


def element_centres_mm(idx, voxel: float, origin):
    """World centre of element (i,j,k): origin + (idx + 0.5) * voxel — the same
    convention ACE's selector engine resolves against."""
    import numpy as np
    return np.asarray(origin, dtype=float) + (np.asarray(idx, dtype=float) + 0.5) * voxel


def _selected_centroid(sel, occ, voxel, origin):
    import numpy as np
    from physics.selectors import resolve_selector
    mask = resolve_selector(sel, occ.shape, voxel, origin) & occ
    n = int(mask.sum())
    if n == 0:
        return None, 0
    idx = np.argwhere(mask)
    return element_centres_mm(idx, voxel, origin).mean(axis=0), n


def selector_catch_audit(job: dict, occ, voxel: float, origin) -> list[dict]:
    """Every fixture/load selector, and how many ACTIVE elements it catches.

    A selector catching ZERO active elements is a broken model, always: a
    fixture that constrains nothing is not a boundary condition, and ACE
    records it as a *note* while `ok` stays true (rotor F6 — a run that quietly
    lost 4 of its 6 boundary conditions returned a 0.036793 mm answer that
    looked converged). The precedent for auditing selectors already exists at
    the other end of the range — the "suspiciously broad" note at >30% — so
    this closes the zero end."""
    from physics.selectors import resolve_selector
    rows = []
    for group in ("fixtures", "loads"):
        for i, entry in enumerate(job.get(group, []) or []):
            sel = entry.get("region_selector", {"type": "all"})
            mask = resolve_selector(sel, occ.shape, voxel, origin) & occ
            rows.append({
                "group": group,
                "index": i,
                "kind": entry.get("kind"),
                "active_elements": int(mask.sum()),
            })
    return rows


def refuse_empty_selectors(audit: list[dict]) -> None:
    """Raise on any fixture/load selector that caught no ACTIVE element."""
    empty = [r for r in audit if r["active_elements"] == 0]
    if not empty:
        return
    where = ", ".join(f"{r['group']}[{r['index']}] ({r['kind']})" for r in empty)
    raise Refusal(
        "empty_selector",
        f"selector catches ZERO active elements: {where}. A fixture that "
        f"constrains nothing is not a boundary condition and a load applied to "
        f"nothing is not a load — the solve would return a plausible field for "
        f"a model missing its boundary conditions. Check the selector band "
        f"against origin_mm + (index + 0.5) * voxel_mm: a bbox that only covers "
        f"the half-voxel of air below a part sitting on z=0 catches the AIR "
        f"layer, not the part.",
        empty_selectors=empty)


def compression_check(job: dict, occ, voxel: float, origin) -> dict:
    """Is there a compressive load path at all? (the buckling precondition)

    Linear eigenvalue buckling looks for the multiplier at which a COMPRESSIVE
    pre-stress destabilises the elastic stiffness. ACE already refuses when
    there is no compressive principal stress ANYWHERE — but Poisson contraction
    against a clamp always manufactures a little transverse compression, so a
    pure tension bar sails through that guard and comes back with
    `buckling_load_factor` 18437 and `design_critical_load_n` 368747 N on a
    12x12x16 mm PLA prism pulled by 40 N. That number is not conservative, it
    is not approximate: it is about a mode that does not exist.

    The test here is the LOAD PATH, computed exactly and cheaply from the job
    itself: for each directional load, take the axis from the fixture centroid
    to that load's centroid and project the load direction onto it.

        alignment = d_hat . unit(c_load - c_fixture)

    alignment = -1  the load pushes the loaded region INTO its supports: a
                    compressive member exists, buckling is a real question.
    alignment = +1  the load pulls the loaded region AWAY from its supports:
                    the load path is a tie, not a strut. No bifurcation.
    alignment ~ 0   transverse / bending / lateral-torsional — buckling IS
                    possible (a beam buckles sideways under a transverse load),
                    so this must NOT refuse.

    We refuse only when EVERY directional load is unambiguously tensile
    (alignment >= TENSILE_ALIGNMENT_THRESHOLD = 0.7, i.e. within ~45 deg of
    "straight away from the supports"). Pressure loads carry no direction and
    make the verdict indeterminate rather than tensile — conservative on
    purpose. `allow_tensile_load_case: true` in the job overrides the refusal
    and RECORDS the override in the receipt; it never disappears."""
    import numpy as np

    fixtures = job.get("fixtures", []) or []
    loads = job.get("loads", []) or []
    result = {
        "method": ("alignment = load_direction . unit(load_centroid - fixture_centroid), "
                   "over ACTIVE elements; -1 = pure compression into the supports, "
                   "+1 = pure tension away from them"),
        "tensile_alignment_threshold": TENSILE_ALIGNMENT_THRESHOLD,
        "loads": [],
        "verdict": "indeterminate",
        "override": bool(job.get("allow_tensile_load_case", False)),
    }
    fmask_centroids = []
    for fx in fixtures:
        c, n = _selected_centroid(fx.get("region_selector", {"type": "all"}), occ, voxel, origin)
        if c is not None:
            fmask_centroids.append((c, n))
    if not fmask_centroids:
        result["note"] = "no fixture caught any active element; load path undecidable"
        return result
    w = np.array([n for _c, n in fmask_centroids], dtype=float)
    c_fix = (np.array([c for c, _n in fmask_centroids]) * w[:, None]).sum(0) / w.sum()
    result["fixture_centroid_mm"] = [round(float(v), 6) for v in c_fix]

    verdicts = []
    for i, ld in enumerate(loads):
        d = ld.get("direction")
        c_l, n = _selected_centroid(ld.get("region_selector", {"type": "all"}), occ, voxel, origin)
        row = {"index": i, "kind": ld.get("kind"), "active_elements": n}
        if c_l is None:
            row["verdict"] = "indeterminate"
            row["why"] = "selector caught no active element"
        elif not d:
            row["verdict"] = "indeterminate"
            row["why"] = f"{ld.get('kind')} load carries no direction vector"
        else:
            row["load_centroid_mm"] = [round(float(v), 6) for v in c_l]
            axis = np.asarray(c_l, dtype=float) - c_fix
            an = float(np.linalg.norm(axis))
            dv = np.asarray(d, dtype=float)
            dn = float(np.linalg.norm(dv))
            if an < 0.5 * voxel or dn == 0.0:
                row["verdict"] = "indeterminate"
                row["why"] = "load and fixture centroids coincide (no load-path axis)"
            else:
                a = float(np.dot(dv / dn, axis / an))
                row["alignment"] = round(a, 6)
                row["verdict"] = ("tensile" if a >= TENSILE_ALIGNMENT_THRESHOLD
                                  else "compressive" if a <= -TENSILE_ALIGNMENT_THRESHOLD
                                  else "transverse")
        verdicts.append(row["verdict"])
        result["loads"].append(row)

    if verdicts and all(v == "tensile" for v in verdicts):
        result["verdict"] = "tensile"
    elif any(v == "compressive" for v in verdicts):
        result["verdict"] = "compressive"
    elif verdicts:
        result["verdict"] = "transverse_or_indeterminate"
    return result


def refuse_tensile_load_case(check: dict) -> None:
    if check.get("verdict") != "tensile":
        return
    if check.get("override"):
        return
    rows = ", ".join(
        f"load[{r['index']}] alignment {r.get('alignment')}"
        for r in check["loads"] if "alignment" in r)
    raise Refusal(
        "no_compressive_load_path",
        f"every applied load pulls AWAY from the supports ({rows}; threshold "
        f"{check['tensile_alignment_threshold']}). Linear buckling is a "
        f"bifurcation of a COMPRESSIVE pre-stress; a tensile load path has no "
        f"bifurcation to find, and the positive eigenvalue such a solve returns "
        f"is a clamp-local Poisson artefact, not a design margin. Reverse the "
        f"reference load, analyse the member that is actually in compression, "
        f"or set \"allow_tensile_load_case\": true to force the solve (the "
        f"override is recorded in the receipt).",
        compression_check=check)


def mesh_resolution_receipt(occ, voxel_mm: float) -> dict:
    """How many ELEMENTS sit across the thin features of this mesh.

    ace_fea converged cleanly on an inclined wall 1.25 elements thick and
    returned 96 mm of deflection under 27 N — 500x too soft, six times PLA's
    yield, `converged: true`, no warning (horn F9). The staircase of a thin
    inclined wall is a hinge chain in hex8: connectivity checks read healthy
    (one 6-connected component, largest fraction 1.000), selectors read healthy,
    and the manifest's signed error band is simply not applicable — outside the
    resolution regime the deflection error is UNBOUNDED and of the OPPOSITE
    sign to the band.

    Measured here, deterministically, from the occupancy the solve itself uses:
    a Euclidean distance transform gives each active element its distance to
    the nearest void; one 3x3x3 max-dilation of that field promotes every
    element that touches a >=3-elements-thick core. `thin_element_fraction` is
    the share of the active mesh that does NOT touch such a core — i.e. the
    share of the model where hex8 bending stiffness is unresolved.

    The digests already state ">= 3 voxels across every wall/strut" as a rule
    for the IMPLICIT/heal exports; it matters just as much for the voxel FEA
    runners, and until now nothing measured it."""
    import numpy as np
    try:
        from scipy import ndimage
    except Exception as exc:  # noqa: BLE001 — never sink a good solve
        return {"available": False, "why": f"scipy.ndimage unavailable: {exc}"}
    active = np.asarray(occ, dtype=bool)
    n_active = int(active.sum())
    if n_active == 0:
        return {"available": False, "why": "no active elements"}
    padded = np.zeros(tuple(s + 2 for s in active.shape), dtype=bool)
    padded[1:-1, 1:-1, 1:-1] = active
    edt = ndimage.distance_transform_edt(padded)
    core = ndimage.grey_dilation(edt, size=3)[1:-1, 1:-1, 1:-1]
    edt_in = edt[1:-1, 1:-1, 1:-1]
    thin = active & (core < 2.0)
    thin_fraction = float(thin.sum()) / n_active
    return {
        "available": True,
        "n_active_elements": n_active,
        "voxel_mm": float(voxel_mm),
        "thickest_feature_elements": round(float(2.0 * edt_in[active].max() - 1.0), 3),
        "thin_element_fraction": round(thin_fraction, 6),
        "thin_threshold_elements": 3,
        "definition": ("thin = an active element with no >=3-element-thick core in its "
                       "3x3x3 neighbourhood (2*EDT-1 < 3); hex8 bending stiffness is "
                       "unresolved there and the manifest error band does not apply"),
        "resolved": bool(thin_fraction < 0.5),
    }


def mesh_resolution_warning(mesh: dict) -> dict | None:
    if not mesh.get("available") or mesh.get("resolved", True):
        return None
    return {
        "kind": "mesh.under_resolved_thin_features",
        "message": (
            f"{mesh['thin_element_fraction']:.1%} of the active mesh sits in features "
            f"under {mesh['thin_threshold_elements']} elements thick at voxel_mm "
            f"{mesh['voxel_mm']}. A hex8 staircase of a thin (especially inclined) wall "
            f"behaves as a kinematic hinge chain: it CONVERGES, connectivity and "
            f"selectors read healthy, and the displacement can be orders of magnitude "
            f"too soft — with the OPPOSITE sign to the analyzer's stated error band. "
            f"Refine until the thinnest load-bearing wall carries >= 3 elements, or "
            f"answer this load case with a sub-model / closed form. Set "
            f"\"strict_warnings\": true to make this a refusal."),
        "thin_element_fraction": mesh["thin_element_fraction"],
    }


_PIN_SIZE_RE = re.compile(r"(?:voxel|elem)_([0-9]*\.?[0-9]+)")


def validated_range_check(job: dict, manifest_ref: str) -> dict | None:
    """Does THIS job sit inside the discretization range the pin measured?

    `tools/manifests/*.manifest.json` states `validation.direction` as an
    unqualified property of the analyzer ("under-predicts peak/tip response;
    error converges from below"). Read as written it licenses deflating a
    deflection gate by the band — and on a real part it was FALSE for
    deflection: refinement 1.5 -> 1.0 mm LOWERED the tip deflection by 7.05%,
    i.e. it converges from ABOVE, while the stress caveat did transfer (rotor
    F14). The pin's specimen (an 8x8 mm cantilever at voxel 1.0/0.5, pure
    bending, 8 voxels across the section) and the part (34x44 mm at voxel 1.5,
    81% of its compliance rigid rotation of two foot plates) exercise error
    mechanisms with OPPOSITE signs, and the manifest exposes only one.

    The general defect is that a validated band was quoted for a point the pin
    never measured. So: report the pinned discretization sizes next to the
    job's, and say plainly when the job is an EXTRAPOLATION."""
    path = REPO_ROOT / manifest_ref
    try:
        m = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:  # noqa: BLE001
        return {"manifest_ref": manifest_ref, "available": False, "why": str(exc)}
    band = (m.get("validation") or {}).get("error_band") or {}
    pinned = sorted({float(mt.group(1)) for k in band
                     for mt in [_PIN_SIZE_RE.search(str(k))] if mt})
    size_key = "voxel_mm" if job.get("voxel_mm") is not None else "elem_size_mm"
    job_size = job.get(size_key)
    out = {
        "manifest_ref": manifest_ref,
        "available": True,
        "pinned_discretization_mm": pinned,
        "error_band": band,
        "direction": (m.get("validation") or {}).get("direction"),
        "job_voxel_mm": float(job["voxel_mm"]) if job.get("voxel_mm") is not None else None,
        "job_discretization_mm": float(job_size) if job_size is not None else None,
        "job_discretization_key": size_key,
        "band_transfers": None,
        "caveat": ("`validation.direction` and `error_band` were measured on the pin's "
                   "OWN specimen and discretization. They are properties of that "
                   "measurement, not unqualified properties of the analyzer: a part "
                   "whose compliance is dominated by a different mechanism (boundary "
                   "resolution of a thin plate rather than section bending) can move "
                   "the OPPOSITE way under refinement. Re-measure the direction on "
                   "your own geometry with a refinement pair before quoting it."),
    }
    if pinned and out["job_discretization_mm"] is not None:
        out["band_transfers"] = bool(
            min(pinned) <= out["job_discretization_mm"] <= max(pinned))
    return out


def validated_range_warning(vr: dict | None) -> dict | None:
    if not vr or not vr.get("available") or vr.get("band_transfers") is not False:
        return None
    pinned = vr["pinned_discretization_mm"]
    return {
        "kind": "manifest.outside_validated_discretization",
        "message": (
            f"this job runs at {vr['job_discretization_key']} "
            f"{vr['job_discretization_mm']}, OUTSIDE the range the "
            f"validation pin measured ({pinned[0]}..{pinned[-1]} mm). The manifest's "
            f"error_band {vr['error_band']} and its `direction` are an EXTRAPOLATION "
            f"here with no measured support — including its sign. Measure a refinement "
            f"pair on THIS geometry before quoting the band."),
        "job_voxel_mm": vr["job_voxel_mm"],
        "job_discretization_mm": vr["job_discretization_mm"],
        "job_discretization_key": vr["job_discretization_key"],
        "pinned_discretization_mm": pinned,
    }


def apply_warnings(payload: dict, job: dict, warnings: list) -> dict:
    """Attach machine-matchable warnings; `strict_warnings: true` promotes them
    to a refusal (opt-in, so today's semantics are the default)."""
    warnings = [w for w in warnings if w]
    payload["warnings"] = warnings
    payload["warning_kinds"] = [w["kind"] for w in warnings]
    if warnings and job.get("strict_warnings"):
        raise Refusal(
            "strict_warnings",
            "strict_warnings is set and the run produced "
            f"{len(warnings)} warning(s): "
            + "; ".join(f"[{w['kind']}] {w['message']}" for w in warnings),
            warnings=warnings)
    return payload
