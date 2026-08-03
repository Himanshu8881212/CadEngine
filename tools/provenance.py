#!/usr/bin/env python3
"""provenance.py — the analysis-result contract for the LMCAD graduation pipeline.

Every number a solver hands to a human should arrive inside ONE envelope that
says (a) what geometry it was computed on, (b) which material data version, (c)
which analyzer at which version, (d) whether that analyzer is VALIDATED against
ground truth or merely demonstrated, and (e) a residual / convergence receipt —
never a bare scalar. This module builds that envelope and computes the
deterministic geometry content-hash it is keyed on.

Two things it is deliberately strict about:

  * DETERMINISM — the geometry hash is a function of geometry CONTENT only. A
    work-order program is canonicalised with sorted JSON keys; an exported STL
    is canonicalised order-independently (triangle emission order and
    vertex-cycle rotation do not change the hash). No wall-clock time is ever
    written into an envelope.

  * HONESTY of the geometry relation. Two reps of "the same" solid are NOT the
    same hash: an STL is a *tessellation* of a B-rep program, related by
    ``derived_from`` with a stated chord-tolerance error bound, not by
    ``equality``. Both relation TYPES are provided as data (``RELATION_TYPES``,
    ``equality_relation``, ``derived_from_relation``) so a caller can state the
    relation truthfully even before the tessellation bridge is wired to emit it
    automatically.

The envelope (schema ``lmcad.analysis.v1``)::

    {
      "schema": "lmcad.analysis.v1",
      "values": <the analyzer's receipt / result payload>,
      "residual_or_convergence": <structured convergence receipt, NOT a scalar>,
      "self_check": <{limit, expected, obtained, passed} | null>,
      "manifest_ref": "tools/manifests/<analyzer>.manifest.json" | null,
      "geometry_relation": <equality|derived_from relation | null>,
      "provenance": {
        "geometry_hash":     "program:sha256:<hex>" | "mesh:sha256:<hex>",
        "material_version":  "<caller-supplied version string>",
        "analyzer_name":     "ace_fea",
        "analyzer_version":  "1.0.0",
        "validation_status": "validated" | "demonstrated" | "cataloged"
                             | "synthesized_inloop" | "synthesized_unvalidated"
                             | "research"
      }
    }

CLI (read-only helpers)::

    python3 provenance.py hash --program job.json
    python3 provenance.py hash --stl part.stl
    python3 provenance.py check --envelope envelope.json   # structural + synthesis gate
"""
from __future__ import annotations

import argparse
import hashlib
import json
import struct
import sys
from pathlib import Path

SCHEMA_ID = "lmcad.analysis.v1"

# ---------------------------------------------------------------------------
# Validation-status vocabulary. The status a result CARRIES is exactly the tier
# of the analyzer that produced it (analyzer_registry.py owns the tier table),
# plus two statuses that only apply to on-the-fly synthesised analysis.
# ---------------------------------------------------------------------------
STATUS_VALIDATED = "validated"  # pinned to independent ground truth, error band documented
STATUS_DEMONSTRATED = "demonstrated"  # runs end-to-end with a self-check, not pinned
STATUS_CATALOGED = "cataloged"  # deterministic rules/arithmetic over cited tables
STATUS_SYNTHESIZED_INLOOP = "synthesized_inloop"  # synthesised, passed its inline self-check limit
STATUS_SYNTHESIZED_UNVALIDATED = "synthesized_unvalidated"  # synthesised, self-check absent/failed
STATUS_RESEARCH = "research"  # frontier tier — hard-gated, ship-nothing-unmarked

ALLOWED_STATUS = frozenset({
    STATUS_VALIDATED,
    STATUS_DEMONSTRATED,
    STATUS_CATALOGED,
    STATUS_SYNTHESIZED_INLOOP,
    STATUS_SYNTHESIZED_UNVALIDATED,
    STATUS_RESEARCH,
})

# The statuses a *synthesised* (on-the-fly) analysis is allowed to leave with.
# Note there is no path to STATUS_VALIDATED without a committed manifest+pin —
# that is the whole point of the fence.
SYNTHESIZED_STATUS = frozenset({STATUS_SYNTHESIZED_INLOOP, STATUS_SYNTHESIZED_UNVALIDATED})

# ---------------------------------------------------------------------------
# Geometry-relation TYPES (contract item 1: equality vs derived-from). These are
# data, not behaviour. `equality` = same content hash, same representation.
# `derived_from` = a bridge (e.g. B-rep -> STL tessellation) with a stated error
# bound; the two hashes intentionally differ.
# ---------------------------------------------------------------------------
RELATION_EQUALITY = "equality"
RELATION_DERIVED_FROM = "derived_from"
RELATION_TYPES = {
    RELATION_EQUALITY: (
        "The analysis geometry IS the referenced geometry: identical content "
        "hash, identical representation. No error bound."
    ),
    RELATION_DERIVED_FROM: (
        "The analysis geometry was DERIVED from another representation by a "
        "bridge (e.g. B-rep program -> STL tessellation, or STL -> voxel "
        "occupancy). The two hashes differ; the relation carries a stated "
        "error_bound describing the approximation."
    ),
}


def equality_relation(geometry_hash: str) -> dict:
    """The analysis ran on exactly the referenced geometry (same rep, same hash)."""
    return {"type": RELATION_EQUALITY, "geometry_hash": geometry_hash}


def derived_from_relation(
    source_geometry_hash: str,
    target_geometry_hash: str,
    error_bound: dict,
    note: str = "",
) -> dict:
    """The analysis geometry was derived from ``source`` by an approximating bridge.

    ``error_bound`` is an open dict, e.g.
    ``{"kind": "chord_tolerance", "value_mm": 0.05}`` for a tessellation, or
    ``{"kind": "voxel_occupancy", "voxel_mm": 1.0,
       "note": "strut < ~4 cells: approximate"}`` for a voxelisation.
    """
    return {
        "type": RELATION_DERIVED_FROM,
        "source_geometry_hash": source_geometry_hash,
        "target_geometry_hash": target_geometry_hash,
        "error_bound": dict(error_bound),
        "note": note,
    }


# ---------------------------------------------------------------------------
# Deterministic geometry content-hash.
# ---------------------------------------------------------------------------
def _canonical_program_bytes(program) -> bytes:
    """Canonical byte encoding of a work-order program.

    Object keys are SORTED (order-independent); list order is PRESERVED because
    op order in a program is semantic. UTF-8, no insignificant whitespace, so
    the encoding is a pure function of the program's content.
    """
    return json.dumps(
        program,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
    ).encode("utf-8")


def _canonical_stl_hash(stl_bytes: bytes) -> str:
    """Order-independent SHA-256 of a BINARY STL's geometry.

    Canonicalisation (so two exports of the same solid hash equal):
      * The 80-byte header and per-facet normals are DROPPED (headers carry
        slicer strings; normals are recomputable and often garbage).
      * Each triangle's three vertices are cyclically rotated to start at the
        lexicographically smallest vertex — this removes vertex-listing-order
        variation WITHOUT flipping winding (orientation is preserved).
      * The triangle list is SORTED, so facet emission order is irrelevant.
    The result is the SHA-256 of the packed little-endian float32 stream of the
    canonicalised triangles. Pure stdlib (struct), no numpy.
    """
    if len(stl_bytes) < 84:
        raise ValueError(f"too small to be a binary STL ({len(stl_bytes)} bytes)")
    (n,) = struct.unpack_from("<I", stl_bytes, 80)
    expected = 84 + 50 * n
    if len(stl_bytes) != expected:
        raise ValueError(
            f"not a well-formed binary STL: header says {n} triangles "
            f"(needs {expected} bytes) but the buffer is {len(stl_bytes)} bytes "
            f"— ASCII STLs are not supported"
        )
    tris = []
    off = 84
    for _ in range(n):
        # 12 float32: normal(3) + v0(3) + v1(3) + v2(3); we keep only the verts.
        vals = struct.unpack_from("<12f", stl_bytes, off)
        off += 50  # 48 bytes floats + 2 bytes attribute count
        v0, v1, v2 = vals[3:6], vals[6:9], vals[9:12]
        cyc = [(v0, v1, v2), (v1, v2, v0), (v2, v0, v1)]
        tris.append(min(cyc))
    tris.sort()
    h = hashlib.sha256()
    for tri in tris:
        for vert in tri:
            h.update(struct.pack("<3f", *vert))
    return h.hexdigest()


def geometry_hash(*, program=None, stl_path=None, stl_bytes=None, density_path=None) -> str:
    """Deterministic content-hash of a geometry, keyed by representation.

    Provide EXACTLY ONE of:
      * ``program``     — a work-order program (dict) -> ``"program:sha256:<hex>"``
      * ``stl_path``    — path to a binary STL      -> ``"mesh:sha256:<hex>"``
      * ``stl_bytes``   — raw binary-STL bytes       -> ``"mesh:sha256:<hex>"``
      * ``density_path``— path to a voxel density ``.npy`` -> ``"density:sha256:<hex>"``

    The rep prefix is part of the identity: a program hash, a mesh hash, and a
    density-grid hash are never comparable by ``==`` even for "the same" solid —
    relate them with ``derived_from_relation`` (each bridge carries an error
    bound; a density grid is derived from a program/mesh with a voxel bound).
    """
    given = [name for name, val in
             (("program", program), ("stl_path", stl_path),
              ("stl_bytes", stl_bytes), ("density_path", density_path))
             if val is not None]
    if len(given) != 1:
        raise ValueError(
            "geometry_hash needs exactly one of "
            f"program/stl_path/stl_bytes/density_path; got {given}"
        )
    if program is not None:
        digest = hashlib.sha256(_canonical_program_bytes(program)).hexdigest()
        return f"program:sha256:{digest}"
    if density_path is not None:
        # Raw content hash of the .npy file — the numpy on-disk format is
        # deterministic for a given array+dtype, so this is a stable identity
        # for the exact voxel occupancy grid that was analysed.
        digest = hashlib.sha256(Path(density_path).read_bytes()).hexdigest()
        return f"density:sha256:{digest}"
    data = Path(stl_path).read_bytes() if stl_path is not None else stl_bytes
    return f"mesh:sha256:{_canonical_stl_hash(data)}"


def material_db_version(path: str) -> str:
    """Deterministic version tag for tools/material_db.json (a decoupled default).

    Content hash of the DB with sorted keys. When the parallel-owned
    ``tools/materials.py`` lands an authoritative version symbol, callers should
    pass THAT string to ``stamp`` instead — this helper only exists so the
    contract is demonstrable today without importing a file this agent does not
    own.
    """
    db = json.loads(Path(path).read_text(encoding="utf-8"))
    digest = hashlib.sha256(_canonical_program_bytes(db)).hexdigest()
    return f"matdb:sha256:{digest[:16]}"


# ---------------------------------------------------------------------------
# The envelope.
# ---------------------------------------------------------------------------
def stamp(
    values,
    *,
    geometry_hash: str,
    material_version: str,
    analyzer_name: str,
    analyzer_version: str,
    validation_status: str,
    residual_or_convergence,
    manifest_ref: str | None = None,
    self_check: dict | None = None,
    geometry_relation: dict | None = None,
) -> dict:
    """Wrap an analyzer result in the ``lmcad.analysis.v1`` envelope.

    ``residual_or_convergence`` must be a structured receipt (dict/list), never
    a bare scalar — a lone number with no convergence context is exactly what
    this contract exists to forbid.
    """
    if validation_status not in ALLOWED_STATUS:
        raise ValueError(
            f"unknown validation_status {validation_status!r}; "
            f"allowed: {sorted(ALLOWED_STATUS)}"
        )
    if isinstance(residual_or_convergence, (int, float, str)) or residual_or_convergence is None:
        raise ValueError(
            "residual_or_convergence must be a structured receipt "
            "(dict/list), not a bare scalar/None — the contract forbids "
            "reporting a number with no convergence context"
        )
    return {
        "schema": SCHEMA_ID,
        "values": values,
        "residual_or_convergence": residual_or_convergence,
        "self_check": self_check,
        "manifest_ref": manifest_ref,
        "geometry_relation": geometry_relation,
        "provenance": {
            "geometry_hash": geometry_hash,
            "material_version": material_version,
            "analyzer_name": analyzer_name,
            "analyzer_version": analyzer_version,
            "validation_status": validation_status,
        },
    }


def stamp_result(result, geometry, material_version, analyzer, status, **kw) -> dict:
    """Convenience wrapper matching the plan's stated signature
    ``stamp(result, geometry, material_version, analyzer, status)``.

    ``geometry`` may be a work-order program (dict), a path to a binary STL
    (str/Path), or a precomputed ``"...:sha256:..."`` hash string. ``analyzer``
    may be a ``"name"`` or a ``("name", "version")`` pair. Remaining keyword
    args (``residual_or_convergence``, ``manifest_ref``, ``self_check``,
    ``geometry_relation``) pass through to :func:`stamp`.
    """
    if isinstance(geometry, dict):
        ghash = geometry_hash(program=geometry)
    elif isinstance(geometry, (str, Path)):
        s = str(geometry)
        ghash = s if ":sha256:" in s else geometry_hash(stl_path=s)
    else:
        raise TypeError(f"unsupported geometry type: {type(geometry).__name__}")
    if isinstance(analyzer, (tuple, list)):
        name, version = analyzer
    else:
        name, version = analyzer, kw.pop("analyzer_version", "0.0.0")
    residual = kw.pop("residual_or_convergence", None)
    if residual is None:
        # An honest placeholder that check_synthesized() will REJECT — a
        # synthesised result is required to supply a real convergence receipt.
        residual = {"reported": False,
                    "note": "analyzer did not supply a residual/convergence receipt"}
    return stamp(
        result,
        geometry_hash=ghash,
        material_version=material_version,
        analyzer_name=name,
        analyzer_version=version,
        validation_status=status,
        residual_or_convergence=residual,
        manifest_ref=kw.pop("manifest_ref", None),
        self_check=kw.pop("self_check", None),
        geometry_relation=kw.pop("geometry_relation", None),
    )


# ---------------------------------------------------------------------------
# Checkers.
# ---------------------------------------------------------------------------
def check_envelope(envelope: dict) -> tuple[bool, list[str]]:
    """Structural check that a dict is a well-formed analysis envelope."""
    problems: list[str] = []
    if not isinstance(envelope, dict):
        return False, ["envelope is not a JSON object"]
    if envelope.get("schema") != SCHEMA_ID:
        problems.append(f"schema != {SCHEMA_ID!r} (got {envelope.get('schema')!r})")
    if "values" not in envelope:
        problems.append("missing 'values'")
    rc = envelope.get("residual_or_convergence")
    if rc is None:
        problems.append("missing 'residual_or_convergence'")
    elif isinstance(rc, (int, float, str)):
        problems.append("'residual_or_convergence' is a bare scalar — must be structured")
    prov = envelope.get("provenance")
    if not isinstance(prov, dict):
        problems.append("missing/invalid 'provenance' object")
    else:
        for key in ("geometry_hash", "material_version", "analyzer_name",
                    "analyzer_version", "validation_status"):
            if not prov.get(key):
                problems.append(f"provenance.{key} missing/empty")
        status = prov.get("validation_status")
        if status is not None and status not in ALLOWED_STATUS:
            problems.append(f"provenance.validation_status {status!r} not in {sorted(ALLOWED_STATUS)}")
    return (not problems), problems


def check_synthesized(envelope: dict) -> tuple[bool, list[str]]:
    """Synthesis guardrail (contract item 5).

    A result produced by ON-THE-FLY (synthesised) analysis may only surface to a
    user if it carries, in the envelope: a manifest reference, a self-check
    against a known limit that PASSED, a structured residual/convergence
    receipt, a geometry hash, and a synthesised validation_status. Anything
    missing means the number is confident-but-unvalidated and must NOT surface
    unmarked. Returns ``(ok, problems)``; ``ok`` is True only when the result is
    safe to surface.
    """
    ok, problems = check_envelope(envelope)
    prov = envelope.get("provenance", {}) if isinstance(envelope, dict) else {}
    status = prov.get("validation_status")

    if status not in SYNTHESIZED_STATUS:
        problems.append(
            f"synthesised result must carry a synthesised status "
            f"{sorted(SYNTHESIZED_STATUS)}, not {status!r} — a synthesised "
            f"analysis can NEVER claim 'validated' without a committed manifest+pin"
        )
    if not envelope.get("manifest_ref"):
        problems.append("synthesised result must emit a manifest_ref (equations+assumptions) BEFORE reporting")
    sc = envelope.get("self_check")
    if not isinstance(sc, dict):
        problems.append("synthesised result must carry a 'self_check' object (a known-limit check)")
    elif sc.get("passed") is not True:
        problems.append(f"synthesised result self_check did not pass: {sc!r}")
    rc = envelope.get("residual_or_convergence")
    if isinstance(rc, dict) and rc.get("reported") is False:
        problems.append("synthesised result carries no real residual/convergence receipt")

    # Belt-and-braces: even a passing self-check on a self_check-less status is unsafe.
    if status == STATUS_SYNTHESIZED_UNVALIDATED:
        problems.append(
            "status is 'synthesized_unvalidated' — self-check absent or failed; "
            "DO NOT surface this number without a WARNING banner"
        )
    return (not problems), problems


# ---------------------------------------------------------------------------
# CLI.
# ---------------------------------------------------------------------------
def _main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="LMCAD analysis-result contract helpers.")
    sub = ap.add_subparsers(dest="cmd", required=True)

    h = sub.add_parser("hash", help="print a deterministic geometry hash")
    g = h.add_mutually_exclusive_group(required=True)
    g.add_argument("--program", help="path to a work-order program JSON")
    g.add_argument("--stl", help="path to a binary STL")

    c = sub.add_parser("check", help="structural + synthesis check of an envelope JSON")
    c.add_argument("--envelope", required=True)
    c.add_argument("--synthesized", action="store_true",
                   help="apply the stricter synthesis guardrail")

    args = ap.parse_args(argv)

    if args.cmd == "hash":
        if args.program:
            prog = json.loads(Path(args.program).read_text(encoding="utf-8"))
            print(geometry_hash(program=prog))
        else:
            print(geometry_hash(stl_path=args.stl))
        return 0

    if args.cmd == "check":
        env = json.loads(Path(args.envelope).read_text(encoding="utf-8"))
        ok, problems = (check_synthesized if args.synthesized else check_envelope)(env)
        if ok:
            print("OK: envelope well-formed" + (" and safe to surface" if args.synthesized else ""))
            return 0
        print("FAIL:")
        for p in problems:
            print(f"  - {p}")
        return 1

    return 2


if __name__ == "__main__":
    raise SystemExit(_main())
