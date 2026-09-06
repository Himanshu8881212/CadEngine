#!/usr/bin/env python3
"""receipt_verify.py — does a receipt still describe the geometry it claims?

Every ACE receipt carries `geometry_hash`, the sha256 of the exact sampled
analysis domain (`ops`/`solid`/`voxel_mm`/`origin_mm`/`shape`/`supersample`,
or the density grid's bytes and placement — `tools/analyzers/_ace.py`
`geometry_hash_for_job`). It is the field an operator reaches for to answer
"was this receipt computed on the CURRENT geometry?" — and until this tool
nothing read it: no runner refused on a mismatch and no check compared
sibling receipts that share a geometry block, so a receipt that was never
re-run after an amendment shipped next to one that was
(`campaign/friction/iso9409_wedge_flexure_gripper.md` #F11: modal 29 544
elements vs buckling 30 054 on "the same" finger slice).

Usage (from anywhere; paths are taken as given):

    python3 tools/receipt_verify.py --pair receipts/modal.json programs/jobs/modal.json \\
                                    --pair receipts/buckling.json programs/jobs/buckling.json \\
                                    [--siblings receipts/modal.json receipts/buckling.json] \\
                                    [--out verify.json]

`--pair RECEIPT JOB` (repeatable): recompute the job's geometry hash and compare
it with `RECEIPT.geometry_hash` (also the `analysis_envelope.geometry_hash`
copy). `--siblings R1 R2 …` (repeatable): every listed receipt must carry ONE
geometry hash — the cross-check for receipts that claim the same domain.
A job may also be found from the receipt itself when it records `job_file` /
`job_path`, so `--receipt R` alone works for such receipts.

Exit 0 when every check holds, 1 on any mismatch (the report names each one),
2 when an input cannot be read or hashed (refusal, not a verdict). The report
goes to stdout as one JSON object (and to `--out` when given).
"""
from __future__ import annotations

import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "analyzers"))

VERSION = "1.0.0"


def _load(path):
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def _receipt_hash(rec):
    """The hash a receipt claims (top level first, then the envelope copy)."""
    h = rec.get("geometry_hash")
    env = rec.get("analysis_envelope") or {}
    eh = env.get("geometry_hash") if isinstance(env, dict) else None
    return h, eh


def _job_hash(job, job_path):
    from _ace import geometry_hash_for_job  # noqa: WPS433 — the runners' own hasher
    cwd = os.getcwd()
    try:
        # NPY jobs name their grid relative to the job file, as the runners resolve it.
        os.chdir(os.path.dirname(os.path.abspath(job_path)) or ".")
        return geometry_hash_for_job(job)
    finally:
        os.chdir(cwd)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--pair", nargs=2, action="append", metavar=("RECEIPT", "JOB"), default=[])
    ap.add_argument("--receipt", action="append", default=[],
                    help="a receipt that records its own job_file / job_path")
    ap.add_argument("--siblings", nargs="+", action="append", default=[],
                    help="receipts that must share ONE geometry hash")
    ap.add_argument("--out", help="also write the report here")
    args = ap.parse_args(argv)

    report = {"tool": "receipt_verify", "version": VERSION, "pairs": [], "siblings": [],
              "mismatches": 0, "ok": True}
    refusals = []

    pairs = list(args.pair)
    for rpath in args.receipt:
        try:
            rec = _load(rpath)
        except (OSError, ValueError) as exc:
            refusals.append(f"cannot read receipt {rpath}: {exc}")
            continue
        jp = rec.get("job_file") or rec.get("job_path") or (rec.get("job") if isinstance(rec.get("job"), str) else None)
        if not jp:
            refusals.append(f"{rpath} records no job_file/job_path — pass it with --pair")
            continue
        if not os.path.isabs(jp):
            jp = os.path.join(os.path.dirname(os.path.abspath(rpath)), jp)
        pairs.append([rpath, jp])

    for rpath, jpath in pairs:
        row = {"receipt": rpath, "job": jpath}
        try:
            rec = _load(rpath)
            job = _load(jpath)
        except (OSError, ValueError) as exc:
            refusals.append(f"cannot read {rpath} / {jpath}: {exc}")
            continue
        claimed, envelope = _receipt_hash(rec)
        try:
            actual = _job_hash(job, jpath)
        except Exception as exc:  # noqa: BLE001 — any hashing failure is a refusal
            refusals.append(f"cannot hash job {jpath}: {exc}")
            continue
        row.update({"claimed": claimed, "envelope_claimed": envelope, "recomputed": actual})
        if claimed is None:
            row["verdict"] = "no_hash_in_receipt"
            row["match"] = False
        else:
            row["match"] = (claimed == actual) and (envelope in (None, claimed))
            row["verdict"] = "match" if row["match"] else "STALE — the receipt was computed on different geometry than this job describes"
        if not row["match"]:
            report["mismatches"] += 1
        report["pairs"].append(row)

    for group in args.siblings:
        hashes = {}
        for rpath in group:
            try:
                h, _ = _receipt_hash(_load(rpath))
            except (OSError, ValueError) as exc:
                refusals.append(f"cannot read receipt {rpath}: {exc}")
                h = None
            hashes[rpath] = h
        distinct = {h for h in hashes.values() if h is not None}
        row = {"receipts": hashes, "distinct_hashes": sorted(distinct),
               "match": len(distinct) == 1 and None not in hashes.values()}
        if not row["match"]:
            report["mismatches"] += 1
            row["verdict"] = "the receipts do NOT share one geometry — at least one is stale or describes another domain"
        else:
            row["verdict"] = "match"
        report["siblings"].append(row)

    if refusals:
        report["ok"] = False
        report["error_kind"] = "refusal"
        report["error"] = "; ".join(refusals)
        code = 2
    elif report["mismatches"]:
        report["ok"] = False
        report["error_kind"] = "gate_failed"
        report["error"] = f"{report['mismatches']} geometry-hash check(s) failed"
        code = 1
    else:
        code = 0
    text = json.dumps(report, indent=1, sort_keys=True)
    if args.out:
        tmp = args.out + ".tmp"
        with open(tmp, "w", encoding="utf-8") as fh:
            fh.write(text + "\n")
        os.replace(tmp, args.out)
    print(text)
    return code


if __name__ == "__main__":
    sys.exit(main())
