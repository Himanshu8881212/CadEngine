#!/usr/bin/env python3
"""sweep_check.py — generic parameter-sweep interference check over the LMCAD engine.

ONE tool for both assembly-physics sweep classes:
  * insertion sweeps  — t = position along an insertion axis (shaft into bore, ...)
  * motion sweeps     — t = joint angle / pose parameter (cam vs follower, ...)

The template is a normal LMCAD work-order whose ops contain the string "$t"
wherever the sweep parameter goes (same substitution semantics as
param_optimize: any string equal to "$t" — including inside coordinate arrays —
is replaced by the numeric station value). At every station the program is run
through the engine (lmcad-mcp one-shot) and each watched `clearance` (or
`coincident_fit`) op's measures are collected.

Usage:  python3 sweep_check.py <job.json>
Persistence: the summary receipt also lands at `<out_dir>/sweep_check_receipt.json`
(or the job's `receipt` path) alongside the per-watch CSV tables.

Job JSON (argv[1]):
{
  "template": {"ops": [ ... ops with "$t" placeholders ... ]},
  "t":        {"from": 29.5, "to": 2.5, "steps": 28},     # steps <= 200, inclusive ends
  "watch":    ["fit", ...],   # ids of `clearance` (or `coincident_fit`) ops in the template
  "out_dir":  "sweep_out"     # per-watch CSV station tables land here
}

Receipts (LAST stdout line, all logging to stderr):
{
  "ok": bool,                # false iff any watch interferes anywhere OR any station failed
  "stations": N, "t_from": ..., "t_to": ...,
  "watches": {
    "<id>": {
      "min_distance": ...,             # smallest surface gap seen across all stations
      "min_distance_t": ...,           # the station where it happened
      "first_interfering_t": ...|null, # first station (in sweep order) with interference
      "interfering_t_ranges": [[a,b]], # contiguous station ranges that interfere
      "stations": N,
      "table_path": "<out_dir>/<id>_sweep.csv"
    }, ...
  },
  "failed_stations": [{"t": ..., "error": "..."}]   # only when a station's program fails
}

A `clearance` watch interferes at t iff its `interfering` measure is true; a
`coincident_fit` watch "interferes" iff `coincident_fit` is true (a press-fit /
coincident-surface hazard — flagged with the same first_interfering_t key so
one receipt shape covers both). Deterministic: the engine is run-to-run
deterministic (R5) and stations are visited in order.
"""
import csv
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import param_optimize  # call_engine + substitute — the one-shot engine pattern
import _receipt

MAX_STEPS = 200


def log(msg):
    print(msg, file=sys.stderr, flush=True)


def stations(spec):
    """Inclusive [from, to] linspace with `steps` stations (steps <= 200)."""
    t0, t1, n = float(spec["from"]), float(spec["to"]), int(spec["steps"])
    if not 1 <= n <= MAX_STEPS:
        raise ValueError(f"t.steps must be 1..{MAX_STEPS}, got {n}")
    if n == 1:
        return [t0]
    return [t0 + i * (t1 - t0) / (n - 1) for i in range(n)]


def watch_row(report, op_id):
    """One watched op's measures at one station -> normalized row dict."""
    for op in report["ops"]:
        if op["id"] == op_id:
            m = op.get("measures") or {}
            if "coincident_fit" in m and "interfering" not in m:
                # coincident_fit watch: the hazard flag IS the interference signal
                return {"distance": None, "interfering": bool(m["coincident_fit"]),
                        "overlap_volume": None, "hazard": bool(m["coincident_fit"])}
            if "interfering" in m:
                return {"distance": m.get("distance"),
                        "interfering": bool(m.get("interfering")),
                        "overlap_volume": m.get("overlap_volume"),
                        "hazard": bool(m.get("coincident_fit_hazard", False))}
            raise ValueError(
                f"watch '{op_id}' is not a clearance/coincident_fit op — measures: {sorted(m)}")
    raise ValueError(f"watch '{op_id}' not found in the template's ops")


def ranges(ts, flags):
    """Contiguous [t_start, t_end] ranges (in sweep order) where flags are true."""
    out, start = [], None
    for t, f in zip(ts, flags):
        if f and start is None:
            start = t
        if not f and start is not None:
            out.append([start, prev])
            start = None
        prev = t
    if start is not None:
        out.append([start, prev])
    return out


def main():
    job = json.load(open(sys.argv[1]))
    ts = stations(job["t"])
    watch = list(job["watch"])
    out_dir = job["out_dir"]
    os.makedirs(out_dir, exist_ok=True)

    rows = {w: [] for w in watch}   # per watch: [(t, row)] over successful stations
    failed = []
    for i, t in enumerate(ts):
        program = param_optimize.substitute(json.loads(json.dumps(job["template"])), {"t": t})
        report = param_optimize.call_engine(program)
        if not report.get("ok"):
            errs = [o.get("error") for o in report.get("ops", []) if o.get("error")]
            failed.append({"t": t, "error": json.dumps(errs[:1])[:300]})
            log(f"station {i + 1}/{len(ts)} t={t:g}: PROGRAM FAILED {errs[:1]}")
            continue
        parts = []
        for w in watch:
            row = watch_row(report, w)
            rows[w].append((t, row))
            parts.append(f"{w}: d={row['distance'] if row['distance'] is not None else '-'}"
                         f" interf={row['interfering']}")
        log(f"station {i + 1}/{len(ts)} t={t:g}: " + "; ".join(parts))

    receipts, any_interference = {}, False
    for w in watch:
        table_path = os.path.join(out_dir, f"{w}_sweep.csv")
        with open(table_path, "w", newline="") as fh:
            cw = csv.writer(fh)
            cw.writerow(["t", "distance", "interfering", "overlap_volume", "coincident_fit_hazard"])
            for t, row in rows[w]:
                cw.writerow([t, row["distance"], row["interfering"],
                             row["overlap_volume"], row["hazard"]])
        dists = [(row["distance"], t) for t, row in rows[w] if row["distance"] is not None]
        flags = [row["interfering"] for _, row in rows[w]]
        w_ts = [t for t, _ in rows[w]]
        first_bad = next((t for t, f in zip(w_ts, flags) if f), None)
        if first_bad is not None:
            any_interference = True
        receipts[w] = {
            "min_distance": min(dists)[0] if dists else None,
            "min_distance_t": min(dists)[1] if dists else None,
            "first_interfering_t": first_bad,
            "interfering_t_ranges": ranges(w_ts, flags),
            "stations": len(rows[w]),
            "table_path": table_path,
        }

    _receipt.emit({
        "ok": not any_interference and not failed,
        "stations": len(ts),
        "t_from": ts[0],
        "t_to": ts[-1],
        "watches": receipts,
        **({"failed_stations": failed} if failed else {}),
    }, job, "sweep_check")


if __name__ == "__main__":
    if len(sys.argv) < 2 or sys.argv[1] in ("-h", "--help"):
        print(__doc__)
        sys.exit(0)
    try:
        main()
    except Exception as e:  # honest failure receipt — the JSON line is the contract
        try:
            _job = json.load(open(sys.argv[1]))
        except Exception:
            _job = {}
        _receipt.emit({"ok": False, "error": f"{type(e).__name__}: {e}"}, _job, "sweep_check")
        sys.exit(1)
