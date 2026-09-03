#!/usr/bin/env python3
"""sweep_check.py — generic parameter-sweep interference check over the LMCAD engine.

ONE tool for both assembly-physics sweep classes:
  * insertion sweeps  — t = position along an insertion axis (shaft into bore, ...)
  * motion sweeps     — t = joint angle / pose parameter (cam vs follower, ...)

The template is a normal LMCAD work-order whose ops contain the string "$t"
wherever the sweep parameter goes (same substitution semantics as
param_optimize: any string equal to "$t" — including inside coordinate arrays —
is replaced by the numeric station value). At every station the program is run
through the engine (`kernel-api run` one-shot) and each watched `clearance` (or
`coincident_fit`) op's measures are collected.

WHAT A SWEEP CAN AND CANNOT PROVE — read this before quoting one
----------------------------------------------------------------
A sweep is a **free-motion proof**: it samples poses and reports that none of
them interfered. It is sampled, so it can only ever support "these stations are
clear". It can NEVER support a *must-NOT-fit* claim:
  * between two clear stations anything may happen (sampling);
  * a body that interferes at EVERY station produces no free/interfering
    transition at all, which is the documented blind spot in `campaign/friction/ENGINE.md`
    #27 — the tool sees a uniformly bad sweep and a uniformly uninformative one
    the same way.
Anything asserting that something does NOT fit belongs on the exact oracle
(`intersection` + `exact_volume`). Every receipt now carries this in
`sweep_semantics`, every watch carries `all_stations_interfering`, and a run in
which NO watch ever saw a clear station is REFUSED
(`error_kind: "refusal.no_free_station"`) rather than returned as a tidy `ok:false` that
reads like a proof of interference.

Usage:  python3 sweep_check.py <job.json> [--out PATH]
Persistence + exit codes: the shared contract in tools/_receipt.py; the summary
receipt lands at `<out_dir>/sweep_check_receipt.json` (or `--out`, or the job's
`receipt` path) alongside the per-watch CSV tables.

Job JSON (argv[1]):
{
  "template": {"ops": [ ... ops with "$t" placeholders ... ]},
  "t":        {"from": 29.5, "to": 2.5, "steps": 28},     # steps <= 200, inclusive ends
  "watch":    ["fit", ...],   # ids of `clearance` (or `coincident_fit`) ops in the template
  "out_dir":  "sweep_out",    # per-watch CSV station tables land here
  "program_dir": "..."        # optional: where each station's substituted program
                              # is written, and therefore the root its relative
                              # `import_step`/`load_part` paths resolve against.
                              # Defaults to out_dir, else the job file's own dir —
                              # never a system temp dir (gripper F4 / turgo F7 /
                              # rotor F11: a swept template could not load a STEP).
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
      "all_stations_interfering": bool,# every evaluated station interfered — the
                                       #   sweep observed no free motion at all
      "table_path": "<out_dir>/<id>_sweep.csv"
    }, ...
  },
  "failed_stations": [{"t": ..., "error": "..."}], # ALWAYS present (empty list when
                                       #   none): a sibling key that was absent on a
                                       #   failing run read as "nothing failed" (din_rail F3)
  "interfering_watches": ["<id>", ...],# top-level localisation of the failure
  "sweep_semantics": "..."             # what a sweep does and does not prove
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

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # tools/: the shared contracts + the layout map
import _layout  # noqa: E402
_layout.add_import_paths()  # tools/, tools/analyzers, tools/publish — sibling-style imports keep working after the 2026-09-02 move
import param_optimize  # call_engine + substitute — the one-shot engine pattern
import _receipt
from _receipt import Refusal

MAX_STEPS = 200

SWEEP_SEMANTICS = (
    "A sweep is a SAMPLED FREE-MOTION proof: ok:true means no evaluated station "
    "interfered. It cannot support a must-NOT-fit claim — between two clear "
    "stations anything may happen, and an interference present at EVERY station "
    "produces no transition for the sweep to see (campaign/friction/ENGINE.md #27). Use "
    "`intersection` + `exact_volume` as the exact oracle for any 'does not fit' "
    "assertion.")


def log(msg):
    print(msg, file=sys.stderr, flush=True)


def stations(spec):
    """Inclusive [from, to] linspace with `steps` stations (steps <= 200)."""
    if not isinstance(spec, dict):
        raise Refusal("missing_t", "job needs `t` {from, to, steps}")
    missing = [k for k in ("from", "to", "steps") if k not in spec]
    if missing:
        raise Refusal("missing_t", f"`t` needs {missing} — got keys {sorted(spec)}")
    t0, t1, n = float(spec["from"]), float(spec["to"]), int(spec["steps"])
    if not 1 <= n <= MAX_STEPS:
        raise Refusal("bad_steps", f"t.steps must be 1..{MAX_STEPS}, got {n}")
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
            raise Refusal("watch_not_a_clearance_op",
                          f"watch '{op_id}' is not a clearance/coincident_fit op — "
                          f"measures: {sorted(m)}")
    raise Refusal("watch_not_found", f"watch '{op_id}' not found in the template's ops")


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


def build(job, job_path=None):
    ts = stations(job.get("t"))
    if not job.get("watch"):
        raise Refusal("no_watch", "job needs a non-empty `watch` list of clearance op ids")
    watch = list(job["watch"])
    if "out_dir" not in job:
        raise Refusal("missing_out_dir", "job needs `out_dir` (the per-watch CSV tables land there)")
    out_dir = job["out_dir"]
    os.makedirs(out_dir, exist_ok=True)
    # Station programs go where the job says, NOT a system temp dir — that is the
    # root their relative import_step/load_part paths resolve against.
    pdir = param_optimize.station_dir(job, job_path)

    rows = {w: [] for w in watch}   # per watch: [(t, row)] over successful stations
    failed = []
    for i, t in enumerate(ts):
        program = param_optimize.substitute(json.loads(json.dumps(job["template"])), {"t": t})
        report = param_optimize.call_engine(program, program_dir=pdir)
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
        all_bad = bool(flags) and all(flags)
        receipts[w] = {
            "min_distance": min(dists)[0] if dists else None,
            "min_distance_t": min(dists)[1] if dists else None,
            "first_interfering_t": first_bad,
            "interfering_t_ranges": ranges(w_ts, flags),
            "stations": len(rows[w]),
            "all_stations_interfering": all_bad,
            "table_path": table_path,
        }

    out = {
        "ok": not any_interference and not failed,
        "stations": len(ts),
        "t_from": ts[0],
        "t_to": ts[-1],
        "watches": receipts,
        # ALWAYS a list. Absent-on-failure read as "nothing failed" (din_rail F3).
        "failed_stations": failed,
        "interfering_watches": [w for w in watch if receipts[w]["first_interfering_t"] is not None],
        "sweep_semantics": SWEEP_SEMANTICS,
    }
    # A sweep in which NO watch ever saw a clear station observed no free motion
    # at all, so it proved nothing — and the shape of that receipt (ok:false,
    # every station interfering) is exactly what a reader mistakes for a proof
    # of interference. Refuse it by name instead (campaign/friction/ENGINE.md #27).
    evaluated = [w for w in watch if receipts[w]["stations"] > 0]
    if evaluated and all(receipts[w]["all_stations_interfering"] for w in evaluated):
        out["error_kind"] = "refusal.no_free_station"
        out["error"] = (
            "no_free_station: every evaluated station of every watch interferes, so this "
            "sweep observed NO free motion and establishes nothing. A sweep proves free "
            "motion ONLY; it cannot support a 'does not fit' claim. Either the sweep range "
            "is wrong / the pose is wrong, or you want the exact oracle "
            "(`intersection` + `exact_volume`) instead.")
        log("REFUSED no_free_station: every station of every watch interferes — a sweep "
            "proves free motion only and cannot support a must-NOT-fit claim")
    return out


def main():
    job_path, _ = _receipt.parse_argv()
    job, out = _receipt.load_job()
    payload = build(job, job_path)
    _receipt.finish(payload, job=job, tool="sweep_check", out=out,
                    kind=payload.get("error_kind"), use_out_dir_default=True)


if __name__ == "__main__":
    _receipt.run_cli("sweep_check", main)
