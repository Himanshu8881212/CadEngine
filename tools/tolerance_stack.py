#!/usr/bin/env python3
"""tolerance_stack.py — 1-D assembly tolerance stack-up (worst-case + RSS) and fit check.

Pure arithmetic, no engine calls. Two modes, either or both in one job:

CHAIN mode — a linear dimension chain that must close inside a functional gap:
{
  "chain": [
    {"name": "housing_depth", "nominal": 20.0, "tol": 0.2,  "dir":  1},
    {"name": "bearing_stack", "nominal": 12.0, "tol": 0.1,  "dir": -1},
    {"name": "spacer",        "nominal": 7.5,  "tol": {"plus": 0.15, "minus": 0.15}, "dir": -1}
  ],
  "closes": {"min_required": 0.02, "max_allowed": 1.0},
  "printer_tol_default": 0.15        # used when an element omits "tol" (FDM default)
}
  * "dir" (+1/-1, default +1) is the element's sign in the gap equation
    gap = sum(dir_i * nominal_i).
  * "tol": t means +/- t. {"plus","minus"} is asymmetric; for RSS it is converted
    to the equivalent bilateral (nominal shifted by dir*(plus-minus)/2,
    t_eq = (plus+minus)/2) — the standard mid-shift treatment.
  * Worst case: every element simultaneously at its unfavorable extreme.
  * RSS convention (stated so the receipt is auditable): each +/-t is treated as
    a 3-sigma band => sigma_i = t_i/3, rss_sigma_gap = sqrt(sum(sigma_i^2)), and
    pass_rss checks nominal_gap +/- 3*rss_sigma_gap against `closes`.

FIT mode — bore/shaft pair, the printer-variance fit check:
{ "fit": {"bore":  {"nominal": 8.2, "tol": 0.15},
          "shaft": {"nominal": 8.0, "tol": 0.15}} }
  clearance = bore - shaft; min_clearance = min_bore - max_shaft (extremes);
  interference_at_extremes flags min_clearance < 0. Omitted "tol" takes
  printer_tol_default (0.15 mm — typical FDM dimensional tolerance; verify
  against YOUR printer's calibration).

Usage:  python3 tolerance_stack.py <job.json>
Persistence: the receipt ALSO lands on disk via the shared `_receipt` rule
(job `receipt` path, else `<out_dir>/tolerance_stack_receipt.json`, else
stdout-only) — verification evidence must survive the pipe (audit 2026-07-16).

Receipts (LAST stdout line, logging to stderr): {"ok", ...} with
  chain: {nominal_gap, worst_min, worst_max, rss_sigma_gap, rss_min, rss_max,
          pass_worst, pass_rss, contributors (ranked by worst-case contribution)}
  fit:   {nominal_clearance, min_clearance, max_clearance,
          interference_at_extremes, pass}
ok = every requested mode passes (chain: pass_worst AND pass_rss; fit: no
interference at extremes and inside `closes` when given).
"""
import json
import math

import _receipt
import sys

DEFAULT_PRINTER_TOL = 0.15  # mm, typical well-tuned FDM — verify per printer


def log(msg):
    print(msg, file=sys.stderr, flush=True)


def norm_tol(tol, default):
    """-> (plus, minus), both >= 0. None -> printer default (bilateral)."""
    if tol is None:
        return float(default), float(default)
    if isinstance(tol, (int, float)):
        return float(tol), float(tol)
    return float(tol["plus"]), float(tol["minus"])


def chain_receipt(chain, closes, printer_tol):
    nominal_gap, worst_lo, worst_hi, var = 0.0, 0.0, 0.0, 0.0
    contributors = []
    for el in chain:
        d = float(el.get("dir", 1))
        if d not in (1.0, -1.0):
            raise ValueError(f"element '{el.get('name')}': dir must be +1 or -1")
        nom = float(el["nominal"])
        plus, minus = norm_tol(el.get("tol"), printer_tol)
        nominal_gap += d * nom
        # Worst case: dir=+1 -> gap gains up to +plus / loses up to -minus;
        # dir=-1 the roles swap.
        worst_hi += plus if d > 0 else minus
        worst_lo += minus if d > 0 else plus
        # RSS on the equivalent bilateral (mid-shifted) tolerance, t/3 = sigma.
        t_eq = (plus + minus) / 2.0
        nominal_gap += d * (plus - minus) / 2.0  # mid-shift of asymmetric tol
        var += (t_eq / 3.0) ** 2
        contributors.append({"name": el.get("name", "?"), "dir": d,
                             "tol_plus": plus, "tol_minus": minus,
                             "worst_contribution": max(plus, minus)})
    worst_min = nominal_gap - worst_lo
    worst_max = nominal_gap + worst_hi
    sigma = math.sqrt(var)
    rss_min = nominal_gap - 3.0 * sigma
    rss_max = nominal_gap + 3.0 * sigma
    total_worst = sum(c["worst_contribution"] for c in contributors) or 1.0
    for c in contributors:
        c["pct_of_worst"] = round(100.0 * c["worst_contribution"] / total_worst, 1)
    contributors.sort(key=lambda c: -c["worst_contribution"])
    lo, hi = float(closes["min_required"]), float(closes["max_allowed"])
    return {
        "nominal_gap": round(nominal_gap, 9),
        "worst_min": round(worst_min, 9),
        "worst_max": round(worst_max, 9),
        "rss_sigma_gap": round(sigma, 9),
        "rss_min": round(rss_min, 9),
        "rss_max": round(rss_max, 9),
        "rss_convention": "each +/-t treated as 3*sigma; band = nominal +/- 3*sqrt(sum((t/3)^2))",
        "closes": {"min_required": lo, "max_allowed": hi},
        "pass_worst": worst_min >= lo and worst_max <= hi,
        "pass_rss": rss_min >= lo and rss_max <= hi,
        "contributors": contributors,
    }


def fit_receipt(fit, closes, printer_tol):
    b_nom = float(fit["bore"]["nominal"])
    s_nom = float(fit["shaft"]["nominal"])
    bp, bm = norm_tol(fit["bore"].get("tol"), printer_tol)
    sp, sm = norm_tol(fit["shaft"].get("tol"), printer_tol)
    min_clear = (b_nom - bm) - (s_nom + sp)   # smallest bore meets biggest shaft
    max_clear = (b_nom + bp) - (s_nom - sm)   # biggest bore meets smallest shaft
    interference = min_clear < 0.0
    ok = not interference
    if closes:
        ok = ok and min_clear >= float(closes["min_required"]) \
            and max_clear <= float(closes["max_allowed"])
    return {
        "nominal_clearance": round(b_nom - s_nom, 9),
        "min_clearance": round(min_clear, 9),
        "max_clearance": round(max_clear, 9),
        "extremes": {"max_shaft": round(s_nom + sp, 9), "min_bore": round(b_nom - bm, 9)},
        "interference_at_extremes": interference,
        "pass": ok,
    }


def main():
    job = json.load(open(sys.argv[1]))
    printer_tol = float(job.get("printer_tol_default", DEFAULT_PRINTER_TOL))
    out, oks = {"ok": True, "printer_tol_default": printer_tol}, []
    if "chain" in job:
        if "closes" not in job:
            raise ValueError("chain mode needs `closes` {min_required, max_allowed}")
        c = chain_receipt(job["chain"], job["closes"], printer_tol)
        out["chain"] = c
        oks.append(c["pass_worst"] and c["pass_rss"])
        log(f"chain: nominal {c['nominal_gap']} worst [{c['worst_min']}, {c['worst_max']}] "
            f"rss3s [{c['rss_min']}, {c['rss_max']}] worst={c['pass_worst']} rss={c['pass_rss']}")
    if "fit" in job:
        f = fit_receipt(job["fit"], job.get("closes") if "chain" not in job else None, printer_tol)
        out["fit"] = f
        oks.append(f["pass"])
        log(f"fit: clearance [{f['min_clearance']}, {f['max_clearance']}] "
            f"interference_at_extremes={f['interference_at_extremes']}")
    if not oks:
        raise ValueError("job needs `chain` and/or `fit`")
    out["ok"] = all(oks)
    _receipt.emit(out, job, "tolerance_stack")


if __name__ == "__main__":
    if len(sys.argv) < 2 or sys.argv[1] in ("-h", "--help"):
        print(__doc__)
        sys.exit(0)
    try:
        main()
    except Exception as e:  # honest failure receipt — the JSON line is the contract
        try:
            job = json.load(open(sys.argv[1]))
        except Exception:
            job = {}
        _receipt.emit({"ok": False, "error": f"{type(e).__name__}: {e}"}, job, "tolerance_stack")
        sys.exit(1)
