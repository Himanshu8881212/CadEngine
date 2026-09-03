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
  * "tol": t means +/- t. {"plus","minus"} is asymmetric.
  * Worst case: every element simultaneously at its unfavorable extreme, taken
    about the TRUE nominal — `nominal_gap` is sum(dir_i * nominal_i) with no
    shift, and the band is [nominal_gap - sum(unfavourable-low), nominal_gap +
    sum(unfavourable-high)]. (Fixed 2026-08-08, ball F4: the RSS mid-shift used
    to be applied to the SHARED nominal, so an asymmetric element moved the
    worst-case band by (plus-minus)/2 — pessimistic low, OPTIMISTIC high.)
  * RSS convention (stated so the receipt is auditable): an asymmetric tol is
    first converted to the equivalent bilateral by the standard mid-shift
    (nominal shifted by dir*(plus-minus)/2, t_eq = (plus+minus)/2); each +/-t_eq
    is treated as a 3-sigma band => sigma_i = t_eq/3, rss_sigma_gap =
    sqrt(sum(sigma_i^2)), and pass_rss checks
    rss_nominal_gap +/- 3*rss_sigma_gap against `closes`. `rss_nominal_gap` is
    reported separately; it equals `nominal_gap` whenever every tol is
    symmetric (which is every job in the shipped portfolio).
  * `closes` may be ONE-SIDED: omit (or null) `min_required` / `max_allowed` to
    leave that side unbounded. The receipt records which sides were checked.

FIT mode — bore/shaft pair, the printer-variance fit check:
{ "fit": {"bore":  {"nominal": 8.2, "tol": 0.15},
          "shaft": {"nominal": 8.0, "tol": 0.15}} }
  clearance = bore - shaft; min_clearance = min_bore - max_shaft (extremes);
  interference_at_extremes flags min_clearance < 0. Omitted "tol" takes
  printer_tol_default (0.15 mm — typical FDM dimensional tolerance; verify
  against YOUR printer's calibration).

Usage:  python3 tolerance_stack.py <job.json> [--out PATH]
Persistence + exit codes: the shared contract in tools/_receipt.py — `--out`
wins and a job `receipt` key that disagrees is REFUSED (it used to clobber
shipped evidence: cleat F7 / singulator F14), `LMCAD_RECEIPT_DRY_RUN=1`
suppresses every on-disk write so a what-if probe on a copied job cannot mutate
the original's receipt, and the exit code AGREES with `ok`
(0 ok / 1 could-not-run / 2 ran-and-refused). Before 2026-08-08 an `ok:false`
verdict exited 0 while an internal KeyError exited 1 (gripper F9).

Receipts (LAST stdout line, logging to stderr): {"ok", ...} with
  chain: {nominal_gap, worst_min, worst_max, rss_nominal_gap, rss_sigma_gap,
          rss_min, rss_max, pass_worst, pass_rss,
          contributors (ranked by worst-case band contribution)}
  fit:   {nominal_clearance, min_clearance, max_clearance,
          interference_at_extremes, pass}
ok = every requested mode passes (chain: pass_worst AND pass_rss; fit: no
interference at extremes and inside `closes` when given).
"""
import json
import math

import os

import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # tools/: the shared contracts + the layout map
import _layout  # noqa: E402
_layout.add_import_paths()  # tools/, tools/analyzers, tools/publish — sibling-style imports keep working after the 2026-09-02 move
import _receipt  # noqa: E402
from _receipt import Refusal  # noqa: E402

DEFAULT_PRINTER_TOL = 0.15  # mm, typical well-tuned FDM — verify per printer


def log(msg):
    print(msg, file=sys.stderr, flush=True)


def norm_tol(tol, default, where):
    """-> (plus, minus), both >= 0. None -> printer default (bilateral).

    Every malformed spelling REFUSES with a named reason: a raw KeyError is not
    a receipt (din_rail F1)."""
    if tol is None:
        return float(default), float(default)
    if isinstance(tol, bool):
        raise Refusal("bad_tol", f"{where}: `tol` must be a number or {{plus, minus}}, got a bool")
    if isinstance(tol, (int, float)):
        t = float(tol)
        if t < 0.0:
            raise Refusal("bad_tol", f"{where}: `tol` must be >= 0, got {t}")
        return t, t
    if not isinstance(tol, dict):
        raise Refusal("bad_tol", f"{where}: `tol` must be a number or {{plus, minus}}, got {type(tol).__name__}")
    missing = [k for k in ("plus", "minus") if k not in tol]
    if missing:
        raise Refusal("bad_tol", f"{where}: asymmetric `tol` needs both `plus` and `minus`, missing {missing}")
    plus, minus = float(tol["plus"]), float(tol["minus"])
    if plus < 0.0 or minus < 0.0:
        raise Refusal("bad_tol", f"{where}: `tol.plus`/`tol.minus` are MAGNITUDES and must be "
                                 f">= 0, got plus={plus} minus={minus}")
    return plus, minus


def norm_closes(closes, where, required=True):
    """-> (lo, hi) as floats, either may be -inf/+inf for a one-sided limit.

    `closes` used to be indexed raw, so a one-sided stack (a minimum engagement
    with no meaningful upper bound) died on `KeyError: 'max_allowed'` and the
    persisted receipt held nothing but that string (din_rail F1). One-sided is
    now first class; a `closes` with NEITHER bound is refused."""
    if closes is None:
        if required:
            raise Refusal("missing_closes",
                          f"{where} needs `closes` {{min_required, max_allowed}} "
                          f"(either bound may be omitted for a one-sided limit)")
        return -math.inf, math.inf
    if not isinstance(closes, dict):
        raise Refusal("bad_closes", f"{where}: `closes` must be an object, got {type(closes).__name__}")
    unknown = sorted(set(closes) - {"min_required", "max_allowed"})
    if unknown:
        raise Refusal("bad_closes", f"{where}: unknown key(s) in `closes`: {unknown} "
                                    f"(expected min_required / max_allowed) — a typo must not "
                                    f"become an unbounded limit")
    lo = closes.get("min_required")
    hi = closes.get("max_allowed")
    if lo is None and hi is None:
        raise Refusal("empty_closes", f"{where}: `closes` sets neither `min_required` nor "
                                      f"`max_allowed` — nothing would be checked")
    lo = -math.inf if lo is None else float(lo)
    hi = math.inf if hi is None else float(hi)
    if lo > hi:
        raise Refusal("bad_closes", f"{where}: min_required {lo} > max_allowed {hi}")
    return lo, hi


def _closes_receipt(lo, hi):
    """JSON-safe echo of the (possibly one-sided) limits + which sides were checked."""
    return {"min_required": None if lo == -math.inf else lo,
            "max_allowed": None if hi == math.inf else hi,
            "sides_checked": [s for s, v in (("min_required", lo != -math.inf),
                                             ("max_allowed", hi != math.inf)) if v]}


def chain_receipt(chain, closes, printer_tol):
    if not isinstance(chain, list) or not chain:
        raise Refusal("empty_chain", "`chain` must be a non-empty list of elements")
    nominal_gap, worst_lo, worst_hi, var = 0.0, 0.0, 0.0, 0.0
    mid_shift = 0.0
    contributors = []
    for i, el in enumerate(chain):
        name = el.get("name", f"#{i}") if isinstance(el, dict) else f"#{i}"
        where = f"chain element '{name}'"
        if not isinstance(el, dict):
            raise Refusal("bad_element", f"{where}: must be an object")
        d = float(el.get("dir", 1))
        if d not in (1.0, -1.0):
            raise Refusal("bad_dir", f"{where}: dir must be +1 or -1, got {el.get('dir')!r}")
        if "nominal" not in el:
            raise Refusal("missing_nominal", f"{where}: needs a `nominal`")
        nom = float(el["nominal"])
        plus, minus = norm_tol(el.get("tol"), printer_tol, where)
        nominal_gap += d * nom
        # Worst case, about the TRUE nominal: dir=+1 -> gap gains up to +plus /
        # loses up to -minus; dir=-1 the roles swap. No mid-shift here — that
        # belongs to the RSS conversion alone (ball F4).
        worst_hi += plus if d > 0 else minus
        worst_lo += minus if d > 0 else plus
        # RSS on the equivalent bilateral (mid-shifted) tolerance, t_eq/3 = sigma.
        t_eq = (plus + minus) / 2.0
        mid_shift += d * (plus - minus) / 2.0
        var += (t_eq / 3.0) ** 2
        contributors.append({"name": el.get("name", "?"), "dir": d,
                             "tol_plus": plus, "tol_minus": minus,
                             "worst_contribution": max(plus, minus),
                             # share of the worst-case BAND WIDTH (= plus+minus);
                             # equals 2x worst_contribution for a symmetric tol,
                             # so the ranking is unchanged for symmetric stacks.
                             "band_contribution": plus + minus})
    worst_min = nominal_gap - worst_lo
    worst_max = nominal_gap + worst_hi
    rss_nominal = nominal_gap + mid_shift
    sigma = math.sqrt(var)
    rss_min = rss_nominal - 3.0 * sigma
    rss_max = rss_nominal + 3.0 * sigma
    total_worst = sum(c["worst_contribution"] for c in contributors) or 1.0
    total_band = sum(c["band_contribution"] for c in contributors) or 1.0
    for c in contributors:
        c["pct_of_worst"] = round(100.0 * c["worst_contribution"] / total_worst, 1)
        c["pct_of_band"] = round(100.0 * c["band_contribution"] / total_band, 1)
    contributors.sort(key=lambda c: -c["band_contribution"])
    lo, hi = norm_closes(closes, "chain mode")
    out = {
        "nominal_gap": round(nominal_gap, 9),
        "worst_min": round(worst_min, 9),
        "worst_max": round(worst_max, 9),
        "rss_nominal_gap": round(rss_nominal, 9),
        "rss_sigma_gap": round(sigma, 9),
        "rss_min": round(rss_min, 9),
        "rss_max": round(rss_max, 9),
        "rss_convention": "each +/-t treated as 3*sigma; band = nominal +/- 3*sqrt(sum((t/3)^2))",
        "closes": _closes_receipt(lo, hi),
        "pass_worst": worst_min >= lo and worst_max <= hi,
        "pass_rss": rss_min >= lo and rss_max <= hi,
        "contributors": contributors,
    }
    if abs(mid_shift) > 0.0:
        out["asymmetric_note"] = (
            f"{sum(1 for c in contributors if c['tol_plus'] != c['tol_minus'])} element(s) carry an "
            f"asymmetric tol. worst_* are about the TRUE nominal_gap; rss_* are about "
            f"rss_nominal_gap = nominal_gap + {round(mid_shift, 9)} (standard mid-shift). "
            f"Before 2026-08-08 the mid-shift was applied to BOTH, which widened worst_min "
            f"and narrowed worst_max by that amount.")
    return out


def fit_receipt(fit, closes, printer_tol):
    if not isinstance(fit, dict):
        raise Refusal("bad_fit", "`fit` must be an object {bore, shaft}")
    for side in ("bore", "shaft"):
        if not isinstance(fit.get(side), dict):
            raise Refusal("bad_fit", f"fit mode needs `{side}` {{nominal, tol}}")
        if "nominal" not in fit[side]:
            raise Refusal("missing_nominal", f"fit `{side}`: needs a `nominal`")
    b_nom = float(fit["bore"]["nominal"])
    s_nom = float(fit["shaft"]["nominal"])
    bp, bm = norm_tol(fit["bore"].get("tol"), printer_tol, "fit `bore`")
    sp, sm = norm_tol(fit["shaft"].get("tol"), printer_tol, "fit `shaft`")
    min_clear = (b_nom - bm) - (s_nom + sp)   # smallest bore meets biggest shaft
    max_clear = (b_nom + bp) - (s_nom - sm)   # biggest bore meets smallest shaft
    interference = min_clear < 0.0
    ok = not interference
    if closes:
        lo, hi = norm_closes(closes, "fit mode")
        ok = ok and min_clear >= lo and max_clear <= hi
    return {
        "nominal_clearance": round(b_nom - s_nom, 9),
        "min_clearance": round(min_clear, 9),
        "max_clearance": round(max_clear, 9),
        "extremes": {"max_shaft": round(s_nom + sp, 9), "min_bore": round(b_nom - bm, 9)},
        "interference_at_extremes": interference,
        "pass": ok,
    }


def build(job):
    printer_tol = float(job.get("printer_tol_default", DEFAULT_PRINTER_TOL))
    out, oks = {"ok": True, "printer_tol_default": printer_tol}, []
    if "chain" in job:
        if "closes" not in job:
            raise Refusal("missing_closes",
                          "chain mode needs `closes` {min_required, max_allowed} "
                          "(either bound may be omitted for a one-sided limit)")
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
        raise Refusal("empty_job", "job needs `chain` and/or `fit`")
    out["ok"] = all(oks)
    return out


def main():
    job, out = _receipt.load_job()
    _receipt.finish(build(job), job=job, tool="tolerance_stack", out=out,
                    use_out_dir_default=True)


if __name__ == "__main__":
    _receipt.run_cli("tolerance_stack", main)
