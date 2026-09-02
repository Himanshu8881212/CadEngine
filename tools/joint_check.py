#!/usr/bin/env python3
"""joint_check.py — fastener/insert rules engine for printed-plastic assemblies.

The #1 assembly failure is the joint, not the part: heat-set inserts pulled out,
threads stripped out of plastic, creep under sustained load. This is a RULES
engine over published-typical capacity tables — no FEA, no engine calls.

Usage:  python3 joint_check.py <job.json> [--out PATH]
Persistence + exit codes: the shared contract in tools/_receipt.py.

**The exit code AGREES with the verdict.** Before 2026-08-08 it was INVERTED in
practice (ball F5): an out-of-table size leaked
`{"ok": false, "error": "KeyError: 'M6'"}` at exit 1 while a genuine
`min_engagement_rule` FAIL exited 0, so `$?` carried the opposite of the
engineering verdict. An unsupported size / material / joint type is now a
per-joint REFUSAL with a machine-matchable `error_kind` and no invented
capacity — the other joints in the job keep their numbers.

Job JSON (argv[1]):
{
  "joints": [{
    "name": "lid_screw",
    "type": "machine_screw_into_heatset" | "screw_into_plastic_thread" | "bolt_through_nut",
    "size": "M3" | "M4" | "M5",
    "material": "petg" | "pla" | "abs" | "asa" | "pc" | "nylon",   # the PLASTIC side
    "loads": {"tension_N": 100, "shear_N": 50, "sustained": false},
    "engagement_mm": 6.0,          # thread/insert engagement depth in the plastic
    "insert_len_mm": 5.7           # heat-set only; default = standard length for the size
  }, ...],
  "safety_factor": 2.0             # required SF (default 2.0)
}

Receipts (LAST stdout line, logging to stderr):
{ "ok": all_pass, "safety_factor_required": 2.0,
  "joints": [{ "name", "type", "governing_mode", "capacity_N", "demand_N",
               "SF_actual", "pass", "modes": {mode: {capacity_N, demand_N, SF}},
               "notes": [...] }] }

Rules applied (each with its capacity table below, sources in comments):
  * heat-set pull-out       — typical published values, scaled by insert length
  * plastic thread strip    — tau * pi * d * L_e * 0.5 (thread-form factor)
  * plastic shear/bearing   — 0.8 x pull-out (boss-geometry dependent, typical)
  * steel screw allowables  — ISO 898-1 class 8.8 (plastic side governs in practice)
  * min engagement          — >= 2*d in plastic threads; >= insert length for heat-set
  * sustained-load derating — x0.25 on ALL plastic-governed capacities (creep)
  * combined loading        — SF_combined = 1 / sqrt((T/Tc)^2 + (S/Sc)^2)

ALL plastic-side numbers are TYPICAL — VERIFY per insert brand / filament / print
settings before trusting a safety-critical joint. Printed parts vary with layer
adhesion, temperature, moisture and infill; the tables are conservative
mid-range values from the cited sources, not guarantees.
"""
import json

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _receipt  # noqa: E402
from _receipt import Refusal  # noqa: E402

# ---------------------------------------------------------------------------
# DATA TABLES — typical values, sources cited. VERIFY per brand before flight.
# ---------------------------------------------------------------------------

# Heat-set insert pull-out capacity (N) at the STANDARD insert length below,
# brass knurled inserts, properly installed (melted in, flush, no boss cracks).
# Sources: CNC Kitchen pull-out tests (S. Hermann, "Threaded Inserts in 3D
# Prints - How strong are they?", 2019 — M3 in PLA measured ~400..900 N across
# insert styles); Ruthex & E-Z Lok datasheet claims are HIGHER — these are the
# conservative low ends. typical — verify per insert brand.
HEATSET_PULLOUT_N = {
    #        pla   petg  abs   asa   pc    nylon
    "M3": {"pla": 400, "petg": 350, "abs": 300, "asa": 300, "pc": 450, "nylon": 350},
    "M4": {"pla": 650, "petg": 550, "abs": 480, "asa": 480, "pc": 700, "nylon": 550},
    "M5": {"pla": 900, "petg": 800, "abs": 650, "asa": 650, "pc": 950, "nylon": 800},
}

# Standard heat-set insert lengths (mm) per size (Ruthex/E-Z Lok standard series).
HEATSET_STD_LEN_MM = {"M3": 5.7, "M4": 8.1, "M5": 9.5}

# Pull-out scales ~linearly with embedded knurl area (= length); clamp the
# linear extrapolation to 0.5x..1.5x of the standard-length value — beyond
# that the boss, not the insert, governs. typical — verify.
HEATSET_LEN_SCALE_CLAMP = (0.5, 1.5)

# Plastic shear strength (MPa) for PRINTED parts, conservative: datasheet bulk
# shear derated for FDM layer adhesion (~60-80% of bulk; sources: Prusament /
# Ultimaker material datasheets for bulk tensile, tau ~ 0.6*tensile, CNC
# Kitchen layer-adhesion tests for the derate). typical — verify per filament.
PLASTIC_SHEAR_MPA = {
    "pla": 20.0, "petg": 18.0, "abs": 15.0, "asa": 15.0, "pc": 25.0, "nylon": 17.0,
}

# Thread-form engagement factor for screws threaded directly into plastic:
# only ~half the cylinder pi*d*L shears as thread material (flank engagement,
# thread-forming screws). Source: plastics joining design guides (e.g. BASF
# "Mechanical fastening of plastics", boss/thread design chapters). typical.
THREAD_FORM_FACTOR = 0.5

# Steel screw allowables, ISO 898-1 class 8.8 (common black-oxide cap screws;
# A2-70 stainless is within ~10%): tension = proof strength (580 MPa) x stress
# area; shear = 0.6 x tension. Stress areas: M3 5.03, M4 8.78, M5 14.2 mm^2.
SCREW_TENSION_N = {"M3": 2900, "M4": 5090, "M5": 8230}
SCREW_SHEAR_N = {k: round(v * 0.6) for k, v in SCREW_TENSION_N.items()}

# Plastic bearing/shear capacity around a heat-set insert as a fraction of
# pull-out — boss-geometry dependent (wall >= 2 mm around the insert assumed).
# typical — verify with the real boss.
HEATSET_SHEAR_FRACTION = 0.8

# Sustained-load derating for ALL plastic-governed capacities: thermoplastic
# creep under constant load (source: long-term stress-rupture curves in
# material datasheets / BASF design guide — sustained allowable ~ 1/4 of
# short-term). Steel modes are NOT derated.
SUSTAINED_DERATE = 0.25

NOMINAL_D_MM = {"M3": 3.0, "M4": 4.0, "M5": 5.0}
MIN_PLASTIC_THREAD_ENGAGE_FACTOR = 2.0  # engagement >= 2*d in plastic threads


def log(msg):
    print(msg, file=sys.stderr, flush=True)


JOINT_TYPES = ("machine_screw_into_heatset", "screw_into_plastic_thread", "bolt_through_nut")


def check_joint(j, sf_required):
    name = j.get("name", "?") if isinstance(j, dict) else "?"
    if not isinstance(j, dict):
        raise Refusal("bad_joint", f"joint '{name}': must be an object")
    for k in ("type", "size"):
        if k not in j:
            raise Refusal(f"missing_{k}", f"joint '{name}': needs a `{k}`")
    jtype, size = j["type"], j["size"]
    material = j.get("material", "petg").lower()
    # Refuse out-of-table inputs by NAME. These used to leak a raw
    # `KeyError: 'M6'` as the whole receipt (ball F5): the tables are the
    # tool's entire authority, so an entry that is not in them has no
    # conservative default — the only honest answer is a refusal.
    if jtype not in JOINT_TYPES:
        raise Refusal("unknown_joint_type",
                      f"joint '{name}': unknown type '{jtype}' — supported: {list(JOINT_TYPES)}")
    if size not in NOMINAL_D_MM:
        raise Refusal("size_not_in_table",
                      f"joint '{name}': size '{size}' is not in the capacity tables — "
                      f"supported: {sorted(NOMINAL_D_MM)}. No capacity is published for it here "
                      f"and extrapolating one would be an invented number.")
    if jtype != "bolt_through_nut" and material not in PLASTIC_SHEAR_MPA:
        raise Refusal("material_not_in_table",
                      f"joint '{name}': plastic '{material}' is not in the capacity tables — "
                      f"supported: {sorted(PLASTIC_SHEAR_MPA)}")
    loads = j.get("loads", {})
    if not isinstance(loads, dict):
        raise Refusal("invalid_load", f"joint '{name}': `loads` must be an object")
    tension = float(loads.get("tension_N", 0.0))
    shear = float(loads.get("shear_N", 0.0))
    if not (math.isfinite(tension) and math.isfinite(shear)) or tension < 0.0 or shear < 0.0:
        raise Refusal(
            "invalid_load",
            f"joint '{name}': tension_N and shear_N are non-negative finite load magnitudes; "
            f"got tension_N={tension!r}, shear_N={shear!r}. Encode direction in the load case, "
            "not by giving a capacity rule a negative demand.")
    sustained = bool(loads.get("sustained", False))
    d = NOMINAL_D_MM[size]
    notes = []
    modes = {}   # mode -> (capacity_N, demand_N)
    derate = SUSTAINED_DERATE if sustained else 1.0
    if sustained:
        notes.append(f"sustained load: plastic capacities derated x{SUSTAINED_DERATE} (creep)")

    engagement = j.get("engagement_mm")
    engagement_ok, engagement_note = True, None
    tension_modes, shear_modes = [], []   # which modes resist which load axis

    if jtype == "machine_screw_into_heatset":
        insert_len = float(j.get("insert_len_mm", HEATSET_STD_LEN_MM[size]))
        if not math.isfinite(insert_len) or insert_len <= 0.0:
            raise Refusal("invalid_geometry", f"joint '{name}': insert_len_mm must be positive and finite")
        base = HEATSET_PULLOUT_N[size][material]
        scale = min(max(insert_len / HEATSET_STD_LEN_MM[size], HEATSET_LEN_SCALE_CLAMP[0]),
                    HEATSET_LEN_SCALE_CLAMP[1])
        pullout = base * scale * derate
        modes["heatset_pullout"] = (pullout, tension)
        modes["plastic_bearing_shear"] = (pullout * HEATSET_SHEAR_FRACTION, shear)
        modes["screw_tension_steel"] = (SCREW_TENSION_N[size], tension)
        modes["screw_shear_steel"] = (SCREW_SHEAR_N[size], shear)
        tension_modes = ["heatset_pullout", "screw_tension_steel"]
        shear_modes = ["plastic_bearing_shear", "screw_shear_steel"]
        notes.append(f"pull-out table: typical {size} in {material} = {base} N at "
                     f"{HEATSET_STD_LEN_MM[size]} mm insert — verify per insert brand")
        if engagement is not None and float(engagement) < insert_len:
            engagement_ok = False
            engagement_note = (f"engagement {engagement} mm < insert length {insert_len} mm "
                               f"(heat-set rule: engagement >= insert length)")
    elif jtype == "screw_into_plastic_thread":
        L = float(engagement if engagement is not None else 0.0)
        if not math.isfinite(L) or L < 0.0:
            raise Refusal("invalid_geometry", f"joint '{name}': engagement_mm must be non-negative and finite")
        tau = PLASTIC_SHEAR_MPA[material]
        strip = tau * math.pi * d * L * THREAD_FORM_FACTOR * derate
        modes["plastic_thread_strip"] = (strip, tension)
        modes["plastic_bearing_shear"] = (strip * HEATSET_SHEAR_FRACTION, shear)
        modes["screw_tension_steel"] = (SCREW_TENSION_N[size], tension)
        modes["screw_shear_steel"] = (SCREW_SHEAR_N[size], shear)
        tension_modes = ["plastic_thread_strip", "screw_tension_steel"]
        shear_modes = ["plastic_bearing_shear", "screw_shear_steel"]
        notes.append(f"thread strip = tau*pi*d*Le*{THREAD_FORM_FACTOR} with tau({material}) = "
                     f"{tau} MPa printed — typical, verify per filament")
        min_L = MIN_PLASTIC_THREAD_ENGAGE_FACTOR * d
        if L < min_L:
            engagement_ok = False
            engagement_note = (f"engagement {L} mm < {min_L} mm "
                               f"(plastic-thread rule: >= {MIN_PLASTIC_THREAD_ENGAGE_FACTOR}*d)")
    elif jtype == "bolt_through_nut":
        modes["screw_tension_steel"] = (SCREW_TENSION_N[size], tension)
        modes["screw_shear_steel"] = (SCREW_SHEAR_N[size], shear)
        tension_modes = ["screw_tension_steel"]
        shear_modes = ["screw_shear_steel"]
        notes.append("steel-on-steel joint: plastic compression/creep under the head is NOT "
                     "modeled — use washers on printed faces")

    # Per-mode SF, plus the combined tension+shear interaction on the joint's
    # weakest tension/shear pair when both loads are present.
    sf_by_mode = {}
    for mode, (cap, dem) in modes.items():
        sf_by_mode[mode] = (cap / dem) if dem > 0 else math.inf
    if tension > 0 and shear > 0:
        tc = min(modes[m][0] for m in tension_modes)
        sc = min(modes[m][0] for m in shear_modes)
        u = math.sqrt((tension / tc) ** 2 + (shear / sc) ** 2)
        sf_by_mode["combined_tension_shear"] = 1.0 / u if u > 0 else math.inf
        modes["combined_tension_shear"] = (None, math.hypot(tension, shear))

    if not engagement_ok:
        governing = "min_engagement_rule"
        sf_actual, cap_g, dem_g, passed = 0.0, 0.0, max(tension, shear), False
        notes.append(engagement_note)
    else:
        governing = min(sf_by_mode, key=lambda m: sf_by_mode[m])
        sf_actual = sf_by_mode[governing]
        cap_g, dem_g = modes[governing]
        passed = sf_actual >= sf_required

    return {
        "name": j.get("name", "?"),
        "type": jtype,
        "size": size,
        "material": material,
        "governing_mode": governing,
        "capacity_N": round(cap_g, 2) if cap_g is not None else None,
        "demand_N": round(dem_g, 2),
        "SF_actual": round(sf_actual, 3) if math.isfinite(sf_actual) else None,
        "pass": passed,
        "modes": {m: {"capacity_N": round(c, 2) if c is not None else None,
                      "demand_N": round(dm, 2),
                      "SF": round(sf_by_mode[m], 3) if math.isfinite(sf_by_mode[m]) else None}
                  for m, (c, dm) in modes.items()},
        "notes": notes,
    }


def build(job):
    if not isinstance(job.get("joints"), list) or not job["joints"]:
        raise Refusal("no_joints", "job needs a non-empty `joints` list")
    sf_required = float(job.get("safety_factor", 2.0))
    receipts = []
    for j in job["joints"]:
        try:
            r = check_joint(j, sf_required)
        except Refusal as e:
            # Per-joint refusal: this joint gets NO capacity number and fails the
            # run, but the other joints keep their evidence (a whole-receipt
            # KeyError threw all of it away — din_rail F1's sibling defect).
            r = {"name": j.get("name", "?") if isinstance(j, dict) else "?",
                 "type": j.get("type") if isinstance(j, dict) else None,
                 "size": j.get("size") if isinstance(j, dict) else None,
                 "material": (j.get("material") if isinstance(j, dict) else None),
                 "governing_mode": None, "capacity_N": None, "demand_N": None,
                 "SF_actual": None, "pass": False,
                 "error_kind": e.kind, "error": f"{e.kind}: {e}",
                 "modes": {}, "notes": [f"REFUSED ({e.kind}): {e}"]}
            log(f"{r['name']}: REFUSED {e.kind}: {e}")
            receipts.append(r)
            continue
        log(f"{r['name']}: {r['governing_mode']} SF={r['SF_actual']} -> "
            f"{'PASS' if r['pass'] else 'FAIL'}")
        receipts.append(r)
    out = {
        "ok": all(r["pass"] for r in receipts),
        "safety_factor_required": sf_required,
        "data_caveat": "capacity tables are typical published values — verify per insert "
                       "brand / filament before trusting a safety-critical joint",
        "joints": receipts,
    }
    refused = [r["name"] for r in receipts if r.get("error_kind")]
    if refused:
        out["refused_joints"] = refused
    return out


def main():
    job, out = _receipt.load_job()
    payload = build(job)
    kind = "refusal.joint_refused" if payload.get("refused_joints") else None
    _receipt.finish(payload, job=job, tool="joint_check", out=out, kind=kind,
                    use_out_dir_default=True)


if __name__ == "__main__":
    _receipt.run_cli("joint_check", main)
