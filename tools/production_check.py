#!/usr/bin/env python3
"""production_check.py — Layer-1 FDM production rules engine on FEA results.

Bridge runner spawned by the LMCAD MCP server (``lmcad-mcp`` tool
``production_check``): takes the peak von Mises stress of a prior ``ace_fea``
run plus the intended material / load character / service temperature /
print orientation, and grades the part against derated allowables from
``tools/material_db.json``. Pure stdlib — no ACE, no numpy.

Usage:  <PYTHON3> production_check.py <job.json>
        <PYTHON3> production_check.py --selftest    (the pinned PETG gate;
                                                     exits nonzero on failure)

Job JSON:
    material                REQUIRED  key into material_db.json — one of
                                      PLA|PETG|ABS|ASA|TPU95A|PC|PA
                                      (case-insensitive; TPU→TPU95A,
                                      NYLON→PA aliases accepted)
    max_von_mises_pa        REQUIRED  peak stress from a prior ace_fea (Pa)
    load_character          optional  {sustained: bool, cyclic: bool},
                                      default both false
    service_temp_c          optional  default 25
    orientation             optional  {build_dir: [x,y,z],
                                       primary_load_dir: [x,y,z]} — enables
                                      the anisotropy rule; absent => rule
                                      skipped WITH a note (never silently)
    safety_factor_required  optional  default 2.0

Rules (each receipt row shows every derating in its arithmetic):
    static      allowable = yield                       (always)
    creep       allowable = yield * creep_sustained_fraction   (sustained)
    fatigue     allowable = ultimate * fatigue_knockdown       (cyclic)
    temp        service_temp_c <= material service limit       (always;
                allowable_c/demand_c in C, SF = limit/service)
    anisotropy  when the primary load is inclined more than 30 deg out of
                the layer plane, ALL stress allowables above are further
                multiplied by layer_adhesion_factor, and an anisotropy row
                reports the across-layer static check explicitly.
                SCALAR-TIER heuristic on the load DIRECTION only; a
                tensor-based layer-normal stress check requires an ACE
                solver change.

Output contract: the LAST non-empty stdout line is ONE JSON object; logging
goes to stderr. ``ok`` is the OVERALL VERDICT — true iff every evaluated rule
passes (a structurally failing part answers ok:false WITH the full per-rule
receipt; a crashed run answers {ok:false, error}). Receipts: {ok, material,
safety_factor_required, anisotropy_derate_applied, rules: [{rule,
allowable_mpa, demand_mpa, SF, pass, detail}], skipped: [{rule, reason}],
notes, disclaimer}.

Honest caveats (echoed by the MCP tool description): allowables are TYPICAL
desktop-FDM datasheet values (verify per filament brand — the db says so);
creep/fatigue knockdowns are engineering rules of thumb, not measured filament
data; the anisotropy rule is scalar-tier (direction heuristic), not a
layer-normal stress tensor check; the demand number inherits ace_fea's ~20%
coarse-mesh under-prediction of peak bending stress.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

DB_PATH = Path(__file__).resolve().parent / "material_db.json"

ALIASES = {"TPU": "TPU95A", "NYLON": "PA"}

SCALAR_TIER_NOTE = (
    "anisotropy heuristic is scalar-tier (load-direction vs build-direction "
    "only); a tensor-based layer-normal stress check requires an ACE change"
)


def log(msg: str) -> None:
    print(msg, file=sys.stderr, flush=True)


def emit(payload: dict) -> None:
    print(json.dumps(payload), flush=True)


def resolve_material(db: dict, name: str) -> tuple[str, dict]:
    """Case-insensitive material lookup with the documented aliases."""
    key = str(name).strip().upper().replace("-", "").replace(" ", "")
    key = ALIASES.get(key, key)
    materials = db["materials"]
    for k in materials:
        if k.upper() == key:
            return k, materials[k]
    raise ValueError(
        f"unknown material {name!r} — available: {', '.join(sorted(materials))} "
        f"(aliases: TPU->TPU95A, NYLON->PA)"
    )


def out_of_plane_deg(build_dir, load_dir) -> float:
    """Angle (deg) of the load direction OUT of the layer plane: 0 = load in
    the plane of the layers, 90 = load exactly along the build direction."""
    bd = [float(v) for v in build_dir]
    ld = [float(v) for v in load_dir]
    nb = math.sqrt(sum(v * v for v in bd))
    nl = math.sqrt(sum(v * v for v in ld))
    if nb == 0.0 or nl == 0.0:
        raise ValueError("orientation vectors must be nonzero")
    cos_a = abs(sum(b * l for b, l in zip(bd, ld))) / (nb * nl)
    return math.degrees(math.asin(min(1.0, cos_a)))


def stress_rule(rule: str, allowable_mpa: float, demand_mpa: float,
                sf_req: float, detail: str) -> dict:
    # SF is null (not Infinity — invalid JSON) at zero demand; zero demand
    # trivially passes any positive allowable.
    sf = allowable_mpa / demand_mpa if demand_mpa > 0 else None
    passed = sf >= sf_req if sf is not None else True
    sf_text = f"{sf:.2f}" if sf is not None else "n/a (zero demand)"
    return {
        "rule": rule,
        "allowable_mpa": round(allowable_mpa, 4),
        "demand_mpa": round(demand_mpa, 4),
        "SF": round(sf, 4) if sf is not None else None,
        "pass": passed,
        "detail": f"{detail}; SF = {allowable_mpa:.2f}/{demand_mpa:.2f} = "
                  f"{sf_text} vs {sf_req:.2f} required -> "
                  f"{'PASS' if passed else 'FAIL'}",
    }


def run_check(job: dict) -> dict:
    """Evaluate all production rules for one job; returns the full receipt."""
    db = json.loads(DB_PATH.read_text(encoding="utf-8"))
    mat_name, m = resolve_material(db, job["material"])

    demand_mpa = float(job["max_von_mises_pa"]) / 1e6
    if demand_mpa < 0:
        raise ValueError(f"max_von_mises_pa must be >= 0, got {demand_mpa} MPa")
    sf_req = float(job.get("safety_factor_required", 2.0))
    character = job.get("load_character") or {}
    sustained = bool(character.get("sustained", False))
    cyclic = bool(character.get("cyclic", False))
    service_c = float(job.get("service_temp_c", 25.0))

    yield_mpa = float(m["yield_mpa"])
    ultimate_mpa = float(m["ultimate_mpa"])
    laf = float(m["layer_adhesion_factor"])
    creep_frac = float(m["creep_sustained_fraction"])
    fatigue_kd = float(m["fatigue_knockdown"])
    limit_c = float(m["service_temp_c"])

    rules: list[dict] = []
    skipped: list[dict] = []
    notes: list[str] = [f"material source: {m['source']}"]

    # --- anisotropy derate (scalar-tier heuristic) --------------------------
    derate = 1.0
    orientation = job.get("orientation")
    if orientation:
        angle = out_of_plane_deg(orientation["build_dir"],
                                 orientation["primary_load_dir"])
        notes.append(SCALAR_TIER_NOTE)
        if angle > 30.0:
            derate = laf
            rules.append(stress_rule(
                "anisotropy", yield_mpa * laf, demand_mpa, sf_req,
                f"primary load {angle:.1f} deg out of the layer plane "
                f"(> 30 deg) -> across-layer allowable = yield {yield_mpa:.2f}"
                f" x layer_adhesion {laf:.2f} = {yield_mpa * laf:.2f} MPa",
            ))
        else:
            skipped.append({
                "rule": "anisotropy",
                "reason": f"primary load only {angle:.1f} deg out of the layer "
                          f"plane (<= 30 deg) — layer adhesion not "
                          f"load-bearing; no derate applied",
            })
    else:
        skipped.append({
            "rule": "anisotropy",
            "reason": "orientation not provided — anisotropy UNCHECKED (give "
                      "{build_dir, primary_load_dir} to evaluate it)",
        })

    ani = f" x layer_adhesion {derate:.2f}" if derate != 1.0 else ""

    # --- static -------------------------------------------------------------
    rules.append(stress_rule(
        "static", yield_mpa * derate, demand_mpa, sf_req,
        f"allowable = yield {yield_mpa:.2f}{ani} = {yield_mpa * derate:.2f} MPa",
    ))

    # --- creep (sustained load) ----------------------------------------------
    if sustained:
        allow = yield_mpa * creep_frac * derate
        rules.append(stress_rule(
            "creep", allow, demand_mpa, sf_req,
            f"allowable = yield {yield_mpa:.2f} x creep_sustained_fraction "
            f"{creep_frac:.2f}{ani} = {allow:.2f} MPa",
        ))
    else:
        skipped.append({"rule": "creep", "reason": "load not sustained"})

    # --- fatigue (cyclic load) ------------------------------------------------
    if cyclic:
        allow = ultimate_mpa * fatigue_kd * derate
        rules.append(stress_rule(
            "fatigue", allow, demand_mpa, sf_req,
            f"allowable = ultimate {ultimate_mpa:.2f} x fatigue_knockdown "
            f"{fatigue_kd:.2f}{ani} = {allow:.2f} MPa (~1e6 cycles)",
        ))
    else:
        skipped.append({"rule": "fatigue", "reason": "load not cyclic"})

    # --- temperature ----------------------------------------------------------
    temp_pass = service_c <= limit_c
    rules.append({
        "rule": "temp",
        "allowable_c": limit_c,
        "demand_c": service_c,
        "SF": round(limit_c / service_c, 4) if service_c > 0 else None,
        "pass": temp_pass,
        "detail": f"service {service_c:.1f} C vs {mat_name} limit "
                  f"{limit_c:.1f} C (HDT-class) -> "
                  f"{'PASS' if temp_pass else 'FAIL'} (no safety factor "
                  f"applied to temperature — the limit IS the derated number)",
    })

    return {
        "ok": all(r["pass"] for r in rules),
        "material": mat_name,
        "safety_factor_required": sf_req,
        "anisotropy_derate_applied": derate != 1.0,
        "rules": rules,
        "skipped": skipped,
        "notes": notes,
        "disclaimer": db["disclaimer"],
    }


def selftest() -> None:
    """The pinned PETG gate (mirrors the ace_*_validation.py exit contract).

    A PETG part at 12 MPa demand under sustained load: static must PASS with
    SF > 4 (yield 50 -> 4.17), creep must FAIL at SF-required 2 (50 x 0.25 =
    12.5 MPa -> SF 1.04); at service 85 C the temp rule must FAIL (limit 75);
    a load across the layers must apply the 0.70 adhesion factor and carry
    the scalar-tier note.
    """
    r1 = run_check({
        "material": "PETG",
        "max_von_mises_pa": 12e6,
        "load_character": {"sustained": True, "cyclic": False},
        "service_temp_c": 25.0,
        "safety_factor_required": 2.0,
    })
    by_rule = {r["rule"]: r for r in r1["rules"]}
    static, creep = by_rule["static"], by_rule["creep"]
    r2 = run_check({"material": "PETG", "max_von_mises_pa": 12e6,
                    "service_temp_c": 85.0})
    temp2 = next(r for r in r2["rules"] if r["rule"] == "temp")
    r3 = run_check({
        "material": "PETG", "max_von_mises_pa": 12e6,
        "orientation": {"build_dir": [0, 0, 1], "primary_load_dir": [0, 0, 1]},
    })
    ani3 = next(r for r in r3["rules"] if r["rule"] == "anisotropy")

    checks = [
        ("static passes SF>4 at 12 MPa vs yield 50",
         static["pass"] and abs(static["SF"] - 50.0 / 12.0) < 1e-3),
        ("creep allowable 12.5 MPa", abs(creep["allowable_mpa"] - 12.5) < 1e-9),
        ("creep SF 1.04 fails required 2.0",
         not creep["pass"] and abs(creep["SF"] - 12.5 / 12.0) < 1e-3),
        ("overall verdict false under sustained load", r1["ok"] is False),
        ("temp fails at 85 C vs PETG limit 75",
         not temp2["pass"] and temp2["allowable_c"] == 75.0
         and temp2["demand_c"] == 85.0),
        ("across-layer load derates by 0.70: allowable 35 MPa",
         r3["anisotropy_derate_applied"]
         and abs(ani3["allowable_mpa"] - 35.0) < 1e-9),
        ("scalar-tier note present", any("scalar-tier" in n for n in r3["notes"])),
    ]
    for name, okay in checks:
        print(f"  {'PASS' if okay else 'FAIL'}: {name}", file=sys.stderr)
    if not all(okay for _, okay in checks):
        print("SELFTEST FAIL")
        sys.exit(1)
    print(f"SELFTEST PASS: static SF {static['SF']}, creep SF {creep['SF']} "
          f"(allowable {creep['allowable_mpa']} MPa), temp 85C vs 75C FAIL, "
          f"anisotropy 35.0 MPa allowable")


def main() -> None:
    job = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
    import _receipt

    _receipt.emit(run_check(job), job, "production_check")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--selftest":
        selftest()  # exit contract: nonzero on failure, assert-style
    else:
        try:
            main()
        except Exception as exc:  # noqa: BLE001 — the JSON line IS the contract
            emit({"ok": False, "error": f"{type(exc).__name__}: {exc}"})
            sys.exit(0)
