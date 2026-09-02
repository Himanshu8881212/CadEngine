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
    duration_h              REQUIRED WHEN sustained — the DESIGN DURATION the
                                      load is held for, in hours. There is no
                                      default: "how long" is half of a creep
                                      question, and a creep verdict with no
                                      duration in it is not a creep verdict.
                                      Also accepted as service.duration_h or
                                      load_character.duration_h.
    orientation             optional  {build_dir: [x,y,z],
                                       primary_load_dir: [x,y,z]} — enables
                                      the anisotropy rule; absent => rule
                                      skipped WITH a note (never silently)
    safety_factor_required  optional  default 2.0
    creep_interpolation     optional  default false. true = OPT IN to reading
                                      the creep allowable INTERPOLATED between
                                      the bracketing table cells (linear in
                                      temperature, log-linear in duration —
                                      materials.creep_lookup(interpolate=True))
                                      instead of the default conservative
                                      round-up (a 30 C request reads the 55 C
                                      row by default). The receipt says so:
                                      top-level creep_interpolation:true, the
                                      creep row's creep_interpolation flag,
                                      creep_cell.basis "interpolated", both
                                      bracketing cells, the formula, and the
                                      bucket the default would have read.
                                      Never extrapolates; above 55 C still
                                      refuses; no Rust mirror.

Rules (each receipt row shows every derating in its arithmetic):
    static      allowable = yield                       (always)
    creep       allowable = the material record's TIME x TEMPERATURE creep
                table cell at (service_temp_c, duration_h), read through
                tools/materials.py — the ONE reader, shared with the Rust
                contract. The row carries a `creep_cell` block naming the
                exact cell and how it was reached (exact / rounded-up /
                extrapolated), so "which cell was this margin read at" is a
                gateable number.  (sustained)
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

CREEP: the table governs, and it can REFUSE (behavior change, 2026-08-08)
------------------------------------------------------------------------
This tool used to compute the sustained allowable as
``yield * thermal.creep_sustained_fraction`` — a time-blind scalar fraction of
the STATIC yield the creep rule exists to replace (PLA: 55.0 x 0.2 = 11.0 MPa),
with no duration input anywhere in the job schema. The material record's own
conflict ledger already named that a conflict, and OPERATOR_BRIEF §7 already
said **the table governs**: PLA's tabulated sustained allowable is 5.0 MPa at
23 C / 24 h and 2.5 MPa at 23 C / 1 y — up to 4.4x below the legacy number.
A campaign that gated a sustained load here and stopped had, in effect, gated it
on yield.

Now: the creep row reads ``creep.sig_allow_mpa`` at the STATED temperature and
duration, and the legacy scalar is carried alongside as
``legacy_scalar_mpa`` (never as the allowable) so the conflict stays visible.
Three things make the row FAIL loudly instead of returning a plausible number:
  * ``sustained: true`` with **no ``duration_h``** -> `pass: false`,
    ``refusal_kind: "creep_duration_required"``;
  * a service temperature **above the hottest tabulated tier** (PLA: 55 C)
    -> `pass: false`, ``refusal_kind: "creep_temp_above_tabulated"`` — the
    reader does NOT fall back to the hot row;
  * a material with **no creep table** -> `pass: false`,
    ``refusal_kind: "creep_no_table"`` — the scalar is NOT served instead.
A refusal is still a full receipt with ``ok: false``; it is never an exception
and never a silent number.

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

TOOLS_DIR = Path(__file__).resolve().parent
DB_PATH = TOOLS_DIR / "material_db.json"
sys.path.insert(0, str(TOOLS_DIR))
import materials as _materials  # noqa: E402 — THE one reader of the creep table

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


def refusal_row(rule: str, demand_mpa: float, sf_req: float, kind: str,
                detail: str, extra: dict | None = None) -> dict:
    """A rule that CANNOT be evaluated is a FAILING row with a machine-matchable
    `refusal_kind`, never a skipped row and never a plausible number. Allowable
    is 0.0 so any downstream `demand <= allowable` reader also fails, and `pass`
    is False even at zero demand — an unknown allowable cannot pass anything."""
    row = {
        "rule": rule,
        "allowable_mpa": 0.0,
        "demand_mpa": round(demand_mpa, 4),
        "SF": 0.0 if demand_mpa > 0 else None,
        "pass": False,
        "refused": True,
        "refusal_kind": kind,
        "detail": f"{detail} -> REFUSED ({kind}); this rule FAILS rather than "
                  f"returning an allowable the data does not support",
    }
    row.update(extra or {})
    return row


#: Where a design duration may be stated in a job, in precedence order. Several
#: spellings are accepted because the friction reports proposed different ones;
#: `duration_h` at the top level is canonical and matches `service_temp_c`.
_DURATION_PATHS = (("duration_h",), ("service", "duration_h"),
                   ("load_character", "duration_h"))


def job_duration_h(job: dict):
    """(hours, dotted_path_it_came_from) or (None, None). Never defaults."""
    for path in _DURATION_PATHS:
        cur = job
        for part in path:
            cur = cur.get(part) if isinstance(cur, dict) else None
        if cur is not None:
            return cur, ".".join(path)
    return None, None


def creep_row(mat_name: str, demand_mpa: float, sf_req: float, service_c: float,
              job: dict, across_layer: bool) -> dict:
    """The sustained-load row, read from the material record's time x temperature
    creep table through `materials.creep_lookup` — the ONE reader, shared with
    `kernel_model::materials::pla::creep_allowable_mpa`. The legacy scalar
    (`yield * thermal.creep_sustained_fraction`) is reported for visibility and
    is never the allowable."""
    duration_h, dur_from = job_duration_h(job)
    try:
        legacy = _materials.legacy_creep_scalar(mat_name)
    except Exception:  # noqa: BLE001 — no record file for this db key
        legacy = {"mpa": None, "note": f"no tools/materials/{mat_name.lower()}.json record"}
    common = {
        "service_temp_c": service_c,
        "duration_h": duration_h,
        "duration_from": dur_from,
        "across_layer": across_layer,
        "legacy_scalar_mpa": legacy.get("mpa"),
        "legacy_scalar_note": legacy.get("note"),
    }

    if duration_h is None:
        return refusal_row(
            "creep", demand_mpa, sf_req, "creep_duration_required",
            "sustained load declared but the job states NO design duration "
            "(`duration_h`, in hours). A creep allowable is a function of "
            "temperature AND time; without a duration the only number this tool "
            f"could return is the time-blind legacy scalar "
            f"({legacy.get('mpa')} MPa = yield x creep_sustained_fraction), which "
            "is a fraction of the STATIC yield this rule exists to replace",
            common)

    # Opt-in ONLY: the default is the conservative bucket (both axes rounded
    # UP). A job that sets "creep_interpolation": true gets the allowable
    # interpolated between the bracketing cells (linear in temperature,
    # log-linear in duration — tools/materials.py CREEP_INTERPOLATION_FORMULA)
    # and the row says so in `creep_interpolation`, `creep_cell.basis` and its
    # detail, naming both cells and the bucket the default would have read.
    interp = bool(job.get("creep_interpolation", False))
    common["creep_interpolation"] = interp
    lookup = _materials.creep_lookup(mat_name, service_c, duration_h,
                                     across_layer=across_layer, interpolate=interp)
    common["creep_cell"] = lookup
    if lookup["refused"]:
        return refusal_row("creep", demand_mpa, sf_req, lookup["refusal_kind"],
                           lookup["note"], common)

    ani = (f" x z/xy {lookup['z_vs_xy_strength_ratio']} (across-layer)"
           if across_layer else " (in-plane)")
    if lookup.get("interpolated"):
        cells = ", ".join(f"[{c['temperature_bucket']}][{c['duration_bucket']}] {c['mpa']:.4g}"
                          for c in lookup["bracketing_cells"])
        db = lookup["default_bucket"]
        detail = (f"allowable = INTERPOLATED (creep_interpolation:true — linear in T, "
                  f"log-linear in time) between {cells} MPa -> "
                  f"{lookup['in_plane_mpa']:.4g} MPa{ani} = {lookup['sig_allow_mpa']:.4g} MPa "
                  f"for {service_c:.1f} C held {duration_h:g} h (basis interpolated; the "
                  f"default conservative bucket would read [{db['temperature_bucket']}]"
                  f"[{db['duration_bucket']}] {db['in_plane_mpa']:.4g} MPa; legacy scalar "
                  f"{legacy.get('mpa')} MPa is SUPERSEDED by the table and is NOT the allowable)")
    else:
        detail = (f"allowable = {lookup['table_source'].split('#')[1]}"
                  f"[{lookup['temperature_bucket']}][{lookup['duration_bucket']}] "
                  f"{lookup['in_plane_mpa']:.4g} MPa{ani} = {lookup['sig_allow_mpa']:.4g} MPa "
                  f"for {service_c:.1f} C held {duration_h:g} h "
                  f"(cell {lookup['cell_match']}; legacy scalar {legacy.get('mpa')} MPa is "
                  f"SUPERSEDED by the table and is NOT the allowable)")
    row = stress_rule("creep", lookup["sig_allow_mpa"], demand_mpa, sf_req, detail)
    row["refused"] = False
    row["refusal_kind"] = None
    row.update(common)
    return row


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
    # The material record's time x temperature TABLE governs (OPERATOR_BRIEF §7);
    # the legacy `yield x creep_sustained_fraction` scalar is reported inside the
    # row for visibility and is never the allowable. See the module docstring.
    if sustained:
        rules.append(creep_row(mat_name, demand_mpa, sf_req, service_c, job,
                               across_layer=(derate != 1.0)))
        notes.append(
            "creep allowable comes from the material record's creep.sig_allow_mpa "
            "table at the STATED service temperature and duration, read through "
            "tools/materials.py (the same reader the Rust contract mirrors); the "
            f"legacy scalar yield {yield_mpa:.2f} x creep_sustained_fraction "
            f"{creep_frac:.2f} = {yield_mpa * creep_frac:.2f} MPa is a recorded "
            "conflict, not an allowable")
        if job.get("creep_interpolation"):
            notes.append(
                "creep_interpolation: true — the creep allowable is INTERPOLATED "
                "between the bracketing table cells (linear in temperature, "
                "log-linear in duration; tools/materials.py "
                "CREEP_INTERPOLATION_FORMULA) instead of the default conservative "
                "round-up; the creep row names both cells and the bucket the default "
                "would have read. Quote it as 'interpolated', never as a table cell — "
                "it is a model between two conservative constructions, has no Rust "
                "mirror, and never extrapolates (above 55 C it still refuses)")
    else:
        skipped.append({"rule": "creep", "reason": "load not sustained"
                        + (" (creep_interpolation requested — no effect without a "
                           "sustained load)" if job.get("creep_interpolation") else "")})

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

    out = {
        "ok": all(r["pass"] for r in rules),
        "material": mat_name,
        "safety_factor_required": sf_req,
        "anisotropy_derate_applied": derate != 1.0,
        "rules": rules,
        "skipped": skipped,
        "notes": notes,
        "disclaimer": db["disclaimer"],
    }
    if job.get("creep_interpolation"):
        # Only present when the job asked for it, so every receipt that never
        # did stays byte-identical to what its campaign shipped.
        out["creep_interpolation"] = True
    return out


def selftest() -> None:
    """The pinned PETG gate plus the PLA creep-table contract (mirrors the
    ace_*_validation.py exit contract).

    A PETG part at 12 MPa demand: static must PASS with SF > 4 (yield 50 ->
    4.17); at service 85 C the temp rule must FAIL (limit 75); a load across the
    layers must apply the 0.70 adhesion factor and carry the scalar-tier note.

    The creep block pins the T14 fix: the verdict comes from the material
    record's creep TABLE at a STATED temperature and duration, it REFUSES when
    the duration is missing / the temperature is off the top of the table / the
    material has no table, and the legacy 0.2-fraction scalar is reported but
    never used.
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

    def pla_creep(**over):
        job = {"material": "PLA", "max_von_mises_pa": 2.0e6,
               "load_character": {"sustained": True}, "service_temp_c": 23.0,
               "safety_factor_required": 1.0}
        job.update(over)
        r = run_check(job)
        return r, next(x for x in r["rules"] if x["rule"] == "creep")

    r_nodur, c_nodur = pla_creep()
    r_1y, c_1y = pla_creep(duration_h=8760.0)
    r_24h, c_24h = pla_creep(duration_h=24.0)
    r_hot, c_hot = pla_creep(duration_h=24.0, service_temp_c=70.0)
    r_25c, c_25c = pla_creep(duration_h=24.0, service_temp_c=25.0)
    r_z, c_z = pla_creep(duration_h=8760.0,
                         orientation={"build_dir": [0, 0, 1],
                                      "primary_load_dir": [0, 0, 1]})
    r_nest, c_nest = pla_creep(service={"duration_h": 8760.0})
    # Opt-in interpolation (2026-09-02): 30 C / 24 h — default reads the 55C row
    # (1.5 MPa); interpolated = 5.0 + (30-23)/(55-23) x (1.5-5.0) = 4.234375 MPa.
    r_i30, c_i30 = pla_creep(duration_h=24.0, service_temp_c=30.0, creep_interpolation=True)
    r_d30, c_d30 = pla_creep(duration_h=24.0, service_temp_c=30.0)

    checks = [
        ("static passes SF>4 at 12 MPa vs yield 50",
         static["pass"] and abs(static["SF"] - 50.0 / 12.0) < 1e-3),
        # PETG has NO creep table: the tool must refuse, not serve 50 x 0.25.
        ("PETG sustained REFUSES (no creep table) instead of serving the "
         "time-blind 12.5 MPa scalar",
         creep["refused"] and creep["refusal_kind"] == "creep_duration_required"
         and creep["allowable_mpa"] == 0.0 and creep["pass"] is False),
        ("overall verdict false under sustained load", r1["ok"] is False),
        ("temp fails at 85 C vs PETG limit 75",
         not temp2["pass"] and temp2["allowable_c"] == 75.0
         and temp2["demand_c"] == 85.0),
        ("across-layer load derates by 0.70: allowable 35 MPa",
         r3["anisotropy_derate_applied"]
         and abs(ani3["allowable_mpa"] - 35.0) < 1e-9),
        ("scalar-tier note present", any("scalar-tier" in n for n in r3["notes"])),
        # --- T14: the creep table governs, and the tool can refuse ---------
        ("sustained with NO duration_h REFUSES (creep_duration_required), "
         "ok:false, and still emits a full receipt",
         c_nodur["refused"] and c_nodur["refusal_kind"] == "creep_duration_required"
         and c_nodur["pass"] is False and r_nodur["ok"] is False
         and c_nodur["legacy_scalar_mpa"] == 11.0),
        ("PLA 23 C / 1 y creep allowable is the TABLE's 2.5 MPa, not the legacy "
         "11.0 MPa scalar (4.4x)",
         abs(c_1y["allowable_mpa"] - 2.5) < 1e-9
         and c_1y["legacy_scalar_mpa"] == 11.0
         and abs(c_1y["SF"] - 1.25) < 1e-9),
        ("PLA 23 C / 24 h creep allowable is 5.0 MPa",
         abs(c_24h["allowable_mpa"] - 5.0) < 1e-9),
        ("every creep row names the CELL it was read at (exact / rounded up)",
         c_1y["creep_cell"]["temperature_bucket"] == "23C"
         and c_1y["creep_cell"]["duration_bucket"] == "1y"
         and c_1y["creep_cell"]["cell_match"] == "exact"
         and c_25c["creep_cell"]["temperature_bucket"] == "55C"
         and c_25c["creep_cell"]["cell_match"] == "rounded_up_conservative"),
        ("a declared 25 C ambient reads the 55 C row (1.5 MPa) — the step is "
         "visible in the receipt instead of hidden in prose",
         abs(c_25c["allowable_mpa"] - 1.5) < 1e-9),
        ("70 C sustained REFUSES (creep_temp_above_tabulated) — no fallback to "
         "the 55 C row",
         c_hot["refused"] and c_hot["refusal_kind"] == "creep_temp_above_tabulated"
         and c_hot["allowable_mpa"] == 0.0 and r_hot["ok"] is False),
        ("an across-layer sustained load derates the CELL by 0.55 and says so: "
         "2.5 -> 1.375 MPa",
         c_z["across_layer"] is True
         and abs(c_z["allowable_mpa"] - 1.375) < 1e-9
         and c_z["creep_cell"]["anisotropy_factor"] == 0.55),
        ("duration may also be stated as service.duration_h",
         c_nest["duration_from"] == "service.duration_h"
         and abs(c_nest["allowable_mpa"] - 2.5) < 1e-9),
        # --- opt-in creep interpolation: the default is UNCHANGED, the opt-in
        # says so everywhere it can ------------------------------------------
        ("30 C / 24 h by DEFAULT reads the 55C row (1.5 MPa) and carries no "
         "interpolation flag",
         abs(c_d30["allowable_mpa"] - 1.5) < 1e-9
         and c_d30["creep_cell"]["cell_match"] == "rounded_up_conservative"
         and c_d30["creep_interpolation"] is False
         and "creep_interpolation" not in r_d30),
        ("creep_interpolation:true at 30 C / 24 h -> 5.0 + 7/32 x (1.5-5.0) = "
         "4.234375 MPa, basis 'interpolated', both cells named, default bucket "
         "1.5 MPa beside it, top-level flag + note present",
         c_i30["creep_cell"]["sig_allow_mpa"] == 4.234375
         and abs(c_i30["allowable_mpa"] - 4.234375) < 1e-4  # the row rounds to 4 decimals
         and c_i30["creep_interpolation"] is True
         and c_i30["creep_cell"]["basis"] == "interpolated"
         and c_i30["creep_cell"]["cell_match"] == "interpolated"
         and [(c["temperature_bucket"], c["duration_bucket"], c["mpa"])
              for c in c_i30["creep_cell"]["bracketing_cells"]]
         == [("23C", "24h", 5.0), ("55C", "24h", 1.5)]
         and c_i30["creep_cell"]["default_bucket_mpa"] == 1.5
         and "INTERPOLATED" in c_i30["detail"]
         and r_i30.get("creep_interpolation") is True
         and any("creep_interpolation: true" in n for n in r_i30["notes"])),
    ]
    for name, okay in checks:
        print(f"  {'PASS' if okay else 'FAIL'}: {name}", file=sys.stderr)
    if not all(okay for _, okay in checks):
        print("SELFTEST FAIL")
        sys.exit(1)
    print(f"SELFTEST PASS: static SF {static['SF']}, PETG sustained refused "
          f"({creep['refusal_kind']}), temp 85C vs 75C FAIL, anisotropy 35.0 MPa; "
          f"PLA creep table governs — 23C/1y {c_1y['allowable_mpa']} MPa (legacy "
          f"scalar {c_1y['legacy_scalar_mpa']} MPa NOT used), 25C reads the "
          f"{c_25c['creep_cell']['temperature_bucket']} row, 70C refuses")


def main() -> None:
    import _receipt

    job, out = _receipt.load_job()
    payload = run_check(job)
    # A refused rule names WHY on the receipt's own error_kind, so a shell gate
    # can branch on `refusal.creep_duration_required` without regexing prose.
    refused = next((r for r in payload["rules"] if r.get("refused")), None)
    kind = (f"refusal.{refused['refusal_kind']}" if refused
            else None if payload["ok"] else "gate_failed")
    _receipt.finish(payload, job=job, tool="production_check", out=out, kind=kind,
                    use_out_dir_default=True)


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--selftest":
        selftest()  # exit contract: nonzero on failure, assert-style
    else:
        import _receipt

        # `_receipt.run_cli` guarantees a receipt on EVERY path and an exit code
        # that AGREES with `ok` (0 pass / 1 could-not-run / 2 ran-and-failed).
        # The old block here emitted the failure JSON and then `sys.exit(0)` —
        # a crashed run looked, to `$?`, exactly like a passing one.
        _receipt.run_cli("production_check", main)
