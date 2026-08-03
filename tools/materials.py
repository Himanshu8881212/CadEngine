#!/usr/bin/env python3
"""materials.py — the UNIFIED MATERIALS SOURCE OF TRUTH for the LMCAD Python
analyzers (plan 1.2).

WHY THIS EXISTS
---------------
Material properties used to be re-keyed in four places that WILL drift:
  * density (rho) hardcoded into ``mass_properties`` / balance / dossier jobs,
  * E, nu, rho pasted into every ``ace_fea`` / ``ace_modal`` / ``ace_buckling``
    job (e.g. ``cyclo26/v2/make_fea_jobs.py`` -> ``MAT``),
  * yield / ultimate / creep / fatigue / temp / anisotropy in
    ``tools/material_db.json`` (read by ``production_check.py``),
  * heat-set pull-out + printed shear tables inside ``joint_check.py``.

They HAVE already drifted — see the ``conflicts`` block inside
``materials/petg.json`` and ``materials/tpu95a.json`` (PETG E is 2.0e9 in the
drive FEA but 2.1e9 in material_db.json; PETG yield 47 vs 50; TPU density 1210
vs 1220). This module collapses them into ONE versioned, content-hashed record
per material, with a per-value citation map and an explicit conflict ledger so
the disagreements are visible instead of silent.

THE RECORD SCHEMA (see materials/*.json)
----------------------------------------
Each ``materials/<key>.json`` is one record:

  meta        { name, version (semver), aliases[], class, description,
                schema_version, hash }
              ``hash`` = sha256 of the record's canonical JSON (sorted keys,
              compact separators) with ``meta.hash`` REMOVED — a deterministic
              content hash. ``version`` is bumped by a human on any numeric
              change; ``hash`` is machine-derived and re-checked on load.

  physical    { density_kg_m3 }                       # rho for mass_properties

  mechanical  { youngs_modulus_pa, poisson, yield_mpa, ultimate_mpa,
                sn_curve, [compression_modulus_pa, resilience_coeff,
                elongation_at_break_pct] }
              ``sn_curve`` is a fatigue model: either
                {"kind":"knockdown", cycles, fraction_of_ultimate}  (the
                 project's rule-of-thumb, == material_db fatigue_knockdown), or
                {"kind":"basquin", sigma_f_prime_pa, b} for a real Basquin fit,
                or {"kind":"points", points:[[cycles, stress_pa], ...]}.
              null means NO fatigue data (honest — do not invent one).

  thermal     { conductivity_w_mk, specific_heat_j_kgk, cte_per_k (alpha),
                tg_or_melt_c, service_temp_c (HDT-class allowable limit),
                creep_sustained_fraction }

  process     FDM printing, ANISOTROPY IS FIRST-CLASS:
              { anisotropy: { z_vs_xy_strength_ratio, out_of_plane_threshold_deg,
                              model },
                layer_adhesion_mpa, fatigue_knockdown,
                heatset_pullout_n: { M3, M4, M5 },
                design_derate: { gate_factor, note } }
              ``z_vs_xy_strength_ratio`` is the across-layer (Z, tension normal
              to the layers) strength divided by the in-plane (XY) strength;
              this IS material_db's ``layer_adhesion_factor``. It is the
              first-class anisotropy handle used by ``derated()``.

  fluid       { surface_roughness_um }                # fluid-adjacent finish

  sources     { "<dotted.field.path>": "citation string", ... }
              per-value provenance for every value whose origin is known.

  conflicts   [ { field, canonical, other_value, other_source, note }, ... ]
              KNOWN cross-source disagreements. ``canonical`` is what THIS record
              serves; ``other_value``/``other_source`` is the value that some
              OTHER live copy still uses. Reconciling a conflict is a human
              physics decision (it changes an allowable), so it is surfaced, not
              silently overwritten.

ANISOTROPY + BUILD ORIENTATION (how a consumer combines record + print dir)
---------------------------------------------------------------------------
FDM parts are weakest across the layers. A record carries the strength ratio
(``z_vs_xy_strength_ratio``); the BUILD ORIENTATION lives with the PART, not the
material. A consumer combines them with ``derated(name, primary_load_dir,
build_dir)``:

  1. angle = asin(|load . build| / (|load| |build|))  degrees   # load out of
     the layer plane (0 = in-plane, 90 = straight across the layers).
  2. if angle > out_of_plane_threshold_deg (default 30):
         allowable = base_allowable * z_vs_xy_strength_ratio
     else:
         allowable = base_allowable                              # in-plane
This is exactly the scalar-tier rule ``production_check.py`` implements; it is
unified here so every consumer derates identically. It is a DIRECTION heuristic,
not a layer-normal stress-tensor check (that needs an ACE solver change).

RUST CONSUMPTION CONTRACT (documented follow-up — NOT implemented here)
----------------------------------------------------------------------
The records are language-neutral JSON. Rust (kernel-model mass_properties, and
any future native FEA bridge) should read the SAME ``tools/materials/<key>.json``
via serde and pull:
    density for mass = record.physical.density_kg_m3
    FEA elastic      = record.mechanical.youngs_modulus_pa, .poisson
The ``meta.hash`` is a stable cache key: a mass-properties or FEA result may be
memoized against (part_geometry_hash, material.meta.hash). A Rust-side loader
MUST recompute the hash the same way (serde_json::to_value -> sort keys ->
serialize with no spaces -> sha256, meta.hash excluded) and refuse a record
whose stored hash disagrees, mirroring ``validate()`` below. No numeric value
should EVER be hardcoded in Rust once it reads the record — that is the whole
point of this file.

CLI
---
  python materials.py --list                 names, versions, hashes
  python materials.py --show PETG            pretty-print one record
  python materials.py --rehash               recompute + write meta.hash back
                                             (run after editing any record)
  python materials.py --selftest             validate all + PROVE the PETG/TPU
                                             numbers match the drive/ball
                                             hardcodes verbatim (nonzero on fail)

Pure stdlib — no numpy, no third-party deps.
"""
from __future__ import annotations

import hashlib
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

REC_DIR = Path(__file__).resolve().parent / "materials"
SCHEMA_VERSION = 1

# Aliases accepted by get()/derated(), case-insensitive, punctuation-stripped.
# Mirrors production_check.py's ALIASES so the two agree on names.
ALIASES = {
    "TPU": "TPU95A",
    "TPU95": "TPU95A",
    "NYLON": "PA",
    "PETG": "PETG",
    "PET-G": "PETG",
}


# ---------------------------------------------------------------------------
# Deterministic content hash
# ---------------------------------------------------------------------------
def _strip_hash(record: dict) -> dict:
    """Deep copy of the record with meta.hash removed (the value hashed OVER)."""
    clone = json.loads(json.dumps(record))  # cheap deep copy, JSON-safe by defn
    clone.get("meta", {}).pop("hash", None)
    return clone


def content_hash(record: dict) -> str:
    """sha256 of the record's canonical JSON (sorted keys, compact separators),
    computed with meta.hash EXCLUDED. Deterministic across runs and machines:
    the only inputs are the record's values and Python's stable number repr."""
    canonical = json.dumps(
        _strip_hash(record), sort_keys=True, separators=(",", ":"),
        ensure_ascii=True,
    )
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


# ---------------------------------------------------------------------------
# Schema validation
# ---------------------------------------------------------------------------
def _require(cond: bool, msg: str) -> None:
    if not cond:
        raise ValueError(f"material schema violation: {msg}")


# Records store density in SI kg/m^3; the Rust BOM/mass path (format.rs) stores
# g/cm^3. The conversion is 1 g/cm^3 = 1000 kg/m^3 — a NAMED constant, never a
# silent factor at a call site (the whole point of the source of truth).
_KG_M3_PER_G_CM3 = 1000.0

# Plausible printed/engineering polymer+metal density band (kg/m^3). A value
# outside it is almost always a UNIT MIXUP — e.g. a g/cm^3 number (1.27) pasted
# where kg/m^3 was expected, or vice-versa. Rejected loudly so the mixup can't
# propagate into a mass or FEA that is off by 1000x.
_DENSITY_MIN_KG_M3 = 100.0     # < balsa; anything lower is a g/cm^3 mixup
_DENSITY_MAX_KG_M3 = 25000.0   # > tungsten; anything higher is nonsense


def kg_m3_to_g_cm3(kg_m3: float) -> float:
    """SI density -> the Rust BOM unit, via the one named conversion."""
    return kg_m3 / _KG_M3_PER_G_CM3


def _assert_density_range(kg_m3: float, name: str) -> None:
    if not (_DENSITY_MIN_KG_M3 <= kg_m3 <= _DENSITY_MAX_KG_M3):
        raise ValueError(
            f"{name}: density {kg_m3} kg/m^3 is outside the sane band "
            f"[{_DENSITY_MIN_KG_M3}, {_DENSITY_MAX_KG_M3}] — this is almost "
            f"certainly a UNIT MIXUP (g/cm^3 pasted as kg/m^3, or the reverse). "
            f"Records are kg/m^3; convert to g/cm^3 with kg_m3_to_g_cm3()."
        )


def validate(record: dict, *, check_hash: bool = True) -> None:
    """Assert one record satisfies the schema. Raises ValueError with a rich
    message on the first violation. If check_hash, the stored meta.hash must
    equal the freshly computed content hash (catches hand-edits that forgot
    --rehash)."""
    meta = record.get("meta")
    _require(isinstance(meta, dict), "missing 'meta' object")
    name = meta.get("name")
    _require(isinstance(name, str) and name, "meta.name must be a non-empty string")
    _require(isinstance(meta.get("version"), str) and meta["version"],
             f"{name}: meta.version must be a non-empty semver string")
    _require(meta.get("schema_version") == SCHEMA_VERSION,
             f"{name}: meta.schema_version must be {SCHEMA_VERSION}")

    phys = record.get("physical", {})
    rho = phys.get("density_kg_m3")
    _require(isinstance(rho, (int, float)) and rho > 0,
             f"{name}: physical.density_kg_m3 must be > 0")
    _assert_density_range(float(rho), name)  # catch a kg/m^3 <-> g/cm^3 mixup at load

    mech = record.get("mechanical", {})
    E = mech.get("youngs_modulus_pa")
    nu = mech.get("poisson")
    ys = mech.get("yield_mpa")
    us = mech.get("ultimate_mpa")
    _require(isinstance(E, (int, float)) and E > 0,
             f"{name}: mechanical.youngs_modulus_pa must be > 0")
    _require(isinstance(nu, (int, float)) and -1.0 < nu < 0.5,
             f"{name}: mechanical.poisson must be in (-1, 0.5)")
    _require(isinstance(ys, (int, float)) and ys > 0,
             f"{name}: mechanical.yield_mpa must be > 0")
    _require(isinstance(us, (int, float)) and us >= ys,
             f"{name}: mechanical.ultimate_mpa must be >= yield_mpa")

    thermal = record.get("thermal", {})
    _require(isinstance(thermal.get("service_temp_c"), (int, float)),
             f"{name}: thermal.service_temp_c must be a number")

    ani = record.get("process", {}).get("anisotropy", {})
    r = ani.get("z_vs_xy_strength_ratio")
    _require(isinstance(r, (int, float)) and 0.0 < r <= 1.0,
             f"{name}: process.anisotropy.z_vs_xy_strength_ratio must be in (0, 1]")

    _require(isinstance(record.get("sources"), dict),
             f"{name}: 'sources' must be an object (per-value citations)")
    _require(isinstance(record.get("conflicts"), list),
             f"{name}: 'conflicts' must be a list")

    if check_hash:
        stored = meta.get("hash")
        fresh = content_hash(record)
        _require(stored == fresh,
                 f"{name}: meta.hash {stored!r} != recomputed {fresh!r} "
                 f"(edit without `python materials.py --rehash`?)")


# ---------------------------------------------------------------------------
# Loading / lookup
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Material:
    """A resolved material record plus its content hash and helper accessors."""
    record: dict
    hash: str

    @property
    def name(self) -> str:
        return self.record["meta"]["name"]

    @property
    def version(self) -> str:
        return self.record["meta"]["version"]

    @property
    def density_kg_m3(self) -> float:
        return float(self.record["physical"]["density_kg_m3"])

    @property
    def density_g_cm3(self) -> float:
        """Density in the Rust BOM/mass unit (format.rs), via the one named
        conversion — so the same record serves Python's kg/m^3 and Rust's
        g/cm^3 without a scattered /1000 anyone can get wrong."""
        return kg_m3_to_g_cm3(self.density_kg_m3)

    @property
    def youngs_modulus_pa(self) -> float:
        return float(self.record["mechanical"]["youngs_modulus_pa"])

    @property
    def poisson(self) -> float:
        return float(self.record["mechanical"]["poisson"])

    @property
    def yield_mpa(self) -> float:
        return float(self.record["mechanical"]["yield_mpa"])

    @property
    def ultimate_mpa(self) -> float:
        return float(self.record["mechanical"]["ultimate_mpa"])

    def fea_material(self) -> dict:
        """The exact {youngs_modulus_pa, poisson, density_kg_m3} block an
        ace_fea / ace_modal / ace_buckling job expects — so a job builder can
        say ``"material": mat.fea_material()`` instead of pasting numbers."""
        return {
            "youngs_modulus_pa": self.youngs_modulus_pa,
            "poisson": self.poisson,
            "density_kg_m3": self.density_kg_m3,
        }


def _normalize(name: str) -> str:
    key = str(name).strip().upper().replace("-", "").replace(" ", "")
    return ALIASES.get(key, key)


def load_all(*, check_hash: bool = True) -> dict[str, dict]:
    """Load and validate every record in tools/materials/. Keyed by UPPER name."""
    if not REC_DIR.is_dir():
        raise FileNotFoundError(f"material record dir not found: {REC_DIR}")
    out: dict[str, dict] = {}
    for path in sorted(REC_DIR.glob("*.json")):
        record = json.loads(path.read_text(encoding="utf-8"))
        # SIDECAR TABLES live in this directory too (they are material data, so
        # they belong beside the records) but they are NOT material records and
        # must not be validated as one. They opt out by declaring a
        # meta.schema_kind other than "material_record" — e.g.
        # materials/fatigue.json (schema_kind "fatigue_table"), read by
        # tools/ace_fatigue_runner.py. Records themselves may omit the key.
        if record.get("meta", {}).get("schema_kind", "material_record") != "material_record":
            continue
        validate(record, check_hash=check_hash)
        key = record["meta"]["name"].upper()
        if key in out:
            raise ValueError(f"duplicate material name {key!r} ({path.name})")
        out[key] = record
    if not out:
        raise FileNotFoundError(f"no material records in {REC_DIR}")
    return out


def get(name: str, version: str | None = None) -> Material:
    """Serve a material by name (case-insensitive, aliases honored). If
    ``version`` is given it must match the record's meta.version exactly.
    Returns a Material (record + deterministic hash)."""
    db = load_all()
    key = _normalize(name)
    record = db.get(key)
    if record is None:
        raise KeyError(
            f"unknown material {name!r} -> {key!r}; available: "
            f"{', '.join(sorted(db))} (aliases: {', '.join(sorted(ALIASES))})"
        )
    if version is not None and record["meta"]["version"] != version:
        raise KeyError(
            f"{key}: requested version {version!r} but record is "
            f"{record['meta']['version']!r}"
        )
    return Material(record=record, hash=record["meta"]["hash"])


# ---------------------------------------------------------------------------
# Anisotropy: combine a record with a build orientation
# ---------------------------------------------------------------------------
def out_of_plane_deg(build_dir, load_dir) -> float:
    """Angle (deg) of the load direction OUT of the layer plane. 0 = load lies
    in the plane of the layers; 90 = load points straight along the build
    direction (across the layers). Identical math to production_check.py."""
    bd = [float(v) for v in build_dir]
    ld = [float(v) for v in load_dir]
    nb = math.sqrt(sum(v * v for v in bd))
    nl = math.sqrt(sum(v * v for v in ld))
    if nb == 0.0 or nl == 0.0:
        raise ValueError("orientation vectors must be nonzero")
    cos_a = abs(sum(b * l for b, l in zip(bd, ld))) / (nb * nl)
    return math.degrees(math.asin(min(1.0, cos_a)))


def derated(name: str, primary_load_dir, build_dir=(0.0, 0.0, 1.0),
            *, version: str | None = None, basis: str = "yield") -> dict:
    """Direction-dependent allowable for a printed part.

    Combines a material record with a PART build orientation and its primary
    load direction. If the load is more than the record's out-of-plane
    threshold (default 30 deg) out of the layer plane, the allowable is
    multiplied by the across-layer strength ratio (z_vs_xy_strength_ratio).

    basis: "yield" (default) -> base = mechanical.yield_mpa
           "ultimate"        -> base = mechanical.ultimate_mpa

    Returns a receipt {material, hash, basis, base_mpa, out_of_plane_deg,
    threshold_deg, ratio, factor_applied, allowable_mpa, note}."""
    mat = get(name, version=version)
    rec = mat.record
    if basis == "yield":
        base = float(rec["mechanical"]["yield_mpa"])
    elif basis == "ultimate":
        base = float(rec["mechanical"]["ultimate_mpa"])
    else:
        raise ValueError(f"basis must be 'yield' or 'ultimate', got {basis!r}")

    ani = rec["process"]["anisotropy"]
    ratio = float(ani["z_vs_xy_strength_ratio"])
    thresh = float(ani.get("out_of_plane_threshold_deg", 30.0))
    angle = out_of_plane_deg(build_dir, primary_load_dir)

    if angle > thresh:
        factor = ratio
        note = (f"load {angle:.1f} deg out of the layer plane (> {thresh:.0f}) "
                f"-> across-layer: allowable = {basis} {base:.2f} x "
                f"z/xy ratio {ratio:.2f}")
    else:
        factor = 1.0
        note = (f"load {angle:.1f} deg out of the layer plane (<= {thresh:.0f}) "
                f"-> in-plane: no anisotropy derate")

    return {
        "material": mat.name,
        "hash": mat.hash,
        "basis": basis,
        "base_mpa": round(base, 6),
        "out_of_plane_deg": round(angle, 4),
        "threshold_deg": thresh,
        "ratio": ratio,
        "factor_applied": factor,
        "allowable_mpa": round(base * factor, 6),
        "note": note,
    }


# ---------------------------------------------------------------------------
# CLI: --rehash, --list, --show, --selftest
# ---------------------------------------------------------------------------
def _rehash() -> None:
    """Recompute meta.hash for every record and write it back (pretty JSON)."""
    for path in sorted(REC_DIR.glob("*.json")):
        record = json.loads(path.read_text(encoding="utf-8"))
        validate(record, check_hash=False)
        record.setdefault("meta", {})["hash"] = content_hash(record)
        path.write_text(json.dumps(record, indent="\t", ensure_ascii=False) + "\n",
                        encoding="utf-8")
        print(f"  {record['meta']['name']:8} v{record['meta']['version']}  "
              f"{record['meta']['hash']}")
    print("rehash: OK")


def _list() -> None:
    for key, rec in sorted(load_all().items()):
        m = rec["meta"]
        print(f"  {m['name']:8} v{m['version']:8} {m['hash'][:16]}...  "
              f"{m.get('class', '?')}")


def _show(name: str) -> None:
    print(json.dumps(get(name).record, indent="\t", ensure_ascii=False))


def _selftest() -> None:
    """Validate all records AND prove the PETG/TPU numbers are byte-for-byte the
    values the drive/ball scripts hardcoded. Exit nonzero on any failure."""
    checks: list[tuple[str, bool]] = []

    # 1) every record validates (schema + hash).
    try:
        db = load_all(check_hash=True)
        checks.append((f"all {len(db)} records validate (schema + hash)", True))
    except Exception as exc:  # noqa: BLE001
        checks.append((f"record validation: {type(exc).__name__}: {exc}", False))
        _report(checks)
        sys.exit(1)

    # 2) hashing is deterministic (recompute == stored, and stable across calls).
    petg = get("PETG")
    checks.append(("PETG hash deterministic (recompute == stored)",
                   content_hash(petg.record) == petg.hash
                   and content_hash(petg.record) == content_hash(petg.record)))

    # 3) PROOF vs the DRIVE hardcodes. The cyclo26 FEA job material block and
    #    make_fea_jobs.py MAT both hardcode E=2.0e9, nu=0.37, rho=1270; the
    #    gate basis analytics.py ALLOW_PETG = 28.2e6 = yield 47 * derate 0.6.
    root = Path(__file__).resolve().parent.parent
    job_path = root / "cyclo26/v2/fea_output_hub6/fea_job.json"
    drive_mat = None
    if job_path.is_file():
        drive_mat = json.loads(job_path.read_text(encoding="utf-8"))["material"]
    # Fallbacks so the proof still runs if the receipt dir was cleaned:
    if drive_mat is None:
        drive_mat = {"youngs_modulus_pa": 2.0e9, "poisson": 0.37,
                     "density_kg_m3": 1270}

    checks.append((f"PETG E == drive {drive_mat['youngs_modulus_pa']:.6g} Pa "
                   f"(record {petg.youngs_modulus_pa:.6g})",
                   petg.youngs_modulus_pa == float(drive_mat["youngs_modulus_pa"])))
    checks.append((f"PETG nu == drive {drive_mat['poisson']} "
                   f"(record {petg.poisson})",
                   petg.poisson == float(drive_mat["poisson"])))
    checks.append((f"PETG rho == drive {drive_mat['density_kg_m3']} kg/m^3 "
                   f"(record {petg.density_kg_m3})",
                   petg.density_kg_m3 == float(drive_mat["density_kg_m3"])))

    # Gate basis: yield 47 * design derate 0.6 == 28.2 MPa (analytics.py).
    derate = float(petg.record["process"]["design_derate"]["gate_factor"])
    allow = petg.yield_mpa * derate
    checks.append((f"PETG gate basis: yield {petg.yield_mpa} * derate {derate} "
                   f"= {allow:.4g} MPa == analytics.py 28.2",
                   abs(allow - 28.2) < 1e-9))

    # 4) PROOF vs the BALL (basketball TPU) hardcodes: rho 1210, compression
    #    modulus 38e6, resilience 0.43, elongation > 560%.
    tpu = get("TPU95A")
    checks.append((f"TPU rho == basketball 1210 kg/m^3 (record {tpu.density_kg_m3})",
                   tpu.density_kg_m3 == 1210.0))
    tmech = tpu.record["mechanical"]
    checks.append((f"TPU compression modulus == 38e6 Pa "
                   f"(record {tmech.get('compression_modulus_pa')})",
                   float(tmech["compression_modulus_pa"]) == 38.0e6))
    checks.append((f"TPU resilience == 0.43 (record {tmech.get('resilience_coeff')})",
                   float(tmech["resilience_coeff"]) == 0.43))
    checks.append((f"TPU elongation-at-break > 560% "
                   f"(record {tmech.get('elongation_at_break_pct')})",
                   float(tmech["elongation_at_break_pct"]) > 560.0))

    # 5) anisotropy helper reproduces production_check's across-layer derate:
    #    an across-layer (Z) load on PETG derates yield 47 by the ratio.
    d = derated("PETG", primary_load_dir=(0, 0, 1), build_dir=(0, 0, 1))
    ratio = float(petg.record["process"]["anisotropy"]["z_vs_xy_strength_ratio"])
    checks.append((f"derated() across-layer PETG = yield 47 x ratio {ratio} "
                   f"= {47.0 * ratio:.2f} (got {d['allowable_mpa']:.2f})",
                   abs(d["allowable_mpa"] - 47.0 * ratio) < 1e-9
                   and d["factor_applied"] == ratio))

    _report(checks)
    if not all(ok for _, ok in checks):
        print("SELFTEST FAIL")
        sys.exit(1)
    print(f"SELFTEST PASS: {len(db)} records; PETG E/nu/rho/yield match the drive "
          f"FEA verbatim, gate basis 28.2 MPa reproduced; TPU basketball numbers "
          f"match; anisotropy derate unified.")


def _report(checks: list[tuple[str, bool]]) -> None:
    for name, ok in checks:
        print(f"  {'PASS' if ok else 'FAIL'}: {name}", file=sys.stderr)


def main(argv: list[str]) -> None:
    if not argv or argv[0] in ("-h", "--help"):
        print(__doc__)
        return
    cmd = argv[0]
    if cmd == "--rehash":
        _rehash()
    elif cmd == "--list":
        _list()
    elif cmd == "--show":
        _show(argv[1])
    elif cmd == "--selftest":
        _selftest()
    else:
        raise SystemExit(f"unknown command {cmd!r}; see --help")


if __name__ == "__main__":
    main(sys.argv[1:])
