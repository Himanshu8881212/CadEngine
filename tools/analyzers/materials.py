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
              ``validate()`` PROVES the ledger is not itself stale: every
              ``field`` must resolve to a real dotted path in the record, and a
              scalar ``canonical`` must equal the value the record actually
              serves. Without that check a ledger entry is prose that can drift
              away from the number it claims to describe (this is what kept the
              PLA cp 1200 vs 1800 J/kgK entry honest, and it is now machine-checked).

  creep       OPTIONAL time x temperature SUSTAINED-stress table:
              { basis, sig_allow_mpa: { "<T>C": { "<n><unit>": MPa, ... }, ... },
                confidence: same shape (per-cell strings), derivation[],
                data_anchors[], model_constants{}, gaps_unknowns[] }
              When a record carries this table it GOVERNS every sustained-load
              allowable for that material, and ``thermal.creep_sustained_fraction``
              is SUPERSEDED (see ``creep_lookup`` / ``legacy_creep_scalar``).
              Read it ONLY through ``creep_lookup`` / ``creep_allowable_mpa`` —
              they are the single reader, and they mirror the Rust contract in
              ``kernel_model::materials::pla`` exactly (see CREEP SEMANTICS).

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

CREEP SEMANTICS — ONE TABLE, ONE READER, THE REFUSING SEMANTIC WINS
-------------------------------------------------------------------
Sustained load is a CREEP case, and creep — not instantaneous strength — is what
kills a printed part held under load. The allowable lives in
``materials/<key>.json#creep.sig_allow_mpa`` as a **temperature x duration step
table**, and this module is its ONLY Python reader. The rules, which are the
same rules `kernel_model::materials::pla::creep_allowable_mpa` implements in
Rust (the cross-language pin in ``materials_crosslang_test.py`` proves it at
every tier boundary):

  * **NO interpolation by default.** The table is a coarse step (printed PLA:
    two temperature tiers, 23 C and 55 C, with NOTHING between). The temperature
    is rounded UP to the next tabulated tier and the duration is rounded UP to
    the next tabulated column, so an in-between request always reads the WORSE
    cell. ``cell_match`` in the receipt says whether the cell was hit ``exact``
    or reached by ``rounded_up_conservative``.
  * **Interpolation is OPT-IN, labelled, and Python-only** (2026-09-02).
    ``creep_lookup(..., interpolate=True)`` returns the allowable interpolated
    between the bracketing cells — linear in temperature, log-linear in
    duration (``CREEP_INTERPOLATION_FORMULA``): a 30 C / 24 h request reads
    5.0 + (30-23)/(55-23) x (1.5-5.0) = 4.234375 MPa instead of the default
    bucket's 1.5 MPa. The receipt then carries ``basis: "interpolated"``,
    ``cell_match: "interpolated"``, every bracketing cell with its confidence
    string in ``bracketing_cells``, the ``formula``, and ``default_bucket`` /
    ``default_bucket_mpa`` (what the default would have read) so the two
    answers always sit side by side. It NEVER extrapolates: above the hottest
    row it still refuses; below the coldest row / before the first column it
    clamps to that cell and is then NOT labelled interpolated (nothing was);
    beyond the last column the last column is reused as in the default. No
    measured cell is invented — it is a model between two conservative
    constructions, the LOWEST bracketing confidence governs, and it has NO
    Rust mirror (the cross-language pin covers the default reader only).
    ``production_check.py`` uses it only when the job sets
    ``"creep_interpolation": true`` and its receipt says so.
  * **Above the last tabulated temperature the lookup REFUSES.** It does NOT
    fall back to the hottest row. ``sig_allow_mpa`` is 0.0, ``known`` is False,
    ``refused`` is True and ``refusal_kind`` is machine-matchable, so a gate
    written as ``demand <= allowable`` FAILS loudly in exactly the regime where
    no data exists. (The previous Python reader returned the 55 C row at 70 C
    and at 120 C, flagging only ``extrapolated: True`` — a field a gate can miss.
    That is the divergence this module exists to end.)
  * **Non-finite / missing / negative inputs REFUSE** rather than defaulting.
    A typo must never become a silent allowable.
  * **Beyond the last duration column the last column is reused**, flagged
    ``duration_match = "extrapolated_beyond_last_column"`` — that is what the
    source record's own bound says to do, and both languages agree on it.
  * **The table GOVERNS the legacy scalar.** ``thermal.creep_sustained_fraction``
    (the blanket "sustained = 20 % of yield" rule, ~11-12 MPa for PLA) is
    reported as ``legacy_scalar_mpa`` for visibility and is NEVER the allowable
    when a table exists. A material with NO table gets a refusal, not the scalar
    (``legacy_creep_scalar()`` exists to quote the conflict, not to gate on it).
  * **Anisotropy is the CALLER's explicit choice, never silent.** The tabulated
    cells are IN-PLANE. Across-layer sustained load is derated by
    ``process.anisotropy.z_vs_xy_strength_ratio`` (0.55 for PLA) only when the
    caller passes ``across_layer=True``; the receipt always records
    ``across_layer`` and ``anisotropy_factor`` so the choice is visible. The
    ratio derates the ALLOWABLE, never the modulus E.

Every lookup returns a RECEIPT (``creep_lookup``) that names the material, its
content hash, the requested T and duration, the cell actually read, how it was
reached, and the per-cell confidence string from the record — so "which cell was
this margin read at" is a gateable number instead of a sentence in a README.
``creep_allowable_mpa`` is the bare scalar for a one-line gate; it is exactly
``creep_lookup(...)["sig_allow_mpa"]``.

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
  python materials.py --creep PLA 23 8760    sustained-creep allowable RECEIPT
                                             (material, T in C, duration in h;
                                             add --across-layer for the 0.55
                                             across-layer derate, --interpolate
                                             for the opt-in labelled
                                             interpolation between bracketing
                                             cells). Exits 1 and still prints
                                             the JSON receipt when the lookup
                                             REFUSES.
  python materials.py --selftest             validate all + PROVE the PETG/TPU
                                             numbers match the drive/ball
                                             hardcodes verbatim + the creep
                                             refusal contract (nonzero on fail)

Pure stdlib — no numpy, no third-party deps.
"""
from __future__ import annotations

import hashlib
import json
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path

REC_DIR = Path(__file__).resolve().parent.parent / "materials"  # tools/materials/ (DATA stays at the top level)
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


_MISSING = object()


def dig(record: dict, dotted: str):
    """Resolve a dotted field path inside a record, or ``_MISSING``. Used by the
    conflict-ledger validator and by consumers that want to quote a field by the
    same name the ledger uses."""
    cur = record
    for part in str(dotted).split("."):
        if not isinstance(cur, dict) or part not in cur:
            return _MISSING
        cur = cur[part]
    return cur


def _validate_conflicts(record: dict, name: str) -> None:
    """A conflict ledger that has drifted from the record is WORSE than no
    ledger: it documents a disagreement about a number the record no longer
    serves. Every entry must name a real dotted path, and a scalar ``canonical``
    must equal the live value. (This is what keeps the PLA cp 1200 vs 1800
    J/kgK entry and the creep_sustained_fraction 0.2 vs creep-table entry
    honest — they are now machine-checked, not prose.)"""
    for i, entry in enumerate(record.get("conflicts") or []):
        _require(isinstance(entry, dict), f"{name}: conflicts[{i}] must be an object")
        field = entry.get("field")
        _require(isinstance(field, str) and field,
                 f"{name}: conflicts[{i}].field must be a non-empty dotted path")
        for key in ("canonical", "other_value", "other_source", "note"):
            _require(key in entry, f"{name}: conflicts[{i}] ({field}) is missing {key!r}")
        live = dig(record, field)
        _require(live is not _MISSING,
                 f"{name}: conflicts[{i}].field {field!r} does not resolve to any "
                 f"field in the record — the ledger names a value this record no "
                 f"longer serves (stale conflict entry)")
        canonical = entry["canonical"]
        if isinstance(canonical, (int, float)) and not isinstance(canonical, bool):
            _require(isinstance(live, (int, float)) and float(live) == float(canonical),
                     f"{name}: conflicts[{i}] says {field} canonical is {canonical!r} "
                     f"but the record serves {live!r} — the ledger has drifted from "
                     f"the value it claims to describe; fix one or the other, never "
                     f"leave them disagreeing")


def _validate_creep_table(record: dict, name: str) -> None:
    """Structural guard on a creep block. The lookup rounds T and duration UP to
    the next tabulated cell to be conservative; that is only conservative if the
    table is non-increasing in BOTH axes. A non-monotone cell would make
    "round up" silently read a ROSIER number."""
    creep = record.get("creep")
    _require(isinstance(creep, dict), f"{name}: 'creep' must be an object")
    _require(isinstance(creep.get("basis"), str) and creep["basis"],
             f"{name}: creep.basis must be a non-empty provenance string")
    rows = creep_cells(record)  # parses/raises on any malformed key
    _require(len(rows) >= 1, f"{name}: creep.sig_allow_mpa has no temperature rows")
    widths = {tuple(h for h, _k, _v in cols) for _t, _tk, cols in rows}
    _require(len(widths) == 1,
             f"{name}: creep.sig_allow_mpa rows have different duration columns "
             f"{sorted(widths)} — every temperature tier must tabulate the same "
             f"durations or 'round the duration up' means different things per row")
    for temp_c, tkey, cols in rows:
        for a, b in zip(cols, cols[1:]):
            _require(a[2] >= b[2],
                     f"{name}: creep.sig_allow_mpa[{tkey}] rises with duration "
                     f"({a[1]}={a[2]} -> {b[1]}={b[2]}); longer must never be stronger")
    for (t_lo, k_lo, lo), (t_hi, k_hi, hi) in zip(rows, rows[1:]):
        for a, b in zip(lo, hi):
            _require(a[2] >= b[2],
                     f"{name}: creep.sig_allow_mpa rises with temperature at {a[1]} "
                     f"({k_lo}={a[2]} -> {k_hi}={b[2]}); hotter must never be stronger")


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
    _validate_conflicts(record, name)
    if record.get("creep") is not None:
        _validate_creep_table(record, name)

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
# CREEP: the ONE reader of creep.sig_allow_mpa (see CREEP SEMANTICS above)
# ---------------------------------------------------------------------------
#: Machine-matchable refusal kinds. A caller gates on these strings, never on
#: prose. `sig_allow_mpa` is 0.0 for every one of them, so a gate written as
#: `demand <= allowable` fails on its own even if the caller ignores the kind.
CREEP_REFUSAL_KINDS = (
    "creep_input_not_finite",       # T or duration is None/NaN/inf/not a number
    "creep_negative_duration",      # duration < 0 h
    "creep_temp_above_tabulated",   # T above the hottest tier — NO fallback row
    "creep_no_table",               # this material has no creep table at all
    "creep_table_malformed",        # a key in the table could not be parsed
)

#: Duration-key suffixes accepted in a creep table, in hours.
CREEP_TIME_UNITS_H = {"h": 1.0, "d": 24.0, "w": 168.0, "mo": 730.0, "y": 8760.0}

CREEP_TABLE_FIELD = "creep.sig_allow_mpa"
LEGACY_CREEP_SCALAR_FIELD = "thermal.creep_sustained_fraction"

_TEMP_KEY_RE = re.compile(r"^(-?\d+(?:\.\d+)?)\s*C$", re.IGNORECASE)
_DURATION_KEY_RE = re.compile(r"^(\d+(?:\.\d+)?)\s*(h|d|w|mo|y)$", re.IGNORECASE)


def creep_temp_key_c(key: str) -> float:
    """'23C' -> 23.0. Raises ValueError on anything else — an unparseable key
    must never sort to a default position and become a silent allowable."""
    m = _TEMP_KEY_RE.match(str(key).strip())
    if not m:
        raise ValueError(
            f"creep table temperature key {key!r} is not '<number>C' — refusing "
            f"to guess which tier it is"
        )
    return float(m.group(1))


def creep_duration_key_hours(key: str) -> float:
    """'24h' -> 24.0, '30d' -> 720.0, '1y' -> 8760.0. Raises ValueError on
    anything else (the old reader mapped an unparseable key to 0.0 hours, which
    sorts FIRST and would then be picked for every request)."""
    m = _DURATION_KEY_RE.match(str(key).strip())
    if not m:
        raise ValueError(
            f"creep table duration key {key!r} is not '<number><h|d|w|mo|y>' — "
            f"refusing to guess how long it is"
        )
    return float(m.group(1)) * CREEP_TIME_UNITS_H[m.group(2).lower()]


def creep_cells(record: dict):
    """Parse ``creep.sig_allow_mpa`` into a sorted, fully-typed structure:
    ``[(temp_c, temp_key, [(hours, dur_key, mpa), ...]), ...]`` ascending in both
    axes. Raises ValueError on a malformed table."""
    table = ((record.get("creep") or {}).get("sig_allow_mpa"))
    if not isinstance(table, dict) or not table:
        raise ValueError(f"{CREEP_TABLE_FIELD} is missing or empty")
    rows = []
    for tkey, col in table.items():
        if not isinstance(col, dict) or not col:
            raise ValueError(f"{CREEP_TABLE_FIELD}[{tkey!r}] is not a non-empty object")
        cells = []
        for dkey, val in col.items():
            if not isinstance(val, (int, float)) or isinstance(val, bool) or val < 0:
                raise ValueError(
                    f"{CREEP_TABLE_FIELD}[{tkey!r}][{dkey!r}] = {val!r} is not a "
                    f"non-negative number of MPa")
            cells.append((creep_duration_key_hours(dkey), dkey, float(val)))
        cells.sort(key=lambda c: c[0])
        rows.append((creep_temp_key_c(tkey), tkey, cells))
    rows.sort(key=lambda r: r[0])
    return rows


def _resolve_material(material, version=None):
    """Accept a material NAME ('pla', 'PLA', 'TPU') or an already-loaded record
    dict — every material-facing entry point in the campaign surface is addressed
    by name, so a name must work (cubesat F12), and the field tools already hold
    a parsed record, so a dict must keep working."""
    if isinstance(material, Material):
        return material.record, material.name, material.hash
    if isinstance(material, dict):
        meta = material.get("meta") or {}
        return material, str(meta.get("name", "?")), meta.get("hash")
    mat = get(material, version=version)
    return mat.record, mat.name, mat.hash


def _is_real(x) -> bool:
    return (isinstance(x, (int, float)) and not isinstance(x, bool)
            and math.isfinite(float(x)))


def legacy_creep_scalar(material, *, version=None) -> dict:
    """The LEGACY, time-blind ``yield x thermal.creep_sustained_fraction`` number
    — reported for VISIBILITY only. When the record carries a creep table the
    table governs and this value is ``superseded``; it is never an allowable.
    Quotes the record's own conflict-ledger entry when there is one."""
    record, name, mat_hash = _resolve_material(material, version)
    frac = dig(record, LEGACY_CREEP_SCALAR_FIELD)
    yld = dig(record, "mechanical.yield_mpa")
    has_table = isinstance((record.get("creep") or {}).get("sig_allow_mpa"), dict)
    ledger = next((c for c in (record.get("conflicts") or [])
                   if c.get("field") == LEGACY_CREEP_SCALAR_FIELD), None)
    out = {
        "material": name,
        "field": LEGACY_CREEP_SCALAR_FIELD,
        "fraction": None if frac is _MISSING else frac,
        "yield_mpa": None if yld is _MISSING else yld,
        "mpa": None,
        "superseded_by": CREEP_TABLE_FIELD if has_table else None,
        "usable_as_allowable": False,
        "note": "",
        "conflict_ledger": ledger,
    }
    if _is_real(out["fraction"]) and _is_real(out["yield_mpa"]):
        out["mpa"] = round(float(out["fraction"]) * float(out["yield_mpa"]), 6)
    if has_table:
        out["note"] = (
            f"SUPERSEDED: {name} carries {CREEP_TABLE_FIELD}, a temperature x "
            f"duration table, and the TABLE GOVERNS. The scalar is time-blind and "
            f"non-conservative at long duration (PLA: 11.0 MPa vs 2.5 MPa at "
            f"23 C / 1 y). Reported so the conflict stays visible; never gate on it."
        )
    else:
        out["note"] = (
            f"{name} has NO creep table. The scalar is a blanket rule of thumb "
            f"with no duration in it, so it cannot answer 'what may this hold for "
            f"how long' — creep_lookup REFUSES for this material rather than "
            f"returning this number as an allowable."
        )
    return out


CREEP_INTERPOLATION_FORMULA = (
    "a(T, t) = a_lo(t) + (T - T_lo)/(T_hi - T_lo) * (a_hi(t) - a_lo(t)),  "
    "a_row(t) = a[row][t_lo] + (ln t - ln t_lo)/(ln t_hi - ln t_lo) * (a[row][t_hi] - a[row][t_lo])  "
    "— linear in temperature between the bracketing rows, log-linear in duration "
    "between the bracketing columns; clamped (never extrapolated) below the coldest "
    "row and before the first column; the last column is reused beyond it; above the "
    "hottest row the lookup still REFUSES."
)


def _creep_interpolate(rows, t: float, h: float):
    """Opt-in interpolation between the bracketing cells of a parsed creep table
    (``creep_cells`` output): linear in temperature, log-linear in duration.

    Returns None when the request sits ON tabulated cells in both axes (or is
    clamped onto one — below the coldest row / before the first column /
    beyond the last column), so the caller falls through to the ordinary
    exact / rounded read and the receipt is never labelled ``interpolated``
    when nothing was interpolated. Otherwise returns a dict with the value
    and the bracketing cells. Callers guarantee t <= hottest row."""
    temps = [r[0] for r in rows]
    if t <= temps[0]:
        lo_i = hi_i = 0
    else:
        hi_i = next(i for i, tc in enumerate(temps) if tc >= t)
        lo_i = hi_i if temps[hi_i] == t else hi_i - 1

    def in_row(row):
        row_c, row_key, cells = row
        hours = [c[0] for c in cells]
        if h <= hours[0]:
            return cells[0][2], [(row_c, row_key) + cells[0]], "clamped_to_first_column"
        if h >= hours[-1]:
            match = "exact" if h == hours[-1] else "extrapolated_beyond_last_column"
            return cells[-1][2], [(row_c, row_key) + cells[-1]], match
        hi = next(i for i, hh in enumerate(hours) if hh >= h)
        if hours[hi] == h:
            return cells[hi][2], [(row_c, row_key) + cells[hi]], "exact"
        lo = hi - 1
        f = (math.log(h) - math.log(hours[lo])) / (math.log(hours[hi]) - math.log(hours[lo]))
        val = cells[lo][2] + f * (cells[hi][2] - cells[lo][2])
        return val, [(row_c, row_key) + cells[lo], (row_c, row_key) + cells[hi]], "log_linear"

    a_lo, cells_lo, dm = in_row(rows[lo_i])
    if lo_i == hi_i:
        tm = "exact" if temps[lo_i] == t else "clamped_to_coldest_row"
        value, cells = a_lo, cells_lo
    else:
        a_hi, cells_hi, dm_hi = in_row(rows[hi_i])
        g = (t - temps[lo_i]) / (temps[hi_i] - temps[lo_i])
        value, cells, tm = a_lo + g * (a_hi - a_lo), cells_lo + cells_hi, "linear"
    if tm != "linear" and dm != "log_linear":
        return None  # nothing was interpolated: the ordinary reader answers this
    return {
        "value": value, "temp_match": tm, "duration_match": dm,
        "rows_c": sorted({c[0] for c in cells}), "cols_h": sorted({c[2] for c in cells}),
        "cells": [{"temperature_bucket": c[1], "duration_bucket": c[3],
                   "row_c": c[0], "col_h": c[2], "mpa": c[4]} for c in cells],
    }


def creep_lookup(material, temp_c, hours, *, across_layer=False,
                 version=None, interpolate=False) -> dict:
    """Sustained-load (creep) allowable RECEIPT for one material at one service
    temperature and one design duration. THE single Python reader of
    ``creep.sig_allow_mpa``; with the default ``interpolate=False`` it is
    numerically identical to ``kernel_model::materials::pla::creep_allowable_mpa``
    at every point (pinned by ``tools/materials_crosslang_test.py``).

    material      material NAME (case-insensitive, aliases honored) or a record
                  dict / Material.
    temp_c        service temperature, deg C — REQUIRED, no default. The table
                  is a coarse step (PLA: 23 C and 55 C, nothing between), so the
                  temperature a margin was read at is the whole question.
    hours         design duration in hours — REQUIRED, no default.
    across_layer  True applies process.anisotropy.z_vs_xy_strength_ratio to the
                  (in-plane) tabulated cell. The caller states this; it is never
                  applied silently, and it derates the ALLOWABLE, never E.
    interpolate   False (default): the conservative BUCKET — both axes round UP
                  to the next tabulated cell (a 30 C request reads the 55 C
                  row). True (opt-in, Python only, no Rust mirror): the
                  allowable is INTERPOLATED between the bracketing cells,
                  linear in temperature and log-linear in duration
                  (``CREEP_INTERPOLATION_FORMULA``); the receipt then says
                  ``basis: "interpolated"``, ``cell_match: "interpolated"``,
                  names every bracketing cell in ``bracketing_cells`` with its
                  confidence string, carries the ``formula``, and reports the
                  bucket the default would have read as ``default_bucket`` /
                  ``default_bucket_mpa`` so the two are always side by side.
                  Interpolation NEVER extrapolates: above the hottest row it
                  still refuses, below the coldest row / before the first
                  column it clamps to that cell (and is then not labelled
                  interpolated, because nothing was). The interpolated number
                  is a model between two conservative constructions, not a
                  measurement — the LOWEST bracketing confidence governs.

    Returns a receipt. On refusal: ``known`` False, ``refused`` True,
    ``refusal_kind`` one of CREEP_REFUSAL_KINDS, ``sig_allow_mpa`` 0.0 — so a
    gate ``demand <= sig_allow_mpa`` fails loudly rather than reading a rosier
    row that the data does not support."""
    record, name, mat_hash = _resolve_material(material, version)
    ratio = float(((record.get("process") or {}).get("anisotropy") or {})
                  .get("z_vs_xy_strength_ratio", 1.0))
    factor = ratio if across_layer else 1.0
    out = {
        "material": name,
        "material_version": (record.get("meta") or {}).get("version"),
        "material_hash": mat_hash,
        "table_source": f"tools/materials/{str(name).lower()}.json#{CREEP_TABLE_FIELD}",
        "temp_c_requested": temp_c,
        "hours_requested": hours,
        "known": False,
        "refused": True,
        "refusal_kind": None,
        "sig_allow_mpa": 0.0,
        "in_plane_mpa": 0.0,
        "temperature_bucket": None,   # legacy key name, kept for existing callers
        "duration_bucket": None,      # legacy key name, kept for existing callers
        "row_used_c": None,
        "col_used_h": None,
        "cell_match": "refused",
        "temp_match": None,
        "duration_match": None,
        "interpolated": False,
        "extrapolated": False,
        "across_layer": bool(across_layer),
        "anisotropy_factor": factor,
        "z_vs_xy_strength_ratio": ratio,
        "basis": None,
        "confidence": None,
        "note": None,
        "legacy_scalar": legacy_creep_scalar(record),
    }
    if interpolate:
        # Only present when asked for, so the default receipt is byte-identical
        # to what every shipped campaign and the cross-language pin recorded.
        out["interpolation_requested"] = True

    def refuse(kind: str, note: str) -> dict:
        out["refusal_kind"] = kind
        out["note"] = note
        return out

    if not _is_real(temp_c) or not _is_real(hours):
        return refuse(
            "creep_input_not_finite",
            f"creep allowable REFUSED: temp_c={temp_c!r} hours={hours!r} — both "
            f"must be finite numbers. State the service temperature and the design "
            f"duration; a missing one is not a default, it is an unanswered question.")
    if float(hours) < 0.0:
        return refuse(
            "creep_negative_duration",
            f"creep allowable REFUSED: hours={hours!r} is negative.")

    try:
        rows = creep_cells(record)
    except ValueError as exc:
        if (record.get("creep") or {}).get("sig_allow_mpa") is None:
            legacy = out["legacy_scalar"]
            return refuse(
                "creep_no_table",
                f"creep allowable REFUSED: {name} has no {CREEP_TABLE_FIELD}. The "
                f"legacy scalar ({legacy['field']} {legacy['fraction']} x yield "
                f"{legacy['yield_mpa']} MPa = {legacy['mpa']} MPa) is time-blind and "
                f"is NOT served as an allowable — research a creep table for {name} "
                f"or design the sustained case out.")
        return refuse("creep_table_malformed",
                      f"creep allowable REFUSED: {name} {CREEP_TABLE_FIELD}: {exc}")

    t = float(temp_c)
    h = float(hours)
    hottest_c, hottest_key, _ = rows[-1]
    if t > hottest_c:
        return refuse(
            "creep_temp_above_tabulated",
            f"creep allowable REFUSED: service {t} C is above the hottest tabulated "
            f"tier ({hottest_key} = {hottest_c} C) for {name}. There is NO sustained "
            f"allowable to read — the reader does NOT fall back to the {hottest_key} "
            f"row, because no data supports one there. Either hold the part below "
            f"{hottest_c} C or state that no sustained load is defensible.")

    row_c, row_key, cells = next(r for r in rows if r[0] >= t)
    col_h, col_key, mpa = next((c for c in cells if c[0] >= h), cells[-1])
    beyond_last = h > cells[-1][0]

    if interpolate:
        interp = _creep_interpolate(rows, t, h)
        if interp is not None:
            conf_tbl = (record.get("creep") or {}).get("confidence") or {}
            for cell in interp["cells"]:
                cell["confidence"] = (conf_tbl.get(cell["temperature_bucket"], {})
                                      .get(cell["duration_bucket"])
                                      or "(no confidence string in the record for this cell)")
            val = float(interp["value"])
            t_lo, t_hi = interp["rows_c"][0], interp["rows_c"][-1]
            h_lo, h_hi = interp["cols_h"][0], interp["cols_h"][-1]
            row_keys = sorted({c["temperature_bucket"] for c in interp["cells"]},
                              key=creep_temp_key_c)
            col_keys = sorted({c["duration_bucket"] for c in interp["cells"]},
                              key=creep_duration_key_hours)
            out.update({
                "known": True,
                "refused": False,
                "refusal_kind": None,
                "in_plane_mpa": round(val, 6),
                "sig_allow_mpa": round(val * factor, 6),
                "temperature_bucket": "..".join(row_keys),
                "duration_bucket": "..".join(col_keys),
                "row_used_c": None if len(row_keys) > 1 else t_lo,
                "col_used_h": None if len(col_keys) > 1 else h_lo,
                "rows_used_c": [t_lo, t_hi],
                "cols_used_h": [h_lo, h_hi],
                "cell_match": "interpolated",
                "temp_match": ("linear_interpolated" if interp["temp_match"] == "linear"
                               else interp["temp_match"]),
                "duration_match": ("log_linear_interpolated" if interp["duration_match"] == "log_linear"
                                   else interp["duration_match"]),
                "interpolated": True,
                "extrapolated": beyond_last,
                "basis": "interpolated",
                "formula": CREEP_INTERPOLATION_FORMULA,
                "bracketing_cells": interp["cells"],
                "default_bucket": {"temperature_bucket": row_key, "duration_bucket": col_key,
                                   "row_c": row_c, "col_h": col_h, "in_plane_mpa": mpa,
                                   "sig_allow_mpa": round(mpa * factor, 6)},
                "default_bucket_mpa": round(mpa * factor, 6),
                "confidence": ("interpolated — the LOWEST bracketing confidence governs: "
                               + "; ".join(f"[{c['temperature_bucket']}][{c['duration_bucket']}] "
                                           f"{c['confidence']}" for c in interp["cells"])),
                "note": (f"INTERPOLATED (opt-in) between "
                         + ", ".join(f"[{c['temperature_bucket']}][{c['duration_bucket']}]={c['mpa']} MPa"
                                     for c in interp["cells"])
                         + f" for {t} C / {h} h -> {round(val, 6)} MPa in-plane"
                         + (f" x z/xy {ratio} (across-layer, caller-stated) = "
                            f"{round(val * factor, 6)} MPa" if across_layer else "")
                         + f"; the default conservative bucket would read [{row_key}][{col_key}] "
                           f"= {mpa} MPa. State 'interpolated' with both cells when you quote this; "
                           f"it is a model between two conservative constructions, not a measurement, "
                           f"and it has no Rust mirror."),
            })
            return out
        out["interpolation_note"] = (
            "interpolation was requested but the request sits on a tabulated cell "
            "(exact or clamped at the table edge) — the ordinary read answers it")

    temp_match = "exact" if row_c == t else "rounded_up"
    if beyond_last:
        duration_match = "extrapolated_beyond_last_column"
    elif col_h == h:
        duration_match = "exact"
    else:
        duration_match = "rounded_up"
    if beyond_last:
        cell_match = "extrapolated_beyond_last_duration"
    elif temp_match == "exact" and duration_match == "exact":
        cell_match = "exact"
    else:
        cell_match = "rounded_up_conservative"

    conf = (((record.get("creep") or {}).get("confidence") or {})
            .get(row_key, {}).get(col_key))
    out.update({
        "known": True,
        "refused": False,
        "refusal_kind": None,
        "in_plane_mpa": mpa,
        "sig_allow_mpa": round(mpa * factor, 6),
        "temperature_bucket": row_key,
        "duration_bucket": col_key,
        "row_used_c": row_c,
        "col_used_h": col_h,
        "cell_match": cell_match,
        "temp_match": temp_match,
        "duration_match": duration_match,
        "interpolated": False,
        "extrapolated": beyond_last,
        "basis": (f"{CREEP_TABLE_FIELD}[{row_key}][{col_key}] = {mpa} MPa in-plane"
                  + (f" x z/xy {ratio} (across-layer, caller-stated) = "
                     f"{round(mpa * factor, 6)} MPa" if across_layer else "")
                  + f"; service {t} C / {h} h -> temperature {temp_match}, "
                    f"duration {duration_match} (NO interpolation: the table is a "
                    f"step and both axes round UP)"),
        "confidence": conf or "(no confidence string in the record for this cell)",
        "note": ((f"read at the {row_key} row for a {t} C service temperature — "
                  f"state {row_key}, not {t} C, as the temperature this margin holds at"
                  ) if temp_match == "rounded_up" else
                 f"exact tabulated tier {row_key}"),
    })
    return out


def creep_allowable_mpa(material, temp_c, hours, *, across_layer=False,
                        version=None, interpolate=False) -> float:
    """Bare sustained (creep) allowable in MPa — the one-line gate the OPERATOR
    BRIEF and DELIVERABLE_SPEC §2 gate 8 tell campaigns to use, now reachable
    from Python. Exactly ``creep_lookup(...)["sig_allow_mpa"]``, and (with the
    default ``interpolate=False``) exactly the number
    ``kernel_model::materials::pla::creep_allowable_mpa`` returns.

    **Returns 0.0 when the lookup REFUSES** (above the hottest tabulated tier,
    non-finite/negative input, or a material with no creep table) — i.e. "no
    sustained load is defensible", so ``demand <= creep_allowable_mpa(...)``
    FAILS there. Use ``creep_lookup`` when you need to know WHY, or a receipt —
    and ALWAYS use it when ``interpolate=True``, because the bare scalar cannot
    carry the bracketing cells a quoted interpolated allowable must name."""
    return creep_lookup(material, temp_c, hours, across_layer=across_layer,
                        version=version, interpolate=interpolate)["sig_allow_mpa"]


# ---------------------------------------------------------------------------
# CLI: --rehash, --list, --show, --creep, --selftest
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


def _creep_cli(argv: list[str]) -> int:
    """--creep <MATERIAL> <TEMP_C> <HOURS> [--across-layer] [--interpolate]

    Prints the full lookup receipt as JSON on stdout. Exit 0 when an allowable
    was read, 1 when the lookup REFUSED — so a shell gate cannot mistake a
    refusal for a number, and the refusal is never silent."""
    args = [a for a in argv if a not in ("--across-layer", "--interpolate")]
    across = "--across-layer" in argv
    interp = "--interpolate" in argv
    if len(args) != 3:
        print(json.dumps({
            "refused": True, "refusal_kind": "creep_input_not_finite",
            "note": "usage: materials.py --creep <MATERIAL> <TEMP_C> <HOURS> "
                    "[--across-layer] [--interpolate]; both the service temperature "
                    "and the design duration are REQUIRED — neither has a default",
        }, indent="\t"))
        return 1
    name, t_raw, h_raw = args
    try:
        temp_c, hours = float(t_raw), float(h_raw)
    except ValueError:
        temp_c, hours = t_raw, h_raw  # let creep_lookup refuse with its own kind
    rec = creep_lookup(name, temp_c, hours, across_layer=across, interpolate=interp)
    print(json.dumps(rec, indent="\t", ensure_ascii=False))
    return 1 if rec["refused"] else 0


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
    root = Path(__file__).resolve().parents[2]  # tools/analyzers/materials.py -> repo root
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

    # 6) CREEP CONTRACT — the refusing semantic, and cell provenance on every
    #    lookup. These mirror crates/kernel-model/tests/materials_creep.rs.
    checks.append(("creep 23C/1h exact cell = 7.5 MPa, cell_match 'exact'",
                   creep_allowable_mpa("PLA", 23.0, 1.0) == 7.5
                   and creep_lookup("PLA", 23.0, 1.0)["cell_match"] == "exact"))
    checks.append(("creep 23C/1y = 2.5 MPa (the table, NOT the 11.0 MPa scalar)",
                   creep_allowable_mpa("PLA", 23.0, 8760.0) == 2.5))
    mid = creep_lookup("PLA", 25.0, 8760.0)
    checks.append(("creep 25C rounds UP to the 55C row (0.5 MPa) and SAYS so",
                   mid["sig_allow_mpa"] == 0.5 and mid["temperature_bucket"] == "55C"
                   and mid["temp_match"] == "rounded_up"
                   and mid["cell_match"] == "rounded_up_conservative"))
    hot = creep_lookup("PLA", 70.0, 24.0)
    checks.append(("creep ABOVE the hot tier REFUSES (0.0 MPa, known False, "
                   "kind creep_temp_above_tabulated) — never the 55C row",
                   hot["sig_allow_mpa"] == 0.0 and hot["known"] is False
                   and hot["refused"] is True
                   and hot["refusal_kind"] == "creep_temp_above_tabulated"))
    checks.append(("creep refuses non-finite / missing / negative inputs",
                   all(creep_lookup("PLA", t, h)["refusal_kind"] in
                       ("creep_input_not_finite", "creep_negative_duration")
                       for t, h in ((None, 24.0), (23.0, None), (float("nan"), 24.0),
                                    (23.0, float("inf")), (23.0, -5.0)))))
    checks.append(("creep refuses a material with NO table instead of serving the "
                   "time-blind 0.2-fraction scalar",
                   creep_lookup("PETG", 23.0, 8760.0)["refusal_kind"] == "creep_no_table"
                   and creep_allowable_mpa("PETG", 23.0, 8760.0) == 0.0))
    across = creep_lookup("PLA", 23.0, 8760.0, across_layer=True)
    checks.append(("across-layer creep derate is the CALLER's explicit choice: "
                   f"2.5 x 0.55 = 1.375 MPa (got {across['sig_allow_mpa']})",
                   across["sig_allow_mpa"] == 1.375
                   and across["anisotropy_factor"] == 0.55
                   and creep_lookup("PLA", 23.0, 8760.0)["anisotropy_factor"] == 1.0))
    legacy = legacy_creep_scalar("PLA")
    checks.append((f"legacy scalar {legacy['mpa']} MPa is reported as SUPERSEDED by "
                   f"the table, never usable as an allowable",
                   legacy["mpa"] == 11.0 and legacy["superseded_by"] == CREEP_TABLE_FIELD
                   and legacy["usable_as_allowable"] is False
                   and legacy["conflict_ledger"] is not None))
    checks.append(("creep table keys refuse to be guessed at (an unparseable "
                   "duration key raises instead of sorting to 0 h)",
                   _raises(lambda: creep_duration_key_hours("soon"))
                   and _raises(lambda: creep_temp_key_c("RT"))))

    # 6b) OPT-IN INTERPOLATION (2026-09-02) — labelled, bracketed, never
    #     extrapolated, and the DEFAULT receipt byte-for-byte unchanged.
    #     Hand values: 30 C / 24 h = 5.0 + 7/32 x (1.5 - 5.0) = 4.234375 (exact
    #     in binary); 23 C / 12 h = 7.5 + ln12/ln24 x (5.0 - 7.5) = 5.545261;
    #     30 C / 12 h = a23(12h) + 7/32 x (a55(12h) - a23(12h)), a55(12h) =
    #     3.0 + ln12/ln24 x (1.5 - 3.0) = 1.827157 -> 4.731925.
    i30 = creep_lookup("PLA", 30.0, 24.0, interpolate=True)
    d30 = creep_lookup("PLA", 30.0, 24.0)
    checks.append(("interpolate=True at 30C/24h = 5.0 + 7/32 x (1.5-5.0) = 4.234375 MPa, "
                   f"basis 'interpolated', both cells named (got {i30['sig_allow_mpa']})",
                   i30["sig_allow_mpa"] == 4.234375 and i30["basis"] == "interpolated"
                   and i30["cell_match"] == "interpolated" and i30["interpolated"] is True
                   and [(c["temperature_bucket"], c["duration_bucket"], c["mpa"])
                        for c in i30["bracketing_cells"]] == [("23C", "24h", 5.0), ("55C", "24h", 1.5)]
                   and i30["default_bucket_mpa"] == 1.5 and i30["formula"] == CREEP_INTERPOLATION_FORMULA
                   and i30["temp_match"] == "linear_interpolated" and i30["duration_match"] == "exact"))
    checks.append(("the DEFAULT at 30C/24h is unchanged: 1.5 MPa from the 55C row, no "
                   "interpolation key on the receipt",
                   d30["sig_allow_mpa"] == 1.5 and d30["cell_match"] == "rounded_up_conservative"
                   and "interpolation_requested" not in d30 and "bracketing_cells" not in d30))
    f12 = math.log(12.0) / math.log(24.0)
    a23_12 = 7.5 + f12 * (5.0 - 7.5)
    a55_12 = 3.0 + f12 * (1.5 - 3.0)
    i23_12 = creep_lookup("PLA", 23.0, 12.0, interpolate=True)
    i30_12 = creep_lookup("PLA", 30.0, 12.0, interpolate=True)
    checks.append((f"log-linear in duration: 23C/12h = {a23_12:.6f} MPa "
                   f"(got {i23_12['sig_allow_mpa']}), exact row, log_linear column",
                   abs(i23_12["sig_allow_mpa"] - a23_12) < 1e-6 and i23_12["temp_match"] == "exact"
                   and i23_12["duration_match"] == "log_linear_interpolated"
                   and len(i23_12["bracketing_cells"]) == 2))
    checks.append((f"bilinear (T linear x ln t) at 30C/12h = {a23_12 + 7 / 32 * (a55_12 - a23_12):.6f} MPa "
                   f"(got {i30_12['sig_allow_mpa']}), 4 bracketing cells",
                   abs(i30_12["sig_allow_mpa"] - (a23_12 + 7.0 / 32.0 * (a55_12 - a23_12))) < 1e-6
                   and len(i30_12["bracketing_cells"]) == 4
                   and i30_12["temperature_bucket"] == "23C..55C" and i30_12["duration_bucket"] == "1h..24h"))
    checks.append(("interpolated value is bracketed by the two cells (monotone table)",
                   1.5 <= i30["sig_allow_mpa"] <= 5.0 and 1.5 <= i30_12["sig_allow_mpa"] <= 7.5))
    checks.append(("interpolate=True still REFUSES above the hottest row (no extrapolation) "
                   "and reads an exact cell as exact (not labelled interpolated)",
                   creep_lookup("PLA", 70.0, 24.0, interpolate=True)["refusal_kind"]
                   == "creep_temp_above_tabulated"
                   and creep_lookup("PLA", 23.0, 8760.0, interpolate=True)["cell_match"] == "exact"
                   and "interpolation_note" in creep_lookup("PLA", 23.0, 8760.0, interpolate=True)
                   and creep_lookup("PLA", 10.0, 0.5, interpolate=True)["cell_match"]
                   == "rounded_up_conservative"))
    checks.append(("across-layer applies to the interpolated value too: 4.234375 x 0.55 = "
                   "2.32890625 -> 2.328906 at the receipt's 6-decimal rounding",
                   creep_lookup("PLA", 30.0, 24.0, interpolate=True, across_layer=True)["sig_allow_mpa"]
                   == round(4.234375 * 0.55, 6)
                   and creep_allowable_mpa("PLA", 30.0, 24.0, interpolate=True) == 4.234375))

    # 7) THE CONFLICT LEDGER IS MACHINE-CHECKED. A ledger that has drifted from
    #    the value it describes is worse than no ledger. Proven on the two live
    #    entries this repo argues about: PLA cp 1200 vs 1800 J/kgK, and the
    #    legacy creep fraction 0.2 vs the creep table.
    pla_rec = get("PLA").record
    cp_entry = next((c for c in pla_rec["conflicts"]
                     if c["field"] == "thermal.specific_heat_j_kgk"), None)
    checks.append((f"PLA cp conflict ledger is live and CHECKED: canonical "
                   f"{cp_entry and cp_entry['canonical']} == record "
                   f"{dig(pla_rec, 'thermal.specific_heat_j_kgk')} J/kgK "
                   f"(other lineage {cp_entry and cp_entry['other_value']})",
                   cp_entry is not None and cp_entry["canonical"] == 1200.0
                   and cp_entry["other_value"] == 1800.0
                   and dig(pla_rec, "thermal.specific_heat_j_kgk") == 1200.0))
    drifted = json.loads(json.dumps(pla_rec))
    drifted["thermal"]["specific_heat_j_kgk"] = 1800.0   # the OTHER lineage
    checks.append(("a record whose value drifts away from its own conflict "
                   "ledger is REFUSED at load (cp silently flipped to 1800)",
                   _raises(lambda: validate(drifted, check_hash=False))))
    stale = json.loads(json.dumps(pla_rec))
    stale["conflicts"][0]["field"] = "thermal.no_such_field"
    checks.append(("a conflict entry naming a field the record no longer serves "
                   "is REFUSED at load (stale ledger)",
                   _raises(lambda: validate(stale, check_hash=False))))
    nonmono = json.loads(json.dumps(pla_rec))
    nonmono["creep"]["sig_allow_mpa"]["55C"]["1y"] = 9.0  # hotter+longer = stronger
    checks.append(("a non-monotone creep table is REFUSED at load — without "
                   "monotonicity, 'round the request UP' is not conservative",
                   _raises(lambda: validate(nonmono, check_hash=False))))

    _report(checks)
    if not all(ok for _, ok in checks):
        print("SELFTEST FAIL")
        sys.exit(1)
    print(f"SELFTEST PASS: {len(db)} records; PETG E/nu/rho/yield match the drive "
          f"FEA verbatim, gate basis 28.2 MPa reproduced; TPU basketball numbers "
          f"match; anisotropy derate unified; creep table governs and REFUSES "
          f"above the hot tier.")


def _raises(fn) -> bool:
    try:
        fn()
    except ValueError:
        return True
    return False


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
    elif cmd == "--creep":
        raise SystemExit(_creep_cli(argv[1:]))
    elif cmd == "--selftest":
        _selftest()
    else:
        raise SystemExit(f"unknown command {cmd!r}; see --help")


if __name__ == "__main__":
    main(sys.argv[1:])
