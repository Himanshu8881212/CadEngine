#!/usr/bin/env python3
"""Cross-language materials consistency test (Unit 3).

Proves the ONE material record (tools/materials/*.json, SI kg/m^3) is consumed
consistently by BOTH sides of the seam.

PART 1 — MASS / UNITS
  * Python (FEA / mass): density_kg_m3, E, nu  — SI.
  * Rust  (BOM / mass, format.rs): density_g_cm3 — g/cm^3.
The geometry VOLUME comes from the Rust engine (kernel-api), so this genuinely
crosses the language boundary. If the two mass formulas — Rust's
`density_g_cm3 * volume_cm3` and Python's SI `density_kg_m3 * volume_m3 * 1000`
— disagree for the same key, the record's units drifted; the named conversion
`kg_m3_to_g_cm3` and the load-time range assertion exist to make that impossible.

PART 2 — THE CREEP PIN (portfolio theme T14, the real deliverable)
Two readers of ONE table, `tools/materials/pla.json#creep.sig_allow_mpa`:
  * Python  `materials.creep_lookup` / `materials.creep_allowable_mpa`
  * Rust    `kernel_model::materials::pla::creep_lookup` / `creep_allowable_mpa`
They once DISAGREED above the hot tier, and the Python one — the only one a
campaign could actually reach — was the NON-CONSERVATIVE side: at 70 C and even
at 120 C it fell back to the last tabulated row and returned 1.5 MPa where the
Rust contract refuses outright, flagging only `extrapolated: True`, a field a
gate can easily not read. Sustained load is the governing failure mode in at
least five of the ten campaigns, so that divergence was a live safety hazard.

`tools/materials/creep_crosslang_vectors.json` is the contract BOTH readers must
satisfy — the allowable, the exact CELL it was read at, and the refusal kind, at
every tier boundary, on BOTH sides of every boundary, above the hot tier, beyond
the last duration column, and for every non-finite/negative input. This file
checks the Python side against it; `crates/kernel-model/tests/materials_creep_crosslang.rs`
checks the Rust side against the SAME file. A divergence of that class can now
only land by breaking one of the two tests.

The vectors are a CONTRACT, not a cache: never regenerate them to make a failing
test pass. `sig_allow_mpa: 0.0` with a `refusal_kind` is an ANSWER ("no sustained
load is defensible"), not a gap.

Run:  python3 tools/materials_crosslang_test.py   (exit 0 on pass, nonzero on fail)
      --no-rust   skip only the live `cargo test` leg (the vector pin, the
                  hand-written doctrine pins and the mass check still run, and
                  the skip is PRINTED — it is never silent)
"""
import json
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # tools/
REPO = os.path.dirname(HERE)
ENGINE = os.path.join(REPO, "target", "release", "kernel-api")
VECTORS = os.path.join(HERE, "materials", "creep_crosslang_vectors.json")
sys.path.insert(0, HERE)
import _layout  # noqa: E402
_layout.add_import_paths()
import materials  # noqa: E402

#: JSON has no literal for these, and they are exactly the inputs that must
#: REFUSE rather than default to a cell.
_SPECIAL_AXIS = {"nan": float("nan"), "inf": float("inf"), "-inf": float("-inf")}


def _axis(v):
    if isinstance(v, str):
        if v not in _SPECIAL_AXIS:
            raise ValueError(f"probe axis string {v!r} is not one of nan/inf/-inf")
        return _SPECIAL_AXIS[v]
    if isinstance(v, (int, float)) and not isinstance(v, bool):
        return float(v)
    raise ValueError(f"probe axis {v!r} is neither a number nor a special-value string")


def creep_vector_checks():
    """Every probe in the shared vectors file, replayed through the PYTHON
    reader. Returns (label, passed) rows."""
    with open(VECTORS, encoding="utf-8") as f:
        doc = json.load(f)
    probes = doc["probes"]
    mismatches = []
    for i, p in enumerate(probes):
        t, h = _axis(p["temp_c"]), _axis(p["hours"])
        got = materials.creep_lookup("PLA", t, h, across_layer=p["across_layer"])
        want = {
            "sig_allow_mpa": p["sig_allow_mpa"], "in_plane_mpa": p["in_plane_mpa"],
            "row_used_c": p["row_used_c"], "col_used_h": p["col_used_h"],
            "cell_match": p["cell_match"], "refusal_kind": p["refusal_kind"],
        }
        # 1e-12 absolute ONLY because the anisotropy product (0.5 x 0.55) is not
        # exact in binary; the tabulated cells themselves compare exactly.
        ok = (abs(got["sig_allow_mpa"] - want["sig_allow_mpa"]) < 1e-12
              and got["in_plane_mpa"] == want["in_plane_mpa"]
              and got["row_used_c"] == want["row_used_c"]
              and got["col_used_h"] == want["col_used_h"]
              and got["cell_match"] == want["cell_match"]
              and got["refusal_kind"] == want["refusal_kind"])
        # The bare scalar entry point must never disagree with the receipt.
        bare = materials.creep_allowable_mpa("PLA", t, h,
                                             across_layer=p["across_layer"])
        ok = ok and bare == got["sig_allow_mpa"]
        if not ok:
            mismatches.append(
                f"probe[{i}] T={p['temp_c']} h={p['hours']} across={p['across_layer']}: "
                f"python={ {k: got[k] for k in want} } contract={want} bare={bare}")

    rows = [(f"creep: Python matches the cross-language contract at all "
             f"{len(probes)} probes (tier boundaries, both sides of each, above "
             f"the hot tier, beyond the last duration column, non-finite inputs)"
             + ("" if not mismatches else "\n      " + "\n      ".join(mismatches[:8])),
             not mismatches)]

    # Hand-written (NOT generated from the reader): the doctrine points, read
    # straight off tools/materials/pla.json. If the vectors file were ever
    # regenerated from a broken reader, these would still fail.
    above_hot = [55.000001, 56.0, 70.0, 120.0, 1.0e6]
    rows.append((
        "creep: ABOVE the hot tier (55.000001/56/70/120/1e6 C) Python REFUSES "
        "with 0.0 MPa and kind creep_temp_above_tabulated — it does NOT fall "
        "back to the 55 C row (this is the exact bug the pin exists to prevent)",
        all(materials.creep_allowable_mpa("PLA", t, 24.0) == 0.0
            and materials.creep_lookup("PLA", t, 24.0)["refusal_kind"]
            == "creep_temp_above_tabulated" for t in above_hot)))
    rows.append((
        "creep: non-finite / negative inputs REFUSE instead of defaulting to a cell",
        all(materials.creep_lookup("PLA", t, h)["refused"] for t, h in (
            (float("nan"), 24.0), (23.0, float("nan")), (23.0, float("inf")),
            (float("inf"), 24.0), (None, 24.0), (23.0, None), (23.0, -1.0)))))
    rt = materials.creep_lookup("PLA", 23.0, 8760.0)
    rows.append((
        f"creep: 23 C / 1 y is the table's 2.5 MPa read at cell [23C][1y] "
        f"(got {rt['sig_allow_mpa']} at [{rt['temperature_bucket']}]"
        f"[{rt['duration_bucket']}], {rt['cell_match']})",
        rt["sig_allow_mpa"] == 2.5 and rt["temperature_bucket"] == "23C"
        and rt["duration_bucket"] == "1y" and rt["cell_match"] == "exact"))
    a25 = materials.creep_lookup("PLA", 25.0, 8760.0)
    rows.append((
        f"creep: a 25 C declared ambient reads the 55 C row (0.5 MPa) and the "
        f"receipt SAYS so (row_used_c={a25['row_used_c']}, "
        f"cell_match={a25['cell_match']}) — the step is a value, not prose",
        a25["sig_allow_mpa"] == 0.5 and a25["row_used_c"] == 55.0
        and a25["cell_match"] == "rounded_up_conservative"))
    across = materials.creep_lookup("PLA", 23.0, 8760.0, across_layer=True)
    rows.append((
        f"creep: the x0.55 across-layer derate is the CALLER's explicit choice "
        f"and is applied to the ALLOWABLE, never to E "
        f"(2.5 -> {across['sig_allow_mpa']} MPa, factor "
        f"{across['anisotropy_factor']}; in-plane default stays "
        f"{rt['anisotropy_factor']})",
        abs(across["sig_allow_mpa"] - 2.5 * 0.55) < 1e-12
        and across["in_plane_mpa"] == 2.5
        and across["anisotropy_factor"] == 0.55 and rt["anisotropy_factor"] == 1.0))
    rows.append((
        "creep: every tabulated cell in tools/materials/pla.json is reachable "
        "EXACTLY, and each equals the value in the record (no reader-side table)",
        all(materials.creep_allowable_mpa("PLA", t_c, h) == mpa
            for t_c, _tk, cells in materials.creep_cells(materials.get("PLA").record)
            for h, _dk, mpa in cells)))
    return rows


def rust_leg_check():
    """Run the RUST half of the pin against the SAME vectors file. A missing
    cargo is reported as a FAILURE, not a skip — an unproven cross-language pin
    is exactly the silence this test exists to remove. `--no-rust` opts out
    loudly and on the record."""
    if "--no-rust" in sys.argv:
        return ("live Rust leg SKIPPED by --no-rust — the vector pin above still "
                "proves the Python side against the shared contract, but the Rust "
                "side is UNPROVEN in this run; run `cargo test -p kernel-model "
                "--release --test materials_creep_crosslang` to close it", True)
    try:
        out = subprocess.run(
            ["cargo", "test", "-p", "kernel-model", "--release",
             "--test", "materials_creep_crosslang"],
            cwd=REPO, capture_output=True, text=True)
    except FileNotFoundError:
        return ("live Rust leg: `cargo` not found — the cross-language pin CANNOT "
                "be proven in this environment (pass --no-rust to acknowledge)", False)
    tail = (out.stdout + out.stderr).strip().splitlines()[-6:]
    return (f"live Rust leg: kernel-model reads the same vectors and agrees "
            f"(cargo exit {out.returncode})" + ("" if out.returncode == 0
                                                else "\n      " + "\n      ".join(tail)),
            out.returncode == 0)


def engine_volume_mm3(program):
    """Exact volume of a box, computed by the RUST engine (cross-language half)."""
    import tempfile
    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
        json.dump(program, f)
        path = f.name
    out = subprocess.run([ENGINE, "run", path, "--out-dir", tempfile.gettempdir()],
                         capture_output=True, text=True)
    os.unlink(path)
    rep = json.loads(out.stdout)
    for op in rep["ops"]:
        if op["id"] == "v":
            return float(op["measures"]["exact_volume"])
    raise RuntimeError(f"engine did not return a volume: {out.stdout[:400]}")


def main() -> int:
    key = "PETG"
    mat = materials.get(key)
    # A box of a known size; the RUST engine reports its exact volume.
    program = {"ops": [
        {"id": "b", "op": "box", "min": [0, 0, 0], "max": [100, 50, 20]},
        {"id": "v", "op": "exact_volume", "in": "b"},
    ]}
    vol_mm3 = engine_volume_mm3(program)
    vol_cm3 = vol_mm3 / 1000.0
    vol_m3 = vol_mm3 / 1e9

    # Rust BOM formula (format.rs): unit_mass_g = density_g_cm3 * volume_cm3.
    mass_rust_g = mat.density_g_cm3 * vol_cm3
    # Python SI formula: kg = density_kg_m3 * volume_m3  ->  * 1000 = grams.
    mass_py_g = mat.density_kg_m3 * vol_m3 * 1000.0

    checks = [
        (f"named conversion exact: {mat.density_kg_m3} kg/m^3 -> {mat.density_g_cm3} g/cm^3",
         abs(mat.density_g_cm3 - mat.density_kg_m3 / 1000.0) < 1e-12),
        (f"same record, same mass across languages: Rust {mass_rust_g:.4f} g == "
         f"Python {mass_py_g:.4f} g (engine volume {vol_cm3:.3f} cm^3)",
         abs(mass_rust_g - mass_py_g) < 1e-6),
        (f"E/rho for '{key}' come from the record: E={mat.youngs_modulus_pa:.3g} Pa, "
         f"rho={mat.density_kg_m3} kg/m^3 (hash {mat.hash[:16]}...)",
         mat.youngs_modulus_pa > 0 and mat.density_kg_m3 > 0),
        # the range assertion must reject a g/cm^3 value pasted as kg/m^3
        ("range assertion catches a kg/m^3 <-> g/cm^3 mixup (1.27 rejected as kg/m^3)",
         _rejects(1.27)),
    ]
    checks += creep_vector_checks()
    checks.append(rust_leg_check())
    ok = True
    for label, passed in checks:
        print(("  PASS: " if passed else "  FAIL: ") + label)
        ok = ok and passed
    print("CROSS-LANGUAGE MATERIALS TEST:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def _rejects(kg_m3: float) -> bool:
    try:
        materials._assert_density_range(kg_m3, "mixup_probe")
        return False
    except ValueError:
        return True


if __name__ == "__main__":
    raise SystemExit(main())
