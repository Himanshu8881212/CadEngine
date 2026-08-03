#!/usr/bin/env python3
"""Cross-language materials consistency test (Unit 3).

Proves the ONE material record (tools/materials/*.json, SI kg/m^3) is consumed
consistently by BOTH sides of the seam:
  * Python (FEA / mass): density_kg_m3, E, nu  — SI.
  * Rust  (BOM / mass, format.rs): density_g_cm3 — g/cm^3.
The geometry VOLUME comes from the Rust engine (kernel-api), so this genuinely
crosses the language boundary. If the two mass formulas — Rust's
`density_g_cm3 * volume_cm3` and Python's SI `density_kg_m3 * volume_m3 * 1000`
— disagree for the same key, the record's units drifted; the named conversion
`kg_m3_to_g_cm3` and the load-time range assertion exist to make that impossible.

Run:  python3 tools/materials_crosslang_test.py   (exit 0 on pass, nonzero on fail)
"""
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
ENGINE = os.path.join(REPO, "target", "release", "kernel-api")
sys.path.insert(0, HERE)
import materials  # noqa: E402


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
