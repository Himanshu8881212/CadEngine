#!/usr/bin/env python3
"""balance_check.py — rotating-assembly static/couple balance over the LMCAD engine.

Runs `mass_properties` (with the full `inertia_tensor` measure) on each part,
combines them with real densities, and reports the imbalance of the assembly
about a spin axis: CG offset, static imbalance, couple (product-of-inertia)
terms, and the estimated 1x-rev wobble force at speed.

Usage:  python3 balance_check.py <job.json> [--out PATH]
Persistence + exit codes: the shared contract in tools/_receipt.py.
Optional `program_dir` (else `out_dir`, else the job file's own directory) is
where each part's measurement program is materialised, and therefore the root
its relative `import_step`/`load_part` paths resolve against — never a system
temp dir.

Job JSON (argv[1]) — one of the two geometry forms:
{
  # form 1: one merged solid
  "ops": [ ... work-order ops ... ], "solid": "<op id>", "density_kg_m3": 1270,

  # form 2: per-part list (poses already baked into each part's ops)
  "parts": [{"name": "rotor", "ops": [...], "solid": "<id>", "density_kg_m3": 1270}, ...],

  "spin_axis": {"point": [0,0,0], "dir": [0,0,1]},
  "spin_rpm": 3000                                  # optional
}

Receipts (LAST stdout line, logging to stderr):
{
  "ok": true,                       # measurement succeeded (this tool measures, it
                                    #  does not impose a balance grade)
  "mass_g": ...,
  "cg_mm": [x,y,z],
  "cg_offset_mm": ...,              # perpendicular distance CG <-> spin axis
  "static_imbalance_g_mm": ...,     # mass_g * cg_offset_mm (single-plane U = m*r)
  "couple_terms": {                 # inertia-tensor entries about the axis frame
    "I_uw_g_mm2": ..., "I_vw_g_mm2": ..., "magnitude_g_mm2": ...,
    "frame": "u,v span the plane normal to the axis; w = axis dir; tensor taken "
             "about spin_axis.point; entries use the dynamics convention "
             "I_uw = -integral(u*w dm) — zero for a dynamically balanced rotor"
  },
  "est_wobble_force_N_at_rpm": ..., # only when spin_rpm given
  "formula": "F = m * r * omega^2, omega = 2*pi*rpm/60 (m in kg, r = cg_offset in m)",
  "per_part": [{"name", "mass_g", "com_mm", "volume_mm3"}]
}

Unit trail (stated so the receipt is auditable): the engine returns volume in
mm^3, CoM in mm, and the unit-density inertia tensor about the CoM in mm^5.
mass = density * volume * 1e-9 [kg]; I[kg*m^2] = tensor * density * 1e-15;
couple terms are reported in g*mm^2 (I[kg*m^2] * 1e9). Parts are combined via
the parallel-axis theorem to spin_axis.point. Provenance: `mass_properties` is
analytic for planar/cylinder/sphere/cone-faced parts (pi-exact volumes).
"""
import json

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # tools/: the shared contracts + the layout map
import _layout  # noqa: E402
_layout.add_import_paths()  # tools/, tools/analyzers, tools/publish — sibling-style imports keep working after the 2026-09-02 move
import param_optimize  # call_engine — the one-shot engine pattern
import _receipt
from _receipt import Refusal

MP_ID = "__balance_mp"


def log(msg):
    print(msg, file=sys.stderr, flush=True)


def v_sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def v_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def v_cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def v_norm(a):
    n = math.sqrt(v_dot(a, a))
    if n < 1e-12:
        raise Refusal("degenerate_axis", "spin_axis.dir must be a non-zero vector")
    return [x / n for x in a]


def measure_part(part, program_dir=None):
    """Run mass_properties on one part -> (volume_mm3, com_mm, tensor_mm5)."""
    ops = list(part["ops"]) + [{"id": MP_ID, "op": "mass_properties", "in": part["solid"]}]
    report = param_optimize.call_engine({"ops": ops}, program_dir=program_dir)
    if not report.get("ok"):
        errs = [o.get("error") for o in report.get("ops", []) if o.get("error")]
        raise Refusal("part_program_failed",
                      f"part '{part.get('name', part['solid'])}' failed: {errs[:1]}")
    m = next(o["measures"] for o in report["ops"] if o["id"] == MP_ID)
    return float(m["volume"]), list(m["center_of_mass"]), [list(r) for r in m["inertia_tensor"]]


def build(job, job_path=None):
    pdir = param_optimize.station_dir(job, job_path)
    if "parts" in job:
        parts = job["parts"]
    else:
        parts = [{"name": job.get("name", job["solid"]), "ops": job["ops"],
                  "solid": job["solid"], "density_kg_m3": job["density_kg_m3"]}]
    if not isinstance(job.get("spin_axis"), dict) or "point" not in job["spin_axis"] \
            or "dir" not in job["spin_axis"]:
        raise Refusal("missing_spin_axis", "job needs `spin_axis` {point: [x,y,z], dir: [x,y,z]}")
    axis = job["spin_axis"]
    p0 = [float(x) for x in axis["point"]]        # mm
    w = v_norm([float(x) for x in axis["dir"]])   # unit axis dir

    # An axis-normal (u, v, w) right-handed frame; u anchored to the least-aligned
    # world axis so the frame is deterministic.
    seed = min(([1, 0, 0], [0, 1, 0], [0, 0, 1]), key=lambda e: abs(v_dot(e, w)))
    u = v_norm(v_cross(seed, w))
    v = v_cross(w, u)

    total_mass = 0.0        # kg
    moment = [0.0, 0.0, 0.0]  # kg*mm (mass-weighted CoM accumulator)
    i_axis = [[0.0] * 3 for _ in range(3)]  # kg*m^2 about p0, world axes
    per_part = []
    for part in parts:
        vol, com, tensor = measure_part(part, pdir)
        rho = float(part["density_kg_m3"])
        mass = rho * vol * 1e-9  # mm^3 * kg/m^3 * 1e-9 = kg
        total_mass += mass
        for k in range(3):
            moment[k] += mass * com[k]
        d = [x * 1e-3 for x in v_sub(com, p0)]   # CoM offset from axis point, meters
        d2 = v_dot(d, d)
        for i in range(3):
            for jx in range(3):
                kron = 1.0 if i == jx else 0.0
                # unit-density mm^5 -> kg*m^2, then parallel-axis CoM -> p0
                i_axis[i][jx] += tensor[i][jx] * rho * 1e-15 \
                    + mass * (d2 * kron - d[i] * d[jx])
        per_part.append({"name": part.get("name", part["solid"]),
                         "mass_g": round(mass * 1e3, 6),
                         "com_mm": [round(x, 6) for x in com],
                         "volume_mm3": round(vol, 6)})
        log(f"part {per_part[-1]['name']}: V={vol:.4f} mm^3, m={mass * 1e3:.4f} g, com={com}")

    if total_mass <= 0:
        raise Refusal("zero_mass", "assembly has zero mass — check densities/volumes")
    cg = [m / total_mass for m in moment]  # mm
    r_vec = v_sub(cg, p0)
    r_perp = v_sub(r_vec, [v_dot(r_vec, w) * c for c in w])
    cg_offset_mm = math.sqrt(v_dot(r_perp, r_perp))
    mass_g = total_mass * 1e3
    static_imbalance = mass_g * cg_offset_mm  # g*mm

    def sandwich(a, b):  # a^T * I_axis * b, kg*m^2
        return sum(a[i] * i_axis[i][jx] * b[jx] for i in range(3) for jx in range(3))

    i_uw = sandwich(u, w) * 1e9  # kg*m^2 -> g*mm^2
    i_vw = sandwich(v, w) * 1e9

    receipt = {
        "ok": True,
        "mass_g": round(mass_g, 6),
        "cg_mm": [round(x, 6) for x in cg],
        "cg_offset_mm": round(cg_offset_mm, 6),
        "static_imbalance_g_mm": round(static_imbalance, 6),
        "couple_terms": {
            "I_uw_g_mm2": round(i_uw, 6),
            "I_vw_g_mm2": round(i_vw, 6),
            "magnitude_g_mm2": round(math.hypot(i_uw, i_vw), 6),
            "frame": "u,v normal to axis, w = axis dir, about spin_axis.point; "
                     "dynamics convention I_uw = -integral(u*w dm); ~0 when "
                     "dynamically balanced",
        },
        "formula": "F = m * r * omega^2, omega = 2*pi*rpm/60 (m in kg, r = cg_offset in m)",
        "per_part": per_part,
    }
    rpm = job.get("spin_rpm")
    if rpm is not None:
        omega = 2.0 * math.pi * float(rpm) / 60.0
        receipt["spin_rpm"] = float(rpm)
        receipt["est_wobble_force_N_at_rpm"] = round(
            total_mass * cg_offset_mm * 1e-3 * omega * omega, 6)
    return receipt


def main():
    job_path, _ = _receipt.parse_argv()
    job, out = _receipt.load_job()
    _receipt.finish(build(job, job_path), job=job, tool="balance_check", out=out,
                    use_out_dir_default=True)


if __name__ == "__main__":
    _receipt.run_cli("balance_check", main)
