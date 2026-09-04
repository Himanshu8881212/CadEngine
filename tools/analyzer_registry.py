#!/usr/bin/env python3
"""analyzer_registry.py — the graduation ledger for every analyzer in tools/.

Enumerates each analysis surface the engine exposes, its committed TIER, whether
it carries a manifest (governing equations + assumptions) and a validation pin
(a check against independent ground truth), and computes the SYSTEM HEALTH
METRIC: the fraction of the analysis surface that sits BELOW the validated line.

Tiers (honest, non-inflatable — full definitions in docs/ANALYSIS_TIERS.md):

  Validated    manifest present AND >= 1 validation pin present; the pin checks
               the result against a closed-form / measured ground truth with a
               documented error band. fea / fea_tet / modal / buckling and BOTH
               optimizers (ace_optimize, param_optimize) qualify as of
               2026-07-17; the three rules/bookkeeping engines tolerance_stack,
               production_check and production_dossier qualify as of 2026-09-02
               (pinned to hand-derived arithmetic — their `kind` still says
               rules_engine / reporting, so the tier never reads as physics).
  Demonstrated runs end-to-end on real geometry through the engine/solver and
               emits a receipt with a built-in self-check or sanity gate, but is
               NOT pinned to independent ground truth.
  Cataloged    a deterministic rules / arithmetic engine over published tables
               or standard formulas — correct by construction relative to its
               cited sources, but neither a physics simulation nor pinned.

The registry NEVER inflates: a `Validated` claim that lacks a present manifest+pin
is downgraded and recorded as a violation (this is what the CI gate fails on).

Usage:
  python3 analyzer_registry.py            # human table + health metric
  python3 analyzer_registry.py --json     # machine-readable resolution
  python3 analyzer_registry.py --check    # CI gate: exit 1 on any violation
  python3 analyzer_registry.py --demo     # read-only stamp() demo on one analyzer
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _layout  # noqa: E402  (the tools/ directory map — analyzers/, publish/, validation/, tests/)
import provenance  # noqa: E402  (sibling module)

TOOLS = Path(__file__).resolve().parent
MANIFESTS = TOOLS / "manifests"

# File paths in the registry are RELATIVE TO tools/ and name the REAL file
# (tools/analyzers/x.py, tools/validation/x_validation.py, tools/tests/test_x.py)
# — never the forwarding shim left at the old flat path. `_path()` resolves a
# bare basename through tools/_layout.find_tool so a row written before the
# 2026-09-02 re-organisation (or a derived manifest's `model_file`) still
# resolves to the real file, and a shim is never mistaken for a surface.
def _path(rel: str) -> Path:
    if "/" in rel:
        return TOOLS / rel
    try:
        return _layout.find_tool(rel)
    except FileNotFoundError:
        return TOOLS / rel  # reported missing by the resolver, never silently ok

VALIDATED = "Validated"
DEMONSTRATED = "Demonstrated"
CATALOGED = "Cataloged"

# ---------------------------------------------------------------------------
# THE REGISTRY. One row per analysis surface. `claimed_tier` is what the surface
# asserts by character; the resolver ENFORCES that Validated requires a present
# manifest + pin, so the number the registry reports is never inflated.
#
# `pins` are expected tools/*_validation.py paths. A pin marked pending (a
# parallel agent owns it) is listed but does not gate — the primary pin does.
# `manifest` is the expected tools/manifests/<name>.manifest.json, or None.
# ---------------------------------------------------------------------------
def entry(name, file, kind, claimed_tier, pins, manifest, rationale,
          gate_suite=None, tier_reason=None):
    """One registry row.

    `gate_suite` is a benchmark/gate file that re-proves the analyzer's own
    claims on every run (tools/test_*.py). It is NOT the same thing as a
    validation `pin` and must never be confused with one — `tools/solvers/
    README.md` calls three solvers "green", which is a GATE-SUITE status, and a
    campaign that read "green" as a TIER would over-claim. `tier_reason` states
    in one sentence why this row sits where it sits and what would move it up,
    so a campaign can QUERY the tier (`--tier <name>`) instead of guessing."""
    return {
        "name": name,
        "file": file,
        "kind": kind,
        "claimed_tier": claimed_tier,
        "pins": list(pins),
        "manifest": manifest,
        "rationale": rationale,
        "gate_suite": gate_suite,
        "tier_reason": tier_reason,
    }


REGISTRY = [
    # --- Validated: physics solvers with closed-form pins + manifests ---------
    entry("ace_fea", "analyzers/ace_fea_runner.py", "physics_solver", VALIDATED,
          ["validation/ace_fea_validation.py", "validation/ace_fea_kt_validation.py"],
          "manifests/ace_fea.manifest.json",
          "hex8 linear-elastic FEA; pinned to Euler-Bernoulli+shear cantilever "
          "(-11.2%/-5.9% converging). Kt pin pending from a parallel agent."),
    entry("ace_fea_tet", "analyzers/ace_fea_tet_runner.py", "physics_solver", VALIDATED,
          ["validation/ace_fea_kt_tet_validation.py"],
          "manifests/ace_fea_tet.manifest.json",
          "body-fitted tet10 linear-elastic FEA (gmsh conforming mesh); pinned "
          "to the Peterson/Pilkey stepped-bar fillet Kt=1.667 — measured Kt "
          "1.545/1.610/1.652 CONVERGING from below under refinement (the voxel "
          "hex8 path scatters -6..+44% and does NOT converge). Surface stress is "
          "nodal-averaged recovery (not SPR); robustness bounded to gmsh-meshable "
          "watertight geometry; pin ladder stops at elem ~r/2.4 for runtime."),
    entry("ace_modal", "analyzers/ace_modal_runner.py", "physics_solver", VALIDATED,
          ["validation/ace_modal_validation.py"],
          "manifests/ace_modal.manifest.json",
          "hex8 free-vibration modal; pinned to Euler-Bernoulli cantilever "
          "first bending frequency (+4.0%/+0.9% converging from above).",
          gate_suite="tests/test_ace_modal_buckling.py"),
    entry("ace_buckling", "analyzers/ace_buckling_runner.py", "physics_solver", VALIDATED,
          ["validation/ace_buckling_validation.py"],
          "manifests/ace_buckling.manifest.json",
          "hex8 linear buckling; pinned to the Euler clamped-free column "
          "(+7.3%/+3.0% converging from above).",
          gate_suite="tests/test_ace_modal_buckling.py"),

    # --- Demonstrated: run end-to-end with a self-check, but not pinned -------
    entry("ace_optimize", "analyzers/ace_optimize_runner.py", "optimizer", VALIDATED,
          ["validation/ace_optimize_validation.py"],
          "manifests/ace_optimize.manifest.json",
          "SIMP/OC over the VALIDATED reference FEA; pinned via exact "
          "inequalities (no closed-form optimal topology exists): OC descent "
          "<=0.9x (measured 0.15x), material-removal monotonicity vs the "
          "solid beam (deflection ratio 1.58 >= 1), volume honesty +/-0.02 "
          "(measured exact), watertight STL gate."),
    # --- Demonstrated: benchmark-GATED against closed form, but no manifest ---
    # These three were "analyzer-shaped tools not in the registry" (a catalogue
    # -drift WARNING) while tools/solvers/README.md called them GREEN. "Green"
    # is a gate-suite status, not a tier, and the gap between the two made it
    # easy to quote a Validated-sounding tier for an unregistered surface. They
    # are registered HERE, at the tier the evidence supports, with the reason
    # machine-readable (`--tier <name>`) so nothing has to be inferred.
    entry("ace_thermal", "analyzers/ace_thermal_runner.py", "physics_solver", DEMONSTRATED,
          [], None,
          "finite-volume conduction (steady + transient). Gate suite re-derives "
          "the 1-D slab, the ln-profile cylinder wall, a Robin-cooled slab and "
          "the semi-infinite erfc transient from closed form, asserts 2nd-order "
          "convergence and an energy balance, and pins 5 negative controls.",
          gate_suite="tests/test_ace_thermal.py",
          tier_reason="Demonstrated, NOT Validated: the evidence (closed-form gates + "
                      "convergence order) is pin-grade, but there is no "
                      "tools/manifests/ace_thermal.manifest.json and no committed "
                      "tools/ace_thermal_validation.py, and this registry's rule is "
                      "that Validated requires BOTH. tools/solvers/thermal.md is prose, "
                      "not a machine-readable manifest. To move to Validated: commit "
                      "the manifest + name the gate suite as the pin."),
    entry("ace_contact", "analyzers/ace_contact_runner.py", "physics_solver", DEMONSTRATED,
          [], None,
          "geometrically-nonlinear planar beam + rigid-obstacle penalty contact, "
          "Newton-Raphson. Gate suite pins the linear limit (PL^3/3EI), the exact "
          "elastica at alpha=3, penalty/statics identities, the reaction at a "
          "prescribed tip displacement (3EId/L^3), and refuses a non-converged "
          "iterate.",
          gate_suite="tests/test_ace_contact_fatigue.py",
          tier_reason="Demonstrated, NOT Validated: no manifest and no committed "
                      "validation pin file, per the same rule as ace_thermal. Note "
                      "additionally that curve row 0 is the UN-EQUILIBRATED initial "
                      "state — the receipt labels it and every receipt statistic "
                      "excludes it."),
    entry("ace_fatigue", "analyzers/ace_fatigue_runner.py", "rules_engine", CATALOGED,
          [], None,
          "stress-life: Basquin S-N + mean-stress correction + Palmgren-Miner, "
          "over a cited printed-polymer S-N registry. Arithmetic is pinned exactly "
          "by the gate suite; the DATA is the limit, and the runner REFUSES any "
          "material without credible printed S-N data.",
          gate_suite="tests/test_ace_contact_fatigue.py",
          tier_reason="Cataloged, deliberately BELOW Demonstrated: this is deterministic "
                      "arithmetic over published tables, not a simulation, and Miner is "
                      "explicitly NOT validated for printed polymers (no "
                      "variable-amplitude printed dataset exists). Screening only. The "
                      "gate suite proves the arithmetic, which is not the same as "
                      "proving the life."),
    entry("graded_infill", "analyzers/graded_infill_runner.py", "geometry_synthesis", DEMONSTRATED,
          [], None,
          "Stress-graded gyroid infill from a prior ace_fea field; ships a "
          "--selftest and a support-necessity audit, but no ground-truth pin "
          "on the graded stiffness."),
    entry("param_optimize", "analyzers/param_optimize.py", "optimizer", VALIDATED,
          ["validation/param_optimize_validation.py"],
          "manifests/param_optimize.manifest.json",
          "Nelder-Mead + targets/robust/multi-start over ANY receipted "
          "analyzer; pinned to analytic known-optimum problems (paraboloid "
          "argmin, cubic-root target, active constraint cap) plus a "
          "byte-identical determinism pin. Pin 3 caught + fixed a real "
          "ulp-scale infeasible-selection defect (feasibility-first "
          "selection)."),
    entry("air_topology_audit", "analyzers/air_topology_audit.py", "geometry_audit", DEMONSTRATED,
          [], None,
          "Voxel flood-fill connectivity gate for internal-air channels "
          "(the TL-91 defect gate); deterministic, defect-taught, but not "
          "pinned against a measured flow/acoustic reference."),
    entry("sweep_check", "analyzers/sweep_check.py", "geometry_audit", DEMONSTRATED,
          [], None,
          "Parameter-sweep interference check over the engine's clearance "
          "measures; deterministic and engine-backed, no ground-truth pin."),
    entry("balance_check", "analyzers/balance_check.py", "geometry_audit", DEMONSTRATED,
          [], None,
          "Rotating-assembly static/couple balance from the engine's "
          "mass_properties inertia tensor + rigid-body arithmetic; measures, "
          "does not impose a balance grade, not pinned to a measured rotor."),

    # --- Cataloged: deterministic rules/arithmetic over cited tables ----------
    entry("joint_check", "analyzers/joint_check.py", "rules_engine", CATALOGED,
          [], None,
          "Fastener/insert capacity rules over published-typical capacity "
          "tables; no FEA, no engine calls, not validated against pull tests."),
    # --- Validated rules/bookkeeping engines: closed-form arithmetic pinned to
    # hand-derived ground truth (2026-09-02). Validated here means "the
    # arithmetic is proven against an independent hand derivation with a
    # stated error band", NOT "a physics simulation" — the `kind` column keeps
    # saying rules_engine / reporting so nobody reads the tier as physics.
    entry("tolerance_stack", "analyzers/tolerance_stack.py", "rules_engine", VALIDATED,
          ["validation/tolerance_stack_validation.py"],
          "manifests/tolerance_stack.manifest.json",
          "1-D worst-case + RSS tolerance stack-up + bore/shaft fit; pinned to "
          "hand-derived textbook stacks (worst-case sum|t|, RSS sqrt(sum t^2) "
          "with +/-t = 3 sigma, asymmetric mid-shift, fit extremes) — exact to "
          "the receipt's 9-decimal rounding, plus the worst-case-fails/RSS-passes "
          "divergence, refusal and exit-code contract, and byte determinism.",
          gate_suite="tests/test_checkers.py"),
    entry("production_check", "analyzers/production_check.py", "rules_engine", VALIDATED,
          ["validation/production_check_validation.py"],
          "manifests/production_check.manifest.json",
          "FDM production derating rules over material_db.json + the PLA creep "
          "table; pinned to the table cells times the documented rules (static "
          "55/10, creep [23C][1y] 2.5 MPa, 25 C rounds UP to the 55C row, "
          "across-layer x0.55, fatigue 60x0.3, temp 55/60) and to the three "
          "creep REFUSALS (no duration / above table / no table -> allowable "
          "0.0, exit 2). The --selftest is self-consistency; the pin is the "
          "independent hand derivation."),
    entry("production_dossier", "publish/production_dossier.py", "reporting", VALIDATED,
          ["validation/production_dossier_validation.py"],
          "manifests/production_dossier.manifest.json",
          "BOM cost rollup + printed-mass model + FDM plate packing; pinned to "
          "analytic box STLs (exact volume and area, the documented shell "
          "formula by hand: 30x20x10 PLA box -> 5.3072 g printed / 7.44 g "
          "solid, thick-section warning, shell cap, TBD buy lines, 2-plate "
          "packing on a 60x60 bed with in-bed non-overlapping placements, "
          "too-tall refusal). Bookkeeping proven exact, NOT a physics analysis.",
          gate_suite="tests/test_aux_tools.py"),
]

# ---------------------------------------------------------------------------
# Derived-model manifests (tools/derived_model.py scaffold) are auto-registered:
# committing tools/manifests/derived/<name>.manifest.json puts the model on the
# ledger. Tier is Demonstrated (manifest + inline self-check gates, no committed
# ground-truth pin) unless the manifest names a pin_file that exists — then it
# may claim Validated and the normal resolver re-verifies. A manifest that does
# not parse is a gate FAILURE, not a silent skip.
# ---------------------------------------------------------------------------
DERIVED_DIR = MANIFESTS / "derived"
DERIVED_SCAN_PROBLEMS: list[str] = []


def derived_entries() -> list[dict]:
    rows = []
    for mf in sorted(DERIVED_DIR.glob("*.manifest.json")):
        rel = str(mf.relative_to(TOOLS))
        try:
            m = json.loads(mf.read_text(encoding="utf-8"))
        except Exception as e:  # noqa: BLE001 — any parse failure must fail the gate
            DERIVED_SCAN_PROBLEMS.append(f"derived manifest {rel} does not parse: {e}")
            continue
        pin = (m.get("validation") or {}).get("pin_file") or ""
        pin_present = bool(pin) and (TOOLS / pin.replace("tools/", "", 1)).is_file()
        n_src = len(m.get("sources") or [])
        rows.append(entry(
            str(m.get("analyzer") or mf.stem),
            str(m.get("model_file") or "derived_model.py"),
            "derived_model",
            VALIDATED if pin_present else DEMONSTRATED,
            [pin.replace("tools/", "", 1)] if pin_present else [],
            rel,
            f"auto-registered derived model ({n_src} cited source(s)): "
            f"{m.get('title', '')} — inline self-check gates re-run every "
            f"invocation; status synthesized_inloop"
            + ("" if pin_present else "; no committed ground-truth pin"),
        ))
    return rows


REGISTRY.extend(derived_entries())

# Tools in tools/ that are deliberately NOT analysis surface (documented so the
# health denominator is defensible). Renderers, codegen, and geometry bridges
# produce pictures / code / preprocessed grids, not analysis numbers that reach
# a user as a claim about physical behaviour.
NON_ANALYSIS = {
    "analysis_sheet.py": "renderer of analysis results (presentation, not an analyzer)",
    "render_sheet.py": "12-view contact-sheet renderer",
    "assembly_doc.py": "assembly-documentation renderer",
    "motion_gif.py": "motion-study GIF renderer",
    "make_all_plate.py": "multi-part bed-plate packer (utility)",
    "bom_audit.py": "STEP-assembly hardware tally (bookkeeping)",
    "gen_discover.py": "codegen: regenerates discover.rs from program.rs",
    "voxelize_stl.py": "STL -> voxel occupancy bridge (preprocessor)",
    "provenance.py": "the analysis-result contract (this pipeline)",
    "analyzer_registry.py": "this registry",
    "_layout.py": "the tools/ directory map (analyzers/ publish/ validation/ tests/) + shim detection",
    "check_ci_security.py": "CI trust-boundary gate (workflows, not parts)",
    "dim_suggest.py": "analytic dimension-callout suggester for render_sheet (documentation helper)",
    "ingest_calibration.py": "printer calibration coupon ingest -> profiles/<printer>.json (data capture)",
    "derived_model.py": "the derived-model scaffold (models register themselves via tools/manifests/derived/)",
    "stress_to_density.py": "stress .npy -> graded density .npy remap (preprocessor)",
    "document_bundle.py": "deliverable-bundle orchestrator (runs the tools above; emits no number of its own)",
    "render_views.py": "quick 4-view PNG renderer",
    "material_db.json": "material data (not a script)",
    "materials.py": "material-data module (parallel-owned; not an analyzer)",
    "_ace.py": "shared ACE runner harness",
    "_stl.py": "shared binary-STL loader",
    # validation/ pins and tests/ suites (evidence, not surface) — listed by
    # basename because the forwarding shims at the old flat paths share them.
    "ace_fea_validation.py": "validation pin (evidence, not a surface)",
    "ace_modal_validation.py": "validation pin",
    "ace_buckling_validation.py": "validation pin",
    "ace_fea_kt_validation.py": "validation pin (parallel-owned, pending)",
    "ace_fea_kt_tet_validation.py": "validation pin (body-fitted tet10 Kt convergence)",
    "ace_optimize_validation.py": "validation pin",
    "param_optimize_validation.py": "validation pin",
    "tolerance_stack_validation.py": "validation pin (hand-derived worst-case + RSS stacks)",
    "production_check_validation.py": "validation pin (material-table cells x the documented rules)",
    "production_dossier_validation.py": "validation pin (analytic box STLs: volume, area, shell mass model, packing)",
    "_receipt.py": "shared receipt + exit-code contract for the runners",
    "test_ace_thermal.py": "benchmark gate suite (evidence, not a surface)",
    "test_ace_contact_fatigue.py": "benchmark gate suite (evidence, not a surface)",
    "test_ace_modal_buckling.py": "benchmark gate suite (evidence, not a surface)",
    "test_aux_tools.py": "benchmark gate suite (evidence, not a surface)",
    "test_checkers.py": "benchmark gate suite (evidence, not a surface)",
    "materials_crosslang_test.py": "cross-language pin: one creep table, two readers, 540 probes",
    "param_optimize_drift_test.py": "witness-selection drift detector for param_optimize (gate, not a surface)",
    "audit_docs.py": (
        "doc-drift auditor: checks the prose corpus (README/API/campaign/DESIGN_GUIDE) against the live op "
        "surface. It analyses DOCUMENTS, not parts — it computes no physical quantity, has no "
        "manifest and no validation pin, and nothing it prints is a number about a design. "
        "Caught by the drift scan only because its filename contains 'audit'."
    ),
    "field_triage.py": "field-report → remediation reasoner (reads material records; computes no field)",
    "field_report.py": "field-report intake (data capture, not an analyzer)",
}

FALLBACK_TIER = DEMONSTRATED  # where an over-claimed Validated lands

# Pins whose FAILURE is a pre-documented known issue: if they fail, --run-pins
# surfaces them as a NAMED, VISIBLE known-issue and does NOT block; if they pass,
# fine. Any pin that fails and is NOT declared here BLOCKS. This is the opposite
# of an off-switch — the alarm rings and is named, it is not swallowed. Empty
# today because every pin currently passes (verified 2026-07-12).
KNOWN_ISSUES: dict[str, str] = {}

# Declared KNOWN LIMITATIONS: always-surfaced caveats on Validated analyzers whose
# pins PASS but whose validity is bounded. These are not failures — they are the
# honest boundary of trust, made structural instead of buried in a pin's stdout.
KNOWN_LIMITATIONS = [
    {
        "analyzer": "ace_fea",
        "pin": "validation/ace_fea_kt_validation.py",
        "limitation": "peak/fillet stress is trustworthy to only ~+/-20-30%, biased HIGH, "
                      "and does NOT converge under refinement",
        "reason": "voxel hex8 staircases curved boundaries into re-entrant corners; the "
                  "'peak' measures the mesh artifact, not the geometry. Nominal/section-"
                  "average stress, displacement, modal, buckling are unaffected (~1%).",
        "fix": "body-fitted meshing or surface-stress recovery (a build, explicitly out of "
               "scope of the wiring pass — the Kt pin makes the error VISIBLE, not fixed).",
    },
    {
        "analyzer": "ace_fea_tet",
        "pin": "validation/ace_fea_kt_tet_validation.py",
        "limitation": "surface stress is nodal-AVERAGED recovery (not full SPR), so the "
                      "converged fillet peak is still a few percent shy of the analytic Kt; "
                      "robustness is bounded to gmsh-meshable WATERTIGHT geometry",
        "reason": "the tet10 path fixes the voxel non-convergence (Kt converges from below to "
                  "1.667), but nodal averaging is not superconvergent patch recovery and the "
                  "meshing depends on gmsh accepting the solid.",
        "fix": "SPR-based recovery for a tighter converged peak; the Kt pin ladder stops at "
               "elem ~r/2.4 (1.0 mm) for runtime — finer meshes tighten the residual.",
    },
]


def _pin_python() -> str:
    """The interpreter the pins need. Since the physics moved in-tree
    (tools/analyzers/physics/, 2026-09-04) that is just an interpreter carrying
    the locked numpy/scipy — by default the one running this registry.
    LMCAD_ANALYSIS_PYTHON overrides (CI points it at the hash-locked venv)."""
    return os.environ.get("LMCAD_ANALYSIS_PYTHON", sys.executable)


def run_pin(pin_file: str) -> dict:
    """Actually EXECUTE a validation pin (not just check it exists). Returns
    {pin, ran, passed, exit_code, tail}. `ran=False` means the pin could not be
    launched (e.g. ACE/miniconda absent) — reported honestly, never silently ok."""
    path = _path(pin_file)
    if not path.is_file():
        return {"pin": pin_file, "ran": False, "passed": False, "exit_code": None,
                "tail": "pin file missing"}
    try:
        # Wall-clock guards are protection against a hang, NOT part of the physics.
        # A 2-core hosted runner is several times slower than a dev machine, so a
        # fixed budget makes a correct pin look broken there (it did: three pins
        # blocked in CI on 2026-09-04 while all ten passed locally). Tunable, with
        # the same default as before.
        timeout = float(os.environ.get("LMCAD_PIN_TIMEOUT_S", "900"))
        proc = subprocess.run([_pin_python(), str(path)], capture_output=True,
                              text=True, timeout=timeout)
    except FileNotFoundError as exc:
        return {"pin": pin_file, "ran": False, "passed": False, "exit_code": None,
                "tail": f"interpreter unavailable: {exc}"}
    except subprocess.TimeoutExpired:
        return {"pin": pin_file, "ran": True, "passed": False, "exit_code": 124,
                "tail": "TIMEOUT (>900s)"}
    tail = (proc.stdout.strip().splitlines() or ["<no stdout>"])[-1]
    # A pin that fails with "<no stdout>" tells the reader nothing — the traceback
    # went to stderr. That opacity cost three CI cycles on 2026-09-04 guessing at
    # failures that reproduce nowhere locally. On failure, carry the stderr tail.
    if proc.returncode != 0:
        err = " | ".join(proc.stderr.strip().splitlines()[-3:])
        if err:
            tail = f"{tail}  [stderr] {err[:400]}"
    return {"pin": pin_file, "ran": True, "passed": proc.returncode == 0,
            "exit_code": proc.returncode, "tail": tail}


def run_pins(rows: list[dict]) -> tuple[bool, list[dict]]:
    """Run every present pin of every Validated analyzer and gate on run+pass.
    A failing pin BLOCKS unless it is a declared KNOWN_ISSUES entry (then it is a
    visible, named, non-blocking known-issue). A pin that could not run BLOCKS
    (an un-run pin is not evidence). Returns (gate_ok, per-pin results)."""
    results, gate_ok = [], True
    for r in rows:
        if r["effective_tier"] != VALIDATED:
            continue
        for pin in r["pins_present"]:
            res = run_pin(pin)
            res["analyzer"] = r["name"]
            if res["passed"]:
                res["status"] = "PASS"
            elif not res["ran"]:
                res["status"] = "COULD-NOT-RUN (blocks)"
                gate_ok = False
            elif pin in KNOWN_ISSUES:
                res["status"] = "KNOWN-ISSUE (non-blocking)"
                res["known_reason"] = KNOWN_ISSUES[pin]
            else:
                res["status"] = "FAIL (blocks)"
                gate_ok = False
            results.append(res)
    return gate_ok, results


# ---------------------------------------------------------------------------
# Resolution.
# ---------------------------------------------------------------------------
def _manifest_valid(path: Path) -> tuple[bool, list[str]]:
    """Light schema check of a manifest file (used by the gate)."""
    required = ("schema", "analyzer", "analyzer_version", "title",
                "governing_equations", "assumptions", "boundary_conditions",
                "units", "discretization", "validation", "caveats",
                "limits_of_validity")
    try:
        m = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:  # noqa: BLE001
        return False, [f"{path.name}: not valid JSON ({exc})"]
    problems = []
    if m.get("schema") != "lmcad.manifest.v1":
        problems.append(f"{path.name}: schema != lmcad.manifest.v1")
    for k in required:
        if not m.get(k):
            problems.append(f"{path.name}: required field '{k}' missing/empty")
    return (not problems), problems


def resolve(e: dict) -> dict:
    """Resolve one registry entry against the filesystem."""
    file_present = _path(e["file"]).is_file()
    manifest_path = (TOOLS / e["manifest"]) if e["manifest"] else None
    manifest_present = bool(manifest_path and manifest_path.is_file())

    pins_present, pins_pending = [], []
    for p in e["pins"]:
        pin_name = p.split(" ")[0]  # tolerate "file.py (pending ...)" annotations
        (pins_present if _path(pin_name).is_file() else pins_pending).append(pin_name)

    violations = []
    manifest_problems = []
    if manifest_present:
        ok_m, manifest_problems = _manifest_valid(manifest_path)
    else:
        ok_m = False

    claimed = e["claimed_tier"]
    effective = claimed
    if claimed == VALIDATED:
        if not manifest_present:
            violations.append(f"{e['name']} claims Validated but has NO manifest")
        if not pins_present:
            violations.append(f"{e['name']} claims Validated but has NO present validation pin")
        if manifest_present and not ok_m:
            violations.extend(manifest_problems)
        if not (manifest_present and pins_present and ok_m):
            effective = FALLBACK_TIER
    if not file_present:
        violations.append(f"{e['name']}: analyzer file '{e['file']}' is missing on disk")

    return {
        "name": e["name"],
        "file": e["file"],
        "kind": e["kind"],
        "claimed_tier": claimed,
        "effective_tier": effective,
        "file_present": file_present,
        "manifest": e["manifest"],
        "manifest_present": manifest_present,
        "manifest_valid": ok_m if manifest_present else None,
        "pins_present": pins_present,
        "pins_pending": pins_pending,
        "has_pin": bool(pins_present),
        "gate_suite": e.get("gate_suite"),
        "gate_suite_present": bool(e.get("gate_suite")
                                   and _path(e["gate_suite"]).is_file()),
        "tier_reason": e.get("tier_reason") or (
            f"{claimed} by the registry rule: Validated requires a present manifest "
            f"AND a present validation pin."),
        "rationale": e["rationale"],
        "violations": violations,
    }


def resolve_all() -> list[dict]:
    return [resolve(e) for e in REGISTRY]


def health(rows: list[dict]) -> dict:
    total = len(rows)
    validated = sum(1 for r in rows if r["effective_tier"] == VALIDATED)
    demonstrated = sum(1 for r in rows if r["effective_tier"] == DEMONSTRATED)
    cataloged = sum(1 for r in rows if r["effective_tier"] == CATALOGED)
    below = total - validated
    return {
        "total_surface": total,
        "validated": validated,
        "demonstrated": demonstrated,
        "cataloged": cataloged,
        "validated_pct": round(100.0 * validated / total, 1) if total else 0.0,
        "below_validated_line_pct": round(100.0 * below / total, 1) if total else 0.0,
        "metric_note": "count-weighted: each registered analyzer = one unit of "
                       "analysis surface (a stated simplification — surfaces are "
                       "not weighted by usage or blast radius).",
    }


def uncatalogued() -> list[str]:
    """Analyzer-shaped files in tools/ (top level, analyzers/, publish/) that are
    neither registered nor declared non-analysis — a soft drift warning (does
    not fail the gate). Forwarding shims at the old flat paths are skipped: a
    shim is a pointer to a surface, not a surface. tools/_parked/ is not
    scanned at all — a parked tool is off the surface by definition."""
    known = {os.path.basename(e["file"]) for e in REGISTRY} | set(NON_ANALYSIS)
    out = []
    for d in (TOOLS, _layout.ANALYZERS, _layout.PUBLISH):
        for f in sorted(d.glob("*.py")):
            if f.name in known or f.name.startswith("__") or _layout.is_shim(f):
                continue
            if f.name.endswith("_runner.py") or f.name.endswith("_check.py") or "audit" in f.name:
                out.append(str(f.relative_to(TOOLS)))
    return out


# ---------------------------------------------------------------------------
# Gate (for CI).
# ---------------------------------------------------------------------------
def gate(rows: list[dict]) -> tuple[bool, list[str]]:
    problems = list(DERIVED_SCAN_PROBLEMS)  # unparseable derived manifests fail loudly
    for r in rows:
        problems.extend(r["violations"])
    # Any file that is present-and-declared-Validated must have a valid manifest.
    return (not problems), problems


# ---------------------------------------------------------------------------
# Rendering.
# ---------------------------------------------------------------------------
def _table(rows: list[dict]) -> str:
    w_name = max(len(r["name"]) for r in rows)
    w_tier = len("Demonstrated")
    lines = []
    header = (f"{'analyzer'.ljust(w_name)}  {'tier'.ljust(w_tier)}  man  pin  "
              f"gate  file")
    lines.append(header)
    lines.append("-" * len(header))
    for r in rows:
        man = "yes" if r["manifest_present"] else " no"
        pin = "yes" if r["has_pin"] else " no"
        # A gate suite is NOT a pin. It is shown in its own column precisely so
        # that "green gates" can never be read as "Validated tier".
        gt = "yes " if r["gate_suite_present"] else " no "
        fp = "ok" if r["file_present"] else "MISSING"
        flag = "  <- OVER-CLAIM" if r["claimed_tier"] != r["effective_tier"] else ""
        lines.append(
            f"{r['name'].ljust(w_name)}  {r['effective_tier'].ljust(w_tier)}  "
            f"{man}  {pin}  {gt}  {fp}{flag}"
        )
    lines.append("")
    lines.append("man = lmcad.manifest.v1 present · pin = validation pin present "
                 "(ground truth) · gate = benchmark gate suite present.")
    lines.append("TIER is decided by man+pin ONLY. A green gate suite is evidence, "
                 "not a tier — query one row with --tier <name>.")
    return "\n".join(lines)


def report(rows: list[dict]) -> str:
    h = health(rows)
    out = ["LMCAD analyzer registry — graduation ledger", ""]
    out.append(_table(rows))
    out.append("")
    out.append(f"SYSTEM HEALTH")
    out.append(f"  analysis surface (registered analyzers): {h['total_surface']}")
    out.append(f"  Validated:    {h['validated']}")
    out.append(f"  Demonstrated: {h['demonstrated']}")
    out.append(f"  Cataloged:    {h['cataloged']}")
    out.append(f"  % ABOVE the validated line (validated):        {h['validated_pct']}%")
    out.append(f"  % BELOW the validated line (not yet validated): {h['below_validated_line_pct']}%")
    out.append(f"  ({h['metric_note']})")
    unc = uncatalogued()
    if unc:
        out.append("")
        out.append("WARNING — analyzer-shaped tools not in the registry (catalogue drift):")
        for f in unc:
            out.append(f"  - {f}")
    ok, problems = gate(rows)
    if not ok:
        out.append("")
        out.append("GATE VIOLATIONS:")
        for p in problems:
            out.append(f"  - {p}")
    return "\n".join(out)


# ---------------------------------------------------------------------------
# THE RUNNER CONTRACT GATE (portfolio theme T3/T7 — "silence is the one
# forbidden outcome"). The registry says WHAT each analyzer is; this says the
# analyzers all still SIGNAL the same way. Every gate drives a real runner as a
# subprocess and asserts on (exit code, receipt) TOGETHER, because the defect
# being closed was exactly that those two disagreed: a genuine `ok:false`
# analysis exited 0 while an internal KeyError exited 1.
#
# Run:  python3 analyzer_registry.py --check-contract     -> exit 0 iff green
# ---------------------------------------------------------------------------
CONTRACT_RUNNERS = [
    "analyzers/ace_fea_runner.py", "analyzers/ace_fea_tet_runner.py",
    "analyzers/ace_modal_runner.py", "analyzers/ace_buckling_runner.py",
    "analyzers/ace_thermal_runner.py", "analyzers/ace_contact_runner.py",
    "analyzers/ace_fatigue_runner.py", "analyzers/ace_optimize_runner.py",
    "analyzers/graded_infill_runner.py",
]


def _why(rec) -> str:
    """The receipt's own explanation, for a gate detail line.

    A contract check that fails with `kind=internal` and no message is a dead
    end for whoever reads the CI log — that is how the 2026-09-04 analysis-gate
    failure stayed opaque across three pushes. Surface what the runner said.
    """
    if not rec:
        return "no receipt"
    err = rec.get("error") or rec.get("error_kind") or "-"
    return str(err)[:200]


def _run_runner(runner: str, argv: list[str], env_extra: dict | None = None,
                timeout: int = 300):
    """Drive a runner CLI. Returns (receipt_or_None, returncode)."""
    env = dict(os.environ)
    env.pop("LMCAD_RUNNER_EXIT", None)
    env.pop("LMCAD_RECEIPT_DRY_RUN", None)
    env.update(env_extra or {})
    proc = subprocess.run([sys.executable, str(TOOLS / runner)] + argv,
                          capture_output=True, text=True, timeout=timeout, env=env)
    receipt = None
    for line in proc.stdout.splitlines():
        line = line.strip()
        if line.startswith("{"):
            try:
                receipt = json.loads(line)
            except json.JSONDecodeError:
                pass
    return receipt, proc.returncode


def _prism_job(out_dir: Path, **over) -> dict:
    """A 12x12x16 mm PLA prism, clamped at the bottom. The base of every gate."""
    job = {
        "out_dir": str(out_dir),
        "voxel_mm": 2.0,
        "ops": [{"id": "b", "op": "box", "min": [0, 0, 0], "max": [12, 12, 16]}],
        "solid": "b",
        "shape": [6, 6, 8],
        "material": "PLA",
        "fixtures": [{"kind": "clamped",
                      "region_selector": {"type": "plane", "axis": "z",
                                          "value_mm": 2.0, "side": "-"}}],
        "loads": [{"kind": "point", "magnitude": 40.0, "direction": [0, 0, -1],
                   "region_selector": {"type": "plane", "axis": "z",
                                       "value_mm": 14.0, "side": "+"}}],
    }
    job.update(over)
    return job


def check_contract() -> tuple[bool, list[dict]]:  # noqa: PLR0915 — one linear gate list
    results = []

    def gate(name: str, passed: bool, detail: str) -> None:
        results.append({"gate": name, "passed": bool(passed), "detail": detail})

    # --- STRUCTURAL: every runner routes through the shared contract ---------
    # A runner that keeps its own `sys.exit(0)` failure path would silently
    # opt out of everything below, so this is checked as source, not behaviour.
    for r in CONTRACT_RUNNERS:
        src = (TOOLS / r).read_text(encoding="utf-8")
        name = os.path.basename(r)
        gate(f"{name} uses run_cli", "run_cli(" in src,
             "routes its __main__ through the shared contract"
             if "run_cli(" in src else "still has a bespoke __main__")
        gate(f"{name} has no bare exit-0-on-failure", "sys.exit(0)" not in src,
             "no literal sys.exit(0)" if "sys.exit(0)" not in src
             else "a literal sys.exit(0) remains — a failure would exit 0")
        # The forwarding shim at the old flat path must still reach the same
        # file: a campaign's `python3 tools/<runner>.py job.json` is a promise.
        shim = TOOLS / name
        gate(f"{name} old path is a forwarding shim",
             shim.is_file() and _layout.is_shim(shim) and name in shim.read_text(encoding="utf-8"),
             f"tools/{name} forwards to tools/{r}" if shim.is_file()
             else f"tools/{name} is missing — old command lines would break")

    with tempfile.TemporaryDirectory(prefix="lmcad_contract_") as td:
        tmp = Path(td)

        def write(name: str, job: dict) -> str:
            p = tmp / f"{name}.json"
            p.write_text(json.dumps(job), encoding="utf-8")
            return str(p)

        # --- 1. POSITIVE CONTROL: exit 0 is reachable -----------------------
        good = write("good", _prism_job(tmp / "good"))
        rec, code = _run_runner("ace_fea_runner.py", [good])
        gate("1 positive control: ok:true AND exit 0",
             code == 0 and rec is not None and rec.get("ok") is True
             and rec.get("exit_code") == 0,
             f"exit {code}, ok={None if rec is None else rec.get('ok')}, err={_why(rec)}")

        # --- 2. A REFUSED ANALYSIS EXITS 2, NOT 0 ---------------------------
        # Pure tension into a buckling solve: the exact din_rail F7 job shape.
        tens = write("tension", _prism_job(
            tmp / "tension",
            loads=[{"kind": "point", "magnitude": 40.0, "direction": [0, 0, 1],
                    "region_selector": {"type": "plane", "axis": "z",
                                        "value_mm": 14.0, "side": "+"}}]))
        rec, code = _run_runner("ace_buckling_runner.py", [tens])
        gate("2 tensile buckling REFUSED: ok:false AND exit 2",
             code == 2 and rec is not None and rec.get("ok") is False
             and rec.get("error_kind") == "refusal.no_compressive_load_path",
             f"exit {code}, kind={None if rec is None else rec.get('error_kind')}")
        gate("2 the refusal carries a compression receipt, not just prose",
             bool(rec) and rec.get("compression_check", {}).get("verdict") == "tensile",
             f"compression_check.verdict="
             f"{(rec or {}).get('compression_check', {}).get('verdict')}")

        # --- 3. ZERO-CATCH SELECTOR REFUSED --------------------------------
        empty = write("empty", _prism_job(
            tmp / "empty",
            fixtures=[{"kind": "clamped",
                       "region_selector": {"type": "bbox",
                                           "min_mm": [0, 0, 40], "max_mm": [12, 12, 60]}}]))
        rec, code = _run_runner("ace_fea_runner.py", [empty])
        gate("3 fixture selector catching 0 active elements REFUSED (exit 2)",
             code == 2 and bool(rec) and rec.get("error_kind") == "refusal.empty_selector",
             f"exit {code}, kind={(rec or {}).get('error_kind')}")

        # --- 4. AN INTERNAL ERROR EXITS 1, NOT 2 (the inversion) ------------
        bad = write("bad", _prism_job(tmp / "bad", npy="/nonexistent/grid.npy"))
        rec, code = _run_runner("ace_fea_runner.py", [bad])
        gate("4 internal error exits 1 (distinct from a refusal's 2)",
             code == 1 and bool(rec) and rec.get("ok") is False
             and rec.get("error_kind") == "internal",
             f"exit {code}, kind={(rec or {}).get('error_kind')}")

        # --- 5. A TYPO IS NOT A SILENT DEFAULT ------------------------------
        rec, code = _run_runner("ace_fea_runner.py", [good, "--nosuchflag"])
        gate("5 unknown flag refused, not ignored",
             code != 0 and bool(rec) and rec.get("ok") is False,
             f"exit {code}, error={(rec or {}).get('error', '')[:60]!r}")

        # --- 6. THE LEGACY OPT-OUT IS LOUD ----------------------------------
        rec, code = _run_runner("ace_buckling_runner.py", [tens],
                                {"LMCAD_RUNNER_EXIT": "legacy"})
        gate("6 legacy opt-out exits 0 but RECORDS mode+suppressed code",
             code == 0 and bool(rec) and rec.get("ok") is False
             and rec.get("exit_contract", {}).get("mode") == "legacy"
             and rec.get("exit_contract", {}).get("suppressed_code") == 2,
             f"exit {code}, contract={(rec or {}).get('exit_contract', {}).get('mode')}")

        # --- 7. WALL BUDGET -> A RECEIPT, NOT A VANISHED RUN ----------------
        # 0.01 s is below the cost of sampling the grid on ANY machine, so this
        # gate is deterministic rather than load-dependent (a flaky gate is a
        # broken gate). What is being proven is the RECEIPT, not the duration.
        slow = write("slow", _prism_job(tmp / "slow", voxel_mm=0.6,
                                        shape=[20, 20, 27], wall_budget_s=0.01))
        rec, code = _run_runner("ace_fea_runner.py", [slow])
        gate("7 wall budget synthesizes an ok:false receipt (never silence)",
             code == 2 and bool(rec) and rec.get("error_kind") == "timeout"
             and rec.get("killed_at_wall_budget") is True,
             f"exit {code}, kind={(rec or {}).get('error_kind')}")

        # --- 8. --out NEVER LOSES TO A JOB `receipt` KEY --------------------
        shipped = tmp / "SHIPPED.json"
        shipped.write_text('{"ok": true, "shipped": true}\n', encoding="utf-8")
        clash = write("clash", _prism_job(tmp / "clash", receipt=str(shipped)))
        rec, code = _run_runner("ace_fea_runner.py",
                                [clash, "--out", str(tmp / "elsewhere.json")])
        untouched = json.loads(shipped.read_text(encoding="utf-8")).get("shipped") is True
        gate("8 job 'receipt' vs --out is REFUSED, shipped file untouched",
             code == 1 and bool(rec)
             and rec.get("error_kind") == "receipt_path_conflict" and untouched,
             f"exit {code}, kind={(rec or {}).get('error_kind')}, "
             f"shipped_intact={untouched}")

        # --- 9. DRY RUN WRITES NOTHING --------------------------------------
        dest = tmp / "dry_target.json"
        dry = write("dry", _prism_job(tmp / "dry", receipt=str(dest)))
        rec, code = _run_runner("ace_fea_runner.py", [dry],
                                {"LMCAD_RECEIPT_DRY_RUN": "1"})
        gate("9 LMCAD_RECEIPT_DRY_RUN=1 writes no file (what-if runs are safe)",
             code == 0 and not dest.exists(),
             f"exit {code}, file_written={dest.exists()}")

        # --- 10. --out IS ATOMIC AND HONOURED -------------------------------
        outp = tmp / "atomic.json"
        rec, code = _run_runner("ace_fea_runner.py", [good, "--out", str(outp)])
        wrote_valid = outp.exists() and json.loads(
            outp.read_text(encoding="utf-8")).get("ok") is True
        gate("10 --out writes a complete receipt (temp+rename, never 0 bytes)",
             code == 0 and wrote_valid,
             f"exit {code}, valid_receipt_on_disk={wrote_valid}")

        # --- 11. T7: THE DETERMINISTIC CORE IS BYTE-COMPARABLE --------------
        a, _ = _run_runner("ace_fea_runner.py", [good])
        b, _ = _run_runner("ace_fea_runner.py", [good])
        da = (a or {}).get("determinism") or {}
        db = (b or {}).get("determinism") or {}
        same_core = bool(da.get("core_digest")) and da.get("core_digest") == db.get("core_digest")
        moved = bool(a) and bool(b) and a.get("timings_s") != b.get("timings_s")
        gate("11 core_digest is stable across runs while timings_s moves",
             same_core,
             f"core_digest equal={same_core} (timings differed={moved}) — this is "
             f"the byte-comparison campaigns were told to make and could not")
        gate("11 the receipt NAMES its non-deterministic parts",
             da.get("nondeterministic_paths") == ["timings_s"],
             f"nondeterministic_paths={da.get('nondeterministic_paths')}")

    return all(r["passed"] for r in results), results


# ---------------------------------------------------------------------------
# Read-only contract demo: wrap ONE analyzer's real output in the envelope
# without modifying the analyzer. Uses tolerance_stack (pure stdlib, hermetic).
# ---------------------------------------------------------------------------
def demo() -> dict:
    chain_job = {
        "chain": [
            {"name": "housing_depth", "nominal": 20.0, "tol": 0.2, "dir": 1},
            {"name": "bearing_stack", "nominal": 12.0, "tol": 0.1, "dir": -1},
            {"name": "spacer", "nominal": 7.5, "tol": {"plus": 0.15, "minus": 0.15}, "dir": -1},
        ],
        "closes": {"min_required": 0.02, "max_allowed": 1.0},
    }
    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as fh:
        json.dump(chain_job, fh)
        job_path = fh.name
    try:
        proc = subprocess.run(
            [sys.executable, str(_layout.find_tool("tolerance_stack.py")), job_path],
            capture_output=True, text=True, timeout=60,
        )
    finally:
        os.unlink(job_path)

    receipt = None
    for line in proc.stdout.splitlines():
        line = line.strip()
        if line.startswith("{"):
            receipt = json.loads(line)
    if receipt is None:
        raise RuntimeError(f"tolerance_stack produced no receipt: {proc.stderr[:300]}")

    # The 'geometry' of a tolerance stack IS its dimension chain — hash it as a
    # program (deterministic, sorted-keys). Status = the analyzer's registry
    # tier, READ FROM THE RESOLVED LEDGER (never hardcoded): if the manifest or
    # pin ever goes missing the demo downgrades with the registry instead of
    # keeping a stale "validated" stamp.
    row = resolve(next(e for e in REGISTRY if e["name"] == "tolerance_stack"))
    status = {VALIDATED: provenance.STATUS_VALIDATED,
              DEMONSTRATED: provenance.STATUS_DEMONSTRATED,
              CATALOGED: provenance.STATUS_CATALOGED}[row["effective_tier"]]
    manifest_ref = (f"tools/{row['manifest']}" if row["manifest_present"] else None)
    ghash = provenance.geometry_hash(program=chain_job)
    matver = provenance.material_db_version(str(TOOLS / "material_db.json"))
    envelope = provenance.stamp(
        values=receipt,
        geometry_hash=ghash,
        material_version=matver,
        analyzer_name="tolerance_stack",
        analyzer_version="1.1.0",
        validation_status=status,
        residual_or_convergence={
            "method": "closed-form worst-case + RSS",
            "iterative": False,
            "exact": True,
            "note": "no iteration; worst-case and RSS are exact for the 1-D "
                    "linear chain — 'convergence' is not applicable, reported "
                    "structurally rather than as a bare number.",
        },
        manifest_ref=manifest_ref,  # None only if the ledger has no manifest for it
        geometry_relation=provenance.equality_relation(ghash),
    )
    return envelope


def _main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="LMCAD analyzer registry / graduation ledger.")
    ap.add_argument("--json", action="store_true", help="machine-readable resolution")
    ap.add_argument("--check", action="store_true", help="CI gate: exit 1 on any violation")
    ap.add_argument("--run-pins", action="store_true",
                    help="EXECUTE every Validated analyzer's pins; exit 1 on an "
                         "un-run or non-known-issue failure")
    ap.add_argument("--demo", action="store_true", help="read-only stamp() demo on one analyzer")
    ap.add_argument("--tier", metavar="NAME",
                    help="machine-queryable tier of ONE analyzer: prints a one-line "
                         "JSON object {name, tier, claimed_tier, over_claim, manifest, "
                         "pins, gate_suite, tier_reason} and exits 0; exits 1 with "
                         "{error, known} if NAME is not registered (a typo must never "
                         "become a silent default)")
    ap.add_argument("--check-contract", action="store_true",
                    help="EXECUTE the runner contract gates (exit codes, refusals, "
                         "receipt destinations, determinism digest, wall budget); "
                         "exit 1 on any failure")
    args = ap.parse_args(argv)

    rows = resolve_all()

    if args.tier:
        row = next((r for r in rows if r["name"] == args.tier), None)
        if row is None:
            print(json.dumps({"error": f"no registered analyzer named {args.tier!r}",
                              "known": sorted(r["name"] for r in rows)}))
            return 1
        print(json.dumps({
            "name": row["name"],
            "tier": row["effective_tier"],
            "claimed_tier": row["claimed_tier"],
            "over_claim": row["claimed_tier"] != row["effective_tier"],
            "manifest": row["manifest"] if row["manifest_present"] else None,
            "pins": row["pins_present"],
            "gate_suite": row["gate_suite"] if row["gate_suite_present"] else None,
            "tier_reason": row["tier_reason"],
            "tier_definitions": {
                VALIDATED: "manifest + >=1 present validation pin against independent "
                           "ground truth with a documented error band",
                DEMONSTRATED: "runs end-to-end with a self-check / gate suite, but is "
                              "not pinned to independent ground truth in this registry",
                CATALOGED: "deterministic rules/arithmetic over published tables; "
                           "correct relative to its sources, not a physics simulation",
            },
        }, sort_keys=True))
        return 0

    if args.check_contract:
        ok, results = check_contract()
        for r in results:
            print(f"  {'PASS' if r['passed'] else 'FAIL'}  {r['gate']}: {r['detail']}")
        print(f"runner-contract gate: {'PASS' if ok else 'FAIL'} "
              f"({sum(1 for r in results if r['passed'])}/{len(results)})")
        return 0 if ok else 1

    if args.demo:
        envelope = demo()
        ok, problems = provenance.check_envelope(envelope)
        print(json.dumps(envelope, indent=2, sort_keys=True))
        print("", file=sys.stderr)
        print(f"envelope well-formed: {ok}" + (f" problems={problems}" if problems else ""),
              file=sys.stderr)
        return 0 if ok else 1

    if args.json:
        print(json.dumps({
            "analyzers": rows,
            "health": health(rows),
            "non_analysis_tools": NON_ANALYSIS,
            "uncatalogued": uncatalogued(),
        }, indent=2, sort_keys=True))
        return 0

    if args.check:
        ok, problems = gate(rows)
        h = health(rows)
        print(f"registry gate: {'PASS' if ok else 'FAIL'} | "
              f"validated {h['validated']}/{h['total_surface']} "
              f"({h['validated_pct']}%), below-line {h['below_validated_line_pct']}%")
        for lim in KNOWN_LIMITATIONS:
            print(f"  KNOWN LIMITATION [{lim['analyzer']}]: {lim['limitation']} "
                  f"(see {lim['pin']}; fix: {lim['fix']})")
        if not ok:
            for p in problems:
                print(f"  VIOLATION: {p}")
            return 1
        return 0

    if args.run_pins:
        gate_ok, results = run_pins(rows)
        print(f"pin execution via {_pin_python()}:")
        for res in results:
            line = f"  {res['status']:<26} {res['analyzer']:<14} {res['pin']}"
            if res.get("exit_code") is not None:
                line += f"  (exit {res['exit_code']})"
            print(line)
            print(f"      -> {res['tail']}")
            if res.get("known_reason"):
                print(f"      known-issue reason: {res['known_reason']}")
        for lim in KNOWN_LIMITATIONS:
            print(f"  KNOWN LIMITATION [{lim['analyzer']}]: {lim['limitation']} "
                  f"(pin {lim['pin']} PASSES as characterization; fix: {lim['fix']})")
        blocked = [r for r in results if "blocks" in r["status"]]
        print(f"\nrun-pins gate: {'PASS' if gate_ok else 'BLOCK'} "
              f"({sum(1 for r in results if r['status']=='PASS')} pass, "
              f"{sum(1 for r in results if r['status'].startswith('KNOWN'))} known-issue, "
              f"{len(blocked)} blocking)")
        return 0 if gate_ok else 1

    print(report(rows))
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
