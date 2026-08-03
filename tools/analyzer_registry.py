#!/usr/bin/env python3
"""analyzer_registry.py — the graduation ledger for every analyzer in tools/.

Enumerates each analysis surface the engine exposes, its committed TIER, whether
it carries a manifest (governing equations + assumptions) and a validation pin
(a check against independent ground truth), and computes the SYSTEM HEALTH
METRIC: the fraction of the analysis surface that sits BELOW the validated line.

Tiers (honest, non-inflatable — full definitions in docs/ANALYSIS_TIERS.md):

  Validated    manifest present AND >= 1 validation pin present; the pin checks
               the result against a closed-form / measured ground truth with a
               documented error band. fea / modal / buckling and BOTH optimizers
               (ace_optimize, param_optimize) qualify as of 2026-07-17.
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
import provenance  # noqa: E402  (sibling module)

TOOLS = Path(__file__).resolve().parent
MANIFESTS = TOOLS / "manifests"

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
def entry(name, file, kind, claimed_tier, pins, manifest, rationale):
    return {
        "name": name,
        "file": file,
        "kind": kind,
        "claimed_tier": claimed_tier,
        "pins": list(pins),
        "manifest": manifest,
        "rationale": rationale,
    }


REGISTRY = [
    # --- Validated: physics solvers with closed-form pins + manifests ---------
    entry("ace_fea", "ace_fea_runner.py", "physics_solver", VALIDATED,
          ["ace_fea_validation.py", "ace_fea_kt_validation.py"],
          "manifests/ace_fea.manifest.json",
          "hex8 linear-elastic FEA; pinned to Euler-Bernoulli+shear cantilever "
          "(-11.2%/-5.9% converging). Kt pin pending from a parallel agent."),
    entry("ace_fea_tet", "ace_fea_tet_runner.py", "physics_solver", VALIDATED,
          ["ace_fea_kt_tet_validation.py"],
          "manifests/ace_fea_tet.manifest.json",
          "body-fitted tet10 linear-elastic FEA (gmsh conforming mesh); pinned "
          "to the Peterson/Pilkey stepped-bar fillet Kt=1.667 — measured Kt "
          "1.545/1.610/1.652 CONVERGING from below under refinement (the voxel "
          "hex8 path scatters -6..+44% and does NOT converge). Surface stress is "
          "nodal-averaged recovery (not SPR); robustness bounded to gmsh-meshable "
          "watertight geometry; pin ladder stops at elem ~r/2.4 for runtime."),
    entry("ace_modal", "ace_modal_runner.py", "physics_solver", VALIDATED,
          ["ace_modal_validation.py"],
          "manifests/ace_modal.manifest.json",
          "hex8 free-vibration modal; pinned to Euler-Bernoulli cantilever "
          "first bending frequency (+4.0%/+0.9% converging from above)."),
    entry("ace_buckling", "ace_buckling_runner.py", "physics_solver", VALIDATED,
          ["ace_buckling_validation.py"],
          "manifests/ace_buckling.manifest.json",
          "hex8 linear buckling; pinned to the Euler clamped-free column "
          "(+7.3%/+3.0% converging from above)."),

    # --- Demonstrated: run end-to-end with a self-check, but not pinned -------
    entry("ace_optimize", "ace_optimize_runner.py", "optimizer", VALIDATED,
          ["ace_optimize_validation.py"],
          "manifests/ace_optimize.manifest.json",
          "SIMP/OC over the VALIDATED reference FEA; pinned via exact "
          "inequalities (no closed-form optimal topology exists): OC descent "
          "<=0.9x (measured 0.15x), material-removal monotonicity vs the "
          "solid beam (deflection ratio 1.58 >= 1), volume honesty +/-0.02 "
          "(measured exact), watertight STL gate."),
    entry("graded_infill", "graded_infill_runner.py", "geometry_synthesis", DEMONSTRATED,
          [], None,
          "Stress-graded gyroid infill from a prior ace_fea field; ships a "
          "--selftest and a support-necessity audit, but no ground-truth pin "
          "on the graded stiffness."),
    entry("param_optimize", "param_optimize.py", "optimizer", VALIDATED,
          ["param_optimize_validation.py"],
          "manifests/param_optimize.manifest.json",
          "Nelder-Mead + targets/robust/multi-start over ANY receipted "
          "analyzer; pinned to analytic known-optimum problems (paraboloid "
          "argmin, cubic-root target, active constraint cap) plus a "
          "byte-identical determinism pin. Pin 3 caught + fixed a real "
          "ulp-scale infeasible-selection defect (feasibility-first "
          "selection)."),
    entry("air_topology_audit", "air_topology_audit.py", "geometry_audit", DEMONSTRATED,
          [], None,
          "Voxel flood-fill connectivity gate for internal-air channels "
          "(the TL-91 defect gate); deterministic, defect-taught, but not "
          "pinned against a measured flow/acoustic reference."),
    entry("sweep_check", "sweep_check.py", "geometry_audit", DEMONSTRATED,
          [], None,
          "Parameter-sweep interference check over the engine's clearance "
          "measures; deterministic and engine-backed, no ground-truth pin."),
    entry("balance_check", "balance_check.py", "geometry_audit", DEMONSTRATED,
          [], None,
          "Rotating-assembly static/couple balance from the engine's "
          "mass_properties inertia tensor + rigid-body arithmetic; measures, "
          "does not impose a balance grade, not pinned to a measured rotor."),

    # --- Cataloged: deterministic rules/arithmetic over cited tables ----------
    entry("joint_check", "joint_check.py", "rules_engine", CATALOGED,
          [], None,
          "Fastener/insert capacity rules over published-typical capacity "
          "tables; no FEA, no engine calls, not validated against pull tests."),
    entry("tolerance_stack", "tolerance_stack.py", "rules_engine", CATALOGED,
          [], None,
          "1-D worst-case + RSS tolerance stack-up; exact closed-form "
          "arithmetic, correct-by-construction, but not a physics model."),
    entry("production_check", "production_check.py", "rules_engine", CATALOGED,
          [], None,
          "FDM production derating rules over material_db.json allowables; "
          "carries a pinned --selftest of the RULE arithmetic (self-consistency, "
          "NOT a ground-truth physics pin)."),
    entry("production_dossier", "production_dossier.py", "reporting", CATALOGED,
          [], None,
          "BOM cost rollup + FDM plate packing; deterministic bookkeeping and "
          "a stated print-time heuristic, not a physics analysis."),
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
    "material_db.json": "material data (not a script)",
    "materials.py": "material-data module (parallel-owned; not an analyzer)",
    "_ace.py": "shared ACE runner harness",
    "_stl.py": "shared binary-STL loader",
    "ace_fea_validation.py": "validation pin (evidence, not a surface)",
    "ace_modal_validation.py": "validation pin",
    "ace_buckling_validation.py": "validation pin",
    "ace_fea_kt_validation.py": "validation pin (parallel-owned, pending)",
    "ace_fea_kt_tet_validation.py": "validation pin (body-fitted tet10 Kt convergence)",
    "ace_optimize_validation.py": "validation pin",
    "param_optimize_validation.py": "validation pin",
    "_receipt.py": "shared receipt emit/persist helper",
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
        "pin": "ace_fea_kt_validation.py",
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
        "pin": "ace_fea_kt_tet_validation.py",
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
    """The interpreter the ACE pins need (miniconda with ACE installed)."""
    return os.environ.get("ACE_PYTHON", sys.executable)


def run_pin(pin_file: str) -> dict:
    """Actually EXECUTE a validation pin (not just check it exists). Returns
    {pin, ran, passed, exit_code, tail}. `ran=False` means the pin could not be
    launched (e.g. ACE/miniconda absent) — reported honestly, never silently ok."""
    path = TOOLS / pin_file
    if not path.is_file():
        return {"pin": pin_file, "ran": False, "passed": False, "exit_code": None,
                "tail": "pin file missing"}
    try:
        proc = subprocess.run([_pin_python(), str(path)], capture_output=True,
                              text=True, timeout=900)
    except FileNotFoundError as exc:
        return {"pin": pin_file, "ran": False, "passed": False, "exit_code": None,
                "tail": f"interpreter unavailable: {exc}"}
    except subprocess.TimeoutExpired:
        return {"pin": pin_file, "ran": True, "passed": False, "exit_code": 124,
                "tail": "TIMEOUT (>900s)"}
    tail = (proc.stdout.strip().splitlines() or ["<no stdout>"])[-1]
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
    file_present = (TOOLS / e["file"]).is_file()
    manifest_path = (TOOLS / e["manifest"]) if e["manifest"] else None
    manifest_present = bool(manifest_path and manifest_path.is_file())

    pins_present, pins_pending = [], []
    for p in e["pins"]:
        pin_name = p.split(" ")[0]  # tolerate "file.py (pending ...)" annotations
        (pins_present if (TOOLS / pin_name).is_file() else pins_pending).append(pin_name)

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
    """Analyzer-shaped files in tools/ that are neither registered nor declared
    non-analysis — a soft drift warning (does not fail the gate)."""
    known = {e["file"] for e in REGISTRY} | set(NON_ANALYSIS)
    out = []
    for f in sorted(TOOLS.glob("*.py")):
        if f.name in known or f.name.startswith("__"):
            continue
        if f.name.endswith("_runner.py") or f.name.endswith("_check.py") or "audit" in f.name:
            out.append(f.name)
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
    header = f"{'analyzer'.ljust(w_name)}  {'tier'.ljust(w_tier)}  man  pin  file"
    lines.append(header)
    lines.append("-" * len(header))
    for r in rows:
        man = "yes" if r["manifest_present"] else " no"
        pin = "yes" if r["has_pin"] else " no"
        fp = "ok" if r["file_present"] else "MISSING"
        flag = "  <- OVER-CLAIM" if r["claimed_tier"] != r["effective_tier"] else ""
        lines.append(
            f"{r['name'].ljust(w_name)}  {r['effective_tier'].ljust(w_tier)}  "
            f"{man}  {pin}  {fp}{flag}"
        )
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
            [sys.executable, str(TOOLS / "tolerance_stack.py"), job_path],
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
    # program (deterministic, sorted-keys). Status = the analyzer's registry tier.
    ghash = provenance.geometry_hash(program=chain_job)
    matver = provenance.material_db_version(str(TOOLS / "material_db.json"))
    envelope = provenance.stamp(
        values=receipt,
        geometry_hash=ghash,
        material_version=matver,
        analyzer_name="tolerance_stack",
        analyzer_version="1.0.0",
        validation_status=provenance.STATUS_CATALOGED,
        residual_or_convergence={
            "method": "closed-form worst-case + RSS",
            "iterative": False,
            "exact": True,
            "note": "no iteration; worst-case and RSS are exact for the 1-D "
                    "linear chain — 'convergence' is not applicable, reported "
                    "structurally rather than as a bare number.",
        },
        manifest_ref=None,  # Cataloged: no manifest — the envelope says so honestly
        geometry_relation=provenance.equality_relation(ghash),
    )
    return envelope


def _main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="LMCAD analyzer registry / graduation ledger.")
    ap.add_argument("--json", action="store_true", help="machine-readable resolution")
    ap.add_argument("--check", action="store_true", help="CI gate: exit 1 on any violation")
    ap.add_argument("--run-pins", action="store_true",
                    help="EXECUTE every Validated analyzer's pins; exit 1 on an "
                         "un-run or non-known-issue failure (needs ACE_PYTHON)")
    ap.add_argument("--demo", action="store_true", help="read-only stamp() demo on one analyzer")
    args = ap.parse_args(argv)

    rows = resolve_all()

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
