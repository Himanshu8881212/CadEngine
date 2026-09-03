#!/usr/bin/env python3
"""Acceptance test for the witness-selection drift detector (param_optimize Unit B).

Reproduces the diagnostic's OWN silent-wrong scenario end-to-end:
  box [0,0,0]-[40,20,$h], one fillet_edge_near at a FIXED witness [38,0,9] with a
  vertical decoy edge 2 mm away, objective = maximize volume so the search climbs $h.

BEFORE detection the optimizer returned ok:true, best_params h=30,
best_objective 23985.514 — a fillet silently rounded on the WRONG (decoy) edge,
reported as full success. AFTER detection the run must:
  * set selection_unstable: true (the fillet's resolved EdgeName changes at $h>=12),
  * attach selection_evidence with the two distinct edges, and
  * NOT report the decoy's 23985.514 as best_objective — best stays on the intended
    top-front edge (drifted candidates are rejected from the search).

Run:  python3 tools/param_optimize_drift_test.py   (exit 0 on pass, nonzero on fail)
Requires the release lmcad-mcp binary (cargo build --release --bin lmcad-mcp).
"""
import json
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # tools/
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import _layout  # noqa: E402

DECOY_OBJECTIVE = 23985.514  # what the WRONG edge yielded at h=30 before detection
DECOY_EDGE = (("Primitive", 2), ("Primitive", 5))   # the vertical decoy edge


def _identity(resolved_edge):
	return tuple((f["operand"], f["source_face"]) for f in resolved_edge["faces"])


def main() -> int:
	job = {
		"template": {"ops": [
			{"id": "b", "op": "box", "min": [0, 0, 0], "max": [40, 20, "$h"]},
			{"id": "f", "op": "fillet_edge_near", "in": "b", "witness": [38, 0, 9], "radius": 1.5},
			{"id": "vol", "op": "exact_volume", "in": "f"},
		]},
		"params": {"h": {"min": 10, "max": 30, "init": 10}},
		"objective": "vol.exact_volume",
	}
	with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as jf:
		json.dump(job, jf)
		job_path = jf.name
	try:
		out = subprocess.run([sys.executable, str(_layout.find_tool("param_optimize.py")), job_path],
		                     capture_output=True, text=True, timeout=600)
	finally:
		os.unlink(job_path)
	last = [ln for ln in out.stdout.splitlines() if ln.strip()][-1]
	r = json.loads(last)

	best = r.get("best_objective", float("nan"))
	best_edge = _identity(r.get("best_measures", {}).get("f", {}).get("resolved_edge", {"faces": []})) \
		if r.get("best_measures", {}).get("f", {}).get("resolved_edge") else None
	ev = (r.get("selection_evidence") or {}).get("f", {})
	distinct = ev.get("distinct_edges", 0)

	checks = [
		("run completed", r.get("ok") is True),
		("selection_unstable flag is set (the alarm rings)", r.get("selection_unstable") is True),
		("evidence names both distinct edges the witness latched", distinct == 2),
		(f"best_objective is NOT the decoy's {DECOY_OBJECTIVE} (got {best:.3f})",
		 abs(best - DECOY_OBJECTIVE) > 1.0),
		(f"best stayed on the intended edge, not the decoy {DECOY_EDGE} (got {best_edge})",
		 best_edge is not None and best_edge != DECOY_EDGE),
	]
	ok = True
	for label, passed in checks:
		print(("  PASS: " if passed else "  FAIL: ") + label)
		ok = ok and passed
	print("WITNESS-SELECTION DRIFT DETECTOR:", "PASS" if ok else "FAIL")
	if not ok:
		print("--- optimizer output ---\n" + json.dumps(r, indent=1), file=sys.stderr)
	return 0 if ok else 1


if __name__ == "__main__":
	raise SystemExit(main())
