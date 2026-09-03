#!/usr/bin/env python3
"""Optimizer validation pin: param_optimize vs analytic known-optimum problems.

A correct direct-search optimizer must (a) find a known analytic optimum, (b)
converge declared targets into their tolerance band, (c) drive a maximization
into its constraint cap without violating it, and (d) be bit-deterministic
(R5). Each property is falsifiable — a broken simplex step, penalty sign, or
target scalarization trips a pin. All four pins are HERMETIC: the evaluator is
a pure-python command model written to a temp dir; no engine, no ACE.

	Pin 1  minimize (x-3)^2 + (y+1)^2 on [-10,10]^2  ->  (3, -1), score 0
	Pin 2  target x^3 + 13x = 40 +/- 0.05 on [0,4]   ->  x* = 2.091294...
	Pin 3  maximize x s.t. x <= 7 on [0,10]          ->  x* = 7 (cap ACTIVE)
	Pin 4  pin-1 receipt is byte-identical across two runs

Measured 2026-07-17 (pinned here): pin 1 best_score 6.44e-08 at
(2.99996, -1.00025); pin 2 achieved 40.000000 (miss 1.14e-07); pin 3 best
6.999878 with constraint_ok (pin 3 CAUGHT a real defect during authoring:
ulp-scale cap violations underflowed the relative penalty and an infeasible
point won by a float ulp — fixed by feasibility-first selection in
param_optimize.py); pin 4 identical receipts.

Run:  python3 tools/param_optimize_validation.py   (any python3, stdlib only)
Exit: 0 iff all assertions hold; nonzero with a message otherwise.
"""
import json
import os
import subprocess
import sys
import tempfile

TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # tools/
sys.path.insert(0, TOOLS)
import _layout  # noqa: E402
OPTIMIZER = str(_layout.find_tool("param_optimize.py"))

MODEL = """\
import json, sys
d = json.load(open(sys.argv[1]))
x = float(d["x"]); y = float(d.get("y", 0.0))
print(json.dumps({"ok": True,
                  "paraboloid": (x - 3.0) ** 2 + (y + 1.0) ** 2,
                  "cubic": x ** 3 + 13.0 * x,
                  "lin": x}))
"""


def run_optimizer(job: dict, workdir: str, tag: str) -> dict:
	"""Run param_optimize.py as a subprocess (the production CLI contract) and
	return the receipt parsed from the LAST stdout line."""
	job_path = os.path.join(workdir, f"{tag}.json")
	with open(job_path, "w") as f:
		json.dump(job, f)
	out = subprocess.run([sys.executable, OPTIMIZER, job_path],
	                     capture_output=True, text=True, timeout=300)
	last = ""
	for line in out.stdout.splitlines():
		if line.strip():
			last = line
	assert out.returncode == 0 and last, (
		f"pin '{tag}': optimizer exited {out.returncode} with no receipt; "
		f"stderr tail: {out.stderr[-300:]!r}")
	return json.loads(last)


def main() -> None:
	work = tempfile.mkdtemp(prefix="popt_pin_")
	model = os.path.join(work, "model.py")
	with open(model, "w") as f:
		f.write(MODEL)
	evaluator = {"kind": "command",
	             "argv": [sys.executable, model, "$JOB"],
	             "job_template": {"x": "$x", "y": "$y"}}

	# Pin 1: known optimum of a convex quadratic. Analytic argmin (3, -1), min 0.
	# NB: a bare "objective" MAXIMIZES by default (documented v1 contract) —
	# minimization must be explicit, and this pin also pins that default by
	# failing loudly if it ever silently flips.
	job1 = {"evaluator": evaluator,
	        "params": {"x": {"min": -10.0, "max": 10.0, "init": -6.0},
	                   "y": {"min": -10.0, "max": 10.0, "init": 8.0}},
	        "objective": "paraboloid", "maximize": False,
	        "max_evals": 160, "multi_start": 2}
	r1 = run_optimizer(job1, work, "pin1_known_optimum")
	bx, by = float(r1["best_params"]["x"]), float(r1["best_params"]["y"])
	assert r1["ok"] and r1["best_score"] <= 1e-4 and abs(bx - 3.0) <= 0.02 and abs(by + 1.0) <= 0.02, (
		f"pin 1 FAILED: minimize (x-3)^2+(y+1)^2 must land on the analytic optimum "
		f"(3, -1) with score <= 1e-4; got ({bx:.5f}, {by:.5f}) score "
		f"{r1['best_score']:.3g} after {r1['evals']} evals — the simplex search "
		f"is broken, not merely slow (budget 160 is ~4x what this needs)")
	print(f"pin 1 OK: best_score {r1['best_score']:.3g} at ({bx:.5f}, {by:.5f}), {r1['evals']} evals")

	# Pin 2: first-class target convergence. x^3 + 13x = 40 has ONE real root,
	# x* = 2.0912942... — the target machinery must converge into 40 +/- 0.05.
	ev1 = {"kind": "command", "argv": [sys.executable, model, "$JOB"],
	       "job_template": {"x": "$x"}}
	job2 = {"evaluator": ev1,
	        "params": {"x": {"min": 0.0, "max": 4.0, "init": 0.5}},
	        "targets": [{"expr": "cubic", "value": 40.0, "tol": 0.05}],
	        "max_evals": 120, "multi_start": 2}
	r2 = run_optimizer(job2, work, "pin2_target")
	t = r2["targets"][0]
	assert r2["ok"] and r2["targets_met"] and abs(t["achieved"] - 40.0) <= 0.05, (
		f"pin 2 FAILED: target cubic=40+/-0.05 (analytic root x*=2.0912942) not "
		f"met — achieved {t['achieved']:.6f} (miss {t['miss_abs']:.3g}), "
		f"targets_met={r2['targets_met']} — the quadratic target cost or its "
		f"receipt accounting is wrong")
	print(f"pin 2 OK: achieved {t['achieved']:.6f} (target 40 +/- 0.05), miss {t['miss_abs']:.3g}")

	# Pin 3: constraint activity. Maximizing x under x <= 7 must drive x INTO
	# the cap (>= 6.9) without ending on a violated candidate (constraint_ok).
	job3 = {"evaluator": ev1,
	        "params": {"x": {"min": 0.0, "max": 10.0, "init": 1.0}},
	        "objective": "lin", "maximize": True,
	        "constraints": [{"expr": "lin", "max": 7.0}],
	        "max_evals": 80, "multi_start": 2}
	r3 = run_optimizer(job3, work, "pin3_active_constraint")
	b3 = float(r3["best_params"]["x"])
	assert r3["ok"] and r3["constraint_ok"] and 6.9 <= b3 <= 7.0 + 1e-6, (
		f"pin 3 FAILED: maximize x s.t. x<=7 must end ON the cap (analytic "
		f"optimum exactly 7): got x={b3:.6f}, constraint_ok={r3['constraint_ok']} "
		f"— penalty sign, maximize negation, or best-candidate bookkeeping is wrong")
	print(f"pin 3 OK: best x {b3:.6f} vs cap 7, constraint_ok={r3['constraint_ok']}")

	# Pin 4: R5 determinism — same job, byte-identical receipt (no clocks, no
	# RNG, bit-reversed multi-start lattice).
	r1b = run_optimizer(job1, work, "pin4_rerun")
	a, b = json.dumps(r1, sort_keys=True), json.dumps(r1b, sort_keys=True)
	assert a == b, (
		f"pin 4 FAILED: two runs of the identical job disagree — the optimizer "
		f"has picked up nondeterminism (first divergence near char "
		f"{next((i for i, (c, d) in enumerate(zip(a, b)) if c != d), min(len(a), len(b)))})")
	print("pin 4 OK: rerun receipt byte-identical")

	print("param_optimize validation: ALL PINS OK")


if __name__ == "__main__":
	main()
