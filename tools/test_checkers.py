#!/usr/bin/env python3
"""test_checkers.py — falsifiable pins for the fit/motion checkers + the optimizer.

Every pin here FAILS against the pre-2026-08-08 sources; each names the friction
entry it closes. Hermetic: pure arithmetic or a pure-python command evaluator
written to a temp dir. The engine-backed pins (sweep_check) are skipped, loudly,
when `target/release/kernel-api` is not built.

Exit codes follow the shared contract in tools/_receipt.py:
  0 ok:true | 1 the tool could not run the request | 2 it ran and REFUSED.

Run:  python3 tools/test_checkers.py      (any python3, stdlib only)
Exit: 0 iff every pin holds.
"""
import json
import os
import subprocess
import sys
import tempfile

TOOLS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TOOLS)
FAILURES = []
PASSES = []


def check(name, cond, detail=""):
	(PASSES if cond else FAILURES).append(name)
	print(f"  {'PASS' if cond else 'FAIL'}: {name}{(' — ' + detail) if detail else ''}")


EXIT_OK, EXIT_ERROR, EXIT_REFUSED = 0, 1, 2


def run_tool(tool, job, workdir, tag, extra_argv=(), env=None):
	"""(receipt, returncode) from the production CLI contract: LAST stdout line."""
	path = os.path.join(workdir, f"{tag}.json")
	with open(path, "w") as f:
		json.dump(job, f)
	out = subprocess.run([sys.executable, os.path.join(TOOLS, tool), path, *extra_argv],
	                     capture_output=True, text=True, timeout=600,
	                     env={**os.environ, **(env or {})})
	last = ""
	for line in out.stdout.splitlines():
		if line.strip():
			last = line
	return (json.loads(last) if last else None), out.returncode


# ---------------------------------------------------------------------------
# tolerance_stack.py
# ---------------------------------------------------------------------------
def test_tolerance_stack(wd):
	print("tolerance_stack.py")

	# T11 / ball F4 — the asymmetric worst-case double count.
	# A = 10.0 +0/-0.10 (dir +1), B = 9.0 exact (dir -1).
	# By hand: A in [9.90, 10.00], B == 9.0, so gap in [0.90, 1.00], nominal 1.00.
	r, rc = run_tool("tolerance_stack.py", {
		"chain": [{"name": "A", "nominal": 10.0, "tol": {"plus": 0.0, "minus": 0.10}, "dir": 1},
		          {"name": "B", "nominal": 9.0, "tol": 0.0, "dir": -1}],
		"closes": {"min_required": 0.0, "max_allowed": 5.0}}, wd, "asym")
	c = r["chain"]
	check("asymmetric worst-case band is about the TRUE nominal (ball F4)",
	      (c["nominal_gap"], c["worst_min"], c["worst_max"]) == (1.0, 0.9, 1.0),
	      f"got nominal {c['nominal_gap']} worst [{c['worst_min']}, {c['worst_max']}]")
	check("the RSS mid-shift is reported separately, not folded into the worst case",
	      c["rss_nominal_gap"] == 0.95 and "asymmetric_note" in c)

	# The dangerous direction: worst_max used to be OPTIMISTIC, so a max_allowed
	# check passed on a band that was really wider.
	r, rc = run_tool("tolerance_stack.py", {
		"chain": [{"name": "A", "nominal": 10.0, "tol": {"plus": 0.10, "minus": 0.0}, "dir": 1},
		          {"name": "B", "nominal": 9.0, "tol": 0.0, "dir": -1}],
		"closes": {"min_required": 0.0, "max_allowed": 1.05}}, wd, "asym_hi")
	check("an asymmetric stack that overruns max_allowed is now CAUGHT",
	      r["chain"]["worst_max"] == 1.1 and r["chain"]["pass_worst"] is False
	      and rc == EXIT_REFUSED,
	      f"worst_max {r['chain']['worst_max']} pass_worst {r['chain']['pass_worst']} rc {rc}")

	# Symmetric stacks — the whole shipped portfolio — must be untouched.
	r, _ = run_tool("tolerance_stack.py", {
		"chain": [{"name": "h", "nominal": 20.0, "tol": 0.2, "dir": 1},
		          {"name": "b", "nominal": 12.0, "tol": 0.1, "dir": -1},
		          {"name": "s", "nominal": 7.5, "tol": {"plus": 0.15, "minus": 0.15}, "dir": -1}],
		"closes": {"min_required": 0.02, "max_allowed": 1.0}}, wd, "sym")
	c = r["chain"]
	check("symmetric stacks are numerically unchanged (backward compat)",
	      (c["nominal_gap"], c["worst_min"], c["worst_max"]) == (0.5, 0.05, 0.95)
	      and c["rss_nominal_gap"] == c["nominal_gap"] and "asymmetric_note" not in c,
	      f"{c['nominal_gap']} [{c['worst_min']}, {c['worst_max']}]")

	# T11 / din_rail F1 — one-sided `closes` used to leak KeyError: 'max_allowed'
	# and the persisted receipt held nothing but that string.
	r, rc = run_tool("tolerance_stack.py", {
		"chain": [{"name": "d", "nominal": 1.4, "tol": 0.15, "dir": 1}],
		"closes": {"min_required": 1.0}}, wd, "onesided")
	check("a one-sided `closes` is first class, not a KeyError (din_rail F1)",
	      r["ok"] is True and r["chain"]["closes"]["sides_checked"] == ["min_required"]
	      and r["chain"]["worst_min"] == 1.25,
	      json.dumps(r.get("error", ""))[:80])

	# ...but a typo must not become an unbounded limit.
	r, rc = run_tool("tolerance_stack.py", {
		"chain": [{"name": "d", "nominal": 1.4, "tol": 0.15}],
		"closes": {"min_requred": 1.0}}, wd, "typo")
	check("a misspelled `closes` key is REFUSED, never silently unbounded",
	      r["ok"] is False and r.get("error_kind") == "refusal.bad_closes"
	      and rc == EXIT_REFUSED, f"{r.get('error_kind')} rc {rc}")

	# T3 / gripper F9 — exit code must carry the verdict.
	_, rc_fail = run_tool("tolerance_stack.py", {
		"chain": [{"name": "d", "nominal": 1.4, "tol": 0.15}],
		"closes": {"min_required": 3.0, "max_allowed": 4.0}}, wd, "failing")
	_, rc_pass = run_tool("tolerance_stack.py", {
		"chain": [{"name": "d", "nominal": 1.4, "tol": 0.15}],
		"closes": {"min_required": 1.0, "max_allowed": 2.0}}, wd, "passing")
	check("a failing verdict exits nonzero and a passing one exits 0 (gripper F9)",
	      (rc_fail, rc_pass) == (EXIT_REFUSED, EXIT_OK),
	      f"fail rc {rc_fail}, pass rc {rc_pass}")

	# T11 / singulator F14 + cleat F7 — the clobber class.
	shipped = os.path.join(wd, "receipts", "shipped.json")
	job = {"chain": [{"name": "d", "nominal": 1.4, "tol": 0.15}],
	       "closes": {"min_required": 0.3, "max_allowed": 4.0}, "receipt": shipped}
	run_tool("tolerance_stack.py", job, wd, "ship")
	baseline = open(shipped).read()
	whatif = dict(job, closes={"min_required": 3.0, "max_allowed": 4.0})
	run_tool("tolerance_stack.py", whatif, wd, "whatif", env={"LMCAD_RECEIPT_DRY_RUN": "1"})
	check("a dry run writes nothing, so a what-if probe cannot mutate evidence (cleat F7)",
	      open(shipped).read() == baseline)
	r, rc = run_tool("tolerance_stack.py", whatif, wd, "whatif2",
	                 ["--out", os.path.join(wd, "elsewhere.json")])
	check("--out colliding with a job `receipt` key is REFUSED (singulator F14)",
	      r["ok"] is False and r.get("error_kind") == "receipt_path_conflict"
	      and rc != EXIT_OK, f"{r.get('error_kind')} rc {rc}")
	check("...and neither file was written during the refusal",
	      open(shipped).read() == baseline and not os.path.exists(os.path.join(wd, "elsewhere.json")))
	r, rc = run_tool("tolerance_stack.py", job, wd, "badflag", ["--outt", "x"])
	check("a mistyped flag is refused, never ignored",
	      r["ok"] is False and rc != EXIT_OK, f"{r.get('error_kind')} rc {rc}")


# ---------------------------------------------------------------------------
# joint_check.py
# ---------------------------------------------------------------------------
def test_joint_check(wd):
	print("joint_check.py")
	good = {"name": "good", "type": "machine_screw_into_heatset", "size": "M3",
	        "material": "pla", "loads": {"tension_N": 50}}
	fail = {"name": "shallow", "type": "screw_into_plastic_thread", "size": "M5",
	        "material": "pla", "loads": {"tension_N": 10, "sustained": True},
	        "engagement_mm": 2}
	m6 = {"name": "m6", "type": "screw_into_plastic_thread", "size": "M6",
	      "material": "pla", "loads": {"tension_N": 10}, "engagement_mm": 8}

	_, rc_pass = run_tool("joint_check.py", {"joints": [good]}, wd, "jc_pass")
	r_fail, rc_fail = run_tool("joint_check.py", {"joints": [fail]}, wd, "jc_fail")
	check("a genuine FAIL verdict exits nonzero, a PASS exits 0 (ball F5 — INVERTED before)",
	      (rc_pass, rc_fail) == (EXIT_OK, EXIT_REFUSED) and r_fail["ok"] is False,
	      f"pass rc {rc_pass}, fail rc {rc_fail}")

	r, rc = run_tool("joint_check.py", {"joints": [m6]}, wd, "jc_m6")
	j = r["joints"][0]
	check("an out-of-table size is a NAMED refusal, not a raw KeyError (ball F5)",
	      j.get("error_kind") == "refusal.size_not_in_table" and j["capacity_N"] is None
	      and r.get("refused_joints") == ["m6"] and rc == EXIT_REFUSED,
	      json.dumps(r)[:100])

	r, _ = run_tool("joint_check.py", {"joints": [good, m6]}, wd, "jc_mixed")
	byname = {x["name"]: x for x in r["joints"]}
	check("a refused joint does not throw away the other joints' evidence",
	      byname["good"]["SF_actual"] == 8.0
	      and byname["m6"]["error_kind"] == "refusal.size_not_in_table"
	      and r["ok"] is False)

	r, _ = run_tool("joint_check.py", {"joints": [dict(good, material="unobtainium")]},
	                wd, "jc_mat")
	check("an out-of-table material is refused by name too",
	      r["joints"][0].get("error_kind") == "refusal.material_not_in_table")

	# Backward compatibility: the shipped receipt shape for a working joint.
	r, _ = run_tool("joint_check.py", {"joints": [good], "safety_factor": 2.0}, wd, "jc_shape")
	j = r["joints"][0]
	check("working joints keep their receipt shape and numbers",
	      j["governing_mode"] == "heatset_pullout" and j["capacity_N"] == 400.0
	      and j["SF_actual"] == 8.0 and j["pass"] is True and set(j["modes"]) == {
		      "heatset_pullout", "plastic_bearing_shear", "screw_tension_steel",
		      "screw_shear_steel"})

	r, rc = run_tool("joint_check.py", {"joints": [dict(good, loads={"tension_N": -100})]}, wd, "jc_negative")
	check("negative load magnitudes are refused rather than turned into an infinite safety factor",
	      r["ok"] is False and r["joints"][0].get("error_kind") == "refusal.invalid_load"
	      and rc == EXIT_REFUSED,
	      json.dumps(r)[:160])


# ---------------------------------------------------------------------------
# param_optimize.py
# ---------------------------------------------------------------------------
SMOOTH = """\
import json, sys
d = json.load(open(sys.argv[1]))
x = float(d["x"])
print(json.dumps({"ok": True, "f": (x - 3.0) ** 2, "g": max(0.0, x - 2.0), "lin": x}))
"""

QUANTIZED = """\
import json, math, sys
d = json.load(open(sys.argv[1]))
x = math.floor(float(d["x"]) / 4.0) * 4.0     # a coarse in-loop discretiser
y = float(d["y"])
print(json.dumps({"ok": True, "f": (x - 7.0) ** 2 + (y - 1.0) ** 2}))
"""


def test_param_optimize(wd):
	print("param_optimize.py")
	smooth = os.path.join(wd, "smooth_model.py")
	open(smooth, "w").write(SMOOTH)
	quant = os.path.join(wd, "quant_model.py")
	open(quant, "w").write(QUANTIZED)

	def job_1d(constraint):
		return {"params": {"x": {"min": 0.0, "max": 10.0, "init": 5.0}},
		        "evaluator": {"kind": "command",
		                      "argv": [sys.executable, smooth, "$JOB"],
		                      "job_template": {"x": "$x"}},
		        "objective": "f", "maximize": False,
		        "constraints": [constraint], "max_evals": 12}

	# T12 / turgo F3 — `max: 0.0` is the natural spelling of "steep_area == 0"
	# and "warnings == 0"; it used to kill the whole run with a ZeroDivisionError.
	r, rc = run_tool("param_optimize.py", job_1d({"expr": "g", "max": 0.0}), wd, "opt_zero")
	check("a constraint with max 0.0 runs instead of dividing by zero (turgo F3)",
	      r["ok"] is True and rc == EXIT_OK, json.dumps(r.get("error", ""))[:120])

	# Found while fixing it: a NEGATIVE bound inverted the penalty sign, so the
	# optimizer was rewarded for violating the constraint and ran to the far end.
	r, _ = run_tool("param_optimize.py", job_1d({"expr": "lin", "max": -1.0}), wd, "opt_neg")
	check("a negative bound penalises violation instead of rewarding it",
	      r["best_params"]["x"] < 10.0 and r["constraint_ok"] is False,
	      f"best x {r['best_params']['x']}")

	# Positive bounds must be bit-for-bit what they were (shipped receipts).
	r, _ = run_tool("param_optimize.py", {
		"params": {"x": {"min": 0.0, "max": 10.0, "init": 1.0}},
		"evaluator": {"kind": "command", "argv": [sys.executable, smooth, "$JOB"],
		              "job_template": {"x": "$x"}},
		"objective": "lin", "maximize": True,
		"constraints": [{"expr": "lin", "max": 7.0}], "max_evals": 40}, wd, "opt_cap")
	check("a positive-bound cap still binds exactly as before (validation pin 3)",
	      r["constraint_ok"] is True and 6.9 < r["best_params"]["x"] <= 7.0,
	      f"best x {r['best_params']['x']}")

	# T12 / rotor F8 — the silent quantization.
	qjob = {"params": {"x": {"min": 0.0, "max": 16.0, "init": 6.0},
	                   "y": {"min": 0.0, "max": 4.0, "init": 2.0}},
	        "evaluator": {"kind": "command", "argv": [sys.executable, quant, "$JOB"],
	                      "job_template": {"x": "$x", "y": "$y"}},
	        "objective": "f", "maximize": False, "max_evals": 20}
	r, _ = run_tool("param_optimize.py", qjob, wd, "opt_quant")
	q = r.get("quantization")
	check("a quantizing evaluator is NAMED in the receipt with a measured resolution (rotor F8)",
	      bool(q) and q["detected"] is True
	      and q["effective_resolution_lower_bound_per_param"].get("x", 0) > 0.0
	      and "evidence" in q,
	      json.dumps(q)[:150] if q else "no `quantization` key")

	# ...and a smooth evaluator must NOT be accused (no crying wolf, and shipped
	# receipts of clean runs keep their exact key set).
	r, _ = run_tool("param_optimize.py", {
		"params": {"x": {"min": -10.0, "max": 10.0, "init": 0.0}},
		"evaluator": {"kind": "command", "argv": [sys.executable, smooth, "$JOB"],
		              "job_template": {"x": "$x"}},
		"objective": "f", "maximize": False, "max_evals": 40}, wd, "opt_smooth")
	check("a smooth evaluator is NOT flagged (a symmetric objective is a level set, "
	      "not a plateau)", "quantization" not in r, str(sorted(r)))

	# T12 / din_rail F9 — `evals` is a count under a plural name.
	check("`n_evals` is published alongside the legacy `evals` count (din_rail F9)",
	      r["n_evals"] == r["evals"] and isinstance(r["n_evals"], int))

	# T4 — station/candidate programs must not go to a system temp dir.
	pd = os.path.join(wd, "progdir")
	os.makedirs(pd, exist_ok=True)
	spy = os.path.join(wd, "spy_model.py")
	open(spy, "w").write(
		"import json, os, sys\n"
		"print(json.dumps({'ok': True, 'f': 1.0, 'dir': os.path.dirname(os.path.abspath(sys.argv[1]))}))\n")
	r, _ = run_tool("param_optimize.py", {
		"params": {"x": {"min": 0.0, "max": 1.0, "init": 0.5}},
		"program_dir": pd,
		"evaluator": {"kind": "command", "argv": [sys.executable, spy, "$JOB"],
		              "job_template": {"x": "$x"}},
		"objective": "f", "maximize": False, "max_evals": 5}, wd, "opt_pdir")
	check("candidate jobs are materialised under the caller's dir, not a system temp dir "
	      "(gripper F4 / turgo F7 / rotor F11)",
	      os.path.realpath(r["best_measures"]["dir"]) == os.path.realpath(pd),
	      r["best_measures"]["dir"])
	check("...and the scratch file is cleaned up afterwards",
	      not [f for f in os.listdir(pd) if f.startswith("_lmcad_station_")],
	      str(os.listdir(pd)))

	# Silence: a run where every evaluation fails must refuse, persist, and exit 1.
	bad = os.path.join(wd, "bad_model.py")
	open(bad, "w").write("import sys; sys.exit(3)\n")
	rcpt = os.path.join(wd, "noeval_receipt.json")
	r, rc = run_tool("param_optimize.py", {
		"params": {"x": {"min": 0.0, "max": 1.0, "init": 0.5}},
		"receipt": rcpt,
		"evaluator": {"kind": "command", "argv": [sys.executable, bad, "$JOB"],
		              "job_template": {"x": "$x"}},
		"objective": "f", "maximize": False, "max_evals": 5}, wd, "opt_dead")
	check("a run with no successful evaluation refuses, persists a receipt, and exits nonzero",
	      r["ok"] is False and r.get("error_kind") == "refusal.no_successful_evaluation"
	      and rc == EXIT_REFUSED and os.path.exists(rcpt),
	      f"{r.get('error_kind')} rc {rc}")

	# A command's JSON line is not success if the process itself failed.
	exit9 = os.path.join(wd, "exit9_model.py")
	open(exit9, "w").write(
		"import json, sys\nprint(json.dumps({'ok': True, 'f': 1.0}))\nsys.exit(9)\n")
	r, rc = run_tool("param_optimize.py", {
		"params": {"x": {"min": 0.0, "max": 1.0, "init": 0.5}},
		"evaluator": {"kind": "command", "argv": [sys.executable, exit9, "$JOB"],
		              "job_template": {"x": "$x"}},
		"objective": "f", "maximize": False, "max_evals": 3}, wd, "opt_exit9")
	check("an evaluator that prints success JSON then exits nonzero is rejected",
	      r["ok"] is False and r.get("error_kind") == "refusal.no_successful_evaluation"
	      and rc == EXIT_REFUSED, json.dumps(r)[:160])

	# Expressions are data selectors/arithmetic, never a Python execution surface.
	marker = os.path.join(wd, "optimizer_expression_owned")
	exploit = (
		"[c for c in ().__class__.__mro__[1].__subclasses__() "
		"if c.__name__=='catch_warnings'][0]()._module.__builtins__['__import__']"
		f"('pathlib').Path({marker!r}).write_text('owned')"
	)
	r, rc = run_tool("param_optimize.py", {
		"params": {"x": {"min": 0.0, "max": 1.0, "init": 0.5}},
		"evaluator": {"kind": "command", "argv": [sys.executable, smooth, "$JOB"],
		              "job_template": {"x": "$x"}},
		"objective": exploit, "maximize": False, "max_evals": 1}, wd, "opt_expr_attack")
	check("optimizer expressions cannot execute Python or touch the filesystem",
	      r["ok"] is False and not os.path.exists(marker) and rc != EXIT_OK,
	      json.dumps({"error_kind": r.get("error_kind"), "marker": os.path.exists(marker)}))

	nan_model = os.path.join(wd, "nan_model.py")
	open(nan_model, "w").write(
		"import json\nprint(json.dumps({'ok': True, 'f': float('nan')}))\n")
	r, rc = run_tool("param_optimize.py", {
		"params": {"x": {"min": 0.0, "max": 1.0, "init": 0.5}},
		"evaluator": {"kind": "command", "argv": [sys.executable, nan_model, "$JOB"],
		              "job_template": {"x": "$x"}},
		"objective": "f", "maximize": False, "max_evals": 2}, wd, "opt_nan")
	check("non-finite evaluator outputs are rejected instead of entering the search",
	      r["ok"] is False and rc == EXIT_REFUSED, json.dumps(r)[:160])


# ---------------------------------------------------------------------------
# sweep_check.py  (engine-backed)
# ---------------------------------------------------------------------------
def sweep_job(out_dir, x_from, x_to):
	"""Two boxes; the moving one slides along +X. `$t` is the moving box's x."""
	return {
		"template": {"ops": [
			{"id": "base", "op": "box", "min": [0, 0, 0], "max": [10, 10, 10]},
			{"id": "mover", "op": "box", "min": ["$t", 3, 3], "max": [20, 7, 7]},
			{"id": "fit", "op": "clearance", "a": "base", "b": "mover"},
		]},
		"t": {"from": x_from, "to": x_to, "steps": 5},
		"watch": ["fit"],
		"out_dir": out_dir,
	}


def test_sweep_check(wd):
	print("sweep_check.py")
	if not os.path.exists(os.path.join(REPO, "target", "release", "kernel-api")):
		check("sweep_check pins (SKIPPED: target/release/kernel-api not built)", True)
		return
	clear = os.path.join(wd, "sweep_clear")
	r, rc = run_tool("sweep_check.py", sweep_job(clear, 12.0, 16.0), wd, "sw_clear")
	check("a clear sweep passes and exits 0",
	      r["ok"] is True and rc == EXIT_OK, json.dumps(r)[:160])
	check("`failed_stations` is ALWAYS a list, never absent (din_rail F3)",
	      r["failed_stations"] == [] and r["interfering_watches"] == [])
	check("the receipt states what a sweep can and cannot prove",
	      "free-motion proof" in r["sweep_semantics"].lower()
	      and "does not fit" in r["sweep_semantics"].lower()
	      and "exact_volume" in r["sweep_semantics"])

	steady = os.path.join(wd, "sweep_steady")
	r, rc = run_tool("sweep_check.py", sweep_job(steady, 2.0, 6.0), wd, "sw_steady")
	check("an all-stations-interfering sweep is REFUSED by name, not returned as a "
	      "tidy ok:false (docs/FRICTION.md #27)",
	      r["ok"] is False and r.get("error_kind") == "refusal.no_free_station"
	      and r["watches"]["fit"]["all_stations_interfering"] is True
	      and rc == EXIT_REFUSED,
	      json.dumps({k: r.get(k) for k in ("ok", "error_kind")})[:160])

	r, rc = run_tool("sweep_check.py", {"watch": ["fit"], "out_dir": clear,
	                                    "template": {"ops": []}}, wd, "sw_no_t")
	check("a missing `t` block is a named refusal, not a KeyError",
	      r["ok"] is False and r.get("error_kind") == "refusal.missing_t"
	      and rc == EXIT_REFUSED, f"{r.get('error_kind')} rc {rc}")


def main():
	with tempfile.TemporaryDirectory(prefix="checker_pins_") as wd:
		test_tolerance_stack(wd)
		test_joint_check(wd)
		test_param_optimize(wd)
		test_sweep_check(wd)
	print()
	if FAILURES:
		print(f"CHECKER PINS: {len(FAILURES)} FAILED / {len(PASSES)} passed")
		for f in FAILURES:
			print(f"  - {f}")
		return 1
	print(f"CHECKER PINS: ALL {len(PASSES)} OK")
	return 0


if __name__ == "__main__":
	sys.exit(main())
