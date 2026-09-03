#!/usr/bin/env python3
"""Validation pin: tolerance_stack.py vs hand-derived textbook worst-case + RSS stacks.

The analyzer is closed-form arithmetic, so its ground truth is a stack anyone
can redo on paper. Every expected number below is DERIVED IN THIS FILE, in a
comment, from the standard formulas (Fischer, *Mechanical Tolerance Stackup and
Analysis*, 2nd ed., CRC 2011; Drake, *Dimensioning and Tolerancing Handbook*,
McGraw-Hill 1999 — worst-case: T_wc = sum|t_i|; RSS: T_rss = sqrt(sum t_i^2)
with each +/-t taken as a 3-sigma band of an independent normal contributor).
A wrong sign, a double-counted asymmetric tolerance, a sigma convention that
drifts, or an exit code that disagrees with `ok` trips a pin.

	Pin 1  symmetric 3-element chain: worst-case AND RSS bands, ranking
	Pin 2  asymmetric element: RSS mid-shift applied, worst-case about the TRUE nominal
	Pin 3  worst-case FAILS while RSS PASSES (the textbook reason RSS exists) -> ok:false, exit 2
	Pin 4  fit mode: interference at extremes (exit 2) and a clearing fit (exit 0)
	Pin 5  a chain with no `closes` REFUSES by name (exit 2, refusal.missing_closes)
	Pin 6  determinism: the stdout receipt is byte-identical across two runs

All pins are HERMETIC: stdlib only, the analyzer is driven through its real
CLI (subprocess, last stdout line = the receipt, `$?` = the contract).

Measured 2026-09-02 (pinned here): every chain/fit number matched the hand
derivation to 0.0 (receipt rounding is 9 decimals); pin 3 exited 2 with
pass_worst false / pass_rss true; pin 6 receipts identical.

Run:  python3 tools/tolerance_stack_validation.py   (any python3, stdlib only)
Exit: 0 iff all assertions hold; nonzero with a message otherwise.
"""
import json
import math
import os
import subprocess
import sys
import tempfile

TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # tools/
sys.path.insert(0, TOOLS)
import _layout  # noqa: E402
TOOL = str(_layout.find_tool("tolerance_stack.py"))
TOL = 1e-9  # the receipt rounds to 9 decimals; the arithmetic itself is exact


def run(job: dict, workdir: str, tag: str):
	"""Drive the production CLI. Returns (receipt from the LAST stdout line, exit code, raw line)."""
	job_path = os.path.join(workdir, f"{tag}.json")
	with open(job_path, "w") as f:
		json.dump(job, f)
	env = dict(os.environ)
	env["LMCAD_RECEIPT_DRY_RUN"] = "1"  # a pin must never write receipts anywhere
	out = subprocess.run([sys.executable, TOOL, job_path], capture_output=True,
	                     text=True, timeout=120, env=env)
	last = ""
	for line in out.stdout.splitlines():
		if line.strip():
			last = line
	assert last, f"pin '{tag}': no receipt on stdout; stderr tail: {out.stderr[-300:]!r}"
	return json.loads(last), out.returncode, last


def close(a, b, what):
	assert abs(float(a) - float(b)) <= TOL, f"{what}: got {a!r}, hand derivation says {b!r}"


def main() -> None:
	work = tempfile.mkdtemp(prefix="tolstack_pin_")

	# ------------------------------------------------------------------
	# Pin 1 — symmetric chain (the module docstring's own example).
	#   housing_depth 20.0 +/-0.20 (dir +1)
	#   bearing_stack 12.0 +/-0.10 (dir -1)
	#   spacer         7.5 +/-0.15 (dir -1)
	# nominal gap  = 20.0 - 12.0 - 7.5 = 0.5
	# worst-case   = sum|t| = 0.20 + 0.10 + 0.15 = 0.45  -> [0.05, 0.95]
	# RSS          = sqrt(0.20^2 + 0.10^2 + 0.15^2) = sqrt(0.0725) = 0.269258240...
	#                sigma_gap = 0.269258240/3 = 0.089752747...
	#                band = 0.5 +/- 0.269258240 -> [0.230741760, 0.769258240]
	# closes {0.02, 1.0}: both bands inside -> pass_worst, pass_rss, ok, exit 0.
	# ranking by band width (plus+minus): housing 0.40 (44.4%), spacer 0.30 (33.3%),
	#                                     bearing 0.20 (22.2%)  of 0.90 total.
	# ------------------------------------------------------------------
	chain = [
		{"name": "housing_depth", "nominal": 20.0, "tol": 0.20, "dir": 1},
		{"name": "bearing_stack", "nominal": 12.0, "tol": 0.10, "dir": -1},
		{"name": "spacer", "nominal": 7.5, "tol": 0.15, "dir": -1},
	]
	r1, rc1, _ = run({"chain": chain, "closes": {"min_required": 0.02, "max_allowed": 1.0}},
	                 work, "pin1_symmetric_chain")
	c = r1["chain"]
	rss_half = math.sqrt(0.20 ** 2 + 0.10 ** 2 + 0.15 ** 2)  # sqrt(0.0725)
	close(c["nominal_gap"], 0.5, "pin 1 nominal_gap = 20 - 12 - 7.5")
	close(c["worst_min"], 0.05, "pin 1 worst_min = 0.5 - 0.45")
	close(c["worst_max"], 0.95, "pin 1 worst_max = 0.5 + 0.45")
	close(c["rss_nominal_gap"], 0.5, "pin 1 rss_nominal_gap (symmetric: no mid-shift)")
	close(c["rss_sigma_gap"], rss_half / 3.0, "pin 1 rss_sigma_gap = sqrt(0.0725)/3")
	close(c["rss_min"], 0.5 - rss_half, "pin 1 rss_min = 0.5 - sqrt(0.0725)")
	close(c["rss_max"], 0.5 + rss_half, "pin 1 rss_max = 0.5 + sqrt(0.0725)")
	assert c["pass_worst"] is True and c["pass_rss"] is True and r1["ok"] is True and rc1 == 0, (
		f"pin 1 FAILED: both bands lie inside [0.02, 1.0] so the stack must pass with exit 0; "
		f"got pass_worst={c['pass_worst']} pass_rss={c['pass_rss']} ok={r1['ok']} exit={rc1}")
	names = [x["name"] for x in c["contributors"]]
	assert names == ["housing_depth", "spacer", "bearing_stack"], (
		f"pin 1 FAILED: contributors must rank by band width 0.40 > 0.30 > 0.20, got {names}")
	close(c["contributors"][0]["pct_of_band"], round(100.0 * 0.40 / 0.90, 1), "pin 1 housing pct_of_band")
	assert c["closes"]["sides_checked"] == ["min_required", "max_allowed"], (
		f"pin 1 FAILED: both sides were given, receipt says {c['closes']['sides_checked']}")
	print(f"pin 1 OK: nominal {c['nominal_gap']}, worst [{c['worst_min']}, {c['worst_max']}], "
	      f"rss [{c['rss_min']}, {c['rss_max']}] (3 sigma = sqrt(0.0725) = {rss_half:.9f})")

	# ------------------------------------------------------------------
	# Pin 2 — asymmetric element (ball F4 regression, re-derived).
	#   A = 10.0 +0.00/-0.10 (dir +1),  B = 9.0 exact (dir -1)
	# A in [9.90, 10.00], B = 9.00  -> gap in [0.90, 1.00], nominal 1.00 (worst-case
	# is about the TRUE nominal: hi = plus_A = 0, lo = minus_A = 0.10).
	# RSS: equal-bilateral conversion of A: mid-shift = dir*(plus-minus)/2 = -0.05,
	#      t_eq = (0+0.10)/2 = 0.05  -> rss_nominal = 0.95, sigma = 0.05/3,
	#      band = 0.95 +/- 0.05 -> [0.90, 1.00]  (one contributor: RSS == worst-case).
	# ------------------------------------------------------------------
	r2, rc2, _ = run({"chain": [
		{"name": "A", "nominal": 10.0, "tol": {"plus": 0.0, "minus": 0.10}, "dir": 1},
		{"name": "B", "nominal": 9.0, "tol": 0.0, "dir": -1}],
		"closes": {"min_required": 0.85, "max_allowed": 1.05}}, work, "pin2_asymmetric")
	c = r2["chain"]
	close(c["nominal_gap"], 1.0, "pin 2 nominal_gap = 10 - 9 (TRUE nominal, no shift)")
	close(c["worst_min"], 0.90, "pin 2 worst_min = 1.0 - 0.10")
	close(c["worst_max"], 1.00, "pin 2 worst_max = 1.0 + 0.0 (asymmetric: NOT 1.05)")
	close(c["rss_nominal_gap"], 0.95, "pin 2 rss_nominal_gap = 1.0 - 0.05 mid-shift")
	close(c["rss_sigma_gap"], 0.05 / 3.0, "pin 2 rss_sigma_gap = t_eq/3 = 0.05/3")
	close(c["rss_min"], 0.90, "pin 2 rss_min = 0.95 - 0.05")
	close(c["rss_max"], 1.00, "pin 2 rss_max = 0.95 + 0.05")
	assert "asymmetric_note" in c and r2["ok"] is True and rc2 == 0, (
		f"pin 2 FAILED: an asymmetric chain must carry asymmetric_note and pass here "
		f"(note present={'asymmetric_note' in c}, ok={r2['ok']}, exit={rc2})")
	print(f"pin 2 OK: worst [{c['worst_min']}, {c['worst_max']}] about nominal {c['nominal_gap']}; "
	      f"rss about {c['rss_nominal_gap']} (mid-shift -0.05)")

	# ------------------------------------------------------------------
	# Pin 3 — the textbook divergence: worst-case FAILS, RSS PASSES.
	# Same chain as pin 1 with min_required 0.10: worst_min 0.05 < 0.10 (fail),
	# rss_min 0.2307 >= 0.10 (pass). ok = pass_worst AND pass_rss = false -> exit 2.
	# ------------------------------------------------------------------
	r3, rc3, _ = run({"chain": chain, "closes": {"min_required": 0.10, "max_allowed": 1.0}},
	                 work, "pin3_worst_fails_rss_passes")
	c = r3["chain"]
	assert c["pass_worst"] is False and c["pass_rss"] is True, (
		f"pin 3 FAILED: worst_min 0.05 < 0.10 must fail while rss_min {c['rss_min']} >= 0.10 "
		f"passes; got pass_worst={c['pass_worst']} pass_rss={c['pass_rss']}")
	assert r3["ok"] is False and rc3 == 2 and r3.get("exit_code") == 2, (
		f"pin 3 FAILED: a failed stack must be ok:false with exit 2 (ran and failed), "
		f"got ok={r3['ok']} exit={rc3} exit_code={r3.get('exit_code')}")
	print(f"pin 3 OK: pass_worst False (0.05 < 0.10), pass_rss True ({c['rss_min']:.6f} >= 0.10), exit 2")

	# ------------------------------------------------------------------
	# Pin 4 — fit mode.
	#   bore 8.2 +/-0.15, shaft 8.0 +/-0.15 (the cookbook's verified gotcha):
	#   min_clearance = (8.2-0.15) - (8.0+0.15) = 8.05 - 8.15 = -0.10 (INTERFERENCE)
	#   max_clearance = (8.2+0.15) - (8.0-0.15) = 8.35 - 7.85 = +0.50
	#   nominal = 0.20 -> ok:false, exit 2.
	#   bore 8.5 +/-0.15 vs shaft 8.0 +/-0.15: min 8.35-8.15 = 0.20, max 8.65-7.85 = 0.80 -> ok.
	# ------------------------------------------------------------------
	r4, rc4, _ = run({"fit": {"bore": {"nominal": 8.2, "tol": 0.15}, "shaft": {"nominal": 8.0, "tol": 0.15}}},
	                 work, "pin4_fit_interferes")
	f = r4["fit"]
	close(f["nominal_clearance"], 0.20, "pin 4 nominal_clearance = 8.2 - 8.0")
	close(f["min_clearance"], -0.10, "pin 4 min_clearance = 8.05 - 8.15")
	close(f["max_clearance"], 0.50, "pin 4 max_clearance = 8.35 - 7.85")
	close(f["extremes"]["max_shaft"], 8.15, "pin 4 max_shaft")
	close(f["extremes"]["min_bore"], 8.05, "pin 4 min_bore")
	assert f["interference_at_extremes"] is True and r4["ok"] is False and rc4 == 2, (
		f"pin 4 FAILED: min clearance -0.10 is an interference -> ok:false exit 2; "
		f"got interference={f['interference_at_extremes']} ok={r4['ok']} exit={rc4}")
	r4b, rc4b, _ = run({"fit": {"bore": {"nominal": 8.5}, "shaft": {"nominal": 8.0}}},
	                   work, "pin4_fit_clears")  # tol omitted -> printer default 0.15
	f = r4b["fit"]
	close(r4b["printer_tol_default"], 0.15, "pin 4 printer_tol_default")
	close(f["min_clearance"], 0.20, "pin 4b min_clearance = 8.35 - 8.15")
	close(f["max_clearance"], 0.80, "pin 4b max_clearance = 8.65 - 7.85")
	assert f["interference_at_extremes"] is False and r4b["ok"] is True and rc4b == 0, (
		f"pin 4b FAILED: 0.5 mm nominal clearance clears at +/-0.15 -> ok exit 0; got "
		f"ok={r4b['ok']} exit={rc4b}")
	print(f"pin 4 OK: 8.2/8.0 fit clearance [{r4['fit']['min_clearance']}, {r4['fit']['max_clearance']}] "
	      f"interferes (exit 2); 8.5/8.0 clears [{f['min_clearance']}, {f['max_clearance']}] (exit 0)")

	# ------------------------------------------------------------------
	# Pin 5 — a chain without `closes` is REFUSED by name, not indexed to a KeyError.
	# ------------------------------------------------------------------
	r5, rc5, _ = run({"chain": chain}, work, "pin5_missing_closes")
	assert r5["ok"] is False and rc5 == 2 and r5.get("error_kind") == "refusal.missing_closes", (
		f"pin 5 FAILED: chain mode with no `closes` must refuse with refusal.missing_closes "
		f"and exit 2; got ok={r5['ok']} exit={rc5} kind={r5.get('error_kind')}")
	print(f"pin 5 OK: missing closes -> {r5['error_kind']}, exit 2")

	# ------------------------------------------------------------------
	# Pin 6 — determinism: no clocks, no paths in the stdout line -> byte-identical.
	# ------------------------------------------------------------------
	_, _, line_a = run({"chain": chain, "closes": {"min_required": 0.02, "max_allowed": 1.0}},
	                   work, "pin6_run_a")
	_, _, line_b = run({"chain": chain, "closes": {"min_required": 0.02, "max_allowed": 1.0}},
	                   work, "pin6_run_b")
	assert line_a == line_b, (
		"pin 6 FAILED: two runs of the identical job produced different stdout receipts — "
		"the analyzer has picked up nondeterminism")
	print("pin 6 OK: rerun receipt byte-identical")

	print("tolerance_stack validation: ALL PINS OK")


if __name__ == "__main__":
	main()
