#!/usr/bin/env python3
"""Universal parametric optimizer over the LMCAD engine (representation-agnostic).

Optimizes named numeric PARAMETERS of a work-order program template — B-rep dimensions,
implicit/TPMS constants, anything expressible as ops — against an OBJECTIVE computed from
the kernel's own measures. Works for every representation because it never touches
geometry directly: substitute params -> run the program -> read receipts -> iterate
(Nelder-Mead with bound clipping + constraint penalties).

Job JSON (argv[1]):
{
  "template": {"ops":[ ... any string "$name" is replaced by the param value ... ]},
  "params":   {"rim_t": {"min":2, "max":8, "init":4}, ...},
  "objective": "mp.inertia_diag[2] / mp.volume",   // measures via <op_id>.<key>; maximize
  "maximize": true,
  "constraints": [{"expr": "mp.volume", "max": 15000}],   // penalty if violated
  "max_evals": 40
}
Receipts (last stdout line): {ok, best_params, best_objective, best_measures, evals,
history_first, history_last, constraint_ok}. Selection is FEASIBILITY-FIRST:
best_params is the best candidate that satisfied every constraint whenever one
was seen; constraint_ok:false means the whole search never found a feasible eval.
All logging to stderr. Default evaluator: the LMCAD engine, one-shot per eval.

=== v2 (analysis audit 2026-07-17): one optimizer over ANY analyzer ===
All additive; every v1 job runs unchanged.

"evaluator": {"kind": "engine"}                       // default, engine template
             {"kind": "command",                      // ANY runnable analyzer —
              "argv": ["python3", "my_model.py", "$JOB"],   // a derived physics
              "job_template": { ...params via "$name"... }} // model, an external
    The command evaluator substitutes params into job_template, writes it to a
    temp json, replaces "$JOB" in argv with its path, runs the command, and
    parses the LAST stdout line as the receipt tree. Objective / constraint /
    target expressions then read THAT tree (dotted attribute access, e.g.
    "spl.ripple_db" or top-level "f3_hz") — physics-in-the-loop and
    geometry-in-the-loop share one optimizer. A receipt with ok:false is a
    failed eval (never silently scored).

"targets": [{"expr": "f3_hz", "value": 40.0, "tol": 1.0, "weight": 1.0}, ...]
    FIRST-CLASS convergence targets (the audit's step 5): each contributes
    weight * ((achieved - value)/tol)^2 to the cost, and the receipt reports
    per-target {expr, value, achieved, miss_abs, met} + targets_met. With
    targets present, "objective" becomes optional.

"objectives": [{"expr": "...", "weight": 1.0, "maximize": false}, ...]
    Weighted multi-objective. HONESTY: a weighted sum is NOT a Pareto front —
    each term is reported separately in the receipt so trade-offs stay
    visible, but only one scalarization is explored per run.

"multi_start": 4
    Deterministic extra Nelder-Mead starts (bit-reversed lattice points across
    the bounds — NO randomness, R5 determinism). max_evals is the TOTAL budget
    split across starts; receipt reports per-start bests.

"robust": {"tols": {"rim_t": 0.15, ...}, "aggregate": "worst"}
    Tolerance-aware optimization: every candidate is ALSO evaluated at the
    tolerance extremes of the named params (full 2^k corners for k<=3, else
    the 2k axis extremes — stated in the receipt) and scored on the WORST
    case; constraints must hold at every corner. "optimize nominal, hope at
    the extremes" becomes "optimize the worst case, with receipts".
"""
import copy, json, math, os, subprocess, sys

REPO = os.environ.get("LMCAD_ROOT", os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BIN = os.path.join(REPO, "target", "release", "kernel-api")
MCP_BIN = os.path.join(REPO, "target", "release", "lmcad-mcp")
OUT_DIR = os.environ.get("CADCODE_OUT_DIR", os.path.join(REPO, "studio_out", "mcp"))


def call_engine(program: dict, out_dir: str | None = None) -> dict:
	"""One-shot engine run returning the FULL report.

	Wire choice (audit 2026-07-16): the `kernel-api` CLI, not the MCP server —
	the MCP tool-result text is capped at 60 KiB to protect LLM contexts, which
	silently truncated large receipts (a `list_faces` of a real part) on the
	old wire. The CLI prints the whole report; exports land in the same
	`studio_out/mcp` tree (override with CADCODE_OUT_DIR). Falls back to the
	MCP one-shot when only that binary is built, with the cap caveat."""
	if os.path.exists(BIN):
		import tempfile

		with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
			json.dump(program, f)
			path = f.name
		try:
			out = subprocess.run(
				[BIN, "run", path, "--out-dir", out_dir or OUT_DIR],
				capture_output=True, text=True, timeout=300, env={**os.environ, "LMCAD_ROOT": REPO}, cwd=REPO,
			)
			if not out.stdout.strip():
				raise RuntimeError(f"engine gave no report: {out.stderr[:300]}")
			return json.loads(out.stdout)
		finally:
			os.unlink(path)
	# Fallback: the MCP one-shot (results capped at 60 KiB by design — build
	# target/release/kernel-api for uncapped tool runs).
	lines = "\n".join([
		json.dumps({"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {"protocolVersion": "2025-06-18", "capabilities": {}}}),
		json.dumps({"jsonrpc": "2.0", "method": "notifications/initialized"}),
		json.dumps({"jsonrpc": "2.0", "id": 2, "method": "tools/call", "params": {"name": "run_program", "arguments": {"program": program}}}),
	]) + "\n"
	out = subprocess.run([MCP_BIN], input=lines, capture_output=True, text=True, timeout=300, env={**os.environ, "LMCAD_ROOT": REPO}, cwd=REPO)
	for line in out.stdout.splitlines():
		if not line.strip():
			continue
		try:
			m = json.loads(line)
		except json.JSONDecodeError:
			continue
		if m.get("id") == 2:
			return json.loads(m["result"]["content"][0]["text"])
	raise RuntimeError(f"engine gave no response: {out.stderr[:300]}")


def substitute(node, values):
	if isinstance(node, dict):
		return {k: substitute(v, values) for k, v in node.items()}
	if isinstance(node, list):
		return [substitute(v, values) for v in node]
	if isinstance(node, str) and node.startswith("$") and node[1:] in values:
		return values[node[1:]]
	return node


class Measures:
	"""Attribute access to a measures/receipt tree: mp.volume,
	mp.inertia_diag[2], chain-nested chain.nominal_gap (dicts wrap recursively)."""
	def __init__(self, d):
		self._d = d or {}
	def __getattr__(self, k):
		if k in self._d:
			v = self._d[k]
			return Measures(v) if isinstance(v, dict) else v
		raise AttributeError(f"no measure '{k}' — available: {sorted(self._d)}")
	def __getitem__(self, k):
		v = self._d[k]
		return Measures(v) if isinstance(v, dict) else v


def eval_env(job, values):
	"""Run ONE candidate through the configured evaluator and return
	(expression_env, measures, err). engine: env maps op ids to Measures.
	command: env is the receipt tree itself (top-level keys + nested dicts)."""
	ev = job.get("evaluator") or {"kind": "engine"}
	kind = ev.get("kind", "engine")
	if kind == "engine":
		program = substitute(copy.deepcopy(job["template"]), values)
		report = call_engine(program)
		if not report.get("ok"):
			errs = [o.get("error") for o in report.get("ops", []) if o.get("error")]
			return None, None, f"program failed: {errs[:1]}"
		env = {op["id"]: Measures(op.get("measures")) for op in report["ops"]}
		env["math"] = math
		measures = {op["id"]: op.get("measures") for op in report["ops"] if op.get("measures")}
		return env, measures, None
	if kind == "command":
		import tempfile

		payload = substitute(copy.deepcopy(ev.get("job_template", {})), values)
		with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
			json.dump(payload, f)
			path = f.name
		try:
			argv = [path if a == "$JOB" else a for a in ev["argv"]]
			out = subprocess.run(argv, capture_output=True, text=True, timeout=float(ev.get("timeout", 300)), cwd=ev.get("cwd") or None)
			last = ""
			for line in out.stdout.splitlines():
				if line.strip():
					last = line
			if not last:
				return None, None, f"command produced no receipt: {out.stderr[-200:]}"
			try:
				receipt = json.loads(last)
			except json.JSONDecodeError:
				return None, None, f"command's last stdout line is not JSON: {last[:120]!r}"
			if isinstance(receipt, dict) and receipt.get("ok") is False:
				return None, None, f"analyzer refused: {json.dumps(receipt)[:200]}"
		finally:
			os.unlink(path)
		env = {k: (Measures(v) if isinstance(v, dict) else v) for k, v in receipt.items()}
		env["math"] = math
		return env, receipt, None
	return None, None, f"unknown evaluator kind {kind!r}"


def score_candidate(job, env):
	"""(score_min, parts, cons_ok) — the scalar the search minimizes, with the
	per-term receipts. Combines legacy `objective`, weighted `objectives`,
	quadratic `targets`, and relative constraint penalties."""
	parts = {}
	base = 0.0
	obj_legacy = None
	if job.get("objective"):
		obj_legacy = eval(job["objective"], {"__builtins__": {}}, env)
		base += (-1.0 if job.get("maximize", True) else 1.0) * obj_legacy
	multi = []
	for o in job.get("objectives", []):
		v = eval(o["expr"], {"__builtins__": {}}, env)
		w = float(o.get("weight", 1.0))
		base += (-w if o.get("maximize", False) else w) * v
		multi.append({"expr": o["expr"], "value": v, "weight": w, "maximize": bool(o.get("maximize", False))})
	targets = []
	for t in job.get("targets", []):
		achieved = eval(t["expr"], {"__builtins__": {}}, env)
		tol = float(t.get("tol", 1e-9)) or 1e-9
		miss = float(achieved) - float(t["value"])
		base += float(t.get("weight", 1.0)) * (miss / tol) ** 2
		targets.append({"expr": t["expr"], "value": t["value"], "tol": tol,
			"achieved": achieved, "miss_abs": abs(miss), "met": abs(miss) <= tol})
	if obj_legacy is None and not multi and not targets:
		raise ValueError("job needs at least one of 'objective', 'objectives', 'targets'")
	penalty, cons_ok = 0.0, True
	for c in job.get("constraints", []):
		v = eval(c["expr"], {"__builtins__": {}}, env)
		if "max" in c and v > c["max"]:
			penalty += v / c["max"] - 1.0
			cons_ok = False
		if "min" in c and v < c["min"]:
			penalty += c["min"] / max(v, 1e-12) - 1.0
			cons_ok = False
	parts = {"objective": obj_legacy, "objectives": multi, "targets": targets,
		"targets_met": all(t["met"] for t in targets) if targets else None}
	return base, penalty, cons_ok, parts


def robust_corners(job, values):
	"""The tolerance-extreme candidates for `robust.tols`: full 2^k corners for
	k<=3, else the 2k axis extremes (stated in the receipt)."""
	tols = (job.get("robust") or {}).get("tols") or {}
	names = [n for n in tols if n in values]
	if not names:
		return [], "none"
	if len(names) <= 3:
		corners = []
		for mask in range(1, 2 ** len(names)):
			c = dict(values)
			for i, n in enumerate(names):
				c[n] = values[n] + (tols[n] if (mask >> i) & 1 else -tols[n])
			corners.append(c)
		# the all-minus corner:
		c = dict(values)
		for n in names:
			c[n] = values[n] - tols[n]
		corners.append(c)
		# dedup
		seen, out = set(), []
		for c in corners:
			key = tuple(sorted(c.items()))
			if key not in seen:
				seen.add(key)
				out.append(c)
		return out, f"full 2^{len(names)} corners"
	corners = []
	for n in names:
		for sgn in (-1.0, 1.0):
			c = dict(values)
			c[n] = values[n] + sgn * tols[n]
			corners.append(c)
	return corners, f"axis extremes (2x{len(names)})"


def evaluate(job, values):
	"""Score one candidate — nominal, plus tolerance corners when `robust` is
	set (aggregate: WORST score; constraints must hold everywhere)."""
	env, measures, err = eval_env(job, values)
	if err:
		return None, None, err
	base, penalty, cons_ok, parts = score_candidate(job, env)
	if job.get("robust"):
		corners, scheme = robust_corners(job, values)
		worst = base
		for c in corners:
			cenv, _, cerr = eval_env(job, c)
			if cerr:
				return None, None, f"robust corner {c} failed: {cerr}"
			cbase, cpen, cok, _ = score_candidate(job, cenv)
			penalty += cpen
			cons_ok = cons_ok and cok
			worst = max(worst, cbase)
		parts["robust"] = {"scheme": scheme, "corners": len(corners), "nominal_score": base, "worst_score": worst}
		base = worst
	return (base, penalty, cons_ok, parts), measures, None


def edge_identity(resolved_edge):
	"""Canonical, hashable identity of a resolved EdgeName: its ordered face-pair
	(operand, source_face). EdgeName::new already canonicalises the pair order, so
	the SAME physical edge compares equal across candidates that reran the same op
	sequence with substituted params — which is exactly what makes cross-candidate
	comparison a valid drift detector."""
	faces = (resolved_edge or {}).get("faces", [])
	return tuple((f.get("operand"), f.get("source_face")) for f in faces)


def witness_features(measures):
	"""(op_id, resolved_edge) for every witness-selected feature in one candidate's
	measures — i.e. every op that left a resolved_edge receipt (kernel-api Unit A:
	fillet_edge_near / chamfer_edge_near)."""
	for op_id, m in (measures or {}).items():
		if isinstance(m, dict) and "resolved_edge" in m:
			yield op_id, m["resolved_edge"]


def main():
	job = json.load(open(sys.argv[1]))
	names = list(job["params"])
	lo = [job["params"][n]["min"] for n in names]
	hi = [job["params"][n]["max"] for n in names]
	x0 = [job["params"][n].get("init", (l + h) / 2) for n, l, h in zip(names, lo, hi)]
	sign = -1.0 if job.get("maximize", True) else 1.0
	state = {"evals": 0, "best": None, "history": [], "witness_edges": {}}

	def cost(x):
		xv = [min(max(v, l), h) for v, l, h in zip(x, lo, hi)]
		values = dict(zip(names, xv))
		state["evals"] += 1
		res, measures, err = evaluate(job, values)
		if err:
			print(f"eval {state['evals']}: {values} -> {err}", file=sys.stderr)
			return 1e12
		(obj, penalty, cons_ok, parts) = res
		# Selection-stability: a witness picks its edge by nearest spatial point, so a
		# swept dimension that MOVES the intended edge can silently latch a different
		# one. Record each witness-feature's resolved EdgeName and compare it to the
		# baseline (first candidate). Drift means the objectives are NOT mutually
		# comparable — some rounded a decoy — so reject the drifted candidate from the
		# search AND stamp the whole run (below, in main). Detection is loud, not skippable.
		we = state["witness_edges"]
		drifted = []
		for op_id, re_obj in witness_features(measures):
			ident = edge_identity(re_obj)
			rec = we.setdefault(op_id, {"baseline": ident, "seen": {}})
			rec["seen"].setdefault(ident, {"params": dict(values), "resolved_edge": re_obj})
			if ident != rec["baseline"]:
				drifted.append(op_id)
		# Objective-relative penalty: a 10% violation must dominate ANY objective magnitude
		# (a fixed scale silently accepted infeasible optima on large objectives — beyblade run 1).
		# `obj` is already the sign-folded minimization scalar from score_candidate.
		score = obj + penalty * 10.0 * (1.0 + abs(obj))
		hist = {"params": values, "score": obj, "constraint_ok": cons_ok, "selection_drifted": bool(drifted)}
		if parts.get("objective") is not None:
			hist["objective"] = parts["objective"]
		if parts.get("targets"):
			hist["targets_met"] = parts["targets_met"]
		state["history"].append(hist)
		if drifted:
			# Reject: this objective is on a DIFFERENT edge than the baseline and is not
			# comparable, so it must never become `best` and the search must be pushed
			# back onto the baseline edge. Ranked worse than any feasible point but above
			# the 1e12 hard-error sentinel (a build failure is still worse than a drift).
			print(f"eval {state['evals']}: {values} -> score {obj:.6g} REJECTED: selection drift on {drifted}", file=sys.stderr)
			return 1e11
		# Feasibility-first selection: a feasible candidate ALWAYS outranks an
		# infeasible one, regardless of score. The penalty steers the WALK, but
		# at ulp-scale violations (x = cap + 1e-15) it underflows against the
		# objective magnitude and an infeasible point can win by a float ulp
		# (caught by param_optimize_validation.py pin 3). constraint_ok:false
		# in the receipt therefore means the search never saw a feasible eval.
		sel = (0 if cons_ok else 1, score)
		if state["best"] is None or sel < state["best"][0]:
			state["best"] = (sel, values, obj, measures, cons_ok, parts)
		print(f"eval {state['evals']}: {values} -> score {obj:.6g} (cons_ok={cons_ok})", file=sys.stderr)
		return score

	# Deterministic multi-start (v2): x0 plus bit-reversed lattice points across
	# the bounds — reproducible (R5), no randomness. The eval budget is TOTAL.
	def bit_reversed(i, base=2):
		f, r = 1.0, 0.0
		while i > 0:
			f /= base
			r += f * (i % base)
			i //= base
		return r

	starts = [list(x0)]
	n_starts = max(1, int(job.get("multi_start", 1)))
	for k in range(1, n_starts):
		frac = bit_reversed(k + 1)
		starts.append([l + (0.15 + 0.7 * ((frac + j * 0.37) % 1.0)) * (h - l) for j, (l, h) in enumerate(zip(lo, hi))])
	budget = int(job.get("max_evals", 40))
	per_start = max(4, budget // len(starts))
	start_bests = []
	try:
		from scipy.optimize import minimize
		for x_start in starts:
			before = state["best"][0] if state["best"] else None
			minimize(cost, x_start, method="Nelder-Mead", options={"maxfev": per_start, "xatol": 1e-3, "fatol": 1e-9})
			start_bests.append({"start": dict(zip(names, x_start)),
				"improved": state["best"] is not None and (before is None or state["best"][0] < before)})
	except ImportError:  # coordinate sweep fallback — still honest, just slower
		for i, n in enumerate(names):
			for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
				x = list(x0)
				x[i] = lo[i] + frac * (hi[i] - lo[i])
				cost(x)
	if state["best"] is None:
		print(json.dumps({"ok": False, "error": "no successful evaluation"}))
		return
	_, values, obj, measures, cons_ok, parts = state["best"]
	# Selection-stability verdict: if any witness-feature resolved to more than one
	# distinct EdgeName across the swept candidates, the run's objectives are not
	# mutually comparable — best_objective above is the best candidate that stayed on
	# the baseline edge, but the flag + evidence MUST travel with it (Rule 5: silence
	# is the bug). Additive: every prior field is kept; consumers gain two new keys.
	unstable = {op: rec for op, rec in state["witness_edges"].items() if len(rec["seen"]) > 1}
	out = {
		"ok": True, "best_params": values,
		# Legacy field: the single-`objective` value when one was given, else the
		# minimized combined score (documented; consumers of v1 jobs see no change).
		"best_objective": parts.get("objective") if parts.get("objective") is not None else obj,
		"best_score": obj,
		"constraint_ok": cons_ok,
		"best_measures": measures, "evals": state["evals"],
		"history_first": state["history"][0], "history_last": state["history"][-1],
		"selection_unstable": bool(unstable),
	}
	if parts.get("targets"):
		out["targets"] = parts["targets"]
		out["targets_met"] = parts["targets_met"]
	if parts.get("objectives"):
		out["objectives"] = parts["objectives"]
	if parts.get("robust"):
		out["robust"] = parts["robust"]
	if len(starts) > 1:
		out["multi_start"] = {"starts": len(starts), "per_start_evals": per_start, "bests": start_bests}
	if unstable:
		out["selection_evidence"] = {
			op: {
				"distinct_edges": len(rec["seen"]),
				"note": "witness latched >1 distinct edge across the sweep — objectives are NOT "
				        "mutually comparable; drifted candidates were rejected so best_objective "
				        "stays on the baseline (first-evaluated) edge, but this run is selection-unstable.",
				"edges": [
					{"resolved_edge": v["resolved_edge"], "first_seen_at_params": v["params"]}
					for v in rec["seen"].values()
				],
			}
			for op, rec in unstable.items()
		}
	import _receipt

	_receipt.emit(out, job, "param_optimize")


if __name__ == "__main__":
	try:
		main()
	except Exception as e:  # honest failure receipt, never a stack-trace-as-contract
		print(json.dumps({"ok": False, "error": str(e)}))
		sys.exit(1)
