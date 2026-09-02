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
  "max_evals": 40,
  "program_dir": "…"        // optional; see PATHS below
}
CLI: python3 param_optimize.py <job.json> [--out PATH]. Persistence + exit
codes: the shared contract in tools/_receipt.py (--out wins over a job
`receipt` key and a disagreement is REFUSED; `LMCAD_RECEIPT_DRY_RUN=1`
suppresses every write; the exit code agrees with `ok`).

Receipts (last stdout line): {ok, best_params, best_objective, best_measures, evals,
n_evals, history_first, history_last, constraint_ok}. Selection is FEASIBILITY-FIRST:
best_params is the best candidate that satisfied every constraint whenever one
was seen; constraint_ok:false means the whole search never found a feasible eval.
`evals` is a COUNT; `n_evals` is the same count under an unambiguous name (a
roll-up doing len(receipt["evals"]) used to raise TypeError — din_rail F9).
All logging to stderr. Default evaluator: the LMCAD engine, one-shot per eval.

CONSTRAINT BOUNDS may be zero or negative: `{"max": 0.0}` is the natural
spelling of "no steep area" / "no warnings" and no longer divides by zero, and
a negative bound no longer inverts the penalty sign. See `relative_violation`.

PATHS. A candidate program / analyzer job is materialised under
`job["program_dir"]`, else `job["out_dir"]`, else the JOB FILE's own directory —
never a system temp dir, so relative `import_step` / `load_part` paths in a
template resolve the same way they do in the job that carries them (gripper F4,
turgo F7, rotor F11). The scratch file is removed after each eval.

EFFECTIVE RESOLUTION. When two evals return a bit-identical score at different
parameter vectors, the evaluator is discretising the search space (a voxel
grid, a mesh seed, a rounded input). The receipt then carries a `quantization`
block with the measured per-parameter lower bound on that resolution and the
evidence pair, and a loud stderr warning — a converged-looking optimum can be a
plateau of the discretiser (rotor F8). Declare a known pitch as
`"params": {"x": {"min":…, "max":…, "resolution": 4.0}}` and search steps below
it are flagged as well.

=== v2 (analysis audit 2026-07-17): one optimizer over ANY analyzer ===
All additive; every v1 job runs unchanged.

"evaluator": {"kind": "engine"}                       // default, engine template
             {"kind": "command",                      // ANY runnable analyzer —
              "argv": ["python3", "my_model.py", "$JOB"],   // a derived physics
              "job_template": { ...params via "$name"... }, // model, an external
              "timeout": 1800,                        // SECONDS PER CANDIDATE,
                                                      //   default 300 — a real
                                                      //   solver loop needs it
                                                      //   raised or every eval
                                                      //   dies (cubesat F6)
              "cwd": "…"}                             // working dir for argv
    The command evaluator substitutes params into job_template, writes it under
    the job's program_dir (see PATHS), replaces "$JOB" in argv with its path,
    runs the command with `timeout` seconds per candidate, and
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
import ast, copy, json, math, operator, os, subprocess, sys

REPO = os.environ.get("LMCAD_ROOT", os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BIN = os.path.join(REPO, "target", "release", "kernel-api")
MCP_BIN = os.path.join(REPO, "target", "release", "lmcad-mcp")
OUT_DIR = os.environ.get("CADCODE_OUT_DIR", os.path.join(REPO, "studio_out", "mcp"))
ANALYZER_VERSION = "param_optimize/safe-ast-nelder-mead/v3"


def station_dir(job: dict, job_path: str | None = None) -> str:
	"""Where a substituted (station / candidate) program is materialised.

	T4 / gripper F4 / turgo F7 / rotor F11: the substituted program used to go to
	the SYSTEM temp dir. `import_step` / `load_part` resolve their `file`
	against the PROGRAM FILE's directory and refuse `..`, so every relative path
	in a swept or optimized template became unresolvable — the two contracts are
	individually reasonable and jointly exclude the feature. Resolution order,
	most explicit first:

	  job["program_dir"]  ->  job["out_dir"]  ->  the JOB FILE's own directory
	  ->  CADCODE_OUT_DIR / studio_out/mcp (only when there is no job at all)

	so a template's relative paths mean the same thing they mean in the job that
	carries them, and a re-run in a fresh checkout resolves identically."""
	for key in ("program_dir", "out_dir"):
		v = (job or {}).get(key)
		if isinstance(v, str) and v:
			return v
	if job_path:
		return os.path.dirname(os.path.abspath(job_path)) or "."
	return OUT_DIR


_STATION_SEQ = [0]


def _materialize(program: dict, program_dir: str) -> str:
	"""Write `program` into `program_dir` and return the path. Deterministic
	directory (that is what path resolution depends on); the basename carries
	pid+counter only so concurrent runs cannot collide, and the file is removed
	by the caller."""
	os.makedirs(program_dir, exist_ok=True)
	_STATION_SEQ[0] += 1
	path = os.path.join(program_dir, f"_lmcad_station_{os.getpid()}_{_STATION_SEQ[0]}.json")
	with open(path, "w") as f:
		json.dump(program, f)
	return path


def call_engine(program: dict, out_dir: str | None = None, program_dir: str | None = None) -> dict:
	"""One-shot engine run returning the FULL report.

	Wire choice (audit 2026-07-16): the `kernel-api` CLI, not the MCP server —
	the MCP tool-result text is capped at 60 KiB to protect LLM contexts, which
	silently truncated large receipts (a `list_faces` of a real part) on the
	old wire. The CLI prints the whole report; exports land in the same
	`studio_out/mcp` tree (override with CADCODE_OUT_DIR). Falls back to the
	MCP one-shot when only that binary is built, with the cap caveat.

	`program_dir` is the directory the substituted program is written to, i.e.
	the root every relative `import_step`/`load_part` path in it resolves
	against — see `station_dir`. Default (None) keeps the legacy system-temp
	behaviour so no existing caller changes; every caller in this tree now
	passes one."""
	if os.path.exists(BIN):
		import tempfile

		if program_dir:
			path = _materialize(program, program_dir)
		else:
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
			report = json.loads(out.stdout)
			if not isinstance(report, dict) or not isinstance(report.get("ok"), bool):
				raise RuntimeError("engine report must be an object with boolean `ok`")
			_assert_finite_tree(report, "$engine_report")
			if out.returncode != 0 and report.get("ok") is True:
				raise RuntimeError(f"engine exited {out.returncode} despite an ok:true report")
			return report
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
	if out.returncode != 0:
		raise RuntimeError(f"MCP evaluator exited {out.returncode}: {out.stderr[-300:]}")
	for line in out.stdout.splitlines():
		if not line.strip():
			continue
		try:
			m = json.loads(line)
		except json.JSONDecodeError:
			continue
		if m.get("id") == 2:
			report = json.loads(m["result"]["content"][0]["text"])
			if not isinstance(report, dict) or not isinstance(report.get("ok"), bool):
				raise RuntimeError("MCP engine report must be an object with boolean `ok`")
			_assert_finite_tree(report, "$engine_report")
			return report
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


_BIN_OPS = {
	ast.Add: operator.add, ast.Sub: operator.sub, ast.Mult: operator.mul,
	ast.Div: operator.truediv, ast.FloorDiv: operator.floordiv,
	ast.Mod: operator.mod, ast.Pow: operator.pow,
}
_UNARY_OPS = {ast.UAdd: operator.pos, ast.USub: operator.neg}
_MATH_NAMES = {
	name for name in (
		"acos", "asin", "atan", "atan2", "ceil", "cos", "degrees", "exp",
		"fabs", "floor", "hypot", "log", "log10", "radians", "sin", "sqrt", "tan"
	) if hasattr(math, name)
}
_SAFE_FUNCTIONS = {"abs": abs, "min": min, "max": max}


def _finite_number(value, label="expression result") -> float:
	"""Return a finite scalar float; booleans/containers are not objectives."""
	if isinstance(value, bool) or not isinstance(value, (int, float)):
		raise ValueError(f"{label} must be a numeric scalar, got {type(value).__name__}")
	value = float(value)
	if not math.isfinite(value):
		raise ValueError(f"{label} must be finite, got {value!r}")
	return value


def _assert_finite_tree(node, path="$receipt"):
	"""Reject NaN/Infinity anywhere in an analyzer receipt."""
	if isinstance(node, float) and not math.isfinite(node):
		raise ValueError(f"{path} is non-finite ({node!r})")
	if isinstance(node, dict):
		for key, value in node.items():
			_assert_finite_tree(value, f"{path}.{key}")
	elif isinstance(node, list):
		for i, value in enumerate(node):
			_assert_finite_tree(value, f"{path}[{i}]")


def safe_numeric_expr(expression: str, env: dict) -> float:
	"""Evaluate the optimizer's tiny numeric expression language.

	Supported: receipt names, dotted fields, numeric list subscripts, finite
	constants, + - * / // % **, unary +/- and an allowlist of ``math`` functions.
	No Python calls, comprehensions, dunder attributes, imports or object graph
	introspection are reachable.
	"""
	if not isinstance(expression, str) or not expression.strip():
		raise ValueError("expression must be a non-empty string")
	try:
		tree = ast.parse(expression, mode="eval")
	except SyntaxError as exc:
		raise ValueError(f"invalid numeric expression: {exc.msg}") from exc
	if sum(1 for _ in ast.walk(tree)) > 128:
		raise ValueError("numeric expression is too complex (maximum 128 syntax nodes)")

	def visit(node):
		if isinstance(node, ast.Expression):
			return visit(node.body)
		if isinstance(node, ast.Constant):
			if isinstance(node.value, bool) or not isinstance(node.value, (int, float)):
				raise ValueError("only finite numeric constants are allowed")
			return _finite_number(node.value, "numeric constant")
		if isinstance(node, ast.Name):
			if node.id in env and node.id != "math":
				return env[node.id]
			if node.id in _SAFE_FUNCTIONS:
				return _SAFE_FUNCTIONS[node.id]
			raise ValueError(f"unknown or forbidden name {node.id!r}")
		if isinstance(node, ast.Attribute):
			if node.attr.startswith("_"):
				raise ValueError("private/dunder attributes are forbidden")
			if isinstance(node.value, ast.Name) and node.value.id == "math":
				if node.attr in _MATH_NAMES or node.attr in {"pi", "e", "tau"}:
					return getattr(math, node.attr)
				raise ValueError(f"math.{node.attr} is not allowed")
			base = visit(node.value)
			if not isinstance(base, Measures):
				raise ValueError("attribute access is allowed only on receipt fields")
			return getattr(base, node.attr)
		if isinstance(node, ast.Subscript):
			base = visit(node.value)
			index = visit(node.slice)
			if isinstance(index, float) and index.is_integer():
				index = int(index)
			if not isinstance(base, (Measures, list, tuple, dict)):
				raise ValueError("subscripts are allowed only on receipt arrays/objects")
			return base[index]
		if isinstance(node, ast.BinOp) and type(node.op) in _BIN_OPS:
			left = _finite_number(visit(node.left), "left operand")
			right = _finite_number(visit(node.right), "right operand")
			if isinstance(node.op, ast.Pow) and abs(right) > 32:
				raise ValueError("power exponents are limited to |32|")
			return _finite_number(_BIN_OPS[type(node.op)](left, right), "arithmetic result")
		if isinstance(node, ast.UnaryOp) and type(node.op) in _UNARY_OPS:
			return _finite_number(_UNARY_OPS[type(node.op)](_finite_number(visit(node.operand))), "unary result")
		if isinstance(node, ast.Call):
			if node.keywords:
				raise ValueError("keyword arguments are not allowed")
			fn = visit(node.func)
			allowed = set(_SAFE_FUNCTIONS.values()) | {getattr(math, n) for n in _MATH_NAMES}
			if fn not in allowed:
				raise ValueError("function call is not allowlisted")
			args = [_finite_number(visit(arg), "function argument") for arg in node.args]
			return _finite_number(fn(*args), "function result")
		raise ValueError(f"forbidden expression syntax: {type(node).__name__}")

	try:
		return _finite_number(visit(tree))
	except (AttributeError, IndexError, KeyError, TypeError, ZeroDivisionError, OverflowError) as exc:
		raise ValueError(f"numeric expression could not be evaluated: {exc}") from exc


def eval_env(job, values, program_dir=None):
	"""Run ONE candidate through the configured evaluator and return
	(expression_env, measures, err). engine: env maps op ids to Measures.
	command: env is the receipt tree itself (top-level keys + nested dicts).

	`program_dir` roots the materialised candidate program / analyzer job, so
	its relative paths mean what they mean in the job file (see `station_dir`)."""
	ev = job.get("evaluator") or {"kind": "engine"}
	kind = ev.get("kind", "engine")
	if kind == "engine":
		program = substitute(copy.deepcopy(job["template"]), values)
		report = call_engine(program, program_dir=program_dir)
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
		if program_dir:
			path = _materialize(payload, program_dir)
		else:
			with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
				json.dump(payload, f)
				path = f.name
		try:
			argv = [path if a == "$JOB" else a for a in ev["argv"]]
			if not argv or not all(isinstance(a, str) and a for a in argv):
				return None, None, "command evaluator argv must be a non-empty list of non-empty strings"
			try:
				out = subprocess.run(argv, capture_output=True, text=True,
					timeout=float(ev.get("timeout", 300)), cwd=ev.get("cwd") or None)
			except subprocess.TimeoutExpired:
				return None, None, f"command timed out after {float(ev.get('timeout', 300))} seconds"
			last = ""
			for line in out.stdout.splitlines():
				if line.strip():
					last = line
			if not last:
				return None, None, f"command produced no receipt: {out.stderr[-200:]}"
			if out.returncode != 0:
				return None, None, (
					f"command exited {out.returncode}; stdout JSON cannot override process failure: "
					f"{out.stderr[-200:]}"
				)
			try:
				receipt = json.loads(last)
			except json.JSONDecodeError:
				return None, None, f"command's last stdout line is not JSON: {last[:120]!r}"
			if not isinstance(receipt, dict):
				return None, None, f"command receipt must be a JSON object, got {type(receipt).__name__}"
			if receipt.get("ok") is not True:
				return None, None, f"analyzer refused: {json.dumps(receipt)[:200]}"
			try:
				_assert_finite_tree(receipt)
			except ValueError as exc:
				return None, None, f"analyzer receipt is invalid: {exc}"
		finally:
			os.unlink(path)
		env = {k: (Measures(v) if isinstance(v, dict) else v) for k, v in receipt.items()}
		env["math"] = math
		return env, receipt, None
	return None, None, f"unknown evaluator kind {kind!r}"


def relative_violation(v, bound, side):
	"""Scale-free, always-POSITIVE penalty for a violated constraint.

	The relative form `v/bound - 1` (max side) is exact and cheap, but it is
	only defined and only correctly SIGNED for a strictly positive bound:

	  * `max: 0.0` raised `ZeroDivisionError` and killed the whole run —
	    yet `steep_area == 0` and `warnings == 0` (DELIVERABLE_SPEC 2.5 / 2.3b)
	    are naturally written exactly that way (turgo F3);
	  * a NEGATIVE bound flipped the sign, so the optimizer was *rewarded* for
	    violating it (found 2026-08-08: `{"max": -1.0}` drove the parameter to
	    the far end of its box and still reported `ok: true`).

	For a positive bound the legacy expression is kept BIT-FOR-BIT, so every
	shipped optimizer receipt reproduces. Otherwise the violation is normalised
	by the largest magnitude in play — dimensionless, positive, and continuous
	across bound == 0."""
	if side == "max":
		if bound > 0.0:
			return v / bound - 1.0
		return (v - bound) / max(abs(bound), abs(v), 1e-12)
	if bound > 0.0:
		return bound / max(v, 1e-12) - 1.0
	return (bound - v) / max(abs(bound), abs(v), 1e-12)


def score_candidate(job, env):
	"""(score_min, parts, cons_ok) — the scalar the search minimizes, with the
	per-term receipts. Combines legacy `objective`, weighted `objectives`,
	quadratic `targets`, and relative constraint penalties."""
	parts = {}
	base = 0.0
	obj_legacy = None
	if job.get("objective"):
		obj_legacy = safe_numeric_expr(job["objective"], env)
		base += (-1.0 if job.get("maximize", True) else 1.0) * obj_legacy
	multi = []
	for o in job.get("objectives", []):
		v = safe_numeric_expr(o["expr"], env)
		w = _finite_number(o.get("weight", 1.0), "objective weight")
		base += (-w if o.get("maximize", False) else w) * v
		multi.append({"expr": o["expr"], "value": v, "weight": w, "maximize": bool(o.get("maximize", False))})
	targets = []
	for t in job.get("targets", []):
		achieved = safe_numeric_expr(t["expr"], env)
		tol = _finite_number(t.get("tol", 1e-9), "target tolerance")
		if tol <= 0.0:
			raise ValueError("target tolerance must be > 0")
		value = _finite_number(t["value"], "target value")
		miss = achieved - value
		base += _finite_number(t.get("weight", 1.0), "target weight") * (miss / tol) ** 2
		targets.append({"expr": t["expr"], "value": t["value"], "tol": tol,
			"achieved": achieved, "miss_abs": abs(miss), "met": abs(miss) <= tol})
	if obj_legacy is None and not multi and not targets:
		raise ValueError("job needs at least one of 'objective', 'objectives', 'targets'")
	penalty, cons_ok = 0.0, True
	for c in job.get("constraints", []):
		v = safe_numeric_expr(c["expr"], env)
		if "max" in c:
			bound = _finite_number(c["max"], "constraint max")
			if v > bound:
				penalty += relative_violation(v, bound, "max")
				cons_ok = False
		if "min" in c:
			bound = _finite_number(c["min"], "constraint min")
			if v < bound:
				penalty += relative_violation(v, bound, "min")
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


def evaluate(job, values, program_dir=None):
	"""Score one candidate — nominal, plus tolerance corners when `robust` is
	set (aggregate: WORST score; constraints must hold everywhere)."""
	env, measures, err = eval_env(job, values, program_dir)
	if err:
		return None, None, err
	base, penalty, cons_ok, parts = score_candidate(job, env)
	if job.get("robust"):
		corners, scheme = robust_corners(job, values)
		worst = base
		for c in corners:
			cenv, _, cerr = eval_env(job, c, program_dir)
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


def _contradicted(evals, names, n, a, b, score):
	"""True when some eval BETWEEN a and b along `n` (all other parameters
	equal) scored differently.

	A discretiser gives a flat PLATEAU: everything between the two points scores
	the same. A symmetric objective gives a LEVEL SET: two isolated points share
	a score with something different in between (f = (x-3)^2 at x = 2.5 and 3.5).
	Only the plateau is evidence of quantization, so a contradicted pair is
	dropped rather than reported — the tool must not cry wolf about the very
	thing it exists to make believable."""
	lo, hi = sorted((float(a[n]), float(b[n])))
	for e in evals:
		p = e["params"]
		if not (lo < float(p[n]) < hi):
			continue
		if any(float(p[k]) != float(a[k]) for k in names if k != n):
			continue
		if repr(e["score"]) != repr(score):
			return True
	return False


def quantization_report(history, names, declared):
	"""EFFECTIVE parameter resolution, measured from the run's own evals.

	The failure this makes impossible to miss (rotor F8): an in-loop
	discretiser — a voxel grid, a mesh seed, a rounded slicer input — snaps a
	parameter, so two candidates that differ in a real dimension return a
	BIT-IDENTICAL result. Nelder-Mead reads that as a flat direction and the run
	*looks converged* while the search space is silently the grid pitch, not the
	declared bounds. Nothing in the receipt said so.

	The detector is evaluator-agnostic on purpose — it never mentions voxels.
	Attribution is only made where it is SOUND: a pair of evals with a
	bit-identical score whose parameter vectors differ in EXACTLY ONE parameter
	pins the blame on that parameter, and the largest such |delta| is a lower
	bound on the effective resolution along that axis. Pairs that differ in
	several parameters are counted separately as `ambiguous_dead_pairs` — an
	identical score there can just as well be a genuine level set of the
	objective, and guessing would be the same sin the tool is reporting.

	`declared` (`params.<n>.resolution`) is an optional operator claim; when
	given, a search step below it is flagged too.

	Returns None when nothing was detected, so receipts of clean runs are
	unchanged (SPEC section 3 byte-comparability)."""
	evals = [h for h in history if not h.get("selection_drifted")]
	dead = {n: {"dead_step_max": 0.0, "pairs": 0, "example": None, "min_live_step": None}
	        for n in names}
	ambiguous = 0
	for i in range(len(evals)):
		for jx in range(i + 1, len(evals)):
			a, b = evals[i]["params"], evals[jx]["params"]
			deltas = {n: abs(float(a[n]) - float(b[n])) for n in names}
			moved = [n for n in names if deltas[n] != 0.0]
			if not moved:
				continue  # the same point evaluated twice — not evidence
			same_score = repr(evals[i]["score"]) == repr(evals[jx]["score"])
			if len(moved) > 1:
				ambiguous += same_score
				continue  # not attributable to any SINGLE parameter
			n, rec = moved[0], dead[moved[0]]
			if not same_score:
				# A step along n that DID move the result — the yardstick the
				# dead steps are judged against.
				if rec["min_live_step"] is None or deltas[n] < rec["min_live_step"]:
					rec["min_live_step"] = deltas[n]
				continue
			if _contradicted(evals, names, n, a, b, evals[i]["score"]):
				continue  # a level set of the objective, not a flat plateau
			rec["pairs"] += 1
			if deltas[n] > rec["dead_step_max"]:
				rec["dead_step_max"] = deltas[n]
				rec["example"] = {"a": {k: float(a[k]) for k in names},
				                  "b": {k: float(b[k]) for k in names},
				                  "delta": deltas[n]}
	# A dead step is only EVIDENCE of quantization when it is at least as wide as
	# a step that does register: a smooth evaluator can also return equal scores
	# for two nearby points once the simplex contracts to float noise, and that
	# is convergence, not a discretised search space. No magic constant — the
	# run's own smallest live step is the yardstick. A parameter that NEVER moved
	# the result (no live step at all) is the strongest case of all.
	suspect = {n: r for n, r in dead.items()
	           if r["pairs"] and (r["min_live_step"] is None
	                              or r["dead_step_max"] >= r["min_live_step"])}
	found = bool(suspect)
	below = {}
	for n, r in declared.items():
		steps = [abs(float(h["params"][n]) - float(g["params"][n]))
		         for h, g in zip(history, history[1:])]
		steps = [s for s in steps if s > 0.0]
		if steps and min(steps) < float(r):
			below[n] = {"declared_resolution": float(r), "smallest_search_step": min(steps)}
	if not found and not below:
		return None
	out = {
		"detected": found,
		"method": "two evals with a bit-identical score whose parameters differ in EXACTLY ONE "
		          "coordinate cannot both be informative — the largest such |delta| is a LOWER "
		          "BOUND on the evaluator's effective resolution along that axis. Pairs that "
		          "differ in several coordinates are counted as ambiguous, not attributed.",
		"effective_resolution_lower_bound_per_param": {
			n: r["dead_step_max"] for n, r in suspect.items()},
		"dead_pairs_per_param": {n: r["pairs"] for n, r in suspect.items()},
		"smallest_step_that_did_register": {
			n: r["min_live_step"] for n, r in suspect.items()},
		"ambiguous_dead_pairs": ambiguous,
		"evidence": {n: r["example"] for n, r in suspect.items()},
		"consequence": "the search space is discretised at (at least) these steps, NOT the "
		               "declared bounds; a converged-looking optimum may be a plateau of the "
		               "discretiser. Re-verify the selected point on a finer evaluator.",
	}
	if below:
		out["steps_below_declared_resolution"] = below
	return out


def main(job=None, job_path=None):
	if job is None:
		job_path = sys.argv[1]
		job = json.load(open(job_path))
	if not isinstance(job.get("params"), dict) or not job["params"]:
		raise ValueError("job needs a non-empty `params` object")
	names = list(job["params"])
	lo, hi, x0 = [], [], []
	for name in names:
		spec = job["params"][name]
		if not isinstance(spec, dict) or "min" not in spec or "max" not in spec:
			raise ValueError(f"parameter {name!r} needs finite min/max bounds")
		lower = _finite_number(spec["min"], f"parameter {name} min")
		upper = _finite_number(spec["max"], f"parameter {name} max")
		if lower > upper:
			raise ValueError(f"parameter {name!r}: min {lower} exceeds max {upper}")
		initial = _finite_number(spec.get("init", (lower + upper) / 2), f"parameter {name} init")
		if not lower <= initial <= upper:
			raise ValueError(f"parameter {name!r}: init {initial} lies outside [{lower}, {upper}]")
		lo.append(lower)
		hi.append(upper)
		x0.append(initial)
	sign = -1.0 if job.get("maximize", True) else 1.0
	state = {"evals": 0, "best": None, "history": [], "witness_edges": {}}
	pdir = station_dir(job, job_path)

	def cost(x):
		xv = [min(max(v, l), h) for v, l, h in zip(x, lo, hi)]
		values = dict(zip(names, xv))
		state["evals"] += 1
		try:
			res, measures, err = evaluate(job, values, program_dir=pdir)
		except (ValueError, TypeError, ArithmeticError) as exc:
			print(f"eval {state['evals']}: {values} -> invalid evaluator/expression: {exc}", file=sys.stderr)
			return 1e12
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
		# A run that evaluated nothing is a REFUSAL, not a quiet return: it used
		# to print a bare line, persist no receipt anywhere, and exit 0.
		return {"ok": False, "error_kind": "refusal.no_successful_evaluation",
		        "error": f"no_successful_evaluation: all {state['evals']} evaluation(s) failed "
		                 f"or were rejected — nothing was optimized",
		        "evals": state["evals"], "n_evals": state["evals"],
		        "history_first": state["history"][0] if state["history"] else None,
		        "history_last": state["history"][-1] if state["history"] else None}
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
		# `evals` is a COUNT under a plural name; a roll-up that did
		# len(receipt["evals"]) — the natural reading, and the shape other
		# receipt roll-ups here use — raised TypeError (din_rail F9). `n_evals`
		# is the unambiguous spelling; `evals` is kept for the shipped readers.
		"n_evals": state["evals"],
		"history_first": state["history"][0], "history_last": state["history"][-1],
		"selection_unstable": bool(unstable),
	}
	declared_res = {n: p["resolution"] for n, p in job["params"].items()
	                if isinstance(p, dict) and p.get("resolution") is not None}
	if declared_res:
		out["declared_resolution"] = {n: float(r) for n, r in declared_res.items()}
	quant = quantization_report(state["history"], names, declared_res)
	if quant:
		out["quantization"] = quant
		print("WARNING: the evaluator QUANTIZES the search space — effective resolution "
		      f"{quant.get('effective_resolution_lower_bound_per_param')}; a converged-looking "
		      "optimum may be a plateau of the discretiser (receipt key `quantization`).",
		      file=sys.stderr, flush=True)
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
	# The optimizer algorithm is pinned, but an arbitrary command evaluator is not
	# automatically promoted to Validated. Inherit a nested analysis envelope when
	# present; otherwise publish the result as Demonstrated while separately naming
	# the optimizer's own validated implementation.
	import provenance
	nested = measures.get("analysis_envelope") if isinstance(measures, dict) else None
	nested_status = ((nested or {}).get("provenance") or {}).get("validation_status") \
		if isinstance(nested, dict) else None
	result_status = nested_status if nested_status in provenance.ALLOWED_STATUS else provenance.STATUS_DEMONSTRATED
	job_identity = {k: v for k, v in job.items()
	                if k not in {"receipt", "out_dir", "program_dir", "wall_budget_s"}}
	job_hash = provenance.geometry_hash(program=job_identity)
	convergence = {
		"method": "Nelder-Mead with bound clipping" if start_bests else "deterministic coordinate sweep",
		"n_evals": state["evals"], "evaluation_budget": budget,
		"successful_candidate": True, "constraint_ok": cons_ok,
		"selection_unstable": bool(unstable),
		"quantization_detected": bool(quant and quant.get("detected")),
	}
	out["optimizer_validation_status"] = provenance.STATUS_VALIDATED
	out["result_validation_status"] = result_status
	out["geometry_hash"] = job_hash
	out["residual_or_convergence"] = convergence
	out["analysis_envelope"] = provenance.stamp(
		values={"best_objective": out["best_objective"], "best_score": out["best_score"],
		        "constraint_ok": cons_ok},
		geometry_hash=job_hash,
		material_version="optimizer-inputs:sha256:" + __import__("hashlib").sha256(
			json.dumps(job.get("params", {}), sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()[:16],
		analyzer_name="param_optimize", analyzer_version=ANALYZER_VERSION,
		validation_status=result_status, residual_or_convergence=convergence,
		manifest_ref="tools/manifests/param_optimize.manifest.json")
	return out


def cli():
	sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
	import _receipt

	job_path, _ = _receipt.parse_argv()
	job, out = _receipt.load_job()
	payload = main(job, job_path=job_path)
	_receipt.finish(payload, job=job, tool="param_optimize", out=out,
	                kind=payload.get("error_kind"), use_out_dir_default=True)


if __name__ == "__main__":
	sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
	import _receipt

	_receipt.run_cli("param_optimize", cli)
