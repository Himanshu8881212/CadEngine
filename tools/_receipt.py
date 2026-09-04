"""_receipt.py — the shared RECEIPT + EXIT-CODE CONTRACT for the tools/ runners.

Two audits are closed here.

(1) 2026-07-16 — persistence. tolerance_stack / balance_check / joint_check /
production_check / sweep_check printed their receipt JSON to stdout ONLY —
unless a human piped it, the verification evidence existed nowhere on disk and
re-verification meant re-running. The stdout line stays the wire contract
(callers parse the LAST stdout line); this module adds the on-disk copy:

- caller's explicit destination (``out=``)  → the receipt is written THERE;
- job key `"receipt": "<path>"`             → the receipt is ALSO written there
  (relative paths join the job's `out_dir` when it has one, else the CWD);
- else, job key `"out_dir"`                 → `<out_dir>/<tool>_receipt.json`
  (only when `use_out_dir_default`, which the checker tools keep on and the
  physics runners keep off — they never invented a receipt file before);
- else                                      → stdout only (unchanged legacy).

(2) 2026-08-08 — THE SILENCE AUDIT (portfolio themes T3/T7). Ten campaigns,
eight of them independently, were burned by four shapes of silence:

  * a tool exited 0 on `ok:false`, so a `$?` shell gate accepted a FAIL, while
    an internal `KeyError` exited 1 — the two signals pointed the wrong way
    round (`ball F5`, `gripper F9`, `cubesat F11`, `din_rail F3`);
  * a timeout or an OOM kill left NO receipt at all — the run VANISHED rather
    than failing (`ball F11`, `wrist F8`);
  * a job-level `receipt` key silently CLOBBERED a shipped receipt when the
    caller had asked for a different destination (`singulator F14`, `cleat F7`);
  * the documented `runner.py job | tail -1 > receipt.json` idiom truncates the
    target at LAUNCH, so an interrupted solve leaves a ZERO-BYTE file where a
    good receipt used to be (`turgo F8`).

THE CONTRACT (one, for every runner in this directory):

  exit 0  `ok: true`  — the analysis ran and the receipt is usable
  exit 1  `ok: false` — the tool could not run the request (usage, unreadable
                        job, internal error). No analysis was performed.
  exit 2  `ok: false` — the tool RAN and REFUSED, or the analysis failed. The
                        receipt says why in `error` + `error_kind`.

  Any nonzero exit means "do not quote this receipt". Both signals — the exit
  code and `ok` — always agree; that is the whole point. `error_kind` is a
  machine-matchable slug (`refusal.*`, `timeout`, `killed.*`, `internal`,
  `usage`, `receipt_path_conflict`) so a gate can branch without regexing prose.

  LOUD BACKWARD-COMPAT NOTE: the ACE runners previously exited 0 on `ok:false`
  BY DESIGN, and OPERATOR_BRIEF told campaigns "parse `ok`, never `$?`". That
  advice still works — `ok` is unchanged. What changes is that `$?` now works
  too. A caller that genuinely depends on exit-0-on-failure sets
  `LMCAD_RUNNER_EXIT=legacy` (env) or `"legacy_exit_zero": true` (job key); the
  receipt then carries `exit_contract.mode = "legacy"` so the opt-out is itself
  on the record. Strict is the default, everywhere, deliberately.

  `--out PATH` (all runners) writes the receipt ATOMICALLY (temp + rename) at
  the END of the run, so it can never truncate a good receipt the way the
  `| tail -1 >` redirect does. A job-level `receipt` key that disagrees with
  `--out` is REFUSED (exit 1, `error_kind: receipt_path_conflict`) instead of
  quietly winning. `LMCAD_RECEIPT_DRY_RUN=1` suppresses every on-disk write —
  what-if runs against a shipped campaign can no longer mutate its evidence.

  A wall budget (`"wall_budget_s"` in the job, or `LMCAD_WALL_BUDGET_S`) and
  SIGTERM/SIGINT both synthesize an honest `ok:false` receipt naming the limit
  instead of dying with a bare traceback. SIGKILL (what `subprocess.run(
  timeout=)` sends) cannot be caught by anyone — which is exactly why the
  budget belongs INSIDE the runner.

The written form is indented (human-diffable); the stdout form stays one line
(machine-parsable). Best-effort on the write: an unwritable receipt path is
reported on stderr and never masks the receipt itself.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import signal
import sys

# --- the exit contract ------------------------------------------------------
EXIT_OK = 0
EXIT_ERROR = 1      # the tool could not run the request at all
EXIT_REFUSED = 2    # the tool ran and refused / the analysis failed

EXIT_MEANING = {
	EXIT_OK: "ok:true — analysis ran, receipt usable",
	EXIT_ERROR: "ok:false — tool could not run the request (usage/internal); no analysis performed",
	EXIT_REFUSED: "ok:false — tool ran and REFUSED, or the analysis failed; see error_kind",
}

STRICT_ENV = "LMCAD_RUNNER_EXIT"        # "strict" (default) | "legacy"
DRY_RUN_ENV = "LMCAD_RECEIPT_DRY_RUN"   # "1" -> never write a receipt file
BUDGET_ENV = "LMCAD_WALL_BUDGET_S"      # float seconds; job key wins


class Refusal(Exception):
	"""A refusal the tool is CERTAIN about: the request is not answerable as
	posed. Carries a machine-matchable `kind` (prefixed `refusal.`) and
	optional structured `details` that land in the receipt verbatim."""

	def __init__(self, kind: str, message: str, **details):
		super().__init__(message)
		self.kind = kind if kind.startswith(("refusal.", "timeout", "killed.")) else f"refusal.{kind}"
		self.details = details


class ReceiptPathConflict(Exception):
	"""The job asked for one receipt destination and the caller for another.
	Refused, never silently resolved — that ambiguity clobbered shipped
	evidence in two campaigns."""


# --- process context (so a signal handler can still produce a receipt) -------
_CTX: dict = {"tool": None, "job": None, "out": None, "emitted": False,
              "budget_s": None}


def dry_run() -> bool:
	return os.environ.get(DRY_RUN_ENV, "") not in ("", "0", "false", "no")


def strict_exit(job: dict | None = None) -> bool:
	"""True (the default) when a failed receipt must also exit nonzero."""
	if isinstance(job, dict) and job.get("legacy_exit_zero"):
		return False
	return os.environ.get(STRICT_ENV, "strict").strip().lower() != "legacy"


# --- where a receipt persists -----------------------------------------------
def _job_receipt_path(job: dict) -> str | None:
	explicit = job.get("receipt") if isinstance(job, dict) else None
	out_dir = job.get("out_dir") if isinstance(job, dict) else None
	if isinstance(explicit, str) and explicit:
		if not os.path.isabs(explicit) and isinstance(out_dir, str) and out_dir:
			return os.path.join(out_dir, explicit)
		return explicit
	return None


def receipt_path(job: dict, tool: str, out: str | None = None,
                 use_out_dir_default: bool = True) -> str | None:
	"""Where this job's receipt persists (see module docs), or None.

	`out` is the CALLER's explicit destination and always wins. A job-level
	`receipt` key that resolves somewhere ELSE is a CONFLICT and raises —
	silently honouring the job key overwrote 12 shipped receipts during a
	read-only verification run (singulator F14)."""
	if not isinstance(job, dict):
		return os.path.abspath(out) if out else None
	from_job = _job_receipt_path(job)
	if out:
		out_abs = os.path.abspath(out)
		if from_job and os.path.abspath(from_job) != out_abs:
			raise ReceiptPathConflict(
				f"job key 'receipt' = {from_job!r} disagrees with the caller's "
				f"destination {out!r}. Refusing rather than picking one: honouring "
				f"the job key silently overwrites the shipped receipt. Remove the "
				f"job's 'receipt' key, or pass --out with the same path.")
		return out_abs
	if from_job:
		return from_job
	out_dir = job.get("out_dir")
	if use_out_dir_default and isinstance(out_dir, str) and out_dir:
		return os.path.join(out_dir, f"{tool}_receipt.json")
	return None


def _write_atomic(path: str, receipt: dict) -> str:
	"""Write via temp+rename so an interrupted run can never leave a truncated
	or zero-byte receipt where a good one used to be (turgo F8)."""
	parent = os.path.dirname(path)
	if parent:
		os.makedirs(parent, exist_ok=True)
	tmp = f"{path}.tmp-{os.getpid()}"
	with open(tmp, "w", encoding="utf-8") as f:
		json.dump(receipt, f, indent=1)
		f.write("\n")
		f.flush()
		os.fsync(f.fileno())
	os.replace(tmp, path)
	return path


def emit(receipt: dict, job: dict | None = None, tool: str | None = None, *,
         out: str | None = None, use_out_dir_default: bool = True) -> str | None:
	"""Print the one-line stdout receipt AND persist it per :func:`receipt_path`.

	Returns the path written (or None). The write is atomic, is announced on
	stderr (a silent write is how a what-if run destroyed shipped evidence),
	and is skipped entirely under `LMCAD_RECEIPT_DRY_RUN=1`."""
	print(json.dumps(receipt), flush=True)
	_CTX["emitted"] = True
	if job is None:
		job = _CTX.get("job")
	if tool is None:
		tool = _CTX.get("tool") or "tool"
	try:
		path = receipt_path(job if isinstance(job, dict) else {}, tool, out=out,
		                    use_out_dir_default=use_out_dir_default)
	except ReceiptPathConflict as exc:
		print(f"receipt NOT persisted: {exc}", file=sys.stderr, flush=True)
		return None
	if not path:
		return None
	if dry_run():
		print(f"receipt NOT persisted to {path}: {DRY_RUN_ENV} is set (dry run)",
		      file=sys.stderr, flush=True)
		return None
	try:
		_write_atomic(path, receipt)
	except OSError as e:  # never mask the receipt itself
		print(f"receipt not persisted to {path}: {e}", file=sys.stderr, flush=True)
		return None
	print(f"receipt written: {path}", file=sys.stderr, flush=True)
	return path


# --- exit-code stamping -----------------------------------------------------
def exit_code_for(payload: dict, *, internal: bool = False) -> int:
	if payload.get("ok"):
		return EXIT_OK
	return EXIT_ERROR if internal else EXIT_REFUSED


def stamp_exit(payload: dict, code: int, *, kind: str | None = None,
               job: dict | None = None) -> dict:
	"""Put the exit contract INSIDE the receipt, so a failed analysis is
	detectable from the receipt alone as well as from `$?`."""
	strict = strict_exit(job if job is not None else _CTX.get("job"))
	effective = code if strict else EXIT_OK
	payload["exit_code"] = effective
	if not payload.get("ok"):
		payload.setdefault("error_kind", kind or "internal")
		payload["exit_contract"] = {
			"mode": "strict" if strict else "legacy",
			"code": effective,
			"meaning": EXIT_MEANING[code],
			"contract": "0 = ok:true; 1 = tool could not run the request; "
			            "2 = ran and REFUSED / analysis failed. Any nonzero: "
			            "do not quote this receipt.",
			"opt_out": f"{STRICT_ENV}=legacy (env) or \"legacy_exit_zero\": true "
			           f"(job key) restores the historical always-exit-0 behaviour "
			           f"and records mode=legacy here.",
		}
		if not strict:
			payload["exit_contract"]["suppressed_code"] = code
	return payload


def finish(payload: dict, *, job: dict | None = None, tool: str | None = None,
           out: str | None = None, kind: str | None = None,
           internal: bool = False, use_out_dir_default: bool = False):
	"""Emit the receipt, persist it, and exit with the contracted code.

	`use_out_dir_default` defaults to False here: the physics runners never
	invented `<out_dir>/<tool>_receipt.json` files and must not start."""
	if job is None:
		job = _CTX.get("job")
	if out is None:
		out = _CTX.get("out")
	code = exit_code_for(payload, internal=internal)
	stamp_exit(payload, code, kind=kind, job=job)
	emit(payload, job, tool, out=out, use_out_dir_default=use_out_dir_default)
	sys.exit(payload["exit_code"])


# --- failure receipts a signal handler can still produce --------------------
def _failure_payload(error: str, kind: str, **extra) -> dict:
	payload = {"ok": False, "error": error, "error_kind": kind}
	payload.update(extra)
	return payload


def _emergency(error: str, kind: str, code: int, **extra):
	"""Last-resort receipt from a signal handler / watchdog. Never raises."""
	try:
		payload = _failure_payload(error, kind, **extra)
		job = _CTX.get("job")
		stamp_exit(payload, code, kind=kind, job=job)
		emit(payload, job, _CTX.get("tool"), out=_CTX.get("out"),
		     use_out_dir_default=False)
		os._exit(payload["exit_code"])
	except BaseException:  # noqa: BLE001 — a receipt attempt must never hang the exit
		try:
			print(json.dumps({"ok": False, "error": error, "error_kind": kind,
			                  "exit_code": code}), flush=True)
		except BaseException:
			pass
		os._exit(code)


def _on_signal(signum, _frame):
	name = signal.Signals(signum).name
	_emergency(
		f"{name}: the runner was signalled and stopped before producing a result. "
		f"This receipt is synthesized by the runner itself so the run FAILS "
		f"visibly instead of vanishing.",
		f"killed.{name}", EXIT_REFUSED, signal=name, signal_number=int(signum))


def _on_budget(_signum, _frame):
	b = _CTX.get("budget_s")
	_emergency(
		f"wall budget exceeded: the runner self-terminated after {b} s "
		f"(job 'wall_budget_s' / {BUDGET_ENV}). A resource kill is a RECORDED "
		f"negative result, not a missing one.",
		"timeout", EXIT_REFUSED, wall_budget_s=b, killed_at_wall_budget=True)


def install_signal_receipts() -> None:
	for sig in ("SIGTERM", "SIGINT", "SIGHUP"):
		s = getattr(signal, sig, None)
		if s is None:
			continue
		try:
			signal.signal(s, _on_signal)
		except (ValueError, OSError):  # not the main thread / not supported
			pass


def arm_wall_budget(job: dict | None) -> float | None:
	"""Arm a self-imposed wall budget so a starved/OOM-bound run produces an
	honest `ok:false` receipt instead of nothing. Returns the budget or None.

	The alarm fires between bytecodes, so a single very long C call (a large
	eigensolve) can overrun the budget before the handler runs — stated, not
	hidden. It is still strictly better than the caller's SIGKILL, which no
	process can turn into a receipt."""
	raw = None
	if isinstance(job, dict):
		raw = job.get("wall_budget_s")
	if raw is None:
		raw = os.environ.get(BUDGET_ENV) or None
	if raw is None:
		return None
	try:
		budget = float(raw)
	except (TypeError, ValueError):
		raise Refusal("bad_wall_budget", f"wall_budget_s must be a number, got {raw!r}")
	if budget <= 0:
		raise Refusal("bad_wall_budget", f"wall_budget_s must be > 0, got {budget}")
	if not hasattr(signal, "SIGALRM"):
		print("wall budget requested but SIGALRM is unavailable on this platform; "
		      "the budget is NOT armed", file=sys.stderr, flush=True)
		return None
	_CTX["budget_s"] = budget
	signal.signal(signal.SIGALRM, _on_budget)
	signal.setitimer(signal.ITIMER_REAL, budget)
	return budget


def disarm_wall_budget() -> None:
	if hasattr(signal, "setitimer") and _CTX.get("budget_s"):
		try:
			signal.setitimer(signal.ITIMER_REAL, 0.0)
		except (ValueError, OSError):
			pass


# --- CLI front door ---------------------------------------------------------
def parse_argv(argv: list[str] | None = None) -> tuple[str, str | None]:
	"""`<job.json> [--out PATH]` — the one CLI shape every runner accepts.
	Positional job path stays first so every existing invocation is unchanged."""
	argv = list(sys.argv[1:] if argv is None else argv)
	out = None
	rest = []
	i = 0
	while i < len(argv):
		a = argv[i]
		if a in ("--out", "--receipt-out"):
			if i + 1 >= len(argv):
				raise Refusal("usage", f"{a} needs a path")
			out = argv[i + 1]
			i += 2
			continue
		if a.startswith("--out="):
			out = a.split("=", 1)[1]
			i += 1
			continue
		rest.append(a)
		i += 1
	if len(rest) != 1:
		raise Refusal("usage", f"usage: {_CTX.get('tool') or 'runner'}.py <job.json> [--out PATH]")
	return rest[0], out


def load_job(argv: list[str] | None = None) -> tuple[dict, str | None]:
	"""Parse argv, read the job, arm the wall budget, and REFUSE a receipt-path
	conflict up front (before the solve, so a what-if run cannot get halfway and
	then clobber). Returns (job, out_path)."""
	job_path, out = parse_argv(argv)
	try:
		with open(job_path, "r", encoding="utf-8") as f:
			job = json.load(f)
	except OSError as exc:
		raise Refusal("job_unreadable", f"cannot read job {job_path!r}: {exc}")
	except json.JSONDecodeError as exc:
		raise Refusal("job_malformed", f"job {job_path!r} is not valid JSON: {exc}")
	if not isinstance(job, dict):
		raise Refusal("job_malformed", f"job {job_path!r} must be a JSON object")
	_CTX["job"] = job
	_CTX["out"] = out
	receipt_path(job, _CTX.get("tool") or "tool", out=out, use_out_dir_default=False)
	arm_wall_budget(job)
	return job, out


def run_cli(tool: str, main, *, install_hint: str | None = None,
            refusal_types: tuple = ()) -> None:
	"""The top-level wrapper every runner's `__main__` uses.

	Guarantees a receipt on EVERY path — success, refusal, internal error,
	signal, wall-budget kill — and an exit code that agrees with it.

	`refusal_types` names the runner's OWN domain-refusal exceptions (its
	`JobError` / `DataRefusal` / `ConvergenceError`). Those are exit 2 (the
	tool ran and refused); anything else is exit 1 (the tool broke). Both are
	nonzero: the historical inversion — internal KeyError exits 1 while a real
	FAIL exits 0 — is what this contract exists to make impossible."""
	_CTX["tool"] = tool
	install_signal_receipts()
	try:
		main()
	except SystemExit:
		raise
	except Refusal as exc:
		disarm_wall_budget()
		payload = _failure_payload(str(exc), exc.kind, **exc.details)
		internal = exc.kind in ("refusal.usage", "refusal.job_unreadable",
		                        "refusal.job_malformed")
		finish(payload, tool=tool, kind=exc.kind, internal=internal)
	except ReceiptPathConflict as exc:
		disarm_wall_budget()
		finish(_failure_payload(str(exc), "receipt_path_conflict"), tool=tool,
		       kind="receipt_path_conflict", internal=True)
	except MemoryError as exc:
		disarm_wall_budget()
		finish(_failure_payload(f"MemoryError: {exc}", "resource.memory"), tool=tool,
		       kind="resource.memory")
	except refusal_types as exc:  # the runner's declared domain refusals
		disarm_wall_budget()
		kind = f"refusal.{type(exc).__name__}"
		finish(_failure_payload(f"{type(exc).__name__}: {exc}", kind), tool=tool,
		       kind=kind)
	except BaseException as exc:  # noqa: BLE001 — the JSON line IS the contract
		disarm_wall_budget()
		error = f"{type(exc).__name__}: {exc}"
		if install_hint and isinstance(exc, (ImportError, ModuleNotFoundError)) \
				and "physics" in str(exc):
			error += f" | hint: {install_hint}"
		finish(_failure_payload(error, "internal"), tool=tool, kind="internal",
		       internal=True)


# --- determinism (T7): a byte-comparable core inside a receipt --------------
DETERMINISM_SCHEMA = "lmcad.determinism.v1"


def _quantize(obj, sig: int):
	if isinstance(obj, bool):
		return obj
	if isinstance(obj, float):
		if obj == 0.0 or not math.isfinite(obj):
			return obj
		return float(f"%.{max(sig - 1, 0)}e" % obj)
	if isinstance(obj, dict):
		return {k: _quantize(v, sig) for k, v in obj.items()}
	if isinstance(obj, (list, tuple)):
		return [_quantize(v, sig) for v in obj]
	return obj


def _strip(obj, paths: set[str], prefix: str = ""):
	if isinstance(obj, dict):
		out = {}
		for k, v in obj.items():
			p = f"{prefix}/{k}" if prefix else k
			if p in paths or k in paths:
				continue
			out[k] = _strip(v, paths, p)
		return out
	if isinstance(obj, list):
		return [_strip(v, paths, prefix) for v in obj]
	return obj


def determinism_block(payload: dict, *, nondeterministic_paths, solver_note: str,
                      sig_figs: int = 12) -> dict:
	"""The T7 fix: name the non-deterministic parts of a receipt and hand back a
	digest over what IS reproducible.

	Every ACE receipt embeds wall-clock `timings_s`, so `cmp` on a whole receipt
	is guaranteed to fail; and the eigen/CG solvers reproduce to ~1e-12..1e-13
	relative, not to the bit (ball F10, cubesat F9, horn F14, turgo F10, cleat
	F8, singulator F15). Both facts are now IN the receipt: `core_digest` is a
	sha256 over the payload with the named wall-clock/environment paths removed
	and every float quantized to `core_sig_figs` significant figures. Two runs
	of the same job on the same geometry produce the SAME `core_digest` — that
	is the byte-comparison campaigns were told to make and could not.

	This states the determinism contract honestly: byte-identical to
	`core_sig_figs` significant figures, NOT to the last bit."""
	paths = set(nondeterministic_paths) | {"determinism", "exit_code", "exit_contract"}
	core = _quantize(_strip(payload, paths), sig_figs)
	digest = hashlib.sha256(
		json.dumps(core, sort_keys=True, separators=(",", ":"),
		           default=str).encode("utf-8")).hexdigest()
	return {
		"schema": DETERMINISM_SCHEMA,
		"nondeterministic_paths": sorted(set(nondeterministic_paths)),
		"core_sig_figs": sig_figs,
		"core_digest": "sha256:" + digest,
		"solver_reproducibility": solver_note,
		"how_to_compare": (
			"compare `determinism.core_digest` between runs, NOT the receipt "
			"bytes: the receipt embeds wall-clock timings and the solver "
			"reproduces to core_sig_figs significant figures, not to the bit."),
	}
