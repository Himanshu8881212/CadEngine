"""_receipt.py — shared receipt PERSISTENCE for the checker tools.

The audit finding this closes (2026-07-16): tolerance_stack / balance_check /
joint_check / production_check / sweep_check printed their receipt JSON to
stdout ONLY — unless a human piped it, the verification evidence existed
nowhere on disk and re-verification meant re-running. The stdout line stays
the wire contract (callers parse the LAST stdout line); this module adds the
on-disk copy:

- job key `"receipt": "<path>"`  → the receipt is ALSO written there
  (relative paths join the job's `out_dir` when it has one, else the CWD);
- else, job key `"out_dir"`      → `<out_dir>/<tool>_receipt.json`;
- else                           → stdout only (unchanged legacy behavior).

`emit(receipt, job, tool)` prints AND persists, then returns the path written
(or None). Failure receipts persist too — a refusal is evidence, not noise.
The written form is indented (human-diffable); the stdout form stays one line
(machine-parsable). Best-effort: an unwritable receipt path is reported on
stderr and never masks the receipt itself.
"""

import json
import os
import sys


def receipt_path(job: dict, tool: str) -> str | None:
	"""Where this job's receipt should persist (see module docs), or None."""
	if not isinstance(job, dict):
		return None
	explicit = job.get("receipt")
	out_dir = job.get("out_dir")
	if isinstance(explicit, str) and explicit:
		if not os.path.isabs(explicit) and isinstance(out_dir, str) and out_dir:
			return os.path.join(out_dir, explicit)
		return explicit
	if isinstance(out_dir, str) and out_dir:
		return os.path.join(out_dir, f"{tool}_receipt.json")
	return None


def emit(receipt: dict, job: dict, tool: str) -> str | None:
	"""Print the one-line stdout receipt AND persist it per `receipt_path`."""
	print(json.dumps(receipt), flush=True)
	path = receipt_path(job, tool)
	if not path:
		return None
	try:
		parent = os.path.dirname(path)
		if parent:
			os.makedirs(parent, exist_ok=True)
		with open(path, "w", encoding="utf-8") as f:
			json.dump(receipt, f, indent=1)
			f.write("\n")
		return path
	except OSError as e:  # never mask the receipt itself
		print(f"receipt not persisted to {path}: {e}", file=sys.stderr, flush=True)
		return None
