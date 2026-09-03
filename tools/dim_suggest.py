#!/usr/bin/env python3
"""dim_suggest.py — draft engineering dimension callouts from ANALYTIC faces.

The auto layer of the FRICTION #21 drawing stack (audit 2026-07-16): runs a
part's program through the engine, reads `list_faces` (analytic surface tags +
witness anchors), and emits a `dimensions: [...]` list ready for
render_sheet.py — so a sheet grows real callouts (bore Øs, step heights)
without a human opening CAD. The AI's job shrinks to PRUNING a draft instead
of authoring from scratch.

What it suggests (and what it refuses to guess):
- **Ø callouts**: one per distinct cylindrical bore/boss (grouped by radius +
  axis direction + axis line, so a full-wrap cylinder split into faces stays
  ONE callout) and per sphere. Values are the exact analytic `2·radius`.
- **Step ladders**: for each principal axis, the distinct axis-aligned plane
  offsets, as consecutive step dimensions (anchored at the larger plane's
  witness) — only when the ladder has 2..6 rungs; denser ladders are dropped
  as unreadable, and SAID so in the receipt. The full-axis step duplicating
  the sheet's overall bbox dimension is skipped.
- Nothing else: cones, tori and freeform faces get no invented numbers.

Usage:  python3 dim_suggest.py job.json [--out PATH]
Job: {"program": {...} | "program_file": path,   the ops that BUILD the part
      "solid": "op id of the solid to measure",
      "out": "dims.json",                         the render_sheet fragment
      "max": 10,                                  callout budget (priority:
                                                  diameters, then z/x/y steps)
      "program_dir": path?,                       where the measurement program is
                                                  materialised = the root its
                                                  relative import_step/load_part
                                                  paths resolve against (default:
                                                  out_dir, else the job file's own
                                                  directory; never a system temp
                                                  dir — turgo F7)
      "receipt": path?}                           see tools/_receipt.py
Stdout: one-line JSON receipt {ok, suggested, dropped, out}; the `out` file
holds {"dimensions": [...]} for direct merge into a render_sheet job.
Persistence + exit codes: the shared contract in tools/_receipt.py.
"""

import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _layout  # noqa: E402
_layout.add_import_paths()  # analyzers/ + publish/ are importable siblings after the 2026-09-02 move
import _receipt  # noqa: E402
import param_optimize  # noqa: E402 — call_engine: the one-shot engine pattern
from _receipt import Refusal  # noqa: E402


def log(msg):
	print(msg, file=sys.stderr, flush=True)


def _norm_axis(a):
	"""Canonical unit axis with a deterministic sign (first nonzero positive)."""
	import math

	n = math.sqrt(sum(x * x for x in a)) or 1.0
	u = [x / n for x in a]
	for x in u:
		if abs(x) > 1e-9:
			return u if x > 0 else [-y for y in u]
	return u


def _dot(a, b):
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def suggest(job, job_path=None):
	if "program" in job:
		program = job["program"]
	elif "program_file" in job:
		program = json.load(open(job["program_file"]))
	else:
		raise Refusal("missing_program", "job needs 'program' (ops) or 'program_file'")
	if "solid" not in job:
		raise Refusal("missing_solid", "job needs 'solid' (the op id of the solid to measure)")
	solid = job["solid"]
	budget = int(job.get("max", 10))

	ops = [op for op in program["ops"] if op.get("op") not in ("export_stl", "export_step", "export_3mf")]
	ops.append({"id": "__dim_faces", "op": "list_faces", "in": solid})
	ops.append({"id": "__dim_bb", "op": "bounding_box", "in": solid})
	rep = param_optimize.call_engine(
		{"ops": ops}, program_dir=param_optimize.station_dir(job, job_path))
	if not rep.get("ok"):
		bad = next((o for o in rep["ops"] if not o.get("ok")), {})
		raise Refusal("program_failed",
		              f"program failed at op '{bad.get('id')}': {bad.get('error')}")
	by_id = {o["id"]: o for o in rep["ops"]}
	faces = by_id["__dim_faces"]["measures"]["faces"]
	bb = by_id["__dim_bb"]["measures"]
	bbox_size = bb["size"]

	diameters, steps = [], {0: {}, 1: {}, 2: {}}
	axes = ([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0])
	seen_cyl = {}
	for f in faces:
		d = f["descriptor"]
		if f["type"] == "cylinder":
			axis = _norm_axis(d["axis"])
			p, r, w = d["point"], float(d["radius"]), f["witness"]
			# Axis line signature: the axis point's component perpendicular to axis.
			t = _dot(p, axis)
			perp = tuple(round(p[i] - t * axis[i], 3) for i in range(3))
			key = (round(r, 4), tuple(round(x, 4) for x in axis), perp)
			if key in seen_cyl:
				continue
			seen_cyl[key] = True
			# Circle center at the witness's axial station — the callout sits on
			# real geometry, not at an arbitrary height.
			tw = _dot(w, axis) - _dot(p, axis)
			center = [p[i] + tw * axis[i] for i in range(3)]
			diameters.append({
				"kind": "diameter", "center": center, "axis": list(axis), "radius": r,
				"label": f"Ø{2.0 * r:.2f}",
			})
		elif f["type"] == "sphere":
			d0 = d
			diameters.append({
				"kind": "diameter", "center": d0["center"], "axis": [0.0, 0.0, 1.0], "radius": float(d0["radius"]),
				"label": f"SØ{2.0 * float(d0['radius']):.2f}",
			})
		elif f["type"] == "plane":
			n = _norm_axis(d["normal"])
			for k in range(3):
				if abs(_dot(n, axes[k])) > 0.999:
					off = round(_dot(d["point"], axes[k]), 5)
					area = f.get("area") or 0.0
					prev = steps[k].get(off)
					if prev is None or area > prev[1]:
						steps[k][off] = (f["witness"], area)

	diameters.sort(key=lambda c: -c["radius"])
	linear = []
	for k in (2, 0, 1):  # z ladders first — heights are what makers ask for
		offs = sorted(steps[k])
		if not (2 <= len(offs) <= 6):
			if len(offs) > 6:
				log(f"axis {'xyz'[k]}: {len(offs)} plane offsets — ladder too dense to read, dropped")
			continue
		for o1, o2 in zip(offs, offs[1:]):
			# Skip the step that just repeats the overall bbox extent.
			if len(offs) == 2 and abs((o2 - o1) - bbox_size[k]) < 1e-6:
				continue
			w2 = steps[k][o2][0]
			a = [w2[0], w2[1], w2[2]]
			b = [w2[0], w2[1], w2[2]]
			a[k], b[k] = o1, o2
			linear.append({"kind": "linear", "a": a, "b": b, "label": f"{abs(o2 - o1):.2f} mm"})

	ordered = diameters + linear
	dims = ordered[:budget]
	dropped = len(ordered) - len(dims)
	out_path = job.get("out")
	if out_path:
		os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
		with open(out_path, "w") as f:
			json.dump({"dimensions": dims}, f, indent=1)
			f.write("\n")
	return {
		"ok": True,
		"suggested": len(dims),
		"diameters": len(diameters),
		"linear_steps": len(linear),
		"dropped_over_budget": dropped,
		"out": os.path.abspath(out_path) if out_path else None,
		"dimensions": dims,
	}


def main():
	job_path, _ = _receipt.parse_argv()
	job, out = _receipt.load_job()
	_receipt.finish(suggest(job, job_path), job=job, tool="dim_suggest", out=out,
	                use_out_dir_default=True)


if __name__ == "__main__":
	_receipt.run_cli("dim_suggest", main)
