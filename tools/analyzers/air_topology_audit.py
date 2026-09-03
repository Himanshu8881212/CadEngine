#!/usr/bin/env python3
"""Internal-air topology audit — the gate the TL-91 defect taught us (2026-07-09).

Voxelizes a watertight STL by per-slice parity fill and flood-labels the INTERNAL AIR.
For any design whose function lives in internal channels (speaker lines, manifolds,
ducts): the functional air path must be ONE connected component between the named seed
points, and the through-wall opening census must match the design intent. Watertight +
geometric_ok CANNOT see this — they gate the material, not the void.

Usage: air_topology_audit.py job.json   (also: --help)

job: {"stl": path, "voxel_mm": 1.0, "wall_margin_mm": 7, "open_faces": ["y+"]  (faces
      sealed later by another part — the domain extends to them minus 1 voxel),
      "seeds": {"name": [x,y,z], ...}, "require_connected": [["chamber","port"], ...],
      "front_openings_face": "y-"|null,
      "receipt": path?  (the receipt is ALSO written there — tools/_receipt.py rules)}

Last stdout line:
  {"ok", "components", "sizes_cm3", "seed_labels", "connected": {...},
   "openings_mm2", "seed_sizes_cm3", "component_sizes_cm3", "grid"}
`ok:false` if any required pair is disconnected — and the process EXITS 1, like every
other shell-gate checker in tools/ (cubesat F11: it used to exit 0 AND ignore the job's
`receipt` key, so a campaign that relied on the key got an ok-looking run with no
evidence on disk).

Reading the sizes — the two adjacent keys that used to look joinable and were not
(horn F13). `sizes_cm3` is the DESCENDING-SORTED, TOP-8 census; `seed_labels` are raw
label ids from the labelling pass. They cannot be indexed into each other. Two extra
keys make the join explicit and complete:
  * `component_sizes_cm3` — {label(str): cm3} for EVERY component, not just 8;
  * `seed_sizes_cm3`      — {seed name: cm3 of the component that seed sits in}.

A seed that lands in MATERIAL (label 0) is a REFUSAL, not a `false` connectivity
verdict: the answer "these are disconnected" would be indistinguishable from "you
pointed at a wall", and only one of those is about the part.

Parity-fill degeneracy (horn F8) is fixed at the source: the fill lives in
`voxelize_stl.parity_fill`, which classifies crossings with a half-open sign rule so a
vertex ring lying exactly on a slice centre no longer deletes the contour. Before that
fix this tool reported a wide-open Ø25.4 bore as SEVERED at voxel_mm 1.0.
"""
import json
import os
import sys

import numpy as np
from scipy import ndimage

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # tools/: the shared contracts + the layout map
import _layout  # noqa: E402
_layout.add_import_paths()  # tools/, tools/analyzers, tools/publish — sibling-style imports keep working after the 2026-09-02 move
from _stl import load_stl  # the shared binary-STL loader
import _receipt
from voxelize_stl import parity_fill  # ONE parity fill, shared with the .npy bridge


def voxelize(tris, h):
	"""Occupancy grid over the mesh's own bbox at pitch h -> (mat, origin)."""
	lo = tris.min((0, 1))
	hi = tris.max((0, 1))
	shape = [int(np.ceil((hi[i] - lo[i]) / h)) for i in range(3)]
	return parity_fill(tris, lo, h, shape), lo


def audit(job):
	h = float(job.get("voxel_mm", 1.0))
	tris = load_stl(job["stl"])
	mat, lo = voxelize(tris, h)
	nx, ny, nz = mat.shape
	m = int(round(float(job.get("wall_margin_mm", 7)) / h))
	slc = [slice(m, nx - m), slice(m, ny - m), slice(m, nz - m)]
	for f in job.get("open_faces", []):
		ax = "xyz".index(f[0])
		hi_face = f[1] == "+"
		slc[ax] = slice(slc[ax].start, mat.shape[ax] - 1) if hi_face else slice(1, slc[ax].stop)
	dom = np.zeros_like(mat)
	dom[tuple(slc)] = True
	air = dom & ~mat
	lab, ncomp = ndimage.label(air)
	sizes = np.bincount(lab.ravel(), minlength=ncomp + 1)
	sizes[0] = 0
	cell_cm3 = h ** 3 / 1000.0

	seeds, misplaced = {}, []
	for k, v in (job.get("seeds") or {}).items():
		idx = tuple(int(np.floor((float(v[i]) - lo[i]) / h)) for i in range(3))
		if any(idx[i] < 0 or idx[i] >= mat.shape[i] for i in range(3)):
			misplaced.append(f"seed {k!r} at {list(v)} is OUTSIDE the mesh bounding box")
			continue
		seeds[k] = int(lab[idx])
		if seeds[k] == 0:
			where = "MATERIAL" if mat[idx] else "outside the audited domain (wall_margin/open_faces)"
			misplaced.append(f"seed {k!r} at {list(v)} lands in {where}, not in audited air")
	if misplaced:
		raise ValueError("seed placement refused: " + "; ".join(misplaced))

	connected = {f"{a}<->{b}": bool(seeds.get(a) and seeds.get(a) == seeds.get(b))
	             for a, b in job.get("require_connected", [])}
	openings = []
	ff = job.get("front_openings_face")
	if ff:
		ax = "xyz".index(ff[0])
		wall = (~mat.take(range(0, m - 1), axis=ax) if ff[1] == "-"
		        else ~mat.take(range(mat.shape[ax] - m + 1, mat.shape[ax]), axis=ax))
		through = wall.all(axis=ax)
		flab, fn = ndimage.label(through)
		fsz = np.bincount(flab.ravel(), minlength=fn + 1)
		fsz[0] = 0
		openings = sorted((int(s) for s in fsz[1:] if s), reverse=True)
	ok = all(connected.values()) if connected else True
	return {
		"ok": ok,
		"components": int(ncomp),
		# DESCENDING-SORTED, TRUNCATED to 8 — kept for compatibility; NOT indexable
		# by a seed label.  Use component_sizes_cm3 / seed_sizes_cm3 for the join.
		"sizes_cm3": [round(float(s) * cell_cm3, 2) for s in sorted(sizes[1:], reverse=True)[:8]],
		"seed_labels": seeds,
		"connected": connected,
		"openings_mm2": openings,
		"component_sizes_cm3": {str(i): round(float(sizes[i]) * cell_cm3, 2)
		                        for i in range(1, ncomp + 1)},
		"seed_sizes_cm3": {k: round(float(sizes[v]) * cell_cm3, 2) for k, v in seeds.items()},
		"grid": {"origin_mm": [round(float(v), 6) for v in lo], "voxel_mm": h,
		         "shape": [int(v) for v in mat.shape]},
	}


def main(argv):
	if len(argv) < 2 or argv[1] in ("-h", "--help"):
		print(__doc__)
		return 0
	if len(argv) != 2:
		print(json.dumps({"ok": False, "error": "usage: air_topology_audit.py job.json"}))
		return 1
	job = {}
	try:
		with open(argv[1]) as f:
			job = json.load(f)
		rec = audit(job)
	except Exception as e:  # noqa: BLE001 — the receipt IS the error channel
		_receipt.emit({"ok": False, "error": f"{type(e).__name__}: {e}"}, job, "air_topology_audit")
		return 1
	_receipt.emit(rec, job, "air_topology_audit")
	return 0 if rec["ok"] else 1


if __name__ == "__main__":
	sys.exit(main(sys.argv))
