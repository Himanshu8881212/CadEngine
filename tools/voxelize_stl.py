#!/usr/bin/env python3
"""voxelize_stl.py — watertight STL -> binary occupancy .npy on an EXPLICIT grid.

The missing bridge for FULL-GEOMETRY physics: ace_fea_runner accepts either
LMCAD ops (B-rep only) or a density .npy — but fused hybrid parts (mesh_carve
output: B-rep + implicit web welded in the voxel stage) are meshes, not ops.
This tool parity-fills the mesh onto the SAME grid frame a fea job uses
(origin_mm/voxel_mm/shape), so fixtures/load selectors keep their world
coordinates and the whole part — lattice struts included — carries load.

Usage
-----
  voxelize_stl.py job.json              voxelize; writes <out> + <out>.provenance.json
  voxelize_stl.py --verify job.json     DO NOT recompute — check the field on disk
                                        is still fresh for its source STL; ok:false
                                        + exit 1 when it is stale
  voxelize_stl.py --verify-field F.npy  same check keyed on the field itself
  voxelize_stl.py --help

Job: {stl, origin_mm:[x,y,z], voxel_mm, shape:[nx,ny,nz], out (.npy),
      provenance? (bool, default true — write the freshness sidecar),
      verify_source? (bool, default false — refuse to overwrite a field whose
                      sidecar says it came from a DIFFERENT stl)}
Receipt (last stdout line):
      {ok, solid_voxels, solid_fraction_mean, bytes, source_stl, source_sha256,
       geometry_hash (mesh:sha256:… of the STL), field_hash (density:sha256:…),
       provenance (sidecar path or null)}.

FRESHNESS — why this tool stamps a sidecar (gripper F8, a shipped BLOCKER)
--------------------------------------------------------------------------
A density field is an INPUT artifact that outlives its source. A campaign
amended geometry, never re-voxelised, and every voxel consumer (ace_fea /
ace_modal / ace_thermal) went on eating the stale grid: 62552 vs 64328 solid
voxels, first mode 70.54 -> 65.84 Hz. Nothing anywhere could tell the two apart.

So every write also emits `<out>.provenance.json`:

    {"schema": "lmcad.field.provenance.v1",
     "source": {"path", "sha256", "geometry_hash": "mesh:sha256:…", "bytes"},
     "grid":   {"origin_mm", "voxel_mm", "shape"},
     "field":  {"path", "sha256", "geometry_hash": "density:sha256:…",
                "dtype", "shape", "solid_voxels"},
     "tool": "voxelize_stl.py"}

`geometry_hash` is written in provenance.py's OWN vocabulary, so the value here
is byte-comparable with the `geometry_hash` an ACE runner puts in its receipt —
the field stops being decorative and becomes checkable. `check_field_freshness`
is the public entry point a consumer calls; `--verify` is its CLI.

No wall clock is ever read: same STL + same grid -> byte-identical .npy AND
byte-identical sidecar.

Slice-plane degeneracy (horn F8 / cubesat F4) — FIXED HERE
----------------------------------------------------------
The parity fill used to keep a triangle edge only when `da*db < 0`, so a
triangle with a vertex EXACTLY on the slice plane contributed one crossing and
was dropped. Exact-B-rep STL exports emit a whole VERTEX RING at the mid-height
of every cylindrical face, so at a commensurate pitch every triangle of a bore
vanished and the slice read fully solid (a wide-open bore filled with phantom
material) or fully empty. `parity_fill` now classifies by a HALF-OPEN sign rule
(`d > 0`), which yields exactly 0 or 2 crossings per triangle for every input
including degenerate ones, and does the same on the y scanline. Non-degenerate
slices are bit-identical to the old rule.

Caveat stated, not hidden: binary occupancy at h resolves a strut of diameter
d across ~d/h cells — quote strut-level stresses as approximate below ~4 cells.
"""
import hashlib
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _stl import load_stl  # the shared binary-STL loader
import provenance as _prov

PROVENANCE_SCHEMA = "lmcad.field.provenance.v1"


# ----------------------------------------------------------------- geometry --
def parity_fill(tris, origin, h, shape):
	"""Binary occupancy of `tris` on the grid (origin, h, shape) -> bool array.

	Scanline parity in x, per (y, z) cell centre. Crossings are classified with
	a HALF-OPEN sign rule (`d > 0`) rather than `da*db < 0`: a vertex or an
	endpoint lying exactly on the plane/scanline is assigned to the closed side
	instead of deleting the whole triangle or segment. That makes the crossing
	count per triangle exactly 0 or 2 for every input, so a mesh whose vertex
	ring coincides with a slice centre fills correctly instead of vanishing.
	"""
	tris = np.asarray(tris, dtype=np.float64)
	lo = np.asarray(origin, dtype=np.float64)
	h = float(h)
	nx, ny, nz = (int(v) for v in shape)
	mat = np.zeros((nx, ny, nz), dtype=bool)
	xs = lo[0] + (np.arange(nx) + 0.5) * h
	ys = lo[1] + (np.arange(ny) + 0.5) * h
	zmin, zmax = tris[:, :, 2].min(1), tris[:, :, 2].max(1)
	for k in range(nz):
		z = lo[2] + (k + 0.5) * h
		sel = tris[(zmin <= z) & (zmax >= z)]
		if not len(sel):
			continue
		segs = []
		for t in sel:
			pts = []
			for i in range(3):
				a, b = t[i], t[(i + 1) % 3]
				da, db = a[2] - z, b[2] - z
				if (da > 0.0) != (db > 0.0):       # half-open: 0 counts as "below"
					s = da / (da - db)             # denominator can never be 0 here
					pts.append((a + s * (b - a))[:2])
			if len(pts) == 2:
				segs.append(pts)
		if not segs:
			continue
		S = np.array(segs)
		y1, y2 = S[:, 0, 1], S[:, 1, 1]
		for j in range(ny):
			yj = ys[j]
			cross = (y1 > yj) != (y2 > yj)         # half-open on the scanline too
			if not cross.any():
				continue
			cs = S[cross]
			tp = (yj - cs[:, 0, 1]) / (cs[:, 1, 1] - cs[:, 0, 1])
			xints = np.sort(cs[:, 0, 0] + tp * (cs[:, 1, 0] - cs[:, 0, 0]))
			mat[:, j, k] |= (np.searchsorted(xints, xs, side="right") % 2).astype(bool)
	return mat


# --------------------------------------------------------------- provenance --
def sha256_file(path):
	h = hashlib.sha256()
	with open(path, "rb") as f:
		for chunk in iter(lambda: f.read(1 << 20), b""):
			h.update(chunk)
	return h.hexdigest()


def provenance_path(npy_path):
	"""Sidecar location for a field. One rule, no search."""
	return f"{npy_path}.provenance.json"


def build_provenance(stl_path, npy_path, origin_mm, voxel_mm, shape, solid_voxels, dtype):
	"""The freshness record. Pure function of file CONTENT — never the clock."""
	return {
		"schema": PROVENANCE_SCHEMA,
		"tool": "voxelize_stl.py",
		"source": {
			"path": os.path.basename(stl_path),
			"sha256": sha256_file(stl_path),
			"geometry_hash": _prov.geometry_hash(stl_path=stl_path),
			"bytes": os.path.getsize(stl_path),
		},
		"grid": {
			"origin_mm": [float(v) for v in origin_mm],
			"voxel_mm": float(voxel_mm),
			"shape": [int(v) for v in shape],
		},
		"field": {
			"path": os.path.basename(npy_path),
			"sha256": sha256_file(npy_path),
			"geometry_hash": _prov.geometry_hash(density_path=npy_path),
			"dtype": str(dtype),
			"shape": [int(v) for v in shape],
			"solid_voxels": int(solid_voxels),
		},
	}


def write_provenance(rec, npy_path):
	p = provenance_path(npy_path)
	with open(p, "w", encoding="utf-8") as f:
		json.dump(rec, f, indent=1, sort_keys=True)
		f.write("\n")
	return p


def read_provenance(npy_path):
	"""The sidecar for `npy_path`, or None when there is none."""
	p = provenance_path(npy_path)
	if not os.path.exists(p):
		return None
	with open(p, encoding="utf-8") as f:
		return json.load(f)


def check_field_freshness(npy_path, stl_path=None, grid=None):
	"""Is the density field on disk still the one its source STL produces?

	THE consumer-side entry point (a runner should call this before it eats a
	.npy). Returns {ok, fresh, reasons:[...], ...} — `fresh` is only True when
	every recorded hash still matches what is on disk right now. An ABSENT
	sidecar is `fresh: null` with reason "no_provenance": unknown is reported as
	unknown, never as fresh.

	`stl_path` overrides the recorded source path (a job knows where its STL
	is); `grid` = {origin_mm, voxel_mm, shape} is compared when given, so a
	field built on a different frame is caught even if the STL is unchanged.
	"""
	out = {"ok": True, "field": os.path.abspath(npy_path), "fresh": None, "reasons": []}
	if not os.path.exists(npy_path):
		return {**out, "ok": False, "fresh": False, "reasons": ["field_missing"],
		        "error": f"density field not found: {npy_path}"}
	rec = read_provenance(npy_path)
	if rec is None:
		out["reasons"].append("no_provenance")
		out["error"] = (f"no freshness record beside {npy_path} "
		                f"(expected {provenance_path(npy_path)}) — re-run voxelize_stl.py "
		                f"to stamp one; staleness CANNOT be ruled out")
		out["ok"] = False
		return out
	if rec.get("schema") != PROVENANCE_SCHEMA:
		return {**out, "ok": False, "fresh": False, "reasons": ["bad_schema"],
		        "error": f"provenance schema {rec.get('schema')!r} != {PROVENANCE_SCHEMA!r}"}
	src = rec.get("source", {})
	fld = rec.get("field", {})
	out["recorded"] = {"source": src.get("path"), "source_sha256": src.get("sha256"),
	                   "geometry_hash": src.get("geometry_hash"),
	                   "field_geometry_hash": fld.get("geometry_hash")}
	reasons = []
	# 1) the field itself must not have been rewritten behind the record
	now_field = _prov.geometry_hash(density_path=npy_path)
	out["field_geometry_hash"] = now_field
	if fld.get("geometry_hash") != now_field:
		reasons.append("field_modified")
	# 2) the source STL must still hash to what produced it
	spath = stl_path or os.path.join(os.path.dirname(os.path.abspath(npy_path)),
	                                 src.get("path", ""))
	out["source"] = os.path.abspath(spath) if spath else None
	if not spath or not os.path.exists(spath):
		reasons.append("source_missing")
	else:
		now_src = _prov.geometry_hash(stl_path=spath)
		out["source_geometry_hash"] = now_src
		if src.get("geometry_hash") != now_src:
			reasons.append("source_changed")
	# 3) the grid frame the consumer intends to use must be the one it was built on
	if grid:
		g = rec.get("grid", {})
		want = {"origin_mm": [float(v) for v in grid["origin_mm"]],
		        "voxel_mm": float(grid["voxel_mm"]),
		        "shape": [int(v) for v in grid["shape"]]}
		if (g.get("origin_mm") != want["origin_mm"] or g.get("voxel_mm") != want["voxel_mm"]
				or g.get("shape") != want["shape"]):
			reasons.append("grid_changed")
			out["recorded_grid"], out["requested_grid"] = g, want
	out["reasons"] = reasons
	out["fresh"] = not reasons
	out["ok"] = not reasons
	if reasons:
		out["error"] = ("STALE density field: " + ", ".join(reasons) +
		                f" — re-run voxelize_stl.py before consuming {npy_path}")
	return out


# ---------------------------------------------------------------------- CLI --
def voxelize(job):
	stl_path = job["stl"]
	tris = load_stl(stl_path)
	lo = np.asarray(job["origin_mm"], dtype=np.float64)
	h = float(job["voxel_mm"])
	shape = [int(v) for v in job["shape"]]
	out_path = job["out"]
	if job.get("verify_source"):
		prev = read_provenance(out_path)
		if prev and prev.get("source", {}).get("geometry_hash") not in (
				None, _prov.geometry_hash(stl_path=stl_path)):
			raise ValueError(
				f"verify_source: {out_path} was built from a DIFFERENT geometry "
				f"({prev['source'].get('path')} {prev['source'].get('geometry_hash')}); "
				f"refusing to overwrite silently — clear verify_source to proceed")
	mat = parity_fill(tris, lo, h, shape)
	rho = mat.astype(np.float32)
	np.save(out_path, rho)
	receipt = {"ok": True, "solid_voxels": int(mat.sum()),
	           "solid_fraction_mean": float(rho.mean()),
	           "bytes": int(rho.nbytes),
	           "source_stl": os.path.abspath(stl_path)}
	rec = None
	if job.get("provenance", True):
		rec = build_provenance(stl_path, out_path, lo, h, shape, int(mat.sum()), rho.dtype)
		p = write_provenance(rec, out_path)
		receipt["source_sha256"] = rec["source"]["sha256"]
		receipt["geometry_hash"] = rec["source"]["geometry_hash"]
		receipt["field_hash"] = rec["field"]["geometry_hash"]
		receipt["provenance"] = os.path.abspath(p)
	else:
		receipt["provenance"] = None
	return receipt


def main(argv):
	if len(argv) < 2 or argv[1] in ("-h", "--help"):
		print(__doc__)
		return 0
	mode, args = "run", argv[1:]
	if args[0] in ("--verify", "--verify-field"):
		mode, args = args[0], args[1:]
	if len(args) != 1:
		print(json.dumps({"ok": False,
		                  "error": "usage: voxelize_stl.py [--verify|--verify-field] <job.json|field.npy>"}))
		return 1
	try:
		if mode == "--verify-field":
			rec = check_field_freshness(args[0])
		elif mode == "--verify":
			job = json.load(open(args[0]))
			rec = check_field_freshness(job["out"], stl_path=job.get("stl"),
			                            grid={"origin_mm": job["origin_mm"],
			                                  "voxel_mm": job["voxel_mm"],
			                                  "shape": job["shape"]})
		else:
			rec = voxelize(json.load(open(args[0])))
	except Exception as e:  # noqa: BLE001 — the receipt IS the error channel
		print(json.dumps({"ok": False, "error": f"{type(e).__name__}: {e}"}))
		return 1
	print(json.dumps(rec))
	return 0 if rec.get("ok") else 1


if __name__ == "__main__":
	sys.exit(main(sys.argv))
