#!/usr/bin/env python3
"""_stl.py — the ONE binary-STL loader shared by the tools/ scripts.

Single source of truth (was copy-pasted into render_sheet/production_dossier/
air_topology_audit/voxelize_stl). Import it as a sibling module:

	sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
	from _stl import load_stl
"""
import os

import numpy as np


def load_stl(path):
	"""Load a BINARY stl -> (n,3,3) float64 triangle array. Refuses ASCII or
	size-inconsistent files with an explicit error (never silently guesses)."""
	size = os.path.getsize(path)
	if size < 84:
		raise ValueError(f"'{path}' is too small to be a binary STL ({size} bytes)")
	with open(path, "rb") as f:
		f.seek(80)
		n = int(np.fromfile(f, dtype=np.uint32, count=1)[0])
		if size != 84 + 50 * n:
			raise ValueError(
				f"'{path}' is not a well-formed binary STL: header says {n} triangles "
				f"(needs {84 + 50 * n} bytes) but the file is {size} bytes — ASCII STLs are not supported"
			)
		data = np.fromfile(f, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=n)
	if n == 0:
		raise ValueError(f"'{path}' contains zero triangles")
	return data["v"].astype(np.float64)


def write_stl(path, tris):
	"""Write (n,3,3) float triangles as binary STL (normals recomputed;
	80-byte header names the writer). The mirror of load_stl."""
	import struct

	import numpy as np

	tris = np.asarray(tris, dtype=np.float64).reshape(-1, 3, 3)
	n = len(tris)
	e1 = tris[:, 1] - tris[:, 0]
	e2 = tris[:, 2] - tris[:, 0]
	nrm = np.cross(e1, e2)
	lens = np.linalg.norm(nrm, axis=1)
	nrm = np.where(lens[:, None] > 1e-30, nrm / np.maximum(lens, 1e-30)[:, None], 0.0)
	with open(path, "wb") as f:
		f.write(b"lmcad tools plate writer".ljust(80, b" "))
		f.write(struct.pack("<I", n))
		for i in range(n):
			f.write(struct.pack("<3f", *nrm[i]))
			for v in tris[i]:
				f.write(struct.pack("<3f", *v))
			f.write(struct.pack("<H", 0))
