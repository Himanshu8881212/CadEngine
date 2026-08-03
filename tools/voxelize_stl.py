#!/usr/bin/env python3
"""voxelize_stl.py — watertight STL -> binary occupancy .npy on an EXPLICIT grid.

The missing bridge for FULL-GEOMETRY physics: ace_fea_runner accepts either
LMCAD ops (B-rep only) or a density .npy — but fused hybrid parts (mesh_carve
output: B-rep + implicit web welded in the voxel stage) are meshes, not ops.
This tool parity-fills the mesh onto the SAME grid frame a fea job uses
(origin_mm/voxel_mm/shape), so fixtures/load selectors keep their world
coordinates and the whole part — lattice struts included — carries load.

Usage: voxelize_stl.py job.json
Job: {stl, origin_mm:[x,y,z], voxel_mm, shape:[nx,ny,nz], out (.npy)}
Receipt (last stdout line): {ok, solid_voxels, solid_fraction_mean, bytes}.
Caveat stated, not hidden: binary occupancy at h resolves a strut of diameter
d across ~d/h cells — quote strut-level stresses as approximate below ~4 cells.
"""
import json, os, sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _stl import load_stl  # the shared binary-STL loader


def main():
	job = json.load(open(sys.argv[1]))
	tris = load_stl(job["stl"])
	lo = np.asarray(job["origin_mm"], dtype=np.float64)
	h = float(job["voxel_mm"])
	nx, ny, nz = job["shape"]
	mat = np.zeros((nx, ny, nz), dtype=bool)
	xs = lo[0] + (np.arange(nx) + 0.5) * h
	ys = lo[1] + (np.arange(ny) + 0.5) * h
	zmin, zmax = tris[:, :, 2].min(1), tris[:, :, 2].max(1)
	for k in range(nz):
		z = lo[2] + (k + 0.5) * h
		sel = tris[(zmin < z) & (zmax > z)]
		if not len(sel):
			continue
		segs = []
		for t in sel:
			pts = []
			for i in range(3):
				a, b = t[i], t[(i + 1) % 3]
				da, db = a[2] - z, b[2] - z
				if da * db < 0:
					s = da / (da - db)
					pts.append((a + s * (b - a))[:2])
			if len(pts) == 2:
				segs.append(pts)
		if not segs:
			continue
		S = np.array(segs)
		y1, y2 = S[:, 0, 1], S[:, 1, 1]
		for j in range(ny):
			yj = ys[j]
			cross = (y1 - yj) * (y2 - yj) < 0
			if not cross.any():
				continue
			cs = S[cross]
			tp = (yj - cs[:, 0, 1]) / (cs[:, 1, 1] - cs[:, 0, 1])
			xints = np.sort(cs[:, 0, 0] + tp * (cs[:, 1, 0] - cs[:, 0, 0]))
			mat[:, j, k] |= (np.searchsorted(xints, xs, side="right") % 2).astype(bool)
	rho = mat.astype(np.float32)
	np.save(job["out"], rho)
	print(json.dumps({"ok": True, "solid_voxels": int(mat.sum()),
	                  "solid_fraction_mean": float(rho.mean()),
	                  "bytes": int(rho.nbytes)}))


if __name__ == "__main__":
	main()
