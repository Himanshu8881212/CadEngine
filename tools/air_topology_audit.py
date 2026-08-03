#!/usr/bin/env python3
"""Internal-air topology audit — the gate the TL-91 defect taught us (2026-07-09).

Voxelizes a watertight STL by per-slice parity fill and flood-labels the INTERNAL AIR.
For any design whose function lives in internal channels (speaker lines, manifolds,
ducts): the functional air path must be ONE connected component between the named seed
points, and the through-wall opening census must match the design intent. Watertight +
geometric_ok CANNOT see this — they gate the material, not the void.

Usage: air_topology_audit.py job.json
job: {"stl": path, "voxel_mm": 1.0, "wall_margin_mm": 7, "open_faces": ["y+"]  (faces
      sealed later by another part — the domain extends to them minus 1 voxel),
      "seeds": {"name": [x,y,z], ...}, "require_connected": [["chamber","port"], ...],
      "front_openings_face": "y-"|null }
Last stdout line: {"ok", "components", "sizes_cm3", "seed_labels", "connected": {...},
                   "openings": [...]} — ok:false if any required pair is disconnected.
"""
import json, os, sys
import numpy as np
from scipy import ndimage

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _stl import load_stl  # the shared binary-STL loader

def voxelize(tris, h):
	lo = tris.min((0,1)); hi = tris.max((0,1))
	nx, ny, nz = [int(np.ceil((hi[i]-lo[i])/h)) for i in range(3)]
	mat = np.zeros((nx, ny, nz), dtype=bool)
	xs = lo[0] + (np.arange(nx)+0.5)*h
	ys = lo[1] + (np.arange(ny)+0.5)*h
	zmin, zmax = tris[:,:,2].min(1), tris[:,:,2].max(1)
	for k in range(nz):
		z = lo[2] + (k+0.5)*h
		sel = tris[(zmin < z) & (zmax > z)]
		if not len(sel): continue
		segs = []
		for t in sel:
			pts = []
			for i in range(3):
				a, b = t[i], t[(i+1)%3]
				da, db = a[2]-z, b[2]-z
				if da*db < 0:
					s = da/(da-db); pts.append((a+s*(b-a))[:2])
			if len(pts) == 2: segs.append(pts)
		if not segs: continue
		S = np.array(segs)
		y1, y2 = S[:,0,1], S[:,1,1]
		for j in range(ny):
			yj = ys[j]
			cross = (y1-yj)*(y2-yj) < 0
			if not cross.any(): continue
			cs = S[cross]
			tp = (yj-cs[:,0,1])/(cs[:,1,1]-cs[:,0,1])
			xints = np.sort(cs[:,0,0] + tp*(cs[:,1,0]-cs[:,0,0]))
			mat[:, j, k] |= (np.searchsorted(xints, xs, side="right") % 2).astype(bool)
	return mat, lo

def main():
	job = json.load(open(sys.argv[1]))
	h = job.get("voxel_mm", 1.0)
	tris = load_stl(job["stl"])
	mat, lo = voxelize(tris, h)
	nx, ny, nz = mat.shape
	m = int(round(job.get("wall_margin_mm", 7) / h))
	slc = [slice(m, nx-m), slice(m, ny-m), slice(m, nz-m)]
	for f in job.get("open_faces", []):
		ax = "xyz".index(f[0]); hi_face = f[1] == "+"
		slc[ax] = slice(slc[ax].start, mat.shape[ax]-1) if hi_face else slice(1, slc[ax].stop)
	dom = np.zeros_like(mat); dom[tuple(slc)] = True
	air = dom & ~mat
	lab, ncomp = ndimage.label(air)
	sizes = np.bincount(lab.ravel()); sizes[0] = 0
	seeds = {k: int(lab[tuple(int((v[i]-lo[i])/h) for i in range(3))]) for k, v in job.get("seeds", {}).items()}
	connected = {f"{a}<->{b}": bool(seeds.get(a) and seeds.get(a) == seeds.get(b)) for a, b in job.get("require_connected", [])}
	openings = []
	ff = job.get("front_openings_face")
	if ff:
		ax = "xyz".index(ff[0])
		wall = ~mat.take(range(0, m-1), axis=ax) if ff[1] == "-" else ~mat.take(range(mat.shape[ax]-m+1, mat.shape[ax]), axis=ax)
		through = wall.all(axis=ax)
		flab, fn = ndimage.label(through)
		fsz = np.bincount(flab.ravel()); fsz[0] = 0
		openings = sorted((int(s) for s in fsz[1:] if s), reverse=True)
	ok = all(connected.values()) if connected else True
	print(json.dumps({"ok": ok, "components": int(ncomp),
		"sizes_cm3": [round(float(s)*h**3/1000, 2) for s in sorted(sizes[1:], reverse=True)[:8]],
		"seed_labels": seeds, "connected": connected, "openings_mm2": openings}))

if __name__ == "__main__":
	main()
