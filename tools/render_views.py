#!/usr/bin/env python3
"""render_views.py — quick multi-view STL renders (matplotlib, headless).

The campaign render_assembly.png / render_half.png images come from here
(promoted from a session scratchpad 2026-07-28 so renders are reproducible
from the repo alone; the richer bordered SHEETS stay with assembly_doc.py).

Usage:  python3 tools/render_views.py in.stl out.png [iso|joint z0 z1]
  iso    — top / iso / front / bottom four-view (default)
  joint  — top / oblique / side of the z-clipped band [z0, z1]
"""
import sys, struct
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def read_stl(path):
	with open(path, "rb") as f:
		f.seek(80)
		n = struct.unpack("<I", f.read(4))[0]
		data = np.frombuffer(f.read(n * 50), dtype=np.uint8).reshape(n, 50)
		tri = data[:, 12:48].copy().view(np.float32).reshape(n, 3, 3)
		nrm = data[:, 0:12].copy().view(np.float32).reshape(n, 3)
	return tri.astype(np.float64), nrm.astype(np.float64)


def render(path, out, views, clip=None):
	tri, nrm = read_stl(path)
	if clip is not None:
		axis, lo, hi = clip
		keep = (tri[:, :, axis].min(axis=1) >= lo) & (tri[:, :, axis].max(axis=1) <= hi)
		tri, nrm = tri[keep], nrm[keep]
	light = np.array([0.4, 0.3, 0.85])
	light = light / np.linalg.norm(light)
	shade = 0.35 + 0.65 * np.clip(nrm @ light, 0, 1)
	fig = plt.figure(figsize=(7 * len(views), 7))
	for i, (el, az, title) in enumerate(views):
		ax = fig.add_subplot(1, len(views), i + 1, projection="3d")
		col = Poly3DCollection(tri, linewidths=0)
		col.set_facecolor(plt.cm.viridis(0.5 * np.ones(len(tri)))[:, :3] * shade[:, None])
		ax.add_collection3d(col)
		lo, hi = tri.min(axis=(0, 1)), tri.max(axis=(0, 1))
		c, r = (lo + hi) / 2, (hi - lo).max() / 2
		ax.set_xlim(c[0] - r, c[0] + r)
		ax.set_ylim(c[1] - r, c[1] + r)
		ax.set_zlim(c[2] - r, c[2] + r)
		ax.view_init(elev=el, azim=az)
		ax.set_title(title)
		ax.set_axis_off()
	plt.tight_layout()
	plt.savefig(out, dpi=90, bbox_inches="tight")
	plt.close()
	print(f"{out}: {len(tri)} tris")


if __name__ == "__main__":
	path, out = sys.argv[1], sys.argv[2]
	mode = sys.argv[3] if len(sys.argv) > 3 else "iso"
	if mode == "joint":
		# top-down + oblique of the joint band only (z clip)
		render(path, out, [(90, -90, "top"), (30, 30, "oblique"), (5, 0, "side")],
		       clip=(2, float(sys.argv[4]), float(sys.argv[5])))
	else:
		render(path, out, [(90, -90, "top"), (30, 30, "iso"), (0, 0, "front"), (-90, 90, "bottom")])
