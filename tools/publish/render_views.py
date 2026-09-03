#!/usr/bin/env python3
"""render_views.py — quick multi-view STL renders (matplotlib, headless).

The campaign render_assembly.png / render_half.png images come from here
(promoted from a session scratchpad 2026-07-28 so renders are reproducible
from the repo alone; the richer bordered SHEETS stay with assembly_doc.py).

Usage:  python3 tools/render_views.py in.stl out.png [iso|joint z0 z1]
        python3 tools/render_views.py --help
  iso    — top / iso / front / bottom four-view (default)
  joint  — top / oblique / side of the z-clipped band [z0, z1]

The LAST stdout line is a JSON receipt ({ok, out, views, triangles, clipped,
bytes} or {ok:false, error}); bad arguments are refused with that receipt and
exit 1, never with a bare traceback.
"""
import json, os, sys, struct
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
	return {"ok": True, "out": os.path.abspath(out), "views": len(views),
	        "triangles": int(len(tri)), "clipped": clip is not None,
	        "bytes": os.path.getsize(out)}


def main(argv):
	# digest F9: `--help` used to be read as the input STL path and stack-trace.
	if len(argv) < 2 or argv[1] in ("-h", "--help"):
		print(__doc__)
		return 0
	try:
		if len(argv) < 3:
			raise ValueError("usage: render_views.py in.stl out.png [iso | joint z0 z1]")
		path, out = argv[1], argv[2]
		mode = argv[3] if len(argv) > 3 else "iso"
		if mode == "joint":
			if len(argv) < 6:
				raise ValueError("mode 'joint' needs z0 and z1: render_views.py in.stl out.png joint z0 z1")
			# top-down + oblique of the joint band only (z clip)
			rec = render(path, out, [(90, -90, "top"), (30, 30, "oblique"), (5, 0, "side")],
			             clip=(2, float(argv[4]), float(argv[5])))
		elif mode == "iso":
			rec = render(path, out, [(90, -90, "top"), (30, 30, "iso"), (0, 0, "front"), (-90, 90, "bottom")])
		else:
			raise ValueError(f"unknown mode {mode!r} — use 'iso' (default) or 'joint z0 z1'")
	except Exception as e:  # noqa: BLE001 — the LAST stdout line is the receipt
		print(json.dumps({"ok": False, "error": f"{type(e).__name__}: {e}"}))
		return 1
	print(json.dumps(rec))
	return 0


if __name__ == "__main__":
	sys.exit(main(sys.argv))
