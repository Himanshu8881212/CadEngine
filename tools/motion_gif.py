#!/usr/bin/env python3
"""motion_gif.py — production-grade motion study GIF for moving assemblies,
in the same design system as the render/assembly/analysis sheets.

Usage: motion_gif.py job.json
Job keys:
  title        header title (e.g. "SQUATCHEE-SPIN — motion study")
  meta         short right-aligned header note (rpm-equivalent, parts, date)
  parts        [{stl, name, color:[r,g,b] 0-1 or "#rrggbb" (optional — palette otherwise),
                 spin: {axis:[x,y,z], center:[x,y,z], turns: float} (optional —
                 static without it)}]
               turns = revolutions PER GIF LOOP; gear trains encode their true
               ratios here (e.g. ring 1.0, planet -2.43) so the loop is a
               kinematically correct cycle.
               keyframes: [{at: 0..1, translate:[x,y,z]?,
                 rotate:{axis,center,degrees}?}, ...] (optional — piecewise-
                 linear pose track; exclusive with spin. The proven pattern
                 for FIT-SWEEP animation: convert the sweep receipt's t
                 stations into keyframes and the clearance proof becomes a
                 visible motion — e.g. a retainer descending then twisting.),
               visible_from: 0..1 (optional — the part is not drawn before
                 this loop fraction; the sequence generator uses it)}]
  sequence     optional ASSEMBLY-SEQUENCE generator (exclusive with hand
               keyframes): {"axis": [0,0,1]?, "distance": mm? (default
               0.6 x scene diagonal — travel reads without wasting frame), "hold": 0.15?} — part 0 stays seated,
               every later part flies IN along `axis` to its modeled pose in
               its own staggered window. Emits the same keyframes machinery.
  frames, fps  default 48 @ 24 — one smooth ~2 s loop
  elev, azim   camera (default 18, -58); azim_sweep adds a slow +/- drift (deg)
  size_px      [W, H] default [900, 640]
  ground       true (default): light ground disc + flattened contact shadow
  date         stamped in the header (determinism: never read from the clock)
  out          output .gif  (loops forever)
  contact_out  optional PNG: 3 frames side by side (for print/docs/review)
  poster_out   optional PNG: one representative frame (default the LAST —
               the assembled state; override with poster_frame: 0..1)
  mp4_out      optional .mp4 via the system ffmpeg — Printables prefers
               video and MP4 is ~10x smaller than GIF. HONEST SKIP: when
               ffmpeg is absent the receipt says {"mp4_skipped": reason}
               and everything else still succeeds.

The camera is fitted ONCE against the union of every part's full swept bounds
(each spinning part's bbox swept around its axis), so nothing ever clips or
jumps between frames.
"""
import json, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import render_sheet as rs

DPI = 100.0


def rot_about(axis, center, deg):
	a = np.asarray(axis, dtype=np.float64)
	a = a / np.linalg.norm(a)
	c, s = math_cos_sin(deg)
	K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
	R = np.eye(3) + s * K + (1.0 - c) * (K @ K)
	ctr = np.asarray(center, dtype=np.float64)
	return R, ctr


def math_cos_sin(deg):
	r = np.radians(deg)
	return float(np.cos(r)), float(np.sin(r))


def swept_bounds(tris, spin):
	"""Conservative bounds of a part swept fully around its spin axis."""
	if not spin:
		return tris.reshape(-1, 3)
	a = np.asarray(spin["axis"], dtype=np.float64); a /= np.linalg.norm(a)
	ctr = np.asarray(spin["center"], dtype=np.float64)
	p = tris.reshape(-1, 3) - ctr
	along = p @ a
	radial = np.linalg.norm(p - along[:, None] * a[:, None].T, axis=1)
	rmax = float(radial.max())
	# ring of sample points at rmax at both axial extremes
	u = np.cross(a, [0.0, 0.0, 1.0])
	u = np.array([1.0, 0.0, 0.0]) if np.linalg.norm(u) < 1e-9 else u / np.linalg.norm(u)
	v = np.cross(a, u)
	ring = [ctr + h * a + rmax * (np.cos(t) * u + np.sin(t) * v)
	        for h in (float(along.min()), float(along.max()))
	        for t in np.linspace(0.0, 2.0 * np.pi, 24, endpoint=False)]
	return np.asarray(ring)


def pose_track(part, t):
	"""Piecewise-linear pose at loop fraction t from part["keyframes"].
	Returns (R, rot_center, translation) — identity when no track."""
	kfs = part.get("keyframes")
	if not kfs:
		return None
	kfs = sorted(kfs, key=lambda k: float(k["at"]))
	lo = kfs[0]
	hi = kfs[-1]
	if t <= float(lo["at"]):
		a, b, f = lo, lo, 0.0
	elif t >= float(hi["at"]):
		a, b, f = hi, hi, 0.0
	else:
		a = max((k for k in kfs if float(k["at"]) <= t), key=lambda k: float(k["at"]))
		b = min((k for k in kfs if float(k["at"]) >= t), key=lambda k: float(k["at"]))
		span = float(b["at"]) - float(a["at"])
		f = 0.0 if span <= 0 else (t - float(a["at"])) / span

	def lerp3(ka, kb, key):
		va = np.asarray(ka.get(key, [0.0, 0.0, 0.0]), dtype=np.float64)
		vb = np.asarray(kb.get(key, [0.0, 0.0, 0.0]), dtype=np.float64)
		return va * (1.0 - f) + vb * f

	trans = lerp3(a, b, "translate")
	ra, rb = a.get("rotate"), b.get("rotate")
	if ra or rb:
		ref = rb or ra
		da = float((ra or {}).get("degrees", 0.0))
		db = float((rb or {}).get("degrees", 0.0))
		deg = da * (1.0 - f) + db * f
		R, ctr = rot_about(ref["axis"], ref.get("center", [0.0, 0.0, 0.0]), deg)
		return (R, ctr, trans)
	return (None, None, trans)


def apply_pose(tris, pose):
	if pose is None:
		return tris
	R, ctr, trans = pose
	out = tris.reshape(-1, 3)
	if R is not None:
		out = (out - ctr) @ R.T + ctr
	out = out + trans
	return out.reshape(-1, 3, 3)


def track_bounds(part):
	"""Conservative camera bounds for a keyframed part: its bbox corners at
	the keyframe stations and midpoints (linear tracks peak at stations)."""
	kfs = part.get("keyframes")
	if not kfs:
		return None
	ts = sorted({float(k["at"]) for k in kfs})
	ts += [(a + b) / 2.0 for a, b in zip(ts, ts[1:])]
	pts = []
	for t in ts:
		pts.append(apply_pose(part["tris"], pose_track(part, t)).reshape(-1, 3))
	pts = np.vstack(pts)
	lo, hi = pts.min(axis=0), pts.max(axis=0)
	corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
	return corners


def generate_sequence(parts, seq):
	"""Assembly-sequence keyframes: part 0 seated; each later part flies in
	along `axis` from `distance` away, in its own staggered window."""
	axis = np.asarray(seq.get("axis", [0.0, 0.0, 1.0]), dtype=np.float64)
	axis = axis / (np.linalg.norm(axis) + 1e-12)
	all_pts = np.vstack([p["tris"].reshape(-1, 3) for p in parts])
	diag = float(np.linalg.norm(all_pts.max(axis=0) - all_pts.min(axis=0)))
	dist = float(seq.get("distance", 0.6 * diag))
	hold = float(seq.get("hold", 0.15))
	movers = parts[1:]
	if not movers:
		return
	window = (1.0 - hold) / len(movers)
	off = list(axis * dist)
	for i, p in enumerate(movers):
		t0, t1 = i * window, (i + 1) * window
		p["visible_from"] = t0
		p["keyframes"] = [
			{"at": t0, "translate": off},
			{"at": t1, "translate": [0.0, 0.0, 0.0]},
		]


def main():
	job = json.load(open(sys.argv[1]))
	frames_n = int(job.get("frames", 48))
	fps = float(job.get("fps", 24))
	Wpx, Hpx = job.get("size_px", [900, 640])
	elev = float(job.get("elev", 18.0))
	azim0 = float(job.get("azim", -58.0))
	sweep = float(job.get("azim_sweep", 0.0))

	parts = []
	for i, p in enumerate(job["parts"]):
		tris = rs.load_stl(p["stl"])
		if len(tris) > 120_000:
			tris = tris[:: int(np.ceil(len(tris) / 120_000))]
		raw = p.get("color", rs.PALETTE[i % len(rs.PALETTE)])
		if isinstance(raw, str):  # accept "#rrggbb" like assembly_doc
			raw = raw.lstrip("#")
			raw = [int(raw[k:k + 2], 16) / 255.0 for k in (0, 2, 4)]
		color = np.asarray(raw, dtype=np.float64)
		if p.get("spin") and p.get("keyframes"):
			raise ValueError(f"part {i}: 'spin' and 'keyframes' are exclusive — encode the spin as rotate keyframes instead")
		parts.append({"tris": tris, "color": color, "spin": p.get("spin"), "keyframes": p.get("keyframes"),
		              "visible_from": float(p.get("visible_from", 0.0)), "name": p.get("name", f"part {i+1}")})

	if job.get("sequence") is not None:
		if any(p["keyframes"] for p in parts):
			raise ValueError("'sequence' and hand-written 'keyframes' are exclusive")
		generate_sequence(parts, job["sequence"])

	fit_pts = np.vstack([
		track_bounds(p) if p.get("keyframes") else swept_bounds(p["tris"], p["spin"]) for p in parts
	])
	zmin = float(min(p["tris"][:, :, 2].min() for p in parts))

	images = []
	header_h = 44.0
	for f in range(frames_n):
		t = f / frames_n
		fig = plt.figure(figsize=(Wpx / DPI, Hpx / DPI), dpi=DPI)
		fig.patch.set_facecolor(rs.STYLE["page_fill"])
		W, H = rs.fig_px(fig)
		rs.px_rect(fig, 6.0, 6.0, W - 12.0, H - 12.0, edge=rs.STYLE["border"], lw=rs.STYLE["border_pt"], z=5)
		rs.px_text(fig, 16.0, H - header_h / 2.0 - 8.0, job["title"], rs.STYLE["fs_caption"] + 3.0,
		           rs.STYLE["ink"], va="center", weight="bold", z=8)
		rs.px_text(fig, W - 16.0, H - header_h / 2.0 - 8.0, job.get("meta", ""), rs.STYLE["fs_table"],
		           rs.STYLE["ink2"], ha="right", va="center", z=8)
		rs.px_line(fig, 12.0, H - header_h, W - 12.0, H - header_h, rs.STYLE["border"], rs.STYLE["border_pt"], z=6)

		view = (14.0, 12.0, W - 28.0, H - header_h - 20.0)
		ax = fig.add_axes((0, 0, 1, 1), projection="3d", computed_zorder=False)
		ax.set_position((view[0] / W, view[1] / H, view[2] / W, view[3] / H))
		ax.set_proj_type("ortho")
		azim = azim0 + sweep * np.sin(2.0 * np.pi * t)
		ax.view_init(elev=elev, azim=azim)
		ax.set_axis_off(); ax.patch.set_alpha(0.0)
		rs.fit_view(ax, fit_pts, elev, azim, view, fill=0.92)
		light = rs.camera_dir(elev + 26.0, azim - 20.0)

		# ONE merged collection for all parts: matplotlib depth-sorts only WITHIN
		# a collection, so per-part collections let a blade draw over a body it
		# is actually behind (caught reviewing frame 2 of the first render).
		shadow_polys, all_tris, all_colors = [], [], []
		for p in parts:
			if t < p.get("visible_from", 0.0):
				continue
			tris = p["tris"]
			if p["spin"]:
				R, ctr = rot_about(p["spin"]["axis"], p["spin"]["center"],
				                   360.0 * p["spin"]["turns"] * t)
				tris = (tris.reshape(-1, 3) - ctr) @ R.T + ctr
				tris = tris.reshape(-1, 3, 3)
			elif p.get("keyframes"):
				tris = apply_pose(tris, pose_track(p, t))
			lam = 0.52 + 0.48 * np.clip(np.abs(rs.tri_normals(tris) @ light), 0.0, 1.0)
			all_tris.append(tris)
			all_colors.append(np.clip(p["color"][None, :] * lam[:, None], 0.0, 1.0))
			if job.get("ground", True):
				sh = tris.copy()
				sh[:, :, 2] = zmin - 0.15
				shadow_polys.append(sh[:: max(1, len(sh) // 4000)])
		if shadow_polys:
			ax.add_collection3d(Poly3DCollection(np.concatenate(shadow_polys),
				facecolors=(0.0, 0.0, 0.0, 0.05), edgecolors="none", zsort="min"))
		if all_tris:
			ax.add_collection3d(Poly3DCollection(np.concatenate(all_tris),
				facecolors=np.concatenate(all_colors), edgecolors="none", zsort="average"))

		fig.canvas.draw()
		buf = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]
		images.append(Image.fromarray(buf))
		plt.close(fig)

	images[0].save(job["out"], save_all=True, append_images=images[1:],
	               duration=int(round(1000.0 / fps)), loop=0, optimize=True)
	result = {"ok": True, "out": os.path.abspath(job["out"]), "frames": frames_n, "fps": fps,
	          "bytes": os.path.getsize(job["out"])}
	if job.get("contact_out"):
		picks = [0, frames_n // 3, (2 * frames_n) // 3]
		strip = Image.new("RGB", (images[0].width * 3 + 16, images[0].height), "#f5f4f2")
		for i, k in enumerate(picks):
			strip.paste(images[k], (i * (images[0].width + 8), 0))
		strip.save(job["contact_out"])
		result["contact_out"] = os.path.abspath(job["contact_out"])
	if job.get("poster_out"):
		k = int(round(float(job.get("poster_frame", 1.0)) * (frames_n - 1)))
		images[max(0, min(frames_n - 1, k))].save(job["poster_out"])
		result["poster_out"] = os.path.abspath(job["poster_out"])
	if job.get("mp4_out"):
		import shutil
		import subprocess
		import tempfile

		ffmpeg = shutil.which("ffmpeg")
		if not ffmpeg:
			result["mp4_out"] = None
			result["mp4_skipped"] = "ffmpeg not found on PATH — GIF written; install ffmpeg for MP4"
		else:
			with tempfile.TemporaryDirectory() as td:
				for i, im in enumerate(images):
					im.save(os.path.join(td, f"f{i:04d}.png"))
				run = subprocess.run(
					[ffmpeg, "-y", "-framerate", str(fps), "-i", os.path.join(td, "f%04d.png"),
					 "-c:v", "libx264", "-pix_fmt", "yuv420p",
					 "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2", job["mp4_out"]],
					capture_output=True, text=True)
			if run.returncode == 0 and os.path.exists(job["mp4_out"]):
				result["mp4_out"] = os.path.abspath(job["mp4_out"])
				result["mp4_bytes"] = os.path.getsize(job["mp4_out"])
			else:
				result["mp4_out"] = None
				result["mp4_skipped"] = f"ffmpeg failed: {run.stderr[-200:]}"
	print(json.dumps(result))


if __name__ == "__main__":
	main()
