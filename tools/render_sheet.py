#!/usr/bin/env python3
"""render_sheet.py — the 12-view VISION contact sheet: one PNG that lets the
designing model SEE a part from every side, inside and out, every iteration.

Usage: render_sheet.py job.json      (also: render_sheet.py --help)
Job keys:
  stl        : path to ONE binary STL,  OR
  stls       : [paths] rendered as an OVERLAY (distinct colors + legend)
  out        : output PNG path (required)
  base_dir   : optional root for every relative path in the job. Without it a
               relative INPUT is looked for under the CWD first (unchanged) and
               then under the JOB FILE's own directory, and a fallback that
               fires is named in the receipt's `notes` — so a job carrying
               `parts/x.stl` rebuilds from the repo root as well as from the
               part directory (gripper F10). Nothing found under any root is a
               refusal that lists the roots tried, never a guess.
  dimensions : optional [{kind:"linear", a, b, label?, view?, offset?} |
               {kind:"diameter", center, axis, radius, label?, view?}] —
               engineering callouts drawn on the ortho panels (views auto-
               picked for least foreshortening / face-on circles). Values
               should come from `measure_dimension` receipts or
               tools/dim_suggest.py so the numbers are ANALYTIC.
  build_dir  : print/build direction, default [0,0,1] — the bed view rotates
               the part so this vector points +Z and rests it on a drawn bed
  sections   : {"x": v, "y": v, "z": v} cut-plane overrides (default: bbox
               centers; any subset may be given)
  date       : optional string stamped in the header (NEVER read from the
               clock — determinism doctrine: same job, byte-identical PNG)
  max_px     : long-edge pixel cap for the whole sheet, default 1600
               (vision-token economy: ONE image, bounded size)

THE 12 PANELS (4x3 grid, orthographic, shared scale for orthos+isos):
   1 top (+Z)    2 bottom (-Z)   3 front (-Y)    4 back (+Y)
   5 left (-X)   6 right (+X)    7 iso az+45     8 iso az-45
   9 bed view (elev 8, azim -60, part in PRINT orientation per build_dir,
     resting on a drawn bed at z=0, with a flattened ground shadow)
  10 SECTION x=cx   11 SECTION y=cy   12 SECTION z=cz  (TRUE cut lines:
     exact triangle/plane intersection segments, with the MATERIAL region
     filled by a 45-degree scanline-parity hatch — voids read as voids)

Presentation contract — the SHARED cadcode design system (STYLE below; the
assembly doc imports it so the two documents read as one family):
- single font family DejaVu Sans, fixed hierarchy (title 18 bold ·
  panel-caption 8.5 bold uppercase tracked · body 9 · table 8.5 ·
  titleblock-label 6.5 · titleblock-value 9.5) — no other sizes;
- fixed 28 px page margins, 16 px gutters, every content zone a bordered
  panel (0.8 pt) with a shaded caption strip attached to its top edge and
  12 px internal padding — nothing floats, nothing touches a border;
- header band (title left, quiet meta right, rule underneath) + a thin page
  frame; balloons/markers are DRAWN circle patches + plain digits, NEVER
  Unicode circled-digit glyphs (missing in some font paths -> '?').
- top + front orthos carry overall <-> dimensions in mm.
- shading is two-sided (0.35 + 0.65*|n.L|) on winding-recomputed normals,
  so coplanar triangles shade identically regardless of stored winding —
  no facet streaks on flat faces.

Design choice, on purpose: the rear iso (azim 180+45) is DROPPED — the back
ortho (panel 4) already covers the rear aspect, and the bed view earns the
slot because it is the one panel the printer cares about. That keeps the
sheet at exactly 12 panels.

Honesty notes:
- Binary STL only (LMCAD's export_stl writes binary); an ASCII or truncated
  file is refused with a clear error, never guessed at.
- Shaded views above ~300k total triangles are uniformly subsampled to stay
  renderable; this is DECLARED in the receipt (decimated/shown_triangles)
  and on the sheet itself. Sections are always exact (never decimated).
- The section hatch is a parity fill of the raw cut segments; it assumes the
  cut curves close (watertight mesh). Odd parity residue on a scanline is
  dropped, never guessed into material.
- Last stdout line is a JSON receipt: {"ok", "out", "panels": 12,
  "px": [w, h], ...}; failures print {"ok": false, "error": ...} and exit 1.
"""

import functools
import json
import os
import sys

import matplotlib
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _stl import load_stl  # noqa: E402 — the shared binary-STL loader

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.font_manager import FontProperties
from matplotlib.lines import Line2D
from matplotlib.patches import Ellipse, Rectangle
from matplotlib.textpath import TextPath
from mpl_toolkits.mplot3d import proj3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ------------------------------------------------------------------ style --
# THE design system. assembly_doc.py imports STYLE + the chrome helpers below
# so both documents share one visual language. All lengths in px of the final
# PNG (layout math is px-based, converted to figure fractions at draw time);
# all font sizes in pt. The hierarchy lists EVERY permitted font size.
STYLE = {
	"font": "DejaVu Sans",       # single family, pinned (byte-deterministic)
	"ink": "#1a1d21",            # primary text
	"ink2": "#555b63",           # secondary / quiet text
	"border": "#3a3f45",         # panel borders, balloons, strong rules
	"border_pt": 0.8,
	"rule": "#c8c8c4",           # light table row rules
	"rule_pt": 0.4,
	"page_fill": "#f5f4f2",
	"panel_fill": "#fbfbfa",
	"caption_fill": "#ececea",   # caption strips + table header rows
	"fs_title": 18.0,            # header band title, bold
	"fs_caption": 8.5,           # panel captions, bold uppercase tracked
	"fs_body": 9.0,              # running text (steps, meta)
	"fs_table": 8.5,             # table cells, dims, scale bars, legends
	"fs_tb_label": 6.5,          # title-block cell labels, uppercase
	"fs_tb_value": 9.5,          # title-block cell values
	"margin": 28.0,              # fixed page margins, all sides
	"gutter": 16.0,              # between panels
	"pad": 12.0,                 # panel internal padding
	"cap_pad": 6.0,              # caption text padding inside its strip
	"header_h": 48.0,            # header band height
}
# Muted professional part palette (steel blue / clay / sage / graphite ...) —
# deliberately NOT matplotlib tab10.
PALETTE = [
	(0.361, 0.478, 0.596),  # steel blue
	(0.718, 0.514, 0.365),  # clay
	(0.494, 0.588, 0.463),  # sage
	(0.427, 0.443, 0.482),  # graphite
	(0.757, 0.647, 0.412),  # ochre
	(0.553, 0.471, 0.580),  # dusty plum
	(0.416, 0.573, 0.576),  # teal slate
	(0.612, 0.573, 0.529),  # warm gray
]
SINGLE_COLOR = (0.478, 0.569, 0.667)  # one-part steel blue
INK = STYLE["ink"]
INK_SOFT = STYLE["ink2"]
PAPER = STYLE["page_fill"]
PANEL_EDGE = STYLE["rule"]
FONT = STYLE["font"]
MAX_SHADED_TRIS = 300_000  # above this, shaded views subsample (declared)


# ------------------------------------------------------ chrome primitives --
def tracked(s):
	"""Panel-caption text treatment: UPPERCASE with thin-space tracking
	(U+2009 — present in DejaVu Sans, so never a missing glyph)."""
	return " ".join(str(s).upper())


@functools.lru_cache(maxsize=8192)
def _text_w_pt(s, fs, weight):
	tp = TextPath((0, 0), s, prop=FontProperties(family=STYLE["font"], size=fs, weight=weight))
	ext = tp.get_extents()
	return float(ext.x1 - ext.x0)


def text_w_px(s, fs, dpi, weight="normal"):
	"""MEASURED text width in px (TextPath metrics — deterministic, no
	renderer round-trip). The wrap/fit logic uses this, never a guess.

	Memoized on (string, size, weight): a greedy word-wrap re-measures the same
	prefixes thousands of times, and TextPath construction dominates the run.
	Pure function of its arguments, so the cache cannot change any output."""
	if not s:
		return 0.0
	return _text_w_pt(str(s), fs, weight) * dpi / 72.0


def fig_px(fig):
	"""(W, H) of the figure in output px."""
	w, h = fig.get_size_inches()
	return w * fig.dpi, h * fig.dpi


def px_rect(fig, x, y, w, h, fill=None, edge=None, lw=0.0, z=0):
	"""Axis-aligned rectangle at px coords (origin bottom-left)."""
	W, H = fig_px(fig)
	r = Rectangle((x / W, y / H), w / W, h / H, transform=fig.transFigure,
		facecolor=fill if fill else "none", edgecolor=edge if edge else "none",
		linewidth=lw, zorder=z)
	fig.add_artist(r)
	return r


def px_line(fig, x0, y0, x1, y1, color, lw, z=6, ls="-"):
	W, H = fig_px(fig)
	ln = Line2D([x0 / W, x1 / W], [y0 / H, y1 / H], transform=fig.transFigure,
		color=color, linewidth=lw, linestyle=ls, zorder=z)
	fig.add_artist(ln)
	return ln


def px_text(fig, x, y, s, fs, color, ha="left", va="baseline", weight="normal", z=8):
	W, H = fig_px(fig)
	return fig.text(x / W, y / H, s, fontsize=fs, family=STYLE["font"], color=color,
		ha=ha, va=va, fontweight=weight, zorder=z)


def caption_h(dpi):
	"""Caption strip height: caption text + 6 px padding above and below."""
	return round(STYLE["fs_caption"] * dpi / 72.0) + 2.0 * STYLE["cap_pad"]


def panel(fig, x, y, w, h, caption, dpi):
	"""ONE design-system panel: bordered body (0.8 pt, #fbfbfa fill) with a
	shaded caption strip attached to its top edge. Returns the inner content
	rect (x, y, w, h) in px after the 12 px internal padding — callers place
	content there, so nothing ever touches a border."""
	ch = caption_h(dpi)
	body_h = h - ch
	px_rect(fig, x, y, w, body_h, fill=STYLE["panel_fill"], z=-6)
	px_rect(fig, x, y + body_h, w, ch, fill=STYLE["caption_fill"], z=-6)
	px_rect(fig, x, y, w, body_h, edge=STYLE["border"], lw=STYLE["border_pt"], z=5)
	px_rect(fig, x, y + body_h, w, ch, edge=STYLE["border"], lw=STYLE["border_pt"], z=5)
	px_text(fig, x + STYLE["cap_pad"], y + body_h + ch / 2.0, tracked(caption),
		fs=STYLE["fs_caption"], color=STYLE["ink"], va="center", weight="bold", z=8)
	pad = STYLE["pad"]
	return (x + pad, y + pad, w - 2.0 * pad, body_h - 2.0 * pad)


def page_frame(fig):
	"""Thin drawing frame 12 px inside the page edge (the 28 px content
	margins keep every panel 16 px clear of it)."""
	W, H = fig_px(fig)
	px_rect(fig, 12.0, 12.0, W - 24.0, H - 24.0, edge=STYLE["border"], lw=STYLE["border_pt"], z=5)


def header_band(fig, title, meta):
	"""Header band: title (18 bold) left, quiet meta (9, secondary) right,
	0.8 pt rule underneath. Returns the rule's y (the band bottom) in px."""
	W, H = fig_px(fig)
	m = STYLE["margin"]
	y_rule = H - m - STYLE["header_h"]
	cy = y_rule + STYLE["header_h"] / 2.0
	px_text(fig, m, cy, title, fs=STYLE["fs_title"], color=STYLE["ink"], va="center", weight="bold")
	px_text(fig, W - m, cy, meta, fs=STYLE["fs_body"], color=STYLE["ink2"], ha="right", va="center")
	px_line(fig, m, y_rule, W - m, y_rule, color=STYLE["border"], lw=STYLE["border_pt"])
	return y_rule


def draw_balloon(fig, x, y, label, r=9.0, face="white", edge=None, text_color=None, lw=1.1, z=30):
	"""Balloon/chip marker: a DRAWN circle patch + a plain bold digit centered
	in it. NEVER a Unicode circled-digit glyph (0x2460..) — those are missing
	from some font fallback paths and render as '?'; plain digits exist in
	every font. r is the radius in px; the digit is sized to the circle."""
	W, H = fig_px(fig)
	edge = edge if edge else STYLE["border"]
	text_color = text_color if text_color else STYLE["ink"]
	fig.add_artist(Ellipse((x / W, y / H), 2.0 * r / W, 2.0 * r / H,
		transform=fig.transFigure, facecolor=face, edgecolor=edge, linewidth=lw, zorder=z))
	s = str(label)
	fs = (1.30 if len(s) == 1 else 1.00) * r * 72.0 / fig.dpi
	fig.text(x / W, (y - 0.08 * r) / H, s, fontsize=fs, family=STYLE["font"],
		color=text_color, fontweight="bold", ha="center", va="center", zorder=z + 1)


# -------------------------------------------------------------- 3D fitting --
def set_limits3d(ax, center, r):
	"""Cubic world window center±r with an aspect-true box."""
	ax.set_xlim(center[0] - r, center[0] + r)
	ax.set_ylim(center[1] - r, center[1] + r)
	ax.set_zlim(center[2] - r, center[2] + r)
	ax.set_box_aspect((1, 1, 1))


def project_px(ax, pts):
	"""World points (n,3) -> display px (n,2). Static under Agg (fixed axes
	position + limits), so callable before savefig."""
	pts = np.asarray(pts, dtype=np.float64)
	xs, ys, _ = proj3d.proj_transform(pts[:, 0], pts[:, 1], pts[:, 2], ax.get_proj())
	return ax.transData.transform(np.column_stack([np.atleast_1d(xs), np.atleast_1d(ys)]))


def fit_view(ax, pts, elev, azim, rect_px, fill=0.86):
	"""Scale + center an ortho 3D view so pts' projection fills `fill` of the
	limiting dimension of rect_px (x, y, w, h in display px), centered — the
	dead-space killer. Ortho projection is linear, so the measure/correct
	loop converges exactly. Returns px-per-world-mm."""
	pts = np.asarray(pts, dtype=np.float64)
	lo, hi = pts.min(axis=0), pts.max(axis=0)
	center = (lo + hi) / 2.0
	r = max(float((hi - lo).max()) / 2.0, 1e-6) * 1.05
	cam = camera_dir(elev, azim)
	u = np.cross(cam, [0.0, 0.0, 1.0])
	u = np.array([1.0, 0.0, 0.0]) if np.linalg.norm(u) < 1e-9 else u / np.linalg.norm(u)
	v = np.cross(u, cam)
	v /= np.linalg.norm(v)
	for _ in range(3):
		set_limits3d(ax, center, r)
		P = project_px(ax, pts)
		plo, phi = P.min(axis=0), P.max(axis=0)
		ext = np.maximum(phi - plo, 1e-9)
		s = min(rect_px[2] * fill / ext[0], rect_px[3] * fill / ext[1])
		c_px = project_px(ax, center[None, :])[0]
		ku = project_px(ax, (center + u)[None, :])[0] - c_px
		kv = project_px(ax, (center + v)[None, :])[0] - c_px
		want = np.array([rect_px[0] + rect_px[2] / 2.0, rect_px[1] + rect_px[3] / 2.0])
		ab = np.linalg.solve(np.column_stack([ku, kv]), want - (plo + phi) / 2.0)
		center = center - ab[0] * u - ab[1] * v
		r = r / s
	set_limits3d(ax, center, r)
	c_px = project_px(ax, center[None, :])[0]
	return float(np.linalg.norm(project_px(ax, (center + u)[None, :])[0] - c_px))


# ---------------------------------------------------------------- geometry --
def tri_normals(tris):
	"""Unit normals recomputed from vertex winding (robust to STL files whose
	stored normals are zero/garbage). Degenerate triangles get a +Z stand-in."""
	n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
	length = np.linalg.norm(n, axis=1)
	bad = length < 1e-30
	n[bad] = [0.0, 0.0, 1.0]
	length[bad] = 1.0
	return n / length[:, None]


def rotation_to_z(build_dir):
	"""Rotation matrix aligning unit vector build_dir with +Z (Rodrigues)."""
	b = np.asarray(build_dir, dtype=np.float64)
	norm = np.linalg.norm(b)
	if norm < 1e-12:
		raise ValueError(f"build_dir {list(build_dir)} is a zero vector")
	b = b / norm
	z = np.array([0.0, 0.0, 1.0])
	c = float(b @ z)
	if c > 1.0 - 1e-12:
		return np.eye(3)
	if c < -1.0 + 1e-12:  # anti-parallel: flip about X
		return np.diag([1.0, -1.0, -1.0])
	axis = np.cross(b, z)
	axis /= np.linalg.norm(axis)
	s = np.sqrt(max(0.0, 1.0 - c * c))
	k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
	return np.eye(3) + s * k + (1.0 - c) * (k @ k)


def section_segments(tris, axis, value):
	"""TRUE cross-section: the 2D intersection segments of the triangles with
	the plane <axis>=value, vectorized. Returns (k,2,2) in the two remaining
	coordinates. The plane is nudged off exact vertex hits so every crossing
	triangle yields exactly two edge intersections."""
	v = tris[:, :, axis]
	span = float(v.max() - v.min()) if len(v) else 1.0
	if np.any(v == value):
		value = value + 1e-9 * (span + 1.0)
	crossing = (v.min(axis=1) < value) & (v.max(axis=1) > value)
	t = tris[crossing]
	if len(t) == 0:
		return np.zeros((0, 2, 2))
	pts = np.full((len(t), 3, 3), np.nan)
	for i in range(3):
		a, b = t[:, i, :], t[:, (i + 1) % 3, :]
		da, db = a[:, axis] - value, b[:, axis] - value
		hit = (da * db) < 0.0
		s = da[hit] / (da[hit] - db[hit])
		pts[hit, i, :] = a[hit] + s[:, None] * (b[hit] - a[hit])
	hitmask = ~np.isnan(pts[:, :, 0])
	two = hitmask.sum(axis=1) == 2
	pt, hm = pts[two], hitmask[two]
	order = np.argsort(~hm, axis=1)[:, :2]  # the two hit slots, in edge order
	seg3 = np.take_along_axis(pt, order[:, :, None], axis=1)
	other = [i for i in range(3) if i != axis]
	return seg3[:, :, other]


def hatch_spans(segs, spacing, angle_deg=45.0):
	"""Scanline-parity fill of a closed 2D cut: returns (m,2,2) hatch segments
	covering the MATERIAL region (adapted from air_topology_audit.voxelize's
	per-slice parity logic, run along rotated scanlines so the fill reads as a
	classic 45-degree engineering hatch). Odd-parity scanline residue (open
	curves / numeric grazing) is dropped, never guessed into material."""
	if len(segs) == 0:
		return np.zeros((0, 2, 2))
	th = np.radians(angle_deg)
	c, s = np.cos(th), np.sin(th)
	rot = np.array([[c, s], [-s, c]])       # world -> hatch frame (rotate by -angle)
	back = np.array([[c, -s], [s, c]])      # hatch frame -> world
	p = segs.reshape(-1, 2) @ rot.T
	q = p.reshape(-1, 2, 2)
	y1, y2 = q[:, 0, 1], q[:, 1, 1]
	ylo, yhi = float(p[:, 1].min()), float(p[:, 1].max())
	spans = []
	n = max(1, int(np.ceil((yhi - ylo) / spacing)))
	for i in range(n):
		y = ylo + (i + 0.5) * spacing
		if y >= yhi:
			break
		near = np.minimum(np.abs(y1 - y), np.abs(y2 - y))
		if near.min() < 1e-9 * (abs(yhi - ylo) + 1.0):  # grazing a vertex: nudge
			y += spacing * 1e-3
		cross = (y1 - y) * (y2 - y) < 0.0
		if not cross.any():
			continue
		cs = q[cross]
		tp = (y - cs[:, 0, 1]) / (cs[:, 1, 1] - cs[:, 0, 1])
		xs = np.sort(cs[:, 0, 0] + tp * (cs[:, 1, 0] - cs[:, 0, 0]))
		for a, b in zip(xs[0::2], xs[1::2]):  # parity pairs = material spans
			spans.append([[a, y], [b, y]])
	if not spans:
		return np.zeros((0, 2, 2))
	sp = np.asarray(spans)
	return sp @ back.T


def camera_dir(elev_deg, azim_deg):
	"""Unit vector from scene center toward the camera for view_init angles."""
	el, az = np.radians(elev_deg), np.radians(azim_deg)
	return np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])


def shade_colors(part_tris, part_colors, light):
	"""Two-sided facet shading: 0.35 + 0.65*|n.L| on winding-recomputed
	normals — coplanar triangles shade identically whatever their winding, so
	flat faces render as flat faces (the streak fix)."""
	return np.concatenate([
		(0.35 + 0.65 * np.abs(tri_normals(t) @ light))[:, None] * np.asarray(c)[None, :]
		for t, c in zip(part_tris, part_colors)
	])


def shaded_view(ax, part_tris, part_colors, elev, azim, center, radius, extra_polys=None):
	"""One painter-shaded orthographic 3D panel. All parts are merged into a
	single Poly3DCollection so matplotlib's z-sort resolves occlusion BETWEEN
	overlay parts, not just within each. Headlight-with-offset lighting keeps
	straight-on ortho views legible (a pure world light flattens them).
	Captions live in the panel chrome, not on the axes.
	extra_polys: [(polygon(n,3), rgba)] background geometry (bed, shadow)
	drawn in its own collection BEHIND the part."""
	cam = camera_dir(elev, azim)
	light = cam + np.array([0.35, 0.30, 0.55])
	light /= np.linalg.norm(light)
	tris = np.concatenate(part_tris)
	cols = shade_colors(part_tris, part_colors, light)
	order = np.argsort(tris.mean(axis=1) @ cam)  # far-to-near painter pre-sort
	if extra_polys:
		bg = Poly3DCollection([p for p, _ in extra_polys], facecolors=[c for _, c in extra_polys],
			edgecolors="none", rasterized=True, zsort="max")
		bg.set_sort_zpos(-1e9)  # always behind the part collection
		ax.add_collection3d(bg)
	pc = Poly3DCollection(list(tris[order]), facecolors=list(np.clip(cols[order], 0, 1)),
		edgecolors="none", rasterized=True)
	ax.add_collection3d(pc)
	set_limits3d(ax, center, radius)
	ax.set_proj_type("ortho")
	ax.view_init(elev=elev, azim=azim)
	ax.set_axis_off()
	ax.set_facecolor("none")  # panels sit on the panel fill, no white boxes


def proj_pt(ax, p):
	"""World point -> 2D coords in the 3D axes' projected data space (the
	documented annotate-a-3D-point trick). Call AFTER limits/view are set."""
	x, y, _ = proj3d.proj_transform(p[0], p[1], p[2], ax.get_proj())
	return np.array([float(x), float(y)])


def dim_annotate(ax, a3, b3, off_dir, off_frac, label, scale_ref, fs=STYLE["fs_table"]):
	"""Engineering <-> dimension between world points a3,b3 on a 3D ortho
	panel: extension lines + double-arrow dimension line + mm text, all drawn
	in projected-2D space. off_dir: 2D screen direction (unit-ish) to push the
	dimension line off the geometry; off_frac: offset as a fraction of
	scale_ref (the projected scene half-diagonal)."""
	a2, b2 = proj_pt(ax, np.asarray(a3, float)), proj_pt(ax, np.asarray(b3, float))
	off = np.asarray(off_dir, float) * (off_frac * scale_ref)
	A, B = a2 + off, b2 + off
	lw = 0.7
	for p0, p1 in ((a2, A), (b2, B)):  # extension lines, slightly past the dim line
		p2 = p1 + off / (np.linalg.norm(off) + 1e-12) * (0.015 * scale_ref)
		ax.annotate("", xy=tuple(p2), xytext=tuple(p0), xycoords="data", textcoords="data",
			annotation_clip=False,
			arrowprops={"arrowstyle": "-", "lw": lw * 0.75, "color": STYLE["ink2"], "shrinkA": 2.5, "shrinkB": 0})
	ax.annotate("", xy=tuple(A), xytext=tuple(B), xycoords="data", textcoords="data",
		annotation_clip=False,
		arrowprops={"arrowstyle": "<|-|>", "mutation_scale": 7, "lw": lw, "color": STYLE["ink"], "shrinkA": 0, "shrinkB": 0})
	mid = (A + B) / 2.0 + off / (np.linalg.norm(off) + 1e-12) * (0.035 * scale_ref)
	horizontal = abs((B - A)[0]) >= abs((B - A)[1])
	ax.text2D(mid[0], mid[1], label, transform=ax.transData, fontsize=fs, color=STYLE["ink"],
		family=STYLE["font"], ha="center", va="center", rotation=0 if horizontal else 90, clip_on=False)


# ------------------------------------------------------------- path roots --
PATH_ROOT_MAX_ASCENT = 6


def resolve_job_root(rel_paths, job_dir, base_dir, notes):
	"""Pick the ONE directory every relative input in this job is relative to.

	gripper F10 (path-root class, T4): job files carry relative paths like
	`parts/palm.stl`, but `kernel-api run` resolves program-relative paths
	against `--out-dir` while this tool resolved them against `os.getcwd()` —
	the one directory the README does not tell you to stand in, so the
	documented repo-root command line could not rebuild any render.

	Candidates, in order: the explicit `base_dir` job key; the CWD (today's
	semantics, so every existing invocation is unchanged); then the job file's
	directory and its ancestors, nearest first, at most PATH_ROOT_MAX_ASCENT up
	(a job at `<part>/programs/jobs/x.json` naming `parts/y.stl` means `<part>`).
	A root qualifies only when EVERY relative input exists under it, so the
	answer is a fact about the job, not about one lucky path.

	This is a search, so it says what it found: the winning root's KIND lands in
	the receipt `notes` (kind, never an absolute path — the receipt stays
	byte-comparable across checkouts). Two ancestors that both qualify are
	AMBIGUOUS and are refused, not silently ranked; none is refused with the
	list of roots tried. Nothing is ever guessed at."""
	if not rel_paths:
		return None, []
	cands = []
	if base_dir:
		cands.append(("base_dir", base_dir))
	cands.append(("cwd", os.getcwd()))
	if job_dir:
		d = os.path.abspath(job_dir)
		for i in range(PATH_ROOT_MAX_ASCENT):
			cands.append((f"job file's directory{'' if i == 0 else f' + {i} up'}", d))
			parent = os.path.dirname(d)
			if parent == d:
				break
			d = parent

	def qualifies(root):
		return all(os.path.exists(os.path.join(root, p)) for p in rel_paths)

	for kind, root in cands[:2 if base_dir else 1]:   # base_dir / cwd: authoritative, no ambiguity
		if qualifies(root):
			return root, []
	hits = [(kind, root) for kind, root in cands if qualifies(root)]
	if not hits:
		raise FileNotFoundError(
			f"none of {rel_paths} resolve under any job path root (tried: "
			+ ", ".join(k for k, _ in cands) + ") — set 'base_dir' in the job")
	roots = {os.path.realpath(r) for _, r in hits}
	if len(roots) > 1:
		raise ValueError(
			f"ambiguous path root: {rel_paths} resolve under {len(roots)} different "
			f"directories ({', '.join(k for k, _ in hits)}) — set 'base_dir' in the job")
	kind, root = hits[0]
	notes.append(f"relative paths resolved against the {kind}, not the CWD")
	return root, notes


# ------------------------------------------------------------------ render --
def render(job, job_dir=None):
	paths = job["stls"] if "stls" in job else [job["stl"]]
	if not isinstance(paths, list) or not paths:
		raise ValueError("'stls' must be a non-empty list of STL paths")
	base_dir = job.get("base_dir")
	path_notes = []
	rel = [p for p in paths if not os.path.isabs(p)]
	root, _ = resolve_job_root(rel, job_dir, base_dir, path_notes)
	paths = [p if os.path.isabs(p) else os.path.join(root, p) for p in paths]
	# The OUTPUT lands under the same root the inputs came from (identical to
	# today whenever that root is the CWD, which is every job that worked before).
	out = job["out"]
	if not os.path.isabs(out):
		out = os.path.join(base_dir or root or os.getcwd(), out)
	max_px = int(job.get("max_px", 1600))
	if max_px < 800:
		raise ValueError(f"max_px {max_px} is too small for the 12-panel grid (min 800)")
	build_dir = job.get("build_dir", [0.0, 0.0, 1.0])

	parts = [load_stl(p) for p in paths]
	names = [os.path.splitext(os.path.basename(p))[0] for p in paths]
	overlay = len(parts) > 1
	colors = PALETTE[: len(parts)] if overlay else [SINGLE_COLOR]
	if len(parts) > len(PALETTE):
		raise ValueError(f"overlay supports at most {len(PALETTE)} STLs (got {len(parts)})")

	# Sections always use the EXACT triangles; shaded views subsample above
	# the cap (declared in the receipt and on the sheet, never silent).
	total_tris = int(sum(len(t) for t in parts))
	decimated = total_tris > MAX_SHADED_TRIS
	shaded_parts = parts
	if decimated:
		keep = MAX_SHADED_TRIS / total_tris
		rng = np.random.default_rng(0)  # deterministic sheet for a given input
		shaded_parts = [t[rng.random(len(t)) < keep] for t in parts]
	shown_tris = int(sum(len(t) for t in shaded_parts))

	allv = np.concatenate(parts).reshape(-1, 3)
	lo, hi = allv.min(axis=0), allv.max(axis=0)
	center, extent = (lo + hi) / 2.0, hi - lo
	radius = float(extent.max()) / 2.0 * 1.14 + 1e-9  # head-room for dimensions

	sec = job.get("sections", {}) or {}
	cuts = [float(sec.get(k, center[i])) for i, k in enumerate(("x", "y", "z"))]

	# Bed view geometry: rotate build_dir -> +Z, rest on z=0, draw a bed quad
	# plus a flattened-silhouette ground shadow at 10% alpha.
	rot = rotation_to_z(build_dir)
	bed_parts = [t @ rot.T for t in shaded_parts]
	bed_v = np.concatenate(bed_parts).reshape(-1, 3)
	drop = bed_v[:, 2].min()
	bed_parts = [t - [0, 0, drop] for t in bed_parts]
	bed_v = np.concatenate(bed_parts).reshape(-1, 3)
	blo, bhi = bed_v.min(axis=0), bed_v.max(axis=0)
	bcenter, bradius = (blo + bhi) / 2.0, float((bhi - blo).max()) / 2.0 * 1.15 + 1e-9
	bcenter[2] = (bhi[2] - 0.0) / 2.0  # frame from the bed up
	m = 0.18 * (bhi - blo)[:2].max() + 1e-9
	bed_quad = np.array(
		[[blo[0] - m, blo[1] - m, 0], [bhi[0] + m, blo[1] - m, 0], [bhi[0] + m, bhi[1] + m, 0], [blo[0] - m, bhi[1] + m, 0]]
	)
	shadow = np.concatenate(bed_parts).copy()
	shadow[:, :, 2] = 0.002 * bradius  # flattened silhouette just above the bed
	extra = [(bed_quad, (0.855, 0.865, 0.878, 1.0))]
	extra += [(tri, (0.16, 0.18, 0.21, 0.10)) for tri in shadow]

	# ---- page grid (all px): margins 28, header band, 4x3 panels, 16 px gutters
	fig_w, fig_h = 16.0, 12.0
	dpi = max_px / fig_w  # long edge == max_px exactly (no bbox_inches='tight')
	W, H = max_px, max_px * fig_h / fig_w
	lw = max(0.45, 0.7 * dpi / 100.0)
	fig = plt.figure(figsize=(fig_w, fig_h), dpi=dpi)
	fig.patch.set_facecolor(STYLE["page_fill"])

	mg, gt = STYLE["margin"], STYLE["gutter"]
	page_frame(fig)
	name = " + ".join(names) if overlay else names[0]
	meta = (f"bbox {extent[0]:.1f} × {extent[1]:.1f} × {extent[2]:.1f} mm · {total_tris} tris · units mm"
		+ (f" · {job['date']}" if job.get("date") else "") + " · cadcode vision sheet")
	if decimated:
		meta = f"shaded views subsampled to {shown_tris} tris (sections exact) · " + meta
	band_y = header_band(fig, name, meta)
	legend_h = 0.0
	if overlay:
		# Swatch legend. din_rail F4: it was drawn INSIDE the header band at a
		# fixed x with NO collision test against the right-aligned meta string,
		# so a long title + 5 parts overprinted the meta line. The presentation
		# contract says "nothing floats, nothing touches" — so MEASURE the free
		# span first and, when the legend does not fit, give it its own full-width
		# strip under the rule (the panel grid gives up exactly that height).
		# A legend that fits is drawn exactly where it always was: identical PNG.
		item_w = [17.0 + text_w_px(nm, STYLE["fs_table"], dpi) + 18.0 for nm in names]
		sx0 = mg + text_w_px(name, STYLE["fs_title"], dpi, weight="bold") + 30.0
		meta_left = W - mg - text_w_px(meta, STYLE["fs_body"], dpi)
		row_px = STYLE["fs_table"] * dpi / 72.0 * 1.9
		if sx0 + sum(item_w) <= meta_left - 12.0:
			sx = sx0
			cy = band_y + STYLE["header_h"] / 2.0
			for nm, c, w in zip(names, colors, item_w):
				px_rect(fig, sx, cy - 4.0, 12.0, 8.0, fill=c, edge=STYLE["border"], lw=0.4, z=8)
				px_text(fig, sx + 17.0, cy, nm, fs=STYLE["fs_table"], color=STYLE["ink"], va="center")
				sx += w
		else:
			rows, cur = [[]], mg
			for nm, c, w in zip(names, colors, item_w):
				if cur + w > W - mg and rows[-1]:
					rows.append([])
					cur = mg
				rows[-1].append((nm, c, w))
				cur += w
			legend_h = len(rows) * row_px + 6.0
			for r, row in enumerate(rows):
				cy = band_y - gt - (r + 0.5) * row_px
				sx = mg
				for nm, c, w in row:
					px_rect(fig, sx, cy - 4.0, 12.0, 8.0, fill=c, edge=STYLE["border"], lw=0.4, z=8)
					px_text(fig, sx + 17.0, cy, nm, fs=STYLE["fs_table"], color=STYLE["ink"], va="center")
					sx += w
			path_notes.append(f"overlay legend moved out of the header band into its own "
			                  f"{len(rows)}-row strip (it would have collided with the meta line)")

	grid_top = band_y - gt - legend_h
	col_w = (W - 2.0 * mg - 3.0 * gt) / 4.0
	row_h = (grid_top - mg - 2.0 * gt) / 3.0
	if row_h < 120.0:
		raise ValueError(f"max_px {max_px} leaves panel rows only {row_h:.0f} px tall — too small to be legible")

	def cell(i):
		"""Panel slot i (0..11, row-major from top-left) -> (x, y, w, h) px."""
		r, c = divmod(i, 4)
		return (mg + c * (col_w + gt), grid_top - (r + 1) * row_h - r * gt, col_w, row_h)

	w_mm, d_mm, h_mm = float(extent[0]), float(extent[1]), float(extent[2])
	orthos = [
		("TOP (+Z)", 90, -90), ("BOTTOM (-Z)", -90, -90), ("FRONT (-Y)", 0, -90), ("BACK (+Y)", 0, 90),
		("LEFT (-X)", 0, 180), ("RIGHT (+X)", 0, 0), ("ISO AZ+45 EL30", 30, 45), ("ISO AZ-45 EL30", 30, -45),
	]
	axes3d = []
	for i, (title, elev, azim) in enumerate(orthos):
		x, y, w, h = cell(i)
		ix, iy, iw, ih = panel(fig, x, y, w, h, title, dpi)
		ax = fig.add_axes([ix / W, iy / H, iw / W, ih / H], projection="3d")
		shaded_view(ax, shaded_parts, colors, elev, azim, center, radius)
		axes3d.append(ax)

	# Overall <-> dimensions on the top + front orthos (panel 1: W/D, panel 3: W/H).
	ax_top = axes3d[0]
	sref = np.linalg.norm(proj_pt(ax_top, hi) - proj_pt(ax_top, lo)) / 2.0 + 1e-12
	dim_annotate(ax_top, [lo[0], lo[1], hi[2]], [hi[0], lo[1], hi[2]], [0, -1], 0.10, f"{w_mm:.1f} mm", sref)
	dim_annotate(ax_top, [hi[0], lo[1], hi[2]], [hi[0], hi[1], hi[2]], [1, 0], 0.10, f"{d_mm:.1f} mm", sref)
	ax_fr = axes3d[2]
	sref = np.linalg.norm(proj_pt(ax_fr, hi) - proj_pt(ax_fr, lo)) / 2.0 + 1e-12
	dim_annotate(ax_fr, [lo[0], lo[1], lo[2]], [hi[0], lo[1], lo[2]], [0, -1], 0.10, f"{w_mm:.1f} mm", sref)
	dim_annotate(ax_fr, [hi[0], lo[1], lo[2]], [hi[0], lo[1], hi[2]], [1, 0], 0.10, f"{h_mm:.1f} mm", sref)

	# --- Feature dimension callouts (audit 2026-07-16: FRICTION #21's drawing
	# half). job["dimensions"]: [{kind: "linear", a, b, label?, view?, offset?} |
	# {kind: "diameter", center, axis, radius, label?, view?}] — values usually
	# come from `measure_dimension` receipts or tools/dim_suggest.py, so the
	# callout numbers are ANALYTIC, not mesh-derived. Views: top/bottom/front/
	# back/left/right (auto-picked for least foreshortening when absent).
	dims_drawn, dims_report = 0, []
	dim_specs = job.get("dimensions") or []
	if dim_specs:
		ortho_keys = {"top": 0, "bottom": 1, "front": 2, "back": 3, "left": 4, "right": 5}
		look = {k: camera_dir(orthos[i][1], orthos[i][2]) for k, i in ortho_keys.items()}
		scene_c2 = {k: proj_pt(axes3d[i], center) for k, i in ortho_keys.items()}
		srefs = {k: np.linalg.norm(proj_pt(axes3d[i], hi) - proj_pt(axes3d[i], lo)) / 2.0 + 1e-12 for k, i in ortho_keys.items()}
		# Anti-collision state per view: concentric Ø leaders fan out to distinct
		# screen angles; parallel linear dims stagger their offset rings.
		fan_angles = [200.0, 160.0, 20.0, 340.0, 120.0, 60.0, 240.0, 300.0]
		dia_count = {k: 0 for k in ortho_keys}
		lin_count = {k: 0 for k in ortho_keys}
		for spec in dim_specs:
			kind = spec.get("kind", "linear")
			view = spec.get("view")
			if view is not None and view not in ortho_keys:
				raise ValueError(f"dimension view must be one of {sorted(ortho_keys)}, got {view!r}")
			if kind == "linear":
				a3, b3 = np.asarray(spec["a"], float), np.asarray(spec["b"], float)
				if view is None:  # least-foreshortened ortho
					view = max(ortho_keys, key=lambda k: np.linalg.norm(proj_pt(axes3d[ortho_keys[k]], b3) - proj_pt(axes3d[ortho_keys[k]], a3)))
				ax = axes3d[ortho_keys[view]]
				a2, b2 = proj_pt(ax, a3), proj_pt(ax, b3)
				seg = b2 - a2
				if np.linalg.norm(seg) < 1e-9:
					dims_report.append({"label": spec.get("label"), "skipped": "zero projected length", "view": view})
					continue
				perp = np.array([-seg[1], seg[0]]) / np.linalg.norm(seg)
				mid = (a2 + b2) / 2.0
				if np.dot(mid - scene_c2[view], perp) < 0.0:
					perp = -perp  # push the dim line AWAY from the scene, never across it
				label = spec.get("label") or f"{np.linalg.norm(b3 - a3):.2f} mm"
				off = float(spec.get("offset", 0.16)) + 0.07 * lin_count[view]
				lin_count[view] += 1
				dim_annotate(ax, a3, b3, perp, off, label, srefs[view])
				dims_drawn += 1
				dims_report.append({"label": label, "view": view, "kind": "linear"})
			elif kind == "diameter":
				c3 = np.asarray(spec["center"], float)
				axis = np.asarray(spec.get("axis", [0, 0, 1]), float)
				axis = axis / (np.linalg.norm(axis) + 1e-12)
				r = float(spec["radius"])
				if view is None:  # most face-on ortho for the circle
					view = max(ortho_keys, key=lambda k: abs(float(np.dot(look[k], axis))))
				ax = axes3d[ortho_keys[view]]
				# Concentric-set legibility: each Ø leader in a view takes the next
				# angle of a fixed fan, so bore/boss families never stack their text.
				theta = np.radians(fan_angles[dia_count[view] % len(fan_angles)])
				ring = 0.14 + 0.045 * (dia_count[view] // 2)  # every 2nd leader steps outward
				dia_count[view] += 1
				target = np.array([np.cos(theta), np.sin(theta)])
				u = np.cross(axis, [0.0, 0.0, 1.0])
				if np.linalg.norm(u) < 1e-6:
					u = np.cross(axis, [0.0, 1.0, 0.0])
				u /= np.linalg.norm(u)
				v = np.cross(axis, u)
				c2 = proj_pt(ax, c3)
				# Sample the circle; take the rim point whose screen direction from
				# the circle's own center best matches the fan angle.
				phis = np.linspace(0.0, 2.0 * np.pi, 24, endpoint=False)
				samples = [c3 + r * (u * np.cos(ph) + v * np.sin(ph)) for ph in phis]
				def screen_dir(pt3):
					d = proj_pt(ax, pt3) - c2
					n = np.linalg.norm(d)
					return d / n if n > 1e-12 else np.array([1.0, 0.0])
				e3 = max(samples, key=lambda pt3: float(np.dot(screen_dir(pt3), target)))
				e2 = proj_pt(ax, e3)
				d2 = screen_dir(e3)
				label = spec.get("label") or f"Ø{2.0 * r:.2f}"
				t2 = e2 + d2 * ring * srefs[view]
				ax.annotate("", xy=tuple(e2), xytext=tuple(t2), xycoords="data", textcoords="data",
					annotation_clip=False,
					arrowprops={"arrowstyle": "-|>", "mutation_scale": 7, "lw": 0.7, "color": STYLE["ink"], "shrinkA": 0, "shrinkB": 0})
				ax.text2D(t2[0] + d2[0] * 0.02 * srefs[view], t2[1] + d2[1] * 0.02 * srefs[view], label,
					transform=ax.transData, fontsize=STYLE["fs_table"], color=STYLE["ink"], family=STYLE["font"],
					ha="left" if d2[0] >= 0 else "right", va="center", clip_on=False)
				dims_drawn += 1
				dims_report.append({"label": label, "view": view, "kind": "diameter"})
			else:
				raise ValueError(f"dimension kind must be 'linear' or 'diameter', got {kind!r}")

	x, y, w, h = cell(8)
	ix, iy, iw, ih = panel(fig, x, y, w, h, "BED VIEW (PRINT ORIENTATION)", dpi)
	ax = fig.add_axes([ix / W, iy / H, iw / W, ih / H], projection="3d")
	shaded_view(ax, bed_parts, colors, 8, -60, bcenter, bradius, extra_polys=extra)

	# Sections: 45-degree parity hatch (material) under crisp cut outlines.
	hatch_pitch = 2.0 * radius / 46.0
	for j, (axis, label) in enumerate([(0, "x"), (1, "y"), (2, "z")]):
		x, y, w, h = cell(9 + j)
		ix, iy, iw, ih = panel(fig, x, y, w, h, f"SECTION {label}={cuts[axis]:.2f}", dpi)
		ax = fig.add_axes([ix / W, iy / H, iw / W, ih / H])
		other = [i for i in range(3) if i != axis]
		for t, c in zip(parts, colors):
			segs = section_segments(t, axis, cuts[axis])
			if not len(segs):
				continue
			fill = hatch_spans(segs, hatch_pitch)
			if len(fill):
				ax.add_collection(LineCollection(fill, colors=[c], linewidths=lw * 0.5, alpha=0.45))
			dark = tuple(0.55 * ch for ch in c)
			ax.add_collection(LineCollection(segs, colors=[dark], linewidths=lw * 1.1))
		ax.set_xlim(center[other[0]] - radius, center[other[0]] + radius)
		ax.set_ylim(center[other[1]] - radius, center[other[1]] + radius)
		ax.set_aspect("equal")
		ax.set_axis_off()
		ax.set_facecolor("none")

	os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
	fig.savefig(out, dpi=dpi, facecolor=STYLE["page_fill"])
	plt.close(fig)

	with open(out, "rb") as f:
		head = f.read(24)
	w, h = int.from_bytes(head[16:20], "big"), int.from_bytes(head[20:24], "big")
	receipt = {"ok": True, "out": os.path.abspath(out), "panels": 12, "px": [w, h], "triangles": total_tris, "parts": len(parts), "bytes": os.path.getsize(out)}
	if dim_specs:
		receipt["dimensions"] = {"drawn": dims_drawn, "requested": len(dim_specs), "callouts": dims_report}
	if decimated:
		receipt["decimated"] = True
		receipt["shown_triangles"] = shown_tris
	if path_notes:
		receipt["notes"] = path_notes
	return receipt


def main():
	if len(sys.argv) < 2 or sys.argv[1] in ("-h", "--help"):
		print(__doc__)
		return 0
	if len(sys.argv) != 2:
		print(json.dumps({"ok": False, "error": "usage: render_sheet.py job.json"}))
		return 1
	try:
		with open(sys.argv[1]) as f:
			job = json.load(f)
		if "out" not in job or ("stl" not in job and "stls" not in job):
			raise ValueError("job needs 'out' and either 'stl' or 'stls'")
		receipt = render(job, job_dir=os.path.dirname(os.path.abspath(sys.argv[1])))
	except Exception as e:  # noqa: BLE001 — the receipt IS the error channel
		print(json.dumps({"ok": False, "error": f"{type(e).__name__}: {e}"}))
		return 1
	print(json.dumps(receipt))
	return 0


if __name__ == "__main__":
	sys.exit(main())
