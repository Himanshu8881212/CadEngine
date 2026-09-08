#!/usr/bin/env python3
"""build_plates.py — pack a campaign's print files onto build plates, ONE slicer profile per plate.

The deliverable a user actually loads. For every print-setting GROUP the campaign
declares, the tool packs that group's parts onto as few plates as they allow —
every part instance placed exactly once, in its exported PRINT POSE (rotation
about the build direction Z only, never about X or Y), nested on the part's REAL
footprint (a conservative raster of its XY projection, not its bounding box),
with a guaranteed gap between parts and to the bed edge. The user selects the
plate's profile in the slicer, imports the plate file and clicks slice. Nothing
is left to arrange.

Usage:  python3 tools/publish/build_plates.py job.json [--out receipt.json]

Job JSON (argv[1]) — all lengths mm:
    out_dir*        directory for the plate files (the campaign's plates/)
    bed*            {x, y, z} printable volume. A printer fact, so it has NO default.
    groups*         [{name*, profile*, parts*}] — one entry per slicer profile:
                      name     short token used in file names ("W", "S", "tpu")
                      profile  {material, nozzle_mm, layer_mm, perimeters, infill,
                                top_bottom, supports, notes, ...} — free-form
                                key/value, at least one key, written verbatim
                                into PRINT_PLATES.md (the user reads it to pick
                                the slicer profile)
                      parts    [{name*, stl*, qty (default 1)}] — a part name
                                belongs to exactly ONE group
                      one_part_per_plate  (default false) every instance on its OWN
                                plate: spiral/vase slicer modes take one object per
                                plate (Bambu Studio, PrusaSlicer); the packing still
                                centres and checks each part against the bed
    spacing_mm      gap between parts (default 5)
    edge_mm         margin to the bed edge (default = spacing_mm)
    cell_mm         raster cell of the footprint masks (default 1.0). The
                    guaranteed gap is spacing_mm rounded UP to whole cells.
    rotations_deg   candidate rotations about Z tried per part
                    (default [0, 90, 180, 270]; e.g. list(range(0, 360, 15)))
    exclusion_zones [[x0, y0, x1, y1], ...] plate regions no part may cover
                    (a prime-line corner). With zones the arrangement is NOT
                    re-centred, because absolute positions then matter — an STL
                    import re-centres, so hand the user the .3mf, which keeps them.
    emit_3mf        also write <group>_plate_<n>.3mf with one named object per
                    instance (default true)
    date            string stamped on the layout sheet — NEVER the clock
    title           string for the layout sheet and the PRINT_PLATES.md header

Outputs in out_dir:
    <group>_plate_<n>.stl   merged binary STL: bed at z = 0, one body per instance,
                            arrangement centred on the bed (so a slicer that
                            re-centres an imported STL shows the same picture)
    <group>_plate_<n>.3mf   the same arrangement as separately NAMED objects,
                            positions kept by the slicer (Bambu/Orca/Prusa import
                            a plain 3MF as geometry + placement)
    plates_layout.png       every plate drawn from the real footprints
    PRINT_PLATES.md         the human sheet: plate -> profile -> parts, and the
                            three-step slicer routine

Receipt (last stdout line; --out PATH writes it atomically):
    {ok, bed, spacing_mm, edge_mm, cell_mm, rotations_deg,
     groups:[{name, profile, n_parts, n_instances, n_plates}],
     plates:[{group, plate, file, file_3mf, profile, n_instances,
              parts:[{name, instance, stl, rot_deg, translate_mm, footprint_mm,
                      height_mm, position_mm}],
              bbox_mm, max_height_mm, utilization, min_gap_cells, gap_ok, centered}],
     summary:{n_groups, n_plates, n_instances}, checks:{...}, files:{...}}

Packing method (stated, not hidden): first-fit-decreasing by footprint area over
the plates already open for the group. A candidate position is any cell where
the part mask's cross-correlation with the plate's BLOCKED mask (placed parts
dilated by the spacing, the edge margin, the exclusion zones) is zero — computed
with FFTs, so every rotation is scored over every position. Score = (enclosing
square after placement, enclosing area, y, x): the pack stays compact and
low-left, so multi-plate groups fill plate 1 before opening plate 2. The gap is
guaranteed by construction (conservative raster + square dilation) AND
re-measured on the finished plate; both numbers are in the receipt.

Refuses (ok:false, exit 1): a part taller than bed.z; a part whose footprint fits
no rotation on an empty plate; a group with no parts or an empty profile; a part
name in two groups; an ASCII or malformed STL. It never rotates a part off its
print pose and never scales anything.
"""
from __future__ import annotations

import json
import math
import os
import re
import sys
import zipfile

import numpy as np

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(TOOLS_DIR))  # tools/: _stl, _receipt, the layout map
import _layout  # noqa: E402
_layout.add_import_paths()
import _receipt  # noqa: E402 — the shared `--out` / `receipt` / dry-run contract
from _stl import load_stl, write_stl  # noqa: E402

DEFAULT_ROTATIONS = [0.0, 90.0, 180.0, 270.0]
HALF_DIAG = math.sqrt(2.0) / 2.0  # cell half-diagonal, in cells: the conservative raster radius


def log(msg: str) -> None:
	print(msg, file=sys.stderr, flush=True)


# ------------------------------------------------------------------ geometry --
def rotate_z(tris: np.ndarray, deg: float) -> np.ndarray:
	"""Rotate (n,3,3) triangles about +Z through the origin. Multiples of 90 deg
	use exact (0, +-1) factors so the coordinates stay bit-identical."""
	q = deg % 360.0
	exact = {0.0: (1.0, 0.0), 90.0: (0.0, 1.0), 180.0: (-1.0, 0.0), 270.0: (0.0, -1.0)}
	if q in exact:
		c, s = exact[q]
	else:
		c, s = math.cos(math.radians(q)), math.sin(math.radians(q))
	out = tris.copy()
	x, y = tris[:, :, 0], tris[:, :, 1]
	out[:, :, 0] = c * x - s * y
	out[:, :, 1] = s * x + c * y
	return out


def _mark_batch(mask: np.ndarray, tri: np.ndarray, ix0: np.ndarray, iy0: np.ndarray, M: int) -> None:
	"""Vectorised conservative raster of a batch of triangles (n,3,2) in CELL
	units, each covering at most M x M cells from its (ix0, iy0) corner: a cell is
	occupied when its centre lies inside the triangle or within half a cell
	diagonal of one of its edges (i.e. the cell square touches the triangle)."""
	n = len(tri)
	if n == 0:
		return
	o = np.arange(M, dtype=np.float64)
	px = ix0[:, None, None] + o[None, None, :] + 0.5      # (n,1,M)
	py = iy0[:, None, None] + o[None, :, None] + 0.5      # (n,M,1)
	px = np.broadcast_to(px, (n, M, M))
	py = np.broadcast_to(py, (n, M, M))
	pos = np.ones((n, M, M), dtype=bool)
	neg = np.ones((n, M, M), dtype=bool)
	mind = np.full((n, M, M), np.inf)
	for k in range(3):
		A = tri[:, k, :]
		B = tri[:, (k + 1) % 3, :]
		ex = (B[:, 0] - A[:, 0])[:, None, None]
		ey = (B[:, 1] - A[:, 1])[:, None, None]
		rx = px - A[:, 0][:, None, None]
		ry = py - A[:, 1][:, None, None]
		e = ex * ry - ey * rx                       # edge function
		pos &= e >= 0
		neg &= e <= 0
		L2 = ex * ex + ey * ey
		t = np.clip((rx * ex + ry * ey) / np.where(L2 > 0, L2, 1.0), 0.0, 1.0)
		dx = rx - t * ex
		dy = ry - t * ey
		mind = np.minimum(mind, dx * dx + dy * dy)
	occ = pos | neg | (mind <= HALF_DIAG * HALF_DIAG)
	H, W = mask.shape
	cy = iy0[:, None, None] + np.arange(M)[None, :, None]
	cx = ix0[:, None, None] + np.arange(M)[None, None, :]
	cy = np.broadcast_to(cy, (n, M, M))[occ]
	cx = np.broadcast_to(cx, (n, M, M))[occ]
	keep = (cy >= 0) & (cy < H) & (cx >= 0) & (cx < W)
	mask[cy[keep], cx[keep]] = True


def footprint_mask(tris: np.ndarray, cell: float) -> np.ndarray:
	"""Conservative raster of the XY projection of (n,3,3) triangles whose XY
	min corner is at (0,0): mask[iy, ix] True when the cell square [ix,ix+1) x
	[iy,iy+1) (in cells) touches any triangle. Every cell the part touches is
	occupied; the part never extends beyond its occupied cells."""
	xy = tris[:, :, :2] / cell
	W = int(math.floor(xy[:, :, 0].max())) + 2
	H = int(math.floor(xy[:, :, 1].max())) + 2
	mask = np.zeros((H, W), dtype=bool)
	lo = np.floor(xy.min(axis=1)).astype(np.int64) - 1
	hi = np.floor(xy.max(axis=1)).astype(np.int64) + 1
	span = (hi - lo).max(axis=1) + 1  # cells per side, expansion included
	# batch by size class so the common small facets go through one vectorised
	# pass each; the few large caps are rasterised on their own grid
	order = np.argsort(span, kind="stable")
	xy_s, lo_s, span_s = xy[order], lo[order], span[order]
	i = 0
	n = len(xy_s)
	while i < n:
		s = int(span_s[i])
		j = i
		while j < n and span_s[j] <= s + 3 and span_s[j] <= 24:
			j += 1
		if j == i:  # a big one: its own pass
			j = i + 1
		M = int(span_s[j - 1])
		if M > 4096:
			raise ValueError("a triangle spans more than 4096 raster cells — coarsen cell_mm or check the STL units")
		# bound the (chunk x M x M) work arrays to ~400k cells so a big batch never balloons memory
		step = max(1, 400_000 // (M * M))
		for a in range(i, j, step):
			b = min(j, a + step)
			_mark_batch(mask, xy_s[a:b], lo_s[a:b, 0], lo_s[a:b, 1], M)
		i = j
	return mask


def dilate_square(mask: np.ndarray, r: int) -> np.ndarray:
	"""Binary dilation by a (2r+1)^2 square (separable max filter, zero-padded:
	nothing wraps around the grid edge)."""
	if r <= 0:
		return mask.copy()
	out = mask
	for axis in (0, 1):
		H, W = out.shape
		pad = [(0, 0), (0, 0)]
		pad[axis] = (r, r)
		P = np.pad(out, pad)
		acc = np.zeros_like(out)
		for k in range(2 * r + 1):
			acc |= P[k:k + H, :] if axis == 0 else P[:, k:k + W]
		out = acc
	return out


def feasible_positions(blocked: np.ndarray, mask: np.ndarray):
	"""(iy, ix) arrays of every top-left cell where `mask` overlaps no blocked
	cell, via FFT cross-correlation (exact: the correlation counts overlapping
	True cells, and only the wrap-free range is kept)."""
	H, W = blocked.shape
	h, w = mask.shape
	if h > H or w > W:
		return np.zeros(0, dtype=np.int64), np.zeros(0, dtype=np.int64)
	Bf = np.fft.rfft2(blocked.astype(np.float64))
	Mp = np.zeros((H, W), dtype=np.float64)
	Mp[:h, :w] = mask
	C = np.fft.irfft2(Bf * np.conj(np.fft.rfft2(Mp)), s=(H, W))
	C = C[: H - h + 1, : W - w + 1]
	return np.nonzero(C < 0.5)


# ------------------------------------------------------------------- packing --
class PlateGrid:
	def __init__(self, bed, cell, edge_cells, sp_cells, zones):
		self.cell = cell
		self.W = int(math.floor(bed["x"] / cell))
		self.H = int(math.floor(bed["y"] / cell))
		self.sp = sp_cells
		self.edge = edge_cells
		self.base = np.zeros((self.H, self.W), dtype=bool)
		e = edge_cells
		self.base[:e, :] = True
		self.base[-e:, :] = True
		self.base[:, :e] = True
		self.base[:, -e:] = True
		for (x0, y0, x1, y1) in zones:
			ix0 = max(0, int(math.floor(x0 / cell)))
			iy0 = max(0, int(math.floor(y0 / cell)))
			ix1 = min(self.W, int(math.ceil(x1 / cell)))
			iy1 = min(self.H, int(math.ceil(y1 / cell)))
			self.base[iy0:iy1, ix0:ix1] = True
		self.occupied = np.zeros((self.H, self.W), dtype=bool)  # placed parts, undilated
		self.blocked = self.base.copy()
		self.placed = []   # dicts
		self.x0 = self.y0 = self.x1 = self.y1 = None  # bbox of placed masks, cells

	def try_place(self, inst: dict):
		"""Best (rotation, iy, ix) on this plate for the instance, or None."""
		best = None
		if self.placed:
			bx0, by0, bx1, by1 = self.x0, self.y0, self.x1, self.y1
		else:
			bx0 = by0 = bx1 = by1 = self.edge
		for ri, (rot, mask) in enumerate(inst["masks"]):
			h, w = mask.shape
			iy, ix = feasible_positions(self.blocked, mask)
			if len(iy) == 0:
				continue
			x1 = np.maximum(bx1, ix + w)
			y1 = np.maximum(by1, iy + h)
			x0 = np.minimum(bx0, ix)
			y0 = np.minimum(by0, iy)
			side = np.maximum(x1 - x0, y1 - y0)
			area = (x1 - x0) * (y1 - y0)
			k = np.lexsort((ix, iy, area, side))[0]
			cand = (int(side[k]), int(area[k]), int(iy[k]), int(ix[k]), ri)
			if best is None or cand < best:
				best = cand
		if best is None:
			return None
		side, area, iy, ix, ri = best
		return ri, iy, ix

	def commit(self, inst: dict, ri: int, iy: int, ix: int):
		rot, mask = inst["masks"][ri]
		h, w = mask.shape
		self.occupied[iy:iy + h, ix:ix + w] |= mask
		self.blocked = self.base | dilate_square(self.occupied, self.sp)
		if not self.placed:
			self.x0, self.y0, self.x1, self.y1 = ix, iy, ix + w, iy + h
		else:
			self.x0, self.y0 = min(self.x0, ix), min(self.y0, iy)
			self.x1, self.y1 = max(self.x1, ix + w), max(self.y1, iy + h)
		self.placed.append({"inst": inst, "rot_index": ri, "iy": iy, "ix": ix})


def measure_gaps(labels: np.ndarray, n_labels: int, cap: int) -> int:
	"""Smallest Chebyshev gap in EMPTY CELLS between any two labelled regions,
	probed by growing each region one cell at a time: a hit after g growth steps
	means g-1 empty cells lay between (0 = adjacent or overlapping). Returns
	`cap` when nothing is met within `cap` steps (i.e. the gap is >= cap)."""
	if n_labels < 2:
		return cap
	best = cap
	for lab in range(1, n_labels + 1):
		m = labels == lab
		others = (labels > 0) & ~m
		if not others.any():
			continue
		if (m & others).any():
			return 0
		grown = m
		for g in range(1, cap + 1):
			grown = dilate_square(grown, 1)
			if (grown & others).any():
				best = min(best, g - 1)
				break
	return best


# --------------------------------------------------------------- writers --
def write_3mf(path: str, title: str, objects: list, items: list) -> None:
	"""Minimal 3MF core-spec package: one <object> per distinct part mesh (vertices
	de-duplicated), one <item> per placed instance with its transform. Deterministic
	bytes (fixed zip timestamps, no clock)."""
	L = ['<?xml version="1.0" encoding="UTF-8"?>',
	     '<model unit="millimeter" xml:lang="en-US" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">',
	     f' <metadata name="Title">{_xml(title)}</metadata>',
	     ' <metadata name="Application">LMCAD tools/publish/build_plates.py</metadata>',
	     ' <resources>']
	for obj in objects:
		tris = obj["tris"].reshape(-1, 3)
		verts, inv = np.unique(tris, axis=0, return_inverse=True)
		inv = np.asarray(inv).reshape(-1).reshape(-1, 3)
		L.append(f'  <object id="{obj["id"]}" name="{_xml(obj["name"])}" type="model">')
		L.append('   <mesh>')
		L.append('    <vertices>')
		L.extend(f'     <vertex x="{v[0]:.5f}" y="{v[1]:.5f}" z="{v[2]:.5f}"/>' for v in verts)
		L.append('    </vertices>')
		L.append('    <triangles>')
		L.extend(f'     <triangle v1="{t[0]}" v2="{t[1]}" v3="{t[2]}"/>' for t in inv)
		L.append('    </triangles>')
		L.append('   </mesh>')
		L.append('  </object>')
	L.append(' </resources>')
	L.append(' <build>')
	for it in items:
		L.append(f'  <item objectid="{it["objectid"]}" transform="{it["transform"]}" partnumber="{_xml(it["name"])}"/>')
	L.append(' </build>')
	L.append('</model>')
	model = "\n".join(L).encode("utf-8")
	ctypes = ('<?xml version="1.0" encoding="UTF-8"?>\n'
	          '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">\n'
	          ' <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>\n'
	          ' <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>\n'
	          '</Types>\n').encode("utf-8")
	rels = ('<?xml version="1.0" encoding="UTF-8"?>\n'
	        '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">\n'
	        ' <Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>\n'
	        '</Relationships>\n').encode("utf-8")
	with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED) as z:
		for name, data in (("[Content_Types].xml", ctypes), ("_rels/.rels", rels), ("3D/3dmodel.model", model)):
			zi = zipfile.ZipInfo(name, date_time=(1980, 1, 1, 0, 0, 0))
			zi.compress_type = zipfile.ZIP_DEFLATED
			z.writestr(zi, data)


def _xml(s: str) -> str:
	return (str(s).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;").replace('"', "&quot;"))


def draw_layout(plates: list, bed: dict, edge: float, zones: list, out_png: str, title: str, date: str) -> None:
	import matplotlib

	matplotlib.use("Agg")
	import matplotlib.pyplot as plt
	from matplotlib.colors import ListedColormap
	from matplotlib.patches import Rectangle

	PAL = ["#5b7d9e", "#c08552", "#6d9b6d", "#8a7f9e", "#b0a15f", "#6d8f8f", "#a86a6a", "#7a9a5a", "#9a7a5a", "#5a7a9a"]
	n = len(plates)
	cols = min(n, 3)
	rows = int(math.ceil(n / cols))
	fig, axs = plt.subplots(rows, cols, figsize=(4.6 * cols, 4.9 * rows), dpi=140, squeeze=False)
	fig.patch.set_facecolor("#f5f4f2")
	for i, pl in enumerate(plates):
		ax = axs[i // cols][i % cols]
		lab = pl["_labels"]
		k = int(lab.max())
		cmap = ListedColormap(["#f5f4f2"] + [PAL[j % len(PAL)] for j in range(max(k, 1))])
		H, W = lab.shape
		ax.imshow(lab, origin="lower", cmap=cmap, vmin=0, vmax=max(k, 1), interpolation="nearest",
		          extent=(0, W * pl["_cell"], 0, H * pl["_cell"]))
		ax.add_patch(Rectangle((0, 0), bed["x"], bed["y"], fill=False, edgecolor="#3a3f45", lw=1.2))
		ax.add_patch(Rectangle((edge, edge), bed["x"] - 2 * edge, bed["y"] - 2 * edge, fill=False,
		                       edgecolor="#9a9a9a", lw=0.7, ls="--"))
		for (x0, y0, x1, y1) in zones:
			ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, facecolor="#d9534f", alpha=0.25, hatch="///", edgecolor="#d9534f", lw=0.6))
		for j, p in enumerate(pl["parts"]):
			cx, cy = p["_centroid_mm"]
			name = p["name"] if p["instance"] == 1 and p["_qty"] == 1 else f"{p['name']} #{p['instance']}"
			ax.text(cx, cy, name, ha="center", va="center", fontsize=6.2, family="DejaVu Sans", color="#1a1d21",
			        bbox=dict(boxstyle="round,pad=0.15", facecolor="white", alpha=0.7, lw=0))
		ax.set_xlim(-6, bed["x"] + 6)
		ax.set_ylim(-6, bed["y"] + 6)
		ax.set_aspect("equal")
		ax.tick_params(labelsize=6)
		ax.set_title(f"{pl['group']} · plate {pl['plate']}/{pl['_n_in_group']} · {pl['n_instances']} parts · "
		             f"{pl['utilization'] * 100:.1f}% of the bed · tallest {pl['max_height_mm']:.0f} mm",
		             fontsize=8, family="DejaVu Sans", weight="bold")
		ax.set_xlabel(f"profile {pl['group']}: {pl['_profile_line']}", fontsize=6.4, family="DejaVu Sans")
	for i in range(n, rows * cols):
		axs[i // cols][i % cols].axis("off")
	fig.suptitle(f"{title} — build plates ({bed['x']:.0f}×{bed['y']:.0f}×{bed['z']:.0f} mm bed)"
	             f"{(' · ' + date) if date else ''}", fontsize=10, family="DejaVu Sans", weight="bold")
	fig.tight_layout(rect=(0, 0, 1, 0.95))
	fig.savefig(out_png, facecolor="#f5f4f2")
	plt.close(fig)


def write_sheet(path: str, job: dict, bed: dict, plates: list, groups: list, spacing: float, edge: float, title: str, date: str) -> None:
	L = [f"# Build plates — {title}", ""]
	L.append(f"Generated by `tools/publish/build_plates.py`{(' · ' + date) if date else ''} · bed {bed['x']:.0f} × {bed['y']:.0f} × {bed['z']:.0f} mm · "
	         f"gap between parts ≥ {spacing:g} mm · margin to the bed edge ≥ {edge:g} mm · layout sheet `plates_layout.png`.")
	L.append("")
	L.append("Every plate carries ONE slicer profile. For each plate: **(1)** select that profile in the slicer, "
	         "**(2)** import the plate file — the `.stl` is one body per part, already arranged; the `.3mf` is the same "
	         "arrangement with every part named and positioned — **(3)** slice. Do not rotate or re-arrange anything: "
	         "every part is in its verified print orientation and the gaps are already checked.")
	L.append("")
	L.append("| plate | file | profile | parts | footprint (mm) | tallest (mm) | bed used |")
	L.append("|---|---|---|---|---|---|---|")
	for pl in plates:
		bb = pl["bbox_mm"]
		L.append(f"| {pl['group']}-{pl['plate']} | `{os.path.basename(pl['file'])}` | {pl['group']} | {pl['n_instances']} | "
		         f"{bb[2] - bb[0]:.0f} × {bb[3] - bb[1]:.0f} | {pl['max_height_mm']:.0f} | {pl['utilization'] * 100:.0f} % |")
	L.append("")
	for g in groups:
		L.append(f"## Profile {g['name']} — {g['n_plates']} plate{'s' if g['n_plates'] != 1 else ''}, {g['n_instances']} part instance{'s' if g['n_instances'] != 1 else ''}")
		L.append("")
		L.append("| setting | value |")
		L.append("|---|---|")
		for k, v in g["profile"].items():
			L.append(f"| {k} | {v} |")
		L.append("")
		for pl in [p for p in plates if p["group"] == g["name"]]:
			names = []
			by = {}
			for p in pl["parts"]:
				by[p["name"]] = by.get(p["name"], 0) + 1
			for nm, c in by.items():
				names.append(f"`{nm}`" + (f" ×{c}" if c > 1 else ""))
			f3 = f" / `{os.path.basename(pl['file_3mf'])}`" if pl.get("file_3mf") else ""
			L.append(f"- **Plate {g['name']}-{pl['plate']}** — `{os.path.basename(pl['file'])}`{f3}: " + ", ".join(names) + ".")
		L.append("")
	L.append("The layout is the tool's, not the slicer's auto-arrange: the parts are nested on their real footprints with the "
	         "gap above, so the slicer's arrange step is not needed and would only undo the checked spacing.")
	L.append("")
	with open(path, "w", encoding="utf-8") as f:
		f.write("\n".join(L))


# ------------------------------------------------------------------- build --
def build_plates(job: dict) -> dict:
	out_dir = job["out_dir"]
	os.makedirs(out_dir, exist_ok=True)
	if not isinstance(job.get("bed"), dict) or any(k not in job["bed"] for k in ("x", "y", "z")):
		raise ValueError("job.bed {x, y, z} is required — the printable volume is a printer fact, it has no default")
	bed = {k: float(job["bed"][k]) for k in ("x", "y", "z")}
	spacing = float(job.get("spacing_mm", 5.0))
	edge = float(job.get("edge_mm", spacing))
	cell = float(job.get("cell_mm", 1.0))
	if cell <= 0 or spacing < 0 or edge < 0:
		raise ValueError("cell_mm must be > 0; spacing_mm and edge_mm must be >= 0")
	rotations = [float(r) for r in job.get("rotations_deg", DEFAULT_ROTATIONS)]
	if not rotations:
		raise ValueError("rotations_deg must name at least one rotation (0 keeps the export as is)")
	zones = [[float(v) for v in z] for z in (job.get("exclusion_zones") or [])]
	for z in zones:
		if len(z) != 4 or z[0] >= z[2] or z[1] >= z[3]:
			raise ValueError(f"exclusion zone {z} must be [x0, y0, x1, y1] with x0 < x1 and y0 < y1")
	emit_3mf = bool(job.get("emit_3mf", True))
	date = str(job.get("date", ""))
	title = str(job.get("title", os.path.basename(os.path.abspath(out_dir))))
	groups_in = job.get("groups")
	if not isinstance(groups_in, list) or not groups_in:
		raise ValueError("job.groups is required: one {name, profile, parts} per slicer profile")
	sp_cells = int(math.ceil(spacing / cell - 1e-9))
	edge_cells = max(1, int(math.ceil(edge / cell - 1e-9)))

	seen_names, seen_groups = {}, set()
	groups, all_instances = [], []
	for gi, g in enumerate(groups_in):
		name = str(g.get("name", "")).strip()
		if not name or any(ch in name for ch in "/\\ \t"):
			raise ValueError(f"groups[{gi}].name must be a short token without spaces or slashes")
		if name in seen_groups:
			raise ValueError(f"group name {name!r} appears twice")
		seen_groups.add(name)
		profile = g.get("profile")
		if not isinstance(profile, dict) or not profile:
			raise ValueError(f"group {name!r} needs a non-empty profile {{setting: value}} — it is what the user selects in the slicer")
		parts = g.get("parts")
		if not isinstance(parts, list) or not parts:
			raise ValueError(f"group {name!r} has no parts")
		n_inst = 0
		for pi, p in enumerate(parts):
			pname = str(p.get("name", "")).strip()
			if not pname:
				raise ValueError(f"group {name!r} parts[{pi}] has no name")
			if pname in seen_names:
				raise ValueError(f"part {pname!r} is in group {seen_names[pname]!r} AND {name!r} — a part belongs to exactly one profile")
			seen_names[pname] = name
			qty = int(p.get("qty", 1))
			if qty < 1:
				raise ValueError(f"part {pname!r}: qty must be >= 1")
			tris = load_stl(p["stl"])
			lo = tris.reshape(-1, 3).min(axis=0)
			hi = tris.reshape(-1, 3).max(axis=0)
			height = float(hi[2] - lo[2])
			if height > bed["z"] + 1e-9:
				raise ValueError(f"part {pname!r} is {height:.2f} mm tall — exceeds bed z {bed['z']:g} mm: REFUSED")
			masks = []
			for rot in rotations:
				rt = rotate_z(tris, rot)
				rlo = rt.reshape(-1, 3).min(axis=0)
				rt = rt - rlo  # min corner to the origin, bed at z = 0
				masks.append((rot, footprint_mask(rt, cell), rlo))
			fits = any(m.shape[0] + 2 * edge_cells <= int(bed["y"] // cell) and m.shape[1] + 2 * edge_cells <= int(bed["x"] // cell)
			           for _, m, _ in masks)
			if not fits:
				w0, d0 = float(hi[0] - lo[0]), float(hi[1] - lo[1])
				raise ValueError(f"part {pname!r} footprint {w0:.1f} x {d0:.1f} mm fits no candidate rotation on the "
				                 f"{bed['x']:g} x {bed['y']:g} bed with a {edge:g} mm edge margin: REFUSED")
			area = int(masks[0][1].sum())
			for k in range(qty):
				all_instances.append({
					"group": name, "name": pname, "instance": k + 1, "qty": qty, "stl": p["stl"],
					"tris": tris, "height": height, "area_cells": area,
					"masks": [(rot, m) for rot, m, _ in masks], "rlo": [rlo for _, _, rlo in masks],
					"footprint_mm": [float(hi[0] - lo[0]), float(hi[1] - lo[1])],
				})
			n_inst += qty
		groups.append({"name": name, "profile": profile, "n_parts": len(parts), "n_instances": n_inst, "n_plates": 0,
		               "one_part_per_plate": bool(g.get("one_part_per_plate", False))})

	# ---- pack, group by group ---------------------------------------------------
	plates_out, obj_meshes = [], {}
	W_cells, H_cells = int(bed["x"] // cell), int(bed["y"] // cell)
	for g in groups:
		insts = [i for i in all_instances if i["group"] == g["name"]]
		insts.sort(key=lambda i: (-i["area_cells"], i["name"], i["instance"]))  # first-fit-decreasing, stable
		plates = []
		one_per = bool(g.get("one_part_per_plate", False))   # spiral/vase profiles: the slicer spirals one object per plate
		for inst in insts:
			placed = False
			for pl in ([] if one_per else plates):
				r = pl.try_place(inst)
				if r is not None:
					pl.commit(inst, *r)
					placed = True
					break
			if not placed:
				pl = PlateGrid(bed, cell, edge_cells, sp_cells, zones)
				r = pl.try_place(inst)
				if r is None:  # pre-checked on the empty-bed test above; belt and braces
					raise ValueError(f"part {inst['name']!r} does not fit an empty plate: REFUSED")
				pl.commit(inst, *r)
				plates.append(pl)
		g["n_plates"] = len(plates)
		for pi, pl in enumerate(plates):
			# final transforms: rotation about Z, then the translation that puts the rotated min corner at the cell
			# position (bed at z = 0); then the whole arrangement centred on the bed unless zones pin it
			placed = []
			merged = []
			for rec in pl.placed:
				inst, ri = rec["inst"], rec["rot_index"]
				rot, mask = inst["masks"][ri]
				rlo = inst["rlo"][ri]
				tx = rec["ix"] * cell - rlo[0]
				ty = rec["iy"] * cell - rlo[1]
				tz = -rlo[2]
				placed.append({"inst": inst, "rot": rot, "t": np.array([tx, ty, tz]), "mask": mask, "iy": rec["iy"], "ix": rec["ix"]})
			# real (not raster) bounding box of the arrangement
			ext = []
			for p in placed:
				rt = rotate_z(p["inst"]["tris"], p["rot"]) + p["t"]
				p["_tris"] = rt
				ext.append((rt[:, :, 0].min(), rt[:, :, 1].min(), rt[:, :, 0].max(), rt[:, :, 1].max()))
			bx0, by0 = min(e[0] for e in ext), min(e[1] for e in ext)
			bx1, by1 = max(e[2] for e in ext), max(e[3] for e in ext)
			centered = not zones
			shift = np.array([(bed["x"] - (bx0 + bx1)) / 2.0, (bed["y"] - (by0 + by1)) / 2.0, 0.0]) if centered else np.zeros(3)
			# snap the shift to whole cells: the packed masks then translate exactly onto the plate grid below
			shift[:2] = np.round(shift[:2] / cell) * cell
			sx, sy = int(round(shift[0] / cell)), int(round(shift[1] / cell))
			labels = np.zeros((H_cells, W_cells), dtype=np.int32)
			parts_rec = []
			outside = False
			for j, p in enumerate(placed):
				p["t"] = p["t"] + shift
				p["_tris"] = p["_tris"] + shift
				rt = p["_tris"]
				lo3 = rt.reshape(-1, 3).min(axis=0)
				hi3 = rt.reshape(-1, 3).max(axis=0)
				# the REAL geometry must sit inside the bed margin
				if lo3[0] < edge - 1e-6 or lo3[1] < edge - 1e-6 or hi3[0] > bed["x"] - edge + 1e-6 or hi3[1] > bed["y"] - edge + 1e-6:
					outside = True
				# the packing mask, translated by whole cells, on the plate grid: the gap re-measurement runs on this
				m = p["mask"]
				oy, ox = p["iy"] + sy, p["ix"] + sx
				h, w = m.shape
				if oy < 0 or ox < 0 or oy + h > H_cells or ox + w > W_cells:
					outside = True
				else:
					sub = labels[oy:oy + h, ox:ox + w]
					if (sub[m] != 0).any():
						outside = True  # two masks on one cell: the packer's own invariant broke
					sub[m] = j + 1
				ys, xs = np.nonzero(m)
				cen = ((xs.mean() + 0.5 + ox) * cell, (ys.mean() + 0.5 + oy) * cell)
				merged.append(rt)
				inst = p["inst"]
				parts_rec.append({
					"name": inst["name"], "instance": inst["instance"], "stl": os.path.abspath(inst["stl"]),
					"rot_deg": float(p["rot"]), "translate_mm": [round(float(v), 4) for v in p["t"]],
					"footprint_mm": [round(v, 2) for v in inst["footprint_mm"]], "height_mm": round(inst["height"], 2),
					"position_mm": [round(float(lo3[0]), 2), round(float(lo3[1]), 2), round(float(hi3[0]), 2), round(float(hi3[1]), 2)],
					"_centroid_mm": cen, "_qty": inst["qty"],
				})
			if outside:
				raise ValueError(f"internal: an instance on plate {g['name']}-{pi + 1} landed outside the bed margin after centring")
			gap_cells = measure_gaps(labels, len(placed), cap=2 * sp_cells + 2)
			gap_ok = gap_cells >= sp_cells
			if not gap_ok:
				raise ValueError(f"internal: plate {g['name']}-{pi + 1} re-measured gap {gap_cells} cells < {sp_cells}")
			fname = os.path.join(out_dir, f"{g['name']}_plate_{pi + 1}.stl")
			write_stl(fname, np.concatenate(merged))
			rec = {
				"group": g["name"], "plate": pi + 1, "file": os.path.abspath(fname), "file_3mf": None,
				"profile": g["profile"], "n_instances": len(placed), "parts": parts_rec,
				"bbox_mm": [round(float(bx0 + shift[0]), 2), round(float(by0 + shift[1]), 2), round(float(bx1 + shift[0]), 2), round(float(by1 + shift[1]), 2)],
				"max_height_mm": round(max(p["inst"]["height"] for p in placed), 2),
				"utilization": round(float((labels > 0).sum()) / float(W_cells * H_cells), 4),
				"min_gap_cells": int(gap_cells), "min_gap_mm_at_least": round(min(gap_cells, 2 * sp_cells + 2) * cell, 3),
				"gap_ok": bool(gap_ok), "centered": bool(centered), "triangles": int(sum(len(t) for t in merged)),
				"_labels": labels, "_cell": cell, "_n_in_group": len(plates),
				"_profile_line": ", ".join(f"{k} {v}" for k, v in g["profile"].items())[:110],
			}
			if emit_3mf:
				objs, items, ids = [], [], {}
				for p in placed:
					inst = p["inst"]
					key = inst["name"]
					if key not in ids:
						ids[key] = len(ids) + 1
						objs.append({"id": ids[key], "name": key, "tris": inst["tris"]})
					q = float(p["rot"]) % 360.0
					ex = {0.0: (1.0, 0.0), 90.0: (0.0, 1.0), 180.0: (-1.0, 0.0), 270.0: (0.0, -1.0)}
					c, s = ex.get(q, (math.cos(math.radians(q)), math.sin(math.radians(q))))
					t = p["t"]
					# 3MF row-vector convention: rows = the transposed rotation, last row = the translation
					tf = f"{c:.9g} {s:.9g} 0 {-s:.9g} {c:.9g} 0 0 0 1 {t[0]:.5f} {t[1]:.5f} {t[2]:.5f}"
					nm = inst["name"] if inst["qty"] == 1 else f"{inst['name']}#{inst['instance']}"
					items.append({"objectid": ids[key], "transform": tf, "name": nm})
				f3 = os.path.join(out_dir, f"{g['name']}_plate_{pi + 1}.3mf")
				write_3mf(f3, f"{title} — plate {g['name']}-{pi + 1}", objs, items)
				rec["file_3mf"] = os.path.abspath(f3)
			plates_out.append(rec)

	layout_png = os.path.join(out_dir, "plates_layout.png")
	draw_layout(plates_out, bed, edge, zones, layout_png, title, date)
	sheet = os.path.join(out_dir, "PRINT_PLATES.md")
	write_sheet(sheet, job, bed, plates_out, groups, spacing, edge, title, date)

	n_inst = sum(g["n_instances"] for g in groups)
	placed_total = sum(p["n_instances"] for p in plates_out)
	# stale plate files from an earlier run with MORE plates in a group would otherwise survive beside the fresh ones
	stale = []
	for g in groups:
		k = g["n_plates"] + 1
		while True:
			found = False
			for ext in (".stl", ".3mf"):
				f = os.path.join(out_dir, f"{g['name']}_plate_{k}{ext}")
				if os.path.exists(f):
					os.remove(f)
					stale.append(os.path.abspath(f))
					found = True
			if not found:
				break
			k += 1
	# plate files of a group that no longer exists (a profile renamed or retired) would survive too
	names = {g["name"] for g in groups}
	for fn in sorted(os.listdir(out_dir)):
		m = re.match(r"^(.+)_plate_(\d+)\.(stl|3mf)$", fn)
		if m and m.group(1) not in names:
			f = os.path.join(out_dir, fn)
			os.remove(f)
			stale.append(os.path.abspath(f))
	for p in plates_out:
		for k in ("_labels", "_cell", "_n_in_group", "_profile_line"):
			p.pop(k, None)
		for q in p["parts"]:
			q.pop("_centroid_mm", None)
			q.pop("_qty", None)
	# scalar views a document can anchor (the audit's key grammar is dotted paths, no list indices)
	by_group = {g["name"]: {"n_plates": g["n_plates"], "n_parts": g["n_parts"], "n_instances": g["n_instances"]} for g in groups}
	by_plate = {f"{p['group']}_{p['plate']}": {"n_instances": p["n_instances"], "utilization": p["utilization"],
	            "utilization_pct": round(p["utilization"] * 100.0, 1), "min_gap_mm_at_least": p["min_gap_mm_at_least"],
	            "max_height_mm": p["max_height_mm"], "footprint_x_mm": round(p["bbox_mm"][2] - p["bbox_mm"][0], 2),
	            "footprint_y_mm": round(p["bbox_mm"][3] - p["bbox_mm"][1], 2), "file": os.path.basename(p["file"])}
	            for p in plates_out}
	checks = {
		"every_instance_placed_once": placed_total == n_inst,
		"every_instance_inside_bed_margin": True,
		"min_gap_ok_on_every_plate": all(p["gap_ok"] for p in plates_out),
		"one_profile_per_plate": True,
		"no_part_in_two_groups": True,
		"print_pose_kept": "rotation about Z only; no X/Y rotation, no scaling",
	}
	receipt = {
		"ok": all(v is True or isinstance(v, str) for v in checks.values()),
		"bed": bed, "spacing_mm": spacing, "edge_mm": edge, "cell_mm": cell, "rotations_deg": rotations,
		"spacing_cells": sp_cells, "edge_cells": edge_cells, "exclusion_zones": zones,
		"groups": groups, "plates": plates_out, "by_group": by_group, "by_plate": by_plate,
		"summary": {"n_groups": len(groups), "n_plates": len(plates_out), "n_instances": n_inst},
		"stale_files_removed": stale,
		"checks": checks,
		"files": {"layout_png": os.path.abspath(layout_png), "sheet_md": os.path.abspath(sheet),
		          "plates": [p["file"] for p in plates_out],
		          "plates_3mf": [p["file_3mf"] for p in plates_out if p.get("file_3mf")]},
		"method": ("first-fit-decreasing by footprint area; conservative raster of each part's XY projection at "
		           f"cell_mm {cell:g}; candidate positions = zero FFT cross-correlation with the blocked mask "
		           "(placed parts dilated by the spacing, the edge margin, the exclusion zones); score = "
		           "(enclosing square, enclosing area, y, x); gap guaranteed by construction and re-measured "
		           "(min_gap_cells) on the finished plate; arrangement centred on the bed unless zones are given"),
	}
	log(f"{len(groups)} group(s) -> {len(plates_out)} plate(s), {n_inst} instance(s)")
	for p in plates_out:
		log(f"  {p['group']}-{p['plate']}: {p['n_instances']} parts, {p['utilization'] * 100:.1f}% of the bed, "
		    f"gap >= {p['min_gap_mm_at_least']:g} mm, tallest {p['max_height_mm']:.1f} mm -> {os.path.basename(p['file'])}")
	return receipt


def _build(job, _job_dir):
	if "out_dir" not in job:
		raise ValueError("job needs 'out_dir'")
	return build_plates(job)


def main():
	return _receipt.doc_cli("build_plates", _build, help_text=__doc__)


if __name__ == "__main__":
	sys.exit(main())
