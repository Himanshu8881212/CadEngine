#!/usr/bin/env python3
"""assembly_doc.py — ONE engineering assembly-documentation sheet: large
exploded iso with BALLOON callouts (drawn circles + plain digits on a single
vertical callout rail, numbers = BOM item numbers), assembled view with scale
bar + overall dimensions, numbered build steps with fastener chips, a ruled
full-width BOM table with a TOTAL row, and an engineering title block — plus
a matching <out_prefix>_instructions.md.

Shares render_sheet.py's DESIGN SYSTEM (the STYLE dict: DejaVu Sans with a
fixed six-size hierarchy, 28 px margins, 16 px gutters, bordered panels with
shaded caption strips and 12 px internal padding) and its loaders/shading
(binary STL only, two-sided 0.35+0.65*|n.L| shading on recomputed normals,
receipts on the last stdout line) — the two documents read as one family.

Usage:  python3 assembly_doc.py job.json

Job JSON (argv[1]) — all lengths mm:
    parts       REQUIRED  [{name, stl (path), color? (any matplotlib color)}]
                          — listed in ASSEMBLY ORDER; the exploded offsets
                          follow list order (part i moves i x spacing_mm)
    explode     REQUIRED  {axis: [x,y,z] OR an axis name "z"/"+z"/"-z",
                          auto: true, gap_mm: 8}  (offsets
                          DERIVED from the parts' bboxes — tight stacks with
                          no hand-tuning)  OR  {axis, spacing_mm: 30}  OR
                          {axis, offsets: {name: [dx,dy,dz], ...}} per-part
    steps       REQUIRED  [{order, text, fasteners?: "4x M3x10"}] — or pass
                          auto_steps: true for a structure-derived DRAFT
                          (one step per part, labeled auto_steps in the
                          receipt so a human/AI knows to polish the prose)
    bom_csv     optional  path to a BOM csv (production_dossier.py's
                          bom_dossier.csv) — its row order defines the BOM
                          item numbers used by the balloons; without it the
                          parts list itself becomes the BOM
    out_prefix  REQUIRED  writes <out_prefix>_assembly_doc.png and
                          <out_prefix>_instructions.md
    date        optional  string stamped in the title block (NEVER read from
                          the clock — determinism doctrine: same job,
                          byte-identical PNG). Defaults to "—".
    rev         optional  title-block revision, default "A"
    project     optional  title-block project name, default "cadcode"
    doc_title   optional  title-block document title, default
                          "<basename(out_prefix)> — assembly"
    view        optional  {elev: 18, azim: -60} — or the list [18, -60] — for
                          both iso panels
    max_px      optional  long-edge pixel cap, default 1800
    max_page_h_in optional tallest page the sheet may grow to, default 20.0
                          (base page is 16 x 10 in; the sheet grows in 0.5 in
                          steps ONLY when the measured steps/BOM do not fit, so
                          a job that fitted before renders byte-identically)

Layout contract (computed, never overlapped): landscape 16:10 page — header
band; main row = EXPLODED VIEW panel (~55% width) + right column stacked
ASSEMBLED (~40% of the column) over ASSEMBLY SEQUENCE; full-width BILL OF
MATERIALS panel; title block strip. Views are FIT: the projected content is
measured and scaled to fill the panel's inner area (no dead space), balloons
sit on one vertical rail with horizontal leaders, steps use a fixed hanging
indent with measured wrapping (font auto-steps 9 -> 6 pt and the doc REFUSES
below that rather than overlap).

Balloon honesty: markers are DRAWN circle patches + plain bold digits (the
same drawn marker in the exploded view and the BOM's ITEM column). Unicode
circled-digit glyphs (0x2460..) are BANNED — they are missing from some font
fallback paths and rendered as '?'. A scene part that cannot be resolved to
a BOM row is a hard ERROR, never a silent '?' balloon.

Output contract: last non-empty stdout line is ONE JSON receipt
({ok:true, png, md, px, parts, steps, bom_rows} or {ok:false, error});
failures exit 1.
"""
from __future__ import annotations

import csv
import json
import os
import re
import sys

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # tools/: the shared contracts + the layout map
import _layout  # noqa: E402
_layout.add_import_paths()  # tools/, tools/analyzers, tools/publish — sibling-style imports keep working after the 2026-09-02 move
from render_sheet import (  # noqa: E402
	PALETTE, STYLE, caption_h, draw_balloon, fit_view, header_band, load_stl,
	page_frame, panel, project_px, proj_pt, px_line, px_rect, px_text,
	shaded_view, text_w_px,
)


def norm_name(s) -> str:
	"""Part-name key for BOM matching: lowercase, parenthetical suffixes and
	punctuation/whitespace runs collapsed — so the scene part 'MOUNT (PETG)'
	resolves to the BOM row 'mount'."""
	s = re.sub(r"\(.*?\)", " ", str(s)).strip().lower()
	return re.sub(r"[\s_\-]+", "_", s).strip("_")


def fmt_mass(g) -> str:
	"""%.0f g (one decimal below 10 g so a 0.3 g part never reads '0 g'),
	x.xx kg above 1000 g."""
	if g is None or g == "":
		return "—"
	g = float(g)
	if g > 1000.0:
		return f"{g / 1000.0:.2f} kg"
	return f"{g:.1f} g" if g < 10.0 else f"{g:.0f} g"


AXIS_NAMES = {"x": (1.0, 0.0, 0.0), "y": (0.0, 1.0, 0.0), "z": (0.0, 0.0, 1.0)}


def parse_axis(value, field):
	"""A direction, given either as a 3-vector or as an axis NAME.

	ball F6 / singulator F12(a): the obvious `"axis": "z"` died with
	`ValueError: could not convert string to float: 'z'` — an error that names
	neither the field nor the accepted shapes. Both spellings are now legal
	(`"z"`, `"+z"`, `"-z"`, `[0,0,1]`), and anything else is refused with a
	message that says what the field is and what it accepts."""
	if value is None:
		raise ValueError(f"{field} is required — a 3-vector like [0,0,1] or an axis name like 'z'/'-z'")
	if isinstance(value, str):
		s = value.strip().lower()
		sign = -1.0 if s.startswith("-") else 1.0
		s = s.lstrip("+-")
		if s not in AXIS_NAMES:
			raise ValueError(f"{field}: unknown axis name {value!r} — use 'x'/'y'/'z' "
			                 f"(optionally signed, e.g. '-z') or a 3-vector like [0,0,1]")
		return sign * np.asarray(AXIS_NAMES[s], dtype=np.float64)
	try:
		v = np.asarray(value, dtype=np.float64).reshape(-1)
	except (TypeError, ValueError) as e:
		raise ValueError(f"{field}: {value!r} is not a 3-vector or an axis name "
		                 f"('x'/'y'/'z', optionally signed) — {e}") from None
	if v.shape != (3,):
		raise ValueError(f"{field}: expected 3 components, got {v.tolist()}")
	return v


def parse_view(value, field="view"):
	"""Camera as `{elev, azim}` OR `[elev, azim]` -> (elev, azim).

	horn F11: every other view-ish field in the render family is a LIST
	(`build_dir`, `explode.axis`, `size_px`), so a list is the natural guess —
	and it died on `AttributeError: 'list' object has no attribute 'get'`, an
	error naming neither the field nor the job. Both shapes are legal now."""
	if value is None:
		return 18.0, -60.0
	if isinstance(value, dict):
		return float(value.get("elev", 18.0)), float(value.get("azim", -60.0))
	if isinstance(value, (list, tuple)) and len(value) == 2:
		return float(value[0]), float(value[1])
	raise ValueError(f"{field}: expected {{elev, azim}} or [elev, azim], got {value!r}")


def resolve_offsets(job, names, tris=None):
	"""Per-part exploded translation vectors, in part-list order.

	Three forms (audit 2026-07-16 — the offsets were hand-tuned per project):
	- explode.offsets {name: [x,y,z]}: manual, wins when present;
	- explode.auto true: offsets DERIVED from geometry — each part's assembled
	  bbox interval along the axis is stacked clear of everything before it
	  plus gap_mm (default 8), so the explode reads tight without hand-tuning;
	- explode.spacing_mm: the legacy uniform ladder."""
	ex = job["explode"]
	axis = parse_axis(ex.get("axis"), "explode.axis")
	n = np.linalg.norm(axis)
	if n < 1e-12:
		raise ValueError("explode.axis is a zero vector")
	axis = axis / n
	if "offsets" in ex:
		offs = ex["offsets"]
		missing = [nm for nm in names if nm not in offs]
		if missing:
			raise ValueError(f"explode.offsets missing parts: {missing}")
		return axis, [np.asarray(offs[nm], dtype=np.float64) for nm in names]
	if ex.get("auto"):
		if tris is None:
			raise ValueError("explode.auto needs part geometry")
		gap = float(ex.get("gap_mm", 8.0))
		intervals = []
		for t in tris:
			proj = t.reshape(-1, 3) @ axis
			intervals.append((float(proj.min()), float(proj.max())))
		offsets = [np.zeros(3)]
		top = intervals[0][1]
		for lo, hi in intervals[1:]:
			d = max(0.0, top + gap - lo)
			offsets.append(axis * d)
			top = hi + d
		return axis, offsets
	spacing = float(ex.get("spacing_mm", 30.0))
	return axis, [axis * spacing * i for i in range(len(names))]


def load_bom(job, parts):
	"""(rows, from_csv): each row {item, name, kind, qty, material, part_number,
	mass_g (line mass = printed grams x qty, None for buys)}. CSV row order
	defines the item numbers; without a csv the parts list is the BOM."""
	if job.get("bom_csv"):
		with open(job["bom_csv"], newline="") as f:
			raw = list(csv.DictReader(f))
		rows = []
		for i, r in enumerate(raw, start=1):
			qty = int(float(r.get("qty") or 1))
			gpu = r.get("grams_per_unit") or ""
			rows.append({
				"item": i, "name": r.get("name", ""), "kind": r.get("kind", ""),
				"qty": qty, "material": (r.get("material") or "—"),
				"part_number": (r.get("part_number") or "—"),
				"mass_g": (float(gpu) * qty if gpu not in ("", "-") else None),
			})
		return rows, True
	rows = [{"item": i + 1, "name": p["name"], "kind": "made", "qty": 1,
		"material": "—", "part_number": "—", "mass_g": None} for i, p in enumerate(parts)]
	return rows, False


def resolve_items(names, bom_rows):
	"""Scene part -> BOM item number, via normalized names. A part with no
	BOM row is a hard error (the old code drew a literal '?' balloon —
	that silent degrade is exactly what this bans)."""
	by_key = {norm_name(r["name"]): r["item"] for r in bom_rows}
	item_of, missing = {}, []
	for nm in names:
		key = norm_name(nm)
		if key in by_key:
			item_of[nm] = by_key[key]
		else:
			missing.append(nm)
	if missing:
		raise ValueError(
			f"parts {missing} have no matching BOM row (BOM names: "
			f"{[r['name'] for r in bom_rows]}) — balloon numbers must be real item numbers")
	return item_of


# --------------------------------------------------------------- balloons --
def draw_balloons(fig, ax, tris_list, names, item_of, inner):
	"""Balloon callouts on ONE vertical rail: for each part, a horizontal
	leader from its projected silhouette edge to the rail, then a drawn
	circle-and-digit balloon (r 9 px). Equal spacing is enforced when part
	centers would collide. inner = the panel's inner rect (px)."""
	r = 9.0
	ix, iy, iw, ih = inner
	rail_x = ix + iw - r  # balloon right edge lands exactly on the inner edge
	per_part = []
	for t in tris_list:
		pts = np.unique(t.reshape(-1, 3), axis=0)
		if len(pts) > 4000:  # px accuracy needs the hull-ish spread, not every vertex
			step = int(np.ceil(len(pts) / 4000.0))
			pts = pts[::step]
		per_part.append(project_px(ax, pts))
	cys = [float((p[:, 1].min() + p[:, 1].max()) / 2.0) for p in per_part]
	order = sorted(range(len(cys)), key=lambda i: -cys[i])
	pitch = 2.0 * r + 8.0
	ys = [cys[i] for i in order]
	collide = any(ys[k - 1] - ys[k] < pitch for k in range(1, len(ys)))
	if collide and len(ys) > 1:  # equal spacing, centered on the content span
		mid = (ys[0] + ys[-1]) / 2.0
		span = max(ys[0] - ys[-1], pitch * (len(ys) - 1))
		ys = [mid + span / 2.0 - k * span / (len(ys) - 1) for k in range(len(ys))]
	ys = [min(max(y, iy + r), iy + ih - r) for y in ys]
	for k, i in enumerate(order):
		P, y = per_part[i], ys[k]
		band = P[np.abs(P[:, 1] - y) <= max(6.0, 0.05 * (P[:, 1].max() - P[:, 1].min()))]
		x_edge = float(band[:, 0].max()) if len(band) else float(P[:, 0].max())
		x0 = min(x_edge + 6.0, rail_x - r - 12.0)
		px_line(fig, x0, y, rail_x - r, y, color=STYLE["ink"], lw=0.8, z=25)
		draw_balloon(fig, rail_x, y, item_of[names[i]], r=r)


def explode_guides(ax, axis, tris_exploded, offsets, radius):
	"""Explode-axis centerline (dash-dot, drawn OVER the parts in projected 2D
	so occlusion never hides it — drafting centerline convention) + per-part
	alignment guides from each part's exploded position back to its assembled
	position (short dashes)."""
	cents_ex, cents_asm = [], []
	for t, off in zip(tris_exploded, offsets):
		v = t.reshape(-1, 3)
		c_ex = (v.min(axis=0) + v.max(axis=0)) / 2.0
		cents_ex.append(c_ex)
		cents_asm.append(c_ex - off)
	all_c = cents_ex + cents_asm
	s = [float(np.dot(c, axis)) for c in all_c]
	base = all_c[int(np.argmin(s))] - axis * 0.14 * radius
	tip = all_c[int(np.argmax(s))] + axis * 0.14 * radius
	p0, p1 = proj_pt(ax, base), proj_pt(ax, tip)
	ax.add_line(Line2D([p0[0], p1[0]], [p0[1], p1[1]], transform=ax.transData,
		color=STYLE["ink"], alpha=0.75, lw=0.9, ls=(0, (7, 3, 1.5, 3)), zorder=50, clip_on=False))
	for c_ex, c_asm, off in zip(cents_ex, cents_asm, offsets):
		if np.linalg.norm(off) < 1e-9:
			continue
		q0, q1 = proj_pt(ax, c_asm), proj_pt(ax, c_ex)
		ax.add_line(Line2D([q0[0], q1[0]], [q0[1], q1[1]], transform=ax.transData,
			color=STYLE["ink"], alpha=0.5, lw=0.7, ls=(0, (2.5, 2.5)), zorder=50, clip_on=False))


# ------------------------------------------------------------------ steps --
def wrap_text(text, fs, dpi, width_px):
	"""Greedy word wrap against MEASURED text widths (TextPath metrics), so a
	wrapped line can never overflow its panel."""
	words = str(text).split()
	lines, cur = [], ""
	for w in words:
		cand = w if not cur else cur + " " + w
		if cur and text_w_px(cand, fs, dpi) > width_px:
			lines.append(cur)
			cur = w
		else:
			cur = cand
	if cur:
		lines.append(cur)
	return lines or [""]


def layout_steps(steps, fs, dpi, wrap_w_px):
	"""(blocks, total_h, line_h, chip_h): wrapped lines + measured heights at
	font size fs. Line height 1.35; 10 px between steps; fastener chips on
	their own line."""
	line_h = 1.35 * fs * dpi / 72.0
	chip_h = 2.1 * STYLE["fs_table"] * dpi / 72.0  # rounded chip incl. its pad
	blocks, total = [], 0.0
	for s in sorted(steps, key=lambda s: s["order"]):
		lines = wrap_text(s["text"], fs, dpi, wrap_w_px)
		h = len(lines) * line_h + ((6.0 + chip_h) if s.get("fasteners") else 0.0)
		blocks.append((s, lines))
		total += h
	total += 10.0 * max(0, len(blocks) - 1)
	return blocks, total, line_h, chip_h


def steps_panel(fig, inner, steps, fs, dpi):
	"""Numbered build steps inside the ASSEMBLY SEQUENCE panel: drawn number
	chips (filled circle, white digit), a FIXED hanging indent (tab at
	chip + 14 px), measured wrapping, fastener callouts as bordered rounded
	chips on their own line at the tab."""
	ix, iy, iw, ih = inner
	chip_r = 8.0
	x_tab = ix + 2.0 * chip_r + 14.0
	blocks, total, line_h, chip_h = layout_steps(steps, fs, dpi, iw - (x_tab - ix))
	y = iy + ih  # cursor: top of the inner rect, moving down
	for s, lines in blocks:
		draw_balloon(fig, ix + chip_r, y - line_h / 2.0, s["order"], r=chip_r,
			face=STYLE["border"], edge=STYLE["border"], text_color="white", lw=0.0)
		for ln in lines:
			px_text(fig, x_tab, y - line_h / 2.0, ln, fs=fs, color=STYLE["ink"], va="center")
			y -= line_h
		if s.get("fasteners"):
			y -= 6.0
			px_text(fig, x_tab + 4.0, y - chip_h / 2.0, s["fasteners"], fs=STYLE["fs_table"],
				color=STYLE["ink"], va="center", z=9).set_bbox(
				{"boxstyle": "round,pad=0.45", "fc": STYLE["caption_fill"],
				 "ec": STYLE["border"], "lw": STYLE["border_pt"]})
			y -= chip_h
		y -= 10.0


# -------------------------------------------------------------------- BOM --
# Proportional columns: (label, left frac, right frac, align) — sums to 100%.
BOM_COLS = [
	("ITEM", 0.00, 0.07, "center"),
	("QTY", 0.07, 0.13, "center"),
	("NAME", 0.13, 0.45, "left"),
	("MATERIAL", 0.45, 0.58, "left"),
	("PART NO.", 0.58, 0.84, "left"),
	("MASS", 0.84, 1.00, "right"),
]


def bom_cell_x(inner, c0, c1, align):
	ix, _, iw, _ = inner
	if align == "left":
		return ix + c0 * iw + 6.0
	if align == "right":
		return ix + c1 * iw - 6.0
	return ix + (c0 + c1) / 2.0 * iw


def bom_panel(fig, inner, rows, row_h, dpi):
	"""Ruled BOM table: shaded bold header row, 0.4 pt row rules, drawn
	mini-balloons (r 7 px) in ITEM — visually identical to the exploded-view
	balloons — 6 px cell padding, right-aligned MASS, and a bold TOTAL row
	behind a 0.8 pt rule."""
	ix, iy, iw, ih = inner
	fs = STYLE["fs_table"]
	y_top = iy + ih
	# header row (shaded, bold)
	px_rect(fig, ix, y_top - row_h, iw, row_h, fill=STYLE["caption_fill"], z=-4)
	for label, c0, c1, align in BOM_COLS:
		px_text(fig, bom_cell_x(inner, c0, c1, align), y_top - row_h / 2.0, label,
			fs=fs, color=STYLE["ink"], ha=align, va="center", weight="bold")
	px_line(fig, ix, y_top - row_h, ix + iw, y_top - row_h, color=STYLE["border"], lw=STYLE["border_pt"], z=7)
	total_mass = 0.0
	for i, r in enumerate(rows):
		yc = y_top - (i + 1.5) * row_h
		# the TOTAL row is labeled "printed parts" — sum ONLY kind=made rows
		# (2026-07-19: a BOM with purchased masses filled in was silently
		# summing the motor into the "printed parts" total)
		if r["mass_g"] is not None and r.get("kind", "made") == "made":
			total_mass += r["mass_g"]
		vals = [None, str(r["qty"]), r["name"], r["material"], r["part_number"], fmt_mass(r["mass_g"])]
		for (label, c0, c1, align), v in zip(BOM_COLS, vals):
			if v is None:  # ITEM: the drawn mini-balloon, not a glyph
				draw_balloon(fig, bom_cell_x(inner, c0, c1, align), yc, r["item"], r=7.0, lw=0.9)
			else:
				px_text(fig, bom_cell_x(inner, c0, c1, align), yc, v, fs=fs,
					color=STYLE["ink"], ha=align, va="center")
		if i < len(rows) - 1:
			yr = y_top - (i + 2) * row_h
			px_line(fig, ix, yr, ix + iw, yr, color=STYLE["rule"], lw=STYLE["rule_pt"], z=7)
	# TOTAL row, separated by a strong rule
	y_rule = y_top - (len(rows) + 1) * row_h
	yc = y_rule - row_h / 2.0
	px_line(fig, ix, y_rule, ix + iw, y_rule, color=STYLE["border"], lw=STYLE["border_pt"], z=7)
	px_text(fig, bom_cell_x(inner, 0.13, 0.45, "left"), yc, "TOTAL (printed parts, est.)",
		fs=fs, color=STYLE["ink"], va="center", weight="bold")
	px_text(fig, bom_cell_x(inner, 0.84, 1.00, "right"), yc, fmt_mass(total_mass),
		fs=fs, color=STYLE["ink"], ha="right", va="center", weight="bold")


# ------------------------------------------------------------ title block --
def title_block(fig, x, y, w, h, fields):
	"""Engineering title block strip: uniform bordered cells, 6.5 pt uppercase
	label top-left in each cell, 9.5 pt value centered."""
	cells = [
		("PROJECT", fields["project"], 0.12, STYLE["ink"]),
		("DOC TITLE", fields["doc_title"], 0.32, STYLE["ink"]),
		("DATE", fields["date"], 0.10, STYLE["ink"]),
		("REV", fields["rev"], 0.06, STYLE["ink"]),
		("UNITS", "mm", 0.07, STYLE["ink"]),
		("SHEET", "1 / 1", 0.09, STYLE["ink"]),
		("GENERATED BY", "cadcode · LMCAD engine", 0.24, STYLE["ink2"]),
	]
	px_rect(fig, x, y, w, h, fill=STYLE["panel_fill"], z=-6)
	px_rect(fig, x, y, w, h, edge=STYLE["border"], lw=STYLE["border_pt"], z=5)
	cx = x
	for i, (label, value, frac, vcol) in enumerate(cells):
		cw = frac * w
		if i > 0:
			px_line(fig, cx, y, cx, y + h, color=STYLE["border"], lw=STYLE["border_pt"], z=6)
		px_text(fig, cx + 6.0, y + h - 6.0, label, fs=STYLE["fs_tb_label"],
			color=STYLE["ink2"], va="top")
		px_text(fig, cx + cw / 2.0, y + h * 0.38, value, fs=STYLE["fs_tb_value"],
			color=vcol, ha="center", va="center")
		cx += cw


# --------------------------------------------------------------- markdown --
def write_markdown(path, title, png, steps, bom_rows, fields):
	md = [f"# {title} — assembly instructions", "",
		f"Project {fields['project']} · rev {fields['rev']} · date {fields['date']} · units mm",
		f"Exploded/assembly sheet: `{os.path.basename(png)}`", "", "## Steps", ""]
	for s in sorted(steps, key=lambda s: s["order"]):
		line = f"{s['order']}. {s['text']}"
		if s.get("fasteners"):
			line += f" — **fasteners: {s['fasteners']}**"
		md.append(line)
	if bom_rows:
		md += ["", "## BOM (item numbers match the sheet balloons)", "",
			"| item | qty | name | material | part no. | mass |",
			"|---|---|---|---|---|---|"]
		md += [f"| {r['item']} | {r['qty']} | {r['name']} | {r['material']} | "
			f"{r['part_number']} | {fmt_mass(r['mass_g'])} |" for r in bom_rows]
	md.append("")
	with open(path, "w") as f:
		f.write("\n".join(md))


# ------------------------------------------------------------------ render --
def render(job):
	parts = job["parts"]
	if not parts:
		raise ValueError("job.parts must be non-empty")
	steps = job.get("steps") or []
	auto_steps = False
	if not steps:
		if not job.get("auto_steps"):
			raise ValueError("job.steps must be non-empty (this IS the instruction sheet) — or pass auto_steps: true for a DRAFT")
		# DRAFT steps (audit 2026-07-16): one per part in assembly order, from
		# the structure alone. Deliberately labeled a draft in the receipt —
		# the AI/human polishes wording; the numbers/masses come from the BOM.
		auto_steps = True
		steps = [{"order": 1, "text": f"Place the {parts[0]['name']} on the work surface in its shown orientation — it is the base every later part lands on."}]
		for i, p in enumerate(parts[1:], start=2):
			steps.append({"order": i, "text": f"Lower the {p['name']} onto the assembly along the explode axis until it seats fully against the previous parts. Check it sits square before continuing."})
	out_prefix = job["out_prefix"]
	elev, azim = parse_view(job.get("view"))
	max_px = int(job.get("max_px", 1800))
	if max_px < 1200:
		raise ValueError(f"max_px {max_px} is too small for the assembly-doc grid (min 1200)")

	names = [p["name"] for p in parts]
	tris = [load_stl(p["stl"]) for p in parts]
	colors = [matplotlib.colors.to_rgb(p["color"]) if p.get("color") else PALETTE[i % len(PALETTE)]
		for i, p in enumerate(parts)]
	axis, offsets = resolve_offsets(job, names, tris)
	exploded = [t + off[None, None, :] for t, off in zip(tris, offsets)]
	title = os.path.basename(out_prefix)
	fields = {
		"project": str(job.get("project", "cadcode")),
		"doc_title": str(job.get("doc_title", f"{title} — assembly")),
		"date": str(job.get("date", "—")),  # job field, NEVER the clock
		"rev": str(job.get("rev", "A")),
	}

	bom_rows, bom_from_csv = load_bom(job, parts)
	if len(bom_rows) > 24:
		raise ValueError(f"BOM has {len(bom_rows)} rows — more than fits one sheet (24); split the doc")
	item_of = resolve_items(names, bom_rows)

	# ---- page grid (all px, origin bottom-left; regions are disjoint) ----
	# Right-column split is MEASURED, never guessed: the sequence panel takes
	# exactly what its steps need at body 9 pt (the assembled view absorbs the
	# rest — dead-space doctrine — between 32% and 66% of the column); if the
	# steps outgrow that, the font steps down to 6 pt.
	#
	# singulator F12(b): an 8-step / 12-BOM-row job was then REFUSED outright —
	# "shorten the steps or split the doc". A page is not a fixed object: the
	# general defect was that the sheet height was a constant while its content
	# is not, so documentation-rich assemblies could not be documented at all and
	# the only workaround was DELETING engineering prose. The page now GROWS
	# (16:10 -> at most 16:`max_page_h_in`, default 20 = aspect 1.25) until the
	# measured layout fits, and only then refuses — with the height it reached.
	# A job that already fitted at 16:10 takes the first iteration and renders a
	# byte-identical PNG.
	fig_w = 16.0
	max_page_h = float(job.get("max_page_h_in", 20.0))
	if max_page_h < 10.0:
		raise ValueError(f"max_page_h_in {max_page_h} is below the 10.0 in base page height")
	mg, gt, pad = STYLE["margin"], STYLE["gutter"], STYLE["pad"]
	dpi = max_px / fig_w
	W = max_px
	content_w = W - 2.0 * mg
	tb_h = 44.0
	row_h = np.ceil(2.0 * STYLE["fs_table"] * dpi / 72.0)
	bom_h = caption_h(dpi) + 2.0 * pad + (len(bom_rows) + 2) * row_h  # header + rows + TOTAL
	left_w = round(0.55 * content_w)
	right_x = mg + left_w + gt
	right_w = content_w - left_w - gt
	seq_probe_w = right_w - 2.0 * pad - (2.0 * 8.0 + 14.0)
	seq_chrome = caption_h(dpi) + 2.0 * pad

	# The step layout depends only on (steps, fs, dpi, wrap width) — all constant
	# across page heights — so it is measured ONCE, and the height search is then
	# pure arithmetic (no figure is created until the winning height is known).
	# Lazily: measuring a step block is the dominant cost of the whole sheet, and
	# the overwhelmingly common case fits at the first size.
	_need_cache = {}

	def need_at(fs):
		if fs not in _need_cache:
			_need_cache[fs] = layout_steps(steps, fs, dpi, seq_probe_w)[1]
		return _need_cache[fs]

	needs = [(fs, need_at) for fs in (9.0, 8.5, 8.0, 7.5, 7.0, 6.5, 6.0)]
	fit, fig_h, grew, last_short = None, 10.0, False, None
	heights = [10.0]
	while heights[-1] + 0.5 <= max_page_h + 1e-9:
		heights.append(round(heights[-1] + 0.5, 6))
	for fig_h in heights:
		H = max_px * fig_h / fig_w
		band_y = H - mg - STYLE["header_h"]
		tb_y = mg
		bom_y = tb_y + tb_h + gt
		main_y = bom_y + bom_h + gt
		main_h = band_y - gt - main_y
		if main_h < 260.0:
			last_short = f"BOM ({len(bom_rows)} rows) leaves only {main_h:.0f} px for the views"
			continue
		for fs, _need_at in needs:
			asm_avail = main_h - gt - (_need_at(fs) + seq_chrome)
			if asm_avail >= 0.32 * main_h:
				fit = (fs, min(asm_avail, 0.66 * main_h))
				break
		if fit:
			grew = fig_h > 10.0
			break
		last_short = (f"steps do not fit the sequence panel even at 6 pt "
			f"({len(steps)} steps, {len(bom_rows)} BOM rows)")
	if not fit:
		raise ValueError(f"{last_short} — even at the maximum page height "
			f"{max_page_h:.1f} in; raise 'max_page_h_in' or split the doc")
	H = max_px * fig_h / fig_w
	fig = plt.figure(figsize=(fig_w, fig_h), dpi=dpi)
	fig.patch.set_facecolor(STYLE["page_fill"])
	page_frame(fig)
	band_y = header_band(fig, fields["doc_title"],
		f"{len(parts)} parts · {len(steps)} steps · units mm")
	steps_fs, asm_h = fit
	seq_h = main_h - asm_h - gt

	# 1) EXPLODED VIEW panel: fitted view, explode guides, balloon rail
	inner = panel(fig, mg, main_y, left_w, main_h, "EXPLODED VIEW", dpi)
	ax_e = fig.add_axes([inner[0] / W, inner[1] / H, inner[2] / W, inner[3] / H], projection="3d")
	shaded_view(ax_e, exploded, colors, elev, azim, np.zeros(3), 1.0)  # limits set by fit_view
	# reserve a right rail zone (balloon + leader run) out of the fit rect
	rail_zone = 2.0 * 9.0 + 30.0
	fit_pts = np.concatenate([t.reshape(-1, 3) for t in exploded])
	if len(fit_pts) > 30000:
		fit_pts = fit_pts[:: int(np.ceil(len(fit_pts) / 30000.0))]
	er = float((fit_pts.max(axis=0) - fit_pts.min(axis=0)).max()) / 2.0
	fit_view(ax_e, fit_pts, elev, azim,
		(inner[0], inner[1], inner[2] - rail_zone, inner[3]), fill=0.88)
	explode_guides(ax_e, axis, exploded, offsets, er)
	draw_balloons(fig, ax_e, exploded, names, item_of, inner)

	# 2) ASSEMBLED panel: fitted view + scale bar + overall dims inside it
	asm_x, asm_y = right_x, main_y + main_h - asm_h
	inner_a = panel(fig, asm_x, asm_y, right_w, asm_h, "ASSEMBLED", dpi)
	ax_a = fig.add_axes([inner_a[0] / W, inner_a[1] / H, inner_a[2] / W, inner_a[3] / H], projection="3d")
	shaded_view(ax_a, tris, colors, elev, azim, np.zeros(3), 1.0)
	foot_h = 44.0  # scale bar + dims line zone, inside the panel
	fit_pts_a = np.concatenate([t.reshape(-1, 3) for t in tris])
	if len(fit_pts_a) > 30000:
		fit_pts_a = fit_pts_a[:: int(np.ceil(len(fit_pts_a) / 30000.0))]
	px_per_mm = fit_view(ax_a, fit_pts_a, elev, azim,
		(inner_a[0], inner_a[1] + foot_h, inner_a[2], inner_a[3] - foot_h), fill=0.85)
	alo, ahi = fit_pts_a.min(axis=0), fit_pts_a.max(axis=0)
	ext = ahi - alo
	body_y = inner_a[1] - pad  # the panel's bottom border
	cx = inner_a[0] + inner_a[2] / 2.0
	px_text(fig, cx, body_y + 8.0, f"overall {ext[0]:.0f} × {ext[1]:.0f} × {ext[2]:.0f} mm (W × D × H)",
		fs=STYLE["fs_table"], color=STYLE["ink"], ha="center", va="baseline", z=9)
	nice = [1, 2, 5, 10, 20, 25, 50, 100, 200, 500]
	L = max([n for n in nice if n * px_per_mm <= 0.35 * inner_a[2]] or [1])
	half = L * px_per_mm / 2.0
	bar_y = body_y + 30.0
	px_line(fig, cx - half, bar_y, cx + half, bar_y, color=STYLE["ink"], lw=1.1, z=9)
	for xt in (cx - half, cx + half):
		px_line(fig, xt, bar_y - 4.0, xt, bar_y + 4.0, color=STYLE["ink"], lw=1.1, z=9)
	px_text(fig, cx, bar_y + 7.0, f"{L} mm", fs=STYLE["fs_table"], color=STYLE["ink"],
		ha="center", va="baseline", z=9)

	# 3) ASSEMBLY SEQUENCE panel (pre-measured font + split)
	inner_s = panel(fig, right_x, main_y, right_w, seq_h, "ASSEMBLY SEQUENCE", dpi)
	steps_panel(fig, inner_s, steps, steps_fs, dpi)

	# 4) BILL OF MATERIALS panel + title block strip
	inner_b = panel(fig, mg, bom_y, content_w, bom_h, "BILL OF MATERIALS", dpi)
	bom_panel(fig, inner_b, bom_rows, row_h, dpi)
	title_block(fig, mg, tb_y, content_w, tb_h, fields)

	png = f"{out_prefix}_assembly_doc.png"
	os.makedirs(os.path.dirname(os.path.abspath(png)), exist_ok=True)
	fig.savefig(png, dpi=dpi, facecolor=STYLE["page_fill"])
	plt.close(fig)

	md = f"{out_prefix}_instructions.md"
	write_markdown(md, title, png, steps, bom_rows, fields)

	with open(png, "rb") as f:
		head = f.read(24)
	w, h = int.from_bytes(head[16:20], "big"), int.from_bytes(head[20:24], "big")
	return {"ok": True, "png": os.path.abspath(png), "md": os.path.abspath(md), "px": [w, h],
		"parts": len(parts), "steps": len(steps), "auto_steps": auto_steps, "bom_rows": len(bom_rows),
		"bom_from_csv": bom_from_csv, "date": fields["date"], "rev": fields["rev"],
		"explode_axis": [round(float(a), 6) for a in axis],
		"page_h_in": round(float(fig_h), 3), "page_grew": bool(grew), "steps_fs": float(steps_fs)}


def main():
	if len(sys.argv) < 2 or sys.argv[1] in ("-h", "--help"):
		print(__doc__)
		return 0
	if len(sys.argv) != 2:
		print(json.dumps({"ok": False, "error": "usage: assembly_doc.py job.json"}))
		return 1
	try:
		job = json.load(open(sys.argv[1]))
		for key in ("parts", "explode", "out_prefix"):
			if key not in job:
				raise ValueError(f"job needs '{key}'")
		if "steps" not in job and not job.get("auto_steps"):
			raise ValueError("job needs 'steps' (or auto_steps: true for a structure-derived draft)")
		receipt = render(job)
	except Exception as e:  # noqa: BLE001 — the receipt IS the error channel
		print(json.dumps({"ok": False, "error": f"{type(e).__name__}: {e}"}))
		return 1
	print(json.dumps(receipt))
	return 0


if __name__ == "__main__":
	sys.exit(main())
