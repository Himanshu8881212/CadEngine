#!/usr/bin/env python3
"""analysis_sheet.py — the ANALYSIS twin of render_sheet.py: one production-grade
sheet that shows how the part PERFORMED, not just what it looks like. DOMAIN
AGNOSTIC: structural FEA, acoustics, kinematics, aero — any analysis renders
through the same typed panels.

Usage: analysis_sheet.py job.json

MODERN FORM — {title, meta_note?, panels:[...], results, gates, date, out}:
  panel {"kind":"view",  caption, stl, loads:[{at,dir,label?}], fixture:{label?},
         elev?, azim?}                      — load-case / part view
                                              (`loads[].label` is OPTIONAL, like
                                              `fixture.label`; `at`/`dir` are
                                              required and are named in the
                                              refusal when they are missing)
  panel {"kind":"field", caption, stl, npy, origin_mm, voxel_mm, cmap, unit,
         field_unit? (the unit the .npy is ACTUALLY in — see UNITS below),
         scale? (multiply raw field), vmax?, hotspot? (bool), elev?, azim?}
                                            — any voxel field mapped to the surface
  panel {"kind":"curve", caption, series:[{x,y,label?}], xlabel, ylabel,
         logx?, targets:[{y?|x?, label}], ylim?}
                                            — any response curve (SPL, transmission
                                              error, torque ripple, stiffness sweep)
  panel {"kind":"image", caption, png}      — a domain tool's OWN plot (polar map,
                                              flow field, B-field...), framed with
                                              the same receipts + gates
  Up to 3 panels per row; 1400 px tall page when there are two rows.
  results: dict (ordered) or [[label, value], ...] — verbatim receipts table
  gates:   {name: bool} chips

LEGACY FORM (structural, auto-converted): the original keys below.
Job keys:
  title        header title (e.g. "VELA-68 — structural analysis")
  stl          the ANALYZED geometry (the mesh the FEA grid was sampled from)
  stress_npy   von Mises field (nx,ny,nz) Pa — ace_fea_runner's stress_field.npy
  disp_npy     displacement magnitude field (nx,ny,nz) m — disp_field.npy
  origin_mm, voxel_mm   the FEA grid frame (same values as the fea job)
  case         one-sentence load-case description (caption of panel 1)
  loads        [{label, at:[x,y,z], dir:[x,y,z]}] drawn as arrows INTO the part
  fixture      {label} — the clamped desk plane z=0 is drawn with ground ticks
  results      {max_von_mises_mpa, allowable_mpa, safety_factor, max_disp_mm,
                mesh:"...", note:"..."} — the receipts table, verbatim numbers
  gates        {name: bool} rendered as PASS/FAIL chips
  elev, azim   camera for all three views (default 16, -62)
  date         stamped in the header (never read from the clock)
  out          output PNG

UNITS — `unit` used to be DECORATIVE (cubesat F8: a Pa field labelled "MPa" drew a
colour bar reading `1.92e+07 MPa`, i.e. every stress on the sheet overstated 1e6×
while looking self-consistent). The voxel runners emit SI (Pa, m, K/°C); `scale`
was the only thing that made `unit` true, and nothing checked the two agreed.

Now: declare `field_unit` (what the .npy holds) alongside `unit` (what the colour
bar says) and the CONVERSION IS COMPUTED and cross-checked —
  * unknown unit, or a dimension mismatch (`"m"` -> `"MPa"`)  -> REFUSED;
  * `field_unit` + a `scale` that disagrees with the computed factor -> REFUSED
    (`unit_scale_conflict`), never silently preferring one;
  * temperature is an OFFSET relation, so K -> °C is refused rather than scaled.
`field_unit` is optional and the default is exactly the old behaviour (`scale`,
default 1.0) — but then the receipt carries a `warnings` entry saying the label
was never verified, because an unchecked label is a claim without a receipt.
Every field panel's `{unit, field_unit, scale, vmax}` lands in the receipt.

Design: same STYLE/palette/chrome as render_sheet.py (imported — one system).
Field-to-surface mapping: each triangle is colored by the MAX of the field in the
2x2x2 voxel neighborhood of its centroid (the outermost half-voxel shell of a
hex8 grid is partially filled; a plain nearest-voxel lookup speckles to zero).
The stress hotspot voxel is marked on the part with its value.
"""
import json, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.patches import FancyArrowPatch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # tools/: the shared contracts + the layout map
import _layout  # noqa: E402
_layout.add_import_paths()  # tools/, tools/analyzers, tools/publish — sibling-style imports keep working after the 2026-09-02 move
import render_sheet as rs

DPI = 100.0

# unit -> (dimension, multiplicative factor to the dimension's SI base).
# Deliberately small and explicit: an unknown unit is REFUSED, never guessed.
UNITS = {
	"Pa": ("pressure", 1.0), "kPa": ("pressure", 1e3), "MPa": ("pressure", 1e6),
	"GPa": ("pressure", 1e9), "N/m^2": ("pressure", 1.0), "N/mm^2": ("pressure", 1e6),
	"bar": ("pressure", 1e5), "psi": ("pressure", 6894.757293168361),
	"m": ("length", 1.0), "cm": ("length", 1e-2), "mm": ("length", 1e-3),
	"um": ("length", 1e-6), "µm": ("length", 1e-6), "nm": ("length", 1e-9),
	"in": ("length", 0.0254),
	"N": ("force", 1.0), "kN": ("force", 1e3), "mN": ("force", 1e-3),
	"N.m": ("torque", 1.0), "N.mm": ("torque", 1e-3),
	"kg": ("mass", 1.0), "g": ("mass", 1e-3),
	"Hz": ("frequency", 1.0), "kHz": ("frequency", 1e3),
	"W": ("power", 1.0), "mW": ("power", 1e-3), "kW": ("power", 1e3),
	"K": ("temperature", 1.0), "degC": ("temperature", 1.0),
	"C": ("temperature", 1.0), "°C": ("temperature", 1.0),
	"": ("dimensionless", 1.0), "-": ("dimensionless", 1.0), "1": ("dimensionless", 1.0),
	"%": ("dimensionless", 0.01),
}


def convert_factor(field_unit, display_unit):
	"""Multiplier taking a value in `field_unit` to `display_unit`.

	Refuses (never guesses) an unknown unit, a dimension mismatch, or a
	temperature change of unit — K -> degC is an OFFSET, not a scale, and a
	sheet that silently multiplied by 1.0 there would be wrong by 273.15."""
	for u in (field_unit, display_unit):
		if u not in UNITS:
			raise ValueError(f"unknown unit {u!r} — known units: {sorted(k for k in UNITS if k)}")
	(df, ff), (dd, fd) = UNITS[field_unit], UNITS[display_unit]
	if df != dd:
		raise ValueError(f"unit dimension mismatch: field_unit {field_unit!r} is {df}, "
		                 f"unit {display_unit!r} is {dd} — these are not convertible")
	if df == "temperature" and field_unit != display_unit:
		raise ValueError(f"temperature conversion {field_unit!r} -> {display_unit!r} is an OFFSET, "
		                 f"not a scale; emit the field in the unit you label it with")
	return ff / fd


def panel_scale(spec, i, warnings):
	"""The multiplier applied to a field panel's raw .npy, with its provenance.

	Returns (scale, field_unit_or_None). `field_unit` present -> the factor is
	COMPUTED and cross-checked against any explicit `scale`; absent -> today's
	semantics (`scale`, default 1.0) plus an honest warning that nothing verified
	the `unit` label."""
	display = str(spec.get("unit", ""))
	fu = spec.get("field_unit")
	explicit = spec.get("scale")
	if fu is None:
		if explicit is None and display not in ("", "-", "1"):
			warnings.append(f"panel {i}: 'unit' {display!r} is a LABEL only — no 'field_unit' and no "
			                f"'scale' declared, so the colour bar is unverified (voxel runners emit SI)")
		return (1.0 if explicit is None else float(explicit)), None
	factor = convert_factor(str(fu), display)
	if explicit is not None:
		s = float(explicit)
		if factor == 0.0 or abs(s - factor) > 1e-9 * max(abs(factor), 1.0):
			raise ValueError(f"panel {i}: unit_scale_conflict — field_unit {fu!r} -> unit {display!r} "
			                 f"is ×{factor!r}, but the job says scale ×{s!r}; drop one of them")
	return factor, str(fu)


def sample_surface(tris, field, origin, voxel):
	"""Per-triangle field value: neighborhood max around each centroid voxel."""
	cen = tris.mean(axis=1)
	idx = np.floor((cen - origin) / voxel).astype(int)
	nx, ny, nz = field.shape
	vals = np.zeros(len(tris))
	for dx in (0, -1):
		for dy in (0, -1):
			for dz in (0, -1):
				i = np.clip(idx[:, 0] + dx, 0, nx - 1)
				j = np.clip(idx[:, 1] + dy, 0, ny - 1)
				k = np.clip(idx[:, 2] + dz, 0, nz - 1)
				vals = np.maximum(vals, field[i, j, k])
	return vals


def field_view(fig, rect, tris, vals, vmax, elev, azim, cmap_name, bar_label, hotspot=None):
	"""Shaded 3D view colored by `vals`, with an in-panel colorbar when a
	bar_label is given (the plain load-case view passes none)."""
	W, H = rs.fig_px(fig)
	bar_h = 30.0 if bar_label else 0.0
	view = (rect[0], rect[1] + bar_h + (8.0 if bar_label else 0.0), rect[2], rect[3] - bar_h - (8.0 if bar_label else 0.0))
	ax = fig.add_axes((0, 0, 1, 1), projection="3d", computed_zorder=False)
	ax.set_position((view[0] / W, view[1] / H, view[2] / W, view[3] / H))
	ax.set_proj_type("ortho")
	ax.view_init(elev=elev, azim=azim)
	ax.set_axis_off()
	ax.patch.set_alpha(0.0)
	pts = tris.reshape(-1, 3)
	rs.fit_view(ax, pts, elev, azim, view, fill=0.9)
	light = rs.camera_dir(elev + 28.0, azim - 22.0)
	lam = 0.55 + 0.45 * np.clip(np.abs(rs.tri_normals(tris) @ light), 0.0, 1.0)
	cmap = matplotlib.colormaps[cmap_name]
	base = cmap(np.clip(vals / max(vmax, 1e-30), 0.0, 1.0))[:, :3]
	colors = np.clip(base * lam[:, None], 0.0, 1.0)
	from mpl_toolkits.mplot3d.art3d import Poly3DCollection
	pc = Poly3DCollection(tris, facecolors=colors, edgecolors="none", zsort="average")
	ax.add_collection3d(pc)
	# in-panel horizontal colorbar, drawn with px chrome (deterministic)
	if not bar_label:
		return ax
	bx, by, bw = rect[0] + 4.0, rect[1], rect[2] - 8.0
	n = 128
	for i in range(n):
		c = cmap((i + 0.5) / n)
		rs.px_rect(fig, bx + bw * i / n, by + 12.0, bw / n + 0.6, bar_h - 16.0, fill=c, z=6)
	rs.px_rect(fig, bx, by + 12.0, bw, bar_h - 16.0, edge=rs.STYLE["border"], lw=0.6, z=7)
	rs.px_text(fig, bx, by, "0", rs.STYLE["fs_table"], rs.STYLE["ink2"], va="baseline", z=8)
	rs.px_text(fig, bx + bw, by, f"{vmax:.3g} {bar_label}", rs.STYLE["fs_table"], rs.STYLE["ink2"],
	           ha="right", va="baseline", z=8)
	if hotspot is not None:
		p = rs.project_px(ax, np.asarray(hotspot["at"])[None, :])[0]
		rs.draw_balloon(fig, p[0], p[1], "!", r=7.5, face="#ffffff", z=32)
		rs.px_text(fig, p[0] + 11.0, p[1] + 8.0, hotspot["label"], rs.STYLE["fs_table"],
		           rs.STYLE["ink"], weight="bold", z=32)
	return ax


CURVE_COLORS = [(0.361, 0.478, 0.596), (0.718, 0.514, 0.365), (0.494, 0.588, 0.463),
                (0.553, 0.471, 0.580), (0.427, 0.443, 0.482)]


def view_panel(fig, rect, spec, tris):
	elev, azim = float(spec.get("elev", 16.0)), float(spec.get("azim", -62.0))
	ax = field_view(fig, rect, tris, np.zeros(len(tris)), 1.0, elev, azim, "Greys", "")
	light = rs.camera_dir(elev + 28.0, azim - 22.0)
	lam = 0.55 + 0.45 * np.clip(np.abs(rs.tri_normals(tris) @ light), 0.0, 1.0)
	from mpl_toolkits.mplot3d.art3d import Poly3DCollection
	for coll in list(ax.collections):
		coll.remove()
	ax.add_collection3d(Poly3DCollection(tris, facecolors=np.clip(
		np.array(rs.SINGLE_COLOR)[None, :] * lam[:, None], 0, 1), edgecolors="none", zsort="average"))
	W, H = rs.fig_px(fig)
	pts = tris.reshape(-1, 3)
	zmin = pts[:, 2].min()
	if spec.get("fixture"):
		P = rs.project_px(ax, pts[pts[:, 2] < zmin + 1.0][::7])
		if len(P):
			gy = P[:, 1].min() - 6.0
			x0, x1 = P[:, 0].min() - 10.0, P[:, 0].max() + 10.0
			rs.px_line(fig, x0, gy, x1, gy, rs.STYLE["border"], 1.2, z=20)
			for xx in np.arange(x0, x1, 14.0):
				rs.px_line(fig, xx, gy - 8.0, xx + 8.0, gy, rs.STYLE["border"], 0.8, z=20)
			rs.px_text(fig, x1 + 6.0, gy - 4.0, spec["fixture"].get("label", "clamped"),
			           rs.STYLE["fs_table"], rs.STYLE["ink2"], z=20)
	for li, ld in enumerate(spec.get("loads", [])):
		at = np.asarray(ld["at"], dtype=np.float64)
		d = np.asarray(ld["dir"], dtype=np.float64)
		n = float(np.linalg.norm(d))
		if n < 1e-12:
			raise ValueError(f"panel {spec.get('_index','?')} load {li}: 'dir' is a zero vector")
		d = d / n
		tail, tip = rs.project_px(ax, np.vstack([at - 34.0 * d, at]))
		fig.add_artist(FancyArrowPatch((tail[0] / W, tail[1] / H), (tip[0] / W, tip[1] / H),
			transform=fig.transFigure, color="#a23b2e", linewidth=2.0,
			arrowstyle="-|>", mutation_scale=16, zorder=30))
		label = ld.get("label")            # OPTIONAL, exactly like fixture.label
		if not label:
			continue
		label = str(label)
		lw_px = rs.text_w_px(label, rs.STYLE["fs_table"], DPI, "bold")
		lx = tail[0] - lw_px - 6.0 if tip[0] >= tail[0] else tail[0] + 6.0
		rs.px_text(fig, lx, tail[1] + 6.0, label, rs.STYLE["fs_table"],
		           "#a23b2e", weight="bold", z=30)


def curve_panel(fig, rect, spec):
	"""A response-curve panel in the design system (any domain: SPL, transmission
	error, torque ripple, stiffness sweep, polar...)."""
	W, H = rs.fig_px(fig)
	ax = fig.add_axes((rect[0] / W, (rect[1] + 26.0) / H, rect[2] / W, (rect[3] - 34.0) / H))
	ax.set_facecolor(rs.STYLE["panel_fill"])
	for sp in ax.spines.values():
		sp.set_color(rs.STYLE["border"]); sp.set_linewidth(0.6)
	ax.tick_params(labelsize=rs.STYLE["fs_table"] - 0.5, colors=rs.STYLE["ink2"], width=0.5, length=3)
	ax.grid(True, color=rs.STYLE["rule"], linewidth=rs.STYLE["rule_pt"], alpha=0.9)
	if spec.get("logx"):
		ax.set_xscale("log")
	for i, ser in enumerate(spec.get("series", [])):
		ax.plot(ser["x"], ser["y"], color=CURVE_COLORS[i % len(CURVE_COLORS)],
		        linewidth=1.5, label=ser.get("label"))
	for tg in spec.get("targets", []):
		if "y" in tg:
			ax.axhline(tg["y"], color="#a23b2e", linewidth=0.9, linestyle="--")
			ax.annotate(tg.get("label", ""), xy=(0.99, tg["y"]), xycoords=("axes fraction", "data"),
			            fontsize=rs.STYLE["fs_table"] - 1, color="#a23b2e", ha="right", va="bottom",
			            family=rs.STYLE["font"])
		if "x" in tg:
			ax.axvline(tg["x"], color="#a23b2e", linewidth=0.9, linestyle="--")
	if spec.get("ylim"):
		ax.set_ylim(spec["ylim"])
	ax.set_xlabel(spec.get("xlabel", ""), fontsize=rs.STYLE["fs_table"], color=rs.STYLE["ink"], family=rs.STYLE["font"])
	ax.set_ylabel(spec.get("ylabel", ""), fontsize=rs.STYLE["fs_table"], color=rs.STYLE["ink"], family=rs.STYLE["font"])
	for lb in ax.get_xticklabels() + ax.get_yticklabels():
		lb.set_family(rs.STYLE["font"])
	if any(ser.get("label") for ser in spec.get("series", [])):
		lg = ax.legend(fontsize=rs.STYLE["fs_table"] - 0.5, framealpha=0.95, edgecolor=rs.STYLE["rule"],
		               prop={"family": rs.STYLE["font"], "size": rs.STYLE["fs_table"] - 0.5})
		lg.get_frame().set_linewidth(0.5)


def render_field_panel(fig, rect, spec, scale=None):
	tris = rs.load_stl(spec["stl"])
	if len(tris) > rs.MAX_SHADED_TRIS:
		tris = tris[:: int(np.ceil(len(tris) / rs.MAX_SHADED_TRIS))]
	if scale is None:
		scale = float(spec.get("scale", 1.0))
	field = np.load(spec["npy"]).astype(np.float64) * float(scale)
	vals = sample_surface(tris, field, np.asarray(spec["origin_mm"], dtype=np.float64),
	                      float(spec["voxel_mm"]))
	vmax = float(spec.get("vmax", field.max()))
	hs = None
	if spec.get("hotspot"):
		hi = np.unravel_index(int(np.argmax(field)), field.shape)
		at = np.asarray(spec["origin_mm"]) + (np.asarray(hi) + 0.5) * float(spec["voxel_mm"])
		hs = {"at": at, "label": f"max {vmax:.3g} {spec.get('unit','')}"}
	field_view(fig, rect, tris, vals, vmax, float(spec.get("elev", 16.0)),
	           float(spec.get("azim", -62.0)), spec.get("cmap", "turbo"), spec.get("unit", ""), hotspot=hs)


def truncate_caption(cap, pw):
	while rs.text_w_px(rs.tracked(cap), rs.STYLE["fs_caption"], DPI, "bold") > pw - 2 * rs.STYLE["cap_pad"] and len(cap) > 8:
		cap = cap[:-1]
	return cap


REQUIRED_BY_KIND = {
	"view": ("stl",),
	"field": ("stl", "npy", "origin_mm", "voxel_mm"),
	"curve": ("series",),
	"image": ("png",),
}


def validate_job(job):
	"""Refuse a malformed job with a message that NAMES the panel and the key.

	cubesat F7: a load with no `label` died on a bare `KeyError: 'label'` from
	inside the renderer — no panel index, no job name, no receipt. The general
	defect is that nothing checked the job before drawing it; the general fix is
	one up-front pass that refuses by name."""
	if not isinstance(job.get("panels"), list) or not job["panels"]:
		raise ValueError("job.panels must be a non-empty list (or use the legacy structural form)")
	if "out" not in job:
		raise ValueError("job.out (output PNG path) is required")
	if "results" not in job:
		raise ValueError("job.results is required (dict or [[label, value], ...])")
	for i, spec in enumerate(job["panels"]):
		if not isinstance(spec, dict) or "kind" not in spec:
			raise ValueError(f"panel {i}: must be an object with a 'kind'")
		kind = spec["kind"]
		if kind not in REQUIRED_BY_KIND:
			raise ValueError(f"panel {i}: unknown panel kind {kind!r} — "
			                 f"known kinds: {sorted(REQUIRED_BY_KIND)}")
		for key in REQUIRED_BY_KIND[kind]:
			if key not in spec:
				raise ValueError(f"panel {i} (kind {kind!r}): missing required key {key!r}")
		spec["_index"] = i
		if kind == "view":
			for li, ld in enumerate(spec.get("loads") or []):
				if not isinstance(ld, dict):
					raise ValueError(f"panel {i} load {li}: must be an object {{at, dir, label?}}")
				for key in ("at", "dir"):
					if key not in ld:
						raise ValueError(f"panel {i} load {li}: missing required key {key!r} "
						                 f"(only 'label' is optional)")


def main():
	job = json.load(open(sys.argv[1]))
	if "panels" not in job:                                   # legacy structural form
		res = job["results"]
		job["panels"] = [
			{"kind": "view", "caption": f"load case — {job.get('case','')}", "stl": job["stl"],
			 "loads": job.get("loads", []), "fixture": job.get("fixture"),
			 "elev": job.get("elev", 16.0), "azim": job.get("azim", -62.0)},
			{"kind": "field", "caption": "von Mises stress — surface map", "stl": job["stl"],
			 "npy": job["stress_npy"], "origin_mm": job["origin_mm"], "voxel_mm": job["voxel_mm"],
			 # ace_fea's stress_field.npy is SI (Pa); declaring BOTH makes the pair
			 # self-checking — a drifted scale is now a refusal, not a 1e6 error.
			 "cmap": "turbo", "unit": "MPa", "field_unit": "Pa", "scale": 1e-6,
			 "vmax": res["max_von_mises_mpa"],
			 "hotspot": True, "elev": job.get("elev", 16.0), "azim": job.get("azim", -62.0)},
			{"kind": "field", "caption": "displacement — surface map", "stl": job["stl"],
			 "npy": job["disp_npy"], "origin_mm": job["origin_mm"], "voxel_mm": job["voxel_mm"],
			 "cmap": "viridis", "unit": "mm", "field_unit": "m", "scale": 1000.0,
			 "vmax": res["max_disp_mm"],
			 "elev": job.get("elev", 16.0), "azim": job.get("azim", -62.0)}]
		job["results"] = [["max von Mises", f"{res['max_von_mises_mpa']:.2f} MPa"],
		                  ["allowable (derated)", f"{res['allowable_mpa']:.1f} MPa"],
		                  ["safety factor", f"{res['safety_factor']:.1f}"],
		                  ["max displacement", f"{res['max_disp_mm']:.3f} mm"],
		                  ["mesh", res.get("mesh", "")], ["honesty", res.get("note", "")]]
		job.setdefault("meta_note",
		               f"hex8 voxel FEA · {job.get('voxel_mm','?')} mm grid")

	if "title" not in job:
		raise ValueError("job.title is required")
	validate_job(job)
	panels = job["panels"]
	# units resolved BEFORE anything is drawn: a refusal must not leave a half-written PNG
	warnings, field_report = [], []
	for spec in panels:
		if spec["kind"] != "field":
			continue
		sc, fu = panel_scale(spec, spec["_index"], warnings)
		spec["_scale"] = sc
		field_report.append({"panel": spec["_index"], "caption": spec.get("caption", ""),
		                     "unit": str(spec.get("unit", "")), "field_unit": fu, "scale": sc})
	rows_n = 1 if len(panels) <= 3 else 2
	per_row = int(np.ceil(len(panels) / rows_n))
	Wpx = 1600.0
	Hpx = 1000.0 if rows_n == 1 else 1400.0
	fig = plt.figure(figsize=(Wpx / DPI, Hpx / DPI), dpi=DPI)
	fig.patch.set_facecolor(rs.STYLE["page_fill"])
	rs.page_frame(fig)
	meta = f"{job.get('meta_note','')} · {job.get('date','')} · cadcode analysis sheet"
	y_rule = rs.header_band(fig, job["title"], meta)

	m, g = rs.STYLE["margin"], rs.STYLE["gutter"]
	table_h = 210.0
	grid_y = m + table_h + g
	grid_h = y_rule - g - grid_y
	row_h = (grid_h - (rows_n - 1) * g) / rows_n
	pw = (Wpx - 2 * m - (per_row - 1) * g) / per_row
	for i, spec in enumerate(panels):
		row, col = i // per_row, i % per_row
		x = m + col * (pw + g)
		y = grid_y + (rows_n - 1 - row) * (row_h + g)
		rect = rs.panel(fig, x, y, pw, row_h, truncate_caption(spec.get("caption", spec["kind"]), pw), DPI)
		if spec["kind"] == "view":
			tris = rs.load_stl(spec["stl"])
			if len(tris) > rs.MAX_SHADED_TRIS:
				tris = tris[:: int(np.ceil(len(tris) / rs.MAX_SHADED_TRIS))]
			view_panel(fig, rect, spec, tris)
		elif spec["kind"] == "field":
			render_field_panel(fig, rect, spec, scale=spec["_scale"])
		elif spec["kind"] == "curve":
			curve_panel(fig, rect, spec)
		elif spec["kind"] == "image":
			W, H = rs.fig_px(fig)
			img = plt.imread(spec["png"])
			ih, iw = img.shape[0], img.shape[1]
			sc = min(rect[2] / iw, rect[3] / ih)
			dw, dh = iw * sc, ih * sc
			axi = fig.add_axes(((rect[0] + (rect[2] - dw) / 2) / W, (rect[1] + (rect[3] - dh) / 2) / H,
			                    dw / W, dh / H))
			axi.imshow(img)
			axi.set_axis_off()
		else:
			raise ValueError(f"unknown panel kind {spec['kind']!r}")

	rows = list(job["results"].items()) if isinstance(job["results"], dict) else [tuple(r) for r in job["results"]]
	tw = (Wpx - 2 * m - g) * 0.60
	rt = rs.panel(fig, m, m, tw, table_h, "results — verbatim receipts", DPI)
	rh = rt[3] / max(1, len(rows))
	for i, (k, v) in enumerate(rows):
		yy = rt[1] + rt[3] - (i + 1) * rh
		if i:
			rs.px_line(fig, rt[0], yy + rh, rt[0] + rt[2], yy + rh, rs.STYLE["rule"], rs.STYLE["rule_pt"])
		rs.px_text(fig, rt[0] + 2.0, yy + rh * 0.32, str(k).upper(), rs.STYLE["fs_tb_label"], rs.STYLE["ink2"])
		rs.px_text(fig, rt[0] + 170.0, yy + rh * 0.32, str(v), rs.STYLE["fs_table"], rs.STYLE["ink"])
	rg = rs.panel(fig, m + tw + g, m, Wpx - 2 * m - g - tw, table_h, "gates", DPI)
	gates = job.get("gates", {})
	cw, chh = (rg[2]) / 2.0, rg[3] / max(1, (len(gates) + 1) // 2)
	for i, (k, ok) in enumerate(gates.items()):
		col, rrow = i % 2, i // 2
		xx = rg[0] + col * cw
		yy = rg[1] + rg[3] - (rrow + 1) * chh
		c = "#2e7d43" if ok else "#a23b2e"
		rs.draw_balloon(fig, xx + 8.0, yy + chh / 2.0, "P" if ok else "F", r=7.0,
		                face="white", edge=c, text_color=c)
		rs.px_text(fig, xx + 20.0, yy + chh / 2.0 - 3.0, k, rs.STYLE["fs_table"], rs.STYLE["ink"], z=9)
	fig.savefig(job["out"], dpi=DPI, facecolor=fig.get_facecolor())
	receipt = {"ok": True, "out": os.path.abspath(job["out"]), "panels": len(panels)}
	if field_report:
		receipt["fields"] = field_report
	if warnings:
		receipt["warnings"] = warnings
	return receipt


def cli(argv):
	if len(argv) < 2 or argv[1] in ("-h", "--help"):
		print(__doc__)
		return 0
	if len(argv) != 2:
		print(json.dumps({"ok": False, "error": "usage: analysis_sheet.py job.json"}))
		return 1
	try:
		receipt = main()
	except Exception as e:  # noqa: BLE001 — the receipt IS the error channel
		# cubesat F7: this used to be a bare traceback with NO receipt line, which a
		# shell gate keyed on the last stdout line could not see at all.
		print(json.dumps({"ok": False, "error": f"{type(e).__name__}: {e}"}))
		return 1
	print(json.dumps(receipt))
	return 0


if __name__ == "__main__":
	sys.exit(cli(sys.argv))
