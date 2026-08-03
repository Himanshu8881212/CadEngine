#!/usr/bin/env python3
"""production_dossier.py — BOM cost rollup + FDM print-plate packing.

The dossier emitter: takes a parts list (MADE parts as STLs + BUY lines),
prices every line, estimates print time with a stated heuristic, packs the
made parts onto printer plates (2D first-fit-decreasing on footprint bboxes),
and writes bom_dossier.json + bom_dossier.csv to out_dir.

Usage:  python3 production_dossier.py job.json

Job JSON (argv[1]) — all lengths mm, masses g, densities kg/m^3:
    out_dir              REQUIRED  directory for bom_dossier.json/.csv
    parts                REQUIRED  list of lines, each ONE of:
      MADE: {name, stl (path), material ("petg"...), qty (default 1),
             material_required: false (true = the material is a FUNCTIONAL
             requirement, e.g. spring fingers — carried into csv/instructions),
             print_notes: "" (orientation/strength notes, carried likewise),
             buy: false (default), wear: false (optional serviceability flag),
             print_params: {...} (optional per-part overrides, see below)}
      BUY:  {name, buy: true, qty (default 1), part_number?, unit_price?,
             source?, wear?}
    bed                  optional  {x:220, y:220, z:250} printable volume (mm)
    density_kg_m3        optional  {material: density} inline overrides;
                                   tools/material_db.json is read first when it
                                   exists, inline values win. A material with
                                   no density from either source is refused.
    print_params         optional  job-wide slicer-heuristic overrides; keys
                                   {perimeters, line_width, layer_h,
                                    top_bottom_layers, infill}; per-part
                                   print_params win over these, which win over
                                   the defaults {perimeters:3, line_width:0.45,
                                   layer_h:0.2, top_bottom_layers:4, infill:0.20}
    filament_price_per_kg optional  default 25 (same currency as unit_price)
    print_speed_factor   optional  default 1.0, multiplies the time heuristic
    spacing_mm           optional  default 5 — gap between parts AND to bed
                                   edges during plate packing

THE PRINTED-MASS MODEL (the 4.6 kg lesson: solid mesh grams are NOT what the
printer extrudes for thick sections):
    solid_g     = rho x V_solid                       (the mesh, fully dense)
    printed_g   = rho x [ V_shell + infill x max(0, V_solid - V_shell) ]
    V_shell     = min(V_solid, A_surface x t_shell)
    t_shell     = perimeters x line_width + top_bottom_layers x layer_h / 2
                  (side walls get `perimeters` lines on EVERY surface; the
                  top/bottom skin stacks are smeared over all surfaces at half
                  thickness as a heuristic, since only up/down-facing regions
                  actually carry them)
    A_surface   = triangle-area sum over the STL (both sides of every wall
                  count, which is what a slicer's perimeters do)
Defaults give t_shell = 3x0.45 + 4x0.2/2 = 1.75 mm. printed_g carries a
STATED +/-30% honesty band — the slicer is the real number, this is a
planning figure. printed_g (not solid_g) drives cost and print time.
THICK-SECTION warning: when solid_g > 2 x printed_g the part is mostly bulk
solid — consider hollowing (e.g. sealed hollow wings) or lattice infill; the
warning lands on the line, in the receipt, and in the csv.

Honesty contract (stated, not hidden):
  - Volume is the signed-tetrahedron sum over the STL triangle soup
    (divergence theorem). The mesh is ASSUMED watertight and consistently
    wound; |sum| is taken. Garbage in => garbage grams.
  - Print time is a HEURISTIC: t_h = printed_g/12 x speed_factor per part
    (0.2 mm layers class) + 0.25 h setup per plate. Honesty band +/-50% —
    slicer time is the real number, this is a planning figure.
  - Buy lines without unit_price are marked TBD and EXCLUDED from the
    numeric total (the receipt says how many lines are TBD).
  - Parts taller than bed.z, or whose footprint cannot fit the bed in
    either 0/90 rotation with spacing, refuse the whole job loudly.

Output contract: last non-empty stdout line is ONE JSON receipt
({ok:true, parts, totals, plates, ...} or {ok:false, error}); the
human-readable per-part table goes to stderr. Failure exits 1.
"""
from __future__ import annotations

import csv
import json
import os
import sys

import numpy as np

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
MATERIAL_DB = os.path.join(TOOLS_DIR, "material_db.json")

sys.path.insert(0, TOOLS_DIR)
from _stl import load_stl  # noqa: E402 — the shared binary-STL loader

GRAMS_PER_HOUR = 12.0   # 0.2 mm layers heuristic denominator (printed grams)
PLATE_SETUP_H = 0.25    # per-plate setup/handling
TIME_BAND = "+/-50%"
MASS_BAND = "+/-30%"
PRINT_PARAM_DEFAULTS = {
	"perimeters": 3, "line_width": 0.45, "layer_h": 0.2,
	"top_bottom_layers": 4, "infill": 0.20,
}
THICK_WARNING = ("bulk solids detected — consider hollowing (e.g. sealed hollow wings) "
	"or lattice")


def log(msg: str) -> None:
	print(msg, file=sys.stderr, flush=True)


def mesh_volume_mm3(tris) -> float:
	"""Signed-tetrahedron volume of the triangle soup (divergence theorem).
	Watertight + consistent winding ASSUMED; |sum| is returned."""
	return float(abs(np.einsum("ij,ij->i", tris[:, 0], np.cross(tris[:, 1], tris[:, 2])).sum()) / 6.0)


def mesh_area_mm2(tris) -> float:
	"""Total triangle area of the STL — the surface a slicer walls with
	perimeters (both sides of every internal wall count)."""
	return float(0.5 * np.linalg.norm(np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]), axis=1).sum())


def resolve_print_params(job_params, part_params):
	"""defaults <- job.print_params <- part.print_params (later wins).
	Unknown keys are refused (a typo must not silently fall back to defaults)."""
	merged = dict(PRINT_PARAM_DEFAULTS)
	for layer_name, override in (("job.print_params", job_params), ("part.print_params", part_params)):
		if not override:
			continue
		unknown = set(override) - set(PRINT_PARAM_DEFAULTS)
		if unknown:
			raise ValueError(f"{layer_name} has unknown keys {sorted(unknown)} "
				f"(valid: {sorted(PRINT_PARAM_DEFAULTS)})")
		merged.update({k: float(v) for k, v in override.items()})
	if not 0.0 <= merged["infill"] <= 1.0:
		raise ValueError(f"infill {merged['infill']} must be a fraction in [0,1]")
	return merged


def printed_mass_g(vol_mm3, area_mm2, density_kg_m3, pp):
	"""Slicer-style printed-mass estimate (see module docstring for the exact
	formula). Returns (printed_g, t_shell_mm, shell_vol_mm3). Capped at the
	solid mass — a part thinner than its own shell prints fully dense."""
	t_shell = pp["perimeters"] * pp["line_width"] + pp["top_bottom_layers"] * pp["layer_h"] / 2.0
	shell_vol = min(vol_mm3, area_mm2 * t_shell)
	printed_vol = shell_vol + pp["infill"] * max(0.0, vol_mm3 - shell_vol)
	return printed_vol * density_kg_m3 / 1e6, t_shell, shell_vol


def load_material_db() -> tuple[dict, bool]:
	"""tools/material_db.json -> {material_lower: density_kg_m3}. Tolerant of
	{mat: number}, {mat: {density_kg_m3}}, {mat: {density_g_cm3}}, or a
	top-level {"materials": {...}} wrapper. Returns ({}, False) when absent."""
	if not os.path.exists(MATERIAL_DB):
		return {}, False
	raw = json.load(open(MATERIAL_DB))
	mats = raw.get("materials", raw) if isinstance(raw, dict) else {}
	out = {}
	for name, entry in mats.items():
		if isinstance(entry, (int, float)):
			out[name.lower()] = float(entry)
		elif isinstance(entry, dict):
			if entry.get("density_kg_m3") is not None:
				out[name.lower()] = float(entry["density_kg_m3"])
			elif entry.get("density_g_cm3") is not None:
				out[name.lower()] = float(entry["density_g_cm3"]) * 1000.0
	return out, True


def resolve_density(material: str, inline: dict, db: dict, db_present: bool):
	"""(density_kg_m3, source) — inline job values win over material_db.json."""
	key = material.lower()
	inline_l = {k.lower(): float(v) for k, v in (inline or {}).items()}
	if key in inline_l:
		return inline_l[key], "job.density_kg_m3"
	if key in db:
		return db[key], "tools/material_db.json"
	where = "tools/material_db.json and job.density_kg_m3" if db_present else \
		"job.density_kg_m3 (tools/material_db.json absent)"
	raise ValueError(f"no density for material '{material}' in {where}")


# ---------------------------------------------------------------- packing --
class Plate:
	"""Shelf-packed plate: shelves stack along +Y, parts fill each shelf +X."""

	def __init__(self, bed, spacing):
		self.bed, self.sp = bed, spacing
		self.shelves = []          # [{y0, depth, x}]
		self.y_cursor = spacing    # spacing to the bed edge too
		self.placed = []           # [{name, instance, x_mm, y_mm, w_mm, d_mm, rotated}]

	def try_place(self, name, instance, w, d) -> bool:
		for shelf in self.shelves:
			for pw, pd, rot in ((w, d, False), (d, w, True)):
				if shelf["x"] + pw + self.sp <= self.bed["x"] and pd <= shelf["depth"]:
					self.placed.append({"name": name, "instance": instance, "x_mm": round(shelf["x"], 2),
						"y_mm": round(shelf["y0"], 2), "w_mm": round(pw, 2), "d_mm": round(pd, 2), "rotated": rot})
					shelf["x"] += pw + self.sp
					return True
		# new shelf — orient with the SHORT side as depth to keep shelves shallow
		pw, pd, rot = (max(w, d), min(w, d), d > w)
		if self.sp + pw + self.sp <= self.bed["x"] and self.y_cursor + pd + self.sp <= self.bed["y"]:
			self.shelves.append({"y0": self.y_cursor, "depth": pd, "x": self.sp + pw + self.sp})
			self.placed.append({"name": name, "instance": instance, "x_mm": round(self.sp, 2),
				"y_mm": round(self.y_cursor, 2), "w_mm": round(pw, 2), "d_mm": round(pd, 2), "rotated": rot})
			self.y_cursor += pd + self.sp
			return True
		return False

	def utilization(self) -> float:
		return round(sum(p["w_mm"] * p["d_mm"] for p in self.placed) / (self.bed["x"] * self.bed["y"]), 4)


def pack_plates(made_parts, bed, spacing):
	"""2D first-fit-decreasing (long side desc) shelf packing, 0/90 rotation.

	made_parts: [{name, qty, footprint (w, d), height}] — one instance per qty.
	Refuses (raises) any part too tall for bed.z or too big for the bed."""
	instances = []
	for p in made_parts:
		w, d, h = p["footprint"][0], p["footprint"][1], p["height"]
		if h > bed["z"]:
			raise ValueError(f"part '{p['name']}' is {h:.1f} mm tall — exceeds bed z {bed['z']} mm: REFUSED")
		if min(w, d) + 2 * spacing > bed["y"] or max(w, d) + 2 * spacing > bed["x"]:
			raise ValueError(
				f"part '{p['name']}' footprint {w:.1f} x {d:.1f} mm cannot fit the "
				f"{bed['x']} x {bed['y']} bed in any 0/90 rotation with {spacing} mm spacing: REFUSED")
		for i in range(p["qty"]):
			instances.append((max(w, d), p["name"], i + 1, w, d))
	instances.sort(key=lambda t: (-t[0], t[1], t[2]))  # FFD: long side decreasing, then stable

	plates = [] if instances else []
	for _, name, inst, w, d in instances:
		if not any(pl.try_place(name, inst, w, d) for pl in plates):
			pl = Plate(bed, spacing)
			if not pl.try_place(name, inst, w, d):  # pre-checked above; belt+braces
				raise ValueError(f"part '{name}' does not fit an empty plate: REFUSED")
			plates.append(pl)
	return plates


def draw_plate_layout(plates, bed, out_png, date=""):
	"""One PNG: each plate as a bed-outline subplot with labeled part rects —
	the picture a human checks before hitting print."""
	import matplotlib

	matplotlib.use("Agg")
	import matplotlib.pyplot as plt
	from matplotlib.patches import Rectangle

	n = len(plates)
	fig, axs = plt.subplots(1, n, figsize=(4.2 * n, 4.6), dpi=140)
	axs = [axs] if n == 1 else list(axs)
	fig.patch.set_facecolor("#f5f4f2")
	for i, (pl, ax) in enumerate(zip(plates, axs)):
		ax.add_patch(Rectangle((0, 0), bed["x"], bed["y"], fill=False, edgecolor="#3a3f45", lw=1.2))
		for j, p in enumerate(pl.placed):
			color = ["#5b7d9e", "#c08552", "#6d9b6d", "#8a7f9e", "#b0a15f", "#6d8f8f"][j % 6]
			ax.add_patch(Rectangle((p["x_mm"], p["y_mm"]), p["w_mm"], p["d_mm"], facecolor=color, alpha=0.55, edgecolor="#3a3f45", lw=0.8))
			label = p["name"] if p["instance"] == 1 else f"{p['name']} #{p['instance']}"
			ax.text(p["x_mm"] + p["w_mm"] / 2.0, p["y_mm"] + p["d_mm"] / 2.0, label,
				ha="center", va="center", fontsize=7, family="DejaVu Sans", color="#1a1d21")
		ax.set_xlim(-8, bed["x"] + 8)
		ax.set_ylim(-8, bed["y"] + 8)
		ax.set_aspect("equal")
		ax.set_title(f"PLATE {i + 1} / {n} — {bed['x']:.0f}×{bed['y']:.0f} mm bed · {pl.utilization() * 100:.1f}% used",
			fontsize=8.5, family="DejaVu Sans", weight="bold")
		ax.tick_params(labelsize=6)
	fig.suptitle(f"Print-plate layout{(' · ' + date) if date else ''}", fontsize=10, family="DejaVu Sans", weight="bold")
	fig.tight_layout(rect=(0, 0, 1, 0.95))
	fig.savefig(out_png, facecolor="#f5f4f2")
	plt.close(fig)


# ----------------------------------------------------------------- rollup --
def build_dossier(job):
	out_dir = job["out_dir"]
	os.makedirs(out_dir, exist_ok=True)
	parts_in = job.get("parts")
	if not parts_in:
		raise ValueError("job.parts is required and must be non-empty")
	bed = {"x": 220.0, "y": 220.0, "z": 250.0}
	bed.update({k: float(v) for k, v in (job.get("bed") or {}).items()})
	spacing = float(job.get("spacing_mm", 5.0))
	price_kg = float(job.get("filament_price_per_kg", 25.0))
	speed = float(job.get("print_speed_factor", 1.0))
	db, db_present = load_material_db()

	lines, made_for_packing, warnings = [], [], []
	stl_by_name = {}
	for p in parts_in:
		name, qty = p["name"], int(p.get("qty", 1))
		wear = bool(p.get("wear", False))
		if p.get("buy", False):
			unit = p.get("unit_price")
			lines.append({
				"name": name, "kind": "buy", "qty": qty,
				"part_number": p.get("part_number", ""), "source": p.get("source", ""),
				"unit_cost": (None if unit is None else round(float(unit), 4)),
				"line_cost": (None if unit is None else round(float(unit) * qty, 4)),
				"wear": wear,
			})
			continue
		tris = load_stl(p["stl"])
		vol = mesh_volume_mm3(tris)
		area = mesh_area_mm2(tris)
		lo = tris.reshape(-1, 3).min(axis=0)
		hi = tris.reshape(-1, 3).max(axis=0)
		ext = hi - lo
		density, dsource = resolve_density(p["material"], job.get("density_kg_m3"), db, db_present)
		pp = resolve_print_params(job.get("print_params"), p.get("print_params"))
		solid_g = vol * density / 1e6  # mm^3 * kg/m^3 -> g
		printed_g, t_shell, shell_vol = printed_mass_g(vol, area, density, pp)
		thick = solid_g > 2.0 * printed_g
		if thick:
			warnings.append(f"{name}: solid {solid_g:.0f} g vs printed ~{printed_g:.0f} g — {THICK_WARNING}")
		t_h = printed_g / GRAMS_PER_HOUR * speed
		unit_cost = printed_g * price_kg / 1000.0
		lines.append({
			"name": name, "kind": "made", "qty": qty, "material": p["material"],
			"stl": os.path.abspath(p["stl"]), "volume_mm3": round(vol, 2),
			"surface_mm2": round(area, 2),
			"density_kg_m3": density, "density_source": dsource,
			"solid_g_per_unit": round(solid_g, 3),
			"printed_g_per_unit": round(printed_g, 3), "printed_mass_band": MASS_BAND,
			"t_shell_mm": round(t_shell, 3), "shell_vol_mm3": round(shell_vol, 2),
			"print_params": pp,
			# grams_per_unit stays the driving figure for downstream consumers
			# (BOM sheets, time, cost) and now means PRINTED grams, not solid.
			"grams_per_unit": round(printed_g, 3), "grams_total": round(printed_g * qty, 3),
			"thick_section_warning": (THICK_WARNING if thick else None),
			"print_time_h_per_unit": round(t_h, 3), "print_time_h": round(t_h * qty, 3),
			"unit_cost": round(unit_cost, 4), "line_cost": round(unit_cost * qty, 4),
			"footprint_mm": [round(float(ext[0]), 2), round(float(ext[1]), 2)],
			"height_mm": round(float(ext[2]), 2), "wear": wear,
			# Per-part print truth (audit 2026-07-16): "PETG required" must live
			# in DATA, not hand-written prose. material_required=True means the
			# stated material is a functional requirement (springs, heat), not a
			# suggestion; print_notes carries orientation/strength notes into the
			# BOM csv and every downstream instructions renderer.
			"material_required": bool(p.get("material_required", False)),
			"print_notes": p.get("print_notes", ""),
		})
		stl_by_name[name] = p["stl"]
		made_for_packing.append({"name": name, "qty": qty,
			"footprint": (float(ext[0]), float(ext[1])), "height": float(ext[2])})

	plates = pack_plates(made_for_packing, bed, spacing)
	plate_receipts = [{"plate": i + 1, "parts": pl.placed, "footprint_utilization": pl.utilization()}
		for i, pl in enumerate(plates)]
	# Combined plate STLs + a bed-layout sheet (audit 2026-07-16: the packing
	# existed only as coordinates; now it is a sliceable artifact + a picture).
	plate_files = []
	if plates and job.get("emit_plates", True):
		from _stl import write_stl

		time_by_name = {ln["name"]: ln["print_time_h_per_unit"] for ln in lines if ln["kind"] == "made"}
		for i, pl in enumerate(plates):
			merged = []
			for placed in pl.placed:
				tris = np.array(load_stl(stl_by_name[placed["name"]]), dtype=np.float64)
				lo = tris.reshape(-1, 3).min(axis=0)
				tris = tris - lo  # corner to origin, bed at z=0
				if placed["rotated"]:
					x = tris[:, :, 0].copy()
					y = tris[:, :, 1].copy()
					w = tris[:, :, 0].max()
					tris[:, :, 0] = y
					tris[:, :, 1] = w - x  # 90 deg about z, back into +xy
					tris = tris - tris.reshape(-1, 3).min(axis=0)
				tris[:, :, 0] += placed["x_mm"]
				tris[:, :, 1] += placed["y_mm"]
				merged.append(tris.reshape(-1, 3, 3))
			path = os.path.join(out_dir, f"plate_{i + 1}.stl")
			write_stl(path, np.concatenate(merged))
			plate_receipts[i]["stl"] = os.path.abspath(path)
			plate_receipts[i]["print_time_h"] = round(
				sum(time_by_name.get(pp["name"], 0.0) for pp in pl.placed) + PLATE_SETUP_H, 3)
			plate_files.append(os.path.abspath(path))
		layout_png = os.path.join(out_dir, "plate_layout.png")
		draw_plate_layout(plates, bed, layout_png, job.get("date", ""))
		plate_files.append(os.path.abspath(layout_png))
	part_plates = {}
	for i, pl in enumerate(plates):
		for pp_ in pl.placed:
			part_plates.setdefault(pp_["name"], set()).add(i + 1)

	made = [l for l in lines if l["kind"] == "made"]
	buys = [l for l in lines if l["kind"] == "buy"]
	total_made = sum(l["line_cost"] for l in made)
	priced_buys = [l for l in buys if l["line_cost"] is not None]
	tbd_lines = [l["name"] for l in buys if l["line_cost"] is None]
	total_buy = sum(l["line_cost"] for l in priced_buys)
	total_time = sum(l["print_time_h"] for l in made) + PLATE_SETUP_H * len(plates)

	totals = {
		"total_made_cost": round(total_made, 2),
		"total_buy_cost": round(total_buy, 2),
		"buy_lines_tbd": tbd_lines,
		"total_cost": round(total_made + total_buy, 2),
		"total_cost_note": ("complete" if not tbd_lines else
			f"EXCLUDES {len(tbd_lines)} TBD buy line(s): {', '.join(tbd_lines)}"),
		"total_print_time_h": round(total_time, 2),
		"time_band": TIME_BAND,
		"n_plates": len(plates),
		"total_printed_grams": round(sum(l["grams_total"] for l in made), 2),
		"printed_grams_band": MASS_BAND,
		"total_solid_grams": round(sum(l["solid_g_per_unit"] * l["qty"] for l in made), 2),
		# kept for downstream compat: total_grams == total PRINTED grams
		"total_grams": round(sum(l["grams_total"] for l in made), 2),
	}
	receipt = {
		"ok": True, "parts": lines, "totals": totals, "plates": plate_receipts,
		"warnings": warnings,
		"bed": bed, "spacing_mm": spacing, "filament_price_per_kg": price_kg,
		"material_db_used": db_present,
		"mass_model": ("printed_g = rho x [V_shell + infill x max(0, V_solid - V_shell)], "
			"V_shell = min(V_solid, A_surface x t_shell), "
			"t_shell = perimeters x line_width + top_bottom_layers x layer_h / 2 "
			f"(defaults {PRINT_PARAM_DEFAULTS} -> t_shell 1.75 mm); band {MASS_BAND}; "
			"A_surface = STL triangle-area sum; printed grams drive cost + time; "
			"solid_g = rho x V_solid is reported alongside; "
			"thick-section warning when solid_g > 2 x printed_g"),
		"time_model": (f"t_h = printed_g/{GRAMS_PER_HOUR:g} x speed_factor({speed:g}) per part "
			f"(0.2 mm layers class) + {PLATE_SETUP_H} h setup per plate; band {TIME_BAND}"),
		"volume_method": ("signed-tetrahedron sum over the STL triangle soup; mesh assumed "
			"watertight + consistently wound (|sum| taken)"),
	}

	json_path = os.path.join(out_dir, "bom_dossier.json")
	csv_path = os.path.join(out_dir, "bom_dossier.csv")
	with open(json_path, "w") as f:
		json.dump(receipt, f, indent=1)
	cols = ["name", "kind", "qty", "material", "material_required", "print_notes",
		"part_number", "source", "volume_mm3",
		"solid_g_per_unit", "printed_g_per_unit", "grams_per_unit", "print_time_h",
		"unit_cost", "line_cost", "plates", "wear", "warning"]
	with open(csv_path, "w", newline="") as f:
		w = csv.DictWriter(f, fieldnames=cols)
		w.writeheader()
		for l in lines:
			row = {c: l.get(c, "") for c in cols}
			row["plates"] = "+".join(str(i) for i in sorted(part_plates.get(l["name"], []))) if l["kind"] == "made" else ""
			if l["kind"] == "buy" and l["line_cost"] is None:
				row["unit_cost"], row["line_cost"] = "TBD", "TBD"
			row["wear"] = "wear" if l["wear"] else ""
			row["warning"] = "THICK-SECTION" if l.get("thick_section_warning") else ""
			w.writerow(row)
	receipt["files"] = {"json": os.path.abspath(json_path), "csv": os.path.abspath(csv_path)}
	if plate_files:
		receipt["files"]["plates"] = plate_files

	# human table -> stderr (the JSON receipt stays the machine contract)
	log(f"{'name':<22}{'kind':<6}{'qty':>4}{'solid_g':>9}{'print_g':>9}{'t_h':>7}{'cost':>9}  plate")
	for l in lines:
		if l["kind"] == "made":
			pl = "+".join(str(i) for i in sorted(part_plates.get(l["name"], [])))
			flag = "  THICK" if l.get("thick_section_warning") else ""
			log(f"{l['name']:<22}{'made':<6}{l['qty']:>4}{l['solid_g_per_unit'] * l['qty']:>9.2f}"
				f"{l['grams_total']:>9.2f}{l['print_time_h']:>7.2f}{l['line_cost']:>9.2f}  {pl}{flag}")
		else:
			cost = "TBD" if l["line_cost"] is None else f"{l['line_cost']:.2f}"
			log(f"{l['name']:<22}{'buy':<6}{l['qty']:>4}{'-':>9}{'-':>9}{'-':>7}{cost:>9}  -")
	for wmsg in warnings:
		log(f"WARNING: {wmsg}")
	log(f"TOTAL made {totals['total_made_cost']} + buy {totals['total_buy_cost']} = {totals['total_cost']} "
		f"({totals['total_cost_note']}); {totals['n_plates']} plate(s), "
		f"{totals['total_print_time_h']} h {TIME_BAND}; printed {totals['total_printed_grams']} g "
		f"{MASS_BAND} (solid would be {totals['total_solid_grams']} g)")
	return receipt


def main():
	if len(sys.argv) != 2:
		print(json.dumps({"ok": False, "error": "usage: production_dossier.py job.json"}))
		return 1
	try:
		job = json.load(open(sys.argv[1]))
		if "out_dir" not in job:
			raise ValueError("job needs 'out_dir'")
		receipt = build_dossier(job)
	except Exception as e:  # noqa: BLE001 — the receipt IS the error channel
		print(json.dumps({"ok": False, "error": f"{type(e).__name__}: {e}"}))
		return 1
	print(json.dumps(receipt))
	return 0


if __name__ == "__main__":
	sys.exit(main())
