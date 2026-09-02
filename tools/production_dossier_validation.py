#!/usr/bin/env python3
"""Validation pin: production_dossier.py vs analytic box STLs.

A box has an exact volume, an exact surface area, and — once the tool's own
documented shell formula is applied by hand — an exact printed mass, cost and
print time. Every expected number below is DERIVED IN THIS FILE from the
closed forms in tools/manifests/production_dossier.manifest.json; a wrong
tetrahedron sign, a swapped area term, a shell cap that stops applying, a
packing rule that lets a part off the bed, or an exit code that disagrees with
`ok` trips a pin.

	Pin 1  30x20x10 mm box: volume, area, solid/printed grams, cost, time, footprint
	Pin 2  50 mm cube: the THICK-SECTION warning fires (solid > 2 x printed)
	Pin 3  30x20x1 mm plate: printed mass CAPPED at the solid mass (thinner than its shell)
	Pin 4  buy lines: priced line summed, unpriced line TBD and EXCLUDED, named in the note
	Pin 5  packing: qty 4 of the box on a 60x60 bed -> 2 plates, every placement in-bed,
	       no overlaps, plate setup time counted twice; plate STL volume == part volume sum
	Pin 6  refusal: a part taller than bed.z refuses the whole job (ok:false, exit 1)
	Pin 7  determinism: the stdout receipt is byte-identical across two runs

Hermetic: numpy only (the tool's own dependency), STLs written to a temp dir,
the analyzer driven through its real CLI (subprocess, last stdout line).
`emit_plates` is off except in pin 5, which also needs matplotlib for the
layout PNG and is skipped LOUDLY (exit 0, but printed) when it is absent.

Measured 2026-09-02 (pinned here): volume/area exact (0.0 relative); printed
5.3072 g / 0.1327 cost / 0.442 h per box; cube warning fired (155.0 g solid vs
57.04 g printed); plate mass capped at 0.744 g; 2 plates for 4 boxes on 60x60;
tall part refused with exit 1; reruns identical.

Run:  python3 tools/production_dossier_validation.py
Exit: 0 iff all assertions hold; nonzero with a message otherwise.
"""
import json
import os
import struct
import subprocess
import sys
import tempfile

TOOLS = os.path.dirname(os.path.abspath(__file__))
TOOL = os.path.join(TOOLS, "production_dossier.py")

# --- the material table cell the tool reads (tools/material_db.json) -------
RHO_PLA = 1240.0            # material_db.json PLA.density_kg_m3
PRICE_PER_KG = 25.0         # tool default
GRAMS_PER_HOUR = 12.0       # tool default (0.2 mm layer class)
PLATE_SETUP_H = 0.25        # tool default
T_SHELL = 3 * 0.45 + 4 * 0.2 / 2.0   # perimeters*line_width + top_bottom_layers*layer_h/2 = 1.75 mm
INFILL = 0.20


def write_box_stl(path, a, b, c):
	"""Axis-aligned box [0,a]x[0,b]x[0,c] as 12 outward-wound binary-STL triangles."""
	v = [(0, 0, 0), (a, 0, 0), (a, b, 0), (0, b, 0), (0, 0, c), (a, 0, c), (a, b, c), (0, b, c)]
	# faces as CCW-from-outside vertex quads
	quads = [
		(0, 3, 2, 1),  # z = 0 (normal -z)
		(4, 5, 6, 7),  # z = c (normal +z)
		(0, 1, 5, 4),  # y = 0 (normal -y)
		(2, 3, 7, 6),  # y = b (normal +y)
		(0, 4, 7, 3),  # x = 0 (normal -x)
		(1, 2, 6, 5),  # x = a (normal +x)
	]
	tris = []
	for q in quads:
		tris.append((v[q[0]], v[q[1]], v[q[2]]))
		tris.append((v[q[0]], v[q[2]], v[q[3]]))
	with open(path, "wb") as f:
		f.write(b"lmcad production_dossier validation box".ljust(80, b" "))
		f.write(struct.pack("<I", len(tris)))
		for t in tris:
			f.write(struct.pack("<3f", 0.0, 0.0, 0.0))  # normals are recomputed by readers
			for p in t:
				f.write(struct.pack("<3f", *p))
			f.write(struct.pack("<H", 0))


def expected_box(a, b, c, rho=RHO_PLA):
	"""The manifest's closed forms, applied by hand."""
	vol = a * b * c
	area = 2.0 * (a * b + b * c + c * a)
	shell = min(vol, area * T_SHELL)
	printed_vol = shell + INFILL * max(0.0, vol - shell)
	solid_g = vol * rho / 1e6
	printed_g = printed_vol * rho / 1e6
	return {
		"vol": vol, "area": area, "shell_vol": shell,
		"solid_g": solid_g, "printed_g": printed_g,
		"unit_cost": printed_g * PRICE_PER_KG / 1000.0,
		"t_h": printed_g / GRAMS_PER_HOUR,
		"thick": solid_g > 2.0 * printed_g,
	}


def run(job: dict, workdir: str, tag: str):
	"""Drive the production CLI. Returns (receipt, exit code, raw stdout line)."""
	job = dict(job)
	job.setdefault("out_dir", os.path.join(workdir, tag))
	job_path = os.path.join(workdir, f"{tag}.json")
	with open(job_path, "w") as f:
		json.dump(job, f)
	out = subprocess.run([sys.executable, TOOL, job_path], capture_output=True,
	                     text=True, timeout=300)
	last = ""
	for line in out.stdout.splitlines():
		if line.strip():
			last = line
	assert last, f"pin '{tag}': no receipt on stdout; stderr tail: {out.stderr[-400:]!r}"
	return json.loads(last), out.returncode, last


def rel(a, b):
	return abs(a - b) / max(abs(b), 1e-12)


def main() -> None:
	work = tempfile.mkdtemp(prefix="dossier_pin_")
	box = os.path.join(work, "box_30_20_10.stl")
	cube = os.path.join(work, "cube_50.stl")
	plate = os.path.join(work, "plate_30_20_1.stl")
	tall = os.path.join(work, "tall_10_10_300.stl")
	write_box_stl(box, 30.0, 20.0, 10.0)
	write_box_stl(cube, 50.0, 50.0, 50.0)
	write_box_stl(plate, 30.0, 20.0, 1.0)
	write_box_stl(tall, 10.0, 10.0, 300.0)

	# ------------------------------------------------------------------
	# Pin 1 — 30 x 20 x 10 mm box, PLA.
	#   V = 30*20*10 = 6000 mm^3 ; A = 2(30*20 + 20*10 + 10*30) = 2(600+200+300) = 2200 mm^2
	#   solid_g   = 6000 * 1240 / 1e6 = 7.44 g
	#   t_shell   = 3*0.45 + 4*0.2/2 = 1.35 + 0.40 = 1.75 mm
	#   V_shell   = min(6000, 2200*1.75 = 3850) = 3850 mm^3
	#   V_printed = 3850 + 0.20 * (6000 - 3850) = 3850 + 430 = 4280 mm^3
	#   printed_g = 4280 * 1240 / 1e6 = 5.3072 g ; not thick (7.44 < 2*5.3072 = 10.6144)
	#   unit_cost = 5.3072 * 25 / 1000 = 0.13268 ; t_h = 5.3072 / 12 = 0.442267 h
	#   footprint [30, 20], height 10 ; 1 plate -> total time 0.442267 + 0.25 = 0.692267 h
	# ------------------------------------------------------------------
	e = expected_box(30.0, 20.0, 10.0)
	r1, rc1, _ = run({"parts": [{"name": "box", "stl": box, "material": "PLA"}],
	                  "emit_plates": False}, work, "pin1_box")
	assert r1["ok"] is True and rc1 == 0, f"pin 1 FAILED: dossier did not run: {r1.get('error')} (exit {rc1})"
	p = r1["parts"][0]
	assert rel(p["volume_mm3"], e["vol"]) <= 1e-6, f"pin 1 FAILED: volume {p['volume_mm3']} vs analytic {e['vol']}"
	assert rel(p["surface_mm2"], e["area"]) <= 1e-6, f"pin 1 FAILED: area {p['surface_mm2']} vs analytic {e['area']}"
	assert p["density_kg_m3"] == RHO_PLA and p["density_source"] == "tools/material_db.json", (
		f"pin 1 FAILED: density {p['density_kg_m3']} from {p['density_source']}; expected PLA 1240 from material_db")
	assert abs(p["t_shell_mm"] - T_SHELL) <= 1e-9, f"pin 1 FAILED: t_shell {p['t_shell_mm']} vs 1.75"
	assert rel(p["shell_vol_mm3"], e["shell_vol"]) <= 1e-6, f"pin 1 FAILED: shell_vol {p['shell_vol_mm3']} vs {e['shell_vol']}"
	assert abs(p["solid_g_per_unit"] - e["solid_g"]) <= 5e-4, f"pin 1 FAILED: solid_g {p['solid_g_per_unit']} vs {e['solid_g']}"
	assert abs(p["printed_g_per_unit"] - e["printed_g"]) <= 5e-4, f"pin 1 FAILED: printed_g {p['printed_g_per_unit']} vs {e['printed_g']}"
	assert p["grams_per_unit"] == p["printed_g_per_unit"], "pin 1 FAILED: grams_per_unit must be the PRINTED grams"
	assert abs(p["unit_cost"] - e["unit_cost"]) <= 5e-5, f"pin 1 FAILED: unit_cost {p['unit_cost']} vs {e['unit_cost']}"
	assert abs(p["print_time_h_per_unit"] - e["t_h"]) <= 5e-4, f"pin 1 FAILED: t_h {p['print_time_h_per_unit']} vs {e['t_h']}"
	assert p["footprint_mm"] == [30.0, 20.0] and p["height_mm"] == 10.0, (
		f"pin 1 FAILED: footprint {p['footprint_mm']} height {p['height_mm']} vs [30,20] x 10")
	assert p["thick_section_warning"] is None and r1["warnings"] == [], "pin 1 FAILED: the box is not thick-section"
	t = r1["totals"]
	assert t["n_plates"] == 1 and abs(t["total_print_time_h"] - (e["t_h"] + PLATE_SETUP_H)) <= 5e-3, (
		f"pin 1 FAILED: 1 plate, total time {t['total_print_time_h']} vs {e['t_h'] + PLATE_SETUP_H:.4f}")
	assert abs(t["total_made_cost"] - e["unit_cost"]) <= 5e-3 and t["buy_lines_tbd"] == [] and t["total_cost_note"] == "complete", (
		f"pin 1 FAILED: totals {t}")
	print(f"pin 1 OK: V {p['volume_mm3']} mm^3, A {p['surface_mm2']} mm^2, solid {p['solid_g_per_unit']} g, "
	      f"printed {p['printed_g_per_unit']} g (hand: {e['printed_g']:.4f}), cost {p['unit_cost']}, t {p['print_time_h_per_unit']} h")

	# ------------------------------------------------------------------
	# Pin 2 — 50 mm cube: V = 125000, A = 6*2500 = 15000; V_shell = min(125000, 26250) = 26250;
	#   V_printed = 26250 + 0.2*98750 = 46000 mm^3 -> 57.04 g ; solid 155.0 g > 2*57.04 = 114.08 -> THICK.
	# ------------------------------------------------------------------
	e2 = expected_box(50.0, 50.0, 50.0)
	assert e2["thick"], "pin 2 self-check: the hand derivation itself must be thick"
	r2, rc2, _ = run({"parts": [{"name": "cube", "stl": cube, "material": "pla"}],  # case-insensitive key
	                  "emit_plates": False}, work, "pin2_cube")
	p = r2["parts"][0]
	assert rc2 == 0 and abs(p["printed_g_per_unit"] - e2["printed_g"]) <= 5e-4 and abs(p["solid_g_per_unit"] - e2["solid_g"]) <= 5e-4, (
		f"pin 2 FAILED: printed {p['printed_g_per_unit']} / solid {p['solid_g_per_unit']} vs hand {e2['printed_g']:.4f} / {e2['solid_g']:.4f}")
	assert p["thick_section_warning"] and len(r2["warnings"]) == 1 and "cube" in r2["warnings"][0], (
		f"pin 2 FAILED: thick-section warning must fire on the line AND in the receipt: {p['thick_section_warning']!r} / {r2['warnings']}")
	print(f"pin 2 OK: cube solid {p['solid_g_per_unit']} g > 2 x printed {p['printed_g_per_unit']} g -> THICK warning fired")

	# ------------------------------------------------------------------
	# Pin 3 — 30 x 20 x 1 mm plate: V = 600, A = 2(600 + 20 + 30) = 1300 ; A*t_shell = 2275 > 600
	#   -> V_shell = 600 (capped), printed volume = 600 + 0.2*0 = 600 = V -> printed_g == solid_g = 0.744 g.
	# ------------------------------------------------------------------
	e3 = expected_box(30.0, 20.0, 1.0)
	r3, rc3, _ = run({"parts": [{"name": "plate", "stl": plate, "material": "PLA"}],
	                  "emit_plates": False}, work, "pin3_plate")
	p = r3["parts"][0]
	assert rc3 == 0 and rel(p["shell_vol_mm3"], 600.0) <= 1e-6 and abs(p["printed_g_per_unit"] - p["solid_g_per_unit"]) <= 1e-9 \
		and abs(p["solid_g_per_unit"] - e3["solid_g"]) <= 5e-4, (
		f"pin 3 FAILED: a part thinner than its shell must print fully dense: shell_vol {p['shell_vol_mm3']}, "
		f"printed {p['printed_g_per_unit']} vs solid {p['solid_g_per_unit']} (hand {e3['solid_g']:.4f})")
	print(f"pin 3 OK: plate printed_g == solid_g == {p['printed_g_per_unit']} g (shell cap applied)")

	# ------------------------------------------------------------------
	# Pin 4 — buy lines: 4 x 0.05 = 0.20 priced; 'bearing' unpriced -> TBD, excluded, named.
	#   total_cost = made 0.13268 + 0.20 = 0.33268 -> 0.33 ; total_cost_note names the 1 TBD line.
	# ------------------------------------------------------------------
	r4, rc4, _ = run({"parts": [
		{"name": "box", "stl": box, "material": "PLA"},
		{"name": "M3x8", "buy": True, "qty": 4, "unit_price": 0.05, "part_number": "ISO 7380"},
		{"name": "bearing", "buy": True, "qty": 2}],
		"emit_plates": False}, work, "pin4_buy")
	t = r4["totals"]
	buys = {l["name"]: l for l in r4["parts"] if l["kind"] == "buy"}
	assert rc4 == 0 and abs(buys["M3x8"]["line_cost"] - 0.20) <= 1e-9 and buys["bearing"]["line_cost"] is None, (
		f"pin 4 FAILED: buy lines {buys}")
	assert t["buy_lines_tbd"] == ["bearing"] and abs(t["total_buy_cost"] - 0.20) <= 1e-9 \
		and abs(t["total_cost"] - round(e["unit_cost"] + 0.20, 2)) <= 5e-3 and "EXCLUDES 1 TBD" in t["total_cost_note"] \
		and "bearing" in t["total_cost_note"], f"pin 4 FAILED: totals {t}"
	print(f"pin 4 OK: buy 0.20 summed, 'bearing' TBD excluded and named; total {t['total_cost']} ({t['total_cost_note']})")

	# ------------------------------------------------------------------
	# Pin 5 — packing. Bed 60 x 60 x 250, spacing 5, qty 4 of the 30x20 box.
	#   Shelf 1 (y0 = 5): 5 + 30 + 5 = 40 <= 60 -> one box; a second needs 40+30+5 = 75 (or 65 rotated) > 60.
	#   Shelf 2 (y0 = 30): 30 + 20 + 5 = 55 <= 60 -> one box.  Shelf 3 would need 55 + 20 + 5 = 80 > 60.
	#   => 2 boxes per plate, 4 boxes -> 2 plates; total time = 4 * 0.442267 + 2 * 0.25 = 2.269 h.
	#   Every placement must lie inside [5, bed - 5] and no two rectangles may overlap.
	#   With emit_plates on, each plate STL's volume must equal 2 * 6000 mm^3 (translation-invariant).
	# ------------------------------------------------------------------
	try:
		import matplotlib  # noqa: F401  — the layout PNG needs it
		emit = True
	except ImportError:
		emit = False
		print("pin 5 NOTE: matplotlib not importable — plate STL/PNG emission skipped, packing still pinned")
	r5, rc5, _ = run({"parts": [{"name": "box", "stl": box, "material": "PLA", "qty": 4}],
	                  "bed": {"x": 60, "y": 60, "z": 250}, "spacing_mm": 5.0,
	                  "emit_plates": emit, "date": "2026-09-02"}, work, "pin5_pack")
	assert rc5 == 0 and r5["totals"]["n_plates"] == 2 and len(r5["plates"]) == 2, (
		f"pin 5 FAILED: 4 boxes on a 60x60 bed at 5 mm spacing pack 2 per plate -> 2 plates; got {r5['totals']['n_plates']}")
	assert abs(r5["totals"]["total_print_time_h"] - (4 * e["t_h"] + 2 * PLATE_SETUP_H)) <= 5e-3, (
		f"pin 5 FAILED: total time {r5['totals']['total_print_time_h']} vs 4 x {e['t_h']:.4f} + 2 x 0.25")
	for pl in r5["plates"]:
		rects = pl["parts"]
		assert len(rects) == 2, f"pin 5 FAILED: plate {pl['plate']} holds {len(rects)} parts, expected 2"
		for q in rects:
			assert q["x_mm"] >= 5.0 - 1e-9 and q["y_mm"] >= 5.0 - 1e-9 \
				and q["x_mm"] + q["w_mm"] <= 60.0 - 5.0 + 1e-9 and q["y_mm"] + q["d_mm"] <= 60.0 - 5.0 + 1e-9, (
				f"pin 5 FAILED: placement {q} leaves the 60x60 bed's 5 mm margin")
		for i in range(len(rects)):
			for j in range(i + 1, len(rects)):
				a, b = rects[i], rects[j]
				sep = (a["x_mm"] + a["w_mm"] + 5.0 <= b["x_mm"] + 1e-9 or b["x_mm"] + b["w_mm"] + 5.0 <= a["x_mm"] + 1e-9
				       or a["y_mm"] + a["d_mm"] + 5.0 <= b["y_mm"] + 1e-9 or b["y_mm"] + b["d_mm"] + 5.0 <= a["y_mm"] + 1e-9)
				assert sep, f"pin 5 FAILED: placements overlap or violate spacing: {a} / {b}"
	if emit:
		sys.path.insert(0, TOOLS)
		from _stl import load_stl  # noqa: E402
		import production_dossier as pd  # noqa: E402
		for pl in r5["plates"]:
			assert os.path.isfile(pl["stl"]), f"pin 5 FAILED: plate STL {pl['stl']} missing"
			vol = pd.mesh_volume_mm3(load_stl(pl["stl"]))
			assert rel(vol, 2 * e["vol"]) <= 1e-6, f"pin 5 FAILED: plate STL volume {vol} vs 2 x {e['vol']}"
			assert abs(pl["print_time_h"] - (2 * e["t_h"] + PLATE_SETUP_H)) <= 5e-3, (
				f"pin 5 FAILED: plate time {pl['print_time_h']} vs 2 x {e['t_h']:.4f} + 0.25")
		assert any(f.endswith("plate_layout.png") and os.path.isfile(f) for f in r5["files"]["plates"]), (
			"pin 5 FAILED: plate_layout.png not emitted")
	print(f"pin 5 OK: 2 plates x 2 boxes, all placements inside the 5 mm margin and mutually separated; "
	      f"total {r5['totals']['total_print_time_h']} h" + (" ; plate STL volumes == 2 x 6000" if emit else ""))

	# ------------------------------------------------------------------
	# Pin 6 — refusal: 300 mm tall part vs bed z 250 -> ok:false, exit 1, no dossier written.
	# ------------------------------------------------------------------
	r6, rc6, _ = run({"parts": [{"name": "tall", "stl": tall, "material": "PLA"}],
	                  "emit_plates": False}, work, "pin6_tall")
	assert r6["ok"] is False and rc6 == 1 and "exceeds bed z" in r6.get("error", ""), (
		f"pin 6 FAILED: a 300 mm part must refuse the job (ok:false, exit 1, 'exceeds bed z'); got {r6} exit {rc6}")
	assert not os.path.exists(os.path.join(work, "pin6_tall", "bom_dossier.json")), (
		"pin 6 FAILED: a refused job must not leave a bom_dossier.json behind")
	print(f"pin 6 OK: tall part refused -> {r6['error'][:70]}..., exit 1")

	# ------------------------------------------------------------------
	# Pin 7 — determinism (same out_dir both runs so the echoed absolute paths agree).
	# ------------------------------------------------------------------
	job7 = {"parts": [{"name": "box", "stl": box, "material": "PLA", "qty": 2},
	                  {"name": "M3x8", "buy": True, "qty": 4, "unit_price": 0.05}],
	        "emit_plates": False, "out_dir": os.path.join(work, "pin7")}
	_, _, la = run(job7, work, "pin7_a")
	_, _, lb = run(job7, work, "pin7_b")
	assert la == lb, "pin 7 FAILED: two runs of the identical job produced different receipts"
	print("pin 7 OK: rerun receipt byte-identical")

	print("production_dossier validation: ALL PINS OK")


if __name__ == "__main__":
	main()
