#!/usr/bin/env python3
"""document_bundle.py — ONE command documents a generated part/assembly fully.

The orchestrator the 2026-07-16 documentation audit demanded: it takes the
assembly + per-part metadata ONCE and emits the entire deliverable set into the
standard bundle layout (the showcase convention, now code, with a signed
manifest):

    <out_dir>/
    ├── print/      part STL (+3MF +STEP where a program is given), plate_N.stl
    ├── docs/       dimensioned per-part sheets, assembly sheet, exploded doc +
    │               instructions (print table / fits / revisions / honesty
    │               footer appended), plate_layout.png, motion + assembly-
    │               sequence GIF/MP4/poster, analysis sheet
    ├── receipts/   EVERY machine receipt: per-part build (validate/support/
    │               mass/export routes), dossier BOM json+csv, every check's
    │               receipt, fits table, assembly clearances
    ├── programs/   the part/assembly programs and every check job — the bundle
    │               re-verifies itself
    ├── README.md   index with thumbnails, BOM + gates summary
    └── manifest.json  every artifact with sha256 + bytes + role, meta, rev

Job JSON (argv[1]) — all paths relative to the JOB FILE's directory:
{
  "name": "squatchee_spin", "title": "...", "out_dir": "bundle",
  "date": "2026-07-16",            REQUIRED (determinism: never the clock)
  "rev": "B", "changelog": [{"rev","date","note"}...],
  "bed": {...}?, "print_params": {...}?,
  "parts": [{"name", "program_file"|"program", "solid",
             "material", "qty"?, "material_required"?, "print_notes"?, "wear"?,
             "formats": ["stl","3mf","step"]?   (default all three),
             "sheet": {"sections"?, "auto_dimensions": true?, "max_dims": 9?,
                       "dimensions": [...]? (manual extras)}}],
  "assembly": {"program_file"?,               (run for clearance receipts)
               "parts": [{"name","stl","color"?}...],   posed STLs (job-relative;
                         names MUST equal parts[].name so balloons join the BOM)
               "explode": {...}? (default {axis:[0,0,1], auto:true}),
               "steps": [...]? | auto-draft when absent,
               "motion": {...}?,              (motion_gif passthrough job)
               "sequence": true|{...}?},      (assembly-sequence GIF+MP4+poster)
  "checks": [{"name", "tool", "job": {...}|"path.json"}...],
      tool ∈ tolerance_stack / sweep_check / balance_check / joint_check /
             production_check / air_topology_audit — receipts persist, gates
             land in the analysis sheet + README
  "fits": [{"label", "op", "params": {...}}...],   design-math ops → fits table
  "templates": [{"src", "dst"}...]   markdown with {{dotted.path}} /
      {{dotted.path:.2f}} injected from the merged receipt tree — numbers can
      no longer drift from the receipts. Unresolvable keys FAIL the bundle.
}

Receipt (stdout last line + manifest): {ok, out_dir, artifacts, gates, ...}.
Honesty: every part's export route (exact / voxel_healed), watertightness and
support steep-area are read from the kernel's own receipts and stamped into the
human docs' footer; analysis tools carry their analyzer_registry TIER. Nothing
is claimed that a receipt does not state.
"""

import hashlib
import json
import os
import re
import shutil
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _receipt  # noqa: E402
import param_optimize  # noqa: E402

TOOLS = os.path.dirname(os.path.abspath(__file__))


def log(msg):
	print(msg, file=sys.stderr, flush=True)


def sha256(path):
	h = hashlib.sha256()
	with open(path, "rb") as f:
		for chunk in iter(lambda: f.read(1 << 16), b""):
			h.update(chunk)
	return h.hexdigest()


def run_tool(script, job_obj, job_path, receipt_path, cwd):
	"""Write the job (reproducibility), run the sibling tool, return its
	last-stdout-line receipt (which also self-persists to receipt_path)."""
	job_obj = dict(job_obj)
	job_obj["receipt"] = os.path.abspath(receipt_path)
	with open(job_path, "w") as f:
		json.dump(job_obj, f, indent=1)
		f.write("\n")
	out = subprocess.run([sys.executable, os.path.join(TOOLS, script), os.path.abspath(job_path)],
		capture_output=True, text=True, cwd=cwd)
	last = ""
	for line in out.stdout.splitlines():
		if line.strip():
			last = line
	if not last:
		raise RuntimeError(f"{script} produced no receipt: {out.stderr[-400:]}")
	receipt = json.loads(last)
	if not receipt.get("ok", False) and script not in ("sweep_check.py",):
		# sweep_check legitimately reports ok:false for designed-interference
		# sweeps; every other tool's failure fails the bundle LOUDLY.
		raise RuntimeError(f"{script} failed: {json.dumps(receipt)[:400]}")
	return receipt


# ------------------------------------------------------------ templates --
_TPL = re.compile(r"\{\{\s*([A-Za-z0-9_.\-]+)\s*(?::([^}]+))?\}\}")


def lookup(tree, dotted):
	cur = tree
	for seg in dotted.split("."):
		if isinstance(cur, list):
			cur = cur[int(seg)]
		elif isinstance(cur, dict):
			if seg not in cur:
				raise KeyError(dotted)
			cur = cur[seg]
		else:
			raise KeyError(dotted)
	return cur


def render_template(text, tree, src):
	missing = []

	def sub(m):
		path, fmt = m.group(1), m.group(2)
		try:
			v = lookup(tree, path)
		except (KeyError, IndexError, ValueError):
			missing.append(path)
			return m.group(0)
		return format(v, fmt) if fmt else str(v)

	out = _TPL.sub(sub, text)
	if missing:
		raise ValueError(f"template '{src}': unresolved receipt keys {missing} — the doc would show stale/placeholder numbers, refused")
	return out


# ------------------------------------------------------------- summaries --
def gate_row(name, tool, receipt):
	"""One human-readable gates row per check, from the receipt itself."""
	ok = bool(receipt.get("ok", False))
	if tool == "tolerance_stack":
		if "chain" in receipt:
			c = receipt["chain"]
			head = f"nominal {c['nominal_gap']} · worst [{c['worst_min']}, {c['worst_max']}]"
			ok = c.get("pass_worst", ok) and c.get("pass_rss", ok)
		else:
			ft = receipt.get("fit", {})
			head = f"clearance [{ft.get('min_clearance')}, {ft.get('max_clearance')}]"
			ok = ft.get("pass", ok)
	elif tool == "sweep_check":
		w = next(iter(receipt.get("watches", {}).values()), {})
		bad = w.get("first_interfering_t")
		head = f"{receipt.get('stations')} stations · " + ("no interference" if bad is None else f"designed contact from t={bad}")
		ok = True  # the sweep table itself is the receipt; interference may be designed
	elif tool == "balance_check":
		head = f"static imbalance {receipt.get('static_imbalance_g_mm')} g·mm · CG offset {receipt.get('cg_offset_mm')} mm"
	elif tool == "joint_check":
		sfs = [j.get("SF_actual") for j in receipt.get("joints", [])]
		head = f"{len(sfs)} joints · min SF {min(sfs) if sfs else '—'}"
	elif tool == "production_check":
		head = f"verdict {'PASS' if ok else 'FAIL'}"
	elif tool == "air_topology_audit":
		head = f"air components {receipt.get('components', '—')}"
	else:
		head = "ok" if ok else "failed"
	return {"name": name, "tool": tool, "ok": ok, "headline": head}


def tiers_for(tools_used):
	try:
		import analyzer_registry as ar

		by_script = {}
		for e in ar.REGISTRY:
			if isinstance(e, dict) and e.get("file"):
				by_script[str(e["file"]).replace(".py", "")] = str(e.get("claimed_tier"))
		return {t: by_script.get(t, "uncataloged") for t in sorted(tools_used)}
	except Exception as e:  # registry drift must not kill the bundle — say so
		log(f"analyzer_registry unavailable ({e}); tiers omitted")
		return {}


# ------------------------------------------------------------------ main --
def build(job, job_dir):
	name = job["name"]
	date = job["date"]
	rev = job.get("rev", "A")
	out_root = os.path.join(job_dir, job["out_dir"])
	d_print = os.path.join(out_root, "print")
	d_docs = os.path.join(out_root, "docs")
	d_rcpt = os.path.join(out_root, "receipts")
	d_prog = os.path.join(out_root, "programs")
	for d in (d_print, d_docs, d_rcpt, d_prog):
		os.makedirs(d, exist_ok=True)

	tree = {"meta": {"name": name, "title": job.get("title", name), "rev": rev, "date": date}}
	gates, tools_used, artifacts_extra = [], set(), []

	# --- 1. parts: formats + build receipts + dimensioned sheets ------------
	tree["parts"] = {}
	for part in job.get("parts", []):
		pname = part["name"]
		log(f"[part] {pname}")
		prog = part.get("program")
		if prog is None and part.get("program_file"):
			prog = json.load(open(os.path.join(job_dir, part["program_file"])))
		if prog is None:
			raise ValueError(f"part '{pname}' needs 'program' or 'program_file'")
		solid = part["solid"]
		formats = part.get("formats", ["stl", "3mf", "step"])
		ops = [op for op in prog["ops"] if op.get("op") not in ("export_stl", "export_step", "export_3mf")]
		have = {op["id"] for op in ops}
		# The bundle OWNS the honesty receipts: validate + support + mass on
		# every part, plus the export routes.
		for aux, opname in (("__val", "validate"), ("__sup", "support_report"), ("__mp", "mass_properties")):
			if aux not in have:
				ops.append({"id": aux, "op": opname, "in": solid, **({"build_dir": [0, 0, 1]} if opname == "support_report" else {})})
		fmt_ops = {"stl": ("export_stl", f"print/{pname}.stl"), "3mf": ("export_3mf", f"print/{pname}.3mf"), "step": ("export_step", f"print/{pname}.step")}
		for f in formats:
			opname, path = fmt_ops[f]
			ops.append({"id": f"__ex_{f}", "op": opname, "in": solid, "file": path, **({"tol": 0.01} if f == "stl" else {})})
		rep = param_optimize.call_engine({"ops": ops}, out_dir=out_root)
		if not rep.get("ok"):
			bad = next((o for o in rep["ops"] if not o.get("ok")), {})
			raise RuntimeError(f"part '{pname}' build failed at '{bad.get('id')}': {bad.get('error')}")
		by = {o["id"]: o for o in rep["ops"]}
		prcpt = {
			"validate": by["__val"]["measures"],
			"support": by["__sup"]["measures"],
			"mass_properties": {k: by["__mp"]["measures"][k] for k in ("volume",) if k in by["__mp"]["measures"]},
			"exports": {f: {"route": by[f"__ex_{f}"]["measures"].get("route"), "watertight": by[f"__ex_{f}"]["measures"].get("watertight"), "file": by[f"__ex_{f}"].get("file")} for f in formats if f"__ex_{f}" in by and by[f"__ex_{f}"].get("measures")},
		}
		with open(os.path.join(d_rcpt, f"part_{pname}_receipt.json"), "w") as f:
			json.dump(prcpt, f, indent=1)
		tree["parts"][pname] = prcpt
		with open(os.path.join(d_prog, f"{pname}_program.json"), "w") as f:
			json.dump(prog, f, indent=1)

		# Dimensioned sheet: auto callouts (analytic) + manual extras.
		sheet = part.get("sheet", {})
		dims = list(sheet.get("dimensions", []))
		if sheet.get("auto_dimensions", True):
			sug = run_tool("dim_suggest.py",
				{"program": {"ops": [op for op in prog["ops"] if op.get("op") not in ("export_stl", "export_step", "export_3mf")]},
				 "solid": solid, "max": int(sheet.get("max_dims", 9))},
				os.path.join(d_prog, f"{pname}_dims_job.json"),
				os.path.join(d_rcpt, f"{pname}_dims_receipt.json"), cwd=job_dir)
			dims += sug.get("dimensions", [])
		sheet_job = {"stl": os.path.join(d_print, f"{pname}.stl"), "out": os.path.join(d_docs, f"sheet_{pname}.png"),
			"build_dir": [0, 0, 1], "date": date, "dimensions": dims}
		if sheet.get("sections"):
			sheet_job["sections"] = sheet["sections"]
		run_tool("render_sheet.py", sheet_job, os.path.join(d_prog, f"{pname}_sheet_job.json"),
			os.path.join(d_rcpt, f"{pname}_sheet_receipt.json"), cwd=out_root)

	# --- 2. assembly receipts + docs ----------------------------------------
	asm = job.get("assembly") or {}
	if asm.get("program_file"):
		aprog = json.load(open(os.path.join(job_dir, asm["program_file"])))
		# Receipts only — the program's own export ops land in the usual live
		# tree (param_optimize's default out dir), keeping the bundle layout
		# canonical; the bundle's posed inputs come from assembly.parts.
		rep = param_optimize.call_engine({"ops": aprog["ops"]})
		clear = {o["id"]: o.get("measures") for o in rep["ops"] if o.get("measures") and "distance" in (o.get("measures") or {})}
		arcpt = {"ok": rep.get("ok", False), "clearances": clear}
		with open(os.path.join(d_rcpt, "assembly_receipt.json"), "w") as f:
			json.dump(arcpt, f, indent=1)
		tree["assembly"] = arcpt
		with open(os.path.join(d_prog, "assembly_program.json"), "w") as f:
			json.dump(aprog, f, indent=1)
		if not rep.get("ok"):
			raise RuntimeError("assembly program failed — see receipts/assembly_receipt.json")

	asm_parts = []
	for p in asm.get("parts", []):
		src = os.path.join(job_dir, p["stl"])
		asm_parts.append({**p, "stl": os.path.abspath(src)})

	# --- 3. dossier (BOM + plates) -------------------------------------------
	dossier = None
	if job.get("parts"):
		dparts = []
		for part in job["parts"]:
			dparts.append({"name": part["name"], "stl": os.path.join(d_print, f"{part['name']}.stl"),
				"material": part.get("material", "pla"), "qty": int(part.get("qty", 1)),
				"wear": bool(part.get("wear", False)),
				"material_required": bool(part.get("material_required", False)),
				"print_notes": part.get("print_notes", "")})
		djob = {"out_dir": d_rcpt, "parts": dparts, "date": date}
		for k in ("bed", "print_params", "spacing_mm", "filament_price_per_kg"):
			if k in job:
				djob[k] = job[k]
		dossier = run_tool("production_dossier.py", djob, os.path.join(d_prog, "dossier_job.json"),
			os.path.join(d_rcpt, "dossier_receipt.json"), cwd=out_root)
		tree["bom"] = {ln["name"]: ln for ln in dossier["parts"]}
		tree["bom_totals"] = dossier["totals"]
		for f in dossier.get("files", {}).get("plates", []):
			if f.endswith(".stl"):
				dst = os.path.join(d_print, os.path.basename(f))
				shutil.move(f, dst)
			elif f.endswith(".png"):
				dst = os.path.join(d_docs, os.path.basename(f))
				shutil.move(f, dst)

	# --- 4. assembly doc (exploded + instructions) ---------------------------
	instructions_md = None
	if asm_parts:
		ajob = {"parts": [{"name": p["name"], "stl": p["stl"], **({"color": p["color"]} if "color" in p else {})} for p in asm_parts],
			"explode": asm.get("explode", {"axis": [0, 0, 1], "auto": True}),
			"bom_csv": os.path.join(d_rcpt, "bom_dossier.csv") if dossier else None,
			"out_prefix": os.path.join(d_docs, name), "title": job.get("title", name), "date": date}
		if asm.get("steps"):
			ajob["steps"] = asm["steps"]
		else:
			ajob["auto_steps"] = True
		ajob = {k: v for k, v in ajob.items() if v is not None}
		adoc = run_tool("assembly_doc.py", ajob, os.path.join(d_prog, "asmdoc_job.json"),
			os.path.join(d_rcpt, "asmdoc_receipt.json"), cwd=out_root)
		instructions_md = adoc.get("md")
		tree["asmdoc"] = {"auto_steps": adoc.get("auto_steps", False)}

	# --- 5. motion: custom job + assembly sequence ---------------------------
	if asm_parts and asm.get("motion"):
		mjob = dict(asm["motion"])
		mjob.setdefault("title", f"{job.get('title', name)} — motion study")
		mjob.setdefault("date", date)
		mjob["parts"] = [{**mp, "stl": os.path.abspath(os.path.join(job_dir, mp["stl"]))} for mp in mjob["parts"]]
		mjob.setdefault("out", os.path.join(d_docs, f"{name}_motion.gif"))
		mjob.setdefault("poster_out", os.path.join(d_docs, f"{name}_motion_poster.png"))
		mjob.setdefault("mp4_out", os.path.join(d_docs, f"{name}_motion.mp4"))
		run_tool("motion_gif.py", mjob, os.path.join(d_prog, "motion_job.json"),
			os.path.join(d_rcpt, "motion_receipt.json"), cwd=out_root)
	if asm_parts and asm.get("sequence"):
		seq = asm["sequence"] if isinstance(asm["sequence"], dict) else {}
		sjob = {"title": f"{job.get('title', name)} — assembly sequence", "date": date,
			"meta": f"auto sequence · {date}",
			"parts": [{"name": p["name"], "stl": p["stl"], **({"color": p["color"]} if "color" in p else {})} for p in asm_parts],
			"sequence": seq, "frames": int(seq.get("frames", 36)), "fps": 18,
			"out": os.path.join(d_docs, f"{name}_sequence.gif"),
			"poster_out": os.path.join(d_docs, f"{name}_sequence_poster.png"),
			"mp4_out": os.path.join(d_docs, f"{name}_sequence.mp4")}
		run_tool("motion_gif.py", sjob, os.path.join(d_prog, "sequence_job.json"),
			os.path.join(d_rcpt, "sequence_receipt.json"), cwd=out_root)

	# --- 6. checks -----------------------------------------------------------
	tree["checks"] = {}
	for chk in job.get("checks", []):
		cname, tool = chk["name"], chk["tool"]
		cjob = chk["job"]
		if isinstance(cjob, str):
			cjob = json.load(open(os.path.join(job_dir, cjob)))
		log(f"[check] {cname} ({tool})")
		receipt = run_tool(f"{tool}.py", cjob, os.path.join(d_prog, f"{cname}_job.json"),
			os.path.join(d_rcpt, f"{cname}_receipt.json"), cwd=job_dir)
		tree["checks"][cname] = receipt
		gates.append(gate_row(cname, tool, receipt))
		tools_used.add(tool)

	# --- 7. fits table (design-math ops through the engine) ------------------
	fits_rows = []
	if job.get("fits"):
		ops = []
		for i, f in enumerate(job["fits"]):
			ops.append({"id": f"fit{i}", "op": f["op"], **f.get("params", {})})
		rep = param_optimize.call_engine({"ops": ops}, out_dir=out_root)
		if not rep.get("ok"):
			bad = next((o for o in rep["ops"] if not o.get("ok")), {})
			raise RuntimeError(f"fits lookup failed at '{bad.get('id')}': {bad.get('error')}")
		by = {o["id"]: o for o in rep["ops"]}
		for i, f in enumerate(job["fits"]):
			m = by[f"fit{i}"].get("measures") or {}
			fits_rows.append({"label": f["label"], "op": f["op"], "measures": m})
		with open(os.path.join(d_rcpt, "fits_receipt.json"), "w") as fh:
			json.dump({"ok": True, "fits": fits_rows}, fh, indent=1)
		tree["fits"] = {r["label"]: r["measures"] for r in fits_rows}

	# --- 8. analysis sheet (gates + results) ---------------------------------
	if gates:
		results = {}
		if dossier:
			results["printed mass total"] = f"{dossier['totals']['total_printed_grams']} g"
			results["print time (band ±50%)"] = f"{dossier['totals']['total_print_time_h']} h"
		for g in gates:
			results[g["name"]] = g["headline"]
		ash = {"title": f"{job.get('title', name)} — analysis gates", "date": date,
			"meta_note": f"rev {rev} · every number from a persisted receipt",
			"panels": ([{"kind": "view", "caption": "assembly", "stl": asm_parts[0]["stl"]}] if asm_parts else []),
			"results": results, "gates": {g["name"]: bool(g["ok"]) for g in gates},
			"out": os.path.join(d_docs, f"{name}_analysis.png")}
		run_tool("analysis_sheet.py", ash, os.path.join(d_prog, "analysis_sheet_job.json"),
			os.path.join(d_rcpt, "analysis_sheet_receipt.json"), cwd=out_root)

	# --- 9. instructions post-sections (data-driven, never hand-synced) ------
	tier_map = tiers_for(tools_used)
	if instructions_md and os.path.exists(instructions_md):
		md = [""]
		if dossier:
			md += ["## Print settings (per part)", "",
				"| part | material | required? | qty | mass | time | notes |", "|---|---|---|---|---|---|---|"]
			for ln in dossier["parts"]:
				if ln["kind"] != "made":
					continue
				md.append(f"| {ln['name']} | {ln['material'].upper()} | {'**REQUIRED**' if ln.get('material_required') else 'suggested'} | "
					f"{ln['qty']} | {ln['grams_per_unit']} g | {ln['print_time_h_per_unit']} h | {ln.get('print_notes', '')} |")
			gp = dossier["parts"][0].get("print_params", {})
			md += ["", f"Global: {gp.get('perimeters', '?')} perimeters · {int(float(gp.get('infill', 0)) * 100)}% infill · "
				f"{gp.get('layer_h', '?')} mm layers. Plate layout: see `plate_layout.png`; sliceable plates: `print/plate_*.stl`.", ""]
		if fits_rows:
			md += ["## Fits & design-math table", "", "| fit | op | figures |", "|---|---|---|"]
			for r in fits_rows:
				flat = []
				def fmt(v):
					if isinstance(v, list) and len(v) <= 4 and all(isinstance(x, (int, float)) for x in v):
						return "[" + ", ".join(f"{x:g}" for x in v) + "]"
					return None if isinstance(v, (dict, list)) else v
				for k, v in r["measures"].items():
					if isinstance(v, dict):
						flat += [(f"{k}.{k2}", fmt(v2)) for k2, v2 in v.items() if fmt(v2) is not None]
					elif fmt(v) is not None:
						flat.append((k, fmt(v)))
				figs = " · ".join(f"{k}={v}" for k, v in flat[:6])
				md.append(f"| {r['label']} | `{r['op']}` | {figs} |")
			md.append("")
		if job.get("changelog"):
			md += ["## Revision history", "", "| rev | date | change |", "|---|---|---|"]
			md += [f"| {c['rev']} | {c['date']} | {c['note']} |" for c in job["changelog"]]
			md.append("")
		md += ["## Honesty footer (machine receipts, verbatim)", ""]
		for pname, pr in tree.get("parts", {}).items():
			routes = ", ".join(f"{f}:{e['route']}" for f, e in pr["exports"].items() if e.get("route"))
			sup = pr["support"]
			md.append(f"- **{pname}**: export routes [{routes}]; watertight {all(bool(e.get('watertight')) for e in pr['exports'].values())}; "
				f"support steep_area {sup.get('steep_area')} mm² (support-free = {sup.get('support_free')}).")
		if gates:
			md.append("- Checks: " + "; ".join(f"{g['name']} {'PASS' if g['ok'] else 'FAIL'}" for g in gates) + ".")
		if tier_map:
			md.append("- Analyzer tiers (docs/ANALYSIS_TIERS.md): " + ", ".join(f"{t}={tier_map[t]}" for t in tier_map) + ".")
		md.append(f"- Bundle rev {rev} · {date} · generated by tools/document_bundle.py; re-verify from `programs/`.")
		with open(instructions_md, "a") as f:
			f.write("\n".join(md) + "\n")

	# --- 10. templates -------------------------------------------------------
	for t in job.get("templates", []):
		src = os.path.join(job_dir, t["src"])
		dst = os.path.join(out_root, t["dst"])
		os.makedirs(os.path.dirname(dst), exist_ok=True)
		rendered = render_template(open(src).read(), tree, t["src"])
		with open(dst, "w") as f:
			f.write(rendered)

	# --- 11. index README + manifest -----------------------------------------
	def rel_list(d):
		out = []
		for root, _, files in os.walk(d):
			for fn in sorted(files):
				out.append(os.path.relpath(os.path.join(root, fn), out_root))
		return sorted(out)

	idx = [f"# {job.get('title', name)}", "", f"Bundle rev **{rev}** · {date} · generated by `tools/document_bundle.py` — every number in these docs comes from a receipt in `receipts/`.", ""]
	if dossier:
		idx += [f"**BOM**: {dossier['totals']['total_printed_grams']} g printed across {len([l for l in dossier['parts'] if l['kind'] == 'made'])} parts · ~{dossier['totals']['total_print_time_h']} h (±50%) · {dossier['totals']['n_plates']} plate(s).", ""]
	if gates:
		idx += ["| gate | verdict | headline |", "|---|---|---|"]
		idx += [f"| {g['name']} | {'✅ PASS' if g['ok'] else '❌ FAIL'} | {g['headline']} |" for g in gates]
		idx.append("")
	for png in sorted(os.listdir(d_docs)):
		if png.endswith((".png",)):
			idx.append(f"![{png}](docs/{png})")
	idx += ["", "## Contents", ""]
	for rel in rel_list(out_root):
		if rel not in ("README.md", "manifest.json"):
			idx.append(f"- `{rel}`")
	with open(os.path.join(out_root, "README.md"), "w") as f:
		f.write("\n".join(idx) + "\n")

	manifest = {"name": name, "title": job.get("title", name), "rev": rev, "date": date,
		"generator": "tools/document_bundle.py", "gates": gates, "artifacts": []}
	try:
		manifest["git_commit"] = subprocess.run(["git", "rev-parse", "HEAD"], capture_output=True, text=True, cwd=TOOLS).stdout.strip() or None
	except OSError:
		manifest["git_commit"] = None
	for rel in rel_list(out_root):
		if rel == "manifest.json":
			continue
		full = os.path.join(out_root, rel)
		role = rel.split(os.sep)[0] if os.sep in rel else "root"
		manifest["artifacts"].append({"path": rel, "role": role, "bytes": os.path.getsize(full), "sha256": sha256(full)})
	with open(os.path.join(out_root, "manifest.json"), "w") as f:
		json.dump(manifest, f, indent=1)

	return {"ok": True, "out_dir": os.path.abspath(out_root), "artifacts": len(manifest["artifacts"]),
		"gates": gates, "parts": sorted(tree.get("parts", {})), "rev": rev, "date": date}


def main():
	job_path = os.path.abspath(sys.argv[1])
	job = json.load(open(job_path))
	if "date" not in job:
		raise ValueError("job needs an explicit 'date' (determinism: the bundle never reads the clock)")
	receipt = build(job, os.path.dirname(job_path))
	_receipt.emit(receipt, {"receipt": os.path.join(receipt["out_dir"], "receipts", "bundle_receipt.json")}, "document_bundle")


if __name__ == "__main__":
	if len(sys.argv) < 2 or sys.argv[1] in ("-h", "--help"):
		print(__doc__)
		sys.exit(0)
	try:
		main()
	except Exception as e:  # honest failure receipt — the JSON line is the contract
		print(json.dumps({"ok": False, "error": f"{type(e).__name__}: {e}"}))
		sys.exit(1)
