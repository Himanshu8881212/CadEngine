#!/usr/bin/env python3
"""field_report.py — INTAKE for physical field failures ("it broke").

The third capture stream of the LMCAD flywheel. The other two already exist:

  * SOFTWARE friction — `kernel_core::telemetry::log_friction` appends refusals
    and failed gates to `docs/friction_inbox.jsonl` automatically.
  * PRINTER FIT reality — `tools/ingest_calibration.py` turns measured coupons
    into a printer profile.

Missing until now: WHAT HAPPENED TO THE PART IN THE REAL WORLD. "The lug
cracked after two months in the dryer." "The hinge whitened after 200 cycles."
"It warped off the bed." Nothing ingested that, so the shop could not learn
from field failure. This tool is the drop-box; `tools/field_triage.py` is the
analytical half that turns a report into an engineering change.

Corpus: `docs/field_reports.jsonl` (one JSON object per line; line 1 is a
self-describing `_schema` header that readers skip). Schema reference and
workflow: `docs/FIELD_REPORTS.md`.

Usage:
	python3 tools/field_report.py --new [field flags ...]   # non-interactive
	python3 tools/field_report.py --new                     # interactive (tty)
	python3 tools/field_report.py --new --from-json f.json  # merge a JSON blob
	python3 tools/field_report.py --list [--include-examples]
	python3 tools/field_report.py --show <id>
	python3 tools/field_report.py --stats [--include-examples]
	python3 tools/field_report.py --self-test

HONESTY LAW OF THIS FILE: the records shipped in-tree are SYNTHETIC
ILLUSTRATIONS. They carry `"example": true` and ids prefixed `EXAMPLE-`, the
two are cross-checked (a mismatch is a refusal), and `--list`/`--stats` hide
them unless `--include-examples` is passed — so a synthetic line can never be
counted as a real observation. Until the user reports something, the real
corpus is EMPTY and every count says so.

Refusals (exit 1, `{"ok": false, "error": ..., "errors": [...]}` on stdout):
missing required fields, unknown failure_mode / severity / material,
mode-specific evidence gaps (creep with no duration, fatigue with no cycle
count, fracture with no load, delamination with no print orientation, fit
reports with no location), self-contradictory conditions (cycles accumulated
in zero elapsed time, failure observed after the part left service, example
flag disagreeing with the id), out-of-range numbers, and — loudest —
a service temperature above the material's `softening_c`, which is a
CONDITION VIOLATION (the part was run outside its stated envelope), not a
design failure. That last one refuses by DEFAULT and is recorded only with
`--ack-condition-violation`, which stamps `classification:
"condition_violation"` on the record so triage never blames the design for it.
"""

import argparse
import datetime
import json
import re
import subprocess
import sys
import tempfile
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent
REPO_ROOT = TOOLS_DIR.parent
REPORTS_PATH = REPO_ROOT / "docs" / "field_reports.jsonl"
SCHEMA_VERSION = 1
SCHEMA_TAG = "lmcad.field_report"

# ---------------------------------------------------------------------------
# Controlled vocabulary — the whole point of a controlled vocabulary is that
# triage can MAP it (field_triage.py FAILURE_ANALYSIS keys these exactly).
# ---------------------------------------------------------------------------
FAILURE_MODES = {
	"fracture": "part broke apart under load (brittle or ductile), one event",
	"creep_deformation": "part slowly changed shape under a sustained load (sagged, flowed, lost preload)",
	"layer_delamination": "part split ALONG a layer line / interlayer bond",
	"wear": "material removed by rubbing/abrasion over use (grooves, slop, polish)",
	"fatigue_crack": "crack grew under repeated cycles, no single overload event",
	"warping": "part distorted from heat or residual stress (lifted off the bed, bowed in service)",
	"chemical_uv": "environmental attack — solvent, oil, moisture/hydrolysis, sunlight/UV, ozone",
	"fit_loose": "mating fit ended up looser than intended (rattles, falls out, no retention)",
	"fit_tight": "mating fit ended up tighter than intended (will not assemble, cracked on insert)",
	"other": "none of the above — REQUIRES a long description; triage refuses to guess",
}

SEVERITY_RANK = {
	"cosmetic": 0,        # looks wrong, works fine
	"degraded": 1,        # works worse (slop, noise, stiffness)
	"functional_loss": 2, # stopped doing its job
	"safety": 3,          # released stored energy / dropped a load / hot or sharp
}

CLASS_DESIGN = "design_failure"
CLASS_CONDITION = "condition_violation"

# Physically sane service envelope for an FDM part; anything outside is a typo
# or a different physics problem, and either way not a service condition.
TEMP_MIN_C, TEMP_MAX_C = -40.0, 300.0

ID_RE = re.compile(r"^[A-Z0-9][A-Z0-9._-]*$")
DATE_RE = re.compile(r"^\d{4}-\d{2}-\d{2}$")


# ---------------------------------------------------------------------------
# Material access (softening_c etc.) — tolerant of a concurrently-edited record
# ---------------------------------------------------------------------------
def material_record(name):
	"""Resolve a material record by name. Prefers `tools/materials.py` (the
	unified source of truth, which validates the content hash); falls back to a
	direct read of `tools/materials/<key>.json` when the module or its hash
	check is unavailable, and SAYS which path was used. Returns
	(record | None, how)."""
	key = str(name).strip().upper().replace("-", "").replace(" ", "")
	try:
		sys.path.insert(0, str(TOOLS_DIR))
		import _layout  # noqa: PLC0415
		_layout.add_import_paths()  # materials.py lives in tools/analyzers/ since 2026-09-02
		import materials  # noqa: PLC0415 — deliberately lazy/optional
		return materials.get(key).record, "materials.py"
	except Exception:
		pass
	finally:
		if sys.path and sys.path[0] == str(TOOLS_DIR):
			sys.path.pop(0)
	aliases = {"TPU": "TPU95A", "TPU95": "TPU95A", "NYLON": "PA", "PETG": "PETG", "PET-G": "PETG"}
	key = aliases.get(key, key)
	path = TOOLS_DIR / "materials" / f"{key.lower()}.json"
	if path.is_file():
		try:
			return json.loads(path.read_text(encoding="utf-8")), f"raw {path.name}"
		except Exception:
			return None, "unreadable"
	return None, "unknown"


def available_materials():
	d = TOOLS_DIR / "materials"
	return sorted(p.stem.upper() for p in d.glob("*.json")) if d.is_dir() else []


# ---------------------------------------------------------------------------
# Corpus I/O
# ---------------------------------------------------------------------------
def header_line():
	return {
		"_schema": SCHEMA_TAG,
		"schema_version": SCHEMA_VERSION,
		"note": (
			"Line 1 is this header and is skipped by readers. Every other line is one field "
			"report. Records with \"example\": true and an id prefixed EXAMPLE- are SYNTHETIC "
			"ILLUSTRATIONS shipped with the tooling to exercise the pipeline — they are NOT real "
			"observations and must never be counted as field data."
		),
		"doc": "docs/FIELD_REPORTS.md",
		"tools": ["tools/field_report.py", "tools/field_triage.py"],
	}


def load_reports(path=None):
	"""Read the corpus. Returns (records, header). Blank lines and the `_schema`
	header are skipped; a malformed line raises (a corrupt corpus must be loud,
	not silently short)."""
	p = Path(path) if path else REPORTS_PATH
	records, header = [], None
	if not p.is_file():
		return records, header
	for n, raw in enumerate(p.read_text(encoding="utf-8").splitlines(), start=1):
		line = raw.strip()
		if not line or line.startswith("#"):
			continue
		try:
			obj = json.loads(line)
		except json.JSONDecodeError as exc:
			raise ValueError(f"{p}:{n}: not valid JSON ({exc.msg}) — the corpus is append-only JSONL") from exc
		if isinstance(obj, dict) and obj.get("_schema") == SCHEMA_TAG:
			header = obj
			continue
		records.append(obj)
	return records, header


def append_report(record, path=None):
	p = Path(path) if path else REPORTS_PATH
	p.parent.mkdir(parents=True, exist_ok=True)
	fresh = not p.is_file() or not p.read_text(encoding="utf-8").strip()
	with p.open("a", encoding="utf-8") as fh:
		if fresh:
			fh.write(json.dumps(header_line(), ensure_ascii=False) + "\n")
		fh.write(json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n")
	return p


# ---------------------------------------------------------------------------
# Validation — every refusal names the field AND what would fix it
# ---------------------------------------------------------------------------
def _num(errors, block, key, label, *, lo=None, hi=None, integer=False):
	"""Fetch an optional numeric; append a refusal on a bad type/range."""
	v = block.get(key)
	if v is None:
		return None
	if isinstance(v, bool) or not isinstance(v, (int, float)):
		errors.append(f"{label}: expected a number, got {v!r}")
		return None
	v = int(v) if integer else float(v)
	if lo is not None and v < lo:
		errors.append(f"{label}: {v} is below the physical floor {lo}")
		return None
	if hi is not None and v > hi:
		errors.append(f"{label}: {v} is above the ceiling {hi}")
		return None
	return v


def validate(record, *, ack_condition_violation=False):
	"""Validate one report. Returns (errors, notes, classification, violation).

	`errors` non-empty ⇒ the report is REFUSED. `violation` is the
	condition-violation detail dict (or None); it is only fatal when the caller
	did not acknowledge it."""
	errors, notes = [], []
	if not isinstance(record, dict):
		return ([f"report must be a JSON object, got {type(record).__name__}"], notes, None, None)

	part = record.get("part") or {}
	process = record.get("process") or {}
	service = record.get("service") or {}
	obs = record.get("observation") or {}
	for name, block in (("part", part), ("process", process), ("service", service), ("observation", obs)):
		if not isinstance(block, dict):
			errors.append(f"{name}: must be a JSON object, got {type(block).__name__}")
			return (errors, notes, None, None)

	# --- identity -----------------------------------------------------------
	rid = str(record.get("id") or "").strip()
	if not rid:
		errors.append("id: required — a stable handle for this report (e.g. FIELD-2026-07-30-A)")
	elif not ID_RE.match(rid):
		errors.append(f"id: {rid!r} — use uppercase letters, digits, '.', '-', '_' only")
	is_example = record.get("example")
	if not isinstance(is_example, bool):
		errors.append("example: required boolean — true ONLY for the synthetic records shipped with the tooling")
	else:
		starts = rid.startswith("EXAMPLE-")
		if is_example and not starts:
			errors.append(f"example=true but id {rid!r} does not start with 'EXAMPLE-' — a synthetic record must be unmistakable")
		if starts and not is_example:
			errors.append(f"id {rid!r} claims EXAMPLE- but example=false — contradictory provenance; pick one")
	reported = str(record.get("reported") or "")
	if not DATE_RE.match(reported):
		errors.append(f"reported: {reported!r} — required, ISO date YYYY-MM-DD (when the failure was REPORTED)")

	# --- identity of the part ----------------------------------------------
	family = str(part.get("family") or "").strip()
	entry = str(part.get("entry") or "").strip()
	if not family:
		errors.append("part.family: required — the campaign family (e.g. 'spool' for spool_system/)")
	if not entry:
		errors.append("part.entry: required — the campaign entry (e.g. 'respool' for spool_system/respool/)")

	# --- material / process -------------------------------------------------
	material = str(process.get("material") or "").strip()
	mat, how = (None, "unknown")
	if not material:
		errors.append(f"process.material: required — one of {', '.join(available_materials()) or '(no records found)'}")
	else:
		mat, how = material_record(material)
		if mat is None:
			errors.append(
				f"process.material: unknown material {material!r}; available: "
				f"{', '.join(available_materials()) or '(no records in tools/materials/)'}"
			)
		else:
			notes.append(f"material {mat['meta']['name']} v{mat['meta']['version']} resolved via {how}")

	# --- observation --------------------------------------------------------
	mode = str(obs.get("failure_mode") or "").strip()
	if mode not in FAILURE_MODES:
		errors.append(
			f"observation.failure_mode: unknown mode {mode!r}. The controlled vocabulary is: "
			+ ", ".join(sorted(FAILURE_MODES))
			+ " — pick the closest, or 'other' with a long description."
		)
	text = str(obs.get("text") or "").strip()
	if len(text) < 15:
		errors.append("observation.text: required — describe WHAT failed, WHERE, and WHEN (at least 15 characters)")
	if mode == "other" and len(text) < 60:
		errors.append("observation.failure_mode='other' requires ≥60 characters of description — 'other' is not a shortcut past thinking")
	severity = str(record.get("severity") or "").strip()
	if severity not in SEVERITY_RANK:
		errors.append(f"severity: {severity!r} — one of {', '.join(SEVERITY_RANK)}")
	photo = record.get("photo")
	if photo is not None and not str(photo).strip():
		errors.append("photo: give a path or omit the field entirely")

	# --- service conditions -------------------------------------------------
	temp_c = _num(errors, service, "temp_c", "service.temp_c", lo=TEMP_MIN_C, hi=TEMP_MAX_C)
	duration_h = _num(errors, service, "duration_h", "service.duration_h", lo=0.0)
	cycles = _num(errors, service, "cycles", "service.cycles", lo=0, integer=True)
	load_n = _num(errors, service, "load_n", "service.load_n", lo=0.0)
	humidity = _num(errors, service, "humidity_pct", "service.humidity_pct", lo=0.0, hi=100.0)
	first_seen = _num(errors, obs, "first_observed_h", "observation.first_observed_h", lo=0.0)
	if not str(service.get("environment") or "").strip():
		errors.append("service.environment: required — where the part lived (e.g. 'filament dryer, continuous 45 °C', 'outdoors, south-facing')")

	# --- self-contradiction -------------------------------------------------
	if cycles is not None and cycles > 0 and duration_h is not None and duration_h == 0.0:
		errors.append(
			f"contradictory conditions: service.cycles={cycles} but service.duration_h=0 — "
			"cycles cannot accumulate in zero elapsed time; give the real service duration"
		)
	if first_seen is not None and duration_h is not None and first_seen > duration_h:
		errors.append(
			f"contradictory conditions: observation.first_observed_h={first_seen} exceeds "
			f"service.duration_h={duration_h} — the failure cannot be observed after the part left service"
		)

	# --- mode-specific evidence requirements --------------------------------
	# Each one exists because triage CANNOT produce a remediation without it.
	if mode == "creep_deformation" and not duration_h:
		errors.append(
			"creep_deformation requires service.duration_h (>0): creep allowables are time-derated "
			"(tools/materials/*.json creep.sig_allow_mpa is a temperature × duration table). "
			"Without a duration there is no allowable to check against."
		)
	if mode == "fatigue_crack" and not cycles:
		errors.append(
			"fatigue_crack requires service.cycles (>0): the fatigue allowable is "
			"ultimate × sn_curve.fraction_of_ultimate at a stated cycle count. "
			"Without cycles this is a fracture report, not a fatigue report."
		)
	if mode == "wear" and not cycles and not duration_h:
		errors.append("wear requires service.cycles or service.duration_h — wear is a rate, and a rate needs a denominator")
	if mode == "fracture" and load_n is None:
		errors.append(
			"fracture requires service.load_n — estimate it (mass × 9.81 is fine, say so in the text). "
			"A fracture report without a load cannot be turned into a static strength margin."
		)
	if mode == "layer_delamination" and not str(process.get("orientation_note") or "").strip():
		errors.append(
			"layer_delamination requires process.orientation_note — how the part was printed "
			"(which face on the bed). The entire remediation is an orientation/interlayer allowable, "
			"and it cannot be derived without knowing where the layers ran."
		)
	if mode == "warping" and temp_c is None:
		errors.append(
			"warping requires service.temp_c — the temperature the part saw (for a bed-warp report, "
			"the bed/chamber temperature). Warping is a thermal event; without a temperature it is a shrug."
		)
	if mode == "chemical_uv" and not duration_h:
		errors.append("chemical_uv requires service.duration_h — environmental attack is a dose, and a dose needs an exposure time")
	if mode in ("fit_loose", "fit_tight") and not str(obs.get("location") or "").strip():
		errors.append(
			f"{mode} requires observation.location — WHICH feature (bore, boss, slot, snap). "
			"A fit remediation is a tolerance on one named feature or it is nothing."
		)

	# --- CONDITION VIOLATION: run outside the material's stated envelope -----
	violation = None
	if mat is not None and temp_c is not None:
		soft = (mat.get("thermal") or {}).get("softening_c")
		if soft is None:
			notes.append(
				f"{mat['meta']['name']}: thermal.softening_c is null in the record — the service-temperature "
				"envelope check was SKIPPED (stated, not silently passed)"
			)
		elif temp_c > float(soft):
			violation = {
				"field": "service.temp_c",
				"value": temp_c,
				"limit_c": float(soft),
				"limit_field": "thermal.softening_c",
				"material": mat["meta"]["name"],
				"note": (
					f"service temperature {temp_c} °C is above {mat['meta']['name']}'s softening_c "
					f"{float(soft)} °C — the part was operated OUTSIDE its material envelope. This is a "
					"CONDITION VIOLATION, not a design failure: the design is not at fault above its "
					"stated envelope. The remediation is a material/process change or a service-limit "
					"warning, never a geometry derate."
				),
			}
			if not ack_condition_violation:
				errors.append(
					f"CONDITION VIOLATION: {violation['note']} "
					"Re-file with --ack-condition-violation to record it as a condition violation."
				)

	classification = None
	if not errors:
		classification = CLASS_CONDITION if violation else CLASS_DESIGN
	return errors, notes, classification, violation


# ---------------------------------------------------------------------------
# Record construction
# ---------------------------------------------------------------------------
def _git_commit():
	try:
		out = subprocess.run(
			["git", "rev-parse", "--short", "HEAD"],
			cwd=str(REPO_ROOT), capture_output=True, text=True, timeout=10, check=False,
		)
		return out.stdout.strip() or None
	except Exception:
		return None


def build_record(args, base=None):
	"""Assemble a record from an optional JSON base plus CLI flags (flags win)."""
	rec = json.loads(json.dumps(base)) if base else {}
	rec.setdefault("schema_version", SCHEMA_VERSION)
	part = rec.setdefault("part", {})
	process = rec.setdefault("process", {})
	service = rec.setdefault("service", {})
	obs = rec.setdefault("observation", {})

	def put(block, key, value):
		if value is not None:
			block[key] = value

	put(rec, "id", args.id)
	if args.example:
		rec["example"] = True
	rec.setdefault("example", False)
	rec["reported"] = args.reported or rec.get("reported") or datetime.date.today().isoformat()
	put(part, "family", args.family)
	put(part, "entry", args.entry)
	put(part, "part", args.part)
	commit = args.commit if args.commit is not None else part.get("commit") or _git_commit()
	put(part, "commit", commit)
	put(process, "material", args.material)
	process.setdefault("process", args.process or "fdm")
	put(process, "layer_h_mm", args.layer_h_mm)
	put(process, "walls", args.walls)
	put(process, "infill_pct", args.infill_pct)
	put(process, "orientation_note", args.orientation_note)
	put(service, "environment", args.environment)
	put(service, "temp_c", args.temp_c)
	put(service, "duration_h", args.duration_h)
	put(service, "cycles", args.cycles)
	put(service, "load_n", args.load_n)
	put(service, "humidity_pct", args.humidity_pct)
	put(obs, "failure_mode", args.failure_mode)
	put(obs, "location", args.location)
	put(obs, "first_observed_h", args.first_observed_h)
	put(obs, "text", args.text)
	put(rec, "severity", args.severity)
	put(rec, "photo", args.photo)
	return rec


def interactive(args):
	"""Prompt for the fields when --new is given with nothing else on a tty."""
	def ask(prompt, default=None, required=False):
		suffix = f" [{default}]" if default is not None else ""
		while True:
			val = input(f"{prompt}{suffix}: ").strip()
			if not val and default is not None:
				return default
			if val or not required:
				return val or None
			print("  (required)")

	def ask_num(prompt, integer=False):
		while True:
			val = input(f"{prompt} (blank = unknown): ").strip()
			if not val:
				return None
			try:
				return int(val) if integer else float(val)
			except ValueError:
				print("  (needs a number)")

	print("LMCAD field report — intake. Blank answers stay UNKNOWN; unknown is a")
	print("first-class answer, but triage will refuse the modes that need the datum.\n")
	print("failure modes: " + ", ".join(sorted(FAILURE_MODES)))
	print("severities:    " + ", ".join(SEVERITY_RANK) + "\n")
	args.id = args.id or ask("report id (e.g. FIELD-2026-07-30-A)", required=True)
	args.family = args.family or ask("campaign family (spool, drawer, ...)", required=True)
	args.entry = args.entry or ask("campaign entry (respool, drybox_roller, ...)", required=True)
	args.part = args.part or ask("which piece failed (free text)")
	args.commit = args.commit or ask("git commit / version the part was built from", default=_git_commit() or "")
	args.material = args.material or ask("material (PLA, PETG, ...)", required=True)
	args.process = args.process or ask("process", default="fdm")
	args.orientation_note = args.orientation_note or ask("print orientation (which face on the bed)")
	args.environment = args.environment or ask("service environment (where it lived)", required=True)
	args.temp_c = args.temp_c if args.temp_c is not None else ask_num("service temperature °C")
	args.duration_h = args.duration_h if args.duration_h is not None else ask_num("service duration, hours")
	args.cycles = args.cycles if args.cycles is not None else ask_num("load/motion cycles", integer=True)
	args.load_n = args.load_n if args.load_n is not None else ask_num("load, N")
	args.humidity_pct = args.humidity_pct if args.humidity_pct is not None else ask_num("relative humidity, %")
	args.failure_mode = args.failure_mode or ask("failure mode", required=True)
	args.location = args.location or ask("location on the part")
	args.first_observed_h = args.first_observed_h if args.first_observed_h is not None else ask_num("first observed at, service hours")
	args.text = args.text or ask("what happened (free text)", required=True)
	args.severity = args.severity or ask("severity", required=True)
	args.photo = args.photo or ask("photo path")
	return args


# ---------------------------------------------------------------------------
# Output helpers
# ---------------------------------------------------------------------------
def fail(errors):
	errs = [errors] if isinstance(errors, str) else list(errors)
	print("REFUSED — the report was not recorded:", file=sys.stderr)
	for e in errs:
		print(f"  - {e}", file=sys.stderr)
	print(json.dumps({"ok": False, "error": errs[0], "errors": errs}, ensure_ascii=False, indent=2))
	sys.exit(1)


def summarize(rec):
	part = rec.get("part", {})
	obs = rec.get("observation", {})
	svc = rec.get("service", {})
	who = f"{part.get('family','?')}/{part.get('entry','?')}"
	if part.get("part"):
		who += f" · {part['part']}"
	cond = []
	for key, unit in (("temp_c", "°C"), ("duration_h", "h"), ("cycles", "cyc"), ("load_n", "N"), ("humidity_pct", "%RH")):
		if svc.get(key) is not None:
			cond.append(f"{svc[key]:g} {unit}")
	return (
		f"{rec.get('id','?'):<22} {rec.get('reported','?')}  {who}\n"
		f"  mode      {obs.get('failure_mode','?')}   severity {rec.get('severity','?')}   "
		f"class {rec.get('classification','?')}{'   [EXAMPLE — synthetic]' if rec.get('example') else ''}\n"
		f"  material  {rec.get('process',{}).get('material','?')}   conditions: {', '.join(cond) or '(none given)'}\n"
		f"  where     {obs.get('location') or '(unstated)'}\n"
		f"  what      {obs.get('text','')}"
	)


def cmd_list(args):
	records, _ = load_reports(args.corpus)
	shown = [r for r in records if args.include_examples or not r.get("example")]
	real = [r for r in records if not r.get("example")]
	print(f"{'id':<22} {'date':<11} {'campaign':<26} {'mode':<19} {'mat':<6} {'severity':<15} class")
	print("-" * 116)
	for r in shown:
		part = r.get("part", {})
		print(
			f"{str(r.get('id','?')):<22} {str(r.get('reported','?')):<11} "
			f"{part.get('family','?') + '/' + part.get('entry','?'):<26} "
			f"{str(r.get('observation',{}).get('failure_mode','?')):<19} "
			f"{str(r.get('process',{}).get('material','?')):<6} {str(r.get('severity','?')):<15} "
			f"{r.get('classification','?')}{'  [EXAMPLE]' if r.get('example') else ''}"
		)
	if not shown:
		print("(none)")
	print(f"\n{len(real)} REAL report(s); {len(records) - len(real)} synthetic example(s) shipped with the tooling.")
	if not real:
		print("The real corpus is EMPTY — nothing has been reported from the field yet.")
	print(json.dumps({"ok": True, "real": len(real), "examples": len(records) - len(real)}))


def cmd_show(args):
	records, _ = load_reports(args.corpus)
	hit = [r for r in records if str(r.get("id")) == args.show]
	if not hit:
		fail([f"no report with id {args.show!r} in {args.corpus or REPORTS_PATH}"])
	for r in hit:
		print(summarize(r))
		print()
		print(json.dumps(r, ensure_ascii=False, indent=2, sort_keys=True))


def cmd_stats(args):
	records, _ = load_reports(args.corpus)
	real = [r for r in records if not r.get("example")]
	pool = records if args.include_examples else real
	tag = "INCLUDING synthetic examples" if args.include_examples else "real reports only"

	def bump(d, key):
		d[key] = d.get(key, 0) + 1

	by_mode, by_mat, by_campaign, cross, by_class, by_sev = {}, {}, {}, {}, {}, {}
	for r in pool:
		mode = r.get("observation", {}).get("failure_mode", "?")
		mat = r.get("process", {}).get("material", "?")
		camp = f"{r.get('part',{}).get('family','?')}/{r.get('part',{}).get('entry','?')}"
		bump(by_mode, mode)
		bump(by_mat, mat)
		bump(by_campaign, camp)
		bump(cross, f"{mode} × {mat} × {camp}")
		bump(by_class, r.get("classification", "?"))
		bump(by_sev, r.get("severity", "?"))

	print(f"FIELD REPORT STATS ({tag}) — {len(pool)} record(s)")
	print(f"corpus: {args.corpus or REPORTS_PATH}")
	if not real:
		print("\n*** The REAL corpus is EMPTY. Nothing below is a measurement of anything that")
		print("*** happened to a physical part; the shipped records are synthetic illustrations.")
	for title, table in (
		("by failure_mode", by_mode), ("by material", by_mat), ("by campaign", by_campaign),
		("by severity", by_sev), ("by classification", by_class),
		("failure_mode × material × campaign", cross),
	):
		print(f"\n{title}:")
		if not table:
			print("  (none)")
		for key in sorted(table, key=lambda k: (-table[k], k)):
			print(f"  {table[key]:>4}  {key}")
	print()
	print(json.dumps({
		"ok": True, "counted": len(pool), "real": len(real),
		"examples": len(records) - len(real), "include_examples": bool(args.include_examples),
		"by_failure_mode": by_mode, "by_material": by_mat, "by_campaign": by_campaign,
		"by_severity": by_sev, "by_classification": by_class, "cross": cross,
	}, ensure_ascii=False, indent=2, sort_keys=True))


def cmd_new(args):
	base = None
	if args.from_json:
		src = sys.stdin.read() if args.from_json == "-" else Path(args.from_json).read_text(encoding="utf-8")
		base = json.loads(src)
	interactive_wanted = (
		not args.from_json
		and not any((args.id, args.family, args.entry, args.material, args.failure_mode, args.text))
	)
	if interactive_wanted:
		if not sys.stdin.isatty():
			fail([
				"--new with no field flags needs a terminal to prompt on. Either run it interactively, "
				"pass the flags (--id --family --entry --material --failure-mode --text --severity "
				"--environment ...), or feed a record with --from-json <file|->."
			])
		args = interactive(args)
	rec = build_record(args, base)
	errors, notes, classification, violation = validate(rec, ack_condition_violation=args.ack_condition_violation)
	if errors:
		fail(errors)
	rec["classification"] = classification
	if violation:
		rec["condition_violation"] = violation
		print("*" * 78)
		print("CONDITION VIOLATION — recorded as such, NOT as a design failure.")
		print(violation["note"])
		print("*" * 78)
	for n in notes:
		print(f"note: {n}")
	if args.dry_run:
		print("\n--dry-run: validated, NOT appended.\n")
		print(summarize(rec))
		print(json.dumps({"ok": True, "dry_run": True, "id": rec["id"], "classification": classification,
		                  "record": rec}, ensure_ascii=False, indent=2, sort_keys=True))
		return
	existing, _ = load_reports(args.corpus)
	if any(str(r.get("id")) == rec["id"] for r in existing):
		fail([f"id {rec['id']!r} already exists in {args.corpus or REPORTS_PATH} — ids are unique handles; pick another"])
	path = append_report(rec, args.corpus)
	print()
	print(summarize(rec))
	print(f"\nappended to {path}")
	print(f"next: python3 tools/field_triage.py --id {rec['id']}   # report → engineering consequence")
	print(json.dumps({"ok": True, "id": rec["id"], "classification": classification, "appended": str(path)},
	                 ensure_ascii=False, indent=2, sort_keys=True))


# ---------------------------------------------------------------------------
# Self-test — provokes every refusal and pins the shipped corpus
# ---------------------------------------------------------------------------
def _base_record(**over):
	rec = {
		"id": "SELFTEST-001", "example": False, "reported": "2026-07-30", "schema_version": SCHEMA_VERSION,
		"part": {"family": "spool", "entry": "respool", "part": "hub tongue", "commit": "deadbee"},
		"process": {"material": "PLA", "process": "fdm", "orientation_note": "flange face down on the bed"},
		"service": {"environment": "filament dryer", "temp_c": 45.0, "duration_h": 720.0, "load_n": 12.0, "cycles": 0},
		"observation": {"failure_mode": "creep_deformation", "location": "lug root",
		                "first_observed_h": 700.0, "text": "self-test record, not a real observation"},
		"severity": "degraded",
	}
	for path, value in over.items():
		keys = path.split(".")
		node = rec
		for k in keys[:-1]:
			node = node.setdefault(k, {})
		if value is None:
			node.pop(keys[-1], None)
		else:
			node[keys[-1]] = value
	return rec


def self_test():
	checks = []

	def check(label, cond, detail=""):
		checks.append((label, bool(cond), detail))

	ok_errors, _, cls, viol = validate(_base_record())
	check("a complete report validates", not ok_errors, "; ".join(ok_errors))
	check("complete report classifies as design_failure", cls == CLASS_DESIGN, str(cls))
	check("45 °C PLA is inside the envelope (no violation)", viol is None, str(viol))

	cases = [
		("incomplete: no material", _base_record(**{"process.material": None}), "process.material: required"),
		("incomplete: no id", _base_record(id=None), "id: required"),
		("incomplete: no environment", _base_record(**{"service.environment": None}), "service.environment: required"),
		("unknown failure_mode", _base_record(**{"observation.failure_mode": "exploded"}), "unknown mode"),
		("unknown severity", _base_record(severity="very bad"), "severity:"),
		("unknown material", _base_record(**{"process.material": "unobtanium"}), "unknown material"),
		("creep with no duration", _base_record(**{"service.duration_h": None, "observation.first_observed_h": None}),
		 "creep_deformation requires service.duration_h"),
		("fatigue with no cycles", _base_record(**{"observation.failure_mode": "fatigue_crack", "service.cycles": None}),
		 "fatigue_crack requires service.cycles"),
		("fracture with no load", _base_record(**{"observation.failure_mode": "fracture", "service.load_n": None}),
		 "fracture requires service.load_n"),
		("delamination with no orientation", _base_record(**{"observation.failure_mode": "layer_delamination",
		                                                     "process.orientation_note": None}),
		 "layer_delamination requires process.orientation_note"),
		("fit report with no location", _base_record(**{"observation.failure_mode": "fit_loose",
		                                                "observation.location": None}),
		 "fit_loose requires observation.location"),
		("warping with no temperature", _base_record(**{"observation.failure_mode": "warping", "service.temp_c": None}),
		 "warping requires service.temp_c"),
		("'other' without a real description", _base_record(**{"observation.failure_mode": "other"}),
		 "requires ≥60 characters"),
		("contradiction: cycles in zero time", _base_record(**{"service.cycles": 500, "service.duration_h": 0.0,
		                                                       "observation.first_observed_h": None}),
		 "cycles cannot accumulate in zero elapsed time"),
		("contradiction: observed after service ended", _base_record(**{"observation.first_observed_h": 900.0}),
		 "cannot be observed after the part left service"),
		("contradiction: example flag vs id", _base_record(example=True), "does not start with 'EXAMPLE-'"),
		("contradiction: EXAMPLE id without the flag", _base_record(id="EXAMPLE-999"), "contradictory provenance"),
		("out of range: humidity 150 %", _base_record(**{"service.humidity_pct": 150.0}), "above the ceiling"),
		("out of range: 900 °C service", _base_record(**{"service.temp_c": 900.0}), "above the ceiling"),
		("condition violation refused by default", _base_record(**{"service.temp_c": 70.0}), "CONDITION VIOLATION"),
	]
	for label, rec, needle in cases:
		errs, _, _, _ = validate(rec)
		hit = any(needle in e for e in errs)
		check(f"REFUSES {label}", errs and hit, f"errors={errs!r}")

	errs, _, cls, viol = validate(_base_record(**{"service.temp_c": 70.0}), ack_condition_violation=True)
	check("acknowledged condition violation is accepted", not errs, "; ".join(errs))
	check("...and classified condition_violation, not design_failure", cls == CLASS_CONDITION, str(cls))
	check("...with the material limit named", viol and viol.get("limit_c") == 55.0 and viol.get("material") == "PLA", str(viol))

	# round trip through a temp corpus
	with tempfile.TemporaryDirectory() as td:
		tmp = Path(td) / "field_reports.jsonl"
		append_report(_base_record(), tmp)
		append_report(_base_record(id="SELFTEST-002"), tmp)
		back, hdr = load_reports(tmp)
		check("append/read round-trip keeps 2 records", len(back) == 2, str(len(back)))
		check("header line is written and skipped", hdr is not None and hdr.get("_schema") == SCHEMA_TAG, str(hdr))

	# the shipped corpus must be valid AND unmistakably synthetic
	shipped, hdr = load_reports()
	check("shipped corpus exists", bool(shipped), f"{len(shipped)} records at {REPORTS_PATH}")
	check("shipped corpus has the schema header", hdr is not None and hdr.get("_schema") == SCHEMA_TAG, str(hdr))
	bad = [r.get("id") for r in shipped if not r.get("example") or not str(r.get("id", "")).startswith("EXAMPLE-")]
	check("EVERY shipped record is a labelled example", not bad, f"non-example ids: {bad}")
	for r in shipped:
		errs, _, cls, _ = validate(r, ack_condition_violation=True)
		check(f"shipped {r.get('id')} validates", not errs, "; ".join(errs))
		check(f"shipped {r.get('id')} classification matches revalidation",
		      r.get("classification") == cls, f"{r.get('classification')} vs {cls}")

	failed = [(l, d) for l, c, d in checks if not c]
	width = max(len(l) for l, _, _ in checks)
	for label, cond, detail in checks:
		print(f"  [{'OK' if cond else 'FAIL'}] {label:<{width}}  {detail if not cond else ''}")
	print(f"\n{len(checks) - len(failed)}/{len(checks)} checks passed")
	if failed:
		print(json.dumps({"ok": False, "error": f"{len(failed)} self-test check(s) failed",
		                  "errors": [f"{l}: {d}" for l, d in failed]}, ensure_ascii=False, indent=2))
		sys.exit(1)
	print(json.dumps({"ok": True, "self_test": "PASS", "checks": len(checks),
	                  "shipped_records": len(shipped), "real_records": 0}, indent=2))


# ---------------------------------------------------------------------------
def main(argv):
	p = argparse.ArgumentParser(
		prog="field_report.py",
		description="Intake for physical field failures — the third capture stream of the LMCAD flywheel.",
		epilog="failure modes: " + "; ".join(f"{k} = {v}" for k, v in sorted(FAILURE_MODES.items())),
	)
	mode = p.add_mutually_exclusive_group(required=True)
	mode.add_argument("--new", action="store_true", help="create a report (interactive on a tty, else flag-driven)")
	mode.add_argument("--list", action="store_true", help="list reports")
	mode.add_argument("--show", metavar="ID", help="print one report in full")
	mode.add_argument("--stats", action="store_true", help="counts by failure_mode × material × campaign")
	mode.add_argument("--self-test", dest="self_test", action="store_true", help="run the validation gates")
	p.add_argument("--corpus", help=f"corpus path (default {REPORTS_PATH})")
	p.add_argument("--include-examples", action="store_true", help="count/list the synthetic example records too")
	p.add_argument("--dry-run", action="store_true", help="validate and print, do not append")
	p.add_argument("--from-json", metavar="PATH", help="read a record from a JSON file ('-' = stdin); flags override")
	p.add_argument("--ack-condition-violation", action="store_true",
	               help="acknowledge that the part was run outside its material envelope and record it as such")
	g = p.add_argument_group("report fields")
	g.add_argument("--id")
	g.add_argument("--example", action="store_true", help="mark as a SYNTHETIC example (id must start with EXAMPLE-)")
	g.add_argument("--reported", help="ISO date YYYY-MM-DD (default: today)")
	g.add_argument("--family", help="campaign family, e.g. spool")
	g.add_argument("--entry", help="campaign entry, e.g. respool")
	g.add_argument("--part", help="which piece failed")
	g.add_argument("--commit", help="git commit / version the part was built from (default: current HEAD)")
	g.add_argument("--material", help="PLA, PETG, ABS, ASA, PC, PA, TPU95A")
	g.add_argument("--process", help="fdm (default), sla, ...")
	g.add_argument("--layer-h-mm", type=float)
	g.add_argument("--walls", type=int)
	g.add_argument("--infill-pct", type=float)
	g.add_argument("--orientation-note", help="how it was printed — which face on the bed")
	g.add_argument("--environment", help="where the part lived")
	g.add_argument("--temp-c", type=float)
	g.add_argument("--duration-h", type=float)
	g.add_argument("--cycles", type=int)
	g.add_argument("--load-n", type=float)
	g.add_argument("--humidity-pct", type=float)
	# Deliberately NOT argparse `choices`: an unknown mode/severity must come
	# back as this tool's own pointed refusal (with the vocabulary and a
	# suggestion), not as argparse's exit-2 usage dump.
	g.add_argument("--failure-mode", metavar="MODE", help="one of: " + ", ".join(sorted(FAILURE_MODES)))
	g.add_argument("--location", help="where on the part")
	g.add_argument("--first-observed-h", type=float)
	g.add_argument("--text", help="what happened, in the reporter's words")
	g.add_argument("--severity", metavar="LEVEL",
	               help="one of: " + ", ".join(sorted(SEVERITY_RANK, key=lambda k: SEVERITY_RANK[k])))
	g.add_argument("--photo", help="path to a photo of the failure")
	args = p.parse_args(argv)

	if args.self_test:
		self_test()
	elif args.list:
		cmd_list(args)
	elif args.show:
		cmd_show(args)
	elif args.stats:
		cmd_stats(args)
	else:
		cmd_new(args)


if __name__ == "__main__":
	try:
		main(sys.argv[1:])
	except (ValueError, OSError, json.JSONDecodeError) as exc:
		fail([str(exc)])
	except KeyboardInterrupt:
		print("\naborted", file=sys.stderr)
		sys.exit(130)
