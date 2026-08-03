#!/usr/bin/env python3
"""field_triage.py — turn a FIELD REPORT into an ENGINEERING CONSEQUENCE.

`tools/field_report.py` captures what happened to the part in the real world.
This is the analytical half: for each report it computes what the shop should
CHANGE. Deterministic, closed-form, no ML — every number comes from the
material record or from the report itself, and anything unknown is said to be
unknown rather than filled in.

Three products per report:

1. **The remediation plan** — failure_mode → the analysis that would have
   caught it (creep → the material record's time × temperature creep table and
   a sustained-load check; fatigue_crack → `tools/ace_fatigue_runner.py`;
   fracture → a static strength margin; layer_delamination → the orientation /
   interlayer allowable; warping → the process/DFM and service-envelope rules;
   fit → the tolerance stack against a MEASURED printer profile), the solver to
   run, the allowable to re-derate, and the gate to add.

2. **The re-audit** — the campaign's generated `analysis/ANALYSIS.md` is parsed
   into labelled claims, and the ones this failure CONTRADICTS are listed.
   **A field failure that contradicts a green gate is the highest-value signal
   in the whole system**, so it is printed loudest and carries its own
   `contradicted_gates` field. Claims that live in an "out of scope" /
   "not performed" section are reported separately as `acknowledged_gaps` —
   the campaign said in advance it did not check this, which is honest, and
   the report is the evidence that the gap must now be closed.

3. **The permanent rule** — the one-line design rule / derated allowable /
   new gate that must land so every FUTURE part inherits the lesson.
   Doctrine: a field failure that does
   not become a gate is a lesson lost.

Usage:
	python3 tools/field_triage.py --all            # every real open report
	python3 tools/field_triage.py --id EXAMPLE-001
	python3 tools/field_triage.py --campaign spool/respool
	python3 tools/field_triage.py --all --include-examples
	python3 tools/field_triage.py --json-only      # machine block only
	python3 tools/field_triage.py --self-test      # pipeline gates, exit 1 on mismatch
"""

import argparse
import json
import re
import sys
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent
REPO_ROOT = TOOLS_DIR.parent
sys.path.insert(0, str(TOOLS_DIR))
import field_report as fr  # noqa: E402 — sibling tool, same directory

# ---------------------------------------------------------------------------
# failure_mode → the analysis that would have caught it, and what to change.
# Keyed EXACTLY by field_report.FAILURE_MODES (a missing key is a self-test
# failure, so the two files cannot drift apart silently).
# ---------------------------------------------------------------------------
FAILURE_ANALYSIS = {
	"creep_deformation": {
		"analysis": "sustained-load (creep) check at the SERVICE temperature and the SERVICE duration",
		"missed_because": (
			"a static strength margin says nothing about a load held for weeks. Creep allowables are "
			"derated by BOTH temperature and duration; a part can sit at 1% of the static allowable and "
			"still flow if the load never comes off."
		),
		"data": "tools/materials/<material>.json → creep.sig_allow_mpa[temperature][duration], else thermal.creep_sustained_fraction",
		"run": ["tools/production_check.py"],
		"run_note": "job JSON with load_character.sustained=true and service_temp_c set to the REPORTED temperature",
		"derate": "re-derate the sustained allowable to the reported temperature/duration bucket (round temperature and duration UP, allowable DOWN)",
		"gate": "sustained stress at the service temperature and duration ≤ the time-derated creep allowable",
		"rule": "every part that holds a load for longer than a print job gets a creep gate at its stated service temperature AND duration — not a static margin",
	},
	"fatigue_crack": {
		"analysis": "cyclic-life check: peak cyclic stress vs a cycle-count allowable",
		"missed_because": (
			"a static margin is blind to crack growth. Repeated cycles at a stress far below yield still "
			"accumulate damage, and printed layer lines are the crack starters."
		),
		"data": "tools/materials/<material>.json → mechanical.sn_curve (a 1e6-cycle KNOCKDOWN rule of thumb, NOT a measured S-N curve)",
		"run": ["tools/ace_fatigue_runner.py", "tools/ace_fea_runner.py"],
		"run_note": "FEA for the peak cyclic stress at the reported location, then the fatigue check at the reported cycle count",
		"derate": "apply mechanical.sn_curve.fraction_of_ultimate to the ultimate; if the reported cycle count is far from sn_curve.cycles the allowable is UNKNOWN, not interpolatable",
		"gate": "peak cyclic stress at the flexing feature ≤ the fatigue allowable at the DESIGN cycle count, with the design cycle count stated",
		"rule": "any feature that flexes in use declares a design cycle count and gates against it; 'wear-tolerant by construction' is a hypothesis, not a gate",
	},
	"fracture": {
		"analysis": "static strength margin at the fracture location, against the derated printed allowable",
		"missed_because": (
			"either the load case was not modelled, the load was larger than assumed, or the allowable was "
			"not derated for the printed direction at the fracture location."
		),
		"data": "tools/materials/<material>.json → mechanical.yield_mpa / ultimate_mpa, with process.anisotropy.z_vs_xy_strength_ratio when the load runs across layers",
		"run": ["tools/ace_fea_runner.py", "tools/production_check.py"],
		"run_note": "FEA (or the campaign's closed form) for the stress at the REPORTED location under the REPORTED load",
		"derate": "if the reported load exceeds the load case in the campaign's ANALYSIS.md, the load case is wrong before the allowable is",
		"gate": "stress at the fracture location under the reported load ≤ the derated allowable, with the reported load as the new design load",
		"rule": "the design load comes from what the field actually applies, not from what the design hoped it would",
	},
	"layer_delamination": {
		"analysis": "interlayer (across-layer) allowable check in the AS-PRINTED orientation",
		"missed_because": (
			"the allowable used was the in-plane one. A split ALONG a layer line means the load ran across "
			"the layers, where printed strength is a fraction of in-plane."
		),
		"data": "tools/materials/<material>.json → process.anisotropy.z_vs_xy_strength_ratio and out_of_plane_threshold_deg",
		"run": ["tools/production_check.py"],
		"run_note": "job JSON with orientation.build_dir and orientation.primary_load_dir set — the anisotropy rule is SKIPPED without them",
		"derate": "multiply every stress allowable by z_vs_xy_strength_ratio for the across-layer component",
		"gate": "across-layer stress at the split ≤ allowable × z_vs_xy_strength_ratio, in the orientation the part is actually printed in",
		"rule": "print orientation is a load-bearing design parameter: it is stated in the campaign, gated with the anisotropy derate, and printed in the build instructions",
	},
	"wear": {
		"analysis": "contact/wear life — NO in-tree solver exists; this is characterization work, not a computation",
		"missed_because": "no wear model is implemented in this repo, so nothing checked it. Saying so is the honest answer.",
		"data": "none — the material records carry no wear coefficient, and inventing one is forbidden",
		"run": [],
		"run_note": "there is no wear solver to run; the remediation is a design rule plus a cycle-life coupon the user can print and abuse",
		"derate": "none available — do not fabricate a wear allowable",
		"gate": "geometric: contact pressure (load / real contact area) below a stated bound, plus a sacrificial/replaceable-part rule for the wearing feature",
		"rule": "sliding contact between two printed parts is designed as consumable (replaceable wear part) or moved onto a bought bearing/bushing surface",
	},
	"warping": {
		"analysis": "thermal/process check: service temperature vs the material's softening envelope, plus the print-stress (DFM) rules",
		"missed_because": (
			"either the service temperature was above the material envelope (a CONDITION violation), or the "
			"part's residual print stress and section geometry were never checked against a hot service."
		),
		"data": "tools/materials/<material>.json → thermal.softening_c, thermal.tg_or_melt_c, thermal.service_temp_c",
		"run": ["tools/ace_thermal_runner.py", "tools/production_check.py"],
		"run_note": "production_check's temperature rule compares service temperature against the record's limit; thermal FEA only when a gradient drives it",
		"derate": "no stress derate — the envelope itself is the limit; if service demands more, the material changes",
		"gate": "declared service temperature ≤ material softening_c, gated in the campaign and printed on the product page",
		"rule": "every campaign states a MAXIMUM SERVICE TEMPERATURE derived from the material record, gates it, and ships it in README/PRINTABLES_LISTING — an unstated envelope is an envelope the user will exceed",
	},
	"chemical_uv": {
		"analysis": "environmental compatibility — NO in-tree solver; a material-selection decision with cited data",
		"missed_because": "environmental exposure is not part of any current gate suite; the campaign never declared the environment.",
		"data": "none in the material records (no chemical-resistance or UV block) — the answer comes from cited manufacturer data",
		"run": [],
		"run_note": "research pass (lmcad-research) against manufacturer chemical/UV resistance data; no solver applies",
		"derate": "none available — an unquantified environmental knockdown is a guess",
		"gate": "declared service ENVIRONMENT in the campaign, with a material chosen against cited resistance data for it",
		"rule": "outdoor/solvent/oil service is declared up front and drives material selection; PLA is refused for it by name",
	},
	"fit_loose": {
		"analysis": "tolerance stack for the named feature against a MEASURED printer profile",
		"missed_because": (
			"the fit was designed against nominal or a default clearance instead of the printer's measured "
			"compensation, or the stack-up of the mating chain was never closed."
		),
		"data": "profiles/<printer>.json from tools/ingest_calibration.py (measured coupons), not a default guess",
		"run": ["tools/tolerance_stack.py", "tools/ingest_calibration.py"],
		"run_note": "print the calibration coupons, ingest them, then re-close the stack for the named feature at worst case",
		"derate": "tighten the nominal clearance by the measured deviation; carry the printer tolerance as a band, not a point",
		"gate": "worst-case clearance at the named feature stays inside [min_required, max_allowed], with a printable fit coupon shipped in optional/",
		"rule": "every load-bearing fit is gated at NOMINAL and at WORST CASE against a measured profile, and ships a coupon so the user verifies it in minutes",
	},
	"fit_tight": {
		"analysis": "tolerance stack for the named feature against a MEASURED printer profile (interference direction)",
		"missed_because": (
			"the fit was designed against nominal or a default clearance; the printer's real deviation ate "
			"the gap, or the assembly stack closed the wrong way."
		),
		"data": "profiles/<printer>.json from tools/ingest_calibration.py (measured coupons), not a default guess",
		"run": ["tools/tolerance_stack.py", "tools/ingest_calibration.py"],
		"run_note": "print the calibration coupons, ingest them, then re-close the stack for the named feature at worst case",
		"derate": "open the nominal clearance by the measured deviation; a press fit gets an insertion-stress check too",
		"gate": "worst-case interference at the named feature stays inside the intended band, with a printable fit coupon shipped in optional/",
		"rule": "every load-bearing fit is gated at NOMINAL and at WORST CASE against a measured profile, and ships a coupon so the user verifies it in minutes",
	},
	"other": {
		"analysis": "UNCLASSIFIED — triage refuses to guess an analysis for 'other'",
		"missed_because": "the failure has not been classified into a mode, so no analysis can be selected deterministically.",
		"data": "none until classified",
		"run": [],
		"run_note": "a human re-reads the report and either picks a mode from the vocabulary or extends the vocabulary in tools/field_report.py",
		"derate": "none",
		"gate": "none derivable — classify first",
		"rule": "if 'other' keeps recurring, the vocabulary is wrong and the vocabulary gets extended (with its triage mapping) in the same change",
	},
}

# Words in a claim that mark it as an ADEQUACY assertion — a claim that
# something is fine. Only adequacy claims can be CONTRADICTED by a failure.
ADEQUACY_PATTERNS = [
	r"\d[\d.,]*\s*×",              # "4146×", "70×" — margin multipliers
	r"×\s*\d[\d.,]*\s*(\||$)",     # "×0.5" as the last cell of a table row
	r"\bmargin\b", r"\ballowable\b", r"\bcapacity\b", r"\bcannot\b",
	r"\bnever\b", r"nothing to\b", r"out of reach", r"\bwithin\b",
	r"\bzero\b", r"\bfree\b", r"\bcaptive\b", r"\bretains?\b", r"\bholds?\b",
	r"[≤≥]", r"\bnot the failure point\b", r"\bgated?\b", r"\bbound\b",
	r"\btolerant\b", r"\babsorbed\b", r"\bavoids\b", r"\bsafe\b",
]

# Headings / phrases that mark a claim as an ACKNOWLEDGED GAP rather than a
# green claim. The campaign said in advance it did not check this.
GAP_HEADING_RE = re.compile(
	r"out of scope|not performed|honesty|gaps?\b|unknowns?\b|limitations?|open\b", re.I)
GAP_TEXT_RE = re.compile(
	r"not performed|out of scope|not life-tested|no honest closed form|"
	r"\bunknown\b|not required|\bNOT\b performed|stated, not hidden", re.I)

# failure_mode → the words a relevant ANALYSIS.md claim would contain.
MODE_KEYWORDS = {
	"creep_deformation": ["creep", "relax", "sustained", "hot", "hdt", "tg", "long-term", "long term",
	                      "cold-flow", "preload", "flow", "vicat"],
	"fracture": ["strength", "stress", "shear", "tension", "tensile", "bearing", "yield", "ultimate",
	             "mpa", "load", "pull-apart", "torque", "impact", "drop", "brittle", "fasten"],
	"fatigue_crack": ["fatigue", "cycle", "cycles", "detent", "repeated", "life", "snap", "flex", "spring"],
	"layer_delamination": ["layer", "adhesion", "interlayer", "anisotrop", "orientation", "across-layer",
	                       "build direction", "posed", "print-posed"],
	"wear": ["wear", "abrasion", "rub", "contact", "bearing", "detent", "slid", "cycles", "running"],
	"warping": ["warp", "thermal", "temperature", "hdt", "tg", "vicat", "expansion", "cte", "heat",
	            "dry", "dryer", "melt", "setpoint"],
	"chemical_uv": ["uv", "chemical", "solvent", "moisture", "humid", "hydrolys", "oil", "outdoor",
	                "sunlight", "ozone", "desiccant"],
	"fit_loose": ["fit", "clearance", "gap", "tolerance", "bore", "play", "slop", "retention", "seat",
	              "free-run", "captive", "margin"],
	"fit_tight": ["fit", "clearance", "gap", "tolerance", "bore", "interference", "press", "penetration",
	              "crush", "seat", "bite"],
	"other": [],
}

STOPWORDS = {
	"the", "and", "all", "with", "from", "that", "this", "where", "when", "meets", "three", "worst",
	"side", "onto", "over", "into", "after", "about", "roughly", "half", "part", "parts", "side",
	"body", "along", "its", "for", "one", "two", "each", "both", "some", "very", "then", "than",
}


# ---------------------------------------------------------------------------
# ANALYSIS.md → labelled claims
# ---------------------------------------------------------------------------
def _clean(text):
	"""Strip markdown emphasis/links so labels and matching see plain words."""
	text = re.sub(r"\[([^\]]*)\]\([^)]*\)", r"\1", text)
	text = text.replace("**", "").replace("`", "")
	return re.sub(r"\s+", " ", text).strip()


def _label_from(cell, section, used):
	base = _clean(cell)[:70].rstrip(" .:—-")
	label = f"{section} / {base}" if section else base
	n = 2
	while label in used:
		label = f"{section} / {base} #{n}" if section else f"{base} #{n}"
		n += 1
	used.add(label)
	return label


def parse_analysis(path):
	"""Parse a generated campaign ANALYSIS.md into labelled claims.

	Three claim shapes are recognised, which is all these generated documents
	contain: TABLE ROWS (label = section / first cell), BULLETS (label =
	section / bolded span or opening words, continuation lines joined), and
	bolded PROSE assertions. Header and separator rows are skipped. A claim
	inheriting an "out of scope"-class heading, or whose own text says it was
	not performed, is marked `gap=True` — an honest declared limitation, not a
	green claim."""
	claims = []
	if not Path(path).is_file():
		return claims
	lines = Path(path).read_text(encoding="utf-8").splitlines()
	section, used = "", set()
	in_table, pending = False, None

	def flush():
		nonlocal pending
		if pending:
			pending["text"] = _clean(pending["text"])
			pending["gap"] = bool(pending["gap"] or GAP_TEXT_RE.search(pending["text"]))
			pending["adequacy"] = any(re.search(p, pending["text"], re.I) for p in ADEQUACY_PATTERNS)
			claims.append(pending)
			pending = None

	for n, raw in enumerate(lines, start=1):
		line = raw.rstrip()
		stripped = line.strip()
		if stripped.startswith("#"):
			flush()
			section = _clean(stripped.lstrip("#"))
			in_table = False
			continue
		if not stripped:
			flush()
			in_table = False
			continue
		gap_section = bool(GAP_HEADING_RE.search(section))
		if stripped.startswith("|"):
			flush()
			cells = [c.strip() for c in stripped.strip("|").split("|")]
			if not in_table:                       # first row of a table = its header
				in_table = True
				continue
			if all(set(c) <= set("-: ") for c in cells if c):
				continue                            # separator row
			claims.append({
				"label": _label_from(cells[0], section, used), "section": section, "kind": "table_row",
				"line": n, "text": _clean(stripped.strip("|").replace("|", " · ")), "gap": gap_section,
				"adequacy": any(re.search(p, _clean(stripped), re.I) for p in ADEQUACY_PATTERNS),
			})
			continue
		in_table = False
		if re.match(r"^[-*]\s+", stripped):
			flush()
			body = re.sub(r"^[-*]\s+", "", stripped)
			bold = re.search(r"\*\*(.+?)\*\*", body)
			key = bold.group(1) if bold else body
			pending = {"label": _label_from(key, section, used), "section": section, "kind": "bullet",
			           "line": n, "text": body, "gap": gap_section, "adequacy": False}
			continue
		if pending is not None and line.startswith((" ", "\t")):
			pending["text"] += " " + stripped        # continuation of the bullet
			continue
		flush()
		bold = re.search(r"\*\*(.+?)\*\*", stripped)
		if bold:
			pending = {"label": _label_from(bold.group(1), section, used), "section": section,
			           "kind": "prose", "line": n, "text": stripped, "gap": gap_section, "adequacy": False}
	flush()
	return claims


def _tokens(*texts):
	out = set()
	for t in texts:
		for w in re.findall(r"[a-z][a-z-]{3,}", str(t or "").lower()):
			if w in STOPWORDS:
				continue
			out.add(w)
			if len(w) > 4 and w.endswith("s"):
				out.add(w[:-1])
	return out


def _mode_hits(text, mode):
	low = text.lower()
	hits = []
	for kw in MODE_KEYWORDS.get(mode, []):
		if kw.isalpha():
			if re.search(r"(?<![a-z])" + re.escape(kw) + r"(?![a-z])", low):
				hits.append(kw)
		elif kw in low:
			hits.append(kw)
	return hits


def reaudit(record, claims):
	"""Score every claim against the report. Returns (contradicted, gaps, weak).

	A claim is CONTRADICTED when it is an adequacy assertion (it says something
	is fine) in a non-gap section AND it talks about this failure mode STRONGLY
	enough. "Strongly enough" is a stated rule, not a magic threshold: at least
	two distinct failure-mode words, OR one such word plus a word naming the
	same part/location as the report. One generic word (`hot`, `cycles`) with
	nothing tying it to this part is a coincidence, not a contradiction — those
	land in `weak` so they are demoted but never hidden.

	A matching claim inside a declared out-of-scope section is an ACKNOWLEDGED
	GAP instead: the campaign said in advance it had not checked this, and the
	field just proved it matters."""
	obs = record.get("observation", {})
	mode = obs.get("failure_mode", "")
	loc = _tokens(obs.get("location"), record.get("part", {}).get("part"))
	contradicted, gaps, weak = [], [], []
	for c in claims:
		hits = _mode_hits(c["text"], mode)
		if not hits:
			continue
		lhits = sorted(loc & _tokens(c["text"]))
		strong = len(hits) >= 2 or bool(lhits)
		item = {
			"label": c["label"], "section": c["section"], "line": c["line"], "kind": c["kind"],
			"text": c["text"], "matched_failure_mode_words": hits, "matched_location_words": lhits,
			"score": 2 * len(hits) + 3 * len(lhits) + (2 if c["adequacy"] else 0),
			"where": "declared gap" if c["gap"] else "claim",
		}
		if not strong:
			item["why"] = (
				f"one generic word ({hits[0]}) and nothing naming this part — demoted to a weak match, "
				"shown so it is not hidden, not counted as a contradiction"
			)
			weak.append(item)
		elif c["gap"]:
			item["why"] = (
				f"the campaign declared this out of scope; the report is field evidence that "
				f"'{mode}' at this location is now in scope"
			)
			gaps.append(item)
		elif c["adequacy"]:
			item["why"] = (
				f"this claim asserts adequacy ({', '.join(hits)}) for exactly the failure the field "
				f"reported — it is green in ANALYSIS.md and false in the field"
			)
			contradicted.append(item)
		else:
			item["why"] = "relevant to the failure mode but asserts no adequacy — context, not a contradiction"
			weak.append(item)
	for lst in (contradicted, gaps, weak):
		lst.sort(key=lambda i: (-i["score"], i["line"]))
	return contradicted, gaps, weak


# ---------------------------------------------------------------------------
# Material-driven numbers (real data from the record, or an explicit UNKNOWN)
# ---------------------------------------------------------------------------
_TIME_H = {"h": 1.0, "d": 24.0, "w": 168.0, "mo": 730.0, "y": 8760.0}


def _duration_hours(key):
	m = re.match(r"^(\d+(?:\.\d+)?)(h|d|w|mo|y)$", key.strip())
	return float(m.group(1)) * _TIME_H[m.group(2)] if m else None


def creep_allowable(mat, temp_c, duration_h):
	"""Pick the conservative cell of the record's creep table: the smallest
	tabulated temperature ≥ service and the smallest tabulated duration ≥
	service (i.e. round BOTH up, allowable down). Falls back to the scalar
	`thermal.creep_sustained_fraction` when no table exists, and returns an
	explicit UNKNOWN when neither is present — never a guess."""
	table = (mat.get("creep") or {}).get("sig_allow_mpa")
	if not table:
		frac = (mat.get("thermal") or {}).get("creep_sustained_fraction")
		yld = (mat.get("mechanical") or {}).get("yield_mpa")
		if frac is None or yld is None:
			return {"known": False, "note": "no creep table and no creep_sustained_fraction in the material record — allowable UNKNOWN"}
		return {"known": True, "sig_allow_mpa": round(float(frac) * float(yld), 3),
		        "basis": f"thermal.creep_sustained_fraction {frac} × yield {yld} MPa (scalar, time-blind)",
		        "temperature_bucket": None, "duration_bucket": None, "confidence": "scalar rule of thumb — no time dependence"}
	temps = sorted(((float(re.sub(r"[^0-9.\-]", "", k)), k) for k in table), key=lambda x: x[0])
	pick_t = next((k for v, k in temps if temp_c is not None and v >= temp_c), temps[-1][1])
	col = table[pick_t]
	durs = sorted(((_duration_hours(k) or 0.0, k) for k in col), key=lambda x: x[0])
	pick_d = next((k for v, k in durs if duration_h is not None and v >= duration_h), durs[-1][1])
	conf = ((mat.get("creep") or {}).get("confidence") or {}).get(pick_t, {}).get(pick_d)
	return {
		"known": True, "sig_allow_mpa": float(col[pick_d]),
		"temperature_bucket": pick_t, "duration_bucket": pick_d,
		"basis": f"creep.sig_allow_mpa[{pick_t}][{pick_d}] — service {temp_c} °C / {duration_h} h rounded UP to the tabulated cell",
		"confidence": conf or "(no confidence string in the record for this cell)",
		"extrapolated": (temp_c is not None and temp_c > temps[-1][0]) or (duration_h is not None and duration_h > (durs[-1][0] or 0.0)),
	}


def fatigue_allowable(mat, cycles):
	sn = (mat.get("mechanical") or {}).get("sn_curve") or {}
	ult = (mat.get("mechanical") or {}).get("ultimate_mpa")
	if sn.get("kind") != "knockdown" or ult is None:
		return {"known": False, "note": "no knockdown S-N model in the material record — fatigue allowable UNKNOWN"}
	at = float(sn["cycles"])
	out = {
		"known": True, "sig_allow_mpa": round(float(ult) * float(sn["fraction_of_ultimate"]), 3),
		"basis": f"ultimate {ult} MPa × sn_curve.fraction_of_ultimate {sn['fraction_of_ultimate']} at {at:.0f} cycles",
		"at_cycles": at, "reported_cycles": cycles,
		"confidence": sn.get("note", ""),
	}
	if cycles and abs(cycles - at) / at > 0.5:
		out["warning"] = (
			f"the record carries ONE knockdown point at {at:.0f} cycles; the report is at {cycles:g} cycles. "
			"The record has no S-N slope, so the allowable at the reported life is UNKNOWN — do not interpolate. "
			"Either measure a coupon or design to the 1e6 point and say so."
		)
	return out


# ---------------------------------------------------------------------------
# Triage of one report
# ---------------------------------------------------------------------------
def analysis_md_path(record):
	part = record.get("part", {})
	return REPO_ROOT / f"{part.get('family','?')}_system" / str(part.get("entry", "?")) / "analysis" / "ANALYSIS.md"


def campaign_source(record):
	entry = str(record.get("part", {}).get("entry", ""))
	p = REPO_ROOT / "crates" / "kernel-model" / "examples" / f"{entry}.rs"
	return p if p.is_file() else None


def triage(record):
	obs = record.get("observation", {})
	svc = record.get("service", {})
	mode = obs.get("failure_mode", "")
	plan = FAILURE_ANALYSIS.get(mode)
	out = {
		"report_id": record.get("id"), "example": bool(record.get("example")),
		"campaign": f"{record.get('part',{}).get('family','?')}/{record.get('part',{}).get('entry','?')}",
		"part": record.get("part", {}).get("part"), "material": record.get("process", {}).get("material"),
		"failure_mode": mode, "severity": record.get("severity"),
		"classification": record.get("classification"),
	}
	if plan is None:
		out["error"] = f"no remediation mapping for failure_mode {mode!r} — FAILURE_ANALYSIS and field_report.FAILURE_MODES have drifted"
		return out

	condition = record.get("classification") == fr.CLASS_CONDITION
	mat, how = fr.material_record(out["material"] or "")
	out["material_record"] = f"{mat['meta']['name']} v{mat['meta']['version']} ({how})" if mat else "UNRESOLVED"

	remediation = {
		"analysis_that_would_have_caught_it": plan["analysis"],
		"missed_because": plan["missed_because"],
		"data_source": plan["data"].replace("<material>", str(out["material"] or "?").lower()),
		"run": [{"tool": t, "present": (REPO_ROOT / t).is_file()} for t in plan["run"]],
		"run_note": plan["run_note"],
		"allowable_to_rederate": plan["derate"],
		"gate_to_add": plan["gate"],
		"permanent_rule": plan["rule"],
	}
	if condition:
		cv = record.get("condition_violation", {})
		remediation["classification_override"] = (
			"CONDITION VIOLATION — the part ran outside its material envelope "
			f"({cv.get('value')} °C vs {cv.get('material')} {cv.get('limit_field')} {cv.get('limit_c')} °C). "
			"The design is NOT derated for this. The remediation is a stated service limit and/or a material "
			"change, and the re-audit below is advisory only: a green gate is not contradicted by a part run "
			"outside the envelope it was gated for."
		)
		remediation["gate_to_add"] = (
			"declared MAXIMUM SERVICE TEMPERATURE ≤ material thermal.softening_c, gated in the campaign and "
			"printed in README/PRINTABLES_LISTING so the user meets the envelope before the part does"
		)
		remediation["allowable_to_rederate"] = "none — the envelope is the limit, not the allowable"

	# Real numbers where the material record can supply them.
	numbers = {}
	if mat and mode == "creep_deformation":
		numbers["creep"] = creep_allowable(mat, svc.get("temp_c"), svc.get("duration_h"))
	if mat and mode == "fatigue_crack":
		numbers["fatigue"] = fatigue_allowable(mat, svc.get("cycles"))
	if mat and mode in ("warping", "layer_delamination", "chemical_uv"):
		th, pr = mat.get("thermal") or {}, mat.get("process") or {}
		numbers["envelope"] = {
			"softening_c": th.get("softening_c"), "service_temp_c": th.get("service_temp_c"),
			"tg_or_melt_c": th.get("tg_or_melt_c"),
			"z_vs_xy_strength_ratio": (pr.get("anisotropy") or {}).get("z_vs_xy_strength_ratio"),
		}
	if numbers:
		remediation["numbers"] = numbers
	out["remediation"] = remediation

	# --- re-audit against the shipped campaign's generated ANALYSIS.md ------
	md = analysis_md_path(record)
	claims = parse_analysis(md)
	contradicted, gaps, weak = reaudit(record, claims) if claims else ([], [], [])
	if condition:
		# Advisory only: an out-of-envelope run cannot falsify an in-envelope gate.
		for c in contradicted:
			c["advisory_only"] = "condition violation — this claim is NOT falsified by an out-of-envelope run"
	out["analysis_md"] = str(md.relative_to(REPO_ROOT)) if md.is_file() else f"MISSING: {md}"
	out["claims_parsed"] = len(claims)
	out["contradicted_gates"] = [] if condition else contradicted
	out["advisory_matches"] = contradicted if condition else []
	out["acknowledged_gaps"] = gaps
	out["weak_matches"] = weak
	out["campaign_source"] = str(campaign_source(record).relative_to(REPO_ROOT)) if campaign_source(record) else None

	sev = fr.SEVERITY_RANK.get(record.get("severity"), 0)
	if out["contradicted_gates"] or sev >= 3:
		out["priority"] = "P0"
		out["priority_why"] = ("a green gate is contradicted by the field" if out["contradicted_gates"]
		                       else "safety severity")
	elif condition:
		out["priority"] = "P2"
		out["priority_why"] = "condition violation — envelope/documentation work, not a design derate"
	elif gaps:
		out["priority"] = "P1"
		out["priority_why"] = ("design failure landing on a DECLARED gap — the campaign was honest that it had "
		                       "not checked this; the field just made it due")
	elif sev >= 1:
		out["priority"] = "P1"
		out["priority_why"] = "design failure with no contradicted gate — a missing analysis, not a wrong one"
	else:
		out["priority"] = "P2"
		out["priority_why"] = "cosmetic severity"
	return out


# ---------------------------------------------------------------------------
# Human-readable rendering
# ---------------------------------------------------------------------------
def render(t):
	L = []
	tag = "  [EXAMPLE — synthetic, not a real observation]" if t.get("example") else ""
	L.append("=" * 78)
	L.append(f"{t['report_id']}  ·  {t['campaign']}  ·  {t['failure_mode']}  ·  {t['priority']}{tag}")
	L.append("=" * 78)
	if t.get("error"):
		L.append(f"ERROR: {t['error']}")
		return "\n".join(L)
	L.append(f"part          {t.get('part') or '(unstated)'}")
	L.append(f"material      {t.get('material')}  ({t.get('material_record')})")
	L.append(f"severity      {t.get('severity')}   classification {t.get('classification')}")
	L.append(f"priority      {t['priority']} — {t['priority_why']}")
	r = t["remediation"]
	if r.get("classification_override"):
		L.append("")
		L.append("*** " + r["classification_override"])
	L.append("")
	L.append("REMEDIATION PLAN")
	L.append(f"  analysis that would have caught it : {r['analysis_that_would_have_caught_it']}")
	L.append(f"  why it was missed                  : {r['missed_because']}")
	L.append(f"  data source                        : {r['data_source']}")
	if r["run"]:
		for tool in r["run"]:
			L.append(f"  solver to run                      : {tool['tool']}"
			         + ("" if tool["present"] else "   [NOT PRESENT in this tree — say so, do not pretend it ran]"))
		L.append(f"                                       {r['run_note']}")
	else:
		L.append(f"  solver to run                      : NONE EXISTS — {r['run_note']}")
	L.append(f"  allowable to re-derate             : {r['allowable_to_rederate']}")
	L.append(f"  GATE TO ADD                        : {r['gate_to_add']}")
	L.append(f"  permanent design rule              : {r['permanent_rule']}")
	for key, block in (r.get("numbers") or {}).items():
		L.append(f"  numbers ({key}):")
		for k, v in block.items():
			L.append(f"      {k:<20} {v}")
	L.append("")
	if t["contradicted_gates"]:
		L.append("!" * 78)
		L.append(f"!!  {len(t['contradicted_gates'])} CONTRADICTED CLAIM(S) in {t['analysis_md']}")
		L.append("!!  A field failure that contradicts a green gate is the highest-value")
		L.append("!!  signal in this system. Re-open the campaign; do not close this report")
		L.append("!!  until each claim below is re-derived, retracted, or dismissed WITH a")
		L.append("!!  written reason. Each entry is a CANDIDATE: the matcher reads vocabulary")
		L.append("!!  and adequacy wording, NOT physics. A human adjudicates every one.")
		L.append("!" * 78)
		for c in t["contradicted_gates"]:
			L.append(f"  [{c['score']:>2}] {c['label']}   (line {c['line']}, {c['kind']})")
			L.append(f"       claim : {c['text'][:200]}")
			L.append(f"       why   : {c['why']}")
			if c.get("matched_location_words"):
				L.append(f"       same location words: {', '.join(c['matched_location_words'])}")
	elif t.get("advisory_matches"):
		L.append(f"claims touching this mode in {t['analysis_md']} (ADVISORY — condition violation, not contradictions):")
		for c in t["advisory_matches"][:5]:
			L.append(f"  [{c['score']:>2}] {c['label']}  (line {c['line']})")
	elif str(t["analysis_md"]).startswith("MISSING"):
		L.append(f"RE-AUDIT COULD NOT RUN — {t['analysis_md']}. This is NOT 'no contradictions found':")
		L.append("the campaign has no generated ANALYSIS.md to audit, so nothing was checked. Say so.")
	else:
		L.append(f"no contradicted claims found in {t['analysis_md']} ({t['claims_parsed']} claims parsed)")
	if t["acknowledged_gaps"]:
		L.append("")
		L.append("ACKNOWLEDGED GAPS this report lands on (the campaign said it had NOT checked this):")
		for g in t["acknowledged_gaps"]:
			L.append(f"  [{g['score']:>2}] {g['label']}   (line {g['line']})")
			L.append(f"       {g['text'][:200]}")
		L.append("  → honest in advance, and now due: the field just proved the gap matters.")
	if t.get("weak_matches"):
		L.append("")
		L.append(f"weak matches (shown, not counted — one generic word, nothing naming this part): "
		         + ", ".join(f"{w['label']} [{w['score']}]" for w in t["weak_matches"][:6]))
	L.append("")
	L.append("CLOSE THE LOOP: land the gate above in "
	         + (t.get("campaign_source") or "the campaign example")
	         + ", re-derate the allowable, add the design rule, then tell the user what changed.")
	L.append("Doctrine: a field failure that does not become a gate is a lesson lost.")
	return "\n".join(L)


# ---------------------------------------------------------------------------
def select(args):
	records, _ = fr.load_reports(args.corpus)
	if args.id:
		hit = [r for r in records if str(r.get("id")) == args.id]
		if not hit:
			print(json.dumps({"ok": False, "error": f"no report with id {args.id!r}"}, indent=2))
			sys.exit(1)
		return hit
	pool = [r for r in records if args.include_examples or not r.get("example")]
	if args.campaign:
		want = args.campaign.strip("/")
		pool = [r for r in pool if f"{r.get('part',{}).get('family')}/{r.get('part',{}).get('entry')}" == want]
	return pool


def self_test():
	"""Run the whole pipeline on the labelled EXAMPLE reports and assert the
	expected remediations. Exit 1 on any mismatch."""
	checks = []

	def check(label, cond, detail=""):
		checks.append((label, bool(cond), detail))

	# the two files must not drift apart
	check("every failure_mode has a remediation mapping",
	      set(FAILURE_ANALYSIS) == set(fr.FAILURE_MODES),
	      f"only in triage: {sorted(set(FAILURE_ANALYSIS) - set(fr.FAILURE_MODES))}; "
	      f"only in intake: {sorted(set(fr.FAILURE_MODES) - set(FAILURE_ANALYSIS))}")

	records, _ = fr.load_reports()
	by_id = {str(r.get("id")): r for r in records}
	check("shipped corpus has the three EXAMPLE reports",
	      {"EXAMPLE-001", "EXAMPLE-002", "EXAMPLE-003"} <= set(by_id), f"ids: {sorted(by_id)}")
	if not {"EXAMPLE-001", "EXAMPLE-002", "EXAMPLE-003"} <= set(by_id):
		_report(checks)

	# --- the real respool ANALYSIS.md parses -------------------------------
	respool_md = REPO_ROOT / "spool_system" / "respool" / "analysis" / "ANALYSIS.md"
	claims = parse_analysis(respool_md)
	check("respool ANALYSIS.md parses into claims", len(claims) >= 15, f"{len(claims)} claims")
	check("respool claims carry section labels", all("/" in c["label"] for c in claims),
	      str([c["label"] for c in claims if "/" not in c["label"]][:3]))
	check("the out-of-scope section is recognised as gaps, not green claims",
	      sum(1 for c in claims if c["gap"]) >= 3, f"{sum(1 for c in claims if c['gap'])} gap claims")

	# --- EXAMPLE-001: creep on respool, contradicts a green claim ----------
	t1 = triage(by_id["EXAMPLE-001"])
	check("E-001 maps to the sustained-load creep analysis",
	      "creep" in t1["remediation"]["analysis_that_would_have_caught_it"],
	      t1["remediation"]["analysis_that_would_have_caught_it"])
	check("E-001 names production_check.py as the solver to run",
	      any(x["tool"] == "tools/production_check.py" and x["present"] for x in t1["remediation"]["run"]),
	      str(t1["remediation"]["run"]))
	creep = t1["remediation"]["numbers"]["creep"]
	check("E-001 re-derates from the PLA creep TABLE, conservative cell (55C/1y)",
	      creep["known"] and creep["temperature_bucket"] == "55C" and creep["duration_bucket"] == "1y",
	      str(creep))
	check("E-001 derated sustained allowable is 0.5 MPa (the record's stated BOUND)",
	      creep["sig_allow_mpa"] == 0.5, str(creep.get("sig_allow_mpa")))
	check("E-001 CONTRADICTS at least one green claim in the real respool ANALYSIS.md",
	      len(t1["contradicted_gates"]) >= 1, f"{len(t1['contradicted_gates'])} contradicted")
	labels1 = " | ".join(c["label"] for c in t1["contradicted_gates"])
	check("E-001 names the 'joint cannot be the failure point in a dryer' claim",
	      "cannot be the failure point" in labels1, labels1)
	check("E-001 also lands on the declared out-of-scope long-term creep gap",
	      any("creep" in g["label"].lower() for g in t1["acknowledged_gaps"]),
	      str([g["label"] for g in t1["acknowledged_gaps"]]))
	check("E-001 ties LC4/LC5 in by the part's own words (tongue, lugs)",
	      any(c["matched_location_words"] for c in t1["contradicted_gates"]),
	      str([(c["label"], c["matched_location_words"]) for c in t1["contradicted_gates"]]))
	check("E-001 demotes the generic one-word 'hot' rows (LC1/LC2) to weak, not contradicted",
	      any(w["label"].startswith("Load cases / LC1") for w in t1["weak_matches"])
	      and not any(c["label"].startswith("Load cases / LC1") for c in t1["contradicted_gates"]),
	      f"weak={[w['label'] for w in t1['weak_matches']]}")
	check("E-001 is P0 (a green gate was contradicted)", t1["priority"] == "P0", t1["priority"])

	# --- EXAMPLE-002: fatigue on respool, lands on a declared gap ----------
	t2 = triage(by_id["EXAMPLE-002"])
	check("E-002 maps to the cyclic-life analysis",
	      "cyclic" in t2["remediation"]["analysis_that_would_have_caught_it"],
	      t2["remediation"]["analysis_that_would_have_caught_it"])
	check("E-002 names tools/ace_fatigue_runner.py (presence reported honestly)",
	      any(x["tool"] == "tools/ace_fatigue_runner.py" for x in t2["remediation"]["run"]),
	      str(t2["remediation"]["run"]))
	fat = t2["remediation"]["numbers"]["fatigue"]
	check("E-002 fatigue allowable comes from the record's knockdown (60 × 0.3 = 18 MPa)",
	      fat["known"] and abs(fat["sig_allow_mpa"] - 18.0) < 1e-9, str(fat))
	check("E-002 refuses to interpolate the S-N curve at 2400 cycles", "warning" in fat, str(fat))
	check("E-002 lands on respool's declared 'fatigue of the detent' gap",
	      any("fatigue" in g["label"].lower() for g in t2["acknowledged_gaps"]),
	      str([g["label"] for g in t2["acknowledged_gaps"]]))
	check("E-002 contradicts NO green claim (the 'cycles' in the thermal bullet is demoted)",
	      t2["contradicted_gates"] == [] and any("cycles" in w["matched_failure_mode_words"] for w in t2["weak_matches"]),
	      f"contradicted={[c['label'] for c in t2['contradicted_gates']]} weak={[w['label'] for w in t2['weak_matches']]}")
	check("E-002 is P1 — a declared gap made due, not a contradicted gate",
	      t2["priority"] == "P1" and "DECLARED gap" in t2["priority_why"], f"{t2['priority']}: {t2['priority_why']}")

	# --- EXAMPLE-003: condition violation, must NOT blame the design -------
	t3 = triage(by_id["EXAMPLE-003"])
	check("E-003 is classified a condition violation", t3["classification"] == fr.CLASS_CONDITION, str(t3["classification"]))
	check("E-003 contradicts NOTHING (an out-of-envelope run cannot falsify an in-envelope gate)",
	      t3["contradicted_gates"] == [], str(t3["contradicted_gates"]))
	check("E-003 states the envelope override", "CONDITION VIOLATION" in t3["remediation"].get("classification_override", ""),
	      str(t3["remediation"].get("classification_override"))[:80])
	check("E-003 re-derates NO allowable", "none" in t3["remediation"]["allowable_to_rederate"].lower(),
	      t3["remediation"]["allowable_to_rederate"])
	check("E-003 is P2 (envelope/documentation work)", t3["priority"] == "P2", t3["priority"])
	check("E-003 gate is a declared max service temperature",
	      "SERVICE TEMPERATURE" in t3["remediation"]["gate_to_add"], t3["remediation"]["gate_to_add"])

	# every triage renders without blowing up
	for t in (t1, t2, t3):
		check(f"{t['report_id']} renders", len(render(t)) > 400, "")
	_report(checks)


def _report(checks):
	failed = [(l, d) for l, c, d in checks if not c]
	width = max(len(l) for l, _, _ in checks)
	for label, cond, detail in checks:
		print(f"  [{'OK' if cond else 'FAIL'}] {label:<{width}}  {detail if not cond else ''}")
	print(f"\n{len(checks) - len(failed)}/{len(checks)} checks passed")
	if failed:
		print(json.dumps({"ok": False, "error": f"{len(failed)} self-test check(s) failed",
		                  "errors": [f"{l}: {d}" for l, d in failed]}, ensure_ascii=False, indent=2))
		sys.exit(1)
	print(json.dumps({"ok": True, "self_test": "PASS", "checks": len(checks)}, indent=2))


def main(argv):
	p = argparse.ArgumentParser(
		prog="field_triage.py",
		description="Field report → engineering consequence: remediation plan, campaign re-audit, permanent rule.")
	m = p.add_mutually_exclusive_group(required=True)
	m.add_argument("--all", action="store_true", help="triage every open report")
	m.add_argument("--id", metavar="ID", help="triage one report")
	m.add_argument("--campaign", metavar="FAMILY/ENTRY", help="triage every report against one campaign")
	m.add_argument("--self-test", dest="self_test", action="store_true", help="run the pipeline on the EXAMPLE reports")
	p.add_argument("--corpus", help=f"corpus path (default {fr.REPORTS_PATH})")
	p.add_argument("--include-examples", action="store_true", help="include the synthetic example records")
	p.add_argument("--json-only", action="store_true", help="print only the machine JSON block")
	args = p.parse_args(argv)

	if args.self_test:
		self_test()
		return
	records = select(args)
	results = [triage(r) for r in records]
	if not args.json_only:
		if not results:
			print("No open field reports. The real corpus is empty until the user reports something.")
			print("(Add --include-examples to triage the synthetic example records.)")
		for t in results:
			print(render(t))
			print()
	print(json.dumps({"ok": True, "triaged": len(results),
	                  "p0": sum(1 for t in results if t.get("priority") == "P0"), "reports": results},
	                 ensure_ascii=False, indent=2, sort_keys=True))


if __name__ == "__main__":
	main(sys.argv[1:])
