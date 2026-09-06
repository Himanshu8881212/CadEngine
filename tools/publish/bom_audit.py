#!/usr/bin/env python3
"""bom_audit.py — unified-BOM audit across a family of STEP assemblies.

Tally hardware instances from each assembly's STEP tree and check them against a
declared unified BOM: every hardware name must be in the BOM, and a name the BOM
restricts to certain assemblies must not appear anywhere else.

Usage
-----
    bom_audit.py job.json          run the audit
    bom_audit.py --example         print a ready-to-run example job (the old
                                   hardcoded cyclo26/harmonic26/planetary26
                                   gearbox project, verbatim) to stdout
    bom_audit.py --help

Job JSON
--------
    {
      "assemblies": [{"name": "cyclo26", "step": "cyclo26/ASSEMBLY.step"}, ...],
        REQUIRED. `step` is resolved against `base_dir` if given, else the JOB
        FILE's directory, else the CWD — whichever contains it (a search that
        finds nothing is a refusal naming the roots tried, never a guess).
      "name_pattern": "hw_[a-z0-9_]+",      optional; the quoted instance names
        the audit counts. Default matches the `hw_*` convention.
      "calibrate_with": "hw_nema17",        optional; the name that is known to
        occur EXACTLY ONCE per assembly, used to subtract the per-name STEP
        metadata overhead. Omit it and `overhead` is used instead.
      "overhead": 0,                        optional; explicit metadata-row
        overhead when there is no single-instance calibrator.
      "bom": {"hw_bearing_6804": {"label": "6804 bearing"},
              "hw_dowel_2x20":   {"label": "...", "only": ["cyclo26"]},
              ...}                          REQUIRED. `only` restricts a part to
        the named assemblies. `expect` (int) optionally pins the family total.
      "receipt": "<path>"                   optional; tools/_receipt.py rules.
    }

Receipt (last stdout line, and persisted when `receipt`/`out_dir` is given):
    {ok, assemblies:[{name, step, items:[{name, count, label, verdict}]}],
     family_totals:{name: count}, findings:[...]}
`ok` is false and the process EXITS 1 when any finding is recorded: a name not
in the BOM, a name used outside its `only` list, a declared BOM entry that never
appears, or a family total that misses its `expect`.

Why this is a job file (digest F5): the tool used to hardcode one project's
three STEP paths and its hardware table in source, so it could not audit any
other campaign without editing the tool. Nothing about "count NAUO instance
names and check them against a declared BOM" is project-specific — only the
paths and the table are, and those are inputs.
"""
import collections
import json
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # tools/: the shared contracts + the layout map
import _layout  # noqa: E402
_layout.add_import_paths()  # tools/, tools/analyzers, tools/publish — sibling-style imports keep working after the 2026-09-02 move
import _receipt

EXAMPLE_JOB = {
	"_project": "gearbox26 family — the original hardcoded bom_audit.py project, "
	            "kept as a worked example. Run it from a directory holding the three trees.",
	"assemblies": [
		{"name": "cyclo26", "step": "cyclo26/ASSEMBLY.step"},
		{"name": "harmonic26", "step": "harmonic26/ASSEMBLY.step"},
		{"name": "planetary26", "step": "planetary26/ASSEMBLY.step"},
	],
	"name_pattern": "hw_[a-z0-9_]+",
	"calibrate_with": "hw_nema17",
	"bom": {
		"hw_bearing_6804": {"label": "6804 bearing"},
		"hw_bearing_693zz": {"label": "693ZZ bearing (harmonic wave gen ONLY)", "only": ["harmonic26"]},
		"hw_bearing_688_ecc": {"label": "688 bearing (cyclo backdrivable eccentric ONLY)", "only": ["cyclo26"]},
		"hw_dowel_2x20": {"label": "Ø2×20 dowel (cyclo ring ONLY)", "only": ["cyclo26"]},
		"hw_m3x40_sandwich": {"label": "M3×40 button"},
		"hw_m3x30_sandwich": {"label": "M3×30 button (2026-07-19 tap-depth audit: M3×40 "
		                               "bottomed out in the ~4.5 mm blind NEMA-17 face taps)"},
		"hw_m3x12_pin": {"label": "M3×12 csk (output pins)"},
		"hw_m3x12_hub": {"label": "M3×12 csk (hub)"},
		"hw_m3x10_hub": {"label": "M3×10 csk (hub) — harmonic26 pilot-depth audit 2026-07-19"},
		"hw_m3x8_retainer": {"label": "M3×8 button (retainer)"},
		"hw_m3x8_axle": {"label": "M3×8 button (roller axles)", "only": ["harmonic26"]},
		"hw_m3x5_set": {"label": "M3×5 DIN916 set screw", "only": ["cyclo26", "harmonic26"]},
		"hw_m3x5_axle": {"label": "M3×5 DIN916 (planet axle = same screw as the set screw)",
		                 "only": ["planetary26"]},
		"hw_nema17": {"label": "NEMA-17 motor"},
	},
}


def resolve(path, job_dir, base_dir):
	"""base_dir -> job file's directory -> CWD; nothing found is a refusal."""
	if os.path.isabs(path):
		if os.path.exists(path):
			return path
		raise FileNotFoundError(f"{path!r} does not exist")
	roots = [("base_dir", base_dir)] if base_dir else []
	roots += [("job file's directory", job_dir), ("cwd", os.getcwd())]
	for _, root in roots:
		if root and os.path.exists(os.path.join(root, path)):
			return os.path.join(root, path)
	raise FileNotFoundError(f"{path!r} not found under any root (tried: "
	                        + ", ".join(k for k, r in roots if r) + ")")


# STEP entity types whose quoted strings can carry an instance / product name.
# `_CONTEXT` / `_CATEGORY` / `_RELATIONSHIP` rows are deliberately excluded: they
# carry the AP203 keywords, never a part name.
_NAME_ROW = re.compile(
	r"^(NEXT_ASSEMBLY_USAGE_OCCURRENCE|PRODUCT|PRODUCT_DEFINITION|PRODUCT_DEFINITION_FORMATION"
	r"(_WITH_SPECIFIED_SOURCE)?|PRODUCT_DEFINITION_SHAPE|SHAPE_DEFINITION_REPRESENTATION"
	r"|(ADVANCED_BREP_)?SHAPE_REPRESENTATION|MANIFOLD_SOLID_BREP|CLOSED_SHELL)$")


def count_names(step_text, pattern, calibrate_with, overhead):
	"""Instance counts per hardware name in one STEP tree.

	A name appears once per PRODUCT/PRODUCT_DEFINITION metadata row plus once per
	NEXT_ASSEMBLY_USAGE_OCCURRENCE, so the raw quoted-name count carries a fixed
	per-name overhead. It is CALIBRATED against a name known to occur exactly
	once, rather than assumed."""
	# Count ONLY inside the product / assembly-usage / shape entity records —
	# never over the whole file. A natural pattern such as `[a-z][a-z0-9_]+`
	# used to sweep up the AP203 vocabulary ('axis', 'refdir', 'design',
	# 'mechanical', 'config_control_design', 'placement', ...) from
	# AXIS2_PLACEMENT_3D / *_CONTEXT / *_CATEGORY / measure rows and report them
	# as undeclared hardware (slas F7).
	names = []
	for rec in re.finditer(r"#\d+\s*=\s*([A-Z0-9_]+)\s*\((.*?)\)\s*;", step_text, re.S):
		if _NAME_ROW.match(rec.group(1)):
			names.extend(re.findall(rf"'({pattern})'", rec.group(2)))
	c = collections.Counter(names)
	if calibrate_with is not None:
		if calibrate_with not in c:
			raise ValueError(f"calibrate_with {calibrate_with!r} does not occur in this STEP tree — "
			                 f"pick a name present in EVERY assembly, or set 'overhead' explicitly")
		overhead = c[calibrate_with] - 1
	return {k: v - overhead for k, v in c.items()}, overhead


def audit(job, job_dir=None):
	asms = job.get("assemblies")
	if not isinstance(asms, list) or not asms:
		raise ValueError("job.assemblies must be a non-empty list of {name, step}")
	bom = job.get("bom")
	if not isinstance(bom, dict) or not bom:
		raise ValueError("job.bom must be a non-empty {name: {label, only?, expect?}} map")
	pattern = str(job.get("name_pattern", "hw_[a-z0-9_]+"))
	calib = job.get("calibrate_with")
	overhead_default = int(job.get("overhead", 0))
	base_dir = job.get("base_dir")

	findings, out_asms = [], []
	grand = collections.Counter()
	seen = set()
	for a in asms:
		if "name" not in a or "step" not in a:
			raise ValueError(f"assembly entry {a!r} needs both 'name' and 'step'")
		name = str(a["name"])
		path = resolve(str(a["step"]), job_dir, base_dir)
		with open(path, encoding="utf-8", errors="replace") as f:
			counts, overhead = count_names(f.read(), pattern, calib, overhead_default)
		items = []
		for hw, n in sorted(counts.items()):
			grand[hw] += n
			seen.add(hw)
			spec = bom.get(hw)
			if spec is None:
				verdict = "NOT_IN_BOM"
				findings.append(f"{name}: {hw} ×{n} is NOT in the unified BOM")
			elif spec.get("only") and name not in spec["only"]:
				verdict = "NOT_ALLOWED_HERE"
				findings.append(f"{name}: {hw} ×{n} is restricted to {spec['only']}")
			else:
				verdict = "ok"
			items.append({"name": hw, "count": int(n),
			              "label": (spec or {}).get("label", ""), "verdict": verdict})
		out_asms.append({"name": name, "step": os.path.basename(path),
		                 "overhead": int(overhead), "items": items})

	for hw, spec in sorted(bom.items()):
		if hw not in seen:
			findings.append(f"{hw} is declared in the BOM but appears in NO assembly")
		elif "expect" in spec and int(spec["expect"]) != grand[hw]:
			findings.append(f"{hw}: family total {grand[hw]} != declared expect {spec['expect']}")

	return {"ok": not findings, "assemblies": out_asms,
	        "family_totals": {k: int(v) for k, v in sorted(grand.items())},
	        "findings": findings}


def _example():
	print(json.dumps(EXAMPLE_JOB, indent=1))
	return 0


def main(argv):
	# `<job.json> [--out PATH] | --example | --help` — the shared runner shape
	# (ratcheting F11: this was the fourth tool without `--out`).
	return _receipt.doc_cli("bom_audit", lambda job, job_dir: audit(job, job_dir=job_dir),
	                        help_text=__doc__, argv=argv[1:], extra_flags={"--example": _example})


if __name__ == "__main__":
	sys.exit(main(sys.argv))
