"""_layout.py — the map of tools/ after the 2026-09-02 re-organisation.

RETIREMENT DATE FOR THE FORWARDING SHIMS: **2027-03-02** (six months).
The 46 shims at the old flat `tools/<name>.py` paths exist so that campaigns
already written, and any CI job or command line quoting the old path, keep
working across the move. They are debt with an end date, not a permanent layer.
Before that date: regenerate the campaigns in the workspace against the real
paths (`tools/analyzers/…`, `tools/publish/…`), then delete the shims and drop
the "old path is a forwarding shim" checks from the runner-contract gate in
tools/analyzer_registry.py. `python3 tools/_layout.py --shims` lists what is
still standing.

tools/ used to be one flat directory of ~70 files. It is now:

    tools/
    ├── analyzers/    every registered analysis surface: the ACE runners
    │                 (ace_*_runner.py, graded_infill_runner.py), the shared
    │                 harness _ace.py, materials.py, the checkers
    │                 (tolerance_stack, production_check, joint_check,
    │                 sweep_check, balance_check, air_topology_audit),
    │                 param_optimize.py, derived_model.py, and the geometry
    │                 bridges the runners import (voxelize_stl.py,
    │                 stress_to_density.py)
    ├── publish/      renderers and document emitters: render_sheet,
    │                 render_views, analysis_sheet, assembly_doc, motion_gif,
    │                 production_dossier, document_bundle, make_all_plate,
    │                 bom_audit
    ├── validation/   the *_validation.py ground-truth pins
    ├── tests/        the gate suites (test_*.py, *_test.py)
    ├── manifests/    lmcad.manifest.v1 files (DATA — unchanged location, every
    │                 receipt's manifest_ref and every doc still names it)
    ├── materials/    material records (DATA — unchanged location; the Rust
    │                 tests and the cross-language pin read it by this path)
    ├── _parked/      orphaned tools kept out of the surface (see its README)
    └── *.py          the shared contracts and repo-wide utilities that every
                      group imports: _receipt.py, _stl.py, provenance.py,
                      analyzer_registry.py, check_ci_security.py, audit_docs,
                      gen_discover, ingest_calibration, dim_suggest,
                      field_report, field_triage — plus one FORWARDING SHIM
                      per moved script, so `python3 tools/<name>.py job.json`
                      keeps working for every campaign and CI job
                      (same argv, same stdout, same exit code).

This module is the ONE place that knows the map. A moved script puts tools/
on sys.path (its grandparent directory) and calls `add_import_paths()`; a test
or orchestrator that needs to spawn a sibling script asks `find_tool(name)`
instead of joining its own directory, so the tests never depend on where they
themselves live.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
REPO_ROOT = TOOLS.parent
ANALYZERS = TOOLS / "analyzers"
PUBLISH = TOOLS / "publish"
VALIDATION = TOOLS / "validation"
TESTS = TOOLS / "tests"
MANIFESTS = TOOLS / "manifests"
MATERIALS = TOOLS / "materials"
PARKED = TOOLS / "_parked"

#: Where a script may live, in lookup order. `_parked/` is deliberately absent:
#: a parked tool is not on the surface.
SEARCH_DIRS = (ANALYZERS, PUBLISH, VALIDATION, TESTS, TOOLS)

#: Every forwarding shim carries this marker in its first lines; the registry's
#: catalogue-drift scan and `find_tool` skip shims so the real file is always
#: the one found and hashed.
SHIM_MARKER = "LMCAD forwarding shim"


def is_shim(path: str | os.PathLike) -> bool:
	"""True iff `path` is one of the forwarding shims left at an old location."""
	try:
		with open(path, "r", encoding="utf-8", errors="replace") as fh:
			return SHIM_MARKER in fh.read(600)
	except OSError:
		return False


def find_tool(name: str) -> Path:
	"""Absolute path of the REAL script called `name` (e.g. 'ace_fea_runner.py'),
	wherever it lives now. A bare name is searched across SEARCH_DIRS (shims are
	skipped); a name with a directory part is resolved relative to tools/.
	Raises FileNotFoundError naming every directory searched — a typo must not
	become a silent fallback."""
	name = str(name)
	if os.path.isabs(name):
		return Path(name)
	if "/" in name or os.sep in name:
		p = TOOLS / name
		if p.is_file():
			return p
		raise FileNotFoundError(f"no tool at tools/{name}")
	for d in SEARCH_DIRS:
		p = d / name
		if p.is_file() and not is_shim(p):
			return p
	raise FileNotFoundError(
		f"no tool named {name!r} in {[str(d.relative_to(TOOLS)) or '.' for d in SEARCH_DIRS]} "
		f"(tools/_layout.py SEARCH_DIRS)")


def relative(name: str) -> str:
	"""'ace_fea_runner.py' -> 'analyzers/ace_fea_runner.py' (relative to tools/)."""
	return find_tool(name).relative_to(TOOLS).as_posix()


def add_import_paths() -> None:
	"""Put tools/, tools/analyzers and tools/publish on sys.path (front, in that
	priority order: publish < analyzers < tools) so sibling-style imports —
	`import _receipt`, `import materials`, `from render_sheet import STYLE` —
	resolve from any of the groups."""
	for d in (PUBLISH, ANALYZERS, TOOLS):
		s = str(d)
		if s in sys.path:
			sys.path.remove(s)
		sys.path.insert(0, s)
