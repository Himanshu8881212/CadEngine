#!/usr/bin/env python3
"""test_doc_contracts.py — execute the load-bearing examples in the operator docs.

WHY THIS EXISTS
---------------
T1 ("doc / digest / cookbook / `describe` drift") was the single most universal
tax in the 10-campaign portfolio: 9 of 10 campaigns paid wall-clock for a doc
that disagreed with the binary. Every one of those was a claim nobody could
*run*. `tools/audit_docs.py` checks that the docs are internally consistent —
that op names, section anchors and paths resolve. It cannot check that a
documented VALUE is still true.

This script closes that hole. Every assertion below is a claim written verbatim
in one of:

    campaign/DELIVERABLE_SPEC.md      §2.2  §2.5  §2.11  §2.13  §3
    campaign/OPERATOR_BRIEF.md        §3.1  §3.2  §7     §8
    campaign/digests/ops_core.md      §10   §11a  §11b   "ENGINE UPDATE"
    campaign/digests/tools_cookbook.md  wire contract, ace_fea loads, ace_fatigue
    campaign/digests/analysis_honesty.md  creep, determinism
    DESIGN_GUIDE.md §22

If a fix phase changes the behaviour, THIS FILE FAILS FIRST and names the doc
section that has gone stale. That is the point: the binary and the tools are the
authority, and the docs are now gated on them.

Run:  python3 "docs/test_doc_contracts.py"          (from the repo root, or anywhere)
      python3 "docs/test_doc_contracts.py" -v       (print every check)

Exit 0 iff every contract holds. Checks whose prerequisite is absent (no built
binary, no ACE install) are reported SKIP and do not fail the run — a missing
prerequisite is not a doc defect.
"""

import json
import os
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KERNEL = os.path.join(REPO, "target", "release", "kernel-api")
VERBOSE = "-v" in sys.argv[1:]

_results = []


def check(name, doc_ref):
	"""Decorator: register a contract check and the doc section it defends."""
	def wrap(fn):
		_results.append((name, doc_ref, fn))
		return fn
	return wrap


class Skip(Exception):
	pass


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------

def run_program(ops, out_dir=None, program_dir=None):
	"""Run a JSON program through `kernel-api run`. Returns (report, exit_code)."""
	if not os.path.exists(KERNEL):
		raise Skip(f"no binary at {KERNEL} — build with `cargo build --release -p kernel-api`")
	tmp = tempfile.mkdtemp(prefix="doccontract_")
	program_dir = program_dir or tmp
	out_dir = out_dir or os.path.join(tmp, "out")
	os.makedirs(program_dir, exist_ok=True)
	os.makedirs(out_dir, exist_ok=True)
	path = os.path.join(program_dir, "p.json")
	with open(path, "w") as fh:
		json.dump({"ops": ops}, fh)
	proc = subprocess.run([KERNEL, "run", path, "--out-dir", out_dir],
	                      capture_output=True, text=True)
	try:
		return json.loads(proc.stdout), proc.returncode
	except json.JSONDecodeError:
		raise AssertionError(f"kernel-api emitted no JSON report; stderr={proc.stderr[:400]!r}")


def measures(report):
	return {o["id"]: o.get("measures", {}) for o in report["ops"]}


def op_error(report, op_id):
	for o in report["ops"]:
		if o["id"] == op_id:
			return o.get("error")
	return None


def eq(actual, expected, what):
	assert actual == expected, f"{what}: expected {expected!r}, measured {actual!r}"


def box(i, lo, hi):
	return {"id": i, "op": "box", "min": lo, "max": hi}


def square_section(half, z):
	return [[-half, -half, z], [half, -half, z], [half, half, z], [-half, half, z]]


# --------------------------------------------------------------------------
# §2.2 / ops_core "ENGINE UPDATE" — the two connectivity oracles
# --------------------------------------------------------------------------

@check("severed bar fires BOTH shells and components",
       "DELIVERABLE_SPEC §2.2 table row 1; exemplars.md 'Connectivity is TWO separate oracles'")
def t_severed_bar():
	rep, _ = run_program([
		box("bar", [-20, -5, -5], [20, 5, 5]),
		box("knife", [-1, -10, -10], [1, 10, 10]),
		{"id": "cut", "op": "difference", "a": "bar", "b": "knife"},
		{"id": "v", "op": "validate", "in": "cut"},
		{"id": "mc", "op": "mesh_components", "in": "cut"},
	])
	m = measures(rep)
	eq(m["v"]["shells"], 2, "severed bar validate.shells")
	eq(m["mc"]["components"], 2, "severed bar mesh_components.components")
	eq(m["v"]["valid"], True, "severed bar is still `valid` (that is the whole point)")


@check("sub-weld-tol sever: components is the WEAKER check",
       "DELIVERABLE_SPEC §2.2 table row 2 — the inversion the spec used to state backwards")
def t_subweld_sever():
	rep, _ = run_program([
		box("a", [0, 0, 0], [10, 10, 10]),
		box("b", [10.0005, 0, 0], [20, 10, 10]),
		{"id": "u", "op": "union_all", "in": ["a", "b"]},
		{"id": "v", "op": "validate", "in": "u"},
		{"id": "c", "op": "mesh_components", "in": "u"},
	])
	m = measures(rep)
	eq(m["v"]["shells"], 2, "0.0005 mm sever validate.shells")
	eq(m["c"]["components"], 1, "0.0005 mm sever mesh_components.components")
	eq(m["c"]["is_one_body"], True, "0.0005 mm sever is_one_body")


@check("NC-B is constructible: components PASSES, shells FAILS, exit 1",
       "DELIVERABLE_SPEC §2.13 NC-B (the replacement for the un-constructible old example)")
def t_oracle_nc_b():
	rep, rc = run_program([
		box("a", [0, 0, 0], [10, 10, 10]),
		box("b", [10.0005, 0, 0], [20, 10, 10]),
		{"id": "u", "op": "union_all", "in": ["a", "b"]},
		{"id": "c", "op": "mesh_components", "in": "u"},
		{"id": "g_comp", "op": "assert", "in": "u", "components": 1},
		{"id": "g_shell", "op": "assert", "in": "u", "shells": 1},
	])
	ok = {o["id"]: o["ok"] for o in rep["ops"]}
	eq(ok["g_comp"], True, "NC-B: the components gate must PASS (that is what it proves)")
	eq(ok["g_shell"], False, "NC-B: the shells gate must FAIL")
	eq(op_error(rep, "g_shell")["kind"], "assert_failed", "NC-B failure kind")
	eq(rc, 1, "NC-B process exit code")


@check("NC-A is constructible: components FAILS, exit 1",
       "DELIVERABLE_SPEC §2.13 NC-A")
def t_oracle_nc_a():
	rep, rc = run_program([
		box("bar", [-20, -5, -5], [20, 5, 5]),
		box("knife", [-1, -10, -10], [1, 10, 10]),
		{"id": "cut", "op": "difference", "a": "bar", "b": "knife"},
		{"id": "g_comp", "op": "assert", "in": "cut", "components": 1},
	])
	eq(op_error(rep, "g_comp")["kind"], "assert_failed", "NC-A failure kind")
	eq(rc, 1, "NC-A process exit code")


@check("components counts truly on a holed plate (closed tessellation since the f64 predicate fix)",
       "DELIVERABLE_SPEC 2.2 table row 3 update note; fixlog 2026-08-27-pose-robustness")
def t_hole_loop_refusal():
	# HISTORY: until 2026-08-27 the 2-hole plate's tessellation left boundary
	# edges (the cap triangulator's cleanliness check false-positived at f32),
	# so mesh_components REFUSED rather than count faceter cracks - that
	# refusal was itself this contract. The f64 predicate fix closed the
	# tessellation, so the oracle now has a trustworthy surface and must
	# return the TRUE body count. The refusal channel still exists for
	# genuinely unclosed tessellations; what this pins is that the holed
	# plate is no longer one of them, in either direction of degradation.
	geom = {"id": "p", "op": "extrude_with_holes",
	        "outer": [[0, 0], [40, 0], [40, 20], [0, 20]],
	        "holes": [[[5, 5], [10, 5], [10, 10], [5, 10]],
	                  [[25, 5], [30, 5], [30, 10], [25, 10]]],
	        "height": 5}
	rep, rc = run_program([geom,
	                       {"id": "v", "op": "validate", "in": "p"},
	                       {"id": "c", "op": "mesh_components", "in": "p"}])
	m = measures(rep)
	eq(m["v"]["shells"], 1, "2-hole plate validate.shells - shells stays trustworthy")
	eq(m["v"]["genus"], 2, "2-hole plate genus")
	eq(op_error(rep, "c"), None, "the closed tessellation must not refuse")
	eq(m["c"]["components"], 1, "true body count, not a faceter-crack count")
	eq(m["c"]["boundary_edges"], 0, "the measurement surface is closed")
	eq(rc, 0, "clean run")

	# ...and the gate passes on the same trustworthy walk
	rep, rc = run_program([geom, {"id": "g", "op": "assert", "in": "p", "components": 1}])
	eq(op_error(rep, "g"), None, "assert components:1 must pass on the closed walk")
	eq(rc, 0, "gate run exits 0")


@check("weld band: welded strictly below weld_tol 0.001 mm, separate strictly above",
       "DELIVERABLE_SPEC §2.2 weld-scale table; ops_core §'ENGINE UPDATE'")
def t_weld_band():
	for base in (0.0, 10.0, 100.0):
		for gap, expect in ((0.0001, 1), (0.0005, 1), (0.0009, 1), (0.0011, 2), (0.0015, 2)):
			rep, _ = run_program([
				box("a", [base - 10, 0, 0], [base, 10, 10]),
				box("b", [base + gap, 0, 0], [base + 10, 10, 10]),
				{"id": "u", "op": "union_all", "in": ["a", "b"]},
				{"id": "v", "op": "validate", "in": "u"},
				{"id": "c", "op": "mesh_components", "in": "u"},
			])
			m = measures(rep)
			eq(m["v"]["shells"], 2, f"base {base} gap {gap}: shells always 2")
			eq(m["c"]["components"], expect, f"base {base} gap {gap}: components")


@check("weld_tol tunes the MEASURE and the GATE, from one shared default",
       "DELIVERABLE_SPEC §2.2 last paragraph; refutes T6's 'fixed, non-tunable weld_tol'")
def t_weld_tol_asymmetry():
	# UPDATED 2026-08-08: this contract used to pin `assert`'s param set by
	# ENUMERATION and assert the absence of `tol`/`weld_tol`. The kernel then
	# added both (T6a) — the improvement the spec had asked for — and the
	# contract failed for being right about yesterday. An enumeration is not the
	# property; it breaks on every additive param. What is pinned now is the
	# property the spec actually promises: the gate is tunable, its defaults are
	# the measure's defaults, and a tightened gate catches a sever the default
	# one passes.
	pair = [box("a", [0, 0, 0], [10, 10, 10]),
	        box("b", [10.0005, 0, 0], [20, 10, 10]),
	        {"id": "u", "op": "union_all", "in": ["a", "b"]}]
	rep, _ = run_program(pair + [
		{"id": "c_default", "op": "mesh_components", "in": "u"},
		{"id": "c_tight", "op": "mesh_components", "in": "u", "weld_tol": 0.0001},
		{"id": "a_default", "op": "assert", "in": "u", "components": 1},
		{"id": "d", "op": "describe", "name": "assert"},
	])
	m = measures(rep)
	eq(m["c_default"]["components"], 1, "default weld_tol")
	eq(m["c_default"]["weld_tol"], 0.001, "default weld_tol echoed")
	eq(m["c_tight"]["components"], 2, "weld_tol 0.0001 separates the pair")
	eq(m["c_tight"]["weld_tol"], 0.0001, "tightened weld_tol echoed in the receipt")
	names = {p["name"] for p in m["d"]["params"]}
	missing = {"in", "components", "shells", "genus", "closed", "manifold", "valid",
	           "tol", "weld_tol", "require"} - names
	eq(missing, set(), "assert must expose the gate keys AND the two connectivity tolerances")

	# The gate at its DEFAULTS agrees with the measure at its defaults...
	rep_ok, rc_ok = run_program(pair + [{"id": "g", "op": "assert", "in": "u", "components": 1}])
	eq(rc_ok, 0, "assert components:1 passes at the default weld scale, as it always has")
	# ...and tightened, it fails the same sever the tightened measure sees.
	rep_bad, rc_bad = run_program(pair + [
		{"id": "g", "op": "assert", "in": "u", "components": 1, "weld_tol": 0.0001}])
	eq(rc_bad, 1, "a tightened assert must FAIL the 0.0005 mm sever")
	eq(rep_bad["ops"][-1]["error"]["kind"], "assert_failed", "and fail as a gate, not as an error")


@check("`require` is a universal gate on every measure op (closes SPEC §2.4/2.5/2.6/2.7)",
       "DELIVERABLE_SPEC §2 'Use require'; OPERATOR_BRIEF §1.3")
def t_require_universal_gate():
	geom = [box("post", [0, 0, 0], [5, 10, 20]),
	        box("arm", [5, 0, 15], [20, 10, 20]),
	        {"id": "u", "op": "union", "a": "post", "b": "arm"}]

	# every gate the spec mandates, expressed IN the program
	rep, rc = run_program(geom + [
		{"id": "e", "op": "export_stl", "in": "u", "file": "x.stl",
		 "require": {"watertight": True, "route": "exact"}},
		{"id": "bb", "op": "bounding_box", "in": "u", "envelope": [256, 256, 256],
		 "require": {"fits_within": True}},
		{"id": "w", "op": "wall_thickness", "in": "u", "flag_below": 1.6,
		 "require": {"thin_area": 0.0}},
		{"id": "s", "op": "support_report", "in": "u", "build_dir": [0, 0, 1],
		 "require": {"steep_area": 0.0}},
	])
	eq(rc, 0, "all four require-gates must pass on this body")
	m = measures(rep)
	for op_id, key in (("e", "watertight"), ("bb", "fits_within"),
	                   ("w", "thin_area"), ("s", "steep_area")):
		assert "required" in m[op_id], \
			f"{op_id}: a met `require` must echo a `required` block into the measures"
		assert key in m[op_id]["required"], f"{op_id}: `required` must record what was gated"

	# and an UNMET expectation fails the run
	rep, rc = run_program(geom + [
		{"id": "g", "op": "support_report", "in": "u", "build_dir": [0, 0, 1],
		 "require": {"max_bridge_span": {"max": 5.0}}},
	])
	err = op_error(rep, "g")
	eq(err["kind"], "assert_failed", "an unmet require is assert_failed")
	assert "require failed: max_bridge_span: measured 10.0, expected <= 5" in err["message"], \
		f"the require failure message drifted: {err['message']}"
	eq(rc, 1, "an unmet require exits 1")

	# require carries the tolerance the plain `assert` cannot take (§2.2)
	rep, rc = run_program([
		box("a", [0, 0, 0], [10, 10, 10]),
		box("b", [10.0005, 0, 0], [20, 10, 10]),
		{"id": "u", "op": "union_all", "in": ["a", "b"]},
		{"id": "loose", "op": "assert", "in": "u", "components": 1},
		{"id": "tight", "op": "mesh_components", "in": "u", "weld_tol": 0.0001,
		 "require": {"components": 1}},
	])
	ok = {o["id"]: o["ok"] for o in rep["ops"]}
	eq(ok["loose"], True, "the default-tolerance gate passes the sub-weld-tol sever")
	eq(ok["tight"], False, "the tightened require-gate catches it")
	eq(rc, 1, "tightened gate exits 1")


# --------------------------------------------------------------------------
# §2.5 / ops_core §11a / DESIGN_GUIDE §22 — support_report semantics
# --------------------------------------------------------------------------

@check("build_dir points AWAY from the bed",
       "DELIVERABLE_SPEC §2.5; ops_core §11a; DESIGN_GUIDE §22")
def t_build_dir_polarity():
	geom = [
		box("post", [0, 0, 0], [5, 10, 20]),
		box("arm", [5, 0, 15], [20, 10, 20]),
		{"id": "u", "op": "union", "a": "post", "b": "arm"},
	]
	rep, _ = run_program(geom + [
		{"id": "s_up", "op": "support_report", "in": "u", "build_dir": [0, 0, 1]},
		{"id": "s_dn", "op": "support_report", "in": "u", "build_dir": [0, 0, -1]},
	])
	m = measures(rep)
	eq(m["s_up"]["bed_area"], 50.0, "build_dir [0,0,1] -> bed at min-Z (the 5x10 foot)")
	eq(m["s_dn"]["bed_area"], 200.0, "build_dir [0,0,-1] -> bed at max-Z (the two top faces)")
	eq(m["s_up"]["bridge_area"], 150.0, "the arm underside is BRIDGE, not steep")
	eq(m["s_up"]["steep_area"], 0.0, "a horizontal underside is not steep")
	eq(m["s_up"]["support_free"], True,
	   "support_free TRUE with 150 mm2 of bridging — why max_bridge_span must be quoted too")


@check("a LARGER overhang_deg is MORE permissive; default is 45",
       "DELIVERABLE_SPEC §2.5; ops_core §11a table")
def t_overhang_polarity():
	import math
	# lofted frustum whose wall tilts `tilt` degrees away from the build direction
	def frustum(tag, tilt_deg, rise=10.0, base_half=5.0):
		run = rise * math.tan(math.radians(tilt_deg))
		return {"id": tag, "op": "loft",
		        "sections": [square_section(base_half, 0.0),
		                     square_section(base_half + run, rise)]}

	# 45 deg wall: steep at 44, clean at 45 (strict >)
	rep, _ = run_program([
		frustum("f45", 45.0),
		{"id": "s44", "op": "support_report", "in": "f45", "build_dir": [0, 0, 1], "overhang_deg": 44},
		{"id": "s45", "op": "support_report", "in": "f45", "build_dir": [0, 0, 1], "overhang_deg": 45},
	])
	m = measures(rep)
	assert m["s44"]["steep_area"] > 0.0, "45 deg wall must be STEEP at overhang_deg 44"
	eq(m["s45"]["steep_area"], 0.0, "45 deg wall must be CLEAN at overhang_deg 45")

	# 63.435 deg wall: steep at 63, clean at 64 -> the threshold is measured from build_dir
	rep, _ = run_program([
		frustum("f63", math.degrees(math.atan(2.0))),  # run 20 over rise 10
		{"id": "s63", "op": "support_report", "in": "f63", "build_dir": [0, 0, 1], "overhang_deg": 63},
		{"id": "s64", "op": "support_report", "in": "f63", "build_dir": [0, 0, 1], "overhang_deg": 64},
	])
	m = measures(rep)
	assert m["s63"]["steep_area"] > 0.0, "63.4 deg wall must be STEEP at overhang_deg 63"
	eq(m["s64"]["steep_area"], 0.0, "63.4 deg wall must be CLEAN at overhang_deg 64")

	# default: steep at 46 deg tilt, clean at 44 deg tilt  =>  default == 45
	rep, _ = run_program([
		frustum("f44", 44.0), frustum("f46", 46.0),
		{"id": "d44", "op": "support_report", "in": "f44", "build_dir": [0, 0, 1]},
		{"id": "d46", "op": "support_report", "in": "f46", "build_dir": [0, 0, 1]},
	])
	m = measures(rep)
	eq(m["d44"]["steep_area"], 0.0, "default overhang_deg: 44 deg tilt is clean")
	assert m["d46"]["steep_area"] > 0.0, "default overhang_deg: 46 deg tilt is steep => default 45"


@check("max_bridge_span is the SHORT way across the bridging region",
       "DELIVERABLE_SPEC §2.5; ops_core §11a")
def t_bridge_span():
	def deck(depth):
		return [
			box("p1", [0, 0, 0], [5, depth, 20]),
			box("p2", [35, 0, 0], [40, depth, 20]),
			box("deck", [0, 0, 15], [40, depth, 20]),
			{"id": "u1", "op": "union", "a": "p1", "b": "p2"},
			{"id": "u", "op": "union", "a": "u1", "b": "deck"},
			{"id": "s", "op": "support_report", "in": "u", "build_dir": [0, 0, 1]},
		]
	rep, _ = run_program(deck(8))     # underside 30 x 8
	eq(measures(rep)["s"]["max_bridge_span"], 8.0, "30x8 underside -> short way = 8")
	rep, _ = run_program(deck(50))    # underside 30 x 50
	eq(measures(rep)["s"]["max_bridge_span"], 30.0, "30x50 underside -> short way = 30")


# --------------------------------------------------------------------------
# §2.11 / ops_core §11b — clearance on nested pairs + the grown-gauge bracket
# --------------------------------------------------------------------------

def _tube_and(radius, tag):
	return [
		{"id": "outer", "op": "cylinder", "base": [0, 0, 0], "axis": [0, 0, 1], "radius": 10, "height": 20},
		{"id": "bore", "op": "cylinder", "base": [0, 0, -1], "axis": [0, 0, 1], "radius": 6, "height": 22},
		{"id": "tube", "op": "difference", "a": "outer", "b": "bore"},
		{"id": tag, "op": "cylinder", "base": [0, 0, 2], "axis": [0, 0, 1], "radius": radius, "height": 16},
	]


@check("clearance on a NESTED pair returns a faceted distance that UNDER-reads ~10%",
       "DELIVERABLE_SPEC §2.11; OPERATOR_BRIEF §8; ops_core §11b")
def t_clearance_nested_underreads():
	rep, _ = run_program(_tube_and(5.7, "pin") + [
		{"id": "c_nested", "op": "clearance", "a": "tube", "b": "pin"},
		box("far", [-100, -100, -100], [-90, -90, -90]),
		{"id": "c_far", "op": "clearance", "a": "tube", "b": "far"},
	])
	m = measures(rep)
	d = m["c_nested"]["distance"]
	# true radial gap is 0.300 mm; the measure is `faceted` and reads LOW (conservative).
	assert 0.25 < d < 0.300, \
		f"nested clearance must be a faceted UNDER-read of the 0.300 mm gap, got {d}"
	assert abs(d - 0.2711) < 5e-3, f"the documented 0.2711 mm has drifted to {d}"
	eq(m["c_nested"]["provenance"], "faceted", "clearance provenance — why it under-reads")
	eq(m["c_nested"]["interfering"], False, "nested pair is not interfering")
	assert m["c_far"]["distance"] > 100.0, \
		f"separated pair must return a real distance, got {m['c_far']['distance']}"


@check("assert_disjoint now PASSES the nested pair it used to false-fail",
       "DELIVERABLE_SPEC §2.11; ops_core §9 + §11b; OPERATOR_BRIEF §4")
def t_assert_disjoint_nested_passes():
	rep, rc = run_program(_tube_and(5.7, "pin") + [
		{"id": "ad", "op": "assert_disjoint", "a": "tube", "b": "pin"},
	])
	assert op_error(rep, "ad") is None, \
		f"assert_disjoint must pass a 0.30 mm-gap nested pair; got {op_error(rep, 'ad')}"
	eq(rc, 0, "assert_disjoint exit code")


@check("grown-gauge bracket: refuses below the gap, binds an analytic volume above it",
       "DELIVERABLE_SPEC §2.11 table; ops_core §11b table")
def t_grown_gauge_bracket():
	rep, rc = run_program(_tube_and(5.99, "gauge") + [
		{"id": "inter", "op": "intersection", "a": "tube", "b": "gauge"},
	])
	err = op_error(rep, "inter")
	assert err is not None, "delta 0.29 mm must still REFUSE (that is the lower bracket)"
	eq(err["kind"], "invalid_param", "empty-intersection refusal kind")
	eq(rc, 1, "refusing bracket run exits 1 — the report IS the evidence")

	rep, _ = run_program(_tube_and(6.01, "gauge") + [
		{"id": "inter", "op": "intersection", "a": "tube", "b": "gauge"},
		{"id": "x", "op": "exact_volume", "in": "inter"},
	])
	m = measures(rep)
	eq(m["x"]["provenance"], "analytic", "bracket volume provenance (better than `faceted`)")
	assert abs(m["x"]["exact_volume"] - 6.0369) < 1e-3, \
		f"delta 0.31 mm bracket volume: expected ~6.0369 mm3, got {m['x']['exact_volume']}"


# --------------------------------------------------------------------------
# ops_core §10 / OPERATOR_BRIEF §3.2 — the path-root asymmetry
# --------------------------------------------------------------------------

@check("export joins --out-dir; import: program dir first, out-dir fallback, '..' refused",
       "ops_core 10 (updated 2026-08-27); OPERATOR_BRIEF 3.2; resolve_input_or_out doc")
def t_path_root_asymmetry():
	if not os.path.exists(KERNEL):
		raise Skip("no binary")
	tmp = tempfile.mkdtemp(prefix="doccontract_paths_")
	prog, out = os.path.join(tmp, "prog"), os.path.join(tmp, "out")
	os.makedirs(prog); os.makedirs(out)

	run_program([box("b", [0, 0, 0], [10, 10, 10]),
	             {"id": "e", "op": "export_step", "in": "b", "file": "b.step"}],
	            out_dir=out, program_dir=prog)
	assert os.path.exists(os.path.join(out, "b.step")), "export_step must write under --out-dir"
	assert not os.path.exists(os.path.join(prog, "b.step")), "export must NOT write beside the program"

	# Since the T4 heal, import resolves against the PROGRAM dir FIRST and
	# falls back to --out-dir when the file only exists there - so the
	# export-then-import round trip with the same relative name now WORKS.
	rep, _ = run_program([{"id": "i", "op": "import_step", "file": "b.step"}],
	                     out_dir=out, program_dir=prog)
	assert op_error(rep, "i") is None, \
		f"import_step must fall back to --out-dir when absent beside the program; got {op_error(rep, 'i')}"

	rep, _ = run_program([{"id": "i", "op": "import_step", "file": "../out/b.step"}],
	                     out_dir=out, program_dir=prog)
	err = op_error(rep, "i")
	assert err and err["kind"] == "invalid_param" and ".." in err["message"], \
		f"import_step must refuse '..'; got {err}"

	for op in ("import_mesh", "mesh_carve"):
		ops = ([{"id": "i", "op": "import_mesh", "file": "b.stl"}] if op == "import_mesh" else
		       [box("s", [0, 0, 0], [20, 20, 20]),
		        {"id": "c", "op": "mesh_carve", "in": "s", "file": "b.stl",
		         "bool": "difference", "out": "carved.stl"}])
		rep, _ = run_program(ops, out_dir=out, program_dir=prog)
		err = op_error(rep, "i" if op == "import_mesh" else "c")
		assert err and prog in err["message"], f"{op}.file must resolve against the program dir"


# --------------------------------------------------------------------------
# tools_cookbook — wire contract, ace_fatigue stress block, ace_fea body load
# --------------------------------------------------------------------------

def _tool(name):
	"""The REAL file of a tool (tools/{analyzers,publish,...}/<name> since the
	2026-09-02 layout), never the forwarding shim at the old flat path — the
	source-text contracts below must read the runner, not its 12-line pointer."""
	sys.path.insert(0, os.path.join(REPO, "tools"))
	try:
		import _layout  # noqa: PLC0415
		return str(_layout.find_tool(name))
	except (ImportError, FileNotFoundError):
		pass
	p = os.path.join(REPO, "tools", name)
	if not os.path.exists(p):
		raise Skip(f"missing tools/{name}")
	return p


def run_tool(name, job, extra_argv=()):
	"""Run a Python tool on a job dict. Returns (last_json_line, exit_code)."""
	path = _tool(name)
	tmp = tempfile.mkdtemp(prefix="doccontract_tool_")
	jp = os.path.join(tmp, "job.json")
	job = dict(job)
	job.setdefault("out_dir", tmp)
	with open(jp, "w") as fh:
		json.dump(job, fh)
	proc = subprocess.run([sys.executable, path, jp, *extra_argv],
	                      capture_output=True, text=True, cwd=REPO)
	lines = [ln for ln in proc.stdout.splitlines() if ln.strip()]
	if not lines:
		return None, proc.returncode
	try:
		return json.loads(lines[-1]), proc.returncode
	except json.JSONDecodeError:
		return None, proc.returncode


@check("ace_fatigue: `stress` is a NESTED block; the top-level form is REFUSED (exit 2)",
       "tools_cookbook.md ace_fatigue_runner — the line 3 campaigns hit")
def t_fatigue_stress_nesting():
	base = {"material": "PLA", "load_orientation": "in_plane",
	        "spectrum": [{"cycles": 100000, "r_ratio": 0.0}]}

	bad, rc = run_tool("ace_fatigue_runner.py", {**base, "sigma_ref_mpa": 4.15})
	assert bad is not None, "a refusal must still emit a receipt line"
	eq(bad["ok"], False, "top-level sigma_ref_mpa must be REFUSED")
	assert "stress block required" in bad["error"], f"refusal text drifted: {bad['error']}"
	eq(rc, 2, "a REFUSAL is exit 2 — the tool RAN and refused (cookbook wire-contract table)")
	eq(bad["error_kind"], "refusal.JobError", "error_kind must be machine-matchable")

	good, rc = run_tool("ace_fatigue_runner.py", {**base, "stress": {"sigma_ref_mpa": 4.15}})
	eq(good["ok"], True, "nested stress block must be accepted")
	eq(good["stress_input"]["mode"], "scalar", "nested scalar mode")
	eq(rc, 0, "ace_fatigue exits 0 on success")


@check("ace_fea body loads are N/kg (acceleration), NOT N/m^3",
       "tools_cookbook.md ace_fea_runner 'the 1240x trap'")
def t_body_load_unit_is_per_mass():
	# The authority is the ACE source; the digest is the artefact that drifted.
	src = os.path.expanduser("~/Work/ACE/engine/verify/fea.py")
	if not os.path.exists(src):
		raise Skip("ACE not installed at ~/Work/ACE")
	with open(src) as fh:
		text = fh.read()
	assert "magnitude(N/kg)" in text and "tributary" in text, \
		"ACE's _apply_body_load no longer states the N/kg-x-tributary-mass convention — " \
		"re-derive the unit before trusting tools_cookbook.md"


@check("assembly_doc.explode.axis accepts BOTH a 3-vector and an axis name",
       "tools_cookbook.md assembly_doc — the ball F6 / singulator F12 fix")
def t_explode_axis_forms():
	sys.path.insert(0, os.path.join(REPO, "tools"))
	try:
		import numpy as np
		import assembly_doc as ad
	except Exception as exc:
		raise Skip(f"assembly_doc not importable: {exc}")
	assert hasattr(ad, "parse_axis"), "assembly_doc.parse_axis is gone — re-check the cookbook"
	for form, want in (([0, 0, 1], [0, 0, 1]), ("z", [0, 0, 1]),
	                   ("+z", [0, 0, 1]), ("-z", [0, 0, -1])):
		v = np.asarray(ad.parse_axis(form, "explode.axis"), dtype=float)
		v = v / np.linalg.norm(v)
		assert np.allclose(v, want), f"explode.axis {form!r} -> {v}, expected {want}"
	try:
		ad.parse_axis("q", "explode.axis")
	except ValueError:
		pass
	else:
		raise AssertionError("an unknown axis name must REFUSE, not default silently")


@check("the THREE-code exit contract: 0 ok / 1 could-not-run / 2 ran-and-refused",
       "tools_cookbook.md wire contract; OPERATOR_BRIEF §3.1; tools/_receipt.py docstring")
def t_three_code_exit_contract():
	ok, rc = run_tool("tolerance_stack.py",
	                  {"fit": {"bore": {"nominal": 8.4, "tol": 0.15},
	                           "shaft": {"nominal": 8.0, "tol": 0.15}}})
	eq(ok["ok"], True, "0.4 mm nominal clearance passes at +/-0.15")
	eq(rc, 0, "exit 0 iff ok:true")

	fail, rc = run_tool("tolerance_stack.py",
	                    {"fit": {"bore": {"nominal": 8.0, "tol": 0.15},
	                             "shaft": {"nominal": 8.0, "tol": 0.15}}})
	eq(fail["ok"], False, "zero-clearance fit must read ok:false")
	eq(rc, 2, "a FAILED analysis is exit 2 — the tool RAN")
	eq(fail["exit_contract"]["mode"], "strict", "strict is the default, everywhere")
	assert "do not quote this receipt" in fail["exit_contract"]["contract"], \
		"the self-describing exit_contract text drifted"

	bad, rc = run_tool("tolerance_stack.py", {"fit": {"bore": {"nominal": 8.4}}},
	                   extra_argv=("--nope",))
	eq(rc, 1, "an unknown flag means the tool COULD NOT RUN -> exit 1, not 2")
	eq(bad["error_kind"], "refusal.usage", "error_kind must be machine-matchable")


@check("every runner shares ONE exit contract; the legacy opt-out is on the record",
       "tools_cookbook.md wire contract; OPERATOR_BRIEF §3.1")
def t_one_exit_contract_for_all_runners():
	# The old two-family split (six ACE runners exiting 0 on failure) is gone: every runner
	# routes its exit through tools/_receipt.py. If that stops being true, the wire-contract
	# table has to go back to naming exceptions.
	with open(_tool("_receipt.py")) as fh:
		rcpt = fh.read()
	for token in ("EXIT_OK = 0", "EXIT_ERROR = 1", "EXIT_REFUSED = 2",
	              "LMCAD_RUNNER_EXIT", "legacy_exit_zero", "wall_budget_s",
	              "LMCAD_RECEIPT_DRY_RUN", "receipt_path_conflict"):
		assert token in rcpt, f"tools/_receipt.py no longer defines {token!r}"
	for name in ("ace_fea_runner.py", "ace_fea_tet_runner.py", "ace_modal_runner.py",
	             "ace_buckling_runner.py", "ace_optimize_runner.py",
	             "graded_infill_runner.py", "ace_fatigue_runner.py",
	             "tolerance_stack.py", "production_check.py"):
		with open(_tool(name)) as fh:
			text = fh.read()
		assert "_receipt" in text, f"{name} no longer shares the _receipt exit contract"


# --------------------------------------------------------------------------
# analysis_honesty / OPERATOR_BRIEF §7 — the creep surface
# --------------------------------------------------------------------------

@check("creep: the reader rounds temperature UP, so 25 C reads at the 55 C row",
       "OPERATOR_BRIEF §7 'Creep'; analysis_honesty.md 'THE TRAP'")
def t_creep_rounds_temperature_up():
	sys.path.insert(0, os.path.join(REPO, "tools"))
	try:
		import materials as M
	except Exception as exc:
		raise Skip(f"tools/materials.py not importable: {exc}")

	eq(M.creep_allowable_mpa("PLA", 23, 1), 7.5, "23 C / 1 h cell")
	eq(M.creep_allowable_mpa("PLA", 23, 8760), 2.5, "23 C / 1 y cell")
	eq(round(M.creep_allowable_mpa("PLA", 23, 8760, across_layer=True), 6), 1.375,
	   "across_layer applies 0.55 INSIDE the lookup")

	r = M.creep_lookup("PLA", 25, 720)
	eq(r["sig_allow_mpa"], 0.5, "25 C / 30 d reads the 55 C row")
	eq(r["row_used_c"], 55.0, "row_used_c must expose WHICH cell the margin came from")
	eq(r["cell_match"], "rounded_up_conservative", "cell_match tag")

	r = M.creep_lookup("PLA", 56, 1)
	eq(r["refused"], True, "above 55 C the reader REFUSES")
	eq(r["refusal_kind"], "creep_temp_above_tabulated", "refusal kind")
	assert "does NOT fall back" in r["note"], "the no-fallback promise is gone from the note"


@check("determinism.core_digest reproduces across runs while timings_s does not",
       "DELIVERABLE_SPEC §3 determinism table; analysis_honesty.md 'Never cmp a receipt'")
def t_core_digest_is_the_comparison():
	sys.path.insert(0, os.path.join(REPO, "tools"))
	job = {"material": "PLA", "load_orientation": "in_plane",
	       "stress": {"sigma_ref_mpa": 4.15},
	       "spectrum": [{"cycles": 100000, "r_ratio": 0.0}]}
	a_rec, _ = run_tool("ace_fatigue_runner.py", job)
	b_rec, _ = run_tool("ace_fatigue_runner.py", job)
	assert a_rec and "timings_s" in a_rec, "the wall-clock block that makes byte-diffs fail is gone"
	det = a_rec.get("determinism")
	assert det, "receipt carries no `determinism` block — the docs promise one"
	eq(det["schema"], "lmcad.determinism.v1", "determinism schema")
	assert "timings_s" in det["nondeterministic_paths"], "timings_s must be declared non-deterministic"
	assert det["core_digest"].startswith("sha256:"), "core_digest form"
	assert det["core_sig_figs"] >= 6, "core_sig_figs must be a usable precision"
	eq(a_rec["determinism"]["core_digest"], b_rec["determinism"]["core_digest"],
	   "core_digest MUST reproduce across runs — that is the whole contract")
	# timings_s is DECLARED non-deterministic; it need not differ on a sub-millisecond
	# run, but it must be excluded from the digest — prove that by perturbing it.
	import copy
	perturbed = copy.deepcopy(a_rec)
	perturbed["timings_s"] = {"total_s": 999.999}
	from _receipt import determinism_block  # noqa: E402  (tools/ is on sys.path)
	again = determinism_block(
		{k: v for k, v in perturbed.items() if k != "determinism"},
		nondeterministic_paths=det["nondeterministic_paths"],
		solver_note=det["solver_reproducibility"], sig_figs=det["core_sig_figs"])
	base = determinism_block(
		{k: v for k, v in a_rec.items() if k != "determinism"},
		nondeterministic_paths=det["nondeterministic_paths"],
		solver_note=det["solver_reproducibility"], sig_figs=det["core_sig_figs"])
	eq(again["core_digest"], base["core_digest"],
	   "core_digest must ignore the declared wall-clock paths entirely")


# --------------------------------------------------------------------------
# driver
# --------------------------------------------------------------------------

def main():
	width = max(len(n) for n, _, _ in _results)
	passed = failed = skipped = 0
	failures = []
	for name, doc_ref, fn in _results:
		try:
			fn()
		except Skip as exc:
			skipped += 1
			print(f"SKIP  {name.ljust(width)}  ({exc})")
			continue
		except AssertionError as exc:
			failed += 1
			failures.append((name, doc_ref, str(exc)))
			print(f"FAIL  {name.ljust(width)}")
			continue
		except Exception as exc:  # a doc example that no longer even RUNS is a failure
			failed += 1
			failures.append((name, doc_ref, f"{type(exc).__name__}: {exc}"))
			print(f"FAIL  {name.ljust(width)}")
			continue
		passed += 1
		if VERBOSE:
			print(f"ok    {name.ljust(width)}  [{doc_ref}]")

	print(f"\n{passed} passed, {failed} failed, {skipped} skipped "
	      f"({len(_results)} doc contracts checked against the binary and tools)")
	if failures:
		print("\nA doc contract no longer holds. The binary/tools are the authority — "
		      "fix the DOC section named below (or the regression, if it is one):\n")
		for name, doc_ref, msg in failures:
			print(f"  * {name}\n      doc: {doc_ref}\n      {msg}\n")
		return 1
	return 0


if __name__ == "__main__":
	sys.exit(main())
