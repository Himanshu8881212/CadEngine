#!/usr/bin/env python3
"""Design-intent check over the OFFICIAL `kernel-api asm` report (gearbox).

Replaces the retired `tools/asmcheck` Rust workaround harness (FRICTION #1/#2,
both resolved-w6): the kernel side — load, mate residual, BOM, per-instance and
per-state exports, and the B-rep-aware contact scan — is now the CLI's job and
is gated by its exit code. What remains *gearbox-specific* is the design-intent
allowlist below: every touching pair the scan finds must be a DESIGNED contact
(fit/seat/butt), and the designed-contact count and the must-clear flank gaps
are pinned as regressions.

Handles BOTH assembly variants: the flat gearbox.lmcasm and the nested
gearbox_nested.lmcasm (same 37 parts regrouped into three asm/shaft_*.lmcasm
shaft stacks). Nested reports name sub-assembly members hierarchically
("stack_in/g1p"); the allowlist classifies by the LEAF name — member names are
globally unique, so this is a bijection back onto the flat names and both
reports are held to the identical 52-designed-contact standard. A nested
report additionally has its BOM v2 payload checked (tree rollup, meta-derived
masses) by check_bom_v2 below.

usage: check_asm.py out/asm_report.json    (exit 0 iff design intent holds)
"""

import json
import math
import sys

# Every touching pair must belong to one of these design-intent classes.
# The whole drivetrain of one axis is a contacting stack (shaft/gears/keys/
# spacers/bearing inner races):
SHAFT_OF = {
	"shaft_in": "in", "g1p": "in", "608_in_A": "in", "608_in_B": "in",
	"spacer9_0": "in", "spacer23_0": "in", "key8_0": "in", "key10_0": "in",
	"shaft_mid": "mid", "g1w": "mid", "g2p": "mid", "608_mid_A": "mid",
	"608_mid_B": "mid", "spacer10_0": "mid", "spacer10_1": "mid",
	"key8_2": "mid", "key12_0": "mid",
	"shaft_out": "out", "g2w": "out", "608_out_A": "out", "608_out_B": "out",
	"spacer21_0": "out", "spacer11_0": "out", "key8_1": "out", "key10_1": "out",
}

# The designed-contact count through the official scan (was 52 via the retired
# asmcheck harness; the kernel rebuild is deterministic, so this is exact).
EXPECTED_DESIGNED_CONTACTS = 52

# KNOWN TESSELLATION ARTIFACTS (FRICTION.md #19) — phantom 0.0-distance pairs:
# the housing base's exact adaptive tessellation is currently LEAKY (post-Wave-5
# triangulator) and its crack triangles cross two corner M4-insert pockets and
# three web bores, so the mesh-distance scan reads "touching" where the EXACT
# boolean proves clear air (bolt↔pilot 0.8 mm radial, spacer↔web bore 2 mm).
# Each pair here is re-proven disjoint EVERY RUN by `check_artifacts.json`
# (pose → union with base → assert shells == 2, all exact); run_all.sh executes
# it right before this script. Tolerated-if-present, never required — when the
# triangulator is fixed these simply stop appearing.
KNOWN_TESSELLATION_ARTIFACTS = {
	frozenset(p) for p in [
		("base", "bolt_0"), ("base", "bolt_3"),
		("base", "spacer9_0"), ("base", "spacer10_0"), ("base", "spacer21_0"),
	]
}

# Pairs that must NOT touch (gear flanks carry designed backlash; rotating
# parts clear the housing). They may appear in the proximity window with a
# positive distance — the tightest is the designed 0.05 mm flank gap.
MUST_CLEAR = [
	("g1p", "g1w"), ("g2p", "g2w"), ("g1w", "g2w"),
	("g1w", "shaft_in"), ("g2w", "shaft_mid"),
	("g1p", "base"), ("g1w", "base"), ("g2p", "base"), ("g2w", "base"),
	("g1w", "lid"), ("g2w", "lid"),
	("shaft_in", "base"), ("shaft_mid", "base"), ("shaft_out", "base"),
]


def leaf(name):
	"""The leaf instance name: nested reports prefix sub-assembly members
	("stack_in/g1p" -> "g1p"); flat names pass through unchanged."""
	return name.split("/")[-1]


def designed_contact(a, b):
	"""Is the instance pair (a, b) a DESIGNED contact (fit/seat/butt)?"""
	a, b = leaf(a), leaf(b)
	sa, sb = SHAFT_OF.get(a), SHAFT_OF.get(b)
	if sa is not None and sa == sb:
		return True
	# bearings seat in the housing pockets
	if (a.startswith("608_") and b == "base") or (b.startswith("608_") and a == "base"):
		return True
	# lid sits on the base seal face; dowels press/slip in both; screws seat in the lid
	if {a, b} == {"lid", "base"}:
		return True
	if (a.startswith("dowel_") and b in ("base", "lid")) or (b.startswith("dowel_") and a in ("base", "lid")):
		return True
	if (a.startswith("bolt_") and b == "lid") or (b.startswith("bolt_") and a == "lid"):
		return True
	return False


# --- BOM v2 expectations for the NESTED report (gearbox_nested.lmcasm) ----------
# generate.py is the single source of truth (META blocks + STACKS grouping);
# these pin its output. unit_mass_g = density x exact engine volume: bearing and
# key are re-derived from closed forms here (envelope ring / key block — both
# exact B-reps; the ring's tetra fan sums in f64, hence the 1e-9 slack). The
# shaft volumes carry keyway/circlip boolean cuts, so their masses are asserted
# present / positive / volume_source "exact" rather than re-derived.
EXPECTED_STACKS = {"stack_in": 8, "stack_mid": 9, "stack_out": 8}
EXPECTED_LEAF_PARTS = 37
BEARING_UNIT_MASS = 7.85 * math.pi * (11.0 ** 2 - 4.0 ** 2) * 7.0 / 1000.0  # 18.126 g (envelope ring — overstates a real 608ZZ, see README)
KEY8_UNIT_MASS = 8.4 * (8.0 * 2.0 * 2.0) / 1000.0                           # 0.2688 g
METAL_LINES = {  # flat-line name -> (part_number, material, make_or_buy, count)
	"shaft_input": ("GBX-SH-IN", "steel", "make", 1),
	"shaft_intermediate": ("GBX-SH-MID", "steel", "make", 1),
	"shaft_output": ("GBX-SH-OUT", "steel", "make", 1),
	"bearing_608": ("608ZZ", "steel", "buy", 6),
	"key_2x2_8": ("DIN6885B-2x2x8", "brass", "buy", 3),
}


def check_bom_v2(report):
	"""Nested-report BOM v2 design intent: schema "bom/2", the three-stack tree
	rollup (8/9/8), flat and tree both totalling the same 37 leaf parts, and the
	five meta-stamped metal lines carrying part number / material / sourcing and
	mass = density x exact volume. Returns a list of problem strings."""
	problems = []
	bom = next((op for op in report["ops"] if op["id"] == "bom"), None)
	if bom is None or not bom.get("ok"):
		return ["no successful 'bom' entry in the asm report"]
	m = bom["measures"]
	if m.get("schema") != "bom/2":
		problems.append(f"bom schema is {m.get('schema')!r}, want 'bom/2'")
	tree, flat = m["tree"], m["flat"]
	branches = {n["instance"]: n["count"] for n in tree if n.get("children")}
	if branches != EXPECTED_STACKS:
		problems.append(f"sub-assembly branches {branches} != expected {EXPECTED_STACKS}")
	for n in tree:
		if n.get("children") and n["count"] != sum(c["count"] for c in n["children"]):
			problems.append(f"tree rollup broken at {n['instance']}: {n['count']} != sum(children)")
	flat_total = sum(l["count"] for l in flat)
	tree_total = sum(n["count"] for n in tree)
	if not (flat_total == tree_total == EXPECTED_LEAF_PARTS):
		problems.append(f"part totals: flat {flat_total} / tree {tree_total}, want {EXPECTED_LEAF_PARTS} both")
	lines = {l["name"]: l for l in flat}
	for name, (pn, mat, mob, count) in METAL_LINES.items():
		l = lines.get(name)
		if l is None:
			problems.append(f"no flat BOM line for {name}")
			continue
		got = (l.get("part_number"), (l.get("material") or {}).get("name"),
		       l.get("make_or_buy"), l["count"], l.get("volume_source"))
		want = (pn, mat, mob, count, "exact")
		if got != want:
			problems.append(f"{name}: (part_number, material, sourcing, count, volume_source) = {got}, want {want}")
		unit, line = l.get("unit_mass_g"), l.get("line_mass_g")
		if not (unit and unit > 0.0 and line is not None and abs(line - unit * count) < 1e-9):
			problems.append(f"{name}: unit_mass_g={unit} line_mass_g={line} (want line = unit x {count})")
	for name, want in [("bearing_608", BEARING_UNIT_MASS), ("key_2x2_8", KEY8_UNIT_MASS)]:
		got = lines.get(name, {}).get("unit_mass_g")
		if got is None or abs(got - want) > 1e-9:
			problems.append(f"{name}: unit_mass_g={got}, closed form says {want}")
	if not problems:
		masses = {n: round(lines[n]["unit_mass_g"], 3) for n in METAL_LINES}
		print(f"BOM v2: tree branches {branches}, {flat_total} leaf parts, unit masses (g) {masses}")
	return problems


def main():
	if len(sys.argv) != 2:
		print(__doc__.strip(), file=sys.stderr)
		return 2
	with open(sys.argv[1]) as f:
		report = json.load(f)
	contacts = next((op for op in report["ops"] if op["id"] == "contacts"), None)
	if contacts is None or not contacts.get("ok"):
		print("FAIL: no successful 'contacts' entry in the asm report")
		return 1
	pairs = contacts["measures"]["pairs"]
	touching = [(p["a"], p["b"]) for p in pairs if p["touching"]]
	gaps = {(leaf(p["a"]), leaf(p["b"])): p["distance"] for p in pairs}

	artifacts = sorted(p for p in touching if frozenset(map(leaf, p)) in KNOWN_TESSELLATION_ARTIFACTS)
	unexpected = sorted(p for p in touching if not designed_contact(*p) and frozenset(map(leaf, p)) not in KNOWN_TESSELLATION_ARTIFACTS)
	designed = sorted(p for p in touching if designed_contact(*p))
	clear_violations = []
	tightest = (None, float("inf"))
	for a, b in MUST_CLEAR:
		d = gaps.get((a, b), gaps.get((b, a)))
		if d is not None and d <= 1e-6:
			clear_violations.append((a, b))
		elif d is not None and d < tightest[1]:
			tightest = ((a, b), d)

	print(f"designed contacts: {len(designed)} (expected {EXPECTED_DESIGNED_CONTACTS})")
	if tightest[0] is not None:
		print(f"tightest must-clear gap: {tightest[1]:.3f} mm ({tightest[0][0]} <-> {tightest[0][1]})")
	for p in artifacts:
		print(f"known tessellation artifact (FRICTION #19, exact-proven disjoint by check_artifacts.json): {p[0]} <-> {p[1]}")
	for p in unexpected:
		print(f"UNEXPECTED CONTACT: {p[0]} <-> {p[1]}")
	for p in clear_violations:
		print(f"MUST-CLEAR PAIR TOUCHES: {p[0]} <-> {p[1]}")

	# Nested reports (top_level < leaf instances) also carry the BOM v2 intent.
	load = next((op for op in report["ops"] if op["id"] == "load"), None)
	lm = (load or {}).get("measures") or {}
	nested = lm.get("top_level", lm.get("instances")) != lm.get("instances")
	bom_problems = check_bom_v2(report) if nested else []
	for p in bom_problems:
		print(f"BOM V2 INTENT BROKEN: {p}")

	ok = (not unexpected and not clear_violations and not bom_problems
	      and len(designed) == EXPECTED_DESIGNED_CONTACTS)
	print("check_asm:", "PASS" if ok else "FAIL")
	return 0 if ok else 1


if __name__ == "__main__":
	sys.exit(main())
