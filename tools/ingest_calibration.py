#!/usr/bin/env python3
"""ingest_calibration.py — turn caliper measurements of the CALIBRATE-FDM
coupons into a measured printer profile (`profiles/<printer_name>.json`, the
`kernel_model::process::FdmProfile` schema).

Usage:
	python3 tools/ingest_calibration.py <measurements.json> [--out DIR]
	python3 tools/ingest_calibration.py --self-test
	python3 tools/ingest_calibration.py --print-nominals

Input: `measurements.json` per the schema written by the calibrate_fdm
example (`calibration_system/fdm_coupons/measurements.example.json` is the
placeholder-marked template; every placeholder must be replaced or this tool
refuses). Coupon nominal dimensions are EMBEDDED below (coupon set v1) and
pinned against `kernel_model::process::coupons` by `tests/process.rs`; when a
`coupon_nominals.json` sits next to the measurements file it is cross-checked
and any mismatch refuses loudly.

Sign conventions (all mm, stated in the output too):
- hole_diameter_comp / bore_comp: DIAMETRAL, `nominal − measured`, positive =
  holes print undersized ⇒ ADD the comp to a designed hole/bore diameter.
- xy_clearance_tight / xy_clearance_free: RADIAL true gap at the first
  `press` / first `free` fit-ladder bore, computed from MEASURED geometry:
  `(measured_bore − pin_d_max) / 2`, where `measured_bore` interpolates the
  hole ladder's deviation at that bore's diameter and `pin_d_max` is the
  caliper max across the seam. Positive = gap; a small negative tight value
  = light interference press.
- first_layer_comp: RADIAL elephant-foot flare `(Ø_first_layer − Ø_mid)/2`,
  clamped at 0 (a negative flare means the slicer over-compensates; budget 0
  and a warning is printed).
- seam_allowance: RADIAL seam bump `pin Ø_max − Ø_min` (the bump sits on one
  side, so it enters a diametral reading once).
- max_bridge: longest ladder span with sag ≤ 0.5 mm.
- min_wall: thinnest ladder fin classified `solid`.
- max_unsupported_angle: steepest fan angle classified `clean` (capped at
  the coupon's 60° — never extrapolated).
- z_clearance: NOT measured by coupon set v1 — the conservative default
  0.30 is carried forward (a note is printed).

Refusals (exit 1, `{"ok": false, "errors": [...]}` on stdout): missing or
extra keys, placeholders, non-numeric values, a hole deviating > 1 mm from
nominal (typo class), non-monotone fit/wall/overhang ladders, a fit ladder
that never reaches `press`+`free`, no solid wall, no clean overhang,
negative sag, or a nominals-file mismatch.

`--self-test`: runs a synthetic PERFECT-printer measurement set through the
full pipeline and asserts every compensation lands at exactly 0 (and the
ladder-derived fields at their designed values) — the round-trip gate pinned
by `tests/process.rs`. No files are written.
"""

import json
import math
import os
import sys

# ---- coupon set v1 nominals (mm) — pinned against kernel_model::process::coupons
NOMINALS = {
	"coupons_version": 1,
	"holes_d": [3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5, 8.0],
	"fit_pin_d": 6.0,
	"fit_bores_d": [6.0, 6.1, 6.2, 6.3, 6.4, 6.5, 6.6],
	"bore_large_d": 22.0,
	"disc_d": 20.0,
	"bridge_spans": [5.0, 10.0, 15.0, 20.0, 25.0],
	"walls_t": [0.8, 1.2, 1.6, 2.0, 2.4],
	"overhang_deg": [35.0, 40.0, 50.0, 55.0, 60.0],
}

BRIDGE_SAG_OK = 0.5  # mm — > 2.5 layers of droop at 0.2 mm is not a usable ceiling
MAX_HOLE_DEV = 1.0  # mm — a measured hole further off than this is a typo, not a printer
Z_CLEARANCE_DEFAULT = 0.30  # respool CEIL_CLR — carried forward, not measured (v1)

FIT_CLASSES = ("no_go", "press", "free")
WALL_CLASSES = ("gaps", "solid")
OVERHANG_CLASSES = ("fail", "rough", "clean")

# FdmProfile field order — must match the Rust struct declaration order so a
# python-written profile and a Rust-saved one have the same shape.
PROFILE_FIELDS = [
	"name", "xy_clearance_tight", "xy_clearance_free", "z_clearance",
	"hole_diameter_comp", "bore_comp", "first_layer_comp", "seam_allowance",
	"max_bridge", "max_unsupported_angle", "min_wall", "bed_x", "bed_y", "bed_z",
]

REQUIRED_KEYS = {
	"printer_name", "material", "nozzle_mm", "layer_mm", "bed_mm",
	"coupons_version", "holes", "fit", "bore_22", "pin", "disc",
	"bridge_sag", "walls", "overhang",
}


def g(x):
	"""Trailing-zero-free decimal key ('3', '3.5', '6.1') — matches the Rust
	side's fmt_g."""
	return "%g" % x


def fail(errors):
	print(json.dumps({"ok": False, "errors": errors}, indent=2))
	sys.exit(1)


def check_measurements(m):
	"""Validate the measurement set against the schema; return a list of
	errors (empty = clean)."""
	errors = []
	keys = set(m.keys())
	for k in sorted(REQUIRED_KEYS - keys):
		errors.append(f"missing required key: {k}")
	for k in sorted(keys - REQUIRED_KEYS):
		errors.append(f"unknown key: {k} (schema is fixed — see measurements.example.json)")
	if errors:
		return errors

	def num(path, val, lo, hi):
		if not isinstance(val, (int, float)) or isinstance(val, bool) or not math.isfinite(val) or not (lo <= val <= hi):
			errors.append(f"{path}: expected a number in [{lo}, {hi}], got {val!r}")
			return None
		return float(val)

	def cls(path, val, allowed):
		if not isinstance(val, str) or val not in allowed:
			errors.append(f"{path}: expected one of {list(allowed)}, got {val!r}")
			return None
		return val

	name = m["printer_name"]
	if not isinstance(name, str) or not name.strip() or "PLACEHOLDER" in name:
		errors.append(f"printer_name {name!r} is empty or still a placeholder — name the measured printer")
	if m["coupons_version"] != NOMINALS["coupons_version"]:
		errors.append(
			f"coupons_version {m['coupons_version']!r} != {NOMINALS['coupons_version']} — these measurements are for a different coupon set"
		)
	num("nozzle_mm", m["nozzle_mm"], 0.1, 2.0)
	num("layer_mm", m["layer_mm"], 0.02, 1.0)
	if not (isinstance(m["bed_mm"], list) and len(m["bed_mm"]) == 3):
		errors.append(f"bed_mm: expected [x, y, z] mm, got {m['bed_mm']!r}")
	else:
		for i, b in enumerate(m["bed_mm"]):
			num(f"bed_mm[{i}]", b, 10.0, 2000.0)

	def ladder(path, obj, nominals, checker):
		if not isinstance(obj, dict):
			errors.append(f"{path}: expected an object keyed by nominal value")
			return
		want = {g(n) for n in nominals}
		got = set(obj.keys())
		for k in sorted(want - got):
			errors.append(f"{path}: missing entry for nominal {k}")
		for k in sorted(got - want):
			errors.append(f"{path}: unknown nominal {k} (coupon set v1 has {sorted(want)})")
		for n in nominals:
			k = g(n)
			if k in obj:
				checker(f"{path}[{k}]", n, obj[k])

	ladder("holes", m["holes"], NOMINALS["holes_d"],
		lambda p, n, v: (num(p, v, 0.5, 20.0) is not None
			and abs(n - v) > MAX_HOLE_DEV
			and errors.append(f"{p}: measured {v} deviates {abs(n - v):.2f} mm from nominal {g(n)} — typo class (> {MAX_HOLE_DEV})")))
	ladder("fit", m["fit"], NOMINALS["fit_bores_d"], lambda p, n, v: cls(p, v, FIT_CLASSES))
	num("bore_22", m["bore_22"], 15.0, 30.0)
	if isinstance(m["pin"], dict) and set(m["pin"].keys()) == {"d_min", "d_max"}:
		dmin = num("pin.d_min", m["pin"]["d_min"], 3.0, 9.0)
		dmax = num("pin.d_max", m["pin"]["d_max"], 3.0, 9.0)
		if dmin is not None and dmax is not None and dmax < dmin:
			errors.append(f"pin: d_max {dmax} < d_min {dmin} — swapped readings?")
	else:
		errors.append(f"pin: expected {{'d_min': .., 'd_max': ..}}, got {m['pin']!r}")
	if isinstance(m["disc"], dict) and set(m["disc"].keys()) == {"d_mid", "d_first_layer"}:
		dmid = num("disc.d_mid", m["disc"]["d_mid"], 15.0, 25.0)
		num("disc.d_first_layer", m["disc"]["d_first_layer"], 15.0, 25.0)
		if dmid is not None and abs(dmid - NOMINALS["disc_d"]) > 0.5:
			errors.append(
				f"disc.d_mid {dmid} is {abs(dmid - NOMINALS['disc_d']):.2f} mm from the Ø{g(NOMINALS['disc_d'])} nominal — XY scale broken or wrong coupon"
			)
	else:
		errors.append(f"disc: expected {{'d_mid': .., 'd_first_layer': ..}}, got {m['disc']!r}")
	ladder("bridge_sag", m["bridge_sag"], NOMINALS["bridge_spans"], lambda p, n, v: num(p, v, 0.0, 30.0))
	ladder("walls", m["walls"], NOMINALS["walls_t"], lambda p, n, v: cls(p, v, WALL_CLASSES))
	ladder("overhang", m["overhang"], NOMINALS["overhang_deg"], lambda p, n, v: cls(p, v, OVERHANG_CLASSES))
	if errors:
		return errors

	# ---- ladder consistency (monotonicity) -----------------------------------
	fit_ranks = [FIT_CLASSES.index(m["fit"][g(d)]) for d in NOMINALS["fit_bores_d"]]
	if any(b < a for a, b in zip(fit_ranks, fit_ranks[1:])):
		errors.append(
			f"fit ladder is non-monotone ({[m['fit'][g(d)] for d in NOMINALS['fit_bores_d']]}) — a bigger bore cannot fit tighter; re-check"
		)
	if 1 not in fit_ranks and 2 not in fit_ranks:
		errors.append("fit ladder never reaches press or free — the Ø6.0–6.6 ladder did not straddle your printer; check the pin for blobs")
	if 2 not in fit_ranks:
		errors.append("fit ladder never reaches a free fit — cannot state xy_clearance_free")
	wall_solid = [m["walls"][g(t)] == "solid" for t in NOMINALS["walls_t"]]
	if any(a and not b for a, b in zip(wall_solid, wall_solid[1:])):
		errors.append(f"wall ladder is non-monotone ({wall_solid}) — a thicker wall printed worse than a thinner one; re-check")
	if not any(wall_solid):
		errors.append("no wall in the ladder printed solid — cannot state min_wall; check extrusion")
	over_clean = [m["overhang"][g(a)] == "clean" for a in NOMINALS["overhang_deg"]]
	if any(not a and b for a, b in zip(over_clean, over_clean[1:])):
		errors.append(f"overhang ladder is non-monotone ({over_clean}) — a steeper fin printed cleaner; re-check")
	if not any(over_clean):
		errors.append("no overhang fin printed clean — even 35° failing means a printer fault; cannot state max_unsupported_angle")
	return errors


def hole_dev_at(m, d):
	"""Piecewise-linear interpolation of (nominal − measured) over the hole
	ladder, evaluated at diameter d (the fit bores sit inside the ladder)."""
	xs = NOMINALS["holes_d"]
	devs = [n - float(m["holes"][g(n)]) for n in xs]
	if d <= xs[0]:
		return devs[0]
	if d >= xs[-1]:
		return devs[-1]
	for a, b, da, db in zip(xs, xs[1:], devs, devs[1:]):
		if a <= d <= b:
			t = (d - a) / (b - a)
			return da + t * (db - da)
	raise AssertionError("unreachable: ladder covers d")


def derive_profile(m):
	"""Measurements (already validated) → (profile dict, warnings list)."""
	warnings = []
	devs = [n - float(m["holes"][g(n)]) for n in NOMINALS["holes_d"]]
	hole_comp = sum(devs) / len(devs)
	spread = max(devs) - min(devs)
	if spread > 0.3:
		warnings.append(f"hole deviation spread {spread:.3f} mm across the ladder — comp is the mean; expect per-size residuals")
	bore_comp = NOMINALS["bore_large_d"] - float(m["bore_22"])

	pin_max = float(m["pin"]["d_max"])
	seam = pin_max - float(m["pin"]["d_min"])
	flare = (float(m["disc"]["d_first_layer"]) - float(m["disc"]["d_mid"])) / 2.0
	if flare < 0.0:
		warnings.append(f"first layer measured SMALLER than mid ({flare:.3f} mm radial) — slicer over-compensates; budgeting 0")
	first_layer = max(0.0, flare)

	def measured_bore(d):
		return d - hole_dev_at(m, d)

	fit_cls = [m["fit"][g(d)] for d in NOMINALS["fit_bores_d"]]
	first_free = next(d for d, c in zip(NOMINALS["fit_bores_d"], fit_cls) if c == "free")
	xy_free = (measured_bore(first_free) - pin_max) / 2.0
	if "press" in fit_cls:
		first_press = next(d for d, c in zip(NOMINALS["fit_bores_d"], fit_cls) if c == "press")
		xy_tight = (measured_bore(first_press) - pin_max) / 2.0
	else:
		xy_tight = xy_free
		warnings.append("no press fit observed on the ladder — xy_clearance_tight set equal to free (conservative)")

	max_bridge = 0.0
	for span in NOMINALS["bridge_spans"]:
		if float(m["bridge_sag"][g(span)]) <= BRIDGE_SAG_OK:
			max_bridge = max(max_bridge, span)
	if max_bridge == 0.0:
		warnings.append(f"every bridge sagged > {BRIDGE_SAG_OK} mm — max_bridge recorded as 0.0 (bridging unusable as measured)")

	min_wall = min(t for t in NOMINALS["walls_t"] if m["walls"][g(t)] == "solid")
	max_angle = max(a for a in NOMINALS["overhang_deg"] if m["overhang"][g(a)] == "clean")
	if max_angle == NOMINALS["overhang_deg"][-1]:
		warnings.append(f"cleanest fan fin is the coupon's steepest ({g(max_angle)}°) — true limit may be higher; NOT extrapolated")
	warnings.append(f"z_clearance not measured by coupon set v1 — conservative default {Z_CLEARANCE_DEFAULT} carried forward")

	profile = {
		"name": m["printer_name"],
		"xy_clearance_tight": round(xy_tight, 4),
		"xy_clearance_free": round(xy_free, 4),
		"z_clearance": Z_CLEARANCE_DEFAULT,
		"hole_diameter_comp": round(hole_comp, 4),
		"bore_comp": round(bore_comp, 4),
		"first_layer_comp": round(first_layer, 4),
		"seam_allowance": round(seam, 4),
		"max_bridge": max_bridge,
		"max_unsupported_angle": max_angle,
		"min_wall": min_wall,
		"bed_x": float(m["bed_mm"][0]),
		"bed_y": float(m["bed_mm"][1]),
		"bed_z": float(m["bed_mm"][2]),
	}
	# Mirror FdmProfile::validate's ranges so a file we write always loads.
	post = []
	if not (-0.2 <= profile["xy_clearance_tight"] <= 2.0):
		post.append(f"derived xy_clearance_tight {profile['xy_clearance_tight']} outside [-0.2, 2.0] — measurements inconsistent")
	if not (0.0 <= profile["xy_clearance_free"] <= 2.0):
		post.append(f"derived xy_clearance_free {profile['xy_clearance_free']} outside [0, 2.0] — measurements inconsistent")
	if profile["xy_clearance_tight"] > profile["xy_clearance_free"]:
		post.append("derived tight clearance exceeds free clearance — fit ladder vs pin measurements inconsistent")
	for f, lo, hi in (("hole_diameter_comp", -1.0, 1.0), ("bore_comp", -1.0, 1.0), ("seam_allowance", 0.0, 1.0), ("first_layer_comp", 0.0, 2.0)):
		if not (lo <= profile[f] <= hi):
			post.append(f"derived {f} {profile[f]} outside [{lo}, {hi}] — measurements inconsistent")
	if post:
		fail(post)
	return profile, warnings


def write_profile(profile, out_dir):
	os.makedirs(out_dir, exist_ok=True)
	ordered = {k: profile[k] for k in PROFILE_FIELDS}
	path = os.path.join(out_dir, f"{profile['name']}.json")
	with open(path, "w") as f:
		f.write(json.dumps(ordered, indent=2) + "\n")
	return path


def cross_check_nominals(meas_path):
	side = os.path.join(os.path.dirname(os.path.abspath(meas_path)), "coupon_nominals.json")
	if not os.path.exists(side):
		return [f"note: {side} not found — using embedded coupon set v1 nominals"]
	with open(side) as f:
		disk = json.load(f)
	if disk != NOMINALS:
		fail([
			f"coupon_nominals.json at {side} does not match this tool's embedded coupon set v1 — "
			"the printed coupons and this ingest disagree; regenerate with `cargo run --release -p kernel-model --example calibrate_fdm` "
			"(the example is parked in legacy/kernel-model-examples/ — restore it per that folder's README first) "
			"and use the matching tool version"
		])
	return []


def self_test():
	"""Synthetic PERFECT printer through the full pipeline: every measured
	value exactly nominal ⇒ every compensation must land at exactly 0 and the
	ladder fields at their designed values."""
	m = {
		"printer_name": "self_test_perfect",
		"material": "PLA",
		"nozzle_mm": 0.4,
		"layer_mm": 0.2,
		"bed_mm": [256.0, 256.0, 256.0],
		"coupons_version": 1,
		"holes": {g(d): d for d in NOMINALS["holes_d"]},
		# zero true clearance = snug press; ≥ 0.05 radial slides free
		"fit": {g(d): ("press" if d == 6.0 else "free") for d in NOMINALS["fit_bores_d"]},
		"bore_22": 22.0,
		"pin": {"d_min": 6.0, "d_max": 6.0},
		"disc": {"d_mid": 20.0, "d_first_layer": 20.0},
		"bridge_sag": {g(s): 0.0 for s in NOMINALS["bridge_spans"]},
		"walls": {g(t): "solid" for t in NOMINALS["walls_t"]},
		"overhang": {g(a): "clean" for a in NOMINALS["overhang_deg"]},
	}
	errors = check_measurements(m)
	if errors:
		fail(["self-test: perfect measurements failed validation"] + errors)
	profile, _warnings = derive_profile(m)
	expected = {
		"name": "self_test_perfect",
		"xy_clearance_tight": 0.0,  # perfect Ø6.0 bore on a perfect Ø6.0 pin
		"xy_clearance_free": 0.05,  # first free bore Ø6.1 ⇒ 0.05 radial
		"z_clearance": Z_CLEARANCE_DEFAULT,
		"hole_diameter_comp": 0.0,
		"bore_comp": 0.0,
		"first_layer_comp": 0.0,
		"seam_allowance": 0.0,
		"max_bridge": 25.0,
		"max_unsupported_angle": 60.0,
		"min_wall": 0.8,
		"bed_x": 256.0,
		"bed_y": 256.0,
		"bed_z": 256.0,
	}
	bad = []
	for k, want in expected.items():
		got = profile[k]
		if isinstance(want, float) or isinstance(want, int):
			if abs(float(got) - float(want)) > 1e-9:
				bad.append(f"{k}: got {got}, want {want}")
		elif got != want:
			bad.append(f"{k}: got {got!r}, want {want!r}")
	if bad:
		fail(["self-test FAILED — a perfect printer must produce zero compensations:"] + bad)
	print(json.dumps({"ok": True, "self_test": "PASS", "profile": {k: profile[k] for k in PROFILE_FIELDS}}, indent=2))
	sys.exit(0)


def main(argv):
	if "--self-test" in argv:
		self_test()
	if "--print-nominals" in argv:
		print(json.dumps(NOMINALS, indent=2))
		sys.exit(0)
	args = [a for a in argv if not a.startswith("--")]
	out_dir = "profiles"
	if "--out" in argv:
		i = argv.index("--out")
		if i + 1 >= len(argv):
			fail(["--out needs a directory argument"])
		out_dir = argv[i + 1]
		if out_dir in args:
			args.remove(out_dir)
	if len(args) != 1:
		fail(["usage: ingest_calibration.py <measurements.json> [--out DIR] | --self-test | --print-nominals"])
	meas_path = args[0]
	try:
		with open(meas_path) as f:
			m = json.load(f)
	except (OSError, json.JSONDecodeError) as e:
		fail([f"cannot read measurements at {meas_path}: {e}"])
	if not isinstance(m, dict):
		fail([f"{meas_path}: expected a JSON object"])
	notes = cross_check_nominals(meas_path)
	errors = check_measurements(m)
	if errors:
		fail(errors)
	profile, warnings = derive_profile(m)
	path = write_profile(profile, out_dir)
	print(json.dumps({
		"ok": True,
		"profile_path": path,
		"profile": {k: profile[k] for k in PROFILE_FIELDS},
		"sign_conventions": {
			"hole_diameter_comp/bore_comp": "diametral, nominal − measured; positive = printed undersized, ADD to designed diameter",
			"xy_clearance_*": "radial true gap (measured bore − pin d_max)/2 at the first press/free bore; negative tight = light interference",
			"first_layer_comp": "radial elephant-foot flare, clamped ≥ 0",
			"seam_allowance": "radial seam bump (pin d_max − d_min), budgeted once per fit interface",
		},
		"warnings": warnings + notes,
	}, indent=2))
	sys.exit(0)


if __name__ == "__main__":
	main(sys.argv[1:])
