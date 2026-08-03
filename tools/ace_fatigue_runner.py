#!/usr/bin/env python3
"""ace_fatigue_runner.py — stress-life (S-N) cyclic damage for PRINTED parts.

The shop's SIXTH permanent solver (registry card: tools/solvers/fatigue.md).
In-house, NumPy only. It is a POST-PROCESSOR: it consumes a static stress field
(normally `stress_field.npy` from tools/ace_fea_runner.py — per-element von
Mises, Pa) or a scalar reference stress, plus a declared cycle spectrum, and
returns Palmgren-Miner damage.

BLUNT VALIDITY STATEMENT (repeated in tools/solvers/fatigue.md and in every
receipt): fatigue of FDM parts is dominated by LAYER ADHESION and process
DEFECTS, not by the bulk polymer. This is a COMPARATIVE / SCREENING tool for
ranking design variants and rejecting overstressed features. It is not a
certification basis. The published 90%/10%-survival band for printed PLA spans
a factor of 3.7x to 90x in LIFE; a single predicted cycle count means little
without that band, so the receipt always carries it.

WHAT IT REFUSES (a refusal is a first-class answer here)
  * a material whose printed S-N data is `insufficient` or `unknown` in
    tools/materials/fatigue.json — it will NOT silently substitute bulk-polymer
    data (printed ABS reaches 6e4 cycles where injection-moulded ABS reaches
    6e6 at the same 10 MPa: a 100x error waiting to happen);
  * `load_orientation: "across_layer"` — no across-layer printed S-N dataset
    exists for any material in the registry, and a STATIC z/xy strength ratio
    is not a fatigue-slope ratio;
  * a mean-stress correction stacked on a curve that already absorbs mean
    stress (double counting);
  * a block whose effective stress exceeds the printed UTS (that is static
    failure, not fatigue).

Usage:  python3 ace_fatigue_runner.py <job.json>

Job JSON:
    out_dir            REQUIRED  directory for damage .npy output
    material           REQUIRED  registry key ("PLA") resolved against
                                 tools/materials/fatigue.json (+ tools/materials.py
                                 for UTS cross-checks), or an INLINE dict
                                 {curve:{a_mpa, b, stress_measure, n_valid?},
                                  sigma_uts_mpa, name?} for synthetic/benchmark
                                 use — inline curves are stamped
                                 `data_provenance: "inline (caller-supplied)"`
    curve              optional  "design" (default, PS >= 90%) | "median" (PS 50%)
    load_orientation   optional  "in_plane" (default) | "across_layer" (REFUSED)
    stress, one of:
      {npy, unit?: "pa"|"mpa", reference_load?: 1.0}   per-element field; each
                                 block's `load_factor` scales it linearly (valid
                                 because the source solve is linear elastic)
      {sigma_ref_mpa} | {sigma_ref_pa}                 single scalar hot spot
    spectrum           REQUIRED  [{name?, cycles, load_factor?, r_ratio?}]  or
                                 [{name?, cycles, sigma_a_mpa, sigma_m_mpa?}]
                                 von Mises is unsigned, so a field-mode block
                                 MUST declare r_ratio (default 0.0 = zero-tension,
                                 the usual printed-part duty cycle) — it cannot
                                 be inferred from the field
    mean_stress        optional  "goodman" | "gerber" | "none" | "intrinsic"
                                 default = whatever the chosen curve requires
    sigma_uts_mpa      optional  override the registry's printed UTS (recorded)
    damage_limit       optional  1.0

Output contract: mirrors ace_thermal_runner / ace_contact_runner — the LAST
non-empty stdout line is ONE JSON receipt; logging to stderr. In field mode
`damage_field.npy` (float32, same shape as the input field) lands in out_dir.
Failure/refusal = {ok:false, error} + **exit 1**.
"""
from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS_DIR))

ANALYZER_VERSION = "ace_fatigue/basquin-miner/v1"
FATIGUE_DB = TOOLS_DIR / "materials" / "fatigue.json"


def log(msg: str) -> None:
	print(msg, file=sys.stderr, flush=True)


def emit(payload: dict) -> None:
	print(json.dumps(payload), flush=True)


class JobError(ValueError):
	"""A manifest refusal with a user-actionable message."""


class DataRefusal(ValueError):
	"""The requested material/orientation has no credible printed S-N data.

	A distinct type because it is NOT a manifest mistake — it is the honest
	answer, and the receipt names the material, the status and the reason."""


# ---------------------------------------------------------------------------
# Curve model: Basquin  sigma = a * N^b   ->   N = (sigma / a)^(1/b)
# ---------------------------------------------------------------------------
def basquin_life(sigma_mpa, a_mpa: float, b: float):
	"""Cycles to failure at effective stress `sigma_mpa`. sigma <= 0 -> inf."""
	import numpy as np

	s = np.asarray(sigma_mpa, dtype=np.float64)
	out = np.full(s.shape, np.inf)
	pos = s > 0.0
	out[pos] = (s[pos] / a_mpa) ** (1.0 / b)
	return out


def basquin_fit(cycles, sigma_a_mpa):
	"""Least-squares Basquin fit in log-log space -> (a_mpa, b, r2).

	log(sigma) = log(a) + b log(N) is linear, so an ordinary least-squares line
	through (log N, log sigma) IS the Basquin fit — the round-trip gate pins
	that it recovers planted coefficients to machine precision."""
	import numpy as np

	N = np.asarray(cycles, dtype=np.float64)
	S = np.asarray(sigma_a_mpa, dtype=np.float64)
	if N.shape != S.shape or N.ndim != 1 or N.size < 2:
		raise JobError("basquin_fit needs two matching 1-D arrays with >= 2 points")
	if not ((N > 0).all() and (S > 0).all()):
		raise JobError("basquin_fit needs strictly positive cycles and stresses")
	x, y = np.log(N), np.log(S)
	b, log_a = np.polyfit(x, y, 1)
	pred = log_a + b * x
	ss_res = float(np.sum((y - pred) ** 2))
	ss_tot = float(np.sum((y - y.mean()) ** 2))
	r2 = 1.0 - ss_res / ss_tot if ss_tot > 0.0 else 1.0
	return float(math.exp(log_a)), float(b), r2


# ---------------------------------------------------------------------------
# Mean-stress corrections
# ---------------------------------------------------------------------------
def mean_stress_correct(model: str, sigma_a, sigma_m, sigma_uts: float):
	"""Equivalent fully-reversed amplitude (or max stress for `intrinsic`).

	goodman   sigma_ar = sigma_a / (1 - sigma_m/sigma_u)      (sigma_m > 0)
	gerber    sigma_ar = sigma_a / (1 - (sigma_m/sigma_u)^2)  (sigma_m > 0)
	none      sigma_ar = sigma_a
	intrinsic sigma_eff = sigma_m + sigma_a = sigma_max

	Compressive mean stress is deliberately NOT given credit (sigma_ar =
	sigma_a for sigma_m <= 0): the beneficial branch is unvalidated for printed
	polymers and taking it would be unconservative."""
	import numpy as np

	a = np.asarray(sigma_a, dtype=np.float64)
	m = np.asarray(sigma_m, dtype=np.float64)
	if model == "none":
		return a.copy()
	if model == "intrinsic":
		return m + a
	ratio = np.where(m > 0.0, m / sigma_uts, 0.0)
	if model == "goodman":
		denom = 1.0 - ratio
	elif model == "gerber":
		denom = 1.0 - ratio ** 2
	else:
		raise JobError(f"mean_stress must be goodman|gerber|none|intrinsic, got {model!r}")
	if np.any(denom <= 0.0):
		raise JobError(
			f"mean stress reaches or exceeds the ultimate strength "
			f"(sigma_m/sigma_uts >= 1 with sigma_uts = {sigma_uts} MPa) — that is "
			f"static failure, not fatigue; the {model} correction is undefined there")
	return a / denom


# ---------------------------------------------------------------------------
# Material / curve resolution
# ---------------------------------------------------------------------------
def load_fatigue_db() -> dict:
	if not FATIGUE_DB.exists():
		raise JobError(f"fatigue registry missing: {FATIGUE_DB}")
	return json.loads(FATIGUE_DB.read_text(encoding="utf-8"))


def life_scatter_band(entry: dict):
	"""(min, max) LIFE factor between the 10% and 90% survival curves.

	Recomputed from the stored source table every run (T_life = T_sigma^k) so
	the quoted confidence band can never drift away from the source data."""
	tab = (entry.get("source_table") or {}).get("rows")
	if not tab:
		return None
	factors = [row[6] ** row[4] for row in tab]
	return (min(factors), max(factors))


def resolve_curve(job: dict):
	"""-> (curve dict, meta dict). Raises DataRefusal when data is absent."""
	spec = job.get("material")
	which = job.get("curve", "design")
	if isinstance(spec, dict):
		curve = dict(spec.get("curve") or {})
		for key in ("a_mpa", "b"):
			if key not in curve:
				raise JobError(f"inline material.curve needs {key}")
		curve.setdefault("stress_measure", "amplitude")
		curve.setdefault("mean_stress_handling",
		                 "intrinsic" if curve["stress_measure"] == "max" else "correction_required")
		curve.setdefault("n_valid", None)
		meta = {
			"name": spec.get("name", "inline"),
			"status": "inline",
			"confidence": spec.get("confidence", "caller-supplied (NOT from the registry)"),
			"sigma_uts_mpa": spec.get("sigma_uts_mpa"),
			"data_provenance": "inline (caller-supplied)",
			"curve_name": spec.get("curve_name", "inline"),
		}
		return curve, meta, None

	if not isinstance(spec, str):
		raise JobError(f"material must be a registry key string or an inline dict, got "
		               f"{type(spec).__name__}")
	db = load_fatigue_db()
	key = None
	for k in db["materials"]:
		if k.lower() == spec.lower():
			key = k
			break
	if key is None:
		raise DataRefusal(
			f"material '{spec}' is not in the printed-fatigue registry "
			f"({FATIGUE_DB.name}; known: {sorted(db['materials'])}). No life number is "
			f"produced for an unregistered material — add a researched entry with "
			f"sources first.")
	entry = db["materials"][key]
	status = entry.get("status")
	if status != "measured":
		raise DataRefusal(
			f"REFUSING to estimate fatigue life for '{key}': printed S-N data status is "
			f"'{status}'. {entry.get('refusal_reason', '')} "
			f"(registry: {FATIGUE_DB.name}; policy: a non-'measured' material gets no "
			f"life number, and bulk-polymer data is never substituted). "
			f"{('What would change this: ' + entry['what_would_change_this']) if entry.get('what_would_change_this') else ''}")
	curves = entry.get("curves") or {}
	if which not in curves:
		raise JobError(f"curve '{which}' not defined for {key}; available: {sorted(curves)}")
	curve = dict(curves[which])
	# The stored a_mpa/b are DERIVED from the primitives (k, sigma_at_n_ref,
	# n_ref). Re-derive and refuse on drift, so a hand edit of one without the
	# other cannot silently change every answer.
	if all(curve.get(x) is not None for x in ("k", "n_ref", "sigma_at_n_ref_mpa")):
		b_chk = -1.0 / float(curve["k"])
		a_chk = float(curve["sigma_at_n_ref_mpa"]) * float(curve["n_ref"]) ** (-b_chk)
		if abs(b_chk - float(curve["b"])) > 1e-12 * abs(b_chk) or \
		   abs(a_chk - float(curve["a_mpa"])) > 1e-9 * a_chk:
			raise JobError(
				f"{FATIGUE_DB.name} [{key}.curves.{which}] is internally inconsistent: "
				f"stored (a_mpa={curve['a_mpa']}, b={curve['b']}) but the primitives "
				f"(k={curve['k']}, sigma_at_n_ref={curve['sigma_at_n_ref_mpa']} MPa at "
				f"N={curve['n_ref']}) derive (a_mpa={a_chk!r}, b={b_chk!r}) — fix the record")
	meta = {
		"name": key, "status": status,
		"confidence": entry.get("confidence"),
		"confidence_basis": entry.get("confidence_basis"),
		"sigma_uts_mpa": entry.get("sigma_uts_mpa_printed"),
		"sigma_uts_source": entry.get("sigma_uts_source"),
		"data_provenance": f"{FATIGUE_DB.name} [{key}.curves.{which}]",
		"curve_name": which,
		"probability_of_survival": curve.get("probability_of_survival"),
		"sources": [s.get("citation") for s in entry.get("sources", [])],
		"source_urls": [s.get("url") for s in entry.get("sources", [])],
		"gaps_unknowns": entry.get("gaps_unknowns", []),
		"conflicts": entry.get("conflicts", []),
		"test_conditions": entry.get("test_conditions", {}),
	}
	return curve, meta, entry


# ---------------------------------------------------------------------------
# Stress input
# ---------------------------------------------------------------------------
def load_stress(job: dict):
	"""-> (sigma_ref_mpa array-or-scalar, mode, receipt)."""
	import numpy as np

	spec = job.get("stress")
	if not isinstance(spec, dict):
		raise JobError("stress block required: {npy,...} or {sigma_ref_mpa} or {sigma_ref_pa}")
	if spec.get("npy"):
		field = np.load(spec["npy"]).astype(np.float64)
		if not np.isfinite(field).all():
			raise JobError(f"stress field {spec['npy']} contains non-finite values")
		if (field < 0.0).any():
			raise JobError("stress field contains negative values — this runner expects an "
			               "unsigned equivalent stress (von Mises), not a signed component")
		unit = str(spec.get("unit", "pa")).lower()
		if unit == "pa":
			field = field * 1e-6
		elif unit != "mpa":
			raise JobError(f"stress.unit must be 'pa' or 'mpa', got {unit!r}")
		ref_load = float(spec.get("reference_load", 1.0))
		if not (np.isfinite(ref_load) and ref_load > 0.0):
			raise JobError(f"stress.reference_load must be finite and > 0, got {ref_load}")
		field = field / ref_load
		rec = {"mode": "field", "npy": str(spec["npy"]), "shape": list(field.shape),
		       "reference_load": ref_load,
		       "max_sigma_ref_mpa": float(field.max()),
		       "note": "field is per-unit reference load; each block scales it by load_factor "
		               "(linear elasticity). von Mises is UNSIGNED, so r_ratio must be declared."}
		return field, "field", rec
	for key, scale in (("sigma_ref_mpa", 1.0), ("sigma_ref_pa", 1e-6)):
		if spec.get(key) is not None:
			v = float(spec[key]) * scale
			if not (np.isfinite(v) and v >= 0.0):
				raise JobError(f"stress.{key} must be finite and >= 0, got {spec[key]!r}")
			return np.array([v]), "scalar", {"mode": "scalar", "sigma_ref_mpa": v}
	raise JobError("stress needs npy, sigma_ref_mpa or sigma_ref_pa")


# ---------------------------------------------------------------------------
# Spectrum -> per-block (sigma_a, sigma_m)
# ---------------------------------------------------------------------------
def block_stresses(blk, bi: int, sigma_ref, mode: str):
	"""-> (sigma_a, sigma_m, receipt fields). Arrays share sigma_ref's shape."""
	import numpy as np

	if blk.get("sigma_a_mpa") is not None:
		sa = np.full(np.shape(sigma_ref), float(blk["sigma_a_mpa"]))
		sm = np.full(np.shape(sigma_ref), float(blk.get("sigma_m_mpa", 0.0)))
		if not (np.isfinite(sa).all() and np.isfinite(sm).all()):
			raise JobError(f"spectrum[{bi}] sigma_a_mpa/sigma_m_mpa must be finite")
		if (sa < 0.0).any():
			raise JobError(f"spectrum[{bi}].sigma_a_mpa must be >= 0")
		return sa, sm, {"source": "explicit sigma_a/sigma_m"}
	lf = float(blk.get("load_factor", 1.0))
	r = float(blk.get("r_ratio", 0.0))
	if not np.isfinite(lf):
		raise JobError(f"spectrum[{bi}].load_factor must be finite, got {lf}")
	if not np.isfinite(r) or r >= 1.0:
		raise JobError(f"spectrum[{bi}].r_ratio must be finite and < 1 "
		               f"(R = sigma_min/sigma_max; R = 1 is a static hold, not a cycle), got {r}")
	smax = np.abs(lf) * np.asarray(sigma_ref, dtype=np.float64)
	sa = smax * (1.0 - r) / 2.0
	sm = smax * (1.0 + r) / 2.0
	return sa, sm, {"source": f"load_factor {lf} x reference field, R = {r}"}


# ---------------------------------------------------------------------------
# Run
# ---------------------------------------------------------------------------
def run(job: dict) -> dict:
	import numpy as np

	out_dir = Path(job["out_dir"])
	out_dir.mkdir(parents=True, exist_ok=True)
	t0 = time.monotonic()

	orient = job.get("load_orientation", "in_plane")
	if orient not in ("in_plane", "across_layer"):
		raise JobError(f"load_orientation must be in_plane|across_layer, got {orient!r}")
	if orient == "across_layer":
		raise DataRefusal(
			"REFUSING: load_orientation = 'across_layer'. Every S-N curve in "
			f"{FATIGUE_DB.name} is measured on IN-PLANE (flat on the build plate) "
			"specimens, and no across-layer printed-polymer S-N dataset was found for any "
			"registry material (2026-07-30). Applying the STATIC z_vs_xy_strength_ratio "
			"knockdown would be an invention: a static strength ratio is not a fatigue "
			"slope ratio, and across-layer failure is a different mechanism (interlayer "
			"debonding) with its own exponent. Redesign so the cyclic principal stress "
			"lies in the layer plane, or measure the curve.")

	curve, meta, entry = resolve_curve(job)
	a_mpa, b = float(curve["a_mpa"]), float(curve["b"])
	if not (np.isfinite(a_mpa) and a_mpa > 0.0):
		raise JobError(f"curve a_mpa must be finite and > 0, got {a_mpa}")
	if not (np.isfinite(b) and b < 0.0):
		raise JobError(f"curve b (Basquin exponent) must be finite and < 0, got {b} — "
		               f"a non-negative exponent means life GROWS with stress")
	measure = curve.get("stress_measure", "amplitude")
	handling = curve.get("mean_stress_handling",
	                     "intrinsic" if measure == "max" else "correction_required")
	model = job.get("mean_stress")
	if model is None:
		model = "intrinsic" if handling == "intrinsic" else "goodman"
	if handling == "intrinsic" and model not in ("intrinsic",):
		raise JobError(
			f"curve '{meta['curve_name']}' is a MAX-STRESS curve: it already absorbs the "
			f"mean-stress effect (mean_stress_handling = 'intrinsic'). Applying "
			f"'{model}' on top double-counts mean stress. Use mean_stress='intrinsic', "
			f"or select the amplitude-based curve (curve: 'median').")
	if handling == "correction_required" and model == "intrinsic":
		raise JobError(
			f"curve '{meta['curve_name']}' is an AMPLITUDE curve on an R = "
			f"{curve.get('r_ratio_basis')} basis; 'intrinsic' would feed it a max stress. "
			f"Use goodman|gerber|none.")

	sigma_uts = job.get("sigma_uts_mpa", meta.get("sigma_uts_mpa"))
	if model in ("goodman", "gerber"):
		if sigma_uts is None:
			raise JobError(f"{model} needs an ultimate strength; the registry entry has none — "
			               f"pass sigma_uts_mpa explicitly")
		sigma_uts = float(sigma_uts)
		if not (np.isfinite(sigma_uts) and sigma_uts > 0.0):
			raise JobError(f"sigma_uts_mpa must be finite and > 0, got {sigma_uts}")

	sigma_ref, mode, stress_rec = load_stress(job)
	spectrum = job.get("spectrum")
	if not spectrum:
		raise JobError("spectrum required: [{cycles, load_factor|sigma_a_mpa, ...}, ...]")

	n_valid = curve.get("n_valid")
	shape = np.shape(sigma_ref)
	damage = np.zeros(shape)
	blocks = []
	total_cycles = 0.0
	any_extrapolated = False
	for bi, blk in enumerate(spectrum):
		n_i = float(blk.get("cycles", 0.0))
		if not (np.isfinite(n_i) and n_i >= 0.0):
			raise JobError(f"spectrum[{bi}].cycles must be finite and >= 0, got {blk.get('cycles')!r}")
		sa, sm, src = block_stresses(blk, bi, sigma_ref, mode)
		s_eff = mean_stress_correct(model, sa, sm, float(sigma_uts) if sigma_uts else 1.0)
		peak = float(np.max(sm + sa))
		if sigma_uts is not None and peak > float(sigma_uts):
			raise JobError(
				f"spectrum[{bi}] peak stress {peak:.4g} MPa exceeds the printed ultimate "
				f"{float(sigma_uts):.4g} MPa — that is STATIC failure, not fatigue. Fix the "
				f"load case (or the geometry) before asking for a cycle count.")
		N_i = basquin_life(s_eff, a_mpa, b)
		with np.errstate(divide="ignore", invalid="ignore"):
			d_i = np.where(np.isfinite(N_i), n_i / N_i, 0.0)
		damage = damage + d_i
		total_cycles += n_i
		n_crit = float(np.min(N_i))
		extrap = bool(n_valid) and np.isfinite(n_crit) and (
			n_crit > float(n_valid[1]) or n_crit < float(n_valid[0]))
		any_extrapolated = any_extrapolated or extrap
		blocks.append({
			"index": bi, "name": blk.get("name", f"block{bi}"),
			"cycles": n_i,
			"stress_source": src["source"],
			"sigma_a_mpa_max": float(np.max(sa)),
			"sigma_m_mpa_max": float(np.max(sm)),
			"sigma_max_mpa": peak,
			"sigma_effective_mpa_max": float(np.max(s_eff)),
			"cycles_to_failure_at_critical": (n_crit if np.isfinite(n_crit) else None),
			"damage_max": float(np.max(d_i)),
			"zero_amplitude": bool(np.max(sa) == 0.0),
			"extrapolated_beyond_curve_validity": extrap,
		})

	d_max = float(np.max(damage))
	limit = float(job.get("damage_limit", 1.0))
	crit_index = np.unravel_index(int(np.argmax(damage)), shape) if damage.size else None
	if d_max > 0.0:
		life_status = "finite"
		repeats = limit / d_max
		cycles_to_failure = repeats * total_cycles
	else:
		life_status = "no_damage"
		repeats = None
		cycles_to_failure = None

	band = life_scatter_band(entry) if entry else None
	confidence = {
		"material_data_confidence": meta.get("confidence"),
		"material_data_basis": meta.get("confidence_basis"),
		"probability_of_survival": meta.get("probability_of_survival"),
		"curve": meta.get("curve_name"),
		"data_provenance": meta.get("data_provenance"),
		"mean_stress_model": model,
		"mean_stress_validated_for_printed_polymers": (model == "intrinsic"),
		"miner_rule_validated_for_printed_polymers": False,
		"any_block_extrapolated_beyond_curve_validity": any_extrapolated,
		"curve_valid_cycles": n_valid,
		"life_scatter_factor_90_10": (
			{"min": band[0], "max": band[1],
			 "meaning": "ratio of the 10%-survival life to the 90%-survival life in the "
			            "SOURCE data (T_sigma^k, recomputed from the source table). The "
			            "prediction below is a point on a distribution this wide."}
			if band else None),
		"statement": (
			"SCREENING ESTIMATE. Printed-part fatigue is controlled by layer adhesion and "
			"process defects, not by the bulk polymer; Miner's linear rule is unvalidated "
			"for printed polymers (sequence effects unmodelled) and the source scatter "
			"alone spans "
			+ (f"{band[0]:.1f}x to {band[1]:.0f}x in life. " if band else "an unquantified range. ")
			+ ("At least one block extrapolates beyond the fitted curve's validated cycle "
			   "range — treat that block's life as unevidenced. " if any_extrapolated else "")
			+ "Rank variants with this; do not certify a cycle count with it."),
	}

	payload = {
		"ok": True,
		"analyzer_version": ANALYZER_VERSION,
		"method": (f"Basquin S-N (sigma_{'max' if measure == 'max' else 'a'} = a N^b, "
		           f"a = {a_mpa:.6g} MPa, b = {b:.6g}) + {model} mean-stress + "
		           f"Palmgren-Miner linear damage"),
		"material": {k: v for k, v in meta.items() if k != "conflicts"},
		"curve": {"stress_measure": measure, "a_mpa": a_mpa, "b": b,
		          "k_negative_inverse_slope": curve.get("k"),
		          "n_valid": n_valid, "r_ratio_basis": curve.get("r_ratio_basis")},
		"sigma_uts_mpa": (float(sigma_uts) if sigma_uts is not None else None),
		"stress_input": stress_rec,
		"load_orientation": orient,
		"blocks": blocks,
		"spectrum_cycles_total": total_cycles,
		"damage": {
			"total_at_critical_location": d_max,
			"limit": limit,
			"life_status": life_status,
			"spectrum_repeats_to_failure": repeats,
			"cycles_to_failure": cycles_to_failure,
			"critical_index": ([int(v) for v in crit_index] if crit_index is not None else None),
			"note": ("life_status 'no_damage': every block has zero stress amplitude, so "
			         "Basquin gives N = infinity and D = 0. Life is INFINITE WITHIN THIS "
			         "MODEL — the model has no endurance limit and no static/creep failure "
			         "mode, so this is not a statement that the part lasts forever."
			         if life_status == "no_damage" else
			         "D = sum(n_i/N_i) at the most-damaged location; failure predicted at "
			         "D = limit."),
		},
		"confidence": confidence,
		"conflicts": meta.get("conflicts", []),
		"gaps_unknowns": meta.get("gaps_unknowns", []),
		"timings_s": {"total_s": round(time.monotonic() - t0, 4)},
	}
	if mode == "field":
		dpath = out_dir / "damage_field.npy"
		np.save(dpath, np.ascontiguousarray(damage.astype(np.float32)))
		payload["damage_field_npy"] = str(dpath)
		payload["damage"]["field_shape"] = list(shape)
	return payload


def main() -> None:
	if len(sys.argv) != 2:
		emit({"ok": False, "error": "usage: ace_fatigue_runner.py <job.json>"})
		sys.exit(1)
	job = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
	emit(run(job))


if __name__ == "__main__":
	try:
		main()
	except Exception as exc:  # noqa: BLE001 — the JSON line is the contract...
		# ...and this runner ALSO exits 1, including for a DataRefusal: "no
		# credible printed data for this material" is a failure of the request,
		# not a result, and a shell gate must see it as one.
		emit({"ok": False, "error": f"{type(exc).__name__}: {exc}"})
		sys.exit(1)
