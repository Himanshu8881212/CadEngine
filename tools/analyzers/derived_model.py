#!/usr/bin/env python3
"""derived_model.py — the scaffold that makes an on-the-fly physics model honest.

The analysis loop this repo serves is: (1) research the physics, (2) write the
governing equations down WITH SOURCES, (3) implement them as a runnable model,
(4) validate the implementation against closed forms / known limits, (5) only
then let an optimizer (tools/param_optimize.py, command evaluator) drive it.
This module enforces that order BY CONSTRUCTION for step-3 models an agent
derives on the fly (a vented-box speaker, a thermal RC network, a linkage…):

  * A subclass CANNOT EXIST without governing equations, assumptions, units,
    limits of validity, and at least one SOURCE citation — `__init_subclass__`
    refuses at import time, so an unsourced model never runs at all.
  * Every run re-executes the subclass's validation GATES (checks against
    closed-form ground truth) and REFUSES to evaluate if any gate fails —
    a broken model cannot emit numbers ("refuse-before-run").
  * Results ship inside the `lmcad.analysis.v1` provenance envelope with
    validation_status `synthesized_inloop`, a structured convergence receipt,
    and a manifest reference — and are re-checked by
    `provenance.check_synthesized` before printing. A derived model can NEVER
    claim `validated`; that tier needs a committed manifest + pin
    (docs/MANIFEST_SCHEMA.md), which `write_manifest` drafts for you.
  * The LAST stdout line is the envelope JSON (`ok: true`), so param_optimize
    can drive any derived model directly:
        "evaluator": {"kind": "command", "argv": ["python3", "my_model.py", "$JOB"]}
        "targets":   [{"expr": "values.overshoot_pct", "value": 10, "tol": 0.5}]

Usage:
  python3 tools/derived_model.py job.json        # run the worked exemplar
  python3 tools/derived_model.py --selftest      # gates + envelope + determinism
  python3 tools/derived_model.py --manifest OUT  # write the exemplar's manifest
  python3 tools/derived_model.py --new NAME      # start a new model file

The worked exemplar (`DampedOscillator`) is a real second-order step-response
model gated against three textbook closed forms — copy its shape. Committed
derived manifests under tools/manifests/derived/ are auto-registered in the
graduation ledger (tools/analyzer_registry.py) at tier Demonstrated (Validated
only once a pin file exists).
"""
from __future__ import annotations

import json
import math
import os
import sys

sys.dont_write_bytecode = True
TOOLS = os.path.dirname(os.path.abspath(__file__))  # tools/analyzers
sys.path.insert(0, os.path.dirname(TOOLS))  # tools/: provenance + the layout map
import _layout  # noqa: E402
_layout.add_import_paths()
import provenance  # noqa: E402


def log(msg: str) -> None:
	print(msg, file=sys.stderr, flush=True)


class DerivedModel:
	"""Base class for agent-derived physics models. Subclasses declare the
	contract as class attributes and implement `run_gates()` + `evaluate()`."""

	# --- REQUIRED declarations (import-time enforced) ---------------------
	name: str = ""                       # registry-style slug, e.g. "vented_box_spl"
	version: str = "0.1.0"
	title: str = ""                      # one-line human title
	equations: list = []                 # [{name, expr, description}]
	assumptions: list = []               # [str, ...]
	sources: list = []                   # [{title, ref, used_for}] — CITATIONS, >= 1
	units: dict = {}                     # {"inputs": {...}, "outputs": {...}}
	limits_of_validity: dict = {}        # parameter ranges the gates vouch for
	discretization: dict = {}            # {method, element, notes}
	boundary_conditions: dict = {"fixtures": "not applicable (lumped/1-D model)",
	                             "loads": "not applicable",
	                             "selector_count_unit": "not applicable"}
	measured: str = ""                   # fixed date the gates were last measured (never a clock read)

	def __init_subclass__(cls, **kw):
		super().__init_subclass__(**kw)
		missing = [a for a in ("name", "title", "equations", "assumptions",
		                       "sources", "units", "limits_of_validity",
		                       "discretization", "measured") if not getattr(cls, a)]
		if missing:
			raise TypeError(
				f"derived model {cls.__name__} is missing required contract "
				f"declarations: {missing} — a model with no stated equations, "
				f"assumptions, units, limits, or SOURCES must not exist (write "
				f"them first; that is the point of the scaffold)")
		bad = [s for s in cls.sources
		       if not (isinstance(s, dict) and s.get("title") and s.get("ref") and s.get("used_for"))]
		if bad:
			raise TypeError(
				f"derived model {cls.__name__}: every source needs "
				f"{{title, ref, used_for}} — got {bad!r}. 'ref' is the citation "
				f"(book+edition+section, DOI, standard number, or URL); "
				f"'used_for' says which equation/gate it backs")

	# --- subclass API ------------------------------------------------------
	def run_gates(self) -> list:
		"""Return [{name, ground_truth, expected, obtained, limit_rel}, ...] —
		each gate compares the IMPLEMENTATION against an independent closed
		form / known limit. The base class computes pass/fail."""
		raise NotImplementedError

	def evaluate(self, job: dict):
		"""Return (values: dict, residual_or_convergence: dict). Raise
		ValueError with an actionable message for out-of-limits inputs —
		limits_of_validity is ENFORCED here, not just documented."""
		raise NotImplementedError

	# --- provided machinery -------------------------------------------------
	def checked_gates(self) -> list:
		gates = []
		for g in self.run_gates():
			exp, obt = float(g["expected"]), float(g["obtained"])
			lim = float(g["limit_rel"])
			scale = max(abs(exp), 1e-30)
			rel = abs(obt - exp) / scale
			gates.append({**g, "rel_error": rel, "passed": rel <= lim})
		if not gates:
			raise ValueError(f"{self.name}: run_gates() returned no gates — a "
			                 f"derived model with nothing checking it must not run")
		return gates

	def manifest(self) -> dict:
		pin = getattr(self, "pin_file", None)
		return {
			"schema": "lmcad.manifest.v1",
			"analyzer": self.name,
			"analyzer_version": self.version,
			"title": self.title,
			"model_file": os.path.basename(getattr(sys.modules[type(self).__module__], "__file__", "derived_model.py")),
			"derived_model": True,
			"governing_equations": self.equations,
			"assumptions": self.assumptions,
			"sources": self.sources,
			"boundary_conditions": self.boundary_conditions,
			"units": self.units,
			"discretization": self.discretization,
			"validation": {
				"pin_file": pin or "",
				"additional_pins": [],
				"ground_truth": "; ".join(f"{g['name']}: {g['ground_truth']}" for g in self.run_gates()),
				"error_band": {g["name"]: f"<= {100.0 * float(g['limit_rel']):g}% rel" for g in self.run_gates()},
				"direction": "inline self-check gates re-run on EVERY invocation "
				             "(refuse-before-run); no committed pin unless pin_file is set",
				"last_measured": self.measured,
			},
			"caveats": list(getattr(self, "caveats", [])) + [
				"Derived model: validation_status is synthesized_inloop, never "
				"'validated' — graduation needs a committed pin per docs/MANIFEST_SCHEMA.md.",
			],
			"limits_of_validity": self.limits_of_validity,
		}

	def write_manifest(self, out_path: str) -> str:
		os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
		with open(out_path, "w") as f:
			json.dump(self.manifest(), f, indent=2)
			f.write("\n")
		return os.path.abspath(out_path)

	def run(self, job: dict) -> dict:
		"""Gates -> evaluate -> stamped envelope (the enforced order)."""
		gates = self.checked_gates()
		worst = max(gates, key=lambda g: g["rel_error"] / max(float(g["limit_rel"]), 1e-30))
		self_check = {"limit": worst["limit_rel"], "expected": worst["expected"],
		              "obtained": worst["obtained"], "passed": all(g["passed"] for g in gates),
		              "gate": worst["name"], "gates_run": len(gates)}
		if not self_check["passed"]:
			failed = [g for g in gates if not g["passed"]]
			return {"ok": False,
			        "error": f"validation gates FAILED — model refuses to evaluate: "
			                 + "; ".join(f"{g['name']}: expected {g['expected']:.6g}, got "
			                             f"{g['obtained']:.6g} (rel {g['rel_error']:.2e} > limit {g['limit_rel']:.0e})"
			                             for g in failed),
			        "self_check": self_check, "gates": gates}
		try:
			values, residual = self.evaluate(job)
		except (ValueError, KeyError) as e:
			return {"ok": False, "error": f"{type(e).__name__}: {e}", "self_check": self_check}
		if not isinstance(residual, dict):
			raise ValueError(f"{self.name}.evaluate must return a STRUCTURED residual/convergence dict")
		residual = {**residual, "gates": gates}
		manifest_ref = getattr(self, "manifest_ref", None) or f"derived_model:{self.name}@{self.version}"
		env = provenance.stamp(
			values,
			geometry_hash=provenance.geometry_hash(
				program={"derived_model": f"{self.name}@{self.version}",
				         "inputs": {k: job[k] for k in sorted(job)}}),
			material_version=str(job.get("material_version", "n/a: lumped/derived model")),
			analyzer_name=self.name,
			analyzer_version=self.version,
			validation_status=provenance.STATUS_SYNTHESIZED_INLOOP,
			residual_or_convergence=residual,
			manifest_ref=manifest_ref,
			self_check=self_check,
		)
		ok, problems = provenance.check_synthesized(env)
		if not ok:
			return {"ok": False, "error": f"synthesis guardrail rejected the envelope: {problems}"}
		return {"ok": True, **env}

	@classmethod
	def main(cls, argv=None) -> int:
		argv = list(sys.argv[1:] if argv is None else argv)
		if not argv or argv[0] in ("-h", "--help"):
			print(sys.modules[cls.__module__].__doc__ or cls.__doc__)
			return 0
		model = cls()
		if argv[0] == "--manifest":
			out = argv[1] if len(argv) > 1 else os.path.join(
				os.path.dirname(TOOLS), "manifests", "derived", f"{model.name}.manifest.json")
			print(json.dumps({"ok": True, "manifest": model.write_manifest(out)}))
			return 0
		if argv[0] == "--selftest":
			return selftest(model)
		try:
			job = json.load(open(argv[0]))
			out = model.run(job)
		except Exception as e:  # honest failure receipt — the JSON line is the contract
			out = {"ok": False, "error": f"{type(e).__name__}: {e}"}
		print(json.dumps(out))
		return 0 if out.get("ok") else 1


# ---------------------------------------------------------------------------
# Worked exemplar: 2nd-order underdamped step response, gated on closed forms.
# ---------------------------------------------------------------------------
class DampedOscillator(DerivedModel):
	"""Unit step response of m x'' + c x' + k x = k·u(t), normalized to
	x'' + 2 zeta omega_n x' + omega_n^2 x = omega_n^2 — integrated by RK4 and
	gated against the textbook overshoot / peak-time / period closed forms."""

	name = "damped_oscillator"
	version = "1.0.0"
	title = "2nd-order underdamped step response (scaffold's worked exemplar)"
	equations = [
		{"name": "normalized 2nd-order ODE",
		 "expr": "x'' + 2 zeta omega_n x' + omega_n^2 x = omega_n^2 u(t)",
		 "description": "Mass-spring-damper / RLC / servo prototype with damping ratio zeta and natural frequency omega_n; unit step command."},
		{"name": "peak overshoot (gate ground truth)",
		 "expr": "Mp = exp(-pi zeta / sqrt(1 - zeta^2))",
		 "description": "Closed-form fractional overshoot of the underdamped step response."},
		{"name": "peak time (gate ground truth)",
		 "expr": "tp = pi / (omega_n sqrt(1 - zeta^2))",
		 "description": "Time of the first response maximum."},
	]
	assumptions = [
		"Linear, time-invariant, single degree of freedom; constant coefficients.",
		"Underdamped regime 0 <= zeta < 1 (enforced — evaluate refuses otherwise).",
		"Zero initial conditions; ideal unit step input.",
	]
	sources = [
		{"title": "K. Ogata, Modern Control Engineering, 5th ed., sec. 5-3 (transient response of 2nd-order systems)",
		 "ref": "ISBN 978-0136156734",
		 "used_for": "overshoot Mp and peak-time tp closed forms (gates 1-2)"},
		{"title": "S. S. Rao, Mechanical Vibrations, 6th ed., ch. 2 (free vibration of SDOF systems)",
		 "ref": "ISBN 978-0134361307",
		 "used_for": "undamped natural period 2*pi/omega_n (gate 3) and the damped frequency relation"},
	]
	units = {
		"inputs": {"zeta": "dimensionless (0 <= zeta < 1)", "omega_n_rad_s": "rad/s",
		           "t_end_s": "s (optional, default 10 periods)", "dt_s": "s (optional)"},
		"outputs": {"overshoot_pct": "% of final value", "peak_time_s": "s",
		            "settling_time_2pct_s": "s", "damped_freq_hz": "Hz"},
	}
	discretization = {"method": "explicit time integration", "element": "RK4, fixed step",
	                  "notes": "default dt = T_d/400; convergence receipt compares dt vs dt/2."}
	limits_of_validity = {
		"regime": "underdamped 0 <= zeta < 1 ONLY — the gates' closed forms do not exist at zeta >= 1 and the model refuses",
		"slenderness": "not applicable",
		"resolution": "outputs quoted from a trajectory sampled at dt; peak/settling times are dt-quantized (receipt carries dt)",
	}
	caveats = ["settling_time_2pct_s is the LAST 2%-band exit found on the sampled trajectory (dt-quantized)."]
	measured = "2026-07-17"

	@staticmethod
	def _integrate(zeta: float, wn: float, t_end: float, dt: float):
		"""RK4 on [x, v]; returns (ts, xs)."""
		def f(x, v):
			return v, wn * wn * (1.0 - x) - 2.0 * zeta * wn * v
		n = max(8, int(round(t_end / dt)))
		x, v, ts, xs = 0.0, 0.0, [0.0], [0.0]
		for i in range(n):
			k1x, k1v = f(x, v)
			k2x, k2v = f(x + 0.5 * dt * k1x, v + 0.5 * dt * k1v)
			k3x, k3v = f(x + 0.5 * dt * k2x, v + 0.5 * dt * k2v)
			k4x, k4v = f(x + dt * k3x, v + dt * k3v)
			x += dt * (k1x + 2 * k2x + 2 * k3x + k4x) / 6.0
			v += dt * (k1v + 2 * k2v + 2 * k3v + k4v) / 6.0
			ts.append((i + 1) * dt)
			xs.append(x)
		return ts, xs

	@classmethod
	def _metrics(cls, zeta: float, wn: float, t_end: float, dt: float) -> dict:
		ts, xs = cls._integrate(zeta, wn, t_end, dt)
		i_pk = max(range(len(xs)), key=lambda i: xs[i])
		settle = 0.0
		for t, x in zip(ts, xs):
			if abs(x - 1.0) > 0.02:
				settle = t
		wd = wn * math.sqrt(max(1.0 - zeta * zeta, 0.0))
		return {"overshoot_pct": 100.0 * max(xs[i_pk] - 1.0, 0.0),
		        "peak_time_s": ts[i_pk],
		        "settling_time_2pct_s": settle,
		        "damped_freq_hz": wd / (2.0 * math.pi)}

	def run_gates(self) -> list:
		wn = 10.0
		zeta = 0.2
		wd = wn * math.sqrt(1.0 - zeta * zeta)
		td = 2.0 * math.pi / wd
		m = self._metrics(zeta, wn, 10.0 * td, td / 400.0)
		gates = [
			{"name": "overshoot_vs_closed_form",
			 "ground_truth": "Mp = exp(-pi*zeta/sqrt(1-zeta^2)) at zeta=0.2 (Ogata 5-3)",
			 "expected": 100.0 * math.exp(-math.pi * zeta / math.sqrt(1.0 - zeta * zeta)),
			 "obtained": m["overshoot_pct"], "limit_rel": 5e-3},
			{"name": "peak_time_vs_closed_form",
			 "ground_truth": "tp = pi/(omega_n*sqrt(1-zeta^2)) at zeta=0.2 (Ogata 5-3)",
			 "expected": math.pi / wd,
			 "obtained": m["peak_time_s"], "limit_rel": 5e-3},
		]
		# Gate 3: zeta = 0 — consecutive peaks of the undamped response are one
		# natural period 2*pi/omega_n apart (Rao ch. 2).
		t0 = 2.0 * math.pi / wn
		ts, xs = self._integrate(0.0, wn, 3.2 * t0, t0 / 400.0)
		peaks = [i for i in range(1, len(xs) - 1) if xs[i] > xs[i - 1] and xs[i] >= xs[i + 1]]
		gates.append(
			{"name": "undamped_period",
			 "ground_truth": "T = 2*pi/omega_n between consecutive undamped peaks (Rao ch. 2)",
			 "expected": t0,
			 "obtained": ts[peaks[1]] - ts[peaks[0]] if len(peaks) >= 2 else float("nan"),
			 "limit_rel": 1e-3})
		return gates

	def evaluate(self, job: dict):
		zeta = float(job["zeta"])
		wn = float(job["omega_n_rad_s"])
		if not (0.0 <= zeta < 1.0):
			raise ValueError(f"invalid_param: zeta={zeta} — this model's gates vouch "
			                 f"only for the underdamped regime 0 <= zeta < 1 "
			                 f"(limits_of_validity is enforced, not advisory)")
		if wn <= 0.0:
			raise ValueError(f"invalid_param: omega_n_rad_s={wn} must be > 0")
		wd = wn * math.sqrt(1.0 - zeta * zeta)
		td = 2.0 * math.pi / wd
		t_end = float(job.get("t_end_s", 10.0 * td))
		dt = float(job.get("dt_s", td / 400.0))
		vals = self._metrics(zeta, wn, t_end, dt)
		half = self._metrics(zeta, wn, t_end, dt / 2.0)
		drift = abs(half["overshoot_pct"] - vals["overshoot_pct"]) / max(abs(half["overshoot_pct"]), 1e-30)
		residual = {"integrator": "RK4 fixed-step", "dt_s": dt, "t_end_s": t_end,
		            "dt_refinement_rel_change": drift,
		            "dt_converged": drift < 1e-3, "reported": True}
		return vals, residual


NEW_TEMPLATE = '''#!/usr/bin/env python3
"""{name}_model.py — derived physics model (STUB — fill every TODO before running).

Contract: tools/derived_model.py. The model will REFUSE to import until the
equations / assumptions / units / limits / SOURCES are declared, and refuse to
run until the gates pass against real closed-form ground truth.
"""
import os, sys
sys.path.insert(0, {tools!r})
from derived_model import DerivedModel


class {cls}(DerivedModel):
	name = "{name}"
	version = "0.1.0"
	title = ""        # TODO one-line title
	equations = []    # TODO [{{"name", "expr", "description"}}] — the physics you researched
	assumptions = []  # TODO what must hold for the equations to apply
	sources = []      # TODO [{{"title", "ref", "used_for"}}] — REAL citations (book/DOI/standard/URL)
	units = {{"inputs": {{}}, "outputs": {{}}}}  # TODO every wire quantity
	discretization = {{"method": "", "element": "", "notes": ""}}  # TODO (or "closed-form, none")
	limits_of_validity = {{"regime": "", "slenderness": "n/a", "resolution": ""}}  # TODO; ENFORCE in evaluate()
	measured = ""     # TODO fixed date you measured the gates (never a clock read)

	def run_gates(self):
		# TODO >= 1 gate: compare the IMPLEMENTATION against an independent
		# closed form / known limit from your sources. No gate, no numbers.
		raise NotImplementedError("write the validation gates first — that is the contract")

	def evaluate(self, job):
		# TODO return (values_dict, residual_or_convergence_dict).
		# Refuse out-of-limits inputs with ValueError("invalid_param: ...").
		raise NotImplementedError


if __name__ == "__main__":
	raise SystemExit({cls}.main())
'''


def selftest(model: DerivedModel) -> int:
	"""Hermetic proof the scaffold enforces its contract end to end."""
	gates = model.checked_gates()
	worst = max(g["rel_error"] for g in gates)
	assert all(g["passed"] for g in gates), (
		f"selftest: exemplar gates must pass, got "
		f"{[(g['name'], g['rel_error']) for g in gates if not g['passed']]}")

	job = {"zeta": 0.15, "omega_n_rad_s": 25.0}
	out1, out2 = model.run(dict(job)), model.run(dict(job))
	assert out1["ok"], f"selftest: exemplar run refused: {out1.get('error')}"
	mp = 100.0 * math.exp(-math.pi * 0.15 / math.sqrt(1.0 - 0.15 ** 2))
	got = out1["values"]["overshoot_pct"]
	assert abs(got - mp) / mp < 5e-3, (
		f"selftest: overshoot {got:.4f}% vs closed form {mp:.4f}% off by "
		f"{abs(got - mp) / mp:.2e} (limit 5e-3)")
	assert out1["provenance"]["validation_status"] == "synthesized_inloop", out1["provenance"]
	assert out1["residual_or_convergence"]["dt_converged"] is True, out1["residual_or_convergence"]
	assert json.dumps(out1, sort_keys=True) == json.dumps(out2, sort_keys=True), (
		"selftest: envelope is not deterministic across identical runs")
	ok, problems = provenance.check_synthesized(out1)
	assert ok, f"selftest: guardrail must accept the stamped envelope: {problems}"

	# The refusal paths must be LOUD, not silent.
	bad = model.run({"zeta": 1.4, "omega_n_rad_s": 25.0})
	assert not bad.get("ok") and "invalid_param" in bad.get("error", ""), (
		f"selftest: out-of-limits zeta must refuse with invalid_param, got {bad!r}")
	try:
		class Unsourced(DerivedModel):  # noqa: N801 — deliberate contract violation
			name, title = "x", "y"
			equations = [{"name": "e", "expr": "x", "description": "d"}]
			assumptions = ["a"]
			sources = []  # <- the violation
			units = {"inputs": {}, "outputs": {}}
			discretization = {"method": "m", "element": "e", "notes": "n"}
			limits_of_validity = {"regime": "r"}
			measured = "2026-07-17"
		raise AssertionError("selftest: a source-less model class must be impossible to define")
	except TypeError as e:
		assert "sources" in str(e) or "SOURCES" in str(e), f"wrong refusal: {e}"

	print(f"derived_model selftest PASS: {len(gates)} gates (worst rel {worst:.2e}), "
	      f"envelope deterministic + guardrail-accepted, refusals loud")
	return 0


def main() -> int:
	if len(sys.argv) >= 2 and sys.argv[1] == "--new":
		name = sys.argv[2] if len(sys.argv) > 2 else ""
		if not name.isidentifier():
			print(json.dumps({"ok": False, "error": f"--new needs a python-identifier name, got {name!r}"}))
			return 1
		path = os.path.abspath(f"{name}_model.py")
		if os.path.exists(path):
			print(json.dumps({"ok": False, "error": f"{path} already exists — refusing to overwrite"}))
			return 1
		cls = "".join(w.capitalize() for w in name.split("_")) or "Model"
		with open(path, "w") as f:
			f.write(NEW_TEMPLATE.format(name=name, cls=cls, tools=TOOLS))
		print(json.dumps({"ok": True, "created": path,
		                  "next": "fill equations/assumptions/units/limits/SOURCES, "
		                          "write the gates, then --selftest your job"}))
		return 0
	return DampedOscillator.main()


if __name__ == "__main__":
	raise SystemExit(main())
