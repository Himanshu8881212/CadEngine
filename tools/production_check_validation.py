#!/usr/bin/env python3
"""Validation pin: production_check.py vs the material tables it derates from.

The analyzer is a rules engine over two tables — tools/material_db.json
(yield, ultimate, HDT-class limit, layer-adhesion factor, fatigue knockdown)
and tools/materials/pla.json#creep.sig_allow_mpa (the temperature x duration
creep table read through tools/materials.py). Its ground truth is therefore
the table cell times the documented rule, which anyone can redo on paper; every
expected number below is DERIVED IN THIS FILE from a named cell. A wrong
divisor, a derate applied to the wrong rule, a creep cell read at the wrong
row, a refusal that quietly serves a number, or an exit code that disagrees
with `ok` trips a pin.

	Pin 1  static rule:  PLA yield 55 / demand 10 -> SF 5.5 (pass at 2.0); temp 55/25 = 2.2
	Pin 2  creep rule:   PLA 23 C / 8760 h -> table cell [23C][1y] = 2.5 MPa (SF 2.5 at 1 MPa);
	                     23 C / 24 h -> [23C][24h] = 5.0; 25 C / 720 h rounds UP to [55C][30d] = 0.5
	Pin 3  temperature:  PLA at 60 C > limit 55 C -> temp FAILS, SF 55/60, ok:false, exit 2
	Pin 4  anisotropy:   across-layer static 55 x 0.55 = 30.25 MPa; sustained 2.5 x 0.55 = 1.375;
	                     an in-plane load skips the rule WITH a note
	Pin 5  fatigue:      PLA ultimate 60 x knockdown 0.3 = 18 MPa (SF 3.6 at 5 MPa)
	Pin 6  refusals:     sustained with NO duration_h -> refusal.creep_duration_required;
	                     70 C sustained -> refusal.creep_temp_above_tabulated;
	                     PETG sustained -> refusal.creep_no_table — each allowable 0.0,
	                     pass false, ok:false, exit 2, legacy 11.0 MPa scalar reported NOT used
	Pin 7  a request the tool cannot run (unknown material) exits 1, not 2
	Pin 8  determinism: the stdout receipt is byte-identical across two runs

Hermetic: stdlib only; the analyzer is driven through its real CLI
(subprocess, last stdout line = the receipt, `$?` = the contract).

Measured 2026-09-02 (pinned here): every SF matched the hand derivation to
1e-4 (receipt rounding); the three refusals carried their named kinds with
allowable 0.0 and exit 2; the unknown-material request exited 1; reruns
identical.

Run:  python3 tools/production_check_validation.py   (any python3, stdlib only)
Exit: 0 iff all assertions hold; nonzero with a message otherwise.
"""
import json
import os
import subprocess
import sys
import tempfile

TOOLS = os.path.dirname(os.path.abspath(__file__))
TOOL = os.path.join(TOOLS, "production_check.py")
TOL = 1e-4  # the receipt rounds allowables/SF to 4 decimals

# --- the table cells the rules read (cited so a drift in the table is visible) ---
PLA_YIELD = 55.0        # tools/material_db.json PLA.yield_mpa
PLA_ULT = 60.0          # tools/material_db.json PLA.ultimate_mpa
PLA_LIMIT_C = 55.0      # tools/material_db.json PLA.service_temp_c (HDT-class)
PLA_LAF = 0.55          # tools/material_db.json PLA.layer_adhesion_factor (= pla.json z_vs_xy_strength_ratio)
PLA_FATIGUE_KD = 0.3    # tools/material_db.json PLA.fatigue_knockdown
PLA_LEGACY_MPA = 11.0   # 55 x creep_sustained_fraction 0.2 — reported, NEVER an allowable
CREEP_23C_1Y = 2.5      # tools/materials/pla.json creep.sig_allow_mpa["23C"]["1y"]
CREEP_23C_24H = 5.0     # tools/materials/pla.json creep.sig_allow_mpa["23C"]["24h"]
CREEP_55C_30D = 0.5     # tools/materials/pla.json creep.sig_allow_mpa["55C"]["30d"]


def run(job: dict, workdir: str, tag: str):
	job_path = os.path.join(workdir, f"{tag}.json")
	with open(job_path, "w") as f:
		json.dump(job, f)
	env = dict(os.environ)
	env["LMCAD_RECEIPT_DRY_RUN"] = "1"  # a pin must never write receipts anywhere
	out = subprocess.run([sys.executable, TOOL, job_path], capture_output=True,
	                     text=True, timeout=120, env=env)
	last = ""
	for line in out.stdout.splitlines():
		if line.strip():
			last = line
	assert last, f"pin '{tag}': no receipt on stdout; stderr tail: {out.stderr[-300:]!r}"
	return json.loads(last), out.returncode, last


def rule(rec: dict, name: str) -> dict:
	rows = [r for r in rec.get("rules", []) if r["rule"] == name]
	assert rows, f"receipt has no '{name}' rule row; rules={[r['rule'] for r in rec.get('rules', [])]}"
	return rows[0]


def skipped(rec: dict, name: str):
	return next((s for s in rec.get("skipped", []) if s["rule"] == name), None)


def close(a, b, what, tol=TOL):
	assert a is not None and abs(float(a) - float(b)) <= tol, f"{what}: got {a!r}, hand derivation says {b!r}"


def main() -> None:
	work = tempfile.mkdtemp(prefix="pcheck_pin_")

	# ------------------------------------------------------------------
	# Pin 1 — static + temperature at defaults (25 C, SF 2.0), demand 10 MPa.
	#   static: 55 / 10 = 5.5 >= 2.0 -> pass ; temp: 25 <= 55 pass, SF = 55/25 = 2.2
	#   creep/fatigue skipped (not sustained / not cyclic); anisotropy skipped with a note.
	# ------------------------------------------------------------------
	r1, rc1, _ = run({"material": "PLA", "max_von_mises_pa": 10e6}, work, "pin1_static")
	st, tp = rule(r1, "static"), rule(r1, "temp")
	close(st["allowable_mpa"], PLA_YIELD, "pin 1 static allowable = yield 55")
	close(st["demand_mpa"], 10.0, "pin 1 demand = 10e6 Pa / 1e6")
	close(st["SF"], PLA_YIELD / 10.0, "pin 1 static SF = 55/10")
	close(tp["allowable_c"], PLA_LIMIT_C, "pin 1 temp limit")
	close(tp["demand_c"], 25.0, "pin 1 default service temperature")
	close(tp["SF"], PLA_LIMIT_C / 25.0, "pin 1 temp SF = 55/25")
	assert st["pass"] and tp["pass"] and r1["ok"] is True and rc1 == 0, (
		f"pin 1 FAILED: SF 5.5 >= 2.0 and 25 C <= 55 C must pass with exit 0; got {r1['ok']} exit {rc1}")
	assert skipped(r1, "creep") and skipped(r1, "fatigue") and skipped(r1, "anisotropy") \
		and "UNCHECKED" in skipped(r1, "anisotropy")["reason"], (
		f"pin 1 FAILED: creep/fatigue/anisotropy must be SKIPPED with reasons: {r1['skipped']}")
	assert r1["safety_factor_required"] == 2.0 and r1["anisotropy_derate_applied"] is False
	print(f"pin 1 OK: static SF {st['SF']} (55/10), temp SF {tp['SF']} (55/25), exit 0")

	# ------------------------------------------------------------------
	# Pin 2 — the creep TABLE governs.
	#   23 C / 8760 h (1 y), demand 1 MPa: cell [23C][1y] = 2.5 -> SF 2.5 (pass at 2.0), cell exact.
	#   23 C / 24 h: cell [23C][24h] = 5.0 -> SF 5.0.
	#   25 C / 720 h (30 d): temperature rounds UP to the 55C row -> [55C][30d] = 0.5 -> SF 0.5 FAILS.
	#   The legacy scalar 55 x 0.2 = 11.0 is on the row and is NOT the allowable.
	# ------------------------------------------------------------------
	base = {"material": "PLA", "max_von_mises_pa": 1.0e6, "load_character": {"sustained": True},
	        "service_temp_c": 23.0}
	r2, rc2, _ = run(dict(base, duration_h=8760.0), work, "pin2_creep_1y")
	cr = rule(r2, "creep")
	close(cr["allowable_mpa"], CREEP_23C_1Y, "pin 2 creep allowable = [23C][1y]")
	close(cr["SF"], CREEP_23C_1Y / 1.0, "pin 2 creep SF = 2.5/1.0")
	assert cr["pass"] and cr["refused"] is False and r2["ok"] is True and rc2 == 0, (
		f"pin 2 FAILED: SF 2.5 >= 2.0 must pass with exit 0; got pass={cr['pass']} ok={r2['ok']} exit={rc2}")
	cell = cr["creep_cell"]
	assert cell["temperature_bucket"] == "23C" and cell["duration_bucket"] == "1y" and cell["cell_match"] == "exact" \
		and cell["row_used_c"] == 23.0 and cell["col_used_h"] == 8760.0, f"pin 2 FAILED: cell provenance {cell}"
	close(cr["legacy_scalar_mpa"], PLA_LEGACY_MPA, "pin 2 legacy scalar reported (55 x 0.2)")
	assert cr["duration_from"] == "duration_h" and cr["duration_h"] == 8760.0
	r2b, _, _ = run(dict(base, duration_h=24.0), work, "pin2_creep_24h")
	close(rule(r2b, "creep")["allowable_mpa"], CREEP_23C_24H, "pin 2b creep allowable = [23C][24h]")
	r2c, rc2c, _ = run(dict(base, service_temp_c=25.0, duration_h=720.0), work, "pin2_creep_25c")
	crc = rule(r2c, "creep")
	close(crc["allowable_mpa"], CREEP_55C_30D, "pin 2c 25 C rounds UP to [55C][30d]")
	assert crc["creep_cell"]["temperature_bucket"] == "55C" and crc["creep_cell"]["cell_match"] == "rounded_up_conservative" \
		and crc["pass"] is False and r2c["ok"] is False and rc2c == 2 and r2c.get("error_kind") == "gate_failed", (
		f"pin 2c FAILED: 25 C must read the 55C row (0.5 MPa), fail SF 0.5 < 2.0, ok:false, exit 2, "
		f"error_kind gate_failed; got {crc['creep_cell']['temperature_bucket']} / {crc['pass']} / {r2c['ok']} / {rc2c} / {r2c.get('error_kind')}")
	print(f"pin 2 OK: creep 23C/1y {cr['allowable_mpa']} MPa (SF {cr['SF']}), 23C/24h {rule(r2b, 'creep')['allowable_mpa']}, "
	      f"25C/30d reads the 55C row -> {crc['allowable_mpa']} MPa, exit 2; legacy {cr['legacy_scalar_mpa']} MPa not used")

	# ------------------------------------------------------------------
	# Pin 3 — temperature rule: 60 C > PLA limit 55 C -> FAIL, SF = 55/60 = 0.9167; static still passes.
	# ------------------------------------------------------------------
	r3, rc3, _ = run({"material": "PLA", "max_von_mises_pa": 1.0e6, "service_temp_c": 60.0}, work, "pin3_hot")
	tp = rule(r3, "temp")
	close(tp["allowable_c"], PLA_LIMIT_C, "pin 3 limit")
	close(tp["demand_c"], 60.0, "pin 3 service")
	close(tp["SF"], PLA_LIMIT_C / 60.0, "pin 3 temp SF = 55/60")
	assert tp["pass"] is False and rule(r3, "static")["pass"] is True and r3["ok"] is False and rc3 == 2 \
		and r3.get("error_kind") == "gate_failed", (
		f"pin 3 FAILED: 60 C > 55 C must fail the temp rule -> ok:false exit 2 gate_failed; got {tp['pass']} / {r3['ok']} / {rc3} / {r3.get('error_kind')}")
	print(f"pin 3 OK: 60 C vs limit 55 C -> temp FAIL (SF {tp['SF']}), ok:false, exit 2")

	# ------------------------------------------------------------------
	# Pin 4 — anisotropy (scalar-tier): load along the build direction is 90 deg out of plane (> 30)
	#   -> derate 0.55 on EVERY stress allowable: static 55 x 0.55 = 30.25 (SF 3.025 at 10 MPa),
	#   explicit anisotropy row at 30.25; sustained creep cell 2.5 x 0.55 = 1.375.
	#   A load in the layer plane (x with build z) is 0 deg out of plane -> rule skipped WITH a note.
	# ------------------------------------------------------------------
	r4, rc4, _ = run({"material": "PLA", "max_von_mises_pa": 10e6,
	                  "orientation": {"build_dir": [0, 0, 1], "primary_load_dir": [0, 0, 1]}}, work, "pin4_across")
	an, st = rule(r4, "anisotropy"), rule(r4, "static")
	close(an["allowable_mpa"], PLA_YIELD * PLA_LAF, "pin 4 across-layer allowable = 55 x 0.55")
	close(st["allowable_mpa"], PLA_YIELD * PLA_LAF, "pin 4 static allowable derated = 30.25")
	close(st["SF"], PLA_YIELD * PLA_LAF / 10.0, "pin 4 static SF = 30.25/10")
	assert r4["anisotropy_derate_applied"] is True and rc4 == 0 and any("scalar-tier" in n for n in r4["notes"]), (
		f"pin 4 FAILED: derate must be applied and the scalar-tier note present: {r4['anisotropy_derate_applied']} / {r4['notes']}")
	r4b, _, _ = run({"material": "PLA", "max_von_mises_pa": 1.0e6, "load_character": {"sustained": True},
	                 "service_temp_c": 23.0, "duration_h": 8760.0,
	                 "orientation": {"build_dir": [0, 0, 1], "primary_load_dir": [0, 0, 1]}}, work, "pin4_across_creep")
	crz = rule(r4b, "creep")
	close(crz["allowable_mpa"], CREEP_23C_1Y * PLA_LAF, "pin 4b across-layer creep = 2.5 x 0.55")
	assert crz["across_layer"] is True and crz["creep_cell"]["anisotropy_factor"] == PLA_LAF
	r4c, _, _ = run({"material": "PLA", "max_von_mises_pa": 10e6,
	                 "orientation": {"build_dir": [0, 0, 1], "primary_load_dir": [1, 0, 0]}}, work, "pin4_inplane")
	sk = skipped(r4c, "anisotropy")
	assert sk and "0.0 deg" in sk["reason"] and r4c["anisotropy_derate_applied"] is False \
		and abs(rule(r4c, "static")["allowable_mpa"] - PLA_YIELD) <= TOL, (
		f"pin 4c FAILED: an in-plane load must skip the rule with a note and leave yield at 55: {sk} / {rule(r4c, 'static')}")
	print(f"pin 4 OK: across-layer static {an['allowable_mpa']} MPa (55 x 0.55), creep {crz['allowable_mpa']} MPa (2.5 x 0.55); "
	      f"in-plane skipped: '{sk['reason'][:48]}...'")

	# ------------------------------------------------------------------
	# Pin 5 — fatigue rule: ultimate 60 x knockdown 0.3 = 18 MPa -> SF 3.6 at 5 MPa.
	# ------------------------------------------------------------------
	r5, rc5, _ = run({"material": "PLA", "max_von_mises_pa": 5e6, "load_character": {"cyclic": True}}, work, "pin5_fatigue")
	fa = rule(r5, "fatigue")
	close(fa["allowable_mpa"], PLA_ULT * PLA_FATIGUE_KD, "pin 5 fatigue allowable = 60 x 0.3")
	close(fa["SF"], PLA_ULT * PLA_FATIGUE_KD / 5.0, "pin 5 fatigue SF = 18/5")
	assert fa["pass"] and rc5 == 0 and skipped(r5, "creep")["reason"] == "load not sustained"
	print(f"pin 5 OK: fatigue allowable {fa['allowable_mpa']} MPa (60 x 0.3), SF {fa['SF']}")

	# ------------------------------------------------------------------
	# Pin 6 — the three creep REFUSALS. Each: a full receipt, the creep row refused with its
	# machine-matchable kind, allowable 0.0, pass false, ok:false, exit 2, error_kind refusal.<kind>,
	# and the legacy 11.0 MPa scalar reported but never served.
	# ------------------------------------------------------------------
	cases = [
		("pin6_no_duration", {"material": "PLA", "max_von_mises_pa": 1.0e6, "load_character": {"sustained": True},
		                      "service_temp_c": 23.0}, "creep_duration_required"),
		("pin6_above_table", {"material": "PLA", "max_von_mises_pa": 1.0e6, "load_character": {"sustained": True},
		                      "service_temp_c": 70.0, "duration_h": 24.0}, "creep_temp_above_tabulated"),
		("pin6_no_table", {"material": "PETG", "max_von_mises_pa": 1.0e6, "load_character": {"sustained": True},
		                   "service_temp_c": 23.0, "duration_h": 24.0}, "creep_no_table"),
	]
	for tag, job, kind in cases:
		r6, rc6, _ = run(job, work, tag)
		cr = rule(r6, "creep")
		assert cr["refused"] is True and cr["refusal_kind"] == kind and cr["allowable_mpa"] == 0.0 \
			and cr["pass"] is False and r6["ok"] is False and rc6 == 2 and r6.get("error_kind") == f"refusal.{kind}", (
			f"{tag} FAILED: expected refusal {kind} with allowable 0.0, ok:false, exit 2, error_kind refusal.{kind}; "
			f"got refused={cr['refused']} kind={cr['refusal_kind']} allow={cr['allowable_mpa']} ok={r6['ok']} "
			f"exit={rc6} error_kind={r6.get('error_kind')}")
		assert cr["legacy_scalar_mpa"] is not None and cr["legacy_scalar_mpa"] > 0.0, (
			f"{tag} FAILED: the legacy scalar must be REPORTED on a refused row (never served): {cr['legacy_scalar_mpa']}")
		assert len(r6["rules"]) >= 2, f"{tag} FAILED: a refusal must still carry the full per-rule receipt"
		print(f"pin 6 OK: {job['material']} {kind} -> allowable 0.0, exit 2 (legacy {cr['legacy_scalar_mpa']} MPa reported, not used)")
	# 70 C also breaks the temperature rule — both rows must fail, neither may hide the other.
	r6b, _, _ = run(cases[1][1], work, "pin6_above_table_again")
	assert rule(r6b, "temp")["pass"] is False and rule(r6b, "creep")["pass"] is False

	# ------------------------------------------------------------------
	# Pin 7 — could-not-run is exit 1, distinct from a refusal's 2.
	# ------------------------------------------------------------------
	r7, rc7, _ = run({"material": "UNOBTAINIUM", "max_von_mises_pa": 1.0e6}, work, "pin7_unknown_material")
	assert r7["ok"] is False and rc7 == 1 and r7.get("error_kind") == "internal" and "unknown material" in r7.get("error", ""), (
		f"pin 7 FAILED: an unknown material is a request the tool cannot run -> exit 1, internal; got {rc7} / {r7}")
	print(f"pin 7 OK: unknown material -> exit 1 ({r7['error'][:50]}...)")

	# ------------------------------------------------------------------
	# Pin 8 — determinism.
	# ------------------------------------------------------------------
	_, _, la = run(dict(base, duration_h=8760.0), work, "pin8_a")
	_, _, lb = run(dict(base, duration_h=8760.0), work, "pin8_b")
	assert la == lb, "pin 8 FAILED: two runs of the identical job produced different receipts"
	print("pin 8 OK: rerun receipt byte-identical")

	print("production_check validation: ALL PINS OK")


if __name__ == "__main__":
	main()
