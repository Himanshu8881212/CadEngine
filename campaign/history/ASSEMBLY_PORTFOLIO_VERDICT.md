# Assembly portfolio verdict — 5 campaigns, verified and repaired

Date: 2026-08-24. Companion to `PART_PORTFOLIO_VERDICT.md` (the 10-part round).
Method: DELIVERABLE_SPEC §5 both halves — the mechanical half by the round-4
census (clone → re-run → byte-compare, `workflows/regress.py`), the semantic
half by five independent adversarial verifier passes (claims traced to
receipts, the five recurring defect classes hunted), followed by five fix
passes and a re-verification of every repair. All fixes were generator-level;
no gate was weakened anywhere; several were strengthened.

## The five, and what verification found

| campaign | verified findings | the sharpest one |
|---|---|---|
| laboratory / slas_microplate_row_index_stage | 1 BLOCKER, 2 MAJOR, 6 MINOR | the shipped BOM could not build the assembly: 4× M3×16 for a joint that is physically 2× Ø6.6 M6, dowels ordered in the class the design record explicitly rejected, pivot hardware missing |
| horology / graham_deadbeat_escapement | 1 BLOCKER, 6 MAJOR, 8 MINOR | the §10 corridor prescription — the campaign's main output — was hand-typed and wrong against its own receipt (claimed exit band 37.2–38.4 mm; receipts measure 38.42–39.83) |
| rail / ls45_turnout_throw_lock | 6 MAJOR, 7 MINOR | DESIGN §§1–13 was a Stage-1 fossil under a "generated" header, and the drift guard structurally could not detect it |
| assistive / ratcheting_cap_wrench | 3 MAJOR, 5 MINOR | the jaw-span coverage promise (67 mm spanning the assumed 63+3 cap band "with margin") was silently invalidated at 63.0 as built |
| hydroponics / reservoir_topoff_float_valve | 2 MAJOR, 9 MINOR | a hand-typed "10:1" lever ratio shipped in all three docs while the receipts beside it say 11.1:1 |

Totals: **2 BLOCKER · 19 MAJOR · 35 MINOR** — the same defect profile as the
parts round. In every campaign the measured numeric core traced receipt-true
(~500 values spot-checked across the five); the defect mass sat where
hand-typed content lives outside the gate machinery: hardware tables, plan
fossils, coverage overclaims, prose duplicating receipts.

## What the fix passes shipped

- **SLAS**: hardware lines now DERIVED from freeze/geom (2× M6×30 — computed
  from the 20 mm grip stack; the verifier-suggested M6×16 was itself
  impossible), h8 dowels with a refuse-if-m6 guard, pivot + washer rows added;
  BOM/instructions/README/geometry agree; thin-wall oracle NC-W observed
  firing; clamp-seating margin now a receipt.
- **Graham**: corridor bands COMPUTED from `prove_stations.json` locate rows
  with a refusal guard; the acting/idle story restated to what receipts show
  (acting FACE clean; pads/strips foul, S2–S4 honestly unlocated); across-layer
  creep allowable corrected 2.5 → 1.375 MPa (a wrong-JSON-key read, 1.8×
  non-conservative); the L2 dwell case shipped as a loud NOT-PERFORMED row
  with its pre-computed SF 0.92 and a duty limit; wall-floor supersessions
  logged (D9/D10) and the rod gained a p05 gate.
- **LS45**: DESIGN §§1–13 regenerated from live receipts; the documented regen
  command no longer truncates the file; drift protection made real
  (tamper-tested: a hand-edited margin now fails the byte diff); the C1–C9
  coverage checker BUILT and receipted; the wheel-pick negative control
  SHIPPED (S-4.2 flange binds 47.093 mm³ analytic, lifted twin clears); the
  23→55 °C creep cliff quantified in a receipt (capacity 11.563 → 2.313 N).
- **Cap wrench**: the 67→63 narrowing recorded as S6-D1 with the Ø63–66
  exclusion stated as a population coverage limit; the hot-fill refusal
  control made failable and PROVEN to fire (stubbed pass → exit 3, null-quote
  refusal); ring genus 3 now gated; falsifiability heading names observed vs
  never-observed classes.
- **Float valve**: ratio derived from baked params everywhere; the phantom
  `tet_prebake_*` citation replaced with the honest overwritten-receipts
  statement; the published reopening limit now FLOORS (2.38 bar); wall-gate
  oracle NC-F observed firing; 72/72 self-check cases green two passes.

## Cross-cutting repairs landed with this round

- **Census truth**: the round-4 census records 267 programs, 0 real failures,
  0 warnings; classifiers now honor declared "expected REFUSAL" headers (10 of
  the 12 old "FAILs" were misfiled designed refusals).
- **Friction F3 retired portfolio-wide**: the round-4 kernel exports merged
  scenes green. LS45 ships kernel scene STLs (guard inverted), SLAS's F9/F10
  retired with 20 scene STLs byte-identical to 2026-08-08, cap wrench
  reclassified with fresh receipts. Graham's `asm_scene_probe` stays a
  DESIGNED refusal (verified byte-identical — a genuine proper
  self-intersection).
- **New friction F5** (engine backlog): the float valve's per-instance
  refusal is a POSED-instance tessellation defect — the identical solid
  exports `route: exact` clean unposed. Recorded in the campaign's friction
  ledger for the next engine fix phase.
- **Non-orientable disclosure**: 0 of 65 shipped portfolio export files carry
  any non-orientable edge (was 26/41 ops with 3–395). DELIVERABLE_SPEC §2.4
  and REBASELINE_RUNBOOK carry the record.

## What remains open, stated as conditions

- Physical coupons and prints: NOTHING HAS BEEN PRINTED. Every print-bearing
  claim remains gated on the printed-coupon steps each campaign lists.
- Engine backlog: union_all disjoint-body cost (biomedical `sweep_motion`
  re-run exception is disclosed in its BUILD_LOG); posed-instance
  tessellation (F5); curved faces with inner loops; the transverse-curve
  sliver family at fine tolerances (now witness-locatable).
- Graham L2 dwell: NOT PERFORMED, shipped as a duty limit.
- LS45 high-bracket creep: still a condition limit (margin 0.3215 at the high
  end), now stated correctly everywhere including the >23 °C cliff.
