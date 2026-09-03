# campaign/history — records of finished rounds (not binding)

These documents describe what happened in the August 2026 campaign rounds. They
are kept verbatim as the record and are **not** part of the binding reading
list — that list is `campaign/OPERATOR_BRIEF.md`, `campaign/DELIVERABLE_SPEC.md`,
`campaign/digests/` and `campaign/PRINTABLES_LISTING_SPEC.md`. Moved here on
2026-09-02; nothing in `crates/`, `tools/` or CI reads them (the two campaign
workflow scripts that cite them as house rules point here).

| document | what it was | date |
|---|---|---|
| `CONCEPTS.md` | The frozen ten-campaign slate (one part per domain) with both adversarial reviews applied — the concept cards the part round was built from. | frozen 2026-08-06 |
| `PART_PORTFOLIO_VERDICT.md` | Verdict on the ten-part round: independent hostile verification, the defect classes found, and the stage-5 repair pass. | 2026-08-07 → 2026-08-08 |
| `ASSEMBLY_PORTFOLIO_VERDICT.md` | Verdict on the five-campaign assembly round (round-4 census byte-compare + five adversarial verifier passes); companion to the part verdict. | 2026-08-24 |
| `ENGINE_FIX_REPORT.md` | Integrator (owner G) report proving the tree healthy after the six-owner fix pass (owners A–F) and that the ten campaigns and the showcase did not regress. | 2026-08-08 |
| `ORCHESTRATOR_VERIFICATION.md` | The orchestrator's independent re-verification of that fix phase, run without trusting the integrator's first (false-green) harness; the house rules `campaign/workflows/regress.py` cites. | 2026-08-08 |
| `REBASELINE_RUNBOOK.md` | Runbook for the engine fix round 2 re-baseline: how shipped artifact bytes were deliberately swapped in after kernel changes, and the order of operations. | prepared 2026-08-14 |
