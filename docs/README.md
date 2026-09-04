# `docs/` — what the engine IS, and how far it can be trusted

This folder answers *engineering* questions about the kernel: what it computes,
to what tolerance, how it was measured, and where it stops. It is descriptive
and evidence-bearing — nothing here tells a model how to run a design campaign.

**The other door is [`campaign/`](../campaign/)** — the operating rules a model
works under (brief, deliverable contract, op digests, friction log). If you are
about to design a part, start there and come back here for a contract.

| file | what it is for |
|---|---|
| [`ROBUSTNESS.md`](ROBUSTNESS.md) | published pass rates for randomized feature-chain fuzzing — the validity and repair floors |
| [`NUMERICS.md`](NUMERICS.md) | the tolerance contracts: units, ranges, determinism, failure modes, and what is bit-authoritative |
| [`BAR.md`](BAR.md) | the falsifiable 1–10 grading ladder and every dated re-grade against it |
| [`BENCH.md`](BENCH.md) | the performance baseline: workloads, machine, method, measured timings |
| [`ANALYSIS_TIERS.md`](ANALYSIS_TIERS.md) | what Validated / Demonstrated / Cataloged mean, and the guardrails on synthesizing a new analyzer |
| [`ANALYSIS_DOMAINS.md`](ANALYSIS_DOMAINS.md) | the capability contract for physics analysis: which domains this repo can honestly analyze, and which it refuses |
| [`MANIFEST_SCHEMA.md`](MANIFEST_SCHEMA.md) | `lmcad.manifest.v1` — the falsifiable spec format an analyzer must ship to be tiered |
| [`OP_USAGE.md`](OP_USAGE.md) | the op census: which of the 161 dispatched ops shipped campaigns actually named, and the evidence behind the `catalog` cargo feature |
| [`FIELD_REPORTS.md`](FIELD_REPORTS.md) | what happened to printed parts in the real world — the physical half of the flywheel |
| [`ACE_INTEGRATION.md`](ACE_INTEGRATION.md) | the physics moved in-tree (`tools/analyzers/physics/`, Apache-2.0 inside an MIT repo) — why, what the package is, and how ACE and LMCAD divide the work now |
| [`CHANGELOG.md`](CHANGELOG.md) | the dated capability and root-cause-fix ledger |

Also here: `test_doc_contracts.py`, which checks the claims these documents make
against the live binary and tools (`python3 docs/test_doc_contracts.py`), and
`field_reports.jsonl`, the machine-readable form of `FIELD_REPORTS.md`.

`ANALYSIS_TIERS.md` and `ANALYSIS_DOMAINS.md` are referenced by path from CI and
from `tools/analyzer_registry.py`; they stay in `docs/` for that reason even
though a campaign reads them too.
