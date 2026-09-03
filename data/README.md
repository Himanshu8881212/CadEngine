# data/ — inputs the code reads by path

Not documentation and not a campaign: small data files that the kernel and the
tools open by a repo-relative path, so they must live at a stable location.

| folder | what it is | read by |
|---|---|---|
| `profiles/` | FDM process profiles (`<printer>.json`); `conservative_default.json` is the one every campaign falls back to | `kernel_model::process` (`data/profiles/{name}.json`), `tools/ingest_calibration.py`, `tools/field_triage.py` |
| `bench/` | the agent-surface benchmark baseline (`scorecard.json`) | `crates/agent-bench` writes it, `docs/BENCH.md` quotes it |

Moved here from the repository root on 2026-09-03; the path convention in the
code changed with them, so `profiles/x.json` is now `data/profiles/x.json`.
