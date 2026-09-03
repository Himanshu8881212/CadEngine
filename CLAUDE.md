# LMCAD — agent orders

Two folders, one job each. **`campaign/` = how you must WORK** (the binding
rules). **`docs/` = what the engine IS and how far it can be TRUSTED** (the
contracts you cite). Each has a `README.md` listing its files.

The engine is driven through one binary, the `kernel-api` CLI. There is no
server, no IDE and no MCP surface — those were removed on 2026-09-03.

## Before ANY part/design campaign

Read, in this order (they are binding, not background):

1. `campaign/OPERATOR_BRIEF.md` — engine mental model, exact commands,
   failure playbook.
2. `campaign/DELIVERABLE_SPEC.md` — the contract every campaign must meet
   (layout, gates, negative controls, honesty rules, final self-check).
3. `campaign/digests/` — exact op/tool JSON shapes when authoring.
   `campaign/DESIGN_GUIDE.md` is the full operator manual behind them.
4. `campaign/PRINTABLES_LISTING_SPEC.md` — binding for every `publish/` listing:
   the live form's fields and limits (summary 120 chars hard, mandatory AI
   declaration), contest mechanics, description template, image order,
   `campaign/listing_lint.py` + the campaign's receipt checker in `<campaign>/run_all.sh`.

Before claiming a number, check its contract in `docs/`: `docs/ANALYSIS_TIERS.md`
(what a tier means) and `docs/ANALYSIS_DOMAINS.md` (what may be analyzed at all),
`docs/NUMERICS.md` (tolerances), `docs/ROBUSTNESS.md` (validity floors).
Before concluding the engine is broken, check `campaign/friction/ENGINE.md`.

Repo root contains a space: always quote
`"/Users/himanshu/Work/New-LMCAD/cad engine"`.

## Non-negotiables the maintainer has called out

- **Every campaign ships an `assembly/` folder — single-part campaigns
  included.** It holds the ballooned exploded diagram
  (`tools/publish/assembly_doc.py`), `ASSEMBLY_instructions.md`, and the BOM
  (`tools/publish/production_dossier.py` → `bom_dossier.{csv,json}`), plus
  `scene/*.stl` when the part has distinguishable bodies. Wire the
  generation into the campaign's `<campaign>/run_all.sh`. See DELIVERABLE_SPEC §1 and
  the `school_system/*` exemplars.
- Engine (`crates/`) and `tools/` source are read-only during campaigns:
  log issues to `campaign/friction/<part>.md` instead (shape in
  `campaign/friction/README.md`). Engine fixes happen only when the maintainer
  asks for them explicitly.
- No claim without a receipt; refusals are recorded, never laundered;
  negative controls must actually fail.
- Listings declare "Yes — AI-assisted creation" on Printables; contest T&C
  with an AI clause are quoted to the user before entry (spec §2).

## Quick commands

```sh
"./target/release/kernel-api" run <program.json> --out-dir <dir>
"./target/release/kernel-api" asm <assembly.lmcasm> --out-dir <dir>
python3 tools/<tool>.py job.json [--out receipt.json]   # forwards to tools/{analyzers,publish}/<tool>.py (2026-09-02 layout; map in tools/_layout.py)
sh <campaign>/run_all.sh          # from repo root; NCs must exit 1
```
