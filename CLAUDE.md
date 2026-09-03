# LMCAD — agent orders

## Before ANY part/design campaign

Read, in this order (they are binding, not background):

1. `campaign/OPERATOR_BRIEF.md` — engine mental model, exact commands,
   failure playbook.
2. `campaign/DELIVERABLE_SPEC.md` — the contract every campaign must meet
   (layout, gates, negative controls, honesty rules, final self-check).
3. `campaign/digests/` — exact op/tool JSON shapes when authoring.
4. `campaign/PRINTABLES_LISTING_SPEC.md` — binding for every `publish/` listing:
   the live form's fields and limits (summary 120 chars hard, mandatory AI
   declaration), contest mechanics, description template, image order,
   `campaign/listing_lint.py` + the campaign's receipt checker in `run_all.sh`.

Repo root contains a space: always quote
`"/Users/himanshu/Work/New-LMCAD/cad engine"`.

## Non-negotiables the maintainer has called out

- **Every campaign ships an `assembly/` folder — single-part campaigns
  included.** It holds the ballooned exploded diagram
  (`tools/publish/assembly_doc.py`), `ASSEMBLY_instructions.md`, and the BOM
  (`tools/publish/production_dossier.py` → `bom_dossier.{csv,json}`), plus
  `scene/*.stl` when the part has distinguishable bodies. Wire the
  generation into the campaign's `run_all.sh`. See DELIVERABLE_SPEC §1 and
  the `school_system/*` exemplars.
- Engine (`crates/`) and `tools/` source are read-only during campaigns:
  log issues to `campaign/friction/<part>.md` instead. Engine fixes happen
  only when the maintainer asks for them explicitly.
- No claim without a receipt; refusals are recorded, never laundered;
  negative controls must actually fail.
- Listings declare "Yes — AI-assisted creation" on Printables; contest T&C
  with an AI clause are quoted to the user before entry (spec §2).

## Quick commands

```sh
"./target/release/kernel-api" run <program.json> --out-dir <dir>
python3 tools/<tool>.py job.json [--out receipt.json]   # forwards to tools/{analyzers,publish}/<tool>.py (2026-09-02 layout; map in tools/_layout.py)
sh <campaign>/run_all.sh          # from repo root; NCs must exit 1
```
