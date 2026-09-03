# `campaign/` — how the model must WORK

This folder is the operating contract for a model driving this engine: the rules
it designs under, the shapes it authors, and the mistakes it must not repeat. It
is prescriptive. Design campaigns themselves live **outside** this repository
(in the workspace beside the checkout); what lives here is the law they run
under.

**The other door is [`docs/`](../docs/)** — what the engine is and how far it can
be trusted (tolerances, robustness floors, analysis tiers, benchmarks). Come
here to know what to do; go there to know what a number means.

## Read in this order before any part campaign

| file | what it is for |
|---|---|
| [`OPERATOR_BRIEF.md`](OPERATOR_BRIEF.md) | engine mental model, the exact commands, the failure playbook — read FIRST |
| [`DELIVERABLE_SPEC.md`](DELIVERABLE_SPEC.md) | the contract every campaign must meet: layout, gates, negative controls, honesty rules, final self-check |
| [`digests/`](digests/) | the exact op/tool JSON shapes to author against — `ops_core.md` (exact B-rep half), `implicit_recipes.md` (implicit/hybrid/`.lmcasm`), `tools_cookbook.md` (every Python analyzer's job schema), `exemplars.md` (finished-campaign layouts and process lessons), `analysis_honesty.md` |
| [`PRINTABLES_LISTING_SPEC.md`](PRINTABLES_LISTING_SPEC.md) | binding for every `publish/` listing: live form limits, the mandatory AI declaration, contest mechanics, image order |
| [`DESIGN_GUIDE.md`](DESIGN_GUIDE.md) | the full operator manual — every op family taught with an executed receipt. The reference the digests condense |

## Supporting folders

| folder | what it is for |
|---|---|
| [`friction/`](friction/) | the friction logs: `ENGINE.md` is the engine-wide one, the rest are per-campaign. See [`friction/README.md`](friction/README.md) |
| [`fixlog/`](fixlog/) | per-owner records of fix rounds — what was changed in the engine/tools and why |
| [`history/`](history/) | records of finished rounds and verdicts. Kept verbatim; **not** binding |
| [`workflows/`](workflows/) | reusable campaign machinery: `regress.py` (regression/re-baseline census), `asmfinish.js` |

`listing_lint.py` is the machine validator for `PRINTABLES_LISTING_SPEC.md`:
`python3 campaign/listing_lint.py <campaign>/publish/PRINTABLES_LISTING.md`.

## The rules that outrank everything else

- No claim without a receipt. Refusals are recorded, never laundered. Negative
  controls must actually fail.
- Engine (`crates/`) and `tools/` source are read-only during a campaign — log
  the issue to `campaign/friction/<part>.md` instead of patching around it.
- Every campaign ships an `assembly/` folder, single-part campaigns included.
