# `campaign/friction/` — the friction logs

A friction log exists for one reason: **so the next model does not lose the same
day twice.** Every entry names a symptom, a minimal repro, expected-vs-actual,
and the workaround actually used — and is either dispositioned (fixed, with the
fix named) or left open on purpose.

| file | scope |
|---|---|
| [`ENGINE.md`](ENGINE.md) | **the engine-wide log.** The long-running record of kernel and surface friction, opened by the dogfood gearbox and carried forward wave by wave: numbered items (#1, #2, …) with a `STATUS:` line each, plus the open frontier. This is the one to read before you conclude the engine is broken — and it moved here from `docs/FRICTION.md` on 2026-09-03, because a friction log is an operating rule, not a description of the engine |
| every other `*.md` | **per-campaign logs**, one per part, named for the campaign (`uphill_roller.md`, `rated_desk_hook.md`, …). Written *during* that campaign under DELIVERABLE_SPEC §4, with the resolutions appended when a fix round closes them |
| `_digest_phase_findings.md` | the one non-campaign exception: friction the digest readers found while checking the docs against the binary |

## Writing one

During a campaign, `crates/` and `tools/` are read-only. When the engine or a
tool fights you, the deliverable is an entry in `campaign/friction/<part>.md`,
not a patch. Use the shape the existing files use:

```
## F<n> — one-line symptom (YYYY-MM-DD)
- severity: blocker | major | minor | papercut
- surface: <one op name, tool path, or named surface — the rollup key>
- status: open | fixed — <what fixed it>
- symptom: the exact error text or the wrong number
- minimal repro: the smallest program/job that shows it
- expected vs actual: what the doc/brief promised, what the binary did
- workaround used: what you actually shipped, and its cost
- recurrence of: <file>#<id>   ← only when you re-hit a known item
```

The three machine-read fields (`severity`, `surface`, `status`) are the
contract; the prose under them is the evidence. Definitions, in full, are
DELIVERABLE_SPEC §4 — in one line each:

| severity | means |
|---|---|
| `blocker` | could not proceed without a workaround that WEAKENS a shipped claim |
| `major` | wrong / silent / missing behaviour, worked around, no claim weakened |
| `minor` | cost time, no claim affected |
| `papercut` | ergonomics only: message text, `--help`, naming, layout |
| `note` | not friction — context kept for balance; excluded from index counts |

`status` is `open`, `partial`, or `fixed — <the fix>`.

`fixed` is a `status`, never a severity: a closed item keeps the grade it
earned, because the grade is the record of what it cost. `surface` is one
token — the op as spelled in program JSON, the tool's real path (not the
`tools/*.py` shim), or a named surface like `kernel-api cli` — and it is what
`docs/FRICTION_INDEX.md` groups on, so spelling it a new way hides a
recurrence. `python3 tools/friction_index.py --surfaces` prints the tokens
already in use; `--surface <token>` prints every item filed against one.

An entry that a later fix round closes gets a `RESOLUTIONS` section appended
naming the fix — the original entry is never edited away. Anything general
enough to bite every campaign is promoted into `ENGINE.md` with a number.
