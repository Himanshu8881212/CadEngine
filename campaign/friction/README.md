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
- symptom: the exact error text or the wrong number
- minimal repro: the smallest program/job that shows it
- expected vs actual: what the doc/brief promised, what the binary did
- workaround used: what you actually shipped, and its cost
```

An entry that a later fix round closes gets a `RESOLUTIONS` section appended
naming the fix — the original entry is never edited away. Anything general
enough to bite every campaign is promoted into `ENGINE.md` with a number.
