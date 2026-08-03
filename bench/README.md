# bench/ — CAD Code eval harness

How the agent surface's score is **proven objectively**, not asserted. Grows every cycle.
The composite score in `PROGRESS.md` is only allowed to move when a bench artifact backs it.

## Three layers
1. **Boundary tests** (Rust, `crates/kernel-api/tests/`) — contract (schema matches the
   `OpKind` enum), fuzz-the-boundary (malformed ops, absolute/`..` paths, huge inputs
   must be REFUSED not crash), and API-level determinism (same request → byte-identical
   report). These gate CI.
2. **Agentic-CAD scenarios** (`bench/scenarios/`) — representative "design task → the
   harness must let an agent complete it end-to-end through the API." Scored pass/fail.
3. **Scorer** (`bench/score.*`) — rolls the layers into the per-dimension + composite
   score using the rubric in `PLAN.md`'s definition-of-9.

## Scenario backlog (north-star acceptance in bold)
- `s01_bolt_circle` — add 4 counterbored holes on a bolt circle; verify `min_ligament` ≥ spec. *(needs M2 discovery, M4 measure)*
- `s02_param_edit` — tweak one param, re-measure mass; recompute must be incremental. *(needs M3)*
- `s03_reroute_self_intersect` — a self-overlapping union must be REFUSED/flagged, then rerouted. *(needs M1)*
- `s04_query_then_fillet` — `list_faces` → fillet a returned edge ref (no guessed witness). *(needs M3 introspection)*
- **`s05_five_part_assembly`** — an agent, given only the API, decomposes a ≥5-part
  assembly, fans a sub-agent per part, each passes the engine's gates, and the whole
  validates: **no crash, no silent-wrong result, full provenance on every number.**
  This is the ship gate.

## Status
Skeleton. Boundary tests land first (cycle 1+ under M0). Scenarios follow their milestones.
