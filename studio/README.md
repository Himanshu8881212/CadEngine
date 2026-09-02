# LMCAD Studio — Wave IDE-1

The chat-driven CAD IDE shell over the LMCAD hybrid kernel: a single window
with a chat panel (the AI designs by running real kernel work orders), a
three.js viewport showing the part as it's built, a live code pane showing the
model source, and PARAMS/PARTS panels for direct human editing. The kernel
runs **in-process** inside the server — no subprocess, no second build path.

```
studio/
  server/   Rust (axum) workspace member `studio-server` — the whole backend
  web/      Vite + React + TS + three.js front-end (built into web/dist)
```

## Run it

```bash
# 1. Build the front-end (one-time, and after web/ changes)
cd studio/web && npm ci && npm run build && cd ../..

# 2. Start the server (serves the UI + API on one port)
ANTHROPIC_API_KEY=sk-ant-…  cargo run -p studio-server --release
# → http://localhost:7878
```

Without `ANTHROPIC_API_KEY` everything except chat works; the chat panel shows
the explicit "chat disabled — set ANTHROPIC_API_KEY" event. `STUDIO_ADDR`
overrides the listen address; `LMCAD_ROOT` overrides the repo root (defaults
to the working directory — run from the workspace root).

Studio is loopback-only by default. A non-loopback bind refuses unless both
`CADCODE_ALLOW_REMOTE=1` and a non-empty `CADCODE_API_TOKEN` are set. In the web
UI, use **AUTH** to enter that bearer token; it is kept in `sessionStorage` for
the current tab only and is attached to JSON, SSE, mesh display, and download
requests. Put any remote deployment behind TLS and network/container isolation.
`CADCODE_COMPUTE_CONCURRENCY` bounds CPU jobs, including work that outlives an
HTTP timeout.

## The demo script (Wave IDE-1 acceptance path)

1. **Open a recipe** — MODEL tab → quick-chip `spacer_21.lmcpart` (or type any
   repo-relative `.lmcpart` path). The viewport frames the part; the receipt
   strip shows the kernel's numbers (volume/route/tris/watertight); the code
   pane shows the live `.lmcpart` source. `shaft_input.lmcpart` also loads —
   note it has **zero Dims** (its dimensions are literals in the feature
   tree), which the PARAMS tab states honestly; use the spacer/key parts for
   parametric play.
2. **Drag a Dim** — PARAMS tab → `len` slider. Each release fires one
   `set_dim`: the kernel rebuilds, the recipe file on disk is updated
   (canonical bytes), the mesh and the before/after volumes refresh from the
   kernel's receipt. This *edits the real file* — `git diff` shows the edit.
3. **Insert hardware** — PARTS → pick a family (34, all real catalog ops) →
   INSERT runs `part → exact_volume → export_stl` as an ordinary work order;
   the code pane shows exactly what ran.
4. **Chat** (needs the key) — "make me a 30 mm cube with a Ø10 through-hole".
   Claude answers with THINKING blocks, tool status lines
   (`run_work_order · running → done · N ops · ok`), the work order lands in
   the code pane, and every exported STL refreshes the viewport.
5. **EXPORT** downloads the current STL. IMPORT/SHARE are visible but Wave 2.

## API surface (all JSON; see `server/src/lib.rs` docs)

| route | what |
|---|---|
| `POST /api/run` | execute a work order; full kernel report + artifact URLs |
| `GET /api/mesh/{path}?session=` | exported STL/STEP/3MF binaries |
| `POST /api/part/load` | `.lmcpart` → Dims, features, configs, viewport mesh + receipt |
| `POST /api/part/set_dim` | edit one Dim → rebuild → save → fresh mesh + before/after volumes |
| `POST /api/part/save` | validated round-trip write of an envelope |
| `GET /api/catalog` | the 34 standard-parts families + param schemas |
| `POST /api/chat` | SSE: Claude operator loop (`claude-opus-4-8`, `run_work_order` tool) |

## Tests

`cargo test -p studio-server --release` — six in-process router tests
(tower `oneshot`, no network): run+mesh smoke, kernel-failure surfacing,
traversal rejection, part load/set_dim/save round-trip with exact-volume
assertions, catalog schema + defaults-actually-build, and the chat no-key
graceful path. The live chat loop is exercised manually (`SMOKE.md`).

## Wave 2 candidates (honest cut list)

Feature-tree editing (suppress/insert/labels) from the MODEL tab; undo/redo
wired to `DocumentHistory`; `.lmcasm` assemblies in the viewport (instances +
mates + exploded states); IMPORT (STL/STEP upload → `MeshSdf` bridge) and
SHARE; chat-session persistence + multi-session UI; STEP/3MF export buttons;
a wasm mode running the kernel client-side.
