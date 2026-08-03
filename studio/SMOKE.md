# Studio smoke — executed 2026-06-11

Every number below is from a real run on this machine (macOS, release build,
server started fresh with `ANTHROPIC_API_KEY` **unset**). Transcript captured
to `/tmp/smoke_transcript.txt` during the run; reproduced verbatim here.

Setup: `cd studio/web && npm ci && npm run build` (clean install: **0
vulnerabilities, `tsc -b && vite build` ✓ built in 1.37s**), then
`./target/release/studio-server` from the repo root.

## 1. Catalog

```
$ curl -s http://127.0.0.1:7878/api/catalog
families: 34 | first: spur_gear
```

**34 families**, schemas asserted sane in-tests (unique ops, defaults within
bounds/options, spur_gear builds from its own defaults).

## 2. Part load — `gearbox/parts/spacer_21.lmcpart`

```
$ curl -s -X POST .../api/part/load -d '{"path": "gearbox/parts/spacer_21.lmcpart"}'
name: spacer_8x12_21 | dims: [{'name': 'len', 'value': 21.0}]
features: [('Cylinder', 'tube'), ('Cylinder', None), ('Boolean', 'bored')]
receipt: volume 1211.2724635180803 exact | route exact | tris 398 | watertight True
artifact: /api/mesh/spacer_21.stl?session=default
```

(Note: `shaft_input.lmcpart` also loads cleanly — six features with labels,
BOM meta — but its Dims table is **empty** (all dimensions are literals in
its feature tree), so the PARAMS demo uses this spacer, which has a real Dim.)

## 3. set_dim — the params-panel contract (`len` 21 → 30)

```
$ curl -s -X POST .../api/part/set_dim -d '{"path": "gearbox/parts/spacer_21.lmcpart", "dim": "len", "value": 30}'
dim: len 21.0 -> 30.0
volume_before: 1211.2724635180803 -> volume_after: 2118.3130944625236 ( exact )
route: exact | tris: 248 | watertight: True
```

**Volume delta receipt: 1211.272 mm³ → 2118.313 mm³ (both `exact`,
π-exact bore via surface tags).** A follow-up load confirms persistence:

```
dims now: [{'name': 'len', 'value': 30.0}]
```

The edit lands in the actual repo file; after the smoke it was restored with
`git checkout -- gearbox/parts/spacer_21.lmcpart` (0 residual changes).

## 4. Mesh fetch

```
$ curl -s -o mesh.stl -w '...' ".../api/mesh/spacer_21.stl?session=default"
HTTP 200, 12484 bytes, content-type model/stl
binary STL: 12484 bytes, 248 triangles, well-formed: True   (84 + 50·248 = 12484)
```

## 5. Work order through /api/run (the chat demo shape, run directly)

The "30 mm cube with a Ø10 through-hole" the chat demo asks for:

```
ok: True | exact_volume: {'exact_volume': 24643.805509807644}
stl: {'route': 'exact', 'triangles': 1408, 'watertight': True}
artifacts: ['cube_bored.stl']
```

Closed form 27000 − π·5²·30 = 24643.8055… — matches the kernel receipt; the
`assert {genus: 1, valid: true}` gate in the program passed.

## 6. Chat — no-key path (executed) and live loop (pending key)

`ANTHROPIC_API_KEY` was **not present** in this environment. The no-key path,
verbatim SSE from the live server:

```
event: chat_disabled
data: {"message":"chat disabled — set ANTHROPIC_API_KEY"}

event: done
data: {"stop_reason":"chat_disabled"}
```

**The live tool loop (`claude-opus-4-8` + `run_work_order`) was NOT executed —
no key was available. Marked "pending key", not faked.** The loop's pieces
that don't need the network are covered: the tool executor is the same
in-process `run_program` path proven in §5, and the SSE plumbing is the same
channel the no-key event above traveled. When a key is present: start the
server with it, ask "make me a 30 mm cube with a Ø10 through-hole", and the
expected stream is `thinking*`, `text*`, `tool running/done`, `refresh`
(viewport reload), `done`.

## 7. Suite + lint gates

```
cargo test --workspace --release   → 683 passed, 0 failed   (677 pre-existing + 6 studio-server)
cargo clippy --workspace --all-targets --release → 0 warnings/errors
cd studio/web && npm ci && npm run build         → ✓ (from a clean install)
```
