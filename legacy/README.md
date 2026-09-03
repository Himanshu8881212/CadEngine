# legacy/ — parked code that is NOT built

Everything under this directory is kept for its history and for the day it is
wanted again, but **no Cargo target, CI job, test, tool or campaign depends on
it**. `cargo build --workspace` never sees it. Each folder was moved here with
`git mv`, so `git log --follow` still shows its full history.

| folder | what it is | last known state |
|---|---|---|
| `kernel-model-examples/` | the 32 pre-JSON-era Rust part generators and campaign gate suites that used to be `crates/kernel-model/examples/*.rs` | see its own [README](kernel-model-examples/README.md) |
| `kernel-gpu/` | `kernel-gpu`: wgpu/WGSL evaluation of the implicit half (GPU field codegen mirroring every CPU distance formula, GPU Surface Nets, narrow-band extraction). CPU stays bit-authoritative; the GPU path is a tolerance-equivalent preview — `docs/NUMERICS.md` §"GPU evaluation and extraction" states the contract. 3.9k lines, 18 tests that need a GPU adapter (they skip loudly without one), `examples/bench_gpu.rs` behind `docs/BENCH.md`'s GPU table | built and green on Apple M3 / Metal at the 2026-07-30 narrow-band wave; nothing in `crates/`, `studio/`, `tools/` or CI referenced it when it was parked (2026-09) |
| `kernel-wasm/` | `kernel-wasm`: the `wasm-bindgen` surface (`demo()` builds a hybrid part and returns its mesh) plus `web/index.html` + `web/viewer.js`, a three.js viewer for the `wasm-pack` output. 195 lines, no tests | compiled on the host at the 2026-06 waves; `studio/web` never used it (the studio runs the kernel server-side); parked 2026-09 |

## Why they were parked

- `kernel-gpu` pulled `wgpu` (and its ~100 transitive crates) into every
  workspace build and clippy run for a preview path no shipped surface calls.
- `kernel-wasm` had no consumer: the studio talks to `studio-server` over
  HTTP, and the release archive ships native binaries only.
- The example campaigns are covered in `kernel-model-examples/README.md`.

## Restoring a crate

```sh
# from the repo root
git mv legacy/kernel-gpu crates/kernel-gpu          # or legacy/kernel-wasm
```

Then add the crate back to `members` in the root `Cargo.toml` and, for
`kernel-gpu`, restore its workspace dependency pins there:

```toml
[workspace.dependencies]
wgpu = "26"
bytemuck = "1"
pollster = "0.4"
```

(`kernel-wasm` pins `wasm-bindgen = "0.2"` in its own `Cargo.toml`.) Run
`cargo build -p kernel-gpu` (or `-p kernel-wasm`) and expect small fixes
against whatever moved in `kernel-core` / `kernel-implicit` since the crate
was last built. `Cargo.lock` regenerates the dropped entries on the next cargo
invocation; commit it. Nothing else — no CI workflow, tool or doc audit —
needs editing to bring a crate back.
