# legacy/kernel-model-examples — pre-JSON-era Rust part generators

**Not compiled. Not tested. Not in any Cargo target.** These 32 files were
`crates/kernel-model/examples/*.rs` until 2026-09: gate-driven Rust `main`s that
build a part or a whole print campaign directly against the `kernel_brep` /
`kernel_model` Rust API, re-prove every design claim on every run, and exit
non-zero on any failed gate (the "Rust-style campaign" of `DESIGN_GUIDE.md`
§25). They predate the JSON program surface (`kernel-api run program.json`)
that every current campaign uses; the campaign folders they produced
(`spool_system/`, `hook_system/`, `camera_system/`, …) keep their own README,
gate tables and print files, so nothing that shipped depends on these sources
building.

They were moved here because they were 22k lines of code that CI compiled on
every `cargo test --workspace --all-targets` run without exercising anything the
JSON surface does not already cover, and because each one is pinned to the
kernel API as it stood on the day it last ran. Expect an old file to need small
fixes against the current API when restored. `git log --follow <file>` still
shows each file's full history (they were moved with `git mv`).

The only example still compiled and run in CI is
`crates/kernel-model/examples/parts_gallery.rs` (the manufacturing-gallery
acceptance and serialized round-trip gate).

## Restoring one

```sh
# from the repo root
git mv legacy/kernel-model-examples/<name>.rs crates/kernel-model/examples/
cargo run -p kernel-model --release --example <name>
```

Cargo auto-discovers `examples/*.rs`, so no `Cargo.toml` edit is needed.
`nullspin.rs` uses `include_str!` on its two siblings — restore
`nullspin_design.md.in` and `nullspin_listing.md.in` alongside it. Once the
run is done, move the file back here (or delete it) so CI does not pick it up
again.

## What is here

| file | lines | what it is |
|---|---:|---|
| `bench_kernel.rs` | 237 | Kernel performance baseline — five representative workloads spanning both halves of the hybrid kernel, median-of-3 timings. The numbers in `docs/BENCH.md` came from this. |
| `bracket_gen.rs` | 1898 | BRACKET GEN — a PLA wall/shelf bracket through the whole generative-design loop in one gated run: exact-B-rep baseline → ACE hex8 FEA (receipts) → SIMP topology optimisation → rebuilt part. |
| `calibrate_fdm.rs` | 1072 | CALIBRATE-FDM — the measured-reality coupon campaign behind `kernel_model::process::FdmProfile` (seven coupons; nominals shared with `tools/ingest_calibration.py`). |
| `capstan_drive.rs` | 605 | UCM-17 — universal capstan module, iteration 1: NEMA-17 + Dyneema capstan stage + drum as one rotary actuator brick. |
| `card_magazine.rs` | 949 | TWO-STATE MAGAZINE — media-card magazine where "shot"/"fresh" is the depth the card sits at; the Rust-style exemplar cited in `campaign/digests/exemplars.md` (35 gates). |
| `catalog.rs` | 327 | Standard-parts catalog acceptance: one of every part family in `kernel_model::parts` at two parameter sets, every body validated. |
| `cyclo26.rs` | 1230 | CYCLO-26 — from-scratch, gate-perfect N:1 cycloidal actuator for NEMA-17 (`lobes` in `cyclo26/params.csv` sets the ratio). |
| `cyclo26_sim.rs` | 472 | CYCLO simulator — quasi-static simulation of the 26:1 cycloidal drive (dense mesh sweep, transmission error). |
| `cyclo_drive.rs` | 190 | CYCLOIDAL DRIVE 10:1 — modular robot-joint actuator for a NEMA 17 as a hybrid assembly (exact B-rep gear train + implicit gyroid arm). |
| `drawer_system.rs` | 825 | DOVESTACK — modular dovetail drawer system (Printables Designer Challenge, July 2026). |
| `drill_hook.rs` | 2007 | DRILL HOOK — over-the-edge shelf hook for a 1.8 kg cordless drill; the campaign behind `docs/FRICTION.md` #24/#25. |
| `drybox_roller.rs` | 766 | DRYBOX ROLLER — bearing-roller + desiccant base for ~4 L flip-top "cereal keeper" containers; one of the two nightly gate suites until 2026-09. |
| `harmonic26.rs` | 980 | HARM-26 — 26:1 strain-wave (harmonic) drive in the same Cricket-class NEMA-17 envelope as cyclo26. |
| `harmonic26_sim.rs` | 286 | HARM-26 kinematic simulator — deformed-tooth verification of the strain-wave mesh at tooth-polygon level. |
| `harmonic_drive.rs` | 148 | HARMONIC DRIVE 50:1 for a NEMA 17 — a robotic-arm joint actuator assembled from real catalog parts. |
| `hybrid_showcase.rs` | 352 | Hybrid flagship: two parts that each need both halves of the kernel (ISO-thread machine bolt, lattice-cored part). |
| `nullspin.rs` | 4140 | NULLSPIN — grounded-carrier epicyclic fidget spinner, two rotors counter-rotating at an exact integer ratio (contest entry). Needs the two `.md.in` siblings below. |
| `nullspin_design.md.in` | 215 | `include_str!` template for `nullspin.rs` (design document). |
| `nullspin_gen.rs` | 4954 | NULLSPIN-GEN — the spinner of `nullspin.rs` with its held frame replaced by a topology-optimised organic web (contest entry #2). |
| `nullspin_listing.md.in` | 304 | `include_str!` template for `nullspin.rs` (Printables listing). |
| `planetary26.rs` | 1036 | PLAN-26 — 26:1 backdrivable two-stage involute planetary in the same envelope as cyclo26 / harmonic26. |
| `planetary26_sim.rs` | 269 | PLAN-26 kinematic simulator — tooth-level verification of both planetary stages with the printed involute outlines. |
| `pool_noodle_hub.rs` | 414 | NOODLEDOCK — pool-noodle hubs with machine-verified buoyancy (Printables "Pool Accessories", July 2026, entry 2). |
| `pool_staples.rs` | 453 | POOLSTAPLES — replacement-part staples for pool plumbing (entry 4). |
| `pool_tpms_basket.rs` | 601 | TPMS LEAF CATCHER — gyroid pre-filter basket for pool skimmers (entry 3, the implicit half at work). |
| `pool_tubedock.rs` | 594 | POOLDOCK — snap-on accessory docks for frame-pool rails (entry 1). |
| `respool.rs` | 1136 | RESPOOL — two-part printable reusable spool for Bambu-style 1 kg refills; one of the two nightly gate suites until 2026-09 and the source of the conservative FDM numbers in `kernel_model::process`. |
| `retract26.rs` | 690 | RETRACT26 — retractable USB-cable reel with a printed compliant spiral power spring, zero hardware. |
| `rim_saddle.rs` | 936 | RIM SADDLE — hive-tool fulcrum and box-rim protector for beekeepers. |
| `rocket_demo.rs` | 398 | Rocket thrust-chamber demo — the PicoGK-parity primitives on one part needing both kernel halves. |
| `sweeper.rs` | 364 | TRI-SWEEP — one-piece TPU-95A three-edge floor sweeper, printed flat, support-free. |
| `tri_benchmark.rs` | 156 | TRI-BENCHMARK — one assembly, three parts, three representations (exact B-rep, implicit, hybrid); the joinery-doctrine example cited in `DESIGN_GUIDE.md` §18.6. |
