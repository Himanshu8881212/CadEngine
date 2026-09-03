# `profiles/` — measured manufacturing process profiles

Machine-readable printer/process reality, consumed by campaigns through
[`kernel_model::process`](../crates/kernel-model/src/process.rs) instead of
clearance constants retyped per campaign.

Each `*.json` here is one `FdmProfile`: the clearances, compensations,
bridging/overhang/wall limits and bed envelope of **one printer + material +
settings combination**. Campaigns call the fit helpers
(`fit_free_bore_d`, `fit_tight_shaft_r`, `hole_d`, `bridge_ok`, `wall_ok`,
`bed_fits`) so a design re-targets to a different printer by loading a
different profile — no geometry edits.

## Files

| file | what it is |
|---|---|
| `conservative_default.json` | the research-derived fallback: exactly the numbers the shipped RESPOOL / DRYBOX campaigns froze as consts and proved in print. Regenerated on every `calibrate_fdm` run; per-field provenance in `process.rs` doc comments and `calibration_system/fdm_coupons/analysis/DESIGN.md`. **Not a measurement.** |
| `<printer_name>.json` | a MEASURED profile written by `tools/ingest_calibration.py` from your caliper readings of the calibration coupons |

## Getting a measured profile (10 minutes of caliper work)

```sh
# calibrate_fdm.rs was removed from the tree on 2026-09-03; recover it first:
#   git show 5a70984:legacy/kernel-model-examples/calibrate_fdm.rs \
#     > crates/kernel-model/examples/calibrate_fdm.rs
# then:
cargo run --release -p kernel-model --example calibrate_fdm   # builds the coupons
# print calibration_system/fdm_coupons/parts/ (one plate, ~43 g)
# measure per calibration_system/fdm_coupons/README.md
cp calibration_system/fdm_coupons/measurements.example.json \
   calibration_system/fdm_coupons/measurements.json
# fill in every PLACEHOLDER, then:
python3 tools/ingest_calibration.py calibration_system/fdm_coupons/measurements.json
# -> profiles/<printer_name>.json
```

The ingest tool refuses loudly (`{"ok": false}`, exit 1) on missing,
placeholder, or self-inconsistent measurements rather than writing a profile
it cannot defend.

## Using one in a campaign

```rust
use kernel_model::process::FdmProfile;

// measured if the user has one, honest fallback otherwise
let p = FdmProfile::load("profiles/my_printer.json")
    .unwrap_or_else(|_| FdmProfile::conservative_default());

let bore_d  = p.fit_free_bore_d(6.0);   // designed bore for a free-running Ø6 pin
let shaft_r = p.fit_tight_shaft_r(4.0); // designed press-stub radius in a Ø8 bearing
assert!(p.bridge_ok(span) && p.wall_ok(rib_t));
```

## Schema

Field-for-field the Rust `FdmProfile` struct (`deny_unknown_fields`: a typo'd
key is a hard load error, never a silent default). All lengths mm, angles
degrees; radial-vs-diametral and sign conventions are stated per field in the
`process.rs` doc comments and echoed by the ingest tool's output.

Both writers — `FdmProfile::save` (Rust) and `ingest_calibration.py` (Python)
— emit the same fixed field order with a trailing newline, so profiles are
diffable and byte-stable across regeneration. The format is pinned by
`crates/kernel-model/tests/process.rs::profile_json_snapshot_byte_stable`.

## Other processes

Sheet metal, casting and CNC are **declared siblings** in
`kernel_model::process::Process` with no profiles yet: requesting one returns
a loud `NotImplemented` error (the casting refusal points at
`kernel_brep::draft_analysis`, the castability check that does exist). When
those land, their profiles live here too.
