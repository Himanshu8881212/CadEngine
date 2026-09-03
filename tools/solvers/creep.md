# creep — sustained-load allowable (material card, not a solver)

**Runner**: `tools/analyzers/production_check.py` (the `creep` rule) ·
`tools/analyzers/materials.py --creep <MAT> <TEMP_C> <HOURS>` (bare lookup + receipt) ·
`kernel_model::materials::pla::creep_lookup` / `creep_allowable_mpa` (Rust).
**Gates**: `python3 tools/tests/materials_crosslang_test.py` ·
`cargo test -p kernel-model --release --test materials_creep --test materials_creep_crosslang` ·
`python3 tools/analyzers/production_check.py --selftest` · `python3 tools/analyzers/materials.py --selftest`.
**Status**: green. **Tier**: a researched TABLE with per-cell confidence, not a
model — there is nothing to converge and nothing to mesh.

This card is not a solver card. It is here because the number it serves is the
one that actually kills printed parts: **sustained load is a CREEP case**, and
in the ten-campaign portfolio creep — not instantaneous strength — was the
governing failure mode in at least five parts and the single smallest margin in
two (`ball_kinematic_mirror_mount`, margin 1.064x; `folding_deck_cleat`, SF
0.0555). A static yield margin says nothing about a part held under load.

## The physics, and why it is a table

Printed PLA cold-flows at room temperature. The allowable is a function of BOTH
temperature and time held, so a creep gate needs **two** inputs a static gate
does not: the service temperature and the **design duration**. There is no
closed-form printed-PLA creep law worth trusting, so the record carries a
conservative CONSTRUCTION from published creep/rupture data (SF 2.0 on the worst
measured printed rupture, iso-deformation time-derating at 1.29/decade from the
only quantified printed creep-compliance history, every cell rounded DOWN to
0.5 MPa steps). The full derivation chain, the data anchors with URLs, the WLF /
Arrhenius constants and the per-cell confidence strings live in
`tools/materials/pla.json#creep` — **read them before designing against a cell.**

## The table (PLA, in-plane, unannealed, dry, constant load), MPa

| tier | 1 h | 24 h | 30 d | 1 y |
|---|---|---|---|---|
| **23 °C** | 7.5 | 5.0 | 3.5 | 2.5 |
| **55 °C** | 3.0 | 1.5 | **0.5** | **0.5** |

The 55 °C / 30 d and 55 °C / 1 y cells are **bounds, not measurements** — read
them as "do not design sustained load into unannealed PLA at 55 °C".

## Lookup semantics — the contract both languages implement

1. **No interpolation by default.** The table is a coarse STEP: two
   temperature tiers, *nothing between*. Temperature rounds **UP** to the next
   tier and duration rounds **UP** to the next column, so an in-between request
   always reads the WORSE cell. `creep_allowable_mpa(23.0, 8760)` = 2.5;
   `creep_allowable_mpa(23.000001, 8760)` = **0.5**. That five-fold cliff at a
   temperature nobody controls a room to is deliberate, and it is the reason
   every lookup now returns the cell it used.
   **Opt-in interpolation (Python only, 2026-09-02):**
   `creep_lookup(..., interpolate=True)` / `materials.py --creep ... --interpolate`
   / a `production_check` job with `"creep_interpolation": true` reads the
   allowable interpolated between the bracketing cells — linear in
   temperature, log-linear in duration (`materials.CREEP_INTERPOLATION_FORMULA`):
   30 °C / 24 h → 5.0 + (30−23)/(55−23) × (1.5−5.0) = **4.234375** MPa instead
   of the default 1.5. The receipt then says `basis: "interpolated"`,
   `cell_match: "interpolated"`, lists every bracketing cell with its
   confidence string, carries the formula, and reports the bucket the default
   would have read (`default_bucket_mpa`) beside it. It never extrapolates
   (above 55 °C it still refuses; below the coldest row / before the first
   column it clamps and is then *not* labelled interpolated), invents no
   measured cell, takes the LOWEST bracketing confidence, and has **no Rust
   mirror** — the cross-language pin covers the default reader only. Quote an
   interpolated allowable as interpolated, with both cells.
2. **Above the hottest tier the lookup REFUSES.** `sig_allow_mpa` = 0.0,
   `known` = false, `refused` = true, `refusal_kind` =
   `creep_temp_above_tabulated`. It does **not** fall back to the 55 °C row.
   A gate written `demand <= allowable` therefore FAILS in exactly the regime
   where no data exists, which is the intended behaviour.
3. **Non-finite, missing or negative inputs REFUSE** (`creep_input_not_finite`,
   `creep_negative_duration`). A typo never becomes a silent allowable.
4. **A material with no creep table REFUSES** (`creep_no_table`). Only PLA has
   one. The legacy blanket `yield x thermal.creep_sustained_fraction` scalar is
   reported as `legacy_scalar_mpa` for visibility and is **never** an allowable
   — see the conflict below.
5. **Beyond the last duration column the last column is reused**, flagged
   `duration_match = extrapolated_beyond_last_column`. Both languages agree.
6. **Anisotropy is the caller's explicit choice.** The cells are IN-PLANE.
   Across-layer sustained load is `x process.anisotropy.z_vs_xy_strength_ratio`
   (**0.55** for PLA), applied only when the caller passes `across_layer=True`
   (or, in `production_check.py`, when the job's `orientation` puts the primary
   load more than 30° out of the layer plane). It derates the **allowable**,
   never `E`. The factor is always on the receipt.

## Every lookup is a receipt

`materials.creep_lookup` (Python) and `pla::creep_lookup` (Rust) return, beside
the number: `temp_c_requested`, `hours_requested`, `row_used_c`, `col_used_h`,
`temperature_bucket` / `duration_bucket` (the literal cell keys), `cell_match`
(`exact` | `rounded_up_conservative` | `extrapolated_beyond_last_duration` |
`refused`), `refusal_kind`, `across_layer`, `anisotropy_factor`, the record's
per-cell `confidence` string, and the material `hash`. **"Which cell was this
margin read at" is a gateable value, not a sentence in a README** — a campaign
whose declared ambient is 25 °C can no longer write "gated against
`creep_allowable_mpa(23 °C, 1 year)`" and look compliant.

## The recorded conflict: the 0.2-fraction vs the table

`tools/material_db.json` and `thermal.creep_sustained_fraction` carry a legacy
blanket rule — "sustained = 20 % of yield", i.e. **11.0 MPa** for PLA, time-blind
and derived from the very static yield the creep rule exists to replace. The
record's own `conflicts` ledger names it, and `OPERATOR_BRIEF §7` rules on it:
**the table governs.** At 23 °C / 1 y the table says **2.5 MPa** — the legacy
number is 4.4x optimistic. Every consumer in `tools/` now reads the table, and
the scalar survives only as `legacy_scalar_mpa` next to the real allowable so
the disagreement stays visible. `materials.validate()` proves the ledger itself
has not drifted from the values the record serves.

## Benchmark gates (measured, frozen)

| gate | what it pins | where |
|---|---|---|
| 540-probe cross-language vector pin | Python and Rust return the same allowable, the same CELL and the same refusal at every tier boundary, on both sides of it, above the hot tier, beyond the last duration column, and for every non-finite/negative input | `tools/materials/creep_crosslang_vectors.json`, read by `tools/tests/materials_crosslang_test.py` **and** `crates/kernel-model/tests/materials_creep_crosslang.rs` |
| Rust table mirror | the Rust `CREEP_SIG_ALLOW_MPA` equals the researched JSON | `crates/kernel-model/tests/materials_creep.rs` |
| monotonicity | longer is never stronger, hotter is never stronger (without which "round up" is not conservative) | `materials._validate_creep_table`, enforced at record load |
| production_check creep rows | 23 °C/1 y = 2.5 MPa, 25 °C reads the 55 °C row, 70 °C refuses, missing duration refuses, across-layer 2.5 → 1.375 | `tools/analyzers/production_check.py --selftest` |

## Validity limits / out of scope

- **Only PLA has a creep table.** Every other material refuses. That is honest,
  not a gap to be filled with the 0.2-fraction.
- **Nothing measured between 23 °C and 55 °C**, and nothing above 55 °C. The
  step is real missing data, not a modelling choice. The default reader makes
  the step *visible* and refuses outside the table; the opt-in interpolation
  (semantics §1) fills the gap with a labelled MODEL between the two
  constructed rows — it is not data, and the receipt says so.
- **No printed-PLA dataset beyond ~170 h** exists at any temperature; the 30 d
  and 1 y cells are extrapolations (23 °C) or stated bounds (55 °C).
- Unannealed, dry, constant load, in-plane. Humidity coupling at 55 °C is
  flagged significant and quantified nowhere. Annealing improves creep
  resistance ~1.85x at 47 °C but has no long-duration data either.
- Creep **shear** exists only in Rust (`creep_shear_allowable_mpa`, the same 0.6
  ratio the static tier uses); it is a derived number with no independent
  printed-PLA creep-shear dataset behind it, and there is no record field for
  the ratio, so it has no Python mirror. Say so when you cite it.
- This is an ALLOWABLE, not a deformation prediction. It answers "may this hold
  that load for that long", never "how far will it sag".

## When to use it

Any load held longer than a print job: a bracket carrying books, a spool
standing in a warm dryer, a preloaded spring hook, a clamped joint, a
threaded insert under standing tension. If `load_character.sustained` is true,
`production_check.py` now **requires** `duration_h` — state the design duration
and the service temperature, and quote the cell the receipt says you read.
