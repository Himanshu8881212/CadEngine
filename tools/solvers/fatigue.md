# fatigue — stress-life (S-N) cyclic damage for PRINTED parts

- **Runner**: `tools/analyzers/ace_fatigue_runner.py` · **Gates**: `tools/tests/test_ace_contact_fatigue.py` · **Data**: `tools/materials/fatigue.json` (this solver's own researched registry; it never edits the material records) · in-house (NumPy only)
- **BLUNT VALIDITY STATEMENT (read before quoting any number)**: fatigue of FDM parts is dominated by **layer adhesion and process defects**, not by the bulk polymer — the same review that reports FDM ABS failing at 6.0e4 cycles reports injection-moulded ABS at 6.0e6 cycles for the SAME 10 MPa (100x). This is a **COMPARATIVE / SCREENING** tool for ranking design variants and rejecting overstressed features. **It is not a certification basis.** The published 90%/10%-survival band for printed PLA spans **3.7x to 90x in LIFE** (recomputed from the source table into every receipt), so a single predicted cycle count without that band is misleading.
- **Physics**: stress-life. Basquin `sigma = a N^b` -> `N = (sigma/a)^(1/b)`; mean-stress correction; Palmgren-Miner linear damage `D = sum n_i/N_i`, failure at `D = 1`. It is a POST-PROCESSOR: it consumes a static stress field (normally `stress_field.npy` from `tools/analyzers/ace_fea_runner.py`, per-element von Mises in Pa) or a scalar hot-spot stress, and a declared cycle spectrum.
- **Mean stress** (`mean_stress`, named in the receipt): `goodman` `sigma_ar = sigma_a/(1 - sigma_m/sigma_u)`, `gerber` `sigma_ar = sigma_a/(1 - (sigma_m/sigma_u)^2)` (both Shigley ch. 6; compressive mean gets NO credit — that branch is unvalidated for printed polymers and taking it would be unconservative), `none`, and `intrinsic` `sigma_eff = sigma_max`. **`intrinsic` is the only one validated on printed specimens**: Ezeh & Susmel 2019 Fig. 8a show the 2e6-cycle endurance limit of AM PLA expressed in MAXIMUM cycle stress is invariant (9.5 MPa +-2SD) across R = -1, -0.5, 0, +0.3 — the max stress already carries the mean effect. Stacking Goodman on top of a max-stress curve is double counting and is REFUSED.
- **Damage rule**: Palmgren-Miner (Miner 1945). Explicitly **not validated for printed polymers** — no variable-amplitude printed dataset was found; sequence effects are unmodelled. `D = 1` is a nominal threshold, and the receipt says so.

## The data situation, stated honestly (`tools/materials/fatigue.json`, researched 2026-07-30)
| material | status | what exists | runner behaviour |
|---|---|---|---|
| **PLA** | `measured`, confidence **Medium** | Ezeh & Susmel, *Int. J. Fatigue* 126 (2019) 319-326 — 143 printed specimens, R = -1/-0.5/0/+0.3, raster 0/30/45 deg, 10 Hz, runout 2e6, per-configuration `k` and `sigma_A,50%` + the paper's own design curve `k=5.5`, `sigma_MAX = 0.1 sigma_UTS at 2e6`. Corroborated INDEPENDENTLY by *Polymers* 18(1):1 (2026) smooth-specimen fit `sigma_a = 68.159 N^-0.161` (R^2 0.991, R=0.05, 5 Hz, measured UTS 37.23 MPa): the Basquin coefficient A agrees to **0.42%** (68.159 vs 68.446 MPa) and the R=-1-equivalent 2e6 amplitude to 21%. | returns a life number + the scatter band + every gap |
| **PETG** | `insufficient` | ONE study (JFAP 2019), coefficients paywalled, lives top out ~2e4 cycles at 60-90% UTS — under two decades, single lineage | **REFUSES** with the named reason and "what would change this" |
| **ABS** | `insufficient` | one review lineage: ~6e4 cycles at 10 MPa (vs 6e6 injection-moulded) + a raster-orientation ranking. One point cannot fit a slope. | **REFUSES** by name |
| **ASA / PA / PC / TPU95A** | `unknown` | nothing usable found; two candidate papers could not be retrieved (CAPTCHA) and are recorded as LEADS, not sources | **REFUSES** by name |
| **across-layer (Z) loading, ANY material** | `unknown` | no printed S-N dataset in ANY orientation normal to the layers | **REFUSES** `load_orientation:"across_layer"` — a STATIC `z_vs_xy_strength_ratio` is not a fatigue-slope ratio |
Canonical PLA `sigma_UTS = 40.9 MPa` (the LOWEST measured printed UTS in the source), deliberately **not** `pla.json` `ultimate_mpa = 55.0` (datasheet-class; using it would raise the design endurance 34% and soften every Goodman correction). Conflict recorded, record untouched. A second conflict is recorded and NOT averaged: rotating-bending PLA (100 Hz, no measured UTS) gives 6.2x longer life at 14 MPa with a much steeper slope; axial is canonical because a bending gradient over-reports strength relative to the uniform stress an FEA von Mises field represents.

## I/O contract (JSON manifest -> damage .npy + JSON receipt on last stdout line)
```
{out_dir,
 material: "PLA" | {name, sigma_uts_mpa, curve:{a_mpa, b, stress_measure:"amplitude"|"max", n_valid?}},  # inline = stamped "inline (caller-supplied)"
 curve?: "design" (DEFAULT, PS >= 90%) | "median" (PS 50% — never a default: half the parts fail first),
 load_orientation?: "in_plane" (default) | "across_layer" (REFUSED),
 stress: {npy, unit?:"pa"|"mpa", reference_load?} | {sigma_ref_mpa} | {sigma_ref_pa},
 spectrum: [{name?, cycles, load_factor?, r_ratio?} | {name?, cycles, sigma_a_mpa, sigma_m_mpa?}],   # von Mises is UNSIGNED -> r_ratio must be declared (default 0.0)
 mean_stress?: "goodman"|"gerber"|"none"|"intrinsic", sigma_uts_mpa?, damage_limit?: 1.0}
```
Outputs (field mode): `damage_field.npy` float32, same shape as the input field. Receipt: per-block `sigma_a/sigma_m/sigma_max/sigma_effective`, `cycles_to_failure_at_critical`, `damage_max`, `zero_amplitude`, `extrapolated_beyond_curve_validity`; `damage{total_at_critical_location, life_status, spectrum_repeats_to_failure, cycles_to_failure, critical_index}`; a **`confidence`** block (data confidence + basis, PS, mean-stress model and whether it is validated for printed polymers, Miner-validated = false, extrapolation flag, the recomputed 90/10 LIFE band, and a plain-English statement); plus the registry's `conflicts` and `gaps_unknowns` verbatim. Failure/refusal = `{ok:false, error}` + **exit 1**.

## Benchmark gates (measured 2026-07-30, all green)
| gate | closed form | measured | band asserted |
|---|---|---|---|
| 5a Miner, 2 blocks | curve `sigma_a = 100 N^-0.1` -> `N1 = 2^10 = 1024`, `N2 = 4^10 = 1048576`; `D = 100/1024 + 10000/1048576 = 0.1071929931640625` (exact dyadic) | `D` bit-identical, `N1` 1024.000000, `N2` 1048576.0000, repeats 9.328967971530 | rel err **0.00e+00** (gate 1e-15) |
| 5b Basquin round-trip | plant `a=137.5`, `b=-0.0917`, 12 lives over 4.5 decades, refit | a err 1.24e-15, b err 1.51e-16, R^2 = 1.000000000000000 | <= 1e-12 (mission asked 1e-6) |
| 5c Goodman / Gerber | `12/(1-8/40) = 15.0`; `12/(1-(8/40)^2) = 12.5` (sigma_u 40 MPa) | 15.000000000000000 / 12.500000000000000, and the resulting lives match to 1e-12 — verified END-TO-END through the CLI, so a wrong correction cannot hide inside the life number | <= 1e-12 |
| 5d PLA registry | `a = 4.09 x (2e6)^(1/5.5)`, `b = -1/5.5` from the stored primitives | `a = 57.195929924926574` MPa, `b = -0.18181818181818182` bit-exact; life at `sigma_max = 6` MPa = 243039.2 cycles; the runner re-derives `(a,b)` from `(k, sigma_at_n_ref, n_ref)` every load and refuses on drift | exact equality; life <= 1e-14 |
| 5d scatter band | `T_life = T_sigma^k` over the 12 source rows | **3.695x .. 90.39x** (best row k=7.7/T=1.185, worst k=5.8/T=2.174) | recomputed from data, quoted in every receipt |
| 6 refusals | — | PETG/ABS `insufficient`, TPU95A `unknown`, `across_layer`, Goodman-on-max-curve (double count), peak stress above the printed UTS (static failure) — all exit 1 with a pointed, named reason | exit != 0 |
| 6 zero amplitude | Basquin gives `N = inf` at `sigma_a = 0` | `life_status:"no_damage"`, `D = 0.0`, `cycles_to_failure: null`, per-block `zero_amplitude: true`, exit 0 | explicit status, NOT a silently-huge integer |
| 7 meta-control | flip the Basquin exponent sign in a scratch copy | 5 red gates (both Miner gates, both mean-stress gates, plus a raise) | suite must go RED |

## Validity limits / out of scope
- Screening only (see the blunt statement above). No notch/`Kf` factor (the source measures a 16-29% knockdown for edge notches and holes — apply it yourself, by hand, and say so), no surface-finish factor, no size factor.
- 100%-infill, room-temperature, dry, unaged, 5-10 Hz, IN-PLANE data only. Sparse infill (source 2: ~40x life loss from 100% to 25%), elevated temperature (PLA `softening_c` is 55 C), high frequency (self-heating) and moisture are all OUT of the calibration set.
- **Nothing beyond 2e6 cycles**: any block whose predicted life exceeds the curve's `n_valid` is flagged `extrapolated_beyond_curve_validity` and downgrades the receipt's confidence. There is no evidence of a true endurance limit for printed PLA.
- Von Mises input is UNSIGNED, so the R-ratio is a DECLARATION, not an inference; block scaling assumes the source solve is linear elastic. Multiaxial/non-proportional loading and crack growth (`da/dN`) are not modelled.
- TPU-class elastomers would need hyperelastic stress and self-heating even with good data — the registry says so rather than pretending the model would apply.

## When to use
Repeated-actuation questions on printed parts: a snap-fit that clicks 500 times (drive it with `tools/solvers/contact.md`, take `path_max.abs_stress_pa`, feed the spectrum here), a drybox roller shaft under a rotating load, a bracket cycled by a printer's motion. Also the honest **stop sign**: if the material is PETG/ABS/ASA/PA/PC/TPU, or the cyclic principal stress is across the layers, this solver refuses — that refusal is the deliverable, and the fix is to redesign or to measure.

Run: `python3 tools/analyzers/ace_fatigue_runner.py job.json` · prove: `python3 tools/tests/test_ace_contact_fatigue.py`
