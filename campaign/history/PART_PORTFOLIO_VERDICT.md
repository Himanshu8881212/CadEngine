# PART PORTFOLIO VERDICT — 10 parts, LMCAD CAD engine

**Scope.** Ten independent design campaigns, one per domain, each built to
`campaign/DELIVERABLE_SPEC.md`, then subjected to an independent hostile
verification pass and a stage-5 repair pass (2026-08-07 → 2026-08-08).

**Scale.** 648 receipt JSONs, 34 print STLs, 29 exact AP203 STEP files,
127 engine/tool friction entries across 10 friction files (+13 from the
pre-campaign digest phase), 144 repair items applied, 12 findings refused or
partially refused with counter-evidence, 47 items carried forward as
explicitly-open.

**The one-line verdict.** Every campaign shipped geometry that was correct and
documentation that was not. All 19 blockers found by verification were defects
of *analysis provenance* — stale literals, unasserted measures, claims quoting
receipts that said something else — and **zero were geometry defects**. After
repair, 9 of 10 campaigns are clean against the full spec; the tenth ships one
named, reasoned spec gap rather than an invented number.

---

## 1. Portfolio verdict table

| part | domain | rebuild clean | blockers found | blockers fixed | still open | headline receipt — the single most load-bearing measured number |
|---|---|---|---|---|---|---|
| `cubesat_1u_dev_frame` | aerospace | ✅ 43/43 gates, 0 warnings | 0 (7 major) | — | 3 | **`design_critical_load_n` = 322.7316 N** (λ₁ = 0.5378860 at a 1200 N reference → `critical_load_N` 645.463, ×0.5 mandatory knockdown) vs the 1200 N CDS dispenser-stack gate — **misses by 3.72×**. `receipts/buckling_frame.json:knockdown.design_critical_load_n` (verified this pass) |
| `prosthetic_wrist_quick_disconnect` | biomedical | ✅ | 2 | 2 | 5 | **0.03672 N·m** index hold, the maximum reachable anywhere in the box — the literal 1.5 N·m requirement returned `constraint_ok: false` after 44 evaluations across 4 Nelder-Mead starts. **41× short**; the part is re-scoped from locking wrist to indexing aid. `receipts/opt_A_card_literal.receipt.json`, `opt_B_achievable.receipt.json` |
| `iso9409_wedge_flexure_gripper` | robotics | ✅ | 1 | 1 | 3 | **fatigue SF 0.682, `pass: false`** at the blade root (allowable 18.0 MPa, demand 26.3966 MPa) → N = 37.4 cycles, *below the S-N curve's own 500-cycle validity floor*, so no life may be quoted at all. `receipts/production_check_blade_root.json` (verified this pass) |
| `din_rail_pi4_enclosure` | electronics | ✅ | 3 | 3 | 5 | **1.0029717 mm** of latch-nose engagement measured on the **exact B-rep** at the worst-case ±0.15 mm FDM stack, against a 1.0 mm spec (the closed-form chain gives the looser 1.0050 mm; the exact measure is the one published). `receipts/optimize_latch_receipt.json:best_measures.geom.engagement_worst_mm` (verified this pass) |
| `rotor_runout_gauge_bridge` | automotive | ✅ (crates/ dirt pre-dates campaign) | 1 | 1 | 6 | **`tip_displacement` 0.0265897 mm** at the outermost bore station under the 2.0 N measuring force, vs a 0.00712 mm gate → the gauge is only accurate out to **y ≈ 113.3 mm**, not to the 144.6 mm it reaches. `receipts/fea_L1_c_run.json:tip_displacement_m = 2.658970508e-05` (verified this pass) |
| `folding_deck_cleat` | marine | ✅ | 1 | 1 | 6 | **5.6 N sustained at 40 °C / 24 h** — the top of the declared service envelope. Creep cell `sig_allow_mpa[55C][24h]` = 1.5 MPa, ×0.55 across-layer = 0.825 MPa. The concept asked for 100 N sustained: **LC2 fails at SF 0.0555 (40 °C) and SF 0.1850 (23 °C)**. `receipts/handcalc_stage3.json:gates.lc2_sustained_creep_design` (verified this pass) |
| `ball_kinematic_mirror_mount` | optics | ✅ 77 receipts, 0 warnings | 4 | 4 | 3 | **3.290 MPa vs 3.500 MPa = margin 1.064×** on `L7-bearing` (spring hook on the 3×3 mm printed anchor bar, 23 °C / 30 d creep). The smallest number in the campaign, and it is a *creep* number, not a strength number. `receipts/creep_gates.json:worst_row/worst_margin = 1.0639` (verified this pass) |
| `turgo_runner` | energy | ✅ | 2 | 2 | 6 | **fatigue SF 0.7028, `pass: false`** on the *refined* mesh (allowable 9.9 MPa, demand 14.0874 MPa) — the third of three shipped fatigue failures, and the refined reading is **higher** than the coarse one, so the quoted peak is not an upper bound. `receipts/production_check_stall_mesh_refined_bound.json` (verified this pass) |
| `jar_top_seed_singulator` | agriculture | ✅ | 2 | 2 | 5 | **0.8067 mm** worst singulation margin at the **simultaneous adverse corner** (`acc_pocket_adv`), replacing the 1.1064 mm own-corner headline that was **37.1 % optimistic**. All four margins still positive. `receipts/opt_chosen_point.json:adverse_corner.worst_margin_adverse` (verified this pass) |
| `screw_on_exponential_horn` | acoustics | ⚠️ **154/154 gates pass, but SPEC §5.6 NOT met** — one mating interface (M4 heat-set vs its Ø5.6 printed pilot) has no tolerance stack, because no source publishes a pilot-bore tolerance and an invented band returning `ok:true` is worse than a named gap | 3 | 3 | 5 | **SF 1.41 → 0.28.** The bracket mounting-face outer edge is the tightest sustained margin: SF 1.41 against the 1-year 23 °C across-layer creep cell — but the campaign's own declared ambient is **25 °C**, and `creep_allowable_mpa` steps to the 55 °C row above 23 °C, giving **SF 0.28**. Published as a condition limit with a gate that *asserts the elevated SF stays below 1.0* so it can never quietly become a pass. `receipts/` creep table gates + `tools/materials/pla.json` |
| **TOTALS** | 10 domains | **9 clean / 1 named gap** | **19** | **19 (100 %)** | **47** | — |

Notes on the table:
- "Blockers" = findings the verifier classified BLOCKER (would invalidate a
  headline claim if shipped). `cubesat_1u_dev_frame` had none; its worst seven
  were MAJOR. By the repair records' own severity labels the portfolio total is
  **19 BLOCKER + 52 MAJOR + 60 MINOR = 131 findings**, closed by **144 repair
  items** (some repairs closed more than one finding; a dozen were found *by the
  repairer*, not the verifier — including two of the failing gates in §2.2).
- Three campaigns (`iso9409_wedge_flexure_gripper`, `rotor_runout_gauge_bridge`,
  `screw_on_exponential_horn`) report `git status` dirt under `crates/` —
  7 modified files in `crates/kernel-api/`, 3 untracked examples in
  `crates/kernel-model/`. Verified **not attributable**: mtimes pre-date the part
  directories by up to two days, and no campaign edited outside its own tree.
  `tools/` is clean in every campaign. This is a repo-hygiene item for the
  maintainer, not a campaign failure.

---

## 2. The honest failures this portfolio SHIPS on purpose

This is the credibility of the exercise. **Nine of ten parts ship at least one
gate that fails, or a rating below the requirement it was designed against, or a
mandatory process step without which no number on the page applies.** Not one was
converted into a margin, and not one was hidden. The single exception,
`ball_kinematic_mirror_mount`, passes every live gate — and its headline is a
**1.064× creep margin**, an un-converged first mode and 9 recorded refusals,
which is the same honesty by a different route.

### 2.1 Requirements the parts provably do NOT meet

| part | requirement | delivered | shortfall | receipt |
|---|---|---|---|---|
| cubesat frame | 1200 N dispenser rail-stack (CDS 30 g quasi-static) | 322.7316 N design critical | **3.72×** | `buckling_frame.json` |
| wrist QD | ≥ 1.5 N·m index hold | 0.03672 N·m | **41×**; the feasible set is EMPTY (`constraint_ok:false`, 44 evals / 4 starts) | `opt_A_card_literal`, `opt_B_achievable` |
| gripper | ~20 mm jaw stroke at 1e5 cycles | 10.927 mm rated; **2.603 mm continuous-duty** (23.82 % of rated) | not fatigue-rated at full stroke at all | `opt_asbuilt.json`, `fatigue_rated_1e5.json` |
| deck cleat | 400 N static / 100 N sustained | 136 N static (SF 1.5) / **5.6 N sustained at 40 °C** | ~18× on sustained; best found anywhere in the box over **190 evaluations across 4 optimizer runs** is 19.9 N at 23 °C | `handcalc_stage3.json`, `opt_B_rating_n8.json` |
| rotor gauge | 0.00712 mm tip deflection across the whole travel | 0.0265897 mm at the outermost station | tool is accurate to y ≈ 113.3 mm, reaches 144.6 mm | `fea_L1_c_run.json` |
| DIN-rail enclosure | thermally house a 7.6 W Pi 4 | fails at **both** ends of the bracket: 109.520 °C optimistic, 262.199 °C conservative | this enclosure does not cool a Pi 4 | `thermal_derate` / `optimistic_bracket.verdict` |
| turgo runner | a duty rating | **none claimed.** The "~12 h cumulative" concept figure is formally WITHDRAWN | 166,521 cycles ≈ 58 min at 48.06 Hz — and that is the *optimistic in-plane* direction, not the governing one | `refusals.json`, DESIGN D10 |
| horn | mouth loading kR ≥ 1 | kR = 0.92895 at cutoff | not reachable: the support-free slope cap and kR = 1 are the same equation | `horn_expansion_design_point.json` |

### 2.2 Gates that ship FAILING, on purpose

| gate | verdict | why failing is the correct outcome |
|---|---|---|
| `prodcheck_L6_relief_root` (wrist) | **SF 1.3564**, required 2.0, 22.3013 MPa at the detent-relief root | Rather than add a fillet (a geometry change that would have to re-clear the whole gate suite and re-run the FEA with no honest budget), off-axis bending is **de-rated 4.00 → 2.71 N·m**. A published de-rate beats an unverified fillet. |
| `production_check_blade_root` (gripper) | **fatigue SF 0.682**, N = 37.4 < the curve's own 500-cycle floor | The part is published "not fatigue-rated at full stroke", with a separately-gated continuous-duty stroke of 2.603 mm at Nf = 99,863.6. |
| `production_check_crank_pin` (gripper) | **fatigue SF 1.7399**, `ok:false` | Was *hidden* pre-repair — the analysis table hand-picked 6 of 25 rows. Now every rule of every `production_check` receipt is emitted (25 rows / 17 rules, 2 failing) with `production_check.py`'s verbatim `detail`, plus the reasoning that a 1e6-cycle fatigue rule is not the governing demand for a transient stall cap. A hand-picked subset is no longer expressible in the generator. |
| `production_check_stall_mesh_refined_bound` (turgo) | **fatigue SF 0.7028** — the *third* shipped fatigue failure | The refined mesh reads **+9.6 % higher** (12.854 → 14.087 MPa), so the campaign's own earlier wording ("the conservative one") is retracted: the governing peak is a rigid-clamp singularity that does not converge and is **not** an upper bound. |
| `tol_i1_rotate_on_swing` (DIN rail) | `worst_min` **0.0000 mm**, `pass_worst false` | Found *during repair*, not by the verifier. Shipped as a stated marginal with an assembly instruction, exactly like `tol_i2` (−0.0500 mm), rather than tuning the threshold. |
| `sweep_rotate_limit` (DIN rail) | `ok:false` **by design** | It exists to pin the over-rotation limit at 16.0 deg. A sweep that stops at its own minimum proves nothing; this one runs past it. |
| NC6 / NC-O oracles (all 10) | **exit 1 by design** | Every campaign ships at least one negative control whose only job is to prove a gate can go red. Several were rebuilt in repair because the original proved nothing (see §3). |
| `closure_arithmetic.pass_hot_shop_worst_print` (rotor) | **false** — 4.786 MPa vs the 3.0 MPa 40 °C/1 h cell | Hot-shop assembly is a condition limit, not a margin. Survives this pass unconverted. |
| `airtopo_nc6_plugged` (cubesat) | `ok:false` **by design**, must-FAIL row in `check_gates.py` | Built in repair to give the air-topology oracle a red case. Only the *left* channel is plugged, so a global voxelization failure cannot masquerade as the oracle firing. |
| `tol_i5_guide_centred` (gripper) | `ok:false`, `worst_min` −0.0 mm | Line-to-line, never negative, non-governing — and now *said so*, with the governing running fit (`tol_i5_guide`, min_clearance 0.300 mm) cited beside it. |
| `pre_opt/creep_gates.json` (mirror) | `all_pass false`, `worst_margin 0.6013` | The **stage-2 infeasibility record**. Repair made the archive WRITE-ONCE so a re-run can never destroy the evidence that the pre-optimisation design did not close. |

### 2.3 Condition limits — stated, never converted into margins

- **cubesat thermal**: 2 W through the four M3 bosses drives the hottest voxel to
  **197.479 °C = 3.59× PLA's 55 °C softening point**. Backed out, the allowable
  dissipation is **0.34787 W total (0.08697 W/boss)** — and that number was
  *verified by re-running the solver at that power*, returning `t_max` 55.000 °C.
- **horn creep above 23 °C**: the declared ambient is 25 °C; `creep_allowable_mpa`
  is a two-row step lookup that jumps to the 55 °C row above 23 °C. Both cells now
  ship side by side: plate flat SF 4.80 → **0.96**, plate on lugs 1.12 → **0.22**,
  bracket 1.41 → **0.28**. Two gates *assert the elevated-row SF stays below 1.0*,
  so the condition limit cannot silently become a pass. Falsification-tested.
- **wrist creep**: **three** non-closing cells shipped, not one. Detent seated
  3.3304 MPa → SF 2.2520 / 1.5013 / 1.0509 / **0.7507** at 1 h / 24 h / 30 d / 1 y;
  design duration published as the **1 h cell** (the longest reaching SF 2.0);
  continuous hang republished at **25.3 N min-material** (was 30.3 N nominal).
  Post-repair the campaign looks *worse* and is *truer*.
- **singulator creep**: SF **1.56** (LC1a, 40 °C/24 h), SF **0.66** (LC1b sideways,
  40 °C/1 h), SF **1.65** (LC1b sideways, 23 °C/1 h), each labelled CONDITION LIMIT,
  plus the recorded conflict with `production_check`'s more lenient 11.0 MPa rule
  and the ruling that "the table governs".
- **mirror mount**: 40 °C empty-preload-window; frame f1 **not converged** (1.6 mm
  and 1.2 mm grids give 2.19 and 2.92 voxels across a wall against the ≥4 rule;
  +21.8 % refinement delta; resolving the rib needs voxel ≤ 0.875 mm).
- **DIN rail**: 84.398 °C boss condition violation; and the bolt's buckling factor
  367.3956 is explicitly labelled **not a margin** because the runner's own
  yield-before-buckling note (547.6 MPa ≥ 55 MPa yield) is quoted as the governing
  physics.

### 2.4 Required process steps — without these, no number on the page applies

- **cubesat**: the CDS rail envelope stack is `ok:false` as printed —
  `nominal_gap` **−0.100 mm**, `worst_min` **−0.500 mm** with the first-layer
  elephant-foot flare. Post-faced, the same chain passes at 0.100…0.700 mm.
  **Post-facing the four rail pads is mandatory**, and the flare term itself is
  still an assumption (a coupon ships to measure it).
- **rotor gauge**: a **Ø28 × Ø15 penny washer under each lug nut**, and lug nuts at
  **1.5 N·m** — window 1.3877 – 2.4 N·m (×1.73 span, governed by *turnability*:
  15.0 N·m hand torque ÷ 6.25 breakaway-per-N·m). Outside it the pad creeps or
  the joint gaps. M4/M5 screws at 0.08 N·m.
- **DIN rail**: `tol_i1` and `tol_i2` both ship marginal, each with the assembly
  instruction that follows from it, printed inline in the generated
  `assembly/instructions.md`.
- **gripper**: five jobs exit **0** while emitting `ok:false`. The README now names
  all five and instructs: **parse `ok`, never `$?`**.
- **singulator**: `wall_thickness` and `support_report` are **UNGATED on the file
  that is actually printed** (`housing_top_threaded.stl`) — an engine limit, not a
  choice. Disclosed in three places including a `gates_NOT_closed` block inside the
  receipt itself.
- **deck cleat**: DESIGN D16's shrouding split-pin end pocket **was never built** —
  the split-pin eye stands ~2.4 mm proud of the lug face, ~3 mm above the plate top.
  Found during repair, disclosed in three places with the measured protrusion, and
  deliberately *not* silently built (a geometry change to a frozen, fully-re-gated
  design would require the whole physics re-run).

---

## 3. What verification actually caught — the process lesson

**Ranked by how many of the 10 campaigns the class recurred in.** Every one of
these survived the campaign's own final self-check. That is the finding.

### Rank 1 — Hand-typed literals in document generators that no longer match the receipt they cite — **10/10 campaigns**

Not stale *documents* (every campaign correctly regenerates ANALYSIS.md from
receipts) — stale **constants inside the generator**, which then propagate into a
"generated" document that looks trustworthy. Representative instances:

| campaign | the literal | the receipt said |
|---|---|---|
| turgo | `gen_analysis.py:281` mesh volume **976.591 mm³** → "+28.41 %" | 1256.3974 mm³ → **−0.184 %** (a new `exact_volume` op had to be added to measure it at all) |
| cubesat | `gen_analysis_body.py:497` modal delta **+0.36 %** | 100·(169.39793616/169.10059317 − 1) = **+0.18 %** — §2 and §8 of the same document contradicted each other |
| mirror | hard-coded platform mass **11.5 g** in the bounce bound | `report_platform.json:g_mass` = 12.8208 g → the bound was **3.4 % HIGH** (49.156 → 47.540 Hz) |
| horn | `M_P2_G = 72.4 g`, cited to a `mass_properties` op **that does not exist** | 72.2146 g; headline printed set 480.92 → **480.73 g** |
| gripper | `gen_checks.py` hand-typed **3429200 Pa** demand | verdicting a stress that no longer existed after the voxel field went stale |
| DIN rail | `109.2 / 261.3 °C` hand-typed **inside a receipt** | 109.520 / 262.199 °C |
| wrist | `gen_design_s17.py:173` "voxel 0.35 mm" beside a 0.28 mm run's results | job's real `voxel_mm` |
| cleat | `gen_poses.py:66` "lifted 2.4 mm" (pre-amendment) baked into every pose `_why` | LIFT = 3.8 mm |
| singulator | `%%` printf artefacts, an objective column reading `?`, a stale build_dir docstring | — |
| rotor | insertion springing **0.033 MPa** in DESIGN §19.2 | 0.1218 / 0.3798 MPa |

**Lesson:** "generated from receipts" is a property of the *pipeline*, not of the
document. Every campaign now derives these values at generation time; several
added a guard that refuses to write the document at all if the source receipt is
missing or unparseable. `jar_top_seed_singulator` proved this the hard way: a
shell redirect (`creep_gate.py > receipts/creep_gate.json`) **truncated the valid
JSON the script writes itself** and replaced it with a human table — and the
generator's `rec()` silently returned `None`, deleting the entire creep gate
section from both documents. It now raises `SystemExit` on a present-but-corrupt
receipt.

### Rank 2 — Claims that contradict the receipt they cite — **10/10 campaigns**

The single most-repeated *individual* defect in the portfolio:
**5 of 10 campaigns shipped the sentence "`components` catches the severance while
`shells` still reads 1"** — copied from `DELIVERABLE_SPEC.md §2.2`'s example — as
though it were a measurement of their own oracle. In all five, the oracle receipt
said `shells: 2`.

`turgo_runner` then proved the spec's example is **not constructible on this
kernel**: for any solid `validate` accepts, a closed 2-manifold surface bounds
exactly one connected volume; every vertex- or edge-touching construction is
refused before it can be measured (`union_all failed validate(): closed=true
manifold=false genus=1 euler_characteristic=3 shells=2 — refusing to bind an
invalid solid`). And the real blind spot runs the **opposite** way: two bodies
0.0005 mm apart — half of `mesh_components`' 0.001 mm weld tolerance — measure
`shells 2` / `components 1, is_one_body true`. **`components` is the weaker of the
two, not the stronger.** `din_rail_pi4_enclosure` reached the same conclusion
independently. Both now ship the *inverse* oracle as a measurement.

Other instances: cubesat's "the binding constraint was buckling" (the optimizer
receipt shows **neither** constraint active — f1 +22.2 % slack, buckling +25.8 %
slack, and the search stopped on its 12-evaluation budget); rotor's README
error-band bullet with the **stress direction backwards** (the manifest says
*under*-predicts; measured 18.6 % low); mirror's provenance tag "exact
`overlap_volume`" — **no such key exists in any receipt in the tree**; cleat's
"not reachable in this parameter box" cited to a **MINIMISE-mass** run.

### Rank 3 — Recorded-but-unasserted measures; gates that cannot fail — **10/10 campaigns**

`DELIVERABLE_SPEC §2` says "a recorded-but-unchecked measure is worthless", and
every campaign shipped some anyway:

- **rotor**: the NC-O severance oracle was measuring a faceter artefact — the
  *intact* STEP-imported bridge already read `components: 6`, so severing 6 → 2
  proved nothing. Rebuilt natively with an intact control that must PASS first.
- **horn**: the two shipped **print files** had **zero** connectivity or bed gates
  anywhere (`implicit` binds no solid). Repair parses the shipped binary STL
  directly and welds on a 1e-3 grid; falsification-tested against a synthetic
  severance moving only **0.49 %** of the volume — far below the only volume check
  that existed.
- **turgo**: 5 measures + 10 posed overlaps were recorded and never asserted;
  a `--self-test` now perturbs each the wrong way (**8 of 8 rejected**).
- **cubesat**: the two shipped gauge STLs — budgeted into the print plate — carried
  **zero asserts**. Six added, all six proven falsifiable.
- **mirror**: the claim "the rib stops at 3.5 mm because of the wall rule" was
  never measured. Measured: `n_active_constraints = 0`. Claim retracted; the
  sibling optimiser's version survives on evidence (`n_active = 2`).
- **wrist**: `selfcheck` item 4f asserted `len(tol) >= 14` — **a count can never
  fail for a missing interface**. Replaced with an explicit 17-ID coverage list.
- **singulator**: `parts/housing_top_threaded.stl`, the actual print file, had no
  gates at all.

### Rank 4 — Physics gated at the wrong condition, or a validity limit overreached — **8/10 campaigns**

The dominant sub-class is **creep read at 23 °C for a part whose declared service
temperature is higher**:

- cleat: 23 °C → 40 °C moved the sustained rating **18.5 N → 5.6 N**;
- horn: 23 °C → 25 °C moved three margins from SF 4.80/1.12/1.41 to **0.96/0.22/0.28**;
- wrist: the detent's only creep gate was `production_check`'s lenient yield×0.20
  with **no design duration at all**;
- singulator, mirror, cubesat: the same axis, caught before ship.

Adjacent: singulator's headline margin was posed at the corner that *flattered*
each term (**37.1 % optimistic**); rotor **imported** a mesh-convergence direction
from another geometry's manifest and it was **wrong in the unconservative
direction** — measured, the 1.5 mm grid reads 7.05 % HIGH on deflection and 18.6 %
LOW on stress.

### Rank 5 — "Reproducing" sections that do not reproduce — **8/10 campaigns**

- gripper: **all five** documented `render_sheet.py` commands died with
  `FileNotFoundError` from the repo root;
- DIN rail: step 4 died from the repo root;
- mirror: `run_stage3.py`'s `archive_pre_opt()` was **not idempotent** — a second
  run *overwrote the stage-2 infeasibility audit trail*; and the documented
  `gen_opt.py --which 2` used an 8-eval default against the shipped 6, silently
  drifting a parameter and staling every downstream frame number;
- cleat: `cp _gen/*.stl ../parts/` also copied a posed *render scene* into the
  print-file directory; four receipts had no rebuild command at all;
- singulator: the step-3 kernel loop ran two **tool** jobs (no `ops` key) through
  the kernel, depositing junk receipts;
- turgo: derived `production_check`/`refusals` jobs were **not downstream** of the
  shipped tet receipt — the new SHA-256 provenance check went **red on its first
  run**, independently confirming it.

### Rank 6 — Stale derived data consumed silently — **blocker-class where it hit**

`iso9409_wedge_flexure_gripper`'s blocker: `voxelize_stl.py` has no staleness
guard, so a design amendment moved geometry and **every** voxel-consuming
analysis kept eating the old density field. Fresh voxelisation of the
byte-identical shipped STL gave 64,328 solid voxels against the shipped 62,552 —
**4,178 cells different**. Re-running moved f1 **70.54 → 65.84 Hz** (−6.7 %) and
every stress and deflection with it. Same class, different mechanism: two
receipts meshed from *character-for-character identical* program geometry shipped
**different `geometry_hash` values** and nothing in the toolchain reads that field.

### Ranks 7–9 — the remainder

7. **Orphan / missing / phantom receipts — 5/10.** DIN rail shipped a receipt no
   step regenerated (and a second of the same class); turgo cited
   `receipts/opt_receipt.json` and `receipts/modal_screen_110_*` which **never
   existed**; mirror's `gen_analysis.py` would silently drop two honesty
   disclosures if their receipts vanished — it now refuses to write at all.
8. **Build orientation stated backwards in prose — 4/10.** The wrist's blocker:
   README said "spigot down, +Z" while the gate measured `[0,0,-1]`, and
   `gen_render.py` hardcoded `[0,0,1]` for **all three** contact sheets — so the
   shipped render drew the wrong bed. Singulator: DESIGN's `build_dir` column said
   `[0,0,1]` while its own prose column said "collar face down" for both housings.
9. **Plan rows and constraints that vanished silently — 5/10.** cubesat and turgo
   each dropped a §6 optimiser constraint that then appeared in neither optimiser
   job; singulator lost three plan rows; cleat lost two open questions from
   DESIGN §O.5 between stages.

### The meta-lesson

**No campaign shipped bad geometry. Every campaign shipped bad bookkeeping about
good geometry.** The gates that check *solids* — validate, components, volume
windows, watertight, bed fit — held everywhere. The gates that check *claims* did
not exist, because the spec has no mechanism to assert that a sentence matches a
number. Every repair that stuck moved the check from prose into code: read the
receipt at generation time, refuse to write on a missing source, assert coverage
by ID rather than by count, and build a negative control whose *intact* case must
pass before the failing case means anything.

---

## 4. What could NOT be broken — the strongest verified claims

1. **Geometry, universally.** Across every campaign that re-ran its pipeline after
   repair, **every** shipped artifact came back byte-identical: mirror **17/17**
   (STL, 3MF, STEP, 6 renders, instructions, plate layout, coupons); horn **24/24**
   verified by SHA-256; gripper 10 CAD artifacts + 6 PNGs; rotor both STLs and both
   STEPs; wrist all print files; cubesat both gauge STLs; turgo `part_program.json`
   byte-identical across a generator refactor. `iso9409` states it plainly: *"the
   geometry was never wrong, only the analysis computed against a stale
   voxelisation of it."* **19 blockers, 0 geometry defects.**
2. **Zero `warnings` entries, everywhere.** Every campaign greps its full receipt
   tree at every nesting depth and reports 0. Horn: 68 receipt JSONs, 0 warnings.
   Mirror: 77 receipts scanned, 0 non-empty `warnings` arrays. This gate — added
   2026-08-06 — held in 10/10 campaigns with no exceptions.
3. **Negative-control interference volumes are exactly reproducible.** Mirror's
   five NC volumes reproduce to the last stored digit (0.24025, 0.24025, 2.74933,
   978.0461, 69.9710, 2.66484 mm³). DIN rail's 9.0852 / 3.9200 / 912.0000 mm³
   likewise. These are *exact* B-rep intersections, not voxel estimates, and they
   are the strongest measurement class in the portfolio.
4. **Closed-form cross-checks agree with the kernel to 4–6 significant figures on
   independent routes.** Singulator's thread ridge: measured 1912.146 mm³ vs
   Pappus 1924.365 mm³ = **−0.635 %**. Cubesat's gauge: closed form
   25×(130²−100.4²) = 170496.000 mm³ inside a 0.01 % band. Horn's P1: exact volume
   306270.9459 mm³ against a closed-form target of **306270.9459 mm³**.
   Singulator's NC4 "miss": the 0.012 mm shortfall is the inscribed-36-gon sagitta
   3.15·(1−cos 5°) = 0.011987 mm against a measured 0.011988 mm — **agreement to
   six decimals**, which is what allowed the finding to be correctly refused.
5. **STEP round-trip conservation.** Rotor: `exact_volume` on the re-imported BREP
   agrees to **1.5e-14 %** (bridge) and 1.4e-12 % (carriage); the mesh-volume route
   agrees to 2.4e-6 % and 0.0068 %. Horn's gated round-trip: 0.0409 %, against a
   2.5 % assert.
6. **Every new gate added in repair was proven able to go red.** Horn:
   **103 → 154 gates**, every new one falsification-tested (synthetic severed and
   oversize STLs; in-memory receipt mutation with a clean unmutated control).
   Turgo: 8/8 perturbations rejected. Cubesat: all six new gauge asserts return
   `assert failed: …` and exit 1. Mirror: removing a required receipt exits 1 and
   leaves ANALYSIS.md untouched. Gripper: reverting the render paths turns the
   self-check red on all five sheets. This is the portfolio's answer to "how do you
   know the gate works".
7. **Byte-stable documents across genuine solver drift.** Cubesat re-ran
   `ace_buckling` until the eigenvalue actually moved
   (0.5378860166134007 → 0.5378860166133207) and ANALYSIS.md still returned
   byte-identical by `shasum`. Fixed-precision printing is a sufficient campaign-side
   mitigation for the solver non-determinism in §6-T7.
8. **Independent re-measurement confirmed the verifier in almost every case.**
   `rotor_runout_gauge_bridge` re-measured all twelve findings before touching
   anything and refused none outright. Where a repairer disagreed, they disagreed
   **with evidence and to six decimals** — see §5.3.

---

## 5. Residual risk

### 5.1 The standing "required, NOT performed" set, aggregated

**Nothing in this portfolio has been printed. 0 of 34 print files have been made,
and 0 of 10 parts have been physically tested.** Every mechanical claim is a
solver or closed-form result on a digital solid.

| test | parts requiring it | why it matters |
|---|---|---|
| **Any print at all; any physical fit check** | **10/10** | Every tolerance stack, support report and wall gate is a model of a printer, not a printer |
| **A slicer run** | 10/10 — called out explicitly for the horn's 0.5092 / 0.7638 mm thread grooves | The slicer, not the kernel, decides whether a groove prints |
| Coupon: elephant-foot flare | cubesat (a **required process step** depends on the assumed flare term) | The rail-pad post-facing requirement rests on an assumed number |
| Coupon: heat-set pilot ladder at ±0.15 mm with real inserts | horn (**this is the §5.6 gap**), singulator | No source publishes a pilot-bore *tolerance*; the band cannot be invented |
| Coupon: land-gap printability (C1) | rotor | |
| Gauge R&R on a real rotor | rotor | The gauge's whole purpose is unvalidated |
| Physical singulation count test | singulator | No singulation **rate** is claimed (refusal R1: granular flow has no in-tree solver); acceptance is measured on **sphere gauges, not seeds** |
| Free-motion sweep | wrist — "required, NOT PERFORMED" | |
| Drop / shake / thermal cycling | 10/10 | None performed anywhere |
| Bolted-joint / set-screw preload test | turgo — contact is the stated weakest link, gated by **one closed form** (1.458 vs 2.5 MPa, 1.72×) with no contact model and no FEA | |

### 5.2 Analysis that could not be completed (engine-limited, not effort-limited)

- **Across-layer fatigue is unknowable.** `ace_fatigue` refuses across-layer
  loading for *every* material because no printed-polymer across-layer S-N data
  exists. This is the **governing** direction for the turgo bucket root and is a
  recorded DataRefusal for the wrist. Turgo states it plainly: *"There is no route
  around this refusal."* Every life quoted anywhere rides the **3.7×–90×** 90/10
  scatter band.
- **No body-fitted (tet) arbitration anywhere.** 7 of 10 campaigns tried;
  `ace_fea_tet` either refuses the affordable element size or does not finish at
  the size that meshes clean. Consequence: the hex8 stress fields have **no
  independent check**, and the ±20 % coarse under-read is a manifest claim the
  portfolio could not verify on its own geometry — indeed rotor showed the
  manifest's *direction* does not transfer between geometries.
- **Two grids is not a convergence study.** Rotor and turgo both refuse to claim
  Richardson extrapolation from a 1.5× refinement ratio on a stair-stepped voxel
  boundary. Turgo's governing stall peak **cannot** converge — it is a rigid-clamp
  singularity (12.854 → 14.087 MPa, +9.6 %, and a third rung would read higher).
  Mirror's frame f1 is not converged and needs voxel ≤ 0.875 mm to be.
- **No Kf notch knockdown** on the cleat's fatigue screen (source measures 16–29 %),
  because a notch-resolved stress requires the refused tet route.
- **Three step-4 optimiser receipts in `cubesat_1u_dev_frame` are structurally
  unreproducible**: re-running the optimiser rewrites `programs/geom.py` and
  therefore changes the shipped part. Mitigated by a **live optimiser-provenance
  guard** that fires if `geom.py` no longer holds the point those receipts selected
  (demonstrated), and disclosed in three places — but an independent verifier
  cannot re-execute them, and the campaign says so in those words.

### 5.3 Findings that were refused, with the counter-evidence

12 refusals were filed. Five are substantive refusals of a finding or its
prescribed fix; the rest are partial corrections that changed the *reasoning* but
kept the fix. The pattern is healthy: **repairers re-measured before repairing,
and said so when the verifier was wrong.**

- **wrist, finding 10 (insert build orientation "backwards")** — refused on the
  revolve profile itself: the bore is void from z=0 and *closed* at z=13.40, so it
  opens at z=0, not z=20. Arithmetic confirmation from the receipt: `bed_area`
  306.56 mm² ≈ π(12.00²−6.50²) − twelve 45° notches = 306.6; the stud tip would be
  213.8. The flange face **is** the TD face — nothing said so, which is the real
  defect, now fixed.
- **singulator, finding 8 (NC4 misses its 0.30 mm criterion)** — refused: the
  0.011988 mm shortfall is the inscribed-36-gon sagitta 0.011987 mm. Agreement to
  six decimals. The *non-disclosure* was the real defect and is fixed.
- **singulator, finding 11 (nc3 is an exact-contact pose)** — refused: holding the
  pose byte-identical and varying only segment count gives distance 0.0 at 36 and
  128 segments, **0.5999 mm at 360** — exactly the design clearance. Floating the
  pose would have masked a measurement artefact.
- **turgo, finding 2's second remedy (build a shells-1/components-2 oracle)** —
  refused as not constructible, with three verbatim kernel refusals; the *inverse*
  oracle was built instead.
- **cubesat, finding 5's reasoning** — the verifier attributed a +0.063 % buckling
  change to solver noise "4e-13 relative". Off by **nine orders of magnitude**
  (6.24e-4 / 4e-13 ≈ 1.6e9). The claim was fixed anyway, on the correct ground:
  `ace_buckling`'s own 10–30 % coarse-hex8 band, ~160× larger than the delta.
- **cubesat, finding 7's prescribed fix** — refused because applying it as written
  would ship a **permanently-red alarm** (the three receipts pre-date `geom.py` by
  design), and a guard that is always on carries no information.

### 5.4 Open items carried forward (47 total)

Structural, not oversights, in order of consequence:

1. **The turgo runner's governing peak does not converge and cannot** (rigid-clamp
   singularity; the fix needs a compliant boundary condition the mesher refuses).
2. **The mirror mount's f1 is not converged**, and no tet stress field exists
   (both attempts resource-killed, now reproducibly so at a bounded 1200 s budget).
3. **Three creep cells in the wrist and one in the horn do not close at SF 2.0**
   and no geometry was changed to close them. Both are published as duty limits.
4. **`housing_top_threaded.stl` (singulator) is printed ungated** on walls and
   supports; **`collar/stand` threaded files (horn)** likewise inherit their wall
   and support gates from exact pre-thread twins.
5. **The horn's §5.6 heat-set stack** — a named gap, not an invented band.
6. **The cleat's D16 split-pin pocket does not exist**; disclosed with the measured
   protrusion.
7. **Solver receipts can never be `cmp`-clean** (§6-T7). Documents are byte-stable;
   receipts are not, and every campaign now says so rather than letting a reader
   read a diff as drift.
8. **`crates/` is dirty in the shared repo** — 7 modified + 3 untracked, pre-dating
   every campaign. Maintainer's to reconcile; `DELIVERABLE_SPEC §5.11` cannot pass
   until it is.

---

## 6. Input to the engine fix phase — friction themes ranked

127 friction entries across 10 campaign files, plus 13 from the pre-campaign
digest. Deduplicated into 16 themes and ranked by **how many independent
campaigns hit them** — a theme 7 campaigns hit independently is a design defect,
not bad luck.

| # | theme | entries | campaigns | severity | representative entries |
|---|---|---|---|---|---|
| **T1** | **Doc / digest / cookbook / `describe` drift.** Wrong argument names, wrong nesting, wrong units, empty `doc` strings. Every campaign paid for this in wall-clock. | 16 | **9/10** | High (universal tax) | ball F3 (`N.m-3` vs `N/kg`), cubesat F6 + wrist F6 + gripper F3 (`ace_fatigue` `stress` block nesting — **3 campaigns, same line of the cookbook**), wrist F5/F9, ball F6 + singulator F12 (`explode.axis`), rotor F1, horn F11, singulator F16 (`describe support_report` ships **empty** doc) |
| **T2** | **Body-fitted / large-model solve path is unusable.** `ace_fea_tet`/gmsh refuses watertight validate-clean STLs, aborts the process on retry, is unaffordable at the needed size, and is not bit-deterministic. `ace_modal`/`ace_fea` unaffordable or non-convergent at the frame every other analysis uses. | 12 | **8/10** | **Critical** — removes the only independent check on every stress number in the portfolio | cleat F5, gripper F5, singulator F9, wrist F10, rotor F9 + F12, turgo F2 + F10 + F12, ball F8, cubesat F5, turgo F5 (360k dof > 10 GB) |
| **T3** | **Exit-code and receipt contract violations.** Tools exit 0 on `ok:false`; internal `KeyError` exits 1 while a real FAIL exits 0; a timeout or kill leaves **no receipt at all**; a job-level `receipt` key silently clobbers a shipped file. | 10 | **8/10** | **Critical** — the one outcome `SPEC §3` forbids is *silence*, and these produce it | gripper F9 (`tolerance_stack` + `production_check` exit 0 on `ok:false`), ball F5 (`joint_check` **inverted**), ball F11 (`physlib.run_tool` `TimeoutExpired` → no receipt), singulator F14 (**clobbers a shipped receipt**), turgo F8 (documented `\| tail -1 >` idiom **destroys** a good receipt), wrist F8, cubesat F11, din_rail F3, cleat F7, ball F7 |
| **T4** | **Path / root asymmetry.** `export_step` writes under `--out-dir`; `import_step` resolves against the *program* dir and refuses `..`. `sweep_check` / `param_optimize.call_engine` write station programs to a **system temp dir**. `render_sheet` resolves job-relative paths against the CWD. Reports echo the resolved path, so they are not byte-comparable. | 13 | **7/10** | **High** — the direct cause of "Reproducing does not reproduce" in 4 campaigns | singulator F5 + F8, horn F4, rotor F3 + F11, ball F2, gripper F4 + F7 + F10, turgo F7, din_rail F5 + F6, digest F7 |
| **T5** | **`clearance` / `assert_disjoint` return no usable number for non-touching bodies.** `distance: 0.0` with `interfering: true` for nested, interlocked, enclosed or coaxial pairs with real gaps; `overlap_volume: null` on helical, high-face-count and STEP-imported operands; an exact-contact 0.0 that is a **faceting artefact** of inscribed polygons. | 9 | **7/10** | **High** — `SPEC §2.11` mandates a *measured clearance* on every legal path, and this op cannot deliver one | horn F7 (real 0.30 mm gap reads 0), turgo F6, wrist F2, cleat F3, rotor F5, gripper F2, singulator F6 + F18, horn F6 |
| **T6** | **The connectivity gate is not trustworthy, and the spec's oracle for it is not constructible.** `mesh_components` over-counts one component per `extrude_with_holes` hole loop; reports a one-body part as 9 (wrist) or 24 (horn) or 6 (rotor) bodies after `import_step`; welds at a **fixed, non-tunable 0.001 mm** `weld_tol` that hides a real severance. | 8 | **7/10** | **Critical** — `SPEC §2.2` calls this first-class and mandatory, and it is the gate most campaigns could not trust | rotor F10 (**the mandatory gate does not survive `import_step`**), horn F10, wrist F11, cleat F6, gripper F1, turgo F11, din_rail F8 |
| **T7** | **Solver receipts can never be byte-compared.** `ace_modal` (~1e-13 to 2.6e-12 relative), `ace_buckling`, `ace_fea_tet`/superlu are not bit-deterministic; **and** every ACE receipt embeds a wall-clock `timings_s` field. | 6 | **6/10** (7 affected) | Medium — mitigable campaign-side, but `SPEC §3`'s determinism rule names only STLs/PNGs and is therefore silently unmeetable for receipts | ball F10, cubesat F9, horn F14, turgo F10, cleat F8, singulator F15 |
| **T8** | **Kernel booleans refuse or fail to terminate on legal geometry.** `difference` refuses a solid carrying both end chamfers and a vertical-edge fillet; `union_all` over disjoint cutters never returns; a **0.04 mm** change to one cutter makes a later, **27 mm-distant** boolean fail `validate`; STEP round-trip refuses a body the kernel itself calls valid at a 0.1 mm threshold; `cone` is a true cone with no frustum constructor. | 13 | **6/10** | High — forces geometry to be authored around the kernel rather than the design | singulator F4 + F7 + F10 + F11, cubesat F1 + F2, cleat F2 + F4 + F9, rotor F2, wrist F3 + F4, horn F3 |
| **T9** | **Auxiliary tooling crashes or mis-renders on legal input.** `assembly_doc` refuses a legal job when the step prose is long and takes `view` as a dict not a list; `analysis_sheet` crashes with a bare `KeyError` on a load with no `label` and has no unit conversion; `render_sheet` overlay collides with the meta line; `air_topology_audit` silently **seals a wide-open bore** when a slice centre lands on a vertex ring, and its `seed_labels`/`sizes_cm3` cannot be joined. | 10 | **6/10** | Medium-High (`air_topology_audit`'s false-seal is High) | horn F8 + F11 + F13, cubesat F7 + F8 + F11, singulator F12, din_rail F4, ball F6, gripper F10 |
| **T10** | **`assert` has no vocabulary for half the mandatory gates, and voxel-route bodies bind no geometry at all.** `assert` accepts only `volume_within` / `exact_volume_within` / `genus` / `shells` / `components` / `closed` / `manifold` / `valid` — so `SPEC §2.4` (route/watertight), §2.5 (`steep_area == 0.0`), §2.6 (`thin_area`/`p05`) and §2.7 (`fits_within`) **cannot be in-program gates for any campaign in this repo**. Worse: `implicit`, `hybrid_boolean` and `import_mesh` bind no solid, so the actual print files of two campaigns cannot be gated at all. | 7 | **5/10** | **Critical** — this is the single largest gap between what the spec mandates and what the engine can express | turgo F9, horn F5 + F15, singulator F17 (verbatim: `op 'p_validate' param 'in': '…' is a measure/export op and binds no geometry`), wrist F13 + F14, cubesat F3 |
| **T11** | **`tolerance_stack.py` defects.** Double-counts an **asymmetric** tolerance in the worst-case band; CHAIN mode leaks a raw `KeyError` instead of refusing; bakes the receipt path into the job with no dry-run; a job-level `receipt` key silently overrides the caller's `--out`. | 5 | **5/10** | High — this tool produces the `SPEC §2.10` evidence for **every** mating interface in the portfolio | ball F4, din_rail F1, cleat F7, singulator F14, gripper F9 |
| **T12** | **`param_optimize` quirks.** A constraint with `max: 0.0` crashes the whole run; a coarse in-loop voxel **silently quantizes** a parameter with nothing in the receipt saying so; `evals` written as an int under a plural name; command timeout undocumented; station programs to a temp dir. | 5 | **5/10** | Medium-High (the silent quantization is High — it changes the answer) | turgo F3, rotor F8, din_rail F9, cubesat F6, gripper F4 |
| **T13** | **Physics runners accept physically meaningless jobs and return a number instead of refusing.** `ace_buckling_runner` accepts a purely **tensile** load case and returns a positive factor plus a knockdown block, no warning. An `ace_fea` `slider` fixture **silently degrades to "no fixture"** when its selector catches only inactive voxels. `ace_fea` converges cleanly on an inclined thin wall that is a kinematic hinge chain. `ace_contact` curve row 0 is the un-equilibrated initial state. Manifest `validation.direction` does not transfer between geometries and nothing warns. | 5 | **4/10** | **Critical** — the same class as digest F6 ("silent acceptance — THE trap"); these produce confident wrong numbers | din_rail F7, rotor F6 + F14, horn F9, wrist F7 |
| **T14** | **The creep-allowable surface is contradictory, unreachable and too coarse.** `creep_allowable_mpa(T, hours)` is not reachable from the surface the brief points campaigns to; Python and Rust readers of the **same table disagree above 55 °C**, with Python in the **non-conservative** direction; `production_check.py`'s creep rule contradicts the material record and has **no duration input at all**, so its creep verdict is derived from the static yield it is meant to replace; the table is a two-row step (23 °C, 55 °C) with **nothing between**, and nothing in the gate surface makes the temperature visible. | 6 | **4/10** filed | **Critical** — this caused blocker- or major-class defects in **6** campaigns; sustained load is the governing mode in at least 5 | singulator F1 + F2 + F3, wrist F15, horn F16, cubesat F12, digest F12 |
| **T15** | **`validate.geometric_ok` false-positives.** Flips false on a solid every other gate calls clean; on the *second* of two mirror-image cuts; on polar patterns of off-axis tubes. | 3 | **3/10** | Medium — campaigns learned to ignore it, which is the worst outcome for a validity flag | ball F1, rotor F4, turgo F1 |
| **T16** | **`support_report` semantics are undocumented.** `build_dir` sign convention has an **empty `doc` string**; `overhang_deg` polarity is inverted from intuition; the threshold is a knife-edge at modelled face angles. | 2 filed | 2/10 filed | High by consequence — orientation prose was **wrong in 4 campaigns**, including one that shipped a render of the wrong bed | singulator F16, rotor F13, digest F10 |
| **—** | **Staleness has no guard anywhere in the toolchain.** `voxelize_stl.py` has no staleness check, so `ace_fea`/`ace_modal`/`ace_thermal` silently consume an out-of-date density field; `geometry_hash` differs between sibling receipts and **nothing reads it**. | 2 | 1/10 filed | **Critical by consequence** — this was a shipped BLOCKER in `iso9409` and a near-miss in 4 more | gripper F8 + F11 |

### 6.1 Recommended fix order for the maintainer

Ranked by *campaigns unblocked × severity*, not by entry count:

1. **T10 — give `assert` the rest of the spec's vocabulary, and let voxel-route
   bodies bind.** Either add `route` / `watertight` / `steep_area` / `thin_area` /
   `p05_thickness` / `fits_within` / mass assertions to the `assert` op, or let
   `hybrid_boolean` / `import_mesh` bind their result as a solid, or add a
   mesh→solid op (the 161-op surface has none). Until then, four mandatory §2
   gates are unexpressible and two campaigns' actual print files are ungateable.
   Three campaigns independently built out-of-engine replacements; a fourth
   campaign that copies the program but not the runner inherits the blind spot.
2. **T14 — fix the creep surface.** One reader, one table, a duration argument on
   `production_check.py`, at least one cell between 23 °C and 55 °C, and a receipt
   field recording *which cell* a margin was read at. This single fix would have
   prevented four of the nineteen blockers.
3. **T3 + T13 — make silence impossible.** Non-zero exit on `ok:false`; a
   synthesized `ok:false` receipt on timeout/kill (never a bare traceback); refuse
   a load case with no compressive pre-stress; refuse a fixture whose selector
   catches zero active voxels; never let a job-level `receipt` key clobber the
   caller's `--out`.
4. **T6 — fix or re-specify the connectivity gate.** Expose `weld_tol`; stop
   over-counting hole loops and `import_step` bodies; and **correct
   `DELIVERABLE_SPEC §2.2`**, whose oracle example is not constructible and whose
   stated rationale is inverted on this kernel (`components` is the weaker check,
   not the stronger). Three campaigns proved this independently.
5. **T5 — make `clearance` return a real distance for disjoint bodies**, or
   document that it cannot and bless `exact_volume` on an `intersection` body as
   the §2.11 measure (which is what 6 campaigns fell back to anyway).
6. **T4 — one path root.** Resolve every `file` against `--out-dir` consistently;
   stop echoing resolved paths into reports; stop writing station programs to a
   system temp dir.
7. **T2 — a body-fitted route that works, or an honest cost model.** If
   `ace_fea_tet` cannot mesh kernel-exported STLs at affordable sizes, campaigns
   need to know that *before* planning, not after a 90-minute stall. A
   `--wall-budget` with an honest `ok:false` receipt would help immediately.
8. **T1 + T16 — regenerate the digests from the binary** and fill the empty
   `describe` `doc` strings, starting with `support_report.build_dir`.
9. **Staleness guard** — an input-hash `--verify` mode on the runners, and a
   `voxelize_stl.py` freshness check. One line of tooling would have caught a
   shipped blocker.
10. **T7 — split wall-clock timings out of the physics payload** so receipts have a
    byte-comparable core, and state the determinism contract for solvers honestly
    in `SPEC §3`.

---

*Generated 2026-08-08 from the 10 stage-5 repair records, the 10 campaign friction
files, the shipped READMEs, and direct re-reads of the receipts. Every number in
§1 was spot-checked against its cited receipt file this pass (8/8 verified);
no engine or tools source was modified.*
