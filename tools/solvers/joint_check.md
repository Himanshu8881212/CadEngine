# joint_check — will the FASTENER hold, or will the plastic let go first?

**Runner**: `tools/analyzers/joint_check.py` ·
`python3 tools/analyzers/joint_check.py job.json [--out PATH]` (the
pre-2026-09-02 path `tools/joint_check.py` is a forwarding shim).
**Gates**: none — see Benchmark gates and Validity limits.
**Status**: no gate suite. **Tier**: **Cataloged** — the LOWEST tier in
`docs/ANALYSIS_TIERS.md`
(`python3 tools/analyzer_registry.py --tier joint_check` →
`{"tier": "Cataloged", "gate_suite": null, "manifest": null, "pins": []}`,
`tier_reason`: "Cataloged by the registry rule: Validated requires a present
manifest AND a present validation pin").

**What Cataloged means, plainly.** `docs/ANALYSIS_TIERS.md` §2 defines the bar
as "deterministic rules/arithmetic over cited tables/formulas", backed by
nothing more than being "correct-by-construction relative to its sources;
neither a physics simulation nor pinned; only as good as the tables and the
(usually 1-D) assumptions". The registry's own definition is the same:
"deterministic rules/arithmetic over published tables; correct relative to its
sources, not a physics simulation." So: this tool does arithmetic you could do
by hand, on typical published capacity numbers hard-coded in its own source. It
does not simulate anything, it is not checked against any pull test, and its
answer is exactly as good as those table entries. It sits below `Demonstrated`
because it has no ground-truth pin AND no physics — the registry's note says
"not validated against pull tests". Treat its safety factors as a screening
ranking, not as a capacity.

The reason it exists anyway: **the #1 assembly failure is the joint, not the
part.** Heat-set inserts pull out, threads strip out of plastic, and joints
creep under sustained load, and none of that is visible in a stress field of the
bracket. The steel side is almost never what governs.

## What it actually computes

A rules engine. Pure stdlib, no FEA, no engine calls, no geometry — everything
below is in `tools/analyzers/joint_check.py`, tables and source citations
together.

Per joint, capacities by mode. `d` is the nominal diameter (M3 3.0, M4 4.0,
M5 5.0 mm), `derate` is 1.0 or the sustained factor:

    heatset_pullout       = HEATSET_PULLOUT_N[size][material] * scale * derate
                            scale = clamp(insert_len_mm / std_len_mm, 0.5, 1.5)
    plastic_thread_strip  = tau(material) * pi * d * L_e * 0.5 * derate
    plastic_bearing_shear = 0.8 * (the joint's plastic tension capacity)
    screw_tension_steel   = SCREW_TENSION_N[size]        (steel, NOT derated)
    screw_shear_steel     = 0.6 * SCREW_TENSION_N[size]

Tables, with the sources the source file cites:

| table | values | source cited in the file |
|---|---|---|
| `HEATSET_PULLOUT_N` (N, at the standard insert length) | M3: pla 400, petg 350, abs 300, asa 300, pc 450, nylon 350 · M4: 650/550/480/480/700/550 · M5: 900/800/650/650/950/800 | CNC Kitchen pull-out tests (S. Hermann, "Threaded Inserts in 3D Prints — How strong are they?", 2019; M3 in PLA measured ~400..900 N across insert styles). Ruthex / E-Z Lok datasheet claims are HIGHER; these are the conservative low ends |
| `HEATSET_STD_LEN_MM` | M3 5.7, M4 8.1, M5 9.5 | Ruthex / E-Z Lok standard series |
| `PLASTIC_SHEAR_MPA` (printed) | pla 20.0, petg 18.0, abs 15.0, asa 15.0, pc 25.0, nylon 17.0 | datasheet bulk tensile (Prusament / Ultimaker), `tau ~ 0.6 * tensile`, derated for FDM layer adhesion (~60–80 % of bulk, CNC Kitchen layer-adhesion tests) |
| `SCREW_TENSION_N` | M3 2900, M4 5090, M5 8230 | ISO 898-1 class 8.8: proof 580 MPa × stress area (M3 5.03, M4 8.78, M5 14.2 mm²); A2-70 stainless within ~10 % |
| `THREAD_FORM_FACTOR` | 0.5 | plastics joining design guides (e.g. BASF *Mechanical fastening of plastics*, boss/thread chapters) — only ~half the `pi*d*L` cylinder shears as thread material |
| `HEATSET_SHEAR_FRACTION` | 0.8 | boss-geometry dependent, wall ≥ 2 mm around the insert ASSUMED |
| `SUSTAINED_DERATE` | 0.25 on ALL plastic-governed capacities | long-term stress-rupture curves / BASF design guide: sustained allowable ~ 1/4 of short-term. Steel modes are NOT derated |
| `HEATSET_LEN_SCALE_CLAMP` | 0.5×..1.5× | pull-out scales ~linearly with embedded knurl area; beyond the clamp the boss, not the insert, governs |
| `MIN_PLASTIC_THREAD_ENGAGE_FACTOR` | 2.0 (engagement ≥ 2·d) | plastic-thread design rule |

Every one of these is labelled `typical — verify` in the source. They are the
tool's entire authority.

Verdict logic. Per mode `SF = capacity / demand` (`inf` at zero demand). When
both tension and shear are present, an elliptical interaction is added on the
joint's weakest tension/shear pair:

    SF_combined = 1 / sqrt( (T/Tc)^2 + (S/Sc)^2 )
    Tc = min capacity over the tension modes, Sc = min over the shear modes

The **governing mode is the minimum SF over all modes**, and `pass` is
`SF_actual >= safety_factor` (default 2.0). A minimum-engagement violation
short-circuits all of it: `governing_mode = "min_engagement_rule"`,
`SF_actual = 0.0`, `pass = false` — engagement ≥ insert length for heat-set,
≥ 2·d for plastic threads.

Assumptions baked in: 1-D per-mode capacities with no interaction beyond the
ellipse; a properly installed insert (melted in, flush, no boss cracks); a boss
with ≥ 2 mm wall; no preload, no thread friction, no torque model; no geometry
of any kind is read.

## The contract (job → receipt)

Job: `joints: [{name, type, size, material, loads, engagement_mm,
insert_len_mm?}]` plus `safety_factor` (default 2.0). `type` ∈
`machine_screw_into_heatset` | `screw_into_plastic_thread` | `bolt_through_nut`;
`size` ∈ M3 | M4 | M5; `material` is **the PLASTIC side** ∈ pla | petg | abs |
asa | pc | nylon (default `petg`); `loads` = `{tension_N, shear_N, sustained}`
as non-negative finite magnitudes.

Receipt (LAST non-empty stdout line, logging to stderr; persisted per
`tools/_receipt.py` with `use_out_dir_default` on): `ok` (all joints pass),
`safety_factor_required`, `data_caveat` (the typical-values warning, on every
receipt), `joints: [{name, type, size, material, governing_mode, capacity_N,
demand_N, SF_actual, pass, modes: {mode: {capacity_N, demand_N, SF}}, notes}]`,
and `refused_joints` when any joint was refused.

Refusals are **per joint**, by exact `error_kind` string:
`refusal.size_not_in_table`, `refusal.material_not_in_table`,
`refusal.unknown_joint_type`, `refusal.invalid_load`,
`refusal.invalid_geometry`, `refusal.missing_type`, `refusal.missing_size`,
`refusal.bad_joint`; plus the job-level `refusal.no_joints`. A refused joint
gets NO capacity number and fails the run, but **the other joints keep their
evidence** (a whole-receipt `KeyError` used to throw all of it away — din_rail
F1's sibling defect). When any joint is refused, the top-level
`error_kind` is `refusal.joint_refused`. There is no conservative default for an
out-of-table size or material: the tables are the tool's whole authority, so
extrapolating one would be an invented number, and it refuses instead.

Exit codes follow the shared contract in `tools/_receipt.py`: **0** `ok:true`,
**1** the tool could not run the request, **2** it ran and REFUSED or the
verdict is FAIL. **The exit code agrees with the engineering verdict.** Before
2026-08-08 it was inverted in practice (ball F5): an out-of-table size leaked
`{"ok": false, "error": "KeyError: 'M6'"}` at exit 1 while a genuine
`min_engagement_rule` FAIL exited 0, so `$?` carried the opposite of the answer.

## Benchmark gates (measured, frozen)

**None. This analyzer has no benchmark gate suite.** The registry records
`gate: no` for it (`python3 tools/analyzer_registry.py --tier joint_check`
returns `"gate_suite": null`); there is no manifest under `tools/manifests/` and
no validation script under `tools/validation/`.

What exists is a set of **behavioural pins** in `tools/tests/test_checkers.py`
(`python3 tools/tests/test_checkers.py`, section `joint_check.py`; all pass in
this checkout). They pin exit-code polarity, refusal naming, and the shipped
receipt shape:

| pin | what it holds | what it does NOT pin |
|---|---|---|
| PASS exits 0, a genuine FAIL exits 2 with `ok:false` (ball F5) | the exit contract is no longer inverted | any capacity |
| an M6 joint is `refusal.size_not_in_table` with `capacity_N: null`, `refused_joints: ["m6"]`, exit 2 | out-of-table inputs refuse by name, not by `KeyError` | — |
| a refused joint does not discard the other joints' evidence | per-joint refusal isolation, with `good` keeping `SF_actual == 8.0` | — |
| an out-of-table material is `refusal.material_not_in_table` | — | — |
| receipt shape: M3 heat-set in PLA at 50 N tension → `governing_mode "heatset_pullout"`, `capacity_N 400.0`, `SF_actual 8.0`, `pass true`, modes exactly {`heatset_pullout`, `plastic_bearing_shear`, `screw_tension_steel`, `screw_shear_steel`} | the arithmetic reproduces the tool's OWN table (400 N / 50 N = 8.0) | that 400 N is the right capacity |
| a negative load magnitude is `refusal.invalid_load` at exit 2 | a negative demand cannot become an infinite SF | — |

`campaign/digests/tools_cookbook.md` additionally records a worked case: M3
heat-set in PLA under 100 N tension + 50 N shear → **SF 3.39, pass**. That is
reproducible by hand from the tables above (`Tc = 400`, `Sc = 320`,
`1/sqrt((100/400)² + (50/320)²) = 3.392`) — which is the point: it confirms the
arithmetic, and confirms nothing about the newtons.

**No capacity this tool returns has been compared with a pull test, a torque
test, or any independent measurement in this repo.** The table values carry
their published sources, and the CNC Kitchen source itself spans **~400..900 N
for M3 in PLA** across insert styles — the tool takes the low end, so the spread
between insert brands is roughly a factor of two and is not modelled. Every
receipt carries `data_caveat` for this reason.

## Validity limits / out of scope

- **The tables are typical published values, not guarantees.** Printed parts
  vary with layer adhesion, nozzle and bed temperature, moisture, infill, and
  insert brand. Verify per insert brand / filament / print settings before
  trusting a safety-critical joint. The spread is real: see the ~400..900 N M3
  range above.
- **Only M3, M4, M5, and only six plastics.** Everything else refuses by name.
  That is the correct behaviour, not a gap to be filled by extrapolating the
  table.
- **The sustained-load rule is a time-blind blanket, and it conflicts with this
  repo's creep authority.** `SUSTAINED_DERATE` applies a flat ×0.25 to all
  plastic-governed capacities regardless of duration or temperature, and this
  tool never reads `tools/analyzers/materials.py` or the researched creep table
  in `tools/materials/pla.json#creep`. `tools/solvers/creep.md` records the
  ruling that for sustained load **the table governs** and that exactly this
  class of blanket fraction is superseded by it. The two have not been
  reconciled: a sustained joint number from here is a different, unreconciled
  basis from a sustained margin from `production_check`. Say which you used, and
  prefer the creep table for the plastic allowable.
- **No geometry is read at all.** Boss diameter, wall thickness around the
  insert, edge distance, hole tolerance, and the print orientation of the boss
  are invisible. `HEATSET_SHEAR_FRACTION = 0.8` simply ASSUMES a ≥ 2 mm wall; a
  thin or ribbed boss is outside the rule and the tool cannot tell.
- **Heat-set minimum engagement is only checked when you supply
  `engagement_mm`.** Omit it and the `>= insert length` rule is silently not
  applied (the plastic-thread branch instead treats a missing engagement as
  `L = 0` and fails). Always pass it.
- **Anisotropy is not modelled.** `PLASTIC_SHEAR_MPA` is a single number per
  material with the layer-adhesion derate already folded in; there is no
  in-plane vs across-layer distinction, and the tool does not apply
  `materials.py derated()`. A boss loaded to pull the insert straight out along
  the layer normal is the worst case and gets no extra penalty here.
- **`bolt_through_nut` is a steel-only check.** Plastic compression and creep
  under the head or nut is explicitly NOT modelled — the receipt's own note says
  to use washers on printed faces.
- **No preload, torque, thread friction, gasket, or vibration-loosening model.**
  No thermal effects, no re-installation cycles (a heat-set insert reheated and
  reseated is not the same joint), no fatigue — repeated actuation is `fatigue`,
  not this tool.
- **The interaction ellipse is a convention, not a measured envelope for
  printed plastic.** No printed-joint biaxial dataset backs it here.
- **Not characterised: the error band.** No study in this repo establishes how
  far any of these capacities is from a real pull-out or strip load for a
  printed boss. Until one exists, treat `SF_actual` as unbounded error, use it
  to RANK design variants and to catch obviously overloaded joints, and do not
  gate a safety-critical claim on it.

## When to use it

At the point a design commits to fasteners: choosing between a heat-set insert,
a screw threaded straight into plastic, and a bolt through a captive nut; sizing
a boss's engagement depth; checking that a lid screw carrying a preload or a
hinge pin carrying shear has margin; deciding whether a joint that will be
loaded for months needs a different strategy. Run it early — the answer usually
changes the boss geometry, which is cheap before the part is drawn and expensive
after. Then quote the governing mode, not just the SF, and carry the
`data_caveat` with the number.
