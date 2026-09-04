# balance_check — how far off the spin axis is a rotating assembly's mass?

**Runner**: `tools/analyzers/balance_check.py` ·
`python3 tools/analyzers/balance_check.py job.json [--out PATH]` (the
pre-2026-09-02 path `tools/balance_check.py` is a forwarding shim).
**Gates**: none — see Benchmark gates and Validity limits.
**Status**: no gate suite. **Tier**: **Demonstrated**
(`python3 tools/analyzer_registry.py --tier balance_check` →
`{"tier": "Demonstrated", "gate_suite": null, "manifest": null, "pins": []}`,
`tier_reason`: "Demonstrated by the registry rule: Validated requires a present
manifest AND a present validation pin").

A printed rotor that is 0.2 mm off axis is not a cosmetic problem: the 1×-rev
force it throws grows with the SQUARE of speed, and it is carried by a plastic
bearing seat and a plastic boss. Propellers, flywheels, spool hubs, turbine and
pump rotors, anything driven by a motor — the question "where is the mass
relative to the axis it will actually spin about" has to be answered before the
part is printed, because it is a consequence of the geometry, not of the print.

This tool answers it by running the engine's `mass_properties` measure (with the
full `inertia_tensor`) on each part, combining the parts with real densities,
and reporting the assembly's CG offset, single-plane static imbalance, and the
couple (product-of-inertia) terms about the declared spin axis.

**It measures. It imposes no balance grade** — `ok: true` means the measurement
succeeded, not that the rotor is balanced. See Validity limits.

## The physics

Per part, the engine returns volume `V` in mm³, centre of mass in mm, and the
**unit-density** inertia tensor about the CoM in mm⁵. The tool's unit trail,
stated so the receipt is auditable:

    m       = rho * V * 1e-9                      [kg]     (rho in kg/m^3, V in mm^3)
    I[kg m^2] = tensor_mm5 * rho * 1e-15
    couple terms reported in g*mm^2 = I[kg m^2] * 1e9

Parts are combined about the spin-axis point `p0` by the parallel-axis theorem,
with `d` the CoM offset from `p0` in metres:

    I_p0[i][j] = sum_parts ( tensor[i][j] * rho * 1e-15
                             + m * ( (d.d) delta_ij - d[i] d[j] ) )

The assembly CG is the mass-weighted mean of the part CoMs. With `w` the unit
spin direction:

    r        = cg - p0
    r_perp   = r - (r.w) w
    cg_offset_mm       = |r_perp|
    static_imbalance   = mass_g * cg_offset_mm         [g mm]   (single-plane U = m r)

An axis-normal right-handed frame `(u, v, w)` is built deterministically: the
seed is whichever world axis is LEAST aligned with `w`, then
`u = normalize(seed × w)`, `v = w × u`. The couple terms are the two
off-diagonal entries that couple the axis to that plane:

    I_uw = u^T I_p0 w ,   I_vw = v^T I_p0 w      (g mm^2, about p0)
    magnitude = hypot(I_uw, I_vw)

reported in the dynamics convention `I_uw = -integral(u w dm)`, so both are zero
for a dynamically balanced rotor. Static imbalance and couple imbalance are
different failures: a rotor can have its CG exactly on the axis and still wobble
because its mass is distributed asymmetrically along the axis.

When `spin_rpm` is given:

    omega = 2 pi rpm / 60
    est_wobble_force_N = m[kg] * cg_offset[m] * omega^2

The formula string is on the receipt. Assumptions baked in: rigid bodies,
uniform density per part, poses already baked into each part's ops, the spin
axis as declared (not derived from the geometry), and no bearing/support
compliance anywhere in the chain.

## The contract (job → receipt)

Two geometry forms. Form 1, one merged solid: `ops`, `solid` (op id),
`density_kg_m3`. Form 2, per-part list: `parts: [{name, ops, solid,
density_kg_m3}, …]` with each part's pose already in its ops. Both forms need
`spin_axis: {point: [x,y,z], dir: [x,y,z]}`; `spin_rpm` is optional. Optional
`program_dir` (else `out_dir`, else the job file's own directory) is where each
part's measurement program is materialised — and therefore the root its relative
`import_step`/`load_part` paths resolve against, never a system temp dir.

Receipt (LAST non-empty stdout line, logging to stderr; persisted per
`tools/_receipt.py` with `use_out_dir_default` on, i.e.
`<out_dir>/balance_check_receipt.json` unless `--out` or a job `receipt` key
says otherwise): `ok`, `mass_g`, `cg_mm`, `cg_offset_mm`,
`static_imbalance_g_mm`, `couple_terms` (`I_uw_g_mm2`, `I_vw_g_mm2`,
`magnitude_g_mm2`, and a `frame` string spelling out the convention),
`formula`, `per_part` (`name`, `mass_g`, `com_mm`, `volume_mm3`), plus
`spin_rpm` and `est_wobble_force_N_at_rpm` when an rpm was given.

Refusals, by exact `error_kind` string: `refusal.missing_spin_axis` (no
`spin_axis` with both `point` and `dir`), `refusal.degenerate_axis` (`dir` norm
below 1e-12), `refusal.part_program_failed` (a part's engine program failed —
the failing part is named), `refusal.zero_mass`. Exit codes follow the shared
contract in `tools/_receipt.py`: **0** `ok:true`, **1** the tool could not run
the request, **2** it ran and REFUSED. Note that a form-1 job missing `solid` or
`density_kg_m3` is a plain `KeyError`, i.e. exit 1 with `error_kind: internal`,
not a named refusal.

## Benchmark gates (measured, frozen)

**None. This analyzer has no benchmark gate suite.** The registry records
`gate: no` for it (`python3 tools/analyzer_registry.py --tier balance_check`
returns `"gate_suite": null`); there is no manifest under `tools/manifests/`,
no validation script under `tools/validation/`, **and no test anywhere in
`tools/tests/`** — unlike its three sibling checkers, this tool has not one
regression pin. Its only mention outside its own source and the layout map is
the registry, `tools/_receipt.py`'s persistence audit, and
`tools/publish/document_bundle.py`, which consumes its receipt.

Every number this tool returns is therefore unvalidated against an independent
reference, and nothing in this repo would catch a regression in its arithmetic.

Two observations ARE recorded in the campaign digests. They are digest-recorded
campaign results, **not gates**, and nothing re-runs them:

- `campaign/digests/tools_cookbook.md`: "VERIFIED on a centered box: all zeros,
  `ok:true`" — a self-consistency spot check, not a reference comparison.
- `campaign/digests/exemplars.md` §2.9: the squatchee propeller measured
  **CG 1.8e-9 mm off axis → 0.0 g·mm static imbalance, couple terms 0.0 g·mm²**,
  a balance achieved by construction (exact y-mirror, 180° blade symmetry,
  axisymmetric bore) and then measured. That is a zero-check on a symmetric
  part: it exercises the cancellation, not the magnitude.

Neither establishes an error band, and no non-zero imbalance from this tool has
been compared with a measurement or a closed form in this repo.

## Validity limits / out of scope

- **`ok: true` is NOT a pass.** The tool measures and deliberately imposes no
  balance grade. A shell gate on its exit code gates that the measurement ran,
  and nothing about the rotor. The allowable is the caller's to state.
- **There is no balance-grade allowable in this repo.** No ISO 1940 / G-class
  table, no `g·mm per kg at rpm` limit, no printed-rotor acceptance criterion
  exists in `tools/materials/` or anywhere else in-tree. What imbalance is
  acceptable for a given part at a given speed is **not characterised** here —
  cite an external standard explicitly, or state the criterion you chose and
  why. Do not let this receipt imply one.
- **Accuracy is inherited from the engine, and unbounded here.**
  `mass_properties` is analytic for planar / cylinder / sphere / cone-faced
  parts (π-exact volumes), but nothing in this repo pins the `inertia_tensor`
  measure — nor pins that its sign convention agrees with the
  `I_uw = -integral(u w dm)` convention this tool assumes when it adds the
  parallel-axis term. The couple terms are right only if they agree, and that
  agreement is **not characterised**. Sanity-check a couple term on a part whose
  answer you know before gating on one.
- **`est_wobble_force_N_at_rpm` is the STATIC (single-plane) force only** —
  `m r omega^2` from the CG offset. It does not include the couple's
  contribution, so a rotor with `cg_offset_mm ≈ 0` and large `I_uw`/`I_vw`
  reports a near-zero force and will still wobble. Quote the couple terms beside
  it, always.
- **As-designed, not as-printed.** Uniform density per part, exactly as
  modelled. Real imbalance from asymmetric infill, seam placement, moisture,
  layer voids, warp, and part-to-part variation is entirely outside this
  calculation. A balanced-by-construction geometry is a necessary condition, not
  a balanced part. (The squatchee campaign carried "symmetric infill preserves
  balance" into its print settings for exactly this reason.)
- **Rigid, no dynamics.** No bearing or shaft compliance, no gyroscopic effects,
  no critical-speed / whirl analysis, no aerodynamic or hydrodynamic loading, no
  transient run-up. "Will this rotor pass through a resonance" is `modal`, not
  this tool.
- **The spin axis is declared, not derived.** Give it wrong — wrong point, wrong
  direction, a shaft axis that is not the axis the bearing actually defines —
  and every number is confidently wrong.
- **Fasteners, magnets, inserts and other non-modelled mass are invisible.** The
  assembly is the parts you list, at the densities you give.
- **Needs the engine binary** (`kernel-api`, via
  `param_optimize.call_engine`); a missing one is a loud error.

## When to use it

Any part that spins under power or at speed: propellers and fans, flywheels,
turbine and pump rotors, spool and reel hubs, centrifuge carriers, drive pulleys
and capstans, tool holders. Run it on the AS-DESIGNED assembly with the real
spin axis and the intended rpm, quote `cg_offset_mm`, `static_imbalance_g_mm`
AND `couple_terms.magnitude_g_mm2` together, and state which allowable you are
holding them to and where that allowable came from — because this tool does not
supply one.
