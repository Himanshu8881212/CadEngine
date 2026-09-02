# ANALYSIS HONESTY CONTRACT — campaign digest

This is the contract that keeps an agent from claiming things the engine cannot
back. Every number you quote must trace to (a) a pinned solver whose card's
validity limits include your case, (b) a cited derived model that passed its
gates this run, or (c) a bold **"required, NOT performed"** row. Silence about a
required analysis is the one forbidden outcome (DESIGN_GUIDE §25.7).

Repo root (space in path — ALWAYS quote): `"/Users/himanshu/Work/New-LMCAD/cad engine"`

```sh
# Geometry engine CLI — note the REQUIRED `run` subcommand:
"/Users/himanshu/Work/New-LMCAD/cad engine/target/release/kernel-api" run program.json --out-dir OUT
# assemblies:
"/Users/himanshu/Work/New-LMCAD/cad engine/target/release/kernel-api" asm a.lmcasm --out-dir OUT [--voxel MM]
# Solver runners (plain python3 works — tools/_ace.py puts ~/Work/ACE on sys.path automatically):
python3 tools/ace_fea_runner.py job.json        # etc.
```

---

## 1. The trust ladder (docs/ANALYSIS_TIERS.md)

Every analysis result carries a `validation_status` inside the
`lmcad.analysis.v1` envelope (built by `tools/provenance.py stamp(...)`):

| status | bar |
|---|---|
| `validated` | committed manifest (`tools/manifests/`) AND >= 1 ground-truth pin that CI runs. ONLY these may be quoted as validated numbers. |
| `demonstrated` | runs end-to-end with self-checks, no ground-truth pin |
| `cataloged` | deterministic arithmetic over cited tables (ISO fits, deratings) — never a physics simulation |
| `synthesized_inloop` | on-the-fly model whose self-check PASSED this run |
| `synthesized_unvalidated` | self-check absent/failed — must not surface unmarked |
| `research` | fenced domains, exploratory only |

`stamp()` refuses a bare-scalar `residual_or_convergence` and unknown statuses.
A synthesized result can NEVER claim `validated` — no code path exists.
Ledger: `python3 tools/analyzer_registry.py`. As of 2026-07-17: 14 surfaces,
5 Validated (ace_fea, ace_modal, ace_buckling, ace_optimize, param_optimize),
64.3% honestly below the line (thermal/contact/fatigue cards are green in-house
gate suites; check the registry before claiming tier).

Geometry hashes: `program:sha256:<hex>` (sorted-key canonical JSON) or
`mesh:sha256:<hex>` (order-independent STL canonicalisation). Two reps of the
"same" solid do NOT share a hash — the relation is `equality` or
`derived_from` (with a stated `error_bound`), as data.

---

## 2. The six solvers (`tools/solvers/` — read the card before quoting)

Universal receipt contract: **the LAST non-empty stdout line is one JSON
object**; logs go to stderr. Exit-code split (verified):

- **Every runner, one contract** (`tools/_receipt.py`, since 2026-08-08): the
  `{ok:false, error, error_kind}` JSON line is still the contract, and the exit
  code now agrees with it — **0** ok · **1** could-not-run · **2** ran-and-
  REFUSED / analysis failed. The ACE-bridge runners (`ace_fea`, `ace_modal`,
  `ace_buckling`, `ace_fea_tet`, `ace_optimize`, `graded_infill`) used to exit
  0 on failure and no longer do. Any nonzero: do not quote the receipt.

All six are as-designed, isotropic, homogeneous: **no solver models printed-layer
anisotropy inside the solve**. Apply `tools/materials.py derated()` to the
ALLOWABLE, never to E.

### 2.1 ace_fea — hex8 linear-elastic voxel FEA (Validated)
- Physics: `K u = F`, von Mises; small strain + small displacement; no
  plasticity/contact/geometric nonlinearity. Optional SIMP density mode
  (stress then homogenized, NOT solid-material stress).
- Job JSON: `{out_dir, voxel_mm, origin_mm?, ops+solid+shape[+supersample] | npy,
  material: "PLA"|{youngs_modulus_pa, poisson, density_kg_m3},
  fixtures: [{kind: clamped|pinned|slider, region_selector, dof_constrained?}]  (REQUIRED),
  loads?: [{kind: point|body|pressure, magnitude, direction?, region_selector}],
  regions?, simp_penalty?, density_floor?, direct_solver_max_dof?}`.
  Outputs `stress_field.npy` (per-ELEMENT von Mises, Pa), `disp_field.npy`.
- Band (pinned): cantilever deflection **-11.2% at voxel 1.0, -5.9% at 0.5** —
  under-predicts (hex8 stiff), converges from below; asserted coarse (-20%,0),
  fine (-10%,0).
- HARD LIMIT: fillet/notch PEAK stress is staircase-dominated — **±20-30%,
  biased high, does NOT converge to Kt** under refinement (Kt pin: scatter
  -6%..+44%, plateaus +20..29%). Use closed-form Kt × the FEA's accurate
  NOMINAL stress (<1% error) instead of the voxel peak. Quote feature stresses
  as approximate below ~4 voxels across the feature.
- Loads catching >30% of active elements are flagged "suspiciously broad".
- GridField hand-off: fields per-element → GridField origin = job origin + voxel/2.

### 2.2 modal — natural frequencies + mode shapes (Validated)
- Physics: `K φ = ω² M φ`, undamped, about the UNLOADED state. No damping (no
  amplitudes/Q), no stress stiffening (preloaded/spinning part: say so, don't quote).
- Job adds: `free_free?: true` (EXPLICIT opt-in — no fixtures without it =
  **refusal**, never silent), `n_modes?: 6`; accepts `stl+shape` input.
  Density must be > 0 (refused otherwise).
- Band: frequencies a-few-percent HIGH on coarse grids (**+2.6/+1.3% at vox
  0.75, +1.4/+0.2% at 0.5**); resolve bending thickness with ≥ 4 voxels.
  Cross-pinned vs ACE reference to 3e-13.
- Mode IDENTIFICATION is on the caller: use the `participation` receipts
  (effective mass + kinetic fraction per axis) — a bare sorted frequency list
  mixes bending/torsion/axial families.

### 2.3 buckling — linear eigenvalue buckling (Validated)
- Physics: bifurcation on PERFECT geometry: static pass + `K φ = -λ K_g φ`.
  **λ is an UPPER bound** — real structures buckle earlier. First field of
  every receipt is this caveat.
- MANDATORY: apply `knockdown.recommended_factor = 0.5` → use
  `design_critical_load_n` in gates, NEVER raw λ. For shell-like modes even
  0.5 can be unconservative (NASA SP-8007 class 0.32-0.65).
- Band: Euler column **+6.3% at vox 0.75, +3.4% at 0.5**, over-prediction only.
- REFUSES: zero/absent reference load; no compressive stress anywhere; no
  positive eigenvalue. Moment loads skipped (no rotational DOFs). Slender
  ELASTIC members only (pinned at L_eff/r ~ 100) — stocky columns yield first;
  pair every buckling gate with a strength gate.

### 2.4 thermal — voxel heat conduction, steady + transient (in-house, exits 1)
- Physics: conduction only; convection ONLY as a Robin film `h_c` you supply.
  No radiation, no internal convection/CFD, no T-dependent k, no phase change,
  no contact resistance, single material per job.
- Job: `{out_dir, voxel_mm, npy | stl+shape | shape+solid:"full",
  material: "PLA"|{k_w_mk,...}, bcs: [{kind: fixed_t|flux|convection, box_mm,
  faces?}], sources?, probes_mm?, transient?: {t_initial_c, dt_s, t_end_s}, ...}`.
  Output `T_field.npy` per-VOXEL + receipt with energy balance.
- Band: planar axis-aligned faces exact to solver tolerance; curved surfaces
  staircase at ~O(h), a few % at ~2 voxels per feature radius. Transient is
  backward Euler — unconditionally stable, first-order in dt (need dt << time
  of interest for early-time answers).
- REFUSES: no BCs, zero k, empty grid, any connected solid component not
  anchored by a fixed_t/convection BC (never guessed).

### 2.5 contact — geometrically-nonlinear planar beam + rigid-obstacle contact (in-house, exits 1)
- The snap-fit/latch/living-hinge solver. Corotational Euler-Bernoulli beam,
  exact for large rotation + small local strain; penalty contact against
  plane/cylinder/profile obstacles (optionally translating → insertion curve).
  Pinned against the exact elastica (err 1.4e-5 at 80 elements).
- Why not ace_fea: linear FEA over-predicts snap-arm stiffness badly (elastica
  δ/L = 0.603 where linear says 1.00).
- Receipt: `insertion{peak_force_n, ...}`, `path_max` (worst stress/strain over
  the whole path — a sprung-back latch ends unstressed), linear vs nonlinear
  side by side, per-step convergence. Curve in `curve.npy`.
- LIMITS: PLANAR only (no 3-D, torsion, out-of-plane buckling, plate width
  effects); Euler-Bernoulli (few % stiff below L/t ~ 10); friction (`mu > 0`) is
  **untested capability** — indicative only, say so; quasi-static (no snap-back
  "click" dynamics); elastic (check `path_max.strain` vs allowable + creep
  table for held deflections); snap-through/limit points **REFUSE** (no
  arc-length continuation) — non-convergence raises, never a silent last iterate.
  Sharp zero-radius obstacle corners stall Newton — fillet them.

### 2.6 fatigue — S-N stress-life, SCREENING ONLY (in-house, exits 1)
- Basquin + mean-stress correction + Palmgren-Miner, post-processing an
  `ace_fea` stress field or scalar hot-spot. **Comparative/screening tool for
  ranking variants — NOT a certification basis.** FDM fatigue is dominated by
  layer adhesion/defects; the printed-PLA 90/10-survival band spans
  **3.7x–90x in LIFE** and is recomputed into every receipt. Never quote a
  cycle count without that band.
- Data (tools/materials/fatigue.json): **PLA is the ONLY material with a life
  number** (Ezeh & Susmel 2019, 143 printed specimens, ≤ 2e6 cycles,
  independently corroborated to 0.42% on the Basquin coefficient).
- **MUST-REFUSE list (verified: refusals are the deliverable, exit 1 with a
  named reason + "what would change this")**:
  - PETG (`insufficient` — one paywalled study), ABS (`insufficient`),
    ASA / PA / PC / TPU95A (`unknown`)
  - `load_orientation: "across_layer"` for ANY material — no printed Z-load
    S-N dataset exists anywhere; the static 0.55 Z-ratio is not a fatigue slope
  - Goodman stacked on the max-stress (`intrinsic`) curve — double counting
  - peak stress above printed UTS (that's static failure, not fatigue)
- Defaults/conventions: `curve: "design"` (PS ≥ 90%) is default; `median` is
  never a default. Von Mises is UNSIGNED → R-ratio is a DECLARATION (default
  0.0). Compressive mean gets NO credit. Anything past 2e6 cycles is flagged
  `extrapolated_beyond_curve_validity` — there is no evidence of a true
  endurance limit for printed PLA. Calibration set: 100% infill, RT, dry,
  in-plane, 5–10 Hz — sparse infill (~40x life loss at 25%), heat, moisture,
  high frequency are OUT. No notch factor (apply Kf yourself: source measures
  16-29% knockdown — and say so). Miner rule itself is NOT validated for
  printed polymers (receipt says so).
- Canonical fatigue UTS = **40.9 MPa** (lowest measured printed), deliberately
  NOT pla.json's 55/60 datasheet numbers — recorded conflict, not averaged.

### 2.7 Optimizers
- `ace_optimize` (SIMP topology, Validated: exact inequality pins) and
  `param_optimize` (target-driven optimization over ANY receipted analyzer,
  Validated: analytic optima to ~1e-7, byte-identical determinism). An eval
  with `ok:false` is a failed eval, never silently scored.

---

## 3. In-scope vs MUST-REFUSE (docs/ANALYSIS_DOMAINS.md + the research fence)

**No in-tree solver exists for** (say so; reduce to a cited tier-(b) 1-D model
with stated limits, or bridge externally — never present a lumped estimate as a
field solution):
- 3-D fluid flow / aerodynamics (CFD) — a declared gap, fenced
- 3-D electromagnetic fields / motor magnetics — fenced
- Conjugate heat / natural-or-forced convection networks / thermal-CFD — fenced
  (conduction with a user-supplied film coefficient is the in-scope ceiling)
- Nonlinear 3-D FEA: plasticity, 3-D contact, large deformation, hyperelastics
  (the planar contact beam is the only nonlinear path)
- Fatigue beyond cited S-N arithmetic; crack growth da/dN; PETG/ABS/ASA/PA/PC/
  TPU fatigue; across-layer fatigue in ANY material
- FEA/CFD directly on NURBS/freeform faces (validated path = voxelized hex8)
- Fenced (`research`) outputs must carry `validation_status: "research"` and be
  presented as exploratory. No auto-promotion — graduation needs manifest +
  pin + registry row + CI.

JSON-surface note: there are still **no FEA/load-case ops in programs** — the
JSON analysis vocabulary is mass properties, wall thickness, draft, clearance,
support_report, thin_wall, min_ligament. Physics runs through the Python
runners on exported geometry.

---

## 4. Physics outside the built-ins: `tools/derived_model.py`

The only honest path for a new domain (acoustics, RC thermal networks, beam
formulas, linkages, magnetic circuits...):

1. Subclass `DerivedModel` — **cannot import** without `equations`,
   `assumptions`, `units`, `limits_of_validity`, `discretization`, `measured`
   date, and ≥ 1 real `sources` citation (`__init_subclass__` refuses).
2. Every run re-executes the validation gates against closed forms and
   **refuses to evaluate if any gate fails** ("refuse-before-run").
3. Results ship as `synthesized_inloop` (never `validated`) with a structured
   convergence receipt; re-checked by `provenance.check_synthesized` before
   printing. Last stdout line = envelope JSON → `param_optimize` can drive it.
4. Criterion for existence: **if you cannot write a gate against an independent
   closed form or known limit, the model must not run.**
5. Start: `python3 tools/derived_model.py --new my_domain`. Commit the manifest
   to `tools/manifests/derived/` → auto-registers at Demonstrated; Validated
   only when a pin file lands.

Synthesis guardrails (mandatory, ANALYSIS_TIERS §3): manifest BEFORE reporting;
self-check `{limit, expected, obtained, passed=true}`; structured residual
(bare scalars refused by `stamp()`); sandboxed execution; pinned versions/seeds.

External-solver bridge: geometry OUT via `export_stl`/`export_step` or
`tools/voxelize_stl.py` (STL → occupancy `.npy`, same contract the ACE runners
consume); numbers BACK only wrapped by `provenance.stamp` with an honest
status. STL↔program relation is `derived_from` with the chord tol — not equality.

---

## 5. Print-readiness method (DESIGN_GUIDE §22)

1. Design printable features from the catalog: `teardrop_hole` (horizontal
   bores, 45° crown), `bridged_counterbore` (sacrificial membrane, genus
   receipt proves it), `heatset_insert_boss` (undersized Ruthex pocket;
   `heatset_spec` for numbers).
2. Voxel from the §17.4 table (FDM 0.3–0.4 mm at 0.2 mm layers) and drive the
   watertight receipt green — for lattices the receipt, not the table, wins.
3. Wall-thickness gate: `wall_thickness` with `flag_below` = min printable
   wall; judge `thin_area` + `p05_thickness` (min_thickness alone is corner noise).
4. Overhang audit on the exported STL: triangles with nz < -cos45° above the
   bed are violations; gate on violating AREA vs a negative control. Copy
   `iphone_stand/tools/overhang_audit.py` (checks signed volume first).
5. Datum-plane probe for anything blended: §14 pillow check (bbox z_min == bed
   plane) — receipts smiled through a 0.46 mm pillow; probe explicitly.
6. Fits: FDM 0.4 mm nozzle → ~0.2–0.3 mm designed radial clearance on push
   fits; MEASURE every fit with `assert_disjoint`/`contacts`, don't trust the slicer.
Campaign additions (§25): per part, validate → **connectivity
(`Mesh::is_one_body`)** → pose → `support_free_report(Z, 45.0, 0.3)` →
watertight → bed-fit → export; one negative control per oracle (a gate that
cannot fail is not a gate).

---

## 6. Failure playbook (DESIGN_GUIDE §23 — all provoked against the current binary)

Execution stops at the first failing op; errors carry a machine-matchable `kind`:

| kind | fix |
|---|---|
| `parse` | fix the `{"ops":[...]}` envelope |
| `unknown_op` | spelling — message (verified): "not one of the **160** supported ops; call the `describe` op to enumerate them" (`box`, not `cube`) |
| `duplicate_id` | ids unique per program |
| `missing_ref` | reference a geometry-BINDING op (implicit/measures/exports/design-math bind nothing) |
| `wrong_type` | route sketches through `sketch_extrude`/`sketch_revolve` |
| `invalid_param` | message names it; empty boolean → check poses; prove emptiness with `assert_disjoint` |
| `feature_failed` | fillets/chamfers: witness must be near a CONVEX straight edge between planar faces or a convex circular rim; concave junctions are out of scope for BOTH — move witness, shrink radius, or use the quarter-round cove workaround |
| `sketch_failed` | conflicting constraints / open profile |
| `invalid_geometry` | CW profile (wind CCW); implicit tree not watertight at voxel → refine voxel / switch mesher / bury tangent contacts; export leaky even after heal |
| `admission_rejected` | library sample failed to rebuild — shrink declared ranges |
| `dependents_exist` | library_remove refused — deprecate or `force` under VC |
| `assert_failed` | design ≠ declared intent: fix geometry or a wrong expectation, NEVER delete the gate |
| `io` | paths; `load_part` resolves relative to the program file |
| `internal` | kernel bug — report with the program |

**When booleans refuse** (empty result, or `try_*` withholds an invalid solid):
check poses; use `assert_disjoint`/`assert shells` for proofs; lift
exact-contact poses ~0.05–0.1 mm (coplanar overlap is the least-margin corner);
prefer small embedments; `boolean_hazards` while authoring, `ChainLog::seal()`
past ~10 ops; ease (fillet/chamfer) primitives FIRST, boolean LAST — post-
boolean fragmented planes make edge features fail.

**When meshes come back non-watertight**: program exports auto-heal through the
winding-number SDF at `voxel` (default 0.3) → `"route": "voxel_healed"`; still
leaky → op FAILS (a program never writes a garbage file). ASSEMBLY instance
exports NEVER fail the run — receipt carries `"watertight": false`, exit stays
0, and YOUR policy layer must gate on the per-instance receipt. Known heals:
symmetric hole-row panels (nudge datum 1.5 mm or accept heal), the 64-feature
housing (leak is mesh-side; arbitrate surprising contacts with exact booleans).

**Silent modes → tripwires** (no exit code fires): misspelled optional param →
default silently in effect (**the engine ignores unknown params — assert a
measure the param drives, always**); negative radius absolutized → volume
window; hole loop crossing outer → volume window; rim-fillet witness snapping
to a far rim → measure after fillet; under-declared Lipschitz → silent holes;
blend pillow with green receipts → datum probe + volume window;
`touching: true` can't distinguish seat from interference → `union` + `assert
shells`; hole wizard has zero edge-proximity awareness → `wall_thickness` +
volume window after every cut; vertex-only z-window scans read wrong
silhouettes → `Mesh::radial_extent`.

---

## 7. Limits ledger, condensed (DESIGN_GUIDE §24 — the known edges)

1. NURBS through booleans/fillets: freeform boolean = planar half-space
   difference/intersection ONLY; quadric/solid tools, freeform∩freeform,
   multi-patch, unions refuse BY NAME; no fillets on freeform faces.
2. One gearbox-housing tessellation leaks → ships `voxel_healed`; 4 phantom
   contact pairs re-proven disjoint by exact booleans each run. Pattern:
   arbitrate surprising contact receipts with the exact route.
3. Mirror-symmetric hole rows crack both tessellation routes; 1.5 mm datum
   shift heals; hybrid router still delivers watertight voxel mesh.
4. Fuzz 10,000/10,000; every solid op gates on `validate()` and fails loudly.
5. Blend pillowing is physics, permanent hazard: `fillet_union`/`smooth_union`
   add material wherever surfaces run parallel within the blend radius, with
   GREEN receipts (measured 0.44 mm bed bulge). Fuse first, cut last, probe datums.
6. Validity ≠ geometric truth at the input boundary — the gate guarantees
   closed/manifold, not that the input meant what you meant. Assert measures.
7. Scan classes, not certainties: `contacts.touching` is a class;
   `min_thickness` is corner noise; `assert_disjoint` accurate to `tol`.
8. Simulation bridge is tools/campaign-level; **no JSON load-case ops; no CFD
   at all** (declared gap).
9. Out of scope: sheet metal/casting/CNC process profiles (refuse loudly),
   PLY write, OBJ/glTF from JSON, `.lmcasm` STEP export. Drawings exist
   (SVG/DXF, measure-traced dims) but no GD&T/thread callouts/auto placement.
10. Coplanar-overlap forests: supported but least-margin — prefer embedments.
11. **Validity does not imply CONNECTEDNESS** (hardest silent-wrongness class):
    a part severed into floating lumps is `valid`, `closed`, `manifold`,
    watertight and plausibly measured. **Gate it with BOTH oracles** —
    `assert shells:1` (exact B-rep topology) *and* `assert components:1`
    (`Mesh::component_count` / `is_one_body`, faceted). The older claim that
    "`shell_count()` still says 1" on a severed part is **retracted**: a
    `difference`-severed bar reads `shells 2` AND `components 2`. The regime
    where they disagree is the opposite one — a sever narrower than the
    0.001 mm mesh weld reads `shells 2` but `components 1`, and
    `extrude_with_holes` reads `shells 1` while `mesh_components` REFUSES
    (`invalid_geometry` — the faceted surface is not closed, so its count would
    be faceter cracks, not bodies).  Follow the refusal: keep `shells:1` live
    and re-run the walk on the exported mesh.
    Measured tables, weld-scale band, and the two constructible
    oracle-negative-controls: `digests/ops_core.md` §"ENGINE UPDATE 2026-08-06"
    and `DELIVERABLE_SPEC.md` §2.2 / §2.13. Any tapering cutter's apex must
    stay strictly INSIDE material, and no designed ligament may land in the
    0.001 mm weld scale.

Numerics you inherit (docs/NUMERICS.md): units are mm everywhere (non-mm
`.lmcpart`/`.lmcasm` refused, never rescaled); angles radians; B-rep f64
working range |x| ≲ 1e7 mm (auto re-centred beyond), implicit/voxel f32
degrades by ~1e6 mm — model near origin, pose at the f64 layer; determinism is
per-platform/per-libm, byte-identical run-to-run (threaded booleans identical
BY CONSTRUCTION, `LMCAD_BREP_THREADS`); GPU is tolerance-equivalent
(≤ 1e-4·(1+|d|)), never bit-authoritative; 1-Lipschitz contract on SDFs —
blends/transforms are the caller's risk (redistance before narrow-band);
dense meshers silently return EMPTY above 2^28 cells (narrow-band escapes to
2^44); Manifold DC guarantees "closed, never worse than Surface Nets", not full
manifoldness — `check_mesh` + `make_manifold` when it matters.

**Determinism stops at the kernel boundary — solver receipts are NOT
byte-comparable.** The paragraph above is about `kernel-api`. The ACE solvers
are a different régime, and a "Reproducing" section that tells a reader to
`cmp` a solver receipt is a defect: it fails for reasons that have nothing to
do with the design.

| artefact | standard | how you check it |
|---|---|---|
| STLs / STEPs from `kernel-api`; PNGs/GIFs from the doc tools | **byte-identical**, mandatory (this is why doc jobs take `date` as a literal string) | `cmp` |
| ANY tool receipt — checkers and ACE solvers alike | **`determinism.core_digest` identical**, mandatory | compare that one field |
| tool receipt BYTES | **not comparable, ever** | don't |

**Never `cmp` a receipt.** Since 2026-08-08 every runner receipt carries a
`determinism` block (`tools/_receipt.py`, schema `lmcad.determinism.v1`):

```json
"determinism": {
  "schema": "lmcad.determinism.v1",
  "nondeterministic_paths": ["timings_s"],
  "core_sig_figs": 12,
  "core_digest": "sha256:5b790443f2…",
  "solver_reproducibility": "<what THIS solver guarantees — read and quote it>",
  "how_to_compare": "compare `determinism.core_digest` between runs, NOT the receipt bytes…"
}
```

`core_digest` is a sha256 over the payload with `nondeterministic_paths`
stripped and every float quantized to `core_sig_figs` significant figures —
i.e. **the byte-comparison campaigns were told to make and could not.**
Verified: two runs of one `ace_fatigue` job gave identical `core_digest` while
`timings_s` differed.

The two structural reasons a raw receipt diff always fails, now disclosed IN
the receipt instead of discovered the hard way:
1. **Every ACE receipt embeds a wall-clock `timings_s` block** — a
   bit-identical solve still produces different bytes.
2. **The numerics are not bit-reproducible.** `ace_modal` reruns differ by
   ~1e-13 to 2.6e-12 relative; `ace_buckling` and the `ace_fea_tet`/superlu
   path likewise (iterative / sparse-direct ordering).

So the honest reproducibility claim is **"`core_digest` reproduces; the
payload reproduces to `core_sig_figs` significant figures"** — never "byte
identical". Quote `solver_reproducibility` verbatim in ANALYSIS.md: it states
per-solver whether the digest is expected stable across *machines* or only
across runs here. `geometry_hash` in the FEA receipt pins the *input*; use it
to prove two receipts describe the same geometry.

---

## 8. PLA material data and what claims it licenses

Sources (all in-tree; `tools/materials.py` is the ONE source of truth, records
in `tools/materials/<key>.json`; `tools/material_db.json` is the legacy flat
DB the records were migrated from — datasheet-class, "verify per filament brand"):

**PLA record (`tools/materials/pla.json` v1.1.0)** — key values:
- Mechanical: E = 3.3 GPa, ν = 0.36, yield 55 MPa, ultimate 60 MPa,
  ρ = 1240 kg/m³. These are as-printed XY datasheet-class numbers → license
  STATIC, short-duration, in-plane, room-temperature claims only.
- Anisotropy: `z_vs_xy_strength_ratio = 0.55`, applied when the primary load is
  > 30° out of the layer plane. Use `materials.derated(name, load_dir,
  build_dir, basis="yield"|"ultimate")` → returns a receipt with
  `allowable_mpa`. Derate the ALLOWABLE, not E.
- Thermal: k = 0.13 W/mK (conservative-low, cited), cp = 1200 J/kgK (RT-class;
  the 1800 handbook figure is a recorded conflict, likely near-melt), Tg 60 °C,
  **softening_c = 55 °C** (= low end of HDT-B 0.45 MPa across brand TDS).
  Thermal-limit claims licensed: "hot side stays below softening_c" via the
  thermal solver. Printed parts conduct WORSE than bulk and anisotropically
  (noted, not modeled).
- Heat-set pull-out: M3 400 N / M4 650 N / M5 900 N (conservative low ends).

**Creep — the sustained-load law** (pla.json `creep` block; Rust mirror
`kernel_model::materials::pla::creep_allowable_mpa(temp_c, hours)`):

| | 1 h | 24 h | 30 d | 1 y |
|---|---|---|---|---|
| 23 °C | 7.5 | 5.0 | 3.5 | 2.5 MPa |
| 55 °C | 3.0 | 1.5 | 0.5 | 0.5 MPa |

- A part under sustained load is a CREEP case, not a static one (§25.7). Gate
  `stress <= creep_allowable_mpa(T, hours)` and state the design duration.
- **Read it through Python `tools/materials.py`, and SHIP THE LOOKUP RECEIPT,
  not the scalar:**
  ```python
  import sys; sys.path.insert(0, "tools"); import materials as M
  M.creep_allowable_mpa("PLA", 23, 8760)                    # 2.5   bare scalar
  M.creep_allowable_mpa("PLA", 23, 8760, across_layer=True) # 1.375 x0.55 applied FOR you
  M.creep_lookup("PLA", 25, 720)                            # the RECEIPT — ship this
  ```
  The receipt carries `row_used_c`, `col_used_h`, `cell_match`, `temp_match`,
  `duration_match`, `extrapolated`, `refused`/`refusal_kind`,
  `anisotropy_factor`, `legacy_scalar`, `material_hash` — i.e. **which cell
  your margin was read at**, which is the number a reviewer needs.
- **THE TRAP: there is nothing between 23 °C and 55 °C, and the lookup rounds
  the service temperature UP to the next tabulated row.** A declared ambient of
  **25 °C** reads at the **55 °C** row: `creep_lookup("PLA", 25, 720)` →
  `0.5 MPa`, `row_used_c: 55.0`, `cell_match: "rounded_up_conservative"`, note
  *"state 55C, not 25.0 C, as the temperature this margin holds at"*. That is
  **7× below** the 23 °C / 30 d cell (3.5 MPa). Declaring 25 °C ambient and
  then quoting the 23 °C row is a blocker-class defect — it happened. Declare
  23 °C or accept the 55 °C row; there is no middle.
- Above 55 °C the reader **REFUSES** (`refused: true`,
  `refusal_kind: "creep_temp_above_tabulated"`) and does **not** fall back to
  the 55 °C row — "no sustained load is defensible" fails the gate loudly.
- The 55 °C / 30 d and 1 y cells are **BOUNDS, not measurements** — read as
  "do not design sustained load into unannealed PLA at 55 °C".
- Across-layer sustained load: pass `across_layer=True` (the 0.55 ratio is
  applied inside the lookup and echoed as `anisotropy_factor`). Do **not**
  also multiply by 0.55 yourself.
- `production_check.py`'s verdict is a **static-strength** verdict. It is not a
  creep verdict and it takes no duration — run the creep gate separately.
- `creep_shear_allowable_mpa` = 0.6 × tension value — derived, not measured; say so.
- The legacy scalar `creep_sustained_fraction = 0.2` (→ 12 MPa) is NOT
  supported at year scale — recorded conflict; the table governs.
- pla.json `sn_curve` (`0.3 × ultimate at 1e6`) is a rule of thumb, NOT a
  measured S-N curve — for fatigue LIFE numbers use only the fatigue solver's
  own registry (`tools/materials/fatigue.json`, canonical UTS 40.9 MPa).

Other materials (PETG/ABS/ASA/TPU95A/PC/PA) have records with the same shape:
they license static derated strength and thermal-conduction claims, but **NO
fatigue life** (solver refuses) and no creep table (PLA is the only researched
creep block). PA values are DRY — moisture can halve them.

---

## 9. Deep-dive pointers

| topic | source |
|---|---|
| exports/interop, STEP scope, watertight gating | DESIGN_GUIDE §21 |
| print-readiness worked example | DESIGN_GUIDE §22 + `iphone_stand/tools/` |
| error kinds + silent modes | DESIGN_GUIDE §23 |
| limits ledger (full text) | DESIGN_GUIDE §24 |
| campaign architecture, analysis-plan law, implicit toolbox | DESIGN_GUIDE §25, §25.1, §25.7 |
| capability contract, tiers (a)/(b)/(c), external bridge | docs/ANALYSIS_DOMAINS.md |
| envelope schema, tier ladder, synthesis guardrails, research fence | docs/ANALYSIS_TIERS.md |
| manifest format | docs/MANIFEST_SCHEMA.md |
| units, tolerances, determinism, f32/f64, Lipschitz, GPU | docs/NUMERICS.md |
| per-solver cards (bands, refusals, job schemas) | tools/solvers/{README,ace_fea,modal,buckling,thermal,contact,fatigue}.md |
| solver gate suites (run to re-prove) | tools/ace_*_validation.py, tools/test_ace_thermal.py, tools/test_ace_modal_buckling.py, tools/test_ace_contact_fatigue.py |
| derived-model scaffold + exemplar | tools/derived_model.py (`--selftest`, `--new`) |
| provenance/envelope enforcement | tools/provenance.py |
| analyzer ledger | `python3 tools/analyzer_registry.py` |
| material records + derating | tools/materials.py, tools/materials/*.json, tools/material_db.json |
| creep allowables (Rust) | `kernel_model::materials::pla::{creep_allowable_mpa, creep_shear_allowable_mpa}` (crates/kernel-model/src/lib.rs ~4180) |
