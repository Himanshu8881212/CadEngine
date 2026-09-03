# LMCAD changelog — the dated capability & fix ledger

Chronological record of every capability wave, campaign, and root-cause fix
(moved out of CLAUDE.md in the 2026-07-29 cleanup so the working context
stays short; nothing was reworded — these are the original entries).
Current-state summary and open frontier live in CLAUDE.md; the falsifiable
scorecard in docs/BAR.md; deep friction write-ups in docs/FRICTION.md.

FIXED 2026-06-09 (do not regress; repro tests live in-tree): **R1** revolve of
concave/multi-segment profiles; **R2/R3** booleans on faces with inner loops and cuts
crossing cut curved walls (loop-aware triangulation/recovery/healing in booleans.rs);
**R4** `exact_volume` hole-loop signs. Chain-fuzz pass rate 38.5% → 99.5%
(`docs/ROBUSTNESS.md`; the ≥99% Level-6 bar test runs in the default suite).
FIXED 2026-06-10: **R5** run-to-run nondeterminism in the boolean pipeline — three
HashMap/HashSet iteration-order dependences (`cancel_coincident` drain order,
`recover_faces` region order, `boundary_loops` pinch successors); guarded by
`tests/determinism.rs` (40× bit-identical flange rebuild).

FIXED 2026-06-10 Wave-1: fuzz residue eliminated — **100.0% chain validity
(2000/2000, floors 98)**, all 9 seeds pinned as regressions (3 more arrangement
root causes: non-unit split normals, sub-EPS piece discard, micro-subdivision
debris); `Mesh::fill_holes`/kernel-core repair now deterministic; sphere+cone
inertia analytic; concave bore-rim torus fillets machine-exact; standards parts
catalog (`kernel_model::parts`), hole wizard (`kernel_brep::holes`), JSON binding
(`kernel-api` + API.md), `try_*` checked booleans, Document JSON persistence.

ADDED 2026-06-11 (post-v1.0 ledger work): assembly nesting (`asm_path`
sub-assemblies as rigid units, cycle detection, hierarchical names,
branch-drop suppression) + BOM v2 (`.lmcpart` `meta` part_number/material/
make_or_buy, mass = density × engine volume with honest `volume_source`,
tree+flat+`bom.csv`, byte-identical) — `tests/nesting.rs`, gearbox nested
demo in `run_all.sh`. DESIGN_GUIDE v2 (3.2k lines, every snippet executed)
is the operator manual. RETIRED 2026-06-11: the coplanar STACKED-primitive
union seed-sensitivity — retested on current main during the guide-v2
re-measurement, passes deterministically (DESIGN_GUIDE §7).

ADDED 2026-07-02 (DOVESTACK campaign — Printables modular-drawer contest entry,
`examples/drawer_system.rs` + `drawer_system/`, 16 print-ready parts, all gates
asserted): `SupportFreeReport` (bed/bridge/steep FDM support-necessity audit,
kernel-core), `holes::teardrop_hole` (support-free horizontal bores), and a
root-cause weld fix — `Mesh::weld` now drops collapsed needle triangles, which
ALSO fixed the coplanar tube-on-plane tessellation gap of 2026-06 (its pinning
test `coplanar_tube_tessellation.rs` now asserts watertight). FRICTION #23
characterized honestly: booleans on an isolated notch-plate whose overlap is two
parallel-flank sliver strips mis-stitch and REFUSE via try_* (repro in
`kernel-brep/tests/recovery_needle_weld.rs`); same joint inside the real drawer
shell resolves and is gated numerically by the example.

ADDED 2026-07-12 (implicit/voxel gap-closing): three more TPMS families —
**Neovius, Schoen I-WP, Fischer-Koch S** — alongside gyroid/Schwarz-P/diamond
(`kernel_implicit::TpmsKind`, tight Lipschitz constants 7 / 3√3 / √6 pinned
numerically; `tests/tpms.rs` gates all six ~50%-solid + ≤1-Lipschitz +
mesh-closed), now first-class on the op surface via a new `tpms` shape leaf
(kind × network/sheet, wrapped `primitive_bound` per the FieldQuality contract;
`implicit.rs` + `tests/implicit.rs`). And `expr_sdf`'s declared `lipschitz_bound`
is now **sample-verified** before a narrow-band mesh (`implicit::probe_lipschitz`
— an under-declared bound is a loud, actionable refusal instead of silent holes;
scoped to the narrow-band extractor since the dense meshers need no bound).

ADDED 2026-07-24 (POOLDOCK campaign — Printables pool-accessories flash contest,
4 entries in `pool_system/`, examples `pool_tubedock` / `pool_noodle_hub` /
`pool_tpms_basket` / `pool_staples`, all gates asserted, suite green): the
POOLDOCK dovetail (6.0 opening / 12.0 root / 2.5 deep, 50° flank — DOVESTACK
males stay captive), snap-on C-docks for Ø25.4/32/38 rails + Ø65 noodles, a
production validation matrix (3×4 seat/overlap_volume/15-pose swept-insertion
fit matrix, snap strain + rattle + neighbour-gap bounds), buoyancy asserted
from engine volumes (3.99× reserve), honest gyroid support budgets (never
zeroed), and per-entry assembly-doc sheets/BOMs via tools/assembly_doc.py.

ADDED 2026-07-28 (RESPOOL campaign — two-part printable reusable spool for
Bambu-style 1 kg refills, `examples/respool.rs` → `spool_system/respool/`,
all gates asserted, ANALYSIS.md + assembly-doc sheet generated): ONE
hermaphroditic half printed twice (Ø200 × 67, Ø55 bore — researched official
envelope; Ø81 barrel + Ø81.7 crush-rib envelope for the Ø82 × 60 cardboard
refill core), 3+3-sector internal bayonet (42° tongues, 15° CW twist,
chamfer-matched sloped detent, zero-preload geometric retention) proved by
posed-solid sweeps (insertion/twist/detent-bite/retention/overtwist/
wrong-angle NC + witness-pin holes that align only at lock), closed-form
strength+thermal load cases gated at RT AND 50 °C-derated PLA allowables,
ACE hex8 FEA jobs (`fea/`), and as-printed tolerance-stack gates. Geometry
robustness law learned here (the DESIGN_GUIDE §7.4 least-margin corner made
concrete, documented at `BUMP_AC` in respool.rs): boolean features must
respect the revolve facet grid — a small embedded union straddling a facet
meridian cracked default tessellation (valid B-rep, leaky mesh), a cutter
side-plane exactly ON a meridian degenerated the arrangement, and a
flush-coincident pocket floor whose cutter edge lay INSIDE the coplanar
overlap broke validity late in the chain; hardened by SEG divisible by the
pattern count (126), mid-facet bump placement, 0.1-embedded pocket floors,
and cutter inner faces pulled into open air. A minimal tube+pocket+bump
repro does NOT crack (needs the full face neighbourhood), so no kernel pin
test was shipped — the defence is design-side.

ADDED 2026-07-28b (engine hardening from the RESPOOL retro — every gap that
campaign hit, closed the same day): `kernel_brep::boolean_hazards` pre-flight
linter (names coincident/near-coincident plane+cylinder pairs and straight
edges lying inside the other operand's planar face — the facet-meridian and
coplanar-overlap-edge classes — grouped per analytic surface with severity
and location); `try_{union,difference,intersection}_sealed` (validate AND
tessellation-watertight, returning the mesh, else `SealedError` with
boundary/non-manifold edge counts — closes the valid-but-leaky gap);
`ChainLog` boolean-chain debugger (per-step validate+seal, refuses past the
first bad step BY NAME, keeps last-good — the RESPOOL bisect harness,
promoted); polar builders `sector_prism`/`radial_frame`/`force_ccw` + facet-
meridian documentation on `revolve`; `Mesh::radial_extent` (band-clipped,
exact max, interior-foot-aware min — kills the vertex-window measurement
trap); `SupportFreeReport.steep_exemplars`/`bridge_patches` (WHERE, not just
how much); `kernel_model::{sweep_check, penetration_estimate, materials}`
(the campaign kinematic-sweep idiom promoted; vertex-sampled penetration is
an honest underestimate — load-bearing poses still gate on overlap_volume).
Tests: kernel-brep/tests/{hazards_linter,chain_and_sealed,mesh_measures}.rs,
kernel-model/tests/sweep_check.rs. DESIGN_GUIDE: new §7.7 boolean-hygiene
checklist, §25 campaign cookbook, two new silent-mode rows in §23.
mesh_io_roundtrip test temp files made per-process (two overlapping suite
runs raced on fixed names). STEP exporter size work intentionally untouched
(already ledgered in-progress above).

ADDED 2026-07-28c (DRYBOX ROLLER campaign — bearing-roller + desiccant base
turning the community-standard ~4 L cereal container into a rolling dry-feed
box, `examples/drybox_roller.rs` → `spool_system/drybox_roller/`, ALL GATES
PASS): tray + sliding hatch + 4× 608 on D-profile push-on stubs (zero
hardware), 151 ml slotted desiccant tank, hatch captive-in-box by geometry;
researched container floor/height/spool bounds asserted. First full consumer
of the 07-28b hardening (ChainLog::seal chain, boolean_hazards pre-flights,
sweep_check paths, materials) — and a live demonstration of why they exist:
the gates caught, in one afternoon, a coplanar-forest parapet union and a
flush-coincident coupon embed that each drove booleans::resolve_t_junctions
into a 40+-minute recursive cascade (NEW FRICTION, unpinned: exact booleans
on §7.4-class inputs can quasi-hang, not just refuse — sample(1) shows 100%
in resolve_t_junctions; design-side hygiene + batched disjoint cutters cut
the tray chain from unfinished-at-40-min to 20 s), a teardrop stub whose
apex (1.44·r) no Ø8 bore could pass (twice — full and truncated variants,
caught by penetration_estimate), a 52°-chord D-profile leaving
38°-from-horizontal facets (caught by the support audit), an 11 mm edge
deck bay over the bridge gate, and a bearing push-on path that collided
with the parapet (fixed by station notches). ChainLog gained an env-gated
live trace (LMCAD_CHAIN_TRACE=1) because the silent 100%-CPU spin had no
name until then. A user-prompted re-audit then found the sharpest hole of
the day: the hatch slider COLLIDED with the end parapet on its exit path,
and the vertex-sampled sweep read the thin-wall crossing as pen 0.000 (no
contained vertices on either side) — a vacuously-green free-run gate.
Fixed three ways: `SweepReport.contacts` (poses at <0.02 surface distance;
free-run gates must assert contacts == 0 — kisses and crossings both
count), an end-wall exit notch with a 0.3-clear sill (a flush sill promptly
re-registered as 7 kissing contacts — the hardened gate biting), and
follow-on honesty fixes (race-engagement gate restated for the true ±2.1
play, shoulder-on-inner-ring-land assertion, tray STEP round-trip, ±2.0
lateral spool poses, taper-adjusted captive margin with a widened slider).

ADDED 2026-07-28d (roadmap wave — the four items the day's campaigns ranked
worst, fixed): (1) `resolve_t_junctions` was O(edges × ALL vertices) — the
45-minute 100%-CPU quasi-hang was a quadratic scan, not recursion — now a
deterministic spatial hash (candidates re-sorted to the linear scan's exact
evaluation order: OUTPUT BIT-IDENTICAL, R5/fuzz untouched; drybox
superstructure op 15.2 s → sub-second class). (2) exact crossing oracle:
`Mesh::crosses_mesh` (BVH-pruned proper tri-tri, touches excluded) +
`SweepReport.crossings`; the vertex-blind thin-wall miss is pinned by
kernel-model/tests/sweep_check.rs (crossings=1 while sampled pen=0.0).
(3) coarse-grained parallelism where determinism is safe: pose-parallel
`sweep_check` and `kernel_brep::overlap_volume_many` (independent booleans
on scoped threads, results by index; adopted by respool's twist sweep) —
intra-arrangement threading DEFERRED deliberately: it risks R5 bit-
determinism for a win the coarse grain already delivers at campaign scale.
(4) FRICTION #20 fragmentation remedy: `kernel_brep::coalesce_coplanar`
(opt-in finishing pass; plane groups merged across shared edges via the
region-boundary half-edge walk, islands preserved, vertex array compacted —
a pads-on-plate union measured 65 faces → 16 exactly, volume-exact, valid,
watertight; tests/coalesce_coplanar.rs). #20's REMAINING half stays open
honestly: the rebuild resets provenance names, so witness re-resolution
after coalescing is future work — it is a geometry-finishing pass, not yet
a mid-feature-chain one.

ADDED 2026-07-29 (implicit expansion wave — the six ranked implicit gaps
closed by a five-agent fleet, each landing Rust-API + doc contract + pinned
tests; integration-passed as one suite):
(1) **shell/offset** — `kernel_model::shell::{offset_mesh, shell_mesh,
offset_to_solid, shell_to_solid}`: voxel-routed and labeled so, pinned at
−0.10…−0.19% vs exact Steiner analytics on the rounded-rim cases, shell
cavity proven to survive B-rep conversion (shells=2; tests/shell_offset.rs).
(2) **strut lattices** — `kernel_implicit::strut::{StrutLattice
(Bcc/Fcc/Octet), graph_lattice, pipe_path}`: exactly 1-Lipschitz min-capsule
fields; periodic 27-image tiling with the equality proof (and the seam
counterexample for the naive prune) in the module doc — border field jump
≤ 1.2e-3, tiled meshes closed; solid fractions pinned 19.8/22.1/39.3% at
cell 10 / r 1 (tests/strut.rs).
(3) **simulation→geometry** — `kernel_implicit::grid_field::GridField`:
hand-parsed NPY v1–v3 (refuses NaN / Fortran order / shape surprises
loudly), trilinear + border-clamped, `into_grade_law` emitting the EXISTING
`offset_by`/`lerp` closure type (no parallel grading path). Loop pin: a
stress-ramp NPY grades the §16.8 gyroid so the high-stress half carries
1.42× the material of the low-stress half, both halves watertight.
`tools/stress_to_density.py` (percentile-clipped floor/γ density law)
round-tripped against the real RESPOOL FEA field, 162×162×35
(tests/grid_field.rs).
(4) **surface textures** — `kernel_implicit::texture::displaced` with
Knurl/Stipple/Noise: displacement with DERIVED Lipschitz constants
(knurl df·π/pitch via the six-diagonal triangle inequality — and the
product-to-sum identity showing the knurl IS a scaled gyroid; stipple dome
max-slope 8/(3√3·r₀); noise √3/cell), emitted field renormalized ÷L′ so the
`DistanceBound` contract stays sound — pinned by dense vs narrow-band
volumes agreeing bit-for-bit (tests/texture_text.rs).
(5) **text** — `kernel_implicit::text::text_field`: Hershey Simplex strokes
(decoded from the canonical .jhf, provenance + Hershey/Hurt license in the
module doc) as exact capsule SDFs, exactly 1-Lipschitz; engrave pin removes
128.05 mm³ vs 128.04 analytic half-round groove (0.01%)
(tests/texture_text.rs).
(6) **reverse bridge v1** — `kernel_model::reverse::{mesh_to_solid,
implicit_to_solid}`: implicit/mesh → faceted B-rep (weld → solid_from_mesh
→ coalesce_coplanar → validate) with a volume-conservation gate and
counts-carrying refusals; STEP round-trip drift exactly 0.0 mm³ on a
2089-face smooth-union blob, cuboid mesh → exactly 6 faces. FACETED ONLY by
written contract — analytic curved-surface recovery stays ledgered as v2.
Interrogation probes shipped alongside: `thin_wall_report` (sampled,
under-report ≤ one cell, documented like `penetration_estimate`) and
`kernel_brep::holes::min_ligament` (FRICTION #21 advisory echo; 2.000 mm on
the Ø6-at-5-mm case) (tests/reverse_bridge.rs, tests/holes.rs).
Hygiene rider: the always-on friction inbox is now SILENT inside cargo
test/bench binaries unless `LMCAD_FRICTION_INBOX` is set — intentional
refusal-path tests had been appending "evil" chain_refusal noise to a
crate-local inbox on every `cargo test` (file removed; carve-out pinned in
the telemetry unit test; detection = executable parent dir `deps`, no env
contract). Docs: §25.1 campaign implicit-toolbox table, §24 item 8
simulation-bridge status flip, research-skill analysis-plan step (research
now also freezes WHICH analyses the artifact class requires; every item in
ANALYSIS.md answered by receipts, by a benchmark-gated new solver, or by a
bold NOT PERFORMED — silence forbidden).

ADDED 2026-07-30 (frontier wave — the five deliberately-deferred items,
closed by a second five-agent fleet; every claim test-pinned):
(1) **Reverse bridge v2, analytic quadric recovery** —
`kernel_brep::recover::recover_quadrics` (+ `fit_{plane,cylinder,sphere,
cone,torus}` and `kernel_model::reverse::{mesh_to_solid_recovered,
implicit_to_solid_recovered}`): coalesce generalized to quadrics — region
growing over smooth-bend edges, 5-kind least-squares cascade with
sagitta-aware acceptance (vertices + edge midpoints + centroids) and
outlier peeling; no vertex ever moves (carrier recovery, residual
reported). Pins: implicit cylinder 1326→80 faces (r to 0.0016 mm, axis to
2e-12), STEP 3.06× smaller, re-import drift 0.0014%; cone 24 710→126
faces, apex 0.019 mm; sphere/torus retagged with exact STEP surfaces,
volume bit-identical (single-face merge would break the 0.5% volume gate
through boundary-ring tessellation — sector span capped 0.11 rad,
span²/6 bound in-module); hex-prism negative control rejected at 21.4×
tol with the closed-form sagitta residual (kernel-brep/tests/recover.rs,
kernel-model/tests/reverse_bridge.rs).
(2) **Sparse SDF grids** — `kernel_implicit::sparse::{SparseGrid,
OctreeGrid}`: Lipschitz-safe tile allocation (centre test provably cannot
miss a surface-crossing tile; proof + over-claimer negative control
in-module), 16-bit in-band storage (quantum 30× under trilinear error).
Pins: 200 mm sphere domain @ 0.4 mm = 22.6 MB vs 503 MB dense (4.49%),
build evaluations 9.75% of dense, cache error 5.06e-4 mm, mesh-through-
cache volume delta 0.00002%, hash-identical independent builds; octree
scoped honestly to evaluation caching (T-junction discontinuity + non-
conservative far field stated) (tests/sparse.rs).
(3) **GPU narrow-band extraction** — `extract_narrow_band` (kernel-gpu; the crate
was parked in `legacy/` 2026-09):
coarse Lipschitz-safe block scan → prefix-sum compaction → refine active
blocks with the SAME cube-edge unroll as dense (shared `edge_unroll()`);
per-block re-evaluation of identical global-index coordinates ⇒ identical
f32 bits at shared points ⇒ dense closure argument carries over. Pins:
volume parity with dense-GPU and CPU to 4 decimals; 8.3% active blocks /
5.6× fewer samples on the 60 mm sphere; delivers past the dense 2²⁸ cap
(3.0e8-cell sphere watertight, −0.001% vs analytic); band floor clamps
pinned bit-identical; TPMS egg-crate honest (active 0.993 — no gain,
correctness gated) (kernel-gpu/tests/narrow_band.rs; 18/18 GPU tests ran
on Metal).
(4) **JSON op surface 155→160** — leaves `strut_lattice`/`pipe_path`/
`text` (+ charset pre-validation), combinator `displace`, `{"grid":…}`
field source (NPY confined under input base), ops `offset_solid`/
`shell_solid`/`solid_from_implicit` (route-labeled `voxel`), measures
`thin_wall`/`min_ligament` (explicit no-material/no-interior statuses, no
raw NaN in JSON); ~15 provoked machine-matchable refusals; discover.rs
regenerated (md5-stable); every API.md example executed. Genuine findings
pinned: thin_wall on sharp-edged exact solids reads the edge-wedge sliver
(interior-domain idiom documented); heavy displace can pinch the
narrow-band mesher (refusal names it) (kernel-api/tests/implicit_wave.rs,
21 suites green).
(5) **Intra-arrangement threading, R5 preserved** — co-refine + fragment
classification as chunked pure flat-maps
(`kernel_core::par::par_flat_map_chunks`: chunk boundaries a pure function
of item count; concat in ascending chunk order; identical per-item float
sequences ⇒ byte-identical BY CONSTRUCTION); stitch stays sequential by
necessity (where the three R5 bugs lived), triangulate by measurement
(threading it lost). `LMCAD_BREP_THREADS` (default on; `1` = same code,
one thread), work-based cutoffs, nested-pool guard (booleans inside
`kernel_core::par` workers stay sequential — no oversubscription). Pins:
11-case parity corpus byte-identical across schedules (full canonical
dumps incl. f64 bits), threaded 40× determinism variant, engagement
counters (55k+ parallel items; ==0 under `=1`); measured 1.85–2.2× on
heavy chains, stitch = honest Amdahl floor (tests/threading_parity.rs,
determinism.rs; 46/46 kernel-brep suites incl. fuzz floors).
Docs: Appendix A re-derived mechanically 116→160 with per-family
arithmetic (11+3+4+11+13+3+2+3+10+4+5+48+13+13+7+7+3), §25.1 v2/sparse
rows + engine-wide riders, NUMERICS threading + narrow-band doctrine,
AGENTS.md layout/frontier refresh.

ADDED 2026-07-30 (SYSTEM wave — fourteen agents; the shop stops being a
geometry kernel with analyses bolted on and becomes an engineering shop
whose capability ACCUMULATES. Every claim below is pinned by a named test
or a benchmark suite that has been proven able to fail):

**Solvers — the registry** (`tools/solvers/`, index README; card format:
physics · equations · discretization · I/O contract · CURRENT measured
benchmark numbers · validity limits · when to use). Six solvers now:
`ace_fea` (retro-carded, biases documented: −11.2% cantilever, +20–29%
non-convergent at staircased fillets), **thermal** (`ace_thermal_runner.py`
— steady + backward-Euler transient conduction, Dirichlet/flux/convection;
measured order 2.02 on the source-slab, erfc transient 6.7e-3, energy
residual 5.5e-9; 21 gates), **modal** (`ace_modal_runner.py` — hex8 +
lumped mass, shift-invert eigsh, free-free via negative shift; cantilever
modes 1–3 within +1.4/+0.2/−1.6% of Euler-Bernoulli with the omitted
Timoshenko shear DERIVED, 6 rigid modes at ≤7.7e-3 Hz, cross-check vs the
reference implementation 2.3e-13), **buckling** (`ace_buckling_runner.py`
— geometric stiffness two-pass; Euler column +3.4% converging, 2E→λ×2.000
exactly; ships a cited 0.5 knockdown and a yield-before-buckling warning),
**contact/nonlinear** (`ace_contact_runner.py` — corotational
Euler-Bernoulli beam with node-to-rigid penalty contact and an insertion
force-displacement curve; exact-elastica error 1.43e-5, linear limit
2.7e-13, equilibrium 8.6e-12; snap-fit peak insertion **3.007 N vs the
naive P·tan30 3.238 N — the hand formula over-predicts 7.1%**), and
**fatigue** (`ace_fatigue_runner.py` — Basquin + Goodman/Gerber + Miner;
46-gate suite, Miner exact to 0.0e0, Basquin round-trip 1.2e-15).
Fatigue DATA is the honest part: PLA is the ONLY material with measured
printed S-N (Ezeh & Susmel 2019, 143 specimens; independently corroborated
to 0.42% on the Basquin coefficient), PETG/ABS refuse as `insufficient`,
ASA/PA/PC/TPU are `unknown`, **across-layer fatigue is UNKNOWN for every
material and the runner refuses that orientation** rather than reuse the
static anisotropy ratio; measured life scatter 3.7×–90× rides in every
receipt; only the max-stress mean-stress model is validated for printed
polymers, so stacking Goodman on it is refused as double counting
(`tools/materials/fatigue.json`, `test_ace_contact_fatigue.py`).
Each new solver suite carries a META-NEGATIVE-CONTROL: a deliberate break
in a scratch copy must turn gates red (contact: the corotational-kinematics
break makes the solver REFUSE rather than lie; fatigue: the Basquin
exponent-sign break reddens 5).

**Materials — time-dependent allowables.** Researched thermal properties
for all 7 materials (≥2 cited sources per number, conflicts kept as data,
printed-anisotropy noted) plus a PLA **creep** block: sustained allowables
23 °C {7.5/5.0/3.5/2.5} and 55 °C {3.0/1.5/0.5/0.5} MPa over
{1 h, 24 h, 30 d, 1 y}, with the full derivation chain, per-cell
confidence, and the 55 °C long-duration cells flagged as BOUNDS not
measurements. Promoted to Rust as
`kernel_model::materials::pla::{creep_allowable_mpa, creep_shear_allowable_mpa,
CREEP_*}` — conservative lookup (temperature and duration both round UP to
the worse cell; **0.0 above the hot tier**, so a sustained-load gate fails
loudly exactly where no data exists), with a CROSS-LANGUAGE pin asserting
the Rust mirror never drifts from the JSON (`tests/materials_creep.rs`).

**Process layer — a making engine, not a printing engine.**
`kernel_model::process`: `FdmProfile` (serde JSON in `profiles/`) with fit
helpers that reproduce the shipped campaigns' frozen constants exactly
(`fit_free_shaft_r(37.3) → 37.05` = RESPOOL's `R_TO`;
`fit_tight_shaft_r(4.0) → 3.95` = DRYBOX's `STUB_R`), DFM checks, and
sheet-metal / casting / CNC as DECLARED siblings that refuse loudly
(casting's refusal names `draft_analysis` as the half that does exist).
New campaign `calibrate_fdm` → `calibration_system/fdm_coupons/`: a 43 g
coupon plate (hole/fit/bridge/wall ladders, Ø22 bearing gauge, overhang fan
— 45° deliberately ABSENT because it sits on the threshold and makes the
measurement a coin flip), 26 gates incl. 5 negative controls, feeding
`tools/ingest_calibration.py` (measured calipers → `profiles/<printer>.json`;
`--self-test` drives a synthetic perfect printer and asserts every
compensation is exactly 0.0).

**System-level engineering** (`kernel_model::{tolerance,loads,mechanism}`):
tolerance stack-up with worst-case + RSS (the statistical assumption is
carried BY VALUE in the result, not just in docs) and contributors ranked
by share — headline pin is the aggregate-only failure: six ±0.08 links that
each pass alone and fail together, plus the honest note that RSS passes the
same window worst-case fails. Load paths solve rigid-body equilibrium to
per-part reactions (residuals exactly 0.0 vs hand statics) and REFUSE on
statically indeterminate input with the redundancy count, and on floating
assemblies with a FLOATING hint — never zeros; the FEA manifest carries
`unrepresented_moments` explicitly rather than dropping a couple a
point-load job cannot express. Mechanism kinematics sweeps driven joints
over full cycles (four-bar limits match the law of cosines to 7.6e-13,
slider-crank stroke exactly 2r, Kutzbach DOF cross-checked against the
numeric Jacobian rank with a Grübler-paradox flag) — and the pin that
justifies it: a pair CLEAR at both endpoint poses (8 mm) is convicted
mid-cycle at 22.5°, with `penetration = 0.0` and `min_distance = 0.0`,
i.e. only the exact triangle-crossing oracle sees it.

**Optimization harness** (`kernel_model::optimize`): declared design
variables / objectives / constraints, full-factorial + pattern search +
Pareto extraction, parallel-deterministic (bit-identical canonical reports),
sensitivity as a stated screening indicator. The SIMP honesty doctrine
generalized: `best()` RE-EVALUATES the winner and a mismatch is a typed
`ImpureEvaluator` error (negative control included); infeasible designs are
retained and marked; `gate_study` lets a campaign re-prove "the shipped
design IS the study optimum" every run. Real-geometry study pinned
(ribbed plate: 2.0 mm wall × 3 ribs, I = 704.4876 mm⁴ at 9.70 g, matching a
composite-section hand calc to <1e-5).

**Making-engine breadth** (`kernel_model::{drawing,cost}`): orthographic +
section views with SAMPLED hidden-line removal (BVH ray classification),
facet-seam suppression (a Ø20 cylinder draws 2 lines, not 32), analytic
circles at exact radii, dimensions that each carry the named measure they
came from (five declared sources; NO code path accepts a caller literal) and
a dimension SCHEDULE on the sheet; deterministic SVG + DXF R12 (no clock —
the date is an input, proven by a substitution test). Cost: deposition-volume
FDM model with `FDM_ACCURACY_CLASS` (±30%) as a REQUIRED field on every
breakdown, money rates labelled placeholders, support volume documented as an
upper bound (over-quotes, never under-quotes), costed BOM grouping.

**The physical learning loop** — the flywheel's missing half:
`tools/field_report.py` (structured intake, controlled failure vocabulary,
mode-specific evidence REQUIRED, condition violations separated from design
failures) + `tools/field_triage.py` (mode → the analysis that would have
caught it → the permanent change) with a **re-audit hook** that parses a
shipped `ANALYSIS.md` and names which green claim the failure contradicts.
Proven on the real RESPOOL analysis: 25 claims parsed, the top hit landing
exactly on the creep sentence. `docs/field_reports.jsonl` is a permanent
LEDGER (never truncated, unlike the friction inbox); doctrine in
`.claude/skills/lmcad-field`. Consequence, applied the same day: RESPOOL and
DRYBOX gained sustained-load gates against the CREEP table instead of the
static one (respool T3b: 14× margin vs the 1-year bound, was cited as 70%
of a static allowable; drybox: 43× bearing / 23× root shear, replacing a
prose "structural analysis intentionally absent"). Neither product changed;
both now prove the right CLASS of claim.

**Meta-layer proofs** (the historical META_PROOFS document): `tools/audit_docs.py`
(op-count / path / section / symbol / claim-freshness drift, each check
proven able to fire by injection into both a synthetic fixture and a copy of
the real tree), the historical coldstart probe (entry-path readiness — and it states
plainly it does NOT prove a fresh model would succeed; the real exam is a
written manual protocol), the historical portability check (canonicality, adapter
parity, Claude-specific-assumption scan). First run found 5 op-count errors,
3 dead section pointers, 3 ops missing from API.md entirely, and a
PORTABILITY BUG in the lessons skill (it told agents to update a section of
the then-current CLAUDE.md shim). All fixed.

**Engine fixes found BY this wave's own gates:**
- **45° knife-edge** (`kernel-core/src/mesh/mod.rs`): the support threshold
  was converted and sined in f32 then widened, while facet normals are f64 —
  so geometry designed exactly ON the threshold was reported as needing
  support (`teardrop_hole`, roof at exactly 45°, measured 8 mm² steep at
  overhang_deg 45 and 0 at 46). Now evaluated in f64 with a MEASURED slack
  (1e-4 in cosine ≈ 0.008° at 45°, ~9× the worst f32 normal noise of 1.14e-5
  found by sweeping angle × part scale × facet size). `overhang_analysis`
  aligned to match. Pinned by `kernel-core/tests/support_threshold.rs`, which
  asserts BOTH that an at-threshold facet passes and that a 1°-past facet
  still fails.
- **`fillet::round_edge_by_id` silently dropped inner loops** (found by the
  provenance work making it newly reachable): a plain holed plate returned
  `Ok` with closed=false topology and a NEGATIVE −73 mm³ cut. Now a loud
  `Unsupported` refusal.
- **`tri_sdf` winding bug — CAMPAIGN-LOCAL, not an engine fix** (listed here
  because it is instructive, and to correct an earlier draft of this entry
  that filed it under engine fixes): `bracket_gen`'s own 2-D triangle SDF
  helper assumed CCW winding, so its CW roof triangles cut nothing and the
  teardrop/countersink tunnels were bare circles (689 mm² of flagged
  ceiling). Found by the campaign's own support gate and fixed in the
  campaign. Nothing in `crates/**` was involved.
- **Sidecar tables broke the material loader** (found by adding
  `fatigue.json`): `load_all` globbed every JSON as a material record. Now
  opts out on `meta.schema_kind`.

**GRADUATION EXAM — `bracket_gen` → `bracket_system/gen_bracket/`.** One
product through the ENTIRE generative loop in a single gated run: baseline
solid → ACE FEA → SIMP (40 iters, compliance 8.086e-3 → 1.293e-3) →
density field → threshold+smooth to a 2.5D web → watertight mesh → STEP →
**honest re-analysis of the final binary geometry**. Measured: **290 g →
173 g (−40.3%) at ×1.35 tip deflection** (gate ≤1.5) with peak von Mises
going DOWN ×0.92. Governing margin is the SUSTAINED one — 23 °C/1-year
creep, ×1.88 — not the static RT ×7.5, and the analysis states that printed
upright the same part would fall to ×1.03. The screw seat is the design's
weakest link at 1.83 MPa WITH the cup washer and 3.61 MPa without, so the
washer is a REQUIRED BOM part; buckling λ→×37 rated, not governing. Five
real bugs were caught by its own gates before it passed (a leaky recovered
solid reading 143× deflection; a parity-fill voxelizer producing 43
components and 1410× deflection; the CCW winding bug; a SIMP filter radius
leaving 0.5 mm necks; and a negative control too weak at 1.4× that now
severs the truss at 224×). 54 deliverables byte-identical across two runs.

**CAMPAIGN 2026-07-31 — DRILL HOOK → `hook_system/drill_hook/`.** A permanent
over-the-edge shelf hook that hangs a 1.8 kg cordless drill by its grip: one
printed part, no hardware, **zero supports and zero bridging** (steep area
0.000 mm², max bridge span 0.0 mm). Three findings shaped it, none of them
geometric. (1) The load never comes off, so **creep governs**: every
structural gate is judged against `materials::pla::creep_allowable_mpa(23 °C,
1 y)` = 2.5 MPa, giving ×3.4 on sections measured off the real filleted,
window-pierced profile rather than retyped rectangles. (2) The part is a
**prism along the shelf-edge axis, printed standing on that end** — every
layer is the identical silhouette, so nothing needs support AND every bending
stress lands in the layer plane, where the across-layer knockdown (×0.55 in
the record, ×0.33 as Prusa measure it) does not apply; both wrong orientations
are negative controls that fire every run. (3) **"12 mm" is not one number** —
boards sold as 12 mm measure 11.1–13.7 mm — and a printed PLA spring would
relax under permanent load, so the answer is a parallel slot plus a 50 mm lip
that converts slack into a gated rock (≤1.72° across the whole band) instead
of a lost grip; the out-of-scope thicknesses are declared, not hidden.

Research (`analysis/DESIGN.md`) caught a trap worth carrying: **DeWalt's US
site publishes CARTON dimensions** in its `Assembled Product` fields (a 161 mm
tool listed at 9 in); only the dewalt.co.uk metric tables are real. The three
envelope numbers the cantilever actually needs are published by nobody, so
they are DERIVED, marked as such, taken pessimistically, and then proved by an
overlap gate against a box keep-out with two collision negative controls.
Temperature is the limit that decides the material: measured hot-climate attic
air (56.6 °C, FSEC-PF-336-98) already exceeds Prusament PLA's 55 °C HDT, so
the part is declared INDOOR-ONLY and a gate asserts the 55 °C margin is under
2× so the limitation cannot quietly vanish from the deliverable.

FEA (`ace_fea`, hex8, 2 mm) is read exactly as its card says to: tip
deflection 0.39–0.42 mm across two boundary-condition idealisations (quotable
— it is what the solver is benchmarked on), interior-field peak 1.14 MPa
agreeing with the closed form within 1.5× (the sharpening), and the raw
boundary-layer peak quoted UN-derated at 2.22 MPa (×1.13) with the card's
×1.25 bending derate applied to the interior value instead of stacked on a
value already biased high. Zero interior elements exceed the allowable. A
40 %-thickness twin on the same grid is the negative control. Fatigue screened
at D = 1.3e-5.

**Three engine findings, all in `docs/FRICTION.md` (#24–#27).** The big one:
an early draft of this hook was **in two disconnected pieces** and passed
`validate`, `is_watertight`, `volume`, every clearance and stress gate, and
rendered convincingly — `Solid::shell_count()` reports 1 for it too. Only a
mesh connected-component count catches it, and the campaign now gates on that
with an NC that pins the `shell_count`-says-1 contrast. Also recorded:
`overlap_volume` refusing at exactly one keep-out offset while its neighbours
resolve; `ace_fea` correctly refusing to converge at 1.6 and 3.0 mm voxels on
this geometry (the 5 mm lip falls under two cells), which cost the campaign
its grid-convergence pair; and a voxel grid losing a 2.5 mm structural tie the
exact B-rep has — reported the part 5× softer and INVERTED the negative
control, which is now the design rule "a tie a voxel grid can lose is a tie a
slicer's perimeters can lose too" (`END_TIE` = 5 mm).

ADDED 2026-07-31 (the COLD-START EXAM, and what it earned):

The exam that had never been run: a session with ZERO context, one message, no
hints and no corrections, given a part that appears nowhere in the docs — "a
hook that hangs over a 12 mm shelf edge and holds a 1.8 kg cordless drill by
its handle; it will hang there permanently" — plus six words of method: "Follow
the repo's own rules." Graded MECHANICALLY from artefacts (the grader lives
outside the repo; the graded party must not be able to touch it), campaign tier
per docs/META_PROOFS.md §4.

**VERDICT: PASS — 10/10 required criteria, plus both probes.** It shipped
`legacy/kernel-model-examples/drill_hook.rs` (then under `crates/kernel-model/examples/`) → `hook_system/drill_hook/`:
40 gate rows, exit 0, the full 7-folder deliverable, ANALYSIS.md generated from
live numbers, workspace suite green, clippy clean. Unprompted it also produced
a print-first fit coupon, negative controls, and FEA receipts.

The probe that mattered: the brief never said creep, time, sustained or
allowable. The session derived from "permanently" alone that the part is a
sustained-load case, gated it against `creep_allowable_mpa(23 °C, 1 y)` =
2.5 MPa instead of the 10 MPa static tier, applied the hex8 derate its solver
card warns about, measured section properties off the real filleted profile
rather than retyped rectangles, and declared impact/UV/board-creep as
"required, NOT performed" with reasons. The knowledge system TEACHES; it does
not merely contain.

**What the exam earned — connectivity promoted to a THIRD oracle.** An early
draft of the hook was severed into two floating bodies by a tapered cutter
whose apex ran out through both end faces. It passed `validate`,
`is_watertight`, `volume`, the support audit, keep-out and insertion sweeps,
stress sections and STEP round-trip, and rendered convincingly —
`Solid::shell_count()` reported 1 for it too (it counts B-rep shell records,
not connected geometry). Nothing in the engine's validity vocabulary could see
it. Now `Mesh::component_count(weld_tol)` / `Mesh::is_one_body()` in
kernel-core, pinned by `tests/connectivity.rs` which asserts the property that
justifies the oracle: on a severed mesh, watertightness and volume both still
look right and ONLY the component count objects. Added to the campaign
checklist (skill + DESIGN_GUIDE §25 step 3) and to the limits ledger as §24
item 11, with the operating rule: any tapering cutter must keep its apex
strictly inside the material. Also ledgered by the exam: FRICTION #24–#27
(overlap_volume refusing at one keep-out offset, ace_fea honestly refusing to
converge at coarse voxels, and a voxel grid losing a 2.5 mm structural tie the
exact B-rep has — which reported the part 5× softer and INVERTED a negative
control, earning the rule that a tie a voxel grid can lose is a tie a slicer's
perimeters can lose too).

Process notes, recorded because they are the kind of thing that quietly rots:
the first grader was written into the shared session scratchpad and the exam
session DELETED it during its own cleanup (the grader must live where the
subject cannot reach it), and the rewritten grader's first verdict was a FALSE
FAIL — git collapses a wholly-untracked directory to `family_system/` with no
subpath, so the deliverable-locating regex found nothing. Both fixed; neither
the session's own PASS claim nor the grader's false FAIL was accepted.

ADDED 2026-08-01 (NULLSPIN campaign — Printables "Designer Challenge: Geared
Spinners" entry, `examples/nullspin.rs` + `spinner_system/nullspin/`, the full
7-folder standard): a grounded-carrier ("star") epicyclic fidget spinner —
66T ring, 42T sun, 6 × 12T planets on FIXED axes at m1.0 / 25° PA. The claim is
the counter-rotation at an exact integer ratio (7·R = 11·S = 462, a compile-time
identity) plus the angular-momentum cancellation it enables (eta = 0.981 on the
exact B-rep, published as a band with its sensitivity table and its common-mode
invariance PROVED to 1e-16 on the bearingless set). 39 gates, 9 of them negative
controls, every oracle carrying one.

**New capability this campaign wrote, benchmarked before use (§25.7
answer-type 2):** a spin-down solver for the research's governing model
`I·dω/dt = −Σ cⱼω^pⱼ`, solved by exact quadrature with the ω→0 singularity
removed analytically (substitution `ω = ω₀·s^{1/(1−p_min)}`). Gated against TWO
closed forms (pure power law and pure Coulomb) to <0.5%, plus a
meta-negative-control proving the benchmark can go red. Also written here
because the engine has no API for them: transverse contact ratio (external AND
internal, each driven by its own negative control), the ISO undercut floor, a
polygon polar-second-moment helper benchmarked against πR⁴/2, an EN 71-1 section 4.10
rod-rule gate with an 8-planet negative control that fires, and a grounded-carrier
STAR pose evaluator (`kinematics::instance_poses` is ring-fixed/sun-driven and
does not cover a held carrier).

**What the campaign earned, and what it refused.**
- **Designing ON the 45° support threshold is a coin flip.** The ring's bed
  chamfer measured 1.037e-6 mm² of steep area at exactly 45° — float noise, but
  a `steep_area < 1e-6` gate is right to fire on it (mesh positions are f32, so
  the f64 normal carries its own representation noise; `support_free_report`'s
  own 1e-4 cosine slack is for at-limit facets, not past-limit ones). Every
  relief in this campaign is now cut at 1.40 rise:run. The fix was geometric;
  the gate was NOT loosened.
- **A hoop-expansion snap does not exist at this scale, and the arithmetic says
  so.** The frozen spec's Ø6.40 barb over a Ø5.60 hole is 14 % hoop strain,
  eight times PLA's 1.67 % yield strain; a compliant cantilever was evaluated
  next and refused too (a ~3 mm stub reaches yield at ~0.045 mm of tip
  deflection). Shipped: a 0.025 mm click ring at 0.89 % strain, the DRYBOX joint
  class. The insertion/pull-off FORCE is declared REQUIRED, NOT PERFORMED — no
  registry solver covers this joint (the `contact` card is a planar beam against
  a RIGID obstacle) and the elastic Lamé bound over-predicts a printed joint
  that conforms plastically.
- **The frozen spec's spin-time estimate did not survive its own physics.** The
  spec costed the planet journals and the 608 but not the RING'S OWN AXIAL
  THRUST — in a star arrangement the ring is not on a bearing, so held flat its
  weight rubs on the held frame at r ≈ 34.75 mm. That single Coulomb term is
  67 % of the budget, and the campaign publishes 2.4 s (band 1.4–4.0 s) with the
  derivation and the named dominant term rather than the spec's 15–25 s.
  Deliberate consequence, also published: ring mass is NOT a lever, because
  `T ∝ m_ring` and `I_eff ∝ m_ring` together.
- **First campaign to use `optimize::Study` for real geometry.** The rotors are
  prisms, so `I_zz = ρ·h·J` is exact in the face width and the study runs on the
  polygon second moments of the very outlines the solids are built from;
  `gate_study` asserts the shipped point IS the re-evaluated optimum. The ring
  wall is declared over the legal extrusion-line set {3, 4, 5 lines} with the
  5-line floor as an ACTIVE constraint — the unconstrained optimum prefers a
  thinner wall, and the campaign records that it bought the stiffer one.
- **Authorship left open on purpose.** The contest forbids AI-generated models
  and publishes no guidance on parametric/code-authored CAD. `publish/` carries
  a deliberately INCOMPLETE AUTHORSHIP section for the human author, and no
  listing copy implies hand-modelling.

ADDED 2026-08-01 (NULLSPIN v2 — the ball thrust race: the campaign attacks its
own published dominant loss, `examples/nullspin.rs`, 85 gates, exit 0):
- **The ring's axial support moved from sliding to ROLLING, and the spin-time
  prediction went 2.4 s → 5.7 s (2.4×) with v2's PESSIMISTIC end (3.4 s) above
  v1's OPTIMISTIC end (3.9 s).** v1 had measured and named its own killer: in a
  grounded-carrier star the ring is not on a bearing, so held flat its weight
  rubbed on the held frame at r ≈ 34.75 mm — 0.406 N·mm, 50 % of the budget,
  Coulomb class. v2 puts that load on **24 loose Ø1.50 mm chrome-steel balls**
  in a printed annular channel in the base rim, running on the ring's own flat
  underside. Both architectures are recomputed every run by the same solver and
  published side by side; the v1 row is not quoted from memory.
- **The choice is forced, not preferred, and the proof is in ANALYSIS.md.** Two
  bodies rotating about the same axis have a relative motion that is a rotation
  about that axis, so every off-axis contact between them SLIDES — a rolling
  element between them is the only remaining option. Corollary, also gated: a
  planet flange can never carry the ring axially by rolling either (for parallel
  axes the only rolling locus is the vertical pitch line, which cannot carry
  axial load), and a pad there reads 0.78× at best.
- **The rolling term is published as a RIGOROUS BOUND, not a fitted
  coefficient.** No rolling-resistance coefficient exists for a hard ball on
  printed PLA. What is rigorous is that a pressure resultant cannot lie outside
  its own contact patch, so `f ≤ a`, the Hertz contact radius — computed from
  PLA's own published modulus. Every v2 spin time is therefore a LOWER bound on
  the model's answer. New helpers `e_star`/`hertz_a`/`hertz_p0`/`hertz_delta`
  are benchmarked BEFORE use (B5 against the independent `δ = (9F²/16RE*²)^⅓`
  path via `a² = Rδ`, B6 the meta-negative-control), and a rigid-ball
  sensitivity shows the answer does not rest on the steel constants (0.6 %).
- **Two new falsifiability gates.** G18e drives the ring back onto a sliding
  land — which is also what a seized or contaminated race does in the field —
  and the same solver must return ≤0.75× (it returns 0.42×). G18f is an
  ANTI-GAMING gate: the ball count is swept 6–48 and the predicted spin must
  move <5 % (it moves 3.3 %), so the one free integer cannot have been tuned.
- **Two v1 numbers CORRECTED, both in the worse direction.** (a) The planet
  thrust pads were costed at a 3.25 mm arm; the planet's own bed relief makes
  the touching annulus 3.45–3.50, so the arm is 3.475 and v1 was 6 % optimistic.
  (b) v1's usage note "held edge-on the ring term largely vanishes" was
  incomplete — the load does not stop at the mesh, it leaves through six sliding
  pin journals reflected by 5.5×. Costed end to end it is **1.30×, not
  "vanishes"** (G19b asserts both halves), and with the race fitted the advice
  **INVERTS**: flat is now 1.84× edge-on (G19c). Listing and instructions updated.
- **The ball set is a third rotor and it is in the eta ledger.** Its centres
  orbit at exactly ω/2, so 0.333 g at r 34.6 carries real z angular momentum in
  the ring's sense — which happens to nearly cancel the residual the printed set
  leaves: **eta 0.9814 → 0.9975**, worst differential corner 0.9374 → 0.9545.
  G9b's exact common-mode-flow invariance was NOT relaxed to accommodate the new
  steel; it was re-pointed at the all-PLA case where it is exactly true, and the
  shipped set's (larger) deviation is measured by G9b2 instead.
- **Refused, with numbers, in ANALYSIS.md**: a second 608 (no vertical room —
  the cap fixes the stack at 12.0 mm); 608s as rollers (k = 3.16 reflects as
  k^1.5 ⇒ 4× WORSE); printed rollers (gain is r_journal/r_roller and the 2.3 mm
  of space caps the roller at Ø2); a free-floating thrust washer (exactly 1.00×
  — two Coulomb interfaces in series is a theorem, not a measurement); a central
  web to a small-radius support (the only cheap SLIDING fix, ~4.5 s, but it adds
  ≈720 g·mm² of ring-sense momentum against an eta budget of ≈520 — **the
  spin-time fix and the campaign's actual claim are in direct conflict and eta
  wins**); magnets (unsourced force data + an active 2026 CPSC recall for magnet
  fidget-spinner sets); and ball races under the six planets too (~13 s — it
  wins on physics and loses on assembly: 18 more loose Ø1.5 balls loaded blind).
- **Costs, stated.** The race rim adds ~1.6 g of PLA, which makes the 28 g mass
  ceiling an ACTIVE study constraint and pulls the sun face 8.20 → 8.16 mm. The
  study's frame-mass model was also found CIRCULAR (the shipped t_sun shortens
  the post, which frees mass, which moves the optimum — it visibly oscillated on
  the 0.02 grid) and now bounds the post over the whole t_sun window instead.
  The channel's outer retaining lip is 0.60 mm, below the profile's 1.20 min
  wall — declared rather than hidden, with G18g computing the hoop stress it
  actually carries (0.008 MPa against 55). No cage ships: one cannot be made a
  single body inside a 2.00 mm channel, and one riding the channel floor would
  cost more than the balls do. Ball-to-ball rub is bounded (adjacent balls touch
  on their own spin axes, so only the ω/2 component slips) and ball migration is
  declared REQUIRED, NOT PERFORMED.

ADDED 2026-08-02 (NULLSPIN v3 — ZERO non-printed parts, `examples/nullspin.rs`,
86 gates, exit 0). The bought list is EMPTY: no 608, no steel balls, no magnets,
screws, nuts, weights or inserts. The `You also need:` line reads *nothing*.
- **The cost is measured, not described: 6.2 s → 1.9 s.** All three
  architectures are now rebuilt by ONE solver on ONE rotor in ONE run, each
  carrying its own I_eff — v1 (sliding ring land + 608) 2.5 s, v2 (24-ball race
  + 608) 6.2 s, v3 (fully printed) **1.9 s / 17 rev, band 1.2–2.9 s**, ceiling
  with the ring's support deleted 3.9 s. Coulomb share 31 % → **90 %**. The
  README, BOM, instructions and Printables listing all say "about 2 seconds" in
  those words. G17f is the falsifier: put the hardware back and the same solver
  must return a strictly better number.
- **Where the loss went.** The 608 (0.1881 N·mm, ω^0.5) is replaced by the sun's
  printed thrust land (0.2192, Coulomb) — near-parity in magnitude, strictly
  worse in class. The 24-ball race (0.0096) is replaced by six printed thrust
  pads (0.3582) — 37×, and 47 % of the whole budget. Coulomb torque is μWr and
  area-independent, so the ONLY lever is the arm: the post shrank Ø7.90 → Ø5.50
  the moment it stopped having to be a 608 bore (sun arm 3.70 mm), and the ring
  pads moved to r 34.60, the start of the ring's own continuous flat underside.
- **eta did NOT get worse — I_eff did, and the entry says so.** Removing
  610 g·mm² from the sun side would read 0.9044 at v2's design point (published
  as a row). The study rebalanced by thinning the ring (t_ring 4.50 → 4.00,
  t_planet 4.50 → 3.50) and the shipped eta is **0.9990**, above v2's 0.9975.
  What the deletion actually cost is inertia: I_eff 15011 → 12666 g·mm².
  G9b's exact common-mode-flow invariance is now true of the **shipped** set for
  the first time (no non-scaling steel anywhere); G9b2 measures how much the 608
  used to break it. G8's concentricity residual collapses to 0.000 mm because
  the sun is no longer doubly located.
- **The central web was RE-OPENED (eta is a supporting receipt now, not a veto)
  and refused again — on grounds eta has nothing to do with.** G20a: the web
  must clear the sun, so the rim needs a 4.10 mm dead shell at r 34.25–36.50
  (3185 g·mm²); swept over the whole t_sun range the best ring FACE the eta
  budget then leaves is 0.94 mm, where the mesh needs 3.50. G20b: printed
  teeth-down the spokes bridge 30.25 mm against a 6.00 max. v2's eta-based
  refusal was the weaker argument — the web's own eta row reads 0.9607, still
  above the gate floor.
- **Printed balls: refused, and NOT for either expected reason.** G21a recomputes
  Hertz for PLA-on-PLA rather than scaling the steel answer (E* falls 1.97×, a
  grows only 1.25× to 0.0075 mm, p₀ FALLS to 12.1 MPa) — they would give 3.8 s.
  G21b records a genuine negative result: the support oracle does NOT refuse a
  Ø1.50 sphere (its whole overhang sits inside the first-layer tolerance) and is
  shown not to be blind (Ø6.00 reports 12 mm²). G21b3: not a stress refusal
  either (34.9 MPa with ONE ball carrying the ring). What refuses them is FORM
  ERROR — 7.5 layers tall, ±0.100 mm staircase, 167× a G25 ball — whose
  governing loss has no model here.
- **The on-axis point pivot wins on drag and is refused on ergonomics, with the
  price published (G21c): 2.8 s vs 1.9 s, +44 % left on the table.** A blind
  on-axis socket and a static thumb pad are geometrically exclusive.
- **New gates.** G17a–i (arm minimality, three bearing pressures, an
  ANTI-GAMING pad-count sweep that must be EXACTLY flat, the running-fit and
  §7.7 hub-recess checks, the printed-pin floor), G20a/b, G21a–d, G22a–c (the
  cap now retains the sun; the cap press fit; no part carries a bridge).
- **Two things found by gates, fixed geometrically.** (a) Recessing the hub by
  one axial clearance lands it exactly on the arms' top plane — §7.7 rule 3 —
  and the chain went invalid (genus 2, not watertight); it is two clearances
  now. (b) The inherited `xy_clearance_tight` cap fit is 1.27 % hoop strain on a
  Ø7.90 post but **1.82 %** on Ø5.50, past PLA's 1.67 % yield. G22b fired; the
  interference drops to `CLICK_R` (0.91 %, the print-proved click-ring class)
  with a negative control asserting the old fit still fails.
- **Deleted with the hardware**: the ball channel and race rim, the 608 collar,
  the sun's bearing lip, `optional/sun_p_bearingless` (the shipped sun IS
  bearingless), the ball STL, every `bought` row in `bom.csv`, the "ZZ or open,
  never 2RS" guidance and the loose-steel-ball small-parts warning. Assembly is
  5 steps; the coupon has 3 printed-to-printed fits and takes ~12 min.

ADDED 2026-08-02 (NULLSPIN v4 — GEOMETRIC top-spider retention, `examples/nullspin.rs`,
110 gates, exit 0). v3's cover was held on by friction and its own G16e said so:
the click band's 0.025 mm interference reached exactly ZERO at 0.025 mm/side of
printer error, against the campaign's own worst-case XY figure of 0.15 mm/side.
v4 retains by a shoulder instead, and the dependency becomes a pass/fail proof.
- **A bayonet, per RESPOOL's zero-preload law.** Each pin gets a Ø2.70 NECK
  through the spider and a radial FIN above it; each spider arm gets a slot.
  Drop on 7° out, twist to a hard stop (3.30 mm at the pin circle), and the fin
  overhangs the slot's outboard wall by **1.15 mm of solid material**. Nothing
  is strained at rest and assembly needs no strain at all.
- **The dependency is gone, and that is measured on solids.** G16b/c: engagement
  survives 0.15 mm/side XY (+0.85 left) and XY + the 0.20 mm Prusa foot on BOTH
  members (+0.45). G16d: the shipped pair is not touching anywhere inside its
  0.35 mm float (zero preload). G16e: lift it 3 mm and it collides with the pins
  (12.8 mm³). G16f: rebuild one pin and one arm with 0.35 mm/side of error on
  every retention surface and the same lift still reads 0.276 mm³/pin. TWO
  negative controls must read exactly zero — G16g deletes the slot's lip (round
  holes), G16h poses the SHIPPED part untwisted. G16j: 82 % of the twist must be
  undone before release. G16k/l: capacity **35.2 N** (435× the carried weight)
  from neck bending, with the shoulder's bearing proved NOT to govern — a real
  number from section properties, where v3 had a μ-dependent bound it called
  absurd itself.
- **The snap CLASS is refused on record, not just the spec's barb (G16m).** A
  hoop's bore strain is δ/a exactly, so a Ø5.60 hole in this arm stretches
  0.047 mm before yield while a snap must swallow the same 0.30 mm stack the
  engagement does — 6.4× short, and no variant inside the 12 mm envelope closes
  it. The Ø6.40 barb NC (14.3 %) is kept.
- **A design law this campaign had to find: a support-free retaining face is
  always a ≥RELIEF_SLOPE cone, and a cone WEDGES 1.40 : 1.** If any of that
  horizontal share is TANGENTIAL, the bayonet cams itself back toward the entry
  under ANY load — the failure is scale-free, so the carried weight alone is
  enough to unscrew it. The fix is geometric: trim the fin to ±1.00 mm about the
  pin so its overhang is symmetric and wholly inside the wall's material, and
  the whole wedge force becomes RADIAL (resisted by the spider's hoop, which
  would have to grow 1.15 mm = 4.3 % strain to let go).
- **The slot follows the ARC, not a chord.** The pin travels the pin circle;
  over 7° that bows 0.20 mm inboard, most of one C_FREE, and it also arrives at
  the bulge 7° out of square with the fin's flats. G16h caught the second one as
  0.014 mm of interference. The slot is now drawn in (radial offset, arc length)
  and mapped through `arcp`.
- **Found while doing it, and recorded rather than quietly fixed:** v3's SHIPPED
  click band was Ø5.55 in a Ø5.60 hole — a 0.025 mm/side CLEARANCE. The
  interference its strain gates were computing was not in the geometry at all.
- **What got worse, stated plainly.** The fin makes the pin 0.93 mm taller,
  which tightens the envelope constraint and re-opened the design study;
  hollowing the pins also gave back 0.40 g, and the optimiser spent it — G11
  failed loudly at t_planet 3.50 and the shipped point moved to **4.00**
  (I_eff 12857 → 13056 g·mm², eta 0.9990 → **0.9950**). Assembly is 6 steps, not
  5. The fit coupon is now TWO pieces (`coupon_key` carries the shipped slot,
  because a bayonet cannot be gauged by one body). The v2 ledger row moves
  6.2 → 5.9 s and edge-on 2.7 → 2.6 s on the new rotor. Shipped spin time
  (1.9 s), printed mass (27.3 g) and the empty bought list are unchanged.
- **Two more things gates caught during the change.** G22c (no part carries a
  real bridge) fired at 21.9 mm on the coupon — the shipped pin's thrust boss
  was hanging in air under the plate; it is cut off at the plate now. G23c's
  no-rim negative control started PASSING capture by accident once the arms
  took the shipped planform, so its arms are explicitly truncated inboard of
  the ring again.
- **Deleted:** `CLICK_R`/`CLICK_H` and the six click bands; v3's G16a–f
  (hoop strain, Lamé pressure, the μ-dependent pull-off bound and the disclosed
  calibration dependency). `CAP_PRESS_R` keeps the cap's 0.025 mm press — the
  ONE interference fit left, and the one residual calibration dependence, which
  G16n prints every run so the headline cannot imply otherwise.

FIXED 2026-09-02 (friction l12_mini_case F3/F4, uphill_roller F2/F3;
`campaign/fixlog/2026-09-02-export-demotion-wedge-thickness.md`): **export
demotion receipt** — `export_stl` / `export_3mf` (and `asm_export` per-instance
entries) on the `voxel_healed` route carry `demotion: {reason, boundary_edges,
non_manifold_edges, non_orientable_edges, non_manifold_vertices,
degenerate_triangles, self_intersections, exact_triangles, witness[≤8]}`,
naming the first failing check of the abandoned exact tessellation and
locating it in the body's frame; the routing decision is untouched and exact
exports are unchanged. **`wall_thickness` sampler** — area-uniform stratified
sampling (65 536-sample budget, deterministic `(triangle, sub-cell)` hash
jitter, no RNG state) replaces one centroid ray per triangle, so mirror-image
dovetail grooves read 76.6 vs 76.8 mm² (the campaign measured 19.6 vs 101);
meshes whose triangles are all below one budget cell (fine voxel-route meshes)
read byte-identically; per-triangle `thickness` keeps the centroid ray. New
`exclude_wedge_deg` sets knife-edge readings (ray exits through an
edge-adjacent face at a convex material dihedral below the threshold) aside
under `thin_area_wedge` / `thin_area_total` / `thin_wedge_witness`;
`thin_witness` (≤ 8 thinnest flagged samples) and `samples` are always
reported. Tests: `kernel-core mesh::thickness`, `kernel-api
tests/export_demotion.rs`, `tests/wall_thickness_wedge.rs`.
