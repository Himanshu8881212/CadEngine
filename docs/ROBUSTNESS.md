# Chain-Robustness Evidence (feature-chain fuzz corpus)

Published pass rates for random feature chains — the Level-6 measurement gate of
`BAR.md` and the seed of the Level-9 "robustness evidence" requirement. Produced by
`crates/kernel-brep/tests/fuzz_chains.rs` (deterministic seeded corpus, no
dependencies). **These numbers are deliberately honest**: the generator is not
curated to pass, and the in-repo test ratchets a floor under the lowest measured
rate (5 points at the noisy pre-determinism baseline, 2 points since the corpus
became run-deterministic; the manually-run Level-9 evidence corpus pins its
measured rate EXACTLY since 2026-07-30) so regressions fail loudly while the
real number stays published here.

## Measured 2026-07-30 — the two Level-9 residual seeds FIXED: N=10 000 at 100.00 %

The two chains that survived every wave since the corpus first ran at N=10 000
(2026-06-10, "the honest residue") are fixed. Both were `closed=false
manifold=false` stitch holes, and both root causes were **micro-scale blind
spots of the stitcher's own healing machinery** — not the W5 snapping (the
seam snapper moved nothing in either failure; the "sagitta-scale budget-free
interleaving" mode W5 capped was a different, already-closed class):

1. **seed 83894724552572** (chain #9084, op 1 `intersection`, sphere ∩ sphere):
   `resolve_t_junctions` guarded degenerate edges with `len2 < EPS` — but
   `len2` is the SQUARED length, so every edge shorter than √EPS ≈ 3.2e-5 mm
   (80× the healing tolerance) was silently exempt from receiving T-vertices.
   Where the two spheres' seam polyline crossed both operands' facet edges
   within one micro cut stub (2.4e-5 mm), operand B's crossing vertex — ~1e-9
   off operand A's stub, t = 0.54 interior — could never be inserted, and the
   sliver filter's safety argument ("a dropped sliver's apex gets healed into
   the neighbour's edge") broke: an unhealable 2.4e-5 micro-triangle hole.
   The guard now rejects only sub-`WELD_EPS` edges (physically impossible
   after welding, i.e. NaN/repeated-id protection only) — micro stubs heal.
2. **seed 83894724550888** (chain #7400, op 7 `difference`, holed 5-gon
   extrude off a 6-op accumulated body): two fragments of ONE operand face
   disagreed about a shared seam corner by **1.051e-7 — 5 % OVER the greedy
   weld's 1e-7 first-fit ball** — so the welder kept two copies, and the pair
   is *unstitchable by construction*: `resolve_t_junctions` cannot insert
   either copy into a long edge ending at the other (the projection parameter
   lands within EPS of the endpoint and is rejected as the endpoint itself),
   leaving an unpairable zero-area slit (`8→74, 74→73, 73→8`). Stitch now
   merges vertex clusters closer than `TJUNCTION_EPS` right after welding
   (min-id union-find on a `TJUNCTION_EPS` grid, ids ascending, representative
   keeps its own bits — deterministic): the stitch's sliver filter and healer
   already treat that scale as noise, so two vertices under it ARE the same
   point at stitch resolution. The fuzz corpus was the arbiter that this
   merge regresses nothing (below).

Re-measured after both fixes (the 2026-07-30 threaded arrangement is
untouched: both changes live in the sequential stitch, and the determinism
40×-rebuild + threading-parity suites stay green):

| corpus | chains | pass rate | runs |
|---|---|---|---|
| standard N=200 | 200 | **100.0 % (200/200)** | in-suite |
| deep N=2000 | 2000 | **100.0 % (2000/2000)** | 2, byte-identical |
| Level-9 N=10 000 | 10 000 | **100.00 % (10 000/10 000), both runs byte-identical** | 2 |

Ratchets: standard/deep floors stay 98.0 (measured rates unchanged at 100.0 —
nothing to raise). The Level-9 corpus rose 99.98 → **100.00** and its floor is
now the measured rate itself — an EXACT 10 000/10 000 pin in
`fuzz_10000_feature_chains_level9_corpus`, deliberately tighter than the
default-suite 2-point convention: that headroom exists for cross-platform libm
drift in every-run gates, while the Level-9 test is the manually-run evidence
measurement on the publishing machine, where the corpus is run-deterministic;
any future flip must be re-diagnosed, never absorbed. Both formerly-failing
seeds are additionally pinned in the DEFAULT suite by
`residual_level9_seeds_stay_fixed` (full-chain replays), so a regression is
loud without the manual 10k run. Honest scope note: the sub-heal-scale
duplicate merge slightly coarsens stitch's vertex resolution (pairs under
4e-7 mm now unify; previously only pairs under 1e-7 did) — this is the
documented `TJUNCTION_EPS` doctrine of `tol.rs` applied to vertices, and the
whole corpus plus the kernel-brep suite (incl. exact-volume and seam-snap
gates at 1e-9) hold under it.

## Measured 2026-06-10 (W5) — parameter-space triangulator + relaxed seam snapping

Level-7 completion work: W3's planarity contract is lifted. Warped curved-tagged
faces are now ear-clipped in their surface's **parameter space**
(`geom::SurfaceChart` — cylinder unroll `(r·θ̃, z)` with the angular seam
anchored on the ring's mean radial direction; sphere via the **gnomonic**
projection about the ring's mean direction, which removes the (θ, φ) pole
singularity instead of special-casing it; cone via its isometric development;
torus `(R·θ̃, r·ψ̃)`), so `snap_seam_vertices` may move seam vertices that WARP
their incident facets: oblique plane∩cylinder cuts, cut rims ending mid-facet,
and plane-less quadric∩quadric seams (cylinder∪cylinder) now land on the exact
intersection (≤ 1e-9 per surface, asserted in-tree). The chart engages only when
a ring is measurably warped (`CURVED_WARP_EPS` = 1e-6, above stitch noise ≤
~4e-7, far below the sagitta-scale warp) — a planar ring keeps the old
projection-plane path byte-identically, which is what kept the no-snap corpus
bit-stable (a prior prototype that charted every curved face unconditionally
re-diagonalised planar facets and shifted marginal chains; it was reverted).

As in W3, the deep corpus was the design arbiter — every variant measured on
N=2000 (floors 98):

| variant | deep rate | failure mode |
|---|---|---|
| W3 baseline reproduction (planarity contract) | **100.0 %** | — |
| chart triangulator alone, snapping unchanged | **100.0 %** | (verified no-op) |
| + chart guard replaces planarity (vertex-level accept) | 99.7 % | partially-snapped seams: an unmovable chord-depth junction strands beside snapped corners; the zigzag's two sides overlap in a near-zero-thickness fin, 4-half-edge sandwich in the NEXT boolean (seed 83894724543990 traced) |
| + degenerate-ear guard only | 99.7 % | same (disproved first theory: 3-D-degenerate chart ears were real but not the root cause) |
| + **seam coherence** (all-or-nothing per surface pair) | **100.0 %** | — |
| + plane-less (quadric∩quadric) seams admitted | **100.0 %** | — |
| + spheres / cones / tori chart-owned | **100.0 %** | — |
| + budget-free moves for ALL kinds (incl. spheres) | 99.9 % | sphere sagitta is 10–20× a cylinder's (~0.3 mm on a 16×12 fuzz sphere); freed moves interleave neighbour faces beyond what weld/T-heal absorb (seeds 83894724544075/544699) |
| + budget-free moves for plane/cylinder only | **100.0 %** | …but the N=10 000 superset caught the same mode on a coarse 11-gon r=7.5 cylinder (sagitta 0.30 mm, seed 83894724546286): the danger scales with SAGITTA, not surface kind |
| + 0.05 mm absolute cap on budget-free moves (**final**) | **100.0 %** | — (and N=10 000 back to its two pre-W5 residual seeds) |

Final config, all three corpora: standard N=200 **100.0 % (200/200)**, deep
N=2000 **100.0 % (2000/2000)**, each 3/3 runs byte-identical; Level-9 N=10 000
**99.98 % (9998/10000)** — exactly the two documented pre-W5 residual seeds,
re-measured below. Floors stay 98.0 (measured rate unchanged at the ratchet).

The load-bearing pieces, each traced from a failing seed:

1. **Parameter-space clipping** (`face_clip_p2`, `tessellate_curved_verbatim`,
   the adaptive fallback): a warped simple-on-surface boundary cannot fold in
   its chart, where a projection-plane ear-clip self-intersects (the W3 failure
   class). In chart mode a 2-D-convex corner can still be a 3-D-collinear run
   whose ear would be silently dropped as degenerate, deleting a shared boundary
   step — such corners are not ears (`ear_clip_ring_tris`; unreachable in a
   projection plane, where affine maps preserve collinearity).
2. **Chart guard** (`chart_snap_safe`) for chart-owned faces: proposed ring maps
   inside the chart's injective domain, stays simple, keeps its area
   orientation, and neither is nor becomes thinner than the heal scale (area ≤
   perimeter × `TJUNCTION_EPS` — inflating a zero-thickness sliver face into a
   finite-width fin self-overlaps the boundary; the planarity guard's
   `INF > INF` never caught this and W3 simply never moved those vertices).
   There is deliberately NO per-wedge flip condition: a sagitta-scale chord
   wedge legitimately reverses convexity when its vertices land on the true
   curve, and ear-clipping triangulates any simple ring exactly.
3. **Seam coherence**: a surface pair's seam moves all-or-nothing. A vertex
   vetoes its seam unless it snaps, already lies on the exact intersection
   (≤ 2× `TJUNCTION_EPS` per surface), or is a mid-edge chord sample whose host
   stretches stay straight under the proposed moves (a cap-rim chord sample at
   sagitta depth is construction geometry, not a stranded seam vertex — without
   this exemption every facet-rim T-junction vetoed its whole seam).
4. **Budget split + absolute cap**: plane/cylinder-only vertices snap with the
   full sagitta-scale budget (2× the smallest incident curved chord band,
   absolutely capped at `SNAP_MOVE_CAP` = 0.05 mm — the kernel's
   absolute-tolerance convention; every practical tessellation the kernel emits
   stays under it) and their straight-seam chord samples snap with their seam;
   sphere/cone/torus vertices keep W3's `0.1 × min-edge` budget and mid-edge
   skip (the honest boundary — the measured 99.9 % and N=10 000 rows).

In-tree gates: `quadric_quadric_seam_keeps_the_chord_contract` was upgraded
honestly — the perpendicular cyl∪cyl seam (a space quartic, no conic closed
form) now has **every seam vertex on BOTH true cylinders to ≤ 1e-9** (was: within
the 1.7e-2 chord band, the W3 contract), the result is watertight and survives a
chained boolean through the warped seam region;
`oblique_cut_seam_snaps_and_carries_the_exact_ellipse` (the W3-rejected oblique
class, now exact and Curve-tagged); `sphere_plane_seam_snaps_within_w3_budgets`;
`surface_charts_keep_warped_on_surface_rings_simple` (incl. a polar-cap ring the
(θ, φ) chart would shred); W3's `cut_seam_vertices_land_on_the_true_cylinder`
(⟂ cuts + keyway corners at y=√21, exact_volume to ≤1e-9 of πr²h) is green and
unchanged. Honest volume note: the snapped seam tightens the PL boundary itself
(perpendicular cyl∪cyl faceted-volume error 1.79 → 1.30 mm³ of 170.24 mm³
ground truth) but `exact_volume`'s bulge corrections assume chord facets and
now partially double-count on warped seam facets (0.31 → 0.59 mm³, still
beating the faceted value; both contracts were facet-level under W3 too) —
measured in `quadric_quadric_union_volume_stays_facet_level_and_beats_faceted`;
a warp-aware correction in `validate.rs` is the named follow-up. Known honest
boundaries: seams in configurations that leave stranded chord-depth junctions
(e.g. unions with COPLANAR coincident caps, a pre-existing fragile class) are
coherence-vetoed back to chord accuracy rather than half-snapped; sphere/cone/
torus seams snap only within the W3 budgets.

## Measured 2026-06-10 — Level-9 evidence corpus (N=10 000, 2 runs)

HISTORICAL NOTE: the two residual seeds this section enumerates were FIXED on
2026-07-30 (section above) — the corpus now measures 100.00 %.

The BAR.md Level-9 robustness bar reads "≥99.5% on a 10k-chain fuzz corpus".
Measured with the in-tree `#[ignore]`d test `fuzz_10000_feature_chains_level9_corpus`
(`cargo test -p kernel-brep --release --test fuzz_chains -- --ignored fuzz_10000
--nocapture`), seeds `83894724543488 + i` for i < 10 000 — a strict superset of the
N=200/2000 corpora:

| corpus | chains | pass rate | runs |
|---|---|---|---|
| Level-9 N=10 000 | 10 000 | **99.98 % (9998/10000), both runs byte-identical** | 2 |

The report's one-decimal print shows "100.0 %"; the exact rate is **99.98 % — two
chains fail**, identically in both runs (the corpus is deterministic), both in the
familiar `closed=false manifold=false` stitch class and both beyond the N=2000
prefix (chains #7400 and #9084 of the corpus):

```
seed=83894724550888 op=7 [difference]:   difference extrude_with_holes 5-gon r=5.7 h=5.8 holes=1 at (14.1, -3.8, -13.5) overlapping → closed=false manifold=false genus=1 shells=1 χ=-1
seed=83894724552572 op=1 [intersection]: intersection sphere r=5.1 16x12 at (5.4, -4.9, 10.4) overlapping → closed=false manifold=false genus=1 shells=1 χ=1
```

Histogram: difference 1/11 693, intersection 1/11 637, every other op kind 0 —
including all 10 000 base constructions and all 8 590 filleted-operand booleans.
The Level-9 ≥99.5 % bar is met with a 48-chain margin (the roadmap's stretch
≥99.9 % is met too); the bar is **not** 100 %, and the two seeds above are the
honest residue — replayable via `FUZZ_SEED=<seed> … replay_chain_from_env_seed`,
left unfixed here because this wave was performance-only (no boolean changes).
The in-tree test ratchets the same floor as the deep corpus (98.0 %).

RE-MEASURED after the W5 boolean change (parameter-space triangulator + seam
relaxation, the section above): **99.98 % (9998/10000), 2 runs byte-identical —
the same two residual seeds, unchanged.** This corpus also did real design work
for W5: an intermediate variant that freed plane/cylinder move budgets without
an absolute cap held N=2000 at 100.0 % yet failed one extra 10k chain (seed
83894724546286, a coarse 11-gon r=7.5 cylinder whose 0.30 mm sagitta-scale moves
out-ran the healers) — the `SNAP_MOVE_CAP` row of the W5 table is what restored
this corpus to its pre-W5 residue.

## Measured 2026-06-10 (W3) — WITH SSI seam snapping wired into booleans

HISTORICAL NOTE: the planarity contract this section ships was deliberately
LIFTED by the W5 parameter-space triangulator (section above) — the W3 numbers
and root-cause analysis remain the reference for WHY the contract existed.

Level-7 work: cut-seam vertices are now Newton-snapped onto the exact
surface–surface intersection inside the boolean stitch (`snap_seam_vertices`,
driving `ssi::project` / the new fully-determined 3-surface `ssi::project3`).
The fuzz corpus was the design arbiter for HOW MUCH snapping is safe — every
acceptance-rule variant was measured on the deep N=2000 corpus (floors 98):

| acceptance rule | deep rate | failure mode |
|---|---|---|
| no snapping (baseline) | **100.0 %** | — |
| unrestricted snapping | 88.5 % | warped facets fold under ear-clip |
| distance caps (min-edge / wedge-margin / guards) | 99.3–99.7 % | same, fewer |
| + plane-anchored seams only | 99.6 % | sphere-seam warps |
| + cylinders only | 99.8 % | mid-facet rim endpoints still warp |
| + **planarity guard** (final) | **100.0 %** | — |

Root cause of every snap-induced failure class: a snapped vertex that does not
land on a configuration keeping its incident facets EXACTLY planar warps them by
up to the facet sagitta; ear-clipping a warped polygon in a projection plane can
self-intersect and emit folded, double-covered triangles whose surplus directed
edges cannot twin-pair — the NEXT boolean of a chain then stitch-explodes
(diagnosed via duplicate directed-edge tracing on seed 83894724544611). The
shipped contract is therefore: **plane∩cylinder seams snap exactly where the
snapped facets stay true planes** (⟂ cuts, axis-parallel keyways/notches and
their 3-surface corners — vertices land on cylinder generators), everything
else (oblique endpoints mid-facet, sphere/cone/torus, quadric∩quadric) keeps
chord vertices, asserted in-tree by
`cut_seam_vertices_land_on_the_true_cylinder` (≤1e-9 on the true cylinder,
exact_volume to ≤1e-9 of πr²h, keyway corners at y=√21 to ≤1e-9) and
`quadric_quadric_seam_keeps_the_chord_contract`. Two supporting fixes the
corpus demanded: result faces are stripped of inherited collinear chain
micro-subdivisions at stitch time (born clean, `chain_redundant_in_rings`), and
soup triangles carry their OWN plane normal rather than the face's averaged
Newell normal (a stitched face is only planar to ~weld tolerance; re-splitting
its fragments against a borrowed average plane shifted seam corners at the heal
scale — the last 1/2000 failure, seed 83894724543716).

Both corpora hold the mop-up baseline: standard N=200 **100.0 % (200/200)**,
deep N=2000 **100.0 % (2000/2000)**, 3/3 runs byte-identical each. Floors stay
98.0 (no ratchet change — measured rate unchanged).

## Measured 2026-06-10 — AFTER the Level-6 mop-up (zero residual failures)

The nine residual stitch explosions left after R5 are FIXED — the deterministic
corpus now passes 100.0 % at both sizes. Three `booleans.rs` root causes, found by
replaying the nine seeds:

1. **Non-unit split-line normals** (`split_triangle_by_segments`): the in-plane
   line normal handed to `split_convex_by_line` had length = |cut segment|, so the
   EPS "on-line" band was EPS/|segment| in real distance. A SHORT cut stub (a
   chord clipped to a sliver of the other operand near a seam corner, often
   ~1e-5 mm) ballooned the band to ~1e-4, swallowing genuine crossings — the two
   operands then disagreed about the seam polyline by far more than
   WELD/TJUNCTION_EPS and the stitch left micro-triangle holes (the dominant
   residual failure class). Normals are now normalized; degenerate directions are
   skipped.
2. **Sub-EPS-area piece discard** (`split_convex_by_line`): split pieces with 2-D
   area ≤ EPS were dropped, but a seam-corner micro-area piece can still have
   ~1e-4-long edges — discarding it ripped a coverage hole that welding (1e-7)
   and T-junction healing (4e-7) cannot close. Both pieces are now always kept;
   true zero-extent debris is still pruned downstream (`len < 3` fans,
   `Tri::is_degenerate()`, sub-WELD_EPS welding).
3. **Inherited boundary micro-subdivisions** (new `chain_redundant_vertices`
   pass): every boolean bequeaths collinear T-junction-heal vertices to its
   result's face boundaries; re-triangulating such a face fans needle triangles
   along the chain ("deck of cards") whose altitudes straddle the stitch sliver
   filter — dropping a RUN of them rips an unhealable slit. The sharpest victim
   was seed 83894724544576: a **disjoint** difference (zero intersection
   geometry, a pure re-stitch of the accumulated body) still exploded. Face
   boundaries are now stripped of redundant chain vertices before triangulation —
   only vertices used by exactly two loops, with matching reversed neighbours,
   within TJUNCTION_EPS of the neighbour chord are removed (per-vertex and
   index-ordered, hence deterministic; rings always keep ≥ 3 vertices; vertices
   on 3+ loops — real topological junctions — are never touched).

| corpus | chains | pass rate | runs |
|---|---|---|---|
| standard N=200 | 200 | **100.0 % (200/200), all 3 runs byte-identical** | 3 |
| deep N=2000 | 2000 | **100.0 % (2000/2000), all 3 runs byte-identical** | 3 |

Ratchet: standard measured 99.5 → **100.0** (floor 97.5 → **98.0**); deep measured
99.5 → **100.0** (floor 97.5 → **98.0**). The retained 2-point headroom covers
cross-platform libm (sin/cos) differences and marginal chains legitimately
shifting under future kernel changes — at N=2000 that is a 40-chain margin.

All nine formerly-failing seeds (the list in the R5 section below) are
additionally pinned by `residual_level6_seeds_stay_fixed` in `fuzz_chains.rs`,
which replays each full chain in the default suite — a regression of any single
seed fails loudly even while the corpus floor would still pass. The last known
order-dependence OUTSIDE the boolean pipeline — `kernel-core Mesh::fill_holes`
walking boundary edges in HashSet order — was fixed the same day (W1, commit
e0b1673): boundary edges are now taken in triangle order (see the
`fill_holes` doc note in `crates/kernel-core/src/mesh/mod.rs` — the 2026-07-28
split turned `mesh.rs` into the `mesh/` directory).

## Measured 2026-06-10 — AFTER the R5 determinism fix (run-deterministic corpus)

R5 (run-to-run nondeterminism) is FIXED in the kernel-brep boolean pipeline. Three
surviving `HashMap`/`HashSet` iteration-order dependences were eliminated:

1. `booleans.rs cancel_coincident` — the coincident-facet accumulator was DRAINED in
   HashMap order, so the welded triangle-soup order (which decides region
   representatives → normal/surface/provenance tags, face order, and the next
   chained boolean's entire arrangement) was random per run. It now emits in
   first-insertion key order.
2. `booleans.rs recover_faces` — coplanar-region merging ran in `edge_map.values()`
   (HashMap-random) order; the union-find *partition* is order-independent but the
   surviving root ids are not, and regions were ordered by sorted root id — so
   result-face ORDER shuffled run to run. Regions are now ordered by their smallest
   member triangle index.
3. `curved_boolean.rs boundary_loops` — per-vertex successor lists were filled in
   HashSet-random order, so a pinch vertex split its loops differently run to run
   (mesh-level helpers; not on the fuzz path, fixed for completeness). Successor
   lists are now sorted.

Evidence (all in-repo): `tests/determinism.rs` rebuilds the flange recipe (L-profile
revolve ∪ rim-filleted boss → bore → 6 bolt drills) **40× in-process** and asserts
bit-identical topology counts and volume bits — it failed on the first repeat before
the fix. Shell-level: 10 runs of the full fuzz suite (N=200 + N=2000) produced a
**byte-identical report all 10 times** (modulo wall-clock lines), and 10 runs of the
determinism binary printed the identical snapshot.

| corpus | chains | pass rate | runs |
|---|---|---|---|
| standard N=200 | 200 | **99.5 % (199/200), all 10 runs identical** | 10 |
| deep N=2000 | 2000 | **99.5 % (1991/2000), all 10 runs identical** | 10 |

Ratchet: standard measured 99.0 → **99.5** (floor 97.0 → **97.5**); deep unchanged at
99.5 (floor 97.5). The retained 2-point headroom no longer covers in-run flake
(there is none); it covers cross-platform libm (sin/cos) differences and marginal
chains legitimately shifting under future kernel changes.

Residuals — the open Level-6 mop-up, a FIXED, fully-enumerated set of exactly 9
(SUPERSEDED by the mop-up section above: all nine now PASS and are pinned by
`residual_level6_seeds_stay_fixed`; the list is preserved here as the historical
diagnosis). The ordering fix changed *which* marginal chains fail: of the five seeds
published on 2026-06-09, 83894724543872 / 83894724544576 / 83894724544707 still
fail, while 83894724544390 and 83894724544779 now pass (the other four old members
went unpublished, so no per-seed comparison is possible for them):

```
seed=83894724543558 op=3 [difference(filleted cuboid)] → closed=false shells=2 χ=3
seed=83894724543872 op=2 [difference, sphere]          → closed=false shells=1 χ=1
seed=83894724544312 op=4 [union, sphere]               → closed=false shells=1 χ=1
seed=83894724544576 op=5 [difference, sphere disjoint] → closed=false shells=1 χ=1
seed=83894724544707 op=5 [union, sphere]               → closed=false shells=2 χ=3
seed=83894724544946 op=5 [intersection, holed extrude] → closed=false shells=1 χ=-1
seed=83894724544984 op=1 [intersection, cylinder]      → closed=false shells=1 χ=1
seed=83894724545208 op=6 [intersection(filleted)]      → closed=false shells=1 χ=1
seed=83894724545313 op=2 [union(filleted)]             → closed=false shells=1 χ=1
```

All are `closed=false manifold=false` stitch explosions; replay any via
`FUZZ_SEED=<seed> … replay_chain_from_env_seed`. Known order-dependence remaining
OUTSIDE the boolean pipeline: `kernel-core Mesh::fill_holes` walks boundary edges in
HashSet order (pinch-vertex cap splicing) — the import-repair path, not booleans.

## Measured 2026-06-09 (evening) — AFTER the loop-aware arrangement fix

The Level-6 fleet landed R1–R4 the same day (loop-aware boolean triangulation /
recovery / healing in booleans.rs, revolve orientation in build.rs, exact_volume
loop signs in validate.rs). Re-measured on the identical corpus:

| corpus | chains | pass rate | runs |
|---|---|---|---|
| standard N=200 | 200 | **99.0 / 99.5 / 99.0 %** | 3 |
| deep N=2000 | 2000 | **99.5 % (1991/2000)** | 1 |

**38.5 % → 99.5 %.** The Level-6 bar test (≥ 99 %) is UN-IGNORED and now runs the
deep corpus in the default suite (N=2000 so the residual one-chain flake cannot
straddle the threshold). Ratchet floors raised: standard ≥ **97.0 %**, deep ≥ **97.5 %**.

Residuals as recorded that evening (SUPERSEDED by the 2026-06-10 section above —
the seed list shifted under the determinism fix and R5 is now closed):
- **~0.5 % failure class**: 9/2000 chains still explode (`closed=false` stitch
  failures; sphere/extrude differences, one union, two filleted-cuboid ops — one of
  them on a DISJOINT difference). First seeds: 83894724543872 (op 2, sphere
  difference), 83894724544390, 83894724544576, 83894724544707, 83894724544779 —
  replay via `FUZZ_SEED=<seed> … replay_chain_from_env_seed --ignored --nocapture`.
- **R5 nondeterminism reduced, not eliminated**: fixed seeds still flip exactly one
  marginal chain between runs (99.0 ↔ 99.5), so at least one HashMap-order
  dependence survives the two that were fixed.

## Measured 2026-06-09 (pre-fix baseline — preserved for history)

Chain = 1 random base solid (cuboid / 8–32-seg cylinder / convex extrusion / 1–2-hole
extrusion / 16×12 sphere, ~40–70 mm) + 3–7 random booleans against smaller random
base solids at random translations (≈half overlapping; intersections bias to overlap
since a disjoint intersection legitimately empties the chain), occasionally with a
named-edge fillet applied to a fresh cuboid operand first. A chain passes if
`validate()` reports closed + manifold + genus ≥ 0 after **every** op (genus may
legitimately grow; panics are caught and counted as failures).

| corpus | chains | pass rate | runs |
|---|---|---|---|
| standard (`fuzz_200_feature_chains_hold_the_measured_pass_rate`) | 200 | **37.0–40.0 % (median 38.5 %)** | 8 |
| deep `#[ignore]`d (`fuzz_2000_feature_chains_deep_corpus`) | 2000 | **34.5–34.7 %** | 3 |

Ratchet floors in the test: standard ≥ **32.0 %** (lowest observed − 5), deep ≥
**29.5 %**. The `#[ignore]`d Level-6 bar test (≥ 99 %) stays ignored until Level 6
lands — today the kernel is ~60 points below that bar.

### Failure histogram by op kind (N=2000 deep corpus, failed/attempted)

| op kind | failed/attempted | failure rate |
|---|---|---|
| base solid construction | 0/2000 | 0.0 % |
| union | 457/1449 | **31.5 %** |
| difference | 339/1428 | 23.7 % |
| intersection | 343/1474 | 23.3 % |
| union (filleted-cuboid operand) | 56/365 | 15.3 % |
| difference (filleted-cuboid operand) | 65/393 | 16.5 % |
| intersection (filleted-cuboid operand) | 50/346 | 14.5 % |

351/2000 chains ended legitimately empty (disjoint intersection or all-consuming
difference) and count as passes of the steps they executed.

### What the failures look like

Every sampled failure is `closed=false manifold=false` — the boolean's stitch emits
open / non-manifold topology, frequently with impossible Euler characteristics.
First failing seeds (reproduce with the replay helper below):

```
seed=83894724543489 op=4 [intersection]  intersection extrude 8-gon r=6.6 h=8.5 overlapping → closed=false manifold=false genus=1 shells=1 χ=-1
seed=83894724543490 op=1 [difference]   difference extrude_with_holes 9-gon holes=2 overlapping → closed=false manifold=false genus=2 shells=3 χ=2
seed=83894724543492 op=2 [union]        union extrude_with_holes 5-gon holes=2 DISJOINT → closed=false manifold=false genus=2 shells=5 χ=6
seed=83894724543493 op=4 [difference]   difference extrude 10-gon overlapping → closed=false manifold=false genus=2 shells=1 χ=-3
seed=83894724543496 op=1 [intersection] intersection extrude_with_holes 10-gon holes=1 overlapping → closed=false manifold=false genus=1 shells=1 χ=0
```

Triage pointers (consistent with the BAR.md 2026-06-09 bisect findings):

- **R2 dominates**: `extrude_with_holes` operands appear in most early failures.
  Seed 83894724543492 is the sharpest diagnostic — a **disjoint** union (zero
  intersection geometry) against a holed extrusion still explodes, so the boolean's
  re-stitch of multi-loop faces is broken independently of intersection complexity.
- **Chained booleans on plain solids also fail** (~20–30 % per op): once a body has
  been cut, the next arrangement inherits its slivers/T-junctions.
- **Filleted-cuboid operands fail *less*** (≈15 % vs ≈25–30 %): the operands are
  fresh single-fillet cuboids — failure rate tracks the *accumulated* body
  complexity, not the operand's curvature.
- **The kernel is not run-deterministic**: 8 runs of the identical binary on fixed
  seeds scored 37.0–40.0 %. Boolean results depend on std `HashMap`/`HashSet`
  iteration order (process-random hasher), so the same chain can pass in one run and
  fail in the next. Determinism is itself a Level-9 quality bar — fix alongside the
  stitcher (e.g. ordered maps or seeded hashers in `booleans.rs`).
- Base solids are never born invalid here (0/2000) — `revolve` (R1, invalid-at-birth
  L-profiles) is **not** in this generator's base vocabulary yet; add it once R1 is
  fixed, or as a deliberately-failing bucket.

## Reproducing

```bash
# standard gate (also runs in `cargo test --workspace --release`):
cargo test -p kernel-brep --release --test fuzz_chains -- --nocapture

# deep 2000-chain corpus:
cargo test -p kernel-brep --release --test fuzz_chains -- fuzz_2000 --ignored --nocapture

# Level-9 10 000-chain corpus (~2 min in release):
cargo test -p kernel-brep --release --test fuzz_chains -- fuzz_10000 --ignored --nocapture

# replay one chain verbosely by seed (prints every op recipe and verdict):
FUZZ_SEED=83894724543492 cargo test -p kernel-brep --release --test fuzz_chains \
    -- replay_chain_from_env_seed --ignored --nocapture
```

`replay(seed)` is also callable from any debugger/test. Chain `i` of a corpus uses
seed `0x4C4D43414400 + i` (= 83894724543488 + i), so the deep corpus is a strict
superset of the standard one. Recipes are a function of (seed, generator version): if the generator in
`fuzz_chains.rs` changes, re-measure and update this file and the floors together.

## History

| date | corpus | pass rate | note |
|---|---|---|---|
| 2026-06-09 | N=200 ×8 runs | 37.0–40.0 % (median 38.5) | first baseline; floor set to 32.0 |
| 2026-06-09 | N=2000 ×3 runs | 34.5–34.7 % | floor 29.5; union worst op at 31.5 % |
| 2026-06-09 | N=200 ×3 / N=2000 ×1 | 99.0–99.5 % / 99.5 % | post R1–R4; floors 97.0 / 97.5; R5 flake ±1 chain |
| 2026-06-10 | N=200 ×10 / N=2000 ×10 | 99.5 % / 99.5 %, byte-identical | post R5 determinism fix; floor 97.0 → 97.5 (standard); same 9 deep seeds every run |
| 2026-06-10 | N=200 ×3 / N=2000 ×3 | 100.0 % / 100.0 %, byte-identical | Level-6 mop-up: 9/9 residual seeds fixed (unit split normals, no area discard, boundary-chain strip); floors → 98.0; seeds pinned in-tree |
| 2026-06-10 | N=10 000 ×2 | 99.98 % (9998/10000), byte-identical | Level-9 evidence corpus (≥99.5 % bar met, 48-chain margin); 2 residual seeds listed above, booleans untouched (perf-only wave) |
| 2026-07-30 | N=200 / N=2000 ×2 / N=10 000 ×2 | 100.0 % / 100.0 % / **100.00 %**, byte-identical | both Level-9 residual seeds fixed (micro-edge T-heal guard `len2 < EPS` → sub-weld only; sub-`TJUNCTION_EPS` duplicate-vertex merge at stitch entry); Level-9 floor → exact 10 000/10 000 pin; seeds pinned in-suite (`residual_level9_seeds_stay_fixed`) |
