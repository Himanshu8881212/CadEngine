// Copyright (c) LMCAD. Licensed under the MIT License.

//! Exact constructive-solid-geometry booleans on **planar-faced (polyhedral)**
//! B-rep [`Solid`]s — union, difference and intersection.
//!
//! The algorithm is the classical mesh-arrangement boolean specialised to the
//! exact B-rep representation, and it is **general**: it works for any valid
//! closed planar solid (cuboid, [`extrude`](crate::extrude)d prism, any convex
//! or non-convex polyhedron), never for one specific shape.
//!
//! ## Pipeline
//! 1. **Triangulate** both operands. Each B-rep face is planar, so ear-clipping
//!    is exact and every emitted triangle lies in its face's supporting plane.
//!    A face with inner (hole) loops is bridged into one simple ring first, so a
//!    hole is a true opening, never silently filled. Boundaries are first stripped
//!    of redundant collinear micro-subdivisions (T-junction chains a previous
//!    boolean healed into the faces) so chained ops do not accumulate needle
//!    fans — see [`chain_redundant_vertices`]. The triangle is the unit of
//!    work; because the input faces are planar this loses no geometry (re-merging
//!    coplanar triangles afterwards restores faces).
//! 2. **Co-refine.** Every triangle of A is split along its intersection with
//!    every triangle of B (plane–plane SSI gives a line; clipped to both triangle
//!    polygons it gives a segment), and vice-versa. After this pass no triangle
//!    of A straddles the surface of B, so each fragment is wholly inside or
//!    wholly outside the other solid.
//! 3. **Classify** every fragment by its centroid against the *other* solid using
//!    a robust ray-cast point-in-polyhedron test on the other solid's triangles.
//! 4. **Select & orient** fragments per the boolean rule (union keeps the parts
//!    of each surface outside the other; intersection keeps the inside parts;
//!    difference keeps A-outside plus B-inside with B's facets flipped).
//! 5. **Stitch** the kept triangles back into a closed half-edge [`Solid`] via
//!    [`Solid::from_faces`], welding coincident vertices to a tolerance so that
//!    twin half-edges pair up. Sub-tolerance slivers are dropped, T-junctions are
//!    healed by vertex insertion, and coplanar adjacent triangles are merged back
//!    into maximal planar faces — each merged face is verified to ear-clip to
//!    exactly its triangles' area (else it is re-expanded), so chained booleans
//!    (a second hole into an already-drilled face, a cut crossing a bored wall)
//!    stay valid instead of exploding.
//!
//! ## Scope and the curved-seam contract
//! The **arrangement** itself is planar: a curved input face enters as its
//! tessellated planar facets, carrying its analytic [`Surface`] tag by value.
//! Three post-passes then restore curved exactness where it is well-defined:
//!
//! - **Cut fragments keep their analytic tags** ([`recover_faces`]): a clipped
//!   bore band is still `Surface::Cylinder`, so adaptive tessellation, section
//!   queries and `exact_volume` see the true surface through the cut.
//! - **Cut-seam vertices land on the TRUE surface–surface intersection**
//!   ([`snap_seam_vertices`]): a seam vertex on two or three distinct analytic
//!   surfaces is Newton-projected onto their exact intersection via the
//!   [`crate::ssi`] machinery, to ≤ 1e-11 per surface — not left on the ~1e-2
//!   facet chords the raw arrangement produces. Since W5 this covers seams that
//!   WARP their incident facets — oblique cuts, cut rims ending mid-facet, and
//!   plane-less quadric∩quadric seams (cylinder∪cylinder) — because warped
//!   curved-tagged faces are no longer ear-clipped in a projection plane (where
//!   they fold) but in their surface's PARAMETER SPACE ([`SurfaceChart`]; W3's
//!   planarity contract existed precisely because the projection-plane clip of
//!   warped polygons exploded chained booleans). Snapping stays fuzz-guarded by
//!   per-face chart guards ([`chart_snap_safe`]) and an all-or-nothing per-seam
//!   coherence rule; the conservative W3 move budgets remain in force for
//!   sphere/cone/torus vertices, whose 10–20× larger facet sagitta measurably
//!   breaks chains when freed (deep N=2000 corpus — `ROBUSTNESS.md` W5). Seam
//!   EDGES between snapped vertices remain chords of the true curve (the
//!   polyline is vertex-exact, not arc-exact).
//! - **Plane∩quadric cut seams carry exact [`Curve`] tags**
//!   ([`attach_seam_curves`]): circles / ellipses / generator lines from
//!   [`Surface::plane_section`], on snapped cut seams as well as surviving
//!   construction rims — since W5 including the oblique-cut ELLIPSE, whose seam
//!   vertices now land on the section curve. An edge is tagged only when both
//!   endpoints lie ON the section curve, so a chord-accurate seam (a
//!   coherence-vetoed snap) is honestly left untagged, and a quadric∩quadric
//!   seam has no conic closed form and remains an untagged vertex-exact
//!   polyline.
//!
//! Volume: for purely planar solids the result is exact to floating-point (e.g.
//! the union volume of two overlapping boxes matches inclusion–exclusion to
//! ~1e-9). With curved faces, `exact_volume`'s analytic bulge corrections are
//! machine-exact whenever each cut facet remains a θ-rectangular patch of its
//! surface — ⟂ and axis-parallel cylinder cuts qualify, and seam snapping keeps
//! them exact through chained cuts (the keyway identity closes to round-off);
//! oblique conic cuts and quadric∩quadric seams remain facet-level
//! approximations (the corrections assume chord facets, which a snapped warped
//! facet is not — measured honestly in
//! `quadric_quadric_union_volume_stays_facet_level_and_beats_faceted`) —
//! stated, never silent.

use std::collections::{HashMap, HashSet};

use kernel_core::math::DVec3;
use kernel_core::{orient2d, orient3d};

use crate::geom::{perp_basis, Curve, Surface, SurfaceChart};
use crate::topo::{FaceInput, FaceName, FaceSource, Solid};

use crate::tol::{EPS, TJUNCTION_EPS, WELD_EPS};

/// Which boolean to evaluate.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Op {
	Union,
	Difference,
	Intersection,
}

// --- Intra-arrangement parallelism (structurally bit-deterministic) -----------
//
// Three pipeline stages are pure per-item maps — each item's output is a
// deterministic function of (that item, read-only shared input), computed with
// the identical float expression sequence as the sequential loop, with no
// shared accumulators and no iteration-order-dependent containers:
//
//   * `co_refine` — per SUBJECT TRIANGLE: collect cut segments from the
//     AABB-pruned, read-only cutter list (grid candidates are sorted + deduped,
//     so candidate ORDER is deterministic) and split the triangle;
//   * `classify_select` — per FRAGMENT: centroid ray-cast against the other
//     operand's read-only triangle list, then the keep/flip decision.
//
// `triangulate_solid` is a pure per-face map too, but stays SEQUENTIAL by
// measurement, not by necessity: it is 1–3% of boolean time and threading it
// measurably LOST (flange chain, 2.5 k-face operand: ~1.7 ms sequential vs
// ~2.1 ms threaded — the per-face work is allocation-bound, which scoped
// workers only contend over). Honest scheduling: threads go where the profile
// says the time is (classification ~50–65%, co-refinement ~5–25%).
//
// Each stage runs through [`kernel_core::par::par_flat_map_chunks`]: outputs are
// produced per contiguous chunk and concatenated in ascending chunk order, so
// the result is byte-identical to the sequential loop BY CONSTRUCTION — thread
// scheduling decides only WHEN a chunk is computed, never what it contains or
// where it lands (the R5 bit-determinism contract of `docs/NUMERICS.md` is
// preserved structurally, and pinned by `tests/threading_parity.rs` +
// `tests/determinism.rs`). Everything downstream of classification — welding,
// coincident-facet cancellation, face recovery, T-junction healing, seam
// snapping (`stitch`), and `attach_seam_curves` — is ordered mutation of shared
// state and deliberately stays sequential.
//
// Control surface: `LMCAD_BREP_THREADS` — unset or `0` ⇒ available parallelism
// (default ON), `1` ⇒ sequential, `N` ⇒ N workers. Read once per [`boolean`]
// invocation and plumbed down as a parameter, so tests can flip the env var
// between calls without racing an in-flight arrangement. A boolean already
// running ON a `kernel_core::par` worker thread (pose-parallel `sweep_check`,
// `overlap_volume_many`) stays sequential regardless of the env var — the
// coarse grain owns the cores there, and 8×8 nested scoped threads would only
// oversubscribe. All of this is scheduling; none of it can touch output bytes.

/// Upper bound on items per work chunk (a chunk is one scheduling quantum;
/// small enough that a hot spot — one triangle splitting into hundreds of
/// fragments — still load-balances across workers).
const PAR_CHUNK: usize = 32;

/// Co-refinement's item-count engage arm: below this many subject triangles a
/// stage has ≲ 0.5 ms of uniform work, and spawn/join of the scoped workers
/// (~0.1 ms measured on the 8-core M-class dev machine) plus allocator
/// contention eats the win. Measured at the boundary (cylinder∖cuboid sweep,
/// tests note in `tests/threading_parity.rs`): segs=64 ≈ 390 subject tris stays
/// sequential at 1.49 ms both ways; segs=128 ≈ 780 tris engages at parity
/// (4.68 ms threaded vs 4.70 ms sequential); the win grows with size (flange
/// op 2 co-refine 16.9 → 7.7 ms). Below the cutoff both env settings run the
/// byte-identical sequential schedule at identical cost (segs=8: 99 µs both).
const PAR_CUTOFF: usize = 512;

/// Classification cost is O(items × |other|) — per-ITEM cost spans three orders
/// of magnitude (an `other` of 90 tris vs 10 000) — so its engage decision and
/// chunk length are WORK-based, in units of one fragment-vs-triangle scan
/// (measured ~8 ns: flange op 3 classify = 3.2 M units in 24.9 ms sequential).
/// [`CLASSIFY_WORK_CUTOFF`] = 200 k units ≈ 1.6 ms of stage work — engage-
/// boundary cases measure at parity (segs 96–128 sweep, ±1%), and above it the
/// stage scales (flange classify 24.9 → 5.4 ms, 99 → 20 ms on the union op).
/// [`CLASSIFY_CHUNK_WORK`] ≈ 0.13 ms per chunk keeps a handful of very
/// expensive fragments load-balanced. Chunk length NEVER affects output bytes
/// (see `par_flat_map_chunks`) — these are scheduling economies only.
const CLASSIFY_WORK_CUTOFF: usize = 200_000;
const CLASSIFY_CHUNK_WORK: usize = 16_000;

/// Monotonic count of items actually dispatched to the THREADED schedule (the
/// work-engagement receipt for `tests/threading_parity.rs`: it proves the
/// parallel path genuinely engaged, so a trivial always-sequential
/// implementation cannot pass the parity gates). Telemetry only — never read by
/// the pipeline itself, so it cannot influence geometry.
static PAR_ITEMS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// Total items the boolean pipeline has processed on the threaded schedule in
/// this process (see [`PAR_ITEMS`]). Tests snapshot it before/after a call.
pub fn par_items_processed() -> u64 {
	PAR_ITEMS.load(std::sync::atomic::Ordering::Relaxed)
}

/// Worker count for THIS boolean invocation: `LMCAD_BREP_THREADS` unset or `0`
/// ⇒ available parallelism, `1` ⇒ sequential, `N` ⇒ `N`. Unparsable values fall
/// back to the default (never a panic mid-arrangement). Sequential regardless
/// on a `kernel_core::par` worker thread — see the control-surface note above.
fn brep_workers() -> usize {
	if kernel_core::par::in_worker_thread() {
		return 1;
	}
	let default = || std::thread::available_parallelism().map(|n| n.get()).unwrap_or(1);
	match std::env::var("LMCAD_BREP_THREADS") {
		Ok(s) => match s.trim().parse::<usize>() {
			Ok(0) | Err(_) => default(),
			Ok(n) => n,
		},
		Err(_) => default(),
	}
}

/// Run one pure per-item stage over `items`: threaded in `chunk_len`-sized
/// chunks when `engage` (the stage's measured is-it-worth-spawning predicate)
/// and `workers > 1`, else the identical chunk loop sequentially — one
/// implementation, two schedules (see the module section above for the
/// bit-determinism argument; neither `workers` nor `chunk_len` can affect
/// output bytes).
fn stage_flat_map<T: Sync, R: Send>(
	workers: usize,
	items: &[T],
	chunk_len: usize,
	engage: bool,
	f: impl Fn(&[T]) -> Vec<R> + Sync,
) -> Vec<R> {
	let chunk_len = chunk_len.max(1);
	let w = if engage { workers } else { 1 };
	if w > 1 && items.len() > chunk_len {
		// Threaded schedule genuinely dispatches ≥ 2 chunks: record the receipt.
		PAR_ITEMS.fetch_add(items.len() as u64, std::sync::atomic::Ordering::Relaxed);
	}
	kernel_core::par::par_flat_map_chunks(w, items, chunk_len, f)
}

/// A triangle carrying the supporting plane it came from (so coplanar fragments
/// can later be merged back into a maximal planar face).
#[derive(Clone, Copy, Debug)]
struct Tri {
	v: [DVec3; 3],
	/// Unit outward normal of the source face (orientation reference).
	normal: DVec3,
	/// Persistent name of the operand face this triangle (and its fragments) came
	/// from — propagated through co-refinement so result faces can be re-identified.
	source: FaceName,
	/// The analytic surface of the operand face this triangle tessellates, carried
	/// BY VALUE (always in the operands' original frame — the recentred arrangement
	/// frame never reads it). A name-keyed surface lookup is ambiguous in a CHAINED
	/// boolean: an operand that is itself a boolean result carries `OperandA/B` face
	/// names from ITS OWN operands, which collide with the other operand's names —
	/// that mis-tagged e.g. a first bore's wall with the second bore's cylinder and
	/// made result volumes flake run to run.
	surface: Surface,
}

impl Tri {
	fn centroid(&self) -> DVec3 {
		(self.v[0] + self.v[1] + self.v[2]) / 3.0
	}

	/// Twice the area vector (length = 2·area, direction = geometric normal).
	fn area_vec(&self) -> DVec3 {
		(self.v[1] - self.v[0]).cross(self.v[2] - self.v[0])
	}

	fn area(&self) -> f64 {
		self.area_vec().length() * 0.5
	}

	fn is_degenerate(&self) -> bool {
		self.area() <= EPS * EPS
	}
}

/// The exact union `A ∪ B` of two planar-faced solids.
pub fn union(a: &Solid, b: &Solid) -> Solid {
	boolean(a, b, Op::Union)
}

/// The exact difference `A − B` of two planar-faced solids (material of `a` with
/// the material of `b` removed).
pub fn difference(a: &Solid, b: &Solid) -> Solid {
	boolean(a, b, Op::Difference)
}

/// The exact intersection `A ∩ B` of two planar-faced solids (the material common
/// to both).
pub fn intersection(a: &Solid, b: &Solid) -> Solid {
	boolean(a, b, Op::Intersection)
}

// --- Driver ------------------------------------------------------------------

fn boolean(a: &Solid, b: &Solid, op: Op) -> Solid {
	// Worker budget for the pure-map stages, read ONCE per invocation (see the
	// parallelism section above; `LMCAD_BREP_THREADS=1` is the exact legacy
	// sequential schedule — same code, trivial iteration).
	let workers = brep_workers();
	let mut tris_a = triangulate_solid(a, FaceSource::OperandA);
	let mut tris_b = triangulate_solid(b, FaceSource::OperandB);
	if tris_a.is_empty() || tris_b.is_empty() {
		// One operand is empty: union/difference is A, intersection is empty.
		return match op {
			Op::Union | Op::Difference => a.clone(),
			Op::Intersection => Solid::default(),
		};
	}

	// Far from the origin the arrangement's fixed *absolute* tolerances (EPS,
	// WELD_EPS) fall below the f64 ulp (ulp ≈ 1e-8 at 1e8) and coincidence tests
	// collapse. Re-centre the operands so the arrangement runs in full precision,
	// then shift the kept fragments back before stitching. This only engages in the
	// far field: in place (`ZERO`) the translate is an exact no-op, so near-origin
	// results — including EPS-sensitive off-axis geometry — are byte-identical to the
	// un-centred path. The un-centred arrangement is reliable to ~1e7. Normals are
	// translation-invariant, so only vertices move.
	let bbox_center = tris_bbox_center(&tris_a, &tris_b);
	let center = if bbox_center.abs().max_element() > 1e7 { bbox_center } else { DVec3::ZERO };
	translate_tris(&mut tris_a, -center);
	translate_tris(&mut tris_b, -center);

	// Co-refine each operand against the other so no fragment straddles the other
	// solid's surface.
	let frags_a = co_refine(&tris_a, &tris_b, workers);
	let frags_b = co_refine(&tris_b, &tris_a, workers);

	// Classify + select. Each kept fragment already carries its operand-face name.
	// A-side kept fragments precede B-side ones, exactly as the sequential
	// appends always did.
	let mut kept: Vec<Tri> = classify_select(&frags_a, &tris_b, op, false, workers);
	kept.extend(classify_select(&frags_b, &tris_a, op, true, workers));

	// Shift the kept fragments back to the operands' original location, then stitch
	// in that frame so the output vertices and their supporting surfaces are derived
	// consistently (stitching in the centred frame and translating the finished
	// solid would round vertices and surface anchors apart and crack the tessellation).
	// Each fragment carries its operand face's analytic surface BY VALUE (in this
	// original frame), so an uncut curved facet keeps its Surface::Cylinder/Sphere tag
	// on the result without any name-keyed lookup.
	translate_tris(&mut kept, center);
	let mut solid = stitch(&kept);
	attach_seam_curves(&mut solid);
	solid
}

/// Tag each edge that bounds a planar face and a curved face with the exact analytic
/// intersection [`Curve`] of that plane with the curved surface (a cylinder/sphere/cone
/// cut ⟂ its axis → a [`Curve::Circle`], obliquely → an [`Curve::Ellipse`], an
/// axis-parallel cut → the matching generator [`Curve::Line`], …), via
/// [`Surface::plane_section`]. So where a boolean CUTS a curved surface, the seam carries
/// exact geometry (and exports as a CIRCLE/ELLIPSE in STEP) instead of a bare polyline.
/// SNAPPED cut seams qualify, not only surviving construction rims:
/// [`snap_seam_vertices`] has already placed their vertices on the true intersection, so
/// the on-curve endpoint test below accepts them. A seam whose snap was rejected (the
/// planarity contract) keeps chord vertices and is honestly left untagged, and a
/// quadric∩quadric seam — e.g. cylinder∪cylinder — has no conic closed form at all.
///
/// The faces' operand surfaces are read from the result face tags, which [`recover_faces`]
/// sets from the surfaces carried on the fragments themselves — exact even in chained
/// booleans (a name-keyed lookup collided between an operand-that-is-a-boolean's carried
/// names and the other operand's names). An edge is tagged only with the UNIQUE section
/// curve through both its endpoints; zero matches (a legacy chord-bound seam, a skipped
/// snap) or several (tangent degeneracies) leave it untagged — never a guess.
fn attach_seam_curves(solid: &mut Solid) {
	let orig = |f: crate::topo::FaceId| -> Option<Surface> { Some(solid.face(f).surface) };
	let mut updates: Vec<(crate::topo::EdgeId, Curve)> = Vec::new();
	for e in solid.edges() {
		let he = solid.edge(e).half_edge;
		let twin = match solid.half_edge(he).twin {
			Some(t) => t,
			None => continue,
		};
		let (sa, sb) = match (orig(solid.half_edge(he).face), orig(solid.half_edge(twin).face)) {
			(Some(a), Some(b)) => (a, b),
			_ => continue,
		};
		let (po, pn, curved) = match (sa, sb) {
			(Surface::Plane { origin, normal }, c) if !matches!(c, Surface::Plane { .. }) => (origin, normal, c),
			(c, Surface::Plane { origin, normal }) if !matches!(c, Surface::Plane { .. }) => (origin, normal, c),
			_ => continue,
		};
		let sections = curved.plane_section(po, pn);
		// Both endpoints must lie on the SAME section curve, and on exactly one of
		// them, so an unrelated plane/curved adjacency — or an ambiguous tangent
		// section — is never mis-tagged.
		let va = solid.position(solid.half_edge(he).origin);
		let vb = solid.position(solid.half_edge(solid.half_edge(he).next).origin);
		let mut matching = sections.iter().filter(|c| point_on_curve(c, va) && point_on_curve(c, vb));
		if let (Some(c), None) = (matching.next(), matching.next()) {
			updates.push((e, *c));
		}
	}
	for (e, c) in updates {
		solid.set_edge_curve_by_id(e, c);
	}
}

/// Whether `p` lies on `curve` within a small tolerance.
fn point_on_curve(curve: &Curve, p: DVec3) -> bool {
	match *curve {
		Curve::Circle { center, normal, radius } => {
			let d = p - center;
			let h = d.dot(normal);
			let radial = (d - normal * h).length();
			h.abs() < 1e-6 && (radial - radius).abs() < 1e-6
		}
		Curve::Ellipse { center, normal, u, a, b } => {
			// In the ellipse plane AND satisfying x²/a² + y²/b² = 1 (not merely coplanar).
			let d = p - center;
			if d.dot(normal).abs() >= 1e-6 || a <= 0.0 || b <= 0.0 {
				return false;
			}
			let v = normal.cross(u).normalize_or_zero();
			let (x, y) = (d.dot(u) / a, d.dot(v) / b);
			(x * x + y * y - 1.0).abs() < 1e-6
		}
		Curve::Line { origin, dir } => (p - origin).cross(dir.normalize_or_zero()).length() < 1e-6,
		_ => true,
	}
}

/// Midpoint of the combined bounding box of every triangle vertex in `a` and `b`.
fn tris_bbox_center(a: &[Tri], b: &[Tri]) -> DVec3 {
	let mut lo = DVec3::splat(f64::INFINITY);
	let mut hi = DVec3::splat(f64::NEG_INFINITY);
	for t in a.iter().chain(b) {
		for v in t.v {
			lo = lo.min(v);
			hi = hi.max(v);
		}
	}
	(lo + hi) * 0.5
}

/// Shift every triangle vertex by `off` (normals are translation-invariant).
fn translate_tris(tris: &mut [Tri], off: DVec3) {
	for t in tris.iter_mut() {
		for v in t.v.iter_mut() {
			*v += off;
		}
	}
}

// --- Step 1: triangulation ---------------------------------------------------

/// Distance from `p` to the segment `a..b` (clamped to the endpoints).
fn dist_point_segment(p: DVec3, a: DVec3, b: DVec3) -> f64 {
	let ab = b - a;
	let len2 = ab.length_squared();
	if len2 <= EPS * EPS {
		return (p - a).length();
	}
	let t = ((p - a).dot(ab) / len2).clamp(0.0, 1.0);
	(p - a - ab * t).length()
}

/// Vertices of `s` that are redundant micro-subdivisions of a face-boundary chain:
/// used by exactly TWO loops (a plain interior subdivision of one shared edge, with
/// matching reversed neighbours on both sides) and within [`TJUNCTION_EPS`] of the
/// segment joining those neighbours. Booleans bequeath such chains to their results
/// (every T-junction heal and weld inserts collinear boundary vertices), and a later
/// triangulation of those faces fans NEEDLE triangles along the chain whose
/// altitudes straddle the stitch sliver filter — dropping a run of them rips an
/// unhealable slit (the disjoint-difference fuzz failure), while near-coincident
/// chain steps make the two operands disagree about seam corners at the ~1e-6 scale
/// (the nano-hole fuzz failures). Removing the chain vertex from BOTH loops is
/// exact to the same tolerance the healer already moves geometry by, and keeps the
/// two sides' triangulations consistent because the decision is per-vertex, not
/// per-face. Vertices on 3+ loops (real topological junctions) are never touched.
fn chain_redundant_vertices(s: &Solid) -> Vec<bool> {
	// Every boundary ring (outer + inner) of every face, as vertex-id cycles.
	let mut rings: Vec<Vec<u32>> = Vec::new();
	for f in s.faces() {
		let face = s.face(f);
		for &lid in std::iter::once(&face.outer).chain(face.inner.iter()) {
			rings.push(s.loop_half_edges(lid).into_iter().map(|he| s.half_edge(he).origin.0).collect());
		}
	}
	let pos: Vec<DVec3> = (0..s.vertex_count() as u32).map(|v| s.position(crate::topo::VertexId(v))).collect();
	chain_redundant_in_rings(&rings, &pos)
}

/// Core of [`chain_redundant_vertices`], over explicit boundary `rings` (vertex-id
/// cycles into `pos`). Shared between operand triangulation (rings from a [`Solid`]'s
/// loops) and [`stitch`]'s output cleanup (rings from the result's [`FaceInput`]
/// boundaries), so a boolean result is born without the chain micro-subdivisions the
/// next boolean would strip anyway.
fn chain_redundant_in_rings(rings: &[Vec<u32>], pos: &[DVec3]) -> Vec<bool> {
	let nv = pos.len();
	let mut removed = vec![false; nv];
	loop {
		// Per-vertex usages over the live (not-yet-removed) rings: (ring, prev, next).
		// Indexed by vertex id (not hashed) so the scan order — and hence which vertex
		// of a budget-limited ring wins — is deterministic.
		let mut usage: Vec<Vec<(usize, u32, u32)>> = vec![Vec::new(); nv];
		let mut live_len = vec![0usize; rings.len()];
		for (ri, ring) in rings.iter().enumerate() {
			let live: Vec<u32> = ring.iter().copied().filter(|&v| !removed[v as usize]).collect();
			let n = live.len();
			live_len[ri] = n;
			if n < 3 {
				continue;
			}
			for (i, &v) in live.iter().enumerate() {
				usage[v as usize].push((ri, live[(i + n - 1) % n], live[(i + 1) % n]));
			}
		}
		// A ring must keep ≥ 3 vertices; spend at most (len − 3) removals per round.
		let mut budget: Vec<usize> = live_len.iter().map(|&l| l.saturating_sub(3)).collect();
		let mut changed = false;
		for v in 0..nv as u32 {
			if removed[v as usize] || usage[v as usize].len() != 2 {
				continue;
			}
			let ((r1, p1, n1), (r2, p2, n2)) = (usage[v as usize][0], usage[v as usize][1]);
			// Exactly two loops, traversing the same neighbour pair in opposite
			// directions — the signature of a mid-edge subdivision vertex. Defer to the
			// next round if a neighbour was itself removed this round, so the
			// collinearity test below never reads a stale segment endpoint.
			if r1 == r2 || p1 != n2 || n1 != p2 || p1 == n1 || p1 == v || n1 == v {
				continue;
			}
			if budget[r1] == 0 || budget[r2] == 0 || removed[p1 as usize] || removed[n1 as usize] {
				continue;
			}
			let (pv, pa, pb) = (pos[v as usize], pos[p1 as usize], pos[n1 as usize]);
			if dist_point_segment(pv, pa, pb) > TJUNCTION_EPS {
				continue;
			}
			removed[v as usize] = true;
			budget[r1] -= 1;
			budget[r2] -= 1;
			changed = true;
		}
		if !changed {
			return removed;
		}
	}
}

/// Ear-clip every face of `s` into triangles in its supporting plane. Planar
/// faces triangulate exactly; the outward normal is taken from the topological
/// winding so the orientation is reliable regardless of the surface tag sign.
///
/// A face carrying inner (hole) loops is triangulated **loop-aware**: each hole is
/// bridged into the outer ring (the same algorithm as the multi-loop tessellator)
/// and the merged ring is ear-clipped, so the hole is a true opening. Without this,
/// a holed operand face (an `extrude_with_holes` cap, a mirrored pocket) was
/// triangulated by its outer loop alone — the hole was silently FILLED, the soup
/// double-covered the hole's rim against the hole's wall facets, and any boolean
/// touching such a face exploded (R2).
///
/// Boundaries are first stripped of redundant collinear chain vertices (see
/// [`chain_redundant_vertices`]) so a chained boolean does not inherit the previous
/// ops' T-junction micro-subdivisions — the residual Level-6 stitch-explosion source.
/// Sequential by MEASUREMENT, not necessity: this is a pure per-face map (each
/// face reads only the immutable solid and `drop_v`), but it is 1–3% of boolean
/// time and allocation-bound — threading it measurably lost (see the module's
/// parallelism section), so it keeps the plain loop.
fn triangulate_solid(s: &Solid, operand: FaceSource) -> Vec<Tri> {
	let drop_v = chain_redundant_vertices(s);
	let live_positions = |ids: &[crate::topo::VertexId]| -> Vec<DVec3> {
		ids.iter().filter(|v| !drop_v[v.0 as usize]).map(|&v| s.position(v)).collect()
	};
	let mut out = Vec::new();
	for f in s.faces() {
		let poly = live_positions(&s.face_vertices(f));
		if poly.len() < 3 {
			continue;
		}
		let normal = newell_normal(&poly);
		if normal.length_squared() < EPS {
			continue;
		}
		// Name every triangle by the operand face it tessellates. If this operand is
		// itself a boolean result, CARRY its existing face name (so identity survives
		// nested booleans — a face from A stays an A-face through `(A∪B)−C`). A
		// primitive's own face names are re-tagged by THIS operand instead, so a
		// direct primitive name never leaks into boolean-result provenance.
		let source = match s.face_name(f) {
			Some(name) if name.operand != FaceSource::Primitive => name,
			_ => FaceName { operand, source_face: f.0 },
		};
		let surface = s.face(f).surface;
		let inner = &s.face(f).inner;
		if inner.is_empty() {
			ear_clip(&poly, normal, source, surface, &mut out);
		} else {
			let holes: Vec<Vec<DVec3>> = inner
				.iter()
				.map(|&lid| {
					let ids: Vec<crate::topo::VertexId> =
						s.loop_half_edges(lid).into_iter().map(|he| s.half_edge(he).origin).collect();
					live_positions(&ids)
				})
				.collect();
			ear_clip_with_holes(&poly, &holes, normal, source, surface, &mut out);
		}
	}
	out
}

/// Ear-clip a planar face with inner hole loops into [`Tri`]s: project to the face
/// plane, orient the outer ring CCW and each hole CW, bridge every hole into the
/// outer ring ([`crate::tessellate::bridge_hole_into`] — the proven multi-loop washer
/// path), then ear-clip the merged simple ring. Emitted triangles keep the face's
/// outward `normal` winding, like [`ear_clip`].
fn ear_clip_with_holes(outer3d: &[DVec3], holes3d: &[Vec<DVec3>], normal: DVec3, source: FaceName, surface: Surface, out: &mut Vec<Tri>) {
	let mut all: Vec<DVec3> = outer3d.to_vec();
	for h in holes3d {
		all.extend_from_slice(h);
	}
	let (p2, chart) = face_clip_p2(&all, outer3d, normal, &surface);

	let mut outer: Vec<usize> = (0..outer3d.len()).collect();
	if signed_area_2d(&p2, &outer) < 0.0 {
		outer.reverse(); // outer CCW in the (u, v) projection = wound about `normal`
	}
	let mut holes: Vec<Vec<usize>> = Vec::new();
	let mut start = outer3d.len();
	for h in holes3d {
		let mut ring: Vec<usize> = (start..start + h.len()).collect();
		if signed_area_2d(&p2, &ring) > 0.0 {
			ring.reverse(); // holes CW (opposite the outer)
		}
		holes.push(ring);
		start += h.len();
	}
	// Bridge the right-most holes first so their bridges don't cross later ones.
	holes.sort_by(|a, b| {
		crate::tessellate::ring_max_x(&p2, b)
			.partial_cmp(&crate::tessellate::ring_max_x(&p2, a))
			.unwrap_or(std::cmp::Ordering::Equal)
	});
	let all_rings = holes.clone();
	for hole in &holes {
		crate::tessellate::bridge_hole_into(&p2, &mut outer, hole, &all_rings);
	}
	// Ear-clip the merged INDEX ring (bridge vertices repeat as the same index, so
	// they never block each other's ears — see `ear_clip_ring_tris`).
	ear_clip_ring_tris(&all, &p2, outer, normal, source, surface, chart, out);
}

/// Area-weighted polygon normal following the winding (Newell's method).
fn newell_normal(poly: &[DVec3]) -> DVec3 {
	let mut n = DVec3::ZERO;
	let len = poly.len();
	for i in 0..len {
		let c = poly[i];
		let d = poly[(i + 1) % len];
		n.x += (c.y - d.y) * (c.z + d.z);
		n.y += (c.z - d.z) * (c.x + d.x);
		n.z += (c.x - d.x) * (c.y + d.y);
	}
	n.normalize_or_zero()
}

/// Ear-clip a face polygon into triangles, each carrying `normal` and `source`.
/// A planar polygon clips in its projection plane (exact); a WARPED curved-tagged
/// polygon clips in its surface's parameter space (see [`face_clip_p2`]).
fn ear_clip(poly: &[DVec3], normal: DVec3, source: FaceName, surface: Surface, out: &mut Vec<Tri>) {
	let (p2, chart) = face_clip_p2(poly, poly, normal, &surface);
	ear_clip_ring_tris(poly, &p2, (0..poly.len()).collect(), normal, source, surface, chart, out);
}

/// 2-D clip coordinates for the `pts` of one face (outer boundary first, then any
/// hole vertices), plus whether they are CHART coordinates: the PARAMETER-SPACE
/// chart of the face's analytic surface when the boundary is measurably warped
/// off its plane (a seam-snapped curved face — projection-plane ear-clipping can
/// fold there, see [`crate::geom::CURVED_WARP_EPS`]), else the plain projection
/// onto the face plane. `ring` (the outer boundary) anchors the chart's angular
/// unwrap. Falls back to the projection plane whenever the chart refuses (planar
/// ring, degenerate ring, a vertex outside the injective domain) — the
/// pre-parameter-space behaviour, never a guess.
fn face_clip_p2(pts: &[DVec3], ring: &[DVec3], normal: DVec3, surface: &Surface) -> (Vec<glam::DVec2>, bool) {
	if let Some(p2) = SurfaceChart::for_warped_ring(surface, ring, normal).and_then(|c| c.uv_ring(pts)) {
		return (p2, true);
	}
	let (u, v) = perp_basis(normal);
	(pts.iter().map(|p| glam::DVec2::new(p.dot(u), p.dot(v))).collect(), false)
}

/// Ear-clip an index ring over `poly`/`p2` into [`Tri`]s. The ring may repeat an
/// index (a bridged hole's doubled corridor vertices): the ear containment test
/// skips occurrences of the ear's own indices by INDEX equality, so a repeated
/// bridge vertex never blocks its other occurrence's ears — the property that makes
/// bridged multi-loop rings clip cleanly (mirrors `tessellate::ear_clip_ring`).
///
/// `chart` marks `p2` as PARAMETER-SPACE coordinates (a warped curved face). The
/// clip then refuses to take a 3-D-degenerate corner as an ear: a chart bends a
/// 3-D-collinear boundary run (T-junction-heal samples shared with 3+ loops) into
/// a genuinely convex 2-D corner, and clipping it would emit a zero-area triangle
/// the degenerate filter silently DROPS — deleting a boundary step the neighbour
/// face still carries, an unpairable directed edge that explodes the stitch. In a
/// projection plane this guard is unreachable (the projection is affine, so a
/// 2-D-convex corner is never 3-D-collinear), so the planar path is untouched.
#[allow(clippy::too_many_arguments)] // one ring + its clip space + face tags
fn ear_clip_ring_tris(poly: &[DVec3], p2: &[glam::DVec2], mut idx: Vec<usize>, normal: DVec3, source: FaceName, surface: Surface, chart: bool, out: &mut Vec<Tri>) {
	if idx.len() < 3 {
		return;
	}
	// Restore the original winding if the index list was reversed for the ear
	// test, so emitted triangles keep the face's outward orientation.
	let reversed = signed_area_2d(p2, &idx) < 0.0;
	if reversed {
		idx.reverse();
	}
	let emit = |out: &mut Vec<Tri>, a: usize, b: usize, c: usize| {
		let mut t = if reversed {
			Tri { v: [poly[c], poly[b], poly[a]], normal, source, surface }
		} else {
			Tri { v: [poly[a], poly[b], poly[c]], normal, source, surface }
		};
		if !t.is_degenerate() {
			// Carry the triangle's OWN unit plane normal (sign-matched to the face's
			// outward `normal`), not the face's averaged Newell normal. For an exactly
			// planar face the two agree to f64 round-off, but a stitched result face
			// is only planar to the weld/heal tolerance (~1e-7): re-triangulating it
			// in a LATER boolean must reconstruct each fragment in the plane of the
			// triangle it came from (`split_triangle_by_segments` works ⊥ `t.normal`
			// through `t.v[0]`), or fragments flatten onto the borrowed average plane
			// and the two operands disagree about shared seam corners at the heal
			// scale — unhealable micro-holes in chained booleans.
			let gn = t.area_vec().normalize_or_zero();
			if gn.length_squared() > 0.5 {
				t.normal = if gn.dot(normal) < 0.0 { -gn } else { gn };
			}
			out.push(t);
		}
	};

	let mut guard = 0;
	while idx.len() > 3 && guard < 100_000 {
		guard += 1;
		let n = idx.len();
		let mut clipped = false;
		for i in 0..n {
			let ip = idx[(i + n - 1) % n];
			let ic = idx[i];
			let inx = idx[(i + 1) % n];
			let (a, b, c) = (p2[ip], p2[ic], p2[inx]);
			// Exact orientation: a near-collinear corner classifies as reflex/flat
			// consistently rather than by a rounded f64 cross product.
			if orient2d([a.x, a.y], [b.x, b.y], [c.x, c.y]) <= 0.0 {
				continue; // reflex corner
			}
			// In a chart, a 2-D-convex corner can still be a 3-D-collinear run whose
			// ear would be dropped as degenerate, deleting a shared boundary step
			// (see the doc above) — leave the vertex for a neighbouring ear instead.
			if chart {
				let area2 = (poly[ic] - poly[ip]).cross(poly[inx] - poly[ic]).length();
				if area2 * 0.5 <= EPS * EPS {
					continue;
				}
			}
			let mut ok = true;
			for &j in &idx {
				if j == ip || j == ic || j == inx {
					continue;
				}
				if point_in_tri_2d(p2[j], a, b, c) {
					ok = false;
					break;
				}
			}
			if ok {
				emit(out, ip, ic, inx);
				idx.remove(i);
				clipped = true;
				break;
			}
		}
		if !clipped {
			break;
		}
	}
	for w in 1..idx.len().saturating_sub(1) {
		emit(out, idx[0], idx[w], idx[w + 1]);
	}
}

fn signed_area_2d(p2: &[glam::DVec2], idx: &[usize]) -> f64 {
	let mut a = 0.0;
	let n = idx.len();
	for i in 0..n {
		let c = p2[idx[i]];
		let d = p2[idx[(i + 1) % n]];
		a += c.x * d.y - d.x * c.y;
	}
	a * 0.5
}

fn point_in_tri_2d(p: glam::DVec2, a: glam::DVec2, b: glam::DVec2, c: glam::DVec2) -> bool {
	// Exact orientation sign (`sign(p1,p2,p3) = orient2d(p3,p1,p2)`): a point on a
	// candidate ear's edge classifies consistently instead of by a rounded f64.
	let sign = |p1: glam::DVec2, p2: glam::DVec2, p3: glam::DVec2| orient2d([p3.x, p3.y], [p1.x, p1.y], [p2.x, p2.y]);
	let d1 = sign(p, a, b);
	let d2 = sign(p, b, c);
	let d3 = sign(p, c, a);
	let has_neg = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
	let has_pos = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
	!(has_neg && has_pos)
}

// --- Step 2: co-refinement ---------------------------------------------------

/// Split every triangle of `subject` along its intersection with every triangle
/// of `cutter`, returning the fragments. After this the surface of `cutter` never
/// crosses the interior of any returned fragment, so each fragment is uniformly
/// inside or outside the `cutter` solid.
fn co_refine(subject: &[Tri], cutter: &[Tri], workers: usize) -> Vec<Tri> {
	// Broadphase: two triangles can only cut each other if their AABBs overlap. A
	// uniform spatial grid over the cutter boxes yields the nearby candidates in
	// ~O(1), turning the naïve O(n·m) scan into O(n+m) typical. It is purely a cull —
	// the f64 AABB re-test below keeps the output byte-for-byte identical.
	let cutter_boxes: Vec<(DVec3, DVec3)> = cutter.iter().map(tri_aabb).collect();
	let grid = CutterGrid::build(&cutter_boxes);
	// Pure per-SUBJECT-TRIANGLE map (parallelism-safe, see the module's
	// parallelism section): the grid and cutter list are read-only shared input
	// (`candidates` sorts + dedups, so candidate ORDER never depends on HashMap
	// iteration), the scratch buffers are chunk-local and cleared per triangle
	// exactly as the sequential loop cleared them, and fragments concatenate in
	// subject order.
	//
	// Scheduling (output-invariant): a FEW large subject triangles against MANY
	// small cutters is the expensive shape — each large AABB overflows the grid's
	// span cap into the check-all path, costing O(|cutter|) per item — so the
	// engage test and chunk length are work-based like classification's, with
	// the pessimistic |subject|×|cutter| bound as the work proxy (over-engaging
	// a well-pruned stage wastes only the ~0.1 ms spawn, never correctness).
	let work = subject.len().saturating_mul(cutter.len().max(1));
	let chunk_len = (CLASSIFY_CHUNK_WORK / cutter.len().max(1)).clamp(1, PAR_CHUNK);
	let engage = subject.len() >= PAR_CUTOFF || work >= CLASSIFY_WORK_CUTOFF;
	stage_flat_map(workers, subject, chunk_len, engage, |chunk| {
		let mut out = Vec::with_capacity(chunk.len());
		let mut segments: Vec<(DVec3, DVec3)> = Vec::new();
		let mut cand: Vec<u32> = Vec::new();
		for &t in chunk {
			segments.clear();
			let tb = tri_aabb(&t);
			grid.candidates(tb, cutter.len(), &mut cand);
			for &ci in &cand {
				if aabb_overlap(tb, cutter_boxes[ci as usize]) {
					tri_tri_cuts(&t, &cutter[ci as usize], &mut segments);
				}
			}
			if segments.is_empty() {
				out.push(t);
			} else {
				split_triangle_by_segments(&t, &segments, &mut out);
			}
		}
		out
	})
}

/// A uniform spatial hash over cutter triangle AABBs for broadphase candidate
/// lookup. Cutters spanning too many cells go in a `large` list checked against
/// every subject. It returns a *superset* of the overlapping cutters (overlapping
/// AABBs always share a cell), so the f64 re-test in [`co_refine`] still culls
/// exactly and the boolean result is unchanged.
struct CutterGrid {
	inv_cell: f64,
	cells: HashMap<[i64; 3], Vec<u32>>,
	large: Vec<u32>,
}

impl CutterGrid {
	fn key(inv: f64, p: DVec3) -> [i64; 3] {
		[(p.x * inv).floor() as i64, (p.y * inv).floor() as i64, (p.z * inv).floor() as i64]
	}

	fn build(boxes: &[(DVec3, DVec3)]) -> Self {
		const CAP: i64 = 64; // cell budget before a cutter is treated as "large"
		if boxes.is_empty() {
			return CutterGrid { inv_cell: 1.0, cells: HashMap::new(), large: Vec::new() };
		}
		// Cell ≈ mean triangle extent, so a typical triangle spans ~one cell.
		let mean = boxes.iter().map(|(lo, hi)| (*hi - *lo).max_element()).sum::<f64>() / boxes.len() as f64;
		let inv_cell = 1.0 / mean.max(1e-12);
		let mut cells: HashMap<[i64; 3], Vec<u32>> = HashMap::new();
		let mut large = Vec::new();
		for (i, &(lo, hi)) in boxes.iter().enumerate() {
			let (klo, khi) = (Self::key(inv_cell, lo), Self::key(inv_cell, hi));
			let span = (khi[0] - klo[0] + 1) * (khi[1] - klo[1] + 1) * (khi[2] - klo[2] + 1);
			if span > CAP {
				large.push(i as u32);
				continue;
			}
			for cx in klo[0]..=khi[0] {
				for cy in klo[1]..=khi[1] {
					for cz in klo[2]..=khi[2] {
						cells.entry([cx, cy, cz]).or_default().push(i as u32);
					}
				}
			}
		}
		CutterGrid { inv_cell, cells, large }
	}

	fn candidates(&self, (lo, hi): (DVec3, DVec3), n: usize, out: &mut Vec<u32>) {
		out.clear();
		let (klo, khi) = (Self::key(self.inv_cell, lo), Self::key(self.inv_cell, hi));
		let span = (khi[0] - klo[0] + 1) * (khi[1] - klo[1] + 1) * (khi[2] - klo[2] + 1);
		if span > 4096 {
			out.extend(0..n as u32); // pathologically large subject: just check all
			return;
		}
		out.extend_from_slice(&self.large);
		for cx in klo[0]..=khi[0] {
			for cy in klo[1]..=khi[1] {
				for cz in klo[2]..=khi[2] {
					if let Some(v) = self.cells.get(&[cx, cy, cz]) {
						out.extend_from_slice(v);
					}
				}
			}
		}
		out.sort_unstable();
		out.dedup();
	}
}

/// Axis-aligned bounding box of a triangle.
fn tri_aabb(t: &Tri) -> (DVec3, DVec3) {
	(t.v[0].min(t.v[1]).min(t.v[2]), t.v[0].max(t.v[1]).max(t.v[2]))
}

/// Do two AABBs overlap? Uses *exact* comparisons (no `EPS` slack) so this stays
/// consistent with [`CutterGrid`]'s exact `floor` cell keys: overlapping boxes then
/// provably share a grid cell, preserving the broadphase superset invariant. A
/// genuine geometric tolerance is applied downstream by `tri_tri_cuts`, not here.
fn aabb_overlap(a: (DVec3, DVec3), b: (DVec3, DVec3)) -> bool {
	a.0.x <= b.1.x && a.1.x >= b.0.x && a.0.y <= b.1.y && a.1.y >= b.0.y && a.0.z <= b.1.z && a.1.z >= b.0.z
}

/// The intersection segment of two triangles, in `subject`'s plane, or `None` if
/// they do not cross in a segment (disjoint, touching at a point, or coplanar —
/// coplanar overlap is handled by classification, not by splitting).
fn tri_tri_cuts(subject: &Tri, cutter: &Tri, out: &mut Vec<(DVec3, DVec3)>) {
	let n_s = subject.area_vec().normalize_or_zero();
	let n_c = cutter.area_vec().normalize_or_zero();
	if n_s.length_squared() < 0.5 || n_c.length_squared() < 0.5 {
		return;
	}
	// `|n_s × n_c|` is the sine of the angle between the two face normals. Use the
	// same ~1e-6 tolerance as the in-plane / on-plane coincidence tests below and in
	// `point_in_tri_3d`, so a pair of near-coplanar shared faces is imprinted (not
	// co-refined as transversal, which would leave the stitched solid non-manifold).
	if n_s.cross(n_c).length() < 1e-6 {
		// Coplanar: imprint the cutter's three edge *lines* onto the subject so the
		// coplanar overlap region is carved out along the cutter's boundary. The
		// raw (unclipped) edges are emitted as cut segments; `split_convex_by_line`
		// only acts where a line actually crosses the subject, so a cutter edge that
		// runs outside the subject still contributes its bounding line — which is
		// essential, since the overlap polygon is bounded by every cutter edge, not
		// only the ones passing through the subject triangle.
		if (n_s.dot(cutter.v[0] - subject.v[0])).abs() > 1e-6 {
			return; // parallel but not in the same plane
		}
		for i in 0..3 {
			out.push((cutter.v[i], cutter.v[(i + 1) % 3]));
		}
		return;
	}
	// Transversal case: plane–plane SSI line clipped to both triangle polygons.
	let d_s = n_s.dot(subject.v[0]);
	let Some(chord) = triangle_plane_chord(&cutter.v, &subject.v, n_s, d_s) else {
		return;
	};
	let Some(seg) = clip_segment_to_triangle(chord, subject) else {
		return;
	};
	let Some(seg) = clip_segment_to_triangle(seg, cutter) else {
		return;
	};
	if (seg.1 - seg.0).length() >= EPS {
		out.push(seg);
	}
}

/// Exact side of `p` relative to the oriented plane through `plane`'s three
/// points: `+1` / `-1` / `0` (on the plane), via the exact [`orient3d`] predicate.
/// The absolute sign convention is irrelevant to the callers here — they only use
/// equality-to-zero and opposite-signs, both invariant under a global flip.
fn plane_side(plane: &[DVec3; 3], p: DVec3) -> i32 {
	let arr = |v: DVec3| [v.x, v.y, v.z];
	let o = orient3d(arr(plane[0]), arr(plane[1]), arr(plane[2]), arr(p));
	if o > 0.0 {
		1
	} else if o < 0.0 {
		-1
	} else {
		0
	}
}

/// Chord where triangle `tri` crosses the supporting plane of `plane` (the
/// subject's three vertices): the segment of `tri` lying on the plane, or `None`
/// if `tri` does not straddle it.
///
/// Which side each vertex lies on — and therefore which edges cross — is decided
/// by the **exact** [`orient3d`] predicate, so a near-coplanar vertex is classified
/// by its true side rather than swallowed by an absolute epsilon. The crossing
/// *position* is still interpolated in `f64` from the plane distance (`n`, `d`),
/// which is all the downstream geometry needs.
fn triangle_plane_chord(tri: &[DVec3; 3], plane: &[DVec3; 3], n: DVec3, d: f64) -> Option<(DVec3, DVec3)> {
	let side = [plane_side(plane, tri[0]), plane_side(plane, tri[1]), plane_side(plane, tri[2])];
	let dist = [n.dot(tri[0]) - d, n.dot(tri[1]) - d, n.dot(tri[2]) - d];
	let mut hits: Vec<DVec3> = Vec::new();
	for i in 0..3 {
		let j = (i + 1) % 3;
		if side[i] == 0 {
			hits.push(tri[i]);
		}
		// Exactly-opposite sides ⇒ the edge crosses the plane between the vertices.
		if side[i] * side[j] < 0 {
			let (di, dj) = (dist[i], dist[j]);
			let t = di / (di - dj);
			hits.push(tri[i] + (tri[j] - tri[i]) * t);
		}
	}
	dedup_close(&mut hits);
	if hits.len() >= 2 {
		Some((hits[0], hits[hits.len() - 1]))
	} else {
		None
	}
}

fn dedup_close(pts: &mut Vec<DVec3>) {
	let mut i = 0;
	while i < pts.len() {
		let mut j = i + 1;
		while j < pts.len() {
			if (pts[i] - pts[j]).length() <= EPS {
				pts.remove(j);
			} else {
				j += 1;
			}
		}
		i += 1;
	}
}

/// Clip a 3D segment to a triangle, treating the segment and triangle as
/// coplanar (the segment already lies in the triangle's plane). Returns the
/// portion of the segment inside the triangle, or `None` if it misses.
fn clip_segment_to_triangle(seg: (DVec3, DVec3), tri: &Tri) -> Option<(DVec3, DVec3)> {
	let n = tri.area_vec().normalize_or_zero();
	if n.length_squared() < 0.5 {
		return None;
	}
	let (mut t0, mut t1) = (0.0f64, 1.0f64);
	let dir = seg.1 - seg.0;
	// Half-plane clip against each triangle edge (inward normal = n × edge).
	for i in 0..3 {
		let a = tri.v[i];
		let b = tri.v[(i + 1) % 3];
		let edge = b - a;
		let inward = n.cross(edge); // points into the triangle for CCW (n,winding)
		// Constraint: inward · (P − a) >= 0.
		let denom = inward.dot(dir);
		let num = inward.dot(seg.0 - a);
		if denom.abs() < EPS {
			if num < -EPS {
				return None; // segment entirely outside this edge
			}
		} else {
			let t = -num / denom;
			if denom > 0.0 {
				if t > t0 {
					t0 = t;
				}
			} else if t < t1 {
				t1 = t;
			}
		}
		if t0 > t1 + EPS {
			return None;
		}
	}
	let p0 = seg.0 + dir * t0;
	let p1 = seg.0 + dir * t1;
	if (p1 - p0).length() < EPS {
		None
	} else {
		Some((p0, p1))
	}
}

/// Split triangle `t` so that none of the `segments` (each lying in `t`'s plane)
/// crosses a fragment's interior. Each cut segment defines a supporting line in
/// the face plane; the triangle (as a convex polygon) is split by every such line
/// into convex sub-polygons, which are then fanned into triangles.
///
/// Splitting by the *whole* supporting line can over-split (a fragment may be cut
/// where the segment does not actually reach), but it never under-splits, so no
/// fragment ever straddles a cutter face — exactly the invariant classification
/// needs. Over-splitting is harmless: coplanar adjacent fragments are merged back
/// into one face during stitching.
fn split_triangle_by_segments(t: &Tri, segments: &[(DVec3, DVec3)], out: &mut Vec<Tri>) {
	let n = t.normal;
	let (u, v) = perp_basis(n);
	let to2 = |p: DVec3| glam::DVec2::new(p.dot(u), p.dot(v));
	let to3 = |q: glam::DVec2| {
		// Reconstruct the 3D point: u,v span the plane through t.v[0].
		let o = t.v[0];
		o + u * (q.x - o.dot(u)) + v * (q.y - o.dot(v))
	};

	// Work in 2D. Start with the triangle as one convex polygon.
	let mut polys: Vec<Vec<glam::DVec2>> = vec![t.v.iter().map(|&p| to2(p)).collect()];

	for &(p, q) in segments {
		let a = to2(p);
		let b = to2(q);
		let dir = b - a;
		if dir.length_squared() < EPS * EPS {
			continue;
		}
		// UNIT line normal (in-plane); signed distance side = line_n · (x − a).
		// Normalizing is load-bearing: with the raw perp (length = |segment|) the
		// EPS on-line band in `split_convex_by_line` was EPS/|segment| in real
		// distance — a SHORT cut segment (a chord clipped to a sliver of the other
		// operand near a seam corner, often ~1e-5 mm) ballooned the band to ~1e-4,
		// swallowing genuine crossings; the two operands then disagreed about the
		// seam polyline by far more than WELD/TJUNCTION_EPS and the stitch left a
		// micro-triangle hole (the dominant residual fuzz-failure class).
		let line_n = glam::DVec2::new(-dir.y, dir.x).normalize_or_zero();
		if line_n.length_squared() < 0.5 {
			continue;
		}
		let mut next: Vec<Vec<glam::DVec2>> = Vec::with_capacity(polys.len());
		for poly in &polys {
			split_convex_by_line(poly, a, line_n, &mut next);
		}
		polys = next;
	}

	// Fan each convex sub-polygon into triangles.
	for poly in &polys {
		if poly.len() < 3 {
			continue;
		}
		for w in 1..poly.len() - 1 {
			let frag = Tri { v: [to3(poly[0]), to3(poly[w]), to3(poly[w + 1])], normal: n, source: t.source, surface: t.surface };
			if !frag.is_degenerate() {
				out.push(orient_tri(frag, n));
			}
		}
	}
}

/// Split a convex polygon by the line `{x : line_n·(x − a) = 0}` (`line_n` is a
/// UNIT normal) into the pieces on each side, appending the pieces to `out`. A
/// polygon entirely on one side passes through unchanged.
fn split_convex_by_line(
	poly: &[glam::DVec2],
	a: glam::DVec2,
	line_n: glam::DVec2,
	out: &mut Vec<Vec<glam::DVec2>>,
) {
	// NB: an absolute EPS (not an exact orient2d sign) is load-bearing here. This
	// polygon lives in f64-*projected* 2-D coordinates, so a vertex that is
	// geometrically on the split line is only approximately on it numerically; the
	// EPS keeps it shared between both sub-polygons (crack-free). Exact orientation
	// on these inexact coordinates would split such a vertex to one side and open a
	// crack on off-axis geometry — the win requires an exact *projection*, not just
	// an exact predicate.
	let dist: Vec<f64> = poly.iter().map(|&p| line_n.dot(p - a)).collect();
	let has_pos = dist.iter().any(|&d| d > EPS);
	let has_neg = dist.iter().any(|&d| d < -EPS);
	if !(has_pos && has_neg) {
		out.push(poly.to_vec()); // does not straddle the line
		return;
	}
	let mut pos: Vec<glam::DVec2> = Vec::new();
	let mut neg: Vec<glam::DVec2> = Vec::new();
	let n = poly.len();
	for i in 0..n {
		let j = (i + 1) % n;
		let (di, dj) = (dist[i], dist[j]);
		let pi = poly[i];
		// Classify current vertex.
		if di >= -EPS {
			pos.push(pi);
		}
		if di <= EPS {
			neg.push(pi);
		}
		// If the edge crosses the line, add the split point to both sides.
		if (di > EPS && dj < -EPS) || (di < -EPS && dj > EPS) {
			let tparam = di / (di - dj);
			let x = pi + (poly[j] - pi) * tparam;
			pos.push(x);
			neg.push(x);
		}
	}
	// Keep BOTH pieces regardless of area. A piece whose area is tiny (a seam-corner
	// micro-triangle where two cut lines cross near a mesh edge) can still have edges
	// ~1e-4 long; discarding it (the old `area > EPS` gate) ripped a hole in the
	// operand's coverage that welding (1e-7) and T-junction healing (4e-7) cannot
	// close — the second residual fuzz-failure mechanism. True zero-extent debris is
	// still pruned downstream: the triangle fan skips `len < 3` polygons and drops
	// `Tri::is_degenerate()` fragments, and welding collapses sub-WELD_EPS pieces.
	if pos.len() >= 3 {
		out.push(pos);
	}
	if neg.len() >= 3 {
		out.push(neg);
	}
}

/// True if `p` lies inside or on triangle `t` (coplanar test).
fn point_in_tri_3d(p: DVec3, t: &Tri) -> bool {
	// Scale-invariant degeneracy guard: `normalize_or_zero` yields a unit normal for
	// any non-collapsed triangle (however small its area) and the zero vector only
	// for a genuinely degenerate one — matching the convention used elsewhere here.
	let nn = t.area_vec().normalize_or_zero();
	if nn.length_squared() < 0.5 {
		return false;
	}
	// Must be (near) coplanar.
	if (p - t.v[0]).dot(nn).abs() > 1e-6 {
		return false;
	}
	// EPS slack (not an exact predicate) is load-bearing: `p` is typically an
	// interpolated point, so its incidence with an edge is only approximate. See the
	// note in `split_convex_by_line`.
	for i in 0..3 {
		let a = t.v[i];
		let b = t.v[(i + 1) % 3];
		let c = (b - a).cross(p - a);
		if c.dot(nn) < -EPS {
			return false;
		}
	}
	true
}

/// Re-orient a triangle's winding so its geometric normal agrees with `n`.
fn orient_tri(mut t: Tri, n: DVec3) -> Tri {
	if t.area_vec().dot(n) < 0.0 {
		t.v.swap(1, 2);
	}
	t.normal = n;
	t
}

// --- Step 3 + 4: classification and selection --------------------------------

/// Where a fragment's centroid lies relative to the *other* solid.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Side {
	Inside,
	Outside,
	/// The fragment is coplanar with (and overlapping) a face of the other solid.
	/// `aligned` is true when the two surface normals point the same way.
	On { aligned: bool },
}

/// Classify each fragment of one operand against the `other` solid's triangles
/// and return the kept (and possibly flipped) fragments, in fragment order.
///
/// Fragments coplanar with a face of the other solid (shared/coincident facets)
/// are resolved by normal agreement so they appear in the output exactly once:
/// to avoid double-counting a shared face, only the A-side keeps coincident
/// faces; the B-side drops them.
///
/// Pure per-FRAGMENT map (parallelism-safe, see the module's parallelism
/// section): each fragment's verdict reads only that fragment and the read-only
/// `other` triangle list — the ray-cast scans `other` in slice order with fixed
/// retry directions, no shared accumulator anywhere.
fn classify_select(frags: &[Tri], other: &[Tri], op: Op, is_b: bool, workers: usize) -> Vec<Tri> {
	// Work-based scheduling (output-invariant, see CLASSIFY_WORK_CUTOFF): each
	// fragment costs one scan of `other`, so total work — and the chunk length
	// that yields ~CLASSIFY_CHUNK_WORK units per chunk — scales with |other|.
	let work = frags.len().saturating_mul(other.len().max(1));
	let chunk_len = (CLASSIFY_CHUNK_WORK / other.len().max(1)).clamp(1, PAR_CHUNK);
	stage_flat_map(workers, frags, chunk_len, work >= CLASSIFY_WORK_CUTOFF, |chunk| {
		let mut kept: Vec<Tri> = Vec::new();
		for &t in chunk {
			if t.is_degenerate() {
				continue;
			}
			let side = classify_point(t.centroid(), t.normal, other);
			let keep = match side {
				Side::Inside => match op {
					Op::Union => false,
					Op::Intersection => true,
					Op::Difference => is_b, // A inside B removed; B inside A kept (flipped)
				},
				Side::Outside => match op {
					Op::Union => true,
					Op::Intersection => false,
					Op::Difference => !is_b, // A outside B kept; B outside A removed
				},
				Side::On { aligned } => {
					// Coincident faces lie on both solids' boundaries. With this
					// fragment's outward normal `n`, its own material is on the `−n`
					// side; the coincident other face has material on the same side when
					// `aligned`, opposite when not.
					//
					// * Union / intersection: an aligned coincident face has material on
					//   one side and void on the other for the result ⇒ it is a true
					//   boundary ⇒ keep. Opposed faces have material on both sides ⇒
					//   interior ⇒ drop. Both operands emit it; `cancel_coincident`
					//   collapses the duplicate to a single facet.
					// * Difference A−B: where the faces are *aligned*, B's material
					//   coincides with A's and is subtracted away, so A's face vanishes
					//   ⇒ drop. Where *opposed*, B is on the far side and A's face
					//   survives ⇒ keep (A-side only; the flipped B-side is suppressed).
					match (op, is_b) {
						// Aligned coincident faces are a shared boundary that must appear in
						// the result exactly once. Keep the A-side copy and drop the B-side
						// outright (`is_b`), rather than keeping both and trusting
						// `cancel_coincident` to collapse them: when the two faces only
						// partially overlap and were cut by *different* triangulation
						// diagonals, the duplicates are not identical and fail to cancel,
						// leaving a doubled, non-manifold shared face.
						(Op::Union, false) | (Op::Intersection, false) => aligned,
						(Op::Union, true) | (Op::Intersection, true) => false,
						(Op::Difference, false) => !aligned,
						(Op::Difference, true) => false,
					}
				}
			};
			if !keep {
				continue;
			}
			if op == Op::Difference && is_b {
				let mut f = t;
				f.v.swap(1, 2);
				f.normal = -f.normal;
				kept.push(f);
			} else {
				kept.push(t);
			}
		}
		kept
	})
}

/// Three-way classification of point `p` (carrying its fragment `normal`) against
/// the `other` solid. Coplanar coincidence is detected first; otherwise ray
/// casting decides inside/outside.
fn classify_point(p: DVec3, normal: DVec3, other: &[Tri]) -> Side {
	for c in other {
		let nc = c.area_vec();
		let nl = nc.length();
		if nl < EPS {
			continue;
		}
		let ncn = nc / nl;
		// Centroid on this face's plane and inside its triangle?
		if (p - c.v[0]).dot(ncn).abs() <= 1e-7 && point_in_tri_3d(p, c) {
			return Side::On { aligned: normal.dot(ncn) > 0.0 };
		}
	}
	if point_inside(p, other) {
		Side::Inside
	} else {
		Side::Outside
	}
}

/// Robust point-in-polyhedron test by ray casting against `tris`. Counts crossings
/// of a ray from `p` in a pseudo-random direction; odd ⇒ inside. Faces are
/// treated as a closed surface; a small jitter and retry avoids degenerate hits
/// through shared edges/vertices.
fn point_inside(p: DVec3, tris: &[Tri]) -> bool {
	let dirs = [
		DVec3::new(1.0, 1.0, 1.0).normalize(),
		DVec3::new(1.0, 0.0, 1.0).normalize(),
		DVec3::new(0.26726, 0.53452, 0.80178),
		DVec3::new(-0.4082, 0.8165, -0.4082),
		DVec3::new(0.123, -0.567, 0.814).normalize(),
	];
	for &dir in &dirs {
		if let Some(crossings) = ray_crossings(p, dir, tris) {
			return crossings % 2 == 1;
		}
		// `None` ⇒ a near-degenerate hit; retry with another direction.
	}
	false
}

/// Count ray–triangle crossings, or `None` if any hit is numerically ambiguous
/// (ray grazes an edge/vertex/plane), signalling the caller to pick a new ray.
fn ray_crossings(orig: DVec3, dir: DVec3, tris: &[Tri]) -> Option<usize> {
	let mut count = 0usize;
	for t in tris {
		match moller_trumbore(orig, dir, t) {
			Hit::Cross => count += 1,
			Hit::Miss => {}
			Hit::Degenerate => return None,
		}
	}
	Some(count)
}

enum Hit {
	Cross,
	Miss,
	Degenerate,
}

/// Möller–Trumbore ray/triangle test, classifying grazing hits as `Degenerate`
/// so the caller can re-cast and keep parity well-defined.
fn moller_trumbore(orig: DVec3, dir: DVec3, t: &Tri) -> Hit {
	let e1 = t.v[1] - t.v[0];
	let e2 = t.v[2] - t.v[0];
	let pvec = dir.cross(e2);
	let det = e1.dot(pvec);
	if det.abs() < 1e-12 {
		return Hit::Miss; // ray parallel to triangle
	}
	let inv = 1.0 / det;
	let tvec = orig - t.v[0];
	let u = tvec.dot(pvec) * inv;
	let qvec = tvec.cross(e1);
	let v = dir.dot(qvec) * inv;
	let dist = e2.dot(qvec) * inv;
	let edge_tol = 1e-9;
	// Grazing the boundary (barycentric on/near an edge) ⇒ ambiguous parity.
	if u > -edge_tol && u < edge_tol
		|| v > -edge_tol && v < edge_tol
		|| (u + v) > 1.0 - edge_tol && (u + v) < 1.0 + edge_tol
		|| dist.abs() < edge_tol
	{
		// Only ambiguous if the hit is actually within/near the triangle and ahead.
		if u >= -edge_tol && v >= -edge_tol && u + v <= 1.0 + edge_tol && dist > -edge_tol {
			return Hit::Degenerate;
		}
	}
	if !(0.0..=1.0).contains(&u) || v < 0.0 || u + v > 1.0 {
		return Hit::Miss;
	}
	if dist > edge_tol {
		Hit::Cross
	} else {
		Hit::Miss
	}
}

// --- Step 5: stitch into a closed half-edge solid ----------------------------

/// Merge coplanar adjacent kept triangles back into maximal planar faces and
/// build a closed [`Solid`] via [`Solid::from_faces`]. Vertices are welded to
/// [`WELD_EPS`] so twin half-edges pair up.
fn stitch(kept: &[Tri]) -> Solid {
	if kept.is_empty() {
		return Solid::default();
	}

	// Weld vertices to a shared index space.
	let mut verts: Vec<DVec3> = Vec::new();
	let mut grid: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::new();
	let inv = 1.0 / WELD_EPS;
	let key = |p: DVec3| {
		(
			(p.x * inv).round() as i64,
			(p.y * inv).round() as i64,
			(p.z * inv).round() as i64,
		)
	};
	let weld = |p: DVec3, verts: &mut Vec<DVec3>, grid: &mut HashMap<(i64, i64, i64), Vec<u32>>| -> u32 {
		let k = key(p);
		for dz in -1..=1 {
			for dy in -1..=1 {
				for dx in -1..=1 {
					if let Some(ids) = grid.get(&(k.0 + dx, k.1 + dy, k.2 + dz)) {
						for &id in ids {
							if (verts[id as usize] - p).length() <= WELD_EPS {
								return id;
							}
						}
					}
				}
			}
		}
		let id = verts.len() as u32;
		verts.push(p);
		grid.entry(k).or_default().push(id);
		id
	};

	// Weld every triangle's vertices first (ids only; filters run after the
	// duplicate merge below so they see final positions).
	let widx: Vec<[u32; 3]> = kept
		.iter()
		.map(|t| {
			[
				weld(t.v[0], &mut verts, &mut grid),
				weld(t.v[1], &mut verts, &mut grid),
				weld(t.v[2], &mut verts, &mut grid),
			]
		})
		.collect();

	// Merge sub-heal-scale vertex DUPLICATES the greedy weld left distinct. The
	// weld's first-fit ball is [`WELD_EPS`]; the stitch's own resolution is
	// [`TJUNCTION_EPS`] (the sliver filter drops thinner triangles, the healer
	// moves geometry by up to it) — so two distinct vertices closer than
	// TJUNCTION_EPS are the SAME point at stitch resolution, yet they are
	// unstitchable: `resolve_t_junctions` cannot insert either into an edge
	// ending at the other (the projection parameter lands within EPS of the
	// endpoint) and the welder already ruled. Fuzz seed 83894724550888 measured
	// the class: two fragments of one operand face disagreed about a seam corner
	// by 1.051e-7 — 5% OVER the weld ball — leaving an unpairable zero-area slit
	// (`8→74, 74→73, 73→8`). Clusters are found on a TJUNCTION_EPS-sized grid
	// and united by min id (union-find, ids ascending — deterministic); the
	// representative keeps its own coordinates, so every survivor is bit-stable
	// and a merged PAIR moves one vertex ≤ TJUNCTION_EPS. Transitive chains can
	// in principle span further (the weld only guarantees pairwise > WELD_EPS,
	// so a string of ~2e-7 steps would unify) — but such a string is already
	// sub-resolution geometry to the sliver filter and healer, and the honest
	// arbiter is the corpus: N=200/2000/10 000 all at 100 % with this merge
	// (ROBUSTNESS.md 2026-07-30), exact-volume and 1e-9 seam gates unchanged.
	let remap: Vec<u32> = {
		let cell = 1.0 / TJUNCTION_EPS;
		let ckey = |p: DVec3| ((p.x * cell).round() as i64, (p.y * cell).round() as i64, (p.z * cell).round() as i64);
		let mut cgrid: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::new();
		for (i, p) in verts.iter().enumerate() {
			cgrid.entry(ckey(*p)).or_default().push(i as u32);
		}
		let mut parent: Vec<u32> = (0..verts.len() as u32).collect();
		fn find(p: &mut [u32], mut i: u32) -> u32 {
			while p[i as usize] != i {
				p[i as usize] = p[p[i as usize] as usize];
				i = p[i as usize];
			}
			i
		}
		let mut candidates: Vec<u32> = Vec::new();
		for i in 0..verts.len() as u32 {
			let p = verts[i as usize];
			let k = ckey(p);
			candidates.clear();
			for dz in -1..=1_i64 {
				for dy in -1..=1_i64 {
					for dx in -1..=1_i64 {
						if let Some(ids) = cgrid.get(&(k.0 + dx, k.1 + dy, k.2 + dz)) {
							candidates.extend(ids.iter().copied().filter(|&j| j > i));
						}
					}
				}
			}
			candidates.sort_unstable();
			for &j in &candidates {
				if (verts[j as usize] - p).length() <= TJUNCTION_EPS {
					let (ri, rj) = (find(&mut parent, i), find(&mut parent, j));
					if ri != rj {
						// Union by MIN root so the cluster representative is the
						// smallest id — first-insertion order, like the weld itself.
						let (lo, hi) = if ri < rj { (ri, rj) } else { (rj, ri) };
						parent[hi as usize] = lo;
					}
				}
			}
		}
		(0..verts.len() as u32).map(|i| find(&mut parent, i)).collect()
	};

	// Indexed triangles, dropping any that collapse on welding (or on the
	// duplicate merge). Each carries the persistent name and analytic surface of
	// the operand face it came from.
	let mut raw: Vec<RawTri> = Vec::with_capacity(kept.len());
	for (t, ids) in kept.iter().zip(&widx) {
		let a = remap[ids[0] as usize];
		let b = remap[ids[1] as usize];
		let c = remap[ids[2] as usize];
		if a == b || b == c || a == c {
			continue;
		}
		// Drop slivers thinner than the T-junction healing tolerance (min altitude =
		// 2·area / longest side, on the WELDED positions). Such a triangle is wedged
		// between two differently-subdivided runs of the same line — re-triangulating
		// a previous boolean's face emits them along healed near-collinear chains —
		// and it cannot be healed by vertex insertion: `resolve_t_junctions` would
		// fold its own apex into its base edge, tripling that directed edge and
		// breaking twin pairing. Deleting it instead leaves a sub-tolerance gap whose
		// two sides T-junction-heal directly against each other (the apex IS within
		// `TJUNCTION_EPS` of the base, so it gets inserted into the neighbour's edge).
		//
		// CAVEAT (the former disjoint-difference fuzz failure): that heal argument is
		// LOCAL — it fails for a RUN of consecutive needles fanned from a far apex
		// along a near-collinear boundary chain ("deck of cards"), where dropping the
		// run leaves flanks a multiple of `TJUNCTION_EPS` apart. Those chains are now
		// removed at the source: `chain_redundant_vertices` strips redundant boundary
		// micro-subdivisions before triangulation, so such fans no longer arise.
		let (pa, pb, pc) = (verts[a as usize], verts[b as usize], verts[c as usize]);
		let area2 = (pb - pa).cross(pc - pa).length();
		let longest = (pb - pa).length().max((pc - pb).length()).max((pa - pc).length());
		if area2 < longest * TJUNCTION_EPS {
			continue;
		}
		raw.push(([a, b, c], t.normal, t.source, t.surface));
	}

	// Coincident-facet cancellation: two fragments occupying the same triangle in
	// opposite orientations form an interior membrane (delete both); two with the
	// same orientation are an exact duplicate (keep one). Without this, coincident
	// coplanar overlaps (e.g. two prisms sharing a cap) would emit a duplicate
	// directed edge and break the half-edge twin matcher.
	let (itris, tri_normal, tri_source, tri_surface) = cancel_coincident(&raw);

	// Group coplanar triangles that are connected, and recover their boundary
	// polygon. This turns the triangle soup back into B-rep faces. As a robust
	// fallback (e.g. a face that re-merges into a non-simple polygon) the group
	// is emitted as its individual triangles, which is still a valid closed solid.
	// `provenance` runs parallel to `faces`, carrying each face's source operand;
	// `face_members` (also parallel) carries each merged region's triangle indices.
	let (mut faces, mut provenance, face_members) = recover_faces(&itris, &tri_normal, &tri_source, &tri_surface, &verts);

	// Resolve T-junctions: a vertex lying in the interior of another face's edge
	// must be inserted into that edge, otherwise the half-edge twin matcher cannot
	// pair the long edge with the two shorter edges across the junction. This is
	// what makes the stitched solid closed + manifold for general overlaps. It edits
	// boundaries in place (a slice) so face count/order — and the parallel
	// `provenance` — stay aligned.
	resolve_t_junctions(&mut faces, &verts);

	// FINAL geometric verification, after healing: every merged region face must
	// (a) have an id-sane boundary — no repeated vertex, hence no (x,y)+(y,x)
	// self-fold that would put 3+ half-edges on one undirected edge — and (b)
	// ear-clip (in both the boolean's and the tessellator's clipper) to exactly its
	// member triangles' area. A region that wrapped around an unhealed over-split
	// seam walks an id-simple boundary with zero-width SPURS; healing can also jam a
	// previously clean polygon. Such a face tessellates to garbage area — and which
	// rotation of the identical boundary jammed depended on HashMap order, making
	// result volumes flake run to run. Re-expand any failing face into its member
	// triangles (always clippable), then re-heal those against the vertex pool
	// (`resolve_t_junctions` is idempotent on already-healed edges).
	let boundary_id_sane = |b: &[u32]| {
		let mut ids = b.to_vec();
		ids.sort_unstable();
		ids.windows(2).all(|w| w[0] != w[1])
	};
	let mut replaced = false;
	for fi in 0..faces.len() {
		let members = &face_members[fi];
		if members.len() < 2
			|| (boundary_id_sane(&faces[fi].boundary)
				&& boundary_covers_region(&faces[fi].boundary, members, &itris, &verts, tri_normal[members[0]]))
		{
			continue;
		}
		replaced = true;
		let face_surface = |ti: usize| -> Surface {
			let t = itris[ti];
			match tri_surface[ti] {
				s if !matches!(s, Surface::Plane { .. }) => s,
				_ => Surface::Plane { origin: verts[t[0] as usize], normal: tri_normal[ti] },
			}
		};
		let tri_input = |ti: usize| FaceInput { boundary: itris[ti].to_vec(), surface: face_surface(ti) };
		faces[fi] = tri_input(members[0]);
		provenance[fi] = tri_source[members[0]];
		for &ti in &members[1..] {
			faces.push(tri_input(ti));
			provenance.push(tri_source[ti]);
		}
	}
	if replaced {
		resolve_t_junctions(&mut faces, &verts);
	}

	// Strip redundant collinear chain micro-subdivisions from the RESULT boundaries —
	// the same per-vertex predicate `triangulate_solid` applies to operands, applied
	// at build time so the output is born clean. Welding and T-junction healing leave
	// mid-edge subdivision vertices on cut seams; on a CURVED operand's chords they
	// sit OFF the true intersection curve by the chord sagitta, and they are exactly
	// the vertices the seam snapper below must NOT pull onto the surface: bending an
	// exact chord facet there would unbalance the analytic bulge corrections that
	// make `exact_volume` machine-exact for ⟂ cuts. A removed vertex is used by
	// exactly two rings and is removed from both, so twin pairing is preserved.
	let rings: Vec<Vec<u32>> = faces.iter().map(|f| f.boundary.clone()).collect();
	let drop_v = chain_redundant_in_rings(&rings, &verts);
	if drop_v.iter().any(|&d| d) {
		for f in faces.iter_mut() {
			f.boundary.retain(|&v| !drop_v[v as usize]);
			// The per-ring removal budget keeps every boundary ≥ 3 vertices, so the
			// face list (and the parallel `provenance`) never shrinks here.
			debug_assert!(f.boundary.len() >= 3, "chain strip must leave a valid boundary");
		}
	}

	// Snap the remaining cut-seam vertices — now all genuine seam corners — onto the
	// exact surface–surface intersection of the operands' analytic surfaces.
	snap_seam_vertices(&mut verts, &faces);

	// Drop vertices no longer referenced by any face boundary. `recover_faces` merges a
	// coplanar triangle region into one maximal face, orphaning the vertices that were
	// interior to the merge; left in the array they inflate V (and the Euler characteristic
	// — a box crossing many curved facets otherwise reports a spurious genus). Compact the
	// vertex list and remap the boundaries so `from_faces` builds the true topology.
	let used: HashSet<u32> = faces.iter().flat_map(|f| f.boundary.iter().copied()).collect();
	if used.len() < verts.len() {
		let mut remap = vec![u32::MAX; verts.len()];
		let mut compact: Vec<DVec3> = Vec::with_capacity(used.len());
		for (i, p) in verts.iter().enumerate() {
			if used.contains(&(i as u32)) {
				remap[i] = compact.len() as u32;
				compact.push(*p);
			}
		}
		for f in faces.iter_mut() {
			for v in f.boundary.iter_mut() {
				*v = remap[*v as usize];
			}
		}
		verts = compact;
	}

	let mut solid = Solid::from_faces(verts, faces);
	solid.provenance = provenance;
	solid
}

/// Below this deviation a seam vertex counts as already ON the exact intersection
/// and is left untouched — so constructed-exact geometry (primitive rims, fillet
/// bands, machine-exact unions) is never perturbed and stays bit-identical.
const SNAP_ALREADY_EXACT: f64 = 1e-9;
/// Newton convergence tolerance (per-surface |signed distance|) for the seam snap.
/// The projector accepts up to 10× this after its iteration cap, so every snapped
/// vertex lies on each of its surfaces to ≤ 1e-11 — well inside test gates at 1e-9.
const SNAP_TOL: f64 = 1e-12;
/// Wedge cross products smaller than this (squared, mm⁴) carry no usable
/// orientation: the wedge is degenerate-collinear and the flip guard of
/// [`snap_seam_vertices`] has nothing to preserve there.
const SNAP_WEDGE_DEGENERATE: f64 = 1e-18;

/// Snap cut-seam vertices onto the exact surface–surface intersection of the
/// operands' analytic surfaces — including, since W5, seams whose snap WARPS
/// the incident curved facets.
///
/// Co-refinement places seam vertices on the operands' tessellation chords: for a
/// curved operand that is off the true intersection curve by up to the facet
/// sagitta (~1.7e-2 mm on a default 24-gon, radius-2 cylinder). Each result vertex
/// whose incident faces carry exactly TWO distinct analytic surfaces is
/// Newton-projected onto `f = g = 0` with the same min-norm projector as the SSI
/// tracer ([`crate::ssi::project`]); a vertex on exactly THREE distinct surfaces
/// (a seam corner — e.g. a cut plane pair crossing a curved wall, or a cap rim
/// meeting a quadric∩quadric seam) is solved fully determined
/// ([`crate::ssi::project3`]). Plane∩plane-only vertices are already exact and
/// are skipped, as are vertices on a single surface or on 4+ (ambiguous).
///
/// **W3 → W5.** W3's load-bearing *planarity contract* accepted a move only if
/// every incident facet stayed exactly planar (seam corners on cylinder
/// generators: ⟂ cuts, axis-parallel keyways/notches), because a WARPED polygon
/// could fold under projection-plane ear-clipping in the next boolean of a chain
/// — folded, double-covered triangles whose surplus directed edges cannot
/// twin-pair (fuzz-measured: unrestricted snapping 88.5%). W5 removes the fold
/// at its root: a warped curved-tagged face is now clipped in its surface's
/// PARAMETER SPACE ([`SurfaceChart`] via [`face_clip_p2`]), where a boundary
/// that bounds a simple region of the surface cannot fold. The guards became:
///
/// - **chart guard** ([`chart_snap_safe`]) instead of the planarity guard for
///   every face whose surface the triangulator owns ([`snap_chart_surface`]):
///   the proposed ring must chart inside the injective domain, stay simple,
///   keep its area orientation, and not be (or become) thinner than the heal
///   scale — inflating a zero-thickness sliver face into a finite fin
///   self-overlaps the boundary (fuzz seed 83894724543990). Planes keep the
///   planarity + 3-D flip guards verbatim (the intersection curve lies IN every
///   incident plane, so true seam moves cost planes nothing);
/// - **seam coherence**, all-or-nothing per surface PAIR: a seam may move only
///   if every vertex on it snaps, already lies on the exact intersection
///   (≤ 2×[`TJUNCTION_EPS`] per surface), or is a mid-edge chord sample whose
///   host stretches stay straight under the proposed moves. Vertex-level
///   accept/reject otherwise leaves a seam HALF-snapped — a zigzag between the
///   true curve and the chords whose two sides overlap in an unclassifiable
///   near-zero-thickness fin;
/// - the conservative W3 **move budgets stay in force for sphere/cone/torus
///   vertices** (≤ a tenth of the shortest incident boundary edge, wedge-margin
///   alternative for 3-surface corners): their facet sagitta is 10–20× a
///   cylinder's, and freeing it measurably breaks chains (deep N=2000: 99.9%
///   with budget-free spheres, 100.0% with this split). Plane/cylinder-only
///   vertices snap with the full sagitta-scale budget (≤ 2× the smallest
///   incident curved chord band), and their straight-seam chord samples snap
///   WITH their seam — a plane-anchored chord rim still never bends mid-edge
///   (the θ-rectangular `exact_volume` premise).
///
/// Unchanged phase-1 gates: the vertex lies within each incident surface's
/// measured chord band (it IS a seam vertex of those surfaces); it is off the
/// intersection by more than [`SNAP_ALREADY_EXACT`] and the move exceeds the
/// weld/heal noise scale (2× [`TJUNCTION_EPS`] — nudging by ~1e-7..1e-6 only
/// creates clusters the welder cannot merge); Newton must converge (tangential
/// contact ⇒ parallel gradients ⇒ skip). All rejections run in whole-set sweeps
/// to a fixed point — deterministic, and each rejection re-checks the
/// neighbourhood it re-exposes. Only vertex *positions* change — ids and
/// topology are untouched, so a closed manifold result stays closed and
/// manifold. Chain-fuzz measured at every step (deep N=2000 ×3 byte-identical,
/// floors 98): the full W5 variant table is in `ROBUSTNESS.md`.
fn snap_seam_vertices(verts: &mut [DVec3], faces: &[FaceInput]) {
	// Distinct surfaces across the result (geometric dedup) with their measured
	// chord bands; `face_surf` maps each face to its surface index.
	let mut surfs: Vec<Surface> = Vec::new();
	let mut bands: Vec<f64> = Vec::new();
	let mut face_surf: Vec<usize> = Vec::with_capacity(faces.len());
	let mut any_curved = false;
	for f in faces {
		let si = match surfs.iter().position(|s| s.same_locus(&f.surface, 1e-9)) {
			Some(i) => i,
			None => {
				surfs.push(f.surface);
				bands.push(0.0);
				surfs.len() - 1
			}
		};
		face_surf.push(si);
		if !matches!(f.surface, Surface::Plane { .. }) {
			any_curved = true;
			let n = f.boundary.len();
			for i in 0..n {
				let mid = (verts[f.boundary[i] as usize] + verts[f.boundary[(i + 1) % n] as usize]) * 0.5;
				bands[si] = bands[si].max(f.surface.signed_value(mid).abs());
			}
		}
	}
	if !any_curved {
		return; // purely planar arrangement — every seam is already exact
	}
	// Distinct incident surfaces per vertex; the corner test (a vertex within
	// TJUNCTION_EPS of the segment joining its ring neighbours on ANY incident
	// boundary is a chord sample, not a corner); and the two pre-snap move
	// budgets: the shortest incident boundary edge and the wedge margin (the
	// smallest distance to the line through the ring neighbours — the distance
	// at which the wedge `(prev, v, next)` would invert).
	let mut incident: Vec<Vec<usize>> = vec![Vec::new(); verts.len()];
	let mut mid_edge_sample = vec![false; verts.len()];
	let mut min_edge = vec![f64::INFINITY; verts.len()];
	let mut wedge_margin = vec![f64::INFINITY; verts.len()];
	// Ring-neighbour pairs of every mid-edge sample — the endpoints of the
	// straight stretches it subdivides, for the seam-coherence "stretch stays
	// straight" test below (filled in a second pass once the flags are known).
	let mut stretch_hosts: Vec<Vec<(u32, u32)>> = vec![Vec::new(); verts.len()];
	for (fi, f) in faces.iter().enumerate() {
		let n = f.boundary.len();
		for (i, &v) in f.boundary.iter().enumerate() {
			let list = &mut incident[v as usize];
			if !list.contains(&face_surf[fi]) {
				list.push(face_surf[fi]);
			}
			let prev = f.boundary[(i + n - 1) % n];
			let next = f.boundary[(i + 1) % n];
			if prev == v || next == v {
				continue;
			}
			let (pv, pa, pb) = (verts[v as usize], verts[prev as usize], verts[next as usize]);
			min_edge[v as usize] = min_edge[v as usize].min((pb - pv).length()).min((pa - pv).length());
			let dir = pb - pa;
			let margin = if dir.length_squared() > 1e-24 {
				(pv - pa).cross(dir).length() / dir.length()
			} else {
				(pv - pa).length()
			};
			wedge_margin[v as usize] = wedge_margin[v as usize].min(margin);
			if prev != next && dist_point_segment(pv, pa, pb) <= TJUNCTION_EPS {
				mid_edge_sample[v as usize] = true;
			}
		}
	}
	for f in faces.iter() {
		let n = f.boundary.len();
		for (i, &v) in f.boundary.iter().enumerate() {
			if !mid_edge_sample[v as usize] {
				continue;
			}
			let prev = f.boundary[(i + n - 1) % n];
			let next = f.boundary[(i + 1) % n];
			if prev != v && next != v {
				stretch_hosts[v as usize].push((prev, next));
			}
		}
	}
	// Phase 1 — per-vertex Newton targets through every gate except the
	// planarity/flip guards (those need the full proposed configuration).
	let mut target: Vec<Option<DVec3>> = vec![None; verts.len()];
	let mut any_target = false;
	for (vi, list) in incident.iter().enumerate() {
		// A snappable seam vertex sits on two or three distinct surfaces, every
		// curved one of a kind the parameter-space triangulator owns
		// ([`snap_chart_surface`]). All-plane vertices are already exact; 4+
		// surfaces are ambiguous. Since W5 a vertex needs NO plane among its
		// surfaces (quadric∩quadric seams — cylinder∪cylinder — snap too): the
		// warped facets a plane-less snap leaves behind are exactly what the
		// chart triangulator exists to clip.
		let n_planes = list.iter().filter(|&&i| matches!(surfs[i], Surface::Plane { .. })).count();
		let kinds_ok = list
			.iter()
			.all(|&i| matches!(surfs[i], Surface::Plane { .. } | Surface::Cylinder { .. }) || snap_chart_surface(&surfs[i]));
		if !(list.len() == 2 || list.len() == 3)
			|| (n_planes == 0 && !SNAP_ALLOW_PLANELESS)
			|| n_planes == list.len()
			|| !kinds_ok
		{
			continue;
		}
		// Vertices whose surfaces are all planes/CYLINDERS take the FULL W5
		// relaxation: the phase-2 guards judge their post-move configuration
		// exactly (the move lies in each incident plane, so planarity is free;
		// cylinder faces get the chart guard), so the W3-era pre-filters below
		// relax for them. Sphere/cone/torus vertices keep the W3 budgets even
		// though their faces are chart-clipped: their facet sagitta is typically
		// 10–20× a cylinder's (a 16×12 fuzz sphere's band is ~0.3 mm), and
		// fuzz-measured at that scale the freed moves interleave neighbouring
		// faces faster than weld/T-junction healing can absorb (deep N=2000:
		// budget-free spheres 99.9%, this split 100.0% — see ROBUSTNESS.md W5).
		let full_relax = list
			.iter()
			.all(|&i| matches!(surfs[i], Surface::Plane { .. } | Surface::Cylinder { .. }));
		// Chord samples merely subdivide a straight facet-boundary stretch and stay
		// put where a PLANE anchors the seam: the chord rim is what the
		// θ-rectangular bulge corrections of `exact_volume` integrate over (see the
		// doc above). On a pure-CYLINDER seam (quadric∩quadric) there is no exact
		// rectangular patch to protect — and leaving a sample at chord∩chord depth
		// while its seam corners snap would zigzag the seam, which the coherence
		// rule below then (rightly) vetoes wholesale — so there the samples snap
		// WITH their seam.
		if mid_edge_sample[vi] && (n_planes > 0 || !full_relax) {
			continue;
		}
		let p = verts[vi];
		let mut curved_band = f64::INFINITY;
		let mut max_dev = 0.0f64;
		let mut in_band = true;
		for &i in list {
			let dev = surfs[i].signed_value(p).abs();
			in_band &= dev <= bands[i].max(TJUNCTION_EPS);
			if !matches!(surfs[i], Surface::Plane { .. }) {
				curved_band = curved_band.min(bands[i]);
			}
			max_dev = max_dev.max(dev);
		}
		if !in_band || max_dev <= SNAP_ALREADY_EXACT {
			continue;
		}
		let proj = match list.as_slice() {
			[a, b] => crate::ssi::project(&surfs[*a], &surfs[*b], p, SNAP_TOL),
			[a, b, c] => crate::ssi::project3(&surfs[*a], &surfs[*b], &surfs[*c], p, SNAP_TOL),
			_ => None,
		};
		if let Some(s) = proj {
			// Pre-snap move budget. (a) ≤ 2× the smallest incident CURVED band: the
			// move bends each curved facet by at most ~2× the sagitta its analytic
			// tag already promises, and a glancing intersection cannot yank the
			// vertex (the plane faces are warp-free — the intersection curve lies IN
			// every incident plane). (b) for a vertex whose curved surfaces are NOT
			// all chart-owned, additionally ≤ a tenth of the shortest incident
			// boundary edge, so no fragment is bent at its own scale; a
			// fully-determined THREE-surface corner (project3, a genuine turn on
			// every ring) may alternatively spend half its wedge margin — the
			// distance at which a wedge would invert — which is what admits a corner
			// sitting next to a short co-refinement stub edge (the keyway corner).
			// A plane/cylinder-only vertex needs no edge pre-cap: bending a fragment
			// at its own scale is dangerous exactly when the bend must stay planar,
			// and the chart/flip guards of phase 2 judge any inversion the move
			// could cause exactly, face by face (a quadric∩quadric seam vertex
			// routinely needs its full sagitta-scale move next to a short
			// seam-polyline edge).
			let edge_budget = 0.1 * min_edge[vi];
			let budget = if full_relax {
				(2.0 * curved_band).min(SNAP_MOVE_CAP)
			} else {
				(2.0 * curved_band).min(if list.len() == 3 {
					edge_budget.max(0.5 * wedge_margin[vi])
				} else {
					edge_budget
				})
			};
			// Also skip MICRO-moves below the weld/heal scale: a vertex < ~1e-6 off
			// the exact intersection is co-refinement/heal noise, and nudging it
			// creates near-coincident vertex clusters (1e-7..1e-6 apart) that the
			// welder (1e-7) cannot merge and T-junction healing (4e-7) cannot
			// close — sliver micro-holes in the next boolean of a chain. Genuine
			// chord-seam vertices sit ≥ ~1e-3 off; nothing of value is skipped.
			let move_len = (s - p).length();
			if move_len <= budget && move_len > 2.0 * TJUNCTION_EPS {
				target[vi] = Some(s);
				any_target = true;
			}
		}
	}
	if !any_target {
		return;
	}
	// SEAM COHERENCE groups (W5): all vertices lying on the seam of one surface
	// PAIR, keyed by the sorted surface-index pair (only pairs with a curved
	// member — plane∩plane is always exact). A pair's seam may move only
	// ALL-OR-NOTHING: if any vertex on it can neither snap (no surviving target)
	// nor already sits on the exact intersection to within the weld/heal scale,
	// every target on that seam is dropped. Without this, vertex-level
	// accept/reject left seams PARTIALLY snapped — a zigzag polyline between the
	// true curve and the chords — and the faces on the two sides of the zigzag
	// (one following the snapped corners, one still holding unmoved chord-depth
	// junction vertices, e.g. where a third face crosses the seam mid-line)
	// OVERLAP in a near-zero-thickness fin that the NEXT boolean's arrangement
	// cannot classify (fuzz seed 83894724543990, op 4 — a 4-half-edge
	// non-manifold sandwich along the snapped generator).
	let mut pair_groups: Vec<((usize, usize), Vec<usize>)> = {
		let mut map: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
		for (vi, list) in incident.iter().enumerate() {
			for i in 0..list.len() {
				for j in i + 1..list.len() {
					let (a, b) = (list[i].min(list[j]), list[i].max(list[j]));
					if matches!(surfs[a], Surface::Plane { .. }) && matches!(surfs[b], Surface::Plane { .. }) {
						continue;
					}
					map.entry((a, b)).or_default().push(vi);
				}
			}
		}
		map.into_iter().collect()
	};
	pair_groups.sort_unstable_by_key(|(k, _)| *k);
	// On the exact (a, b) intersection to within the scale weld + T-junction
	// healing already absorb — the same floor the phase-1 micro-move skip uses,
	// so a skipped near-exact vertex never vetoes its seam.
	let near_pair = |vi: usize, a: usize, b: usize, verts: &[DVec3]| -> bool {
		let p = verts[vi];
		surfs[a].signed_value(p).abs() <= 2.0 * TJUNCTION_EPS && surfs[b].signed_value(p).abs() <= 2.0 * TJUNCTION_EPS
	};
	// Phase 2 — face guards (planarity/flip, or the chart guard for faces the
	// parameter-space triangulator owns) + seam coherence, iterated to a fixed
	// point. All are evaluated with every surviving candidate at its proposed
	// target (so a seam snapping coherently is judged as a whole); a violating
	// face/wedge/pair rejects ALL its moved members, and rejection can re-expose
	// a neighbour to the pre-snap position, so sweep until clean. The candidate
	// set only shrinks, so this terminates; sweeps are whole-set per pass and
	// faces/boundaries are deterministic, so the outcome is run-deterministic.
	loop {
		let pos = |v: u32| target[v as usize].unwrap_or(verts[v as usize]);
		// A mid-edge chord SAMPLE is coherent with its seam exactly when every
		// straight stretch it subdivides STAYS straight under the proposed moves
		// (its hosts' endpoints unmoved, or moved along the stretch): then it is
		// construction geometry (a facet-chord interior point — e.g. a cap-rim
		// chord sample at the expected sagitta depth), not a stranded seam
		// vertex. If a host stretch BENDS away from it, the sample would be left
		// as a zigzag spike off the snapped seam — the partial-snap fin — and it
		// must veto.
		let stretch_stays = |vi: usize| -> bool {
			!stretch_hosts[vi].is_empty()
				&& stretch_hosts[vi]
					.iter()
					.all(|&(a, b)| dist_point_segment(verts[vi], pos(a), pos(b)) <= TJUNCTION_EPS)
		};
		let mut reject: Vec<u32> = Vec::new();
		for ((a, b), members) in &pair_groups {
			if members
				.iter()
				.all(|&vi| target[vi].is_some() || near_pair(vi, *a, *b, verts) || stretch_stays(vi))
			{
				continue;
			}
			reject.extend(members.iter().copied().filter(|&vi| target[vi].is_some()).map(|vi| vi as u32));
		}
		for f in faces.iter() {
			let n = f.boundary.len();
			let f_moved: Vec<u32> = f
				.boundary
				.iter()
				.copied()
				.filter(|&m| target[m as usize].is_some())
				.collect();
			if f_moved.is_empty() {
				continue;
			}
			let prop: Vec<DVec3> = f.boundary.iter().map(|&v| pos(v)).collect();
			let cur: Vec<DVec3> = f.boundary.iter().map(|&v| verts[v as usize]).collect();
			// A face whose surface the PARAMETER-SPACE triangulator owns may leave
			// its chord plane — every later clip of it runs in its surface chart
			// ([`SurfaceChart`]), where warp is invisible. The planarity/flip guards
			// are replaced by the chart guard: the proposed ring must stay
			// clip-safe in that chart (every vertex inside the injective domain,
			// every wedge keeping its 2-D orientation, no self-crossing). W3's
			// planarity contract remains in force verbatim for every face whose
			// surface the chart does NOT own (planes stay true planes).
			if snap_chart_surface(&f.surface) {
				if !chart_snap_safe(&f.surface, &cur, &prop) {
					reject.extend(f_moved.iter().copied());
				}
				continue;
			}
			// PLANARITY guard: with every candidate at its proposed target, the face
			// must stay as planar as it was (within round-off), so the result remains
			// a true planar arrangement. ⟂ and axis-parallel cuts pass — their seam
			// vertices land on cylinder GENERATORS, and a facet with corners on two
			// generators is exactly planar. An oblique or partial-depth cut whose
			// seam endpoint lands mid-facet would WARP the facet by up to the
			// sagitta; ear-clipping a warped polygon in a projection plane can fold
			// (double-cover) and the next boolean in a chain then stitch-explodes —
			// the measured post-snap fuzz failure class. Such snaps are rejected;
			// those seams keep their chord vertices (the stated residual contract).
			let planarity = |poly: &[DVec3]| -> f64 {
				let nrm = newell_normal(poly);
				if nrm.length_squared() < 0.5 {
					return f64::INFINITY;
				}
				poly.iter().map(|&p| (p - poly[0]).dot(nrm).abs()).fold(0.0, f64::max)
			};
			if planarity(&prop) > planarity(&cur) + 10.0 * SNAP_ALREADY_EXACT {
				reject.extend(f_moved.iter().copied());
				continue;
			}
			// FLIP guard: no boundary wedge whose corner or neighbours moved may
			// reverse its cross-product orientation (an inverted sliver is sliver
			// debris for the next boolean even inside a still-planar face).
			for (i, &v) in f.boundary.iter().enumerate() {
				let prev = f.boundary[(i + n - 1) % n];
				let next = f.boundary[(i + 1) % n];
				let moved: Vec<u32> = [prev, v, next]
					.into_iter()
					.filter(|&m| target[m as usize].is_some())
					.collect();
				if moved.is_empty() {
					continue;
				}
				let (a0, v0, b0) = (verts[prev as usize], verts[v as usize], verts[next as usize]);
				let w0 = (v0 - a0).cross(b0 - v0);
				if w0.length_squared() <= SNAP_WEDGE_DEGENERATE {
					continue; // collinear pre-snap: no orientation to preserve
				}
				let w1 = (pos(v) - pos(prev)).cross(pos(next) - pos(v));
				if w0.dot(w1) <= 0.0 {
					reject.extend(moved);
				}
			}
		}
		if reject.is_empty() {
			break;
		}
		for v in reject {
			target[v as usize] = None;
		}
	}
	for (vi, t) in target.into_iter().enumerate() {
		if let Some(s) = t {
			verts[vi] = s;
		}
	}
}

/// The curved surface kinds whose snapped (warped) facets the W5
/// **parameter-space triangulator** owns end to end — [`face_clip_p2`] /
/// `tessellate_curved_verbatim` clip such faces in their [`SurfaceChart`], so
/// [`snap_seam_vertices`] may let them leave their chord planes (the chart guard
/// [`chart_snap_safe`] replaces W3's planarity guard there). Admission is
/// per-kind and fuzz-measured on the deep N=2000 corpus ×3 (see the W5 variant
/// table in `ROBUSTNESS.md`); a kind that cannot hold the 100.0% rate stays out
/// and keeps the W3 chord contract.
fn snap_chart_surface(s: &Surface) -> bool {
	// All four analytic curved kinds: each held the deep corpus at 100.0% ×3
	// (cones/tori enter the corpus only through fillet bands; their admission
	// additionally rests on the chart unit tests and the in-tree torus/fillet
	// suites). `false` for a kind would restore W3's planarity guard for its
	// faces — the honest fallback if a kind ever fails to hold the rate.
	matches!(
		s,
		Surface::Cylinder { .. } | Surface::Sphere { .. } | Surface::Cone { .. } | Surface::Torus { .. }
	)
}

/// Whether seam vertices with NO plane among their surfaces (quadric∩quadric —
/// e.g. cylinder∪cylinder) may snap. Requires the parameter-space triangulator:
/// a plane-less snap warps EVERY incident facet.
const SNAP_ALLOW_PLANELESS: bool = true;

/// Absolute cap (mm) on a budget-free (plane/cylinder-only) seam move — the same
/// absolute-tolerance convention as EPS/WELD_EPS/TJUNCTION_EPS. The chart guards
/// judge each FACE exactly, but a sagitta-scale move also interleaves the warped
/// face with its NEIGHBOURS, and beyond ~this depth the weld (1e-7) and
/// T-junction healing (4e-7) of the next boolean measurably stop absorbing the
/// disagreement: a coarse 11-gon r=7.5 fuzz cylinder (sagitta 0.30 mm) regressed
/// the Level-9 N=10 000 corpus until capped, while every practical tessellation
/// this kernel emits (24-gon at r ≤ ~25 mm, sagitta ≤ 0.05) snaps in full.
/// Seams needing larger moves keep their chords — the honest boundary.
const SNAP_MOVE_CAP: f64 = 0.05;

/// Whether the proposed (post-snap) boundary of a chart-owned curved face stays
/// safe to ear-clip in its surface's parameter space — the W5 replacement for
/// the planarity guard on such faces. Anchored on the CURRENT ring (`cur` never
/// changes between fixed-point sweeps, so the chart — and hence the verdict
/// sequence — is deterministic). Four conditions, all conservative:
///
/// - the chart exists and maps **every** current and proposed vertex inside its
///   injective domain (a vertex at the gnomonic horizon / on an axis refuses);
/// - the CURRENT ring is not thinner than the heal scale (chart area ≤ perimeter
///   × [`TJUNCTION_EPS`] — e.g. the zero-thickness sliver face bridging two
///   differently-subdivided copies of one seam line). Such a ring carries no
///   orientation to judge a move by, and moving its corners INFLATES the sliver
///   into a finite-width fin overlapping its neighbour faces — a
///   self-overlapping boundary that explodes the NEXT boolean's arrangement
///   (fuzz seed 83894724543990): a snap may realign a face's vertices on its
///   surface, never grow new area. The PROPOSED ring must clear the same bar
///   (a move may not thin a real face into heal-scale debris);
/// - the proposed ring keeps the current ring's signed-area ORIENTATION (a ring
///   turning inside-out would flip the face winding);
/// - the proposed ring stays **simple** — no proper crossing between
///   non-adjacent edges (O(n²) over short boundaries). Ear-clipping triangulates
///   any simple ring exactly, so individual wedges are free to change convexity
///   — a chord-polyline wedge at the sagitta scale legitimately REVERSES when
///   its vertices land on the true intersection curve, which is why this guard
///   deliberately has no per-wedge flip condition (W3's 3-D flip guard stays in
///   force for non-chart faces, where an inverted sliver inside a still-planar
///   face is real debris).
fn chart_snap_safe(surface: &Surface, cur: &[DVec3], prop: &[DVec3]) -> bool {
	let Some(chart) = SurfaceChart::new(surface, cur) else {
		return false;
	};
	let (Some(uv_cur), Some(uv_prop)) = (chart.uv_ring(cur), chart.uv_ring(prop)) else {
		return false;
	};
	let n = uv_prop.len();
	let ring_area2_perim = |uv: &[glam::DVec2]| {
		let (mut area2, mut perim) = (0.0, 0.0);
		for i in 0..n {
			let (a, b) = (uv[i], uv[(i + 1) % n]);
			area2 += a.perp_dot(b);
			perim += (b - a).length();
		}
		(area2, perim)
	};
	let (cur_area2, cur_perim) = ring_area2_perim(&uv_cur);
	let (prop_area2, prop_perim) = ring_area2_perim(&uv_prop);
	if cur_area2.abs() * 0.5 <= cur_perim * TJUNCTION_EPS
		|| prop_area2.abs() * 0.5 <= prop_perim * TJUNCTION_EPS
		|| cur_area2 * prop_area2 <= 0.0
	{
		return false;
	}
	// Simplicity of the proposed ring: no proper (interior×interior) crossing
	// between non-adjacent edges.
	for i in 0..n {
		let (a, b) = (uv_prop[i], uv_prop[(i + 1) % n]);
		for j in i + 1..n {
			if j == i || (j + 1) % n == i || (i + 1) % n == j {
				continue;
			}
			let (c, d) = (uv_prop[j], uv_prop[(j + 1) % n]);
			let o = |p: glam::DVec2, q: glam::DVec2, r: glam::DVec2| orient2d([p.x, p.y], [q.x, q.y], [r.x, r.y]);
			if o(a, b, c) * o(a, b, d) < 0.0 && o(c, d, a) * o(c, d, b) < 0.0 {
				return false;
			}
		}
	}
	true
}

/// A welded soup triangle: vertex ids, face normal, provenance name, and the
/// operand face's analytic surface.
type RawTri = ([u32; 3], DVec3, FaceName, Surface);

/// Cancel coincident facets occupying the same welded triangle. A triangle that
/// appears with both orientations forms an interior membrane and is removed
/// entirely; identical-orientation duplicates collapse to a single copy. The
/// surviving triangle keeps its **original winding** (never reconstructed), so the
/// global orientation of the soup is preserved.
fn cancel_coincident(raw: &[RawTri]) -> (Vec<[u32; 3]>, Vec<DVec3>, Vec<FaceName>, Vec<Surface>) {
	// Net signed multiplicity per unordered vertex triple, plus a representative
	// triangle for each winding seen.
	let key_of = |t: [u32; 3]| -> [u32; 3] {
		let mut s = t;
		s.sort_unstable();
		s
	};
	// canonical-positive winding = the even rotations of the sorted triple.
	let is_positive = |t: [u32; 3]| -> bool {
		let mut s = t;
		s.sort_unstable();
		[[s[0], s[1], s[2]], [s[1], s[2], s[0]], [s[2], s[0], s[1]]].contains(&t)
	};
	// (net count, +winding rep, −winding rep); each rep carries its normal,
	// provenance and surface.
	type Slot = (i32, Option<RawTri>, Option<RawTri>);
	let mut acc: HashMap<[u32; 3], Slot> = HashMap::new();
	// Emit in FIRST-INSERTION key order, not HashMap iteration order: the order of
	// this soup decides every downstream face-recovery choice (region representative
	// → normal/surface/provenance tags, face order → the next chained boolean's
	// entire arrangement), so a per-instance-random drain made results flake run to
	// run (R5). The map stays for O(1) accumulation; `order` drives the output.
	let mut order: Vec<[u32; 3]> = Vec::with_capacity(raw.len());
	for &(t, n, src, surf) in raw {
		let k = key_of(t);
		let e = acc.entry(k).or_insert_with(|| {
			order.push(k);
			(0, None, None)
		});
		if is_positive(t) {
			e.0 += 1;
			e.1 = Some((t, n, src, surf));
		} else {
			e.0 -= 1;
			e.2 = Some((t, n, src, surf));
		}
	}
	let mut tris = Vec::new();
	let mut normals = Vec::new();
	let mut sources = Vec::new();
	let mut surfaces = Vec::new();
	for k in order {
		let (count, pos, neg) = acc.remove(&k).expect("every recorded key was inserted exactly once");
		// Net zero ⇒ membrane (equal opposite copies) ⇒ drop. Otherwise keep one
		// copy in the winning winding, with its original (un-reconstructed) order.
		let pick = if count > 0 { pos } else if count < 0 { neg } else { None };
		if let Some((t, n, src, surf)) = pick {
			tris.push(t);
			normals.push(n);
			sources.push(src);
			surfaces.push(surf);
		}
	}
	(tris, normals, sources, surfaces)
}

/// Recover maximal planar faces from indexed triangles. Triangles are grouped by
/// connected coplanar regions sharing an edge; each region's outer boundary is
/// extracted by walking unshared (boundary) edges. Non-simple regions fall back
/// to per-triangle faces.
///
/// Returns, parallel to the faces: each face's provenance, and the soup-triangle
/// indices a merged region face covers (a single index for a fallback triangle) —
/// so [`stitch`]'s post-healing verification can re-expand a face that fails to
/// ear-clip back into its triangles.
fn recover_faces(
	itris: &[[u32; 3]],
	normals: &[DVec3],
	sources: &[FaceName],
	surfaces: &[Surface],
	verts: &[DVec3],
) -> (Vec<FaceInput>, Vec<FaceName>, Vec<Vec<usize>>) {
	// The analytic surface a result region inherits from its source face — but only a
	// CURVED one (a plane is recovered exactly from the geometry). `None` ⇒ tag a plane.
	let curved_surface = |ti: usize| -> Option<Surface> {
		match surfaces[ti] {
			s if !matches!(s, Surface::Plane { .. }) => Some(s),
			_ => None,
		}
	};
	let n = itris.len();
	// Union-find over triangles that are coplanar and edge-adjacent.
	let mut parent: Vec<usize> = (0..n).collect();
	fn find(p: &mut [usize], mut x: usize) -> usize {
		while p[x] != x {
			p[x] = p[p[x]];
			x = p[x];
		}
		x
	}
	// Map undirected edge → triangles touching it.
	let mut edge_map: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
	for (ti, t) in itris.iter().enumerate() {
		for &(a, b) in &[(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
			let k = if a < b { (a, b) } else { (b, a) };
			edge_map.entry(k).or_default().push(ti);
		}
	}
	for tlist in edge_map.values() {
		// Merge ONLY across 2-manifold edges (exactly two incident triangles). An edge
		// used by ≥3 triangles is a non-manifold junction (e.g. a leftover fold, or a
		// wall meeting a cap along a shared line): merging across it would (a) depend on
		// which triangle happens to be `tlist[0]` — the soup order is HashMap-random, so
		// the recovered faces (and even the result's volume) varied run to run — and
		// (b) chain triangles from opposite sides of the junction into one region whose
		// boundary walk is id-simple yet geometrically self-overlapping.
		if tlist.len() == 2 && coplanar(normals[tlist[0]], normals[tlist[1]]) {
			let (ri, rj) = (find(&mut parent, tlist[0]), find(&mut parent, tlist[1]));
			if ri != rj {
				parent[ri] = rj;
			}
		}
	}
	// Bucket triangles by region.
	let mut regions: HashMap<usize, Vec<usize>> = HashMap::new();
	for ti in 0..n {
		let r = find(&mut parent, ti);
		regions.entry(r).or_default().push(ti);
	}

	// Deterministic region order. The union-find PARTITION is merge-order-independent,
	// but the surviving ROOT id per region is not: merges run in `edge_map.values()`
	// order, which is HashMap-random per run, so sorting the root keys (the previous
	// fix) still shuffled the relative region — hence result-face — order between runs
	// (R5). Each member list is filled in ascending `ti`, so its first element is the
	// region's smallest triangle index: a stable key that depends only on the
	// (deterministic) soup order. Sort by that instead of by root.
	let mut region_list: Vec<Vec<usize>> = regions.into_values().collect();
	region_list.sort_unstable_by_key(|m| m[0]);

	let mut faces = Vec::new();
	let mut provenance = Vec::new();
	let mut face_members: Vec<Vec<usize>> = Vec::new();
	for members in &region_list {
		let normal = normals[members[0]];
		// A coplanar-connected region comes from one operand's surface; take the
		// representative member's provenance for the merged face.
		let region_src = sources[members[0]];
		// A recovered boundary is only accepted if it is a single simple loop whose
		// winding can be made to agree with the region normal; otherwise we fall
		// back to per-triangle faces, which always stitch into a valid closed solid.
		// (Whether the merged polygon also EAR-CLIPS to the region's area is verified
		// after T-junction healing, in `stitch` — failures re-expand to triangles.)
		let recovered = region_boundary(members, itris)
			.filter(|b| b.len() >= 3)
			.and_then(|b| orient_boundary(b, normal, verts));
		match recovered {
			Some(boundary) => {
				let origin = verts[boundary[0] as usize];
				// A coplanar-connected region of a curved primitive is a SINGLE chord facet
				// of that surface — adjacent facets differ in normal (e.g. a 48-gon cylinder's
				// bands are 7.5° apart, far above the coplanar tolerance) so they never merge
				// here. Keep the analytic tag regardless of vertex count: a clipped bore band
				// recovers as a >4-gon (T-junctions add collinear verts) but is still one flat
				// facet, so it tessellates flat at subdivision 1 (watertight) yet adaptive
				// tessellation and exact_volume recover the true curved surface from the tag.
				let surface = curved_surface(members[0]).unwrap_or(Surface::Plane { origin, normal });
				faces.push(FaceInput { boundary, surface });
				provenance.push(region_src);
				face_members.push(members.clone());
			}
			None => {
				for &ti in members {
					let t = itris[ti];
					let origin = verts[t[0] as usize];
					// A per-triangle fallback facet is always ≤3 vertices, so a curved
					// source surface is safe to keep (it tessellates flat).
					let surface = curved_surface(ti).unwrap_or(Surface::Plane { origin, normal });
					faces.push(FaceInput { boundary: vec![t[0], t[1], t[2]], surface });
					provenance.push(sources[ti]);
					face_members.push(vec![ti]);
				}
			}
		}
	}
	(faces, provenance, face_members)
}

/// Whether ear-clipping `boundary` reproduces the region's exact triangle area —
/// the guarantee that the recovered polygon is geometrically equivalent to the
/// triangles it replaces. An id-simple walk can still DOUBLE BACK along an
/// unhealed split-line seam interior to the region (the two sides of a full-line
/// over-split carry different subdivision vertices, so their directed edges do not
/// cancel and both chains land in the walk as zero-width spurs). Such a spur
/// polygon has the right Newell area but cannot be ear-clipped — the fan fallback
/// covers the wrong geometry, which (pre-fix) made result volumes wrong AND
/// HashMap-order flaky. The area check converts that into a per-triangle fallback.
fn boundary_covers_region(boundary: &[u32], members: &[usize], itris: &[[u32; 3]], verts: &[DVec3], normal: DVec3) -> bool {
	let tri_area = |t: &[u32; 3]| {
		let (a, b, c) = (verts[t[0] as usize], verts[t[1] as usize], verts[t[2] as usize]);
		(b - a).cross(c - a).length() * 0.5
	};
	let member_area: f64 = members.iter().map(|&ti| tri_area(&itris[ti])).sum();
	let poly: Vec<DVec3> = boundary.iter().map(|&i| verts[i as usize]).collect();

	// BOTH downstream triangulators must reproduce the area: the boolean's own
	// `ear_clip` (this polygon is re-triangulated when the result enters another
	// boolean) and the tessellator's `ear_clip_ring` (volume / rendering / export
	// meshes). Their blocking predicates differ slightly, so a polygon can clip
	// cleanly in one and jam (fan-fallback garbage) in the other.
	let mut clipped: Vec<Tri> = Vec::new();
	let scratch = FaceName { operand: FaceSource::Primitive, source_face: 0 };
	ear_clip(&poly, normal, scratch, Surface::Plane { origin: poly[0], normal }, &mut clipped);
	let boolean_area: f64 = clipped.iter().map(|t| t.area()).sum();
	if (boolean_area - member_area).abs() > member_area.max(1.0) * 1e-9 {
		return false;
	}

	let mut mesh = kernel_core::mesh::Mesh::new();
	let (u, v) = perp_basis(normal);
	let p2: Vec<glam::DVec2> = poly.iter().map(|p| glam::DVec2::new(p.dot(u), p.dot(v))).collect();
	crate::tessellate::ear_clip_ring(&mut mesh, &poly, &p2, (0..poly.len()).collect(), normal);
	let mut mesh_area = 0.0;
	for t in mesh.indices.chunks_exact(3) {
		let (a, b, c) = (mesh.positions[t[0] as usize], mesh.positions[t[1] as usize], mesh.positions[t[2] as usize]);
		mesh_area += ((b - a).cross(c - a)).length() as f64 * 0.5;
	}
	// The mesh stores f32 positions, so this comparison carries ~1e-6 relative noise;
	// a jammed clip diverges by whole percents, far above the 1e-4 line.
	(mesh_area - member_area).abs() <= member_area.max(1.0) * 1e-4
}

/// Force a recovered boundary loop's winding to agree with `normal`, or return
/// `None` if the loop is geometrically degenerate (near-zero area). This makes the
/// orientation independent of which directed edges the walk happened to pick.
fn orient_boundary(boundary: Vec<u32>, normal: DVec3, verts: &[DVec3]) -> Option<Vec<u32>> {
	let poly: Vec<DVec3> = boundary.iter().map(|&i| verts[i as usize]).collect();
	let nrm = newell_normal(&poly);
	if nrm.length_squared() < 0.5 {
		return None; // degenerate / self-overlapping loop
	}
	if nrm.dot(normal) < 0.0 {
		let mut b = boundary;
		b.reverse();
		Some(b)
	} else {
		Some(boundary)
	}
}

/// Insert collinear-interior vertices into face boundary edges so that every
/// shared boundary is split identically on both incident faces (eliminating
/// T-junctions). General over any face configuration.
fn resolve_t_junctions(faces: &mut [FaceInput], verts: &[DVec3]) {
	// Candidate vertices: every vertex that appears on some face boundary.
	let mut used: Vec<u32> = faces.iter().flat_map(|f| f.boundary.iter().copied()).collect();
	used.sort_unstable();
	used.dedup();

	// Spatial hash over the candidates. The previous form scanned EVERY used
	// vertex for EVERY boundary edge — O(E·V), and at campaign scale (28k
	// faces) that is billions of point/segment tests: observed as a silent
	// 45-minute 100%-CPU quasi-hang (DRYBOX 2026-07-28, `sample` pinned 100%
	// of time here). Per edge we now visit only candidates whose grid cells
	// the edge's inflated AABB touches. Determinism is preserved EXACTLY:
	// gathered candidates are re-sorted into ascending-index order — the same
	// order the linear scan produced — before the identical filtering and the
	// identical stable sort by parameter, so the output boundaries are
	// bit-for-bit what the O(E·V) loop built (the map is only ever queried by
	// key, never iterated — the R5 lesson).
	let cell = {
		let mut lo = DVec3::splat(f64::INFINITY);
		let mut hi = DVec3::splat(f64::NEG_INFINITY);
		for &c in &used {
			let p = verts[c as usize];
			lo = lo.min(p);
			hi = hi.max(p);
		}
		let diag = (hi - lo).length().max(1e-6);
		(diag / (used.len().max(1) as f64).cbrt()).max(TJUNCTION_EPS * 8.0)
	};
	let key = |p: DVec3| ((p.x / cell).floor() as i64, (p.y / cell).floor() as i64, (p.z / cell).floor() as i64);
	let mut grid: std::collections::HashMap<(i64, i64, i64), Vec<u32>> = std::collections::HashMap::new();
	for &c in &used {
		grid.entry(key(verts[c as usize])).or_default().push(c);
	}
	let mut candidates: Vec<u32> = Vec::new();

	for f in faces.iter_mut() {
		let mut out: Vec<u32> = Vec::with_capacity(f.boundary.len());
		let n = f.boundary.len();
		for i in 0..n {
			let a = f.boundary[i];
			let b = f.boundary[(i + 1) % n];
			out.push(a);
			let pa = verts[a as usize];
			let pb = verts[b as usize];
			let ab = pb - pa;
			let len2 = ab.length_squared();
			// Degenerate-edge guard: reject only sub-weld (physically impossible
			// post-weld, so in practice repeated-id / NaN) edges. This compares
			// SQUARED length, so the old `len2 < EPS` form silently exempted every
			// edge shorter than √EPS ≈ 3.2e-5 mm from healing — 80× the healing
			// tolerance. A micro cut stub near a seam corner (~1e-5, born where a
			// seam polyline crosses both operands' facet edges close together)
			// could then never receive the OTHER operand's interior T-vertex, and
			// the sliver filter's safety argument ("a dropped sliver's apex gets
			// inserted into the neighbour's edge") broke: fuzz seed 83894724552572
			// (sphere∩sphere) left an unhealable 2.4e-5 micro-triangle hole.
			if len2 < WELD_EPS * WELD_EPS {
				continue;
			}
			// Gather candidates from the cells the edge's inflated AABB covers,
			// then restore the ascending-index order of the original scan.
			candidates.clear();
			let (elo, ehi) = (pa.min(pb) - DVec3::splat(TJUNCTION_EPS), pa.max(pb) + DVec3::splat(TJUNCTION_EPS));
			let (k0, k1) = (key(elo), key(ehi));
			for kx in k0.0..=k1.0 {
				for ky in k0.1..=k1.1 {
					for kz in k0.2..=k1.2 {
						if let Some(cs) = grid.get(&(kx, ky, kz)) {
							candidates.extend_from_slice(cs);
						}
					}
				}
			}
			candidates.sort_unstable();
			// Collect vertices strictly interior to edge a→b, ordered by parameter.
			let mut on_edge: Vec<(f64, u32)> = Vec::new();
			for &c in &candidates {
				if c == a || c == b {
					continue;
				}
				let pc = verts[c as usize];
				let t = (pc - pa).dot(ab) / len2;
				// `!(in range)` rather than `t <= EPS || t >= ...` so a non-finite `t`
				// (NaN from a degenerate zero-length edge, `len2 ≈ 0`) is rejected too.
				if !(t > EPS && t < 1.0 - EPS) {
					continue;
				}
				// Perpendicular distance to the line a→b.
				let proj = pa + ab * t;
				if (pc - proj).length() <= TJUNCTION_EPS {
					on_edge.push((t, c));
				}
			}
			on_edge.sort_by(|x, y| x.0.total_cmp(&y.0));
			for (_, c) in on_edge {
				out.push(c);
			}
		}
		// Drop any consecutive duplicate that may arise from the insertion.
		out.dedup();
		if out.len() >= 2 && out[0] == *out.last().unwrap() {
			out.pop();
		}
		// Collapse zero-width back-track spurs `… v, w, v …` (cyclically). Insertion
		// creates them when a face's boundary doubles back along a sub-tolerance-wide
		// arm — e.g. an over-split seam the region wrapped around — and a vertex of
		// one arm lands on the other arm's edge. Left in place, the boundary carries
		// both (v,w) and (w,v): a third+fourth half-edge on one undirected edge, which
		// the twin matcher cannot pair (the un-closable seam R2 exposed). The spur is
		// geometrically a zero-area sliver, so removing it is exact to `TJUNCTION_EPS`.
		collapse_backtracks(&mut out);
		if out.len() >= 3 {
			f.boundary = out;
		}
	}
}

/// Iteratively remove cyclic back-track patterns `v, w, v` from a boundary ring
/// (deleting `w` and one `v`), then consecutive duplicates, until stable.
fn collapse_backtracks(ring: &mut Vec<u32>) {
	loop {
		let n = ring.len();
		if n < 3 {
			return;
		}
		let mut collapsed = false;
		for i in 0..n {
			let prev = ring[(i + n - 1) % n];
			let next = ring[(i + 1) % n];
			if prev == next {
				// Remove the spur tip `ring[i]` and one copy of the repeated vertex.
				let (hi, lo) = if i > (i + 1) % n { (i, (i + 1) % n) } else { ((i + 1) % n, i) };
				ring.remove(hi);
				ring.remove(lo);
				collapsed = true;
				break;
			}
		}
		if !collapsed {
			return;
		}
		ring.dedup();
		if ring.len() >= 2 && ring[0] == *ring.last().unwrap() {
			ring.pop();
		}
	}
}

fn coplanar(a: DVec3, b: DVec3) -> bool {
	a.dot(b) > 0.0 && a.cross(b).length() < 1e-7
}

/// Extract the single outer boundary loop of a connected coplanar triangle region
/// by walking its boundary (singly-used directed) edges. Returns `None` if the
/// region's boundary is not a single simple loop (holes, pinches, T-junctions).
fn region_boundary(members: &[usize], itris: &[[u32; 3]]) -> Option<Vec<u32>> {
	// Count directed edges; a boundary edge appears once, an interior edge cancels
	// with its reverse.
	let mut dir_count: HashMap<(u32, u32), i32> = HashMap::new();
	for &ti in members {
		let t = itris[ti];
		for &(a, b) in &[(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
			*dir_count.entry((a, b)).or_insert(0) += 1;
		}
	}
	// Boundary directed edges: those whose reverse is absent.
	let mut next: HashMap<u32, u32> = HashMap::new();
	for (&(a, b), &c) in &dir_count {
		let rev = dir_count.get(&(b, a)).copied().unwrap_or(0);
		let net = c - rev;
		if net > 0 {
			if next.contains_key(&a) {
				return None; // branching boundary (not a simple loop)
			}
			next.insert(a, b);
		}
	}
	if next.is_empty() {
		return None;
	}
	// Walk the loop from a deterministic start (smallest vertex id) so the result
	// does not depend on HashMap iteration order.
	let start = *next.keys().min().unwrap();
	let mut loop_v = vec![start];
	let mut cur = start;
	let mut guard = 0;
	while let Some(&nx) = next.get(&cur) {
		guard += 1;
		if guard > next.len() + 1 {
			return None; // failed to close cleanly
		}
		if nx == start {
			break;
		}
		loop_v.push(nx);
		cur = nx;
	}
	if loop_v.len() != next.len() {
		return None; // disjoint loops ⇒ region has a hole; fall back
	}
	Some(loop_v)
}

#[cfg(test)]
mod tests {
	use super::*;
	use crate::build::cuboid;
	use crate::tessellate::tessellate_default;
	use crate::validate::{exact_volume, validate, volume};
	use kernel_core::math::DAffine3;

	#[test]
	fn boolean_records_per_face_provenance() {
		// Carve a corner of box A with cutter B. The result's surface is part A's
		// original faces and part B's cut walls, so `face_source` must report BOTH
		// operands — the persistent handle for re-selecting the cut faces later.
		let a = cuboid(DVec3::splat(-2.0), DVec3::splat(2.0));
		let b = cuboid(DVec3::ZERO, DVec3::splat(3.0));
		let d = difference(&a, &b);

		let sources: Vec<Option<FaceSource>> = d.faces().map(|f| d.face_source(f)).collect();
		let from_a = sources.iter().filter(|s| **s == Some(FaceSource::OperandA)).count();
		let from_b = sources.iter().filter(|s| **s == Some(FaceSource::OperandB)).count();
		assert!(
			sources.iter().all(Option::is_some) && from_a > 0 && from_b > 0,
			"every result face has provenance and both operands contribute (A={from_a}, B={from_b})"
		);
		// A primitive carries stable Primitive names (so its edges are nameable), but
		// those never leak into a boolean result — every result face traces to an operand.
		assert!(a.faces().all(|f| a.face_source(f) == Some(FaceSource::Primitive)), "a primitive's faces are named as Primitive");
		assert!(
			sources.iter().all(|s| *s == Some(FaceSource::OperandA) || *s == Some(FaceSource::OperandB)),
			"no Primitive name leaks into the boolean result"
		);
	}

	#[test]
	fn edge_name_persists_and_re_resolves_across_an_edit() {
		// An edge is named by the two faces it bounds, so a fillet/chamfer edge can be
		// rebound after an edit. Box A carved by cutter B has edges where A's faces meet
		// B's cut walls; store one such edge's name, resize, re-run, and re-select it.
		let cut = |s: f64| difference(&cuboid(DVec3::splat(-s), DVec3::splat(s)), &cuboid(DVec3::ZERO, DVec3::splat(2.0 * s)));

		let d1 = cut(2.0);
		// An edge whose two faces come from different operands (an A-face meets a B-cut).
		let mixed = d1
			.edges()
			.find(|&e| {
				d1.edge_name(e).is_some_and(|n| n.faces[0].operand != n.faces[1].operand)
			})
			.expect("an edge where operand A meets operand B");
		let name = d1.edge_name(mixed).unwrap();
		assert!(d1.edges_named(name).contains(&mixed), "an edge is among those bearing its own name");

		let d2 = cut(4.0);
		assert!(!d2.edges_named(name).is_empty(), "stored EdgeName {name:?} must re-resolve after the edit");
	}

	#[test]
	fn vertex_name_persists_and_re_resolves_across_an_edit() {
		use crate::topo::VertexName;
		// A box's +X∧+Y∧+Z corner is named by the triple of its three face names (cuboid
		// faces 5=+X, 3=+Y, 1=+Z). The stored corner name re-resolves to the corresponding
		// vertex after the box is resized — the third leg of face/edge/vertex naming.
		let corner = VertexName::new(
			FaceName { operand: FaceSource::Primitive, source_face: 5 },
			FaceName { operand: FaceSource::Primitive, source_face: 3 },
			FaceName { operand: FaceSource::Primitive, source_face: 1 },
		);
		let b1 = cuboid(DVec3::splat(-1.0), DVec3::splat(1.0));
		let v1 = b1.vertices_named(corner);
		assert_eq!(v1.len(), 1, "the corner name resolves to one vertex");
		assert!((b1.position(v1[0]) - DVec3::splat(1.0)).length() < 1e-9, "it is the (+,+,+) corner");

		let b2 = cuboid(DVec3::splat(-2.0), DVec3::splat(2.0));
		let v2 = b2.vertices_named(corner);
		assert!(
			v2.len() == 1 && (b2.position(v2[0]) - DVec3::splat(2.0)).length() < 1e-9,
			"the same name re-resolves to the resized (+,+,+) corner"
		);
	}

	#[test]
	fn cylinder_rims_carry_their_analytic_circle() {
		use crate::build::cylinder;
		use crate::geom::Curve;
		// The cylinder's two circular rims are recorded as EXACT analytic circles on their
		// edges (not just polylines) — the first curved topological edges in the B-rep,
		// the basis for exact section queries and faithful STEP export.
		let cyl = cylinder(DVec3::ZERO, DVec3::Z, 2.0, 5.0, 8);
		let circles: Vec<(DVec3, DVec3, f64)> = cyl
			.edges()
			.filter_map(|e| match cyl.edge_curve(e) {
				Some(Curve::Circle { center, normal, radius }) => Some((center, normal, radius)),
				_ => None,
			})
			.collect();
		assert_eq!(circles.len(), 16, "both rims' edges carry the circle (8 base + 8 top)");
		assert!(
			circles.iter().all(|&(_, n, r)| (r - 2.0).abs() < 1e-12 && (n - DVec3::Z).length() < 1e-12),
			"every rim edge's circle is radius 2 about +Z"
		);
		assert!(
			circles.iter().any(|&(c, ..)| c.z.abs() < 1e-12) && circles.iter().any(|&(c, ..)| (c.z - 5.0).abs() < 1e-12),
			"rims recorded at both z=0 and z=5"
		);
	}

	#[test]
	fn boolean_derives_an_analytic_seam_circle() {
		use crate::build::cylinder;
		use crate::geom::Curve;
		// A boolean rebuilds the solid from a triangle soup, so build-time edge curves are
		// lost. Where a planar face meets a curved surface ALONG that surface, the boolean
		// RE-DERIVES the exact analytic seam circle from the operands' surfaces (plane ∩
		// cylinder), so the result edge carries true circular geometry (and exports as a
		// STEP CIRCLE). Since seam snapping (2026-06-10) this covers the CUT seam too, not
		// only the surviving construction rim: the z=3 cut boundary's vertices land on the
		// true circle (chord micro-samples are stripped, corners snapped), so its edges
		// pass the on-curve test and are tagged like the z=0 rim. (Pre-snap, cut vertices
		// sat on the facet chords ~1e-4 inside the circle and the z=3 rim stayed untagged —
		// the old HONEST LIMITATION note here.)
		let cyl = cylinder(DVec3::ZERO, DVec3::Z, 2.0, 5.0, 24);
		let cutter = cuboid(DVec3::new(-3.0, -3.0, 3.0), DVec3::new(3.0, 3.0, 10.0));
		let result = difference(&cyl, &cutter);
		assert!(validate(&result).is_valid(), "cut cylinder is a valid solid: {:?}", validate(&result));

		let seam_circles: Vec<(DVec3, f64)> = result
			.edges()
			.filter_map(|e| match result.edge_curve(e) {
				Some(Curve::Circle { center, radius, .. }) => Some((center, radius)),
				_ => None,
			})
			.collect();
		let at = |z: f64| seam_circles.iter().filter(|&&(c, _)| (c.z - z).abs() < 1e-6).count();
		assert!(
			seam_circles.iter().all(|&(c, r)| (r - 2.0).abs() < 1e-6 && (c.z.abs() < 1e-6 || (c.z - 3.0).abs() < 1e-6))
				&& at(0.0) == 24
				&& at(3.0) == 24,
			"both the surviving z=0 rim and the CUT z=3 rim carry all 24 radius-2 circle edges, got {} at z=0, {} at z=3: {seam_circles:?}",
			at(0.0),
			at(3.0)
		);
	}

	#[test]
	fn cut_seam_vertices_land_on_the_true_cylinder() {
		use crate::build::cylinder;
		use crate::geom::Surface;
		// L7 seam snapping, the headline measurement. A ⟂ box cut across a cylinder
		// produces cut-seam vertices that the raw planar arrangement leaves on the facet
		// CHORDS, off the true cylinder by up to the sagitta r·(1−cos(π/segs)) ≈ 1.7e-2 —
		// `snap_seam_vertices` Newton-projects them onto the exact plane∩cylinder
		// intersection. Asserted: every vertex of every cylinder-tagged face (cut rim
		// included) is on the TRUE cylinder to ≤ 1e-9 — seven orders below the chord
		// error it replaces — the cut fragments all KEEP their analytic tags (re-tag
		// through the cut, not only through construction), and `exact_volume` of the cut
		// result is machine-exact against the closed form π r² h.
		let (r, segs) = (2.0, 24usize);
		let cyl = cylinder(DVec3::ZERO, DVec3::Z, r, 5.0, segs);
		let cutter = cuboid(DVec3::new(-3.0, -3.0, 3.0), DVec3::new(3.0, 3.0, 10.0));
		let cut = difference(&cyl, &cutter);
		let v = validate(&cut);
		let true_cyl = Surface::Cylinder { origin: DVec3::ZERO, axis: DVec3::Z, radius: r };
		let mut max_dev = 0.0f64;
		let mut n_tagged = 0;
		for f in cut.faces() {
			if matches!(cut.face(f).surface, Surface::Cylinder { .. }) {
				n_tagged += 1;
				for vid in cut.face_vertices(f) {
					max_dev = max_dev.max(true_cyl.signed_value(cut.position(vid)).abs());
				}
			}
		}
		let sagitta = r * (1.0 - (std::f64::consts::PI / segs as f64).cos());
		let exact_err = (exact_volume(&cut).abs() - std::f64::consts::PI * r * r * 3.0).abs();
		assert!(
			v.is_valid()
				&& v.euler_characteristic == 2
				&& n_tagged == segs
				&& max_dev <= 1e-9
				&& sagitta > 1e-3
				&& exact_err <= 1e-9
				&& tessellate_default(&cut).is_watertight(),
			"⟂-cut cylinder: valid {v:?}, all {segs} cut wall fragments keep Surface::Cylinder (got {n_tagged}), \
			 every curved-face vertex on the true cylinder to ≤1e-9 (got {max_dev:.3e}, vs the {sagitta:.3e} chord \
			 sagitta the snap replaces), exact_volume to ≤1e-9 of πr²h (got {exact_err:.3e}), watertight"
		);

		// The keyway corner — a fully-determined THREE-surface point (cap plane ∩ keyway
		// wall plane ∩ bore cylinder, solved by ssi::project3) — lands on the exact
		// intersection: x=±2 on the r=5 bore ⇒ y = √21, machine-exact, even though that
		// corner sits next to a short co-refinement stub edge (the wedge-margin budget).
		let blank = cuboid(DVec3::new(-20.0, -20.0, 0.0), DVec3::new(20.0, 20.0, 30.0));
		let bore = cylinder(DVec3::new(0.0, 0.0, -1.0), DVec3::Z, 5.0, 32.0, 48);
		let keyway = cuboid(DVec3::new(-2.0, 3.0, -1.0), DVec3::new(2.0, 8.0, 31.0));
		let keyed = difference(&difference(&blank, &bore), &keyway);
		// Select exactly the four bore-side corners: x = ±2, z on a cap, y on the BORE
		// side of √21 (the cap∩wall seam continues to y=8 with plane∩plane vertices
		// ABOVE √21 that legitimately do not touch the cylinder; an unsnapped chord
		// corner sits ~5.9e-3 BELOW √21, inside this filter, so a snap regression
		// still fails loudly).
		let y_true = 21.0f64.sqrt();
		let corners: Vec<f64> = (0..keyed.vertex_count() as u32)
			.map(|i| keyed.position(crate::topo::VertexId(i)))
			.filter(|p| {
				(p.x.abs() - 2.0).abs() < 1e-7
					&& p.y > y_true - 0.05
					&& p.y <= y_true + 1e-9
					&& (p.z.abs() < 1e-7 || (p.z - 30.0).abs() < 1e-7)
			})
			.map(|p| (p.y - y_true).abs())
			.collect();
		assert!(
			corners.len() == 4 && corners.iter().all(|&d| d <= 1e-9),
			"all 4 keyway∩bore corners sit at y=√21 to ≤1e-9: {corners:?}"
		);
	}

	#[test]
	fn quadric_quadric_seam_keeps_the_chord_contract() {
		use crate::build::cylinder;
		use crate::geom::Surface;
		// W5 UPGRADE of the former chord contract: a quadric∩quadric seam — two
		// perpendicular cylinders, no plane to slide in — now SNAPS onto the exact
		// surface–surface intersection (the space quartic). W3 had to reject these
		// moves: they warp the incident facets off their chord planes, and warped
		// polygons fold under projection-plane ear-clipping in the next boolean of a
		// chain. The W5 parameter-space triangulator clips warped cylinder facets in
		// their (r·θ, z) chart, where the snapped boundary stays a simple polygon —
		// so the seam can be vertex-exact AND the chain stays robust (deep-fuzz
		// measured, see ROBUSTNESS.md W5). Asserted: every seam vertex lies on BOTH
		// true cylinders to ≤ 1e-9 (it used to sit on the chords, off by up to the
		// 1.7e-2 sagitta this test once granted as the contract), the union is a
		// valid watertight genus-0 solid, and a CHAINED boolean through the warped
		// seam region — the exact W3 failure class — stays valid and watertight.
		// Seam EDGES between the vertices remain chords of the quartic (vertex-exact,
		// not arc-exact), and the seam carries no Curve tag (no conic closed form).
		let ca = cylinder(DVec3::new(0.0, 0.0, -5.0), DVec3::Z, 2.0, 10.0, 24);
		let cb = cylinder(DVec3::new(-5.0, 0.0, 0.0), DVec3::X, 1.5, 10.0, 24);
		let u = union(&ca, &cb);
		let v = validate(&u);
		let sa = Surface::Cylinder { origin: DVec3::ZERO, axis: DVec3::Z, radius: 2.0 };
		let sb = Surface::Cylinder { origin: DVec3::ZERO, axis: DVec3::X, radius: 1.5 };
		let band = 2.0 * (1.0 - (std::f64::consts::PI / 24.0).cos()); // larger sagitta
		// Seam vertices = on faces tagged with BOTH cylinders' surfaces.
		let mut on_a: Vec<u32> = Vec::new();
		let mut on_b: Vec<u32> = Vec::new();
		for f in u.faces() {
			match u.face(f).surface {
				Surface::Cylinder { axis, .. } if axis.z.abs() > 0.5 => on_a.extend(u.face_vertices(f).iter().map(|v| v.0)),
				Surface::Cylinder { .. } => on_b.extend(u.face_vertices(f).iter().map(|v| v.0)),
				_ => {}
			}
		}
		let seam: Vec<DVec3> = on_a
			.iter()
			.filter(|i| on_b.contains(i))
			.map(|&i| u.position(crate::topo::VertexId(i)))
			.collect();
		let max_dev = seam
			.iter()
			.map(|&p| sa.signed_value(p).abs().max(sb.signed_value(p).abs()))
			.fold(0.0f64, f64::max);
		// The chained op crosses the warped seam region (a box clipping the junction).
		let chained = difference(&u, &cuboid(DVec3::new(0.5, -3.0, -1.5), DVec3::new(6.0, 3.0, 1.5)));
		let vc = validate(&chained);
		assert!(
			v.is_valid()
				&& v.euler_characteristic == 2
				&& seam.len() >= 8
				&& max_dev <= 1e-9
				&& sagitta_sanity(band)
				&& tessellate_default(&u).is_watertight()
				&& vc.is_valid()
				&& tessellate_default(&chained).is_watertight(),
			"cyl∪cyl: valid genus-0 {v:?}, all {} seam vertices on BOTH true cylinders to ≤1e-9 \
			 (got {max_dev:.3e}, vs the {band:.3e} chord sagitta the W3 contract allowed), watertight, \
			 and a chained boolean through the warped seam stays valid ({vc:?}) and watertight",
			seam.len()
		);
	}

	/// The chord band a snapped seam replaces must be a REAL improvement target —
	/// guards the quadric test against accidentally trivialising its own claim.
	fn sagitta_sanity(band: f64) -> bool {
		band > 1e-3
	}

	#[test]
	fn quadric_quadric_union_volume_stays_facet_level_and_beats_faceted() {
		use crate::build::cylinder;
		// Volume side of the W5 seam snap, measured against ground truth: for the
		// perpendicular cylinder union of `quadric_quadric_seam_keeps_the_chord_contract`,
		// the true volume is V(A) + V(B) − V(A∩B) with the Steinmetz-style overlap
		// V∩ = 4∫√(r₁²−y²)·√(r₂²−y²) dy over |y| ≤ r₂, evaluated to machine accuracy
		// with Gauss–Legendre after y = r₂·sin t (the integrand becomes smooth).
		//
		// HONEST MEASUREMENT (this exact geometry, 2026-06-10): the snapped seam
		// tightens the PL boundary itself — the plain faceted volume error drops
		// 1.79 → 1.30 mm³ — but `exact_volume`'s analytic bulge corrections assume
		// θ-rectangular CHORD facets, and on the warped seam facets they now
		// partially double-count material the facet already covers: its error moves
		// 0.31 → 0.59 mm³ (0.18% → 0.35% of 170.24 mm³). Both before and after, the
		// analytic value beats the faceted one and stays facet-level — quadric∩quadric
		// volume was facet-level under the W3 chord contract too, never exact. A
		// warp-aware bulge correction lives in `validate.rs` (outside the W5
		// triangulator scope) and is the named follow-up.
		let (r1, r2) = (2.0f64, 1.5f64);
		let ca = cylinder(DVec3::new(0.0, 0.0, -5.0), DVec3::Z, r1, 10.0, 24);
		let cb = cylinder(DVec3::new(-5.0, 0.0, 0.0), DVec3::X, r2, 10.0, 24);
		let u = union(&ca, &cb);
		// 64-point Gauss–Legendre on the substituted integrand (machine-exact for
		// this smooth function; verified stable to 1e-12 against panel refinement).
		let gauss = |f: &dyn Fn(f64) -> f64, a: f64, b: f64| -> f64 {
			const N: usize = 200;
			let h = (b - a) / N as f64;
			const X: [f64; 5] = [-0.906_179_845_938_664, -0.538_469_310_105_683, 0.0, 0.538_469_310_105_683, 0.906_179_845_938_664];
			const W: [f64; 5] = [0.236_926_885_056_189, 0.478_628_670_499_366, 0.568_888_888_888_889, 0.478_628_670_499_366, 0.236_926_885_056_189];
			let mut s = 0.0;
			for p in 0..N {
				let mid = a + h * (p as f64 + 0.5);
				for k in 0..5 {
					s += W[k] * f(mid + 0.5 * h * X[k]);
				}
			}
			s * 0.5 * h
		};
		let integrand = |t: f64| {
			let y = r2 * t.sin();
			4.0 * (r1 * r1 - y * y).sqrt() * (r2 * r2 - y * y).sqrt() * r2 * t.cos()
		};
		let v_overlap = gauss(&integrand, -std::f64::consts::FRAC_PI_2, std::f64::consts::FRAC_PI_2);
		let v_true = std::f64::consts::PI * (r1 * r1 + r2 * r2) * 10.0 - v_overlap;
		let err_exact = (exact_volume(&u) - v_true).abs();
		let err_facet = (volume(&u).abs() - v_true).abs();
		assert!(
			err_facet < 1.5 && err_exact < err_facet && err_exact < 0.7,
			"cyl∪cyl volume vs quadrature ground truth {v_true:.6}: snapped-seam faceted err {err_facet:.3e} \
			 (chord baseline 1.79) and exact_volume err {err_exact:.3e} (≤0.7, beats faceted; chord baseline 0.31 \
			 — see the honest-measurement note above)"
		);
	}

	#[test]
	fn oblique_cut_seam_snaps_and_carries_the_exact_ellipse() {
		use crate::build::cylinder;
		use crate::geom::{Curve, Surface};
		// An OBLIQUE plane cut across a cylinder — the seam endpoints land mid-facet,
		// the very class W3's planarity contract had to leave on chords (the warped
		// facets folded projection-plane ear-clipping). With the W5 parameter-space
		// triangulator the seam snaps: every vertex shared between a cylinder-tagged
		// face and the tilted cut face lies on the TRUE cylinder AND the true cut
		// plane to ≤ 1e-9, the warped result re-enters a chained boolean without
		// exploding, and `attach_seam_curves` now tags snapped seam edges with the
		// exact plane∩cylinder ELLIPSE (pre-W5 the chord-bound seam stayed untagged).
		let (r, segs) = (2.0, 24usize);
		let cyl = cylinder(DVec3::ZERO, DVec3::Z, r, 10.0, segs);
		// Cutter: a big box rotated 30° about X so its bottom face cuts obliquely.
		let m = DAffine3::from_translation(DVec3::new(0.0, 0.0, 5.0))
			* DAffine3::from_rotation_x(0.5)
			* DAffine3::from_translation(DVec3::new(0.0, 0.0, 4.0));
		let cutter = cuboid(DVec3::new(-4.0, -4.0, -4.0), DVec3::new(4.0, 4.0, 4.0)).transformed(m);
		let cut = difference(&cyl, &cutter);
		let v = validate(&cut);
		let true_cyl = Surface::Cylinder { origin: DVec3::ZERO, axis: DVec3::Z, radius: r };
		// The tilted plane: normal/origin of the cutter's bottom face in world space.
		let pn = m.transform_vector3(DVec3::Z);
		let po = m.transform_point3(DVec3::new(0.0, 0.0, -4.0));
		// Seam vertices: shared between a cylinder-tagged face and the oblique plane face.
		let mut on_cyl: Vec<u32> = Vec::new();
		let mut on_plane: Vec<u32> = Vec::new();
		for f in cut.faces() {
			match cut.face(f).surface {
				Surface::Cylinder { .. } => on_cyl.extend(cut.face_vertices(f).iter().map(|v| v.0)),
				Surface::Plane { origin, normal }
					if normal.cross(pn).length() < 1e-9 && (origin - po).dot(pn).abs() < 1e-9 =>
				{
					on_plane.extend(cut.face_vertices(f).iter().map(|v| v.0));
				}
				_ => {}
			}
		}
		let seam: Vec<DVec3> = on_cyl
			.iter()
			.filter(|i| on_plane.contains(i))
			.map(|&i| cut.position(crate::topo::VertexId(i)))
			.collect();
		let max_dev = seam
			.iter()
			.map(|&p| true_cyl.signed_value(p).abs().max((p - po).dot(pn).abs()))
			.fold(0.0f64, f64::max);
		let sagitta = r * (1.0 - (std::f64::consts::PI / segs as f64).cos());
		// The snapped seam edges carry the exact analytic ellipse.
		let ellipse_edges = cut
			.edges()
			.filter(|&e| matches!(cut.edge_curve(e), Some(Curve::Ellipse { .. })))
			.count();
		let chained = difference(&cut, &cuboid(DVec3::new(0.0, -3.0, 2.0), DVec3::new(3.0, 3.0, 9.0)));
		let vc = validate(&chained);
		assert!(
			v.is_valid()
				&& !seam.is_empty()
				&& max_dev <= 1e-9
				&& sagitta_sanity(sagitta)
				&& ellipse_edges >= seam.len() / 2
				&& tessellate_default(&cut).is_watertight()
				&& vc.is_valid()
				&& tessellate_default(&chained).is_watertight(),
			"oblique cut: valid {v:?}, all {} seam vertices on the true cylinder AND the tilted plane \
			 to ≤1e-9 (got {max_dev:.3e}, vs the {sagitta:.3e} chord sagitta W3 left), {ellipse_edges} \
			 seam edges tagged with the exact ellipse, watertight, chained boolean valid ({vc:?}) and watertight",
			seam.len()
		);
	}

	#[test]
	fn sphere_plane_seam_snaps_within_w3_budgets() {
		use crate::build::sphere;
		use crate::geom::Surface;
		// A ⟂ plane cap cut of a sphere. Sphere faces are chart-owned (warps clip in
		// the gnomonic chart), but sphere VERTICES keep the W3 move budgets — their
		// facet sagitta is 10–20× a cylinder's and budget-free moves measurably break
		// chains (deep fuzz 99.9%, see ROBUSTNESS.md W5). Within those budgets this
		// cut's seam snaps whole: every vertex shared between a sphere-tagged face
		// and the cut plane lands on the TRUE sphere and the plane to ≤ 1e-9, and the
		// warped cap re-enters a chained boolean safely.
		let r = 3.0;
		let s = sphere(DVec3::ZERO, r, 16, 12);
		let cutter = cuboid(DVec3::new(-4.0, -4.0, 1.2), DVec3::new(4.0, 4.0, 4.0));
		let cut = difference(&s, &cutter);
		let v = validate(&cut);
		let true_sph = Surface::Sphere { center: DVec3::ZERO, radius: r };
		let mut on_sph: Vec<u32> = Vec::new();
		let mut on_cap: Vec<u32> = Vec::new();
		for f in cut.faces() {
			match cut.face(f).surface {
				Surface::Sphere { .. } => on_sph.extend(cut.face_vertices(f).iter().map(|v| v.0)),
				Surface::Plane { origin, normal }
					if normal.cross(DVec3::Z).length() < 1e-9 && (origin.z - 1.2).abs() < 1e-9 =>
				{
					on_cap.extend(cut.face_vertices(f).iter().map(|v| v.0));
				}
				_ => {}
			}
		}
		let seam: Vec<DVec3> = on_sph
			.iter()
			.filter(|i| on_cap.contains(i))
			.map(|&i| cut.position(crate::topo::VertexId(i)))
			.collect();
		let max_dev = seam
			.iter()
			.map(|&p| true_sph.signed_value(p).abs().max((p.z - 1.2).abs()))
			.fold(0.0f64, f64::max);
		let chained = difference(&cut, &cuboid(DVec3::new(0.5, -4.0, -0.5), DVec3::new(4.0, 4.0, 2.0)));
		let vc = validate(&chained);
		assert!(
			v.is_valid()
				&& !seam.is_empty()
				&& max_dev <= 1e-9
				&& tessellate_default(&cut).is_watertight()
				&& vc.is_valid()
				&& tessellate_default(&chained).is_watertight(),
			"sphere cap cut: valid {v:?}, all {} seam vertices on the true sphere AND the z=1.2 plane \
			 to ≤1e-9 (got {max_dev:.3e}), watertight, chained boolean valid ({vc:?}) and watertight",
			seam.len()
		);
	}

	#[test]
	fn torus_perpendicular_plane_section_is_concentric_circles() {
		use crate::geom::{Curve, Surface};
		// Torus: major R=5, minor r=2, axis +Z at the origin.
		let t = Surface::Torus { center: DVec3::ZERO, axis: DVec3::Z, major: 5.0, minor: 2.0 };
		// ⟂-axis plane through the centre (z=0) → two concentric circles, R−r=3 and R+r=7.
		let mut radii: Vec<f64> = t
			.plane_section(DVec3::ZERO, DVec3::Z)
			.iter()
			.filter_map(|c| match c {
				Curve::Circle { radius, .. } => Some(*radius),
				_ => None,
			})
			.collect();
		radii.sort_by(|a, b| a.partial_cmp(b).unwrap());
		assert_eq!(radii.len(), 2, "the midplane section is two circles");
		assert!((radii[0] - 3.0).abs() < 1e-9 && (radii[1] - 7.0).abs() < 1e-9, "radii R∓r = 3 and 7, got {radii:?}");
		// Plane tangent to the tube (z = r = 2) → one circle of radius R = 5.
		let tan = t.plane_section(DVec3::new(0.0, 0.0, 2.0), DVec3::Z);
		assert!(matches!(tan.as_slice(), [Curve::Circle { radius, .. }] if (*radius - 5.0).abs() < 1e-9), "tangent section is one R=5 circle, got {tan:?}");
		// Beyond the tube (z=3) → empty; an oblique plane → empty (quartic, unimplemented).
		assert!(t.plane_section(DVec3::new(0.0, 0.0, 3.0), DVec3::Z).is_empty(), "a plane past the tube misses it");
		assert!(t.plane_section(DVec3::ZERO, DVec3::new(1.0, 0.0, 1.0)).is_empty(), "oblique torus section is not yet closed-form");
	}

	#[test]
	fn point_on_curve_ellipse_uses_the_ellipse_equation() {
		use crate::geom::Curve;
		// Ellipse in the XY plane, semi-axes a=3 (along X), b=2 (along Y).
		let el = Curve::Ellipse { center: DVec3::ZERO, normal: DVec3::Z, u: DVec3::X, a: 3.0, b: 2.0 };
		assert!(point_on_curve(&el, DVec3::new(3.0, 0.0, 0.0)), "the +X vertex is on the ellipse");
		assert!(point_on_curve(&el, DVec3::new(0.0, 2.0, 0.0)), "the +Y vertex is on the ellipse");
		assert!(point_on_curve(&el, el.point_at(0.7)), "an arbitrary parameter point is on the ellipse");
		// Coplanar but NOT on the ellipse — the old plane-incidence-only guard wrongly accepted these.
		assert!(!point_on_curve(&el, DVec3::new(1.0, 0.0, 0.0)), "an interior coplanar point is rejected");
		assert!(!point_on_curve(&el, DVec3::new(3.0, 2.0, 0.0)), "an exterior coplanar point is rejected");
	}

	#[test]
	fn adaptive_curved_tessellation_is_watertight_and_refines() {
		use crate::build::{cone, cylinder, sphere};
		use crate::tessellate_adaptive;
		// Edge-consistent tessellation: each shared edge is subdivided ONCE and both
		// incident faces consume the identical projected polyline, so a curved solid stays
		// watertight even at high subdivision — the watertight-curved keystone (which the
		// default subdiv=1 tessellator avoids by faceting). Validate on all three quadrics,
		// and that more subdivision yields a finer (converging) mesh.
		for solid in [
			cylinder(DVec3::ZERO, DVec3::Z, 2.0, 5.0, 8),
			sphere(DVec3::ZERO, 3.0, 12, 6),
			cone(DVec3::ZERO, DVec3::Z, 2.0, 5.0, 8),
		] {
			for seg in [1usize, 3, 6] {
				assert!(tessellate_adaptive(&solid, seg).is_watertight(), "adaptive curved tessellation is watertight at edge_segments={seg}");
			}
			assert!(
				tessellate_adaptive(&solid, 6).indices.len() > tessellate_adaptive(&solid, 1).indices.len(),
				"higher subdivision yields a finer mesh"
			);
		}
	}

	#[test]
	fn multiloop_faces_build_a_valid_washer() {
		use crate::geom::Surface;
		use crate::topo::FaceLoops;
		// A square frame (washer): outer prism [-3,3]²×[0,2] with a [-1,1]² hole through it.
		// The top/bottom caps are MULTI-LOOP faces (outer square + inner hole loop), so this
		// exercises from_faces_multiloop — faces with holes. A washer has ONE through-hole,
		// so it is a genus-1 solid (χ = 0). The prerequisite topology for periodic curved faces.
		let q = |x: f64, y: f64, z: f64| DVec3::new(x, y, z);
		let positions = vec![
			q(-3., -3., 0.), q(3., -3., 0.), q(3., 3., 0.), q(-3., 3., 0.), // 0-3 outer bottom
			q(-3., -3., 2.), q(3., -3., 2.), q(3., 3., 2.), q(-3., 3., 2.), // 4-7 outer top
			q(-1., -1., 0.), q(1., -1., 0.), q(1., 1., 0.), q(-1., 1., 0.), // 8-11 inner bottom
			q(-1., -1., 2.), q(1., -1., 2.), q(1., 1., 2.), q(-1., 1., 2.), // 12-15 inner top
		];
		let pl = |o: DVec3, n: DVec3| Surface::Plane { origin: o, normal: n };
		let face = |loops: Vec<Vec<u32>>, s: Surface| FaceLoops { loops, surface: s };
		let faces = vec![
			// bottom cap (z=0, −Z): outer loop CW-from-above + inner hole CCW-from-above.
			face(vec![vec![0, 3, 2, 1], vec![8, 9, 10, 11]], pl(q(0., 0., 0.), -DVec3::Z)),
			// top cap (z=2, +Z): outer CCW + inner hole CW.
			face(vec![vec![4, 5, 6, 7], vec![12, 15, 14, 13]], pl(q(0., 0., 2.), DVec3::Z)),
			// outer walls (normals point out).
			face(vec![vec![0, 1, 5, 4]], pl(q(0., -3., 0.), -DVec3::Y)),
			face(vec![vec![1, 2, 6, 5]], pl(q(3., 0., 0.), DVec3::X)),
			face(vec![vec![2, 3, 7, 6]], pl(q(0., 3., 0.), DVec3::Y)),
			face(vec![vec![3, 0, 4, 7]], pl(q(-3., 0., 0.), -DVec3::X)),
			// inner walls (normals point INTO the hole).
			face(vec![vec![9, 8, 12, 13]], pl(q(0., -1., 0.), DVec3::Y)),
			face(vec![vec![10, 9, 13, 14]], pl(q(1., 0., 0.), -DVec3::X)),
			face(vec![vec![11, 10, 14, 15]], pl(q(0., 1., 0.), -DVec3::Y)),
			face(vec![vec![8, 11, 15, 12]], pl(q(-1., 0., 0.), DVec3::X)),
		];
		let washer = crate::topo::Solid::from_faces_multiloop(positions, faces);
		let v = validate(&washer);
		assert!(v.closed && v.manifold, "multi-loop washer is a closed manifold: {v:?}");
		assert_eq!(v.euler_characteristic, 0, "a washer (one through-hole) is genus-1, χ=0: {v:?}");
		assert_eq!(v.genus, 1, "genus 1");
		// Volume = (outer 6×6 − inner 2×2) × height 2 = 64, now that multi-loop faces
		// tessellate the hole rather than fan-filling the outer loop.
		assert!((volume(&washer) - 64.0).abs() < 1e-6, "washer volume {} should be 64", volume(&washer));
	}

	#[test]
	fn boolean_volumes_satisfy_inclusion_exclusion() {
		use crate::build::cylinder;
		// vol(A∪B) + vol(A∩B) == vol(A) + vol(B) — a fundamental set-theoretic identity that
		// catches classification / volume errors the topology check (valid genus-0) cannot.
		// Exact for planar operands; within faceting tolerance when a shared faceted curved
		// operand is involved (the same facets appear on both sides, so they cancel).
		let a = cuboid(DVec3::splat(-2.0), DVec3::splat(2.0));
		let cases: [(Solid, f64); 3] = [
			(cuboid(DVec3::new(-1.0, -1.0, -1.0), DVec3::new(3.0, 3.0, 3.0)), 1e-9),
			(cuboid(DVec3::new(0.5, -0.5, -0.5), DVec3::splat(3.0)), 1e-9),
			(cylinder(DVec3::new(0.0, 0.0, -3.0), DVec3::Z, 1.5, 6.0, 32), 1e-6),
		];
		for (b, tol) in &cases {
			let lhs = volume(&union(&a, b)) + volume(&intersection(&a, b));
			let rhs = volume(&a) + volume(b);
			assert!((lhs - rhs).abs() < *tol, "vol(A∪B)+vol(A∩B)={lhs} must equal vol(A)+vol(B)={rhs} (within {tol})");
		}
	}

	#[test]
	fn booleans_are_valid_or_empty_across_a_config_sweep() {
		use crate::build::{cylinder, sphere};
		// Deterministic robustness sweep: every union / difference / intersection of a range
		// of overlapping and disjoint primitive pairs (box, cylinder, sphere at swept offsets)
		// must be EITHER a valid closed solid (closed + manifold + genus ≥ 0) OR empty (no
		// overlap / fully consumed) — never corrupt topology. This is the invariant the
		// orphaned-vertex bug violated; the sweep guards against arrangement regressions.
		let mut checked = 0;
		for &dx in &[-3.0_f64, -1.0, 0.0, 1.0, 3.0] {
			let a = cuboid(DVec3::splat(-2.0), DVec3::splat(2.0));
			let others = [
				cuboid(DVec3::new(dx - 1.5, -1.5, -1.5), DVec3::new(dx + 1.5, 1.5, 1.5)),
				cylinder(DVec3::new(dx, 0.0, -3.0), DVec3::Z, 1.5, 6.0, 16),
				sphere(DVec3::new(dx, 0.0, 0.0), 1.8, 16, 8),
			];
			for other in &others {
				for result in [union(&a, other), difference(&a, other), intersection(&a, other)] {
					if result.face_count() == 0 {
						continue; // legitimately empty (disjoint operands / fully consumed)
					}
					let v = validate(&result);
					assert!(v.is_valid(), "a boolean at dx={dx} must be a valid solid, not {v:?}");
					checked += 1;
				}
			}
		}
		assert!(checked >= 30, "the sweep exercised many non-empty configs (got {checked})");
	}

	#[test]
	fn boolean_carries_an_uncut_curved_face_through() {
		use crate::build::cylinder;
		use crate::geom::{Curve, Surface};
		// A box poking out the cylinder's top cap leaves the ENTIRE lateral cylinder
		// surface uncut. The union must carry those facets through as Surface::Cylinder
		// (not flatten them to planes), staying a valid watertight genus-0 solid — the
		// first analytic curved face that survives a B-rep boolean.
		let cyl = cylinder(DVec3::ZERO, DVec3::Z, 2.0, 5.0, 24);
		let bx = cuboid(DVec3::new(-1.0, -1.0, 4.0), DVec3::new(1.0, 1.0, 7.0));
		let u = union(&cyl, &bx);

		let v = validate(&u);
		assert!(v.is_valid() && v.euler_characteristic == 2, "cylinder∪box is a valid genus-0 solid: {v:?}");
		assert!(tessellate_default(&u).is_watertight(), "the curved-carry union tessellates watertight");
		let ncyl = u.faces().filter(|&f| matches!(u.face(f).surface, Surface::Cylinder { .. })).count();
		assert_eq!(ncyl, 24, "all 24 uncut lateral facets keep their Surface::Cylinder tag, got {ncyl}");

		// End-to-end: the analytic cylinder survives into a section query of the RESULT —
		// a ⟂ cut below the box finds the carried cylinder's exact radius-2 circle.
		let sec = u.section_curves(DVec3::new(0.0, 0.0, 2.5), DVec3::Z);
		assert!(
			sec.iter().any(|c| matches!(c, Curve::Circle { radius, .. } if (*radius - 2.0).abs() < 1e-9)),
			"a perpendicular section of the union finds the carried cylinder's radius-2 circle, got {sec:?}"
		);
		// The carry-through is surface-agnostic: a SPHERE and a CONE likewise keep their
		// uncut analytic faces through a union (now that the orphaned-vertex topology bug is
		// fixed, a box crossing the curved surface stays a valid genus-0 solid).
		let su = union(&crate::build::sphere(DVec3::ZERO, 2.0, 16, 8), &cuboid(DVec3::new(-0.5, -0.5, 1.0), DVec3::new(0.5, 0.5, 4.0)));
		let sv = validate(&su);
		assert!(sv.is_valid() && sv.euler_characteristic == 2, "sphere∪box valid genus-0: {sv:?}");
		assert!(su.faces().any(|f| matches!(su.face(f).surface, Surface::Sphere { .. })), "uncut sphere faces keep their Surface::Sphere tag");

		let cu = union(&crate::build::cone(DVec3::ZERO, DVec3::Z, 2.0, 5.0, 24), &cuboid(DVec3::new(-0.3, -0.3, -1.0), DVec3::new(0.3, 0.3, 1.0)));
		let cv = validate(&cu);
		assert!(cv.is_valid() && cv.euler_characteristic == 2, "cone∪box valid genus-0: {cv:?}");
		assert!(cu.faces().any(|f| matches!(cu.face(f).surface, Surface::Cone { .. })), "uncut cone faces keep their Surface::Cone tag");
	}

	#[test]
	fn box_crossing_many_curved_facets_is_genus_zero() {
		use crate::build::cylinder;
		// FIXED (was a wrong-Euler bug): a box crossing many lateral facets of a cylinder.
		// Root cause was orphaned vertices — `recover_faces` merges a coplanar triangle
		// region into one face, leaving its interior vertices unreferenced; left in the
		// array they inflated V and made `validate` report a spurious genus −1. `stitch`
		// now compacts unreferenced vertices before building the solid. Swept over a range
		// of segment counts that previously failed (≥16).
		for seg in [8usize, 12, 16, 20, 24, 32, 48, 64] {
			let u = union(&cylinder(DVec3::ZERO, DVec3::Z, 2.0, 5.0, seg), &cuboid(DVec3::new(1.0, -1.0, 1.0), DVec3::new(4.0, 1.0, 4.0)));
			let v = validate(&u);
			assert!(
				v.is_valid() && v.euler_characteristic == 2,
				"cyl{seg}∪box must be a valid genus-0 solid: {v:?}"
			);
		}
		// (Watertight tessellation at very high facet counts is tracked separately — a
		// distinct near-degenerate-cut issue from the orphaned-vertex topology bug fixed here.)
	}

	#[test]
	fn section_curves_returns_exact_analytic_cross_sections() {
		use crate::build::cylinder;
		use crate::geom::Curve;
		let cyl = cylinder(DVec3::ZERO, DVec3::Z, 2.0, 5.0, 24);

		// Perpendicular cut at z=2.5 → exactly ONE analytic circle of radius 2 on the axis.
		let perp = cyl.section_curves(DVec3::new(0.0, 0.0, 2.5), DVec3::Z);
		let circles: Vec<(DVec3, f64)> = perp
			.iter()
			.filter_map(|c| match c {
				Curve::Circle { center, radius, .. } => Some((*center, *radius)),
				_ => None,
			})
			.collect();
		assert_eq!(circles.len(), 1, "a perpendicular cylinder section is one circle, got {perp:?}");
		assert!(
			(circles[0].1 - 2.0).abs() < 1e-9 && (circles[0].0 - DVec3::new(0.0, 0.0, 2.5)).length() < 1e-9,
			"the section circle is radius 2 centered at z=2.5"
		);

		// Oblique cut → an ELLIPSE with semi-minor = radius and semi-major larger.
		let obl = cyl.section_curves(DVec3::new(0.0, 0.0, 2.5), DVec3::new(0.0, 0.4, 1.0));
		assert!(
			obl.iter().any(|c| matches!(c, Curve::Ellipse { a, b, .. } if *a > *b + 1e-9 && (*b - 2.0).abs() < 1e-6)),
			"an oblique cylinder section includes an ellipse (a>b=2), got {obl:?}"
		);

		// A box cut by z=0 → the section lines of the 4 crossed side faces (caps are parallel).
		let bx = cuboid(DVec3::splat(-1.0), DVec3::splat(1.0));
		let lines = bx.section_curves(DVec3::ZERO, DVec3::Z);
		assert!(
			lines.len() == 4 && lines.iter().all(|c| matches!(c, Curve::Line { .. })),
			"a box z=0 section is 4 side-face lines, got {lines:?}"
		);
	}

	#[test]
	fn face_name_persists_and_re_resolves_across_an_edit() {
		// Topological naming: a stored `FaceName` re-selects the logical face even
		// after an upstream parameter edit re-runs the boolean — the persistent
		// reference a parametric feature needs.
		let cut = |s: f64| difference(&cuboid(DVec3::splat(-s), DVec3::splat(s)), &cuboid(DVec3::ZERO, DVec3::splat(2.0 * s)));

		let d1 = cut(2.0);
		// Within a solid a name round-trips: a face is among those bearing its name.
		let f0 = d1.faces().find(|&f| d1.face_source(f) == Some(FaceSource::OperandB)).expect("a B-sourced cut face");
		let name = d1.face_name(f0).unwrap();
		assert!(d1.faces_named(name).contains(&f0), "a face is among those bearing its own name");

		// Across an edit (the part doubled in size), the stored name still resolves to
		// the corresponding result face — it refers to input topology, not result ids.
		let d2 = cut(4.0);
		let resolved = d2.faces_named(name);
		assert!(
			!resolved.is_empty() && resolved.iter().all(|&f| d2.face_name(f) == Some(name)),
			"stored FaceName {name:?} must re-resolve after the edit (got {} faces)",
			resolved.len()
		);
	}

	#[test]
	fn face_identity_survives_a_nested_boolean() {
		// Chained provenance: in `(A∪B)−C`, a face that originated in B must still
		// trace to B through the SECOND boolean. Because the boolean carries an
		// operand's existing provenance instead of relabelling by the immediate
		// operand, B's surviving wall keeps `OperandB` — without carry-through every
		// face of the `A∪B` operand would read `OperandA`. B's +x wall (x=3) lies
		// outside A and is untouched by C, so it is the unambiguous witness: it cannot
		// come from C (whose faces sit at x∈{1,4}), and it can only read `OperandB`
		// if the inner union's provenance was carried forward.
		let a = cuboid(DVec3::splat(-2.0), DVec3::splat(2.0));
		let b = cuboid(DVec3::ZERO, DVec3::splat(3.0));
		let c = cuboid(DVec3::splat(1.0), DVec3::splat(4.0));

		let chained = difference(&union(&a, &b), &c);
		let b_wall = chained
			.faces()
			.find(|&f| chained.face_polygon(f).iter().all(|p| (p.x - 3.0).abs() < 1e-6))
			.expect("B's +x wall at x=3 survives `(A∪B)−C`");
		assert_eq!(
			chained.face_source(b_wall),
			Some(FaceSource::OperandB),
			"B's wall must keep its B-identity through the nested boolean (chained provenance)"
		);
	}

	#[test]
	fn boolean_stays_valid_far_from_the_origin() {
		// The arrangement's coincidence/weld tests use fixed absolute tolerances, so
		// without re-centring they fail once the f64 ulp grows past them (ulp ≈ 1e-8
		// at 1e8) and the result collapses. Centring the operands keeps the union of
		// two overlapping boxes a valid closed genus-0 solid arbitrarily far out.
		for &t in &[0.0_f64, 1e6, 1e8, 1e10] {
			let off = DVec3::splat(t);
			let a = cuboid(DVec3::splat(-1.0) + off, DVec3::splat(1.0) + off);
			let b = cuboid(off, DVec3::splat(2.0) + off);
			let v = validate(&union(&a, &b));
			assert!(
				v.closed && v.manifold && v.euler_characteristic == 2,
				"union at t={t:e} must be a valid genus-0 solid: closed={} manifold={} χ={}",
				v.closed, v.manifold, v.euler_characteristic
			);
		}
	}

	/// Overlap volume of two axis-aligned boxes given by their min/max corners.
	fn overlap_volume(amin: DVec3, amax: DVec3, bmin: DVec3, bmax: DVec3) -> f64 {
		let lo = amin.max(bmin);
		let hi = amax.min(bmax);
		let d = (hi - lo).max(DVec3::ZERO);
		d.x * d.y * d.z
	}

	fn box_vol(min: DVec3, max: DVec3) -> f64 {
		let d = max - min;
		d.x * d.y * d.z
	}

	#[test]
	fn union_of_overlapping_boxes_has_exact_volume() {
		let amin = DVec3::new(0.0, 0.0, 0.0);
		let amax = DVec3::new(10.0, 10.0, 10.0);
		let bmin = DVec3::new(5.0, 5.0, 5.0);
		let bmax = DVec3::new(15.0, 15.0, 15.0);
		let a = cuboid(amin, amax);
		let b = cuboid(bmin, bmax);

		let u = union(&a, &b);
		let v = validate(&u);
		let expected =
			box_vol(amin, amax) + box_vol(bmin, bmax) - overlap_volume(amin, amax, bmin, bmax);

		assert!(v.is_valid(), "union must be closed + manifold: {v:?}");
		assert!(
			tessellate_default(&u).is_watertight(),
			"union must tessellate watertight"
		);
		assert!(
			(volume(&u).abs() - expected).abs() < 1e-6,
			"union volume {} != expected {}",
			volume(&u).abs(),
			expected
		);
	}

	#[test]
	fn union_box_as_wide_as_base_sharing_side_planes_is_clean() {
		// The demo's ORIGINAL failing case: a base slab and a wall exactly as WIDE as
		// the base, so they share the x=±40 side planes (and z=0). Must be a single
		// clean genus-0 solid.
		let amin = DVec3::new(-40.0, -35.0, 0.0);
		let amax = DVec3::new(40.0, 35.0, 8.0);
		let bmin = DVec3::new(-40.0, 10.0, 0.0);
		let bmax = DVec3::new(40.0, 20.0, 50.0);
		let u = union(&cuboid(amin, amax), &cuboid(bmin, bmax));
		let v = validate(&u);
		let expected = box_vol(amin, amax) + box_vol(bmin, bmax) - overlap_volume(amin, amax, bmin, bmax);
		assert!(
			v.is_valid() && v.genus == 0 && (volume(&u).abs() - expected).abs() < 1e-6,
			"wide-wall union must be a clean genus-0 solid of volume {expected}: {v:?} vol={}",
			volume(&u).abs()
		);
	}

	#[test]
	fn union_of_boxes_stacked_face_to_face_is_one_box() {
		// Two boxes stacked so A's top face and B's bottom face are coincident (and
		// anti-aligned). Their union is a single 10×10×8 box (volume 800).
		let amin = DVec3::new(0.0, 0.0, 0.0);
		let amax = DVec3::new(10.0, 10.0, 4.0);
		let bmin = DVec3::new(0.0, 0.0, 4.0);
		let bmax = DVec3::new(10.0, 10.0, 8.0);
		let u = union(&cuboid(amin, amax), &cuboid(bmin, bmax));
		let v = validate(&u);
		assert!(
			v.is_valid() && v.genus == 0 && (volume(&u).abs() - 800.0).abs() < 1e-6,
			"face-to-face stack union must be one clean box of volume 800: {v:?} vol={}",
			volume(&u).abs()
		);
	}

	#[test]
	fn difference_with_a_coplanar_shared_face_is_clean() {
		// Cut an open slot into A's −X side: the cutter shares A's x=0, z=0 and z=10
		// face planes (and pokes out the x=0 side). The result is a clean notched box.
		let amin = DVec3::new(0.0, 0.0, 0.0);
		let amax = DVec3::new(10.0, 10.0, 10.0);
		let bmin = DVec3::new(0.0, 3.0, 0.0);
		let bmax = DVec3::new(4.0, 7.0, 10.0);
		let d = difference(&cuboid(amin, amax), &cuboid(bmin, bmax));
		let v = validate(&d);
		let expected = box_vol(amin, amax) - overlap_volume(amin, amax, bmin, bmax);
		assert!(
			v.is_valid() && (volume(&d).abs() - expected).abs() < 1e-6,
			"coplanar-face difference must be a clean solid of volume {expected}: {v:?} vol={}",
			volume(&d).abs()
		);
	}

	#[test]
	fn union_of_boxes_sharing_face_planes_is_a_clean_solid() {
		// A slab and a wall that interpenetrate AND share three face planes (x=0, x=10,
		// z=0). This is the coplanar partial-overlap case the demo exposed: the union
		// must still be a single closed, manifold, genus-0 solid (a slab with a wall).
		//
		// (task #15, FIXED — live regression test): a coplanar cutter face that extends
		// BEYOND the subject face, where the shared coplanar edge meets a transversal cut,
		// used to break the union. The coplanar partial-overlap handling now resolves it, so
		// this asserts a clean genus-0 solid rather than guarding a known failure.
		let amin = DVec3::new(0.0, 0.0, 0.0);
		let amax = DVec3::new(10.0, 10.0, 4.0);
		let bmin = DVec3::new(0.0, 3.0, 0.0);
		let bmax = DVec3::new(10.0, 7.0, 12.0);
		let u = union(&cuboid(amin, amax), &cuboid(bmin, bmax));
		let v = validate(&u);
		let expected = box_vol(amin, amax) + box_vol(bmin, bmax) - overlap_volume(amin, amax, bmin, bmax);
		assert!(
			v.is_valid() && v.genus == 0 && (volume(&u).abs() - expected).abs() < 1e-6,
			"coplanar-face union must be a clean genus-0 solid of volume {expected}: {v:?} vol={}",
			volume(&u).abs()
		);
	}

	#[test]
	fn difference_of_overlapping_boxes_has_exact_volume() {
		let amin = DVec3::new(0.0, 0.0, 0.0);
		let amax = DVec3::new(10.0, 10.0, 10.0);
		let bmin = DVec3::new(5.0, 5.0, 5.0);
		let bmax = DVec3::new(15.0, 15.0, 15.0);
		let a = cuboid(amin, amax);
		let b = cuboid(bmin, bmax);

		let d = difference(&a, &b);
		let v = validate(&d);
		let expected = box_vol(amin, amax) - overlap_volume(amin, amax, bmin, bmax);

		assert!(v.is_valid(), "difference must be closed + manifold: {v:?}");
		assert!(
			tessellate_default(&d).is_watertight(),
			"difference must tessellate watertight"
		);
		assert!(
			(volume(&d).abs() - expected).abs() < 1e-6,
			"difference volume {} != expected {}",
			volume(&d).abs(),
			expected
		);
	}

	#[test]
	fn intersection_of_overlapping_boxes_has_exact_volume() {
		let amin = DVec3::new(0.0, 0.0, 0.0);
		let amax = DVec3::new(10.0, 10.0, 10.0);
		let bmin = DVec3::new(5.0, 5.0, 5.0);
		let bmax = DVec3::new(15.0, 15.0, 15.0);
		let a = cuboid(amin, amax);
		let b = cuboid(bmin, bmax);

		let i = intersection(&a, &b);
		let v = validate(&i);
		let expected = overlap_volume(amin, amax, bmin, bmax);

		assert!(v.is_valid(), "intersection must be closed + manifold: {v:?}");
		assert!(
			tessellate_default(&i).is_watertight(),
			"intersection must tessellate watertight"
		);
		assert!(
			(volume(&i).abs() - expected).abs() < 1e-6,
			"intersection volume {} != expected {}",
			volume(&i).abs(),
			expected
		);
	}

	#[test]
	fn union_of_disjoint_boxes_keeps_both_volumes() {
		let a = cuboid(DVec3::new(0.0, 0.0, 0.0), DVec3::new(2.0, 2.0, 2.0));
		let b = cuboid(DVec3::new(5.0, 5.0, 5.0), DVec3::new(7.0, 7.0, 7.0));
		let u = union(&a, &b);
		assert!((volume(&u).abs() - 16.0).abs() < 1e-6, "two disjoint 2³ boxes: {}", volume(&u).abs());
		assert!(validate(&u).closed, "disjoint union still closed");
	}

	#[test]
	fn intersection_of_disjoint_boxes_is_empty() {
		let a = cuboid(DVec3::new(0.0, 0.0, 0.0), DVec3::new(2.0, 2.0, 2.0));
		let b = cuboid(DVec3::new(5.0, 5.0, 5.0), DVec3::new(7.0, 7.0, 7.0));
		let i = intersection(&a, &b);
		assert_eq!(i.face_count(), 0, "disjoint intersection is empty");
	}

	#[test]
	fn difference_removing_corner_is_general_nonconvex() {
		// A non-convex result: cut a smaller box out of a corner of a larger one.
		let amin = DVec3::new(0.0, 0.0, 0.0);
		let amax = DVec3::new(10.0, 10.0, 10.0);
		let bmin = DVec3::new(-1.0, -1.0, -1.0);
		let bmax = DVec3::new(4.0, 4.0, 4.0);
		let a = cuboid(amin, amax);
		let b = cuboid(bmin, bmax);
		let d = difference(&a, &b);
		let expected = box_vol(amin, amax) - overlap_volume(amin, amax, bmin, bmax);
		assert!(validate(&d).is_valid(), "corner-cut solid is valid: {:?}", validate(&d));
		assert!(
			(volume(&d).abs() - expected).abs() < 1e-6,
			"corner cut volume {} != {}",
			volume(&d).abs(),
			expected
		);
	}

	#[test]
	fn intersection_with_fully_containing_box_returns_inner_solid() {
		// Generality (not a box-on-box special case): a triangular prism wholly
		// inside a large box. A ∩ B == the prism, exactly.
		let prism = crate::build::extrude(
			&[
				glam::DVec2::new(1.0, 1.0),
				glam::DVec2::new(5.0, 1.0),
				glam::DVec2::new(2.0, 4.0),
			],
			3.0,
		);
		let big = cuboid(DVec3::new(-10.0, -10.0, -10.0), DVec3::new(20.0, 20.0, 20.0));
		let prism_vol = volume(&prism).abs();

		let i = intersection(&prism, &big);
		assert!(validate(&i).is_valid(), "prism ∩ box valid: {:?}", validate(&i));
		assert!(tessellate_default(&i).is_watertight(), "prism ∩ box watertight");
		assert!(
			(volume(&i).abs() - prism_vol).abs() < 1e-6,
			"prism ∩ containing box == prism: {} vs {}",
			volume(&i).abs(),
			prism_vol
		);
	}

	#[test]
	fn union_with_fully_contained_solid_returns_outer_volume() {
		// B ⊂ A ⇒ A ∪ B == A (in volume), for a non-axis-aligned inner prism.
		let outer = cuboid(DVec3::new(0.0, 0.0, 0.0), DVec3::new(10.0, 10.0, 10.0));
		let inner = crate::build::extrude(
			&[
				glam::DVec2::new(3.0, 3.0),
				glam::DVec2::new(7.0, 4.0),
				glam::DVec2::new(5.0, 8.0),
			],
			4.0,
		)
		.transformed(kernel_core::math::DAffine3::from_translation(DVec3::new(0.0, 0.0, 2.0)));
		let outer_vol = volume(&outer).abs();
		let u = union(&outer, &inner);
		assert!(validate(&u).is_valid(), "A ∪ (B⊂A) valid: {:?}", validate(&u));
		assert!(
			(volume(&u).abs() - outer_vol).abs() < 1e-6,
			"A ∪ contained == A: {} vs {}",
			volume(&u).abs(),
			outer_vol
		);
	}

	#[test]
	fn difference_of_general_prism_overlap_is_valid_and_volume_correct() {
		// Two overlapping triangular prisms (general planar solids, not boxes).
		// The difference volume equals V(A) minus the volume A and B share, which
		// here equals V(A) − V(A∩B); we verify the CSG identity numerically via the
		// intersection operator (independent code path computing the same overlap).
		let a = crate::build::extrude(
			&[
				glam::DVec2::new(0.0, 0.0),
				glam::DVec2::new(6.0, 0.0),
				glam::DVec2::new(3.0, 6.0),
			],
			5.0,
		);
		let b = crate::build::extrude(
			&[
				glam::DVec2::new(2.0, 2.0),
				glam::DVec2::new(8.0, 2.0),
				glam::DVec2::new(5.0, 8.0),
			],
			5.0,
		);
		let a_vol = volume(&a).abs();
		let inter_vol = volume(&intersection(&a, &b)).abs();
		let d = difference(&a, &b);
		assert!(validate(&d).is_valid(), "prism − prism valid: {:?}", validate(&d));
		assert!(tessellate_default(&d).is_watertight(), "prism − prism watertight");
		assert!(
			(volume(&d).abs() - (a_vol - inter_vol)).abs() < 1e-6,
			"V(A−B) == V(A) − V(A∩B): {} vs {}",
			volume(&d).abs(),
			a_vol - inter_vol
		);
	}

	#[test]
	fn empty_operand_is_handled_gracefully() {
		let a = cuboid(DVec3::new(0.0, 0.0, 0.0), DVec3::new(2.0, 2.0, 2.0));
		let empty = Solid::default();
		// Union/difference with empty leaves A; intersection with empty is empty.
		assert!((volume(&union(&a, &empty)).abs() - 8.0).abs() < 1e-9);
		assert!((volume(&difference(&a, &empty)).abs() - 8.0).abs() < 1e-9);
		assert_eq!(intersection(&a, &empty).face_count(), 0);
	}

	#[test]
	fn rotated_box_union_is_general_off_axis() {
		// Generality off the coordinate axes: a box rotated 30° about Z, unioned
		// with an overlapping axis-aligned box. We verify the CSG identity
		// V(A∪B) == V(A) + V(B) − V(A∩B) using the (independent) intersection path.
		use kernel_core::math::DAffine3;
		let a = cuboid(DVec3::new(0.0, 0.0, 0.0), DVec3::new(10.0, 10.0, 10.0)).transformed(
			DAffine3::from_rotation_z(30.0_f64.to_radians()),
		);
		let b = cuboid(DVec3::new(3.0, 3.0, 2.0), DVec3::new(13.0, 13.0, 8.0));
		let va = volume(&a).abs();
		let vb = volume(&b).abs();
		let vi = volume(&intersection(&a, &b)).abs();
		let u = union(&a, &b);
		let expected = va + vb - vi;
		assert!(validate(&u).is_valid(), "rotated union valid: {:?}", validate(&u));
		assert!(tessellate_default(&u).is_watertight(), "rotated union watertight");
		// Off-axis geometry carries irrational coordinates (cos/sin 30°), so the
		// agreement is to floating-point relative precision rather than the ~1e-9
		// exactness of axis-aligned planar input.
		assert!(
			(volume(&u).abs() - expected).abs() / expected < 1e-5,
			"V(A∪B) {} != V(A)+V(B)−V(A∩B) {}",
			volume(&u).abs(),
			expected
		);
	}

	#[test]
	fn self_union_is_idempotent() {
		// A ∪ A == A (volume), and the result is a valid closed solid: a stress test
		// for coincident-face handling (every face is shared/aligned).
		let a = cuboid(DVec3::new(-2.0, -2.0, -2.0), DVec3::new(2.0, 2.0, 2.0));
		let u = union(&a, &a);
		assert!(validate(&u).is_valid(), "A∪A valid: {:?}", validate(&u));
		assert!(
			(volume(&u).abs() - 64.0).abs() < 1e-6,
			"A∪A volume {} != 64",
			volume(&u).abs()
		);
	}

	// --- Loop-aware chained booleans (R2/R3, BAR Level 6) -------------------------
	//
	// Root causes fixed (2026-06-09), each previously exploding genus/shells:
	// 1. Sub-tolerance sliver triangles: re-triangulating a boolean RESULT emits
	//    near-degenerate triangles along T-junction-healed near-collinear chains;
	//    after welding, `resolve_t_junctions` folded such a triangle's own apex into
	//    its base edge ([a,b,c] → [a,b,c,b]), tripling directed edges and breaking
	//    twin pairing. `stitch` now drops sub-`TJUNCTION_EPS`-altitude slivers.
	// 2. Outer-loop-only triangulation: a face with INNER loops (extrude_with_holes
	//    cap) was triangulated as if filled. `triangulate_solid` now bridges holes
	//    into the outer ring (same algorithm as the multi-loop tessellator).
	// 3. Region merging across non-manifold (≥3-triangle) edges was HashMap-order
	//    dependent; `recover_faces` now merges across exactly-2-triangle edges only.
	// 4. Surface tags were looked up by FaceName, which COLLIDES in chained booleans
	//    (an operand that is itself a boolean carries `OperandA/B` names of ITS
	//    operands); a first bore's wall could get the second bore's cylinder and
	//    tessellate onto the wrong surface. Fragments now carry their operand face's
	//    `Surface` by value.

	/// n-gon prism cross-section area for a faceted "cylinder" of radius `r`.
	fn ngon_area(r: f64, n: usize) -> f64 {
		0.5 * n as f64 * r * r * (std::f64::consts::TAU / n as f64).sin()
	}

	#[test]
	fn second_hole_into_the_same_face_stays_valid() {
		use crate::build::cylinder;
		// R2 repro 1: drilling a SECOND hole through caps that already carry the first
		// hole's rims. Before the fix: closed=false, genus ≈ 125, shells ≈ 24. The
		// result must be a valid genus-2 solid (two through-holes) with the exact
		// faceted volume — and deterministically so (the surface-tag collision made
		// the volume flake run to run), hence the repeated runs.
		let plate = cuboid(DVec3::ZERO, DVec3::new(60.0, 40.0, 8.0));
		let hole = |x: f64, y: f64| cylinder(DVec3::new(x, y, -1.0), DVec3::Z, 3.5, 10.0, 32);
		let d1 = difference(&plate, &hole(45.0, 12.0));
		let v1 = validate(&d1);
		assert!(v1.is_valid() && v1.genus == 1, "first hole: valid genus-1 plate: {v1:?}");

		// Volume tolerance 1e-3 mm³ (relative ~5e-9): T-junction healing legitimately
		// moves seam vertices by up to TJUNCTION_EPS (4e-7) and the sliver filter drops
		// sub-tolerance gap area, so a chained result is exact only to that scale.
		let expected = 60.0 * 40.0 * 8.0 - 2.0 * ngon_area(3.5, 32) * 8.0;
		for run in 0..5 {
			let d2 = difference(&d1, &hole(45.0, 28.0));
			let v2 = validate(&d2);
			let vol = volume(&d2).abs();
			assert!(
				v2.is_valid() && v2.genus == 2 && (vol - expected).abs() < 1e-3,
				"second hole, run {run}: must be a valid genus-2 solid of volume {expected}: {v2:?} vol={vol}"
			);
		}
	}

	#[test]
	fn coplanar_union_against_a_multiloop_holed_face_is_clean() {
		use crate::build::extrude_with_holes;
		// R2 repro 2: union an upright onto a plate whose caps carry a TRUE inner
		// loop (extrude_with_holes), sharing the x=0, y=0, y=40 and z-contact planes.
		// The hole is far from the contact. Before the fix the holed caps were
		// triangulated outer-loop-only (hole filled) and the union exploded; now it
		// is a valid genus-1 solid with the exact faceted volume.
		let circle: Vec<glam::DVec2> = (0..32)
			.map(|i| {
				let a = std::f64::consts::TAU * i as f64 / 32.0;
				glam::DVec2::new(45.0 + 3.5 * a.cos(), 20.0 + 3.5 * a.sin())
			})
			.collect();
		let outer = vec![
			glam::DVec2::new(0.0, 0.0),
			glam::DVec2::new(60.0, 0.0),
			glam::DVec2::new(60.0, 40.0),
			glam::DVec2::new(0.0, 40.0),
		];
		let plate = extrude_with_holes(&outer, &[circle], 8.0);
		let upright = cuboid(DVec3::ZERO, DVec3::new(8.0, 40.0, 50.0));
		let u = union(&plate, &upright);
		let v = validate(&u);
		let expected = (60.0 * 40.0 - ngon_area(3.5, 32)) * 8.0 + 8.0 * 40.0 * 50.0 - 8.0 * 40.0 * 8.0;
		// 1e-3 mm³ tolerance: see `second_hole_into_the_same_face_stays_valid`.
		assert!(
			v.is_valid() && v.genus == 1 && (volume(&u).abs() - expected).abs() < 1e-3,
			"holed-plate ∪ upright must be a valid genus-1 solid of volume {expected}: {v:?} vol={}",
			volume(&u).abs()
		);

		// The same union where the hole was drilled by a BOOLEAN instead (the plate is
		// then a triangle-soup B-rep whose caps carry the bore rims as healed chains —
		// the other half of R2 repro 2, which also exploded before the fix).
		use crate::build::cylinder;
		let plain = cuboid(DVec3::ZERO, DVec3::new(60.0, 40.0, 8.0));
		let drilled = difference(&plain, &cylinder(DVec3::new(45.0, 20.0, -1.0), DVec3::Z, 3.5, 10.0, 32));
		let u2 = union(&drilled, &upright);
		let v2 = validate(&u2);
		assert!(
			v2.is_valid() && v2.genus == 1 && (volume(&u2).abs() - expected).abs() < 1e-3,
			"boolean-drilled plate ∪ upright must be a valid genus-1 solid of volume {expected}: {v2:?} vol={}",
			volume(&u2).abs()
		);
	}

	#[test]
	fn keyway_crossing_a_bored_wall_stays_valid() {
		use crate::build::cylinder;
		// R3: cut a keyway that crosses a previously-cut curved bore wall. Before the
		// fix: genus ≈ 204, shells ≈ 45. Volume is checked against the independent
		// intersection path: V(bored − keyway) = V(bored) − V(bored ∩ keyway).
		let blank = cuboid(DVec3::new(-20.0, -20.0, 0.0), DVec3::new(20.0, 20.0, 30.0));
		let bore = cylinder(DVec3::new(0.0, 0.0, -1.0), DVec3::Z, 5.0, 32.0, 48);
		let bored = difference(&blank, &bore);
		let vb = validate(&bored);
		assert!(vb.is_valid() && vb.genus == 1, "bored blank is a valid genus-1 solid: {vb:?}");

		let keyway = cuboid(DVec3::new(-2.0, 3.0, -1.0), DVec3::new(2.0, 8.0, 31.0));
		let keyed = difference(&bored, &keyway);
		let vk = validate(&keyed);
		// The identity is checked in the ANALYTIC measure, 1000× tighter than the old
		// 1e-3 faceted-volume gate. Every cut here is ⟂ or parallel to the bore axis, so
		// post-snap each cut bore facet is a θ-rectangular cylinder patch and the
		// exact_volume bulge corrections close the identity to round-off. The faceted
		// (tessellated) identity no longer closes tightly — and should not: seam snapping
		// (2026-06-10) lands the keyway∩bore seam ON the true cylinder, so the cut
		// results hug the bore more closely than the *uncut* operand's chord facets do.
		let expected = exact_volume(&bored).abs() - exact_volume(&intersection(&bored, &keyway)).abs();
		assert!(
			vk.is_valid() && vk.genus == 1 && (exact_volume(&keyed).abs() - expected).abs() < 1e-6,
			"keyway through bore must stay a valid genus-1 solid of exact volume {expected}: {vk:?} vol={}",
			exact_volume(&keyed).abs()
		);
	}

	#[test]
	fn chained_bolt_circle_differences_stay_valid() {
		use crate::build::cylinder;
		// R2 repro 3: six sequential bolt-hole differences into one disc — every cut
		// lands on caps already carrying the previous holes' rims. Before the fix the
		// genus walked 129 → 217 → 284 → 399 → 457 with dozens of shells; now each
		// step is a valid solid of genus k+1 and the final volume is exact.
		let mut cur = cylinder(DVec3::ZERO, DVec3::Z, 30.0, 6.0, 48);
		for k in 0..6 {
			let a = std::f64::consts::TAU * k as f64 / 6.0;
			let bolt = cylinder(DVec3::new(22.0 * a.cos(), 22.0 * a.sin(), -1.0), DVec3::Z, 2.5, 8.0, 24);
			cur = difference(&cur, &bolt);
			let v = validate(&cur);
			assert!(
				v.is_valid() && v.genus == k + 1 && v.shells == 1,
				"flange after bolt {k}: must be one valid genus-{} shell: {v:?}",
				k + 1
			);
		}
		let expected = (ngon_area(30.0, 48) - 6.0 * ngon_area(2.5, 24)) * 6.0;
		// 1e-3 mm³ tolerance: see `second_hole_into_the_same_face_stays_valid`.
		assert!(
			(volume(&cur).abs() - expected).abs() < 1e-3,
			"6-bolt flange volume {} must equal the exact faceted {expected}",
			volume(&cur).abs()
		);
	}

	#[test]
	fn degenerate_configurations_never_panic() {
		// Robustness: tricky contacts (coincident faces, edge-only / corner-only
		// touching, full coincidence, a near-zero sliver overlap) drive the
		// co-refinement into collinear and zero-length-edge situations. The kernel
		// must be TOTAL on these — every boolean COMPLETES (no panic: the on-edge
		// insertion previously divided by a zero-length edge and sorted a NaN, and
		// the stitch fed a non-manifold soup to `from_faces` which then asserted)
		// and returns a finite mesh that `validate` can inspect without panicking.
		// (Validity itself is not claimed: edge-/corner-only contact has a genuinely
		// non-manifold union that no closed B-rep can represent.)
		let unit = |o: DVec3| cuboid(o, o + DVec3::splat(2.0));
		let configs = [
			("coincident-face", unit(DVec3::ZERO), unit(DVec3::new(2.0, 0.0, 0.0))),
			("edge-touching", unit(DVec3::ZERO), unit(DVec3::new(2.0, 2.0, 0.0))),
			("corner-touching", unit(DVec3::ZERO), unit(DVec3::new(2.0, 2.0, 2.0))),
			("full-coincidence", unit(DVec3::ZERO), unit(DVec3::ZERO)),
			("sliver-overlap", unit(DVec3::ZERO), unit(DVec3::new(2.0 - 1e-9, 0.0, 0.0))),
		];
		for (name, a, b) in configs {
			for (op, r) in [
				("union", union(&a, &b)),
				("difference", difference(&a, &b)),
				("intersection", intersection(&a, &b)),
			] {
				let _ = validate(&r); // must not panic
				let mesh = tessellate_default(&r); // must not panic
				assert!(
					mesh.positions.iter().all(|p| p.x.is_finite() && p.y.is_finite() && p.z.is_finite()),
					"{name}/{op}: tessellation produced a non-finite vertex"
				);
			}
		}
	}
}

