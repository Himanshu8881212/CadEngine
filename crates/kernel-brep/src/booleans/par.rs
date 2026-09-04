// Copyright (c) LMCAD. Licensed under the MIT License.

//! Intra-arrangement parallelism for the boolean pipeline: the worker budget,
//! the measured engage cutoffs, and the single chunked pure-map stage runner
//! every threaded stage goes through. Nothing here can affect output bytes —
//! see the determinism argument below.

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
pub(super) const PAR_CHUNK: usize = 32;

/// Co-refinement's item-count engage arm: below this many subject triangles a
/// stage has ≲ 0.5 ms of uniform work, and spawn/join of the scoped workers
/// (~0.1 ms measured on the 8-core M-class dev machine) plus allocator
/// contention eats the win. Measured at the boundary (cylinder∖cuboid sweep,
/// tests note in `tests/threading_parity.rs`): segs=64 ≈ 390 subject tris stays
/// sequential at 1.49 ms both ways; segs=128 ≈ 780 tris engages at parity
/// (4.68 ms threaded vs 4.70 ms sequential); the win grows with size (flange
/// op 2 co-refine 16.9 → 7.7 ms). Below the cutoff both env settings run the
/// byte-identical sequential schedule at identical cost (segs=8: 99 µs both).
pub(super) const PAR_CUTOFF: usize = 512;

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
pub(super) const CLASSIFY_WORK_CUTOFF: usize = 200_000;
pub(super) const CLASSIFY_CHUNK_WORK: usize = 16_000;

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
pub(super) fn brep_workers() -> usize {
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
pub(super) fn stage_flat_map<T: Sync, R: Send>(
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
