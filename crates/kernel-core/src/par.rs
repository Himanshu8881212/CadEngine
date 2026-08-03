// Copyright (c) LMCAD. Licensed under the MIT License.

//! Deterministic coarse-grained parallelism: map independent items on scoped
//! threads, results returned BY INDEX so scheduling cannot change them. The
//! shared work-queue that `sweep_check` and `overlap_volume_many` each
//! hand-rolled (2026-07-28), extracted.
//!
//! Since 2026-07-30 this also carries the one pattern under which threading
//! INSIDE an algorithm is admissible without risking bit-determinism (R5,
//! `docs/NUMERICS.md`): [`par_flat_map_chunks`], a pure chunked flat-map whose
//! output is byte-identical to the sequential run BY CONSTRUCTION — used by the
//! boolean arrangement's pure per-item stages (`kernel-brep/src/booleans.rs`).
//! Anything with ordered mutation of shared state (welding, ring assembly,
//! region recovery) remains sequential there; this module offers no tool for
//! such stages on purpose.

thread_local! {
	/// True on threads spawned by this module's worker pools.
	static IN_WORKER: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// Whether the current thread is a worker of one of this module's pools. Nested
/// parallel constructs use this to stay sequential instead of oversubscribing
/// (e.g. the boolean arrangement's intra stages inside a pose-parallel
/// `sweep_check` / `overlap_volume_many` map — the coarse grain already owns
/// the cores there). Scheduling-only: it can never change output bytes, because
/// every construct here is deterministic for EVERY worker count.
pub fn in_worker_thread() -> bool {
	IN_WORKER.with(|w| w.get())
}

/// Map `f` over `items` on up to `available_parallelism` scoped threads.
/// Result order equals input order; panics propagate like a serial map.
pub fn par_map_indexed<T: Sync, R: Send>(items: &[T], f: impl Fn(usize, &T) -> R + Sync) -> Vec<R> {
	let workers = std::thread::available_parallelism().map(|n| n.get()).unwrap_or(1).min(items.len().max(1));
	if workers <= 1 || items.len() <= 1 {
		return items.iter().enumerate().map(|(i, t)| f(i, t)).collect();
	}
	let mut slots: Vec<Option<R>> = (0..items.len()).map(|_| None).collect();
	let next = std::sync::atomic::AtomicUsize::new(0);
	let slots_ref = std::sync::Mutex::new(&mut slots);
	std::thread::scope(|s| {
		for _ in 0..workers {
			s.spawn(|| {
				IN_WORKER.with(|w| w.set(true));
				loop {
					let i = next.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
					if i >= items.len() {
						break;
					}
					let r = f(i, &items[i]);
					slots_ref.lock().unwrap()[i] = Some(r);
				}
			});
		}
	});
	slots.into_iter().map(|s| s.expect("worker filled every slot")).collect()
}

/// Flat-map `f` over `items` in contiguous `chunk_len`-sized chunks, on up to
/// `workers` scoped threads, concatenating the per-chunk outputs **in ascending
/// chunk order**. `workers <= 1` runs the identical chunk loop on the calling
/// thread — one implementation, two schedules.
///
/// ## Structural bit-determinism (the R5 argument)
/// The result is `concat(f(chunk_0), f(chunk_1), …)` where the chunk boundaries
/// are a pure function of `(items.len(), chunk_len)` — never of the worker
/// count or of scheduling. When `f` is a **pure function of its chunk slice and
/// read-only captured state** (no shared accumulators, no iteration-order-
/// dependent containers), each `f(chunk_i)` computes the identical float
/// expression sequence whether it runs on the calling thread or any worker, and
/// the concatenation position of its output is fixed by `i`. Thread scheduling
/// therefore cannot reorder or perturb ANY observable byte of the result:
/// threaded output is bit-identical to sequential output by construction, not
/// by testing alone. (Tests still pin it: `kernel-brep/tests/threading_parity.rs`.)
///
/// Panics in `f` propagate to the caller like a serial map (scoped threads
/// re-raise on join); `f` must be side-effect-free for the schedules to be
/// equivalent, which the purity requirement above already demands.
pub fn par_flat_map_chunks<T: Sync, R: Send>(
	workers: usize,
	items: &[T],
	chunk_len: usize,
	f: impl Fn(&[T]) -> Vec<R> + Sync,
) -> Vec<R> {
	let chunk_len = chunk_len.max(1);
	let n_chunks = items.len().div_ceil(chunk_len).max(1);
	let workers = workers.min(n_chunks);
	if workers <= 1 || n_chunks <= 1 {
		let mut out = Vec::new();
		for chunk in items.chunks(chunk_len) {
			out.extend(f(chunk));
		}
		return out;
	}
	let mut slots: Vec<Option<Vec<R>>> = (0..n_chunks).map(|_| None).collect();
	let next = std::sync::atomic::AtomicUsize::new(0);
	let slots_ref = std::sync::Mutex::new(&mut slots);
	std::thread::scope(|s| {
		for _ in 0..workers {
			s.spawn(|| {
				IN_WORKER.with(|w| w.set(true));
				loop {
					let ci = next.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
					if ci >= n_chunks {
						break;
					}
					let lo = ci * chunk_len;
					let hi = (lo + chunk_len).min(items.len());
					let r = f(&items[lo..hi]);
					slots_ref.lock().unwrap()[ci] = Some(r);
				}
			});
		}
	});
	let mut out = Vec::new();
	for slot in slots {
		out.extend(slot.expect("worker filled every chunk slot"));
	}
	out
}

#[cfg(test)]
mod tests {
	use super::*;

	/// Pinning test: pool workers are flagged so nested parallel constructs can
	/// decline to oversubscribe; the calling thread is never flagged.
	#[test]
	fn worker_threads_are_flagged_and_caller_is_not() {
		assert!(
			!in_worker_thread(),
			"the calling thread must not be flagged as a pool worker before any map runs"
		);
		let items: Vec<u32> = (0..64).collect();
		let flags = par_map_indexed(&items, |_, _| in_worker_thread());
		assert!(
			flags.iter().all(|&f| f),
			"every par_map_indexed item must observe in_worker_thread() == true (got {} false)",
			flags.iter().filter(|&&f| !f).count()
		);
		let flags = par_flat_map_chunks(4, &items, 8, |chunk| chunk.iter().map(|_| in_worker_thread()).collect());
		assert!(
			flags.iter().all(|&f| f),
			"every par_flat_map_chunks item must observe in_worker_thread() == true (got {} false)",
			flags.iter().filter(|&&f| !f).count()
		);
		assert!(!in_worker_thread(), "the flag must not leak onto the calling thread after the maps");
	}

	/// Pinning test: the chunked flat-map's output equals the plain sequential
	/// flat-map for every worker count and for chunk lengths that do / do not
	/// divide the item count — the by-construction determinism made falsifiable.
	#[test]
	fn par_flat_map_chunks_matches_sequential_for_all_schedules() {
		let items: Vec<u64> = (0..1000).collect();
		// A per-item expansion with variable output length, so concatenation
		// order mistakes cannot cancel out.
		let expand = |chunk: &[u64]| -> Vec<u64> {
			let mut out = Vec::new();
			for &x in chunk {
				for k in 0..(x % 3 + 1) {
					out.push(x * 1_000 + k);
				}
			}
			out
		};
		let reference: Vec<u64> = expand(&items);
		for workers in [1usize, 2, 3, 8, 64] {
			for chunk_len in [1usize, 7, 128, 1000, 5000] {
				let got = par_flat_map_chunks(workers, &items, chunk_len, expand);
				assert!(
					got == reference,
					"par_flat_map_chunks(workers={workers}, chunk_len={chunk_len}) diverged from \
					 the sequential flat-map: got {} items, expected {} — first mismatch at index {:?}",
					got.len(),
					reference.len(),
					got.iter().zip(&reference).position(|(a, b)| a != b)
				);
			}
		}
	}
}
