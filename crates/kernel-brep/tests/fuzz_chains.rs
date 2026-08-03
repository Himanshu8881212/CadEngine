// Copyright (c) LMCAD. Licensed under the MIT License.

//! Feature-chain fuzz harness — the Level-6 measurement gate (BAR.md) and the seed of
//! the Level-9 "published robustness evidence".
//!
//! Each chain starts from a random base solid (cuboid / cylinder / convex extrusion /
//! holed extrusion / sphere) in a ~100 mm domain and applies 3–7 random booleans
//! (union / difference / intersection) against smaller random base solids at random
//! translations — overlapping the body about half the time (intersections bias to
//! overlap, see [`gen_operand`]), disjoint otherwise, and occasionally with a
//! named-edge fillet applied to a fresh cuboid operand first (the persistent-naming
//! idiom from `fillet.rs`). After every op the body must stay
//! a closed, manifold, non-negative-genus B-rep per [`validate`] (genus may
//! legitimately grow — drilling adds handles). A chain fails at the first invalid
//! step (or panic, which is caught and recorded, never curated away).
//!
//! Everything derives deterministically from a chain's seed via an inline xorshift64
//! PRNG (no dependencies), so any failure reproduces exactly: call [`replay`] with the
//! recorded seed, or run
//!
//! ```text
//! FUZZ_SEED=<seed> cargo test -p kernel-brep --release --test fuzz_chains \
//!     -- replay_chain_from_env_seed --ignored --nocapture
//! ```
//!
//! Recipes are a function of (seed, generator version): editing the generator
//! re-shuffles the corpus, so re-measure the baseline whenever this file changes.
//!
//! REPORTING IS THE PRODUCT: the standard test prints pass/fail counts, the pass
//! rate, a per-op-kind failure histogram and the failing seeds (visible with
//! `-- --nocapture`; the assertion message repeats the headline numbers). The
//! measured numbers are published in `ROBUSTNESS.md` at the repo root. Since the
//! R5 determinism fix the whole report is byte-identical run to run.

use std::panic::{catch_unwind, AssertUnwindSafe};

use kernel_brep::math::{DAffine3, DVec2, DVec3};
use kernel_brep::{
	cuboid, cylinder, difference, extrude, extrude_with_holes, fillet_edge, intersection, sphere, union, validate,
	EdgeName, FaceName, FaceSource, Solid, Validity,
};

/// First seed of the corpus; chain `i` uses seed `BASE_SEED + i`, so the N=2000 deep
/// corpus is a strict superset of the N=200 standard one.
const BASE_SEED: u64 = 0x4C4D_4341_4400; // "LMCAD" + room for the index

/// RATCHET HISTORY (HONEST — raise on fixes, never lower to hide a regression):
/// - 2026-06-09 pre-fix baseline, 8 runs: 37.0–40.0% (median 38.5). The ±1.5-point
///   spread on FIXED seeds was the kernel's own run-to-run nondeterminism (std
///   `HashMap` iteration order in the boolean pipeline) — finding R5.
/// - 2026-06-09 post loop-aware-arrangement fix (R1–R4): 99.0 / 99.5 / 99.0 % over
///   3 runs. R5 was REDUCED but residual: exactly one marginal chain still flipped
///   between runs, so the floor kept 2 points of headroom below the lowest
///   observed rate.
/// - 2026-06-10 post R5 determinism fix (booleans.rs `cancel_coincident` drain order
///   and `recover_faces` region order; see ROBUSTNESS.md): 99.5% on 10/10 runs with
///   a byte-identical report — the corpus is now run-deterministic on fixed seeds.
///   Measured raised 99.0 → 99.5 (floor 97.0 → 97.5). The remaining ~0.5% failure
///   class (stitch explosions, 9 seeds) was the open Level-6 mop-up.
/// - 2026-06-10 post Level-6 mop-up (booleans.rs: unit-normal split distances,
///   no sub-EPS-area piece discard, operand boundary-chain simplification; see
///   ROBUSTNESS.md): **100.0% on both corpora, 3/3 runs byte-identical.** Measured
///   raised 99.5 → 100.0 (floor 97.5 → 98.0). The 2-point headroom covers
///   cross-platform libm (sin/cos) differences and future legitimate kernel
///   changes shifting marginal chains — at N=2000 that is a 40-chain margin. The
///   nine formerly-failing seeds are pinned by
///   [`residual_level6_seeds_stay_fixed`].
/// - 2026-07-30 the two Level-9 residual seeds are FIXED (booleans.rs: the
///   `resolve_t_junctions` degenerate-edge guard compared SQUARED length against
///   EPS, exempting every edge under √EPS ≈ 3.2e-5 from healing — seed
///   83894724552572's micro cut stub could never receive its interior T-vertex;
///   and a sub-heal-scale duplicate merge at stitch entry — seed 83894724550888
///   left two copies of one seam corner 1.051e-7 apart, 5% OVER the weld ball,
///   unpairable and unhealable). Standard/deep corpora unchanged at 100.0
///   (floors stay 98.0); the N=10 000 corpus rose 99.98 → **100.00** and its
///   floor ratchets to the measured rate (see
///   [`fuzz_10000_feature_chains_level9_corpus`]); both seeds are pinned by
///   [`residual_level9_seeds_stay_fixed`].
const MEASURED_PASS_RATE: f64 = 100.0;
const PASS_RATE_FLOOR: f64 = MEASURED_PASS_RATE - 2.0;

/// Deep corpus: 34.5–34.7% pre-fix (3 runs) → 99.5% post-R1–R4 (1991/2000) →
/// deterministically 99.5% post-R5 (the SAME 9 `closed=false` stitch explosions
/// every run) → **100.0% (2000/2000) post mop-up, 3/3 runs byte-identical**.
/// Same ratchet rule.
const DEEP_MEASURED_PASS_RATE: f64 = 100.0;
const DEEP_PASS_RATE_FLOOR: f64 = DEEP_MEASURED_PASS_RATE - 2.0;

// --- Deterministic PRNG --------------------------------------------------------

/// A tiny deterministic xorshift64 PRNG (same idiom as `kernel-implicit`'s query
/// tests) — keeps the corpus reproducible without pulling in a dependency.
struct Rng(u64);

impl Rng {
	/// Seed through a splitmix64-style mix so small consecutive seeds (0, 1, 2, …)
	/// land on well-separated xorshift states, and the all-zero fixed point is avoided.
	fn new(seed: u64) -> Rng {
		let mut z = seed.wrapping_add(0x9E37_79B9_7F4A_7C15);
		z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
		z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
		Rng((z ^ (z >> 31)) | 1)
	}

	fn next(&mut self) -> u64 {
		self.0 ^= self.0 << 13;
		self.0 ^= self.0 >> 7;
		self.0 ^= self.0 << 17;
		self.0
	}

	/// Uniform `f64` in `[lo, hi)`.
	fn f(&mut self, lo: f64, hi: f64) -> f64 {
		lo + (self.next() >> 11) as f64 / (1u64 << 53) as f64 * (hi - lo)
	}

	/// Uniform `usize` in `[0, n)` (modulo bias is irrelevant at fuzz scale).
	fn u(&mut self, n: usize) -> usize {
		(self.next() % n as u64) as usize
	}

	fn chance(&mut self, p: f64) -> bool {
		self.f(0.0, 1.0) < p
	}
}

// --- Op vocabulary --------------------------------------------------------------

/// The histogram bucket of one chain step. Boolean kinds are split by whether the
/// operand carried a named-edge fillet (a curved fillet face entering the boolean),
/// so the report shows whether filleted operands change the failure rate.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum OpKind {
	Base,
	Union,
	Difference,
	Intersection,
	FilletUnion,
	FilletDifference,
	FilletIntersection,
}

impl OpKind {
	const ALL: [OpKind; 7] = [
		OpKind::Base,
		OpKind::Union,
		OpKind::Difference,
		OpKind::Intersection,
		OpKind::FilletUnion,
		OpKind::FilletDifference,
		OpKind::FilletIntersection,
	];

	fn of(bool_kind: usize, with_fillet: bool) -> OpKind {
		match (bool_kind, with_fillet) {
			(0, false) => OpKind::Union,
			(1, false) => OpKind::Difference,
			(_, false) => OpKind::Intersection,
			(0, true) => OpKind::FilletUnion,
			(1, true) => OpKind::FilletDifference,
			(_, true) => OpKind::FilletIntersection,
		}
	}

	fn name(self) -> &'static str {
		match self {
			OpKind::Base => "base",
			OpKind::Union => "union",
			OpKind::Difference => "difference",
			OpKind::Intersection => "intersection",
			OpKind::FilletUnion => "union(filleted cuboid)",
			OpKind::FilletDifference => "difference(filleted cuboid)",
			OpKind::FilletIntersection => "intersection(filleted cuboid)",
		}
	}

	fn index(self) -> usize {
		OpKind::ALL.iter().position(|&k| k == self).unwrap()
	}
}

// --- Random geometry generators ---------------------------------------------------

/// A convex polygon: `n` vertices on a circle of radius `r` at jittered, strictly
/// increasing angles (jitter < ±0.5 step keeps the order, points on a circle in
/// angular order are convex). Worst-case angular gap is 1.7·(2π/n), so the polygon
/// always contains the disc of radius `r·cos(1.7π/n)` — the guarantee the hole
/// generator below relies on.
fn gen_convex_polygon(rng: &mut Rng, r: f64, n: usize) -> Vec<DVec2> {
	(0..n)
		.map(|i| {
			let a = std::f64::consts::TAU * (i as f64 + rng.f(-0.35, 0.35)) / n as f64;
			DVec2::new(r * a.cos(), r * a.sin())
		})
		.collect()
}

/// A regular `n`-gon of radius `r` centred at `(cx, cy)` — the hole vocabulary.
fn regular_ngon(cx: f64, cy: f64, r: f64, n: usize) -> Vec<DVec2> {
	(0..n)
		.map(|i| {
			let a = std::f64::consts::TAU * i as f64 / n as f64;
			DVec2::new(cx + r * a.cos(), cy + r * a.sin())
		})
		.collect()
}

/// A random base solid of overall size ~`scale` (mm), roughly centred on the origin.
/// Returns the solid plus a human-readable recipe fragment for the report.
fn gen_base(rng: &mut Rng, scale: f64) -> (Solid, String) {
	match rng.u(5) {
		0 => {
			let e = DVec3::new(rng.f(0.5, 1.0), rng.f(0.5, 1.0), rng.f(0.5, 1.0)) * scale;
			(cuboid(-e * 0.5, e * 0.5), format!("cuboid {:.1}x{:.1}x{:.1}", e.x, e.y, e.z))
		}
		1 => {
			let r = rng.f(0.2, 0.4) * scale;
			let h = rng.f(0.5, 1.0) * scale;
			let segs = 8 + rng.u(25); // 8..=32
			(
				cylinder(DVec3::new(0.0, 0.0, -h * 0.5), DVec3::Z, r, h, segs),
				format!("cylinder r={r:.1} h={h:.1} segs={segs}"),
			)
		}
		2 => {
			let n = 5 + rng.u(6); // 5..=10
			let r = rng.f(0.25, 0.5) * scale;
			let h = rng.f(0.3, 0.8) * scale;
			(
				extrude(&gen_convex_polygon(rng, r, n), h),
				format!("extrude {n}-gon r={r:.1} h={h:.1}"),
			)
		}
		3 => {
			let n = 5 + rng.u(6);
			let r = rng.f(0.25, 0.5) * scale;
			let h = rng.f(0.3, 0.8) * scale;
			let outer = gen_convex_polygon(rng, r, n);
			// 1–2 holes, kept strictly inside the outer polygon and apart from each
			// other: centre distance ≤ 0.26r and hole radius ≤ 0.14r stay inside the
			// guaranteed inscribed disc (≥ 0.48r for n ≥ 5); two holes sit on opposite
			// sides of the origin so their gap is ≥ 0.40r − 0.28r of radii.
			let nh = 1 + rng.u(2);
			let theta0 = rng.f(0.0, std::f64::consts::TAU);
			let holes: Vec<Vec<DVec2>> = (0..nh)
				.map(|k| {
					let a = theta0 + k as f64 * std::f64::consts::PI;
					let d = rng.f(0.20, 0.26) * r;
					let hr = rng.f(0.08, 0.14) * r;
					regular_ngon(d * a.cos(), d * a.sin(), hr, 8 + rng.u(5))
				})
				.collect();
			(
				extrude_with_holes(&outer, &holes, h),
				format!("extrude_with_holes {n}-gon r={r:.1} h={h:.1} holes={nh}"),
			)
		}
		_ => {
			let r = rng.f(0.25, 0.5) * scale;
			(sphere(DVec3::ZERO, r, 16, 12), format!("sphere r={r:.1} 16x12"))
		}
	}
}

/// A fresh cuboid with one of its four vertical edges rounded via its persistent
/// [`EdgeName`] (cuboid canonical faces: 2=−Y 3=+Y 4=−X 5=+X — the `fillet.rs` test
/// idiom). The fillet radius always fits, so an `Err` here is a genuine robustness
/// failure and is reported as one.
fn gen_filleted_cuboid(rng: &mut Rng, scale: f64) -> Result<(Solid, String), String> {
	let e = DVec3::new(rng.f(0.5, 1.0), rng.f(0.5, 1.0), rng.f(0.5, 1.0)) * scale;
	let pairs = [(5u32, 3u32), (5, 2), (4, 3), (4, 2)];
	let (fa, fb) = pairs[rng.u(4)];
	let radius = rng.f(0.08, 0.25) * e.x.min(e.y);
	let desc = format!("cuboid {:.1}x{:.1}x{:.1} fillet(faces {fa}/{fb}, r={radius:.2})", e.x, e.y, e.z);
	let edge = EdgeName::new(
		FaceName { operand: FaceSource::Primitive, source_face: fa },
		FaceName { operand: FaceSource::Primitive, source_face: fb },
	);
	let body = cuboid(-e * 0.5, e * 0.5);
	match fillet_edge(&body, edge, radius) {
		Ok(filleted) => {
			let v = validate(&filleted);
			if v.is_valid() {
				Ok((filleted, desc))
			} else {
				Err(format!("{desc} → filleted operand invalid: {}", validity_summary(&v)))
			}
		}
		Err(err) => Err(format!("{desc} → fillet_edge failed: {err:?}")),
	}
}

/// `(min+max)/2` and half the diagonal of a solid's AABB.
fn aabb_center_halfdiag(s: &Solid) -> (DVec3, f64) {
	let (lo, hi) = s.aabb();
	((lo + hi) * 0.5, (hi - lo).length() * 0.5)
}

/// Generate one boolean operand: a smaller random base solid (or a filleted fresh
/// cuboid), translated so its AABB centre lands inside the body's AABB (overlapping,
/// ~half the time) or strictly outside it (disjoint). Returns the placed operand and
/// its recipe; `Err` carries an operand-construction failure (fillet refused etc.).
///
/// `overlap_p` is 0.5 for union/difference; intersections bias to 0.85 because a
/// disjoint intersection is the *trivial* empty set, which legitimately ends the
/// chain — at a flat 0.5 most chains ended within a step or two and the corpus
/// barely exercised the booleans. The disjoint-intersection path keeps 15% coverage;
/// this biases the corpus HARDER (longer chains, more boolean steps), never softer.
fn gen_operand(rng: &mut Rng, body: &Solid, body_scale: f64, with_fillet: bool, overlap_p: f64) -> Result<(Solid, String), String> {
	let scale = body_scale * rng.f(0.35, 0.6);
	let (raw, desc) = if with_fillet { gen_filleted_cuboid(rng, scale)? } else { gen_base(rng, scale) };
	let overlap = rng.chance(overlap_p);
	let (blo, bhi) = body.aabb();
	let (bc, bhalf) = aabb_center_halfdiag(body);
	let (oc, ohalf) = aabb_center_halfdiag(&raw);
	let target = if overlap {
		// A point well inside the body's AABB (the middle 80% per axis).
		DVec3::new(
			blo.x + rng.f(0.1, 0.9) * (bhi.x - blo.x),
			blo.y + rng.f(0.1, 0.9) * (bhi.y - blo.y),
			blo.z + rng.f(0.1, 0.9) * (bhi.z - blo.z),
		)
	} else {
		// Beyond the two bounding spheres plus a margin — guaranteed disjoint.
		let theta = rng.f(0.0, std::f64::consts::TAU);
		let z = rng.f(-1.0, 1.0);
		let xy = (1.0 - z * z).max(0.0).sqrt();
		let dir = DVec3::new(xy * theta.cos(), xy * theta.sin(), z);
		bc + dir * (bhalf + ohalf + 2.0)
	};
	let t = target - oc;
	let placed = raw.transformed(DAffine3::from_translation(t));
	let mode = if overlap { "overlapping" } else { "disjoint" };
	Ok((placed, format!("{desc} at ({:.1}, {:.1}, {:.1}) {mode}", t.x, t.y, t.z)))
}

// --- Chain driver ---------------------------------------------------------------

/// One recorded chain failure: everything needed to reproduce and triage it.
struct Failure {
	seed: u64,
	op_index: usize,
	kind: OpKind,
	detail: String,
}

/// The outcome of one chain run.
struct ChainResult {
	/// Op kinds attempted, in order (including the failing one, if any).
	attempts: Vec<OpKind>,
	failure: Option<Failure>,
	/// The chain hit a legitimately-empty result (disjoint intersection, or a
	/// difference that consumed the whole body) and ended early as a pass.
	ended_empty: bool,
}

fn validity_summary(v: &Validity) -> String {
	format!(
		"closed={} manifold={} genus={} shells={} χ={}",
		v.closed, v.manifold, v.genus, v.shells, v.euler_characteristic
	)
}

fn panic_message(payload: Box<dyn std::any::Any + Send>) -> String {
	if let Some(s) = payload.downcast_ref::<&str>() {
		(*s).to_string()
	} else if let Some(s) = payload.downcast_ref::<String>() {
		s.clone()
	} else {
		"non-string panic payload".to_string()
	}
}

/// Run the chain for `seed`. With `verbose`, print every step's recipe and verdict
/// (printing never consumes randomness, so verbose and quiet runs are identical).
fn run_chain(seed: u64, verbose: bool) -> ChainResult {
	let mut rng = Rng::new(seed);
	let total_ops = 4 + rng.u(5); // 4..=8 ops, op 0 being the base solid
	let body_scale = rng.f(40.0, 70.0);
	let mut attempts = Vec::with_capacity(total_ops);
	let fail = |op_index: usize, kind: OpKind, detail: String, attempts: Vec<OpKind>| {
		if verbose {
			println!("  op {op_index}: {} → FAIL: {detail}", kind.name());
		}
		ChainResult { attempts, failure: Some(Failure { seed, op_index, kind, detail }), ended_empty: false }
	};

	// Op 0: the base solid must be born valid.
	attempts.push(OpKind::Base);
	let base = catch_unwind(AssertUnwindSafe(|| {
		let (solid, desc) = gen_base(&mut rng, body_scale);
		let v = validate(&solid);
		(solid, desc, v)
	}));
	let mut body = match base {
		Err(payload) => {
			return fail(0, OpKind::Base, format!("panicked: {}", panic_message(payload)), attempts);
		}
		Ok((solid, desc, v)) => {
			if !v.is_valid() || solid.face_count() == 0 {
				return fail(0, OpKind::Base, format!("{desc} → invalid base: {}", validity_summary(&v)), attempts);
			}
			if verbose {
				println!("  op 0: base {desc} → ok ({})", validity_summary(&v));
			}
			solid
		}
	};

	for op_index in 1..total_ops {
		let bool_kind = rng.u(3);
		let with_fillet = rng.chance(0.2);
		let kind = OpKind::of(bool_kind, with_fillet);
		attempts.push(kind);

		let overlap_p = if bool_kind == 2 { 0.85 } else { 0.5 };
		let step = catch_unwind(AssertUnwindSafe(|| -> Result<(Solid, Validity, String), String> {
			let (operand, desc) = gen_operand(&mut rng, &body, body_scale, with_fillet, overlap_p)?;
			let result = match bool_kind {
				0 => union(&body, &operand),
				1 => difference(&body, &operand),
				_ => intersection(&body, &operand),
			};
			let v = validate(&result);
			Ok((result, v, desc))
		}));

		match step {
			Err(payload) => {
				return fail(op_index, kind, format!("panicked: {}", panic_message(payload)), attempts);
			}
			Ok(Err(detail)) => {
				return fail(op_index, kind, detail, attempts);
			}
			Ok(Ok((result, v, desc))) => {
				if result.face_count() == 0 {
					// The empty set is the *correct* value of a disjoint intersection or
					// an all-consuming difference; nothing is left to operate on, so the
					// chain ends here as a pass. A union can never legitimately empty a
					// non-empty body — that is a failure.
					if bool_kind == 0 {
						return fail(op_index, kind, format!("{} {desc} → empty result from a union", kind.name()), attempts);
					}
					if verbose {
						println!("  op {op_index}: {} {desc} → empty (legitimate; chain ends)", kind.name());
					}
					return ChainResult { attempts, failure: None, ended_empty: true };
				}
				if !v.is_valid() {
					return fail(
						op_index,
						kind,
						format!("{} {desc} → invalid: {}", kind.name(), validity_summary(&v)),
						attempts,
					);
				}
				if verbose {
					println!("  op {op_index}: {} {desc} → ok ({})", kind.name(), validity_summary(&v));
				}
				body = result;
			}
		}
	}
	ChainResult { attempts, failure: None, ended_empty: false }
}

/// Re-run a single chain verbosely — the one-call reproduction handle for any
/// `(seed, op)` tuple in a report. Returns whether the chain passed.
pub fn replay(seed: u64) -> bool {
	println!("replaying chain seed={seed} (0x{seed:X})");
	let r = run_chain(seed, true);
	match &r.failure {
		Some(f) => {
			println!("chain seed={seed} FAILED at op {} [{}]: {}", f.op_index, f.kind.name(), f.detail);
			false
		}
		None => {
			println!("chain seed={seed} PASSED{}", if r.ended_empty { " (ended empty)" } else { "" });
			true
		}
	}
}

// --- Corpus runner + report -------------------------------------------------------

struct CorpusReport {
	n: usize,
	passed: usize,
	ended_empty: usize,
	attempts_per_kind: [usize; 7],
	failures_per_kind: [usize; 7],
	failures: Vec<Failure>,
	elapsed: std::time::Duration,
}

impl CorpusReport {
	fn pass_rate(&self) -> f64 {
		100.0 * self.passed as f64 / self.n as f64
	}
}

/// Run `n` chains seeded `BASE_SEED..BASE_SEED+n`. Panics inside an op are recorded
/// as that op's failure (the default panic hook is silenced for the duration so a
/// noisy corpus doesn't flood stderr; it is restored before returning).
fn run_corpus(n: usize) -> CorpusReport {
	let start = std::time::Instant::now();
	let prev_hook = std::panic::take_hook();
	std::panic::set_hook(Box::new(|_| {}));
	let mut report = CorpusReport {
		n,
		passed: 0,
		ended_empty: 0,
		attempts_per_kind: [0; 7],
		failures_per_kind: [0; 7],
		failures: Vec::new(),
		elapsed: std::time::Duration::ZERO,
	};
	for i in 0..n {
		let r = run_chain(BASE_SEED + i as u64, false);
		for &k in &r.attempts {
			report.attempts_per_kind[k.index()] += 1;
		}
		match r.failure {
			Some(f) => {
				report.failures_per_kind[f.kind.index()] += 1;
				report.failures.push(f);
			}
			None => {
				report.passed += 1;
				if r.ended_empty {
					report.ended_empty += 1;
				}
			}
		}
	}
	std::panic::set_hook(prev_hook);
	report.elapsed = start.elapsed();
	report
}

fn print_report(r: &CorpusReport) {
	println!("=== feature-chain fuzz report ===");
	println!(
		"chains: {} | passed: {} | failed: {} | pass rate: {:.1}% | ended-empty passes: {} | wall: {:.1}s",
		r.n,
		r.passed,
		r.failures.len(),
		r.pass_rate(),
		r.ended_empty,
		r.elapsed.as_secs_f64()
	);
	println!("failure histogram by op kind (failed/attempted):");
	for kind in OpKind::ALL {
		let (f, a) = (r.failures_per_kind[kind.index()], r.attempts_per_kind[kind.index()]);
		if a > 0 {
			println!("  {:<30} {:>4}/{:<5} ({:.1}%)", kind.name(), f, a, 100.0 * f as f64 / a as f64);
		}
	}
	if !r.failures.is_empty() {
		// Print EVERY failing chain (bounded to keep a catastrophic regression readable):
		// post-R5 the corpus is run-deterministic, so this list IS the residual-seed
		// list published in ROBUSTNESS.md — identical on every run.
		println!("failing chains (replay(seed) reproduces each):");
		for f in r.failures.iter().take(20) {
			println!("  seed={} op={} [{}]: {}", f.seed, f.op_index, f.kind.name(), f.detail);
		}
		if r.failures.len() > 20 {
			println!("  … and {} more", r.failures.len() - 20);
		}
	}
}

// --- Tests -------------------------------------------------------------------------

/// The standard N=200 corpus: prints the full report and ratchets the measured pass
/// rate (see [`PASS_RATE_FLOOR`]). This test is GREEN at the honest baseline — it
/// exists to catch regressions and publish the number, not to look good.
#[test]
fn fuzz_200_feature_chains_hold_the_measured_pass_rate() {
	let r = run_corpus(200);
	print_report(&r);
	assert!(
		r.pass_rate() >= PASS_RATE_FLOOR,
		"chain pass rate regressed: {:.1}% < floor {PASS_RATE_FLOOR:.1}% (measured baseline {MEASURED_PASS_RATE:.1}% on 2026-06-09; {} of {} chains failed, first: {})",
		r.pass_rate(),
		r.failures.len(),
		r.n,
		r.failures
			.first()
			.map(|f| format!("seed={} op={} [{}] {}", f.seed, f.op_index, f.kind.name(), f.detail))
			.unwrap_or_else(|| "none".to_string())
	);
}

/// Deep corpus (superset of the standard one) for overnight / pre-release runs:
/// `cargo test -p kernel-brep --release --test fuzz_chains -- --ignored fuzz_2000`.
#[test]
#[ignore = "deep 2000-chain corpus (~10x the standard runtime); run explicitly with --ignored"]
fn fuzz_2000_feature_chains_deep_corpus() {
	let r = run_corpus(2000);
	print_report(&r);
	assert!(
		r.pass_rate() >= DEEP_PASS_RATE_FLOOR,
		"deep-corpus pass rate regressed: {:.1}% < floor {DEEP_PASS_RATE_FLOOR:.1}% (measured baseline {DEEP_MEASURED_PASS_RATE:.1}% on 2026-06-09; {} of {} chains failed)",
		r.pass_rate(),
		r.failures.len(),
		r.n
	);
}

/// The Level-6 bar (BAR.md): ≥ 99% of random feature chains stay valid. UN-IGNORED
/// 2026-06-09 after the loop-aware arrangement fix — measured 99.5% on this corpus.
/// Runs the DEEP N=2000 corpus (~20 s) rather than N=200 so the residual one-chain
/// run-to-run flake (R5) cannot straddle the 99.0 threshold: at N=2000 the margin is
/// ~11 chains.
#[test]
fn fuzz_chains_meet_the_level6_bar() {
	let r = run_corpus(2000);
	print_report(&r);
	assert!(
		r.pass_rate() >= 99.0,
		"Level-6 bar not met: {:.1}% < 99.0% ({} of {} chains failed)",
		r.pass_rate(),
		r.failures.len(),
		r.n
	);
}

/// The nine Level-6 residual seeds — the ONLY chains of the deterministic N=2000
/// corpus that still stitch-exploded after R5 (`closed=false manifold=false`
/// micro-holes and slits; full diagnosis in ROBUSTNESS.md, 2026-06-10 section).
/// Fixed by three booleans.rs changes: (1) `split_convex_by_line` distances are
/// now measured against a UNIT line normal, so the on-line EPS band no longer
/// balloons by 1/|segment| for short clipped cut stubs at seam corners; (2) split
/// pieces are no longer discarded for sub-EPS area — a micro-area piece can still
/// have ~1e-4-long edges, and dropping it ripped unhealable coverage holes;
/// (3) `chain_redundant_vertices` strips redundant collinear boundary
/// micro-subdivisions (inherited T-junction chains) before triangulation, so
/// re-triangulating an accumulated body no longer fans sub-tolerance needles
/// (the disjoint-difference re-stitch failure) and operands no longer disagree
/// about seam corners at the ~1e-6 scale. Each seed replays its whole chain;
/// a regression in any op re-fails it loudly.
#[test]
fn residual_level6_seeds_stay_fixed() {
	const RESIDUAL_SEEDS: [u64; 9] = [
		83894724543558, // op 3 difference(filleted cuboid) — was shells=2 χ=3
		83894724543872, // op 2 difference, sphere          — was shells=1 χ=1
		83894724544312, // op 4 union, sphere               — was shells=1 χ=1
		83894724544576, // op 5 difference, sphere DISJOINT — was a pure re-stitch slit
		83894724544707, // op 5 union, sphere               — was shells=2 χ=3
		83894724544946, // op 5 intersection, holed extrude — was χ=-1
		83894724544984, // op 1 intersection, cylinder      — was shells=1 χ=1
		83894724545208, // op 6 intersection(filleted)      — was shells=1 χ=1
		83894724545313, // op 2 union(filleted)             — was shells=1 χ=1
	];
	let failures: Vec<String> = RESIDUAL_SEEDS
		.iter()
		.filter_map(|&seed| {
			run_chain(seed, false)
				.failure
				.map(|f| format!("seed={seed} op={} [{}]: {}", f.op_index, f.kind.name(), f.detail))
		})
		.collect();
	assert!(
		failures.is_empty(),
		"formerly-failing Level-6 residual seeds regressed ({} of 9):\n{}",
		failures.len(),
		failures.join("\n")
	);
}

/// The Level-9 evidence corpus (BAR.md: "≥99.5% on a 10k-chain fuzz corpus"):
/// N=10 000 chains, a strict superset of the N=200/2000 corpora. Run explicitly
/// (several minutes) with
/// `cargo test -p kernel-brep --release --test fuzz_chains -- --ignored fuzz_10000 --nocapture`
/// and publish the printed report in `ROBUSTNESS.md`.
///
/// FLOOR = the measured rate itself, **10 000/10 000** (2026-07-30, two runs
/// byte-identical), after the last two residual seeds were fixed — an EXACT pin,
/// deliberately tighter than the default-suite corpora's 2-point headroom: this
/// test is the manually-run evidence measurement on the publishing machine, the
/// corpus is run-deterministic there, and any future chain flip must be
/// re-diagnosed (and this comment re-dated), never absorbed. Cross-platform
/// libm drift is the default-suite floors' concern, not this pin's. Raise-only
/// still applies: the floor can never go back below 100.0% at this N.
#[test]
#[ignore = "Level-9 10k-chain corpus (~5x the deep runtime); run explicitly with --ignored"]
fn fuzz_10000_feature_chains_level9_corpus() {
	const LEVEL9_PASS_RATE_FLOOR: f64 = 100.0;
	let r = run_corpus(10_000);
	print_report(&r);
	assert!(
		r.pass_rate() >= LEVEL9_PASS_RATE_FLOOR,
		"10k-corpus pass rate regressed: {:.2}% < floor {LEVEL9_PASS_RATE_FLOOR:.1}% ({} of {} chains failed; the corpus measured 10000/10000 on 2026-07-30 after the two residual seeds — 83894724550888/83894724552572, pinned in residual_level9_seeds_stay_fixed — were fixed)",
		r.pass_rate(),
		r.failures.len(),
		r.n
	);
}

/// The two Level-9 residual seeds — the ONLY chains of the deterministic
/// N=10 000 corpus that still failed after the Level-6 mop-up and W3/W5 (both
/// `closed=false manifold=false` stitch holes; ROBUSTNESS.md 2026-07-30 section
/// has the full diagnosis). Fixed by two `booleans.rs` changes:
/// (1) `resolve_t_junctions`' degenerate-edge guard compared SQUARED edge
/// length against `EPS`, silently exempting every edge shorter than
/// √EPS ≈ 3.2e-5 mm from T-junction healing — 80× the healing tolerance. Seed
/// 83894724552572 (sphere ∩ sphere) left a 2.4e-5 micro cut stub on one
/// operand's seam whose interior vertex (the other operand's crossing point,
/// ~1e-9 off the stub) could never be inserted: an unhealable micro-triangle
/// hole. The guard now rejects only sub-weld (physically impossible) edges.
/// (2) The greedy weld (first-fit, [1e-7] ball) can leave TWO copies of one
/// seam corner just over the ball apart — seed 83894724550888 measured
/// 1.051e-7 — which are unstitchable: the healer cannot insert either copy
/// into an edge ending at the other (the projection parameter lands within
/// EPS of the endpoint), so the twin matcher saw an unpairable zero-area slit.
/// Stitch now merges vertex clusters closer than `TJUNCTION_EPS` (the stitch's
/// own resolution: the sliver filter and healer already treat that scale as
/// noise) by min-id union-find before filtering. Each seed replays its whole
/// chain here, in the default suite, so a regression is loud without the
/// manual 10k run.
#[test]
fn residual_level9_seeds_stay_fixed() {
	const RESIDUAL_SEEDS: [u64; 2] = [
		83894724550888, // chain #7400: op 7 difference, holed 5-gon extrude — was χ=-1, dup seam corner 1.051e-7 apart
		83894724552572, // chain #9084: op 1 intersection, sphere∩sphere — was χ=1, unhealable 2.4e-5 micro stub
	];
	let failures: Vec<String> = RESIDUAL_SEEDS
		.iter()
		.filter_map(|&seed| {
			run_chain(seed, false)
				.failure
				.map(|f| format!("seed={seed} op={} [{}]: {}", f.op_index, f.kind.name(), f.detail))
		})
		.collect();
	assert!(
		failures.is_empty(),
		"formerly-failing Level-9 residual seeds regressed ({} of 2):\n{}",
		failures.len(),
		failures.join("\n")
	);
}

/// Reproduce one chain verbosely from the `FUZZ_SEED` env var:
/// `FUZZ_SEED=<seed> cargo test -p kernel-brep --release --test fuzz_chains \
///     -- replay_chain_from_env_seed --ignored --nocapture`
#[test]
#[ignore = "manual reproduction helper; set FUZZ_SEED and run with --ignored --nocapture"]
fn replay_chain_from_env_seed() {
	// No-op without FUZZ_SEED so blanket `--include-ignored` runs stay green; the
	// helper only does work when an engineer aims it at a seed.
	let Ok(var) = std::env::var("FUZZ_SEED") else {
		println!("replay: set FUZZ_SEED=<seed> to replay a chain; nothing to do");
		return;
	};
	let seed: u64 = var.parse().expect("FUZZ_SEED must be a u64");
	// The replay is diagnostic: print the verdict either way, do not assert — the
	// engineer is here precisely because the chain fails.
	replay(seed);
}
