// Copyright (c) LMCAD. Licensed under the MIT License.

//! Threading parity — the intra-arrangement parallelism gate (2026-07-30).
//!
//! The boolean pipeline's pure per-item stages (co-refinement, classification)
//! may run on scoped worker threads, controlled by `LMCAD_BREP_THREADS` (unset
//! or `0` ⇒ available parallelism, `1` ⇒ the exact legacy sequential schedule,
//! `N` ⇒ N workers). The R5 contract (`docs/NUMERICS.md`) says results are
//! bit-exact per platform, so the threaded schedule must be BYTE-IDENTICAL to
//! the sequential one — guaranteed structurally (each stage is a chunked pure
//! flat-map concatenated in ascending chunk order; see the parallelism section
//! in `src/booleans.rs` and `kernel_core::par::par_flat_map_chunks`) and
//! pinned here empirically over a corpus of diverse booleans.
//!
//! Everything runs inside ONE `#[test]`: the control surface is a process-
//! global environment variable, and Rust's test harness runs sibling tests on
//! concurrent threads — a second test in this binary could observe a schedule
//! flipped mid-build. One test, sequential case loop, no race.
//!
//! Wall-clock is MEASURED and printed but never asserted (machine-dependent
//! and flaky under CI load); the work-engagement receipt is asserted instead:
//! [`kernel_brep::booleans::par_items_processed`] must advance during the
//! threaded heavy builds (so a trivial always-sequential implementation cannot
//! pass) and must NOT advance under `LMCAD_BREP_THREADS=1`.
//!
//! Cutoff measurement note (referenced from `src/booleans.rs` constants), all
//! on the 8-core M-class dev machine, release build, 2026-07-30:
//! cylinder∖cuboid sweep — segs=8 (~56 tris): 99 µs sequential AND threaded
//! (sub-cutoff ops take the identical sequential schedule); segs=64 (~390
//! tris): 1.49 ms both; segs=128 (~780 tris, first size to engage): 4.68 ms
//! threaded vs 4.70 ms sequential (parity at the boundary — the cutoffs sit
//! where threading stops paying, not where it starts losing). Above the
//! boundary: flange-op classify 24.9 → 5.4 ms, co-refine 16.9 → 7.7 ms; whole
//! 9-boolean flange chain 0.69 → 0.38 s; the heavy revolve chain below
//! measured in-test (printed at the end of the run).

use kernel_brep::math::{DAffine3, DVec2, DVec3};
use kernel_brep::{
	cuboid, cylinder, difference, exact_volume, extrude, extrude_with_holes, intersection, revolve, sphere, union, validate, volume, Solid,
	VertexId,
};

// --- Canonical byte-level serialization ---------------------------------------

/// Full canonical dump of a solid: every vertex coordinate as raw f64 BITS, every
/// face's boundary/inner vertex-id rings in storage order, surface and provenance
/// tags, edge curve tags, topology counts, and the volume bit patterns. Two solids
/// with equal dumps are byte-identical for every downstream consumer (tessellation,
/// STEP export, chained booleans all read exactly these fields).
fn canonical_dump(s: &Solid) -> String {
	use std::fmt::Write as _;
	let mut d = String::new();
	let v = validate(s);
	let _ = writeln!(
		d,
		"counts v={} he={} e={} f={} sh={} | closed={} manifold={} shells={} genus={} euler={}",
		s.vertex_count(),
		s.half_edge_count(),
		s.edge_count(),
		s.face_count(),
		s.shell_count(),
		v.closed,
		v.manifold,
		v.shells,
		v.genus,
		v.euler_characteristic
	);
	let _ = writeln!(d, "volume_bits={:016x} exact_volume_bits={:016x}", volume(s).to_bits(), exact_volume(s).to_bits());
	for i in 0..s.vertex_count() {
		let p = s.position(VertexId(i as u32));
		let _ = writeln!(d, "v{} {:016x} {:016x} {:016x}", i, p.x.to_bits(), p.y.to_bits(), p.z.to_bits());
	}
	for f in s.faces() {
		let outer: Vec<u32> = s.face_vertices(f).iter().map(|v| v.0).collect();
		let _ = writeln!(d, "f{} outer={:?} surface={:?} name={:?}", f.0, outer, s.face(f).surface, s.face_name(f));
		for &l in &s.face(f).inner {
			let ring: Vec<u32> = s.loop_half_edges(l).iter().map(|&he| s.half_edge(he).origin.0).collect();
			let _ = writeln!(d, "f{} inner={:?}", f.0, ring);
		}
	}
	for e in s.edges() {
		if let Some(c) = s.edge_curve(e) {
			let _ = writeln!(d, "e{} curve={:?}", e.0, c);
		}
	}
	d
}

/// Assert two dumps equal, reporting the FIRST divergent line with context.
fn assert_dumps_equal(case: &str, schedule_a: &str, a: &str, schedule_b: &str, b: &str) {
	if a == b {
		return;
	}
	let (la, lb): (Vec<&str>, Vec<&str>) = (a.lines().collect(), b.lines().collect());
	let n = la.len().min(lb.len());
	let first = (0..n).find(|&i| la[i] != lb[i]);
	match first {
		Some(i) => panic!(
			"case `{case}`: {schedule_a} and {schedule_b} schedules diverged at dump line {i}:\n \
			 {schedule_a}: {}\n {schedule_b}: {}\n(R5 bit-determinism violated — a parallel stage \
			 reordered or perturbed an observable byte)",
			la[i], lb[i]
		),
		None => panic!(
			"case `{case}`: {schedule_a} dump has {} lines, {schedule_b} dump has {} lines (equal up to \
			 the shorter) — the schedules produced different amounts of geometry",
			la.len(),
			lb.len()
		),
	}
}

// --- Corpus --------------------------------------------------------------------

/// A tiny xorshift64 PRNG, the exact idiom of `tests/fuzz_chains.rs` (that
/// generator lives in a sibling TEST BINARY, so it is not importable from here;
/// these three seeded chains mirror its recipe vocabulary instead — random base,
/// random smaller operands at random offsets, mixed union/difference/intersection).
struct Rng(u64);

impl Rng {
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

	fn f(&mut self, lo: f64, hi: f64) -> f64 {
		lo + (self.next() >> 11) as f64 / (1u64 << 53) as f64 * (hi - lo)
	}

	fn u(&mut self, n: usize) -> usize {
		(self.next() % n as u64) as usize
	}
}

/// One seeded fuzz-mirror chain: random base solid, then three booleans against
/// smaller offset operands. Deterministic in the seed alone.
fn fuzz_mirror_chain(seed: u64) -> Solid {
	let mut rng = Rng::new(seed);
	let base = |rng: &mut Rng, scale: f64| -> Solid {
		match rng.u(4) {
			0 => {
				let e = DVec3::new(rng.f(0.5, 1.0), rng.f(0.5, 1.0), rng.f(0.5, 1.0)) * scale;
				cuboid(-e * 0.5, e * 0.5)
			}
			1 => {
				let r = rng.f(0.2, 0.4) * scale;
				let h = rng.f(0.5, 1.0) * scale;
				cylinder(DVec3::new(0.0, 0.0, -h * 0.5), DVec3::Z, r, h, 8 + rng.u(25))
			}
			2 => {
				let n = 5 + rng.u(6);
				let r = rng.f(0.25, 0.5) * scale;
				let poly: Vec<DVec2> = (0..n)
					.map(|i| {
						let a = std::f64::consts::TAU * (i as f64 + rng.f(-0.35, 0.35)) / n as f64;
						DVec2::new(r * a.cos(), r * a.sin())
					})
					.collect();
				extrude(&poly, rng.f(0.3, 0.8) * scale)
			}
			_ => sphere(DVec3::ZERO, rng.f(0.25, 0.5) * scale, 16, 12),
		}
	};
	let mut body = base(&mut rng, 100.0);
	for _ in 0..3 {
		let op = rng.u(3);
		let operand = base(&mut rng, 45.0).transformed(DAffine3::from_translation(DVec3::new(
			rng.f(-25.0, 25.0),
			rng.f(-25.0, 25.0),
			rng.f(-25.0, 25.0),
		)));
		body = match op {
			0 => union(&body, &operand),
			1 => difference(&body, &operand),
			_ => intersection(&body, &operand),
		};
		assert!(
			body.face_count() > 0,
			"fuzz-mirror seed {seed} emptied its chain (disjoint intersection) — a bare-primitive \
			 fallback would make the parity case vacuous; pick a different seed (11/23/47 are \
			 verified to run full 3-op chains)"
		);
	}
	body
}

/// The heavy case: 7 chained differences on a 256-segment revolve (bore at 128
/// segments + six 48-segment bolt drills) — the profile workload from the
/// implementation notes, ~0.3 s sequential on the dev machine.
fn heavy_revolve_drill_chain() -> Solid {
	let profile = [DVec2::new(10.0, 0.0), DVec2::new(40.0, 0.0), DVec2::new(40.0, 7.0), DVec2::new(39.0, 8.0), DVec2::new(10.0, 8.0)];
	let mut body = revolve(&profile, 256);
	body = difference(&body, &cylinder(DVec3::new(0.0, 0.0, -1.0), DVec3::Z, 12.0, 10.0, 128));
	for k in 0..6 {
		let a = std::f64::consts::TAU * k as f64 / 6.0;
		let bolt = cylinder(DVec3::new(30.0 * a.cos(), 30.0 * a.sin(), -1.0), DVec3::Z, 3.0, 10.0, 48);
		body = difference(&body, &bolt);
	}
	body
}

/// §7.7-style feature chain: plate ∪ boss, three drills, a keyway slot — the
/// DESIGN_GUIDE boolean-hygiene shape class (caps carrying earlier heal seams).
fn sect77_style_chain() -> Solid {
	let plate = cuboid(DVec3::new(-30.0, -20.0, 0.0), DVec3::new(30.0, 20.0, 6.0));
	let boss = cylinder(DVec3::new(0.0, 0.0, 6.0), DVec3::Z, 9.0, 10.0, 48);
	let mut body = union(&plate, &boss);
	for &(x, y) in &[(-22.0, -12.0), (22.0, -12.0), (0.0, 13.0)] {
		body = difference(&body, &cylinder(DVec3::new(x, y, -1.0), DVec3::Z, 2.5, 8.0, 24));
	}
	let slot = extrude(&[DVec2::new(-2.0, -25.0), DVec2::new(2.0, -25.0), DVec2::new(2.0, -6.0), DVec2::new(-2.0, -6.0)], 20.0);
	difference(&body, &slot.transformed(DAffine3::from_translation(DVec3::new(0.0, 0.0, -1.0))))
}

/// Multi-body arrangement: two disjoint shells as ONE operand, unioned with a
/// plate crossing both — classification must be right per shell.
fn multi_body_union() -> Solid {
	let a = cuboid(DVec3::new(-30.0, -8.0, -8.0), DVec3::new(-10.0, 8.0, 8.0));
	let b = cylinder(DVec3::new(20.0, 0.0, -8.0), DVec3::Z, 8.0, 16.0, 32);
	let two = a.disjoint_union(&b);
	let plate = cuboid(DVec3::new(-35.0, -3.0, -3.0), DVec3::new(35.0, 3.0, 3.0));
	union(&two, &plate)
}

struct Case {
	name: &'static str,
	/// Heavy enough that the threaded schedule MUST engage (the receipt assert).
	expect_engaged: bool,
	build: fn() -> Solid,
}

const CORPUS: &[Case] = &[
	Case { name: "heavy_revolve_drill_chain", expect_engaged: true, build: heavy_revolve_drill_chain },
	Case {
		name: "revolve_union_boss",
		expect_engaged: true,
		build: || {
			let profile = [DVec2::new(12.0, 0.0), DVec2::new(34.0, 0.0), DVec2::new(34.0, 9.0), DVec2::new(12.0, 9.0)];
			union(&revolve(&profile, 96), &cylinder(DVec3::new(0.0, 0.0, 4.0), DVec3::Z, 20.0, 22.0, 96))
		},
	},
	Case {
		name: "coplanar_shared_face_union",
		expect_engaged: false,
		build: || {
			// Two boxes sharing the z=10 plane exactly: the On{aligned} path.
			let lo = cuboid(DVec3::new(-10.0, -10.0, 0.0), DVec3::new(10.0, 10.0, 10.0));
			let hi = cuboid(DVec3::new(-6.0, -6.0, 10.0), DVec3::new(6.0, 6.0, 18.0));
			union(&lo, &hi)
		},
	},
	Case {
		name: "coplanar_partial_overlap_difference",
		expect_engaged: false,
		build: || {
			// Cutter shares part of the top face plane AND a side plane: partially
			// overlapping coincident facets cut by different diagonals.
			let a = cuboid(DVec3::new(-12.0, -12.0, 0.0), DVec3::new(12.0, 12.0, 12.0));
			let b = cuboid(DVec3::new(2.0, -5.0, 4.0), DVec3::new(12.0, 9.0, 12.0));
			difference(&a, &b)
		},
	},
	Case { name: "multi_body_union", expect_engaged: false, build: multi_body_union },
	Case { name: "sect77_style_chain", expect_engaged: false, build: sect77_style_chain },
	Case {
		name: "sphere_cylinder_intersection",
		expect_engaged: true,
		build: || intersection(&sphere(DVec3::ZERO, 14.0, 32, 24), &cylinder(DVec3::new(0.0, 0.0, -20.0), DVec3::Z, 9.0, 40.0, 48)),
	},
	Case {
		name: "holed_prism_cross_cut",
		expect_engaged: false,
		build: || {
			// Faces with INNER LOOPS entering triangulation (bridged-hole ear clip).
			let outer: Vec<DVec2> = (0..8)
				.map(|i| {
					let a = std::f64::consts::TAU * i as f64 / 8.0;
					DVec2::new(16.0 * a.cos(), 16.0 * a.sin())
				})
				.collect();
			let hole: Vec<DVec2> = (0..8)
				.map(|i| {
					let a = std::f64::consts::TAU * i as f64 / 8.0;
					DVec2::new(4.0 * a.cos(), 4.0 * a.sin())
				})
				.collect();
			let tube = extrude_with_holes(&outer, &[hole], 20.0);
			let bar = cuboid(DVec3::new(-30.0, -3.0, 6.0), DVec3::new(30.0, 3.0, 14.0));
			difference(&tube, &bar)
		},
	},
	Case { name: "fuzz_mirror_seed_11", expect_engaged: false, build: || fuzz_mirror_chain(11) },
	Case { name: "fuzz_mirror_seed_23", expect_engaged: false, build: || fuzz_mirror_chain(23) },
	Case { name: "fuzz_mirror_seed_47", expect_engaged: false, build: || fuzz_mirror_chain(47) },
];

// --- The gate --------------------------------------------------------------------

const THREADS_VAR: &str = "LMCAD_BREP_THREADS";

#[test]
fn threaded_and_sequential_schedules_are_byte_identical() {
	let engaged = kernel_brep::booleans::par_items_processed;
	let mut heavy_timing: Option<(std::time::Duration, std::time::Duration)> = None;
	let mut total_threaded_items = 0u64;

	for case in CORPUS {
		// Sequential schedule (`=1`): the exact legacy path. It must never touch
		// the threaded dispatcher.
		std::env::set_var(THREADS_VAR, "1");
		let before_seq = engaged();
		let t_seq = std::time::Instant::now();
		let solid_seq = (case.build)();
		let t_seq = t_seq.elapsed();
		assert!(
			engaged() == before_seq,
			"case `{}`: LMCAD_BREP_THREADS=1 must be a pure sequential schedule, but {} items \
			 went through the threaded dispatcher",
			case.name,
			engaged() - before_seq
		);
		let dump_seq = canonical_dump(&solid_seq);

		// The corpus itself must be real geometry, not vacuous empties.
		let v = validate(&solid_seq);
		assert!(
			v.closed && v.manifold && solid_seq.face_count() > 0,
			"case `{}`: corpus entry must be a closed manifold solid to be a meaningful parity \
			 subject, got closed={} manifold={} faces={}",
			case.name,
			v.closed,
			v.manifold,
			solid_seq.face_count()
		);

		// Threaded schedule, fixed width (`=4`).
		std::env::set_var(THREADS_VAR, "4");
		let before_thr = engaged();
		let t_thr = std::time::Instant::now();
		let solid_thr = (case.build)();
		let t_thr = t_thr.elapsed();
		let thr_items = engaged() - before_thr;
		total_threaded_items += thr_items;
		if case.expect_engaged {
			assert!(
				thr_items > 0,
				"case `{}`: the threaded schedule never engaged (0 items dispatched to workers) — \
				 the parity assertion would be vacuous on this heavy case",
				case.name
			);
		}
		assert_dumps_equal(case.name, "sequential(=1)", &dump_seq, "threaded(=4)", &canonical_dump(&solid_thr));

		// Default schedule (unset ⇒ ON with available parallelism).
		std::env::remove_var(THREADS_VAR);
		let solid_def = (case.build)();
		assert_dumps_equal(case.name, "sequential(=1)", &dump_seq, "default(unset)", &canonical_dump(&solid_def));

		if case.name == "heavy_revolve_drill_chain" {
			heavy_timing = Some((t_seq, t_thr));
		}
		println!(
			"parity `{}`: {} dump bytes, seq {:?} vs threaded {:?}, {} threaded items",
			case.name,
			dump_seq.len(),
			t_seq,
			t_thr,
			thr_items
		);
	}

	// Global work receipt: across the corpus the parallel path genuinely ran.
	// (Wall-clock is printed above but deliberately NOT asserted — it flakes
	// under CI load; the item counters are the scheduling-independent receipt.)
	assert!(
		total_threaded_items > 0,
		"no case dispatched any items to the threaded schedule — intra-arrangement parallelism \
		 is not engaging at all"
	);
	let (seq, thr) = heavy_timing.expect("heavy case ran");
	println!(
		"heavy_revolve_drill_chain wall-clock: {:.2?} sequential -> {:.2?} on 4 workers \
		 (measured, not asserted)",
		seq, thr
	);
}
