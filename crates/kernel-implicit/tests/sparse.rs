// Copyright (c) LMCAD. Licensed under the MIT License.

//! Sparse SDF caches (`kernel_implicit::sparse`) — the six 2026-07-30 gates:
//! 1. memory ∝ surface band, not domain volume (r = 80 sphere, 200³ box,
//!    0.4 mm voxel, ±2 mm band: < 5 % of the 503 MB dense grid, and build
//!    *evaluations* also a small fraction of the dense sample count);
//! 2. accuracy: trilinear-class near the surface, strictly conservative
//!    (never claims farther than truth) and sign-correct in the far field;
//! 3. mesh equivalence: narrow-band Surface Nets through the cache vs the
//!    analytic SDF — both watertight, volumes agree;
//! 4. determinism: independent builds are bit-identical (content hash);
//! 5. octree: adaptive depth on a CSG part — node count ≪ full-depth count,
//!    near-surface accuracy, far-field sign correctness;
//! 6. negative control: a ×3-scaled (over-claiming, hence out-of-contract)
//!    field demonstrably loses surface-crossing tiles, while the honest
//!    field loses none — the documented Lipschitz contract, pinned.

use std::sync::atomic::{AtomicUsize, Ordering};

use kernel_implicit::sparse::{OctreeGrid, SparseGrid};
use kernel_implicit::{surface_nets_narrowband, Aabb, Cuboid, Node, Resolution, Sdf, Sphere, Vec3};

/// Counts every `distance` evaluation — proves build-time sparsity honestly.
struct Counting<'a, S: Sdf + ?Sized> {
	inner: &'a S,
	evals: AtomicUsize,
}

impl<S: Sdf + ?Sized> Sdf for Counting<'_, S> {
	fn distance(&self, p: Vec3) -> f32 {
		self.evals.fetch_add(1, Ordering::Relaxed);
		self.inner.distance(p)
	}
	fn bounds(&self) -> Aabb {
		self.inner.bounds()
	}
}

/// A ×3-scaled distance: same zero set, but 3-Lipschitz and OVER-claiming
/// (|field| = 3 × true distance) — precisely the out-of-contract case the
/// module docs call out (fix: `kernel_implicit::redistance`).
struct TimesThree<S: Sdf>(S);

impl<S: Sdf> Sdf for TimesThree<S> {
	fn distance(&self, p: Vec3) -> f32 {
		3.0 * self.0.distance(p)
	}
	fn bounds(&self) -> Aabb {
		self.0.bounds()
	}
}

/// Deterministic spherical Fibonacci direction `i` of `n`.
fn fib_dir(i: usize, n: usize) -> Vec3 {
	let phi = (1.0 + 5.0f64.sqrt()) / 2.0;
	let z = 1.0 - 2.0 * (i as f64 + 0.5) / n as f64;
	let r = (1.0 - z * z).sqrt();
	let th = std::f64::consts::TAU * ((i as f64 / phi) % 1.0);
	Vec3::new((r * th.cos()) as f32, (r * th.sin()) as f32, z as f32)
}

#[test]
fn gate1_memory_and_evals_scale_with_band_not_volume() {
	// Dense arithmetic: 200 mm / 0.4 mm = 500 cells → 501 lattice points per
	// axis; 501³ = 125,751,501 f32 samples × 4 B = 503,006,004 B ≈ 503.0 MB.
	// 5 % of that is 25,150,300 B. The ±2 mm shell of the r = 80 sphere holds
	// ≈ 4π·80²·4 mm³ / 0.4³ ≈ 5.0e6 samples — the irreducible band content is
	// ≈ 20.1 MB in f32, i.e. 4.0 % of dense BEFORE any tile rounding, which is
	// why in-band samples are stored as 16-bit fixed point (module docs).
	let sphere = Sphere::new(Vec3::ZERO, 80.0);
	let counting = Counting { inner: &sphere, evals: AtomicUsize::new(0) };
	let bounds = Aabb::from_center_half_extent(Vec3::ZERO, Vec3::splat(100.0));
	let g = SparseGrid::build(&counting, bounds, 0.4, 2.0);
	let evals = counting.evals.load(Ordering::Relaxed);

	let dense_samples: usize = 501 * 501 * 501; // 125,751,501
	let dense_bytes: usize = dense_samples * 4; // 503,006,004
	let mem = g.memory_bytes();
	let mem_pct = 100.0 * mem as f64 / dense_bytes as f64;
	let tile_ratio = g.allocated_tiles() as f64 / g.total_tiles() as f64;
	let eval_pct = 100.0 * evals as f64 / dense_samples as f64;

	// Measured 2026-07-30: 22,596,232 B = 22.60 MB = 4.49 % of dense;
	// 20,113 / 250,047 tiles = 8.04 %; 12,255,935 evals = 9.75 % of dense.
	assert!(
		mem * 20 < dense_bytes // the ledgered bar: < 5 % of dense
			&& (18_000_000..25_150_300).contains(&mem) // and genuinely storing the ±2 mm band (≈ 20–24 MB), not gaming by storing nothing
			&& (0.05..0.11).contains(&tile_ratio)
			&& evals * 9 < dense_samples, // build evaluations also band-bound (expected ≈ 10 %, not 100 %)
		"sparse cache of r=80 sphere in 200³ @ 0.4 mm, band 2 mm: memory {:.2} MB = {mem_pct:.2}% of dense 503.0 MB \
		 (gate < 5%); {} of {} tiles allocated = {:.2}%; build evaluated {evals} samples = {eval_pct:.2}% of the \
		 dense 125,751,501 (gate < 11.1%)",
		mem as f64 / 1e6,
		g.allocated_tiles(),
		g.total_tiles(),
		100.0 * tile_ratio,
	);
}

#[test]
fn gate2_accuracy_trilinear_near_surface_conservative_far() {
	let sphere = Sphere::new(Vec3::ZERO, 80.0);
	let bounds = Aabb::from_center_half_extent(Vec3::ZERO, Vec3::splat(100.0));
	let g = SparseGrid::build(&sphere, bounds, 0.4, 2.0);

	// NEAR: probes inside the guaranteed-exact zone |d| ≤ band − √3·vs =
	// 2.0 − 0.693 = 1.31 (every corner sample of their cells is cached).
	// Trilinear theory for a sphere: err ≤ (3/8)·vs²·max|∂²d| ≈ 0.375 × 0.16
	// / 78.7 ≈ 7.6e-4, plus the 16-bit quantum band/32767 = 6.1e-5 and ~2e-5
	// of f32 evaluation noise at |p| ≈ 80 ⇒ bound 1.2e-3, O(voxel²/r) scale.
	let mut max_near = 0.0f64;
	for i in 0..200 {
		let dir = fib_dir(i, 200);
		for off in [-1.2f32, -0.75, -0.31, 0.0, 0.4, 0.85, 1.2] {
			let p = dir * (80.0 + off);
			let truth = p.as_dvec3().length() - 80.0;
			let err = (f64::from(g.distance(p)) - truth).abs();
			max_near = max_near.max(err);
		}
	}

	// FAR: probes ≥ 8 mm from the surface are guaranteed to land in
	// unallocated tiles (allocation reaches at most band + 2·h_tile =
	// 2 + 2×2.77 = 7.54 mm from the surface), where the returned constant
	// must STRICTLY under-claim (|v| ≤ |true|, sign-correct) — the
	// DistanceBound direction: never claim more clearance than exists. It
	// must also stay useful: |v| ≥ |off| − 6.3 (far constants give up at most
	// one stride-cube diagonal + one cell diagonal) and ≥ band − √3·vs.
	let mut far_ok = true;
	let mut worst_slack = f64::INFINITY; // min(|true| − |v|): must stay ≥ ~0
	let mut worst_floor = f64::INFINITY; // min(|v| − floor): must stay ≥ ~0
	for i in 0..64 {
		let dir = fib_dir(i, 64);
		for off in [8.0f32, 12.0, 19.0, -8.0, -20.0, -50.0, -75.0] {
			let p = dir * (80.0 + off);
			let truth = p.as_dvec3().length() - 80.0;
			let v = f64::from(g.distance(p));
			far_ok &= (v < 0.0) == (truth < 0.0);
			worst_slack = worst_slack.min(truth.abs() - v.abs());
			let floor = (f64::from(off.abs()) - 6.3).max(1.3);
			worst_floor = worst_floor.min(v.abs() - floor);
		}
	}

	// Measured 2026-07-30: max_near = 5.06e-4 (bound 1.2e-3), worst_slack =
	// +0.698 (strictly conservative with margin), worst_floor = +1.147.
	assert!(
		max_near < 1.2e-3 && far_ok && worst_slack > -1.0e-3 && worst_floor > -1.0e-3,
		"r=80 sphere @ 0.4 mm/±2 mm: max |cache − exact| near surface = {max_near:.2e} (gate 1.2e-3, trilinear \
		 O(voxel²/r) ≈ 7.6e-4 + 6.1e-5 quantum); far field sign-correct = {far_ok}, strict conservativeness \
		 slack min(|true|−|v|) = {worst_slack:.4} (≥ 0 up to 1e-3 f32 rounding), usefulness floor slack = {worst_floor:.4}"
	);
}

#[test]
fn gate3_narrowband_mesh_through_cache_matches_analytic() {
	// Same mesher, same lattice, only the field differs: analytic sphere vs
	// the sparse cache. Both must be watertight and agree in volume (≤ 1 %).
	let sphere = Sphere::new(Vec3::ZERO, 20.0);
	let bounds = Aabb::from_center_half_extent(Vec3::ZERO, Vec3::splat(25.0));
	let g = SparseGrid::build(&sphere, bounds, 0.25, 1.0);

	let ma = surface_nets_narrowband(&sphere, bounds, Resolution::VoxelSize(0.25));
	let mg = surface_nets_narrowband(&g, bounds, Resolution::VoxelSize(0.25));
	let (va, vg) = (ma.signed_volume(), mg.signed_volume());
	let delta_pct = 100.0 * ((vg - va) / va).abs();
	let exact = 4.0 / 3.0 * std::f64::consts::PI * 20.0f64.powi(3); // 33,510.3 mm³

	// Measured 2026-07-30: va = vg = 33,504.33 mm³, delta 0.00002 %, both
	// watertight (analytic sphere volume 33,510.3 mm³ → 0.018 % discretisation).
	assert!(
		ma.is_watertight() && mg.is_watertight() && delta_pct < 1.0 && ((va - exact) / exact).abs() < 0.02,
		"narrow-band mesh r=20 sphere @ 0.25 mm: analytic {va:.1} mm³ (watertight {}), through SparseGrid \
		 {vg:.1} mm³ (watertight {}), delta {delta_pct:.4}% (gate ≤ 1%); analytic vs true 33,510 mm³",
		ma.is_watertight(),
		mg.is_watertight()
	);
}

#[test]
fn gate4_independent_builds_are_bit_identical() {
	let sphere = Sphere::new(Vec3::ZERO, 20.0);
	let bounds = Aabb::from_center_half_extent(Vec3::ZERO, Vec3::splat(25.0));
	let a = SparseGrid::build(&sphere, bounds, 0.4, 1.0);
	let b = SparseGrid::build(&sphere, bounds, 0.4, 1.0);

	let part = Node::primitive(Sphere::new(Vec3::ZERO, 8.0));
	let ob = Aabb::from_center_half_extent(Vec3::ZERO, Vec3::splat(12.0));
	let oa = OctreeGrid::build(&part, ob, 5, 0.8);
	let oc = OctreeGrid::build(&part, ob, 5, 0.8);

	// Degenerate input must not panic or poison: inverted box → tiny far grid.
	let degenerate = SparseGrid::build(&sphere, Aabb::new(Vec3::splat(1.0), Vec3::splat(-1.0)), 0.5, 1.0);
	let dg = degenerate.distance(Vec3::ZERO);

	// Measured 2026-07-30: SparseGrid hash 0x2dd784cad7caca65 both builds
	// (1,024,144 B, 968 tiles); OctreeGrid hash 0x3b02e28b3c620757 both builds
	// (10,505 nodes); degenerate-bounds distance −11.34 (finite).
	assert!(
		a.content_hash() == b.content_hash()
			&& a.memory_bytes() == b.memory_bytes()
			&& a.allocated_tiles() == b.allocated_tiles()
			&& oa.content_hash() == oc.content_hash()
			&& oa.node_count() == oc.node_count()
			&& !dg.is_nan(),
		"determinism: SparseGrid hashes {:#018x} vs {:#018x} (memory {} B, {} tiles), OctreeGrid hashes \
		 {:#018x} vs {:#018x} ({} nodes); degenerate-bounds distance = {dg} (must be non-NaN)",
		a.content_hash(),
		b.content_hash(),
		a.memory_bytes(),
		a.allocated_tiles(),
		oa.content_hash(),
		oc.content_hash(),
		oa.node_count()
	);
}

#[test]
fn gate5_octree_adaptive_depth_on_csg_part() {
	// CSG, not a lone primitive: box ∪ sphere via the existing Node ops. The
	// union field is a DistanceBound (under-claims near the seam) — exactly
	// the allocation-safe direction per the module docs.
	let part = Node::primitive(Cuboid::new(Vec3::new(-12.0, 0.0, 0.0), Vec3::new(10.0, 8.0, 8.0)))
		.union(Node::primitive(Sphere::new(Vec3::new(18.0, 0.0, 0.0), 10.0)));
	let bounds = Aabb::from_center_half_extent(Vec3::ZERO, Vec3::splat(30.0));
	let oct = OctreeGrid::build(&part, bounds, 6, 1.0);

	// Full-depth node count: Σ_{l=0}^{6} 8^l = (8^7 − 1)/7 = 299,593.
	let full: usize = (0..=6).map(|l| 8usize.pow(l)).sum();
	assert_eq!(full, 299_593);
	let nodes = oct.node_count();
	let node_pct = 100.0 * nodes as f64 / full as f64;

	// Accuracy vs the FIELD it caches (part.distance), probed on the sphere
	// limb within ±0.4 mm of the surface — max-depth leaves, cell 60/64 =
	// 0.9375 mm ⇒ trilinear-class err ≈ (3/8)·0.9375²/9.6 ≈ 0.034 mm.
	let mut max_sphere = 0.0f32;
	for i in 0..100 {
		let dir = fib_dir(i, 100);
		for off in [-0.4f32, 0.0, 0.4] {
			let p = Vec3::new(18.0, 0.0, 0.0) + dir * (10.0 + off);
			max_sphere = max_sphere.max((oct.distance(p) - part.distance(p)).abs());
		}
	}
	// On the planar box face the field is locally linear ⇒ trilinear is exact
	// up to sampling noise.
	let mut max_face = 0.0f32;
	for p in [Vec3::new(-12.0, 1.5, 7.65), Vec3::new(-15.0, -3.0, 8.2), Vec3::new(-8.0, 4.0, 8.0)] {
		max_face = max_face.max((oct.distance(p) - part.distance(p)).abs());
	}

	// Far field: sign-correct everywhere it is probed (|true field| > 2 mm).
	let mut sign_probes = 0usize;
	let mut sign_ok = true;
	for xi in 0..7 {
		for yi in 0..7 {
			for zi in 0..7 {
				let p = Vec3::new(-27.0 + 9.0 * xi as f32, -27.0 + 9.0 * yi as f32, -27.0 + 9.0 * zi as f32);
				let truth = part.distance(p);
				if truth.abs() > 2.0 {
					sign_probes += 1;
					sign_ok &= (oct.distance(p) < 0.0) == (truth < 0.0);
				}
			}
		}
	}

	let mem = oct.memory_bytes();
	// Measured 2026-07-30: 25,257 nodes = 8.43 % of 299,593; depth reached 6;
	// 909,324 B; max_sphere = 0.0220 mm; max_face = 0.0 mm; 320/320 signs.
	assert!(
		nodes * 8 < full // ≪ full-depth: measured expectation ≈ 7 %
			&& (3_000..60_000).contains(&nodes)
			&& oct.max_depth_reached() == 6
			&& max_sphere < 0.09
			&& max_face < 0.02
			&& sign_ok
			&& sign_probes > 250
			&& (150_000..4_000_000).contains(&mem),
		"octree on box∪sphere in 60³ @ depth 6, band 1 mm: {nodes} nodes = {node_pct:.2}% of full-depth 299,593 \
		 (gate < 12.5%), depth reached {}, memory {:.2} MB (full-depth would be {:.1} MB); |cache − field| max \
		 {max_sphere:.4} mm on the sphere limb (trilinear-class ≈ 0.034), {max_face:.5} mm on the planar face; \
		 far-field sign correct on {sign_probes} probes = {sign_ok}",
		oct.max_depth_reached(),
		mem as f64 / 1e6,
		(full * 36) as f64 / 1e6
	);
}

#[test]
fn gate6_overclaiming_field_is_out_of_contract_and_misses_tiles() {
	// The allocation rule is only safe for fields that never over-claim
	// distance (module docs). A ×3-scaled sphere keeps the same surface but
	// reports 3× the distance, so the centre test |d(c)| ≤ band + h can skip
	// tiles the surface actually crosses. The honest field must miss NONE
	// (empirical proof of the safety theorem); the scaled field must
	// demonstrably miss some, and the cache then reads far-from-surface at a
	// true surface point. Remedy, per docs: kernel_implicit::redistance.
	let bounds = Aabb::from_center_half_extent(Vec3::ZERO, Vec3::splat(25.0));
	let honest = SparseGrid::build(&Sphere::new(Vec3::ZERO, 20.0), bounds, 0.4, 1.0);
	let bad = SparseGrid::build(&TimesThree(Sphere::new(Vec3::ZERO, 20.0)), bounds, 0.4, 1.0);

	// Analytic straddling-tile enumeration: a tile's stride cube crosses the
	// r = 20 sphere iff min/max radius over the cube bracket 20.
	let stride = honest.tile_size(); // 8 × 0.4 = 3.2 mm
	let [tdx, tdy, tdz] = honest.tile_dims();
	let (mut straddle, mut missed_honest, mut missed_bad) = (0usize, 0usize, 0usize);
	let mut witness: Option<Vec3> = None;
	for tz in 0..tdz {
		for ty in 0..tdy {
			for tx in 0..tdx {
				let cmin = honest.origin() + Vec3::new(tx as f32, ty as f32, tz as f32) * stride;
				let cmax = cmin + Vec3::splat(stride);
				let rmin = Vec3::ZERO.clamp(cmin, cmax).length();
				let rmax = [
					cmin,
					Vec3::new(cmax.x, cmin.y, cmin.z),
					Vec3::new(cmin.x, cmax.y, cmin.z),
					Vec3::new(cmax.x, cmax.y, cmin.z),
					Vec3::new(cmin.x, cmin.y, cmax.z),
					Vec3::new(cmax.x, cmin.y, cmax.z),
					Vec3::new(cmin.x, cmax.y, cmax.z),
					cmax,
				]
				.iter()
				.map(|c| c.length())
				.fold(0.0f32, f32::max);
				if rmin <= 20.0 && 20.0 <= rmax {
					straddle += 1;
					if !honest.tile_allocated(tx, ty, tz) {
						missed_honest += 1;
					}
					if !bad.tile_allocated(tx, ty, tz) {
						missed_bad += 1;
						if witness.is_none() {
							// A true surface point inside this very tile, if
							// the radial one lands there.
							let centre = (cmin + cmax) * 0.5;
							let s = centre.normalize() * 20.0;
							if s.cmpge(cmin).all() && s.cmplt(cmax).all() {
								witness = Some(s);
							}
						}
					}
				}
			}
		}
	}

	// The witness far value can legitimately be as small as band − √3·vs =
	// 0.31 mm (a sampled-then-dropped tile) — still clearly off-surface.
	let witness_read = witness.map(|s| bad.distance(s).abs());
	// Measured 2026-07-30: 740 straddling tiles; honest missed 0; ×3 field
	// missed 360 (48.6 %); witness surface point reads |d| = 2.04 mm.
	assert!(
		straddle > 500 && missed_honest == 0 && missed_bad > 0 && missed_bad < straddle && witness_read.is_some_and(|v| v > 0.2),
		"Lipschitz contract, r=20 sphere @ 0.4 mm/±1 mm ({straddle} surface-crossing tiles): honest field missed \
		 {missed_honest} (theorem: must be 0); ×3 over-claiming field missed {missed_bad} = {:.1}% (documented \
		 out-of-contract behaviour — redistance first); cache reads |d| = {:.3} mm at a true surface point in a \
		 missed tile (> 0.2)",
		100.0 * missed_bad as f64 / straddle as f64,
		witness_read.unwrap_or(f32::NAN)
	);
}
