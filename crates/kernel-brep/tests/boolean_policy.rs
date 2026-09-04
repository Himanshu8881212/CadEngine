// Copyright (c) LMCAD. Licensed under the MIT License.

//! The tiered boolean policy — outcome-reporting is correct and the path matches
//! reality (`kernel_brep::policy`).
//!
//! Each test pins one tier of the declared policy and cross-checks the reported
//! [`BooleanPath`] against ground truth (the strict [`try_*`] API, the raw op's
//! validity): a clean boolean reports EXACT and is bit-identical to the raw op; a
//! genuinely cracked operand reports HEALED (and only the heal tier rescues it); a
//! documented coincident/tangent-face degeneracy reports REFUSED honestly (the
//! heal tier does NOT paper over it). A final metric test folds a mixed batch into
//! [`BooleanStats`] so the fallback/refusal rates are measurable.

use kernel_brep::math::{DAffine3, DVec2, DVec3};
use kernel_brep::{
	boolean_with_policy, cuboid, cylinder, difference, extrude, try_difference, try_union, validate, volume, BooleanPath, BooleanStats,
	FaceInput, MeshBoolOp, Solid,
};

fn v(x: f64, y: f64, z: f64) -> DVec3 {
	DVec3::new(x, y, z)
}

// --- A cracked-shell generator (the same idiom as heal.rs's tests) --------------

/// Deterministic xorshift64 in [0, 1) — no dependency, reproducible corpus.
struct Rng(u64);
impl Rng {
	fn next_f64(&mut self) -> f64 {
		self.0 ^= self.0 << 13;
		self.0 ^= self.0 >> 7;
		self.0 ^= self.0 << 17;
		(self.0 >> 11) as f64 / (1u64 << 53) as f64
	}
}

/// Explode `s` so every face owns private copies of its vertices, each perturbed
/// by a random direction of magnitude in `[lo, hi]` — a deliberately cracked shell
/// of the kind a lossy import produces. Every face is present; only the shared
/// -vertex identification is torn, so the exact boolean cannot close the result but
/// a tolerant heal within `tol > hi` re-welds it.
fn cracked(s: &Solid, lo: f64, hi: f64, seed: u64) -> Solid {
	let mut rng = Rng(seed);
	let mut positions: Vec<DVec3> = Vec::new();
	let mut faces: Vec<FaceInput> = Vec::new();
	for f in s.faces() {
		let poly = s.face_polygon(f);
		let base = positions.len() as u32;
		for p in &poly {
			let dir = DVec3::new(rng.next_f64() * 2.0 - 1.0, rng.next_f64() * 2.0 - 1.0, rng.next_f64() * 2.0 - 1.0).normalize_or_zero();
			let mag = lo + (hi - lo) * rng.next_f64();
			positions.push(*p + dir * mag);
		}
		faces.push(FaceInput { boundary: (base..base + poly.len() as u32).collect(), surface: s.face(f).surface });
	}
	Solid::from_faces(positions, faces)
}

/// The socket-notched plate + bowtie key whose true overlap is two thin
/// parallel-flank sliver strips — the documented arrangement degeneracy that
/// mis-stitches and is refused (recovery_needle_weld.rs / FRICTION #23).
fn notch_sliver_pair() -> (Solid, Solid) {
	let plate_prof: Vec<DVec2> = [(-20.0, 1.0), (-3.0, 1.0), (-4.5, 3.5), (4.5, 3.5), (3.0, 1.0), (20.0, 1.0), (20.0, 7.0), (-20.0, 7.0)]
		.iter()
		.map(|&(x, y)| DVec2::new(x, y))
		.collect();
	let plate = extrude(&plate_prof, 25.0);
	let bowtie: Vec<DVec2> = [(-2.8, 0.0), (-4.21, 2.35), (4.21, 2.35), (2.8, 0.0), (4.21, -2.35), (-4.21, -2.35)]
		.iter()
		.map(|&(x, y)| DVec2::new(x, y))
		.collect();
	let key = extrude(&bowtie, 27.0).transformed(DAffine3::from_translation(v(0.0, 0.0, -1.0)));
	(plate, key)
}

// --- Tests ----------------------------------------------------------------------

#[test]
fn clean_boolean_reports_exact_and_is_bit_identical_to_the_raw_op() {
	// A through-bore in a plate is a valid genus-1 boolean. The policy must report
	// the EXACT tier with error bound 0, hand back the BIT-IDENTICAL raw-op solid
	// (this is instrumentation, not a new algorithm), and its verdict must agree
	// with the strict checked API (which validates the same result).
	let plate = cuboid(v(-10.0, -10.0, -3.0), v(10.0, 10.0, 3.0));
	let bore = cylinder(v(0.0, 0.0, -4.0), DVec3::Z, 2.5, 8.0, 48);
	let out = boolean_with_policy(&plate, &bore, MeshBoolOp::Difference, 1e-6);
	let raw = difference(&plate, &bore);
	let solid = out.solid.as_ref().expect("a clean drilled plate must produce a solid");
	let strict_ok = try_difference(&plate, &bore).is_ok();
	assert!(
		out.is_exact()
			&& out.path == BooleanPath::Exact
			&& out.error_bound() == Some(0.0)
			&& out.op == "difference"
			&& out.validity.is_valid()
			&& out.validity.genus == 1
			&& volume(solid).to_bits() == volume(&raw).to_bits()
			&& solid.face_count() == raw.face_count()
			&& strict_ok,
		"clean difference must report EXACT, bound 0, bit-identical to raw, and agree with try_difference: \
		 path={:?} bound={:?} vol {} vs {} faces {} vs {} genus={} strict_ok={strict_ok}",
		out.path,
		out.error_bound(),
		volume(solid),
		volume(&raw),
		solid.face_count(),
		raw.face_count(),
		out.validity.genus,
	);
}

#[test]
fn cracked_operand_falls_back_via_the_heal_tier_and_only_the_heal_tier() {
	// A 10 mm cube exploded into 6 quads with private vertices perturbed 2e-5..1e-4
	// (gaps far above the boolean's weld EPS), unioned with an overlapping cube. The
	// exact tier cannot close it (strict try_union refuses); the HEALED fallback at
	// 1e-3 re-welds the cracks and the identical union validates. Disabling the heal
	// tier (heal_tol = 0) must REFUSE the very same input — proving it was the
	// fallback, not the exact tier, that rescued it. Error bound = the heal tol.
	let a = cracked(&cuboid(DVec3::ZERO, DVec3::splat(10.0)), 2e-5, 1e-4, 0xbad5_eed5_0000_0042);
	let b = cuboid(v(5.0, 5.0, 5.0), v(15.0, 15.0, 15.0));
	let strict_refused = try_union(&a, &b).is_err();
	let healed = boolean_with_policy(&a, &b, MeshBoolOp::Union, 1e-3);
	let no_heal = boolean_with_policy(&a, &b, MeshBoolOp::Union, 0.0);
	let solid = healed.solid.as_ref();
	// Inclusion–exclusion for two 10-cubes overlapping in a 5-cube: 1000+1000-125.
	let vol = solid.map(volume).unwrap_or(f64::NAN);
	assert!(
		strict_refused
			&& healed.fell_back()
			&& healed.path == BooleanPath::HealedFallback { tol: 1e-3 }
			&& healed.error_bound() == Some(1e-3)
			&& healed.validity.is_valid()
			&& (vol - 1875.0).abs() < 5e-2
			&& no_heal.refused()
			&& no_heal.solid.is_none(),
		"cracked union must take the HEALED tier (bound 1e-3) while exact refuses, and REFUSE with the heal disabled: \
		 strict_refused={strict_refused} healed_path={:?} bound={:?} vol={vol} no_heal_path={:?}",
		healed.path,
		healed.error_bound(),
		no_heal.path,
	);
}

#[test]
fn coincident_face_degeneracy_is_refused_honestly_not_papered_over() {
	// The notch-plate sliver overlap (FRICTION #23): the arrangement mis-stitches
	// the two parallel-flank strips in every op. The policy must report REFUSED —
	// with NO solid and NO error bound — even though a heal tolerance was offered,
	// because the heal tier welds cracks, it does not resolve coincident-face
	// arrangement degeneracies. The reported path must match ground truth: the
	// strict try_difference refuses this exact pair.
	let (plate, key) = notch_sliver_pair();
	let out = boolean_with_policy(&key, &plate, MeshBoolOp::Difference, 1e-3);
	let strict_refused = try_difference(&key, &plate).is_err();
	let raw_invalid = !validate(&difference(&key, &plate)).is_valid();
	assert!(
		out.refused()
			&& out.path == BooleanPath::Refused
			&& out.solid.is_none()
			&& out.error_bound().is_none()
			&& !out.validity.is_valid()
			&& out.op == "difference"
			&& strict_refused
			&& raw_invalid,
		"notch-sliver difference must report REFUSED (no solid, no bound) matching reality — heal must NOT paper it over: \
		 path={:?} solid_some={} bound={:?} valid={} strict_refused={strict_refused} raw_invalid={raw_invalid}",
		out.path,
		out.solid.is_some(),
		out.error_bound(),
		out.validity.is_valid(),
	);
}

#[test]
fn boolean_stats_aggregate_the_path_breakdown_over_a_mixed_batch() {
	// The measurable metric: fold a batch that deliberately spans all three
	// reachable tiers into BooleanStats and read the breakdown/rates. Two clean
	// booleans (EXACT), one cracked union (HEALED), one notch degeneracy (REFUSED)
	// ⇒ 2/1/1 out of 4. This is where fallback-/refusal-rate is honestly measured:
	// the hardened fuzz corpus is ~100% EXACT by construction, so a mix like this,
	// not the corpus, is what exercises the fallback tiers.
	let plate = cuboid(v(-10.0, -10.0, -3.0), v(10.0, 10.0, 3.0));
	let bore = cylinder(v(0.0, 0.0, -4.0), DVec3::Z, 2.5, 8.0, 48);
	let box_a = cuboid(DVec3::ZERO, DVec3::splat(10.0));
	let box_b = cuboid(v(5.0, 5.0, 5.0), v(15.0, 15.0, 15.0));
	let cracked_a = cracked(&box_a, 2e-5, 1e-4, 0xbad5_eed5_0000_0042);
	let (n_plate, n_key) = notch_sliver_pair();

	let mut stats = BooleanStats::default();
	for out in [
		boolean_with_policy(&plate, &bore, MeshBoolOp::Difference, 1e-6),    // EXACT
		boolean_with_policy(&box_a, &box_b, MeshBoolOp::Union, 1e-6),        // EXACT
		boolean_with_policy(&cracked_a, &box_b, MeshBoolOp::Union, 1e-3),    // HEALED
		boolean_with_policy(&n_key, &n_plate, MeshBoolOp::Difference, 1e-3), // REFUSED
	] {
		stats.record(&out);
	}
	assert!(
		stats == BooleanStats { exact: 2, healed_fallback: 1, refused: 1 }
			&& stats.total() == 4
			&& (stats.exact_rate() - 0.5).abs() < 1e-12
			&& (stats.fallback_rate() - 0.25).abs() < 1e-12
			&& (stats.refusal_rate() - 0.25).abs() < 1e-12,
		"mixed batch must aggregate to 2 exact / 1 healed / 1 refused with matching rates: {stats:?} \
		 (exact={:.3} fallback={:.3} refusal={:.3})",
		stats.exact_rate(),
		stats.fallback_rate(),
		stats.refusal_rate(),
	);
}
