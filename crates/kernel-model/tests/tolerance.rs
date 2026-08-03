// Copyright (c) LMCAD. Licensed under the MIT License.

//! Pinning tests for `kernel_model::tolerance` — stack-up over assembly
//! chains. Every arithmetic claim here is hand-computed in the comment above
//! the assertion; the assertions carry the measured values.

use kernel_core::math::{Affine3A, DVec3, Vec3};
use kernel_model::process::FdmProfile;
use kernel_model::tolerance::{
	ChainLink, Contribution, Dimension, PrintedFeature, Sign, Stack, StackMethod, ToleranceError, DEFAULT_SIGMA_LEVEL,
};
use kernel_model::{Assembly, Document, Instance};

/// The reference chain both accumulation gates use: a shaft-in-housing stack.
///
/// housing_depth 20.00 ±0.20 (adds) − bearing 7.00 ±0.10 − spacer 4.50 ±0.05
/// − shim 1.00 ±0.02  ⇒  nominal gap 7.50 mm.
fn four_link_chain() -> Stack {
	Stack::new("shaft_end_float")
		.with(Contribution::declared("housing_depth", Sign::Adds, Dimension::symmetric(20.0, 0.20)))
		.with(Contribution::catalog("bearing_width", Sign::Subtracts, Dimension::symmetric(7.0, 0.10), "608ZZ"))
		.with(Contribution::declared("spacer", Sign::Subtracts, Dimension::symmetric(4.5, 0.05)))
		.with(Contribution::declared("shim", Sign::Subtracts, Dimension::symmetric(1.0, 0.02)))
}

/// Hand arithmetic, four links, both methods, to machine precision.
///
/// nominal = 20 − 7 − 4.5 − 1 = **7.50**
/// worst case: Σ|tol| = 0.20 + 0.10 + 0.05 + 0.02 = **0.37**
///   ⇒ [7.13, 7.87], span 0.74
/// RSS: √(0.20² + 0.10² + 0.05² + 0.02²) = √0.0529 = **0.23** exactly
///   ⇒ [7.27, 7.73], span 0.46
/// worst-case shares: 0.20/0.37 = 54.054 %, then 0.10, 0.05, 0.02
/// RSS shares (variance share, h²/H): 0.04/0.23 = 0.173913 (75.652 %),
///   0.01/0.23 = 0.043478, 0.0025/0.23 = 0.010870, 0.0004/0.23 = 0.001739
///   — and they sum to 0.23, the stack half-band, exactly.
#[test]
fn four_link_chain_matches_hand_arithmetic_under_both_methods() {
	let s = four_link_chain();
	let wc = s.worst_case();
	let rss = s.rss(DEFAULT_SIGMA_LEVEL);
	let wc_share_sum: f64 = wc.contributors.iter().map(|c| c.contribution).sum();
	let rss_share_sum: f64 = rss.contributors.iter().map(|c| c.contribution).sum();
	let rss_fraction_sum: f64 = rss.contributors.iter().map(|c| c.fraction).sum();
	assert!(
		(wc.nominal - 7.5).abs() < 1e-12
			&& (wc.mean - 7.5).abs() < 1e-12
			&& (wc.half_band - 0.37).abs() < 1e-12
			&& (wc.min - 7.13).abs() < 1e-12
			&& (wc.max - 7.87).abs() < 1e-12
			&& (wc.span - 0.74).abs() < 1e-12
			&& (rss.half_band - 0.23).abs() < 1e-12
			&& (rss.min - 7.27).abs() < 1e-12
			&& (rss.max - 7.73).abs() < 1e-12
			&& wc.contributors[0].name == "housing_depth"
			&& rss.contributors[0].name == "housing_depth"
			&& (wc.contributors[0].fraction - 0.20 / 0.37).abs() < 1e-12
			&& (rss.contributors[0].contribution - 0.04 / 0.23).abs() < 1e-12
			&& (wc_share_sum - wc.half_band).abs() < 1e-12
			&& (rss_share_sum - rss.half_band).abs() < 1e-12
			&& (rss_fraction_sum - 1.0).abs() < 1e-12
			&& rss.assumption.contains("3-sigma")
			&& rss.assumption.contains("INDEPENDENCE IS ASSUMED")
			&& wc.assumption.contains("hard bound"),
		"4-link chain: worst case nominal {:.12} (want 7.5), half_band {:.12} (want 0.37), [{:.12}, {:.12}] (want [7.13, 7.87]); \
		 RSS half_band {:.12} (want 0.23 = sqrt(0.0529)), [{:.12}, {:.12}] (want [7.27, 7.73]); dominant wc '{}' frac {:.6} \
		 (want housing_depth 0.540541), dominant rss '{}' contribution {:.12} (want 0.173913); share sums wc {:.12} vs {:.12}, \
		 rss {:.12} vs {:.12}, rss fractions {:.12} (want 1.0); rss assumption carried: {}",
		wc.nominal,
		wc.half_band,
		wc.min,
		wc.max,
		rss.half_band,
		rss.min,
		rss.max,
		wc.contributors[0].name,
		wc.contributors[0].fraction,
		rss.contributors[0].name,
		rss.contributors[0].contribution,
		wc_share_sum,
		wc.half_band,
		rss_share_sum,
		rss.half_band,
		rss_fraction_sum,
		rss.assumption
	);
}

/// The classic demonstration: **worst case ⊇ RSS ∋ nominal** for a stack of
/// symmetric contributions. Both intervals share the same centre; RSS is
/// strictly narrower whenever two or more links carry a band, because
/// √(Σh²) < Σh.
#[test]
fn worst_case_contains_rss_contains_nominal() {
	let s = four_link_chain();
	let wc = s.worst_case();
	let rss = s.rss(DEFAULT_SIGMA_LEVEL);
	assert!(
		wc.min < rss.min
			&& rss.min < wc.nominal
			&& wc.nominal < rss.max
			&& rss.max < wc.max
			&& (wc.mean - rss.mean).abs() < 1e-15
			&& rss.span < wc.span,
		"containment ordering: worst [{:.6}, {:.6}] must strictly contain rss [{:.6}, {:.6}], which must strictly contain the \
		 nominal {:.6}; shared centre wc {:.15} vs rss {:.15}; spans {:.6} (wc) vs {:.6} (rss)",
		wc.min,
		wc.max,
		rss.min,
		rss.max,
		wc.nominal,
		wc.mean,
		rss.mean,
		wc.span,
		rss.span
	);
}

/// **The whole point of the module.** Six links, each with a ±0.08 band that
/// comfortably fits the ±0.30 variation budget on its own — and a chain that
/// blows the budget by 0.18 mm at both ends.
///
/// nominal: 20.00 − 4.00 − 4.00 − 4.00 − 4.00 − 3.50 = **0.50** (the design
/// clearance, dead-centre in the required [0.20, 0.80] window).
/// worst case half-band: 6 × 0.08 = **0.48** ⇒ [0.02, 0.98]; undershoot and
/// overshoot are both 0.18.
/// RSS half-band: 0.08·√6 = **0.195959…** ⇒ [0.304, 0.696] — RSS PASSES the
/// same window the hard bound fails, which is exactly why a load-bearing fit
/// is gated on worst case.
#[test]
fn aggregate_fails_while_every_link_passes_alone() {
	let mut s = Stack::new("stacked_spacer_train");
	s.push(Contribution::declared("housing", Sign::Adds, Dimension::symmetric(20.0, 0.08)));
	for (i, nom) in [4.0, 4.0, 4.0, 4.0, 3.5].iter().enumerate() {
		s.push(Contribution::declared(format!("spacer_{i}"), Sign::Subtracts, Dimension::symmetric(*nom, 0.08)));
	}
	let target = Dimension::window(0.20, 0.80);
	let wc_report = s.gate_report(StackMethod::WorstCase, target);
	let rss_report = s.gate_report(StackMethod::Rss { sigma_level: DEFAULT_SIGMA_LEVEL }, target);
	let violation = wc_report.aggregate.as_ref().expect("worst case must violate the window");

	// Each link's band, alone at the design nominal, passes its own gate — the
	// pairwise review every one of these dimensions would have survived. (The
	// sign is the chain's business, not the band's: a one-link stack is
	// measured in the chain direction, so it carries Sign::Adds.)
	let alone_all_pass = (0..s.len()).all(|i| {
		Stack::new("alone")
			.with(Contribution::declared("link", Sign::Adds, Dimension::symmetric(0.50, s.contributions[i].dim.plus)))
			.gate(target)
			.is_ok()
	});

	assert!(
		wc_report.aggregate_only_failure
			&& wc_report.links_passing_alone.iter().all(|(_, ok)| *ok)
			&& alone_all_pass
			&& (violation.achieved_min - 0.02).abs() < 1e-12
			&& (violation.achieved_max - 0.98).abs() < 1e-12
			&& (violation.low_excess - 0.18).abs() < 1e-12
			&& (violation.high_excess - 0.18).abs() < 1e-12
			&& violation.dominant.as_ref().is_some_and(|d| (d.half_band - 0.08).abs() < 1e-12)
			&& rss_report.aggregate.is_none()
			&& (rss_report.result.half_band - 0.08 * 6.0f64.sqrt()).abs() < 1e-12
			&& wc_report.verdict.contains("aggregate only"),
		"aggregate-only failure: verdict '{}', aggregate_only_failure {}, every link passes alone (report {}, one-link gate {}), \
		 achieved [{:.12}, {:.12}] (want [0.02, 0.98]) vs required [{:.4}, {:.4}], undershoot {:.12} / overshoot {:.12} (want \
		 0.18 each), dominant half-band {:?} (want 0.08); RSS half-band {:.12} (want 0.195959, = 0.08*sqrt(6)) PASSES the same \
		 window: {}",
		wc_report.verdict,
		wc_report.aggregate_only_failure,
		wc_report.links_passing_alone.iter().all(|(_, ok)| *ok),
		alone_all_pass,
		violation.achieved_min,
		violation.achieved_max,
		violation.required_min,
		violation.required_max,
		violation.low_excess,
		violation.high_excess,
		violation.dominant.as_ref().map(|d| d.half_band),
		rss_report.result.half_band,
		rss_report.aggregate.is_none()
	);
}

/// Asymmetric bands mirror correctly under a sign flip, and the one honest
/// exception to the containment ordering is pinned rather than hidden.
///
/// A single 10.00 +0.30/−0.05 link: mid 10.125, half-band 0.175, worst-case
/// interval [9.95, 10.30]. Flipped to `Subtracts` the whole interval mirrors
/// to [−10.30, −9.95] — `up`/`down` swap with the sign.
///
/// Two 10.00 +1.00/−0.00 links: nominal 20.00 but mid-shifted mean 21.00,
/// RSS half-band √(0.5² + 0.5²) = 0.70711 ⇒ [20.293, 21.707], which does NOT
/// contain the nominal 20.00. Worst case [20.00, 22.00] always does. This is
/// stated in `Stack::rss`'s doc and asserted here so nobody quotes the
/// symmetric ordering as universal.
#[test]
fn asymmetric_bands_mirror_and_rss_can_exclude_the_nominal() {
	let plus_side = Stack::new("asym").with(Contribution::declared("boss", Sign::Adds, Dimension::asymmetric(10.0, 0.30, 0.05)));
	let minus_side = Stack::new("asym").with(Contribution::declared("boss", Sign::Subtracts, Dimension::asymmetric(10.0, 0.30, 0.05)));
	let a = plus_side.worst_case();
	let b = minus_side.worst_case();

	let one_sided = Stack::new("one_sided")
		.with(Contribution::declared("grow_a", Sign::Adds, Dimension::asymmetric(10.0, 1.0, 0.0)))
		.with(Contribution::declared("grow_b", Sign::Adds, Dimension::asymmetric(10.0, 1.0, 0.0)));
	let o_wc = one_sided.worst_case();
	let o_rss = one_sided.rss(DEFAULT_SIGMA_LEVEL);

	assert!(
		(a.mean - 10.125).abs() < 1e-12
			&& (a.half_band - 0.175).abs() < 1e-12
			&& (a.min - 9.95).abs() < 1e-12
			&& (a.max - 10.30).abs() < 1e-12
			&& (b.min + 10.30).abs() < 1e-12
			&& (b.max + 9.95).abs() < 1e-12
			&& (b.mean + 10.125).abs() < 1e-12
			&& (o_wc.nominal - 20.0).abs() < 1e-12
			&& (o_wc.min - 20.0).abs() < 1e-12
			&& (o_wc.max - 22.0).abs() < 1e-12
			&& (o_rss.mean - 21.0).abs() < 1e-12
			&& (o_rss.half_band - 0.5 * 2.0f64.sqrt()).abs() < 1e-12
			&& o_rss.min > o_rss.nominal,
		"asymmetric handling: +side mean {:.12} (want 10.125) half {:.12} (want 0.175) [{:.12}, {:.12}] (want [9.95, 10.30]); \
		 -side mirrors to [{:.12}, {:.12}] (want [-10.30, -9.95]) mean {:.12} (want -10.125); one-sided pair nominal {:.12} \
		 (want 20.0), worst [{:.12}, {:.12}] (want [20, 22], contains the nominal), rss mean {:.12} (want 21.0) half {:.12} \
		 (want 0.707107) min {:.12} which is ABOVE the nominal {:.12} — the documented exception",
		a.mean,
		a.half_band,
		a.min,
		a.max,
		b.min,
		b.max,
		b.mean,
		o_wc.nominal,
		o_wc.min,
		o_wc.max,
		o_rss.mean,
		o_rss.half_band,
		o_rss.min,
		o_rss.nominal
	);
}

/// **Negative control — a mis-signed link.** Flipping `bearing_width` from
/// `Subtracts` to `Adds` moves the chain nominal by exactly `2 × 7.00 = 14.00`
/// (7.50 → 21.50), the band is unchanged (a sign mirrors a band, it never
/// widens it), the correct chain PASSES the [7.00, 8.00] window and the
/// mis-signed one FAILS it by 13.87 mm. A silently wrong sign cannot survive
/// the gate.
#[test]
fn mis_signed_link_is_wrong_by_twice_the_dimension_and_the_gate_convicts() {
	let good = four_link_chain();
	let mut bad = four_link_chain();
	bad.contributions[1].sign = bad.contributions[1].sign.flipped();
	let target = Dimension::window(7.0, 8.0);
	let g = good.worst_case();
	let b = bad.worst_case();
	let good_gate = good.gate(target);
	let bad_gate = bad.gate(target);
	let bad_violation = bad_gate.as_ref().expect_err("the mis-signed chain must be convicted");
	assert!(
		good_gate.is_ok()
			&& (b.nominal - g.nominal - 14.0).abs() < 1e-12
			&& (b.half_band - g.half_band).abs() < 1e-12
			&& (bad_violation.high_excess - 13.87).abs() < 1e-12
			&& bad_violation.low_excess == 0.0
			&& bad_violation.malformed.is_none(),
		"mis-sign control: correct chain gate {:?} (want Ok) nominal {:.12}; mis-signed nominal {:.12}, difference {:.12} (want \
		 exactly 14.0 = 2 x 7.0), half-band unchanged {:.12} vs {:.12}, overshoot {:.12} (want 13.87), undershoot {:.12} (want 0)",
		good_gate.is_ok(),
		g.nominal,
		b.nominal,
		b.nominal - g.nominal,
		g.half_band,
		b.half_band,
		bad_violation.high_excess,
		bad_violation.low_excess
	);
}

/// Malformed stacks are refused loudly by the gate path, and the arithmetic
/// path says so in its doc rather than pretending. Three refusals: a negative
/// band, an empty chain, and a meaningless RSS sigma level.
#[test]
fn malformed_stacks_are_refused_with_typed_errors() {
	let neg = Stack::new("neg").with(Contribution::declared("bad", Sign::Adds, Dimension::asymmetric(10.0, -0.1, 0.1)));
	let empty = Stack::new("empty");
	let ok = four_link_chain();
	let target = Dimension::window(0.0, 100.0);

	let neg_err = neg.validate().expect_err("negative band must be refused");
	let empty_err = empty.validate().expect_err("empty stack must be refused");
	let sigma_err = StackMethod::Rss { sigma_level: 0.0 }.validate().expect_err("zero sigma must be refused");
	let neg_gate = neg.gate(target).expect_err("gate must refuse a malformed stack");
	let sigma_gate = ok.gate_method(StackMethod::Rss { sigma_level: -1.0 }, target).expect_err("gate must refuse a bad sigma");

	assert!(
		matches!(neg_err, ToleranceError::NegativeBand { plus, .. } if (plus + 0.1).abs() < 1e-12)
			&& matches!(empty_err, ToleranceError::EmptyStack { .. })
			&& matches!(sigma_err, ToleranceError::BadSigmaLevel { sigma_level } if sigma_level == 0.0)
			&& neg_gate.malformed.is_some()
			&& neg_gate.achieved_min.is_nan()
			&& neg_gate.dominant.is_none()
			&& sigma_gate.malformed.is_some()
			&& neg_gate.message.contains("non-negative")
			&& sigma_gate.message.contains("sigma level"),
		"typed refusals: negative band -> {neg_err}; empty -> {empty_err}; sigma -> {sigma_err}; gate on the negative band is \
		 malformed={:?} with NaN achieved_min={} and no dominant={}; gate on sigma=-1 is malformed={:?}",
		neg_gate.malformed,
		neg_gate.achieved_min,
		neg_gate.dominant.is_none(),
		sigma_gate.malformed
	);
}

/// **The process seam.** A contribution's band can come from an
/// [`FdmProfile`] instead of a literal, and the derivation says what it does
/// and does not cover.
///
/// With `first_layer_comp = 0.12` and `seam_allowance = 0.05`, an outer
/// printed surface's band is one-sided `+0.17/−0.00` — it can only grow.
/// With `hole_diameter_comp = 0.10`, an uncompensated Ø6 hole measures
/// 6.00 − 0.10 = **5.90**, and the band is ZERO because the profile carries no
/// measured hole scatter. That zero is asserted together with the note that
/// says so — what is NOT asserted here is any statistical spread for a printed
/// hole, because the profile does not contain one.
#[test]
fn process_seam_derives_bands_from_the_fdm_profile() {
	let profile = FdmProfile {
		name: "test_printer".to_string(),
		first_layer_comp: 0.12,
		seam_allowance: 0.05,
		hole_diameter_comp: 0.10,
		..FdmProfile::conservative_default()
	};
	let outer = Contribution::printed_feature("boss_od", Sign::Adds, 12.0, &profile, PrintedFeature::OuterSurfaceRadial);
	let hole = Contribution::printed_feature("pin_bore", Sign::Subtracts, 6.0, &profile, PrintedFeature::UncompensatedHoleDiametral);
	// The generic hook: a band from a process model this module does not own.
	let external = Contribution::from_process_tolerance(
		"cnc_slot",
		Sign::Adds,
		8.0,
		0.02,
		0.02,
		"haas_vf2",
		"end_mill_slot",
		"supplier CMM study, 30 parts",
	);
	let s = Stack::new("printed_chain").with(outer.clone()).with(hole.clone()).with(external.clone());
	let wc = s.worst_case();
	assert!(
		(outer.dim.plus - 0.17).abs() < 1e-12
			&& outer.dim.minus == 0.0
			&& outer.source.label() == "process:test_printer/outer_surface_radial"
			&& (hole.dim.nominal - 5.90).abs() < 1e-12
			&& hole.dim.plus == 0.0
			&& hole.dim.minus == 0.0
			&& hole.source.label() == "process:test_printer/uncompensated_hole_diametral"
			&& external.source.label() == "process:haas_vf2/end_mill_slot"
			&& (wc.nominal - (12.0 - 5.90 + 8.0)).abs() < 1e-12
			&& (wc.half_band - (0.085 + 0.0 + 0.02)).abs() < 1e-12
			&& wc.contributors[0].name == "boss_od",
		"process seam: outer +{:.12}/-{:.12} (want +0.17/-0.00) source '{}'; uncompensated hole nominal {:.12} (want 5.90, a \
		 BIAS) with band +{:.12}/-{:.12} (want zero — the profile has no hole scatter); generic hook source '{}'; chain nominal \
		 {:.12} (want 14.10), half-band {:.12} (want 0.105), dominant '{}'",
		outer.dim.plus,
		outer.dim.minus,
		outer.source.label(),
		hole.dim.nominal,
		hole.dim.plus,
		hole.dim.minus,
		external.source.label(),
		wc.nominal,
		wc.half_band,
		wc.contributors[0].name
	);
}

/// The mate-chain constructor: nominals are READ OUT of the poses (so the
/// classic mis-sign cannot happen), and a link with no direction on the chain
/// axis is refused rather than guessed.
///
/// Poses at x = 0, 30, 75, 60. Links 0→1 (+30), 1→2 (+45), 2→3 (−15) give a
/// chain nominal of 30 + 45 − 15 = **60**, which is exactly the axial distance
/// from pose 0 to pose 3.
#[test]
fn pose_chain_reads_nominals_from_geometry_and_refuses_a_degenerate_link() {
	let poses: Vec<Affine3A> =
		[0.0f32, 30.0, 75.0, 60.0].iter().map(|&x| Affine3A::from_translation(Vec3::new(x, 0.0, 0.0))).collect();
	let links = vec![
		ChainLink::symmetric("base_to_a", 0, 1, 0.05),
		ChainLink::symmetric("a_to_b", 1, 2, 0.05),
		ChainLink::symmetric("b_to_c", 2, 3, 0.05),
	];
	let s = Stack::from_pose_chain("axial", &poses, DVec3::X, &links).expect("chain must build");
	let wc = s.worst_case();
	// The same chain read straight off an Assembly's instance poses.
	let mut asm = Assembly::new();
	for &pose in &poses {
		asm.add(Instance::document(Document::new(), pose));
	}
	let from_asm = Stack::from_assembly_chain("axial", &asm, DVec3::X, &links).expect("assembly chain must build");
	// A link whose two ends sit at the same station on the axis has no sign.
	let flat = Stack::from_pose_chain("flat", &poses, DVec3::Y, &links).expect_err("a Y-axis chain has no direction here");
	let oob = Stack::from_pose_chain("oob", &poses, DVec3::X, &[ChainLink::symmetric("nope", 0, 9, 0.05)])
		.expect_err("an out-of-range instance must be refused");
	assert!(
		(wc.nominal - 60.0).abs() < 1e-5
			&& s.contributions[0].sign == Sign::Adds
			&& s.contributions[2].sign == Sign::Subtracts
			&& (s.contributions[2].dim.nominal - 15.0).abs() < 1e-5
			&& (wc.half_band - 0.15).abs() < 1e-12
			&& matches!(flat, ToleranceError::DegenerateChainLink { .. })
			&& matches!(oob, ToleranceError::ChainIndexOutOfRange { index: 9, poses: 4, .. })
			&& from_asm == s,
		"pose chain: nominal {:.9} (want 60.0 = 30 + 45 - 15, f32 poses so 1e-5 tolerance), signs [{:?}, {:?}, {:?}], third \
		 link nominal {:.9} (want 15.0), half-band {:.12} (want 0.15); the Assembly-sourced chain is identical: {}; refusals: \
		 {flat} / {oob}",
		wc.nominal,
		s.contributions[0].sign,
		s.contributions[1].sign,
		s.contributions[2].sign,
		s.contributions[2].dim.nominal,
		wc.half_band,
		from_asm == s
	);
}
