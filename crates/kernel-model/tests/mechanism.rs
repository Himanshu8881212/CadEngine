// Copyright (c) LMCAD. Licensed under the MIT License.

//! Pinning tests for `kernel_model::mechanism` — motion over time.
//!
//! The four-bar and slider-crank gates check the solver against closed-form
//! linkage geometry (law of cosines / the slider-crank displacement equation)
//! to machine precision; the interference gate proves the capability's whole
//! justification, namely that a pair which is CLEAR at both endpoints of a
//! motion can interpenetrate in between; and the locked gates prove the
//! mobility refusal fires with the right Kutzbach count.

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use kernel_brep::{cuboid, tessellate_default};
use kernel_core::math::{DVec2, DVec3};
use kernel_model::mechanism::{Joint, Link, Mechanism, MechanismError, Pose2};

// Four-bar geometry: ground 100, crank 30, coupler 90, rocker 60.
const R1: f64 = 100.0;
const R2: f64 = 30.0;
const R3: f64 = 90.0;
const R4: f64 = 60.0;

/// The reference crank-rocker. Grashof: s + l = 30 + 100 = 130 < p + q =
/// 90 + 60 = 150, and the shortest link (the crank) is adjacent to ground, so
/// the crank fully rotates and the rocker oscillates.
///
/// Declared on the UPPER assembly branch at crank θ = 90°: the crank pin sits
/// at (0, 30) and the coupler/rocker pin at ≈ (85.455, 58.211).
fn four_bar() -> Mechanism {
	let mut m = Mechanism::new("crank_rocker");
	m.add_link(Link::new("ground", Pose2::new(0.0, 0.0, 0.0)));
	m.add_link(Link::new("crank", Pose2::new(0.0, 0.0, FRAC_PI_2)));
	m.add_link(Link::new("coupler", Pose2::new(0.0, 30.0, 0.31895)));
	m.add_link(Link::new("rocker", Pose2::new(R1, 0.0, 1.81591)));
	m.add_joint(Joint::revolute("crank_pivot", 0, DVec2::ZERO, 1, DVec2::ZERO).driven());
	m.add_joint(Joint::revolute("crank_pin", 1, DVec2::new(R2, 0.0), 2, DVec2::ZERO));
	m.add_joint(Joint::revolute("coupler_pin", 2, DVec2::new(R3, 0.0), 3, DVec2::new(R4, 0.0)));
	m.add_joint(Joint::revolute("rocker_pivot", 3, DVec2::ZERO, 0, DVec2::new(R1, 0.0)));
	m.track("coupler_tip", 2, DVec2::new(R3, 0.0));
	m
}

/// A slider-crank: crank 30, coupler 90, slider on the X axis. Declared at
/// crank θ = 0, where the slider sits at its outer dead centre `l + r = 120`.
fn slider_crank(ground_mesh: Option<kernel_core::mesh::Mesh>, coupler_mesh: Option<kernel_core::mesh::Mesh>) -> Mechanism {
	let mut m = Mechanism::new("slider_crank");
	let mut ground = Link::new("ground", Pose2::new(0.0, 0.0, 0.0));
	ground.mesh = ground_mesh;
	let mut coupler = Link::new("coupler", Pose2::new(R2, 0.0, 0.0));
	coupler.mesh = coupler_mesh;
	m.add_link(ground);
	m.add_link(Link::new("crank", Pose2::new(0.0, 0.0, 0.0)));
	m.add_link(coupler);
	m.add_link(Link::new("slider", Pose2::new(R2 + R3, 0.0, 0.0)));
	m.add_joint(Joint::revolute("crank_pivot", 0, DVec2::ZERO, 1, DVec2::ZERO).driven());
	m.add_joint(Joint::revolute("crank_pin", 1, DVec2::new(R2, 0.0), 2, DVec2::ZERO));
	m.add_joint(Joint::revolute("wrist_pin", 2, DVec2::new(R3, 0.0), 3, DVec2::ZERO));
	m.add_joint(Joint::prismatic("slide", 0, DVec2::ZERO, DVec2::X, 3, DVec2::ZERO));
	m
}

/// **Four-bar mobility and range of motion, against the law of cosines.**
///
/// Kutzbach: F = 3(n−1) − 2j₁ − j₂ = 3(4−1) − 2·4 − 0 = **1**.
///
/// The rocker's extremes occur where crank and coupler are collinear, so the
/// coupler/rocker pin sits at |OB| = r₂+r₃ = 120 (extended) or r₃−r₂ = 60
/// (folded). With ψ the angle at the rocker pivot measured from the ground
/// line, the law of cosines on triangle O–C–B gives
///   cos ψ = (r₁² + r₄² − |OB|²) / (2·r₁·r₄)
///   extended: (10000 + 3600 − 14400)/12000 = −1/15  ⇒ ψ = 93.8223°
///   folded:   (10000 + 3600 −  3600)/12000 =  5/6   ⇒ ψ = 33.5573°
/// and the rocker's body angle is θ₄ = π − ψ, i.e. **86.1777° … 146.4427°**,
/// a swing of **60.2650°**.
///
/// The crank angles that produce them come from the same triangle on O:
///   extended: cos θ = (120² + r₁² − r₄²)/(2·120·r₁) = 20800/24000 = 13/15
///   folded:   cos θ = (60² + r₁² − r₄²)/(2·(−60)·r₁) = −10000/12000 = −5/6
///     (folded puts B at −60·û, opposite the crank, so θ is negated).
#[test]
fn four_bar_has_one_dof_and_a_rocker_range_matching_the_law_of_cosines() {
	let m = four_bar();
	let mob = m.mobility();

	let psi_ext = ((R1 * R1 + R4 * R4 - (R2 + R3) * (R2 + R3)) / (2.0 * R1 * R4)).acos();
	let psi_fold = ((R1 * R1 + R4 * R4 - (R3 - R2) * (R3 - R2)) / (2.0 * R1 * R4)).acos();
	let theta4_ext = PI - psi_ext; // 1.5041331… rad = 86.1777°
	let theta4_fold = PI - psi_fold; // 2.5559068… rad = 146.4427°
	let crank_ext = (((R2 + R3) * (R2 + R3) + R1 * R1 - R4 * R4) / (2.0 * (R2 + R3) * R1)).acos();
	let crank_fold = -(((R3 - R2) * (R3 - R2) + R1 * R1 - R4 * R4) / (-2.0 * (R3 - R2) * R1)).acos();

	let at_ext = m.pose_at(crank_ext).expect("extended limit is reachable");
	let at_fold = m.pose_at(crank_fold).expect("folded limit is reachable");
	let sweep = m.sweep(720).expect("a mobility-1 crank-rocker sweeps a full turn");
	let rocker = &sweep.range_of_motion[3];
	let gap_lo = rocker.theta_min - theta4_ext;
	let gap_hi = theta4_fold - rocker.theta_max;

	// The tracked coupler tip IS the coupler/rocker pin, so it rides the
	// rocker circle of radius r₄ about (100, 0) between the two limits:
	//   x_min = 100 − 60·(5/6)  = 50 exactly      (folded limit, cos ψ = 5/6)
	//   x_max = 100 + 60·(1/15) = 104 exactly     (extended limit, cos ψ = −1/15)
	//   y_max = 60 exactly (the rocker passes vertical, 90° is inside the swing)
	//   y_min = 60·sin(146.4427°) = 10·√11 = 33.166248
	let want_trace = [50.0, 104.0, 10.0 * 11.0f64.sqrt(), 60.0];
	let trace = sweep.traces[0].extents;
	let trace_err = (0..4).fold(0.0f64, |a, i| a.max((trace[i] - want_trace[i]).abs()));

	assert!(
		mob.kutzbach_dof == 1
			&& mob.rank_dof == 1
			&& !mob.paradox
			&& mob.formula.contains("3(4-1) - 2*4 - 0 = 1")
			&& (at_ext[3].theta - theta4_ext).abs() < 1e-9
			&& (at_fold[3].theta - theta4_fold).abs() < 1e-9
			&& rocker.theta_min >= theta4_ext - 1e-9
			&& rocker.theta_max <= theta4_fold + 1e-9
			&& gap_lo < 1e-5
			&& gap_hi < 1e-5
			&& sweep.steps == 721
			&& sweep.max_newton_residual < 1e-10
			&& sweep.max_step_translation_jump < 0.30
			&& sweep.max_step_rotation_jump < 0.010
			&& sweep.traces.len() == 1
			&& trace_err < 5e-4
			&& sweep.first_interference.is_none(),
		"four-bar: {} (kutzbach {} rank {} paradox {}); rocker at the extended limit {:.12} rad vs law-of-cosines {:.12} \
		 (86.1777°), at the folded limit {:.12} vs {:.12} (146.4427°); swept range [{:.9}, {:.9}] sits inside those extremes \
		 with sampling gaps {:.3e} / {:.3e} rad at 720 steps; {} poses, max Newton residual {:.3e}, continuity jumps \
		 {:.6} mm / {:.6} rad per step (crank pin travels r2·Δθ = {:.6} mm); coupler-tip trace extents {:?} vs hand-computed \
		 {:?}, worst {:.3e} mm",
		mob.formula,
		mob.kutzbach_dof,
		mob.rank_dof,
		mob.paradox,
		at_ext[3].theta,
		theta4_ext,
		at_fold[3].theta,
		theta4_fold,
		rocker.theta_min,
		rocker.theta_max,
		gap_lo,
		gap_hi,
		sweep.steps,
		sweep.max_newton_residual,
		sweep.max_step_translation_jump,
		sweep.max_step_rotation_jump,
		R2 * TAU / 720.0,
		trace,
		want_trace,
		trace_err
	);
}

/// **Slider-crank stroke against the closed form.**
///
/// `x(θ) = r·cos θ + √(l² − r² sin²θ)` with r = 30, l = 90:
///   θ = 0   ⇒ x = 30 + 90 = **120** (outer dead centre)
///   θ = π   ⇒ x = −30 + 90 = **60** (inner dead centre)
///   θ = π/3 ⇒ x = 15 + √(8100 − 675) = 15 + √7425 = **101.168…**
/// so the stroke is exactly `2r = 60`, and the swept slider extents must
/// reproduce both dead centres.
#[test]
fn slider_crank_stroke_matches_the_closed_form() {
	let m = slider_crank(None, None);
	let closed_form = |theta: f64| R2 * theta.cos() + (R3 * R3 - R2 * R2 * theta.sin() * theta.sin()).sqrt();
	let at_0 = m.pose_at(0.0).expect("outer dead centre");
	let at_pi = m.pose_at(PI).expect("inner dead centre");
	let at_60 = m.pose_at(PI / 3.0).expect("mid stroke");
	let sweep = m.sweep(360).expect("a mobility-1 slider-crank sweeps a full turn");
	let slider = &sweep.range_of_motion[3];
	let stroke = slider.origin_extents[1] - slider.origin_extents[0];
	let worst = sweep
		.poses_per_step
		.iter()
		.zip(&sweep.driven_value)
		.fold(0.0f64, |acc, (p, &q)| acc.max((p[3].x - closed_form(q)).abs()));

	assert!(
		m.mobility().kutzbach_dof == 1
			&& (at_0[3].x - 120.0).abs() < 1e-9
			&& (at_pi[3].x - 60.0).abs() < 1e-9
			&& (at_60[3].x - (15.0 + 7425.0f64.sqrt())).abs() < 1e-9
			&& at_0[3].y.abs() < 1e-9
			&& (stroke - 2.0 * R2).abs() < 1e-9
			&& (slider.origin_extents[1] - (R3 + R2)).abs() < 1e-9
			&& (slider.origin_extents[0] - (R3 - R2)).abs() < 1e-9
			&& worst < 1e-9
			&& sweep.max_newton_residual < 1e-10,
		"slider-crank: x(0) {:.12} (want 120 = l + r), x(π) {:.12} (want 60 = l − r), x(π/3) {:.12} (want {:.12} = 15 + √7425), \
		 slider stays on the rail (y = {:.3e}); swept extents [{:.12}, {:.12}] give stroke {:.12} (want exactly 2r = 60); worst \
		 deviation from the closed form over 361 poses {:.3e}; max Newton residual {:.3e}",
		at_0[3].x,
		at_pi[3].x,
		at_60[3].x,
		15.0 + 7425.0f64.sqrt(),
		at_0[3].y,
		slider.origin_extents[0],
		slider.origin_extents[1],
		stroke,
		worst,
		sweep.max_newton_residual
	);
}

/// **The capability's whole justification.** A post fixed to ground at
/// x ∈ [41, 49], y ∈ [11, 19] and a 6 mm-thick coupler bar. At crank θ = 0 the
/// bar lies along y ∈ [−3, 3] with the post 8 mm away, and at θ = π it lies
/// along the same band, again 8 mm away — so a check at the two ENDPOINTS of
/// the half-cycle sees a comfortable clearance and passes.
///
/// In between, the coupler swings up through the post: the same sweep at 8
/// steps convicts, with `sweep_check`'s exact triangle-crossing oracle. The
/// endpoint-only report must find NOTHING, which is the whole point.
#[test]
fn a_mid_cycle_collision_is_found_where_an_endpoint_check_sees_8_mm_of_clearance() {
	let post = tessellate_default(&cuboid(DVec3::new(41.0, 11.0, -5.0), DVec3::new(49.0, 19.0, 5.0)));
	let bar = tessellate_default(&cuboid(DVec3::new(0.0, -3.0, -3.0), DVec3::new(R3, 3.0, 3.0)));
	let m = slider_crank(Some(post), Some(bar));

	let endpoints = m.sweep_range(0.0, PI, 1).expect("endpoint-only sweep");
	let dense = m.sweep_range(0.0, PI, 8).expect("dense sweep");
	let hit = dense.first_interference.as_ref().expect("the dense sweep must convict");

	assert!(
		endpoints.steps == 2
			&& endpoints.first_interference.is_none()
			&& endpoints.min_clearance_over_cycle > 7.5
			&& endpoints.pair_sweeps.len() == 1
			&& endpoints.pair_sweeps[0].pair == (0, 2)
			&& endpoints.pair_sweeps[0].crossings == 0
			&& dense.steps == 9
			&& hit.step > 0
			&& hit.step < 8
			&& hit.crossing
			&& hit.pair == (0, 2)
			&& hit.driven_value > 0.0
			&& hit.driven_value < PI
			&& dense.pair_sweeps[0].crossings > 0,
		"mid-cycle interference: the ENDPOINT-ONLY sweep ({} poses, θ = 0 and π) finds first_interference = {:?} with \
		 min clearance {:.4} mm over pair {:?} and {} crossing(s) — it misses the collision entirely. The SAME motion at 9 \
		 poses convicts at step {} (θ = {:.6} rad = {:.2}°), pair {:?} {:?}, exact crossing {}, sampled penetration {:.4} mm, \
		 surface distance {:.4} mm; that pair crosses at {} of the dense poses",
		endpoints.steps,
		endpoints.first_interference,
		endpoints.min_clearance_over_cycle,
		endpoints.pair_sweeps[0].pair,
		endpoints.pair_sweeps[0].crossings,
		hit.step,
		hit.driven_value,
		hit.driven_value.to_degrees(),
		hit.pair,
		hit.pair_names,
		hit.crossing,
		hit.penetration,
		hit.min_distance,
		dense.pair_sweeps[0].crossings
	);
}

/// **The mobility refusal.** Two structures that cannot move:
///
/// - a triangle (ground + 2 links, 3 revolutes): F = 3(3−1) − 2·3 = **0**, and
///   the Jacobian rank agrees — an honest, non-paradoxical structure;
/// - one link pinned to ground TWICE: F = 3(2−1) − 2·2 = **−1**, while the
///   Jacobian rank says 0 free DOF. The two counts disagree, which is exactly
///   the redundant-constraint (Grübler paradox) situation, and the refusal
///   reports both numbers instead of pretending the formula settled it.
#[test]
fn locked_mechanisms_refuse_with_their_kutzbach_count() {
	let mut tri = Mechanism::new("triangle");
	tri.add_link(Link::new("ground", Pose2::new(0.0, 0.0, 0.0)));
	tri.add_link(Link::new("strut_a", Pose2::new(0.0, 0.0, PI / 3.0)));
	tri.add_link(Link::new("strut_b", Pose2::new(30.0, 51.9615, -0.63788)));
	tri.add_joint(Joint::revolute("pin_o", 0, DVec2::ZERO, 1, DVec2::ZERO).driven());
	tri.add_joint(Joint::revolute("pin_a", 1, DVec2::new(60.0, 0.0), 2, DVec2::ZERO));
	tri.add_joint(Joint::revolute("pin_c", 2, DVec2::new(87.17798, 0.0), 0, DVec2::new(100.0, 0.0)));

	let mut twice = Mechanism::new("double_pinned");
	twice.add_link(Link::new("ground", Pose2::new(0.0, 0.0, 0.0)));
	twice.add_link(Link::new("plate", Pose2::new(0.0, 0.0, 0.0)));
	twice.add_joint(Joint::revolute("pin_0", 0, DVec2::ZERO, 1, DVec2::ZERO).driven());
	twice.add_joint(Joint::revolute("pin_1", 0, DVec2::new(50.0, 0.0), 1, DVec2::new(50.0, 0.0)));

	let tri_mob = tri.mobility();
	let twice_mob = twice.mobility();
	let tri_err = tri.sweep(8).expect_err("a triangle cannot move");
	let twice_err = twice.sweep(8).expect_err("a twice-pinned plate cannot move");

	assert!(
		matches!(tri_err, MechanismError::Locked { dof: 0, rank_dof: 0, .. })
			&& !tri_mob.paradox
			&& tri_mob.constraint_rows == 6
			&& tri_mob.coordinates == 6
			&& matches!(twice_err, MechanismError::Locked { dof: -1, rank_dof: 0, .. })
			&& twice_mob.paradox
			&& twice_mob.kutzbach_dof == -1
			&& twice_mob.rank_dof == 0
			&& twice_mob.jacobian_rank == 3
			&& twice_err.to_string().contains("paradox"),
		"locked refusals: triangle {} -> {tri_err}; double-pinned {} (jacobian rank {}, rank_dof {}, paradox {}) -> {twice_err}",
		tri_mob.formula,
		twice_mob.formula,
		twice_mob.jacobian_rank,
		twice_mob.rank_dof,
		twice_mob.paradox
	);
}

/// A command outside the linkage's reachable range is refused rather than
/// silently clamped or left half-solved.
///
/// Four-bar with ground 100, driven link 70, coupler 90, rocker 45: the driven
/// link can only ROCK, between the configurations where coupler and rocker are
/// collinear — |AC| ∈ [45, 135] with |AC|² = 70² + 100² − 2·70·100·cos θ, i.e.
/// θ ∈ [23.05°, 103.74°]. Declared at 60°, commanded to 150°, the loop cannot
/// close.
///
/// Which of the two loop-closure refusals fires (`Convergence`, or `Singular`
/// at the change point reached on the way) depends on where the continuation
/// lands relative to the limit; both are honest refusals and the test accepts
/// either, asserting the step and the commanded value are reported.
#[test]
fn a_command_outside_the_reachable_range_is_refused() {
	let mut m = Mechanism::new("non_rotating_input");
	m.add_link(Link::new("ground", Pose2::new(0.0, 0.0, 0.0)));
	m.add_link(Link::new("input", Pose2::new(0.0, 0.0, PI / 3.0)));
	m.add_link(Link::new("coupler", Pose2::new(35.0, 60.6218, -0.24216)));
	m.add_link(Link::new("rocker", Pose2::new(100.0, 0.0, 1.05105)));
	m.add_joint(Joint::revolute("input_pivot", 0, DVec2::ZERO, 1, DVec2::ZERO).driven());
	m.add_joint(Joint::revolute("input_pin", 1, DVec2::new(70.0, 0.0), 2, DVec2::ZERO));
	m.add_joint(Joint::revolute("coupler_pin", 2, DVec2::new(90.0, 0.0), 3, DVec2::new(45.0, 0.0)));
	m.add_joint(Joint::revolute("rocker_pivot", 3, DVec2::ZERO, 0, DVec2::new(100.0, 0.0)));

	let inside = m.pose_at(PI / 3.0).expect("the declared configuration is reachable");
	let err = m.pose_at(150.0f64.to_radians()).expect_err("150° is outside the input's rocking range");
	let ok_variant = matches!(err, MechanismError::Convergence { .. } | MechanismError::Singular { .. });
	assert!(
		m.mobility().kutzbach_dof == 1 && inside.len() == 4 && ok_variant && err.to_string().contains("mechanism"),
		"out-of-range command: mobility {}, the declared 60° pose solves ({} links), and 150° refuses with a loop-closure error \
		 (Convergence or Singular, both honest): {err:?} -> {err}",
		m.mobility().kutzbach_dof,
		inside.len()
	);
}

/// Structural refusals: no driven joint, a joint on a link that does not
/// exist, and a prismatic drive asked for a 2π cycle it does not have.
#[test]
fn malformed_mechanisms_are_refused_with_typed_errors() {
	let mut undriven = four_bar();
	undriven.joints[0].driven = false;
	let mut bad_link = Mechanism::new("bad_link");
	bad_link.add_link(Link::new("ground", Pose2::identity()));
	bad_link.add_link(Link::new("arm", Pose2::identity()));
	bad_link.add_joint(Joint::revolute("nowhere", 0, DVec2::ZERO, 7, DVec2::ZERO).driven());

	let mut prismatic_drive = slider_crank(None, None);
	prismatic_drive.joints[0].driven = false;
	prismatic_drive.joints[3].driven = true;

	// Same linkage declared at crank 90° so the slide drive is NOT starting at
	// the slider's dead centre (where commanding the slider genuinely cannot
	// decide which way the crank turns — `slider_crank()` is declared exactly
	// there, and driving its slide from that pose is refused as Singular).
	let mut slide_drive = Mechanism::new("slide_driven");
	slide_drive.add_link(Link::new("ground", Pose2::new(0.0, 0.0, 0.0)));
	slide_drive.add_link(Link::new("crank", Pose2::new(0.0, 0.0, FRAC_PI_2)));
	slide_drive.add_link(Link::new("coupler", Pose2::new(0.0, R2, -0.339837)));
	slide_drive.add_link(Link::new("slider", Pose2::new(84.8528137, 0.0, 0.0)));
	slide_drive.add_joint(Joint::revolute("crank_pivot", 0, DVec2::ZERO, 1, DVec2::ZERO));
	slide_drive.add_joint(Joint::revolute("crank_pin", 1, DVec2::new(R2, 0.0), 2, DVec2::ZERO));
	slide_drive.add_joint(Joint::revolute("wrist_pin", 2, DVec2::new(R3, 0.0), 3, DVec2::ZERO));
	slide_drive.add_joint(Joint::prismatic("slide", 0, DVec2::ZERO, DVec2::X, 3, DVec2::ZERO).driven());

	let e1 = undriven.sweep(8).expect_err("no driven joint");
	let e2 = bad_link.sweep(8).expect_err("bad link index");
	let e3 = prismatic_drive.sweep(8).expect_err("a slide has no cycle");
	let e4 = prismatic_drive.sweep_range(110.0, 100.0, 4).expect_err("declared at the slider's dead centre");
	let stroke = slide_drive.sweep_range(84.0, 80.0, 4).expect("off the dead centre, a stroke sweeps fine");

	assert!(
		matches!(e1, MechanismError::NoDrivenJoint { .. })
			&& matches!(e2, MechanismError::BadInput { .. })
			&& matches!(e3, MechanismError::NonCyclicDrive { .. })
			&& matches!(e4, MechanismError::Singular { step: 0, .. })
			&& stroke.steps == 5
			&& (stroke.driven_value[0] - 84.0).abs() < 1e-12
			&& (stroke.range_of_motion[3].origin_extents[0] - 80.0).abs() < 1e-9
			&& (stroke.range_of_motion[3].origin_extents[1] - 84.0).abs() < 1e-9
			&& stroke.max_newton_residual < 1e-10,
		"structural refusals: {e1} / {e2} / {e3}; driving the slide from the slider's dead centre also refuses: {e4}; off the \
		 dead centre the same drive over an explicit 84→80 mm stroke gives {} poses with slider x in [{:.9}, {:.9}] (want \
		 [80, 84]), max Newton residual {:.3e}",
		stroke.steps,
		stroke.range_of_motion[3].origin_extents[0],
		stroke.range_of_motion[3].origin_extents[1],
		stroke.max_newton_residual
	);
}
