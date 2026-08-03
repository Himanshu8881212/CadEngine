// Copyright (c) LMCAD. Licensed under the MIT License.

//! Pinning tests for `kernel_model::loads` — static load paths through an
//! assembly. Every reaction is hand-computed in the comment above its
//! assertion; every case additionally asserts that global equilibrium closes,
//! and the two dangerous outcomes (indeterminate, unsupported) are proven to
//! REFUSE rather than return a plausible number.

use kernel_core::math::DVec3;
use kernel_model::loads::{AppliedLoad, Attach, Connection, JointKind, LoadCase, LoadError};

/// Vector closeness with the measured gap reported by the caller.
fn near(a: DVec3, b: DVec3, tol: f64) -> bool {
	(a - b).length() <= tol
}

/// A two-link cantilever: a root arm clamped to ground at the origin, a tip
/// arm rigidly joined to it at x = 100, and a 10 N downward load at x = 200.
fn two_link_cantilever() -> LoadCase {
	let mut c = LoadCase::new("two_link_cantilever");
	let root = c.add_part("arm_root");
	let tip = c.add_part("arm_tip");
	c.add_connection(Connection::support("base_clamp", root, DVec3::ZERO, JointKind::Rigid));
	c.add_connection(Connection::joint("elbow", root, tip, DVec3::new(100.0, 0.0, 0.0), JointKind::Rigid));
	c.add_load(AppliedLoad::force("tip_load", tip, DVec3::new(200.0, 0.0, 0.0), DVec3::new(0.0, 0.0, -10.0)));
	c
}

/// Hand statics, two rigid links, to machine precision.
///
/// Sign convention: a connection's wrench is what end `b` applies to end `a`.
///
/// TIP ARM (`b` of `elbow`) carries only the 10 N tip load, so the elbow must
/// hand it +10 N in z and a 1000 N·mm couple:
///   ΣF: −F_elbow + (0,0,−10) = 0 ⇒ F_elbow = (0, 0, −10) N
///   ΣM₀: (100,0,0)×(0,0,10) − M_elbow + (200,0,0)×(0,0,−10) = 0
///        (0,−1000,0) − M_elbow + (0, 2000, 0) = 0 ⇒ M_elbow = (0, 1000, 0) N·mm
/// ROOT ARM then closes on the clamp:
///   ΣF: F_clamp + (0,0,−10) = 0 ⇒ F_clamp = (0, 0, 10) N
///   ΣM₀: M_clamp + (100,0,0)×(0,0,−10) + (0,1000,0) = 0
///        M_clamp + (0,1000,0) + (0,1000,0) = 0 ⇒ M_clamp = (0, −2000, 0) N·mm
/// which is the textbook 10 N × 200 mm = 2000 N·mm base moment.
#[test]
fn two_link_chain_reactions_match_hand_statics() {
	let case = two_link_cantilever();
	let path = case.solve().expect("a clamped two-link chain is determinate");
	let clamp = path.connection("base_clamp").expect("clamp reaction");
	let elbow = path.connection("elbow").expect("elbow reaction");
	let tip = path.part("arm_tip").expect("tip part");
	assert!(
		near(clamp.force, DVec3::new(0.0, 0.0, 10.0), 1e-12)
			&& near(clamp.moment, DVec3::new(0.0, -2000.0, 0.0), 1e-9)
			&& near(elbow.force, DVec3::new(0.0, 0.0, -10.0), 1e-12)
			&& near(elbow.moment, DVec3::new(0.0, 1000.0, 0.0), 1e-9)
			&& path.unknowns == 12
			&& path.independent_equations == 12
			&& path.max_residual_force < 1e-12
			&& path.max_residual_moment < 1e-9
			&& path.global_residual_force < 1e-12
			&& path.global_residual_moment < 1e-9
			&& tip.via_mates.len() == 1
			&& near(tip.via_mates[0].force, DVec3::new(0.0, 0.0, 10.0), 1e-12)
			&& near(tip.via_mates[0].moment, DVec3::new(0.0, -1000.0, 0.0), 1e-9),
		"two-link cantilever: clamp F {:?} (want [0,0,10] N) M {:?} (want [0,-2000,0] N·mm); elbow F {:?} (want [0,0,-10]) M \
		 {:?} (want [0,1000,0]); unknowns {} rank {} (want 12/12, determinate); residuals part F {:.3e} M {:.3e}, global F \
		 {:.3e} M {:.3e}; the tip arm sees exactly one mate delivering F {:?} M {:?} (want [0,0,10] / [0,-1000,0])",
		clamp.force,
		clamp.moment,
		elbow.force,
		elbow.moment,
		path.unknowns,
		path.independent_equations,
		path.max_residual_force,
		path.max_residual_moment,
		path.global_residual_force,
		path.global_residual_moment,
		tip.via_mates[0].force,
		tip.via_mates[0].moment
	);
}

/// A lever: hinge (revolute about +Y) at the origin, roller at x = 100, and a
/// 20 N downward load at x = 50 — the classic simply-supported beam with a
/// mid-span load.
///
/// ΣM_y about the origin: −100·N + 1000 = 0 ⇒ **N = 10 N** at the roller, and
/// ΣF_z ⇒ the hinge takes the other **10 N**. The hinge's moment about its own
/// axis does not exist by construction (a revolute is free there), and both of
/// its other moment components come out **zero** because every force is
/// parallel to z on the x-axis.
#[test]
fn lever_moment_balance_splits_a_mid_span_load_evenly() {
	let mut case = LoadCase::new("lever");
	let beam = case.add_part("beam");
	case.add_connection(Connection::support("hinge", beam, DVec3::ZERO, JointKind::Revolute { axis: DVec3::Y }));
	case.add_connection(Connection::support(
		"roller",
		beam,
		DVec3::new(100.0, 0.0, 0.0),
		JointKind::Contact { normal: DVec3::Z },
	));
	case.add_load(AppliedLoad::force("mid_load", beam, DVec3::new(50.0, 0.0, 0.0), DVec3::new(0.0, 0.0, -20.0)));
	let path = case.solve().expect("hinge + roller is determinate in 3D");
	let hinge = path.connection("hinge").expect("hinge");
	let roller = path.connection("roller").expect("roller");
	assert!(
		near(hinge.force, DVec3::new(0.0, 0.0, 10.0), 1e-12)
			&& near(hinge.moment, DVec3::ZERO, 1e-9)
			&& near(roller.force, DVec3::new(0.0, 0.0, 10.0), 1e-12)
			&& !roller.tension_on_unilateral
			&& path.gate_unilateral().is_ok()
			&& path.unknowns == 6
			&& path.independent_equations == 6
			&& path.max_residual_force < 1e-12
			&& path.max_residual_moment < 1e-9
			&& path.global_residual_force < 1e-12
			&& path.global_residual_moment < 1e-9,
		"lever: hinge F {:?} (want [0,0,10] N) M {:?} (want zero — free about its own axis, and the other two components vanish); \
		 roller F {:?} (want [0,0,10] N) in compression {}; unknowns {} rank {} (want 6/6); residuals part F {:.3e} M {:.3e}, \
		 global F {:.3e} M {:.3e}",
		hinge.force,
		hinge.moment,
		roller.force,
		!roller.tension_on_unilateral,
		path.unknowns,
		path.independent_equations,
		path.max_residual_force,
		path.max_residual_moment,
		path.global_residual_force,
		path.global_residual_moment
	);
}

/// A pure couple propagates with **zero force**: a 500 N·mm torque about +Z on
/// an arm rigidly stacked on a clamped post travels down the joint as a pure
/// moment and lands on ground as −500 N·mm. Nothing in the chain carries any
/// force at all, which is the check that the moment path is real and not an
/// artefact of a force couple.
#[test]
fn a_pure_couple_travels_the_chain_as_a_pure_moment() {
	let mut case = LoadCase::new("torque_post");
	let post = case.add_part("post");
	let arm = case.add_part("arm");
	case.add_connection(Connection::support("base", post, DVec3::ZERO, JointKind::Rigid));
	case.add_connection(Connection::joint("collar", post, arm, DVec3::new(0.0, 0.0, 50.0), JointKind::Rigid));
	case.add_load(AppliedLoad::couple("drive_torque", arm, DVec3::new(0.0, 0.0, 500.0)));
	let path = case.solve().expect("determinate");
	let base = path.connection("base").expect("base");
	let collar = path.connection("collar").expect("collar");
	assert!(
		near(base.force, DVec3::ZERO, 1e-12)
			&& near(base.moment, DVec3::new(0.0, 0.0, -500.0), 1e-9)
			&& near(collar.force, DVec3::ZERO, 1e-12)
			&& near(collar.moment, DVec3::new(0.0, 0.0, 500.0), 1e-9)
			&& path.max_residual_force < 1e-12
			&& path.max_residual_moment < 1e-9
			&& path.global_residual_force < 1e-12
			&& path.global_residual_moment < 1e-9,
		"pure couple: base F {:?} (want zero) M {:?} (want [0,0,-500] N·mm); collar F {:?} (want zero) M {:?} (want [0,0,500]); \
		 residuals part F {:.3e} M {:.3e}, global F {:.3e} M {:.3e}",
		base.force,
		base.moment,
		collar.force,
		collar.moment,
		path.max_residual_force,
		path.max_residual_moment,
		path.global_residual_force,
		path.global_residual_moment
	);
}

/// **The indeterminacy refusal.** A propped cantilever — clamped at x = 0
/// (6 unknowns) and propped on a roller at x = 100 (1 unknown) — has 7
/// reaction unknowns against 6 equilibrium equations. The redundancy is
/// exactly **1**, the textbook first-degree-indeterminate case, and a
/// rigid-body model must refuse it rather than pick one of the infinitely many
/// splits between clamp and prop.
#[test]
fn propped_cantilever_refuses_with_redundancy_one() {
	let mut case = LoadCase::new("propped_cantilever");
	let beam = case.add_part("beam");
	case.add_connection(Connection::support("clamp", beam, DVec3::ZERO, JointKind::Rigid));
	case.add_connection(Connection::support("prop", beam, DVec3::new(100.0, 0.0, 0.0), JointKind::Contact { normal: DVec3::Z }));
	case.add_load(AppliedLoad::force("mid", beam, DVec3::new(50.0, 0.0, 0.0), DVec3::new(0.0, 0.0, -10.0)));
	let err = case.solve().expect_err("an indeterminate structure must refuse");
	let msg = err.to_string();
	assert!(
		matches!(
			err,
			LoadError::Indeterminate { redundancy: 1, unknowns: 7, independent_equations: 6, .. }
		) && msg.contains("STATICALLY INDETERMINATE")
			&& msg.contains("stiffness"),
		"propped cantilever must refuse with redundancy 1 (7 unknowns, 6 independent equations) and say why; got: {err:?} -> {msg}"
	);
}

/// **The negative control.** An unsupported (floating) two-link assembly under
/// a real load has no equilibrium solution at all. Returning zeros here would
/// be the most dangerous possible answer — a part sized against no load —
/// so the solve refuses and reports the unbalanced resultant that proves it.
#[test]
fn a_floating_assembly_refuses_instead_of_returning_zeros() {
	let mut case = LoadCase::new("floating_chain");
	let a = case.add_part("link_a");
	let b = case.add_part("link_b");
	case.add_connection(Connection::joint("pin", a, b, DVec3::new(100.0, 0.0, 0.0), JointKind::Spherical));
	case.add_load(AppliedLoad::force("tip", b, DVec3::new(200.0, 0.0, 0.0), DVec3::new(0.0, 0.0, -10.0)));
	let err = case.solve().expect_err("a floating assembly must refuse");
	let msg = err.to_string();
	let (nf, sup) = match &err {
		LoadError::NoEquilibrium { net_force, supports, .. } => (*net_force, *supports),
		other => panic!("wrong refusal for a floating assembly: {other:?}"),
	};
	assert!(
		case.support_count() == 0
			&& sup == 0
			&& (nf[2] + 10.0).abs() < 1e-12
			&& nf[0] == 0.0
			&& nf[1] == 0.0
			&& msg.contains("FLOATING")
			&& msg.contains("CANNOT BE EQUILIBRATED"),
		"floating control: declared supports {}, reported supports {sup} (want 0), net external force {nf:?} (want [0,0,-10] N — \
		 nothing can react it), refusal text: {msg}",
		case.support_count()
	);
}

/// A unilateral contact solved in TENSION is reported, not hidden. Move the
/// roller to the far side of the hinge (x = −100) and the same 20 N mid-span
/// load requires it to PULL 10 N: ΣM_y ⇒ +100·N + 1000 = 0 ⇒ N = −10 N.
/// The arithmetic is right; the contact model is wrong, and
/// `gate_unilateral` says so.
#[test]
fn a_contact_solved_in_tension_is_flagged_not_hidden() {
	let mut case = LoadCase::new("lever_wrong_roller");
	let beam = case.add_part("beam");
	case.add_connection(Connection::support("hinge", beam, DVec3::ZERO, JointKind::Revolute { axis: DVec3::Y }));
	case.add_connection(Connection::support(
		"roller",
		beam,
		DVec3::new(-100.0, 0.0, 0.0),
		JointKind::Contact { normal: DVec3::Z },
	));
	case.add_load(AppliedLoad::force("mid_load", beam, DVec3::new(50.0, 0.0, 0.0), DVec3::new(0.0, 0.0, -20.0)));
	let path = case.solve().expect("still determinate — only the contact SENSE is wrong");
	let roller = path.connection("roller").expect("roller");
	let flagged = path.gate_unilateral();
	assert!(
		near(roller.force, DVec3::new(0.0, 0.0, -10.0), 1e-12)
			&& roller.tension_on_unilateral
			&& flagged.as_ref().is_err_and(|v| v.len() == 1 && v[0].starts_with("roller"))
			&& path.max_residual_force < 1e-12
			&& path.max_residual_moment < 1e-9,
		"unilateral flag: roller F {:?} (want [0,0,-10] N — it would have to PULL), flagged {}, gate {:?}; residuals F {:.3e} \
		 M {:.3e}",
		roller.force,
		roller.tension_on_unilateral,
		flagged,
		path.max_residual_force,
		path.max_residual_moment
	);
}

/// The FEA hand-off: each part is clamped at its ground-ward mate, loaded at
/// the others, and **no reaction moment is dropped**.
///
/// For the two-link cantilever the root arm is clamped at `base_clamp`
/// (ground, depth 0) and loaded at `elbow` with the 10 N the tip arm hands
/// back plus the 1000 N·mm couple, which a point-load FEA cannot express and
/// which therefore appears in `unrepresented_moments` with a note. The tip arm
/// is clamped at `elbow` and loaded by its own 10 N tip force.
#[test]
fn fea_manifest_clamps_the_ground_ward_mate_and_never_drops_a_moment() {
	let case = two_link_cantilever();
	let path = case.solve().expect("determinate");
	let jobs = path.fea_manifest(&case, 1.5);
	let root = jobs.iter().find(|j| j.part == "arm_root").expect("root job");
	let tip = jobs.iter().find(|j| j.part == "arm_tip").expect("tip job");
	let json = path.fea_manifest_json(&case, 1.5);
	let root_moment = root.unrepresented_moments.first().map(|m| DVec3::from_array(m.moment_n_mm).length()).unwrap_or(0.0);
	assert!(
		jobs.len() == 2
			&& root.fixtures.len() == 1
			&& root.fixtures[0].source == "base_clamp"
			&& root.fixtures[0].kind == "clamped"
			&& root.loads.len() == 1
			&& root.loads[0].source == "elbow"
			&& (root.loads[0].magnitude_n - 10.0).abs() < 1e-12
			&& (root.loads[0].direction[2] + 1.0).abs() < 1e-12
			&& root.unrepresented_moments.len() == 1
			&& (root_moment - 1000.0).abs() < 1e-9
			&& root.notes.iter().any(|n| n.contains("unrepresented_moments"))
			&& tip.fixtures.len() == 1
			&& tip.fixtures[0].source == "elbow"
			&& tip.loads.len() == 1
			&& tip.loads[0].source == "tip_load"
			&& json.contains("lmcad.load_path.fea.v1")
			&& json.contains("\"kind\": \"clamped\""),
		"FEA manifest: {} job(s) (want 2); root fixture '{}' kind '{}' (want base_clamp/clamped), root load '{}' {:.4} N dir \
		 {:?} (want elbow 10 N along -z), root unrepresented moment |M| {:.4} N·mm (want 1000, carried NOT dropped), notes {:?}; \
		 tip fixture '{}' (want elbow), tip load '{}' (want tip_load); JSON schema tag present: {}",
		jobs.len(),
		root.fixtures[0].source,
		root.fixtures[0].kind,
		root.loads[0].source,
		root.loads[0].magnitude_n,
		root.loads[0].direction,
		root_moment,
		root.notes,
		tip.fixtures[0].source,
		tip.loads[0].source,
		json.contains("lmcad.load_path.fea.v1")
	);
}

/// Structural refusals: an unknown part index, a self-connection, and a
/// degenerate joint axis are all rejected before any arithmetic runs.
#[test]
fn malformed_load_cases_are_refused_with_typed_errors() {
	let mut bad_index = LoadCase::new("bad_index");
	let p = bad_index.add_part("only");
	bad_index.add_load(AppliedLoad::force("ghost", 7, DVec3::ZERO, DVec3::Z));
	let mut self_conn = LoadCase::new("self");
	let q = self_conn.add_part("only");
	self_conn.add_connection(Connection::joint("loop", q, q, DVec3::ZERO, JointKind::Rigid));
	let mut zero_axis = LoadCase::new("zero_axis");
	let r = zero_axis.add_part("only");
	zero_axis.add_connection(Connection::support("hinge", r, DVec3::ZERO, JointKind::Revolute { axis: DVec3::ZERO }));

	let e1 = bad_index.solve().expect_err("unknown part");
	let e2 = self_conn.solve().expect_err("self connection");
	let e3 = zero_axis.solve().expect_err("degenerate axis");
	assert!(
		p == 0
			&& matches!(e1, LoadError::UnknownPart { index: 7, parts: 1, .. })
			&& matches!(e2, LoadError::SelfConnection { part: Some(0), .. })
			&& matches!(e3, LoadError::DegenerateAxis { kind: "revolute", .. })
			&& q == 0
			&& r == 0,
		"structural refusals: {e1} / {e2} / {e3}"
	);
}

/// The joint vocabulary's unknown counts are the contract the determinacy
/// arithmetic rests on, so they are pinned directly: rigid 6, revolute 5,
/// prismatic 5, spherical 3, contact 1 — and each `basis()` really produces
/// that many independent directions.
#[test]
fn joint_unknown_counts_are_pinned() {
	let kinds = [
		(JointKind::Rigid, 6usize),
		(JointKind::Revolute { axis: DVec3::Z }, 5),
		(JointKind::Prismatic { axis: DVec3::X }, 5),
		(JointKind::Spherical, 3),
		(JointKind::Contact { normal: DVec3::Z }, 1),
	];
	let all_match = kinds.iter().all(|(k, want)| {
		let (f, m) = k.basis().expect("basis");
		k.unknowns() == *want && f.len() + m.len() == *want
	});
	// A revolute is free about its own axis: neither moment direction may have
	// a component along it.
	let (_, rev_moments) = JointKind::Revolute { axis: DVec3::Z }.basis().expect("revolute basis");
	let axial_leak = rev_moments.iter().fold(0.0f64, |a, m| a.max(m.dot(DVec3::Z).abs()));
	// A prismatic is free ALONG its axis: neither force direction may have one.
	let (pri_forces, _) = JointKind::Prismatic { axis: DVec3::X }.basis().expect("prismatic basis");
	let slide_leak = pri_forces.iter().fold(0.0f64, |a, f| a.max(f.dot(DVec3::X).abs()));
	assert!(
		all_match && axial_leak < 1e-15 && slide_leak < 1e-15 && Attach::Part(3).part() == Some(3) && Attach::Ground.part().is_none(),
		"joint contract: counts match {all_match} (rigid 6 / revolute 5 / prismatic 5 / spherical 3 / contact 1); revolute \
		 moment leak along its own axis {axial_leak:.3e} (want 0); prismatic force leak along its slide {slide_leak:.3e} (want 0)"
	);
}
