// Copyright (c) LMCAD. Licensed under the MIT License.

//! Regression tests for `kernel_model::kinematics` — the PLAN-26 / HARM-26 /
//! CYCLO-26 numbers pinned exactly, INCLUDING the two bugs that shipped when
//! this math was hand-rolled in the drive simulators (the wrong textbook
//! stepped-planet install phase, and the strain-wave tangential sign).

use kernel_model::kinematics::{CycloidTrain, EpicyclicTrain, StrainWaveTrain};
use std::f64::consts::{PI, TAU};

/// The PLAN-26 Wolfrom train (12/36 stage 1, 12→11 stepped planets, 39T
/// output ring, 3 planets).
fn plan26() -> EpicyclicTrain {
	EpicyclicTrain { sun_teeth: 12, ring1_teeth: 36, planet_a_teeth: 12, planet_b_teeth: 11, ring2_teeth: 39, n_planets: 3 }
}

#[test]
fn plan26_validates_and_ratio_is_exactly_26() {
	let t = plan26();
	let val = t.validate_assembly();
	let ratio = t.ratio();
	let poses = t.poses(1.0);
	assert!(
		val.is_ok() && (ratio - 26.0).abs() < 1e-12 && (poses.carrier - 0.25).abs() < 1e-15 && (poses.ring2 - 1.0 / 26.0).abs() < 1e-15,
		"PLAN-26 snapshot: validate {val:?}, ratio {ratio} (want 26 to 1e-12), carrier at θ=1 {} (want 0.25), ring2 at θ=1 {} (want 1/26 = {})",
		poses.carrier,
		poses.ring2,
		1.0 / 26.0
	);
}

#[test]
fn epicyclic_validate_rejects_each_bad_tooth_count() {
	let odd = EpicyclicTrain { ring1_teeth: 37, ..plan26() }.validate_assembly();
	let spacing1 = EpicyclicTrain { sun_teeth: 14, ring1_teeth: 36, ..plan26() }.validate_assembly();
	let spacing2 = EpicyclicTrain { ring2_teeth: 40, ..plan26() }.validate_assembly();
	let zero = EpicyclicTrain { n_planets: 0, ..plan26() }.validate_assembly();
	assert!(
		odd.as_ref().is_err_and(|e| e.contains("odd"))
			&& spacing1.as_ref().is_err_and(|e| e.contains("not divisible by n_planets"))
			&& spacing2.as_ref().is_err_and(|e| e.contains("R2"))
			&& zero.is_err(),
		"each violated assembly condition must refuse with its own message: odd-sum {odd:?}, (S+R1)%n {spacing1:?}, R2%n {spacing2:?}, n=0 {zero:?}"
	);
}

#[test]
fn plan26_install_phase_is_beta_and_the_textbook_formula_is_the_shipped_bug() {
	// At θ = 0 every planet must sit at exactly its install phase ψ0 = βj —
	// the rigid-rotation convention. The textbook stepped-planet formula
	// β·(1 + S/Pa) shipped as a bug: for planet 1 it differs from the correct
	// phase by 2π/3, which modulo the 11T B-band pitch (2π/11) is EXACTLY ⅔
	// of a tooth — the mis-clock that jammed stage 2.
	let t = plan26();
	let p = t.poses(0.0);
	let exact_at_zero = (0..3).all(|j| {
		let beta = TAU * j as f64 / 3.0;
		p.planets[j].spin == beta && p.planets[j].install_phase == beta && p.planets[j].azimuth == beta
	});
	let beta1 = TAU / 3.0;
	let wrong = beta1 * (1.0 + 12.0 / 12.0); // textbook β(1+S/Pa)
	let bug_offset = wrong - p.planets[1].spin;
	let tooth_pitch_b = TAU / 11.0;
	let bug_in_teeth = (bug_offset % tooth_pitch_b) / tooth_pitch_b;
	assert!(
		exact_at_zero
			&& (bug_offset - TAU / 3.0).abs() < 1e-12
			&& (bug_in_teeth - 2.0 / 3.0).abs() < 1e-12
			&& (p.sun_install_phase - PI / 12.0).abs() < 1e-15
			&& p.ring2_install_phase == 0.0,
		"PLAN-26 install-phase snapshot: ψj==βj at θ=0 {exact_at_zero}; textbook β(1+S/Pa) offset {bug_offset} (want 2π/3 = {}), = {bug_in_teeth:.6} of an 11T pitch (want 2/3 — the shipped ⅔-tooth mis-clock); sun phase {} (want π/12), ring2 phase {}",
		TAU / 3.0,
		p.sun_install_phase,
		p.ring2_install_phase
	);
}

#[test]
fn plan26_spin_formula_and_simple_ratio() {
	// spin(θ) = φc − (S/Pa)(θ − φc): for PLAN-26 φc = θ/4 and S/Pa = 1, so
	// spin(θ) = θ/4 − 3θ/4 = −θ/2 — the planets counter-rotate at half the
	// input rate. And the plain single-stage helper: 1 + R/S.
	let t = plan26();
	let theta = 1.7;
	let spin = t.poses(theta).planets[0].spin; // β0 = 0
	let simple = EpicyclicTrain::simple_ratio(12, 36);
	assert!(
		(spin - (-theta / 2.0)).abs() < 1e-15 && simple == 4.0,
		"spin(1.7) = {spin} (want −θ/2 = {}), simple_ratio(12,36) = {simple} (want 4)",
		-theta / 2.0
	);
}

#[test]
fn harm26_strain_wave_ratio_creep_and_deformation_signs() {
	// HARM-26: 52T flexspline, 54T circular spline, ratio F/2 = 26, creep
	// ψ = −2θ/F (NEGATIVE — the output walks backwards), and the
	// inextensible deformation field w = w0·cos2(φ−θ), v = −(w0/2)·sin2(φ−θ).
	// The tangential sign is the second shipped bug: at φ−θ = 45° the
	// tangential displacement must be NEGATIVE (−w0/2), not +w0/2.
	let t = StrainWaveTrain { flex_teeth: 52 };
	let w0 = 0.6;
	let creep = t.flex_creep(1.0);
	let (r_major, v_major) = t.deformation(w0, 0.8, 0.8); // φ = θ: major axis
	let (r_45, v_45) = t.deformation(w0, PI / 4.0, 0.0); // φ − θ = 45°
	assert!(
		t.ratio() == 26.0
			&& t.circ_teeth() == 54
			&& creep == -2.0 / 52.0
			&& creep < 0.0
			&& r_major == w0
			&& v_major == 0.0
			&& v_45 < 0.0
			&& (v_45 - (-w0 / 2.0)).abs() < 1e-15
			&& r_45.abs() < 1e-15,
		"HARM-26 snapshot: ratio {} (want 26), circ {} (want 54), creep(1) {creep} (want −2/52, NEGATIVE), deformation at φ=θ ({r_major}, {v_major}) (want ({w0}, 0)), at φ−θ=45° ({r_45}, {v_45}) (want (0, −{}) — the tangential SIGN is the pinned bug)",
		t.ratio(),
		t.circ_teeth(),
		w0 / 2.0
	);
}

#[test]
fn cyclo26_ratio_creep_and_second_disc_phase() {
	// CYCLO-26: 26 lobes, ratio 26:1, disc creep −θ/26, and the balanced
	// second disc (cam phase π) carries the spin phase −π/26.
	let t = CycloidTrain { lobes: 26 };
	let creep_rev = t.disc_creep(TAU);
	assert!(
		t.ratio() == 26.0 && creep_rev == -TAU / 26.0 && creep_rev < 0.0 && t.second_disc_phase() == -PI / 26.0,
		"CYCLO-26 snapshot: ratio {} (want 26), creep per cam rev {creep_rev} (want −2π/26 = {}), second-disc phase {} (want −π/26 = {})",
		t.ratio(),
		-TAU / 26.0,
		t.second_disc_phase(),
		-PI / 26.0
	);
}
