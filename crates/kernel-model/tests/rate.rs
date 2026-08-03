// Copyright (c) LMCAD. Licensed under the MIT License.

//! Regression tests for `kernel_model::rate` — the PLAN-26 / HARM-26 hand
//! ratings pinned as numbers, plus exact stackup arithmetic.

use kernel_model::rate::{cantilever_bending_stress, lewis_form_factor, lewis_tooth_load, thin_ring_bending_strain, Stackup};

#[test]
fn plan26_lewis_rating_reproduces_the_hand_calc() {
	// PLAN-26 stage-2 rating: σ_allow 50 MPa (printed PETG class), face width
	// b = 3.6 mm, module m = 0.754286 (the M_B of the 11T/39T stage), Y = 0.30
	// (11T conservative floor). Three planets share the load at the ring2
	// pitch radius 14.71 mm → torque ≈ 1.8 N·m. Pin within 5%.
	let f_per_tooth = lewis_tooth_load(50.0, 3.6, 0.754286, 0.30);
	let torque_nm = f_per_tooth * 3.0 * 14.71 / 1000.0;
	assert!(
		(torque_nm - 1.8).abs() / 1.8 < 0.05,
		"PLAN-26 Lewis rating: per-tooth load {f_per_tooth:.2} N, 3 planets at r=14.71 mm → {torque_nm:.4} N·m (want 1.8 ± 5%)"
	);
}

#[test]
fn lewis_form_factor_table_endpoints_and_interpolation() {
	// Barth-table rows exact, clamped ends, linear between rows (13T is the
	// midpoint of the 12→0.31 / 14→0.33 span → 0.32).
	let snapshot = [
		(10, 0.30), // below-table clamp (conservative floor)
		(11, 0.30),
		(12, 0.31),
		(13, 0.32), // interpolated
		(14, 0.33),
		(17, 0.35),
		(25, 0.40),
		(40, 0.45),
		(120, 0.45), // above-table clamp
	];
	let got: Vec<(usize, f64)> = snapshot.iter().map(|&(t, _)| (t, lewis_form_factor(t))).collect();
	assert!(
		snapshot.iter().zip(&got).all(|(&(_, want), &(_, y))| (y - want).abs() < 1e-12),
		"Lewis form-factor table snapshot mismatch: want {snapshot:?}, got {got:?}"
	);
}

#[test]
fn harm26_flexspline_wall_strain_matches_the_hand_calc() {
	// HARM-26 flexspline: wall t = 1.2 mm, wave amplitude w0 = 0.6 mm,
	// neutral (mid-wall) radius rn = 14.46 mm → ε = (t/2)·3·w0/rn² ≈ 0.0052
	// (0.52%, inside printed-nylon flexural fatigue). Pin within 5%.
	let eps = thin_ring_bending_strain(1.2, 0.6, 14.46);
	assert!(
		(eps - 0.0052).abs() / 0.0052 < 0.05,
		"HARM-26 flexspline strain: ε = {eps:.6} (want ≈ 0.0052 ± 5%)"
	);
}

#[test]
fn cantilever_bending_stress_both_support_cases() {
	// F = 100 N on a Ø4 pin over L = 10 mm: free-tip cantilever M = FL =
	// 1000 N·mm → σ = 32·1000/(π·64); both-ends-guided M = FL/8 → exactly
	// one eighth of that.
	let free = cantilever_bending_stress(100.0, 10.0, 4.0, false);
	let guided = cantilever_bending_stress(100.0, 10.0, 4.0, true);
	let want_free = 32.0 * 1000.0 / (std::f64::consts::PI * 64.0);
	assert!(
		(free - want_free).abs() < 1e-9 && (guided - want_free / 8.0).abs() < 1e-9,
		"pin bending: free-tip {free:.3} MPa (want {want_free:.3}), both-ends-guided {guided:.3} MPa (want {:.3} = free/8)",
		want_free / 8.0
	);
}

#[test]
fn stackup_worst_case_and_rss_exact_on_a_bearing_stack() {
	// Bearing stack: 7.0±0.10 bearing + 4.5±0.05 spacer + 1.0±0.02 shim.
	// Worst case and RSS asserted EXACTLY (identical arithmetic, same fold
	// order as the implementation).
	let mut s = Stackup::new();
	s.push(7.0, 0.10);
	s.push(4.5, 0.05);
	s.push(1.0, 0.02);
	let (wc_nom, wc_tol) = s.worst_case();
	let (rss_nom, rss_tol) = s.rss();
	let want_nom = 7.0 + 4.5 + 1.0;
	let want_wc = 0.10 + 0.05 + 0.02;
	let want_rss = (0.10 * 0.10 + 0.05 * 0.05 + 0.02 * 0.02_f64).sqrt();
	assert!(
		wc_nom == want_nom && wc_tol == want_wc && rss_nom == want_nom && rss_tol == want_rss && rss_tol < wc_tol && s.len() == 3 && !s.is_empty(),
		"stackup snapshot: worst ({wc_nom}, ±{wc_tol}) want ({want_nom}, ±{want_wc}); rss ({rss_nom}, ±{rss_tol}) want ({want_nom}, ±{want_rss}); RSS must be tighter than worst case"
	);
}
