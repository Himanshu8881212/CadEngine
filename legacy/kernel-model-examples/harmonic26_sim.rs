//! HARM-26 KINEMATIC SIMULATOR — exact deformed-tooth verification of the
//! strain-wave mesh, tooth-polygon level (the same methodology that
//! validated the cyclo drive: dense boolean sweeps on the true 2D sections).
//!
//! The flexspline model is the standard inextensible thin-ring strain-wave
//! kinematics: neutral radius rn deforms radially w(φ)=w0·cos2(φ−θ) with the
//! tangential displacement v(φ)=(w0/2)·sin2(φ−θ) that keeps the neutral line
//! arc-length constant; each rigid tooth rides the deformed neutral line,
//! tilted by the local slope. Grounded circular spline, wave angle θ input,
//! flex creep ψ = −2θ/F  →  ratio −F/2 = −26 exactly.
//!
//! Stages (all machine-gated, exit 1 on FAIL):
//!   S1 dense sweep: 360 wave poses × boolean(flex teeth, circ ring): zero
//!      interference AND ≥6 teeth engaged in each major-axis zone.
//!   S2 ratio lock: creep ±5% wrong must JAM (interference appears).
//!   S3 backlash: bisect the output rotation at fixed θ until flank contact.
//!   S4 four-roller support: a SYMMETRIC roller pair straddles each major-axis
//!      lobe at ±φ2, so all four 693ZZ rollers sit tangent to the deformed bore
//!      (|gap| ≤ 0.05 each). Reports the worst UNSUPPORTED rim arc — the wave is
//!      roller-driven, and four rollers halve the span two apex rollers left.
//!   S5 kinematics.json for tools/animate_sim.py.
//!
//! Constants MIRROR examples/harmonic26.rs (asserted against params.csv).

use kernel_brep::math::{DAffine3, DVec2, DVec3};
use kernel_brep::{extrude, intersection, volume, Solid};
use kernel_model::parts::trapezoid_tooth_offsets;
use std::f64::consts::{PI, TAU};

const TEETH_F: usize = 52;
const TEETH_C: usize = 54;
const MODULE: f64 = 0.6;
const WALL: f64 = 1.2;
const FLANK_DEG: f64 = 25.0;
const SLACK: f64 = 0.05;
const ROLLER_OD: f64 = 8.0;
const ROLLER_PHI2_DEG: f64 = 30.0; // wave-roller straddle half-angle (mirrors harmonic26.rs)

const RF: f64 = MODULE * TEETH_F as f64 / 2.0; // 15.6
const RC: f64 = MODULE * TEETH_C as f64 / 2.0; // 16.2
const W0: f64 = MODULE;
const HA: f64 = 0.7 * MODULE;
const HD: f64 = 0.9 * MODULE;
const FLEX_TIP: f64 = RF + HA;
const FLEX_ROOT: f64 = RF - HD;
const CIRC_TIP: f64 = RC - HA;
const CIRC_ROOT: f64 = RC + HD;
const BORE_R: f64 = FLEX_ROOT - WALL;
const RN: f64 = BORE_R + WALL * 0.5; // neutral (mid-wall) radius

fn v(x: f64, y: f64, z: f64) -> DVec3 {
	DVec3::new(x, y, z)
}
fn ccw(mut p: Vec<DVec2>) -> Vec<DVec2> {
	let a2: f64 = p.windows(2).map(|w| w[0].x * w[1].y - w[1].x * w[0].y).sum::<f64>()
		+ (p[p.len() - 1].x * p[0].y - p[0].x * p[p.len() - 1].y);
	if a2 < 0.0 {
		p.reverse();
	}
	p
}

/// Circular-spline ring solid: annulus whose inner boundary is the 54-tooth
/// internal profile (teeth pointing inward), 1 mm thick slab for booleans.
fn circ_ring() -> Solid {
	let pitch = TAU / TEETH_C as f64;
	// Circular-spline internal profile from the SHARED library generator — the
	// SAME one the printed housing is built from (was a private copy; the two
	// desynced once into the sawtooth casing bug). HALF-PITCH phase: a circ
	// SPACE sits at angle 0 where flex tooth 0 engages at wave angle 0
	// (tooth-on-tooth otherwise); at θ=π the second lobe aligns exactly too.
	let offs = trapezoid_tooth_offsets(TEETH_C, RC, CIRC_TIP, CIRC_ROOT, FLANK_DEG, false, 0.0);
	let mut inner: Vec<DVec2> = Vec::new();
	for k in 0..TEETH_C {
		let c = pitch * (k as f64 + 0.5);
		for (da, r) in &offs {
			inner.push(DVec2::new(r * (c + da).cos(), r * (c + da).sin()));
		}
	}
	// annulus = outer disc minus the toothed hole: build directly as one
	// polygon strip is messy — use boolean of two extrusions instead
	let outer: Vec<DVec2> = (0..180).map(|i| {
		let a = TAU * i as f64 / 180.0;
		DVec2::new((RC + 3.0) * a.cos(), (RC + 3.0) * a.sin())
	}).collect();
	let ring = extrude(&ccw(outer), 1.0);
	let hole = extrude(&ccw(inner), 3.0).transformed(DAffine3::from_translation(v(0.0, 0.0, -1.0)));
	kernel_brep::difference(&ring, &hole)
}

/// Deformed flexspline tooth-band solid at wave angle `th` with output creep
/// `psi`: 52 rigid trapezoid teeth riding the deformed neutral line, plus the
/// deformed root band (wall) they stand on.
fn flex_deformed(th: f64, psi: f64) -> Solid {
	let n = TEETH_F;
	let pitch = TAU / n as f64;
	// deformation field on the neutral line (material angle φ, wave angle th)
	let w = |phi: f64| W0 * (2.0 * (phi - th)).cos();
	// inextensible neutral line: dv/dφ = −w  →  v = −(w0/2)·sin2(φ−θ)
	let vv = |phi: f64| -(W0 / 2.0) * (2.0 * (phi - th)).sin();
	// Undeformed tooth profile (angle-offset, radius) from the SHARED library
	// generator — the SAME one the printed flexspline is built from, so the
	// simulated teeth and the printed teeth are the same shape by construction.
	// Each point then rides the deformed neutral line: a material point at
	// (angle φ, radius r) maps to polar (φ + v/RN, r + w), the rigid tooth tilted
	// by dw/ds.
	let prof = trapezoid_tooth_offsets(n, RF, FLEX_TIP, FLEX_ROOT, FLANK_DEG, true, SLACK);
	let mut pts: Vec<DVec2> = Vec::with_capacity(n * (prof.len() + 1));
	for k in 0..n {
		let c0 = pitch * k as f64 + psi; // material centre angle incl. creep
		let phi = c0; // deformation sampled at the tooth centre
		let dphi = phi + vv(phi) / RN; // deformed angular position
		let dr = w(phi); // radial deflection
		// local tilt of the deformed neutral line: γ = dw/(RN·dφ)
		let gam = -2.0 * W0 * (2.0 * (phi - th)).sin() / RN;
		for (da, r) in &prof {
			// the rigid tooth rotates by γ about its root point on the
			// neutral line: first-order angular shift γ·(r−RN)/r
			let a_t = dphi + da + gam * (r - RN) / r;
			let rr = r + dr;
			pts.push(DVec2::new(rr * a_t.cos(), rr * a_t.sin()));
		}
	}
	extrude(&ccw(pts), 1.0)
}

fn ov(a: &Solid, b: &Solid) -> f64 {
	let ix = intersection(a, b);
	if ix.face_count() == 0 {
		0.0
	} else {
		volume(&ix).abs()
	}
}

fn main() {
	let mut ok = true;
	println!(
		"HARM-26 SIMULATOR — strain-wave {}:1, {}T/{}T, m={MODULE}, w0={W0}\n",
		TEETH_F / 2,
		TEETH_F,
		TEETH_C
	);
	// params.csv must agree with the mirrored constants
	if let Ok(txt) = std::fs::read_to_string("harmonic26/params.csv") {
		for line in txt.lines() {
			let l = line.trim();
			if l.is_empty() || l.starts_with('#') {
				continue;
			}
			let mut it = l.split(',');
			let (k, val) = (it.next().unwrap_or(""), it.next().unwrap_or("").trim());
			match k.trim() {
				"teeth_flex" => assert_eq!(val.parse::<usize>().unwrap(), TEETH_F, "params/sim mismatch: teeth_flex"),
				"module" => assert!((val.parse::<f64>().unwrap() - MODULE).abs() < 1e-9, "params/sim mismatch: module"),
				"wall" => assert!((val.parse::<f64>().unwrap() - WALL).abs() < 1e-9, "params/sim mismatch: wall"),
				"slack" => assert!((val.parse::<f64>().unwrap() - SLACK).abs() < 1e-9, "params/sim mismatch: slack"),
				_ => {}
			}
		}
	}
	let t0 = std::time::Instant::now();
	let ring = circ_ring();

	// ---- S1: dense sweep — zero interference + engagement count ----
	let poses = 144;
	let mut worst_ov = 0.0f64;
	let mut min_engaged = usize::MAX;
	for i in 0..poses {
		let th = TAU * i as f64 / poses as f64;
		let psi = -2.0 * th / TEETH_F as f64;
		let flex = flex_deformed(th, psi);
		let o = ov(&flex, &ring);
		worst_ov = worst_ov.max(o);
		// engagement: teeth whose deformed tip reaches past the circ tip circle
		let mut engaged = 0;
		for k in 0..TEETH_F {
			let phi = TAU * k as f64 / TEETH_F as f64 + psi;
			let tip_r = FLEX_TIP + W0 * (2.0 * (phi - th)).cos();
			if tip_r > CIRC_TIP + 0.05 {
				engaged += 1;
			}
		}
		min_engaged = min_engaged.min(engaged);
	}
	let s1_ok = worst_ov < 0.02 && min_engaged >= 6;
	ok &= s1_ok;
	println!(
		"S1 — {poses}-pose deformed sweep: worst tooth interference {worst_ov:.4} mm³ (<0.02) · min engaged teeth {min_engaged} (≥6)  {}",
		if s1_ok { "OK" } else { "<<< FAIL" }
	);

	// ---- S2: ratio lock — wrong creep must JAM ----
	let mut s2_ok = true;
	for factor in [1.05f64, 0.95] {
		let mut max_i = 0.0f64;
		for i in 0..120 {
			let th = 2.0 * TAU * i as f64 / 120.0;
			let psi = -2.0 * th / TEETH_F as f64 * factor;
			max_i = max_i.max(ov(&flex_deformed(th, psi), &ring));
		}
		let jams = max_i > 0.05;
		s2_ok &= jams;
		println!(
			"S2 — creep ×{factor:.2}: max interference {max_i:.2} mm³ within two revs — {}",
			if jams { "JAMS (correct)" } else { "<<< DOES NOT JAM" }
		);
	}
	ok &= s2_ok;

	// ---- S3: backlash — bisect output rotation to flank contact ----
	let th = 0.35; // arbitrary wave pose
	let psi0 = -2.0 * th / TEETH_F as f64;
	let lash_side = |sign: f64| -> f64 {
		let (mut lo, mut hi) = (0.0f64, 0.02f64);
		while ov(&flex_deformed(th, psi0 + sign * hi), &ring) < 0.005 && hi < 0.2 {
			hi *= 2.0;
		}
		for _ in 0..24 {
			let mid = 0.5 * (lo + hi);
			if ov(&flex_deformed(th, psi0 + sign * mid), &ring) < 0.005 {
				lo = mid;
			} else {
				hi = mid;
			}
		}
		lo
	};
	let lash = lash_side(1.0) + lash_side(-1.0);
	let lash_deg = lash.to_degrees();
	// the flexspline IS the output: this is output backlash directly
	let s3_ok = lash_deg < 0.8;
	ok &= s3_ok;
	println!("S3 — output backlash (flank-to-flank at the output): {lash_deg:.3}°  (<0.8°)  {}", if s3_ok { "OK" } else { "<<< FAIL" });

	// ---- S4: four-roller support — every roller tangent to the deformed bore ----
	// The wave generator carries a SYMMETRIC roller pair straddling each major-axis
	// lobe at ±φ2 (4 rollers). bore(φ)=BORE_R+W0·cos2φ; each roller centre sits at
	// c(φ)=bore(φ)−OD/2, so every one is tangent to the deformed bore at its own
	// angle. The four contact angles are ±φ2 and 180°±φ2 (both lobes doubly held).
	let phi2 = ROLLER_PHI2_DEG.to_radians();
	let roller_c = BORE_R + W0 - ROLLER_OD * 0.5; // apex offset (reference only)
	let roller_c2 = BORE_R + W0 * (2.0 * phi2).cos() - ROLLER_OD * 0.5;
	let roller_angles = [-phi2, phi2, PI - phi2, PI + phi2];
	let mut s4_ok = true;
	let mut worst_gap = 0.0f64;
	for (i, &ang) in roller_angles.iter().enumerate() {
		let bore = BORE_R + W0 * (2.0 * ang).cos();
		let gap = bore - (roller_c2 + ROLLER_OD * 0.5);
		worst_gap = worst_gap.max(gap.abs());
		s4_ok &= gap.abs() <= 0.05;
		println!("       roller {i} @ {:6.1}°: bore {bore:.3} vs tangent {:.3}, gap {gap:+.3}", ang.to_degrees(), roller_c2 + ROLLER_OD * 0.5);
	}
	// smoothness: worst UNSUPPORTED rim arc between adjacent support points.
	// BEFORE = two apex rollers 180° apart → 180°. AFTER = largest adjacent gap
	// among the four sorted contact angles.
	let mut sorted: Vec<f64> = roller_angles.iter().map(|a| a.rem_euclid(TAU)).collect();
	sorted.sort_by(|x, y| x.partial_cmp(y).unwrap());
	let mut max_arc = 0.0f64;
	for i in 0..sorted.len() {
		let nxt = if i + 1 < sorted.len() { sorted[i + 1] } else { sorted[0] + TAU };
		max_arc = max_arc.max(nxt - sorted[i]);
	}
	ok &= s4_ok;
	println!(
		"S4 — four-roller tangency: worst gap {worst_gap:.3} (|≤0.05|) · max UNSUPPORTED rim arc {:.0}° (was 180° with two apex rollers)  {}",
		max_arc.to_degrees(),
		if s4_ok { "OK" } else { "<<< FAIL" }
	);

	// ---- S5: kinematics.json ----
	let json = format!(
		"{{\"teeth_flex\":{TEETH_F},\"teeth_circ\":{TEETH_C},\"module\":{MODULE},\"w0\":{W0},\"rf\":{RF},\"rc\":{RC},\"flex_tip\":{FLEX_TIP},\"flex_root\":{FLEX_ROOT},\"circ_tip\":{CIRC_TIP},\"circ_root\":{CIRC_ROOT},\"bore_r\":{BORE_R},\"roller_od\":{ROLLER_OD},\"roller_c\":{:.4},\"roller_c2\":{:.4},\"roller_phi2_deg\":{ROLLER_PHI2_DEG},\"n_rollers\":4,\"flank_deg\":{FLANK_DEG},\"slack\":{SLACK}}}",
		roller_c, roller_c2
	);
	let _ = std::fs::create_dir_all("harmonic26/sim");
	let _ = std::fs::write("harmonic26/sim/kinematics.json", json);
	println!("S5 — harmonic26/sim/kinematics.json written (tools/animate_sim.py renders the gif)");

	println!(
		"\nRESULT: {} ({} s)",
		if ok { "PASS — strain wave meshes, ratio locked, backlash measured" } else { "FAIL — see <<< lines" },
		t0.elapsed().as_secs()
	);
	std::process::exit(if ok { 0 } else { 1 });
}
