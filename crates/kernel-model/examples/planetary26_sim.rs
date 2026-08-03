//! PLAN-26 KINEMATIC SIMULATOR — exact tooth-level verification of the
//! two-stage simple involute planetary, BOTH stages, using the SAME involute
//! outlines the printed parts are built from (kernel_model::parts, single
//! source of truth) and the SAME pose math the kernel exposes
//! (kernel_model::kinematics::EpicyclicTrain — no mirrored constants to drift).
//!
//! Each stage is a simple planetary (sun in, carrier out, ring FIXED); its
//! ratio is `EpicyclicTrain::simple_ratio` (1 + R/S) and its carrier/planet/sun
//! poses are exactly the Wolfrom `poses` of that train (a Wolfrom's first stage
//! IS a simple planetary — identical carrier/planet/sun formulas; the ring2/
//! planet_b fields are unused because the OUTPUT is the carrier). The compound
//! coupling is `stage-2 sun = stage-1 carrier`, so stage 2 is driven at the
//! stage-1 carrier rate:
//!   input θ → carrier1 = θ·S1/(S1+R1) = θ/5.2 → carrier2 = θ/26 EXACTLY.
//!
//! Stages (machine-gated, exit 1 on FAIL):
//!   S1 dense sweep, BOTH stages at their exact epicyclic poses: sun↔planet and
//!      planet↔ring interference is zero over two carrier revolutions.
//!   S2 ratio lock: an OUTPUT (carrier) driven ±5 % off its exact rate JAMS —
//!      the planets can satisfy the fixed ring OR the sun, not both, off-ratio.
//!      Exact 26 is asserted through the kinematics module + the integer identity.
//!   S3 backlash: rock the OUTPUT carrier at fixed input to flank contact — the
//!      output free-play directly (stage-2 dominant + stage-1 referred).
//!
//! Backdrivability is a friction/efficiency property; this sim measures
//! KINEMATICS only — it CANNOT and does NOT certify backdrive torque.

use kernel_brep::math::{DAffine3, DVec2, DVec3};
use kernel_brep::{extrude, intersection, volume, Solid};
use kernel_model::kinematics::EpicyclicTrain;
use kernel_model::parts::involute_ring_outline_shifted;
use std::f64::consts::TAU;

const S1_T: usize = 15;
const P1_T: usize = 24;
const R1_T: usize = 63;
const S2_T: usize = 12;
const P2_T: usize = 18;
const R2_T: usize = 48;
const N_PL: usize = 3;
const M1: f64 = 0.6;
const M2: f64 = 0.79;
const PA: f64 = 25.0;
const LASH: f64 = 0.05;
// v2: ISO 53 profile shift on the OUTPUT stage — MUST match planetary26.rs
// (sun2 +X2, planet2 −X2, ring2 −X2: both meshes stay at standard CD; the S1
// sweep below is the ground-truth proof the shifted teeth still run clean).
const X2: f64 = 0.14;
const CD1: f64 = M1 * ((S1_T + P1_T) as f64) / 2.0;
const CD2: f64 = M2 * ((S2_T + P2_T) as f64) / 2.0;

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
fn tr(x: f64, y: f64) -> DAffine3 {
	DAffine3::from_translation(v(x, y, 0.0))
}
fn rotz(a: f64) -> DAffine3 {
	DAffine3::from_rotation_z(a)
}
fn gear_slab(outline: Vec<DVec2>) -> Solid {
	extrude(&ccw(outline), 1.0)
}
/// internal ring slab: outer disc minus the toothed cavity (profile shift x)
fn ring_slab(module: f64, teeth: usize, outer_r: f64, x: f64) -> Solid {
	let hole = ccw(involute_ring_outline_shifted(module, teeth, PA, false, false, LASH, x).expect("ring outline"));
	let outer: Vec<DVec2> = (0..180)
		.map(|i| {
			let a = TAU * i as f64 / 180.0;
			DVec2::new(outer_r * a.cos(), outer_r * a.sin())
		})
		.collect();
	let blank = extrude(&ccw(outer), 1.0);
	let cutter = extrude(&hole, 3.0).transformed(DAffine3::from_translation(v(0.0, 0.0, -1.0)));
	kernel_brep::difference(&blank, &cutter)
}
fn ov(a: &Solid, b: &Solid) -> f64 {
	let ix = intersection(a, b);
	if ix.face_count() == 0 {
		0.0
	} else {
		volume(&ix).abs()
	}
}

/// One simple planetary stage (ring fixed). Slabs built from the library
/// outlines the printed parts use.
struct Stage {
	sun: Solid,
	planet: Solid,
	ring: Solid,
	p: usize,
	r: usize,
	cd: f64,
}

impl Stage {
	/// `x = (x_sun, x_planet, x_ring)`: ISO 53 profile-shift coefficients
	/// (zeros reproduce the unshifted outlines byte-for-byte).
	fn new(module: f64, s: usize, p: usize, r: usize, cd: f64, x: (f64, f64, f64)) -> Self {
		let (xs, xp, xr) = x;
		let spur = |z, x| involute_ring_outline_shifted(module, z, PA, true, false, LASH, x).expect("spur");
		Stage {
			sun: gear_slab(spur(s, xs)),
			planet: gear_slab(spur(p, xp)),
			ring: ring_slab(module, r, module * r as f64 / 2.0 + 3.0, xr),
			p,
			r,
			cd,
		}
	}
	/// planet absolute spin (past install β) when rolling on the FIXED ring at
	/// carrier angle φ: ψ = φ·(P−R)/P (equivalently the module's poses formula).
	fn roll(&self, phi: f64) -> f64 {
		phi * (self.p as f64 - self.r as f64) / self.p as f64
	}
	/// interference (sun↔planet + planet↔ring) with the sun at absolute angle
	/// `sigma`, the ring fixed at 0, and the carrier at `phi` with the planets
	/// ROLLING on the ring. Zero ⇔ the whole stage meshes at (σ, φ).
	fn iface_rolling(&self, sigma: f64, phi: f64) -> f64 {
		let suns = self.sun.transformed(rotz(sigma));
		let mut total = 0.0;
		for j in 0..N_PL {
			let beta = TAU * j as f64 / N_PL as f64;
			let az = beta + phi;
			let psi = beta + self.roll(phi);
			let pl = self.planet.transformed(tr(self.cd * az.cos(), self.cd * az.sin()) * rotz(psi));
			total += ov(&pl, &suns) + ov(&pl, &self.ring);
		}
		total
	}
	/// interference with the planets RIGIDLY clocked to the carrier, rocked
	/// `delta` from the nominal pose at carrier `phi0` (sun fixed at `sigma`,
	/// ring fixed) — the OUTPUT free-play model for backlash.
	fn iface_rigid(&self, sigma: f64, phi0: f64, delta: f64) -> f64 {
		let suns = self.sun.transformed(rotz(sigma));
		let mut total = 0.0;
		for j in 0..N_PL {
			let beta = TAU * j as f64 / N_PL as f64;
			let az = beta + phi0 + delta;
			let psi = beta + self.roll(phi0) + delta;
			let pl = self.planet.transformed(tr(self.cd * az.cos(), self.cd * az.sin()) * rotz(psi));
			total += ov(&pl, &suns) + ov(&pl, &self.ring);
		}
		total
	}
	/// output backlash (deg) at fixed input: bisect the rigid rock both ways.
	fn backlash_deg(&self, sigma: f64, phi0: f64) -> f64 {
		let side = |sign: f64| -> f64 {
			let (mut lo, mut hi) = (0.0f64, 0.005f64);
			while self.iface_rigid(sigma, phi0, sign * hi) < 0.005 && hi < 0.3 {
				hi *= 2.0;
			}
			for _ in 0..22 {
				let mid = 0.5 * (lo + hi);
				if self.iface_rigid(sigma, phi0, sign * mid) < 0.005 {
					lo = mid;
				} else {
					hi = mid;
				}
			}
			lo
		};
		(side(1.0) + side(-1.0)).to_degrees()
	}
}

fn main() {
	let mut ok = true;
	let e1 = EpicyclicTrain { sun_teeth: S1_T, ring1_teeth: R1_T, planet_a_teeth: P1_T, planet_b_teeth: P1_T, ring2_teeth: R1_T, n_planets: N_PL };
	let e2 = EpicyclicTrain { sun_teeth: S2_T, ring1_teeth: R2_T, planet_a_teeth: P2_T, planet_b_teeth: P2_T, ring2_teeth: R2_T, n_planets: N_PL };
	let r1 = EpicyclicTrain::simple_ratio(S1_T, R1_T);
	let r2 = EpicyclicTrain::simple_ratio(S2_T, R2_T);
	let ratio = r1 * r2;
	// carrier rate coefficients (input θ → carrier)
	let wc1 = S1_T as f64 / (S1_T + R1_T) as f64; // 1/5.2
	let wc2 = S2_T as f64 / (S2_T + R2_T) as f64; // 1/5.0
	let inst1 = e1.poses(0.0).sun_install_phase; // π/S1
	let inst2 = e2.poses(0.0).sun_install_phase; // π/S2
	println!(
		"PLAN-26 SIMULATOR — 2-stage involute planetary {ratio:.4}:1  (stage1 {r1:.2} · stage2 {r2:.2}), \
		 sun {S1_T}/planet {P1_T}/ring {R1_T} m{M1} + sun {S2_T}/planet {P2_T}/ring {R2_T} m{M2}\n"
	);
	let t0 = std::time::Instant::now();
	let s1 = Stage::new(M1, S1_T, P1_T, R1_T, CD1, (0.0, 0.0, 0.0));
	let s2 = Stage::new(M2, S2_T, P2_T, R2_T, CD2, (X2, -X2, -X2));

	// ---- S1: dense sweep, both stages at their exact epicyclic poses ----
	let poses = 72;
	let mut worst = 0.0f64;
	for i in 0..poses {
		// two full carrier revolutions of the INPUT shaft
		let th = 2.0 * TAU * (S1_T + R1_T) as f64 / S1_T as f64 * i as f64 / poses as f64;
		let phic1 = th * wc1;
		let sigma1 = inst1 + th;
		let i1 = s1.iface_rolling(sigma1, phic1);
		// stage 2 is driven by the stage-1 carrier: its sun turns φc1 past install
		let sigma2 = inst2 + phic1;
		let phic2 = phic1 * wc2;
		let i2 = s2.iface_rolling(sigma2, phic2);
		worst = worst.max(i1 + i2);
	}
	let s1_ok = worst < 0.02;
	ok &= s1_ok;
	println!(
		"S1 — {poses}-pose sweep over 2 carrier revs, BOTH stages: worst interference {worst:.4} mm³ (<0.02)  {}",
		if s1_ok { "OK" } else { "<<< FAIL" }
	);

	// ---- S2: ratio lock (both stages) + exact 26 ----
	let ratio_exact = (ratio - 26.0).abs() < 1e-12 && (S1_T + R1_T) * (S2_T + R2_T) == 26 * S1_T * S2_T;
	ok &= ratio_exact && e1.validate_assembly().is_ok() && e2.validate_assembly().is_ok();
	println!(
		"S2 — exact ratio: (S1+R1)(S2+R2) = {}·{} = {} = 26·S1·S2 → {ratio:.6}  {}",
		S1_T + R1_T,
		S2_T + R2_T,
		(S1_T + R1_T) * (S2_T + R2_T),
		if ratio_exact { "EXACT 26 OK" } else { "<<< FAIL" }
	);
	let mut s2_ok = true;
	for f in [1.05f64, 0.95] {
		let (mut m1, mut m2) = (0.0f64, 0.0f64);
		for i in 0..40 {
			let th = TAU * (S1_T + R1_T) as f64 / S1_T as f64 * i as f64 / 40.0;
			let phic1 = th * wc1;
			// stage 1 output (carrier1) forced f× off its exact rate → sun1 jams
			m1 = m1.max(s1.iface_rolling(inst1 + th, phic1 * f));
			// stage 2 output (carrier2 = OUTPUT) forced f× off → sun2 jams
			let phic2 = phic1 * wc2;
			m2 = m2.max(s2.iface_rolling(inst2 + phic1, phic2 * f));
		}
		let jams = m1 > 0.05 && m2 > 0.05;
		s2_ok &= jams;
		println!(
			"     carriers at {f:.2}×exact: stage1 max {m1:.2} / stage2 max {m2:.2} mm³ — {}",
			if jams { "JAMS (correct)" } else { "<<< DOES NOT JAM" }
		);
	}
	ok &= s2_ok;

	// ---- S3: output backlash (rock the carriers at fixed input) ----
	let th = 0.7;
	let phic1 = th * wc1;
	let lash2 = s2.backlash_deg(inst2 + phic1, phic1 * wc2);
	let lash1 = s1.backlash_deg(inst1 + th, phic1);
	// referred to the output: stage-1 lash divides by the stage-2 ratio
	let out_lash = lash2 + lash1 / r2;
	let s3_ok = out_lash < 1.5;
	ok &= s3_ok;
	println!(
		"S3 — output backlash: stage2 {lash2:.3}° + stage1 {lash1:.3}°/{r2:.0} = {out_lash:.3}° at the output (<1.5°)  {}",
		if s3_ok { "OK" } else { "<<< FAIL" }
	);

	println!(
		"\nRESULT: {} ({} s)",
		if ok { "PASS — both stages mesh, 26:1 exact, ratio locked, output backlash measured" } else { "FAIL — see <<< lines" },
		t0.elapsed().as_secs()
	);
	std::process::exit(if ok { 0 } else { 1 });
}
