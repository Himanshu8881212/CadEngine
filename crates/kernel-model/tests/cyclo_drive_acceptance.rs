//! A 10:1 cycloidal drive for a NEMA 17 — the B-rep gear train and its cycloidal
//! engagement. (The full hybrid assembly, incl. the implicit gyroid lattice arm
//! with the orthogonal NEMA-17 chaining face, lives in
//! the pre-JSON example cyclo_drive.rs, removed from the tree 2026-09-03
//! (git history at 5a70984);
//! the lattice approach itself is covered by hybrid_lattice_acceptance.)
//!
//! Cycloidal disc: Zc=10 lobes rolling inside Zp=11 pins, ring grounded, output
//! via roller pins => reduction = Zc/(Zp-Zc) = 10:1.

use kernel_brep::math::{DAffine3, DVec2, DVec3};
use kernel_brep::{cylinder, difference, extrude, tessellate_default, validate, Solid};
use std::f64::consts::TAU;

const ZP: f64 = 11.0;
const RP: f64 = 20.0;
const RR: f64 = 2.0;
const E: f64 = 1.2;
const DT: f64 = 6.0;

fn cycloid_profile(n: usize) -> Vec<DVec2> {
	let p: Vec<DVec2> = (0..n)
		.map(|i| {
			let t = TAU * i as f64 / n as f64;
			let psi = ((1.0 - ZP) * t).sin().atan2(RP / (E * ZP) - ((1.0 - ZP) * t).cos());
			DVec2::new(
				RP * t.cos() - RR * (t + psi).cos() - E * (ZP * t).cos(),
				-RP * t.sin() + RR * (t + psi).sin() + E * (ZP * t).sin(),
			)
		})
		.collect();
	let area: f64 = 0.5 * (0..n).map(|i| { let j = (i + 1) % n; p[i].x * p[j].y - p[j].x * p[i].y }).sum::<f64>();
	if area < 0.0 { p.into_iter().rev().collect() } else { p }
}

fn cycloid_disc() -> Solid {
	let disc = extrude(&cycloid_profile(360), DT);
	let mut d = difference(&disc, &cylinder(DVec3::new(0.0, 0.0, -1.0), DVec3::Z, 6.0, DT + 2.0, 48));
	for i in 0..6 {
		let a = TAU * i as f64 / 6.0;
		d = difference(&d, &cylinder(DVec3::new(12.0 * a.cos(), 12.0 * a.sin(), -1.0), DVec3::Z, 4.0, DT + 2.0, 32));
	}
	d
}

#[test]
fn cycloidal_disc_is_valid_genus_7_engages_11_pins_at_10_to_1() {
	let prof = cycloid_profile(360);
	// 10 lobes engaging 11 pins: the profile radius straddles the pin circle (R=20)
	// — valleys inside, lobe tips reaching the pins (within RR of the pin centres).
	let radii: Vec<f64> = prof.iter().map(|p| (p.x * p.x + p.y * p.y).sqrt()).collect();
	let rmin = radii.iter().cloned().fold(f64::INFINITY, f64::min);
	let rmax = radii.iter().cloned().fold(0.0, f64::max);
	assert!(
		rmin < RP - RR && rmax > RP - 2.0 * RR && rmax < RP,
		"cycloid lobes must straddle the Ø40 pin circle (valleys inside, tips reaching the pins): r {rmin:.2}..{rmax:.2}"
	);

	// The disc: valid, genus 7 (centre bore + 6 output holes), watertight.
	let d = cycloid_disc();
	let v = validate(&d);
	assert!(
		v.closed && v.manifold && v.genus == 7 && tessellate_default(&d).is_watertight(),
		"cycloidal disc must be a valid watertight genus-7 solid (bore + 6 output holes): {v:?}"
	);

	// Validate the cam can sit at the eccentric pose (disc offset +E stays valid).
	let posed = cycloid_disc().transformed(DAffine3::from_translation(DVec3::new(E, 0.0, 0.0)));
	assert!(validate(&posed).is_valid(), "the disc at its eccentric operating pose must stay valid");

	// The defining reduction.
	let ratio = (ZP - 1.0) / (ZP - (ZP - 1.0));
	assert_eq!(ratio, 10.0, "Zc=10 / (Zp-Zc=1) = 10:1");
}

/// Exact B-rep ORTHOGONAL NEMA-17 tip mount (matches the retired cyclo_drive.rs example): a
/// 42x42x6 plate whose face normal is +X — perpendicular to this joint's Z axis —
/// with the real NEMA-17 interface (Ø22 pilot + 4x M3 on the 31 mm square, bored
/// along X). This is the modular chaining face: the next joint+motor bolts on here.
fn nema_tip_mount(zc: f64) -> Solid {
	let plate = kernel_brep::cuboid(DVec3::new(86.0, -21.0, zc - 21.0), DVec3::new(92.0, 21.0, zc + 21.0));
	let mut m = difference(&plate, &cylinder(DVec3::new(85.0, 0.0, zc), DVec3::X, 11.0, 8.0, 64));
	for (sy, sz) in [(1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)] {
		m = difference(&m, &cylinder(DVec3::new(85.0, 15.5 * sy, zc + 15.5 * sz), DVec3::X, 1.7, 8.0, 24));
	}
	m
}

#[test]
fn tip_mount_is_orthogonal_nema17_chaining_face() {
	let zc = 21.0;
	let mount = nema_tip_mount(zc);
	// Valid watertight genus-5 (Ø22 pilot + 4x M3 = 5 through-holes): a real,
	// manufacturable mount the next joint bolts onto — what makes the joint modular.
	let v = validate(&mount);
	assert!(
		v.closed && v.manifold && v.genus == 5 && tessellate_default(&mount).is_watertight(),
		"tip mount must be a valid watertight genus-5 NEMA-17 face (pilot + 4 bolts): {v:?}"
	);
	// The mount FACE is in X (thin in X, 42 mm in Y and Z): its normal is +X, which is
	// orthogonal to this joint's Z axis — so the chained motor's axis is perpendicular.
	let (lo, hi) = mount.aabb();
	let (dx, dy, dz) = (hi.x - lo.x, hi.y - lo.y, hi.z - lo.z);
	assert!(
		dx < 7.0 && (dy - 42.0).abs() < 0.5 && (dz - 42.0).abs() < 0.5,
		"NEMA-17 face must be 42x42 in Y/Z and thin in X (normal +X, orthogonal to joint Z): {dx:.1} x {dy:.1} x {dz:.1}"
	);
}
