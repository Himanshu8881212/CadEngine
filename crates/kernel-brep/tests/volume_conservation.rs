// Copyright (c) LMCAD. Licensed under the MIT License.

//! Volume-conservation regression for the subtraction-based overlap metric.
//!
//! Production bug (BUG CLASS A): computing an interference volume as
//! `vol(A) − vol(A ∖ B)` FABRICATED phantom overlaps of 0.27–6.4 mm³ when `A`
//! is a complex solid already carrying many boolean cuts (an octagonal housing
//! with 27 small cylindrical pockets + a register recess), and in the reverse
//! direction UNDERREPORTED a real ~90 mm³ intersection. This test rebuilds that
//! shape class and asserts both directions.
//!
//! MEASURED on the current kernel (2026-07-04, this worktree): the leak does
//! NOT reproduce — `|vol(A) − vol(A ∖ B)|` for a disjoint tool is ~1e-6 mm³
//! (tessellated) and ~1e-10 mm³ (`exact_volume`), and subtraction agrees with
//! the direct intersection to ~3e-6 mm³ across floating / coplanar-floor /
//! off-center / near-wall / dipping tool placements (probe run over 8
//! variants). The production leak predates the 2026-06 boolean hardening
//! (R1–R5, Wave-1 fuzz seeds, W5 seam snapping in `booleans.rs`), so no
//! in-tree root cause remains to fix; this test pins the shape class at the
//! production tolerance (0.05 mm³) so any regression of that hardening is
//! caught. The canonical interference metric remains the direct intersection —
//! see [`kernel_brep::overlap_volume`] — because it is structurally immune to
//! `A`-side re-stitching residue, not merely currently clean.

use kernel_brep::math::{DVec2, DVec3};
use kernel_brep::{cylinder, extrude, overlap_volume, try_difference, try_intersection, volume, Solid};

/// The production failure-case shape: an octagonal housing (42 across flats,
/// 8 tall), 27 Ø2 through-pockets on a Ø33 circle, one Ø22 register recess
/// 3 deep in the top face. 28 chained differences — a solid whose faces carry
/// heavy boolean history.
fn octagonal_housing() -> Solid {
	let apothem = 21.0; // 42 across flats
	let circum = apothem / (std::f64::consts::PI / 8.0).cos();
	let octagon: Vec<DVec2> = (0..8)
		.map(|k| {
			let th = (std::f64::consts::PI / 8.0) * (2 * k + 1) as f64;
			DVec2::new(circum * th.cos(), circum * th.sin())
		})
		.collect();
	let mut a = extrude(&octagon, 8.0);
	// 27 Ø2 through-holes on a Ø33 bolt circle.
	for k in 0..27 {
		let th = (2.0 * std::f64::consts::PI / 27.0) * k as f64;
		let c = DVec3::new(16.5 * th.cos(), 16.5 * th.sin(), -1.0);
		let tool = cylinder(c, DVec3::Z, 1.0, 10.0, 16);
		a = try_difference(&a, &tool).expect("housing pocket difference must validate");
	}
	// Ø22 register recess, 3 deep from the top (z = 8).
	let recess = cylinder(DVec3::new(0.0, 0.0, 5.0), DVec3::Z, 11.0, 5.0, 48);
	try_difference(&a, &recess).expect("register recess difference must validate")
}

#[test]
fn difference_by_a_disjoint_tool_conserves_volume_on_a_complex_housing() {
	let a = octagonal_housing();
	// A Ø13 cylinder floating wholly inside the housing's register-recess AIR:
	// radial clearance 4.5 mm to the recess wall, 0.5 mm axial clearance to the
	// recess floor. It touches NO material of A, so the difference must conserve
	// A's volume and the direct intersection must be empty.
	let b = cylinder(DVec3::new(0.0, 0.0, 5.5), DVec3::Z, 6.5, 4.0, 32);
	let va = volume(&a);
	let vd = volume(&try_difference(&a, &b).expect("difference by a disjoint tool must validate"));
	let vi = volume(&try_intersection(&a, &b).expect("intersection with a disjoint tool must validate"));
	assert!(
		(va - vd).abs() < 0.05 && vi.abs() < 0.05,
		"volume not conserved (production bar 0.05 mm3, measured ~1e-6 when healthy): \
		 vol(A)={va:.6}, vol(A\\B)={vd:.6}, leak={:.6} mm3, vol(A∩B)={vi:.6} mm3",
		(va - vd).abs()
	);
}

#[test]
fn real_intersection_on_the_complex_housing_is_not_underreported() {
	// The reverse production failure: a real ~90 mm³ overlap read LOW through the
	// subtraction metric. Here the tool dips 1 mm into the recess floor, so the
	// true overlap is a Ø13 × 1 disc: π·6.5²·1 ≈ 132.73 mm³ analytic. Both the
	// canonical overlap_volume (analytic, bulge-corrected) and the subtraction
	// metric must report it in full — no fabrication, no underreport.
	let a = octagonal_housing();
	let b = cylinder(DVec3::new(0.0, 0.0, 4.0), DVec3::Z, 6.5, 6.0, 32);
	let want = std::f64::consts::PI * 6.5 * 6.5 * 1.0;
	let ov = overlap_volume(&a, &b).expect("housing∩tool is a plain valid boolean");
	let sub = volume(&a) - volume(&try_difference(&a, &b).expect("dipping difference must validate"));
	// The subtraction metric sees the tool's FACETED disc (32-gon: sin-deficit
	// ~0.64%); overlap_volume recovers the analytic value exactly.
	let want_faceted = 16.0 * 6.5 * 6.5 * (std::f64::consts::PI / 16.0).sin() * 1.0;
	assert!(
		(ov - want).abs() < 0.05 && (sub - want_faceted).abs() < 0.05,
		"real overlap misreported: overlap_volume={ov:.6} (want analytic {want:.6}), \
		 subtraction metric={sub:.6} (want faceted {want_faceted:.6}), both ±0.05 mm3"
	);
}
