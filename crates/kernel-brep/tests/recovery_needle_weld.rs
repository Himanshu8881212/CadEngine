//! Regression: a chained union+difference sequence (the drawer_system desk-dock
//! plate: plate + dovetail rails + fence, then countersunk screw holes) drove the
//! boolean face recovery to emit a zero-area needle face whose far corners differ
//! at ~1e-9. The B-rep stayed topologically valid, but the tessellation kept the
//! collapsed needle and the mesh went non-manifold (edge incidence 4) — caught by
//! `Mesh::is_watertight`. Fixed at the weld: collapsed triangles are dropped.
//!
//! Characterized (NOT yet fixed) alongside: booleans where a male dovetail
//! profile overlaps a notched plate as two thin parallel-flank sliver strips
//! mis-stitch the A/B fragments; the checked APIs detect the broken result and
//! REFUSE (the designed honest failure — no silent garbage). The same joint
//! geometry embedded in the real module shell (big profile, key piercing the
//! front face) resolves fine — the drawer_system example asserts those gates
//! numerically on every run. See campaign/friction/ENGINE.md #23.

use kernel_brep::math::{DVec2, DVec3};
use kernel_brep::{cone, cuboid, cylinder, difference, extrude, tessellate_default, try_intersection, union, validate};
use std::f64::consts::FRAC_PI_2;

fn v(x: f64, y: f64, z: f64) -> DVec3 {
	DVec3::new(x, y, z)
}

#[test]
fn chained_union_difference_plate_meshes_watertight() {
	// plate
	let mut s = cuboid(v(-50.0, 0.0, 0.0), v(50.0, 130.0, 4.0));
	// two dovetail rails (male profile, embedded 0.5)
	for rx in [-20.0f64, 20.0] {
		let prof: Vec<DVec2> = [(rx - 2.8, 3.5), (rx + 2.8, 3.5), (rx + 2.8, 4.0), (rx + 4.21, 6.35), (rx - 4.21, 6.35), (rx - 2.8, 4.0)]
			.iter()
			.map(|&(x, z)| DVec2::new(x, z))
			.collect();
		let rail = extrude(&prof, 118.0).transformed(
			kernel_brep::math::DAffine3::from_translation(v(0.0, 122.0, 0.0)) * kernel_brep::math::DAffine3::from_rotation_x(FRAC_PI_2),
		);
		s = union(&s, &rail);
	}
	// fence
	s = union(&s, &cuboid(v(-42.0, 122.5, 3.5), v(42.0, 129.4, 12.0)));
	// two countersunk holes IN A LINE with a rail between them — the layout that
	// left the needle face behind
	for (hx, hy) in [(-38.0, 20.0), (-38.0, 110.0)] {
		s = difference(&s, &cylinder(v(hx, hy, -0.5), DVec3::Z, 2.2, 5.0, 32));
		s = difference(&s, &cone(v(hx, hy, 5.0), -DVec3::Z, 5.69, 4.78, 32));
	}
	let val = validate(&s);
	let wt = tessellate_default(&s).is_watertight();
	assert!(
		val.is_valid() && wt,
		"rails+fence+2csk plate must stay valid AND mesh watertight: valid={} wt={wt} \
		 (a recovery needle face used to double-count its long edge after welding)",
		val.is_valid(),
	);
}

#[test]
fn notch_sliver_overlap_refuses_honestly() {
	// Socket-notched plate (opening 6, root 9, depth 2.5) lifted 1.0 over a
	// centred bowtie key: the true overlap is two disjoint 0.4-wide parallelogram
	// strips, 25 long — volume 2 × (0.4 × 1.35) × 25 = 27.
	let plate_prof: Vec<DVec2> = [(-20.0, 1.0), (-3.0, 1.0), (-4.5, 3.5), (4.5, 3.5), (3.0, 1.0), (20.0, 1.0), (20.0, 7.0), (-20.0, 7.0)]
		.iter()
		.map(|&(x, y)| DVec2::new(x, y))
		.collect();
	let plate = extrude(&plate_prof, 25.0);
	let bowtie: Vec<DVec2> = [(-2.8, 0.0), (-4.21, 2.35), (4.21, 2.35), (2.8, 0.0), (4.21, -2.35), (-4.21, -2.35)]
		.iter()
		.map(|&(x, y)| DVec2::new(x, y))
		.collect();
	// The key overshoots both plate caps by 1 (the project-wide pierce idiom —
	// EXACTLY coincident end caps are the known coincident-face degeneracy and
	// break the difference route too).
	let key = extrude(&bowtie, 27.0).transformed(kernel_brep::math::DAffine3::from_translation(v(0.0, 0.0, -1.0)));

	// On this ISOLATED plate the arrangement mis-stitches the two parallel-flank
	// sliver strips in every op — and the honest contract is that the kernel
	// KNOWS: plain results validate as broken, and every checked op refuses
	// rather than hand back garbage. When the arrangement one day resolves this,
	// these flip — then tighten this test to assert the exact 27 mm³ overlap
	// (2 × 0.4 × 1.35 × 25) and close FRICTION #23.
	let d_invalid = !validate(&difference(&key, &plate)).is_valid();
	let refused_i = try_intersection(&plate, &key).is_err();
	let refused_d = kernel_brep::try_difference(&key, &plate).is_err();
	assert!(
		d_invalid && refused_i && refused_d,
		"notch-sliver overlap must fail HONESTLY while unfixed: plain difference invalid={d_invalid}, \
		 try_intersection refused={refused_i}, try_difference refused={refused_d} \
		 (all must be true — if an op now succeeds, verify overlap == 27 mm³ and close FRICTION #23)",
	);
}
