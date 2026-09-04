//! `validate.geometric_ok` changed SOURCE: it is now derived from
//! `Mesh::self_intersection_witness`, so the flag and the witness it reports can
//! never contradict each other. That is only safe if the new predicate answers
//! exactly what the historic `kernel_brep::self_intersects` answered.
//!
//! This file pins that equivalence over a battery of solids covering the shapes
//! the flag is asked about in practice — the five primitives, boolean pockets
//! and bores, disjoint and fused unions, a revolved frustum, a mirrored copy,
//! and the off-axis-tube polar pattern that made the flag famous. A single
//! disagreement fails the suite, so the two oracles cannot drift apart unnoticed.

use kernel_brep::math::DVec3;
use kernel_brep::Solid;

/// The battery: `(name, solid)`.
fn battery() -> Vec<(&'static str, Solid)> {
	let mut v: Vec<(&'static str, Solid)> = vec![
		("box", kernel_brep::cuboid(DVec3::ZERO, DVec3::new(10.0, 10.0, 10.0))),
		("cylinder", kernel_brep::cylinder(DVec3::ZERO, DVec3::Z, 5.0, 10.0, 32)),
		("sphere", kernel_brep::sphere(DVec3::ZERO, 5.0, 24, 16)),
		("cone", kernel_brep::cone(DVec3::ZERO, DVec3::Z, 5.0, 12.0, 32)),
		("torus", kernel_brep::torus(DVec3::ZERO, DVec3::Z, 10.0, 3.0, 32, 16)),
	];

	// A drilled plate: a boolean with a curved seam.
	let plate = kernel_brep::cuboid(DVec3::ZERO, DVec3::new(40.0, 20.0, 5.0));
	let drill = kernel_brep::cylinder(DVec3::new(10.0, 10.0, -1.0), DVec3::Z, 3.0, 7.0, 32);
	v.push(("drilled_plate", kernel_brep::difference(&plate, &drill)));

	// A blind pocket, and a through tube (annular end caps).
	let blank = kernel_brep::cuboid(DVec3::ZERO, DVec3::new(20.0, 20.0, 20.0));
	let pocket = kernel_brep::cuboid(DVec3::new(5.0, 5.0, 10.0), DVec3::new(15.0, 15.0, 25.0));
	v.push(("pocket", kernel_brep::difference(&blank, &pocket)));
	let outer = kernel_brep::cylinder(DVec3::ZERO, DVec3::Z, 10.0, 20.0, 48);
	let bore = kernel_brep::cylinder(DVec3::new(0.0, 0.0, -1.0), DVec3::Z, 4.0, 22.0, 48);
	let tube = kernel_brep::difference(&outer, &bore);
	v.push(("tube", tube.clone()));

	// Disjoint union (two shells in one solid) and an overlapping fuse.
	let far = kernel_brep::cuboid(DVec3::new(60.0, 0.0, 0.0), DVec3::new(70.0, 10.0, 10.0));
	v.push(("disjoint_union", kernel_brep::union(&plate, &far)));
	let overlap = kernel_brep::cuboid(DVec3::new(5.0, 5.0, 0.0), DVec3::new(45.0, 25.0, 5.0));
	v.push(("fused_union", kernel_brep::union(&plate, &overlap)));

	// The turgo repro: an off-axis tube patterned about Z — the case three
	// campaigns filed as an unexplained `geometric_ok:false`.
	let o = kernel_brep::cylinder(DVec3::new(24.0, 0.0, 7.0), DVec3::X, 14.2, 24.0, 64);
	let i = kernel_brep::cylinder(DVec3::new(23.0, 0.0, 7.0), DVec3::X, 12.0, 26.0, 64);
	let d = kernel_brep::difference(&o, &i);
	v.push(("offaxis_tube", d.clone()));
	let mut acc = d.clone();
	for k in 1..3 {
		let m = kernel_brep::math::DAffine3::from_axis_angle(DVec3::Z, (120.0 * k as f64).to_radians());
		acc = kernel_brep::union(&acc, &d.transformed(m));
	}
	v.push(("offaxis_tube_polar_x3", acc));

	// A revolved frustum (the new `cone` top_radius path) and a mirrored copy.
	let profile = [
		kernel_brep::math::DVec2::new(0.0, 0.0),
		kernel_brep::math::DVec2::new(10.0, 0.0),
		kernel_brep::math::DVec2::new(4.0, 20.0),
		kernel_brep::math::DVec2::new(0.0, 20.0),
	];
	let frustum = kernel_brep::revolve(&profile, 64);
	v.push(("frustum", frustum.clone()));
	v.push(("frustum_mirrored", frustum.mirrored(DVec3::ZERO, DVec3::X)));

	v
}

/// The flag's new source must agree with its historic one on every solid.
#[test]
fn the_witness_agrees_with_the_historic_flag() {
	let mut disagreements: Vec<String> = Vec::new();
	for (name, s) in battery() {
		let historic = kernel_brep::self_intersects(&s);
		let witness = kernel_brep::tessellate_default(&s).self_intersection_witness();
		if historic != witness.is_some() {
			disagreements.push(format!("{name}: self_intersects={historic}, witness={witness:?}"));
		}
	}
	assert!(disagreements.is_empty(), "the two self-intersection oracles disagree:\n{}", disagreements.join("\n"));
}

/// The witness is a WITNESS: when it fires, the two triangles it names really do
/// cross, and the point it reports is on both of their bounding boxes. A
/// diagnostic that points at the wrong place is worse than none.
#[test]
fn the_reported_witness_is_real() {
	let mut checked = 0;
	for (name, s) in battery() {
		let mesh = kernel_brep::tessellate_default(&s);
		let Some(w) = mesh.self_intersection_witness() else {
			continue;
		};
		checked += 1;
		let tris: Vec<[u32; 3]> = mesh.triangles().collect();
		assert!(w.triangles[0] < w.triangles[1], "{name}: witness pair must be ordered — {w:?}");
		assert!(w.triangles[1] < tris.len(), "{name}: witness index out of range — {w:?}");
		assert!(w.pairs >= 1, "{name}: a witness implies at least one pair — {w:?}");
		// The named triangles must not be adjacent (adjacent triangles are
		// EXPECTED to touch and are never a self-intersection).
		let (a, b) = (tris[w.triangles[0]], tris[w.triangles[1]]);
		assert!(!a.iter().any(|v| b.contains(v)), "{name}: witness names two ADJACENT triangles — {w:?}");
		// The point must lie inside both triangles' bounding boxes.
		for t in [a, b] {
			let pts = [mesh.positions[t[0] as usize], mesh.positions[t[1] as usize], mesh.positions[t[2] as usize]];
			let bb = kernel_core::Aabb::from_points(&pts).pad(1e-3);
			assert!(bb.contains(w.point), "{name}: witness point {:?} is outside triangle box {bb:?}", w.point);
		}
	}
	assert!(checked > 0, "the battery must contain at least one self-intersecting case, or this test proves nothing");
}

/// A clean solid must never grow a witness — the flag stays a signal, not noise.
#[test]
fn clean_solids_carry_no_witness() {
	for name in ["box", "cylinder", "sphere", "cone", "torus", "drilled_plate", "pocket", "tube", "frustum"] {
		let (_, s) = battery().into_iter().find(|(n, _)| *n == name).unwrap();
		assert!(kernel_brep::tessellate_default(&s).self_intersection_witness().is_none(), "{name} must be geometrically clean");
	}
}
