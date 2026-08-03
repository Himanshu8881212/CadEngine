//! Exact SDF field-value identities for the CSG algebra that thick-walling and
//! lattices (the implicit half's bread and butter) are built on. The property
//! tests cover mesh closure/volume and redistance; this pins the raw field math:
//! a primitive is a true signed distance, CSG is min/max/max(A,-B), offset shifts
//! the level set, and shell makes a symmetric two-sided wall.

use kernel_implicit::{Node, Sdf, Sphere, Vec3};

fn at(n: &Node, r: f32) -> f32 {
	n.distance(Vec3::new(r, 0.0, 0.0))
}

#[test]
fn sdf_csg_field_identities_are_exact() {
	let s5 = || Node::primitive(Sphere::new(Vec3::ZERO, 5.0));
	let eps = 1e-5;

	// Primitive = true signed distance (negative inside, positive outside).
	assert!((at(&s5(), 3.0) + 2.0).abs() < eps && (at(&s5(), 8.0) - 3.0).abs() < eps, "sphere field must be the signed distance");

	// offset(+2) moves the zero level set outward to r=7; field(r) = (r-5)-2.
	let off = s5().offset(2.0);
	assert!((at(&off, 7.0)).abs() < eps && (at(&off, 10.0) - 3.0).abs() < eps, "offset must shift the level set by exactly the offset");

	// shell(1) = |dist| - 1: a symmetric wall with zeros at r=4 and r=6, mid-wall r=5 -> -1.
	let sh = s5().shell(1.0);
	assert!(
		at(&sh, 4.0).abs() < eps && (at(&sh, 5.0) + 1.0).abs() < eps && at(&sh, 6.0).abs() < eps,
		"shell must make a symmetric two-sided wall (zeros at r=4,6; mid-wall -1)"
	);

	// CSG combinators are exact min / max / max(A,-B).
	let a = || Node::primitive(Sphere::new(Vec3::ZERO, 5.0));
	let b = || Node::primitive(Sphere::new(Vec3::new(6.0, 0.0, 0.0), 5.0));
	for p in [Vec3::new(3.0, 0.0, 0.0), Vec3::new(1.0, 2.0, 0.0), Vec3::new(8.0, 1.0, -1.0)] {
		let (da, db) = (a().distance(p), b().distance(p));
		assert!(
			(a().union(b()).distance(p) - da.min(db)).abs() < eps
				&& (a().intersection(b()).distance(p) - da.max(db)).abs() < eps
				&& (a().difference(b()).distance(p) - da.max(-db)).abs() < eps,
			"CSG field at {p:?}: union=min, intersection=max, difference=max(A,-B) must hold exactly (A={da} B={db})"
		);
	}
}
