// Copyright (c) LMCAD. Licensed under the MIT License.

//! General two-mesh booleans via the arrangement on triangulated solids.

use std::f64::consts::PI;

use kernel_brep::math::DVec3;
use std::collections::HashSet;

use kernel_brep::{
	cuboid, exact_boolean, exact_boolean_auto, mesh_difference, mesh_intersection, mesh_union,
	refine_seam_to_intersection, snap_seam_to_intersection, sphere, tessellate_default, Mesh, MeshBoolOp, Surface,
};

#[test]
fn two_boxes_boolean_to_inclusion_exclusion_volumes() {
	let a = tessellate_default(&cuboid(DVec3::splat(-1.0), DVec3::splat(1.0))); // [-1,1]³ = 8
	let b = tessellate_default(&cuboid(DVec3::ZERO, DVec3::splat(2.0))); // [0,2]³ = 8, overlap [0,1]³ = 1
	let u = mesh_union(&a, &b).signed_volume().abs();
	let i = mesh_intersection(&a, &b).signed_volume().abs();
	let d = mesh_difference(&a, &b).signed_volume().abs();
	assert!((u - 15.0).abs() < 0.3, "union {u} (expected 15)");
	assert!((i - 1.0).abs() < 0.3, "intersection {i} (expected 1)");
	assert!((d - 7.0).abs() < 0.3, "difference {d} (expected 7)");
}

#[test]
fn two_tessellated_spheres_intersect_in_a_lens() {
	// A curved mesh∩mesh boolean. Two r=8 spheres, centres 8 apart, overlap in a lens
	// of volume π(4r+d)(2r−d)²/12 = π·40·64/12 ≈ 670.2. NB: on fine curved tessellations
	// the arrangement's *volume* is close but the surface is not guaranteed watertight
	// (see the module docs); for a clean curved cut use `crate::curved_boolean`.
	let a = tessellate_default(&sphere(DVec3::ZERO, 8.0, 24, 16));
	let b = tessellate_default(&sphere(DVec3::X * 8.0, 8.0, 24, 16));
	let lens = mesh_intersection(&a, &b);
	assert!(lens.triangle_count() > 50, "lens should be a real solid, got {}", lens.triangle_count());
	let vol = lens.signed_volume().abs() as f64;
	let expected = PI * 40.0 * 64.0 / 12.0;
	assert!((vol - expected).abs() / expected < 0.1, "lens volume {vol} vs analytic {expected}");
}

#[test]
fn reversed_winding_operand_is_normalized() {
	// A closed mesh wound clockwise-from-outside must still boolean correctly —
	// mesh_to_solid re-orients it outward rather than building an inside-out solid.
	let a = tessellate_default(&cuboid(DVec3::splat(-1.0), DVec3::splat(1.0)));
	let mut b_cw = tessellate_default(&cuboid(DVec3::ZERO, DVec3::splat(2.0)));
	for t in b_cw.indices.chunks_exact_mut(3) {
		t.swap(1, 2); // reverse every triangle's winding
	}
	let u = mesh_union(&a, &b_cw).signed_volume().abs();
	assert!((u - 15.0).abs() < 0.3, "CW-wound operand should still union to 15, got {u}");
}

#[test]
fn disjoint_meshes_boolean_to_sane_results() {
	let a = tessellate_default(&sphere(DVec3::ZERO, 2.0, 16, 12));
	let b = tessellate_default(&sphere(DVec3::X * 10.0, 2.0, 16, 12));
	let one = 4.0 / 3.0 * PI * 8.0; // ≈ 33.5
	let u = mesh_union(&a, &b).signed_volume().abs() as f64;
	let i = mesh_intersection(&a, &b);
	let d = mesh_difference(&a, &b).signed_volume().abs() as f64;
	assert!((u - 2.0 * one).abs() / (2.0 * one) < 0.05, "disjoint union {u} vs {}", 2.0 * one);
	assert_eq!(i.triangle_count(), 0, "disjoint intersection should be empty");
	assert!((d - one).abs() / one < 0.05, "disjoint difference {d} vs {one}");
}

#[test]
fn empty_operand_booleans_are_defined() {
	let a = tessellate_default(&sphere(DVec3::ZERO, 2.0, 16, 12));
	let empty = Mesh::new();
	let va = a.signed_volume().abs();
	// A − ∅ = A;  A ∩ ∅ = ∅;  A ∪ ∅ = A.
	assert!((mesh_difference(&a, &empty).signed_volume().abs() - va).abs() < 0.5 * va.max(1.0));
	assert_eq!(mesh_intersection(&a, &empty).triangle_count(), 0);
	assert!((mesh_union(&a, &empty).signed_volume().abs() - va).abs() < 0.5 * va.max(1.0));
}

#[test]
fn snapping_makes_two_sphere_seam_exact() {
	// Boolean two analytic spheres, then snap the tessellation-level seam onto the
	// exact intersection circle of the two surfaces.
	let sa = Surface::Sphere { center: DVec3::ZERO, radius: 8.0 };
	let sb = Surface::Sphere { center: DVec3::X * 8.0, radius: 8.0 };
	let ma = tessellate_default(&sphere(DVec3::ZERO, 8.0, 24, 16));
	let mb = tessellate_default(&sphere(DVec3::X * 8.0, 8.0, 24, 16));
	let mut lens = mesh_intersection(&ma, &mb);
	let watertight = lens.is_watertight();

	let band = 0.15;
	let seam_err = |m: &Mesh| {
		let (mut max, mut cnt) = (0.0_f64, 0);
		for v in &m.positions {
			let p = v.as_dvec3();
			let (da, db) = (sa.signed_value(p).abs(), sb.signed_value(p).abs());
			if da < band && db < band {
				max = max.max(da.max(db));
				cnt += 1;
			}
		}
		(max, cnt)
	};
	let (before, n) = seam_err(&lens);
	snap_seam_to_intersection(&mut lens, &sa, &sb, band);
	let (after, _) = seam_err(&lens);

	assert!(n > 10, "should have seam vertices, got {n}");
	assert!(after < 1e-4, "snapped seam should lie on both spheres, off by {after}");
	assert!(before > 10.0 * after, "snap should sharpen the seam (before {before}, after {after})");
	assert_eq!(lens.is_watertight(), watertight, "snap must not change topology");
}

#[test]
fn exact_boolean_one_call_produces_an_exact_seam() {
	// The one-call exact two-solid boolean: tessellated booleans + auto seam-snap.
	let sa = Surface::Sphere { center: DVec3::ZERO, radius: 8.0 };
	let sb = Surface::Sphere { center: DVec3::X * 8.0, radius: 8.0 };
	let ma = tessellate_default(&sphere(DVec3::ZERO, 8.0, 24, 16));
	let mb = tessellate_default(&sphere(DVec3::X * 8.0, 8.0, 24, 16));
	let lens = exact_boolean(&ma, &mb, &sa, &sb, MeshBoolOp::Intersection, 0.15);

	let (mut max, mut n) = (0.0_f64, 0);
	for v in &lens.positions {
		let p = v.as_dvec3();
		let (da, db) = (sa.signed_value(p).abs(), sb.signed_value(p).abs());
		if da < 0.15 && db < 0.15 {
			max = max.max(da.max(db));
			n += 1;
		}
	}
	assert!(n > 10 && max < 1e-4, "exact_boolean seam should lie on both spheres (n={n}, off={max})");
	let expected = PI * 40.0 * 64.0 / 12.0;
	assert!((lens.signed_volume().abs() as f64 - expected).abs() / expected < 0.1, "lens volume off");

	// The auto-band variant infers a sensible band and snaps the seam exact too.
	let auto = exact_boolean_auto(&ma, &mb, &sa, &sb, MeshBoolOp::Intersection);
	let (mut amax, mut an) = (0.0_f64, 0);
	for v in &auto.positions {
		let p = v.as_dvec3();
		let (da, db) = (sa.signed_value(p).abs(), sb.signed_value(p).abs());
		if da < 0.15 && db < 0.15 {
			amax = amax.max(da.max(db));
			an += 1;
		}
	}
	assert!(an > 10 && amax < 1e-4, "exact_boolean_auto seam off by {amax}");
}

#[test]
fn refine_densifies_a_clean_mesh_along_a_surface_intersection() {
	// `refine_seam_to_intersection` is a general utility: it conformally bisects every
	// edge lying on `f ∩ g` and re-projects the midpoint onto the exact curve. On a
	// clean (watertight, oriented) mesh it must stay clean. Here: a sphere mesh densified
	// along its intersection with the z = 0 plane (the equator circle, radius 8).
	let f = Surface::Sphere { center: DVec3::ZERO, radius: 8.0 };
	let g = Surface::Plane { origin: DVec3::ZERO, normal: DVec3::Z };
	let mut m = tessellate_default(&sphere(DVec3::ZERO, 8.0, 24, 16));
	assert!(m.is_watertight());
	let before = m.triangle_count();

	refine_seam_to_intersection(&mut m, &f, &g, 0.6);

	assert!(m.is_watertight(), "refine must keep a clean mesh watertight");
	assert!(m.triangle_count() > before, "refine should densify near the curve");
	// No inverted sub-triangle: in a consistently-oriented mesh no directed edge repeats.
	let mut dir: HashSet<(u32, u32)> = HashSet::new();
	for t in m.indices.chunks_exact(3) {
		for &(a, b) in &[(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
			assert!(dir.insert((a, b)), "refine introduced an inverted sub-triangle");
		}
	}
	// New midpoints land exactly on the equator circle (z = 0, radius 8).
	for v in &m.positions {
		let p = v.as_dvec3();
		if f.signed_value(p).abs() < 1e-4 && g.signed_value(p).abs() < 1e-4 {
			assert!(p.z.abs() < 1e-4 && ((p.x * p.x + p.y * p.y).sqrt() - 8.0).abs() < 1e-3, "off the circle");
		}
	}
}

#[test]
fn fine_sphere_boolean_stays_fast_via_broadphase() {
	// ~3k triangles per sphere: the naïve O(n²) co-refine would do ~9M intersection
	// tests, but the AABB broadphase culls almost all of them. Correctness unchanged.
	let a = tessellate_default(&sphere(DVec3::ZERO, 8.0, 48, 32));
	let b = tessellate_default(&sphere(DVec3::X * 8.0, 8.0, 48, 32));
	let lens = mesh_intersection(&a, &b).signed_volume().abs() as f64;
	let expected = PI * 40.0 * 64.0 / 12.0;
	assert!((lens - expected).abs() / expected < 0.05, "fine lens volume {lens} vs {expected}");
}
