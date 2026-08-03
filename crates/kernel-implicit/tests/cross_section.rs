// Copyright (c) LMCAD. Licensed under the MIT License.

//! Cross-sectioning a meshed solid by a plane: the slicer must produce the right
//! number of closed contours, land them exactly on the cutting plane, and
//! enclose the correct area/perimeter — validated against closed forms and on
//! shapes whose section has more than one loop (disjoint solids, a torus annulus).

use kernel_implicit::surface_nets;
use kernel_implicit::{Cuboid, Cylinder, Node, Resolution, Sdf, Sphere, Torus, Vec3};
use std::f64::consts::PI;

/// In-plane enclosed area and perimeter of a contour loop (normal `n`).
fn area_perimeter(ring: &[Vec3], n: Vec3) -> (f64, f64) {
	let nd = n.normalize().as_dvec3();
	let (mut area2, mut perim) = (0.0f64, 0.0f64);
	for k in 0..ring.len() {
		let a = ring[k].as_dvec3();
		let b = ring[(k + 1) % ring.len()].as_dvec3();
		area2 += a.cross(b).dot(nd);
		perim += (b - a).length();
	}
	(area2.abs() * 0.5, perim)
}

#[test]
fn sphere_section_is_a_circle_on_the_plane() {
	// A sphere of radius 10 cut at z = 4 sections to a circle of radius √(100−16).
	let s = Node::primitive(Sphere::new(Vec3::ZERO, 10.0));
	let m = surface_nets(&s, s.bounds(), Resolution::VoxelSize(0.3));
	let loops = m.cross_section(Vec3::new(0.0, 0.0, 4.0), Vec3::Z);
	assert_eq!(loops.len(), 1, "a sphere section is a single closed loop");

	// Every contour point lies exactly on the cutting plane (the slicer interpolates
	// onto it), independent of the mesh resolution.
	for p in &loops[0] {
		assert!((p.z - 4.0).abs() < 1e-3, "section point off the plane: z={}", p.z);
	}
	let r = (100.0f64 - 16.0).sqrt();
	let (area, perim) = area_perimeter(&loops[0], Vec3::Z);
	assert!((area - PI * r * r).abs() / (PI * r * r) < 0.02, "section area {area} vs {}", PI * r * r);
	assert!((perim - 2.0 * PI * r).abs() / (2.0 * PI * r) < 0.02, "section perimeter {perim} vs {}", 2.0 * PI * r);
}

#[test]
fn box_section_is_a_square_of_known_area() {
	// A 20 mm cube cut through the middle sections to a 20×20 square.
	let c = Node::primitive(Cuboid::new(Vec3::ZERO, Vec3::splat(10.0)));
	let m = surface_nets(&c, c.bounds(), Resolution::VoxelSize(0.5));
	let loops = m.cross_section(Vec3::ZERO, Vec3::Z);
	assert_eq!(loops.len(), 1, "a convex box section is a single loop");
	let (area, perim) = area_perimeter(&loops[0], Vec3::Z);
	assert!((area - 400.0).abs() / 400.0 < 0.05, "box section area {area} vs 400");
	assert!((perim - 80.0).abs() / 80.0 < 0.05, "box section perimeter {perim} vs 80");
}

#[test]
fn two_disjoint_spheres_section_to_two_loops() {
	// A plane through both centers of two separated spheres yields two contours —
	// exercises multi-loop stitching.
	let two = Node::primitive(Sphere::new(Vec3::new(-15.0, 0.0, 0.0), 8.0))
		.union(Node::primitive(Sphere::new(Vec3::new(15.0, 0.0, 0.0), 8.0)));
	let m = surface_nets(&two, two.bounds(), Resolution::VoxelSize(0.4));
	let loops = m.cross_section(Vec3::ZERO, Vec3::Z);
	assert_eq!(loops.len(), 2, "two disjoint spheres section to two loops");
	for l in &loops {
		let (area, _) = area_perimeter(l, Vec3::Z);
		assert!((area - PI * 64.0).abs() / (PI * 64.0) < 0.03, "each loop is a Ø16 circle, got area {area}");
	}
}

#[test]
fn torus_section_in_its_plane_is_an_annulus() {
	// A torus (major 10, minor 3, axis Z) cut by its own mid-plane sections to an
	// annulus: an outer ring (r≈13) and an inner ring (r≈7) — two nested loops.
	let t = Node::primitive(Torus::new(Vec3::ZERO, Vec3::Z, 10.0, 3.0));
	let m = surface_nets(&t, t.bounds(), Resolution::VoxelSize(0.25));
	let loops = m.cross_section(Vec3::ZERO, Vec3::Z);
	assert_eq!(loops.len(), 2, "a torus mid-plane section is two concentric loops");
	let mut radii: Vec<f64> = loops
		.iter()
		.map(|l| area_perimeter(l, Vec3::Z).1 / (2.0 * PI)) // perimeter → mean radius
		.collect();
	radii.sort_by(f64::total_cmp);
	assert!((radii[0] - 7.0).abs() < 0.4, "inner radius {} vs 7", radii[0]);
	assert!((radii[1] - 13.0).abs() < 0.4, "outer radius {} vs 13", radii[1]);
}

#[test]
fn plane_missing_the_solid_yields_no_contours() {
	let s = Node::primitive(Sphere::new(Vec3::ZERO, 10.0));
	let m = surface_nets(&s, s.bounds(), Resolution::VoxelSize(0.4));
	let loops = m.cross_section(Vec3::new(0.0, 0.0, 100.0), Vec3::Z);
	assert!(loops.is_empty(), "a plane that misses the solid sections to nothing");
}

#[test]
fn square_section_properties_match_closed_form() {
	// A 20 mm box sectioned at mid-height: a 20×20 square. A = 400, centroid at the
	// origin, second moment I = b·h³/12 on each axis, product of area ~0.
	let c = Node::primitive(Cuboid::new(Vec3::ZERO, Vec3::splat(10.0)));
	let m = surface_nets(&c, c.bounds(), Resolution::VoxelSize(0.3));
	let sp = m.section_properties(Vec3::ZERO, Vec3::Z).expect("section");
	assert!((sp.area - 400.0).abs() / 400.0 < 0.02, "area {} vs 400", sp.area);
	assert!(sp.centroid.length() < 0.3, "centroid {:?}", sp.centroid);
	let exact = 20.0 * 20.0f64.powi(3) / 12.0; // 13333
	assert!((sp.i_uu - exact).abs() / exact < 0.03 && (sp.i_vv - exact).abs() / exact < 0.03, "I ({},{}) vs {exact}", sp.i_uu, sp.i_vv);
	assert!(sp.i_uv.abs() / exact < 0.02, "product of area should ~0, got {}", sp.i_uv);
}

#[test]
fn circular_section_properties_match_closed_form() {
	// A cylinder sectioned across its axis: a disc. A = πr², I = πr⁴/4.
	let cyl = Node::primitive(Cylinder::new(Vec3::new(0.0, 0.0, -10.0), Vec3::new(0.0, 0.0, 10.0), 8.0));
	let m = surface_nets(&cyl, cyl.bounds(), Resolution::VoxelSize(0.3));
	let sp = m.section_properties(Vec3::ZERO, Vec3::Z).expect("section");
	let (a, i) = (PI * 64.0, PI * 8.0f64.powi(4) / 4.0);
	assert!((sp.area - a).abs() / a < 0.02, "area {} vs {a}", sp.area);
	assert!((sp.i_uu - i).abs() / i < 0.03 && (sp.i_vv - i).abs() / i < 0.03, "I ({},{}) vs {i}", sp.i_uu, sp.i_vv);
}

#[test]
fn tube_section_subtracts_the_bore() {
	// Annulus (R=10, r=6): the bore MUST be subtracted via even–odd nesting —
	// wrong handling would report π(R²+r²) instead of π(R²−r²).
	let outer = Node::primitive(Cylinder::new(Vec3::new(0.0, 0.0, -10.0), Vec3::new(0.0, 0.0, 10.0), 10.0));
	let inner = Node::primitive(Cylinder::new(Vec3::new(0.0, 0.0, -11.0), Vec3::new(0.0, 0.0, 11.0), 6.0));
	let tube = outer.difference(inner);
	let b = tube.bounds();
	let m = surface_nets(&tube, b, Resolution::VoxelSize(0.3));
	let sp = m.section_properties(Vec3::ZERO, Vec3::Z).expect("section");
	let a = PI * (100.0 - 36.0);
	let i = PI * (10.0f64.powi(4) - 6.0f64.powi(4)) / 4.0;
	assert!((sp.area - a).abs() / a < 0.03, "annulus area {} vs {a} (bore not subtracted?)", sp.area);
	assert!((sp.i_uu - i).abs() / i < 0.05, "annulus I {} vs {i}", sp.i_uu);
}
