// Copyright (c) LMCAD. Licensed under the MIT License.

//! TRI-BENCHMARK — one assembly, three parts, three representations at full power:
//! `base` pure exact B-rep (revolve ∪ filleted boss, chained counterbores),
//! `damper` pure implicit (field-graded gyroid puck — soft top, stiff bottom),
//! `cap` true hybrid (`hybrid_boolean`: exact ring ⊕ beam-lattice field operand).
//! Stacked coaxially, clearance-checked, exported. Exit non-zero on any failure.
//! Run: `cargo run --example tri_benchmark -p kernel-model --release` → tri_out/

use std::sync::Arc;

use kernel_brep::holes::{counterbore_hole, Fit};
use kernel_brep::math::{DVec2, DVec3};
use kernel_brep::{
	cylinder, difference, export_step, fillet_circular_rim, mass_properties, revolve, tessellate_adaptive_tol, union,
	validate, Surface,
};
use kernel_core::check_mesh;
use kernel_core::math::Affine3A;
use kernel_core::mesh::Mesh;
use kernel_implicit::{
	make_manifold, manifold_dual_contour, Aabb, BeamLattice, Cylinder as VoxCylinder, Gyroid, LatticeCell, Node,
	Resolution, Vec3,
};
use kernel_model::hybrid::{hybrid_boolean, HybridOperand};
use kernel_model::{Assembly, BooleanOp, Instance};

fn merge_into(dst: &mut Mesh, src: &Mesh, dz: f32) {
	let b = dst.positions.len() as u32;
	for p in &src.positions {
		dst.positions.push(kernel_core::math::Vec3::new(p.x, p.y, p.z + dz));
	}
	for t in src.triangles() {
		dst.push_triangle(b + t[0], b + t[1], b + t[2]);
	}
}

fn main() {
	let dir = "tri_out";
	std::fs::create_dir_all(dir).expect("mkdir");
	let mut ok = true;

	// ---- PART 1: pure B-rep base (machined flange + filleted boss + counterbores)
	let flange = revolve(
		&[DVec2::new(10.0, 0.0), DVec2::new(40.0, 0.0), DVec2::new(40.0, 7.0), DVec2::new(39.0, 8.0), DVec2::new(10.0, 8.0)],
		64,
	);
	let boss = cylinder(DVec3::new(0.0, 0.0, 8.0), DVec3::Z, 18.0, 22.0, 64);
	let boss = fillet_circular_rim(&boss, DVec3::new(18.0, 0.0, 30.0), 2.0, 8).expect("torus rim fillet");
	let mut base = difference(&union(&flange, &boss), &cylinder(DVec3::new(0.0, 0.0, -1.0), DVec3::Z, 10.0, 32.0, 64));
	for i in 0..4 {
		let a = i as f64 * std::f64::consts::FRAC_PI_2;
		base = counterbore_hole(&base, DVec3::new(28.0 * a.cos(), 28.0 * a.sin(), 8.0), DVec3::NEG_Z, 5.0, Fit::Medium, None)
			.expect("counterbore");
	}
	// Joinery: a 2 mm-deep recess in the boss top seats the damper (0.3 mm radial
	// clearance), so the stack REGISTERS instead of merely resting.
	base = difference(&base, &cylinder(DVec3::new(0.0, 0.0, 28.0), DVec3::Z, 17.2, 2.5, 64));
	let v = validate(&base);
	let torus = base.faces().filter(|&f| matches!(base.face(f).surface, Surface::Torus { .. })).count();
	let mp = mass_properties(&base);
	let base_mesh = tessellate_adaptive_tol(&base, 0.02);
	let p1 = v.closed && v.manifold && v.genus == 5 && torus > 0 && base_mesh.is_watertight();
	ok &= p1;
	println!(
		"  base (B-rep)   genus={} (want 5) torus_faces={} vol={:.0} CoM z={:.2} wt={} {}",
		v.genus, torus, mp.volume, mp.center_of_mass.z, base_mesh.is_watertight(),
		if p1 { "PASS" } else { "FAIL" }
	);
	base_mesh.write_stl_binary(format!("{dir}/base.stl")).unwrap();
	std::fs::write(format!("{dir}/base.step"), export_step(&base, "tri_base")).unwrap();

	// ---- PART 2: pure implicit damper (field-graded gyroid puck Ø34×20)
	let region = Aabb::from_center_half_extent(Vec3::new(0.0, 0.0, 10.0), Vec3::new(18.0, 18.0, 11.0));
	let grade: kernel_implicit::ScalarField = Arc::new(|p: Vec3| 0.25 - 0.025 * p.z); // +0.25 bottom → -0.25 top
	let damper_node = Node::primitive(Gyroid::new(region, 0.55, 1.3))
		.offset_by(grade, 0.3)
		.intersection(Node::primitive(VoxCylinder::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 20.0), 16.9)))
		// Joinery: solid end discs give the lattice real mating faces…
		.union(Node::primitive(VoxCylinder::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.5), 16.9)))
		.union(Node::primitive(VoxCylinder::new(Vec3::new(0.0, 0.0, 18.5), Vec3::new(0.0, 0.0, 20.0), 16.9)))
		// …and a Ø9 clearance path so an M8 threaded rod clamps the whole stack.
		.difference(Node::primitive(VoxCylinder::new(Vec3::new(0.0, 0.0, -1.0), Vec3::new(0.0, 0.0, 21.0), 4.5)));
	// TPMS saddles pinch under narrow-band surface nets; the manifold mesher is
	// the right extractor here (the same lesson the lattice mount taught).
	let mut damper = manifold_dual_contour(&damper_node, region.pad(1.0), Resolution::VoxelSize(0.25));
	if check_mesh(&damper).non_manifold_edges > 0 || !damper.is_watertight() {
		damper = make_manifold(&damper);
		if check_mesh(&damper).non_manifold_edges > 0 || !damper.is_watertight() {
			damper = manifold_dual_contour(&damper_node, region.pad(1.2), Resolution::VoxelSize(0.27));
		}
	}
	let dr = check_mesh(&damper);
	let p2 = damper.is_watertight() && dr.non_manifold_edges == 0 && damper.triangle_count() > 50_000;
	ok &= p2;
	println!(
		"  damper (voxel) {} tris wt={} nme={} vol={:.0} (graded gyroid) {}",
		damper.triangle_count(), damper.is_watertight(), dr.non_manifold_edges,
		damper.signed_volume().abs(),
		if p2 { "PASS" } else { "FAIL" }
	);
	damper.write_stl_binary(format!("{dir}/damper.stl")).unwrap();
	damper.write_3mf(format!("{dir}/damper.3mf")).unwrap();

	// ---- PART 3: hybrid cap (exact ring ⊕ octet beam-lattice shroud, ONE op)
	let mut ring = difference(
		&cylinder(DVec3::ZERO, DVec3::Z, 20.0, 10.0, 64),
		&cylinder(DVec3::new(0.0, 0.0, -1.0), DVec3::Z, 10.5, 12.0, 64),
	);
	// Joinery: underside recess registers over the damper's top disc (Ø34.4 x 2).
	ring = difference(&ring, &cylinder(DVec3::new(0.0, 0.0, -1.0), DVec3::Z, 17.2, 3.0, 64));
	let shroud_region = Aabb::from_center_half_extent(Vec3::new(0.0, 0.0, 5.0), Vec3::new(27.0, 27.0, 5.0));
	let lattice = Node::primitive(BeamLattice::from_cells(shroud_region, LatticeCell::Octet, 7.0, 1.0))
		.intersection(Node::primitive(VoxCylinder::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 10.0), 27.0)));
	// Pre-mesh the lattice with the manifold extractor + snip remedy so the
	// operand is a clean 2-manifold and the EXACT stitch route engages.
	let mut lat_mesh = manifold_dual_contour(&lattice, shroud_region.pad(1.0), Resolution::VoxelSize(0.5));
	if check_mesh(&lat_mesh).non_manifold_edges > 0 || !lat_mesh.is_watertight() {
		lat_mesh = make_manifold(&lat_mesh);
	}
	let hy = hybrid_boolean(&ring, HybridOperand::Mesh(&lat_mesh), BooleanOp::Union, 0.5).expect("hybrid op");
	let kept = hy.solid.as_ref().map(|s| s.face_count()).unwrap_or(0);
	let p3 = hy.mesh.is_watertight() && check_mesh(&hy.mesh).non_manifold_edges == 0;
	ok &= p3;
	println!(
		"  cap (hybrid)   route={:?} {} tris wt={} solid_faces={} vol={:.0} {}",
		hy.route, hy.mesh.triangle_count(), hy.mesh.is_watertight(), kept,
		hy.mesh.signed_volume().abs(),
		if p3 { "PASS" } else { "FAIL" }
	);
	hy.mesh.write_stl_binary(format!("{dir}/cap.stl")).unwrap();

	// ---- ASSEMBLY: base(z0..30) + damper(z30..50) + cap(z50..60), coaxial
	let mut asm = Assembly::new();
	asm.add(Instance::from_mesh(&base_mesh, Affine3A::IDENTITY));
	asm.add(Instance::from_mesh(&damper, Affine3A::from_translation(Vec3::new(0.0, 0.0, 28.0))));
	asm.add(Instance::from_mesh(&hy.mesh, Affine3A::from_translation(Vec3::new(0.0, 0.0, 46.0))));
	// Stacked parts TOUCH by design: base→damper and damper→cap = exactly the
	// two designed contacts (the gearbox's contact-scan philosophy).
	let hits = asm.interferences(0.05, 0.4f32);
	let p4 = hits.len() == 2;
	ok &= p4;
	let mut merged = Mesh::default();
	merge_into(&mut merged, &base_mesh, 0.0);
	merge_into(&mut merged, &damper, 28.0);
	merge_into(&mut merged, &hy.mesh, 46.0);
	merged.write_stl_binary(format!("{dir}/tri_assembly.stl")).unwrap();
	println!(
		"  assembly       3 instances, designed_contacts={} (want 2) merged {} tris {}",
		hits.len(), merged.triangle_count(),
		if p4 { "PASS" } else { "FAIL" }
	);

	println!("\n{} — wrote ./{dir}/", if ok { "TRI-BENCHMARK: ALL PASS" } else { "TRI-BENCHMARK: FAILED" });
	std::process::exit(if ok { 0 } else { 1 });
}
