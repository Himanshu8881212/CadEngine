// Copyright (c) LMCAD. Licensed under the MIT License.

//! Unit tests for the crate root: the feature tree, evaluation and assemblies.

use kernel_core::math::{Affine3A, DMat3, DVec3, Vec3};
use kernel_core::mesh::Mesh;
use kernel_core::mesher::Resolution;
use kernel_implicit::ops::Node;
use kernel_implicit::primitives::{Cuboid, Sphere};

use super::*;

/// Mesh volume of a document at a fixed voxel size.
fn doc_volume(doc: &Document, vs: f32) -> f64 {
	doc.mesh(Resolution::VoxelSize(vs)).signed_volume()
}

/// A 4 × 2 rectangle sketch fully constrained and anchored at the origin, with the
/// index of its width [`SketchConstraint::Distance`] returned for parametric driving.
fn rectangle_sketch() -> (Sketch, usize) {
	let mut s = Sketch::new();
	let p0 = s.add_point(kernel_core::math::DVec2::new(0.1, -0.2));
	let p1 = s.add_point(kernel_core::math::DVec2::new(3.0, 0.05));
	let p2 = s.add_point(kernel_core::math::DVec2::new(2.9, 1.8));
	let p3 = s.add_point(kernel_core::math::DVec2::new(-0.1, 2.1));
	s.add_segment(p0, p1);
	s.add_segment(p1, p2);
	s.add_segment(p2, p3);
	s.add_segment(p3, p0);
	s.add_constraint(SketchConstraint::Fixed { point: p0, at: kernel_core::math::DVec2::ZERO });
	s.add_constraint(SketchConstraint::Horizontal { a: p0, b: p1 });
	s.add_constraint(SketchConstraint::Horizontal { a: p3, b: p2 });
	s.add_constraint(SketchConstraint::Vertical { a: p0, b: p3 });
	s.add_constraint(SketchConstraint::Vertical { a: p1, b: p2 });
	let width = s.add_constraint(SketchConstraint::Distance { a: p0, b: p1, distance: 4.0 });
	s.add_constraint(SketchConstraint::Distance { a: p0, b: p3, distance: 2.0 });
	(s, width)
}

#[test]
fn sketch_feature_re_extrudes_when_the_height_parameter_changes() {
	// A sketch-driven extrude in the feature tree: changing the height parameter
	// must re-extrude the 4×2 profile, so the B-rep volume tracks 8 × height.
	let (sketch, _) = rectangle_sketch();
	let mut doc = Document::new();
	doc.set_param("h", 5.0);
	let f = doc.add(Feature::ExtrudeSketch { sketch, height: Dim::param("h"), dims: vec![], draft: Dim::Literal(0.0) });
	doc.set_root(f);

	let vol5 = kernel_brep::volume(&doc.evaluate_brep().expect("sketch extrudes"));
	doc.set_param("h", 10.0);
	let vol10 = kernel_brep::volume(&doc.evaluate_brep().expect("sketch re-extrudes"));

	assert!(
		(vol5 - 40.0).abs() < 1e-6 && (vol10 - 80.0).abs() < 1e-6,
		"parametric extrude: vol(h=5)={vol5} (want 40), vol(h=10)={vol10} (want 80)"
	);
}

#[test]
fn sketch_feature_reshapes_when_a_width_dimension_parameter_changes() {
	// Drive the rectangle's WIDTH distance from a parameter. Editing it must change
	// the solved profile itself (not just height), so the volume tracks width×2×5.
	let (sketch, width) = rectangle_sketch();
	let mut doc = Document::new();
	doc.set_param("w", 4.0);
	let f = doc.add(Feature::ExtrudeSketch {
		sketch,
		height: Dim::Literal(5.0),
		dims: vec![(width, Dim::param("w"))],
		draft: Dim::Literal(0.0),
	});
	doc.set_root(f);

	let vol_w4 = kernel_brep::volume(&doc.evaluate_brep().expect("sketch extrudes"));
	doc.set_param("w", 7.0);
	let vol_w7 = kernel_brep::volume(&doc.evaluate_brep().expect("sketch reshapes"));

	assert!(
		(vol_w4 - 40.0).abs() < 1e-6 && (vol_w7 - 70.0).abs() < 1e-6,
		"parametric width: vol(w=4)={vol_w4} (want 40), vol(w=7)={vol_w7} (want 70)"
	);
}

#[test]
fn sketch_feature_drafts_the_walls_when_a_draft_parameter_is_set() {
	// The draft/taper op reachable end-to-end through the parametric tree: a 4×2
	// sketch extruded 5mm with a 0.05-rad draft slopes the walls inward (a moulded
	// boss). The result is a genus-0 watertight frustum whose volume matches the
	// prismatoid closed form; setting the draft parameter to 0 recovers the plain
	// 8×5 = 40 prism — so draft is a real, re-evaluable feature parameter.
	let (sketch, _) = rectangle_sketch();
	let mut doc = Document::new();
	doc.set_param("h", 5.0);
	doc.set_param("a", 0.05);
	let f = doc.add(Feature::ExtrudeSketch {
		sketch,
		height: Dim::param("h"),
		dims: vec![],
		draft: Dim::param("a"),
	});
	doc.set_root(f);

	let s = doc.evaluate_brep().expect("drafted sketch extrudes");
	let v = kernel_brep::validate(&s);
	let vol = kernel_brep::volume(&s);
	let h = 5.0_f64;
	let d = h * 0.05_f64.tan();
	// Prismatoid of a rectangle drafted by d on every side: bottom 4×2, mid
	// (4−d)×(2−d), top (4−2d)×(2−2d). Relative tol (volume() uses the f32 mesh).
	let prismatoid = h / 6.0 * (8.0 + 4.0 * (4.0 - d) * (2.0 - d) + (4.0 - 2.0 * d) * (2.0 - 2.0 * d));
	doc.set_param("a", 0.0);
	let plain = kernel_brep::volume(&doc.evaluate_brep().expect("draft=0 ⇒ plain prism"));
	assert!(
		v.closed && v.manifold && v.genus == 0 && (vol - prismatoid).abs() / prismatoid < 1e-5 && (plain - 40.0).abs() < 1e-6,
		"drafted extrude: genus-0 frustum vol≈{prismatoid} (got {vol}); draft=0 ⇒ 40 (got {plain}): {v:?}"
	);
}

#[test]
fn instances_mate_by_derived_brep_faces() {
	// Two 2×2×2 cubes. Derive cube A's +Z (top) face and cube B's −Z (bottom)
	// face straight from their B-reps, then mate them face-to-face: instance B
	// (starting far away) must move so its bottom face lands on A's top face.
	let a = kernel_brep::cuboid(DVec3::new(0.0, 0.0, 0.0), DVec3::new(2.0, 2.0, 2.0));
	let b = kernel_brep::cuboid(DVec3::new(0.0, 0.0, 0.0), DVec3::new(2.0, 2.0, 2.0));
	let face_facing = |s: &kernel_brep::Solid, want: DVec3| {
		s.faces().find_map(|f| {
			let (p, n) = s.face_plane(f)?;
			(n.dot(want) > 0.99).then_some((p, n))
		})
	};
	let (pa, na) = face_facing(&a, DVec3::Z).expect("A has a +Z face");
	let (pb, nb) = face_facing(&b, -DVec3::Z).expect("B has a -Z face");

	// Instance 0 (A) is ground at the identity; instance 1 (B) starts offset.
	let mut sys = ConstraintSystem::new(
		vec![Affine3A::IDENTITY, Affine3A::from_translation(Vec3::new(5.0, 4.0, 9.0))],
		vec![],
	);
	sys.add_face_mate(0, pa, na, 1, pb, nb);
	let residual = sys.solve(256);

	// B's bottom-face point, in world, must now meet A's top-face point.
	let wb = sys.transforms()[1].transform_point3(pb.as_vec3());
	assert!(
		residual < 1e-6 && (wb - pa.as_vec3()).length() < 1e-4,
		"derived face mate should seat B's face on A's: residual {residual}, gap {}",
		(wb - pa.as_vec3()).length()
	);
}

#[test]
fn assembly_mass_properties_match_meshing_the_whole_assembly() {
	// Two box parts at rigid poses (one rotated about Z). Summing each part's analytic
	// mass properties through its pose by the parallel-axis theorem (Assembly::
	// mass_properties) must equal meshing the whole assembly exactly and measuring it
	// — volume, center of mass and the full inertia tensor (products included).
	let box_doc = |sx: f64, sy: f64, sz: f64| {
		let mut doc = Document::new();
		let b = doc.add(Feature::Box {
			center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
			size: [Dim::Literal(sx), Dim::Literal(sy), Dim::Literal(sz)],
		});
		doc.set_root(b);
		doc
	};
	let mut asm = Assembly::new();
	asm.add(Instance::document(box_doc(2.0, 2.0, 2.0), Affine3A::from_translation(Vec3::new(-4.0, 0.0, 0.0))));
	asm.add(Instance::document(
		box_doc(4.0, 3.0, 2.0),
		Affine3A::from_translation(Vec3::new(6.0, 2.0, 1.0)) * Affine3A::from_rotation_z(0.6),
	));
	let combined = asm.mass_properties(Resolution::VoxelSize(0.5));
	let whole = asm.mesh_all_exact(1e-4, Resolution::VoxelSize(0.5)).mass_properties();
	let fro2 = |m: DMat3| m.x_axis.length_squared() + m.y_axis.length_squared() + m.z_axis.length_squared();
	let inertia_rel = (fro2(combined.inertia - whole.inertia) / fro2(whole.inertia)).sqrt();
	assert!(
		(combined.volume - whole.volume).abs() / whole.volume < 1e-5
			&& (combined.center_of_mass - whole.center_of_mass).length() / whole.center_of_mass.length() < 1e-5
			&& inertia_rel < 1e-5,
		"assembly combine vs whole-mesh: V {} vs {}, CoM {:?} vs {:?}, inertia rel {inertia_rel}",
		combined.volume,
		whole.volume,
		combined.center_of_mass,
		whole.center_of_mass
	);
}

#[test]
fn chamfered_cylinder_feature_rebuilds_when_the_chamfer_parameter_changes() {
	// The chamfer counterpart of the parametric rounded boss: a bigger 45° top-rim chamfer
	// removes more material, so editing the chamfer parameter shrinks the volume.
	let mut doc = Document::new();
	let f = doc.add(Feature::ChamferedCylinder {
		radius: Dim::Literal(5.0),
		height: Dim::Literal(12.0),
		chamfer: Dim::param("c"),
	});
	doc.set_root(f);
	doc.set_param("c", 1.0);
	let v1 = doc.mass_properties().expect("brep").volume;
	doc.set_param("c", 3.0);
	let v3 = doc.mass_properties().expect("brep").volume;
	assert!(
		v3 < v1 && v1 < std::f64::consts::PI * 25.0 * 12.0,
		"parametric chamfer: c=1 → vol {v1}, c=3 → vol {v3}"
	);
}

#[test]
fn filleted_cylinder_feature_rebuilds_when_the_fillet_parameter_changes() {
	// A parametric rounded boss (curved-edge rim fillet wired into the Document tree): a bigger
	// top-rim fillet removes more material, so editing the fillet parameter shrinks the volume,
	// and it stays below the sharp cylinder πR²h. Proves the torus-fillet feature is parametric.
	let mut doc = Document::new();
	let f = doc.add(Feature::FilletedCylinder {
		radius: Dim::Literal(5.0),
		height: Dim::Literal(12.0),
		fillet: Dim::param("fr"),
	});
	doc.set_root(f);
	doc.set_param("fr", 1.0);
	let v1 = doc.mass_properties().expect("brep").volume;
	doc.set_param("fr", 3.0);
	let v3 = doc.mass_properties().expect("brep").volume;
	let sharp = std::f64::consts::PI * 25.0 * 12.0;
	assert!(
		v3 < v1 && v1 < sharp,
		"parametric rounded boss: fr=1 → vol {v1}, fr=3 → vol {v3} (sharp cyl {sharp})"
	);
}

#[test]
fn document_mass_properties_track_a_parameter_edit() {
	// A parametric box: its mass properties come straight off the Document in one call and
	// update when a width parameter changes — proving parametric mass evaluation (the real
	// "what does my part weigh as I vary a dimension?" workflow) without manual evaluate_brep.
	let mut doc = Document::new();
	let b = doc.add(Feature::Box {
		center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
		size: [Dim::param("w"), Dim::Literal(4.0), Dim::Literal(2.0)],
	});
	doc.set_root(b);
	doc.set_param("w", 3.0);
	let v3 = doc.mass_properties().expect("brep").volume; // 3·4·2 = 24
	doc.set_param("w", 6.0);
	let v6 = doc.mass_properties().expect("brep").volume; // 6·4·2 = 48
	assert!(
		(v3 - 24.0).abs() < 1e-6 && (v6 - 48.0).abs() < 1e-6,
		"parametric mass: w=3 → vol {v3} (want 24), w=6 → vol {v6} (want 48)"
	);
}

#[test]
fn imported_mesh_becomes_an_assembly_part() {
	// An imported / scanned triangle mesh must drop straight into an assembly: lift a box
	// mesh through Instance::from_mesh (Mesh → winding-number SDF → node), place it, and
	// mesh the assembly — the result reproduces the box (bounds and volume) via the
	// mesh→SDF bridge, proving an interchange import becomes a first-class assembly part.
	let box_solid = kernel_brep::cuboid(DVec3::new(-2.0, -2.0, -2.0), DVec3::new(2.0, 2.0, 2.0));
	let box_mesh = kernel_brep::tessellate_default(&box_solid);
	let mut asm = Assembly::new();
	asm.add(Instance::from_mesh(&box_mesh, Affine3A::IDENTITY));
	let out = asm.mesh_all(Resolution::VoxelSize(0.25));
	let aabb = out.aabb();
	let vol = out.signed_volume().abs();
	assert!(
		out.triangle_count() > 0
			&& (aabb.min - Vec3::splat(-2.0)).length() < 0.6
			&& (aabb.max - Vec3::splat(2.0)).length() < 0.6
			&& (vol - 64.0).abs() / 64.0 < 0.15,
		"imported box part: {} tris, aabb {:?}..{:?}, vol {} (want ~64)",
		out.triangle_count(),
		aabb.min,
		aabb.max,
		vol
	);
}

#[test]
fn interference_volume_measures_the_overlap_of_two_boxes() {
	// Two 4³ boxes offset by 2 in x overlap in the slab x∈[0,2] → a 2×4×4 = 32 mm³ shared
	// volume. The voxel-sampled interference volume must recover that — the quantitative
	// clash metric the binary interferences flag can't give.
	let unit_box = || {
		let mut doc = Document::new();
		let b = doc.add(Feature::Box {
			center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
			size: [Dim::Literal(4.0), Dim::Literal(4.0), Dim::Literal(4.0)],
		});
		doc.set_root(b);
		doc
	};
	let mut asm = Assembly::new();
	asm.add(Instance::document(unit_box(), Affine3A::IDENTITY));
	asm.add(Instance::document(unit_box(), Affine3A::from_translation(Vec3::new(2.0, 0.0, 0.0))));
	let v = asm.interference_volume(0, 1, 0.2);
	assert!((v - 32.0).abs() / 32.0 < 0.05, "overlap volume {v} (want ~32)");
}

#[test]
fn assembly_interferences_flag_only_the_overlapping_parts() {
	// Three unit-ish cubes: A at the origin, B shifted +1 in x so it penetrates A, and C
	// far away. Interference detection must flag exactly the A–B clash and report the
	// true 8 mm gap between A and the distant C.
	let unit_box = || {
		let mut doc = Document::new();
		let b = doc.add(Feature::Box {
			center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
			size: [Dim::Literal(2.0), Dim::Literal(2.0), Dim::Literal(2.0)],
		});
		doc.set_root(b);
		doc
	};
	let mut asm = Assembly::new();
	asm.add(Instance::document(unit_box(), Affine3A::IDENTITY)); // A spans x∈[-1,1]
	asm.add(Instance::document(unit_box(), Affine3A::from_translation(Vec3::new(1.0, 0.0, 0.0)))); // B x∈[0,2], overlaps A
	asm.add(Instance::document(unit_box(), Affine3A::from_translation(Vec3::new(10.0, 0.0, 0.0)))); // C x∈[9,11], clear
	let hits = asm.interferences(1e-6, Resolution::VoxelSize(0.2));
	let gap_ac = asm.clearance(0, 2, Resolution::VoxelSize(0.2));
	assert!(
		hits == vec![(0, 1)] && (gap_ac - 8.0).abs() < 0.5,
		"interferences {hits:?} (want [(0,1)]); A–C clearance {gap_ac} (want ~8)"
	);
}

#[test]
fn assembly_checks_see_brep_only_parts() {
	// FRICTION #2 regression: catalog parts (and every other B-rep-only feature)
	// evaluate to None on the implicit half, so clearance/interferences/
	// interference_volume used to see EMPTY instances — `inf` clearance and no
	// clashes, silently, for exactly the parts a gearbox is made of. Three Ø8×20
	// catalog shafts along +Z: A at x=0, B at x=10 (surface gap 2.0 mm), C at
	// x=−6 (overlapping only A, by a 2 mm-deep lens: area 2r²·acos(d/2r) −
	// (d/2)·√(4r²−d²) ≈ 7.25 mm² × 20 mm ≈ 145 mm³ for the 32-gon facets).
	let shaft = || {
		let mut doc = Document::new();
		let s = doc.add(Feature::CatalogPart {
			part: CatalogPart::Shaft { d: Dim::Literal(8.0), length: Dim::Literal(20.0) },
		});
		doc.set_root(s);
		doc
	};
	let mut asm = Assembly::new();
	asm.add(Instance::document(shaft(), Affine3A::IDENTITY));
	asm.add(Instance::document(shaft(), Affine3A::from_translation(Vec3::new(10.0, 0.0, 0.0))));
	asm.add(Instance::document(shaft(), Affine3A::from_translation(Vec3::new(-6.0, 0.0, 0.0))));
	let gap_ab = asm.clearance(0, 1, Resolution::VoxelSize(0.4));
	let hits = asm.interferences(1e-6, Resolution::VoxelSize(0.4));
	let overlap_ac = asm.interference_volume(0, 2, 0.2);
	let prox = asm.proximity_pairs(3.0, 0.05, Resolution::VoxelSize(0.4));
	let prox_ok = prox.len() == 2
		&& prox[0].0 == 0 && prox[0].1 == 1 && (prox[0].2 - 2.0).abs() < 0.1
		&& prox[1].0 == 0 && prox[1].1 == 2 && prox[1].2 <= 1e-9;
	assert!(
		(gap_ab - 2.0).abs() < 0.1 && hits == vec![(0, 2)] && (overlap_ac - 145.0).abs() / 145.0 < 0.1 && prox_ok,
		"B-rep-only parts must be visible to the assembly checks (used to be inf/none/0 silently): \
		 A–B clearance {gap_ab} (want ~2.0), interferences {hits:?} (want [(0, 2)]), \
		 A–C overlap {overlap_ac} mm³ (want ~145), proximity {prox:?} (want [(0,1,~2.0), (0,2,0.0)])"
	);
}

#[test]
fn threaded_bolt_thread_adds_material_and_stays_watertight() {
	// Regression guard for the showcase bolt: a helical thread fused onto the shank at
	// the MESH level (its exact B-rep union self-intersects) must (a) keep the part
	// watertight and (b) ADD material vs the bare shank — the exact symptom that was
	// silently broken when the thread was dropped on a failed B-rep validity check.
	use std::f64::consts::TAU;
	let shank = kernel_brep::cylinder(DVec3::ZERO, DVec3::Z, 4.0, 20.0, 48);

	// A triangular thread crest swept along a helix climbing the shank.
	let (pitch, turns, steps) = (2.4_f64, 5.0_f64, 32usize);
	let n = (turns * steps as f64) as usize;
	let path: Vec<DVec3> = (0..=n)
		.map(|k| {
			let t = k as f64 / steps as f64;
			let a = t * TAU;
			DVec3::new(4.0 * a.cos(), 4.0 * a.sin(), 2.0 + t * pitch)
		})
		.collect();
	let hw = pitch * 0.25; // ridge ~half the pitch, leaving wide valleys that mesh watertight
	// Wound so the sweep's outward normals point away from the helix (positive volume);
	// the reverse order makes the sweep inside-out, which would carve a groove instead
	// of adding a thread ridge in the winding-number heal.
	let profile = vec![DVec3::new(4.0, 0.0, 2.0 + hw), DVec3::new(4.9, 0.0, 2.0), DVec3::new(4.0, 0.0, 2.0 - hw)];
	let thread = kernel_brep::sweep_solid(&profile, &path).expect("thread sweeps");
	assert!(kernel_brep::volume(&thread) > 0.0, "thread sweep should be outward (vol {})", kernel_brep::volume(&thread));

	let merge = |soup: &mut Mesh, src: &Mesh| {
		let base = soup.positions.len() as u32;
		for p in &src.positions {
			soup.positions.push(*p);
		}
		for t in src.triangles() {
			soup.push_triangle(base + t[0], base + t[1], base + t[2]);
		}
	};
	let shank_tess = kernel_brep::tessellate_default(&shank);
	let mut bolt_soup = shank_tess.clone();
	merge(&mut bolt_soup, &kernel_brep::tessellate_default(&thread));

	// Heal both at the same voxel size; comparing volumes is robust to voxel noise.
	let plain = watertight_mesh_of(&shank_tess, 0.25);
	let threaded = watertight_mesh_of(&bolt_soup, 0.25);
	assert!(
		plain.is_watertight() && threaded.is_watertight() && threaded.signed_volume() > plain.signed_volume() + 5.0,
		"thread must stay watertight and add material: plain_vol={} threaded_vol={}",
		plain.signed_volume(),
		threaded.signed_volume()
	);
}

#[test]
fn precise_mesh_is_exact_and_watertight_for_curved_solids() {
	// The precision AI path. A STANDALONE cylinder meshes micron-fine via the EXACT analytic
	// tessellation: every lateral chord lies within ~the tolerance of the true radius, no
	// voxel grid. A drilled plate (box − cylinder bore) meshes WATERTIGHT and fine the same
	// way — but its bore wall inherits the boolean's construction resolution (curved boolean
	// walls are not yet re-fitted to the analytic surface; tracked), so the micron chord
	// bound is asserted only on the standalone primitive, honestly.
	let plate = kernel_brep::difference(
		&kernel_brep::cuboid(DVec3::new(-10.0, -10.0, -5.0), DVec3::new(10.0, 10.0, 5.0)),
		&kernel_brep::cylinder(DVec3::new(0.0, 0.0, -6.0), DVec3::Z, 4.0, 12.0, 48),
	);
	let cyl = kernel_brep::cylinder(DVec3::ZERO, DVec3::Z, 5.0, 12.0, 24);
	let mp = precise_mesh(&plate, 0.005);
	let mc = precise_mesh(&cyl, 0.005);
	// Chord deviation of the standalone cylinder's lateral wall (vertices on radius 5) from
	// the true surface: the midpoint of each wall chord must sit within ~tol of radius 5.
	let mut max_dev = 0.0f64;
	for t in mc.indices.chunks_exact(3) {
		let p = [
			mc.positions[t[0] as usize].as_dvec3(),
			mc.positions[t[1] as usize].as_dvec3(),
			mc.positions[t[2] as usize].as_dvec3(),
		];
		let on_cyl = p.iter().all(|v| ((v.x * v.x + v.y * v.y).sqrt() - 5.0).abs() < 1e-2);
		// Exclude the flat caps (all three vertices on one z-plane); their rim-spanning
		// chords cut across the disk and are not a measure of curved-wall fidelity.
		let on_cap = p.iter().all(|v| v.z.abs() < 1e-3) || p.iter().all(|v| (v.z - 12.0).abs() < 1e-3);
		if on_cyl && !on_cap {
			for &(i, j) in &[(0, 1), (1, 2), (2, 0)] {
				let mid = (p[i] + p[j]) * 0.5;
				max_dev = max_dev.max(5.0 - (mid.x * mid.x + mid.y * mid.y).sqrt());
			}
		}
	}
	assert!(
		mp.is_watertight() && mp.triangle_count() > 1000 && mc.is_watertight() && mc.triangle_count() > 400 && max_dev > 0.0 && max_dev <= 0.005 * 1.5,
		"precise_mesh: plate wt={} tris={}, cyl wt={} tris={}, cyl chord_dev={max_dev} (want 0 < dev ≤ {})",
		mp.is_watertight(),
		mp.triangle_count(),
		mc.is_watertight(),
		mc.triangle_count(),
		0.005 * 1.5
	);
}

#[test]
fn watertight_mesh_of_fuses_self_intersecting_soup() {
	// Two overlapping boxes as raw triangle SOUP (no valid B-rep union between them)
	// heal into ONE watertight solid via the winding-number field — the move that lets
	// a self-intersecting helical thread fuse onto a bolt shank. Material is the union,
	// so the volume exceeds either box yet is less than their disjoint sum.
	let mut soup = kernel_brep::tessellate_default(&kernel_brep::cuboid(DVec3::new(-2.0, -2.0, -2.0), DVec3::new(2.0, 2.0, 2.0)));
	let b = kernel_brep::tessellate_default(&kernel_brep::cuboid(DVec3::new(0.0, 0.0, 0.0), DVec3::new(4.0, 4.0, 4.0)));
	let base = soup.positions.len() as u32;
	for p in &b.positions {
		soup.positions.push(*p);
	}
	for t in b.triangles() {
		soup.push_triangle(base + t[0], base + t[1], base + t[2]);
	}
	let healed = watertight_mesh_of(&soup, 0.2);
	let v = healed.signed_volume();
	assert!(
		healed.is_watertight() && v > 64.0 && v < 128.0,
		"fused soup must be a watertight union (64..128 mm³): wt={} vol={}",
		healed.is_watertight(),
		v
	);
}

#[test]
fn voxel_path_unions_a_tilted_box_watertight() {
	// The HYBRID point: the B-rep boolean used to choke on tilted / face-sharing
	// boxes, but the voxel/SDF path (min/max on signed distances + Manifold Dual
	// Contouring) is robust to them — a tilted wall unioned onto a base meshes
	// watertight regardless. This is what makes the hybrid stronger than either half.
	let mut doc = Document::new();
	let base = doc.add(Feature::Box {
		center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(4.0)],
		size: [Dim::Literal(80.0), Dim::Literal(70.0), Dim::Literal(8.0)],
	});
	let wall = doc.add(Feature::Box {
		center: [Dim::Literal(0.0), Dim::Literal(23.0), Dim::Literal(40.0)],
		size: [Dim::Literal(80.0), Dim::Literal(8.0), Dim::Literal(80.0)],
	});
	let tilted = doc.add(Feature::Transform { input: wall, xform: Affine3A::from_axis_angle(Vec3::X, 12.0_f32.to_radians()) });
	let u = doc.add(Feature::Boolean { op: BooleanOp::Union, a: base, b: tilted });
	doc.set_root(u);

	let mesh = doc.mesh(Resolution::VoxelSize(2.0));
	assert!(
		mesh.is_watertight() && mesh.signed_volume() > 0.0,
		"voxel union of a tilted box must mesh watertight: watertight={}, vol={}",
		mesh.is_watertight(),
		mesh.signed_volume()
	);
}

#[test]
fn shell_hollows_a_box_into_a_watertight_wall() {
	// The voxel-half SHELL op: hollow a solid into a thin wall, preserving outer
	// dimensions. A 10-cube shelled to a 1-thick wall keeps material 10³ − 8³ = 488,
	// and the two nested surfaces mesh watertight — a job the SDF half does robustly
	// while the exact B-rep half (no general face-offset) returns None.
	let mut doc = Document::new();
	let b = doc.add(Feature::Box {
		center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
		size: [Dim::Literal(10.0), Dim::Literal(10.0), Dim::Literal(10.0)],
	});
	let sh = doc.add(Feature::Shell { input: b, thickness: Dim::Literal(1.0) });
	doc.set_root(sh);

	let mesh = doc.mesh(Resolution::VoxelSize(0.25));
	let wall = 10.0_f64.powi(3) - 8.0_f64.powi(3); // 488: outer minus inner cavity
	assert!(
		mesh.is_watertight() && (mesh.signed_volume() - wall).abs() / wall < 0.1,
		"shelled box must be a watertight {wall}-volume wall: wt={} vol={}",
		mesh.is_watertight(),
		mesh.signed_volume()
	);
	// And the shell is voxel-half-only: the exact B-rep path has no shell yet.
	assert!(doc.evaluate_brep().is_none(), "shell must be absent on the B-rep path");
}

#[test]
fn smooth_union_blends_spheres_into_a_watertight_organic_solid() {
	// The signature ORGANIC workflow: three overlapping spheres smooth-unioned into a
	// metaball-style blob through the parametric tree. The voxel/SDF half meshes the
	// filleted junctions watertight (a hard union would leave sharp creases), and the
	// blend fuses them into one solid that is bigger than a single sphere yet smaller
	// than three disjoint ones.
	let mut doc = Document::new();
	let r = 5.0;
	let s0 = doc.add(Feature::Sphere { center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)], radius: Dim::Literal(r) });
	let s1 = doc.add(Feature::Sphere { center: [Dim::Literal(6.0), Dim::Literal(0.0), Dim::Literal(0.0)], radius: Dim::Literal(r) });
	let s2 = doc.add(Feature::Sphere { center: [Dim::Literal(3.0), Dim::Literal(5.0), Dim::Literal(0.0)], radius: Dim::Literal(r) });
	let b01 = doc.add(Feature::SmoothUnion { a: s0, b: s1, blend: Dim::Literal(2.0) });
	let blob = doc.add(Feature::SmoothUnion { a: b01, b: s2, blend: Dim::Literal(2.0) });
	doc.set_root(blob);

	let mesh = doc.mesh(Resolution::VoxelSize(0.4));
	let sphere_vol = 4.0 / 3.0 * std::f64::consts::PI * r * r * r; // ≈ 523.6
	let v = mesh.signed_volume();
	assert!(
		mesh.is_watertight() && v > 1.2 * sphere_vol && v < 3.0 * sphere_vol,
		"smooth-union blob must be a watertight organic solid (1.2..3 spheres): wt={} vol={} (sphere {sphere_vol})",
		mesh.is_watertight(),
		v
	);
	// Voxel-half-only: there is no exact analytic blend on the B-rep path.
	assert!(doc.evaluate_brep().is_none(), "smooth union must be absent on the B-rep path");
}

#[test]
fn smooth_difference_carves_a_watertight_organic_pocket() {
	// The organic CARVE workflow: a sphere smooth-subtracted from a box leaves a
	// rounded crater (a filleted pocket, not a sharp dimple). The voxel half meshes
	// it watertight, and material is removed so the result is strictly less than the
	// 20×20×10 = 4000 box yet keeps most of the block.
	let mut doc = Document::new();
	let block = doc.add(Feature::Box {
		center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
		size: [Dim::Literal(20.0), Dim::Literal(20.0), Dim::Literal(10.0)],
	});
	let tool = doc.add(Feature::Sphere { center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(5.0)], radius: Dim::Literal(4.0) });
	let carved = doc.add(Feature::SmoothDifference { a: block, b: tool, blend: Dim::Literal(1.5) });
	doc.set_root(carved);

	let mesh = doc.mesh(Resolution::VoxelSize(0.4));
	let v = mesh.signed_volume();
	assert!(
		mesh.is_watertight() && v < 4000.0 && v > 3000.0,
		"smooth-difference pocket must be a watertight carved block (3000..4000): wt={} vol={}",
		mesh.is_watertight(),
		v
	);
	assert!(doc.evaluate_brep().is_none(), "smooth difference must be absent on the B-rep path");
}

#[test]
fn smooth_intersection_of_two_spheres_is_a_watertight_lens() {
	// Smooth intersection keeps the rounded common volume of two overlapping spheres
	// (a lens), meshed watertight, smaller than either sphere yet non-empty.
	let mut doc = Document::new();
	let r = 5.0;
	let a = doc.add(Feature::Sphere { center: [Dim::Literal(-2.0), Dim::Literal(0.0), Dim::Literal(0.0)], radius: Dim::Literal(r) });
	let b = doc.add(Feature::Sphere { center: [Dim::Literal(2.0), Dim::Literal(0.0), Dim::Literal(0.0)], radius: Dim::Literal(r) });
	let lens = doc.add(Feature::SmoothIntersection { a, b, blend: Dim::Literal(1.0) });
	doc.set_root(lens);

	let mesh = doc.mesh(Resolution::VoxelSize(0.3));
	let sphere_vol = 4.0 / 3.0 * std::f64::consts::PI * r * r * r;
	let v = mesh.signed_volume();
	assert!(
		mesh.is_watertight() && v > 0.0 && v < sphere_vol,
		"smooth-intersection lens must be a watertight solid (0..one sphere {sphere_vol}): wt={} vol={}",
		mesh.is_watertight(),
		v
	);
}

#[test]
fn gyroid_feature_meshes_a_bounded_lattice_infill() {
	// TPMS lattice infill reachable END-TO-END as a Feature (the additive-
	// manufacturing workflow): a gyroid bounded to its box → a rich, in-bounds,
	// plausibly-sized lattice block via Document::mesh. HONEST: a TPMS shell has
	// saddle pinches, so the lattice is rich + closed but not guaranteed fully
	// watertight — we assert the same rich/bounded properties as the kernel-implicit
	// gyroid test, not watertightness.
	let mut doc = Document::new();
	let half = 20.0;
	let g = doc.add(Feature::Gyroid {
		center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
		size: [Dim::Literal(2.0 * half), Dim::Literal(2.0 * half), Dim::Literal(2.0 * half)],
		scale: Dim::Literal(0.35),
		thickness: Dim::Literal(0.30),
	});
	doc.set_root(g);

	let mesh = doc.mesh(Resolution::VoxelSize(0.8));
	let vol = mesh.signed_volume();
	let cube_vol = 8.0 * half * half * half;
	let bb = mesh.aabb();
	assert!(
		mesh.triangle_count() > 5000 && vol > 0.01 * cube_vol && vol < 0.6 * cube_vol && bb.min.x >= -(half as f32) - 1.0 && bb.max.x <= half as f32 + 1.0,
		"gyroid feature must mesh a rich bounded lattice: tris={} vol={} (cube {cube_vol})",
		mesh.triangle_count(),
		vol
	);
	assert!(doc.evaluate_brep().is_none(), "gyroid is voxel-half-only on the B-rep path");
}

#[test]
fn smooth_union_blend_radius_is_a_live_parameter() {
	// The blend radius is a real re-evaluable parameter: increasing it fuses the two
	// overlapping spheres more, adding fillet material, so the meshed volume grows.
	let mut doc = Document::new();
	let r = 5.0;
	let a = doc.add(Feature::Sphere { center: [Dim::Literal(-4.0), Dim::Literal(0.0), Dim::Literal(0.0)], radius: Dim::Literal(r) });
	let b = doc.add(Feature::Sphere { center: [Dim::Literal(4.0), Dim::Literal(0.0), Dim::Literal(0.0)], radius: Dim::Literal(r) });
	let u = doc.add(Feature::SmoothUnion { a, b, blend: Dim::param("k") });
	doc.set_root(u);

	doc.set_param("k", 0.5);
	let v_small = doc.mesh(Resolution::VoxelSize(0.4)).signed_volume();
	doc.set_param("k", 4.0);
	let v_big = doc.mesh(Resolution::VoxelSize(0.4)).signed_volume();
	assert!(
		v_small > 0.0 && v_big > v_small,
		"larger blend radius must add fillet material: v(k=0.5)={v_small} v(k=4)={v_big}"
	);
}

#[test]
fn gyroid_thickness_is_a_live_parameter() {
	// Infill density is editable end-to-end: the gyroid wall thickness is a
	// re-evaluable parameter, so increasing it thickens the lattice walls and adds
	// material (the same parametric story as the blend radius, for the lattice).
	let mut doc = Document::new();
	let half = 16.0;
	let g = doc.add(Feature::Gyroid {
		center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
		size: [Dim::Literal(2.0 * half), Dim::Literal(2.0 * half), Dim::Literal(2.0 * half)],
		scale: Dim::Literal(0.35),
		thickness: Dim::param("t"),
	});
	doc.set_root(g);

	doc.set_param("t", 0.2);
	let v_thin = doc.mesh(Resolution::VoxelSize(0.8)).signed_volume();
	doc.set_param("t", 0.5);
	let v_thick = doc.mesh(Resolution::VoxelSize(0.8)).signed_volume();
	assert!(
		v_thin > 0.0 && v_thick > v_thin,
		"thicker lattice walls must add material: vol(t=0.2)={v_thin} vol(t=0.5)={v_thick}"
	);
}

#[test]
fn gyroid_infills_a_part_via_intersection() {
	// The advertised infill workflow: intersect a gyroid lattice with a part to fill
	// it with lattice. A gyroid (bounded to a box containing the sphere) ∩ a sphere →
	// a lattice-filled ball: a rich mesh, non-empty, inside the sphere, with strictly
	// less material than the solid sphere.
	let mut doc = Document::new();
	let r = 12.0;
	let lattice = doc.add(Feature::Gyroid {
		center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
		size: [Dim::Literal(2.0 * r), Dim::Literal(2.0 * r), Dim::Literal(2.0 * r)],
		scale: Dim::Literal(0.4),
		thickness: Dim::Literal(0.35),
	});
	let part = doc.add(Feature::Sphere { center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)], radius: Dim::Literal(r) });
	let infilled = doc.add(Feature::Boolean { op: BooleanOp::Intersection, a: lattice, b: part });
	doc.set_root(infilled);

	let mesh = doc.mesh(Resolution::VoxelSize(0.6));
	let v = mesh.signed_volume();
	let sphere_vol = 4.0 / 3.0 * std::f64::consts::PI * r * r * r;
	let bb = mesh.aabb();
	assert!(
		mesh.triangle_count() > 2000 && v > 0.0 && v < sphere_vol && bb.min.x >= -(r as f32) - 1.0 && bb.max.x <= r as f32 + 1.0,
		"gyroid-infilled sphere must be a rich bounded lattice inside the sphere: tris={} vol={} (sphere {sphere_vol})",
		mesh.triangle_count(),
		v
	);
}

#[test]
fn document_watertight_brep_mesh_heals_a_curved_part() {
	// A parametric block with a cylindrical hole, meshed watertight in one call
	// through the document's B-rep + hybrid heal — the AI-facing one-shot path.
	let mut doc = Document::new();
	doc.set_param("r", 4.0);
	let block = doc.add(Feature::Box {
		center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
		size: [Dim::Literal(20.0), Dim::Literal(20.0), Dim::Literal(10.0)],
	});
	let hole = doc.add(Feature::Cylinder {
		center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
		radius: Dim::param("r"),
		height: Dim::Literal(14.0),
	});
	let part = doc.add(Feature::Boolean { op: BooleanOp::Difference, a: block, b: hole });
	doc.set_root(part);

	let mesh = doc.watertight_brep_mesh(1.0);
	let exact = 20.0 * 20.0 * 10.0 - std::f64::consts::PI * 16.0 * 10.0;
	assert!(
		mesh.is_watertight() && (mesh.signed_volume() - exact).abs() / exact < 0.08,
		"document B-rep heal should be watertight with plausible volume: wt={} vol={} (exact {exact})",
		mesh.is_watertight(),
		mesh.signed_volume()
	);
}

#[test]
fn curved_boolean_meshes_watertight_both_exactly_and_via_voxel_heal() {
	// A hex nut (hex prism − a cylindrical hole). The EXACT B-rep tessellation now meshes
	// this watertight DIRECTLY: the robust ear-clipper honours the boolean's near-collinear
	// annular rim instead of skipping a point into an overlapping sliver (see
	// brep_validity::boolean_annular_cap_tessellates_watertight_via_exact_path). The hybrid
	// VOXEL heal (tessellate → MeshSdf winding field → Manifold Dual Contouring) is the
	// robust fallback and must AGREE: it also returns a watertight mesh of the same volume,
	// so a part the exact path cannot close (a self-intersecting feature) still meshes —
	// that genuinely-non-watertight case is covered by watertight_mesh_of_fuses_self_intersecting_soup.
	let r = 7.5;
	let hex: Vec<kernel_brep::math::DVec2> = (0..6)
		.map(|i| {
			let a = std::f64::consts::PI / 6.0 + i as f64 * std::f64::consts::PI / 3.0;
			kernel_brep::math::DVec2::new(r * a.cos(), r * a.sin())
		})
		.collect();
	let prism = kernel_brep::extrude(&hex, 6.0);
	let hole = kernel_brep::cylinder(DVec3::new(0.0, 0.0, -1.0), DVec3::Z, 4.2, 8.0, 48);
	let nut = kernel_brep::difference(&prism, &hole);

	let raw = kernel_brep::tessellate_default(&nut);
	let healed = watertight_mesh(&nut, 1.0);
	// hex area (3√3/2)r² × height − cylinder π·4.2²·6.
	let exact = 1.5 * 3.0_f64.sqrt() * r * r * 6.0 - std::f64::consts::PI * 4.2 * 4.2 * 6.0;
	assert!(
		raw.is_watertight()
			&& healed.is_watertight()
			&& (raw.signed_volume() - exact).abs() / exact < 0.01
			&& (healed.signed_volume() - exact).abs() / exact < 0.08,
		"curved nut should mesh watertight both exactly and via heal: raw_wt={} raw_vol={} healed_wt={} healed_vol={} (exact {exact})",
		raw.is_watertight(),
		raw.signed_volume(),
		healed.is_watertight(),
		healed.signed_volume()
	);
}

#[test]
fn parametric_fillet_survives_a_split_edge_with_a_witness() {
	// A bar unioned across the top of a box splits some of the box's named edges
	// into collinear fragments sharing one EdgeName. Filleting such an edge WITHOUT
	// a witness fails (ambiguous); WITH a witness the parametric fillet picks the
	// nearest fragment and succeeds — so a named fillet survives an edit that splits
	// its edge instead of breaking the feature tree.
	let mut doc = Document::new();
	let a = doc.add(Feature::Box {
		center: [Dim::Literal(5.0), Dim::Literal(5.0), Dim::Literal(5.0)],
		size: [Dim::Literal(10.0), Dim::Literal(10.0), Dim::Literal(10.0)],
	});
	let bar = doc.add(Feature::Box {
		center: [Dim::Literal(5.0), Dim::Literal(5.0), Dim::Literal(11.0)],
		size: [Dim::Literal(14.0), Dim::Literal(4.0), Dim::Literal(6.0)],
	});
	let u = doc.add(Feature::Boolean { op: BooleanOp::Union, a, b: bar });
	doc.set_root(u);
	let solid = doc.evaluate_brep().expect("union evaluates");

	// The witness feature is the *ambiguity resolution*, which is deterministic: a split
	// name resolves to >1 fragments so `fillet_edge` reports EdgeAmbiguous, while a witness
	// selects the nearest single fragment (so the `_near` resolver never reports
	// EdgeAmbiguous). We assert that contrast on this one build — NOT the geometric round
	// outcome, which is not yet bit-reproducible across boolean rebuilds (a frontier item).
	use kernel_brep::FilletError;
	let mut counts: std::collections::BTreeMap<String, kernel_brep::EdgeName> = std::collections::BTreeMap::new();
	let mut seen: std::collections::BTreeMap<String, usize> = std::collections::BTreeMap::new();
	for e in solid.edges() {
		if let Some(n) = solid.edge_name(e) {
			let k = format!("{n:?}");
			*seen.entry(k.clone()).or_insert(0) += 1;
			counts.entry(k).or_insert(n);
		}
	}
	// First split name in deterministic (sorted) order.
	let split = seen
		.iter()
		.find(|(_, &c)| c > 1)
		.map(|(k, _)| counts[k])
		.expect("the box+bar union splits at least one named edge into fragments");

	// Without a witness the kernel reports the split edge ambiguous …
	assert!(
		matches!(kernel_brep::fillet_edge(&solid, split, 0.4), Err(FilletError::EdgeAmbiguous)),
		"a split edge name must be reported EdgeAmbiguous without a witness"
	);
	// … and a witness resolves it to a single fragment — never EdgeAmbiguous — for every
	// witness near the part (the nearest-fragment pick always disambiguates).
	let witnesses = [DVec3::new(0.0, 5.0, 10.0), DVec3::new(10.0, 5.0, 10.0), DVec3::ZERO, DVec3::splat(10.0)];
	let all_resolve = witnesses
		.iter()
		.all(|&wp| !matches!(kernel_brep::fillet_edge_near(&solid, split, 0.4, wp), Err(FilletError::EdgeAmbiguous)));
	assert!(all_resolve, "a witness must resolve the ambiguous split edge to one fragment");
}

#[test]
fn assembly_mesh_all_exact_keeps_brep_parts_crisp_not_voxelized() {
	// A placed assembly of B-rep parts meshes via the EXACT analytic tessellation, not the
	// voxel grid: two 4 mm boxes placed apart come out as exactly 2×12 = 24 crisp triangles
	// (a box is 12 tris), whereas the voxel mesh_all quantizes each into many more. Every
	// vertex is finite. This is what keeps a machined-component assembly micron-sharp.
	let unit_box = || {
		let mut d = Document::new();
		d.add(Feature::Box {
			center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
			size: [Dim::Literal(4.0), Dim::Literal(4.0), Dim::Literal(4.0)],
		});
		d
	};
	let mut asm = Assembly::new();
	asm.add(Instance::document(unit_box(), Affine3A::from_translation(Vec3::new(-5.0, 0.0, 0.0))));
	asm.add(Instance::document(unit_box(), Affine3A::from_translation(Vec3::new(5.0, 0.0, 0.0))));
	let exact = asm.mesh_all_exact(0.005, Resolution::VoxelSize(0.5));
	let voxel = asm.mesh_all(Resolution::VoxelSize(0.5));
	assert!(
		exact.triangle_count() == 24 && exact.positions.iter().all(|p| p.is_finite()) && voxel.triangle_count() > exact.triangle_count(),
		"exact assembly must be 24 crisp tris (2 boxes), not voxelized: exact={} voxel={}",
		exact.triangle_count(),
		voxel.triangle_count()
	);
}

#[test]
fn assembly_mates_two_parts_through_solve_mates() {
	// End-to-end through the Assembly API: two 2×2×2 cube parts, A grounded and B
	// placed far away. Derive A's +Z face and B's −Z face from their B-reps, mate
	// them face-to-face, and solve_mates → B's pose moves so its bottom face seats
	// on A's top face, and mesh_all reflects the moved part.
	let a_solid = kernel_brep::cuboid(DVec3::new(-1.0, -1.0, -1.0), DVec3::new(1.0, 1.0, 1.0));
	let b_solid = kernel_brep::cuboid(DVec3::new(-1.0, -1.0, -1.0), DVec3::new(1.0, 1.0, 1.0));
	let face = |s: &kernel_brep::Solid, want: DVec3| {
		s.faces().find_map(|f| {
			let (p, n) = s.face_plane(f)?;
			(n.dot(want) > 0.99).then_some((p, n))
		})
	};
	let (pa, na) = face(&a_solid, DVec3::Z).expect("A +Z face");
	let (pb, nb) = face(&b_solid, -DVec3::Z).expect("B -Z face");

	let cube = || Node::primitive(Cuboid::new(Vec3::ZERO, Vec3::splat(1.0)));
	let mut asm = Assembly::new();
	asm.add(Instance::node(cube(), Affine3A::IDENTITY)); // 0 = ground
	asm.add(Instance::node(cube(), Affine3A::from_translation(Vec3::new(5.0, 4.0, 9.0))));

	let residual = asm.solve_mates(
		&[
			Constraint::Coincident { a: 0, a_point: pa, b: 1, b_point: pb },
			Constraint::Parallel { a: 0, a_dir: na, b: 1, b_dir: nb },
		],
		256,
	);

	let world_b = asm.instances[1].pose.transform_point3(pb.as_vec3());
	let mesh = asm.mesh_all(Resolution::VoxelSize(0.5));
	assert!(
		residual < 1e-6 && (world_b - pa.as_vec3()).length() < 1e-4 && !mesh.is_empty(),
		"solve_mates should seat B on A and mesh: residual {residual}, gap {}, tris {}",
		(world_b - pa.as_vec3()).length(),
		mesh.triangle_count()
	);
}

#[test]
fn instances_mate_coaxial_by_derived_cylinder_axes() {
	// A shaft and a sleeve, each a cylinder along Z. Read each one's axis straight
	// off its B-rep (a lateral cylindrical face), then concentric-mate them: the
	// misaligned sleeve must rotate + translate so its axis is collinear with the
	// shaft's (the Z axis).
	let shaft = kernel_brep::cylinder(DVec3::ZERO, DVec3::Z, 2.0, 10.0, 32);
	let sleeve = kernel_brep::cylinder(DVec3::ZERO, DVec3::Z, 3.0, 6.0, 32);
	let axis_of = |s: &kernel_brep::Solid| s.faces().find_map(|f| s.face_axis(f));
	let (pa, da) = axis_of(&shaft).expect("shaft has a cylindrical face");
	let (pb, db) = axis_of(&sleeve).expect("sleeve has a cylindrical face");

	let mut sys = ConstraintSystem::new(
		vec![
			Affine3A::IDENTITY,
			Affine3A::from_translation(Vec3::new(3.0, 4.0, 5.0)) * Affine3A::from_axis_angle(Vec3::Y, 0.6),
		],
		vec![],
	);
	sys.add_axis_mate(0, pa, da, 1, pb, db);
	let residual = sys.solve(256);

	let pose = sys.transforms()[1];
	let b_dir = pose.transform_vector3(db.as_vec3()).normalize_or_zero().as_dvec3();
	let b_pt = pose.transform_point3(pb.as_vec3()).as_dvec3();
	let parallel = da.cross(b_dir).length();
	let rel = b_pt - pa;
	let offset = (rel - da * rel.dot(da)).length();
	assert!(
		residual < 1e-6 && parallel < 1e-4 && offset < 1e-4,
		"coaxial mate should make the axes collinear: residual {residual}, parallel {parallel}, offset {offset}"
	);
}

#[test]
fn touching_linear_pattern_fuses_into_one_solid() {
	// Pattern step EQUALS the cube size, so adjacent copies SHARE a face (touch,
	// not gap). Thanks to the coplanar boolean fix the four copies fuse into a
	// SINGLE solid — a 4×1×1 bar of volume 4, one shell — instead of fragmenting.
	let mut doc = Document::new();
	let cube = doc.add(Feature::Box {
		center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
		size: [Dim::Literal(1.0), Dim::Literal(1.0), Dim::Literal(1.0)],
	});
	let bar = doc.add(Feature::LinearPattern {
		input: cube,
		count: 4,
		step: [Dim::Literal(1.0), Dim::Literal(0.0), Dim::Literal(0.0)],
	});
	doc.set_root(bar);

	let solid = doc.evaluate_brep().expect("touching pattern evaluates");
	let v = kernel_brep::validate(&solid);
	assert!(
		v.is_valid() && v.shells == 1 && (kernel_brep::volume(&solid).abs() - 4.0).abs() < 1e-6,
		"touching pattern should fuse into one bar (1 shell, vol 4): {v:?} vol={}",
		kernel_brep::volume(&solid).abs()
	);
}

#[test]
fn curved_circular_pattern_of_cylinders_is_exact_via_brep() {
	// A circular pattern of DISJOINT cylinders (a bolt-circle hole pattern, pegs on a ring) now
	// builds EXACTLY via the B-rep: the copies are AABB-disjoint, so they merge by topology
	// (disjoint_union) instead of chaining boolean unions — which used to self-intersect and
	// corrupt the volume (e.g. 6 disjoint cylinders unioned read ~23% low). Six Ø4×4 pegs at
	// radius 15 → a valid 6-shell solid, free of self-intersection, of volume 6·π·2²·4.
	let lit3 = |x: f64, y: f64, z: f64| [Dim::Literal(x), Dim::Literal(y), Dim::Literal(z)];
	let mut d = Document::new();
	let peg = d.add(Feature::Cylinder { center: lit3(15.0, 0.0, 0.0), radius: Dim::Literal(2.0), height: Dim::Literal(4.0) });
	let ring = d.add(Feature::CircularPattern {
		input: peg,
		count: 6,
		axis_point: lit3(0.0, 0.0, 0.0),
		axis_dir: lit3(0.0, 0.0, 1.0),
		angle: Dim::Literal(std::f64::consts::TAU / 6.0),
	});
	d.set_root(ring);
	let solid = d.evaluate_brep().expect("circular pattern of cylinders evaluates");
	let v = kernel_brep::validate(&solid);
	let expected = 6.0 * std::f64::consts::PI * 2.0 * 2.0 * 4.0;
	assert!(
		v.is_valid() && v.shells == 6 && !kernel_brep::self_intersects(&solid) && (kernel_brep::volume(&solid).abs() - expected).abs() / expected < 0.03,
		"curved circular pattern must be exact (valid 6-shell, no self-int, vol ~{expected:.0}): {v:?} self_int={} vol={:.0}",
		kernel_brep::self_intersects(&solid),
		kernel_brep::volume(&solid).abs()
	);
}

#[test]
fn curved_circular_pattern_bolt_circle_is_watertight_and_correct_via_voxel() {
	// A bolt circle — a plate with a CIRCULAR PATTERN of cylindrical holes — is a ubiquitous
	// part. Its exact B-rep is NOT reliable here: a pattern chains boolean unions of the
	// copies, and chained unions of CURVED operands self-intersect (the result passes
	// validate()'s closed/manifold/genus checks but is geometrically corrupt — `self_intersects`
	// is true and the volume is far off). The robust route is the VOXEL/SDF half: Document::mesh
	// heals it into a watertight solid of the correct volume. Six Ø5 holes on a 40×40×6 plate →
	// plate 9600 − 6·π·2.5²·6 ≈ 8893 mm³.
	let lit3 = |x: f64, y: f64, z: f64| [Dim::Literal(x), Dim::Literal(y), Dim::Literal(z)];
	let mut d = Document::new();
	let plate = d.add(Feature::Box { center: lit3(0.0, 0.0, 0.0), size: lit3(40.0, 40.0, 6.0) });
	let hole = d.add(Feature::Cylinder { center: lit3(15.0, 0.0, 0.0), radius: Dim::Literal(2.5), height: Dim::Literal(8.0) });
	let holes = d.add(Feature::CircularPattern {
		input: hole,
		count: 6,
		axis_point: lit3(0.0, 0.0, 0.0),
		axis_dir: lit3(0.0, 0.0, 1.0),
		angle: Dim::Literal(std::f64::consts::TAU / 6.0),
	});
	let bolt_circle = d.add(Feature::Boolean { op: BooleanOp::Difference, a: plate, b: holes });
	d.set_root(bolt_circle);
	let mesh = d.mesh(Resolution::VoxelSize(0.4));
	let expected = 9600.0 - 6.0 * std::f64::consts::PI * 2.5 * 2.5 * 6.0;
	assert!(
		mesh.is_watertight() && (mesh.signed_volume() - expected).abs() / expected < 0.02,
		"bolt circle (voxel path) must be watertight with the correct volume ~{expected:.0}: wt={} vol={:.0}",
		mesh.is_watertight(),
		mesh.signed_volume()
	);
}

#[test]
fn linear_pattern_repeats_a_feature_parametrically() {
	// Four unit cubes stepped 3 mm apart (a clear gap, so no shared face planes):
	// the pattern is a valid solid of volume 4×1. Widening the step keeps it 4
	// disjoint cubes (still volume 4); the count drives how many copies appear.
	let mut doc = Document::new();
	doc.set_param("gap", 3.0);
	let cube = doc.add(Feature::Box {
		center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
		size: [Dim::Literal(1.0), Dim::Literal(1.0), Dim::Literal(1.0)],
	});
	let pat = doc.add(Feature::LinearPattern {
		input: cube,
		count: 4,
		step: [Dim::param("gap"), Dim::Literal(0.0), Dim::Literal(0.0)],
	});
	doc.set_root(pat);

	let solid = doc.evaluate_brep().expect("pattern evaluates");
	let v = kernel_brep::validate(&solid);
	assert!(
		v.is_valid() && v.shells == 4 && (kernel_brep::volume(&solid).abs() - 4.0).abs() < 1e-6,
		"4 spaced cubes should be a valid 4-shell solid of volume 4: {v:?} vol={}",
		kernel_brep::volume(&solid).abs()
	);
}

#[test]
fn mirror_reflects_a_feature_across_a_plane() {
	// A unit cube centred at x=3 (so it sits fully in x>0, with a gap from the
	// plane) mirrored across x=0 → two cubes at x=±3: a valid 2-shell solid of
	// volume 2×1, each correctly oriented (positive volume, not inside-out).
	let mut doc = Document::new();
	let cube = doc.add(Feature::Box {
		center: [Dim::Literal(3.0), Dim::Literal(0.0), Dim::Literal(0.0)],
		size: [Dim::Literal(1.0), Dim::Literal(1.0), Dim::Literal(1.0)],
	});
	let m = doc.add(Feature::Mirror {
		input: cube,
		plane_point: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
		plane_normal: [Dim::Literal(1.0), Dim::Literal(0.0), Dim::Literal(0.0)],
	});
	doc.set_root(m);

	let solid = doc.evaluate_brep().expect("mirror evaluates");
	let v = kernel_brep::validate(&solid);
	assert!(
		v.is_valid() && v.shells == 2 && (kernel_brep::volume(&solid).abs() - 2.0).abs() < 1e-6,
		"mirrored cube should be a valid 2-shell solid of volume 2: {v:?} vol={}",
		kernel_brep::volume(&solid).abs()
	);
}

#[test]
fn mirror_of_a_curved_part_is_exact_via_brep() {
	// Mirroring a CURVED part across a non-cutting plane now builds EXACTLY via the B-rep: the
	// part and its reflection are AABB-disjoint, so they merge by topology (disjoint_union)
	// instead of a boolean union — which on disjoint curved solids self-intersects and reads
	// the volume low. A Ø4×4 cylinder at x=10 mirrored across x=0 → a valid 2-shell solid, free
	// of self-intersection, of volume 2·π·2²·4.
	let lit3 = |x: f64, y: f64, z: f64| [Dim::Literal(x), Dim::Literal(y), Dim::Literal(z)];
	let mut doc = Document::new();
	let cyl = doc.add(Feature::Cylinder { center: lit3(10.0, 0.0, 0.0), radius: Dim::Literal(2.0), height: Dim::Literal(4.0) });
	let m = doc.add(Feature::Mirror { input: cyl, plane_point: lit3(0.0, 0.0, 0.0), plane_normal: lit3(1.0, 0.0, 0.0) });
	doc.set_root(m);
	let solid = doc.evaluate_brep().expect("curved mirror evaluates");
	let v = kernel_brep::validate(&solid);
	let expected = 2.0 * std::f64::consts::PI * 2.0 * 2.0 * 4.0;
	assert!(
		v.is_valid() && v.shells == 2 && !kernel_brep::self_intersects(&solid) && (kernel_brep::volume(&solid).abs() - expected).abs() / expected < 0.03,
		"mirrored cylinder must be exact (valid 2-shell, no self-int, vol ~{expected:.0}): {v:?} self_int={} vol={:.0}",
		kernel_brep::self_intersects(&solid),
		kernel_brep::volume(&solid).abs()
	);
}

#[test]
fn circular_pattern_repeats_a_feature_around_an_axis() {
	// Six unit cubes at radius 5 from the Z axis, stepped 60° apart: a ring of 6.
	// Adjacent centres are 5 mm apart (>> the 1 mm cube), so copies never touch →
	// a valid 6-shell solid of volume 6×1.
	let mut doc = Document::new();
	let cube = doc.add(Feature::Box {
		center: [Dim::Literal(5.0), Dim::Literal(0.0), Dim::Literal(0.0)],
		size: [Dim::Literal(1.0), Dim::Literal(1.0), Dim::Literal(1.0)],
	});
	let ring = doc.add(Feature::CircularPattern {
		input: cube,
		count: 6,
		axis_point: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
		axis_dir: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(1.0)],
		angle: Dim::Literal(std::f64::consts::FRAC_PI_3), // 60°
	});
	doc.set_root(ring);

	let solid = doc.evaluate_brep().expect("circular pattern evaluates");
	let v = kernel_brep::validate(&solid);
	assert!(
		v.is_valid() && v.shells == 6 && (kernel_brep::volume(&solid).abs() - 6.0).abs() < 1e-6,
		"6-box ring should be a valid 6-shell solid of volume 6: {v:?} vol={}",
		kernel_brep::volume(&solid).abs()
	);
}

/// Build a 40 × 40 × 10 plate with a centred through-hole of radius `hole_r`.
fn plate_with_hole() -> Document {
	let mut doc = Document::new();
	doc.set_param("hole_r", 4.0);
	let plate = doc.add(Feature::Box {
		center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
		size: [Dim::Literal(40.0), Dim::Literal(40.0), Dim::Literal(10.0)],
	});
	// Cylinder taller than the plate so it punches all the way through.
	let hole = doc.add(Feature::Cylinder {
		center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
		radius: Dim::param("hole_r"),
		height: Dim::Literal(20.0),
	});
	let part = doc.add(Feature::Boolean { op: BooleanOp::Difference, a: plate, b: hole });
	doc.set_root(part);
	doc
}

#[test]
fn parametric_update_larger_hole_shrinks_volume() {
	let mut doc = plate_with_hole();

	let small_hole_vol = doc_volume(&doc, 0.6);

	// Parametric edit: widen the hole, then re-evaluate + re-mesh.
	doc.set_param("hole_r", 8.0);
	let large_hole_vol = doc_volume(&doc, 0.6);

	// Sanity-check against the closed-form plate-minus-cylinder volume.
	let plate = 40.0f64 * 40.0 * 10.0;
	let expect_small = plate - std::f64::consts::PI * 4.0f64.powi(2) * 10.0;
	let expect_large = plate - std::f64::consts::PI * 8.0f64.powi(2) * 10.0;

	assert!(
		large_hole_vol < small_hole_vol
			&& (small_hole_vol - expect_small).abs() / expect_small < 0.05
			&& (large_hole_vol - expect_large).abs() / expect_large < 0.05,
		"hole_r 4→8 should shrink volume: small={small_hole_vol} (≈{expect_small}), \
		 large={large_hole_vol} (≈{expect_large})"
	);
}

#[test]
fn assembly_bounds_span_both_instances() {
	// Two unit-ish boxes (full side 10) translated apart along x.
	let mk_box = || {
		let mut doc = Document::new();
		let id = doc.add(Feature::Box {
			center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
			size: [Dim::Literal(10.0), Dim::Literal(10.0), Dim::Literal(10.0)],
		});
		doc.set_root(id);
		doc
	};

	let mut asm = Assembly::new();
	asm.add(Instance::document(mk_box(), Affine3A::from_translation(Vec3::new(-20.0, 0.0, 0.0))));
	asm.add(Instance::document(mk_box(), Affine3A::from_translation(Vec3::new(20.0, 0.0, 0.0))));

	let bounds = asm.bounds();
	// Left box spans x∈[-25,-15], right box x∈[15,25] ⇒ combined x∈[-25,25].
	assert!(
		bounds.is_valid()
			&& bounds.min.x <= -24.9
			&& bounds.max.x >= 24.9
			&& (bounds.min.y + 5.0).abs() < 0.1
			&& (bounds.max.y - 5.0).abs() < 0.1,
		"combined bounds should span both instances, got {bounds:?}"
	);

	// And the merged mesh must contain geometry from both parts.
	let mesh = asm.mesh_all(Resolution::VoxelSize(1.0));
	assert!(!mesh.is_empty() && mesh.aabb().min.x <= -24.9 && mesh.aabb().max.x >= 24.9);
}

#[test]
fn difference_features_mesh_to_a_watertight_manifold() {
	// A Difference feature makes a concave crease; the document mesher must return
	// a closed 2-manifold there (via Manifold Dual Contouring), not the
	// non-manifold edges plain Surface Nets leaves. Covers a through-hole plate
	// and an overlapping sphere−sphere cut (the case that exposed the bug).
	let plate = plate_with_hole().mesh(Resolution::VoxelSize(0.6));

	let mut doc = Document::new();
	let a = doc.add(Feature::Sphere {
		center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
		radius: Dim::Literal(8.0),
	});
	let b = doc.add(Feature::Sphere {
		center: [Dim::Literal(8.0), Dim::Literal(0.0), Dim::Literal(0.0)],
		radius: Dim::Literal(8.0),
	});
	let cut = doc.add(Feature::Boolean { op: BooleanOp::Difference, a, b });
	doc.set_root(cut);
	let spheres = doc.mesh(Resolution::VoxelSize(0.5));

	assert_eq!(
		(plate.is_watertight(), spheres.is_watertight()),
		(true, true),
		"difference features must mesh to a watertight manifold"
	);
}

#[test]
fn brep_document_face_names_survive_a_parameter_edit() {
	use kernel_brep::{validate, FaceSource};
	// A parametric box with a corner carved by a cutter, built as a B-rep. A face
	// from the cutter (operand B) carries a persistent name; after moving the
	// cutter and re-evaluating, the same logical face is re-selected by that name —
	// topological naming working end-to-end through the Document layer.
	let lit3 = |x: f64, y: f64, z: f64| [Dim::Literal(x), Dim::Literal(y), Dim::Literal(z)];
	let mut doc = Document::new();
	doc.set_param("c", 5.0);
	let a = doc.add(Feature::Box { center: lit3(0.0, 0.0, 0.0), size: lit3(10.0, 10.0, 10.0) });
	let b = doc.add(Feature::Box {
		center: [Dim::param("c"), Dim::param("c"), Dim::param("c")],
		size: lit3(10.0, 10.0, 10.0),
	});
	let d = doc.add(Feature::Boolean { op: BooleanOp::Difference, a, b });
	doc.set_root(d);

	let s1 = doc.evaluate_brep().expect("brep document evaluates");
	assert!(validate(&s1).is_valid(), "brep document is a valid solid: {:?}", validate(&s1));
	let cut = s1.faces().find(|&f| s1.face_source(f) == Some(FaceSource::OperandB)).expect("a cut face from operand B");
	let name = s1.face_name(cut).unwrap();

	doc.set_param("c", 4.0);
	let s2 = doc.evaluate_brep().unwrap();
	assert!(!s2.faces_named(name).is_empty(), "stored face name re-resolves in the edited document");
}

#[test]
fn brep_document_fillet_survives_a_parameter_edit() {
	use kernel_brep::{tessellate_default, validate, EdgeName, FaceName, FaceSource, Surface};
	// A name-consuming feature in the parametric tree: store an edge's persistent
	// name, add a Fillet on it, then EDIT the box size and re-evaluate. The fillet
	// re-attaches to the corresponding edge of the rebuilt part — topological naming
	// load-bearing end-to-end through the Document, not just at the kernel level.
	let lit3 = |x: f64, y: f64, z: f64| [Dim::Literal(x), Dim::Literal(y), Dim::Literal(z)];
	let cyl_axis_xy = |s: &kernel_brep::Solid| -> (f64, f64) {
		s.faces()
			.find_map(|fc| match s.face(fc).surface {
				Surface::Cylinder { origin, .. } => Some((origin.x, origin.y)),
				_ => None,
			})
			.expect("a cylinder fillet face")
	};

	let mut doc = Document::new();
	doc.set_param("s", 10.0);
	let b = doc.add(Feature::Box { center: lit3(0.0, 0.0, 0.0), size: [Dim::param("s"), Dim::param("s"), Dim::param("s")] });
	doc.set_root(b);

	// The +X∧+Y edge of the box (faces 5 and 3 in cuboid's canonical order).
	let edge = EdgeName::new(
		FaceName { operand: FaceSource::Primitive, source_face: 5 },
		FaceName { operand: FaceSource::Primitive, source_face: 3 },
	);
	assert_eq!(doc.evaluate_brep().unwrap().edges_named(edge).len(), 1, "the named edge exists on the box");

	// Append the fillet feature referencing that persistent edge name.
	let f = doc.add(Feature::Fillet { input: b, edge, radius: Dim::Literal(2.0), near: None });
	doc.set_root(f);

	let r1 = doc.evaluate_brep().expect("filleted document evaluates");
	assert!(validate(&r1).is_valid() && tessellate_default(&r1).is_watertight(), "filleted doc valid+watertight: {:?}", validate(&r1));
	let (x1, y1) = cyl_axis_xy(&r1);
	assert!((x1 - 3.0).abs() < 1e-9 && (y1 - 3.0).abs() < 1e-9, "size-10 box fillet axis at +X+Y corner (3,3), got ({x1},{y1})");

	// PARAMETRIC EDIT: grow the box; the SAME stored name re-resolves and the fillet
	// re-attaches — its axis moves from (3,3) to the resized corner (8,8).
	doc.set_param("s", 20.0);
	let r2 = doc.evaluate_brep().expect("edited filleted document evaluates");
	assert!(validate(&r2).is_valid() && tessellate_default(&r2).is_watertight(), "edited filleted doc valid+watertight: {:?}", validate(&r2));
	let (x2, y2) = cyl_axis_xy(&r2);
	assert!((x2 - 8.0).abs() < 1e-9 && (y2 - 8.0).abs() < 1e-9, "size-20 box fillet re-attached to resized +X+Y corner (8,8), got ({x2},{y2})");
}

#[test]
fn feature_suppress_toggles_a_fillet_in_the_rebuild() {
	use kernel_brep::{validate, EdgeName, FaceName, FaceSource};
	// Suppress/unsuppress — the standard parametric-edit toggle: a fillet feature can
	// be switched OFF (the rebuild skips it, yielding its input — the plain box) and
	// back ON, without deleting it. The box is 10³ = 1000 exactly; the fillet rounds
	// an edge, removing a little material; suppressing restores the exact box.
	let lit3 = |x: f64, y: f64, z: f64| [Dim::Literal(x), Dim::Literal(y), Dim::Literal(z)];
	let mut doc = Document::new();
	let b = doc.add(Feature::Box { center: lit3(0.0, 0.0, 0.0), size: lit3(10.0, 10.0, 10.0) });
	let edge = EdgeName::new(
		FaceName { operand: FaceSource::Primitive, source_face: 5 },
		FaceName { operand: FaceSource::Primitive, source_face: 3 },
	);
	let f = doc.add(Feature::Fillet { input: b, edge, radius: Dim::Literal(2.0), near: None });
	doc.set_root(f);

	let vol_on = kernel_brep::volume(&doc.evaluate_brep().expect("filleted"));
	doc.set_suppressed(f, true);
	let suppressed = doc.evaluate_brep().expect("suppressed → plain box");
	let vol_supp = kernel_brep::volume(&suppressed);
	doc.set_suppressed(f, false);
	let vol_back = kernel_brep::volume(&doc.evaluate_brep().expect("unsuppressed → filleted again"));

	assert!(
		validate(&suppressed).is_valid()
			&& (vol_supp - 1000.0).abs() < 1e-6   // suppressed = the exact plain box
			&& vol_on < 999.0
			&& vol_on > 985.0                     // fillet removed a little material
			&& (vol_back - vol_on).abs() < 1e-6,  // unsuppress restores the fillet
		"suppress toggle: on={vol_on} suppressed={vol_supp} back={vol_back}"
	);
}

#[test]
fn brep_document_chamfer_feature_evaluates() {
	use kernel_brep::{tessellate_default, validate, EdgeName, FaceName, FaceSource, Surface};
	// The Chamfer feature is name-consuming like Fillet, but bevels flat. A box with
	// its +X∧+Y edge chamfered evaluates to a valid watertight solid carrying the
	// diagonal bevel plane and no cylindrical face.
	let lit3 = |x: f64, y: f64, z: f64| [Dim::Literal(x), Dim::Literal(y), Dim::Literal(z)];
	let mut doc = Document::new();
	let b = doc.add(Feature::Box { center: lit3(0.0, 0.0, 0.0), size: lit3(10.0, 10.0, 10.0) });
	let edge = EdgeName::new(
		FaceName { operand: FaceSource::Primitive, source_face: 5 },
		FaceName { operand: FaceSource::Primitive, source_face: 3 },
	);
	let c = doc.add(Feature::Chamfer { input: b, edge, radius: Dim::Literal(2.0), near: None });
	doc.set_root(c);

	let s = doc.evaluate_brep().expect("chamfered document evaluates");
	assert!(validate(&s).is_valid() && tessellate_default(&s).is_watertight(), "chamfered doc valid+watertight: {:?}", validate(&s));
	let inv_sqrt2 = std::f64::consts::FRAC_1_SQRT_2;
	assert!(
		s.faces().any(|f| matches!(s.face(f).surface,
			Surface::Plane { normal, .. } if (normal.x - inv_sqrt2).abs() < 1e-6 && (normal.y - inv_sqrt2).abs() < 1e-6 && normal.z.abs() < 1e-6)),
		"the chamfer feature adds the diagonal bevel plane"
	);
	assert!(!s.faces().any(|f| matches!(s.face(f).surface, Surface::Cylinder { .. })), "a chamfer has no cylindrical faces");
}

#[test]
fn empty_document_meshes_to_nothing() {
	let doc = Document::new();
	assert!(doc.evaluate().is_none() && doc.mesh(Resolution::VoxelSize(1.0)).is_empty());
}

#[test]
fn prebuilt_node_instance_meshes() {
	// A prebuilt (non-document) source still places and meshes.
	let node = Node::primitive(Sphere::new(Vec3::ZERO, 6.0));
	let mut asm = Assembly::new();
	asm.add(Instance::node(node, Affine3A::from_translation(Vec3::new(3.0, 0.0, 0.0))));
	let mesh = asm.mesh_all(Resolution::VoxelSize(0.5));
	let v = mesh.signed_volume();
	let expect = 4.0 / 3.0 * std::f64::consts::PI * 6.0f64.powi(3);
	assert!((v - expect).abs() / expect < 0.03, "prebuilt sphere vol {v} vs {expect}");
}
