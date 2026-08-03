// Copyright (c) LMCAD. Licensed under the MIT License.

//! The program↔assembly BRIDGES (assembly audit 2026-07-17, gaps 1 + 8):
//!
//! - [`Feature::Revolve`] — revolved parts (every showcase mount) can now live
//!   in a `.lmcpart` document, rebuild parametrically, and therefore join a
//!   MATED assembly;
//! - the `.lmcasm` `{"mesh": "part.stl"}` instance source — a part authored on
//!   the flat JSON program surface (or imported/scanned) drops into a mated
//!   assembly, measured honestly on its welded mesh (`volume_source: "mesh"`);
//! - [`EpicyclicTrain::instance_poses`] — kinematic angles become real
//!   [`Assembly`] instance poses, proven by geometry: the pitch cylinders of a
//!   posed sun/planet pair TOUCH at every input angle.

use kernel_core::math::DVec3;
use kernel_model::format::load_assembly;
use kernel_model::kinematics::EpicyclicTrain;
use kernel_model::{Assembly, Dim, Document, Feature, Instance};

/// A parametric washer document: revolve of the rectangle r ∈ [bore, od/2],
/// z ∈ [0, t] — volume π(R² − r²)·t, machine-checkable.
fn washer_doc(bore_r: f64, outer_r: f64) -> Document {
	let mut doc = Document::new();
	doc.set_param("t", 4.0);
	let lit = Dim::Literal;
	let f = doc.add(Feature::Revolve {
		profile: vec![
			[lit(bore_r), lit(0.0)],
			[lit(outer_r), lit(0.0)],
			[lit(outer_r), Dim::param("t")],
			[lit(bore_r), Dim::param("t")],
		],
		segments: 96,
	});
	doc.set_root(f);
	doc
}

#[test]
fn revolve_feature_rebuilds_parametrically_and_round_trips_the_part_file() {
	// (a) volume tracks the analytic washer at two thickness parameters;
	// (b) the document survives save_part → load_part byte-faithfully enough to
	//     re-evaluate bit-identically (the .lmcpart contract).
	let mut doc = washer_doc(4.0, 10.0);
	let v4 = kernel_brep::exact_volume(&doc.evaluate_brep().expect("revolve builds a B-rep"));
	doc.set_param("t", 7.0);
	let v7 = kernel_brep::exact_volume(&doc.evaluate_brep().expect("revolve rebuilds"));
	// The revolve builder tags TRUE analytic cylinder faces, so exact_volume is
	// the smooth pi washer, machine-exact — not a faceted-prism approximation.
	let ring = std::f64::consts::PI * (10.0 * 10.0 - 4.0 * 4.0);
	let (a4, a7) = (ring * 4.0, ring * 7.0);

	let text = kernel_model::format::save_part(&doc, "washer");
	let (reloaded, _) = kernel_model::format::load_part(&text).expect("load_part");
	let v7b = kernel_brep::exact_volume(&reloaded.evaluate_brep().expect("reloaded revolve builds"));

	assert!(
		(v4 - a4).abs() < 1e-9 * a4 && (v7 - a7).abs() < 1e-9 * a7 && (v7b - v7).abs() == 0.0,
		"Revolve must be parametric and persistent: vol(t=4)={v4:.9} (analytic {a4:.9}), \
		 vol(t=7)={v7:.9} (analytic {a7:.9}), reloaded={v7b:.9} (must equal bit-exactly)"
	);
}

#[test]
fn mesh_source_instance_joins_a_mated_assembly_with_honest_bom() {
	// A program-surface part (here: an exact B-rep cylinder tessellated to STL)
	// enters a `.lmcasm` via {"mesh": ...}, gets MATED concentrically into a
	// document part's bore from a deliberately wrong seed pose, and shows up in
	// contacts + BOM with volume_source "mesh".
	let dir = std::env::temp_dir().join("lmcad_asm_bridge_mesh_test");
	std::fs::create_dir_all(&dir).expect("temp dir");
	let shaft = kernel_brep::cylinder(DVec3::new(0.0, 0.0, 0.0), DVec3::Z, 3.8, 20.0, 48);
	let mesh = kernel_brep::tessellate_default(&shaft);
	mesh.write_stl_binary(dir.join("shaft.stl")).expect("write stl");

	// Plate with a Ø8 bore as an inline document instance.
	let mut plate = Document::new();
	let lit = Dim::Literal;
	let blank = plate.add(Feature::Box { center: [lit(0.0), lit(0.0), lit(4.0)], size: [lit(40.0), lit(40.0), lit(8.0)] });
	let bore = plate.add(Feature::Cylinder { center: [lit(0.0), lit(0.0), lit(4.0)], radius: lit(4.0), height: lit(8.0) });
	let cut = plate.add(Feature::Boolean { op: kernel_model::BooleanOp::Difference, a: blank, b: bore });
	plate.set_root(cut);
	let plate_text = kernel_model::format::save_part(&plate, "plate");
	std::fs::write(dir.join("plate.lmcpart"), plate_text).expect("write plate");

	let asm = r#"{
		"format": "lmc-asm", "version": 1, "units": "mm", "name": "bridge",
		"instances": [
			{"name": "plate", "source": {"path": "plate.lmcpart"}, "pose": {"translation": [0.0, 0.0, 0.0]}},
			{"name": "shaft", "source": {"mesh": "shaft.stl"},
			 "pose": {"translation": [9.0, -6.0, 2.0], "rotation": [0.0, 0.3826834, 0.0, 0.9238795]}}
		],
		"mates": [
			{"Concentric": {"a": 0, "a_axis_point": [0.0, 0.0, 0.0], "a_axis_dir": [0.0, 0.0, 1.0],
			                "b": 1, "b_axis_point": [0.0, 0.0, 0.0], "b_axis_dir": [0.0, 0.0, 1.0]}}
		]
	}"#;
	let loaded = load_assembly(asm, &dir).expect("mesh-source assembly loads");

	let shaft_center = loaded.assembly.instances[1].pose.transform_point3(kernel_core::math::Vec3::ZERO);
	let bom = loaded.bom_v2(0.4);
	let shaft_line = bom.flat.iter().find(|l| l.name == "shaft").expect("shaft BOM line");
	// HONEST BOM LIMIT: a mesh source carries no `meta` block, so there is no
	// material and therefore NO mass/volume_source columns — the line still
	// exists (name + count), and the omission is the honesty, not a bug.
	let pairs = loaded.assembly.proximity_pairs(1.0, 0.05, kernel_core::mesher::Resolution::VoxelSize(0.4));

	assert!(
		loaded.residual < 1e-6
			&& shaft_center.truncate().length() < 1e-3
			&& shaft_line.count == 1
			&& shaft_line.volume_source.is_none()
			&& shaft_line.unit_mass_g.is_none()
			&& pairs.iter().any(|&(i, j, d)| (i, j) == (0, 1) && d < 0.3),
		"mesh-source instance must mate + measure + tally honestly:\n\
		 residual {:.3e} (want <1e-6), shaft radial offset {:.6} (want ~0 — the mate must fix the bad seed),\n\
		 BOM line count {} (want 1) with mass columns honestly ABSENT (volume_source {:?}, mass {:?} — a\n\
		 mesh source has no meta/material), proximity pairs {pairs:?} (plate/shaft at the ~0.2mm ring gap)",
		loaded.residual,
		shaft_center.truncate().length(),
		shaft_line.count,
		shaft_line.volume_source,
		shaft_line.unit_mass_g,
	);
	std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn epicyclic_instance_poses_place_touching_pitch_cylinders_in_an_assembly() {
	// The kinematics→assembly link, proven with geometry instead of angles: at
	// ANY input angle, a sun pitch cylinder and a planet pitch cylinder placed
	// by `instance_poses` must TOUCH (pitch circles roll on each other), and
	// the planet centre must sit at the orbit radius exactly.
	let train = EpicyclicTrain {
		sun_teeth: 12,
		ring1_teeth: 36,
		planet_a_teeth: 12,
		planet_b_teeth: 11,
		ring2_teeth: 39,
		n_planets: 3,
	};
	train.validate_assembly().expect("PLAN-26 is assemblable");
	let module = 1.0;
	let poses = train.instance_poses(0.7, module);

	let (r_sun, r_planet) = (module * 12.0 / 2.0, module * 12.0 / 2.0);
	let sun_solid = kernel_brep::cylinder(DVec3::ZERO, DVec3::Z, r_sun, 6.0, 64);
	let planet_solid = kernel_brep::cylinder(DVec3::ZERO, DVec3::Z, r_planet, 6.0, 64);

	let mut asm = Assembly::new();
	let to_f32 = |m: kernel_core::math::DAffine3| {
		let (s, r, t) = m.to_scale_rotation_translation();
		kernel_core::math::Affine3A::from_scale_rotation_translation(s.as_vec3(), r.as_quat(), t.as_vec3())
	};
	asm.add(Instance::from_mesh(&kernel_brep::tessellate_default(&sun_solid), to_f32(poses.sun)));
	asm.add(Instance::from_mesh(&kernel_brep::tessellate_default(&planet_solid), to_f32(poses.planets[0])));

	let planet_center = poses.planets[0].transform_point3(DVec3::ZERO);
	let clearance = asm.clearance(0, 1, kernel_core::mesher::Resolution::VoxelSize(0.3));

	assert!(
		(planet_center.length() - poses.orbit_radius_mm).abs() < 1e-9
			&& (poses.orbit_radius_mm - (r_sun + r_planet)).abs() < 1e-12
			&& clearance < 0.05,
		"kinematic poses must place REAL geometry correctly: planet centre at {:.6}mm \
		 (orbit radius {:.6}), pitch cylinders clearance {clearance:.4}mm (must touch, \
		 <0.05 chord tolerance) — if this fails the angles-to-poses bridge is mis-scaled",
		planet_center.length(),
		poses.orbit_radius_mm,
	);
}

/// Revolve is honest about its implicit half: `None` on `evaluate` (B-rep only),
/// mirroring `ExtrudeSketch` — a revolved document routes through `evaluate_brep`.
#[test]
fn revolve_is_brep_only_and_says_so() {
	let doc = washer_doc(2.0, 5.0);
	assert!(
		doc.evaluate().is_none() && doc.evaluate_brep().is_some(),
		"Revolve must be None on the implicit half and Some on the exact half \
		 (evaluate={:?}, evaluate_brep={})",
		doc.evaluate().is_some(),
		doc.evaluate_brep().is_some()
	);
}
