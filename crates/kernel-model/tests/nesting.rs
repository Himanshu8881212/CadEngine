// Copyright (c) LMCAD. Licensed under the MIT License.

//! Assembly nesting (`asm_path` sub-assemblies) + BOM v2: recursive loading to
//! ≥3 levels with hierarchical names, rigid-unit parent mates, loud include
//! cycles, branch-dropping suppression, mass/part-number/material enrichment
//! with honest volume-source labels, the tree/flat rollup invariant, the CSV
//! golden line, and byte-identical determinism across repeated loads.

use std::path::{Path, PathBuf};

use kernel_core::math::{Affine3A, DVec3, Vec3};
use kernel_core::mesher::Resolution;
use kernel_model::format::{
	load_assembly, save_assembly, save_part, save_part_with_meta, AsmInstance, AsmSource, FormatError, MakeOrBuy, Material,
	PartBomMeta, VolumeSource,
};
use kernel_model::{Constraint, Dim, Document, Feature};

/// A unique per-test scratch directory under the system temp dir.
fn scratch_dir(name: &str) -> PathBuf {
	let dir = std::env::temp_dir().join(format!("lmcad_nesting_{name}_{}", std::process::id()));
	let _ = std::fs::remove_dir_all(&dir);
	std::fs::create_dir_all(&dir).expect("create scratch dir");
	dir
}

/// An axis-aligned box document centred at the origin.
fn box_doc(sx: f64, sy: f64, sz: f64) -> Document {
	let mut doc = Document::new();
	let b = doc.add(Feature::Box {
		center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
		size: [Dim::Literal(sx), Dim::Literal(sy), Dim::Literal(sz)],
	});
	doc.set_root(b);
	doc
}

/// A path-sourced instance at `pose`.
fn part_at(name: &str, file: &str, pose: Affine3A) -> AsmInstance {
	AsmInstance { name: Some(name.to_string()), source: AsmSource::Path(file.to_string()), pose, suppressed: false }
}

/// A sub-assembly instance at `pose`.
fn sub_at(name: &str, file: &str, pose: Affine3A) -> AsmInstance {
	AsmInstance { name: Some(name.to_string()), source: AsmSource::Assembly(file.to_string()), pose, suppressed: false }
}

/// Write `bottom.lmcasm` into `dir`: a 4×4×2 `base` and a 2×2×2 `cap` with the
/// cap's seed pose deliberately off and two INTERNAL mates seating it on the
/// base's top face — solved on load, the cap centre lands at (0, 0, 2) in the
/// sub-assembly's own frame.
fn write_bottom(dir: &Path) {
	std::fs::write(dir.join("base.lmcpart"), save_part(&box_doc(4.0, 4.0, 2.0), "base")).expect("write base");
	std::fs::write(dir.join("cap.lmcpart"), save_part(&box_doc(2.0, 2.0, 2.0), "cap")).expect("write cap");
	let instances = [
		part_at("base", "base.lmcpart", Affine3A::IDENTITY),
		part_at("cap", "cap.lmcpart", Affine3A::from_translation(Vec3::new(0.4, -0.3, 2.6))),
	];
	let mates = [
		Constraint::Coincident { a: 0, a_point: DVec3::new(0.0, 0.0, 1.0), b: 1, b_point: DVec3::new(0.0, 0.0, -1.0) },
		Constraint::Parallel { a: 0, a_dir: DVec3::Z, b: 1, b_dir: DVec3::Z },
	];
	let text = save_assembly("bottom", &instances, &mates).expect("bottom saves");
	std::fs::write(dir.join("bottom.lmcasm"), text).expect("write bottom.lmcasm");
}

#[test]
fn nested_assembly_three_levels_flattens_with_hierarchical_names_and_rollup() {
	// THE nesting contract end-to-end over THREE levels (top → mid → bottom):
	// `asm_path` sources save with byte-stable envelopes, load recursively
	// (each file's sources against its own directory), solve the bottom's OWN
	// mates first (cap seated at z=2 in the bottom frame), then stack the
	// parent poses rigidly — so the cap lands at (10, 20, 2) in the world. The
	// flattened assembly carries hierarchical leaf names, the tree mirrors the
	// nesting with correct rollup counts, and the flat/tree totals agree.
	let dir = scratch_dir("three_levels");
	write_bottom(&dir);
	std::fs::write(dir.join("deck.lmcpart"), save_part(&box_doc(8.0, 8.0, 1.0), "deck")).expect("write deck");
	std::fs::write(dir.join("plate.lmcpart"), save_part(&box_doc(30.0, 30.0, 1.0), "plate")).expect("write plate");
	let mid = save_assembly(
		"mid",
		&[
			part_at("deck", "deck.lmcpart", Affine3A::IDENTITY),
			sub_at("b", "bottom.lmcasm", Affine3A::from_translation(Vec3::new(10.0, 0.0, 0.0))),
		],
		&[],
	)
	.expect("mid saves");
	std::fs::write(dir.join("mid.lmcasm"), &mid).expect("write mid.lmcasm");
	let top_instances = [
		part_at("plate", "plate.lmcpart", Affine3A::IDENTITY),
		sub_at("m", "mid.lmcasm", Affine3A::from_translation(Vec3::new(0.0, 20.0, 0.0))),
	];
	let top = save_assembly("top", &top_instances, &[]).expect("top saves");
	let top_again = save_assembly("top", &top_instances, &[]).expect("top saves again");

	let loaded = load_assembly(&top, &dir).expect("three-level assembly loads");
	let names: Vec<Option<String>> = loaded.instance_names.clone();
	let cap_center = loaded.assembly.instances[3].pose.transform_point3(Vec3::ZERO);
	let tree = loaded.bom_tree();
	let flat = loaded.bom();
	let flat_total: usize = flat.iter().map(|l| l.count).sum();
	let tree_total: usize = tree.iter().map(|n| n.count).sum();
	let m_node = &tree[1];
	let b_node = m_node.children.iter().find(|c| c.instance == "b");

	assert!(
		top == top_again
			&& top.contains("\"asm_path\": \"mid.lmcasm\"")
			&& mid.contains("\"asm_path\": \"bottom.lmcasm\"")
			&& loaded.assembly.instances.len() == 4
			&& loaded.tree.len() == 2
			&& names
				== vec![
					Some("plate".to_string()),
					Some("m/deck".to_string()),
					Some("m/b/base".to_string()),
					Some("m/b/cap".to_string())
				]
			&& loaded.part_names == vec!["plate", "deck", "base", "cap"]
			&& loaded.residual < 1e-6
			&& (cap_center - Vec3::new(10.0, 20.0, 2.0)).length() < 1e-3
			&& m_node.instance == "m"
			&& m_node.name == "mid"
			&& m_node.count == 3
			&& b_node.is_some_and(|b| b.name == "bottom" && b.count == 2 && b.children.len() == 2)
			&& tree[0].count == 1
			&& flat.len() == 4
			&& flat_total == 4
			&& flat_total == tree_total,
		"3-level nesting: names={names:?} cap={cap_center:?} (want (10,20,2)) residual={} tree={tree:#?} flat={flat:#?}",
		loaded.residual
	);
	let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn parent_mates_place_a_sub_assembly_as_one_rigid_unit() {
	// Pose/mate semantics, precisely: the bottom sub-assembly solves its OWN
	// mates internally (cap exactly +2 z above base in the unit frame), then
	// the PARENT's mates move the unit as one rigid body — its seed pose is
	// deliberately wrong, the parent Coincident+Parallel pulls the whole stack
	// onto the plate's top face. The members' relative pose must survive the
	// move bit-tight, and a post-load `LoadedAssembly::solve_mates` (the
	// nesting-aware re-solve) must hold the same solution.
	let dir = scratch_dir("rigid_unit");
	write_bottom(&dir);
	std::fs::write(dir.join("plate.lmcpart"), save_part(&box_doc(20.0, 20.0, 10.0), "plate")).expect("write plate");
	let instances = [
		part_at("plate", "plate.lmcpart", Affine3A::IDENTITY),
		sub_at("stack", "bottom.lmcasm", Affine3A::from_translation(Vec3::new(7.0, 3.0, 9.0))),
	];
	// Seat the stack's base bottom face (sub-frame (0,0,-1)) on the plate's top
	// face point (0,0,5): the sub frame must land at (0,0,6).
	let mates = [
		Constraint::Coincident { a: 0, a_point: DVec3::new(0.0, 0.0, 5.0), b: 1, b_point: DVec3::new(0.0, 0.0, -1.0) },
		Constraint::Parallel { a: 0, a_dir: DVec3::Z, b: 1, b_dir: DVec3::Z },
	];
	let text = save_assembly("seated", &instances, &mates).expect("parent saves");

	let mut loaded = load_assembly(&text, &dir).expect("parent loads");
	let base_center = loaded.assembly.instances[1].pose.transform_point3(Vec3::ZERO);
	let cap_center = loaded.assembly.instances[2].pose.transform_point3(Vec3::ZERO);
	let relative = cap_center - base_center;
	let re_residual = loaded.solve_mates(256);
	let base_after = loaded.assembly.instances[1].pose.transform_point3(Vec3::ZERO);

	assert!(
		loaded.residual < 1e-6
			&& (base_center - Vec3::new(0.0, 0.0, 6.0)).length() < 1e-3
			&& (relative - Vec3::new(0.0, 0.0, 2.0)).length() < 1e-6
			&& re_residual < 1e-6
			&& (base_after - base_center).length() < 1e-6,
		"rigid-unit mate: base={base_center:?} (want (0,0,6)) cap-base={relative:?} (want (0,0,2)) \
		 residual={} re_residual={re_residual} base_after={base_after:?}",
		loaded.residual
	);
	let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn sub_assembly_include_cycles_fail_loudly() {
	// A-includes-B-includes-A and the degenerate self-include both refuse with
	// FormatError::AsmCycle naming the chain — never an infinite recursion,
	// never a half-loaded assembly. (Two SIBLING instances of the same
	// sub-assembly are legal and proven elsewhere; only re-entering a file
	// that is still being loaded is a cycle.)
	let dir = scratch_dir("cycles");
	std::fs::write(dir.join("part.lmcpart"), save_part(&box_doc(1.0, 1.0, 1.0), "part")).expect("write part");
	let a = save_assembly("a", &[sub_at("child", "b.lmcasm", Affine3A::IDENTITY)], &[]).expect("a saves");
	let b = save_assembly("b", &[sub_at("child", "a.lmcasm", Affine3A::IDENTITY)], &[]).expect("b saves");
	std::fs::write(dir.join("a.lmcasm"), &a).expect("write a");
	std::fs::write(dir.join("b.lmcasm"), &b).expect("write b");
	let c = save_assembly("c", &[sub_at("me", "c.lmcasm", Affine3A::IDENTITY)], &[]).expect("c saves");
	std::fs::write(dir.join("c.lmcasm"), &c).expect("write c");

	let two_file = load_assembly(&a, &dir);
	let self_include = load_assembly(&c, &dir);
	let two_msg = two_file.as_ref().err().map(ToString::to_string).unwrap_or_default();
	let self_msg = self_include.as_ref().err().map(ToString::to_string).unwrap_or_default();

	assert!(
		matches!(two_file, Err(FormatError::AsmCycle { ref path, ref chain }) if path.ends_with("b.lmcasm") && chain.len() == 3)
			&& two_msg.contains("sub-assembly cycle")
			&& two_msg.contains("b.lmcasm")
			&& two_msg.contains("a.lmcasm")
			&& matches!(self_include, Err(FormatError::AsmCycle { ref path, ref chain }) if path.ends_with("c.lmcasm") && chain.len() == 2)
			&& self_msg.contains("sub-assembly cycle")
			&& self_msg.contains("c.lmcasm"),
		"include cycles must fail loudly and specifically:\n A<->B: {two_msg}\n self: {self_msg}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn suppressing_a_sub_assembly_drops_its_entire_branch() {
	// Branch-drop semantics: (a) suppressing the SUB-ASSEMBLY INSTANCE in the
	// parent file removes every leaf under it from geometry, BOM flat and BOM
	// tree (the anchor part alone remains — its exact 32 mm³ is the whole
	// assembly volume); (b) with the sub active, a member suppressed INSIDE
	// the sub's own file stays suppressed in the parent (base survives, cap
	// does not). Mass properties are B-rep exact, so the volumes are sharp.
	let dir = scratch_dir("branch_drop");
	write_bottom(&dir);
	std::fs::write(dir.join("anchor.lmcpart"), save_part(&box_doc(4.0, 4.0, 2.0), "anchor")).expect("write anchor");
	// (a) the parent suppresses the whole sub-assembly instance.
	let mut suppressed_sub = sub_at("stack", "bottom.lmcasm", Affine3A::from_translation(Vec3::new(20.0, 0.0, 0.0)));
	suppressed_sub.suppressed = true;
	let parent_a = save_assembly("drop_a", &[part_at("anchor", "anchor.lmcpart", Affine3A::IDENTITY), suppressed_sub], &[])
		.expect("parent a saves");
	let loaded_a = load_assembly(&parent_a, &dir).expect("parent a loads");
	let vol_a = loaded_a.assembly.mass_properties(Resolution::VoxelSize(0.5)).volume;
	let flat_a = loaded_a.bom();
	let tree_a = loaded_a.bom_tree();

	// (b) the sub is active but ITS OWN file suppresses the cap member.
	let bottom_text = std::fs::read_to_string(dir.join("bottom.lmcasm")).expect("read bottom");
	assert_eq!(bottom_text.matches("\"name\": \"cap\"").count(), 1, "fixture: cap instance must be unique\n{bottom_text}");
	let bottom_capless = bottom_text.replace("\"name\": \"cap\"", "\"name\": \"cap\",\n   \"suppressed\": true");
	std::fs::write(dir.join("bottom_capless.lmcasm"), bottom_capless).expect("write capless bottom");
	let parent_b = save_assembly(
		"drop_b",
		&[
			part_at("anchor", "anchor.lmcpart", Affine3A::IDENTITY),
			sub_at("stack", "bottom_capless.lmcasm", Affine3A::from_translation(Vec3::new(20.0, 0.0, 0.0))),
		],
		&[],
	)
	.expect("parent b saves");
	let loaded_b = load_assembly(&parent_b, &dir).expect("parent b loads");
	let vol_b = loaded_b.assembly.mass_properties(Resolution::VoxelSize(0.5)).volume;
	let flat_b_names: Vec<(String, usize)> = loaded_b.bom().into_iter().map(|l| (l.name, l.count)).collect();

	assert!(
		(vol_a - 32.0).abs() < 1e-6 // anchor only: 4×4×2
			&& flat_a.len() == 1
			&& flat_a[0].name == "anchor"
			&& tree_a.len() == 1
			&& tree_a[0].name == "anchor"
			&& loaded_a.assembly.is_instance_suppressed(1)
			&& loaded_a.assembly.is_instance_suppressed(2)
			&& (vol_b - 64.0).abs() < 1e-6 // anchor 32 + base 32; the cap's 8 is gone
			&& flat_b_names == vec![("anchor".to_string(), 1), ("base".to_string(), 1)],
		"branch drop: vol_a={vol_a} (want 32) flat_a={flat_a:?} tree_a={tree_a:?} vol_b={vol_b} (want 64) flat_b={flat_b_names:?}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn bom_v2_masses_are_density_times_engine_volume_with_honest_source_labels() {
	// BOM v2 enrichment against closed forms: a steel 20×10×5 block (exact
	// B-rep ⇒ volume EXACTLY 1000 mm³ = 1 cm³) must carry unit_mass_g =
	// 7.85 × 1.0 through the same arithmetic the kernel uses, labeled
	// volume_source "exact"; an implicit-only blob (SmoothUnion has no exact
	// B-rep) must be labeled "mesh" with a voxel-accurate mass inside the
	// closed-form bracket [one sphere, two spheres] × density — honest
	// routing, never silently exact. A meta-less part carries NO optional
	// fields. The CSV is golden-line checked (stable column order), and two
	// independent loads serialize byte-identically (determinism).
	let dir = scratch_dir("bom_v2");
	let steel = PartBomMeta {
		part_number: Some("BLK-1".to_string()),
		material: Some(Material { name: "steel".to_string(), density_g_cm3: 7.85 }),
		make_or_buy: Some(MakeOrBuy::Make),
	};
	std::fs::write(dir.join("block.lmcpart"), save_part_with_meta(&box_doc(20.0, 10.0, 5.0), "block", Some(&steel)))
		.expect("write block");
	// Implicit-only: a smooth union of two r=5 spheres 4 apart — evaluate_brep
	// is None by design, so the BOM must take the voxel route.
	let mut blob = Document::new();
	let s0 = blob.add(Feature::Sphere {
		center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
		radius: Dim::Literal(5.0),
	});
	let s1 = blob.add(Feature::Sphere {
		center: [Dim::Literal(4.0), Dim::Literal(0.0), Dim::Literal(0.0)],
		radius: Dim::Literal(5.0),
	});
	let fused = blob.add(Feature::SmoothUnion { a: s0, b: s1, blend: Dim::Literal(1.0) });
	blob.set_root(fused);
	let brass = PartBomMeta {
		part_number: None,
		material: Some(Material { name: "brass".to_string(), density_g_cm3: 8.4 }),
		make_or_buy: Some(MakeOrBuy::Buy),
	};
	std::fs::write(dir.join("blob.lmcpart"), save_part_with_meta(&blob, "blob", Some(&brass))).expect("write blob");
	std::fs::write(dir.join("plain.lmcpart"), save_part(&box_doc(3.0, 3.0, 3.0), "plain")).expect("write plain");

	let at = |x: f32| Affine3A::from_translation(Vec3::new(x, 0.0, 0.0));
	let instances = [
		part_at("b0", "block.lmcpart", at(0.0)),
		part_at("b1", "block.lmcpart", at(30.0)),
		part_at("b2", "block.lmcpart", at(60.0)),
		part_at("organic", "blob.lmcpart", at(100.0)),
		part_at("spare", "plain.lmcpart", at(140.0)),
	];
	let text = save_assembly("massy", &instances, &[]).expect("assembly saves");
	let loaded = load_assembly(&text, &dir).expect("assembly loads");
	let bom = loaded.bom_v2(0.4);
	let bom_again = load_assembly(&text, &dir).expect("assembly re-loads").bom_v2(0.4);

	let block = bom.flat.iter().find(|l| l.name == "block").expect("block line");
	let blob_line = bom.flat.iter().find(|l| l.name == "blob").expect("blob line");
	let plain = bom.flat.iter().find(|l| l.name == "plain").expect("plain line");
	// THE documented formula, mirrored bit-for-bit: mass = density ×
	// engine volume (exact_volume on the exact route) / 1000. The engine volume
	// itself must sit on the closed form 20×10×5 = 1000 mm³ to f64 tetra-fan
	// slack (the fan sums in f64; the last ulp is the only divergence allowed).
	let block_solid = box_doc(20.0, 10.0, 5.0).evaluate_brep().expect("the block is an exact B-rep");
	let engine_vol = kernel_brep::exact_volume(&block_solid);
	let want_unit = 7.85 * engine_vol / 1000.0;
	let want_line = want_unit * 3.0;
	let sphere = 4.0 / 3.0 * std::f64::consts::PI * 125.0;
	let blob_mass = blob_line.unit_mass_g.unwrap_or(f64::NAN);
	let csv = bom.to_csv();
	let csv_lines: Vec<&str> = csv.lines().collect();
	let golden_block = format!("block,3,,BLK-1,steel,7.85,exact,{want_unit},{want_line},make");

	assert!(
		(engine_vol - 1000.0).abs() < 1e-9
			&& block.count == 3
			&& block.part_number.as_deref() == Some("BLK-1")
			&& block.volume_source == Some(VolumeSource::Exact)
			&& block.unit_mass_g == Some(want_unit)
			&& block.line_mass_g == Some(want_line)
			&& block.make_or_buy == Some(MakeOrBuy::Make)
			&& blob_line.volume_source == Some(VolumeSource::Mesh)
			&& blob_line.make_or_buy == Some(MakeOrBuy::Buy)
			// voxel-route mass: bracketed by the closed forms (one sphere ≤
			// blended union ≤ two spheres), NOT asserted exact — that is the
			// honest accuracy class of the mesh route.
			&& blob_mass > 8.4 * sphere / 1000.0
			&& blob_mass < 8.4 * 2.0 * sphere / 1000.0
			&& plain.part_number.is_none()
			&& plain.material.is_none()
			&& plain.unit_mass_g.is_none()
			&& plain.volume_source.is_none()
			&& csv_lines.first() == Some(&"name,count,params,part_number,material,density_g_cm3,volume_source,unit_mass_g,line_mass_g,make_or_buy")
			&& csv_lines.contains(&golden_block.as_str())
			&& csv_lines.len() == 1 + bom.flat.len()
			&& bom.to_json() == bom_again.to_json()
			&& csv == bom_again.to_csv()
			&& bom.to_json().contains("\"schema\": \"bom/2\""),
		"BOM v2 mass/CSV/determinism: block={block:?} blob={blob_line:?} (mass {blob_mass}, sphere bracket [{}, {}]) plain={plain:?}\nCSV:\n{csv}",
		8.4 * sphere / 1000.0,
		8.4 * 2.0 * sphere / 1000.0
	);
	let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn two_sibling_instances_of_the_same_sub_assembly_are_legal_and_counted() {
	// The cycle check must NOT fire for the legitimate reuse case: the same
	// `.lmcasm` placed twice as siblings (each load pushes/pops its own
	// recursion). Both units resolve, names stay distinct, and the BOM groups
	// the leaf parts across both placements (base ×2, cap ×2) while the tree
	// keeps the two placements separate with rollup 2 each.
	let dir = scratch_dir("siblings");
	write_bottom(&dir);
	let parent = save_assembly(
		"twins",
		&[
			sub_at("left", "bottom.lmcasm", Affine3A::IDENTITY),
			sub_at("right", "bottom.lmcasm", Affine3A::from_translation(Vec3::new(15.0, 0.0, 0.0))),
		],
		&[],
	)
	.expect("twins saves");
	let loaded = load_assembly(&parent, &dir).expect("sibling sub-assemblies load");
	let flat: Vec<(String, usize)> = loaded.bom().into_iter().map(|l| (l.name, l.count)).collect();
	let tree = loaded.bom_tree();
	assert!(
		loaded.assembly.instances.len() == 4
			&& loaded.instance_names
				== vec![
					Some("left/base".to_string()),
					Some("left/cap".to_string()),
					Some("right/base".to_string()),
					Some("right/cap".to_string())
				]
			&& flat == vec![("base".to_string(), 2), ("cap".to_string(), 2)]
			&& tree.len() == 2
			&& tree[0].count == 2
			&& tree[1].count == 2,
		"sibling reuse must load and roll up: names={:?} flat={flat:?} tree={tree:#?}",
		loaded.instance_names
	);
	let _ = std::fs::remove_dir_all(&dir);
}
