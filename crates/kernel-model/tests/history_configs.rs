// Copyright (c) LMCAD. Licensed under the MIT License.

//! History & configurations (Wave 4, T2): feature-tree **rollback**
//! (`evaluate_to` / `evaluate_brep_to`), **insert into the history**
//! (`insert_feature_at` with full id remapping), named **configurations**
//! (parameter-override sets persisted in `.lmcpart`, hand-editable I5-style),
//! the bounded **undo/redo** snapshot stack, and on the assembly side:
//! per-instance suppression + named **states** persisted in `.lmcasm`, and the
//! **BOM** with JSON export. Every restoration is asserted to the volume BIT —
//! the rebuild is deterministic, so nothing weaker is honest.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use kernel_brep::volume;
use kernel_core::math::{Affine3A, Vec3};
use kernel_core::mesher::Resolution;
use kernel_model::format::{
	load_assembly, load_part, save_assembly_with_states, save_part, AsmInstance, AsmSource, BomLine, FormatError,
};
use kernel_model::{AsmState, BooleanOp, Dim, Document, DocumentHistory, Feature, FeatureId};

/// Three literal [`Dim`]s.
fn lit3(x: f64, y: f64, z: f64) -> [Dim; 3] {
	[Dim::Literal(x), Dim::Literal(y), Dim::Literal(z)]
}

/// A unique per-test scratch directory under the system temp dir.
fn scratch_dir(name: &str) -> PathBuf {
	let dir = std::env::temp_dir().join(format!("lmcad_history_{name}_{}", std::process::id()));
	std::fs::create_dir_all(&dir).expect("create scratch dir");
	dir
}

/// The shared part fixture: a 20×10×`h` plate drilled by a Ø6 bore — parameters,
/// two primitives and a boolean.
fn plate_doc() -> Document {
	let mut doc = Document::new();
	doc.set_param("h", 5.0);
	let plate = doc.add(Feature::Box { center: lit3(0.0, 0.0, 0.0), size: [Dim::Literal(20.0), Dim::Literal(10.0), Dim::param("h")] });
	let bore = doc.add(Feature::Cylinder { center: lit3(5.0, 0.0, 0.0), radius: Dim::Literal(3.0), height: Dim::Literal(40.0) });
	let drilled = doc.add(Feature::Boolean { op: BooleanOp::Difference, a: plate, b: bore });
	doc.set_root(drilled);
	doc
}

#[test]
fn rollback_evaluation_matches_a_prefix_only_rebuild_to_the_bit() {
	// The rollback bar: evaluating TO an earlier feature must equal a document
	// that never had the later features — on the exact half to the volume BIT,
	// and on the implicit half to the meshed-volume bit (the rebuild is
	// deterministic, R5). The pinned root is ignored by rollback.
	let doc = plate_doc();
	let v_plate_rollback = volume(&doc.evaluate_brep_to(FeatureId(0)).expect("the plate prefix evaluates"));
	let v_full = volume(&doc.evaluate_brep().expect("the full document evaluates"));

	let mut plate_only = Document::new();
	plate_only.set_param("h", 5.0);
	let p = plate_only.add(Feature::Box { center: lit3(0.0, 0.0, 0.0), size: [Dim::Literal(20.0), Dim::Literal(10.0), Dim::param("h")] });
	plate_only.set_root(p);
	let v_plate_fresh = volume(&plate_only.evaluate_brep().expect("the plate-only document evaluates"));

	let mesh_of = |node: kernel_implicit::ops::Node| {
		kernel_implicit::manifold_dual_contour(&node, kernel_core::sdf::Sdf::bounds(&node), Resolution::VoxelSize(0.5)).signed_volume()
	};
	let m_rollback = mesh_of(doc.evaluate_to(FeatureId(0)).expect("the implicit plate prefix evaluates"));
	let m_fresh = plate_only.mesh(Resolution::VoxelSize(0.5)).signed_volume();

	assert!(
		v_plate_rollback.to_bits() == v_plate_fresh.to_bits()
			&& v_full < v_plate_rollback - 1.0
			&& m_rollback.to_bits() == m_fresh.to_bits(),
		"rollback must equal the prefix-only rebuild: brep {v_plate_rollback} vs {v_plate_fresh}, full {v_full}, \
		 implicit {m_rollback} vs {m_fresh}"
	);
}

#[test]
fn insert_feature_at_remaps_ids_root_suppression_and_keeps_geometry() {
	// Insert a small far-away cube at position 1 of the plate fixture. Every
	// later FeatureId must shift: the boolean's operands, the pinned root and the
	// suppression set all remap, the bore's label travels with its record, and
	// the rebuilt volume is BIT-identical (the new feature is not yet
	// referenced). Then a union with the inserted cube adds exactly its 8 mm³.
	let mut doc = plate_doc();
	doc.set_label(FeatureId(1), "bore");
	let xform = doc.add(Feature::Transform { input: FeatureId(2), xform: Affine3A::from_translation(Vec3::new(1.0, 0.0, 0.0)) });
	doc.set_root(xform);
	doc.set_suppressed(xform, true); // suppressed modifier ⇒ evaluates to the plain difference
	let v_before = volume(&doc.evaluate_brep().expect("fixture evaluates"));

	let cube = doc.insert_feature_at(1, Feature::Box { center: lit3(40.0, 0.0, 0.0), size: lit3(2.0, 2.0, 2.0) });
	let v_after = volume(&doc.evaluate_brep().expect("the remapped document evaluates"));

	let union = doc.add(Feature::Boolean { op: BooleanOp::Union, a: FeatureId(4), b: cube });
	doc.set_root(union);
	let v_union = volume(&doc.evaluate_brep().expect("the union with the inserted cube evaluates"));

	assert!(
		cube == FeatureId(1)
			&& v_after.to_bits() == v_before.to_bits()
			&& doc.label(FeatureId(2)) == Some("bore")
			&& doc.is_suppressed(FeatureId(4))
			&& !doc.is_suppressed(FeatureId(1))
			// Relative: the curved plate ∪ far cube boolean leaves ~1e-8 mm³ of seam
			// residue (the same sub-µm³ bookkeeping as the wizard chain), far below
			// any geometric meaning.
			&& (v_union - (v_before + 8.0)).abs() / v_union < 1e-9,
		"insert must remap ids/root/suppression and keep geometry: v {v_before} → {v_after}, label(2)={:?}, \
		 suppressed(4)={}, union {v_union} (want {})",
		doc.label(FeatureId(2)),
		doc.is_suppressed(FeatureId(4)),
		v_before + 8.0
	);
}

#[test]
fn configurations_switch_the_volume_predictably_and_round_trip() {
	// Named configurations: "thick" and "thin" override the plate's height
	// parameter. Activating one changes the rebuilt volume by EXACTLY the
	// closed-form plate scaling (the bore pierces through in all variants);
	// the active configuration and the override sets persist in the `.lmcpart`
	// (byte-stably) and the loaded document switches variants the same way.
	// A typo'd activation is refused and changes nothing. Documents without
	// configurations serialize without the keys (back-compat bytes).
	let mut doc = plate_doc();
	let v_base = volume(&doc.evaluate_brep().expect("base evaluates"));
	let bare = save_part(&doc, "plate");
	doc.add_config("thick", [("h".to_string(), 10.0)]);
	doc.add_config("thin", [("h".to_string(), 2.5)]);

	let activated = doc.activate_config("thick");
	let v_thick = volume(&doc.evaluate_brep().expect("thick variant evaluates"));
	// Plate 20×10×h minus a through-bore: V(h) scales linearly in h.
	let v_thick_expect = v_base * 2.0;

	let saved = save_part(&doc, "plate");
	let saved_again = save_part(&doc, "plate");
	let (mut loaded, _) = load_part(&saved).expect("config-carrying part loads");
	let v_loaded = volume(&loaded.evaluate_brep().expect("loaded thick variant evaluates"));
	let switched = loaded.activate_config("thin");
	let v_thin = volume(&loaded.evaluate_brep().expect("thin variant evaluates"));
	loaded.deactivate_config();
	let v_back = volume(&loaded.evaluate_brep().expect("deactivated variant evaluates"));
	let typo = loaded.activate_config("thicc");

	assert!(
		activated
			&& switched
			&& !typo
			&& loaded.active_config().is_none()
			&& (v_thick - v_thick_expect).abs() < 1e-9
			&& (v_thin - v_base * 0.5).abs() < 1e-9
			&& v_loaded.to_bits() == v_thick.to_bits()
			&& v_back.to_bits() == v_base.to_bits()
			&& doc.effective_param("h") == Some(10.0)
			&& doc.param("h") == Some(5.0)
			&& saved == saved_again
			&& saved.contains("\"active_config\": \"thick\"")
			&& !bare.contains("configs")
			&& !bare.contains("active_config"),
		"configurations: base {v_base}, thick {v_thick} (want {v_thick_expect}), thin {v_thin} (want {}), \
		 loaded {v_loaded}, back {v_back}; activated={activated} switched={switched} typo={typo}",
		v_base * 0.5
	);
}

#[test]
fn hand_edited_active_config_in_the_saved_text_activates() {
	// The I5-style proof for configurations: a user opens the saved `.lmcpart`
	// in a text editor and switches the variant by typing the active_config line
	// (plain string surgery — no serde, no kernel calls). The reloaded document
	// must rebuild the override variant bit-identically to the programmatic
	// activation.
	let mut doc = plate_doc();
	doc.add_config("thick", [("h".to_string(), 10.0)]);
	let saved = save_part(&doc, "plate");
	assert!(!saved.contains("active_config"), "fixture: no active configuration in the saved text\n{saved}");
	assert_eq!(saved.matches("\"root\":").count(), 1, "fixture: the root line must be unique\n{saved}");

	let edited = saved.replace("\"root\":", "\"active_config\": \"thick\",\n    \"root\":");
	let (hand, _) = load_part(&edited).expect("the hand-edited part loads");
	let v_hand = volume(&hand.evaluate_brep().expect("the hand-edited part evaluates"));

	doc.activate_config("thick");
	let v_api = volume(&doc.evaluate_brep().expect("the programmatic variant evaluates"));

	assert!(
		hand.active_config() == Some("thick") && v_hand.to_bits() == v_api.to_bits(),
		"a hand-typed active_config must rebuild the variant exactly: {v_hand} vs {v_api} (active {:?})",
		hand.active_config()
	);
}

#[test]
fn document_history_undo_redo_restores_volume_bits_and_is_bounded() {
	// The session undo stack: three states (base plate → taller → drilled wider),
	// then undo×2 / redo / branch-discarding push, each restoration checked by
	// rebuilding the CURRENT document and comparing volume BITS. A capacity-2
	// history drops the oldest state (it becomes unreachable), keeping the rest.
	let vol_of = |doc: &Document| volume(&doc.evaluate_brep().expect("snapshot evaluates"));
	let mut doc = plate_doc();
	let v0 = vol_of(&doc);
	let mut history = DocumentHistory::new(doc.clone(), 8);

	doc.set_param("h", 9.0);
	history.push(doc.clone());
	let v1 = vol_of(history.current());

	doc.set_param("bore_r", 4.0); // future-proof: unused param, geometry-inert
	doc.set_param("h", 12.0);
	history.push(doc.clone());
	let v2 = vol_of(history.current());

	let u1 = history.undo().map(vol_of);
	let u0 = history.undo().map(vol_of);
	let bottom = history.undo().is_none();
	let r1 = history.redo().map(vol_of);
	// Branch discard: pushing after an undo forgets the v2 state.
	doc.set_param("h", 6.0);
	history.push(doc.clone());
	let v3 = vol_of(history.current());
	let no_redo = history.redo().is_none();

	let mut bounded = DocumentHistory::new(plate_doc(), 2);
	let mut b = plate_doc();
	b.set_param("h", 7.0);
	let v_b7 = vol_of(&b);
	bounded.push(b.clone());
	b.set_param("h", 8.0);
	bounded.push(b.clone()); // exceeds capacity 2 → the h=5 base drops off
	let b_undo = bounded.undo().map(vol_of);
	let b_bottom = bounded.undo().is_none();

	// Restorations are compared by f64 equality, i.e. BIT equality of the
	// re-evaluated volumes: an undone snapshot rebuilds exactly its old solid.
	// (The h-linearity sanity is relative — the boolean's volume is not
	// closed-form linear to the last bit.)
	let linear = |v: f64, h: f64| (v - v0 * h / 5.0).abs() / v < 1e-9;
	assert!(
		linear(v1, 9.0)
			&& linear(v2, 12.0)
			&& u1 == Some(v1)
			&& u0 == Some(v0)
			&& bottom
			&& r1 == Some(v1)
			&& linear(v3, 6.0)
			&& v3 < v1
			&& no_redo
			&& b_undo == Some(v_b7)
			&& b_bottom,
		"undo/redo must restore volume bits and stay bounded: v0={v0} v1={v1} v2={v2} u1={u1:?} u0={u0:?} \
		 bottom={bottom} r1={r1:?} v3={v3} no_redo={no_redo} b_undo={b_undo:?} (want {v_b7}) b_bottom={b_bottom}"
	);
}

/// A 2×2×2 cube document (the "screw" stand-in part: tiny, exact volume 8).
fn cube_doc() -> Document {
	let mut doc = Document::new();
	let b = doc.add(Feature::Box { center: lit3(0.0, 0.0, 1.0), size: lit3(2.0, 2.0, 2.0) });
	doc.set_root(b);
	doc
}

/// A 6×6×2 plate document with its thickness as a named parameter (volume 72).
fn base_doc() -> Document {
	let mut doc = Document::new();
	doc.set_param("t", 2.0);
	let b = doc.add(Feature::Box { center: lit3(0.0, 0.0, 0.0), size: [Dim::Literal(6.0), Dim::Literal(6.0), Dim::param("t")] });
	doc.set_root(b);
	doc
}

/// The shared `.lmcasm` fixture: a plate inline plus two screws referenced by
/// path (one suppressed in `suppress_screw2`), and two named states.
fn save_asm_fixture(dir: &Path, suppress_screw2: bool) -> String {
	std::fs::write(dir.join("screw.lmcpart"), save_part(&cube_doc(), "M5 screw")).expect("write screw part");
	let at = |x: f32, y: f32| Affine3A::from_translation(Vec3::new(x, y, 1.0));
	let instances = [
		AsmInstance { name: Some("plate".to_string()), source: AsmSource::Part { name: "plate".to_string(), document: base_doc(), meta: None }, pose: Affine3A::IDENTITY, suppressed: false },
		AsmInstance { name: Some("screw front".to_string()), source: AsmSource::Path("screw.lmcpart".to_string()), pose: at(10.0, 0.0), suppressed: false },
		AsmInstance { name: Some("screw back".to_string()), source: AsmSource::Path("screw.lmcpart".to_string()), pose: at(-10.0, 0.0), suppressed: suppress_screw2 },
	];
	// Two states: "assembled" (both screws seated) and "service" (back screw
	// lifted 8 mm and the front screw suppressed).
	let states: BTreeMap<String, AsmState> = [
		(
			"assembled".to_string(),
			AsmState { poses: vec![Affine3A::IDENTITY, at(10.0, 0.0), at(-10.0, 0.0)], suppressed: vec![] },
		),
		(
			"service".to_string(),
			AsmState {
				poses: vec![Affine3A::IDENTITY, at(10.0, 0.0), Affine3A::from_translation(Vec3::new(-10.0, 0.0, 9.0))],
				suppressed: vec![1],
			},
		),
	]
	.into();
	save_assembly_with_states("clamp", &instances, &[], &states).expect("the assembly with states saves")
}

#[test]
fn lmcasm_per_instance_suppression_and_named_states_round_trip() {
	// The assembly-side T2 contract: the per-instance suppressed flag persists
	// (a suppressed screw contributes no mass/geometry on load), the two named
	// states persist byte-stably, applying a state moves poses AND swaps the
	// suppression set (with exact mass arithmetic: plate 72 + screw 8 each), and
	// a state that does not fit the assembly is refused loudly on save and load.
	let dir = scratch_dir("states");
	let json = save_asm_fixture(&dir, true);
	let json_again = save_asm_fixture(&dir, true);
	let mut loaded = load_assembly(&json, &dir).expect("the assembly loads");
	let res = Resolution::VoxelSize(0.25);
	let mass = |asm: &kernel_model::Assembly| asm.mass_properties(res).volume;

	let loaded_suppression = (loaded.assembly.is_instance_suppressed(1), loaded.assembly.is_instance_suppressed(2));
	let m_loaded = mass(&loaded.assembly); // plate + screw front (screw back suppressed in the file)
	let applied_service = loaded.assembly.apply_state(&loaded.states["service"]);
	let service_suppression = (loaded.assembly.is_instance_suppressed(1), loaded.assembly.is_instance_suppressed(2));
	let m_service = mass(&loaded.assembly); // plate + screw back (front suppressed)
	let back_pose = loaded.assembly.instances[2].pose.transform_point3(Vec3::ZERO);
	let applied_assembled = loaded.assembly.apply_state(&loaded.states["assembled"]);
	let m_assembled = mass(&loaded.assembly); // everything

	// A state that cannot fit (wrong pose count) must be refused on save…
	let bad_save = save_assembly_with_states(
		"clamp",
		&[AsmInstance { name: None, source: AsmSource::Part { name: "p".to_string(), document: cube_doc(), meta: None }, pose: Affine3A::IDENTITY, suppressed: false }],
		&[],
		&[("broken".to_string(), AsmState { poses: vec![], suppressed: vec![] })].into(),
	);
	// … and a hand-broken suppressed index must be refused on load.
	let state_suppression = "\"suppressed\": [\n        1\n      ]";
	assert_eq!(json.matches(state_suppression).count(), 1, "fixture: the state suppression list is unique\n{json}");
	let broken = json.replace(state_suppression, "\"suppressed\": [\n        9\n      ]");
	let bad_load = load_assembly(&broken, &dir);

	assert!(
		json == json_again
			&& json.contains("\"states\"")
			&& json.contains("\"suppressed\": true")
			&& loaded_suppression == (false, true)
			&& (m_loaded - 80.0).abs() < 1e-6
			&& applied_service
			&& loaded.states.len() == 2
			&& (m_service - 80.0).abs() < 1e-6
			&& service_suppression == (true, false)
			&& (back_pose - Vec3::new(-10.0, 0.0, 9.0)).length() < 1e-6
			&& applied_assembled
			&& (m_assembled - 88.0).abs() < 1e-6
			&& matches!(bad_save, Err(FormatError::BadState { ref state, .. }) if state == "broken")
			&& matches!(bad_load, Err(FormatError::BadState { ref state, .. }) if state == "service"),
		"asm states: byte-stable={} loaded={m_loaded} (want 80) service={m_service} (want 80, back at {back_pose:?}) \
		 assembled={m_assembled} (want 88); bad_save={bad_save:?} bad_load={:?}",
		json == json_again,
		bad_load.err()
	);
	let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn bom_counts_a_three_instance_assembly_and_exports_json() {
	// The BOM: two path-sourced screws of the same part plus one inline plate
	// group into two lines — `2× M5 screw` and `1× plate (t=2)` — sorted, with
	// the parameter summary separating same-named parts built to different
	// dimensions. The JSON export carries exactly the same lines. Suppressing a
	// screw (live toggle) drops its count to 1: a suppressed component is absent
	// material, consistent with mass_properties.
	let dir = scratch_dir("bom");
	let json = save_asm_fixture(&dir, false);
	let mut loaded = load_assembly(&json, &dir).expect("the assembly loads");

	let bom = loaded.bom();
	let bom_json = loaded.bom_json();
	loaded.assembly.set_instance_suppressed(1, true);
	let bom_suppressed = loaded.bom();

	assert_eq!(
		(bom.clone(), bom_suppressed, bom_json.contains("\"count\": 2"), serde_json::from_str::<serde_json::Value>(&bom_json).is_ok()),
		(
			vec![
				BomLine { name: "M5 screw".to_string(), count: 2, params: String::new(), ..BomLine::default() },
				BomLine { name: "plate".to_string(), count: 1, params: "t=2".to_string(), ..BomLine::default() },
			],
			vec![
				BomLine { name: "M5 screw".to_string(), count: 1, params: String::new(), ..BomLine::default() },
				BomLine { name: "plate".to_string(), count: 1, params: "t=2".to_string(), ..BomLine::default() },
			],
			true,
			true,
		),
		"the BOM must group, count, summarize and export: {bom:?}\n{bom_json}"
	);
	let _ = std::fs::remove_dir_all(&dir);
}
