// Copyright (c) LMCAD. Licensed under the MIT License.

//! Native file formats (BAR.md, I3b + I5): `.lmcpart` / `.lmcasm` round-trips,
//! loud failure modes, byte-stable saves, and the I5 hand-edit proof — a saved
//! part edited as a USER would edit it (plain string surgery on the JSON text:
//! a parameter value, a suppression flag, a label) reloads and rebuilds exactly
//! as intended. The same file is the medium whether a human or an AI made the
//! last edit.

use std::path::PathBuf;

use kernel_brep::volume;
use kernel_core::math::{Affine3A, DVec3, Vec3};
use kernel_core::mesher::Resolution;
use kernel_model::format::{
	load_assembly, load_part, save_assembly, save_part, save_part_with_meta, AsmInstance, AsmSource, FormatError, MakeOrBuy, Material,
	PartBomMeta,
};
use kernel_model::{BooleanOp, Constraint, Dim, Document, Feature, FeatureId};

/// A unique per-test scratch directory under the system temp dir.
fn scratch_dir(name: &str) -> PathBuf {
	let dir = std::env::temp_dir().join(format!("lmcad_formats_{name}_{}", std::process::id()));
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

/// The part fixture: a parametric plate (thickness driven by `"h"`) with a
/// through-bore, the plate labelled and annotated — params, two primitives, a
/// boolean and the I5 metadata in one document.
fn plate_doc() -> Document {
	let mut doc = Document::new();
	doc.set_param("h", 6.0);
	let plate = doc.add(Feature::Box {
		center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
		size: [Dim::Literal(20.0), Dim::Literal(12.0), Dim::param("h")],
	});
	let bore = doc.add(Feature::Cylinder {
		center: [Dim::Literal(4.0), Dim::Literal(0.0), Dim::Literal(0.0)],
		radius: Dim::Literal(2.0),
		height: Dim::Literal(20.0),
	});
	let drilled = doc.add(Feature::Boolean { op: BooleanOp::Difference, a: plate, b: bore });
	doc.set_root(drilled);
	doc.set_label(plate, "plate");
	doc.set_notes(plate, "thickness driven by h");
	doc
}

#[test]
fn lmcpart_round_trip_is_byte_stable_and_rebuilds_bit_identically() {
	// THE I3b part contract: the envelope is self-describing, saving twice gives
	// IDENTICAL bytes (git-diffable designs), and the loaded document rebuilds to
	// the bit-identical solid with its labels/metadata intact.
	let doc = plate_doc();
	let v0 = volume(&doc.evaluate_brep().expect("fixture evaluates"));

	let saved = save_part(&doc, "drilled plate");
	let saved_again = save_part(&doc, "drilled plate");
	let (loaded, meta) = load_part(&saved).expect("saved part loads");
	let v1 = volume(&loaded.evaluate_brep().expect("loaded part evaluates"));

	assert!(
		saved == saved_again
			&& saved.contains("\"format\": \"lmc-part\"")
			&& saved.contains("\"version\": 1")
			&& saved.contains("\"units\": \"mm\"")
			&& meta
				== kernel_model::format::PartMeta {
					name: "drilled plate".to_string(),
					units: "mm".to_string(),
					created_with: format!("lmcad-kernel {}", env!("CARGO_PKG_VERSION")),
					meta: None,
				} && v0.to_bits() == v1.to_bits()
			&& loaded.label(FeatureId(0)) == Some("plate")
			&& loaded.notes(FeatureId(0)) == Some("thickness driven by h"),
		"part round-trip: byte-stable={} meta={meta:?} vol {v0} ({:#018x}) vs {v1} ({:#018x}) label={:?}",
		saved == saved_again,
		v0.to_bits(),
		v1.to_bits(),
		loaded.label(FeatureId(0))
	);
}

#[test]
fn lmcpart_meta_block_round_trips_and_meta_less_saves_stay_v1_bytes() {
	// BOM v2's `meta` block is strictly additive. Proven here: (a) `save_part`
	// (no meta) writes NO "meta" key and byte-equals `save_part_with_meta(…,
	// None)`, so every pre-v2 file and byte-stability contract is untouched;
	// (b) a pre-v2 envelope (the meta-less text) loads with `meta: None`;
	// (c) a full meta block — part number, material+density, make-or-buy —
	// round-trips field-for-field and saves byte-stably.
	let doc = plate_doc();
	let plain = save_part(&doc, "p");
	let plain_via_meta_api = save_part_with_meta(&doc, "p", None);
	let stamped = PartBomMeta {
		part_number: Some("LM-0042".to_string()),
		material: Some(Material { name: "steel".to_string(), density_g_cm3: 7.85 }),
		make_or_buy: Some(MakeOrBuy::Make),
	};
	let with_meta = save_part_with_meta(&doc, "p", Some(&stamped));
	let with_meta_again = save_part_with_meta(&doc, "p", Some(&stamped));
	let (_, header_plain) = load_part(&plain).expect("meta-less part loads");
	let (_, header_meta) = load_part(&with_meta).expect("meta-stamped part loads");
	assert!(
		plain == plain_via_meta_api
			&& !plain.contains("\"meta\"")
			&& header_plain.meta.is_none()
			&& with_meta == with_meta_again
			&& with_meta.contains("\"part_number\": \"LM-0042\"")
			&& with_meta.contains("\"density_g_cm3\": 7.85")
			&& with_meta.contains("\"make_or_buy\": \"make\"")
			&& header_meta.meta.as_ref() == Some(&stamped),
		"meta must be additive and byte-stable: plain_identical={} stamped_identical={} loaded_meta={:?}\n{with_meta}",
		plain == plain_via_meta_api,
		with_meta == with_meta_again,
		header_meta.meta
	);
}

#[test]
fn lmcpart_load_rejects_garbage_wrong_format_version_and_units() {
	// Every contract violation has its own loud FormatError — nothing half-loads
	// and nothing is guessed. The wrong-format case feeds a genuine `.lmcasm`
	// envelope to the part loader (the realistic mix-up).
	let saved = save_part(&plate_doc(), "p");
	let asm_json = save_assembly("a", &[], &[]).expect("empty assembly saves");
	let garbage = load_part("{ not json");
	let not_a_part = load_part(&asm_json);
	let no_format = load_part("{}");
	let future = load_part(&saved.replace("\"version\": 1", "\"version\": 99"));
	let inches = load_part(&saved.replace("\"units\": \"mm\"", "\"units\": \"in\""));
	assert!(
		matches!(garbage, Err(FormatError::Parse(_)))
			&& matches!(not_a_part, Err(FormatError::WrongFormat { expected: "lmc-part", found: Some(ref f) }) if f == "lmc-asm")
			&& matches!(no_format, Err(FormatError::WrongFormat { found: None, .. }))
			&& matches!(future, Err(FormatError::UnsupportedVersion { found: Some(99), supported: 1 }))
			&& matches!(inches, Err(FormatError::UnsupportedUnits { ref found }) if found == "in"),
		"loud rejects required: garbage={garbage:?} not_a_part={not_a_part:?} no_format={no_format:?} future={future:?} inches={inches:?}",
	);
}

#[test]
fn lmcasm_round_trips_path_and_inline_sources_and_resolves_a_face_mate() {
	// THE I3b assembly contract: one instance referenced BY PATH (resolved
	// relative to base_dir), one EMBEDDED inline, a face mate (coincident +
	// parallel) between them. Save → load from a scratch dir → the mates are
	// re-solved on load (residual ~0), re-solving again stays converged, the cap
	// instance lands seated on the base (centre at z = 2), and the assembly
	// meshes non-empty at roughly the two boxes' combined volume.
	let dir = scratch_dir("asm_roundtrip");
	std::fs::write(dir.join("base.lmcpart"), save_part(&box_doc(4.0, 4.0, 2.0), "base")).expect("write base part");

	let instances = [
		AsmInstance {
			name: Some("base".to_string()),
			source: AsmSource::Path("base.lmcpart".to_string()),
			pose: Affine3A::IDENTITY,
			suppressed: false,
		},
		AsmInstance {
			name: Some("cap".to_string()),
			source: AsmSource::Part { name: "cap".to_string(), document: box_doc(2.0, 2.0, 2.0), meta: None },
			pose: Affine3A::from_translation(Vec3::new(5.0, 4.0, 9.0)),
			suppressed: false,
		},
	];
	// Seat the cap's bottom face (local z = −1) on the base's top face (z = +1).
	let mates = [
		Constraint::Coincident { a: 0, a_point: DVec3::new(0.0, 0.0, 1.0), b: 1, b_point: DVec3::new(0.0, 0.0, -1.0) },
		Constraint::Parallel { a: 0, a_dir: DVec3::Z, b: 1, b_dir: DVec3::Z },
	];

	let json = save_assembly("clamp", &instances, &mates).expect("assembly saves");
	let json_again = save_assembly("clamp", &instances, &mates).expect("assembly saves again");
	let mut loaded = load_assembly(&json, &dir).expect("assembly loads");
	let re_residual = loaded.assembly.solve_mates(&loaded.mates, 256);
	let cap_center = loaded.assembly.instances[1].pose.transform_point3(Vec3::ZERO);
	let mesh = loaded.assembly.mesh_all(Resolution::VoxelSize(0.25));
	let vol = mesh.signed_volume();

	assert!(
		json == json_again
			&& json.contains("\"format\": \"lmc-asm\"")
			&& loaded.name == "clamp"
			&& loaded.units == "mm"
			&& loaded.instance_names == vec![Some("base".to_string()), Some("cap".to_string())]
			&& loaded.mates.len() == 2
			&& loaded.residual < 1e-6
			&& re_residual < 1e-6
			&& (cap_center - Vec3::new(0.0, 0.0, 2.0)).length() < 1e-3
			&& mesh.triangle_count() > 0
			&& (vol - 40.0).abs() / 40.0 < 0.15,
		"assembly round-trip: byte-stable={} residual={} re_residual={re_residual} cap_center={cap_center:?} tris={} vol={vol} (want ~40)\n{json}",
		json == json_again,
		loaded.residual,
		mesh.triangle_count()
	);
	let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn lmcasm_failures_are_loud_missing_file_bad_referenced_part_and_scaled_pose() {
	let dir = scratch_dir("asm_failures");
	let path_instance =
		|file: &str| AsmInstance { name: None, source: AsmSource::Path(file.to_string()), pose: Affine3A::IDENTITY, suppressed: false };

	// (a) A path source that does not exist on disk.
	let missing_json = save_assembly("a", &[path_instance("nowhere.lmcpart")], &[]).expect("saves");
	let missing = load_assembly(&missing_json, &dir);

	// (b) A path source whose file exists but is a future-version part.
	let future_part = save_part(&box_doc(1.0, 1.0, 1.0), "future").replace("\"version\": 1", "\"version\": 7");
	std::fs::write(dir.join("future.lmcpart"), future_part).expect("write future part");
	let future_json = save_assembly("a", &[path_instance("future.lmcpart")], &[]).expect("saves");
	let future = load_assembly(&future_json, &dir);

	// (c) A part envelope fed to the assembly loader.
	let mixed_up = load_assembly(&save_part(&box_doc(1.0, 1.0, 1.0), "p"), &dir);

	// (d) A scaled pose cannot be represented by the rigid v1 format — saving
	// must refuse rather than silently drop the scale.
	let scaled = save_assembly(
		"a",
		&[AsmInstance {
			name: None,
			source: AsmSource::Part { name: "p".to_string(), document: box_doc(1.0, 1.0, 1.0), meta: None },
			pose: Affine3A::from_scale(Vec3::splat(2.0)),
			suppressed: false,
		}],
		&[],
	);

	assert!(
		matches!(missing, Err(FormatError::Io { ref path, .. }) if path.ends_with("nowhere.lmcpart"))
			&& matches!(future, Err(FormatError::PartSource { ref path, ref error })
				if path.ends_with("future.lmcpart") && matches!(**error, FormatError::UnsupportedVersion { found: Some(7), .. }))
			&& matches!(mixed_up, Err(FormatError::WrongFormat { expected: "lmc-asm", found: Some(ref f) }) if f == "lmc-part")
			&& matches!(scaled, Err(FormatError::BadPose { instance: 0 })),
		"loud assembly failures required: missing={:?} future={:?} mixed_up={:?} scaled={:?}",
		missing.as_ref().err(),
		future.as_ref().err(),
		mixed_up.as_ref().err(),
		scaled.as_ref().err()
	);
	let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn hand_edited_lmcpart_rebuilds_as_the_user_intended() {
	// THE I5 proof: take the SAVED `.lmcpart` TEXT and edit it exactly as a user
	// would in a text editor — plain string surgery, no serde, no kernel calls —
	// then reload and rebuild:
	//   1. change a parameter value ("h": 5.0 → 9.0)   ⇒ volume scales 80 → 144
	//   2. add a "label" next to the Box feature        ⇒ reads back via label()
	//   3. flip a "suppressed" entry ([] → [1])         ⇒ the pattern drops out,
	//                                                     volume 144 → 72
	// Volumes are exact (planar solids, disjoint pattern copies), so the edits
	// are verified by closed-form prediction, not by tolerance hand-waving.
	let mut doc = Document::new();
	doc.set_param("h", 5.0);
	let block = doc.add(Feature::Box {
		center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
		size: [Dim::Literal(4.0), Dim::Literal(2.0), Dim::param("h")],
	});
	let pattern =
		doc.add(Feature::LinearPattern { input: block, count: 2, step: [Dim::Literal(30.0), Dim::Literal(0.0), Dim::Literal(0.0)] });
	doc.set_root(pattern);

	let saved = save_part(&doc, "patterned block");
	assert_eq!(saved, save_part(&doc, "patterned block"), "two saves of the same doc must be byte-identical");

	let (original, _) = load_part(&saved).expect("unedited file loads");
	let v_original = volume(&original.evaluate_brep().expect("unedited file evaluates"));

	// Edit 1 + 2: the user retypes the parameter value and writes a label next to
	// the feature. Both target strings occur exactly once in the saved text.
	assert_eq!(saved.matches("\"h\": 5.0").count(), 1, "fixture: the param line must be unique\n{saved}");
	assert_eq!(saved.matches("\"Box\": {").count(), 1, "fixture: the Box feature must be unique\n{saved}");
	let edited = saved.replace("\"h\": 5.0", "\"h\": 9.0").replace("\"Box\": {", "\"label\": \"the block\", \"Box\": {");
	let (taller, _) = load_part(&edited).expect("param+label hand-edit loads");
	let v_taller = volume(&taller.evaluate_brep().expect("param-edited file evaluates"));

	// Edit 3: the user suppresses the pattern feature (id 1) in the same file.
	assert_eq!(edited.matches("\"suppressed\": []").count(), 1, "fixture: the suppression list must be unique\n{edited}");
	let suppressed = edited.replace("\"suppressed\": []", "\"suppressed\": [1]");
	let (single, _) = load_part(&suppressed).expect("suppression hand-edit loads");
	let v_single = volume(&single.evaluate_brep().expect("suppression-edited file evaluates"));

	assert!(
		(v_original - 80.0).abs() < 1e-9 // 2 copies × (4 × 2 × 5)
			&& (v_taller - 144.0).abs() < 1e-9 // 2 copies × (4 × 2 × 9): the param edit took effect
			&& taller.label(block) == Some("the block") // the hand-added label reads back
			&& !single.is_suppressed(block)
			&& single.is_suppressed(pattern) // the hand-flipped suppression applied …
			&& (v_single - 72.0).abs() < 1e-9, // … and the pattern dropped out: 4 × 2 × 9
		"hand-edited file must rebuild as intended: vol original={v_original} (want 80), \
		 param-edited={v_taller} (want 144), suppressed={v_single} (want 72), label={:?}",
		taller.label(block)
	);
}
