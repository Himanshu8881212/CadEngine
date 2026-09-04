// Copyright (c) LMCAD. Licensed under the MIT License.

//! Document persistence (BAR.md, I3): a parametric model saves/loads as JSON and
//! **re-evaluates bit-identically** — the design file an AI session can resume.
//!
//! The fixture deliberately spans the feature vocabulary a real session uses:
//! named parameters, `Box`, `Cylinder`, a `Boolean` difference, a `Fillet`
//! referenced by persistent edge name WITH a `near` witness point, a sketch-driven
//! `ExtrudeSketch` whose width dimension is parameter-driven, and a
//! `LinearPattern` — so the round-trip covers `Dim`s, `EdgeName`s (a foreign
//! type bridged in `persist`), `Sketch`/`SketchConstraint` and the suppression /
//! root bookkeeping in one document.

use kernel_brep::{validate, volume, EdgeName, FaceName, FaceSource};
use kernel_core::math::DVec2;
use kernel_model::{BooleanOp, Dim, Document, Feature, Sketch, SketchConstraint};

/// A fully-constrained 4 × 2 rectangle anchored at the origin; returns the index
/// of its width `Distance` constraint for parametric driving.
fn rectangle_sketch() -> (Sketch, usize) {
	let mut s = Sketch::new();
	let p0 = s.add_point(DVec2::new(0.1, -0.2));
	let p1 = s.add_point(DVec2::new(3.0, 0.05));
	let p2 = s.add_point(DVec2::new(2.9, 1.8));
	let p3 = s.add_point(DVec2::new(-0.1, 2.1));
	s.add_segment(p0, p1);
	s.add_segment(p1, p2);
	s.add_segment(p2, p3);
	s.add_segment(p3, p0);
	s.add_constraint(SketchConstraint::Fixed { point: p0, at: DVec2::ZERO });
	s.add_constraint(SketchConstraint::Horizontal { a: p0, b: p1 });
	s.add_constraint(SketchConstraint::Horizontal { a: p3, b: p2 });
	s.add_constraint(SketchConstraint::Vertical { a: p0, b: p3 });
	s.add_constraint(SketchConstraint::Vertical { a: p1, b: p2 });
	let width = s.add_constraint(SketchConstraint::Distance { a: p0, b: p1, distance: 4.0 });
	s.add_constraint(SketchConstraint::Distance { a: p0, b: p3, distance: 2.0 });
	(s, width)
}

/// The session fixture: a corner-filleted, drilled plate with a sketch-extruded
/// boss, patterned 3× — every dimension that matters driven from a parameter.
fn session_document() -> Document {
	let lit3 = |x: f64, y: f64, z: f64| [Dim::Literal(x), Dim::Literal(y), Dim::Literal(z)];
	let mut doc = Document::new();
	doc.set_param("s", 20.0); // plate side
	doc.set_param("r", 3.0); // bore radius
	doc.set_param("h", 5.0); // boss extrusion height
	doc.set_param("w", 4.0); // boss profile width (a sketch dimension)

	let plate = doc.add(Feature::Box { center: lit3(0.0, 0.0, 0.0), size: [Dim::param("s"), Dim::param("s"), Dim::Literal(6.0)] });
	// Round the +X∧+Y vertical edge by persistent name, with a `near` witness at
	// that corner (exercises the witness-point disambiguation form).
	let edge = EdgeName::new(
		FaceName { operand: FaceSource::Primitive, source_face: 5 },
		FaceName { operand: FaceSource::Primitive, source_face: 3 },
	);
	let filleted = doc.add(Feature::Fillet {
		input: plate,
		edge,
		radius: Dim::Literal(2.0),
		near: Some([Dim::Literal(10.0), Dim::Literal(10.0), Dim::Literal(0.0)]),
	});
	// Parametric through-bore, off-centre so it clears the boss and the fillet.
	let bore = doc.add(Feature::Cylinder { center: lit3(-5.0, -5.0, 0.0), radius: Dim::param("r"), height: Dim::Literal(8.0) });
	let drilled = doc.add(Feature::Boolean { op: BooleanOp::Difference, a: filleted, b: bore });
	// Sketch-driven boss (profile width driven by "w", height by "h"), unioned on.
	let (sketch, width) = rectangle_sketch();
	let boss =
		doc.add(Feature::ExtrudeSketch { sketch, height: Dim::param("h"), dims: vec![(width, Dim::param("w"))], draft: Dim::Literal(0.0) });
	let part = doc.add(Feature::Boolean { op: BooleanOp::Union, a: drilled, b: boss });
	// Three disjoint copies along +x.
	let pattern =
		doc.add(Feature::LinearPattern { input: part, count: 3, step: [Dim::Literal(30.0), Dim::Literal(0.0), Dim::Literal(0.0)] });
	doc.set_root(pattern);
	doc
}

#[test]
fn document_round_trips_and_re_evaluates_bit_identically() {
	// THE I3 contract: save → load → rebuild must reproduce the exact same solid
	// (volume BITS, not "approximately equal"), and a parameter edit on the LOADED
	// document must rebuild exactly like the same edit on the original — i.e. the
	// file really is the resumable session, not a lossy snapshot.
	let doc = session_document();
	let s0 = doc.evaluate_brep().expect("the session document evaluates");
	let v0 = volume(&s0);
	let val0 = validate(&s0);

	let json = doc.save_json();
	let loaded = Document::load_json(&json).expect("saved JSON loads");
	let s1 = loaded.evaluate_brep().expect("the loaded document evaluates");
	let v1 = volume(&s1);

	// Resume the session: drive the boss height up on BOTH documents and compare.
	let mut doc_edit = doc.clone();
	let mut loaded_edit = loaded;
	doc_edit.set_param("h", 9.0);
	loaded_edit.set_param("h", 9.0);
	let ve_doc = volume(&doc_edit.evaluate_brep().expect("edited original evaluates"));
	let ve_loaded = volume(&loaded_edit.evaluate_brep().expect("edited loaded doc evaluates"));

	assert!(
		val0.closed
			&& val0.manifold
			&& val0.shells == 3
			&& v0.to_bits() == v1.to_bits()
			&& ve_doc.to_bits() == ve_loaded.to_bits()
			&& ve_doc > v0,
		"JSON round-trip must re-evaluate bit-identically and stay parametric:\n  \
		 validity {val0:?}\n  vol {v0} ({:#018x}) vs loaded {v1} ({:#018x})\n  \
		 edited vol {ve_doc} ({:#018x}) vs edited loaded {ve_loaded} ({:#018x})",
		v0.to_bits(),
		v1.to_bits(),
		ve_doc.to_bits(),
		ve_loaded.to_bits()
	);
}

#[test]
fn malformed_or_mismatched_json_errors_instead_of_panicking() {
	// Loading never panics and never half-loads: broken syntax, a JSON value of
	// the wrong shape, and an unknown feature VARIANT (the documented loud-failure
	// mode for files from a newer kernel) all return Err.
	let cases = [
		"{ definitely not json",
		"",
		"{\"params\": 7}",
		"[1, 2, 3]",
		// Right shape, but a feature kind this kernel version does not know.
		"{\"params\": {}, \"features\": [{\"Frobnicate\": {}}], \"root\": null, \"suppressed\": []}",
	];
	for json in cases {
		assert!(Document::load_json(json).is_err(), "must reject without panicking: {json:?}");
	}
}

#[test]
fn feature_labels_and_notes_survive_the_round_trip_without_changing_geometry() {
	// The I5 metadata channel: a label + notes attached to features must come back
	// from save/load verbatim, appear in the file NEXT to the feature they describe
	// (`{"Box": {...}, "label": ...}` — the hand-editable spot), leave unlabelled
	// features serialized exactly as before (no metadata keys), and never affect
	// the rebuilt geometry.
	let mut doc = session_document();
	let plate = kernel_model::FeatureId(0);
	let bore = kernel_model::FeatureId(2);
	let v_before = volume(&doc.evaluate_brep().expect("fixture evaluates"));
	doc.set_label(plate, "base plate");
	doc.set_notes(plate, "datum A; keep 6 mm for the M4 inserts");
	doc.set_label(bore, "dowel bore");
	let json = doc.save_json();
	let loaded = Document::load_json(&json).expect("labelled document loads");
	let v_after = volume(&loaded.evaluate_brep().expect("labelled document evaluates"));
	assert!(
		loaded.label(plate) == Some("base plate")
			&& loaded.notes(plate) == Some("datum A; keep 6 mm for the M4 inserts")
			&& loaded.label(bore) == Some("dowel bore")
			&& loaded.notes(bore).is_none()
			&& loaded.label(kernel_model::FeatureId(1)).is_none()
			&& json.contains("\"label\": \"base plate\"")
			&& !json.contains("\"notes\": null")
			&& v_before.to_bits() == v_after.to_bits(),
		"labels/notes must round-trip verbatim and be geometry-inert:\n  label(plate)={:?} notes(plate)={:?} label(bore)={:?}\n  vol {v_before} vs {v_after}\n  json:\n{json}",
		loaded.label(plate),
		loaded.notes(plate),
		loaded.label(bore)
	);
}

#[test]
fn save_json_is_deterministic_and_matches_the_writer_helper() {
	// Saving the same document twice yields identical bytes (parameter map and
	// suppression set are written sorted), and the to_writer helper emits exactly
	// the same bytes as save_json — so files are diff-able across sessions.
	let mut doc = session_document();
	doc.set_suppressed(kernel_model::FeatureId(1), true); // exercise the suppressed set in the file
	let a = doc.save_json();
	let b = doc.save_json();
	let mut via_writer = Vec::new();
	doc.save_json_writer(&mut via_writer).expect("writing to a Vec cannot fail");
	let reloaded = Document::load_json_reader(via_writer.as_slice()).expect("reader helper loads");
	assert!(
		a == b && a.as_bytes() == via_writer.as_slice() && reloaded.is_suppressed(kernel_model::FeatureId(1)),
		"deterministic bytes and equivalent writer/reader helpers required"
	);
}
