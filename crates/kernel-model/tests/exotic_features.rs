// Copyright (c) LMCAD. Licensed under the MIT License.

//! Exotic parts as hand-editable `.lmcpart` documents (W6): the tri-benchmark's
//! graded-gyroid **damper** and a hybrid ring+lattice **cap** in its spirit,
//! expressed as pure-data feature
//! trees — [`Feature::GyroidLattice`] (+ the declarative [`LinearGrade`] law),
//! [`Feature::BeamLatticeFill`], [`Feature::PipeFeat`] and
//! [`Feature::HybridFuse`] — each saved, reloaded, **string-edited like a user
//! in a text editor**, and rebuilt. Every round-trip asserts byte-stable saves
//! and bit-identical re-evaluation (R5); every voxel-half feature asserts its
//! honest `None` on the exact path (the mirror of `Feature::Shell`).

use kernel_core::mesher::Resolution;
use kernel_model::format::{load_part, save_part};
use kernel_model::{BooleanOp, Dim, Document, Feature, FeatureId, HybridRoute, LatticeCellKind, LinearGrade};

/// Three literal [`Dim`]s.
fn lit3(x: f64, y: f64, z: f64) -> [Dim; 3] {
	[Dim::Literal(x), Dim::Literal(y), Dim::Literal(z)]
}

/// The tri-benchmark damper as a **document**: a field-graded gyroid puck —
/// stiff (thick-walled) bottom, soft top — clipped to a Ø34×20 cylinder. The
/// grade is the closed-form linear law `0.25 − 0.025·z` (clamped ±0.3), stored
/// as data ([`LinearGrade`]) with the rate parameter-bound for test (c).
fn damper_doc() -> Document {
	let mut doc = Document::new();
	doc.set_param("g_rate", -0.025);
	let lattice = doc.add(Feature::GyroidLattice {
		region: [lit3(-18.0, -18.0, -1.0), lit3(18.0, 18.0, 21.0)],
		scale: Dim::Literal(0.55),
		thickness: Dim::Literal(1.3),
		grade: Some(LinearGrade {
			axis: lit3(0.0, 0.0, 1.0),
			per_unit: Dim::param("g_rate"),
			offset: Dim::Literal(0.25),
			max_abs: Dim::Literal(0.3),
		}),
	});
	let puck = doc.add(Feature::Cylinder { center: lit3(0.0, 0.0, 10.0), radius: Dim::Literal(17.0), height: Dim::Literal(20.0) });
	// Implicit ∩ implicit composes through the EXISTING Boolean feature — no
	// dedicated clip variant is needed (Document::build handles Node operands).
	let damper = doc.add(Feature::Boolean { op: BooleanOp::Intersection, a: lattice, b: puck });
	doc.set_root(damper);
	doc
}

/// Damper meshing resolution: coarse enough for the default suite, fine enough
/// that the lattice is rich (>50k triangles) and volumes resolve to ~1%.
const DAMPER_VOXEL: f32 = 0.5;

#[test]
fn graded_gyroid_damper_round_trips_and_a_hand_edit_rescales_predictably() {
	// (a) The damper as a `.lmcpart`: save → load → mesh must be volume-BIT
	// identical (the file is the part; geometry is never stored), and a hand
	// string-edit of the gyroid scale 0.55 → 0.8 must rebuild a predictably
	// different lattice: a higher cell frequency thickens the printable wall set
	// (the |g| < scale·(t + √3·grade) threshold scales with frequency), so the
	// volume grows by ≈ the frequency ratio 0.8/0.55 ≈ 1.45 (measured 1.47) and
	// the finer cells carry more triangles. HONEST meshing note: a TPMS shell
	// has saddle pinches, so the damper mesh is rich and closed but not asserted
	// watertight — the same documented caveat as `Feature::Gyroid`.
	let doc = damper_doc();
	let mesh = doc.mesh(Resolution::VoxelSize(DAMPER_VOXEL));
	let (v0, tris0) = (mesh.signed_volume(), mesh.triangle_count());
	let puck_vol = std::f64::consts::PI * 17.0 * 17.0 * 20.0; // ≈ 18158

	let saved = save_part(&doc, "tri damper");
	let (loaded, _) = load_part(&saved).expect("saved damper loads");
	let v1 = loaded.mesh(Resolution::VoxelSize(DAMPER_VOXEL)).signed_volume();

	assert!(
		saved == save_part(&doc, "tri damper")
			&& v0.to_bits() == v1.to_bits()
			&& tris0 > 50_000
			&& v0 > 0.25 * puck_vol
			&& v0 < 0.70 * puck_vol
			&& doc.evaluate_brep().is_none(),
		"damper round-trip must be byte-stable, volume-bit-identical, rich and honestly voxel-half-only: \
		 v0={v0} ({:#018x}) v1={v1} ({:#018x}) tris={tris0} puck={puck_vol:.0} brep={:?}",
		v0.to_bits(),
		v1.to_bits(),
		doc.evaluate_brep().map(|s| s.face_count())
	);

	// The hand edit, exactly as a user would type it (string surgery, no serde).
	// A `Dim` serializes as `"scale": { "Literal": 0.55 }`; the 0.55 literal
	// occurs exactly once in the file, so the edit is unambiguous.
	assert_eq!(saved.matches("\"Literal\": 0.55").count(), 1, "fixture: the scale literal must be unique\n{saved}");
	let edited = saved.replace("\"Literal\": 0.55", "\"Literal\": 0.8");
	let (fine, _) = load_part(&edited).expect("scale hand-edit loads");
	let fine_mesh = fine.mesh(Resolution::VoxelSize(DAMPER_VOXEL));
	let (vf, trisf) = (fine_mesh.signed_volume(), fine_mesh.triangle_count());
	let ratio = vf / v0;
	assert!(
		trisf > tris0 && ratio > 1.25 && ratio < 1.65,
		"scale 0.55→0.8 must rebuild finer, heavier lattice (volume ×~1.45): \
		 vol {v0:.0} → {vf:.0} (ratio {ratio:.3}, want 1.25..1.65), tris {tris0} → {trisf}"
	);
}

#[test]
fn gyroid_grade_law_is_parameter_driven() {
	// (c) The grading law's rate is a Dim::Param, so set_param re-grades the
	// SAME document: zeroing the rate turns the graded field (+0.25 at z=0
	// fading to −0.25 at z=20, average inflation ≈ 0) into a uniform +0.25
	// inflation — strictly more material everywhere above z=0, measured ≈ +34%.
	let mut doc = damper_doc();
	let v_graded = doc.mesh(Resolution::VoxelSize(DAMPER_VOXEL)).signed_volume();
	doc.set_param("g_rate", 0.0);
	let v_uniform = doc.mesh(Resolution::VoxelSize(DAMPER_VOXEL)).signed_volume();
	assert!(
		v_graded > 0.0 && v_uniform > 1.15 * v_graded && v_uniform < 2.0 * v_graded,
		"zeroing the grade rate must uniformly inflate the walls: graded={v_graded:.0}, rate=0 ⇒ {v_uniform:.0} \
		 (ratio {:.3}, want 1.15..2.0)",
		v_uniform / v_graded
	);
}

/// The hybrid cap as a **document**: an exact B-rep ring (cylinder − bore
/// through the existing Boolean feature) fused with a raw whole-cell cubic
/// beam-lattice insert spanning the bore in ONE [`Feature::HybridFuse`].
///
/// The geometry is **measured to exact-stitch** (parameter scan, 2026-06-10):
/// a RAW `from_cells` block whose strut/ring crossings are all transversal
/// (corner verticals at r≈8.49 embedded in the 5.5–10 annulus, in-plane struts
/// crossing the bore wall radially; nothing tangent to the cylinders or caps),
/// cubic cells because Manifold DC meshes their degree-≤6 junctions as a clean
/// closed 2-manifold at voxel 0.6 (6400 operand triangles, ≈1.4 s/fuse). The
/// honest contrast cases stay asserted elsewhere: octet's degree-12 junctions
/// (and lattices clipped by a cylinder) pinch the operand mesh into the
/// not-a-closed-2-manifold refusal (measured here; the same class hybrid.rs's
/// open-scan test pins), and a dense TPMS operand overruns the
/// `HYBRID_EXACT_MAX_OPERAND_TRIS` rail (the healed-route test below).
fn cap_doc() -> (Document, FeatureId) {
	let mut doc = Document::new();
	let outer = doc.add(Feature::Cylinder { center: lit3(0.0, 0.0, 3.0), radius: Dim::Literal(10.0), height: Dim::Literal(6.0) });
	let bore = doc.add(Feature::Cylinder { center: lit3(0.0, 0.0, 3.0), radius: Dim::Literal(5.5), height: Dim::Literal(8.0) });
	let ring = doc.add(Feature::Boolean { op: BooleanOp::Difference, a: outer, b: bore });
	let lattice = doc.add(Feature::BeamLatticeFill {
		region: [lit3(-6.0, -6.0, 0.0), lit3(6.0, 6.0, 6.0)],
		cell: LatticeCellKind::Cubic,
		cell_size: Dim::Literal(6.0),
		radius: Dim::Literal(1.0),
	});
	let cap = doc.add(Feature::HybridFuse { brep: ring, field: lattice, op: BooleanOp::Union, voxel: Dim::Literal(0.6) });
	doc.set_root(cap);
	(doc, cap)
}

#[test]
fn hybrid_fuse_cap_exact_stitches_and_round_trips() {
	// (b) The cap as a `.lmcpart`. The fuse must take the EXACT-STITCH route:
	// evaluate_brep returns the stitched partial-credit solid (faces > 0, some
	// ring faces verbatim — with their CURVED analytic tags — per the measured
	// report) and feeds it downstream; the fuse mesh is verified watertight; and
	// the whole hybrid document round-trips byte-stably to the volume BIT
	// (hybrid_boolean is deterministic). The cell kind is stored as the
	// hand-editable string "cubic" in the file.
	let (doc, cap) = cap_doc();
	let out = doc.hybrid_fuse_result(cap).expect("root is a HybridFuse").expect("the fuse produces a result");
	let solid = doc.evaluate_brep().expect("exact-stitch route must yield a B-rep for downstream features");
	let v0 = kernel_brep::volume(&solid);

	let saved = save_part(&doc, "tri cap");
	let (loaded, _) = load_part(&saved).expect("saved cap loads");
	let v1 = kernel_brep::volume(&loaded.evaluate_brep().expect("loaded cap re-stitches"));

	assert!(
		out.route == HybridRoute::ExactStitch
			&& out.mesh.is_watertight()
			&& out.mesh.non_manifold_edge_count() == 0
			&& out.solid.as_ref().map(|s| s.face_count()) == Some(solid.face_count())
			&& solid.face_count() > 0
			&& out.report.kept_exact > 0
			&& out.report.kept_exact_curved > 0
			&& saved == save_part(&doc, "tri cap")
			&& saved.contains("\"cell\": \"cubic\"")
			&& v0.to_bits() == v1.to_bits(),
		"hybrid cap must exact-stitch, mesh watertight and round-trip to the volume bit: \
		 route={:?} wt={} nme={} faces={} report={:?} v0={v0} ({:#018x}) v1={v1} ({:#018x})",
		out.route,
		out.mesh.is_watertight(),
		out.mesh.non_manifold_edge_count(),
		solid.face_count(),
		out.report,
		v0.to_bits(),
		v1.to_bits()
	);
}

#[test]
fn hybrid_fuse_healed_route_is_honestly_brep_less_but_still_meshes() {
	// The OTHER half of the HybridFuse contract, on a MEASURED refusal class:
	// this TPMS (gyroid) field operand meshes manifold but DENSE — 121,208
	// triangles at voxel 0.5, beyond the HYBRID_EXACT_MAX_OPERAND_TRIS rail —
	// so the exact stitch is refused with the density reason in bounded time
	// (without the rail the arrangement ground `classify_select` for 15+
	// minutes, measured 2026-06-10; that grind is also what ate the first
	// fixture-tuning scan). Then:
	//  - evaluate_brep returns None (a mesh-only result honestly cannot chain
	//    into downstream exact features),
	//  - hybrid_fuse_result retrieves the route WITH its stated reason plus the
	//    VERIFIED-watertight healed mesh (this is the printable result), and
	//  - export_mesh routes the document through the SDF half and says so.
	// The implicit twin (Document::mesh) stays available too; with a TPMS field
	// it inherits the TPMS closed-but-not-guaranteed-watertight caveat, so the
	// assertion here is non-empty + union-adds-material, not watertightness —
	// the verified watertight mesh is hybrid_fuse_result's.
	let mut doc = Document::new();
	let plate = doc.add(Feature::Box { center: lit3(0.0, 0.0, -1.0), size: lit3(30.0, 30.0, 4.0) });
	let lattice = doc.add(Feature::GyroidLattice {
		region: [lit3(-18.0, -18.0, -1.0), lit3(18.0, 18.0, 21.0)],
		scale: Dim::Literal(0.55),
		thickness: Dim::Literal(1.3),
		grade: None,
	});
	let puck = doc.add(Feature::Cylinder { center: lit3(0.0, 0.0, 10.0), radius: Dim::Literal(17.0), height: Dim::Literal(20.0) });
	let clipped = doc.add(Feature::Boolean { op: BooleanOp::Intersection, a: lattice, b: puck });
	let fuse = doc.add(Feature::HybridFuse { brep: plate, field: clipped, op: BooleanOp::Union, voxel: Dim::Literal(0.5) });
	doc.set_root(fuse);

	let out = doc.hybrid_fuse_result(fuse).expect("root is a HybridFuse").expect("healed route still yields a result");
	let twin = doc.mesh(Resolution::VoxelSize(0.5));
	let (export, report) = doc.export_mesh(0.02);
	assert!(
		matches!(&out.route, HybridRoute::Healed { reason } if reason.contains("too dense for the exact arrangement"))
			&& out.report.operand_triangles > kernel_model::HYBRID_EXACT_MAX_OPERAND_TRIS
			&& out.solid.is_none()
			&& doc.evaluate_brep().is_none()
			&& out.mesh.is_watertight()
			&& out.mesh.non_manifold_edge_count() == 0
			&& out.report.kept_exact == 0
			&& twin.signed_volume() > 30.0 * 30.0 * 4.0
			&& !export.is_empty()
			&& report.route == kernel_model::MeshRoute::Healed,
		"a healed fuse must be brep-less, retrievable with its reason, and watertightly meshable via the heal: \
		 route={:?} solid={:?} brep={:?} fuse_wt={} twin_vol={:.0} (plate 3600) export_route={:?} why={:?}",
		out.route,
		out.solid.as_ref().map(|s| s.face_count()),
		doc.evaluate_brep().map(|s| s.face_count()),
		out.mesh.is_watertight(),
		twin.signed_volume(),
		report.route,
		report.why
	);
}

#[test]
fn pipe_feature_round_trips_and_meshes_a_tapered_tube() {
	// A PipeFeat (the conformal-channel / tubing primitive as a document
	// feature): a 3-point bent polyline tapering 2.0 → 1.0 mm. It must mesh
	// watertight (a smooth tube has no strut junctions), round-trip to the
	// volume bit, be honestly absent on the exact half, and fail LOUDLY (whole
	// document → None / empty mesh) when a radius is hand-edited non-positive.
	let mut doc = Document::new();
	doc.set_param("r0", 2.0);
	let pipe = doc.add(Feature::PipeFeat {
		path: vec![lit3(0.0, 0.0, 0.0), lit3(10.0, 0.0, 0.0), lit3(10.0, 8.0, 4.0)],
		radii: vec![Dim::param("r0"), Dim::Literal(1.5), Dim::Literal(1.0)],
	});
	doc.set_root(pipe);

	let mesh = doc.mesh(Resolution::VoxelSize(0.2));
	let v0 = mesh.signed_volume();
	let saved = save_part(&doc, "bent pipe");
	let (loaded, _) = load_part(&saved).expect("saved pipe loads");
	let v1 = loaded.mesh(Resolution::VoxelSize(0.2)).signed_volume();
	// Naive frustum+caps yardstick (kernel_implicit::Pipe::volume_estimate's
	// closed form): π/3·Σ L(ra²+ra·rb+rb²) + the two end half-spheres.
	let l2 = (8.0_f64 * 8.0 + 4.0 * 4.0).sqrt();
	let est = std::f64::consts::PI / 3.0 * (10.0 * (4.0 + 3.0 + 2.25) + l2 * (2.25 + 1.5 + 1.0))
		+ 2.0 / 3.0 * std::f64::consts::PI * (8.0 + 1.0);
	assert!(
		mesh.is_watertight() && v0.to_bits() == v1.to_bits() && (v0 - est).abs() / est < 0.08 && doc.evaluate_brep().is_none(),
		"pipe must mesh watertight, round-trip to the bit, match the tube estimate and stay voxel-half-only: \
		 wt={} v0={v0:.1} v1={v1:.1} est={est:.1} brep={:?}",
		mesh.is_watertight(),
		doc.evaluate_brep().map(|s| s.face_count())
	);

	// Hand-corrupt a radius to 0: Pipe::new's contract is enforced as a loud
	// failure-to-evaluate, never a panic and never a silently degenerate tube.
	doc.set_param("r0", 0.0);
	assert!(
		doc.evaluate().is_none() && doc.mesh(Resolution::VoxelSize(0.5)).is_empty(),
		"a non-positive pipe radius must fail the whole evaluation loudly"
	);
}

#[test]
fn beam_lattice_fill_guards_fail_loud_and_old_documents_still_load() {
	// (d, plus guards) A BeamLatticeFill with a zero cell size (the hand-edit
	// typo class) must fail to evaluate loudly — None, not a panicking
	// from_cells, and not an OOM (the cell-count rail). And a pre-W6 `.lmcpart`
	// payload — written before the exotic variants existed — must still load
	// and rebuild: new variants only EXTEND the schema (serde back-compat).
	let mut doc = Document::new();
	doc.set_param("cell", 0.0);
	let lat = doc.add(Feature::BeamLatticeFill {
		region: [lit3(0.0, 0.0, 0.0), lit3(20.0, 20.0, 20.0)],
		cell: LatticeCellKind::Octet,
		cell_size: Dim::param("cell"),
		radius: Dim::Literal(1.0),
	});
	doc.set_root(lat);
	let zero_fails = doc.evaluate().is_none();
	doc.set_param("cell", 0.001); // 8e9 cells: beyond the documented memory rail
	let rail_fails = doc.evaluate().is_none();
	doc.set_param("cell", 10.0);
	let sane_builds = doc.evaluate().is_some();
	assert!(
		zero_fails && rail_fails && sane_builds && doc.evaluate_brep().is_none()
			&& save_part(&doc, "octet guards").contains("\"cell\": \"octet\""),
		"lattice-fill guards: cell=0 fails={zero_fails}, 1µm rail fails={rail_fails}, cell=10 builds={sane_builds}, \
		 brep honestly None, cell kind saved as the string \"octet\""
	);

	// A document JSON exactly as an older (pre-exotic-variants) kernel wrote it.
	let old = r#"{
  "params": { "h": 5.0 },
  "features": [
    { "Box": { "center": [{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": 0.0}],
               "size": [{"Literal": 4.0}, {"Literal": 2.0}, {"Param": "h"}] } }
  ],
  "root": 0,
  "suppressed": []
}"#;
	let loaded = Document::load_json(old).expect("a pre-W6 document still loads (back-compat)");
	let v = kernel_brep::volume(&loaded.evaluate_brep().expect("old document evaluates"));
	assert!((v - 40.0).abs() < 1e-9, "old-schema box must rebuild exactly: vol={v} (want 40)");

	// The forward half of grade back-compat: an UNGRADED GyroidLattice must
	// serialize with NO "grade" key at all (serde skip), so its files are
	// byte-identical to what a pre-grading kernel would write — and a
	// hand-written minimal lattice without the key loads as grade=None.
	let mut g = Document::new();
	let lat2 = g.add(Feature::GyroidLattice {
		region: [lit3(0.0, 0.0, 0.0), lit3(10.0, 10.0, 10.0)],
		scale: Dim::Literal(0.8),
		thickness: Dim::Literal(0.5),
		grade: None,
	});
	g.set_root(lat2);
	let saved_g = save_part(&g, "ungraded");
	let (g2, _) = load_part(&saved_g).expect("ungraded gyroid lattice loads");
	assert!(
		!saved_g.contains("\"grade\"") && g2.evaluate().is_some(),
		"an ungraded GyroidLattice must omit the grade key and reload evaluable:\n{saved_g}"
	);

	// And the real pre-W6 artifacts in the repo: every `.lmcpart` of the W5
	// dogfood corpus (written before the exotic variants existed — see
	// tests/fixtures/pre_w6_parts/README.md; these files must never be
	// regenerated) must still parse through `load_part`, and a known-cheap one
	// must still rebuild a solid.
	let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/pre_w6_parts");
	let mut parts: Vec<String> = std::fs::read_dir(&dir)
		.expect("tests/fixtures/pre_w6_parts exists")
		.filter_map(|e| e.ok().map(|e| e.path()))
		.filter(|p| p.extension().is_some_and(|x| x == "lmcpart"))
		.map(|p| p.display().to_string())
		.collect();
	parts.sort();
	let mut spacer_rebuilds = false;
	for p in &parts {
		let json = std::fs::read_to_string(p).expect("fixture readable");
		let (doc, meta) = load_part(&json).unwrap_or_else(|e| panic!("pre-W6 fixture {p} must still load: {e:?}"));
		if p.ends_with("spacer_10.lmcpart") {
			let vol = kernel_brep::volume(&doc.evaluate_brep().expect("spacer rebuilds a B-rep"));
			spacer_rebuilds = vol > 0.0 && meta.name == "spacer_8x12_10";
		}
	}
	assert!(
		parts.len() == 20 && spacer_rebuilds,
		"all 20 pre-W6 fixtures must load and the spacer must rebuild: found {} ({:?}…), spacer_ok={spacer_rebuilds}",
		parts.len(),
		parts.first()
	);
}

/// [`Feature::Tpms`] — the six-family Document-tree twin of the op surface's
/// `tpms` op. Every family rebuilds a CLOSED (region-clamped) sheet block that
/// meshes watertight; the feature is honestly voxel-half-only (`None` on the
/// exact B-rep path, the mirror of `Feature::Shell`); a save is byte-stable
/// across a load round-trip with bit-identical re-meshed volume (R5); and the
/// two loud-`None` guards hold (sheet without a positive level, non-positive
/// cell). One assert, full per-family report.
#[test]
fn tpms_feature_six_families_watertight_bytestable_and_loud_guards() {
	use kernel_model::TpmsFamily;
	let families = [
		TpmsFamily::Gyroid,
		TpmsFamily::SchwarzP,
		TpmsFamily::Diamond,
		TpmsFamily::Neovius,
		TpmsFamily::SchoenIwp,
		TpmsFamily::FischerKochS,
	];
	let build = |kind: TpmsFamily, sheet: bool, level: Option<f64>, cell: f64| {
		let mut doc = Document::new();
		let lat = doc.add(Feature::Tpms {
			region: [lit3(-10.0, -10.0, -10.0), lit3(10.0, 10.0, 10.0)],
			kind,
			cell: Dim::Literal(cell),
			sheet,
			level: level.map(Dim::Literal),
		});
		doc.set_root(lat);
		doc
	};
	let mut report = String::new();
	let mut families_ok = true;
	for kind in families {
		let doc = build(kind, true, Some(0.8), 8.0);
		let mesh = doc.mesh(Resolution::VoxelSize(0.5));
		let wt = mesh.is_watertight() && mesh.triangle_count() > 0;
		let brep_none = doc.evaluate_brep().is_none();
		families_ok &= wt && brep_none;
		report += &format!("\n  {kind:?}: watertight={wt} tris={} brep_none={brep_none}", mesh.triangle_count());
	}
	// Byte-stable save/load + bit-identical rebuild (R5) on one family.
	let doc = build(TpmsFamily::FischerKochS, true, Some(0.8), 8.0);
	let saved = save_part(&doc, "tpms block");
	let (loaded, _) = load_part(&saved).expect("saved tpms block loads");
	let byte_stable = saved == save_part(&loaded, "tpms block");
	let v0 = doc.mesh(Resolution::VoxelSize(0.5)).signed_volume();
	let v1 = loaded.mesh(Resolution::VoxelSize(0.5)).signed_volume();
	let rebuild_identical = v0.to_bits() == v1.to_bits();
	// Loud-None guards: sheet without level / with a zero level / zero cell.
	let guards_ok = build(TpmsFamily::Gyroid, true, None, 8.0).evaluate().is_none()
		&& build(TpmsFamily::Gyroid, true, Some(0.0), 8.0).evaluate().is_none()
		&& build(TpmsFamily::Gyroid, false, Some(0.0), 0.0).evaluate().is_none();
	report += &format!("\n  byte_stable={byte_stable} rebuild_bit_identical={rebuild_identical} (v={v0}) loud_guards={guards_ok}");
	assert!(
		families_ok && byte_stable && rebuild_identical && guards_ok,
		"Feature::Tpms must mesh all six families watertight as closed sheet blocks, stay honest on the exact path, save byte-stably, rebuild bit-identically, and fail loud on bad inputs:{report}"
	);
}
