// Copyright (c) LMCAD. Licensed under the MIT License.

//! Feature-tree coverage (Wave 4, T1 + T3): the new [`Feature`] variants — the
//! hole wizard, circular-rim torus fillets, loft/sweep solids, catalog parts and
//! the standard groove / insert-boss cuts — each (a) round-trip a `.lmcpart` to
//! the **bit-identical** rebuilt volume and (b) re-evaluate after a parameter
//! edit; plus the kernel-owned mesh-routing policy of [`Document::export_mesh`]
//! (T3): exact when the exact path is sound, healed WITH A STATED REASON when it
//! is not — never a silent degrade.

use kernel_brep::{validate, volume};
use kernel_model::format::{load_part, save_part};
use kernel_model::{BooleanOp, CatalogPart, Dim, Document, Feature, HoleFit, HoleKind, MeshRoute};

/// Three literal [`Dim`]s.
fn lit3(x: f64, y: f64, z: f64) -> [Dim; 3] {
	[Dim::Literal(x), Dim::Literal(y), Dim::Literal(z)]
}

/// Save → load → rebuild: two saves must be byte-identical and the loaded
/// document's exact volume must equal the original's to the BIT (geometry is
/// never stored; the recipe re-evaluates deterministically). Returns the loaded
/// document for further parametric edits.
fn round_trip_bits(doc: &Document, name: &str) -> Document {
	let v0 = volume(&doc.evaluate_brep().unwrap_or_else(|| panic!("{name}: fixture evaluates")));
	let saved = save_part(doc, name);
	let saved_again = save_part(doc, name);
	let (loaded, _) = load_part(&saved).unwrap_or_else(|e| panic!("{name}: saved part loads: {e}"));
	let v1 = volume(&loaded.evaluate_brep().unwrap_or_else(|| panic!("{name}: loaded part evaluates")));
	assert!(
		saved == saved_again && v0.to_bits() == v1.to_bits(),
		"{name}: round-trip must be byte-stable and rebuild bit-identically: stable={} v0={v0} ({:#018x}) v1={v1} ({:#018x})",
		saved == saved_again,
		v0.to_bits(),
		v1.to_bits()
	);
	loaded
}

#[test]
fn hole_features_cover_the_wizard_and_stay_parametric() {
	// One 80×40×10 plate, all five wizard kinds chained (each consuming the
	// previous): a parametric blind drill, an M5 close-fit clearance hole, an M5
	// counterbore, an M5 countersink and an M6 blind tap pilot. The rollback API
	// (`evaluate_brep_to`) reads the volume after each stage — every hole must
	// remove material — and the clearance stage must remove EXACTLY its Ø5.3
	// 32-gon prism through the 10 mm plate (exact_volume is analytic). The saved
	// file rebuilds bit-identically and re-drills when the diameter parameter
	// grows. The wizard is B-rep-only: the implicit preview is honestly absent
	// (mirror of ExtrudeSketch), not a silently-undrilled plate.
	let mut doc = Document::new();
	doc.set_param("d", 6.0);
	let down = lit3(0.0, 0.0, -1.0);
	let plate = doc.add(Feature::Box { center: lit3(0.0, 0.0, 5.0), size: lit3(80.0, 40.0, 10.0) });
	let drilled = doc.add(Feature::Hole {
		input: plate,
		kind: HoleKind::Drill,
		m_or_d: Dim::param("d"),
		at: lit3(-30.0, 0.0, 10.0),
		axis: down.clone(),
		fit: None,
		depth: Some(Dim::Literal(6.0)),
	});
	let cleared = doc.add(Feature::Hole {
		input: drilled,
		kind: HoleKind::Clearance,
		m_or_d: Dim::Literal(5.0),
		at: lit3(-15.0, 0.0, 10.0),
		axis: down.clone(),
		fit: Some(HoleFit::Close),
		depth: None,
	});
	let cbored = doc.add(Feature::Hole {
		input: cleared,
		kind: HoleKind::Counterbore,
		m_or_d: Dim::Literal(5.0),
		at: lit3(0.0, 0.0, 10.0),
		axis: down.clone(),
		fit: None,
		depth: None,
	});
	let sunk = doc.add(Feature::Hole {
		input: cbored,
		kind: HoleKind::Countersink,
		m_or_d: Dim::Literal(5.0),
		at: lit3(15.0, 0.0, 10.0),
		axis: down.clone(),
		fit: None,
		depth: None,
	});
	let tapped = doc.add(Feature::Hole {
		input: sunk,
		kind: HoleKind::Tap,
		m_or_d: Dim::Literal(6.0),
		at: lit3(30.0, 0.0, 10.0),
		axis: down.clone(),
		fit: None,
		depth: Some(Dim::Literal(8.0)),
	});
	doc.set_root(tapped);

	let stages: Vec<f64> = [plate, drilled, cleared, cbored, sunk, tapped]
		.iter()
		.map(|&id| volume(&doc.evaluate_brep_to(id).expect("every wizard prefix evaluates")))
		.collect();
	let each_removes = stages.windows(2).all(|w| w[1] < w[0] - 1.0);
	// ISO 273 close fit for M5 is Ø5.3; the tool is a 32-gon prism through 10 mm.
	// Asserted to 1e-6 relative: the boolean's seam bookkeeping leaves sub-µm³
	// residue, six orders below the ~7% spacing of the fit series — so this still
	// pins the exact diameter and fit, honestly.
	let gon32 = 0.5 * 32.0 * (std::f64::consts::TAU / 32.0).sin();
	let clearance_removed = stages[1] - stages[2];
	let clearance_exact = gon32 * 2.65 * 2.65 * 10.0;

	let v = validate(&doc.evaluate_brep().expect("the full wizard chain evaluates"));
	let mut loaded = round_trip_bits(&doc, "wizard plate");
	loaded.set_param("d", 9.0);
	let v_d9 = volume(&loaded.evaluate_brep().expect("the loaded plate re-drills"));

	assert!(
		each_removes
			&& (clearance_removed - clearance_exact).abs() / clearance_exact < 1e-6
			&& v.is_valid()
			&& v_d9 < stages[5]
			&& doc.evaluate().is_none(),
		"wizard chain: stages {stages:?} must strictly decrease; clearance removed {clearance_removed} (exact {clearance_exact}); \
		 validity {v:?}; d 6→9 re-drill {v_d9} < {}; implicit preview honestly absent",
		stages[5]
	);
}

#[test]
fn circular_rim_fillet_feature_rounds_a_boss_and_a_bore_lip() {
	// CONVEX: a Ø12×12 boss with its top rim rolled by a parametric exact-torus
	// fillet — more radius, less material, always below the sharp boss. CONCAVE:
	// a drilled plate's bore exit lip rounded (the ball sheds the 90° corner ring
	// into the bore). Both round-trip bit-identically, and the implicit preview
	// passes the input through (the documented Fillet-style preview gap), so the
	// voxel path still meshes the unrounded boss.
	let mut boss_doc = Document::new();
	boss_doc.set_param("fr", 1.0);
	let boss = boss_doc.add(Feature::Cylinder { center: lit3(0.0, 0.0, 6.0), radius: Dim::Literal(6.0), height: Dim::Literal(12.0) });
	let rounded = boss_doc.add(Feature::CircularRimFillet {
		input: boss,
		near: lit3(6.0, 0.0, 12.0),
		radius: Dim::param("fr"),
		concave: false,
	});
	boss_doc.set_root(rounded);
	let sharp = volume(&boss_doc.evaluate_brep_to(boss).expect("the sharp boss evaluates"));
	let v_fr1 = volume(&boss_doc.evaluate_brep().expect("the rounded boss evaluates"));
	let mut loaded = round_trip_bits(&boss_doc, "rounded boss");
	loaded.set_param("fr", 2.5);
	let v_fr25 = volume(&loaded.evaluate_brep().expect("the loaded boss re-rounds"));

	// The qualifying bored-cap shape for the concave kernel's honest scope: an
	// octagonal boss (sketch-extruded, so the cap is a plain polygon) drilled by
	// the 32-segment Cylinder feature (a cuboid's cap splits in a way the rebuild
	// does not yet tile — kernel scope, rejected loudly as documented).
	let mut oct = kernel_model::Sketch::new();
	let pts: Vec<_> = (0..8)
		.map(|i| {
			let a = i as f64 * std::f64::consts::TAU / 8.0;
			oct.add_point(kernel_core::math::DVec2::new(12.0 * a.cos(), 12.0 * a.sin()))
		})
		.collect();
	for i in 0..8 {
		oct.add_segment(pts[i], pts[(i + 1) % 8]);
	}
	let mut lip_doc = Document::new();
	let boss8 = lip_doc.add(Feature::ExtrudeSketch { sketch: oct, height: Dim::Literal(8.0), dims: vec![], draft: Dim::Literal(0.0) });
	let bore = lip_doc.add(Feature::Cylinder { center: lit3(0.0, 0.0, 4.0), radius: Dim::Literal(5.0), height: Dim::Literal(12.0) });
	let drilled = lip_doc.add(Feature::Boolean { op: BooleanOp::Difference, a: boss8, b: bore });
	let lip = lip_doc.add(Feature::CircularRimFillet {
		input: drilled,
		near: lit3(0.0, 0.0, 9.0),
		radius: Dim::Literal(1.0),
		concave: true,
	});
	lip_doc.set_root(lip);
	let v_drilled = volume(&lip_doc.evaluate_brep_to(drilled).expect("the drilled plate evaluates"));
	let v_lip = volume(&lip_doc.evaluate_brep().expect("the bore lip rounds"));
	round_trip_bits(&lip_doc, "rounded bore lip");

	assert!(
		v_fr25 < v_fr1 && v_fr1 < sharp && v_lip < v_drilled && boss_doc.evaluate().is_some(),
		"rim fillets: convex fr=1 → {v_fr1}, fr=2.5 → {v_fr25} (sharp {sharp}); concave lip {v_lip} < drilled {v_drilled}; \
		 implicit preview passes through"
	);
}

#[test]
fn loft_and_sweep_solid_features_build_exact_parametric_solids() {
	// LOFT: an aligned square frustum (8-square to 4-square over a parametric
	// height) — planar walls, so the exact volume is the closed-form prismatoid
	// h/3·(A₁ + A₂ + √(A₁A₂)), and doubling the height parameter doubles it.
	// SWEEP: a 4-square swept along a straight parametric path — a prism of
	// exactly area × length. Both are B-rep-only (implicit preview honestly
	// absent) and round-trip bit-identically.
	let square = |s: f64, z: Dim| -> Vec<[Dim; 3]> {
		vec![
			[Dim::Literal(s), Dim::Literal(-s), z.clone()],
			[Dim::Literal(s), Dim::Literal(s), z.clone()],
			[Dim::Literal(-s), Dim::Literal(s), z.clone()],
			[Dim::Literal(-s), Dim::Literal(-s), z],
		]
	};
	let mut loft_doc = Document::new();
	loft_doc.set_param("h", 6.0);
	let frustum = loft_doc.add(Feature::LoftSolid {
		sections: vec![square(4.0, Dim::Literal(0.0)), square(2.0, Dim::param("h"))],
	});
	loft_doc.set_root(frustum);
	let v_loft = volume(&loft_doc.evaluate_brep().expect("the loft evaluates"));
	let frustum_exact = 6.0 / 3.0 * (64.0 + 16.0 + (64.0_f64 * 16.0).sqrt());
	let mut loft_loaded = round_trip_bits(&loft_doc, "square frustum");
	loft_loaded.set_param("h", 12.0);
	let v_loft_h12 = volume(&loft_loaded.evaluate_brep().expect("the loaded loft re-skins"));

	let mut sweep_doc = Document::new();
	sweep_doc.set_param("len", 10.0);
	let prism = sweep_doc.add(Feature::SweepSolid {
		profile: square(2.0, Dim::Literal(0.0)),
		path: vec![lit3(0.0, 0.0, 0.0), lit3(0.0, 0.0, 5.0), [Dim::Literal(0.0), Dim::Literal(0.0), Dim::param("len")]],
	});
	sweep_doc.set_root(prism);
	let v_sweep = volume(&sweep_doc.evaluate_brep().expect("the sweep evaluates"));
	let mut sweep_loaded = round_trip_bits(&sweep_doc, "swept prism");
	sweep_loaded.set_param("len", 20.0);
	let v_sweep_l20 = volume(&sweep_loaded.evaluate_brep().expect("the loaded sweep re-sweeps"));

	assert!(
		(v_loft - frustum_exact).abs() < 1e-9
			&& (v_loft_h12 - 2.0 * frustum_exact).abs() < 1e-9
			&& (v_sweep - 160.0).abs() < 1e-9
			&& (v_sweep_l20 - 320.0).abs() < 1e-9
			&& loft_doc.evaluate().is_none()
			&& sweep_doc.evaluate().is_none(),
		"loft/sweep: frustum {v_loft} (want {frustum_exact}), h→12 {v_loft_h12} (want {}); prism {v_sweep} (want 160), \
		 len→20 {v_sweep_l20} (want 320); both B-rep-only on the implicit path",
		2.0 * frustum_exact
	);
}

#[test]
fn catalog_part_features_hold_every_main_part_in_a_lmcpart() {
	// A `.lmcpart` can hold each of the twelve main catalog parts as ONE feature:
	// every variant evaluates to a valid positive-volume B-rep and round-trips
	// byte-stably to the bit-identical volume. The washer's thickness is
	// parameter-driven to prove catalog dimensions resolve from the parameter
	// table: doubling it exactly doubles the (revolved, thickness-linear) volume.
	let part_doc = |part: CatalogPart| {
		let mut doc = Document::new();
		let id = doc.add(Feature::CatalogPart { part });
		doc.set_root(id);
		doc
	};
	let mut washer_doc = part_doc(CatalogPart::Washer {
		outer_d: Dim::Literal(16.0),
		inner_d: Dim::Literal(8.4),
		thickness: Dim::param("t"),
	});
	washer_doc.set_param("t", 1.5);
	let fixtures: Vec<(&str, Document)> = vec![
		(
			"spur gear",
			part_doc(CatalogPart::SpurGear {
				module: Dim::Literal(2.0),
				teeth: 20,
				face_width: Dim::Literal(8.0),
				bore_d: Dim::Literal(8.0),
				pressure_angle_deg: Dim::Literal(20.0),
				keyway: true,
			}),
		),
		(
			"hex bolt",
			part_doc(CatalogPart::HexBolt {
				head_width: Dim::Literal(13.0),
				head_height: Dim::Literal(5.3),
				shank_d: Dim::Literal(8.0),
				shank_len: Dim::Literal(30.0),
			}),
		),
		("hex nut", part_doc(CatalogPart::HexNut { width: Dim::Literal(13.0), height: Dim::Literal(6.5), bore_d: Dim::Literal(8.0) })),
		("washer", washer_doc.clone()),
		("shcs", part_doc(CatalogPart::SocketHeadCapScrew { m: Dim::Literal(5.0), length: Dim::Literal(20.0) })),
		(
			"gt2 pulley",
			part_doc(CatalogPart::Gt2Pulley { teeth: 20, belt_width: Dim::Literal(6.0), bore_d: Dim::Literal(5.0), flanged: true }),
		),
		(
			"sprocket",
			part_doc(CatalogPart::ChainSprocket {
				pitch: Dim::Literal(12.7),
				roller_d: Dim::Literal(7.92),
				teeth: 16,
				bore_d: Dim::Literal(12.0),
			}),
		),
		("shaft", part_doc(CatalogPart::Shaft { d: Dim::Literal(8.0), length: Dim::Literal(40.0) })),
		("o-ring", part_doc(CatalogPart::ORing { dash: 214 })),
		("dowel pin", part_doc(CatalogPart::DowelPin { d: Dim::Literal(6.0), length: Dim::Literal(30.0) })),
		(
			"gear rack",
			part_doc(CatalogPart::GearRack {
				module: Dim::Literal(2.0),
				length: Dim::Literal(60.0),
				width: Dim::Literal(10.0),
				pressure_angle_deg: Dim::Literal(20.0),
			}),
		),
		(
			"internal gear",
			part_doc(CatalogPart::InternalGear {
				module: Dim::Literal(2.0),
				teeth: 36,
				face_width: Dim::Literal(8.0),
				rim_od: Dim::Literal(85.0),
				pressure_angle_deg: Dim::Literal(20.0),
			}),
		),
	];

	let mut failures: Vec<String> = Vec::new();
	for (name, doc) in &fixtures {
		match doc.evaluate_brep() {
			Some(solid) => {
				let v = validate(&solid);
				let vol = volume(&solid).abs();
				if !(v.closed && v.manifold && vol > 1.0) {
					failures.push(format!("{name}: invalid or implausibly small ({v:?}, vol {vol})"));
				} else {
					round_trip_bits(doc, name);
				}
				if doc.evaluate().is_some() {
					failures.push(format!("{name}: catalog parts must be honestly B-rep-only on the implicit path"));
				}
			}
			None => failures.push(format!("{name}: does not evaluate")),
		}
	}
	// The parametric proof: washer volume is exactly linear in its thickness param.
	let v_t15 = volume(&washer_doc.evaluate_brep().expect("washer evaluates"));
	washer_doc.set_param("t", 3.0);
	let v_t30 = volume(&washer_doc.evaluate_brep().expect("washer re-evaluates"));
	if (v_t30 / v_t15 - 2.0).abs() > 1e-9 {
		failures.push(format!("washer: thickness 1.5→3.0 must exactly double the volume ({v_t15} → {v_t30})"));
	}
	assert!(failures.is_empty(), "catalog coverage failures:\n  {}", failures.join("\n  "));
}

#[test]
fn groove_and_boss_features_cut_the_standard_seats_and_suppress_cleanly() {
	// The three standard modifier cuts as features: a DIN 471 circlip groove on a
	// Ø10 shaft (removes a ring), an AS568 -112 O-ring gland on its nominal Ø16.49
	// piston (removes the gland ring), and an M5 heat-set insert boss on a plate
	// (boss minus pocket ADDS material). Each round-trips bit-identically, and —
	// being single-input modifiers — suppressing the feature restores the base
	// volume to the bit (primary-input fallback), the same toggle Fillet has.
	let shaft_doc = {
		let mut doc = Document::new();
		let shaft = doc.add(Feature::Cylinder { center: lit3(0.0, 0.0, 30.0), radius: Dim::Literal(5.0), height: Dim::Literal(60.0) });
		let grooved = doc.add(Feature::CirclipGroove {
			input: shaft,
			at: lit3(0.0, 0.0, 50.0),
			axis: lit3(0.0, 0.0, 1.0),
			d: Dim::Literal(10.0),
			internal: false,
		});
		doc.set_root(grooved);
		doc
	};
	let piston_doc = {
		let mut doc = Document::new();
		// AS568 -112: ID 12.37, gland depth 2.06 ⇒ nominal piston Ø 12.37 + 2·2.06.
		let piston = doc.add(Feature::Cylinder { center: lit3(0.0, 0.0, 15.0), radius: Dim::Literal((12.37 + 2.0 * 2.06) * 0.5), height: Dim::Literal(30.0) });
		let gland = doc.add(Feature::ORingGroove { input: piston, at: lit3(0.0, 0.0, 12.0), axis: lit3(0.0, 0.0, 1.0), dash: 112 });
		doc.set_root(gland);
		doc
	};
	let mut boss_doc = Document::new();
	boss_doc.set_param("m", 5.0);
	let plate = boss_doc.add(Feature::Box { center: lit3(0.0, 0.0, 3.0), size: lit3(30.0, 30.0, 6.0) });
	let bossed = boss_doc.add(Feature::HeatsetBoss { input: plate, at: lit3(8.0, 8.0, 6.0), axis: lit3(0.0, 0.0, 1.0), m: Dim::param("m") });
	boss_doc.set_root(bossed);

	let mut failures: Vec<String> = Vec::new();
	for (name, doc, expect_more) in [("circlip groove", &shaft_doc, false), ("o-ring gland", &piston_doc, false), ("heatset boss", &boss_doc, true)] {
		let mut doc = doc.clone();
		let root = doc.root().expect("fixture has a root");
		let base_id = kernel_model::FeatureId(0);
		let base = volume(&doc.evaluate_brep_to(base_id).expect("base stock evaluates"));
		let modified = volume(&doc.evaluate_brep().expect("modified part evaluates"));
		let grew = modified > base + 1.0;
		let shrank = modified < base - 1.0;
		if expect_more && !grew {
			failures.push(format!("{name}: must ADD material ({base} → {modified})"));
		}
		if !expect_more && !shrank {
			failures.push(format!("{name}: must REMOVE material ({base} → {modified})"));
		}
		round_trip_bits(&doc, name);
		doc.set_suppressed(root, true);
		let suppressed = volume(&doc.evaluate_brep().expect("suppressed modifier falls back to its input"));
		if suppressed.to_bits() != base.to_bits() {
			failures.push(format!("{name}: suppressing must restore the base stock bits ({base} vs {suppressed})"));
		}
	}
	// Parametric: a smaller insert (M4) grows a smaller boss than M5.
	let v_m5 = volume(&boss_doc.evaluate_brep().expect("M5 boss evaluates"));
	boss_doc.set_param("m", 4.0);
	let v_m4 = volume(&boss_doc.evaluate_brep().expect("M4 boss evaluates"));
	if v_m4 >= v_m5 {
		failures.push(format!("heatset boss: M4 must add less than M5 ({v_m4} vs {v_m5})"));
	}
	assert!(failures.is_empty(), "groove/boss coverage failures:\n  {}", failures.join("\n  "));
}

#[test]
fn export_mesh_routes_exact_when_the_exact_path_is_sound() {
	// T3, the happy path: a drilled plate (box − bore) tessellates watertight on
	// the EXACT adaptive path, so the routing report must say Exact, watertight,
	// with the reason stating it — and the mesh is the analytic tessellation
	// (rich, finite). The route decision is the kernel's, not the caller's.
	let mut doc = Document::new();
	let plate = doc.add(Feature::Box { center: lit3(0.0, 0.0, 0.0), size: lit3(20.0, 20.0, 10.0) });
	let bore = doc.add(Feature::Cylinder { center: lit3(0.0, 0.0, 0.0), radius: Dim::Literal(4.0), height: Dim::Literal(12.0) });
	let part = doc.add(Feature::Boolean { op: BooleanOp::Difference, a: plate, b: bore });
	doc.set_root(part);

	let (mesh, report) = doc.export_mesh(0.005);
	assert!(
		report.route == MeshRoute::Exact
			&& report.watertight
			&& mesh.is_watertight()
			&& report.tris == mesh.triangle_count()
			&& report.tris > 1000
			&& report.why.contains("exact"),
		"a sound curved part must route Exact: {report:?}"
	);
}

#[test]
fn export_mesh_heals_self_intersecting_and_voxel_only_documents_with_reasons() {
	// T3, the honest fallbacks. (a) A deliberately SELF-INTERSECTING solid — a
	// square swept along a 2 mm-pitch helix with a 4 mm-tall profile, so adjacent
	// turns interpenetrate: its exact tessellation can be edge-watertight yet
	// geometrically corrupt, so the router must detect the self-intersection,
	// heal through the voxel half, and say why. (b) A voxel-half-only document (a
	// smooth union) has no exact B-rep at all: healed route, its own reason.
	// (c) An empty document: empty mesh, zero triangles, not watertight.
	use std::f64::consts::TAU;
	let turns = 2.5;
	let steps = (turns * 24.0) as usize;
	let path: Vec<[Dim; 3]> = (0..=steps)
		.map(|k| {
			let a = TAU * turns * k as f64 / steps as f64;
			lit3(8.0 * a.cos(), 8.0 * a.sin(), 2.0 * turns * k as f64 / steps as f64)
		})
		.collect();
	let profile = vec![lit3(10.0, 0.0, -2.0), lit3(6.0, 0.0, -2.0), lit3(6.0, 0.0, 2.0), lit3(10.0, 0.0, 2.0)];
	let mut helix_doc = Document::new();
	let coil = helix_doc.add(Feature::SweepSolid { profile, path });
	helix_doc.set_root(coil);
	let solid = helix_doc.evaluate_brep().expect("the helical sweep evaluates");
	assert!(kernel_brep::self_intersects(&solid), "fixture: the overlapping coil must self-intersect");
	let (helix_mesh, helix_report) = helix_doc.export_mesh(0.05);

	let mut blob_doc = Document::new();
	let a = blob_doc.add(Feature::Sphere { center: lit3(0.0, 0.0, 0.0), radius: Dim::Literal(5.0) });
	let b = blob_doc.add(Feature::Sphere { center: lit3(6.0, 0.0, 0.0), radius: Dim::Literal(5.0) });
	let blob = blob_doc.add(Feature::SmoothUnion { a, b, blend: Dim::Literal(2.0) });
	blob_doc.set_root(blob);
	let (blob_mesh, blob_report) = blob_doc.export_mesh(0.05);

	let (empty_mesh, empty_report) = Document::new().export_mesh(0.05);

	assert!(
		helix_report.route == MeshRoute::Healed
			&& helix_report.watertight
			&& helix_mesh.is_watertight()
			&& helix_report.why.contains("self-intersects")
			&& blob_report.route == MeshRoute::Healed
			&& blob_report.watertight
			&& blob_mesh.is_watertight()
			&& blob_report.why.contains("no exact B-rep")
			&& empty_report.tris == 0
			&& !empty_report.watertight
			&& empty_mesh.is_empty(),
		"healed routes must carry their reasons:\n  helix: {helix_report:?}\n  blob: {blob_report:?}\n  empty: {empty_report:?}"
	);
}
