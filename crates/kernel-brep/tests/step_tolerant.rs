// Copyright (c) LMCAD. Licensed under the MIT License.

//! The tolerant STEP import and the three vendor-file root causes it was built
//! against (Framework Laptop 12 mainboard / battery / Expansion Card, friction
//! F1 of `campaign/friction/l12_mini_case.md`):
//!
//! (a) a trim vertex slightly off its B-spline patch — snapped within the
//!     file's asserted uncertainty instead of refused;
//! (b) inner (hole) loops on a curved analytic face — tessellated on the exact
//!     surface through the parameter-patch path;
//! (c) periodic sphere/torus regions the ring-grid resampler cannot phase —
//!     read through the general periodic parameterisation.
//!
//! Plus the receipt contract: every solid listed with its product name and
//! placed envelope, skips and repairs verbatim, the compound body valid.

use kernel_brep::math::DVec3;
use kernel_brep::{
	cuboid, cylinder, export_step, import_step, import_step_tolerant, step_census, union, validate, volume, SolidStatus, StepError,
	VertexId,
};

/// A hand-written STEP fragment: a bilinear (degree 1×1) B-spline square
/// `[0,10]²` at `z = 0` trimmed by four line edges whose vertices sit at
/// `z = off`, with the file asserting `uncertainty` mm.
fn bspline_square_with_lifted_trim(off: f64, uncertainty: f64) -> String {
	format!(
		"\
#1=CARTESIAN_POINT('',(0.,0.,0.));\n\
#2=CARTESIAN_POINT('',(0.,10.,0.));\n\
#3=CARTESIAN_POINT('',(10.,0.,0.));\n\
#4=CARTESIAN_POINT('',(10.,10.,0.));\n\
#5=B_SPLINE_SURFACE_WITH_KNOTS('',1,1,((#1,#2),(#3,#4)),.UNSPECIFIED.,.F.,.F.,.F.,(2,2),(2,2),(0.,1.),(0.,1.),.UNSPECIFIED.);\n\
#10=CARTESIAN_POINT('',(0.,0.,{off}));\n\
#11=VERTEX_POINT('',#10);\n\
#12=CARTESIAN_POINT('',(10.,0.,{off}));\n\
#13=VERTEX_POINT('',#12);\n\
#14=CARTESIAN_POINT('',(10.,10.,{off}));\n\
#15=VERTEX_POINT('',#14);\n\
#16=CARTESIAN_POINT('',(0.,10.,{off}));\n\
#17=VERTEX_POINT('',#16);\n\
#20=EDGE_CURVE('',#11,#13,$,.T.);\n\
#21=EDGE_CURVE('',#13,#15,$,.T.);\n\
#22=EDGE_CURVE('',#15,#17,$,.T.);\n\
#23=EDGE_CURVE('',#17,#11,$,.T.);\n\
#30=ORIENTED_EDGE('',*,*,#20,.T.);\n\
#31=ORIENTED_EDGE('',*,*,#21,.T.);\n\
#32=ORIENTED_EDGE('',*,*,#22,.T.);\n\
#33=ORIENTED_EDGE('',*,*,#23,.T.);\n\
#34=EDGE_LOOP('',(#30,#31,#32,#33));\n\
#35=FACE_OUTER_BOUND('',#34,.T.);\n\
#36=ADVANCED_FACE('',(#35),#5,.T.);\n\
#40=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE({uncertainty}),#41,'closure','');\n"
	)
}

/// (a) Root cause of the mainboard refusal: a trim vertex 5 µm off its patch
/// against a projection tolerance of ~1e-5 mm. With the file asserting 0.01 mm
/// uncertainty the vertex snaps and the face imports (strict mode included);
/// with 0.001 mm asserted, strict still refuses, tolerant accepts under its
/// 10× allowance and REPORTS the snap as a repair; 0.5 mm off is refused by both.
#[test]
fn trim_vertex_off_patch_snaps_within_the_file_uncertainty() {
	let ok = import_step(&bspline_square_with_lifted_trim(0.005, 0.01)).expect("5 µm off under 10 µm uncertainty must import");
	// The boundary is consumed verbatim: the lifted vertices keep their z.
	let lifted = (0..ok.vertex_count()).filter(|&i| (ok.position(VertexId(i as u32)).z - 0.005).abs() < 1e-12).count();
	assert!(ok.face_count() >= 2 && lifted == 4, "faces={} lifted vertices={lifted} (want 4)", ok.face_count());

	let strict = import_step(&bspline_square_with_lifted_trim(0.005, 0.001));
	assert!(
		matches!(strict, Err(StepError::Unsupported(ref m)) if m.contains("does not lie on B-spline patch")),
		"5 µm off under 1 µm uncertainty must stay a strict refusal: {strict:?}"
	);
	let tol = import_step_tolerant(&bspline_square_with_lifted_trim(0.005, 0.001)).expect("tolerant import");
	let snaps = tol.repaired.iter().filter(|e| e.kind == "ADVANCED_FACE" && e.reason.contains("projected onto")).count();
	// The face reads (four reported snaps, no face skipped); the one-face
	// FRAGMENT is not a closed solid, so the census lists it as skipped for
	// validity — with its envelope from the entity geometry.
	assert!(
		tol.solids.len() == 1
			&& snaps == 4
			&& tol.skipped.iter().all(|e| e.kind != "ADVANCED_FACE")
			&& tol.solids[0].status == SolidStatus::Skipped
			&& tol.solids[0].reason.as_deref().is_some_and(|r| r.contains("valid solid"))
			&& (tol.solids[0].bbox_max - DVec3::new(10.0, 10.0, 0.005)).length() < 1e-9,
		"tolerant: solids={:?} repaired={:?} skipped={:?}",
		tol.solids,
		tol.repaired,
		tol.skipped
	);
	assert!(
		import_step(&bspline_square_with_lifted_trim(0.5, 0.01)).is_err()
			&& import_step_tolerant(&bspline_square_with_lifted_trim(0.5, 0.01)).map(|t| t.solid.is_none()).unwrap_or(true),
		"0.5 mm off must be refused by both modes"
	);
}

/// (b) Root cause of the battery refusal: a hole on a curved analytic face. A
/// half-cylinder wall (r = 10, z ∈ [0, 20], θ ∈ [0, π]) carrying a rectangular
/// window (θ ∈ [60°, 120°], z ∈ [8, 12]) bounded by two arcs and two rulings.
/// Imports as facets ON the cylinder with the window's area removed.
#[test]
fn hole_on_a_cylindrical_face_imports_on_the_exact_surface() {
	use std::f64::consts::PI;
	let r = 10.0_f64;
	let p = |theta_deg: f64, z: f64| {
		let t = theta_deg.to_radians();
		(r * t.cos(), r * t.sin(), z)
	};
	let pt = |id: u32, (x, y, z): (f64, f64, f64)| {
		format!("#{id}=CARTESIAN_POINT('',({x:.12},{y:.12},{z:.12}));\n#{}=VERTEX_POINT('',#{id});\n", id + 1)
	};
	let circle = |id: u32, z: f64| {
		format!(
			"#{id}=CARTESIAN_POINT('',(0.,0.,{z}));\n#{}=DIRECTION('',(0.,0.,1.));\n#{}=DIRECTION('',(1.,0.,0.));\n#{}=AXIS2_PLACEMENT_3D('',#{id},#{},#{});\n#{}=CIRCLE('',#{},{r});\n",
			id + 1,
			id + 2,
			id + 3,
			id + 1,
			id + 2,
			id + 4,
			id + 3
		)
	};
	let mut s = String::new();
	// Outer loop vertices: O0 = (θ0,z0), O1 = (θ180,z0), O2 = (θ180,z20), O3 = (θ0,z20).
	s += &pt(10, p(0.0, 0.0));
	s += &pt(12, p(180.0, 0.0));
	s += &pt(14, p(180.0, 20.0));
	s += &pt(16, p(0.0, 20.0));
	// Hole vertices: H0 = (120°, 8), H1 = (60°, 8), H2 = (60°, 12), H3 = (120°, 12).
	s += &pt(20, p(120.0, 8.0));
	s += &pt(22, p(60.0, 8.0));
	s += &pt(24, p(60.0, 12.0));
	s += &pt(26, p(120.0, 12.0));
	s += &circle(30, 0.0); // #34
	s += &circle(40, 20.0); // #44
	s += &circle(50, 8.0); // #54
	s += &circle(60, 12.0); // #64
						 // Edges (curve parameterisation runs with +θ; reversed traversals use ORIENTED_EDGE .F.).
	s += "#70=EDGE_CURVE('',#11,#13,#34,.T.);\n"; // bottom arc 0→180 at z=0
	s += "#71=EDGE_CURVE('',#13,#15,$,.T.);\n"; // ruling up at 180°
	s += "#72=EDGE_CURVE('',#17,#15,#44,.T.);\n"; // top arc 0→180 at z=20 (traversed reversed)
	s += "#73=EDGE_CURVE('',#17,#11,$,.T.);\n"; // ruling down at 0°
	s += "#74=EDGE_CURVE('',#23,#21,#54,.T.);\n"; // hole bottom arc 60→120 at z=8 (traversed reversed: 120→60)
	s += "#75=EDGE_CURVE('',#23,#25,$,.T.);\n"; // ruling up at 60°
	s += "#76=EDGE_CURVE('',#25,#27,#64,.T.);\n"; // hole top arc 60→120 at z=12
	s += "#77=EDGE_CURVE('',#27,#21,$,.T.);\n"; // ruling down at 120°
	s += "#80=ORIENTED_EDGE('',*,*,#70,.T.);\n#81=ORIENTED_EDGE('',*,*,#71,.T.);\n#82=ORIENTED_EDGE('',*,*,#72,.F.);\n#83=ORIENTED_EDGE('',*,*,#73,.T.);\n";
	s += "#84=EDGE_LOOP('',(#80,#81,#82,#83));\n#85=FACE_OUTER_BOUND('',#84,.T.);\n";
	s += "#90=ORIENTED_EDGE('',*,*,#74,.F.);\n#91=ORIENTED_EDGE('',*,*,#75,.T.);\n#92=ORIENTED_EDGE('',*,*,#76,.T.);\n#93=ORIENTED_EDGE('',*,*,#77,.T.);\n";
	s += "#94=EDGE_LOOP('',(#90,#91,#92,#93));\n#95=FACE_BOUND('',#94,.T.);\n";
	s += "#100=CARTESIAN_POINT('',(0.,0.,0.));\n#101=DIRECTION('',(0.,0.,1.));\n#102=DIRECTION('',(1.,0.,0.));\n#103=AXIS2_PLACEMENT_3D('',#100,#101,#102);\n";
	s += &format!("#104=CYLINDRICAL_SURFACE('',#103,{r});\n");
	s += "#105=ADVANCED_FACE('',(#85,#95),#104,.T.);\n";

	let wall = import_step(&s).expect("a cylindrical face with a hole must import");
	let max_off = (0..wall.vertex_count())
		.map(|i| {
			let q = wall.position(VertexId(i as u32));
			((q.x * q.x + q.y * q.y).sqrt() - r).abs()
		})
		.fold(0.0_f64, f64::max);
	let area = kernel_brep::area(&wall);
	// The wall minus the window — where the window's two 60° arcs are imported
	// as single chords (the ≤ 90° chord contract), so the wall keeps the two
	// circular segments between each chord and its arc: r²/2·(θ − sin θ) each.
	let theta = 60.0_f64.to_radians();
	let segment = r * r / 2.0 * (theta - theta.sin());
	let want = PI * r * 20.0 - (theta * r) * 4.0 + 2.0 * segment;
	let v = validate(&wall);
	assert!(
		!v.closed && wall.face_count() > 50 && max_off < 1e-9 && (area - want).abs() / want < 0.01,
		"holed cylinder wall: closed={} faces={} max radial deviation={max_off:.2e} area={area:.3} (want ≈{want:.3})",
		v.closed,
		wall.face_count()
	);
	// The hole's four corners are boundary vertices of the import (consumed verbatim).
	for corner in [p(120.0, 8.0), p(60.0, 8.0), p(60.0, 12.0), p(120.0, 12.0)] {
		let c = DVec3::new(corner.0, corner.1, corner.2);
		assert!(
			(0..wall.vertex_count()).any(|i| (wall.position(VertexId(i as u32)) - c).length() < 1e-12),
			"hole corner {c:?} must be a vertex of the import"
		);
	}
}

/// (c) Root cause of the Expansion Card refusal: a periodic torus/sphere region
/// the ring-grid resampler cannot phase. A torus band (R = 8, r = 2.5, the outer
/// half of the tube between the top and bottom rims) whose two rims are full
/// circles seeded at DIFFERENT longitudes (0° and 30°) — "rings off the grid
/// phase" — imports through the general periodic parameterisation as facets ON
/// the torus with the band's exact area within 1%.
#[test]
fn torus_band_with_rims_off_the_grid_phase_imports() {
	use std::f64::consts::PI;
	let (rm, rt) = (8.0_f64, 2.5_f64);
	let v2 = (rm * 30.0_f64.to_radians().cos(), rm * 30.0_f64.to_radians().sin());
	let s = format!(
		"\
#1=CARTESIAN_POINT('',(0.,0.,0.));\n\
#2=DIRECTION('',(0.,0.,1.));\n\
#3=DIRECTION('',(1.,0.,0.));\n\
#4=AXIS2_PLACEMENT_3D('',#1,#2,#3);\n\
#5=CARTESIAN_POINT('',(0.,0.,{rt}));\n\
#6=AXIS2_PLACEMENT_3D('',#5,#2,#3);\n\
#7=CIRCLE('',#6,{rm});\n\
#8=CARTESIAN_POINT('',(0.,0.,-{rt}));\n\
#9=AXIS2_PLACEMENT_3D('',#8,#2,#3);\n\
#10=CIRCLE('',#9,{rm});\n\
#11=CARTESIAN_POINT('',({rm},0.,{rt}));\n\
#12=VERTEX_POINT('',#11);\n\
#13=CARTESIAN_POINT('',({:.12},{:.12},-{rt}));\n\
#14=VERTEX_POINT('',#13);\n\
#15=EDGE_CURVE('',#12,#12,#7,.T.);\n\
#16=EDGE_CURVE('',#14,#14,#10,.T.);\n\
#17=ORIENTED_EDGE('',*,*,#15,.F.);\n\
#18=EDGE_LOOP('',(#17));\n\
#19=FACE_OUTER_BOUND('',#18,.T.);\n\
#20=ORIENTED_EDGE('',*,*,#16,.T.);\n\
#21=EDGE_LOOP('',(#20));\n\
#22=FACE_BOUND('',#21,.T.);\n\
#23=TOROIDAL_SURFACE('',#4,{rm},{rt});\n\
#24=ADVANCED_FACE('',(#19,#22),#23,.T.);\n",
		v2.0, v2.1
	);
	let band = import_step(&s).expect("a torus band with off-phase rims must import");
	let max_off = (0..band.vertex_count())
		.map(|i| {
			let q = band.position(VertexId(i as u32));
			let rho = (q.x * q.x + q.y * q.y).sqrt();
			(((rho - rm).powi(2) + q.z * q.z).sqrt() - rt).abs()
		})
		.fold(0.0_f64, f64::max);
	let area = kernel_brep::area(&band);
	let want = 2.0 * PI * rt * (rm * PI + 2.0 * rt); // outer half of the tube
	let v = validate(&band);
	assert!(
		!v.closed && band.face_count() > 200 && max_off < 1e-9 && (area - want).abs() / want < 0.01,
		"off-phase torus band: closed={} faces={} max torus deviation={max_off:.2e} area={area:.3} (want ≈{want:.3})",
		v.closed,
		band.face_count()
	);
}

/// The receipt on a clean file: a box ∪ boss exported by this kernel imports
/// tolerantly as ONE solid, imported, envelope exact, no skips, no repairs, and
/// the compound conserves the volume.
#[test]
fn tolerant_import_of_a_clean_export_lists_one_imported_solid() {
	let part = union(&cuboid(DVec3::ZERO, DVec3::new(20.0, 10.0, 5.0)), &cylinder(DVec3::new(10.0, 5.0, 5.0), DVec3::Z, 3.0, 6.0, 48));
	let step = export_step(&part, "part");
	let t = import_step_tolerant(&step).expect("tolerant import");
	let solid = t.solid.as_ref().expect("a body");
	let s = &t.solids[0];
	assert!(
		t.solids.len() == 1
			&& s.status == SolidStatus::Imported
			&& s.name == "part"
			&& s.bbox_source == "brep"
			&& (s.bbox_min - DVec3::ZERO).length() < 1e-9
			&& (s.bbox_max - DVec3::new(20.0, 10.0, 11.0)).length() < 1e-9
			&& t.skipped.is_empty()
			&& t.repaired.is_empty()
			&& validate(solid).is_valid()
			&& (volume(solid) - volume(&part)).abs() < 1e-6 * volume(&part),
		"clean tolerant import: {:?} skipped={:?} repaired={:?}",
		t.solids,
		t.skipped,
		t.repaired
	);
}

/// Per-face containment: one face of a box re-typed to an unsupported surface
/// (`SURFACE_OF_REVOLUTION`) is refused by strict mode but flat-repaired by the
/// tolerant importer — the solid still imports (its loops were consumed
/// verbatim, so the shell closes), the repair is reported with the face id.
#[test]
fn tolerant_import_flat_repairs_an_unsupported_face_and_reports_it() {
	let step = export_step(&cuboid(DVec3::ZERO, DVec3::new(4.0, 3.0, 2.0)), "box");
	// Re-type the first PLANE entity.
	let line = step.lines().find(|l| l.contains("PLANE('")).expect("a PLANE entity");
	let broken = step.replacen(line, &line.replace("PLANE('", "SURFACE_OF_REVOLUTION('"), 1);
	assert!(matches!(import_step(&broken), Err(StepError::Unsupported(_))), "strict must refuse the unsupported surface");
	let t = import_step_tolerant(&broken).expect("tolerant import");
	let solid = t.solid.as_ref().expect("the box still binds");
	let repairs: Vec<&str> = t.repaired.iter().map(|e| e.reason.as_str()).collect();
	assert!(
		t.solids.len() == 1
			&& t.solids[0].status == SolidStatus::Imported
			&& t.solids[0].faces_repaired == 1
			&& t.skipped.is_empty()
			&& t.repaired.len() == 1
			&& t.repaired[0].kind == "ADVANCED_FACE"
			&& repairs[0].contains("SURFACE_OF_REVOLUTION")
			&& repairs[0].contains("flat facets")
			&& validate(solid).is_valid()
			&& (volume(solid) - 24.0).abs() < 1e-9,
		"flat repair: solids={:?} repaired={repairs:?} skipped={:?}",
		t.solids,
		t.skipped
	);
}

/// The real vendor file (Framework Expansion Card enclosure, CC BY 4.0 — see
/// `fixtures/NOTICE`): three breps, one instanced twice → FOUR solids listed
/// with their product names and OpenCascade-agreeing envelopes (within
/// 0.05 mm), every one imported, the compound valid.
#[test]
fn framework_expansion_card_imports_tolerantly_with_its_solid_census() {
	let text = include_str!("fixtures/fw_expansion_card.step");
	let t = import_step_tolerant(text).expect("tolerant import of the Expansion Card");
	let names: Vec<&str> = t.solids.iter().map(|s| s.name.as_str()).collect();
	let imported = t.solids.iter().filter(|s| s.status == SolidStatus::Imported).count();
	assert!(
		t.solids.len() == 4
			&& names.iter().filter(|n| **n == "STAR_SCREW_M2X3L_298_1").count() == 2
			&& names.contains(&"COMPOUND_1")
			&& names.contains(&"FW_EXP_1USBC_FRAME_CLIP_BC_229_"),
		"census: {names:?}"
	);
	assert!(imported == 4, "all four solids must import (got {imported}); skipped={:?}", t.skipped);
	// OpenCascade (XCAF, `BRepBndLib::Add` without triangulation) boxes, mm.
	// Those boxes are CONSERVATIVE — OpenCascade bounds curved faces by their
	// surface UV box / poles and pads by the shape tolerance — so the contract
	// is containment: our envelope never exceeds OpenCascade's by more than
	// 0.05 mm, and it reaches the true extent (the PCB's box is tight on every
	// axis and must agree within 0.05 mm; the clip's OpenCascade box is 0.211 mm
	// wider in x than its own tessellation, which ends at ±15.0).
	let occ = [
		("COMPOUND_1", [-13.026, 3.1, -30.0], [13.026, 3.9, -0.3], true),
		("FW_EXP_1USBC_FRAME_CLIP_BC_229_", [-15.211, -0.002, -32.033], [15.211, 6.802, 0.002], false),
		("STAR_SCREW_M2X3L_298_1", [-13.194, 0.95, -12.394], [-9.406, 4.75, -8.606], false),
		("STAR_SCREW_M2X3L_298_1", [9.406, 0.95, -12.394], [13.194, 4.75, -8.606], false),
	];
	let mut matched = 0usize;
	for (name, min, max, tight) in occ {
		let (min, max) = (DVec3::from_array(min), DVec3::from_array(max));
		let inside = t.solids.iter().filter(|s| s.name == name).find(|s| {
			let contained = (s.bbox_min - min).min_element() >= -0.05 && (s.bbox_max - max).max_element() <= 0.05;
			let agrees = (s.bbox_min - min).abs().max_element() <= 0.05 && (s.bbox_max - max).abs().max_element() <= 0.05;
			// The same-named screws are told apart by their placement.
			let this_one = (s.bbox_min - min).abs().max_element() < 1.0;
			this_one && contained && (!tight || agrees)
		});
		assert!(
			inside.is_some(),
			"{name}: no listed instance sits within OpenCascade's box {min:?}..{max:?} (+0.05); solids={:?}",
			t.solids
		);
		matched += 1;
	}
	assert_eq!(matched, 4);
	// The true extent of the whole assembly: OpenCascade's own tessellation of
	// the file spans exactly [-15, 0, -32]..[15, 6.8, 0].
	let (mut lo, mut hi) = (DVec3::splat(f64::INFINITY), DVec3::splat(f64::NEG_INFINITY));
	for s in &t.solids {
		lo = lo.min(s.bbox_min);
		hi = hi.max(s.bbox_max);
	}
	assert!(
		(lo - DVec3::new(-15.0, 0.0, -32.0)).abs().max_element() <= 0.05 && (hi - DVec3::new(15.0, 6.8, 0.0)).abs().max_element() <= 0.05,
		"assembly envelope {lo:?}..{hi:?} must match the tessellated truth within 0.05 mm"
	);
	let solid = t.solid.as_ref().expect("a compound body");
	let v = validate(solid);
	assert!(v.is_valid() && v.shells >= 4, "compound must be valid with ≥ 4 shells: {v:?}");
	assert!(
		t.solids.iter().all(|s| s.faces_repaired == 0 && s.faces_skipped == 0),
		"every face must take an exact route (no flat repairs): {:?}",
		t.repaired
	);
	// The file states three uncertainties (3.95, 4.47 µm and 0.62 µm across its
	// representation contexts); the importer takes the largest.
	assert_eq!(t.uncertainty.map(|u| (u * 1e6).round() / 1e6), Some(0.004471), "the file's largest asserted uncertainty");
}

/// The census alone (no reconstruction) lists the same four instances with
/// envelopes that contain the imported ones to within the arc/pole bulges the
/// entity geometry cannot see (here < 0.05 mm): the seconds-long envelope pass.
#[test]
fn framework_expansion_card_census_matches_the_import() {
	let text = include_str!("fixtures/fw_expansion_card.step");
	let census = step_census(text).expect("census");
	let full = import_step_tolerant(text).expect("tolerant import");
	assert!(census.solid.is_none() && census.solids.len() == 4 && census.solids.iter().all(|s| s.status == SolidStatus::Skipped));
	for (c, f) in census.solids.iter().zip(&full.solids) {
		assert_eq!(c.name, f.name);
		assert_eq!(c.entity, f.entity);
		let d = (c.bbox_min - f.bbox_min).abs().max_element().max((c.bbox_max - f.bbox_max).abs().max_element());
		assert!(d <= 0.05, "{}: census envelope differs from the import's by {d:.4} mm", c.name);
	}
}

/// Developer aid: `LMCAD_STEP_CENSUS=/path/to/file.step cargo test -p kernel-brep
/// --test step_tolerant census_of_any_file -- --ignored --nocapture` prints the
/// census of any STEP file as JSON lines.
#[test]
#[ignore]
fn census_of_any_file() {
	let Ok(path) = std::env::var("LMCAD_STEP_CENSUS") else { return };
	let text = std::fs::read_to_string(&path).expect("read the STEP file");
	let t0 = std::time::Instant::now();
	let census = step_census(&text).expect("census");
	eprintln!("census of {path}: {} solids in {:.1} s", census.solids.len(), t0.elapsed().as_secs_f64());
	for s in &census.solids {
		println!(
			"{{\"name\":\"{}\",\"entity\":{},\"path\":\"{}\",\"faces\":{},\"min\":[{:.4},{:.4},{:.4}],\"max\":[{:.4},{:.4},{:.4}]}}",
			s.name, s.entity, s.path, s.faces, s.bbox_min.x, s.bbox_min.y, s.bbox_min.z, s.bbox_max.x, s.bbox_max.y, s.bbox_max.z
		);
	}
	for e in &census.repaired {
		eprintln!("note: {} #{}: {}", e.kind, e.entity, e.reason);
	}
}
