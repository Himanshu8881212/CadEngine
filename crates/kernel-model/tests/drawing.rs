// Copyright (c) LMCAD. Licensed under the MIT License.

//! Pinning tests for `kernel_model::drawing` — the dimensioned 2-D drawing
//! slice.
//!
//! What these gates prove, and why each one exists:
//!
//! - **Every dimension is a measurement.** The Ø8 bores read 8.0 and the 80 mm
//!   plate reads 80.0 to 1e-9 *because the values come out of the model*, and
//!   each carries the name of the measure that produced it.
//! - **A missing feature is refused, never invented.** Asking for bore #7 on a
//!   two-bore plate, for a wall with no enclosing boss, or for a bore's position
//!   along its own axis all produce typed refusals.
//! - **The sheet states its own limitation.** The hidden-line note is asserted
//!   to be present in the rendered SVG (and DXF) — grepped out of the real
//!   output, not out of the constant.
//! - **The output is byte-stable.** Two independent builds — rebuilding the
//!   solid, the views, the section and the dimensions from scratch — produce
//!   identical SVG and DXF bytes.
//! - **The views are right.** A known plate's front/top/right views are pinned
//!   entity by entity, coordinates included, and the cuboid's hidden-line
//!   arithmetic (12 edges → 4 degenerate, 4 visible, 4 hidden) is pinned exactly.

use kernel_brep::math::{DVec2, DVec3};
use kernel_brep::{coalesce_coplanar, cuboid, cylinder, difference, drill, HoleDepth, Solid};
use kernel_model::drawing::{
	auto_dimensions, bores, bosses, cylindrical_features, measure_dimension, project_view, section_view, Axis, CylKind, DimKind,
	DimRequest, Drawing, DrawingError, FixedTolerance, TitleBlock, View, ViewDir, ViewEntity, ViewOptions, Visibility, HLR_NOTE,
	MAX_HLR_SAMPLES, M_BORE_D, M_BORE_DEPTH, M_BORE_POS, M_COAX_WALL, M_EXTENT, PROVENANCE_NOTE,
};

// --- fixtures --------------------------------------------------------------------

/// An 80 × 40 × 10 plate with two Ø8 through-bores on `+Z` at (20, 20) and
/// (60, 20). `coalesce_coplanar` is the documented post-boolean finishing pass
/// (DESIGN_GUIDE §7.7 / FRICTION #20); a drawing is exactly the "finishing"
/// consumer it exists for.
fn drilled_plate() -> Solid {
	let plate = cuboid(DVec3::ZERO, DVec3::new(80.0, 40.0, 10.0));
	let a = drill(&plate, DVec3::new(20.0, 20.0, 0.0), DVec3::Z, 8.0, HoleDepth::Through(10.0), Some(32)).expect("first Ø8 bore");
	let b = drill(&a, DVec3::new(60.0, 20.0, 0.0), DVec3::Z, 8.0, HoleDepth::Through(10.0), Some(32)).expect("second Ø8 bore");
	coalesce_coplanar(&b)
}

/// A tube: Ø40 outside, Ø20 bore, 30 tall — the coaxial-wall and section fixture.
fn tube() -> Solid {
	let outer = cylinder(DVec3::ZERO, DVec3::Z, 20.0, 30.0, 48);
	let inner = cylinder(DVec3::new(0.0, 0.0, -1.0), DVec3::Z, 10.0, 32.0, 48);
	coalesce_coplanar(&difference(&outer, &inner))
}

/// Build the reference sheet from scratch — solid, views, section, dimensions.
/// Called twice by the determinism gate so nothing is shared between the runs.
fn reference_sheet() -> Drawing {
	let plate = drilled_plate();
	let title = TitleBlock::new("DRAW-PLATE", "2026-07-30", &FixedTolerance::new(0.2, "test fixture"))
		.with_part_number("LMC-DRW-001")
		.with_material("PLA", Some(38.5))
		.with_process("fdm");
	Drawing::new(title)
		.with_view(project_view(&plate, ViewDir::Front, &ViewOptions::default()))
		.with_view(project_view(&plate, ViewDir::Top, &ViewOptions::default()))
		.with_view(project_view(&plate, ViewDir::Right, &ViewOptions::default()))
		.with_section(section_view(&plate, DVec3::new(0.0, 20.0, 0.0), DVec3::Y, 2.0, "A-A").expect("mid-plate section"))
		.with_dimensions(auto_dimensions(&plate))
}

/// The segments of a view, sorted into a canonical order so a pin does not
/// depend on emission order.
fn sorted_segments(v: &View) -> Vec<[f64; 4]> {
	let mut out: Vec<[f64; 4]> = v
		.entities
		.iter()
		.filter_map(|e| match e {
			ViewEntity::Segment { a, b } => {
				let (p, q) = if (a.x, a.y) <= (b.x, b.y) { (*a, *b) } else { (*b, *a) };
				Some([p.x, p.y, q.x, q.y])
			}
			_ => None,
		})
		.collect();
	out.sort_by(|x, y| {
		x.iter().zip(y.iter()).find_map(|(a, b)| a.partial_cmp(b).filter(|o| o.is_ne())).unwrap_or(std::cmp::Ordering::Equal)
	});
	out
}

/// The circles of a view as `(radius, cx, cy)`, sorted.
fn sorted_circles(v: &View) -> Vec<[f64; 3]> {
	let mut out: Vec<[f64; 3]> = v
		.entities
		.iter()
		.filter_map(|e| match e {
			ViewEntity::Circle { center, radius } => Some([*radius, center.x, center.y]),
			_ => None,
		})
		.collect();
	out.sort_by(|x, y| {
		x.iter().zip(y.iter()).find_map(|(a, b)| a.partial_cmp(b).filter(|o| o.is_ne())).unwrap_or(std::cmp::Ordering::Equal)
	});
	out
}

/// Assert a set of segments matches, to 1e-9, with a rich message.
fn assert_segments(v: &View, want: &[[f64; 4]], what: &str) {
	let got = sorted_segments(v);
	assert_eq!(
		got.len(),
		want.len(),
		"{what}: expected {} drawn segments, got {} — {:?} (view receipts: considered {}, visible {}, hidden {}, merged {})",
		want.len(),
		got.len(),
		got,
		v.edges_considered,
		v.edges_visible,
		v.edges_hidden,
		v.segments_merged
	);
	for (i, (g, w)) in got.iter().zip(want.iter()).enumerate() {
		for k in 0..4 {
			assert!(
				(g[k] - w[k]).abs() < 1e-9,
				"{what}: segment {i} component {k} = {} but the model measures {} (whole segment {:?} vs {:?})",
				g[k],
				w[k],
				g,
				w
			);
		}
	}
}

/// All text content of an SVG with whitespace collapsed — so a note that the
/// renderer word-wrapped across several `<text>` elements can still be grepped
/// as one sentence.
fn svg_text(svg: &str) -> String {
	let mut out = String::new();
	let mut inside_tag = false;
	for c in svg.chars() {
		match c {
			'<' => inside_tag = true,
			'>' => {
				inside_tag = false;
				out.push(' ');
			}
			_ if !inside_tag => out.push(c),
			_ => {}
		}
	}
	out.split_whitespace().collect::<Vec<_>>().join(" ")
}

// --- dimensions are measurements --------------------------------------------------

#[test]
fn overall_extents_equal_the_exact_bounding_box_to_1e_9() {
	let s = drilled_plate();
	for (axis, want) in [(Axis::X, 80.0), (Axis::Y, 40.0), (Axis::Z, 10.0)] {
		let d = measure_dimension(&s, &DimRequest::OverallExtent(axis)).expect("a plate has overall extents");
		assert!(
			(d.value - want).abs() < 1e-9,
			"overall {} extent measured {} but the plate was built {} mm — a drawn dimension that is not the model measure is a lie",
			axis.name(),
			d.value,
			want
		);
		assert_eq!(d.from_measure, M_EXTENT, "the {} extent must record the measure that produced it", axis.name());
		assert_eq!(d.kind, DimKind::Linear, "an overall extent is a linear dimension");
	}
}

#[test]
fn bore_diameters_and_positions_are_the_exact_analytic_measures() {
	let s = drilled_plate();
	let b = bores(&s);
	assert_eq!(b.len(), 2, "the plate was drilled twice, cylindrical_features found {} bore(s): {b:?}", b.len());

	// Bores sort by radius then axis point, so #0 is the one at x = 20.
	for (index, want_x) in [(0usize, 20.0f64), (1, 60.0)] {
		let d = measure_dimension(&s, &DimRequest::BoreDiameter(index)).expect("the bore exists");
		assert!(
			(d.value - 8.0).abs() < 1e-9,
			"bore #{index} reads Ø{} but was drilled Ø8.0 — the value must come from Surface::Cylinder{{radius}} × 2, got measure '{}'",
			d.value,
			d.from_measure
		);
		assert_eq!(d.from_measure, M_BORE_D, "bore #{index} diameter provenance");
		assert_eq!(d.text, "\u{d8}8.000", "a diameter is drawn with its Ø symbol, got '{}'", d.text);

		let px = measure_dimension(&s, &DimRequest::BorePosition { index, axis: Axis::X }).expect("in-plane position");
		assert!(
			(px.value - want_x).abs() < 1e-9,
			"bore #{index} X position reads {} but was drilled at x = {want_x} (datum = bounding_box.min)",
			px.value
		);
		assert_eq!(px.from_measure, M_BORE_POS, "bore #{index} position provenance");

		let py = measure_dimension(&s, &DimRequest::BorePosition { index, axis: Axis::Y }).expect("in-plane position");
		assert!((py.value - 20.0).abs() < 1e-9, "bore #{index} Y position reads {} but was drilled at y = 20", py.value);

		let depth = measure_dimension(&s, &DimRequest::BoreDepth(index)).expect("axial depth");
		assert!(
			(depth.value - 10.0).abs() < 1e-9,
			"bore #{index} depth reads {} but the plate is 10 mm thick and the bore goes through",
			depth.value
		);
		assert_eq!(depth.from_measure, M_BORE_DEPTH, "bore #{index} depth provenance");
	}
}

#[test]
fn a_coaxial_wall_is_the_difference_of_two_analytic_radii() {
	let t = tube();
	let feats = cylindrical_features(&t);
	assert_eq!(feats.iter().filter(|f| f.kind == CylKind::Bore).count(), 1, "the tube has exactly one bore, got features {feats:?}");
	assert_eq!(bosses(&t).len(), 1, "the tube has exactly one round boss (its own Ø40 outside)");
	let d = measure_dimension(&t, &DimRequest::CoaxialWall(0)).expect("a Ø20 bore inside a Ø40 boss has a determinable wall");
	assert!(
		(d.value - 10.0).abs() < 1e-9,
		"the tube wall reads {} but R20 − R10 = 10.0 exactly — measured from the two Surface::Cylinder radii, got measure '{}'",
		d.value,
		d.from_measure
	);
	assert_eq!(d.from_measure, M_COAX_WALL, "wall provenance");
	assert_eq!(d.kind, DimKind::Wall, "a ligament is a wall dimension");
}

#[test]
fn every_auto_dimension_records_a_real_measure_and_none_is_a_literal() {
	let s = drilled_plate();
	let dims = auto_dimensions(&s);
	assert_eq!(
		dims.len(),
		3 + 2 * 4,
		"expected 3 extents + 4 dimensions per bore (Ø, depth, X, Y) for 2 bores, got {}: {:?}",
		dims.len(),
		dims.iter().map(|d| d.subject.clone()).collect::<Vec<_>>()
	);
	let known = [M_EXTENT, M_BORE_D, M_BORE_DEPTH, M_BORE_POS, M_COAX_WALL];
	for d in &dims {
		assert!(
			known.contains(&d.from_measure),
			"dimension '{}' claims measure '{}', which is not one of the module's declared measures {known:?}",
			d.text,
			d.from_measure
		);
		assert!(!d.subject.is_empty(), "dimension '{}' must name the feature it measured", d.text);
		assert!(d.value.is_finite(), "dimension '{}' is not a finite number", d.text);
	}
	// The plate's bores have no enclosing coaxial boss, so no wall is offered —
	// an omission, never an invented number.
	assert!(
		!dims.iter().any(|d| d.from_measure == M_COAX_WALL),
		"a flat plate's bores have no enclosing boss; auto_dimensions must omit the wall, not guess it"
	);
}

// --- negative controls ------------------------------------------------------------

#[test]
fn dimensioning_a_bore_that_does_not_exist_refuses_loudly() {
	let s = drilled_plate();
	let err = measure_dimension(&s, &DimRequest::BoreDiameter(7)).expect_err("the plate has 2 bores, not 8 — this must refuse");
	match &err {
		DrawingError::FeatureNotFound { requested, available } => {
			assert_eq!(requested, "bore #7", "the refusal must name what was asked for, got '{requested}'");
			assert!(
				available.contains("2 bore(s)"),
				"the refusal must name what the model DOES have so the caller can fix the request, got '{available}'"
			);
		}
		other => panic!("expected FeatureNotFound for a non-existent bore, got {other:?}"),
	}
	let msg = err.to_string();
	assert!(msg.contains("never invents a dimension value"), "the refusal message must say why there is no number, got '{msg}'");
	// And the same request on a position / depth refuses identically — no path
	// through the API produces a fabricated value for a missing feature.
	for req in [DimRequest::BoreDepth(7), DimRequest::BorePosition { index: 7, axis: Axis::X }, DimRequest::CoaxialWall(7)] {
		assert!(
			matches!(measure_dimension(&s, &req), Err(DrawingError::FeatureNotFound { .. })),
			"{req:?} on a 2-bore plate must refuse with FeatureNotFound"
		);
	}
}

#[test]
fn a_wall_with_no_enclosing_boss_refuses_rather_than_guessing() {
	let s = drilled_plate();
	let err = measure_dimension(&s, &DimRequest::CoaxialWall(0)).expect_err("a bore in a flat plate has no annular wall");
	match &err {
		DrawingError::NotDeterminable { measure, subject, why } => {
			assert_eq!(*measure, M_COAX_WALL, "the refusal must name the measure it could not take");
			assert!(subject.contains("bore #0"), "the refusal must name the feature, got '{subject}'");
			assert!(why.contains("no coaxial cylindrical boss"), "the refusal must say what the geometry lacks, got '{why}'");
		}
		other => panic!("expected NotDeterminable for a wall with no enclosing boss, got {other:?}"),
	}
}

#[test]
fn a_bore_has_no_position_along_its_own_axis() {
	let s = drilled_plate();
	let err =
		measure_dimension(&s, &DimRequest::BorePosition { index: 0, axis: Axis::Z }).expect_err("a +Z bore has no located Z position");
	assert!(
		matches!(&err, DrawingError::NotDeterminable { measure, .. } if *measure == M_BORE_POS),
		"expected NotDeterminable naming the position measure, got {err:?}"
	);
}

#[test]
fn a_section_plane_that_misses_the_solid_refuses() {
	let s = drilled_plate();
	let err =
		section_view(&s, DVec3::new(0.0, 0.0, 500.0), DVec3::Z, 2.0, "Z-Z").expect_err("a plane 500 mm above a 10 mm plate cuts nothing");
	assert!(matches!(err, DrawingError::EmptySection { .. }), "expected EmptySection, got {err:?}");
	assert!(err.to_string().contains("cuts no material"), "the refusal must say what happened, got '{err}'");
}

#[test]
fn absurd_section_parameters_refuse() {
	let s = drilled_plate();
	for (pitch, normal, field) in [(0.0, DVec3::Z, "hatch_pitch"), (f64::NAN, DVec3::Z, "hatch_pitch"), (2.0, DVec3::ZERO, "plane_normal")]
	{
		let err = section_view(&s, DVec3::new(0.0, 0.0, 5.0), normal, pitch, "A-A").expect_err("an impossible section must refuse");
		assert!(matches!(&err, DrawingError::BadInput { field: f, .. } if *f == field), "expected BadInput on '{field}', got {err:?}");
	}
}

// --- the views --------------------------------------------------------------------

#[test]
fn hidden_line_removal_removes_the_back_of_a_box_and_the_arithmetic_is_pinned() {
	let bx = cuboid(DVec3::ZERO, DVec3::new(80.0, 40.0, 10.0));
	let hlr = project_view(&bx, ViewDir::Front, &ViewOptions::default());
	let wire =
		project_view(&bx, ViewDir::Front, &ViewOptions { hidden_line_removal: false, merge_collinear: false, ..ViewOptions::default() });
	// A cuboid has 12 edges. Seen from the front, 4 run along the sight and
	// project to points (counted nowhere); of the remaining 8, the 4 on the near
	// face are visible and the 4 on the far face are occluded by it.
	assert_eq!(hlr.edges_considered, 8, "12 box edges − 4 parallel to the sight = 8 offered to the visibility test");
	assert_eq!(hlr.edges_visible, 4, "only the near face's 4 edges are visible, got {}", hlr.edges_visible);
	assert_eq!(hlr.edges_hidden, 4, "the far face's 4 edges must be REMOVED, got {} hidden", hlr.edges_hidden);
	assert_eq!(hlr.segment_count(), 4, "the front view of a box is a rectangle: 4 segments, got {}", hlr.segment_count());
	assert_eq!(
		wire.segment_count(),
		8,
		"with HLR off and no merge the wireframe draws all 8 non-degenerate edges, got {}",
		wire.segment_count()
	);
	assert_eq!(
		hlr.visibility,
		Visibility::RaySampled { max_samples_per_edge: MAX_HLR_SAMPLES },
		"the view must record HOW visibility was decided"
	);
	assert_eq!(wire.visibility, Visibility::Wireframe, "a wireframe must say so");
}

#[test]
fn front_view_of_the_drilled_plate_is_its_exact_outline() {
	let s = drilled_plate();
	let v = project_view(&s, ViewDir::Front, &ViewOptions::default());
	assert_segments(
		&v,
		&[[0.0, 0.0, 0.0, 10.0], [0.0, 0.0, 80.0, 0.0], [0.0, 10.0, 80.0, 10.0], [80.0, 0.0, 80.0, 10.0]],
		"front view of the 80×40×10 drilled plate",
	);
	assert_eq!(v.circles, 0, "no bore axis is parallel to the front sight, so no analytic circle belongs here");
	assert!(
		(v.min - DVec2::ZERO).length() < 1e-9 && (v.max - DVec2::new(80.0, 10.0)).length() < 1e-9,
		"front view extents are {:?}..{:?}, expected (0,0)..(80,10) — the projection of the model's own 80 × 10 face",
		v.min,
		v.max
	);
	assert!(
		v.edges_hidden > 0,
		"the plate's two bores and its far face lie behind the front wall — HLR must have removed something, hidden = {}",
		v.edges_hidden
	);
}

#[test]
fn top_view_shows_the_bores_as_true_circles_at_the_analytic_radius() {
	let s = drilled_plate();
	let v = project_view(&s, ViewDir::Top, &ViewOptions::default());
	assert_segments(
		&v,
		&[[0.0, 0.0, 0.0, 40.0], [0.0, 0.0, 80.0, 0.0], [0.0, 40.0, 80.0, 40.0], [80.0, 0.0, 80.0, 40.0]],
		"top view of the drilled plate",
	);
	assert_eq!(
		sorted_circles(&v),
		vec![[4.0, 20.0, 20.0], [4.0, 60.0, 20.0]],
		"a bore seen down its own axis must draw as a TRUE circle at the analytic Surface::Cylinder radius (R4 at the drilled centres), not as a 32-gon"
	);
	assert_eq!(v.circles, 2, "two bores, two circles");
}

#[test]
fn right_view_is_the_plates_end_elevation() {
	let s = drilled_plate();
	let v = project_view(&s, ViewDir::Right, &ViewOptions::default());
	assert_segments(
		&v,
		&[[0.0, 0.0, 0.0, 10.0], [0.0, 0.0, 40.0, 0.0], [0.0, 10.0, 40.0, 10.0], [40.0, 0.0, 40.0, 10.0]],
		"right view of the drilled plate (40 deep × 10 thick)",
	);
}

#[test]
fn a_view_is_a_deterministic_function_of_the_solid() {
	let a = project_view(&drilled_plate(), ViewDir::Iso, &ViewOptions::default());
	let b = project_view(&drilled_plate(), ViewDir::Iso, &ViewOptions::default());
	assert_eq!(a.entities.len(), b.entities.len(), "two identical iso projections produced different entity counts");
	assert_eq!(a.entities, b.entities, "two identical iso projections produced different geometry");
	assert_eq!(
		(a.edges_considered, a.edges_visible, a.edges_hidden, a.segments_merged),
		(b.edges_considered, b.edges_visible, b.edges_hidden, b.segments_merged),
		"the view receipts must be reproducible"
	);
}

// --- the section ------------------------------------------------------------------

#[test]
fn section_of_a_tube_cuts_two_walls_and_hatches_only_the_material() {
	let t = tube();
	// The plane x = 0 contains the axis: the cut is the two wall rectangles,
	// 10 mm thick × 30 mm tall. The cut-plane frame from perp_basis(+X) is
	// (u, v) = (+Y, +Z), so material is |u| ∈ [10, 20], v ∈ [0, 30].
	let sec = section_view(&t, DVec3::ZERO, DVec3::X, 2.0, "A-A").expect("a plane through the axis cuts the tube");
	assert_eq!(sec.boundary.len(), 2, "an axial cut of a tube is TWO closed wall loops, got {}", sec.boundary.len());
	// The boundary is MY f64 construction: pin both wall rectangles exactly.
	let mut walls: Vec<(f64, f64, f64, f64)> = sec
		.boundary
		.iter()
		.map(|e| match e {
			ViewEntity::Polyline(pts) => (
				pts.iter().map(|p| p.x).fold(f64::INFINITY, f64::min),
				pts.iter().map(|p| p.x).fold(f64::NEG_INFINITY, f64::max),
				pts.iter().map(|p| p.y).fold(f64::INFINITY, f64::min),
				pts.iter().map(|p| p.y).fold(f64::NEG_INFINITY, f64::max),
			),
			other => panic!("a section boundary must be a closed polyline, got {other:?}"),
		})
		.collect();
	walls.sort_by(|a, b| a.0.total_cmp(&b.0));
	for (i, (got, want)) in walls.iter().zip([(-20.0, -10.0, 0.0, 30.0), (10.0, 20.0, 0.0, 30.0)]).enumerate() {
		let g = [got.0, got.1, got.2, got.3];
		let w = [want.0, want.1, want.2, want.3];
		for k in 0..4 {
			assert!(
				(g[k] - w[k]).abs() < 1e-9,
				"wall {i} bound {k} = {} but a Ø40/Ø20 × 30 tube cut on its axis gives {} (whole rect {g:?} vs {w:?})",
				g[k],
				w[k]
			);
		}
	}
	// The AREA receipt is kernel_brep::section_properties, which integrates the
	// TESSELLATION, so it lands near — not on — the exact 600 mm². The band is
	// summation-order noise, not accuracy: the 2026-08-23 annular-cap
	// triangulation rework moved the residual from <1e-6 to 1.8e-6 on the same
	// exact geometry (different, correct facet layout), so the pin allows 5e-6
	// — still eight significant figures on a 600 mm² section.
	let area = sec.area_mm2.expect("kernel_brep::section_properties measures the cut");
	assert!(
		(area - 600.0).abs() < 5e-6,
		"the cut area reads {area} mm²; the two 10 × 30 walls are 600 mm² exactly and section_properties integrates the tessellation, so the residual must stay under 5e-6 (it was {:e})",
		(area - 600.0).abs()
	);
	assert!(
		sec.exact_curves > 0,
		"kernel_brep::section_curves_with_fallback must report closed-form conics for this cut, got {} exact / {} polyline",
		sec.exact_curves,
		sec.polyline_curves
	);
	assert_eq!(sec.polyline_curves, 0, "no oblique-torus fallback is involved in a cylinder cut");
	assert!(!sec.hatch.is_empty(), "a section with material must be hatched, got {} segments", sec.hatch.len());
	for (i, (a, b)) in sec.hatch.iter().enumerate() {
		let mid = (*a + *b) * 0.5;
		assert!(
			mid.x.abs() >= 10.0 - 1e-6 && mid.x.abs() <= 20.0 + 1e-6 && mid.y >= -1e-6 && mid.y <= 30.0 + 1e-6,
			"hatch segment {i} runs through {mid:?}, which is outside the cut material (|u| ∈ [10, 20], v ∈ [0, 30]) — hatching the bore would draw material that is not there"
		);
		let dir = (*b - *a).normalize();
		assert!((dir.x.abs() - dir.y.abs()).abs() < 1e-9, "hatch segment {i} runs {dir:?}, but section hatching is 45°");
	}
}

#[test]
fn a_section_through_two_bores_subtracts_them_exactly() {
	let s = drilled_plate();
	// The plane y = 20 runs through both bore axes: the cut is the 80 × 10
	// elevation MINUS the two Ø8 bores at their full diameter.
	let sec = section_view(&s, DVec3::new(0.0, 20.0, 0.0), DVec3::Y, 2.0, "A-A").expect("mid-plate section");
	assert_eq!(
		sec.boundary.len(),
		3,
		"cutting through both bore centres leaves THREE material bands (0–16, 24–56, 64–80 mm), got {} loop(s)",
		sec.boundary.len()
	);
	// The boundary is MY f64 construction, so it is pinned exactly: the three
	// bands run 0–16, 24–56 and 64–80 mm in model x. The cut-plane frame from
	// perp_basis(+Y) is (u, v) = (+X, −Z) about the plane point, so model
	// x = u.
	let mut bands: Vec<(f64, f64)> = sec
		.boundary
		.iter()
		.map(|e| match e {
			ViewEntity::Polyline(pts) => {
				let lo = pts.iter().map(|p| p.x).fold(f64::INFINITY, f64::min);
				let hi = pts.iter().map(|p| p.x).fold(f64::NEG_INFINITY, f64::max);
				(lo, hi)
			}
			other => panic!("a section boundary must be a closed polyline, got {other:?}"),
		})
		.collect();
	bands.sort_by(|a, b| a.0.total_cmp(&b.0));
	for (i, (got, want)) in bands.iter().zip([(0.0, 16.0), (24.0, 56.0), (64.0, 80.0)]).enumerate() {
		assert!(
			(got.0 - want.0).abs() < 1e-9 && (got.1 - want.1).abs() < 1e-9,
			"section band {i} spans model x {:.9}..{:.9}, but the plate minus two Ø8 bores at x = 20 and 60 leaves {:.1}..{:.1}",
			got.0,
			got.1,
			want.0,
			want.1
		);
	}
	// The AREA receipt is kernel_brep::section_properties, which integrates the
	// TESSELLATION — so it lands near, not on, the exact 640 mm². Gated at the
	// tessellation's accuracy, and labelled as such rather than pretended exact.
	let area = sec.area_mm2.expect("section_properties measures the cut");
	assert!(
		(area - 640.0).abs() < 1e-5,
		"the cut area reads {area} mm²; 80 × 10 − 2 × (8 × 10) = 640 mm² exactly and kernel_brep::section_properties integrates the tessellation, so the residual must stay under 1e-5 (it was {:e})",
		(area - 640.0).abs()
	);
	// No hatch may fall inside either bore.
	for (i, (a, b)) in sec.hatch.iter().enumerate() {
		let mid = (*a + *b) * 0.5;
		// Cut-plane frame from perp_basis(+Y) is (u, v) = (+X, −Z) about the
		// plane point (0, 20, 0), so model x = u.
		let x = mid.x;
		for (bore, cx) in [(0, 20.0f64), (1, 60.0)] {
			assert!(
				(x - cx).abs() >= 4.0 - 1e-6,
				"hatch segment {i} runs through model x = {x}, which is inside bore #{bore} (Ø8 at x = {cx}) — hatching a hole invents material"
			);
		}
	}
}

#[test]
fn section_hatch_pitch_is_honoured_and_deterministic() {
	let t = tube();
	let coarse = section_view(&t, DVec3::ZERO, DVec3::X, 4.0, "A-A").expect("section");
	let fine = section_view(&t, DVec3::ZERO, DVec3::X, 1.0, "A-A").expect("section");
	assert!(
		fine.hatch.len() > coarse.hatch.len(),
		"a 1 mm hatch pitch must draw more lines than a 4 mm one: {} vs {}",
		fine.hatch.len(),
		coarse.hatch.len()
	);
	let again = section_view(&tube(), DVec3::ZERO, DVec3::X, 1.0, "A-A").expect("section");
	assert_eq!(fine.hatch.len(), again.hatch.len(), "the hatch pattern must be reproducible");
	for (i, (a, b)) in fine.hatch.iter().enumerate() {
		assert!(
			(a.x - again.hatch[i].0.x).abs() < 1e-12 && (b.y - again.hatch[i].1.y).abs() < 1e-12,
			"hatch segment {i} moved between runs: {a:?}->{b:?} vs {:?}->{:?}",
			again.hatch[i].0,
			again.hatch[i].1
		);
	}
}

// --- the sheet --------------------------------------------------------------------

#[test]
fn the_sheet_states_the_hidden_line_limitation_in_its_own_output() {
	let sheet = reference_sheet();
	let svg = sheet.to_svg();
	let text = svg_text(&svg);
	assert!(
		text.contains(HLR_NOTE),
		"the rendered SVG must carry the hidden-line note VERBATIM — a drawing that silently omits hidden detail is a trap. Sheet text was:\n{text}"
	);
	assert!(text.contains(PROVENANCE_NOTE), "the rendered SVG must state where its numbers come from. Sheet text was:\n{text}");
	assert!(
		text.contains("GENERAL TOLERANCE +/-0.200 mm (source: test fixture)"),
		"the units/tolerance note must carry the caller-supplied value AND its source. Sheet text was:\n{text}"
	);
	assert!(text.contains("DIMENSION SCHEDULE"), "the auditable dimension schedule must be on the sheet. Sheet text was:\n{text}");
	let dxf = sheet.to_dxf();
	assert!(
		dxf.contains("VISIBLE EDGES ONLY"),
		"the DXF must carry the hidden-line limitation too — the exchange file travels further than the SVG"
	);
}

#[test]
fn the_dimension_schedule_carries_every_value_with_its_measure() {
	let sheet = reference_sheet();
	let text = svg_text(&sheet.to_svg());
	for d in &sheet.dimensions {
		let row = format!("{} | {} | {} | {}", d.text, d.kind.name(), d.subject, d.from_measure);
		assert!(
			text.contains(&row),
			"the schedule must show '{row}' so a reader can audit where the number came from. Sheet text was:\n{text}"
		);
	}
	assert!(text.contains("\u{d8}8.000 | diameter"), "the Ø8 bore must appear in the schedule as a diameter. Sheet text was:\n{text}");
	assert!(text.contains("80.000 | linear"), "the 80 mm extent must appear in the schedule");
}

#[test]
fn the_title_block_carries_caller_identity_and_no_clock_read() {
	let sheet = reference_sheet();
	let text = svg_text(&sheet.to_svg());
	for row in ["PART: DRAW-PLATE", "PART NO: LMC-DRW-001", "MATERIAL: PLA", "MASS: 38.500 g", "PROCESS: fdm", "DATE: 2026-07-30"] {
		assert!(text.contains(row), "the title block must show '{row}'. Sheet text was:\n{text}");
	}
	assert_eq!(sheet.scale_text(), "2:1", "an 80 mm part on an A3 sheet snaps to a STANDARD scale, got {}", sheet.scale_text());
	// The date is an INPUT, not a clock read: change only the date and exactly
	// that one substring of the output changes.
	let mut other = reference_sheet();
	other.title.date = "1999-01-01".to_string();
	let a = sheet.to_svg();
	let b = other.to_svg();
	assert_ne!(a, b, "changing the title-block date must change the sheet");
	assert_eq!(
		a.replace("2026-07-30", "1999-01-01"),
		b,
		"the date must appear in the output ONLY where the caller put it — anything else would mean the module read a clock"
	);
}

#[test]
fn svg_is_byte_identical_across_two_independent_builds() {
	let a = reference_sheet().to_svg();
	let b = reference_sheet().to_svg();
	assert_eq!(a.len(), b.len(), "two independent sheet builds produced {} vs {} SVG bytes", a.len(), b.len());
	if a != b {
		let at = a.as_bytes();
		let bt = b.as_bytes();
		let i = at.iter().zip(bt.iter()).position(|(x, y)| x != y).unwrap_or(0);
		let lo = i.saturating_sub(80);
		panic!(
			"SVG is not byte-stable: first difference at byte {i}\n  run A: …{}…\n  run B: …{}…",
			&a[lo..(i + 80).min(a.len())],
			&b[lo..(i + 80).min(b.len())]
		);
	}
	assert!(a.starts_with("<svg xmlns="), "the SVG must be a bare, self-describing document");
	assert!(a.contains("width=\"420.0000mm\""), "the sheet is A3 landscape in real millimetres");
	// Fixed-decimal formatting is what makes it stable: no exponents, no −0.
	let exponent =
		a.as_bytes().windows(3).any(|w| w[0].is_ascii_digit() && (w[1] == b'e' || w[1] == b'E') && (w[2] == b'-' || w[2] == b'+'));
	assert!(!exponent, "no coordinate may be written in exponent form — shortest-round-trip formatting is not byte-stable");
	assert!(!a.contains("\"-0.0000\""), "negative zero must be normalized away");
	assert!(!a.contains(">-0.000<"), "negative zero must be normalized away in dimension text too");
}

#[test]
fn dxf_is_byte_identical_and_is_a_readable_r12_document() {
	let a = reference_sheet().to_dxf();
	let b = reference_sheet().to_dxf();
	assert_eq!(a, b, "two independent sheet builds produced different DXF bytes ({} vs {})", a.len(), b.len());
	for token in ["AC1009", "\nENDSEC\n", "\nEOF\n", "\nLINE\n", "\nTEXT\n", "\nLAYER\n", "\nCONTINUOUS\n"] {
		assert!(a.contains(token), "an R12 DXF must contain '{}' — got a {}-byte document", token.escape_debug(), a.len());
	}
	assert!(a.contains("\nCIRCLE\n"), "the top view's bores must travel as real CIRCLE entities, not polygons");
	// The Ø control code, not a raw non-ASCII byte, is what R12 readers expect.
	assert!(a.contains("%%C8.000"), "a diameter value must use the AutoCAD %%C control code in DXF");
	assert!(!a.contains('\u{d8}'), "no raw Ø byte may reach an R12 DXF");
	// `0\nSECTION\n2\n` is the section HEADER pattern; a bare "SECTION" also
	// occurs as a layer name and as every cut line's layer reference.
	let sections = a.matches("0\nSECTION\n2\n").count();
	assert_eq!(sections, 3, "R12 needs HEADER, TABLES and ENTITIES sections, found {sections}");
	for name in ["HEADER", "TABLES", "ENTITIES"] {
		assert!(a.contains(&format!("0\nSECTION\n2\n{name}\n")), "the DXF is missing its {name} section");
	}
}

#[test]
fn a_sheet_with_no_geometry_still_renders_its_notes() {
	// Nothing to draw is not a failure — but the limitation note is not optional.
	let sheet = Drawing::new(TitleBlock::new("EMPTY", "2026-07-30", &FixedTolerance::new(0.1, "none")));
	let text = svg_text(&sheet.to_svg());
	assert!(text.contains(HLR_NOTE), "even an empty sheet states its limitation. Got:\n{text}");
	assert_eq!(sheet.scale_text(), "1:1", "an empty sheet has nothing to scale");
}
