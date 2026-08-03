// Copyright (c) LMCAD. Licensed under the MIT License.

//! **DESIGN_GUIDE §24 item 1, the slice that flipped.** A freeform (NURBS)
//! face could not be an operand of a boolean at all; it can now carry ONE
//! bounded operation — a planar half-space cut — under an explicitly stated
//! *exact-surface, tolerance-curve* contract:
//!
//! - the rational patch is EXACT and untouched (both trimmed halves reference
//!   the same control net / knot vectors, bit for bit);
//! - the plane∩patch intersection CURVE is a polyline traced in the patch's
//!   parameter chart and refined to a stated chord tolerance;
//! - the cut SOLID is the operand tessellation trimmed against the plane with
//!   its patch seam snapped onto that exact curve, gated watertight.
//!
//! These tests pin the capability against an **independent oracle** (a chart
//! quadrature of the exact patch that never touches the boolean/mesh code),
//! pin the curve tolerances, pin the STEP round-trip of the result, and pin
//! the refusal boundary — everything outside the slice must refuse LOUDLY
//! with a message naming the slice.

use kernel_brep::checked::{try_freeform_boolean, FreeformTool};
use kernel_brep::freeform::{
	freeform_plane_cut, freeform_plate, plane_patch_curves, FreeformBoolError, FreeformCutOptions, FreeformSolid,
};
use kernel_brep::math::{DVec2, DVec3};
use kernel_brep::{Keep, MeshBoolOp, NurbsSurface};

// ---------------------------------------------------------------------------
// Fixture: a genuinely freeform "pillow plate" — a bicubic B-spline dome over a
// flat base. Plan coordinates run on a regular 0..40 lattice; z is sculpted, so
// the top face is a real curved patch (not a ruled or developable cheat).
// ---------------------------------------------------------------------------

const PLATE_SPAN: f64 = 40.0;
const PLATE_BASE_Z: f64 = 0.0;

fn pillow_patch() -> NurbsSurface {
	// 5×5 control grid, degree 3 in both directions, clamped open-uniform knots
	// ([0,0,0,0,1,2,2,2,2] for 5 control points at degree 3).
	let knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0, 2.0];
	let mut control = Vec::new();
	let mut weights = Vec::new();
	for i in 0..5 {
		let mut row = Vec::new();
		let mut wrow = Vec::new();
		for j in 0..5 {
			let x = PLATE_SPAN * i as f64 / 4.0;
			let y = PLATE_SPAN * j as f64 / 4.0;
			// Rim at 6, interior lifted to 15 (corners of the interior 3×3 to 11):
			// a smooth pillow with real curvature in both directions.
			let interior = (1..=3).contains(&i) && (1..=3).contains(&j);
			let centre = i == 2 && j == 2;
			let z = if centre {
				15.0
			} else if interior {
				11.0
			} else {
				6.0
			};
			row.push(DVec3::new(x, y, z));
			wrow.push(1.0);
		}
		control.push(row);
		weights.push(wrow);
	}
	NurbsSurface::new(3, 3, knots.clone(), knots, control, weights).expect("pillow patch")
}

fn pillow_plate(nu: usize, nv: usize) -> FreeformSolid {
	freeform_plate(&pillow_patch(), PLATE_BASE_Z, nu, nv).expect("pillow plate builds")
}

// ---------------------------------------------------------------------------
// The independent oracle: a chart quadrature of the EXACT patch.
//
// For a vertical cut plane the kept-side predicate depends only on the plan
// (x, y) position, so the plate volume kept is
//     V = ∫∫_{chart : S·n ≥ d} (S_z − base) · |∂(x,y)/∂(u,v)| du dv
// evaluated straight off `NurbsSurface::point_at` / `partials`. This shares NO
// code with the boolean, the trimmer, the mesh, or the tessellator — it is a
// genuine outside check. Cells whose corners disagree on the predicate are
// subdivided `REFINE`× per axis, which drives the boundary quadrature error
// well under the tessellation band the gates state.
// ---------------------------------------------------------------------------

fn oracle_volume(surf: &NurbsSurface, base_z: f64, plane_origin: DVec3, plane_normal: DVec3, keep: Keep, n: usize) -> f64 {
	const REFINE: usize = 8;
	let ((u0, u1), (v0, v1)) = surf.domain();
	let (su, sv) = (u1 - u0, v1 - v0);
	let keep_sign = if keep == Keep::Outside { 1.0 } else { -1.0 };
	let kept = |fu: f64, fv: f64| {
		let p = surf.point_at(u0 + su * fu, v0 + sv * fv);
		(p - plane_origin).dot(plane_normal) * keep_sign >= 0.0
	};
	// Integrand: (height above base) × |plan Jacobian|, in normalized chart
	// coordinates (the chart spans are folded into the Jacobian).
	let cell = |fu: f64, fv: f64| {
		let (u, v) = (u0 + su * fu, v0 + sv * fv);
		let p = surf.point_at(u, v);
		let (du, dv) = surf.partials(u, v);
		let jac = (du.x * dv.y - du.y * dv.x).abs() * su * sv;
		(p.z - base_z) * jac
	};
	let h = 1.0 / n as f64;
	let mut total = 0.0;
	for i in 0..n {
		for j in 0..n {
			let (fu, fv) = (i as f64 * h, j as f64 * h);
			let corners = [kept(fu, fv), kept(fu + h, fv), kept(fu, fv + h), kept(fu + h, fv + h)];
			let all_in = corners.iter().all(|&c| c);
			let all_out = corners.iter().all(|&c| !c);
			if all_out {
				continue;
			}
			if all_in {
				total += cell(fu + h * 0.5, fv + h * 0.5) * h * h;
				continue;
			}
			// Straddling cell: refine.
			let hr = h / REFINE as f64;
			for a in 0..REFINE {
				for b in 0..REFINE {
					let (cu, cv) = (fu + (a as f64 + 0.5) * hr, fv + (b as f64 + 0.5) * hr);
					if kept(cu, cv) {
						total += cell(cu, cv) * hr * hr;
					}
				}
			}
		}
	}
	total
}

/// Mesh volume (positive), the measured quantity of every cut gate.
fn vol(m: &kernel_brep::Mesh) -> f64 {
	m.signed_volume().abs()
}

// ---------------------------------------------------------------------------
// THE §24 ITEM 1 REPRO — flipped from "cannot be an operand" to measured.
// ---------------------------------------------------------------------------

#[test]
fn freeform_face_carries_a_planar_cut_with_volume_conservation_and_an_oracle_check() {
	let plate = pillow_plate(28, 28);
	let uncut = vol(&plate.mesh);
	// A vertical cut plane through the dome's flank (x = 15 of a 0..40 span).
	let origin = DVec3::new(15.0, 0.0, 0.0);
	let normal = DVec3::X;
	let opts = FreeformCutOptions::default();

	let keep_hi = freeform_plane_cut(&plate, origin, normal, Keep::Outside, &opts)
		.unwrap_or_else(|e| panic!("the §24-item-1 repro must now CUT, not refuse: {e}"));
	let keep_lo = freeform_plane_cut(&plate, origin, normal, Keep::Inside, &opts)
		.unwrap_or_else(|e| panic!("the complementary half must cut too: {e}"));

	// (1) Both halves are closed, watertight solids (the gate inside the cut
	// already withholds otherwise — this re-proves it from the outside).
	let (v_hi, v_lo) = (vol(&keep_hi.mesh), vol(&keep_lo.mesh));
	assert!(
		keep_hi.mesh.is_watertight() && keep_lo.mesh.is_watertight() && keep_hi.mesh.is_two_manifold() && keep_lo.mesh.is_two_manifold(),
		"both cut halves must be closed 2-manifolds: hi watertight={} 2mf={} / lo watertight={} 2mf={}",
		keep_hi.mesh.is_watertight(),
		keep_hi.mesh.is_two_manifold(),
		keep_lo.mesh.is_watertight(),
		keep_lo.mesh.is_two_manifold()
	);

	// (2a) The cut CROSS-SECTION is shared exactly: both halves cap the same
	// planar polygon, so their cap areas must agree to round-off. This is the
	// sharp, tessellation-free statement about the cut itself.
	let cap_gap = (keep_hi.cap_area - keep_lo.cap_area).abs();
	assert!(
		cap_gap < 1e-9 && keep_hi.cap_area > 0.0,
		"the two halves must cap the SAME cross-section: {:.9} vs {:.9} (gap {cap_gap:.3e}, limit 1e-9)",
		keep_hi.cap_area,
		keep_lo.cap_area
	);

	// (2b) CONSERVATION — the two halves re-sum to the operand. This is NOT
	// bit-exact and the honest reason is worth stating: each half's seam is
	// snapped onto the exact intersection curve independently, and the facets
	// dragged along by that snap differ between the halves, so the sum carries
	// a second-order (chord-error × snap-displacement) term. It is a
	// TESSELLATION artifact, not a boolean defect — which the refinement pin
	// below proves by driving it to zero.
	let sum_rel = ((v_hi + v_lo) - uncut).abs() / uncut;
	assert!(
		sum_rel < 5e-5,
		"the two cut halves must reconstruct the operand: {v_hi:.6} + {v_lo:.6} = {:.6} vs uncut {uncut:.6} (rel {sum_rel:.3e}, limit 5e-5)",
		v_hi + v_lo
	);

	// (2c) …and that residual must CONVERGE with the operand tessellation —
	// the evidence that it is chord error and not a leak. Measured on this
	// fixture: 6.80e-5 at nu=14 → 1.14e-7 at nu=56 (≈600×); the gate demands a
	// factor of 10 so it fails loudly if the cut ever starts leaking a
	// mesh-independent sliver.
	let residual = |n: usize| {
		let p = pillow_plate(n, n);
		let total = vol(&p.mesh);
		let a = freeform_plane_cut(&p, origin, normal, Keep::Outside, &opts).expect("coarse outside cut");
		let b = freeform_plane_cut(&p, origin, normal, Keep::Inside, &opts).expect("coarse inside cut");
		((vol(&a.mesh) + vol(&b.mesh)) - total).abs() / total
	};
	let (coarse, fine) = (residual(14), residual(56));
	assert!(
		fine * 10.0 < coarse,
		"the conservation residual must converge with tessellation (it is chord error, not a leak): {coarse:.3e} at nu=14 vs {fine:.3e} at nu=56 — want at least a 10× improvement"
	);

	// (3) INDEPENDENT ORACLE — a chart quadrature of the exact patch. The
	// residual is dominated by the OPERAND's own chord sag (its top face is a
	// 28×28 facet grid over a curved patch), so the same oracle is run on the
	// uncut operand: the cut's error must be no worse than the operand's, and
	// both inside the stated 0.5% band.
	let o_hi = oracle_volume(&pillow_patch(), PLATE_BASE_Z, origin, normal, Keep::Outside, 220);
	let o_lo = oracle_volume(&pillow_patch(), PLATE_BASE_Z, origin, normal, Keep::Inside, 220);
	let o_all = o_hi + o_lo;
	let rel_hi = (v_hi - o_hi).abs() / o_hi;
	let rel_lo = (v_lo - o_lo).abs() / o_lo;
	let rel_uncut = (uncut - o_all).abs() / o_all;
	assert!(
		rel_hi < 5e-3 && rel_lo < 5e-3 && rel_hi < rel_uncut + 2e-3 && rel_lo < rel_uncut + 2e-3,
		"cut volumes must track the exact-patch oracle inside the stated 0.5% tessellation band: \
		 hi {v_hi:.5} vs oracle {o_hi:.5} (rel {rel_hi:.5}), lo {v_lo:.5} vs {o_lo:.5} (rel {rel_lo:.5}); \
		 the UNCUT operand's own oracle error is {rel_uncut:.5} — a cut error materially above it would be the boolean's fault"
	);

	// (4) The patch survives EXACTLY: both trimmed halves reference the same
	// control net and knot vectors, bit for bit.
	let src = pillow_patch();
	for (label, face) in [("kept", keep_hi.kept_face.as_ref()), ("dropped", keep_hi.dropped_face.as_ref())] {
		let f = face.unwrap_or_else(|| panic!("{label} half must carry the trimmed patch"));
		let same_net = f
			.surface
			.control
			.iter()
			.flatten()
			.zip(src.control.iter().flatten())
			.all(|(a, b)| a.to_array().map(f64::to_bits) == b.to_array().map(f64::to_bits));
		let same_knots = f.surface.knots_u.iter().zip(src.knots_u.iter()).all(|(a, b)| a.to_bits() == b.to_bits())
			&& f.surface.knots_v.iter().zip(src.knots_v.iter()).all(|(a, b)| a.to_bits() == b.to_bits());
		assert!(
			same_net && same_knots && f.surface.degree_u == 3 && f.surface.degree_v == 3 && f.rings.len() == 1,
			"{label} half must carry the EXACT source patch (control net bit-identical={same_net}, knots bit-identical={same_knots}, \
			 degrees {}/{}, rings {})",
			f.surface.degree_u,
			f.surface.degree_v,
			f.rings.len()
		);
	}

	// (5) The trimmed rings lie ON the patch and on the correct side of the
	// plane (the trim is a real parameter-space split, not a bbox guess).
	let seeds = src.projection_seeds(24);
	let kept_ring = &keep_hi.kept_face.as_ref().unwrap().rings[0];
	let dropped_ring = &keep_hi.dropped_face.as_ref().unwrap().rings[0];
	let off_patch = kept_ring
		.iter()
		.chain(dropped_ring.iter())
		.filter(|p| src.project(&seeds, **p, 1e-9).is_none())
		.count();
	let wrong_side_kept = kept_ring.iter().filter(|p| (**p - origin).dot(normal) < -1e-6).count();
	let wrong_side_dropped = dropped_ring.iter().filter(|p| (**p - origin).dot(normal) > 1e-6).count();
	assert!(
		off_patch == 0 && wrong_side_kept == 0 && wrong_side_dropped == 0,
		"trim rings must lie on the patch and on their own side of the cut: {off_patch} ring points off the patch, \
		 {wrong_side_kept}/{} kept-ring points on the wrong side, {wrong_side_dropped}/{} dropped-ring points on the wrong side",
		kept_ring.len(),
		dropped_ring.len()
	);
}

#[test]
fn the_intersection_curve_is_exact_on_the_patch_and_chord_refined_to_its_stated_tolerance() {
	// The tolerance half of the contract, measured directly: every emitted
	// point is an exact patch evaluation ON the plane, and consecutive chords
	// deviate from the true curve by no more than the requested chord tol.
	let surf = pillow_patch();
	let origin = DVec3::new(15.0, 0.0, 0.0);
	let normal = DVec3::X;
	for &tol in &[0.05_f64, 0.005, 0.0005] {
		let curves = plane_patch_curves(&surf, origin, normal, tol, 64);
		assert_eq!(curves.len(), 1, "a plane through the dome flank must trace exactly one crossing curve at tol {tol}");
		let c = &curves[0];
		assert!(!c.closed, "the flank crossing runs boundary-to-boundary, not as an island");

		// On the plane, to the tracer's Newton tolerance.
		assert!(
			c.plane_dev < 1e-9,
			"every curve point must lie ON the cut plane: max |plane distance| {:.3e} at tol {tol}",
			c.plane_dev
		);
		// On the patch, exactly (each point IS an S(u,v) evaluation — re-prove
		// it by inverting the point back onto the surface).
		let seeds = surf.projection_seeds(24);
		let off = c.points.iter().filter(|p| surf.project(&seeds, **p, 1e-10).is_none()).count();
		assert_eq!(off, 0, "curve points must invert back onto the exact patch (tol {tol}): {off} of {} failed", c.points.len());

		// Chord accuracy: the true curve point at each chord's parameter
		// midpoint must sit within `tol` of the chord.
		let mut worst: f64 = 0.0;
		for w in c.uv.windows(2) {
			let mid_uv = (w[0] + w[1]) * 0.5;
			let ((u0, u1), (v0, v1)) = surf.domain();
			// Slide the parameter midpoint back onto the curve, then measure it
			// against the chord it is supposed to represent.
			let mut uv = mid_uv;
			for _ in 0..24 {
				let (u, v) = (u0 + (u1 - u0) * uv.x, v0 + (v1 - v0) * uv.y);
				let f = (surf.point_at(u, v) - origin).dot(normal);
				if f.abs() < 1e-13 {
					break;
				}
				let (du, dv) = surf.partials(u, v);
				let g = DVec2::new(du.dot(normal) * (u1 - u0), dv.dot(normal) * (v1 - v0));
				if g.length_squared() < 1e-30 {
					break;
				}
				uv = (uv - g * (f / g.length_squared())).clamp(DVec2::ZERO, DVec2::ONE);
			}
			let (u, v) = (u0 + (u1 - u0) * uv.x, v0 + (v1 - v0) * uv.y);
			let m = surf.point_at(u, v);
			let (a, b) = (
				surf.point_at(u0 + (u1 - u0) * w[0].x, v0 + (v1 - v0) * w[0].y),
				surf.point_at(u0 + (u1 - u0) * w[1].x, v0 + (v1 - v0) * w[1].y),
			);
			let chord = b - a;
			let t = if chord.length_squared() > 1e-30 {
				((m - a).dot(chord) / chord.length_squared()).clamp(0.0, 1.0)
			} else {
				0.5
			};
			worst = worst.max((m - (a + chord * t)).length());
		}
		assert!(
			worst <= tol,
			"the polyline must honour its STATED chord tolerance: measured max deviation {worst:.3e} > requested {tol:.3e} \
			 ({} points)",
			c.points.len()
		);
	}
}

#[test]
fn the_cut_result_round_trips_through_step_as_a_true_bspline_face() {
	// Interchange gate: the cut solid exports with its trimmed patch written as
	// a real B_SPLINE_SURFACE_WITH_KNOTS (not facet soup) and re-imports at the
	// same volume — i.e. the boolean's output is still a freeform B-rep.
	use kernel_brep::{export_step_freeform, import_step, solid_from_mesh, volume};

	let plate = pillow_plate(20, 20);
	let cut = freeform_plane_cut(&plate, DVec3::new(15.0, 0.0, 0.0), DVec3::X, Keep::Outside, &FreeformCutOptions::default())
		.expect("planar cut");
	let solid = solid_from_mesh(&cut.mesh);
	let patches = vec![cut.kept_face.clone().expect("kept trimmed patch")];
	let text = export_step_freeform(&solid, &patches, "freeform_cut");
	let has_bspline = text.contains("B_SPLINE_SURFACE_WITH_KNOTS");
	let back = import_step(&text).expect("the exported cut must re-import");
	let (v0, v1) = (volume(&solid).abs(), volume(&back).abs());
	let rel = (v1 - v0).abs() / v0;
	assert!(
		has_bspline && rel < 5e-3,
		"the cut result must survive STEP as a true B-spline face: B_SPLINE_SURFACE_WITH_KNOTS written={has_bspline}, \
		 re-imported volume {v1:.4} vs exported {v0:.4} (rel {rel:.5}, limit 0.005)"
	);
}

// ---------------------------------------------------------------------------
// THE REFUSAL BOUNDARY — everything outside the slice must say so, by name.
// ---------------------------------------------------------------------------

#[test]
fn out_of_slice_operands_refuse_loudly_and_name_the_supported_slice() {
	let plate = pillow_plate(16, 16);
	let opts = FreeformCutOptions::default();
	let half = FreeformTool::HalfSpace { origin: DVec3::new(15.0, 0.0, 0.0), normal: DVec3::X };
	let cyl = kernel_brep::Surface::Cylinder { origin: DVec3::new(20.0, 20.0, -1.0), axis: DVec3::Z, radius: 4.0 };
	let box_solid = kernel_brep::cuboid(DVec3::new(10.0, 10.0, -1.0), DVec3::new(20.0, 20.0, 20.0));
	let two_patch = FreeformSolid { mesh: plate.mesh.clone(), faces: vec![plate.faces[0].clone(), plate.faces[0].clone()] };

	let cases: Vec<(&str, FreeformBoolError, &str)> = vec![
		(
			"union with a half-space",
			try_freeform_boolean(&plate, &half, MeshBoolOp::Union, &opts).expect_err("union with a half-space is unbounded"),
			"union",
		),
		(
			"cylinder drill (freeform ∩ quadric — ledgered, not shipped)",
			try_freeform_boolean(&plate, &FreeformTool::Quadric(&cyl), MeshBoolOp::Difference, &opts)
				.expect_err("a quadric tool is out of the shipped slice"),
			"cylinder",
		),
		(
			"general B-rep solid tool",
			try_freeform_boolean(&plate, &FreeformTool::Solid(&box_solid), MeshBoolOp::Difference, &opts)
				.expect_err("a solid tool is out of the shipped slice"),
			"B-rep solid",
		),
		(
			"freeform ∩ freeform",
			try_freeform_boolean(&plate, &FreeformTool::Freeform(&plate), MeshBoolOp::Intersection, &opts)
				.expect_err("freeform ∩ freeform is out of the shipped slice"),
			"freeform",
		),
		(
			"multi-patch operand",
			try_freeform_boolean(&two_patch, &half, MeshBoolOp::Difference, &opts)
				.expect_err("a two-patch operand is out of the shipped slice"),
			"2 freeform patches",
		),
	];
	for (label, err, needle) in cases {
		let line = err.to_string();
		assert!(
			matches!(err, FreeformBoolError::OutOfScope { .. })
				&& line.contains("freeform boolean support: planar half-space cuts")
				&& line.contains("chord tol")
				&& line.contains(needle)
				&& line.contains("out of scope"),
			"{label} must refuse with a message NAMING the shipped slice and what was asked (looking for {needle:?}): {line:?}"
		);
	}
}

#[test]
fn degenerate_in_slice_cuts_are_refused_rather_than_half_built() {
	let plate = pillow_plate(16, 16);
	let opts = FreeformCutOptions::default();

	// (a) A plane that removes everything: nothing to build.
	let all_gone = freeform_plane_cut(&plate, DVec3::new(-5.0, 0.0, 0.0), DVec3::X, Keep::Inside, &opts)
		.expect_err("a cut that removes the whole solid must refuse");
	// (b) A horizontal plane slicing the dome traces a CLOSED island curve —
	// out of the split's scope, and refused as such rather than mis-trimmed.
	let island = freeform_plane_cut(&plate, DVec3::new(0.0, 0.0, 12.0), DVec3::Z, Keep::Inside, &opts)
		.expect_err("an island (closed) crossing is not in the shipped split");
	assert!(
		matches!(all_gone, FreeformBoolError::DegenerateCut { .. }) && matches!(island, FreeformBoolError::DegenerateCut { .. }),
		"degenerate cuts must be named, not half-built: full-removal → {all_gone}; island crossing → {island}"
	);
	assert!(
		island.to_string().contains("closed curve") || island.to_string().contains("closed"),
		"the island refusal must say WHAT it saw: {island}"
	);
}

#[test]
fn a_plane_that_misses_the_patch_keeps_it_whole_on_the_correct_side() {
	// In-slice but non-splitting: the plate is cut through its base skirt only,
	// below the patch rim — the patch must come through UNSPLIT and attributed
	// to the correct side, with no intersection curve claimed.
	let plate = pillow_plate(20, 20);
	let opts = FreeformCutOptions::default();
	let cut = freeform_plane_cut(&plate, DVec3::new(0.0, 0.0, 3.0), DVec3::Z, Keep::Outside, &opts)
		.expect("a horizontal cut below the patch rim keeps the whole patch");
	assert!(
		cut.kept_face.is_some() && cut.dropped_face.is_none() && cut.curve.is_empty() && cut.mesh.is_watertight(),
		"an untouched patch must stay whole on the kept side with no curve claimed: kept={} dropped={} curve points={} watertight={}",
		cut.kept_face.is_some(),
		cut.dropped_face.is_some(),
		cut.curve.len(),
		cut.mesh.is_watertight()
	);
}

#[test]
fn the_checked_entry_point_routes_difference_and_intersection_to_complementary_halves() {
	// The AI-facing surface: try_freeform_boolean(Difference) keeps the outside
	// of the tool half-space, Intersection keeps the inside, and the two agree
	// with the direct cut calls bit for bit on volume.
	let plate = pillow_plate(20, 20);
	let opts = FreeformCutOptions::default();
	let origin = DVec3::new(18.0, 0.0, 0.0);
	let tool = FreeformTool::HalfSpace { origin, normal: DVec3::X };
	let diff = try_freeform_boolean(&plate, &tool, MeshBoolOp::Difference, &opts).expect("difference with a half-space");
	let inter = try_freeform_boolean(&plate, &tool, MeshBoolOp::Intersection, &opts).expect("intersection with a half-space");
	let direct_hi = freeform_plane_cut(&plate, origin, DVec3::X, Keep::Outside, &opts).expect("direct outside cut");
	let direct_lo = freeform_plane_cut(&plate, origin, DVec3::X, Keep::Inside, &opts).expect("direct inside cut");
	let total = vol(&plate.mesh);
	let (vd, vi) = (vol(&diff.mesh), vol(&inter.mesh));
	assert!(
		vd.to_bits() == vol(&direct_hi.mesh).to_bits()
			&& vi.to_bits() == vol(&direct_lo.mesh).to_bits()
			&& ((vd + vi) - total).abs() / total < 5e-5,
		"checked routing must equal the direct cuts and conserve volume: difference {vd:.6} (direct {:.6}), \
		 intersection {vi:.6} (direct {:.6}), sum vs operand {:.6}/{total:.6}",
		vol(&direct_hi.mesh),
		vol(&direct_lo.mesh),
		vd + vi
	);
	assert!(
		diff.chord_tol > 0.0 && (diff.chord_tol - inter.chord_tol).abs() < 1e-15 && diff.curve_plane_dev < 1e-9,
		"the result must carry its resolved contract: chord tol {} / {}, curve plane deviation {:.3e}",
		diff.chord_tol,
		inter.chord_tol,
		diff.curve_plane_dev
	);
}
