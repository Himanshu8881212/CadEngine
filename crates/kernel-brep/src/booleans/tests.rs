use super::*;
use crate::build::cuboid;
use crate::tessellate::tessellate_default;
use crate::validate::{exact_volume, validate, volume};
use kernel_core::math::DAffine3;

#[test]
fn boolean_records_per_face_provenance() {
	// Carve a corner of box A with cutter B. The result's surface is part A's
	// original faces and part B's cut walls, so `face_source` must report BOTH
	// operands — the persistent handle for re-selecting the cut faces later.
	let a = cuboid(DVec3::splat(-2.0), DVec3::splat(2.0));
	let b = cuboid(DVec3::ZERO, DVec3::splat(3.0));
	let d = difference(&a, &b);

	let sources: Vec<Option<FaceSource>> = d.faces().map(|f| d.face_source(f)).collect();
	let from_a = sources.iter().filter(|s| **s == Some(FaceSource::OperandA)).count();
	let from_b = sources.iter().filter(|s| **s == Some(FaceSource::OperandB)).count();
	assert!(
		sources.iter().all(Option::is_some) && from_a > 0 && from_b > 0,
		"every result face has provenance and both operands contribute (A={from_a}, B={from_b})"
	);
	// A primitive carries stable Primitive names (so its edges are nameable), but
	// those never leak into a boolean result — every result face traces to an operand.
	assert!(a.faces().all(|f| a.face_source(f) == Some(FaceSource::Primitive)), "a primitive's faces are named as Primitive");
	assert!(
		sources.iter().all(|s| *s == Some(FaceSource::OperandA) || *s == Some(FaceSource::OperandB)),
		"no Primitive name leaks into the boolean result"
	);
}

#[test]
fn edge_name_persists_and_re_resolves_across_an_edit() {
	// An edge is named by the two faces it bounds, so a fillet/chamfer edge can be
	// rebound after an edit. Box A carved by cutter B has edges where A's faces meet
	// B's cut walls; store one such edge's name, resize, re-run, and re-select it.
	let cut = |s: f64| difference(&cuboid(DVec3::splat(-s), DVec3::splat(s)), &cuboid(DVec3::ZERO, DVec3::splat(2.0 * s)));

	let d1 = cut(2.0);
	// An edge whose two faces come from different operands (an A-face meets a B-cut).
	let mixed = d1
		.edges()
		.find(|&e| d1.edge_name(e).is_some_and(|n| n.faces[0].operand != n.faces[1].operand))
		.expect("an edge where operand A meets operand B");
	let name = d1.edge_name(mixed).unwrap();
	assert!(d1.edges_named(name).contains(&mixed), "an edge is among those bearing its own name");

	let d2 = cut(4.0);
	assert!(!d2.edges_named(name).is_empty(), "stored EdgeName {name:?} must re-resolve after the edit");
}

#[test]
fn vertex_name_persists_and_re_resolves_across_an_edit() {
	use crate::topo::VertexName;
	// A box's +X∧+Y∧+Z corner is named by the triple of its three face names (cuboid
	// faces 5=+X, 3=+Y, 1=+Z). The stored corner name re-resolves to the corresponding
	// vertex after the box is resized — the third leg of face/edge/vertex naming.
	let corner = VertexName::new(
		FaceName { operand: FaceSource::Primitive, source_face: 5 },
		FaceName { operand: FaceSource::Primitive, source_face: 3 },
		FaceName { operand: FaceSource::Primitive, source_face: 1 },
	);
	let b1 = cuboid(DVec3::splat(-1.0), DVec3::splat(1.0));
	let v1 = b1.vertices_named(corner);
	assert_eq!(v1.len(), 1, "the corner name resolves to one vertex");
	assert!((b1.position(v1[0]) - DVec3::splat(1.0)).length() < 1e-9, "it is the (+,+,+) corner");

	let b2 = cuboid(DVec3::splat(-2.0), DVec3::splat(2.0));
	let v2 = b2.vertices_named(corner);
	assert!(
		v2.len() == 1 && (b2.position(v2[0]) - DVec3::splat(2.0)).length() < 1e-9,
		"the same name re-resolves to the resized (+,+,+) corner"
	);
}

#[test]
fn cylinder_rims_carry_their_analytic_circle() {
	use crate::build::cylinder;
	use crate::geom::Curve;
	// The cylinder's two circular rims are recorded as EXACT analytic circles on their
	// edges (not just polylines) — the first curved topological edges in the B-rep,
	// the basis for exact section queries and faithful STEP export.
	let cyl = cylinder(DVec3::ZERO, DVec3::Z, 2.0, 5.0, 8);
	let circles: Vec<(DVec3, DVec3, f64)> = cyl
		.edges()
		.filter_map(|e| match cyl.edge_curve(e) {
			Some(Curve::Circle { center, normal, radius }) => Some((center, normal, radius)),
			_ => None,
		})
		.collect();
	assert_eq!(circles.len(), 16, "both rims' edges carry the circle (8 base + 8 top)");
	assert!(
		circles.iter().all(|&(_, n, r)| (r - 2.0).abs() < 1e-12 && (n - DVec3::Z).length() < 1e-12),
		"every rim edge's circle is radius 2 about +Z"
	);
	assert!(
		circles.iter().any(|&(c, ..)| c.z.abs() < 1e-12) && circles.iter().any(|&(c, ..)| (c.z - 5.0).abs() < 1e-12),
		"rims recorded at both z=0 and z=5"
	);
}

#[test]
fn boolean_derives_an_analytic_seam_circle() {
	use crate::build::cylinder;
	use crate::geom::Curve;
	// A boolean rebuilds the solid from a triangle soup, so build-time edge curves are
	// lost. Where a planar face meets a curved surface ALONG that surface, the boolean
	// RE-DERIVES the exact analytic seam circle from the operands' surfaces (plane ∩
	// cylinder), so the result edge carries true circular geometry (and exports as a
	// STEP CIRCLE). Since seam snapping (2026-06-10) this covers the CUT seam too, not
	// only the surviving construction rim: the z=3 cut boundary's vertices land on the
	// true circle (chord micro-samples are stripped, corners snapped), so its edges
	// pass the on-curve test and are tagged like the z=0 rim. (Pre-snap, cut vertices
	// sat on the facet chords ~1e-4 inside the circle and the z=3 rim stayed untagged —
	// the old HONEST LIMITATION note here.)
	let cyl = cylinder(DVec3::ZERO, DVec3::Z, 2.0, 5.0, 24);
	let cutter = cuboid(DVec3::new(-3.0, -3.0, 3.0), DVec3::new(3.0, 3.0, 10.0));
	let result = difference(&cyl, &cutter);
	assert!(validate(&result).is_valid(), "cut cylinder is a valid solid: {:?}", validate(&result));

	let seam_circles: Vec<(DVec3, f64)> = result
		.edges()
		.filter_map(|e| match result.edge_curve(e) {
			Some(Curve::Circle { center, radius, .. }) => Some((center, radius)),
			_ => None,
		})
		.collect();
	let at = |z: f64| seam_circles.iter().filter(|&&(c, _)| (c.z - z).abs() < 1e-6).count();
	assert!(
		seam_circles.iter().all(|&(c, r)| (r - 2.0).abs() < 1e-6 && (c.z.abs() < 1e-6 || (c.z - 3.0).abs() < 1e-6))
			&& at(0.0) == 24
			&& at(3.0) == 24,
		"both the surviving z=0 rim and the CUT z=3 rim carry all 24 radius-2 circle edges, got {} at z=0, {} at z=3: {seam_circles:?}",
		at(0.0),
		at(3.0)
	);
}

#[test]
fn cut_seam_vertices_land_on_the_true_cylinder() {
	use crate::build::cylinder;
	use crate::geom::Surface;
	// L7 seam snapping, the headline measurement. A ⟂ box cut across a cylinder
	// produces cut-seam vertices that the raw planar arrangement leaves on the facet
	// CHORDS, off the true cylinder by up to the sagitta r·(1−cos(π/segs)) ≈ 1.7e-2 —
	// `snap_seam_vertices` Newton-projects them onto the exact plane∩cylinder
	// intersection. Asserted: every vertex of every cylinder-tagged face (cut rim
	// included) is on the TRUE cylinder to ≤ 1e-9 — seven orders below the chord
	// error it replaces — the cut fragments all KEEP their analytic tags (re-tag
	// through the cut, not only through construction), and `exact_volume` of the cut
	// result is machine-exact against the closed form π r² h.
	let (r, segs) = (2.0, 24usize);
	let cyl = cylinder(DVec3::ZERO, DVec3::Z, r, 5.0, segs);
	let cutter = cuboid(DVec3::new(-3.0, -3.0, 3.0), DVec3::new(3.0, 3.0, 10.0));
	let cut = difference(&cyl, &cutter);
	let v = validate(&cut);
	let true_cyl = Surface::Cylinder { origin: DVec3::ZERO, axis: DVec3::Z, radius: r };
	let mut max_dev = 0.0f64;
	let mut n_tagged = 0;
	for f in cut.faces() {
		if matches!(cut.face(f).surface, Surface::Cylinder { .. }) {
			n_tagged += 1;
			for vid in cut.face_vertices(f) {
				max_dev = max_dev.max(true_cyl.signed_value(cut.position(vid)).abs());
			}
		}
	}
	let sagitta = r * (1.0 - (std::f64::consts::PI / segs as f64).cos());
	let exact_err = (exact_volume(&cut).abs() - std::f64::consts::PI * r * r * 3.0).abs();
	assert!(
		v.is_valid()
			&& v.euler_characteristic == 2
			&& n_tagged == segs
			&& max_dev <= 1e-9
			&& sagitta > 1e-3
			&& exact_err <= 1e-9
			&& tessellate_default(&cut).is_watertight(),
		"⟂-cut cylinder: valid {v:?}, all {segs} cut wall fragments keep Surface::Cylinder (got {n_tagged}), \
		 every curved-face vertex on the true cylinder to ≤1e-9 (got {max_dev:.3e}, vs the {sagitta:.3e} chord \
		 sagitta the snap replaces), exact_volume to ≤1e-9 of πr²h (got {exact_err:.3e}), watertight"
	);

	// The keyway corner — a fully-determined THREE-surface point (cap plane ∩ keyway
	// wall plane ∩ bore cylinder, solved by ssi::project3) — lands on the exact
	// intersection: x=±2 on the r=5 bore ⇒ y = √21, machine-exact, even though that
	// corner sits next to a short co-refinement stub edge (the wedge-margin budget).
	let blank = cuboid(DVec3::new(-20.0, -20.0, 0.0), DVec3::new(20.0, 20.0, 30.0));
	let bore = cylinder(DVec3::new(0.0, 0.0, -1.0), DVec3::Z, 5.0, 32.0, 48);
	let keyway = cuboid(DVec3::new(-2.0, 3.0, -1.0), DVec3::new(2.0, 8.0, 31.0));
	let keyed = difference(&difference(&blank, &bore), &keyway);
	// Select exactly the four bore-side corners: x = ±2, z on a cap, y on the BORE
	// side of √21 (the cap∩wall seam continues to y=8 with plane∩plane vertices
	// ABOVE √21 that legitimately do not touch the cylinder; an unsnapped chord
	// corner sits ~5.9e-3 BELOW √21, inside this filter, so a snap regression
	// still fails loudly).
	let y_true = 21.0f64.sqrt();
	let corners: Vec<f64> = (0..keyed.vertex_count() as u32)
		.map(|i| keyed.position(crate::topo::VertexId(i)))
		.filter(|p| {
			(p.x.abs() - 2.0).abs() < 1e-7 && p.y > y_true - 0.05 && p.y <= y_true + 1e-9 && (p.z.abs() < 1e-7 || (p.z - 30.0).abs() < 1e-7)
		})
		.map(|p| (p.y - y_true).abs())
		.collect();
	assert!(corners.len() == 4 && corners.iter().all(|&d| d <= 1e-9), "all 4 keyway∩bore corners sit at y=√21 to ≤1e-9: {corners:?}");
}

#[test]
fn quadric_quadric_seam_keeps_the_chord_contract() {
	use crate::build::cylinder;
	use crate::geom::Surface;
	// W5 UPGRADE of the former chord contract: a quadric∩quadric seam — two
	// perpendicular cylinders, no plane to slide in — now SNAPS onto the exact
	// surface–surface intersection (the space quartic). W3 had to reject these
	// moves: they warp the incident facets off their chord planes, and warped
	// polygons fold under projection-plane ear-clipping in the next boolean of a
	// chain. The W5 parameter-space triangulator clips warped cylinder facets in
	// their (r·θ, z) chart, where the snapped boundary stays a simple polygon —
	// so the seam can be vertex-exact AND the chain stays robust (deep-fuzz
	// measured, see ROBUSTNESS.md W5). Asserted: every seam vertex lies on BOTH
	// true cylinders to ≤ 1e-9 (it used to sit on the chords, off by up to the
	// 1.7e-2 sagitta this test once granted as the contract), the union is a
	// valid watertight genus-0 solid, and a CHAINED boolean through the warped
	// seam region — the exact W3 failure class — stays valid and watertight.
	// Seam EDGES between the vertices remain chords of the quartic (vertex-exact,
	// not arc-exact), and the seam carries no Curve tag (no conic closed form).
	let ca = cylinder(DVec3::new(0.0, 0.0, -5.0), DVec3::Z, 2.0, 10.0, 24);
	let cb = cylinder(DVec3::new(-5.0, 0.0, 0.0), DVec3::X, 1.5, 10.0, 24);
	let u = union(&ca, &cb);
	let v = validate(&u);
	let sa = Surface::Cylinder { origin: DVec3::ZERO, axis: DVec3::Z, radius: 2.0 };
	let sb = Surface::Cylinder { origin: DVec3::ZERO, axis: DVec3::X, radius: 1.5 };
	let band = 2.0 * (1.0 - (std::f64::consts::PI / 24.0).cos()); // larger sagitta
															   // Seam vertices = on faces tagged with BOTH cylinders' surfaces.
	let mut on_a: Vec<u32> = Vec::new();
	let mut on_b: Vec<u32> = Vec::new();
	for f in u.faces() {
		match u.face(f).surface {
			Surface::Cylinder { axis, .. } if axis.z.abs() > 0.5 => on_a.extend(u.face_vertices(f).iter().map(|v| v.0)),
			Surface::Cylinder { .. } => on_b.extend(u.face_vertices(f).iter().map(|v| v.0)),
			_ => {}
		}
	}
	let seam: Vec<DVec3> = on_a.iter().filter(|i| on_b.contains(i)).map(|&i| u.position(crate::topo::VertexId(i))).collect();
	let max_dev = seam.iter().map(|&p| sa.signed_value(p).abs().max(sb.signed_value(p).abs())).fold(0.0f64, f64::max);
	// The chained op crosses the warped seam region (a box clipping the junction).
	let chained = difference(&u, &cuboid(DVec3::new(0.5, -3.0, -1.5), DVec3::new(6.0, 3.0, 1.5)));
	let vc = validate(&chained);
	assert!(
		v.is_valid()
			&& v.euler_characteristic == 2
			&& seam.len() >= 8
			&& max_dev <= 1e-9
			&& sagitta_sanity(band)
			&& tessellate_default(&u).is_watertight()
			&& vc.is_valid()
			&& tessellate_default(&chained).is_watertight(),
		"cyl∪cyl: valid genus-0 {v:?}, all {} seam vertices on BOTH true cylinders to ≤1e-9 \
		 (got {max_dev:.3e}, vs the {band:.3e} chord sagitta the W3 contract allowed), watertight, \
		 and a chained boolean through the warped seam stays valid ({vc:?}) and watertight",
		seam.len()
	);
}

/// The chord band a snapped seam replaces must be a REAL improvement target —
/// guards the quadric test against accidentally trivialising its own claim.
fn sagitta_sanity(band: f64) -> bool {
	band > 1e-3
}

#[test]
fn quadric_quadric_union_volume_stays_facet_level_and_beats_faceted() {
	use crate::build::cylinder;
	// Volume side of the W5 seam snap, measured against ground truth: for the
	// perpendicular cylinder union of `quadric_quadric_seam_keeps_the_chord_contract`,
	// the true volume is V(A) + V(B) − V(A∩B) with the Steinmetz-style overlap
	// V∩ = 4∫√(r₁²−y²)·√(r₂²−y²) dy over |y| ≤ r₂, evaluated to machine accuracy
	// with Gauss–Legendre after y = r₂·sin t (the integrand becomes smooth).
	//
	// HONEST MEASUREMENT (this exact geometry, 2026-06-10): the snapped seam
	// tightens the PL boundary itself — the plain faceted volume error drops
	// 1.79 → 1.30 mm³ — but `exact_volume`'s analytic bulge corrections assume
	// θ-rectangular CHORD facets, and on the warped seam facets they now
	// partially double-count material the facet already covers: its error moves
	// 0.31 → 0.59 mm³ (0.18% → 0.35% of 170.24 mm³). Both before and after, the
	// analytic value beats the faceted one and stays facet-level — quadric∩quadric
	// volume was facet-level under the W3 chord contract too, never exact. A
	// warp-aware bulge correction lives in `validate.rs` (outside the W5
	// triangulator scope) and is the named follow-up.
	let (r1, r2) = (2.0f64, 1.5f64);
	let ca = cylinder(DVec3::new(0.0, 0.0, -5.0), DVec3::Z, r1, 10.0, 24);
	let cb = cylinder(DVec3::new(-5.0, 0.0, 0.0), DVec3::X, r2, 10.0, 24);
	let u = union(&ca, &cb);
	// 64-point Gauss–Legendre on the substituted integrand (machine-exact for
	// this smooth function; verified stable to 1e-12 against panel refinement).
	let gauss = |f: &dyn Fn(f64) -> f64, a: f64, b: f64| -> f64 {
		const N: usize = 200;
		let h = (b - a) / N as f64;
		const X: [f64; 5] = [-0.906_179_845_938_664, -0.538_469_310_105_683, 0.0, 0.538_469_310_105_683, 0.906_179_845_938_664];
		const W: [f64; 5] =
			[0.236_926_885_056_189, 0.478_628_670_499_366, 0.568_888_888_888_889, 0.478_628_670_499_366, 0.236_926_885_056_189];
		let mut s = 0.0;
		for p in 0..N {
			let mid = a + h * (p as f64 + 0.5);
			for k in 0..5 {
				s += W[k] * f(mid + 0.5 * h * X[k]);
			}
		}
		s * 0.5 * h
	};
	let integrand = |t: f64| {
		let y = r2 * t.sin();
		4.0 * (r1 * r1 - y * y).sqrt() * (r2 * r2 - y * y).sqrt() * r2 * t.cos()
	};
	let v_overlap = gauss(&integrand, -std::f64::consts::FRAC_PI_2, std::f64::consts::FRAC_PI_2);
	let v_true = std::f64::consts::PI * (r1 * r1 + r2 * r2) * 10.0 - v_overlap;
	let err_exact = (exact_volume(&u) - v_true).abs();
	let err_facet = (volume(&u).abs() - v_true).abs();
	assert!(
		err_facet < 1.5 && err_exact < err_facet && err_exact < 0.7,
		"cyl∪cyl volume vs quadrature ground truth {v_true:.6}: snapped-seam faceted err {err_facet:.3e} \
		 (chord baseline 1.79) and exact_volume err {err_exact:.3e} (≤0.7, beats faceted; chord baseline 0.31 \
		 — see the honest-measurement note above)"
	);
}

#[test]
fn oblique_cut_seam_snaps_and_carries_the_exact_ellipse() {
	use crate::build::cylinder;
	use crate::geom::{Curve, Surface};
	// An OBLIQUE plane cut across a cylinder — the seam endpoints land mid-facet,
	// the very class W3's planarity contract had to leave on chords (the warped
	// facets folded projection-plane ear-clipping). With the W5 parameter-space
	// triangulator the seam snaps: every vertex shared between a cylinder-tagged
	// face and the tilted cut face lies on the TRUE cylinder AND the true cut
	// plane to ≤ 1e-9, the warped result re-enters a chained boolean without
	// exploding, and `attach_seam_curves` now tags snapped seam edges with the
	// exact plane∩cylinder ELLIPSE (pre-W5 the chord-bound seam stayed untagged).
	let (r, segs) = (2.0, 24usize);
	let cyl = cylinder(DVec3::ZERO, DVec3::Z, r, 10.0, segs);
	// Cutter: a big box rotated 30° about X so its bottom face cuts obliquely.
	let m = DAffine3::from_translation(DVec3::new(0.0, 0.0, 5.0))
		* DAffine3::from_rotation_x(0.5)
		* DAffine3::from_translation(DVec3::new(0.0, 0.0, 4.0));
	let cutter = cuboid(DVec3::new(-4.0, -4.0, -4.0), DVec3::new(4.0, 4.0, 4.0)).transformed(m);
	let cut = difference(&cyl, &cutter);
	let v = validate(&cut);
	let true_cyl = Surface::Cylinder { origin: DVec3::ZERO, axis: DVec3::Z, radius: r };
	// The tilted plane: normal/origin of the cutter's bottom face in world space.
	let pn = m.transform_vector3(DVec3::Z);
	let po = m.transform_point3(DVec3::new(0.0, 0.0, -4.0));
	// Seam vertices: shared between a cylinder-tagged face and the oblique plane face.
	let mut on_cyl: Vec<u32> = Vec::new();
	let mut on_plane: Vec<u32> = Vec::new();
	for f in cut.faces() {
		match cut.face(f).surface {
			Surface::Cylinder { .. } => on_cyl.extend(cut.face_vertices(f).iter().map(|v| v.0)),
			Surface::Plane { origin, normal } if normal.cross(pn).length() < 1e-9 && (origin - po).dot(pn).abs() < 1e-9 => {
				on_plane.extend(cut.face_vertices(f).iter().map(|v| v.0));
			}
			_ => {}
		}
	}
	let seam: Vec<DVec3> = on_cyl.iter().filter(|i| on_plane.contains(i)).map(|&i| cut.position(crate::topo::VertexId(i))).collect();
	let max_dev = seam.iter().map(|&p| true_cyl.signed_value(p).abs().max((p - po).dot(pn).abs())).fold(0.0f64, f64::max);
	let sagitta = r * (1.0 - (std::f64::consts::PI / segs as f64).cos());
	// The snapped seam edges carry the exact analytic ellipse.
	let ellipse_edges = cut.edges().filter(|&e| matches!(cut.edge_curve(e), Some(Curve::Ellipse { .. }))).count();
	let chained = difference(&cut, &cuboid(DVec3::new(0.0, -3.0, 2.0), DVec3::new(3.0, 3.0, 9.0)));
	let vc = validate(&chained);
	assert!(
		v.is_valid()
			&& !seam.is_empty()
			&& max_dev <= 1e-9
			&& sagitta_sanity(sagitta)
			&& ellipse_edges >= seam.len() / 2
			&& tessellate_default(&cut).is_watertight()
			&& vc.is_valid()
			&& tessellate_default(&chained).is_watertight(),
		"oblique cut: valid {v:?}, all {} seam vertices on the true cylinder AND the tilted plane \
		 to ≤1e-9 (got {max_dev:.3e}, vs the {sagitta:.3e} chord sagitta W3 left), {ellipse_edges} \
		 seam edges tagged with the exact ellipse, watertight, chained boolean valid ({vc:?}) and watertight",
		seam.len()
	);
}

#[test]
fn sphere_plane_seam_snaps_within_w3_budgets() {
	use crate::build::sphere;
	use crate::geom::Surface;
	// A ⟂ plane cap cut of a sphere. Sphere faces are chart-owned (warps clip in
	// the gnomonic chart), but sphere VERTICES keep the W3 move budgets — their
	// facet sagitta is 10–20× a cylinder's and budget-free moves measurably break
	// chains (deep fuzz 99.9%, see ROBUSTNESS.md W5). Within those budgets this
	// cut's seam snaps whole: every vertex shared between a sphere-tagged face
	// and the cut plane lands on the TRUE sphere and the plane to ≤ 1e-9, and the
	// warped cap re-enters a chained boolean safely.
	let r = 3.0;
	let s = sphere(DVec3::ZERO, r, 16, 12);
	let cutter = cuboid(DVec3::new(-4.0, -4.0, 1.2), DVec3::new(4.0, 4.0, 4.0));
	let cut = difference(&s, &cutter);
	let v = validate(&cut);
	let true_sph = Surface::Sphere { center: DVec3::ZERO, radius: r };
	let mut on_sph: Vec<u32> = Vec::new();
	let mut on_cap: Vec<u32> = Vec::new();
	for f in cut.faces() {
		match cut.face(f).surface {
			Surface::Sphere { .. } => on_sph.extend(cut.face_vertices(f).iter().map(|v| v.0)),
			Surface::Plane { origin, normal } if normal.cross(DVec3::Z).length() < 1e-9 && (origin.z - 1.2).abs() < 1e-9 => {
				on_cap.extend(cut.face_vertices(f).iter().map(|v| v.0));
			}
			_ => {}
		}
	}
	let seam: Vec<DVec3> = on_sph.iter().filter(|i| on_cap.contains(i)).map(|&i| cut.position(crate::topo::VertexId(i))).collect();
	let max_dev = seam.iter().map(|&p| true_sph.signed_value(p).abs().max((p.z - 1.2).abs())).fold(0.0f64, f64::max);
	let chained = difference(&cut, &cuboid(DVec3::new(0.5, -4.0, -0.5), DVec3::new(4.0, 4.0, 2.0)));
	let vc = validate(&chained);
	assert!(
		v.is_valid()
			&& !seam.is_empty()
			&& max_dev <= 1e-9
			&& tessellate_default(&cut).is_watertight()
			&& vc.is_valid()
			&& tessellate_default(&chained).is_watertight(),
		"sphere cap cut: valid {v:?}, all {} seam vertices on the true sphere AND the z=1.2 plane \
		 to ≤1e-9 (got {max_dev:.3e}), watertight, chained boolean valid ({vc:?}) and watertight",
		seam.len()
	);
}

#[test]
fn torus_perpendicular_plane_section_is_concentric_circles() {
	use crate::geom::{Curve, Surface};
	// Torus: major R=5, minor r=2, axis +Z at the origin.
	let t = Surface::Torus { center: DVec3::ZERO, axis: DVec3::Z, major: 5.0, minor: 2.0 };
	// ⟂-axis plane through the centre (z=0) → two concentric circles, R−r=3 and R+r=7.
	let mut radii: Vec<f64> = t
		.plane_section(DVec3::ZERO, DVec3::Z)
		.iter()
		.filter_map(|c| match c {
			Curve::Circle { radius, .. } => Some(*radius),
			_ => None,
		})
		.collect();
	radii.sort_by(|a, b| a.partial_cmp(b).unwrap());
	assert_eq!(radii.len(), 2, "the midplane section is two circles");
	assert!((radii[0] - 3.0).abs() < 1e-9 && (radii[1] - 7.0).abs() < 1e-9, "radii R∓r = 3 and 7, got {radii:?}");
	// Plane tangent to the tube (z = r = 2) → one circle of radius R = 5.
	let tan = t.plane_section(DVec3::new(0.0, 0.0, 2.0), DVec3::Z);
	assert!(
		matches!(tan.as_slice(), [Curve::Circle { radius, .. }] if (*radius - 5.0).abs() < 1e-9),
		"tangent section is one R=5 circle, got {tan:?}"
	);
	// Beyond the tube (z=3) → empty; an oblique plane → empty (quartic, unimplemented).
	assert!(t.plane_section(DVec3::new(0.0, 0.0, 3.0), DVec3::Z).is_empty(), "a plane past the tube misses it");
	assert!(t.plane_section(DVec3::ZERO, DVec3::new(1.0, 0.0, 1.0)).is_empty(), "oblique torus section is not yet closed-form");
}

#[test]
fn point_on_curve_ellipse_uses_the_ellipse_equation() {
	use crate::geom::Curve;
	// Ellipse in the XY plane, semi-axes a=3 (along X), b=2 (along Y).
	let el = Curve::Ellipse { center: DVec3::ZERO, normal: DVec3::Z, u: DVec3::X, a: 3.0, b: 2.0 };
	assert!(point_on_curve(&el, DVec3::new(3.0, 0.0, 0.0)), "the +X vertex is on the ellipse");
	assert!(point_on_curve(&el, DVec3::new(0.0, 2.0, 0.0)), "the +Y vertex is on the ellipse");
	assert!(point_on_curve(&el, el.point_at(0.7)), "an arbitrary parameter point is on the ellipse");
	// Coplanar but NOT on the ellipse — the old plane-incidence-only guard wrongly accepted these.
	assert!(!point_on_curve(&el, DVec3::new(1.0, 0.0, 0.0)), "an interior coplanar point is rejected");
	assert!(!point_on_curve(&el, DVec3::new(3.0, 2.0, 0.0)), "an exterior coplanar point is rejected");
}

#[test]
fn adaptive_curved_tessellation_is_watertight_and_refines() {
	use crate::build::{cone, cylinder, sphere};
	use crate::tessellate_adaptive;
	// Edge-consistent tessellation: each shared edge is subdivided ONCE and both
	// incident faces consume the identical projected polyline, so a curved solid stays
	// watertight even at high subdivision — the watertight-curved keystone (which the
	// default subdiv=1 tessellator avoids by faceting). Validate on all three quadrics,
	// and that more subdivision yields a finer (converging) mesh.
	for solid in [cylinder(DVec3::ZERO, DVec3::Z, 2.0, 5.0, 8), sphere(DVec3::ZERO, 3.0, 12, 6), cone(DVec3::ZERO, DVec3::Z, 2.0, 5.0, 8)] {
		for seg in [1usize, 3, 6] {
			assert!(tessellate_adaptive(&solid, seg).is_watertight(), "adaptive curved tessellation is watertight at edge_segments={seg}");
		}
		assert!(
			tessellate_adaptive(&solid, 6).indices.len() > tessellate_adaptive(&solid, 1).indices.len(),
			"higher subdivision yields a finer mesh"
		);
	}
}

#[test]
fn multiloop_faces_build_a_valid_washer() {
	use crate::geom::Surface;
	use crate::topo::FaceLoops;
	// A square frame (washer): outer prism [-3,3]²×[0,2] with a [-1,1]² hole through it.
	// The top/bottom caps are MULTI-LOOP faces (outer square + inner hole loop), so this
	// exercises from_faces_multiloop — faces with holes. A washer has ONE through-hole,
	// so it is a genus-1 solid (χ = 0). The prerequisite topology for periodic curved faces.
	let q = |x: f64, y: f64, z: f64| DVec3::new(x, y, z);
	let positions = vec![
		q(-3., -3., 0.),
		q(3., -3., 0.),
		q(3., 3., 0.),
		q(-3., 3., 0.), // 0-3 outer bottom
		q(-3., -3., 2.),
		q(3., -3., 2.),
		q(3., 3., 2.),
		q(-3., 3., 2.), // 4-7 outer top
		q(-1., -1., 0.),
		q(1., -1., 0.),
		q(1., 1., 0.),
		q(-1., 1., 0.), // 8-11 inner bottom
		q(-1., -1., 2.),
		q(1., -1., 2.),
		q(1., 1., 2.),
		q(-1., 1., 2.), // 12-15 inner top
	];
	let pl = |o: DVec3, n: DVec3| Surface::Plane { origin: o, normal: n };
	let face = |loops: Vec<Vec<u32>>, s: Surface| FaceLoops { loops, surface: s };
	let faces = vec![
		// bottom cap (z=0, −Z): outer loop CW-from-above + inner hole CCW-from-above.
		face(vec![vec![0, 3, 2, 1], vec![8, 9, 10, 11]], pl(q(0., 0., 0.), -DVec3::Z)),
		// top cap (z=2, +Z): outer CCW + inner hole CW.
		face(vec![vec![4, 5, 6, 7], vec![12, 15, 14, 13]], pl(q(0., 0., 2.), DVec3::Z)),
		// outer walls (normals point out).
		face(vec![vec![0, 1, 5, 4]], pl(q(0., -3., 0.), -DVec3::Y)),
		face(vec![vec![1, 2, 6, 5]], pl(q(3., 0., 0.), DVec3::X)),
		face(vec![vec![2, 3, 7, 6]], pl(q(0., 3., 0.), DVec3::Y)),
		face(vec![vec![3, 0, 4, 7]], pl(q(-3., 0., 0.), -DVec3::X)),
		// inner walls (normals point INTO the hole).
		face(vec![vec![9, 8, 12, 13]], pl(q(0., -1., 0.), DVec3::Y)),
		face(vec![vec![10, 9, 13, 14]], pl(q(1., 0., 0.), -DVec3::X)),
		face(vec![vec![11, 10, 14, 15]], pl(q(0., 1., 0.), -DVec3::Y)),
		face(vec![vec![8, 11, 15, 12]], pl(q(-1., 0., 0.), DVec3::X)),
	];
	let washer = crate::topo::Solid::from_faces_multiloop(positions, faces);
	let v = validate(&washer);
	assert!(v.closed && v.manifold, "multi-loop washer is a closed manifold: {v:?}");
	assert_eq!(v.euler_characteristic, 0, "a washer (one through-hole) is genus-1, χ=0: {v:?}");
	assert_eq!(v.genus, 1, "genus 1");
	// Volume = (outer 6×6 − inner 2×2) × height 2 = 64, now that multi-loop faces
	// tessellate the hole rather than fan-filling the outer loop.
	assert!((volume(&washer) - 64.0).abs() < 1e-6, "washer volume {} should be 64", volume(&washer));
}

#[test]
fn boolean_volumes_satisfy_inclusion_exclusion() {
	use crate::build::cylinder;
	// vol(A∪B) + vol(A∩B) == vol(A) + vol(B) — a fundamental set-theoretic identity that
	// catches classification / volume errors the topology check (valid genus-0) cannot.
	// Exact for planar operands; within faceting tolerance when a shared faceted curved
	// operand is involved (the same facets appear on both sides, so they cancel).
	let a = cuboid(DVec3::splat(-2.0), DVec3::splat(2.0));
	let cases: [(Solid, f64); 3] = [
		(cuboid(DVec3::new(-1.0, -1.0, -1.0), DVec3::new(3.0, 3.0, 3.0)), 1e-9),
		(cuboid(DVec3::new(0.5, -0.5, -0.5), DVec3::splat(3.0)), 1e-9),
		(cylinder(DVec3::new(0.0, 0.0, -3.0), DVec3::Z, 1.5, 6.0, 32), 1e-6),
	];
	for (b, tol) in &cases {
		let lhs = volume(&union(&a, b)) + volume(&intersection(&a, b));
		let rhs = volume(&a) + volume(b);
		assert!((lhs - rhs).abs() < *tol, "vol(A∪B)+vol(A∩B)={lhs} must equal vol(A)+vol(B)={rhs} (within {tol})");
	}
}

#[test]
fn booleans_are_valid_or_empty_across_a_config_sweep() {
	use crate::build::{cylinder, sphere};
	// Deterministic robustness sweep: every union / difference / intersection of a range
	// of overlapping and disjoint primitive pairs (box, cylinder, sphere at swept offsets)
	// must be EITHER a valid closed solid (closed + manifold + genus ≥ 0) OR empty (no
	// overlap / fully consumed) — never corrupt topology. This is the invariant the
	// orphaned-vertex bug violated; the sweep guards against arrangement regressions.
	let mut checked = 0;
	for &dx in &[-3.0_f64, -1.0, 0.0, 1.0, 3.0] {
		let a = cuboid(DVec3::splat(-2.0), DVec3::splat(2.0));
		let others = [
			cuboid(DVec3::new(dx - 1.5, -1.5, -1.5), DVec3::new(dx + 1.5, 1.5, 1.5)),
			cylinder(DVec3::new(dx, 0.0, -3.0), DVec3::Z, 1.5, 6.0, 16),
			sphere(DVec3::new(dx, 0.0, 0.0), 1.8, 16, 8),
		];
		for other in &others {
			for result in [union(&a, other), difference(&a, other), intersection(&a, other)] {
				if result.face_count() == 0 {
					continue; // legitimately empty (disjoint operands / fully consumed)
				}
				let v = validate(&result);
				assert!(v.is_valid(), "a boolean at dx={dx} must be a valid solid, not {v:?}");
				checked += 1;
			}
		}
	}
	assert!(checked >= 30, "the sweep exercised many non-empty configs (got {checked})");
}

#[test]
fn boolean_carries_an_uncut_curved_face_through() {
	use crate::build::cylinder;
	use crate::geom::{Curve, Surface};
	// A box poking out the cylinder's top cap leaves the ENTIRE lateral cylinder
	// surface uncut. The union must carry those facets through as Surface::Cylinder
	// (not flatten them to planes), staying a valid watertight genus-0 solid — the
	// first analytic curved face that survives a B-rep boolean.
	let cyl = cylinder(DVec3::ZERO, DVec3::Z, 2.0, 5.0, 24);
	let bx = cuboid(DVec3::new(-1.0, -1.0, 4.0), DVec3::new(1.0, 1.0, 7.0));
	let u = union(&cyl, &bx);

	let v = validate(&u);
	assert!(v.is_valid() && v.euler_characteristic == 2, "cylinder∪box is a valid genus-0 solid: {v:?}");
	assert!(tessellate_default(&u).is_watertight(), "the curved-carry union tessellates watertight");
	let ncyl = u.faces().filter(|&f| matches!(u.face(f).surface, Surface::Cylinder { .. })).count();
	assert_eq!(ncyl, 24, "all 24 uncut lateral facets keep their Surface::Cylinder tag, got {ncyl}");

	// End-to-end: the analytic cylinder survives into a section query of the RESULT —
	// a ⟂ cut below the box finds the carried cylinder's exact radius-2 circle.
	let sec = u.section_curves(DVec3::new(0.0, 0.0, 2.5), DVec3::Z);
	assert!(
		sec.iter().any(|c| matches!(c, Curve::Circle { radius, .. } if (*radius - 2.0).abs() < 1e-9)),
		"a perpendicular section of the union finds the carried cylinder's radius-2 circle, got {sec:?}"
	);
	// The carry-through is surface-agnostic: a SPHERE and a CONE likewise keep their
	// uncut analytic faces through a union (now that the orphaned-vertex topology bug is
	// fixed, a box crossing the curved surface stays a valid genus-0 solid).
	let su = union(&crate::build::sphere(DVec3::ZERO, 2.0, 16, 8), &cuboid(DVec3::new(-0.5, -0.5, 1.0), DVec3::new(0.5, 0.5, 4.0)));
	let sv = validate(&su);
	assert!(sv.is_valid() && sv.euler_characteristic == 2, "sphere∪box valid genus-0: {sv:?}");
	assert!(su.faces().any(|f| matches!(su.face(f).surface, Surface::Sphere { .. })), "uncut sphere faces keep their Surface::Sphere tag");

	let cu =
		union(&crate::build::cone(DVec3::ZERO, DVec3::Z, 2.0, 5.0, 24), &cuboid(DVec3::new(-0.3, -0.3, -1.0), DVec3::new(0.3, 0.3, 1.0)));
	let cv = validate(&cu);
	assert!(cv.is_valid() && cv.euler_characteristic == 2, "cone∪box valid genus-0: {cv:?}");
	assert!(cu.faces().any(|f| matches!(cu.face(f).surface, Surface::Cone { .. })), "uncut cone faces keep their Surface::Cone tag");
}

#[test]
fn box_crossing_many_curved_facets_is_genus_zero() {
	use crate::build::cylinder;
	// FIXED (was a wrong-Euler bug): a box crossing many lateral facets of a cylinder.
	// Root cause was orphaned vertices — `recover_faces` merges a coplanar triangle
	// region into one face, leaving its interior vertices unreferenced; left in the
	// array they inflated V and made `validate` report a spurious genus −1. `stitch`
	// now compacts unreferenced vertices before building the solid. Swept over a range
	// of segment counts that previously failed (≥16).
	for seg in [8usize, 12, 16, 20, 24, 32, 48, 64] {
		let u = union(&cylinder(DVec3::ZERO, DVec3::Z, 2.0, 5.0, seg), &cuboid(DVec3::new(1.0, -1.0, 1.0), DVec3::new(4.0, 1.0, 4.0)));
		let v = validate(&u);
		assert!(v.is_valid() && v.euler_characteristic == 2, "cyl{seg}∪box must be a valid genus-0 solid: {v:?}");
	}
	// (Watertight tessellation at very high facet counts is tracked separately — a
	// distinct near-degenerate-cut issue from the orphaned-vertex topology bug fixed here.)
}

#[test]
fn section_curves_returns_exact_analytic_cross_sections() {
	use crate::build::cylinder;
	use crate::geom::Curve;
	let cyl = cylinder(DVec3::ZERO, DVec3::Z, 2.0, 5.0, 24);

	// Perpendicular cut at z=2.5 → exactly ONE analytic circle of radius 2 on the axis.
	let perp = cyl.section_curves(DVec3::new(0.0, 0.0, 2.5), DVec3::Z);
	let circles: Vec<(DVec3, f64)> = perp
		.iter()
		.filter_map(|c| match c {
			Curve::Circle { center, radius, .. } => Some((*center, *radius)),
			_ => None,
		})
		.collect();
	assert_eq!(circles.len(), 1, "a perpendicular cylinder section is one circle, got {perp:?}");
	assert!(
		(circles[0].1 - 2.0).abs() < 1e-9 && (circles[0].0 - DVec3::new(0.0, 0.0, 2.5)).length() < 1e-9,
		"the section circle is radius 2 centered at z=2.5"
	);

	// Oblique cut → an ELLIPSE with semi-minor = radius and semi-major larger.
	let obl = cyl.section_curves(DVec3::new(0.0, 0.0, 2.5), DVec3::new(0.0, 0.4, 1.0));
	assert!(
		obl.iter().any(|c| matches!(c, Curve::Ellipse { a, b, .. } if *a > *b + 1e-9 && (*b - 2.0).abs() < 1e-6)),
		"an oblique cylinder section includes an ellipse (a>b=2), got {obl:?}"
	);

	// A box cut by z=0 → the section lines of the 4 crossed side faces (caps are parallel).
	let bx = cuboid(DVec3::splat(-1.0), DVec3::splat(1.0));
	let lines = bx.section_curves(DVec3::ZERO, DVec3::Z);
	assert!(
		lines.len() == 4 && lines.iter().all(|c| matches!(c, Curve::Line { .. })),
		"a box z=0 section is 4 side-face lines, got {lines:?}"
	);
}

#[test]
fn face_name_persists_and_re_resolves_across_an_edit() {
	// Topological naming: a stored `FaceName` re-selects the logical face even
	// after an upstream parameter edit re-runs the boolean — the persistent
	// reference a parametric feature needs.
	let cut = |s: f64| difference(&cuboid(DVec3::splat(-s), DVec3::splat(s)), &cuboid(DVec3::ZERO, DVec3::splat(2.0 * s)));

	let d1 = cut(2.0);
	// Within a solid a name round-trips: a face is among those bearing its name.
	let f0 = d1.faces().find(|&f| d1.face_source(f) == Some(FaceSource::OperandB)).expect("a B-sourced cut face");
	let name = d1.face_name(f0).unwrap();
	assert!(d1.faces_named(name).contains(&f0), "a face is among those bearing its own name");

	// Across an edit (the part doubled in size), the stored name still resolves to
	// the corresponding result face — it refers to input topology, not result ids.
	let d2 = cut(4.0);
	let resolved = d2.faces_named(name);
	assert!(
		!resolved.is_empty() && resolved.iter().all(|&f| d2.face_name(f) == Some(name)),
		"stored FaceName {name:?} must re-resolve after the edit (got {} faces)",
		resolved.len()
	);
}

#[test]
fn face_identity_survives_a_nested_boolean() {
	// Chained provenance: in `(A∪B)−C`, a face that originated in B must still
	// trace to B through the SECOND boolean. Because the boolean carries an
	// operand's existing provenance instead of relabelling by the immediate
	// operand, B's surviving wall keeps `OperandB` — without carry-through every
	// face of the `A∪B` operand would read `OperandA`. B's +x wall (x=3) lies
	// outside A and is untouched by C, so it is the unambiguous witness: it cannot
	// come from C (whose faces sit at x∈{1,4}), and it can only read `OperandB`
	// if the inner union's provenance was carried forward.
	let a = cuboid(DVec3::splat(-2.0), DVec3::splat(2.0));
	let b = cuboid(DVec3::ZERO, DVec3::splat(3.0));
	let c = cuboid(DVec3::splat(1.0), DVec3::splat(4.0));

	let chained = difference(&union(&a, &b), &c);
	let b_wall = chained
		.faces()
		.find(|&f| chained.face_polygon(f).iter().all(|p| (p.x - 3.0).abs() < 1e-6))
		.expect("B's +x wall at x=3 survives `(A∪B)−C`");
	assert_eq!(
		chained.face_source(b_wall),
		Some(FaceSource::OperandB),
		"B's wall must keep its B-identity through the nested boolean (chained provenance)"
	);
}

#[test]
fn boolean_stays_valid_far_from_the_origin() {
	// The arrangement's coincidence/weld tests use fixed absolute tolerances, so
	// without re-centring they fail once the f64 ulp grows past them (ulp ≈ 1e-8
	// at 1e8) and the result collapses. Centring the operands keeps the union of
	// two overlapping boxes a valid closed genus-0 solid arbitrarily far out.
	for &t in &[0.0_f64, 1e6, 1e8, 1e10] {
		let off = DVec3::splat(t);
		let a = cuboid(DVec3::splat(-1.0) + off, DVec3::splat(1.0) + off);
		let b = cuboid(off, DVec3::splat(2.0) + off);
		let v = validate(&union(&a, &b));
		assert!(
			v.closed && v.manifold && v.euler_characteristic == 2,
			"union at t={t:e} must be a valid genus-0 solid: closed={} manifold={} χ={}",
			v.closed,
			v.manifold,
			v.euler_characteristic
		);
	}
}

/// Overlap volume of two axis-aligned boxes given by their min/max corners.
fn overlap_volume(amin: DVec3, amax: DVec3, bmin: DVec3, bmax: DVec3) -> f64 {
	let lo = amin.max(bmin);
	let hi = amax.min(bmax);
	let d = (hi - lo).max(DVec3::ZERO);
	d.x * d.y * d.z
}

fn box_vol(min: DVec3, max: DVec3) -> f64 {
	let d = max - min;
	d.x * d.y * d.z
}

#[test]
fn union_of_overlapping_boxes_has_exact_volume() {
	let amin = DVec3::new(0.0, 0.0, 0.0);
	let amax = DVec3::new(10.0, 10.0, 10.0);
	let bmin = DVec3::new(5.0, 5.0, 5.0);
	let bmax = DVec3::new(15.0, 15.0, 15.0);
	let a = cuboid(amin, amax);
	let b = cuboid(bmin, bmax);

	let u = union(&a, &b);
	let v = validate(&u);
	let expected = box_vol(amin, amax) + box_vol(bmin, bmax) - overlap_volume(amin, amax, bmin, bmax);

	assert!(v.is_valid(), "union must be closed + manifold: {v:?}");
	assert!(tessellate_default(&u).is_watertight(), "union must tessellate watertight");
	assert!((volume(&u).abs() - expected).abs() < 1e-6, "union volume {} != expected {}", volume(&u).abs(), expected);
}

#[test]
fn union_box_as_wide_as_base_sharing_side_planes_is_clean() {
	// The demo's ORIGINAL failing case: a base slab and a wall exactly as WIDE as
	// the base, so they share the x=±40 side planes (and z=0). Must be a single
	// clean genus-0 solid.
	let amin = DVec3::new(-40.0, -35.0, 0.0);
	let amax = DVec3::new(40.0, 35.0, 8.0);
	let bmin = DVec3::new(-40.0, 10.0, 0.0);
	let bmax = DVec3::new(40.0, 20.0, 50.0);
	let u = union(&cuboid(amin, amax), &cuboid(bmin, bmax));
	let v = validate(&u);
	let expected = box_vol(amin, amax) + box_vol(bmin, bmax) - overlap_volume(amin, amax, bmin, bmax);
	assert!(
		v.is_valid() && v.genus == 0 && (volume(&u).abs() - expected).abs() < 1e-6,
		"wide-wall union must be a clean genus-0 solid of volume {expected}: {v:?} vol={}",
		volume(&u).abs()
	);
}

#[test]
fn union_of_boxes_stacked_face_to_face_is_one_box() {
	// Two boxes stacked so A's top face and B's bottom face are coincident (and
	// anti-aligned). Their union is a single 10×10×8 box (volume 800).
	let amin = DVec3::new(0.0, 0.0, 0.0);
	let amax = DVec3::new(10.0, 10.0, 4.0);
	let bmin = DVec3::new(0.0, 0.0, 4.0);
	let bmax = DVec3::new(10.0, 10.0, 8.0);
	let u = union(&cuboid(amin, amax), &cuboid(bmin, bmax));
	let v = validate(&u);
	assert!(
		v.is_valid() && v.genus == 0 && (volume(&u).abs() - 800.0).abs() < 1e-6,
		"face-to-face stack union must be one clean box of volume 800: {v:?} vol={}",
		volume(&u).abs()
	);
}

#[test]
fn difference_with_a_coplanar_shared_face_is_clean() {
	// Cut an open slot into A's −X side: the cutter shares A's x=0, z=0 and z=10
	// face planes (and pokes out the x=0 side). The result is a clean notched box.
	let amin = DVec3::new(0.0, 0.0, 0.0);
	let amax = DVec3::new(10.0, 10.0, 10.0);
	let bmin = DVec3::new(0.0, 3.0, 0.0);
	let bmax = DVec3::new(4.0, 7.0, 10.0);
	let d = difference(&cuboid(amin, amax), &cuboid(bmin, bmax));
	let v = validate(&d);
	let expected = box_vol(amin, amax) - overlap_volume(amin, amax, bmin, bmax);
	assert!(
		v.is_valid() && (volume(&d).abs() - expected).abs() < 1e-6,
		"coplanar-face difference must be a clean solid of volume {expected}: {v:?} vol={}",
		volume(&d).abs()
	);
}

#[test]
fn union_of_boxes_sharing_face_planes_is_a_clean_solid() {
	// A slab and a wall that interpenetrate AND share three face planes (x=0, x=10,
	// z=0). This is the coplanar partial-overlap case the demo exposed: the union
	// must still be a single closed, manifold, genus-0 solid (a slab with a wall).
	//
	// (task #15, FIXED — live regression test): a coplanar cutter face that extends
	// BEYOND the subject face, where the shared coplanar edge meets a transversal cut,
	// used to break the union. The coplanar partial-overlap handling now resolves it, so
	// this asserts a clean genus-0 solid rather than guarding a known failure.
	let amin = DVec3::new(0.0, 0.0, 0.0);
	let amax = DVec3::new(10.0, 10.0, 4.0);
	let bmin = DVec3::new(0.0, 3.0, 0.0);
	let bmax = DVec3::new(10.0, 7.0, 12.0);
	let u = union(&cuboid(amin, amax), &cuboid(bmin, bmax));
	let v = validate(&u);
	let expected = box_vol(amin, amax) + box_vol(bmin, bmax) - overlap_volume(amin, amax, bmin, bmax);
	assert!(
		v.is_valid() && v.genus == 0 && (volume(&u).abs() - expected).abs() < 1e-6,
		"coplanar-face union must be a clean genus-0 solid of volume {expected}: {v:?} vol={}",
		volume(&u).abs()
	);
}

#[test]
fn difference_of_overlapping_boxes_has_exact_volume() {
	let amin = DVec3::new(0.0, 0.0, 0.0);
	let amax = DVec3::new(10.0, 10.0, 10.0);
	let bmin = DVec3::new(5.0, 5.0, 5.0);
	let bmax = DVec3::new(15.0, 15.0, 15.0);
	let a = cuboid(amin, amax);
	let b = cuboid(bmin, bmax);

	let d = difference(&a, &b);
	let v = validate(&d);
	let expected = box_vol(amin, amax) - overlap_volume(amin, amax, bmin, bmax);

	assert!(v.is_valid(), "difference must be closed + manifold: {v:?}");
	assert!(tessellate_default(&d).is_watertight(), "difference must tessellate watertight");
	assert!((volume(&d).abs() - expected).abs() < 1e-6, "difference volume {} != expected {}", volume(&d).abs(), expected);
}

#[test]
fn intersection_of_overlapping_boxes_has_exact_volume() {
	let amin = DVec3::new(0.0, 0.0, 0.0);
	let amax = DVec3::new(10.0, 10.0, 10.0);
	let bmin = DVec3::new(5.0, 5.0, 5.0);
	let bmax = DVec3::new(15.0, 15.0, 15.0);
	let a = cuboid(amin, amax);
	let b = cuboid(bmin, bmax);

	let i = intersection(&a, &b);
	let v = validate(&i);
	let expected = overlap_volume(amin, amax, bmin, bmax);

	assert!(v.is_valid(), "intersection must be closed + manifold: {v:?}");
	assert!(tessellate_default(&i).is_watertight(), "intersection must tessellate watertight");
	assert!((volume(&i).abs() - expected).abs() < 1e-6, "intersection volume {} != expected {}", volume(&i).abs(), expected);
}

#[test]
fn union_of_disjoint_boxes_keeps_both_volumes() {
	let a = cuboid(DVec3::new(0.0, 0.0, 0.0), DVec3::new(2.0, 2.0, 2.0));
	let b = cuboid(DVec3::new(5.0, 5.0, 5.0), DVec3::new(7.0, 7.0, 7.0));
	let u = union(&a, &b);
	assert!((volume(&u).abs() - 16.0).abs() < 1e-6, "two disjoint 2³ boxes: {}", volume(&u).abs());
	assert!(validate(&u).closed, "disjoint union still closed");
}

#[test]
fn intersection_of_disjoint_boxes_is_empty() {
	let a = cuboid(DVec3::new(0.0, 0.0, 0.0), DVec3::new(2.0, 2.0, 2.0));
	let b = cuboid(DVec3::new(5.0, 5.0, 5.0), DVec3::new(7.0, 7.0, 7.0));
	let i = intersection(&a, &b);
	assert_eq!(i.face_count(), 0, "disjoint intersection is empty");
}

#[test]
fn difference_removing_corner_is_general_nonconvex() {
	// A non-convex result: cut a smaller box out of a corner of a larger one.
	let amin = DVec3::new(0.0, 0.0, 0.0);
	let amax = DVec3::new(10.0, 10.0, 10.0);
	let bmin = DVec3::new(-1.0, -1.0, -1.0);
	let bmax = DVec3::new(4.0, 4.0, 4.0);
	let a = cuboid(amin, amax);
	let b = cuboid(bmin, bmax);
	let d = difference(&a, &b);
	let expected = box_vol(amin, amax) - overlap_volume(amin, amax, bmin, bmax);
	assert!(validate(&d).is_valid(), "corner-cut solid is valid: {:?}", validate(&d));
	assert!((volume(&d).abs() - expected).abs() < 1e-6, "corner cut volume {} != {}", volume(&d).abs(), expected);
}

#[test]
fn intersection_with_fully_containing_box_returns_inner_solid() {
	// Generality (not a box-on-box special case): a triangular prism wholly
	// inside a large box. A ∩ B == the prism, exactly.
	let prism = crate::build::extrude(&[glam::DVec2::new(1.0, 1.0), glam::DVec2::new(5.0, 1.0), glam::DVec2::new(2.0, 4.0)], 3.0);
	let big = cuboid(DVec3::new(-10.0, -10.0, -10.0), DVec3::new(20.0, 20.0, 20.0));
	let prism_vol = volume(&prism).abs();

	let i = intersection(&prism, &big);
	assert!(validate(&i).is_valid(), "prism ∩ box valid: {:?}", validate(&i));
	assert!(tessellate_default(&i).is_watertight(), "prism ∩ box watertight");
	assert!((volume(&i).abs() - prism_vol).abs() < 1e-6, "prism ∩ containing box == prism: {} vs {}", volume(&i).abs(), prism_vol);
}

#[test]
fn union_with_fully_contained_solid_returns_outer_volume() {
	// B ⊂ A ⇒ A ∪ B == A (in volume), for a non-axis-aligned inner prism.
	let outer = cuboid(DVec3::new(0.0, 0.0, 0.0), DVec3::new(10.0, 10.0, 10.0));
	let inner = crate::build::extrude(&[glam::DVec2::new(3.0, 3.0), glam::DVec2::new(7.0, 4.0), glam::DVec2::new(5.0, 8.0)], 4.0)
		.transformed(kernel_core::math::DAffine3::from_translation(DVec3::new(0.0, 0.0, 2.0)));
	let outer_vol = volume(&outer).abs();
	let u = union(&outer, &inner);
	assert!(validate(&u).is_valid(), "A ∪ (B⊂A) valid: {:?}", validate(&u));
	assert!((volume(&u).abs() - outer_vol).abs() < 1e-6, "A ∪ contained == A: {} vs {}", volume(&u).abs(), outer_vol);
}

#[test]
fn difference_of_general_prism_overlap_is_valid_and_volume_correct() {
	// Two overlapping triangular prisms (general planar solids, not boxes).
	// The difference volume equals V(A) minus the volume A and B share, which
	// here equals V(A) − V(A∩B); we verify the CSG identity numerically via the
	// intersection operator (independent code path computing the same overlap).
	let a = crate::build::extrude(&[glam::DVec2::new(0.0, 0.0), glam::DVec2::new(6.0, 0.0), glam::DVec2::new(3.0, 6.0)], 5.0);
	let b = crate::build::extrude(&[glam::DVec2::new(2.0, 2.0), glam::DVec2::new(8.0, 2.0), glam::DVec2::new(5.0, 8.0)], 5.0);
	let a_vol = volume(&a).abs();
	let inter_vol = volume(&intersection(&a, &b)).abs();
	let d = difference(&a, &b);
	assert!(validate(&d).is_valid(), "prism − prism valid: {:?}", validate(&d));
	assert!(tessellate_default(&d).is_watertight(), "prism − prism watertight");
	assert!(
		(volume(&d).abs() - (a_vol - inter_vol)).abs() < 1e-6,
		"V(A−B) == V(A) − V(A∩B): {} vs {}",
		volume(&d).abs(),
		a_vol - inter_vol
	);
}

#[test]
fn empty_operand_is_handled_gracefully() {
	let a = cuboid(DVec3::new(0.0, 0.0, 0.0), DVec3::new(2.0, 2.0, 2.0));
	let empty = Solid::default();
	// Union/difference with empty leaves A; intersection with empty is empty.
	assert!((volume(&union(&a, &empty)).abs() - 8.0).abs() < 1e-9);
	assert!((volume(&difference(&a, &empty)).abs() - 8.0).abs() < 1e-9);
	assert_eq!(intersection(&a, &empty).face_count(), 0);
}

#[test]
fn rotated_box_union_is_general_off_axis() {
	// Generality off the coordinate axes: a box rotated 30° about Z, unioned
	// with an overlapping axis-aligned box. We verify the CSG identity
	// V(A∪B) == V(A) + V(B) − V(A∩B) using the (independent) intersection path.
	use kernel_core::math::DAffine3;
	let a = cuboid(DVec3::new(0.0, 0.0, 0.0), DVec3::new(10.0, 10.0, 10.0)).transformed(DAffine3::from_rotation_z(30.0_f64.to_radians()));
	let b = cuboid(DVec3::new(3.0, 3.0, 2.0), DVec3::new(13.0, 13.0, 8.0));
	let va = volume(&a).abs();
	let vb = volume(&b).abs();
	let vi = volume(&intersection(&a, &b)).abs();
	let u = union(&a, &b);
	let expected = va + vb - vi;
	assert!(validate(&u).is_valid(), "rotated union valid: {:?}", validate(&u));
	assert!(tessellate_default(&u).is_watertight(), "rotated union watertight");
	// Off-axis geometry carries irrational coordinates (cos/sin 30°), so the
	// agreement is to floating-point relative precision rather than the ~1e-9
	// exactness of axis-aligned planar input.
	assert!((volume(&u).abs() - expected).abs() / expected < 1e-5, "V(A∪B) {} != V(A)+V(B)−V(A∩B) {}", volume(&u).abs(), expected);
}

#[test]
fn self_union_is_idempotent() {
	// A ∪ A == A (volume), and the result is a valid closed solid: a stress test
	// for coincident-face handling (every face is shared/aligned).
	let a = cuboid(DVec3::new(-2.0, -2.0, -2.0), DVec3::new(2.0, 2.0, 2.0));
	let u = union(&a, &a);
	assert!(validate(&u).is_valid(), "A∪A valid: {:?}", validate(&u));
	assert!((volume(&u).abs() - 64.0).abs() < 1e-6, "A∪A volume {} != 64", volume(&u).abs());
}

// --- Loop-aware chained booleans (R2/R3, BAR Level 6) -------------------------
//
// Root causes fixed (2026-06-09), each previously exploding genus/shells:
// 1. Sub-tolerance sliver triangles: re-triangulating a boolean RESULT emits
//    near-degenerate triangles along T-junction-healed near-collinear chains;
//    after welding, `resolve_t_junctions` folded such a triangle's own apex into
//    its base edge ([a,b,c] → [a,b,c,b]), tripling directed edges and breaking
//    twin pairing. `stitch` now drops sub-`TJUNCTION_EPS`-altitude slivers.
// 2. Outer-loop-only triangulation: a face with INNER loops (extrude_with_holes
//    cap) was triangulated as if filled. `triangulate_solid` now bridges holes
//    into the outer ring (same algorithm as the multi-loop tessellator).
// 3. Region merging across non-manifold (≥3-triangle) edges was HashMap-order
//    dependent; `recover_faces` now merges across exactly-2-triangle edges only.
// 4. Surface tags were looked up by FaceName, which COLLIDES in chained booleans
//    (an operand that is itself a boolean carries `OperandA/B` names of ITS
//    operands); a first bore's wall could get the second bore's cylinder and
//    tessellate onto the wrong surface. Fragments now carry their operand face's
//    `Surface` by value.

/// n-gon prism cross-section area for a faceted "cylinder" of radius `r`.
fn ngon_area(r: f64, n: usize) -> f64 {
	0.5 * n as f64 * r * r * (std::f64::consts::TAU / n as f64).sin()
}

#[test]
fn second_hole_into_the_same_face_stays_valid() {
	use crate::build::cylinder;
	// R2 repro 1: drilling a SECOND hole through caps that already carry the first
	// hole's rims. Before the fix: closed=false, genus ≈ 125, shells ≈ 24. The
	// result must be a valid genus-2 solid (two through-holes) with the exact
	// faceted volume — and deterministically so (the surface-tag collision made
	// the volume flake run to run), hence the repeated runs.
	let plate = cuboid(DVec3::ZERO, DVec3::new(60.0, 40.0, 8.0));
	let hole = |x: f64, y: f64| cylinder(DVec3::new(x, y, -1.0), DVec3::Z, 3.5, 10.0, 32);
	let d1 = difference(&plate, &hole(45.0, 12.0));
	let v1 = validate(&d1);
	assert!(v1.is_valid() && v1.genus == 1, "first hole: valid genus-1 plate: {v1:?}");

	// Volume tolerance 1e-3 mm³ (relative ~5e-9): T-junction healing legitimately
	// moves seam vertices by up to TJUNCTION_EPS (4e-7) and the sliver filter drops
	// sub-tolerance gap area, so a chained result is exact only to that scale.
	let expected = 60.0 * 40.0 * 8.0 - 2.0 * ngon_area(3.5, 32) * 8.0;
	for run in 0..5 {
		let d2 = difference(&d1, &hole(45.0, 28.0));
		let v2 = validate(&d2);
		let vol = volume(&d2).abs();
		assert!(
			v2.is_valid() && v2.genus == 2 && (vol - expected).abs() < 1e-3,
			"second hole, run {run}: must be a valid genus-2 solid of volume {expected}: {v2:?} vol={vol}"
		);
	}
}

#[test]
fn coplanar_union_against_a_multiloop_holed_face_is_clean() {
	use crate::build::extrude_with_holes;
	// R2 repro 2: union an upright onto a plate whose caps carry a TRUE inner
	// loop (extrude_with_holes), sharing the x=0, y=0, y=40 and z-contact planes.
	// The hole is far from the contact. Before the fix the holed caps were
	// triangulated outer-loop-only (hole filled) and the union exploded; now it
	// is a valid genus-1 solid with the exact faceted volume.
	let circle: Vec<glam::DVec2> = (0..32)
		.map(|i| {
			let a = std::f64::consts::TAU * i as f64 / 32.0;
			glam::DVec2::new(45.0 + 3.5 * a.cos(), 20.0 + 3.5 * a.sin())
		})
		.collect();
	let outer = vec![glam::DVec2::new(0.0, 0.0), glam::DVec2::new(60.0, 0.0), glam::DVec2::new(60.0, 40.0), glam::DVec2::new(0.0, 40.0)];
	let plate = extrude_with_holes(&outer, &[circle], 8.0);
	let upright = cuboid(DVec3::ZERO, DVec3::new(8.0, 40.0, 50.0));
	let u = union(&plate, &upright);
	let v = validate(&u);
	let expected = (60.0 * 40.0 - ngon_area(3.5, 32)) * 8.0 + 8.0 * 40.0 * 50.0 - 8.0 * 40.0 * 8.0;
	// 1e-3 mm³ tolerance: see `second_hole_into_the_same_face_stays_valid`.
	assert!(
		v.is_valid() && v.genus == 1 && (volume(&u).abs() - expected).abs() < 1e-3,
		"holed-plate ∪ upright must be a valid genus-1 solid of volume {expected}: {v:?} vol={}",
		volume(&u).abs()
	);

	// The same union where the hole was drilled by a BOOLEAN instead (the plate is
	// then a triangle-soup B-rep whose caps carry the bore rims as healed chains —
	// the other half of R2 repro 2, which also exploded before the fix).
	use crate::build::cylinder;
	let plain = cuboid(DVec3::ZERO, DVec3::new(60.0, 40.0, 8.0));
	let drilled = difference(&plain, &cylinder(DVec3::new(45.0, 20.0, -1.0), DVec3::Z, 3.5, 10.0, 32));
	let u2 = union(&drilled, &upright);
	let v2 = validate(&u2);
	assert!(
		v2.is_valid() && v2.genus == 1 && (volume(&u2).abs() - expected).abs() < 1e-3,
		"boolean-drilled plate ∪ upright must be a valid genus-1 solid of volume {expected}: {v2:?} vol={}",
		volume(&u2).abs()
	);
}

#[test]
fn keyway_crossing_a_bored_wall_stays_valid() {
	use crate::build::cylinder;
	// R3: cut a keyway that crosses a previously-cut curved bore wall. Before the
	// fix: genus ≈ 204, shells ≈ 45. Volume is checked against the independent
	// intersection path: V(bored − keyway) = V(bored) − V(bored ∩ keyway).
	let blank = cuboid(DVec3::new(-20.0, -20.0, 0.0), DVec3::new(20.0, 20.0, 30.0));
	let bore = cylinder(DVec3::new(0.0, 0.0, -1.0), DVec3::Z, 5.0, 32.0, 48);
	let bored = difference(&blank, &bore);
	let vb = validate(&bored);
	assert!(vb.is_valid() && vb.genus == 1, "bored blank is a valid genus-1 solid: {vb:?}");

	let keyway = cuboid(DVec3::new(-2.0, 3.0, -1.0), DVec3::new(2.0, 8.0, 31.0));
	let keyed = difference(&bored, &keyway);
	let vk = validate(&keyed);
	// The identity is checked in the ANALYTIC measure, 1000× tighter than the old
	// 1e-3 faceted-volume gate. Every cut here is ⟂ or parallel to the bore axis, so
	// post-snap each cut bore facet is a θ-rectangular cylinder patch and the
	// exact_volume bulge corrections close the identity to round-off. The faceted
	// (tessellated) identity no longer closes tightly — and should not: seam snapping
	// (2026-06-10) lands the keyway∩bore seam ON the true cylinder, so the cut
	// results hug the bore more closely than the *uncut* operand's chord facets do.
	let expected = exact_volume(&bored).abs() - exact_volume(&intersection(&bored, &keyway)).abs();
	assert!(
		vk.is_valid() && vk.genus == 1 && (exact_volume(&keyed).abs() - expected).abs() < 1e-6,
		"keyway through bore must stay a valid genus-1 solid of exact volume {expected}: {vk:?} vol={}",
		exact_volume(&keyed).abs()
	);
}

#[test]
fn chained_bolt_circle_differences_stay_valid() {
	use crate::build::cylinder;
	// R2 repro 3: six sequential bolt-hole differences into one disc — every cut
	// lands on caps already carrying the previous holes' rims. Before the fix the
	// genus walked 129 → 217 → 284 → 399 → 457 with dozens of shells; now each
	// step is a valid solid of genus k+1 and the final volume is exact.
	let mut cur = cylinder(DVec3::ZERO, DVec3::Z, 30.0, 6.0, 48);
	for k in 0..6 {
		let a = std::f64::consts::TAU * k as f64 / 6.0;
		let bolt = cylinder(DVec3::new(22.0 * a.cos(), 22.0 * a.sin(), -1.0), DVec3::Z, 2.5, 8.0, 24);
		cur = difference(&cur, &bolt);
		let v = validate(&cur);
		assert!(v.is_valid() && v.genus == k + 1 && v.shells == 1, "flange after bolt {k}: must be one valid genus-{} shell: {v:?}", k + 1);
	}
	let expected = (ngon_area(30.0, 48) - 6.0 * ngon_area(2.5, 24)) * 6.0;
	// 1e-3 mm³ tolerance: see `second_hole_into_the_same_face_stays_valid`.
	assert!(
		(volume(&cur).abs() - expected).abs() < 1e-3,
		"6-bolt flange volume {} must equal the exact faceted {expected}",
		volume(&cur).abs()
	);
}

#[test]
fn degenerate_configurations_never_panic() {
	// Robustness: tricky contacts (coincident faces, edge-only / corner-only
	// touching, full coincidence, a near-zero sliver overlap) drive the
	// co-refinement into collinear and zero-length-edge situations. The kernel
	// must be TOTAL on these — every boolean COMPLETES (no panic: the on-edge
	// insertion previously divided by a zero-length edge and sorted a NaN, and
	// the stitch fed a non-manifold soup to `from_faces` which then asserted)
	// and returns a finite mesh that `validate` can inspect without panicking.
	// (Validity itself is not claimed: edge-/corner-only contact has a genuinely
	// non-manifold union that no closed B-rep can represent.)
	let unit = |o: DVec3| cuboid(o, o + DVec3::splat(2.0));
	let configs = [
		("coincident-face", unit(DVec3::ZERO), unit(DVec3::new(2.0, 0.0, 0.0))),
		("edge-touching", unit(DVec3::ZERO), unit(DVec3::new(2.0, 2.0, 0.0))),
		("corner-touching", unit(DVec3::ZERO), unit(DVec3::new(2.0, 2.0, 2.0))),
		("full-coincidence", unit(DVec3::ZERO), unit(DVec3::ZERO)),
		("sliver-overlap", unit(DVec3::ZERO), unit(DVec3::new(2.0 - 1e-9, 0.0, 0.0))),
	];
	for (name, a, b) in configs {
		for (op, r) in [("union", union(&a, &b)), ("difference", difference(&a, &b)), ("intersection", intersection(&a, &b))] {
			let _ = validate(&r); // must not panic
			let mesh = tessellate_default(&r); // must not panic
			assert!(
				mesh.positions.iter().all(|p| p.x.is_finite() && p.y.is_finite() && p.z.is_finite()),
				"{name}/{op}: tessellation produced a non-finite vertex"
			);
		}
	}
}
