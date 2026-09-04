// Copyright (c) LMCAD. Licensed under the MIT License.

//! Reverse-bridge acceptance: the one-way street reopened, end to end.
//!
//! V1: an implicit result (a field-native smooth blend no B-rep op can author)
//! is extracted, wrapped as a FACETED B-rep (planar facets + coplanar
//! coalescing), validated, exported to STEP, re-imported, and the volume drift
//! across the whole street is bounded. Plus the coalesce pin (a tessellated
//! box wraps back to exactly 6 faces) and the thin-wall field interrogation on
//! a known 2 mm hollow-box wall.
//!
//! V2 (analytic quadric recovery): the same street with
//! `implicit_to_solid_recovered` — a voxelized cylinder collapses from
//! thousands of facet planes to dozens of `Surface::Cylinder` sector faces
//! (volume conserved against BOTH the v1 solid and the closed form), the STEP
//! file shrinks by the pinned ratio and re-imports exactly; a sphere and a
//! torus come back with their center/radius/axis within tolerance (carrier
//! recovery, face count honestly unchanged — doubly-curved merge is
//! fidelity-limited, see kernel_brep::recover); a cone recovers apex and
//! half-angle from an implicit frustum.

use kernel_brep::math::DVec3;
use kernel_brep::{cuboid, export_step, import_step, tessellate_default, validate, volume, Surface};
use kernel_core::math::Vec3;
use kernel_core::sdf::Sdf;
use kernel_implicit::{Cone, Cuboid, Cylinder, Node, Sphere, Torus};
use kernel_model::reverse::{implicit_to_solid, implicit_to_solid_recovered, mesh_to_solid, thin_wall_report};

#[test]
fn implicit_blend_bridges_to_step_and_back_within_volume_drift() {
	// A smooth-blended implicit — sphere ∪ smooth-union box, k = 1.5 — is a
	// shape only the field half can make (the blend fillet is field-native).
	let blend = Node::primitive(Sphere::new(Vec3::ZERO, 6.0))
		.smooth_union(Node::primitive(Cuboid::new(Vec3::new(5.0, 0.0, 0.0), Vec3::splat(4.0))), 1.5);
	let solid = implicit_to_solid(&blend, blend.bounds(), 0.8).expect("implicit → faceted B-rep must bridge");
	let v = validate(&solid);
	let v_out = volume(&solid);
	let step = export_step(&solid, "reverse_bridge_blend");
	let back = import_step(&step).expect("bridged STEP must re-import");
	let v_back = validate(&back);
	let vol_back = volume(&back);
	let drift = (vol_back - v_out).abs() / v_out;
	// Measured on this deterministic pipeline: 2089 faces, 1185.669 mm³ both
	// sides, drift exactly 0.0 — the 2.5% bar is headroom for coarser shapes,
	// not slack this shape needs.
	assert!(
		v.is_valid() && v_back.is_valid() && v_out > 0.0 && drift < 0.025,
		"implicit → solid → STEP → solid must survive the full street: bridged validity {v:?}, \
		 re-import validity {v_back:?}, volume {v_out:.3} → {vol_back:.3} mm³ (round-trip drift {:.3}%, bar 2.5%), \
		 {} faces, STEP {} bytes",
		drift * 100.0,
		solid.face_count(),
		step.len()
	);
}

#[test]
fn tessellated_cuboid_wraps_and_coalesces_back_to_six_faces() {
	// The coalesce pin: 12 triangles in, exactly the 6 planar faces out.
	let mesh = tessellate_default(&cuboid(DVec3::ZERO, DVec3::new(20.0, 10.0, 5.0)));
	let solid = mesh_to_solid(&mesh).expect("a tessellated box must wrap");
	let v = validate(&solid);
	let vol = volume(&solid);
	assert!(
		v.is_valid() && solid.face_count() == 6 && (vol - 1000.0).abs() < 1e-9,
		"a 12-triangle box mesh must coalesce back to exactly 6 planar faces (got {}) as a valid solid ({v:?}) conserving the 1000 mm³ volume (got {vol:.12})",
		solid.face_count()
	);
}

#[test]
fn thin_wall_report_reads_the_2mm_hollow_box_wall() {
	// 2 mm wall everywhere: 20 mm outer box minus 16 mm concentric inner box.
	let hollow =
		Node::primitive(Cuboid::new(Vec3::ZERO, Vec3::splat(10.0))).difference(Node::primitive(Cuboid::new(Vec3::ZERO, Vec3::splat(8.0))));
	let report = thin_wall_report(&hollow, hollow.bounds(), 96, 1.0);
	// The medial shell of a flat 2 mm wall sits 9 mm from centre; the sampled
	// estimate under-reports by up to ~one lattice cell (20/95 ≈ 0.21 mm), so
	// the expected reading is ~1.8–2.0 mm — assert the ±20% band. Measured on
	// this (deterministic) lattice: 1.8947 mm at |x| ≈ 8.947.
	let medial_coord = report.at.abs().max_element();
	assert!(
		(report.thinnest - 2.0).abs() <= 0.4 && report.below_count == 0 && (medial_coord - 9.0).abs() < 0.5,
		"sampled thin-wall estimate must read the 2 mm wall within ±20%: measured {:.4} mm at {:?} (|max coord| {:.3}, medial shell is at 9.0), {} samples below 1.0 mm",
		report.thinnest,
		report.at,
		medial_coord,
		report.below_count
	);
}

/// The first non-plane face's surface matching `pick`.
fn find_surface(s: &kernel_brep::Solid, pick: impl Fn(&Surface) -> bool) -> Option<Surface> {
	s.faces().map(|f| s.face(f).surface).find(|surf| !matches!(surf, Surface::Plane { .. }) && pick(surf))
}

#[test]
fn implicit_cylinder_recovery_collapses_faces_and_shrinks_step() {
	// Gate 1 + 2: r = 8, h = 40 cylinder SDF at voxel 0.4, recovery tol 0.05 mm.
	let sdf = Cylinder::new(Vec3::new(0.0, 0.0, -20.0), Vec3::new(0.0, 0.0, 20.0), 8.0);
	let bounds = sdf.bounds();
	let v1 = implicit_to_solid(&sdf, bounds, 0.4).expect("v1 faceted bridge must succeed");
	let (v2, rep) = implicit_to_solid_recovered(&sdf, bounds, 0.4, 0.05).expect("v2 recovery must succeed");
	let vol1 = volume(&v1);
	let vol2 = volume(&v2);
	let analytic = std::f64::consts::PI * 8.0 * 8.0 * 40.0;
	let drift_v1 = (vol2 - vol1).abs() / vol1;
	let drift_analytic = (vol2 - analytic).abs() / analytic;
	let cyl = find_surface(&v2, |s| matches!(s, Surface::Cylinder { .. }));
	let Some(Surface::Cylinder { axis, radius, .. }) = cyl else {
		panic!("v2 solid must carry a recovered Surface::Cylinder, report {rep:?}");
	};
	// STEP payoff: v1 faceted vs v2 recovered, same part.
	let s1 = export_step(&v1, "reverse_v1_faceted");
	let s2 = export_step(&v2, "reverse_v2_recovered");
	let ratio = s1.len() as f64 / s2.len() as f64;
	let back = import_step(&s2).expect("recovered STEP must re-import");
	let vol_back = volume(&back);
	let reimport_drift = (vol_back - vol2).abs() / vol2;
	// Measured on this deterministic pipeline, UPDATED 2026-07-30 when merged
	// curved faces gained interior-refined tessellation (the 0.11-rad sector
	// budget is gone — see kernel_brep::recover / ::tessellate):
	// report {1 cylinder, 2 tolerant planes (the caps), faces 1326 → 24 = 2
	// half-wrap chart sectors + 22 honest crease-chamfer leftover facets
	// (previously 80 = 58 budgeted sectors + 2 caps + 22 leftovers), residual
	// 0.0208 mm}; fitted r 8.00159, axis·Z = 1 − 2e-11; volume 8043.55 (v1) /
	// 8028.04 (v2) / 8042.48 (πr²h) mm³ — drifts 0.193% and 0.180%; STEP
	// 5 742 349 → 180 033 bytes (ratio 31.90, was 3.06 — the payoff of the
	// deeper collapse); re-import volume 8028.0420, drift 0.0000% (the
	// importer verifies a periodic-strip reconstruction against the parameter
	// chart by FLUX and re-reads a folded strip on the chart instead).
	assert!(
		rep.cylinders >= 1
			&& rep.faces_after * 50 <= rep.faces_before
			&& rep.faces_after < 30
			&& validate(&v2).is_valid()
			&& tessellate_default(&v2).is_watertight()
			&& drift_v1 < 0.005
			&& drift_analytic < 0.005
			&& (radius - 8.0).abs() < 0.01
			&& axis.dot(DVec3::Z).abs() > 1.0 - 1e-6
			&& rep.max_fit_residual > 0.0
			&& rep.max_fit_residual <= 0.05
			&& ratio >= 25.0
			&& validate(&back).is_valid()
			&& reimport_drift < 0.002,
		"implicit cylinder v2 gates: report {rep:?} (want ≥50× face collapse, <30 faces; the pre-2026-07-30 \
		 boundary-only tessellation pinned 80 faces at ≥16×); \
		 volume v1 {vol1:.4} / v2 {vol2:.4} / analytic {analytic:.4} mm³ \
		 (v1 drift {:.4}%, bar 0.5%; analytic drift {:.4}%, bar 0.5%); fitted r {radius:.5} (bar ±0.01), axis·Z {:.10}; \
		 STEP {} → {} bytes (ratio {ratio:.2}, bar 25×); re-import validity {:?}, volume {vol_back:.4} (drift {:.4}%, bar 0.2%)",
		drift_v1 * 100.0,
		drift_analytic * 100.0,
		axis.dot(DVec3::Z),
		s1.len(),
		s2.len(),
		validate(&back),
		reimport_drift * 100.0
	);
}

#[test]
fn implicit_sphere_recovers_center_and_radius() {
	// Gate 3: sphere SDF at (2, −1, 3), r = 8, voxel 0.4, tol 0.05.
	let sdf = Sphere::new(Vec3::new(2.0, -1.0, 3.0), 8.0);
	let bounds = sdf.bounds();
	let v1 = implicit_to_solid(&sdf, bounds, 0.4).expect("v1 sphere bridge must succeed");
	let (v2, rep) = implicit_to_solid_recovered(&sdf, bounds, 0.4, 0.05).expect("v2 sphere recovery must succeed");
	let (vol1, vol2) = (volume(&v1), volume(&v2));
	let sph = find_surface(&v2, |s| matches!(s, Surface::Sphere { .. }));
	let Some(Surface::Sphere { center, radius }) = sph else {
		panic!("v2 sphere solid must carry Surface::Sphere, report {rep:?}");
	};
	let center_err = (center - DVec3::new(2.0, -1.0, 3.0)).length();
	let analytic = 4.0 / 3.0 * std::f64::consts::PI * 8.0_f64.powi(3);
	let drift = (vol2 - vol1).abs() / vol1;
	let (err_in, err_out) = ((vol1 - analytic).abs(), (vol2 - analytic).abs());
	// Measured on this deterministic pipeline, UPDATED 2026-07-30: doubly-curved
	// regions now COLLAPSE into cubemap chart faces — 14 979 facets → 6 — where
	// the previous pin asserted retag-only (faces unchanged at 14 979) because
	// the boundary-ring-only tessellators could not keep a merged sphere face's
	// bulge. Fitted center 0.00034 mm off the SDF's, radius 8.00297, vertex
	// residual 0.00369 mm; volume 2145.7464 → 2145.7102 (drift 0.0017%), and
	// the merged solid stays at least as close to 4πr³/3 = 2144.6606 as the
	// chord input it replaced.
	assert!(
		rep.spheres == 1
			&& rep.faces_after == 6
			&& rep.faces_after == v2.face_count()
			&& center_err < 5e-3
			&& (radius - 8.0).abs() < 0.01
			&& rep.max_fit_residual > 0.0
			&& rep.max_fit_residual <= 0.05
			&& drift < 0.005
			&& err_out <= err_in
			&& validate(&v2).is_valid()
			&& tessellate_default(&v2).is_watertight(),
		"implicit sphere recovery COLLAPSES to cubemap chart faces (the pre-2026-07-30 boundary-only tessellation \
		 pinned retag-only, {} faces in and out): report {rep:?} (want 6 faces); fitted center {center:?} \
		 (truth (2,−1,3), off {center_err:.5} mm, bar 5e-3), radius {radius:.5} (truth 8, bar 0.01); \
		 residual {:.5} ≤ tol 0.05; volume {vol1:.4} → {vol2:.4} (drift {:.4}%, bar 0.5%); \
		 4πr³/3 = {analytic:.4}, |err| {err_in:.4} → {err_out:.4} (merge must not be further from truth)",
		rep.faces_before,
		rep.max_fit_residual,
		drift * 100.0
	);
}

#[test]
fn implicit_cone_and_torus_recover_analytic_carriers() {
	// Gate 6, BOTH halves. Cone: frustum r 10 → 4 over z −15..15 ⇒ apex at
	// z = +35 (extrapolated 20 mm past the material), half-angle atan(0.2).
	let cone_sdf = Cone { a: Vec3::new(0.0, 0.0, -15.0), b: Vec3::new(0.0, 0.0, 15.0), ra: 10.0, rb: 4.0 };
	let bounds = kernel_core::math::Aabb::new(Vec3::new(-11.0, -11.0, -16.0), Vec3::new(11.0, 11.0, 16.0));
	let v1 = implicit_to_solid(&cone_sdf, bounds, 0.4).expect("v1 cone bridge must succeed");
	let (v2, rep) = implicit_to_solid_recovered(&cone_sdf, bounds, 0.4, 0.05).expect("v2 cone recovery must succeed");
	let (vol1, vol2) = (volume(&v1), volume(&v2));
	let analytic = std::f64::consts::PI * 30.0 * (100.0 + 40.0 + 16.0) / 3.0; // frustum πh(R²+Rr+r²)/3
	let con = find_surface(&v2, |s| matches!(s, Surface::Cone { .. }));
	let Some(Surface::Cone { apex, axis, half_angle }) = con else {
		panic!("v2 cone solid must carry Surface::Cone, report {rep:?}");
	};
	let truth_half = (0.2_f64).atan();
	let apex_err = (apex - DVec3::new(0.0, 0.0, 35.0)).length();
	// Measured on this deterministic pipeline, UPDATED 2026-07-30 (interior-
	// refined merged faces replaced the 0.11-rad sector budget): 24 710 → 30
	// faces (824×, was 126 at 196×), apex (0.0018, 0.0018, 35.019) — 0.0188 mm
	// from the extrapolated truth — half-angle 0.197339 vs atan(0.2) =
	// 0.197396, volume 4901.62 (v1) / 4880.45 (v2) / 4900.88 (frustum closed
	// form) mm³: drifts 0.432% / 0.417% (the merged cone's two half-wrap
	// development charts carry the mesher's own boundary fidelity; both stay
	// inside the 0.5% gate, which is what refuses a worse merge).
	let cone_ok = rep.cones >= 1
		&& rep.faces_after * 500 <= rep.faces_before
		&& apex_err < 0.1
		&& axis.dot(-DVec3::Z) > 1.0 - 1e-4
		&& (half_angle - truth_half).abs() < 1e-3
		&& (vol2 - vol1).abs() / vol1 < 0.005
		&& (vol2 - analytic).abs() / analytic < 0.005
		&& validate(&v2).is_valid();

	// Torus: full donut, major 12 / minor 4, voxel 0.4. Doubly-curved regions
	// CAN now collapse into a 4 × 4 azimuth × tube-angle quadrant chart grid —
	// a builder-faceted torus does exactly that, pinned at 4608 → 16 faces in
	// `kernel-brep/tests/curved_faces.rs`. This MESHER-derived one does not, and
	// the reason is pinned rather than papered over: on the marching-cubes
	// boundary the merged quadrant faces and their neighbours ear-clip
	// coincident chords, leaving 11 four-triangle edges — a valid B-rep whose
	// default mesh is not edge-closed. `recover_quadrics` gates watertightness
	// and drops to the retag rung, so the honest result here is carrier
	// recovery with the face count unchanged. Closing that gap (a shared-chord-
	// aware merged triangulation) is the open follow-up.
	let torus_sdf = Torus::new(Vec3::ZERO, Vec3::Z, 12.0, 4.0);
	let tb = torus_sdf.bounds();
	let t1 = implicit_to_solid(&torus_sdf, tb, 0.4).expect("v1 torus bridge must succeed");
	let (t2, trep) = implicit_to_solid_recovered(&torus_sdf, tb, 0.4, 0.05).expect("v2 torus recovery must succeed");
	let (tvol1, tvol2) = (volume(&t1), volume(&t2));
	let tor = find_surface(&t2, |s| matches!(s, Surface::Torus { .. }));
	let Some(Surface::Torus { center, axis: taxis, major, minor }) = tor else {
		panic!("v2 torus solid must carry Surface::Torus, report {trep:?}");
	};
	// Measured on this deterministic pipeline: 33 331 facets in and out (the
	// chart merge reached 53 faces but its mesh carried 11 doubled edges, so
	// the pass refused it and took the retag rung), center 0.0023 mm off,
	// axis·Z = 1 − 1.3e-8, major 12.00107, minor 4.00273, vertex residual
	// 0.0085 mm, volumes bit-identical at 3792.3976.
	let torus_truth = 2.0 * std::f64::consts::PI.powi(2) * 12.0 * 16.0;
	let (t_err_in, t_err_out) = ((tvol1 - torus_truth).abs(), (tvol2 - torus_truth).abs());
	let torus_drift = (tvol2 - tvol1).abs() / tvol1;
	let torus_ok = trep.tori == 1
		&& trep.faces_after == trep.faces_before
		&& center.length() < 0.01
		&& taxis.dot(DVec3::Z).abs() > 1.0 - 1e-6
		&& (major - 12.0).abs() < 0.01
		&& (minor - 4.0).abs() < 0.01
		&& torus_drift < 0.005
		&& t_err_out <= t_err_in
		&& validate(&t2).is_valid()
		&& tessellate_default(&t2).is_watertight();
	assert!(
		cone_ok && torus_ok,
		"gate 6 — cone AND torus recovered from implicit sources, both with a real face-count COLLAPSE \
		 (the pre-2026-07-30 boundary-only tessellation pinned the cone at 126 faces and the torus at retag-only).\n\
		 cone ok={cone_ok}: report {rep:?} (want ≥500× face collapse), apex {apex:?} (truth (0,0,35), off {apex_err:.4}, bar 0.1), \
		 half-angle {half_angle:.5} (truth {truth_half:.5}, bar 1e-3), volume v1 {vol1:.3} / v2 {vol2:.3} / analytic {analytic:.3};\n\
		 torus ok={torus_ok} (carrier recovery, faces honestly UNCHANGED here — the chart merge reached 53 faces but \
		 its default mesh carried 11 doubled edges, so the watertightness gate took the retag rung; the clean \
		 builder torus DOES collapse 4608 → 16, see kernel-brep/tests/curved_faces.rs): report {trep:?}, center {center:?} (bar 0.01), \
		 axis·Z {:.8}, major {major:.5} (truth 12), minor {minor:.5} (truth 4, bars 0.01), volume {tvol1:.4} → {tvol2:.4} \
		 (drift {:.4}%, bar 0.5%); 2π²Rr² = {torus_truth:.4}, |err| {t_err_in:.4} → {t_err_out:.4} (merge must not be further from truth)",
		taxis.dot(DVec3::Z),
		torus_drift * 100.0
	);
}
