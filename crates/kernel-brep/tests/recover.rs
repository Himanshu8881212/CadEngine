//! Analytic quadric recovery (reverse bridge v2) — B-rep-level gates.
//!
//! `recover_quadrics` is `coalesce_coplanar` generalized to quadrics: these
//! tests exercise it on mesher-free, exactly-known geometry (builder solids
//! whose analytic tags are STRIPPED to planar facets), so every pin has a
//! closed-form truth to compare against: recovered axis/radius/apex against
//! the builder's numbers, sector merging against the span budget, volume
//! conservation against the input's own tessellation, and the two honesty
//! gates — the hexagonal-prism NEGATIVE control (a 6-facet ring is NOT a
//! cylinder: pinned via the measured midpoint residual) and the builder-
//! cylinder NO-OP (already-analytic solids pass through structurally
//! unchanged).

use kernel_brep::math::DVec3;
use kernel_brep::recover::{fit_cylinder, fit_residual, recover_quadrics};
use kernel_brep::{cone, cylinder, extrude, sphere, tessellate_default, validate, volume, FaceLoops, Solid, Surface};
use kernel_core::math::DVec2;

/// Strip every analytic tag: rebuild each face as a `Surface::Plane` facet
/// (origin = centroid, normal = Newell), exactly what the reverse bridge's v1
/// faceted contract hands downstream.
fn detagged(s: &Solid) -> Solid {
	let positions: Vec<DVec3> = (0..s.vertex_count() as u32).map(|i| s.position(kernel_brep::VertexId(i))).collect();
	let faces: Vec<FaceLoops> = s
		.faces()
		.map(|f| {
			let face = s.face(f);
			let loops: Vec<Vec<u32>> = std::iter::once(face.outer)
				.chain(face.inner.iter().copied())
				.map(|lp| s.loop_half_edges(lp).iter().map(|&he| s.half_edge(he).origin.0).collect())
				.collect();
			let poly: Vec<DVec3> = loops[0].iter().map(|&v| positions[v as usize]).collect();
			let centroid = poly.iter().copied().sum::<DVec3>() / poly.len() as f64;
			let mut n = DVec3::ZERO;
			for i in 0..poly.len() {
				let (c, d) = (poly[i], poly[(i + 1) % poly.len()]);
				n.x += (c.y - d.y) * (c.z + d.z);
				n.y += (c.z - d.z) * (c.x + d.x);
				n.z += (c.x - d.x) * (c.y + d.y);
			}
			FaceLoops { loops, surface: Surface::Plane { origin: centroid, normal: n.normalize_or_zero() } }
		})
		.collect();
	Solid::from_faces_multiloop(positions, faces)
}

/// The first face carrying a given quadric kind, as (surface, count of faces
/// carrying any non-plane tag).
fn first_quadric(s: &Solid, pick: impl Fn(&Surface) -> bool) -> (Option<Surface>, usize) {
	let mut found = None;
	let mut curved = 0usize;
	for f in s.faces() {
		let surf = s.face(f).surface;
		if !matches!(surf, Surface::Plane { .. }) {
			curved += 1;
			if found.is_none() && pick(&surf) {
				found = Some(surf);
			}
		}
	}
	(found, curved)
}

#[test]
fn detagged_cylinder_recovers_carrier_and_merges_sectors() {
	// A 384-segment builder cylinder stripped to planar facets: vertices sit
	// EXACTLY on the r = 8 cylinder, so the fit truth is closed-form.
	let truth_r = 8.0;
	let solid = detagged(&cylinder(DVec3::ZERO, DVec3::Z, truth_r, 40.0, 384));
	let before = solid.face_count();
	let v_before = volume(&solid);
	let (rec, rep) = recover_quadrics(&solid, 0.05).expect("a finely faceted cylinder must recover");
	let v_after = volume(&rec);
	let drift = (v_after - v_before).abs() / v_before;
	let (cyl_surf, _) = first_quadric(&rec, |s| matches!(s, Surface::Cylinder { .. }));
	let Some(Surface::Cylinder { origin, axis, radius }) = cyl_surf else {
		panic!("recovered solid must carry a Surface::Cylinder face, report {rep:?}");
	};
	let axis_align = axis.dot(DVec3::Z).abs();
	let origin_off_axis = DVec3::new(origin.x, origin.y, 0.0).length();
	// Idempotence: a second pass finds nothing new.
	let (rec2, rep2) = recover_quadrics(&rec, 0.05).expect("recovery must be idempotent");
	// Measured on this deterministic pipeline: 386 → 4 faces — TWO half-wrap
	// chart sectors (a single-loop face cannot be a full periodic wrap) + 2 caps.
	// UPDATED 2026-07-30 from the previous pin of 60 faces (58 span-budgeted
	// sectors + 2 caps): interior-refined tessellation of merged curved faces
	// (tessellate.rs) replaced the 0.11-rad boundary-only span budget, so the
	// collapse is 15× deeper AND the volume is more faithful — 8042.118258 →
	// 8042.123680 mm³, drift 0.00007% (was 0.0647%) against a 0.5% bar, and the
	// error against πr²h = 8042.477193 shrinks from 0.358935 to 0.353513 mm³.
	// Fitted r 7.999946 (the Kåsa point set includes facet centroids, which sit
	// one sagitta ≈ 2.7e-4 inside the chords — a 5.4e-5 bias on r, reported not
	// hidden).
	let closed_form = std::f64::consts::PI * truth_r * truth_r * 40.0;
	let err_in = (v_before - closed_form).abs();
	let err_out = (v_after - closed_form).abs();
	assert!(
		rep.cylinders == 1
			&& rep.spheres + rep.cones + rep.tori + rep.planes == 0
			&& rep.faces_before == before
			&& rep.faces_after == rec.face_count()
			&& rep.faces_after == 4
			&& (radius - truth_r).abs() < 1e-4
			&& axis_align > 1.0 - 1e-12
			&& origin_off_axis < 1e-9
			&& rep.max_fit_residual < 1e-4
			&& drift < 0.005
			&& err_out < 4.0 * err_in
			&& validate(&rec).is_valid()
			&& tessellate_default(&rec).is_watertight()
			&& rep2 == kernel_brep::recover::RecoveryReport { faces_before: rec.face_count(), faces_after: rec2.face_count(), ..Default::default() }
			&& rec2.face_count() == rec.face_count(),
		"detagged 384-seg cylinder recovery: report {rep:?} (want 1 cylinder, faces {before} → 4 = 2 half-wrap chart sectors + 2 caps; \
		 the pre-2026-07-30 boundary-only tessellation pinned 60), \
		 fitted r {radius:.12} (truth {truth_r}, bar 1e-4), |axis·Z| {axis_align:.15}, origin off-axis {origin_off_axis:.3e} mm, \
		 volume {v_before:.6} → {v_after:.6} mm³ (drift {:.5}%, bar 0.5%; closed form {closed_form:.6}, |err| {err_in:.6} → {err_out:.6}), \
		 validity {:?}, second pass {rep2:?}",
		drift * 100.0,
		validate(&rec)
	);
}

#[test]
fn detagged_sphere_and_cone_recover_their_carriers() {
	// Sphere: vertices exactly on r = 6 about (1, 2, 3). Doubly-curved regions
	// now COLLAPSE into cubemap chart faces (2026-07-30): the six dominant-axis
	// sextants, each triangulated with interior points on the exact sphere. The
	// previous pin asserted retag-only (faces unchanged at 1152) because the
	// boundary-ring-only tessellators lost the bulge of any merged doubly-curved
	// face; that limit is lifted, so the pin is now the collapse itself PLUS the
	// honest fidelity claim: the merged solid must be strictly CLOSER to the
	// closed form 4πr³/3 than its own chord input was.
	let center = DVec3::new(1.0, 2.0, 3.0);
	let sph = detagged(&sphere(center, 6.0, 48, 24));
	let before_s = sph.face_count();
	let v_before_s = volume(&sph);
	let (rec_s, rep_s) = recover_quadrics(&sph, 0.05).expect("a 48×24 faceted sphere must recover");
	let (sph_surf, _) = first_quadric(&rec_s, |s| matches!(s, Surface::Sphere { .. }));
	let Some(Surface::Sphere { center: c_fit, radius: r_fit }) = sph_surf else {
		panic!("recovered sphere solid must carry Surface::Sphere, report {rep_s:?}");
	};
	let v_after_s = volume(&rec_s);
	let sphere_truth = 4.0 / 3.0 * std::f64::consts::PI * 6.0_f64.powi(3);
	let err_in_s = (v_before_s - sphere_truth).abs();
	let err_out_s = (v_after_s - sphere_truth).abs();
	let drift_s = (v_after_s - v_before_s).abs() / v_before_s;
	// Measured on this deterministic pipeline: 1152 → 6 chart faces, fitted
	// center exact to 1e-13, r 5.996107, volume 898.337819 → 901.022703 mm³
	// against the closed form 904.778684 — the chord input sits 0.712% under the
	// true sphere and the merged/refined solid 0.415% under, so the 0.2989%
	// drift IS the fidelity gain (|err| 6.44 → 3.76 mm³: the merged face honours
	// the analytic carrier its tag claims). The 0.5% gate therefore also bounds
	// how coarse an input may be and still merge; a coarser one falls back to
	// the retag rung by design.
	assert!(
		rep_s.spheres == 1
			&& rep_s.cylinders + rep_s.cones + rep_s.tori + rep_s.planes == 0
			&& rep_s.faces_after == 6
			&& rep_s.faces_after == rec_s.face_count()
			&& (c_fit - center).length() < 5e-3
			&& (r_fit - 6.0).abs() < 5e-3
			&& drift_s < 0.005
			&& err_out_s < err_in_s
			&& validate(&rec_s).is_valid()
			&& tessellate_default(&rec_s).is_watertight(),
		"detagged sphere recovery COLLAPSES to cubemap chart faces (was retag-only, faces unchanged at {before_s}, before \
		 interior-refined tessellation landed 2026-07-30): report {rep_s:?} (want 1 sphere, faces {before_s} → 6), \
		 fitted center {c_fit:?} (truth {center:?}, off by {:.6} mm), radius {r_fit:.6} (truth 6), \
		 volume {v_before_s:.6} → {v_after_s:.6} (drift {:.4}%, bar 0.5%); closed form 4πr³/3 = {sphere_truth:.6}, \
		 |err| {err_in_s:.6} → {err_out_s:.6} (the merge must be strictly closer to truth)",
		(c_fit - center).length(),
		drift_s * 100.0
	);

	// Cone: 240 segments, base r = 9 at z = 0, apex at z = 30 → half-angle
	// atan(9/30). Single-curved: sectors merge like the cylinder.
	let con = detagged(&cone(DVec3::ZERO, DVec3::Z, 9.0, 30.0, 240));
	let before_c = con.face_count();
	let v_before_c = volume(&con);
	let (rec_c, rep_c) = recover_quadrics(&con, 0.05).expect("a 240-seg faceted cone must recover");
	let (cone_surf, _) = first_quadric(&rec_c, |s| matches!(s, Surface::Cone { .. }));
	let Some(Surface::Cone { apex, axis, half_angle }) = cone_surf else {
		panic!("recovered cone solid must carry Surface::Cone, report {rep_c:?}");
	};
	let v_after_c = volume(&rec_c);
	let drift_c = (v_after_c - v_before_c).abs() / v_before_c;
	let truth_half = (9.0_f64 / 30.0).atan();
	assert!(
		rep_c.cones == 1
			&& rep_c.cylinders + rep_c.spheres + rep_c.tori + rep_c.planes == 0
			&& rep_c.faces_after < before_c / 2
			&& (apex - DVec3::new(0.0, 0.0, 30.0)).length() < 0.02
			&& axis.dot(-DVec3::Z) > 1.0 - 1e-6
			&& (half_angle - truth_half).abs() < 2e-3
			&& drift_c < 0.005
			&& validate(&rec_c).is_valid()
			&& tessellate_default(&rec_c).is_watertight(),
		"detagged 240-seg cone recovery: report {rep_c:?} (want 1 cone, faces {before_c} → under {}), \
		 apex {apex:?} (truth (0,0,30), off {:.6}), axis·(−Z) {:.9}, half-angle {half_angle:.6} (truth {truth_half:.6}), \
		 volume {v_before_c:.6} → {v_after_c:.6} (drift {:.5}%)",
		before_c / 2,
		(apex - DVec3::new(0.0, 0.0, 30.0)).length(),
		axis.dot(-DVec3::Z),
		drift_c * 100.0
	);
}

#[test]
fn hexagonal_prism_is_not_a_cylinder_negative_control() {
	// Gate 4: a GENUINE hexagonal prism (6 flats from a builder) must recover
	// ZERO quadrics at tight tol. The discriminator is the sagitta-aware
	// residual: all 12 corners lie EXACTLY on the circumscribed r = 10
	// cylinder, but the flat-side midpoints sit at the inradius r·cos 30°. The
	// LSQ radius averages 4 vertex samples (r = 10) and 1 centroid sample
	// (inradius) per wall, r̂ = r·(4 + cos 30°)/5, so the best candidate's
	// residual has the closed form r̂ − r·cos 30° = 0.8·r·(1 − cos 30°) =
	// 1.0718 mm — 21× over tol. A 6-facet ring is not a cylinder.
	let r_c = 10.0_f64;
	let hexagon: Vec<DVec2> = (0..6)
		.map(|k| {
			let a = std::f64::consts::TAU * k as f64 / 6.0;
			DVec2::new(r_c * a.cos(), r_c * a.sin())
		})
		.collect();
	let prism = extrude(&hexagon, 12.0);
	let tol = 0.05;
	let (out, rep) = recover_quadrics(&prism, tol).expect("the no-recovery path must still succeed");
	// Direct fit of the best cylinder candidate through the six wall faces.
	let mut samples: Vec<(DVec3, DVec3)> = Vec::new();
	let mut probes: Vec<DVec3> = Vec::new();
	for f in prism.faces() {
		let Surface::Plane { normal, .. } = prism.face(f).surface else { unreachable!("extrude emits planes") };
		if normal.z.abs() > 0.5 {
			continue; // caps
		}
		let poly = prism.face_polygon(f);
		let centroid = poly.iter().copied().sum::<DVec3>() / poly.len() as f64;
		samples.push((centroid, normal));
		for (i, &p) in poly.iter().enumerate() {
			samples.push((p, normal));
			probes.push(p);
			probes.push((p + poly[(i + 1) % poly.len()]) * 0.5);
		}
		probes.push(centroid);
	}
	let best = fit_cylinder(&samples).expect("a best-fit cylinder candidate exists even for a hex prism");
	let residual = fit_residual(&best, &probes);
	let truth_residual = 0.8 * r_c * (1.0 - (30.0_f64).to_radians().cos());
	assert!(
		rep == kernel_brep::recover::RecoveryReport { faces_before: 8, faces_after: 8, ..Default::default() }
			&& out.face_count() == 8
			&& residual > tol
			&& (residual - truth_residual).abs() < 1e-6,
		"a 6-facet ring is NOT a cylinder: recover report {rep:?} (want zero quadrics, 8 faces untouched); \
		 best-fit cylinder residual {residual:.6} mm (closed form 0.8·r·(1−cos30°) = {truth_residual:.6}) \
		 exceeds tol {tol} by {:.1}×",
		residual / tol
	);
}

#[test]
fn recovery_carries_provenance_through_the_rebuild() {
	// Same policy as `coalesce_coplanar` (see `FaceName`'s doc): an unmerged
	// face keeps its name exactly, a merged face inherits the
	// lexicographically-least constituent name, and the pass invents nothing —
	// so `recover_quadrics` is no longer finishing-only either.
	//
	// A detagged cylinder carrying PRIMITIVE names: the 384 lateral facets merge
	// into 2 half-wrap charts, so those two faces must carry the least name
	// among their constituents, and the 2 caps (untouched) keep theirs exactly.
	let named_in = detagged(&cylinder(DVec3::ZERO, DVec3::Z, 8.0, 40.0, 384)).with_primitive_names();
	let caps_in: Vec<kernel_brep::FaceName> = named_in
		.faces()
		.filter(|&f| matches!(named_in.face(f).surface, Surface::Plane { origin, normal } if normal.z.abs() > 0.9 && origin.z.abs() < 1e-9 || normal.z.abs() > 0.9))
		.filter_map(|f| named_in.face_name(f))
		.collect();
	let (rec, rep) = recover_quadrics(&named_in, 0.05).expect("named cylinder must recover");
	let names_in: std::collections::BTreeSet<kernel_brep::FaceName> = named_in.faces().filter_map(|f| named_in.face_name(f)).collect();
	let names_out: std::collections::BTreeSet<kernel_brep::FaceName> = rec.faces().filter_map(|f| rec.face_name(f)).collect();
	let all_named = rec.faces().all(|f| rec.face_name(f).is_some());
	let subset = names_out.is_subset(&names_in);
	// The caps are re-emitted verbatim, so their exact names must survive.
	let caps_kept = caps_in.iter().filter(|n| names_out.contains(n)).count();
	// Determinism: two runs give the same names in the same order.
	let (rec2, _) = recover_quadrics(&named_in, 0.05).expect("named cylinder must recover twice");
	let order1: Vec<Option<kernel_brep::FaceName>> = rec.faces().map(|f| rec.face_name(f)).collect();
	let order2: Vec<Option<kernel_brep::FaceName>> = rec2.faces().map(|f| rec2.face_name(f)).collect();

	// An UNNAMED input must stay unnamed (all-or-nothing carry, heal's rule).
	let plain = detagged(&cylinder(DVec3::ZERO, DVec3::Z, 8.0, 40.0, 384));
	let (plain_out, _) = recover_quadrics(&plain, 0.05).expect("unnamed cylinder must recover");
	let stays_unnamed = plain_out.faces().all(|f| plain_out.face_name(f).is_none());

	assert!(
		all_named && subset && caps_kept == 2 && order1 == order2 && stays_unnamed && rec.face_count() == 4,
		"recover_quadrics provenance policy (was: names reset by the rebuild ⇒ finishing-pass only): \
		 every rebuilt face named={all_named}, names ⊆ input names={subset} ({} in → {} out), \
		 both caps kept their exact name: {caps_kept}/2, deterministic name order={}, \
		 unnamed input stays unnamed={stays_unnamed}, faces {} → {} (report {rep:?})",
		names_in.len(),
		names_out.len(),
		order1 == order2,
		rep.faces_before,
		rec.face_count()
	);
}

#[test]
fn builder_cylinder_is_a_structural_no_op() {
	// Gate 5: an already-analytic solid (builder cylinder: lateral faces carry
	// Surface::Cylinder, caps are single planes) has no planar facet regions
	// to recover — the pass must return it structurally unchanged.
	let solid = cylinder(DVec3::new(2.0, -3.0, 1.0), DVec3::Z, 6.0, 20.0, 24);
	let (out, rep) = recover_quadrics(&solid, 0.05).expect("no-op recovery must succeed");
	assert!(
		out.face_count() == solid.face_count()
			&& out.vertex_count() == solid.vertex_count()
			&& out.edge_count() == solid.edge_count()
			&& rep == kernel_brep::recover::RecoveryReport {
				faces_before: solid.face_count(),
				faces_after: solid.face_count(),
				..Default::default()
			},
		"an already-analytic builder cylinder must pass through unchanged: {} faces / {} verts / {} edges in, \
		 {} / {} / {} out, report {rep:?}",
		solid.face_count(),
		solid.vertex_count(),
		solid.edge_count(),
		out.face_count(),
		out.vertex_count(),
		out.edge_count()
	);
}
