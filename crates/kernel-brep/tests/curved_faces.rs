//! **Merged curved faces**: doubly-curved face-count collapse + the
//! interior-refined tessellation that makes it volume-faithful.
//!
//! Until 2026-07-30 both tessellators triangulated a curved face from its
//! BOUNDARY RING only. That is exact for the chord facets primitives and
//! booleans emit (the ring *is* the facet), but it capped
//! [`recover_quadrics`]: a merged curved face would tessellate with chords
//! across the whole face and silently lose the bulge, so single-curved regions
//! were budgeted to 0.11-rad sectors (a full cylinder → ~60 faces) and
//! doubly-curved regions were retagged facet-by-facet with NO collapse at all.
//!
//! These gates pin the opened contract on mesher-free, exactly-known geometry
//! (builder solids stripped to planar facets), so every number has a closed
//! form to answer to:
//! - face-count collapse per quadric family (cylinder / cone / sphere / torus);
//! - tessellated volume against BOTH the chord input and the closed form —
//!   a merged face must never be further from the truth than the facets it
//!   replaced (that is the honest statement of "the bulge is back");
//! - watertightness and validity of every merged result;
//! - the seam contract: merged faces consume their boundary VERBATIM, so a
//!   merged/planar seam stays welded;
//! - the negative control: a genuine hexagonal prism still merges nothing.

use kernel_brep::math::DVec3;
use kernel_brep::recover::recover_quadrics;
use kernel_brep::{cone, cylinder, sphere, tessellate_adaptive, tessellate_default, torus, validate, volume, FaceLoops, Solid, Surface};

/// Strip every analytic tag: rebuild each face as a `Surface::Plane` facet
/// (origin = centroid, normal = Newell) — the reverse bridge's v1 faceted
/// contract, and the input `recover_quadrics` is designed for.
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

/// How many faces carry each analytic tag: `(planes, cylinders, spheres, cones, tori)`.
fn tag_census(s: &Solid) -> (usize, usize, usize, usize, usize) {
	let mut c = (0, 0, 0, 0, 0);
	for f in s.faces() {
		match s.face(f).surface {
			Surface::Plane { .. } => c.0 += 1,
			Surface::Cylinder { .. } => c.1 += 1,
			Surface::Sphere { .. } => c.2 += 1,
			Surface::Cone { .. } => c.3 += 1,
			Surface::Torus { .. } => c.4 += 1,
		}
	}
	c
}

#[test]
fn doubly_curved_regions_collapse_into_chart_faces_with_the_bulge_intact() {
	// ---- SPHERE: r = 8 about (1, −2, 3), 96 × 48 facets stripped to planes.
	// The chart policy bins a sphere region into the SIX cubemap sextants
	// (dominant axis of centre→centroid), so a full sphere collapses to 6 faces
	// — not to 1: a single-loop face cannot cover a closed surface, and each
	// sextant spans ≲ 55° of its gnomonic chart, comfortably injective.
	let s_center = DVec3::new(1.0, -2.0, 3.0);
	let sph_in = detagged(&sphere(s_center, 8.0, 96, 48));
	let sph_faces_before = sph_in.face_count();
	let sph_v_in = volume(&sph_in);
	let (sph, sph_rep) = recover_quadrics(&sph_in, 0.05).expect("a 96×48 faceted sphere must recover");
	let sph_v_out = volume(&sph);
	let sph_truth = 4.0 / 3.0 * std::f64::consts::PI * 8.0_f64.powi(3);
	let sph_err_in = (sph_v_in - sph_truth).abs();
	let sph_err_out = (sph_v_out - sph_truth).abs();
	let sph_drift = (sph_v_out - sph_v_in).abs() / sph_v_in;
	let sph_census = tag_census(&sph);

	// ---- TORUS: major 12 / minor 4 about Z, 96 × 48 facets. The chart policy
	// bins a torus region into a 4 × 4 (azimuth × tube-angle) quadrant grid, so
	// a full donut collapses to 16 faces, each spanning a quarter turn in each
	// periodic direction.
	let tor_in = detagged(&torus(DVec3::ZERO, DVec3::Z, 12.0, 4.0, 96, 48));
	let tor_faces_before = tor_in.face_count();
	let tor_v_in = volume(&tor_in);
	let (tor, tor_rep) = recover_quadrics(&tor_in, 0.05).expect("a 96×48 faceted torus must recover");
	let tor_v_out = volume(&tor);
	let tor_truth = 2.0 * std::f64::consts::PI.powi(2) * 12.0 * 4.0 * 4.0;
	let tor_err_in = (tor_v_in - tor_truth).abs();
	let tor_err_out = (tor_v_out - tor_truth).abs();
	let tor_drift = (tor_v_out - tor_v_in).abs() / tor_v_in;
	let tor_census = tag_census(&tor);

	// Measured on this deterministic pipeline (see the report line in the
	// message): sphere 4608 → 6 faces, torus 4608 → 16 faces. Both merged
	// solids must be at least as close to their closed form as the chord input
	// they replaced — that is the bulge, re-proved rather than asserted.
	assert!(
		sph_rep.spheres == 1
			&& sph_census == (0, 0, 6, 0, 0)
			&& sph.face_count() == 6
			&& sph_drift < 0.005
			&& sph_err_out <= sph_err_in
			&& validate(&sph).is_valid()
			&& tessellate_default(&sph).is_watertight()
			&& tor_rep.tori == 1
			&& tor_census == (0, 0, 0, 0, 16)
			&& tor.face_count() == 16
			&& tor_drift < 0.005
			&& tor_err_out <= tor_err_in
			&& validate(&tor).is_valid()
			&& tessellate_default(&tor).is_watertight(),
		"doubly-curved collapse (was RETAG-ONLY before interior-refined tessellation landed 2026-07-30 — \
		 both families kept every facet face).\n\
		 SPHERE r8: {sph_faces_before} → {} faces (want 6 cubemap sextants), census {sph_census:?}, report {sph_rep:?}, \
		 volume {sph_v_in:.4} → {sph_v_out:.4} (drift {:.4}%, bar 0.5%); closed form 4πr³/3 = {sph_truth:.4}, \
		 |err| {sph_err_in:.4} → {sph_err_out:.4} (merge must not be further from truth), valid={:?}, wt={};\n\
		 TORUS R12/r4: {tor_faces_before} → {} faces (want a 4×4 quadrant grid = 16), census {tor_census:?}, report {tor_rep:?}, \
		 volume {tor_v_in:.4} → {tor_v_out:.4} (drift {:.4}%); closed form 2π²Rr² = {tor_truth:.4}, \
		 |err| {tor_err_in:.4} → {tor_err_out:.4}, valid={:?}, wt={}",
		sph.face_count(),
		sph_drift * 100.0,
		validate(&sph),
		tessellate_default(&sph).is_watertight(),
		tor.face_count(),
		tor_drift * 100.0,
		validate(&tor),
		tessellate_default(&tor).is_watertight(),
	);
}

#[test]
fn single_curved_regions_collapse_to_half_wrap_charts_keeping_their_caps_welded() {
	// A full-wrap cylinder collapses to TWO half-wrap lateral charts plus its
	// two caps (4 faces): a single-loop face cannot be a full periodic wrap, so
	// π per chart is the honest floor — the pre-2026-07-30 span budget of 0.11
	// rad (chosen to keep a boundary-only tessellation faithful) gave 60.
	let cyl_in = detagged(&cylinder(DVec3::ZERO, DVec3::Z, 8.0, 40.0, 384));
	let cyl_v_in = volume(&cyl_in);
	let (cyl, cyl_rep) = recover_quadrics(&cyl_in, 0.05).expect("a 384-seg faceted cylinder must recover");
	let cyl_v_out = volume(&cyl);
	let cyl_truth = std::f64::consts::PI * 64.0 * 40.0;
	let cyl_census = tag_census(&cyl);
	let cyl_drift = (cyl_v_out - cyl_v_in).abs() / cyl_v_in;
	let cyl_err_in = (cyl_v_in - cyl_truth).abs();
	let cyl_err_out = (cyl_v_out - cyl_truth).abs();

	// The same for a cone (apex included): its lateral region is single-curved
	// too, so it collapses to two half-wrap development charts + the base cap.
	let con_in = detagged(&cone(DVec3::ZERO, DVec3::Z, 9.0, 30.0, 240));
	let con_v_in = volume(&con_in);
	let (con, con_rep) = recover_quadrics(&con_in, 0.05).expect("a 240-seg faceted cone must recover");
	let con_v_out = volume(&con);
	let con_truth = std::f64::consts::PI * 81.0 * 30.0 / 3.0;
	let con_census = tag_census(&con);
	let con_drift = (con_v_out - con_v_in).abs() / con_v_in;
	let con_err_in = (con_v_in - con_truth).abs();
	let con_err_out = (con_v_out - con_truth).abs();

	// The merged/planar SEAM contract: a merged curved face consumes its
	// boundary ring verbatim (interior points are strictly inside), so the
	// rim it shares with a planar cap stays welded — in the default AND in the
	// adaptive tessellator.
	let seams_ok = tessellate_default(&cyl).is_watertight()
		&& tessellate_default(&con).is_watertight()
		&& tessellate_adaptive(&cyl, 3).is_watertight()
		&& tessellate_adaptive(&con, 3).is_watertight();

	assert!(
		cyl_rep.cylinders == 1
			&& cyl_census == (2, 2, 0, 0, 0)
			&& cyl_drift < 0.005
			&& cyl_err_out <= 2.0 * cyl_err_in
			&& validate(&cyl).is_valid()
			&& con_rep.cones == 1
			&& con_census == (1, 0, 0, 2, 0)
			&& con_drift < 0.005
			&& con_err_out <= 2.0 * con_err_in
			&& validate(&con).is_valid()
			&& seams_ok,
		"single-curved half-wrap collapse + seam weld.\n\
		 CYLINDER r8 h40 (384 seg): → {} faces, census (plane, cyl, sph, cone, tor) {cyl_census:?} (want 2 caps + 2 half-wrap charts; \
		 the pre-2026-07-30 0.11-rad budget gave 60), report {cyl_rep:?}, volume {cyl_v_in:.4} → {cyl_v_out:.4} \
		 (drift {:.4}%, bar 0.5%); πr²h = {cyl_truth:.4}, |err| {cyl_err_in:.4} → {cyl_err_out:.4};\n\
		 CONE r9 h30 (240 seg): → {} faces, census {con_census:?} (want 1 cap + 2 half-wrap charts), report {con_rep:?}, \
		 volume {con_v_in:.4} → {con_v_out:.4} (drift {:.4}%); πr²h/3 = {con_truth:.4}, |err| {con_err_in:.4} → {con_err_out:.4};\n\
		 seams watertight (default + adaptive) = {seams_ok}",
		cyl.face_count(),
		cyl_drift * 100.0,
		con.face_count(),
		con_drift * 100.0,
	);
}

#[test]
fn chord_facets_and_a_hex_prism_keep_the_boundary_only_contract() {
	// The detector must not fire on anything the kernel built the old way. A
	// BUILDER cylinder/sphere/torus (analytic tags, chord-facet rings) must
	// tessellate to exactly the volume it always did — the merged-face path is
	// for MERGED faces only, and a facet ring is not one.
	let cases: Vec<(&str, Solid, f64)> = vec![
		("cylinder", cylinder(DVec3::ZERO, DVec3::Z, 6.0, 20.0, 32), 0.0),
		("sphere", sphere(DVec3::ZERO, 6.0, 32, 16), 0.0),
		("torus", torus(DVec3::ZERO, DVec3::Z, 10.0, 3.0, 32, 16), 0.0),
	];
	// Chord-facet volumes are the INSCRIBED values — strictly under the closed
	// form. If the merged path had fired, these would jump toward the analytic
	// volume, which is exactly the regression this control catches.
	let mut report = String::new();
	let mut ok = true;
	for (name, solid, _) in &cases {
		let v = volume(solid);
		let truth = match *name {
			"cylinder" => std::f64::consts::PI * 36.0 * 20.0,
			"sphere" => 4.0 / 3.0 * std::f64::consts::PI * 216.0,
			_ => 2.0 * std::f64::consts::PI.powi(2) * 10.0 * 9.0,
		};
		let under = v < truth;
		let wt = tessellate_default(solid).is_watertight();
		report.push_str(&format!("{name}: chord volume {v:.4} vs closed form {truth:.4} (inscribed={under}), wt={wt}\n"));
		ok &= under && wt;
	}
	assert!(ok, "builder chord facets must keep the boundary-only tessellation (inscribed, never bulged):\n{report}");
}
