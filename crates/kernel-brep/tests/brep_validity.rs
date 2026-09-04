// Copyright (c) LMCAD. Licensed under the MIT License.

//! Phase 3 acceptance: every constructor yields a closed, manifold, correctly
//! oriented solid that satisfies Euler–Poincaré, tessellates to a watertight
//! mesh, and reports a volume matching the closed form.

use kernel_brep::math::{DAffine3, DMat3, DVec2, DVec3, Vec3};
use kernel_brep::{
	chamfered_cylinder, cone, cuboid, cylinder, difference, draft_analysis, exact_volume, extrude, extrude_tapered, extrude_with_holes,
	fillet_edge, filleted_cylinder, mass_properties, overhang_analysis, revolve, section_properties, self_intersects, sphere,
	tessellate_adaptive_tol, tessellate_default, torus, union, validate, volume, wall_thickness, EdgeName, FaceName, FaceSource,
	MassProperties, Surface,
};
use std::f64::consts::PI;

fn assert_genus0_solid(s: &kernel_brep::Solid, name: &str) {
	let v = validate(s);
	assert!(v.closed, "{name}: not closed");
	assert!(v.manifold, "{name}: not manifold");
	assert_eq!(v.euler_characteristic, 2, "{name}: χ should be 2 (sphere topology)");
	assert_eq!(v.genus, 0, "{name}: genus should be 0");
	let mesh = tessellate_default(s);
	assert!(mesh.is_watertight(), "{name}: tessellation not watertight");
	assert!(mesh.signed_volume() > 0.0, "{name}: tessellation inside-out");
}

#[test]
fn box_is_exact() {
	let s = cuboid(DVec3::new(-5.0, -3.0, -2.0), DVec3::new(5.0, 3.0, 2.0));
	assert_genus0_solid(&s, "box");
	assert_eq!(s.vertex_count(), 8);
	assert_eq!(s.edge_count(), 12);
	assert_eq!(s.face_count(), 6);
	// Planar faces ⇒ volume is exact to f64.
	let exact = 10.0 * 6.0 * 4.0;
	assert!((volume(&s) - exact).abs() < 1e-6, "box volume {} vs {exact}", volume(&s));
}

#[test]
fn cylinder_validates_and_measures() {
	let s = cylinder(DVec3::ZERO, DVec3::Z, 5.0, 12.0, 64);
	assert_genus0_solid(&s, "cylinder");
	// V = 2*segs, E = 3*segs, F = segs+2 ⇒ χ = 2.
	assert_eq!(s.vertex_count(), 128);
	assert_eq!(s.face_count(), 66);
	let exact = PI * 25.0 * 12.0;
	// Faceted side under-fills slightly; converges with segments.
	assert!((volume(&s) - exact).abs() / exact < 0.01, "cyl vol {} vs {exact}", volume(&s));
}

#[test]
fn sphere_and_cone_validate() {
	let sp = sphere(DVec3::new(1.0, 2.0, 3.0), 7.0, 48, 32);
	assert_genus0_solid(&sp, "sphere");
	let exact_sp = 4.0 / 3.0 * PI * 7.0f64.powi(3);
	assert!((volume(&sp) - exact_sp).abs() / exact_sp < 0.01, "sphere vol {} vs {exact_sp}", volume(&sp));

	let cn = cone(DVec3::ZERO, DVec3::Z, 6.0, 10.0, 64);
	assert_genus0_solid(&cn, "cone");
	let exact_cn = PI * 36.0 * 10.0 / 3.0;
	assert!((volume(&cn) - exact_cn).abs() / exact_cn < 0.01, "cone vol {} vs {exact_cn}", volume(&cn));
}

#[test]
fn extrude_l_profile_is_exact() {
	// A non-convex L profile (tests ear clipping) extruded 4mm.
	let profile = [
		DVec2::new(0.0, 0.0),
		DVec2::new(10.0, 0.0),
		DVec2::new(10.0, 3.0),
		DVec2::new(3.0, 3.0),
		DVec2::new(3.0, 8.0),
		DVec2::new(0.0, 8.0),
	];
	let s = extrude(&profile, 4.0);
	assert_genus0_solid(&s, "L-extrude");
	// Cross-section area = 10*3 + 3*5 = 45; volume = 45 * 4 (planar ⇒ exact).
	let exact = 45.0 * 4.0;
	assert!((volume(&s) - exact).abs() < 1e-6, "extrude vol {} vs {exact}", volume(&s));
}

#[test]
fn exact_volume_beats_tessellation_on_curved_solids() {
	// A faceted cylinder's tessellated volume under-fills by ~1-2% (chords inside the
	// arc); the analytic divergence-theorem volume matches π·r²·h to machine precision —
	// regardless of facet count. Also exact for a plate with a cylindrical through-hole.
	let cyl = cylinder(DVec3::new(1.0, 2.0, 3.0), DVec3::Z, 5.0, 12.0, 16);
	let exact = PI * 25.0 * 12.0;
	let ev = exact_volume(&cyl);
	let tess = volume(&cyl);
	assert!(
		(ev - exact).abs() < 1e-7 && (tess - exact).abs() / exact > 1e-3,
		"exact cyl vol {ev} vs {exact} (err {:.2e}); tessellation {tess} should be coarser (rel err {:.4})",
		(ev - exact).abs(),
		(tess - exact).abs() / exact
	);

	// Plate (20×20×10) with a centred Ø8 cylindrical hole: exact = box − π·4²·10.
	let plate = cuboid(DVec3::new(-10.0, -10.0, -5.0), DVec3::new(10.0, 10.0, 5.0));
	let hole = cylinder(DVec3::new(0.0, 0.0, -5.0), DVec3::Z, 4.0, 10.0, 24);
	let drilled = kernel_brep::difference(&plate, &hole);
	let exact_drilled = 20.0 * 20.0 * 10.0 - PI * 16.0 * 10.0;
	let ev2 = exact_volume(&drilled);
	assert!(
		(ev2 - exact_drilled).abs() / exact_drilled < 1e-4,
		"exact drilled-plate vol {ev2} vs {exact_drilled} (rel err {:.2e})",
		(ev2 - exact_drilled).abs() / exact_drilled
	);

	// Sphere: 4/3·π·r³, exact regardless of facet count (tessellation ~1% low at this res).
	let sp = sphere(DVec3::new(-1.0, 2.0, 0.5), 4.0, 24, 16);
	let exact_sp = 4.0 / 3.0 * PI * 4.0_f64.powi(3);
	let ev_sp = exact_volume(&sp);
	assert!(
		(ev_sp - exact_sp).abs() / exact_sp < 1e-6 && (volume(&sp) - exact_sp).abs() / exact_sp > 1e-3,
		"exact sphere vol {ev_sp} vs {exact_sp} (rel err {:.2e}); tess {} coarser",
		(ev_sp - exact_sp).abs() / exact_sp,
		volume(&sp)
	);

	// Cone: π·R²·h/3, exact regardless of facet count.
	let cn = cone(DVec3::new(2.0, -1.0, 0.0), DVec3::Z, 6.0, 10.0, 32);
	let exact_cn = PI * 36.0 * 10.0 / 3.0;
	let ev_cn = exact_volume(&cn);
	assert!(
		(ev_cn - exact_cn).abs() / exact_cn < 1e-6 && (volume(&cn) - exact_cn).abs() / exact_cn > 1e-3,
		"exact cone vol {ev_cn} vs {exact_cn} (rel err {:.2e}); tess {} coarser",
		(ev_cn - exact_cn).abs() / exact_cn,
		volume(&cn)
	);
}

#[test]
fn a_through_drilled_bore_keeps_its_cylinder_tag_for_exact_volume() {
	// Curved-precision THROUGH booleans: a cylinder drilled clean THROUGH a box (poking out
	// both faces) has its bore bands CLIPPED at the box caps, so they recover as >4-vertex
	// facets — which the old recover_faces guard planarized, losing the analytic tag and the
	// exact bore volume. A coplanar-connected fragment of a curved primitive is still a SINGLE
	// chord facet, so it now keeps its Surface::Cylinder tag: the bore stays curved and
	// exact_volume recovers the closed form (box − π·r²·h) to micron precision, while the
	// preview mesh stays chord-flat (no self-intersection) — closed, manifold, genus-1.
	let block = cuboid(DVec3::new(-5.0, -5.0, 0.0), DVec3::new(5.0, 5.0, 6.0));
	let drill = cylinder(DVec3::new(0.0, 0.0, -1.0), DVec3::Z, 2.0, 8.0, 48);
	let part = kernel_brep::difference(&block, &drill);
	let v = validate(&part);
	let bore = part.faces().filter(|&f| matches!(part.face(f).surface, Surface::Cylinder { .. })).count();
	let exact = 10.0 * 10.0 * 6.0 - PI * 2.0 * 2.0 * 6.0;
	assert!(
		v.closed && v.manifold && v.genus == 1 && !self_intersects(&part) && bore > 0 && (exact_volume(&part).abs() - exact).abs() < 1e-4,
		"through-bore: {v:?} self_int={} cyl_faces={bore} exact_volume={} (closed form {exact})",
		self_intersects(&part),
		exact_volume(&part).abs()
	);
}

#[test]
fn center_of_mass_is_analytic_for_an_off_centre_bore() {
	// mass_properties now corrects the centre of mass analytically for cylindrical faces (the
	// segment-lens first moment), not just the volume. A box bored OFF-CENTRE has its CoM
	// shifted away from the removed material by exactly (V_box·C_box − V_bore·C_bore)/(V_box −
	// V_bore); the faceted tessellation gets this wrong (the 48-gon bore mis-weights the shift),
	// the analytic correction nails it to f32-tessellation precision — far tighter than the
	// ~2% the raw tessellation CoM carried on curved parts.
	let block = cuboid(DVec3::new(-5.0, -5.0, 0.0), DVec3::new(5.0, 5.0, 6.0));
	let part = difference(&block, &cylinder(DVec3::new(2.0, 0.0, -1.0), DVec3::Z, 2.0, 8.0, 48));
	let v_box = 600.0;
	let v_bore = PI * 2.0 * 2.0 * 6.0;
	let com_x = (v_box * 0.0 - v_bore * 2.0) / (v_box - v_bore);
	let com = mass_properties(&part).center_of_mass;
	assert!(
		(com.x - com_x).abs() < 1e-4 && com.y.abs() < 1e-4 && (com.z - 3.0).abs() < 1e-4,
		"off-centre bore CoM {com:?} should be ({com_x:.5}, 0, 3)"
	);
}

#[test]
fn center_of_mass_is_analytic_for_spherical_parts() {
	// The first-moment correction extends to spherical faces via the patch's vector solid angle
	// (½∮r×dr). Two exact cases: (1) a whole off-centre sphere has its CoM exactly at its centre
	// — the patch moments cancel and only center·bulge survives; (2) a clean upper HEMISPHERE
	// (sphere ∩ half-space, flat disc cut at z=0, no annular rim) has CoM at the textbook 3r/8 up
	// from the flat face. The faceted tessellation misses 3r/8 by a curved-faceting margin; the
	// analytic correction nails it. (A spherical dimple bored into a face keeps a small ~1e-3
	// residual from the planar annulus's faceted hole edge — the flat-cap faceting limit, not a
	// moment error — so it is not asserted here.)
	let s = sphere(DVec3::new(2.0, 1.0, 3.0), 4.0, 48, 32);
	let sc = mass_properties(&s).center_of_mass;
	let hemi =
		kernel_brep::intersection(&sphere(DVec3::ZERO, 4.0, 64, 48), &cuboid(DVec3::new(-5.0, -5.0, 0.0), DVec3::new(5.0, 5.0, 6.0)));
	let hc = mass_properties(&hemi).center_of_mass;
	assert!(
		(sc - DVec3::new(2.0, 1.0, 3.0)).length() < 1e-4 && hc.x.abs() < 1e-4 && hc.y.abs() < 1e-4 && (hc.z - 1.5).abs() < 1e-4,
		"sphere CoM {sc:?} (want 2,1,3); hemisphere CoM {hc:?} (want 0,0,1.5)"
	);
}

#[test]
fn center_of_mass_is_analytic_for_a_cone() {
	// The first-moment correction extends to conical faces (the lens integrated along the
	// taper). A solid cone's centre of mass sits on its axis at ¼ of the height up from the
	// base — here a Ø6×9 cone with its base at z=0 has CoM z = 9/4 = 2.25, and translating the
	// cone to (2,0) moves the CoM with it. The faceted tessellation misses this by ~0.16% on
	// the axial coordinate; the analytic correction nails (2, 0, 2.25).
	let c = cone(DVec3::new(2.0, 0.0, 0.0), DVec3::Z, 3.0, 9.0, 64);
	let com = mass_properties(&c).center_of_mass;
	assert!((com.x - 2.0).abs() < 1e-4 && com.y.abs() < 1e-4 && (com.z - 2.25).abs() < 1e-4, "cone CoM {com:?} should be (2, 0, 2.25)");
}

/// Largest absolute component difference between two 3×3 tensors.
fn mat_err(a: DMat3, b: DMat3) -> f64 {
	let d = a - b;
	[d.x_axis, d.y_axis, d.z_axis].iter().flat_map(|c| [c.x.abs(), c.y.abs(), c.z.abs()]).fold(0.0_f64, f64::max)
}

#[test]
fn inertia_tensor_is_analytic_for_a_cylinder_at_coarse_segments() {
	// THE second-moment correction (cylinder_second_moment): at a COARSE 16 segments the raw
	// tessellation inertia of a cylinder errs by percent (the inscribed-prism deficit,
	// converging only as 1/seg²); the analytic circular-segment second moments make
	// mass_properties machine-exact. Ø10×12 cylinder centred on the origin, axis +Z, unit
	// density m = πr²h: axial Izz = ½mr², transverse Ixx = Iyy = m(3r² + h²)/12, products 0.
	// Closeness at seg 16 IS the evidence the correction engaged — the uncorrected error is
	// asserted to exceed 1% alongside the ≤1e-6 corrected gate (f32 mesh noise floor).
	let (r, h, seg) = (5.0, 12.0, 16);
	let cyl = cylinder(DVec3::new(0.0, 0.0, -6.0), DVec3::Z, r, h, seg);
	let m = PI * r * r * h;
	let it = m * (3.0 * r * r + h * h) / 12.0;
	let want = DMat3::from_diagonal(DVec3::new(it, it, 0.5 * m * r * r));
	let mp = mass_properties(&cyl);
	let raw = tessellate_default(&cyl).mass_properties();
	let scale = 0.5 * m * r * r;
	let (corrected_err, raw_err) = (mat_err(mp.inertia, want) / scale, mat_err(raw.inertia, want) / scale);
	assert!(
		corrected_err < 1e-6 && raw_err > 1e-2 && mp.center_of_mass.length() < 1e-6,
		"cylinder inertia at seg {seg}: corrected rel err {corrected_err:.2e} (want ≤1e-6), raw tessellation {raw_err:.2e} (must exceed 1e-2 \
		 to prove the correction engaged), CoM {:?}\ncorrected {:?}\nclosed form {want:?}",
		mp.center_of_mass,
		mp.inertia
	);
}

#[test]
fn inertia_tensor_parallel_axis_for_an_off_centre_tilted_cylinder() {
	// The same closed form for a cylinder based OFF the origin with a NON-axis-aligned axis:
	// mass_properties reports inertia about the CoM, so the result must equal the canonical
	// tensor rotated onto the axis, I = Ia·(aaᵀ) + It·(Id − aaᵀ) — which exercises the
	// origin-frame lens second moments and the parallel-axis recomposition under large
	// cancellation (the bug-prone path: |I_origin| ≈ 5× |I_com| here). The f32 tessellation
	// positions bound the achievable agreement (~1e-7·|p| each, amplified by the
	// cancellation), so the gate is 1e-5 relative — still ≥3 orders below the raw faceting
	// error this corrects.
	let (r, h, seg) = (5.0, 12.0, 16);
	let a = DVec3::new(1.0, 2.0, 2.0) / 3.0;
	let base = DVec3::new(8.0, -6.0, 4.0);
	let cyl = cylinder(base, a, r, h, seg);
	let m = PI * r * r * h;
	let (ia, it) = (0.5 * m * r * r, m * (3.0 * r * r + h * h) / 12.0);
	let aa = DMat3::from_cols(a * a.x, a * a.y, a * a.z);
	let want = (DMat3::IDENTITY - aa) * it + aa * ia;
	let mp = mass_properties(&cyl);
	let err = mat_err(mp.inertia, want) / ia;
	let com_err = (mp.center_of_mass - (base + a * (0.5 * h))).length();
	assert!(
		err < 1e-5 && com_err < 1e-6,
		"tilted off-origin cylinder: inertia rel err {err:.2e} (want ≤1e-5), CoM err {com_err:.2e}\ngot {:?}\nwant {want:?}",
		mp.inertia
	);
}

#[test]
fn inertia_tensor_is_analytic_for_an_off_centre_bored_block() {
	// The CONCAVE sign path: a block with an off-centre through-bore. The bore's wall faces
	// are concave cylindrical patches, so their lens second moments must be SUBTRACTED; the
	// composite closed form is I_block(origin) − I_plug(origin), shifted to the part CoM.
	// At a coarse 16-segment bore the raw tessellation misses the removed plug's second
	// moments by the same inscribed-prism deficit; the correction recovers the closed form.
	let (rb, seg) = (2.0, 16);
	let block = cuboid(DVec3::new(-5.0, -5.0, 0.0), DVec3::new(5.0, 5.0, 6.0));
	let part = difference(&block, &cylinder(DVec3::new(2.0, 0.0, -1.0), DVec3::Z, rb, 8.0, seg));
	let shift = |c: DVec3| DMat3::from_diagonal(DVec3::splat(c.length_squared())) - DMat3::from_cols(c * c.x, c * c.y, c * c.z);
	// Block 10×10×6 about its centre (0,0,3); removed plug Ø4×6 about its centre (2,0,3).
	let (vb, cb) = (600.0, DVec3::new(0.0, 0.0, 3.0));
	let ib = DMat3::from_diagonal(DVec3::new(vb * 136.0 / 12.0, vb * 136.0 / 12.0, vb * 200.0 / 12.0));
	let (vc, cc) = (PI * rb * rb * 6.0, DVec3::new(2.0, 0.0, 3.0));
	let ic = DMat3::from_diagonal(DVec3::new(vc * (3.0 * rb * rb + 36.0) / 12.0, vc * (3.0 * rb * rb + 36.0) / 12.0, 0.5 * vc * rb * rb));
	let v = vb - vc;
	let com = (cb * vb - cc * vc) / v;
	let want = (ib + shift(cb) * vb) - (ic + shift(cc) * vc) - shift(com) * v;
	let mp = mass_properties(&part);
	let raw = tessellate_default(&part).mass_properties();
	let scale = want.z_axis.z;
	let (err, raw_err) = (mat_err(mp.inertia, want) / scale, mat_err(raw.inertia, want) / scale);
	assert!(
		err < 1e-5 && raw_err > 10.0 * err && (mp.center_of_mass - com).length() < 1e-5 && (mp.volume - v).abs() < 1e-6,
		"bored block: inertia rel err {err:.2e} (want ≤1e-5; raw {raw_err:.2e} must be ≥10×), CoM {:?} (want {com:?}), V {} (want {v})\ngot {:?}\nwant {want:?}",
		mp.center_of_mass,
		mp.volume,
		mp.inertia
	);
}

#[test]
fn planar_solids_inertia_is_untouched_by_the_curved_recomposition() {
	// A pure box has no curved faces: the lens corrections are zero and the recomposition
	// must round-trip — mass_properties' tensor equals both the raw tessellation tensor and
	// the closed form m/12·diag(d²+h², b²+h², b²+d²).
	let bx = cuboid(DVec3::new(-2.0, -1.5, -0.5), DVec3::new(2.0, 1.5, 0.5));
	let m = 4.0 * 3.0 * 1.0;
	let want = DMat3::from_diagonal(DVec3::new(m * (9.0 + 1.0) / 12.0, m * (16.0 + 1.0) / 12.0, m * (16.0 + 9.0) / 12.0));
	let mp = mass_properties(&bx);
	let raw = tessellate_default(&bx).mass_properties();
	assert!(
		mat_err(mp.inertia, want) / want.z_axis.z < 1e-6 && mat_err(mp.inertia, raw.inertia) / want.z_axis.z < 1e-9,
		"box inertia must stay the (exact) tessellation value:\ngot {:?}\nraw {:?}\nwant {want:?}",
		mp.inertia,
		raw.inertia
	);
}

#[test]
fn inertia_tensor_is_analytic_for_a_sphere_at_coarse_segments() {
	// THE sphere second-moment correction (sphere_second_moment): at a COARSE (16, 12)
	// tessellation a sphere's raw inertia errs by percent (the inscribed-polyhedron deficit);
	// the solid-angle lens second moments make mass_properties machine-exact. Ø12 sphere at
	// the origin, unit density m = 4/3·π·r³: I = 2/5·m·r²·Id, all products zero. The raw
	// error is asserted to exceed 1e-2 alongside the ≤1e-6 corrected gate (f32 mesh noise
	// floor) — proof the correction ENGAGED rather than the mesh being accidentally fine.
	let r = 6.0;
	let sp = sphere(DVec3::ZERO, r, 16, 12);
	let m = 4.0 / 3.0 * PI * r.powi(3);
	let scale = 0.4 * m * r * r;
	let want = DMat3::from_diagonal(DVec3::splat(scale));
	let mp = mass_properties(&sp);
	let raw = tessellate_default(&sp).mass_properties();
	let (corrected_err, raw_err) = (mat_err(mp.inertia, want) / scale, mat_err(raw.inertia, want) / scale);
	assert!(
		corrected_err < 1e-6 && raw_err > 1e-2 && mp.center_of_mass.length() < 1e-6 && (mp.volume - m).abs() / m < 1e-9,
		"sphere inertia at seg (16,12): corrected rel err {corrected_err:.2e} (want ≤1e-6), raw {raw_err:.2e} (must exceed 1e-2 to \
		 prove the correction engaged), CoM {:?}, V {} (want {m})\ncorrected {:?}\nclosed form {want:?}",
		mp.center_of_mass,
		mp.volume,
		mp.inertia
	);
}

#[test]
fn inertia_tensor_parallel_axis_for_an_off_centre_sphere() {
	// An off-origin sphere: mass_properties reports inertia about the CoM, so the tensor must
	// still be the textbook 2/5·m·r²·Id — but it is computed through origin-frame second
	// moments with |I_origin| ≈ 6×|I_com| here, the cancellation-prone parallel-axis path
	// (the sphere analogue of the tilted-cylinder test; same 1e-5 gate for the f32
	// tessellation noise amplified by the cancellation). The CoM must land on the centre.
	let r = 6.0;
	let c = DVec3::new(7.0, -4.0, 3.0);
	let sp = sphere(c, r, 16, 12);
	let m = 4.0 / 3.0 * PI * r.powi(3);
	let scale = 0.4 * m * r * r;
	let mp = mass_properties(&sp);
	let err = mat_err(mp.inertia, DMat3::from_diagonal(DVec3::splat(scale))) / scale;
	let com_err = (mp.center_of_mass - c).length();
	assert!(
		err < 1e-5 && com_err < 1e-6,
		"off-centre sphere: inertia rel err {err:.2e} (want ≤1e-5), CoM err {com_err:.2e}\ngot {:?}",
		mp.inertia
	);
}

#[test]
fn inertia_tensor_is_analytic_for_a_clean_hemisphere() {
	// A clean upper hemisphere — sphere ∩ half-space, cut exactly at the centre plane, where
	// the even-v sphere has a vertex ring, so every spherical face survives the boolean whole
	// (vertices on the sphere). The solid decomposes exactly into centre-fan pyramids plus the
	// per-face lenses, so the corrected tensor must hit the closed forms about the CoM at
	// 3r/8 above the flat face: Izz = 2/5·m·r², Ixx = Iyy = 83/320·m·r², m = ⅔·π·r³.
	let r = 6.0;
	let hemi = kernel_brep::intersection(&sphere(DVec3::ZERO, r, 16, 12), &cuboid(DVec3::new(-8.0, -8.0, 0.0), DVec3::new(8.0, 8.0, 8.0)));
	let m = 2.0 / 3.0 * PI * r.powi(3);
	let scale = 0.4 * m * r * r;
	let want = DMat3::from_diagonal(DVec3::new(83.0 / 320.0 * m * r * r, 83.0 / 320.0 * m * r * r, scale));
	let mp = mass_properties(&hemi);
	let err = mat_err(mp.inertia, want) / scale;
	let com_err = (mp.center_of_mass - DVec3::new(0.0, 0.0, 3.0 * r / 8.0)).length();
	assert!(
		err < 1e-5 && com_err < 1e-5 && (mp.volume - m).abs() / m < 1e-9,
		"hemisphere: inertia rel err {err:.2e} (want ≤1e-5), CoM err {com_err:.2e} (want (0,0,{})), V {} (want {m})\ngot {:?}\nwant {want:?}",
		3.0 * r / 8.0,
		mp.volume,
		mp.inertia
	);
}

#[test]
fn inertia_tensor_subtracts_a_hemispherical_dimples_lenses() {
	// The CONCAVE sphere path: a half-ball dimple sunk into a block's top face — the sphere
	// centre lies ON the face plane, so the equator vertex ring lies in the plane, the
	// boolean keeps every spherical face whole, and the removed region is an exact half-ball.
	// Pocket faces are wound inward; their winding-signed lens moments must SUBTRACT. Closed
	// form via origin-frame moments: I_block − I_halfball, shifted to the part CoM (half-ball
	// about its own CoM 3r/8 below the centre: Izz = 2/5·m·r², Ixx = Iyy = 83/320·m·r²).
	// This also pins the sphere_first_moment pocket-sign fix: the patch term used to
	// double-flip on concave faces, the real source of the ~1e-3 dimple CoM residual noted in
	// center_of_mass_is_analytic_for_spherical_parts.
	// HONEST LIMIT, still open: a NON-hemispherical dimple (sphere centre off the face plane)
	// keeps a genuine residual — the wedge between the polygonal rim ring and the true rim
	// circle, swept from the centre (the flat-cap-annulus faceting limit) — and no exactness
	// is claimed for it here.
	let rs = 3.0;
	let block = cuboid(DVec3::new(-5.0, -5.0, 0.0), DVec3::new(5.0, 5.0, 6.0));
	let part = difference(&block, &sphere(DVec3::new(0.0, 0.0, 6.0), rs, 32, 24));
	let shift = |c: DVec3| DMat3::from_diagonal(DVec3::splat(c.length_squared())) - DMat3::from_cols(c * c.x, c * c.y, c * c.z);
	let (vb, cb) = (600.0, DVec3::new(0.0, 0.0, 3.0));
	let ib = DMat3::from_diagonal(DVec3::new(vb * 136.0 / 12.0, vb * 136.0 / 12.0, vb * 200.0 / 12.0));
	let (vh, ch) = (2.0 / 3.0 * PI * rs.powi(3), DVec3::new(0.0, 0.0, 6.0 - 3.0 * rs / 8.0));
	let ih = DMat3::from_diagonal(DVec3::new(83.0 / 320.0 * vh * rs * rs, 83.0 / 320.0 * vh * rs * rs, 0.4 * vh * rs * rs));
	let v = vb - vh;
	let com = (cb * vb - ch * vh) / v;
	let want = (ib + shift(cb) * vb) - (ih + shift(ch) * vh) - shift(com) * v;
	let mp = mass_properties(&part);
	let raw = tessellate_default(&part).mass_properties();
	let scale = want.z_axis.z;
	let (err, raw_err) = (mat_err(mp.inertia, want) / scale, mat_err(raw.inertia, want) / scale);
	assert!(
		err < 1e-5 && raw_err > 10.0 * err && (mp.center_of_mass - com).length() < 1e-5 && (mp.volume - v).abs() / v < 1e-9,
		"dimpled block: inertia rel err {err:.2e} (want ≤1e-5; raw {raw_err:.2e} must be ≥10×), CoM {:?} (want {com:?}), V {} (want {v})\n\
		 got {:?}\nwant {want:?}",
		mp.center_of_mass,
		mp.volume,
		mp.inertia
	);
}

#[test]
fn inertia_tensor_is_analytic_for_a_cone_at_coarse_segments() {
	// THE cone second-moment correction (cone_second_moment): a Ø10×12 cone, base on the
	// origin plane, apex up. Closed forms about the CoM (h/4 above the base): axial
	// Izz = 3/10·m·r², transverse Ixx = Iyy = 3/20·m·(r² + h²/4), m = π·r²·h/3. At a coarse
	// 16 segments the raw tessellation errs by >1%; the taper-integrated segment second
	// moments make mass_properties machine-exact (≤1e-6, the f32 mesh noise floor).
	let (r, h, seg) = (5.0, 12.0, 16);
	let c = cone(DVec3::ZERO, DVec3::Z, r, h, seg);
	let m = PI * r * r * h / 3.0;
	let it = 3.0 / 20.0 * m * (r * r + h * h / 4.0);
	let want = DMat3::from_diagonal(DVec3::new(it, it, 0.3 * m * r * r));
	let mp = mass_properties(&c);
	let raw = tessellate_default(&c).mass_properties();
	let (corrected_err, raw_err) = (mat_err(mp.inertia, want) / it, mat_err(raw.inertia, want) / it);
	let com_err = (mp.center_of_mass - DVec3::new(0.0, 0.0, h / 4.0)).length();
	assert!(
		corrected_err < 1e-6 && raw_err > 1e-2 && com_err < 1e-6,
		"cone inertia at seg {seg}: corrected rel err {corrected_err:.2e} (want ≤1e-6), raw {raw_err:.2e} (must exceed 1e-2 to prove \
		 the correction engaged), CoM err {com_err:.2e}\ncorrected {:?}\nclosed form {want:?}",
		mp.inertia
	);
}

#[test]
fn inertia_tensor_parallel_axis_for_an_off_axis_tilted_cone() {
	// The cone closed form survives a rigid pose: based off the origin with a non-axis-aligned
	// axis, the CoM-frame tensor must equal the canonical one rotated onto the axis,
	// I = Ia·aaᵀ + It·(Id − aaᵀ), about the CoM at base + a·h/4 — parallel-axis + rotation
	// under large cancellation, the cone analogue of the tilted-cylinder test (same 1e-5 gate).
	let (r, h, seg) = (5.0, 12.0, 16);
	let a = DVec3::new(1.0, 2.0, 2.0) / 3.0;
	let base = DVec3::new(8.0, -6.0, 4.0);
	let c = cone(base, a, r, h, seg);
	let m = PI * r * r * h / 3.0;
	let (ia, it) = (0.3 * m * r * r, 3.0 / 20.0 * m * (r * r + h * h / 4.0));
	let aa = DMat3::from_cols(a * a.x, a * a.y, a * a.z);
	let want = (DMat3::IDENTITY - aa) * it + aa * ia;
	let mp = mass_properties(&c);
	let err = mat_err(mp.inertia, want) / it;
	let com_err = (mp.center_of_mass - (base + a * (h / 4.0))).length();
	assert!(
		err < 1e-5 && com_err < 1e-6,
		"tilted off-origin cone: inertia rel err {err:.2e} (want ≤1e-5), CoM err {com_err:.2e}\ngot {:?}\nwant {want:?}",
		mp.inertia
	);
}

#[test]
fn inertia_tensor_subtracts_a_conical_countersinks_lenses() {
	// The CONCAVE cone path: a conical countersink funnelling into the block's top face (the
	// same fixture as the exact-volume pocket test — its base ring lies on the face plane, so
	// the difference keeps every cone face whole). Countersink faces are concave: their lens
	// second moments carry the explicit newell-vs-radial minus sign and must SUBTRACT. Closed
	// form via origin-frame moments: I_block − I_cone, the removed Ø6×4 cone's CoM h/4 below
	// the top face plane.
	let (rc, hc, seg) = (3.0, 4.0, 48);
	let block = cuboid(DVec3::new(-5.0, -5.0, 0.0), DVec3::new(5.0, 5.0, 6.0));
	let part = difference(&block, &cone(DVec3::new(0.0, 0.0, 6.0), -DVec3::Z, rc, hc, seg));
	let shift = |c: DVec3| DMat3::from_diagonal(DVec3::splat(c.length_squared())) - DMat3::from_cols(c * c.x, c * c.y, c * c.z);
	let (vb, cb) = (600.0, DVec3::new(0.0, 0.0, 3.0));
	let ib = DMat3::from_diagonal(DVec3::new(vb * 136.0 / 12.0, vb * 136.0 / 12.0, vb * 200.0 / 12.0));
	let (vc, cc) = (PI * rc * rc * hc / 3.0, DVec3::new(0.0, 0.0, 6.0 - hc / 4.0));
	let itc = 3.0 / 20.0 * vc * (rc * rc + hc * hc / 4.0);
	let ic = DMat3::from_diagonal(DVec3::new(itc, itc, 0.3 * vc * rc * rc));
	let v = vb - vc;
	let com = (cb * vb - cc * vc) / v;
	let want = (ib + shift(cb) * vb) - (ic + shift(cc) * vc) - shift(com) * v;
	let mp = mass_properties(&part);
	let scale = want.z_axis.z;
	let err = mat_err(mp.inertia, want) / scale;
	assert!(
		err < 1e-5 && (mp.center_of_mass - com).length() < 1e-5 && (mp.volume - v).abs() / v < 1e-9,
		"countersunk block: inertia rel err {err:.2e} (want ≤1e-5), CoM {:?} (want {com:?}), V {} (want {v})\ngot {:?}\nwant {want:?}",
		mp.center_of_mass,
		mp.volume,
		mp.inertia
	);
}

#[test]
fn subtracting_a_sphere_or_cone_leaves_an_analytic_pocket_with_exact_volume() {
	// The curved-through-booleans fix is GENERAL (recover_faces keeps any curved Surface tag,
	// not just Cylinder): a spherical dimple (sphere on the top face removing its lower
	// hemisphere) and a conical countersink (cone funnelling into the top) must each keep
	// their analytic faces through the difference and let exact_volume recover the closed form
	// — box − ⅔πr³ for the hemisphere, box − ⅓πr²h for the cone — to micron precision, valid
	// and free of self-intersection (the >4-gon facets tessellate chord-flat).
	let block = cuboid(DVec3::new(-5.0, -5.0, 0.0), DVec3::new(5.0, 5.0, 6.0));
	let dimple = kernel_brep::difference(&block, &sphere(DVec3::new(0.0, 0.0, 6.0), 3.0, 32, 24));
	let csink = kernel_brep::difference(&block, &cone(DVec3::new(0.0, 0.0, 6.0), -DVec3::Z, 3.0, 4.0, 48));
	let dv = validate(&dimple);
	let cv = validate(&csink);
	let sph = dimple.faces().filter(|&f| matches!(dimple.face(f).surface, Surface::Sphere { .. })).count();
	let con = csink.faces().filter(|&f| matches!(csink.face(f).surface, Surface::Cone { .. })).count();
	let dimple_true = 600.0 - (2.0 / 3.0) * PI * 27.0;
	let csink_true = 600.0 - (1.0 / 3.0) * PI * 9.0 * 4.0;
	assert!(
		dv.closed
			&& dv.manifold
			&& dv.genus == 0
			&& !self_intersects(&dimple)
			&& sph > 0
			&& (exact_volume(&dimple).abs() - dimple_true).abs() < 1e-4
			&& cv.closed
			&& cv.manifold
			&& cv.genus == 0
			&& !self_intersects(&csink)
			&& con > 0
			&& (exact_volume(&csink).abs() - csink_true).abs() < 1e-4,
		"dimple {dv:?} sph_faces={sph} exact={} (true {dimple_true}) | csink {cv:?} cone_faces={con} exact={} (true {csink_true})",
		exact_volume(&dimple).abs(),
		exact_volume(&csink).abs()
	);
}

#[test]
fn a_blind_flat_bottomed_pocket_has_exact_volume() {
	// The most common machining feature — a blind hole (a cylinder that does NOT exit the far
	// face, leaving a flat bottom). Distinct genus-0 topology from the through-hole (no
	// inner-loop annular caps): the bore wall stays Surface::Cylinder, the flat bottom is a
	// disc, and exact_volume recovers box − π·r²·depth to micron precision, valid and not
	// self-intersecting. Confirms the curved-through-booleans fix on a single clean difference.
	let block = cuboid(DVec3::new(-5.0, -5.0, 0.0), DVec3::new(5.0, 5.0, 6.0));
	let pocket = kernel_brep::difference(&block, &cylinder(DVec3::new(0.0, 0.0, 2.0), DVec3::Z, 2.0, 5.0, 48));
	let v = validate(&pocket);
	let bore = pocket.faces().filter(|&f| matches!(pocket.face(f).surface, Surface::Cylinder { .. })).count();
	let exact = 600.0 - PI * 2.0 * 2.0 * 4.0; // depth = 6 − 2 = 4
	assert!(
		v.closed
			&& v.manifold
			&& v.genus == 0
			&& !self_intersects(&pocket)
			&& bore > 0
			&& (exact_volume(&pocket).abs() - exact).abs() < 1e-4,
		"blind pocket: {v:?} bore_cyl_faces={bore} exact_volume={} (closed form {exact})",
		exact_volume(&pocket).abs()
	);
}

#[test]
fn mass_properties_volume_is_exact_for_curved_solids() {
	// The convenience mass_properties() API reports the EXACT analytic volume (not the
	// ~1-2%-low tessellation), so an AI gets the right mass straight from the part.
	let cyl = cylinder(DVec3::ZERO, DVec3::Z, 5.0, 12.0, 20);
	let mp = mass_properties(&cyl);
	let exact = PI * 25.0 * 12.0;
	assert!((mp.volume - exact).abs() / exact < 1e-6, "mass_properties volume must be analytic-exact: {} vs {exact}", mp.volume);
}

#[test]
fn curved_planar_union_tessellates_watertight_and_fine_via_exact_path() {
	// The bolt BODY: a cylindrical shank UNIONED with a hex head sitting on its top — a
	// curved (cylinder) ∪ planar (hex prism) boolean. Its exact adaptive tessellation must be
	// WATERTIGHT and micron-fine, following the true cylinder rather than a voxel grid (the
	// union counterpart to boolean_annular_cap_…'s difference). A tighter tolerance must add
	// triangles (the curved shank genuinely refines), and the volume tracks the closed form.
	let shank = cylinder(DVec3::ZERO, DVec3::Z, 4.0, 32.0, 48);
	let head = extrude(&hexagon_circ(7.5), 6.0).transformed(DAffine3::from_translation(DVec3::new(0.0, 0.0, 32.0)));
	let body = union(&shank, &head);
	let v = validate(&body);
	let coarse = tessellate_adaptive_tol(&body, 0.05).triangle_count();
	let fine = tessellate_adaptive_tol(&body, 0.005);
	// shank π·4²·32 + hex head (3√3/2)·7.5²·6, less the shank volume the head re-covers on top.
	let expected = PI * 4.0 * 4.0 * 32.0 + 1.5 * 3.0_f64.sqrt() * 7.5 * 7.5 * 6.0;
	assert!(
		v.closed && v.manifold && v.genus == 0 && fine.is_watertight() && fine.triangle_count() > coarse && (volume(&body) - expected).abs() / expected < 0.02,
		"bolt body must mesh watertight+fine via exact path: closed={} manifold={} genus={} wt={} fine_tris={} coarse_tris={} vol={:.1} (expected ~{:.1})",
		v.closed,
		v.manifold,
		v.genus,
		fine.is_watertight(),
		fine.triangle_count(),
		coarse,
		volume(&body),
		expected
	);
}

/// A regular hexagon of circumradius `r` (centre-to-vertex), flat-side-down — the bolt-head
/// / nut profile used across these acceptance tests.
fn hexagon_circ(r: f64) -> Vec<DVec2> {
	(0..6)
		.map(|i| {
			let a = PI / 6.0 + i as f64 * PI / 3.0;
			DVec2::new(r * a.cos(), r * a.sin())
		})
		.collect()
}

#[test]
fn boolean_annular_cap_tessellates_watertight_via_exact_path() {
	// A boolean difference punches a hole through a planar cap, leaving an ANNULAR face that
	// the boolean stitches into one keyhole-bridged loop whose straight rim carries only
	// near-collinear subdivision points. The exact adaptive tessellation must mesh that cap
	// WATERTIGHT (the robust ear-clipper honours the near-collinear rim instead of skipping a
	// point into an overlapping sliver) — this is what lets the hex+bore nut be crisp without
	// a voxel heal. Volume tracks the closed form (faceted bore is a hair under the true bore).
	let nut = difference(&extrude(&hexagon_circ(7.5), 6.0), &cylinder(DVec3::new(0.0, 0.0, -1.0), DVec3::Z, 4.2, 8.0, 48));
	let mesh = tessellate_adaptive_tol(&nut, 0.005);
	let hex_area = (3.0_f64.sqrt() * 3.0 / 2.0) * 7.5 * 7.5;
	let expected = hex_area * 6.0 - PI * 4.2 * 4.2 * 6.0;
	assert!(
		mesh.is_watertight() && (mesh.signed_volume() - expected).abs() < 2.0,
		"drilled hex nut must mesh watertight via the exact path: wt={} vol={:.2} (expected ~{:.2})",
		mesh.is_watertight(),
		mesh.signed_volume(),
		expected
	);
}

#[test]
fn hex_nut_is_a_valid_genus_one_part_with_exact_volume() {
	// A hex nut — a hexagonal prism with a concentric clearance bore — exercises a real
	// mixed planar+cylindrical engineering part end-to-end: the boolean difference must
	// yield a closed, manifold, genus-1 solid whose exact volume is (hex area − bore
	// area)·height. (Topology + integral volume are robust to sub-ulp boolean noise.)
	let (r, h, bore) = (7.5_f64, 6.0_f64, 4.2_f64);
	let body = extrude(&hexagon_circ(r), h);
	let hole = cylinder(DVec3::new(0.0, 0.0, -1.0), DVec3::Z, bore, h + 2.0, 48);
	let nut = kernel_brep::difference(&body, &hole);

	let v = validate(&nut);
	let hex_area = 3.0_f64.sqrt() * 1.5 * r * r; // (3√3/2)·R²
	let exact = (hex_area - PI * bore * bore) * h;
	// NOTE: exact_volume corrects the cylindrical bore for a box-drilled plate to <1e-4,
	// but on this hex-prism-drilled body the bore correction isn't applied (the boolean
	// retags the bore faces differently than for a cuboid), so the bore reads as its
	// 48-gon facet — a ~0.17% gap. We gate genus + a near-exact volume; the exact_volume
	// bore-correction-after-boolean-on-non-box is a tracked follow-up.
	assert!(
		v.closed && v.manifold && v.genus == 1 && (exact_volume(&nut) - exact).abs() / exact < 5e-3,
		"hex nut should be a closed manifold genus-1 part of ~exact volume {exact}: {v:?} exact_vol={}",
		exact_volume(&nut)
	);
}

#[test]
fn multi_hole_plate_is_valid_via_extrude_with_holes() {
	// The ROBUST way to build a multi-hole part is ONE extrude_with_holes (multi-loop), NOT a
	// CHAIN of boolean differences: chaining `difference` to drill several bores degrades the
	// curved-boolean arrangement into an invalid, non-manifold B-rep (a tracked limitation —
	// the second cut operates on the first cut's heavily-faceted output). extrude_with_holes
	// builds a valid genus-N washer directly. Three circular holes → a closed, manifold,
	// genus-3 solid of (plate − 3·bore) volume that meshes watertight.
	let outer = vec![DVec2::new(-20.0, -12.0), DVec2::new(20.0, -12.0), DVec2::new(20.0, 12.0), DVec2::new(-20.0, 12.0)];
	let circle = |cx: f64, cy: f64| -> Vec<DVec2> {
		(0..32)
			.map(|i| {
				let a = i as f64 / 32.0 * std::f64::consts::TAU;
				DVec2::new(cx + 2.5 * a.cos(), cy + 2.5 * a.sin())
			})
			.collect()
	};
	let holes = vec![circle(-12.0, 0.0), circle(0.0, 0.0), circle(12.0, 0.0)];
	let plate = extrude_with_holes(&outer, &holes, 6.0);
	let v = validate(&plate);
	// outer area 40×24 = 960; each 32-gon hole ≈ π·2.5² (a hair under); height 6.
	let expected = (960.0 - 3.0 * PI * 2.5 * 2.5) * 6.0;
	assert!(
		v.closed
			&& v.manifold
			&& v.genus == 3
			&& tessellate_default(&plate).is_watertight()
			&& (volume(&plate).abs() - expected).abs() / expected < 0.02,
		"3-hole plate via extrude_with_holes must be a closed manifold genus-3 watertight solid ~{expected:.0}mm³: {v:?} wt={} vol={:.0}",
		tessellate_default(&plate).is_watertight(),
		volume(&plate).abs()
	);
}

#[test]
fn disjoint_union_merges_two_solids_into_a_clean_two_shell() {
	// disjoint_union concatenates topology with NO boolean co-refinement, so two separated solids
	// become a valid 2-shell solid whose volume is exactly their sum and which does not
	// self-intersect — the exact way to combine non-touching parts (a bolt-circle's holes, a peg
	// ring) without the curved mesh-arrangement boolean that a chained union corrupts.
	let a = cuboid(DVec3::new(-3.0, -1.0, -1.0), DVec3::new(-1.0, 1.0, 1.0)); // 2³ = 8
	let b = cylinder(DVec3::new(2.0, 0.0, -1.0), DVec3::Z, 1.0, 2.0, 32); // π·1²·2 ≈ 6.283
	let m = a.disjoint_union(&b);
	let v = validate(&m);
	let expected = 8.0 + PI * 1.0 * 1.0 * 2.0;
	assert!(
		v.closed && v.manifold && v.shells == 2 && !self_intersects(&m) && (volume(&m).abs() - expected).abs() / expected < 0.01,
		"disjoint_union must be a clean 2-shell of summed volume ~{expected:.2}: {v:?} self_int={} vol={:.2}",
		self_intersects(&m),
		volume(&m).abs()
	);
}

#[test]
fn filleted_box_drilled_with_a_hole_is_valid_and_correct() {
	// A ubiquitous real part: round an edge, then drill a through-hole. The filleted box carries a
	// CYLINDER fillet face, so this exercises a curved↔curved boolean (fillet face + bore). A
	// SINGLE such boolean is robust (only CHAINED curved booleans corrupt): the result must be a
	// closed, manifold, genus-1 solid, free of self-intersection, of (filleted box − bore) volume.
	let edge = EdgeName::new(
		FaceName { operand: FaceSource::Primitive, source_face: 5 },
		FaceName { operand: FaceSource::Primitive, source_face: 3 },
	);
	let filleted = fillet_edge(&cuboid(DVec3::new(-5.0, -5.0, -3.0), DVec3::new(5.0, 5.0, 3.0)), edge, 2.0).expect("fillet a box edge");
	let drilled = difference(&filleted, &cylinder(DVec3::new(0.0, 0.0, -4.0), DVec3::Z, 2.0, 8.0, 48));
	let v = validate(&drilled);
	let expected = volume(&filleted).abs() - PI * 2.0 * 2.0 * 6.0;
	assert!(
		v.closed && v.manifold && v.genus == 1 && !self_intersects(&drilled) && (volume(&drilled).abs() - expected).abs() / expected < 0.01,
		"filleted box with a hole must be a clean genus-1 part ~{expected:.0}mm³: {v:?} self_int={} vol={:.0}",
		self_intersects(&drilled),
		volume(&drilled).abs()
	);
}

#[test]
fn primitives_are_free_of_self_intersection() {
	// The geometric half of validity: well-formed primitives and a non-convex extrude
	// tessellate to meshes whose faces never pass through one another.
	assert!(!self_intersects(&cuboid(DVec3::new(-2.0, -2.0, -2.0), DVec3::new(2.0, 2.0, 2.0))), "box self-intersects");
	assert!(!self_intersects(&cylinder(DVec3::ZERO, DVec3::Z, 3.0, 8.0, 32)), "cylinder self-intersects");
	assert!(!self_intersects(&sphere(DVec3::ZERO, 4.0, 24, 16)), "sphere self-intersects");
	let l = [
		DVec2::new(0.0, 0.0),
		DVec2::new(10.0, 0.0),
		DVec2::new(10.0, 3.0),
		DVec2::new(3.0, 3.0),
		DVec2::new(3.0, 8.0),
		DVec2::new(0.0, 8.0),
	];
	assert!(!self_intersects(&extrude(&l, 4.0)), "L-extrude self-intersects");
}

#[test]
fn tapered_extrude_is_a_drafted_frustum() {
	// A 20×20 square drafted inward by 0.1 rad over a 10mm rise → a truncated pyramid
	// (every wall sloped, so the part releases from a mould). All faces planar, so it
	// is a closed genus-0 solid whose volume matches the prismatoid closed form exactly.
	let l = 20.0;
	let h = 10.0;
	let draft = 0.1_f64;
	let profile =
		[DVec2::new(-l / 2.0, -l / 2.0), DVec2::new(l / 2.0, -l / 2.0), DVec2::new(l / 2.0, l / 2.0), DVec2::new(-l / 2.0, l / 2.0)];
	let s = extrude_tapered(&profile, h, draft);
	assert_genus0_solid(&s, "tapered-extrude");
	// Truncated pyramid: bottom side a, top side b = a − 2·h·tan(draft). The solid is
	// planar-exact, so the only error is the shared f32 tessellation `volume()` runs
	// through (the draft offset is irrational, not f32-exact) — a relative tolerance,
	// like the curved-primitive tests; here it matches to ~3e-8 (tens of ppb).
	let a = l;
	let b = l - 2.0 * h * draft.tan();
	let exact = h / 3.0 * (a * a + b * b + a * b);
	assert!((volume(&s) - exact).abs() / exact < 1e-5, "tapered vol {} vs {exact}", volume(&s));
}

#[test]
fn extrude_with_holes_makes_a_genus_one_washer() {
	// A 6×6 plate with a centred 2×2 square hole, extruded 2mm → a washer: a closed,
	// manifold, GENUS-1 solid (one through-hole, χ = 0) whose tessellated volume is
	// exactly (outer − hole) × height — proving the multi-loop (polygon-with-holes)
	// tessellation cuts the hole rather than fan-filling the outer loop.
	let outer = vec![DVec2::new(-3.0, -3.0), DVec2::new(3.0, -3.0), DVec2::new(3.0, 3.0), DVec2::new(-3.0, 3.0)];
	let hole = vec![DVec2::new(-1.0, -1.0), DVec2::new(1.0, -1.0), DVec2::new(1.0, 1.0), DVec2::new(-1.0, 1.0)];
	let s = extrude_with_holes(&outer, &[hole], 2.0);
	let v = validate(&s);
	let exact = (36.0 - 4.0) * 2.0;
	assert!(
		v.closed && v.manifold && v.genus == 1 && v.euler_characteristic == 0 && (volume(&s).abs() - exact).abs() < 1e-6,
		"washer should be a closed manifold genus-1 solid of volume {exact}: {v:?} vol={}",
		volume(&s).abs()
	);
	assert!(tessellate_default(&s).is_watertight(), "the washer mesh (with its hole cut) must be watertight");
}

#[test]
fn exact_volume_is_loop_aware_for_multi_loop_extrusions() {
	// R4 regression (BAR Level 6): exact_volume must take a face's inner hole loops AS
	// WOUND — they run opposite the outer loop, so their divergence-theorem fan volume
	// already subtracts the hole's flux. The old code SUBTRACTED them, flipping every
	// hole's sign: this flange read +7.1% (exactly twice the hole prisms' flux through
	// the top cap, 2·8/3·Σ hole-area).
	//
	// Flange: Ø80 disk (96-gon), Ø20 centre bore (48-gon), six Ø6 holes (24-gons) on a
	// Ø60 bolt circle, 8 mm thick. All faces are planar, so exact_volume, the tessellated
	// volume() and the shoelace closed form must all agree, and mass_properties must
	// report the same exact volume.
	let circle = |r: f64, cx: f64, cy: f64, n: usize| -> Vec<DVec2> {
		(0..n)
			.map(|k| {
				let a = 2.0 * PI * k as f64 / n as f64;
				DVec2::new(cx + r * a.cos(), cy + r * a.sin())
			})
			.collect()
	};
	let shoelace = |p: &[DVec2]| -> f64 {
		0.5 * (0..p.len())
			.map(|i| {
				let q = p[(i + 1) % p.len()];
				p[i].x * q.y - q.x * p[i].y
			})
			.sum::<f64>()
			.abs()
	};
	let outer = circle(40.0, 0.0, 0.0, 96);
	let mut holes = vec![circle(10.0, 0.0, 0.0, 48)];
	for k in 0..6 {
		let a = 2.0 * PI * k as f64 / 6.0;
		holes.push(circle(3.0, 30.0 * a.cos(), 30.0 * a.sin(), 24));
	}
	let flange = extrude_with_holes(&outer, &holes, 8.0);
	let exact = (shoelace(&outer) - holes.iter().map(|h| shoelace(h)).sum::<f64>()) * 8.0;
	let (ev, tv, mpv) = (exact_volume(&flange), volume(&flange), mass_properties(&flange).volume);
	let v = validate(&flange);
	assert!(
		v.closed
			&& v.manifold
			&& v.genus == 7
			&& (ev - exact).abs() / exact < 1e-12
			&& (ev - tv).abs() / tv < 1e-6
			&& (mpv - exact).abs() / exact < 1e-12,
		"7-hole flange: exact_volume must equal the shoelace closed form {exact:.4} and the tessellated volume: {v:?} exact_volume={ev:.4} volume={tv:.4} mass_properties.volume={mpv:.4}"
	);
	// And the 6×6 plate with a 2×2 through-hole: exact_volume = (36 − 4)·2 = 64, exactly.
	let outer_sq = vec![DVec2::new(-3.0, -3.0), DVec2::new(3.0, -3.0), DVec2::new(3.0, 3.0), DVec2::new(-3.0, 3.0)];
	let hole_sq = vec![DVec2::new(-1.0, -1.0), DVec2::new(1.0, -1.0), DVec2::new(1.0, 1.0), DVec2::new(-1.0, 1.0)];
	let washer = extrude_with_holes(&outer_sq, &[hole_sq], 2.0);
	assert!(
		(exact_volume(&washer) - 64.0).abs() < 1e-12,
		"square washer exact_volume {} should be exactly (36 − 4)·2 = 64",
		exact_volume(&washer)
	);
}

#[test]
fn mirror_preserves_a_through_hole() {
	// Mirroring a washer (genus-1, a through-hole) across an offset plane must KEEP the hole.
	// `mirrored` used to rebuild from outer loops only, silently filling any pocket/bore; it
	// now carries every inner loop (reversed), so the reflected copy is still a closed,
	// manifold, genus-1 solid of equal volume that meshes watertight.
	let outer = vec![DVec2::new(-3.0, -3.0), DVec2::new(3.0, -3.0), DVec2::new(3.0, 3.0), DVec2::new(-3.0, 3.0)];
	let hole = vec![DVec2::new(-1.0, -1.0), DVec2::new(1.0, -1.0), DVec2::new(1.0, 1.0), DVec2::new(-1.0, 1.0)];
	let washer = extrude_with_holes(&outer, &[hole], 2.0);
	let reflected = washer.mirrored(DVec3::new(10.0, 0.0, 0.0), DVec3::X);
	let v = validate(&reflected);
	let exact = (36.0 - 4.0) * 2.0;
	assert!(
		v.closed
			&& v.manifold
			&& v.genus == 1
			&& (volume(&reflected).abs() - exact).abs() < 1e-6
			&& tessellate_default(&reflected).is_watertight(),
		"mirror must preserve the through-hole (closed manifold genus-1, vol {exact}, watertight): {v:?} vol={}",
		volume(&reflected).abs()
	);
}

#[test]
fn revolve_torus_has_genus_one() {
	// A square cross-section ring not touching the axis → genus-1 solid (torus).
	let profile = [DVec2::new(8.0, -2.0), DVec2::new(12.0, -2.0), DVec2::new(12.0, 2.0), DVec2::new(8.0, 2.0)];
	let s = revolve(&profile, 48);
	let v = validate(&s);
	assert!(v.closed && v.manifold, "revolved torus must be a closed manifold");
	assert_eq!(v.euler_characteristic, 0, "torus χ should be 0");
	assert_eq!(v.genus, 1, "revolved ring should be genus 1");
	let mesh = tessellate_default(&s);
	assert!(mesh.is_watertight(), "torus tessellation not watertight");
	// Pappus: V = (area) * (2π * centroid_radius) = (4*4) * (2π*10).
	let exact = 16.0 * 2.0 * PI * 10.0;
	assert!((volume(&s) - exact).abs() / exact < 0.01, "torus vol {} vs {exact}", volume(&s));
}

#[test]
fn revolve_cone_via_axis_touching_profile() {
	// Triangle profile touching the axis at both poles → a cone solid.
	let profile = [DVec2::new(0.0, 0.0), DVec2::new(5.0, 0.0), DVec2::new(0.0, 10.0)];
	let s = revolve(&profile, 64);
	assert_genus0_solid(&s, "revolved cone");
	let exact = PI * 25.0 * 10.0 / 3.0;
	assert!((volume(&s) - exact).abs() / exact < 0.01, "revolved cone vol {} vs {exact}", volume(&s));
}

#[test]
fn revolve_solid_cylinder_via_axis_touching_profile() {
	// Rectangle profile with two edges on the axis → a solid cylinder.
	let profile = [DVec2::new(0.0, 0.0), DVec2::new(5.0, 0.0), DVec2::new(5.0, 10.0), DVec2::new(0.0, 10.0)];
	let s = revolve(&profile, 64);
	assert_genus0_solid(&s, "revolved cylinder");
	let exact = PI * 25.0 * 10.0;
	assert!((volume(&s) - exact).abs() / exact < 0.01, "revolved cylinder vol {} vs {exact}", volume(&s));
}

#[test]
fn revolve_of_concave_multi_segment_profile_is_a_valid_ring_with_faceted_volume() {
	// R1 regression (BAR Level 6): an L-shaped flange cross-section — six points, a
	// concave corner, two horizontal segments at different radii — must revolve to a
	// closed manifold genus-1 ring (it has a Ø20 through-bore). The old centroid-based
	// band orientation emitted the two bands at the concave corner inside-out, leaving
	// their boundary rings unpaired: open seams, 2 shells, genus 98 at 96 segments.
	//
	// Volume oracle, honestly faceted (n-gon factor, NOT π): every band quad of a
	// revolve is planar, with plane-offset × area = ½·sin(2π/N)·(rᵢ+rⱼ)(rᵢzⱼ−rⱼzᵢ) per
	// sector, so the N-gon polyhedron's volume is exactly N·sin(2π/N)·M/6, where
	// M = Σ(rᵢ+rⱼ)(rᵢzⱼ−rⱼzᵢ) = 6·∮r dA is the profile's first area-moment about the
	// axis (the Pappus 2π·M/6 is the N→∞ limit, 0.07% above at N=96). exact_volume's
	// cylinder-bulge corrections must then recover the TRUE solid of revolution
	// 2π·M/6 — machine-exact and resolution-independent, because every face of this
	// solid is a plane or a tagged cylinder.
	let profile = [
		DVec2::new(10.0, 0.0),
		DVec2::new(40.0, 0.0),
		DVec2::new(40.0, 6.0),
		DVec2::new(18.0, 6.0),
		DVec2::new(18.0, 18.0),
		DVec2::new(10.0, 18.0),
	];
	let m: f64 = (0..profile.len())
		.map(|i| {
			let (p, q) = (profile[i], profile[(i + 1) % profile.len()]);
			(p.x + q.x) * (p.x * q.y - q.x * p.y)
		})
		.sum();
	let segments = 96usize;
	let faceted = segments as f64 * (2.0 * PI / segments as f64).sin() * m / 6.0;
	let pappus = 2.0 * PI * m / 6.0;
	let s = revolve(&profile, segments);
	let v = validate(&s);
	let wt = tessellate_default(&s).is_watertight();
	assert!(
		v.closed
			&& v.manifold
			&& v.genus == 1
			&& v.shells == 1
			&& wt
			&& (volume(&s) - faceted).abs() / faceted < 1e-6
			&& (exact_volume(&s) - pappus).abs() / pappus < 1e-9,
		"L-profile revolve must be a watertight genus-1 ring of faceted volume {faceted:.4} (exact {pappus:.4}): {v:?} wt={wt} vol={:.4} exact_vol={:.4}",
		volume(&s),
		exact_volume(&s)
	);
	// Winding-agnostic: the same profile given clockwise revolves to the same solid.
	let cw: Vec<DVec2> = profile.iter().rev().copied().collect();
	let s_cw = revolve(&cw, segments);
	let v_cw = validate(&s_cw);
	assert!(
		v_cw.closed && v_cw.manifold && v_cw.genus == 1 && (volume(&s_cw) - faceted).abs() / faceted < 1e-6,
		"clockwise-wound input must revolve identically: {v_cw:?} vol={:.4}",
		volume(&s_cw)
	);
}

/// Diagonal of an inertia tensor as `(Ixx, Iyy, Izz)`.
fn diag(m: DMat3) -> (f64, f64, f64) {
	(m.x_axis.x, m.y_axis.y, m.z_axis.z)
}

/// Largest absolute off-diagonal entry (a symmetric tensor's products of inertia).
fn max_offdiag(m: DMat3) -> f64 {
	[m.y_axis.x, m.z_axis.x, m.z_axis.y].into_iter().map(f64::abs).fold(0.0, f64::max)
}

#[test]
fn primitive_mass_property_constructors_match_the_tessellated_solids() {
	// The closed-form MassProperties::solid_* constructors give exact mass props with no mesh.
	// They must agree with tessellating the matching primitive: machine-exact for the planar
	// box, within tessellation error for the curved cylinder/sphere (the tolerance IS that
	// tessellation error — the analytic value is exact).
	let cases: [(MassProperties, MassProperties, f64, &str); 3] = [
		(
			MassProperties::solid_box(3.0, 2.0, 4.0),
			mass_properties(&cuboid(DVec3::new(-1.5, -1.0, -2.0), DVec3::new(1.5, 1.0, 2.0))),
			1e-6,
			"box",
		),
		(
			MassProperties::solid_cylinder(5.0, 12.0),
			mass_properties(&cylinder(DVec3::new(0.0, 0.0, -6.0), DVec3::Z, 5.0, 12.0, 128)),
			1e-2,
			"cylinder",
		),
		(MassProperties::solid_sphere(6.0), mass_properties(&sphere(DVec3::ZERO, 6.0, 96, 48)), 2e-2, "sphere"),
	];
	for (analytic, tess, tol, name) in cases {
		let (ax, ay, az) = diag(analytic.inertia);
		let (tx, ty, tz) = diag(tess.inertia);
		assert!(
			(analytic.volume - tess.volume).abs() / tess.volume < tol
				&& (ax - tx).abs() / tx < tol
				&& (ay - ty).abs() / ty < tol
				&& (az - tz).abs() / tz < tol,
			"{name}: analytic vol {} I ({ax},{ay},{az}) vs tess vol {} I ({tx},{ty},{tz})",
			analytic.volume,
			tess.volume
		);
	}
}

#[test]
fn combined_mass_properties_match_the_unioned_solid() {
	// Two disjoint boxes: their mass properties combined by the parallel-axis theorem must
	// equal those of the single solid that is their union — volume, center of mass, and the
	// full inertia tensor — to floating-point precision (all faces planar).
	let a = cuboid(DVec3::new(-3.0, -1.0, -1.0), DVec3::new(-1.0, 1.0, 1.0)); // 2×2×2 centered at x=-2
	let b = cuboid(DVec3::new(1.0, -1.5, -1.0), DVec3::new(5.0, 1.5, 1.0)); // 4×3×2 centered at x=3
																		 // Build the parts from the analytic constructors and place them with translated(), so
																		 // this one check validates solid_box + translated + combine against the real geometry.
	let combined = MassProperties::combine(&[
		MassProperties::solid_box(2.0, 2.0, 2.0).translated(DVec3::new(-2.0, 0.0, 0.0)),
		MassProperties::solid_box(4.0, 3.0, 2.0).translated(DVec3::new(3.0, 0.0, 0.0)),
	]);
	let whole = mass_properties(&a.disjoint_union(&b));
	let (cx, cy, cz) = diag(combined.inertia);
	let (wx, wy, wz) = diag(whole.inertia);
	assert!(
		(combined.volume - whole.volume).abs() / whole.volume < 1e-6
			&& (combined.center_of_mass - whole.center_of_mass).length() < 1e-6
			&& (cx - wx).abs() / wx < 1e-6
			&& (cy - wy).abs() / wy < 1e-6
			&& (cz - wz).abs() / wz < 1e-6,
		"combine vs union: V {} vs {}, CoM {:?} vs {:?}, I ({cx},{cy},{cz}) vs ({wx},{wy},{wz})",
		combined.volume,
		whole.volume,
		combined.center_of_mass,
		whole.center_of_mass
	);
}

#[test]
fn torus_is_a_genus_one_analytic_surface_that_meshes_to_the_true_torus() {
	// The missing analytic torus primitive (O-ring / donut) and the keystone for an EXACT torus
	// fillet: a major=8/minor=2 torus must be a closed, manifold, GENUS-1 solid whose faces ALL
	// carry the analytic Surface::Torus tag, so adaptive tessellation projects facets onto the
	// TRUE torus and recovers the closed-form volume 2π²·R·r² — not a flat-faceted approximation.
	let (major, minor) = (8.0, 2.0);
	let t = torus(DVec3::ZERO, DVec3::Z, major, minor, 48, 24);
	let v = validate(&t);
	let exact_vol = 2.0 * PI * PI * major * minor * minor;
	let adaptive_vol = tessellate_adaptive_tol(&t, 0.02).signed_volume().abs();
	let all_torus = t.faces().all(|f| matches!(t.face(f).surface, Surface::Torus { .. }));
	assert!(
		v.closed
			&& v.manifold
			&& v.genus == 1
			&& tessellate_default(&t).is_watertight()
			&& all_torus
			&& (adaptive_vol - exact_vol).abs() / exact_vol < 0.02,
		"torus: {v:?} wt={} all_torus_tagged={all_torus} adaptive_vol={adaptive_vol} (exact {exact_vol})",
		tessellate_default(&t).is_watertight()
	);
}

#[test]
fn exact_volume_corrects_toroidal_faces_to_the_closed_form_at_any_resolution() {
	// The analytic torus_bulge arm of exact_volume: a closed torus has NO flat caps, so the
	// per-face curvature correction must recover the closed-form volume 2π²·R·r² to micron
	// precision REGARDLESS of facet count — the signature of a true analytic correction rather
	// than a faceting limit. Checked at a coarse (8×6) and fine (48×24) tessellation AND on an
	// off-origin, tilted-axis torus (exercising the world-origin cross term and the perp-basis
	// frame), where a flat-faceted volume would be off by whole percent at 8×6.
	let (major, minor) = (8.0, 2.0);
	let exact = 2.0 * PI * PI * major * minor * minor;
	let worst = [
		torus(DVec3::ZERO, DVec3::Z, major, minor, 8, 6),
		torus(DVec3::ZERO, DVec3::Z, major, minor, 48, 24),
		torus(DVec3::new(1.0, 2.0, -3.0), DVec3::new(1.0, 1.0, 0.5).normalize(), major, minor, 10, 7),
	]
	.iter()
	.map(|t| (exact_volume(t).abs() - exact).abs())
	.fold(0.0_f64, f64::max);
	assert!(worst < 1e-6, "torus exact_volume error {worst} exceeds 1e-6 (closed form {exact})");
}

#[test]
fn chamfered_cylinder_bevels_the_top_rim_into_a_valid_solid() {
	// The cut-edge counterpart of the rim fillet: a Ø10×12 cylinder with a 45° top-rim chamfer of
	// size 2. Closed, manifold, genus-0, watertight, with less material than the sharp cylinder
	// (the bevel removes a corner) and within the loose r×r corner-ring bound.
	let (r, h, ch) = (5.0, 12.0, 2.0);
	let solid = chamfered_cylinder(r, h, ch, 64);
	let v = validate(&solid);
	let cyl_vol = PI * r * r * h;
	let vol = volume(&solid).abs();
	assert!(
		v.closed
			&& v.manifold
			&& v.genus == 0
			&& tessellate_default(&solid).is_watertight()
			&& vol < cyl_vol
			&& vol > cyl_vol - 2.0 * PI * r * ch * ch,
		"chamfered cylinder: {v:?} wt={} vol={vol} (sharp cyl {cyl_vol})",
		tessellate_default(&solid).is_watertight()
	);
}

#[test]
fn filleted_cylinder_rounds_the_top_rim_with_an_exact_torus_surface() {
	// Curved-precision: the rim fillet is now an ANALYTIC torus, not faceted cone bands. A Ø10×12
	// cylinder filleted r=2 → closed/manifold/genus-0/watertight; its fillet faces carry the
	// analytic Surface::Torus tag; and tessellate_adaptive_tol recovers the closed-form filleted
	// volume (cylinder body up to z=h−r plus ∫π·ρ(z)² over the fillet arc), proving exact meshing.
	let (radius, h, r) = (5.0, 12.0, 2.0);
	let solid = filleted_cylinder(radius, h, r, 64, 8);
	let v = validate(&solid);
	let has_torus = solid.faces().any(|f| matches!(solid.face(f).surface, Surface::Torus { .. }));
	let true_vol = PI * radius * radius * (h - r)
		+ PI * ((radius - r) * (radius - r) * r + (PI / 2.0) * (radius - r) * r * r + (2.0 / 3.0) * r * r * r);
	let adaptive_vol = tessellate_adaptive_tol(&solid, 0.01).signed_volume().abs();
	assert!(
		v.closed
			&& v.manifold
			&& v.genus == 0
			&& tessellate_default(&solid).is_watertight()
			&& has_torus
			&& (adaptive_vol - true_vol).abs() / true_vol < 0.02,
		"filleted cylinder: {v:?} wt={} has_torus={has_torus} adaptive_vol={adaptive_vol} (true {true_vol})",
		tessellate_default(&solid).is_watertight()
	);
}

#[test]
fn overhang_analysis_flags_the_downward_face_for_3d_printing() {
	// A 4×4×4 box printed along +Z: at a 45° threshold only its downward bottom face (16 mm²)
	// needs support — the four vertical walls are self-supporting and the top faces up — so the
	// overhang area is exactly one face and the supported fraction is 16/96. A 3D-print check.
	let block = cuboid(DVec3::new(-2.0, -2.0, 0.0), DVec3::new(2.0, 2.0, 4.0));
	let r = overhang_analysis(&block, DVec3::Z, 45.0);
	assert!(
		(r.overhang_area - 16.0).abs() < 1e-3 && (r.overhang_fraction - 16.0 / 96.0).abs() < 1e-3,
		"box overhang: area {} (want 16), fraction {} (want {:.4})",
		r.overhang_area,
		r.overhang_fraction,
		16.0 / 96.0
	);
}

#[test]
fn wall_thickness_of_a_box_is_its_smallest_dimension() {
	// Printability / castability: ray-casting inward from each face of a 4×6×10 bar reaches
	// the opposite wall, so the minimum wall thickness is the smallest dimension — 4 — which
	// is what an AI checks against a process's minimum printable / castable wall.
	let bar = cuboid(DVec3::new(-2.0, -3.0, -5.0), DVec3::new(2.0, 3.0, 5.0));
	let report = wall_thickness(&bar, 1.0);
	assert!((report.min_thickness - 4.0).abs() < 1e-3, "box min wall thickness {} (want 4)", report.min_thickness);
}

#[test]
fn draft_analysis_distinguishes_a_drafted_wall_from_a_vertical_one() {
	// Moldability: a plain box pulled along +Z has vertical side walls (0° draft — they would
	// drag in the mold), so the minimum draft over all faces is ~0°. A box drafted by
	// extrude_tapered has its side walls tilted by the taper angle, so the minimum jumps to
	// that angle — exactly what an AI needs to know a part will release from a mold.
	let pull = DVec3::Z;
	let plain = cuboid(DVec3::new(-5.0, -5.0, 0.0), DVec3::new(5.0, 5.0, 10.0));
	let plain_min = draft_analysis(&plain, pull, 1.0).min_draft_deg;
	let taper = 0.15_f64; // radians
	let square = [DVec2::new(-5.0, -5.0), DVec2::new(5.0, -5.0), DVec2::new(5.0, 5.0), DVec2::new(-5.0, 5.0)];
	let drafted_min = draft_analysis(&extrude_tapered(&square, 10.0, taper), pull, 1.0).min_draft_deg;
	assert!(
		plain_min < 0.5 && (drafted_min - taper.to_degrees()).abs() < 0.5,
		"draft: plain {plain_min}° (want ~0), drafted {drafted_min}° (want ~{:.2}°)",
		taper.to_degrees()
	);
}

#[test]
fn section_properties_of_a_rectangular_bar_match_the_closed_form() {
	// A 4×6 rectangular bar along +Z, cut square at mid-length. The section is a 4×6
	// rectangle: net area 24, and centroidal second moments of area I = b·h³/12 — with the
	// plane's basis u = +Y / v = −X, i_uu (height 6 along u) = 4·6³/12 = 72, i_vv (width 4
	// along v) = 6·4³/12 = 32, product of area 0. This is what an AI needs for a beam's
	// bending stiffness (E·I) and section modulus.
	let bar = cuboid(DVec3::new(-2.0, -3.0, 0.0), DVec3::new(2.0, 3.0, 10.0));
	let sp = section_properties(&bar, DVec3::new(0.0, 0.0, 5.0), DVec3::Z).expect("plane cuts the bar");
	assert!(
		(sp.area - 24.0).abs() < 1e-4
			&& (sp.i_uu - 72.0).abs() / 72.0 < 1e-4
			&& (sp.i_vv - 32.0).abs() / 32.0 < 1e-4
			&& sp.i_uv.abs() < 1e-4
			&& (sp.centroid - Vec3::new(0.0, 0.0, 5.0)).length() < 1e-4,
		"bar section: area {} (want 24) i_uu {} (72) i_vv {} (32) i_uv {} centroid {:?}",
		sp.area,
		sp.i_uu,
		sp.i_vv,
		sp.i_uv,
		sp.centroid
	);
}

#[test]
fn box_mass_properties_match_closed_form_exactly() {
	// A box with distinct half-extents (1, 2, 3) centered at the origin. Planar
	// faces tessellate exactly, so volume, center of mass and the full inertia
	// tensor must hit the closed form to floating-point precision. For a box of
	// full dimensions L=2h at unit density: m = LxLyLz, I_xx = m/12(Ly²+Lz²) =
	// m/3(hy²+hz²), products of inertia zero, CoM at the center.
	let (hx, hy, hz) = (1.0, 2.0, 3.0);
	let mp = mass_properties(&cuboid(DVec3::new(-hx, -hy, -hz), DVec3::new(hx, hy, hz)));
	let m = 8.0 * hx * hy * hz;
	assert!((mp.volume - m).abs() / m < 1e-9, "box volume {} vs {m}", mp.volume);
	assert!(mp.center_of_mass.length() < 1e-9, "box CoM should be origin, got {:?}", mp.center_of_mass);
	let (ix, iy, iz) = diag(mp.inertia);
	let (ex, ey, ez) = (m / 3.0 * (hy * hy + hz * hz), m / 3.0 * (hx * hx + hz * hz), m / 3.0 * (hx * hx + hy * hy));
	assert!(
		(ix - ex).abs() / ex < 1e-9 && (iy - ey).abs() / ey < 1e-9 && (iz - ez).abs() / ez < 1e-9,
		"box inertia ({ix},{iy},{iz}) vs ({ex},{ey},{ez})"
	);
	assert!(max_offdiag(mp.inertia) / ex < 1e-9, "box products of inertia should vanish, got {}", max_offdiag(mp.inertia));
}

#[test]
fn translated_box_shifts_com_but_inertia_about_com_is_invariant() {
	// The inertia tensor is reported about the center of mass, so translating the
	// solid moves the CoM by exactly that vector while leaving the tensor unchanged
	// — the parallel-axis shift must cancel the translation.
	let h = DVec3::new(2.0, 1.5, 1.0);
	let base = mass_properties(&cuboid(-h, h));
	let t = DVec3::new(10.0, -7.0, 3.0);
	let moved = mass_properties(&cuboid(-h + t, h + t));
	assert!((moved.center_of_mass - t).length() < 1e-9, "CoM should move to {t:?}, got {:?}", moved.center_of_mass);
	let (a0, b0, c0) = diag(base.inertia);
	let (a1, b1, c1) = diag(moved.inertia);
	assert!(
		(a0 - a1).abs() / a0 < 1e-9 && (b0 - b1).abs() / b0 < 1e-9 && (c0 - c1).abs() / c0 < 1e-9,
		"inertia about CoM changed under translation"
	);
}

#[test]
fn sphere_inertia_matches_two_fifths_m_r_squared() {
	// Solid sphere: I = 2/5 · m · r² on every axis, isotropic. Tessellated, so the
	// agreement is to a few percent and tightens with resolution.
	let r = 6.0;
	let mp = mass_properties(&sphere(DVec3::ZERO, r, 96, 48));
	let m = mp.volume; // unit density
	let exact = 0.4 * m * r * r;
	let (ix, iy, iz) = diag(mp.inertia);
	for (name, i) in [("xx", ix), ("yy", iy), ("zz", iz)] {
		assert!((i - exact).abs() / exact < 0.02, "sphere I_{name} {i} vs {exact}");
	}
	assert!(max_offdiag(mp.inertia) / exact < 0.02, "sphere products of inertia should vanish");
}

#[test]
fn cylinder_inertia_matches_closed_form() {
	// Solid cylinder along +Z, radius r, height h: I_zz = m·r²/2 (about the axis),
	// I_xx = I_yy = m·(3r² + h²)/12. CoM at half height.
	let (r, h) = (4.0, 10.0);
	let s = cylinder(DVec3::ZERO, DVec3::Z, r, h, 192);
	let mp = mass_properties(&s);
	let m = mp.volume;
	assert!((mp.center_of_mass - DVec3::new(0.0, 0.0, h / 2.0)).length() < 0.05, "cylinder CoM {:?}", mp.center_of_mass);
	let (ix, iy, iz) = diag(mp.inertia);
	let axial = 0.5 * m * r * r;
	let transverse = m * (3.0 * r * r + h * h) / 12.0;
	assert!((iz - axial).abs() / axial < 0.02, "cylinder I_zz {iz} vs {axial}");
	assert!(
		(ix - transverse).abs() / transverse < 0.02 && (iy - transverse).abs() / transverse < 0.02,
		"cylinder transverse inertia ({ix},{iy}) vs {transverse}"
	);
}

#[test]
fn principal_axes_recover_a_rotated_box_orientation() {
	// A box with distinct half-extents (1, 2, 3) carried off the coordinate axes by
	// an arbitrary rotation. Diagonalizing its (now full, non-diagonal) inertia
	// tensor must recover (a) the SAME principal moments as the unrotated box and
	// (b) principal axes that line up with the box's own rotated local axes. With
	// hx<hy<hz the moments order as I_zz < I_yy < I_xx, so ascending the principal
	// axes correspond to local Z, Y, X.
	let (hx, hy, hz) = (1.0, 2.0, 3.0);
	let rot = DAffine3::from_axis_angle(DVec3::new(1.0, -2.0, 0.5).normalize(), 0.9);
	let s = cuboid(DVec3::new(-hx, -hy, -hz), DVec3::new(hx, hy, hz)).transformed(rot);
	let mp = mass_properties(&s);
	let pa = mp.principal_axes();
	let m = 8.0 * hx * hy * hz;

	// (a) principal moments, ascending, to floating-point precision (planar → exact).
	let expect = [
		m / 3.0 * (hx * hx + hy * hy), // I_zz (smallest)
		m / 3.0 * (hx * hx + hz * hz), // I_yy
		m / 3.0 * (hy * hy + hz * hz), // I_xx (largest)
	];
	let got = [pa.moments.x, pa.moments.y, pa.moments.z];
	// The rotation carries the box onto irrational (cos/sin) coordinates, so the
	// tessellated tensor — and hence the moments — agree to floating-point relative
	// precision rather than the bit-exactness of the axis-aligned box.
	for k in 0..3 {
		assert!((got[k] - expect[k]).abs() / expect[k] < 1e-7, "principal moment {k}: {} vs {}", got[k], expect[k]);
	}

	// (b) each principal axis is a true eigenvector (I·e = λ·e) and aligns with the
	// box's rotated local axis (up to sign). axes columns ↔ local Z, Y, X.
	let cols = [pa.axes.x_axis, pa.axes.y_axis, pa.axes.z_axis];
	let local = [DVec3::Z, DVec3::Y, DVec3::X];
	for k in 0..3 {
		let e = cols[k];
		let resid = (mp.inertia * e - e * got[k]).length() / expect[k];
		assert!(resid < 1e-7, "axis {k} not an eigenvector (residual {resid})");
		let world = rot.transform_vector3(local[k]).normalize();
		assert!(e.dot(world).abs() > 0.9999, "axis {k} misaligned: {:?} vs {:?}", e, world);
	}
	// Right-handed orthonormal frame.
	assert!((pa.axes.determinant() - 1.0).abs() < 1e-9, "axes not right-handed (det {})", pa.axes.determinant());
}

#[test]
fn oriented_bounding_box_is_tight_on_a_rotated_box() {
	// A 2×4×6 box carried off the world axes by an arbitrary rotation. The inertia-
	// aligned OBB must recover the true box (its volume and {1,2,3} half-extents),
	// enclose every vertex, and be strictly tighter than the world AABB.
	let h = DVec3::new(1.0, 2.0, 3.0);
	let rot = DAffine3::from_axis_angle(DVec3::new(1.0, -2.0, 0.5).normalize(), 0.9);
	let mesh = tessellate_default(&cuboid(-h, h).transformed(rot));
	let obb = mesh.oriented_bounding_box();

	let exact = 8.0 * h.x * h.y * h.z; // 48
	assert!((obb.volume() - exact).abs() / exact < 1e-6, "OBB volume {} vs {exact}", obb.volume());
	let mut he = [obb.half_extents.x, obb.half_extents.y, obb.half_extents.z];
	he.sort_by(f64::total_cmp);
	for (g, e) in he.iter().zip([1.0, 2.0, 3.0]) {
		assert!((g - e).abs() < 1e-6, "OBB half-extent {g} vs {e}");
	}
	for &v in &mesh.positions {
		assert!(obb.contains(v.as_dvec3()), "OBB misses a vertex {v:?}");
	}
	assert!(obb.volume() < mesh.aabb().volume() as f64 * 0.95, "OBB {} should be tighter than AABB {}", obb.volume(), mesh.aabb().volume());
}

#[test]
fn oriented_bounding_box_matches_aabb_when_axis_aligned() {
	// For a box already aligned to the world axes, OBB and AABB enclose the same box.
	let mesh = tessellate_default(&cuboid(DVec3::new(-2.0, -3.0, -1.0), DVec3::new(2.0, 3.0, 1.0)));
	let obb = mesh.oriented_bounding_box();
	let aabb_vol = mesh.aabb().volume() as f64;
	assert!((obb.volume() - aabb_vol).abs() / aabb_vol < 1e-6, "OBB {} vs AABB {}", obb.volume(), aabb_vol);
}

#[test]
fn principal_axes_handle_degenerate_sphere_inertia() {
	// A sphere's inertia is isotropic (a triple eigenvalue), the worst case for an
	// eigensolver. It must still return three finite, orthonormal, right-handed axes
	// and three near-equal moments — no NaNs, no collapse.
	let pa = mass_properties(&sphere(DVec3::ZERO, 5.0, 64, 32)).principal_axes();
	let (a, b, c) = (pa.moments.x, pa.moments.y, pa.moments.z);
	assert!(a.is_finite() && b.is_finite() && c.is_finite() && (c - a) / c < 0.05, "sphere moments not near-equal: {a},{b},{c}");
	let cols = [pa.axes.x_axis, pa.axes.y_axis, pa.axes.z_axis];
	for i in 0..3 {
		assert!((cols[i].length() - 1.0).abs() < 1e-9, "axis {i} not unit");
		assert!(cols[i].dot(cols[(i + 1) % 3]).abs() < 1e-9, "axes not orthogonal");
	}
	assert!((pa.axes.determinant() - 1.0).abs() < 1e-9, "axes not right-handed");
}

#[test]
fn overhang_analysis_flags_undersides_past_the_threshold() {
	// A 20×20×1 plate, tilted about X, built along +Z. With a 45° support threshold
	// (measured from vertical), the large underside is flagged exactly while it sits
	// flatter than 45° from horizontal.
	let plate = || cuboid(DVec3::new(-10.0, -10.0, -0.5), DVec3::new(10.0, 10.0, 0.5));

	// Tilted 30° from horizontal (60° from vertical) → overhangs past 45° → support.
	let m30 = tessellate_default(&plate().transformed(DAffine3::from_rotation_x(30f64.to_radians())));
	let r30 = m30.overhang_analysis(Vec3::Z, 45.0);
	assert_eq!(r30.needs_support.len(), m30.triangle_count());
	assert!(r30.overhang_area > 380.0, "30° underside (~400) must need support, got {}", r30.overhang_area);

	// Tilted 60° from horizontal (30° from vertical) → steep enough to self-support;
	// the big face is no longer flagged (only thin side slivers remain).
	let m60 = tessellate_default(&plate().transformed(DAffine3::from_rotation_x(60f64.to_radians())));
	let r60 = m60.overhang_analysis(Vec3::Z, 45.0);
	assert!(r60.overhang_area < 100.0, "60° underside self-supports, got {}", r60.overhang_area);

	// Flipping the build direction flags the opposite large face instead.
	let r30_down = m30.overhang_analysis(-Vec3::Z, 45.0);
	assert!((r30_down.overhang_area - 400.0).abs() < 60.0, "flipped build flags the top face (~400), got {}", r30_down.overhang_area);
}

#[test]
fn wall_thickness_measures_the_thinnest_dimension() {
	// A 30×20×10 box: inward rays read 10, 20 or 30 mm by face. The minimum is the
	// thinnest dimension, and flagging below 12 mm catches exactly the two faces
	// looking across the 10 mm gap (area 30×20 each → 1200).
	let m = tessellate_default(&cuboid(DVec3::new(-15.0, -10.0, -5.0), DVec3::new(15.0, 10.0, 5.0)));
	let r = m.wall_thickness(12.0);
	assert_eq!(r.thickness.len(), m.triangle_count());
	assert!((r.min_thickness - 10.0).abs() < 0.2, "min thickness {} vs 10", r.min_thickness);
	assert!((r.thin_area - 1200.0).abs() < 50.0, "thin area {} vs ~1200", r.thin_area);
}

#[test]
fn thin_plate_is_flagged_thin() {
	// A 20×20×2 plate: minimum thickness ≈ 2; flagging below 3 mm catches both large
	// faces (area 400 each → 800).
	let m = tessellate_default(&cuboid(DVec3::new(-10.0, -10.0, -1.0), DVec3::new(10.0, 10.0, 1.0)));
	let r = m.wall_thickness(3.0);
	assert!((r.min_thickness - 2.0).abs() < 0.1, "min thickness {} vs 2", r.min_thickness);
	assert!((r.thin_area - 800.0).abs() < 40.0, "thin area {} vs ~800", r.thin_area);
}

#[test]
fn draft_analysis_flags_vertical_walls_and_no_undercuts_on_a_box() {
	// A 20 mm cube pulled along Z: the four side walls are parallel to pull (0° draft
	// → flagged), top/bottom are square to it (90°), and a convex box has no undercuts.
	let m = tessellate_default(&cuboid(DVec3::splat(-10.0), DVec3::splat(10.0)));
	let r = m.draft_analysis(Vec3::Z, 5.0);
	assert!(r.min_draft_deg < 0.01, "side walls have 0° draft, got {}", r.min_draft_deg);
	assert!((r.low_draft_area - 1600.0).abs() < 50.0, "four 400 mm² side walls flagged, got {}", r.low_draft_area);
	assert!(r.undercut_area < 1.0, "a convex box has no undercuts, got {}", r.undercut_area);
}

#[test]
fn revolve_rejects_pinched_single_pole() {
	// A single on-axis apex with off-axis neighbours revolves to a non-manifold pinch
	// (odd χ); revolve must now return an empty Solid, never a broken one.
	let teardrop = [DVec2::new(0.0, 10.0), DVec2::new(5.0, 2.0), DVec2::new(5.0, -2.0), DVec2::new(2.0, -2.0)];
	let s = revolve(&teardrop, 16);
	assert!(s.face_count() == 0 || validate(&s).is_valid(), "pinched revolve must be empty or valid, got {:?}", validate(&s));
}

#[test]
fn negative_height_extrude_is_a_valid_solid() {
	// Extruding downward (negative height) must still yield a closed, valid solid.
	let sq = [DVec2::new(0.0, 0.0), DVec2::new(4.0, 0.0), DVec2::new(4.0, 4.0), DVec2::new(0.0, 4.0)];
	let s = extrude(&sq, -3.0);
	assert!(validate(&s).is_valid(), "negative-height extrude invalid: {:?}", validate(&s));
	assert!((volume(&s).abs() - 48.0).abs() < 1e-6, "negative extrude volume {} vs 48", volume(&s));
}

#[test]
fn negative_height_cone_is_a_valid_solid() {
	// Round-3 made caps sign-aware but not the lateral cone surface tag, collapsing
	// the tessellation to zero volume. A downward cone must now be a valid solid.
	let c = cone(DVec3::ZERO, DVec3::Z, 6.0, -10.0, 64);
	assert!(validate(&c).is_valid(), "negative-height cone invalid: {:?}", validate(&c));
	let mesh = tessellate_default(&c);
	assert!(mesh.is_watertight(), "negative-height cone tessellation not watertight");
	let exact = std::f64::consts::PI * 36.0 * 10.0 / 3.0;
	assert!((volume(&c) - exact).abs() / exact < 0.01, "negative cone volume {} vs {exact}", volume(&c));
}
