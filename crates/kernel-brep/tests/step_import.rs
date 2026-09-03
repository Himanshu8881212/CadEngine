// Copyright (c) LMCAD. Licensed under the MIT License.

//! STEP import: round-trip and real-file reconstruction of planar B-reps.

use kernel_brep::math::{DVec2, DVec3};
use kernel_brep::{
	cone, cuboid, cylinder, difference, export_step, extrude_with_holes, import_bspline_curve, import_bspline_mesh,
	import_bspline_surface, import_step, import_step_assembly, sphere, tessellate_default, validate, volume, Curve,
	StepError, Surface,
};

#[test]
fn imports_a_bspline_curve_from_step() {
	// NURBS edge/trim curves read from STEP: a degree-2 Bézier (single span) curve over
	// three control points evaluates to the expected Bernstein blend at its midpoint.
	let step = "\
#1=CARTESIAN_POINT('',(0.,0.,0.));\n\
#2=CARTESIAN_POINT('',(1.,2.,0.));\n\
#3=CARTESIAN_POINT('',(2.,0.,0.));\n\
#10=B_SPLINE_CURVE_WITH_KNOTS('',2,(#1,#2,#3),.UNSPECIFIED.,.F.,.F.,(3,3),(0.,1.),.UNSPECIFIED.);\n";
	let curve = import_bspline_curve(step).expect("parse B-spline curve from STEP");
	// B(0.5) = 0.25·P0 + 0.5·P1 + 0.25·P2 = (1, 1, 0); ends interpolate the corners.
	let mid = curve.point_at(0.5);
	let p0 = curve.point_at(0.0);
	let p1 = curve.point_at(1.0);
	assert!(
		(mid - DVec3::new(1.0, 1.0, 0.0)).length() < 1e-9
			&& (p0 - DVec3::new(0.0, 0.0, 0.0)).length() < 1e-9
			&& (p1 - DVec3::new(2.0, 0.0, 0.0)).length() < 1e-9,
		"imported degree-2 B-spline curve: mid={mid:?} (want 1,1,0) p0={p0:?} p1={p1:?}"
	);
}

#[test]
fn imports_a_rational_bspline_circle_arc_from_step() {
	// A rational B-spline (the form CAD exporters use for circles/conics) is a _COMPLEX instance
	// whose RATIONAL_B_SPLINE_CURVE record carries per-control-point weights. A degree-2 rational
	// Bézier with weights (1, cos45°, 1) over (1,0)-(1,1)-(0,1) is EXACTLY a quarter of the unit
	// circle, so every point lies at radius 1. With the weights silently dropped to 1.0 the
	// midpoint would bulge to radius ~1.06 — so this proves the weights are read, not faked.
	let step = "\
#1=CARTESIAN_POINT('',(1.,0.,0.));\n\
#2=CARTESIAN_POINT('',(1.,1.,0.));\n\
#3=CARTESIAN_POINT('',(0.,1.,0.));\n\
#10=( BOUNDED_CURVE() B_SPLINE_CURVE(2,(#1,#2,#3),.UNSPECIFIED.,.F.,.F.) B_SPLINE_CURVE_WITH_KNOTS((3,3),(0.,1.),.UNSPECIFIED.) CURVE() GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE((1.,0.70710678118,1.)) REPRESENTATION_ITEM('') );\n";
	let curve = import_bspline_curve(step).expect("parse rational B-spline circle arc from STEP");
	let radius = |t: f64| {
		let p = curve.point_at(t);
		(p.x * p.x + p.y * p.y).sqrt()
	};
	assert!(
		(radius(0.0) - 1.0).abs() < 1e-6 && (radius(0.5) - 1.0).abs() < 1e-6 && (radius(1.0) - 1.0).abs() < 1e-6,
		"rational arc must lie on the unit circle (weights honoured): r(0)={} r(.5)={} r(1)={}",
		radius(0.0),
		radius(0.5),
		radius(1.0)
	);
}

#[test]
fn imports_and_meshes_a_curved_bspline_surface_from_step() {
	// End-to-end NURBS read path: a curved (degree-2×2 Bézier) patch with a raised centre
	// control point is imported from STEP and tessellated into a rich, in-bounds mesh.
	let step = "\
#1=CARTESIAN_POINT('',(0.,0.,0.));\n\
#2=CARTESIAN_POINT('',(0.,1.,0.));\n\
#3=CARTESIAN_POINT('',(0.,2.,0.));\n\
#4=CARTESIAN_POINT('',(1.,0.,0.));\n\
#5=CARTESIAN_POINT('',(1.,1.,1.));\n\
#6=CARTESIAN_POINT('',(1.,2.,0.));\n\
#7=CARTESIAN_POINT('',(2.,0.,0.));\n\
#8=CARTESIAN_POINT('',(2.,1.,0.));\n\
#9=CARTESIAN_POINT('',(2.,2.,0.));\n\
#20=B_SPLINE_SURFACE_WITH_KNOTS('',2,2,((#1,#2,#3),(#4,#5,#6),(#7,#8,#9)),.UNSPECIFIED.,.F.,.F.,.F.,(3,3),(3,3),(0.,1.),(0.,1.),.UNSPECIFIED.);\n";
	let mesh = import_bspline_mesh(step, 10, 10).expect("import + tessellate B-spline surface");
	let bb = mesh.aabb();
	assert!(
		mesh.triangle_count() > 100
			&& bb.min.x >= -0.1 && bb.max.x <= 2.1
			&& bb.min.y >= -0.1 && bb.max.y <= 2.1
			&& bb.min.z >= -0.1 && bb.max.z <= 0.6, // Bézier bulge reaches z≈0.25, never the control z=1
		"imported B-spline mesh must be rich + in-bounds: tris={} bb=[{:?},{:?}]",
		mesh.triangle_count(),
		bb.min,
		bb.max
	);
}

#[test]
fn imports_a_bspline_surface_from_step() {
	// The reading half of NURBS interchange: a B_SPLINE_SURFACE_WITH_KNOTS entity is
	// parsed into a NurbsSurface that evaluates correctly. This bilinear (degree 1×1)
	// patch interpolates its four corner control points, so the centre is their mean.
	let step = "\
#11=CARTESIAN_POINT('',(0.,0.,0.));\n\
#12=CARTESIAN_POINT('',(0.,2.,0.));\n\
#13=CARTESIAN_POINT('',(2.,0.,1.));\n\
#14=CARTESIAN_POINT('',(2.,2.,1.));\n\
#20=B_SPLINE_SURFACE_WITH_KNOTS('',1,1,((#11,#12),(#13,#14)),.UNSPECIFIED.,.F.,.F.,.F.,(2,2),(2,2),(0.,1.),(0.,1.),.UNSPECIFIED.);\n";
	let surf = import_bspline_surface(step).expect("parse B-spline surface from STEP");
	let mid = surf.point_at(0.5, 0.5);
	let c00 = surf.point_at(0.0, 0.0);
	let c11 = surf.point_at(1.0, 1.0);
	assert!(
		(mid - DVec3::new(1.0, 1.0, 0.5)).length() < 1e-9
			&& (c00 - DVec3::new(0.0, 0.0, 0.0)).length() < 1e-9
			&& (c11 - DVec3::new(2.0, 2.0, 1.0)).length() < 1e-9,
		"imported bilinear B-spline: mid={mid:?} (want 1,1,0.5) c00={c00:?} c11={c11:?}"
	);
}

#[test]
fn imports_a_rational_bspline_cylinder_patch_from_step() {
	// A cylindrical/conical surface exports as a RATIONAL B-spline (a _COMPLEX whose
	// RATIONAL_B_SPLINE_SURFACE record carries a weight GRID). This is a quarter-cylinder of
	// radius 1, height 4: degree 2 in u (the rational arc, weights (1,cos45°,1)) × degree 1 in v
	// (the straight height). Its mid-surface point must sit on the true cylinder (radius 1) at
	// mid-height — with the weights dropped to 1.0 the arc bulges to radius ~1.06, so this proves
	// the weight grid is read, not faked.
	let step = "\
#1=CARTESIAN_POINT('',(1.,0.,0.));\n\
#2=CARTESIAN_POINT('',(1.,0.,4.));\n\
#3=CARTESIAN_POINT('',(1.,1.,0.));\n\
#4=CARTESIAN_POINT('',(1.,1.,4.));\n\
#5=CARTESIAN_POINT('',(0.,1.,0.));\n\
#6=CARTESIAN_POINT('',(0.,1.,4.));\n\
#20=( BOUNDED_SURFACE() B_SPLINE_SURFACE(2,1,((#1,#2),(#3,#4),(#5,#6)),.UNSPECIFIED.,.F.,.F.,.F.) B_SPLINE_SURFACE_WITH_KNOTS((3,3),(2,2),(0.,1.),(0.,1.),.UNSPECIFIED.) GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_SURFACE(((1.,1.),(0.70710678118,0.70710678118),(1.,1.))) REPRESENTATION_ITEM('') SURFACE() );\n";
	let surf = import_bspline_surface(step).expect("parse rational B-spline cylinder patch from STEP");
	let p = surf.point_at(0.5, 0.5);
	let radius = (p.x * p.x + p.y * p.y).sqrt();
	assert!(
		(radius - 1.0).abs() < 1e-6 && (p.z - 2.0).abs() < 1e-6,
		"rational cylinder patch mid-point must lie on the unit cylinder at mid-height (weights honoured): p={p:?} radius={radius}"
	);
}

#[test]
fn round_trips_a_planar_solid_through_step() {
	// Export a box to STEP, parse it back, and confirm the reconstructed B-rep is a
	// valid closed manifold of the same volume — geometry survived the text format.
	let orig = cuboid(DVec3::new(-10.0, -6.0, -3.0), DVec3::new(10.0, 6.0, 3.0)); // 20·12·6 = 1440
	let step = export_step(&orig, "box");
	let back = import_step(&step).expect("import should succeed");

	let v = validate(&back);
	assert!(v.closed && v.manifold, "imported solid must be a closed manifold: {v:?}");
	assert_eq!(back.face_count(), 6, "a box has six faces");
	assert!((volume(&back).abs() - 1440.0).abs() < 1e-6, "imported volume {} vs 1440", volume(&back).abs());
	assert!(tessellate_default(&back).is_watertight(), "imported solid tessellates watertight");
}

#[test]
fn round_trips_curved_primitives_through_step() {
	// Cylinder, sphere and cone carry CYLINDRICAL/SPHERICAL/CONICAL_SURFACE tags.
	// The vertices (hence volume and topology) must survive the STEP round-trip, and
	// the imported solid must be a valid closed manifold.
	let makers: [fn() -> kernel_brep::Solid; 3] = [
		|| cylinder(DVec3::ZERO, DVec3::Z, 5.0, 10.0, 32),
		|| sphere(DVec3::ZERO, 6.0, 24, 16),
		|| cone(DVec3::ZERO, DVec3::Z, 5.0, 12.0, 32),
	];
	for make in makers {
		let solid = make();
		let want_vol = volume(&solid).abs();
		let back = import_step(&export_step(&solid, "part")).expect("curved primitive should import");
		let v = validate(&back);
		assert!(v.closed && v.manifold, "imported curved solid must be a closed manifold: {v:?}");
		assert!((volume(&back).abs() - want_vol).abs() / want_vol < 1e-6, "round-trip volume {} vs {want_vol}", volume(&back).abs());
		assert!(tessellate_default(&back).is_watertight());
	}
}

#[test]
fn round_trips_a_drilled_block_through_step() {
	// A MULTI-FEATURE part — a block with a through-hole (a boolean difference: genus 1,
	// mixing planar faces with a cylindrical bore wall) — must survive the STEP round-trip as
	// a valid genus-1 manifold of the same volume. Harder than a lone primitive: it exercises
	// boolean-created topology with inner loops + a curved interior face through the text form.
	let block = cuboid(DVec3::new(-8.0, -8.0, -3.0), DVec3::new(8.0, 8.0, 3.0));
	let drill = cylinder(DVec3::new(0.0, 0.0, -4.0), DVec3::Z, 3.0, 8.0, 48);
	let part = difference(&block, &drill);
	let want_vol = volume(&part).abs();
	let back = import_step(&export_step(&part, "drilled_block")).expect("drilled block should import");
	let v = validate(&back);
	assert!(
		v.closed && v.manifold && v.genus == 1 && (volume(&back).abs() - want_vol).abs() / want_vol < 1e-6,
		"STEP round-trip of a drilled block: {v:?}, vol {} vs {want_vol}",
		volume(&back).abs()
	);
}

#[test]
fn round_trips_a_genus_3_cross_bore_manifold_through_step() {
	// The strongest topology case: a block with TWO intersecting perpendicular bores
	// (a Steinmetz seam, genus 3 — multiple handles, curved interior faces meeting on a
	// curved seam). It must survive the STEP text round-trip as a valid genus-3 manifold
	// of the same volume, proving multi-handle boolean topology serialises and re-imports.
	let block = cuboid(DVec3::splat(-25.0), DVec3::splat(25.0));
	let bx = cylinder(DVec3::new(-30.0, 0.0, 0.0), DVec3::X, 10.0, 60.0, 32);
	let by = cylinder(DVec3::new(0.0, -30.0, 0.0), DVec3::Y, 10.0, 60.0, 32);
	let part = difference(&difference(&block, &bx), &by);
	let want_vol = volume(&part).abs();
	let want_genus = validate(&part).genus;
	let back = import_step(&export_step(&part, "cross_bore_manifold")).expect("genus-3 manifold should import");
	let v = validate(&back);
	assert!(
		v.closed && v.manifold && v.genus == 3 && want_genus == 3 && (volume(&back).abs() - want_vol).abs() / want_vol < 1e-6,
		"STEP round-trip of a genus-3 cross-bore manifold: {v:?} (orig genus {want_genus}), vol {} vs {want_vol}",
		volume(&back).abs()
	);
}

#[test]
fn imports_a_real_step_file_from_disk() {
	// A producer-written STEP file (the project's golden block), not our own export.
	let text = include_str!("fixtures/showcase_block.step");
	let solid = import_step(text).expect("golden STEP block should import");
	let v = validate(&solid);
	assert!(v.closed && v.manifold && solid.face_count() == 6, "golden block is a closed 6-face manifold: {v:?}");
	assert!(volume(&solid).abs() > 0.0 && tessellate_default(&solid).is_watertight());
}

#[test]
fn imports_a_real_exporter_style_cylinder_with_full_circle_caps() {
	// The shape FreeCAD/SolidWorks (OpenCascade-family) emit for a cylinder r=5 h=10:
	// each cap is ONE full-circle EDGE_CURVE (start vertex == end vertex), and the wall
	// is ONE periodic cylindrical face whose loop is [bottom circle, seam up, top
	// circle reversed, seam down] — the seam edge appears twice with opposite senses.
	// The importer must tessellate the circle edges into 48-segment rings and split
	// the periodic wall into chord facets on the exact cylinder.
	let step = "\
#1=CARTESIAN_POINT('',(0.,0.,0.));\n\
#2=DIRECTION('',(0.,0.,1.));\n\
#3=DIRECTION('',(1.,0.,0.));\n\
#4=AXIS2_PLACEMENT_3D('',#1,#2,#3);\n\
#5=CIRCLE('',#4,5.);\n\
#6=CARTESIAN_POINT('',(0.,0.,10.));\n\
#7=AXIS2_PLACEMENT_3D('',#6,#2,#3);\n\
#8=CIRCLE('',#7,5.);\n\
#10=CARTESIAN_POINT('',(5.,0.,0.));\n\
#11=VERTEX_POINT('',#10);\n\
#12=CARTESIAN_POINT('',(5.,0.,10.));\n\
#13=VERTEX_POINT('',#12);\n\
#14=EDGE_CURVE('',#11,#11,#5,.T.);\n\
#15=EDGE_CURVE('',#13,#13,#8,.T.);\n\
#17=VECTOR('',#2,1.);\n\
#18=LINE('',#10,#17);\n\
#19=EDGE_CURVE('',#11,#13,#18,.T.);\n\
#21=ORIENTED_EDGE('',*,*,#14,.F.);\n\
#22=EDGE_LOOP('',(#21));\n\
#23=FACE_OUTER_BOUND('',#22,.T.);\n\
#24=PLANE('',#4);\n\
#25=ADVANCED_FACE('',(#23),#24,.F.);\n\
#26=ORIENTED_EDGE('',*,*,#15,.T.);\n\
#27=EDGE_LOOP('',(#26));\n\
#28=FACE_OUTER_BOUND('',#27,.T.);\n\
#29=PLANE('',#7);\n\
#30=ADVANCED_FACE('',(#28),#29,.T.);\n\
#31=ORIENTED_EDGE('',*,*,#14,.T.);\n\
#32=ORIENTED_EDGE('',*,*,#19,.T.);\n\
#33=ORIENTED_EDGE('',*,*,#15,.F.);\n\
#34=ORIENTED_EDGE('',*,*,#19,.F.);\n\
#35=EDGE_LOOP('',(#31,#32,#33,#34));\n\
#36=FACE_OUTER_BOUND('',#35,.T.);\n\
#37=CYLINDRICAL_SURFACE('',#4,5.);\n\
#38=ADVANCED_FACE('',(#36),#37,.T.);\n";
	let solid = import_step(step).expect("a real-exporter-style cylinder must import");
	let v = validate(&solid);
	let vol = volume(&solid).abs();
	let want = std::f64::consts::PI * 25.0 * 10.0; // πr²h = 785.4; a 48-segment faceting sits ~0.3% under
	let wall_facets = solid
		.faces()
		.filter(|&f| matches!(solid.face(f).surface, Surface::Cylinder { radius, .. } if (radius - 5.0).abs() < 1e-9))
		.count();
	let plane_caps = solid.faces().filter(|&f| matches!(solid.face(f).surface, Surface::Plane { .. })).count();
	let circle_edges = solid
		.edges()
		.filter(|&e| matches!(solid.edge_curve(e), Some(Curve::Circle { radius, .. }) if (radius - 5.0).abs() < 1e-9))
		.count();
	assert!(
		v.closed
			&& v.manifold
			&& v.genus == 0
			&& (vol - want).abs() / want < 0.01
			&& wall_facets >= 90
			&& plane_caps == 2
			&& circle_edges == 96
			&& tessellate_default(&solid).is_watertight(),
		"arc-bounded cylinder import: {v:?}, vol {vol} (want ≈{want}), {wall_facets} cylinder facets, \
		 {plane_caps} planar caps, {circle_edges} circle-tagged ring edges"
	);
}

#[test]
fn round_trips_a_washer_with_inner_loops_through_step() {
	// A washer built with multi-loop caps (Face::inner populated): the exporter must
	// write the hole loops as FACE_BOUNDs (previously they were silently dropped) and
	// the importer must rebuild multi-loop faces — genus 1 and exact volume preserved.
	let outer: Vec<DVec2> = vec![
		DVec2::new(-10.0, -10.0),
		DVec2::new(10.0, -10.0),
		DVec2::new(10.0, 10.0),
		DVec2::new(-10.0, 10.0),
	];
	let hole: Vec<DVec2> = vec![
		DVec2::new(-4.0, -4.0),
		DVec2::new(4.0, -4.0),
		DVec2::new(4.0, 4.0),
		DVec2::new(-4.0, 4.0),
	];
	let washer = extrude_with_holes(&outer, &[hole], 6.0);
	let want_vol = volume(&washer).abs(); // (400 − 64)·6 = 2016, exact for planar faces
	let step = export_step(&washer, "washer");
	assert!(step.contains("FACE_BOUND('"), "export must write inner loops as FACE_BOUND records");
	let back = import_step(&step).expect("washer with inner loops must round-trip");
	let v = validate(&back);
	let vol = volume(&back).abs();
	assert!(
		v.closed && v.manifold && v.genus == 1 && (vol - want_vol).abs() / want_vol < 1e-6,
		"washer round-trip: {v:?}, vol {vol} vs {want_vol}"
	);
}

#[test]
fn imports_an_external_style_washer_with_circular_hole_bounds() {
	// External-producer washer r_out=8, r_in=3, h=4: BOTH caps are multi-loop planar
	// faces (outer bound + a FACE_BOUND hole, each a single full-circle edge), and both
	// walls are periodic cylindrical faces with seams — inner loops (T2) and
	// arc-bounded faces (T1) in one part. Must import as a valid genus-1 solid.
	let step = "\
#1=CARTESIAN_POINT('',(0.,0.,0.));\n\
#2=DIRECTION('',(0.,0.,1.));\n\
#3=DIRECTION('',(1.,0.,0.));\n\
#4=AXIS2_PLACEMENT_3D('',#1,#2,#3);\n\
#5=CARTESIAN_POINT('',(0.,0.,4.));\n\
#6=AXIS2_PLACEMENT_3D('',#5,#2,#3);\n\
#7=CIRCLE('',#4,8.);\n\
#8=CIRCLE('',#6,8.);\n\
#9=CIRCLE('',#4,3.);\n\
#10=CIRCLE('',#6,3.);\n\
#11=CARTESIAN_POINT('',(8.,0.,0.));\n\
#12=VERTEX_POINT('',#11);\n\
#13=CARTESIAN_POINT('',(8.,0.,4.));\n\
#14=VERTEX_POINT('',#13);\n\
#15=CARTESIAN_POINT('',(3.,0.,0.));\n\
#16=VERTEX_POINT('',#15);\n\
#17=CARTESIAN_POINT('',(3.,0.,4.));\n\
#18=VERTEX_POINT('',#17);\n\
#19=EDGE_CURVE('',#12,#12,#7,.T.);\n\
#20=EDGE_CURVE('',#14,#14,#8,.T.);\n\
#21=EDGE_CURVE('',#16,#16,#9,.T.);\n\
#22=EDGE_CURVE('',#18,#18,#10,.T.);\n\
#23=VECTOR('',#2,1.);\n\
#24=LINE('',#11,#23);\n\
#25=EDGE_CURVE('',#12,#14,#24,.T.);\n\
#26=LINE('',#15,#23);\n\
#27=EDGE_CURVE('',#16,#18,#26,.T.);\n\
#30=ORIENTED_EDGE('',*,*,#19,.T.);\n\
#31=ORIENTED_EDGE('',*,*,#25,.T.);\n\
#32=ORIENTED_EDGE('',*,*,#20,.F.);\n\
#33=ORIENTED_EDGE('',*,*,#25,.F.);\n\
#34=EDGE_LOOP('',(#30,#31,#32,#33));\n\
#35=FACE_OUTER_BOUND('',#34,.T.);\n\
#36=CYLINDRICAL_SURFACE('',#4,8.);\n\
#37=ADVANCED_FACE('',(#35),#36,.T.);\n\
#40=ORIENTED_EDGE('',*,*,#27,.T.);\n\
#41=ORIENTED_EDGE('',*,*,#22,.T.);\n\
#42=ORIENTED_EDGE('',*,*,#27,.F.);\n\
#43=ORIENTED_EDGE('',*,*,#21,.F.);\n\
#44=EDGE_LOOP('',(#40,#41,#42,#43));\n\
#45=FACE_OUTER_BOUND('',#44,.T.);\n\
#46=CYLINDRICAL_SURFACE('',#4,3.);\n\
#47=ADVANCED_FACE('',(#45),#46,.F.);\n\
#50=ORIENTED_EDGE('',*,*,#19,.F.);\n\
#51=EDGE_LOOP('',(#50));\n\
#52=FACE_OUTER_BOUND('',#51,.T.);\n\
#53=ORIENTED_EDGE('',*,*,#21,.T.);\n\
#54=EDGE_LOOP('',(#53));\n\
#55=FACE_BOUND('',#54,.T.);\n\
#56=PLANE('',#4);\n\
#57=ADVANCED_FACE('',(#52,#55),#56,.F.);\n\
#60=ORIENTED_EDGE('',*,*,#20,.T.);\n\
#61=EDGE_LOOP('',(#60));\n\
#62=FACE_OUTER_BOUND('',#61,.T.);\n\
#63=ORIENTED_EDGE('',*,*,#22,.F.);\n\
#64=EDGE_LOOP('',(#63));\n\
#65=FACE_BOUND('',#64,.T.);\n\
#66=PLANE('',#6);\n\
#67=ADVANCED_FACE('',(#62,#65),#66,.T.);\n";
	let solid = import_step(step).expect("external-style washer must import");
	let v = validate(&solid);
	let vol = volume(&solid).abs();
	let want = std::f64::consts::PI * (64.0 - 9.0) * 4.0; // π(R²−r²)h = 691.2, 48-gon faceting ~0.3% under
	assert!(
		v.closed && v.manifold && v.genus == 1 && v.shells == 1 && (vol - want).abs() / want < 0.01,
		"external washer import: {v:?}, vol {vol} (want ≈{want})"
	);
}

#[test]
fn imports_an_obliquely_cut_cylinder_with_an_ellipse_cap() {
	// A cylinder r=5 cut by a plane tilted 30°: the cap rim is ONE full-ellipse
	// EDGE_CURVE (semi-axes 5/cos30° × 5). The importer must tessellate the ellipse
	// into the boundary, attach the analytic Curve::Ellipse to its segments, and split
	// the wall (bounded below by a circle, above by the ellipse) on the exact cylinder.
	let step = "\
#1=CARTESIAN_POINT('',(0.,0.,0.));\n\
#2=DIRECTION('',(0.,0.,1.));\n\
#3=DIRECTION('',(1.,0.,0.));\n\
#4=AXIS2_PLACEMENT_3D('',#1,#2,#3);\n\
#5=CIRCLE('',#4,5.);\n\
#6=CARTESIAN_POINT('',(0.,0.,10.));\n\
#7=DIRECTION('',(-0.5,0.,0.8660254037844387));\n\
#8=DIRECTION('',(0.8660254037844387,0.,0.5));\n\
#9=AXIS2_PLACEMENT_3D('',#6,#7,#8);\n\
#10=ELLIPSE('',#9,5.773502691896258,5.);\n\
#11=CARTESIAN_POINT('',(5.,0.,0.));\n\
#12=VERTEX_POINT('',#11);\n\
#13=CARTESIAN_POINT('',(5.,0.,12.886751345948129));\n\
#14=VERTEX_POINT('',#13);\n\
#15=EDGE_CURVE('',#12,#12,#5,.T.);\n\
#16=EDGE_CURVE('',#14,#14,#10,.T.);\n\
#17=VECTOR('',#2,1.);\n\
#18=LINE('',#11,#17);\n\
#19=EDGE_CURVE('',#12,#14,#18,.T.);\n\
#20=ORIENTED_EDGE('',*,*,#15,.T.);\n\
#21=ORIENTED_EDGE('',*,*,#19,.T.);\n\
#22=ORIENTED_EDGE('',*,*,#16,.F.);\n\
#23=ORIENTED_EDGE('',*,*,#19,.F.);\n\
#24=EDGE_LOOP('',(#20,#21,#22,#23));\n\
#25=FACE_OUTER_BOUND('',#24,.T.);\n\
#26=CYLINDRICAL_SURFACE('',#4,5.);\n\
#27=ADVANCED_FACE('',(#25),#26,.T.);\n\
#30=ORIENTED_EDGE('',*,*,#15,.F.);\n\
#31=EDGE_LOOP('',(#30));\n\
#32=FACE_OUTER_BOUND('',#31,.T.);\n\
#33=PLANE('',#4);\n\
#34=ADVANCED_FACE('',(#32),#33,.F.);\n\
#35=ORIENTED_EDGE('',*,*,#16,.T.);\n\
#36=EDGE_LOOP('',(#35));\n\
#37=FACE_OUTER_BOUND('',#36,.T.);\n\
#38=PLANE('',#9);\n\
#39=ADVANCED_FACE('',(#37),#38,.T.);\n";
	let solid = import_step(step).expect("an obliquely cut cylinder with an ellipse cap must import");
	let v = validate(&solid);
	let vol = volume(&solid).abs();
	let want = std::f64::consts::PI * 25.0 * 10.0; // mean height is 10, so volume = πr²·10 again
	let ellipse_edges = solid
		.edges()
		.filter(|&e| {
			matches!(solid.edge_curve(e), Some(Curve::Ellipse { a, b, .. })
				if (a - 5.773502691896258).abs() < 1e-9 && (b - 5.0).abs() < 1e-9)
		})
		.count();
	let wall_facets = solid
		.faces()
		.filter(|&f| matches!(solid.face(f).surface, Surface::Cylinder { radius, .. } if (radius - 5.0).abs() < 1e-9))
		.count();
	assert!(
		v.closed && v.manifold && v.genus == 0 && (vol - want).abs() / want < 0.01 && ellipse_edges == 48 && wall_facets >= 90,
		"oblique-cut cylinder import: {v:?}, vol {vol} (want ≈{want}), {ellipse_edges} ellipse-tagged edges, {wall_facets} cylinder facets"
	);
}

#[test]
fn imports_a_bspline_topped_box_on_the_exact_patch() {
	// A box whose TOP face is a degree-2×2 B_SPLINE_SURFACE_WITH_KNOTS with a raised
	// centre control point but straight boundary edges. The trimmed-NURBS route
	// triangulates the face ON the exact patch (trim chords kept verbatim, interior
	// refined on the surface), so the import carries the bulge volume — and still
	// welds watertight against the planar walls.
	let step = "\
#1=CARTESIAN_POINT('',(0.,0.,0.));\n\
#2=CARTESIAN_POINT('',(10.,0.,0.));\n\
#3=CARTESIAN_POINT('',(10.,10.,0.));\n\
#4=CARTESIAN_POINT('',(0.,10.,0.));\n\
#5=CARTESIAN_POINT('',(0.,0.,5.));\n\
#6=CARTESIAN_POINT('',(10.,0.,5.));\n\
#7=CARTESIAN_POINT('',(10.,10.,5.));\n\
#8=CARTESIAN_POINT('',(0.,10.,5.));\n\
#11=VERTEX_POINT('',#1);\n\
#12=VERTEX_POINT('',#2);\n\
#13=VERTEX_POINT('',#3);\n\
#14=VERTEX_POINT('',#4);\n\
#15=VERTEX_POINT('',#5);\n\
#16=VERTEX_POINT('',#6);\n\
#17=VERTEX_POINT('',#7);\n\
#18=VERTEX_POINT('',#8);\n\
#20=DIRECTION('',(1.,0.,0.));\n\
#21=DIRECTION('',(0.,1.,0.));\n\
#22=DIRECTION('',(0.,0.,1.));\n\
#23=VECTOR('',#20,1.);\n\
#24=VECTOR('',#21,1.);\n\
#25=VECTOR('',#22,1.);\n\
#30=LINE('',#1,#23);\n\
#31=EDGE_CURVE('',#11,#12,#30,.T.);\n\
#32=LINE('',#2,#24);\n\
#33=EDGE_CURVE('',#12,#13,#32,.T.);\n\
#34=LINE('',#4,#23);\n\
#35=EDGE_CURVE('',#14,#13,#34,.T.);\n\
#36=LINE('',#1,#24);\n\
#37=EDGE_CURVE('',#11,#14,#36,.T.);\n\
#38=LINE('',#5,#23);\n\
#39=EDGE_CURVE('',#15,#16,#38,.T.);\n\
#40=LINE('',#6,#24);\n\
#41=EDGE_CURVE('',#16,#17,#40,.T.);\n\
#42=LINE('',#8,#23);\n\
#43=EDGE_CURVE('',#18,#17,#42,.T.);\n\
#44=LINE('',#5,#24);\n\
#45=EDGE_CURVE('',#15,#18,#44,.T.);\n\
#46=LINE('',#1,#25);\n\
#47=EDGE_CURVE('',#11,#15,#46,.T.);\n\
#48=LINE('',#2,#25);\n\
#49=EDGE_CURVE('',#12,#16,#48,.T.);\n\
#50=LINE('',#3,#25);\n\
#51=EDGE_CURVE('',#13,#17,#50,.T.);\n\
#52=LINE('',#4,#25);\n\
#53=EDGE_CURVE('',#14,#18,#52,.T.);\n\
#60=ORIENTED_EDGE('',*,*,#31,.F.);\n\
#61=ORIENTED_EDGE('',*,*,#37,.T.);\n\
#62=ORIENTED_EDGE('',*,*,#35,.T.);\n\
#63=ORIENTED_EDGE('',*,*,#33,.F.);\n\
#64=EDGE_LOOP('',(#60,#61,#62,#63));\n\
#65=FACE_OUTER_BOUND('',#64,.T.);\n\
#66=CARTESIAN_POINT('',(5.,5.,8.));\n\
#67=CARTESIAN_POINT('',(5.,0.,5.));\n\
#68=CARTESIAN_POINT('',(10.,5.,5.));\n\
#69=CARTESIAN_POINT('',(5.,10.,5.));\n\
#70=CARTESIAN_POINT('',(0.,5.,5.));\n\
#71=B_SPLINE_SURFACE_WITH_KNOTS('',2,2,((#5,#70,#8),(#67,#66,#69),(#6,#68,#7)),.UNSPECIFIED.,.F.,.F.,.F.,(3,3),(3,3),(0.,1.),(0.,1.),.UNSPECIFIED.);\n\
#72=ORIENTED_EDGE('',*,*,#39,.T.);\n\
#73=ORIENTED_EDGE('',*,*,#41,.T.);\n\
#74=ORIENTED_EDGE('',*,*,#43,.F.);\n\
#75=ORIENTED_EDGE('',*,*,#45,.F.);\n\
#76=EDGE_LOOP('',(#72,#73,#74,#75));\n\
#77=FACE_OUTER_BOUND('',#76,.T.);\n\
#78=ADVANCED_FACE('',(#77),#71,.T.);\n\
#80=ORIENTED_EDGE('',*,*,#31,.T.);\n\
#81=ORIENTED_EDGE('',*,*,#49,.T.);\n\
#82=ORIENTED_EDGE('',*,*,#39,.F.);\n\
#83=ORIENTED_EDGE('',*,*,#47,.F.);\n\
#84=EDGE_LOOP('',(#80,#81,#82,#83));\n\
#85=FACE_OUTER_BOUND('',#84,.T.);\n\
#86=DIRECTION('',(0.,-1.,0.));\n\
#87=AXIS2_PLACEMENT_3D('',#1,#86,#20);\n\
#88=PLANE('',#87);\n\
#89=ADVANCED_FACE('',(#85),#88,.T.);\n\
#90=ORIENTED_EDGE('',*,*,#33,.T.);\n\
#91=ORIENTED_EDGE('',*,*,#51,.T.);\n\
#92=ORIENTED_EDGE('',*,*,#41,.F.);\n\
#93=ORIENTED_EDGE('',*,*,#49,.F.);\n\
#94=EDGE_LOOP('',(#90,#91,#92,#93));\n\
#95=FACE_OUTER_BOUND('',#94,.T.);\n\
#96=AXIS2_PLACEMENT_3D('',#2,#20,#21);\n\
#97=PLANE('',#96);\n\
#98=ADVANCED_FACE('',(#95),#97,.T.);\n\
#100=ORIENTED_EDGE('',*,*,#35,.F.);\n\
#101=ORIENTED_EDGE('',*,*,#53,.T.);\n\
#102=ORIENTED_EDGE('',*,*,#43,.T.);\n\
#103=ORIENTED_EDGE('',*,*,#51,.F.);\n\
#104=EDGE_LOOP('',(#100,#101,#102,#103));\n\
#105=FACE_OUTER_BOUND('',#104,.T.);\n\
#106=AXIS2_PLACEMENT_3D('',#3,#21,#20);\n\
#107=PLANE('',#106);\n\
#108=ADVANCED_FACE('',(#105),#107,.T.);\n\
#110=ORIENTED_EDGE('',*,*,#37,.F.);\n\
#111=ORIENTED_EDGE('',*,*,#47,.T.);\n\
#112=ORIENTED_EDGE('',*,*,#45,.T.);\n\
#113=ORIENTED_EDGE('',*,*,#53,.F.);\n\
#114=EDGE_LOOP('',(#110,#111,#112,#113));\n\
#115=FACE_OUTER_BOUND('',#114,.T.);\n\
#116=DIRECTION('',(-1.,0.,0.));\n\
#117=AXIS2_PLACEMENT_3D('',#1,#116,#21);\n\
#118=PLANE('',#117);\n\
#119=ADVANCED_FACE('',(#115),#118,.T.);\n\
#120=ORIENTED_EDGE('',*,*,#37,.T.);\n\
#121=ORIENTED_EDGE('',*,*,#35,.T.);\n\
#122=ORIENTED_EDGE('',*,*,#33,.F.);\n\
#123=ORIENTED_EDGE('',*,*,#31,.F.);\n\
#124=EDGE_LOOP('',(#120,#121,#122,#123));\n\
#125=FACE_OUTER_BOUND('',#124,.T.);\n\
#126=DIRECTION('',(0.,0.,-1.));\n\
#127=AXIS2_PLACEMENT_3D('',#1,#126,#20);\n\
#128=PLANE('',#127);\n\
#129=ADVANCED_FACE('',(#125),#128,.F.);\n";
	let solid = import_step(step).expect("a B-spline-topped box must import on the exact patch");
	let v = validate(&solid);
	let vol = volume(&solid).abs();
	// Exact bulge volume of the tensor-quadratic patch: x = 10u, y = 10v, and
	// ∫∫ Bᵢ(u)Bⱼ(v) du dv = 1/9 per control, so ∫∫ (z − 5) dx dy = 100·(8 − 5)/9.
	let want = 500.0 + 100.0 * 3.0 / 9.0;
	let rel = (vol - want).abs() / want;
	let watertight = tessellate_default(&solid).is_watertight();
	assert!(
		v.closed && v.manifold && v.genus == 0 && solid.face_count() > 6 && rel < 0.005 && watertight,
		"B-spline-topped box must import closed+watertight WITH the patch bulge (interior \
		 vertices on the exact surface): {v:?}, faces {}, vol {vol:.4} vs exact {want:.4} (rel {rel:.5}), watertight {watertight}",
		solid.face_count()
	);
}

#[test]
fn unsupported_periodic_and_revolved_faces_stay_loud() {
	// The quality contract: anything outside the support matrix is a typed, actionable
	// error — never a silent drop or a garbage solid.
	let placement = "\
#1=CARTESIAN_POINT('',(0.,0.,0.));\n\
#2=DIRECTION('',(0.,0.,1.));\n\
#3=DIRECTION('',(1.,0.,0.));\n\
#4=AXIS2_PLACEMENT_3D('',#1,#2,#3);\n\
#5=CIRCLE('',#4,5.);\n\
#10=CARTESIAN_POINT('',(5.,0.,0.));\n\
#11=VERTEX_POINT('',#10);\n\
#14=EDGE_CURVE('',#11,#11,#5,.T.);\n\
#21=ORIENTED_EDGE('',*,*,#14,.T.);\n\
#22=EDGE_LOOP('',(#21));\n\
#23=FACE_OUTER_BOUND('',#22,.T.);\n";
	// (a) A HALF-torus wall (torus cut by a plane through its axis): bounded by two
	// full tube circles plus an equator seam arc — periodic around the tube but only a
	// PARTIAL turn about the axis. The ring-grid resampler covers full-turn rings,
	// caps and bands; this needs partial-turn handling and must refuse loudly.
	let half_torus = "\
#1=CARTESIAN_POINT('',(0.,0.,0.));\n\
#2=DIRECTION('',(0.,0.,1.));\n\
#3=DIRECTION('',(1.,0.,0.));\n\
#4=AXIS2_PLACEMENT_3D('',#1,#2,#3);\n\
#5=CARTESIAN_POINT('',(8.,0.,0.));\n\
#6=DIRECTION('',(0.,-1.,0.));\n\
#7=AXIS2_PLACEMENT_3D('',#5,#6,#3);\n\
#8=CIRCLE('',#7,2.5);\n\
#9=CARTESIAN_POINT('',(-8.,0.,0.));\n\
#12=AXIS2_PLACEMENT_3D('',#9,#6,#3);\n\
#13=CIRCLE('',#12,2.5);\n\
#14=CIRCLE('',#4,10.5);\n\
#15=CARTESIAN_POINT('',(10.5,0.,0.));\n\
#16=VERTEX_POINT('',#15);\n\
#17=CARTESIAN_POINT('',(-10.5,0.,0.));\n\
#18=VERTEX_POINT('',#17);\n\
#19=EDGE_CURVE('',#16,#16,#8,.T.);\n\
#20=EDGE_CURVE('',#18,#18,#13,.T.);\n\
#21=EDGE_CURVE('',#16,#18,#14,.T.);\n\
#22=ORIENTED_EDGE('',*,*,#19,.T.);\n\
#23=ORIENTED_EDGE('',*,*,#21,.T.);\n\
#24=ORIENTED_EDGE('',*,*,#20,.T.);\n\
#25=ORIENTED_EDGE('',*,*,#21,.F.);\n\
#26=EDGE_LOOP('',(#22,#23,#24,#25));\n\
#27=FACE_OUTER_BOUND('',#26,.T.);\n\
#28=TOROIDAL_SURFACE('',#4,8.,2.5);\n\
#29=ADVANCED_FACE('',(#27),#28,.T.);\n"
		.to_string();
	// (b) A surface of revolution face.
	let rev = format!("{placement}#24=SURFACE_OF_REVOLUTION('',#5,#4);\n#25=ADVANCED_FACE('',(#23),#24,.T.);\n");
	// (c) A parabola edge curve.
	let parabola = "\
#1=CARTESIAN_POINT('',(0.,0.,0.));\n\
#2=DIRECTION('',(0.,0.,1.));\n\
#3=DIRECTION('',(1.,0.,0.));\n\
#4=AXIS2_PLACEMENT_3D('',#1,#2,#3);\n\
#10=CARTESIAN_POINT('',(5.,0.,0.));\n\
#11=VERTEX_POINT('',#10);\n\
#12=CARTESIAN_POINT('',(-5.,0.,0.));\n\
#13=VERTEX_POINT('',#12);\n\
#14=PARABOLA('',#4,1.);\n\
#15=EDGE_CURVE('',#11,#13,#14,.T.);\n\
#21=ORIENTED_EDGE('',*,*,#15,.T.);\n\
#22=EDGE_LOOP('',(#21));\n\
#23=FACE_OUTER_BOUND('',#22,.T.);\n\
#24=PLANE('',#4);\n\
#25=ADVANCED_FACE('',(#23),#24,.T.);\n"
		.to_string();
	let results: Vec<bool> = [&rev, &parabola]
		.iter()
		.map(|s| matches!(import_step(s), Err(StepError::Unsupported(_))))
		.collect();
	assert_eq!(
		results,
		vec![true, true],
		"SURFACE_OF_REVOLUTION and PARABOLA edges must each be a loud StepError::Unsupported"
	);
	// (a) used to be a documented refusal; since the parameter-patch fallback it
	// imports as facets ON the exact torus: an open shell (the wall alone) whose
	// area is half the torus' 4π²Rr within 1%, every vertex on the surface.
	let wall = import_step(&half_torus).expect("a half-torus wall must import through the parameter-patch fallback");
	let v = validate(&wall);
	let (r_major, r_minor) = (8.0_f64, 2.5_f64);
	let max_off = (0..wall.vertex_count())
		.map(|i| {
			let p = wall.position(kernel_brep::VertexId(i as u32));
			let rho = (p.x * p.x + p.y * p.y).sqrt();
			(((rho - r_major).powi(2) + p.z * p.z).sqrt() - r_minor).abs()
		})
		.fold(0.0_f64, f64::max);
	let area = kernel_brep::area(&wall);
	let want = 0.5 * 4.0 * std::f64::consts::PI * std::f64::consts::PI * r_major * r_minor;
	assert!(
		!v.closed && wall.face_count() > 200 && max_off < 1e-9 && (area - want).abs() / want < 0.01,
		"half-torus wall: closed={} faces={} max torus deviation={max_off:.2e} area={area:.3} (want ≈{want:.3})",
		v.closed,
		wall.face_count()
	);
}

#[test]
fn imports_a_seamless_pole_spanning_dome_on_the_exact_sphere() {
	// A bare spherical cap: ONE full-circle rim on a SPHERICAL_SURFACE with NO seam and
	// no pole vertex (some exporters skip the seam excursion for caps). The region side
	// comes from the loop's circulation; the cap is resampled as rings + a pole fan ON
	// the exact sphere. (This used to be a documented Unsupported case; the closed
	// rim+seam hemisphere lives in the corpus as fc_hemisphere.)
	let step = "\
#1=CARTESIAN_POINT('',(0.,0.,0.));\n\
#2=DIRECTION('',(0.,0.,1.));\n\
#3=DIRECTION('',(1.,0.,0.));\n\
#4=AXIS2_PLACEMENT_3D('',#1,#2,#3);\n\
#5=CIRCLE('',#4,5.);\n\
#10=CARTESIAN_POINT('',(5.,0.,0.));\n\
#11=VERTEX_POINT('',#10);\n\
#14=EDGE_CURVE('',#11,#11,#5,.T.);\n\
#21=ORIENTED_EDGE('',*,*,#14,.T.);\n\
#22=EDGE_LOOP('',(#21));\n\
#23=FACE_OUTER_BOUND('',#22,.T.);\n\
#24=SPHERICAL_SURFACE('',#4,5.);\n\
#25=ADVANCED_FACE('',(#23),#24,.T.);\n";
	let dome = import_step(step).expect("a seamless pole-spanning dome must import");
	// Open shell (just the dome — the rim pairs with nothing), every vertex ON the
	// exact sphere, and the area within 1% of the true hemisphere 2πr².
	let v = validate(&dome);
	let max_off = (0..dome.vertex_count())
		.map(|i| (dome.position(kernel_brep::VertexId(i as u32)).length() - 5.0).abs())
		.fold(0.0_f64, f64::max);
	let area = kernel_brep::area(&dome);
	let want = 2.0 * std::f64::consts::PI * 25.0;
	assert!(
		!v.closed && dome.face_count() > 500 && max_off < 1e-9 && (area - want).abs() / want < 0.01,
		"seamless dome: closed={} faces={} max sphere deviation={max_off:.2e} area={area:.3} (want ≈{want:.3})",
		v.closed,
		dome.face_count()
	);
}

#[test]
fn malformed_input_returns_a_descriptive_error_not_a_panic() {
	// AI-usable contract: bad input fails with a typed `Result`, never a panic.
	let cases = [
		"not a step file at all",
		"#1 = CARTESIAN_POINT('',(1.0,2.0));", // 2 coords, and no faces
		"#1 = ADVANCED_FACE('',(#2),#3,.T.);", // dangling references
	];
	let results: Vec<bool> = cases.iter().map(|c| import_step(c).is_err()).collect();
	assert_eq!(results, vec![true, true, true], "every malformed input must return Err");
	// And the error is a real, matchable variant.
	assert!(matches!(import_step("").unwrap_err(), StepError::Topology(_)));
}

#[test]
fn imports_a_trimmed_bspline_face_with_a_hole_on_the_exact_patch() {
	// A bare planar B-spline patch (degree 1×1, z = 5) trimmed by an outer square and
	// a FACE_BOUND hole — the parameter-space hole-bridging + interior-refinement path.
	// The fragment imports as an open shell whose area is EXACTLY outer − hole (the
	// flat patch makes every chord facet exact), with the hole loop intact.
	let step = "\
#1=CARTESIAN_POINT('',(0.,0.,5.));\n\
#2=CARTESIAN_POINT('',(10.,0.,5.));\n\
#3=CARTESIAN_POINT('',(0.,10.,5.));\n\
#4=CARTESIAN_POINT('',(10.,10.,5.));\n\
#5=B_SPLINE_SURFACE_WITH_KNOTS('',1,1,((#1,#3),(#2,#4)),.UNSPECIFIED.,.F.,.F.,.F.,(2,2),(2,2),(0.,1.),(0.,1.),.UNSPECIFIED.);\n\
#10=VERTEX_POINT('',#1);\n\
#11=VERTEX_POINT('',#2);\n\
#12=VERTEX_POINT('',#4);\n\
#13=VERTEX_POINT('',#3);\n\
#14=EDGE_CURVE('',#10,#11,$,.T.);\n\
#15=EDGE_CURVE('',#11,#12,$,.T.);\n\
#16=EDGE_CURVE('',#12,#13,$,.T.);\n\
#17=EDGE_CURVE('',#13,#10,$,.T.);\n\
#18=ORIENTED_EDGE('',*,*,#14,.T.);\n\
#19=ORIENTED_EDGE('',*,*,#15,.T.);\n\
#20=ORIENTED_EDGE('',*,*,#16,.T.);\n\
#21=ORIENTED_EDGE('',*,*,#17,.T.);\n\
#22=EDGE_LOOP('',(#18,#19,#20,#21));\n\
#23=FACE_OUTER_BOUND('',#22,.T.);\n\
#30=CARTESIAN_POINT('',(3.,3.,5.));\n\
#31=CARTESIAN_POINT('',(7.,3.,5.));\n\
#32=CARTESIAN_POINT('',(7.,7.,5.));\n\
#33=CARTESIAN_POINT('',(3.,7.,5.));\n\
#34=VERTEX_POINT('',#30);\n\
#35=VERTEX_POINT('',#31);\n\
#36=VERTEX_POINT('',#32);\n\
#37=VERTEX_POINT('',#33);\n\
#38=EDGE_CURVE('',#34,#37,$,.T.);\n\
#39=EDGE_CURVE('',#37,#36,$,.T.);\n\
#40=EDGE_CURVE('',#36,#35,$,.T.);\n\
#41=EDGE_CURVE('',#35,#34,$,.T.);\n\
#42=ORIENTED_EDGE('',*,*,#38,.T.);\n\
#43=ORIENTED_EDGE('',*,*,#39,.T.);\n\
#44=ORIENTED_EDGE('',*,*,#40,.T.);\n\
#45=ORIENTED_EDGE('',*,*,#41,.T.);\n\
#46=EDGE_LOOP('',(#42,#43,#44,#45));\n\
#47=FACE_BOUND('',#46,.T.);\n\
#48=ADVANCED_FACE('',(#23,#47),#5,.T.);\n";
	let shell = import_step(step).expect("a trimmed B-spline face with a hole must import");
	let v = validate(&shell);
	let area = kernel_brep::area(&shell);
	assert!(
		!v.closed && (area - 84.0).abs() < 1e-9,
		"holed patch fragment: closed={} area={area:.6} (want exactly 100 − 16 = 84)",
		v.closed
	);
}

#[test]
fn off_patch_trim_vertices_and_seam_crossing_loops_stay_loud() {
	// (a) A trim vertex 1 unit OFF its flat patch: the Newton projection cannot land
	// within tolerance and the face must refuse, not snap the vertex onto the patch.
	let off_patch = "\
#1=CARTESIAN_POINT('',(0.,0.,0.));\n\
#2=CARTESIAN_POINT('',(10.,0.,0.));\n\
#3=CARTESIAN_POINT('',(0.,10.,0.));\n\
#4=CARTESIAN_POINT('',(10.,10.,0.));\n\
#5=B_SPLINE_SURFACE_WITH_KNOTS('',1,1,((#1,#3),(#2,#4)),.UNSPECIFIED.,.F.,.F.,.F.,(2,2),(2,2),(0.,1.),(0.,1.),.UNSPECIFIED.);\n\
#6=CARTESIAN_POINT('',(5.,5.,1.));\n\
#10=VERTEX_POINT('',#1);\n\
#11=VERTEX_POINT('',#2);\n\
#12=VERTEX_POINT('',#6);\n\
#14=EDGE_CURVE('',#10,#11,$,.T.);\n\
#15=EDGE_CURVE('',#11,#12,$,.T.);\n\
#16=EDGE_CURVE('',#12,#10,$,.T.);\n\
#18=ORIENTED_EDGE('',*,*,#14,.T.);\n\
#19=ORIENTED_EDGE('',*,*,#15,.T.);\n\
#20=ORIENTED_EDGE('',*,*,#16,.T.);\n\
#22=EDGE_LOOP('',(#18,#19,#20));\n\
#23=FACE_OUTER_BOUND('',#22,.T.);\n\
#48=ADVANCED_FACE('',(#23),#5,.T.);\n";
	// (b) A SINGLE trim loop winding once around a closed patch (a degree-1
	// triangular tube whose first and last control rows coincide, bounded by one
	// triangle ring at v=0): one winding rim bounds no parameter region — a band
	// needs BOTH rims, and seam-crossing disk loops unwrap instead (see
	// imports_an_unseamed_two_rim_closed_nurbs_tube and the fc_nurbs_* corpus
	// parts) — so the importer refuses loudly, reporting the loop windings.
	let seam_crossing = "\
#1=CARTESIAN_POINT('',(1.,0.,0.));\n\
#2=CARTESIAN_POINT('',(-0.5,0.866025403784439,0.));\n\
#3=CARTESIAN_POINT('',(-0.5,-0.866025403784439,0.));\n\
#4=CARTESIAN_POINT('',(1.,0.,1.));\n\
#5=CARTESIAN_POINT('',(-0.5,0.866025403784439,1.));\n\
#6=CARTESIAN_POINT('',(-0.5,-0.866025403784439,1.));\n\
#7=B_SPLINE_SURFACE_WITH_KNOTS('',1,1,((#1,#4),(#2,#5),(#3,#6),(#1,#4)),.UNSPECIFIED.,.F.,.F.,.F.,(2,1,1,2),(2,2),(0.,1.,2.,3.),(0.,1.),.UNSPECIFIED.);\n\
#10=VERTEX_POINT('',#1);\n\
#11=VERTEX_POINT('',#2);\n\
#12=VERTEX_POINT('',#3);\n\
#14=EDGE_CURVE('',#10,#11,$,.T.);\n\
#15=EDGE_CURVE('',#11,#12,$,.T.);\n\
#16=EDGE_CURVE('',#12,#10,$,.T.);\n\
#18=ORIENTED_EDGE('',*,*,#14,.T.);\n\
#19=ORIENTED_EDGE('',*,*,#15,.T.);\n\
#20=ORIENTED_EDGE('',*,*,#16,.T.);\n\
#22=EDGE_LOOP('',(#18,#19,#20));\n\
#23=FACE_OUTER_BOUND('',#22,.T.);\n\
#48=ADVANCED_FACE('',(#23),#7,.T.);\n";
	let results: Vec<bool> = [off_patch, seam_crossing]
		.iter()
		.map(|s| matches!(import_step(s), Err(StepError::Unsupported(_))))
		.collect();
	assert_eq!(
		results,
		vec![true, true],
		"an off-patch trim vertex and a seam-crossing loop on a closed patch must each be a loud StepError::Unsupported"
	);
}

#[test]
fn imports_an_assembly_with_nauo_placements_from_disk() {
	// FreeCAD-style assembly fixture: a plate and TWO instances of one pin product,
	// each NAUO placed by an ITEM_DEFINED_TRANSFORMATION (one upright on the plate,
	// one rotated so the pin's +Z lies along assembly +X).
	let parts = import_step_assembly(include_str!("fixtures/fc_asm_pin_plate.step"))
		.expect("the NAUO assembly fixture must import");
	let summary: Vec<(String, f64, DVec3, DVec3)> = parts
		.iter()
		.map(|(name, solid, t)| {
			let vol = volume(solid).abs();
			(name.clone(), vol, t.transform_point3(DVec3::ZERO), t.transform_point3(DVec3::new(0.0, 0.0, 12.0)))
		})
		.collect();
	// Both pins are the SAME reconstructed geometry (bit-equal volumes) and sit
	// within the documented 48-gon faceting of the true cylinder π·3²·12.
	let pin_vol = std::f64::consts::PI * 9.0 * 12.0;
	let ok = summary.len() == 3
		&& summary[0].0 == "plate"
		&& (summary[0].1 - 4800.0).abs() < 1e-9
		&& summary[0].2.distance(DVec3::ZERO) < 1e-12
		&& summary[1].0 == "pin"
		&& (summary[1].1 - pin_vol).abs() / pin_vol < 0.005
		&& summary[1].2.distance(DVec3::new(10.0, 10.0, 6.0)) < 1e-12
		&& summary[1].3.distance(DVec3::new(10.0, 10.0, 18.0)) < 1e-12
		&& summary[2].0 == "pin"
		&& (summary[2].1 - summary[1].1).abs() < 1e-9
		&& summary[2].2.distance(DVec3::new(30.0, 10.0, 6.0)) < 1e-12
		&& summary[2].3.distance(DVec3::new(42.0, 10.0, 6.0)) < 1e-12;
	assert!(
		ok,
		"assembly must flatten to [plate@origin, pin@(10,10,6) upright, pin@(30,10,6) along +X] \
		 with shared pin geometry (≈{pin_vol:.4} less the 48-gon faceting); got {summary:?}"
	);
}

#[test]
fn imports_mapped_item_instances_with_representation_map_frames() {
	// Lightweight MAPPED_ITEM instancing (no NAUO): one tetrahedron representation
	// mapped twice into the root SHAPE_REPRESENTATION. The placement is
	// target ∘ origin⁻¹ — the map's origin frame sits at (1,0,0), so targets at
	// (5,0,0) and (0,2,0) place the part at net translations (4,0,0) and (−1,2,0).
	let step = "\
#1=PRODUCT('mapped_asm','mapped_asm','',());\n\
#2=PRODUCT_DEFINITION_FORMATION('','',#1);\n\
#3=PRODUCT_DEFINITION('design','',#2,$);\n\
#4=PRODUCT_DEFINITION_SHAPE('','',#3);\n\
#10=CARTESIAN_POINT('',(0.,0.,0.));\n\
#11=CARTESIAN_POINT('',(2.,0.,0.));\n\
#12=CARTESIAN_POINT('',(0.,2.,0.));\n\
#13=CARTESIAN_POINT('',(0.,0.,2.));\n\
#14=VERTEX_POINT('',#10);\n\
#15=VERTEX_POINT('',#11);\n\
#16=VERTEX_POINT('',#12);\n\
#17=VERTEX_POINT('',#13);\n\
#20=EDGE_CURVE('',#14,#15,$,.T.);\n\
#21=EDGE_CURVE('',#14,#16,$,.T.);\n\
#22=EDGE_CURVE('',#14,#17,$,.T.);\n\
#23=EDGE_CURVE('',#15,#16,$,.T.);\n\
#24=EDGE_CURVE('',#16,#17,$,.T.);\n\
#25=EDGE_CURVE('',#17,#15,$,.T.);\n\
#30=ORIENTED_EDGE('',*,*,#21,.T.);\n\
#31=ORIENTED_EDGE('',*,*,#23,.F.);\n\
#32=ORIENTED_EDGE('',*,*,#20,.F.);\n\
#33=EDGE_LOOP('',(#30,#31,#32));\n\
#34=FACE_OUTER_BOUND('',#33,.T.);\n\
#35=CARTESIAN_POINT('',(0.,0.,-1.));\n\
#36=DIRECTION('',(0.,0.,-1.));\n\
#37=DIRECTION('',(1.,0.,0.));\n\
#38=AXIS2_PLACEMENT_3D('',#10,#36,#37);\n\
#39=PLANE('',#38);\n\
#40=ADVANCED_FACE('',(#34),#39,.T.);\n\
#41=ORIENTED_EDGE('',*,*,#20,.T.);\n\
#42=ORIENTED_EDGE('',*,*,#25,.F.);\n\
#43=ORIENTED_EDGE('',*,*,#22,.F.);\n\
#44=EDGE_LOOP('',(#41,#42,#43));\n\
#45=FACE_OUTER_BOUND('',#44,.T.);\n\
#46=DIRECTION('',(0.,-1.,0.));\n\
#47=AXIS2_PLACEMENT_3D('',#10,#46,#37);\n\
#48=PLANE('',#47);\n\
#49=ADVANCED_FACE('',(#45),#48,.T.);\n\
#50=ORIENTED_EDGE('',*,*,#22,.T.);\n\
#51=ORIENTED_EDGE('',*,*,#24,.F.);\n\
#52=ORIENTED_EDGE('',*,*,#21,.F.);\n\
#53=EDGE_LOOP('',(#50,#51,#52));\n\
#54=FACE_OUTER_BOUND('',#53,.T.);\n\
#55=DIRECTION('',(-1.,0.,0.));\n\
#56=DIRECTION('',(0.,1.,0.));\n\
#57=AXIS2_PLACEMENT_3D('',#10,#55,#56);\n\
#58=PLANE('',#57);\n\
#59=ADVANCED_FACE('',(#54),#58,.T.);\n\
#60=ORIENTED_EDGE('',*,*,#23,.T.);\n\
#61=ORIENTED_EDGE('',*,*,#24,.T.);\n\
#62=ORIENTED_EDGE('',*,*,#25,.T.);\n\
#63=EDGE_LOOP('',(#60,#61,#62));\n\
#64=FACE_OUTER_BOUND('',#63,.T.);\n\
#65=DIRECTION('',(0.577350269189626,0.577350269189626,0.577350269189626));\n\
#66=AXIS2_PLACEMENT_3D('',#11,#65,#56);\n\
#67=PLANE('',#66);\n\
#68=ADVANCED_FACE('',(#64),#67,.T.);\n\
#70=CLOSED_SHELL('',(#40,#49,#59,#68));\n\
#71=MANIFOLD_SOLID_BREP('tet',#70);\n\
#72=DIRECTION('',(0.,0.,1.));\n\
#73=AXIS2_PLACEMENT_3D('',#10,#72,#37);\n\
#74=ADVANCED_BREP_SHAPE_REPRESENTATION('tet_rep',(#73,#71),$);\n\
#80=CARTESIAN_POINT('',(1.,0.,0.));\n\
#81=AXIS2_PLACEMENT_3D('',#80,#72,#37);\n\
#82=REPRESENTATION_MAP(#81,#74);\n\
#83=CARTESIAN_POINT('',(5.,0.,0.));\n\
#84=AXIS2_PLACEMENT_3D('',#83,#72,#37);\n\
#85=MAPPED_ITEM('',#82,#84);\n\
#86=CARTESIAN_POINT('',(0.,2.,0.));\n\
#87=AXIS2_PLACEMENT_3D('',#86,#72,#37);\n\
#88=MAPPED_ITEM('',#82,#87);\n\
#89=SHAPE_REPRESENTATION('root',(#73,#85,#88),$);\n\
#90=SHAPE_DEFINITION_REPRESENTATION(#4,#89);\n";
	let parts = import_step_assembly(step).expect("a MAPPED_ITEM file must import as instances");
	let summary: Vec<(String, f64, DVec3)> = parts
		.iter()
		.map(|(name, solid, t)| (name.clone(), volume(solid).abs(), t.transform_point3(DVec3::ZERO)))
		.collect();
	let ok = summary.len() == 2
		&& summary.iter().all(|(n, v, _)| n == "tet_rep" && (v - 4.0 / 3.0).abs() < 1e-9)
		&& summary[0].2.distance(DVec3::new(4.0, 0.0, 0.0)) < 1e-12
		&& summary[1].2.distance(DVec3::new(-1.0, 2.0, 0.0)) < 1e-12;
	assert!(
		ok,
		"two tet instances at net translations (4,0,0) and (−1,2,0), volume 4/3 each; got {summary:?}"
	);
}

#[test]
fn single_part_files_degrade_to_one_identity_assembly_component() {
	// The documented total-function contract: a plain part file (no NAUO, no
	// MAPPED_ITEM) imports as exactly one component, named after its product, at the
	// identity placement, with the same solid `import_step` reconstructs.
	let text = include_str!("fixtures/fc_sphere_ball.step");
	let parts = import_step_assembly(text).expect("a single-part file must import as one component");
	let whole = volume(&import_step(text).expect("the same file imports as a solid")).abs();
	let ok = parts.len() == 1
		&& parts[0].0 == "fc_sphere_ball"
		&& (volume(&parts[0].1).abs() - whole).abs() < 1e-9
		&& parts[0].2.transform_point3(DVec3::new(1.0, 2.0, 3.0)).distance(DVec3::new(1.0, 2.0, 3.0)) < 1e-12;
	assert!(
		ok,
		"single-part fallback: want [(\"fc_sphere_ball\", <whole-file solid {whole:.4}>, identity)]; got {} component(s): {:?}",
		parts.len(),
		parts.iter().map(|(n, s, t)| (n.clone(), volume(s).abs(), t.translation)).collect::<Vec<_>>()
	);
}

#[test]
fn imports_an_unseamed_two_rim_closed_nurbs_tube() {
	// An UNTRIMMED closed B-spline patch: the tube wall face is bounded ONLY by its
	// two full-circle rational B-spline rims (no seam edge at all — each rim loop
	// winds the closed direction once, in opposite senses). The importer bridges the
	// two rims with a synthetic seam in the universal cover (`bridge_band_rings`),
	// whose two copies weld back in 3-D. With planar caps the solid closes. The
	// seam-traversed-twice variant lives in the corpus as fc_nurbs_tube.
	let (r, h) = (4.0, 6.0);
	let w = 0.5_f64.sqrt();
	let ring = [
		(r, 0.0), (r, r), (0.0, r), (-r, r), (-r, 0.0), (-r, -r), (0.0, -r), (r, -r), (r, 0.0),
	];
	let mut s = String::new();
	for (k, (x, y)) in ring.iter().enumerate() {
		s += &format!("#{}=CARTESIAN_POINT('',({x:?},{y:?},0.));\n", 10 + k);
		s += &format!("#{}=CARTESIAN_POINT('',({x:?},{y:?},{h:?}));\n", 20 + k);
	}
	let wts = (0..9).map(|k| if k % 2 == 0 { "1.".into() } else { format!("{w:?}") }).collect::<Vec<_>>().join(",");
	let bot = (0..9).map(|k| format!("#{}", 10 + k)).collect::<Vec<_>>().join(",");
	let top = (0..9).map(|k| format!("#{}", 20 + k)).collect::<Vec<_>>().join(",");
	let grid = (0..9).map(|k| format!("(#{},#{})", 10 + k, 20 + k)).collect::<Vec<_>>().join(",");
	let wgrid = (0..9)
		.map(|k| if k % 2 == 0 { "(1.,1.)".into() } else { format!("({w:?},{w:?})") })
		.collect::<Vec<_>>()
		.join(",");
	let knots = "(3,2,2,2,3),(0.,0.25,0.5,0.75,1.)";
	s += "#30=VERTEX_POINT('',#10);\n#31=VERTEX_POINT('',#20);\n";
	s += &format!("#36=( BOUNDED_CURVE() B_SPLINE_CURVE(2,({bot}),.UNSPECIFIED.,.F.,.F.) B_SPLINE_CURVE_WITH_KNOTS({knots},.UNSPECIFIED.) CURVE() GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE(({wts})) REPRESENTATION_ITEM('') );\n");
	s += "#37=EDGE_CURVE('',#30,#30,#36,.T.);\n";
	s += &format!("#38=( BOUNDED_CURVE() B_SPLINE_CURVE(2,({top}),.UNSPECIFIED.,.F.,.F.) B_SPLINE_CURVE_WITH_KNOTS({knots},.UNSPECIFIED.) CURVE() GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE(({wts})) REPRESENTATION_ITEM('') );\n");
	s += "#39=EDGE_CURVE('',#31,#31,#38,.T.);\n";
	s += &format!("#40=( BOUNDED_SURFACE() B_SPLINE_SURFACE(2,1,({grid}),.UNSPECIFIED.,.F.,.F.,.F.) B_SPLINE_SURFACE_WITH_KNOTS({knots},(2,2),(0.,1.),.UNSPECIFIED.) GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_SURFACE(({wgrid})) REPRESENTATION_ITEM('') SURFACE() );\n");
	// Wall: two rim bounds, NO seam edge. Bottom rim forward (+θ, the outer bound),
	// top rim reversed — opposite windings, as a band's rims must be.
	s += "#41=ORIENTED_EDGE('',*,*,#37,.T.);\n#42=EDGE_LOOP('',(#41));\n#43=FACE_OUTER_BOUND('',#42,.T.);\n";
	s += "#44=ORIENTED_EDGE('',*,*,#39,.F.);\n#45=EDGE_LOOP('',(#44));\n#46=FACE_BOUND('',#45,.T.);\n";
	s += "#47=ADVANCED_FACE('',(#43,#46),#40,.T.);\n";
	// Caps.
	s += "#50=DIRECTION('',(0.,0.,1.));\n#51=DIRECTION('',(1.,0.,0.));\n#52=CARTESIAN_POINT('',(0.,0.,0.));\n";
	s += "#53=AXIS2_PLACEMENT_3D('',#52,#50,#51);\n#54=PLANE('',#53);\n";
	s += "#55=ORIENTED_EDGE('',*,*,#37,.F.);\n#56=EDGE_LOOP('',(#55));\n#57=FACE_OUTER_BOUND('',#56,.T.);\n";
	s += "#58=ADVANCED_FACE('',(#57),#54,.T.);\n";
	s += &format!("#60=CARTESIAN_POINT('',(0.,0.,{h:?}));\n");
	s += "#61=AXIS2_PLACEMENT_3D('',#60,#50,#51);\n#62=PLANE('',#61);\n";
	s += "#63=ORIENTED_EDGE('',*,*,#39,.T.);\n#64=EDGE_LOOP('',(#63));\n#65=FACE_OUTER_BOUND('',#64,.T.);\n";
	s += "#66=ADVANCED_FACE('',(#65),#62,.T.);\n";
	s += "#70=CLOSED_SHELL('',(#47,#58,#66));\n#71=MANIFOLD_SOLID_BREP('tube2',#70);\n";
	let solid = import_step(&s).expect("an unseamed two-rim closed NURBS tube must import");
	let v = validate(&solid);
	let vol = volume(&solid).abs();
	let want = std::f64::consts::PI * r * r * h;
	let rel = (vol - want).abs() / want;
	let watertight = tessellate_default(&solid).is_watertight();
	assert!(
		v.closed && v.manifold && v.genus == 0 && rel < 0.005 && watertight,
		"two-rim closed NURBS tube: {v:?}, vol {vol:.4} vs exact {want:.4} (rel {rel:.5}), watertight {watertight}"
	);
}

#[test]
fn freeform_round_trip_reexports_true_bspline_surfaces() {
	use kernel_brep::{export_step_freeform, import_step_freeform};
	// The writing half of NURBS interchange: import a trimmed-NURBS part WITH its
	// freeform sidecar, re-export it through export_step_freeform (which writes true
	// B_SPLINE_SURFACE_WITH_KNOTS faces and skips the patches' chord facets), and
	// re-import. The volume must agree within ±0.5% — the patch geometry survived as
	// NURBS, not as facet soup. Exercised on the open-patch pad (3 patches: top + two
	// ruled walls) and on the CLOSED-patch tube (a slit ring crossing the seam).
	let cases = [
		("fc_freeform_pad", include_str!("fixtures/fc_freeform_pad.step"), 3, false),
		("fc_nurbs_tube", include_str!("fixtures/fc_nurbs_tube.step"), 1, true),
	];
	let mut failures = Vec::new();
	for (name, text, n_patches, rational) in cases {
		let (solid, patches) = match import_step_freeform(text) {
			Ok(sp) => sp,
			Err(e) => {
				failures.push(format!("{name}: first import failed: {e}"));
				continue;
			}
		};
		let step2 = export_step_freeform(&solid, &patches, name);
		let back = match import_step(&step2) {
			Ok(b) => b,
			Err(e) => {
				failures.push(format!("{name}: re-import failed: {e}"));
				continue;
			}
		};
		let (v0, v1) = (volume(&solid).abs(), volume(&back).abs());
		let rel = (v1 - v0).abs() / v0;
		let v = validate(&back);
		let has_nurbs = step2.contains("B_SPLINE_SURFACE_WITH_KNOTS");
		let has_rational = step2.contains("RATIONAL_B_SPLINE_SURFACE");
		if !(patches.len() == n_patches
			&& has_nurbs
			&& has_rational == rational
			&& v.closed && v.manifold
			&& rel < 0.005
			&& tessellate_default(&back).is_watertight())
		{
			failures.push(format!(
				"{name}: patches {} (want {n_patches}), B-spline surface written {has_nurbs} (rational {has_rational}, want {rational}), re-import {v:?}, volume {v1:.4} vs {v0:.4} (rel {rel:.5})",
				patches.len()
			));
		}
	}
	assert!(failures.is_empty(), "freeform STEP round-trips must keep NURBS surfaces:\n{}", failures.join("\n"));
}

#[test]
fn assembly_export_round_trips_through_nauo_tree() {
	use kernel_brep::export_step_assembly;
	use kernel_brep::math::{DAffine3, DQuat};
	// Export an assembly (two instances of one pin + a plate), then flatten it back
	// with import_step_assembly: same component names, volumes, and placements (probe
	// a point through each instance transform). The pin instances share one product.
	let pin = cylinder(DVec3::ZERO, DVec3::Z, 1.5, 8.0, 16);
	let plate = cuboid(DVec3::ZERO, DVec3::new(40.0, 20.0, 5.0));
	let lay_flat = DAffine3::from_rotation_translation(DQuat::from_rotation_y(std::f64::consts::FRAC_PI_2), DVec3::new(30.0, 10.0, 5.0));
	let parts = vec![
		("plate".to_string(), plate.clone(), DAffine3::IDENTITY),
		("pin".to_string(), pin.clone(), DAffine3::from_translation(DVec3::new(10.0, 10.0, 5.0))),
		("pin".to_string(), pin.clone(), lay_flat),
	];
	let step = export_step_assembly(&parts, "pin_plate").expect("rigid placements must export");
	let back = import_step_assembly(&step).expect("the exported NAUO tree must re-import");
	let probe = DVec3::new(0.0, 0.0, 8.0); // a pin's top-axis point
	let summary: Vec<(String, f64, DVec3)> = back
		.iter()
		.map(|(n, s, t)| (n.clone(), volume(s).abs(), t.transform_point3(probe)))
		.collect();
	let expect: Vec<(String, f64, DVec3)> = parts
		.iter()
		.map(|(n, s, t)| (n.clone(), volume(s).abs(), t.transform_point3(probe)))
		.collect();
	let ok = summary.len() == 3
		&& summary.iter().zip(&expect).all(|((an, av, ap), (bn, bv, bp))| {
			an == bn && (av - bv).abs() < 1e-9 * bv.max(1.0) && ap.distance(*bp) < 1e-9
		});
	assert!(ok, "assembly export must round-trip names/volumes/placements:\nexported {expect:?}\nre-imported {summary:?}");

	// A mirrored placement cannot be encoded in an AXIS2_PLACEMENT_3D: loud refusal.
	let mirrored = DAffine3::from_scale(DVec3::new(-1.0, 1.0, 1.0));
	let bad = vec![("pin".to_string(), pin, mirrored)];
	assert!(
		matches!(export_step_assembly(&bad, "bad"), Err(StepError::Unsupported(_))),
		"a mirrored placement must be a loud StepError::Unsupported"
	);
}
