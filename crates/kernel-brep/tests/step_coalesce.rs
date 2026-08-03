//! STEP export face coalescing (the export side of FRICTION #20): facets that
//! share one analytic surface must merge into a single properly-bounded
//! ADVANCED_FACE (full cylinder AND cone-frustum wraps split into two
//! half-bands so no exported face is periodic; a cone region touching its apex
//! is not a two-rim band and falls back to facets). Motivation: the
//! faceted-exact export of the robot-arm assembly was 100 MB / ~2M entities of
//! chord-bounded curved faces, which stalled Onshape's translator; merged faces
//! have on-surface boundaries (rim arcs + rulings) and 10-50x fewer entities.
//! On top of merging, identical geometry records (CIRCLEs, surfaces and their
//! placements) are hash-consed within one solid's emission. Fallback for
//! anything unclean is the old faceted path — asserted here via a
//! boolean-scarred part and the apex-full cone.

use kernel_brep::math::DVec3;
use kernel_brep::{
	cone, cuboid, cylinder, difference, export_step, import_step, tessellate_default, union, validate, volume,
};

fn count(hay: &str, needle: &str) -> usize {
	hay.matches(needle).count()
}

/// Total `#N = …;` entity records in the DATA section.
fn entity_count(step: &str) -> usize {
	step.lines().filter(|l| l.starts_with('#')).count()
}

/// Ids (without `#`) of every entity whose record contains `name`.
fn ids_of(step: &str, name: &str) -> Vec<String> {
	step.lines()
		.filter(|l| l.contains(name))
		.filter_map(|l| l.split(" =").next())
		.map(|id| id.trim_start_matches('#').to_string())
		.collect()
}

/// How many ADVANCED_FACEs reference the surface entity `surf_id` —
/// `ADVANCED_FACE('',(…),#surf,.T.)` puts the surface ref right before the flag.
fn faces_on_surface(step: &str, surf_id: &str) -> usize {
	step.lines()
		.filter(|l| l.contains("= ADVANCED_FACE(") && (l.contains(&format!("),#{surf_id},.T.)")) || l.contains(&format!("),#{surf_id},.F.)"))))
		.count()
}

#[test]
fn cylinder_exports_as_two_half_bands_and_round_trips() {
	let cyl = cylinder(DVec3::ZERO, DVec3::Z, 10.0, 20.0, 32);
	let step = export_step(&cyl, "cyl");
	let faces = count(&step, "ADVANCED_FACE");
	let cyl_surfs = count(&step, "CYLINDRICAL_SURFACE");
	// 2 caps + 2 half-bands (was 2 + 32 faceted). Both half-bands reference the
	// SAME hash-consed CYLINDRICAL_SURFACE entity (equal parameters + placement),
	// so exactly ONE such record exists — referenced twice.
	let band_faces = ids_of(&step, "= CYLINDRICAL_SURFACE(").first().map(|id| faces_on_surface(&step, id)).unwrap_or(0);
	let back = import_step(&step).expect("re-import");
	let v0 = volume(&cyl).abs();
	let v1 = volume(&back).abs();
	let dv = (v0 - v1).abs() / v0;
	assert!(
		faces == 4
			&& cyl_surfs == 1
			&& band_faces == 2
			&& validate(&back).is_valid()
			&& tessellate_default(&back).is_watertight()
			&& dv < 0.005,
		"coalesced cylinder: {faces} ADVANCED_FACEs (want 4), {cyl_surfs} CYLINDRICAL_SURFACEs (want 1, deduped) \
		 referenced by {band_faces} half-band faces (want 2), round-trip valid={} wt={} volume Δ {dv:.4}",
		validate(&back).is_valid(),
		tessellate_default(&back).is_watertight(),
	);
}

#[test]
fn cone_frustum_coalesces_to_two_conical_half_bands() {
	// A frustum: a cone primitive with its tip cut off by a box. The lateral
	// facets share one Surface::Cone tag; the base rim carries the primitive's
	// Curve::Circle tag and the CUT rim lands on boolean chords — the exporter
	// re-synthesizes those as true arcs (both endpoints on the tagged cone at
	// one axial height, radius h·tan α). Result: base cap + top cap + exactly 2
	// CONICAL half-band faces, sharing ONE hash-consed CONICAL_SURFACE record.
	// The per-facet fallback of this part (dedup still on) is ~650 entities and
	// the pre-v1 writer emitted more still; assert the merged file stays far
	// below both AND round-trips through our own importer. Measured: 377.
	let full = cone(DVec3::ZERO, DVec3::Z, 12.0, 24.0, 32);
	let frustum = difference(&full, &cuboid(DVec3::new(-20.0, -20.0, 10.0), DVec3::new(20.0, 20.0, 30.0)));
	let step = export_step(&frustum, "frustum");
	let faces = count(&step, "ADVANCED_FACE");
	let cone_ids = ids_of(&step, "= CONICAL_SURFACE(");
	let band_faces = cone_ids.first().map(|id| faces_on_surface(&step, id)).unwrap_or(0);
	let entities = entity_count(&step);
	let back = import_step(&step).expect("frustum re-import");
	let dv = (volume(&frustum).abs() - volume(&back).abs()).abs() / volume(&frustum).abs();
	assert!(
		faces <= 4 && cone_ids.len() == 1 && band_faces == 2 && entities < 500 && dv < 0.005 && validate(&back).is_valid(),
		"coalesced frustum: {faces} ADVANCED_FACEs (want <=4), {} CONICAL_SURFACE records (want 1, deduped) \
		 referenced by {band_faces} half-band faces (want 2), {entities} entities (faceted ~1400, want <500), \
		 round-trip Δ {dv:.4} valid={}",
		cone_ids.len(),
		validate(&back).is_valid(),
	);
}

#[test]
fn apex_full_cone_falls_back_to_facets() {
	// A cone WITH its apex: the lateral region contains the tip vertex, which
	// can never bound a two-rim band — the exporter must refuse to merge it
	// (asserted: the per-facet fallback keeps one ADVANCED_FACE per lateral
	// facet plus the base cap) and the faceted file must still round-trip.
	let solid = cone(DVec3::ZERO, DVec3::Z, 10.0, 20.0, 32);
	let step = export_step(&solid, "apex_cone");
	let faces = count(&step, "ADVANCED_FACE");
	let back = import_step(&step).expect("apex cone re-import");
	let dv = (volume(&solid).abs() - volume(&back).abs()).abs() / volume(&solid).abs();
	assert!(
		faces == 33 && dv < 0.005 && validate(&back).is_valid(),
		"apex-full cone must stay faceted: {faces} ADVANCED_FACEs (want 32 lateral + 1 cap = 33), \
		 round-trip Δ {dv:.4} valid={}",
		validate(&back).is_valid(),
	);
}

#[test]
fn bolt_circle_plate_dedups_rim_circles_and_round_trips() {
	// Entity dedup: a plate with a 4-bore bolt circle has 8 rim circles (2 per
	// bore). Every 16-segment rim used to emit one CIRCLE (plus a private
	// placement) PER ARC — 128 CIRCLEs; hash-consing shares one record per rim,
	// so exactly 8 CIRCLE entities remain, and the file still round-trips.
	let mut plate = cuboid(DVec3::new(0.0, 0.0, 0.0), DVec3::new(60.0, 60.0, 8.0));
	for (cx, cy) in [(15.0, 15.0), (45.0, 15.0), (45.0, 45.0), (15.0, 45.0)] {
		plate = difference(&plate, &cylinder(DVec3::new(cx, cy, -1.0), DVec3::Z, 4.0, 10.0, 16));
	}
	let step = export_step(&plate, "bolt_plate");
	let circles = count(&step, "= CIRCLE(");
	let back = import_step(&step).expect("bolt plate re-import");
	let dv = (volume(&plate).abs() - volume(&back).abs()).abs() / volume(&plate).abs();
	assert!(
		circles == 8 && dv < 0.005 && validate(&back).is_valid(),
		"bolt-circle plate: {circles} CIRCLE entities (want 8 — one per rim, was one per arc = 128), \
		 round-trip Δ {dv:.4} valid={}",
		validate(&back).is_valid(),
	);
}

#[test]
fn drilled_plate_coalesces_bore_and_plane_but_falls_back_where_scarred() {
	// plate with a clean through-bore: the bore wall must coalesce to 2
	// half-bands and the plate faces stay single planar faces.
	let plate = cuboid(DVec3::new(0.0, 0.0, 0.0), DVec3::new(40.0, 30.0, 10.0));
	let bored = difference(&plate, &cylinder(DVec3::new(20.0, 15.0, -1.0), DVec3::Z, 6.0, 12.0, 32));
	let step = export_step(&bored, "bored_plate");
	let faces = count(&step, "ADVANCED_FACE");
	// 6 plate faces + 2 half-bands = 8 (the faceted path would emit 6 + 32).
	let back = import_step(&step).expect("re-import bored");
	let dv = (volume(&bored).abs() - volume(&back).abs()).abs() / volume(&bored).abs();
	assert!(
		faces <= 10 && dv < 0.005 && validate(&back).is_valid(),
		"bored plate: {faces} faces (want <=10), round-trip valid={} volume Δ {dv:.4}",
		validate(&back).is_valid(),
	);

	// a union seam that fragments faces: export must still SUCCEED (coalesce
	// what is clean, facet the rest) and round-trip volume-stable.
	let scarred = union(&bored, &cylinder(DVec3::new(40.0, 15.0, 2.0), DVec3::X, 4.0, 12.0, 24));
	let step2 = export_step(&scarred, "scarred");
	let back2 = import_step(&step2).expect("re-import scarred");
	let dv2 = (volume(&scarred).abs() - volume(&back2).abs()).abs() / volume(&scarred).abs();
	assert!(
		dv2 < 0.01,
		"scarred part must still export/import volume-stable: Δ {dv2:.4}"
	);
}

#[test]
fn cylinder_union_box_mixed_solid_round_trips() {
	// A mixed analytic solid — a cylinder unioned with a box poking out of its
	// side. The boolean scars the wall where the box enters, so the wrap is no
	// longer a clean full barrel there: the exporter must coalesce whatever
	// remains verifiably clean and fall back to faceted chord faces for the rest,
	// and the file must re-import volume-stable either way. (Measured on this
	// geometry: the round-trip volume delta is ~0; 0.5% is asserted, well inside
	// the 2.5% acceptance bar for mixed solids.)
	let mixed = union(
		&cylinder(DVec3::ZERO, DVec3::Z, 10.0, 20.0, 32),
		&cuboid(DVec3::new(5.0, -8.0, 4.0), DVec3::new(30.0, 8.0, 16.0)),
	);
	let step = export_step(&mixed, "mixed");
	let back = import_step(&step).expect("mixed cylinder-union-box must re-import");
	let dv = (volume(&mixed).abs() - volume(&back).abs()).abs() / volume(&mixed).abs();
	assert!(
		dv < 0.005 && validate(&back).is_valid(),
		"cylinder ∪ box round-trip: volume Δ {dv:.5} (bar 0.005), valid={}",
		validate(&back).is_valid()
	);
}
