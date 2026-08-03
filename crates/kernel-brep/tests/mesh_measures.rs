//! `Mesh::radial_extent` must be exact where vertex scanning is not: banded
//! measurements with no vertices in the band (a box's silhouette), interior-
//! foot minima (a face's closest point to a centered axis is mid-face), and
//! axis-piercing minima. `SupportFreeReport` must now say WHERE its steep and
//! bridge areas are.

use kernel_brep::math::DVec3;
use kernel_brep::{cuboid, tessellate_default, union};
use kernel_core::math::Vec3;

#[test]
fn radial_extent_is_exact_on_banded_boxes_and_pierced_caps() {
	// 20×20×40 box centered on the z axis: side walls at half-width 10,
	// corners at 10√2. Mid-height band (10, 30) contains NO mesh vertices —
	// clipping + the parallel-plane interior foot must still find min=10
	// (mid-face, on no vertex or edge) and max=10√2 (clipped corner columns).
	let m = tessellate_default(&cuboid(DVec3::new(-10.0, -10.0, 0.0), DVec3::new(10.0, 10.0, 40.0)));
	let (rmin_band, rmax_band) = m
		.radial_extent(Vec3::ZERO, Vec3::Z, Some((10.0, 30.0)))
		.expect("band intersects the box");
	// Full extent: the caps are PIERCED by the axis → min 0.
	let (rmin_full, _) = m.radial_extent(Vec3::ZERO, Vec3::Z, None).expect("full extent");
	// Empty band → None.
	let empty = m.radial_extent(Vec3::ZERO, Vec3::Z, Some((100.0, 200.0)));
	let d = 10.0 * 2.0_f64.sqrt();
	assert!(
		(rmin_band - 10.0).abs() < 1e-9
			&& (rmax_band - d).abs() < 1e-9
			&& rmin_full < 1e-9
			&& empty.is_none(),
		"radial_extent: band min {rmin_band} (want 10 exactly, interior-foot), band max {rmax_band} (want {d}), \
		 full min {rmin_full} (want 0, pierced caps), empty band {empty:?} (want None)"
	);
}

#[test]
fn support_report_names_where_its_bridges_and_steep_areas_are() {
	// T-shape: a wide slab on a narrow post. The slab underside ring at z=20
	// is a dead-flat ceiling → a bridge patch whose exemplar sits at that
	// height. A cone hung under the slab (apex down at 30° from horizontal
	// flanks) adds honest STEEP area with exemplars on the cone flank.
	let post = cuboid(DVec3::new(-5.0, -5.0, 0.0), DVec3::new(5.0, 5.0, 20.0));
	let slab = cuboid(DVec3::new(-20.0, -20.0, 20.0), DVec3::new(20.0, 20.0, 26.0));
	let t = union(&post, &slab);
	let rep = tessellate_default(&t).support_free_report(Vec3::Z, 45.0, 0.3);
	let bridge_at_slab = rep
		.bridge_patches
		.first()
		.map(|(span, at)| *span > 1.0 && (at.z - 20.0).abs() < 0.5)
		.unwrap_or(false);

	// steep exemplars: a 30°-from-horizontal down-facing cone flank under a cap
	// (Ø16 × 4.6 tall: flank climbs atan(4.6/8) ≈ 30° < the 45° threshold)
	let cone = kernel_brep::cone(DVec3::new(0.0, 0.0, 10.0), -DVec3::Z, 8.0, 4.6, 48);
	let cap = cuboid(DVec3::new(-9.0, -9.0, 9.0), DVec3::new(9.0, 9.0, 12.0));
	let hung = union(&cap, &cone);
	let rep2 = tessellate_default(&hung).support_free_report(Vec3::Z, 45.0, 0.3);
	let steep_named = rep2.steep_area > 10.0
		&& !rep2.steep_exemplars.is_empty()
		&& rep2.steep_exemplars.iter().all(|p| p.z > 1.0 && p.z < 10.5);

	assert!(
		bridge_at_slab && steep_named,
		"report locations: widest bridge patch at slab underside={bridge_at_slab} (patches {:?}); steep cone flank \
		 named={steep_named} (steep_area {:.1}, exemplars {:?})",
		rep.bridge_patches,
		rep2.steep_area,
		rep2.steep_exemplars
	);
}
