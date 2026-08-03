//! A pipe TEE fitting: the UNION of two cylinders at 90deg, whose seam is a
//! curved-curved (cylinder∩cylinder) saddle intersection. Unlike a thread ridge
//! piercing a shank wall (a true self-intersection that must route through the
//! voxel heal), a clean cylinder∩cylinder saddle is stitched EXACTLY by the
//! planar arrangement. This pins that: the solid tee is a watertight genus-0
//! body, and hollowing both runs makes a valid genus-2 fitting — all exact, no
//! self-intersection.

use kernel_brep::math::DVec3;
use kernel_brep::{cylinder, tessellate_default, try_difference, try_union, validate, volume};

#[test]
fn pipe_tee_curved_union_is_exact_and_hollows_to_a_valid_fitting() {
	let run = cylinder(DVec3::new(-30.0, 0.0, 0.0), DVec3::X, 8.0, 60.0, 48);
	let branch = cylinder(DVec3::new(0.0, 0.0, 0.0), DVec3::Z, 8.0, 25.0, 48);

	let tee = try_union(&run, &branch).expect("curved-curved cylinder union (pipe tee) must succeed exactly");
	let tv = validate(&tee);
	let tm = tessellate_default(&tee);
	assert!(
		tv.closed && tv.manifold && tv.genus == 0 && tm.is_watertight() && !tm.has_self_intersection() && (14_000.0..17_000.0).contains(&volume(&tee).abs()),
		"solid pipe tee must be a watertight genus-0 union with no self-intersection: {tv:?} watertight={} self_int={} vol={:.0}",
		tm.is_watertight(),
		tm.has_self_intersection(),
		volume(&tee).abs()
	);

	// Hollow both runs (r=6) -> a real fitting with a T-shaped bore (3 mouths).
	let bore_run = cylinder(DVec3::new(-35.0, 0.0, 0.0), DVec3::X, 6.0, 70.0, 48);
	let bore_branch = cylinder(DVec3::new(0.0, 0.0, -1.0), DVec3::Z, 6.0, 30.0, 48);
	let hollow = try_difference(&tee, &bore_run)
		.and_then(|t| try_difference(&t, &bore_branch))
		.expect("hollowing the tee bores must succeed");
	let hv = validate(&hollow);
	let hm = tessellate_default(&hollow);
	assert!(
		hv.closed && hv.manifold && hv.genus == 2 && hm.is_watertight() && !hm.has_self_intersection() && volume(&hollow).abs() > 0.0,
		"hollow pipe tee must be a watertight genus-2 fitting (T-bore, 3 mouths): {hv:?} watertight={} self_int={} vol={:.0}",
		hm.is_watertight(),
		hm.has_self_intersection(),
		volume(&hollow).abs()
	);
}
