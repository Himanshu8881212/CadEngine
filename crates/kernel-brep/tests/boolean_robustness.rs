//! Robustness boundary of the difference boolean around coplanarity, and that the
//! CHECKED op fails safely rather than degrading silently.
//!
//! - An EXACTLY-coplanar cut (a pocket whose top is flush with a block face) is
//!   handled: a valid solid of the correct volume.
//! - A NEAR-coplanar cut (the cut face a sub-tolerance 1e-7 below an existing
//!   face) leaves a degenerate sliver the planar arrangement cannot close. The
//!   raw boolean would yield a non-closed solid — but `try_difference` REFUSES it
//!   (a clean `BooleanError`, result withheld), so no caller silently ships an
//!   invalid body. Auto-snapping near-coplanar faces to exactly coplanar would
//!   lift the limitation; that is deliberate boolean surgery, not done here. This
//!   test pins the graceful-failure contract and the exact-coplanar capability.

use kernel_brep::math::DVec3;
use kernel_brep::{cuboid, try_difference, validate, volume};

#[test]
fn exact_coplanar_cut_is_valid_and_sub_tolerance_near_coplanar_is_refused() {
	let block = cuboid(DVec3::ZERO, DVec3::new(10.0, 10.0, 10.0));

	// Exactly coplanar: blind pocket flush with the top face (z = 10). Valid,
	// genus 0, vol = 1000 - 6*6*5 = 820.
	let flush = cuboid(DVec3::new(2.0, 2.0, 5.0), DVec3::new(8.0, 8.0, 10.0));
	let cut = try_difference(&block, &flush).expect("an exactly-coplanar pocket cut must succeed");
	let v = validate(&cut);
	assert!(
		v.closed && v.manifold && v.genus == 0 && (volume(&cut).abs() - 820.0).abs() < 1e-6,
		"flush (exactly coplanar) pocket must be a valid genus-0 solid of volume 820: got {v:?} vol={}",
		volume(&cut).abs()
	);

	// Near coplanar: a through-cut whose top is 1e-7 BELOW the block top, leaving a
	// 1e-7-thick sliver lid. Sub-tolerance => the arrangement cannot close it, and
	// the checked op must refuse rather than hand back an invalid solid.
	let near = cuboid(DVec3::new(2.0, 2.0, -1.0), DVec3::new(8.0, 8.0, 10.0 - 1e-7));
	assert!(
		try_difference(&block, &near).is_err(),
		"a sub-tolerance (1e-7) near-coplanar cut must be refused by try_difference (graceful failure), not returned as an invalid solid"
	);
}
