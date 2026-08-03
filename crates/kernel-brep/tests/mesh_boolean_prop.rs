// Copyright (c) LMCAD. Licensed under the MIT License.

//! Property-based (fuzz) coverage for the two-mesh booleans. Two random axis-aligned
//! boxes are the arrangement's *exact* case (planar faces, no curved degeneracies),
//! so their boolean volumes must match inclusion–exclusion across all configurations
//! — exercising the grid broadphase and classification on many random overlaps.

use kernel_brep::math::DVec3;
use kernel_brep::{cuboid, mesh_difference, mesh_intersection, mesh_union, tessellate_default};
use proptest::prelude::*;

/// Length of the overlap of two 1-D intervals (0 if disjoint).
fn overlap1(a0: f64, a1: f64, b0: f64, b1: f64) -> f64 {
	(a1.min(b1) - a0.max(b0)).max(0.0)
}

proptest! {
	#![proptest_config(ProptestConfig::with_cases(48))]

	#[test]
	fn random_box_booleans_match_inclusion_exclusion(
		ax in -3.0f64..0.0, ay in -3.0f64..0.0, az in -3.0f64..0.0,
		aw in 1.0f64..4.0, ah in 1.0f64..4.0, ad in 1.0f64..4.0,
		bx in -2.0f64..1.0, by in -2.0f64..1.0, bz in -2.0f64..1.0,
		bw in 1.0f64..4.0, bh in 1.0f64..4.0, bd in 1.0f64..4.0,
	) {
		let amin = DVec3::new(ax, ay, az);
		let amax = amin + DVec3::new(aw, ah, ad);
		let bmin = DVec3::new(bx, by, bz);
		let bmax = bmin + DVec3::new(bw, bh, bd);
		let a = tessellate_default(&cuboid(amin, amax));
		let b = tessellate_default(&cuboid(bmin, bmax));

		let (va, vb) = (aw * ah * ad, bw * bh * bd);
		let ov = overlap1(amin.x, amax.x, bmin.x, bmax.x)
			* overlap1(amin.y, amax.y, bmin.y, bmax.y)
			* overlap1(amin.z, amax.z, bmin.z, bmax.z);

		let tol = 0.02;
		let u = mesh_union(&a, &b).signed_volume().abs() as f64;
		let i = mesh_intersection(&a, &b).signed_volume().abs() as f64;
		let d = mesh_difference(&a, &b).signed_volume().abs() as f64;
		prop_assert!((u - (va + vb - ov)).abs() < tol, "union {u} vs {}", va + vb - ov);
		prop_assert!((i - ov).abs() < tol, "intersection {i} vs {ov}");
		prop_assert!((d - (va - ov)).abs() < tol, "difference {d} vs {}", va - ov);
	}
}
