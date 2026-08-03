// Copyright (c) LMCAD. Licensed under the MIT License.

//! Field-quality tagging + propagation, and the surfacing of that quality by
//! the distance-assuming ops (`offset` / `shell` / `offset_by`).
//!
//! The contract under test (see `kernel_implicit::ops` module docs): a node is
//! [`FieldQuality::ExactSdf`] only when its field is the true signed Euclidean
//! distance; otherwise it is a [`FieldQuality::DistanceBound`] (a 1-Lipschitz
//! bound that only agrees with the exact distance away from seams/blends).
//! Offsetting a bound is APPROXIMATE, and the checked ops must say so — without
//! ever changing the numeric result.

use std::sync::Arc;

use kernel_implicit::features::{chamfer_union, fillet_difference, fillet_union};
use kernel_implicit::primitives::{Capsule, Cone, Cuboid, Cylinder, Gyroid, Plane, Sphere, Torus};
use kernel_implicit::{FieldQuality, Node};
use kernel_core::math::{Aabb, Affine3A, Vec3};
use kernel_core::sdf::Sdf;

use FieldQuality::{DistanceBound as Bound, ExactSdf as Exact};

fn sphere() -> Node {
	Node::primitive(Sphere::new(Vec3::ZERO, 5.0))
}
fn cube() -> Node {
	Node::primitive(Cuboid::new(Vec3::new(4.0, 0.0, 0.0), Vec3::splat(5.0)))
}
fn gyroid() -> Node {
	Node::primitive_bound(Gyroid::new(Aabb::from_center_half_extent(Vec3::ZERO, Vec3::splat(8.0)), 0.35, 0.3))
}

#[test]
fn every_node_kind_is_classified_honestly() {
	// One snapshot over the whole grammar. EXACT: the analytic primitives, and
	// the three distance-preserving combinators (offset / shell / transform) OVER
	// an exact child. BOUND (rigorous, conservative): every boolean and smooth
	// blend, the fillet/chamfer wrappers, all patterns, the field-modulated ops,
	// the bound-tagged leaves, and any preserving op sitting ON a bound child.
	let ramp: kernel_implicit::ScalarField = Arc::new(|p: Vec3| 0.1 * p.z);
	let cases: Vec<(&str, Node, FieldQuality)> = vec![
		// --- exact analytic primitives ---
		("sphere", sphere(), Exact),
		("cuboid", cube(), Exact),
		("cylinder", Node::primitive(Cylinder::new(Vec3::ZERO, Vec3::Z * 10.0, 3.0)), Exact),
		("cone", Node::primitive(Cone::new(Vec3::ZERO, Vec3::Z * 10.0, 4.0, 1.0)), Exact),
		("torus", Node::primitive(Torus::new(Vec3::ZERO, Vec3::Z, 8.0, 2.0)), Exact),
		("capsule", Node::primitive(Capsule::new(-Vec3::X * 4.0, Vec3::X * 4.0, 2.0)), Exact),
		("plane", Node::primitive(Plane::new(Vec3::ZERO, Vec3::Z)), Exact),
		// --- bound-tagged leaves ---
		("gyroid(bound leaf)", gyroid(), Bound),
		// --- booleans: min/max is only a bound near seams ---
		("union", sphere().union(cube()), Bound),
		("intersection", sphere().intersection(cube()), Bound),
		("difference", sphere().difference(cube()), Bound),
		// --- smooth blends: bound by construction ---
		("smooth_union", sphere().smooth_union(cube(), 2.0), Bound),
		("smooth_intersection", sphere().smooth_intersection(cube(), 2.0), Bound),
		("smooth_difference", sphere().smooth_difference(cube(), 2.0), Bound),
		// --- fillet / chamfer wrappers: bound ---
		("fillet_union", fillet_union(sphere(), cube(), 2.0), Bound),
		("chamfer_union", chamfer_union(sphere(), cube(), 2.0), Bound),
		("fillet_difference", fillet_difference(sphere(), cube(), 2.0), Bound),
		// --- distance-preserving ops over an EXACT child stay exact ---
		("offset(exact)", sphere().offset(1.0), Exact),
		("shell(exact)", sphere().shell(1.0), Exact),
		("transform(exact)", sphere().transform(Affine3A::from_translation(Vec3::X)), Exact),
		("translate(exact)", sphere().translate(Vec3::Y * 3.0), Exact),
		("scale(exact)", sphere().scale(2.0), Exact),
		// --- distance-preserving ops over a BOUND child propagate the bound ---
		("offset(bound)", sphere().union(cube()).offset(1.0), Bound),
		("shell(bound)", sphere().union(cube()).shell(1.0), Bound),
		("transform(bound)", gyroid().transform(Affine3A::from_translation(Vec3::X)), Bound),
		// --- patterns: min-union of copies → bound ---
		("linear_pattern", sphere().linear_pattern(Vec3::X * 12.0, 3), Bound),
		("circular_pattern", sphere().circular_pattern(Vec3::ZERO, Vec3::Z, 1.0, 4), Bound),
		("mirror", sphere().translate(Vec3::X * 8.0).mirror(Vec3::ZERO, Vec3::X), Bound),
		// --- field-modulated ops: never a true SDF ---
		("offset_by", sphere().offset_by(ramp.clone(), 2.0), Bound),
		("lerp", sphere().lerp(cube(), ramp.clone()), Bound),
		// --- deep nesting: exact all the way, vs. one bound poisons the branch ---
		("transform(offset(shell(sphere)))", sphere().shell(1.0).offset(0.5).transform(Affine3A::from_translation(Vec3::Z)), Exact),
		("offset(union) nested", sphere().union(cube()).offset(1.0).transform(Affine3A::IDENTITY), Bound),
	];

	let mismatches: Vec<String> = cases
		.iter()
		.filter(|(_, n, want)| n.field_quality() != *want)
		.map(|(label, n, want)| format!("{label}: got {:?}, want {want:?}", n.field_quality()))
		.collect();
	assert!(mismatches.is_empty(), "field-quality classification disagrees with the honest contract:\n{}", mismatches.join("\n"));
}

#[test]
fn checked_offset_surfaces_quality_without_changing_the_result() {
	// The checked op must (a) flag an exact input as NOT approximate with no
	// warning, (b) flag a bound input (smooth blend) as approximate WITH a loud
	// warning, and (c) return the bit-identical field the unchecked op would —
	// surfacing the quality must never silently alter geometry.
	let exact = sphere().offset_checked(1.5);
	assert!(
		!exact.is_approximate() && exact.warning().is_none() && exact.input_quality == Exact && exact.op == "offset",
		"exact offset must be reported exact: approx={} warn={:?} q={:?}",
		exact.is_approximate(),
		exact.warning(),
		exact.input_quality
	);

	let blended = sphere().smooth_union(cube(), 3.0).offset_checked(1.5);
	let w = blended.warning();
	assert!(
		blended.is_approximate() && blended.input_quality == Bound && w.as_deref().is_some_and(|s| s.contains("APPROXIMATE") && s.contains("offset")),
		"smooth-blend offset must be reported approximate with a loud warning: approx={} q={:?} warn={w:?}",
		blended.is_approximate(),
		blended.input_quality
	);

	// Result identity: checked node == unchecked node, pointwise.
	let checked = sphere().smooth_union(cube(), 3.0).offset_checked(1.5).node;
	let unchecked = sphere().smooth_union(cube(), 3.0).offset(1.5);
	let max_diff = [Vec3::ZERO, Vec3::new(3.0, 1.0, -2.0), Vec3::new(7.0, 0.0, 0.0), Vec3::new(-4.0, 4.0, 4.0)]
		.iter()
		.map(|&p| (checked.distance(p) - unchecked.distance(p)).abs())
		.fold(0.0f32, f32::max);
	assert!(max_diff == 0.0, "checked offset must equal the unchecked field exactly, max |Δ| = {max_diff}");
}

#[test]
fn checked_shell_and_offset_by_surface_quality() {
	// shell: exact input clean, bound input flagged.
	let shell_exact = sphere().shell_checked(1.0);
	let shell_bound = sphere().union(cube()).shell_checked(1.0);
	assert!(
		!shell_exact.is_approximate() && shell_bound.is_approximate() && shell_bound.op == "shell" && shell_bound.warning().is_some(),
		"shell_checked: exact clean={}, bound flagged={} (op={}, warn={:?})",
		!shell_exact.is_approximate(),
		shell_bound.is_approximate(),
		shell_bound.op,
		shell_bound.warning()
	);

	// offset_by ALWAYS produces a bound; the check reports on the INPUT the op
	// assumes exact — an exact input is still reported clean (the input was
	// exact), while a bound input is flagged doubly approximate.
	let ramp: kernel_implicit::ScalarField = Arc::new(|p: Vec3| 0.05 * p.z);
	let ob_exact = sphere().offset_by_checked(ramp.clone(), 2.0);
	let ob_bound = sphere().smooth_union(cube(), 2.0).offset_by_checked(ramp.clone(), 2.0);
	assert!(
		ob_exact.input_quality == Exact
			&& !ob_exact.is_approximate()
			&& ob_bound.input_quality == Bound
			&& ob_bound.is_approximate()
			&& ob_bound.op == "offset_by"
			// the RESULT node of offset_by is a bound regardless of input.
			&& ob_exact.node.field_quality() == Bound
			&& ob_bound.node.field_quality() == Bound,
		"offset_by_checked: exact-input clean={}, bound-input flagged={}, result-always-bound=({:?},{:?})",
		!ob_exact.is_approximate(),
		ob_bound.is_approximate(),
		ob_exact.node.field_quality(),
		ob_bound.node.field_quality()
	);
}

/// Unit 4a: `has_approximate_offset` flags an offset/shell/offset_by applied to a
/// distance-BOUND field (the unsound case the checked constructors warn about),
/// and stays quiet for an offset of an exact primitive OR a legitimate bound
/// blend that is NOT offset (a smooth_union on its own is fine — only offsetting
/// it is unsound). This is what the `implicit` op surfaces so the approximation
/// can never pass unnoticed.
#[test]
fn has_approximate_offset_flags_only_the_unsound_offsets() {
	let exact_offset = sphere().offset(1.0); // exact input -> sound
	let sound_shell = sphere().shell(1.0); // exact input -> sound
	let blend_only = sphere().smooth_union(cube(), 3.0); // bound, but NOT offset -> fine
	let unsound_offset = sphere().smooth_union(cube(), 3.0).offset(1.0); // offset OF a bound -> unsound
	let unsound_shell = sphere().union(cube()).shell(1.0); // shell OF a bound -> unsound
	let nested = sphere().smooth_union(cube(), 3.0).offset(1.0).transform(Affine3A::IDENTITY); // unsound, deep
	assert!(
		!exact_offset.has_approximate_offset()
			&& !sound_shell.has_approximate_offset()
			&& !blend_only.has_approximate_offset()
			&& unsound_offset.has_approximate_offset()
			&& unsound_shell.has_approximate_offset()
			&& nested.has_approximate_offset(),
		"has_approximate_offset must flag ONLY offset/shell of a bound field: \
		 exact_offset={} sound_shell={} blend_only={} unsound_offset={} unsound_shell={} nested={}",
		exact_offset.has_approximate_offset(),
		sound_shell.has_approximate_offset(),
		blend_only.has_approximate_offset(),
		unsound_offset.has_approximate_offset(),
		unsound_shell.has_approximate_offset(),
		nested.has_approximate_offset(),
	);
}
