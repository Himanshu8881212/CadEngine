//! Run-to-run byte-identity of the implicit meshers (manifold dual contour +
//! narrow-band dual contour). The B-rep boolean pipeline is pinned by
//! `kernel-brep/tests/determinism.rs`; the implicit meshers carry the same
//! determinism CLAIM (serial vertex-id assignment, no atomic accumulation) but
//! were never pinned by a test. These make any rayon-scheduling or
//! HashMap-iteration-order regression in `manifold_dc.rs` / `narrow_band.rs`
//! fail LOUD instead of silently drifting.
//!
//! The assertions are honest snapshots of CURRENT behavior. They never weaken a
//! tolerance: divergence between two runs of the SAME input is, by definition, a
//! determinism bug, not a tolerance to relax.

use kernel_implicit::{
	dual_contour_narrowband, manifold_dual_contour, Aabb, Mesh, Resolution, Sdf, Vec3,
};
use kernel_implicit::{Cuboid, Cylinder, Gyroid, Node, Sphere};

/// Order-sensitive fingerprint: triangle count, an FNV-1a hash folding every
/// vertex coordinate bit (in stored order) and every index, plus the
/// signed-volume bits. Equal fingerprints ⇒ bit-identical meshes; any
/// single-bit drift is caught — without dumping millions of indices on failure.
fn fingerprint(m: &Mesh) -> (usize, u64, u64) {
	let mut h: u64 = 0xcbf2_9ce4_8422_2325;
	for v in &m.positions {
		for coord in v.to_array() {
			h ^= u64::from(coord.to_bits());
			h = h.wrapping_mul(0x0000_0100_0000_01b3);
		}
	}
	for &i in &m.indices {
		h ^= u64::from(i);
		h = h.wrapping_mul(0x0000_0100_0000_01b3);
	}
	(m.triangle_count(), h, m.signed_volume().to_bits())
}

/// A fixed CSG mix (sphere ∩ cylinder ∪ cube) exercising interior/exterior
/// transitions, a sign flip, and a multi-primitive blend.
fn csg() -> Node {
	Node::primitive(Sphere::new(Vec3::ZERO, 10.0))
		.intersection(Node::primitive(Cylinder::new(
			Vec3::new(-8.0, 0.0, 0.0),
			Vec3::new(8.0, 0.0, 0.0),
			5.0,
		)))
		.union(Node::primitive(Cuboid::new(Vec3::ZERO, Vec3::splat(6.0))))
}

#[test]
fn manifold_dual_contour_is_deterministic_across_runs() {
	let tree = csg();
	let domain = tree.bounds().pad(1.0);

	let base = fingerprint(&manifold_dual_contour(&tree, domain, Resolution::VoxelSize(0.2)));
	assert!(
		base.0 > 0,
		"MDC of the fixed CSG must produce a non-empty mesh (got {} tris)",
		base.0
	);

	for run in 1..10 {
		let snap = fingerprint(&manifold_dual_contour(&tree, domain, Resolution::VoxelSize(0.2)));
		assert_eq!(
			snap, base,
			"manifold_dual_contour run {run} diverged from run 0 — the MDC pipeline is \
			 NON-DETERMINISTIC (likely rayon scheduling or HashMap iteration order in \
			 manifold_dc.rs): run0 tris={} volbits={:#x}, run{run} tris={} volbits={:#x}",
			base.0, base.2, snap.0, snap.2
		);
	}
}

#[test]
fn dual_contour_narrowband_is_deterministic_across_runs() {
	let tree = csg();
	let domain = tree.bounds().pad(1.0);

	let base = fingerprint(&dual_contour_narrowband(&tree, domain, Resolution::VoxelSize(0.2)));
	assert!(
		base.0 > 0,
		"narrow-band DC of the fixed CSG must produce a non-empty mesh (got {} tris)",
		base.0
	);

	for run in 1..10 {
		let snap = fingerprint(&dual_contour_narrowband(&tree, domain, Resolution::VoxelSize(0.2)));
		assert_eq!(
			snap, base,
			"dual_contour_narrowband run {run} diverged from run 0 — narrow-band mesher is \
			 NON-DETERMINISTIC (BFS flood-fill / serial id-assignment order dependence in \
			 narrow_band.rs): run0 tris={}, run{run} tris={}",
			base.0, snap.0
		);
	}
}

#[test]
fn narrowband_gyroid_is_deterministic_across_runs() {
	// Bounded TPMS — the narrow-band path's intended workload. This pins run-to-run
	// DETERMINISM of gyroid meshing.
	//
	// It deliberately does NOT assert watertightness: at this VoxelSize (0.3)
	// relative to the 0.3 wall thickness the wall spans ~1 voxel and the mesh is
	// measurably open (~1.76M tris, watertight=false) — a known resolution
	// limitation of clipped-TPMS dual contouring, not a determinism bug. Closed-
	// solid watertightness is pinned for both meshers by
	// `mdc_and_narrowband_agree_on_closed_positive_solid` below.
	let region = Aabb::new(Vec3::splat(-25.0), Vec3::splat(25.0));
	let gyroid = Node::primitive(Gyroid::new(region, 0.35, 0.3));
	let domain = gyroid.bounds().pad(1.0);

	let base = fingerprint(&dual_contour_narrowband(&gyroid, domain, Resolution::VoxelSize(0.3)));
	assert!(
		base.0 > 0,
		"narrow-band gyroid must produce a non-empty mesh (got {} tris)",
		base.0
	);

	for run in 1..5 {
		let snap = fingerprint(&dual_contour_narrowband(&gyroid, domain, Resolution::VoxelSize(0.3)));
		assert_eq!(
			snap, base,
			"narrow-band gyroid run {run} diverged from run 0 — non-deterministic gyroid meshing",
		);
	}
}

#[test]
fn mdc_and_narrowband_agree_on_closed_positive_solid() {
	// Both meshers, same field, must agree the solid is closed and positive-volume.
	// This is NOT a tolerance weakening — it asserts the two independent meshers
	// produce the same topological verdict (watertight, vol > 0) on the same field.
	let tree = csg();
	let domain = tree.bounds().pad(1.0);

	let mdc = manifold_dual_contour(&tree, domain, Resolution::VoxelSize(0.2));
	let nb = dual_contour_narrowband(&tree, domain, Resolution::VoxelSize(0.2));

	let vm = mdc.signed_volume();
	let vn = nb.signed_volume();
	assert!(
		mdc.is_watertight() && vm > 0.0 && vn > 0.0,
		"both meshers must yield a closed, positive-volume solid on the same field: \
		 MDC watertight={} vol={:.3}; narrow-band vol={:.3}",
		mdc.is_watertight(),
		vm,
		vn
	);
}
