//! The signature HYBRID path (the kernel's differentiator, and the lattice
//! domain): a B-rep mounting plate fused with an SDF gyroid lattice column via
//! hybrid_boolean. Pins two honest contracts:
//!
//! 1. A RAW gyroid network is an open labyrinth (the region box cuts it), so the
//!    hybrid op REFUSES it (NotWatertight) rather than shipping a leaky body — you
//!    must clip the lattice with a closing solid first.
//! 2. A box-CLIPPED gyroid fuses to a watertight, 2-manifold result. A gyroid is a
//!    dense organic mesh the exact planar arrangement won't stitch, so it routes
//!    honestly through the voxel HEAL (route = Healed), and the provenance report
//!    counts the plate's faces.

use kernel_brep::cuboid;
use kernel_brep::math::DVec3;
use kernel_core::math::{Aabb, Vec3};
use kernel_implicit::{Cuboid, Node, Tpms, TpmsKind};
use kernel_model::hybrid::{hybrid_boolean, HybridOperand, HybridRoute};
use kernel_model::BooleanOp;

fn plate() -> kernel_brep::Solid {
	cuboid(DVec3::new(-20.0, -20.0, 0.0), DVec3::new(20.0, 20.0, 8.0))
}
fn region() -> Aabb {
	Aabb::new(Vec3::new(-10.0, -10.0, 8.0), Vec3::new(10.0, 10.0, 38.0))
}

#[test]
fn raw_open_gyroid_is_refused_but_clipped_gyroid_fuses_watertight_via_heal() {
	// (1) Raw (open) gyroid labyrinth -> the hybrid op refuses a leaky operand.
	let raw = Node::primitive(Tpms::network(region(), TpmsKind::Gyroid, 6.0, 0.0));
	assert!(
		hybrid_boolean(&plate(), HybridOperand::Node(&raw), BooleanOp::Union, 0.5).is_err(),
		"a raw open gyroid network must be refused (NotWatertight), not fused into a leaky body — clip it first"
	);

	// (2) Box-clipped gyroid -> watertight 2-manifold fuse via the honest heal.
	let clipped = Node::primitive(Tpms::network(region(), TpmsKind::Gyroid, 6.0, 0.0))
		.intersection(Node::primitive(Cuboid::new(Vec3::new(0.0, 0.0, 23.0), Vec3::new(10.0, 10.0, 15.0))));
	let res = hybrid_boolean(&plate(), HybridOperand::Node(&clipped), BooleanOp::Union, 0.5)
		.expect("a box-clipped gyroid must fuse with the plate");
	assert!(
		res.mesh.is_watertight()
			&& res.mesh.is_two_manifold()
			&& matches!(res.route, HybridRoute::Healed { .. })
			&& res.report.brep_faces == 6,
		"plate ∪ clipped gyroid must be watertight 2-manifold via the honest voxel heal (gyroid is voxel-native): \
		 watertight={} 2mf={} report={:?}",
		res.mesh.is_watertight(),
		res.mesh.is_two_manifold(),
		res.report
	);
}
