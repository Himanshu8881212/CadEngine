//! `Tpms` lattice primitive: each network family must be ~50% solid at level 0
//! (one labyrinth = half space, by symmetry), stay ≤ 1-Lipschitz (the narrow-band
//! contract), and mesh to a CLOSED surface when bounded by a shroud.

use kernel_implicit::{check_mesh, manifold_dual_contour, Aabb, Cuboid, Node, Resolution, Sdf, Tpms, TpmsKind, Vec3};

#[test]
fn tpms_networks_half_dense_one_lipschitz_and_mesh_closed() {
	let region = Aabb::new(Vec3::splat(-18.0), Vec3::splat(18.0));
	let n = 30usize;
	let lo = -14.0f32;
	let step = 28.0 / n as f32;
	let h = 0.05f32;

	let mut report = String::new();
	let mut all_ok = true;
	for kind in [TpmsKind::Gyroid, TpmsKind::SchwarzP, TpmsKind::Diamond, TpmsKind::Neovius, TpmsKind::SchoenIwp, TpmsKind::FischerKochS] {
		let net = Tpms::network(region, kind, 8.0, 0.0);

		// (1) one labyrinth ≈ 50% solid; (2) finite-difference gradient ≤ 1 (Lipschitz).
		let (mut inside, mut total, mut max_grad) = (0u32, 0u32, 0.0f32);
		for i in 0..n {
			for j in 0..n {
				for k in 0..n {
					let p = Vec3::new(lo + i as f32 * step, lo + j as f32 * step, lo + k as f32 * step);
					let d = net.distance(p);
					if d < 0.0 {
						inside += 1;
					}
					total += 1;
					let gx = (net.distance(p + Vec3::new(h, 0.0, 0.0)) - d) / h;
					let gy = (net.distance(p + Vec3::new(0.0, h, 0.0)) - d) / h;
					let gz = (net.distance(p + Vec3::new(0.0, 0.0, h)) - d) / h;
					max_grad = max_grad.max((gx * gx + gy * gy + gz * gz).sqrt());
				}
			}
		}
		let frac = inside as f32 / total as f32;

		// (3) bounded into a cube, the network meshes a CLOSED surface.
		let solid = Node::primitive(net).intersection(Node::primitive(Cuboid::new(Vec3::ZERO, Vec3::splat(14.0))));
		let mesh = manifold_dual_contour(&solid, Aabb::new(Vec3::splat(-16.0), Vec3::splat(16.0)), Resolution::VoxelSize(0.4));
		let rep = check_mesh(&mesh);

		let ok = (0.40..0.60).contains(&frac) && max_grad <= 1.05 && rep.boundary_edges == 0 && mesh.triangle_count() > 0;
		all_ok &= ok;
		report += &format!(
			"\n  {kind:?}: solid_frac={frac:.2} (want ~0.50)  max|grad|={max_grad:.3} (want ≤1)  boundary_edges={}  tris={}",
			rep.boundary_edges,
			mesh.triangle_count()
		);
	}

	assert!(all_ok, "TPMS network primitives must be ~50% solid at level 0, ≤1-Lipschitz, and mesh closed:{report}");
}
