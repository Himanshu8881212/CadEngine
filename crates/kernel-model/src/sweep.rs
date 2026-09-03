// Copyright (c) LMCAD. Licensed under the MIT License.

//! Posed-pair sweep checking: move one mesh through a list of poses against a
//! fixed one and report the worst penetration and the tightest clearance.

use kernel_core::mesh::Mesh;

/// Vertex-sampled penetration ESTIMATE between two meshes: the deepest of
/// either mesh's vertices inside the other's winding-number field, in model
/// units (0.0 ⟺ no sampled vertex is contained). Hundreds of times cheaper
/// than an exact boolean, so a kinematic sweep can afford dense poses —
/// but it is an **underestimate by construction** (an edge–edge crossing
/// with no contained vertex reads 0.0): gate load-bearing poses with
/// [`kernel_brep::overlap_volume`], use this for the dense in-between poses.
/// At most `max_samples` vertices per side are tested (evenly strided).
pub fn penetration_estimate(a: &Mesh, b: &Mesh, max_samples: usize) -> f64 {
	let mut worst = 0.0f64;
	let mut probe = |host: &Mesh, guest: &Mesh| {
		let sdf = kernel_implicit::MeshSdf::new(host);
		let n = guest.positions.len().max(1);
		let stride = (n / max_samples.max(1)).max(1);
		for p in guest.positions.iter().step_by(stride) {
			let d = kernel_core::sdf::Sdf::distance(&sdf, *p) as f64;
			if d < -worst {
				worst = -d;
			}
		}
	};
	probe(a, b);
	probe(b, a);
	worst
}

/// One pose of a [`sweep_check`]: the mesh↔mesh clearance, the sampled
/// penetration estimate, and the EXACT proper-crossing verdict at that pose.
#[derive(Clone, Copy, Debug)]
pub struct SweepPose {
	pub min_distance: f64,
	pub penetration: f64,
	/// Exact triangle-level proper crossing ([`Mesh::crosses_mesh`]) — the
	/// oracle vertex sampling cannot fake.
	pub crossing: bool,
}

/// Result of sweeping a moving mesh against a fixed one along a pose path.
#[derive(Clone, Debug)]
pub struct SweepReport {
	pub poses: Vec<SweepPose>,
	/// Smallest clearance seen across poses with zero sampled penetration.
	pub min_clearance: f64,
	/// Deepest sampled penetration across all poses (0.0 = none detected).
	pub max_penetration: f64,
	/// Poses whose surface distance was ≈0 (< 0.02): touching OR crossing.
	/// The penetration estimate is vertex-sampled and can read 0.0 through a
	/// thin wall with no contained vertices (a real slider-through-parapet
	/// collision did exactly that, DRYBOX 2026-07-28) — so a FREE-RUN gate
	/// must assert `contacts == 0`, not just `max_penetration ≈ 0`.
	pub contacts: usize,
	/// Poses with an EXACT proper triangle crossing — the definitive
	/// interpenetration verdict (touching and coplanar kisses excluded).
	/// A free-run gate asserts `crossings == 0 && contacts == 0`; an
	/// intentional-interference sweep (a click ring) expects `crossings > 0`.
	pub crossings: usize,
}

/// The campaign kinematic-sweep idiom (DOVESTACK → POOLDOCK → RESPOOL),
/// promoted: pose `moving` by each transform, measure clearance to `fixed`
/// (BVH mesh distance) and a sampled penetration estimate. Cheap enough for
/// dense insertion/twist paths; see [`penetration_estimate`] for what the
/// estimate can and cannot see — poses that must PROVE non-interference
/// (locks, seats) still deserve an exact `overlap_volume` gate on top.
pub fn sweep_check(fixed: &Mesh, moving: &Mesh, poses: &[kernel_core::math::DAffine3]) -> SweepReport {
	// Poses are independent — kernel_core::par::par_map_indexed evaluates
	// them on scoped threads and returns BY INDEX, so the report is identical
	// to a serial run regardless of scheduling. (Coarse-grained only: the
	// boolean arrangement stays single-threaded to protect R5.)
	let results: Vec<SweepPose> = kernel_core::par::par_map_indexed(poses, |_, m| {
		let posed = moving.transformed_by(*m);
		let min_distance = fixed.min_distance(&posed);
		let near = min_distance < 0.05;
		let penetration = if near { penetration_estimate(fixed, &posed, 4000) } else { 0.0 };
		let crossing = near && fixed.crosses_mesh(&posed);
		SweepPose { min_distance, penetration, crossing }
	});
	let mut out = SweepReport {
		poses: Vec::with_capacity(poses.len()),
		min_clearance: f64::INFINITY,
		max_penetration: 0.0,
		contacts: 0,
		crossings: 0,
	};
	for sp in results {
		if sp.min_distance < 0.02 {
			out.contacts += 1;
		}
		if sp.crossing {
			out.crossings += 1;
		}
		if sp.penetration == 0.0 {
			out.min_clearance = out.min_clearance.min(sp.min_distance);
		}
		out.max_penetration = out.max_penetration.max(sp.penetration);
		out.poses.push(sp);
	}
	out
}
