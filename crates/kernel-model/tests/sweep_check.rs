//! The promoted kinematic-sweep idiom: `sweep_check` must report honest
//! clearances on approach poses and DETECT a forced interpenetration via the
//! vertex-sampled penetration estimate — while `penetration_estimate` reads
//! 0.0 for separated bodies.

use kernel_brep::math::{DAffine3, DVec3};
use kernel_brep::{cuboid, tessellate_default};
use kernel_model::{materials, penetration_estimate, sweep_check};

#[test]
fn sweep_check_clears_approaches_and_detects_forced_overlap() {
	let fixed = tessellate_default(&cuboid(DVec3::new(-10.0, -10.0, 0.0), DVec3::new(10.0, 10.0, 10.0)));
	let moving = tessellate_default(&cuboid(DVec3::new(-5.0, -5.0, 0.0), DVec3::new(5.0, 5.0, 8.0)));
	// approach from above: gaps 10, 4, 1, 0.2 — then a forced 2.0 overlap
	let poses: Vec<DAffine3> = [20.0, 14.0, 11.0, 10.2, 8.0]
		.iter()
		.map(|&z| DAffine3::from_translation(DVec3::new(0.0, 0.0, z)))
		.collect();
	let rep = sweep_check(&fixed, &moving, &poses);
	// a genuinely separated pair (10 mm air gap) must estimate zero penetration
	let far = tessellate_default(&cuboid(DVec3::new(-5.0, -5.0, 20.0), DVec3::new(5.0, 5.0, 28.0)));
	let sep = penetration_estimate(&fixed, &far, 4000);
	assert!(
		rep.poses.len() == 5
			&& (rep.poses[0].min_distance - 10.0).abs() < 0.01
			&& (rep.poses[3].min_distance - 0.2).abs() < 0.01
			&& (rep.min_clearance - 0.2).abs() < 0.01
			&& rep.max_penetration > 1.0
			&& rep.poses[4].penetration > 1.0
			&& rep.contacts == 1
			&& rep.crossings == 1
			&& sep == 0.0
			&& (materials::PLA_G_PER_MM3 - 0.00124).abs() < 1e-12,
		"sweep contract: 5 poses (got {}), far gap {:.3} (want 10), near gap {:.3} (want 0.2), min_clearance {:.3}, \
		 forced-overlap penetration {:.2} (want >1, vertex-sampled), contacts {} (want 1), crossings {} (want 1), separated estimate {sep} (want 0), \
		 PLA const {}",
		rep.poses.len(),
		rep.poses[0].min_distance,
		rep.poses[3].min_distance,
		rep.min_clearance,
		rep.max_penetration,
		rep.contacts,
		rep.crossings,
		materials::PLA_G_PER_MM3
	);
}


/// THE vertex-blind regression: a wide plate crossing a THIN wall, posed so
/// neither mesh has a single vertex inside the other. Sampled penetration
/// reads 0.0 — the exact crossing oracle must still convict. (This is the
/// slider-through-parapet miss, pinned.)
#[test]
fn exact_crossing_convicts_where_vertex_sampling_is_blind() {
	let wall = tessellate_default(&cuboid(DVec3::new(-1.0, -30.0, 0.0), DVec3::new(1.0, 30.0, 40.0)));
	let plate = tessellate_default(&cuboid(DVec3::new(-40.0, -20.0, 15.0), DVec3::new(40.0, 20.0, 18.0)));
	let poses = [DAffine3::IDENTITY];
	let rep = sweep_check(&wall, &plate, &poses);
	let pen = penetration_estimate(&wall, &plate, 8000);
	assert!(
		rep.crossings == 1 && rep.poses[0].crossing && pen == 0.0 && rep.contacts == 1,
		"vertex-blind crossing pin: crossings {} (want 1), sampled pen {pen} (must be 0.0 — the blindness this \
		 test exists to preserve evidence of), contacts {}",
		rep.crossings,
		rep.contacts
	);
}
