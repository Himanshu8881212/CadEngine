//! ChainLog must record every good step, refuse the first bad one BY NAME,
//! and keep the last-good solid; the sealed booleans must return the solid
//! together with its verified-watertight default tessellation.

use kernel_brep::math::DVec3;
use kernel_brep::topo::{FaceInput, Solid};
use kernel_brep::{
	cuboid, cylinder, difference, try_difference_sealed, validate, volume, ChainLog, Surface,
};

/// An intentionally OPEN solid (five faces of a box): validates as not-closed,
/// the deterministic "bad op result" for exercising the chain's refusal path.
fn open_box() -> Solid {
	let lo = DVec3::ZERO;
	let hi = DVec3::new(10.0, 10.0, 10.0);
	let positions = vec![
		DVec3::new(lo.x, lo.y, lo.z),
		DVec3::new(hi.x, lo.y, lo.z),
		DVec3::new(lo.x, hi.y, lo.z),
		DVec3::new(hi.x, hi.y, lo.z),
		DVec3::new(lo.x, lo.y, hi.z),
		DVec3::new(hi.x, lo.y, hi.z),
		DVec3::new(lo.x, hi.y, hi.z),
		DVec3::new(hi.x, hi.y, hi.z),
	];
	let quad = |q: [u32; 4], origin: DVec3, normal: DVec3| FaceInput {
		boundary: q.to_vec(),
		surface: Surface::Plane { origin, normal },
	};
	// no +Z cap — an open shoebox
	let faces = vec![
		quad([0, 2, 3, 1], DVec3::ZERO, -DVec3::Z),
		quad([0, 1, 5, 4], DVec3::ZERO, -DVec3::Y),
		quad([2, 6, 7, 3], hi, DVec3::Y),
		quad([0, 4, 6, 2], DVec3::ZERO, -DVec3::X),
		quad([1, 3, 7, 5], hi, DVec3::X),
	];
	Solid::from_faces(positions, faces)
}

#[test]
fn chain_log_records_good_steps_and_names_the_first_bad_one() {
	let plate = cuboid(DVec3::new(0.0, 0.0, 0.0), DVec3::new(30.0, 20.0, 6.0));
	let bore = cylinder(DVec3::new(10.0, 10.0, -1.0), DVec3::Z, 3.0, 8.0, 48);
	let bore2 = cylinder(DVec3::new(22.0, 10.0, -1.0), DVec3::Z, 2.0, 8.0, 48);

	let mut chain = ChainLog::start("plate", plate.clone()).expect("plate validates").seal();
	chain.apply("bore1", |s| difference(s, &bore)).expect("bore1 is a clean cut");
	chain.apply("bore2", |s| difference(s, &bore2)).expect("bore2 is a clean cut");
	let good_volume = volume(chain.solid()).abs();
	let steps_before = chain.steps().len();

	// a "cutter" op that returns an open solid: the chain must refuse, name the
	// step, and keep the last-good solid untouched
	let err = chain.apply("evil", |_| open_box()).expect_err("an open result must be refused");
	let direct = volume(&difference(&difference(&plate, &bore), &bore2)).abs();

	assert!(
		steps_before == 3
			&& chain.steps().iter().all(|s| s.validity.is_valid() && s.watertight == Some(true))
			&& err.label == "evil"
			&& err.step == 3
			&& !err.validity.closed
			&& (volume(chain.solid()).abs() - good_volume).abs() < 1e-9
			&& (good_volume - direct).abs() < 1e-9,
		"ChainLog contract: 3 sealed good steps (got {}, all valid+wt={}), refusal names 'evil' step 3 \
		 (got '{}' step {}, closed={}), last-good preserved ({} vs {}), matches direct chain ({})",
		steps_before,
		chain.steps().iter().all(|s| s.validity.is_valid() && s.watertight == Some(true)),
		err.label,
		err.step,
		err.validity.closed,
		volume(chain.solid()).abs(),
		good_volume,
		direct
	);
}

#[test]
fn sealed_boolean_returns_solid_with_verified_watertight_mesh() {
	let plate = cuboid(DVec3::new(-10.0, -10.0, -3.0), DVec3::new(10.0, 10.0, 3.0));
	let bore = cylinder(DVec3::new(0.0, 0.0, -4.0), DVec3::Z, 2.5, 8.0, 48);
	let (drilled, mesh) = try_difference_sealed(&plate, &bore).expect("a drilled plate seals clean");
	assert!(
		validate(&drilled).is_valid() && mesh.is_watertight() && volume(&drilled).abs() > 0.0,
		"sealed difference: valid={} watertight={} volume={}",
		validate(&drilled).is_valid(),
		mesh.is_watertight(),
		volume(&drilled).abs()
	);
}
