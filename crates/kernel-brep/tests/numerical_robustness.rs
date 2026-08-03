//! Numerical robustness at extreme distance-from-origin and scale — a real
//! concern for parts placed far out in a large assembly, or micro/large parts.
//! A cross-bored block (genus 3) must stay a VALID genus-3 solid when built far
//! from the origin or at extreme scale; volume is translation-invariant (to
//! graceful f32-tessellation precision) and scales exactly with size^3.

use kernel_brep::math::{DAffine3, DVec3};
use kernel_brep::{cuboid, cylinder, try_difference, validate, volume, Solid};

fn cross_bored(scale: f64) -> Solid {
	let (h, r, l) = (25.0 * scale, 10.0 * scale, 60.0 * scale);
	let block = cuboid(DVec3::splat(-h), DVec3::splat(h));
	let bx = cylinder(DVec3::new(-l / 2.0, 0.0, 0.0), DVec3::X, r, l, 48);
	let by = cylinder(DVec3::new(0.0, -l / 2.0, 0.0), DVec3::Y, r, l, 48);
	try_difference(&block, &bx).and_then(|s| try_difference(&s, &by)).expect("cross-bored block builds")
}

fn assert_valid_genus3(name: &str, s: &Solid, want_vol: f64, tol: f64) {
	let v = validate(s);
	let vol = volume(s).abs();
	let rel = (vol - want_vol).abs() / want_vol;
	assert!(
		v.closed && v.manifold && v.genus == 3 && rel < tol,
		"{name}: must stay a valid genus-3 solid with volume ~{want_vol:.3e}: {v:?} vol={vol:.3e} relerr={rel:.2e} (tol {tol:.0e})"
	);
}

#[test]
fn cross_bore_stays_valid_far_from_origin_and_at_extreme_scale() {
	let base = cross_bored(1.0);
	let v0 = volume(&base).abs();

	// One kilometre from the origin: still valid, volume preserved to <0.1% (the
	// f32 tessellation loses a little precision out there, but never goes invalid).
	let far = base.transformed(DAffine3::from_translation(DVec3::splat(1.0e6)));
	assert_valid_genus3("translated +1e6", &far, v0, 1e-3);

	// Micro and large scale: valid, volume scales exactly as size^3.
	assert_valid_genus3("scale 0.01", &cross_bored(0.01), v0 * 0.01f64.powi(3), 1e-6);
	assert_valid_genus3("scale 100", &cross_bored(100.0), v0 * 100.0f64.powi(3), 1e-6);
}
