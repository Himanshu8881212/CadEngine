//! Production-grade input robustness: the public builders / booleans / queries
//! must NEVER panic on adversarial input (empty / NaN / inf / negative / zero /
//! mismatched / collinear). A library shipped to people returns None/Err or a
//! degenerate-but-handled result — it does not crash. The panic hook is silenced
//! so a (would-be) caught panic doesn't spam the test output.

use kernel_brep::math::{DVec2, DVec3};
use kernel_brep::*;
use std::panic::catch_unwind;

/// Returns `Some(name)` if `f` panicked, else `None`.
fn panics(name: &'static str, f: impl FnOnce() + std::panic::UnwindSafe) -> Option<&'static str> {
	catch_unwind(f).err().map(|_| name)
}

#[test]
fn public_api_never_panics_on_adversarial_input() {
	let prev = std::panic::take_hook();
	std::panic::set_hook(Box::new(|_| {}));
	let nan = f64::NAN;
	let inf = f64::INFINITY;

	let failures: Vec<&str> = [
		panics("extrude empty", || { extrude(&[], 5.0); }),
		panics("extrude single point", || { extrude(&[DVec2::new(1.0, 1.0)], 5.0); }),
		panics("extrude collinear", || { extrude(&[DVec2::new(0.0, 0.0), DVec2::new(1.0, 0.0), DVec2::new(2.0, 0.0)], 5.0); }),
		panics("extrude NaN profile", || { extrude(&[DVec2::new(nan, 0.0), DVec2::new(10.0, nan), DVec2::new(10.0, 10.0)], 5.0); }),
		panics("extrude bad heights", || {
			let s = [DVec2::new(0.0, 0.0), DVec2::new(10.0, 0.0), DVec2::new(10.0, 10.0), DVec2::new(0.0, 10.0)];
			extrude(&s, 0.0);
			extrude(&s, -5.0);
			extrude(&s, nan);
			extrude(&s, inf);
		}),
		panics("revolve degenerate", || {
			revolve(&[], 32);
			revolve(&[DVec2::new(1.0, 0.0), DVec2::new(2.0, 0.0), DVec2::new(2.0, 2.0)], 0);
			revolve(&[DVec2::new(-5.0, 0.0), DVec2::new(2.0, 0.0), DVec2::new(2.0, 2.0)], 32);
			revolve(&[DVec2::new(nan, 0.0), DVec2::new(2.0, 0.0), DVec2::new(2.0, 2.0)], 32);
		}),
		panics("cylinder degenerate", || {
			cylinder(DVec3::ZERO, DVec3::Z, 0.0, 10.0, 32);
			cylinder(DVec3::ZERO, DVec3::Z, -5.0, 10.0, 32);
			cylinder(DVec3::ZERO, DVec3::Z, nan, 10.0, 32);
			cylinder(DVec3::ZERO, DVec3::Z, 5.0, 10.0, 0);
			cylinder(DVec3::ZERO, DVec3::ZERO, 5.0, 10.0, 32);
		}),
		panics("cone/sphere/torus degenerate", || {
			cone(DVec3::ZERO, DVec3::Z, nan, 10.0, 32);
			sphere(DVec3::ZERO, 0.0, 16, 8);
			sphere(DVec3::ZERO, 5.0, 0, 0);
			torus(DVec3::ZERO, DVec3::Z, nan, 2.0, 32, 16);
		}),
		panics("cuboid inverted/NaN", || {
			cuboid(DVec3::splat(10.0), DVec3::splat(-10.0));
			cuboid(DVec3::splat(nan), DVec3::splat(10.0));
		}),
		panics("loft mismatch/NaN", || {
			loft_solid(&[vec![DVec3::ZERO; 3], vec![DVec3::ZERO; 4]]);
			loft_solid(&[vec![DVec3::splat(nan); 4], vec![DVec3::ZERO; 4]]);
		}),
		panics("sweep empty path", || { sweep_solid(&[DVec3::ZERO; 4], &[]); }),
		panics("booleans with degenerate", || {
			let good = cuboid(DVec3::ZERO, DVec3::splat(10.0));
			let degen = extrude(&[], 1.0);
			difference(&good, &degen);
			union(&good, &degen);
			let _ = try_difference(&good, &degen);
		}),
		panics("validate/volume/tess degenerate", || {
			let degen = extrude(&[], 1.0);
			validate(&degen);
			volume(&degen);
			tessellate_default(&degen);
		}),
	]
	.into_iter()
	.flatten()
	.collect();

	std::panic::set_hook(prev);
	assert!(failures.is_empty(), "the public API must not panic on adversarial input — these panicked: {failures:?}");
}
