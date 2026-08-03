//! Regression: degenerate / non-finite catalog inputs must return `None`, not
//! panic (index out of bounds) or OOM (`inf as usize` section count).

use kernel_model::parts::{compression_spring, iso_thread_solid};

#[test]
fn degenerate_catalog_inputs_return_none_not_panic_or_oom() {
	// Sub-step fractional turns round the step count to 0 -> would index path[1] OOB.
	assert!(compression_spring(1.0, 8.0, 2.0, 0.005).is_none(), "tiny active_turns must be None");
	// A normal spring still builds.
	assert!(compression_spring(1.0, 8.0, 2.0, 5.0).is_some(), "a valid spring still builds");
	// Non-finite length -> `inf as usize` section count -> OOM; must be refused.
	assert!(iso_thread_solid(6.0, 1.0, 0.0, f64::INFINITY).is_none(), "infinite length must be None");
	assert!(iso_thread_solid(6.0, 1.0, 0.0, f64::NAN).is_none(), "NaN length must be None");
	// A valid thread still builds.
	assert!(iso_thread_solid(6.0, 1.0, 0.0, 5.0).is_some(), "a valid thread still builds");
}
