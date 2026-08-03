// Copyright (c) LMCAD. Licensed under the MIT License.

//! Analytic torus inertia: `mass_properties` adds the closed-form lens
//! first+second-moment correction for toroidal faces (`torus_lens_moments` in
//! validate.rs), so a full torus's inertia tensor is exact at ANY facet count —
//! a coarse tessellation must match a fine one AND the closed-form solid-torus
//! inertia (`I_axis = m(R² + ¾r²)`, `I_perp = m(½R² + ⅝r²)` about the centre).
//! Before this correction a 12×8 torus's inertia was ~18% off a 200×100 one.
//! The torus is deliberately off-origin with a tilted axis so the world-frame
//! offset and basis-change terms of the correction are exercised, not just the
//! easy axis-aligned case.

use kernel_brep::math::{DMat3, DVec3};
use kernel_brep::{mass_properties, tessellate_default, torus};

/// `aᵀ M a` — the inertia about the (unit) direction `a`.
fn about(m: &DMat3, a: DVec3) -> f64 {
	a.dot(*m * a)
}

#[test]
fn full_torus_inertia_is_analytic_and_matches_the_closed_form() {
	// A solid torus is the revolve of a circle (r = 4) about an axis at ring
	// radius R = 12 — built here by the `torus` primitive, whose facets carry the
	// analytic Surface::Torus tag with vertices exactly on the surface (the plain
	// `revolve` builder tags only Cylinder/Plane/Cone, so it cannot receive the
	// torus lens correction; that routing is documented on `revolve`).
	let (r_maj, r_min) = (12.0_f64, 4.0_f64);
	let center = DVec3::new(3.0, -2.0, 5.0);
	let axis = DVec3::new(1.0, 2.0, 2.0) / 3.0; // unit, deliberately tilted
	let coarse = mass_properties(&torus(center, axis, r_maj, r_min, 12, 8));
	let fine = mass_properties(&torus(center, axis, r_maj, r_min, 200, 100));

	// Closed-form solid torus at unit density, about its centre of mass.
	let m = 2.0 * std::f64::consts::PI.powi(2) * r_maj * r_min * r_min;
	let i_axis = m * (r_maj * r_maj + 0.75 * r_min * r_min);
	let i_perp = m * (0.5 * r_maj * r_maj + 0.625 * r_min * r_min);

	let axis_rel = (about(&coarse.inertia, axis) - i_axis).abs() / i_axis;
	let trace_c = coarse.inertia.x_axis.x + coarse.inertia.y_axis.y + coarse.inertia.z_axis.z;
	let trace_f = fine.inertia.x_axis.x + fine.inertia.y_axis.y + fine.inertia.z_axis.z;
	let trace_rel = (trace_c - (i_axis + 2.0 * i_perp)).abs() / (i_axis + 2.0 * i_perp);
	let coarse_vs_fine = (trace_c - trace_f).abs() / trace_f;
	// The perpendicular moment, read off the coarse tensor in a direction ⊥ axis.
	let perp = (DVec3::X - axis * axis.x).normalize();
	let perp_rel = (about(&coarse.inertia, perp) - i_perp).abs() / i_perp;
	let vol_rel = (coarse.volume - m).abs() / m;
	let com_off = (coarse.center_of_mass - center).length() / r_maj;

	// Negative control: the RAW tessellation-level inertia of the same coarse solid
	// (no lens correction) must be materially off the closed form — this is the gap
	// the correction closes; if it ever reads near-zero the control facets stopped
	// being chords and the whole lens construction needs re-examination.
	let raw = tessellate_default(&torus(center, axis, r_maj, r_min, 12, 8)).mass_properties();
	let raw_trace = raw.inertia.x_axis.x + raw.inertia.y_axis.y + raw.inertia.z_axis.z;
	let raw_rel = (raw_trace - (i_axis + 2.0 * i_perp)).abs() / (i_axis + 2.0 * i_perp);

	// The correction itself is machine-exact in f64 (the lens terms telescope over
	// the θ-closed torus; volume, pure f64, lands at ~1e-15). The inertia/CoM floor
	// is set by the FACETED baseline the correction is added to: `tessellate_default`
	// meshes into f32 positions, so the tessellation-side moments carry ~1e-7-relative
	// noise at any facet count (measured here: ~1e-8, non-converging, sign-flipping —
	// the same floor behind the 1e-5 bar in `mass_properties_analytic.rs` for the
	// cylinder/sphere/cone lenses). 1e-5 keeps that sibling bar; it is still ~18000×
	// below the 18.6% coarse-vs-closed-form gap this correction closes (asserted as
	// the negative control below) and ~500× below the 0.5% acceptance bar.
	assert!(
		axis_rel < 1e-5 && perp_rel < 1e-5 && trace_rel < 1e-5 && coarse_vs_fine < 1e-5 && vol_rel < 1e-12 && com_off < 1e-6 && raw_rel > 0.01,
		"full-torus inertia must be analytic (coarse == fine == closed form):\n\
		 I_axis rel err {axis_rel:.3e}, I_perp rel err {perp_rel:.3e}, trace rel err {trace_rel:.3e},\n\
		 coarse-vs-fine trace {coarse_vs_fine:.3e}, volume rel err {vol_rel:.3e}, CoM offset {com_off:.3e},\n\
		 uncorrected 12x8 trace rel err {raw_rel:.3e} (the gap the lens correction must be closing)\n\
		 (coarse trace {trace_c}, fine trace {trace_f}, closed form {})",
		i_axis + 2.0 * i_perp
	);
}
