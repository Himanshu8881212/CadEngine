// Copyright (c) LMCAD. Licensed under the MIT License.

//! Pinning tests for the voxel-route shell/offset (`kernel_model::shell`) —
//! the PROGRESS.md Tier-1 "shell/hollow (voxel route, honestly labeled)".
//!
//! Fixture: a Ø30×20 B-rep cylinder (r = 15, h = 20, 64 segments), voxelized
//! at 0.5 mm. Every band below is **±10 % around the closed-form analytic
//! value** — deliberately voxel-honest: the route promises voxel-accurate
//! meshes (surface within ~voxel/2 + input chord error; measured volume error
//! here is 0.1–0.2 %), so the asserts pin correctness with ~50× margin instead
//! of faking exact-kernel tolerances the route never claimed.

use kernel_core::math::DVec3;
use kernel_model::shell::{offset_mesh, offset_to_solid, shell_mesh, shell_to_solid};

const R: f64 = 15.0;
const H: f64 = 20.0;
const VOXEL: f32 = 0.5;
const PI: f64 = std::f64::consts::PI;

/// The shared fixture: Ø30×20 cylinder, base disk at the origin, axis +Z.
/// 64 segments keep the faceting deficit at 0.16 % — invisible inside ±10 %.
fn cyl() -> kernel_brep::Solid {
	kernel_brep::cylinder(DVec3::ZERO, DVec3::Z, R, H, 64)
}

#[test]
fn offset_plus_two_grows_to_the_rounded_rim_analytic_volume() {
	// A +2 offset of a convex solid is its Minkowski sum with a ball r = 2, so the
	// exact volume decomposes (Steiner/Pappus) into
	//   barrel  π(r+δ)²h            = π·17²·20      = 18 158.4
	//   slabs   2·πr²δ              = 2π·15²·2      =  2 827.4
	//   rims    2·2π(r + 4δ/3π)·πδ²/4 (two quarter-torus rim rounds) = 625.7
	//   total                                        = 21 611.5 mm³.
	// The sharp-cornered "slab+barrel" Ø34×24 figure π·17²·24 = 21 790.2 overstates
	// it by only +0.83 %, so the ±10 % band around the exact rounded-rim value
	// contains both — that band is the honest voxel tolerance, not a fudge.
	let d = 2.0_f64;
	let exact = PI * (R + d).powi(2) * H // barrel
		+ 2.0 * PI * R * R * d // cap slabs
		+ 2.0 * (2.0 * PI * (R + 4.0 * d / (3.0 * PI))) * (PI * d * d / 4.0); // rim rounds
	let sharp = PI * (R + d).powi(2) * (H + 2.0 * d);
	let m = offset_mesh(&cyl(), d, VOXEL);
	let vol = m.signed_volume();
	assert!(
		m.is_watertight() && (vol - exact).abs() / exact < 0.10,
		"offset +2 of Ø30×20 @ voxel 0.5 must be a watertight mesh within ±10% of the rounded-rim analytic volume: \
		 measured vol={vol:.1} vs exact {exact:.1} (rel err {:+.2}%; sharp Ø34×24 slab+barrel bound {sharp:.1}), watertight={}, tris={}",
		(vol - exact) / exact * 100.0,
		m.is_watertight(),
		m.triangle_count()
	);
}

#[test]
fn shell_thickness_two_hollows_the_cylinder_keeping_the_outer_surface() {
	// The shell keeps the outer Ø30×20 surface and erodes a cavity 2 mm in. The
	// erosion of a convex cylinder is the SHARP cylinder Ø26×16 (inward offsets do
	// not round on a convex body — only the voxel grid rounds the cavity's concave
	// rim at ~voxel scale), so the wall is
	//   π·15²·20 − π·13²·16 = 14 137.2 − 8 494.9 = 5 642.3 mm³,
	// which is 39.9 % of the full solid — the hollowness proof pins < 60 %.
	let full = PI * R * R * H;
	let wall = full - PI * (R - 2.0).powi(2) * (H - 4.0);
	let m = shell_mesh(&cyl(), 2.0, VOXEL);
	let vol = m.signed_volume();
	assert!(
		m.is_watertight() && (vol - wall).abs() / wall < 0.10 && vol < 0.6 * full,
		"shell t=2 of Ø30×20 @ voxel 0.5 must be a watertight hollow wall within ±10% of analytic and truly hollow: \
		 measured vol={vol:.1} vs analytic wall {wall:.1} (rel err {:+.2}%), hollowness {:.1}% of full {full:.1} (gate < 60%), watertight={}, tris={}",
		(vol - wall) / wall * 100.0,
		vol / full * 100.0,
		m.is_watertight(),
		m.triangle_count()
	);
}

#[test]
fn offset_minus_two_erodes_to_the_sharp_inner_cylinder() {
	// Erosion by 2 of the convex Ø30×20 cylinder is exactly the sharp cylinder
	// Ø26×16 (r−δ, h−2δ; no rim rounding on the eroded body): π·13²·16 = 8 494.9 mm³
	// — 60.1 % of the original, so the "negative offset shrinks" claim is pinned
	// against the full volume too.
	let full = PI * R * R * H;
	let eroded = PI * (R - 2.0).powi(2) * (H - 4.0);
	let m = offset_mesh(&cyl(), -2.0, VOXEL);
	let vol = m.signed_volume();
	assert!(
		m.is_watertight() && (vol - eroded).abs() / eroded < 0.10 && vol < full,
		"offset −2 of Ø30×20 @ voxel 0.5 must be a watertight shrunk solid within ±10% of the sharp Ø26×16 erosion: \
		 measured vol={vol:.1} vs analytic {eroded:.1} (rel err {:+.2}%; full solid {full:.1}, shrink gate vol < full), watertight={}, tris={}",
		(vol - eroded) / eroded * 100.0,
		m.is_watertight(),
		m.triangle_count()
	);
}

#[test]
fn to_solid_wraps_hollow_shell_and_eroded_offset_as_faceted_breps() {
	// The *_to_solid conveniences wrap the voxel meshes into FACETED B-reps (one
	// planar face per triangle — documented, not hidden). The pin is topological +
	// volumetric: the hollow wall must arrive as a valid TWO-shell solid (cavity
	// kept as a nested shell, not filled) whose B-rep volume sits in the same ±10 %
	// band as the mesh test above; the eroded offset must arrive as a valid
	// ONE-shell solid in its own band.
	let full = PI * R * R * H;
	let wall = full - PI * (R - 2.0).powi(2) * (H - 4.0);
	let eroded = PI * (R - 2.0).powi(2) * (H - 4.0);
	let hollow = shell_to_solid(&cyl(), 2.0, VOXEL);
	let core = offset_to_solid(&cyl(), -2.0, VOXEL);
	let (hv, cv) = (kernel_brep::validate(&hollow), kernel_brep::validate(&core));
	let (hvol, cvol) = (kernel_brep::volume(&hollow), kernel_brep::volume(&core));
	assert!(
		hv.is_valid() && hv.shells == 2
			&& (hvol - wall).abs() / wall < 0.10
			&& cv.is_valid() && cv.shells == 1
			&& (cvol - eroded).abs() / eroded < 0.10,
		"faceted-B-rep wraps must keep topology and volume: shell_to_solid(t=2) valid={} shells={} (want 2) vol={hvol:.1} vs analytic wall {wall:.1}; \
		 offset_to_solid(−2) valid={} shells={} (want 1) vol={cvol:.1} vs analytic {eroded:.1}",
		hv.is_valid(),
		hv.shells,
		cv.is_valid(),
		cv.shells
	);
}
