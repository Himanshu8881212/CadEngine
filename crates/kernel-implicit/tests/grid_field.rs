// Copyright (c) LMCAD. Licensed under the MIT License.

//! `GridField` — the simulation→geometry bridge. Two gates:
//! 1. the hand-rolled NPY parser + trilinear sampler are exact (corners,
//!    dyadic center, border clamp, f8 narrowing, fortran/dtype refusals);
//! 2. the CLOSED LOOP: a stress-ramp grid, serialized as real NPY bytes,
//!    normalized to density and fed to the kernel's EXISTING grade mechanism
//!    (`Node::offset_by`) on a gyroid, measurably thickens the high-stress
//!    half of the meshed lattice versus the low-stress half.

use kernel_implicit::grid_field::GridField;
use kernel_implicit::{manifold_dual_contour, Aabb, Cuboid, Gyroid, Node, Resolution, ScalarField, Sdf, Vec3};

/// Serialize an NPY buffer the way `numpy.save` does (v1: u16 header length,
/// v2: u32), space-padded to 64-byte alignment with a trailing newline.
fn npy_bytes(version: u8, descr: &str, fortran: bool, shape: &str, payload: &[u8]) -> Vec<u8> {
	let dict = format!("{{'descr': '{descr}', 'fortran_order': {}, 'shape': {shape}, }}", if fortran { "True" } else { "False" });
	let mut out = b"\x93NUMPY".to_vec();
	out.push(version);
	out.push(0);
	let preamble = if version == 1 { 10 } else { 12 };
	let mut header = dict.into_bytes();
	while (preamble + header.len() + 1) % 64 != 0 {
		header.push(b' ');
	}
	header.push(b'\n');
	if version == 1 {
		out.extend_from_slice(&(header.len() as u16).to_le_bytes());
	} else {
		out.extend_from_slice(&(header.len() as u32).to_le_bytes());
	}
	out.extend_from_slice(&header);
	out.extend_from_slice(payload);
	out
}

fn f4_payload(vals: &[f32]) -> Vec<u8> {
	vals.iter().flat_map(|v| v.to_le_bytes()).collect()
}

#[test]
fn npy_parse_and_trilinear_sampling_are_exact() {
	// 2×2×2 grid, C-order over (nx, ny, nz): value(i, j, k) = 4i + 2j + k —
	// linear in the index, so trilinear interpolation must reproduce it
	// EXACTLY (all weights here are dyadic; no f32 rounding slack is needed).
	let vals: Vec<f32> = (0..8).map(|v| v as f32).collect();
	let g = GridField::from_npy_bytes(&npy_bytes(1, "<f4", false, "(2, 2, 2)", &f4_payload(&vals)), Vec3::ZERO, 1.0)
		.expect("v1 <f4 2x2x2 must parse");

	let mut ok = true;
	let mut report = String::new();
	let mut check = |name: &str, got: f32, want: f32| {
		let pass = got == want;
		ok &= pass;
		report += &format!("\n  {name}: got {got}, want {want} exactly {}", if pass { "[ok]" } else { "[FAIL]" });
	};

	// Corners hit stored values exactly.
	check("corner (0,0,0)", g.sample(Vec3::ZERO), 0.0);
	check("corner (1,0,0)", g.sample(Vec3::X), 4.0);
	check("corner (0,1,0)", g.sample(Vec3::Y), 2.0);
	check("corner (0,0,1)", g.sample(Vec3::Z), 1.0);
	check("corner (1,1,1)", g.sample(Vec3::ONE), 7.0);
	// Dyadic center: mean of all eight corners.
	check("trilinear center (.5,.5,.5)", g.sample(Vec3::splat(0.5)), 3.5);
	// Border clamp: outside → nearest border value (constant extrapolation).
	check("clamp far below (-3,-4,-5)", g.sample(Vec3::new(-3.0, -4.0, -5.0)), 0.0);
	check("clamp far above (9,9,9)", g.sample(Vec3::splat(9.0)), 7.0);
	check("clamp mixed (.5,-2,7)", g.sample(Vec3::new(0.5, -2.0, 7.0)), 3.0); // x mid of (0,0,1)=1 and (1,0,1)=5

	// '<f8' narrows losslessly for these values and samples identically.
	let payload8: Vec<u8> = vals.iter().flat_map(|v| (*v as f64).to_le_bytes()).collect();
	let g8 = GridField::from_npy_bytes(&npy_bytes(1, "<f8", false, "(2, 2, 2)", &payload8), Vec3::ZERO, 1.0).expect("v1 <f8 must parse");
	check("f8 center", g8.sample(Vec3::splat(0.5)), 3.5);
	check("f8 corner (1,1,1)", g8.sample(Vec3::ONE), 7.0);

	// v2 header (u32 length field) parses to the same field.
	let g2 = GridField::from_npy_bytes(&npy_bytes(2, "<f4", false, "(2, 2, 2)", &f4_payload(&vals)), Vec3::ZERO, 1.0)
		.expect("v2 <f4 must parse");
	check("v2 center", g2.sample(Vec3::splat(0.5)), 3.5);

	// The ScalarField adapter is the same sampler behind the ops-facing type.
	let sf: ScalarField = g.clone().into_scalar_field();
	check("into_scalar_field center", sf(Vec3::splat(0.5)), 3.5);
	// normalized(): affine to [0,1], clamped; frame unchanged.
	let n = g.normalized(0.0, 7.0);
	check("normalized (1,1,1)", n.sample(Vec3::ONE), 1.0);
	check("normalized (0,0,0)", n.sample(Vec3::ZERO), 0.0);
	check("normalized range lo", n.value_range().0, 0.0);
	check("normalized range hi", n.value_range().1, 1.0);

	// Honest refusals, not silent misreads.
	let fortran_err = GridField::from_npy_bytes(&npy_bytes(1, "<f4", true, "(2, 2, 2)", &f4_payload(&vals)), Vec3::ZERO, 1.0)
		.expect_err("fortran_order=True must be refused");
	let dtype_err = GridField::from_npy_bytes(&npy_bytes(1, "<i4", false, "(2, 2, 2)", &[0u8; 32]), Vec3::ZERO, 1.0)
		.expect_err("'<i4' must be refused");
	let short_err = GridField::from_npy_bytes(&npy_bytes(1, "<f4", false, "(2, 2, 2)", &f4_payload(&vals[..7])), Vec3::ZERO, 1.0)
		.expect_err("short payload must be refused");
	for (name, err, needle) in [
		("fortran refusal", &fortran_err, "fortran_order"),
		("dtype refusal", &dtype_err, "<f4"),
		("short-payload refusal", &short_err, "needs exactly"),
	] {
		let pass = err.contains(needle);
		ok &= pass;
		report += &format!("\n  {name}: {err:?} (must mention {needle:?}) {}", if pass { "[ok]" } else { "[FAIL]" });
	}

	assert!(ok, "GridField NPY parse / trilinear sample snapshot:{report}");
}

#[test]
fn stress_ramp_npy_grades_gyroid_thicker_in_high_stress_half() {
	// The closed simulation→geometry loop, end to end on the wire format:
	// a synthetic von-Mises ramp (0 → 80 "MPa" along +x), serialized as the
	// same v1 '<f4' C-order NPY that tools/ace_fea_runner.py writes, parsed
	// back, normalized to density, and applied through the kernel's EXISTING
	// grade mechanism — `Node::offset_by` on a gyroid (the very closure form
	// kernel-model's LinearGrade compiles to). The meshed lattice must hold
	// measurably more solid in the high-stress half than the low-stress half.
	let region = Aabb::new(Vec3::new(-15.0, -15.0, 0.0), Vec3::new(15.0, 15.0, 20.0));
	let origin = Vec3::new(-15.0, -15.0, 0.0);
	let (nx, ny, nz) = (7usize, 7usize, 5usize);
	let cell = 5.0f32;
	let mut stress = Vec::with_capacity(nx * ny * nz);
	for i in 0..nx {
		for _j in 0..ny {
			for _k in 0..nz {
				stress.push(80.0 * (i as f32 * cell) / 30.0); // linear in x only
			}
		}
	}
	let bytes = npy_bytes(1, "<f4", false, "(7, 7, 5)", &f4_payload(&stress));
	let field = GridField::from_npy_bytes(&bytes, origin, cell).expect("ramp NPY must parse");

	// Density 0 → −0.25 mm (thin the idle walls), density 1 → +0.25 mm
	// (inflate the hot walls): the damper recipe of DESIGN_GUIDE §16.8, fed
	// from data instead of a declarative LinearGrade.
	let law: ScalarField = field.normalized(0.0, 80.0).into_grade_law(-0.25, 0.25);
	let (l_lo, l_mid, l_hi) = (law(Vec3::new(-15.0, 0.0, 10.0)), law(Vec3::new(0.0, 0.0, 10.0)), law(Vec3::new(15.0, 0.0, 10.0)));

	// Mesh each half with Manifold Dual Contouring: manifold by construction,
	// so watertightness is assertable (plain surface_nets leaves a handful of
	// non-manifold pinch edges at graded-TPMS saddles at this resolution).
	let half_volume = |x0: f32, x1: f32| {
		let node = Node::primitive_bound(Gyroid::new(region, 0.55, 1.3))
			.offset_by(law.clone(), 0.3)
			.intersection(Node::primitive(Cuboid::from_corners(Vec3::new(x0, -15.0, 0.0), Vec3::new(x1, 15.0, 20.0))));
		let mesh = manifold_dual_contour(&node, node.bounds().pad(1.0), Resolution::VoxelSize(0.25));
		(mesh.signed_volume(), mesh.is_watertight())
	};
	let (vol_lo, wt_lo) = half_volume(-15.0, 0.0);
	let (vol_hi, wt_hi) = half_volume(0.0, 15.0);
	let ratio = vol_hi / vol_lo.max(1e-9);

	// Measured on landing: 3445 mm³ vs 4888 mm³, ratio 1.42; the 1.30 floor
	// leaves real margin without weakening the direction-of-effect claim.
	assert!(
		l_lo == -0.25 && l_mid == 0.0 && l_hi == 0.25 && wt_lo && wt_hi && vol_lo > 500.0 && ratio >= 1.30,
		"stress→density→grade→mesh loop: law endpoints {l_lo}/{l_mid}/{l_hi} mm (want -0.25/0/0.25); \
		 low-stress-half volume {vol_lo:.0} mm³ (watertight={wt_lo}) vs high-stress-half {vol_hi:.0} mm³ \
		 (watertight={wt_hi}), ratio {ratio:.2} (want ≥ 1.30, measured 1.42 on landing — the graded \
		 lattice must be measurably denser where the FEA field says the part works hardest)"
	);
}
