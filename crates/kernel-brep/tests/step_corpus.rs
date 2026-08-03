// Copyright (c) LMCAD. Licensed under the MIT License.

//! Third-party-style STEP corpus: hand-authored fixtures structured exactly as
//! OpenCascade-family exporters (FreeCAD / SolidWorks / Onshape) emit real parts —
//! AP214 product boilerplate, `ADVANCED_BREP_SHAPE_REPRESENTATION`,
//! `MANIFOLD_SOLID_BREP` → (oriented) closed shells, per-face
//! `FACE_OUTER_BOUND`+`FACE_BOUND` mixes, single-vertex full-circle edges, periodic
//! walls with `SEAM_CURVE`-wrapped seams and `PCURVE` clutter, multi-solid files.
//!
//! Every fixture must import as a closed manifold of the expected genus, shell count
//! and volume — within the stated tolerance, which is the documented import faceting
//! (48-segment conic rings ≈ 0.3% radial deficit per circular cross-section; ring
//! grids on the exact surface for sphere/torus regions).

use kernel_brep::{import_step, tessellate_default, validate, volume};

struct Expect {
	name: &'static str,
	text: &'static str,
	genus: i64,
	shells: usize,
	/// Exact analytic volume of the modelled part.
	volume: f64,
	/// Stated relative tolerance for the faceted import volume.
	tol: f64,
}

#[test]
fn corpus_imports_closed_manifold_with_expected_genus_and_volume() {
	use std::f64::consts::PI;
	let cases = [
		Expect {
			name: "fc_plate_bore",
			text: include_str!("fixtures/fc_plate_bore.step"),
			genus: 1,
			shells: 1,
			volume: 40.0 * 30.0 * 8.0 - PI * 36.0 * 8.0, // plate − bore
			tol: 0.002,
		},
		Expect {
			name: "sw_stepped_shaft",
			text: include_str!("fixtures/sw_stepped_shaft.step"),
			genus: 0,
			shells: 1,
			volume: PI * (100.0 * 12.0 + 36.0 * 18.0), // two coaxial cylinders
			tol: 0.005,
		},
		Expect {
			name: "onshape_genus2_block",
			text: include_str!("fixtures/onshape_genus2_block.step"),
			genus: 2,
			shells: 1,
			volume: 60.0 * 20.0 * 10.0 - 2.0 * PI * 16.0 * 10.0, // block − two bores
			tol: 0.002,
		},
		Expect {
			name: "fc_cone_frustum_oriented",
			text: include_str!("fixtures/fc_cone_frustum_oriented.step"),
			genus: 0,
			shells: 1,
			volume: PI * 10.0 * (64.0 + 24.0 + 9.0) / 3.0, // frustum πh(R²+Rr+r²)/3
			tol: 0.005,
		},
		Expect {
			name: "fc_multibody",
			text: include_str!("fixtures/fc_multibody.step"),
			genus: 0,
			shells: 2,
			volume: 1000.0 + PI * 16.0 * 8.0, // box + separate cylinder
			tol: 0.005,
		},
		Expect {
			name: "fc_sphere_ball",
			text: include_str!("fixtures/fc_sphere_ball.step"),
			genus: 0,
			shells: 1,
			volume: 4.0 / 3.0 * PI * 1000.0, // full sphere r=10 (one seam-bounded face)
			tol: 0.01,                       // 48×24 lat-long ring grid sits ~0.7% under
		},
		Expect {
			name: "fc_hemisphere",
			text: include_str!("fixtures/fc_hemisphere.step"),
			genus: 0,
			shells: 1,
			volume: 2.0 / 3.0 * PI * 1000.0, // pole-spanning dome + equator disk
			tol: 0.01,                       // measured deficit ≈ 0.7% (ring grid)
		},
		Expect {
			name: "fc_torus_ring",
			text: include_str!("fixtures/fc_torus_ring.step"),
			genus: 1,
			shells: 1,
			volume: 2.0 * PI * PI * 8.0 * 2.5 * 2.5, // full torus 2π²Rr², R=8 r=2.5
			tol: 0.01,                               // 48×48 ring grid sits ~0.6% under
		},
		Expect {
			name: "sw_wedge_cylinder",
			text: include_str!("fixtures/sw_wedge_cylinder.step"),
			genus: 0,
			shells: 1,
			// Cylinder r=6 cut by the plane z = 10 + x/2 (full-ELLIPSE top rim):
			// the oblique cap integrates to the height at the axis, V = πr²·10.
			volume: PI * 36.0 * 10.0,
			tol: 0.005, // 48-gon wall ≈ 0.29% radial deficit
		},
		Expect {
			name: "fc_freeform_pad",
			text: include_str!("fixtures/fc_freeform_pad.step"),
			genus: 0,
			shells: 1,
			// 24×16×6 pad under a tensor-quadratic B-spline top: x = 24u, y = 16v and
			// ∫∫ BᵢBⱼ du dv = 1/9 per control, so the bulge above z=6 integrates to
			// 24·16·(Σ z_ctrl − 6)/9 = 384·(1.5 + 4.5 + 1.5)/9 = 320.
			volume: 24.0 * 16.0 * 6.0 + 320.0,
			tol: 0.005, // measured deficit ≈ 0.29% (interior chords at the 1/8-domain pitch)
		},
		Expect {
			name: "fc_nurbs_tube",
			text: include_str!("fixtures/fc_nurbs_tube.step"),
			genus: 0,
			shells: 1,
			// A closed rational B-spline cylinder (r=6, h=10) whose wall is ONE
			// closed/periodic patch face: full-circle rational B-spline rims + a seam
			// edge traversed twice (the slit), unwrapped into the universal cover.
			volume: PI * 36.0 * 10.0,
			tol: 0.005, // 64-chord adaptive rims ≈ 0.16% radial deficit; wall interior on the exact patch
		},
		Expect {
			name: "fc_nurbs_seam_pocket",
			text: include_str!("fixtures/fc_nurbs_seam_pocket.step"),
			genus: 0,
			shells: 1,
			// A keyseat pocket milled ACROSS the seam of the closed B-spline cylinder:
			// the wall face's trim loop (252° rational arcs through the seam + axial
			// sides) crosses the patch seam and is unwrapped, not refused. Exact
			// D-section volume: h · (½r²·252° + ½r²·sin 108°).
			volume: 10.0 * (18.0 * 252.0_f64.to_radians() + 18.0 * 108.0_f64.to_radians().sin()),
			tol: 0.005, // 64-chord adaptive arcs ≈ 0.07% cap deficit
		},
		Expect {
			name: "sw_boss_fillet",
			text: include_str!("fixtures/sw_boss_fillet.step"),
			genus: 0,
			shells: 1,
			// plate 30·30·6 + boss πr²h (r=5, z 7.5..14) + the fillet collar between
			// z=6 and z=7.5: V = 1.5π·∫₀^{π/2}(6.5−1.5cosθ)²cosθ dθ
			//                  = 1.5π·(6.5² − 2·6.5·1.5·(π/4) + 1.5²·(2/3)).
			volume: 5400.0 + PI * 25.0 * 6.5 + 1.5 * PI * (42.25 - 19.5 * (PI / 4.0) + 1.5),
			tol: 0.001, // measured deficit ≈ 0.03% (48-gon boss + exact-torus fillet band)
		},
	];

	let mut failures: Vec<String> = Vec::new();
	for c in cases {
		let solid = match import_step(c.text) {
			Ok(s) => s,
			Err(e) => {
				failures.push(format!("{}: import failed: {e}", c.name));
				continue;
			}
		};
		let v = validate(&solid);
		let vol = volume(&solid).abs();
		let rel = (vol - c.volume).abs() / c.volume;
		let watertight = tessellate_default(&solid).is_watertight();
		if !(v.closed && v.manifold && v.genus == c.genus && v.shells == c.shells && rel < c.tol && watertight) {
			failures.push(format!(
				"{}: validity {v:?} (want genus {} shells {}), volume {vol:.4} vs {:.4} (rel err {rel:.5}, tol {}), watertight {watertight}",
				c.name, c.genus, c.shells, c.volume, c.tol
			));
		}
	}
	assert!(
		failures.is_empty(),
		"third-party-style corpus parts must import closed+manifold with expected genus/volume:\n{}",
		failures.join("\n")
	);
}
