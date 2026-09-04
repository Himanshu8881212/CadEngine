//! The `mass_properties` full `inertia_tensor` measure (assembly-physics wave, Item A).
//!
//! The engine reports the 3×3 inertia tensor **about the center of mass at unit
//! density** (mm⁵), rows `[[Ixx,Ixy,Ixz],[Iyx,Iyy,Iyz],[Izx,Izy,Izz]]`, standard
//! dynamics convention (off-diagonals are −∫xy dV …). Verified here against the
//! analytic box tensor and the parallel-axis theorem:
//!   I_origin = I_com + V·(|d|²·Id − d·dᵀ)      (d = center of mass)
//! so for an axis-aligned box translated by d the ORIGIN-frame products must be
//! exactly −V·dx·dy etc., while the reported CoM-frame tensor stays translation-
//! invariant with ~0 off-diagonals. A box is planar-faced (machine-exact analytic
//! path), so the tolerance is relative 1e-9 — stated on every assert.

use kernel_api::{run_program, Report};
use serde_json::json;
use std::path::Path;

fn run(dir: &Path, ops: serde_json::Value) -> Report {
	run_program(&serde_json::to_string(&json!({ "ops": ops })).unwrap(), dir)
}

/// Pull `{volume, center_of_mass, inertia_diag, inertia_tensor}` for one op id.
fn mass_props(r: &Report, id: &str) -> (f64, [f64; 3], [f64; 3], [[f64; 3]; 3]) {
	let m = r
		.ops
		.iter()
		.find(|o| o.id == id)
		.and_then(|o| o.measures.as_ref())
		.unwrap_or_else(|| panic!("op '{id}' must return measures — {r:#?}"));
	let v3 = |key: &str| -> [f64; 3] {
		let a = m[key].as_array().unwrap_or_else(|| panic!("'{key}' must be an array — {m:#}"));
		[a[0].as_f64().unwrap(), a[1].as_f64().unwrap(), a[2].as_f64().unwrap()]
	};
	let t = m["inertia_tensor"].as_array().unwrap_or_else(|| panic!("'inertia_tensor' must be a 3×3 array — {m:#}"));
	let row = |i: usize| -> [f64; 3] {
		let a = t[i].as_array().unwrap();
		[a[0].as_f64().unwrap(), a[1].as_f64().unwrap(), a[2].as_f64().unwrap()]
	};
	(m["volume"].as_f64().expect("volume"), v3("center_of_mass"), v3("inertia_diag"), [row(0), row(1), row(2)])
}

#[test]
fn inertia_tensor_is_com_frame_and_parallel_axis_consistent() {
	let dir = std::env::temp_dir().join(format!("cadcode_inertia_tensor_{}", std::process::id()));
	std::fs::create_dir_all(&dir).unwrap();

	// A 10×6×4 box centered on the origin, and the SAME box translated by d = (5, 7, 9).
	let (a, b, c) = (10.0_f64, 6.0_f64, 4.0_f64);
	let d = [5.0_f64, 7.0, 9.0];
	let r = run(
		&dir,
		json!([
			{"id":"centered", "op":"box", "min":[-a/2.0,-b/2.0,-c/2.0], "max":[a/2.0,b/2.0,c/2.0]},
			{"id":"moved", "op":"translate", "in":"centered", "offset": d},
			{"id":"mp0", "op":"mass_properties", "in":"centered"},
			{"id":"mp1", "op":"mass_properties", "in":"moved"},
		]),
	);
	assert!(r.ok, "program must succeed — {r:#?}");
	let (v0, com0, diag0, t0) = mass_props(&r, "mp0");
	let (v1, com1, _diag1, t1) = mass_props(&r, "mp1");

	// Analytic CoM-frame tensor of an a×b×c box at unit density (products all zero).
	let vol = a * b * c;
	let ixx = vol * (b * b + c * c) / 12.0;
	let iyy = vol * (a * a + c * c) / 12.0;
	let izz = vol * (a * a + b * b) / 12.0;
	let scale = izz; // largest analytic entry — the relative-tolerance yardstick
	let tol = 1e-9 * scale; // planar-exact analytic path ⇒ relative 1e-9 (stated)

	// (1) Centered box: diagonal analytic, off-diagonals ~0, tensor diag == inertia_diag.
	let analytic0 = [[ixx, 0.0, 0.0], [0.0, iyy, 0.0], [0.0, 0.0, izz]];
	let mut worst0 = 0.0_f64;
	for i in 0..3 {
		for j in 0..3 {
			worst0 = worst0.max((t0[i][j] - analytic0[i][j]).abs());
		}
	}
	assert!(
		worst0 <= tol
			&& (v0 - vol).abs() <= 1e-9 * vol
			&& com0.iter().all(|x| x.abs() <= 1e-9)
			&& (0..3).all(|i| (t0[i][i] - diag0[i]).abs() <= tol),
		"centered {a}×{b}×{c} box must report the analytic CoM-frame tensor \
		 diag=({ixx:.6},{iyy:.6},{izz:.6}) with ~0 off-diagonals and a matching inertia_diag \
		 (tol {tol:.3e} = 1e-9·Izz): got volume {v0}, com {com0:?}, tensor {t0:?}"
	);

	// (2) Translated box: the CoM-frame tensor is translation-invariant (still ~0 products) …
	let mut drift = 0.0_f64;
	for i in 0..3 {
		for j in 0..3 {
			drift = drift.max((t1[i][j] - t0[i][j]).abs());
		}
	}
	assert!(
		drift <= tol && (0..3).all(|i| (com1[i] - d[i]).abs() <= 1e-9 * scale),
		"the CoM-frame tensor must be translation-invariant (tol {tol:.3e}): \
		 centered {t0:?} vs moved {t1:?} (max drift {drift:.3e}), com {com1:?} vs d {d:?}"
	);

	// (3) … and the parallel-axis reconstruction I_origin = I_com + V·(|d|²·Id − d·dᵀ)
	// must match the ANALYTIC origin-frame tensor: products exactly −V·di·dj.
	let d2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
	let mut worst_pa = 0.0_f64;
	for i in 0..3 {
		for j in 0..3 {
			let kron = if i == j { 1.0 } else { 0.0 };
			let reconstructed = t1[i][j] + v1 * (d2 * kron - d[i] * d[j]);
			let analytic = analytic0[i][j] + vol * (d2 * kron - d[i] * d[j]);
			worst_pa = worst_pa.max((reconstructed - analytic).abs());
		}
	}
	assert!(
		worst_pa <= 1e-9 * (scale + vol * d2),
		"parallel-axis origin-frame tensor must match the analytic formula \
		 (products −V·di·dj; tol 1e-9·(Izz+V·|d|²) = {:.3e}): worst deviation {worst_pa:.3e}, \
		 moved tensor {t1:?}, V {v1}, d {d:?}",
		1e-9 * (scale + vol * d2)
	);

	let _ = std::fs::remove_dir_all(&dir);
}
