// Copyright (c) LMCAD. Licensed under the MIT License.

//! **ISO 286 limits and fits**: the hole-basis preferred fits (H7/g6, H7/h6, H7/k6,
//! H7/n6, H7/p6, H7/s6, H8/f7) resolved to numeric limit deviations for nominal sizes
//! up to 120 mm — the lookup an agent needs to turn "Ø8 H7/g6" into actual bore and
//! shaft limits before modelling or toleranced manufacture. Pure table math: no
//! geometry is built here; results are **deviations from the nominal diameter in mm**
//! (µm-grade values, so e.g. +0.021 mm).

/// Resolved ISO 286 fit limits for one nominal diameter. All values are deviations
/// from the nominal in **mm**; `(lower, upper)` with `lower ≤ upper`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FitLimits {
	/// Hole (bore) deviation band — e.g. H7 on Ø25 is `(0.0, +0.021)`.
	pub hole: (f64, f64),
	/// Shaft deviation band — e.g. g6 on Ø25 is `(−0.020, −0.007)`.
	pub shaft: (f64, f64),
	/// Assembly clearance band `(min, max)` = `(hole.0 − shaft.1, hole.1 − shaft.0)`;
	/// negative values are **interference**.
	pub clearance: (f64, f64),
}

/// One ISO 286 diameter-range row, all tolerance values in **µm**. Ranges are the
/// standard "over a, up to and including `to`" steps, split at 65 and 100 mm where
/// the s-column changes mid-step.
struct Iso286Row {
	/// Range upper bound (mm): the row covers `previous.to < d ≤ to`.
	to: f64,
	/// Standard tolerance grade IT5 (the bearing-seat shaft grade).
	it5: f64,
	/// Standard tolerance grades IT6 / IT7 / IT8.
	it: [f64; 3],
	/// Standard tolerance grades IT9 / IT11 (the loose running / free fits).
	it_loose: [f64; 2],
	/// Upper deviation `es` of shaft c (loose running; the 30–50 and 50–80
	/// steps are split at 40 and 65 in the standard — see `c_es_split`).
	c_es: f64,
	/// Upper deviation `es` of shaft d (free running).
	d_es: f64,
	/// Upper deviation `es` of shaft g (clearance side, negative).
	g_es: f64,
	/// Upper deviation `es` of shaft f.
	f_es: f64,
	/// Upper deviation `es` of shaft j, grade 5 (bearing seats; `ei = es − IT5`).
	j5_es: f64,
	/// Upper deviation `es` of shaft j, grade 6 (`ei = es − IT6`).
	j6_es: f64,
	/// Lower deviation `ei` of shaft k, grades 4–7 (transition side).
	k_ei: f64,
	/// Lower deviation `ei` of shaft n.
	n_ei: f64,
	/// Lower deviation `ei` of shaft p.
	p_ei: f64,
	/// Lower deviation `ei` of shaft s.
	s_ei: f64,
}

/// The ISO 286 table, ≤ 120 mm. Sources: ISO 286-1:2010 Table 1 (standard tolerance
/// grades IT5–IT8) and ISO 286-2:2010 fundamental-deviation tables for shafts
/// f/g/j/k/n/p/s, as republished in the common limits-and-fits charts (e.g.
/// amesweb.info/fits-tolerances, Machinery's Handbook "Preferred Metric Limits and
/// Fits"). The j5/j6 columns (2026-09-05, ENGINE #12) are the tabulated
/// upper deviations; each row was cross-checked against `es − ei = IT` for its
/// grade. The K/N/P HOLE deviations are not tabulated: ISO 286-1 §4.3 derives
/// them from the same-letter shaft by `ES = −ei + Δ`, `Δ = IT(n) − IT(n−1)`,
/// which `iso286_fit` applies (and which reproduces every published N7/P7/K7
/// row ≤ 120 mm — e.g. Ø25 N7 = −7/−28, P7 = −14/−35, K7 = +6/−15).
#[rustfmt::skip]
const ISO286: [Iso286Row; 10] = [
	Iso286Row { to: 3.0,   it5: 4.0, it: [6.0, 10.0, 14.0],  it_loose: [25.0, 60.0],  c_es: -60.0,  d_es: -20.0,  g_es: -2.0,  f_es: -6.0,  j5_es: 2.0, j6_es: 4.0, k_ei: 0.0, n_ei: 4.0,  p_ei: 6.0,  s_ei: 14.0 },
	Iso286Row { to: 6.0,   it5: 5.0, it: [8.0, 12.0, 18.0],  it_loose: [30.0, 75.0],  c_es: -70.0,  d_es: -30.0,  g_es: -4.0,  f_es: -10.0, j5_es: 3.0, j6_es: 6.0, k_ei: 1.0, n_ei: 8.0,  p_ei: 12.0, s_ei: 19.0 },
	Iso286Row { to: 10.0,  it5: 6.0, it: [9.0, 15.0, 22.0],  it_loose: [36.0, 90.0],  c_es: -80.0,  d_es: -40.0,  g_es: -5.0,  f_es: -13.0, j5_es: 4.0, j6_es: 7.0, k_ei: 1.0, n_ei: 10.0, p_ei: 15.0, s_ei: 23.0 },
	Iso286Row { to: 18.0,  it5: 8.0, it: [11.0, 18.0, 27.0], it_loose: [43.0, 110.0], c_es: -95.0,  d_es: -50.0,  g_es: -6.0,  f_es: -16.0, j5_es: 5.0, j6_es: 8.0, k_ei: 1.0, n_ei: 12.0, p_ei: 18.0, s_ei: 28.0 },
	Iso286Row { to: 30.0,  it5: 9.0, it: [13.0, 21.0, 33.0], it_loose: [52.0, 130.0], c_es: -110.0, d_es: -65.0,  g_es: -7.0,  f_es: -20.0, j5_es: 5.0, j6_es: 9.0, k_ei: 2.0, n_ei: 15.0, p_ei: 22.0, s_ei: 35.0 },
	Iso286Row { to: 50.0,  it5: 11.0, it: [16.0, 25.0, 39.0], it_loose: [62.0, 160.0], c_es: -120.0, d_es: -80.0,  g_es: -9.0,  f_es: -25.0, j5_es: 6.0, j6_es: 11.0, k_ei: 2.0, n_ei: 17.0, p_ei: 26.0, s_ei: 43.0 },
	Iso286Row { to: 65.0,  it5: 13.0, it: [19.0, 30.0, 46.0], it_loose: [74.0, 190.0], c_es: -140.0, d_es: -100.0, g_es: -10.0, f_es: -30.0, j5_es: 6.0, j6_es: 12.0, k_ei: 2.0, n_ei: 20.0, p_ei: 32.0, s_ei: 53.0 },
	Iso286Row { to: 80.0,  it5: 13.0, it: [19.0, 30.0, 46.0], it_loose: [74.0, 190.0], c_es: -150.0, d_es: -100.0, g_es: -10.0, f_es: -30.0, j5_es: 6.0, j6_es: 12.0, k_ei: 2.0, n_ei: 20.0, p_ei: 32.0, s_ei: 59.0 },
	Iso286Row { to: 100.0, it5: 15.0, it: [22.0, 35.0, 54.0], it_loose: [87.0, 220.0], c_es: -170.0, d_es: -120.0, g_es: -12.0, f_es: -36.0, j5_es: 6.0, j6_es: 13.0, k_ei: 3.0, n_ei: 23.0, p_ei: 37.0, s_ei: 71.0 },
	Iso286Row { to: 120.0, it5: 15.0, it: [22.0, 35.0, 54.0], it_loose: [87.0, 220.0], c_es: -180.0, d_es: -120.0, g_es: -12.0, f_es: -36.0, j5_es: 6.0, j6_es: 13.0, k_ei: 3.0, n_ei: 23.0, p_ei: 37.0, s_ei: 79.0 },
];

/// The c-shaft upper deviation splits the 30–50 step at 40 mm (−120 / −130):
/// the row above carries the 30–40 value; over 40 up to 50 it is −130 µm.
fn c_es_split(d: f64, row_c_es: f64) -> f64 {
	if d > 40.0 && d <= 50.0 {
		-130.0
	} else {
		row_c_es
	}
}

/// Resolve a nominal diameter `d` (0 < d ≤ 120 mm) and one of the ISO 286
/// **preferred fits** into numeric [`FitLimits`]. Hole-basis: `"H11/c11"` (loose
/// running), `"H9/d9"` (free running), `"H8/f7"` (close running), `"H7/g6"`
/// (sliding), `"H7/h6"` (locational clearance), `"H7/k6"` (transition), `"H7/n6"`
/// (transition/light press), `"H7/p6"` (press), `"H7/s6"` (medium drive).
/// Shaft-basis clearance fits: `"C11/h11"`, `"D9/h9"`, `"F8/h7"`, `"G7/h6"` (the
/// mirror-letter rule, exact for a–h). Bearing-seat fits (ENGINE #12): shafts
/// `"H6/j5"`, `"H6/k5"`, `"H7/j6"` (j/k against an H bore — the bearing's own
/// bore band is the ISO 492 class, use H6 as its proxy) and housings
/// `"K7/h6"`, `"N7/h6"`, `"P7/h6"` (the K/N/P holes by the Δ rule,
/// `ES = −ei(shaft letter) + IT7 − IT6`). Case-insensitive. `None` for any
/// other fit string (the heavy H7/u6 and the S-hole fits stay outside this
/// table — stated, not guessed) or a diameter outside `(0, 120]`.
pub fn iso286_fit(d: f64, fit: &str) -> Option<FitLimits> {
	if !(d > 0.0 && d <= 120.0) {
		return None; // NaN-safe: the conjunction refuses NaN diameters too
	}
	let r = ISO286.iter().find(|row| d <= row.to)?;
	let [it6, it7, it8] = r.it;
	let [it9, it11] = r.it_loose;
	let c_es = c_es_split(d, r.c_es);
	let delta = if d <= 3.0 { 0.0 } else { it7 - it6 };
	// Holes: H = zero fundamental deviation, band (0, +IT). Shafts: c/d/g/f hang
	// their IT band below the upper deviation es; k/n/p/s stand it above the
	// lower ei. Shaft-basis: h = (−IT, 0); a hole letter C/D/F/G is the mirror of
	// its shaft letter (EI = −es, band above), exact for the clearance letters
	// a–h (ISO 286-1 §4.3 — the Δ correction applies only to K…ZC holes, which
	// this table does not carry).
	let (hole_um, shaft_um) = match fit.to_ascii_lowercase().as_str() {
		// hole-basis preferred fits
		"h11/c11" => ((0.0, it11), (c_es - it11, c_es)),
		"h9/d9" => ((0.0, it9), (r.d_es - it9, r.d_es)),
		"h8/f7" => ((0.0, it8), (r.f_es - it7, r.f_es)),
		"h7/g6" => ((0.0, it7), (r.g_es - it6, r.g_es)),
		"h7/h6" => ((0.0, it7), (-it6, 0.0)),
		"h7/k6" => ((0.0, it7), (r.k_ei, r.k_ei + it6)),
		"h7/n6" => ((0.0, it7), (r.n_ei, r.n_ei + it6)),
		"h7/p6" => ((0.0, it7), (r.p_ei, r.p_ei + it6)),
		"h7/s6" => ((0.0, it7), (r.s_ei, r.s_ei + it6)),
		// shaft-basis preferred clearance fits (mirror letters)
		"c11/h11" => ((-c_es, -c_es + it11), (-it11, 0.0)),
		"d9/h9" => ((-r.d_es, -r.d_es + it9), (-it9, 0.0)),
		"f8/h7" => ((-r.f_es, -r.f_es + it8), (-it7, 0.0)),
		"g7/h6" => ((-r.g_es, -r.g_es + it7), (-it6, 0.0)),
		// Bearing seats — shafts j5/k5/j6 against an H bore (ENGINE #12).
		"h6/j5" => ((0.0, it6), (r.j5_es - r.it5, r.j5_es)),
		"h6/k5" => ((0.0, it6), (r.k_ei, r.k_ei + r.it5)),
		"h7/j6" => ((0.0, it7), (r.j6_es - it6, r.j6_es)),
		// Bearing housings — K7/N7/P7 holes by ISO 286-1 §4.3: ES = −ei + Δ, Δ = IT7 − IT6
		// (Δ applies ABOVE 3 mm; at ≤ 3 mm the hole is the plain mirror: Δ = 0).
		"k7/h6" => ((-r.k_ei + delta - it7, -r.k_ei + delta), (-it6, 0.0)),
		"n7/h6" => ((-r.n_ei + delta - it7, -r.n_ei + delta), (-it6, 0.0)),
		"p7/h6" => ((-r.p_ei + delta - it7, -r.p_ei + delta), (-it6, 0.0)),
		_ => return None,
	};
	let mm = |um: (f64, f64)| (um.0 * 1e-3, um.1 * 1e-3);
	let (hole, shaft) = (mm(hole_um), mm(shaft_um));
	Some(FitLimits { hole, shaft, clearance: (hole.0 - shaft.1, hole.1 - shaft.0) })
}

#[cfg(test)]
mod tests {
	use super::*;

	/// `(hole hi, shaft lo, shaft hi, clearance min, clearance max)` in µm, rounded —
	/// the shape the published fit charts print.
	fn um(d: f64, fit: &str) -> Vec<i64> {
		let f = iso286_fit(d, fit).expect("supported fit");
		[f.hole.1, f.shaft.0, f.shaft.1, f.clearance.0, f.clearance.1].iter().map(|v| (v * 1e3).round() as i64).collect()
	}

	#[test]
	fn preferred_fits_reproduce_the_published_iso286_chart_values() {
		// One snapshot across the chart, checked against the published hole-basis
		// tables: Ø25 H7/g6 (+21 hole; −20/−7 shaft; clearance +7..+41), Ø25 H8/f7
		// (+33; −41/−20; +20..+74), Ø40 H7/p6 (+25; +26/+42; −42..−1 interference),
		// Ø10 H7/s6 (+15; +23/+32; −32..−8), Ø60 H7/s6 (s splits at 65: +30; +53/+72;
		// −72..−23), Ø70 H7/s6 (next split: +59/+78), Ø8 H7/k6 (+15; +1/+10; −10..+14
		// transition), Ø3 H7/h6 (+10; −6/0; 0..+16) — plus case-insensitivity and the
		// refusals (unsupported fit string, d > 120, d = 0, NaN).
		let chart: Vec<Vec<i64>> = [
			(25.0, "H7/g6"),
			(25.0, "H8/f7"),
			(40.0, "H7/p6"),
			(10.0, "H7/s6"),
			(60.0, "H7/s6"),
			(70.0, "H7/s6"),
			(8.0, "H7/k6"),
			(3.0, "H7/h6"),
		]
		.iter()
		.map(|&(d, fit)| um(d, fit))
		.collect();
		assert_eq!(
			chart,
			vec![
				vec![21, -20, -7, 7, 41],
				vec![33, -41, -20, 20, 74],
				vec![25, 26, 42, -42, -1],
				vec![15, 23, 32, -32, -8],
				vec![30, 53, 72, -72, -23],
				vec![30, 59, 78, -78, -29],
				vec![15, 1, 10, -10, 14],
				vec![10, -6, 0, 0, 16],
			],
			"ISO 286 preferred-fit chart values (µm: hole hi, shaft lo/hi, clearance min/max)"
		);
		assert!(
			iso286_fit(25.0, "h7/G6") == iso286_fit(25.0, "H7/g6")
				&& iso286_fit(25.0, "H7/u6").is_none()
				&& iso286_fit(125.0, "H7/g6").is_none()
				&& iso286_fit(0.0, "H7/g6").is_none()
				&& iso286_fit(f64::NAN, "H7/g6").is_none(),
			"case-insensitive lookup; unsupported fit, out-of-range and NaN diameters refused"
		);
	}

	#[test]
	fn bearing_seat_fits_reproduce_the_published_k_j_n_p_rows() {
		// Ø25 (18–30 mm step): the rows every bearing-fit chart prints, in mm.
		let um = |v: f64| (v * 1000.0).round();
		let f = iso286_fit(25.0, "H6/k5").expect("H6/k5");
		assert_eq!((um(f.shaft.0), um(f.shaft.1)), (2.0, 11.0), "k5 = +2/+11 µm");
		let f = iso286_fit(25.0, "H6/j5").expect("H6/j5");
		assert_eq!((um(f.shaft.0), um(f.shaft.1)), (-4.0, 5.0), "j5 = −4/+5 µm");
		let f = iso286_fit(25.0, "H7/j6").expect("H7/j6");
		assert_eq!((um(f.shaft.0), um(f.shaft.1)), (-4.0, 9.0), "j6 = −4/+9 µm");
		let f = iso286_fit(25.0, "K7/h6").expect("K7/h6");
		assert_eq!((um(f.hole.0), um(f.hole.1)), (-15.0, 6.0), "K7 = +6/−15 µm");
		let f = iso286_fit(25.0, "N7/h6").expect("N7/h6");
		assert_eq!((um(f.hole.0), um(f.hole.1)), (-28.0, -7.0), "N7 = −7/−28 µm");
		let f = iso286_fit(25.0, "P7/h6").expect("P7/h6");
		assert_eq!((um(f.hole.0), um(f.hole.1)), (-35.0, -14.0), "P7 = −14/−35 µm");
		// Ø8 and Ø60: the Δ rule reproduces the chart on the small and large steps too.
		let f = iso286_fit(8.0, "N7/h6").expect("N7/h6 Ø8");
		assert_eq!((um(f.hole.0), um(f.hole.1)), (-19.0, -4.0), "Ø8 N7 = −4/−19 µm");
		let f = iso286_fit(60.0, "P7/h6").expect("P7/h6 Ø60");
		assert_eq!((um(f.hole.0), um(f.hole.1)), (-51.0, -21.0), "Ø60 P7 = −21/−51 µm");
		let f = iso286_fit(60.0, "H6/j5").expect("H6/j5 Ø60");
		assert_eq!((um(f.shaft.0), um(f.shaft.1)), (-7.0, 6.0), "Ø60 j5 = −7/+6 µm");
	}

	#[test]
	fn every_fit_keeps_ordered_bands_across_the_whole_diameter_table() {
		// Structural property over all rows × fits: lower ≤ upper on hole, shaft and
		// clearance; hole lower is exactly 0 (hole basis); and the fit families keep
		// their character everywhere — g6/h6/f7 never interfere (min clearance ≥ 0),
		// s6 always interferes (max clearance < 0), k6/n6 straddle (transition), and
		// p6 is the locational-interference borderline: per the published chart its
		// max fit is +4 µm at the ≤3 mm step (line contact) and ≤ 0 everywhere above.
		let mut violations: Vec<String> = Vec::new();
		for row in &ISO286 {
			let d = row.to; // probe at each range's upper bound
			for fit in [
				"H7/g6", "H7/h6", "H7/k6", "H7/n6", "H7/p6", "H7/s6", "H8/f7", "H9/d9", "H11/c11", "C11/h11", "D9/h9", "F8/h7", "G7/h6",
				"H6/j5", "H6/k5", "H7/j6", "K7/h6", "N7/h6", "P7/h6",
			] {
				let f = iso286_fit(d, fit).expect("in range");
				// The basis member carries the zero line: an H hole's EI, an h shaft's es.
				let anchored = if fit.starts_with('H') { f.hole.0 == 0.0 } else { f.shaft.1 == 0.0 };
				let ordered = f.hole.0 <= f.hole.1 && f.shaft.0 <= f.shaft.1 && f.clearance.0 <= f.clearance.1 && anchored;
				let character = match fit {
					"H7/g6" | "H7/h6" | "H8/f7" | "H9/d9" | "H11/c11" | "C11/h11" | "D9/h9" | "F8/h7" | "G7/h6" => f.clearance.0 >= 0.0,
					"H7/s6" => f.clearance.1 < 0.0,
					// P7/h6 is the locational-interference press fit: its max fit is 0 (line
					// contact) on the small steps and negative above, like H7/p6.
					"P7/h6" => f.clearance.0 < 0.0 && f.clearance.1 <= 1e-12,
					"H7/p6" => f.clearance.0 < 0.0 && f.clearance.1 <= if d <= 3.0 { 4.0e-3 + 1e-12 } else { 0.0 },
					_ => f.clearance.0 < 0.0 && f.clearance.1 > 0.0, // k/n/j shafts, K7/N7 housings: transition
				};
				if !(ordered && character) {
					violations.push(format!("Ø{d} {fit}: {f:?}"));
				}
			}
		}
		assert!(violations.is_empty(), "every fit must keep ordered bands and its clearance character; violations: {violations:#?}");
	}
}
