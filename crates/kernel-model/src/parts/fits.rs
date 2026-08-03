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
	/// Standard tolerance grades IT6 / IT7 / IT8.
	it: [f64; 3],
	/// Upper deviation `es` of shaft g (clearance side, negative).
	g_es: f64,
	/// Upper deviation `es` of shaft f.
	f_es: f64,
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
/// grades IT6–IT8) and ISO 286-2:2010 fundamental-deviation tables for shafts
/// f/g/k/n/p/s, as republished in the common limits-and-fits charts (e.g.
/// amesweb.info/fits-tolerances, Machinery's Handbook "Preferred Metric Limits and
/// Fits").
#[rustfmt::skip]
const ISO286: [Iso286Row; 10] = [
	Iso286Row { to: 3.0,   it: [6.0, 10.0, 14.0],  g_es: -2.0,  f_es: -6.0,  k_ei: 0.0, n_ei: 4.0,  p_ei: 6.0,  s_ei: 14.0 },
	Iso286Row { to: 6.0,   it: [8.0, 12.0, 18.0],  g_es: -4.0,  f_es: -10.0, k_ei: 1.0, n_ei: 8.0,  p_ei: 12.0, s_ei: 19.0 },
	Iso286Row { to: 10.0,  it: [9.0, 15.0, 22.0],  g_es: -5.0,  f_es: -13.0, k_ei: 1.0, n_ei: 10.0, p_ei: 15.0, s_ei: 23.0 },
	Iso286Row { to: 18.0,  it: [11.0, 18.0, 27.0], g_es: -6.0,  f_es: -16.0, k_ei: 1.0, n_ei: 12.0, p_ei: 18.0, s_ei: 28.0 },
	Iso286Row { to: 30.0,  it: [13.0, 21.0, 33.0], g_es: -7.0,  f_es: -20.0, k_ei: 2.0, n_ei: 15.0, p_ei: 22.0, s_ei: 35.0 },
	Iso286Row { to: 50.0,  it: [16.0, 25.0, 39.0], g_es: -9.0,  f_es: -25.0, k_ei: 2.0, n_ei: 17.0, p_ei: 26.0, s_ei: 43.0 },
	Iso286Row { to: 65.0,  it: [19.0, 30.0, 46.0], g_es: -10.0, f_es: -30.0, k_ei: 2.0, n_ei: 20.0, p_ei: 32.0, s_ei: 53.0 },
	Iso286Row { to: 80.0,  it: [19.0, 30.0, 46.0], g_es: -10.0, f_es: -30.0, k_ei: 2.0, n_ei: 20.0, p_ei: 32.0, s_ei: 59.0 },
	Iso286Row { to: 100.0, it: [22.0, 35.0, 54.0], g_es: -12.0, f_es: -36.0, k_ei: 3.0, n_ei: 23.0, p_ei: 37.0, s_ei: 71.0 },
	Iso286Row { to: 120.0, it: [22.0, 35.0, 54.0], g_es: -12.0, f_es: -36.0, k_ei: 3.0, n_ei: 23.0, p_ei: 37.0, s_ei: 79.0 },
];

/// Resolve a nominal diameter `d` (0 < d ≤ 120 mm) and one of the ISO 286 hole-basis
/// **preferred fits** — `"H7/g6"` (sliding), `"H7/h6"` (locational clearance),
/// `"H7/k6"` (transition), `"H7/n6"` (transition/light press), `"H7/p6"` (press),
/// `"H7/s6"` (medium drive), `"H8/f7"` (close running) — into numeric [`FitLimits`].
/// Case-insensitive. `None` for any other fit string (the looser preferred fits
/// H9/d9, H11/c11 and the heavy H7/u6 are outside this table) or a diameter outside
/// `(0, 120]`.
pub fn iso286_fit(d: f64, fit: &str) -> Option<FitLimits> {
	if !(d > 0.0 && d <= 120.0) {
		return None; // NaN-safe: the conjunction refuses NaN diameters too
	}
	let r = ISO286.iter().find(|row| d <= row.to)?;
	let [it6, it7, it8] = r.it;
	// Holes: H = zero fundamental deviation, band (0, +IT). Shafts: g/f hang their
	// IT band below the upper deviation es; k/n/p/s stand it above the lower ei.
	let (hole_um, shaft_um) = match fit.to_ascii_lowercase().as_str() {
		"h7/g6" => ((0.0, it7), (r.g_es - it6, r.g_es)),
		"h7/h6" => ((0.0, it7), (-it6, 0.0)),
		"h7/k6" => ((0.0, it7), (r.k_ei, r.k_ei + it6)),
		"h7/n6" => ((0.0, it7), (r.n_ei, r.n_ei + it6)),
		"h7/p6" => ((0.0, it7), (r.p_ei, r.p_ei + it6)),
		"h7/s6" => ((0.0, it7), (r.s_ei, r.s_ei + it6)),
		"h8/f7" => ((0.0, it8), (r.f_es - it7, r.f_es)),
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
			for fit in ["H7/g6", "H7/h6", "H7/k6", "H7/n6", "H7/p6", "H7/s6", "H8/f7"] {
				let f = iso286_fit(d, fit).expect("in range");
				let ordered = f.hole.0 <= f.hole.1 && f.shaft.0 <= f.shaft.1 && f.clearance.0 <= f.clearance.1 && f.hole.0 == 0.0;
				let character = match fit {
					"H7/g6" | "H7/h6" | "H8/f7" => f.clearance.0 >= 0.0,
					"H7/s6" => f.clearance.1 < 0.0,
					"H7/p6" => f.clearance.0 < 0.0 && f.clearance.1 <= if d <= 3.0 { 4.0e-3 + 1e-12 } else { 0.0 },
					_ => f.clearance.0 < 0.0 && f.clearance.1 > 0.0, // k6/n6 transition
				};
				if !(ordered && character) {
					violations.push(format!("Ø{d} {fit}: {f:?}"));
				}
			}
		}
		assert!(violations.is_empty(), "every fit must keep ordered bands and its clearance character; violations: {violations:#?}");
	}
}
