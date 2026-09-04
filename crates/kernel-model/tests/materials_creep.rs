// Copyright (c) LMCAD. Licensed under the MIT License.

//! Pins for the time-dependent (creep) PLA allowables promoted into
//! [`kernel_model::materials::pla`] by the 2026-07-30 research wave.
//!
//! Two classes of pin:
//! 1. **Contract pins** — the conservative lookup rule (round temperature and
//!    duration UP to the next tabulated cell), the refusal regime above the
//!    hot tier, and monotonicity in both axes.
//! 2. **A cross-language pin** — the Rust table must equal the researched
//!    block in `tools/materials/pla.json`, which is the single source of
//!    truth (it carries the derivation chain, per-cell confidence and the
//!    sources). Two copies of a safety number that can drift silently is the
//!    failure mode this test exists to prevent.

use kernel_model::materials::pla;

#[test]
fn creep_lookup_is_conservative_and_monotone() {
	// Exact tabulated cells.
	let rt_1h = pla::creep_allowable_mpa(23.0, 1.0);
	let rt_1y = pla::creep_allowable_mpa(23.0, 8760.0);
	let hot_24h = pla::creep_allowable_mpa(55.0, 24.0);

	// Conservative rounding: an in-between request must read the WORSE cell.
	// 40 °C is between the tiers -> must use the 55 °C row, not the 23 °C row.
	let mid_temp = pla::creep_allowable_mpa(40.0, 1.0);
	// 100 h is between 24 h and 30 d -> must use the 30 d column.
	let mid_time = pla::creep_allowable_mpa(23.0, 100.0);

	// Above the hot tier there is no defensible sustained allowable at all.
	let too_hot = pla::creep_allowable_mpa(70.0, 1.0);
	// Garbage in -> 0.0 (a gate written as `stress <= allowable` then FAILS).
	let nonsense = pla::creep_allowable_mpa(f64::NAN, 1.0) + pla::creep_allowable_mpa(23.0, -5.0);

	// Monotonicity: longer is never stronger, hotter is never stronger.
	let time_monotone = (0..pla::CREEP_HOURS.len() - 1).all(|i| {
		pla::CREEP_SIG_ALLOW_MPA[0][i] >= pla::CREEP_SIG_ALLOW_MPA[0][i + 1]
			&& pla::CREEP_SIG_ALLOW_MPA[1][i] >= pla::CREEP_SIG_ALLOW_MPA[1][i + 1]
	});
	let temp_monotone = (0..pla::CREEP_HOURS.len()).all(|i| pla::CREEP_SIG_ALLOW_MPA[0][i] >= pla::CREEP_SIG_ALLOW_MPA[1][i]);

	// The static RT design point must be STRONGER than any sustained cell —
	// if it were not, the static tier would be the unsafe one.
	let static_above_sustained = pla::SIG_ALLOW_RT > rt_1h;

	// Shear rides the same 0.6 ratio as the static tier.
	let shear_ok = (pla::creep_shear_allowable_mpa(23.0, 1.0) - 0.6 * rt_1h).abs() < 1e-12;

	assert!(
		(rt_1h - 7.5).abs() < 1e-12
			&& (rt_1y - 2.5).abs() < 1e-12
			&& (hot_24h - 1.5).abs() < 1e-12
			&& (mid_temp - 3.0).abs() < 1e-12
			&& (mid_time - 3.5).abs() < 1e-12
			&& too_hot == 0.0
			&& nonsense == 0.0
			&& time_monotone
			&& temp_monotone
			&& static_above_sustained
			&& shear_ok,
		"PLA creep contract violated: 23C/1h={rt_1h} (want 7.5) 23C/1y={rt_1y} (want 2.5) \
		 55C/24h={hot_24h} (want 1.5) 40C/1h={mid_temp} (want 3.0 — must round UP to the hot row) \
		 23C/100h={mid_time} (want 3.5 — must round UP to the 30d column) 70C={too_hot} (want 0.0) \
		 nonsense={nonsense} (want 0.0) time_monotone={time_monotone} temp_monotone={temp_monotone} \
		 static({}) > sustained_1h: {static_above_sustained} shear_ratio_ok={shear_ok}",
		pla::SIG_ALLOW_RT,
	);
}

#[test]
fn rust_creep_table_matches_the_researched_json() {
	// tools/materials/pla.json carries the derivation, confidence and sources;
	// the Rust table is a mirror for campaign gates. They must not drift.
	let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tools/materials/pla.json");
	let raw = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("cannot read {}: {e}", path.display()));

	// Minimal scrape (kernel-model has no JSON-value dependency in tests):
	// find "sig_allow_mpa", then read the four numbers after each temperature
	// key in tabulated order.
	let table = raw.split_once("\"sig_allow_mpa\"").unwrap_or_else(|| panic!("pla.json has no creep.sig_allow_mpa block")).1;
	let mut found = [[f64::NAN; 4]; 2];
	for (row, temp_key) in ["\"23C\"", "\"55C\""].iter().enumerate() {
		let seg = table.split_once(temp_key).unwrap_or_else(|| panic!("pla.json creep table has no {temp_key} row")).1;
		let end = seg.find('}').unwrap_or(seg.len());
		// `rsplit_once` (not `split_once`): the first comma-chunk still carries
		// the block's own `": {"` separator, so splitting on the FIRST colon
		// reads the wrong side and silently drops that cell.
		let cells: Vec<f64> =
			seg[..end].split(',').filter_map(|kv| kv.rsplit_once(':').and_then(|(_, v)| v.trim().parse::<f64>().ok())).collect();
		assert!(
			cells.len() == 4,
			"pla.json creep row {temp_key} parsed {} cells, want 4 (segment: {:?})",
			cells.len(),
			&seg[..end.min(160)]
		);
		found[row].copy_from_slice(&cells);
	}

	let matches =
		found.iter().zip(pla::CREEP_SIG_ALLOW_MPA.iter()).all(|(j, r)| j.iter().zip(r.iter()).all(|(a, b)| (a - b).abs() < 1e-12));

	assert!(
		matches,
		"Rust creep table has drifted from tools/materials/pla.json — json={found:?} rust={:?}. \
		 The JSON is the source of truth (it carries the derivation chain and per-cell confidence); \
		 update the Rust mirror in kernel_model::materials::pla, never the other way round.",
		pla::CREEP_SIG_ALLOW_MPA,
	);
}
