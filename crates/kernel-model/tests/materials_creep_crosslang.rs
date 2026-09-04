// Copyright (c) LMCAD. Licensed under the MIT License.

//! THE CROSS-LANGUAGE PIN for the sustained-load (creep) allowable — Rust half.
//!
//! Two readers of ONE table (`tools/materials/pla.json#creep.sig_allow_mpa`)
//! once disagreed: `kernel_model::materials::pla::creep_allowable_mpa` refused
//! above the hot tier (0.0 MPa — "no sustained load is defensible"), while the
//! Python helper campaigns could actually reach fell back to the LAST tabulated
//! row and returned 1.5 MPa at 70 °C and at 120 °C, flagged only by an
//! `extrapolated: true` field a gate can miss. The reachable reader was the
//! NON-CONSERVATIVE one, and sustained load is the governing failure mode in
//! half the portfolio.
//!
//! `tools/materials/creep_crosslang_vectors.json` is the contract both readers
//! must satisfy: the allowable, the CELL it was read at, and the refusal kind,
//! at every tier boundary and on both sides of it. This test proves the Rust
//! side; `tools/materials_crosslang_test.py` proves the Python side against the
//! same file. A divergence of that class can now only land by breaking one of
//! the two tests.
//!
//! The vectors file is a CONTRACT, not a cache: never regenerate it to make a
//! failing test pass.

use kernel_model::materials::pla;
use serde_json::Value;

fn vectors() -> Value {
	let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tools/materials/creep_crosslang_vectors.json");
	let raw = std::fs::read_to_string(&path)
		.unwrap_or_else(|e| panic!("cannot read the cross-language creep vectors at {}: {e}", path.display()));
	serde_json::from_str(&raw).unwrap_or_else(|e| panic!("{} is not valid JSON: {e}", path.display()))
}

/// A probe axis is a JSON number, or one of the strings "nan" / "inf" / "-inf"
/// (JSON has no literal for those, and they are exactly the inputs that must
/// REFUSE rather than default to a cell).
fn axis(v: &Value) -> f64 {
	match v {
		Value::Number(n) => n.as_f64().expect("probe axis is not representable as f64"),
		Value::String(s) => match s.as_str() {
			"nan" => f64::NAN,
			"inf" => f64::INFINITY,
			"-inf" => f64::NEG_INFINITY,
			other => panic!("probe axis string {other:?} is not one of nan/inf/-inf"),
		},
		other => panic!("probe axis {other:?} is neither a number nor a special-value string"),
	}
}

fn opt_f64(v: &Value) -> Option<f64> {
	match v {
		Value::Null => None,
		Value::Number(n) => Some(n.as_f64().expect("expected a float")),
		other => panic!("expected null or a number, got {other:?}"),
	}
}

#[test]
fn rust_and_python_agree_at_every_creep_cell_boundary() {
	let doc = vectors();
	let probes = doc["probes"].as_array().expect("vectors file has no `probes` array");
	assert!(
		probes.len() >= 200,
		"the cross-language creep pin has only {} probes — it must cover every tier boundary, \
		 both sides of every boundary, above the hot tier, beyond the last duration column, and \
		 every non-finite/negative input",
		probes.len()
	);

	let mut bad: Vec<String> = Vec::new();
	for (i, p) in probes.iter().enumerate() {
		let t = axis(&p["temp_c"]);
		let h = axis(&p["hours"]);
		let across = p["across_layer"].as_bool().expect("probe.across_layer must be a bool");
		let got = pla::creep_lookup(t, h, across);

		let want_sigma = p["sig_allow_mpa"].as_f64().expect("probe.sig_allow_mpa must be a number");
		let want_in_plane = p["in_plane_mpa"].as_f64().expect("probe.in_plane_mpa must be a number");
		let want_row = opt_f64(&p["row_used_c"]);
		let want_col = opt_f64(&p["col_used_h"]);
		let want_match = p["cell_match"].as_str().expect("probe.cell_match must be a string");
		let want_refusal = p["refusal_kind"].as_str();

		let got_refusal = got.refusal.map(|r| r.as_str());
		// Tolerance is 1e-12 absolute, not exact equality, ONLY because the
		// anisotropy product 0.5 * 0.55 is not exact in binary; the table cells
		// themselves must match bit-for-bit.
		let sigma_ok = (got.sig_allow_mpa - want_sigma).abs() < 1e-12;
		let in_plane_ok = got.in_plane_mpa == want_in_plane;
		let row_ok = got.row_used_c == want_row;
		let col_ok = got.col_used_h == want_col;
		let match_ok = got.cell_match.as_str() == want_match;
		let refusal_ok = got_refusal == want_refusal;

		if !(sigma_ok && in_plane_ok && row_ok && col_ok && match_ok && refusal_ok) {
			bad.push(format!(
				"probe[{i}] T={t} h={h} across={across}: rust sigma={} in_plane={} row={:?} col={:?} \
				 match={} refusal={:?} | python sigma={want_sigma} in_plane={want_in_plane} \
				 row={want_row:?} col={want_col:?} match={want_match} refusal={want_refusal:?}",
				got.sig_allow_mpa,
				got.in_plane_mpa,
				got.row_used_c,
				got.col_used_h,
				got.cell_match.as_str(),
				got_refusal,
			));
		}

		// The bare scalar entry point must never disagree with the receipted
		// one — `creep_allowable_mpa` IS `creep_lookup(..).sig_allow_mpa`.
		if !across {
			let bare = pla::creep_allowable_mpa(t, h);
			// `>= 1e-12` rather than `!(< 1e-12)` so clippy can see the intent, and
			// `is_nan()` spelled out so a NaN on either side is still a MISMATCH —
			// that is what the negated comparison was buying, and a NaN allowable
			// slipping through as "agrees" would be the worst possible pass.
			let delta = bare - got.sig_allow_mpa;
			if delta.is_nan() || delta.abs() >= 1e-12 {
				bad.push(format!(
					"probe[{i}] T={t} h={h}: creep_allowable_mpa={bare} disagrees with \
					 creep_lookup().sig_allow_mpa={}",
					got.sig_allow_mpa
				));
			}
		}
	}

	assert!(
		bad.is_empty(),
		"the Rust and Python readers of tools/materials/pla.json#creep.sig_allow_mpa DISAGREE \
		 at {} of {} probes. One table, one semantic, and the REFUSING semantic wins — a reader \
		 that returns a number where the other refuses is the non-conservative one and is the bug. \
		 Do NOT regenerate tools/materials/creep_crosslang_vectors.json to silence this.\n{}",
		bad.len(),
		probes.len(),
		bad.join("\n")
	);
}

/// Hand-written, NOT generated: the doctrine points, read straight off the
/// researched record. If the vectors file were ever regenerated from a broken
/// reader, these would still fail.
#[test]
fn creep_refusal_doctrine_is_hand_pinned() {
	let above_hot = [55.000001_f64, 56.0, 70.0, 120.0, 1.0e6];
	let refuses_above_hot = above_hot.iter().all(|t| {
		let c = pla::creep_lookup(*t, 24.0, false);
		c.sig_allow_mpa == 0.0 && c.refused() && c.refusal == Some(pla::CreepRefusal::TempAboveTabulated) && c.row_used_c.is_none()
	});
	let refuses_nonsense = [(f64::NAN, 24.0), (23.0, f64::NAN), (23.0, f64::INFINITY), (f64::INFINITY, 24.0)]
		.iter()
		.all(|(t, h)| pla::creep_lookup(*t, *h, false).refusal == Some(pla::CreepRefusal::InputNotFinite))
		&& pla::creep_lookup(23.0, -1.0, false).refusal == Some(pla::CreepRefusal::NegativeDuration);

	// The 25 °C trap: a design whose declared ambient is 25 °C reads the 55 °C
	// row, and the receipt must SAY so instead of leaving it in prose.
	let ambient_25 = pla::creep_lookup(25.0, 8760.0, false);
	let step_is_visible = ambient_25.sig_allow_mpa == 0.5
		&& ambient_25.row_used_c == Some(55.0)
		&& ambient_25.cell_match == pla::CreepCellMatch::RoundedUpConservative;

	// Exact cells, and the across-layer derate as an explicit caller choice.
	let exact = pla::creep_lookup(23.0, 8760.0, false);
	let across = pla::creep_lookup(23.0, 8760.0, true);
	let cells_ok = exact.sig_allow_mpa == 2.5
		&& exact.cell_match == pla::CreepCellMatch::Exact
		&& exact.anisotropy_factor == 1.0
		&& (across.sig_allow_mpa - 2.5 * pla::Z_VS_XY_STRENGTH_RATIO).abs() < 1e-12
		&& across.in_plane_mpa == 2.5
		&& across.anisotropy_factor == pla::Z_VS_XY_STRENGTH_RATIO;

	// Beyond the last duration column the last column is reused, and it is
	// FLAGGED — the extrapolation is on the record, not implied.
	let beyond = pla::creep_lookup(23.0, 87600.0, false);
	let beyond_ok =
		beyond.sig_allow_mpa == 2.5 && beyond.cell_match == pla::CreepCellMatch::ExtrapolatedBeyondLastDuration && !beyond.refused();

	assert!(
		refuses_above_hot && refuses_nonsense && step_is_visible && cells_ok && beyond_ok,
		"creep refusal doctrine violated: refuses_above_hot={refuses_above_hot} \
		 refuses_nonsense={refuses_nonsense} step_is_visible={step_is_visible} (25C -> {:?} MPa at \
		 row {:?}) cells_ok={cells_ok} beyond_ok={beyond_ok}",
		ambient_25.sig_allow_mpa,
		ambient_25.row_used_c,
	);
}
