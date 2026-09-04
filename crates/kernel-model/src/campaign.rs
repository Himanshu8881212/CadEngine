// Copyright (c) LMCAD. Licensed under the MIT License.

//! **Campaign gates** — the standard pass/fail verdict line for campaign
//! examples, with automatic Level-1 telemetry + friction capture.
//!
//! Promoted from the RESPOOL / DRYBOX examples' byte-identical local
//! `fn gate` (the rule-of-two idiom promotion, 2026-07-29): every campaign
//! prints its verdicts through this exact format, so the shared copy now also
//! feeds the self-learning flywheel — each gate becomes a `"gate"` event in
//! the opt-in telemetry log ([`kernel_core::telemetry::log`]) and every
//! FAILED gate is ALWAYS captured to the friction inbox
//! ([`kernel_core::telemetry::log_friction`]) for the lessons workflow.

/// Print one campaign gate line — byte-identical to the historical
/// example-local format — and fold `pass` into the run verdict `ok`.
///
/// Side channels (stdout is untouched by them): when telemetry is enabled a
/// `"gate"` event (label, pass, detail) is appended to the engine log, and a
/// failed gate is ALWAYS appended to the friction inbox as `"gate_fail"` —
/// a failure is always worth capturing, opted-in or not.
pub fn gate(label: &str, pass: bool, detail: String, ok: &mut bool) {
	*ok &= pass;
	println!("  {label:58} {detail:24} {}", if pass { "OK" } else { "<<< FAIL" });
	// Labels/details are simple ASCII; swap '"' for '\'' so a stray quote
	// cannot break the JSON line.
	let esc = |s: &str| s.replace('"', "'");
	if kernel_core::telemetry::enabled() {
		kernel_core::telemetry::log("gate", &format!("\"label\":\"{}\",\"pass\":{pass},\"detail\":\"{}\"", esc(label), esc(&detail)));
	}
	if !pass {
		kernel_core::telemetry::log_friction("gate_fail", &format!("\"label\":\"{}\",\"detail\":\"{}\"", esc(label), esc(&detail)));
	}
}
