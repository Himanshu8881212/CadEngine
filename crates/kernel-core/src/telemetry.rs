// Copyright (c) LMCAD. Licensed under the MIT License.

//! **Level-1 telemetry** — the capture layer of the self-learning data flywheel.
//!
//! Two append-only JSONL streams, both written best-effort (every I/O error is
//! silently swallowed — telemetry must never break geometry work):
//!
//! - [`log`] — the **opt-in engine event log** (`telemetry/engine_log.jsonl`,
//!   env override `LMCAD_TELEMETRY_PATH`). On when the env var
//!   `LMCAD_TELEMETRY` is set to anything other than `"0"`, or after a
//!   programmatic [`enable`]. Chain ops, campaign gates, … — one JSON object
//!   per line. This is the Level-1 data flywheel: the dataset that future
//!   learned advisors train on.
//! - [`log_friction`] — the **always-on friction inbox**
//!   (`docs/friction_inbox.jsonl`, env override `LMCAD_FRICTION_INBOX`). NOT
//!   gated by [`enabled`]: a refusal or failed gate is always worth capturing.
//!   Exception: cargo test/bench binaries are silent unless the env override
//!   is set — intentional failure-path tests are proof, not friction (see
//!   [`log_friction`]). This raw capture is what the lessons workflow curates
//!   into `docs/FRICTION.md`.
//!
//! Each call opens its file in append mode and writes one whole line
//! (`O_APPEND` keeps concurrent small-line appends atomic enough); there is no
//! global file handle and no lock held across calls.

use std::io::Write;
use std::sync::atomic::{AtomicBool, Ordering};

/// Programmatic opt-in latch, OR-ed with the `LMCAD_TELEMETRY` env var (see
/// [`enable`] / [`enabled`]).
static ENABLED: AtomicBool = AtomicBool::new(false);

/// Turn the opt-in event log on for the rest of this process (idempotent; there
/// is deliberately no `disable` — a run that opted into the flywheel stays in).
/// Campaign examples call this at the top of `main`.
pub fn enable() {
	ENABLED.store(true, Ordering::Relaxed);
}

/// Whether [`log`] currently writes: the env var `LMCAD_TELEMETRY` is set to
/// anything other than `"0"`, OR [`enable`] was called. ([`log_friction`]
/// ignores this — the friction inbox is always on.)
pub fn enabled() -> bool {
	ENABLED.load(Ordering::Relaxed) || std::env::var_os("LMCAD_TELEMETRY").is_some_and(|v| v != "0")
}

/// Append one event line `{"t":<unix_secs>,"kind":"<kind>",<payload fields>}`
/// to the event log (path: env `LMCAD_TELEMETRY_PATH`, default
/// `telemetry/engine_log.jsonl`; parent directories are created). A no-op
/// unless [`enabled`].
///
/// `payload_json` is a comma-separated run of JSON fields *without* braces —
/// e.g. `"\"label\":\"bore1\",\"secs\":0.12"` — spliced into the object
/// verbatim; the caller is responsible for escaping its string values. `kind`
/// is a plain ASCII identifier (`chain_op`, `gate`, …). Best-effort: all I/O
/// errors are silently ignored — telemetry must never break geometry work.
pub fn log(kind: &str, payload_json: &str) {
	if !enabled() {
		return;
	}
	let path = std::env::var("LMCAD_TELEMETRY_PATH").unwrap_or_else(|_| "telemetry/engine_log.jsonl".to_string());
	append(&path, &line("kind", kind, payload_json));
}

/// Append one line `{"t":<unix_secs>,"source":"<source>",<payload fields>}` to
/// the friction inbox (path: env `LMCAD_FRICTION_INBOX`, default
/// `docs/friction_inbox.jsonl`; parent directories are created).
///
/// ALWAYS appends — not gated by [`enabled`]: a failure is always worth
/// capturing, whether or not the run opted into the event log. This is the raw
/// stream the lessons workflow curates into `docs/FRICTION.md`. Same
/// `payload_json` splice contract and best-effort I/O as [`log`].
///
/// One carve-out: inside a cargo test/bench binary (detected via
/// [`is_test_binary`]) with no explicit `LMCAD_FRICTION_INBOX`, this is a
/// no-op. Failure-path tests intentionally feed the engine garbage to prove it
/// refuses; those refusals are the test PASSING, not friction, and they were
/// observed polluting a crate-local inbox on every `cargo test` run. Setting
/// `LMCAD_FRICTION_INBOX` re-enables capture even in tests (the telemetry unit
/// test uses this to pin the write path).
pub fn log_friction(source: &str, payload_json: &str) {
	let explicit = std::env::var("LMCAD_FRICTION_INBOX");
	if explicit.is_err() && is_test_binary() {
		return;
	}
	let path = explicit.unwrap_or_else(|_| "docs/friction_inbox.jsonl".to_string());
	append(&path, &line("source", source, payload_json));
}

/// True when the running executable is a cargo test or bench binary. Those run
/// un-uplifted straight out of `target/<profile>/deps/`, whereas examples and
/// real binaries are hard-linked up to `target/<profile>/(examples/)` — so
/// "parent directory named `deps`" separates the two without any env-var
/// contract (`CARGO_*` vars are compile-time, not reliably in the runtime env).
fn is_test_binary() -> bool {
	std::env::current_exe()
		.ok()
		.and_then(|p| p.parent().and_then(|d| d.file_name()).map(|n| n == "deps"))
		.unwrap_or(false)
}

/// Build one JSONL line: unix-seconds timestamp, the tag field, then the
/// caller's payload fields spliced in (an empty payload yields a well-formed
/// two-field object rather than a trailing comma).
fn line(tag: &str, value: &str, payload_json: &str) -> String {
	let t = std::time::SystemTime::now()
		.duration_since(std::time::UNIX_EPOCH)
		.map(|d| d.as_secs())
		.unwrap_or(0);
	if payload_json.is_empty() {
		format!("{{\"t\":{t},\"{tag}\":\"{value}\"}}")
	} else {
		format!("{{\"t\":{t},\"{tag}\":\"{value}\",{payload_json}}}")
	}
}

/// Best-effort append of one line: create parent dirs, open `O_APPEND`
/// per call (no global handle), swallow every error.
fn append(path: &str, line: &str) {
	let p = std::path::Path::new(path);
	if let Some(parent) = p.parent() {
		if !parent.as_os_str().is_empty() {
			let _ = std::fs::create_dir_all(parent);
		}
	}
	if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(p) {
		let _ = writeln!(f, "{line}");
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	/// ONE test function only: it mutates process-global state (env vars and the
	/// [`enable`] latch), so a sibling test in this parallel-threaded binary
	/// would race it. Unique temp paths per the mesh_io_roundtrip precedent
	/// (`std::env::temp_dir()` + process id).
	#[test]
	fn telemetry_gating_and_line_shape() {
		let pid = std::process::id();
		let ev = std::env::temp_dir().join(format!("lmcad_telemetry_{pid}.jsonl"));
		let fr = std::env::temp_dir().join(format!("lmcad_friction_{pid}.jsonl"));
		let _ = std::fs::remove_file(&ev);
		let _ = std::fs::remove_file(&fr);
		// The opt-in below must come from enable(), not an inherited env var.
		std::env::remove_var("LMCAD_TELEMETRY");
		std::env::remove_var("LMCAD_FRICTION_INBOX");

		// This lib-test binary runs from target/…/deps/, so friction with NO
		// explicit inbox must be suppressed (test refusals are not friction).
		let in_deps = is_test_binary();
		let default_inbox = std::path::Path::new("docs/friction_inbox.jsonl");
		let _ = std::fs::remove_file(default_inbox);
		log_friction("test_refusal", "\"label\":\"suppressed\"");
		let suppressed_in_tests = in_deps && !default_inbox.exists();
		let _ = std::fs::remove_dir("docs"); // only if the no-op left it empty

		std::env::set_var("LMCAD_TELEMETRY_PATH", &ev);
		std::env::set_var("LMCAD_FRICTION_INBOX", &fr);

		log("noop", "\"x\":1"); // disabled → must not create the file
		let silent_while_disabled = !ev.exists();

		log_friction("test_refusal", "\"label\":\"probe\""); // friction is ALWAYS on
		let friction_line = std::fs::read_to_string(&fr).unwrap_or_default();

		enable();
		log("test_kind", "\"label\":\"probe\",\"secs\":0.5");
		let event_line = std::fs::read_to_string(&ev).unwrap_or_default();

		let _ = std::fs::remove_file(&ev);
		let _ = std::fs::remove_file(&fr);

		assert!(
			silent_while_disabled
				&& suppressed_in_tests
				&& enabled()
				&& friction_line.starts_with("{\"t\":")
				&& friction_line.contains("\"source\":\"test_refusal\",\"label\":\"probe\"")
				&& friction_line.ends_with("}\n")
				&& event_line.starts_with("{\"t\":")
				&& event_line.contains("\"kind\":\"test_kind\",\"label\":\"probe\",\"secs\":0.5")
				&& event_line.ends_with("}\n"),
			"telemetry contract violated: silent_while_disabled={silent_while_disabled} \
			 suppressed_in_tests={suppressed_in_tests} (in_deps={in_deps}) \
			 enabled={} friction_line={friction_line:?} event_line={event_line:?}",
			enabled(),
		);
	}
}
