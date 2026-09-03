// Copyright (c) LMCAD. Licensed under the MIT License.

//! The LMCAD Studio HTTP/SSE client — everything the TUI and the headless
//! modes share, kept free of any terminal code so it is unit-testable.
//!
//! Three server surfaces (see `studio/server/src/{run,chat,catalog}.rs`):
//!
//! - `POST /api/run` — execute a work-order JSON program; returns the kernel
//!   [`Report`] (the only geometry contract) plus exported [`Artifact`]s.
//! - `POST /api/chat` — the server-side Claude operator loop, streamed back as
//!   SSE frames which [`SseParser`] maps to typed [`ChatEvent`]s.
//! - `GET /api/catalog` — the standard-parts families.
//!
//! Transport is blocking `ureq` (no async runtime); the SSE stream is read
//! line-by-line off the response body. When `CADCODE_API_TOKEN` is set every
//! request carries `Authorization: Bearer <token>`.

use std::collections::HashMap;
use std::io::{BufRead, BufReader};
use std::time::Duration;

use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

// ---------------------------------------------------------------------------
// Chat event protocol
// ---------------------------------------------------------------------------

/// One item of the orchestrator's TodoWrite-style plan (a `tasks` frame).
#[derive(Clone, Debug, PartialEq)]
pub struct TaskItem {
	/// What the step is.
	pub content: String,
	/// `"pending"`, `"in_progress"` or `"completed"`.
	pub status: String,
}

/// Checklist mark for a task status: `☐` pending, `◐` in_progress,
/// `☑` completed, `?` for anything unrecognized (rendered, never guessed).
pub fn task_mark(status: &str) -> &'static str {
	match status {
		"pending" => "☐",
		"in_progress" => "◐",
		"completed" => "☑",
		_ => "?",
	}
}

/// One typed event of the `/api/chat` SSE stream (`studio/server/src/chat.rs`
/// module docs are the wire contract this mirrors).
#[derive(Clone, Debug, PartialEq)]
pub enum ChatEvent {
	/// `text` — an assistant text delta (append to the current reply).
	Text(String),
	/// `thinking` — a thinking-summary delta (rendered dim, `[thinking]`).
	Thinking(String),
	/// `tool` — a server-side tool-call status line (`running` → `done`) for
	/// the kernel-facing tools (`run_program` / `describe_api`).
	Tool {
		/// `"running"` or `"done"`.
		state: String,
		/// Tool name (`run_program` / `describe_api`).
		name: String,
		/// Op count of the submitted work order (report op count on `done`).
		ops: u64,
		/// Present on `done`: whether the whole report is ok.
		ok: Option<bool>,
		/// Present on `done` when an op failed: `kind: message` of the first error.
		error: Option<String>,
	},
	/// `tasks` — the orchestrator's current plan, re-sent whole on every
	/// `task_update` (replace, don't merge).
	Tasks(Vec<TaskItem>),
	/// `subagent` — one parallel part-agent lifecycle line:
	/// `started` → `tool`* → `done` | `error`.
	Subagent {
		/// The part agent's name.
		name: String,
		/// `"started"`, `"tool"`, `"done"` or `"error"`.
		state: String,
		/// Extra detail (tool summary / error message), when present.
		detail: Option<String>,
	},
	/// `refresh` — a run exported meshes: artifact file names plus the
	/// server's viewport receipt (volume / route / tris / watertight), verbatim.
	Refresh {
		/// Exported file names (relative to the session out-dir).
		artifacts: Vec<String>,
		/// The receipt object as sent, when present (never synthesized here).
		receipt: Option<Value>,
	},
	/// `chat_disabled` — the server has no `ANTHROPIC_API_KEY`; message shown
	/// once, everything else keeps working.
	Disabled(String),
	/// `error` — an API/transport failure surfaced by the server.
	Error(String),
	/// `done` — the loop finished, with its stop reason.
	Done(String),
}

/// Incremental SSE parser: feed decoded lines (no trailing newline; a trailing
/// `\r` is tolerated), get a typed [`ChatEvent`] whenever a blank line
/// completes a frame. Per the SSE spec, multiple `data:` lines in one frame
/// join with `\n`, comment lines (leading `:`) are ignored (axum's keep-alive
/// pings arrive this way), and unknown event names or fields are tolerated.
#[derive(Default)]
pub struct SseParser {
	event: String,
	data: Vec<String>,
}

impl SseParser {
	/// A fresh parser with no partial frame.
	pub fn new() -> Self {
		Self::default()
	}

	/// Feed one line; `Some(event)` when this line completed a known frame.
	pub fn push_line(&mut self, line: &str) -> Option<ChatEvent> {
		let line = line.strip_suffix('\r').unwrap_or(line);
		if line.is_empty() {
			return self.dispatch();
		}
		if let Some(rest) = line.strip_prefix(':') {
			let _ = rest; // SSE comment (keep-alive) — ignored
			return None;
		}
		let (field, value) = match line.split_once(':') {
			Some((f, v)) => (f, v.strip_prefix(' ').unwrap_or(v)),
			None => (line, ""),
		};
		match field {
			"event" => self.event = value.to_string(),
			"data" => self.data.push(value.to_string()),
			_ => {} // id / retry / unknown fields — tolerated
		}
		None
	}

	fn dispatch(&mut self) -> Option<ChatEvent> {
		if self.event.is_empty() && self.data.is_empty() {
			return None; // blank line between frames / after comments
		}
		let name = std::mem::take(&mut self.event);
		let data = std::mem::take(&mut self.data).join("\n");
		map_event(&name, &data)
	}
}

/// Map one completed SSE frame to a [`ChatEvent`]. Unknown event names return
/// `None` (forward-compatible); malformed JSON on a *known* event is surfaced
/// as [`ChatEvent::Error`] rather than swallowed.
fn map_event(name: &str, data: &str) -> Option<ChatEvent> {
	if !matches!(name, "text" | "thinking" | "tool" | "tasks" | "subagent" | "refresh" | "chat_disabled" | "error" | "done") {
		return None;
	}
	let v: Value = match serde_json::from_str(data) {
		Ok(v) => v,
		Err(e) => return Some(ChatEvent::Error(format!("unparseable `{name}` event data: {e}"))),
	};
	let s = |key: &str| v.get(key).and_then(Value::as_str).unwrap_or("").to_string();
	Some(match name {
		"text" => ChatEvent::Text(s("delta")),
		"thinking" => ChatEvent::Thinking(s("delta")),
		"tool" => ChatEvent::Tool {
			state: s("state"),
			name: s("name"),
			ops: v.get("ops").and_then(Value::as_u64).unwrap_or(0),
			ok: v.get("ok").and_then(Value::as_bool),
			error: v.get("error").and_then(Value::as_str).map(str::to_string),
		},
		"tasks" => ChatEvent::Tasks(
			v.get("tasks")
				.and_then(Value::as_array)
				.map(|a| {
					a.iter()
						.map(|t| TaskItem {
							content: t.get("content").and_then(Value::as_str).unwrap_or("?").to_string(),
							status: t.get("status").and_then(Value::as_str).unwrap_or("pending").to_string(),
						})
						.collect()
				})
				.unwrap_or_default(),
		),
		"subagent" => ChatEvent::Subagent {
			name: s("name"),
			state: s("state"),
			detail: v.get("detail").and_then(Value::as_str).map(str::to_string),
		},
		"refresh" => ChatEvent::Refresh {
			artifacts: v
				.get("artifacts")
				.and_then(Value::as_array)
				.map(|a| a.iter().map(|x| x.get("file").and_then(Value::as_str).unwrap_or("?").to_string()).collect())
				.unwrap_or_default(),
			receipt: v.get("receipt").filter(|r| !r.is_null()).cloned(),
		},
		"chat_disabled" => ChatEvent::Disabled(s("message")),
		"error" => ChatEvent::Error(s("message")),
		"done" => ChatEvent::Done(s("stop_reason")),
		_ => unreachable!("gated by the matches! above"),
	})
}

/// Parse a whole raw SSE byte-stream (as text) into events — the pure-function
/// form of [`SseParser`] used by tests and one-shot consumers. A trailing
/// unterminated frame is flushed.
pub fn parse_sse_lines(raw: &str) -> Vec<ChatEvent> {
	let mut parser = SseParser::new();
	let mut out: Vec<ChatEvent> = raw.lines().filter_map(|l| parser.push_line(l)).collect();
	if let Some(last) = parser.push_line("") {
		out.push(last);
	}
	out
}

// ---------------------------------------------------------------------------
// /api/run report mirror (lightweight — the TUI does not link the kernel)
// ---------------------------------------------------------------------------

/// Response of `POST /api/run`.
#[derive(Clone, Debug, Deserialize)]
pub struct RunResponse {
	/// Mirror of `report.ok`.
	pub ok: bool,
	/// Session the run executed in.
	pub session: String,
	/// The kernel's own report, verbatim.
	pub report: Report,
	/// Files the report says were written (with serve URLs, unused here).
	#[serde(default)]
	pub artifacts: Vec<Artifact>,
}

/// One exported file of a run.
#[derive(Clone, Debug, Deserialize)]
pub struct Artifact {
	/// Path relative to the session out-dir.
	pub file: String,
}

/// The kernel execution report (`kernel-api` `Report`, deserialized shape only).
#[derive(Clone, Debug, Deserialize)]
pub struct Report {
	/// Response contract version (`cadcode.v1`).
	#[serde(default)]
	pub api_version: Option<String>,
	/// True iff every attempted op succeeded.
	pub ok: bool,
	/// Per-op results in program order, up to and including the first failure.
	pub ops: Vec<OpReport>,
}

/// One op's report entry.
#[derive(Clone, Debug, Deserialize)]
pub struct OpReport {
	/// The op's `id` from the program.
	pub id: String,
	/// Whether the op succeeded.
	pub ok: bool,
	/// Op-specific measurements (validate / volume / export route …).
	#[serde(default)]
	pub measures: Option<Value>,
	/// The path actually written, for export ops.
	#[serde(default)]
	pub file: Option<String>,
	/// The structured failure, present iff `ok` is false.
	#[serde(default)]
	pub error: Option<OpError>,
}

/// A structured op failure.
#[derive(Clone, Debug, Deserialize)]
pub struct OpError {
	/// Machine-matchable failure class (snake_case string on the wire).
	pub kind: String,
	/// Human-readable detail.
	pub message: String,
}

// ---------------------------------------------------------------------------
// Report formatting (shared by the receipts pane and the headless printer)
// ---------------------------------------------------------------------------

/// One-line `k=v` join of an op's `measures` object. Renders exactly what the
/// report carries — never synthesizes. Long arrays are elided by count so a
/// `describe` payload cannot flood a receipt line.
pub fn summarize_measures(measures: &Value) -> String {
	fn compact(v: &Value) -> String {
		match v {
			Value::String(s) => s.clone(),
			Value::Array(a) if a.len() > 8 => format!("[{} items]", a.len()),
			other => other.to_string(),
		}
	}
	match measures.as_object() {
		Some(map) => map.iter().map(|(k, v)| format!("{k}={}", compact(v))).collect::<Vec<_>>().join(" "),
		None => measures.to_string(),
	}
}

/// Render one op report line: `✓|✗ id · op-name · measures → file · error`.
/// `op_name` comes from the submitted program (the report does not echo it).
pub fn op_line(op: &OpReport, op_name: Option<&str>) -> String {
	let mark = if op.ok { "✓" } else { "✗" };
	let mut line = match op_name {
		Some(name) => format!("{mark} {} · {name}", op.id),
		None => format!("{mark} {}", op.id),
	};
	if let Some(m) = &op.measures {
		let s = summarize_measures(m);
		if !s.is_empty() {
			line.push_str(" · ");
			line.push_str(&s);
		}
	}
	if let Some(f) = &op.file {
		let name = std::path::Path::new(f).file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_else(|| f.clone());
		line.push_str(" → ");
		line.push_str(&name);
	}
	if let Some(e) = &op.error {
		line.push_str(&format!(" · {}: {}", e.kind, e.message));
	}
	line
}

/// One-line summary of a `refresh` viewport receipt — only the fields the
/// server actually sent (volume/route/tris/watertight), never synthesized.
pub fn receipt_summary(receipt: &Value) -> String {
	let mut parts = Vec::new();
	if let Some(v) = receipt.get("volume").and_then(Value::as_f64) {
		let source = receipt.get("volume_source").and_then(Value::as_str).unwrap_or("?");
		parts.push(format!("volume={v:.3} ({source})"));
	}
	if let Some(r) = receipt.get("route").and_then(Value::as_str) {
		parts.push(format!("route={r}"));
	}
	if let Some(t) = receipt.get("tris").and_then(Value::as_u64) {
		parts.push(format!("tris={t}"));
	}
	if let Some(w) = receipt.get("watertight").and_then(Value::as_bool) {
		parts.push(format!("watertight={w}"));
	}
	parts.join(" ")
}

/// Map op `id` → op name from a submitted program (bare `{"ops": [...]}` or
/// wrapped `{"program": {"ops": [...]}}`), so report lines can name the op.
pub fn op_names_of(program: &Value) -> HashMap<String, String> {
	let ops = program.get("ops").or_else(|| program.pointer("/program/ops")).and_then(Value::as_array);
	let mut names = HashMap::new();
	for op in ops.into_iter().flatten() {
		if let (Some(id), Some(name)) = (op.get("id").and_then(Value::as_str), op.get("op").and_then(Value::as_str)) {
			names.insert(id.to_string(), name.to_string());
		}
	}
	names
}

// ---------------------------------------------------------------------------
// The client
// ---------------------------------------------------------------------------

/// One prior conversation turn, resent to the server every chat request (the
/// client owns the history; tool turns live server-side within a request).
#[derive(Clone, Debug, Serialize)]
pub struct ChatTurn {
	/// `"user"` or `"assistant"`.
	pub role: String,
	/// Plain text content.
	pub content: String,
}

/// Why a health probe failed — connection-refused is the auto-spawn trigger.
#[derive(Debug)]
pub enum HealthError {
	/// TCP connect refused: nothing is listening at the base URL.
	Refused,
	/// Any other transport failure (DNS, timeout, TLS …).
	Other(String),
}

/// Blocking HTTP client for one LMCAD Studio server.
#[derive(Clone)]
pub struct Client {
	/// Base URL without a trailing slash (e.g. `http://127.0.0.1:7878`).
	pub base: String,
	/// Bearer token, sent as `Authorization: Bearer <token>` when set.
	pub token: Option<String>,
	agent: ureq::Agent,
}

impl Client {
	/// Build from the environment: `CADCODE_SERVER` (default
	/// `http://127.0.0.1:7878`) and `CADCODE_API_TOKEN` (optional bearer).
	pub fn from_env() -> Self {
		let base = std::env::var("CADCODE_SERVER")
			.ok()
			.filter(|s| !s.trim().is_empty())
			.unwrap_or_else(|| "http://127.0.0.1:7878".to_string());
		let token = std::env::var("CADCODE_API_TOKEN").ok().filter(|s| !s.is_empty());
		Self::new(&base, token)
	}

	/// Build for an explicit base URL (trailing `/` trimmed) and token.
	pub fn new(base: &str, token: Option<String>) -> Self {
		// No global timeout: chat streams and heavy kernel runs are long-lived by
		// design (the server enforces its own 300 s request deadline). Connect
		// timeout only, so a dead host fails fast.
		let config = ureq::Agent::config_builder()
			.http_status_as_error(false)
			.timeout_connect(Some(Duration::from_secs(5)))
			.build();
		Self { base: base.trim_end_matches('/').to_string(), token, agent: ureq::Agent::new_with_config(config) }
	}

	fn url(&self, path: &str) -> String {
		format!("{}{path}", self.base)
	}

	fn auth<B>(&self, req: ureq::RequestBuilder<B>) -> ureq::RequestBuilder<B> {
		match &self.token {
			Some(t) => req.header("Authorization", format!("Bearer {t}")),
			None => req,
		}
	}

	/// GET `/` — any HTTP response means the server is up (the route serves
	/// either the built front-end or a plain-text pointer).
	pub fn health(&self) -> Result<(), HealthError> {
		let req = self.agent.get(self.url("/")).config().timeout_global(Some(Duration::from_secs(3))).build();
		match req.call() {
			Ok(_) => Ok(()),
			Err(e) => Err(classify_health(e)),
		}
	}

	/// Make sure a server is answering: probe health, and on connection-refused
	/// optionally auto-spawn `./target/release/studio-server` (relative to the
	/// current directory), detached, retrying health for ~3 s. Returns the
	/// spawn note when a server was started, `Err` with a start hint when the
	/// server stayed down.
	pub fn ensure_server(&self, allow_spawn: bool) -> Result<Option<String>, String> {
		match self.health() {
			Ok(()) => return Ok(None),
			Err(HealthError::Other(m)) => return Err(format!("cannot reach {}: {m}", self.base)),
			Err(HealthError::Refused) => {}
		}
		let hint = format!(
			"cannot reach {} (connection refused).\nstart it with: cargo run -p studio-server --release   (or point CADCODE_SERVER elsewhere)",
			self.base
		);
		if !allow_spawn {
			return Err(hint);
		}
		let local_target = self.base.starts_with("http://127.0.0.1:")
			|| self.base.starts_with("http://localhost:")
			|| self.base.starts_with("http://[::1]:");
		if !local_target {
			return Err(format!("refusing to auto-spawn a local server for non-loopback CADCODE_SERVER '{}'.\n{hint}", self.base));
		}
		let bin = if let Some(explicit) = std::env::var_os("CADCODE_SERVER_BIN") {
			std::fs::canonicalize(&explicit)
				.map_err(|e| format!("CADCODE_SERVER_BIN '{}' is not a readable executable: {e}\n{hint}", std::path::Path::new(&explicit).display()))?
		} else {
			let exe = std::env::current_exe().map_err(|e| format!("cannot locate lmcad-tui executable: {e}\n{hint}"))?;
			let sibling = exe.parent().ok_or_else(|| format!("lmcad-tui executable has no parent directory\n{hint}"))?.join("studio-server");
			std::fs::canonicalize(&sibling).map_err(|_| {
				format!("trusted sibling server binary '{}' is absent. Set CADCODE_SERVER_BIN explicitly or start the server manually.\n{hint}", sibling.display())
			})?
		};
		if !bin.is_file() {
			return Err(format!("server binary '{}' is not a regular file\n{hint}", bin.display()));
		}
		let mut command = std::process::Command::new(&bin);
		// A release-layout sibling lives at <repo>/target/release/studio-server.
		// Start it from <repo> so the default LMCAD_ROOT cannot be attacker-chosen cwd.
		if std::env::var_os("LMCAD_ROOT").is_none() {
			if let Some(repo) = bin.parent().and_then(std::path::Path::parent).and_then(std::path::Path::parent) {
				if repo.join("Cargo.toml").is_file() {
					command.current_dir(repo).env("LMCAD_ROOT", repo);
				}
			}
		}
		command
			.stdin(std::process::Stdio::null())
			.stdout(std::process::Stdio::null())
			.stderr(std::process::Stdio::null())
			.spawn()
			.map_err(|e| format!("failed to spawn {}: {e}\n{hint}", bin.display()))?;
		for _ in 0..10 {
			std::thread::sleep(Duration::from_millis(300));
			if self.health().is_ok() {
				return Ok(Some(format!("started {} (it was not running)", bin.display())));
			}
		}
		Err(format!("spawned {} but it did not come up within ~3 s.\n{hint}", bin.display()))
	}

	/// POST `/api/run`. `body` is passed through verbatim — either a bare
	/// program `{"ops": [...]}` or `{"program": {...}, "session": "..."}`.
	pub fn run_program(&self, body: &Value) -> Result<RunResponse, String> {
		let mut resp = self.auth(self.agent.post(self.url("/api/run"))).send_json(body).map_err(transport)?;
		let status = resp.status();
		let text = resp.body_mut().read_to_string().map_err(|e| format!("reading /api/run response: {e}"))?;
		if !status.is_success() {
			return Err(http_error("/api/run", status.as_u16(), &text));
		}
		serde_json::from_str(&text).map_err(|e| format!("unparseable /api/run response: {e}"))
	}

	/// GET `/api/catalog` — the standard-parts families, as raw JSON.
	pub fn catalog(&self) -> Result<Value, String> {
		let mut resp = self.auth(self.agent.get(self.url("/api/catalog"))).call().map_err(transport)?;
		let status = resp.status();
		let text = resp.body_mut().read_to_string().map_err(|e| format!("reading /api/catalog response: {e}"))?;
		if !status.is_success() {
			return Err(http_error("/api/catalog", status.as_u16(), &text));
		}
		serde_json::from_str(&text).map_err(|e| format!("unparseable /api/catalog response: {e}"))
	}

	/// List every op name the kernel knows, via the `describe` op (the
	/// self-describing surface — the list comes from the kernel, not a table
	/// baked into this client).
	pub fn list_ops(&self) -> Result<Vec<String>, String> {
		let resp = self.run_program(&json!({"ops": [{"id": "ops", "op": "describe"}]}))?;
		let op = resp.report.ops.iter().find(|o| o.id == "ops").ok_or("describe report had no `ops` entry")?;
		if let Some(e) = &op.error {
			return Err(format!("describe failed — {}: {}", e.kind, e.message));
		}
		let names = op
			.measures
			.as_ref()
			.and_then(|m| m.get("ops"))
			.and_then(Value::as_array)
			.ok_or("describe measures had no `ops` array")?;
		Ok(names.iter().filter_map(Value::as_str).map(str::to_string).collect())
	}

	/// POST `/api/chat` and stream the SSE response, invoking `on_event` for
	/// every typed event. Returns `Ok(())` once the server's `done` event
	/// arrived; `Err` is a *transport* failure (server-side failures arrive as
	/// [`ChatEvent::Error`] followed by `done`).
	pub fn chat(&self, messages: &[ChatTurn], session: &str, mut on_event: impl FnMut(ChatEvent)) -> Result<(), String> {
		let body = json!({"messages": messages, "session": session});
		let resp = self.auth(self.agent.post(self.url("/api/chat"))).send_json(&body).map_err(transport)?;
		let status = resp.status();
		if !status.is_success() {
			let text = resp.into_body().read_to_string().unwrap_or_default();
			return Err(http_error("/api/chat", status.as_u16(), &text));
		}
		let reader = BufReader::new(resp.into_body().into_reader());
		let mut parser = SseParser::new();
		for line in reader.lines() {
			let line = line.map_err(|e| format!("chat stream cut: {e}"))?;
			if let Some(event) = parser.push_line(&line) {
				let is_done = matches!(event, ChatEvent::Done(_));
				on_event(event);
				if is_done {
					return Ok(());
				}
			}
		}
		Err("chat stream ended without a `done` event".to_string())
	}
}

/// Human-readable transport error for a failed request.
fn transport(e: ureq::Error) -> String {
	match &e {
		ureq::Error::Io(io) if io.kind() == std::io::ErrorKind::ConnectionRefused => {
			format!("connection refused: {e}")
		}
		_ => format!("transport error: {e}"),
	}
}

/// Classify a health-probe failure (connection-refused triggers auto-spawn).
fn classify_health(e: ureq::Error) -> HealthError {
	match &e {
		ureq::Error::Io(io) if io.kind() == std::io::ErrorKind::ConnectionRefused => HealthError::Refused,
		ureq::Error::ConnectionFailed => HealthError::Refused,
		_ => HealthError::Other(e.to_string()),
	}
}

/// Non-2xx HTTP response → message, preferring the server's `{"error": ...}`.
fn http_error(path: &str, status: u16, body: &str) -> String {
	let detail = serde_json::from_str::<Value>(body)
		.ok()
		.and_then(|v| v.get("error").and_then(Value::as_str).map(str::to_string))
		.unwrap_or_else(|| body.chars().take(200).collect());
	format!("{path} returned HTTP {status}: {detail}")
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
	use super::*;

	/// The full event vocabulary in one canned stream — including an axum
	/// keep-alive comment, a CRLF-terminated frame, a multi-line `data:` field
	/// (SSE joins with `\n`), an unknown event (tolerated → dropped), an
	/// unknown field (`id:` — ignored), and a trailing unterminated frame.
	#[test]
	fn parses_canned_multi_event_stream() {
		let raw = concat!(
			": keep-alive ping\n",
			"\n",
			"event: thinking\n",
			"data: {\"delta\":\"sizing the box\"}\n",
			"\n",
			"event: text\r\n",
			"data: {\"delta\":\"Here is\"}\r\n",
			"\r\n",
			"event: text\n",
			"id: 42\n",
			"data: {\"delta\":\" a 10 mm cube.\"}\n",
			"\n",
			"event: tool\n",
			"data: {\"state\":\"running\",\"name\":\"run_program\",\"ops\":5,\"program\":{\"ops\":[]}}\n",
			"\n",
			"event: tool\n",
			"data: {\"state\":\"done\",\"name\":\"run_program\",\"ops\":5,\"ok\":false,\"error\":\"InvalidParam: op 'c' r must be > 0\"}\n",
			"\n",
			"event: refresh\n",
			"data: {\"artifacts\":[{\"file\":\"part.stl\",\"url\":\"/api/mesh/part.stl?session=default\",\"kind\":\"stl\"}],\n",
			"data: \"receipt\":{\"route\":\"exact\",\"tris\":12,\"watertight\":true}}\n",
			"\n",
			"event: shiny_new_event\n",
			"data: {\"anything\":1}\n",
			"\n",
			"event: chat_disabled\n",
			"data: {\"message\":\"chat disabled — set ANTHROPIC_API_KEY\"}\n",
			"\n",
			"event: error\n",
			"data: {\"message\":\"Anthropic API error 529: overloaded\"}\n",
			"\n",
			"event: done\n",
			"data: {\"stop_reason\":\"end_turn\"}\n",
		);
		let events = parse_sse_lines(raw);
		let expected = vec![
			ChatEvent::Thinking("sizing the box".into()),
			ChatEvent::Text("Here is".into()),
			ChatEvent::Text(" a 10 mm cube.".into()),
			ChatEvent::Tool { state: "running".into(), name: "run_program".into(), ops: 5, ok: None, error: None },
			ChatEvent::Tool {
				state: "done".into(),
				name: "run_program".into(),
				ops: 5,
				ok: Some(false),
				error: Some("InvalidParam: op 'c' r must be > 0".into()),
			},
			ChatEvent::Refresh {
				artifacts: vec!["part.stl".into()],
				receipt: Some(json!({"route": "exact", "tris": 12, "watertight": true})),
			},
			ChatEvent::Disabled("chat disabled — set ANTHROPIC_API_KEY".into()),
			ChatEvent::Error("Anthropic API error 529: overloaded".into()),
			ChatEvent::Done("end_turn".into()),
		];
		assert_eq!(
			events, expected,
			"canned SSE stream must parse to exactly the typed protocol (unknown events dropped, \
			 multi-line data joined, CRLF + comments + unknown fields tolerated, trailing frame flushed)"
		);
	}

	/// The agent-harness frames of commit 0a35d10: `tasks` (whole-plan
	/// re-sends) and `subagent` (lifecycle with optional detail), interleaved
	/// with an unknown event to confirm forward tolerance still holds.
	#[test]
	fn parses_agent_harness_frames() {
		let raw = concat!(
			"event: tasks\n",
			"data: {\"tasks\":[{\"content\":\"design the bracket\",\"status\":\"completed\"},",
			"{\"content\":\"cut the bolt circle\",\"status\":\"in_progress\"},",
			"{\"content\":\"export STEP\",\"status\":\"pending\"}]}\n",
			"\n",
			"event: subagent\n",
			"data: {\"name\":\"bracket-agent\",\"state\":\"started\"}\n",
			"\n",
			"event: some_future_frame\n",
			"data: {\"v\":2}\n",
			"\n",
			"event: subagent\n",
			"data: {\"name\":\"bracket-agent\",\"state\":\"tool\",\"detail\":\"run_program (7 ops) ok\"}\n",
			"\n",
			"event: subagent\n",
			"data: {\"name\":\"bracket-agent\",\"state\":\"done\",\"detail\":\"volume 5210.3 exact\"}\n",
			"\n",
			"event: subagent\n",
			"data: {\"name\":\"lid-agent\",\"state\":\"error\",\"detail\":\"turn budget exhausted\"}\n",
			"\n",
			"event: done\n",
			"data: {\"stop_reason\":\"end_turn\"}\n",
		);
		let events = parse_sse_lines(raw);
		let expected = vec![
			ChatEvent::Tasks(vec![
				TaskItem { content: "design the bracket".into(), status: "completed".into() },
				TaskItem { content: "cut the bolt circle".into(), status: "in_progress".into() },
				TaskItem { content: "export STEP".into(), status: "pending".into() },
			]),
			ChatEvent::Subagent { name: "bracket-agent".into(), state: "started".into(), detail: None },
			ChatEvent::Subagent {
				name: "bracket-agent".into(),
				state: "tool".into(),
				detail: Some("run_program (7 ops) ok".into()),
			},
			ChatEvent::Subagent {
				name: "bracket-agent".into(),
				state: "done".into(),
				detail: Some("volume 5210.3 exact".into()),
			},
			ChatEvent::Subagent { name: "lid-agent".into(), state: "error".into(), detail: Some("turn budget exhausted".into()) },
			ChatEvent::Done("end_turn".into()),
		];
		assert_eq!(
			events, expected,
			"agent-harness frames must parse typed (tasks whole-plan, subagent lifecycle with optional \
			 detail) while unknown events stay tolerated (some_future_frame dropped, stream continues)"
		);
	}

	/// The incremental parser must produce identical events no matter how the
	/// byte stream is chopped into lines (network chunking never aligns with
	/// frames) — and malformed JSON on a KNOWN event surfaces as an Error
	/// event instead of being swallowed.
	#[test]
	fn incremental_parse_and_malformed_data() {
		let mut p = SseParser::new();
		let mut got = Vec::new();
		for line in ["event: text", "data: {\"delta\":\"hi\"}", "", "event: text", "data: {not json", "", "event: done", "data: {\"stop_reason\":\"end_turn\"}", ""] {
			if let Some(e) = p.push_line(line) {
				got.push(e);
			}
		}
		assert_eq!(got.len(), 3, "three frames → three events, got {got:?}");
		assert_eq!(got[0], ChatEvent::Text("hi".into()));
		assert!(
			matches!(&got[1], ChatEvent::Error(m) if m.contains("unparseable `text`")),
			"malformed data on a known event must surface honestly, got {:?}",
			got[1]
		);
		assert_eq!(got[2], ChatEvent::Done("end_turn".into()));
	}

	/// Report-line rendering: exactly what the report carries — measures
	/// one-lined, file basename, structured error — never invented fields.
	#[test]
	fn op_line_renders_report_verbatim() {
		let ok_op = OpReport {
			id: "e".into(),
			ok: true,
			measures: Some(json!({"route": "exact", "triangles": 12, "watertight": true})),
			file: Some("/abs/out/default/part.stl".into()),
			error: None,
		};
		let bad_op = OpReport {
			id: "c".into(),
			ok: false,
			measures: None,
			file: None,
			error: Some(OpError { kind: "invalid_param".into(), message: "r must be > 0".into() }),
		};
		let names = op_names_of(&json!({"ops": [{"id": "e", "op": "export_stl"}, {"id": "c", "op": "cylinder"}]}));
		assert_eq!(
			(op_line(&ok_op, names.get("e").map(String::as_str)), op_line(&bad_op, names.get("c").map(String::as_str))),
			(
				"✓ e · export_stl · route=exact triangles=12 watertight=true → part.stl".to_string(),
				"✗ c · cylinder · invalid_param: r must be > 0".to_string()
			),
			"receipt lines must render the report verbatim (marks, measures k=v, file basename, error kind+message)"
		);
	}
}
