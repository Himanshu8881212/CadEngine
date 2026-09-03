// Copyright (c) LMCAD. Licensed under the MIT License.

//! # studio-mcp — the LMCAD MCP stdio server
//!
//! Exposes the hybrid kernel as native [Model Context Protocol] tools so an
//! MCP client (Claude Code) can spawn the `lmcad-mcp` binary and call the
//! geometry engine **in-process** — no HTTP, no API key, no async runtime.
//!
//! Transport: JSON-RPC 2.0 over stdio, one message per line. Requests are
//! answered on stdout (single-line JSON, flushed per message); notifications
//! get no response; logs go to stderr only. The protocol subset implemented
//! here is exactly what a tools-only MCP server needs: `initialize`, `ping`,
//! `tools/list`, `tools/call`, plus honest JSON-RPC errors (`-32700` parse,
//! `-32601` method not found, `-32602` invalid params).
//!
//! Tools (all answers are the kernel's own receipts — this layer adds
//! transport, never geometry claims):
//!
//! | tool | what |
//! |---|---|
//! | `run_program` | execute a work-order JSON program via [`kernel_api::run_program_with_input_base`]; exports land under `<repo_root>/studio_out/mcp/` |
//! | `describe_api` | the live op catalogue + per-op `API.md` sections (via [`studio_server::apidoc`]) |
//! | `run_assembly` | execute a repo-relative `.lmcasm` via [`kernel_api::run_assembly`] |
//! | `ace_fea` | ACE's hex8 reference FEA on LMCAD geometry, out-of-process via `tools/ace_fea_runner.py` (env `ACE_PYTHON`) |
//! | `ace_optimize` | SIMP topology optimization (OC loop over the same FEA) via `tools/ace_optimize_runner.py`, with an honest binary as-built re-analysis + gated STL |
//! | `ace_modal` | natural frequencies: ACE's hex8 free-vibration (lumped-mass) reference solver via `tools/ace_modal_runner.py` — linear, no damping; validated against the Euler-Bernoulli cantilever pin |
//! | `ace_buckling` | linear (eigenvalue) buckling load factors via `tools/ace_buckling_runner.py` — an UPPER bound on the elastic critical load; validated against the Euler column pin |
//! | `graded_infill` | stress-graded gyroid lattice infill via `tools/graded_infill_runner.py` — wall thickness follows a prior `ace_fea` stress field; meshed by the kernel's gated `mesh_density_grid` |
//! | `production_check` | Layer-1 FDM production rules on a prior `ace_fea` peak stress via `tools/production_check.py` + `tools/material_db.json` — static/creep/fatigue/temp/anisotropy allowables with every derating in the arithmetic; typical published data, verify per filament brand. The creep rule reads the material record's time × temperature TABLE and therefore needs `duration_h`; it refuses (never guesses) when the duration is absent, the temperature is above the table, or the material has no table |
//! | `render_views` | VISION: the 12-view contact sheet (6 orthos + 2 isos + bed view + 3 true sections) via `tools/render_sheet.py`, returned as an MCP **image** content item (base64 PNG) + the text receipt — the designing model literally sees the part |
//!
//! The seven Python-runner tools (`ace_*`, `graded_infill`,
//! `production_check`, `render_views`) spawn their runner (the JSON receipt
//! on its last stdout line is the contract) with a wall-clock timeout; the
//! child is killed on expiry and a payload without an `"ok"` key is refused.
//! `render_views` additionally reads the receipt's PNG and answers it as
//! image content (re-rendering once at a smaller pixel cap when the sheet
//! exceeds [`MAX_SHEET_BYTES`]).

use std::path::{Component, Path, PathBuf};
use std::time::Duration;

use base64::Engine as _;
use serde_json::{json, Value};

/// Protocol version answered when the client does not name one. MCP servers
/// echo the client's requested version when they support it; this server is
/// version-agnostic over the tools-only subset, so it echoes whatever the
/// client asks for and falls back to this revision.
pub const DEFAULT_PROTOCOL_VERSION: &str = "2025-06-18";

/// Hard cap on a tool-result text, in bytes (~60 KB). A full kernel report
/// for a large program can exceed what a model context comfortably carries;
/// beyond the cap the text is truncated at a UTF-8 boundary with an explicit
/// marker, never silently.
pub const MAX_RESULT_BYTES: usize = 60 * 1024;

/// Default ACE Python interpreter, relative to `$HOME` (override with env
/// `ACE_PYTHON`): ACE is typically pip-installed `-e` into a miniconda base.
/// Resolved by [`default_ace_python`]; a bare relative path if `HOME` is unset.
pub const DEFAULT_ACE_PYTHON_REL: &str = "miniconda3/bin/python3";

/// The default ACE interpreter path: `$HOME/miniconda3/bin/python3`.
pub fn default_ace_python() -> PathBuf {
	match std::env::var_os("HOME") {
		Some(home) => PathBuf::from(home).join(DEFAULT_ACE_PYTHON_REL),
		None => PathBuf::from(DEFAULT_ACE_PYTHON_REL),
	}
}

/// Actionable hint attached to every ACE spawn-level failure.
const ACE_HINT: &str = "hint: the ace_* tools need a Python with the ACE package importable — set ACE_PYTHON to such an interpreter and `pip install -e <your ACE checkout>` into it (or set ACE_ROOT)";

/// Actionable hint attached to render spawn-level failures (render needs
/// only numpy+matplotlib, not the ACE package).
const RENDER_HINT: &str = "hint: render_views needs a Python with numpy+matplotlib importable — set ACE_PYTHON to such an interpreter";

/// Actionable hint attached to production_check spawn-level failures (the
/// rules engine is pure stdlib — any Python 3 will do).
const PROD_HINT: &str = "hint: production_check needs only a Python 3 interpreter (stdlib, no ACE) — set ACE_PYTHON to any working python3";

/// Sheet-PNG payload cap (~800 KB): above it the sheet is re-rendered ONCE at
/// `max_px` 1200 for context economy (an image content item lands directly in
/// the model's context). The re-render is declared, never silent.
pub const MAX_SHEET_BYTES: u64 = 800 * 1024;

/// The MCP server: where repo-relative inputs resolve and where exports land.
pub struct Server {
	/// Repository root. Relative input paths (work-order `load_part` files,
	/// `run_assembly` `.lmcasm` paths) resolve against this directory.
	repo_root: PathBuf,
	/// Export directory for everything the tools write
	/// (`<repo_root>/studio_out/mcp` in production; created on demand).
	out_dir: PathBuf,
	/// Python interpreter for the `ace_*` runners (`ACE_PYTHON` env or
	/// [`default_ace_python`]).
	ace_python: PathBuf,
	/// Directory holding `ace_fea_runner.py` / `ace_optimize_runner.py`
	/// (`<repo_root>/tools` in production; tests point it at fakes).
	ace_runner_dir: PathBuf,
}

impl Server {
	/// A server over explicit directories (tests use a scratch `out_dir`).
	pub fn new(repo_root: PathBuf, out_dir: PathBuf) -> Server {
		let ace_python = std::env::var_os("ACE_PYTHON").map(PathBuf::from).unwrap_or_else(default_ace_python);
		let ace_runner_dir = repo_root.join("tools");
		Server { repo_root, out_dir, ace_python, ace_runner_dir }
	}

	/// Override the ACE interpreter + runner directory (tests drive the
	/// `ace_*` transport against fake runners without a real solve).
	pub fn with_ace(mut self, python: PathBuf, runner_dir: PathBuf) -> Server {
		self.ace_python = python;
		self.ace_runner_dir = runner_dir;
		self
	}

	/// The production constructor: repo root from `LMCAD_ROOT` when set, else
	/// the current directory (`.mcp.json` launches the binary with the repo
	/// root as cwd); exports land under `CADCODE_OUT_DIR` when set (a product
	/// front-end pointing exports at the USER's project instead of the engine
	/// install), else `<root>/studio_out/mcp` (created now, so a broken launch
	/// dir fails loudly at startup instead of mid-call).
	pub fn from_env() -> Result<Server, String> {
		let root = match std::env::var_os("LMCAD_ROOT") {
			Some(r) => PathBuf::from(r),
			None => std::env::current_dir().map_err(|e| format!("cannot resolve current dir: {e}"))?,
		};
		let out = match std::env::var_os("CADCODE_OUT_DIR") {
			Some(o) => PathBuf::from(o),
			None => root.join("studio_out").join("mcp"),
		};
		std::fs::create_dir_all(&out).map_err(|e| format!("cannot create out dir '{}': {e}", out.display()))?;
		Ok(Server::new(root, out))
	}

	/// Handle one raw stdin line. Returns the single-line JSON response to
	/// write, or `None` when the message is a notification (no `id` — never
	/// answered, per JSON-RPC). A line that is not valid JSON gets a `-32700`
	/// parse error with a `null` id.
	pub fn handle_line(&self, line: &str) -> Option<String> {
		let msg: Value = match serde_json::from_str(line) {
			Ok(v) => v,
			Err(e) => return Some(rpc_error(Value::Null, -32700, format!("parse error: {e}")).to_string()),
		};
		self.handle_message(&msg).map(|v| v.to_string())
	}

	/// Dispatch one parsed JSON-RPC message. `None` for notifications
	/// (id-less messages, e.g. `notifications/initialized`); otherwise the
	/// full response object (result or error) echoing the request id.
	pub fn handle_message(&self, msg: &Value) -> Option<Value> {
		let Some(id) = msg.get("id").filter(|v| !v.is_null()).cloned() else {
			return None; // notification — no response, even for unknown methods
		};
		let Some(method) = msg.get("method").and_then(Value::as_str) else {
			return Some(rpc_error(id, -32600, "invalid request: no 'method' string".to_string()));
		};
		let params = msg.get("params").cloned().unwrap_or(Value::Null);
		match method {
			"initialize" => {
				let version = params.get("protocolVersion").and_then(Value::as_str).unwrap_or(DEFAULT_PROTOCOL_VERSION);
				Some(rpc_result(
					id,
					json!({
						"protocolVersion": version,
						"capabilities": {"tools": {}},
						"serverInfo": {"name": "lmcad", "version": "1.0.0"},
					}),
				))
			}
			"ping" => Some(rpc_result(id, json!({}))),
			"tools/list" => Some(rpc_result(id, json!({"tools": tool_definitions()}))),
			"tools/call" => Some(self.tools_call(id, &params)),
			other => Some(rpc_error(id, -32601, format!("method not found: {other}"))),
		}
	}

	/// Execute one `tools/call`: dispatch on the tool name and wrap the
	/// outcome in the MCP content envelope. Unknown tool / missing name is a
	/// protocol-level `-32602`; a tool that ran but failed reports
	/// `isError: true` with the honest failure text as content. Most tools
	/// answer one text item; `render_views` answers [image, text] — image
	/// data is never truncated (a cut base64 payload would be corruption,
	/// not economy; the size cap lives in the re-render rule instead).
	fn tools_call(&self, id: Value, params: &Value) -> Value {
		let Some(name) = params.get("name").and_then(Value::as_str) else {
			return rpc_error(id, -32602, "tools/call requires params.name (string)".to_string());
		};
		let default_args = json!({});
		let args = params.get("arguments").unwrap_or(&default_args);
		let (content, is_error) = match name {
			"run_program" => text_content(self.tool_run_program(args)),
			"describe_api" => text_content(self.tool_describe_api(args)),
			"run_assembly" => text_content(self.tool_run_assembly(args)),
			"ace_fea" => text_content(self.tool_ace_fea(args)),
			"ace_optimize" => text_content(self.tool_ace_optimize(args)),
			"ace_modal" => text_content(self.tool_ace_modal(args)),
			"ace_buckling" => text_content(self.tool_ace_buckling(args)),
			"graded_infill" => text_content(self.tool_graded_infill(args)),
			"production_check" => text_content(self.tool_production_check(args)),
			"render_views" => self.tool_render_views(args),
			other => return rpc_error(id, -32602, format!("unknown tool: {other}")),
		};
		rpc_result(id, json!({"content": content, "isError": is_error}))
	}

	/// `run_program`: serialize the `program` argument back to JSON text and
	/// run it through the kernel executor. Relative input paths resolve
	/// against the repo root; exports land under the out dir. The result text
	/// is the kernel's full serialized [`kernel_api::Report`].
	fn tool_run_program(&self, args: &Value) -> (String, bool) {
		let Some(program) = args.get("program").filter(|p| p.is_object()) else {
			return ("run_program requires 'arguments.program': a JSON object of shape {\"ops\": [...]}".to_string(), true);
		};
		if let Err(e) = std::fs::create_dir_all(&self.out_dir) {
			return (format!("cannot create out dir '{}': {e}", self.out_dir.display()), true);
		}
		let report = kernel_api::run_program_with_input_base(&program.to_string(), &self.out_dir, &self.repo_root);
		(serialize_report(&report), !report.ok)
	}

	/// `describe_api`: no `op` → the live catalogue (count + names, from a
	/// real in-process `describe` run); with `op` → existence plus the op's
	/// `API.md` section, or an honest no-doc note for undocumented ops (same
	/// contract as the Studio chat harness's tool).
	fn tool_describe_api(&self, args: &Value) -> (String, bool) {
		let op = args.get("op").and_then(Value::as_str).map(str::trim).filter(|s| !s.is_empty());
		let (count, names) = match studio_server::apidoc::op_catalogue(&self.out_dir) {
			Ok(c) => c,
			Err(e) => return (format!("describe_api failed: {e}"), true),
		};
		let result = match op {
			None => json!({"count": count, "ops": names}),
			Some(op) => {
				if !names.iter().any(|n| n == op) {
					json!({
						"op": op,
						"exists": false,
						"note": format!("not one of the {count} ops — call describe_api with no arguments for the full catalogue"),
					})
				} else {
					let section = std::fs::read_to_string(self.repo_root.join("API.md"))
						.ok()
						.and_then(|md| studio_server::apidoc::extract_section(&md, op));
					match section {
						Some(doc) => json!({"op": op, "exists": true, "doc": doc}),
						None => json!({
							"op": op,
							"exists": true,
							"doc": Value::Null,
							"note": "op exists but has no doc section in API.md; probe params carefully — unknown params are silently ignored, so verify each with a measure",
						}),
					}
				}
			}
		};
		(result.to_string(), false)
	}

	/// `run_assembly`: confine the `.lmcasm` path to the repo root (absolute
	/// paths and `..` refused), then run the full assembly pipeline. Optional
	/// `tol` / `voxel` / `window` (mm) override the documented CLI defaults
	/// ([`kernel_api::AsmOptions::default`]) — clamped to sane positive ranges
	/// so a typo cannot request a 1 µm voxel grid over a whole assembly.
	fn tool_run_assembly(&self, args: &Value) -> (String, bool) {
		let Some(rel) = args.get("asm_path").and_then(Value::as_str) else {
			return ("run_assembly requires 'arguments.asm_path': a .lmcasm path relative to the repo root".to_string(), true);
		};
		let path = match confine(&self.repo_root, rel) {
			Ok(p) => p,
			Err(e) => return (e, true),
		};
		if let Err(e) = std::fs::create_dir_all(&self.out_dir) {
			return (format!("cannot create out dir '{}': {e}", self.out_dir.display()), true);
		}
		let mut opts = kernel_api::AsmOptions::default();
		if let Some(tol) = args.get("tol").and_then(Value::as_f64) {
			opts.tol = tol.clamp(0.001, 1.0);
		}
		if let Some(voxel) = args.get("voxel").and_then(Value::as_f64) {
			opts.voxel = voxel.clamp(0.05, 5.0);
		}
		if let Some(window) = args.get("window").and_then(Value::as_f64) {
			opts.window = window.clamp(0.0, 100.0);
		}
		let report = kernel_api::run_assembly(&path, &self.out_dir, &opts);
		(serialize_report(&report), !report.ok)
	}

	/// `ace_fea`: one reference-FEA job. `mesh` selects the discretization:
	/// `"voxel"` (default, back-compat) runs the hex8 grid runner; `"body_fitted"`
	/// runs the conforming tet10 runner (`ace_fea_tet_runner.py`) — a true
	/// curved-fillet mesh that resolves stress concentrations the voxel grid
	/// under-reads, with a different geometry block (stl / specimen + elem_size_mm
	/// instead of ops/npy + voxel_mm). Wall clock capped by the optional
	/// `timeout_s` argument (default 300 s, clamped to [1, 3600]).
	fn tool_ace_fea(&self, args: &Value) -> (String, bool) {
		let timeout = args.get("timeout_s").and_then(Value::as_f64).unwrap_or(300.0).clamp(1.0, 3600.0);
		let runner = match args.get("mesh").and_then(Value::as_str) {
			Some("body_fitted") => "ace_fea_tet_runner.py",
			Some(other) if other != "voxel" => {
				return (format!("ace_fea: unknown mesh '{other}' — expected 'voxel' or 'body_fitted'"), true);
			}
			_ => "ace_fea_runner.py",
		};
		self.run_ace_job("ace_fea", runner, args, Duration::from_secs_f64(timeout), ACE_HINT)
	}

	/// `ace_optimize`: one SIMP optimization job. Wall clock = the job's
	/// `time_budget_s` (default 600 s, clamped to [1, 7200]) + 120 s of grace
	/// for sampling, the as-built re-analysis and the gated STL.
	fn tool_ace_optimize(&self, args: &Value) -> (String, bool) {
		let budget = args.get("time_budget_s").and_then(Value::as_f64).unwrap_or(600.0).clamp(1.0, 7200.0);
		self.run_ace_job("ace_optimize", "ace_optimize_runner.py", args, Duration::from_secs_f64(budget + 120.0), ACE_HINT)
	}

	/// `ace_modal`: one hex8 free-vibration (natural-frequency) job. Wall
	/// clock capped by the optional `timeout_s` argument (default 300 s,
	/// clamped to [1, 3600]) — the eigensolve at ~70k DOF runs ~80 s.
	fn tool_ace_modal(&self, args: &Value) -> (String, bool) {
		let timeout = args.get("timeout_s").and_then(Value::as_f64).unwrap_or(300.0).clamp(1.0, 3600.0);
		self.run_ace_job("ace_modal", "ace_modal_runner.py", args, Duration::from_secs_f64(timeout), ACE_HINT)
	}

	/// `ace_buckling`: one hex8 linear (eigenvalue) buckling job. Wall clock
	/// capped by the optional `timeout_s` argument (default 300 s, clamped to
	/// [1, 3600]) — pre-stress solve + eigensolve at ~60k DOF runs ~60 s.
	fn tool_ace_buckling(&self, args: &Value) -> (String, bool) {
		let timeout = args.get("timeout_s").and_then(Value::as_f64).unwrap_or(300.0).clamp(1.0, 3600.0);
		self.run_ace_job("ace_buckling", "ace_buckling_runner.py", args, Duration::from_secs_f64(timeout), ACE_HINT)
	}

	/// `graded_infill`: one stress-graded gyroid infill job. Wall clock capped
	/// by the optional `timeout_s` argument (default 300 s, clamped to
	/// [1, 3600]) — covers grading plus up to three kernel meshing rungs.
	fn tool_graded_infill(&self, args: &Value) -> (String, bool) {
		let timeout = args.get("timeout_s").and_then(Value::as_f64).unwrap_or(300.0).clamp(1.0, 3600.0);
		self.run_ace_job("graded_infill", "graded_infill_runner.py", args, Duration::from_secs_f64(timeout), ACE_HINT)
	}

	/// `production_check`: one Layer-1 FDM production-rules job (pure-stdlib
	/// rules engine — instant). Wall clock capped by the optional `timeout_s`
	/// argument (default 60 s, clamped to [1, 600]). NOTE: the receipt's `ok`
	/// is the VERDICT — a part failing its rules answers `isError: true` with
	/// the full per-rule receipt attached, by design (it is a gate).
	fn tool_production_check(&self, args: &Value) -> (String, bool) {
		let timeout = args.get("timeout_s").and_then(Value::as_f64).unwrap_or(60.0).clamp(1.0, 600.0);
		self.run_ace_job("production_check", "production_check.py", args, Duration::from_secs_f64(timeout), PROD_HINT)
	}

	/// Shared Python-runner transport for the `ace_*` / `production_check`
	/// tools: write the model's arguments as a job JSON under `<out_dir>/ace/`
	/// (with a server-managed `out_dir` injected so a caller can never write
	/// outside the export tree), then run it via [`Server::run_python_runner`]
	/// with the tool's actionable spawn-failure hint.
	fn run_ace_job(&self, tool: &str, runner_file: &str, args: &Value, timeout: Duration, hint: &str) -> (String, bool) {
		let ace_dir = self.out_dir.join("ace");
		if let Err(e) = std::fs::create_dir_all(&ace_dir) {
			return (format!("cannot create ACE out dir '{}': {e}", ace_dir.display()), true);
		}
		let stamp = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_nanos()).unwrap_or(0);
		let stem = format!("{tool}_{stamp}");
		let mut job = if args.is_object() { args.clone() } else { json!({}) };
		job["out_dir"] = json!(ace_dir.join(&stem).display().to_string());
		let job_path = ace_dir.join(format!("{stem}.json"));
		if let Err(e) = std::fs::write(&job_path, job.to_string()) {
			return (format!("cannot write job file '{}': {e}", job_path.display()), true);
		}
		self.run_python_runner(tool, runner_file, &job_path, timeout, hint)
	}

	/// Spawn `ACE_PYTHON <tools/runner_file> <job.json>`, kill the child on
	/// timeout, and adopt ACE's orchestrator rule: the LAST non-empty stdout
	/// line must be one JSON object carrying an `"ok"` key — anything else is
	/// refused. `isError` = spawn failure | timeout | unparseable receipt |
	/// `ok:false`. Shared by the `ace_*` tools and `render_views`.
	fn run_python_runner(&self, tool: &str, runner_file: &str, job_path: &Path, timeout: Duration, hint: &str) -> (String, bool) {
		if !self.ace_python.exists() {
			return (format!("python interpreter not found at '{}' — {hint}", self.ace_python.display()), true);
		}
		let runner = self.ace_runner_dir.join(runner_file);
		if !runner.exists() {
			return (format!("runner script not found at '{}' — expected under <repo_root>/tools/", runner.display()), true);
		}
		let spawned = std::process::Command::new(&self.ace_python)
			.arg(&runner)
			.arg(job_path)
			.current_dir(&self.repo_root)
			.stdin(std::process::Stdio::null())
			.stdout(std::process::Stdio::piped())
			.stderr(std::process::Stdio::piped())
			.spawn();
		let mut child = match spawned {
			Ok(c) => c,
			Err(e) => return (format!("cannot spawn '{}': {e} — {hint}", self.ace_python.display()), true),
		};
		// Reader threads keep both pipes drained (a full pipe would deadlock
		// the child); results come back over channels so a timeout never
		// blocks on a pipe an orphaned grandchild still holds open.
		let read_to_channel = |stream: Option<Box<dyn std::io::Read + Send>>| {
			let (tx, rx) = std::sync::mpsc::channel::<String>();
			if let Some(mut s) = stream {
				std::thread::spawn(move || {
					let mut buf = String::new();
					let _ = std::io::Read::read_to_string(&mut s, &mut buf);
					let _ = tx.send(buf);
				});
			}
			rx
		};
		let stdout_rx = read_to_channel(child.stdout.take().map(|s| Box::new(s) as Box<dyn std::io::Read + Send>));
		let stderr_rx = read_to_channel(child.stderr.take().map(|s| Box::new(s) as Box<dyn std::io::Read + Send>));
		let deadline = std::time::Instant::now() + timeout;
		let timed_out = loop {
			match child.try_wait() {
				Ok(Some(_status)) => break false,
				Ok(None) if std::time::Instant::now() >= deadline => {
					let _ = child.kill();
					let _ = child.wait();
					break true;
				}
				Ok(None) => std::thread::sleep(Duration::from_millis(25)),
				Err(e) => return (format!("{tool}: failed to wait on the runner: {e}"), true),
			}
		};
		let grace = Duration::from_secs(2);
		let stdout = stdout_rx.recv_timeout(grace).unwrap_or_default();
		let stderr = stderr_rx.recv_timeout(grace).unwrap_or_default();
		if timed_out {
			return (
				format!(
					"{tool} timed out after {:.0} s — child killed, no receipt trusted. stderr tail: {}",
					timeout.as_secs_f64(),
					tail(&stderr, 2000)
				),
				true,
			);
		}
		let last = stdout.lines().rev().find(|l| !l.trim().is_empty()).unwrap_or("");
		let receipt: Option<Value> = serde_json::from_str(last).ok();
		match receipt {
			Some(v) if v.get("ok").is_some() => {
				let ok = v["ok"] == json!(true);
				(last.to_string(), !ok)
			}
			_ => (
				format!(
					"{tool}: the runner produced no JSON receipt with an \"ok\" key on its last stdout line — refusing the payload. stdout tail: {} | stderr tail: {}",
					tail(&stdout, 1000),
					tail(&stderr, 2000)
				),
				true,
			),
		}
	}

	/// `render_views` — VISION IN THE LOOP: render the 12-view contact sheet
	/// for an STL and answer it as an MCP **image** content item (base64 PNG)
	/// plus the runner's text receipt, so the sheet lands directly in the
	/// calling model's context. Input is either `stl` (a path confined to the
	/// out dir) or `program`+`solid` (run the ops now, export a temp STL,
	/// render, delete the temp). If the sheet exceeds [`MAX_SHEET_BYTES`] it
	/// is re-rendered ONCE at `max_px` 1200 (declared in the receipt's `px`).
	fn tool_render_views(&self, args: &Value) -> (Vec<Value>, bool) {
		let fail = |msg: String| (vec![json!({"type": "text", "text": truncate_result(msg)})], true);
		let stamp = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_nanos()).unwrap_or(0);
		// Resolve the STL: an existing export, or a program run right now.
		let (stl, temp_stl) = if let Some(rel) = args.get("stl").and_then(Value::as_str) {
			match self.confine_to_out_dir(rel) {
				Ok(p) => (p, false),
				Err(e) => return fail(e),
			}
		} else if let Some(program) = args.get("program").filter(|p| p.is_object()) {
			let Some(solid) = args.get("solid").and_then(Value::as_str).filter(|s| !s.trim().is_empty()) else {
				return fail("render_views with 'program' also requires 'solid': the op id of the solid to export and render".to_string());
			};
			let mut prog = program.clone();
			let Some(ops) = prog.get_mut("ops").and_then(Value::as_array_mut) else {
				return fail("render_views 'program' must be a work-order object of shape {\"ops\": [...]}".to_string());
			};
			let stl_name = format!("render_views_{stamp}.stl");
			ops.push(json!({"id": "__render_views_stl", "op": "export_stl", "in": solid, "file": stl_name}));
			if let Err(e) = std::fs::create_dir_all(&self.out_dir) {
				return fail(format!("cannot create out dir '{}': {e}", self.out_dir.display()));
			}
			let report = kernel_api::run_program_with_input_base(&prog.to_string(), &self.out_dir, &self.repo_root);
			if !report.ok {
				return fail(format!("render_views: the program failed before rendering — {}", serialize_report(&report)));
			}
			let file = report.ops.iter().find(|o| o.id == "__render_views_stl").and_then(|o| o.file.clone());
			let Some(file) = file else {
				return fail("render_views: the program ran but the appended export_stl op reported no file".to_string());
			};
			let p = PathBuf::from(&file);
			(if p.is_absolute() { p } else { self.out_dir.join(p) }, true)
		} else {
			return fail("render_views requires either 'stl' (an out-dir path, e.g. a run_program export) or 'program' + 'solid'".to_string());
		};
		let cleanup = |keep: bool| {
			if temp_stl && !keep {
				let _ = std::fs::remove_file(&stl);
			}
		};
		// Render (once at 1600 px; once more at 1200 px if the PNG is heavy).
		let stem = stl.file_stem().map(|s| s.to_string_lossy().into_owned()).unwrap_or_else(|| "part".to_string());
		let sheet = stl.with_file_name(format!("{stem}_sheet.png"));
		let render_dir = self.out_dir.join("render");
		if let Err(e) = std::fs::create_dir_all(&render_dir) {
			cleanup(false);
			return fail(format!("cannot create render dir '{}': {e}", render_dir.display()));
		}
		let timeout = args.get("timeout_s").and_then(Value::as_f64).unwrap_or(180.0).clamp(1.0, 3600.0);
		let mut receipt = String::new();
		for max_px in [1600u32, 1200] {
			let mut job = json!({
				"stl": stl.display().to_string(),
				"out": sheet.display().to_string(),
				"max_px": max_px,
			});
			if let Some(bd) = args.get("build_dir").filter(|v| v.is_array()) {
				job["build_dir"] = bd.clone();
			}
			if let Some(sec) = args.get("sections").filter(|v| v.is_object()) {
				job["sections"] = sec.clone();
			}
			let job_path = render_dir.join(format!("render_views_{stamp}_{max_px}.json"));
			if let Err(e) = std::fs::write(&job_path, job.to_string()) {
				cleanup(false);
				return fail(format!("cannot write render job '{}': {e}", job_path.display()));
			}
			let (text, is_error) = self.run_python_runner("render_views", "render_sheet.py", &job_path, Duration::from_secs_f64(timeout), RENDER_HINT);
			if is_error {
				cleanup(false);
				return fail(text);
			}
			receipt = text;
			let bytes = std::fs::metadata(&sheet).map(|m| m.len()).unwrap_or(0);
			if bytes <= MAX_SHEET_BYTES {
				break; // small enough — a 1200 px retry only happens above the cap
			}
		}
		cleanup(false);
		let png = match std::fs::read(&sheet) {
			Ok(b) => b,
			Err(e) => return fail(format!("render_views: receipt was ok but the sheet '{}' cannot be read: {e}", sheet.display())),
		};
		if !png.starts_with(&[0x89, b'P', b'N', b'G', 0x0d, 0x0a, 0x1a, 0x0a]) {
			return fail(format!("render_views: '{}' is not a PNG (bad signature) — refusing to return it as image content", sheet.display()));
		}
		if png.len() as u64 > MAX_SHEET_BYTES {
			receipt.push_str("\nnote: the sheet still exceeds 800 KB after the 1200 px re-render — returned anyway; expect a heavy context item");
		}
		let data = base64::engine::general_purpose::STANDARD.encode(&png);
		(
			vec![
				json!({"type": "image", "data": data, "mimeType": "image/png"}),
				json!({"type": "text", "text": truncate_result(receipt)}),
			],
			false,
		)
	}

	/// Resolve an `stl` argument for READING: relative paths join the out dir
	/// first, then the repo root (`..` refused in both); absolute paths must
	/// canonicalize under one of those two trees. The repo-root fallback exists
	/// because rendering a campaign's shipped `parts/*.stl` is the tool's most
	/// common use, and the out-dir-only rule rejected exactly that with an
	/// error that named no fix (friction rated_desk_hook F2, 2026-08-27).
	/// Rendering is read-only, so admitting the repo tree loosens nothing the
	/// sandbox protects (writes still land under the out dir).
	fn confine_to_out_dir(&self, rel_or_abs: &str) -> Result<PathBuf, String> {
		let p = Path::new(rel_or_abs);
		let out_canon = self
			.out_dir
			.canonicalize()
			.map_err(|e| format!("out dir '{}' is unavailable: {e}", self.out_dir.display()))?;
		let repo_canon = self.repo_root.canonicalize().ok();
		let roots: Vec<&Path> =
			std::iter::once(out_canon.as_path()).chain(repo_canon.as_deref()).collect();
		let candidates: Vec<PathBuf> = if p.is_absolute() {
			vec![p.to_path_buf()]
		} else {
			let mut v = vec![confine(&self.out_dir, rel_or_abs)?];
			if repo_canon.is_some() {
				v.push(confine(&self.repo_root, rel_or_abs)?);
			}
			v
		};
		for candidate in &candidates {
			if let Ok(canon) = candidate.canonicalize() {
				if roots.iter().any(|r| canon.starts_with(r)) {
					return Ok(canon);
				}
				return Err(format!(
					"stl path must live under the out dir '{}' or the repo root '{}': '{rel_or_abs}'",
					self.out_dir.display(),
					self.repo_root.display()
				));
			}
		}
		Err(format!(
			"stl '{rel_or_abs}' not found under the out dir '{}' or the repo root '{}' — pass a path relative to either tree (e.g. a run_program export, or a campaign's parts/<name>.stl)",
			self.out_dir.display(),
			self.repo_root.display()
		))
	}
}

/// One text content item (the envelope for every text-only tool).
fn text_content((text, is_error): (String, bool)) -> (Vec<Value>, bool) {
	(vec![json!({"type": "text", "text": truncate_result(text)})], is_error)
}

/// Last `max_bytes` of a string, cut at a UTF-8 boundary (for error tails).
fn tail(s: &str, max_bytes: usize) -> &str {
	if s.len() <= max_bytes {
		return s.trim_end();
	}
	let mut start = s.len() - max_bytes;
	while !s.is_char_boundary(start) {
		start += 1;
	}
	s[start..].trim_end()
}

/// The ten tool definitions served by `tools/list`. Descriptions carry the
/// doctrine the driving model must hold: every measure carries provenance,
/// exports name their route (`exact` vs `voxel_healed`) and watertightness,
/// and nothing beyond the kernel's (or the ACE solver's) report may be
/// claimed — including the ACE tools' hex8/homogenization caveats. The
/// `render_views` description carries the vision doctrine: LOOK at every
/// part after every geometry change, before declaring an iteration done.
pub fn tool_definitions() -> Vec<Value> {
	vec![
		json!({
			"name": "run_program",
			"description": "Execute an LMCAD work-order JSON program: ops with unique ids referencing prior ids; the ONLY way to build/measure/export geometry. Returns the kernel's full report (per-op ok, measures with provenance, files, machine-matchable errors). Exports land under studio_out/mcp/. Doctrine: measures carry provenance and exports name their route (exact vs voxel_healed) plus watertightness — never claim beyond the report. Unknown params are silently ignored, so verify op names/params with describe_api instead of guessing.",
			"inputSchema": {
				"type": "object",
				"properties": {
					"program": {"type": "object", "description": "{\"ops\":[...]}"}
				},
				"required": ["program"]
			}
		}),
		json!({
			"name": "describe_api",
			"description": "Discover the op surface: no args → all op names + count; {op:\"name\"} → existence + that op's API.md documentation section (param table + example), or an honest no-doc note. Consult this BEFORE guessing parameters — run_program silently ignores unknown params.",
			"inputSchema": {
				"type": "object",
				"properties": {
					"op": {"type": "string"}
				}
			}
		}),
		json!({
			"name": "run_assembly",
			"description": "Execute an LMCAD assembly (.lmcasm path relative to the repo root): re-solves mates (residual gated at 1e-6; receipts carry per_mate residuals + a numeric DOF report — under-constrained assemblies say so), exports merged + per-instance STLs, the AP214 STEP assembly (B-rep instances; mesh instances honestly skipped) and BOM under studio_out/mcp/, runs the contact/clearance scan, returns the full report. Instance sources: .lmcpart path, inline part, sub-assembly asm_path, or a mesh file ({\"mesh\": \"part.stl\"} — the bridge for program-built/imported parts). Optional tol/voxel/window (mm) override the measurement defaults (0.05/0.4/1.0). Doctrine: per-instance exports name their route (exact vs voxel_healed); trust only the report's measures.",
			"inputSchema": {
				"type": "object",
				"properties": {
					"asm_path": {"type": "string"},
					"tol": {"type": "number", "description": "chord tolerance mm for exact tessellation (default 0.05)"},
					"voxel": {"type": "number", "description": "voxel size mm for organic/mesh parts (default 0.4)"},
					"window": {"type": "number", "description": "contact-scan proximity window mm (default 1.0)"}
				},
				"required": ["asm_path"]
			}
		}),
		json!({
			"name": "ace_fea",
			"description": "Real structural physics: ACE's benchmark-validated hex8 linear-elastic reference FEA on LMCAD geometry, run out-of-process in Python (env ACE_PYTHON; needs `pip install -e ~/Work/ACE`). Geometry either as LMCAD ops (ops + solid naming the op id to voxelize + shape [nx,ny,nz]) or as npy (absolute path of an existing (nx,ny,nz) density grid); grid fixed by voxel_mm (+ optional origin_mm). Boundary conditions are VOXEL-REGION selectors, NOT B-rep faces — selector types (all mm): {type:'all'} | {type:'bbox',min_mm:[x,y,z],max_mm:[x,y,z]} | {type:'plane',axis:'x'|'y'|'z',value_mm:n,side:'+'|'-'} | {type:'cylinder',axis,center_mm,radius_mm,length_mm?} | {type:'sphere',center_mm,radius_mm}; 'shell' is unsupported and refused. Only fixtures constrain DOFs (regions merely override occupancy); moment loads are accepted but NOT applied (a note says so). Example: fixtures:[{kind:'clamped',region_selector:{type:'plane',axis:'x',value_mm:0,side:'-'}}], loads:[{kind:'point',magnitude:200,direction:[0,0,-1],region_selector:{type:'bbox',min_mm:[38,0,0],max_mm:[40,15,10]}}], material:{youngs_modulus_pa:2e9,poisson:0.35,density_kg_m3:1240}. Returns the solver receipt (max_von_mises_pa, max/tip displacement m, n_active_elements, n_dof, method, notes, stress/disp .npy paths, timings). HONESTY: coarse hex8 grids under-predict peak bending stress by ~20% vs a converged mesh — echo the returned method and treat peak stress as optimistic; with simp_penalty set the stress is homogenized (rho_eff^p-scaled), not solid-material stress. Wall clock capped by timeout_s (default 300).",
			"inputSchema": {
				"type": "object",
				"properties": {
					"ops": {"type": "array", "description": "LMCAD JSON ops building the geometry (route 1)"},
					"solid": {"type": "string", "description": "op id of the solid to voxelize (route 1)"},
					"shape": {"type": "array", "description": "[nx,ny,nz] voxel grid shape (route 1)"},
					"supersample": {"type": "integer", "description": "per-axis subsamples for solid fractions, default 2 (route 1)"},
					"npy": {"type": "string", "description": "absolute path of an existing (nx,ny,nz) density .npy (route 2)"},
					"voxel_mm": {"type": "number", "description": "cubic voxel edge in mm"},
					"origin_mm": {"type": "array", "description": "world position of grid node (0,0,0), default [0,0,0]"},
					"regions": {"type": "array", "description": "[{kind:'frozen'|'fixed'|'design'|'void', selector:{...}}] — occupancy overrides only, NOT constraints"},
					"material": {"type": "object", "description": "{youngs_modulus_pa, poisson, density_kg_m3}"},
					"fixtures": {"type": "array", "description": "[{kind:'clamped'|'pinned'|'slider', region_selector:{...}, dof_constrained?}]"},
					"loads": {"type": "array", "description": "[{kind:'point'|'body'|'pressure', magnitude, direction (unit 3-vec for point/body), region_selector:{...}}]"},
					"simp_penalty": {"type": ["number", "null"], "description": "null (default) = binary as-built occupancy rho>=0.5; float p = SIMP density mode (homogenized stress)"},
					"density_floor": {"type": "number", "description": "SIMP activity floor, default 0.02"},
					"direct_solver_max_dof": {"type": "integer", "description": "default 0 = always Jacobi-CG (the direct solver needs ~10 GB at 237k DOF); CG raises instead of returning a bad solution"},
					"mesh": {"type": "string", "enum": ["voxel", "body_fitted"], "description": "discretization (default 'voxel' = hex8 grid). 'body_fitted' routes to the conforming tet10 runner: geometry becomes {stl} OR {specimen:'shouldered_bar',d,D,r,l_small,l_large} OR {specimen:'box',lx,ly,lz} PLUS elem_size_mm (NOT voxel_mm/ops/npy); it resolves the true curved fillet the voxel staircase under-reads. Body-fitted selectors are plane/box only (cylinder/sphere refused); fields are unstructured per-node ((n_nodes,) stress, (n_nodes,3) disp) not a grid; the receipt adds mesh{n_tets,n_nodes,min_corner_jacobian_mm3,volume_mm3}."},
					"stl": {"type": "string", "description": "watertight surface STL path (mesh:'body_fitted')"},
					"specimen": {"type": "string", "enum": ["shouldered_bar", "box"], "description": "parametric benchmark specimen (mesh:'body_fitted'); shouldered_bar needs d,D,r,l_small,l_large; box needs lx,ly,lz"},
					"elem_size_mm": {"type": "number", "description": "target tet edge length mm (mesh:'body_fitted')"},
					"timeout_s": {"type": "number", "description": "wall-clock cap, default 300 (clamped 1..3600)"}
				},
				"required": ["material", "fixtures"]
			}
		}),
		json!({
			"name": "ace_optimize",
			"description": "SIMP topology optimization (density filter + optimality criteria, top88 lineage) driven by ACE's hex8 FEA in SIMP mode, then an HONEST finish: one binary-occupancy re-analysis of the thresholded design (as_built — the part as printed, not the homogenized proxy) and a watertight-or-fail STL through LMCAD's gated mesher. Same geometry/grid/regions/material/fixtures/loads block as ace_fea, plus volfrac (target design-region volume fraction, required). Design domain = voxels solid in the INITIAL geometry AND region kind 'design'; frozen/fixed voxels are re-pinned to 1.0 and void to 0.0 every iteration. MARK LOAD AND FIXTURE REGIONS 'frozen' — the solver only applies loads on active elements, so an unprotected load patch can be optimized away. Returns iterations, stop_reason (converged|max_iters|time_budget), compliance_first/last (SIMP proxy — trust as_built for stress/displacement), volume_fraction_achieved, final_rho_npy, as_built{max_von_mises_pa,max_displacement_m,n_active_elements}, stl{ok,watertight,volume_mm3,num_triangles,path,mesh_upsample}, timings. HONESTY: the result is a MESH ONLY (density grid + STL) — no density-to-B-rep reconstruction exists; BCs are voxel selectors, not B-rep faces; coarse hex8 under-predicts peak bending stress ~20%; the STL may be meshed at voxel/2 or voxel/3 (reported as mesh_upsample) when the native-resolution level set pinches. Wall clock = time_budget_s (default 600) + 120 s grace.",
			"inputSchema": {
				"type": "object",
				"properties": {
					"ops": {"type": "array", "description": "LMCAD JSON ops building the design domain (route 1)"},
					"solid": {"type": "string", "description": "op id of the solid to voxelize (route 1)"},
					"shape": {"type": "array", "description": "[nx,ny,nz] voxel grid shape (route 1)"},
					"supersample": {"type": "integer", "description": "per-axis subsamples, default 2 (route 1)"},
					"npy": {"type": "string", "description": "absolute path of an existing (nx,ny,nz) density .npy (route 2)"},
					"voxel_mm": {"type": "number", "description": "cubic voxel edge in mm"},
					"origin_mm": {"type": "array", "description": "world position of grid node (0,0,0), default [0,0,0]"},
					"regions": {"type": "array", "description": "[{kind:'frozen'|'fixed'|'design'|'void', selector:{...}}]; mark load/fixture patches 'frozen'"},
					"material": {"type": "object", "description": "{youngs_modulus_pa, poisson, density_kg_m3}"},
					"fixtures": {"type": "array", "description": "[{kind:'clamped'|'pinned'|'slider', region_selector:{...}, dof_constrained?}]"},
					"loads": {"type": "array", "description": "[{kind:'point'|'body'|'pressure', magnitude, direction (unit 3-vec for point/body), region_selector:{...}}]"},
					"volfrac": {"type": "number", "description": "target volume fraction of the design region, in (0,1)"},
					"penalty": {"type": "number", "description": "SIMP penalty p, default 3.0"},
					"filter_radius_vox": {"type": "number", "description": "cone density-filter radius in voxels, default 1.5"},
					"max_iters": {"type": "integer", "description": "OC iteration cap, default 60"},
					"move": {"type": "number", "description": "OC move limit, default 0.2"},
					"density_floor": {"type": "number", "description": "SIMP floor, default 0.02"},
					"iso": {"type": "number", "description": "threshold for the as-built check + STL, default 0.5"},
					"time_budget_s": {"type": "number", "description": "optimization budget, default 600 (loop stops at 80%; MCP kills at budget+120)"},
					"direct_solver_max_dof": {"type": "integer", "description": "default 0 = always Jacobi-CG"}
				},
				"required": ["voxel_mm", "material", "fixtures", "volfrac"]
			}
		}),
		json!({
			"name": "ace_modal",
			"description": "Natural frequencies: ACE's independent hex8 free-vibration (modal) reference solver on LMCAD geometry, run out-of-process in Python (env ACE_PYTHON; needs `pip install -e ~/Work/ACE`). Same geometry/grid/regions/fixtures conventions as ace_fea (LMCAD ops+solid+shape or npy; voxel-region selectors, NOT B-rep faces); NO loads — modal analysis takes none; material density_kg_m3 MUST be > 0 (mass matrix). Solves K phi = lambda M phi on the free DOFs (lumped row-sum hex8 mass) for the lowest n_modes (default 6) natural frequencies. Returns {frequencies_hz ascending, first_mode_hz, n_modes, n_active_elements, n_dof, n_free_dof, method, fixture node-count receipts, notes, timings_s}. HONESTY: LINEAR modal analysis — no damping, no preload/stress stiffening, no plasticity, no contact; binary occupancy only (a simp_penalty is IGNORED with a note — penalized-stiffness+linear-mass modal produces spurious low-rho modes); hex8 + the one-voxel clamp layer are slightly STIFF: measured +4.0% (voxel h/8) and +0.9% (h/16) HIGH vs the Euler-Bernoulli cantilever pin, converging from above (tools/ace_modal_validation.py). An under-constrained part is refused honestly (rigid-body modes), never reported as ~0 Hz. Wall clock capped by timeout_s (default 300; ~70k DOF runs ~80 s).",
			"inputSchema": {
				"type": "object",
				"properties": {
					"ops": {"type": "array", "description": "LMCAD JSON ops building the geometry (route 1)"},
					"solid": {"type": "string", "description": "op id of the solid to voxelize (route 1)"},
					"shape": {"type": "array", "description": "[nx,ny,nz] voxel grid shape (route 1)"},
					"supersample": {"type": "integer", "description": "per-axis subsamples for solid fractions, default 2 (route 1)"},
					"npy": {"type": "string", "description": "absolute path of an existing (nx,ny,nz) density .npy (route 2)"},
					"voxel_mm": {"type": "number", "description": "cubic voxel edge in mm"},
					"origin_mm": {"type": "array", "description": "world position of grid node (0,0,0), default [0,0,0]"},
					"regions": {"type": "array", "description": "[{kind:'frozen'|'fixed'|'design'|'void', selector:{...}}] — occupancy overrides only, NOT constraints"},
					"material": {"type": "object", "description": "{youngs_modulus_pa, poisson, density_kg_m3 > 0 (REQUIRED for the mass matrix)}"},
					"fixtures": {"type": "array", "description": "[{kind:'clamped'|'pinned'|'slider', region_selector:{...}, dof_constrained?}] — must constrain at least one DOF"},
					"n_modes": {"type": "integer", "description": "number of lowest positive modes, default 6"},
					"timeout_s": {"type": "number", "description": "wall-clock cap, default 300 (clamped 1..3600)"}
				},
				"required": ["voxel_mm", "material", "fixtures"]
			}
		}),
		json!({
			"name": "ace_buckling",
			"description": "Elastic stability: ACE's independent hex8 linear (eigenvalue) buckling reference solver on LMCAD geometry, run out-of-process in Python (env ACE_PYTHON; needs `pip install -e ~/Work/ACE`). Same geometry/grid/regions/material/fixtures/loads conventions as ace_fea (voxel-region selectors, NOT B-rep faces); the loads block is the REFERENCE load case: solves the pre-stress state K u = F, assembles the geometric stiffness from recovered Gauss-point stresses, then K phi = -lambda K_g phi for the smallest positive load factors. Returns {load_factors ascending, buckling_load_factor, applied_reference_load_N, critical_load_N = smallest factor x applied reference load, n_active_elements, n_dof, n_free_dof, method, fixture/load receipts, notes, timings_s}. HONESTY: LINEAR eigenvalue buckling is an UPPER bound on the elastic critical load — no imperfections, no plasticity, no large-displacement path following; coarse hex8 over-predicts the factor (ACE's docs say 10-30%; measured +7.3% at voxel 1.0 and +3.0% at voxel 0.5 on the Euler clamped-free column pin, tools/ace_buckling_validation.py); moment loads are NOT applied (C0 hex8, noted); a purely tensile/shear state is refused honestly (no compressive buckling mode), never reported as a huge factor. Wall clock capped by timeout_s (default 300; ~60k DOF runs ~60 s).",
			"inputSchema": {
				"type": "object",
				"properties": {
					"ops": {"type": "array", "description": "LMCAD JSON ops building the geometry (route 1)"},
					"solid": {"type": "string", "description": "op id of the solid to voxelize (route 1)"},
					"shape": {"type": "array", "description": "[nx,ny,nz] voxel grid shape (route 1)"},
					"supersample": {"type": "integer", "description": "per-axis subsamples for solid fractions, default 2 (route 1)"},
					"npy": {"type": "string", "description": "absolute path of an existing (nx,ny,nz) density .npy (route 2)"},
					"voxel_mm": {"type": "number", "description": "cubic voxel edge in mm"},
					"origin_mm": {"type": "array", "description": "world position of grid node (0,0,0), default [0,0,0]"},
					"regions": {"type": "array", "description": "[{kind:'frozen'|'fixed'|'design'|'void', selector:{...}}] — occupancy overrides only, NOT constraints"},
					"material": {"type": "object", "description": "{youngs_modulus_pa, poisson [, density_kg_m3 — only needed for body loads]}"},
					"fixtures": {"type": "array", "description": "[{kind:'clamped'|'pinned'|'slider', region_selector:{...}, dof_constrained?}]"},
					"loads": {"type": "array", "description": "the REFERENCE load case: [{kind:'point'|'body'|'pressure', magnitude, direction (unit 3-vec for point/body), region_selector:{...}}]; critical_load_N = factor x this"},
					"n_modes": {"type": "integer", "description": "number of lowest positive buckling factors, default 4"},
					"timeout_s": {"type": "number", "description": "wall-clock cap, default 300 (clamped 1..3600)"}
				},
				"required": ["voxel_mm", "material", "fixtures", "loads"]
			}
		}),
		json!({
			"name": "graded_infill",
			"description": "Stress-graded gyroid lattice infill (the 'bone interior'): keeps a solid skin of shell_mm and fills the interior with a sheet-gyroid lattice whose wall thickness follows a PRIOR ace_fea von Mises field — thick walls where stress is high, thin where it coasts. Geometry as LMCAD ops (ops + solid + shape) or npy (e.g. the solid_fraction.npy an ace_fea run saved); stress_npy is REQUIRED and must be the stress_field.npy of an ace_fea run on the SAME grid — a shape mismatch is refused, never resampled. Grading: von Mises percentiles lo_pct..hi_pct over SOLID voxels map linearly to wall.min..wall.max mm (clamped outside); the |gyroid|-threshold per wall thickness is calibrated numerically so the band matches the VOLUME of a true wall of that thickness on cell_mm. The graded density is meshed by the KERNEL's gated mesher (mesh_density_grid: dual contour + heal, watertight-or-fail), escalating to voxel/2 and voxel/3 with the field re-evaluated analytically when thin walls pinch (reported as mesh_upsample). HONESTY: gyroid chosen over Voronoi because sheet-gyroid walls are self-supporting for FDM (continuous curvature, short self-buttressed overhangs) — a claim to VERIFY per part: the ops-surface support_report audits B-rep solids and refuses imported meshes, so check this mesh in a slicer support preview; the result is a MESH ONLY (no B-rep reconstruction); wall thickness is volume-calibrated — local thickness varies ~+/-10% (p10-p90) and thins where sheets merge; walls under ~2 voxels only resolve on the upsampled rungs. Returns the runner receipt (volume_mm3 from the kernel's mesh receipt, solid_volume_mm3, volume_fraction, skin/interior voxels, stress_pcts_used, watertight, healed, triangles, file, timings). Wall clock capped by timeout_s (default 300).",
			"inputSchema": {
				"type": "object",
				"properties": {
					"ops": {"type": "array", "description": "LMCAD JSON ops building the geometry (route 1)"},
					"solid": {"type": "string", "description": "op id of the solid to voxelize (route 1)"},
					"shape": {"type": "array", "description": "[nx,ny,nz] voxel grid shape (route 1)"},
					"supersample": {"type": "integer", "description": "per-axis subsamples for solid fractions, default 2 (route 1)"},
					"npy": {"type": "string", "description": "absolute path of an existing (nx,ny,nz) density .npy (route 2)"},
					"voxel_mm": {"type": "number", "description": "cubic voxel edge in mm — must match the stress field's grid"},
					"origin_mm": {"type": "array", "description": "world position of grid node (0,0,0), default [0,0,0]"},
					"stress_npy": {"type": "string", "description": "stress_field.npy of a prior ace_fea on the SAME grid (required)"},
					"cell_mm": {"type": "number", "description": "gyroid cell size in mm, default 8"},
					"wall": {"type": "object", "description": "{min, max} wall thickness range in mm, default {0.8, 2.4}"},
					"stress_map": {"type": "object", "description": "{lo_pct, hi_pct} stress percentiles over solid voxels mapped to wall.min..wall.max, default {20, 95}"},
					"shell_mm": {"type": "number", "description": "solid skin depth preserved by binary erosion, default 1.5"},
					"iso": {"type": "number", "description": "occupancy threshold on the density grid, default 0.5"},
					"file": {"type": "string", "description": "output mesh name (.stl/.3mf) inside the job out dir, default graded_infill.stl"},
					"timeout_s": {"type": "number", "description": "wall-clock cap, default 300 (clamped 1..3600)"}
				},
				"required": ["voxel_mm", "stress_npy"]
			}
		}),
		json!({
			"name": "production_check",
			"description": "Layer-1 FDM production rules on a prior ace_fea result: grades a part's peak von Mises stress against DERATED allowables for a named filament (tools/material_db.json: PLA|PETG|ABS|ASA|TPU95A|PC|PA; aliases TPU, NYLON). Rules — static (yield), creep (the material record's MEASURED time x temperature table, tools/materials/<mat>.json creep.sig_allow_mpa, when load_character.sustained), fatigue (ultimate x fatigue_knockdown ~1e6 cycles, when load_character.cyclic), temp (service_temp_c vs the material's HDT-class limit), anisotropy (when the primary load is > 30 deg out of the layer plane, ALL stress allowables are further multiplied by layer_adhesion_factor and an explicit across-layer row is added). Every derating is shown in each rule's arithmetic; rules that don't apply are listed under skipped WITH the reason (orientation absent => anisotropy UNCHECKED, said so). CREEP IS TIME-DEPENDENT AND SO IS ITS INPUT: a sustained load REQUIRES duration_h. The rule reads the table cell at the stated temperature and duration, rounding BOTH up to the next tabulated cell (never interpolating — the data between rows does not exist), and reports the cell it used as creep_cell {row_used_c, col_used_h, temperature_bucket, duration_bucket, cell_match}. Three conditions produce a FAILING row with a machine-matchable refusal_kind rather than a number: creep_duration_required (no duration_h), creep_temp_above_tabulated (hotter than the table's top tier — there is no fallback row), creep_no_table (the material has no creep data; the legacy time-blind yield x creep_sustained_fraction scalar is reported for visibility and is NEVER used as an allowable). Only PLA carries a table today. VERDICT SEMANTICS: the receipt's ok is the overall verdict — a part FAILING its rules answers isError:true with the full per-rule receipt {rule, allowable_mpa, demand_mpa, SF, pass, detail} attached (it is a gate, not a crash). HONESTY: material data are TYPICAL published desktop-FDM values — verify per filament brand (the receipt carries this disclaimer); the fatigue knockdown is an engineering rule of thumb, not measured filament data; the anisotropy rule is SCALAR-TIER (load-direction heuristic) — a tensor-based layer-normal stress check requires an ACE change; the demand inherits ace_fea's ~20% coarse-mesh under-prediction of peak bending stress, so pair it with an adequate safety_factor_required (default 2).",
			"inputSchema": {
				"type": "object",
				"properties": {
					"material": {"type": "string", "description": "PLA|PETG|ABS|ASA|TPU95A|PC|PA (case-insensitive; TPU->TPU95A, NYLON->PA)"},
					"max_von_mises_pa": {"type": "number", "description": "peak von Mises stress in Pa from a prior ace_fea run"},
					"load_character": {"type": "object", "description": "{sustained: bool, cyclic: bool}, default both false — enables the creep/fatigue rules"},
					"duration_h": {"type": "number", "description": "how long the sustained load is HELD, in hours (e.g. 24, 720, 8760). REQUIRED whenever load_character.sustained is true — the creep allowable is a function of duration as much as of temperature, and without it the rule refuses with refusal_kind creep_duration_required rather than inventing one. Also accepted as service.duration_h / load_character.duration_h."},
					"service_temp_c": {"type": "number", "description": "service temperature in C, default 25. NOTE the creep table is a coarse step (PLA: 23 C and 55 C): 25 C reads the 55 C row, and the receipt's creep_cell says so."},
					"orientation": {"type": "object", "description": "{build_dir:[x,y,z], primary_load_dir:[x,y,z]} — enables the anisotropy rule; absent => anisotropy unchecked (noted)"},
					"safety_factor_required": {"type": "number", "description": "required SF on every stress rule, default 2"},
					"timeout_s": {"type": "number", "description": "wall-clock cap, default 60 (clamped 1..600)"}
				},
				"required": ["material", "max_von_mises_pa"]
			}
		}),
		json!({
			"name": "render_views",
			"description": "SEE the part — VISION IN THE LOOP: renders a 12-view contact sheet as ONE PNG and returns it as an IMAGE content item straight into your context, plus the JSON receipt. Panels: 6 orthos (top/bottom/front/back/left/right), iso az+45 and az-45 (elev 30), a bed view with the part in PRINT orientation per build_dir resting on a drawn bed, and 3 TRUE cross-sections (exact triangle/plane cut lines, NOT silhouettes) at x/y/z (default bbox centers; override via sections). Call this after EVERY iteration that produces or changes geometry and LOOK before declaring the iteration done: (1) shape matches intent from every side, (2) sections show internal features as designed — channels connected, bores where expected, walls present, (3) the bed view shows a printable posture. Numeric gates cannot see everything: the TL-91 speaker line passed every gate with its internal channels sealed — eyes caught it. Input: either stl (a path under the export dir, e.g. a run_program export — overlays are not supported here, render parts one at a time or export a merged solid) or program+solid (run the ops now, export a temp STL, render it, delete the temp). The sheet is capped at 1600 px and auto-re-rendered once at 1200 px if it exceeds 800 KB (declared in the receipt's px). Needs a Python with numpy+matplotlib (env ACE_PYTHON).",
			"inputSchema": {
				"type": "object",
				"properties": {
					"stl": {"type": "string", "description": "STL path relative to the export dir (or absolute within it), e.g. 'tline_v2/tline_v2_body.stl' (route 1)"},
					"program": {"type": "object", "description": "work-order {\"ops\":[...]} to build the geometry now (route 2; requires solid)"},
					"solid": {"type": "string", "description": "op id in 'program' of the solid to export and render (route 2)"},
					"build_dir": {"type": "array", "description": "print/build direction for the bed view, default [0,0,1]"},
					"sections": {"type": "object", "description": "{x?,y?,z?} cut-plane overrides in mm, default bbox centers"},
					"timeout_s": {"type": "number", "description": "wall-clock cap, default 180 (clamped 1..3600)"}
				}
			}
		}),
	]
}

/// A JSON-RPC 2.0 success response.
fn rpc_result(id: Value, result: Value) -> Value {
	json!({"jsonrpc": "2.0", "id": id, "result": result})
}

/// A JSON-RPC 2.0 error response.
fn rpc_error(id: Value, code: i64, message: String) -> Value {
	json!({"jsonrpc": "2.0", "id": id, "error": {"code": code, "message": message}})
}

/// Serialize a kernel report; a serialization failure (never expected) still
/// yields parseable, honest JSON.
fn serialize_report(report: &kernel_api::Report) -> String {
	serde_json::to_string(report).unwrap_or_else(|e| format!("{{\"ok\":false,\"serialize_error\":\"{e}\"}}"))
}

/// Truncate a tool-result text at [`MAX_RESULT_BYTES`], backing off to a
/// UTF-8 character boundary and appending an explicit marker.
fn truncate_result(mut s: String) -> String {
	if s.len() > MAX_RESULT_BYTES {
		let mut cut = MAX_RESULT_BYTES;
		while !s.is_char_boundary(cut) {
			cut -= 1;
		}
		s.truncate(cut);
		s.push_str("…[truncated]");
	}
	s
}

/// Join `rel` onto `base`, rejecting absolute paths and any `..`/prefix
/// component so a tool caller can never escape the repository root.
fn confine(base: &Path, rel: &str) -> Result<PathBuf, String> {
	let p = Path::new(rel);
	if p.is_absolute() {
		return Err(format!("absolute paths are not allowed: '{rel}'"));
	}
	let escapes = p.components().any(|c| matches!(c, Component::ParentDir | Component::Prefix(_)));
	if escapes {
		return Err(format!("path may not contain '..': '{rel}'"));
	}
	Ok(base.join(p))
}

#[cfg(test)]
mod tests {
	use super::*;

	/// A server over the REAL repo root (so `API.md` and the kernel op
	/// surface are the production ones) with a scratch out dir (the test
	/// programs export nothing, so nothing is left behind).
	fn test_server() -> Server {
		let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).parent().unwrap().parent().unwrap().to_path_buf();
		Server::new(root, std::env::temp_dir())
	}

	/// Extract the single text-content string and `isError` from a
	/// `tools/call` response.
	fn call_result(resp: &Value) -> (String, bool) {
		let result = resp.get("result").expect("tools/call response has a result");
		let text = result["content"][0]["text"].as_str().expect("text content").to_string();
		let is_error = result["isError"].as_bool().expect("isError flag");
		(text, is_error)
	}

	/// Gate 2a: a `tools/list` round-trip over the dispatch function returns
	/// exactly the ten tools, each with a name, a description, and an
	/// object-typed input schema whose `required` params exist as properties.
	#[test]
	fn tools_list_serves_ten_valid_tools() {
		let server = test_server();
		let resp = server
			.handle_message(&json!({"jsonrpc": "2.0", "id": 1, "method": "tools/list"}))
			.expect("tools/list is a request, not a notification");
		let tools = resp["result"]["tools"].as_array().expect("tools array").clone();
		let names: Vec<&str> = tools.iter().filter_map(|t| t["name"].as_str()).collect();
		let all_valid = tools.iter().all(|t| {
			let schema = &t["inputSchema"];
			let props = schema["properties"].as_object();
			t["name"].is_string()
				&& t["description"].as_str().is_some_and(|d| !d.is_empty())
				&& schema["type"] == "object"
				&& props.is_some_and(|p| !p.is_empty())
				&& schema["required"].as_array().unwrap_or(&vec![]).iter().all(|r| {
					r.as_str().is_some_and(|r| props.is_some_and(|p| p.contains_key(r)))
				})
		});
		assert!(
			names == ["run_program", "describe_api", "run_assembly", "ace_fea", "ace_optimize", "ace_modal", "ace_buckling", "graded_infill", "production_check", "render_views"] && all_valid,
			"tools/list must serve exactly the ten tools with valid schemas; got names {names:?}, all_valid {all_valid}, response {resp}"
		);
	}

	/// A server whose `ace_fea` runner is a fake shell script (run through
	/// `/bin/sh` standing in for the ACE python): the ace transport contract
	/// — last-stdout-line JSON, the `"ok"`-key refusal rule, isError mapping
	/// — is a property of the Rust side, independent of the real solver
	/// (which the repo's live ACE gates exercise end-to-end).
	fn fake_ace_server(tag: &str, script: &str) -> (Server, PathBuf) {
		let dir = std::env::temp_dir().join(format!("lmcad_mcp_ace_{tag}_{}", std::process::id()));
		let _ = std::fs::remove_dir_all(&dir); // stale job files from a previous run would break counts
		std::fs::create_dir_all(&dir).expect("scratch dir");
		std::fs::write(dir.join("ace_fea_runner.py"), script).expect("fake runner");
		let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).parent().unwrap().parent().unwrap().to_path_buf();
		let server = Server::new(root, dir.clone()).with_ace(PathBuf::from("/bin/sh"), dir.clone());
		(server, dir)
	}

	/// One `tools/call ace_fea` against the given fake-runner script.
	fn call_ace_fea(server: &Server, args: Value) -> (String, bool) {
		let resp = server
			.handle_message(&json!({
				"jsonrpc": "2.0", "id": 9, "method": "tools/call",
				"params": {"name": "ace_fea", "arguments": args}
			}))
			.expect("tools/call is a request");
		call_result(&resp)
	}

	/// ace transport, success + failure + refusal parse paths: (a) a runner
	/// whose LAST non-empty stdout line is an `ok:true` JSON receipt (after
	/// stderr noise and a non-JSON stdout line) returns that line verbatim
	/// with `isError:false`, and the job JSON landed under `<out>/ace/` with
	/// the server-managed `out_dir` injected; (b) an `ok:false` receipt maps
	/// to `isError:true` carrying the runner's error; (c) a JSON payload
	/// WITHOUT an `"ok"` key is refused (ACE orchestrator rule).
	#[test]
	fn ace_fea_transport_parses_receipts_and_refuses_okless_payloads() {
		let (ok_server, ok_dir) = fake_ace_server(
			"ok",
			"echo 'log noise' >&2\necho 'not json'\necho '{\"ok\": true, \"max_von_mises_pa\": 12345.0}'\n",
		);
		let (ok_text, ok_err) = call_ace_fea(&ok_server, json!({"voxel_mm": 1.0}));
		let job_files: Vec<PathBuf> = std::fs::read_dir(ok_dir.join("ace"))
			.map(|rd| rd.filter_map(|e| e.ok().map(|e| e.path())).filter(|p| p.extension().is_some_and(|x| x == "json")).collect())
			.unwrap_or_default();
		let job: Value = job_files
			.first()
			.and_then(|p| std::fs::read_to_string(p).ok())
			.and_then(|s| serde_json::from_str(&s).ok())
			.unwrap_or(Value::Null);
		let (fail_server, _) = fake_ace_server("fail", "echo '{\"ok\": false, \"error\": \"boom: no fixtures\"}'\n");
		let (fail_text, fail_err) = call_ace_fea(&fail_server, json!({}));
		let (okless_server, _) = fake_ace_server("okless", "echo '{\"result\": 1}'\n");
		let (okless_text, okless_err) = call_ace_fea(&okless_server, json!({}));
		assert!(
			!ok_err
				&& ok_text == "{\"ok\": true, \"max_von_mises_pa\": 12345.0}"
				&& job_files.len() == 1
				&& job["voxel_mm"] == json!(1.0)
				&& job["out_dir"].as_str().is_some_and(|d| d.starts_with(ok_dir.join("ace").to_str().unwrap()))
				&& fail_err && fail_text.contains("boom: no fixtures")
				&& okless_err && okless_text.contains("\"ok\""),
			"ace transport contract broken:\nok ({ok_err}): {ok_text}\njob files {job_files:?}, job {job}\nfail ({fail_err}): {fail_text}\nokless ({okless_err}): {okless_text}"
		);
	}

	/// ace transport, timeout path: a runner that sleeps past `timeout_s` is
	/// KILLED (exec makes the sleep the child itself, so the kill is direct)
	/// and answers `isError:true` naming the timeout — no receipt trusted.
	/// The spawn-level guard for a missing interpreter answers the actionable
	/// `pip install -e` hint instead of a raw ENOENT.
	#[test]
	fn ace_fea_timeout_kills_child_and_missing_python_hints() {
		let (slow_server, _) = fake_ace_server("slow", "exec sleep 5\n");
		let t0 = std::time::Instant::now();
		let (slow_text, slow_err) = call_ace_fea(&slow_server, json!({"timeout_s": 1}));
		let elapsed = t0.elapsed();
		let (missing_dir_server, dir) = fake_ace_server("nopython", "echo unused\n");
		let missing_server = missing_dir_server.with_ace(dir.join("no_such_python"), dir.clone());
		let (missing_text, missing_err) = call_ace_fea(&missing_server, json!({}));
		assert!(
			slow_err
				&& slow_text.contains("timed out after 1 s")
				&& elapsed < std::time::Duration::from_secs(4)
				&& missing_err && missing_text.contains("pip install -e"),
			"timeout/missing-python contract broken:\nslow ({slow_err}, {elapsed:?}): {slow_text}\nmissing ({missing_err}): {missing_text}"
		);
	}

	/// A server whose `render_sheet.py` is a fake sh script run through
	/// `/bin/sh` (same stand-in trick as the ace fakes): the render transport
	/// contract — job JSON, receipt parse, PNG→base64 image content, the
	/// over-800-KB auto-downsize — is a property of the Rust side, independent
	/// of matplotlib (which the repo's live render gate exercises end-to-end).
	fn fake_render_server(tag: &str, script: &str) -> (Server, PathBuf) {
		let dir = std::env::temp_dir().join(format!("lmcad_mcp_render_{tag}_{}", std::process::id()));
		let _ = std::fs::remove_dir_all(&dir);
		std::fs::create_dir_all(&dir).expect("scratch dir");
		std::fs::write(dir.join("render_sheet.py"), script).expect("fake renderer");
		let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).parent().unwrap().parent().unwrap().to_path_buf();
		let server = Server::new(root, dir.clone()).with_ace(PathBuf::from("/bin/sh"), dir.clone());
		(server, dir)
	}

	/// One `tools/call render_views`, returning the raw content array + isError.
	fn call_render_views(server: &Server, args: Value) -> (Vec<Value>, bool) {
		let resp = server
			.handle_message(&json!({
				"jsonrpc": "2.0", "id": 11, "method": "tools/call",
				"params": {"name": "render_views", "arguments": args}
			}))
			.expect("tools/call is a request");
		let result = resp.get("result").expect("render_views answers a result").clone();
		let content = result["content"].as_array().expect("content array").clone();
		(content, result["isError"].as_bool().expect("isError flag"))
	}

	/// A fake renderer: reads the sheet path out of the job JSON (`"out"`),
	/// writes a real-signature PNG there (900 KB at max_px 1600, tiny at the
	/// 1200 retry), and prints an ok receipt — POSIX octal escapes only.
	const FAKE_SHEET_SCRIPT: &str = concat!(
		"out=$(sed -n 's/.*\"out\":\"\\([^\"]*\\)\".*/\\1/p' \"$1\")\n",
		"printf '\\211PNG\\015\\012\\032\\012' > \"$out\"\n",
		"if grep -q '\"max_px\":1200' \"$1\"; then head -c 64 /dev/zero >> \"$out\"; px=1200\n",
		"else head -c 900000 /dev/zero >> \"$out\"; px=1600; fi\n",
		"echo \"{\\\"ok\\\": true, \\\"out\\\": \\\"$out\\\", \\\"panels\\\": 12, \\\"px\\\": [$px, 900]}\"\n",
	);

	/// render_views, stl route: the response carries BOTH an image content
	/// item whose base64 decodes to a real PNG signature AND the text receipt;
	/// a first render above [`MAX_SHEET_BYTES`] triggers exactly one automatic
	/// re-render at max_px 1200 (two job files, small final payload).
	#[test]
	fn render_views_returns_image_content_and_auto_downsizes() {
		let (server, dir) = fake_render_server("ok", FAKE_SHEET_SCRIPT);
		std::fs::write(dir.join("part.stl"), b"not read by the fake").expect("fake stl");
		let (content, is_error) = call_render_views(&server, json!({"stl": "part.stl"}));
		let image = &content[0];
		let text = content[1]["text"].as_str().unwrap_or_default();
		let png = base64::engine::general_purpose::STANDARD
			.decode(image["data"].as_str().unwrap_or_default())
			.expect("image data is valid base64");
		let jobs: Vec<String> = std::fs::read_dir(dir.join("render"))
			.map(|rd| rd.filter_map(|e| e.ok().map(|e| e.file_name().to_string_lossy().into_owned())).filter(|n| n.ends_with(".json")).collect())
			.unwrap_or_default();
		assert!(
			!is_error
				&& content.len() == 2
				&& image["type"] == json!("image")
				&& image["mimeType"] == json!("image/png")
				&& png.starts_with(&[0x89, b'P', b'N', b'G', 0x0d, 0x0a, 0x1a, 0x0a])
				&& (png.len() as u64) < MAX_SHEET_BYTES
				&& text.contains("\"ok\": true")
				&& text.contains("[1200, 900]")
				&& jobs.len() == 2,
			"render_views image contract broken: isError {is_error}, content len {}, image type {}, png head {:?} len {}, text {text}, jobs {jobs:?}",
			content.len(),
			image["type"],
			&png[..png.len().min(8)],
			png.len()
		);
	}

	/// render_views, program route: the ops run through the real kernel, the
	/// appended `export_stl` writes a TEMP stl under the out dir, the sheet is
	/// rendered from it, and the temp stl is deleted afterwards (the sheet
	/// stays). Also the argument guards: no input, a `..` path, an absolute
	/// path outside the out dir, and a missing file are all refused with
	/// `isError: true` and an honest message.
	#[test]
	fn render_views_program_route_and_path_confinement() {
		let (server, dir) = fake_render_server("prog", FAKE_SHEET_SCRIPT);
		let program = json!({"ops": [{"id": "b", "op": "box", "min": [0, 0, 0], "max": [5, 5, 5]}]});
		let (content, is_error) = call_render_views(&server, json!({"program": program, "solid": "b"}));
		let leftover_stls: Vec<String> = std::fs::read_dir(&dir)
			.map(|rd| rd.filter_map(|e| e.ok().map(|e| e.file_name().to_string_lossy().into_owned())).filter(|n| n.ends_with(".stl")).collect())
			.unwrap_or_default();
		let sheets: Vec<String> = std::fs::read_dir(&dir)
			.map(|rd| rd.filter_map(|e| e.ok().map(|e| e.file_name().to_string_lossy().into_owned())).filter(|n| n.ends_with("_sheet.png")).collect())
			.unwrap_or_default();
		let guard = |args: Value| {
			let (content, is_error) = call_render_views(&server, args);
			(content[0]["text"].as_str().unwrap_or_default().to_string(), is_error)
		};
		let (none_text, none_err) = guard(json!({}));
		let (dotdot_text, dotdot_err) = guard(json!({"stl": "../escape.stl"}));
		let (abs_text, abs_err) = guard(json!({"stl": "/etc/passwd"}));
		let (missing_text, missing_err) = guard(json!({"stl": "no_such.stl"}));
		assert!(
			!is_error
				&& content[0]["type"] == json!("image")
				&& leftover_stls.is_empty()
				&& sheets.len() == 1
				&& none_err && none_text.contains("requires either")
				&& dotdot_err && dotdot_text.contains("..")
				&& abs_err && abs_text.contains("must live under")
				&& missing_err && missing_text.contains("not found"),
			"program route / confinement broken: isError {is_error}, leftover stls {leftover_stls:?}, sheets {sheets:?},\nnone ({none_err}): {none_text}\ndotdot ({dotdot_err}): {dotdot_text}\nabs ({abs_err}): {abs_text}\nmissing ({missing_err}): {missing_text}"
		);
	}

	/// Gate 2b: `tools/call run_program` with a box + validate program
	/// succeeds and the result text is the kernel report carrying the
	/// geometric-validity flag.
	#[test]
	fn run_program_box_validate_reports_geometric_ok() {
		let server = test_server();
		let resp = server
			.handle_message(&json!({
				"jsonrpc": "2.0", "id": 2, "method": "tools/call",
				"params": {"name": "run_program", "arguments": {"program": {"ops": [
					{"id": "b", "op": "box", "min": [0, 0, 0], "max": [5, 5, 5]},
					{"id": "v", "op": "validate", "in": "b"},
				]}}}
			}))
			.expect("tools/call is a request");
		let (text, is_error) = call_result(&resp);
		assert!(
			!is_error && text.contains("\"geometric_ok\":true") && text.contains("\"api_version\":\"cadcode.v1\""),
			"box+validate must succeed with geometric_ok:true in the kernel report; isError {is_error}, text {text}"
		);
	}

	/// Gate 2c: `describe_api {op: \"box\"}` returns the real API.md doc
	/// section; `describe_api {}` returns the full catalogue with count ≥ 120.
	#[test]
	fn describe_api_serves_docs_and_catalogue() {
		let server = test_server();
		let call = |args: Value| {
			let resp = server
				.handle_message(&json!({
					"jsonrpc": "2.0", "id": 3, "method": "tools/call",
					"params": {"name": "describe_api", "arguments": args}
				}))
				.expect("tools/call is a request");
			call_result(&resp)
		};
		let (box_text, box_err) = call(json!({"op": "box"}));
		let box_doc: Value = serde_json::from_str(&box_text).expect("describe_api returns JSON");
		let (cat_text, cat_err) = call(json!({}));
		let cat: Value = serde_json::from_str(&cat_text).expect("catalogue is JSON");
		let count = cat["count"].as_u64().unwrap_or(0);
		let ops_len = cat["ops"].as_array().map_or(0, Vec::len);
		assert!(
			!box_err
				&& box_doc["exists"] == json!(true)
				&& box_doc["doc"].as_str().is_some_and(|d| d.starts_with("### `box`") && d.contains("low corner"))
				&& !cat_err
				&& count >= 120
				&& count as usize == ops_len,
			"describe_api must serve the box doc section and a catalogue of ≥ 120 ops; box: {box_text}, count {count}, ops_len {ops_len}"
		);
	}

	/// Gate 2d: a malformed stdin line answers `-32700` (null id), an unknown
	/// method answers `-32601`, and id-less notifications (known or unknown)
	/// answer nothing.
	#[test]
	fn protocol_errors_are_honest_jsonrpc() {
		let server = test_server();
		let malformed: Value = serde_json::from_str(&server.handle_line("{not json").expect("parse error is answered")).unwrap();
		let unknown = server
			.handle_message(&json!({"jsonrpc": "2.0", "id": 7, "method": "resources/list"}))
			.expect("unknown method with id is answered");
		let initialized = server.handle_message(&json!({"jsonrpc": "2.0", "method": "notifications/initialized"}));
		let unknown_notification = server.handle_message(&json!({"jsonrpc": "2.0", "method": "notifications/cancelled"}));
		assert!(
			malformed["error"]["code"] == json!(-32700)
				&& malformed["id"] == Value::Null
				&& unknown["error"]["code"] == json!(-32601)
				&& unknown["id"] == json!(7)
				&& initialized.is_none()
				&& unknown_notification.is_none(),
			"protocol errors broken: malformed {malformed}, unknown {unknown}, initialized answered: {}, unknown notification answered: {}",
			initialized.is_some(),
			unknown_notification.is_some()
		);
	}

	/// The remaining protocol surface: `initialize` echoes the client's
	/// protocol version (falling back to the default), `ping` answers `{}`,
	/// `tools/call` on an unknown tool is `-32602`, and `run_assembly`
	/// refuses absolute and `..` paths with `isError: true`.
	#[test]
	fn initialize_ping_unknown_tool_and_path_confinement() {
		let server = test_server();
		let init = server
			.handle_message(&json!({
				"jsonrpc": "2.0", "id": 1, "method": "initialize",
				"params": {"protocolVersion": "2025-03-26", "capabilities": {}, "clientInfo": {"name": "t", "version": "0"}}
			}))
			.unwrap();
		let init_default = server.handle_message(&json!({"jsonrpc": "2.0", "id": 2, "method": "initialize"})).unwrap();
		let ping = server.handle_message(&json!({"jsonrpc": "2.0", "id": 3, "method": "ping"})).unwrap();
		let bad_tool = server
			.handle_message(&json!({"jsonrpc": "2.0", "id": 4, "method": "tools/call", "params": {"name": "nope", "arguments": {}}}))
			.unwrap();
		let call_asm = |path: &str| {
			let resp = server
				.handle_message(&json!({
					"jsonrpc": "2.0", "id": 5, "method": "tools/call",
					"params": {"name": "run_assembly", "arguments": {"asm_path": path}}
				}))
				.unwrap();
			call_result(&resp)
		};
		let (abs_text, abs_err) = call_asm("/etc/passwd");
		let (dotdot_text, dotdot_err) = call_asm("../outside.lmcasm");
		assert!(
			init["result"]["protocolVersion"] == json!("2025-03-26")
				&& init["result"]["serverInfo"]["name"] == json!("lmcad")
				&& init["result"]["capabilities"]["tools"].is_object()
				&& init_default["result"]["protocolVersion"] == json!(DEFAULT_PROTOCOL_VERSION)
				&& ping["result"] == json!({})
				&& bad_tool["error"]["code"] == json!(-32602)
				&& abs_err && abs_text.contains("absolute")
				&& dotdot_err && dotdot_text.contains(".."),
			"protocol surface broken: init {init}, init_default {init_default}, ping {ping}, bad_tool {bad_tool}, abs ({abs_err}, {abs_text}), dotdot ({dotdot_err}, {dotdot_text})"
		);
	}

	/// Truncation backs off to a UTF-8 boundary and appends the marker; short
	/// strings pass through untouched.
	#[test]
	fn truncation_is_utf8_safe_and_marked() {
		let short = truncate_result("ok".to_string());
		// Multi-byte chars ('é' = 2 bytes) straddling the cut must not split.
		let long = truncate_result("é".repeat(MAX_RESULT_BYTES));
		assert!(
			short == "ok" && long.ends_with("…[truncated]") && long.len() <= MAX_RESULT_BYTES + "…[truncated]".len() && String::from_utf8(long.clone().into_bytes()).is_ok(),
			"truncation contract broken: short {short:?}, long len {}, tail {:?}",
			long.len(),
			&long[long.len().saturating_sub(20)..]
		);
	}
}
