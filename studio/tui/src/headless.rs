// Copyright (c) LMCAD. Licensed under the MIT License.

//! The two headless one-shot modes — plain stdout, script/CI-friendly:
//!
//! - `lmcad-tui -p "<prompt>"` — one chat turn, the SSE event stream printed as
//!   plain text (text deltas verbatim, thinking prefixed `[thinking]`,
//!   tool/tasks/subagent/refresh/error/done as bracketed status lines). Exit 0
//!   once the server's `done` event arrived; nonzero only on transport failure.
//! - `lmcad-tui --run <file.json | ->` — POST a work-order program to
//!   `/api/run`, pretty-print the kernel report (per-op ✓/✗, measures
//!   one-lined, artifacts). Exit 0 iff `report.ok`.

use std::io::{Read, Write};

use serde_json::Value;

use crate::client::{op_line, op_names_of, receipt_summary, task_mark, ChatEvent, ChatTurn, Client};

/// What the printer was last emitting, so mode switches insert the right
/// separators without double newlines (deltas stream mid-line).
#[derive(PartialEq)]
enum Emitting {
	Nothing,
	Text,
	Thinking,
}

/// `-p` mode: send one user prompt, stream the reply to stdout. Returns the
/// process exit code.
pub fn prompt_mode(client: &Client, prompt: &str, session: &str) -> i32 {
	let messages = vec![ChatTurn { role: "user".to_string(), content: prompt.to_string() }];
	let mut out = std::io::stdout();
	let mut mode = Emitting::Nothing;
	let result = client.chat(&messages, session, |event| match event {
		ChatEvent::Text(delta) => {
			if mode == Emitting::Thinking {
				let _ = writeln!(out);
			}
			let _ = write!(out, "{delta}");
			let _ = out.flush();
			mode = Emitting::Text;
		}
		ChatEvent::Thinking(delta) => {
			if mode != Emitting::Thinking {
				if mode == Emitting::Text {
					let _ = writeln!(out);
				}
				let _ = write!(out, "[thinking] ");
			}
			let _ = write!(out, "{delta}");
			let _ = out.flush();
			mode = Emitting::Thinking;
		}
		ChatEvent::Tool { state, name, ops, ok, error } => {
			let line = match state.as_str() {
				"running" => format!("[tool {name} ({ops} ops) running]"),
				_ => match (ok, error) {
					(Some(true), _) => format!("[tool {name} ({ops} ops) ok]"),
					(_, Some(e)) => format!("[tool {name} error: {e}]"),
					_ => format!("[tool {name} ({ops} ops) {state}]"),
				},
			};
			status(&mut out, &mut mode, &line);
		}
		ChatEvent::Tasks(tasks) => {
			// The whole plan, re-printed as a checklist block on every update.
			for task in &tasks {
				status(&mut out, &mut mode, &format!("[plan] {} {}", task_mark(&task.status), task.content));
			}
		}
		ChatEvent::Subagent { name, state, detail } => {
			let line = match detail {
				Some(d) => format!("[subagent {name}] {state}: {d}"),
				None => format!("[subagent {name}] {state}"),
			};
			status(&mut out, &mut mode, &line);
		}
		ChatEvent::Refresh { artifacts, receipt } => {
			let mut line = format!("[refresh artifacts: {}", artifacts.join(", "));
			if let Some(summary) = receipt.as_ref().map(receipt_summary).filter(|s| !s.is_empty()) {
				line.push_str(&format!(" · {summary}"));
			}
			line.push(']');
			status(&mut out, &mut mode, &line);
		}
		ChatEvent::Disabled(message) => status(&mut out, &mut mode, &format!("[chat_disabled] {message}")),
		ChatEvent::Error(message) => status(&mut out, &mut mode, &format!("[error] {message}")),
		ChatEvent::Done(stop_reason) => status(&mut out, &mut mode, &format!("[done {stop_reason}]")),
	});
	match result {
		Ok(()) => 0,
		Err(e) => {
			eprintln!("lmcad-tui: {e}");
			1
		}
	}
}

/// `--run` mode: read a program from `path` (`-` = stdin), POST it, print the
/// report. Returns the process exit code (0 iff `report.ok`).
pub fn run_mode(client: &Client, path: &str) -> i32 {
	let text = match read_source(path) {
		Ok(t) => t,
		Err(e) => {
			eprintln!("lmcad-tui: {e}");
			return 1;
		}
	};
	let program: Value = match serde_json::from_str(&text) {
		Ok(v) => v,
		Err(e) => {
			eprintln!("lmcad-tui: '{path}' is not valid JSON: {e}");
			return 1;
		}
	};
	let names = op_names_of(&program);
	let resp = match client.run_program(&program) {
		Ok(r) => r,
		Err(e) => {
			eprintln!("lmcad-tui: {e}");
			return 1;
		}
	};
	let verdict = if resp.report.ok { "ok" } else { "FAIL" };
	let version = resp.report.api_version.as_deref().unwrap_or("?");
	println!("report {version} · {verdict} · session {}", resp.session);
	for op in &resp.report.ops {
		println!("  {}", op_line(op, names.get(&op.id).map(String::as_str)));
	}
	if !resp.artifacts.is_empty() {
		let files: Vec<&str> = resp.artifacts.iter().map(|a| a.file.as_str()).collect();
		println!("artifacts: {}", files.join(" "));
	}
	if resp.report.ok {
		0
	} else {
		1
	}
}

/// Emit one bracketed status line, closing any open delta line first.
fn status(out: &mut std::io::Stdout, mode: &mut Emitting, line: &str) {
	if *mode != Emitting::Nothing {
		let _ = writeln!(out);
	}
	let _ = writeln!(out, "{line}");
	*mode = Emitting::Nothing;
}

fn read_source(path: &str) -> Result<String, String> {
	if path == "-" {
		let mut buf = String::new();
		std::io::stdin().read_to_string(&mut buf).map_err(|e| format!("reading stdin: {e}"))?;
		Ok(buf)
	} else {
		std::fs::read_to_string(path).map_err(|e| format!("cannot read '{path}': {e}"))
	}
}
