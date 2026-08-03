// Copyright (c) LMCAD. Licensed under the MIT License.

//! `lmcad-mcp` — the MCP stdio server binary. Reads one JSON-RPC message per
//! stdin line, answers on stdout (single-line JSON, flushed per message),
//! logs to stderr only. Exits 0 on stdin EOF (the client hung up).
//!
//! stdout discipline is absolute: nothing but JSON-RPC responses is ever
//! written there — a stray print would corrupt the MCP stream. A panic that
//! escapes the kernel's own catch_unwind guards is converted to a JSON-RPC
//! internal error instead of killing the server.

use std::io::{BufRead, Write};
use std::panic::AssertUnwindSafe;

fn main() {
	let server = match studio_mcp::Server::from_env() {
		Ok(s) => s,
		Err(e) => {
			eprintln!("lmcad-mcp: startup failed: {e}");
			std::process::exit(1);
		}
	};
	// Panic messages default to stderr, which is safe here — but a panic hook
	// makes the origin explicit in the client's server log.
	std::panic::set_hook(Box::new(|info| eprintln!("lmcad-mcp: caught panic: {info}")));

	let stdin = std::io::stdin();
	let mut stdout = std::io::stdout().lock();
	for line in stdin.lock().lines() {
		let line = match line {
			Ok(l) => l,
			Err(e) => {
				eprintln!("lmcad-mcp: stdin read error: {e}");
				break;
			}
		};
		if line.trim().is_empty() {
			continue;
		}
		// The kernel entry points carry their own catch_unwind boundaries;
		// this outer guard covers everything else so the server NEVER dies
		// mid-session on a request.
		let response = std::panic::catch_unwind(AssertUnwindSafe(|| server.handle_line(&line))).unwrap_or_else(|_| {
			Some(
				serde_json::json!({
					"jsonrpc": "2.0",
					"id": null,
					"error": {"code": -32603, "message": "internal error: request handler panicked (see server stderr log)"},
				})
				.to_string(),
			)
		});
		if let Some(response) = response {
			if writeln!(stdout, "{response}").and_then(|()| stdout.flush()).is_err() {
				eprintln!("lmcad-mcp: stdout closed — exiting");
				break;
			}
		}
	}
}
