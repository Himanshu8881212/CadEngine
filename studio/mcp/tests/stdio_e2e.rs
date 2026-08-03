// Copyright (c) LMCAD. Licensed under the MIT License.

//! End-to-end proof over REAL stdio: spawn the `lmcad-mcp` binary as a child
//! process (exactly how Claude Code launches an MCP server), speak the wire
//! protocol — `initialize` → `notifications/initialized` → `tools/list` →
//! `tools/call run_program` — as four newline-delimited JSON-RPC messages,
//! and assert the three responses parse and carry the kernel's report.

use std::io::{BufRead, BufReader, Write};
use std::process::{Command, Stdio};

use serde_json::{json, Value};

/// The full MCP session a client performs, over the released binary's real
/// stdin/stdout. Three responses are expected (the notification gets none);
/// each must be single-line JSON echoing its request id, and the tool call
/// must return the kernel report with `geometric_ok:true` for the 5 mm box.
#[test]
fn mcp_session_over_real_stdio() {
	let repo_root = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).parent().unwrap().parent().unwrap().to_path_buf();
	let mut child = Command::new(env!("CARGO_BIN_EXE_lmcad-mcp"))
		.current_dir(&repo_root)
		.env("LMCAD_ROOT", &repo_root)
		.stdin(Stdio::piped())
		.stdout(Stdio::piped())
		.stderr(Stdio::null())
		.spawn()
		.expect("lmcad-mcp binary spawns");

	let mut stdin = child.stdin.take().expect("child stdin");
	let stdout = BufReader::new(child.stdout.take().expect("child stdout"));

	let messages = [
		json!({"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {
			"protocolVersion": "2025-06-18",
			"capabilities": {},
			"clientInfo": {"name": "e2e-test", "version": "0.0.0"},
		}}),
		json!({"jsonrpc": "2.0", "method": "notifications/initialized"}),
		json!({"jsonrpc": "2.0", "id": 2, "method": "tools/list"}),
		json!({"jsonrpc": "2.0", "id": 3, "method": "tools/call", "params": {
			"name": "run_program",
			"arguments": {"program": {"ops": [
				{"id": "b", "op": "box", "min": [0, 0, 0], "max": [5, 5, 5]},
				{"id": "v", "op": "validate", "in": "b"},
			]}},
		}}),
	];
	for msg in &messages {
		writeln!(stdin, "{msg}").expect("write request line");
	}
	stdin.flush().expect("flush requests");
	drop(stdin); // EOF — the server answers everything pending, then exits 0

	let responses: Vec<Value> = stdout
		.lines()
		.map(|l| {
			let line = l.expect("read response line");
			serde_json::from_str(&line).unwrap_or_else(|e| panic!("stdout line is not JSON ({e}): {line}"))
		})
		.collect();
	let status = child.wait().expect("child exits");

	assert!(
		responses.len() == 3 && status.success(),
		"expected exactly 3 responses (the notification is unanswered) and exit 0; got {} responses, status {status}: {responses:?}",
		responses.len()
	);
	let init = &responses[0];
	let list = &responses[1];
	let call = &responses[2];
	let tool_names: Vec<&str> =
		list["result"]["tools"].as_array().map(Vec::as_slice).unwrap_or_default().iter().filter_map(|t| t["name"].as_str()).collect();
	let call_text = call["result"]["content"][0]["text"].as_str().unwrap_or_default();
	let report: Value = serde_json::from_str(call_text).unwrap_or(Value::Null);
	assert!(
		init["id"] == json!(1)
			&& init["result"]["protocolVersion"] == json!("2025-06-18")
			&& init["result"]["serverInfo"]["name"] == json!("lmcad")
			&& list["id"] == json!(2)
			&& tool_names == ["run_program", "describe_api", "run_assembly", "ace_fea", "ace_optimize", "ace_modal", "ace_buckling", "graded_infill", "production_check", "render_views"]
			&& call["id"] == json!(3)
			&& call["result"]["isError"] == json!(false)
			&& report["ok"] == json!(true)
			&& report["api_version"] == json!("cadcode.v1")
			&& call_text.contains("\"geometric_ok\":true"),
		"MCP stdio session contract broken.\ninitialize: {init}\ntools/list names: {tool_names:?}\ncall: {call}"
	);
}
