// Copyright (c) LMCAD. Licensed under the MIT License.

//! `lmcad-tui` — the LMCAD TUI, a terminal front-end for the LMCAD Studio
//! server. Three modes:
//!
//! - `lmcad-tui` — the full TUI (conversation log + receipts pane + prompt).
//! - `lmcad-tui -p "<prompt>"` — headless one-shot chat, streamed to stdout.
//! - `lmcad-tui --run <file.json | ->` — headless work-order run, report printed,
//!   exit 0 iff the kernel report says ok.
//!
//! Server discovery: `CADCODE_SERVER` (default `http://127.0.0.1:7878`); when
//! the health probe is refused and `./target/release/studio-server` exists it
//! is auto-started (disable with `--no-spawn`). `CADCODE_API_TOKEN` adds a
//! bearer header to every API request.

use std::io::IsTerminal;
use std::process::ExitCode;

use studio_tui::{client, headless, tui};

const HELP: &str = "\
lmcad-tui — LMCAD TUI, terminal front-end for the LMCAD Studio server

USAGE:
  lmcad-tui                      full TUI (needs a tty)
  lmcad-tui -p \"<prompt>\"        one-shot chat: stream the reply to stdout, exit 0 on done
  lmcad-tui --run <file | ->     POST a work order to /api/run, print the report, exit 0 iff ok

OPTIONS:
  -p, --prompt <text>    headless chat prompt
      --run <file | ->   headless work-order run (- reads the program from stdin)
      --session <name>   session for runs and chat exports (default: default)
      --no-spawn         never auto-start ./target/release/studio-server
  -h, --help             this help

ENVIRONMENT:
  CADCODE_SERVER       server base URL (default http://127.0.0.1:7878)
  CADCODE_API_TOKEN    bearer token, when the server requires auth

TUI COMMANDS (typed at the › prompt; plain text is sent to chat):
  :run <file.json> · :ops · :catalog · :session <name> · :clear · :quit
  PgUp/PgDn scroll the log · Esc Esc or Ctrl+C quit";

enum Mode {
	Tui,
	Prompt(String),
	Run(String),
}

struct Args {
	mode: Mode,
	session: String,
	no_spawn: bool,
}

fn parse_args() -> Result<Option<Args>, String> {
	let mut mode = Mode::Tui;
	let mut session = "default".to_string();
	let mut no_spawn = false;
	let mut args = std::env::args().skip(1);
	while let Some(arg) = args.next() {
		match arg.as_str() {
			"-p" | "--prompt" => {
				let p = args.next().ok_or("-p needs a prompt argument")?;
				mode = Mode::Prompt(p);
			}
			"--run" => {
				let f = args.next().ok_or("--run needs a file argument (or - for stdin)")?;
				mode = Mode::Run(f);
			}
			"--session" => {
				session = args.next().ok_or("--session needs a name")?;
				let ok = !session.is_empty()
					&& session.len() <= 64
					&& session.chars().all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-');
				if !ok {
					return Err(format!("invalid session '{session}': use [A-Za-z0-9_-], at most 64 chars"));
				}
			}
			"--no-spawn" => no_spawn = true,
			"-h" | "--help" => {
				println!("{HELP}");
				return Ok(None);
			}
			other => return Err(format!("unknown argument '{other}' (see lmcad-tui --help)")),
		}
	}
	Ok(Some(Args { mode, session, no_spawn }))
}

fn main() -> ExitCode {
	let args = match parse_args() {
		Ok(Some(a)) => a,
		Ok(None) => return ExitCode::SUCCESS,
		Err(e) => {
			eprintln!("lmcad-tui: {e}");
			return ExitCode::from(2);
		}
	};

	// The TUI needs a real terminal on both ends; a piped invocation gets a
	// pointer to the headless modes instead of a crashed alternate screen.
	if matches!(args.mode, Mode::Tui) && (!std::io::stdin().is_terminal() || !std::io::stdout().is_terminal()) {
		eprintln!(
			"lmcad-tui: the TUI needs a tty (stdin/stdout is piped or redirected).\n\
			 use `lmcad-tui -p \"<prompt>\"` for one-shot chat, or `lmcad-tui --run <file.json | ->` to execute a work order."
		);
		return ExitCode::from(2);
	}

	let client = client::Client::from_env();
	let spawn_note = match client.ensure_server(!args.no_spawn) {
		Ok(note) => note,
		Err(e) => {
			eprintln!("lmcad-tui: {e}");
			return ExitCode::from(1);
		}
	};

	match args.mode {
		Mode::Prompt(prompt) => {
			if let Some(note) = spawn_note {
				eprintln!("lmcad-tui: {note}");
			}
			exit_code(headless::prompt_mode(&client, &prompt, &args.session))
		}
		Mode::Run(source) => {
			if let Some(note) = spawn_note {
				eprintln!("lmcad-tui: {note}");
			}
			exit_code(headless::run_mode(&client, &source))
		}
		Mode::Tui => {
			let mut notes = vec![format!("lmcad-tui — LMCAD Studio at {}", client.base)];
			if let Some(note) = spawn_note {
				notes.push(note);
			}
			match tui::run(client, args.session, notes) {
				Ok(()) => ExitCode::SUCCESS,
				Err(e) => {
					eprintln!("lmcad-tui: terminal error: {e}");
					ExitCode::from(1)
				}
			}
		}
	}
}

fn exit_code(code: i32) -> ExitCode {
	ExitCode::from(u8::try_from(code).unwrap_or(1))
}
