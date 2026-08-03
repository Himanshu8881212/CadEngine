// Copyright (c) LMCAD. Licensed under the MIT License.

//! The full-screen TUI: a conversation log (streamed chat with plan/subagent
//! progress), a right-hand pane (the agent's PLAN checklist over the RECEIPTS
//! of the last run, both rendered verbatim), and a one-line command prompt.
//!
//! This layer is deliberately thin: every network interaction goes through
//! [`crate::client`] on a worker `std::thread`, results come back over one
//! `std::sync::mpsc` channel, and the loop just folds messages into state and
//! redraws. Geometry claims on screen are always the server's own receipts.

use std::collections::HashMap;
use std::sync::mpsc::{channel, Receiver, Sender};
use std::time::Duration;

use crossterm::event::{self, Event, KeyCode, KeyEvent, KeyEventKind, KeyModifiers};
use ratatui::layout::{Constraint, Layout, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span, Text};
use ratatui::widgets::{Block, Paragraph};
use ratatui::{DefaultTerminal, Frame};
use serde_json::{json, Value};

use crate::client::{op_line, op_names_of, receipt_summary, task_mark, ChatEvent, ChatTurn, Client, RunResponse, TaskItem};

/// Width of the right-hand PLAN/RECEIPTS pane.
const RECEIPTS_WIDTH: u16 = 38;
/// Command palette, shown in help and unknown-command errors.
const COMMANDS: &str = ":run <file.json> · :ops · :catalog · :session <name> · :clear · :quit";

/// Run the TUI until quit. `notes` are startup lines (e.g. the auto-spawn
/// notice) shown at the top of the log.
pub fn run(client: Client, session: String, notes: Vec<String>) -> std::io::Result<()> {
	let mut terminal = ratatui::init();
	let result = App::new(client, session, notes).main_loop(&mut terminal);
	ratatui::restore();
	result
}

/// Whether the server's chat loop is available — unknown until the first chat
/// attempt answers (`chat_disabled` = off; any streamed content = on).
#[derive(Clone, Copy, PartialEq)]
enum ChatStatus {
	Unknown,
	On,
	Off,
}

/// Visual class of one conversation-log entry.
#[derive(Clone, Copy, PartialEq)]
enum LogKind {
	/// The user's prompt, echoed.
	User,
	/// Streamed assistant text.
	Assistant,
	/// Streamed thinking summary (dim + italic, `[thinking]`-prefixed).
	Thinking,
	/// A neutral status line (tool running, artifacts).
	Status,
	/// A completed-ok status line.
	StatusOk,
	/// A failed status line.
	StatusErr,
	/// A parallel part-agent progress line (`◆ …`) — magenta so it never
	/// reads as one of the parent's own ▶/✓ tool lines.
	Subagent,
	/// App info (greeting, command output).
	Info,
	/// An error (transport, bad command, chat_disabled notice).
	Error,
}

impl LogKind {
	fn style(self) -> Style {
		match self {
			LogKind::User => Style::new().fg(Color::Cyan).add_modifier(Modifier::BOLD),
			LogKind::Assistant => Style::new(),
			LogKind::Thinking => Style::new().fg(Color::DarkGray).add_modifier(Modifier::ITALIC),
			LogKind::Status => Style::new().fg(Color::Yellow),
			LogKind::StatusOk => Style::new().fg(Color::Green),
			LogKind::StatusErr => Style::new().fg(Color::Red),
			LogKind::Subagent => Style::new().fg(Color::Magenta),
			LogKind::Info => Style::new().fg(Color::DarkGray),
			LogKind::Error => Style::new().fg(Color::Red),
		}
	}
}

/// One log entry — may span many display lines after wrapping.
struct LogEntry {
	kind: LogKind,
	text: String,
}

/// Tone of one receipts-pane line.
#[derive(Clone, Copy)]
enum Tone {
	Head,
	Ok,
	Err,
	Plain,
}

/// One line of the receipts pane.
struct ReceiptLine {
	tone: Tone,
	text: String,
}

/// A worker-thread result folded into the UI.
enum Msg {
	/// One streamed chat event.
	Chat(ChatEvent),
	/// The chat worker finished (`Err` = transport failure).
	ChatEnded(Result<(), String>),
	/// A `:run` finished.
	RunDone { label: String, names: HashMap<String, String>, result: Box<Result<RunResponse, String>> },
	/// A `:ops` describe finished.
	Ops(Result<Vec<String>, String>),
	/// A `:catalog` fetch finished.
	Catalog(Result<Value, String>),
}

struct App {
	client: Client,
	session: String,
	chat_status: ChatStatus,
	/// Full text history, resent to the server every turn.
	history: Vec<ChatTurn>,
	log: Vec<LogEntry>,
	receipts: Vec<ReceiptLine>,
	/// The orchestrator's latest plan (`tasks` frames replace it whole);
	/// empty = no PLAN section, receipts get the whole right pane.
	plan: Vec<TaskItem>,
	input: String,
	/// What is in flight (`"chat"` / `"run"` / …); new network work is refused
	/// while set so the log and receipts stay coherent.
	busy: Option<&'static str>,
	/// Auto-follow the log tail unless the user scrolled up.
	follow: bool,
	scroll: usize,
	/// Set by draw: log viewport height and max scroll, for paging keys.
	log_page: usize,
	log_max_scroll: usize,
	esc_armed: bool,
	hint: Option<String>,
	/// Assistant text accumulated this turn (for the history).
	assistant_acc: String,
	/// This turn answered `chat_disabled` — drop the unanswered user turn.
	disabled_this_turn: bool,
	quit: bool,
	tx: Sender<Msg>,
	rx: Receiver<Msg>,
}

impl App {
	fn new(client: Client, session: String, notes: Vec<String>) -> Self {
		let (tx, rx) = channel();
		let mut app = App {
			client,
			session,
			chat_status: ChatStatus::Unknown,
			history: Vec::new(),
			log: Vec::new(),
			receipts: vec![ReceiptLine { tone: Tone::Plain, text: "no run yet — :run <file.json> or ask in chat".into() }],
			plan: Vec::new(),
			input: String::new(),
			busy: None,
			follow: true,
			scroll: 0,
			log_page: 1,
			log_max_scroll: 0,
			esc_armed: false,
			hint: None,
			assistant_acc: String::new(),
			disabled_this_turn: false,
			quit: false,
			tx,
			rx,
		};
		for note in notes {
			app.push(LogKind::Info, note);
		}
		app.push(LogKind::Info, format!("commands: {COMMANDS} — Esc Esc or Ctrl+C to quit"));
		app
	}

	fn main_loop(&mut self, terminal: &mut DefaultTerminal) -> std::io::Result<()> {
		while !self.quit {
			while let Ok(msg) = self.rx.try_recv() {
				self.on_msg(msg);
			}
			terminal.draw(|f| self.draw(f))?;
			if event::poll(Duration::from_millis(50))? {
				match event::read()? {
					Event::Key(key) if key.kind != KeyEventKind::Release => self.on_key(key),
					_ => {}
				}
			}
		}
		Ok(())
	}

	// -- state folding -------------------------------------------------------

	fn push(&mut self, kind: LogKind, text: impl Into<String>) {
		self.log.push(LogEntry { kind, text: text.into() });
	}

	/// Append a streamed delta to the tail entry of `kind`, or open a new one.
	fn stream(&mut self, kind: LogKind, prefix: &str, delta: &str) {
		if let Some(last) = self.log.last_mut() {
			if last.kind == kind {
				last.text.push_str(delta);
				return;
			}
		}
		self.log.push(LogEntry { kind, text: format!("{prefix}{delta}") });
	}

	fn on_msg(&mut self, msg: Msg) {
		match msg {
			Msg::Chat(event) => self.on_chat_event(event),
			Msg::ChatEnded(result) => {
				self.busy = None;
				if self.disabled_this_turn {
					// The turn was never answered — drop it so the history the
					// server sees stays an answered conversation.
					self.history.pop();
				} else if !self.assistant_acc.is_empty() {
					self.history.push(ChatTurn { role: "assistant".into(), content: std::mem::take(&mut self.assistant_acc) });
				}
				if let Err(e) = result {
					self.push(LogKind::Error, format!("[transport] {e}"));
				}
			}
			Msg::RunDone { label, names, result } => {
				self.busy = None;
				match *result {
					Ok(resp) => self.on_run_report(&label, &names, &resp),
					Err(e) => {
						self.push(LogKind::StatusErr, format!("✗ run {label}: {e}"));
						self.receipts = vec![
							ReceiptLine { tone: Tone::Head, text: format!("run {label}") },
							ReceiptLine { tone: Tone::Err, text: e },
						];
					}
				}
			}
			Msg::Ops(result) => {
				self.busy = None;
				match result {
					Ok(names) => {
						self.push(LogKind::Info, format!("kernel ops ({}):", names.len()));
						for chunk in names.chunks(6) {
							self.push(LogKind::Info, format!("  {}", chunk.join(" · ")));
						}
					}
					Err(e) => self.push(LogKind::Error, format!(":ops failed — {e}")),
				}
			}
			Msg::Catalog(result) => {
				self.busy = None;
				match result {
					Ok(v) => self.on_catalog(&v),
					Err(e) => self.push(LogKind::Error, format!(":catalog failed — {e}")),
				}
			}
		}
	}

	fn on_chat_event(&mut self, event: ChatEvent) {
		match event {
			ChatEvent::Text(delta) => {
				self.chat_status = ChatStatus::On;
				self.assistant_acc.push_str(&delta);
				self.stream(LogKind::Assistant, "", &delta);
			}
			ChatEvent::Thinking(delta) => {
				self.chat_status = ChatStatus::On;
				self.stream(LogKind::Thinking, "[thinking] ", &delta);
			}
			ChatEvent::Tool { state, name, ops, ok, error } => {
				self.chat_status = ChatStatus::On;
				if state == "running" {
					self.push(LogKind::Status, format!("▶ {name} ({ops} ops)…"));
					return;
				}
				// Resolve the matching running line in place when present.
				let resolved = match (ok, &error) {
					(Some(true), _) => (LogKind::StatusOk, format!("✓ {name} ({ops} ops) ok")),
					(_, Some(e)) => (LogKind::StatusErr, format!("✗ {name}: {e}")),
					_ => (LogKind::Status, format!("• {name} ({ops} ops) {state}")),
				};
				let running_prefix = format!("▶ {name}");
				match self.log.iter_mut().rev().find(|e| e.kind == LogKind::Status && e.text.starts_with(&running_prefix)) {
					Some(entry) => {
						entry.kind = resolved.0;
						entry.text = resolved.1.clone();
					}
					None => self.push(resolved.0, resolved.1.clone()),
				}
				self.receipts = vec![
					ReceiptLine { tone: Tone::Head, text: format!("tool {name}") },
					ReceiptLine { tone: if ok == Some(true) { Tone::Ok } else { Tone::Err }, text: resolved.1 },
				];
			}
			ChatEvent::Tasks(tasks) => {
				self.chat_status = ChatStatus::On;
				self.plan = tasks;
			}
			ChatEvent::Subagent { name, state, detail } => {
				self.chat_status = ChatStatus::On;
				match state.as_str() {
					"started" => self.push(LogKind::Subagent, format!("◆ {name} started")),
					"done" => self.push(LogKind::Subagent, format!("✓ {name} done")),
					"error" => {
						let detail = detail.as_deref().unwrap_or("(no detail)");
						self.push(LogKind::StatusErr, format!("✗ {name} error: {detail}"));
					}
					// `tool` and any future intermediate states: name · detail.
					other => {
						let detail = detail.as_deref().unwrap_or(other);
						self.push(LogKind::Subagent, format!("◆ {name} · {detail}"));
					}
				}
			}
			ChatEvent::Refresh { artifacts, receipt } => {
				self.chat_status = ChatStatus::On;
				let summary = receipt.as_ref().map(receipt_summary).filter(|s| !s.is_empty());
				let mut line = format!("⟳ artifacts: {}", artifacts.join(", "));
				if let Some(s) = &summary {
					line.push_str(&format!(" · {s}"));
				}
				self.push(LogKind::Status, line);
				if let Some(s) = summary {
					self.receipts.push(ReceiptLine { tone: Tone::Plain, text: s });
				}
				if !artifacts.is_empty() {
					self.receipts.push(ReceiptLine { tone: Tone::Head, text: "artifacts".into() });
					for file in artifacts {
						self.receipts.push(ReceiptLine { tone: Tone::Plain, text: format!("  {file}") });
					}
				}
			}
			ChatEvent::Disabled(message) => {
				self.chat_status = ChatStatus::Off;
				self.disabled_this_turn = true;
				self.push(LogKind::Error, message);
			}
			ChatEvent::Error(message) => {
				// A server-side API error implies the chat loop exists (key set).
				self.chat_status = ChatStatus::On;
				self.push(LogKind::Error, format!("[error] {message}"));
			}
			ChatEvent::Done(stop_reason) => {
				if !matches!(stop_reason.as_str(), "end_turn" | "chat_disabled") {
					self.push(LogKind::Info, format!("— done ({stop_reason})"));
				}
			}
		}
	}

	fn on_run_report(&mut self, label: &str, names: &HashMap<String, String>, resp: &RunResponse) {
		let n = resp.report.ops.len();
		if resp.report.ok {
			self.push(LogKind::StatusOk, format!("✓ run {label}: ok ({n} ops)"));
		} else {
			let first = resp
				.report
				.ops
				.iter()
				.find_map(|o| o.error.as_ref().map(|e| format!("{}: {}", e.kind, e.message)))
				.unwrap_or_else(|| "failed (no structured error?)".into());
			self.push(LogKind::StatusErr, format!("✗ run {label}: {first}"));
		}
		if !resp.artifacts.is_empty() {
			let files: Vec<&str> = resp.artifacts.iter().map(|a| a.file.as_str()).collect();
			self.push(LogKind::Status, format!("⟳ artifacts: {}", files.join(", ")));
		}
		let verdict = if resp.report.ok { "ok" } else { "FAIL" };
		let mut receipts = vec![ReceiptLine { tone: Tone::Head, text: format!("run {label} · {verdict} · session {}", resp.session) }];
		for op in &resp.report.ops {
			receipts.push(ReceiptLine {
				tone: if op.ok { Tone::Ok } else { Tone::Err },
				text: op_line(op, names.get(&op.id).map(String::as_str)),
			});
		}
		if !resp.artifacts.is_empty() {
			receipts.push(ReceiptLine { tone: Tone::Head, text: "artifacts".into() });
			for a in &resp.artifacts {
				receipts.push(ReceiptLine { tone: Tone::Plain, text: format!("  {}", a.file) });
			}
		}
		self.receipts = receipts;
	}

	fn on_catalog(&mut self, v: &Value) {
		let count = v.get("count").and_then(Value::as_u64).unwrap_or(0);
		self.push(LogKind::Info, format!("catalog: {count} standard-part families"));
		let mut by_category: std::collections::BTreeMap<&str, Vec<&str>> = Default::default();
		for family in v.get("families").and_then(Value::as_array).into_iter().flatten() {
			let category = family.get("category").and_then(Value::as_str).unwrap_or("?");
			let op = family.get("op").and_then(Value::as_str).unwrap_or("?");
			by_category.entry(category).or_default().push(op);
		}
		for (category, ops) in by_category {
			self.push(LogKind::Info, format!("  {category}: {}", ops.join(", ")));
		}
	}

	// -- input ---------------------------------------------------------------

	fn on_key(&mut self, key: KeyEvent) {
		self.hint = None;
		if key.modifiers.contains(KeyModifiers::CONTROL) {
			match key.code {
				KeyCode::Char('c') => self.quit = true,
				KeyCode::Char('u') => self.input.clear(),
				_ => {}
			}
			self.esc_armed = false;
			return;
		}
		match key.code {
			KeyCode::Esc => {
				if self.esc_armed {
					self.quit = true;
				} else {
					self.esc_armed = true;
					self.hint = Some("Esc again to quit".into());
				}
				return;
			}
			KeyCode::Enter => self.submit(),
			KeyCode::Backspace => {
				self.input.pop();
			}
			KeyCode::PageUp => {
				self.follow = false;
				self.scroll = self.scroll.saturating_sub(self.log_page.max(1));
			}
			KeyCode::PageDown => {
				self.scroll = self.scroll.saturating_add(self.log_page.max(1));
				if self.scroll >= self.log_max_scroll {
					self.follow = true;
				}
			}
			KeyCode::Char(c) => self.input.push(c),
			_ => {}
		}
		self.esc_armed = false;
	}

	fn submit(&mut self) {
		let text = self.input.trim().to_string();
		if text.is_empty() {
			return;
		}
		if let Some(rest) = text.strip_prefix(':') {
			self.input.clear();
			self.push(LogKind::User, format!("› {text}"));
			self.command(rest);
			return;
		}
		if let Some(what) = self.busy {
			self.hint = Some(format!("busy ({what}) — wait for it to finish"));
			return; // keep the typed prompt in the input line
		}
		self.input.clear();
		self.push(LogKind::User, format!("› {text}"));
		self.history.push(ChatTurn { role: "user".into(), content: text });
		self.assistant_acc.clear();
		self.disabled_this_turn = false;
		self.busy = Some("chat");
		let client = self.client.clone();
		let messages = self.history.clone();
		let session = self.session.clone();
		let tx = self.tx.clone();
		std::thread::spawn(move || {
			let result = client.chat(&messages, &session, |event| {
				let _ = tx.send(Msg::Chat(event));
			});
			let _ = tx.send(Msg::ChatEnded(result));
		});
	}

	fn command(&mut self, line: &str) {
		let (cmd, arg) = match line.split_once(' ') {
			Some((c, a)) => (c, a.trim()),
			None => (line, ""),
		};
		match cmd {
			"q" | "quit" => self.quit = true,
			"clear" => {
				self.log.clear();
				self.history.clear();
				self.assistant_acc.clear();
				self.plan.clear();
				self.push(LogKind::Info, "log, chat history and plan cleared");
			}
			"session" => {
				let valid = !arg.is_empty()
					&& arg.len() <= 64
					&& arg.chars().all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-');
				if valid {
					self.session = arg.to_string();
					self.push(LogKind::Info, format!("session → {arg}"));
				} else {
					self.push(LogKind::Error, format!("invalid session '{arg}': use [A-Za-z0-9_-], at most 64 chars"));
				}
			}
			"run" => self.cmd_run(arg),
			"ops" => self.cmd_network("ops", |client, tx| {
				let _ = tx.send(Msg::Ops(client.list_ops()));
			}),
			"catalog" => self.cmd_network("catalog", |client, tx| {
				let _ = tx.send(Msg::Catalog(client.catalog()));
			}),
			other => self.push(LogKind::Error, format!("unknown command :{other} — commands: {COMMANDS}")),
		}
	}

	/// Spawn one network worker unless something is already in flight.
	fn cmd_network(&mut self, what: &'static str, job: impl FnOnce(Client, Sender<Msg>) + Send + 'static) {
		if let Some(busy) = self.busy {
			self.push(LogKind::Error, format!("busy ({busy}) — wait for it to finish"));
			return;
		}
		self.busy = Some(what);
		let client = self.client.clone();
		let tx = self.tx.clone();
		std::thread::spawn(move || job(client, tx));
	}

	fn cmd_run(&mut self, arg: &str) {
		if arg.is_empty() {
			self.push(LogKind::Error, "usage: :run <file.json>");
			return;
		}
		let text = match std::fs::read_to_string(arg) {
			Ok(t) => t,
			Err(e) => {
				self.push(LogKind::Error, format!("cannot read '{arg}': {e}"));
				return;
			}
		};
		let program: Value = match serde_json::from_str(&text) {
			Ok(v) => v,
			Err(e) => {
				self.push(LogKind::Error, format!("'{arg}' is not valid JSON: {e}"));
				return;
			}
		};
		// Bare {"ops": ...} is wrapped so the run lands in the current session;
		// an explicit {"program": ...} envelope is respected (session injected
		// only when absent).
		let body = if program.get("ops").is_some() {
			json!({"program": program, "session": self.session})
		} else if program.get("program").is_some() {
			let mut wrapped = program.clone();
			if wrapped.get("session").is_none() {
				wrapped["session"] = json!(self.session);
			}
			wrapped
		} else {
			self.push(LogKind::Error, format!("'{arg}' is not a work order: expected {{\"ops\": [...]}} or {{\"program\": ...}}"));
			return;
		};
		let names = op_names_of(&body);
		let ops = body.pointer("/program/ops").and_then(Value::as_array).map(Vec::len).unwrap_or(0);
		self.push(LogKind::Status, format!("▶ run {arg} ({ops} ops)…"));
		let label = arg.to_string();
		self.cmd_network("run", move |client, tx| {
			let result = client.run_program(&body);
			let _ = tx.send(Msg::RunDone { label, names, result: Box::new(result) });
		});
	}

	// -- drawing ---------------------------------------------------------------

	fn draw(&mut self, f: &mut Frame) {
		let rows = Layout::vertical([Constraint::Length(1), Constraint::Min(3), Constraint::Length(1)]).split(f.area());
		let cols = Layout::horizontal([Constraint::Min(20), Constraint::Length(RECEIPTS_WIDTH)]).split(rows[1]);
		self.draw_header(f, rows[0]);
		self.draw_log(f, cols[0]);
		self.draw_side(f, cols[1]);
		self.draw_input(f, rows[2]);
	}

	/// The right-hand pane: PLAN over RECEIPTS when a plan exists, otherwise
	/// receipts get the whole pane.
	fn draw_side(&mut self, f: &mut Frame, area: Rect) {
		if self.plan.is_empty() {
			self.draw_receipts(f, area);
			return;
		}
		// Enough rows for every task + borders, capped at half the pane so
		// receipts always keep room.
		let want = (self.plan.len() as u16).saturating_add(2);
		let cap = (area.height / 2).max(3);
		let parts = Layout::vertical([Constraint::Length(want.min(cap)), Constraint::Min(3)]).split(area);
		self.draw_plan(f, parts[0]);
		self.draw_receipts(f, parts[1]);
	}

	fn draw_plan(&self, f: &mut Frame, area: Rect) {
		let block = Block::bordered().title(" PLAN ").border_style(Style::new().fg(Color::DarkGray));
		let inner_width = area.width.saturating_sub(2).max(6) as usize;
		let inner_height = area.height.saturating_sub(2) as usize;
		// One line per task, truncated; if the plan overflows the capped pane,
		// say how many are hidden instead of silently dropping them.
		let visible = if self.plan.len() > inner_height { inner_height.saturating_sub(1) } else { self.plan.len() };
		let mut lines: Vec<Line> = Vec::new();
		for task in &self.plan[..visible] {
			let style = match task.status.as_str() {
				"in_progress" => Style::new().fg(Color::Yellow).add_modifier(Modifier::BOLD),
				"completed" => Style::new().fg(Color::DarkGray),
				_ => Style::new(),
			};
			let text = truncate_line(&format!("{} {}", task_mark(&task.status), task.content), inner_width);
			lines.push(Line::from(Span::styled(text, style)));
		}
		let hidden = self.plan.len() - visible;
		if hidden > 0 {
			lines.push(Line::from(Span::styled(format!("… +{hidden} more"), Style::new().fg(Color::DarkGray))));
		}
		f.render_widget(Paragraph::new(Text::from(lines)).block(block), area);
	}

	fn draw_header(&self, f: &mut Frame, area: Rect) {
		let chat = match self.chat_status {
			ChatStatus::Unknown => "?",
			ChatStatus::On => "on",
			ChatStatus::Off => "off",
		};
		let mut spans = vec![
			Span::styled("CAD CODE", Style::new().add_modifier(Modifier::BOLD)),
			Span::raw(format!(" · {} · session {} · chat {chat}", self.client.base, self.session)),
		];
		if let Some(what) = self.busy {
			spans.push(Span::styled(format!(" · {what}…"), Style::new().fg(Color::Yellow)));
		}
		if !self.follow {
			spans.push(Span::styled(" · [scrolled — PgDn to follow]", Style::new().fg(Color::DarkGray)));
		}
		f.render_widget(Paragraph::new(Line::from(spans)).style(Style::new().bg(Color::Rgb(30, 30, 46))), area);
	}

	fn draw_log(&mut self, f: &mut Frame, area: Rect) {
		let width = area.width.saturating_sub(1).max(8) as usize;
		let mut lines: Vec<Line> = Vec::new();
		for entry in &self.log {
			let style = entry.kind.style();
			for piece in wrap_text(&entry.text, width) {
				lines.push(Line::from(Span::styled(piece, style)));
			}
		}
		let total = lines.len();
		let view = area.height as usize;
		self.log_page = view.max(1);
		self.log_max_scroll = total.saturating_sub(view);
		if self.follow {
			self.scroll = self.log_max_scroll;
		} else {
			self.scroll = self.scroll.min(self.log_max_scroll);
		}
		let offset = u16::try_from(self.scroll).unwrap_or(u16::MAX);
		f.render_widget(Paragraph::new(Text::from(lines)).scroll((offset, 0)), area);
	}

	fn draw_receipts(&self, f: &mut Frame, area: Rect) {
		let block = Block::bordered().title(" RECEIPTS ").border_style(Style::new().fg(Color::DarkGray));
		let inner_width = area.width.saturating_sub(2).max(8) as usize;
		let inner_height = area.height.saturating_sub(2) as usize;
		let mut lines: Vec<Line> = Vec::new();
		for r in &self.receipts {
			let style = match r.tone {
				Tone::Head => Style::new().add_modifier(Modifier::BOLD),
				Tone::Ok => Style::new().fg(Color::Green),
				Tone::Err => Style::new().fg(Color::Red),
				Tone::Plain => Style::new(),
			};
			for piece in wrap_text(&r.text, inner_width) {
				lines.push(Line::from(Span::styled(piece, style)));
			}
		}
		// Failures and artifacts land at the report tail — keep the tail visible.
		if lines.len() > inner_height && inner_height > 1 {
			let hidden = lines.len() - (inner_height - 1);
			let mut tail: Vec<Line> = lines.split_off(hidden);
			let mut shown = vec![Line::from(Span::styled(format!("… {hidden} more above"), Style::new().fg(Color::DarkGray)))];
			shown.append(&mut tail);
			lines = shown;
		}
		f.render_widget(Paragraph::new(Text::from(lines)).block(block), area);
	}

	fn draw_input(&self, f: &mut Frame, area: Rect) {
		let avail = area.width.saturating_sub(3) as usize;
		let chars: Vec<char> = self.input.chars().collect();
		let visible: String = if chars.len() > avail { chars[chars.len() - avail..].iter().collect() } else { self.input.clone() };
		let mut spans = vec![Span::styled("› ", Style::new().fg(Color::Cyan).add_modifier(Modifier::BOLD)), Span::raw(visible.clone())];
		if let Some(hint) = &self.hint {
			spans.push(Span::styled(format!("   — {hint}"), Style::new().fg(Color::DarkGray)));
		}
		f.render_widget(Paragraph::new(Line::from(spans)), area);
		let cursor_x = area.x + 2 + visible.chars().count() as u16;
		f.set_cursor_position((cursor_x.min(area.x + area.width.saturating_sub(1)), area.y));
	}
}

/// Truncate to `width` chars, ending in `…` when anything was cut (single
/// plan line — long task content must never wrap the checklist).
fn truncate_line(text: &str, width: usize) -> String {
	if text.chars().count() <= width {
		return text.to_string();
	}
	let mut out: String = text.chars().take(width.saturating_sub(1)).collect();
	out.push('…');
	out
}

/// Soft-wrap `text` to `width` columns (char-count based), breaking at the
/// last space inside the window when one exists. `\n` in the text is honored.
fn wrap_text(text: &str, width: usize) -> Vec<String> {
	let width = width.max(1);
	let mut out = Vec::new();
	for raw in text.split('\n') {
		let chars: Vec<char> = raw.chars().collect();
		if chars.is_empty() {
			out.push(String::new());
			continue;
		}
		let mut start = 0;
		while start < chars.len() {
			let end = (start + width).min(chars.len());
			let mut cut = end;
			if end < chars.len() {
				if let Some(space) = (start..end).rev().find(|&i| chars[i] == ' ') {
					if space > start {
						cut = space;
					}
				}
			}
			out.push(chars[start..cut].iter().collect());
			start = if cut < chars.len() && chars[cut] == ' ' { cut + 1 } else { cut };
		}
	}
	out
}

#[cfg(test)]
mod tests {
	use super::{truncate_line, wrap_text};

	/// Plan lines are truncated, never wrapped: at most `width` chars, `…`
	/// marks a cut, short lines pass through untouched.
	#[test]
	fn truncate_line_marks_cuts() {
		assert_eq!(
			(
				truncate_line("☐ short task", 20),
				truncate_line("◐ a very long task description that overflows", 20),
				truncate_line("◐ a very long task description that overflows", 20).chars().count()
			),
			("☐ short task".to_string(), "◐ a very long task …".to_string(), 20),
			"truncation must cap at width chars and end in … only when content was cut"
		);
	}

	/// The wrapper must terminate and cover every char exactly once per input
	/// line, prefer space breaks, honor embedded newlines, and never emit a
	/// line wider than the target.
	#[test]
	fn wrap_text_invariants() {
		let cases = [
			("hello world this is a long line of text", 10),
			("nospacesatallinthisverylongtoken", 8),
			("a\n\nb", 5),
			("", 7),
			("exact fit!", 10),
			("trailing space ", 6),
		];
		for (text, width) in cases {
			let lines = wrap_text(text, width);
			assert!(
				lines.iter().all(|l| l.chars().count() <= width),
				"wrap({text:?}, {width}): line wider than {width}: {lines:?}"
			);
			// Breaks may replace a space or fall mid-token, so compare content
			// with all whitespace stripped: nothing lost, nothing invented.
			let rejoined: String = lines.concat().chars().filter(|c| !c.is_whitespace()).collect();
			let original: String = text.chars().filter(|c| !c.is_whitespace()).collect();
			assert_eq!(rejoined, original, "wrap({text:?}, {width}) lost or invented content: {lines:?}");
		}
	}
}
