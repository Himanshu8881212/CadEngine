// Copyright (c) LMCAD. Licensed under the MIT License.

//! `/api/chat` — the CAD agent harness, streamed to the client as SSE.
//!
//! The server proxies the Anthropic Messages API (model `claude-opus-4-8`,
//! raw HTTP — there is no official Rust SDK) and runs a Claude-Code-style
//! agentic loop over the CAD kernel. The PARENT orchestrator gets four tools:
//!
//! - `run_program` — execute a work-order JSON program in the in-process kernel
//!   (the executor `/api/run` uses).
//! - `describe_api` — self-serve documentation: the live op catalogue (a real
//!   `describe` run) and per-op sections extracted from the repo-root `API.md`.
//! - `task_update` — TodoWrite-style plan tracking; a pure echo to the client.
//! - `spawn_subagents` — fan out up to 8 parallel PART AGENTS (concurrency 4),
//!   each running its own fresh loop with ONLY `run_program` + `describe_api`
//!   (one level deep — children cannot spawn). Child text does NOT stream to
//!   the client; children surface as condensed `subagent` frames and return
//!   `{name, ok, receipts, summary}` to the parent as the tool result.
//!
//! Tool-use turns loop **server-side**; the client receives a flat stream:
//!
//! | SSE event | data | meaning |
//! |---|---|---|
//! | `text` | `{delta}` | parent assistant text delta (child text never streams) |
//! | `thinking` | `{delta}` | parent thinking summary delta (`display: "summarized"`) |
//! | `tool` | `{state, name, ops?, ok?, error?, program?}` | tool-call status line (`running` → `done`) for `run_program` / `describe_api` |
//! | `tasks` | `{tasks: [{content, status}]}` | the model's current plan (a `task_update` echo — render as a checklist) |
//! | `subagent` | `{name, state, detail?}` | part-agent lifecycle: `started` → `tool`* → `done` \| `error` |
//! | `refresh` | `{artifacts, receipt?}` | a run exported meshes — reload the viewport (parent AND child runs) |
//! | `chat_disabled` | `{message}` | no `ANTHROPIC_API_KEY`; rest of the app still works |
//! | `error` | `{message}` | transport/API failure (surfaced, never retried silently) |
//! | `done` | `{stop_reason}` | the loop finished |
//!
//! `task_update` and `spawn_subagents` emit their dedicated events (`tasks` /
//! `subagent`) instead of `tool` frames. Everything that existed before this
//! harness stays byte-compatible.
//!
//! No sampling parameters are sent (removed on `claude-opus-4-8`); thinking is
//! adaptive with summarized display so the client can render THINKING blocks.
//!
//! Turn budgets: 32 parent / 16 per child, overridable via `CADCODE_MAX_TURNS`
//! / `CADCODE_SUBAGENT_MAX_TURNS` (parsed once, clamped to 1..=100).

use std::path::PathBuf;
use std::sync::{Arc, OnceLock};

use axum::extract::State;
use axum::response::sse::{Event, KeepAlive, Sse};
use axum::response::{IntoResponse, Response};
use axum::Json;
use futures_util::StreamExt;
use kernel_api::Report;
use serde::Deserialize;
use serde_json::{json, Value};
use tokio::sync::mpsc;
use tokio_stream::wrappers::ReceiverStream;

use crate::run::artifacts_of;
use crate::AppState;

/// Default server-side tool-use turn budget for the parent orchestrator.
const DEFAULT_PARENT_TURNS: usize = 32;
/// Default turn budget for each spawned part agent.
const DEFAULT_CHILD_TURNS: usize = 16;
/// Hard cap on part agents per `spawn_subagents` call.
const MAX_SUBAGENTS: usize = 8;
/// How many part agents run concurrently (the rest queue, order preserved).
const SUBAGENT_CONCURRENCY: usize = 4;
/// Tool results larger than this are truncated (the model still gets the
/// leading content + a truncation note; the full report stayed on the server).
const MAX_TOOL_RESULT: usize = 60_000;
/// The exact model id (see the project Claude-API guidance; do not suffix).
const MODEL: &str = "claude-opus-4-8";

/// Parse-and-clamp a turn-budget override: integers clamp to 1..=100,
/// anything unparseable falls back to `default`.
fn clamp_turns(raw: Option<String>, default: usize) -> usize {
	raw.and_then(|s| s.trim().parse::<usize>().ok()).map(|n| n.clamp(1, 100)).unwrap_or(default)
}

/// Parent turn budget (`CADCODE_MAX_TURNS`, parsed once per process).
fn parent_max_turns() -> usize {
	static V: OnceLock<usize> = OnceLock::new();
	*V.get_or_init(|| clamp_turns(std::env::var("CADCODE_MAX_TURNS").ok(), DEFAULT_PARENT_TURNS))
}

/// Per-child turn budget (`CADCODE_SUBAGENT_MAX_TURNS`, parsed once per process).
fn child_max_turns() -> usize {
	static V: OnceLock<usize> = OnceLock::new();
	*V.get_or_init(|| clamp_turns(std::env::var("CADCODE_SUBAGENT_MAX_TURNS").ok(), DEFAULT_CHILD_TURNS))
}

/// Anthropic API base URL (`ANTHROPIC_BASE_URL`, parsed once per process; default the real API).
/// The standard SDK override: lets deployments route through a proxy, and lets the harness be
/// live-fired against a local model shim without touching this code.
fn anthropic_base_url() -> &'static str {
	static V: OnceLock<String> = OnceLock::new();
	V.get_or_init(|| {
		std::env::var("ANTHROPIC_BASE_URL")
			.ok()
			.map(|s| s.trim().trim_end_matches('/').to_string())
			.filter(|s| !s.is_empty())
			.unwrap_or_else(|| "https://api.anthropic.com".to_string())
	})
}

/// One prior conversation turn from the client (text only — tool turns live
/// server-side within a single request).
#[derive(Deserialize)]
pub struct ChatTurn {
	/// `"user"` or `"assistant"`.
	pub role: String,
	/// Plain text content.
	pub content: String,
}

/// Request of `/api/chat`.
#[derive(Deserialize)]
pub struct ChatRequest {
	/// Full conversation history, oldest first, ending with the new user turn.
	pub messages: Vec<ChatTurn>,
	/// Session whose out-dir receives exported meshes.
	pub session: Option<String>,
}

/// Shared prompt core: the work-order grammar (DESIGN_GUIDE §4, enforced),
/// the op-family map and the working style. Both the orchestrator and every
/// part agent carry this — it is what makes their programs valid.
const PROMPT_CORE: &str = r#"WORK-ORDER GRAMMAR (enforced by the engine):
- A work order is {"ops": [{"id": ..., "op": ..., ...params}, ...]}, executed top-to-bottom; execution stops at the FIRST failing op and the report carries one structured error (kind + message).
- Each "id" is unique; later ops reference earlier results via "in" / "a" / "b" / "sketch". Geometry-producing ops bind their result to their id; measure/export/assert/design-math ops bind NOTHING (referencing one is a missing_ref error).
- Units are mm; ALL angles on this surface are degrees ("degrees", "*_deg"). Bores and shanks are DIAMETERS, never radii; hex sizes are across flats.
- "extrude" profiles must be CCW simple polygons (CW fails invalid_geometry); extrude_with_holes / extrude_tapered / revolve / sketch sweeps re-wind automatically.
- Unknown fields are SILENTLY IGNORED, including misspelled optional params (the default quietly applies) — verify op names and parameters with describe_api instead of guessing, and confirm a doubtful param took effect with a measure. Misspelling a required param is a loud invalid_param.
- An empty boolean result is an op failure (invalid_param), not an empty solid. To prove disjointness use assert_disjoint, or union + assert {"shells": N}.
- "pose" = rotation about an arbitrary axis (optional center) THEN translate; needs at least one part.
- "assert" needs ≥1 check of: volume_within {target, percent|abs} / exact_volume_within / genus / shells / closed / manifold / valid. Declare acceptance criteria as assert ops so they travel with the design.
- Solid-producing ops are gated through validate(); invalid geometry is never bound silently.

OP FAMILIES (describe_api with no argument lists every op; with {op} it returns the op's parameter table and a worked example):
- Constructors: box{min,max}, cylinder{base,axis,r|d,h}, sphere, cone, torus, extrude{profile,height}, extrude_with_holes, extrude_tapered, revolve.
- Sketches: sketch{points,segments,constraints (fixed/horizontal/vertical/distance/...)} → sketch_extrude / sketch_revolve. Read the solver receipt: state should be "well_constrained", free_dof 0.
- Booleans: union, difference, intersection, union_all.
- Features: fillet_edge_near{in,at,radius}, chamfer_edge_near, fillet_circular_rim; transforms: translate{in,by}, rotate_z{in,degrees}, pose.
- Measures: validate, volume, exact_volume (π-exact via surface tags), mass_properties, bounding_box, wall_thickness, draft_analysis, support_report (FDM support-necessity audit). Assertions: assert, assert_disjoint.
- Exports: export_stl{in,file}, export_step, export_3mf. The report names the mesh route: "exact" or "voxel_healed" — relay it honestly, never claim exactness on a healed route.
- Implicit half: gyroid_block, implicit{expr,voxel,mesher} (expression trees: SDF leaves, csg/blend combinators like fillet_union/chamfer_difference, expr_sdf scalar fields) — for lattices, threads, organic blends.
- Native formats: load_part{file} (.lmcpart recipes; relative paths resolve against the repo root). Library: library_add/search/instantiate/deprecate/remove.
- Hole wizard (cuts into an existing solid, axis points INTO the material): drill{in,at,axis,d,through}, clearance_hole{m,fit}, counterbore_hole{m}, countersink_hole{m}, tap_drill_hole{m}, bolt_circle{center,axis,circle_d,n,hole:{kind,m}}, bearing_seat{designation}.
- Standard parts catalog (ISO/DIN/ANSI tables, build at origin along +Z, place with pose): spur_gear, gear_rack, internal_gear, gt2_pulley, chain_sprocket, hex_bolt, hex_nut, washer, spring_washer, socket_head_cap_screw, flat_head_screw, button_head_screw, set_screw, lock_nut, threaded_rod, standoff, shoulder_bolt, shaft, parallel_key, dowel_pin, circlip_external, circlip_internal, deep_groove_bearing, flanged_bearing, thrust_bearing, kp08_pillow_block, linear_bearing_lmuu, sc8uu_block, shaft_support_sk8, shaft_support_shf8, mgn12_rail, mgn12_carriage, nema_motor, nema_mount_plate, extrusion_2020, extrusion_3030, tnut_2020, lead_screw_tr8, lead_screw_nut_tr8, compression_spring, o_ring, o_ring_cord, jaw_coupling_hub, jaw_coupling_spider, set_screw_coupling, clamp_coupling, pipe_boss_g, hose_barb.
- Standard feature cuts: heatset_insert_boss, circlip_groove_external/internal, o_ring_groove, o_ring_face_gland(_racetrack), tr8_nut_trap, nema_mount_cut, servo_pocket, pc4_port, teardrop_hole, board_mount, bridged_counterbore.
- Design math (bind nothing, return numbers): gt2_belt, gt2_center_distance, iso286_fit, heatset_spec, metric_cord_gland, racetrack_cord_length, pipe_thread_g.

WORKING STYLE:
- ALWAYS end a geometry-building work order with export_stl (the user's viewport shows it immediately). Use simple lowercase file names like "part.stl".
- Prefer catalog ops over modelling standard hardware; prefer the hole wizard over hand-built cutters.
- Gate intent with assert ops (genus, volume windows) instead of prose claims; when a run fails, read the structured error kind, fix the program, and run again (a few attempts at most).
- Relay the kernel's receipts (volume, route, watertight, genus) — exact numbers, honest routes.
- Keep work orders small and readable; build incrementally rather than one giant program."#;

/// The orchestrator's role brief: receipts doctrine plus the harness contract
/// (plan via task_update, verify via describe_api, delegate via
/// spawn_subagents, gates define done).
const PARENT_ROLE: &str = r#"You are the LMCAD orchestrator inside LMCAD Studio: you design real, manufacturable parts for the user by driving a hybrid CAD kernel (exact B-rep + implicit/voxel halves) through JSON work orders. The ONLY way you make or measure geometry is the run_program tool. Everything you claim about geometry must come from its report — the kernel hands receipts; never claim beyond the report.

HOW YOU WORK (the harness contract):
- PLAN FIRST. For anything beyond a one-op tweak, post a short task plan via task_update before building, then KEEP IT CURRENT: mark a task in_progress when you start it and completed the moment its gates pass. The user watches this plan live — a stale plan is a broken promise.
- VERIFY BEFORE GUESSING. Use describe_api before guessing an op's parameters — with no argument it lists every op; with {op} it returns the op's parameter table and a worked example. The engine SILENTLY IGNORES unknown params (a typo'd optional param just applies the default and fails no test), so verify op names via describe_api and confirm doubtful params with a measure.
- DECOMPOSE AND DELEGATE. For multi-part designs, split the work and call spawn_subagents with ONE AGENT PER INDEPENDENT PART (they run in parallel); sequence DEPENDENT work yourself — children cannot see each other's results and cannot spawn further agents. Write each brief as a complete, self-contained spec: exact dimensions, interfaces and mating features, the gates it must pass, and a distinct export file name (children share your session's output directory). Children return receipts plus a summary; verify their receipts before relaying any claim.
- GATES DEFINE DONE. A part is DONE only when its gates pass through run_program: validate → geometric_ok:true; a watertight export with the route noted (exact vs voxel_healed — never claim exactness on a healed route); support_report when the part is to be FDM printed. Declare acceptance criteria as assert ops so they travel with the design.
- REPORT HONESTLY. Relay exact numbers from reports; when a run fails, name the structured error kind, say what you changed, and run again."#;

/// A part agent's role brief (its name is spliced in): build exactly the
/// briefed part, gate it, and end with a terse receipt summary.
const CHILD_ROLE: &str = r#"a focused LMCAD part agent inside LMCAD Studio. You build EXACTLY ONE part — the one specified in the brief that follows — by driving a hybrid CAD kernel (exact B-rep + implicit/voxel halves) through JSON work orders. Do not redesign the brief, do not add extra parts, and do not ask questions: build it, gate it, report it.

RULES:
- The ONLY way you make or measure geometry is the run_program tool. Everything you claim must come from its report.
- Consult describe_api before guessing an op's parameters (no argument lists every op; {op} returns its parameter table and a worked example). Unknown params are SILENTLY IGNORED by the engine, so verify names instead of guessing.
- End with the FULL gate stack green through run_program: validate → geometric_ok:true; assert ops for the brief's acceptance criteria; a watertight export (use the brief's file name) with the route noted; support_report if the brief says the part will be printed.
- If a run fails, read the structured error kind, fix the program, and retry (a few attempts at most). If the part is truly unbuildable as briefed, stop and say exactly why, quoting the error.
- Your FINAL message is a terse receipt summary — key dimensions built, gates passed with their numbers (volume, genus, watertight, route), and the artifact file name — NOT prose. The orchestrator reads it verbatim."#;

/// The full orchestrator system prompt (role + shared core).
fn parent_system_prompt() -> String {
	format!("{PARENT_ROLE}\n\n{PROMPT_CORE}")
}

/// The full part-agent system prompt for the child named `name`.
fn child_system_prompt(name: &str) -> String {
	format!("You are part agent \"{name}\", {CHILD_ROLE}\n\n{PROMPT_CORE}")
}

/// Tool definition: `run_program` — the kernel work-order executor.
fn tool_run_program() -> Value {
	json!({
		"name": "run_program",
		"description": "Execute an LMCAD work-order JSON program ({\"ops\": [...]}) in the kernel. Ops run top-to-bottom; the result is the full kernel report: per-op ok, measures (volume/genus/route/...), written files, and a structured error for the first failing op. Exported STLs appear in the user's 3D viewport immediately.",
		"input_schema": {
			"type": "object",
			"properties": {
				"program": {
					"type": "object",
					"description": "The work order: an object with an \"ops\" array. Each op: {\"id\": unique string, \"op\": op name, ...params}."
				}
			},
			"required": ["program"]
		}
	})
}

/// Tool definition: `describe_api` — self-serve op documentation.
fn tool_describe_api() -> Value {
	json!({
		"name": "describe_api",
		"description": "Self-serve documentation for the work-order surface. With no arguments: the full op catalogue (every op name + count) from a live in-process describe run. With {op}: whether the op exists AND its documentation section from API.md — parameter table (names, types, required/optional), conventions, and a worked JSON example. ALWAYS call this before guessing an op's parameters: unknown params are silently ignored by the engine, so a typo'd param fails no test.",
		"input_schema": {
			"type": "object",
			"properties": {
				"op": {
					"type": "string",
					"description": "Op name to document (e.g. \"box\", \"teardrop_hole\"). Omit to list the full op catalogue."
				}
			}
		}
	})
}

/// Tool definition: `task_update` — TodoWrite-style plan tracking (pure echo).
fn tool_task_update() -> Value {
	json!({
		"name": "task_update",
		"description": "Post or update your visible task plan. Send the FULL current list every time (not a delta); the harness stores nothing — the list is streamed straight to the user's UI so they can watch plan progress. Update it whenever a status changes: mark a task in_progress when you start it and completed the moment its gates pass. Returns \"ok\".",
		"input_schema": {
			"type": "object",
			"properties": {
				"tasks": {
					"type": "array",
					"description": "The full current plan, in order.",
					"items": {
						"type": "object",
						"properties": {
							"content": {"type": "string", "description": "Short imperative task description."},
							"status": {"type": "string", "enum": ["pending", "in_progress", "completed"]}
						},
						"required": ["content", "status"]
					}
				}
			},
			"required": ["tasks"]
		}
	})
}

/// Tool definition: `spawn_subagents` — parallel part-agent fan-out.
fn tool_spawn_subagents() -> Value {
	json!({
		"name": "spawn_subagents",
		"description": "Fan out parallel PART AGENTS — one per INDEPENDENT part of a multi-part design. Each agent starts a fresh context containing only its brief, builds exactly that part via run_program (describe_api available; no further spawning), and returns {name, ok, receipts, summary}. Agents run concurrently (max 8 per call, 4 at a time) and share your session's output directory, so give every brief a DISTINCT export file name and a complete self-contained spec (dimensions, interfaces, gates). Sequence DEPENDENT parts yourself — agents cannot see each other's results.",
		"input_schema": {
			"type": "object",
			"properties": {
				"agents": {
					"type": "array",
					"description": "One entry per independent part (1..8).",
					"items": {
						"type": "object",
						"properties": {
							"name": {"type": "string", "description": "Short unique agent name, e.g. the part name."},
							"brief": {"type": "string", "description": "Complete self-contained part spec: exact dimensions, interfaces/mating features, required gates, export file name."}
						},
						"required": ["name", "brief"]
					}
				}
			},
			"required": ["agents"]
		}
	})
}

/// The orchestrator's tool set, in the order the model sees it.
fn parent_tools() -> Vec<Value> {
	vec![tool_run_program(), tool_describe_api(), tool_task_update(), tool_spawn_subagents()]
}

/// A part agent's tool set: build + document only (no recursion, no plan echo).
fn child_tools() -> Vec<Value> {
	vec![tool_run_program(), tool_describe_api()]
}

/// Build a viewport receipt from a run report's measures (exact_volume /
/// volume + the export op's route/triangles/watertight), when present.
fn receipt_from_report(report: &Report) -> Option<Value> {
	let mut volume: Option<(f64, &str)> = None;
	let mut route: Option<(String, u64, bool)> = None;
	for op in &report.ops {
		let Some(m) = &op.measures else { continue };
		if let Some(v) = m.get("exact_volume").and_then(Value::as_f64) {
			volume = Some((v, "exact"));
		} else if volume.is_none() {
			if let Some(v) = m.get("volume").and_then(Value::as_f64) {
				volume = Some((v, "mesh"));
			}
		}
		if let Some(r) = m.get("route").and_then(Value::as_str) {
			route = Some((
				r.to_string(),
				m.get("triangles").and_then(Value::as_u64).unwrap_or(0),
				m.get("watertight").and_then(Value::as_bool).unwrap_or(false),
			));
		}
	}
	let (route, tris, watertight) = route?;
	let (volume, source) = volume.unwrap_or((f64::NAN, "mesh"));
	Some(json!({
		"volume": volume,
		"volume_source": source,
		"route": route,
		"why": "from the run report",
		"tris": tris,
		"watertight": watertight,
		"artifact": {"file": "", "url": "", "kind": "stl"},
	}))
}

/// Compact, honest summary of a run report for a subagent receipt: per-op
/// `ok` + every SCALAR measure (numbers/bools/strings — face lists and other
/// dumps are dropped) + written files, and the structured error on the
/// failing op. This is what the parent verifies a child's claims against.
fn receipts_summary(report: &Report) -> Value {
	let ops: Vec<Value> = report
		.ops
		.iter()
		.map(|op| {
			let mut o = serde_json::Map::new();
			o.insert("id".into(), json!(op.id));
			o.insert("ok".into(), json!(op.ok));
			if let Some(map) = op.measures.as_ref().and_then(Value::as_object) {
				let scalars: serde_json::Map<String, Value> = map
					.iter()
					.filter(|(_, v)| !v.is_array() && !v.is_object())
					.map(|(k, v)| (k.clone(), v.clone()))
					.collect();
				if !scalars.is_empty() {
					o.insert("measures".into(), Value::Object(scalars));
				}
			}
			if let Some(f) = &op.file {
				o.insert("file".into(), json!(f));
			}
			if let Some(e) = &op.error {
				o.insert("error".into(), json!(format!("{:?}: {}", e.kind, e.message)));
			}
			Value::Object(o)
		})
		.collect();
	json!({"ok": report.ok, "ops": ops})
}

/// Cap a tool result at [`MAX_TOOL_RESULT`] bytes on a char boundary (the
/// model still gets the leading content + a truncation note; the full report
/// stayed on the server).
fn truncate_result(mut s: String) -> String {
	if s.len() > MAX_TOOL_RESULT {
		let mut cut = MAX_TOOL_RESULT;
		while !s.is_char_boundary(cut) {
			cut -= 1;
		}
		s.truncate(cut);
		s.push_str("\n…[report truncated]");
	}
	s
}

/// The SSE sink for one agent. The parent streams the full protocol; a child
/// (part agent) is CONDENSED for context economy — its text/thinking deltas
/// are dropped (the parent gets the final summary via the tool result) and
/// its tool calls surface as one-line `subagent` frames. `refresh` passes
/// through for both, so a child's exports still reload the viewport.
#[derive(Clone)]
struct Sink {
	tx: mpsc::Sender<Event>,
	/// `None` = the parent orchestrator; `Some(name)` = a child part agent.
	child: Option<String>,
}

impl Sink {
	/// Send one named SSE event with a JSON payload; `false` = client gone.
	async fn event(&self, name: &str, data: Value) -> bool {
		self.tx.send(Event::default().event(name).data(data.to_string())).await.is_ok()
	}

	/// Assistant text delta — streamed for the parent, dropped for children.
	/// `false` = client gone (a suppressed child delta is never a disconnect).
	async fn text_delta(&self, t: &str) -> bool {
		if self.child.is_some() {
			return true;
		}
		self.event("text", json!({"delta": t})).await
	}

	/// Thinking delta — same parent/child policy as [`Sink::text_delta`].
	async fn thinking_delta(&self, t: &str) -> bool {
		if self.child.is_some() {
			return true;
		}
		self.event("thinking", json!({"delta": t})).await
	}

	/// One `subagent` lifecycle frame (`started`/`tool`/`done`/`error`).
	/// No-op on the parent sink.
	async fn subagent(&self, state: &str, detail: Option<String>) -> bool {
		let Some(name) = &self.child else { return true };
		let mut data = json!({"name": name, "state": state});
		if let Some(d) = detail {
			data["detail"] = json!(d);
		}
		self.event("subagent", data).await
	}
}

/// Everything an agent loop needs that is not role-specific: the HTTP client,
/// the API key, and the kernel's directories/session.
struct AgentCtx {
	client: reqwest::Client,
	api_key: String,
	out_dir: PathBuf,
	repo_root: PathBuf,
	session: String,
}

/// Role parameters of one agent loop — the parent/child split of the shared
/// core (emit policy lives on [`Sink`]).
struct AgentParams {
	/// Full system prompt for this role.
	system_prompt: String,
	/// Anthropic tool definitions this role sees.
	tools: Vec<Value>,
	/// Turn budget; exhaustion ends the loop with `stop_reason: "turn_limit"`.
	max_turns: usize,
	/// Only the orchestrator may call `task_update` / `spawn_subagents`
	/// (children calling them is an unknown-tool error — one level deep,
	/// like Claude Code).
	orchestrator: bool,
}

/// How one agent loop ended, plus what its caller needs from it.
struct LoopOutcome {
	/// The API stop reason of the final turn (`end_turn`, `max_tokens`, …),
	/// or `turn_limit` when the budget ran out.
	stop_reason: String,
	/// Transport/API failure, when the loop died on one.
	error: Option<String>,
	/// The SSE client hung up mid-stream (everything stops silently).
	client_gone: bool,
	/// Text of the last assistant turn that had any (a child's receipt summary).
	final_text: String,
	/// [`receipts_summary`] of the LAST `run_program` report, if any ran.
	last_receipts: Option<Value>,
}

/// POST `/api/chat` — see the module docs for the event protocol.
pub async fn chat_endpoint(State(state): State<Arc<AppState>>, Json(req): Json<ChatRequest>) -> Response {
	let (tx, rx) = mpsc::channel::<Event>(64);
	let sink = Sink { tx, child: None };
	let Some(api_key) = state.api_key.clone() else {
		// Graceful no-key path: one explicit event, then done. Everything else
		// in the app keeps working.
		tokio::spawn(async move {
			sink.event("chat_disabled", json!({"message": "chat disabled — set ANTHROPIC_API_KEY"})).await;
			sink.event("done", json!({"stop_reason": "chat_disabled"})).await;
		});
		return sse_response(rx);
	};
	let out_dir = match state.session_dir(req.session.as_deref()) {
		Ok(d) => d,
		Err(e) => return crate::run::bad_request(&e),
	};
	let session = req.session.unwrap_or_else(|| "default".to_string());
	let repo_root = state.repo_root.clone();
	let messages: Vec<Value> = req
		.messages
		.iter()
		.filter(|t| !t.content.trim().is_empty() && (t.role == "user" || t.role == "assistant"))
		.map(|t| json!({"role": t.role, "content": t.content}))
		.collect();
	if messages.is_empty() || messages.last().and_then(|m| m.get("role")).and_then(Value::as_str) != Some("user") {
		return crate::run::bad_request("messages must end with a non-empty user turn");
	}
	let ctx = AgentCtx { client: reqwest::Client::new(), api_key, out_dir, repo_root, session };
	tokio::spawn(agent_loop(sink, ctx, messages));
	sse_response(rx)
}

fn sse_response(rx: mpsc::Receiver<Event>) -> Response {
	Sse::new(ReceiverStream::new(rx).map(Ok::<Event, std::convert::Infallible>))
		.keep_alive(KeepAlive::default())
		.into_response()
}

/// The parent wrapper: run the orchestrator loop, then translate its outcome
/// into the client-facing `error`/`done` events (turn exhaustion stays the
/// honest error it always was).
async fn agent_loop(sink: Sink, ctx: AgentCtx, messages: Vec<Value>) {
	let params = AgentParams {
		system_prompt: parent_system_prompt(),
		tools: parent_tools(),
		max_turns: parent_max_turns(),
		orchestrator: true,
	};
	let outcome = run_agent(&sink, &ctx, &params, messages).await;
	if outcome.client_gone {
		return;
	}
	if let Some(e) = outcome.error {
		sink.event("error", json!({"message": e})).await;
		sink.event("done", json!({"stop_reason": "error"})).await;
		return;
	}
	if outcome.stop_reason == "turn_limit" {
		sink.event("error", json!({"message": format!("tool loop stopped after {} turns", params.max_turns)})).await;
		sink.event("done", json!({"stop_reason": "turn_limit"})).await;
		return;
	}
	sink.event("done", json!({"stop_reason": outcome.stop_reason})).await;
}

/// The shared agent-loop core (parent AND children run THIS): stream a model
/// turn, surface deltas per the sink's policy, execute tool calls, feed all
/// results back in one user message, repeat until a terminal stop reason or
/// the turn budget runs out.
async fn run_agent(sink: &Sink, ctx: &AgentCtx, params: &AgentParams, mut messages: Vec<Value>) -> LoopOutcome {
	let mut last_receipts: Option<Value> = None;
	let mut final_text = String::new();
	let fail = |stop: &str, error: Option<String>, client_gone: bool, final_text: &str, receipts: &Option<Value>| LoopOutcome {
		stop_reason: stop.to_string(),
		error,
		client_gone,
		final_text: final_text.to_string(),
		last_receipts: receipts.clone(),
	};
	for _turn in 0..params.max_turns {
		let body = json!({
			"model": MODEL,
			"max_tokens": 16000,
			"stream": true,
			"thinking": {"type": "adaptive", "display": "summarized"},
			"system": params.system_prompt,
			"tools": params.tools,
			"messages": messages,
		});
		let turn = match stream_one_turn(sink, &ctx.client, &ctx.api_key, &body).await {
			Ok(Some(t)) => t,
			Ok(None) => return fail("client_gone", None, true, &final_text, &last_receipts),
			Err(e) => return fail("error", Some(e), false, &final_text, &last_receipts),
		};
		let turn_text: String = turn
			.content
			.iter()
			.filter(|b| b.get("type").and_then(Value::as_str) == Some("text"))
			.filter_map(|b| b.get("text").and_then(Value::as_str))
			.collect();
		if !turn_text.is_empty() {
			final_text = turn_text;
		}
		messages.push(json!({"role": "assistant", "content": turn.content}));
		match turn.stop_reason.as_str() {
			"tool_use" => {
				let mut results: Vec<Value> = Vec::new();
				for block in turn.content.iter().filter(|b| b.get("type").and_then(Value::as_str) == Some("tool_use")) {
					let id = block.get("id").and_then(Value::as_str).unwrap_or("").to_string();
					let name = block.get("name").and_then(Value::as_str).unwrap_or("?").to_string();
					let input = block.get("input").cloned().unwrap_or(Value::Null);
					let (result, is_error) = run_tool(sink, ctx, params, &mut last_receipts, &name, &input).await;
					results.push(json!({
						"type": "tool_result",
						"tool_use_id": id,
						"content": result,
						"is_error": is_error,
					}));
				}
				// All results of one assistant turn go back in ONE user message.
				messages.push(json!({"role": "user", "content": results}));
			}
			// Server-side pause: re-send with the assistant turn appended.
			"pause_turn" => continue,
			other => return fail(other, None, false, &final_text, &last_receipts),
		}
	}
	fail("turn_limit", None, false, &final_text, &last_receipts)
}

/// Dispatch one tool call. Parent-only tools are gated by
/// [`AgentParams::orchestrator`] — a child calling them gets the same honest
/// unknown-tool error as a hallucinated name.
async fn run_tool(
	sink: &Sink,
	ctx: &AgentCtx,
	params: &AgentParams,
	last_receipts: &mut Option<Value>,
	name: &str,
	input: &Value,
) -> (String, bool) {
	match name {
		"run_program" => run_program_tool(sink, ctx, last_receipts, input).await,
		"describe_api" => describe_api_tool(sink, ctx, input).await,
		"task_update" if params.orchestrator => task_update_tool(sink, input).await,
		"spawn_subagents" if params.orchestrator => spawn_subagents_tool(sink, ctx, input).await,
		_ => {
			let available = if params.orchestrator {
				"run_program, describe_api, task_update, spawn_subagents"
			} else {
				"run_program, describe_api"
			};
			(format!("unknown tool '{name}' — available tools: {available}"), true)
		}
	}
}

/// Execute one `run_program` call in the kernel: parent gets the
/// running/done `tool` frames, a child gets one condensed `subagent` frame;
/// both get the viewport `refresh` when meshes were exported. Returns the
/// (truncated) report JSON and `is_error = !report.ok`.
async fn run_program_tool(sink: &Sink, ctx: &AgentCtx, last_receipts: &mut Option<Value>, input: &Value) -> (String, bool) {
	let program = input.get("program").cloned().unwrap_or(Value::Null);
	let op_count = program.get("ops").and_then(Value::as_array).map(Vec::len).unwrap_or(0);
	if sink.child.is_none() {
		sink.event("tool", json!({"state": "running", "name": "run_program", "ops": op_count, "program": program})).await;
	}
	let text = program.to_string();
	let dir = ctx.out_dir.clone();
	let base = ctx.repo_root.clone();
	let report = match tokio::task::spawn_blocking(move || kernel_api::run_program_with_input_base(&text, &dir, &base)).await {
		Ok(r) => r,
		Err(e) => {
			let msg = format!("kernel task failed: {e}");
			if sink.child.is_none() {
				sink.event("tool", json!({"state": "done", "name": "run_program", "ops": op_count, "ok": false, "error": msg})).await;
			} else {
				sink.subagent("tool", Some(format!("run_program {op_count} ops → {msg}"))).await;
			}
			return (msg, true);
		}
	};
	let first_error = report.ops.iter().find_map(|op| op.error.as_ref()).map(|e| format!("{:?}: {}", e.kind, e.message));
	if sink.child.is_none() {
		sink.event(
			"tool",
			json!({"state": "done", "name": "run_program", "ops": report.ops.len(), "ok": report.ok, "error": first_error}),
		)
		.await;
	} else {
		let outcome = match &first_error {
			None => "ok".to_string(),
			Some(e) => e.clone(),
		};
		sink.subagent("tool", Some(format!("run_program {} ops → {outcome}", report.ops.len()))).await;
	}
	let artifacts = artifacts_of(&report, &ctx.out_dir, &ctx.session);
	if !artifacts.is_empty() {
		sink.event("refresh", json!({"artifacts": artifacts, "receipt": receipt_from_report(&report)})).await;
	}
	*last_receipts = Some(receipts_summary(&report));
	let result = serde_json::to_string(&report).unwrap_or_else(|e| format!("{{\"ok\":false,\"serialize_error\":\"{e}\"}}"));
	(truncate_result(result), !report.ok)
}

/// Execute one `describe_api` call: no arg → the live op catalogue; with
/// `op` → existence (from the catalogue) plus the op's API.md section, or an
/// honest "no doc section" note for the known-undocumented ops.
async fn describe_api_tool(sink: &Sink, ctx: &AgentCtx, input: &Value) -> (String, bool) {
	let op = input.get("op").and_then(Value::as_str).map(str::trim).filter(|s| !s.is_empty()).map(str::to_string);
	if sink.child.is_none() {
		sink.event("tool", json!({"state": "running", "name": "describe_api", "op": op})).await;
	}
	let scratch = ctx.out_dir.clone();
	let (count, names) = match tokio::task::spawn_blocking(move || crate::apidoc::op_catalogue(&scratch)).await {
		Ok(Ok(c)) => c,
		Ok(Err(e)) => return describe_fail(sink, &op, e).await,
		Err(e) => return describe_fail(sink, &op, format!("task failed: {e}")).await,
	};
	let result = match &op {
		None => json!({"count": count, "ops": names}),
		Some(op) => {
			let exists = names.iter().any(|n| n == op);
			if !exists {
				json!({
					"op": op,
					"exists": false,
					"note": format!("not one of the {count} ops — call describe_api with no arguments for the full catalogue"),
				})
			} else {
				let section = tokio::fs::read_to_string(ctx.repo_root.join("API.md"))
					.await
					.ok()
					.and_then(|md| crate::apidoc::extract_section(&md, op));
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
	if sink.child.is_none() {
		sink.event("tool", json!({"state": "done", "name": "describe_api", "ok": true, "op": op})).await;
	} else {
		sink.subagent("tool", Some(format!("describe_api {}", op.as_deref().unwrap_or("(catalogue)")))).await;
	}
	(truncate_result(result.to_string()), false)
}

/// Surface a `describe_api` failure on the right channel (parent `tool`
/// frame / child `subagent` frame) and return the tool error.
async fn describe_fail(sink: &Sink, op: &Option<String>, e: String) -> (String, bool) {
	let msg = format!("describe_api failed: {e}");
	if sink.child.is_none() {
		sink.event("tool", json!({"state": "done", "name": "describe_api", "ok": false, "op": op, "error": msg})).await;
	} else {
		sink.subagent("tool", Some(format!("describe_api → {e}"))).await;
	}
	(msg, true)
}

/// Execute one `task_update` call: validate, echo the list to the client as
/// a `tasks` event, store nothing. Parent-only.
async fn task_update_tool(sink: &Sink, input: &Value) -> (String, bool) {
	let Some(tasks) = input.get("tasks").and_then(Value::as_array) else {
		return ("task_update needs {\"tasks\": [{\"content\", \"status\"}]}".to_string(), true);
	};
	for (i, t) in tasks.iter().enumerate() {
		let content_ok = t.get("content").and_then(Value::as_str).is_some_and(|c| !c.trim().is_empty());
		let status_ok = matches!(t.get("status").and_then(Value::as_str), Some("pending" | "in_progress" | "completed"));
		if !content_ok || !status_ok {
			return (
				format!("task #{i}: needs a non-empty \"content\" string and \"status\" of pending|in_progress|completed"),
				true,
			);
		}
	}
	sink.event("tasks", json!({"tasks": tasks})).await;
	("ok".to_string(), false)
}

/// Validate a `spawn_subagents` input into `(name, brief)` pairs. Pure — the
/// tool refuses (is_error) before any network or child work on bad input.
fn validate_spawn_input(input: &Value) -> Result<Vec<(String, String)>, String> {
	let Some(agents) = input.get("agents").and_then(Value::as_array) else {
		return Err("spawn_subagents needs {\"agents\": [{\"name\", \"brief\"}]}".to_string());
	};
	if agents.is_empty() {
		return Err("agents must not be empty — spawn at least one part agent".to_string());
	}
	if agents.len() > MAX_SUBAGENTS {
		return Err(format!("too many agents ({} > {MAX_SUBAGENTS}) — split the fan-out into batches", agents.len()));
	}
	let mut seen = std::collections::HashSet::new();
	let mut out = Vec::with_capacity(agents.len());
	for (i, a) in agents.iter().enumerate() {
		let name = a.get("name").and_then(Value::as_str).map(str::trim).filter(|s| !s.is_empty());
		let brief = a.get("brief").and_then(Value::as_str).map(str::trim).filter(|s| !s.is_empty());
		let (Some(name), Some(brief)) = (name, brief) else {
			return Err(format!("agent #{i}: needs a non-empty \"name\" and a non-empty \"brief\""));
		};
		if !seen.insert(name.to_string()) {
			return Err(format!("duplicate agent name '{name}' — names must be unique"));
		}
		out.push((name.to_string(), brief.to_string()));
	}
	Ok(out)
}

/// Execute one `spawn_subagents` call: validate, run the children
/// concurrently (capped, order-preserving), and hand the parent the JSON
/// array of `{name, ok, receipts, summary}`. `is_error` only when EVERY
/// child failed — partial failure is data, not a tool error.
async fn spawn_subagents_tool(sink: &Sink, ctx: &AgentCtx, input: &Value) -> (String, bool) {
	let agents = match validate_spawn_input(input) {
		Ok(a) => a,
		Err(e) => return (e, true),
	};
	let children = agents.into_iter().map(|(name, brief)| run_child(sink, ctx, name, brief));
	let results: Vec<Value> = futures_util::stream::iter(children).buffered(SUBAGENT_CONCURRENCY).collect().await;
	let all_failed = results.iter().all(|r| r.get("ok") == Some(&Value::Bool(false)));
	(Value::Array(results).to_string(), all_failed)
}

/// Run ONE part agent: fresh message history (part-agent system prompt + the
/// brief as the user turn), child tool set, child turn budget, condensed
/// `subagent` lifecycle frames. Returns its `{name, ok, receipts, summary}`.
async fn run_child(parent: &Sink, ctx: &AgentCtx, name: String, brief: String) -> Value {
	let sink = Sink { tx: parent.tx.clone(), child: Some(name.clone()) };
	let first_line: String = brief.lines().next().unwrap_or("").chars().take(96).collect();
	sink.subagent("started", Some(first_line)).await;
	let params = AgentParams {
		system_prompt: child_system_prompt(&name),
		tools: child_tools(),
		max_turns: child_max_turns(),
		orchestrator: false,
	};
	let messages = vec![json!({"role": "user", "content": brief})];
	// Box the recursive re-entry into the shared loop core (parent loop →
	// spawn tool → child loop) so the future stays finite-sized.
	let outcome = Box::pin(run_agent(&sink, ctx, &params, messages)).await;
	if let Some(e) = &outcome.error {
		sink.subagent("error", Some(e.clone())).await;
		return json!({"name": name, "ok": false, "receipts": outcome.last_receipts, "summary": format!("part agent failed: {e}")});
	}
	if outcome.client_gone {
		return json!({"name": name, "ok": false, "receipts": outcome.last_receipts, "summary": "client disconnected"});
	}
	if outcome.stop_reason == "turn_limit" {
		sink.subagent("error", Some(format!("turn budget exhausted after {} turns", params.max_turns))).await;
		let summary = format!(
			"turn budget exhausted after {} turns without finishing; last message: {}",
			params.max_turns, outcome.final_text
		);
		return json!({"name": name, "ok": false, "receipts": outcome.last_receipts, "summary": summary});
	}
	let ok = outcome.stop_reason == "end_turn";
	let done_line: String = outcome.final_text.lines().next().unwrap_or("").chars().take(96).collect();
	sink.subagent("done", if done_line.is_empty() { None } else { Some(done_line) }).await;
	json!({"name": name, "ok": ok, "receipts": outcome.last_receipts, "summary": outcome.final_text})
}

/// One fully-streamed assistant turn.
struct Turn {
	/// Reassembled content blocks, ready to echo back verbatim (text, thinking
	/// with signature, redacted_thinking, tool_use with parsed input).
	content: Vec<Value>,
	stop_reason: String,
}

/// In-flight content block being accumulated from stream deltas.
#[derive(Default)]
struct BlockAcc {
	start: Value,
	text: String,
	thinking: String,
	signature: String,
	input_json: String,
}

/// POST one streaming Messages request and forward deltas per the sink's
/// policy. `Ok(None)` means the client hung up; `Err` carries an
/// API/transport error.
async fn stream_one_turn(sink: &Sink, client: &reqwest::Client, api_key: &str, body: &Value) -> Result<Option<Turn>, String> {
	let resp = client
		.post(format!("{}/v1/messages", anthropic_base_url()))
		.header("x-api-key", api_key)
		.header("anthropic-version", "2023-06-01")
		.header("content-type", "application/json")
		.json(body)
		.send()
		.await
		.map_err(|e| format!("cannot reach the Anthropic API: {e}"))?;
	if !resp.status().is_success() {
		let status = resp.status();
		let body = resp.text().await.unwrap_or_default();
		let detail = serde_json::from_str::<Value>(&body)
			.ok()
			.and_then(|v| v.pointer("/error/message").and_then(Value::as_str).map(str::to_string))
			.unwrap_or(body);
		return Err(format!("Anthropic API error {status}: {detail}"));
	}

	let mut stream = resp.bytes_stream();
	let mut buffer = String::new();
	let mut blocks: Vec<BlockAcc> = Vec::new();
	let mut stop_reason = String::from("end_turn");
	while let Some(chunk) = stream.next().await {
		let chunk = chunk.map_err(|e| format!("stream error: {e}"))?;
		buffer.push_str(&String::from_utf8_lossy(&chunk));
		while let Some(cut) = buffer.find("\n\n") {
			let frame = buffer[..cut].to_string();
			buffer.drain(..cut + 2);
			let Some(data) = parse_sse_frame(&frame) else { continue };
			match data.get("type").and_then(Value::as_str) {
				Some("content_block_start") => {
					let index = data.get("index").and_then(Value::as_u64).unwrap_or(0) as usize;
					while blocks.len() <= index {
						blocks.push(BlockAcc::default());
					}
					blocks[index].start = data.get("content_block").cloned().unwrap_or(Value::Null);
				}
				Some("content_block_delta") => {
					let index = data.get("index").and_then(Value::as_u64).unwrap_or(0) as usize;
					while blocks.len() <= index {
						blocks.push(BlockAcc::default());
					}
					let acc = &mut blocks[index];
					let delta = data.get("delta").cloned().unwrap_or(Value::Null);
					match delta.get("type").and_then(Value::as_str) {
						Some("text_delta") => {
							let t = delta.get("text").and_then(Value::as_str).unwrap_or("");
							acc.text.push_str(t);
							if !sink.text_delta(t).await {
								return Ok(None);
							}
						}
						Some("thinking_delta") => {
							let t = delta.get("thinking").and_then(Value::as_str).unwrap_or("");
							acc.thinking.push_str(t);
							if !sink.thinking_delta(t).await {
								return Ok(None);
							}
						}
						Some("signature_delta") => {
							acc.signature.push_str(delta.get("signature").and_then(Value::as_str).unwrap_or(""));
						}
						Some("input_json_delta") => {
							acc.input_json.push_str(delta.get("partial_json").and_then(Value::as_str).unwrap_or(""));
						}
						_ => {}
					}
				}
				Some("message_delta") => {
					if let Some(r) = data.pointer("/delta/stop_reason").and_then(Value::as_str) {
						stop_reason = r.to_string();
					}
				}
				Some("message_stop") => break,
				Some("error") => {
					let msg = data.pointer("/error/message").and_then(Value::as_str).unwrap_or("stream error");
					return Err(format!("Anthropic stream error: {msg}"));
				}
				_ => {}
			}
		}
	}

	// Reassemble the turn's content blocks for the follow-up request.
	let content: Vec<Value> = blocks
		.into_iter()
		.filter_map(|acc| {
			let kind = acc.start.get("type").and_then(Value::as_str).unwrap_or("");
			match kind {
				"text" => Some(json!({"type": "text", "text": acc.text})),
				"thinking" => Some(json!({"type": "thinking", "thinking": acc.thinking, "signature": acc.signature})),
				// Complete in the start event; must be echoed back verbatim.
				"redacted_thinking" => Some(acc.start),
				"tool_use" => {
					let input = if acc.input_json.is_empty() {
						acc.start.get("input").cloned().unwrap_or(json!({}))
					} else {
						serde_json::from_str(&acc.input_json).unwrap_or(json!({}))
					};
					Some(json!({
						"type": "tool_use",
						"id": acc.start.get("id").cloned().unwrap_or(Value::Null),
						"name": acc.start.get("name").cloned().unwrap_or(Value::Null),
						"input": input,
					}))
				}
				_ => None,
			}
		})
		.collect();
	Ok(Some(Turn { content, stop_reason }))
}

/// Extract the JSON `data:` payload of one SSE frame (ignores comments and
/// the `event:` line — the payload's own `type` field is authoritative).
fn parse_sse_frame(frame: &str) -> Option<Value> {
	let mut data_lines: Vec<&str> = Vec::new();
	for line in frame.lines() {
		if let Some(rest) = line.strip_prefix("data:") {
			data_lines.push(rest.trim_start());
		}
	}
	if data_lines.is_empty() {
		return None;
	}
	serde_json::from_str(&data_lines.join("\n")).ok()
}

#[cfg(test)]
mod tests {
	use super::*;

	/// A tool context over temp dirs and a dummy key — no test here touches
	/// the network (validation and kernel execution happen before any HTTP).
	fn test_ctx() -> AgentCtx {
		let out_dir = std::env::temp_dir().join(format!("chat_harness_test_{}", std::process::id()));
		std::fs::create_dir_all(&out_dir).unwrap();
		AgentCtx {
			client: reqwest::Client::new(),
			api_key: "test-key-never-sent".to_string(),
			out_dir,
			repo_root: std::env::temp_dir(),
			session: "t-harness".to_string(),
		}
	}

	/// A sink whose receiver stays alive for the test's duration.
	fn test_sink() -> (Sink, mpsc::Receiver<Event>) {
		let (tx, rx) = mpsc::channel(64);
		(Sink { tx, child: None }, rx)
	}

	/// The four tool definitions serialize into the Anthropic `tools` array
	/// shape: name + description + object input_schema with properties, and
	/// every `required` field exists in `properties`. The child set is the
	/// documented two-tool subset.
	#[test]
	fn tool_schemas_are_anthropic_shaped() {
		let parent = parent_tools();
		let child = child_tools();
		let names: Vec<&str> = parent.iter().filter_map(|t| t.get("name").and_then(Value::as_str)).collect();
		let child_names: Vec<&str> = child.iter().filter_map(|t| t.get("name").and_then(Value::as_str)).collect();
		let shaped = parent.iter().all(|t| {
			let desc_ok = t.get("description").and_then(Value::as_str).is_some_and(|d| d.len() > 40);
			let schema_ok = t.pointer("/input_schema/type") == Some(&json!("object"))
				&& t.pointer("/input_schema/properties").is_some_and(Value::is_object);
			let required_ok = match t.pointer("/input_schema/required").and_then(Value::as_array) {
				None => true,
				Some(req) => req.iter().all(|r| {
					r.as_str()
						.is_some_and(|r| t.pointer(&format!("/input_schema/properties/{r}")).is_some())
				}),
			};
			desc_ok && schema_ok && required_ok
		});
		assert!(
			names == ["run_program", "describe_api", "task_update", "spawn_subagents"]
				&& child_names == ["run_program", "describe_api"]
				&& shaped,
			"tool suite must be the documented four (children: two) in Anthropic shape; got parent {names:?}, child {child_names:?}, shaped {shaped}"
		);
	}

	/// Turn budgets parse and clamp to 1..=100, falling back to the default
	/// on garbage — the env override contract (parsed once at runtime).
	#[test]
	fn turn_budgets_parse_and_clamp() {
		let cases = [
			(None, 32, 32),
			(Some("16".to_string()), 32, 16),
			(Some("0".to_string()), 32, 1),
			(Some("999".to_string()), 32, 100),
			(Some("nope".to_string()), 32, 32),
			(Some(" 8 ".to_string()), 16, 8),
		];
		let got: Vec<usize> = cases.iter().map(|(raw, d, _)| clamp_turns(raw.clone(), *d)).collect();
		let want: Vec<usize> = cases.iter().map(|(_, _, w)| *w).collect();
		assert_eq!(got, want, "clamp_turns must clamp to 1..=100 and default on garbage");
	}

	/// spawn_subagents input validation refuses fast — empty list, >8 agents,
	/// missing brief, missing agents key, duplicate names — all as tool-error
	/// strings (is_error=true) with NO network and NO children started.
	#[tokio::test]
	async fn spawn_subagents_input_validation_fails_fast_without_network() {
		let (sink, _rx) = test_sink();
		let ctx = test_ctx();
		let nine: Vec<Value> = (0..9).map(|i| json!({"name": format!("a{i}"), "brief": "spec"})).collect();
		let (m_empty, e_empty) = spawn_subagents_tool(&sink, &ctx, &json!({"agents": []})).await;
		let (m_nine, e_nine) = spawn_subagents_tool(&sink, &ctx, &json!({"agents": nine})).await;
		let (m_brief, e_brief) = spawn_subagents_tool(&sink, &ctx, &json!({"agents": [{"name": "lid"}]})).await;
		let (m_key, e_key) = spawn_subagents_tool(&sink, &ctx, &json!({})).await;
		let (m_dup, e_dup) =
			spawn_subagents_tool(&sink, &ctx, &json!({"agents": [{"name": "lid", "brief": "a"}, {"name": "lid", "brief": "b"}]}))
				.await;
		assert!(
			e_empty && e_nine && e_brief && e_key && e_dup
				&& m_empty.contains("empty")
				&& m_nine.contains("too many agents")
				&& m_brief.contains("agent #0")
				&& m_key.contains("agents")
				&& m_dup.contains("duplicate"),
			"bad spawn inputs must all refuse with is_error=true and a naming message; got: empty=({e_empty},{m_empty}) nine=({e_nine},{m_nine}) brief=({e_brief},{m_brief}) key=({e_key},{m_key}) dup=({e_dup},{m_dup})"
		);
	}

	/// A child (non-orchestrator) calling a parent-only tool gets the honest
	/// unknown-tool error naming ITS available tools.
	#[tokio::test]
	async fn children_cannot_reach_parent_only_tools() {
		let (sink, _rx) = test_sink();
		let ctx = test_ctx();
		let child_params = AgentParams {
			system_prompt: child_system_prompt("t"),
			tools: child_tools(),
			max_turns: 1,
			orchestrator: false,
		};
		let mut receipts = None;
		let (m_spawn, e_spawn) =
			run_tool(&sink, &ctx, &child_params, &mut receipts, "spawn_subagents", &json!({"agents": []})).await;
		let (m_tasks, e_tasks) = run_tool(&sink, &ctx, &child_params, &mut receipts, "task_update", &json!({"tasks": []})).await;
		assert!(
			e_spawn && e_tasks && m_spawn.contains("unknown tool") && m_tasks.contains("run_program, describe_api"),
			"one level of spawning only: children must be refused parent tools; got ({e_spawn},{m_spawn}) ({e_tasks},{m_tasks})"
		);
	}

	/// task_update echoes a valid list as ONE `tasks` SSE event and refuses
	/// malformed entries; nothing is stored server-side (pure echo).
	#[tokio::test]
	async fn task_update_echoes_and_validates() {
		let (sink, mut rx) = test_sink();
		let good = json!({"tasks": [
			{"content": "model the plate", "status": "in_progress"},
			{"content": "drill the pattern", "status": "pending"},
		]});
		let (ok_msg, ok_err) = task_update_tool(&sink, &good).await;
		let frame = rx.try_recv().ok();
		let (bad_msg, bad_err) = task_update_tool(&sink, &json!({"tasks": [{"content": "x", "status": "later"}]})).await;
		let no_extra = rx.try_recv().is_err();
		assert!(
			ok_msg == "ok" && !ok_err && frame.is_some() && bad_err && bad_msg.contains("task #0") && no_extra,
			"task_update must echo valid lists (one tasks event) and refuse bad statuses without emitting; got ok=({ok_err},{ok_msg}), frame={}, bad=({bad_err},{bad_msg}), no_extra={no_extra}",
			frame.is_some()
		);
	}

	/// The run_program executor still truncates oversized reports (a REAL
	/// kernel run — 64 describe ops serialize well past the cap) and the
	/// truncation helper never splits a UTF-8 char.
	#[tokio::test]
	async fn run_program_truncates_oversized_reports() {
		let (sink, _rx) = test_sink();
		let ctx = test_ctx();
		let ops: Vec<Value> = (0..64).map(|i| json!({"id": format!("d{i}"), "op": "describe"})).collect();
		let mut receipts = None;
		let (result, is_error) = run_program_tool(&sink, &ctx, &mut receipts, &json!({"program": {"ops": ops}})).await;
		let multibyte = truncate_result("é".repeat(MAX_TOOL_RESULT)); // 2 bytes/char forces a boundary hit
		assert!(
			!is_error
				&& result.ends_with("…[report truncated]")
				&& result.len() <= MAX_TOOL_RESULT + 32
				&& receipts.as_ref().and_then(|r| r.get("ok")) == Some(&json!(true))
				&& multibyte.ends_with("…[report truncated]"),
			"oversized reports must truncate with the note (len {} vs cap {MAX_TOOL_RESULT}), keep ok receipts, and respect char boundaries; is_error={is_error}",
			result.len()
		);
	}

	/// Subagent receipts keep per-op ok + scalar measures + files and drop
	/// array/object dumps — the contract the parent verifies children against.
	#[test]
	fn receipts_summary_keeps_scalars_and_flags() {
		let program = r#"{"ops": [
			{"id": "b", "op": "box", "min": [0, 0, 0], "max": [10, 10, 10]},
			{"id": "v", "op": "volume", "in": "b"},
			{"id": "cat", "op": "describe"}
		]}"#;
		let report = kernel_api::run_program(program, &std::env::temp_dir());
		let s = receipts_summary(&report);
		let vol = s.pointer("/ops/1/measures/volume").and_then(Value::as_f64).unwrap_or(f64::NAN);
		let describe_ops_dropped = s.pointer("/ops/2/measures/ops").is_none();
		let describe_count_kept = s.pointer("/ops/2/measures/count").is_some();
		assert!(
			s["ok"] == json!(true)
				&& s.pointer("/ops/0/ok") == Some(&json!(true))
				&& (vol - 1000.0).abs() < 1e-9
				&& describe_ops_dropped
				&& describe_count_kept,
			"receipts must keep scalar measures (volume {vol}) and drop list dumps (ops dropped: {describe_ops_dropped}, count kept: {describe_count_kept}); got {s}"
		);
	}
}
