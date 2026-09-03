// Copyright (c) LMCAD. Licensed under the MIT License.

//! `/api/run` + `/api/mesh` — execute work orders in-process and serve the
//! meshes they export.

use std::sync::Arc;

use axum::extract::{Path as UrlPath, Query, State};
use axum::http::{header, StatusCode};
use axum::response::{IntoResponse, Response};
use axum::Json;
use kernel_api::Report;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

use crate::AppState;

/// One exported file of a run, addressable through `/api/mesh`.
#[derive(Clone, Debug, Serialize)]
pub struct Artifact {
	/// Path relative to the session out-dir (what `/api/mesh/{path}` takes).
	pub file: String,
	/// Ready-to-fetch URL (`/api/mesh/<file>?session=<session>`).
	pub url: String,
	/// Lower-case file extension (`stl` / `step` / `3mf` / …).
	pub kind: String,
}

/// Response of `/api/run`: the kernel's own [`Report`] (the only geometry
/// contract) plus the exported artifacts found in it.
#[derive(Serialize)]
pub struct RunResponse {
	/// Mirror of `report.ok` for quick checks.
	pub ok: bool,
	/// The session the run executed in.
	pub session: String,
	/// The full kernel report, verbatim.
	pub report: Report,
	/// Every file the report says was written, with serve URLs.
	pub artifacts: Vec<Artifact>,
}

/// Collect the artifacts a finished report claims were written, as paths
/// relative to `out_dir` (skipping anything that doesn't exist on disk).
pub fn artifacts_of(report: &Report, out_dir: &std::path::Path, session: &str) -> Vec<Artifact> {
	let mut out = Vec::new();
	for op in &report.ops {
		let Some(file) = &op.file else { continue };
		let full = std::path::Path::new(file);
		if !full.exists() {
			continue;
		}
		let rel = full.strip_prefix(out_dir).unwrap_or(full);
		let rel = rel.to_string_lossy().replace('\\', "/");
		let kind = full.extension().and_then(|e| e.to_str()).unwrap_or("").to_ascii_lowercase();
		out.push(Artifact { url: format!("/api/mesh/{rel}?session={session}"), file: rel, kind });
	}
	out
}

/// POST `/api/run` — execute a work-order JSON program.
///
/// Body: either a bare program `{"ops": [...]}` or a wrapper
/// `{"program": {"ops": [...]}, "session": "name"}`. The program executes via
/// [`kernel_api::run_program_with_input_base`] with exports landing in the
/// session out-dir and relative *input* paths (`load_part`) resolving against
/// the repository root, so `crates/kernel-model/tests/fixtures/pre_w6_parts/x.lmcpart` just works.
pub async fn run_endpoint(State(state): State<Arc<AppState>>, Json(body): Json<Value>) -> Response {
	let (program, session) = if body.get("ops").is_some() {
		(body.clone(), None)
	} else {
		let session = body.get("session").and_then(Value::as_str).map(str::to_string);
		match body.get("program") {
			Some(p) => (p.clone(), session),
			None => {
				return bad_request("body must be a work order {\"ops\": [...]} or {\"program\": {\"ops\": [...]}, \"session\": ...}")
			}
		}
	};
	let session = session.unwrap_or_else(|| "default".to_string());
	let out_dir = match state.session_dir(Some(&session)) {
		Ok(d) => d,
		Err(e) => return bad_request(&e),
	};
	let program_text = program.to_string();
	let input_base = state.repo_root.clone();
	let run_dir = out_dir.clone();
	// The kernel is synchronous and can take seconds on heavy booleans — keep it
	// off the async runtime. Panics inside ops are already caught by the kernel
	// and surfaced as ErrorKind::Internal in the report.
	let report = match state
		.spawn_compute(move || kernel_api::run_program_with_input_base(&program_text, &run_dir, &input_base))
		.await
	{
		Ok(r) => r,
		Err(e) => return server_error(&format!("run task failed: {e}")),
	};
	let artifacts = artifacts_of(&report, &out_dir, &session);
	Json(RunResponse { ok: report.ok, session, report, artifacts }).into_response()
}

/// Query of `/api/mesh/{*path}`: which session's out-dir to serve from.
#[derive(Deserialize)]
pub struct MeshQuery {
	/// Session name (defaults to `default`).
	pub session: Option<String>,
}

/// GET `/api/mesh/{*path}` — serve an exported binary from the session
/// out-dir (three.js `STLLoader` consumes the STLs directly). Path traversal
/// is rejected; a missing file is a plain 404.
pub async fn mesh_endpoint(
	State(state): State<Arc<AppState>>,
	UrlPath(path): UrlPath<String>,
	Query(q): Query<MeshQuery>,
) -> Response {
	let out_dir = match state.session_dir(q.session.as_deref()) {
		Ok(d) => d,
		Err(e) => return bad_request(&e),
	};
	let full = match crate::confine(&out_dir, &path) {
		Ok(p) => p,
		Err(e) => return bad_request(&e),
	};
	let bytes = match tokio::fs::read(&full).await {
		Ok(b) => b,
		Err(_) => return (StatusCode::NOT_FOUND, format!("no artifact '{path}' in session out-dir")).into_response(),
	};
	let mime = match full.extension().and_then(|e| e.to_str()).map(str::to_ascii_lowercase).as_deref() {
		Some("stl") => "model/stl",
		Some("step") | Some("stp") => "application/step",
		Some("3mf") => "model/3mf",
		_ => "application/octet-stream",
	};
	([(header::CONTENT_TYPE, mime)], bytes).into_response()
}

/// 400 with a JSON `{ "error": ... }` body.
pub(crate) fn bad_request(message: &str) -> Response {
	(StatusCode::BAD_REQUEST, Json(json!({ "error": message }))).into_response()
}

/// 500 with a JSON `{ "error": ... }` body.
pub(crate) fn server_error(message: &str) -> Response {
	(StatusCode::INTERNAL_SERVER_ERROR, Json(json!({ "error": message }))).into_response()
}
