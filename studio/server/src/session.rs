// Copyright (c) LMCAD. Licensed under the MIT License.

//! Stateful editing sessions (M4 loop) — the SERVER layer, deliberately.
//!
//! `kernel_api::run_program` is a PURE, stateless batch function and stays that way. A session is
//! inherently stateful (open a document, edit it across SEPARATE calls, read it back), so the state
//! lives here — a process-local in-memory registry of open [`Document`]s the agent edits — never in
//! the kernel. Nothing here is persisted or shared across processes; that is a later concern.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use axum::extract::{Path, State};
use axum::http::StatusCode;
use axum::response::{IntoResponse, Response};
use axum::Json;
use kernel_model::format::load_part;
use kernel_model::Document;
use serde::{Deserialize, Serialize};

use crate::AppState;

/// Process-local session registry. Held in [`AppState`]; kernel-api never sees it.
#[derive(Default)]
pub struct SessionStore {
	inner: Mutex<Inner>,
}
pub type Sessions = Arc<SessionStore>;

#[derive(Default)]
struct Inner {
	sessions: HashMap<String, Session>,
	next: u64,
}
struct Session {
	doc: Document,
	/// A monotonically-increasing edit counter — a call-independent proof that mutation persists.
	edits: u64,
}

#[derive(Serialize)]
struct DimInfo {
	name: String,
	value: f64,
}
#[derive(Serialize)]
struct SessionState {
	session_id: String,
	edits: u64,
	dims: Vec<DimInfo>,
}
fn state_of(id: &str, s: &Session) -> SessionState {
	SessionState {
		session_id: id.to_string(),
		edits: s.edits,
		dims: s.doc.params_iter().map(|(n, v)| DimInfo { name: n.to_string(), value: v }).collect(),
	}
}

/// `POST /api/session/open` — load an inline `.lmcpart` into a new in-memory session.
#[derive(Deserialize)]
pub struct OpenRequest {
	/// The `.lmcpart` document text (same format `load_part` accepts).
	pub part: String,
}
pub async fn open(State(app): State<Arc<AppState>>, Json(req): Json<OpenRequest>) -> Response {
	let (doc, _meta) = match load_part(&req.part) {
		Ok(d) => d,
		Err(e) => return (StatusCode::BAD_REQUEST, format!("not a loadable .lmcpart: {e}")).into_response(),
	};
	let mut inner = app.sessions.inner.lock().expect("session store");
	let id = format!("s{}", inner.next);
	inner.next += 1;
	let sess = Session { doc, edits: 0 };
	let resp = Json(state_of(&id, &sess)).into_response();
	inner.sessions.insert(id, sess);
	resp
}

/// `GET /api/session/{id}` — read the session's current state (proves the doc persisted).
pub async fn read(State(app): State<Arc<AppState>>, Path(id): Path<String>) -> Response {
	let inner = app.sessions.inner.lock().expect("session store");
	match inner.sessions.get(&id) {
		Some(s) => Json(state_of(&id, s)).into_response(),
		None => (StatusCode::NOT_FOUND, "no such session").into_response(),
	}
}

/// `POST /api/session/{id}/set_param` — edit one parameter of the in-memory document.
#[derive(Deserialize)]
pub struct SetParamRequest {
	pub name: String,
	pub value: f64,
}
pub async fn set_param(State(app): State<Arc<AppState>>, Path(id): Path<String>, Json(req): Json<SetParamRequest>) -> Response {
	let mut inner = app.sessions.inner.lock().expect("session store");
	match inner.sessions.get_mut(&id) {
		Some(s) => {
			s.doc.set_param(&req.name, req.value);
			s.edits += 1;
			Json(state_of(&id, s)).into_response()
		}
		None => (StatusCode::NOT_FOUND, "no such session").into_response(),
	}
}

/// `POST /api/session/{id}/close` — discard the session.
pub async fn close(State(app): State<Arc<AppState>>, Path(id): Path<String>) -> Response {
	let mut inner = app.sessions.inner.lock().expect("session store");
	let existed = inner.sessions.remove(&id).is_some();
	Json(serde_json::json!({ "closed": existed })).into_response()
}

#[cfg(test)]
mod tests {
	use super::*;
	use crate::router;
	use axum::body::{to_bytes, Body};
	use axum::http::Request as HttpRequest;
	use std::path::PathBuf;
	use tower::ServiceExt;

	fn app() -> axum::Router {
		let state = Arc::new(AppState {
			repo_root: std::env::temp_dir(),
			out_root: std::env::temp_dir(),
			api_key: None,
			sessions: Default::default(),
			compute_slots: Arc::new(tokio::sync::Semaphore::new(2)),
		});
		router(state, &PathBuf::from("no-web-dist"), None)
	}
	async fn json_of(resp: Response) -> serde_json::Value {
		let bytes = to_bytes(resp.into_body(), 1 << 20).await.unwrap();
		serde_json::from_slice(&bytes).unwrap_or(serde_json::Value::Null)
	}
	fn post(uri: &str, body: serde_json::Value) -> HttpRequest<Body> {
		HttpRequest::builder().method("POST").uri(uri).header("content-type", "application/json").body(Body::from(body.to_string())).unwrap()
	}
	fn get(uri: &str) -> HttpRequest<Body> {
		HttpRequest::builder().uri(uri).body(Body::empty()).unwrap()
	}

	#[tokio::test]
	async fn session_state_persists_across_separate_calls() {
		let app = app();
		// A valid (empty) .lmcpart round-tripped through the real format.
		let part = kernel_model::format::save_part(&Document::new(), "session_test");

		// open → session id, edits 0
		let opened = json_of(app.clone().oneshot(post("/api/session/open", serde_json::json!({ "part": part }))).await.unwrap()).await;
		let id = opened["session_id"].as_str().expect("session_id").to_string();
		assert_eq!(opened["edits"].as_u64(), Some(0), "fresh session starts at 0 edits: {opened}");

		// edit via a SEPARATE call — the mutation must land on the stored doc
		let edited = json_of(app.clone().oneshot(post(&format!("/api/session/{id}/set_param"), serde_json::json!({ "name": "any", "value": 3.0 }))).await.unwrap()).await;
		assert_eq!(edited["edits"].as_u64(), Some(1), "set_param increments the stored edit count: {edited}");

		// read in yet ANOTHER call — the edit persisted (a stateless API could not do this)
		let read = json_of(app.clone().oneshot(get(&format!("/api/session/{id}"))).await.unwrap()).await;
		assert_eq!(read["edits"].as_u64(), Some(1), "the edit persisted across calls: {read}");

		// close, then a read is 404 (the session is gone)
		let closed = json_of(app.clone().oneshot(post(&format!("/api/session/{id}/close"), serde_json::json!({}))).await.unwrap()).await;
		assert_eq!(closed["closed"], serde_json::json!(true), "close reports it removed the session: {closed}");
		let after = app.oneshot(get(&format!("/api/session/{id}"))).await.unwrap();
		assert_eq!(after.status(), StatusCode::NOT_FOUND, "a closed session is gone");
	}
}
