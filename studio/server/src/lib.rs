// Copyright (c) LMCAD. Licensed under the MIT License.

//! # studio-server — the LMCAD Studio backend
//!
//! Embeds the LMCAD hybrid kernel **in-process** (no subprocess, no IPC) behind a
//! small HTTP API, and statically serves the Studio front-end. One binary is the
//! whole dev loop: `cargo run -p studio-server` → <http://localhost:7878>.
//!
//! Endpoints (all JSON unless noted):
//!
//! | route | verb | what |
//! |---|---|---|
//! | `/api/run` | POST | execute a work-order JSON program via [`kernel_api::run_program_with_input_base`]; returns the full kernel [`kernel_api::Report`] plus the exported mesh artifacts |
//! | `/api/mesh/{*path}` | GET | serve an exported binary (STL/STEP/3MF) from the per-session out-dir |
//! | `/api/part/load` | POST | read + parse a `.lmcpart`, list Dims/features/configs, export a viewport mesh |
//! | `/api/part/save` | POST | validate + write a `.lmcpart` envelope back to disk (canonical bytes) |
//! | `/api/part/set_dim` | POST | load → `set_param` → rebuild → save → re-export mesh → receipt (powers the PARAMS panel) |
//! | `/api/catalog` | GET | the standard-parts families (names + param schemas) behind the PARTS button |
//! | `/api/chat` | POST | SSE: the CAD agent harness — Anthropic Messages API proxy with `run_program` / `describe_api` / `task_update` / `spawn_subagents` wired in-process (see [`chat`]) |
//!
//! Every kernel call runs on a blocking thread ([`tokio::task::spawn_blocking`])
//! so a long boolean never stalls the HTTP runtime, and every geometry answer is
//! the kernel's own receipt (report / route / measures) — the server adds
//! transport, never geometry claims.

use std::path::{Path, PathBuf};
use std::sync::Arc;

use axum::extract::{DefaultBodyLimit, Request};
use axum::http::{header, StatusCode};
use axum::middleware::{self, Next};
use axum::response::{IntoResponse, Response};
use axum::routing::{get, post};
use axum::Router;
use std::time::Duration;
use tokio::sync::Semaphore;
use tower::limit::GlobalConcurrencyLimitLayer;
use tower_http::services::{ServeDir, ServeFile};
use tower_http::timeout::TimeoutLayer;

pub mod apidoc;
pub mod catalog;
pub mod chat;
pub mod part;
pub mod run;
pub mod session;

/// Shared server state: where inputs resolve, where outputs land, and the
/// (optional) Anthropic key for the chat loop.
pub struct AppState {
	/// Repository root. Relative input paths — `load_part` files inside work
	/// orders, `/api/part/*` paths — resolve against this directory.
	pub repo_root: PathBuf,
	/// Root of the per-session output directories (`<out_root>/<session>/`).
	/// Exported meshes are written and served from here.
	pub out_root: PathBuf,
	/// Anthropic API key captured at startup (`ANTHROPIC_API_KEY`). `None`
	/// disables `/api/chat` with an explicit SSE event; everything else works.
	pub api_key: Option<String>,
	/// In-memory stateful-editing sessions (M4 loop). Lives HERE, not in kernel-api,
	/// which stays a pure batch function. Process-local; not persisted.
	pub sessions: session::Sessions,
	/// Dedicated permits held by geometry tasks until the blocking computation
	/// actually exits. HTTP timeout must not release admission while an orphaned
	/// `spawn_blocking` task is still consuming CPU.
	pub compute_slots: Arc<Semaphore>,
}

impl AppState {
	/// Run synchronous kernel work while retaining a compute permit for the whole
	/// task lifetime, including after an HTTP request future times out.
	pub async fn spawn_compute<F, T>(&self, work: F) -> Result<T, tokio::task::JoinError>
	where
		F: FnOnce() -> T + Send + 'static,
		T: Send + 'static,
	{
		let permit = self.compute_slots.clone().acquire_owned().await.expect("compute semaphore is never closed");
		tokio::task::spawn_blocking(move || {
			let _permit = permit;
			work()
		})
		.await
	}
	/// Resolve (and create) the output directory for `session`. Session names
	/// are confined to `[A-Za-z0-9_-]{1,64}` so a session id can never escape
	/// `out_root`; anything else is rejected.
	pub fn session_dir(&self, session: Option<&str>) -> Result<PathBuf, String> {
		let name = session.unwrap_or("default");
		let ok = !name.is_empty()
			&& name.len() <= 64
			&& name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-');
		if !ok {
			return Err(format!("invalid session name '{name}': use [A-Za-z0-9_-], at most 64 chars"));
		}
		std::fs::create_dir_all(&self.out_root)
			.map_err(|e| format!("cannot create output root '{}': {e}", self.out_root.display()))?;
		let dir = confine(&self.out_root, name)?;
		std::fs::create_dir_all(&dir).map_err(|e| format!("cannot create session dir '{}': {e}", dir.display()))?;
		// Re-check after creation so a pre-existing session symlink is never used.
		confine(&self.out_root, name)
	}

	/// Resolve a repo-relative file path, refusing absolute paths and any `..`
	/// component so API callers stay inside the repository root.
	pub fn repo_file(&self, rel: &str) -> Result<PathBuf, String> {
		confine(&self.repo_root, rel)
	}
}

/// Join `rel` onto `base`, rejecting absolute paths and parent-dir escapes.
pub(crate) fn confine(base: &Path, rel: &str) -> Result<PathBuf, String> {
	let p = Path::new(rel);
	if p.is_absolute() {
		return Err(format!("absolute paths are not allowed: '{rel}'"));
	}
	let escapes = p
		.components()
		.any(|c| matches!(c, std::path::Component::ParentDir | std::path::Component::Prefix(_)));
	if escapes {
		return Err(format!("path may not contain '..': '{rel}'"));
	}
	let canonical_base = std::fs::canonicalize(base)
		.map_err(|e| format!("cannot canonicalize sandbox '{}': {e}", base.display()))?;
	let mut current = canonical_base.clone();
	for component in p.components() {
		if let std::path::Component::Normal(name) = component {
			current.push(name);
			match std::fs::symlink_metadata(&current) {
				Ok(meta) if meta.file_type().is_symlink() => {
					return Err(format!("path crosses a symbolic link at '{}': '{rel}'", current.display()));
				}
				Ok(_) => {}
				Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
				Err(e) => return Err(format!("cannot inspect '{}': {e}", current.display())),
			}
		}
	}
	Ok(canonical_base.join(p))
}

/// Build the full Studio router over `state`, serving the API plus the built
/// front-end from `web_dist` (when it exists; otherwise `/` answers with a
/// plain-text pointer to the build command so the API remains usable).
pub fn router(state: Arc<AppState>, web_dist: &Path, auth_token: Option<String>) -> Router {
	// V6 hardening (env-tunable, sane defaults): a per-request deadline generous enough for
	// heavy legit jobs but far below a runaway, a global concurrency cap sized to cores, a
	// request body cap.
	let timeout_secs: u64 = std::env::var("CADCODE_REQUEST_TIMEOUT_SECS").ok().and_then(|s| s.parse().ok()).unwrap_or(300);
	let concurrency = std::thread::available_parallelism().map(|n| n.get()).unwrap_or(4).max(2);
	const MAX_BODY: usize = 16 * 1024 * 1024; // 16 MB

	let api = Router::new()
		.route("/api/run", post(run::run_endpoint))
		.route("/api/mesh/{*path}", get(run::mesh_endpoint))
		.route("/api/part/load", post(part::load_endpoint))
		.route("/api/part/save", post(part::save_endpoint))
		.route("/api/part/set_dim", post(part::set_dim_endpoint))
		.route("/api/catalog", get(catalog::catalog_endpoint))
		.route("/api/chat", post(chat::chat_endpoint))
		.route("/api/session/open", post(session::open))
		.route("/api/session/{id}", get(session::read))
		.route("/api/session/{id}/set_param", post(session::set_param))
		.route("/api/session/{id}/close", post(session::close))
		.with_state(state)
		// Bearer-token auth on the /api routes ONLY (route_layer excludes the static front-end).
		.route_layer(middleware::from_fn(move |req: Request, next: Next| {
			let expected = auth_token.clone();
			async move { require_auth(expected, req, next).await }
		}));

	let index = web_dist.join("index.html");
	let app = if index.exists() {
		api.fallback_service(ServeDir::new(web_dist).not_found_service(ServeFile::new(index)))
	} else {
		api.fallback(|| async {
			"LMCAD Studio API is up. Front-end not built yet: cd studio/web && npm ci && npm run build, then restart."
		})
	};
	// The per-request timeout is ALSO the hard backstop for a runaway boolean (V4's
	// coincident-fit hang): the op runs via spawn_blocking, so at the deadline the request
	// future is dropped and returns 408. AppState's compute semaphore stays inside the
	// blocking closure, so detached work still consumes a slot until it actually exits.
	app.layer(TimeoutLayer::with_status_code(StatusCode::REQUEST_TIMEOUT, Duration::from_secs(timeout_secs)))
		.layer(GlobalConcurrencyLimitLayer::new(concurrency))
		.layer(DefaultBodyLimit::max(MAX_BODY))
}

/// Bearer-token gate for the `/api` routes. With no token configured (local dev — the binary
/// refuses a non-loopback bind without one), requests pass. Otherwise an
/// `Authorization: Bearer <token>` header must match exactly, else `401`.
async fn require_auth(expected: Option<String>, req: Request, next: Next) -> Response {
	match expected {
		None => next.run(req).await,
		Some(token) => {
			let presented = req
				.headers()
				.get(header::AUTHORIZATION)
				.and_then(|v| v.to_str().ok())
				.and_then(|s| s.strip_prefix("Bearer "));
			if presented.is_some_and(|value| constant_time_eq(value.as_bytes(), token.as_bytes())) {
				next.run(req).await
			} else {
				(StatusCode::UNAUTHORIZED, "missing or invalid bearer token").into_response()
			}
		}
	}
}

/// Compare secret bytes without a data-dependent early return. Length is part
/// of the accumulated difference, so differently sized values never match.
fn constant_time_eq(a: &[u8], b: &[u8]) -> bool {
	let mut diff = a.len() ^ b.len();
	for i in 0..a.len().max(b.len()) {
		let av = a.get(i).copied().unwrap_or(0);
		let bv = b.get(i).copied().unwrap_or(0);
		diff |= usize::from(av ^ bv);
	}
	diff == 0
}

#[cfg(test)]
mod tests {
	use super::*;
	use axum::body::Body;
	use axum::http::Request as HttpRequest;
	use tower::ServiceExt; // oneshot

	fn test_app(token: Option<String>) -> Router {
		let state = Arc::new(AppState {
			repo_root: std::env::temp_dir(), out_root: std::env::temp_dir(), api_key: None,
			sessions: Default::default(), compute_slots: Arc::new(Semaphore::new(2)),
		});
		router(state, Path::new("/no-such-web-dist"), token)
	}
	fn get_catalog(auth: Option<&str>) -> HttpRequest<Body> {
		let mut b = HttpRequest::builder().uri("/api/catalog");
		if let Some(a) = auth {
			b = b.header("Authorization", a);
		}
		b.body(Body::empty()).unwrap()
	}

	#[tokio::test]
	async fn api_requires_bearer_token_when_configured() {
		let app = test_app(Some("secret".to_string()));
		let no = app.clone().oneshot(get_catalog(None)).await.unwrap();
		assert_eq!(no.status(), StatusCode::UNAUTHORIZED, "no token → 401");
		let wrong = app.clone().oneshot(get_catalog(Some("Bearer nope"))).await.unwrap();
		assert_eq!(wrong.status(), StatusCode::UNAUTHORIZED, "wrong token → 401");
		let ok = app.oneshot(get_catalog(Some("Bearer secret"))).await.unwrap();
		assert_ne!(ok.status(), StatusCode::UNAUTHORIZED, "correct token → not 401 (got {})", ok.status());
	}

	#[tokio::test]
	async fn no_token_configured_allows_loopback_dev() {
		let app = test_app(None);
		let resp = app.oneshot(get_catalog(None)).await.unwrap();
		assert_ne!(resp.status(), StatusCode::UNAUTHORIZED, "no token configured → open for loopback dev");
	}

	#[tokio::test]
	async fn oversized_request_body_is_rejected() {
		let app = test_app(None);
		let big = vec![b'x'; 20 * 1024 * 1024]; // 20 MB > the 16 MB cap
		let req = HttpRequest::builder()
			.method("POST")
			.uri("/api/run")
			.header("content-type", "application/json")
			.body(Body::from(big))
			.unwrap();
		let resp = app.oneshot(req).await.unwrap();
		assert_eq!(resp.status(), StatusCode::PAYLOAD_TOO_LARGE, "oversized body → 413 (got {})", resp.status());
	}

	#[tokio::test]
	async fn timed_out_compute_keeps_its_slot_until_the_worker_really_exits() {
		let slots = Arc::new(Semaphore::new(1));
		let state = AppState {
			repo_root: std::env::temp_dir(), out_root: std::env::temp_dir(), api_key: None,
			sessions: Default::default(), compute_slots: slots.clone(),
		};
		let timed = tokio::time::timeout(
			Duration::from_millis(10),
			state.spawn_compute(|| std::thread::sleep(Duration::from_millis(120))),
		).await;
		assert!(timed.is_err(), "the simulated HTTP deadline must elapse first");
		assert_eq!(slots.available_permits(), 0,
			"dropping the HTTP future must not admit another CPU task while the orphan still runs");
		tokio::time::sleep(Duration::from_millis(160)).await;
		assert_eq!(slots.available_permits(), 1, "the real worker exit releases its slot");
	}
}
