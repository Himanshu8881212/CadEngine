// Copyright (c) LMCAD. Licensed under the MIT License.

//! LMCAD Studio server binary: `cargo run -p studio-server` → <http://localhost:7878>.
//!
//! Environment:
//! - `ANTHROPIC_API_KEY` — enables the `/api/chat` Claude loop (optional; the
//!   rest of the app works without it).
//! - `STUDIO_ADDR` — listen address (default `127.0.0.1:7878`).
//! - `LMCAD_ROOT` — repository root for resolving part paths (default: the
//!   current working directory, which is the workspace root under `cargo run`).

use std::path::PathBuf;
use std::sync::Arc;

use studio_server::{router, AppState};

#[tokio::main]
async fn main() {
	let repo_root = std::env::var("LMCAD_ROOT")
		.map(PathBuf::from)
		.unwrap_or_else(|_| std::env::current_dir().expect("cwd is readable"));
	let out_root = repo_root.join("studio_out");
	let api_key = std::env::var("ANTHROPIC_API_KEY").ok().filter(|k| !k.trim().is_empty());
	let chat = if api_key.is_some() { "enabled" } else { "disabled (set ANTHROPIC_API_KEY)" };
	let web_dist = repo_root.join("studio/web/dist");
	let api_token = std::env::var("CADCODE_API_TOKEN").ok().filter(|t| !t.trim().is_empty());
	let state = Arc::new(AppState { repo_root: repo_root.clone(), out_root, api_key, sessions: Default::default() });
	let app = router(state, &web_dist, api_token.clone());

	let addr = std::env::var("STUDIO_ADDR").unwrap_or_else(|_| "127.0.0.1:7878".to_string());
	let listener = tokio::net::TcpListener::bind(&addr).await.unwrap_or_else(|e| panic!("cannot bind {addr}: {e}"));
	// V6: never serve the API open on a public interface. A non-loopback bind requires a token.
	let loopback = listener.local_addr().map(|a| a.ip().is_loopback()).unwrap_or(true);
	if api_token.is_none() && !loopback {
		panic!("refusing to serve on non-loopback {addr} without CADCODE_API_TOKEN — set a bearer token to expose the API");
	}
	let auth = if api_token.is_some() { "on (bearer token)" } else { "off (loopback dev)" };
	println!("LMCAD Studio: http://{addr}  (root {}, chat {chat}, auth {auth})", repo_root.display());
	axum::serve(listener, app).await.expect("server runs");
}
