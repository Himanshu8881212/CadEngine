// Copyright (c) LMCAD. Licensed under the MIT License.

//! In-process endpoint tests for the Studio server: the router is exercised
//! through `tower::ServiceExt::oneshot` — no sockets, no network, no
//! subprocesses. Chat is tested ONLY for the graceful no-key path elsewhere;
//! nothing here talks to the Anthropic API.

use std::path::PathBuf;
use std::sync::Arc;

use axum::body::Body;
use axum::http::{Request, StatusCode};
use axum::Router;
use serde_json::{json, Value};
use studio_server::{router, AppState};
use tower::ServiceExt;

/// A router over a fresh temp out-root (so parallel tests never share session
/// dirs) with the repo root pointed at `repo_root`.
fn test_router(repo_root: PathBuf, tag: &str) -> Router {
	let out_root = std::env::temp_dir().join(format!("studio_test_{tag}_{}", std::process::id()));
	let state = Arc::new(AppState { repo_root, out_root, api_key: None, sessions: Default::default() });
	// Web dist intentionally absent in tests: the API fallback text is fine.
	router(state, &PathBuf::from("studio/web/dist-not-built"), None)
}

/// The workspace root (two levels above this crate's manifest).
fn workspace_root() -> PathBuf {
	PathBuf::from(env!("CARGO_MANIFEST_DIR")).parent().unwrap().parent().unwrap().to_path_buf()
}

async fn post_json(app: &Router, uri: &str, body: Value) -> (StatusCode, Value) {
	let req = Request::builder()
		.method("POST")
		.uri(uri)
		.header("content-type", "application/json")
		.body(Body::from(body.to_string()))
		.unwrap();
	let resp = app.clone().oneshot(req).await.unwrap();
	let status = resp.status();
	let bytes = axum::body::to_bytes(resp.into_body(), usize::MAX).await.unwrap();
	let value = serde_json::from_slice(&bytes).unwrap_or(Value::Null);
	(status, value)
}

async fn get_raw(app: &Router, uri: &str) -> (StatusCode, Vec<u8>) {
	let req = Request::builder().method("GET").uri(uri).body(Body::empty()).unwrap();
	let resp = app.clone().oneshot(req).await.unwrap();
	let status = resp.status();
	let bytes = axum::body::to_bytes(resp.into_body(), usize::MAX).await.unwrap();
	(status, bytes.to_vec())
}

async fn get_json(app: &Router, uri: &str) -> (StatusCode, Value) {
	let (status, bytes) = get_raw(app, uri).await;
	(status, serde_json::from_slice(&bytes).unwrap_or(Value::Null))
}

/// `/api/run` executes a real work order through the in-process kernel and the
/// exported STL is then fetchable through `/api/mesh` as a well-formed binary
/// STL (84-byte header + 50 bytes/triangle), all inside one session.
#[tokio::test]
async fn run_endpoint_executes_and_mesh_serves_the_export() {
	let app = test_router(workspace_root(), "run");
	let program = json!({
		"program": {"ops": [
			{"id": "plate", "op": "box", "min": [0, 0, 0], "max": [30, 20, 10]},
			{"id": "v", "op": "volume", "in": "plate"},
			{"id": "stl", "op": "export_stl", "in": "plate", "file": "plate.stl"}
		]},
		"session": "t-run"
	});
	let (status, body) = post_json(&app, "/api/run", program).await;
	let volume = body["report"]["ops"][1]["measures"]["volume"].as_f64().unwrap_or(f64::NAN);
	let artifact = body["artifacts"][0].clone();
	assert!(
		status == StatusCode::OK
			&& body["ok"] == json!(true)
			&& body["report"]["ok"] == json!(true)
			&& (volume - 6000.0).abs() < 1e-6
			&& artifact["file"] == json!("plate.stl")
			&& artifact["kind"] == json!("stl"),
		"run response should carry the kernel report (volume 6000) and the plate.stl artifact; got status {status}, body {body}"
	);

	let (mesh_status, bytes) = get_raw(&app, "/api/mesh/plate.stl?session=t-run").await;
	let triangles = u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize;
	assert!(
		mesh_status == StatusCode::OK && bytes.len() == 84 + 50 * triangles && triangles >= 12,
		"mesh endpoint should serve a well-formed binary STL (got {} bytes, {} triangles, status {mesh_status})",
		bytes.len(),
		triangles
	);
}

/// A failing program still answers 200 with the kernel's structured report
/// (`ok: false`, machine-matchable error kind) — the report IS the contract,
/// HTTP errors are reserved for transport problems.
#[tokio::test]
async fn run_endpoint_surfaces_kernel_failures_as_reports() {
	let app = test_router(workspace_root(), "runfail");
	let (status, body) = post_json(
		&app,
		"/api/run",
		json!({"ops": [{"id": "x", "op": "no_such_op_lol"}]}),
	)
	.await;
	assert!(
		status == StatusCode::OK
			&& body["ok"] == json!(false)
			&& body["report"]["ops"][0]["error"]["kind"] == json!("unknown_op"),
		"kernel failures must surface as structured reports, got status {status}, body {body}"
	);
}

/// A hand-written parametric test part (the DESIGN_GUIDE §3.3 shape, anchored
/// so the bore always overshoots): box 30×20×h centred at z=10 minus a Ø8
/// through-bore ⇒ exact volume (600 − 16π)·h.
fn spacer_part(h: f64) -> Value {
	json!({
		"format": "lmc-part",
		"version": 1,
		"units": "mm",
		"name": "studio-test-spacer",
		"created_with": "studio-server tests",
		"document": {
			"params": {"h": h},
			"features": [
				{"Box": {"center": [{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": 10.0}],
				         "size": [{"Literal": 30.0}, {"Literal": 20.0}, {"Param": "h"}]},
				 "label": "blank"},
				{"Cylinder": {"center": [{"Literal": 0.0}, {"Literal": 0.0}, {"Literal": 10.0}],
				              "radius": {"Literal": 4.0}, "height": {"Literal": 40.0}}},
				{"Boolean": {"op": "Difference", "a": 0, "b": 1}, "label": "through bore"}
			],
			"root": 2,
			"suppressed": []
		}
	})
}

/// Exact volume of [`spacer_part`] at height `h`: (30·20 − π·4²)·h.
fn spacer_volume(h: f64) -> f64 {
	(600.0 - 16.0 * std::f64::consts::PI) * h
}

/// The PARAMS-panel contract end-to-end on a temp repo root: load lists the
/// part's Dims/features and exports a viewport mesh with an exact-volume
/// receipt; set_dim rebuilds at the new value, returns the before/after
/// volumes, and **persists** the edit (a reload sees the new Dim); save
/// round-trips an envelope through validation onto disk.
#[tokio::test]
async fn part_load_set_dim_save_round_trip() {
	let repo = std::env::temp_dir().join(format!("studio_test_repo_{}", std::process::id()));
	std::fs::create_dir_all(&repo).unwrap();
	std::fs::write(repo.join("spacer.lmcpart"), spacer_part(8.0).to_string()).unwrap();
	let app = test_router(repo.clone(), "part");

	// 1. Load: dims + features + exact receipt + artifact on disk.
	let (status, info) = post_json(&app, "/api/part/load", json!({"path": "spacer.lmcpart", "session": "t-part"})).await;
	let vol = info["receipt"]["volume"].as_f64().unwrap_or(f64::NAN);
	assert!(
		status == StatusCode::OK
			&& info["name"] == json!("studio-test-spacer")
			&& info["dims"] == json!([{"name": "h", "value": 8.0}])
			&& info["features"][0]["kind"] == json!("Box")
			&& info["features"][0]["label"] == json!("blank")
			&& info["features"][2]["kind"] == json!("Boolean")
			&& info["receipt"]["volume_source"] == json!("exact")
			&& (vol - spacer_volume(8.0)).abs() < 1e-6,
		"load should list dims/features and hand back the exact-volume receipt (expected {}, got {vol}); body {info}",
		spacer_volume(8.0)
	);

	// 2. set_dim h: 8 → 12 rebuilds, reports both volumes, persists.
	let (status, resp) = post_json(
		&app,
		"/api/part/set_dim",
		json!({"path": "spacer.lmcpart", "dim": "h", "value": 12.0, "session": "t-part"}),
	)
	.await;
	let before = resp["volume_before"].as_f64().unwrap_or(f64::NAN);
	let after = resp["receipt"]["volume"].as_f64().unwrap_or(f64::NAN);
	assert!(
		status == StatusCode::OK
			&& resp["before"] == json!(8.0)
			&& resp["after"] == json!(12.0)
			&& (before - spacer_volume(8.0)).abs() < 1e-6
			&& (after - spacer_volume(12.0)).abs() < 1e-6,
		"set_dim should report exact before/after volumes ({} → {}); got {before} → {after}; body {resp}",
		spacer_volume(8.0),
		spacer_volume(12.0)
	);
	let (_, reloaded) = post_json(&app, "/api/part/load", json!({"path": "spacer.lmcpart", "session": "t-part"})).await;
	assert!(
		reloaded["dims"] == json!([{"name": "h", "value": 12.0}]),
		"set_dim must persist to disk: a reload should see h = 12, got {}",
		reloaded["dims"]
	);

	// 3. Unknown dim is refused, naming what IS available.
	let (status, err) = post_json(
		&app,
		"/api/part/set_dim",
		json!({"path": "spacer.lmcpart", "dim": "nope", "value": 1.0, "session": "t-part"}),
	)
	.await;
	assert!(
		status == StatusCode::BAD_REQUEST && err["error"].as_str().unwrap_or("").contains("available: [h]"),
		"unknown dim must 400 and list the available dims, got {status} {err}"
	);

	// 4. Save: a valid envelope lands canonicalized; an invalid one never touches disk.
	let (status, saved) = post_json(&app, "/api/part/save", json!({"path": "spacer.lmcpart", "envelope": spacer_part(8.0)})).await;
	let (_, after_save) = post_json(&app, "/api/part/load", json!({"path": "spacer.lmcpart", "session": "t-part"})).await;
	let (bad_status, _) = post_json(&app, "/api/part/save", json!({"path": "spacer.lmcpart", "envelope": {"format": "not-a-part"}})).await;
	let (_, still) = post_json(&app, "/api/part/load", json!({"path": "spacer.lmcpart", "session": "t-part"})).await;
	assert!(
		status == StatusCode::OK
			&& saved["ok"] == json!(true)
			&& after_save["dims"] == json!([{"name": "h", "value": 8.0}])
			&& bad_status == StatusCode::BAD_REQUEST
			&& still["dims"] == json!([{"name": "h", "value": 8.0}]),
		"save must round-trip a valid envelope (h back to 8) and refuse an invalid one without touching disk; got {status}/{bad_status}, dims {} then {}",
		after_save["dims"],
		still["dims"]
	);
	std::fs::remove_dir_all(&repo).ok();
}

/// The catalog is schema-sane (unique ops, defaults inside their declared
/// bounds/options) and — the honest part — a family instantiates end-to-end
/// through `/api/run` using nothing but its own schema defaults.
#[tokio::test]
async fn catalog_lists_families_and_defaults_actually_build() {
	let app = test_router(workspace_root(), "catalog");
	let (status, body) = get_json(&app, "/api/catalog").await;
	let families = body["families"].as_array().cloned().unwrap_or_default();
	let mut ops = std::collections::BTreeSet::new();
	let mut schema_ok = true;
	for f in &families {
		schema_ok &= ops.insert(f["op"].as_str().unwrap_or("").to_string());
		for p in f["params"].as_array().cloned().unwrap_or_default() {
			let d = &p["default"];
			schema_ok &= !d.is_null();
			if let Some(options) = p["options"].as_array() {
				schema_ok &= options.contains(d);
			}
			if let (Some(min), Some(max), Some(v)) = (p["min"].as_f64(), p["max"].as_f64(), d.as_f64()) {
				schema_ok &= min <= v && v <= max;
			}
		}
	}
	assert!(
		status == StatusCode::OK && families.len() >= 30 && schema_ok,
		"catalog should list ≥30 schema-sane families (unique ops, defaults within bounds/options); got {} families, schema_ok {schema_ok}",
		families.len()
	);

	// Instantiate spur_gear purely from its schema defaults.
	let gear = families.iter().find(|f| f["op"] == json!("spur_gear")).expect("spur_gear family exists");
	let mut op = serde_json::Map::new();
	op.insert("id".into(), json!("part"));
	op.insert("op".into(), json!("spur_gear"));
	for p in gear["params"].as_array().unwrap() {
		op.insert(p["name"].as_str().unwrap().to_string(), p["default"].clone());
	}
	let program = json!({
		"program": {"ops": [
			Value::Object(op),
			{"id": "v", "op": "exact_volume", "in": "part"},
			{"id": "stl", "op": "export_stl", "in": "part", "file": "catalog_part.stl"}
		]},
		"session": "t-catalog"
	});
	let (run_status, run) = post_json(&app, "/api/run", program).await;
	assert!(
		run_status == StatusCode::OK && run["ok"] == json!(true) && run["artifacts"][0]["file"] == json!("catalog_part.stl"),
		"a catalog family must instantiate from its own defaults through /api/run; got {run_status}, body {run}"
	);
}

/// With no ANTHROPIC_API_KEY configured (`api_key: None` on the state — these
/// tests never read the process environment), `/api/chat` answers with the
/// explicit `chat_disabled` SSE event and a clean `done` — the documented
/// graceful path. The live tool loop needs a key + network and is exercised
/// in SMOKE.md, never here.
#[tokio::test]
async fn chat_without_key_streams_the_disabled_event() {
	let app = test_router(workspace_root(), "chat");
	let req = Request::builder()
		.method("POST")
		.uri("/api/chat")
		.header("content-type", "application/json")
		.body(Body::from(json!({"messages": [{"role": "user", "content": "make me a cube"}]}).to_string()))
		.unwrap();
	let resp = app.clone().oneshot(req).await.unwrap();
	let status = resp.status();
	let content_type = resp.headers().get("content-type").and_then(|v| v.to_str().ok()).unwrap_or("").to_string();
	let body = String::from_utf8(axum::body::to_bytes(resp.into_body(), usize::MAX).await.unwrap().to_vec()).unwrap();
	assert!(
		status == StatusCode::OK
			&& content_type.starts_with("text/event-stream")
			&& body.contains("event: chat_disabled")
			&& body.contains("chat disabled — set ANTHROPIC_API_KEY")
			&& body.contains("event: done"),
		"no-key chat must stream the explicit disabled event then done; got {status} {content_type} body: {body}"
	);
}

/// `/api/mesh` refuses traversal out of the session dir and 404s cleanly on
/// missing artifacts.
#[tokio::test]
async fn mesh_endpoint_rejects_traversal_and_missing_files() {
	let app = test_router(workspace_root(), "mesh");
	let (esc_status, _) = get_raw(&app, "/api/mesh/sub/../../../etc/passwd?session=t-mesh").await;
	let (missing_status, _) = get_raw(&app, "/api/mesh/nope.stl?session=t-mesh").await;
	let (bad_session, _) = get_raw(&app, "/api/mesh/a.stl?session=..%2F..").await;
	assert!(
		esc_status == StatusCode::BAD_REQUEST && missing_status == StatusCode::NOT_FOUND && bad_session == StatusCode::BAD_REQUEST,
		"traversal must 400, missing artifact must 404, bad session must 400; got {esc_status} / {missing_status} / {bad_session}"
	);
}
