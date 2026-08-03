// Copyright (c) LMCAD. Licensed under the MIT License.

//! `/api/part/*` — `.lmcpart` recipes over HTTP: load (Dims + features +
//! configs + a viewport mesh), save (validated round-trip), and `set_dim`
//! (the PARAMS panel: edit one Dim → rebuild → save → fresh mesh + receipt).

use std::path::Path;
use std::sync::Arc;

use axum::extract::State;
use axum::response::{IntoResponse, Response};
use axum::Json;
use kernel_model::format::{load_part, save_part_with_meta, PartBomMeta};
use kernel_model::{Document, MeshRoute};
use serde::{Deserialize, Serialize};
use serde_json::Value;

use crate::run::{bad_request, server_error, Artifact};
use crate::AppState;

/// Viewport-mesh chord tolerance (mm). Coarser than the CLI export default
/// (0.01) on purpose — these meshes are for the 3D view, not manufacturing;
/// the receipt still names the route taken.
const VIEW_TOL: f64 = 0.05;

/// One named parameter (a "Dim") of a loaded part.
#[derive(Serialize)]
pub struct DimInfo {
	/// Parameter name as it appears in the document.
	pub name: String,
	/// Current base value (mm or unitless, per the recipe's own convention).
	pub value: f64,
}

/// One feature of a loaded part's history.
#[derive(Serialize)]
pub struct FeatureInfo {
	/// Position in the feature history (the `FeatureId`).
	pub index: usize,
	/// Feature kind (the `Feature` enum variant name, e.g. `Box`, `Boolean`).
	pub kind: String,
	/// Human label, if the recipe carries one.
	#[serde(skip_serializing_if = "Option::is_none")]
	pub label: Option<String>,
	/// Whether the feature is currently suppressed.
	pub suppressed: bool,
}

/// The rebuild receipt attached to every part response: what the kernel
/// measured, never a server-side estimate.
#[derive(Serialize)]
pub struct MeshReceipt {
	/// Volume in mm³ (see `volume_source` for how it was measured).
	pub volume: f64,
	/// `"exact"` — analytic `exact_volume` on the evaluated B-rep; `"mesh"` —
	/// divergence-theorem volume of the exported mesh (voxel-half documents).
	pub volume_source: String,
	/// Mesh route taken: `"exact"` or `"voxel_healed"`.
	pub route: String,
	/// The kernel's stated reason for the route.
	pub why: String,
	/// Triangle count of the exported mesh.
	pub tris: usize,
	/// Whether the exported mesh is watertight.
	pub watertight: bool,
	/// The exported viewport mesh.
	pub artifact: Artifact,
}

/// Response of `/api/part/load`.
#[derive(Serialize)]
pub struct PartInfo {
	/// Repo-relative path the part was loaded from.
	pub path: String,
	/// Envelope `name`.
	pub name: String,
	/// Envelope `units` (always `"mm"` in v1).
	pub units: String,
	/// Envelope `created_with`.
	pub created_with: String,
	/// BOM v2 `meta` block, when present.
	#[serde(skip_serializing_if = "Option::is_none")]
	pub meta: Option<PartBomMeta>,
	/// Named parameters, sorted by name (the PARAMS panel rows).
	pub dims: Vec<DimInfo>,
	/// Feature history: kind + label per feature.
	pub features: Vec<FeatureInfo>,
	/// Named configurations → their parameter overrides.
	pub configs: Value,
	/// The active configuration, if any.
	#[serde(skip_serializing_if = "Option::is_none")]
	pub active_config: Option<String>,
	/// The raw `.lmcpart` envelope text (the code pane shows this).
	pub envelope: String,
	/// Rebuild receipt + viewport mesh.
	pub receipt: MeshReceipt,
}

fn dims_of(doc: &Document) -> Vec<DimInfo> {
	let mut dims: Vec<DimInfo> = doc.params_iter().map(|(name, value)| DimInfo { name: name.to_string(), value }).collect();
	dims.sort_by(|a, b| a.name.cmp(&b.name));
	dims
}

/// Feature kinds + labels via the document's own serde form (the persisted
/// schema: externally-tagged `Feature` flattened with optional `label`/`notes`),
/// so this never drifts from what `.lmcpart` files actually contain.
fn features_of(doc: &Document) -> Vec<FeatureInfo> {
	let value = serde_json::to_value(doc).unwrap_or(Value::Null);
	let mut out = Vec::new();
	if let Some(features) = value.get("features").and_then(Value::as_array) {
		for (index, record) in features.iter().enumerate() {
			let kind = record
				.as_object()
				.and_then(|o| o.keys().find(|k| k.as_str() != "label" && k.as_str() != "notes"))
				.cloned()
				.unwrap_or_else(|| "?".to_string());
			let label = record.get("label").and_then(Value::as_str).map(str::to_string);
			out.push(FeatureInfo { index, kind, label, suppressed: doc.is_suppressed(kernel_model::FeatureId(index)) });
		}
	}
	out
}

fn configs_of(doc: &Document) -> Value {
	serde_json::to_value(doc).ok().and_then(|v| v.get("configs").cloned()).unwrap_or(Value::Object(Default::default()))
}

/// Evaluate `doc`, export its viewport mesh as `<stem>.stl` into `out_dir`,
/// and assemble the honest receipt (exact volume when the B-rep evaluates,
/// mesh volume otherwise; route + reason from the kernel's `RouteReport`).
fn rebuild_and_export(doc: &Document, stem: &str, out_dir: &Path, session: &str) -> Result<MeshReceipt, String> {
	let exact = doc.evaluate_brep().map(|solid| kernel_brep::exact_volume(&solid));
	let (mesh, route) = doc.export_mesh(VIEW_TOL);
	if mesh.triangle_count() == 0 {
		return Err("document evaluated to empty geometry (no features, or the root feature failed)".to_string());
	}
	let file = format!("{stem}.stl");
	let full = out_dir.join(&file);
	mesh.write_stl_binary(&full).map_err(|e| format!("cannot write '{}': {e}", full.display()))?;
	let (volume, volume_source) = match exact {
		Some(v) => (v, "exact".to_string()),
		None => (mesh.signed_volume(), "mesh".to_string()),
	};
	Ok(MeshReceipt {
		volume,
		volume_source,
		route: match route.route {
			MeshRoute::Exact => "exact".to_string(),
			MeshRoute::Healed => "voxel_healed".to_string(),
		},
		why: route.why,
		tris: route.tris,
		watertight: route.watertight,
		artifact: Artifact {
			url: format!("/api/mesh/{file}?session={session}"),
			file,
			kind: "stl".to_string(),
		},
	})
}

/// A safe artifact stem from a part path/name (alphanumeric + `_-`).
fn stem_of(path: &str) -> String {
	let base = Path::new(path).file_stem().and_then(|s| s.to_str()).unwrap_or("part");
	let cleaned: String = base.chars().map(|c| if c.is_ascii_alphanumeric() || c == '-' || c == '_' { c } else { '_' }).collect();
	if cleaned.is_empty() {
		"part".to_string()
	} else {
		cleaned
	}
}

/// Request of `/api/part/load`.
#[derive(Deserialize)]
pub struct LoadRequest {
	/// Repo-relative `.lmcpart` path (e.g. `gearbox/parts/spacer_21.lmcpart`).
	pub path: String,
	/// Session whose out-dir receives the viewport mesh (default `default`).
	pub session: Option<String>,
}

/// POST `/api/part/load` — read + parse a `.lmcpart`, list its Dims, features
/// and configs, and export a viewport mesh with the rebuild receipt.
pub async fn load_endpoint(State(state): State<Arc<AppState>>, Json(req): Json<LoadRequest>) -> Response {
	let session = req.session.clone().unwrap_or_else(|| "default".to_string());
	let out_dir = match state.session_dir(Some(&session)) {
		Ok(d) => d,
		Err(e) => return bad_request(&e),
	};
	let full = match state.repo_file(&req.path) {
		Ok(p) => p,
		Err(e) => return bad_request(&e),
	};
	let text = match std::fs::read_to_string(&full) {
		Ok(t) => t,
		Err(e) => return bad_request(&format!("cannot read '{}': {e}", req.path)),
	};
	let path = req.path.clone();
	let result = tokio::task::spawn_blocking(move || -> Result<PartInfo, String> {
		let (doc, meta) = load_part(&text).map_err(|e| format!("'{path}' is not a loadable .lmcpart: {e}"))?;
		let receipt = rebuild_and_export(&doc, &stem_of(&path), &out_dir, &session)?;
		Ok(PartInfo {
			path,
			name: meta.name,
			units: meta.units,
			created_with: meta.created_with,
			meta: meta.meta,
			dims: dims_of(&doc),
			features: features_of(&doc),
			configs: configs_of(&doc),
			active_config: doc.active_config().map(str::to_string),
			envelope: text,
			receipt,
		})
	})
	.await;
	match result {
		Ok(Ok(info)) => Json(info).into_response(),
		Ok(Err(e)) => bad_request(&e),
		Err(e) => server_error(&format!("load task failed: {e}")),
	}
}

/// Request of `/api/part/save`.
#[derive(Deserialize)]
pub struct SaveRequest {
	/// Repo-relative `.lmcpart` path to write.
	pub path: String,
	/// The envelope to save: a JSON object or a string of JSON.
	pub envelope: Value,
}

/// Response of `/api/part/save`.
#[derive(Serialize)]
pub struct SaveResponse {
	/// True (errors use HTTP status + `{"error"}` instead).
	pub ok: bool,
	/// The path written.
	pub path: String,
	/// Bytes written.
	pub bytes: usize,
	/// The saved part's envelope name.
	pub name: String,
	/// Number of Dims in the saved document.
	pub dims: usize,
}

/// POST `/api/part/save` — validate an `.lmcpart` envelope (full
/// `load_part` round-trip) and write it back **canonicalized** through
/// [`save_part_with_meta`], so files on disk are always the byte-stable form
/// regardless of the editor's whitespace. Invalid envelopes never touch disk.
pub async fn save_endpoint(State(state): State<Arc<AppState>>, Json(req): Json<SaveRequest>) -> Response {
	let full = match state.repo_file(&req.path) {
		Ok(p) => p,
		Err(e) => return bad_request(&e),
	};
	if full.extension().and_then(|e| e.to_str()) != Some("lmcpart") {
		return bad_request("save path must end in .lmcpart");
	}
	let text = match &req.envelope {
		Value::String(s) => s.clone(),
		other => other.to_string(),
	};
	let (doc, meta) = match load_part(&text) {
		Ok(p) => p,
		Err(e) => return bad_request(&format!("envelope is not a valid .lmcpart (nothing written): {e}")),
	};
	let canonical = save_part_with_meta(&doc, &meta.name, meta.meta.as_ref());
	if let Some(parent) = full.parent() {
		if let Err(e) = std::fs::create_dir_all(parent) {
			return server_error(&format!("cannot create '{}': {e}", parent.display()));
		}
	}
	match std::fs::write(&full, &canonical) {
		Ok(()) => Json(SaveResponse {
			ok: true,
			path: req.path,
			bytes: canonical.len(),
			name: meta.name,
			dims: doc.params_iter().count(),
		})
		.into_response(),
		Err(e) => server_error(&format!("cannot write '{}': {e}", full.display())),
	}
}

/// Request of `/api/part/set_dim`.
#[derive(Deserialize)]
pub struct SetDimRequest {
	/// Repo-relative `.lmcpart` path.
	pub path: String,
	/// The Dim (named parameter) to change. Must already exist in the part —
	/// this endpoint edits recipes, it does not grow them.
	pub dim: String,
	/// New value.
	pub value: f64,
	/// Session whose out-dir receives the refreshed mesh.
	pub session: Option<String>,
}

/// Response of `/api/part/set_dim`: the before/after receipt.
#[derive(Serialize)]
pub struct SetDimResponse {
	/// True (errors use HTTP status + `{"error"}` instead).
	pub ok: bool,
	/// The edited Dim.
	pub dim: String,
	/// Value before the edit.
	pub before: f64,
	/// Value after the edit.
	pub after: f64,
	/// Volume before the edit (same `volume_source` as the receipt).
	pub volume_before: f64,
	/// All Dims after the edit (so the panel can re-render without re-loading).
	pub dims: Vec<DimInfo>,
	/// Rebuild receipt + refreshed viewport mesh (volume here = after).
	pub receipt: MeshReceipt,
	/// The canonical envelope text now on disk (code pane refresh).
	pub envelope: String,
}

/// POST `/api/part/set_dim` — load the part, change one existing Dim, rebuild,
/// **save the recipe back to disk**, re-export the viewport mesh, and return
/// the before/after receipt. This is the PARAMS panel's whole contract: a
/// slider drag is one call, and the file on disk always matches the viewport.
pub async fn set_dim_endpoint(State(state): State<Arc<AppState>>, Json(req): Json<SetDimRequest>) -> Response {
	if !req.value.is_finite() {
		return bad_request("value must be a finite number");
	}
	let session = req.session.clone().unwrap_or_else(|| "default".to_string());
	let out_dir = match state.session_dir(Some(&session)) {
		Ok(d) => d,
		Err(e) => return bad_request(&e),
	};
	let full = match state.repo_file(&req.path) {
		Ok(p) => p,
		Err(e) => return bad_request(&e),
	};
	let text = match std::fs::read_to_string(&full) {
		Ok(t) => t,
		Err(e) => return bad_request(&format!("cannot read '{}': {e}", req.path)),
	};
	let result = tokio::task::spawn_blocking(move || -> Result<SetDimResponse, String> {
		let (mut doc, meta) = load_part(&text).map_err(|e| format!("'{}' is not a loadable .lmcpart: {e}", req.path))?;
		let Some(before) = doc.param(&req.dim) else {
			let available: Vec<String> = dims_of(&doc).into_iter().map(|d| d.name).collect();
			return Err(format!("part has no Dim '{}'; available: [{}]", req.dim, available.join(", ")));
		};
		// Volume before, measured the same way the receipt will measure after.
		let volume_before = match doc.evaluate_brep() {
			Some(solid) => kernel_brep::exact_volume(&solid),
			None => doc.export_mesh(VIEW_TOL).0.signed_volume(),
		};
		doc.set_param(&req.dim, req.value);
		let receipt = rebuild_and_export(&doc, &stem_of(&req.path), &out_dir, &session)?;
		// Rebuild succeeded — persist the edit (canonical bytes, meta preserved).
		let canonical = save_part_with_meta(&doc, &meta.name, meta.meta.as_ref());
		std::fs::write(&full, &canonical).map_err(|e| format!("rebuilt but cannot save '{}': {e}", full.display()))?;
		Ok(SetDimResponse {
			ok: true,
			dim: req.dim,
			before,
			after: req.value,
			volume_before,
			dims: dims_of(&doc),
			receipt,
			envelope: canonical,
		})
	})
	.await;
	match result {
		Ok(Ok(resp)) => Json(resp).into_response(),
		Ok(Err(e)) => bad_request(&e),
		Err(e) => server_error(&format!("set_dim task failed: {e}")),
	}
}
