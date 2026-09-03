// Copyright (c) LMCAD. Licensed under the MIT License.

//! The curated, admission-gated parts library (BAR.md I7): `library_add` /
//! `library_search` / `library_instantiate` / `library_deprecate` /
//! `library_remove`.

use std::path::Path;

#[cfg(feature = "catalog")]
use kernel_model::library::{AddOptions, AdmissionError, EntryMeta, Library, LibraryError, ParamSpec, Provenance};
use serde_json::{json, Value};

use crate::interp::{err, Outcome};
#[cfg(feature = "catalog")]
use crate::program::LibraryMetaSpec;
use crate::program::OpKind;
use crate::report::{ErrorKind, OpError};

use super::meshio::{resolve_input_path};
use super::support::{bind_solid};

/// Map a kernel [`AdmissionError`] to a structured op error: gate failures
/// (build / validity / determinism at a sample) are `admission_rejected`, file
/// writes are `io`, and meta/format problems are `invalid_param` — each with
/// the kernel's precise message (gate messages name the failing sample).
#[cfg(feature = "catalog")]
pub(crate) fn map_admission_error(op_id: &str, e: AdmissionError) -> OpError {
	let kind = match &e {
		AdmissionError::GateBuildFailed { .. } | AdmissionError::GateInvalid { .. } | AdmissionError::GateNondeterministic { .. } => {
			ErrorKind::AdmissionRejected
		}
		AdmissionError::Io { .. } => ErrorKind::Io,
		_ => ErrorKind::InvalidParam,
	};
	err(kind, format!("op '{op_id}': library_add: {e}"))
}

/// Map a kernel [`LibraryError`] to a structured op error: the dependents
/// refusal keeps its own machine-matchable kind, I/O stays `io`, and the rest
/// (unknown name/version/parameter, out-of-range value, corrupt index/part)
/// is `invalid_param` with the kernel's precise message.
#[cfg(feature = "catalog")]
pub(crate) fn map_library_error(op_id: &str, what: &str, e: LibraryError) -> OpError {
	let kind = match &e {
		LibraryError::DependentsExist { .. } => ErrorKind::DependentsExist,
		LibraryError::Io { .. } | LibraryError::AsmUnreadable { .. } => ErrorKind::Io,
		_ => ErrorKind::InvalidParam,
	};
	err(kind, format!("op '{op_id}': {what}: {e}"))
}

/// Open (creating on demand) the library at `dir`, resolved like input paths
/// (confined under `--out-dir` — absolute paths and `..` are refused).
#[cfg(feature = "catalog")]
pub(crate) fn open_library(op_id: &str, out_dir: &Path, dir: &str) -> Result<Library, OpError> {
	Library::open(resolve_input_path(op_id, out_dir, dir)?).map_err(|e| map_library_error(op_id, "library", e))
}

/// Translate the JSON `meta` of `library_add` into the kernel's [`EntryMeta`].
#[cfg(feature = "catalog")]
pub(crate) fn to_kernel_meta(meta: LibraryMetaSpec) -> EntryMeta {
	EntryMeta {
		name: meta.name,
		version: meta.version,
		category: meta.category,
		tags: meta.tags,
		description: meta.description,
		provenance: Provenance {
			author: meta.provenance.author,
			created_with: meta.provenance.created_with,
			date: meta.provenance.date,
		},
		param_interface: meta
			.params
			.into_iter()
			.map(|p| ParamSpec { name: p.name, units: p.units, default: p.default, min: p.min, max: p.max, description: p.description })
			.collect(),
	}
}

/// Execute one op of this family. The dispatch table in [`crate::interp`]
/// routes exactly the variants matched below, so the catch-all is dead code
/// kept only to satisfy the compiler.
pub(crate) fn exec(op_id: &str, out_dir: &Path, kind: OpKind) -> Result<Outcome, OpError> {
	match kind {
		#[cfg(feature = "catalog")]
		OpKind::LibraryAdd { dir, part, part_file, meta } => {
			let part_json = match (part, part_file) {
				// An inline envelope object; a JSON string is accepted too and
				// treated as the raw envelope text (e.g. pasted file contents).
				(Some(Value::String(text)), None) => text,
				(Some(value), None) => value.to_string(),
				(None, Some(file)) => {
					let path = resolve_input_path(op_id, out_dir, &file)?;
					std::fs::read_to_string(&path)
						.map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot read '{}': {e}", path.display())))?
				}
				_ => {
					return Err(err(
						ErrorKind::InvalidParam,
						format!("op '{op_id}': exactly one of 'part' (inline .lmcpart envelope) or 'part_file' (path to one) is required"),
					));
				}
			};
			let mut library = open_library(op_id, out_dir, &dir)?;
			let entry = library
				.add(&part_json, to_kernel_meta(meta), &AddOptions::default())
				.map_err(|e| map_admission_error(op_id, e))?;
			Ok(Outcome::measures(json!({
				"name": entry.name,
				"version": entry.version,
				"file": entry.file,
				"gate_samples": entry.admitted.samples.len(),
				"gate_rebuilds": entry.admitted.rebuilds,
				// The gate's first sample is always the interface defaults.
				"volume_at_defaults": entry.admitted.samples.first().map(|s| s.volume),
			})))
		}
		#[cfg(feature = "catalog")]
		OpKind::LibrarySearch { dir, text, tags } => {
			let library = open_library(op_id, out_dir, &dir)?;
			let matches: Vec<Value> = library
				.search(&text, &tags)
				.into_iter()
				.map(|e| {
					json!({
						"name": e.name,
						"version": e.version,
						"category": e.category,
						"tags": e.tags,
						"description": e.description,
						"params": e
							.param_interface
							.iter()
							.map(|p| json!({ "name": p.name, "units": p.units, "default": p.default, "min": p.min, "max": p.max }))
							.collect::<Vec<_>>(),
					})
				})
				.collect();
			Ok(Outcome::measures(json!({ "matches": matches })))
		}
		#[cfg(feature = "catalog")]
		OpKind::LibraryInstantiate { dir, name, version, params } => {
			let library = open_library(op_id, out_dir, &dir)?;
			let built = library
				.instantiate(&name, version, &params)
				.map_err(|e| map_library_error(op_id, "library_instantiate", e))?;
			let solid = built.document.evaluate_brep().ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': the entry's feature tree produced no exact B-rep (voxel-half-only features — shell, gyroid, smooth booleans — cannot enter the solid environment)"),
				)
			})?;
			let outcome = bind_solid(op_id, "library_instantiate", solid)?;
			let mut measures = json!({
				"name": built.name,
				"version": built.version,
				"deprecated": built.deprecated,
				"params": params,
			});
			if built.deprecated {
				// BAR.md I7: a deprecated entry still builds, but instantiate WARNS.
				measures["warning"] = json!(format!(
					"'{}' v{} is deprecated — it still builds, but curation retired it; search the library for a successor",
					built.name, built.version
				));
			}
			Ok(Outcome { measures: Some(measures), ..outcome })
		}
		#[cfg(feature = "catalog")]
		OpKind::LibraryDeprecate { dir, name } => {
			let mut library = open_library(op_id, out_dir, &dir)?;
			let count = library.deprecate(&name).map_err(|e| map_library_error(op_id, "library_deprecate", e))?;
			Ok(Outcome::measures(json!({ "name": name, "deprecated_versions": count })))
		}
		#[cfg(feature = "catalog")]
		OpKind::LibraryRemove { dir, name, force } => {
			let mut library = open_library(op_id, out_dir, &dir)?;
			let removed = library.remove(&name, force).map_err(|e| map_library_error(op_id, "library_remove", e))?;
			Ok(Outcome::measures(json!({ "name": name, "removed_files": removed, "forced": force })))
		}

		_ => unreachable!("ops::library: op routed to the wrong family"),
	}
}
