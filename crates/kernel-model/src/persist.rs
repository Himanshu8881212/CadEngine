// Copyright (c) LMCAD. Licensed under the MIT License.

//! [`Document`] persistence (BAR.md, I3) — the parametric model as a JSON file.
//!
//! A [`Document`] is pure data (parameters, features, sketches, suppression
//! state, root), so it serializes losslessly: [`Document::save_json`] /
//! [`Document::load_json`] round-trip every `f64` **bit-exactly** (serde_json
//! writes shortest-round-trip decimals), and the kernel's deterministic rebuild
//! (R5) then re-evaluates the loaded document to the bit-identical solid. This is
//! the design file an AI session can write, resume, and keep editing
//! parametrically — geometry is never stored, only the recipe.
//!
//! ## Schema contract (honest scope)
//! - The schema is **exact for this kernel version**: it mirrors the Rust types
//!   field-for-field with no version stamp or migration layer. Unknown *fields*
//!   in a struct are ignored (serde's default), but an unknown enum **variant**
//!   (a feature/constraint kind from a newer kernel) fails to load with a serde
//!   error — loud, never a silently dropped feature.
//! - Save output is **deterministic**: the parameter map and suppression set are
//!   written in sorted order, so saving the same document twice yields identical
//!   bytes (diff-able files).
//! - Values must be finite: serde_json writes a non-finite `f64` (NaN/∞) as
//!   `null`, which then **fails to load**. No feature constructor produces
//!   non-finite dimensions, so this only arises from a hand-corrupted document.

use std::collections::{HashMap, HashSet};

use serde::ser::Serialize as _;

use crate::{Document, FeatureId};

impl Document {
	/// Serialize this document — parameters, features (sketches included), root
	/// and suppression state — to a pretty-printed JSON string. Deterministic:
	/// the same document always yields the same bytes (maps/sets are sorted).
	/// See the [module docs](self) for the schema contract.
	pub fn save_json(&self) -> String {
		// Infallible for a `Document`: all map keys are strings and the tree is
		// acyclic data (serde_json only errors on non-string keys or an I/O sink).
		serde_json::to_string_pretty(self).expect("a Document serializes: string keys only, plain data")
	}

	/// Parse a document back from [`Document::save_json`] output. Malformed JSON
	/// or a schema mismatch (e.g. an unknown feature variant from a newer kernel)
	/// returns the serde error — loading never panics and never half-loads.
	pub fn load_json(json: &str) -> Result<Document, serde_json::Error> {
		serde_json::from_str(json)
	}

	/// [`Document::save_json`] straight into a writer (a file, a socket) without
	/// building the intermediate string.
	pub fn save_json_writer<W: std::io::Write>(&self, writer: W) -> Result<(), serde_json::Error> {
		serde_json::to_writer_pretty(writer, self)
	}

	/// [`Document::load_json`] straight from a reader (a file, a socket).
	pub fn load_json_reader<R: std::io::Read>(reader: R) -> Result<Document, serde_json::Error> {
		serde_json::from_reader(reader)
	}
}

/// Serialize the parameter table in sorted key order, so a saved document is
/// byte-deterministic (a `HashMap` would write in per-process random order).
pub(crate) fn sorted_params<S: serde::Serializer>(params: &HashMap<String, f64>, ser: S) -> Result<S::Ok, S::Error> {
	let sorted: std::collections::BTreeMap<&str, f64> = params.iter().map(|(k, &v)| (k.as_str(), v)).collect();
	sorted.serialize(ser)
}

/// Serialize the suppression set as a sorted id list (same determinism rationale
/// as [`sorted_params`]); it deserializes back through the plain `HashSet` impl.
pub(crate) fn sorted_feature_ids<S: serde::Serializer>(ids: &HashSet<FeatureId>, ser: S) -> Result<S::Ok, S::Error> {
	let mut sorted: Vec<FeatureId> = ids.iter().copied().collect();
	sorted.sort();
	sorted.serialize(ser)
}

/// Serde bridge for [`kernel_brep::EdgeName`] — a foreign type (`kernel-brep`
/// stays serde-free) mirrored field-for-field through local repr types. An edge
/// name is written as its two `(operand, source_face)` face names; loading
/// re-canonicalizes through [`kernel_brep::EdgeName::new`], so a hand-edited
/// file cannot smuggle in a non-canonical (unsorted) name.
pub(crate) mod edge_name_serde {
	use kernel_brep::{EdgeName, FaceName, FaceSource};
	use serde::{Deserialize, Deserializer, Serialize, Serializer};

	/// JSON mirror of [`FaceSource`].
	#[derive(Serialize, Deserialize)]
	enum FaceSourceRepr {
		Primitive,
		OperandA,
		OperandB,
	}

	/// JSON mirror of [`FaceName`].
	#[derive(Serialize, Deserialize)]
	struct FaceNameRepr {
		operand: FaceSourceRepr,
		source_face: u32,
	}

	impl From<FaceName> for FaceNameRepr {
		fn from(name: FaceName) -> Self {
			let operand = match name.operand {
				FaceSource::Primitive => FaceSourceRepr::Primitive,
				FaceSource::OperandA => FaceSourceRepr::OperandA,
				FaceSource::OperandB => FaceSourceRepr::OperandB,
			};
			FaceNameRepr { operand, source_face: name.source_face }
		}
	}

	impl From<FaceNameRepr> for FaceName {
		fn from(repr: FaceNameRepr) -> Self {
			let operand = match repr.operand {
				FaceSourceRepr::Primitive => FaceSource::Primitive,
				FaceSourceRepr::OperandA => FaceSource::OperandA,
				FaceSourceRepr::OperandB => FaceSource::OperandB,
			};
			FaceName { operand, source_face: repr.source_face }
		}
	}

	pub(crate) fn serialize<S: Serializer>(name: &EdgeName, ser: S) -> Result<S::Ok, S::Error> {
		let faces: [FaceNameRepr; 2] = [name.faces[0].into(), name.faces[1].into()];
		faces.serialize(ser)
	}

	pub(crate) fn deserialize<'de, D: Deserializer<'de>>(de: D) -> Result<EdgeName, D::Error> {
		let [a, b] = <[FaceNameRepr; 2]>::deserialize(de)?;
		Ok(EdgeName::new(a.into(), b.into()))
	}
}
