// Copyright (c) LMCAD. Licensed under the MIT License.

//! The machine-readable execution report: one entry per executed op, a structured
//! error for the (first) failing op, and an overall `ok` flag. This is the ONLY
//! output contract of the binding — an AI caller parses this, never log text.

use serde::{Deserialize, Serialize};

/// Why an op failed. Every failure carries one of these stable, machine-matchable
/// kinds plus a human/AI-readable message naming the offending op and parameter.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ErrorKind {
	/// The program file is not valid JSON or not `{"ops": [...]}` shaped. Reported
	/// on a synthetic op id `$program` since no op could be identified.
	Parse,
	/// The `op` field names no known operation.
	UnknownOp,
	/// Two ops share the same `id` (ids must be unique across the whole program).
	DuplicateId,
	/// An input reference (`in` / `a` / `b` / `sketch`) names no prior op result.
	MissingRef,
	/// An input reference resolved, but to the wrong kind of value (e.g. a sketch
	/// where a solid is required).
	WrongType,
	/// A parameter is missing, malformed, or geometrically degenerate (the kernel
	/// rejected the inputs and produced an empty solid).
	InvalidParam,
	/// A feature operation (fillet / chamfer / rim fillet) could not be applied:
	/// witness matched nothing, radius does not fit, or the edge is outside the
	/// operation's supported scope.
	FeatureFailed,
	/// A sketch could not be solved (conflicting constraints) or could not be
	/// turned into a profile/solid (open loop, degenerate area).
	SketchFailed,
	/// The op ran but its result failed `validate()` — not closed, not manifold, or
	/// negative genus — or was unexpectedly empty. The message carries the
	/// `Validity` details. Invalid geometry is never bound silently.
	InvalidGeometry,
	/// A `library_add` admission gate rejected the candidate: a sample of its
	/// declared parameter ranges failed to build, was not a closed manifold, or
	/// did not rebuild volume-bit-deterministically. The message names the exact
	/// failing sample and its parameter values; nothing was admitted.
	AdmissionRejected,
	/// `library_remove` refused: `.lmcasm` assemblies in the library directory
	/// still reference the entry by path (the message lists them). Pass
	/// `"force": true` to remove anyway.
	DependentsExist,
	/// A declared expectation was not met by the measured geometry: an op's
	/// universal `require` gate failed, an `assert` / `assert_disjoint` op check
	/// failed, or the `asm` runner's mate-residual gate tripped. The message lists
	/// each failed check with measured vs expected values; the op's `measures`
	/// carry the measured numbers. Distinct from `invalid_param`, which means the
	/// GATE ITSELF was malformed (an empty `require`, a key naming no measure) —
	/// "the part is wrong" and "the program is wrong" never share a kind.
	AssertFailed,
	/// A file could not be written (export ops).
	Io,
	/// The kernel panicked. The panic is caught and surfaced as a structured error
	/// so a driving process always gets a report; treat it as a kernel bug.
	Internal,
}

/// A structured op failure: a stable [`ErrorKind`] plus a message that names the
/// failing op id and parameter.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct OpError {
	/// Machine-matchable failure class.
	pub kind: ErrorKind,
	/// Human/AI-readable detail (includes the op id and offending values).
	pub message: String,
}

/// The report entry for one executed op.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct OpReport {
	/// The op's `id` from the program (or `$program` / `#<index>` when the op had
	/// no usable id).
	pub id: String,
	/// Whether the op succeeded.
	pub ok: bool,
	/// Op-specific measurements (validate/volume/mass/sketch-DOF/export route …).
	#[serde(default, skip_serializing_if = "Option::is_none")]
	pub measures: Option<serde_json::Value>,
	/// Non-fatal hazards the interpreter noticed — today: params the op does not
	/// accept (a typo would otherwise leave the default silently in force, the
	/// worst trap on this surface). Keys starting with `_` are the documented
	/// in-op comment convention and never warn. Absent when empty, so reports
	/// without warnings keep their exact historical bytes (determinism contract).
	#[serde(default, skip_serializing_if = "Vec::is_empty")]
	pub warnings: Vec<String>,
	/// The path actually written, for export ops.
	#[serde(default, skip_serializing_if = "Option::is_none")]
	pub file: Option<String>,
	/// The structured failure, present iff `ok` is false.
	#[serde(default, skip_serializing_if = "Option::is_none")]
	pub error: Option<OpError>,
}

/// The whole-program report. Execution stops at the FIRST failing op (later ops
/// are not attempted and have no entry), so `ops` lists every attempted op in
/// program order and `ok` is true iff every attempted op succeeded — which, with
/// the stop-on-failure rule, means the whole program ran.
#[derive(Clone, Debug, PartialEq, Deserialize)]
pub struct Report {
	/// True iff every op in the program executed successfully.
	pub ok: bool,
	/// Per-op results, in program order, up to and including the first failure.
	pub ops: Vec<OpReport>,
}

/// Response contract version (M0). Stamped on every serialized [`Report`] so an agent can
/// detect the envelope it is talking to. A constant, so it never breaks `determinism_same_bytes`.
pub const API_VERSION: &str = "cadcode.v1";

// Manual `Serialize` (not derived) so the response envelope carries `api_version` without adding
// a field to every `Report {…}` construction site. Backward-compatible: input programs are still
// a bare `{"ops":[…]}` (this only shapes the OUTPUT the agent reads back).
impl serde::Serialize for Report {
	fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
		use serde::ser::SerializeStruct;
		let mut st = serializer.serialize_struct("Report", 3)?;
		st.serialize_field("api_version", API_VERSION)?;
		st.serialize_field("ok", &self.ok)?;
		st.serialize_field("ops", &self.ops)?;
		st.end()
	}
}

impl Report {
	/// The report for a program that failed before any op could run, on the
	/// synthetic op id `$program` (unreadable file → [`ErrorKind::Io`],
	/// unparseable JSON / missing `ops` array → [`ErrorKind::Parse`]).
	pub fn program_failure(kind: ErrorKind, message: impl Into<String>) -> Report {
		Report {
			ok: false,
			ops: vec![OpReport {
				id: "$program".to_string(),
				ok: false,
				measures: None,
				warnings: Vec::new(),
				file: None,
				error: Some(OpError { kind, message: message.into() }),
			}],
		}
	}
}
