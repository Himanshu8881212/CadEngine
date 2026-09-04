// Copyright (c) LMCAD. Licensed under the MIT License.

//! `require` — the universal in-program gate.
//!
//! Every op that reports measures accepts an optional `require` object whose
//! keys name that op's OWN measures and whose values are expectations. When an
//! expectation is unmet the op FAILS with [`ErrorKind::AssertFailed`], so a
//! mandatory gate lives in the program instead of an external grep over the
//! report.
//!
//! # Why here and not on `assert`
//!
//! `assert` takes a bound SOLID, so it can only ever gate what can be measured
//! from a solid with no further parameters — genus, shells, closed, manifold,
//! volume. The gates campaigns actually owe (`SPEC` §2.4 export route and
//! watertightness, §2.5 `steep_area == 0`, §2.6 `thin_area`/`p05_thickness`,
//! §2.7 `fits_within`) are all measured by ops that need their own parameters:
//! the export path and format, the build direction and overhang angle, the
//! thin-wall flag threshold, the build envelope. Putting those gates on `assert`
//! would mean duplicating every one of those parameters onto `assert` and then
//! keeping the two copies in step forever — two ways to say the same thing, and
//! a guaranteed drift.
//!
//! Hanging the expectation off the op that already owns the parameters keeps
//! exactly ONE way to express a gate, and it generalises for free: every op that
//! reports measures — including every op added later — is gateable the moment it
//! exists, with no new vocabulary to learn.
//!
//! # Grammar
//!
//! ```json
//! "require": {
//!   "<measure key>": <expectation>,
//!   "<nested.key>": <expectation>,
//!   "<array key>":  [<expectation> | null, ...]
//! }
//! ```
//!
//! A key is a dotted PATH into the op's measures (`bbox.size`, `stress.max`);
//! an integer path segment indexes an array (`size.2`). An expectation is
//! either:
//!
//! * a plain JSON scalar (`true`, `1`, `"exact"`, `null`) — must be EQUAL;
//! * an array — element-wise against an array measure, `null` skipping an element;
//! * an object with one or more of
//!   `equals` / `min` / `max` / `within` / `not_null`, all of which must hold.
//!   `min` and `max` are INCLUSIVE; `within` is
//!   `{"target": t, "abs": a}` or `{"target": t, "percent": p}`.
//!
//! # Refusals (never a silent pass)
//!
//! * `require` present but empty → refused. An empty gate that reports success
//!   is the worst possible outcome.
//! * a key that names no measure → refused, listing the keys that exist. A typo
//!   must never become a gate that quietly checks nothing.
//! * a numeric bound against a non-numeric (or absent) measure → refused.
//! * `require` on an op that reports no measures → refused.

use serde_json::{Map, Value};

use crate::interp::err;
use crate::report::{ErrorKind, OpError};

/// The wire name of the universal gate parameter. Accepted on every op, so the
/// unknown-param tripwire must not flag it.
pub(crate) const REQUIRE_KEY: &str = "require";

/// One-line description of `require`, advertised by `describe` for every op so
/// the gate vocabulary is discoverable from the binary (never only from a doc).
pub(crate) const REQUIRE_DOC: &str =
	"Universal gate: {\"<measure key>\": expectation} checked against this op's own measures; the op FAILS (assert_failed) when an expectation is unmet. Expectation = scalar (equality), array (element-wise), or {equals|min|max|within|not_null}. Keys may be dotted paths.";

/// Apply the `require` object of `raw` to the measures an op produced.
///
/// Returns the measures to report (the input plus a `required` echo, so the
/// receipt records the gate that was applied and not merely that it passed).
/// `Ok(None)` means the op declared no `require` and its measures are unchanged
/// — byte-identical reports for every program written before this existed.
pub(crate) fn apply(op_id: &str, raw: &Map<String, Value>, measures: Option<&Value>) -> Result<Option<Value>, OpError> {
	let Some(spec) = raw.get(REQUIRE_KEY) else {
		return Ok(None);
	};
	let Some(spec) = spec.as_object() else {
		return Err(err(
			ErrorKind::InvalidParam,
			format!("op '{op_id}': 'require' must be a JSON object mapping measure keys to expectations, e.g. {{\"watertight\": true}}"),
		));
	};
	if spec.is_empty() {
		return Err(err(
			ErrorKind::InvalidParam,
			format!("op '{op_id}': 'require' is empty — a gate that checks nothing must not report success; give at least one expectation"),
		));
	}
	let Some(Value::Object(measured)) = measures else {
		return Err(err(
			ErrorKind::InvalidParam,
			format!(
				"op '{op_id}': 'require' has nothing to check — this op reports no measures. Gate it with an op that measures (validate / bounding_box / support_report / wall_thickness / mesh_components / clearance / export_*)."
			),
		));
	};

	let mut failures: Vec<String> = Vec::new();
	// BTreeMap ordering of the incoming object is preserved by serde_json's
	// `preserve_order`-free default (a BTreeMap), so failure text is deterministic.
	for (path, expectation) in spec {
		let value = lookup(measured, path).ok_or_else(|| {
			err(
				ErrorKind::InvalidParam,
				format!("op '{op_id}': require key '{path}' names no measure of this op — it measures: {}", key_list(measured)),
			)
		})?;
		check(op_id, path, value, expectation, &mut failures)?;
	}

	if !failures.is_empty() {
		return Err(err(ErrorKind::AssertFailed, format!("op '{op_id}': require failed: {}", failures.join("; "))));
	}
	// The echo must never overwrite a real measurement. No op emits a `required`
	// measure today; if one ever does, this refuses loudly instead of silently
	// replacing its value with the gate.
	if measured.contains_key("required") {
		return Err(err(
			ErrorKind::Internal,
			format!(
				"op '{op_id}': this op already reports a measure named 'required', which collides with the gate echo — rename the measure"
			),
		));
	}
	let mut out = measured.clone();
	out.insert("required".to_string(), Value::Object(spec.clone()));
	Ok(Some(Value::Object(out)))
}

/// Comma-joined measure keys of an object, for the "no such measure" refusal.
fn key_list(m: &Map<String, Value>) -> String {
	m.keys().map(String::as_str).collect::<Vec<_>>().join(", ")
}

/// Resolve a dotted path (`bbox.size.2`) against a measures object. An integer
/// segment indexes an array; anything else indexes an object.
fn lookup<'v>(measured: &'v Map<String, Value>, path: &str) -> Option<&'v Value> {
	let mut cur: &Value = measured.get(path.split('.').next()?)?;
	for seg in path.split('.').skip(1) {
		cur = match cur {
			Value::Array(a) => a.get(seg.parse::<usize>().ok()?)?,
			Value::Object(o) => o.get(seg)?,
			_ => return None,
		};
	}
	Some(cur)
}

/// Check one expectation, pushing a human-readable line onto `failures` when it
/// is unmet. Returns `Err` only for a MALFORMED expectation (a program bug, not
/// a gate result) — the two outcomes must never be confused.
fn check(op_id: &str, path: &str, value: &Value, expectation: &Value, failures: &mut Vec<String>) -> Result<(), OpError> {
	match expectation {
		// An array expectation is element-wise; `null` skips an element. This is
		// how a bounding box gets a per-axis gate without a bespoke vocabulary.
		Value::Array(want) => {
			let Value::Array(got) = value else {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': require '{path}': an array expectation needs an array measure, but '{path}' measured {value}"),
				));
			};
			if got.len() != want.len() {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': require '{path}': expectation has {} elements, the measure has {}", want.len(), got.len()),
				));
			}
			for (i, (g, w)) in got.iter().zip(want).enumerate() {
				if w.is_null() {
					continue; // documented "don't care" slot
				}
				check(op_id, &format!("{path}.{i}"), g, w, failures)?;
			}
			Ok(())
		}
		Value::Object(o) => check_object(op_id, path, value, o, failures),
		// Any other scalar is an equality expectation.
		want => {
			if value != want {
				failures.push(format!("{path}: measured {value}, expected {want}"));
			}
			Ok(())
		}
	}
}

/// The `{equals|min|max|within|not_null}` form. Every present clause must hold.
fn check_object(op_id: &str, path: &str, value: &Value, spec: &Map<String, Value>, failures: &mut Vec<String>) -> Result<(), OpError> {
	const CLAUSES: [&str; 5] = ["equals", "min", "max", "within", "not_null"];
	if spec.is_empty() {
		return Err(err(
			ErrorKind::InvalidParam,
			format!("op '{op_id}': require '{path}': the expectation object is empty — give one of {}", CLAUSES.join(" / ")),
		));
	}
	for k in spec.keys() {
		if !CLAUSES.contains(&k.as_str()) {
			return Err(err(
				ErrorKind::InvalidParam,
				format!("op '{op_id}': require '{path}': unknown clause '{k}' — expected one of {}", CLAUSES.join(" / ")),
			));
		}
	}
	// A numeric clause needs a number to compare against; refusing here (rather
	// than failing the gate) keeps "the program is wrong" separate from "the part
	// is wrong". `null` is the common case: a measure the op could not compute.
	let numeric_wanted = spec.contains_key("min") || spec.contains_key("max") || spec.contains_key("within");
	let measured = value.as_f64();
	if numeric_wanted && measured.is_none() {
		return Err(err(
			ErrorKind::InvalidParam,
			format!("op '{op_id}': require '{path}': min/max/within need a numeric measure, but '{path}' measured {value}"),
		));
	}

	if let Some(want) = spec.get("not_null") {
		let Some(want) = want.as_bool() else {
			return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': require '{path}': 'not_null' must be true or false")));
		};
		if want == value.is_null() {
			failures.push(format!("{path}: measured {value}, expected {}", if want { "a value (not null)" } else { "null" }));
		}
	}
	if let Some(want) = spec.get("equals") {
		if value != want {
			failures.push(format!("{path}: measured {value}, expected {want}"));
		}
	}
	if let Some(bound) = spec.get("min") {
		let bound = number(op_id, path, "min", bound)?;
		if measured.unwrap_or(f64::NAN) < bound {
			failures.push(format!("{path}: measured {value}, expected >= {bound}"));
		}
	}
	if let Some(bound) = spec.get("max") {
		let bound = number(op_id, path, "max", bound)?;
		if measured.unwrap_or(f64::NAN) > bound {
			failures.push(format!("{path}: measured {value}, expected <= {bound}"));
		}
	}
	if let Some(w) = spec.get("within") {
		let Some(w) = w.as_object() else {
			return Err(err(
				ErrorKind::InvalidParam,
				format!(
					"op '{op_id}': require '{path}': 'within' must be {{\"target\": t, \"abs\": a}} or {{\"target\": t, \"percent\": p}}"
				),
			));
		};
		for k in w.keys() {
			if !["target", "abs", "percent"].contains(&k.as_str()) {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': require '{path}': 'within' has unknown key '{k}' — expected target / abs / percent"),
				));
			}
		}
		let target = number(op_id, path, "within.target", w.get("target").unwrap_or(&Value::Null))?;
		let half_width = match (w.get("abs"), w.get("percent")) {
			(Some(a), None) => number(op_id, path, "within.abs", a)?,
			(None, Some(p)) => target.abs() * number(op_id, path, "within.percent", p)? / 100.0,
			_ => {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': require '{path}': 'within' needs EXACTLY one of 'abs' / 'percent'"),
				));
			}
		};
		if half_width < 0.0 {
			return Err(err(
				ErrorKind::InvalidParam,
				format!("op '{op_id}': require '{path}': 'within' tolerance must be non-negative, got {half_width}"),
			));
		}
		if (measured.unwrap_or(f64::NAN) - target).abs() > half_width {
			failures.push(format!("{path}: measured {value}, expected {target} ± {half_width}"));
		}
	}
	Ok(())
}

/// A finite number from an expectation clause, or a refusal naming the clause.
fn number(op_id: &str, path: &str, clause: &str, v: &Value) -> Result<f64, OpError> {
	match v.as_f64() {
		Some(x) if x.is_finite() => Ok(x),
		_ => Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': require '{path}': '{clause}' must be a finite number, got {v}"))),
	}
}
