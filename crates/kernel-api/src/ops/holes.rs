// Copyright (c) LMCAD. Licensed under the MIT License.

//! The ISO/DIN hole wizard: `drill`, `clearance_hole`, `counterbore_hole`,
//! `countersink_hole`, `tap_drill_hole`, the `bolt_circle` pattern and the
//! `bearing_seat` cut — each off the standard tables, each echoing the table row
//! it used.

use std::collections::{BTreeMap, BTreeSet};

use kernel_brep::holes::{self, HoleDepth};
use serde_json::{json, Value};

use crate::interp::{err, fetch_solid, EnvValue, Outcome};
use crate::program::{BoltHoleSpec, FitSpec, OpKind};
use crate::report::{ErrorKind, OpError};

use super::support::{bind_solid, dv3, map_hole_error};

/// Resolve the mutually exclusive `depth` (blind) / `through` JSON params of the
/// drilling ops into a [`HoleDepth`].
pub(crate) fn hole_depth(op_id: &str, depth: Option<f64>, through: Option<f64>) -> Result<HoleDepth, OpError> {
	match (depth, through) {
		(Some(d), None) => Ok(HoleDepth::Blind(d)),
		(None, Some(t)) => Ok(HoleDepth::Through(t)),
		_ => Err(err(
			ErrorKind::InvalidParam,
			format!("op '{op_id}': exactly one of 'depth' (blind hole) or 'through' (through-hole material span) is required"),
		)),
	}
}

/// Translate the JSON fit series into the kernel's [`holes::Fit`].
pub(crate) fn to_kernel_fit(fit: FitSpec) -> holes::Fit {
	match fit {
		FitSpec::Close => holes::Fit::Close,
		FitSpec::Medium => holes::Fit::Medium,
		FitSpec::Coarse => holes::Fit::Coarse,
	}
}

/// The JSON name of a fit series, for echoing in measures.
pub(crate) fn fit_name(fit: FitSpec) -> &'static str {
	match fit {
		FitSpec::Close => "close",
		FitSpec::Medium => "medium",
		FitSpec::Coarse => "coarse",
	}
}

/// The ISO/DIN table row a hole-wizard cut used, echoed as measures so a caller
/// can pose mating hardware without reading the kernel source (FRICTION #9).
/// Call only after the cut succeeded (which proves `m` is in the table).
pub(crate) fn metric_spec_row(m: f64) -> &'static holes::MetricHoleSpec {
	holes::metric_hole_spec(m).expect("the cut succeeded, so the size is in the table")
}

/// The blind/through depth facts of a drill-style cut, for measures.
pub(crate) fn depth_measures(measures: &mut serde_json::Map<String, Value>, d: f64, dep: HoleDepth) {
	match dep {
		HoleDepth::Blind(depth) => {
			measures.insert("kind".into(), json!("blind"));
			measures.insert("depth".into(), json!(depth));
			// the 118° point extends past the full-diameter depth
			measures.insert("point_depth".into(), json!(depth + holes::drill_tip_height(d)));
		}
		HoleDepth::Through(span) => {
			measures.insert("kind".into(), json!("through"));
			measures.insert("through".into(), json!(span));
		}
	}
}

/// Execute one op of this family. The dispatch table in [`crate::interp`]
/// routes exactly the variants matched below, so the catch-all is dead code
/// kept only to satisfy the compiler.
pub(crate) fn exec(
	op_id: &str,
	env: &mut BTreeMap<String, EnvValue>,
	all_ids: &BTreeSet<String>,
	kind: OpKind,
) -> Result<Outcome, OpError> {
	match kind {
		OpKind::Drill { input, at, axis, d, depth, through, segments } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let dep = hole_depth(op_id, depth, through)?;
			let solid = holes::drill(s, dv3(at), dv3(axis), d, dep, segments).map_err(|e| map_hole_error(op_id, "drill", e))?;
			let mut measures = serde_json::Map::new();
			measures.insert("d".into(), json!(d));
			depth_measures(&mut measures, d, dep);
			Ok(Outcome { measures: Some(Value::Object(measures)), ..bind_solid(op_id, "drill", solid)? })
		}
		OpKind::ClearanceHole { input, at, axis, m, fit, segments } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid = holes::clearance_hole(s, dv3(at), dv3(axis), m, to_kernel_fit(fit), segments)
				.map_err(|e| map_hole_error(op_id, "clearance_hole", e))?;
			let spec = metric_spec_row(m);
			let measures = json!({
				"m": m,
				"fit": fit_name(fit),
				"clearance_d": spec.clearance[to_kernel_fit(fit) as usize],
			});
			Ok(Outcome { measures: Some(measures), ..bind_solid(op_id, "clearance_hole", solid)? })
		}
		OpKind::CounterboreHole { input, at, axis, m, fit, segments } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid = holes::counterbore_hole(s, dv3(at), dv3(axis), m, to_kernel_fit(fit), segments)
				.map_err(|e| map_hole_error(op_id, "counterbore_hole", e))?;
			let spec = metric_spec_row(m);
			let measures = json!({
				"m": m,
				"fit": fit_name(fit),
				"clearance_d": spec.clearance[to_kernel_fit(fit) as usize],
				"counterbore_d": spec.counterbore_d,
				"counterbore_depth": spec.counterbore_depth,
			});
			Ok(Outcome { measures: Some(measures), ..bind_solid(op_id, "counterbore_hole", solid)? })
		}
		OpKind::CountersinkHole { input, at, axis, m, fit, segments } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid = holes::countersink_hole(s, dv3(at), dv3(axis), m, to_kernel_fit(fit), segments)
				.map_err(|e| map_hole_error(op_id, "countersink_hole", e))?;
			let spec = metric_spec_row(m);
			let measures = json!({
				"m": m,
				"fit": fit_name(fit),
				"clearance_d": spec.clearance[to_kernel_fit(fit) as usize],
				// the cut succeeded, so the form-F row exists (M3+)
				"countersink_d": spec.countersink_d.expect("countersink cut succeeded, so the form-F row exists"),
			});
			Ok(Outcome { measures: Some(measures), ..bind_solid(op_id, "countersink_hole", solid)? })
		}
		OpKind::TapDrillHole { input, at, axis, m, depth, through, segments } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let dep = hole_depth(op_id, depth, through)?;
			let solid =
				holes::tap_drill_hole(s, dv3(at), dv3(axis), m, dep, segments).map_err(|e| map_hole_error(op_id, "tap_drill_hole", e))?;
			let spec = metric_spec_row(m);
			let pilot_d = spec.m - spec.pitch;
			let mut measures = serde_json::Map::new();
			measures.insert("m".into(), json!(m));
			measures.insert("pitch".into(), json!(spec.pitch));
			measures.insert("pilot_d".into(), json!(pilot_d));
			depth_measures(&mut measures, pilot_d, dep);
			Ok(Outcome { measures: Some(Value::Object(measures)), ..bind_solid(op_id, "tap_drill_hole", solid)? })
		}
		OpKind::BoltCircle { input, center, axis, circle_d, n, start_deg, hole, segments } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			if !start_deg.is_finite() {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': start_deg must be finite")));
			}
			// Validate exclusive depth params up front (bolt_circle would surface
			// them per hole otherwise) and pre-build the per-hole measure echo.
			let axis_v = dv3(axis);
			let mut hole_measures = serde_json::Map::new();
			let solid = match hole {
				BoltHoleSpec::Drill { d, depth, through } => {
					let dep = hole_depth(op_id, depth, through)?;
					hole_measures.insert("hole".into(), json!("drill"));
					hole_measures.insert("d".into(), json!(d));
					depth_measures(&mut hole_measures, d, dep);
					holes::bolt_circle(s, dv3(center), axis_v, circle_d, n, start_deg.to_radians(), |acc, p| {
						holes::drill(&acc, p, axis_v, d, dep, segments)
					})
				}
				BoltHoleSpec::Clearance { m, fit } => {
					hole_measures.insert("hole".into(), json!("clearance"));
					hole_measures.insert("m".into(), json!(m));
					hole_measures.insert("fit".into(), json!(fit_name(fit)));
					holes::bolt_circle(s, dv3(center), axis_v, circle_d, n, start_deg.to_radians(), |acc, p| {
						holes::clearance_hole(&acc, p, axis_v, m, to_kernel_fit(fit), segments)
					})
				}
				BoltHoleSpec::Counterbore { m, fit } => {
					hole_measures.insert("hole".into(), json!("counterbore"));
					hole_measures.insert("m".into(), json!(m));
					hole_measures.insert("fit".into(), json!(fit_name(fit)));
					holes::bolt_circle(s, dv3(center), axis_v, circle_d, n, start_deg.to_radians(), |acc, p| {
						holes::counterbore_hole(&acc, p, axis_v, m, to_kernel_fit(fit), segments)
					})
				}
				BoltHoleSpec::Countersink { m, fit } => {
					hole_measures.insert("hole".into(), json!("countersink"));
					hole_measures.insert("m".into(), json!(m));
					hole_measures.insert("fit".into(), json!(fit_name(fit)));
					holes::bolt_circle(s, dv3(center), axis_v, circle_d, n, start_deg.to_radians(), |acc, p| {
						holes::countersink_hole(&acc, p, axis_v, m, to_kernel_fit(fit), segments)
					})
				}
				BoltHoleSpec::TapDrill { m, depth, through } => {
					let dep = hole_depth(op_id, depth, through)?;
					hole_measures.insert("hole".into(), json!("tap_drill"));
					hole_measures.insert("m".into(), json!(m));
					holes::bolt_circle(s, dv3(center), axis_v, circle_d, n, start_deg.to_radians(), |acc, p| {
						holes::tap_drill_hole(&acc, p, axis_v, m, dep, segments)
					})
				}
			}
			.map_err(|e| map_hole_error(op_id, "bolt_circle", e))?;
			// Echo the table row for metric cuts now that the cut proved m valid.
			match hole {
				BoltHoleSpec::Clearance { m, fit } | BoltHoleSpec::Counterbore { m, fit } | BoltHoleSpec::Countersink { m, fit } => {
					let spec = metric_spec_row(m);
					hole_measures.insert("clearance_d".into(), json!(spec.clearance[to_kernel_fit(fit) as usize]));
					if matches!(hole, BoltHoleSpec::Counterbore { .. }) {
						hole_measures.insert("counterbore_d".into(), json!(spec.counterbore_d));
						hole_measures.insert("counterbore_depth".into(), json!(spec.counterbore_depth));
					}
					if matches!(hole, BoltHoleSpec::Countersink { .. }) {
						hole_measures
							.insert("countersink_d".into(), json!(spec.countersink_d.expect("countersink cut succeeded, so the form-F row exists")));
					}
				}
				BoltHoleSpec::TapDrill { m, depth, through } => {
					let spec = metric_spec_row(m);
					let pilot_d = spec.m - spec.pitch;
					hole_measures.insert("pitch".into(), json!(spec.pitch));
					hole_measures.insert("pilot_d".into(), json!(pilot_d));
					// re-derive the (already validated) depth for the echo
					depth_measures(&mut hole_measures, pilot_d, hole_depth(op_id, depth, through)?);
				}
				BoltHoleSpec::Drill { .. } => {}
			}
			let measures = json!({
				"n": n,
				"circle_d": circle_d,
				"start_deg": start_deg,
				"hole": Value::Object(hole_measures),
			});
			Ok(Outcome { measures: Some(measures), ..bind_solid(op_id, "bolt_circle", solid)? })
		}
		OpKind::BearingSeat { input, at, axis, bearing, segments } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid =
				holes::bearing_seat(s, dv3(at), dv3(axis), &bearing, segments).map_err(|e| map_hole_error(op_id, "bearing_seat", e))?;
			let spec = holes::bearing_spec(&bearing).expect("the seat cut succeeded, so the designation is in the table");
			let measures = json!({
				"bearing": spec.designation,
				"bore_d": spec.bore,
				"outer_d": spec.outer,
				"width": spec.width,
				"pocket_d": spec.outer,
				"pocket_depth": spec.width,
				"shoulder_d": (spec.bore + spec.outer) * 0.5,
			});
			Ok(Outcome { measures: Some(measures), ..bind_solid(op_id, "bearing_seat", solid)? })
		}

		_ => unreachable!("ops::holes: op routed to the wrong family"),
	}
}
