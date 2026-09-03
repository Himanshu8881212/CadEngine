// Copyright (c) LMCAD. Licensed under the MIT License.

//! Design-math lookups: sizing tables that return numbers, not geometry — GT2 belt
//! sizing, ISO 286 limit fits, heat-set and O-ring cord specs, G pipe threads.

use kernel_model::parts;
use serde_json::json;

use crate::interp::{err, Outcome};
use crate::program::OpKind;
use crate::report::{ErrorKind, OpError};

use super::support::{METRIC_CORD_SIZES, SMALL_SIZES_M2_M6};

/// Execute one op of this family. The dispatch table in [`crate::interp`]
/// routes exactly the variants matched below, so the catch-all is dead code
/// kept only to satisfy the compiler.
pub(crate) fn exec(op_id: &str, kind: OpKind) -> Result<Outcome, OpError> {
	match kind {
		#[cfg(feature = "catalog")]
		OpKind::Gt2Belt { center_distance, t1, t2 } => {
			let (pitch_length, belt_teeth) = parts::gt2_belt(center_distance, t1, t2).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': gt2_belt: needs t1, t2 ≥ 2 and center_distance beyond the pitch-radius sum (pitch Ø = teeth·2/π)"),
				)
			})?;
			Ok(Outcome::measures(json!({ "pitch_length": pitch_length, "belt_teeth": belt_teeth })))
		}
		#[cfg(feature = "catalog")]
		OpKind::Gt2CenterDistance { belt_teeth, t1, t2 } => {
			let center_distance = parts::gt2_center_distance(belt_teeth, t1, t2).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': gt2_center_distance: needs t1, t2 ≥ 2 and a belt long enough to wrap both pulleys"),
				)
			})?;
			Ok(Outcome::measures(json!({ "center_distance": center_distance })))
		}
		OpKind::Iso286Fit { d, fit } => {
			let f = parts::iso286_fit(d, &fit).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': iso286_fit: '{fit}' at Ø{d} — supported fits are H7/g6, H7/h6, H7/k6, H7/n6, H7/p6, H7/s6, H8/f7 for 0 < d ≤ 120 mm"),
				)
			})?;
			Ok(Outcome::measures(json!({
				"hole": [f.hole.0, f.hole.1],
				"shaft": [f.shaft.0, f.shaft.1],
				"clearance": [f.clearance.0, f.clearance.1],
			})))
		}
		OpKind::HeatsetSpec { m } => {
			let spec = parts::heatset_spec(m).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': heatset_spec: M{m} is not a heat-set insert size ({SMALL_SIZES_M2_M6})"),
				)
			})?;
			// pocket/boss sizing rules per `heatset_insert_boss` (documented there):
			// pocket depth = insert length + 1 mm melt room; boss Ø = 2 × pilot.
			Ok(Outcome::measures(json!({
				"m": spec.m,
				"pilot_d": spec.pilot_d,
				"insert_length": spec.length,
				"pocket_depth": spec.length + 1.0,
				"boss_d": 2.0 * spec.pilot_d,
			})))
		}
		OpKind::MetricCordGland { cord_d } => {
			let g = parts::metric_cord_gland(cord_d).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': metric_cord_gland: Ø{cord_d} is not a stocked metric cord size (supported: {METRIC_CORD_SIZES})"),
				)
			})?;
			Ok(Outcome::measures(json!({
				"gland_depth": g.gland_depth,
				"groove_width": g.groove_width,
				"squeeze": (g.cord_d - g.gland_depth) / g.cord_d,
				"fill": std::f64::consts::PI * (g.cord_d * 0.5) * (g.cord_d * 0.5) / (g.gland_depth * g.groove_width),
			})))
		}
		OpKind::RacetrackCordLength { x_len, y_len, corner_r } => {
			let cord_length = parts::racetrack_cord_length(x_len, y_len, corner_r).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': racetrack_cord_length: needs positive finite sides with 2·corner_r ({}) within both ({x_len} × {y_len})", 2.0 * corner_r),
				)
			})?;
			Ok(Outcome::measures(json!({ "cord_length": cord_length })))
		}
		#[cfg(feature = "catalog")]
		OpKind::PipeThreadG { designation } => {
			let g = parts::g_thread_spec(&designation).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': pipe_thread_g: '{designation}' is not stocked (G1/8, G1/4, G3/8, G1/2)"),
				)
			})?;
			Ok(Outcome::measures(json!({
				"major_d": g.major_d,
				"tpi": g.tpi,
				"pitch": g.pitch,
				"tap_drill_d": g.tap_drill_d,
			})))
		}

		_ => unreachable!("ops::designmath: op routed to the wrong family"),
	}
}
