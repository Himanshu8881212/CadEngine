// Copyright (c) LMCAD. Licensed under the MIT License.

//! The standard feature cuts a catalog part mates into: heat-set bosses, circlip
//! and O-ring glands, the printable `teardrop_hole` / `bridged_counterbore`, board
//! and motor mounts, and the Tr8 nut trap.

use std::collections::{BTreeMap, BTreeSet};

use kernel_model::parts;
use serde_json::json;

use crate::interp::{err, fetch_solid, EnvValue, Outcome};
use crate::program::OpKind;
use crate::report::{ErrorKind, OpError};

use super::support::{bind_solid, dv3, METRIC_CORD_SIZES, SMALL_SIZES_M2_M6};
#[cfg(feature = "catalog")]
use super::support::{AS568_DASHES, DIN471_SIZES, DIN472_SIZES, NEMA_FRAMES, SERVO_MODELS};

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
		OpKind::HeatsetInsertBoss { input, at, axis, m } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid = parts::heatset_insert_boss(s, dv3(at), dv3(axis), m).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': heatset_insert_boss: M{m} must be a heat-set insert size ({SMALL_SIZES_M2_M6}) and the axis non-zero"),
				)
			})?;
			bind_solid(op_id, "heatset_insert_boss", solid)
		}
		#[cfg(feature = "catalog")]
		OpKind::CirclipGrooveExternal { input, at, axis, shaft_d } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid = parts::circlip_groove_external(s, dv3(at), dv3(axis), shaft_d).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': circlip_groove_external: Ø{shaft_d} must be a DIN 471 size ({DIN471_SIZES}) and the axis non-zero"),
				)
			})?;
			bind_solid(op_id, "circlip_groove_external", solid)
		}
		#[cfg(feature = "catalog")]
		OpKind::CirclipGrooveInternal { input, at, axis, bore_d } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid = parts::circlip_groove_internal(s, dv3(at), dv3(axis), bore_d).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': circlip_groove_internal: Ø{bore_d} must be a DIN 472 size ({DIN472_SIZES}) and the axis non-zero"),
				)
			})?;
			bind_solid(op_id, "circlip_groove_internal", solid)
		}
		#[cfg(feature = "catalog")]
		OpKind::ORingGroove { input, at, axis, dash } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid = parts::o_ring_groove(s, dv3(at), dv3(axis), dash).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': o_ring_groove: dash -{dash} must be an AS568 table size ({AS568_DASHES}) and the axis non-zero"),
				)
			})?;
			bind_solid(op_id, "o_ring_groove", solid)
		}
		OpKind::ORingFaceGland { input, at, axis, gland_center_d, cord_d } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid = parts::o_ring_face_gland(s, dv3(at), dv3(axis), gland_center_d, cord_d).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': o_ring_face_gland: cord Ø{cord_d} must be a stocked metric size ({METRIC_CORD_SIZES}), the axis non-zero, and gland_center_d (Ø{gland_center_d}) wider than the groove"),
				)
			})?;
			let outcome = bind_solid(op_id, "o_ring_face_gland", solid)?;
			// Echo the gland dimensions the table chose (the FRICTION #9 lesson:
			// report what the cut used, so the seal stack can be posed without
			// re-reading kernel tables).
			let g = parts::metric_cord_gland(cord_d).expect("cord validated above");
			Ok(Outcome {
				measures: Some(json!({
					"gland_depth": g.gland_depth,
					"groove_width": g.groove_width,
					"squeeze": (g.cord_d - g.gland_depth) / g.cord_d,
					"fill": std::f64::consts::PI * (g.cord_d * 0.5) * (g.cord_d * 0.5) / (g.gland_depth * g.groove_width),
					"cord_length": std::f64::consts::PI * gland_center_d,
				})),
				..outcome
			})
		}
		#[cfg(feature = "catalog")]
		OpKind::ORingFaceGlandRacetrack { input, at, axis, x_len, y_len, corner_r, cord_d } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid = parts::o_ring_face_gland_racetrack(s, dv3(at), dv3(axis), x_len, y_len, corner_r, cord_d).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': o_ring_face_gland_racetrack: cord Ø{cord_d} must be a stocked metric size ({METRIC_CORD_SIZES}), the axis non-zero, corner_r at least half the groove width, and 2·corner_r within both {x_len}×{y_len} sides"),
				)
			})?;
			let outcome = bind_solid(op_id, "o_ring_face_gland_racetrack", solid)?;
			let g = parts::metric_cord_gland(cord_d).expect("cord validated above");
			let cord_length = parts::racetrack_cord_length(x_len, y_len, corner_r).expect("path validated above");
			Ok(Outcome {
				measures: Some(json!({
					"gland_depth": g.gland_depth,
					"groove_width": g.groove_width,
					"squeeze": (g.cord_d - g.gland_depth) / g.cord_d,
					"fill": std::f64::consts::PI * (g.cord_d * 0.5) * (g.cord_d * 0.5) / (g.gland_depth * g.groove_width),
					"cord_length": cord_length,
				})),
				..outcome
			})
		}

		#[cfg(feature = "catalog")]
		OpKind::Pc4Port { input, at, axis, m, through } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid = parts::pc4_port_cut(s, dv3(at), dv3(axis), m, through).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': pc4_port: m ({m}) must be 6 or 10, the axis non-zero, and 'through' ({through}) past the pocket depth"),
				)
			})?;
			bind_solid(op_id, "pc4_port", solid)
		}
		OpKind::TeardropHole { input, at, axis, up, d, through } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid = parts::teardrop_hole(s, dv3(at), dv3(axis), dv3(up), d, through).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': teardrop_hole: needs a non-zero axis, 'up' not parallel to it, and positive finite d ({d}) / through ({through})"),
				)
			})?;
			bind_solid(op_id, "teardrop_hole", solid)
		}
		OpKind::BridgedCounterbore { input, at, axis, m, through, bridge } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid = parts::bridged_counterbore(s, dv3(at), dv3(axis), m, through, bridge).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': bridged_counterbore: M{m} must be M2–M12 with a positive bridge ({bridge}) and through ({through}) > pocket + bridge"),
				)
			})?;
			bind_solid(op_id, "bridged_counterbore", solid)
		}
		OpKind::BoardMount { input, at, axis, board } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid = parts::board_mount_cut(s, dv3(at), dv3(axis), &board).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': board_mount: '{board}' must be rpi, arduino_uno, vesa75 or vesa100 (with a non-zero axis)"),
				)
			})?;
			bind_solid(op_id, "board_mount", solid)
		}
		#[cfg(feature = "catalog")]
		OpKind::Tr8NutTrap { input, at, axis, through } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid = parts::tr8_nut_trap(s, dv3(at), dv3(axis), through).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': tr8_nut_trap: the axis must be non-zero and 'through' ({through}) must exceed the 3.7 mm flange recess"),
				)
			})?;
			bind_solid(op_id, "tr8_nut_trap", solid)
		}
		#[cfg(feature = "catalog")]
		OpKind::NemaMountCut { input, at, axis, frame, through } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid = parts::nema_mount_cut(s, dv3(at), dv3(axis), frame, through).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': nema_mount_cut: frame {frame} must be a NEMA table size ({NEMA_FRAMES}), the axis non-zero and 'through' ({through}) positive"),
				)
			})?;
			bind_solid(op_id, "nema_mount_cut", solid)
		}
		#[cfg(feature = "catalog")]
		OpKind::ServoPocket { input, at, axis, model, through } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid = parts::servo_pocket(s, dv3(at), dv3(axis), &model, through).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': servo_pocket: model '{model}' must be a servo table size ({SERVO_MODELS}), the axis non-zero and 'through' ({through}) positive"),
				)
			})?;
			bind_solid(op_id, "servo_pocket", solid)
		}

		_ => unreachable!("ops::cuts: op routed to the wrong family"),
	}
}
