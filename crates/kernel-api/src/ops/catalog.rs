// Copyright (c) LMCAD. Licensed under the MIT License.

//! The standard hardware catalog: gears and racks, fasteners and screws, pins and
//! circlips, pulleys and sprockets, shafts and keys, springs, bearings, couplings,
//! linear motion, extrusion stock, O-rings, motors and mount plates. Each op
//! returns modelled geometry off a published table and refuses a size the table
//! does not carry.

use kernel_model::parts;

use crate::interp::{err, Outcome};
use crate::program::OpKind;
use crate::report::{ErrorKind, OpError};

use super::support::{bind_solid, size_err, AS568_DASHES, DIN471_SIZES, FASTENER_SIZES, METRIC_CORD_SIZES, SCREW_SIZES_M3_M12};
#[cfg(feature = "catalog")]
use super::support::{
	CLAMP_COUPLING_BORES, DIN472_SIZES, JAW_COUPLING_SIZES, NEMA_FRAMES, SET_SCREW_COUPLING_BORES, SMALL_SIZES_M2_M6,
};

/// Execute one op of this family. The dispatch table in [`crate::interp`]
/// routes exactly the variants matched below, so the catch-all is dead code
/// kept only to satisfy the compiler.
pub(crate) fn exec(op_id: &str, kind: OpKind) -> Result<Outcome, OpError> {
	match kind {
		OpKind::SpurGear { module, teeth, face_width, bore, pressure_angle_deg, keyway } => {
			let key = if keyway {
				Some(parts::din6885_key_size(bore).ok_or_else(|| {
					err(
						ErrorKind::InvalidParam,
						format!("op '{op_id}': spur_gear: no DIN 6885-1 key size for a {bore} mm bore (table covers over 6 up to 75 mm)"),
					)
				})?)
			} else {
				None
			};
			bind_solid(op_id, "spur_gear", parts::spur_gear(module, teeth, face_width, bore, pressure_angle_deg, key))
		}
		#[cfg(feature = "catalog")]
		OpKind::HexBolt { m, length } => {
			let solid = parts::hex_bolt_iso4017(m, length).ok_or_else(|| size_err(op_id, "hex_bolt", "ISO 4017", m, FASTENER_SIZES))?;
			bind_solid(op_id, "hex_bolt", solid)
		}
		OpKind::HexNut { m } => {
			let solid = parts::hex_nut_iso4032(m).ok_or_else(|| size_err(op_id, "hex_nut", "ISO 4032", m, FASTENER_SIZES))?;
			bind_solid(op_id, "hex_nut", solid)
		}
		OpKind::Washer { m } => {
			let solid = parts::washer_iso7089(m).ok_or_else(|| size_err(op_id, "washer", "ISO 7089", m, FASTENER_SIZES))?;
			bind_solid(op_id, "washer", solid)
		}
		OpKind::SocketHeadCapScrew { m, length } => {
			let solid = parts::socket_head_cap_screw(m, length)
				.ok_or_else(|| size_err(op_id, "socket_head_cap_screw", "DIN 912", m, FASTENER_SIZES))?;
			bind_solid(op_id, "socket_head_cap_screw", solid)
		}
		#[cfg(feature = "catalog")]
		OpKind::Gt2Pulley { teeth, belt_width, bore, flanged } => {
			bind_solid(op_id, "gt2_pulley", parts::gt2_pulley(teeth, belt_width, bore, flanged))
		}
		#[cfg(feature = "catalog")]
		OpKind::ChainSprocket { pitch, roller_d, teeth, bore } => {
			bind_solid(op_id, "chain_sprocket", parts::chain_sprocket(pitch, roller_d, teeth, bore))
		}
		#[cfg(feature = "catalog")]
		OpKind::Shaft { d, length, keyway } => {
			let keyway = match keyway {
				None => None,
				Some(spec) => {
					let size = parts::din6885_key_size(d).ok_or_else(|| {
						err(
							ErrorKind::InvalidParam,
							format!("op '{op_id}': shaft: no DIN 6885-1 key size for a {d} mm shaft (table covers over 6 up to 75 mm)"),
						)
					})?;
					Some(parts::ShaftKeyway { size, length: spec.length, offset: spec.offset })
				}
			};
			bind_solid(op_id, "shaft", parts::shaft(d, length, keyway))
		}

		#[cfg(feature = "catalog")]
		OpKind::ParallelKey { b, h, l } => bind_solid(op_id, "parallel_key", parts::parallel_key(b, h, l)),
		OpKind::DowelPin { d, length } => {
			let solid = parts::dowel_pin(d, length).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': dowel_pin: Ø{d} ×{length} — Ø must be an ISO 2338 size (1, 1.5, 2, 2.5, 3, 4, 5, 6, 8, 10, 12) and the length must exceed the two 0.2·d chamfers"),
				)
			})?;
			bind_solid(op_id, "dowel_pin", solid)
		}
		OpKind::CirclipExternal { shaft_d } => {
			let solid = parts::circlip_external(shaft_d).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': circlip_external: Ø{shaft_d} is not in the DIN 471 table (supported: {DIN471_SIZES})"),
				)
			})?;
			bind_solid(op_id, "circlip_external", solid)
		}
		#[cfg(feature = "catalog")]
		OpKind::CirclipInternal { bore_d } => {
			let solid = parts::circlip_internal(bore_d).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': circlip_internal: Ø{bore_d} is not in the DIN 472 table (supported: {DIN472_SIZES})"),
				)
			})?;
			bind_solid(op_id, "circlip_internal", solid)
		}
		OpKind::FlatHeadScrew { m, length } => {
			let solid = parts::flat_head_screw(m, length).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': flat_head_screw: M{m}×{length} — M{m} must be an ISO 10642 size ({FASTENER_SIZES}) and the overall length must contain the head cone and socket"),
				)
			})?;
			bind_solid(op_id, "flat_head_screw", solid)
		}
		OpKind::ButtonHeadScrew { m, length } => {
			let solid = parts::button_head_screw(m, length).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': button_head_screw: M{m}×{length} — M{m} must be an ISO 7380 size ({SCREW_SIZES_M3_M12}) and the length positive"),
				)
			})?;
			bind_solid(op_id, "button_head_screw", solid)
		}
		OpKind::SetScrew { m, length } => {
			let solid = parts::set_screw(m, length).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': set_screw: M{m}×{length} — M{m} must be a DIN 916 size ({SCREW_SIZES_M3_M12}) and the length must hold the cup, socket and a 0.5 mm web"),
				)
			})?;
			bind_solid(op_id, "set_screw", solid)
		}
		OpKind::LockNut { m } => {
			let solid = parts::lock_nut(m).ok_or_else(|| size_err(op_id, "lock_nut", "DIN 985", m, FASTENER_SIZES))?;
			bind_solid(op_id, "lock_nut", solid)
		}
		#[cfg(feature = "catalog")]
		OpKind::ThreadedRod { m, length } => {
			let solid = parts::threaded_rod(m, length).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': threaded_rod: M{m}×{length} — M{m} must be an ISO 261 coarse size ({FASTENER_SIZES}) and the length must exceed the two half-pitch chamfers"),
				)
			})?;
			bind_solid(op_id, "threaded_rod", solid)
		}
		#[cfg(feature = "catalog")]
		OpKind::Standoff { m, length } => {
			let solid = parts::standoff(m, length).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': standoff: M{m}×{length} — M{m} must be a standoff size ({SMALL_SIZES_M2_M6}) and the length positive"),
				)
			})?;
			bind_solid(op_id, "standoff", solid)
		}
		OpKind::CompressionSpring { wire_d, outer_d, pitch, turns } => {
			let solid = parts::compression_spring(wire_d, outer_d, pitch, turns).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': compression_spring: needs wire_d > 0, outer_d > 2·wire_d, turns > 0 and pitch > wire_d (touching coils would self-intersect)"),
				)
			})?;
			bind_solid(op_id, "compression_spring", solid)
		}
		#[cfg(feature = "catalog")]
		OpKind::Extrusion2020 { length } => bind_solid(op_id, "extrusion_2020", parts::extrusion_2020(length)),
		#[cfg(feature = "catalog")]
		OpKind::Extrusion3030 { length } => bind_solid(op_id, "extrusion_3030", parts::extrusion_3030(length)),
		#[cfg(feature = "catalog")]
		OpKind::Tnut2020 {} => bind_solid(op_id, "tnut_2020", parts::tnut_2020()),
		OpKind::ORing { dash } => {
			let solid = parts::o_ring(dash).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': o_ring: dash -{dash} is not an AS568 table size (supported: {AS568_DASHES})"),
				)
			})?;
			bind_solid(op_id, "o_ring", solid)
		}
		OpKind::ORingCord { ring_id, cord_d } => {
			let solid = parts::o_ring_cord(ring_id, cord_d).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': o_ring_cord: needs a positive finite ring_id and a stocked metric cord Ø ({METRIC_CORD_SIZES}); got Ø{ring_id} × {cord_d}"),
				)
			})?;
			bind_solid(op_id, "o_ring_cord", solid)
		}
		#[cfg(feature = "catalog")]
		OpKind::JawCouplingHub { od, bore } => {
			let solid = parts::jaw_coupling_hub(od, bore).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': jaw_coupling_hub: OD {od} must be a coupling size ({JAW_COUPLING_SIZES}) and bore Ø{bore} within that row's range"),
				)
			})?;
			bind_solid(op_id, "jaw_coupling_hub", solid)
		}
		#[cfg(feature = "catalog")]
		OpKind::JawCouplingSpider { od } => {
			let solid = parts::jaw_coupling_spider(od).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': jaw_coupling_spider: OD {od} is not a coupling size ({JAW_COUPLING_SIZES})"),
				)
			})?;
			bind_solid(op_id, "jaw_coupling_spider", solid)
		}
		#[cfg(feature = "catalog")]
		OpKind::SetScrewCoupling { bore1, bore2 } => {
			let solid = parts::set_screw_coupling(bore1, bore2).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': set_screw_coupling: both bores (Ø{bore1} × Ø{bore2}) must be stocked sizes ({SET_SCREW_COUPLING_BORES})"),
				)
			})?;
			bind_solid(op_id, "set_screw_coupling", solid)
		}
		#[cfg(feature = "catalog")]
		OpKind::ClampCoupling { bore1, bore2 } => {
			let solid = parts::clamp_coupling(bore1, bore2).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': clamp_coupling: both bores (Ø{bore1} × Ø{bore2}) must be stocked sizes ({CLAMP_COUPLING_BORES})"),
				)
			})?;
			bind_solid(op_id, "clamp_coupling", solid)
		}
		#[cfg(feature = "catalog")]
		OpKind::LinearBearingLmuu { bore } => {
			let solid = parts::linear_bearing_lmuu(bore).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': linear_bearing_lmuu: bore Ø{bore} must be 8 (LM8UU) or 12 (LM12UU)"),
				)
			})?;
			bind_solid(op_id, "linear_bearing_lmuu", solid)
		}
		#[cfg(feature = "catalog")]
		OpKind::Sc8uuBlock {} => bind_solid(op_id, "sc8uu_block", parts::sc8uu_block()),
		#[cfg(feature = "catalog")]
		OpKind::ShaftSupportSk8 {} => bind_solid(op_id, "shaft_support_sk8", parts::shaft_support_sk8()),
		#[cfg(feature = "catalog")]
		OpKind::ShaftSupportShf8 {} => bind_solid(op_id, "shaft_support_shf8", parts::shaft_support_shf8()),
		#[cfg(feature = "catalog")]
		OpKind::Mgn12Rail { length } => {
			let solid = parts::mgn12_rail(length).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': mgn12_rail: length ({length}) must be finite and at least one 25 mm hole pitch"),
				)
			})?;
			bind_solid(op_id, "mgn12_rail", solid)
		}
		#[cfg(feature = "catalog")]
		OpKind::Mgn12Carriage {} => bind_solid(op_id, "mgn12_carriage", parts::mgn12_carriage()),
		OpKind::DeepGrooveBearing { designation } => {
			let solid = parts::deep_groove_bearing(&designation).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': deep_groove_bearing: '{designation}' is not in the seat table (603, 608, 625, 688, 6000, 6001, 6804)"),
				)
			})?;
			bind_solid(op_id, "deep_groove_bearing", solid)
		}
		OpKind::FlangedBearing { designation } => {
			let solid = parts::flanged_bearing(&designation).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': flanged_bearing: '{designation}' must be F608 or F623"),
				)
			})?;
			bind_solid(op_id, "flanged_bearing", solid)
		}
		#[cfg(feature = "catalog")]
		OpKind::ThrustBearing { designation } => {
			let solid = parts::thrust_bearing(&designation).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': thrust_bearing: '{designation}' must be 51100 or 51101"),
				)
			})?;
			bind_solid(op_id, "thrust_bearing", solid)
		}
		#[cfg(feature = "catalog")]
		OpKind::Kp08PillowBlock {} => bind_solid(op_id, "kp08_pillow_block", parts::kp08_pillow_block()),
		#[cfg(feature = "catalog")]
		OpKind::PipeBossG { designation, wall, length } => {
			let solid = parts::pipe_boss_g(&designation, wall, length).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': pipe_boss_g: '{designation}' must be G1/8…G1/2, wall ({wall}) ≥ 1, length ({length}) past chamfer + pitch"),
				)
			})?;
			bind_solid(op_id, "pipe_boss_g", solid)
		}
		#[cfg(feature = "catalog")]
		OpKind::HoseBarb { hose_id, barbs } => {
			let solid = parts::hose_barb(hose_id, barbs).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': hose_barb: hose_id (Ø{hose_id}) must be positive finite and barbs ({barbs}) ≥ 1"),
				)
			})?;
			bind_solid(op_id, "hose_barb", solid)
		}
		#[cfg(feature = "catalog")]
		OpKind::ShoulderBolt { shoulder_d, shoulder_len } => {
			let solid = parts::shoulder_bolt(shoulder_d, shoulder_len).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': shoulder_bolt: shoulder Ø{shoulder_d} must be an ISO 7379 size (6.5, 8, 10, 13, 16) and shoulder_len ({shoulder_len}) positive finite"),
				)
			})?;
			bind_solid(op_id, "shoulder_bolt", solid)
		}
		#[cfg(feature = "catalog")]
		OpKind::SpringWasher { m } => {
			let solid = parts::spring_washer(m).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': spring_washer: M{m} is outside the DIN 127 B table (M3–M12)"),
				)
			})?;
			bind_solid(op_id, "spring_washer", solid)
		}
		#[cfg(feature = "catalog")]
		OpKind::LeadScrewTr8 { length, lead } => {
			let solid = parts::lead_screw_tr8(length, lead).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': lead_screw_tr8: lead {lead} must be a Tr8 variant (2, 4, 8 — all pitch 2) and length ({length}) > one pitch"),
				)
			})?;
			bind_solid(op_id, "lead_screw_tr8", solid)
		}
		#[cfg(feature = "catalog")]
		OpKind::LeadScrewNutTr8 {} => bind_solid(op_id, "lead_screw_nut_tr8", parts::lead_screw_nut_tr8()),
		#[cfg(feature = "catalog")]
		OpKind::NemaMotor { frame, body_len } => {
			let solid = parts::nema_motor(frame, body_len).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': nema_motor: frame {frame} must be a NEMA table size ({NEMA_FRAMES}) and body_len ({body_len}) positive"),
				)
			})?;
			bind_solid(op_id, "nema_motor", solid)
		}
		#[cfg(feature = "catalog")]
		OpKind::NemaMountPlate { frame, thickness, margin } => {
			let solid = parts::nema_mount_plate(frame, thickness, margin).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': nema_mount_plate: frame {frame} must be a NEMA table size ({NEMA_FRAMES}), thickness ({thickness}) positive and margin ({margin}) ≥ 0"),
				)
			})?;
			bind_solid(op_id, "nema_mount_plate", solid)
		}
		#[cfg(feature = "catalog")]
		OpKind::GearRack { module, length, width, pressure_angle_deg } => {
			let solid = parts::gear_rack(module, length, width, pressure_angle_deg).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': gear_rack: needs positive dimensions, a pressure angle in (0°, 32°), and a bar long enough for one whole tooth"),
				)
			})?;
			bind_solid(op_id, "gear_rack", solid)
		}
		#[cfg(feature = "catalog")]
		OpKind::InternalGear { module, teeth, face_width, rim_od, pressure_angle_deg } => {
			let solid = parts::internal_gear(module, teeth, face_width, rim_od, pressure_angle_deg).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': internal_gear: needs teeth ≥ 8, rim_od > m·(teeth + 2.5) (the root circle), positive dimensions, and a pressure angle low enough to keep the root land open"),
				)
			})?;
			bind_solid(op_id, "internal_gear", solid)
		}

		_ => unreachable!("ops::catalog: op routed to the wrong family"),
	}
}
