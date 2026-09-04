// Copyright (c) LMCAD. Licensed under the MIT License.

//! Modelled ISO threads: `thread_spec` (the ISO 68-1 numbers), `thread_ridge` (the
//! exact ridge geometry) and `export_threaded` (fused or cut through the voxel
//! half, gated on the volume actually changing).

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

use kernel_core::check_mesh;
use kernel_implicit::{mesh_boolean_implicit, BoolOp};
use kernel_model::{parts, watertight_mesh_of};
use serde_json::json;

use crate::interp::{err, fetch_solid, EnvValue, Outcome};
use crate::program::OpKind;
use crate::report::{ErrorKind, OpError};

use super::meshio::{merge_soup, write_mesh_healed};
use super::support::{bind_solid, grid_guard, size_err, FASTENER_SIZES};

/// Cap on the helical turns a thread op will loft (the ridge is stitched at
/// 96 stations per turn, so unbounded turns would be an allocation hazard).
pub(crate) const MAX_THREAD_TURNS: f64 = 200.0;

/// Radial crest clearance (mm) of an `export_threaded` INTERNAL cut: the
/// male-profile ridge is enlarged to crest Ø `m + 2 × this` before being
/// subtracted from the bore wall — the documented print-practical female
/// approximation (NOT the ISO D1/D4 basic female form).
pub(crate) const INTERNAL_CREST_CLEARANCE: f64 = 0.2;

/// The ISO 261 coarse pitch for nominal Ø `m`, or a structured error naming the
/// supported table sizes.
pub(crate) fn iso_pitch(op_id: &str, what: &str, m: f64) -> Result<f64, OpError> {
	parts::iso_coarse_pitch(m).ok_or_else(|| size_err(op_id, what, "ISO 261 coarse-pitch", m, FASTENER_SIZES))
}

/// Reject a degenerate or allocation-hostile threaded span before any loft.
pub(crate) fn thread_turns_guard(op_id: &str, what: &str, length: f64, pitch: f64) -> Result<(), OpError> {
	if !(length.is_finite() && length > 0.0 && pitch.is_finite() && pitch > 0.0) {
		return Err(err(
			ErrorKind::InvalidParam,
			format!("op '{op_id}': {what}: length ({length}) and pitch ({pitch}) must be positive and finite"),
		));
	}
	let turns = length / pitch;
	if turns > MAX_THREAD_TURNS {
		return Err(err(
			ErrorKind::InvalidParam,
			format!(
				"op '{op_id}': {what}: {turns:.0} turns (length/pitch) exceeds the cap {MAX_THREAD_TURNS:.0} — the 96-station-per-turn loft would be enormous; thread a shorter span"
			),
		));
	}
	Ok(())
}

/// Execute one op of this family. The dispatch table in [`crate::interp`]
/// routes exactly the variants matched below, so the catch-all is dead code
/// kept only to satisfy the compiler.
pub(crate) fn exec(
	op_id: &str,
	env: &mut BTreeMap<String, EnvValue>,
	all_ids: &BTreeSet<String>,
	out_dir: &Path,
	kind: OpKind,
) -> Result<Outcome, OpError> {
	match kind {
		OpKind::ThreadSpec { m } => {
			// Measures-only table lookup (the `pipe_thread_g` pattern): the ISO 261
			// coarse pitch plus the ISO 68-1 derived dimensions a designer needs.
			let pitch = iso_pitch(op_id, "thread_spec", m)?;
			let h = 3.0_f64.sqrt() * 0.5 * pitch; // ISO 68-1 fundamental triangle height
			Ok(Outcome::measures(json!({
				"m": m,
				"pitch": pitch,
				"h": h,
				// basic minor Ø: crests − 2 × (5/8)H, the kernel ridge's root-flat Ø
				"minor_d": m - 1.25 * h,
				// the standard tap-drill rule Ø = m − pitch
				"tap_drill_d": m - pitch,
			})))
		}
		OpKind::ThreadRidge { m, major_d, pitch, z0, length } => {
			// The exact ISO 68-1 ridge solid, bound to the environment (it validates —
			// closed, manifold, genus 0). Its exact union with a shank SELF-INTERSECTS
			// by design (the root is buried P/4 into the shank): fuse via
			// `export_threaded`, never the exact `union` op.
			let (d, p) = match (m, major_d, pitch) {
				(Some(m), None, None) => (m, iso_pitch(op_id, "thread_ridge", m)?),
				(None, Some(d), Some(p)) => (d, p),
				_ => {
					return Err(err(
						ErrorKind::InvalidParam,
						format!("op '{op_id}': thread_ridge: give either 'm' (ISO coarse, {FASTENER_SIZES}) or BOTH 'major_d' and 'pitch' — not a mixture"),
					));
				}
			};
			thread_turns_guard(op_id, "thread_ridge", length, p)?;
			if !z0.is_finite() {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': thread_ridge: z0 must be finite")));
			}
			let solid = parts::iso_thread_solid(d, p, z0, length).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!(
						"op '{op_id}': thread_ridge: degenerate thread — major_d ({d}), pitch ({p}) and length ({length}) must be positive finite with the buried root radius still positive (the pitch is too large for the diameter)"
					),
				)
			})?;
			let h = 3.0_f64.sqrt() * 0.5 * p;
			let measures = json!({
				"major_d": d,
				"pitch": p,
				"minor_d": d - 1.25 * h,
				"z0": z0,
				"length": length,
				"turns": length / p,
			});
			Ok(Outcome { measures: Some(measures), ..bind_solid(op_id, "thread_ridge", solid)? })
		}
		OpKind::ExportThreaded { input, m, z0, length, internal, voxel, file } => {
			// Thread a bound body through the VOXEL half — the proven hybrid route,
			// because the exact union(body, ridge) self-intersects and no planar
			// arrangement can stitch it. External: merge the tessellation soups and
			// heal via the winding-number SDF (route "voxel_healed"). Internal: voxel-
			// subtract an oversized male ridge from the bore wall (route
			// "voxel_implicit") — a print-practical approximation of a female thread,
			// NOT the ISO D1/D4 form (documented in API.md). The thread axis is world
			// +Z through the origin: place the body's shank/bore there first.
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let pitch = iso_pitch(op_id, "export_threaded", m)?;
			thread_turns_guard(op_id, "export_threaded", length, pitch)?;
			if !z0.is_finite() {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': z0 must be finite")));
			}
			let voxel = voxel.unwrap_or(pitch / 8.0);
			if !(voxel.is_finite() && voxel > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': voxel must be a positive voxel size in mm")));
			}
			// VOXEL GUARD (deterministic): a lattice coarser than pitch/6 cannot
			// resolve the ISO profile — it smears the crests into a smooth band and
			// the "thread" silently becomes decoration. Refused, never degraded.
			if voxel > pitch / 6.0 {
				return Err(err(
					ErrorKind::InvalidParam,
					format!(
						"op '{op_id}': voxel {voxel} mm is coarser than pitch/6 ({:.4} mm for the M{m} pitch {pitch}) — the grid would smear the thread crests; use voxel ≤ pitch/6 (the default is pitch/8)",
						pitch / 6.0
					),
				));
			}
			let ridge_d = if internal { m + 2.0 * INTERNAL_CREST_CLEARANCE } else { m };
			let ridge = parts::iso_thread_solid(ridge_d, pitch, z0, length).ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': export_threaded: the M{m} ridge is degenerate over z0 {z0}, length {length} — see thread_ridge"),
				)
			})?;
			// Raw exact tessellations: the winding-number SDF consumes soups directly.
			let body_mesh = kernel_brep::tessellate_adaptive_tol(s, 0.01);
			let ridge_mesh = kernel_brep::tessellate_adaptive_tol(&ridge, 0.01);
			// Deterministic misplacement refusal: a thread that does not even overlap
			// the body's bounding box cannot fuse with (or cut) it — a floating ridge
			// would still pass a naive volume check as its own shell, so this is
			// caught HERE, not left to the delta guard.
			if !body_mesh.aabb().intersection(ridge_mesh.aabb()).is_valid() {
				return Err(err(
					ErrorKind::InvalidParam,
					format!(
						"op '{op_id}': the M{m} thread span (z {z0}..{}, crest Ø{ridge_d}) does not overlap the body's bounding box — the thread axis is world +Z through the origin; pose/translate the body onto it first",
						z0 + length
					),
				));
			}
			let domain = body_mesh.aabb().union(ridge_mesh.aabb()).pad(2.0 * voxel as f32);
			grid_guard(op_id, "export_threaded", domain, voxel)?;
			// The body alone, healed at the SAME voxel, is the volume baseline — the
			// in-tree regression's guard (voxel noise cancels between the two heals).
			let baseline = watertight_mesh_of(&body_mesh, voxel as f32).signed_volume();
			let (mesh, route) = if internal {
				(mesh_boolean_implicit(&body_mesh, &ridge_mesh, BoolOp::Difference, voxel), "voxel_implicit")
			} else {
				let mut soup = body_mesh.clone();
				merge_soup(&mut soup, &ridge_mesh);
				(watertight_mesh_of(&soup, voxel as f32), "voxel_healed")
			};
			let report = check_mesh(&mesh);
			if mesh.triangle_count() == 0 || !mesh.is_watertight() || report.non_manifold_edges > 0 {
				return Err(err(
					ErrorKind::InvalidGeometry,
					format!(
						"op '{op_id}': the threaded result did not mesh watertight at voxel {voxel} (triangles={}, watertight={}, non_manifold_edges={}) — refine the voxel or check the body placement on the +Z axis",
						mesh.triangle_count(),
						mesh.is_watertight(),
						report.non_manifold_edges
					),
				));
			}
			let volume = mesh.signed_volume();
			let delta = volume - baseline;
			// The regression guard, asserted per direction: an external thread MUST add
			// material, an internal one MUST remove it — a zero delta means the ridge
			// missed the body (wrong axis placement, bore too wide, shank too thin).
			if !internal && delta <= 0.0 {
				return Err(err(
					ErrorKind::InvalidGeometry,
					format!(
						"op '{op_id}': the external thread added no material (volume delta {delta:.3} mm³) — the ridge does not overlap the body; the shank must sit on the +Z axis through the origin and reach the ridge's buried base Ø{:.3}",
						ridge_d - 1.25 * (3.0_f64.sqrt() * 0.5 * pitch) - 0.5 * pitch
					),
				));
			}
			if internal && delta >= 0.0 {
				return Err(err(
					ErrorKind::InvalidGeometry,
					format!(
						"op '{op_id}': the internal thread removed no material (volume delta {delta:.3} mm³) — the ridge (crests Ø{ridge_d}) does not reach the bore wall; the bore must sit on the +Z axis through the origin with Ø below {ridge_d}"
					),
				));
			}
			let path = write_mesh_healed(op_id, out_dir, &file, &mesh)?;
			Ok(Outcome {
				value: Some(EnvValue::Mesh(mesh.clone())),
				measures: Some(json!({
					"route": route,
					"m": m,
					"pitch": pitch,
					"internal": internal,
					"voxel": voxel,
					"triangles": mesh.triangle_count(),
					"watertight": true,
					"volume": volume,
					"volume_delta_vs_body": delta,
				})),
				file: Some(path),
			})
		}
		_ => unreachable!("ops::threads: op routed to the wrong family"),
	}
}
