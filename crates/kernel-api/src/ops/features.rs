// Copyright (c) LMCAD. Licensed under the MIT License.

//! Modelling features and rigid transforms: the witness-selected edge features
//! (`fillet_edge_near`, `chamfer_edge_near`, `fillet_circular_rim`), the
//! placements (`translate`, the axis rotations, the general rigid `pose`, the
//! orientation-safe `mirror`) and the clone-union `linear_pattern` /
//! `polar_pattern`.

use std::collections::{BTreeMap, BTreeSet};

use kernel_brep::math::DAffine3;

use crate::interp::{err, fetch_solid, EnvValue, Outcome};
use crate::program::OpKind;
use crate::report::{ErrorKind, OpError};

use super::support::{bind_solid, dv3, map_fillet_error, pattern_guard, resolved_edge_measures, snap_rotation, witness_edge};

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
		OpKind::FilletEdgeNear { input, witness, radius, max_distance } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let w = dv3(witness);
			let (name, distance, limit) = witness_edge(op_id, s, w, max_distance)?;
			let solid = kernel_brep::fillet_edge_near(s, name, radius, w).map_err(|e| map_fillet_error(op_id, "fillet_edge_near", e))?;
			let mut outcome = bind_solid(op_id, "fillet_edge_near", solid)?;
			outcome.measures = Some(resolved_edge_measures(name, distance, limit));
			Ok(outcome)
		}
		OpKind::ChamferEdgeNear { input, witness, radius, max_distance } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let w = dv3(witness);
			let (name, distance, limit) = witness_edge(op_id, s, w, max_distance)?;
			let solid = kernel_brep::chamfer_edge_near(s, name, radius, w).map_err(|e| map_fillet_error(op_id, "chamfer_edge_near", e))?;
			let mut outcome = bind_solid(op_id, "chamfer_edge_near", solid)?;
			outcome.measures = Some(resolved_edge_measures(name, distance, limit));
			Ok(outcome)
		}
		OpKind::FilletCircularRim { input, witness, radius, arc_segments } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let solid = kernel_brep::fillet_circular_rim(s, dv3(witness), radius, arc_segments).ok_or_else(|| {
				err(
					ErrorKind::FeatureFailed,
					format!(
						"op '{op_id}': no fillable circular rim near witness [{}, {}, {}] — the rim must be a convex cylinder-wall/planar-cap ring and the radius must fit (see API.md)",
						witness[0], witness[1], witness[2]
					),
				)
			})?;
			bind_solid(op_id, "fillet_circular_rim", solid)
		}
		OpKind::Translate { input, offset } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			bind_solid(op_id, "translate", s.transformed(DAffine3::from_translation(dv3(offset))))
		}
		OpKind::RotateZ { input, degrees } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			bind_solid(op_id, "rotate_z", s.transformed(snap_rotation(DAffine3::from_rotation_z(degrees.to_radians()))))
		}
		OpKind::Pose { input, translate, rotate } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			if translate.is_none() && rotate.is_none() {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': pose needs 'translate' and/or 'rotate' — an empty pose would be a no-op"),
				));
			}
			let mut m = DAffine3::IDENTITY;
			if let Some(r) = rotate {
				let Some(axis) = dv3(r.axis).try_normalize() else {
					return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': rotate.axis must be a non-zero finite vector")));
				};
				if !r.degrees.is_finite() {
					return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': rotate.degrees must be finite")));
				}
				let center = dv3(r.center);
				m = DAffine3::from_translation(center)
					* snap_rotation(DAffine3::from_axis_angle(axis, r.degrees.to_radians()))
					* DAffine3::from_translation(-center);
			}
			if let Some(t) = translate {
				m = DAffine3::from_translation(dv3(t)) * m;
			}
			bind_solid(op_id, "pose", s.transformed(m))
		}
		OpKind::RotateX { input, degrees } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			if !degrees.is_finite() {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': degrees must be finite")));
			}
			bind_solid(op_id, "rotate_x", s.transformed(snap_rotation(DAffine3::from_rotation_x(degrees.to_radians()))))
		}
		OpKind::RotateY { input, degrees } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			if !degrees.is_finite() {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': degrees must be finite")));
			}
			bind_solid(op_id, "rotate_y", s.transformed(snap_rotation(DAffine3::from_rotation_y(degrees.to_radians()))))
		}
		OpKind::Mirror { input, plane } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let p = dv3(plane.point);
			let n = dv3(plane.normal);
			if !p.is_finite() {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': plane.point must be finite")));
			}
			// The kernel's `mirrored` silently returns an unchanged clone for a
			// degenerate normal — reject it loudly here instead.
			if !(n.is_finite() && n.length_squared() > f64::EPSILON) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': plane.normal must be a non-zero finite vector")));
			}
			// Orientation-safe by construction: `Solid::mirrored` rebuilds every
			// face loop reversed, so the reflection is a valid outward solid (a raw
			// negative-determinant `transformed` would leave it inside-out).
			bind_solid(op_id, "mirror", s.mirrored(p, n))
		}
		OpKind::LinearPattern { input, count, step } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			pattern_guard(op_id, "linear_pattern", count, s.face_count())?;
			let st = dv3(step);
			if !(st.is_finite() && st.length_squared() > 0.0) {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': linear_pattern: step must be a non-zero finite vector — a zero step stacks every clone onto the original (a coincident-face degeneracy)"),
				));
			}
			let mut acc = s.clone();
			for i in 1..count {
				acc = kernel_brep::union(&acc, &s.transformed(DAffine3::from_translation(st * i as f64)));
			}
			bind_solid(op_id, "linear_pattern", acc)
		}
		OpKind::PolarPattern { input, count, center, axis, step_deg } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			pattern_guard(op_id, "polar_pattern", count, s.face_count())?;
			let Some(ax) = dv3(axis).try_normalize() else {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': polar_pattern: axis must be a non-zero finite vector")));
			};
			let c = dv3(center);
			if !c.is_finite() {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': polar_pattern: center must be finite")));
			}
			let step = step_deg.unwrap_or(360.0 / count as f64);
			if !step.is_finite() || step % 360.0 == 0.0 {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': polar_pattern: step_deg ({step}) must be finite and not a multiple of 360° — coincident clones are degenerate"),
				));
			}
			let mut acc = s.clone();
			for k in 1..count {
				let m = DAffine3::from_translation(c)
					* snap_rotation(DAffine3::from_axis_angle(ax, (step * k as f64).to_radians()))
					* DAffine3::from_translation(-c);
				acc = kernel_brep::union(&acc, &s.transformed(m));
			}
			bind_solid(op_id, "polar_pattern", acc)
		}

		_ => unreachable!("ops::features: op routed to the wrong family"),
	}
}
