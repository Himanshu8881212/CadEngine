// Copyright (c) LMCAD. Licensed under the MIT License.

//! Solid primitives and the sweep family: the analytic shapes (`box`, `cylinder`,
//! `sphere`, `cone`, `torus`) plus `extrude` and its variants, `revolve`, `loft`
//! and `sweep`.

use kernel_brep::math::{DAffine3, DVec2, DVec3};

use crate::interp::{err, Outcome};
use crate::program::OpKind;
use crate::report::{ErrorKind, OpError};

use super::support::{align_z_to, bind_solid, dv3, profile2d};

/// Execute one op of this family. The dispatch table in [`crate::interp`]
/// routes exactly the variants matched below, so the catch-all is dead code
/// kept only to satisfy the compiler.
pub(crate) fn exec(op_id: &str, kind: OpKind) -> Result<Outcome, OpError> {
	match kind {
		OpKind::Box { min, max } => {
			// Reject a degenerate box (non-positive extent on any axis) up front — an inverted or
			// zero-thickness box is a user error, not something to silently normalize/build.
			if (0..3).any(|i| max[i] <= min[i]) {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': box has non-positive extent — max must exceed min on every axis (min={min:?}, max={max:?})"),
				));
			}
			bind_solid(op_id, "box", kernel_brep::cuboid(dv3(min), dv3(max)))
		}
		OpKind::Cylinder { base, axis, radius, height, segments } => {
			bind_solid(op_id, "cylinder", kernel_brep::cylinder(dv3(base), dv3(axis), radius, height, segments))
		}
		OpKind::Sphere { center, radius, u, v } => bind_solid(op_id, "sphere", kernel_brep::sphere(dv3(center), radius, u, v)),
		OpKind::Cone { base, axis, radius, height, segments, top_radius } => {
			let top = top_radius.unwrap_or(0.0);
			if !(top.is_finite() && top >= 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': top_radius must be a finite non-negative radius in mm")));
			}
			if top == 0.0 {
				return bind_solid(op_id, "cone", kernel_brep::cone(dv3(base), dv3(axis), radius, height, segments));
			}
			if !(radius.is_finite() && radius > 0.0 && height.is_finite() && height != 0.0) {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': a frustum needs a positive finite 'radius' and a non-zero finite 'height'"),
				));
			}
			if (top - radius).abs() <= 1e-12 * radius.abs().max(1.0) {
				return Err(err(
					ErrorKind::InvalidParam,
					format!(
						"op '{op_id}': top_radius {top} equals radius {radius} — that solid is a CYLINDER, not a frustum; use the 'cylinder' op (a cone surface with no apex is not representable)"
					),
				));
			}
			// A frustum is the revolution of the trapezoid (0,0)→(r,0)→(rt,h)→(0,h).
			// Reusing `revolve` is not a shortcut: it is what gives the lateral band
			// its exact `Surface::Cone` tag (and the caps their planes), so
			// `exact_volume` / `mass_properties` / STEP export stay analytic —
			// exactly as they are for the un-truncated `cone`.
			let profile = [DVec2::new(0.0, 0.0), DVec2::new(radius, 0.0), DVec2::new(top, height.abs()), DVec2::new(0.0, height.abs())];
			let solid = kernel_brep::revolve(&profile, segments.max(3));
			if solid.face_count() == 0 {
				return Err(err(
					ErrorKind::InvalidGeometry,
					format!("op '{op_id}': the frustum profile (radius {radius}, top_radius {top}, height {height}) does not revolve to a valid solid"),
				));
			}
			// `revolve` builds about +Z through the origin; place it on the requested
			// base/axis with the same conventions the `cone` op already uses (a
			// negative height puts the small end below the base plane).
			let ax = dv3(axis);
			let Some(dir) = (ax * height.signum()).try_normalize() else {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': axis must be a non-zero finite vector")));
			};
			let b = dv3(base);
			if !b.is_finite() {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': base must be finite")));
			}
			let m = DAffine3::from_translation(b) * DAffine3::from_mat3(align_z_to(dir));
			bind_solid(op_id, "cone", solid.transformed(m))
		}
		OpKind::Torus { center, axis, major, minor, ring_segments, tube_segments } => {
			bind_solid(op_id, "torus", kernel_brep::torus(dv3(center), dv3(axis), major, minor, ring_segments, tube_segments))
		}
		OpKind::Extrude { profile, height } => bind_solid(op_id, "extrude", kernel_brep::extrude(&profile2d(&profile), height)),
		OpKind::ExtrudeWithHoles { outer, holes, height } => {
			let holes: Vec<Vec<DVec2>> = holes.iter().map(|h| profile2d(h)).collect();
			bind_solid(op_id, "extrude_with_holes", kernel_brep::extrude_with_holes(&profile2d(&outer), &holes, height))
		}
		OpKind::ExtrudeTapered { profile, height, draft_deg } => {
			bind_solid(op_id, "extrude_tapered", kernel_brep::extrude_tapered(&profile2d(&profile), height, draft_deg.to_radians()))
		}
		OpKind::Revolve { profile, segments } => bind_solid(op_id, "revolve", kernel_brep::revolve(&profile2d(&profile), segments)),
		OpKind::Loft { sections } => {
			let secs: Vec<Vec<DVec3>> = sections.iter().map(|s| s.iter().map(|&p| dv3(p)).collect()).collect();
			match kernel_brep::loft_solid(&secs) {
				Some(s) => bind_solid(op_id, "loft", s),
				None => Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': 'loft' needs ≥2 sections of ≥3 points each, all the same length, with finite coordinates; see API.md"),
				)),
			}
		}
		OpKind::Sweep { profile, path } => {
			let prof: Vec<DVec3> = profile.iter().map(|&p| dv3(p)).collect();
			let pth: Vec<DVec3> = path.iter().map(|&p| dv3(p)).collect();
			match kernel_brep::sweep_solid(&prof, &pth) {
				Some(s) => bind_solid(op_id, "sweep", s),
				None => Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': 'sweep' needs a profile of ≥3 points and a path of ≥2 points, all finite; see API.md"),
				)),
			}
		}

		_ => unreachable!("ops::primitives: op routed to the wrong family"),
	}
}
