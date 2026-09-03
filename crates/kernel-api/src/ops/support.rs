// Copyright (c) LMCAD. Licensed under the MIT License.

//! Helpers shared across the op families: the small geometry conversions, the
//! `bind_solid` validity gate every solid-producing op passes through, the
//! witness→edge resolver behind `fillet`/`chamfer`, the kernel-error mappers,
//! and the catalog size tables quoted in "size not in table" refusals.

use kernel_brep::holes::HoleError;
use kernel_brep::math::{DAffine3, DVec2, DVec3};
use kernel_brep::{FilletError, Solid};
use kernel_core::Aabb;
use kernel_model::SketchError;
use serde_json::{json, Value};

use crate::interp::{err, EnvValue, Outcome, MAX_GRID_CELLS, MAX_PATTERN_COUNT, MAX_PATTERN_FACES};
use crate::report::{ErrorKind, OpError};

/// A rotation taking +Z onto the unit vector `dir` (any rotation about `dir`
/// will do — the shapes placed with it are surfaces of revolution). Uses the
/// shortest-arc axis; the antipodal case gets an explicit 180° flip because the
/// cross product vanishes there.
pub(crate) fn align_z_to(dir: DVec3) -> kernel_brep::math::DMat3 {
	use kernel_brep::math::DMat3;
	let z = DVec3::Z;
	let c = z.dot(dir);
	if c > 1.0 - 1e-12 {
		return DMat3::IDENTITY;
	}
	if c < -1.0 + 1e-12 {
		return DMat3::from_rotation_x(std::f64::consts::PI);
	}
	let axis = z.cross(dir).normalize();
	DMat3::from_axis_angle(axis, c.clamp(-1.0, 1.0).acos())
}

/// DVec3 → JSON array, for entity descriptors.
pub(crate) fn v3a(v: DVec3) -> [f64; 3] {
	[v.x, v.y, v.z]
}

/// Centroid of a boundary polygon (a witness point on/near the face).
pub(crate) fn polygon_centroid(pts: &[DVec3]) -> DVec3 {
	if pts.is_empty() {
		return DVec3::ZERO;
	}
	pts.iter().fold(DVec3::ZERO, |a, &p| a + p) / pts.len() as f64
}

/// Newell area of a planar polygon (exact for planar faces; boundary-only for curved).
pub(crate) fn polygon_area(pts: &[DVec3]) -> f64 {
	if pts.len() < 3 {
		return 0.0;
	}
	let mut n = DVec3::ZERO;
	for i in 0..pts.len() {
		n += pts[i].cross(pts[(i + 1) % pts.len()]);
	}
	n.length() * 0.5
}

pub(crate) fn dv3(a: [f64; 3]) -> DVec3 {
	DVec3::new(a[0], a[1], a[2])
}

pub(crate) fn profile2d(points: &[[f64; 2]]) -> Vec<DVec2> {
	points.iter().map(|p| DVec2::new(p[0], p[1])).collect()
}

/// Gate every solid-producing op: an empty result is an `invalid_param` failure
/// (the kernel rejected degenerate input) and a non-valid result is an
/// `invalid_geometry` failure carrying the `Validity` details. A solid is bound
/// to the environment ONLY through this gate.
pub(crate) fn bind_solid(op_id: &str, what: &str, solid: Solid) -> Result<Outcome, OpError> {
	if solid.face_count() == 0 {
		return Err(err(
			ErrorKind::InvalidParam,
			format!("op '{op_id}': {what} produced an empty solid — degenerate input, parameters outside the op's documented domain, or an empty boolean result (e.g. a disjoint intersection); see API.md"),
		));
	}
	let v = kernel_brep::validate(&solid);
	if !v.is_valid() {
		return Err(err(
			ErrorKind::InvalidGeometry,
			format!(
				"op '{op_id}': {what} failed validate(): closed={} manifold={} genus={} euler_characteristic={} shells={} — refusing to bind an invalid solid",
				v.closed, v.manifold, v.genus, v.euler_characteristic, v.shells
			),
		));
	}
	Ok(Outcome { value: Some(EnvValue::Solid(solid)), measures: None, file: None })
}

/// Gate a pattern op's instance count: 2..=[`MAX_PATTERN_COUNT`] (the structural
/// `check_limits` pass already rejected larger counts before dispatch — this arm
/// repeats the ceiling so the invariant does not depend on field-name matching)
/// AND `count × per-clone face count` within the union fold's face budget.
pub(crate) fn pattern_guard(op_id: &str, what: &str, count: usize, clone_faces: usize) -> Result<(), OpError> {
	if count < 2 {
		return Err(err(
			ErrorKind::InvalidParam,
			format!("op '{op_id}': {what}: count must be at least 2 (a 1-pattern is a no-op — use the input id directly)"),
		));
	}
	if count as u64 > MAX_PATTERN_COUNT {
		return Err(err(
			ErrorKind::InvalidParam,
			format!("op '{op_id}': {what}: count {count} exceeds the safety cap {MAX_PATTERN_COUNT} — rejected before allocation"),
		));
	}
	let total = count.saturating_mul(clone_faces);
	if total > MAX_PATTERN_FACES {
		return Err(err(
			ErrorKind::InvalidParam,
			format!("op '{op_id}': {what}: count {count} × {clone_faces} faces per clone = {total} faces exceeds the pattern budget {MAX_PATTERN_FACES} — pattern a simpler solid or reduce the count"),
		));
	}
	Ok(())
}

/// Distance from `p` to the segment `a → b`.
pub(crate) fn point_segment_distance(p: DVec3, a: DVec3, b: DVec3) -> f64 {
	let ab = b - a;
	let len2 = ab.length_squared();
	if len2 <= f64::EPSILON {
		return (p - a).length();
	}
	let t = ((p - a).dot(ab) / len2).clamp(0.0, 1.0);
	(a + ab * t - p).length()
}

/// The named edge of `solid` nearest to `witness` plus its distance, or `None`
/// when the solid carries no edge names at all.
pub(crate) fn nearest_named_edge(solid: &Solid, witness: DVec3) -> Option<(kernel_brep::EdgeName, f64)> {
	let mut best: Option<(kernel_brep::EdgeName, f64)> = None;
	for e in solid.edges() {
		let Some(name) = solid.edge_name(e) else { continue };
		let he = *solid.half_edge(solid.edge(e).half_edge);
		let a = solid.position(he.origin);
		let b = solid.position(solid.half_edge(he.next).origin);
		let d = point_segment_distance(witness, a, b);
		if best.is_none_or(|(_, bd)| d < bd) {
			best = Some((name, d));
		}
	}
	best
}

/// Diagonal length of the solid's vertex bounding box (0 for an empty solid).
pub(crate) fn bbox_diagonal(solid: &Solid) -> f64 {
	let mut min = DVec3::splat(f64::INFINITY);
	let mut max = DVec3::splat(f64::NEG_INFINITY);
	for i in 0..solid.vertex_count() as u32 {
		let p = solid.position(kernel_brep::VertexId(i));
		min = min.min(p);
		max = max.max(p);
	}
	if min.x > max.x {
		return 0.0;
	}
	(max - min).length()
}

/// Resolve a fillet/chamfer witness to the nearest named edge, enforcing the
/// max-distance guard (default: 10% of the bounding-box diagonal) so a witness
/// that matches nothing is a structured failure, not a far-away surprise edge.
///
/// Returns the chosen [`kernel_brep::EdgeName`] together with the witness→edge
/// distance and the limit that was in effect — the raw material for the
/// `resolved_edge` receipt (which edge a spatial witness actually latched, and
/// how close the match was). Selection and the guard are unchanged; the extra
/// return values are only *recorded*, never acted on here.
pub(crate) fn witness_edge(
	op_id: &str,
	solid: &Solid,
	witness: DVec3,
	max_distance: Option<f64>,
) -> Result<(kernel_brep::EdgeName, f64, f64), OpError> {
	let Some((name, distance)) = nearest_named_edge(solid, witness) else {
		return Err(err(
			ErrorKind::FeatureFailed,
			format!("op '{op_id}': the solid carries no named edges — fillet/chamfer needs primitive or boolean provenance"),
		));
	};
	let limit = max_distance.unwrap_or(0.1 * bbox_diagonal(solid));
	if distance > limit {
		return Err(err(
			ErrorKind::FeatureFailed,
			format!(
				"op '{op_id}': witness [{}, {}, {}] matched no edge — nearest edge is {distance:.3} mm away (limit {limit:.3}; pass max_distance to widen)",
				witness.x, witness.y, witness.z
			),
		));
	}
	Ok((name, distance, limit))
}

/// Serialize a [`kernel_brep::FaceName`] exactly as kernel-model's
/// `edge_name_serde` does (operand variant name + `source_face`), so the
/// stateless op layer and the durable feature layer describe the same face the
/// same way and their names are directly comparable.
pub(crate) fn face_name_json(f: kernel_brep::FaceName) -> Value {
	let operand = match f.operand {
		kernel_brep::FaceSource::Primitive => "Primitive",
		kernel_brep::FaceSource::OperandA => "OperandA",
		kernel_brep::FaceSource::OperandB => "OperandB",
	};
	json!({ "operand": operand, "source_face": f.source_face })
}

/// The op-result measures carrying the `resolved_edge` receipt a
/// witness-selecting op leaves behind: the canonical face-pair
/// [`kernel_brep::EdgeName`] the spatial witness actually latched, plus how
/// close the match was and the limit in effect. Its purpose is detection — a
/// parameter sweep can compare this identity across candidates and catch a
/// witness silently jumping to a different edge. Selection is unchanged; this
/// only records what was chosen.
pub(crate) fn resolved_edge_measures(name: kernel_brep::EdgeName, witness_distance: f64, max_distance: f64) -> Value {
	json!({
		"resolved_edge": {
			"faces": [face_name_json(name.faces[0]), face_name_json(name.faces[1])],
			"witness_distance": witness_distance,
			"max_distance": max_distance,
		}
	})
}

/// Map a kernel [`FilletError`] to a structured op error.
pub(crate) fn map_fillet_error(op_id: &str, what: &str, e: FilletError) -> OpError {
	match e {
		FilletError::BadRadius => err(ErrorKind::InvalidParam, format!("op '{op_id}': {what}: radius must be positive and finite")),
		FilletError::RadiusTooLarge => err(
			ErrorKind::FeatureFailed,
			format!("op '{op_id}': {what}: the radius does not fit within the adjacent faces"),
		),
		FilletError::EdgeNotFound => err(ErrorKind::FeatureFailed, format!("op '{op_id}': {what}: the selected edge no longer resolves")),
		FilletError::EdgeAmbiguous => err(
			ErrorKind::FeatureFailed,
			format!("op '{op_id}': {what}: the edge name resolves to several fragments — move the witness closer to one"),
		),
		// One kernel variant covers every scope refusal (concave junction, curved
		// wall, non-trivalent corner, …), so the message states the WHOLE
		// supported scope and calls out the most common trap: a blind-agent
		// live-fire hit a concave junction and was told the edge "is not
		// straight/perpendicular" when it was both — the real reason was
		// convexity. Verified: chamfer_edge_near shares the convexity check, so
		// it is NOT offered as the concave alternative.
		FilletError::Unsupported => err(
			ErrorKind::FeatureFailed,
			format!(
				"op '{op_id}': {what}: the edge near the witness is outside the supported scope — supported: CONVEX straight edges between two planar faces (any convex dihedral angle, simple 3-face corners at both ends) via fillet_edge_near/chamfer_edge_near, and convex circular rims via fillet_circular_rim. Concave junctions (inside corners, where the round would ADD material) are out of scope for BOTH fillet_edge_near and chamfer_edge_near — model the cove explicitly instead: difference a cylinder from a corner bar to leave a quarter-round strip, then union it into the junction"
			),
		),
	}
}

/// Map a kernel [`SketchError`] to a structured op error.
pub(crate) fn map_sketch_error(op_id: &str, what: &str, e: SketchError) -> OpError {
	let reason = match e {
		SketchError::Degenerate => "the profile is degenerate (fewer than 3 points, or it encloses no area)",
		SketchError::NotClosed => "the segments/arcs do not form a single closed loop",
		SketchError::EmptySolid => "the sweep produced no solid (zero height, or radii outside the revolve domain)",
	};
	err(ErrorKind::SketchFailed, format!("op '{op_id}': {what}: {reason}"))
}

/// Map a kernel [`HoleError`] to a structured op error (every variant is a bad
/// or out-of-table parameter, so the kind is `invalid_param`; the message
/// carries the kernel's precise reason).
pub(crate) fn map_hole_error(op_id: &str, what: &str, e: HoleError) -> OpError {
	err(ErrorKind::InvalidParam, format!("op '{op_id}': {what}: {e} — see API.md for the supported table sizes"))
}

/// Snap rotation-matrix entries that are pure float dirt to exact 0 / ±1.
///
/// An axis-permutation rotation (90/180/270° about a coordinate axis, 120°
/// about [1,1,1], …) SHOULD be an exact signed permutation matrix, but the
/// axis-angle construction leaves ~1e-16 residue in the "zero" entries. That
/// residue is what turned exactly-coplanar face pairs into near-coplanar
/// limbo inside the boolean arrangement: a prism posed by the [1,1,1]/120°
/// permutation and abutting a hole wall failed `union` with
/// `invalid_geometry` while the identical axis-aligned box unioned fine
/// (friction folding_book_stand F1/F3, 2026-08-27). Entries within 1e-12 of
/// {0, ±1} are snapped; a genuinely oblique rotation has no such entries and
/// passes through unchanged.
pub(crate) fn snap_rotation(m: DAffine3) -> DAffine3 {
	let snap = |v: f64| {
		if v.abs() < 1e-12 {
			0.0
		} else if (v - 1.0).abs() < 1e-12 {
			1.0
		} else if (v + 1.0).abs() < 1e-12 {
			-1.0
		} else {
			v
		}
	};
	let mut out = m;
	let m3 = &mut out.matrix3;
	for col in [&mut m3.x_axis, &mut m3.y_axis, &mut m3.z_axis] {
		col.x = snap(col.x);
		col.y = snap(col.y);
		col.z = snap(col.z);
	}
	out
}

/// Reject a voxel lattice over `domain` that would exceed [`MAX_GRID_CELLS`]
/// BEFORE allocating it (the same discipline as `shell` / the density grids).
pub(crate) fn grid_guard(op_id: &str, what: &str, domain: Aabb, voxel: f64) -> Result<(), OpError> {
	let size = domain.size();
	let cells = (f64::from(size.x) / voxel).ceil() * (f64::from(size.y) / voxel).ceil() * (f64::from(size.z) / voxel).ceil();
	if !(cells.is_finite() && cells <= MAX_GRID_CELLS as f64) {
		return Err(err(
			ErrorKind::InvalidParam,
			format!("op '{op_id}': {what}: the voxel lattice would be ≈{cells:.0} cells (bbox/voxel per axis), over the cap {MAX_GRID_CELLS} — use a coarser voxel"),
		));
	}
	Ok(())
}

/// Structured error for a catalog part size outside its standard's table.
pub(crate) fn size_err(op_id: &str, what: &str, standard: &str, m: f64, supported: &str) -> OpError {
	err(ErrorKind::InvalidParam, format!("op '{op_id}': {what}: M{m} is not in the {standard} table (supported: {supported})"))
}

/// The fastener tables (ISO 4017 / ISO 4032 / ISO 7089 / DIN 912 / ISO 10642 /
/// DIN 985 / ISO 261 coarse) share rows.
pub(crate) const FASTENER_SIZES: &str = "M3, M4, M5, M6, M8, M10, M12, M16";

/// The M3–M12 screw tables (ISO 7380 / DIN 916).
pub(crate) const SCREW_SIZES_M3_M12: &str = "M3, M4, M5, M6, M8, M10, M12";

/// The small-thread tables (heat-set inserts, hex standoffs).
pub(crate) const SMALL_SIZES_M2_M6: &str = "M2, M2.5, M3, M4, M5, M6";

/// DIN 471 external circlip shaft diameters.
pub(crate) const DIN471_SIZES: &str = "Ø8, 10, 12, 15, 20, 25, 30";

/// DIN 472 internal circlip bore diameters.
#[cfg(feature = "catalog")]
pub(crate) const DIN472_SIZES: &str = "Ø16, 20, 22, 26, 32, 35, 42, 47";

/// The supported AS568 dash numbers (see `kernel_model::parts::as568_spec`).
pub(crate) const AS568_DASHES: &str = "10, 12, 14, 16, 18, 20, 110, 112, 115, 120, 210, 214, 218, 222, 325";

/// Stocked metric O-ring cord cross-sections (see `kernel_model::parts::metric_cord_gland`).
pub(crate) const METRIC_CORD_SIZES: &str = "Ø1, 1.5, 1.78, 2, 2.5, 2.62, 3, 3.53, 4, 5, 5.33, 6, 7";

/// Jaw-coupling body sizes (see `kernel_model::parts::jaw_coupling_spec`).
#[cfg(feature = "catalog")]
pub(crate) const JAW_COUPLING_SIZES: &str = "20 (L25), 25 (L30), 30 (L35), 40 (L50)";

/// Stocked set-screw rigid-coupling bores.
#[cfg(feature = "catalog")]
pub(crate) const SET_SCREW_COUPLING_BORES: &str = "Ø4, 5, 6, 6.35, 8, 10, 12";

/// Stocked clamp-coupling bores.
#[cfg(feature = "catalog")]
pub(crate) const CLAMP_COUPLING_BORES: &str = "Ø4, 5, 6, 8, 10, 12";

/// NEMA stepper frames in the table (see `kernel_model::parts::nema_spec`).
#[cfg(feature = "catalog")]
pub(crate) const NEMA_FRAMES: &str = "17, 23";

/// Hobby-servo models in the table (see `kernel_model::parts::servo_spec`).
#[cfg(feature = "catalog")]
pub(crate) const SERVO_MODELS: &str = "sg90, mg996r";
