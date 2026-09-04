// Copyright (c) LMCAD. Licensed under the MIT License.

//! [`Importer`] — the resolved view over a parsed entity graph: typed accessors
//! for points, directions, placements, curves and surfaces, the B-rep shell
//! index, and the per-file uncertainty allowance the tolerant path widens.

use std::collections::HashMap;

use kernel_core::math::DVec3;

use crate::geom::{perp_basis, Curve, Surface};
use crate::nurbs::{NurbsCurve, NurbsSurface};

use super::edges::{
	complex_part, edge_sweep, expand_knots, last_enum, max_chord_turn, sample_arc, ShellFaces, BSPLINE_EDGE_SEGMENTS, FULL_TURN_SEGMENTS,
	MAX_BSPLINE_EDGE_SEGMENTS,
};
use super::parse::{file_uncertainty, Entity, Value};
use super::StepError;

// --- Reconstruction ----------------------------------------------------------

pub(crate) struct Importer<'a> {
	pub(crate) ents: &'a HashMap<u32, Entity>,
	/// The file's asserted uncertainty (mm, [`file_uncertainty`]), `0` when absent
	/// — the allowance within which a trim vertex may sit off its B-spline patch
	/// and still be projected onto it (root cause (a) of the vendor refusals).
	pub(crate) uncertainty: f64,
	/// Tolerant-mode allowance multiplier on `uncertainty` for the trim-vertex
	/// snap: `1` in strict mode (the file's own assertion is the limit),
	/// [`TOLERANT_SNAP_FACTOR`] in tolerant mode.
	pub(crate) snap_factor: f64,
}

/// How many times the file's asserted uncertainty a trim vertex may sit off its
/// B-spline patch in **tolerant** mode before the face is refused (strict mode
/// allows exactly the uncertainty). Each such acceptance is a reported repair.
pub(crate) const TOLERANT_SNAP_FACTOR: f64 = 10.0;

impl<'a> Importer<'a> {
	/// Strict importer over a parsed entity graph.
	pub(crate) fn new(ents: &'a HashMap<u32, Entity>) -> Self {
		Importer { ents, uncertainty: file_uncertainty(ents).unwrap_or(0.0), snap_factor: 1.0 }
	}

	/// The absolute distance (mm) a trim vertex may sit off its patch and still be
	/// projected onto it: the file's uncertainty times the mode's factor.
	pub(crate) fn snap_allowance(&self) -> f64 {
		self.uncertainty * self.snap_factor
	}

	pub(crate) fn get(&self, id: u32) -> Result<&Entity, StepError> {
		self.ents.get(&id).ok_or_else(|| StepError::Reference(format!("missing entity #{id}")))
	}

	/// Resolve a `CARTESIAN_POINT` reference to a 3-D position.
	pub(crate) fn point(&self, id: u32) -> Result<DVec3, StepError> {
		let e = self.get(id)?;
		if e.name != "CARTESIAN_POINT" {
			return Err(StepError::Reference(format!("#{id} is {}, expected CARTESIAN_POINT", e.name)));
		}
		let coords = e
			.args
			.iter()
			.find_map(Value::as_list)
			.ok_or_else(|| StepError::Parse(format!("#{id} CARTESIAN_POINT has no coordinate list")))?;
		let c: Vec<f64> = coords.iter().filter_map(Value::as_real).collect();
		if c.len() < 3 {
			return Err(StepError::Parse(format!("#{id} CARTESIAN_POINT needs 3 coordinates")));
		}
		Ok(DVec3::new(c[0], c[1], c[2]))
	}

	/// Resolve a `DIRECTION` reference to a vector.
	fn direction(&self, id: u32) -> Result<DVec3, StepError> {
		let e = self.get(id)?;
		if e.name != "DIRECTION" {
			return Err(StepError::Reference(format!("#{id} is {}, expected DIRECTION", e.name)));
		}
		let coords =
			e.args.iter().find_map(Value::as_list).ok_or_else(|| StepError::Parse(format!("#{id} DIRECTION has no component list")))?;
		let c: Vec<f64> = coords.iter().filter_map(Value::as_real).collect();
		if c.len() < 3 {
			return Err(StepError::Parse(format!("#{id} DIRECTION needs 3 components")));
		}
		Ok(DVec3::new(c[0], c[1], c[2]))
	}

	/// Resolve a `VERTEX_POINT` reference to its position.
	pub(crate) fn vertex(&self, id: u32) -> Result<DVec3, StepError> {
		let e = self.get(id)?;
		if e.name != "VERTEX_POINT" {
			return Err(StepError::Reference(format!("#{id} is {}, expected VERTEX_POINT", e.name)));
		}
		let cp = e.args.iter().find_map(Value::as_ref).ok_or_else(|| StepError::Parse(format!("#{id} VERTEX_POINT has no point")))?;
		self.point(cp)
	}

	/// Resolve a face's supporting [`Surface`] (plane and the four analytic quadrics).
	pub(crate) fn surface(&self, id: u32) -> Result<Surface, StepError> {
		let e = self.get(id)?;
		let placement =
			|| e.args.iter().find_map(Value::as_ref).ok_or_else(|| StepError::Parse(format!("#{id} {} has no placement", e.name)));
		// Trailing scalar parameters (radius, semi-angle, …) after the placement.
		let reals: Vec<f64> = e.args.iter().filter_map(Value::as_real).collect();
		let real = |k: usize| reals.get(k).copied().ok_or_else(|| StepError::Parse(format!("#{id} {} missing scalar {k}", e.name)));
		match e.name.as_str() {
			"PLANE" => {
				let (origin, normal, _) = self.placement(placement()?)?;
				Ok(Surface::Plane { origin, normal })
			}
			"CYLINDRICAL_SURFACE" => {
				let (origin, axis, _) = self.placement(placement()?)?;
				Ok(Surface::Cylinder { origin, axis, radius: real(0)? })
			}
			"SPHERICAL_SURFACE" => {
				let (center, _, _) = self.placement(placement()?)?;
				Ok(Surface::Sphere { center, radius: real(0)? })
			}
			"CONICAL_SURFACE" => {
				// STEP places the cone at a reference plane (radius `r` there, half
				// angle `a`); the apex is back along −axis where the radius vanishes.
				let (location, axis, _) = self.placement(placement()?)?;
				let (r, half_angle) = (real(0)?, real(1)?);
				let apex = location - axis * (r / half_angle.tan());
				Ok(Surface::Cone { apex, axis, half_angle })
			}
			"TOROIDAL_SURFACE" => {
				let (center, axis, _) = self.placement(placement()?)?;
				Ok(Surface::Torus { center, axis, major: real(0)?, minor: real(1)? })
			}
			other => Err(StepError::Unsupported(format!("surface #{id} of type {other}"))),
		}
	}

	/// Parse a `B_SPLINE_SURFACE_WITH_KNOTS` entity into a [`NurbsSurface`] (the
	/// non-rational case; unit weights). Control points come from the
	/// `CARTESIAN_POINT` grid, and the full knot vectors are reconstructed from the
	/// distinct-knot + multiplicity lists. Evaluation and tessellation are then
	/// provided by [`NurbsSurface`] — the reading half of NURBS interchange.
	pub(crate) fn bspline_surface(&self, id: u32) -> Result<NurbsSurface, StepError> {
		let e = self.get(id)?;
		// Accept a plain B_SPLINE_SURFACE_WITH_KNOTS entity OR a _COMPLEX (rational) instance, which
		// splits its data across B_SPLINE_SURFACE (degrees + control grid), B_SPLINE_SURFACE_WITH_KNOTS
		// (knots + multiplicities) and RATIONAL_B_SPLINE_SURFACE (a weight grid). Reading the weight
		// grid is what makes an imported cylinder/sphere/conic surface (which CAD exporters encode as a
		// rational B-spline) geometrically exact instead of silently de-rationalised to unit weights.
		let (field_args, rational_weights): (Vec<Value>, Option<Vec<Vec<f64>>>) = match e.name.as_str() {
			"B_SPLINE_SURFACE_WITH_KNOTS" => (e.args.clone(), None),
			"_COMPLEX" => {
				let mut a = Vec::new();
				for sub in ["B_SPLINE_SURFACE", "B_SPLINE_SURFACE_WITH_KNOTS"] {
					if let Some(p) = complex_part(&e.args, sub) {
						a.extend_from_slice(p);
					}
				}
				if a.is_empty() {
					return Err(StepError::Reference(format!("#{id} _COMPLEX has no B_SPLINE_SURFACE record")));
				}
				let w = complex_part(&e.args, "RATIONAL_B_SPLINE_SURFACE")
					.and_then(|p| {
						p.iter().find_map(|v| match v {
							Value::List(rows) if matches!(rows.first(), Some(Value::List(_))) => Some(rows),
							_ => None,
						})
					})
					.map(|rows| {
						rows.iter()
							.filter_map(|r| r.as_list().map(|cells| cells.iter().filter_map(Value::as_real).collect::<Vec<f64>>()))
							.collect()
					});
				(a, w)
			}
			other => return Err(StepError::Reference(format!("#{id} is {other}, expected B_SPLINE_SURFACE_WITH_KNOTS"))),
		};
		let args = &field_args;
		// The two top-level integer scalars are the u- and v-degrees (multiplicities and
		// control references live inside nested lists, never at the top level).
		let degs: Vec<usize> = args.iter().filter_map(|v| v.as_int().map(|i| i.max(0) as usize)).collect();
		if degs.len() < 2 {
			return Err(StepError::Parse(format!("#{id} B_SPLINE_SURFACE_WITH_KNOTS missing degrees")));
		}
		let (deg_u, deg_v) = (degs[0], degs[1]);
		// Control grid: the first list whose elements are themselves lists (of point refs).
		let grid = args
			.iter()
			.find_map(|v| match v {
				Value::List(rows) if matches!(rows.first(), Some(Value::List(_))) => Some(rows),
				_ => None,
			})
			.ok_or_else(|| StepError::Parse(format!("#{id} B_SPLINE_SURFACE_WITH_KNOTS has no control grid")))?;
		let mut control: Vec<Vec<DVec3>> = Vec::with_capacity(grid.len());
		for row in grid {
			let cells = row.as_list().ok_or_else(|| StepError::Parse(format!("#{id} control row is not a list")))?;
			let mut pts = Vec::with_capacity(cells.len());
			for c in cells {
				let cp = c.as_ref().ok_or_else(|| StepError::Parse(format!("#{id} control cell is not a point reference")))?;
				pts.push(self.point(cp)?);
			}
			control.push(pts);
		}
		// The remaining flat lists are, in order, the u- and v-multiplicities (all-Int)
		// and the u- and v-knots (all-Real); the control grid (lists/refs) is skipped.
		let mut mults: Vec<Vec<i64>> = Vec::new();
		let mut knots: Vec<Vec<f64>> = Vec::new();
		for v in args {
			let Value::List(items) = v else { continue };
			if items.is_empty() || items.iter().any(|x| matches!(x, Value::List(_) | Value::Ref(_))) {
				continue;
			}
			if items.iter().all(|x| matches!(x, Value::Int(_))) {
				mults.push(items.iter().filter_map(Value::as_int).collect());
			} else if items.iter().all(|x| matches!(x, Value::Real(_) | Value::Int(_))) {
				knots.push(items.iter().filter_map(Value::as_real).collect());
			}
		}
		if mults.len() < 2 || knots.len() < 2 {
			return Err(StepError::Parse(format!("#{id} B_SPLINE_SURFACE_WITH_KNOTS missing knot/multiplicity lists")));
		}
		let knots_u = expand_knots(&knots[0], &mults[0]);
		let knots_v = expand_knots(&knots[1], &mults[1]);
		let weights: Vec<Vec<f64>> = rational_weights.unwrap_or_else(|| control.iter().map(|r| vec![1.0; r.len()]).collect());
		NurbsSurface::new(deg_u, deg_v, knots_u, knots_v, control, weights)
			.ok_or_else(|| StepError::Topology(format!("#{id} B_SPLINE_SURFACE_WITH_KNOTS has inconsistent dimensions")))
	}

	/// Parse a `B_SPLINE_CURVE_WITH_KNOTS` entity into a [`NurbsCurve`] (non-rational;
	/// unit weights). The control points are a flat `CARTESIAN_POINT` reference list and
	/// the full knot vector is reconstructed from the distinct-knot + multiplicity lists.
	pub(crate) fn bspline_curve(&self, id: u32) -> Result<NurbsCurve, StepError> {
		let e = self.get(id)?;
		// Accept a plain B_SPLINE_CURVE_WITH_KNOTS entity OR a _COMPLEX (rational) instance, which
		// splits its data across B_SPLINE_CURVE (degree + control points), B_SPLINE_CURVE_WITH_KNOTS
		// (knots + multiplicities) and RATIONAL_B_SPLINE_CURVE (per-control-point weights). Reading
		// the weights is what makes an imported circle/conic (which CAD exporters encode as a
		// rational B-spline) geometrically exact instead of silently de-rationalised to unit weights.
		let (field_args, rational_weights): (Vec<Value>, Option<Vec<f64>>) = match e.name.as_str() {
			"B_SPLINE_CURVE_WITH_KNOTS" => (e.args.clone(), None),
			"_COMPLEX" => {
				let mut a = Vec::new();
				for sub in ["B_SPLINE_CURVE", "B_SPLINE_CURVE_WITH_KNOTS"] {
					if let Some(p) = complex_part(&e.args, sub) {
						a.extend_from_slice(p);
					}
				}
				if a.is_empty() {
					return Err(StepError::Reference(format!("#{id} _COMPLEX has no B_SPLINE_CURVE record")));
				}
				let w = complex_part(&e.args, "RATIONAL_B_SPLINE_CURVE")
					.and_then(|p| p.iter().find_map(Value::as_list))
					.map(|w| w.iter().filter_map(Value::as_real).collect());
				(a, w)
			}
			other => return Err(StepError::Reference(format!("#{id} is {other}, expected B_SPLINE_CURVE_WITH_KNOTS"))),
		};
		let args = &field_args;
		let degree = args
			.iter()
			.find_map(Value::as_int)
			.map(|i| i.max(0) as usize)
			.ok_or_else(|| StepError::Parse(format!("#{id} B_SPLINE_CURVE_WITH_KNOTS missing degree")))?;
		// Control points: the first list whose elements are point references.
		let refs = args
			.iter()
			.find_map(|v| match v {
				Value::List(items) if matches!(items.first(), Some(Value::Ref(_))) => Some(items),
				_ => None,
			})
			.ok_or_else(|| StepError::Parse(format!("#{id} B_SPLINE_CURVE_WITH_KNOTS has no control list")))?;
		let mut control = Vec::with_capacity(refs.len());
		for c in refs {
			let cp = c.as_ref().ok_or_else(|| StepError::Parse(format!("#{id} control cell is not a point reference")))?;
			control.push(self.point(cp)?);
		}
		// The flat Int list is the multiplicities; the flat Real list, the distinct knots.
		let mut mults: Option<Vec<i64>> = None;
		let mut knots: Option<Vec<f64>> = None;
		for v in args {
			let Value::List(items) = v else { continue };
			if items.is_empty() || items.iter().any(|x| matches!(x, Value::Ref(_) | Value::List(_))) {
				continue;
			}
			if items.iter().all(|x| matches!(x, Value::Int(_))) {
				mults.get_or_insert_with(|| items.iter().filter_map(Value::as_int).collect());
			} else if items.iter().all(|x| matches!(x, Value::Real(_) | Value::Int(_))) {
				knots.get_or_insert_with(|| items.iter().filter_map(Value::as_real).collect());
			}
		}
		let mults = mults.ok_or_else(|| StepError::Parse(format!("#{id} B_SPLINE_CURVE_WITH_KNOTS missing multiplicities")))?;
		let knots = knots.ok_or_else(|| StepError::Parse(format!("#{id} B_SPLINE_CURVE_WITH_KNOTS missing knots")))?;
		let weights = rational_weights.unwrap_or_else(|| vec![1.0; control.len()]);
		NurbsCurve::new(degree, expand_knots(&knots, &mults), control, weights)
			.ok_or_else(|| StepError::Topology(format!("#{id} B_SPLINE_CURVE_WITH_KNOTS has inconsistent dimensions")))
	}

	/// `AXIS2_PLACEMENT_3D` → (location, axis/normal, ref direction).
	pub(crate) fn placement(&self, id: u32) -> Result<(DVec3, DVec3, DVec3), StepError> {
		let e = self.get(id)?;
		if e.name != "AXIS2_PLACEMENT_3D" {
			return Err(StepError::Reference(format!("#{id} is {}, expected AXIS2_PLACEMENT_3D", e.name)));
		}
		let refs: Vec<u32> = e.args.iter().filter_map(Value::as_ref).collect();
		if refs.is_empty() {
			return Err(StepError::Parse(format!("#{id} AXIS2_PLACEMENT_3D has no location")));
		}
		let origin = self.point(refs[0])?;
		// Axis and ref direction are optional in the schema; default sensibly.
		let normal = refs.get(1).map(|&r| self.direction(r)).transpose()?.unwrap_or(DVec3::Z);
		let refdir = refs.get(2).map(|&r| self.direction(r)).transpose()?.unwrap_or(DVec3::X);
		Ok((origin, normal.normalize_or_zero(), refdir))
	}

	/// Orthonormal frame of an `AXIS2_PLACEMENT_3D`: `(location, unit axis, unit x, unit y)`
	/// with `x` the reference direction re-orthogonalised against the axis (per ISO
	/// 10303-42) and `y = axis × x` — the frame conic parameter angles are measured in.
	pub(crate) fn frame(&self, id: u32) -> Result<(DVec3, DVec3, DVec3, DVec3), StepError> {
		let (origin, axis, refdir) = self.placement(id)?;
		let axis = if axis.length_squared() > 1e-20 { axis } else { DVec3::Z };
		let r = refdir - axis * refdir.dot(axis);
		let x = if r.length_squared() > 1e-20 { r.normalize() } else { perp_basis(axis).0 };
		Ok((origin, axis, x, axis.cross(x)))
	}

	/// Canonical polyline of an `EDGE_CURVE` in the edge's own v_start→v_end direction,
	/// plus the analytic [`Curve`] its segments lie on (for conic geometry). `LINE`
	/// edges (and absent geometry) are the exact two-vertex chord. `CIRCLE`/`ELLIPSE`
	/// edges keep their chord up to a 90° sweep (the producer's own granularity, which
	/// keeps re-imports of this kernel's faceted exports exact) and are sampled at the
	/// `FULL_TURN_SEGMENTS` pitch beyond that — a full ring (identical start/end
	/// vertex) becomes a closed 48-segment ring. B-spline edges are sampled over their
	/// knot domain; `SURFACE_CURVE`/`SEAM_CURVE` wrappers are unwrapped to their 3-D
	/// curve. Anything else is a loud [`StepError::Unsupported`]. Endpoints are always
	/// the exact `VERTEX_POINT` positions so shared edges intern identically.
	pub(crate) fn edge_polyline(&self, ec_id: u32) -> Result<(Vec<DVec3>, Option<Curve>), StepError> {
		let ec = self.get(ec_id)?;
		if ec.name != "EDGE_CURVE" {
			return Err(StepError::Reference(format!("#{ec_id} is {}, expected EDGE_CURVE", ec.name)));
		}
		// EDGE_CURVE('', #v_start, #v_end, #curve, same_sense)
		let refs: Vec<u32> = ec.args.iter().filter_map(Value::as_ref).collect();
		if refs.len() < 2 {
			return Err(StepError::Parse(format!("#{ec_id} EDGE_CURVE needs two vertices")));
		}
		let start = self.vertex(refs[0])?;
		let end = self.vertex(refs[1])?;
		let same_sense = last_enum(ec).map(|s| s == "T").unwrap_or(true);
		let Some(&geom_id) = refs.get(2) else {
			return Ok((vec![start, end], None)); // no geometry reference: the straight chord
		};
		let mut gid = geom_id;
		let mut g = self.get(gid)?;
		// AP214 wraps an edge's 3-D geometry: SURFACE_CURVE('', #curve, (pcurves), repr).
		if g.name == "SURFACE_CURVE" || g.name == "SEAM_CURVE" {
			gid = g.args.iter().find_map(Value::as_ref).ok_or_else(|| StepError::Parse(format!("#{gid} {} has no 3-D curve", g.name)))?;
			g = self.get(gid)?;
		}
		match g.name.as_str() {
			"LINE" => Ok((vec![start, end], None)),
			"CIRCLE" => {
				// CIRCLE('', #placement, radius)
				let placement =
					g.args.iter().find_map(Value::as_ref).ok_or_else(|| StepError::Parse(format!("#{gid} CIRCLE has no placement")))?;
				let (center, axis, x, y) = self.frame(placement)?;
				let radius =
					g.args.iter().rev().find_map(Value::as_real).ok_or_else(|| StepError::Parse(format!("#{gid} CIRCLE has no radius")))?;
				let angle = |p: DVec3| (p - center).dot(y).atan2((p - center).dot(x));
				let sweep = edge_sweep(angle(start), angle(end), same_sense, ec_id)?;
				let eval = |t: f64| center + (x * t.cos() + y * t.sin()) * radius;
				Ok((sample_arc(start, end, angle(start), sweep, eval), Some(Curve::Circle { center, normal: axis, radius })))
			}
			"ELLIPSE" => {
				// ELLIPSE('', #placement, semi_axis_1, semi_axis_2): semi_axis_1 lies along
				// the placement's reference direction, semi_axis_2 along axis × ref.
				let placement =
					g.args.iter().find_map(Value::as_ref).ok_or_else(|| StepError::Parse(format!("#{gid} ELLIPSE has no placement")))?;
				let (center, axis, x, y) = self.frame(placement)?;
				let semis: Vec<f64> = g.args.iter().filter_map(Value::as_real).collect();
				if semis.len() < 2 || !(semis[0] > 0.0 && semis[1] > 0.0) {
					return Err(StepError::Parse(format!("#{gid} ELLIPSE needs two positive semi-axes")));
				}
				let (a1, a2) = (semis[0], semis[1]);
				let angle = |p: DVec3| ((p - center).dot(y) / a2).atan2((p - center).dot(x) / a1);
				let sweep = edge_sweep(angle(start), angle(end), same_sense, ec_id)?;
				let eval = |t: f64| center + x * (a1 * t.cos()) + y * (a2 * t.sin());
				// [`Curve::Ellipse`] wants `u` = the semi-MAJOR direction with a ≥ b; the
				// swapped frame parameterises the same point set (a quarter-phase shift).
				let curve = if a1 >= a2 {
					Curve::Ellipse { center, normal: axis, u: x, a: a1, b: a2 }
				} else {
					Curve::Ellipse { center, normal: axis, u: y, a: a2, b: a1 }
				};
				Ok((sample_arc(start, end, angle(start), sweep, eval), Some(curve)))
			}
			name if name == "B_SPLINE_CURVE_WITH_KNOTS"
				|| (name == "_COMPLEX" && complex_part(&g.args, "B_SPLINE_CURVE_WITH_KNOTS").is_some()) =>
			{
				let c = self.bspline_curve(gid)?;
				let (lo, hi) = c.domain();
				// Curvature-honest segment count: start at the historical
				// BSPLINE_EDGE_SEGMENTS granularity and double while any consecutive
				// chord pair still turns more than the conic ring pitch (2π /
				// FULL_TURN_SEGMENTS), capped at MAX_BSPLINE_EDGE_SEGMENTS. Gentle
				// freeform trim edges keep their 8 segments; a closed full-circle
				// B-spline rim (start == end vertex, which a fixed 8 would polygonise
				// to a 10%-deficit octagon) or a long sweeping arc is refined until
				// its chords match the fidelity of imported conic rings.
				let mut n = BSPLINE_EDGE_SEGMENTS;
				while n < MAX_BSPLINE_EDGE_SEGMENTS && max_chord_turn(&c, n) > std::f64::consts::TAU / FULL_TURN_SEGMENTS as f64 {
					n *= 2;
				}
				let mut pts = Vec::with_capacity(n + 1);
				pts.push(start);
				for k in 1..n {
					let f = k as f64 / n as f64;
					// Sample in the edge's direction of travel over the knot domain.
					let t = if same_sense { lo + (hi - lo) * f } else { hi - (hi - lo) * f };
					pts.push(c.point_at(t));
				}
				pts.push(end);
				Ok((pts, None))
			}
			other => Err(StepError::Unsupported(format!(
				"edge #{ec_id} geometry #{gid} of type {other} — importable edge curves are LINE, CIRCLE, ELLIPSE and B_SPLINE_CURVE_WITH_KNOTS"
			))),
		}
	}

	/// Tessellated, ordered boundary of an `EDGE_LOOP`: each `ORIENTED_EDGE` contributes
	/// its edge's canonical polyline (forward for `.T.`, reversed for `.F.`) minus the
	/// closing point, which the next edge supplies. Returns the boundary positions and,
	/// parallel to them, the analytic [`Curve`] each segment `points[i] → points[i+1 mod n]`
	/// lies on. Polylines are cached per `EDGE_CURVE` id, so the two faces sharing an
	/// edge get bit-identical points — the twin-pairing prerequisite.
	pub(crate) fn loop_boundary(
		&self,
		edge_loop: u32,
		cache: &mut HashMap<u32, (Vec<DVec3>, Option<Curve>)>,
	) -> Result<(Vec<DVec3>, Vec<Option<Curve>>), StepError> {
		let e = self.get(edge_loop)?;
		if e.name != "EDGE_LOOP" {
			return Err(StepError::Reference(format!("#{edge_loop} is {}, expected EDGE_LOOP", e.name)));
		}
		let oriented =
			e.args.iter().find_map(Value::as_list).ok_or_else(|| StepError::Parse(format!("#{edge_loop} EDGE_LOOP has no edge list")))?;
		let mut boundary = Vec::with_capacity(oriented.len());
		let mut curves = Vec::with_capacity(oriented.len());
		for oe in oriented {
			let oe_id = oe.as_ref().ok_or_else(|| StepError::Parse("EDGE_LOOP item is not a reference".into()))?;
			let oe_ent = self.get(oe_id)?;
			if oe_ent.name != "ORIENTED_EDGE" {
				return Err(StepError::Reference(format!("#{oe_id} is {}, expected ORIENTED_EDGE", oe_ent.name)));
			}
			// ORIENTED_EDGE('', *, *, #edge_curve, .T./.F.)
			let ec_id = oe_ent
				.args
				.iter()
				.find_map(Value::as_ref)
				.ok_or_else(|| StepError::Parse(format!("#{oe_id} ORIENTED_EDGE has no edge")))?;
			let oe_sense = last_enum(oe_ent).map(|s| s == "T").unwrap_or(true);
			if let std::collections::hash_map::Entry::Vacant(slot) = cache.entry(ec_id) {
				slot.insert(self.edge_polyline(ec_id)?);
			}
			let (pts, curve) = &cache[&ec_id];
			let m = pts.len();
			for k in 0..m - 1 {
				boundary.push(if oe_sense { pts[k] } else { pts[m - 1 - k] });
				curves.push(*curve);
			}
		}
		Ok((boundary, curves))
	}

	/// The placement axis (local +Z `DIRECTION`) of an analytic surface entity — the
	/// datum a periodic sphere face is unwrapped about. The placement is the surface's
	/// first reference; a missing axis defaults to +Z per the schema.
	pub(crate) fn surface_axis(&self, id: u32) -> Result<DVec3, StepError> {
		let e = self.get(id)?;
		let placement =
			e.args.iter().find_map(Value::as_ref).ok_or_else(|| StepError::Parse(format!("#{id} {} has no placement", e.name)))?;
		let (_, axis, _) = self.placement(placement)?;
		Ok(if axis.length_squared() > 1e-20 { axis.normalize() } else { DVec3::Z })
	}

	/// `(ADVANCED_FACE id, reversed)` pairs of one shell, resolving `ORIENTED_CLOSED_SHELL`
	/// wrappers: a `.F.` wrapper logically reverses every contained face's loops (real
	/// exporters use it for the void shells of a `BREP_WITH_VOIDS` and for mirrored
	/// instances). Faces keep the shell's stored order.
	pub(crate) fn shell_faces(&self, shell_id: u32, flip: bool) -> Result<ShellFaces, StepError> {
		let e = self.get(shell_id)?;
		match e.name.as_str() {
			"CLOSED_SHELL" | "OPEN_SHELL" => {
				let list = e
					.args
					.iter()
					.find_map(Value::as_list)
					.ok_or_else(|| StepError::Parse(format!("#{shell_id} {} has no face list", e.name)))?;
				let mut out = Vec::with_capacity(list.len());
				for f in list {
					let fid = f.as_ref().ok_or_else(|| StepError::Parse(format!("#{shell_id} shell face is not a reference")))?;
					out.push((fid, flip));
				}
				Ok(out)
			}
			"ORIENTED_CLOSED_SHELL" => {
				// ORIENTED_CLOSED_SHELL('', *, #shell, .T./.F.)
				let inner = e
					.args
					.iter()
					.find_map(Value::as_ref)
					.ok_or_else(|| StepError::Parse(format!("#{shell_id} ORIENTED_CLOSED_SHELL has no shell")))?;
				let same = last_enum(e).map(|s| s == "T").unwrap_or(true);
				self.shell_faces(inner, flip ^ !same)
			}
			other => Err(StepError::Unsupported(format!("shell #{shell_id} of type {other}"))),
		}
	}

	/// The `(face id, reversed)` sets of every `MANIFOLD_SOLID_BREP` / `BREP_WITH_VOIDS`
	/// in ascending entity-id order (deterministic across runs), keyed by the brep id.
	/// Empty when the file carries no solid-model entities (a bare-face fragment).
	pub(crate) fn brep_face_sets(&self) -> Result<Vec<(u32, ShellFaces)>, StepError> {
		let mut brep_ids: Vec<u32> =
			self.ents.iter().filter(|(_, e)| e.name == "MANIFOLD_SOLID_BREP" || e.name == "BREP_WITH_VOIDS").map(|(&id, _)| id).collect();
		brep_ids.sort_unstable();
		let mut out = Vec::with_capacity(brep_ids.len());
		for id in brep_ids {
			let e = self.get(id)?;
			// MANIFOLD_SOLID_BREP('', #outer) — BREP_WITH_VOIDS('', #outer, (#voids…)),
			// each void an ORIENTED_CLOSED_SHELL (normals already point INTO the material,
			// i.e. out of the cavity, when its flag is honoured).
			let outer =
				e.args.iter().find_map(Value::as_ref).ok_or_else(|| StepError::Parse(format!("#{id} {} has no outer shell", e.name)))?;
			let mut faces = self.shell_faces(outer, false)?;
			if e.name == "BREP_WITH_VOIDS" {
				let voids = e
					.args
					.iter()
					.find_map(Value::as_list)
					.ok_or_else(|| StepError::Parse(format!("#{id} BREP_WITH_VOIDS has no void list")))?;
				for v in voids {
					let vid = v.as_ref().ok_or_else(|| StepError::Parse(format!("#{id} void shell is not a reference")))?;
					faces.extend(self.shell_faces(vid, false)?);
				}
			}
			out.push((id, faces));
		}
		Ok(out)
	}

	/// Every `ADVANCED_FACE` id in ascending order — the fallback face set for files
	/// (e.g. snippets) that carry bare faces without `MANIFOLD_SOLID_BREP` structure.
	pub(crate) fn all_face_ids(&self) -> ShellFaces {
		let mut ids: Vec<u32> = self.ents.iter().filter(|(_, e)| e.name == "ADVANCED_FACE").map(|(&id, _)| id).collect();
		ids.sort_unstable();
		ids.into_iter().map(|id| (id, false)).collect()
	}
}
