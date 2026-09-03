// Copyright (c) LMCAD. Licensed under the MIT License.

//! Constrained 2D sketches: the solver-backed `sketch` op and the two sweeps that
//! consume a solved sketch (`sketch_extrude`, `sketch_revolve`).

use std::collections::{BTreeMap, BTreeSet};

use kernel_brep::math::DVec2;
use kernel_model::{ConstraintState, Sketch, SketchConstraint};
use serde_json::json;

use crate::interp::{err, fetch_sketch, EnvValue, Outcome};
use crate::program::{ConstraintSpec, OpKind};
use crate::report::{ErrorKind, OpError};

use super::support::{bind_solid, map_sketch_error};

/// Build and solve the kernel sketch for an `op: "sketch"`, with full index
/// bounds-checking (the kernel solver would panic on an out-of-range PointId).
#[allow(clippy::too_many_arguments)]
pub(crate) fn build_sketch(
	op_id: &str,
	points: &[[f64; 2]],
	segments: &[[usize; 2]],
	arcs: &[crate::program::ArcSpec],
	circles: &[crate::program::CircleSpec],
	constraints: &[ConstraintSpec],
) -> Result<Outcome, OpError> {
	let n = points.len();
	let check = |what: &str, k: usize, indices: &[usize]| -> Result<(), OpError> {
		for &i in indices {
			if i >= n {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': {what} #{k} references point {i}, but the sketch has only {n} points"),
				));
			}
		}
		Ok(())
	};

	let mut sketch = Sketch::new();
	for p in points {
		sketch.add_point(DVec2::new(p[0], p[1]));
	}
	for (k, s) in segments.iter().enumerate() {
		check("segment", k, &[s[0], s[1]])?;
		sketch.add_segment(s[0], s[1]);
	}
	for (k, a) in arcs.iter().enumerate() {
		check("arc", k, &[a.a, a.b, a.center])?;
		sketch.add_arc(a.a, a.b, a.center, a.ccw);
	}
	for (k, c) in circles.iter().enumerate() {
		check("circle", k, &[c.center, c.radius_point])?;
		sketch.add_circle(c.center, c.radius_point);
	}
	for (k, c) in constraints.iter().enumerate() {
		check("constraint", k, &c.point_indices())?;
		sketch.add_constraint(to_kernel_constraint(c));
	}

	let solve = sketch.solve();
	let analysis = sketch.analyze();
	let state = match analysis.state {
		ConstraintState::UnderConstrained => "under_constrained",
		ConstraintState::WellConstrained => "well_constrained",
		ConstraintState::OverConstrained => "over_constrained",
	};
	if !solve.converged {
		return Err(err(
			ErrorKind::SketchFailed,
			format!(
				"op '{op_id}': constraints did not converge (residual {:.3e} after {} iterations, state {state}) — they are conflicting or inconsistent",
				solve.residual, solve.iterations
			),
		));
	}
	Ok(Outcome {
		value: Some(EnvValue::Sketch(sketch)),
		measures: Some(json!({
			"residual": solve.residual,
			"iterations": solve.iterations,
			"converged": solve.converged,
			"dof": analysis.dof,
			"rank": analysis.rank,
			"free_dof": analysis.free_dof,
			"redundant": analysis.redundant,
			"state": state,
		})),
		file: None,
	})
}

/// Translate a JSON constraint into the kernel's [`SketchConstraint`] (degrees →
/// radians at this boundary).
pub(crate) fn to_kernel_constraint(c: &ConstraintSpec) -> SketchConstraint {
	match *c {
		ConstraintSpec::Fixed { point, at } => SketchConstraint::Fixed { point, at: DVec2::new(at[0], at[1]) },
		ConstraintSpec::Coincident { a, b } => SketchConstraint::Coincident { a, b },
		ConstraintSpec::Horizontal { a, b } => SketchConstraint::Horizontal { a, b },
		ConstraintSpec::Vertical { a, b } => SketchConstraint::Vertical { a, b },
		ConstraintSpec::Distance { a, b, distance } => SketchConstraint::Distance { a, b, distance },
		ConstraintSpec::Parallel { a, b, c, d } => SketchConstraint::Parallel { a, b, c, d },
		ConstraintSpec::Perpendicular { a, b, c, d } => SketchConstraint::Perpendicular { a, b, c, d },
		ConstraintSpec::EqualLength { a, b, c, d } => SketchConstraint::EqualLength { a, b, c, d },
		ConstraintSpec::Tangent { line_a, line_b, center, radius_point } => {
			SketchConstraint::Tangent { line_a, line_b, center, radius_point }
		}
		ConstraintSpec::Angle { a, b, c, d, degrees } => SketchConstraint::Angle { a, b, c, d, radians: degrees.to_radians() },
		ConstraintSpec::Symmetric { a, b, line_a, line_b } => SketchConstraint::Symmetric { a, b, line_a, line_b },
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
		OpKind::Sketch { points, segments, arcs, circles, constraints } => {
			build_sketch(op_id, &points, &segments, &arcs, &circles, &constraints)
		}
		#[cfg(feature = "catalog")]
		OpKind::SketchExtrude { sketch, height } => {
			let sk = fetch_sketch(env, all_ids, op_id, "sketch", &sketch)?;
			let solid = sk.extrude(height).map_err(|e| map_sketch_error(op_id, "sketch_extrude", e))?;
			bind_solid(op_id, "sketch_extrude", solid)
		}
		OpKind::SketchRevolve { sketch, segments } => {
			let sk = fetch_sketch(env, all_ids, op_id, "sketch", &sketch)?;
			let solid = sk.revolve(segments).map_err(|e| map_sketch_error(op_id, "sketch_revolve", e))?;
			bind_solid(op_id, "sketch_revolve", solid)
		}

		_ => unreachable!("ops::sketch: op routed to the wrong family"),
	}
}
