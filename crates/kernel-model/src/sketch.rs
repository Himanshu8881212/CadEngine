// Copyright (c) LMCAD. Licensed under the MIT License.

//! 2D sketch + constraint solver — the parametric front end of the kernel.
//!
//! Most mechanical parts begin life as a *constrained 2D sketch* that is then
//! swept into a solid. This module supplies that missing front door: a [`Sketch`]
//! holds 2D points and the [`Segment`]s between them, plus a list of geometric
//! [`SketchConstraint`]s (horizontal, vertical, distance, coincident, parallel,
//! perpendicular, and a "fixed" anchor). [`Sketch::solve`] drives the points to
//! satisfy those constraints, after which [`Sketch::profile`] reads out the
//! closed boundary loop and [`Sketch::extrude`] / [`Sketch::revolve`] turn it into
//! a [`Solid`] via the B-rep half of the kernel.
//!
//! # The solver
//!
//! Each constraint contributes one or more scalar *residuals* that vanish exactly
//! when it is satisfied (e.g. `Horizontal{a,b}` ⇒ `y_b − y_a`). Stacking every
//! residual gives a vector function `r(x)` of the flattened point coordinates
//! `x = [x₀, y₀, x₁, y₁, …]`. [`Sketch::solve`] minimizes `‖r(x)‖²` with a
//! **Levenberg–Marquardt** iteration — Gauss–Newton `(JᵀJ + λI)Δ = −Jᵀr` with an
//! adaptive damping `λ` that guarantees a descent step even when the system is
//! rank-deficient (an under-constrained sketch simply leaves its free DOFs where
//! they started). The Jacobian `J` is formed by central differences, so adding a
//! new constraint type only requires writing its residual. No external solver
//! crate is used.

use kernel_brep::{
	cylinder as brep_cylinder, extrude as brep_extrude, extrude_tapered as brep_extrude_tapered,
	extrude_with_holes as brep_extrude_with_holes, revolve as brep_revolve, Solid,
};
use kernel_core::math::{DVec2, DVec3};
use serde::{Deserialize, Serialize};

/// Index of a point within a [`Sketch`] (its position in [`Sketch::points`]).
pub type PointId = usize;

/// A straight edge between two sketch points. Segments define the sketch's
/// topology: [`Sketch::profile`] walks them to recover the closed boundary loop.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Segment {
	/// First endpoint.
	pub a: PointId,
	/// Second endpoint.
	pub b: PointId,
}

/// A circular arc edge between two boundary points, bulging around a `center`
/// construction point. `ccw` selects which of the two arcs (the one swept
/// counter-clockwise from `a` to `b`, or its complement). The center is an
/// ordinary sketch point — so it can itself be constrained — but it is *not* part
/// of the boundary loop; only `a` and `b` connect the profile.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Arc {
	/// First endpoint (on the circle).
	pub a: PointId,
	/// Second endpoint (on the circle).
	pub b: PointId,
	/// Center construction point.
	pub center: PointId,
	/// Sweep counter-clockwise from `a` to `b` (otherwise clockwise).
	pub ccw: bool,
}

/// A full circle as a standalone closed profile, defined by a `center` point and a
/// `radius_point` lying on it. A sketch may currently extrude/revolve a single
/// standalone circle (with no segments or arcs); mixing a circle with other edges
/// would form multiple loops, which the extrude bridge does not yet support.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Circle {
	/// Center point.
	pub center: PointId,
	/// A point on the circle (its distance from `center` is the radius).
	pub radius_point: PointId,
}

/// A geometric constraint over a sketch's points.
///
/// Directions are taken between point pairs (`a → b`), so the same small set of
/// variants covers both point constraints (`Fixed`, `Coincident`, `Distance`) and
/// line constraints (`Horizontal`, `Vertical`, `Parallel`, `Perpendicular`).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum SketchConstraint {
	/// Pin `point` to the world position `at` (the sketch's ground anchor).
	Fixed {
		/// The anchored point.
		point: PointId,
		/// Where it is held.
		at: DVec2,
	},
	/// Make two points share the same position.
	Coincident {
		/// First point.
		a: PointId,
		/// Second point.
		b: PointId,
	},
	/// Hold the segment `a → b` horizontal (`y_a == y_b`).
	Horizontal {
		/// First endpoint.
		a: PointId,
		/// Second endpoint.
		b: PointId,
	},
	/// Hold the segment `a → b` vertical (`x_a == x_b`).
	Vertical {
		/// First endpoint.
		a: PointId,
		/// Second endpoint.
		b: PointId,
	},
	/// Hold the straight-line distance between `a` and `b` at `distance`.
	Distance {
		/// First point.
		a: PointId,
		/// Second point.
		b: PointId,
		/// Target separation (clamped to `>= 0`).
		distance: f64,
	},
	/// Keep the directions `a → b` and `c → d` parallel.
	Parallel {
		/// Start of the first direction.
		a: PointId,
		/// End of the first direction.
		b: PointId,
		/// Start of the second direction.
		c: PointId,
		/// End of the second direction.
		d: PointId,
	},
	/// Keep the directions `a → b` and `c → d` perpendicular.
	Perpendicular {
		/// Start of the first direction.
		a: PointId,
		/// End of the first direction.
		b: PointId,
		/// Start of the second direction.
		c: PointId,
		/// End of the second direction.
		d: PointId,
	},
	/// Force the segment `a → b` and the segment `c → d` to the same length.
	EqualLength {
		/// Start of the first segment.
		a: PointId,
		/// End of the first segment.
		b: PointId,
		/// Start of the second segment.
		c: PointId,
		/// End of the second segment.
		d: PointId,
	},
	/// Hold the line through `line_a → line_b` tangent to the circle centered at
	/// `center` and passing through `radius_point`: the line's perpendicular
	/// distance from the center equals the radius.
	Tangent {
		/// First point on the line.
		line_a: PointId,
		/// Second point on the line.
		line_b: PointId,
		/// Circle center.
		center: PointId,
		/// A point on the circle (sets the radius).
		radius_point: PointId,
	},
	/// Hold the angle between directions `a → b` and `c → d` at `radians`
	/// (magnitude; the directed sense is not pinned, so the solver may settle at
	/// `±radians`).
	Angle {
		/// Start of the first direction.
		a: PointId,
		/// End of the first direction.
		b: PointId,
		/// Start of the second direction.
		c: PointId,
		/// End of the second direction.
		d: PointId,
		/// Target angle between the two directions, in radians.
		radians: f64,
	},
	/// Make points `a` and `b` mirror images across the line through
	/// `line_a → line_b` (a line of symmetry).
	Symmetric {
		/// First point.
		a: PointId,
		/// Its mirror.
		b: PointId,
		/// First point on the symmetry line.
		line_a: PointId,
		/// Second point on the symmetry line.
		line_b: PointId,
	},
}

/// The outcome of a [`Sketch::solve`] call.
#[derive(Clone, Copy, Debug)]
pub struct SolveReport {
	/// Final sum of squared residuals (`0` ⇒ every constraint satisfied).
	pub residual: f64,
	/// Number of Levenberg–Marquardt iterations actually taken.
	pub iterations: usize,
	/// Whether the residual reached the convergence tolerance.
	pub converged: bool,
}

/// How well a sketch's degrees of freedom are pinned by its constraints.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConstraintState {
	/// Some degree of freedom is still free — the sketch can move.
	UnderConstrained,
	/// Exactly determined: every DOF is pinned with no redundant constraint.
	WellConstrained,
	/// Every DOF is pinned but there are extra (redundant) constraints, which may
	/// be consistent or conflicting.
	OverConstrained,
}

/// A degree-of-freedom analysis of a sketch, from the rank of its constraint
/// Jacobian at the current configuration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SketchAnalysis {
	/// Total degrees of freedom (`2 × point count`).
	pub dof: usize,
	/// Number of independent constraints (rank of the Jacobian).
	pub rank: usize,
	/// Degrees of freedom left unconstrained (`dof − rank`).
	pub free_dof: usize,
	/// Constraint rows beyond the independent set (`constraint rows − rank`).
	pub redundant: usize,
	/// Summary classification.
	pub state: ConstraintState,
}

/// Why a sketch could not be turned into geometry.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SketchError {
	/// Fewer than three points/segments, or the loop encloses no area.
	Degenerate,
	/// The segments do not form a single closed loop (open chain, branch, or
	/// several disjoint loops).
	NotClosed,
	/// The profile was valid but the sweep produced no solid (e.g. zero height).
	EmptySolid,
}

/// Converged when the summed squared residual drops below this.
const CONVERGED: f64 = 1e-18;

/// A 2D sketch: points, the segments between them, and the constraints to solve.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Sketch {
	points: Vec<DVec2>,
	segments: Vec<Segment>,
	arcs: Vec<Arc>,
	circles: Vec<Circle>,
	constraints: Vec<SketchConstraint>,
}

impl Sketch {
	/// An empty sketch.
	pub fn new() -> Self {
		Self::default()
	}

	/// Add a point at an initial position, returning its [`PointId`].
	pub fn add_point(&mut self, position: DVec2) -> PointId {
		let id = self.points.len();
		self.points.push(position);
		id
	}

	/// Connect two points with a segment, returning its index.
	pub fn add_segment(&mut self, a: PointId, b: PointId) -> usize {
		let id = self.segments.len();
		self.segments.push(Segment { a, b });
		id
	}

	/// Connect two boundary points with a circular arc about `center`, sweeping
	/// counter-clockwise from `a` to `b` when `ccw` (otherwise clockwise). Returns
	/// the arc's index.
	pub fn add_arc(&mut self, a: PointId, b: PointId, center: PointId, ccw: bool) -> usize {
		let id = self.arcs.len();
		self.arcs.push(Arc { a, b, center, ccw });
		id
	}

	/// Add a standalone full circle (a `center` point and a `radius_point` on it),
	/// returning its index. Used as a complete closed profile on its own.
	pub fn add_circle(&mut self, center: PointId, radius_point: PointId) -> usize {
		let id = self.circles.len();
		self.circles.push(Circle { center, radius_point });
		id
	}

	/// Add a constraint to the system, returning its index. The index addresses the
	/// constraint for later parametric overrides ([`Sketch::set_distance`]).
	pub fn add_constraint(&mut self, constraint: SketchConstraint) -> usize {
		let id = self.constraints.len();
		self.constraints.push(constraint);
		id
	}

	/// Override the target value of the [`SketchConstraint::Distance`] at `constraint`,
	/// returning `false` if the index is out of range or that constraint is not a
	/// `Distance`. Used by the parametric feature layer to drive a sketch *dimension*
	/// from a [`Document`](crate::Document) parameter before re-solving.
	pub fn set_distance(&mut self, constraint: usize, distance: f64) -> bool {
		match self.constraints.get_mut(constraint) {
			Some(SketchConstraint::Distance { distance: value, .. }) => {
				*value = distance;
				true
			}
			_ => false,
		}
	}

	/// The current point positions (initial, or solved after [`Sketch::solve`]).
	pub fn points(&self) -> &[DVec2] {
		&self.points
	}

	/// The current position of one point (`DVec2::ZERO` if `id` is out of range).
	pub fn point(&self, id: PointId) -> DVec2 {
		self.points.get(id).copied().unwrap_or(DVec2::ZERO)
	}

	/// Sum of squared residuals of the current positions (`0` ⇒ all satisfied).
	pub fn residual(&self) -> f64 {
		let x = self.flatten();
		sq_norm(&self.residual_vec(&x))
	}

	/// Solve the constraint system, moving the points to satisfy the constraints.
	///
	/// Runs up to a generous default iteration budget; see [`Sketch::solve_with`]
	/// to bound it explicitly. Idempotent on an already-solved sketch.
	pub fn solve(&mut self) -> SolveReport {
		self.solve_with(128)
	}

	/// Solve with an explicit iteration cap (Levenberg–Marquardt).
	pub fn solve_with(&mut self, max_iterations: usize) -> SolveReport {
		let ndof = self.points.len() * 2;
		if ndof == 0 {
			return SolveReport { residual: 0.0, iterations: 0, converged: true };
		}
		let mut x = self.flatten();
		let mut r = self.residual_vec(&x);
		let mut cost = sq_norm(&r);
		let mut lambda = 1e-3_f64;
		let mut iterations = 0;

		while iterations < max_iterations && cost > CONVERGED {
			iterations += 1;
			let m = r.len();
			if m == 0 {
				break;
			}

			// Central-difference Jacobian J (m × ndof, row-major).
			let (jac, _) = self.jacobian(&x);

			// Normal equations: A = JᵀJ, g = Jᵀr.
			let mut a = vec![0.0; ndof * ndof];
			let mut g = vec![0.0; ndof];
			for k in 0..m {
				for i in 0..ndof {
					let jki = jac[k * ndof + i];
					if jki == 0.0 {
						continue;
					}
					g[i] += jki * r[k];
					for j in 0..ndof {
						a[i * ndof + j] += jki * jac[k * ndof + j];
					}
				}
			}

			// Adaptive Levenberg–Marquardt: grow λ until a damped step decreases the
			// cost (or give up this iteration). Damping is scaled by each diagonal so
			// it is invariant to the relative scale of the coordinates.
			let mut stepped = false;
			for _ in 0..12 {
				let mut damped = a.clone();
				for d in 0..ndof {
					damped[d * ndof + d] += lambda * a[d * ndof + d].max(1e-12);
				}
				let neg_g: Vec<f64> = g.iter().map(|v| -v).collect();
				let Some(dx) = solve_dense(&damped, &neg_g, ndof) else {
					lambda = (lambda * 4.0).min(1e12);
					continue;
				};
				let x_new: Vec<f64> = x.iter().zip(&dx).map(|(xi, d)| xi + d).collect();
				let r_new = self.residual_vec(&x_new);
				let cost_new = sq_norm(&r_new);
				if cost_new < cost {
					x = x_new;
					r = r_new;
					cost = cost_new;
					lambda = (lambda * 0.5).max(1e-12);
					stepped = true;
					break;
				}
				lambda = (lambda * 4.0).min(1e12);
			}
			if !stepped {
				break; // no damped step improves the cost: converged or stuck.
			}
		}

		for (i, p) in self.points.iter_mut().enumerate() {
			p.x = x[2 * i];
			p.y = x[2 * i + 1];
		}
		SolveReport { residual: cost, iterations, converged: cost <= CONVERGED }
	}

	/// Recover the closed boundary loop as an ordered, CCW list of points.
	///
	/// Walks the segments (each interior point must be shared by exactly two of
	/// them) into a single cycle, then orients it counter-clockwise so it can feed
	/// [`kernel_brep::extrude`]. Returns [`SketchError`] if the segments do not form
	/// exactly one closed loop.
	pub fn profile(&self) -> Result<Vec<DVec2>, SketchError> {
		let n = self.points.len();
		// A standalone full circle is the one closed *curve* (vertex-free) loop the
		// extrude bridge supports today. Combining it with edges or other circles
		// would make several loops, which is not yet supported.
		if !self.circles.is_empty() {
			if self.circles.len() != 1 || !self.segments.is_empty() || !self.arcs.is_empty() {
				return Err(SketchError::NotClosed);
			}
			let circle = self.circles[0];
			if circle.center >= n || circle.radius_point >= n {
				return Err(SketchError::NotClosed);
			}
			let c = self.points[circle.center];
			let rp = self.points[circle.radius_point];
			if (rp - c).length() < 1e-12 {
				return Err(SketchError::Degenerate);
			}
			// A full 2π sweep from the radius point back to itself tessellates the circle.
			let mut boundary = vec![rp];
			append_arc(&mut boundary, rp, rp, c, true);
			if boundary.len() < 3 {
				return Err(SketchError::Degenerate);
			}
			if signed_area(&boundary) < 0.0 {
				boundary.reverse();
			}
			return Ok(boundary);
		}
		// Unified edge list: straight segments and circular arcs both connect two
		// boundary points (an arc's center is a construction point, not on the loop).
		let mut edges: Vec<(usize, usize, EdgeKind)> = Vec::with_capacity(self.segments.len() + self.arcs.len());
		for s in &self.segments {
			edges.push((s.a, s.b, EdgeKind::Line));
		}
		for arc in &self.arcs {
			if arc.center >= n {
				return Err(SketchError::NotClosed);
			}
			edges.push((arc.a, arc.b, EdgeKind::Arc { center: arc.center, ccw: arc.ccw }));
		}
		// A closed loop needs at least two edges (two arcs already enclose an area).
		if edges.len() < 2 || n < 2 {
			return Err(SketchError::Degenerate);
		}

		// Adjacency over endpoints only, carrying the edge index so the walk can tell
		// which curve (and which of two parallel edges) it is traversing.
		let mut adj: Vec<Vec<(usize, usize)>> = vec![Vec::new(); n];
		for (idx, &(a, b, _)) in edges.iter().enumerate() {
			if a >= n || b >= n || a == b {
				return Err(SketchError::NotClosed);
			}
			adj[a].push((b, idx));
			adj[b].push((a, idx));
		}
		// Every point an edge touches as an endpoint must have degree exactly two: a
		// single closed loop, no open ends and no branch/junction.
		if (0..n).any(|i| !adj[i].is_empty() && adj[i].len() != 2) {
			return Err(SketchError::NotClosed);
		}

		let start = edges[0].0;
		let mut boundary: Vec<DVec2> = vec![self.points[start]];
		let mut cur = start;
		let mut prev_edge = usize::MAX;
		let mut traversed = 0;
		loop {
			let Some(&(next_pt, edge_idx)) = adj[cur].iter().find(|&&(_, e)| e != prev_edge) else {
				return Err(SketchError::NotClosed);
			};
			traversed += 1;
			if let EdgeKind::Arc { center, ccw } = edges[edge_idx].2 {
				// The stored arc runs edges[idx].0 → .1 with `ccw`; flip the sense when
				// the walk crosses it backwards.
				let forward = cur == edges[edge_idx].0;
				let sweep_ccw = if forward { ccw } else { !ccw };
				append_arc(&mut boundary, self.points[cur], self.points[next_pt], self.points[center], sweep_ccw);
			}
			if next_pt == start {
				break;
			}
			boundary.push(self.points[next_pt]);
			prev_edge = edge_idx;
			cur = next_pt;
			if traversed > edges.len() {
				return Err(SketchError::NotClosed);
			}
		}
		// Touching fewer edges than exist means the sketch has several disjoint loops.
		if traversed != edges.len() {
			return Err(SketchError::NotClosed);
		}
		if boundary.len() < 3 || signed_area(&boundary).abs() < 1e-12 {
			return Err(SketchError::Degenerate);
		}
		if signed_area(&boundary) < 0.0 {
			boundary.reverse();
		}
		Ok(boundary)
	}

	/// Extract **every** closed loop in the sketch — each connected component of the
	/// segment/arc graph, plus each standalone circle — as a CCW point ring.
	///
	/// This is the multi-loop generalisation of [`Sketch::profile`]: an outer
	/// boundary plus inner hole loops (a washer). Returns [`SketchError`] if any
	/// component is not a single closed loop (open end / branch).
	pub fn all_loops(&self) -> Result<Vec<Vec<DVec2>>, SketchError> {
		let n = self.points.len();
		let mut loops: Vec<Vec<DVec2>> = Vec::new();
		// Standalone circles: one tessellated loop each.
		for circle in &self.circles {
			if circle.center >= n || circle.radius_point >= n {
				return Err(SketchError::NotClosed);
			}
			let c = self.points[circle.center];
			let rp = self.points[circle.radius_point];
			if (rp - c).length() < 1e-12 {
				return Err(SketchError::Degenerate);
			}
			let mut boundary = vec![rp];
			append_arc(&mut boundary, rp, rp, c, true);
			if boundary.len() < 3 {
				return Err(SketchError::Degenerate);
			}
			if signed_area(&boundary) < 0.0 {
				boundary.reverse();
			}
			loops.push(boundary);
		}
		// Segment / arc edges.
		let mut edges: Vec<(usize, usize, EdgeKind)> = Vec::with_capacity(self.segments.len() + self.arcs.len());
		for s in &self.segments {
			edges.push((s.a, s.b, EdgeKind::Line));
		}
		for arc in &self.arcs {
			if arc.center >= n {
				return Err(SketchError::NotClosed);
			}
			edges.push((arc.a, arc.b, EdgeKind::Arc { center: arc.center, ccw: arc.ccw }));
		}
		if edges.is_empty() {
			return if loops.is_empty() { Err(SketchError::Degenerate) } else { Ok(loops) };
		}
		let mut adj: Vec<Vec<(usize, usize)>> = vec![Vec::new(); n];
		for (idx, &(a, b, _)) in edges.iter().enumerate() {
			if a >= n || b >= n || a == b {
				return Err(SketchError::NotClosed);
			}
			adj[a].push((b, idx));
			adj[b].push((a, idx));
		}
		// Every touched point must have degree exactly two: closed loops, no open ends.
		if (0..n).any(|i| !adj[i].is_empty() && adj[i].len() != 2) {
			return Err(SketchError::NotClosed);
		}
		// Walk every connected component into its own closed loop.
		let mut visited = vec![false; edges.len()];
		for seed in 0..edges.len() {
			if visited[seed] {
				continue;
			}
			let start = edges[seed].0;
			let mut boundary: Vec<DVec2> = vec![self.points[start]];
			let mut cur = start;
			let mut prev_edge = usize::MAX;
			let mut count = 0;
			loop {
				let Some(&(next_pt, edge_idx)) = adj[cur].iter().find(|&&(_, e)| e != prev_edge && !visited[e]) else {
					return Err(SketchError::NotClosed);
				};
				visited[edge_idx] = true;
				count += 1;
				if let EdgeKind::Arc { center, ccw } = edges[edge_idx].2 {
					let forward = cur == edges[edge_idx].0;
					let sweep_ccw = if forward { ccw } else { !ccw };
					append_arc(&mut boundary, self.points[cur], self.points[next_pt], self.points[center], sweep_ccw);
				}
				if next_pt == start {
					break;
				}
				boundary.push(self.points[next_pt]);
				prev_edge = edge_idx;
				cur = next_pt;
				if count > edges.len() {
					return Err(SketchError::NotClosed);
				}
			}
			if boundary.len() < 3 || signed_area(&boundary).abs() < 1e-12 {
				return Err(SketchError::Degenerate);
			}
			if signed_area(&boundary) < 0.0 {
				boundary.reverse();
			}
			loops.push(boundary);
		}
		Ok(loops)
	}

	/// Extrude the sketch along `+Z` by `height` into a [`Solid`]. A single loop
	/// gives a plain prism; multiple loops are treated as an outer boundary (the
	/// largest by area) with the rest as **holes**, yielding a washer / plate.
	/// The resolved `(center, radius)` if this sketch is a single standalone circle (no
	/// segments or arcs) — the case that extrudes to an exact analytic cylinder.
	fn single_circle(&self) -> Option<(DVec2, f64)> {
		if self.circles.len() != 1 || !self.segments.is_empty() || !self.arcs.is_empty() {
			return None;
		}
		let c = self.circles[0];
		let n = self.points.len();
		if c.center >= n || c.radius_point >= n {
			return None;
		}
		let center = self.points[c.center];
		let radius = (self.points[c.radius_point] - center).length();
		(radius >= 1e-12).then_some((center, radius))
	}

	pub fn extrude(&self, height: f64) -> Result<Solid, SketchError> {
		// A standalone circle extrudes to an EXACT analytic cylinder (one Surface::Cylinder
		// side face + Curve::Circle rims), not a faceted 16-gon prism — so a sketched
		// boss/peg/hole is micron-precise and tessellate_adaptive_tol can refine its wall to
		// any chord tolerance, exactly like the cylinder() primitive.
		if let Some((center, radius)) = self.single_circle() {
			let solid = brep_cylinder(DVec3::new(center.x, center.y, 0.0), DVec3::Z, radius, height, 64);
			if solid.face_count() == 0 {
				return Err(SketchError::EmptySolid);
			}
			return Ok(solid);
		}
		let loops = self.all_loops()?;
		let solid = if loops.len() == 1 {
			brep_extrude(&loops[0], height)
		} else {
			let outer_idx = (0..loops.len())
				.max_by(|&a, &b| {
					signed_area(&loops[a])
						.abs()
						.partial_cmp(&signed_area(&loops[b]).abs())
						.unwrap_or(std::cmp::Ordering::Equal)
				})
				.unwrap();
			let outer = loops[outer_idx].clone();
			let holes: Vec<Vec<DVec2>> = loops.iter().enumerate().filter(|(i, _)| *i != outer_idx).map(|(_, l)| l.clone()).collect();
			brep_extrude_with_holes(&outer, &holes, height)
		};
		if solid.face_count() == 0 {
			return Err(SketchError::EmptySolid);
		}
		Ok(solid)
	}

	/// Extrude the sketch along `+Z` by `height` with a **draft** of `draft` radians,
	/// so every wall slopes inward and the part releases from a mould / die (see
	/// [`kernel_brep::extrude_tapered`]). `draft == 0` is identical to [`Sketch::extrude`]
	/// (and keeps full multi-loop / hole support). With a nonzero draft only the outer
	/// boundary (the largest loop by area) is drafted; drafting around holes is not
	/// supported yet, so any inner loops are ignored.
	pub fn extrude_tapered(&self, height: f64, draft: f64) -> Result<Solid, SketchError> {
		if draft == 0.0 {
			return self.extrude(height);
		}
		let loops = self.all_loops()?;
		let outer_idx = (0..loops.len())
			.max_by(|&a, &b| {
				signed_area(&loops[a])
					.abs()
					.partial_cmp(&signed_area(&loops[b]).abs())
					.unwrap_or(std::cmp::Ordering::Equal)
			})
			.unwrap();
		let solid = brep_extrude_tapered(&loops[outer_idx], height, draft);
		if solid.face_count() == 0 {
			return Err(SketchError::EmptySolid);
		}
		Ok(solid)
	}

	/// Revolve the solved profile about the `Z` axis into a [`Solid`], faceted into
	/// `segments` sectors. The sketch's `(x, y)` is interpreted as `(radius, z)`
	/// (radii must be `>= 0`), matching [`kernel_brep::revolve`].
	pub fn revolve(&self, segments: usize) -> Result<Solid, SketchError> {
		let profile = self.profile()?;
		let solid = brep_revolve(&profile, segments);
		if solid.face_count() == 0 {
			return Err(SketchError::EmptySolid);
		}
		Ok(solid)
	}

	/// Classify how well the constraints pin the sketch, from the rank of the
	/// constraint Jacobian at the current point positions.
	///
	/// `free_dof` counts the still-movable degrees of freedom and `redundant`
	/// counts constraint rows beyond the independent set; the `state` label reports
	/// [`ConstraintState::UnderConstrained`] whenever any DOF is free, otherwise
	/// [`ConstraintState::OverConstrained`] if any constraint is redundant, else
	/// [`ConstraintState::WellConstrained`]. (A sketch can be simultaneously
	/// under-constrained in one place and over-constrained in another; the two
	/// counts expose that even though the single label prioritizes "under".)
	pub fn analyze(&self) -> SketchAnalysis {
		let x = self.flatten();
		let (jac, m) = self.jacobian(&x);
		let dof = x.len();
		let rank = matrix_rank(&jac, m, dof);
		let free_dof = dof.saturating_sub(rank);
		let redundant = m.saturating_sub(rank);
		let state = if free_dof > 0 {
			ConstraintState::UnderConstrained
		} else if redundant > 0 {
			ConstraintState::OverConstrained
		} else {
			ConstraintState::WellConstrained
		};
		SketchAnalysis { dof, rank, free_dof, redundant, state }
	}

	/// Central-difference constraint Jacobian `J` (`m × ndof`, row-major) at the
	/// coordinate vector `x`, returned with the residual count `m`. Shared by the
	/// solver and by [`Sketch::analyze`].
	fn jacobian(&self, x: &[f64]) -> (Vec<f64>, usize) {
		let ndof = x.len();
		let m = self.residual_vec(x).len();
		let mut jac = vec![0.0; m * ndof];
		let mut xx = x.to_vec();
		for col in 0..ndof {
			let h = 1e-6 * (1.0 + xx[col].abs());
			let saved = xx[col];
			xx[col] = saved + h;
			let rp = self.residual_vec(&xx);
			xx[col] = saved - h;
			let rm = self.residual_vec(&xx);
			xx[col] = saved;
			for k in 0..m {
				jac[k * ndof + col] = (rp[k] - rm[k]) / (2.0 * h);
			}
		}
		(jac, m)
	}

	/// Flatten the points to the solver's `[x₀, y₀, x₁, y₁, …]` coordinate vector.
	fn flatten(&self) -> Vec<f64> {
		let mut x = Vec::with_capacity(self.points.len() * 2);
		for p in &self.points {
			x.push(p.x);
			x.push(p.y);
		}
		x
	}

	/// Stack every constraint's residual(s) for the coordinate vector `x`. A
	/// constraint referencing an out-of-range point contributes nothing (rather
	/// than panicking), mirroring the assembly solver's defensive handling.
	fn residual_vec(&self, x: &[f64]) -> Vec<f64> {
		let n = self.points.len();
		let p = |i: PointId| -> Option<DVec2> {
			if i < n {
				Some(DVec2::new(x[2 * i], x[2 * i + 1]))
			} else {
				None
			}
		};
		let mut r = Vec::with_capacity(self.constraints.len());
		for c in &self.constraints {
			match *c {
				SketchConstraint::Fixed { point, at } => {
					if let Some(pt) = p(point) {
						r.push(pt.x - at.x);
						r.push(pt.y - at.y);
					}
				}
				SketchConstraint::Coincident { a, b } => {
					if let (Some(pa), Some(pb)) = (p(a), p(b)) {
						r.push(pa.x - pb.x);
						r.push(pa.y - pb.y);
					}
				}
				SketchConstraint::Horizontal { a, b } => {
					if let (Some(pa), Some(pb)) = (p(a), p(b)) {
						r.push(pa.y - pb.y);
					}
				}
				SketchConstraint::Vertical { a, b } => {
					if let (Some(pa), Some(pb)) = (p(a), p(b)) {
						r.push(pa.x - pb.x);
					}
				}
				SketchConstraint::Distance { a, b, distance } => {
					if let (Some(pa), Some(pb)) = (p(a), p(b)) {
						r.push((pb - pa).length() - distance.max(0.0));
					}
				}
				SketchConstraint::Parallel { a, b, c, d } => {
					if let (Some(pa), Some(pb), Some(pc), Some(pd)) = (p(a), p(b), p(c), p(d)) {
						// Cross product of the two directions vanishes iff parallel.
						r.push((pb - pa).perp_dot(pd - pc));
					}
				}
				SketchConstraint::Perpendicular { a, b, c, d } => {
					if let (Some(pa), Some(pb), Some(pc), Some(pd)) = (p(a), p(b), p(c), p(d)) {
						// Dot product of the two directions vanishes iff perpendicular.
						r.push((pb - pa).dot(pd - pc));
					}
				}
				SketchConstraint::EqualLength { a, b, c, d } => {
					if let (Some(pa), Some(pb), Some(pc), Some(pd)) = (p(a), p(b), p(c), p(d)) {
						r.push((pb - pa).length() - (pd - pc).length());
					}
				}
				SketchConstraint::Tangent { line_a, line_b, center, radius_point } => {
					if let (Some(pa), Some(pb), Some(pc), Some(pr)) = (p(line_a), p(line_b), p(center), p(radius_point)) {
						let dir = pb - pa;
						let len = dir.length();
						if len > 1e-12 {
							// Perpendicular distance of the center from the line minus the radius.
							let dist = dir.perp_dot(pc - pa).abs() / len;
							r.push(dist - (pr - pc).length());
						}
					}
				}
				SketchConstraint::Angle { a, b, c, d, radians } => {
					if let (Some(pa), Some(pb), Some(pc), Some(pd)) = (p(a), p(b), p(c), p(d)) {
						let u = (pb - pa).normalize_or_zero();
						let v = (pd - pc).normalize_or_zero();
						if u.length_squared() > 0.5 && v.length_squared() > 0.5 {
							// Branch-free angle-magnitude residual: cos(angle) − cos(target).
							r.push(u.dot(v) - radians.cos());
						}
					}
				}
				SketchConstraint::Symmetric { a, b, line_a, line_b } => {
					if let (Some(pa), Some(pb), Some(la), Some(lb)) = (p(a), p(b), p(line_a), p(line_b)) {
						let dir = (lb - la).normalize_or_zero();
						if dir.length_squared() > 0.5 {
							// Reflect `a` across the line and require it to meet `b`.
							let rel = pa - la;
							let reflected = la + dir * (2.0 * rel.dot(dir)) - rel;
							r.push(reflected.x - pb.x);
							r.push(reflected.y - pb.y);
						}
					}
				}
			}
		}
		r
	}
}

/// Whether a profile edge is straight or a circular arc.
#[derive(Clone, Copy)]
enum EdgeKind {
	/// A straight segment.
	Line,
	/// A circular arc about `center`, swept CCW from the edge's first endpoint when `ccw`.
	Arc {
		/// Center point index.
		center: usize,
		/// Counter-clockwise sweep.
		ccw: bool,
	},
}

/// Append the *interior* tessellation points of the arc from `from` to `to` about
/// `center` (sweeping counter-clockwise when `ccw`) to `boundary`. The endpoints
/// are omitted — `from` is already present and `to` is appended by the caller. The
/// radius is interpolated between the two endpoints so a slightly off-radius
/// endpoint produces a smooth arc rather than a kink.
fn append_arc(boundary: &mut Vec<DVec2>, from: DVec2, to: DVec2, center: DVec2, ccw: bool) {
	let v0 = from - center;
	let v1 = to - center;
	let r0 = v0.length();
	let r1 = v1.length();
	if r0 < 1e-12 || r1 < 1e-12 {
		return;
	}
	let a0 = v0.y.atan2(v0.x);
	let a1 = v1.y.atan2(v1.x);
	let mut sweep = a1 - a0;
	let tau = std::f64::consts::TAU;
	if ccw {
		while sweep <= 1e-12 {
			sweep += tau;
		}
	} else {
		while sweep >= -1e-12 {
			sweep -= tau;
		}
	}
	// About one chord per 22.5° of sweep (16 per full turn), at least one.
	let n = ((sweep.abs() / std::f64::consts::FRAC_PI_8).ceil() as usize).max(1);
	for i in 1..n {
		let t = i as f64 / n as f64;
		let ang = a0 + sweep * t;
		let r = r0 + (r1 - r0) * t;
		boundary.push(center + DVec2::new(ang.cos(), ang.sin()) * r);
	}
}

/// Twice-signed-area test: positive area means the loop is counter-clockwise.
fn signed_area(pts: &[DVec2]) -> f64 {
	let n = pts.len();
	let mut s = 0.0;
	for i in 0..n {
		let a = pts[i];
		let b = pts[(i + 1) % n];
		s += a.x * b.y - b.x * a.y;
	}
	0.5 * s
}

/// Sum of squares of a residual vector.
fn sq_norm(r: &[f64]) -> f64 {
	r.iter().map(|v| v * v).sum()
}

/// Numerical rank of an `m × n` row-major matrix by Gaussian elimination with
/// partial pivoting, counting pivots above a scale-relative tolerance.
fn matrix_rank(mat: &[f64], m: usize, n: usize) -> usize {
	if m == 0 || n == 0 {
		return 0;
	}
	let mut a = mat.to_vec();
	let max_abs = a.iter().fold(0.0_f64, |acc, &v| acc.max(v.abs()));
	let tol = 1e-9 * max_abs.max(1.0);
	let mut rank = 0;
	let mut row = 0;
	for col in 0..n {
		if row >= m {
			break;
		}
		// Largest-magnitude pivot in this column at or below the current row.
		let mut pivot = row;
		let mut best = a[row * n + col].abs();
		for r in (row + 1)..m {
			let v = a[r * n + col].abs();
			if v > best {
				best = v;
				pivot = r;
			}
		}
		if best <= tol {
			continue; // column is dependent on the ones already used
		}
		if pivot != row {
			for c in 0..n {
				a.swap(row * n + c, pivot * n + c);
			}
		}
		let diag = a[row * n + col];
		for r in (row + 1)..m {
			let factor = a[r * n + col] / diag;
			if factor != 0.0 {
				for c in col..n {
					a[r * n + c] -= factor * a[row * n + c];
				}
			}
		}
		rank += 1;
		row += 1;
	}
	rank
}

/// Solve the dense linear system `A z = b` (`A` row-major `n × n`) by Gauss–Jordan
/// elimination with partial pivoting. Returns `None` if `A` is singular to working
/// precision. Small `n` only — the sketch's DOF count.
fn solve_dense(a: &[f64], b: &[f64], n: usize) -> Option<Vec<f64>> {
	let mut m = a.to_vec();
	let mut x = b.to_vec();
	for col in 0..n {
		// Partial pivot: largest magnitude in this column at or below the diagonal.
		let mut pivot = col;
		let mut best = m[col * n + col].abs();
		for row in (col + 1)..n {
			let v = m[row * n + col].abs();
			if v > best {
				best = v;
				pivot = row;
			}
		}
		if best < 1e-14 {
			return None;
		}
		if pivot != col {
			for c in 0..n {
				m.swap(col * n + c, pivot * n + c);
			}
			x.swap(col, pivot);
		}
		let diag = m[col * n + col];
		for row in 0..n {
			if row == col {
				continue;
			}
			let factor = m[row * n + col] / diag;
			if factor == 0.0 {
				continue;
			}
			for c in col..n {
				m[row * n + c] -= factor * m[col * n + c];
			}
			x[row] -= factor * x[col];
		}
	}
	let mut z = vec![0.0; n];
	for i in 0..n {
		z[i] = x[i] / m[i * n + i];
	}
	Some(z)
}

#[cfg(test)]
mod tests {
	use super::*;
	use kernel_brep::validate;

	/// Build an unconstrained, roughly-placed quadrilateral and the four segments
	/// that close it, returning the sketch plus its corner ids in order.
	fn rough_quad() -> (Sketch, [PointId; 4]) {
		let mut s = Sketch::new();
		let p0 = s.add_point(DVec2::new(0.1, -0.2));
		let p1 = s.add_point(DVec2::new(3.0, 0.05));
		let p2 = s.add_point(DVec2::new(2.9, 1.8));
		let p3 = s.add_point(DVec2::new(-0.1, 2.1));
		s.add_segment(p0, p1);
		s.add_segment(p1, p2);
		s.add_segment(p2, p3);
		s.add_segment(p3, p0);
		(s, [p0, p1, p2, p3])
	}

	#[test]
	fn constrained_rectangle_solves_to_exact_corners() {
		// A rough quad fully constrained into a 4×2 rectangle anchored at the origin:
		// bottom/top horizontal, left/right vertical, width and height distances.
		let (mut s, [p0, p1, p2, p3]) = rough_quad();
		s.add_constraint(SketchConstraint::Fixed { point: p0, at: DVec2::ZERO });
		s.add_constraint(SketchConstraint::Horizontal { a: p0, b: p1 });
		s.add_constraint(SketchConstraint::Horizontal { a: p3, b: p2 });
		s.add_constraint(SketchConstraint::Vertical { a: p0, b: p3 });
		s.add_constraint(SketchConstraint::Vertical { a: p1, b: p2 });
		s.add_constraint(SketchConstraint::Distance { a: p0, b: p1, distance: 4.0 });
		s.add_constraint(SketchConstraint::Distance { a: p0, b: p3, distance: 2.0 });

		let report = s.solve();

		let rounded: Vec<(f64, f64)> = s
			.points()
			.iter()
			.map(|p| ((p.x * 1e6).round() / 1e6, (p.y * 1e6).round() / 1e6))
			.collect();
		assert_eq!(
			(report.converged, rounded),
			(true, vec![(0.0, 0.0), (4.0, 0.0), (4.0, 2.0), (0.0, 2.0)]),
			"rectangle constraints should resolve exactly; residual {}",
			report.residual
		);
	}

	#[test]
	fn solved_sketch_extrudes_to_a_validated_prism() {
		// The constrained rectangle, swept 5mm, must be a closed manifold solid of
		// volume 4 × 2 × 5 = 40 (extrusion of a planar profile is exact).
		let (mut s, [p0, p1, p2, p3]) = rough_quad();
		s.add_constraint(SketchConstraint::Fixed { point: p0, at: DVec2::ZERO });
		s.add_constraint(SketchConstraint::Horizontal { a: p0, b: p1 });
		s.add_constraint(SketchConstraint::Horizontal { a: p3, b: p2 });
		s.add_constraint(SketchConstraint::Vertical { a: p0, b: p3 });
		s.add_constraint(SketchConstraint::Vertical { a: p1, b: p2 });
		s.add_constraint(SketchConstraint::Distance { a: p0, b: p1, distance: 4.0 });
		s.add_constraint(SketchConstraint::Distance { a: p0, b: p3, distance: 2.0 });
		s.solve();

		let solid = s.extrude(5.0).expect("constrained rectangle should extrude");
		let v = validate::validate(&solid);
		assert_eq!(
			(v.closed, v.manifold, (validate::volume(&solid) * 1e6).round() / 1e6),
			(true, true, 40.0),
			"extruded rectangle should be a closed manifold prism of volume 40"
		);
	}

	#[test]
	fn sketch_with_a_hole_extrudes_to_a_washer() {
		// A 6×6 outer square loop + a 2×2 inner square loop → the sketch detects two
		// loops, treats the larger as the outer boundary and the smaller as a hole, and
		// extrudes a WASHER: a closed manifold genus-1 solid of volume (36−4)·2 = 64.
		let mut s = Sketch::new();
		let o: Vec<usize> = [(-3.0, -3.0), (3.0, -3.0), (3.0, 3.0), (-3.0, 3.0)]
			.into_iter()
			.map(|(x, y)| s.add_point(DVec2::new(x, y)))
			.collect();
		for i in 0..4 {
			s.add_segment(o[i], o[(i + 1) % 4]);
		}
		let h: Vec<usize> = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
			.into_iter()
			.map(|(x, y)| s.add_point(DVec2::new(x, y)))
			.collect();
		for i in 0..4 {
			s.add_segment(h[i], h[(i + 1) % 4]);
		}

		let solid = s.extrude(2.0).expect("sketch with a hole should extrude to a washer");
		let v = validate::validate(&solid);
		let exact = (36.0 - 4.0) * 2.0;
		assert!(
			v.is_valid() && v.genus == 1 && (validate::volume(&solid).abs() - exact).abs() < 1e-6,
			"washer from a sketch hole should be a genus-1 solid of volume {exact}: {v:?} vol={}",
			validate::volume(&solid).abs()
		);
	}

	#[test]
	fn parallel_and_perpendicular_constraints_are_satisfied() {
		// Two free segments off a fixed base: one forced parallel to the base, one
		// forced perpendicular to it. Anchor enough DOFs that the system is well posed.
		let mut s = Sketch::new();
		let o = s.add_point(DVec2::new(0.0, 0.0));
		let bx = s.add_point(DVec2::new(1.0, 0.0)); // base direction o->bx (the +x axis)
		let par = s.add_point(DVec2::new(0.3, 1.0)); // should slide to y = 0 line dir
		let per = s.add_point(DVec2::new(1.0, 0.4)); // should slide to the +y direction
		s.add_constraint(SketchConstraint::Fixed { point: o, at: DVec2::ZERO });
		s.add_constraint(SketchConstraint::Fixed { point: bx, at: DVec2::new(1.0, 0.0) });
		// Keep the moving points at a definite distance so they cannot collapse onto o.
		s.add_constraint(SketchConstraint::Distance { a: o, b: par, distance: 1.0 });
		s.add_constraint(SketchConstraint::Distance { a: o, b: per, distance: 1.0 });
		s.add_constraint(SketchConstraint::Parallel { a: o, b: bx, c: o, d: par });
		s.add_constraint(SketchConstraint::Perpendicular { a: o, b: bx, c: o, d: per });

		let report = s.solve();

		let base = s.point(bx) - s.point(o);
		let par_dir = s.point(par) - s.point(o);
		let per_dir = s.point(per) - s.point(o);
		assert!(
			base.perp_dot(par_dir).abs() < 1e-6 && base.dot(per_dir).abs() < 1e-6 && report.converged,
			"parallel cross {} and perpendicular dot {} should vanish (residual {})",
			base.perp_dot(par_dir),
			base.dot(per_dir),
			report.residual
		);
	}

	#[test]
	fn angle_constraint_holds_the_target_angle() {
		// A moving segment off a fixed +x base, constrained to 60° between them.
		let mut s = Sketch::new();
		let o = s.add_point(DVec2::ZERO);
		let bx = s.add_point(DVec2::new(1.0, 0.0));
		let m = s.add_point(DVec2::new(1.0, 1.0));
		s.add_constraint(SketchConstraint::Fixed { point: o, at: DVec2::ZERO });
		s.add_constraint(SketchConstraint::Fixed { point: bx, at: DVec2::new(1.0, 0.0) });
		s.add_constraint(SketchConstraint::Distance { a: o, b: m, distance: 1.0 });
		s.add_constraint(SketchConstraint::Angle { a: o, b: bx, c: o, d: m, radians: std::f64::consts::FRAC_PI_3 });

		let report = s.solve();

		let u = (s.point(bx) - s.point(o)).normalize_or_zero();
		let v = (s.point(m) - s.point(o)).normalize_or_zero();
		let ang = u.dot(v).clamp(-1.0, 1.0).acos();
		assert!(
			(ang - std::f64::consts::FRAC_PI_3).abs() < 1e-4 && report.converged,
			"angle should be 60°, got {} rad (residual {})",
			ang,
			report.residual
		);
	}

	#[test]
	fn symmetric_constraint_mirrors_points_across_a_line() {
		// Mirror point `a` (fixed at (3,2)) across the y-axis: `b` must move to (−3,2).
		let mut s = Sketch::new();
		let la = s.add_point(DVec2::ZERO);
		let lb = s.add_point(DVec2::new(0.0, 1.0)); // symmetry line = the y-axis
		let a = s.add_point(DVec2::new(3.0, 2.0));
		let b = s.add_point(DVec2::new(1.0, 5.0));
		s.add_constraint(SketchConstraint::Fixed { point: la, at: DVec2::ZERO });
		s.add_constraint(SketchConstraint::Fixed { point: lb, at: DVec2::new(0.0, 1.0) });
		s.add_constraint(SketchConstraint::Fixed { point: a, at: DVec2::new(3.0, 2.0) });
		s.add_constraint(SketchConstraint::Symmetric { a, b, line_a: la, line_b: lb });

		let report = s.solve();

		let pb = s.point(b);
		assert!(
			(pb - DVec2::new(-3.0, 2.0)).length() < 1e-4 && report.converged,
			"b should mirror to (−3, 2), got {pb:?} (residual {})",
			report.residual
		);
	}

	/// Add the seven constraints that fully determine the rough quad as a 4×2
	/// rectangle anchored at the origin.
	fn fully_constrain_rectangle(s: &mut Sketch, [p0, p1, p2, p3]: [PointId; 4]) {
		s.add_constraint(SketchConstraint::Fixed { point: p0, at: DVec2::ZERO });
		s.add_constraint(SketchConstraint::Horizontal { a: p0, b: p1 });
		s.add_constraint(SketchConstraint::Horizontal { a: p3, b: p2 });
		s.add_constraint(SketchConstraint::Vertical { a: p0, b: p3 });
		s.add_constraint(SketchConstraint::Vertical { a: p1, b: p2 });
		s.add_constraint(SketchConstraint::Distance { a: p0, b: p1, distance: 4.0 });
		s.add_constraint(SketchConstraint::Distance { a: p0, b: p3, distance: 2.0 });
	}

	#[test]
	fn dof_analysis_classifies_under_well_and_over_constrained() {
		// Well: the seven constraints exactly pin all 8 DOF (rank 8, no redundancy).
		let (mut well, ids) = rough_quad();
		fully_constrain_rectangle(&mut well, ids);

		// Under: drop the height distance — one DOF stays free.
		let (mut under, uids) = rough_quad();
		fully_constrain_rectangle(&mut under, uids);
		under.constraints.pop();

		// Over: the full set plus a duplicate width distance — one redundant row.
		let (mut over, oids) = rough_quad();
		fully_constrain_rectangle(&mut over, oids);
		over.add_constraint(SketchConstraint::Distance { a: oids[0], b: oids[1], distance: 4.0 });

		assert_eq!(
			(well.analyze().state, under.analyze().state, over.analyze().state),
			(
				ConstraintState::WellConstrained,
				ConstraintState::UnderConstrained,
				ConstraintState::OverConstrained
			),
			"well={:?} under={:?} over={:?}",
			well.analyze(),
			under.analyze(),
			over.analyze()
		);
	}

	#[test]
	fn conflicting_constraints_report_non_convergence_not_false_success() {
		// Two INCOMPATIBLE distance constraints on the same pair (10 and 20). Fixed
		// pins p0 and Horizontal pins the angle, so every DOF is determined and the
		// conflict lands on a pinned DOF (OverConstrained). The load-bearing honesty
		// property: solve() must report converged=false with a large residual, never
		// a false success that would ship silently-wrong geometry. (Without the
		// Horizontal the angle is genuinely free, so the analysis would correctly
		// read UnderConstrained even while the radius conflicts — a separate axis.)
		let mut s = Sketch::new();
		let p0 = s.add_point(DVec2::ZERO);
		let p1 = s.add_point(DVec2::new(7.0, 3.0));
		s.add_constraint(SketchConstraint::Fixed { point: p0, at: DVec2::ZERO });
		s.add_constraint(SketchConstraint::Horizontal { a: p0, b: p1 });
		s.add_constraint(SketchConstraint::Distance { a: p0, b: p1, distance: 10.0 });
		s.add_constraint(SketchConstraint::Distance { a: p0, b: p1, distance: 20.0 });
		let state = s.analyze().state;
		let report = s.solve();
		// The least-squares optimum is |p0 p1| = 15, residual (15-10)^2+(15-20)^2 = 50.
		assert!(
			state == ConstraintState::OverConstrained && !report.converged && report.residual > 1.0,
			"conflicting distances must be over-constrained AND not converge: state={state:?} converged={} residual={}",
			report.converged,
			report.residual
		);
	}

	/// A unit circle centered at the origin, built from two semicircle arcs between
	/// the points (1,0) and (−1,0).
	fn unit_circle() -> Sketch {
		let mut s = Sketch::new();
		let a = s.add_point(DVec2::new(1.0, 0.0));
		let b = s.add_point(DVec2::new(-1.0, 0.0));
		let c = s.add_point(DVec2::ZERO);
		s.add_arc(a, b, c, true); // top half, CCW through (0, 1)
		s.add_arc(a, b, c, false); // bottom half, CW through (0, -1)
		s
	}

	#[test]
	fn arc_profile_points_lie_on_the_circle() {
		// Tessellating the two arcs must land every boundary point exactly on the
		// unit circle (radius 1) and produce a many-sided polygon.
		let prof = unit_circle().profile().expect("two arcs close into a circle");
		let max_radius_error = prof.iter().map(|p| (p.length() - 1.0).abs()).fold(0.0_f64, f64::max);
		assert!(
			prof.len() >= 8 && max_radius_error < 1e-9,
			"circle profile: {} points, max radius error {max_radius_error}",
			prof.len()
		);
	}

	#[test]
	fn arc_profile_extrudes_to_a_closed_solid() {
		// The tessellated circle, extruded 3mm, must be a closed manifold solid whose
		// volume is exactly its profile-polygon area × height (extrusion is exact).
		let s = unit_circle();
		let area = signed_area(&s.profile().unwrap()).abs();
		let solid = s.extrude(3.0).expect("circle extrudes");
		let v = validate::validate(&solid);
		// Round to 1e-6 — the solid's tetra-sum volume and the shoelace area are the
		// same quantity computed two ways, so they agree only to FP summation noise.
		assert_eq!(
			(v.closed, v.manifold, (validate::volume(&solid) * 1e6).round() / 1e6),
			(true, true, (area * 3.0 * 1e6).round() / 1e6),
			"extruded circle should be a closed manifold of volume area×height"
		);
	}

	#[test]
	fn equal_length_makes_two_segments_equal() {
		// Segment o→p is pinned to length 3; segment o→q (held on the y axis) is forced
		// equal, so it must grow to length 3 as well.
		let mut s = Sketch::new();
		let o = s.add_point(DVec2::ZERO);
		let p = s.add_point(DVec2::new(3.0, 0.0));
		let q = s.add_point(DVec2::new(0.0, 1.0));
		s.add_constraint(SketchConstraint::Fixed { point: o, at: DVec2::ZERO });
		s.add_constraint(SketchConstraint::Fixed { point: p, at: DVec2::new(3.0, 0.0) });
		s.add_constraint(SketchConstraint::Vertical { a: o, b: q });
		s.add_constraint(SketchConstraint::EqualLength { a: o, b: p, c: o, d: q });

		let report = s.solve();

		let l1 = (s.point(p) - s.point(o)).length();
		let l2 = (s.point(q) - s.point(o)).length();
		assert!(
			(l1 - l2).abs() < 1e-6 && (l2 - 3.0).abs() < 1e-6 && report.converged,
			"equal-length should make both 3: {l1} vs {l2} (residual {})",
			report.residual
		);
	}

	#[test]
	fn standalone_circle_profile_lies_on_the_circle() {
		// A circle of radius 2 about (3,1): every tessellated boundary point must sit
		// exactly on it, and the profile must extrude to a closed manifold solid.
		let mut s = Sketch::new();
		let c = s.add_point(DVec2::new(3.0, 1.0));
		let rp = s.add_point(DVec2::new(5.0, 1.0)); // radius 2
		s.add_circle(c, rp);

		let prof = s.profile().expect("a standalone circle is a closed profile");
		let max_radius_error = prof
			.iter()
			.map(|p| ((*p - DVec2::new(3.0, 1.0)).length() - 2.0).abs())
			.fold(0.0_f64, f64::max);
		let solid = s.extrude(4.0).expect("circle extrudes");
		let v = validate::validate(&solid);
		let vol = validate::volume(&solid);
		// A standalone circle now extrudes to an EXACT analytic cylinder, so its volume tracks
		// the TRUE πr²h (within the default tessellation's facet error) — not the coarser 16-gon
		// profile area the prism used to be built from.
		let true_vol = std::f64::consts::PI * 2.0 * 2.0 * 4.0;
		assert!(
			prof.len() >= 8 && max_radius_error < 1e-9 && v.closed && v.manifold && (vol - true_vol).abs() / true_vol < 0.01,
			"circle: {} pts, radius err {max_radius_error}, vol {vol} vs true {true_vol}",
			prof.len()
		);
	}

	#[test]
	fn standalone_circle_extrudes_to_an_exact_analytic_cylinder() {
		// The exactness win: a sketched circle extrudes to an analytic Surface::Cylinder, so the
		// wall is the TRUE cylinder. Its adaptively-tessellated volume tracks πr²h to a fraction
		// of a percent — far tighter than the old faceted 16-gon prism (~2.6% under) — and it is
		// watertight. tessellate_adaptive_tol can refine the wall further toward micron for a
		// finer tolerance, exactly like the cylinder() primitive.
		let mut s = Sketch::new();
		let c = s.add_point(DVec2::new(0.0, 0.0));
		let rp = s.add_point(DVec2::new(3.0, 0.0)); // radius 3
		s.add_circle(c, rp);
		let solid = s.extrude(10.0).expect("circle extrudes to a cylinder");
		let mesh = kernel_brep::tessellate_adaptive_tol(&solid, 0.005);
		let true_vol = std::f64::consts::PI * 3.0 * 3.0 * 10.0;
		let has_cyl = solid.faces().any(|f| matches!(solid.face(f).surface, kernel_brep::geom::Surface::Cylinder { .. }));
		assert!(
			has_cyl && mesh.is_watertight() && (mesh.signed_volume() - true_vol).abs() / true_vol < 0.005,
			"sketched circle must be an exact analytic cylinder: has_cyl={has_cyl} wt={} vol={} vs true {true_vol}",
			mesh.is_watertight(),
			mesh.signed_volume()
		);
	}

	#[test]
	fn tangent_constraint_meets_the_circle_at_the_radius() {
		// A unit-2 circle at the origin; a line through the fixed point (0,5) with a
		// free far end must tilt until it is tangent (perp distance from O equals 2).
		let mut s = Sketch::new();
		let o = s.add_point(DVec2::ZERO);
		let rp = s.add_point(DVec2::new(2.0, 0.0)); // radius 2
		let la = s.add_point(DVec2::new(0.0, 5.0));
		let lb = s.add_point(DVec2::new(5.0, 5.0)); // free
		s.add_constraint(SketchConstraint::Fixed { point: o, at: DVec2::ZERO });
		s.add_constraint(SketchConstraint::Fixed { point: rp, at: DVec2::new(2.0, 0.0) });
		s.add_constraint(SketchConstraint::Fixed { point: la, at: DVec2::new(0.0, 5.0) });
		s.add_constraint(SketchConstraint::Tangent { line_a: la, line_b: lb, center: o, radius_point: rp });

		let report = s.solve();

		let dir = s.point(lb) - s.point(la);
		let perp = dir.perp_dot(s.point(o) - s.point(la)).abs() / dir.length();
		assert!(
			(perp - 2.0).abs() < 1e-6 && report.converged,
			"line should be tangent at radius 2, perpendicular distance {perp} (residual {})",
			report.residual
		);
	}

	#[test]
	fn open_chain_is_rejected_not_extruded() {
		// Four points joined a-b-c-d by three segments: an open chain whose ends have
		// degree one, so it is not a closed profile and must not extrude.
		let mut s = Sketch::new();
		let a = s.add_point(DVec2::new(0.0, 0.0));
		let b = s.add_point(DVec2::new(1.0, 0.0));
		let c = s.add_point(DVec2::new(1.0, 1.0));
		let d = s.add_point(DVec2::new(0.0, 1.0));
		s.add_segment(a, b);
		s.add_segment(b, c);
		s.add_segment(c, d);
		assert_eq!(s.extrude(1.0).err(), Some(SketchError::NotClosed));
	}

	#[test]
	fn empty_sketch_solve_does_not_panic() {
		let mut s = Sketch::new();
		let r = s.solve();
		assert!(r.converged && r.residual == 0.0);
	}
}
