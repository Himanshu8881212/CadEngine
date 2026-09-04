// Copyright (c) LMCAD. Licensed under the MIT License.

//! Assembly constraint solver.
//!
//! This module turns a set of placed [`Instance`](crate::Instance)-style
//! transforms plus a list of geometric **mates** ([`Constraint`]) into a small
//! iterative solver. It does *not* depend on the rest of `kernel-model`; it
//! operates purely on a `Vec` of poses (one per instance) and a `Vec` of
//! constraints that reference instances by index. The caller is free to read the
//! solved poses back out and apply them to its own [`Instance`]s.
//!
//! # Conventions
//!
//! - Poses are stored as [`Affine3A`] (the same f32 type the assembly layer uses
//!   for [`Instance::pose`](crate::Instance::pose)).
//! - All solving happens internally in **f64** ([`DVec3`] / [`DQuat`] /
//!   [`DAffine3`]) for precision, matching the B-rep half of the kernel, then is
//!   written back to the f32 poses.
//! - **Instance `0` is ground**: it is never moved, so the assembly has a fixed
//!   reference frame. Every other instance is free to translate and rotate.
//!
//! # Algorithm
//!
//! A projective / Gauss-Seidel relaxation. Each [`ConstraintSystem::solve`]
//! iteration visits the constraints in order and nudges the *free* instance(s) of
//! each constraint to reduce that constraint's error:
//!
//! - **Translational** mates ([`Constraint::Coincident`], [`Constraint::Distance`])
//!   move the offending point(s) by translating their owning instance.
//! - **Rotational** mates ([`Constraint::Parallel`], [`Constraint::Concentric`],
//!   [`Constraint::Angle`]) rotate the owning instance about its current world
//!   position so the relevant direction aligns. [`Constraint::Concentric`]
//!   additionally translates so the axes become collinear, not just parallel;
//!   [`Constraint::AxisDistance`] drives the same perpendicular gap to a target
//!   center distance instead of zero (the gear/parallel-shaft mate).
//! - [`Constraint::Fixed`] is structural: it grounds its instance (like index 0)
//!   and contributes no error term.
//!
//! Honesty companions: [`ConstraintSystem::validate`] names statically-broken
//! mates the sweep would silently skip, [`ConstraintSystem::per_constraint_residuals`]
//! says WHICH mate is unsatisfied, and [`ConstraintSystem::analyze`] reports the
//! remaining rigid-body DOF (numeric Jacobian rank), so an under-constrained
//! assembly can no longer look identical to a fully-mated one.
//!
//! Because each constraint is satisfied locally and the next constraint may disturb
//! it, the system is iterated with a Jacobi sweep: each sweep averages every
//! constraint's correction per instance (under-relaxed by [`RELAX`]). Competing
//! **translational** mates thus reach the exact least-squares minimum, independent
//! of order; competing **rotational** mates reach a local optimum that can depend
//! on the starting pose (see [`RELAX`]). [`ConstraintSystem::solve`] returns the
//! best residual found and writes back the corresponding poses. No external solver
//! crate is used.

use kernel_core::math::{Affine3A, DAffine3, DQuat, DVec3, Quat, Vec3};
use serde::{Deserialize, Serialize};

/// Numerical floor below which a direction / displacement is treated as zero.
const EPS: f64 = 1e-12;

/// Under-relaxation factor for the Jacobi sweep. Each sweep accumulates every
/// constraint's desired correction per instance and applies their **average**
/// (scaled by `RELAX`). Because the average — not the last constraint — drives each
/// DOF, **translational** mates converge to the exact linear least-squares minimum,
/// independent of constraint order. **Rotational** mates use a small-angle
/// (axis·angle) linearization on SO(3): a single mate is solved exactly, but several
/// competing mates with non-coplanar axes can settle at a *local* optimum that
/// depends on the starting pose (the rotational problem is non-convex). `RELAX`
/// trades convergence speed for stability of the rotational step.
const RELAX: f64 = 0.8;

/// A geometric mate between two instances, referenced by their index in the
/// [`ConstraintSystem`]'s transform list.
///
/// All geometry (`*_point`, `*_dir`, `*_axis_*`) is expressed in the **local**
/// frame of the referenced instance; the solver transforms it to world space
/// through that instance's current pose. Directions and axes need not be
/// normalized — the solver normalizes defensively.
///
/// Serializable (it is the `mates` entry of the `.lmcasm` assembly format — see
/// [`crate::format`]): plain data, externally tagged, points/directions as
/// `[x, y, z]` arrays.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum Constraint {
	/// Make a local point on `a` coincide with a local point on `b` in world space.
	Coincident {
		/// Index of the first instance.
		a: usize,
		/// Point on `a`, in `a`'s local frame.
		a_point: DVec3,
		/// Index of the second instance.
		b: usize,
		/// Point on `b`, in `b`'s local frame.
		b_point: DVec3,
	},
	/// Hold the world points a fixed `distance` apart.
	Distance {
		/// Index of the first instance.
		a: usize,
		/// Point on `a`, in `a`'s local frame.
		a_point: DVec3,
		/// Index of the second instance.
		b: usize,
		/// Point on `b`, in `b`'s local frame.
		b_point: DVec3,
		/// Target separation between the two world points (clamped to `>= 0`).
		distance: f64,
	},
	/// Align a local direction on `a` with a local direction on `b` (parallel).
	///
	/// The mate is satisfied when the two world directions are parallel *or*
	/// anti-parallel (their cross product vanishes); the solver rotates toward
	/// whichever is closer so it never forces a 180° flip.
	Parallel {
		/// Index of the first instance.
		a: usize,
		/// Direction on `a`, in `a`'s local frame.
		a_dir: DVec3,
		/// Index of the second instance.
		b: usize,
		/// Direction on `b`, in `b`'s local frame.
		b_dir: DVec3,
	},
	/// Make two axes collinear: the axis directions become parallel **and** the
	/// axis lines coincide (zero perpendicular offset).
	Concentric {
		/// Index of the first instance.
		a: usize,
		/// A point on `a`'s axis, in `a`'s local frame.
		a_axis_point: DVec3,
		/// Direction of `a`'s axis, in `a`'s local frame.
		a_axis_dir: DVec3,
		/// Index of the second instance.
		b: usize,
		/// A point on `b`'s axis, in `b`'s local frame.
		b_axis_point: DVec3,
		/// Direction of `b`'s axis, in `b`'s local frame.
		b_axis_dir: DVec3,
	},
	/// Hold the world angle between two local directions at a target (degrees).
	///
	/// Unlike [`Constraint::Parallel`], the angle is DIRECTIONAL: the error is
	/// `acos(da·db) − degrees` with the world angle in `[0°, 180°]`, so `0°`
	/// means same direction and `180°` means opposed — an anti-parallel pair is
	/// NOT a satisfied `0°` mate. The error term is in radians² (the same
	/// dimensionless convention as `Parallel`'s cross product).
	Angle {
		/// Index of the first instance.
		a: usize,
		/// Direction on `a`, in `a`'s local frame.
		a_dir: DVec3,
		/// Index of the second instance.
		b: usize,
		/// Direction on `b`, in `b`'s local frame.
		b_dir: DVec3,
		/// Target angle in degrees, clamped to `[0, 180]` by the solver.
		degrees: f64,
	},
	/// Keep two axes PARALLEL with their lines a fixed perpendicular `distance`
	/// apart — the parallel-shaft / gear-mesh center-distance mate.
	/// [`Constraint::Concentric`] is exactly the `distance = 0` special case.
	AxisDistance {
		/// Index of the first instance.
		a: usize,
		/// A point on `a`'s axis, in `a`'s local frame.
		a_axis_point: DVec3,
		/// Direction of `a`'s axis, in `a`'s local frame.
		a_axis_dir: DVec3,
		/// Index of the second instance.
		b: usize,
		/// A point on `b`'s axis, in `b`'s local frame.
		b_axis_point: DVec3,
		/// Direction of `b`'s axis, in `b`'s local frame.
		b_axis_dir: DVec3,
		/// Target perpendicular distance between the axis lines (clamped ≥ 0).
		distance: f64,
	},
	/// Ground `instance`: freeze its pose exactly like the implicit instance-`0`
	/// ground. Structural — contributes no error term — so ANY instance, not
	/// just index 0, can anchor the assembly (or several can, for a fixture
	/// plate plus a clamped part).
	Fixed {
		/// Index of the instance to freeze.
		instance: usize,
	},
}

impl Constraint {
	/// The two instance indices this constraint couples ([`Constraint::Fixed`]
	/// references one instance and returns it twice).
	fn instances(&self) -> (usize, usize) {
		match *self {
			Constraint::Coincident { a, b, .. }
			| Constraint::Distance { a, b, .. }
			| Constraint::Parallel { a, b, .. }
			| Constraint::Concentric { a, b, .. }
			| Constraint::Angle { a, b, .. }
			| Constraint::AxisDistance { a, b, .. } => (a, b),
			Constraint::Fixed { instance } => (instance, instance),
		}
	}

	/// The mate kind as the receipt vocabulary (`"coincident"`, …) — stable
	/// snake_case names used by the API layers' per-mate reports.
	pub fn kind_name(&self) -> &'static str {
		match self {
			Constraint::Coincident { .. } => "coincident",
			Constraint::Distance { .. } => "distance",
			Constraint::Parallel { .. } => "parallel",
			Constraint::Concentric { .. } => "concentric",
			Constraint::Angle { .. } => "angle",
			Constraint::AxisDistance { .. } => "axis_distance",
			Constraint::Fixed { .. } => "fixed",
		}
	}
}

/// An assembly of instance poses plus the mates that relate them.
///
/// Build one with [`ConstraintSystem::new`] (or the `transforms` / `constraints`
/// fields directly), call [`ConstraintSystem::solve`], then read the relaxed
/// poses back with [`ConstraintSystem::transforms`].
pub struct ConstraintSystem {
	/// One world pose per instance. Index `0` is treated as ground.
	transforms: Vec<Affine3A>,
	/// The mates to satisfy.
	constraints: Vec<Constraint>,
}

impl ConstraintSystem {
	/// Create a system from initial poses and constraints.
	pub fn new(transforms: Vec<Affine3A>, constraints: Vec<Constraint>) -> Self {
		Self { transforms, constraints }
	}

	/// Add a constraint to the system.
	pub fn add_constraint(&mut self, constraint: Constraint) {
		self.constraints.push(constraint);
	}

	/// Add a **face-to-face mate** from two faces' planes (each as a local
	/// `point` + outward `normal`, e.g. from
	/// [`kernel_brep::Solid::face_plane`](../../kernel_brep/struct.Solid.html#method.face_plane)).
	///
	/// This couples the two instances so the faces lie flat against one another: a
	/// [`Constraint::Coincident`] brings the face points together and a
	/// [`Constraint::Parallel`] aligns their normals (anti-parallel counts as
	/// parallel here, which is the natural mating sense). It lets a mate be built
	/// from an instance's *actual B-rep geometry* rather than a hand-computed frame.
	pub fn add_face_mate(&mut self, a: usize, a_point: DVec3, a_normal: DVec3, b: usize, b_point: DVec3, b_normal: DVec3) {
		self.add_constraint(Constraint::Coincident { a, a_point, b, b_point });
		self.add_constraint(Constraint::Parallel { a, a_dir: a_normal, b, b_dir: b_normal });
	}

	/// Add an **axis-alignment (concentric) mate** from two faces' axes (each a
	/// local `point` + `dir`, e.g. from
	/// [`kernel_brep::Solid::face_axis`](../../kernel_brep/struct.Solid.html#method.face_axis)).
	/// Makes the two axes collinear — the natural mate for a shaft in a hole.
	pub fn add_axis_mate(&mut self, a: usize, a_point: DVec3, a_dir: DVec3, b: usize, b_point: DVec3, b_dir: DVec3) {
		self.add_constraint(Constraint::Concentric {
			a,
			a_axis_point: a_point,
			a_axis_dir: a_dir,
			b,
			b_axis_point: b_point,
			b_axis_dir: b_dir,
		});
	}

	/// The current (possibly solved) instance poses.
	pub fn transforms(&self) -> &[Affine3A] {
		&self.transforms
	}

	/// Iteratively relax the instance poses to minimize total constraint error.
	///
	/// Runs at most `iterations` Gauss-Seidel sweeps over the constraint list,
	/// holding instance `0` fixed as ground. Returns the final residual error
	/// ([`ConstraintSystem::residual`]) after solving. Stops early once the
	/// residual stops improving, so passing a generous `iterations` budget is
	/// cheap when the system is already satisfied.
	pub fn solve(&mut self, iterations: usize) -> f64 {
		// Solve in f64 for precision, write back to the f32 poses at the end.
		let mut poses: Vec<DAffine3> = self.transforms.iter().map(to_daffine3).collect();

		let n = poses.len();
		let grounded = self.grounded();
		let mut prev = self.residual_for(&poses);
		let mut best = prev;
		let mut best_poses = poses.clone();
		// Per-sweep Jacobi accumulators: the summed desired translation and rotation
		// (as an axis·angle vector) per instance, and how many constraints touched it.
		let (mut acc_t, mut acc_r, mut cnt) = (vec![DVec3::ZERO; n], vec![DVec3::ZERO; n], vec![0u32; n]);
		for _ in 0..iterations {
			acc_t.iter_mut().for_each(|x| *x = DVec3::ZERO);
			acc_r.iter_mut().for_each(|x| *x = DVec3::ZERO);
			cnt.iter_mut().for_each(|x| *x = 0);
			for constraint in &self.constraints {
				apply_constraint(&poses, constraint, &grounded, &mut acc_t, &mut acc_r, &mut cnt);
			}
			// Apply the AVERAGE correction per free instance (grounded instances —
			// index 0 plus any `Fixed` mates — never move), under-relaxed —
			// rotation about the instance's own origin, then translation.
			for i in 1..n {
				if cnt[i] == 0 || grounded[i] {
					continue;
				}
				let w = RELAX / cnt[i] as f64;
				let rv = acc_r[i] * w;
				let ang = rv.length();
				if ang > EPS {
					rotate_about_origin(&mut poses[i], DQuat::from_axis_angle(rv / ang, ang));
				}
				translate(&mut poses[i], acc_t[i] * w);
			}
			let now = self.residual_for(&poses);
			// Keep the best state seen, so an overshoot can never write back a worse one.
			if now < best {
				best = now;
				best_poses.copy_from_slice(&poses);
			}
			// Converged (or stalled): no point burning the rest of the budget.
			if (prev - now).abs() <= EPS * (1.0 + prev) {
				break;
			}
			prev = now;
		}

		for (dst, src) in self.transforms.iter_mut().zip(best_poses.iter()) {
			*dst = to_affine3a(src);
		}
		best
	}

	/// Total constraint error of the current poses: the sum of per-constraint
	/// squared residuals. Zero means every mate is satisfied.
	pub fn residual(&self) -> f64 {
		let poses: Vec<DAffine3> = self.transforms.iter().map(to_daffine3).collect();
		self.residual_for(&poses)
	}

	/// Total squared residual for an arbitrary set of f64 poses.
	fn residual_for(&self, poses: &[DAffine3]) -> f64 {
		self.constraints.iter().map(|c| constraint_error(poses, c)).sum()
	}

	/// Per-instance grounded flags: index 0 plus every [`Constraint::Fixed`]
	/// target. Grounded instances never move and contribute no free DOF.
	pub fn grounded(&self) -> Vec<bool> {
		let n = self.transforms.len();
		let mut grounded = vec![false; n];
		if n > 0 {
			grounded[0] = true;
		}
		for c in &self.constraints {
			if let Constraint::Fixed { instance } = c {
				if *instance < n {
					grounded[*instance] = true;
				}
			}
		}
		grounded
	}

	/// The squared residual of EACH constraint under the current poses, parallel
	/// to the constraint list — the receipt that names WHICH mate is
	/// unsatisfied instead of one anonymous total.
	pub fn per_constraint_residuals(&self) -> Vec<f64> {
		let poses: Vec<DAffine3> = self.transforms.iter().map(to_daffine3).collect();
		self.constraints.iter().map(|c| constraint_error(&poses, c)).collect()
	}

	/// Static mate diagnostics — the problems the solver would otherwise ABSORB
	/// SILENTLY (an out-of-range index or self-mate contributes zero error and
	/// is skipped; a zero direction can never act). Returns one message per
	/// problem, each naming the mate index and kind; empty means clean. API
	/// layers refuse to solve on a non-empty result (the library `solve` stays
	/// tolerant for backward compatibility).
	pub fn validate(&self) -> Vec<String> {
		let n = self.transforms.len();
		let grounded = self.grounded();
		let mut problems = Vec::new();
		let zeroish = |v: DVec3| v.length_squared() <= EPS;
		for (k, c) in self.constraints.iter().enumerate() {
			let kind = c.kind_name();
			let (ia, ib) = c.instances();
			if ia >= n || ib >= n {
				problems.push(format!(
					"mate {k} ({kind}): instance index out of range ({} with only {n} instances) — the solver would silently skip it",
					ia.max(ib)
				));
				continue;
			}
			if !matches!(c, Constraint::Fixed { .. }) {
				if ia == ib {
					problems
						.push(format!("mate {k} ({kind}): references instance {ia} on both sides — a self-mate can never move anything"));
				} else if grounded[ia] && grounded[ib] {
					problems.push(format!(
						"mate {k} ({kind}): both instances ({ia}, {ib}) are grounded — the solver cannot act on it; it is only ever satisfied if the fixed poses already satisfy it"
					));
				}
			}
			match *c {
				Constraint::Parallel { a_dir, b_dir, .. } | Constraint::Angle { a_dir, b_dir, .. } if zeroish(a_dir) || zeroish(b_dir) => {
					problems.push(format!("mate {k} ({kind}): zero-length direction — it can never align anything"));
				}
				Constraint::Concentric { a_axis_dir, b_axis_dir, .. } | Constraint::AxisDistance { a_axis_dir, b_axis_dir, .. }
					if zeroish(a_axis_dir) || zeroish(b_axis_dir) =>
				{
					problems.push(format!("mate {k} ({kind}): zero-length axis direction — it can never align anything"));
				}
				_ => {}
			}
			match *c {
				Constraint::Distance { distance, .. } | Constraint::AxisDistance { distance, .. } if distance < 0.0 => {
					problems.push(format!(
						"mate {k} ({kind}): negative distance {distance} — the solver clamps to 0; state the intended separation explicitly"
					));
				}
				Constraint::Angle { degrees, .. } if !(0.0..=180.0).contains(&degrees) => {
					problems.push(format!(
						"mate {k} ({kind}): target {degrees}° outside [0, 180] — the world angle between two directions always lies in that band"
					));
				}
				_ => {}
			}
		}
		problems
	}

	/// Numeric degrees-of-freedom analysis of the CURRENT configuration — the
	/// assembly-level counterpart of the sketch solver's DOF verdict.
	///
	/// Builds the constraint Jacobian by central differences over the free
	/// instances' 6 pose DOF each (constraint rows use their MINIMAL dimension:
	/// coincident 3, distance 1, parallel 2, concentric 4, angle 1,
	/// axis_distance 3; `fixed` contributes grounding, not rows) and ranks it,
	/// so `free_dof = 6·free_instances − rank` is the number of unconstrained
	/// rigid-body motions REMAINING (an unmated spin or slide shows up here
	/// instead of staying silent). `redundant_rows = rows − rank` counts
	/// over-constraint: harmless when the residual is ~0 (consistent
	/// redundancy), conflicting when it is not. The rank is taken at the
	/// current poses — a singular special configuration can differ from the
	/// generic-pose rank.
	pub fn analyze(&self) -> DofReport {
		let poses: Vec<DAffine3> = self.transforms.iter().map(to_daffine3).collect();
		let n = poses.len();
		let grounded = self.grounded();
		let free: Vec<usize> = (0..n).filter(|&i| !grounded[i]).collect();
		let cols = free.len() * 6;

		// One row-evaluator per constraint, with any projection basis FROZEN at
		// the base configuration (a Jacobian needs a fixed function).
		let evals: Vec<RowEval> = self.constraints.iter().filter_map(|c| row_eval(&poses, c, n)).collect();
		let rows: usize = evals.iter().map(|e| e.dim).sum();

		let mut jac = vec![vec![0.0f64; cols]; rows];
		const H_T: f64 = 1e-5; // mm
		const H_R: f64 = 1e-5; // rad
		for (col, (&inst, k)) in free.iter().flat_map(|i| (0..6).map(move |k| (i, k))).enumerate() {
			let mut plus = poses.clone();
			let mut minus = poses.clone();
			let axis = DVec3::new(f64::from(u8::from(k % 3 == 0)), f64::from(u8::from(k % 3 == 1)), f64::from(u8::from(k % 3 == 2)));
			if k < 3 {
				translate(&mut plus[inst], axis * H_T);
				translate(&mut minus[inst], axis * -H_T);
			} else {
				rotate_about_origin(&mut plus[inst], DQuat::from_axis_angle(axis, H_R));
				rotate_about_origin(&mut minus[inst], DQuat::from_axis_angle(axis, -H_R));
			}
			let h = if k < 3 { H_T } else { H_R };
			let mut row0 = 0;
			for e in &evals {
				let fp = (e.f)(&plus);
				let fm = (e.f)(&minus);
				for (r, (vp, vm)) in fp.iter().zip(fm.iter()).enumerate() {
					jac[row0 + r][col] = (vp - vm) / (2.0 * h);
				}
				row0 += e.dim;
			}
		}
		let rank = matrix_rank(&mut jac);
		let available = cols;
		let free_dof = available.saturating_sub(rank);
		DofReport {
			instances: n,
			grounded_instances: grounded.iter().filter(|&&g| g).count(),
			free_dof_available: available,
			constraint_rows: rows,
			rank,
			free_dof,
			redundant_rows: rows.saturating_sub(rank),
			verdict: if free_dof > 0 { format!("under_constrained ({free_dof} free DOF)") } else { "well_constrained".to_string() },
		}
	}
}

/// The result of [`ConstraintSystem::analyze`]: a numeric DOF accounting of the
/// mated assembly at its current poses.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct DofReport {
	/// Total instances in the system (including grounded ones).
	pub instances: usize,
	/// Instances that can never move (index 0 + every `Fixed` mate target).
	pub grounded_instances: usize,
	/// Rigid-body DOF before mates: `6 × (instances − grounded)`.
	pub free_dof_available: usize,
	/// Total constraint rows contributed by the mates (minimal dimensions).
	pub constraint_rows: usize,
	/// Numeric rank of the constraint Jacobian at the current poses.
	pub rank: usize,
	/// Unconstrained rigid-body motions remaining (`available − rank`).
	pub free_dof: usize,
	/// `rows − rank`: over-constraint count (consistent if the residual is ~0,
	/// conflicting otherwise).
	pub redundant_rows: usize,
	/// `"well_constrained"` or `"under_constrained (N free DOF)"`.
	pub verdict: String,
}

/// The boxed residual-component function of a [`RowEval`].
type RowFn = Box<dyn Fn(&[DAffine3]) -> Vec<f64>>;

/// A frozen row-evaluator: `dim` residual components as a function of poses.
struct RowEval {
	dim: usize,
	f: RowFn,
}

/// Build the minimal-dimension residual rows for one constraint, freezing any
/// projection basis at the `base` configuration. `None` for structural or
/// out-of-range constraints (they contribute no rows).
fn row_eval(base: &[DAffine3], c: &Constraint, n: usize) -> Option<RowEval> {
	let (ia, ib) = c.instances();
	if ia >= n || ib >= n {
		return None;
	}
	match *c {
		Constraint::Fixed { .. } => None,
		Constraint::Coincident { a, a_point, b, b_point } => Some(RowEval {
			dim: 3,
			f: Box::new(move |p| {
				let d = p[a].transform_point3(a_point) - p[b].transform_point3(b_point);
				vec![d.x, d.y, d.z]
			}),
		}),
		Constraint::Distance { a, a_point, b, b_point, distance } => Some(RowEval {
			dim: 1,
			f: Box::new(move |p| vec![(p[a].transform_point3(a_point) - p[b].transform_point3(b_point)).length() - distance.max(0.0)]),
		}),
		Constraint::Parallel { a, a_dir, b, b_dir } => {
			let da0 = base[a].transform_vector3(a_dir).normalize_or_zero();
			let (e1, e2) = perp_basis(da0);
			Some(RowEval {
				dim: 2,
				f: Box::new(move |p| {
					let cx = p[a].transform_vector3(a_dir).normalize_or_zero().cross(p[b].transform_vector3(b_dir).normalize_or_zero());
					vec![cx.dot(e1), cx.dot(e2)]
				}),
			})
		}
		Constraint::Concentric { a, a_axis_point, a_axis_dir, b, b_axis_point, b_axis_dir } => {
			let da0 = base[a].transform_vector3(a_axis_dir).normalize_or_zero();
			let (e1, e2) = perp_basis(da0);
			Some(RowEval {
				dim: 4,
				f: Box::new(move |p| {
					let da = p[a].transform_vector3(a_axis_dir).normalize_or_zero();
					let db = p[b].transform_vector3(b_axis_dir).normalize_or_zero();
					let cx = da.cross(db);
					let rel = p[b].transform_point3(b_axis_point) - p[a].transform_point3(a_axis_point);
					vec![cx.dot(e1), cx.dot(e2), rel.dot(e1), rel.dot(e2)]
				}),
			})
		}
		Constraint::Angle { a, a_dir, b, b_dir, degrees } => Some(RowEval {
			dim: 1,
			f: Box::new(move |p| {
				let da = p[a].transform_vector3(a_dir).normalize_or_zero();
				let db = p[b].transform_vector3(b_dir).normalize_or_zero();
				vec![da.dot(db).clamp(-1.0, 1.0).acos() - degrees.clamp(0.0, 180.0).to_radians()]
			}),
		}),
		Constraint::AxisDistance { a, a_axis_point, a_axis_dir, b, b_axis_point, b_axis_dir, distance } => {
			let da0 = base[a].transform_vector3(a_axis_dir).normalize_or_zero();
			let (e1, e2) = perp_basis(da0);
			Some(RowEval {
				dim: 3,
				f: Box::new(move |p| {
					let da = p[a].transform_vector3(a_axis_dir).normalize_or_zero();
					let db = p[b].transform_vector3(b_axis_dir).normalize_or_zero();
					let cx = da.cross(db);
					let pa_w = p[a].transform_point3(a_axis_point);
					let pb_w = p[b].transform_point3(b_axis_point);
					vec![cx.dot(e1), cx.dot(e2), perpendicular_offset(pb_w, pa_w, da).length() - distance.max(0.0)]
				}),
			})
		}
	}
}

/// Two orthonormal vectors spanning the plane perpendicular to (unit-ish) `v`.
fn perp_basis(v: DVec3) -> (DVec3, DVec3) {
	let e1 = orthonormal(if v.length_squared() > EPS { v } else { DVec3::Z });
	let e2 = v.normalize_or_zero().cross(e1).normalize_or_zero();
	let e2 = if e2.length_squared() > EPS { e2 } else { orthonormal(e1) };
	(e1, e2)
}

/// Numeric rank of `m` (destructive Gaussian elimination with partial pivoting;
/// relative tolerance on the largest entry).
fn matrix_rank(m: &mut [Vec<f64>]) -> usize {
	let rows = m.len();
	if rows == 0 {
		return 0;
	}
	let cols = m[0].len();
	let scale = m.iter().flat_map(|r| r.iter()).fold(0.0f64, |acc, v| acc.max(v.abs()));
	let tol = (scale * 1e-9).max(1e-12);
	let mut rank = 0;
	for col in 0..cols {
		// Find the pivot row for this column among the unreduced rows.
		let Some(pivot) = (rank..rows).max_by(|&i, &j| m[i][col].abs().partial_cmp(&m[j][col].abs()).unwrap()) else {
			break;
		};
		if m[pivot][col].abs() <= tol {
			continue;
		}
		m.swap(rank, pivot);
		let (top, bottom) = m.split_at_mut(rank + 1);
		let pivot_row = &top[rank];
		for row in bottom.iter_mut() {
			let f = row[col] / pivot_row[col];
			for (dst, src) in row[col..].iter_mut().zip(&pivot_row[col..]) {
				*dst -= f * src;
			}
		}
		rank += 1;
		if rank == rows {
			break;
		}
	}
	rank
}

/// Squared error of a single constraint under the given poses.
fn constraint_error(poses: &[DAffine3], c: &Constraint) -> f64 {
	let (ia, ib) = c.instances();
	// Out-of-range indices contribute no error rather than panicking.
	if ia >= poses.len() || ib >= poses.len() {
		return 0.0;
	}
	let pa = &poses[ia];
	let pb = &poses[ib];
	match *c {
		Constraint::Coincident { a_point, b_point, .. } => {
			let wa = pa.transform_point3(a_point);
			let wb = pb.transform_point3(b_point);
			(wa - wb).length_squared()
		}
		Constraint::Distance { a_point, b_point, distance, .. } => {
			let wa = pa.transform_point3(a_point);
			let wb = pb.transform_point3(b_point);
			let d = (wa - wb).length() - distance.max(0.0);
			d * d
		}
		Constraint::Parallel { a_dir, b_dir, .. } => {
			let da = pa.transform_vector3(a_dir).normalize_or_zero();
			let db = pb.transform_vector3(b_dir).normalize_or_zero();
			// |a x b|^2 is zero iff parallel or anti-parallel.
			da.cross(db).length_squared()
		}
		Constraint::Concentric { a_axis_point, a_axis_dir, b_axis_point, b_axis_dir, .. } => {
			let da = pa.transform_vector3(a_axis_dir).normalize_or_zero();
			let db = pb.transform_vector3(b_axis_dir).normalize_or_zero();
			let pa_w = pa.transform_point3(a_axis_point);
			let pb_w = pb.transform_point3(b_axis_point);
			// Direction misalignment + perpendicular offset of one axis point
			// from the other axis line.
			let angular = da.cross(db).length_squared();
			let offset = perpendicular_offset(pb_w, pa_w, da);
			angular + offset.length_squared()
		}
		Constraint::Angle { a_dir, b_dir, degrees, .. } => {
			let da = pa.transform_vector3(a_dir).normalize_or_zero();
			let db = pb.transform_vector3(b_dir).normalize_or_zero();
			if da.length_squared() <= EPS || db.length_squared() <= EPS {
				return 0.0; // degenerate direction — flagged by `validate`, not scored
			}
			let err = da.dot(db).clamp(-1.0, 1.0).acos() - degrees.clamp(0.0, 180.0).to_radians();
			err * err
		}
		Constraint::AxisDistance { a_axis_point, a_axis_dir, b_axis_point, b_axis_dir, distance, .. } => {
			let da = pa.transform_vector3(a_axis_dir).normalize_or_zero();
			let db = pb.transform_vector3(b_axis_dir).normalize_or_zero();
			let pa_w = pa.transform_point3(a_axis_point);
			let pb_w = pb.transform_point3(b_axis_point);
			let angular = da.cross(db).length_squared();
			let gap = perpendicular_offset(pb_w, pa_w, da).length() - distance.max(0.0);
			angular + gap * gap
		}
		Constraint::Fixed { .. } => 0.0, // structural: grounds the instance, no error term
	}
}

/// Accumulate `c`'s desired correction into the per-instance Jacobi buffers
/// (translation, rotation-as-axis·angle, and a touch count) — reading, but not
/// mutating, the current `poses`.
fn apply_constraint(poses: &[DAffine3], c: &Constraint, grounded: &[bool], acc_t: &mut [DVec3], acc_r: &mut [DVec3], cnt: &mut [u32]) {
	let (ia, ib) = c.instances();
	if ia >= poses.len() || ib >= poses.len() {
		return;
	}
	// Which endpoints may move? Grounded instances (index 0 + `Fixed` targets)
	// never do. A self-referential or fully-grounded constraint cannot be
	// satisfied, so we skip it (surfaced loudly by `validate`).
	let a_free = !grounded[ia];
	let b_free = !grounded[ib];
	if (!a_free && !b_free) || ia == ib {
		return;
	}

	match *c {
		Constraint::Coincident { a_point, b_point, .. } => {
			let wa = poses[ia].transform_point3(a_point);
			let wb = poses[ib].transform_point3(b_point);
			let delta = wb - wa; // move A's point toward B's point
			distribute_translation(acc_t, cnt, ia, a_free, ib, b_free, delta);
		}
		Constraint::Distance { a_point, b_point, distance, .. } => {
			let wa = poses[ia].transform_point3(a_point);
			let wb = poses[ib].transform_point3(b_point);
			let sep = wb - wa;
			let len = sep.length();
			let target = distance.max(0.0);
			let dir = if len > EPS {
				sep / len
			} else {
				// Degenerate: points coincide but a non-zero gap is wanted.
				// Pick an arbitrary, deterministic separation axis.
				DVec3::X
			};
			// Positive => points too close (push apart along -dir for A / +dir for B).
			let correction = (target - len) * dir;
			// Pull A backward by `correction` so |A'-B| -> target.
			distribute_translation(acc_t, cnt, ia, a_free, ib, b_free, -correction);
		}
		Constraint::Parallel { a_dir, b_dir, .. } => {
			let da = poses[ia].transform_vector3(a_dir).normalize_or_zero();
			let db = poses[ib].transform_vector3(b_dir).normalize_or_zero();
			distribute_alignment(acc_r, cnt, ia, a_free, ib, b_free, da, db);
		}
		Constraint::Concentric { a_axis_point, a_axis_dir, b_axis_point, b_axis_dir, .. } => {
			// 1) Align the axis directions (rotational part).
			let da = poses[ia].transform_vector3(a_axis_dir).normalize_or_zero();
			let db = poses[ib].transform_vector3(b_axis_dir).normalize_or_zero();
			distribute_alignment(acc_r, cnt, ia, a_free, ib, b_free, da, db);

			// 2) Remove the perpendicular offset between the two axis lines
			//    (translational part).
			let pa_w = poses[ia].transform_point3(a_axis_point);
			let pb_w = poses[ib].transform_point3(b_axis_point);
			// Perpendicular component of (pb - pa) relative to A's axis. To drop B
			// onto A's line, B must move by `-offset`; `distribute_translation`
			// applies `-delta` to B, so pass `delta = offset` (which also moves a
			// free A by `+offset` toward B, meeting in the middle).
			let offset = perpendicular_offset(pb_w, pa_w, da);
			distribute_translation(acc_t, cnt, ia, a_free, ib, b_free, offset);
		}
		Constraint::Angle { a_dir, b_dir, degrees, .. } => {
			let da = poses[ia].transform_vector3(a_dir).normalize_or_zero();
			let db = poses[ib].transform_vector3(b_dir).normalize_or_zero();
			if da.length_squared() <= EPS || db.length_squared() <= EPS {
				return;
			}
			let target = degrees.clamp(0.0, 180.0).to_radians();
			let theta = da.dot(db).clamp(-1.0, 1.0).acos();
			let err = theta - target;
			if err.abs() <= EPS {
				return;
			}
			// Rotating A's body by +err about (da × db) moves da toward db by err,
			// closing theta onto the target; at the parallel/anti-parallel
			// singularity any perpendicular axis serves.
			let cx = da.cross(db);
			let axis = if cx.length_squared() > EPS { cx.normalize() } else { orthonormal(da) };
			match (a_free, b_free) {
				(true, false) => {
					acc_r[ia] += axis * err;
					cnt[ia] += 1;
				}
				(false, true) => {
					acc_r[ib] -= axis * err;
					cnt[ib] += 1;
				}
				(true, true) => {
					acc_r[ia] += axis * (err * 0.5);
					cnt[ia] += 1;
					acc_r[ib] -= axis * (err * 0.5);
					cnt[ib] += 1;
				}
				(false, false) => {}
			}
		}
		Constraint::AxisDistance { a_axis_point, a_axis_dir, b_axis_point, b_axis_dir, distance, .. } => {
			// 1) Align the axis directions (rotational part — same as Concentric).
			let da = poses[ia].transform_vector3(a_axis_dir).normalize_or_zero();
			let db = poses[ib].transform_vector3(b_axis_dir).normalize_or_zero();
			distribute_alignment(acc_r, cnt, ia, a_free, ib, b_free, da, db);

			// 2) Drive the perpendicular gap between the axis lines to `distance`
			//    (Concentric drives it to 0). `distribute_translation` applies
			//    `-delta` to B, so `delta = ô·(len − target)` shrinks/grows the gap.
			let pa_w = poses[ia].transform_point3(a_axis_point);
			let pb_w = poses[ib].transform_point3(b_axis_point);
			let o = perpendicular_offset(pb_w, pa_w, da);
			let len = o.length();
			let target = distance.max(0.0);
			// Coincident axes wanting a gap: push along a deterministic perpendicular.
			let dir = if len > EPS { o / len } else { orthonormal(da) };
			distribute_translation(acc_t, cnt, ia, a_free, ib, b_free, dir * (len - target));
		}
		Constraint::Fixed { .. } => {} // structural: handled via the grounded set
	}
}

/// Accumulate the desired world displacement of A's point (`delta = target −
/// current`) into the Jacobi buffers. If only one instance is free it takes the
/// whole move; if both are free they split it so neither dominates.
fn distribute_translation(acc_t: &mut [DVec3], cnt: &mut [u32], ia: usize, a_free: bool, ib: usize, b_free: bool, delta: DVec3) {
	if delta.length_squared() <= EPS {
		return;
	}
	match (a_free, b_free) {
		(true, false) => {
			acc_t[ia] += delta;
			cnt[ia] += 1;
		}
		(false, true) => {
			acc_t[ib] -= delta;
			cnt[ib] += 1;
		}
		(true, true) => {
			acc_t[ia] += delta * 0.5;
			cnt[ia] += 1;
			acc_t[ib] -= delta * 0.5;
			cnt[ib] += 1;
		}
		(false, false) => {}
	}
}

/// Accumulate the rotation (as an axis·angle vector) that aligns direction `da`
/// (on A) with `db` (on B) into the Jacobi buffers, split between the free instances.
#[allow(clippy::too_many_arguments)] // accumulator pair + the two instances and their dirs
fn distribute_alignment(acc_r: &mut [DVec3], cnt: &mut [u32], ia: usize, a_free: bool, ib: usize, b_free: bool, da: DVec3, db: DVec3) {
	if da.length_squared() <= EPS || db.length_squared() <= EPS {
		return;
	}
	match (a_free, b_free) {
		(true, false) => {
			acc_r[ia] += align_rotvec(da, db);
			cnt[ia] += 1;
		}
		(false, true) => {
			acc_r[ib] += align_rotvec(db, da);
			cnt[ib] += 1;
		}
		(true, true) => {
			acc_r[ia] += align_rotvec(da, db) * 0.5;
			cnt[ia] += 1;
			acc_r[ib] += align_rotvec(db, da) * 0.5;
			cnt[ib] += 1;
		}
		(false, false) => {}
	}
}

/// Translate a pose by `delta` in world space.
fn translate(pose: &mut DAffine3, delta: DVec3) {
	pose.translation += delta;
}

/// The rotation (as an axis·angle vector) that turns world direction `from` to
/// point along `to` — choosing the shorter of `to`/`-to` so the mate is treated as
/// a *parallel* (axis) alignment, never forcing a flip.
fn align_rotvec(from: DVec3, to: DVec3) -> DVec3 {
	let from = from.normalize_or_zero();
	let to = to.normalize_or_zero();
	if from.length_squared() <= EPS || to.length_squared() <= EPS {
		return DVec3::ZERO;
	}
	let to = if from.dot(to) >= 0.0 { to } else { -to };
	let (axis, angle) = stable_rotation_arc(from, to).to_axis_angle();
	axis * angle
}

/// Apply rotation `q` about the pose's world origin (its translation stays put).
fn rotate_about_origin(pose: &mut DAffine3, q: DQuat) {
	if q == DQuat::IDENTITY {
		return;
	}
	let rot = DAffine3::from_quat(q.normalize());
	let t = pose.translation;
	// new = T(t) * R * T(-t) * old, i.e. rotate the body in place.
	*pose = DAffine3::from_translation(t) * rot * DAffine3::from_translation(-t) * *pose;
}

/// Robust shortest-arc rotation from unit `from` to unit `to`, handling the
/// anti-parallel singularity that `from_rotation_arc` leaves under-defined.
fn stable_rotation_arc(from: DVec3, to: DVec3) -> DQuat {
	let d = from.dot(to).clamp(-1.0, 1.0);
	if d > 1.0 - 1e-9 {
		return DQuat::IDENTITY; // already aligned
	}
	if d < -1.0 + 1e-9 {
		// Opposite directions: rotate 180° about any axis perpendicular to `from`.
		let axis = orthonormal(from);
		return DQuat::from_axis_angle(axis, std::f64::consts::PI);
	}
	DQuat::from_rotation_arc(from, to)
}

/// Some unit vector orthogonal to (unit) `v`.
fn orthonormal(v: DVec3) -> DVec3 {
	let a = if v.x.abs() < 0.9 { DVec3::X } else { DVec3::Y };
	v.cross(a).normalize_or_zero()
}

/// Component of `point - line_point` perpendicular to unit-ish `dir`.
///
/// Returns the vector you must add to `point` to drop it onto the line through
/// `line_point` along `dir` (zero if `dir` is degenerate, in which case the full
/// offset is treated as perpendicular).
fn perpendicular_offset(point: DVec3, line_point: DVec3, dir: DVec3) -> DVec3 {
	let rel = point - line_point;
	let dir = dir.normalize_or_zero();
	if dir.length_squared() <= EPS {
		return rel;
	}
	rel - dir * rel.dot(dir)
}

/// Promote an f32 [`Affine3A`] to an exact f64 [`DAffine3`].
fn to_daffine3(m: &Affine3A) -> DAffine3 {
	let (s, r, t) = m.to_scale_rotation_translation();
	// A singular (zero-scale) linear part makes glam's polar decomposition return a
	// NaN quaternion; fall back to identity rotation AND repair any non-finite /
	// zero scale axis, so a degenerate pose cannot poison the solve with NaN and is
	// never written back to the caller as a non-finite transform.
	let r = if r.is_finite() { r } else { Quat::IDENTITY };
	let fix = |v: f32| if v.is_finite() && v != 0.0 { v } else { 1.0 };
	let s = Vec3::new(fix(s.x), fix(s.y), fix(s.z));
	DAffine3::from_scale_rotation_translation(s.as_dvec3(), r.as_dquat(), t.as_dvec3())
}

/// Demote an f64 [`DAffine3`] back to an f32 [`Affine3A`].
fn to_affine3a(m: &DAffine3) -> Affine3A {
	let (s, r, t) = m.to_scale_rotation_translation();
	Affine3A::from_scale_rotation_translation(
		Vec3::new(s.x as f32, s.y as f32, s.z as f32),
		Quat::from_xyzw(r.x as f32, r.y as f32, r.z as f32, r.w as f32),
		Vec3::new(t.x as f32, t.y as f32, t.z as f32),
	)
}

#[cfg(test)]
mod tests {
	use super::*;

	/// Default pose for a fresh instance.
	fn identity() -> Affine3A {
		Affine3A::IDENTITY
	}

	/// A two-instance system whose free instance 1 is pulled toward two conflicting
	/// targets `t_a`, `t_b` (in `+x`) by competing Coincident mates.
	fn two_targets(t_a: f64, t_b: f64) -> ConstraintSystem {
		let mut sys = ConstraintSystem::new(vec![Affine3A::IDENTITY, Affine3A::IDENTITY], vec![]);
		sys.add_constraint(Constraint::Coincident { a: 0, a_point: DVec3::new(t_a, 0.0, 0.0), b: 1, b_point: DVec3::ZERO });
		sys.add_constraint(Constraint::Coincident { a: 0, a_point: DVec3::new(t_b, 0.0, 0.0), b: 1, b_point: DVec3::ZERO });
		sys
	}

	#[test]
	fn over_constrained_settles_to_least_squares_not_last_wins() {
		// Instance 1's origin is pulled toward (±10, 0, 0) by two competing mates. The
		// damped solver must settle NEAR the least-squares minimum (residual ~200), not
		// park at one target (residual 400 — the old full-relaxation last-wins bug), and
		// the reported residual must be order-independent.
		let r1 = two_targets(10.0, -10.0).solve(300);
		let r2 = two_targets(-10.0, 10.0).solve(300); // swapped constraint order
												// Jacobi averaging reaches the EXACT least-squares minimum (midpoint, residual
												// 200), not the old last-wins 400, and is independent of constraint order.
		assert!((r1 - 200.0).abs() < 0.5, "over-constrained residual {r1} should be the 200 least-squares minimum");
		assert!((r1 - r2).abs() < 1e-6, "reported residual must be order-independent: {r1} vs {r2}");
	}

	#[test]
	fn coincident_brings_world_points_together() {
		// Ground instance 0 at origin; free instance 1 offset along x.
		let t0 = identity();
		let t1 = Affine3A::from_translation(Vec3::new(10.0, 0.0, 0.0));
		let mut sys = ConstraintSystem::new(vec![t0, t1], vec![]);
		// Local point (2,0,0) on the ground should meet local point (-3,0,0) on inst 1.
		sys.add_constraint(Constraint::Coincident { a: 0, a_point: DVec3::new(2.0, 0.0, 0.0), b: 1, b_point: DVec3::new(-3.0, 0.0, 0.0) });

		let residual = sys.solve(64);

		let p = &sys.transforms()[1];
		let wa = DVec3::new(2.0, 0.0, 0.0); // ground point in world
		let wb = to_daffine3(p).transform_point3(DVec3::new(-3.0, 0.0, 0.0));
		assert!((wa - wb).length() < 1e-4 && residual < 1e-8, "coincident points should meet: wa={wa:?} wb={wb:?} residual={residual}");
	}

	#[test]
	fn distance_holds_points_at_requested_separation() {
		let t0 = identity();
		let t1 = Affine3A::from_translation(Vec3::new(1.0, 0.0, 0.0));
		let mut sys = ConstraintSystem::new(vec![t0, t1], vec![]);
		let target = 7.5;
		sys.add_constraint(Constraint::Distance { a: 0, a_point: DVec3::ZERO, b: 1, b_point: DVec3::ZERO, distance: target });

		sys.solve(64);

		let wa = DVec3::ZERO;
		let wb = to_daffine3(&sys.transforms()[1]).transform_point3(DVec3::ZERO);
		assert!(((wa - wb).length() - target).abs() < 1e-4, "points should be {target} apart, got {}", (wa - wb).length());
	}

	#[test]
	fn parallel_aligns_two_directions() {
		// Ground points +x; free instance 1 currently rotated so its local +x
		// points along world +y. After solving its local +x must be parallel to
		// the ground's local +x.
		let t0 = identity();
		let t1 = Affine3A::from_rotation_translation(Quat::from_rotation_z(std::f32::consts::FRAC_PI_2), Vec3::new(0.0, 0.0, 0.0));
		let mut sys = ConstraintSystem::new(vec![t0, t1], vec![]);
		sys.add_constraint(Constraint::Parallel { a: 0, a_dir: DVec3::X, b: 1, b_dir: DVec3::X });

		sys.solve(64);

		let da = DVec3::X; // ground dir in world
		let db = to_daffine3(&sys.transforms()[1]).transform_vector3(DVec3::X).normalize_or_zero();
		assert!(da.cross(db).length() < 1e-4, "directions should be parallel, cross={:?}", da.cross(db));
	}

	#[test]
	fn concentric_makes_axes_collinear() {
		// Ground axis is the world z-axis through the origin. Instance 1's axis is
		// its local z, but it starts rotated (local z along world x) and offset.
		let t0 = identity();
		let t1 = Affine3A::from_rotation_translation(Quat::from_rotation_y(std::f32::consts::FRAC_PI_2), Vec3::new(5.0, 4.0, 9.0));
		let mut sys = ConstraintSystem::new(vec![t0, t1], vec![]);
		sys.add_constraint(Constraint::Concentric {
			a: 0,
			a_axis_point: DVec3::ZERO,
			a_axis_dir: DVec3::Z,
			b: 1,
			b_axis_point: DVec3::ZERO,
			b_axis_dir: DVec3::Z,
		});

		let residual = sys.solve(128);

		let p = to_daffine3(&sys.transforms()[1]);
		let dir = p.transform_vector3(DVec3::Z).normalize_or_zero();
		let pt = p.transform_point3(DVec3::ZERO);
		// Axis direction parallel to z, and the axis point lies on the world z-axis
		// (zero x/y offset).
		let parallel = DVec3::Z.cross(dir).length();
		let offset = perpendicular_offset(pt, DVec3::ZERO, DVec3::Z).length();
		assert!(
			parallel < 1e-4 && offset < 1e-4 && residual < 1e-6,
			"axes should be collinear: parallel={parallel} offset={offset} residual={residual}"
		);
	}

	#[test]
	fn ground_instance_never_moves() {
		let t0 = Affine3A::from_translation(Vec3::new(1.0, 2.0, 3.0));
		let t1 = Affine3A::from_translation(Vec3::new(20.0, 0.0, 0.0));
		let mut sys = ConstraintSystem::new(vec![t0, t1], vec![]);
		sys.add_constraint(Constraint::Coincident { a: 0, a_point: DVec3::ZERO, b: 1, b_point: DVec3::ZERO });

		sys.solve(32);

		let g = sys.transforms()[0].translation;
		assert!((g.as_dvec3() - DVec3::new(1.0, 2.0, 3.0)).length() < 1e-5, "ground (instance 0) must stay fixed, got {g:?}");
	}

	#[test]
	fn both_free_instances_share_the_correction() {
		// Neither instance is ground here EXCEPT index 0 is always ground, so make
		// a 3-body system: 0 ground, 1 and 2 both free, coincident between 1 and 2.
		let t0 = identity();
		let t1 = Affine3A::from_translation(Vec3::new(-6.0, 0.0, 0.0));
		let t2 = Affine3A::from_translation(Vec3::new(6.0, 0.0, 0.0));
		let mut sys = ConstraintSystem::new(vec![t0, t1, t2], vec![]);
		sys.add_constraint(Constraint::Coincident { a: 1, a_point: DVec3::ZERO, b: 2, b_point: DVec3::ZERO });

		let residual = sys.solve(128);

		let w1 = to_daffine3(&sys.transforms()[1]).transform_point3(DVec3::ZERO);
		let w2 = to_daffine3(&sys.transforms()[2]).transform_point3(DVec3::ZERO);
		assert!((w1 - w2).length() < 1e-4 && residual < 1e-8, "both-free coincident should meet in the middle: w1={w1:?} w2={w2:?}");
	}

	#[test]
	fn degenerate_inputs_do_not_panic() {
		// Out-of-range indices, zero-length directions, self references.
		let mut sys = ConstraintSystem::new(vec![identity(), identity()], vec![]);
		sys.add_constraint(Constraint::Coincident {
			a: 0,
			a_point: DVec3::ZERO,
			b: 99, // out of range
			b_point: DVec3::ZERO,
		});
		sys.add_constraint(Constraint::Parallel {
			a: 0,
			a_dir: DVec3::ZERO, // degenerate direction
			b: 1,
			b_dir: DVec3::ZERO,
		});
		sys.add_constraint(Constraint::Coincident {
			a: 1,
			a_point: DVec3::ZERO,
			b: 1, // self reference
			b_point: DVec3::X,
		});
		let r = sys.solve(8);
		assert!(r.is_finite(), "degenerate system must stay finite, got {r}");
	}

	#[test]
	fn angle_mate_holds_directions_at_the_target_angle() {
		// Ground holds +X; the free instance's local +X must settle 60° away.
		let mut sys = ConstraintSystem::new(vec![identity(), identity()], vec![]);
		sys.add_constraint(Constraint::Angle { a: 0, a_dir: DVec3::X, b: 1, b_dir: DVec3::X, degrees: 60.0 });
		let residual = sys.solve(256);
		let db = to_daffine3(&sys.transforms()[1]).transform_vector3(DVec3::X).normalize();
		let theta = db.dot(DVec3::X).clamp(-1.0, 1.0).acos().to_degrees();
		assert!(
			(theta - 60.0).abs() < 0.1 && residual < 1e-8,
			"angle mate must hold 60°: settled at {theta:.4}° (residual {residual:.3e})"
		);
	}

	#[test]
	fn axis_distance_mate_places_parallel_axes_at_center_distance() {
		// The gear mate: a free shaft seeded tilted and misplaced must end
		// parallel to the ground z-axis at EXACTLY the 20 mm center distance.
		let seed = Affine3A::from_rotation_translation(Quat::from_axis_angle(Vec3::new(1.0, 0.0, 0.0), 0.5), Vec3::new(31.0, 44.0, 7.0));
		let mut sys = ConstraintSystem::new(vec![identity(), seed], vec![]);
		sys.add_constraint(Constraint::AxisDistance {
			a: 0,
			a_axis_point: DVec3::ZERO,
			a_axis_dir: DVec3::Z,
			b: 1,
			b_axis_point: DVec3::ZERO,
			b_axis_dir: DVec3::Z,
			distance: 20.0,
		});
		let residual = sys.solve(512);
		let pose = to_daffine3(&sys.transforms()[1]);
		let db = pose.transform_vector3(DVec3::Z).normalize();
		let cross = db.cross(DVec3::Z).length();
		let gap = perpendicular_offset(pose.transform_point3(DVec3::ZERO), DVec3::ZERO, DVec3::Z).length();
		assert!(
			cross < 1e-4 && (gap - 20.0).abs() < 1e-3 && residual < 1e-6,
			"axis_distance must yield parallel axes 20mm apart: cross={cross:.2e}, gap={gap:.6}, residual={residual:.3e}"
		);
	}

	#[test]
	fn fixed_mate_grounds_any_instance_not_just_index_zero() {
		// Instance 1 is Fixed at (5,0,0); a Coincident tries to drag it to the
		// origin and must fail to move it, while free instance 2 still solves.
		let held = Affine3A::from_translation(Vec3::new(5.0, 0.0, 0.0));
		let mut sys = ConstraintSystem::new(vec![identity(), held, identity()], vec![]);
		sys.add_constraint(Constraint::Fixed { instance: 1 });
		sys.add_constraint(Constraint::Coincident { a: 0, a_point: DVec3::ZERO, b: 1, b_point: DVec3::ZERO });
		sys.add_constraint(Constraint::Coincident { a: 1, a_point: DVec3::ZERO, b: 2, b_point: DVec3::ZERO });
		sys.solve(128);
		let p1 = sys.transforms()[1].translation;
		let p2 = to_daffine3(&sys.transforms()[2]).transform_point3(DVec3::ZERO);
		assert!(
			(p1.x - 5.0).abs() < 1e-6 && (p2 - DVec3::new(5.0, 0.0, 0.0)).length() < 1e-4,
			"Fixed must hold instance 1 at x=5 (got {p1:?}) while instance 2 mates onto it (got {p2:?})"
		);
	}

	#[test]
	fn validate_names_every_statically_broken_mate() {
		let mut sys = ConstraintSystem::new(vec![identity(), identity()], vec![]);
		sys.add_constraint(Constraint::Coincident { a: 0, a_point: DVec3::ZERO, b: 99, b_point: DVec3::ZERO });
		sys.add_constraint(Constraint::Parallel { a: 0, a_dir: DVec3::ZERO, b: 1, b_dir: DVec3::X });
		sys.add_constraint(Constraint::Coincident { a: 1, a_point: DVec3::ZERO, b: 1, b_point: DVec3::X });
		sys.add_constraint(Constraint::Angle { a: 0, a_dir: DVec3::X, b: 1, b_dir: DVec3::X, degrees: 270.0 });
		sys.add_constraint(Constraint::Concentric {
			a: 0,
			a_axis_point: DVec3::ZERO,
			a_axis_dir: DVec3::Z,
			b: 1,
			b_axis_point: DVec3::ZERO,
			b_axis_dir: DVec3::Z,
		});
		let problems = sys.validate();
		let text = problems.join("\n");
		assert!(
			problems.len() == 4
				&& text.contains("mate 0 (coincident): instance index out of range")
				&& text.contains("mate 1 (parallel): zero-length direction")
				&& text.contains("mate 2 (coincident): references instance 1 on both sides")
				&& text.contains("mate 3 (angle): target 270° outside [0, 180]"),
			"validate must name exactly the 4 broken mates (index + kind + why) and pass the healthy concentric; got {} problems:\n{text}",
			problems.len()
		);
	}

	#[test]
	fn per_constraint_residuals_isolate_the_unsatisfied_mate() {
		// A satisfied Parallel plus an unsatisfiable Distance-vs-Coincident pair:
		// the per-mate receipts must show ~0 for the healthy mate and split the
		// conflict across mates 1 and 2 — naming culprits, not one anonymous sum.
		let mut sys = ConstraintSystem::new(vec![identity(), identity()], vec![]);
		sys.add_constraint(Constraint::Parallel { a: 0, a_dir: DVec3::Z, b: 1, b_dir: DVec3::Z });
		sys.add_constraint(Constraint::Coincident { a: 0, a_point: DVec3::ZERO, b: 1, b_point: DVec3::ZERO });
		sys.add_constraint(Constraint::Distance { a: 0, a_point: DVec3::ZERO, b: 1, b_point: DVec3::ZERO, distance: 4.0 });
		sys.solve(256);
		let per = sys.per_constraint_residuals();
		assert!(
			per.len() == 3 && per[0] < 1e-9 && per[1] > 0.5 && per[2] > 0.5,
			"per-mate residuals must isolate the conflict (parallel ~0, coincident vs distance both violated): {per:?}"
		);
	}

	#[test]
	fn dof_analysis_counts_the_shaft_in_bore_mating_ladder() {
		// The flagship under-constraint story: a shaft concentric in a bore has 2
		// free DOF (slide + spin); adding an axial Distance leaves 1 (spin);
		// adding an Angle clock leaves 0 (well-constrained). The old solver
		// reported an identical happy residual at every rung.
		let axis = |sys: &mut ConstraintSystem| {
			sys.add_constraint(Constraint::Concentric {
				a: 0,
				a_axis_point: DVec3::ZERO,
				a_axis_dir: DVec3::Z,
				b: 1,
				b_axis_point: DVec3::ZERO,
				b_axis_dir: DVec3::Z,
			});
		};
		let mut s1 = ConstraintSystem::new(vec![identity(), identity()], vec![]);
		axis(&mut s1);
		let d1 = s1.analyze();

		let mut s2 = ConstraintSystem::new(vec![identity(), identity()], vec![]);
		axis(&mut s2);
		s2.add_constraint(Constraint::Distance { a: 0, a_point: DVec3::ZERO, b: 1, b_point: DVec3::new(0.0, 0.0, 3.0), distance: 3.0 });
		let d2 = s2.analyze();

		let mut s3 = ConstraintSystem::new(vec![identity(), identity()], vec![]);
		axis(&mut s3);
		s3.add_constraint(Constraint::Distance { a: 0, a_point: DVec3::ZERO, b: 1, b_point: DVec3::new(0.0, 0.0, 3.0), distance: 3.0 });
		s3.add_constraint(Constraint::Angle { a: 0, a_dir: DVec3::X, b: 1, b_dir: DVec3::Y, degrees: 90.0 });
		let d3 = s3.analyze();

		// Redundancy: a duplicated coincident adds 3 rows and no rank.
		let mut s4 = ConstraintSystem::new(vec![identity(), identity()], vec![]);
		s4.add_constraint(Constraint::Coincident { a: 0, a_point: DVec3::ZERO, b: 1, b_point: DVec3::ZERO });
		s4.add_constraint(Constraint::Coincident { a: 0, a_point: DVec3::ZERO, b: 1, b_point: DVec3::ZERO });
		let d4 = s4.analyze();

		assert!(
			d1.free_dof == 2
				&& d1.verdict == "under_constrained (2 free DOF)"
				&& d2.free_dof == 1
				&& d3.free_dof == 0
				&& d3.verdict == "well_constrained"
				&& d4.redundant_rows == 3
				&& d4.free_dof == 3,
			"DOF ladder must read 2 → 1 → 0 free (and the duplicate coincident must show 3 redundant rows):\n\
			 concentric only:        {d1:?}\n\
			 + axial distance:       {d2:?}\n\
			 + angle clock:          {d3:?}\n\
			 duplicated coincident:  {d4:?}"
		);
	}

	#[test]
	fn competing_rotational_mates_settle_seed_dependently_documented_nonconvexity() {
		// PINS the documented limitation (module docs + RELAX): one body
		// direction pulled toward two world targets 90° apart is a non-convex
		// rotational compromise — different seeds may settle at different
		// orientations of comparable residual. The pin asserts (a) both runs
		// IMPROVE on their seed, (b) residuals land in the same band, and
		// (c) the solved orientations genuinely differ — i.e. the result is
		// seed-dependent, which is exactly what callers must know.
		// A FRUSTRATED pair on two body directions with non-coplanar targets:
		// body X wants world X while orthogonal body Y wants a target tilted 45°
		// into XZ — impossible simultaneously, so the compromise has distinct
		// prioritize-one-or-the-other optima.
		let t2 = DVec3::new(std::f64::consts::FRAC_1_SQRT_2, 0.0, std::f64::consts::FRAC_1_SQRT_2);
		let solve_from = |seed: Quat| {
			let pose = Affine3A::from_rotation_translation(seed, Vec3::ZERO);
			let mut sys = ConstraintSystem::new(vec![identity(), pose], vec![]);
			sys.add_constraint(Constraint::Parallel { a: 0, a_dir: DVec3::X, b: 1, b_dir: DVec3::X });
			sys.add_constraint(Constraint::Parallel { a: 0, a_dir: t2, b: 1, b_dir: DVec3::Y });
			let before = sys.residual();
			let after = sys.solve(256);
			let dir = to_daffine3(&sys.transforms()[1]).transform_vector3(DVec3::X).normalize();
			(before, after, dir)
		};
		// Seed 1 sits near the identity family; seed 2 near the 180°-flipped
		// family (Parallel accepts anti-parallel, so BOTH are valid basins that
		// the solver never crosses between). Each run must descend within its
		// own basin and the two settled bodies must differ as WORLD orientations.
		let (b1, a1, dir1) = solve_from(Quat::from_axis_angle(Vec3::new(1.0, 0.4, 0.2).normalize(), 0.5));
		let (b2, a2, dir2) =
			solve_from(Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), 2.9) * Quat::from_axis_angle(Vec3::new(1.0, 0.0, 0.0), 0.4));
		let spread = dir1.dot(dir2).clamp(-1.0, 1.0).acos().to_degrees();
		let band = a1.max(a2) / a1.min(a2).max(1e-12);
		assert!(
			a1 < b1 - 1e-3 && a2 < b2 - 1e-3 && band < 1.5 && spread > 10.0,
			"rotational non-convexity pin: both seeds must descend (b1={b1:.3} → a1={a1:.3}, b2={b2:.3} → a2={a2:.3}), \
			 land in one residual band (ratio {band:.2}), and settle at DIFFERENT orientations \
			 ({spread:.1}° apart) — if spread ~0 the solver has become seed-independent and the \
			 documented limitation (and this pin) should be retired"
		);
	}
}
