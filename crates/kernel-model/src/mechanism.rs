// Copyright (c) LMCAD. Licensed under the MIT License.

//! **Mechanism kinematics — motion over time.** A [`Mechanism`] is links joined
//! by revolute / prismatic joints with one joint DRIVEN; [`Mechanism::sweep`]
//! walks the driven coordinate through a cycle and returns a [`MotionReport`]:
//! every link's pose at every step, the range of motion, tracked-point traces,
//! the minimum clearance over the whole cycle, and the FIRST interference —
//! including the ones that only exist mid-cycle, which is the entire reason
//! this exists. A part pair checked at its two end poses can be perfectly clear
//! at both and collide in between.
//!
//! # Method
//!
//! Planar (XY) rigid-body loop closure. Each link is `(x, y, θ)`; link `0` is
//! **ground** and is frozen at its declared pose. Every joint contributes its
//! constraint rows and the driven joint contributes one more (its coordinate =
//! the commanded value); the square system is solved by damped
//! Newton–Raphson with an analytic Jacobian, warm-started from the previous
//! step so the solution stays on one branch (a branch flip would show up as a
//! discontinuity in [`MotionReport::max_step_translation_jump`], which is why
//! that number is reported and worth gating on).
//!
//! # Mobility, and the formula it is counted with
//!
//! [`Mechanism::mobility`] reports the planar **Kutzbach / Grübler** count
//!
//! ```text
//! F = 3·(n − 1) − 2·j₁ − j₂
//! ```
//!
//! with `n` links (ground included), `j₁` one-DOF lower pairs and `j₂`
//! two-DOF higher pairs (Grübler 1883 / Kutzbach 1929; see Norton,
//! *Design of Machinery*, §2.5). Revolute and prismatic joints are both
//! one-DOF lower pairs, so `j₂ = 0` here and each joint removes two DOF.
//!
//! The criterion counts *ideal* constraints and is famously wrong for special
//! geometries — a parallelogram linkage counts `F = 0` and moves anyway,
//! because one of its constraints is redundant. So [`MobilityReport`] also
//! carries a **numeric** count: the rank of the constraint Jacobian at the
//! declared pose gives `rank_dof = 3(n−1) − rank`, mirroring the doctrine of
//! [`crate::constraints::ConstraintSystem::analyze`]. When the two disagree,
//! the mechanism is a Grübler paradox and the refusal says so instead of
//! pretending the formula settled it.
//!
//! # Refusals
//!
//! Motion is only computed for a mechanism that can actually move under
//! exactly one command: `kutzbach_dof ≤ 0` is [`MechanismError::Locked`],
//! `dof > 1` is [`MechanismError::Underdriven`], more than one driven joint is
//! [`MechanismError::Overdriven`], a Jacobian that goes singular mid-cycle is
//! [`MechanismError::Singular`] (a dead-centre / change-point configuration),
//! and a commanded value the linkage cannot reach is
//! [`MechanismError::Convergence`]. None of these degrade silently into a
//! plausible-looking pose list.
//!
//! # Relationship to [`crate::kinematics`]
//!
//! [`crate::kinematics`] holds the **closed-form** evaluators for the gear
//! trains this repo builds (epicyclic, strain-wave, cycloidal): fixed
//! topology, exact formulas, install-phase conventions. Use it for those — it
//! is exact and far cheaper. This module is the general planar-linkage case
//! solved numerically, for the mechanisms that have no closed form.
//!
//! # Scope and limits
//!
//! - **Planar only** (motion in XY, rotation about Z). No spatial linkages,
//!   no spherical joints; a spatial mechanism must be reduced to its planar
//!   projection by the caller, who then owns that reduction.
//! - **Kinematics only.** No velocities, accelerations, forces, friction or
//!   inertia — [`crate::loads`] does statics.
//! - **Rigid links.** No compliance, no backlash, no joint clearance.
//! - Interference is delegated in full to [`crate::sweep_check`] and inherits
//!   its contract, including the exact triangle-crossing oracle that the
//!   vertex-sampled penetration estimate cannot fake.

use serde::{Deserialize, Serialize};

use kernel_core::math::{DAffine3, DVec2, DVec3};
use kernel_core::mesh::Mesh;

use crate::{sweep_check, SweepReport};

/// Newton convergence tolerance on the infinity norm of the constraint
/// residual (mm / rad). 1e-10 mm is 0.1 picometre — far below any physical
/// meaning, and loose enough that the damped step never stalls against
/// double-precision round-off at 100 mm link scales and reports a spurious
/// [`MechanismError::Convergence`].
pub const NEWTON_TOL: f64 = 1e-10;

/// Maximum Newton iterations per step before [`MechanismError::Convergence`].
const NEWTON_MAX_ITERS: usize = 100;

/// Maximum step halvings per Newton iteration.
const NEWTON_MAX_BACKTRACKS: usize = 24;

/// Continuation sub-steps [`Mechanism::pose_at`] uses to walk from the
/// declared configuration to the commanded one without changing branch.
const POSE_AT_SUBSTEPS: usize = 16;

/// Relative pivot tolerance for the square linear solve.
const PIVOT_EPS: f64 = 1e-12;

/// A planar pose: position and rotation about +Z.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Pose2 {
	/// X (mm).
	pub x: f64,
	/// Y (mm).
	pub y: f64,
	/// Rotation about +Z (rad).
	pub theta: f64,
}

impl Pose2 {
	/// A pose from its three coordinates.
	pub fn new(x: f64, y: f64, theta: f64) -> Self {
		Self { x, y, theta }
	}

	/// The identity pose.
	pub fn identity() -> Self {
		Self { x: 0.0, y: 0.0, theta: 0.0 }
	}

	/// Map a point from this link's local frame into world.
	pub fn apply(&self, p: DVec2) -> DVec2 {
		let (s, c) = self.theta.sin_cos();
		DVec2::new(self.x + c * p.x - s * p.y, self.y + s * p.x + c * p.y)
	}

	/// The 3-D rigid transform of this planar pose (rotation about +Z,
	/// translation in the Z = 0 plane) — the form [`crate::sweep_check`] and
	/// [`crate::Assembly`] take.
	pub fn to_affine(self) -> DAffine3 {
		DAffine3::from_translation(DVec3::new(self.x, self.y, 0.0)) * DAffine3::from_rotation_z(self.theta)
	}
}

/// One rigid link. Link `0` of a [`Mechanism`] is ground.
pub struct Link {
	/// Name — appears in every report and refusal.
	pub name: String,
	/// The pose the mechanism is declared at. Used as the Newton warm start
	/// and, for ground, as the frozen pose. It need not satisfy the
	/// constraints exactly; the first solve snaps it (and reports how far it
	/// had to move, see [`MotionReport::initial_snap`]).
	pub initial: Pose2,
	/// Optional geometry in the link's LOCAL frame, for interference checks.
	/// Links without a mesh are simply not checked.
	pub mesh: Option<Mesh>,
}

impl Link {
	/// A link with no geometry.
	pub fn new(name: impl Into<String>, initial: Pose2) -> Self {
		Self { name: name.into(), initial, mesh: None }
	}

	/// A link carrying local-frame geometry for interference checking.
	pub fn with_mesh(name: impl Into<String>, initial: Pose2, mesh: Mesh) -> Self {
		Self { name: name.into(), initial, mesh: Some(mesh) }
	}
}

/// The joint kinds this module solves. Both are one-DOF lower pairs and both
/// remove two planar DOF.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum JointKind {
	/// A pin joint: the two local points coincide in world space. Its
	/// coordinate (what a DRIVEN revolute commands) is the relative angle
	/// `θ_b − θ_a`, in radians.
	Revolute {
		/// Pin location in link `a`'s local frame.
		a_point: DVec2,
		/// Pin location in link `b`'s local frame.
		b_point: DVec2,
	},
	/// A slider: `b` translates along a fixed direction of `a` with a fixed
	/// relative angle. Its coordinate (what a DRIVEN prismatic commands) is
	/// the slide distance along the axis, in mm.
	Prismatic {
		/// Slide direction in link `a`'s local frame (need not be normalized;
		/// zero is refused).
		axis_in_a: DVec2,
		/// Reference point on `a`, local.
		a_point: DVec2,
		/// Reference point on `b`, local.
		b_point: DVec2,
	},
}

impl JointKind {
	/// Stable lowercase name.
	pub fn name(&self) -> &'static str {
		match self {
			JointKind::Revolute { .. } => "revolute",
			JointKind::Prismatic { .. } => "prismatic",
		}
	}

	/// Planar DOF this joint removes (2 for both lower pairs).
	pub fn constraints(&self) -> usize {
		2
	}
}

/// A joint between two links.
pub struct Joint {
	/// Name — appears in every report and refusal.
	pub name: String,
	/// First link index.
	pub a: usize,
	/// Second link index.
	pub b: usize,
	/// What kind of pair it is.
	pub kind: JointKind,
	/// Whether this joint is the commanded input.
	pub driven: bool,
}

impl Joint {
	/// A free revolute.
	pub fn revolute(name: impl Into<String>, a: usize, a_point: DVec2, b: usize, b_point: DVec2) -> Self {
		Self { name: name.into(), a, b, kind: JointKind::Revolute { a_point, b_point }, driven: false }
	}

	/// A free prismatic.
	pub fn prismatic(name: impl Into<String>, a: usize, a_point: DVec2, axis_in_a: DVec2, b: usize, b_point: DVec2) -> Self {
		Self { name: name.into(), a, b, kind: JointKind::Prismatic { axis_in_a, a_point, b_point }, driven: false }
	}

	/// Mark this joint as the commanded input (builder form).
	pub fn driven(mut self) -> Self {
		self.driven = true;
		self
	}
}

/// A point on a link whose path through the cycle is recorded.
pub struct TrackedPoint {
	/// Name — appears in the trace.
	pub name: String,
	/// Link index.
	pub link: usize,
	/// Point in that link's local frame.
	pub local: DVec2,
}

/// A planar linkage: links, joints, and the points whose motion is tracked.
///
/// Link `0` is ground.
#[derive(Default)]
pub struct Mechanism {
	/// Name — appears in every report and refusal.
	pub name: String,
	/// The links; index `0` is ground.
	pub links: Vec<Link>,
	/// The joints.
	pub joints: Vec<Joint>,
	/// Points whose paths [`Mechanism::sweep`] records.
	pub tracked: Vec<TrackedPoint>,
	/// Link pairs to check for interference, or `None` for the default rule:
	/// every unordered pair of mesh-carrying links that no joint directly
	/// connects (links joined by a joint touch by design).
	pub check_pairs: Option<Vec<(usize, usize)>>,
}

/// The mobility accounting of a [`Mechanism`] — the formula count and the
/// numeric count, side by side.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MobilityReport {
	/// Links, ground included.
	pub links: usize,
	/// Moving links `n − 1`.
	pub moving_links: usize,
	/// Joints.
	pub joints: usize,
	/// One-DOF lower pairs (revolute + prismatic).
	pub lower_pairs: usize,
	/// Two-DOF higher pairs — always 0 here; carried so the formula reads in
	/// full.
	pub higher_pairs: usize,
	/// Planar coordinates available: `3·(n − 1)`.
	pub coordinates: usize,
	/// Constraint rows the joints contribute: `2·j₁ + j₂`.
	pub constraint_rows: usize,
	/// `3(n−1) − 2j₁ − j₂` (Kutzbach / Grübler). May be ≤ 0.
	pub kutzbach_dof: i64,
	/// Numeric rank of the constraint Jacobian at the declared poses.
	pub jacobian_rank: usize,
	/// `3(n−1) − jacobian_rank` — the mobility the geometry actually has.
	pub rank_dof: i64,
	/// Set when `kutzbach_dof != rank_dof`: a Grübler paradox (redundant
	/// constraints, e.g. a parallelogram linkage).
	pub paradox: bool,
	/// Driven joints declared.
	pub driven: usize,
	/// The formula with its numbers substituted.
	pub formula: String,
	/// Human-readable verdict.
	pub verdict: String,
}

/// One tracked point's path through the cycle.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TrackTrace {
	/// Tracked-point name.
	pub name: String,
	/// Link index.
	pub link: usize,
	/// World positions, one per step.
	pub points: Vec<[f64; 2]>,
	/// `[x_min, x_max, y_min, y_max]` over the cycle.
	pub extents: [f64; 4],
}

/// One link's range of motion over the cycle.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct LinkRange {
	/// Link name.
	pub name: String,
	/// Minimum body angle over the cycle (rad).
	pub theta_min: f64,
	/// Maximum body angle over the cycle (rad).
	pub theta_max: f64,
	/// `theta_max − theta_min` (rad) — the swing of a rocker.
	pub theta_range: f64,
	/// `[x_min, x_max, y_min, y_max]` of the link origin (mm) — the stroke of
	/// a slider is `x_max − x_min` when it slides along X.
	pub origin_extents: [f64; 4],
}

/// The first interference found in the cycle.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Interference {
	/// Step index (into [`MotionReport::poses_per_step`]).
	pub step: usize,
	/// Driven coordinate at that step.
	pub driven_value: f64,
	/// The colliding link pair.
	pub pair: (usize, usize),
	/// Their names.
	pub pair_names: (String, String),
	/// Sampled penetration depth at that step (mm). **May be 0.0 while
	/// `crossing` is true** — the estimate is vertex-sampled and blind to an
	/// edge–edge crossing with no contained vertex (see
	/// [`crate::penetration_estimate`]).
	pub penetration: f64,
	/// Surface distance at that step (mm).
	pub min_distance: f64,
	/// The exact triangle-level proper-crossing verdict — the oracle that
	/// convicts where sampling is blind.
	pub crossing: bool,
}

/// Per-pair sweep summary, for the receipt.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PairSweep {
	/// The link pair.
	pub pair: (usize, usize),
	/// Their names.
	pub pair_names: (String, String),
	/// Smallest clearance over the cycle (mm).
	pub min_clearance: f64,
	/// Deepest sampled penetration over the cycle (mm).
	pub max_penetration: f64,
	/// Poses at ≈0 distance (touching OR crossing).
	pub contacts: usize,
	/// Poses with an exact proper triangle crossing.
	pub crossings: usize,
}

/// The result of sweeping a driven joint through a cycle.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MotionReport {
	/// Mechanism name.
	pub mechanism: String,
	/// Number of steps recorded (poses = `steps`).
	pub steps: usize,
	/// The mobility accounting the sweep was allowed by.
	pub mobility: MobilityReport,
	/// Driven joint name.
	pub driven_joint: String,
	/// The commanded coordinate at each step (rad for a revolute, mm for a
	/// prismatic).
	pub driven_value: Vec<f64>,
	/// `[step][link]` poses.
	pub poses_per_step: Vec<Vec<Pose2>>,
	/// Per-link range of motion over the cycle.
	pub range_of_motion: Vec<LinkRange>,
	/// Tracked-point paths.
	pub traces: Vec<TrackTrace>,
	/// Smallest clearance seen across every checked pair and step (mm);
	/// `f64::INFINITY` when no pair carries geometry.
	pub min_clearance_over_cycle: f64,
	/// The FIRST step at which any checked pair interferes (exact crossing or
	/// non-zero sampled penetration).
	pub first_interference: Option<Interference>,
	/// Per-pair sweep summaries.
	pub pair_sweeps: Vec<PairSweep>,
	/// Largest link-origin move between consecutive steps (mm) — a branch
	/// flip or a skipped assembly mode shows up here as a jump.
	pub max_step_translation_jump: f64,
	/// Largest link body-angle change between consecutive steps (rad),
	/// measured on the shortest arc.
	pub max_step_rotation_jump: f64,
	/// Largest Newton residual left at any step (mm / rad).
	pub max_newton_residual: f64,
	/// How far the declared link ORIGINS had to move to satisfy the
	/// constraints at the first commanded value (mm, max over links) — large
	/// means the declared configuration was not assembled. Translation only:
	/// a declared pose whose angles are off but whose origins already sit
	/// right reads ~0 here.
	pub initial_snap: f64,
}

/// Typed refusals from the mechanism module.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum MechanismError {
	/// Structurally malformed input.
	BadInput {
		/// Mechanism name.
		mechanism: String,
		/// What is wrong.
		detail: String,
	},
	/// Mobility ≤ 0: the linkage is a structure, not a mechanism.
	Locked {
		/// Mechanism name.
		mechanism: String,
		/// The Kutzbach count.
		dof: i64,
		/// The Jacobian-rank count.
		rank_dof: i64,
		/// The formula with numbers substituted.
		formula: String,
		/// What to do about it.
		hint: String,
	},
	/// Mobility > 1 with a single command: the configuration is not
	/// determined by the driven coordinate alone.
	Underdriven {
		/// Mechanism name.
		mechanism: String,
		/// The Kutzbach count.
		dof: i64,
		/// Driven joints declared.
		driven: usize,
		/// What to do about it.
		hint: String,
	},
	/// More than one driven joint, or a driven joint on a mobility-1 linkage
	/// that already has one.
	Overdriven {
		/// Mechanism name.
		mechanism: String,
		/// The Kutzbach count.
		dof: i64,
		/// Driven joints declared.
		driven: usize,
		/// What to do about it.
		hint: String,
	},
	/// No joint is marked driven, so there is nothing to sweep.
	NoDrivenJoint {
		/// Mechanism name.
		mechanism: String,
	},
	/// A full-cycle [`Mechanism::sweep`] was asked of a prismatic drive, which
	/// has no natural 2π cycle.
	NonCyclicDrive {
		/// Mechanism name.
		mechanism: String,
		/// The driven joint's name.
		joint: String,
		/// What to use instead.
		hint: String,
	},
	/// The Jacobian went singular: a dead-centre / change-point configuration
	/// where the linkage can branch.
	Singular {
		/// Mechanism name.
		mechanism: String,
		/// Which step.
		step: usize,
		/// The commanded value there.
		driven_value: f64,
		/// What it means.
		hint: String,
	},
	/// Newton did not converge — usually a commanded value outside the
	/// linkage's reachable range.
	Convergence {
		/// Mechanism name.
		mechanism: String,
		/// Which step.
		step: usize,
		/// The commanded value there.
		driven_value: f64,
		/// Residual left (mm / rad).
		residual: f64,
		/// What it means.
		hint: String,
	},
}

impl std::fmt::Display for MechanismError {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		match self {
			MechanismError::BadInput { mechanism, detail } => write!(f, "mechanism '{mechanism}': {detail}"),
			MechanismError::Locked { mechanism, dof, rank_dof, formula, hint } => write!(
				f,
				"mechanism '{mechanism}' is LOCKED: mobility {dof} (≤ 0) — it is a structure, not a mechanism. {formula}. \
				 Jacobian-rank mobility at the declared pose: {rank_dof}. {hint}"
			),
			MechanismError::Underdriven { mechanism, dof, driven, hint } => write!(
				f,
				"mechanism '{mechanism}' has mobility {dof} but {driven} driven joint(s): the configuration is not determined by \
				 the command. {hint}"
			),
			MechanismError::Overdriven { mechanism, dof, driven, hint } => write!(
				f,
				"mechanism '{mechanism}' has mobility {dof} but {driven} driven joint(s): the commands fight each other. {hint}"
			),
			MechanismError::NoDrivenJoint { mechanism } => write!(
				f,
				"mechanism '{mechanism}' has no driven joint — mark the input joint with Joint::driven() before asking for motion"
			),
			MechanismError::NonCyclicDrive { mechanism, joint, hint } => {
				write!(f, "mechanism '{mechanism}': driven joint '{joint}' is prismatic and has no 2π cycle. {hint}")
			}
			MechanismError::Singular { mechanism, step, driven_value, hint } => write!(
				f,
				"mechanism '{mechanism}': the constraint Jacobian is SINGULAR at step {step} (driven = {driven_value:.6}) — a \
				 dead-centre / change-point configuration where the linkage can switch branch. {hint}"
			),
			MechanismError::Convergence { mechanism, step, driven_value, residual, hint } => write!(
				f,
				"mechanism '{mechanism}': loop closure did not converge at step {step} (driven = {driven_value:.6}), residual \
				 {residual:.3e} after {NEWTON_MAX_ITERS} iterations. {hint}"
			),
		}
	}
}

impl std::error::Error for MechanismError {}

impl Mechanism {
	/// An empty named mechanism.
	pub fn new(name: impl Into<String>) -> Self {
		Self { name: name.into(), links: Vec::new(), joints: Vec::new(), tracked: Vec::new(), check_pairs: None }
	}

	/// Add a link, returning its index. The first link added is ground.
	pub fn add_link(&mut self, link: Link) -> usize {
		let i = self.links.len();
		self.links.push(link);
		i
	}

	/// Add a joint, returning its index.
	pub fn add_joint(&mut self, joint: Joint) -> usize {
		let i = self.joints.len();
		self.joints.push(joint);
		i
	}

	/// Track a local point of a link through the cycle.
	pub fn track(&mut self, name: impl Into<String>, link: usize, local: DVec2) {
		self.tracked.push(TrackedPoint { name: name.into(), link, local });
	}

	/// Structural validation, run first by every solve entry point.
	pub fn validate(&self) -> Result<(), MechanismError> {
		let bad = |detail: String| MechanismError::BadInput { mechanism: self.name.clone(), detail };
		if self.links.len() < 2 {
			return Err(bad(format!("needs at least 2 links (ground + one moving), has {}", self.links.len())));
		}
		for l in &self.links {
			if !l.initial.x.is_finite() || !l.initial.y.is_finite() || !l.initial.theta.is_finite() {
				return Err(bad(format!("link '{}' has a non-finite initial pose", l.name)));
			}
		}
		for j in &self.joints {
			if j.a >= self.links.len() || j.b >= self.links.len() {
				return Err(bad(format!("joint '{}' references links ({}, {}) but only {} exist", j.name, j.a, j.b, self.links.len())));
			}
			if j.a == j.b {
				return Err(bad(format!("joint '{}' joins link {} to itself — it constrains nothing", j.name, j.a)));
			}
			if let JointKind::Prismatic { axis_in_a, .. } = j.kind {
				if !axis_in_a.is_finite() || axis_in_a.length() < 1e-12 {
					return Err(bad(format!("prismatic joint '{}' has a zero-length axis — it has no slide direction", j.name)));
				}
			}
		}
		for t in &self.tracked {
			if t.link >= self.links.len() {
				return Err(bad(format!("tracked point '{}' references link {} but only {} exist", t.name, t.link, self.links.len())));
			}
		}
		if let Some(pairs) = &self.check_pairs {
			for &(i, j) in pairs {
				if i >= self.links.len() || j >= self.links.len() {
					return Err(bad(format!("check_pairs entry ({i}, {j}) is out of range for {} links", self.links.len())));
				}
			}
		}
		Ok(())
	}

	/// Indices of the joints marked driven.
	pub fn driven_joints(&self) -> Vec<usize> {
		self.joints.iter().enumerate().filter(|(_, j)| j.driven).map(|(i, _)| i).collect()
	}

	/// The mobility accounting: the Kutzbach / Grübler count AND the numeric
	/// Jacobian-rank count at the declared poses. See the
	/// [module docs](self) for the formula and its citation.
	pub fn mobility(&self) -> MobilityReport {
		let n = self.links.len();
		let moving = n.saturating_sub(1);
		let coordinates = 3 * moving;
		let lower_pairs = self.joints.len();
		let higher_pairs = 0usize;
		let constraint_rows: usize = self.joints.iter().map(|j| j.kind.constraints()).sum();
		let kutzbach_dof = coordinates as i64 - constraint_rows as i64;

		let poses: Vec<Pose2> = self.links.iter().map(|l| l.initial).collect();
		let mut jac = vec![vec![0.0f64; coordinates]; constraint_rows];
		let mut scratch = vec![0.0f64; constraint_rows];
		let mut row = 0usize;
		for j in &self.joints {
			self.joint_rows(j, &poses, &mut jac, &mut scratch, row);
			row += j.kind.constraints();
		}
		let jacobian_rank = matrix_rank(&mut jac);
		let rank_dof = coordinates as i64 - jacobian_rank as i64;
		let paradox = kutzbach_dof != rank_dof;
		let formula = format!(
			"Kutzbach (planar): F = 3(n-1) - 2*j1 - j2 = 3({n}-1) - 2*{lower_pairs} - {higher_pairs} = {kutzbach_dof}"
		);
		let verdict = if paradox {
			format!("GRUBLER PARADOX: formula says {kutzbach_dof}, Jacobian rank at the declared pose says {rank_dof} (redundant constraints)")
		} else if kutzbach_dof <= 0 {
			format!("locked ({kutzbach_dof} DOF): a structure, not a mechanism")
		} else {
			format!("mobility {kutzbach_dof}")
		};
		MobilityReport {
			links: n,
			moving_links: moving,
			joints: self.joints.len(),
			lower_pairs,
			higher_pairs,
			coordinates,
			constraint_rows,
			kutzbach_dof,
			jacobian_rank,
			rank_dof,
			paradox,
			driven: self.driven_joints().len(),
			formula,
			verdict,
		}
	}

	/// The driven joint's coordinate at the declared poses (rad for a
	/// revolute, mm for a prismatic).
	pub fn driven_coordinate(&self) -> Result<f64, MechanismError> {
		self.validate()?;
		let idx = self.single_driven()?;
		let poses: Vec<Pose2> = self.links.iter().map(|l| l.initial).collect();
		Ok(self.drive_coordinate(&self.joints[idx], &poses))
	}

	/// Solve the linkage at ONE commanded value of the driven coordinate.
	///
	/// Continuation-stepped from the declared configuration in
	/// [`POSE_AT_SUBSTEPS`] sub-steps so a large command does not jump branch.
	pub fn pose_at(&self, driven: f64) -> Result<Vec<Pose2>, MechanismError> {
		self.gate_for_motion()?;
		let idx = self.single_driven()?;
		let mut poses: Vec<Pose2> = self.links.iter().map(|l| l.initial).collect();
		let q0 = self.drive_coordinate(&self.joints[idx], &poses);
		for k in 1..=POSE_AT_SUBSTEPS {
			let q = q0 + (driven - q0) * (k as f64) / (POSE_AT_SUBSTEPS as f64);
			let (solved, residual, singular) = self.newton(&poses, idx, q);
			if singular {
				return Err(MechanismError::Singular {
					mechanism: self.name.clone(),
					step: k,
					driven_value: q,
					hint: "the linkage is at a dead centre: the input can no longer determine which way the loop goes. Offset the \
					 command slightly, or re-declare the mechanism away from the change point."
						.to_string(),
				});
			}
			if residual > NEWTON_TOL {
				return Err(MechanismError::Convergence {
					mechanism: self.name.clone(),
					step: k,
					driven_value: q,
					residual,
					hint: "the loop cannot close at that command — most often the driven coordinate is outside the linkage's \
					 reachable range (a rocker driven past its limit), or the link lengths cannot form a closed chain."
						.to_string(),
				});
			}
			poses = solved;
		}
		Ok(poses)
	}

	/// **Sweep a full cycle** of the driven revolute: `cycle_steps` intervals
	/// from the declared coordinate through `+2π`, so `cycle_steps + 1` poses
	/// are recorded and the last repeats the first configuration (which is
	/// what makes the wrap-around continuity check meaningful).
	///
	/// Refuses a prismatic drive, which has no natural cycle — use
	/// [`Mechanism::sweep_range`].
	pub fn sweep(&self, cycle_steps: usize) -> Result<MotionReport, MechanismError> {
		self.validate()?;
		let idx = self.single_driven()?;
		if matches!(self.joints[idx].kind, JointKind::Prismatic { .. }) {
			return Err(MechanismError::NonCyclicDrive {
				mechanism: self.name.clone(),
				joint: self.joints[idx].name.clone(),
				hint: "a prismatic drive has a stroke, not a cycle — call sweep_range(from_mm, to_mm, steps) with the stroke you \
				 mean"
					.to_string(),
			});
		}
		let q0 = self.driven_coordinate()?;
		self.sweep_range(q0, q0 + std::f64::consts::TAU, cycle_steps)
	}

	/// Sweep the driven coordinate from `from` to `to` in `steps` intervals
	/// (`steps + 1` poses; `steps = 0` records the single pose at `from`).
	pub fn sweep_range(&self, from: f64, to: f64, steps: usize) -> Result<MotionReport, MechanismError> {
		self.gate_for_motion()?;
		let mobility = self.mobility();
		let idx = self.single_driven()?;
		let n_poses = steps + 1;

		let mut poses: Vec<Pose2> = self.links.iter().map(|l| l.initial).collect();
		let declared = poses.clone();
		let mut poses_per_step: Vec<Vec<Pose2>> = Vec::with_capacity(n_poses);
		let mut driven_value: Vec<f64> = Vec::with_capacity(n_poses);
		let mut max_newton_residual = 0.0f64;

		for step in 0..n_poses {
			let q = if steps == 0 { from } else { from + (to - from) * (step as f64) / (steps as f64) };
			let (solved, residual, singular) = self.newton(&poses, idx, q);
			if singular {
				return Err(MechanismError::Singular {
					mechanism: self.name.clone(),
					step,
					driven_value: q,
					hint: "the linkage is at a dead centre: the input can no longer determine which way the loop goes. Sweep a \
					 range that avoids the change point, or re-declare the mechanism on one branch."
						.to_string(),
				});
			}
			if residual > NEWTON_TOL {
				return Err(MechanismError::Convergence {
					mechanism: self.name.clone(),
					step,
					driven_value: q,
					residual,
					hint: "the loop cannot close at that command — most often the driven coordinate is outside the linkage's \
					 reachable range (a rocker driven past its limit), or the link lengths cannot form a closed chain."
						.to_string(),
				});
			}
			max_newton_residual = max_newton_residual.max(residual);
			poses = solved;
			driven_value.push(q);
			poses_per_step.push(poses.clone());
		}

		let initial_snap = poses_per_step[0]
			.iter()
			.zip(&declared)
			.fold(0.0f64, |m, (a, b)| m.max(((a.x - b.x).powi(2) + (a.y - b.y).powi(2)).sqrt()));

		// Ranges and traces.
		let mut range_of_motion = Vec::with_capacity(self.links.len());
		for (li, link) in self.links.iter().enumerate() {
			let mut tmin = f64::INFINITY;
			let mut tmax = f64::NEG_INFINITY;
			let mut ext = [f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY];
			for step in &poses_per_step {
				let p = step[li];
				tmin = tmin.min(p.theta);
				tmax = tmax.max(p.theta);
				ext[0] = ext[0].min(p.x);
				ext[1] = ext[1].max(p.x);
				ext[2] = ext[2].min(p.y);
				ext[3] = ext[3].max(p.y);
			}
			range_of_motion.push(LinkRange {
				name: link.name.clone(),
				theta_min: tmin,
				theta_max: tmax,
				theta_range: tmax - tmin,
				origin_extents: ext,
			});
		}
		let traces: Vec<TrackTrace> = self
			.tracked
			.iter()
			.map(|t| {
				let points: Vec<[f64; 2]> = poses_per_step
					.iter()
					.map(|step| {
						let w = step[t.link].apply(t.local);
						[w.x, w.y]
					})
					.collect();
				let mut ext = [f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY];
				for p in &points {
					ext[0] = ext[0].min(p[0]);
					ext[1] = ext[1].max(p[0]);
					ext[2] = ext[2].min(p[1]);
					ext[3] = ext[3].max(p[1]);
				}
				TrackTrace { name: t.name.clone(), link: t.link, points, extents: ext }
			})
			.collect();

		// Continuity.
		let mut max_t = 0.0f64;
		let mut max_r = 0.0f64;
		for w in poses_per_step.windows(2) {
			for (a, b) in w[0].iter().zip(w[1].iter()) {
				max_t = max_t.max(((b.x - a.x).powi(2) + (b.y - a.y).powi(2)).sqrt());
				max_r = max_r.max(shortest_arc(b.theta - a.theta).abs());
			}
		}

		// Interference, delegated to sweep_check.
		let pairs = self.pairs_to_check();
		let mut pair_sweeps = Vec::with_capacity(pairs.len());
		let mut min_clearance = f64::INFINITY;
		let mut first: Option<Interference> = None;
		for (i, j) in pairs {
			let (Some(mi), Some(mj)) = (self.links[i].mesh.as_ref(), self.links[j].mesh.as_ref()) else { continue };
			let rels: Vec<DAffine3> =
				poses_per_step.iter().map(|step| step[i].to_affine().inverse() * step[j].to_affine()).collect();
			let rep: SweepReport = sweep_check(mi, mj, &rels);
			if rep.min_clearance.is_finite() {
				min_clearance = min_clearance.min(rep.min_clearance);
			}
			for (step, sp) in rep.poses.iter().enumerate() {
				if sp.crossing || sp.penetration > 0.0 {
					let better = match &first {
						Some(f) => step < f.step,
						None => true,
					};
					if better {
						first = Some(Interference {
							step,
							driven_value: driven_value[step],
							pair: (i, j),
							pair_names: (self.links[i].name.clone(), self.links[j].name.clone()),
							penetration: sp.penetration,
							min_distance: sp.min_distance,
							crossing: sp.crossing,
						});
					}
					break;
				}
			}
			pair_sweeps.push(PairSweep {
				pair: (i, j),
				pair_names: (self.links[i].name.clone(), self.links[j].name.clone()),
				min_clearance: rep.min_clearance,
				max_penetration: rep.max_penetration,
				contacts: rep.contacts,
				crossings: rep.crossings,
			});
		}

		Ok(MotionReport {
			mechanism: self.name.clone(),
			steps: n_poses,
			mobility,
			driven_joint: self.joints[idx].name.clone(),
			driven_value,
			poses_per_step,
			range_of_motion,
			traces,
			min_clearance_over_cycle: min_clearance,
			first_interference: first,
			pair_sweeps,
			max_step_translation_jump: max_t,
			max_step_rotation_jump: max_r,
			max_newton_residual,
			initial_snap,
		})
	}

	// -- internals ----------------------------------------------------------

	/// The mobility gate every motion entry point passes through.
	fn gate_for_motion(&self) -> Result<(), MechanismError> {
		self.validate()?;
		let m = self.mobility();
		if m.kutzbach_dof <= 0 {
			return Err(MechanismError::Locked {
				mechanism: self.name.clone(),
				dof: m.kutzbach_dof,
				rank_dof: m.rank_dof,
				formula: m.formula.clone(),
				hint: if m.paradox {
					"the Jacobian rank disagrees with the formula: this is a Grubler paradox (a redundant constraint, e.g. a \
					 parallelogram linkage). Remove the redundant link/joint and the count will agree with the motion."
						.to_string()
				} else {
					"remove a joint or add a link: every joint you add removes 2 planar DOF, so a 1-DOF linkage needs \
					 3(n-1) - 2j = 1."
						.to_string()
				},
			});
		}
		let driven = self.driven_joints();
		if driven.is_empty() {
			return Err(MechanismError::NoDrivenJoint { mechanism: self.name.clone() });
		}
		if driven.len() as i64 > m.kutzbach_dof {
			return Err(MechanismError::Overdriven {
				mechanism: self.name.clone(),
				dof: m.kutzbach_dof,
				driven: driven.len(),
				hint: "drive exactly as many joints as the mechanism has mobility; extra commands over-determine the loop."
					.to_string(),
			});
		}
		if m.kutzbach_dof != 1 {
			return Err(MechanismError::Underdriven {
				mechanism: self.name.clone(),
				dof: m.kutzbach_dof,
				driven: driven.len(),
				hint: "this module sweeps mobility-1 linkages with exactly one driven joint; constrain the extra freedoms (add \
				 a joint) or drive them out of the model."
					.to_string(),
			});
		}
		Ok(())
	}

	fn single_driven(&self) -> Result<usize, MechanismError> {
		let driven = self.driven_joints();
		match driven.len() {
			0 => Err(MechanismError::NoDrivenJoint { mechanism: self.name.clone() }),
			1 => Ok(driven[0]),
			n => Err(MechanismError::Overdriven {
				mechanism: self.name.clone(),
				dof: self.mobility().kutzbach_dof,
				driven: n,
				hint: "mark exactly one joint driven".to_string(),
			}),
		}
	}

	/// The pairs interference is checked on (see [`Mechanism::check_pairs`]).
	fn pairs_to_check(&self) -> Vec<(usize, usize)> {
		if let Some(p) = &self.check_pairs {
			return p.clone();
		}
		let joined = |i: usize, j: usize| self.joints.iter().any(|jt| (jt.a == i && jt.b == j) || (jt.a == j && jt.b == i));
		let mut out = Vec::new();
		for i in 0..self.links.len() {
			for j in (i + 1)..self.links.len() {
				if self.links[i].mesh.is_some() && self.links[j].mesh.is_some() && !joined(i, j) {
					out.push((i, j));
				}
			}
		}
		out
	}

	/// The driven joint's coordinate at the given poses.
	fn drive_coordinate(&self, joint: &Joint, poses: &[Pose2]) -> f64 {
		let (pa, pb) = (poses[joint.a], poses[joint.b]);
		match joint.kind {
			JointKind::Revolute { .. } => pb.theta - pa.theta,
			JointKind::Prismatic { axis_in_a, a_point, b_point } => {
				let u = rot(pa.theta, axis_in_a.normalize());
				(pb.apply(b_point) - pa.apply(a_point)).dot(u)
			}
		}
	}

	/// Damped Newton on the loop-closure system. Returns
	/// `(poses, residual_inf_norm, singular)`.
	fn newton(&self, warm: &[Pose2], driven_idx: usize, target: f64) -> (Vec<Pose2>, f64, bool) {
		let moving = self.links.len() - 1;
		let cols = 3 * moving;
		let rows: usize = self.joints.iter().map(|j| j.kind.constraints()).sum::<usize>() + 1;
		let mut poses = warm.to_vec();
		poses[0] = self.links[0].initial; // ground stays put

		let mut residual = self.residual_norm(&poses, driven_idx, target);
		for _ in 0..NEWTON_MAX_ITERS {
			if residual <= NEWTON_TOL {
				return (poses, residual, false);
			}
			let mut jac = vec![vec![0.0f64; cols]; rows];
			let mut res = vec![0.0f64; rows];
			self.assemble(&poses, driven_idx, target, &mut jac, &mut res);
			for v in res.iter_mut() {
				*v = -*v;
			}
			let Some(delta) = solve_square(&mut jac, &mut res) else {
				return (poses, residual, true);
			};
			let mut alpha = 1.0f64;
			let mut accepted = false;
			for _ in 0..NEWTON_MAX_BACKTRACKS {
				let mut trial = poses.clone();
				for k in 0..moving {
					trial[k + 1].x += alpha * delta[3 * k];
					trial[k + 1].y += alpha * delta[3 * k + 1];
					trial[k + 1].theta += alpha * delta[3 * k + 2];
				}
				let r = self.residual_norm(&trial, driven_idx, target);
				if r < residual || r <= NEWTON_TOL {
					poses = trial;
					residual = r;
					accepted = true;
					break;
				}
				alpha *= 0.5;
			}
			if !accepted {
				return (poses, residual, false);
			}
		}
		(poses, residual, false)
	}

	fn residual_norm(&self, poses: &[Pose2], driven_idx: usize, target: f64) -> f64 {
		let rows: usize = self.joints.iter().map(|j| j.kind.constraints()).sum::<usize>() + 1;
		let cols = 3 * (self.links.len() - 1);
		let mut jac = vec![vec![0.0f64; cols]; rows];
		let mut res = vec![0.0f64; rows];
		self.assemble(poses, driven_idx, target, &mut jac, &mut res);
		res.iter().fold(0.0f64, |m, v| m.max(v.abs()))
	}

	/// Fill the joint rows and the driving row of the Newton system.
	fn assemble(&self, poses: &[Pose2], driven_idx: usize, target: f64, jac: &mut [Vec<f64>], res: &mut [f64]) {
		let mut row = 0usize;
		for j in &self.joints {
			self.joint_rows(j, poses, jac, res, row);
			row += j.kind.constraints();
		}
		// The driving row.
		let j = &self.joints[driven_idx];
		let (pa, pb) = (poses[j.a], poses[j.b]);
		match j.kind {
			JointKind::Revolute { .. } => {
				res[row] = (pb.theta - pa.theta) - target;
				set(jac, row, j.a, 2, -1.0, self.links.len());
				set(jac, row, j.b, 2, 1.0, self.links.len());
			}
			JointKind::Prismatic { axis_in_a, a_point, b_point } => {
				let ul = axis_in_a.normalize();
				let u = rot(pa.theta, ul);
				let wa = pa.apply(a_point);
				let wb = pb.apply(b_point);
				let d = wb - wa;
				res[row] = d.dot(u) - target;
				let dwa = drot(pa.theta, a_point);
				let dwb = drot(pb.theta, b_point);
				let du = drot(pa.theta, ul);
				set(jac, row, j.a, 0, -u.x, self.links.len());
				set(jac, row, j.a, 1, -u.y, self.links.len());
				set(jac, row, j.a, 2, -dwa.dot(u) + d.dot(du), self.links.len());
				set(jac, row, j.b, 0, u.x, self.links.len());
				set(jac, row, j.b, 1, u.y, self.links.len());
				set(jac, row, j.b, 2, dwb.dot(u), self.links.len());
			}
		}
	}

	/// Fill one joint's constraint rows (and their Jacobian) starting at
	/// `row`.
	fn joint_rows(&self, j: &Joint, poses: &[Pose2], jac: &mut [Vec<f64>], res: &mut [f64], row: usize) {
		let n = self.links.len();
		if j.a >= poses.len() || j.b >= poses.len() {
			return; // out-of-range joints are refused by `validate`; never index past the poses
		}
		let (pa, pb) = (poses[j.a], poses[j.b]);
		match j.kind {
			JointKind::Revolute { a_point, b_point } => {
				let wa = pa.apply(a_point);
				let wb = pb.apply(b_point);
				let d = wa - wb;
				res[row] = d.x;
				res[row + 1] = d.y;
				let da = drot(pa.theta, a_point);
				let db = drot(pb.theta, b_point);
				set(jac, row, j.a, 0, 1.0, n);
				set(jac, row + 1, j.a, 1, 1.0, n);
				set(jac, row, j.a, 2, da.x, n);
				set(jac, row + 1, j.a, 2, da.y, n);
				set(jac, row, j.b, 0, -1.0, n);
				set(jac, row + 1, j.b, 1, -1.0, n);
				set(jac, row, j.b, 2, -db.x, n);
				set(jac, row + 1, j.b, 2, -db.y, n);
			}
			JointKind::Prismatic { axis_in_a, a_point, b_point } => {
				let ul = axis_in_a.normalize();
				let nl = DVec2::new(-ul.y, ul.x);
				let nw = rot(pa.theta, nl);
				let wa = pa.apply(a_point);
				let wb = pb.apply(b_point);
				let d = wb - wa;
				// The relative angle is frozen from the DECLARED poses: a
				// prismatic pair holds whatever relative orientation the
				// mechanism was declared in.
				let rel0 = self.links[j.b].initial.theta - self.links[j.a].initial.theta;
				res[row] = d.dot(nw);
				res[row + 1] = (pb.theta - pa.theta) - rel0;
				let dwa = drot(pa.theta, a_point);
				let dwb = drot(pb.theta, b_point);
				let dn = drot(pa.theta, nl);
				set(jac, row, j.a, 0, -nw.x, n);
				set(jac, row, j.a, 1, -nw.y, n);
				set(jac, row, j.a, 2, -dwa.dot(nw) + d.dot(dn), n);
				set(jac, row, j.b, 0, nw.x, n);
				set(jac, row, j.b, 1, nw.y, n);
				set(jac, row, j.b, 2, dwb.dot(nw), n);
				set(jac, row + 1, j.a, 2, -1.0, n);
				set(jac, row + 1, j.b, 2, 1.0, n);
			}
		}
	}
}

/// Write `value` into the Jacobian column belonging to `link`'s coordinate
/// `k` (0 = x, 1 = y, 2 = θ). Ground (link 0) has no columns, so its writes
/// are dropped.
fn set(jac: &mut [Vec<f64>], row: usize, link: usize, k: usize, value: f64, links: usize) {
	if link == 0 || link >= links {
		return;
	}
	jac[row][3 * (link - 1) + k] += value;
}

/// Rotate a planar vector by `theta`.
fn rot(theta: f64, v: DVec2) -> DVec2 {
	let (s, c) = theta.sin_cos();
	DVec2::new(c * v.x - s * v.y, s * v.x + c * v.y)
}

/// d/dθ of [`rot`] — the derivative of a rotated local point.
fn drot(theta: f64, v: DVec2) -> DVec2 {
	let (s, c) = theta.sin_cos();
	DVec2::new(-s * v.x - c * v.y, c * v.x - s * v.y)
}

/// Wrap an angle difference into `(−π, π]`.
fn shortest_arc(mut d: f64) -> f64 {
	while d > std::f64::consts::PI {
		d -= std::f64::consts::TAU;
	}
	while d <= -std::f64::consts::PI {
		d += std::f64::consts::TAU;
	}
	d
}

/// Solve a square system by Gaussian elimination with partial pivoting.
/// `None` when the matrix is singular to [`PIVOT_EPS`] relative tolerance.
fn solve_square(a: &mut [Vec<f64>], b: &mut [f64]) -> Option<Vec<f64>> {
	let n = a.len();
	if n == 0 || a.iter().any(|r| r.len() != n) || b.len() != n {
		return None;
	}
	let scale = a.iter().flat_map(|r| r.iter()).fold(0.0f64, |m, v| m.max(v.abs())).max(1e-300);
	let eps = PIVOT_EPS * scale;
	for col in 0..n {
		let (best, mag) = (col..n).fold((col, 0.0f64), |(bi, bm), r| if a[r][col].abs() > bm { (r, a[r][col].abs()) } else { (bi, bm) });
		if mag <= eps {
			return None;
		}
		a.swap(col, best);
		b.swap(col, best);
		let pivot = a[col][col];
		for r in (col + 1)..n {
			let factor = a[r][col] / pivot;
			if factor == 0.0 {
				continue;
			}
			let (top, bottom) = a.split_at_mut(r);
			for (target, p) in bottom[0].iter_mut().zip(top[col].iter()).skip(col) {
				*target -= factor * p;
			}
			b[r] -= factor * b[col];
		}
	}
	let mut x = vec![0.0f64; n];
	for col in (0..n).rev() {
		let mut acc = b[col];
		for c in (col + 1)..n {
			acc -= a[col][c] * x[c];
		}
		x[col] = acc / a[col][col];
	}
	Some(x)
}

/// Numeric rank by row reduction with partial pivoting (relative tolerance).
fn matrix_rank(m: &mut [Vec<f64>]) -> usize {
	let rows = m.len();
	if rows == 0 {
		return 0;
	}
	let cols = m[0].len();
	let scale = m.iter().flat_map(|r| r.iter()).fold(0.0f64, |a, v| a.max(v.abs())).max(1e-300);
	let eps = 1e-9 * scale;
	let mut rank = 0usize;
	for col in 0..cols {
		if rank >= rows {
			break;
		}
		let (best, mag) = (rank..rows).fold((rank, 0.0f64), |(bi, bm), r| if m[r][col].abs() > bm { (r, m[r][col].abs()) } else { (bi, bm) });
		if mag <= eps {
			continue;
		}
		m.swap(rank, best);
		let pivot = m[rank][col];
		for r in (rank + 1)..rows {
			let factor = m[r][col] / pivot;
			if factor == 0.0 {
				continue;
			}
			let (top, bottom) = m.split_at_mut(r);
			for (target, p) in bottom[0].iter_mut().zip(top[rank].iter()).skip(col) {
				*target -= factor * p;
			}
		}
		rank += 1;
	}
	rank
}
