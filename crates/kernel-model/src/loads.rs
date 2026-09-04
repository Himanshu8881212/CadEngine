// Copyright (c) LMCAD. Licensed under the MIT License.

//! **Load paths through an assembly** — external loads propagated through the
//! mate graph to per-part reactions, so a part is analysed under what the
//! ASSEMBLY actually does to it instead of under a load somebody hand-assigned
//! to it.
//!
//! Declare the parts, the [`Connection`]s that join them (and join them to
//! ground), and the [`AppliedLoad`]s acting on them; [`LoadCase::solve`]
//! returns a [`LoadPath`] whose [`PartReaction`]s carry, per part, the wrench
//! every one of its mates transmits. Those wrenches are the boundary
//! conditions that part's FEA needs — [`LoadPath::fea_manifest`] emits them in
//! the shape `tools/ace_fea_runner.py` consumes.
//!
//! # Method and scope — read this before trusting a number
//!
//! **Rigid-body static equilibrium.** Every part is rigid; every connection
//! transmits a wrench whose direction basis is fixed by the joint kind
//! ([`JointKind`]). Six scalar equations per part (ΣF = 0, ΣM = 0 about the
//! world origin) are assembled into one linear system `A x = −f_ext` over the
//! connection unknowns, and it is solved exactly by Gaussian elimination with
//! partial pivoting. Three outcomes, each of them explicit:
//!
//! | outcome | test | result |
//! |---|---|---|
//! | **determinate** | rank(A) = unknowns, system consistent | unique reactions |
//! | **indeterminate** | rank(A) < unknowns, system consistent | [`LoadError::Indeterminate`] with the redundancy count |
//! | **not equilibrable** | system inconsistent | [`LoadError::NoEquilibrium`] with the unbalanced resultant |
//!
//! A statically indeterminate structure has *infinitely many* rigid-body
//! solutions; picking one requires stiffness, which a rigid-body model does
//! not have. Returning one anyway would be a dangerous, plausible-looking lie,
//! so this module **refuses** and names the redundancy count. An unsupported
//! (floating) assembly under a real load has *no* solution, and refuses too —
//! it never returns zeros.
//!
//! **What a future flexible solve would need**, stated so the seam is honest:
//! a stiffness for each connection (6×6 wrench-vs-relative-displacement, or a
//! member stiffness `EA/L`, `EI` derived from the part geometry the assembly
//! already owns) and a compatibility equation per redundant unknown — i.e.
//! `K u = f` on the assembly graph, with the reaction recovered as `k·u`.
//! That converts the [`LoadError::Indeterminate`] refusal into an answer whose
//! honesty then depends on the stiffness model. Nothing here fakes it.
//!
//! **Other limits.** Static only — no inertia, no dynamics, no friction, no
//! preload, no gravity unless the caller applies it as loads. Small
//! displacement: the geometry is the as-declared one and does not move under
//! load. Unilateral [`JointKind::Contact`] joints are solved as **bilateral**
//! (a solved contact that would have to *pull* is reported by
//! [`ConnectionReaction::tension_on_unilateral`] and by
//! [`LoadPath::gate_unilateral`], not silently accepted). And a load case can
//! be perfectly determinate while the assembly is a **mechanism** — statics
//! alone cannot see mobility; use [`crate::mechanism`] for that.

use std::collections::VecDeque;

use serde::{Deserialize, Serialize};

use kernel_core::math::DVec3;

/// Relative pivot tolerance for the rank / elimination decisions, scaled by
/// the largest matrix entry.
const PIVOT_EPS: f64 = 1e-9;

/// Residual above which a row with an all-zero coefficient block is judged
/// inconsistent (scaled the same way).
const CONSISTENCY_EPS: f64 = 1e-7;

/// One end of a [`Connection`]: a part, or the (immovable) ground.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Attach {
	/// The world / fixture / bench — absorbs any wrench, writes no equations.
	Ground,
	/// A part by index into [`LoadCase::parts`].
	Part(usize),
}

impl Attach {
	/// The part index, if this end is a part.
	pub fn part(self) -> Option<usize> {
		match self {
			Attach::Ground => None,
			Attach::Part(i) => Some(i),
		}
	}
}

/// What a connection can transmit. Each variant states its unknown count —
/// that count is exactly the number of scalar reactions the solver introduces.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum JointKind {
	/// Bonded / bolted / press-fit / clamped: all 3 forces and all 3 moments.
	/// **6 unknowns.**
	Rigid,
	/// Ball joint, pinned support, or a short pin free to rotate any way: 3
	/// forces, no moment. **3 unknowns.**
	Spherical,
	/// Revolute (hinge, bearing) about `axis`: 3 forces plus the 2 moments
	/// perpendicular to `axis` — the axial moment is what the joint is free
	/// in. **5 unknowns.**
	Revolute {
		/// World-space axis (need not be normalized; zero is refused).
		axis: DVec3,
	},
	/// Prismatic (slide, rail) along `axis`: the 2 forces perpendicular to
	/// `axis` plus all 3 moments — the axial force is what the joint is free
	/// in. **5 unknowns.**
	Prismatic {
		/// World-space axis (need not be normalized; zero is refused).
		axis: DVec3,
	},
	/// A single-direction contact / roller along `normal`: 1 force. **1
	/// unknown.** Solved bilaterally; a negative solution means the contact
	/// would have to pull and is flagged, not hidden.
	Contact {
		/// World-space contact normal (need not be normalized).
		normal: DVec3,
	},
}

impl JointKind {
	/// Stable lowercase name for receipts.
	pub fn name(&self) -> &'static str {
		match self {
			JointKind::Rigid => "rigid",
			JointKind::Spherical => "spherical",
			JointKind::Revolute { .. } => "revolute",
			JointKind::Prismatic { .. } => "prismatic",
			JointKind::Contact { .. } => "contact",
		}
	}

	/// Number of scalar unknowns this joint contributes.
	pub fn unknowns(&self) -> usize {
		let (f, m) = self.basis_counts();
		f + m
	}

	fn basis_counts(&self) -> (usize, usize) {
		match self {
			JointKind::Rigid => (3, 3),
			JointKind::Spherical => (3, 0),
			JointKind::Revolute { .. } => (3, 2),
			JointKind::Prismatic { .. } => (2, 3),
			JointKind::Contact { .. } => (1, 0),
		}
	}

	/// The world-space `(force_directions, moment_directions)` this joint
	/// transmits. `None` when a declared axis/normal is degenerate.
	pub fn basis(&self) -> Option<(Vec<DVec3>, Vec<DVec3>)> {
		let xyz = || vec![DVec3::X, DVec3::Y, DVec3::Z];
		match *self {
			JointKind::Rigid => Some((xyz(), xyz())),
			JointKind::Spherical => Some((xyz(), Vec::new())),
			JointKind::Revolute { axis } => {
				let (_, u, v) = frame(axis)?;
				Some((xyz(), vec![u, v]))
			}
			JointKind::Prismatic { axis } => {
				let (_, u, v) = frame(axis)?;
				Some((vec![u, v], xyz()))
			}
			JointKind::Contact { normal } => {
				let n = normalize(normal)?;
				Some((vec![n], Vec::new()))
			}
		}
	}
}

/// Normalize, or `None` if the vector is degenerate / non-finite.
fn normalize(v: DVec3) -> Option<DVec3> {
	if !v.is_finite() || v.length() < 1e-12 {
		return None;
	}
	Some(v.normalize())
}

/// An orthonormal frame `(a, u, v)` with `a` the normalized input axis.
fn frame(axis: DVec3) -> Option<(DVec3, DVec3, DVec3)> {
	let a = normalize(axis)?;
	let seed = if a.x.abs() < 0.9 { DVec3::X } else { DVec3::Y };
	let u = a.cross(seed).normalize();
	let v = a.cross(u);
	Some((a, u, v))
}

/// A part of the assembly that carries load.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Part {
	/// Name — appears in every reaction and refusal.
	pub name: String,
	/// The [`crate::Assembly`] instance index this part is, when the load case
	/// was built alongside an assembly.
	pub instance: Option<usize>,
}

/// A joint between two parts, or between a part and ground (a support).
///
/// **Sign convention:** the solved wrench is what end `b` applies **to** end
/// `a`, at `point`, in world space. End `a` therefore receives `+`, end `b`
/// receives `−`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Connection {
	/// Name — appears in every reaction and refusal.
	pub name: String,
	/// The end that receives the `+` wrench.
	pub a: Attach,
	/// The end that receives the `−` wrench.
	pub b: Attach,
	/// World-space point the wrench acts at.
	pub point: DVec3,
	/// What the joint can transmit.
	pub kind: JointKind,
}

impl Connection {
	/// A joint between two parts.
	pub fn joint(name: impl Into<String>, a: usize, b: usize, point: DVec3, kind: JointKind) -> Self {
		Self { name: name.into(), a: Attach::Part(a), b: Attach::Part(b), point, kind }
	}

	/// A **support**: a joint from a part to ground. The solved wrench is what
	/// ground applies to the part.
	pub fn support(name: impl Into<String>, part: usize, point: DVec3, kind: JointKind) -> Self {
		Self { name: name.into(), a: Attach::Part(part), b: Attach::Ground, point, kind }
	}

	/// Whether this connection touches ground (i.e. is a support).
	pub fn is_support(&self) -> bool {
		self.a == Attach::Ground || self.b == Attach::Ground
	}
}

/// An external load applied to one part.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AppliedLoad {
	/// Name — appears in the FEA manifest.
	pub name: String,
	/// Index into [`LoadCase::parts`].
	pub part: usize,
	/// World-space point of application (used for the `r × F` moment).
	pub point: DVec3,
	/// Force (N).
	pub force: DVec3,
	/// Pure moment / couple (N·mm) — applied in addition to `r × force`.
	pub moment: DVec3,
}

impl AppliedLoad {
	/// A pure force at a point.
	pub fn force(name: impl Into<String>, part: usize, point: DVec3, force: DVec3) -> Self {
		Self { name: name.into(), part, point, force, moment: DVec3::ZERO }
	}

	/// A pure couple on a part (no resultant force, no point dependence).
	pub fn couple(name: impl Into<String>, part: usize, moment: DVec3) -> Self {
		Self { name: name.into(), part, point: DVec3::ZERO, force: DVec3::ZERO, moment }
	}
}

/// A complete static load case: parts, the connections between them, the
/// external loads, and the supports (connections to [`Attach::Ground`]).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct LoadCase {
	/// Case name — appears in the receipt.
	pub name: String,
	/// The load-carrying parts.
	pub parts: Vec<Part>,
	/// Joints and supports. A support is a [`Connection`] with one end
	/// [`Attach::Ground`].
	pub connections: Vec<Connection>,
	/// External loads.
	pub loads: Vec<AppliedLoad>,
}

/// Typed refusals from [`LoadCase::solve`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum LoadError {
	/// No parts to write equilibrium equations for.
	Empty {
		/// Case name.
		case: String,
	},
	/// A load / connection references a part index that does not exist.
	UnknownPart {
		/// What referenced it.
		context: String,
		/// The offending index.
		index: usize,
		/// How many parts exist.
		parts: usize,
	},
	/// A vector input is not finite.
	NonFinite {
		/// What carried it.
		context: String,
		/// Which field.
		field: &'static str,
	},
	/// A joint axis / contact normal is (nearly) zero, so its direction basis
	/// cannot be built.
	DegenerateAxis {
		/// Connection name.
		connection: String,
		/// The joint kind that needed the axis.
		kind: &'static str,
	},
	/// A connection joins something to itself, transmitting nothing.
	SelfConnection {
		/// Connection name.
		connection: String,
		/// The end it names twice (`None` = ground to ground).
		part: Option<usize>,
	},
	/// The structure is **statically indeterminate**: more unknown reaction
	/// components than independent equilibrium equations. A rigid-body model
	/// cannot choose among the infinitely many solutions.
	Indeterminate {
		/// Case name.
		case: String,
		/// `unknowns − independent_equations`: how many reaction components
		/// are redundant.
		redundancy: usize,
		/// Total scalar reaction unknowns.
		unknowns: usize,
		/// Rank of the equilibrium system.
		independent_equations: usize,
		/// What to do about it.
		hint: String,
	},
	/// The declared supports **cannot** equilibrate the declared loads: the
	/// system has no solution at all (the classic case is a floating assembly
	/// — no support, or supports that cannot resist the applied resultant).
	NoEquilibrium {
		/// Case name.
		case: String,
		/// Resultant external force on the whole assembly (N) — must be zero
		/// for an unsupported assembly to be in equilibrium.
		net_force: [f64; 3],
		/// Resultant external moment about the world origin (N·mm).
		net_moment: [f64; 3],
		/// Number of connections that touch ground.
		supports: usize,
		/// Worst unbalanced row residual found by the elimination.
		worst_residual: f64,
		/// What to do about it.
		hint: String,
	},
}

impl std::fmt::Display for LoadError {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		match self {
			LoadError::Empty { case } => write!(f, "loads: case '{case}' has no parts — there is nothing to write equilibrium for"),
			LoadError::UnknownPart { context, index, parts } => {
				write!(f, "loads: {context} references part {index} but the case has {parts} part(s)")
			}
			LoadError::NonFinite { context, field } => write!(f, "loads: {context} has a non-finite {field}"),
			LoadError::DegenerateAxis { connection, kind } => write!(
				f,
				"loads: connection '{connection}' ({kind}) has a zero-length axis/normal — a {kind} joint's reaction basis is \
				 defined by that direction and cannot be built without it"
			),
			LoadError::SelfConnection { connection, part } => match part {
				Some(p) => write!(f, "loads: connection '{connection}' joins part {p} to itself — it transmits nothing"),
				None => write!(f, "loads: connection '{connection}' joins ground to ground — it transmits nothing"),
			},
			LoadError::Indeterminate { case, redundancy, unknowns, independent_equations, hint } => write!(
				f,
				"loads: case '{case}' is STATICALLY INDETERMINATE — {unknowns} reaction unknown(s) against \
				 {independent_equations} independent equilibrium equation(s), redundancy {redundancy}. A rigid-body model has \
				 infinitely many solutions here and picking one requires stiffness, so no reactions are returned. {hint}"
			),
			LoadError::NoEquilibrium { case, net_force, net_moment, supports, worst_residual, hint } => write!(
				f,
				"loads: case '{case}' CANNOT BE EQUILIBRATED by its {supports} ground connection(s) — the equations are \
				 inconsistent (worst unbalanced row {worst_residual:.6e}). Net external force [{:.4}, {:.4}, {:.4}] N, net \
				 external moment about the origin [{:.4}, {:.4}, {:.4}] N·mm. {hint}",
				net_force[0], net_force[1], net_force[2], net_moment[0], net_moment[1], net_moment[2]
			),
		}
	}
}

impl std::error::Error for LoadError {}

/// The wrench one connection transmits.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ConnectionReaction {
	/// Index into [`LoadCase::connections`].
	pub index: usize,
	/// Connection name.
	pub name: String,
	/// Joint kind name.
	pub kind: String,
	/// The `+` end.
	pub a: Attach,
	/// The `−` end.
	pub b: Attach,
	/// Where the wrench acts (world).
	pub point: DVec3,
	/// Force `b` applies to `a` (N).
	pub force: DVec3,
	/// Moment `b` applies to `a` about `point` (N·mm).
	pub moment: DVec3,
	/// Set when a unilateral [`JointKind::Contact`] solved negative, i.e. the
	/// contact would have to PULL. The number is still reported — the model,
	/// not the arithmetic, is what is wrong.
	pub tension_on_unilateral: bool,
}

/// What one mate does to one part.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MateReaction {
	/// Index into [`LoadCase::connections`].
	pub connection: usize,
	/// Connection name.
	pub name: String,
	/// The thing on the other side.
	pub other: Attach,
	/// Where it acts (world).
	pub point: DVec3,
	/// Force the other side applies to THIS part (N).
	pub force: DVec3,
	/// Moment the other side applies to THIS part about `point` (N·mm).
	pub moment: DVec3,
}

/// Everything the assembly does to one part — the boundary condition set for
/// that part's own analysis.
///
/// Identity worth knowing: a part in equilibrium has
/// `reaction_force + external_force = 0` and likewise for moments, so
/// [`Self::residual_force`] / [`Self::residual_moment`] are the *proof* the
/// solve worked, and [`Self::via_mates`] — not the net — is the payload.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PartReaction {
	/// Index into [`LoadCase::parts`].
	pub part: usize,
	/// Part name.
	pub name: String,
	/// The [`crate::Assembly`] instance index, when declared.
	pub instance: Option<usize>,
	/// Resultant of the external loads on this part (N).
	pub external_force: DVec3,
	/// Resultant external moment about the world origin, `Σ(r × F) + ΣM`
	/// (N·mm).
	pub external_moment: DVec3,
	/// Resultant of every mate reaction on this part (N).
	pub reaction_force: DVec3,
	/// Resultant mate moment about the world origin (N·mm).
	pub reaction_moment: DVec3,
	/// Per-mate detail — the FEA boundary conditions.
	pub via_mates: Vec<MateReaction>,
	/// `external_force + reaction_force`; ~0 for a solved part.
	pub residual_force: DVec3,
	/// `external_moment + reaction_moment`; ~0 for a solved part.
	pub residual_moment: DVec3,
}

/// The solved load path: every connection's wrench and every part's boundary
/// conditions.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct LoadPath {
	/// Case name.
	pub case: String,
	/// Per part, in [`LoadCase::parts`] order.
	pub per_part: Vec<PartReaction>,
	/// Per connection, in [`LoadCase::connections`] order.
	pub connections: Vec<ConnectionReaction>,
	/// Total scalar reaction unknowns solved.
	pub unknowns: usize,
	/// Rank of the equilibrium system (equal to `unknowns` for a determinate
	/// case — that is what makes it determinate).
	pub independent_equations: usize,
	/// Largest per-part force residual `|ΣF|` (N) — the equilibrium proof.
	pub max_residual_force: f64,
	/// Largest per-part moment residual `|ΣM|` (N·mm) about the world origin.
	pub max_residual_moment: f64,
	/// Global `|Σ F_ext + Σ F_support|` (N) — must be ~0.
	pub global_residual_force: f64,
	/// Global `|Σ M_ext + Σ M_support|` (N·mm) about the world origin.
	pub global_residual_moment: f64,
}

impl LoadPath {
	/// The reaction on one part by name.
	pub fn part(&self, name: &str) -> Option<&PartReaction> {
		self.per_part.iter().find(|p| p.name == name)
	}

	/// The wrench on one connection by name.
	pub fn connection(&self, name: &str) -> Option<&ConnectionReaction> {
		self.connections.iter().find(|c| c.name == name)
	}

	/// Refuse a solution in which a unilateral [`JointKind::Contact`] came out
	/// in tension — the joint would have to pull, so the contact set assumed
	/// by the model is wrong (a different contact lifts off, or a real
	/// fastener is missing). Returns the offending connection names.
	pub fn gate_unilateral(&self) -> Result<(), Vec<String>> {
		let bad: Vec<String> = self
			.connections
			.iter()
			.filter(|c| c.tension_on_unilateral)
			.map(|c| format!("{} ({:.4} N)", c.name, c.force.length()))
			.collect();
		if bad.is_empty() {
			Ok(())
		} else {
			Err(bad)
		}
	}

	/// Per-part **FEA boundary-condition manifest**, shaped for
	/// `tools/ace_fea_runner.py`.
	///
	/// Convention, stated because it is a modelling choice and not a fact:
	/// each part is **clamped at its ground-ward interface** (the connection
	/// whose other end is nearest ground in the connection graph — ground
	/// itself is nearest) and **loaded at every other interface plus its own
	/// external loads**. That removes the part's rigid-body modes the way a
	/// real test fixture would; inertia relief is not implemented.
	///
	/// Every reaction *moment* that a point-load FEA job cannot express is
	/// carried in [`FeaPartJob::unrepresented_moments`] instead of being
	/// dropped: the ACE runner takes point / body / pressure loads only, and a
	/// silently discarded couple is exactly the kind of missing term that
	/// makes an FEA look conservative when it is not.
	///
	/// `selector_half_mm` is the half-size of the bbox selector emitted around
	/// each application point (the ACE selector vocabulary is
	/// `plane` / `bbox` / `all`).
	pub fn fea_manifest(&self, case: &LoadCase, selector_half_mm: f64) -> Vec<FeaPartJob> {
		let depth = ground_depth(case);
		let mut jobs = Vec::with_capacity(self.per_part.len());
		for pr in &self.per_part {
			// The ground-ward mate becomes the fixture; the rest become loads.
			let fixture_mate = pr
				.via_mates
				.iter()
				.enumerate()
				.min_by_key(|(_, m)| match m.other {
					Attach::Ground => 0usize,
					Attach::Part(i) => depth.get(i).copied().unwrap_or(usize::MAX).saturating_add(1),
				})
				.map(|(i, _)| i);
			let mut loads = Vec::new();
			let mut fixtures = Vec::new();
			let mut unrepresented = Vec::new();
			let mut notes = Vec::new();
			for (i, m) in pr.via_mates.iter().enumerate() {
				if Some(i) == fixture_mate {
					fixtures.push(FeaFixture {
						kind: "clamped".to_string(),
						source: m.name.clone(),
						point: [m.point.x, m.point.y, m.point.z],
						half_mm: selector_half_mm,
					});
					continue;
				}
				if m.force.length() > 0.0 {
					loads.push(FeaPointLoad {
						kind: "point".to_string(),
						source: m.name.clone(),
						magnitude_n: m.force.length(),
						direction: unit_or_zero(m.force),
						point: [m.point.x, m.point.y, m.point.z],
						half_mm: selector_half_mm,
					});
				}
				if m.moment.length() > 0.0 {
					unrepresented.push(FeaMoment {
						source: m.name.clone(),
						moment_n_mm: [m.moment.x, m.moment.y, m.moment.z],
						point: [m.point.x, m.point.y, m.point.z],
					});
				}
			}
			for l in case.loads.iter().filter(|l| l.part == pr.part) {
				if l.force.length() > 0.0 {
					loads.push(FeaPointLoad {
						kind: "point".to_string(),
						source: l.name.clone(),
						magnitude_n: l.force.length(),
						direction: unit_or_zero(l.force),
						point: [l.point.x, l.point.y, l.point.z],
						half_mm: selector_half_mm,
					});
				}
				if l.moment.length() > 0.0 {
					unrepresented.push(FeaMoment {
						source: l.name.clone(),
						moment_n_mm: [l.moment.x, l.moment.y, l.moment.z],
						point: [l.point.x, l.point.y, l.point.z],
					});
				}
			}
			if fixtures.is_empty() {
				notes.push(
					"no mate available to clamp: this part is held only by its external loads, so the FEA job as emitted has \
					 rigid-body modes and will not solve — add a support or model the part with inertia relief"
						.to_string(),
				);
			}
			if !unrepresented.is_empty() {
				notes.push(format!(
					"{} reaction moment(s) are NOT representable as ACE point loads and are carried in \
					 `unrepresented_moments`: convert each to a force couple over a real face before running, or the FEA is \
					 missing that term",
					unrepresented.len()
				));
			}
			jobs.push(FeaPartJob {
				part: pr.name.clone(),
				instance: pr.instance,
				loads,
				fixtures,
				unrepresented_moments: unrepresented,
				notes,
			});
		}
		jobs
	}

	/// [`LoadPath::fea_manifest`] as a JSON string (pretty-printed, stable
	/// field order) ready for a campaign to write next to its FEA jobs.
	pub fn fea_manifest_json(&self, case: &LoadCase, selector_half_mm: f64) -> String {
		let jobs = self.fea_manifest(case, selector_half_mm);
		let payload = serde_json::json!({
			"schema": "lmcad.load_path.fea.v1",
			"case": self.case,
			"convention": "each part is clamped at its ground-ward mate and loaded at the others; reaction moments that a point-load \
			 FEA cannot express are listed separately, never dropped",
			"units": {"force": "N", "moment": "N*mm", "length": "mm"},
			"max_residual_force_n": self.max_residual_force,
			"max_residual_moment_n_mm": self.max_residual_moment,
			"parts": jobs,
		});
		serde_json::to_string_pretty(&payload).unwrap_or_else(|e| format!("{{\"error\":\"{e}\"}}"))
	}
}

/// A point load in an ACE FEA job.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FeaPointLoad {
	/// Always `"point"` — the ACE load kind.
	pub kind: String,
	/// Which mate or external load this came from.
	pub source: String,
	/// Magnitude (N).
	pub magnitude_n: f64,
	/// Unit direction.
	pub direction: [f64; 3],
	/// Application point (mm, world).
	pub point: [f64; 3],
	/// Half-size of the bbox selector to build around `point` (mm).
	pub half_mm: f64,
}

/// A fixture in an ACE FEA job.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FeaFixture {
	/// Always `"clamped"` — the ACE fixture kind.
	pub kind: String,
	/// Which mate this came from.
	pub source: String,
	/// Interface point (mm, world).
	pub point: [f64; 3],
	/// Half-size of the bbox selector to build around `point` (mm).
	pub half_mm: f64,
}

/// A reaction moment a point-load FEA job cannot express — carried, never
/// dropped.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FeaMoment {
	/// Which mate or external load this came from.
	pub source: String,
	/// The moment (N·mm).
	pub moment_n_mm: [f64; 3],
	/// Where it acts (mm, world).
	pub point: [f64; 3],
}

/// One part's FEA boundary conditions, derived from the solved load path.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FeaPartJob {
	/// Part name.
	pub part: String,
	/// Assembly instance index, when declared.
	pub instance: Option<usize>,
	/// Point loads (mate reactions + the part's own external forces).
	pub loads: Vec<FeaPointLoad>,
	/// Fixtures (the ground-ward mate).
	pub fixtures: Vec<FeaFixture>,
	/// Moments that are not representable as ACE point loads.
	pub unrepresented_moments: Vec<FeaMoment>,
	/// Honesty notes a campaign must read before running the job.
	pub notes: Vec<String>,
}

fn unit_or_zero(v: DVec3) -> [f64; 3] {
	match normalize(v) {
		Some(u) => [u.x, u.y, u.z],
		None => [0.0, 0.0, 0.0],
	}
}

/// BFS distance (in connections) from ground to each part; `usize::MAX` for a
/// part with no path to ground.
fn ground_depth(case: &LoadCase) -> Vec<usize> {
	let mut depth = vec![usize::MAX; case.parts.len()];
	let mut q: VecDeque<usize> = VecDeque::new();
	for c in &case.connections {
		for (this, other) in [(c.a, c.b), (c.b, c.a)] {
			if other == Attach::Ground {
				if let Attach::Part(i) = this {
					if i < depth.len() && depth[i] == usize::MAX {
						depth[i] = 0;
						q.push_back(i);
					}
				}
			}
		}
	}
	while let Some(i) = q.pop_front() {
		let d = depth[i];
		for c in &case.connections {
			let next = match (c.a, c.b) {
				(Attach::Part(x), Attach::Part(y)) if x == i => Some(y),
				(Attach::Part(x), Attach::Part(y)) if y == i => Some(x),
				_ => None,
			};
			if let Some(j) = next {
				if j < depth.len() && depth[j] == usize::MAX {
					depth[j] = d + 1;
					q.push_back(j);
				}
			}
		}
	}
	depth
}

impl LoadCase {
	/// An empty named case.
	pub fn new(name: impl Into<String>) -> Self {
		Self { name: name.into(), parts: Vec::new(), connections: Vec::new(), loads: Vec::new() }
	}

	/// Add a part, returning its index.
	pub fn add_part(&mut self, name: impl Into<String>) -> usize {
		let i = self.parts.len();
		self.parts.push(Part { name: name.into(), instance: None });
		i
	}

	/// Add a part bound to an [`crate::Assembly`] instance index.
	pub fn add_instance(&mut self, name: impl Into<String>, instance: usize) -> usize {
		let i = self.parts.len();
		self.parts.push(Part { name: name.into(), instance: Some(instance) });
		i
	}

	/// Add a connection, returning its index.
	pub fn add_connection(&mut self, c: Connection) -> usize {
		let i = self.connections.len();
		self.connections.push(c);
		i
	}

	/// Add an external load, returning its index.
	pub fn add_load(&mut self, l: AppliedLoad) -> usize {
		let i = self.loads.len();
		self.loads.push(l);
		i
	}

	/// Number of connections that touch ground.
	pub fn support_count(&self) -> usize {
		self.connections.iter().filter(|c| c.is_support()).count()
	}

	/// Total scalar reaction unknowns across all connections.
	pub fn unknown_count(&self) -> usize {
		self.connections.iter().map(|c| c.kind.unknowns()).sum()
	}

	/// Structural validation, run first by [`LoadCase::solve`].
	pub fn validate(&self) -> Result<(), LoadError> {
		if self.parts.is_empty() {
			return Err(LoadError::Empty { case: self.name.clone() });
		}
		let n = self.parts.len();
		for c in &self.connections {
			for end in [c.a, c.b] {
				if let Attach::Part(i) = end {
					if i >= n {
						return Err(LoadError::UnknownPart { context: format!("connection '{}'", c.name), index: i, parts: n });
					}
				}
			}
			if c.a == c.b {
				return Err(LoadError::SelfConnection { connection: c.name.clone(), part: c.a.part() });
			}
			if !c.point.is_finite() {
				return Err(LoadError::NonFinite { context: format!("connection '{}'", c.name), field: "point" });
			}
			if c.kind.basis().is_none() {
				return Err(LoadError::DegenerateAxis { connection: c.name.clone(), kind: c.kind.name() });
			}
		}
		for l in &self.loads {
			if l.part >= n {
				return Err(LoadError::UnknownPart { context: format!("load '{}'", l.name), index: l.part, parts: n });
			}
			if !l.point.is_finite() {
				return Err(LoadError::NonFinite { context: format!("load '{}'", l.name), field: "point" });
			}
			if !l.force.is_finite() {
				return Err(LoadError::NonFinite { context: format!("load '{}'", l.name), field: "force" });
			}
			if !l.moment.is_finite() {
				return Err(LoadError::NonFinite { context: format!("load '{}'", l.name), field: "moment" });
			}
		}
		Ok(())
	}

	/// **Solve the load path.** See the [module docs](self) for the method and
	/// its three explicit outcomes.
	///
	/// On success every part's equilibrium residual is carried in the report
	/// ([`LoadPath::max_residual_force`] / [`LoadPath::max_residual_moment`]),
	/// so a caller can — and a campaign gate should — assert that the answer
	/// closes rather than trusting that it did.
	pub fn solve(&self) -> Result<LoadPath, LoadError> {
		self.validate()?;
		let n_parts = self.parts.len();
		let rows = 6 * n_parts;

		// Column layout: each connection owns a contiguous block.
		let mut col_start = Vec::with_capacity(self.connections.len());
		let mut bases = Vec::with_capacity(self.connections.len());
		let mut cols = 0usize;
		for c in &self.connections {
			let basis = c.kind.basis().ok_or(LoadError::DegenerateAxis { connection: c.name.clone(), kind: c.kind.name() })?;
			col_start.push(cols);
			cols += basis.0.len() + basis.1.len();
			bases.push(basis);
		}

		let mut a = vec![vec![0.0f64; cols]; rows];
		for (ci, c) in self.connections.iter().enumerate() {
			let (fdirs, mdirs) = &bases[ci];
			let base = col_start[ci];
			for (end, s) in [(c.a, 1.0f64), (c.b, -1.0f64)] {
				let Some(p) = end.part() else { continue };
				for (k, f) in fdirs.iter().enumerate() {
					let col = base + k;
					let m = c.point.cross(*f);
					a[6 * p][col] += s * f.x;
					a[6 * p + 1][col] += s * f.y;
					a[6 * p + 2][col] += s * f.z;
					a[6 * p + 3][col] += s * m.x;
					a[6 * p + 4][col] += s * m.y;
					a[6 * p + 5][col] += s * m.z;
				}
				for (j, mdir) in mdirs.iter().enumerate() {
					let col = base + fdirs.len() + j;
					a[6 * p + 3][col] += s * mdir.x;
					a[6 * p + 4][col] += s * mdir.y;
					a[6 * p + 5][col] += s * mdir.z;
				}
			}
		}

		// External resultants per part, about the world origin.
		let mut ext_f = vec![DVec3::ZERO; n_parts];
		let mut ext_m = vec![DVec3::ZERO; n_parts];
		for l in &self.loads {
			ext_f[l.part] += l.force;
			ext_m[l.part] += l.point.cross(l.force) + l.moment;
		}
		let mut rhs = vec![0.0f64; rows];
		for p in 0..n_parts {
			rhs[6 * p] = -ext_f[p].x;
			rhs[6 * p + 1] = -ext_f[p].y;
			rhs[6 * p + 2] = -ext_f[p].z;
			rhs[6 * p + 3] = -ext_m[p].x;
			rhs[6 * p + 4] = -ext_m[p].y;
			rhs[6 * p + 5] = -ext_m[p].z;
		}

		let red = row_reduce(&mut a, &mut rhs, cols);
		let net_f: DVec3 = ext_f.iter().copied().sum();
		let net_m: DVec3 = ext_m.iter().copied().sum();
		if red.worst_inconsistency > 0.0 {
			return Err(LoadError::NoEquilibrium {
				case: self.name.clone(),
				net_force: [net_f.x, net_f.y, net_f.z],
				net_moment: [net_m.x, net_m.y, net_m.z],
				supports: self.support_count(),
				worst_residual: red.worst_inconsistency,
				hint: if self.support_count() == 0 {
					"the assembly is FLOATING — no connection reaches ground, so nothing can react the applied loads. Add a \
					 support (a Connection to Attach::Ground) at every place a fixture really holds the assembly."
						.to_string()
				} else {
					"the supports that exist are free in the direction the load acts (a roller cannot take a side load, a \
					 revolute cannot take a moment about its own axis). Either the load case or the support model is wrong."
						.to_string()
				},
			});
		}
		if red.rank < cols {
			return Err(LoadError::Indeterminate {
				case: self.name.clone(),
				redundancy: cols - red.rank,
				unknowns: cols,
				independent_equations: red.rank,
				hint: "remove the redundant supports/joints to make the model determinate (a propped cantilever becomes \
				 determinate when the prop or the clamp goes), model the redundant path as a released joint (a Contact or \
				 Spherical instead of Rigid), or bring in stiffness — a rigid-body solve cannot distribute load between \
				 redundant paths."
					.to_string(),
			});
		}

		// Unique solution: free columns do not exist (rank == cols).
		let x = back_substitute(&a, &rhs, cols, &red.pivots);

		let mut connections = Vec::with_capacity(self.connections.len());
		for (ci, c) in self.connections.iter().enumerate() {
			let (fdirs, mdirs) = &bases[ci];
			let base = col_start[ci];
			let mut force = DVec3::ZERO;
			for (k, f) in fdirs.iter().enumerate() {
				force += *f * x[base + k];
			}
			let mut moment = DVec3::ZERO;
			for (j, mdir) in mdirs.iter().enumerate() {
				moment += *mdir * x[base + fdirs.len() + j];
			}
			let tension = matches!(c.kind, JointKind::Contact { .. }) && x[base] < 0.0;
			connections.push(ConnectionReaction {
				index: ci,
				name: c.name.clone(),
				kind: c.kind.name().to_string(),
				a: c.a,
				b: c.b,
				point: c.point,
				force,
				moment,
				tension_on_unilateral: tension,
			});
		}

		let mut per_part = Vec::with_capacity(n_parts);
		let mut max_rf = 0.0f64;
		let mut max_rm = 0.0f64;
		for p in 0..n_parts {
			let mut via = Vec::new();
			let mut rf = DVec3::ZERO;
			let mut rm = DVec3::ZERO;
			for (ci, c) in self.connections.iter().enumerate() {
				let s = if c.a == Attach::Part(p) {
					1.0
				} else if c.b == Attach::Part(p) {
					-1.0
				} else {
					continue;
				};
				let other = if s > 0.0 { c.b } else { c.a };
				let f = connections[ci].force * s;
				let m = connections[ci].moment * s;
				rf += f;
				rm += c.point.cross(f) + m;
				via.push(MateReaction { connection: ci, name: c.name.clone(), other, point: c.point, force: f, moment: m });
			}
			let residual_force = ext_f[p] + rf;
			let residual_moment = ext_m[p] + rm;
			max_rf = max_rf.max(residual_force.length());
			max_rm = max_rm.max(residual_moment.length());
			per_part.push(PartReaction {
				part: p,
				name: self.parts[p].name.clone(),
				instance: self.parts[p].instance,
				external_force: ext_f[p],
				external_moment: ext_m[p],
				reaction_force: rf,
				reaction_moment: rm,
				via_mates: via,
				residual_force,
				residual_moment,
			});
		}

		// Global closure: externals plus everything ground hands in.
		let mut gf = net_f;
		let mut gm = net_m;
		for (ci, c) in self.connections.iter().enumerate() {
			let s = if c.b == Attach::Ground {
				1.0
			} else if c.a == Attach::Ground {
				-1.0
			} else {
				continue;
			};
			let f = connections[ci].force * s;
			let m = connections[ci].moment * s;
			gf += f;
			gm += c.point.cross(f) + m;
		}

		Ok(LoadPath {
			case: self.name.clone(),
			per_part,
			connections,
			unknowns: cols,
			independent_equations: red.rank,
			max_residual_force: max_rf,
			max_residual_moment: max_rm,
			global_residual_force: gf.length(),
			global_residual_moment: gm.length(),
		})
	}
}

/// What [`row_reduce`] learned about the system.
struct Reduction {
	rank: usize,
	/// Pivot column of each pivot row, in row order.
	pivots: Vec<usize>,
	/// Largest `|rhs|` left on a row whose coefficients all vanished — zero
	/// when the system is consistent.
	worst_inconsistency: f64,
}

/// Row-echelon reduce `[a | rhs]` in place with partial pivoting.
fn row_reduce(a: &mut [Vec<f64>], rhs: &mut [f64], cols: usize) -> Reduction {
	let rows = a.len();
	let scale = a.iter().flat_map(|r| r.iter()).fold(0.0f64, |m, v| m.max(v.abs())).max(1e-300);
	let eps = PIVOT_EPS * scale;
	let mut pivots = Vec::new();
	let mut row = 0usize;
	for col in 0..cols {
		if row >= rows {
			break;
		}
		let (best, mag) = (row..rows).fold((row, 0.0f64), |(bi, bm), r| {
			let v = a[r][col].abs();
			if v > bm {
				(r, v)
			} else {
				(bi, bm)
			}
		});
		if mag <= eps {
			continue;
		}
		a.swap(row, best);
		rhs.swap(row, best);
		let pivot = a[row][col];
		for r in (row + 1)..rows {
			let factor = a[r][col] / pivot;
			if factor == 0.0 {
				continue;
			}
			let (top, bottom) = a.split_at_mut(r);
			for (target, p) in bottom[0].iter_mut().zip(top[row].iter()).skip(col) {
				*target -= factor * p;
			}
			rhs[r] -= factor * rhs[row];
		}
		pivots.push(col);
		row += 1;
	}
	let rank = pivots.len();
	// Any row past the last pivot must have a vanishing rhs, or there is no solution.
	let rhs_scale = rhs.iter().fold(0.0f64, |m, v| m.max(v.abs())).max(1.0);
	let tol = CONSISTENCY_EPS * rhs_scale;
	let worst = (rank..rows).fold(0.0f64, |m, r| m.max(rhs[r].abs()));
	Reduction { rank, pivots, worst_inconsistency: if worst > tol { worst } else { 0.0 } }
}

/// Back-substitute a reduced full-column-rank system (`pivots.len() == cols`).
fn back_substitute(a: &[Vec<f64>], rhs: &[f64], cols: usize, pivots: &[usize]) -> Vec<f64> {
	let mut x = vec![0.0f64; cols];
	for (row, &col) in pivots.iter().enumerate().rev() {
		let mut acc = rhs[row];
		for c in (col + 1)..cols {
			acc -= a[row][c] * x[c];
		}
		x[col] = acc / a[row][col];
	}
	x
}
