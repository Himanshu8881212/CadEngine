// Copyright (c) LMCAD. Licensed under the MIT License.

//! The **tiered boolean policy** — a declared, instrumented subsystem for *which*
//! path a boolean took and *how much error* that path admits.
//!
//! The hybrid kernel deliberately does not bet everything on one boolean
//! algorithm. Exact B-rep arrangements are watertight and analytically exact when
//! they succeed, but they are the fragile surface of any solid modeller
//! (coincident/tangent faces, sub-tolerance slivers). So the engine layers
//! *fallbacks* under the exact path, degrading accuracy in a stated, bounded way
//! rather than returning corrupt topology. Historically **which** fallback fired
//! was tribal knowledge buried in call sites; this module turns it into a policy
//! with a machine-readable record ([`BooleanOutcome`]) and an aggregatable metric
//! ([`BooleanStats`]).
//!
//! ## The declared tiers (whole-engine policy)
//! A boolean is attempted top-down; the first tier that yields a *validated*
//! closed 2-manifold wins, and the tier is recorded.
//!
//! 1. **EXACT — exact B-rep arrangement.** [`union`](crate::union) /
//!    [`difference`](crate::difference) / [`intersection`](crate::intersection):
//!    co-refine, classify and stitch the planar arrangement, then
//!    [`validate`]. On success the result is exact to `f64` round-off — **stated
//!    error bound `0`**. This is the only tier that preserves analytic surface
//!    tags and exact volume.
//! 2. **HEALED FALLBACK — tolerant heal, then the identical exact arrangement.**
//!    [`boolean_tolerant`](crate::boolean_tolerant): weld the operands' import-grade
//!    gaps/slivers within `tol`, re-run the *same* exact boolean, re-validate.
//!    Geometry is unchanged on clean operands; on cracked ones the join closes.
//!    **Stated error bound `tol`** — features/gaps at or below the tolerance are
//!    legitimately collapsed (that is what the tolerance *means*). This tier
//!    rescues *cracked operands*; it does **not** resolve hard coincident/tangent
//!    -face arrangement degeneracies (those still refuse — see the honest note).
//! 3. **VOXEL CARVE — winding-number / SDF re-mesh.** When even the healed exact
//!    arrangement cannot validate the join (hard coincident/tangent faces, or a
//!    densely-meshed / implicit-field operand), the engine re-samples **both**
//!    sides into a signed distance field (generalized winding number for the
//!    sign) and CSGs them, re-meshing at `voxel` resolution — *always* watertight
//!    but voxel-approximate everywhere. **Stated error bound ≈ `voxel`** (the grid
//!    spacing). **This tier lives at the model layer**
//!    (`kernel_model::hybrid_boolean` → `HybridRoute::Healed`), because it needs
//!    the implicit crate; `kernel-brep` does not depend on it and therefore
//!    **cannot take this tier itself**. A [`BooleanOutcome::Refused`] from this
//!    crate is precisely the escalation signal the model layer acts on.
//! 4. **REFUSED — withhold.** No reachable tier produced a valid solid. The
//!    result is *not* returned; [`BooleanPath::Refused`] carries the failing
//!    [`Validity`] so the caller re-routes (to the voxel carve, or to a redesign)
//!    instead of chaining features onto corrupt topology.
//!
//! ## What this module does and does NOT change
//! [`boolean_with_policy`] is **instrumentation, not a new algorithm**: on the
//! EXACT path it returns the *bit-identical* solid the raw op builds (the same
//! thing [`try_union`](crate::try_union) validates); on the HEALED path it returns
//! exactly what [`boolean_tolerant`] builds. It only *records* which happened and
//! the stated error bound. It is deterministic — the exact arrangement is
//! determinism-pinned (R5) and the heal is index-order, so identical input yields
//! an identical outcome, path included.
//!
//! ## Honest note — the highest-risk surface
//! The raw [`union`](crate::union) / [`difference`](crate::difference) /
//! [`intersection`](crate::intersection) run the exact arrangement with **no
//! validation and no fallback** and hand back whatever they build. A caller that
//! uses them directly (as `fuzz_chains` and the `parts_gallery` do, by design, to
//! *measure* the exact tier) gets no path record and no safety net — that is the
//! most fragile way to call a boolean. Prefer [`boolean_with_policy`] (or the
//! [`try_*`](crate::try_union) API) wherever an autonomous caller must not chain
//! onto an unvalidated result.

use crate::checked::BooleanError;
use crate::heal::boolean_tolerant;
use crate::mesh_boolean::MeshBoolOp;
use crate::topo::Solid;
use crate::validate::{validate, Validity};

/// Which tier of the [tiered boolean policy](self) produced a result.
///
/// The variants are ordered by increasing error: [`Exact`](BooleanPath::Exact)
/// (bound `0`) → [`HealedFallback`](BooleanPath::HealedFallback) (bound `tol`) →
/// [`Refused`](BooleanPath::Refused) (no result). The VOXEL CARVE tier is not
/// represented here because `kernel-brep` cannot take it (see the [module
/// docs](self)); a `Refused` is the signal the model layer escalates to it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BooleanPath {
	/// Tier 1: the exact B-rep arrangement validated. Bit-identical to the raw
	/// op; exact to `f64` round-off (error bound `0`).
	Exact,
	/// Tier 2: the exact arrangement did not validate, but a tolerant heal at
	/// `tol` (mm) closed the operands' cracks and the re-run exact boolean
	/// validated. Error bound `tol` — features/gaps ≤ `tol` may be collapsed.
	HealedFallback {
		/// The heal tolerance that rescued the boolean, in mm — the stated error
		/// bound of this path.
		tol: f64,
	},
	/// Tier 4: no reachable tier produced a valid solid; the result was withheld.
	/// (The model layer may still escalate to the voxel-carve tier.)
	Refused,
}

impl BooleanPath {
	/// The stated worst-case error bound of this path, in mm: `Some(0.0)` for
	/// [`Exact`](BooleanPath::Exact), `Some(tol)` for the healed fallback, and
	/// `None` for a [`Refused`](BooleanPath::Refused) boolean (there is no result
	/// to bound).
	pub fn error_bound(&self) -> Option<f64> {
		match self {
			BooleanPath::Exact => Some(0.0),
			BooleanPath::HealedFallback { tol } => Some(*tol),
			BooleanPath::Refused => None,
		}
	}

	/// A short, stable label for reports/metrics: `"exact"`, `"healed"` or
	/// `"refused"`.
	pub fn label(&self) -> &'static str {
		match self {
			BooleanPath::Exact => "exact",
			BooleanPath::HealedFallback { .. } => "healed",
			BooleanPath::Refused => "refused",
		}
	}
}

/// The machine-readable record of one [`boolean_with_policy`] call: the operation,
/// the [`BooleanPath`] taken, the [`Validity`] of the result (or of the failure),
/// and the resulting [`Solid`] (absent only when [`Refused`](BooleanPath::Refused)).
///
/// This is *what happened*, not new geometry: on the EXACT path `solid` is the
/// bit-identical raw-op result; on the HEALED path it is exactly the
/// [`boolean_tolerant`] result; on REFUSED it is `None` and `validity` explains
/// why the exact tier failed.
#[derive(Clone, Debug)]
pub struct BooleanOutcome {
	/// Which boolean ran: `"union"`, `"difference"` or `"intersection"`.
	pub op: &'static str,
	/// The policy tier that produced (or refused) the result.
	pub path: BooleanPath,
	/// Validity of the returned solid on a success path; on [`Refused`](BooleanPath::Refused)
	/// it is the (invalid) report of the exact tier — the reason for escalation.
	pub validity: Validity,
	/// The validated result solid, or `None` when the boolean was refused.
	pub solid: Option<Solid>,
}

impl BooleanOutcome {
	/// Whether the exact tier succeeded (error bound `0`).
	pub fn is_exact(&self) -> bool {
		matches!(self.path, BooleanPath::Exact)
	}

	/// Whether a fallback tier produced the result (currently only the healed
	/// fallback in this crate) — i.e. the exact tier alone was insufficient.
	pub fn fell_back(&self) -> bool {
		matches!(self.path, BooleanPath::HealedFallback { .. })
	}

	/// Whether the boolean was refused (no valid solid produced in any reachable
	/// tier).
	pub fn refused(&self) -> bool {
		matches!(self.path, BooleanPath::Refused)
	}

	/// The stated error bound of the path taken (see [`BooleanPath::error_bound`]).
	pub fn error_bound(&self) -> Option<f64> {
		self.path.error_bound()
	}

	/// Consume the outcome into the strict [`try_*`](crate::try_union) contract: the
	/// validated solid on any success path, or the same [`BooleanError`] the strict
	/// checked API raises on refusal. Lets a caller opt into the outcome record
	/// while keeping `?`-friendly ergonomics.
	pub fn into_result(self) -> Result<Solid, BooleanError> {
		match self.solid {
			Some(s) => Ok(s),
			None => Err(BooleanError { op: self.op, validity: self.validity }),
		}
	}
}

/// Run a boolean under the [tiered policy](self), recording the path taken.
///
/// Tiers are attempted in order and the first validated one wins:
/// 1. the **exact** B-rep arrangement ([`union`](crate::union) /
///    [`difference`](crate::difference) / [`intersection`](crate::intersection)) +
///    [`validate`] → [`BooleanPath::Exact`];
/// 2. if that does not validate *and* `heal_tol > 0`, the **healed fallback**
///    ([`boolean_tolerant`] at `heal_tol`) → [`BooleanPath::HealedFallback`];
/// 3. otherwise the boolean is **refused** → [`BooleanPath::Refused`] (the model
///    layer may escalate this to the voxel carve).
///
/// Pass `heal_tol = 0.0` to disable the fallback tier — then this is exactly the
/// strict [`try_*`](crate::try_union) behavior (exact-or-refuse), plus the outcome
/// record. The geometry returned is never altered by this function: it is the
/// bit-identical raw-op solid (EXACT) or the [`boolean_tolerant`] solid (HEALED).
///
/// Deterministic: every tier is deterministic (the arrangement is pinned by R5,
/// the heal is index-order), so identical inputs yield an identical outcome.
///
/// ```
/// use kernel_brep::{boolean_with_policy, cuboid, cylinder, MeshBoolOp};
/// use kernel_brep::math::DVec3;
/// let plate = cuboid(DVec3::new(-10.0, -10.0, -3.0), DVec3::new(10.0, 10.0, 3.0));
/// let bore = cylinder(DVec3::new(0.0, 0.0, -4.0), DVec3::Z, 2.5, 8.0, 48);
/// let out = boolean_with_policy(&plate, &bore, MeshBoolOp::Difference, 1e-6);
/// assert!(out.is_exact() && out.error_bound() == Some(0.0));
/// ```
pub fn boolean_with_policy(a: &Solid, b: &Solid, op: MeshBoolOp, heal_tol: f64) -> BooleanOutcome {
	let name = op_name(op);

	// --- Tier 1: exact B-rep arrangement. Identical geometry to the raw op. ------
	let exact = raw_boolean(a, b, op);
	let v = validate(&exact);
	if v.is_valid() {
		return BooleanOutcome { op: name, path: BooleanPath::Exact, validity: v, solid: Some(exact) };
	}

	// --- Tier 2: healed fallback (only if the caller admitted a tolerance). -------
	// `boolean_tolerant` heals both operands at `heal_tol`, re-runs the identical
	// exact boolean, and returns Ok only if the result validates — so a returned
	// solid is a genuine, validated fallback, never a silently-degraded one.
	if heal_tol > 0.0 {
		if let Ok(tb) = boolean_tolerant(a, b, op, heal_tol) {
			let hv = validate(&tb.solid);
			return BooleanOutcome { op: name, path: BooleanPath::HealedFallback { tol: heal_tol }, validity: hv, solid: Some(tb.solid) };
		}
	}

	// --- Tier 4: refused. The voxel-carve tier (Tier 3) is unreachable from this
	// crate (no dependency on the implicit domain); the model layer escalates a
	// refusal to it. Report the exact tier's validity as the reason.
	BooleanOutcome { op: name, path: BooleanPath::Refused, validity: v, solid: None }
}

/// Aggregate path breakdown over a batch of [`BooleanOutcome`]s — the metric that
/// makes "how often do we fall back / refuse" measurable.
///
/// A plain accumulating counter (no global state, no interior mutability, no
/// `HashMap`): the caller threads one through a batch and reads the rates. A fuzz
/// corpus, a chain replay, or a dedicated metric test can all feed it and publish
/// the breakdown. Deterministic by construction.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct BooleanStats {
	/// Booleans that took the EXACT tier (error bound `0`).
	pub exact: usize,
	/// Booleans that took the HEALED fallback tier (error bound = the heal `tol`).
	pub healed_fallback: usize,
	/// Booleans that were REFUSED (no valid result in any reachable tier).
	pub refused: usize,
}

impl BooleanStats {
	/// Fold one outcome's path into the running counts.
	pub fn record(&mut self, outcome: &BooleanOutcome) {
		match outcome.path {
			BooleanPath::Exact => self.exact += 1,
			BooleanPath::HealedFallback { .. } => self.healed_fallback += 1,
			BooleanPath::Refused => self.refused += 1,
		}
	}

	/// Total booleans recorded.
	pub fn total(&self) -> usize {
		self.exact + self.healed_fallback + self.refused
	}

	/// Fraction (0..=1) that took the exact tier; `0.0` for an empty batch.
	pub fn exact_rate(&self) -> f64 {
		ratio(self.exact, self.total())
	}

	/// Fraction (0..=1) that needed a fallback tier; `0.0` for an empty batch.
	pub fn fallback_rate(&self) -> f64 {
		ratio(self.healed_fallback, self.total())
	}

	/// Fraction (0..=1) that were refused; `0.0` for an empty batch.
	pub fn refusal_rate(&self) -> f64 {
		ratio(self.refused, self.total())
	}
}

/// `n / d` as a fraction, `0.0` when `d == 0` (an empty batch has no rate).
fn ratio(n: usize, d: usize) -> f64 {
	if d == 0 {
		0.0
	} else {
		n as f64 / d as f64
	}
}

/// The stable op label used in [`BooleanOutcome::op`] and [`BooleanError`].
fn op_name(op: MeshBoolOp) -> &'static str {
	match op {
		MeshBoolOp::Union => "union",
		MeshBoolOp::Difference => "difference",
		MeshBoolOp::Intersection => "intersection",
	}
}

/// Dispatch to the raw exact boolean — the exact tier's geometry, unchanged.
fn raw_boolean(a: &Solid, b: &Solid, op: MeshBoolOp) -> Solid {
	match op {
		MeshBoolOp::Union => crate::booleans::union(a, b),
		MeshBoolOp::Difference => crate::booleans::difference(a, b),
		MeshBoolOp::Intersection => crate::booleans::intersection(a, b),
	}
}
