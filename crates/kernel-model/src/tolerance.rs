// Copyright (c) LMCAD. Licensed under the MIT License.

//! Tolerance **stack-up over assembly chains** — the system-level answer to
//! "every fit passed its pairwise check and the assembly still does not go
//! together".
//!
//! A [`Stack`] is an ordered chain of [`Contribution`]s, each a [`Dimension`]
//! (`nominal` with independent `plus` / `minus` bands) carried with a [`Sign`]
//! (`Adds` / `Subtracts`) into the accumulated result. Two accumulation
//! methods, both stated explicitly in the result they produce:
//!
//! - [`Stack::worst_case`] — arithmetic sum of extremes. Every contributor at
//!   its unfavourable limit simultaneously. A hard bound: no distribution is
//!   assumed and none is needed.
//! - [`Stack::rss`] — root-sum-square. **Statistical**, and therefore only as
//!   true as its assumption, which is carried in
//!   [`StackResult::assumption`]: independent, normally distributed
//!   contributors, each centred on the mid-point of its own limits, each
//!   stated band equal to ±`sigma_level`·σ. An RSS number quoted without that
//!   sentence is a lie, so this module never produces one without it.
//!
//! [`Stack::gate`] is the campaign-facing check: does the accumulated
//! variation fit inside a required window? It refuses with a typed
//! [`StackViolation`] naming the **dominant contributor** — the one dimension
//! that owns the largest share of the accumulated band, which is the only
//! actionable output a stack-up has.
//!
//! # The aggregate failure this exists to catch
//!
//! [`Stack::gate_report`] answers both questions at once: does each link pass
//! its own budget *alone*, and does the chain pass *together*? When every link
//! passes alone and the chain does not, [`GateReport::aggregate_only_failure`]
//! is set — that is the exact failure mode pairwise fit checking cannot see.
//!
//! # Relationship to [`crate::rate::Stackup`]
//!
//! [`crate::rate::Stackup`] is the single-line screening stackup: unsigned,
//! symmetric `±tol`, `(nominal, band)` out. It stays for that job. This module
//! generalizes it with the four things a system-level stack needs and it does
//! not have: **signs** (a chain has directions), **asymmetric** bands,
//! **provenance** ([`ToleranceSource`] — declared, catalog, or derived from a
//! manufacturing process profile), and a **typed gate** with ranked
//! contributors.
//!
//! # Cross-language agreement
//!
//! Conventions match `tools/tolerance_stack.py` so a Rust gate and the Python
//! receipt agree number-for-number: signed contributions, asymmetric bands
//! handled by the standard **mid-shift** treatment (a `+p/−m` band becomes a
//! bilateral `±(p+m)/2` about the mid-point `nominal + (p−m)/2`), and a
//! default RSS interpretation of **3σ** ([`DEFAULT_SIGMA_LEVEL`]).
//!
//! # Scope and limits (what this is NOT)
//!
//! - **1-D linear chains only.** Every contribution accumulates along one
//!   declared direction. No angular stacks, no 2-D/3-D variation propagation.
//! - **No GD&T.** Position/orientation/profile tolerance zones, datum shift
//!   and bonus tolerance are not modelled. A GD&T zone must be reduced to an
//!   equivalent 1-D band by the caller, who then owns that reduction.
//! - **No correlation.** RSS assumes independence. Parts off the same printer,
//!   the same layer height, the same batch are correlated, and the RSS band is
//!   then optimistic — [`StackMethod::assumption`] says so in the receipt.
//! - **No Monte-Carlo / non-normal distributions.** Only the two closed forms.

use serde::{Deserialize, Serialize};

use kernel_core::math::{Affine3A, DVec3};

use crate::process::FdmProfile;
use crate::Assembly;

/// The sigma level [`Stack::rss`] is meant to be called with unless a campaign
/// has measured evidence for another: **3σ**, matching the convention frozen
/// in `tools/tolerance_stack.py` so Rust gates and Python receipts agree.
pub const DEFAULT_SIGMA_LEVEL: f64 = 3.0;

/// The statistical assumption every RSS result carries. Stated once here so
/// the sentence in a receipt and the sentence in this doc can never drift.
pub const RSS_DISTRIBUTION: &str = "independent, normally distributed contributors, each centred on the mid-point of its own limits, \
	 with each contributor's stated band equal to ±sigma_level·σ";

/// Below this projection length (mm) a chain link has no usable direction and
/// [`Stack::from_pose_chain`] refuses rather than guess a sign.
const CHAIN_MIN_PROJECTION: f64 = 1e-9;

/// A dimension with **asymmetric** tolerances: it may measure anywhere in
/// `[nominal − minus, nominal + plus]`.
///
/// `plus` and `minus` are both non-negative magnitudes (a `+0.2/−0.05`
/// dimension is `plus = 0.2, minus = 0.05`). Fields are public so a caller can
/// build one literally; [`Dimension::validate`] is the loud check that the
/// bands are sane.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Dimension {
	/// The design/basic value.
	pub nominal: f64,
	/// Upper deviation magnitude (≥ 0): the dimension may measure up to
	/// `nominal + plus`.
	pub plus: f64,
	/// Lower deviation magnitude (≥ 0): the dimension may measure down to
	/// `nominal − minus`.
	pub minus: f64,
}

impl Dimension {
	/// A bilateral dimension `nominal ± tol` (the tolerance magnitude is used
	/// as given; a negative `tol` is caught by [`Dimension::validate`]).
	pub fn symmetric(nominal: f64, tol: f64) -> Self {
		Self { nominal, plus: tol, minus: tol }
	}

	/// An asymmetric dimension `nominal +plus/−minus`.
	pub fn asymmetric(nominal: f64, plus: f64, minus: f64) -> Self {
		Self { nominal, plus, minus }
	}

	/// A dimension with no variation at all (a datum, a machined gauge block,
	/// or a value whose scatter is deliberately excluded — say so if so).
	pub fn exact(nominal: f64) -> Self {
		Self { nominal, plus: 0.0, minus: 0.0 }
	}

	/// A required **window** `[min, max]` expressed as a dimension centred on
	/// the window mid-point — the shape [`Stack::gate`] takes as its target
	/// (e.g. "the clearance must land between 0.05 and 0.60 mm").
	pub fn window(min: f64, max: f64) -> Self {
		let mid = 0.5 * (min + max);
		let half = 0.5 * (max - min);
		Self { nominal: mid, plus: half, minus: half }
	}

	/// Lower limit `nominal − minus`.
	pub fn lower(&self) -> f64 {
		self.nominal - self.minus
	}

	/// Upper limit `nominal + plus`.
	pub fn upper(&self) -> f64 {
		self.nominal + self.plus
	}

	/// Mid-point of the limits, `nominal + (plus − minus)/2`. Equals `nominal`
	/// exactly when the dimension is symmetric. This is the statistical centre
	/// RSS accumulates about (the standard mid-shift treatment).
	pub fn mid(&self) -> f64 {
		self.nominal + 0.5 * (self.plus - self.minus)
	}

	/// Half the total band, `(plus + minus)/2` — the equivalent bilateral
	/// tolerance about [`Dimension::mid`].
	pub fn half_band(&self) -> f64 {
		0.5 * (self.plus + self.minus)
	}

	/// Total band `plus + minus` (`upper − lower`).
	pub fn span(&self) -> f64 {
		self.plus + self.minus
	}

	/// Refuse a malformed dimension: non-finite fields, or a negative band
	/// magnitude (which would silently *shrink* a stack).
	pub fn validate(&self, name: &str) -> Result<(), ToleranceError> {
		for (field, value) in [("nominal", self.nominal), ("plus", self.plus), ("minus", self.minus)] {
			if !value.is_finite() {
				return Err(ToleranceError::NonFinite { name: name.to_string(), field, value });
			}
		}
		if self.plus < 0.0 || self.minus < 0.0 {
			return Err(ToleranceError::NegativeBand { name: name.to_string(), plus: self.plus, minus: self.minus });
		}
		Ok(())
	}
}

/// Which way a contribution accumulates along the chain.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Sign {
	/// The dimension adds to the accumulated value (material, a spacer, a
	/// housing depth measured with the chain).
	Adds,
	/// The dimension subtracts (a bore, a pocket, anything measured against
	/// the chain direction).
	Subtracts,
}

impl Sign {
	/// `+1.0` for [`Sign::Adds`], `−1.0` for [`Sign::Subtracts`].
	pub fn factor(self) -> f64 {
		match self {
			Sign::Adds => 1.0,
			Sign::Subtracts => -1.0,
		}
	}

	/// The opposite sign — used by the mis-sign negative control.
	pub fn flipped(self) -> Sign {
		match self {
			Sign::Adds => Sign::Subtracts,
			Sign::Subtracts => Sign::Adds,
		}
	}

	/// `"+"` / `"-"`, for messages.
	pub fn symbol(self) -> &'static str {
		match self {
			Sign::Adds => "+",
			Sign::Subtracts => "-",
		}
	}
}

/// Where a contribution's tolerance came from — provenance is part of the
/// number. A stack whose dominant contributor is `Declared` is a drawing
/// problem; one whose dominant contributor is `Process` is a printer problem.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum ToleranceSource {
	/// Hand-declared by the designer (a drawing note, a design decision).
	Declared,
	/// A vendor / catalog part's published tolerance; the string names it.
	Catalog(String),
	/// Derived from a manufacturing process profile ([`crate::process`]).
	Process {
		/// Profile name (e.g. `"conservative_default"`).
		profile: String,
		/// Feature class the band was derived for.
		feature: String,
		/// What the derivation covers — and what it does not.
		note: String,
	},
}

impl ToleranceSource {
	/// Short label for messages and receipts.
	pub fn label(&self) -> String {
		match self {
			ToleranceSource::Declared => "declared".to_string(),
			ToleranceSource::Catalog(v) => format!("catalog:{v}"),
			ToleranceSource::Process { profile, feature, .. } => format!("process:{profile}/{feature}"),
		}
	}
}

/// A printed feature class whose dimensional behaviour an [`FdmProfile`]
/// actually knows something about.
///
/// Deliberately short: the profile carries *measured fit clearances and
/// compensations*, not a measured dimensional scatter. Only the classes the
/// profile can honestly answer for are offered here — inventing a σ from
/// fields that do not contain one would be a gamed number.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PrintedFeature {
	/// An OUTER printed surface measured radially (a boss, pin, rail flank).
	/// First-layer flare and the Z-seam bump can only make it **bigger**, so
	/// the derived band is one-sided: `plus = first_layer_comp +
	/// seam_allowance`, `minus = 0`.
	OuterSurfaceRadial,
	/// A printed hole / bore **designed at nominal**, i.e. with no
	/// compensation applied. The profile's diametral compensation is a
	/// *systematic bias*, not scatter: the derived contribution shifts the
	/// nominal by `−comp_for_d(nominal)` and carries a **zero** band, with
	/// the note saying so. Add a declared band from your own coupon
	/// measurements if you need the scatter too.
	UncompensatedHoleDiametral,
}

/// One link of a [`Stack`]: a dimension carried with a sign and provenance.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Contribution {
	/// Human-readable name — this is what a [`StackViolation`] blames.
	pub name: String,
	/// Direction this dimension accumulates in.
	pub sign: Sign,
	/// The dimension and its bands.
	pub dim: Dimension,
	/// Where the tolerance came from.
	pub source: ToleranceSource,
}

impl Contribution {
	/// A hand-declared contribution.
	pub fn declared(name: impl Into<String>, sign: Sign, dim: Dimension) -> Self {
		Self { name: name.into(), sign, dim, source: ToleranceSource::Declared }
	}

	/// A contribution whose tolerance comes from a vendor / catalog datasheet.
	pub fn catalog(name: impl Into<String>, sign: Sign, dim: Dimension, part: impl Into<String>) -> Self {
		Self { name: name.into(), sign, dim, source: ToleranceSource::Catalog(part.into()) }
	}

	/// **The process seam, generic form.** A contribution whose tolerance was
	/// produced by a manufacturing process model outside this module: the
	/// caller supplies the band, this constructor records *where it came from*
	/// so the receipt can distinguish a printer problem from a drawing
	/// problem.
	///
	/// Use this when the process model is not [`crate::process`] (a different
	/// process, an external CAM/DFM tool, a measured coupon set), or when the
	/// derivation is the caller's. For FDM features [`crate::process`] can
	/// answer for, prefer [`Contribution::printed_feature`], which derives the
	/// band from the profile instead of trusting a literal.
	#[allow(clippy::too_many_arguments)]
	pub fn from_process_tolerance(
		name: impl Into<String>,
		sign: Sign,
		nominal: f64,
		plus: f64,
		minus: f64,
		profile: impl Into<String>,
		feature: impl Into<String>,
		note: impl Into<String>,
	) -> Self {
		Self {
			name: name.into(),
			sign,
			dim: Dimension::asymmetric(nominal, plus, minus),
			source: ToleranceSource::Process { profile: profile.into(), feature: feature.into(), note: note.into() },
		}
	}

	/// **The process seam, profile-driven form.** Derive this contribution's
	/// band from an [`FdmProfile`] instead of a literal.
	///
	/// Derivations (each one stated, each one traceable to a profile field
	/// that is *measured* rather than invented):
	///
	/// | [`PrintedFeature`] | nominal | plus | minus |
	/// |---|---|---|---|
	/// | `OuterSurfaceRadial` | as given | `first_layer_comp + seam_allowance` | `0` |
	/// | `UncompensatedHoleDiametral` | `nominal − comp_for_d(nominal)` | `0` | `0` |
	///
	/// **Limit, stated:** an [`FdmProfile`] does not carry a measured
	/// dimensional *scatter* — it carries clearances and compensations. So
	/// `OuterSurfaceRadial` returns a one-sided **envelope** (the most a
	/// printed outer surface can grow), and `UncompensatedHoleDiametral`
	/// returns a **bias** with a zero band. Neither is a σ. If you have coupon
	/// scatter for your printer, add it as a second `Declared` contribution;
	/// the note carried in [`ToleranceSource::Process`] says exactly this.
	pub fn printed_feature(name: impl Into<String>, sign: Sign, nominal: f64, profile: &FdmProfile, feature: PrintedFeature) -> Self {
		let (dim, feature_name, note) = match feature {
			PrintedFeature::OuterSurfaceRadial => (
				Dimension::asymmetric(nominal, profile.first_layer_comp + profile.seam_allowance, 0.0),
				"outer_surface_radial",
				"one-sided GROWTH envelope from first_layer_comp + seam_allowance (elephant foot + Z-seam bump); an outer printed \
				 surface can only get bigger. This is an envelope, NOT a measured sigma — the profile carries no dimensional \
				 scatter; add coupon scatter as a separate declared contribution.",
			),
			PrintedFeature::UncompensatedHoleDiametral => (
				Dimension::exact(nominal - profile.comp_for_d(nominal)),
				"uncompensated_hole_diametral",
				"SYSTEMATIC BIAS only: an uncompensated hole measures nominal - comp_for_d(nominal). The band is ZERO because the \
				 profile carries no measured hole scatter — this contribution moves the mean and asserts nothing about spread; add \
				 coupon scatter as a separate declared contribution.",
			),
		};
		Self {
			name: name.into(),
			sign,
			dim,
			source: ToleranceSource::Process {
				profile: profile.name.clone(),
				feature: feature_name.to_string(),
				note: note.to_string(),
			},
		}
	}

	/// The signed nominal this link puts into the stack.
	pub fn signed_nominal(&self) -> f64 {
		self.sign.factor() * self.dim.nominal
	}

	/// The signed statistical centre this link puts into the stack.
	pub fn signed_mid(&self) -> f64 {
		self.sign.factor() * self.dim.mid()
	}

	/// How far this link can push the stack **up** from its nominal: `plus`
	/// for an adding link, `minus` for a subtracting one.
	pub fn up(&self) -> f64 {
		match self.sign {
			Sign::Adds => self.dim.plus,
			Sign::Subtracts => self.dim.minus,
		}
	}

	/// How far this link can push the stack **down** from its nominal.
	pub fn down(&self) -> f64 {
		match self.sign {
			Sign::Adds => self.dim.minus,
			Sign::Subtracts => self.dim.plus,
		}
	}

	/// Half the link's band — sign-independent (flipping a sign mirrors a
	/// band, it never changes its width).
	pub fn half_band(&self) -> f64 {
		self.dim.half_band()
	}
}

/// How a [`Stack`] was accumulated. Carried in every [`StackResult`] and
/// [`StackViolation`] so a number can never be read without its method.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum StackMethod {
	/// Arithmetic sum of extremes: every contributor at its unfavourable limit
	/// simultaneously. A hard bound; no distribution assumed.
	WorstCase,
	/// Root-sum-square with the stated sigma level. See
	/// [`StackMethod::assumption`] for the full statement.
	Rss {
		/// The sigma level the contributors' bands are declared to represent
		/// (3.0 = each `±band` is a 3σ interval). See [`Stack::rss`] for why
		/// this is a *tag*, not a scale factor.
		sigma_level: f64,
	},
}

impl StackMethod {
	/// Stable short name (`"worst_case"` / `"rss"`).
	pub fn name(&self) -> &'static str {
		match self {
			StackMethod::WorstCase => "worst_case",
			StackMethod::Rss { .. } => "rss",
		}
	}

	/// The full statistical statement this method commits to — the sentence
	/// that has to travel with the number.
	pub fn assumption(&self) -> String {
		match self {
			StackMethod::WorstCase => "worst case: every contributor simultaneously at its unfavourable limit. A hard bound — no \
				 distribution is assumed and none is needed; conforming parts cannot land outside it."
				.to_string(),
			StackMethod::Rss { sigma_level } => format!(
				"root-sum-square at {sigma_level}-sigma: {RSS_DISTRIBUTION}. The accumulated band ±sqrt(sum half_band^2) is therefore \
				 itself a {sigma_level}-sigma interval about the mid-shifted centre. INDEPENDENCE IS ASSUMED, NOT PROVEN — \
				 contributors sharing a printer, a layer height or a batch are correlated and this band is then optimistic; gate \
				 load-bearing fits with worst case."
			),
		}
	}

	/// Refuse a meaningless sigma level (non-finite or ≤ 0).
	pub fn validate(&self) -> Result<(), ToleranceError> {
		match self {
			StackMethod::WorstCase => Ok(()),
			StackMethod::Rss { sigma_level } => {
				if !sigma_level.is_finite() || *sigma_level <= 0.0 {
					Err(ToleranceError::BadSigmaLevel { sigma_level: *sigma_level })
				} else {
					Ok(())
				}
			}
		}
	}
}

/// One contributor's share of the accumulated band — the ranked list that
/// makes a stack-up actionable.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ContributorShare {
	/// Name of the contribution.
	pub name: String,
	/// Its sign in the chain.
	pub sign: Sign,
	/// Its provenance.
	pub source: ToleranceSource,
	/// Its nominal (unsigned).
	pub nominal: f64,
	/// Its own half-band `(plus + minus)/2`.
	pub half_band: f64,
	/// Its share of the **stack** half-band under the result's method:
	/// worst case → `half_band`; RSS → `half_band² / rss_half_band`, the
	/// variance share expressed in band units. Under both methods the shares
	/// sum exactly to the stack half-band (and [`Self::fraction`] sums to 1),
	/// so "this dimension owns 60% of your variation" is literally true.
	pub contribution: f64,
	/// [`Self::contribution`] divided by the stack half-band (0 when the stack
	/// has no variation at all).
	pub fraction: f64,
}

/// The accumulated result of a [`Stack`] under one [`StackMethod`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct StackResult {
	/// The stack's name.
	pub stack: String,
	/// How it was accumulated.
	pub method: StackMethod,
	/// The method's full statistical statement ([`StackMethod::assumption`]),
	/// carried by value so a serialized receipt cannot lose it.
	pub assumption: String,
	/// Σ `sign · nominal` — the pure nominal closure of the chain.
	pub nominal: f64,
	/// Σ `sign · mid` — the statistical centre the band is taken about.
	/// Equals [`Self::nominal`] exactly when every contribution is symmetric.
	pub mean: f64,
	/// Lower accumulated limit.
	pub min: f64,
	/// Upper accumulated limit.
	pub max: f64,
	/// `max − min`.
	pub span: f64,
	/// `span / 2` — the accumulated half-band about [`Self::mean`].
	pub half_band: f64,
	/// Contributors **sorted by [`ContributorShare::contribution`],
	/// descending** (ties broken by name so the order is deterministic).
	pub contributors: Vec<ContributorShare>,
}

impl StackResult {
	/// The single biggest contributor to the accumulated band, if any.
	pub fn dominant(&self) -> Option<&ContributorShare> {
		self.contributors.first()
	}

	/// One-line summary for a gate table row.
	pub fn summary(&self) -> String {
		let dom = match self.dominant() {
			Some(d) => format!("{} owns {:.1}%", d.name, 100.0 * d.fraction),
			None => "no contributors".to_string(),
		};
		format!(
			"{} [{}]: {:.4} in [{:.4}, {:.4}] span {:.4} — {dom}",
			self.stack,
			self.method.name(),
			self.mean,
			self.min,
			self.max,
			self.span
		)
	}
}

/// A typed refusal from [`Stack::gate`] / [`Stack::gate_method`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct StackViolation {
	/// The stack's name.
	pub stack: String,
	/// The method under which it failed.
	pub method: StackMethod,
	/// The method's statistical statement.
	pub assumption: String,
	/// Required lower limit of the target window.
	pub required_min: f64,
	/// Required upper limit of the target window.
	pub required_max: f64,
	/// Achieved lower limit (NaN when the stack is malformed).
	pub achieved_min: f64,
	/// Achieved upper limit (NaN when the stack is malformed).
	pub achieved_max: f64,
	/// How far the stack undershoots `required_min` (0 when it does not).
	pub low_excess: f64,
	/// How far the stack overshoots `required_max` (0 when it does not).
	pub high_excess: f64,
	/// The dominant contributor — the dimension to fix first. `None` only when
	/// the stack is malformed.
	pub dominant: Option<ContributorShare>,
	/// Set when the stack itself was rejected by [`Stack::validate`]; the
	/// achieved numbers are then NaN and mean nothing.
	pub malformed: Option<String>,
	/// Full human-readable refusal.
	pub message: String,
}

impl std::fmt::Display for StackViolation {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		write!(f, "{}", self.message)
	}
}

impl std::error::Error for StackViolation {}

/// Both halves of the gate question: does each link pass its own budget alone,
/// and does the chain pass together?
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GateReport {
	/// The accumulated result.
	pub result: StackResult,
	/// The required window.
	pub target: Dimension,
	/// `None` when the chain passes; the typed refusal when it does not.
	pub aggregate: Option<StackViolation>,
	/// Per link, in chain order: `(name, passes_alone)`. A link "passes alone"
	/// when its own half-band fits inside the target window's half-width —
	/// i.e. a one-link stack of it would be inside the variation budget. This
	/// ignores nominal placement, which is a chain-level property, and is
	/// exactly the pairwise check a fit-by-fit review does.
	pub links_passing_alone: Vec<(String, bool)>,
	/// **The system-level failure**: every link passes alone and the chain
	/// still fails. This is what pairwise fit checking cannot see.
	pub aggregate_only_failure: bool,
	/// `"pass"`, `"fail (aggregate only — every link passes alone)"`, or
	/// `"fail (N link(s) also fail alone)"`.
	pub verdict: String,
}

/// One link of a chain read out of an assembly's poses by
/// [`Stack::from_pose_chain`] / [`Stack::from_assembly_chain`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ChainLink {
	/// Name for the resulting contribution.
	pub name: String,
	/// Index of the instance the link starts at.
	pub from: usize,
	/// Index of the instance the link ends at.
	pub to: usize,
	/// Upper deviation magnitude for this link (≥ 0).
	pub plus: f64,
	/// Lower deviation magnitude for this link (≥ 0).
	pub minus: f64,
}

impl ChainLink {
	/// A link with a symmetric band.
	pub fn symmetric(name: impl Into<String>, from: usize, to: usize, tol: f64) -> Self {
		Self { name: name.into(), from, to, plus: tol, minus: tol }
	}
}

/// Typed refusals from the tolerance module.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum ToleranceError {
	/// A dimension field is not finite.
	NonFinite {
		/// Contribution name.
		name: String,
		/// Which field (`"nominal"` / `"plus"` / `"minus"`).
		field: &'static str,
		/// The offending value.
		value: f64,
	},
	/// A band magnitude is negative — it would *shrink* the stack.
	NegativeBand {
		/// Contribution name.
		name: String,
		/// The `plus` band as given.
		plus: f64,
		/// The `minus` band as given.
		minus: f64,
	},
	/// A stack with no contributions cannot accumulate anything.
	EmptyStack {
		/// The stack's name.
		stack: String,
	},
	/// An RSS sigma level that is not a positive finite number.
	BadSigmaLevel {
		/// The offending value.
		sigma_level: f64,
	},
	/// A chain link references an instance that does not exist.
	ChainIndexOutOfRange {
		/// Link name.
		name: String,
		/// The offending index.
		index: usize,
		/// How many poses exist.
		poses: usize,
	},
	/// A chain link's endpoints project to (nearly) the same point on the
	/// chain axis, so its sign cannot be determined.
	DegenerateChainLink {
		/// Link name.
		name: String,
		/// The measured projection (mm).
		projection: f64,
		/// The chain axis used.
		axis: [f64; 3],
	},
	/// The chain axis has (nearly) zero length.
	ZeroChainAxis {
		/// The axis as given.
		axis: [f64; 3],
	},
}

impl std::fmt::Display for ToleranceError {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		match self {
			ToleranceError::NonFinite { name, field, value } => write!(
				f,
				"tolerance: contribution '{name}' has a non-finite {field} ({value}) — a stack cannot accumulate a value that is \
				 not a number"
			),
			ToleranceError::NegativeBand { name, plus, minus } => write!(
				f,
				"tolerance: contribution '{name}' has a negative band (+{plus}/-{minus}); plus and minus are non-negative \
				 MAGNITUDES (a +0.2/-0.05 dimension is plus=0.2, minus=0.05). A negative band would shrink the accumulated \
				 variation — refused"
			),
			ToleranceError::EmptyStack { stack } => {
				write!(f, "tolerance: stack '{stack}' has no contributions — there is nothing to accumulate")
			}
			ToleranceError::BadSigmaLevel { sigma_level } => write!(
				f,
				"tolerance: RSS sigma level {sigma_level} is not a positive finite number — an RSS band without a meaningful sigma \
				 level carries no statistical statement at all (use {DEFAULT_SIGMA_LEVEL} unless you have measured evidence \
				 otherwise)"
			),
			ToleranceError::ChainIndexOutOfRange { name, index, poses } => {
				write!(f, "tolerance: chain link '{name}' references instance {index} but only {poses} pose(s) exist")
			}
			ToleranceError::DegenerateChainLink { name, projection, axis } => write!(
				f,
				"tolerance: chain link '{name}' projects {projection:.3e} mm onto axis [{:.4}, {:.4}, {:.4}] — its two ends sit at \
				 the same station on the chain axis, so the link has no direction and its sign cannot be determined. Declare it by \
				 hand with an explicit Sign, or pick a chain axis the link actually runs along",
				axis[0], axis[1], axis[2]
			),
			ToleranceError::ZeroChainAxis { axis } => write!(
				f,
				"tolerance: chain axis [{:.4}, {:.4}, {:.4}] has zero length — a 1-D stack needs a direction to accumulate along",
				axis[0], axis[1], axis[2]
			),
		}
	}
}

impl std::error::Error for ToleranceError {}

/// An ordered chain of [`Contribution`]s accumulating along one direction.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Stack {
	/// The stack's name (appears in every result and refusal).
	pub name: String,
	/// The chain, in order.
	pub contributions: Vec<Contribution>,
}

impl Stack {
	/// An empty named stack.
	pub fn new(name: impl Into<String>) -> Self {
		Self { name: name.into(), contributions: Vec::new() }
	}

	/// Append a contribution (builder form).
	pub fn with(mut self, c: Contribution) -> Self {
		self.contributions.push(c);
		self
	}

	/// Append a contribution.
	pub fn push(&mut self, c: Contribution) {
		self.contributions.push(c);
	}

	/// Number of links.
	pub fn len(&self) -> usize {
		self.contributions.len()
	}

	/// Whether the chain is empty.
	pub fn is_empty(&self) -> bool {
		self.contributions.is_empty()
	}

	/// Refuse a malformed stack: empty, non-finite, or a negative band.
	///
	/// [`Stack::worst_case`] / [`Stack::rss`] are pure arithmetic and do NOT
	/// call this (they return exactly what the contributions say, NaN
	/// included); [`Stack::gate`], [`Stack::gate_method`] and
	/// [`Stack::gate_report`] do, so the campaign-facing path always refuses
	/// loudly rather than gating on a nonsense number.
	pub fn validate(&self) -> Result<(), ToleranceError> {
		if self.contributions.is_empty() {
			return Err(ToleranceError::EmptyStack { stack: self.name.clone() });
		}
		for c in &self.contributions {
			c.dim.validate(&c.name)?;
		}
		Ok(())
	}

	/// **Worst-case** accumulation: arithmetic sum of extremes.
	///
	/// `max = Σ sign·nominal + Σ up`, `min = Σ sign·nominal − Σ down`, where a
	/// link's `up`/`down` are its `plus`/`minus` for an adding link and
	/// `minus`/`plus` for a subtracting one — so asymmetric bands mirror
	/// correctly under a sign flip.
	///
	/// A hard bound: no conforming set of parts can land outside it.
	///
	/// Pure arithmetic — it does not validate. A negative band or a non-finite
	/// value flows straight into the result; call [`Stack::validate`] (or use
	/// a `gate*` method, which does) for the typed refusal.
	pub fn worst_case(&self) -> StackResult {
		self.result(StackMethod::WorstCase)
	}

	/// **Root-sum-square** accumulation at the stated sigma level.
	///
	/// Band `= sqrt(Σ half_bandᵢ²)` about the mid-shifted centre `Σ sign·mid`
	/// (the standard asymmetric treatment: a `+p/−m` band is converted to the
	/// equivalent bilateral `±(p+m)/2` about `nominal + (p−m)/2`).
	///
	/// `sigma_level` is an **assumption tag, not a scale factor**: it declares
	/// that each input band is a `sigma_level`·σ interval, from which it
	/// follows that the accumulated band is a `sigma_level`·σ interval too
	/// (the level cancels through σᵢ = halfᵢ/k, σ = sqrt(Σσᵢ²), band = k·σ).
	/// Changing it does not change the number — it changes what the number
	/// *means*, which is why it is carried in [`StackResult::assumption`]
	/// rather than silently defaulted. Use [`DEFAULT_SIGMA_LEVEL`] unless you
	/// have measured evidence otherwise.
	///
	/// **Ordering, and its one exception.** For a stack of symmetric
	/// contributions the RSS interval always sits inside the worst-case
	/// interval and always contains the nominal, i.e.
	/// `worst_case ⊇ rss ∋ nominal`. Both intervals share the same centre
	/// (`mean`), and `sqrt(Σh²) ≤ Σh` gives the first containment. The second
	/// can FAIL for strongly asymmetric bands: the mid-shift moves the centre
	/// away from the nominal by `Σ(plus−minus)/2`, which the RSS band does not
	/// have to cover (two `+1/−0` links put the nominal 1.0 below a mean whose
	/// RSS band is only 0.707 wide). Worst case always contains the nominal.
	/// Both facts are pinned in `tests/tolerance.rs`.
	///
	/// Pure arithmetic, exactly as [`Stack::worst_case`]; a non-positive
	/// `sigma_level` is refused by the `gate*` methods, not here.
	pub fn rss(&self, sigma_level: f64) -> StackResult {
		self.result(StackMethod::Rss { sigma_level })
	}

	/// Accumulate under an explicit [`StackMethod`].
	pub fn result(&self, method: StackMethod) -> StackResult {
		let nominal: f64 = self.contributions.iter().map(Contribution::signed_nominal).sum();
		let mean: f64 = self.contributions.iter().map(Contribution::signed_mid).sum();
		let half_band: f64 = match method {
			StackMethod::WorstCase => self.contributions.iter().map(Contribution::half_band).sum(),
			StackMethod::Rss { .. } => self.contributions.iter().map(|c| c.half_band() * c.half_band()).sum::<f64>().sqrt(),
		};
		let mut contributors: Vec<ContributorShare> = self
			.contributions
			.iter()
			.map(|c| {
				let h = c.half_band();
				let contribution = match method {
					StackMethod::WorstCase => h,
					StackMethod::Rss { .. } => {
						if half_band > 0.0 {
							h * h / half_band
						} else {
							0.0
						}
					}
				};
				let fraction = if half_band > 0.0 { contribution / half_band } else { 0.0 };
				ContributorShare {
					name: c.name.clone(),
					sign: c.sign,
					source: c.source.clone(),
					nominal: c.dim.nominal,
					half_band: h,
					contribution,
					fraction,
				}
			})
			.collect();
		contributors.sort_by(|a, b| {
			b.contribution.partial_cmp(&a.contribution).unwrap_or(std::cmp::Ordering::Equal).then_with(|| a.name.cmp(&b.name))
		});
		StackResult {
			stack: self.name.clone(),
			method,
			assumption: method.assumption(),
			nominal,
			mean,
			min: mean - half_band,
			max: mean + half_band,
			span: 2.0 * half_band,
			half_band,
			contributors,
		}
	}

	/// **The campaign gate.** Does the accumulated variation fit inside the
	/// required window `[target.lower(), target.upper()]`?
	///
	/// Uses [`StackMethod::WorstCase`] — the conservative default, and the
	/// only method that is a hard bound. Use [`Stack::gate_method`] to gate on
	/// RSS instead (and then say so wherever the result is quoted).
	///
	/// The refusal names the dominant contributor: the one dimension that owns
	/// the largest share of the accumulated band.
	// `StackViolation` is deliberately fat: it carries the achieved interval,
	// the required window, the method's full statistical statement and the
	// ranked dominant contributor BY VALUE, so a refusal survives being
	// serialized into a campaign receipt with nothing to look up. Boxing it
	// would hide the diagnosis behind an allocation for no benefit on a path
	// that only runs when a gate fails.
	#[allow(clippy::result_large_err)]
	pub fn gate(&self, target: Dimension) -> Result<(), StackViolation> {
		self.gate_method(StackMethod::WorstCase, target)
	}

	/// [`Stack::gate`] under an explicit method. Refuses a malformed stack or
	/// a meaningless sigma level before it computes anything.
	#[allow(clippy::result_large_err)] // see `Stack::gate`
	pub fn gate_method(&self, method: StackMethod, target: Dimension) -> Result<(), StackViolation> {
		if let Err(e) = method.validate().and_then(|()| self.validate()) {
			return Err(StackViolation {
				stack: self.name.clone(),
				method,
				assumption: method.assumption(),
				required_min: target.lower(),
				required_max: target.upper(),
				achieved_min: f64::NAN,
				achieved_max: f64::NAN,
				low_excess: f64::NAN,
				high_excess: f64::NAN,
				dominant: None,
				malformed: Some(e.to_string()),
				message: format!("stack '{}' cannot be gated: {e}", self.name),
			});
		}
		let r = self.result(method);
		let (lo, hi) = (target.lower(), target.upper());
		let low_excess = (lo - r.min).max(0.0);
		let high_excess = (r.max - hi).max(0.0);
		if low_excess == 0.0 && high_excess == 0.0 {
			return Ok(());
		}
		let dominant = r.dominant().cloned();
		let blame = match &dominant {
			Some(d) => format!(
				"dominant contributor '{}' ({}{:.4} +/-{:.4}, {}) owns {:.1}% of the {:.4} mm accumulated band — fix that one first",
				d.name,
				d.sign.symbol(),
				d.nominal,
				d.half_band,
				d.source.label(),
				100.0 * d.fraction,
				r.span
			),
			None => "no contributors".to_string(),
		};
		let message = format!(
			"stack '{}' [{}] violates its target: accumulated [{:.4}, {:.4}] (nominal {:.4}, mean {:.4}, span {:.4}) must fit \
			 inside required [{:.4}, {:.4}] — undershoot {:.4}, overshoot {:.4}. {blame}. Method assumption: {}",
			self.name,
			method.name(),
			r.min,
			r.max,
			r.nominal,
			r.mean,
			r.span,
			lo,
			hi,
			low_excess,
			high_excess,
			r.assumption
		);
		Err(StackViolation {
			stack: self.name.clone(),
			method,
			assumption: r.assumption.clone(),
			required_min: lo,
			required_max: hi,
			achieved_min: r.min,
			achieved_max: r.max,
			low_excess,
			high_excess,
			dominant,
			malformed: None,
			message,
		})
	}

	/// The full gate answer: the accumulated result, the aggregate verdict,
	/// and **whether each link would have passed alone**.
	///
	/// [`GateReport::aggregate_only_failure`] is the flag worth gating a
	/// campaign on: it means every individual fit review would have said yes
	/// and the assembly still does not close.
	pub fn gate_report(&self, method: StackMethod, target: Dimension) -> GateReport {
		let aggregate = self.gate_method(method, target).err();
		let budget = target.half_band();
		let links_passing_alone: Vec<(String, bool)> =
			self.contributions.iter().map(|c| (c.name.clone(), c.half_band() <= budget)).collect();
		let failing_alone = links_passing_alone.iter().filter(|(_, ok)| !ok).count();
		let aggregate_only_failure = aggregate.is_some() && failing_alone == 0;
		let verdict = match (&aggregate, failing_alone) {
			(None, _) => "pass".to_string(),
			(Some(_), 0) => "fail (aggregate only — every link passes alone)".to_string(),
			(Some(_), n) => format!("fail ({n} link(s) also fail alone)"),
		};
		GateReport { result: self.result(method), target, aggregate, links_passing_alone, aggregate_only_failure, verdict }
	}

	/// Build a stack by reading each link's **nominal out of solved poses**
	/// instead of re-typing it — the mate-chain constructor.
	///
	/// Each [`ChainLink`] names two instances; its nominal is the distance
	/// between their pose origins projected onto `axis`, and its [`Sign`] is
	/// the projection's sign, so the chain's direction comes from the geometry
	/// rather than from a hand-entered `+`/`−` (the classic mis-sign cannot
	/// happen here). Feed it
	/// [`crate::constraints::ConstraintSystem::transforms`] after `solve` to
	/// build the stack of a mated assembly, or [`Stack::from_assembly_chain`]
	/// for an [`Assembly`] directly.
	///
	/// Refuses loudly on: a zero-length axis, an out-of-range instance index,
	/// or a link whose two ends sit at the same station on the axis (no
	/// direction ⇒ no sign).
	///
	/// **Precision limit:** instance poses are `f32` [`Affine3A`], so nominals
	/// read this way carry ~1e-6 relative error. Tolerances are supplied by
	/// the caller in `f64` and are unaffected.
	pub fn from_pose_chain(name: impl Into<String>, poses: &[Affine3A], axis: DVec3, links: &[ChainLink]) -> Result<Stack, ToleranceError> {
		let axis_arr = [axis.x, axis.y, axis.z];
		if axis.length() < CHAIN_MIN_PROJECTION {
			return Err(ToleranceError::ZeroChainAxis { axis: axis_arr });
		}
		let u = axis.normalize();
		let mut stack = Stack::new(name);
		for link in links {
			for idx in [link.from, link.to] {
				if idx >= poses.len() {
					return Err(ToleranceError::ChainIndexOutOfRange { name: link.name.clone(), index: idx, poses: poses.len() });
				}
			}
			let origin = |i: usize| {
				let t = poses[i].translation;
				DVec3::new(f64::from(t.x), f64::from(t.y), f64::from(t.z))
			};
			let d = (origin(link.to) - origin(link.from)).dot(u);
			if d.abs() < CHAIN_MIN_PROJECTION {
				return Err(ToleranceError::DegenerateChainLink { name: link.name.clone(), projection: d, axis: axis_arr });
			}
			let sign = if d > 0.0 { Sign::Adds } else { Sign::Subtracts };
			let dim = Dimension::asymmetric(d.abs(), link.plus, link.minus);
			dim.validate(&link.name)?;
			stack.push(Contribution::declared(link.name.clone(), sign, dim));
		}
		stack.validate()?;
		Ok(stack)
	}

	/// [`Stack::from_pose_chain`] over an [`Assembly`]'s instance poses.
	pub fn from_assembly_chain(
		name: impl Into<String>,
		assembly: &Assembly,
		axis: DVec3,
		links: &[ChainLink],
	) -> Result<Stack, ToleranceError> {
		let poses: Vec<Affine3A> = assembly.instances.iter().map(|i| i.pose).collect();
		Stack::from_pose_chain(name, &poses, axis, links)
	}
}
