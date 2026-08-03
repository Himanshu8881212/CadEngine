// Copyright (c) LMCAD. Licensed under the MIT License.

//! Honest first-order mechanical rating helpers.
//!
//! These formalize the hand calculations used to size the printed drive
//! trains (CYCLO-26 / HARM-26 / PLAN-26): tooth bending capacity, flexspline
//! wall strain, cross-pin bending, and 1D tolerance stackups. Every function
//! names its formula and the assumptions baked into it. They are conservative
//! FIRST-ORDER SCREENS for printed plastic parts — not AGMA/ISO certified
//! ratings, and they say so.
//!
//! Unit convention: N, mm, MPa (1 MPa · mm² = 1 N) — angles never appear.

/// Lewis tooth-bending capacity: **F = σ_allow · b · m · Y**, in newtons.
///
/// Formula: Lewis parabolic-beam gear-tooth bending (Wilfred Lewis, 1892),
/// metric-module form with the tabulated form factor `Y` (see
/// [`lewis_form_factor`]). `sigma_allow_mpa` is the allowable bending stress,
/// `face_width_mm` = `b`, `module_mm` = `m`.
///
/// Assumptions: static load applied at the tooth tip, a single tooth pair
/// carrying the whole load, and NO dynamic factor (Kv), stress-concentration
/// factor (Kf) or load-distribution factor (Km) — i.e. a conservative
/// screening number for slow printed gears, not an AGMA 2001 rating.
/// Units check: MPa · mm · mm = N.
pub fn lewis_tooth_load(sigma_allow_mpa: f64, face_width_mm: f64, module_mm: f64, form_factor_y: f64) -> f64 {
	sigma_allow_mpa * face_width_mm * module_mm * form_factor_y
}

/// Conservative Lewis form factor `Y` for a spur pinion with `teeth` teeth.
///
/// Table provenance: conservative small-pinion values, Barth/Lewis handbook
/// class (load at the tip, full-depth involute). Rows: 11T → 0.30 (flagged
/// conservative — an 11T pinion is at the undercut edge and this is a floor,
/// not a rating), 12 → 0.31, 14 → 0.33, 17 → 0.35, 25 → 0.40, 40+ → 0.45.
/// Linear interpolation between rows; below 11 teeth clamps to 0.30, 40 and
/// above returns 0.45.
pub fn lewis_form_factor(teeth: usize) -> f64 {
	const TABLE: [(f64, f64); 6] = [(11.0, 0.30), (12.0, 0.31), (14.0, 0.33), (17.0, 0.35), (25.0, 0.40), (40.0, 0.45)];
	let t = teeth as f64;
	if t <= TABLE[0].0 {
		return TABLE[0].1;
	}
	for w in TABLE.windows(2) {
		let ((t0, y0), (t1, y1)) = (w[0], w[1]);
		if t <= t1 {
			return y0 + (y1 - y0) * (t - t0) / (t1 - t0);
		}
	}
	TABLE[TABLE.len() - 1].1
}

/// Peak bending strain of a thin ring driven in a two-lobe radial wave:
/// **ε = (t/2) · 3·w0 / rn²** (dimensionless; mm in, mm out).
///
/// Formula: classic thin-ring bending (Timoshenko, *Strength of Materials*
/// thin-ring class) for the inextensible field `w = w0·cos 2φ`: the change of
/// curvature `Δκ = (w + w″)/rn²` peaks at `3·w0/rn²`, and the outer-fibre
/// bending strain is `(t/2)·Δκ` with `t = wall_mm`, `rn = neutral_radius_mm`
/// (mid-wall), `w0 = w0_mm` the radial wave amplitude.
///
/// Assumptions: t ≪ rn (thin ring), neutral axis at mid-wall, pure ring
/// bending — no cup/shell end effects from a real flexspline body, no tooth
/// stiffening. Used to check a printed flexspline wall stays inside the
/// material's flexural fatigue strain.
pub fn thin_ring_bending_strain(wall_mm: f64, w0_mm: f64, neutral_radius_mm: f64) -> f64 {
	(wall_mm / 2.0) * 3.0 * w0_mm / (neutral_radius_mm * neutral_radius_mm)
}

/// Peak bending stress in a round pin of diameter `dia_mm` under transverse
/// load `force_n` over length `length_mm`: **σ = 32·M / (π·d³)** in MPa.
///
/// Formula: Euler–Bernoulli bending of a solid circular section
/// (section modulus `π·d³/32`), with
/// - `M = F·L` for a plain cantilever, load at the free tip
///   (`both_ends_guided = false`);
/// - `M = F·L/8` for a pin built in at BOTH ends with the load at mid-span
///   (`both_ends_guided = true`; fixed–fixed central point load, Roark class)
///   — the case of a cross-pin captured in two walls.
///
/// Assumptions: static load, straight prismatic pin, no root-fillet stress
/// concentration, shear deflection ignored. Units: N·mm / mm³ = MPa.
pub fn cantilever_bending_stress(force_n: f64, length_mm: f64, dia_mm: f64, both_ends_guided: bool) -> f64 {
	let m = if both_ends_guided { force_n * length_mm / 8.0 } else { force_n * length_mm };
	32.0 * m / (std::f64::consts::PI * dia_mm.powi(3))
}

/// One-dimensional tolerance stackup: push `(nominal, ±tol)` contributions,
/// then read the total two ways.
///
/// - [`Stackup::worst_case`] → `(Σ nominal, Σ |tol|)`: every contributor
///   simultaneously at its limit — a guaranteed bound (use for
///   fit/interference GO decisions).
/// - [`Stackup::rss`] → `(Σ nominal, √(Σ tol²))`: root-sum-square, the
///   standard statistical stack. Assumes independent, centred contributor
///   distributions; if each `±tol` is a 3σ band the RSS band is ~3σ of the
///   stack — honest only when contributors really are independent.
///
/// # Example — does a bearing stack fit its 12.7 mm pocket?
/// ```
/// use kernel_model::rate::Stackup;
/// let mut s = Stackup::new();
/// s.push(7.0, 0.10); // 607ZZ bearing width
/// s.push(4.5, 0.05); // printed spacer
/// s.push(1.0, 0.02); // steel shim
/// let (nom, wc) = s.worst_case(); // 12.5 ± 0.17 — still fits at worst case
/// let (_, rss) = s.rss();         // 12.5 ± ~0.114 — statistical band
/// assert!(nom + wc < 12.7 && rss < wc);
/// ```
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Stackup {
	items: Vec<(f64, f64)>,
}

impl Stackup {
	/// New, empty stackup.
	pub fn new() -> Self {
		Self::default()
	}

	/// Add one contribution: `nominal ± tol` (the tolerance is stored as
	/// `|tol|`; a stackup has no signed tolerances).
	pub fn push(&mut self, nominal: f64, tol: f64) {
		self.items.push((nominal, tol.abs()));
	}

	/// Number of contributions pushed so far.
	pub fn len(&self) -> usize {
		self.items.len()
	}

	/// True when nothing has been pushed.
	pub fn is_empty(&self) -> bool {
		self.items.is_empty()
	}

	/// Worst-case total: `(Σ nominal, Σ tol)` — arithmetic tolerance sum,
	/// every contributor at its limit simultaneously. Guaranteed bound.
	pub fn worst_case(&self) -> (f64, f64) {
		(self.items.iter().map(|&(n, _)| n).sum(), self.items.iter().map(|&(_, t)| t).sum())
	}

	/// Root-sum-square total: `(Σ nominal, √(Σ tol²))` — statistical stack
	/// for independent, centred contributors (see the type-level docs for
	/// the honesty caveats).
	pub fn rss(&self) -> (f64, f64) {
		(self.items.iter().map(|&(n, _)| n).sum(), self.items.iter().map(|&(_, t)| t * t).sum::<f64>().sqrt())
	}
}
