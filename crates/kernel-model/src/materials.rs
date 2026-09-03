// Copyright (c) LMCAD. Licensed under the MIT License.

//! Print-material densities in g/mm³ — the constants every campaign example
//! used to re-declare. Multiply an engine volume (mm³) by one of these for a
//! solid-equivalent mass in grams; slicer infill scales it down from there.

/// PLA, 1.24 g/cm³.
pub const PLA_G_PER_MM3: f64 = 0.00124;
/// PETG, 1.27 g/cm³.
pub const PETG_G_PER_MM3: f64 = 0.00127;
/// ABS, 1.05 g/cm³.
pub const ABS_G_PER_MM3: f64 = 0.00105;
/// ASA, 1.07 g/cm³.
pub const ASA_G_PER_MM3: f64 = 0.00107;
/// Polycarbonate, 1.20 g/cm³.
pub const PC_G_PER_MM3: f64 = 0.00120;

/// Printed-PLA structural design allowables — the derating chain the
/// RESPOOL campaign established and FEA cross-checked (its lug-pull case
/// reproduced the τ design point): base tensile 35 MPa (low end of
/// published data) × 0.6 layer adhesion × 0.5 design factor → 10 MPa;
/// shear 0.58·σ; the HOT tier sits just under PLA's own HDT
/// (54 °C @ 1.8 MPa, Bambu TDS) because a loaded part in a filament
/// dryer lives there for hours.
pub mod pla {
	/// Design tension/bearing at 20 °C, MPa.
	pub const SIG_ALLOW_RT: f64 = 10.0;
	/// Design shear at 20 °C, MPa.
	pub const TAU_ALLOW_RT: f64 = 6.0;
	/// Sustained tension/bearing at 50 °C (near-HDT derate), MPa.
	pub const SIG_ALLOW_HOT: f64 = 2.5;
	/// Sustained shear at 50 °C, MPa.
	pub const TAU_ALLOW_HOT: f64 = 1.5;

	// -- Time-dependent (creep) allowables, 2026-07-30 research wave ------
	//
	// The constants above are STATIC design points. They do NOT describe a
	// part held under load for weeks — printed PLA creeps, and creep, not
	// instantaneous strength, is what actually kills a sustained-load part
	// (a spool sitting loaded in a warm dryer, a wall bracket carrying
	// books for a year). The table below is the Rust mirror of the
	// researched block in `tools/materials/pla.json` (full derivation
	// chain, per-cell confidence and sources live there — read it before
	// designing against these).

	/// Tabulated temperature tiers (°C) of [`CREEP_SIG_ALLOW_MPA`].
	pub const CREEP_TEMPS_C: [f64; 2] = [23.0, 55.0];

	/// Tabulated durations (hours) of [`CREEP_SIG_ALLOW_MPA`]:
	/// 1 h, 24 h, 30 d, 1 y.
	pub const CREEP_HOURS: [f64; 4] = [1.0, 24.0, 720.0, 8760.0];

	/// Sustained tension allowables, MPa, `[temperature tier][duration]`
	/// — printed (FDM) PLA, in-plane loading, unannealed, dry, constant
	/// load. Built conservatively: safety factor 2.0 on the worst measured
	/// printed creep-rupture, time-derated from the only quantified
	/// printed creep-compliance history, all cells rounded DOWN.
	///
	/// **Honesty note carried from the source data**: the 55 °C / 30 d and
	/// 55 °C / 1 y cells are *bounds, not measurements* — no experiment
	/// supports any sustained allowable above ~0.5 MPa there. Read them as
	/// "do not design sustained load into unannealed PLA at 55 °C".
	pub const CREEP_SIG_ALLOW_MPA: [[f64; 4]; 2] = [
		[7.5, 5.0, 3.5, 2.5], // 23 °C
		[3.0, 1.5, 0.5, 0.5], // 55 °C
	];

	/// Across-layer (Z) strength ratio for printed PLA — layer adhesion is
	/// the weak axis, so a load pulling ACROSS layers gets this factor on
	/// top of any allowable ([`creep_allowable_mpa`] reports the in-plane
	/// value; multiply yourself and say so in the analysis).
	pub const Z_VS_XY_STRENGTH_RATIO: f64 = 0.55;

	/// Sustained (creep) tension allowable in MPa for a load held at
	/// `temp_c` for `hours`, **in-plane** — the number to gate a
	/// sustained-load design against instead of [`SIG_ALLOW_RT`].
	///
	/// Conservative by construction: the lookup rounds the temperature UP
	/// to the next tabulated tier and the duration UP to the next
	/// tabulated column, so an in-between request never reads a rosier
	/// cell than the data supports.
	///
	/// Returns **0.0** — i.e. "no sustained load is defensible" — above the
	/// hot tier (55 °C, mid-glass-transition for PLA) and for non-finite
	/// or negative input. A gate written as `stress <= creep_allowable_mpa(..)`
	/// therefore FAILS loudly in exactly the regime where no data exists,
	/// which is the intended behavior.
	///
	/// Beyond 1 year the last column is reused; the source block flags
	/// that cell as an extrapolation bound, so state the duration you
	/// designed for in the analysis.
	pub fn creep_allowable_mpa(temp_c: f64, hours: f64) -> f64 {
		creep_lookup(temp_c, hours, false).sig_allow_mpa
	}

	/// How the cell behind an allowable was reached. The table is a COARSE
	/// STEP (two temperature tiers, nothing between), so "which cell was
	/// this margin read at" is the whole question — a campaign that writes
	/// "gated against creep_allowable_mpa(23 C, 1 year)" while its declared
	/// ambient is 25 °C has silently designed to a temperature it does not
	/// hold. This makes the answer a value instead of a sentence.
	///
	/// The string forms are the SAME strings `tools/materials.py` puts in
	/// its receipts, so a Python gate and a Rust gate match on one
	/// vocabulary.
	#[derive(Debug, Clone, Copy, PartialEq, Eq)]
	pub enum CreepCellMatch {
		/// Both the temperature tier and the duration column were hit exactly.
		Exact,
		/// One or both axes were rounded UP to the next tabulated cell, so
		/// the allowable read is the WORSE (conservative) one.
		RoundedUpConservative,
		/// The request is longer than the last tabulated duration; the last
		/// column is reused, as the source record's own bound directs.
		ExtrapolatedBeyondLastDuration,
		/// No cell was read at all — see [`CreepCell::refusal`].
		Refused,
	}

	impl CreepCellMatch {
		pub fn as_str(self) -> &'static str {
			match self {
				CreepCellMatch::Exact => "exact",
				CreepCellMatch::RoundedUpConservative => "rounded_up_conservative",
				CreepCellMatch::ExtrapolatedBeyondLastDuration => "extrapolated_beyond_last_duration",
				CreepCellMatch::Refused => "refused",
			}
		}
	}

	/// Machine-matchable reason a creep lookup REFUSED. Identical slugs to
	/// `tools/materials.CREEP_REFUSAL_KINDS`.
	#[derive(Debug, Clone, Copy, PartialEq, Eq)]
	pub enum CreepRefusal {
		/// Temperature or duration was NaN / infinite.
		InputNotFinite,
		/// Duration was negative.
		NegativeDuration,
		/// Temperature is above the hottest tabulated tier. There is NO
		/// fallback to the hot row — no data supports one there.
		TempAboveTabulated,
	}

	impl CreepRefusal {
		pub fn as_str(self) -> &'static str {
			match self {
				CreepRefusal::InputNotFinite => "creep_input_not_finite",
				CreepRefusal::NegativeDuration => "creep_negative_duration",
				CreepRefusal::TempAboveTabulated => "creep_temp_above_tabulated",
			}
		}
	}

	/// A receipted sustained-allowable lookup: the number PLUS the cell it
	/// came from and how it was reached.
	#[derive(Debug, Clone, Copy, PartialEq)]
	pub struct CreepCell {
		pub temp_c_requested: f64,
		pub hours_requested: f64,
		/// The allowable to gate against, MPa. **0.0 on refusal**, so
		/// `demand <= sig_allow_mpa` fails loudly there.
		pub sig_allow_mpa: f64,
		/// The tabulated (in-plane) cell value before any anisotropy derate.
		pub in_plane_mpa: f64,
		/// Temperature tier actually read, °C (`None` on refusal).
		pub row_used_c: Option<f64>,
		/// Duration column actually read, hours (`None` on refusal).
		pub col_used_h: Option<f64>,
		pub cell_match: CreepCellMatch,
		pub refusal: Option<CreepRefusal>,
		/// Whether the caller asked for the across-layer derate. Never
		/// applied silently — the tabulated cells are IN-PLANE.
		pub across_layer: bool,
		/// 1.0, or [`Z_VS_XY_STRENGTH_RATIO`] when `across_layer`.
		pub anisotropy_factor: f64,
	}

	impl CreepCell {
		pub fn refused(&self) -> bool {
			self.refusal.is_some()
		}
	}

	/// Receipted sustained (creep) allowable. Same lookup rule and same
	/// number as [`creep_allowable_mpa`], but it also reports WHICH cell was
	/// read and how — exact, rounded up, extrapolated, or refused.
	///
	/// `across_layer` applies [`Z_VS_XY_STRENGTH_RATIO`] to the allowable
	/// (never to E). It is the caller's explicit choice, recorded on the
	/// result; `creep_allowable_mpa` is the in-plane form.
	pub fn creep_lookup(temp_c: f64, hours: f64, across_layer: bool) -> CreepCell {
		let factor = if across_layer { Z_VS_XY_STRENGTH_RATIO } else { 1.0 };
		let refuse = |why: CreepRefusal| CreepCell {
			temp_c_requested: temp_c,
			hours_requested: hours,
			sig_allow_mpa: 0.0,
			in_plane_mpa: 0.0,
			row_used_c: None,
			col_used_h: None,
			cell_match: CreepCellMatch::Refused,
			refusal: Some(why),
			across_layer,
			anisotropy_factor: factor,
		};
		if !temp_c.is_finite() || !hours.is_finite() {
			return refuse(CreepRefusal::InputNotFinite);
		}
		if hours < 0.0 {
			return refuse(CreepRefusal::NegativeDuration);
		}
		if temp_c > CREEP_TEMPS_C[CREEP_TEMPS_C.len() - 1] {
			return refuse(CreepRefusal::TempAboveTabulated);
		}
		// Round the temperature UP to the next tabulated tier.
		let row = CREEP_TEMPS_C.iter().position(|t| *t >= temp_c).unwrap_or(CREEP_TEMPS_C.len() - 1);
		// Round the duration UP to the next tabulated column; beyond the
		// last column, reuse it (flagged).
		let beyond_last = hours > CREEP_HOURS[CREEP_HOURS.len() - 1];
		let col = CREEP_HOURS.iter().position(|h| *h >= hours).unwrap_or(CREEP_HOURS.len() - 1);
		let in_plane = CREEP_SIG_ALLOW_MPA[row][col];
		let cell_match = if beyond_last {
			CreepCellMatch::ExtrapolatedBeyondLastDuration
		} else if CREEP_TEMPS_C[row] == temp_c && CREEP_HOURS[col] == hours {
			CreepCellMatch::Exact
		} else {
			CreepCellMatch::RoundedUpConservative
		};
		CreepCell {
			temp_c_requested: temp_c,
			hours_requested: hours,
			sig_allow_mpa: in_plane * factor,
			in_plane_mpa: in_plane,
			row_used_c: Some(CREEP_TEMPS_C[row]),
			col_used_h: Some(CREEP_HOURS[col]),
			cell_match,
			refusal: None,
			across_layer,
			anisotropy_factor: factor,
		}
	}

	/// Sustained (creep) SHEAR allowable, MPa — [`creep_allowable_mpa`]
	/// scaled by the same 0.6 shear ratio the static tier uses
	/// (`TAU_ALLOW_RT / SIG_ALLOW_RT` = `TAU_ALLOW_HOT / SIG_ALLOW_HOT` =
	/// 0.6). No independent printed-PLA creep-shear dataset was found, so
	/// this is a derived number, not a measured one — say so when you cite
	/// it.
	pub fn creep_shear_allowable_mpa(temp_c: f64, hours: f64) -> f64 {
		0.6 * creep_allowable_mpa(temp_c, hours)
	}
}
