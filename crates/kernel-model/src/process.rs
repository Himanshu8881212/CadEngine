// Copyright (c) LMCAD. Licensed under the MIT License.

//! Manufacturing **process profiles** — the engine is a *making* engine, not a
//! printing engine, and this module is where a manufacturing process's measured
//! reality lives as data instead of being frozen into each campaign as consts.
//!
//! # Roadmap (honest status, 2026-07-30)
//!
//! | process | status |
//! |---|---|
//! | FDM | **implemented**: [`FdmProfile`] (serde JSON in `profiles/`), fit/DFM helpers, measured-coupon calibration path (the `calibrate_fdm.rs` example — removed from the tree 2026-09-03, git history at `5a70984` — prints coupons → calipers → `tools/ingest_calibration.py` → `profiles/<printer>.json`) |
//! | sheet metal | declared sibling, **NOT implemented** — no bend allowance / K-factor / min-flange model yet; every entry point refuses loudly |
//! | casting | declared sibling, **NOT implemented** as a profile — but the core castability check already exists as [`kernel_brep::draft_analysis`] (per-face draft angles + undercut detection); the refusal message points there |
//! | CNC | declared sibling, **NOT implemented** — no tool-access / internal-corner-radius model yet; refuses loudly |
//!
//! Siblings refuse with a clear [`ProcessError::NotImplemented`] instead of
//! silently returning defaults: a caller can enumerate [`Process`] variants
//! today and will get honest errors, not fake profiles.
//!
//! # Where the conservative FDM numbers come from
//!
//! [`FdmProfile::conservative_default`] freezes the values the shipped
//! campaigns (`respool.rs`, `drybox_roller.rs` — the modern pre-JSON
//! references, removed from the tree 2026-09-03) proved
//! in print, so a campaign that has no measured printer profile inherits
//! exactly the numbers that already survived physical validation. Each field's
//! doc comment cites its source const. A **measured** profile produced by
//! `tools/ingest_calibration.py` from the `calibrate_fdm` coupon set replaces
//! those numbers with the user's own printer reality.
//!
//! # Gate-consumption API
//!
//! Campaigns call the fit helpers instead of re-typing clearance literals:
//!
//! ```
//! use kernel_model::process::FdmProfile;
//! let p = FdmProfile::conservative_default();
//! // RESPOOL froze `R_TO = RI - C_R` with C_R = 0.25. The profile-driven
//! // equivalent (identical numbers under the conservative default):
//! let ri = 37.3; // barrel inner radius
//! let r_to = p.fit_free_shaft_r(ri);
//! assert!((r_to - 37.05).abs() < 1e-12);
//! // Designed bore diameter for a free fit over a Ø6 printed pin:
//! assert!((p.fit_free_bore_d(6.0) - 6.5).abs() < 1e-12);
//! ```

use kernel_brep::{tessellate_default, validate, Solid};
use kernel_core::math::Vec3;
use kernel_core::mesh::Mesh;
use serde::{Deserialize, Serialize};

/// Diameter (mm) below which [`FdmProfile::hole_diameter_comp`] applies and at
/// or above which [`FdmProfile::bore_comp`] applies. The coupon set measures
/// the small-hole class on a Ø3–Ø8 ladder and the large-bore class on a Ø22
/// gauge (608 bearing OD); Ø12 is the declared crossover between the two
/// measured regimes.
pub const HOLE_BORE_CROSSOVER_D: f64 = 12.0;

/// A manufacturing process a part may be made by. Only FDM carries an
/// implemented profile today; the siblings are declared so downstream code can
/// route on process *now* and gets a loud [`ProcessError::NotImplemented`]
/// (never a silent stub) until their profiles land. See the
/// [module docs](self) for the roadmap.
#[derive(Clone, Debug)]
pub enum Process {
	/// Fused-deposition 3D printing — implemented, profile-driven.
	Fdm(FdmProfile),
	/// Sheet-metal fabrication — declared sibling, refuses (no bend model yet).
	SheetMetal,
	/// Casting/molding — declared sibling, refuses as a *profile*; the draft/
	/// undercut check that already exists is [`kernel_brep::draft_analysis`].
	Casting,
	/// Subtractive machining — declared sibling, refuses (no tool-access model).
	Cnc,
}

impl Process {
	/// Stable lowercase name of the process (used in messages and file names).
	pub fn name(&self) -> &'static str {
		match self {
			Process::Fdm(_) => "fdm",
			Process::SheetMetal => "sheet_metal",
			Process::Casting => "casting",
			Process::Cnc => "cnc",
		}
	}

	/// The FDM profile, if this process is FDM — the sibling processes refuse
	/// with [`ProcessError::NotImplemented`] naming what *does* exist for them.
	pub fn fdm_profile(&self) -> Result<&FdmProfile, ProcessError> {
		match self {
			Process::Fdm(p) => Ok(p),
			other => Err(ProcessError::not_implemented(other)),
		}
	}

	/// Run this process's design-for-manufacturing checks on a print-posed
	/// solid (build direction `+Z`). Implemented for FDM; siblings refuse
	/// loudly. An empty vec means no violations found.
	pub fn dfm_checks(&self, s: &Solid) -> Result<Vec<DfmFinding>, ProcessError> {
		match self {
			Process::Fdm(p) => Ok(p.dfm_checks(s)),
			other => Err(ProcessError::not_implemented(other)),
		}
	}

	/// [`Process::dfm_checks`] on an already-tessellated, print-posed mesh.
	pub fn dfm_checks_mesh(&self, m: &Mesh) -> Result<Vec<DfmFinding>, ProcessError> {
		match self {
			Process::Fdm(p) => Ok(p.dfm_checks_mesh(m)),
			other => Err(ProcessError::not_implemented(other)),
		}
	}
}

/// Errors from the process layer — including the *honest refusals* of the
/// declared-but-unimplemented sibling processes.
#[derive(Debug)]
pub enum ProcessError {
	/// The process is declared but its profile/checks are not implemented.
	/// `note` names what already exists (e.g. `draft_analysis` for casting).
	NotImplemented {
		/// [`Process::name`] of the refused process.
		process: &'static str,
		/// What exists today for this process, or the empty string.
		note: &'static str,
	},
	/// File I/O failed loading/saving a profile.
	Io {
		/// Path involved.
		path: String,
		/// Underlying error.
		err: std::io::Error,
	},
	/// Profile JSON did not match the schema (unknown/missing field, bad type).
	Schema {
		/// Path (or `"<inline>"` for string parses).
		path: String,
		/// Underlying serde error.
		err: serde_json::Error,
	},
	/// A profile field is outside its physically sane range.
	BadProfile {
		/// Field name.
		field: &'static str,
		/// Offending value.
		got: f64,
		/// The sanity rule it broke.
		why: &'static str,
	},
	/// The profile name is empty or still a placeholder.
	BadName(String),
}

impl ProcessError {
	fn not_implemented(p: &Process) -> ProcessError {
		let note = match p {
			Process::Casting => {
				" — the draft/undercut half already exists as kernel_brep::draft_analysis (per-face draft angles + undercut area); shrinkage/min-wall profiles do not"
			}
			_ => "",
		};
		ProcessError::NotImplemented { process: p.name(), note }
	}
}

impl std::fmt::Display for ProcessError {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		match self {
			ProcessError::NotImplemented { process, note } => {
				write!(
					f,
					"{process} process profile not implemented — declared sibling, see kernel_model::process module doc{note}"
				)
			}
			ProcessError::Io { path, err } => write!(f, "profile I/O failed at '{path}': {err}"),
			ProcessError::Schema { path, err } => {
				write!(f, "profile JSON at '{path}' does not match the FdmProfile schema: {err}")
			}
			ProcessError::BadProfile { field, got, why } => {
				write!(f, "profile field '{field}' = {got} is out of range: {why}")
			}
			ProcessError::BadName(n) => {
				write!(f, "profile name '{n}' is empty or a placeholder — name the printer that was measured")
			}
		}
	}
}

impl std::error::Error for ProcessError {}

/// One design-for-manufacturing violation found by
/// [`FdmProfile::dfm_checks`]. Only violations are reported — an empty
/// finding list means the part passed every implemented check.
#[derive(Clone, Debug)]
pub struct DfmFinding {
	/// Which check fired: `"brep_valid"`, `"watertight"`, `"support_steep"`,
	/// `"bridge_span"`, `"thin_wall"` or `"bed_fit"`.
	pub check: &'static str,
	/// The measured value that broke the limit.
	pub measured: f64,
	/// The profile limit it broke.
	pub limit: f64,
	/// Human-readable context with units and (where available) locations.
	pub detail: String,
}

/// A measured (or conservatively defaulted) FDM printer profile — every
/// dimension the shipped campaigns gate on, as *data*. All lengths mm, angles
/// degrees. Radial vs diametral is stated per field; sign conventions match
/// `tools/ingest_calibration.py` (which writes these files from coupon
/// measurements).
///
/// Serialization: [`FdmProfile::save`]/[`FdmProfile::load`] use serde_json
/// (`float_roundtrip` feature workspace-wide, so every `f64` reloads
/// bit-exactly) with a fixed field order — saving the same profile twice
/// yields identical bytes. Unknown fields in a profile file are a hard error
/// (`deny_unknown_fields`): a typo'd field name must never silently fall back
/// to a default.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FdmProfile {
	/// Which printer/material this profile describes (file stem in
	/// `profiles/`). `"conservative_default"` for the research-derived fallback.
	pub name: String,
	/// RADIAL clearance (mm) for a snug/press hand-fit (insertable by hand,
	/// holds by friction). Conservative source: DRYBOX press-stub — seat
	/// Ø7.9 (`STUB_R = 3.95`) in the 608's Ø8.0 bore = 0.05 radial, the
	/// community-proven click fit. May be slightly negative in a measured
	/// profile (a light interference press); positive = gap.
	pub xy_clearance_tight: f64,
	/// RADIAL clearance (mm) for a free running/sliding fit. Conservative
	/// source: RESPOOL `C_R = 0.25` (tongue-outer ↔ mate-wall-inner, the
	/// bayonet's twist fit), inside DESIGN_GUIDE §22.6's proven 0.2–0.3 band.
	pub xy_clearance_free: f64,
	/// AXIAL (Z) clearance (mm) for mating faces stacked along the build
	/// direction — layer-quantized, so it is a separate number from XY.
	/// Conservative source: RESPOOL `CEIL_CLR = 0.30` (lug retention face ↔
	/// pocket ceiling). Not measured by coupon set v1 (needs a mating pair);
	/// ingest carries this default forward and says so.
	pub z_clearance: f64,
	/// DIAMETRAL compensation (mm) ADDED to a designed small hole (Ø <
	/// [`HOLE_BORE_CROSSOVER_D`]) so it *measures* nominal. Positive = holes
	/// print undersized (the common case). Conservative default 0.0: the
	/// frozen campaigns cut holes at nominal and absorb shrink in designed
	/// clearance (e.g. RESPOOL's Ø2.1 witness holes for Ø1.75 filament);
	/// a measured profile replaces this with the Ø3–Ø8 coupon ladder's mean.
	pub hole_diameter_comp: f64,
	/// DIAMETRAL compensation (mm) for large bores (Ø ≥
	/// [`HOLE_BORE_CROSSOVER_D`]). Same sign convention as
	/// [`hole_diameter_comp`](Self::hole_diameter_comp). Conservative default
	/// 0.0: DRYBOX seats a 608 in an as-designed Ø22-class pocket relying on
	/// the press ring, not scaling. Measured on the Ø22 coupon gauge.
	pub bore_comp: f64,
	/// RADIAL first-layer flare (mm) — elephant foot: how far the first layer
	/// spreads outward past nominal. Budget this on any fit that engages the
	/// first layer, or chamfer it away. Conservative default 0.0 (no frozen
	/// campaign compensates it explicitly; DRYBOX's 0.8/side slider channel
	/// absorbs it silently). Measured on the coupon disc: `(Ø_first_layer −
	/// Ø_mid) / 2`, clamped at 0.
	pub first_layer_comp: f64,
	/// RADIAL seam allowance (mm) — the Z-seam bump on an outer perimeter.
	/// Budgeted ONCE per fit interface by the fit helpers (slicers align
	/// seams; one bump passes one counterface). Conservative default 0.0
	/// (RESPOOL's `C_R` absorbs the seam inside its 0.25). Measured on the
	/// coupon pin: `Ø_max − Ø_min` across the seam.
	pub seam_allowance: f64,
	/// Longest flat bridge (mm) the printer spans cleanly. Conservative
	/// source: RESPOOL's per-part emit gate `max_bridge_span <= 6.0` (DRYBOX
	/// ships 10.5 — the default keeps the tighter frozen bound). Measured on
	/// the 5–25 mm bridge-ladder coupon.
	pub max_bridge: f64,
	/// Steepest printable overhang, in DEGREES FROM VERTICAL (a vertical wall
	/// is 0°). Conservative source: every campaign's
	/// `support_free_report(Z, 45.0, 0.3)` gate. Measured on the overhang-fan
	/// coupon (35–60°).
	pub max_unsupported_angle: f64,
	/// Thinnest wall (mm) that prints solid. Conservative source: DRYBOX
	/// `RIB_T = 1.2`, the thinnest wall a frozen campaign ships. Measured on
	/// the 0.8–2.4 mm wall-ladder coupon.
	pub min_wall: f64,
	/// Usable bed X (mm). Conservative source: the RESPOOL/DRYBOX emit gate
	/// `ext <= 250 × 250 × 220`.
	pub bed_x: f64,
	/// Usable bed Y (mm) — see [`bed_x`](Self::bed_x).
	pub bed_y: f64,
	/// Usable build height Z (mm) — see [`bed_x`](Self::bed_x).
	pub bed_z: f64,
}

impl FdmProfile {
	/// The research-derived conservative fallback — exactly the numbers the
	/// shipped RESPOOL/DRYBOX campaigns froze as consts and proved in print
	/// (per-field provenance on each field's doc). Use this when no measured
	/// `profiles/<printer>.json` exists yet; the calibration coupons replace
	/// it with reality.
	pub fn conservative_default() -> FdmProfile {
		FdmProfile {
			name: "conservative_default".to_string(),
			xy_clearance_tight: 0.05,
			xy_clearance_free: 0.25,
			z_clearance: 0.3,
			hole_diameter_comp: 0.0,
			bore_comp: 0.0,
			first_layer_comp: 0.0,
			seam_allowance: 0.0,
			max_bridge: 6.0,
			max_unsupported_angle: 45.0,
			min_wall: 1.2,
			bed_x: 250.0,
			bed_y: 250.0,
			bed_z: 220.0,
		}
	}

	/// Sanity-check every field against its physically meaningful range.
	/// Called by [`load`](Self::load) and [`save`](Self::save) so an insane
	/// profile can neither enter nor leave the engine silently.
	pub fn validate(&self) -> Result<(), ProcessError> {
		if self.name.trim().is_empty() || self.name.contains("PLACEHOLDER") {
			return Err(ProcessError::BadName(self.name.clone()));
		}
		let checks: [(&'static str, f64, f64, f64, &'static str); 13] = [
			("xy_clearance_tight", self.xy_clearance_tight, -0.2, 2.0, "a press fit beyond 0.2 mm interference or 2 mm gap is a measurement error"),
			("xy_clearance_free", self.xy_clearance_free, 0.0, 2.0, "a free fit needs a non-negative clearance under 2 mm"),
			("z_clearance", self.z_clearance, 0.0, 2.0, "axial clearance must be 0–2 mm"),
			("hole_diameter_comp", self.hole_diameter_comp, -1.0, 1.0, "a hole compensation beyond ±1 mm is a measurement error, not a printer"),
			("bore_comp", self.bore_comp, -1.0, 1.0, "a bore compensation beyond ±1 mm is a measurement error, not a printer"),
			("first_layer_comp", self.first_layer_comp, 0.0, 2.0, "elephant-foot budget is a non-negative radial flare under 2 mm"),
			("seam_allowance", self.seam_allowance, 0.0, 1.0, "a seam bump beyond 1 mm is a measurement error"),
			("max_bridge", self.max_bridge, 0.0, 100.0, "bridge span must be 0–100 mm"),
			("max_unsupported_angle", self.max_unsupported_angle, 1.0, 90.0, "overhang threshold is degrees from vertical in [1, 90]"),
			("min_wall", self.min_wall, 0.1, 10.0, "min printable wall must be 0.1–10 mm"),
			("bed_x", self.bed_x, 10.0, 2000.0, "bed extent must be 10–2000 mm"),
			("bed_y", self.bed_y, 10.0, 2000.0, "bed extent must be 10–2000 mm"),
			("bed_z", self.bed_z, 10.0, 2000.0, "bed extent must be 10–2000 mm"),
		];
		for (field, got, lo, hi, why) in checks {
			if !got.is_finite() || got < lo || got > hi {
				return Err(ProcessError::BadProfile { field, got, why });
			}
		}
		if self.xy_clearance_tight > self.xy_clearance_free {
			return Err(ProcessError::BadProfile {
				field: "xy_clearance_tight",
				got: self.xy_clearance_tight,
				why: "tight clearance must not exceed free clearance",
			});
		}
		Ok(())
	}

	/// Serialize to the canonical profile JSON: pretty-printed, fixed field
	/// order, trailing newline — the same profile always yields identical
	/// bytes (and `tools/ingest_calibration.py` writes the same shape).
	pub fn to_json(&self) -> String {
		let mut s = serde_json::to_string_pretty(self).expect("FdmProfile is plain data with string keys");
		s.push('\n');
		s
	}

	/// Parse a profile from [`to_json`](Self::to_json) bytes and range-check
	/// it. Unknown or missing fields are hard errors, never silent defaults.
	pub fn from_json(json: &str) -> Result<FdmProfile, ProcessError> {
		let p: FdmProfile =
			serde_json::from_str(json).map_err(|err| ProcessError::Schema { path: "<inline>".to_string(), err })?;
		p.validate()?;
		Ok(p)
	}

	/// The repo-convention path of a named profile: `profiles/<name>.json`
	/// (relative — campaigns run from the repo root, like their outputs).
	pub fn profiles_path(name: &str) -> String {
		format!("profiles/{name}.json")
	}

	/// Save to `path` in the canonical byte-stable form (see
	/// [`to_json`](Self::to_json)), refusing to write an out-of-range profile.
	pub fn save(&self, path: &str) -> Result<(), ProcessError> {
		self.validate()?;
		std::fs::write(path, self.to_json()).map_err(|err| ProcessError::Io { path: path.to_string(), err })
	}

	/// Load and range-check a profile from `path`.
	pub fn load(path: &str) -> Result<FdmProfile, ProcessError> {
		let text = std::fs::read_to_string(path).map_err(|err| ProcessError::Io { path: path.to_string(), err })?;
		let p: FdmProfile =
			serde_json::from_str(&text).map_err(|err| ProcessError::Schema { path: path.to_string(), err })?;
		p.validate()?;
		Ok(p)
	}

	// ---- fit helpers: the gate-consumption API --------------------------------

	/// Diametral print compensation for a feature of diameter `d`: the
	/// small-hole class below [`HOLE_BORE_CROSSOVER_D`], the large-bore class
	/// at or above it.
	pub fn comp_for_d(&self, d: f64) -> f64 {
		if d < HOLE_BORE_CROSSOVER_D {
			self.hole_diameter_comp
		} else {
			self.bore_comp
		}
	}

	/// Designed diameter for a hole/bore that should *measure* `nominal_d`
	/// after printing (compensation added, no fit clearance): `nominal_d +
	/// comp_for_d(nominal_d)`.
	pub fn hole_d(&self, nominal_d: f64) -> f64 {
		nominal_d + self.comp_for_d(nominal_d)
	}

	/// Designed BORE diameter that gives a **free running fit** over a printed
	/// shaft of `shaft_d`: shaft + 2·(free clearance + one seam allowance) +
	/// print compensation.
	pub fn fit_free_bore_d(&self, shaft_d: f64) -> f64 {
		shaft_d + 2.0 * (self.xy_clearance_free + self.seam_allowance) + self.comp_for_d(shaft_d)
	}

	/// Designed BORE diameter that gives a **snug/press fit** over a printed
	/// shaft of `shaft_d` — same construction as
	/// [`fit_free_bore_d`](Self::fit_free_bore_d) with the tight clearance.
	pub fn fit_tight_bore_d(&self, shaft_d: f64) -> f64 {
		shaft_d + 2.0 * (self.xy_clearance_tight + self.seam_allowance) + self.comp_for_d(shaft_d)
	}

	/// Designed SHAFT radius that runs freely inside a bore of radius
	/// `bore_r`: `bore_r − free clearance − one seam allowance`. External
	/// dimensions carry no diametral compensation (FDM outer walls print near
	/// nominal; the seam bump is budgeted explicitly instead). Reproduces
	/// RESPOOL's `R_TO = RI − C_R` under the conservative default.
	pub fn fit_free_shaft_r(&self, bore_r: f64) -> f64 {
		bore_r - self.xy_clearance_free - self.seam_allowance
	}

	/// Designed SHAFT radius for a snug/press fit inside a bore of radius
	/// `bore_r` — see [`fit_free_shaft_r`](Self::fit_free_shaft_r).
	/// Reproduces DRYBOX's `STUB_R = 3.95` for the 608's 4.0 bore radius.
	pub fn fit_tight_shaft_r(&self, bore_r: f64) -> f64 {
		bore_r - self.xy_clearance_tight - self.seam_allowance
	}

	/// Whether a flat bridge of `span` mm is within this printer's measured
	/// bridging ability.
	pub fn bridge_ok(&self, span: f64) -> bool {
		span <= self.max_bridge
	}

	/// Whether a wall of thickness `t` mm prints solid on this printer.
	pub fn wall_ok(&self, t: f64) -> bool {
		t >= self.min_wall
	}

	/// Whether a print-posed part of the given `[x, y, z]` extents fits the
	/// bed.
	pub fn bed_fits(&self, extents: [f64; 3]) -> bool {
		extents[0] <= self.bed_x && extents[1] <= self.bed_y && extents[2] <= self.bed_z
	}

	// ---- DFM checks ------------------------------------------------------------

	/// FDM design-for-manufacturing audit of a print-posed mesh (build
	/// direction `+Z`, part sitting at/near `z = 0`). Runs the four checks
	/// this profile parameterizes and returns one [`DfmFinding`] per
	/// violation (empty = clean):
	///
	/// - `watertight` — the mesh must enclose a volume to slice at all;
	/// - `support_steep` — downward area beyond
	///   [`max_unsupported_angle`](Self::max_unsupported_angle) (via
	///   [`Mesh::support_free_report`], bed tolerance 0.3 mm);
	/// - `bridge_span` — widest flat bridge vs [`max_bridge`](Self::max_bridge);
	/// - `thin_wall` — area thinner than [`min_wall`](Self::min_wall) (via
	///   [`Mesh::wall_thickness`]);
	/// - `bed_fit` — AABB extents vs the bed.
	pub fn dfm_checks_mesh(&self, m: &Mesh) -> Vec<DfmFinding> {
		let mut out = Vec::new();
		if !m.is_watertight() {
			out.push(DfmFinding {
				check: "watertight",
				measured: 0.0,
				limit: 1.0,
				detail: "mesh is not watertight — repair before slicing (voxel heal or fix the boolean chain)".to_string(),
			});
		}
		let rep = m.support_free_report(Vec3::Z, self.max_unsupported_angle as f32, 0.3);
		if rep.steep_area > 1e-6 {
			let wher = rep
				.steep_exemplars
				.first()
				.map(|p| format!(" — largest patch near ({:.1}, {:.1}, {:.1})", p.x, p.y, p.z))
				.unwrap_or_default();
			out.push(DfmFinding {
				check: "support_steep",
				measured: rep.steep_area,
				limit: 0.0,
				detail: format!(
					"{:.1} mm² needs support at the {:.0}° threshold{}",
					rep.steep_area, self.max_unsupported_angle, wher
				),
			});
		}
		if rep.max_bridge_span > self.max_bridge {
			out.push(DfmFinding {
				check: "bridge_span",
				measured: rep.max_bridge_span,
				limit: self.max_bridge,
				detail: format!(
					"widest flat bridge {:.1} mm exceeds the profile's {:.1} mm",
					rep.max_bridge_span, self.max_bridge
				),
			});
		}
		let walls = m.wall_thickness(self.min_wall);
		if walls.thin_area > 1e-6 {
			out.push(DfmFinding {
				check: "thin_wall",
				measured: walls.thin_area,
				limit: self.min_wall,
				detail: format!(
					"{:.1} mm² thinner than {:.2} mm (thinnest wall {:.2} mm)",
					walls.thin_area, self.min_wall, walls.min_thickness
				),
			});
		}
		let size = m.aabb().size();
		let ext = [size.x as f64, size.y as f64, size.z as f64];
		if !self.bed_fits(ext) {
			out.push(DfmFinding {
				check: "bed_fit",
				measured: ext[0].max(ext[1]).max(ext[2]),
				limit: self.bed_x.min(self.bed_y).min(self.bed_z),
				detail: format!(
					"extents {:.0} × {:.0} × {:.0} mm exceed the {:.0} × {:.0} × {:.0} bed",
					ext[0], ext[1], ext[2], self.bed_x, self.bed_y, self.bed_z
				),
			});
		}
		out
	}

	/// [`dfm_checks_mesh`](Self::dfm_checks_mesh) on an exact solid: validates
	/// the B-rep first (an invalid solid is itself a finding), then audits the
	/// default tessellation.
	pub fn dfm_checks(&self, s: &Solid) -> Vec<DfmFinding> {
		let v = validate(s);
		let mut out = Vec::new();
		if !v.is_valid() {
			out.push(DfmFinding {
				check: "brep_valid",
				measured: 0.0,
				limit: 1.0,
				detail: format!("solid fails validate(): closed={} manifold={}", v.closed, v.manifold),
			});
		}
		out.extend(self.dfm_checks_mesh(&tessellate_default(s)));
		out
	}
}

/// The **frozen nominal dimensions of coupon set v1** — the single source of
/// truth shared by the retired `calibrate_fdm.rs` example (which printed them into
/// geometry and into `coupon_nominals.json`) and `tools/ingest_calibration.py`
/// (which embeds the same numbers and is pinned against these consts by
/// `tests/process.rs`). Change these only together with a version bump.
pub mod coupons {
	/// Coupon-set schema version (`coupons_version` in measurements.json).
	pub const VERSION: u32 = 1;
	/// Small-hole ladder diameters (mm), ascending from the fiducial corner.
	pub const HOLE_LADDER_D: [f64; 11] = [3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5, 8.0];
	/// Reference pin diameter (mm) for the fit ladder.
	pub const FIT_PIN_D: f64 = 6.0;
	/// Fit-ladder bore diameters (mm) — designed diametral clearances 0.0–0.6
	/// over the pin.
	pub const FIT_BORE_D: [f64; 7] = [6.0, 6.1, 6.2, 6.3, 6.4, 6.5, 6.6];
	/// Large-bore gauge diameter (mm) — a 608 bearing's OD, so the printed
	/// coupon can also be sanity-checked with a real bearing.
	pub const BORE_LARGE_D: f64 = 22.0;
	/// Pin-base disc diameter (mm) — elephant-foot + XY scale reference.
	pub const DISC_D: f64 = 20.0;
	/// Bridge-ladder clear spans (mm).
	pub const BRIDGE_SPANS: [f64; 5] = [5.0, 10.0, 15.0, 20.0, 25.0];
	/// Wall-ladder fin thicknesses (mm).
	pub const WALL_LADDER_T: [f64; 5] = [0.8, 1.2, 1.6, 2.0, 2.4];
	/// Overhang-fan angles, degrees from vertical. 45° is deliberately absent:
	/// it sits exactly on the default gate threshold and would make the
	/// steep-area expectation a coin flip.
	pub const OVERHANG_DEG: [f64; 5] = [35.0, 40.0, 50.0, 55.0, 60.0];
}
