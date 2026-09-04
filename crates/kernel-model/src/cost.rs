// Copyright (c) LMCAD. Licensed under the MIT License.

//! Process **cost and time** per part — how much material a part eats, how long
//! the machine is busy, and what that costs, so a BOM can carry money instead
//! of only mass.
//!
//! # Honest status (2026-07-30)
//!
//! | process | status |
//! |---|---|
//! | FDM | **implemented**: [`FdmCostModel`] — deposition volume ÷ volumetric flow, per-layer overhead, travel allowance, support envelope from the mesh's own support report, material + machine cost |
//! | sheet metal | declared sibling, **NOT implemented** — refuses ([`CostError::NotImplemented`]) |
//! | casting | declared sibling, **NOT implemented** — refuses |
//! | CNC | declared sibling, **NOT implemented** — refuses |
//!
//! The siblings mirror the [`crate::process`] doctrine exactly: a declared
//! variant that refuses loudly is honest; a machining-cycle-time model invented
//! here would not be. Nothing in this module guesses a sibling's numbers.
//!
//! # The FDM model, written out
//!
//! With `V` the part's analytic volume, `A` its surface area, and the model's
//! parameters:
//!
//! ```text
//! V_shell     = min(V, A · shell_thickness_mm)
//! V_core      = V − V_shell
//! V_part      = V_shell + infill_fraction · V_core        (= V exactly when infill = 1)
//! V_support   = support_density · support_envelope_mm3
//! V_dep       = V_part + V_support                         deposited material
//! mass_g      = V_dep · density_g_mm3
//! t_extrude   = V_dep / volumetric_flow_mm3_s              [s]
//! layers      = ceil(height_mm / layer_height_mm)          (snapped, see below)
//! t_layer     = layers · per_layer_overhead_s              [s]
//! minutes     = (t_extrude · (1 + travel_fraction) + t_layer) / 60 + setup_minutes
//! material_$  = mass_g / 1000 · material_cost_per_kg
//! machine_$   = minutes / 60 · machine_cost_per_hour
//! total       = material_$ + machine_$
//! ```
//!
//! The layer count snaps `height / layer_height` to an integer when it is within
//! 1e-9 of one, so a 20.000 mm part at 0.200 mm layers reads 100 layers and not
//! 101 from a binary-representation artifact (`1.0 / 0.1` is
//! `10.000000000000002` in IEEE-754).
//!
//! # Accuracy — read this before quoting
//!
//! [`FDM_ACCURACY_CLASS`] is stamped into every [`CostBreakdown`] as a
//! **required field**, not optional prose. The model is a **±30% class**
//! estimate. It deliberately does not model acceleration/jerk limits, per-feature
//! speed profiles (perimeters vs sparse infill vs solid top layers),
//! cooling-limited minimum layer time, first-layer slowdown, retraction and seam
//! time, or the slicer's actual infill geometry. Its money rates are *declared
//! inputs* — an engine cannot measure a filament price. A ±30% number is useful
//! for quoting and comparing designs; it is not a slicer result and must never
//! be presented as one.
//!
//! # Where the default parameters come from
//!
//! [`FdmCostModel::conservative_default`] cites each field in its doc comment.
//! Geometry-side numbers follow the shipped campaigns (`respool.rs`,
//! `drybox_roller.rs`, pre-JSON examples removed from the tree 2026-09-03); the **money** rates are explicitly labelled placeholders
//! for the caller to replace.
//!
//! # Example
//!
//! ```
//! use kernel_brep::cuboid;
//! use kernel_brep::math::DVec3;
//! use kernel_model::cost::{CostProcess, FdmCostModel};
//!
//! let block = cuboid(DVec3::ZERO, DVec3::new(40.0, 40.0, 20.0));
//! let mut model = FdmCostModel::conservative_default();
//! model.infill_fraction = 1.0; // solid
//! let cost = CostProcess::Fdm(model).estimate(&block).expect("FDM is implemented");
//! // 32 000 mm^3 of PLA at 0.00124 g/mm^3.
//! assert!((cost.material_g - 39.68).abs() < 1e-9, "material_g = {}", cost.material_g);
//! assert!(!cost.model_accuracy_note.is_empty(), "the accuracy note is required");
//!
//! // A sibling process refuses instead of inventing a cycle-time model.
//! assert!(CostProcess::Cnc.estimate(&block).is_err());
//! ```

use std::collections::BTreeMap;
use std::fmt::Write as _;

use kernel_brep::math::DVec3;
use kernel_brep::{area, bounding_box, exact_volume, tessellate_default, Solid};
use kernel_core::math::Vec3;

/// The accuracy class stamped into every FDM [`CostBreakdown`]. Loud on
/// purpose: a cost number without its error bar is a lie in a spreadsheet.
pub const FDM_ACCURACY_CLASS: &str = "+/-30% CLASS ESTIMATE (deposition-volume / volumetric-flow model). NOT MODELLED: acceleration and jerk limits, per-feature speed profiles (perimeter vs sparse infill vs solid layers), cooling-limited minimum layer time, first-layer slowdown, retraction and seam time, the slicer's actual infill geometry. Money rates are declared inputs, not measurements. Use for quoting and design comparison; never present as a slicer result.";

/// Build direction assumed by every FDM estimate: `+Z`, i.e. the part is
/// supplied **print-posed**, the same convention as
/// [`crate::process::FdmProfile::dfm_checks`].
pub const BUILD_DIR: Vec3 = Vec3::Z;

/// Bed-contact tolerance (mm) used when classifying support: a downward facet
/// within this of the lowest point is the first layer, not an overhang. Matches
/// the `support_free_report(Z, 45.0, 0.3)` gate every shipped campaign runs.
pub const BED_TOL_MM: f64 = 0.3;

// ---------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------

/// Errors from the cost layer — including the honest refusals of the declared
/// but unimplemented sibling processes.
#[derive(Clone, Debug, PartialEq)]
pub enum CostError {
	/// The process is declared but has no cost model. `note` names what *does*
	/// exist, so a caller can route rather than guess.
	NotImplemented {
		/// [`CostProcess::name`] of the refused process.
		process: &'static str,
		/// What exists today for this process.
		note: &'static str,
	},
	/// A model parameter is outside its physically sane range.
	BadParameter {
		/// Field name.
		field: &'static str,
		/// Offending value.
		got: f64,
		/// The rule it broke.
		why: &'static str,
	},
	/// The part produced no measurable geometry to cost.
	NoGeometry {
		/// What was missing.
		what: &'static str,
	},
}

impl std::fmt::Display for CostError {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		match self {
			CostError::NotImplemented { process, note } => {
				write!(f, "{process} cost model not implemented — declared sibling, see kernel_model::cost module doc{note}")
			}
			CostError::BadParameter { field, got, why } => {
				write!(f, "cost parameter '{field}' = {got} is out of range: {why} — refusing to produce a number from an impossible model")
			}
			CostError::NoGeometry { what } => write!(f, "nothing to cost: {what}"),
		}
	}
}

impl std::error::Error for CostError {}

// ---------------------------------------------------------------------------
// Process routing
// ---------------------------------------------------------------------------

/// A manufacturing process for costing. Only FDM carries a model; the siblings
/// are declared so downstream code can route on process *now* and gets a loud
/// [`CostError::NotImplemented`] — never a silent stub — until their models
/// land. Mirrors [`crate::process::Process`] without depending on it: cost and
/// DFM are separate concerns and a caller may have one without the other.
#[derive(Clone, Debug)]
pub enum CostProcess {
	/// Fused-deposition 3D printing — implemented.
	Fdm(FdmCostModel),
	/// Sheet-metal fabrication — declared sibling, refuses (no blank
	/// development / press-time model).
	SheetMetal,
	/// Casting / molding — declared sibling, refuses (no tooling amortization
	/// or cycle-time model).
	Casting,
	/// Subtractive machining — declared sibling, refuses (no
	/// material-removal-rate or tool-access model).
	Cnc,
}

impl CostProcess {
	/// Stable lowercase name, used in messages and CSV.
	pub fn name(&self) -> &'static str {
		match self {
			CostProcess::Fdm(_) => "fdm",
			CostProcess::SheetMetal => "sheet_metal",
			CostProcess::Casting => "casting",
			CostProcess::Cnc => "cnc",
		}
	}

	/// Cost one part. FDM estimates; every sibling refuses with
	/// [`CostError::NotImplemented`] naming what exists for it instead.
	///
	/// # Errors
	///
	/// Every non-FDM variant; plus [`CostError::BadParameter`] /
	/// [`CostError::NoGeometry`] from the FDM path.
	pub fn estimate(&self, solid: &Solid) -> Result<CostBreakdown, CostError> {
		match self {
			CostProcess::Fdm(m) => m.estimate(solid),
			other => Err(other.refuse()),
		}
	}

	/// The typed refusal for a sibling process.
	fn refuse(&self) -> CostError {
		let note = match self {
			CostProcess::Fdm(_) => "",
			CostProcess::SheetMetal => " — no blank-development or press-time model exists; kernel_brep has no unfold either",
			CostProcess::Casting => {
				" — no tooling-amortization or cycle-time model exists; the castability half that DOES exist is kernel_brep::draft_analysis (per-face draft + undercut area)"
			}
			CostProcess::Cnc => " — no material-removal-rate or tool-access model exists; kernel_model::process::Process::Cnc refuses for the same reason",
		};
		CostError::NotImplemented { process: self.name(), note }
	}
}

// ---------------------------------------------------------------------------
// The FDM model
// ---------------------------------------------------------------------------

/// An FDM cost/time model. Every field is a **declared parameter** — this
/// struct is data, not measurement — and every field's doc comment cites where
/// its default came from. Change them to match your machine and filament.
#[derive(Clone, Debug, PartialEq)]
pub struct FdmCostModel {
	/// A name for the profile these numbers describe.
	pub name: String,
	/// Layer height, mm. Default 0.2 — the common 0.4 mm-nozzle default and the
	/// height every shipped campaign was sliced at.
	pub layer_height_mm: f64,
	/// Sustained volumetric flow rate, mm³/s. Default 12.0 — deliberately below
	/// the 20–30 mm³/s hotends advertise, because a real sliced average over
	/// short perimeters and direction changes lands far under peak.
	pub volumetric_flow_mm3_s: f64,
	/// Fixed per-layer overhead, s (layer change, Z move, seam, prime).
	/// Default 1.5.
	pub per_layer_overhead_s: f64,
	/// Travel time as a fraction of extrusion time. Default 0.12.
	pub travel_fraction: f64,
	/// Printed-solid shell thickness, mm — perimeters + top/bottom skins.
	/// Default 1.2, the thinnest wall a shipped campaign prints
	/// (`drybox_roller` RIB_T; the same number the FDM process profile carries
	/// as `min_wall`).
	pub shell_thickness_mm: f64,
	/// Sparse-infill fraction of the core, 0..=1. Default 0.20 — the middle of
	/// the `drybox_roller` BOM's declared "3 walls 15–25%".
	pub infill_fraction: f64,
	/// Material density, g/mm³. Default [`crate::materials::PLA_G_PER_MM3`].
	pub density_g_mm3: f64,
	/// Material price per kilogram, in the caller's currency.
	/// **PLACEHOLDER (25.0)** — an engine cannot measure a filament price.
	pub material_cost_per_kg: f64,
	/// Machine time price per hour, in the caller's currency.
	/// **PLACEHOLDER (1.0)** — depreciation/power/attention are a business
	/// decision, not a measurement.
	pub machine_cost_per_hour: f64,
	/// Fixed per-part setup time, minutes (plate prep, purge, removal).
	/// Default 5.0 — a placeholder for shop practice.
	pub setup_minutes: f64,
	/// Steepest printable overhang, degrees from vertical. Default 45.0 — the
	/// `support_free_report(Z, 45.0, 0.3)` threshold every campaign gates on.
	pub support_overhang_deg: f64,
	/// Fraction of the support *envelope* actually filled with material.
	/// Default 0.15 — typical sparse support density.
	pub support_density: f64,
}

impl FdmCostModel {
	/// The declared conservative default (see each field's doc comment for its
	/// source). Geometry-side numbers follow the shipped campaigns; the money
	/// rates are placeholders the caller is expected to replace.
	pub fn conservative_default() -> FdmCostModel {
		FdmCostModel {
			name: "conservative_default".to_string(),
			layer_height_mm: 0.2,
			volumetric_flow_mm3_s: 12.0,
			per_layer_overhead_s: 1.5,
			travel_fraction: 0.12,
			shell_thickness_mm: 1.2,
			infill_fraction: 0.20,
			density_g_mm3: crate::materials::PLA_G_PER_MM3,
			material_cost_per_kg: 25.0,
			machine_cost_per_hour: 1.0,
			setup_minutes: 5.0,
			support_overhang_deg: 45.0,
			support_density: 0.15,
		}
	}

	/// Range-check every parameter.
	///
	/// # Errors
	///
	/// [`CostError::BadParameter`] naming the first field outside its range.
	/// Zero flow, negative density and an infill outside `[0, 1]` all refuse
	/// here rather than producing an infinite or negative estimate downstream.
	pub fn validate(&self) -> Result<(), CostError> {
		let checks: [(&'static str, f64, f64, f64, &'static str); 11] = [
			("layer_height_mm", self.layer_height_mm, 1e-3, 5.0, "layer height must be a real positive height in (0.001, 5] mm"),
			(
				"volumetric_flow_mm3_s",
				self.volumetric_flow_mm3_s,
				1e-6,
				1e4,
				"volumetric flow must be strictly positive — a zero flow rate means infinite time, not a free print",
			),
			("per_layer_overhead_s", self.per_layer_overhead_s, 0.0, 600.0, "per-layer overhead must be a non-negative number of seconds"),
			("travel_fraction", self.travel_fraction, 0.0, 10.0, "travel is a non-negative fraction of extrusion time"),
			("shell_thickness_mm", self.shell_thickness_mm, 0.0, 100.0, "shell thickness must be non-negative"),
			("infill_fraction", self.infill_fraction, 0.0, 1.0, "infill is a fraction of the core in [0, 1]"),
			(
				"density_g_mm3",
				self.density_g_mm3,
				1e-9,
				1.0,
				"density must be strictly positive g/mm^3 — a negative density would make a part weigh less than nothing",
			),
			("material_cost_per_kg", self.material_cost_per_kg, 0.0, 1e7, "material price must be non-negative"),
			("machine_cost_per_hour", self.machine_cost_per_hour, 0.0, 1e7, "machine rate must be non-negative"),
			("setup_minutes", self.setup_minutes, 0.0, 1e5, "setup time must be non-negative"),
			("support_density", self.support_density, 0.0, 1.0, "support density is a fraction in [0, 1]"),
		];
		for (field, got, lo, hi, why) in checks {
			if !got.is_finite() || got < lo || got > hi {
				return Err(CostError::BadParameter { field, got, why });
			}
		}
		if !self.support_overhang_deg.is_finite() || self.support_overhang_deg < 1.0 || self.support_overhang_deg > 90.0 {
			return Err(CostError::BadParameter {
				field: "support_overhang_deg",
				got: self.support_overhang_deg,
				why: "the overhang threshold is degrees from vertical in [1, 90]",
			});
		}
		if self.name.trim().is_empty() {
			return Err(CostError::BadParameter { field: "name", got: 0.0, why: "name the profile these numbers describe" });
		}
		Ok(())
	}

	/// Layer count for a part `height_mm` tall: `ceil(height / layer_height)`,
	/// with the ratio snapped to an integer when within 1e-9 of one.
	///
	/// The snap is not cosmetic: `20.0 / 0.2` and `1.0 / 0.1` are respectively
	/// `99.99999999999999` and `10.000000000000002` in IEEE-754, so a raw
	/// `ceil` reports 11 layers for a 1 mm part at 0.1 mm. A part exactly `k`
	/// layers tall reads `k`.
	///
	/// # Errors
	///
	/// [`CostError::BadParameter`] for a non-finite or negative height, or an
	/// invalid layer height.
	pub fn layer_count(&self, height_mm: f64) -> Result<usize, CostError> {
		if !height_mm.is_finite() || height_mm < 0.0 {
			return Err(CostError::BadParameter { field: "height_mm", got: height_mm, why: "part height must be finite and non-negative" });
		}
		if !self.layer_height_mm.is_finite() || self.layer_height_mm <= 0.0 {
			return Err(CostError::BadParameter {
				field: "layer_height_mm",
				got: self.layer_height_mm,
				why: "layer height must be finite and > 0",
			});
		}
		let raw = height_mm / self.layer_height_mm;
		let snapped = if (raw - raw.round()).abs() < 1e-9 { raw.round() } else { raw.ceil() };
		Ok(snapped.max(0.0) as usize)
	}

	/// Print time in minutes for a given **deposited** volume and part height:
	/// `(V/flow · (1 + travel) + layers · overhead) / 60 + setup`.
	///
	/// Exposed on its own so a gate can pin the model's shape directly:
	/// doubling `deposited_mm3` raises the extrusion term exactly two-fold, and
	/// doubling `volumetric_flow_mm3_s` halves it.
	///
	/// # Errors
	///
	/// [`CostError::BadParameter`] for invalid parameters or a non-finite,
	/// negative volume.
	pub fn print_time_minutes(&self, deposited_mm3: f64, height_mm: f64) -> Result<f64, CostError> {
		self.validate()?;
		if !deposited_mm3.is_finite() || deposited_mm3 < 0.0 {
			return Err(CostError::BadParameter {
				field: "deposited_mm3",
				got: deposited_mm3,
				why: "deposited volume must be finite and non-negative",
			});
		}
		let layers = self.layer_count(height_mm)? as f64;
		let extrude_s = deposited_mm3 / self.volumetric_flow_mm3_s;
		Ok((extrude_s * (1.0 + self.travel_fraction) + layers * self.per_layer_overhead_s) / 60.0 + self.setup_minutes)
	}

	/// Deposited part volume (excluding support) for a solid of volume `v` and
	/// surface area `a`: a solid shell of [`Self::shell_thickness_mm`] plus
	/// [`Self::infill_fraction`] of what is left.
	///
	/// The shell term is capped at `v`, so a part thinner than twice the shell
	/// thickness costs as solid — which is what a slicer does too. At
	/// `infill_fraction == 1.0` this returns `v` **exactly** (no floating-point
	/// residue), so a solid part's mass equals `exact_volume × density` to the
	/// bit.
	pub fn deposited_part_mm3(&self, v: f64, a: f64) -> f64 {
		if self.infill_fraction >= 1.0 {
			return v;
		}
		let shell = (a * self.shell_thickness_mm).min(v);
		shell + self.infill_fraction * (v - shell)
	}

	/// Measure the geometric inputs a cost estimate needs from a print-posed
	/// solid (build direction `+Z`).
	///
	/// # Errors
	///
	/// [`CostError::NoGeometry`] when the solid has no bounding box or encloses
	/// no volume.
	pub fn measure(&self, solid: &Solid) -> Result<PartMeasures, CostError> {
		let bb = bounding_box(solid).ok_or(CostError::NoGeometry { what: "the solid has no finite vertices" })?;
		let volume_mm3 = exact_volume(solid).abs();
		if !volume_mm3.is_finite() || volume_mm3 <= 0.0 {
			return Err(CostError::NoGeometry { what: "kernel_brep::exact_volume is zero — the solid encloses nothing" });
		}
		Ok(PartMeasures {
			volume_mm3,
			surface_area_mm2: area(solid),
			height_mm: bb.size().z,
			support_envelope_mm3: support_envelope_mm3(solid, self.support_overhang_deg, BED_TOL_MM),
			volume_source: "exact (kernel_brep::exact_volume)",
		})
	}

	/// Cost a part from already-measured geometry.
	///
	/// # Errors
	///
	/// [`CostError::BadParameter`] for an invalid model or non-finite measures.
	pub fn estimate_measured(&self, m: &PartMeasures) -> Result<CostBreakdown, CostError> {
		self.validate()?;
		for (field, got) in [
			("volume_mm3", m.volume_mm3),
			("surface_area_mm2", m.surface_area_mm2),
			("height_mm", m.height_mm),
			("support_envelope_mm3", m.support_envelope_mm3),
		] {
			if !got.is_finite() || got < 0.0 {
				return Err(CostError::BadParameter { field, got, why: "a measured input must be finite and non-negative" });
			}
		}
		let part_mm3 = self.deposited_part_mm3(m.volume_mm3, m.surface_area_mm2);
		let support_mm3 = self.support_density * m.support_envelope_mm3;
		let deposited_mm3 = part_mm3 + support_mm3;
		let material_g = deposited_mm3 * self.density_g_mm3;
		let time_minutes = self.print_time_minutes(deposited_mm3, m.height_mm)?;
		let material_cost = material_g / 1000.0 * self.material_cost_per_kg;
		let machine_cost = time_minutes / 60.0 * self.machine_cost_per_hour;
		let layers = self.layer_count(m.height_mm)?;
		Ok(CostBreakdown {
			process: "fdm",
			material_g,
			material_cost,
			time_minutes,
			machine_cost,
			total: material_cost + machine_cost,
			model_accuracy_note: format!(
				"{FDM_ACCURACY_CLASS} MODEL: '{}' at {:.3} mm layers ({layers} layers), {:.3} mm3/s flow, {:.3} s/layer overhead, {:.0}% travel allowance, {:.0}% infill inside a {:.3} mm shell, support envelope {:.3} mm3 at {:.0}% density.",
				self.name,
				self.layer_height_mm,
				self.volumetric_flow_mm3_s,
				self.per_layer_overhead_s,
				self.travel_fraction * 100.0,
				self.infill_fraction * 100.0,
				self.shell_thickness_mm,
				m.support_envelope_mm3,
				self.support_density * 100.0
			),
			part_volume_mm3: m.volume_mm3,
			deposited_volume_mm3: deposited_mm3,
			support_volume_mm3: support_mm3,
			layers,
			print_height_mm: m.height_mm,
			volume_source: m.volume_source,
		})
	}

	/// Measure and cost a print-posed solid in one call.
	///
	/// # Errors
	///
	/// As [`Self::measure`] and [`Self::estimate_measured`].
	pub fn estimate(&self, solid: &Solid) -> Result<CostBreakdown, CostError> {
		let m = self.measure(solid)?;
		self.estimate_measured(&m)
	}
}

/// The geometric inputs of a cost estimate, measured from the part.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PartMeasures {
	/// Enclosed volume, mm³.
	pub volume_mm3: f64,
	/// Total surface area, mm² (bores included — they are printed too).
	pub surface_area_mm2: f64,
	/// Build height along `+Z`, mm.
	pub height_mm: f64,
	/// Upper-bound support **envelope** volume, mm³ — see
	/// [`support_envelope_mm3`]. Multiply by the model's support density for
	/// the material actually deposited.
	pub support_envelope_mm3: f64,
	/// Which volume fed the estimate, for the receipt.
	pub volume_source: &'static str,
}

/// Upper-bound support **envelope** volume of a print-posed solid: for every
/// triangle the mesh's own [`kernel_core::SupportFreeReport`] flags as *steep*
/// (needing support — bed contact and bridgeable ceilings excluded), the prism
/// from its bed-projected footprint down to the build plate.
///
/// # Contract and limit
///
/// This is an **upper bound, stated**: a support column that lands on lower part
/// geometry instead of the plate is shorter than the prism this integrates, so
/// the envelope over-counts exactly where a part overhangs itself. It never
/// under-counts, which is the safe direction for a quote. `overhang_deg` is
/// degrees from vertical (45° is the usual limit); `bed_tol_mm` is the
/// first-layer band that counts as bed contact.
///
/// Returns `0.0` for a part that prints support-free in this orientation — the
/// same verdict as `steep_area == 0`.
pub fn support_envelope_mm3(solid: &Solid, overhang_deg: f64, bed_tol_mm: f64) -> f64 {
	let mesh = tessellate_default(solid);
	if mesh.indices.is_empty() {
		return 0.0;
	}
	let report = mesh.support_free_report(BUILD_DIR, overhang_deg as f32, bed_tol_mm as f32);
	let up = DVec3::new(BUILD_DIR.x as f64, BUILD_DIR.y as f64, BUILD_DIR.z as f64).normalize_or_zero();
	let zmin = mesh.positions.iter().map(|p| p.as_dvec3().dot(up)).fold(f64::INFINITY, f64::min);
	if !zmin.is_finite() {
		return 0.0;
	}
	let mut volume = 0.0;
	for (ti, t) in mesh.indices.chunks_exact(3).enumerate() {
		if !report.steep.get(ti).copied().unwrap_or(false) {
			continue;
		}
		let a = mesh.positions[t[0] as usize].as_dvec3();
		let b = mesh.positions[t[1] as usize].as_dvec3();
		let c = mesh.positions[t[2] as usize].as_dvec3();
		let area_vec = (b - a).cross(c - a);
		// Footprint = the triangle's area projected onto the build plate.
		let footprint = area_vec.dot(up).abs() * 0.5;
		let drop = ((a.dot(up) + b.dot(up) + c.dot(up)) / 3.0 - zmin).max(0.0);
		volume += footprint * drop;
	}
	volume
}

/// What one part costs, with its error bar attached.
///
/// [`Self::model_accuracy_note`] is a **required field**: a cost number that
/// travels without its accuracy class becomes a promise the moment it lands in
/// a spreadsheet. Every constructor in this module fills it.
#[derive(Clone, Debug, PartialEq)]
pub struct CostBreakdown {
	/// Which process produced this estimate.
	pub process: &'static str,
	/// Deposited material mass, grams (part + support).
	pub material_g: f64,
	/// Material cost in the caller's currency.
	pub material_cost: f64,
	/// Machine-busy time, minutes (including setup).
	pub time_minutes: f64,
	/// Machine-time cost in the caller's currency.
	pub machine_cost: f64,
	/// `material_cost + machine_cost`.
	pub total: f64,
	/// The model's accuracy class and what it does not model — always present.
	pub model_accuracy_note: String,
	/// The part's own enclosed volume, mm³ (before shell/infill).
	pub part_volume_mm3: f64,
	/// Total deposited volume, mm³ (part material + support material).
	pub deposited_volume_mm3: f64,
	/// Support material volume, mm³ (0 when the part prints support-free).
	pub support_volume_mm3: f64,
	/// Layer count.
	pub layers: usize,
	/// Build height, mm.
	pub print_height_mm: f64,
	/// Where the volume came from — the honest routing verdict.
	pub volume_source: &'static str,
}

impl CostBreakdown {
	/// A one-line human summary with the accuracy class flagged.
	pub fn summary(&self) -> String {
		format!(
			"{}: {:.2} g, {:.1} min, material {:.4} + machine {:.4} = {:.4} [{}]",
			self.process, self.material_g, self.time_minutes, self.material_cost, self.machine_cost, self.total, "+/-30% class"
		)
	}
}

// ---------------------------------------------------------------------------
// Costed BOM
// ---------------------------------------------------------------------------

/// One part offered for costing: its BOM identity (name + parameter summary —
/// the §18.4 grouping key), how many the assembly places, and the geometry.
#[derive(Clone, Copy, Debug)]
pub struct CostItem<'a> {
	/// Part name.
	pub name: &'a str,
	/// The part's parameter summary, exactly as `format::BomLine::params` uses
	/// it, so two same-named parts at different dimensions stay separate lines.
	pub params: &'a str,
	/// How many the assembly places.
	pub count: usize,
	/// The print-posed solid.
	pub solid: &'a Solid,
}

/// One grouped line of a costed BOM.
#[derive(Clone, Debug, PartialEq)]
pub struct CostedBomLine {
	/// Part name.
	pub name: String,
	/// Parameter summary (the second half of the grouping key).
	pub params: String,
	/// Instances in this line.
	pub count: usize,
	/// The cost of ONE instance.
	pub unit: CostBreakdown,
	/// `unit.total × count`.
	pub line_total: f64,
	/// `unit.material_g × count`.
	pub line_material_g: f64,
	/// `unit.time_minutes × count`.
	pub line_time_minutes: f64,
}

/// A complete costed bill of materials.
#[derive(Clone, Debug)]
pub struct CostedBom {
	/// Grouped lines, sorted by name then parameter summary — the §18.4
	/// grouping, so a costed BOM lines up row-for-row with `bom.json`'s flat
	/// view.
	pub lines: Vec<CostedBomLine>,
	/// Currency label (a tag only — the engine does no conversion).
	pub currency: String,
	/// Sum of [`CostedBomLine::line_total`].
	pub total: f64,
	/// Sum of [`CostedBomLine::line_material_g`].
	pub total_material_g: f64,
	/// Sum of [`CostedBomLine::line_time_minutes`].
	pub total_time_minutes: f64,
	/// The accuracy class this whole table inherits — required, like the
	/// per-part note.
	pub model_accuracy_note: String,
}

impl CostedBom {
	/// The table as Markdown, for a campaign's `BOM.md`. Byte-stable: lines are
	/// sorted, every number is fixed-decimal, and the accuracy note is emitted
	/// under the table where a reader cannot miss it.
	pub fn to_markdown(&self) -> String {
		let mut out = String::new();
		let _ = writeln!(
			out,
			"| part | params | qty | unit mass (g) | unit time (min) | unit cost ({c}) | line cost ({c}) |",
			c = self.currency
		);
		let _ = writeln!(out, "|---|---|---:|---:|---:|---:|---:|");
		for l in &self.lines {
			let _ = writeln!(
				out,
				"| {} | {} | {} | {:.3} | {:.2} | {:.4} | {:.4} |",
				l.name, l.params, l.count, l.unit.material_g, l.unit.time_minutes, l.unit.total, l.line_total
			);
		}
		let _ = writeln!(
			out,
			"| **TOTAL** |  | {} | {:.3} | {:.2} |  | {:.4} |",
			self.lines.iter().map(|l| l.count).sum::<usize>(),
			self.total_material_g,
			self.total_time_minutes,
			self.total
		);
		let _ = writeln!(out, "\n> {}", self.model_accuracy_note);
		out
	}

	/// The table as CSV with fixed columns
	/// `name,params,count,unit_material_g,unit_time_minutes,unit_total,line_total`.
	/// Byte-stable; `\n` line endings; trailing newline.
	pub fn to_csv(&self) -> String {
		let mut out = String::from("name,params,count,unit_material_g,unit_time_minutes,unit_total,line_total\n");
		for l in &self.lines {
			let _ = writeln!(
				out,
				"{},{},{},{:.6},{:.6},{:.6},{:.6}",
				csv_field(&l.name),
				csv_field(&l.params),
				l.count,
				l.unit.material_g,
				l.unit.time_minutes,
				l.unit.total,
				l.line_total
			);
		}
		out
	}
}

/// RFC-4180 CSV field: quoted (inner `"` doubled) when it contains a comma,
/// quote or newline; verbatim otherwise. Same rule as `format::BomV2::to_csv`.
fn csv_field(s: &str) -> String {
	if s.contains(',') || s.contains('"') || s.contains('\n') || s.contains('\r') {
		format!("\"{}\"", s.replace('"', "\"\""))
	} else {
		s.to_string()
	}
}

/// Cost a set of parts into a grouped BOM table.
///
/// # Contract
///
/// - Grouping is **§18.4**: `(name, params)`, counts summed, lines sorted by
///   name then params — so a costed table lines up row-for-row with the
///   assembly's `bom.json` flat view.
/// - Every line is costed once, from the FIRST item of its group; two items
///   sharing an identity are assumed to share geometry, exactly as `bom_v2`
///   assumes they share meta.
/// - Deterministic: a `BTreeMap` grouping and fixed-decimal rendering, so
///   [`CostedBom::to_markdown`] and [`CostedBom::to_csv`] are byte-stable.
///
/// # Errors
///
/// Any [`CostError`] from the underlying process — including the sibling
/// refusals, so a BOM cannot be half-costed by a process that has no model.
pub fn costed_bom(items: &[CostItem], process: &CostProcess, currency: &str) -> Result<CostedBom, CostError> {
	let mut groups: BTreeMap<(String, String), (usize, usize)> = BTreeMap::new();
	for (i, item) in items.iter().enumerate() {
		let e = groups.entry((item.name.to_string(), item.params.to_string())).or_insert((0, i));
		e.0 += item.count;
	}
	let mut lines = Vec::with_capacity(groups.len());
	let mut total = 0.0;
	let mut total_material_g = 0.0;
	let mut total_time_minutes = 0.0;
	for ((name, params), (count, first)) in groups {
		let unit = process.estimate(items[first].solid)?;
		let line_total = unit.total * count as f64;
		let line_material_g = unit.material_g * count as f64;
		let line_time_minutes = unit.time_minutes * count as f64;
		total += line_total;
		total_material_g += line_material_g;
		total_time_minutes += line_time_minutes;
		lines.push(CostedBomLine { name, params, count, unit, line_total, line_material_g, line_time_minutes });
	}
	let model_accuracy_note = lines.first().map(|l| l.unit.model_accuracy_note.clone()).unwrap_or_else(|| FDM_ACCURACY_CLASS.to_string());
	Ok(CostedBom { lines, currency: currency.to_string(), total, total_material_g, total_time_minutes, model_accuracy_note })
}
