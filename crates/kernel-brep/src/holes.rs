// Copyright (c) LMCAD. Licensed under the MIT License.

//! The **hole-wizard vocabulary**: standard machining-style hole operations as
//! AI-callable functions that cut a given [`Solid`] — plain drilling, ISO 273
//! clearance holes, DIN 974 counterbores, DIN 74 countersinks, tap-drill pilot
//! holes, bolt circles, and bearing seats — with the real ISO/DIN dimension
//! tables hardcoded as documented constants ([`MetricHoleSpec`], [`BearingSpec`]).
//!
//! Conventions (project-wide): all dimensions are **mm** and **diameters**, never
//! radii; `at` is a point on (or above) the entry face; `axis` points **into** the
//! material; depths are measured from `at` along `axis`. Cutting tools are composed
//! from the existing [`cylinder`]/[`cone`]/[`revolve`] builders and removed with the
//! exact planar [`difference`], so hole walls are faceted into `segments` planar
//! facets (default [`DEFAULT_HOLE_SEGMENTS`] = 32) that carry their exact analytic
//! `Surface::Cylinder`/`Cone` tags by value — the kernel's honest
//! curved-through-boolean route. Every cutter overshoots the entry (and any exit)
//! face by 0.5 mm so a cut never leaves a coplanar zero-thickness membrane.
//!
//! Out-of-table sizes return a typed [`HoleError`] instead of nonsense geometry.

use std::f64::consts::TAU;

use kernel_core::math::{DAffine3, DMat3, DVec2, DVec3};

use crate::booleans::difference;
use crate::build::{cone, cylinder, revolve};
use crate::geom::perp_basis;
use crate::tessellate::tessellate_default;
use crate::topo::Solid;

/// Default angular faceting of every cutting tool (sectors per full turn).
pub const DEFAULT_HOLE_SEGMENTS: usize = 32;

/// Axial overshoot (mm) past the entry and exit planes so a cut pierces faces
/// cleanly instead of leaving a coplanar membrane for the boolean to chew on.
const PIERCE: f64 = 0.5;

/// How deep a drilled hole goes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum HoleDepth {
	/// A through hole: the bore spans `len` of material measured from `at` along
	/// the axis (the cutter overshoots both ends by 0.5 mm, so `len` must be at
	/// least the material extent below `at`).
	Through(f64),
	/// A blind hole: `depth` is the **full-diameter (usable) depth** — the figure a
	/// drawing dimensions — and the 118° drill point extends a further
	/// `(d/2)/tan 59° ≈ 0.300·d` past it.
	Blind(f64),
}

/// ISO 273:1979 clearance-hole series (the standard's *fine/medium/coarse*).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fit {
	/// Series *fine*, tolerance H12 — e.g. M5 → Ø5.3.
	Close = 0,
	/// Series *medium*, tolerance H13 — the default fit — e.g. M5 → Ø5.5.
	Medium = 1,
	/// Series *coarse*, tolerance H14 — e.g. M5 → Ø5.8.
	Coarse = 2,
}

/// Why a hole could not be cut. A typed channel so an AI (or feature tree) gets a
/// precise, actionable reason instead of a panic or silently-wrong geometry.
#[derive(Clone, Debug, PartialEq)]
pub enum HoleError {
	/// `at` (or the bolt-circle centre) has a non-finite coordinate.
	BadLocation,
	/// The axis is zero-length or non-finite, so "into the material" is undefined.
	BadAxis,
	/// A diameter is non-positive or non-finite.
	BadDiameter,
	/// A depth / through-length is non-positive or non-finite.
	BadDepth,
	/// A bolt circle of zero holes was requested.
	BadCount,
	/// The bolt-circle start angle is non-finite.
	BadAngle,
	/// The metric size is not in the supported table (see [`metric_hole_specs`];
	/// countersinks additionally require M3 or larger — DIN 74 form F starts there).
	UnsupportedSize {
		/// The requested nominal thread size in mm.
		m: f64,
	},
	/// The bearing designation is not in the seat table (see [`bearing_specs`]).
	UnknownBearing {
		/// The requested designation string.
		designation: String,
	},
}

impl std::fmt::Display for HoleError {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		match self {
			HoleError::BadLocation => write!(f, "hole location is non-finite"),
			HoleError::BadAxis => write!(f, "hole axis is zero or non-finite"),
			HoleError::BadDiameter => write!(f, "hole diameter must be positive and finite"),
			HoleError::BadDepth => write!(f, "hole depth must be positive and finite"),
			HoleError::BadCount => write!(f, "a bolt circle needs at least one hole"),
			HoleError::BadAngle => write!(f, "bolt-circle start angle is non-finite"),
			HoleError::UnsupportedSize { m } => write!(f, "no dimension-table entry for M{m}"),
			HoleError::UnknownBearing { designation } => write!(f, "no bearing-seat table entry for '{designation}'"),
		}
	}
}

impl std::error::Error for HoleError {}

// --- Dimension tables ----------------------------------------------------------

/// One row of the metric fastener hole table: every diameter the wizard needs for
/// a given nominal thread size, straight from the cited standards (all mm).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MetricHoleSpec {
	/// Nominal thread size (the "5" of M5).
	pub m: f64,
	/// ISO 261/262 coarse thread pitch — the tap-drill pilot is `m − pitch`.
	pub pitch: f64,
	/// ISO 273:1979 clearance-hole diameters `[close (H12), medium (H13), coarse (H14)]`.
	pub clearance: [f64; 3],
	/// DIN 974-1 counterbore diameter for a DIN 912 / ISO 4762 socket-head cap screw.
	pub counterbore_d: f64,
	/// DIN 974-1 counterbore depth `t1` (≥ the DIN 912 head height `k = m`, so the
	/// head sits flush, recessed by 0.2–0.8 mm).
	pub counterbore_depth: f64,
	/// DIN 74-1:2000-11 form F 90° countersink diameter `d2` for a DIN EN ISO 10642
	/// (DIN 7991) countersunk socket screw. `None` below M3 — form F starts at M3
	/// because ISO 10642 has no smaller sizes.
	pub countersink_d: Option<f64>,
}

/// The metric hole table, M2–M12 (the common machine-screw range).
///
/// Sources: ISO 273:1979 clearance holes as tabulated by
/// `zhonghuantools.com/en/resources/bolt-clearance-hole-chart`; ISO 261 coarse
/// pitches and the `tap drill = d − pitch` rule per `amesweb.info` /
/// `fractory.com` metric tap-drill charts; DIN 974-1 counterbores for DIN 912
/// screws per `engineersbible.com/counterbore-socket-din`; DIN 74-1:2000-11
/// table 3 (form F, 90°) countersinks from the standard text itself.
static METRIC_HOLE_TABLE: [MetricHoleSpec; 9] = [
	MetricHoleSpec { m: 2.0, pitch: 0.4, clearance: [2.2, 2.4, 2.6], counterbore_d: 4.4, counterbore_depth: 2.2, countersink_d: None },
	MetricHoleSpec { m: 2.5, pitch: 0.45, clearance: [2.7, 2.9, 3.1], counterbore_d: 5.5, counterbore_depth: 3.0, countersink_d: None },
	MetricHoleSpec { m: 3.0, pitch: 0.5, clearance: [3.2, 3.4, 3.6], counterbore_d: 6.5, counterbore_depth: 3.5, countersink_d: Some(7.5) },
	MetricHoleSpec { m: 4.0, pitch: 0.7, clearance: [4.3, 4.5, 4.8], counterbore_d: 8.0, counterbore_depth: 4.8, countersink_d: Some(10.0) },
	MetricHoleSpec { m: 5.0, pitch: 0.8, clearance: [5.3, 5.5, 5.8], counterbore_d: 10.0, counterbore_depth: 5.8, countersink_d: Some(12.5) },
	MetricHoleSpec { m: 6.0, pitch: 1.0, clearance: [6.4, 6.6, 7.0], counterbore_d: 11.0, counterbore_depth: 6.8, countersink_d: Some(14.5) },
	MetricHoleSpec { m: 8.0, pitch: 1.25, clearance: [8.4, 9.0, 10.0], counterbore_d: 15.0, counterbore_depth: 8.8, countersink_d: Some(19.0) },
	MetricHoleSpec { m: 10.0, pitch: 1.5, clearance: [10.5, 11.0, 12.0], counterbore_d: 18.0, counterbore_depth: 10.8, countersink_d: Some(23.5) },
	MetricHoleSpec { m: 12.0, pitch: 1.75, clearance: [13.0, 13.5, 14.5], counterbore_d: 20.0, counterbore_depth: 12.8, countersink_d: Some(28.0) },
];

/// All supported metric hole sizes, for table-driven callers and AIs listing
/// their vocabulary. Example: `metric_hole_specs().iter().map(|s| s.m)`.
pub fn metric_hole_specs() -> &'static [MetricHoleSpec] {
	&METRIC_HOLE_TABLE
}

/// Look up the table row for nominal size `m` (e.g. `5.0` for M5), or `None` if
/// the size is outside the supported set. Example: `metric_hole_spec(5.0).unwrap().clearance[1]` → `5.5`.
pub fn metric_hole_spec(m: f64) -> Option<&'static MetricHoleSpec> {
	METRIC_HOLE_TABLE.iter().find(|s| (s.m - m).abs() < 1e-9)
}

/// Nominal envelope of a deep-groove ball bearing: bore × outer Ø × width (mm).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BearingSpec {
	/// Standard designation, e.g. `"608"`.
	pub designation: &'static str,
	/// Bore (inner ring) diameter `d`.
	pub bore: f64,
	/// Outer ring diameter `D` — the seat pocket diameter.
	pub outer: f64,
	/// Ring width `B` — the seat pocket depth.
	pub width: f64,
}

/// Common small deep-groove ball bearings (d × D × B per the standard boundary
/// dimension charts, e.g. `bearingworks.com/bearing-sizes`,
/// `bearingsdirect.com` — 608: 8×22×7, 688: 8×16×5, 6804: 20×32×7, …).
static BEARING_TABLE: [BearingSpec; 8] = [
	BearingSpec { designation: "603", bore: 3.0, outer: 9.0, width: 5.0 },
	BearingSpec { designation: "693", bore: 3.0, outer: 8.0, width: 4.0 },
	BearingSpec { designation: "608", bore: 8.0, outer: 22.0, width: 7.0 },
	BearingSpec { designation: "625", bore: 5.0, outer: 16.0, width: 5.0 },
	BearingSpec { designation: "688", bore: 8.0, outer: 16.0, width: 5.0 },
	BearingSpec { designation: "6000", bore: 10.0, outer: 26.0, width: 8.0 },
	BearingSpec { designation: "6001", bore: 12.0, outer: 28.0, width: 8.0 },
	BearingSpec { designation: "6804", bore: 20.0, outer: 32.0, width: 7.0 },
];

/// All supported bearing seats. Example: `bearing_specs()[1].designation` → `"608"`.
pub fn bearing_specs() -> &'static [BearingSpec] {
	&BEARING_TABLE
}

/// Look up a bearing by designation, or `None` if it is not in the seat table.
/// Example: `bearing_spec("608").unwrap().outer` → `22.0`.
pub fn bearing_spec(designation: &str) -> Option<&'static BearingSpec> {
	BEARING_TABLE.iter().find(|b| b.designation == designation)
}

// --- Tool construction helpers --------------------------------------------------

/// Validate `at`/`axis` and return the unit cutting direction (into the material).
fn unit_axis(at: DVec3, axis: DVec3) -> Result<DVec3, HoleError> {
	if !at.is_finite() {
		return Err(HoleError::BadLocation);
	}
	axis.try_normalize().filter(|n| n.is_finite()).ok_or(HoleError::BadAxis)
}

/// `true` for a usable positive finite dimension.
fn positive(x: f64) -> bool {
	x.is_finite() && x > 0.0
}

/// Height of the 118° drill point below the full-diameter depth of a Ø`d` bore:
/// `(d/2) / tan 59° ≈ 0.300·d` (59° is the half of the 118° included point angle).
/// Public so callers can REPORT a blind hole's true total depth (full-diameter
/// depth + this) without re-deriving the tool geometry.
/// Example: `drill_tip_height(6.0)` → ≈ 1.803.
pub fn drill_tip_height(d: f64) -> f64 {
	d * 0.5 / 59.0_f64.to_radians().tan()
}

/// A faceted cylindrical cutter of diameter `d` spanning axis parameters
/// `[t0, t1]` measured from `at` along the unit `axis`.
fn rod(at: DVec3, axis: DVec3, d: f64, t0: f64, t1: f64, segments: usize) -> Solid {
	cylinder(at + axis * t0, axis, d * 0.5, t1 - t0, segments)
}

/// The blind-drill cutter: a Ø`d` shank from 0.5 mm above the entry plane down to
/// the full-diameter `depth`, ending in the 118° tip cone — one revolved profile,
/// watertight by construction, then placed at `at`/`axis`.
fn blind_drill_tool(at: DVec3, axis: DVec3, d: f64, depth: f64, segments: usize) -> Solid {
	let r = d * 0.5;
	let profile = [
		DVec2::new(0.0, -PIERCE),
		DVec2::new(r, -PIERCE),
		DVec2::new(r, depth),
		DVec2::new(0.0, depth + drill_tip_height(d)),
	];
	let (e1, e2) = perp_basis(axis);
	// Local frame: revolve() builds about +Z, so map Z onto the cutting axis.
	revolve(&profile, segments).transformed(DAffine3::from_mat3_translation(DMat3::from_cols(e1, e2, axis), at))
}

/// Axis-parameter extent `[t_min, t_max]` of the solid's AABB corners measured
/// from `at` along the unit `axis` — how far a "through everything" cutter must run.
fn axis_extent(solid: &Solid, at: DVec3, axis: DVec3) -> (f64, f64) {
	let (lo, hi) = solid.aabb();
	let (mut t_min, mut t_max) = (f64::INFINITY, f64::NEG_INFINITY);
	for i in 0..8 {
		let corner = DVec3::new(
			if i & 1 == 0 { lo.x } else { hi.x },
			if i & 2 == 0 { lo.y } else { hi.y },
			if i & 4 == 0 { lo.z } else { hi.z },
		);
		let t = (corner - at).dot(axis);
		t_min = t_min.min(t);
		t_max = t_max.max(t);
	}
	(t_min, t_max)
}

/// Cut a Ø`d` bore through the solid's **entire** extent along `axis` (clearance
/// holes are by definition through-holes). An empty solid passes through unchanged.
fn cut_through_all(solid: &Solid, at: DVec3, axis: DVec3, d: f64, segments: usize) -> Solid {
	let (t_min, t_max) = axis_extent(solid, at, axis);
	difference(solid, &rod(at, axis, d, t_min - PIERCE, t_max + PIERCE, segments))
}

// --- The hole vocabulary ---------------------------------------------------------

/// Drill a plain hole of diameter `d` at `at`, cutting along `axis` (which points
/// into the material). [`HoleDepth::Through`] bores straight through `len` of
/// material; [`HoleDepth::Blind`] stops at the full-diameter `depth` and ends in
/// the standard 118° drill-point cone (twist-drill included point angle), composed
/// as one revolved cylinder-plus-cone cutter. `segments` facets the tool
/// (`None` → 32).
///
/// Example: `drill(&plate, DVec3::new(20.0, 15.0, 8.0), -DVec3::Z, 6.0, HoleDepth::Blind(5.0), None)?`.
pub fn drill(solid: &Solid, at: DVec3, axis: DVec3, d: f64, depth: HoleDepth, segments: Option<usize>) -> Result<Solid, HoleError> {
	let axis = unit_axis(at, axis)?;
	if !positive(d) {
		return Err(HoleError::BadDiameter);
	}
	let segments = segments.unwrap_or(DEFAULT_HOLE_SEGMENTS);
	let tool = match depth {
		HoleDepth::Through(len) => {
			if !positive(len) {
				return Err(HoleError::BadDepth);
			}
			rod(at, axis, d, -PIERCE, len + PIERCE, segments)
		}
		HoleDepth::Blind(depth) => {
			if !positive(depth) {
				return Err(HoleError::BadDepth);
			}
			blind_drill_tool(at, axis, d, depth, segments)
		}
	};
	Ok(difference(solid, &tool))
}

/// Cut a clearance hole for an M-`m` screw through the solid's **whole** extent
/// along `axis` (a clearance hole passes the screw through, so it is always a
/// through-hole here). Diameter per ISO 273:1979 — e.g. M5 → Ø5.3 / 5.5 / 5.8 for
/// [`Fit::Close`] / [`Fit::Medium`] / [`Fit::Coarse`]. Supported sizes: M2, M2.5,
/// M3, M4, M5, M6, M8, M10, M12.
///
/// Example: `clearance_hole(&plate, at, -DVec3::Z, 5.0, Fit::Medium, None)?`.
pub fn clearance_hole(solid: &Solid, at: DVec3, axis: DVec3, m: f64, fit: Fit, segments: Option<usize>) -> Result<Solid, HoleError> {
	let axis = unit_axis(at, axis)?;
	let spec = metric_hole_spec(m).ok_or(HoleError::UnsupportedSize { m })?;
	Ok(cut_through_all(solid, at, axis, spec.clearance[fit as usize], segments.unwrap_or(DEFAULT_HOLE_SEGMENTS)))
}

/// Cut an ISO 273 clearance hole plus the DIN 974-1 counterbore that recesses a
/// DIN 912 / ISO 4762 socket-head cap screw flush: e.g. M5 → Ø10 counterbore,
/// 5.8 mm deep (≥ the 5 mm head height). The counterbore depth is measured from
/// `at`, so place `at` on the entry face. Supported sizes: M2–M12 as in
/// [`metric_hole_specs`].
///
/// Example: `counterbore_hole(&plate, at, -DVec3::Z, 5.0, Fit::Close, None)?`.
pub fn counterbore_hole(solid: &Solid, at: DVec3, axis: DVec3, m: f64, fit: Fit, segments: Option<usize>) -> Result<Solid, HoleError> {
	let axis = unit_axis(at, axis)?;
	let spec = metric_hole_spec(m).ok_or(HoleError::UnsupportedSize { m })?;
	let segments = segments.unwrap_or(DEFAULT_HOLE_SEGMENTS);
	let through = cut_through_all(solid, at, axis, spec.clearance[fit as usize], segments);
	let cb = rod(at, axis, spec.counterbore_d, -PIERCE, spec.counterbore_depth, segments);
	Ok(difference(&through, &cb))
}

/// Cut an ISO 273 clearance hole plus the DIN 74-1 form F 90° countersink that
/// seats a DIN EN ISO 10642 (DIN 7991) countersunk socket screw flush: e.g.
/// M5 → 90° sink to Ø12.5 at the entry plane. Supported sizes: M3–M12 (DIN 74
/// form F starts at M3; M2/M2.5 return [`HoleError::UnsupportedSize`]).
///
/// Example: `countersink_hole(&plate, at, -DVec3::Z, 5.0, Fit::Medium, None)?`.
pub fn countersink_hole(solid: &Solid, at: DVec3, axis: DVec3, m: f64, fit: Fit, segments: Option<usize>) -> Result<Solid, HoleError> {
	let axis = unit_axis(at, axis)?;
	let spec = metric_hole_spec(m).ok_or(HoleError::UnsupportedSize { m })?;
	let dk = spec.countersink_d.ok_or(HoleError::UnsupportedSize { m })?;
	let segments = segments.unwrap_or(DEFAULT_HOLE_SEGMENTS);
	let through = cut_through_all(solid, at, axis, spec.clearance[fit as usize], segments);
	// A 45°-flank cone with Ø dk exactly at the entry plane; extending it 0.5 mm
	// above the surface widens it by the same amount, like plunging the real
	// 90° countersink tool. Its apex (at depth dk/2) is inside the already-cut
	// bore, so the cut adds exactly the conical frustum down to the bore wall.
	let csk = cone(at - axis * PIERCE, axis, dk * 0.5 + PIERCE, dk * 0.5 + PIERCE, segments);
	Ok(difference(&through, &csk))
}

/// Drill the tap-drill pilot hole for an ISO metric **coarse** M-`m` thread:
/// pilot Ø = `m − pitch` (the standard 100%-thread tapping size — e.g. M6×1 → Ø5;
/// machinist charts round M8 → 6.8 and M12 → 10.2, we cut the exact 6.75/10.25).
/// Blind holes get the 118° drill point via [`drill`]. The thread itself is
/// cosmetic/manufacturing detail and is **not** modelled — this is the pilot bore
/// only. Supported sizes: M2–M12 as in [`metric_hole_specs`].
///
/// Example: `tap_drill_hole(&block, at, -DVec3::Z, 6.0, HoleDepth::Blind(12.0), None)?`.
pub fn tap_drill_hole(solid: &Solid, at: DVec3, axis: DVec3, m: f64, depth: HoleDepth, segments: Option<usize>) -> Result<Solid, HoleError> {
	let spec = metric_hole_spec(m).ok_or(HoleError::UnsupportedSize { m })?;
	drill(solid, at, axis, spec.m - spec.pitch, depth, segments)
}

/// Apply any hole cut at `n` equally spaced positions on a bolt circle of
/// **diameter** `circle_d` (the drawing's BCD) centred at `center` in the plane
/// perpendicular to `axis`. `start_angle` (radians) offsets the first hole from
/// the deterministic in-plane reference direction (`perp_basis(axis)`, +X for a
/// Z axis), increasing right-handed about `axis`. `cut` is a closure — chosen over
/// an enum so ANY vocabulary function (or a custom cut) composes by partial
/// application; the running solid is threaded through it and errors propagate.
///
/// Example: `bolt_circle(&plate, c, -DVec3::Z, 50.0, 6, 0.0, |s, p| clearance_hole(&s, p, -DVec3::Z, 5.0, Fit::Medium, None))?`.
pub fn bolt_circle(
	solid: &Solid,
	center: DVec3,
	axis: DVec3,
	circle_d: f64,
	n: usize,
	start_angle: f64,
	mut cut: impl FnMut(Solid, DVec3) -> Result<Solid, HoleError>,
) -> Result<Solid, HoleError> {
	let axis = unit_axis(center, axis)?;
	if !positive(circle_d) {
		return Err(HoleError::BadDiameter);
	}
	if n == 0 {
		return Err(HoleError::BadCount);
	}
	if !start_angle.is_finite() {
		return Err(HoleError::BadAngle);
	}
	let (e1, e2) = perp_basis(axis);
	let mut result = solid.clone();
	for k in 0..n {
		let a = start_angle + TAU * k as f64 / n as f64;
		result = cut(result, center + (e1 * a.cos() + e2 * a.sin()) * (circle_d * 0.5))?;
	}
	Ok(result)
}

/// The 2D outline of a **teardrop** hole of diameter `d` for FDM printing: a
/// circle whose top is replaced by two straight roof flanks at `roof_deg` from
/// horizontal meeting in an apex at height `(d/2)/cos(roof_deg)` above centre —
/// so a hole bored along a HORIZONTAL axis self-supports (no sagging ceiling
/// arc), the standard support-free idiom for horizontal holes. `+v` in the
/// returned CCW profile is the print-up direction. The kept arc is faceted into
/// `segments` sectors; the arc between the two roof tangent points is replaced
/// by the flanks. Returns `None` for a non-positive/non-finite diameter or a
/// `roof_deg` outside (0°, 90°).
pub fn teardrop_profile(d: f64, roof_deg: f64, segments: usize) -> Option<Vec<DVec2>> {
	if !positive(d) || !roof_deg.is_finite() || roof_deg <= 0.0 || roof_deg >= 90.0 {
		return None;
	}
	let r = d * 0.5;
	let roof = roof_deg.to_radians();
	// Roof flanks are tangent to the circle at polar angle ±(90° − roof) about +v
	// and meet in an apex at (0, r / cos roof) — distance from a line through the
	// apex with slope ∓tan(roof) to the centre equals r exactly there.
	let alpha = std::f64::consts::FRAC_PI_2 + roof; // left tangent point
	let n = segments.max(8);
	let mut pts = Vec::with_capacity(n + 2);
	// CCW: from the left tangent point under the bottom to the right tangent
	// point (increasing angle), then the apex; closing edge = left roof flank.
	let span = TAU - 2.0 * roof;
	for k in 0..=n {
		let a = alpha + span * (k as f64 / n as f64);
		pts.push(DVec2::new(r * a.cos(), r * a.sin()));
	}
	pts.push(DVec2::new(0.0, r / roof.cos())); // apex
	Some(pts)
}

/// Cut a **teardrop hole**: a bore of diameter `d` whose ceiling is a two-flank
/// roof (use `roof_deg` = 46° — just past the 45° FDM overhang limit) so the
/// hole prints support-free when its `axis` lies horizontal on the build plate.
/// `up` is the print-up direction (must not be parallel to `axis`); the roof
/// apex points along `up`. The cutter runs `len` along `axis` from `at`,
/// overshooting both ends by 0.5 mm like every hole in this module — pass the
/// material extent for a through hole, or `depth − 0.5` for a blind pocket. The
/// circle is kept intact below the tangent points, so pins, magnets and screws
/// still seat on the full round bearing surface.
///
/// Example: `teardrop_hole(&wall, face_pt, DVec3::Y, DVec3::Z, 6.2, 2.9, 46.0, None)?`
/// — a 2.4-deep magnet pocket in a vertical wall that prints without support.
#[allow(clippy::too_many_arguments)] // a hole + its print-up direction and roof angle
pub fn teardrop_hole(
	solid: &Solid,
	at: DVec3,
	axis: DVec3,
	up: DVec3,
	d: f64,
	len: f64,
	roof_deg: f64,
	segments: Option<usize>,
) -> Result<Solid, HoleError> {
	let axis = unit_axis(at, axis)?;
	if !positive(len) {
		return Err(HoleError::BadDepth);
	}
	let up = up.normalize_or_zero();
	if up == DVec3::ZERO || up.cross(axis).length() < 1e-9 {
		return Err(HoleError::BadAxis);
	}
	let profile = teardrop_profile(d, roof_deg, segments.unwrap_or(DEFAULT_HOLE_SEGMENTS)).ok_or(HoleError::BadDiameter)?;
	// Profile basis: v = print-up projected perpendicular to the bore axis,
	// u = v × axis (right-handed (u, v, axis) keeps the CCW profile outward).
	let v = (up - axis * up.dot(axis)).normalize();
	let u = v.cross(axis);
	let tool = crate::build::extrude(&profile, len + 2.0 * PIERCE)
		.transformed(DAffine3::from_mat3_translation(DMat3::from_cols(u, v, axis), at - axis * PIERCE));
	Ok(difference(solid, &tool))
}

/// Cut a seat for a standard deep-groove ball bearing: a flat-bottom pocket of
/// the bearing's outer Ø `D` and width `B` (nominal — press/slip-fit allowance is
/// the caller's offset), plus a concentric **shoulder bore** through the rest of
/// the material for shaft passage and inner-ring relief. The shoulder bore takes
/// the mean `(d + D)/2` — a generic relief that still seats the outer ring on a
/// `(D − d)/4` ledge; consult the maker's da/Da abutment tables for critical
/// designs. Supported designations: see [`bearing_specs`] (603, 608, 625, 688,
/// 6000, 6001, 6804).
///
/// Example: `bearing_seat(&housing, at, -DVec3::Z, "608", None)?` → Ø22 × 7 pocket + Ø15 bore.
pub fn bearing_seat(solid: &Solid, at: DVec3, axis: DVec3, bearing: &str, segments: Option<usize>) -> Result<Solid, HoleError> {
	let axis = unit_axis(at, axis)?;
	let spec = bearing_spec(bearing).ok_or_else(|| HoleError::UnknownBearing { designation: bearing.to_string() })?;
	let segments = segments.unwrap_or(DEFAULT_HOLE_SEGMENTS);
	let pocket = difference(solid, &rod(at, axis, spec.outer, -PIERCE, spec.width, segments));
	Ok(cut_through_all(&pocket, at, axis, (spec.bore + spec.outer) * 0.5, segments))
}

// --- Advisory interrogation -------------------------------------------------------

/// Angular stations sampled around the would-be bore wall by [`min_ligament`].
const LIGAMENT_RING_SAMPLES: usize = 64;

/// Advisory **minimum-ligament echo** (FRICTION #21): the thinnest remaining
/// material between a PLANNED Ø`d` hole's wall and the solid's existing
/// boundary, estimated **before** any cut. Purely a measurement — no wizard
/// cut's behaviour changes; callers gate on the returned value (e.g. refuse or
/// warn when it is below a wall-thickness rule).
///
/// **Exactly what is measured.** [`LIGAMENT_RING_SAMPLES`] points on the
/// would-be bore cylinder (radius `d/2` about the `at`+`axis` line) on ONE ring
/// at the **mid-span** of the solid's extent along the axis; for each, the
/// Euclidean distance to the solid's current boundary via
/// [`tessellate_default`] + exact per-triangle closest point
/// (`Mesh::closest_point`); the echo is the minimum. Sampling ON the wall —
/// not the axis — is what makes the bore radius "already subtracted": for a
/// wall sample the closest-point distance IS the wall-to-boundary gap, whereas
/// an axis sample's distance minus `d/2` misreads every boundary that is not
/// perpendicular to the axis.
///
/// Honest caveats, in decreasing order of consequence:
/// - Faces the bore will PIERCE (entry/exit) are part of the current boundary,
///   so the echo is **clamped above by the mid-span depth** (≈ half the
///   material span along the axis). A returned value near `span/2` therefore
///   means "no lateral ligament thinner than the mid-depth", not a precise
///   wall reading — the thin-web warning regime (ligament ≪ depth), which is
///   what this echo exists for, is unaffected. One mid-span ring (rather than
///   several depths) maximises that lateral sensitivity; for prismatic walls
///   the lateral gap is depth-independent anyway.
/// - It is a SAMPLED estimate: 64 stations on one ring. The angular
///   quantisation error against a flat wall is ≤ `(d/2)·(1 − cos(π/64))`
///   ≈ `0.0012·d` (negligible); a boundary feature confined strictly between
///   ring depths (e.g. a mid-height side pocket above/below the ring) is not
///   seen.
/// - Curved boundaries are measured to their default-tessellation chords.
/// - The bore is treated as a THROUGH hole over the solid's whole axis extent;
///   blind-hole floor ligaments (material under a planned tip) are out of
///   scope — there is no depth parameter.
///
/// Returns `f64::NAN` for a degenerate question (non-finite `at`, zero `axis`,
/// non-positive `d`, or no material extent along `axis` from `at`) and
/// `f64::INFINITY` for an empty solid (no boundary to measure against).
///
/// Example: `min_ligament(&plate, DVec3::new(5.0, 15.0, 12.0), -DVec3::Z, 6.0)`
/// → ≈ 2.0 for a Ø6 hole centred 5 mm from the plate edge.
pub fn min_ligament(solid: &Solid, at: DVec3, axis: DVec3, d: f64) -> f64 {
	let Ok(axis) = unit_axis(at, axis) else { return f64::NAN };
	if !positive(d) {
		return f64::NAN;
	}
	let (t_min, t_max) = axis_extent(solid, at, axis);
	let t_lo = t_min.max(0.0); // material begins at the entry face, never behind `at`
	if t_max.is_nan() || t_max <= t_lo {
		return f64::NAN; // no material along +axis from `at`
	}
	let mesh = tessellate_default(solid);
	if mesh.triangle_count() == 0 {
		return f64::INFINITY;
	}
	let center = at + axis * (0.5 * (t_lo + t_max));
	let (e1, e2) = perp_basis(axis);
	let mut min_gap = f64::INFINITY;
	for k in 0..LIGAMENT_RING_SAMPLES {
		let a = TAU * k as f64 / LIGAMENT_RING_SAMPLES as f64;
		let p = center + (e1 * a.cos() + e2 * a.sin()) * (d * 0.5);
		if let Some(cp) = mesh.closest_point(p.as_vec3()) {
			min_gap = min_gap.min(cp.distance as f64);
		}
	}
	min_gap
}
