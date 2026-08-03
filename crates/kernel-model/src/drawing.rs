// Copyright (c) LMCAD. Licensed under the MIT License.

//! Dimensioned 2-D drawings — orthographic views, a hatched cut section, and
//! **measured** dimensions on a deterministic SVG / DXF sheet, so a machine
//! shop or casting house can quote from a deliverable instead of guessing at
//! an STL.
//!
//! DESIGN_GUIDE §24 item 9 ledgers drawings as *out of scope*. This module
//! changes that for **one honestly-bounded slice**, described below. Read the
//! limits before you send a sheet to a vendor.
//!
//! # What this ships
//!
//! - [`project_view`] — an orthographic projection of an exact [`Solid`] along
//!   one of four [`ViewDir`]s (third-angle front / top / right, plus an
//!   isometric pictorial). Real edges only: **tessellation facet seams are
//!   suppressed** (an edge whose two faces carry the *same* analytic surface is
//!   not an edge of the exact geometry) **unless that seam is the silhouette**
//!   for the view — so a Ø20 cylinder draws two silhouette lines, not 32
//!   generatrices. A coaxial bore/boss group seen down its own axis is emitted
//!   as a **true circle** at its analytic radius, replacing the faceted rim
//!   chords.
//! - **Hidden-line removal, sampled** ([`Visibility::RaySampled`]): each
//!   candidate edge is diced into at most [`MAX_HLR_SAMPLES`] sub-segments and
//!   each sub-segment's midpoint is classified by casting a ray from outside the
//!   model *toward* that midpoint along the sight direction against the
//!   tessellated mesh's BVH. Occluded runs are **dropped**. See the limits
//!   below — this is not exact HLR and the sheet says so.
//! - [`section_view`] — a cut-plane section with 45° [`SectionView::hatch`]
//!   clipped even-odd against the cut boundary, carrying the exact analytic
//!   conic counts from [`kernel_brep::section_curves_with_fallback`] alongside.
//! - [`Dimension`] / [`measure_dimension`] / [`auto_dimensions`] — every value
//!   comes from an engine measure of *this* model and records
//!   [`Dimension::from_measure`], the name of the measure that produced it. A
//!   drawn-but-unmeasured number is a lie; this module contains no hand-typed
//!   dimension value anywhere, and a request naming a feature the model does not
//!   have is a typed [`DrawingError::FeatureNotFound`], never a plausible
//!   number.
//! - [`Drawing::to_svg`] / [`Drawing::to_dxf`] — byte-stable output. Fixed
//!   4-decimal coordinate formatting with negative zero normalized, ordered
//!   emission throughout (no hash-map iteration), and **no clock read**: the
//!   title block's date is supplied by the caller ([`TitleBlock::date`]).
//!
//! # Limits — state these to whoever reads the sheet
//!
//! 1. **Hidden detail is not drawn.** Hidden edges are removed, not dashed. A
//!    bore behind a wall leaves no trace in that view. [`HLR_NOTE`] carries this
//!    sentence onto every sheet and [`Drawing::to_svg`] always emits it.
//! 2. **Occlusion is sampled, not exact.** A visibility change lands within one
//!    sample interval (edge length ÷ its sample count) of its true position, and
//!    an occluder thinner than [`ViewOptions::visibility_epsilon`] along the
//!    sight line is not seen. The classification runs against the `f32`
//!    tessellation; the *dimension values* never do — they come from the `f64`
//!    B-rep.
//! 3. **No GD&T**, no feature control frames, no surface-finish symbols, no
//!    thread callouts, no auxiliary / detail views, no broken-out sections.
//! 4. **Dimensions are not auto-placed to drafting standard.** Three overall
//!    extents get real dimension lines and each bore gets a Ø leader; *every*
//!    dimension — including the ones with no graphical placement — is listed in
//!    the sheet's **dimension schedule** with its value, class, subject and
//!    originating measure. The schedule, not the leader, is the auditable
//!    artifact.
//! 5. **Curved section boundaries are chord-accurate.** The hatched boundary is
//!    chained from the face-polygon cut (exact for planar walls); the analytic
//!    conics are counted separately in [`SectionView::exact_curves`].
//! 6. **Edges tangent to the sight line read as visible.** An edge lying in a
//!    surface the sight direction grazes (a bore's rim chord sitting exactly in
//!    the part's own top face, seen from the front) is on the silhouette, and
//!    the ray test classifies it visible. It is drawn *on top of* the
//!    silhouette line it coincides with, which the collinear merge
//!    ([`ViewOptions::merge_collinear`]) then absorbs into that one line — but
//!    the underlying classification is a tangency, not a decision.
//!
//! # The `process` seam
//!
//! A sheet's general-tolerance note is *manufacturing* data, not geometry. This
//! module takes it from the caller through [`GeneralTolerance`] and never reads
//! [`crate::process`]: an [`crate::process::FdmProfile`] is the natural future
//! source (it already carries the measured clearance bands), and wiring it is a
//! one-line `impl GeneralTolerance for FdmProfile` on the process side. Keeping
//! the dependency out of `drawing` means a drawing can be produced for a part
//! whose process is not yet chosen — and it keeps this module honest that the
//! number is an *input*, not a measurement.
//!
//! # Example
//!
//! ```
//! use kernel_brep::cuboid;
//! use kernel_brep::math::DVec3;
//! use kernel_model::drawing::{auto_dimensions, project_view, Drawing, FixedTolerance, TitleBlock, ViewDir, ViewOptions};
//!
//! let plate = cuboid(DVec3::ZERO, DVec3::new(80.0, 40.0, 10.0));
//! let view = project_view(&plate, ViewDir::Front, &ViewOptions::default());
//! let title = TitleBlock::new("PLATE", "2026-07-30", &FixedTolerance::new(0.2, "caller-supplied"));
//! let sheet = Drawing::new(title).with_view(view).with_dimensions(auto_dimensions(&plate));
//! let svg = sheet.to_svg();
//! assert_eq!(svg, sheet.to_svg(), "SVG must be byte-identical across runs");
//! assert!(svg.contains("VISIBLE EDGES ONLY"), "the hidden-line limit must reach the sheet");
//! ```

use std::fmt::Write as _;

use kernel_brep::geom::perp_basis;
use kernel_brep::math::{DVec2, DVec3};
use kernel_brep::{bounding_box, section_curves_with_fallback, section_properties, tessellate_default, SectionCurve, Solid, Surface};
use kernel_core::math::Ray;
use kernel_core::MeshBvh;

// ---------------------------------------------------------------------------
// Notes that MUST reach the sheet
// ---------------------------------------------------------------------------

/// The hidden-line limitation, printed on **every** sheet by
/// [`Drawing::to_svg`] and [`Drawing::to_dxf`]. A drawing that silently omits
/// hidden detail is a trap for whoever quotes it; this sentence is the price of
/// shipping visible-edge-only views.
pub const HLR_NOTE: &str = "VISIBLE EDGES ONLY - hidden detail is NOT drawn (no dashed hidden lines). Occlusion is resolved by ray sampling against the tessellated mesh, so a visibility change lands within one sample interval of its true position. Verify internal features against the 3-D model or the section view.";

/// The units + general-tolerance note template. `{tol}` and `{src}` are filled
/// from the [`TitleBlock`].
pub const UNITS_NOTE_TEMPLATE: &str = "ALL DIMENSIONS IN MILLIMETRES. GENERAL TOLERANCE +/-{tol} mm (source: {src}). Dimension values are measured from the model; the DIMENSION SCHEDULE names the measure behind each one.";

/// The provenance note — where the numbers on this sheet come from.
pub const PROVENANCE_NOTE: &str = "Every dimension value on this sheet is produced by an engine measure of the model (kernel_brep bounding_box / analytic Surface tags); none is a hand-typed literal.";

/// Upper bound on hidden-line-removal samples per edge. An edge is diced into
/// at most this many sub-segments for the visibility test, so the worst-case
/// error on an occlusion boundary is `edge_length / MAX_HLR_SAMPLES`.
pub const MAX_HLR_SAMPLES: usize = 48;

/// Lower bound on hidden-line-removal samples per edge, so a short fully
/// occluded stub still gets classified at three points instead of one.
pub const MIN_HLR_SAMPLES: usize = 3;

// ---------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------

/// Typed refusals from the drawing layer. A drawing never invents a value: if a
/// dimension cannot be *measured*, the request fails here.
#[derive(Clone, Debug, PartialEq)]
pub enum DrawingError {
	/// A dimension was requested for a feature this model does not have.
	/// `available` describes what the model *does* have, so the caller can fix
	/// the request instead of guessing.
	FeatureNotFound {
		/// What was asked for, e.g. `"bore #7"`.
		requested: String,
		/// What the model actually offers, e.g. `"2 bore(s), 1 round boss(es)"`.
		available: String,
	},
	/// The measure is a real concept but is not determinable from this geometry
	/// (e.g. an annular wall for a bore with no enclosing coaxial boss).
	NotDeterminable {
		/// The measure that was attempted.
		measure: &'static str,
		/// Which feature it was attempted on.
		subject: String,
		/// Why the geometry does not support it.
		why: &'static str,
	},
	/// A caller-supplied parameter is outside its usable range.
	BadInput {
		/// Parameter name.
		field: &'static str,
		/// Offending value.
		got: f64,
		/// The rule it broke.
		why: &'static str,
	},
	/// The requested cut plane produced no section — it misses the solid.
	EmptySection {
		/// The plane's point.
		point: DVec3,
		/// The plane's normal.
		normal: DVec3,
	},
}

impl std::fmt::Display for DrawingError {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		match self {
			DrawingError::FeatureNotFound { requested, available } => write!(
				f,
				"cannot dimension {requested}: this model has no such feature ({available}) — a drawing never invents a dimension value"
			),
			DrawingError::NotDeterminable { measure, subject, why } => write!(
				f,
				"measure '{measure}' is not determinable for {subject}: {why} — refusing rather than drawing an unmeasured number"
			),
			DrawingError::BadInput { field, got, why } => write!(f, "drawing parameter '{field}' = {got} is unusable: {why}"),
			DrawingError::EmptySection { point, normal } => write!(
				f,
				"section plane through ({}, {}, {}) with normal ({}, {}, {}) cuts no material — no section view produced",
				point.x, point.y, point.z, normal.x, normal.y, normal.z
			),
		}
	}
}

impl std::error::Error for DrawingError {}

// ---------------------------------------------------------------------------
// Deterministic number formatting
// ---------------------------------------------------------------------------

/// Coordinate text for SVG/DXF: fixed 4 decimals with negative zero normalized
/// to `0.0000`. Fixed-width decimal output is what makes the files byte-stable
/// (no shortest-round-trip formatting, no locale, no exponent form).
fn mm(x: f64) -> String {
	let s = format!("{:.4}", if x == 0.0 { 0.0 } else { x });
	if s == "-0.0000" {
		"0.0000".to_string()
	} else {
		s
	}
}

/// Dimension text: fixed 3 decimals with negative zero normalized.
fn dim_text(x: f64) -> String {
	let s = format!("{:.3}", if x == 0.0 { 0.0 } else { x });
	if s == "-0.000" {
		"0.000".to_string()
	} else {
		s
	}
}

/// XML escape for SVG text content and attribute values.
fn xml_escape(s: &str) -> String {
	let mut out = String::with_capacity(s.len());
	for c in s.chars() {
		match c {
			'&' => out.push_str("&amp;"),
			'<' => out.push_str("&lt;"),
			'>' => out.push_str("&gt;"),
			'"' => out.push_str("&quot;"),
			'\'' => out.push_str("&apos;"),
			_ => out.push(c),
		}
	}
	out
}

/// Total-order comparison chain over `f64` keys — the deterministic sort this
/// module uses everywhere (no partial-order surprises, identical across runs).
fn cmp_keys(a: &[f64], b: &[f64]) -> std::cmp::Ordering {
	for (x, y) in a.iter().zip(b.iter()) {
		let o = x.total_cmp(y);
		if o != std::cmp::Ordering::Equal {
			return o;
		}
	}
	std::cmp::Ordering::Equal
}

// ---------------------------------------------------------------------------
// View direction and projection
// ---------------------------------------------------------------------------

/// A world axis, for the dimension requests that name one.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Axis {
	/// World `X`.
	X,
	/// World `Y`.
	Y,
	/// World `Z`.
	Z,
}

impl Axis {
	/// Stable lowercase name (`"x"` / `"y"` / `"z"`).
	pub fn name(self) -> &'static str {
		match self {
			Axis::X => "x",
			Axis::Y => "y",
			Axis::Z => "z",
		}
	}

	/// The unit direction.
	pub fn dir(self) -> DVec3 {
		match self {
			Axis::X => DVec3::X,
			Axis::Y => DVec3::Y,
			Axis::Z => DVec3::Z,
		}
	}

	/// Component of `v` along this axis.
	pub fn of(self, v: DVec3) -> f64 {
		match self {
			Axis::X => v.x,
			Axis::Y => v.y,
			Axis::Z => v.z,
		}
	}
}

/// Which way an orthographic view looks. The three orthogonal views are the
/// **third-angle** set (the sheet arranges TOP above FRONT and RIGHT beside
/// FRONT); [`ViewDir::Iso`] is a pictorial aid, never a dimensioned view.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ViewDir {
	/// Looking along `+Y`: the X–Z plane, model `+X` right and `+Z` up.
	Front,
	/// Looking along `−Z` (down): the X–Y plane, model `+X` right and `+Y` up.
	Top,
	/// Looking along `−X`: the Y–Z plane, model `+Y` right and `+Z` up.
	Right,
	/// Standard isometric pictorial, viewed from `(+1, −1, +1)`.
	Iso,
}

impl ViewDir {
	/// Stable uppercase sheet label.
	pub fn label(self) -> &'static str {
		match self {
			ViewDir::Front => "FRONT",
			ViewDir::Top => "TOP",
			ViewDir::Right => "RIGHT",
			ViewDir::Iso => "ISO",
		}
	}

	/// The orthonormal view frame `(right, up, sight)`, where `sight` points
	/// **from the viewer into the scene** and `right × up = −sight` (so the
	/// projected 2-D frame is right-handed as drawn).
	pub fn frame(self) -> (DVec3, DVec3, DVec3) {
		match self {
			ViewDir::Front => (DVec3::X, DVec3::Z, DVec3::Y),
			ViewDir::Top => (DVec3::X, DVec3::Y, -DVec3::Z),
			ViewDir::Right => (DVec3::Y, DVec3::Z, -DVec3::X),
			ViewDir::Iso => {
				let sight = DVec3::new(-1.0, 1.0, -1.0).normalize();
				let right = DVec3::new(1.0, 1.0, 0.0).normalize();
				let up = (-sight).cross(right).normalize();
				(right, up, sight)
			}
		}
	}
}

/// How a [`View`]'s visibility was decided — recorded on the view so the sheet
/// can state it and a gate can assert it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Visibility {
	/// Hidden-line removal by ray sampling against the tessellated mesh: each
	/// edge diced into at most `max_samples_per_edge` sub-segments, occluded
	/// runs dropped. Approximate at the sample interval; see [`HLR_NOTE`].
	RaySampled {
		/// The dicing cap actually used.
		max_samples_per_edge: usize,
	},
	/// No occlusion test at all — every candidate edge drawn (a wireframe).
	/// Honest, and useless for quoting; available for debugging.
	Wireframe,
}

/// Knobs for [`project_view`].
#[derive(Clone, Copy, Debug)]
pub struct ViewOptions {
	/// Run the sampled hidden-line removal (`true`, the default) or emit a raw
	/// wireframe (`false`).
	pub hidden_line_removal: bool,
	/// Occluder-thickness floor along the sight line, as a **fraction of the
	/// model's bounding-box diagonal**. Material nearer than this to the edge
	/// does not count as occluding it (numerically it is the edge's own
	/// surface). Default 1e-3.
	pub visibility_epsilon: f64,
	/// Emit a true circle for a coaxial cylinder group whose axis is parallel to
	/// the sight direction, replacing its faceted rim chords. Default `true`.
	pub analytic_circles: bool,
	/// Drop projected segments shorter than this (mm) — a generatrix seen end-on
	/// projects to a point. Default 1e-9.
	pub min_segment_mm: f64,
	/// Merge projected segments that land on the same infinite line into single
	/// segments (`true`, the default). Many distinct model edges legitimately
	/// project onto one drawn line — the fragments of a coalesced face, or a
	/// bore's rim chords seen edge-on in the plane of the part's own top
	/// surface — and a drawing shows that as ONE line, not forty. Purely a
	/// rendering merge: the edge-level receipts
	/// ([`View::edges_considered`] / [`View::edges_visible`] /
	/// [`View::edges_hidden`]) are counted before it runs.
	pub merge_collinear: bool,
	/// Tolerance (mm) for "same line" and "touching" in the collinear merge.
	/// Default 1e-9.
	pub collinear_tol: f64,
}

impl Default for ViewOptions {
	fn default() -> Self {
		ViewOptions {
			hidden_line_removal: true,
			visibility_epsilon: 1e-3,
			analytic_circles: true,
			min_segment_mm: 1e-9,
			merge_collinear: true,
			collinear_tol: 1e-9,
		}
	}
}

/// One 2-D entity of a projected view, in **view millimetres** (model units,
/// unscaled; the sheet applies placement and scale).
#[derive(Clone, Debug, PartialEq)]
pub enum ViewEntity {
	/// A straight segment.
	Segment {
		/// Start point.
		a: DVec2,
		/// End point.
		b: DVec2,
	},
	/// A true circle at an analytic radius — a coaxial cylinder group seen down
	/// its own axis. Kept as a circle so SVG and DXF carry an exact arc.
	Circle {
		/// Projected axis position.
		center: DVec2,
		/// The analytic `Surface::Cylinder` radius.
		radius: f64,
	},
	/// A closed chained polyline (section boundaries).
	Polyline(Vec<DVec2>),
}

/// An orthographic projection of a solid: the drawn entities plus the receipts
/// a gate can pin (how many model edges were considered, how many survived, how
/// many were removed as hidden).
#[derive(Clone, Debug)]
pub struct View {
	/// Which way this view looks.
	pub dir: ViewDir,
	/// Drawn entities, in deterministic order: segments in `EdgeId` order, then
	/// analytic circles sorted by `(radius, center.x, center.y)`.
	pub entities: Vec<ViewEntity>,
	/// View-space lower corner (mm).
	pub min: DVec2,
	/// View-space upper corner (mm).
	pub max: DVec2,
	/// Model edges that survived the facet-seam / silhouette filter AND project
	/// to a non-degenerate segment in this view — the edges offered to the
	/// visibility test. (An edge parallel to the sight direction projects to a
	/// point and is counted nowhere: it is neither visible nor hidden.)
	pub edges_considered: usize,
	/// Candidate edges that contributed at least one visible run.
	pub edges_visible: usize,
	/// Candidate edges that were fully occluded and produced nothing.
	pub edges_hidden: usize,
	/// Analytic circles emitted.
	pub circles: usize,
	/// How many projected segments the collinear merge absorbed (0 when
	/// [`ViewOptions::merge_collinear`] is off).
	pub segments_merged: usize,
	/// How visibility was decided.
	pub visibility: Visibility,
}

impl View {
	/// View-space size `(width, height)` in mm.
	pub fn size(&self) -> DVec2 {
		self.max - self.min
	}

	/// Count of [`ViewEntity::Segment`]s.
	pub fn segment_count(&self) -> usize {
		self.entities.iter().filter(|e| matches!(e, ViewEntity::Segment { .. })).count()
	}
}

/// Whether two analytic surfaces are the same surface (same variant, parameters
/// within 1e-9). Two faces sharing a surface are two *facets* of one exact
/// face, so the edge between them is a tessellation seam, not a real edge.
fn same_surface(a: &Surface, b: &Surface) -> bool {
	let close = |x: f64, y: f64| (x - y).abs() < 1e-9;
	let cv = |u: DVec3, v: DVec3| (u - v).length() < 1e-9;
	match (a, b) {
		(Surface::Plane { origin: o1, normal: n1 }, Surface::Plane { origin: o2, normal: n2 }) => {
			// Coplanar facets may carry different in-plane origins; compare the
			// plane itself (same normal direction, same signed offset).
			cv(*n1, *n2) && close(o1.dot(*n1), o2.dot(*n2))
		}
		(Surface::Cylinder { origin: o1, axis: a1, radius: r1 }, Surface::Cylinder { origin: o2, axis: a2, radius: r2 }) => {
			cv(*a1, *a2) && close(*r1, *r2) && cv(*o1 - *a1 * o1.dot(*a1), *o2 - *a2 * o2.dot(*a2))
		}
		(Surface::Sphere { center: c1, radius: r1 }, Surface::Sphere { center: c2, radius: r2 }) => cv(*c1, *c2) && close(*r1, *r2),
		(Surface::Cone { apex: p1, axis: a1, half_angle: h1 }, Surface::Cone { apex: p2, axis: a2, half_angle: h2 }) => {
			cv(*p1, *p2) && cv(*a1, *a2) && close(*h1, *h2)
		}
		(Surface::Torus { center: c1, axis: a1, major: m1, minor: n1 }, Surface::Torus { center: c2, axis: a2, major: m2, minor: n2 }) => {
			cv(*c1, *c2) && cv(*a1, *a2) && close(*m1, *m2) && close(*n1, *n2)
		}
		_ => false,
	}
}

/// Newell outward normal of a face polygon — follows the topological winding,
/// exactly as the tessellator does.
fn newell(poly: &[DVec3]) -> DVec3 {
	let mut n = DVec3::ZERO;
	for i in 0..poly.len() {
		let a = poly[i];
		let b = poly[(i + 1) % poly.len()];
		n.x += (a.y - b.y) * (a.z + b.z);
		n.y += (a.z - b.z) * (a.x + b.x);
		n.z += (a.x - b.x) * (a.y + b.y);
	}
	n.normalize_or_zero()
}

/// Project a model point into the view frame.
fn project(p: DVec3, right: DVec3, up: DVec3) -> DVec2 {
	DVec2::new(p.dot(right), p.dot(up))
}

/// Visible parameter runs of the segment `a → b` under sampled occlusion.
///
/// Each returned `(t0, t1)` is a sub-interval of `[0, 1]` whose sampled
/// midpoints all read visible. The test casts a ray from `span` outside the
/// model **toward** the sample point along `sight`: material in front shortens
/// the first hit, so `hit.t < span − eps` means occluded.
fn visible_runs(a: DVec3, b: DVec3, bvh: &MeshBvh, sight: DVec3, span: f64, eps: f64, samples: usize) -> Vec<(f64, f64)> {
	let dir = sight.as_vec3().normalize_or_zero();
	let mut runs: Vec<(f64, f64)> = Vec::new();
	let mut run_start: Option<usize> = None;
	for i in 0..samples {
		let u = (i as f64 + 0.5) / samples as f64;
		let p = a + (b - a) * u;
		let origin = (p - sight * span).as_vec3();
		let visible = match bvh.raycast(Ray::new(origin, dir)) {
			// Nothing hit at all: the sample is behind no material.
			None => true,
			// A first hit AT the sample point (within eps) is the sample's own
			// surface; anything closer is a genuine occluder.
			Some(h) => (h.t as f64) > span - eps,
		};
		if visible {
			run_start.get_or_insert(i);
		} else if let Some(s) = run_start.take() {
			runs.push((s as f64 / samples as f64, i as f64 / samples as f64));
		}
	}
	if let Some(s) = run_start {
		runs.push((s as f64 / samples as f64, 1.0));
	}
	runs
}

/// Orthographic projection of `solid` along `dir`.
///
/// # Contract
///
/// - Coordinates are **model millimetres** in the view frame
///   ([`ViewDir::frame`]), projected from the solid's exact `f64` vertices — the
///   sheet applies scale and placement, so nothing here is pre-rounded.
/// - Facet seams (edges whose two faces share one analytic surface) are dropped
///   unless the edge is the view's silhouette, so a smooth analytic surface
///   draws as a smooth surface instead of a fan of generatrices.
/// - With [`ViewOptions::hidden_line_removal`] set, occluded runs are **removed
///   and not replaced by dashed lines** ([`HLR_NOTE`]); visibility is decided by
///   ray sampling against the `f32` tessellation, so it is approximate at the
///   sample interval. Dimension values never touch this path.
/// - Segments that land on one infinite line are merged
///   ([`ViewOptions::merge_collinear`]): the fragments a boolean leaves on a
///   coplanar face, and edges the sight line grazes that fall on a silhouette,
///   are one drawn line. Edge-level receipts are counted *before* the merge, so
///   they still describe the model.
/// - Deterministic: segments in `EdgeId` order (merged left to right per line),
///   then circles sorted by `(radius, center.x, center.y)`. No hash-map
///   iteration anywhere.
///
/// An empty (vertex-less) solid yields a view with no entities and a zero
/// rectangle rather than an error — nothing to draw is not a failure.
pub fn project_view(solid: &Solid, dir: ViewDir, opts: &ViewOptions) -> View {
	let (right, up, sight) = dir.frame();
	let mut entities: Vec<ViewEntity> = Vec::new();
	let (lo, hi) = solid.aabb();
	let diag = if lo.is_finite() && hi.is_finite() { (hi - lo).length().max(1e-6) } else { 1.0 };
	let span = diag * 2.0 + 1.0;
	let eps = (diag * opts.visibility_epsilon).max(1e-9);

	// Cylinder groups whose axis is parallel to the sight: their rim chords are
	// replaced by one analytic circle each.
	let features = cylindrical_features(solid);
	let axial: Vec<CylFeature> = if opts.analytic_circles {
		features.iter().copied().filter(|f| f.axis_dir.cross(sight).length() < 1e-9).collect()
	} else {
		Vec::new()
	};
	// Does this face lie on one of the axial cylinder groups? Comparing the
	// SURFACE TAG (not vertex radii) is what makes the rim-chord test survive the
	// T-vertices a boolean leaves mid-chord on a rim.
	let on_axial_cylinder = |surf: &Surface| -> bool {
		let Surface::Cylinder { origin, axis, radius } = *surf else {
			return false;
		};
		let ax = axis.normalize_or_zero();
		axial.iter().any(|f| {
			ax.cross(f.axis_dir).length() < 1e-9 && (radius - f.radius).abs() < 1e-9 && {
				let d = origin - f.axis_point;
				(d - f.axis_dir * d.dot(f.axis_dir)).length() < 1e-9
			}
		})
	};

	let mesh = tessellate_default(solid);
	let bvh = mesh.build_bvh();
	let use_hlr = opts.hidden_line_removal && !mesh.indices.is_empty();

	let mut raw: Vec<(DVec2, DVec2)> = Vec::new();
	let mut considered = 0usize;
	let mut visible_edges = 0usize;
	let mut hidden_edges = 0usize;
	for e in solid.edges() {
		let edge = solid.edge(e);
		let he = solid.half_edge(edge.half_edge);
		let p = solid.position(he.origin);
		let q = solid.position(solid.half_edge(he.next).origin);
		if (p - q).length() < opts.min_segment_mm {
			continue;
		}
		let f1 = he.face;
		let s1 = solid.face(f1).surface;
		let twin_face = he.twin.map(|t| solid.half_edge(t).face);
		let s2 = twin_face.map(|f| solid.face(f).surface);
		// Facet-seam filter: the same analytic surface on both sides is not a real
		// edge of the exact geometry — keep it only when it IS this view's
		// silhouette.
		if let (Some(f2), Some(s2)) = (twin_face, s2) {
			if same_surface(&s1, &s2) {
				let n1 = newell(&solid.face_polygon(f1)).dot(sight);
				let n2 = newell(&solid.face_polygon(f2)).dot(sight);
				if (n1 > 0.0) == (n2 > 0.0) {
					continue;
				}
			}
		}
		// A rim chord of a cylinder group seen down its own axis: the edge sits on
		// that cylinder and runs perpendicular to the sight (= to the axis), so it
		// projects onto the analytic circle that replaces it.
		if (on_axial_cylinder(&s1) || s2.as_ref().is_some_and(&on_axial_cylinder)) && (p - q).dot(sight).abs() < 1e-7 {
			continue;
		}
		// An edge parallel to the sight direction projects to a point: it is
		// neither visible nor hidden, it simply is not a line in this view.
		if (project(p, right, up) - project(q, right, up)).length() < opts.min_segment_mm {
			continue;
		}
		considered += 1;
		let runs = if use_hlr {
			let len = (p - q).length();
			let target = (len / (diag / 24.0).max(1e-9)).ceil() as usize;
			let samples = target.clamp(MIN_HLR_SAMPLES, MAX_HLR_SAMPLES);
			visible_runs(p, q, &bvh, sight, span, eps, samples)
		} else {
			vec![(0.0, 1.0)]
		};
		let before = raw.len();
		for (t0, t1) in runs {
			let a = project(p + (q - p) * t0, right, up);
			let b = project(p + (q - p) * t1, right, up);
			if (a - b).length() >= opts.min_segment_mm {
				raw.push((a, b));
			}
		}
		if raw.len() > before {
			visible_edges += 1;
		} else {
			hidden_edges += 1;
		}
	}
	let raw_count = raw.len();
	let drawn = if opts.merge_collinear { merge_collinear(&raw, opts.collinear_tol.max(0.0)) } else { raw };
	let segments_merged = raw_count - drawn.len();
	entities.extend(drawn.into_iter().map(|(a, b)| ViewEntity::Segment { a, b }));

	// Analytic circles, deduplicated by projected centre + radius and sorted.
	let mut circles: Vec<(f64, f64, f64)> = Vec::new();
	for f in &axial {
		let c = project(f.axis_mid(), right, up);
		if !circles.iter().any(|&(r, x, y)| (r - f.radius).abs() < 1e-9 && (x - c.x).abs() < 1e-9 && (y - c.y).abs() < 1e-9) {
			circles.push((f.radius, c.x, c.y));
		}
	}
	circles.sort_by(|a, b| cmp_keys(&[a.0, a.1, a.2], &[b.0, b.1, b.2]));
	let circle_count = circles.len();
	for (r, x, y) in circles {
		entities.push(ViewEntity::Circle { center: DVec2::new(x, y), radius: r });
	}

	let (min, max) = entity_bounds(&entities);
	View {
		dir,
		entities,
		min,
		max,
		edges_considered: considered,
		edges_visible: visible_edges,
		edges_hidden: hidden_edges,
		circles: circle_count,
		segments_merged,
		visibility: if use_hlr { Visibility::RaySampled { max_samples_per_edge: MAX_HLR_SAMPLES } } else { Visibility::Wireframe },
	}
}

/// Merge projected segments that lie on the same infinite line and overlap or
/// touch (within `tol`) into single segments.
///
/// Deterministic: lines appear in the order their first segment did (the input
/// is in `EdgeId` order), and each line's intervals are merged left to right by
/// total order — so the output is a pure function of the input sequence.
fn merge_collinear(segments: &[(DVec2, DVec2)], tol: f64) -> Vec<(DVec2, DVec2)> {
	/// One infinite line of the merge: unit direction, signed offset from the
	/// origin, and the parameter intervals the segments occupy on it.
	type Line = (DVec2, f64, Vec<(f64, f64)>);
	let mut lines: Vec<Line> = Vec::new();
	for &(a, b) in segments {
		let delta = b - a;
		let len = delta.length();
		if len <= 0.0 {
			continue;
		}
		let mut d = delta / len;
		// Canonical direction so `a→b` and `b→a` land on the same line key.
		if d.x < -tol || (d.x.abs() <= tol && d.y < 0.0) {
			d = -d;
		}
		let nrm = DVec2::new(-d.y, d.x);
		let off = a.dot(nrm);
		let (ta, tb) = (a.dot(d), b.dot(d));
		let interval = (ta.min(tb), ta.max(tb));
		match lines.iter_mut().find(|l| (l.0 - d).length() <= tol && (l.1 - off).abs() <= tol) {
			Some(l) => l.2.push(interval),
			None => lines.push((d, off, vec![interval])),
		}
	}
	let mut out = Vec::with_capacity(lines.len());
	for (d, off, mut intervals) in lines {
		intervals.sort_by(|x, y| x.0.total_cmp(&y.0).then_with(|| x.1.total_cmp(&y.1)));
		let base = DVec2::new(-d.y, d.x) * off;
		let mut cur = intervals[0];
		for &(s, e) in &intervals[1..] {
			if s <= cur.1 + tol {
				cur.1 = cur.1.max(e);
			} else {
				out.push((base + d * cur.0, base + d * cur.1));
				cur = (s, e);
			}
		}
		out.push((base + d * cur.0, base + d * cur.1));
	}
	out
}

/// Bounding rectangle of a list of view entities (`(0,0)–(0,0)` when empty).
fn entity_bounds(entities: &[ViewEntity]) -> (DVec2, DVec2) {
	let mut min = DVec2::splat(f64::INFINITY);
	let mut max = DVec2::splat(f64::NEG_INFINITY);
	let mut push = |p: DVec2| {
		min = min.min(p);
		max = max.max(p);
	};
	for e in entities {
		match e {
			ViewEntity::Segment { a, b } => {
				push(*a);
				push(*b);
			}
			ViewEntity::Circle { center, radius } => {
				push(*center - DVec2::splat(*radius));
				push(*center + DVec2::splat(*radius));
			}
			ViewEntity::Polyline(pts) => {
				for p in pts {
					push(*p);
				}
			}
		}
	}
	if min.x.is_finite() {
		(min, max)
	} else {
		(DVec2::ZERO, DVec2::ZERO)
	}
}

// ---------------------------------------------------------------------------
// Cylindrical features — the model measures behind hole dimensions
// ---------------------------------------------------------------------------

/// Whether a cylindrical face group is material-removed (a bore) or
/// material-added (a round boss). Decided from the face winding, not a name.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum CylKind {
	/// The face normals point **at** the axis: a hole / bore.
	Bore,
	/// The face normals point **away from** the axis: a round boss / shaft.
	Boss,
}

impl CylKind {
	/// Stable lowercase name.
	pub fn name(self) -> &'static str {
		match self {
			CylKind::Bore => "bore",
			CylKind::Boss => "boss",
		}
	}
}

/// A coaxial group of same-radius cylindrical faces — the drawing's notion of
/// "a hole" or "a round boss", derived entirely from the solid's analytic
/// `Surface::Cylinder` tags.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CylFeature {
	/// Bore or boss.
	pub kind: CylKind,
	/// Where the axis pierces the plane through the world origin perpendicular
	/// to the axis — a canonical, tag-origin-independent handle.
	pub axis_point: DVec3,
	/// Unit axis direction, canonically signed (first significant component
	/// positive) so two opposite tags group together.
	pub axis_dir: DVec3,
	/// The analytic radius from `Surface::Cylinder { radius }`.
	pub radius: f64,
	/// Lowest axial station of the group's vertices, measured from
	/// [`Self::axis_point`] along [`Self::axis_dir`].
	pub t_min: f64,
	/// Highest axial station of the group's vertices.
	pub t_max: f64,
	/// How many faces the group collected.
	pub faces: usize,
}

impl CylFeature {
	/// Analytic diameter: `2 × radius`.
	pub fn diameter(&self) -> f64 {
		self.radius * 2.0
	}

	/// Axial length of the group.
	pub fn depth(&self) -> f64 {
		self.t_max - self.t_min
	}

	/// A point on the axis at the group's mid-height.
	pub fn axis_mid(&self) -> DVec3 {
		self.axis_point + self.axis_dir * ((self.t_min + self.t_max) * 0.5)
	}
}

/// Every coaxial, same-radius group of cylindrical faces of `solid`, classified
/// bore vs boss.
///
/// # Contract
///
/// - Grouping key: canonical axis direction, the axis's piercing point through
///   the origin plane, the analytic radius, and the bore/boss class — all
///   matched to 1e-9. A bore split into many faces by a boolean regroups into
///   one feature.
/// - Bore vs boss comes from the face's Newell (winding) normal against the
///   radial direction, so it survives a surface tag whose stored axis sign is
///   incidental.
/// - Deterministic order: bores before bosses, then by radius, then by axis
///   point, then by axis direction. No hash maps.
/// - **Limit**: only `Surface::Cylinder` faces participate. A conical
///   countersink, a spherical seat and a milled slot are not features here; ask
///   for a dimension on one and you get [`DrawingError::FeatureNotFound`], never
///   a made-up number.
pub fn cylindrical_features(solid: &Solid) -> Vec<CylFeature> {
	let mut groups: Vec<CylFeature> = Vec::new();
	for f in solid.faces() {
		let Surface::Cylinder { origin, axis, radius } = solid.face(f).surface else {
			continue;
		};
		let ax = axis.normalize_or_zero();
		if ax.length_squared() < 0.5 || !radius.is_finite() || radius <= 0.0 {
			continue;
		}
		// Canonical sign: first significant component positive.
		let sign = if ax.x.abs() > 1e-12 {
			ax.x.signum()
		} else if ax.y.abs() > 1e-12 {
			ax.y.signum()
		} else if ax.z.abs() > 1e-12 {
			ax.z.signum()
		} else {
			1.0
		};
		let dir = ax * sign;
		let axis_point = origin - dir * origin.dot(dir);
		let poly = solid.face_polygon(f);
		if poly.len() < 3 {
			continue;
		}
		let n = newell(&poly);
		let centroid = poly.iter().fold(DVec3::ZERO, |acc, &p| acc + p) / poly.len() as f64;
		let d = centroid - axis_point;
		let radial = (d - dir * d.dot(dir)).normalize_or_zero();
		let kind = if n.dot(radial) >= 0.0 { CylKind::Boss } else { CylKind::Bore };
		let mut t_min = f64::INFINITY;
		let mut t_max = f64::NEG_INFINITY;
		for p in &poly {
			let t = (*p - axis_point).dot(dir);
			t_min = t_min.min(t);
			t_max = t_max.max(t);
		}
		let existing = groups.iter_mut().find(|g| {
			g.kind == kind && (g.radius - radius).abs() < 1e-9 && (g.axis_dir - dir).length() < 1e-9 && (g.axis_point - axis_point).length() < 1e-9
		});
		match existing {
			Some(g) => {
				g.t_min = g.t_min.min(t_min);
				g.t_max = g.t_max.max(t_max);
				g.faces += 1;
			}
			None => groups.push(CylFeature { kind, axis_point, axis_dir: dir, radius, t_min, t_max, faces: 1 }),
		}
	}
	groups.sort_by(|a, b| {
		a.kind.cmp(&b.kind).then_with(|| {
			cmp_keys(
				&[a.radius, a.axis_point.x, a.axis_point.y, a.axis_point.z, a.axis_dir.x, a.axis_dir.y, a.axis_dir.z],
				&[b.radius, b.axis_point.x, b.axis_point.y, b.axis_point.z, b.axis_dir.x, b.axis_dir.y, b.axis_dir.z],
			)
		})
	});
	groups
}

/// The bores of `solid`, in [`cylindrical_features`] order.
pub fn bores(solid: &Solid) -> Vec<CylFeature> {
	cylindrical_features(solid).into_iter().filter(|f| f.kind == CylKind::Bore).collect()
}

/// The round bosses of `solid`, in [`cylindrical_features`] order.
pub fn bosses(solid: &Solid) -> Vec<CylFeature> {
	cylindrical_features(solid).into_iter().filter(|f| f.kind == CylKind::Boss).collect()
}

// ---------------------------------------------------------------------------
// Dimensions
// ---------------------------------------------------------------------------

/// What class of dimension a [`Dimension`] is — drives its symbol and its
/// placement on the sheet.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DimKind {
	/// A length between two model features.
	Linear,
	/// A diameter (`Ø`), drawn with a leader.
	Diameter,
	/// A feature position measured from the drawing datum.
	Position,
	/// A wall / ligament thickness.
	Wall,
}

impl DimKind {
	/// Stable lowercase name for the schedule table.
	pub fn name(self) -> &'static str {
		match self {
			DimKind::Linear => "linear",
			DimKind::Diameter => "diameter",
			DimKind::Position => "position",
			DimKind::Wall => "wall",
		}
	}

	/// The prefix symbol drawn before the value (`"Ø"` for a diameter).
	pub fn symbol(self) -> &'static str {
		match self {
			DimKind::Diameter => "\u{d8}",
			_ => "",
		}
	}
}

// The measure names below ARE the audit trail printed in the schedule: change
// the code that produces a value and you must change its name here.

/// [`Dimension::from_measure`] for an overall extent.
pub const M_EXTENT: &str = "kernel_brep::bounding_box(solid).size()";
/// [`Dimension::from_measure`] for a bore diameter.
pub const M_BORE_D: &str = "Surface::Cylinder{radius} x 2 (coaxial bore face group)";
/// [`Dimension::from_measure`] for a bore depth.
pub const M_BORE_DEPTH: &str = "coaxial bore face group axial extent (t_max - t_min)";
/// [`Dimension::from_measure`] for a bore position.
pub const M_BORE_POS: &str = "Surface::Cylinder axis station - kernel_brep::bounding_box(solid).min";
/// [`Dimension::from_measure`] for an annular wall.
pub const M_COAX_WALL: &str = "enclosing coaxial Surface::Cylinder{radius} - bore Surface::Cylinder{radius}";

/// One dimension on a sheet.
///
/// [`Self::value`] is produced by an engine measure of the model and
/// [`Self::from_measure`] names the measure that produced it. Nothing in this
/// module ever builds a `Dimension` from a literal: every construction site is
/// a `measure_*` path that has just read the geometry. The fields are public so
/// a caller can render them — if you build one by hand, you own the claim.
#[derive(Clone, Debug, PartialEq)]
pub struct Dimension {
	/// The measured value in millimetres (a diameter is a diameter, not a
	/// radius).
	pub value: f64,
	/// WHICH engine measure produced [`Self::value`] — the audit trail printed
	/// in the sheet's dimension schedule. One of the `M_*` constants.
	pub from_measure: &'static str,
	/// Which concrete feature of *this* model was measured, e.g.
	/// `"bore #0 (D8.000 axis 0.000,0.000,1.000 through 20.000,20.000,0.000)"`.
	pub subject: String,
	/// The class (drives symbol and placement).
	pub kind: DimKind,
	/// Rendered text, e.g. `"80.000"` or `"Ø8.000"`.
	pub text: String,
	/// Where the dimension attaches, in **model** coordinates, when it has a
	/// graphical anchor (a bore axis). `None` for schedule-only dimensions.
	pub anchor: Option<DVec3>,
}

impl Dimension {
	/// Build a dimension from a value the caller has just *measured*, tagging it
	/// with the measure's name. Internal: every call site in this module is a
	/// measurement.
	fn measured(value: f64, from_measure: &'static str, subject: String, kind: DimKind, anchor: Option<DVec3>) -> Dimension {
		let text = format!("{}{}", kind.symbol(), dim_text(value));
		Dimension { value, from_measure, subject, kind, text, anchor }
	}
}

/// A request for one specific measured dimension. Indices refer to [`bores`]
/// order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DimRequest {
	/// Overall model extent along a world axis.
	OverallExtent(Axis),
	/// Diameter of the `index`-th bore.
	BoreDiameter(usize),
	/// Axial length of the `index`-th bore.
	BoreDepth(usize),
	/// Position of the `index`-th bore's axis along a world axis, from the
	/// bounding-box minimum corner (the drawing datum).
	BorePosition {
		/// Bore index.
		index: usize,
		/// Which world axis the position is measured along.
		axis: Axis,
	},
	/// Annular wall between the `index`-th bore and the smallest coaxial boss
	/// that encloses it.
	CoaxialWall(usize),
}

/// Human description of what this model offers, for a refusal message.
fn inventory(solid: &Solid) -> String {
	let feats = cylindrical_features(solid);
	let nb = feats.iter().filter(|f| f.kind == CylKind::Bore).count();
	let nz = feats.len() - nb;
	format!("{nb} bore(s), {nz} round boss(es)")
}

/// Describe a cylindrical feature for a [`Dimension::subject`] / refusal.
fn describe(f: &CylFeature, index: usize) -> String {
	format!(
		"{} #{index} (D{} axis {},{},{} through {},{},{})",
		f.kind.name(),
		dim_text(f.diameter()),
		dim_text(f.axis_dir.x),
		dim_text(f.axis_dir.y),
		dim_text(f.axis_dir.z),
		dim_text(f.axis_point.x),
		dim_text(f.axis_point.y),
		dim_text(f.axis_point.z)
	)
}

/// Measure one requested dimension from the model.
///
/// # Contract
///
/// Every returned [`Dimension::value`] is computed here from the solid's own
/// geometry — a bounding box or an analytic `Surface::Cylinder` tag — and
/// carries the name of the measure that produced it. There is **no fallback
/// path that returns a plausible number**: a request naming a feature the model
/// does not have returns [`DrawingError::FeatureNotFound`], and a measure the
/// geometry cannot support returns [`DrawingError::NotDeterminable`].
///
/// # Errors
///
/// - [`DrawingError::FeatureNotFound`] — bore index out of range, or the solid
///   has no measurable bounding box.
/// - [`DrawingError::NotDeterminable`] — [`DimRequest::CoaxialWall`] on a bore
///   with no enclosing coaxial boss; [`DimRequest::BorePosition`] along the
///   bore's own axis (a bore has no located position along itself).
pub fn measure_dimension(solid: &Solid, req: &DimRequest) -> Result<Dimension, DrawingError> {
	let bb = bounding_box(solid).ok_or_else(|| DrawingError::FeatureNotFound {
		requested: "any dimension".to_string(),
		available: "the solid has no finite vertices — nothing to measure".to_string(),
	})?;
	let feats = cylindrical_features(solid);
	let bore_list: Vec<CylFeature> = feats.iter().copied().filter(|f| f.kind == CylKind::Bore).collect();
	let get_bore = |index: usize| -> Result<CylFeature, DrawingError> {
		bore_list.get(index).copied().ok_or_else(|| DrawingError::FeatureNotFound {
			requested: format!("bore #{index}"),
			available: inventory(solid),
		})
	};
	match *req {
		DimRequest::OverallExtent(axis) => {
			let v = axis.of(bb.size());
			Ok(Dimension::measured(v, M_EXTENT, format!("overall extent along {}", axis.name()), DimKind::Linear, None))
		}
		DimRequest::BoreDiameter(index) => {
			let f = get_bore(index)?;
			Ok(Dimension::measured(f.diameter(), M_BORE_D, describe(&f, index), DimKind::Diameter, Some(f.axis_mid())))
		}
		DimRequest::BoreDepth(index) => {
			let f = get_bore(index)?;
			Ok(Dimension::measured(f.depth(), M_BORE_DEPTH, format!("{} axial depth", describe(&f, index)), DimKind::Linear, Some(f.axis_mid())))
		}
		DimRequest::BorePosition { index, axis } => {
			let f = get_bore(index)?;
			if axis.dir().cross(f.axis_dir).length() < 1e-9 {
				return Err(DrawingError::NotDeterminable {
					measure: M_BORE_POS,
					subject: describe(&f, index),
					why: "the requested axis IS the bore's own axis — a bore has no located position along itself",
				});
			}
			let v = axis.of(f.axis_point) - axis.of(bb.min);
			Ok(Dimension::measured(
				v,
				M_BORE_POS,
				format!("{} position along {} from datum", describe(&f, index), axis.name()),
				DimKind::Position,
				Some(f.axis_mid()),
			))
		}
		DimRequest::CoaxialWall(index) => {
			let f = get_bore(index)?;
			// The smallest coaxial boss strictly larger than the bore.
			let mut best: Option<CylFeature> = None;
			for g in feats.iter().filter(|g| g.kind == CylKind::Boss) {
				let coaxial = (g.axis_dir - f.axis_dir).length() < 1e-9 && (g.axis_point - f.axis_point).length() < 1e-9;
				if coaxial && g.radius > f.radius + 1e-12 && best.is_none_or(|b| g.radius < b.radius) {
					best = Some(*g);
				}
			}
			let outer = best.ok_or_else(|| DrawingError::NotDeterminable {
				measure: M_COAX_WALL,
				subject: describe(&f, index),
				why: "no coaxial cylindrical boss encloses this bore, so the annular wall is not a single determinable number",
			})?;
			Ok(Dimension::measured(
				outer.radius - f.radius,
				M_COAX_WALL,
				format!("{} inside boss D{}", describe(&f, index), dim_text(outer.diameter())),
				DimKind::Wall,
				Some(f.axis_mid()),
			))
		}
	}
}

/// The dimensions a sheet gets by default: the three overall extents, then per
/// bore its diameter, its depth, its positions from the datum on the two axes
/// perpendicular to its own, and its annular wall **where determinable**.
///
/// Refusals are simply skipped here (a bore with no enclosing boss contributes
/// no wall dimension) — the schedule then holds fewer rows, never a fabricated
/// one. Call [`measure_dimension`] directly when you want the typed refusal
/// instead of the omission. Deterministic: bores in [`cylindrical_features`]
/// order, requests in the fixed order above.
pub fn auto_dimensions(solid: &Solid) -> Vec<Dimension> {
	let mut out = Vec::new();
	for axis in [Axis::X, Axis::Y, Axis::Z] {
		if let Ok(d) = measure_dimension(solid, &DimRequest::OverallExtent(axis)) {
			out.push(d);
		}
	}
	let n = bores(solid).len();
	for index in 0..n {
		for req in [
			DimRequest::BoreDiameter(index),
			DimRequest::BoreDepth(index),
			DimRequest::BorePosition { index, axis: Axis::X },
			DimRequest::BorePosition { index, axis: Axis::Y },
			DimRequest::BorePosition { index, axis: Axis::Z },
			DimRequest::CoaxialWall(index),
		] {
			if let Ok(d) = measure_dimension(solid, &req) {
				out.push(d);
			}
		}
	}
	out
}

// ---------------------------------------------------------------------------
// Section view
// ---------------------------------------------------------------------------

/// A cut-plane section: the cut boundary in cut-plane coordinates, its 45°
/// hatching, and the exactness receipts from the engine's section machinery.
#[derive(Clone, Debug)]
pub struct SectionView {
	/// A point on the cut plane.
	pub plane_point: DVec3,
	/// The cut plane's unit normal.
	pub plane_normal: DVec3,
	/// Section label, e.g. `"A-A"`.
	pub label: String,
	/// Closed cut-boundary loops in cut-plane 2-D coordinates (the frame is
	/// [`kernel_brep::geom::perp_basis`] of the normal), chained from the
	/// face-polygon cut: exact for planar walls, chord-accurate for curved.
	pub boundary: Vec<ViewEntity>,
	/// Hatch segments, 45° in the cut-plane frame, clipped even-odd against
	/// [`Self::boundary`].
	pub hatch: Vec<(DVec2, DVec2)>,
	/// How many curves [`kernel_brep::section_curves_with_fallback`] returned as
	/// **exact** analytic conics (the closed-form half of the cut).
	pub exact_curves: usize,
	/// How many it returned as chord polylines (no closed form — oblique torus).
	pub polyline_curves: usize,
	/// Net cut area from [`kernel_brep::section_properties`], when available.
	pub area_mm2: Option<f64>,
	/// Cut-plane lower corner.
	pub min: DVec2,
	/// Cut-plane upper corner.
	pub max: DVec2,
}

/// Cut `solid` with the plane through `plane_point` with `plane_normal` and
/// return a hatched section view labelled `label`.
///
/// # Contract
///
/// - The hatched boundary is chained from every face the plane crosses, cutting
///   each face's outer AND inner loops and pairing the crossings even-odd along
///   the face-plane ∩ cut-plane line: **exact in `f64` for planar walls**
///   (holes included), chord-accurate for curved surfaces because their facets
///   are chords. A vertex lying exactly on the cut plane is handled by a
///   half-open crossing rule, so a cut through a facet seam neither drops nor
///   doubles the seam edge.
/// - The analytic conics from [`kernel_brep::section_curves_with_fallback`] are
///   counted into [`SectionView::exact_curves`] /
///   [`SectionView::polyline_curves`] so the sheet can state how much of the cut
///   is closed-form.
/// - Hatching is 45° in the cut-plane frame at `hatch_pitch` mm, clipped by the
///   even-odd rule against the chained boundary, so a bore stays unhatched.
///   Scanlines are anchored to integer multiples of the pitch, so the pattern is
///   identical across runs.
/// - Deterministic: loops chained in face order, hatch emitted scanline by
///   scanline with crossings sorted by total order.
///
/// # Errors
///
/// - [`DrawingError::BadInput`] for a non-finite / non-positive `hatch_pitch` or
///   a degenerate normal.
/// - [`DrawingError::EmptySection`] when the plane cuts no material.
pub fn section_view(solid: &Solid, plane_point: DVec3, plane_normal: DVec3, hatch_pitch: f64, label: &str) -> Result<SectionView, DrawingError> {
	if !hatch_pitch.is_finite() || hatch_pitch <= 0.0 {
		return Err(DrawingError::BadInput { field: "hatch_pitch", got: hatch_pitch, why: "hatch pitch must be finite and > 0 mm" });
	}
	let n = plane_normal.normalize_or_zero();
	if n.length_squared() < 0.5 {
		return Err(DrawingError::BadInput {
			field: "plane_normal",
			got: plane_normal.length(),
			why: "the cut plane normal must be a non-zero, finite direction",
		});
	}
	let (e1, e2) = perp_basis(n);
	let to2 = |p: DVec3| DVec2::new((p - plane_point).dot(e1), (p - plane_point).dot(e2));

	// Cut each face against the plane. A face is planar (a chord facet for a
	// curved surface), so its section is a segment of the LINE where its plane
	// meets the cut plane: collect the crossings of every loop of the face —
	// outer AND inner hole loops — order them along that line, and pair them
	// even-odd. Exact in f64, holes handled, and the half-open crossing rule
	// (`da > 0` vs `db > 0`) keeps a vertex sitting exactly on the plane from
	// being counted from both of its edges.
	let mut segments: Vec<(DVec2, DVec2)> = Vec::new();
	for f in solid.faces() {
		let face = solid.face(f);
		let outer = solid.loop_polygon(face.outer);
		if outer.len() < 3 {
			continue;
		}
		let line_dir = newell(&outer).cross(n);
		if line_dir.length() < 1e-12 {
			continue; // the face is parallel to (or lies in) the cut plane
		}
		let line_dir = line_dir.normalize();
		let mut hits: Vec<(f64, DVec3)> = Vec::new();
		for lid in std::iter::once(face.outer).chain(face.inner.iter().copied()) {
			let poly = solid.loop_polygon(lid);
			for i in 0..poly.len() {
				let a = poly[i];
				let b = poly[(i + 1) % poly.len()];
				let da = (a - plane_point).dot(n);
				let db = (b - plane_point).dot(n);
				if (da > 0.0) != (db > 0.0) && (da - db).abs() > 1e-15 {
					let p = a + (b - a) * (da / (da - db));
					hits.push((p.dot(line_dir), p));
				}
			}
		}
		hits.sort_by(|x, y| x.0.total_cmp(&y.0));
		for pair in hits.chunks_exact(2) {
			if (pair[1].1 - pair[0].1).length() > 1e-12 {
				segments.push((to2(pair[0].1), to2(pair[1].1)));
			}
		}
	}
	if segments.is_empty() {
		return Err(DrawingError::EmptySection { point: plane_point, normal: n });
	}
	let loops = chain_loops(segments);
	let boundary: Vec<ViewEntity> = loops.iter().cloned().map(ViewEntity::Polyline).collect();
	let hatch = hatch_loops(&loops, hatch_pitch);

	let mut exact_curves = 0usize;
	let mut polyline_curves = 0usize;
	for c in section_curves_with_fallback(solid, plane_point, n) {
		match c {
			SectionCurve::Exact(_) => exact_curves += 1,
			SectionCurve::Polyline(_) => polyline_curves += 1,
		}
	}
	let area_mm2 = section_properties(solid, plane_point, n).map(|p| p.area);
	let (min, max) = entity_bounds(&boundary);
	Ok(SectionView { plane_point, plane_normal: n, label: label.to_string(), boundary, hatch, exact_curves, polyline_curves, area_mm2, min, max })
}

/// Chain 2-D segments into closed loops by endpoint proximity. Segments are
/// consumed from the end of the list, so the loop order is a deterministic
/// function of the face iteration order that produced them.
fn chain_loops(mut segments: Vec<(DVec2, DVec2)>) -> Vec<Vec<DVec2>> {
	const WELD: f64 = 1e-7;
	let mut loops = Vec::new();
	while let Some((a, b)) = segments.pop() {
		let mut chain = vec![a, b];
		for forward in [true, false] {
			loop {
				let end = if forward { *chain.last().expect("chain starts with two points") } else { chain[0] };
				let Some(idx) = segments.iter().position(|&(p, q)| (p - end).length() < WELD || (q - end).length() < WELD) else {
					break;
				};
				let (p, q) = segments.swap_remove(idx);
				let next = if (p - end).length() < WELD { q } else { p };
				if forward {
					chain.push(next);
				} else {
					chain.insert(0, next);
				}
			}
		}
		if chain.len() > 2 && (chain[0] - *chain.last().expect("non-empty chain")).length() < WELD {
			chain.pop();
		}
		if chain.len() >= 3 {
			loops.push(chain);
		}
	}
	loops
}

/// 45° hatch segments clipped even-odd against closed `loops`, spaced `pitch`
/// apart along the hatch normal and anchored to integer multiples of the pitch.
fn hatch_loops(loops: &[Vec<DVec2>], pitch: f64) -> Vec<(DVec2, DVec2)> {
	let inv = 1.0 / 2.0_f64.sqrt();
	let h = DVec2::new(inv, inv); // hatch line direction (45°)
	let g = DVec2::new(-inv, inv); // its normal — the scanline coordinate
	let (mut cmin, mut cmax) = (f64::INFINITY, f64::NEG_INFINITY);
	for lp in loops {
		for p in lp {
			let c = p.dot(g);
			cmin = cmin.min(c);
			cmax = cmax.max(c);
		}
	}
	if !cmin.is_finite() || cmax <= cmin {
		return Vec::new();
	}
	let k0 = (cmin / pitch).ceil() as i64;
	let k1 = (cmax / pitch).floor() as i64;
	let mut out = Vec::new();
	for k in k0..=k1 {
		let c = k as f64 * pitch;
		let mut crossings: Vec<f64> = Vec::new();
		for lp in loops {
			for i in 0..lp.len() {
				let p = lp[i];
				let q = lp[(i + 1) % lp.len()];
				let sp = p.dot(g) - c;
				let sq = q.dot(g) - c;
				// Half-open rule: a crossing is counted once, so a vertex sitting
				// exactly on the scanline never double-counts.
				if (sp <= 0.0) != (sq <= 0.0) && (sp - sq).abs() > 1e-15 {
					let t = sp / (sp - sq);
					crossings.push((p + (q - p) * t).dot(h));
				}
			}
		}
		crossings.sort_by(f64::total_cmp);
		for pair in crossings.chunks_exact(2) {
			if pair[1] - pair[0] > 1e-9 {
				out.push((h * pair[0] + g * c, h * pair[1] + g * c));
			}
		}
	}
	out
}

// ---------------------------------------------------------------------------
// Title block
// ---------------------------------------------------------------------------

/// The caller-supplied source of the sheet's **general tolerance** note.
///
/// # The seam, stated
///
/// General tolerances are process data, not geometry: the same solid drawn for
/// FDM, for casting and for CNC carries three different notes. `drawing`
/// therefore takes the number from *you* and deliberately does **not** depend
/// on [`crate::process`]. When the process layer is ready to own it, the wiring
/// is one impl on that side (`impl GeneralTolerance for FdmProfile`) and nothing
/// in this module changes. Use [`FixedTolerance`] until then.
pub trait GeneralTolerance {
	/// The symmetric general tolerance in millimetres (`±value`).
	fn general_tolerance_mm(&self) -> f64;
	/// Where the number came from — printed on the sheet so a reader can
	/// challenge it (e.g. `"FdmProfile 'p1s_pla' xy_clearance_free"`).
	fn tolerance_source(&self) -> String;
}

/// A literal general tolerance with a stated source — the honest placeholder
/// until a process profile supplies one.
#[derive(Clone, Debug, PartialEq)]
pub struct FixedTolerance {
	/// The `±` value in millimetres.
	pub value_mm: f64,
	/// The provenance string printed on the sheet.
	pub source: String,
}

impl FixedTolerance {
	/// A fixed tolerance with its provenance.
	pub fn new(value_mm: f64, source: &str) -> FixedTolerance {
		FixedTolerance { value_mm, source: source.to_string() }
	}
}

impl GeneralTolerance for FixedTolerance {
	fn general_tolerance_mm(&self) -> f64 {
		self.value_mm
	}

	fn tolerance_source(&self) -> String {
		self.source.clone()
	}
}

/// The sheet's title block. Everything here is caller-supplied *identity*, with
/// one expectation: [`Self::mass_g`] should come from a real volume × density,
/// and the sheet prints `MASS` only when it is present.
///
/// **The date is an input.** Deterministic output forbids reading the clock, so
/// there is no `SystemTime::now()` anywhere in this module.
#[derive(Clone, Debug, PartialEq)]
pub struct TitleBlock {
	/// Part name.
	pub part_name: String,
	/// Part number (empty to omit).
	pub part_number: String,
	/// Material name (empty to omit).
	pub material: String,
	/// Mass in grams, when known from a real measure.
	pub mass_g: Option<f64>,
	/// Manufacturing process name (empty to omit).
	pub process: String,
	/// Date string supplied by the caller, e.g. `"2026-07-30"`.
	pub date: String,
	/// Who/what drew it.
	pub drawn_by: String,
	/// Revision string.
	pub revision: String,
	/// General tolerance in millimetres (`±`).
	pub general_tolerance_mm: f64,
	/// Provenance of the tolerance.
	pub tolerance_source: String,
}

impl TitleBlock {
	/// A title block with the mandatory fields; the rest default to empty.
	pub fn new(part_name: &str, date: &str, tol: &impl GeneralTolerance) -> TitleBlock {
		TitleBlock {
			part_name: part_name.to_string(),
			part_number: String::new(),
			material: String::new(),
			mass_g: None,
			process: String::new(),
			date: date.to_string(),
			drawn_by: "LMCAD kernel_model::drawing".to_string(),
			revision: "A".to_string(),
			general_tolerance_mm: tol.general_tolerance_mm(),
			tolerance_source: tol.tolerance_source(),
		}
	}

	/// Set the part number.
	pub fn with_part_number(mut self, pn: &str) -> Self {
		self.part_number = pn.to_string();
		self
	}

	/// Set material and mass (grams — pass a measured value).
	pub fn with_material(mut self, material: &str, mass_g: Option<f64>) -> Self {
		self.material = material.to_string();
		self.mass_g = mass_g;
		self
	}

	/// Set the process name.
	pub fn with_process(mut self, process: &str) -> Self {
		self.process = process.to_string();
		self
	}

	/// Set the revision.
	pub fn with_revision(mut self, revision: &str) -> Self {
		self.revision = revision.to_string();
		self
	}

	/// The filled units + tolerance note.
	pub fn units_note(&self) -> String {
		UNITS_NOTE_TEMPLATE
			.replace("{tol}", &dim_text(self.general_tolerance_mm))
			.replace("{src}", if self.tolerance_source.is_empty() { "caller-supplied" } else { &self.tolerance_source })
	}

	/// The title-block rows, in sheet order — shared by the SVG and DXF writers
	/// so the two never drift.
	fn rows(&self, scale_text: &str) -> Vec<(&'static str, String)> {
		vec![
			("PART", self.part_name.clone()),
			("PART NO", self.part_number.clone()),
			("MATERIAL", self.material.clone()),
			("MASS", self.mass_g.map(|m| format!("{} g", dim_text(m))).unwrap_or_else(|| "-".to_string())),
			("PROCESS", self.process.clone()),
			("SCALE", scale_text.to_string()),
			("DATE", self.date.clone()),
			("REV", self.revision.clone()),
			("DRAWN", self.drawn_by.clone()),
			("TOL", format!("+/-{} mm ({})", dim_text(self.general_tolerance_mm), self.tolerance_source)),
		]
	}
}

// ---------------------------------------------------------------------------
// The sheet
// ---------------------------------------------------------------------------

/// Sheet width in millimetres (A3 landscape).
pub const SHEET_W: f64 = 420.0;
/// Sheet height in millimetres (A3 landscape).
pub const SHEET_H: f64 = 297.0;
/// Sheet margin in millimetres.
pub const SHEET_MARGIN: f64 = 10.0;
/// Title-block width in millimetres.
const TITLE_W: f64 = 130.0;
/// Title-block height in millimetres.
const TITLE_H: f64 = 46.0;
/// Height of the notes / schedule strip along the sheet bottom.
const NOTES_H: f64 = 78.0;

/// The standard drawing scales this module snaps to, largest first, as
/// `(numerator, denominator)`. A scale is chosen from this list so the title
/// block never reads `1:2.7183`.
pub const SCALE_SERIES: [(u32, u32); 10] = [(10, 1), (5, 1), (2, 1), (1, 1), (1, 2), (1, 5), (1, 10), (1, 20), (1, 50), (1, 100)];

/// A complete drawing sheet: title block, views, optional section, dimensions
/// and notes, renderable to deterministic SVG and DXF.
#[derive(Clone, Debug)]
pub struct Drawing {
	/// The title block.
	pub title: TitleBlock,
	/// The views, in the order added. Placement is by [`ViewDir`] (third-angle),
	/// not by insertion order.
	pub views: Vec<View>,
	/// An optional section view.
	pub section: Option<SectionView>,
	/// Every dimension, in the order added. All appear in the schedule; the
	/// graphical ones also get dimension lines / leaders.
	pub dimensions: Vec<Dimension>,
	/// Extra free-text notes, appended after the mandatory ones.
	pub notes: Vec<String>,
}

impl Drawing {
	/// An empty sheet with a title block.
	pub fn new(title: TitleBlock) -> Drawing {
		Drawing { title, views: Vec::new(), section: None, dimensions: Vec::new(), notes: Vec::new() }
	}

	/// Add a view.
	pub fn with_view(mut self, view: View) -> Self {
		self.views.push(view);
		self
	}

	/// Attach the section view.
	pub fn with_section(mut self, section: SectionView) -> Self {
		self.section = Some(section);
		self
	}

	/// Add measured dimensions.
	pub fn with_dimensions(mut self, dims: Vec<Dimension>) -> Self {
		self.dimensions.extend(dims);
		self
	}

	/// Add a free-text note.
	pub fn with_note(mut self, note: &str) -> Self {
		self.notes.push(note.to_string());
		self
	}

	/// The notes in emission order: the hidden-line limitation, the units +
	/// tolerance note, the provenance note, the section's exactness receipt when
	/// there is a section, then any caller notes.
	///
	/// [`HLR_NOTE`] is always first and always present — no builder can remove
	/// it, which is what makes "the sheet states its own limitation" a property
	/// rather than a habit.
	pub fn all_notes(&self) -> Vec<String> {
		let mut out = vec![HLR_NOTE.to_string(), self.title.units_note(), PROVENANCE_NOTE.to_string()];
		if let Some(s) = &self.section {
			out.push(format!(
				"SECTION {}: {} exact analytic conic(s) + {} chord polyline(s) reported by kernel_brep::section_curves_with_fallback; the hatched boundary is chained from the face-polygon cut (exact for planar walls, chord-accurate for curved).",
				s.label, s.exact_curves, s.polyline_curves
			));
		}
		out.extend(self.notes.iter().cloned());
		out
	}

	/// The drawing area (everything above the notes strip, inside the margin) as
	/// `(x0, y0, x1, y1)` in sheet mm, y up.
	fn draw_area(&self) -> (f64, f64, f64, f64) {
		(SHEET_MARGIN, SHEET_MARGIN + NOTES_H, SHEET_W - SHEET_MARGIN, SHEET_H - SHEET_MARGIN)
	}

	/// Which quadrant cell a view occupies — third-angle arrangement: TOP above
	/// FRONT, RIGHT beside FRONT, ISO in the free corner. `(col, row)`, row 0 at
	/// the bottom.
	fn cell_of(dir: ViewDir) -> (usize, usize) {
		match dir {
			ViewDir::Front => (0, 0),
			ViewDir::Right => (1, 0),
			ViewDir::Top => (0, 1),
			ViewDir::Iso => (1, 1),
		}
	}

	/// The cell the section view lands in: the first free one in the fixed
	/// preference order `(1,1) → (0,1) → (1,0) → (0,0)`.
	fn section_cell(&self) -> (usize, usize) {
		for cell in [(1, 1), (0, 1), (1, 0), (0, 0)] {
			if !self.views.iter().any(|v| Self::cell_of(v.dir) == cell) {
				return cell;
			}
		}
		(1, 1)
	}

	/// The rect of cell `(col, row)` as `(x0, y0, x1, y1)`, row 0 at the bottom.
	fn cell_rect(&self, col: usize, row: usize) -> (f64, f64, f64, f64) {
		let (x0, y0, x1, y1) = self.draw_area();
		let w = (x1 - x0) / 2.0;
		let h = (y1 - y0) / 2.0;
		(x0 + col as f64 * w, y0 + row as f64 * h, x0 + (col as f64 + 1.0) * w, y0 + (row as f64 + 1.0) * h)
	}

	/// The uniform drawing scale, snapped to [`SCALE_SERIES`]: the largest
	/// standard scale at which every view (and the section) fits its cell with a
	/// 15% margin. `(numerator, denominator)`, e.g. `(1, 2)` for 1:2.
	pub fn scale(&self) -> (u32, u32) {
		let (cx0, cy0, cx1, cy1) = self.cell_rect(0, 0);
		let (cw, ch) = ((cx1 - cx0) * 0.85, (cy1 - cy0) * 0.85);
		let mut needed = f64::INFINITY;
		let mut any = false;
		{
			let mut consider = |sz: DVec2| {
				if sz.x > 1e-9 || sz.y > 1e-9 {
					any = true;
					let sx = if sz.x > 1e-9 { cw / sz.x } else { f64::INFINITY };
					let sy = if sz.y > 1e-9 { ch / sz.y } else { f64::INFINITY };
					needed = needed.min(sx.min(sy));
				}
			};
			for v in &self.views {
				consider(v.size());
			}
			if let Some(s) = &self.section {
				consider(s.max - s.min);
			}
		}
		if !any || !needed.is_finite() {
			return (1, 1);
		}
		let mut best = SCALE_SERIES[SCALE_SERIES.len() - 1];
		for &(num, den) in SCALE_SERIES.iter() {
			if num as f64 / den as f64 <= needed {
				best = (num, den);
				break;
			}
		}
		best
	}

	/// The scale as a printable ratio, e.g. `"1:2"`.
	pub fn scale_text(&self) -> String {
		let (n, d) = self.scale();
		format!("{n}:{d}")
	}

	/// Numeric scale factor (view mm → sheet mm).
	fn scale_factor(&self) -> f64 {
		let (n, d) = self.scale();
		n as f64 / d as f64
	}

	/// Placement transform for a cell: view-space → sheet-space (y up), centring
	/// the given rectangle in the cell.
	fn place_in(&self, cell: (usize, usize), min: DVec2, max: DVec2) -> impl Fn(DVec2) -> DVec2 {
		let (x0, y0, x1, y1) = self.cell_rect(cell.0, cell.1);
		let s = self.scale_factor();
		let cx = (x0 + x1) * 0.5;
		let cy = (y0 + y1) * 0.5;
		let mid = (min + max) * 0.5;
		move |p: DVec2| DVec2::new(cx + (p.x - mid.x) * s, cy + (p.y - mid.y) * s)
	}

	/// Render the sheet as deterministic SVG.
	///
	/// # Determinism
	///
	/// Every coordinate goes through a fixed 4-decimal formatter with negative
	/// zero normalized; entities are emitted in the order their producers built
	/// them in; no hash map is iterated; no clock is read (the date comes from
	/// [`TitleBlock::date`]). Two calls on the same [`Drawing`] produce
	/// byte-identical strings, and so do two runs of the same program.
	///
	/// The output always contains [`HLR_NOTE`] — a sheet that hides hidden
	/// detail must say so.
	pub fn to_svg(&self) -> String {
		let mut s = String::with_capacity(32 * 1024);
		let _ = writeln!(
			s,
			"<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" width=\"{}mm\" height=\"{}mm\" viewBox=\"0 0 {} {}\">",
			mm(SHEET_W),
			mm(SHEET_H),
			mm(SHEET_W),
			mm(SHEET_H)
		);
		let _ = writeln!(s, "<title>{}</title>", xml_escape(&self.title.part_name));
		let _ = writeln!(s, "<rect x=\"0\" y=\"0\" width=\"{}\" height=\"{}\" fill=\"#ffffff\"/>", mm(SHEET_W), mm(SHEET_H));
		let _ = writeln!(
			s,
			"<rect x=\"{}\" y=\"{}\" width=\"{}\" height=\"{}\" fill=\"none\" stroke=\"#000000\" stroke-width=\"0.5\"/>",
			mm(SHEET_MARGIN * 0.5),
			mm(SHEET_MARGIN * 0.5),
			mm(SHEET_W - SHEET_MARGIN),
			mm(SHEET_H - SHEET_MARGIN)
		);
		for v in &self.views {
			self.svg_view(&mut s, v);
		}
		if let Some(sec) = &self.section {
			self.svg_section(&mut s, sec);
		}
		self.svg_dimensions(&mut s);
		self.svg_title_block(&mut s);
		self.svg_notes(&mut s);
		let _ = writeln!(s, "</svg>");
		s
	}

	/// One view's geometry plus its label.
	fn svg_view(&self, s: &mut String, v: &View) {
		let cell = Self::cell_of(v.dir);
		let place = self.place_in(cell, v.min, v.max);
		let flip = |p: DVec2| SHEET_H - p.y; // SVG y grows downward
		let _ = writeln!(s, "<g id=\"view-{}\" stroke=\"#000000\" stroke-width=\"0.35\" fill=\"none\">", v.dir.label().to_lowercase());
		for e in &v.entities {
			match e {
				ViewEntity::Segment { a, b } => {
					let (pa, pb) = (place(*a), place(*b));
					let _ = writeln!(s, "<line x1=\"{}\" y1=\"{}\" x2=\"{}\" y2=\"{}\"/>", mm(pa.x), mm(flip(pa)), mm(pb.x), mm(flip(pb)));
				}
				ViewEntity::Circle { center, radius } => {
					let pc = place(*center);
					let _ = writeln!(s, "<circle cx=\"{}\" cy=\"{}\" r=\"{}\"/>", mm(pc.x), mm(flip(pc)), mm(radius * self.scale_factor()));
				}
				ViewEntity::Polyline(pts) => {
					let _ = writeln!(s, "<path d=\"{}\"/>", svg_path(pts, &place));
				}
			}
		}
		let (cx0, cy0, cx1, _) = self.cell_rect(cell.0, cell.1);
		let _ = writeln!(
			s,
			"<text x=\"{}\" y=\"{}\" font-family=\"monospace\" font-size=\"4\" fill=\"#000000\" stroke=\"none\" text-anchor=\"middle\">{}</text>",
			mm((cx0 + cx1) * 0.5),
			mm(SHEET_H - (cy0 + 4.0)),
			xml_escape(v.dir.label())
		);
		let _ = writeln!(s, "</g>");
	}

	/// The section: hatch first (so the outline draws over it), then boundary.
	fn svg_section(&self, s: &mut String, sec: &SectionView) {
		let cell = self.section_cell();
		let place = self.place_in(cell, sec.min, sec.max);
		let flip = |p: DVec2| SHEET_H - p.y;
		let _ = writeln!(s, "<g id=\"section\" stroke=\"#000000\" fill=\"none\">");
		for (a, b) in &sec.hatch {
			let (pa, pb) = (place(*a), place(*b));
			let _ = writeln!(
				s,
				"<line x1=\"{}\" y1=\"{}\" x2=\"{}\" y2=\"{}\" stroke-width=\"0.15\"/>",
				mm(pa.x),
				mm(flip(pa)),
				mm(pb.x),
				mm(flip(pb))
			);
		}
		for e in &sec.boundary {
			if let ViewEntity::Polyline(pts) = e {
				let _ = writeln!(s, "<path d=\"{}\" stroke-width=\"0.5\"/>", svg_path(pts, &place));
			}
		}
		let (cx0, cy0, cx1, _) = self.cell_rect(cell.0, cell.1);
		let area = sec.area_mm2.map(|a| format!("  AREA {} mm2", dim_text(a))).unwrap_or_default();
		let _ = writeln!(
			s,
			"<text x=\"{}\" y=\"{}\" font-family=\"monospace\" font-size=\"4\" fill=\"#000000\" stroke=\"none\" text-anchor=\"middle\">{}</text>",
			mm((cx0 + cx1) * 0.5),
			mm(SHEET_H - (cy0 + 4.0)),
			xml_escape(&format!("SECTION {}{}", sec.label, area))
		);
		let _ = writeln!(s, "</g>");
	}

	/// Graphical dimensions: extent dimension lines beside the views that show
	/// the axis, and a Ø leader per bore in the first view that reads it as a
	/// circle.
	fn svg_dimensions(&self, s: &mut String) {
		let _ = writeln!(s, "<g id=\"dimensions\" stroke=\"#000000\" stroke-width=\"0.25\" fill=\"none\">");
		let scale = self.scale_factor();
		for d in self.dimensions.iter().filter(|d| d.from_measure == M_EXTENT) {
			// The subject names the axis (see `measure_dimension`); X and Z read on
			// FRONT, Y on TOP.
			let (dir, horizontal) = if d.subject.ends_with("along x") {
				(ViewDir::Front, true)
			} else if d.subject.ends_with("along z") {
				(ViewDir::Front, false)
			} else if d.subject.ends_with("along y") {
				(ViewDir::Top, true)
			} else {
				continue; // not one of the three overall extents — schedule only
			};
			let Some(v) = self.views.iter().find(|v| v.dir == dir) else {
				continue;
			};
			let place = self.place_in(Self::cell_of(v.dir), v.min, v.max);
			let lo = place(v.min);
			let hi = place(v.max);
			if horizontal {
				let y = SHEET_H - (lo.y - 8.0);
				svg_dim_line(s, DVec2::new(lo.x, y), DVec2::new(hi.x, y), &d.text, true);
			} else {
				let x = lo.x - 8.0;
				svg_dim_line(s, DVec2::new(x, SHEET_H - lo.y), DVec2::new(x, SHEET_H - hi.y), &d.text, false);
			}
		}
		let circle_view = self.views.iter().find(|v| v.circles > 0);
		for d in self.dimensions.iter().filter(|d| d.kind == DimKind::Diameter) {
			let (Some(anchor), Some(v)) = (d.anchor, circle_view) else {
				continue;
			};
			let (right, up, _) = v.dir.frame();
			let place = self.place_in(Self::cell_of(v.dir), v.min, v.max);
			let c = place(project(anchor, right, up));
			let off = d.value * 0.5 * scale * std::f64::consts::FRAC_1_SQRT_2;
			let tip = DVec2::new(c.x + off, c.y + off);
			let end = DVec2::new(tip.x + 9.0, tip.y + 9.0);
			let _ = writeln!(
				s,
				"<line x1=\"{}\" y1=\"{}\" x2=\"{}\" y2=\"{}\"/>",
				mm(tip.x),
				mm(SHEET_H - tip.y),
				mm(end.x),
				mm(SHEET_H - end.y)
			);
			let _ = writeln!(
				s,
				"<text x=\"{}\" y=\"{}\" font-family=\"monospace\" font-size=\"3.2\" fill=\"#000000\" stroke=\"none\">{}</text>",
				mm(end.x + 0.8),
				mm(SHEET_H - end.y - 0.8),
				xml_escape(&d.text)
			);
		}
		let _ = writeln!(s, "</g>");
	}

	/// The title block, bottom-right.
	fn svg_title_block(&self, s: &mut String) {
		let x0 = SHEET_W - SHEET_MARGIN - TITLE_W;
		let y0 = SHEET_MARGIN;
		let _ = writeln!(s, "<g id=\"title-block\" stroke=\"#000000\" stroke-width=\"0.35\" fill=\"none\">");
		let _ = writeln!(
			s,
			"<rect x=\"{}\" y=\"{}\" width=\"{}\" height=\"{}\"/>",
			mm(x0),
			mm(SHEET_H - y0 - TITLE_H),
			mm(TITLE_W),
			mm(TITLE_H)
		);
		for (i, (k, v)) in self.title.rows(&self.scale_text()).iter().enumerate() {
			let ty = SHEET_H - (y0 + TITLE_H - 5.0 - i as f64 * 4.2);
			let _ = writeln!(
				s,
				"<text x=\"{}\" y=\"{}\" font-family=\"monospace\" font-size=\"3\" fill=\"#000000\" stroke=\"none\">{}</text>",
				mm(x0 + 2.0),
				mm(ty),
				xml_escape(&format!("{k}: {v}"))
			);
		}
		let _ = writeln!(s, "</g>");
	}

	/// The notes strip and the dimension schedule.
	fn svg_notes(&self, s: &mut String) {
		let _ = writeln!(s, "<g id=\"notes\" fill=\"#000000\" stroke=\"none\" font-family=\"monospace\">");
		let mut y = SHEET_H - (SHEET_MARGIN + NOTES_H - 4.0);
		for (i, note) in self.all_notes().iter().enumerate() {
			for (j, line) in wrap(note, 150).into_iter().enumerate() {
				let prefix = if j == 0 { format!("{}. ", i + 1) } else { "   ".to_string() };
				let _ = writeln!(
					s,
					"<text x=\"{}\" y=\"{}\" font-size=\"2.6\">{}</text>",
					mm(SHEET_MARGIN + 2.0),
					mm(y),
					xml_escape(&format!("{prefix}{line}"))
				);
				y += 3.0;
			}
		}
		let _ = writeln!(
			s,
			"<text x=\"{}\" y=\"{}\" font-size=\"3\" font-weight=\"bold\">DIMENSION SCHEDULE (value | class | subject | measure)</text>",
			mm(SHEET_MARGIN + 2.0),
			mm(y + 1.5)
		);
		y += 5.0;
		for d in &self.dimensions {
			let _ = writeln!(
				s,
				"<text x=\"{}\" y=\"{}\" font-size=\"2.4\">{}</text>",
				mm(SHEET_MARGIN + 2.0),
				mm(y),
				xml_escape(&format!("{} | {} | {} | {}", d.text, d.kind.name(), d.subject, d.from_measure))
			);
			y += 2.8;
		}
		let _ = writeln!(s, "</g>");
	}

	/// Render the sheet as **DXF R12 ASCII** (`AC1009`): `LINE`, `CIRCLE` and
	/// `TEXT` entities on named layers, plus the minimal `HEADER` / `TABLES`
	/// sections an R12 reader expects.
	///
	/// # Scope, stated
	///
	/// This is a *geometry + annotation* exchange, not a full drafting file. It
	/// deliberately emits no `DIMENSION` entities (those carry a block reference
	/// and a `DIMSTYLE` table — out of this slice), no `HATCH` entity (R12
	/// predates it; the hatch travels as plain `LINE`s on the `HATCH` layer,
	/// which is what R12 files did anyway), and no `LWPOLYLINE` (also
	/// post-R12 — polylines travel as `LINE` runs). The dimension *values*
	/// travel as `TEXT`, exactly as they appear on the SVG, and the `Ø` symbol
	/// is written as the AutoCAD control code `%%C`. Byte-stable by the same
	/// rules as [`Self::to_svg`].
	pub fn to_dxf(&self) -> String {
		let mut s = String::with_capacity(32 * 1024);
		dxf_header(&mut s);
		// ENTITIES
		dxf_pair(&mut s, 0, "SECTION");
		dxf_pair(&mut s, 2, "ENTITIES");
		for v in &self.views {
			let place = self.place_in(Self::cell_of(v.dir), v.min, v.max);
			for e in &v.entities {
				match e {
					ViewEntity::Segment { a, b } => dxf_line(&mut s, "OUTLINE", place(*a), place(*b)),
					ViewEntity::Circle { center, radius } => dxf_circle(&mut s, "OUTLINE", place(*center), radius * self.scale_factor()),
					ViewEntity::Polyline(pts) => dxf_closed_polyline(&mut s, "OUTLINE", pts, &place),
				}
			}
		}
		if let Some(sec) = &self.section {
			let place = self.place_in(self.section_cell(), sec.min, sec.max);
			for (a, b) in &sec.hatch {
				dxf_line(&mut s, "HATCH", place(*a), place(*b));
			}
			for e in &sec.boundary {
				if let ViewEntity::Polyline(pts) = e {
					dxf_closed_polyline(&mut s, "SECTION", pts, &place);
				}
			}
		}
		let (b0, b1) = (SHEET_MARGIN * 0.5, SHEET_MARGIN * 0.5);
		let (b2, b3) = (SHEET_W - SHEET_MARGIN * 0.5, SHEET_H - SHEET_MARGIN * 0.5);
		dxf_line(&mut s, "OUTLINE", DVec2::new(b0, b1), DVec2::new(b2, b1));
		dxf_line(&mut s, "OUTLINE", DVec2::new(b2, b1), DVec2::new(b2, b3));
		dxf_line(&mut s, "OUTLINE", DVec2::new(b2, b3), DVec2::new(b0, b3));
		dxf_line(&mut s, "OUTLINE", DVec2::new(b0, b3), DVec2::new(b0, b1));
		let mut y = SHEET_MARGIN + NOTES_H - 4.0;
		for (i, note) in self.all_notes().iter().enumerate() {
			for (j, line) in wrap(note, 150).into_iter().enumerate() {
				let prefix = if j == 0 { format!("{}. ", i + 1) } else { "   ".to_string() };
				dxf_text(&mut s, "TEXT", DVec2::new(SHEET_MARGIN + 2.0, y), 2.6, &format!("{prefix}{line}"));
				y -= 3.0;
			}
		}
		dxf_text(&mut s, "TEXT", DVec2::new(SHEET_MARGIN + 2.0, y - 1.5), 3.0, "DIMENSION SCHEDULE (value | class | subject | measure)");
		y -= 5.0;
		for d in &self.dimensions {
			dxf_text(
				&mut s,
				"DIM",
				DVec2::new(SHEET_MARGIN + 2.0, y),
				2.4,
				&format!("{} | {} | {} | {}", d.text, d.kind.name(), d.subject, d.from_measure),
			);
			y -= 2.8;
		}
		let tx = SHEET_W - SHEET_MARGIN - TITLE_W + 2.0;
		let mut ty = SHEET_MARGIN + TITLE_H - 5.0;
		for (k, v) in self.title.rows(&self.scale_text()) {
			dxf_text(&mut s, "TEXT", DVec2::new(tx, ty), 3.0, &format!("{k}: {v}"));
			ty -= 4.2;
		}
		dxf_pair(&mut s, 0, "ENDSEC");
		dxf_pair(&mut s, 0, "EOF");
		s
	}
}

/// An SVG closed-path `d` attribute for a projected polyline.
fn svg_path(pts: &[DVec2], place: &impl Fn(DVec2) -> DVec2) -> String {
	let mut d = String::new();
	for (i, p) in pts.iter().enumerate() {
		let q = place(*p);
		let _ = write!(d, "{}{} {}", if i == 0 { "M" } else { " L" }, mm(q.x), mm(SHEET_H - q.y));
	}
	d.push_str(" Z");
	d
}

/// One dimension line with arrowheads and centred text. `a`/`b` are already in
/// SVG (y-down) sheet coordinates.
fn svg_dim_line(s: &mut String, a: DVec2, b: DVec2, text: &str, horizontal: bool) {
	let _ = writeln!(s, "<line x1=\"{}\" y1=\"{}\" x2=\"{}\" y2=\"{}\"/>", mm(a.x), mm(a.y), mm(b.x), mm(b.y));
	const ARROW: f64 = 1.6;
	let tri = |s: &mut String, tipx: f64, tipy: f64, dx: f64, dy: f64| {
		let _ = writeln!(
			s,
			"<polygon points=\"{},{} {},{} {},{}\" fill=\"#000000\" stroke=\"none\"/>",
			mm(tipx),
			mm(tipy),
			mm(tipx + dx - dy * 0.4),
			mm(tipy + dy - dx * 0.4),
			mm(tipx + dx + dy * 0.4),
			mm(tipy + dy + dx * 0.4)
		);
	};
	if horizontal {
		tri(s, a.x, a.y, ARROW, 0.0);
		tri(s, b.x, b.y, -ARROW, 0.0);
		let _ = writeln!(
			s,
			"<text x=\"{}\" y=\"{}\" font-family=\"monospace\" font-size=\"3.5\" fill=\"#000000\" stroke=\"none\" text-anchor=\"middle\">{}</text>",
			mm((a.x + b.x) * 0.5),
			mm(a.y - 1.2),
			xml_escape(text)
		);
	} else {
		tri(s, a.x, a.y, 0.0, -ARROW);
		tri(s, b.x, b.y, 0.0, ARROW);
		let (tx, ty) = (a.x - 1.2, (a.y + b.y) * 0.5);
		let _ = writeln!(
			s,
			"<text x=\"{}\" y=\"{}\" font-family=\"monospace\" font-size=\"3.5\" fill=\"#000000\" stroke=\"none\" text-anchor=\"middle\" transform=\"rotate(-90 {} {})\">{}</text>",
			mm(tx),
			mm(ty),
			mm(tx),
			mm(ty),
			xml_escape(text)
		);
	}
}

/// One DXF group-code / value pair.
fn dxf_pair(s: &mut String, code: i32, value: &str) {
	let _ = writeln!(s, "{code}\n{value}");
}

/// The DXF `HEADER` + `TABLES` prologue (R12 / `AC1009`).
fn dxf_header(s: &mut String) {
	dxf_pair(s, 0, "SECTION");
	dxf_pair(s, 2, "HEADER");
	dxf_pair(s, 9, "$ACADVER");
	dxf_pair(s, 1, "AC1009");
	dxf_pair(s, 9, "$INSBASE");
	dxf_pair(s, 10, "0.0000");
	dxf_pair(s, 20, "0.0000");
	dxf_pair(s, 30, "0.0000");
	dxf_pair(s, 9, "$EXTMIN");
	dxf_pair(s, 10, "0.0000");
	dxf_pair(s, 20, "0.0000");
	dxf_pair(s, 30, "0.0000");
	dxf_pair(s, 9, "$EXTMAX");
	dxf_pair(s, 10, &mm(SHEET_W));
	dxf_pair(s, 20, &mm(SHEET_H));
	dxf_pair(s, 30, "0.0000");
	dxf_pair(s, 0, "ENDSEC");
	dxf_pair(s, 0, "SECTION");
	dxf_pair(s, 2, "TABLES");
	dxf_pair(s, 0, "TABLE");
	dxf_pair(s, 2, "LTYPE");
	dxf_pair(s, 70, "1");
	dxf_pair(s, 0, "LTYPE");
	dxf_pair(s, 2, "CONTINUOUS");
	dxf_pair(s, 70, "0");
	dxf_pair(s, 3, "Solid line");
	dxf_pair(s, 72, "65");
	dxf_pair(s, 73, "0");
	dxf_pair(s, 40, "0.0000");
	dxf_pair(s, 0, "ENDTAB");
	dxf_pair(s, 0, "TABLE");
	dxf_pair(s, 2, "LAYER");
	dxf_pair(s, 70, "5");
	for (name, color) in [("OUTLINE", "7"), ("SECTION", "7"), ("HATCH", "8"), ("DIM", "3"), ("TEXT", "2")] {
		dxf_pair(s, 0, "LAYER");
		dxf_pair(s, 2, name);
		dxf_pair(s, 70, "0");
		dxf_pair(s, 62, color);
		dxf_pair(s, 6, "CONTINUOUS");
	}
	dxf_pair(s, 0, "ENDTAB");
	dxf_pair(s, 0, "ENDSEC");
}

/// A DXF `LINE` entity.
fn dxf_line(s: &mut String, layer: &str, a: DVec2, b: DVec2) {
	let _ = writeln!(
		s,
		"0\nLINE\n8\n{}\n10\n{}\n20\n{}\n30\n0.0000\n11\n{}\n21\n{}\n31\n0.0000",
		layer,
		mm(a.x),
		mm(a.y),
		mm(b.x),
		mm(b.y)
	);
}

/// A DXF `CIRCLE` entity.
fn dxf_circle(s: &mut String, layer: &str, c: DVec2, r: f64) {
	let _ = writeln!(s, "0\nCIRCLE\n8\n{}\n10\n{}\n20\n{}\n30\n0.0000\n40\n{}", layer, mm(c.x), mm(c.y), mm(r));
}

/// A closed polyline as a run of DXF `LINE`s (R12 has no `LWPOLYLINE`).
fn dxf_closed_polyline(s: &mut String, layer: &str, pts: &[DVec2], place: &impl Fn(DVec2) -> DVec2) {
	for w in pts.windows(2) {
		dxf_line(s, layer, place(w[0]), place(w[1]));
	}
	if pts.len() > 2 {
		dxf_line(s, layer, place(pts[pts.len() - 1]), place(pts[0]));
	}
}

/// A DXF `TEXT` entity. DXF strings are single-line, and the `Ø` symbol becomes
/// the AutoCAD control code `%%C`.
fn dxf_text(s: &mut String, layer: &str, p: DVec2, height: f64, body: &str) {
	let body = body.replace(['\n', '\r'], " ").replace('\u{d8}', "%%C");
	let _ = writeln!(s, "0\nTEXT\n8\n{}\n10\n{}\n20\n{}\n30\n0.0000\n40\n{}\n1\n{}", layer, mm(p.x), mm(p.y), mm(height), body);
}

/// Greedy word wrap at `width` **characters** — deterministic, locale-free and
/// UTF-8 safe (it breaks between words, never inside one, so a single word
/// longer than `width` overruns rather than being split mid-character).
fn wrap(text: &str, width: usize) -> Vec<String> {
	if width == 0 {
		return vec![text.to_string()];
	}
	let mut out: Vec<String> = Vec::new();
	let mut line = String::new();
	let mut count = 0usize;
	for word in text.split(' ') {
		let wlen = word.chars().count();
		if count > 0 && count + 1 + wlen > width {
			out.push(std::mem::take(&mut line));
			count = 0;
		}
		if count > 0 {
			line.push(' ');
			count += 1;
		}
		line.push_str(word);
		count += wlen;
	}
	if !line.is_empty() || out.is_empty() {
		out.push(line);
	}
	out
}
