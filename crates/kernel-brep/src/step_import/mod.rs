// Copyright (c) LMCAD. Licensed under the MIT License.

// Copyright (c) LMCAD. Licensed under the MIT License.

//! ISO-10303-21 (STEP) **import**: parse the physical-file syntax into an entity
//! graph and reconstruct a B-rep [`Solid`].
//!
//! This is the read counterpart of [`crate::step_export`]. The parser is general
//! (it tokenises the full instance syntax — strings, enums, references, nested
//! lists and typed records), so it accepts STEP from any AP203/AP214 producer, not
//! only this kernel's own output. The public entry point [`import_step`] is
//! `Result`-returning so a caller (including an AI agent) gets a precise reason on
//! failure; nothing outside the matrix below is silently dropped.
//!
//! ## Support matrix
//!
//! | construct | handling |
//! |---|---|
//! | `PLANE`, `CYLINDRICAL/SPHERICAL/CONICAL/TOROIDAL_SURFACE` | exact analytic [`Surface`] tag |
//! | trimmed `B_SPLINE_SURFACE_WITH_KNOTS` face (incl. rational `_COMPLEX`) | tessellated **on the exact patch**: trim-loop vertices are Newton-projected into parameter space, the loops are triangulated there (monotone sweep for single rings, hole-bridging ear clip otherwise) and the interior is refined to the `PATCH_SAG_TOL` relative chordal tolerance — subject to the `PATCH_MIN_PITCH` area floor that pins the strip against an unsplittable trim chord (bounded residual sag there; the refinement's termination device) — with every interior vertex evaluated via [`NurbsSurface::point_at`]; trim chords are never split, so the weld with neighbour faces stays watertight. Facets carry their own exact `Plane` tags — the analytic [`Surface`] enum has no freeform variant, so the patch's NURBS identity is not on the [`Solid`] (it IS preserved by the [`import_step_freeform`] sidecar; exact patch reads also via [`import_bspline_surface`]) |
//! | **closed/periodic** B-spline face (`S` periodic across a domain end, verified by evaluation): a trim loop crossing the patch seam, a seam edge traversed twice (a real exporter's closed tube wall), or an untrimmed band bounded only by its two full-period rims | unwrapped into the **universal cover** (`unwrap_ring_defined`, mirroring the analytic periodic-wall split): seam-crossing chords continue into the neighbouring period, slit traversals land one period apart and weld back on interning, two opposite full-period rims are bridged by a synthetic seam (`bridge_band_rings`); then the standard ear-clip + on-patch refinement. Caveat: each chord is unwrapped the SHORT way around, so a single trim chord deliberately spanning > half a period reads as a seam crossing |
//! | closed B-spline face whose loops wind the patch in any other combination (same-sense rims, winding holes) | loud [`StepError::Unsupported`]. A trim vertex OFF the patch is accepted within the file's asserted `UNCERTAINTY_MEASURE_WITH_UNIT` (10× that in tolerant mode, reported as a repair); farther off is a loud refusal |
//! | other surfaces (`SURFACE_OF_REVOLUTION`, offset, swept, …) | loud [`StepError::Unsupported`] |
//! | `FACE_OUTER_BOUND` + `FACE_BOUND` | multi-loop faces: planar and B-spline faces keep their inner (hole) loops; a curved ANALYTIC face with holes is tessellated on its exact surface through the parameter-patch path (`add_patch_face` on an `AnalyticPatch`: the loops unwrapped in the surface's periodic chart, hole-bridging ear clip, batched on-surface refinement) |
//! | `LINE` edges (or absent edge geometry) | the exact two-vertex chord |
//! | `CIRCLE` / `ELLIPSE` edges | sweep ≤ 90°: kept as the producer's chord (re-imports of this kernel's own faceted exports stay bit-exact); sweep > 90° through full rings: sampled at the `FULL_TURN_SEGMENTS` pitch (a one-edge full-circle cap becomes a closed 48-segment ring); segments carry the analytic [`Curve`] |
//! | `B_SPLINE_CURVE_WITH_KNOTS` edges (incl. rational `_COMPLEX`) | sampled over the knot domain at a curvature-adaptive pitch: `BSPLINE_EDGE_SEGMENTS` doubled (≤ `MAX_BSPLINE_EDGE_SEGMENTS`) while consecutive chords turn more than the conic ring pitch — so a closed full-circle B-spline rim gets 64 chords, a gentle freeform trim edge keeps 8 |
//! | `SURFACE_CURVE` / `SEAM_CURVE` edge wrappers | unwrapped to their 3-D curve |
//! | other edge geometry (`PARABOLA`, `TRIMMED_CURVE`, …) | loud [`StepError::Unsupported`] |
//! | periodic cylinder/cone face (seam edge + full-circle rims, e.g. a real exporter's cylinder wall) | split into chord-triangle facets on the exact surface via monotone parameter-strip triangulation — these are ruled in the axial direction, so the chords lie on the inscribed prism/frustum |
//! | periodic / pole-spanning sphere and torus regions (full sphere as one face, caps with or without a seam/pole vertex, bands between rims, full torus, torus bands) | resampled into a ring grid of chord facets ON the exact surface: boundary rings are reused verbatim (the weld), interior rings/pole fans are synthesized at the ring pitch (see `resample_periodic_region`) |
//! | general sub-periodic sphere/torus regions (≤ ~137° span, e.g. the recover pass's cubemap/quadrant chart faces) | triangulated on the exact surface: boundary verbatim, interior chord facets refined via the parameter chart (see `general_curved_region`) |
//! | every other sphere/torus region the ring grid refuses (a corner ball bounded by three arcs, a torus band whose rims start at different longitudes, a half-torus wall bounded by two tube circles and an equator seam, a pole-to-pole lune) and every cylinder/cone region the strip splitter refuses | the **parameter-patch fallback**: the loop unwrapped in the surface's normalised periodic chart (`u` = angle / 2π, `v` = latitude, tube angle or scaled axial distance; poles/apex interpolated), a one-rim cap closed through the pole row, a two-rim band bridged by a synthetic seam sampled ON the surface, then triangulated and refined on the exact surface. Loops that wind the chart in any other combination stay a loud [`StepError::Unsupported`] |
//! | `NEXT_ASSEMBLY_USAGE_OCCURRENCE` / `MAPPED_ITEM` assemblies | flattened component instances via [`import_step_assembly`] (names, per-part solids, accumulated placements) |
//! | any face outside the matrix, in **tolerant** mode ([`crate::step_tolerant::import_step_tolerant`]) | flat-repaired or skipped and REPORTED per face and per solid; every solid of the file listed with its product name and placed envelope |
//!
//! Curved-face routing detail: a curved-tagged face whose tessellated boundary has
//! ≤ 4 vertices, or is planar **and** chord-close to its surface, imports as a single
//! chord facet — exactly this kernel's native representation of curved solids (and the
//! shape of its own exports). Only boundaries that cannot be one flat facet (a periodic
//! wall) are split.
//!
//! ## Module map
//! [`parse`] is the physical-file syntax (values, entities, statements);
//! [`importer`] is the typed view over the parsed graph; [`edges`] samples conic
//! and B-spline edge curves. Faces are routed by [`face`], which reaches for
//! [`periodic`] (seam-aware splitting of analytic walls and regions) or for the
//! parameter-patch path — [`patch`] (the `ParamPatch` abstraction),
//! [`patch_tess`] (trim-ring triangulation and interior refinement),
//! [`patch_face`] (the shared read path) and [`triangulate`] (the two polygon
//! triangulators). [`assembly`] walks the NAUO/`MAPPED_ITEM` tree. The tolerant
//! mode's per-face repair and per-solid census live one level up, in
//! [`crate::step_tolerant`], which drives the same routes.

mod assembly;
mod edges;
mod face;
mod importer;
mod parse;
mod patch;
mod patch_face;
mod patch_tess;
mod periodic;
mod triangulate;

use kernel_core::mesh::Mesh;

use crate::nurbs::{FreeformFace, NurbsCurve, NurbsSurface};
use crate::topo::Solid;

// The paths the rest of the crate reaches for stay at `crate::step_import::…`, so
// splitting this module changed no `use` line in `step_export` / `step_tolerant`.
pub use self::assembly::import_step_assembly;

pub(crate) use self::assembly::AssemblyGraph;
pub(crate) use self::edges::{complex_part, edge_sweep, last_enum, ShellFaces};
pub(crate) use self::face::{add_face, add_face_flat, FaceAccum};
pub(crate) use self::importer::{Importer, TOLERANT_SNAP_FACTOR};
pub(crate) use self::parse::{parse_with, Value};
pub(crate) use self::patch_tess::{PATCH_PROJECT_TOL, PATCH_SAG_TOL, PATCH_SEED_GRID};

use self::parse::parse;

/// Why a STEP import failed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum StepError {
	/// The physical-file syntax could not be parsed.
	Parse(String),
	/// A referenced entity id is missing or has the wrong type.
	Reference(String),
	/// A geometry/topology construct this importer does not yet handle
	/// (e.g. a curved surface) was encountered.
	Unsupported(String),
	/// The reconstructed faces do not form a usable solid.
	Topology(String),
}

impl std::fmt::Display for StepError {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		match self {
			StepError::Parse(m) => write!(f, "STEP parse error: {m}"),
			StepError::Reference(m) => write!(f, "STEP reference error: {m}"),
			StepError::Unsupported(m) => write!(f, "unsupported STEP construct: {m}"),
			StepError::Topology(m) => write!(f, "STEP topology error: {m}"),
		}
	}
}

impl std::error::Error for StepError {}

// --- Entry points ------------------------------------------------------------

/// Import a STEP physical-file string and reconstruct a B-rep [`Solid`].
///
/// Faces keep their exact analytic [`Surface`] tags and conic edges their analytic
/// [`Curve`]s. Arc-bounded faces (a cap bounded by ONE full-circle edge), periodic
/// cylinder/cone walls (seam edge + circular rims), pole-spanning / fully periodic
/// sphere and torus regions, trimmed B-spline patches and planar faces with inner
/// (hole) loops are reconstructed per the module-level support matrix; anything
/// outside it is a loud [`StepError::Unsupported`], never a silent drop. Shared
/// vertices are merged by exact position so adjacent faces pair their edges into a
/// watertight 2-manifold. Faces come from every `MANIFOLD_SOLID_BREP` (in entity-id
/// order — a multi-solid file imports as one [`Solid`] with several shells), falling
/// back to all `ADVANCED_FACE`s for bare fragments.
///
/// ```
/// use kernel_brep::{export_step, import_step, cuboid, volume};
/// use kernel_brep::math::DVec3;
/// let box_ = cuboid(DVec3::splat(-1.0), DVec3::splat(1.0));
/// let step = export_step(&box_, "box");
/// let back = import_step(&step).unwrap();
/// assert!((volume(&back).abs() - 8.0).abs() < 1e-9);
/// ```
pub fn import_step(text: &str) -> Result<Solid, StepError> {
	Ok(import_step_freeform(text)?.0)
}

/// [`import_step`] plus the **NURBS sidecar**: every trimmed `B_SPLINE_SURFACE`
/// face's exact rational patch and verbatim trim rings as a [`FreeformFace`], in
/// face order. The [`Solid`] itself carries only the chord facets (the analytic
/// [`Surface`] enum has no freeform variant); the sidecar is what preserves the
/// patches' NURBS identity so [`crate::step_export::export_step_freeform`] can
/// re-export them as true `B_SPLINE_SURFACE_WITH_KNOTS` faces — the writing half
/// of NURBS interchange. Files without B-spline faces return an empty sidecar.
pub fn import_step_freeform(text: &str) -> Result<(Solid, Vec<FreeformFace>), StepError> {
	let ents = parse(text)?;
	let imp = Importer::new(&ents);

	// Deterministic face collection: solid-model shells first, bare faces as fallback.
	let mut face_list: Vec<(u32, bool)> = Vec::new();
	for (_, faces) in imp.brep_face_sets()? {
		face_list.extend(faces);
	}
	if face_list.is_empty() {
		face_list = imp.all_face_ids();
	}

	let mut acc = FaceAccum::default();
	for (fid, flip) in face_list {
		add_face(&imp, fid, flip, &mut acc)?;
	}
	let freeform = std::mem::take(&mut acc.freeform);
	Ok((acc.finish()?, freeform))
}

/// Import the first `B_SPLINE_SURFACE_WITH_KNOTS` (non-rational NURBS) surface in a
/// STEP file into a [`NurbsSurface`] — the reading half of NURBS interchange. The
/// result can be evaluated ([`NurbsSurface::point_at`]) and tessellated
/// ([`NurbsSurface::tessellate`]). Returns [`StepError::Unsupported`] if the file
/// has no such entity.
pub fn import_bspline_surface(text: &str) -> Result<NurbsSurface, StepError> {
	let ents = parse(text)?;
	let imp = Importer::new(&ents);
	let id = ents
		.iter()
		.find(|(_, e)| {
			e.name == "B_SPLINE_SURFACE_WITH_KNOTS"
				|| (e.name == "_COMPLEX" && complex_part(&e.args, "B_SPLINE_SURFACE_WITH_KNOTS").is_some())
		})
		.map(|(&id, _)| id)
		.ok_or_else(|| StepError::Unsupported("no B_SPLINE_SURFACE_WITH_KNOTS entity".into()))?;
	imp.bspline_surface(id)
}

/// Import the first `B_SPLINE_CURVE_WITH_KNOTS` (non-rational NURBS) curve in a STEP
/// file into a [`NurbsCurve`]. Companion to [`import_bspline_surface`] for trim/edge
/// curves. Returns [`StepError::Unsupported`] if the file has no such entity.
pub fn import_bspline_curve(text: &str) -> Result<NurbsCurve, StepError> {
	let ents = parse(text)?;
	let imp = Importer::new(&ents);
	let id = ents
		.iter()
		.find(|(_, e)| {
			e.name == "B_SPLINE_CURVE_WITH_KNOTS"
				|| (e.name == "_COMPLEX" && complex_part(&e.args, "B_SPLINE_CURVE_WITH_KNOTS").is_some())
		})
		.map(|(&id, _)| id)
		.ok_or_else(|| StepError::Unsupported("no B_SPLINE_CURVE_WITH_KNOTS entity".into()))?;
	imp.bspline_curve(id)
}

/// Import the first `B_SPLINE_SURFACE_WITH_KNOTS` in a STEP file and tessellate it
/// into a [`Mesh`] at an `nu × nv` sample grid — the end-to-end NURBS read path: a
/// freeform STEP surface becomes printable/renderable triangles in one call. The
/// patch is sampled over its full parameter domain (untrimmed).
pub fn import_bspline_mesh(text: &str, nu: usize, nv: usize) -> Result<Mesh, StepError> {
	Ok(import_bspline_surface(text)?.tessellate(nu, nv))
}
