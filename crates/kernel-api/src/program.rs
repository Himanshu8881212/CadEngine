// Copyright (c) LMCAD. Licensed under the MIT License.

//! The JSON program vocabulary: every op the binding accepts, as serde types.
//!
//! A program is `{"ops": [ {"id": "...", "op": "<kind>", ...params}, ... ]}`.
//! Geometry-producing ops bind their result to their `id`; later ops reference it
//! through `in` / `a` / `b` / `sketch`. Units are millimetres; angles in the JSON
//! surface are ALWAYS degrees (converted to radians at the kernel boundary).
//!
//! Unknown JSON fields fail closed as `invalid_param`; `_`-prefixed keys are
//! reserved for inert in-op comments. Missing or malformed required parameters
//! are also loud errors.

#[cfg(feature = "catalog")]
use std::collections::BTreeMap;

use serde::Deserialize;

/// Default segment count for cylinders/cones (matches a 32-gon side wall).
fn d32() -> usize {
	32
}
/// Default sphere `u` (around) segment count.
fn du() -> usize {
	32
}
/// Default sphere `v` (pole-to-pole) segment count.
fn dv() -> usize {
	16
}
/// Default torus ring (around the axis) segment count.
fn dring() -> usize {
	48
}
/// Default torus tube (cross-section) segment count.
fn dtube() -> usize {
	24
}
/// Default revolve sector count.
fn d64() -> usize {
	64
}
/// Default rim-fillet quarter-arc facet count.
fn d8() -> usize {
	8
}
/// Default chord tolerance (mm) for adaptive tessellation on export.
fn dtol() -> f64 {
	0.01
}
/// Default build direction (+Z up) for `support_report`.
fn d_up() -> [f64; 3] {
	[0.0, 0.0, 1.0]
}
/// Default FDM support-overhang threshold (deg from vertical) for `support_report`.
fn d_overhang() -> f64 {
	45.0
}
/// Default voxel size (mm) for the watertight voxel-heal fallback on export.
fn dsupersample() -> usize {
	2
}
/// Default chord tolerance (mm) for measurement tessellations (`mesh_components`),
/// matching `support_report`'s working scale.
fn d005() -> f64 {
	0.05
}
/// Default position-weld scale (mm) for mesh connectivity (the house weld scale).
fn dweld() -> f64 {
	1e-3
}
fn diso() -> f64 {
	0.5
}
fn dvoxel() -> f64 {
	0.3
}
/// Default `thin_wall` sampling lattice (per axis).
fn dsamples() -> usize {
	64
}
/// Arcs default to counter-clockwise sweep.
fn dtrue() -> bool {
	true
}
/// Default gear pressure angle in degrees (ISO 53 basic rack).
fn d20() -> f64 {
	20.0
}
/// Clearance holes default to the ISO 273 *medium* (H13) series.
fn dmedium() -> FitSpec {
	FitSpec::Medium
}
/// The `implicit` op defaults to the Lipschitz-pruned narrow-band extractor.
fn dnarrowband() -> MesherSpec {
	MesherSpec::Narrowband
}
/// Default `created_with` stamp of `library_add` provenance — a build-time
/// constant (this binding's version), NEVER a clock.
fn dcreated() -> String {
	concat!("lmcad kernel-api ", env!("CARGO_PKG_VERSION")).to_string()
}

/// Extraction engine for [`OpKind::Implicit`] (JSON values `"narrowband"` /
/// `"manifold"`).
#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MesherSpec {
	/// Narrow-band Dual Contouring (the default): fast — work scales with
	/// surface area — but its block pruning REQUIRES every field to be
	/// ≤ 1-Lipschitz (the analytic shapes are; `expr_sdf` leaves are normalized
	/// by their declared bound; `offset_by`/`lerp` need a slowly varying field).
	Narrowband,
	/// Manifold Dual Contouring: samples every cell (no Lipschitz assumption)
	/// and resolves saddle cells with one vertex per surface patch — the
	/// extractor for junction-rich beam lattices and TPMS shells.
	Manifold,
}

/// Explicit meshing box for [`OpKind::Implicit`] (overrides the tree's own
/// bounds — required when the tree is unbounded, useful for tight domains).
#[derive(Clone, Copy, Debug, Deserialize)]
pub struct DomainSpec {
	/// Low corner of the meshing box.
	pub min: [f64; 3],
	/// High corner of the meshing box.
	pub max: [f64; 3],
}

/// The boolean of an [`OpKind::MeshCarve`] (JSON values `"union"` /
/// `"difference"` / `"intersection"`), applied solid-vs-mesh through the
/// winding-number voxel boolean.
#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BoolOpSpec {
	/// `a ∪ b`.
	Union,
	/// `a − b`.
	Difference,
	/// `a ∩ b`.
	Intersection,
}

/// ISO 273:1979 clearance-hole fit series for the hole-wizard ops
/// (JSON values `"close"` / `"medium"` / `"coarse"`).
#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FitSpec {
	/// Series *fine* (H12) — e.g. M5 → Ø5.3.
	Close,
	/// Series *medium* (H13), the default — e.g. M5 → Ø5.5.
	Medium,
	/// Series *coarse* (H14) — e.g. M5 → Ø5.8.
	Coarse,
}

/// The DIN 6885 form-A keyway slot of a [`OpKind::Shaft`]: where the slot sits
/// along the axis (its width/depth come from the DIN 6885-1 table for the shaft
/// diameter).
#[derive(Clone, Copy, Debug, Deserialize)]
pub struct ShaftKeywaySpec {
	/// Overall slot length along the axis, including the semicircular ends.
	pub length: f64,
	/// Distance from the shaft's z = 0 end face to the start of the slot
	/// (keep `0 < offset` and `offset + length <` shaft length).
	pub offset: f64,
}

/// One declared interface parameter of a [`OpKind::LibraryAdd`] candidate
/// (mirrors `kernel_model::library::ParamSpec`): the name an instantiate may
/// set, its units, default, and admissible `[min, max]` range — the admission
/// gate builds the part at the defaults, sampled range corners, and midpoint.
#[derive(Clone, Debug, Deserialize)]
pub struct LibraryParamSpec {
	/// Parameter name — must exist in the part document's parameter table.
	pub name: String,
	/// Units of the value (`"mm"`, `"deg"`, `"count"`, …); declared, not converted.
	pub units: String,
	/// Default value, used for parameters an instantiate leaves unset.
	pub default: f64,
	/// Inclusive admissible minimum.
	pub min: f64,
	/// Inclusive admissible maximum.
	pub max: f64,
	/// What the parameter means (optional).
	#[serde(default)]
	pub description: String,
}

/// The provenance block of a [`OpKind::LibraryAdd`]: `author` and `date` are
/// REQUIRED and **caller-supplied** — the kernel never stamps clock time into
/// library data, so identical programs always produce identical bytes.
#[derive(Clone, Debug, Deserialize)]
pub struct LibraryProvenanceSpec {
	/// Who authored the part (a person, an AI session id, a pipeline).
	pub author: String,
	/// Caller-supplied date string (e.g. `"2026-06-10"`).
	pub date: String,
	/// What produced the part (defaults to this binding's version stamp).
	#[serde(default = "dcreated")]
	pub created_with: String,
}

/// The metadata of a [`OpKind::LibraryAdd`] candidate entry.
#[derive(Clone, Debug, Deserialize)]
pub struct LibraryMetaSpec {
	/// Library name: 1–64 chars of `A–Z a–z 0–9 . _ -`, starting alphanumeric
	/// (it becomes the stored `.lmcpart` file stem).
	pub name: String,
	/// Version, ≥ 1. One `(name, version)` is immutable once admitted; changed
	/// geometry must be admitted as a new version.
	pub version: u32,
	/// Coarse search grouping (optional).
	#[serde(default)]
	pub category: String,
	/// Search tags (optional; sorted and deduplicated on admission).
	#[serde(default)]
	pub tags: Vec<String>,
	/// Free-text description (optional; searched).
	#[serde(default)]
	pub description: String,
	/// Authorship record (caller-supplied `date`).
	pub provenance: LibraryProvenanceSpec,
	/// The declared public parameter interface (may be empty for a fixed part).
	#[serde(default)]
	pub params: Vec<LibraryParamSpec>,
}

/// The mirror plane of an [`OpKind::Mirror`]: a point on the plane plus its
/// normal (any non-zero vector, normalized internally).
#[derive(Clone, Copy, Debug, Deserialize)]
pub struct PlaneSpec {
	/// A point the plane passes through.
	pub point: [f64; 3],
	/// Plane normal — any non-zero finite vector, normalized internally.
	pub normal: [f64; 3],
}

/// The rotation half of a [`OpKind::Pose`]: `degrees` about the (any-direction)
/// `axis` through `center`.
#[derive(Clone, Copy, Debug, Deserialize)]
pub struct RotateSpec {
	/// Rotation axis — any non-zero vector, normalized internally.
	pub axis: [f64; 3],
	/// Rotation angle in degrees, right-handed about `axis`.
	pub degrees: f64,
	/// A point the axis passes through (default: the origin).
	#[serde(default)]
	pub center: [f64; 3],
}

/// Instance material for assembly mass/BOM receipts (`asm_instance.material`).
#[derive(Clone, Debug, Deserialize)]
pub struct MaterialSpec {
	/// Material name (free text — appears in receipts/BOM verbatim).
	pub name: String,
	/// Density in g/cm³.
	pub density_g_cm3: f64,
}

/// Tolerance window of an [`OpKind::Assert`] numeric check: the measured value
/// must land in `target ± abs` or `target ± percent·target/100` (exactly one of
/// `abs` / `percent` is required).
#[derive(Clone, Copy, Debug, Deserialize)]
pub struct WithinSpec {
	/// The expected value.
	pub target: f64,
	/// Absolute half-width tolerance (same unit as the measure).
	pub abs: Option<f64>,
	/// Relative half-width tolerance, in percent of `target`.
	pub percent: Option<f64>,
}

/// The repeated cut of a [`OpKind::BoltCircle`], tagged by `kind` — pure data
/// (no closures cross the JSON boundary): one of the five hole-wizard cuts,
/// with that cut's own parameters.
#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum BoltHoleSpec {
	/// Plain Ø`d` drill (blind `depth` with the 118° point, or `through`).
	Drill { d: f64, depth: Option<f64>, through: Option<f64> },
	/// ISO 273 clearance hole for an M-`m` screw (always through).
	Clearance {
		m: f64,
		#[serde(default = "dmedium")]
		fit: FitSpec,
	},
	/// Clearance hole + DIN 974-1 counterbore for a DIN 912 cap screw.
	Counterbore {
		m: f64,
		#[serde(default = "dmedium")]
		fit: FitSpec,
	},
	/// Clearance hole + DIN 74-1 form F 90° countersink (M3+).
	Countersink {
		m: f64,
		#[serde(default = "dmedium")]
		fit: FitSpec,
	},
	/// ISO coarse tap-drill pilot (Ø = m − pitch).
	TapDrill { m: f64, depth: Option<f64>, through: Option<f64> },
}

/// A circular-arc edge of a [`OpKind::Sketch`]: endpoints `a`, `b` and a `center`
/// construction point (all indices into the sketch's `points`).
#[derive(Clone, Debug, Deserialize)]
pub struct ArcSpec {
	/// First endpoint (point index).
	pub a: usize,
	/// Second endpoint (point index).
	pub b: usize,
	/// Center construction point (point index, not part of the boundary loop).
	pub center: usize,
	/// Sweep counter-clockwise from `a` to `b` (default true).
	#[serde(default = "dtrue")]
	pub ccw: bool,
}

/// A standalone full circle of a [`OpKind::Sketch`]: a `center` point and a
/// `radius_point` lying on the circle.
#[derive(Clone, Debug, Deserialize)]
pub struct CircleSpec {
	/// Center (point index).
	pub center: usize,
	/// A point on the circle (point index) — its distance to `center` is the radius.
	pub radius_point: usize,
}

/// One sketch constraint, tagged by `kind`. All `a`/`b`/`c`/`d`/`point`/`line_*`/
/// `center`/`radius_point` fields are indices into the sketch's `points` array.
#[derive(Clone, Debug, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum ConstraintSpec {
	/// Pin `point` to the position `at` (the sketch's ground anchor).
	Fixed { point: usize, at: [f64; 2] },
	/// Make two points share one position.
	Coincident { a: usize, b: usize },
	/// Hold the segment `a → b` horizontal (`y_a == y_b`).
	Horizontal { a: usize, b: usize },
	/// Hold the segment `a → b` vertical (`x_a == x_b`).
	Vertical { a: usize, b: usize },
	/// Hold the distance between `a` and `b` at `distance`.
	Distance { a: usize, b: usize, distance: f64 },
	/// Keep directions `a → b` and `c → d` parallel.
	Parallel { a: usize, b: usize, c: usize, d: usize },
	/// Keep directions `a → b` and `c → d` perpendicular.
	Perpendicular { a: usize, b: usize, c: usize, d: usize },
	/// Force segments `a → b` and `c → d` to equal length.
	EqualLength { a: usize, b: usize, c: usize, d: usize },
	/// Hold the line `line_a → line_b` tangent to the circle centered at `center`
	/// through `radius_point`.
	Tangent { line_a: usize, line_b: usize, center: usize, radius_point: usize },
	/// Hold the angle between directions `a → b` and `c → d` at `degrees`
	/// (magnitude — the solver may settle at ±`degrees`).
	Angle { a: usize, b: usize, c: usize, d: usize, degrees: f64 },
	/// Make `a` and `b` mirror images across the line `line_a → line_b`.
	Symmetric { a: usize, b: usize, line_a: usize, line_b: usize },
}

impl ConstraintSpec {
	/// Every point index this constraint references (for bounds checking).
	pub fn point_indices(&self) -> Vec<usize> {
		match *self {
			ConstraintSpec::Fixed { point, .. } => vec![point],
			ConstraintSpec::Coincident { a, b }
			| ConstraintSpec::Horizontal { a, b }
			| ConstraintSpec::Vertical { a, b }
			| ConstraintSpec::Distance { a, b, .. } => vec![a, b],
			ConstraintSpec::Parallel { a, b, c, d }
			| ConstraintSpec::Perpendicular { a, b, c, d }
			| ConstraintSpec::EqualLength { a, b, c, d }
			| ConstraintSpec::Angle { a, b, c, d, .. } => vec![a, b, c, d],
			ConstraintSpec::Tangent { line_a, line_b, center, radius_point } => {
				vec![line_a, line_b, center, radius_point]
			}
			ConstraintSpec::Symmetric { a, b, line_a, line_b } => vec![a, b, line_a, line_b],
		}
	}
}

/// Every operation the binding executes, tagged by the JSON `op` field.
///
/// Snake-case op names (`"op": "fillet_edge_near"`). The `in` JSON field maps to
/// the `input` Rust field. See `API.md` at the repo root for the per-op cookbook
/// (params, defaults, one runnable example each).
#[derive(Clone, Debug, Deserialize)]
#[serde(tag = "op", rename_all = "snake_case")]
pub enum OpKind {
	// --- Solid primitives & sweeps -------------------------------------------
	/// Axis-aligned box from two corners.
	Box { min: [f64; 3], max: [f64; 3] },
	/// Cylinder from a base-cap center along `axis`.
	Cylinder {
		base: [f64; 3],
		axis: [f64; 3],
		radius: f64,
		height: f64,
		#[serde(default = "d32")]
		segments: usize,
	},
	/// UV sphere.
	Sphere {
		center: [f64; 3],
		radius: f64,
		#[serde(default = "du")]
		u: usize,
		#[serde(default = "dv")]
		v: usize,
	},
	/// Cone from a base disc of `radius` tapering to an apex at `height` — or,
	/// with `top_radius`, the FRUSTUM (truncated cone) that same taper cuts at
	/// `height`. A frustum is the shape almost every printed part actually wants
	/// (a draughted boss, a chamfered spigot, a tapered stand-off); without it a
	/// designer had to build a cone and difference the tip off.
	Cone {
		base: [f64; 3],
		axis: [f64; 3],
		radius: f64,
		height: f64,
		#[serde(default = "d32")]
		segments: usize,
		/// Radius of the flat top face (mm). Omit (or 0) for a true cone — an
		/// apex. Must differ from `radius`: equal radii are a cylinder, and the
		/// op refuses rather than emit a cone surface with no apex.
		top_radius: Option<f64>,
	},
	/// Torus around `axis` (`minor` < `major`).
	Torus {
		center: [f64; 3],
		axis: [f64; 3],
		major: f64,
		minor: f64,
		#[serde(default = "dring")]
		ring_segments: usize,
		#[serde(default = "dtube")]
		tube_segments: usize,
	},
	/// Linear extrusion of a closed CCW XY profile along +Z.
	Extrude { profile: Vec<[f64; 2]>, height: f64 },
	/// Extrusion of an outer profile with hole loops.
	ExtrudeWithHoles {
		outer: Vec<[f64; 2]>,
		holes: Vec<Vec<[f64; 2]>>,
		height: f64,
	},
	/// Drafted extrusion: walls slope inward by `draft_deg` (convex profiles only).
	ExtrudeTapered { profile: Vec<[f64; 2]>, height: f64, draft_deg: f64 },
	/// Full 360° revolution of an `(r, z)` profile about the Z axis.
	Revolve {
		profile: Vec<[f64; 2]>,
		#[serde(default = "d64")]
		segments: usize,
	},
	/// Loft a closed solid through a stack of closed 3D section loops. Every
	/// section must share the same point count (≥3), be ordered along the loft
	/// direction, and wind consistently; the lateral band and end caps are
	/// faceted (honest tessellation of the morph). See API.md.
	Loft { sections: Vec<Vec<[f64; 3]>> },
	/// Sweep a closed 3D `profile` along a 3D `path` polyline with a
	/// rotation-minimising frame. Needs ≥3 profile points and ≥2 path points.
	Sweep { profile: Vec<[f64; 3]>, path: Vec<[f64; 3]> },

	// --- Sketch ----------------------------------------------------------------
	/// A constrained 2D sketch: solved on creation, DOF analysis in the measures.
	Sketch {
		points: Vec<[f64; 2]>,
		#[serde(default)]
		segments: Vec<[usize; 2]>,
		#[serde(default)]
		arcs: Vec<ArcSpec>,
		#[serde(default)]
		circles: Vec<CircleSpec>,
		#[serde(default)]
		constraints: Vec<ConstraintSpec>,
	},
	/// Extrude a previously solved sketch along +Z.
	#[cfg(feature = "catalog")]
	SketchExtrude { sketch: String, height: f64 },
	/// Revolve a previously solved sketch (its `(x, y)` read as `(r, z)`) about Z.
	SketchRevolve {
		sketch: String,
		#[serde(default = "d64")]
		segments: usize,
	},

	// --- Booleans ----------------------------------------------------------------
	/// `a ∪ b`.
	Union { a: String, b: String },
	/// `a − b`.
	Difference { a: String, b: String },
	/// `a ∩ b`.
	Intersection { a: String, b: String },
	/// n-ary union: fold every listed solid into one result, left to right.
	UnionAll {
		#[serde(rename = "in")]
		input: Vec<String>,
	},

	// --- Features & transforms ---------------------------------------------------
	/// Round the straight edge nearest `witness` with a cylindrical fillet.
	FilletEdgeNear {
		#[serde(rename = "in")]
		input: String,
		witness: [f64; 3],
		radius: f64,
		/// Reject a witness farther than this from every edge (default: 10% of the
		/// solid's bounding-box diagonal).
		max_distance: Option<f64>,
	},
	/// Flat-bevel the straight edge nearest `witness` with setback `radius`.
	ChamferEdgeNear {
		#[serde(rename = "in")]
		input: String,
		witness: [f64; 3],
		radius: f64,
		/// Same witness guard as `fillet_edge_near`'s `max_distance`.
		max_distance: Option<f64>,
	},
	/// Exact-torus fillet of the circular convex rim nearest `witness`.
	FilletCircularRim {
		#[serde(rename = "in")]
		input: String,
		witness: [f64; 3],
		radius: f64,
		#[serde(default = "d8")]
		arc_segments: usize,
	},
	/// Rigid translation by `offset`.
	Translate {
		#[serde(rename = "in")]
		input: String,
		offset: [f64; 3],
	},
	/// Rigid rotation about the world Z axis by `degrees`.
	RotateZ {
		#[serde(rename = "in")]
		input: String,
		degrees: f64,
	},
	/// Rigid rotation about the world X axis by `degrees` (sibling of `rotate_z`).
	RotateX {
		#[serde(rename = "in")]
		input: String,
		degrees: f64,
	},
	/// Rigid rotation about the world Y axis by `degrees` (sibling of `rotate_z`).
	RotateY {
		#[serde(rename = "in")]
		input: String,
		degrees: f64,
	},
	/// General rigid pose: rotate about an ARBITRARY axis (through
	/// `rotate.center`, default the origin) by `rotate.degrees`, THEN translate
	/// by `translate`. At least one of the two parts is required.
	Pose {
		#[serde(rename = "in")]
		input: String,
		translate: Option<[f64; 3]>,
		rotate: Option<RotateSpec>,
	},
	/// Reflect a solid across a plane (point + normal). Orientation-safe: the
	/// kernel's dedicated `Solid::mirrored` rebuilds every face loop reversed, so
	/// the reflected copy is a correctly-oriented (outward-normal) valid solid —
	/// never an inside-out one.
	Mirror {
		#[serde(rename = "in")]
		input: String,
		plane: PlaneSpec,
	},
	/// `count` clones of a solid at offsets `i·step` (i = 0..count), folded into
	/// ONE solid with the exact boolean union. Disjoint clones are honest
	/// multi-shell solids (`validate().shells == count`); overlapping clones fuse.
	LinearPattern {
		#[serde(rename = "in")]
		input: String,
		/// Number of instances INCLUDING the original (2..=500).
		count: usize,
		/// Per-instance offset vector (mm); must be non-zero.
		step: [f64; 3],
	},
	/// `count` clones of a solid rotated `k·step_deg` (k = 0..count) about `axis`
	/// through `center`, folded into ONE solid with the exact boolean union
	/// (same disjoint-shells / overlap-fuse behavior as `linear_pattern`).
	PolarPattern {
		#[serde(rename = "in")]
		input: String,
		/// Number of instances INCLUDING the original (2..=500).
		count: usize,
		/// A point on the rotation axis.
		center: [f64; 3],
		/// Rotation axis — any non-zero vector, normalized internally.
		axis: [f64; 3],
		/// Angular pitch between instances in degrees (default `360 / count`,
		/// a full evenly-spaced ring). Multiples of 360° are rejected — the
		/// clones would coincide.
		step_deg: Option<f64>,
	},

	// --- Measures ------------------------------------------------------------------
	/// Topological health: closed / manifold / Euler characteristic / genus / shells.
	Validate {
		#[serde(rename = "in")]
		input: String,
	},
	/// Mesh-faceted enclosed volume (signed; positive for a valid outward solid).
	Volume {
		#[serde(rename = "in")]
		input: String,
	},
	/// Analytic volume recovered from surface tags (π-exact on tagged quadrics).
	ExactVolume {
		#[serde(rename = "in")]
		input: String,
	},
	/// Volume, center of mass, and the inertia tensor at unit density: `inertia_diag`
	/// (compat) plus the full symmetric `inertia_tensor` rows `[[Ixx,Ixy,Ixz],…]` about
	/// the center of mass (mm⁵) — the products of inertia balance analysis needs.
	MassProperties {
		#[serde(rename = "in")]
		input: String,
	},
	/// Axis-aligned bounding box: overall `size` (L×W×H), `center`, space
	/// `diagonal`, and — if an `envelope` is given — whether the part fits that
	/// build volume / stock as-is (`fits_within`) and after a 90° axis turn
	/// (`fits_within_rotated`).
	BoundingBox {
		#[serde(rename = "in")]
		input: String,
		#[serde(default)]
		envelope: Option<[f64; 3]>,
	},
	/// Ray-based wall thickness; flags walls thinner than `flag_below`.
	WallThickness {
		#[serde(rename = "in")]
		input: String,
		flag_below: f64,
		/// Material dihedral angle (degrees, in (0, 180]) below which a flagged reading whose ray exits through a face that shares an edge with the sample's own face is an acute-wedge (knife-edge) reading, counted under `thin_area_wedge` instead of `thin_area`. Absent: every flagged reading counts in `thin_area`.
		#[serde(default)]
		exclude_wedge_deg: Option<f64>,
	},
	/// Draft (moldability) analysis against pull direction `pull`.
	DraftAnalysis {
		#[serde(rename = "in")]
		input: String,
		pull: [f64; 3],
		min_deg: f64,
	},
	/// Connected-body count of the tessellated mesh — the single-body oracle the
	/// other validity gates cannot give. `shells` counts B-rep shell RECORDS and
	/// can read 1 on a part severed into floating lumps (docs/FRICTION.md #24);
	/// this measure union-finds actual triangle connectivity over position-welded
	/// vertices. Returns `{ components, is_one_body, triangles }` with
	/// `provenance: "faceted"`. Gate it with `assert { components: 1 }`.
	MeshComponents {
		#[serde(rename = "in")]
		input: String,
		/// Chord tolerance (mm) of the measurement tessellation (default 0.05).
		#[serde(default = "d005")]
		tol: f64,
		/// Position-weld scale (mm) for vertex identity (default 1e-3, the house
		/// weld scale — coincident-but-unshared boolean vertices count as one point).
		#[serde(default = "dweld")]
		weld_tol: f64,
	},

	// --- Assertions ------------------------------------------------------------------
	/// Declarative checks against a bound solid (or mesh) — the program FAILS
	/// (kind `assert_failed`) when any present expectation is unmet, so intent
	/// lives in the program instead of an external grep. At least one check is
	/// required. This op is the TOPOLOGY gate; every other measure is gated with
	/// the universal `require` parameter on the op that measures it.
	Assert {
		#[serde(rename = "in")]
		input: String,
		/// Faceted (mesh) volume must land in this window.
		volume_within: Option<WithinSpec>,
		/// Analytic `exact_volume` must land in this window.
		exact_volume_within: Option<WithinSpec>,
		/// Topological genus must equal this.
		genus: Option<i64>,
		/// Shell count must equal this (e.g. 2 = two disjoint bodies after a union).
		shells: Option<usize>,
		/// Mesh connected-component count must equal this — the single-body gate
		/// (`components: 1`). Measured exactly like `mesh_components`, with the
		/// same `tol` / `weld_tol` knobs; `shells` alone cannot catch a severed part.
		components: Option<usize>,
		/// `validate().closed` must equal this.
		closed: Option<bool>,
		/// `validate().manifold` must equal this.
		manifold: Option<bool>,
		/// `validate().is_valid()` (closed + manifold + sane genus) must equal this.
		valid: Option<bool>,
		/// Chord tolerance (mm) of the `components` measurement tessellation
		/// (default 0.05) — the same knob `mesh_components` exposes.
		#[serde(default = "d005")]
		tol: f64,
		/// Position-weld scale (mm) for `components` vertex identity (default 1e-3).
		/// A severance NARROWER than this welds shut and reads as one body, so a
		/// hard severance proof needs `weld_tol` below the gap being ruled out.
		#[serde(default = "dweld")]
		weld_tol: f64,
	},
	/// Prove two solids do NOT touch: fails (kind `assert_failed`) unless the
	/// measured surface distance EXCEEDS `min_clearance` (default 0). The
	/// exit-0 proof of non-interference an empty `intersection` cannot give.
	AssertDisjoint {
		a: String,
		b: String,
		/// Required clearance (mm); the assertion passes iff distance > this.
		#[serde(default)]
		min_clearance: f64,
		/// Chord tolerance (mm) of the measurement tessellation — the distance is
		/// accurate to about this; for hard proofs keep `min_clearance` ≳ `tol`.
		#[serde(default = "dtol")]
		tol: f64,
	},
	/// Pre-scan two bound solids for the near-coincident-face hazard class before
	/// running a boolean across them. Returns `{ "coincident_fit": bool }`. A `true`
	/// means the operands share a flush/press-fit face pair (a Ø2 pin vs a Ø1.95
	/// pocket is the classic case that can grind a boolean for many minutes); the
	/// agent should then measure the fit numerically (radius difference, clearance)
	/// or shrink the tool instead of unioning across the coincident pair. Cheap
	/// (O(faces·faces) parameter comparisons, no arrangement); advisory, never
	/// mutates — a `true` flags the hazard class, it does not prove a hang.
	CoincidentFit { a: String, b: String },
	/// FDM support-necessity audit of a solid at `build_dir` (default +Z). Returns
	/// `{ support_free, bed_area, bridge_area, steep_area, total_area, max_bridge_span }`:
	/// a part prints support-free ⟺ `steep_area` (downward area steeper than `overhang_deg`
	/// from vertical, default 45°, that is neither bed contact nor a flat bridge) is ~0.
	SupportReport {
		#[serde(rename = "in")]
		input: String,
		#[serde(default = "d_up")]
		build_dir: [f64; 3],
		#[serde(default = "d_overhang")]
		overhang_deg: f64,
	},
	/// Non-asserting clearance/interference between two solids: `{ distance (min surface gap
	/// mm), interfering (true iff they interpenetrate), overlap_volume (mm³ when interfering) }`.
	/// Unlike `assert_disjoint` it never fails — it MEASURES. The overlap boolean is skipped
	/// (`overlap_volume: null`) on the coincident-fit hazard class (a press-fit), flagged by
	/// `coincident_fit_hazard`, so the query cannot trigger the coincident-fit boolean hang.
	Clearance {
		a: String,
		b: String,
		#[serde(default = "dtol")]
		tol: f64,
	},
	/// Self-describe the op surface: returns `{ ops: [name…], count }`, the authoritative
	/// catalogue of every supported op. The list is derived from the `OpKind` enum through the
	/// compile-forced [`op_tag`] match, so it can never drift from what actually runs. With `op`
	/// set, returns `{ name, exists }` for just that op (the basis of did-you-mean).
	Describe {
		#[serde(default)]
		name: Option<String>,
	},
	/// Enumerate a solid's FACES as references (M4 loop). Returns `{ count, faces: [{ index, type
	/// (plane|cylinder|sphere|cone|torus), descriptor, witness, area }] }` — `witness` is a point on
	/// the face usable to select it / its edges for fillet/chamfer; `descriptor` carries the analytic
	/// surface (plane normal+point, cylinder axis+radius, …); `area` is exact for planar faces, null
	/// for curved. Reads the existing kernel topology — no build, no geometry change.
	ListFaces {
		#[serde(rename = "in")]
		input: String,
	},
	/// Enumerate a solid's EDGES as references (M4 loop). Returns `{ count, edges: [{ index,
	/// midpoint, length, curved }] }` — `midpoint` is a witness point usable with `fillet_edge_near`
	/// / `chamfer_edge_near`; `midpoint`/`length` are the exact chord for a straight edge
	/// (`curved:false`), an approximation for a curved one (`curved:true`).
	ListEdges {
		#[serde(rename = "in")]
		input: String,
	},

	// --- Exports ---------------------------------------------------------------------
	/// Tessellate (chord tolerance `tol`) and write binary STL. Falls back to the
	/// watertight voxel heal when the exact tessellation is not watertight, and
	/// reports which route was taken.
	ExportStl {
		#[serde(rename = "in")]
		input: String,
		file: String,
		#[serde(default = "dtol")]
		tol: f64,
		/// Voxel size for the heal fallback (default 0.3 mm).
		#[serde(default = "dvoxel")]
		voxel: f64,
	},
	/// Write a STEP AP203 file with exact analytic surfaces (plane / cylinder /
	/// sphere / cone / torus).
	ExportStep {
		#[serde(rename = "in")]
		input: String,
		file: String,
	},
	/// Tessellate and write 3MF (same exact-else-heal routing as STL).
	#[serde(rename = "export_3mf")]
	Export3mf {
		#[serde(rename = "in")]
		input: String,
		file: String,
		#[serde(default = "dtol")]
		tol: f64,
		/// Voxel size for the heal fallback (default 0.3 mm).
		#[serde(default = "dvoxel")]
		voxel: f64,
	},

	// --- Implicit / hybrid -------------------------------------------------------------
	/// A gyroid TPMS lattice block (cube of half-extent `half` at `center`),
	/// meshed watertight by Manifold Dual Contouring and written as STL.
	#[cfg(feature = "catalog")]
	GyroidBlock {
		center: [f64; 3],
		half: f64,
		/// Gyroid frequency scale (cells shrink as `scale` grows; try 0.35).
		scale: f64,
		/// Shell thickness (mm).
		thickness: f64,
		/// Voxel size (mm) for the dual-contour grid.
		voxel: f64,
		file: String,
	},

	/// Measure ONE dimension of a bound solid, exactly where the analytic
	/// geometry allows (FRICTION #21 — the drawing-callout measure op). Three
	/// kinds: `point_point` (distance between two given points — provenance
	/// `coordinates`), `face_face` (perpendicular distance between two
	/// PARALLEL planar faces selected by witness points — provenance
	/// `analytic`, exact from the plane equations; non-parallel or non-planar
	/// selections fail loudly with the measured angle / face type), and
	/// `diameter` (Ø of the cylindrical or spherical face nearest `near` —
	/// provenance `analytic`, exact `2·radius`; cones/tori are refused by
	/// design because their Ø varies). The measures carry everything a drawing
	/// needs: value, provenance, the selected faces' descriptors and witness
	/// anchors — `render_sheet.py` consumes them as dimension callouts.
	MeasureDimension {
		#[serde(rename = "in")]
		input: String,
		/// `"point_point"` / `"face_face"` / `"diameter"`.
		kind: String,
		/// `point_point`: first point. `face_face`: witness selecting the first
		/// face (nearest face centroid).
		a: Option<[f64; 3]>,
		/// `point_point`: second point. `face_face`: witness selecting the
		/// second face.
		b: Option<[f64; 3]>,
		/// `diameter`: witness selecting the measured cylinder/sphere face.
		near: Option<[f64; 3]>,
	},

	/// A bounded TPMS lattice block in any of the **six families** — the named-op
	/// twin of the `implicit` tree's `tpms` leaf (same vocabulary, same
	/// validation, same `primitive_bound` field-quality wrapping), so lattices
	/// are DISCOVERABLE in the op catalogue instead of hidden inside one op.
	/// Network mode (`level` = iso-level, 0 ≈ 50% solid) or sheet mode
	/// (`level` = wall half-thickness, > 0). Meshed watertight by Manifold Dual
	/// Contouring at `voxel`; the mesh goes to `file` (`route: "voxel_implicit"`,
	/// like `gyroid_block`/`implicit` it binds no solid).
	#[cfg(feature = "catalog")]
	Tpms {
		/// Family: `gyroid` / `schwarz_p` / `diamond` / `neovius` / `schoen_iwp`
		/// / `fischer_koch_s`.
		kind: String,
		/// Lattice block corner (mm).
		min: [f64; 3],
		/// Opposite corner (mm).
		max: [f64; 3],
		/// Unit-cell edge length (mm).
		cell: f64,
		/// `"network"` (default) or `"sheet"`.
		mode: Option<String>,
		/// network: iso-level (default 0 ≈ 50% solid; negative thins). sheet:
		/// wall half-thickness in mm (> 0, required).
		level: Option<f64>,
		/// Voxel size (mm) for the extraction grid (default 0.3).
		#[serde(default = "dvoxel")]
		voxel: f64,
		/// Output mesh path — the extension picks the format (`.stl` / `.3mf`).
		file: String,
	},

	/// The **flagship hybrid op** (BAR.md Level-9 "true convergence",
	/// `kernel_model::hybrid_boolean`): boolean a bound exact B-rep solid
	/// against a non-B-rep operand — an implicit CSG tree (`field`, the same
	/// grammar as `implicit`) or a mesh FILE (`file`). The exact side **stays
	/// exact wherever the operand does not touch it**: on the exact route the
	/// untouched faces are verbatim (analytic surface tags intact) and the seam
	/// is exact against the operand's facets; when the exact route cannot be
	/// trusted the op falls back to the winding-number voxel twin and SAYS so
	/// (route `"voxel_healed"` + the measured reason). The result is a verified
	/// watertight mesh written to `out` (binds nothing); the measures carry the
	/// per-face convergence receipts (`kept_exact` / `kept_exact_curved` /
	/// `retiled` / `trimmed` / `consumed`), measured on the result — the honest
	/// proof of how much exactness survived.
	HybridBoolean {
		/// The exact B-rep operand (a bound solid).
		#[serde(rename = "in")]
		input: String,
		/// Implicit operand: a nestable CSG tree with finite bounds (exclusive
		/// with `file`; clamp an unbounded field by intersecting with a box).
		field: Option<serde_json::Value>,
		/// Mesh-file operand (`.stl`/`.obj`/`.3mf`/`.ply`; exclusive with `field`).
		file: Option<String>,
		/// Which boolean: `"union"` / `"difference"` / `"intersection"`.
		#[serde(rename = "bool")]
		bool_op: BoolOpSpec,
		/// Voxel size (mm): the field operand's meshing lattice, and the healed
		/// fallback's resampling lattice (default 0.3).
		#[serde(default = "dvoxel")]
		voxel: f64,
		/// Output mesh path — the extension picks the format (`.stl` / `.3mf`).
		out: String,
	},

	/// Mesh a nestable implicit CSG **expression tree** (the `Node` algebra as
	/// JSON, BAR.md I6): leaf shapes (sphere/box/cylinder/cone/capsule/torus/
	/// plane/gyroid/tpms/beam_lattice/voronoi_lattice/strut_lattice/pipe/
	/// pipe_path/helix_pipe/text/`expr_sdf` scalar fields) under
	/// recursive combinators (booleans, smooth/fillet/chamfer blends, `displace`
	/// surface textures, offset, shell, transforms, patterns, `offset_by`/`lerp`
	/// field modulation — with `{"grid": …}` NPY simulation fields as grade
	/// sources), extracted watertight at `voxel` resolution and optionally
	/// written to STL/3MF. See the API.md grammar section for the tree
	/// vocabulary and the `expr_sdf` Lipschitz contract.
	Implicit {
		/// The recursive expression tree (parsed with JSON-path errors).
		expr: serde_json::Value,
		/// Voxel size (mm) for the extraction grid.
		voxel: f64,
		#[serde(default = "dnarrowband")]
		mesher: MesherSpec,
		/// Explicit meshing box; default: the tree's bounds padded by 3·voxel.
		domain: Option<DomainSpec>,
		/// Optional output file — the extension picks the format (`.stl`/`.3mf`).
		file: Option<String>,
	},

	/// Hollow a bound solid into a closed wall of thickness `wall`, preserving
	/// its outer surface (the enclosure workflow). This is a **voxel-route** op
	/// by construction: the solid is lifted into a winding-number SDF, the
	/// inward-offset copy is subtracted, and the result is re-meshed watertight
	/// by Manifold Dual Contouring — accurate to the `voxel` size, NEVER exact.
	/// Like `gyroid_block`/`implicit` it binds no solid (meshes never enter the
	/// solid environment — a mesh consumer such as `mesh_carve` reads the FILE);
	/// the mesh goes to `file` and the measures report the honest route.
	Shell {
		#[serde(rename = "in")]
		input: String,
		/// Wall thickness (mm), > 0 and at least 2·`voxel` so the grid resolves it.
		wall: f64,
		/// Voxel size (mm) for the SDF re-mesh (default 0.3).
		#[serde(default = "dvoxel")]
		voxel: f64,
		/// Optional output file — the extension picks the format (`.stl`/`.3mf`).
		file: Option<String>,
	},

	/// Signed surface offset of a bound solid — grow (`delta > 0`, the Minkowski
	/// sum with a ball: convex edges gain a true `delta` round) or shrink
	/// (`delta < 0`, erosion: regions thinner than `2·|delta|` vanish) — via
	/// `kernel_model::shell::offset_to_solid`. **Voxel route by construction**
	/// (route `"voxel"`): the solid is lifted into a winding-number SDF, the
	/// shifted level set is re-extracted at `voxel`, and the result re-enters the
	/// solid environment as a **faceted** B-rep (one planar face per triangle —
	/// no analytic surfaces survive). Binds the offset solid; the measures carry
	/// the achieved volume and validity.
	OffsetSolid {
		#[serde(rename = "in")]
		input: String,
		/// Signed offset (mm): positive grows, negative shrinks.
		delta: f64,
		/// Voxel size (mm) of the re-extraction lattice (default 0.3).
		#[serde(default = "dvoxel")]
		voxel: f64,
	},
	/// Hollow a bound solid into a closed wall of `thickness` mm (outer surface
	/// preserved, cavity sealed) and re-enter the SOLID environment as a
	/// **faceted** B-rep, via `kernel_model::shell::shell_to_solid` — the
	/// solid-binding sibling of the file-writing `shell` op. **Voxel route by
	/// construction** (route `"voxel"`); the cavity arrives as a second nested
	/// shell (`shells: 2` in the measures — `shells: 1` means the wall met
	/// itself: thickness ≥ the part's inradius left no cavity).
	ShellSolid {
		#[serde(rename = "in")]
		input: String,
		/// Wall thickness (mm), > 0 and at least 2·`voxel` so the grid resolves it.
		thickness: f64,
		/// Voxel size (mm) of the re-extraction lattice (default 0.3).
		#[serde(default = "dvoxel")]
		voxel: f64,
	},
	/// **Reverse bridge v1** (`kernel_model::reverse::implicit_to_solid`): mesh an
	/// implicit `expr` tree (the same grammar as `implicit`) at `voxel` and wrap
	/// it into a validated **faceted** B-rep solid — one planar face per surviving
	/// triangle, coplanar facets coalesced, NO analytic curved-surface recovery
	/// (that is the ledgered v2). The wrap is gated on volume conservation
	/// (|solid − mesh| ≤ 1e-6 relative) — a coalesce that altered geometry is a
	/// refusal, never a quiet corruption. Binds the solid, so a field-born body
	/// can enter exact planar booleans and `export_step` — at chord fidelity
	/// `voxel`, stated honestly (route `"voxel"`).
	SolidFromImplicit {
		/// The implicit expression tree (same grammar as the `implicit` op).
		expr: serde_json::Value,
		/// Extraction voxel size (mm) — also the chord fidelity of every face.
		voxel: f64,
		/// Explicit meshing box; default: the tree's own (finite) bounds.
		domain: Option<DomainSpec>,
	},
	/// SAMPLED thin-wall census (`kernel_model::reverse::thin_wall_report`) of a
	/// bound solid (`in`, lifted through the winding-number SDF) or an implicit
	/// `expr` tree — exactly one. Reports the thinnest local wall estimate
	/// (`2·|d|` at medial samples), where it sits, and how many samples fall
	/// under `t_min`. An ESTIMATE at the lattice resolution: it can under-report
	/// by ~one cell and can miss walls thinner than the cell entirely — a
	/// warning instrument, not an oracle. Binds nothing.
	ThinWall {
		/// A bound solid id (exclusive with `expr`).
		#[serde(rename = "in")]
		input: Option<String>,
		/// An implicit expression tree (exclusive with `in`).
		expr: Option<serde_json::Value>,
		/// Walls thinner than this (mm) are counted in `below_count`.
		t_min: f64,
		/// Sampling lattice points per axis, 8..=256 (default 64; cost ~samples³).
		#[serde(default = "dsamples")]
		samples: usize,
		/// Explicit census box; default: the solid's bounding box / the tree's bounds.
		domain: Option<DomainSpec>,
	},
	/// Advisory minimum-ligament echo (`kernel_brep::holes::min_ligament`,
	/// FRICTION #21): the thinnest remaining material between a PLANNED Ø`d`
	/// through-bore at `at`+`axis` and the solid's existing boundary, measured
	/// BEFORE any cut (64 stations on one mid-span ring, exact closest-point to
	/// the default tessellation; the echo is clamped above by ~half the material
	/// span, since pierce faces are part of the boundary). Purely a measurement —
	/// nothing is cut; an unanswerable question (no material along `axis` from
	/// `at`) reports `status: "no_material"` instead of a number. Binds nothing.
	MinLigament {
		#[serde(rename = "in")]
		input: String,
		/// Planned hole center on the entry face.
		at: [f64; 3],
		/// Drilling direction, INTO the material (hole-wizard convention).
		axis: [f64; 3],
		/// Planned bore **diameter** (mm).
		d: f64,
	},

	/// ACE bridge: sample a bound SOLID (winding-number SDF) or an implicit
	/// `expr` tree into a density grid and write it as `solid_fraction.npy`
	/// in the ACE physics contract — float32 C-order, shape `(nx,ny,nz)`
	/// indexed `rho[i,j,k]` (`i↔x`), voxel centers at `origin+(idx+0.5)·h`,
	/// values = inside-fraction of `supersample`³ sub-points per voxel.
	SampleDensityGrid {
		/// A bound solid id (exclusive with `expr`).
		#[serde(rename = "in")]
		input: Option<String>,
		/// An implicit expression tree (exclusive with `in`).
		expr: Option<serde_json::Value>,
		/// Grid origin (mm) — the LOW corner of voxel (0,0,0), not its center.
		origin: [f64; 3],
		/// Isotropic voxel size h (mm).
		voxel: f64,
		/// Grid shape (nx, ny, nz).
		shape: [usize; 3],
		/// Sub-points per axis per voxel for fractional densities (default 2).
		#[serde(default = "dsupersample")]
		supersample: usize,
		/// Output `.npy` path (relative joins `--out-dir`).
		file: String,
	},

	/// ACE bridge: read an (optimized) density `.npy`, threshold at `iso`,
	/// redistance to a true level-set and mesh it WATERTIGHT through the
	/// kernel's narrow-band pipeline — the gated replacement for a raw
	/// marching-cubes `emit_stl`. Reports `volume_mm3 / num_triangles /
	/// watertight` (the ACE `render.emit_stl` contract fields).
	MeshDensityGrid {
		/// Input `.npy` (float32/float64, C-order, shape `(nx,ny,nz)`).
		npy: String,
		/// Grid origin (mm) — low corner of voxel (0,0,0).
		origin: [f64; 3],
		/// Isotropic voxel size h (mm).
		voxel: f64,
		/// Density threshold: material where `rho >= iso` (default 0.5).
		#[serde(default = "diso")]
		iso: f64,
		/// Output mesh path (`.stl`/`.3mf`).
		file: String,
	},

	// --- Native formats ------------------------------------------------------------------
	/// Load a `.lmcpart` file (the native parametric format, BAR.md I3b),
	/// evaluate its feature tree to the exact B-rep, and bind it as a solid.
	LoadPart { file: String },

	// --- Imports ---------------------------------------------------------------------------
	/// Import a STEP physical file and bind the reconstructed B-rep solid. Faces
	/// keep their exact analytic surface tags; trimmed-NURBS faces are counted in
	/// the measures (`freeform_faces`). A multi-solid file merges into ONE
	/// multi-shell solid (`shells` in the measures says how many).
	ImportStep { file: String },
	/// Import a triangle-mesh file (`.stl` / `.obj` / `.3mf` / `.ply` — sniffed
	/// by extension; the kernel has no glTF reader), weld it, and report the full
	/// `check_mesh` receipt. Binds NOTHING — meshes never enter the solid
	/// environment; `mesh_carve` consumes the mesh FILE directly. `volume` is
	/// reported ONLY when the welded mesh is watertight (a leaky mesh has no
	/// defined enclosed volume — honest omission, not a guess).
	ImportMesh {
		file: String,
		/// Repair before the receipt: cap boundary loops (`fill_holes`) and split
		/// non-manifold junctions (`make_manifold`). If the mesh is STILL leaky
		/// afterwards the op fails `invalid_geometry` (default false).
		#[serde(default)]
		heal: bool,
		/// Optional re-write of the welded/healed mesh — the extension picks the
		/// format (`.stl` / `.3mf`).
		out: Option<String>,
	},
	/// Boolean a bound solid against a mesh FILE through the winding-number voxel
	/// boolean (`kernel_implicit::mesh_boolean_implicit`): the solid is meshed on
	/// the honest exact-else-heal route, the file is welded in, both are lifted to
	/// winding-number SDFs and re-meshed. The output is GUARANTEED a closed
	/// 2-manifold, but the seam is **voxel-resampled** — accurate to `voxel`,
	/// never exact (route `"voxel_implicit"`). Writes `out`; binds nothing.
	MeshCarve {
		#[serde(rename = "in")]
		input: String,
		/// The mesh operand file (`.stl` / `.obj` / `.3mf` / `.ply`).
		file: String,
		/// Which boolean: `"union"` / `"difference"` / `"intersection"`.
		#[serde(rename = "bool")]
		bool_op: BoolOpSpec,
		/// Voxel size (mm) of the resampling lattice (default 0.3).
		#[serde(default = "dvoxel")]
		voxel: f64,
		/// Output mesh path — the extension picks the format (`.stl` / `.3mf`).
		out: String,
	},

	// --- Parts library (curated, admission-gated; BAR.md I7) ------------------------------
	/// Admit a candidate `.lmcpart` (inline or by path) into a library directory.
	/// The candidate must pass the ADMISSION GATE — rebuild closed+manifold and
	/// volume-bit-deterministically at the interface defaults, sampled range
	/// corners and midpoint — or the op fails `admission_rejected`, naming the
	/// failing sample, and nothing is admitted.
	#[cfg(feature = "catalog")]
	LibraryAdd {
		/// Library directory (relative joins `--out-dir`; created on demand).
		dir: String,
		/// The candidate part envelope INLINE (exclusive with `part_file`).
		part: Option<serde_json::Value>,
		/// Path to the candidate `.lmcpart` (exclusive with `part`).
		part_file: Option<String>,
		/// Identity, provenance (caller-supplied date) and parameter interface.
		meta: LibraryMetaSpec,
	},
	/// Search a library's curated view by free text and tags (deprecated
	/// entries are hidden).
	#[cfg(feature = "catalog")]
	LibrarySearch {
		/// Library directory.
		dir: String,
		/// Case-insensitive substring over name/category/description/tags
		/// (empty matches all).
		#[serde(default)]
		text: String,
		/// Tags the entry must all carry (case-insensitive).
		#[serde(default)]
		tags: Vec<String>,
	},
	/// Instantiate a library entry with parameter values and bind the solid.
	/// Unknown names/versions/parameters and out-of-range values fail loudly;
	/// a deprecated entry still builds but the measures carry a warning.
	#[cfg(feature = "catalog")]
	LibraryInstantiate {
		/// Library directory.
		dir: String,
		/// Entry name.
		name: String,
		/// Entry version (default: the highest admitted version).
		version: Option<u32>,
		/// Parameter values; unset parameters take their declared defaults.
		#[serde(default)]
		params: BTreeMap<String, f64>,
	},
	/// Deprecate every version of a name: hidden from `library_search`,
	/// still on disk, still instantiable (with a warning).
	#[cfg(feature = "catalog")]
	LibraryDeprecate {
		/// Library directory.
		dir: String,
		/// Entry name.
		name: String,
	},
	/// Remove every version of a name (files + index rows). Refuses with kind
	/// `dependents_exist` when `.lmcasm` files in the directory still reference
	/// it by path — unless `force` (git history keeps removals recoverable).
	#[cfg(feature = "catalog")]
	LibraryRemove {
		/// Library directory.
		dir: String,
		/// Entry name.
		name: String,
		/// Skip the dependents refusal (default false).
		#[serde(default)]
		force: bool,
	},

	// --- Standard parts catalog ------------------------------------------------------------
	/// ISO 53 involute spur gear, bored, with an optional DIN 6885 hub keyway.
	SpurGear {
		module: f64,
		teeth: usize,
		face_width: f64,
		/// Bore **diameter** (mm). `bore` is canonical; `bore_d` (the Document
		/// field name) is accepted as an alias so the two surfaces match.
		#[serde(alias = "bore_d")]
		bore: f64,
		#[serde(default = "d20")]
		pressure_angle_deg: f64,
		/// Cut the DIN 6885-1 hub keyway sized for `bore` (default false).
		#[serde(default)]
		keyway: bool,
	},
	/// ISO 4017 hex-head bolt body (M3–M16), shank length `length`.
	#[cfg(feature = "catalog")]
	HexBolt { m: f64, length: f64 },
	/// ISO 4032 hex nut (M3–M16).
	HexNut { m: f64 },
	/// ISO 7089 plain washer (M3–M16).
	Washer { m: f64 },
	/// DIN 912 / ISO 4762 socket-head cap screw body (M3–M16) with the hex
	/// drive socket cut into the head.
	SocketHeadCapScrew { m: f64, length: f64 },
	/// GT2 2 mm timing pulley: `teeth` grooves, `belt_width` band, bored.
	#[cfg(feature = "catalog")]
	#[serde(rename = "gt2_pulley")]
	Gt2Pulley {
		teeth: usize,
		belt_width: f64,
		/// Bore **diameter** (mm); `bore_d` (the Document field name) is an alias.
		#[serde(alias = "bore_d")]
		bore: f64,
		/// Add a retaining flange on each end (default false).
		#[serde(default)]
		flanged: bool,
	},
	/// ANSI/ASA B29.1 roller-chain sprocket (e.g. #25: pitch 6.35, roller 3.302).
	#[cfg(feature = "catalog")]
	ChainSprocket {
		pitch: f64,
		roller_d: f64,
		teeth: usize,
		/// Bore **diameter** (mm); `bore_d` (the Document field name) is an alias.
		#[serde(alias = "bore_d")]
		bore: f64,
	},
	/// Plain Ø`d` shaft along +Z, optionally with a DIN 6885 form-A keyway slot.
	#[cfg(feature = "catalog")]
	Shaft {
		/// Shaft **diameter** (mm).
		d: f64,
		length: f64,
		/// Optional keyway slot; its width/depth auto-size from the DIN 6885-1
		/// table for `d`.
		keyway: Option<ShaftKeywaySpec>,
	},
	/// DIN 6885 form-A parallel key bar (`b` × `h` × `l`, round ends) on z = 0.
	#[cfg(feature = "catalog")]
	ParallelKey { b: f64, h: f64, l: f64 },
	/// ISO 2338 parallel dowel pin (Ø 1–12 table), 15° chamfers both ends.
	DowelPin {
		/// Pin **diameter** (mm), an ISO 2338 table size.
		d: f64,
		length: f64,
	},
	/// DIN 471 external retaining ring for a nominal shaft Ø, installed state.
	CirclipExternal { shaft_d: f64 },
	/// DIN 472 internal retaining ring for a nominal bore Ø.
	#[cfg(feature = "catalog")]
	CirclipInternal { bore_d: f64 },
	/// ISO 10642 countersunk (flat-head) socket screw body (M3–M16).
	FlatHeadScrew { m: f64, length: f64 },
	/// ISO 7380 button-head socket screw body (M3–M12).
	ButtonHeadScrew { m: f64, length: f64 },
	/// DIN 916 cup-point set screw (M3–M12).
	SetScrew { m: f64, length: f64 },
	/// DIN 985 nyloc lock-nut body (M3–M16).
	LockNut { m: f64 },
	/// Metric threaded rod with half-pitch end chamfers (M3–M16).
	#[cfg(feature = "catalog")]
	ThreadedRod { m: f64, length: f64 },
	/// Female–female hex standoff at the conventional wrench size (M2–M6).
	#[cfg(feature = "catalog")]
	Standoff { m: f64, length: f64 },
	/// Compression spring: round wire swept on a helix, plain open ends.
	CompressionSpring {
		/// Wire **diameter** (mm).
		wire_d: f64,
		/// Coil outside **diameter** (mm).
		outer_d: f64,
		/// Axial advance per turn (mm), must exceed `wire_d`.
		pitch: f64,
		/// Active turns (may be fractional).
		turns: f64,
	},
	/// 2020 V-slot aluminium extrusion stock along +Z.
	#[cfg(feature = "catalog")]
	#[serde(rename = "extrusion_2020")]
	Extrusion2020 { length: f64 },
	/// 3030 T-slot aluminium extrusion stock along +Z.
	#[cfg(feature = "catalog")]
	#[serde(rename = "extrusion_3030")]
	Extrusion3030 { length: f64 },
	/// 2020-series M5 drop-in tee nut (no parameters), flange-down on z = 0.
	#[cfg(feature = "catalog")]
	#[serde(rename = "tnut_2020")]
	Tnut2020 {},
	/// AS568 O-ring at its free nominal size: the exact torus, axis +Z.
	#[serde(rename = "o_ring")]
	ORing {
		/// AS568 dash number (e.g. `214`).
		dash: u16,
	},
	/// Metric O-ring / cord ring at its free nominal size: the exact torus of any
	/// inside diameter with a stocked metric cord cross-section, axis +Z.
	#[serde(rename = "o_ring_cord")]
	ORingCord {
		/// Ring inside **diameter** (mm) — free, unlike the AS568 dash table.
		ring_id: f64,
		/// Cord cross-section **diameter** (mm), a stocked metric size.
		cord_d: f64,
	},
	/// One jaw-coupling hub (GR-style): body, half-height centre spigot, three 28°
	/// jaws on the 60° station grid, bored.
	#[cfg(feature = "catalog")]
	JawCouplingHub {
		/// Body outer **diameter** (a table size: 20, 25, 30, 40).
		od: f64,
		/// Bore **diameter** (mm), within the size row's range.
		bore: f64,
	},
	/// The elastomer spider (star insert) mating two `jaw_coupling_hub`s.
	#[cfg(feature = "catalog")]
	JawCouplingSpider {
		/// Body outer **diameter** (a table size: 20, 25, 30, 40).
		od: f64,
	},
	/// One-piece set-screw rigid shaft coupling, possibly stepped-bore (4 radial
	/// set-screw tap holes; threads not modelled).
	#[cfg(feature = "catalog")]
	SetScrewCoupling {
		/// Bore at z = 0 (a stocked size).
		bore1: f64,
		/// Bore at z = L (a stocked size).
		bore2: f64,
	},
	/// One-piece slit clamp coupling, possibly stepped-bore (full-length slit + two
	/// DIN 912 cross screws as counterbored clearance holes).
	#[cfg(feature = "catalog")]
	ClampCoupling {
		/// Bore at z = 0 (a stocked size).
		bore1: f64,
		/// Bore at z = L (a stocked size).
		bore2: f64,
	},
	/// Simplified NEMA stepper body: chamfered square body below z = 0, pilot
	/// boss + output shaft along +Z (frames 17 and 23).
	#[cfg(feature = "catalog")]
	NemaMotor {
		/// NEMA frame number (17 or 23).
		frame: usize,
		/// Body length below the faceplate, mm.
		body_len: f64,
	},
	/// Square NEMA mount plate: pilot register bore + the 4-bolt clearance pattern.
	#[cfg(feature = "catalog")]
	NemaMountPlate {
		/// NEMA frame number (17 or 23).
		frame: usize,
		/// Plate thickness, mm.
		thickness: f64,
		/// Extra plate width beyond the motor face, per side (≥ 0).
		margin: f64,
	},
	/// LM-series linear ball-bearing envelope (LM8UU / LM12UU): grooved tube.
	#[cfg(feature = "catalog")]
	#[serde(rename = "linear_bearing_lmuu")]
	LinearBearingLmuu {
		/// Shaft bore **diameter**: 8 (LM8UU) or 12 (LM12UU).
		bore: f64,
	},
	/// SC8UU linear-bearing pillow block envelope (Ø15 seat at height 11).
	#[cfg(feature = "catalog")]
	#[serde(rename = "sc8uu_block")]
	Sc8uuBlock {},
	/// SK8 upright shaft support for Ø8 rod (slit clamp, two base holes).
	#[cfg(feature = "catalog")]
	#[serde(rename = "shaft_support_sk8")]
	ShaftSupportSk8 {},
	/// SHF8 flange shaft support for Ø8 rod (stadium plate, slit clamp).
	#[cfg(feature = "catalog")]
	#[serde(rename = "shaft_support_shf8")]
	ShaftSupportShf8 {},
	/// HIWIN MGN12 profile-rail envelope with countersunk M3 holes on a 25 pitch.
	#[cfg(feature = "catalog")]
	#[serde(rename = "mgn12_rail")]
	Mgn12Rail {
		/// Rail length, mm (≥ one 25 mm pitch).
		length: f64,
	},
	/// MGN12H carriage envelope (45.4 × 27, rail channel, four M3 platform taps).
	#[cfg(feature = "catalog")]
	#[serde(rename = "mgn12_carriage")]
	Mgn12Carriage {},
	/// Deep-groove ball-bearing body: the seat table's d × D × B annulus.
	#[serde(rename = "deep_groove_bearing")]
	DeepGrooveBearing {
		/// Seat-table designation: "603", "608", "625", "688", "6000", "6001", "6804".
		designation: String,
	},
	/// Flanged miniature bearing body, flange face at z = 0.
	#[serde(rename = "flanged_bearing")]
	FlangedBearing {
		/// "F608" (8 × 22 × 7, flange Ø25 × 1.5) or "F623" (3 × 10 × 4, Ø11.5 × 0.6).
		designation: String,
	},
	/// Thrust ball-bearing body, 511 series.
	#[cfg(feature = "catalog")]
	#[serde(rename = "thrust_bearing")]
	ThrustBearing {
		/// "51100" (10 × 24 × 9) or "51101" (12 × 26 × 9).
		designation: String,
	},
	/// KP08 pillow block (Ø8 bore at centre height 15, base 55 × 13, holes at ±21).
	#[cfg(feature = "catalog")]
	#[serde(rename = "kp08_pillow_block")]
	Kp08PillowBlock {},
	/// G-series (ISO 228-1) pipe-thread port boss: tap-drill bore + mouth chamfer.
	#[cfg(feature = "catalog")]
	#[serde(rename = "pipe_boss_g")]
	PipeBossG {
		/// "G1/8", "G1/4", "G3/8" or "G1/2".
		designation: String,
		/// Radial wall beyond the thread major Ø, mm (≥ 1).
		wall: f64,
		/// Boss length along +Z, mm (must contain chamfer + one pitch).
		length: f64,
	},
	/// Parametric hose-barb stem (de-facto proportions; bore Ø 0.6·hose_id).
	#[cfg(feature = "catalog")]
	#[serde(rename = "hose_barb")]
	HoseBarb {
		/// Hose inner **diameter**, mm.
		hose_id: f64,
		/// Number of sawtooth teeth (≥ 1).
		barbs: usize,
	},
	/// ISO 7379 hexagon-socket shoulder screw (thread at major Ø, socket head).
	#[cfg(feature = "catalog")]
	#[serde(rename = "shoulder_bolt")]
	ShoulderBolt {
		/// Shoulder **diameter**: 6.5, 8, 10, 13 or 16 (the ISO 7379 sizes).
		shoulder_d: f64,
		/// Ground-shoulder length, mm (the ordering length).
		shoulder_len: f64,
	},
	/// DIN 127 B spring (split) lock washer: one open helical turn of the b × s section.
	#[cfg(feature = "catalog")]
	#[serde(rename = "spring_washer")]
	SpringWasher {
		/// Nominal thread size: 3, 4, 5, 6, 8, 10 or 12.
		m: f64,
	},
	/// Tr8 trapezoidal lead-screw body (DIN 103 envelope, Ø8 with entry chamfer).
	#[cfg(feature = "catalog")]
	#[serde(rename = "lead_screw_tr8")]
	LeadScrewTr8 {
		/// Screw length, mm.
		length: f64,
		/// Lead: 2 (1-start), 4 (2-start) or 8 (4-start), all pitch 2.
		lead: f64,
	},
	/// The flanged Tr8 brass-nut envelope (body Ø10.2 ×15, flange Ø22, 4 × Ø3.5).
	#[cfg(feature = "catalog")]
	#[serde(rename = "lead_screw_nut_tr8")]
	LeadScrewNutTr8 {},
	/// ISO 53 / DIN 867 basic-rack gear rack (whole teeth, pitch line y = 3·m).
	#[cfg(feature = "catalog")]
	GearRack {
		module: f64,
		length: f64,
		width: f64,
		#[serde(default = "d20")]
		pressure_angle_deg: f64,
	},
	/// Internal (ring) gear: involute tooth spaces in a rim, the exact conjugate
	/// of a `spur_gear` pinion of the same module and pressure angle.
	#[cfg(feature = "catalog")]
	InternalGear {
		module: f64,
		teeth: usize,
		face_width: f64,
		/// Rim outer **diameter** (mm), must exceed the root circle `m(z + 2.5)`.
		rim_od: f64,
		#[serde(default = "d20")]
		pressure_angle_deg: f64,
	},

	// --- Standard feature cuts ------------------------------------------------------------
	/// Heat-set insert boss + correctly undersized pocket grown on a face (M2–M6).
	HeatsetInsertBoss {
		#[serde(rename = "in")]
		input: String,
		/// Boss centre on the host face.
		at: [f64; 3],
		/// Outward face normal.
		axis: [f64; 3],
		m: f64,
	},
	/// DIN 471 circlip groove cut into a shaft, spanning `[at, at + m·axis]`.
	#[cfg(feature = "catalog")]
	CirclipGrooveExternal {
		#[serde(rename = "in")]
		input: String,
		at: [f64; 3],
		axis: [f64; 3],
		shaft_d: f64,
	},
	/// DIN 472 circlip channel cut into a bore wall.
	#[cfg(feature = "catalog")]
	CirclipGrooveInternal {
		#[serde(rename = "in")]
		input: String,
		at: [f64; 3],
		axis: [f64; 3],
		bore_d: f64,
	},
	/// AS568 / Parker static O-ring gland groove cut into a shaft.
	#[cfg(feature = "catalog")]
	#[serde(rename = "o_ring_groove")]
	ORingGroove {
		#[serde(rename = "in")]
		input: String,
		at: [f64; 3],
		axis: [f64; 3],
		dash: u16,
	},
	/// Circular **face-seal (axial)** O-ring gland: an annular channel sunk into a
	/// flat face for a metric cord, centreline Ø `gland_center_d`.
	#[serde(rename = "o_ring_face_gland")]
	ORingFaceGland {
		#[serde(rename = "in")]
		input: String,
		/// Gland centre **on the face**.
		at: [f64; 3],
		/// Outward face normal; the groove sinks along `-axis`.
		axis: [f64; 3],
		/// Channel centreline **diameter** (mm).
		gland_center_d: f64,
		/// Cord cross-section **diameter** (mm), a stocked metric size.
		cord_d: f64,
	},
	/// Racetrack **face-seal** O-ring gland for rectangular lids: a rounded-rect
	/// channel (centreline `x_len × y_len`, corner radius `corner_r`) sunk into a
	/// flat face for a metric cord.
	#[cfg(feature = "catalog")]
	#[serde(rename = "o_ring_face_gland_racetrack")]
	ORingFaceGlandRacetrack {
		#[serde(rename = "in")]
		input: String,
		/// Racetrack centre **on the face**.
		at: [f64; 3],
		/// Outward face normal; the groove sinks along `-axis`.
		axis: [f64; 3],
		/// Centreline rectangle overall length along the face-frame x axis (mm).
		x_len: f64,
		/// Centreline rectangle overall length along the face-frame y axis (mm).
		y_len: f64,
		/// Centreline corner radius (mm), at least half the groove width.
		corner_r: f64,
		/// Cord cross-section **diameter** (mm), a stocked metric size.
		cord_d: f64,
	},

	/// NEMA mount feature cut: pilot register through-bore + the 4 clearance
	/// holes, machined into any face.
	#[cfg(feature = "catalog")]
	NemaMountCut {
		#[serde(rename = "in")]
		input: String,
		/// Motor axis position on the face.
		at: [f64; 3],
		/// Outward face normal.
		axis: [f64; 3],
		/// NEMA frame number (17 or 23).
		frame: usize,
		/// Material span the holes cut through, mm.
		through: f64,
	},
	/// Hobby-servo pocket: rectangular case cutout + ear-screw pilot holes
	/// through a panel (models: "sg90", "mg996r").
	#[cfg(feature = "catalog")]
	ServoPocket {
		#[serde(rename = "in")]
		input: String,
		/// Pocket centre on the face.
		at: [f64; 3],
		/// Outward face normal.
		axis: [f64; 3],
		/// Servo model name ("sg90" or "mg996r").
		model: String,
		/// Material span the pocket cuts through, mm.
		through: f64,
	},

	/// Tr8 nut-trap feature cut: nut-body through-bore + flat flange recess +
	/// 4 × M3 clearance holes, into any face.
	#[cfg(feature = "catalog")]
	#[serde(rename = "tr8_nut_trap")]
	Tr8NutTrap {
		#[serde(rename = "in")]
		input: String,
		/// Screw axis position on the face.
		at: [f64; 3],
		/// Outward face normal.
		axis: [f64; 3],
		/// Material span the bore/holes cut through, mm (> 3.7 recess depth).
		through: f64,
	},

	/// PC4-M6 / PC4-M10 push-fit pneumatic port: flat tap-drill pocket for the
	/// fitting thread + Ø4.2 tube-pass bore through the rest of the material.
	#[cfg(feature = "catalog")]
	#[serde(rename = "pc4_port")]
	Pc4Port {
		#[serde(rename = "in")]
		input: String,
		/// Port centre on the face.
		at: [f64; 3],
		/// Outward face normal.
		axis: [f64; 3],
		/// Fitting thread: 6 (PC4-M6, Ø5 × 6 pocket) or 10 (PC4-M10, Ø9 × 7).
		m: f64,
		/// Total material depth, mm (> pocket depth).
		through: f64,
	},

	/// Printable horizontal hole: Ø`d` bore with a 45° teardrop crown along `up`.
	#[serde(rename = "teardrop_hole")]
	TeardropHole {
		#[serde(rename = "in")]
		input: String,
		/// Hole centre on the entry face.
		at: [f64; 3],
		/// Drilling direction, INTO the material (hole-wizard convention).
		axis: [f64; 3],
		/// Build (print-bed +Z) direction; must not be parallel to `axis`.
		up: [f64; 3],
		/// Bore **diameter**, mm.
		d: f64,
		/// Material span, mm.
		through: f64,
	},

	/// Board mounting pattern: the published clearance-hole positions for a
	/// Raspberry Pi / Arduino Uno / VESA 75/100 board, through all material.
	#[serde(rename = "board_mount")]
	BoardMount {
		#[serde(rename = "in")]
		input: String,
		/// Pattern datum on the face: board bottom-left corner (rpi/arduino_uno)
		/// or pattern centre (vesa75/vesa100).
		at: [f64; 3],
		/// Drilling direction, INTO the material. Face frame: +Z → world (X, Y);
		/// −Z → (X, −Y), i.e. a top-face corner-anchored pattern mirrors in y.
		axis: [f64; 3],
		/// "rpi", "arduino_uno", "vesa75" or "vesa100".
		board: String,
	},

	/// Printable counterbore: DIN 974 pocket whose clearance bore is left sealed
	/// by a `bridge`-thick sacrificial membrane (drill it out after printing).
	#[serde(rename = "bridged_counterbore")]
	BridgedCounterbore {
		#[serde(rename = "in")]
		input: String,
		/// Hole centre on the entry face.
		at: [f64; 3],
		/// Drilling direction, INTO the material (hole-wizard convention).
		axis: [f64; 3],
		/// Nominal screw size M2–M12 (DIN 974 pocket, ISO 273 medium bore).
		m: f64,
		/// Total material depth, mm (> pocket + bridge).
		through: f64,
		/// Sacrificial membrane thickness, mm (one layer height, e.g. 0.2–0.3).
		bridge: f64,
	},

	// --- Assemblies (in-program) --------------------------------------------------------------
	/// Place a bound solid as an ASSEMBLY INSTANCE at a rigid seed pose (rotate
	/// about `rotate.axis` through `rotate.center`, then translate). Instances
	/// are what mates reference (by THIS op's id) and what `asm_solve` moves —
	/// the solid itself stays bound and untouched. The first instance created is
	/// the GROUND frame (it never moves; add an `asm_mate {kind:"fixed"}` to
	/// ground others). Optional `material` (name + density g/cm³) feeds
	/// `asm_mass_properties` and the saved BOM. Binds nothing.
	AsmInstance {
		/// Id of the bound solid to place (built by any earlier solid op).
		solid: String,
		/// Display name in receipts/BOM (default: this op's id).
		name: Option<String>,
		/// Seed translation (mm).
		translate: Option<[f64; 3]>,
		/// Seed rotation (applied before the translation).
		rotate: Option<RotateSpec>,
		/// Material for mass/BOM receipts: `{name, density_g_cm3}`.
		material: Option<MaterialSpec>,
	},
	/// Place a triangle-mesh FILE (`.stl`/`.obj`/`.3mf`/`.ply`) as an assembly
	/// instance — the bridge for imported/scanned parts. The mesh is welded on
	/// load; it participates in mates, contacts and exports, measured honestly
	/// on the mesh (no exact analytic surfaces). Binds nothing.
	AsmInstanceMesh {
		/// Mesh file path (relative paths resolve against the input base).
		file: String,
		/// Display name in receipts/BOM (default: this op's id).
		name: Option<String>,
		/// Seed translation (mm).
		translate: Option<[f64; 3]>,
		/// Seed rotation (applied before the translation).
		rotate: Option<RotateSpec>,
		/// Material for mass/BOM receipts: `{name, density_g_cm3}`.
		material: Option<MaterialSpec>,
	},
	/// Declare a MATE between two instances (referenced by their `asm_instance`
	/// op ids), with the geometry given explicitly in each instance's LOCAL
	/// frame. `kind`: `coincident` (a_point↔b_point) · `distance` (+`distance`)
	/// · `parallel` (a_dir∥b_dir) · `concentric` (a_axis_point/a_axis_dir ↔ b_…)
	/// · `angle` (+`degrees`, 0–180, directional) · `axis_distance` (parallel
	/// axes at center `distance` — the gear mate) · `fixed` (grounds `a`; no
	/// `b`). Mates are DECLARATIONS — nothing moves until `asm_solve`. For mates
	/// derived from real B-rep faces use `asm_mate_axis` / `asm_mate_face`
	/// instead of hand-computing frames. Binds nothing.
	AsmMate {
		/// Mate kind (see list above).
		kind: String,
		/// First instance (an `asm_instance`/`asm_instance_mesh` op id).
		a: String,
		/// Second instance (required for every kind except `fixed`).
		b: Option<String>,
		/// Point on `a` (local mm) — coincident/distance.
		a_point: Option<[f64; 3]>,
		/// Point on `b` (local mm) — coincident/distance.
		b_point: Option<[f64; 3]>,
		/// Direction on `a` (local) — parallel/angle.
		a_dir: Option<[f64; 3]>,
		/// Direction on `b` (local) — parallel/angle.
		b_dir: Option<[f64; 3]>,
		/// Axis point on `a` (local mm) — concentric/axis_distance.
		a_axis_point: Option<[f64; 3]>,
		/// Axis direction on `a` (local) — concentric/axis_distance.
		a_axis_dir: Option<[f64; 3]>,
		/// Axis point on `b` (local mm) — concentric/axis_distance.
		b_axis_point: Option<[f64; 3]>,
		/// Axis direction on `b` (local) — concentric/axis_distance.
		b_axis_dir: Option<[f64; 3]>,
		/// Target separation (mm) — distance/axis_distance.
		distance: Option<f64>,
		/// Target angle in degrees (0–180) — angle.
		degrees: Option<f64>,
	},
	/// Concentric (or center-distance) mate DERIVED from the instances' real
	/// B-rep geometry: each witness picks the nearest cylindrical/conical/toric
	/// face on that instance's SOLID (local frame, same anchors `list_faces`
	/// reports) and its exact analytic axis becomes the mate axis — no
	/// hand-computed frames, no transcription drift. With `distance` the axes
	/// mate parallel at that center distance (gear mesh) instead of collinear.
	/// Receipts echo the derived axes. Solid-backed instances only (a mesh
	/// instance has no analytic faces — use `asm_mate` with explicit geometry).
	AsmMateAxis {
		/// First instance (op id).
		a: String,
		/// Point near the axis-carrying face on `a`'s solid (LOCAL mm).
		a_witness: [f64; 3],
		/// Second instance (op id).
		b: String,
		/// Point near the axis-carrying face on `b`'s solid (LOCAL mm).
		b_witness: [f64; 3],
		/// Optional center distance (mm): axis_distance instead of concentric.
		distance: Option<f64>,
	},
	/// Face-on-face mate DERIVED from the instances' real B-rep planes: each
	/// witness picks the nearest face; its plane (point + outward normal) forms
	/// a coincident + parallel pair seating the faces flat together, `offset`
	/// millimetres apart along the normal (0 = flush contact). TWO honesty
	/// notes: (1) the seat is CENTROID-ON-CENTROID — the faces center on each
	/// other, in-plane slide is constrained, not left free; (2) the witness
	/// picks ONE face, and a boolean can FRAGMENT a large planar face into
	/// several — the seat then centers on the picked fragment (echoed in the
	/// receipts). For a specific landing point, or slide freedom, use raw
	/// `asm_mate` coincident/parallel instead. Solid-backed instances only.
	AsmMateFace {
		/// First instance (op id).
		a: String,
		/// Point near the mating face on `a`'s solid (LOCAL mm).
		a_witness: [f64; 3],
		/// Second instance (op id).
		b: String,
		/// Point near the mating face on `b`'s solid (LOCAL mm).
		b_witness: [f64; 3],
		/// Face separation along the normal (mm, default 0 = flush).
		offset: Option<f64>,
	},
	/// SOLVE the declared mates: relaxes every non-grounded instance pose to
	/// satisfy the mate set, then reports the full honesty bundle — total +
	/// per-mate residuals, a numeric DOF report (`under_constrained (N free
	/// DOF)` vs `well_constrained`, redundant rows), and every solved pose
	/// (translation + rotation quaternion, ready to copy into `pose` ops or an
	/// `.lmcasm`). Statically broken mates (bad ids, zero directions) REFUSE
	/// before solving. FAILS (`assert_failed`) when the residual exceeds
	/// `max_residual` unless `allow_unconverged` — an unsatisfiable mate set
	/// never passes silently. Binds nothing.
	AsmSolve {
		/// Relaxation sweep budget (default 256).
		iterations: Option<usize>,
		/// Residual gate (default 1e-6).
		max_residual: Option<f64>,
		/// Report an unconverged solve as ok:true with `converged:false`
		/// instead of failing (default false — loud).
		#[serde(default)]
		allow_unconverged: bool,
	},
	/// Contact / clearance scan across ALL instances at their CURRENT poses
	/// (run `asm_solve` first for mated positions): every pair with surface
	/// distance ≤ `window` (mm, default 1.0), measured on exact adaptive
	/// tessellation for solids (chord `tol`, default 0.05) and the welded mesh
	/// for mesh instances. `touching` counts pairs at ≤ 1e-6 — the designed
	/// contact / interference class. Binds nothing.
	AsmContacts {
		/// Proximity window (mm, default 1.0).
		window: Option<f64>,
		/// Chord tolerance for exact tessellation (mm, default 0.05).
		tol: Option<f64>,
	},
	/// Overlap VOLUME (mm³) between two instances at their current poses —
	/// how much material they share where `asm_contacts` only flags that they
	/// touch. Sampled on a `voxel` grid (default 0.3) over the pose-space AABB
	/// overlap through each instance's winding-number SDF; resolution-bounded
	/// by `voxel`. Binds nothing.
	AsmInterferenceVolume {
		/// First instance (op id).
		a: String,
		/// Second instance (op id).
		b: String,
		/// Sampling cell size (mm, default 0.3).
		voxel: Option<f64>,
	},
	/// Assembly mass rollup at the current poses: per-instance volume (exact
	/// analytic for solids, `volume_source: "exact"`; mesh volume for mesh
	/// instances, `"mesh"` — honest, never conflated), mass from each
	/// instance's `material` density, total mass and volume-weighted centre of
	/// mass. Instances without a material are listed with volume only and the
	/// total is flagged `mass_complete: false`. (Full inertia tensors: use the
	/// single-solid `mass_properties` op.) Binds nothing.
	AsmMassProperties {},
	/// Export the assembly at its CURRENT poses: one merged mesh to `file`
	/// (`.stl`/`.3mf`) and, with `parts_dir`, one world-posed file per
	/// instance. Solid instances tessellate exact-else-heal with the route
	/// named per instance; mesh instances export their welded mesh verbatim
	/// (route `"mesh"`).
	AsmExport {
		/// Merged output path (`.stl` / `.3mf`).
		file: String,
		/// Optional directory for per-instance STLs.
		parts_dir: Option<String>,
		/// Chord tolerance for exact tessellation (mm, default 0.05).
		tol: Option<f64>,
		/// Voxel size for the watertight heal fallback (mm, default 0.3).
		voxel: Option<f64>,
	},
	/// Export the assembly as an AP214 STEP file (NAUO product tree, solved
	/// poses, volume-conserving) — B-rep-backed instances only; mesh instances
	/// are listed as honestly `skipped` (STEP carries no tessellation here).
	AsmExportStep {
		/// Output path (`.step`).
		file: String,
	},
	/// SAVE the in-program assembly as a `.lmcasm` file — the persistent,
	/// re-executable artifact (`kernel-api asm` / MCP `run_assembly` re-solves
	/// its mates on every load). Instances bound from `load_part` keep their
	/// `.lmcpart` path source; every other instance's geometry is exported as
	/// an STL next to the file (under `parts_dir`, default `parts/`) and
	/// referenced with the `{"mesh": …}` source. Mates are serialized verbatim;
	/// current (solved) poses become the stored seed poses.
	AsmSave {
		/// Output `.lmcasm` path.
		file: String,
		/// Assembly name in the envelope (default: the file stem).
		name: Option<String>,
		/// Directory (relative to the `.lmcasm`) for exported instance meshes.
		parts_dir: Option<String>,
	},
	/// Exact pose set of a Wolfrom/epicyclic gear train at one input angle —
	/// the kinematics→assembly bridge. Validates the tooth-count assembly
	/// conditions (loud refusal with the failing condition named), then returns
	/// per-member placements: sun/carrier/ring2 rotations (degrees about +z)
	/// and per-planet `{translation, rotation_deg}` at the orbit radius
	/// `module·(S+Pa)/2` — ready to feed `asm_instance.rotate/translate` so a
	/// posed train MESHES exactly (install phases included). Binds nothing.
	GearTrainPoses {
		/// Sun tooth count (input).
		sun_teeth: usize,
		/// Grounded ring tooth count.
		ring1_teeth: usize,
		/// First planet band tooth count.
		planet_a_teeth: usize,
		/// Second (stepped) planet band tooth count.
		planet_b_teeth: usize,
		/// Output ring tooth count.
		ring2_teeth: usize,
		/// Number of equally spaced planets.
		n_planets: usize,
		/// Gear module (mm) — scales tooth counts into radii.
		module: f64,
		/// Input (sun) angle in degrees.
		theta_deg: f64,
	},

	// --- Design-math lookups ----------------------------------------------------------------
	/// GT2 2 mm two-pulley belt sizing: exact loop length + nearest whole tooth.
	#[cfg(feature = "catalog")]
	#[serde(rename = "gt2_belt")]
	Gt2Belt { center_distance: f64, t1: usize, t2: usize },
	/// Inverse belt sizing: exact centre distance for a given belt tooth count.
	#[cfg(feature = "catalog")]
	#[serde(rename = "gt2_center_distance")]
	Gt2CenterDistance { belt_teeth: usize, t1: usize, t2: usize },
	/// ISO 286 hole-basis preferred fit resolved to limit deviations (mm).
	#[serde(rename = "iso286_fit")]
	Iso286Fit { d: f64, fit: String },
	/// Heat-set insert table lookup (Ruthex M2–M6): the pilot/pocket sizing an
	/// insert pocket needs, without growing a boss.
	HeatsetSpec { m: f64 },
	/// Metric-cord static face-seal gland lookup: groove depth/width plus the
	/// design squeeze and fill ratios for a stocked cord cross-section.
	#[serde(rename = "metric_cord_gland")]
	MetricCordGland { cord_d: f64 },
	/// Cord cut length for a racetrack (rounded-rectangle) face-seal path.
	#[serde(rename = "racetrack_cord_length")]
	RacetrackCordLength { x_len: f64, y_len: f64, corner_r: f64 },
	/// ISO 228-1 G/BSPP pipe-thread lookup: major Ø, TPI, pitch, tap drill.
	#[cfg(feature = "catalog")]
	#[serde(rename = "pipe_thread_g")]
	PipeThreadG {
		/// "G1/8", "G1/4", "G3/8" or "G1/2".
		designation: String,
	},

	// --- Hole wizard --------------------------------------------------------------------
	/// Drill a plain Ø`d` hole at `at` along `axis` (pointing into the material):
	/// blind to `depth` (118° drill point) or through `through` mm of material.
	Drill {
		#[serde(rename = "in")]
		input: String,
		at: [f64; 3],
		axis: [f64; 3],
		/// Hole **diameter** (mm).
		d: f64,
		/// Full-diameter depth of a blind hole (exclusive with `through`).
		depth: Option<f64>,
		/// Material span of a through hole (exclusive with `depth`).
		through: Option<f64>,
		/// Tool facet count (default 32).
		segments: Option<usize>,
	},
	/// ISO 273 clearance hole for an M-`m` screw, always through everything.
	ClearanceHole {
		#[serde(rename = "in")]
		input: String,
		at: [f64; 3],
		axis: [f64; 3],
		m: f64,
		#[serde(default = "dmedium")]
		fit: FitSpec,
		segments: Option<usize>,
	},
	/// ISO 273 clearance hole + DIN 974-1 counterbore that recesses a DIN 912
	/// socket-head cap screw flush.
	CounterboreHole {
		#[serde(rename = "in")]
		input: String,
		at: [f64; 3],
		axis: [f64; 3],
		m: f64,
		#[serde(default = "dmedium")]
		fit: FitSpec,
		segments: Option<usize>,
	},
	/// ISO 273 clearance hole + DIN 74-1 form F 90° countersink for a flush
	/// ISO 10642 countersunk screw (M3 and larger).
	CountersinkHole {
		#[serde(rename = "in")]
		input: String,
		at: [f64; 3],
		axis: [f64; 3],
		m: f64,
		#[serde(default = "dmedium")]
		fit: FitSpec,
		segments: Option<usize>,
	},
	/// Tap-drill pilot bore for an ISO coarse M-`m` thread (Ø = m − pitch);
	/// blind holes end in the 118° drill point. The thread itself is not modelled.
	TapDrillHole {
		#[serde(rename = "in")]
		input: String,
		at: [f64; 3],
		axis: [f64; 3],
		m: f64,
		/// Full-diameter depth of a blind pilot (exclusive with `through`).
		depth: Option<f64>,
		/// Material span of a through pilot (exclusive with `depth`).
		through: Option<f64>,
		segments: Option<usize>,
	},
	/// Repeat one hole-wizard cut (`hole`) at `n` equally spaced positions on a
	/// bolt circle of **diameter** `circle_d` centred at `center`, in the plane
	/// perpendicular to `axis`, starting `start_deg` from the deterministic
	/// in-plane reference direction.
	BoltCircle {
		#[serde(rename = "in")]
		input: String,
		center: [f64; 3],
		axis: [f64; 3],
		circle_d: f64,
		n: usize,
		#[serde(default)]
		start_deg: f64,
		hole: BoltHoleSpec,
		segments: Option<usize>,
	},
	/// Cut the seat for a standard deep-groove ball bearing (e.g. `"608"` →
	/// Ø22 × 7 pocket + Ø15 shoulder bore), nominal table dimensions.
	BearingSeat {
		#[serde(rename = "in")]
		input: String,
		at: [f64; 3],
		axis: [f64; 3],
		/// Bearing designation: 603, 608, 625, 688, 6000, 6001 or 6804.
		bearing: String,
		segments: Option<usize>,
	},

	// --- Modelled ISO threads ---------------------------------------------------------------
	/// Measures-only ISO 261/262 coarse-thread lookup for a nominal M-size:
	/// `pitch`, the ISO 68-1 fundamental height `h`, the basic minor Ø and the
	/// standard tap-drill Ø. No geometry.
	ThreadSpec { m: f64 },
	/// The external ISO 68-1 thread RIDGE as an exact, watertight B-rep solid:
	/// the basic profile swept on an exact helix along +Z through the origin
	/// (96 stations/turn), crests exactly at the major Ø, root buried P/4 below
	/// the minor Ø so it overlaps the shank it is meant to fuse with. Give
	/// either `m` (ISO coarse pitch from the table) or BOTH `major_d` and
	/// `pitch`. **The exact boolean `union(body, ridge)` self-intersects and
	/// will not stitch** — fuse through the voxel half with `export_threaded`.
	ThreadRidge {
		/// Nominal ISO size (M3–M16 coarse); exclusive with `major_d`+`pitch`.
		m: Option<f64>,
		/// Explicit crest **diameter** (mm); requires `pitch`.
		major_d: Option<f64>,
		/// Explicit thread pitch (mm); requires `major_d`.
		pitch: Option<f64>,
		/// Axial start of the ridge (default 0).
		#[serde(default)]
		z0: f64,
		/// Axial span of the ridge (`length/pitch` turns, capped at 200).
		length: f64,
	},
	/// Fuse (external) or cut (internal) an ISO thread onto a bound body and
	/// export the result through the **voxel half** — the proven hybrid route
	/// for the self-intersecting exact union. The thread axis is world +Z
	/// through the origin: place the body's shank/bore there first. External:
	/// body + ridge tessellations are merged and healed via the winding-number
	/// SDF (`volume_delta_vs_body` is asserted > 0 — a thread that adds no
	/// material fails loudly). Internal: a male-profile ridge with crests at
	/// Ø(m + 0.4) — 0.2 mm radial crest clearance — is voxel-subtracted from the
	/// bore wall; this is a **print-practical approximation**, NOT the ISO
	/// D1/D4 female form (delta asserted < 0). Result accurate to `voxel`,
	/// never exact (route `"voxel_healed"` / `"voxel_implicit"`).
	ExportThreaded {
		#[serde(rename = "in")]
		input: String,
		/// Nominal ISO size (M3–M16, coarse pitch).
		m: f64,
		/// Axial start of the threaded span (default 0).
		#[serde(default)]
		z0: f64,
		/// Axial span of the thread (`length/pitch` turns, capped at 200).
		length: f64,
		/// Cut a female thread into a bore instead of fusing a male one (default false).
		#[serde(default)]
		internal: bool,
		/// Voxel size (mm); default pitch/8. Values above pitch/6 are REFUSED —
		/// the lattice would smear the crests into a smooth band.
		voxel: Option<f64>,
		/// Output mesh path — the extension picks the format (`.stl` / `.3mf`).
		file: String,
	},
}
