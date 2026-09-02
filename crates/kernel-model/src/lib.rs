// Copyright (c) LMCAD. Licensed under the MIT License.

//! `kernel-model` — a parametric, re-evaluable feature tree on top of
//! `kernel-implicit`.
//!
//! # What this layer adds
//!
//! The implicit kernel (`kernel_implicit::Node`) is a *static* CSG tree: once
//! built it has no notion of named dimensions, no edit history, and no way to
//! re-evaluate after a parameter changes. This crate adds the missing modelling
//! state:
//!
//! - A [`Document`] holds named **parameters** (`HashMap<String, f64>`) and an
//!   ordered list of [`Feature`]s — a tiny **feature history / tree**.
//! - Feature dimensions are expressed as [`Dim`]s that reference either a
//!   literal or a parameter by name, so editing one parameter ripples through
//!   every feature that uses it.
//! - [`Document::evaluate`] rebuilds a fresh CSG [`Node`] from the *current*
//!   parameter values, and [`Document::set_param`] mutates a value so the next
//!   `evaluate` re-meshes the updated solid — i.e. **parametric update**.
//! - An [`Assembly`] of [`Instance`]s places several documents (or prebuilt
//!   nodes) at arbitrary [`Affine3A`] poses, with [`Assembly::mesh_all`] and a
//!   combined [`Assembly::bounds`] — i.e. **assemblies**.
//!
//! # Honest scope (what this is NOT)
//!
//! This gives you parametric history, assemblies, and re-meshing on top of the
//! implicit/voxel engine. The boolean **result is still a mesh**: booleans are
//! evaluated as `min`/`max` on signed distances and then sampled by Surface
//! Nets, exactly as in `kernel_implicit`. A document therefore remains fully
//! re-evaluable after a boolean (the history is replayed from scratch every
//! `evaluate`), but it does *not* produce an exact native B-rep boolean result.
//! Emitting true B-rep topology (faces / edges / vertices) from a boolean
//! remains future work for the B-rep half of the kernel.

use std::borrow::Cow;
use std::collections::{BTreeMap, HashMap, HashSet};

use kernel_core::math::{Aabb, Affine3A, DAffine3, DMat3, DVec3, Mat3, Vec3};
use kernel_core::mesh::{MassProperties, Mesh};
use kernel_core::mesher::Resolution;
use kernel_core::sdf::Sdf;
use kernel_implicit::lattice::{BeamLattice, LatticeCell, Pipe};
use kernel_implicit::manifold_dual_contour;
use kernel_implicit::ops::Node;
use kernel_implicit::primitives::{Cuboid, Cylinder, Gyroid, Sphere};
use kernel_implicit::{Tpms, TpmsKind};
use serde::{Deserialize, Serialize};

pub mod campaign;
pub mod cost;
pub mod drawing;
pub mod loads;
pub mod mechanism;
pub mod optimize;
pub mod process;
pub mod reverse;
pub mod shell;
pub mod tolerance;
pub mod constraints;
pub mod format;
pub mod hybrid;
pub mod kinematics;
pub mod library;
pub mod parts;
pub mod persist;
pub mod rate;
pub mod sketch;
pub use constraints::{Constraint, ConstraintSystem, DofReport};
pub use hybrid::{hybrid_boolean, HybridError, HybridOperand, HybridReport, HybridResult, HybridRoute, HYBRID_EXACT_MAX_OPERAND_TRIS};
pub use kinematics::{CycloidTrain, EpicyclicPoses, EpicyclicTrain, PlanetPose, StrainWaveTrain};
pub use rate::{cantilever_bending_stress, lewis_form_factor, lewis_tooth_load, thin_ring_bending_strain, Stackup};
pub use sketch::{
	Arc, Circle, ConstraintState, Segment, Sketch, SketchAnalysis, SketchConstraint, SketchError, SolveReport,
};

/// Stable identifier of a [`Feature`] within a [`Document`].
///
/// Equal to the feature's index in the document's feature list. Returned by
/// [`Document::add`] and referenced by [`Feature::Boolean`] / [`Feature::Transform`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct FeatureId(pub usize);

/// A dimension value: either a fixed literal or a reference to a named parameter.
///
/// Resolving a [`Dim::Param`] against a [`Document`] looks the value up in the
/// parameter table; a missing name resolves to `0.0` (degenerate but never panics).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Dim {
	/// A fixed value, in document units (millimetres by convention).
	Literal(f64),
	/// The current value of the named parameter.
	Param(String),
}

impl Dim {
	/// Convenience constructor for a parameter reference.
	pub fn param(name: impl Into<String>) -> Self {
		Dim::Param(name.into())
	}

	/// Resolve this dimension against a parameter table.
	///
	/// An unknown parameter name resolves to `0.0` so evaluation never panics on
	/// a partially-authored document; callers can validate names separately.
	pub fn resolve(&self, params: &HashMap<String, f64>) -> f64 {
		match self {
			Dim::Literal(v) => *v,
			Dim::Param(name) => params.get(name).copied().unwrap_or(0.0),
		}
	}
}

impl From<f64> for Dim {
	fn from(v: f64) -> Self {
		Dim::Literal(v)
	}
}

/// The boolean operator of a [`Feature::Boolean`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum BooleanOp {
	/// `a ∪ b` (`min` on signed distances).
	Union,
	/// `a − b` (`max(a, -b)`).
	Difference,
	/// `a ∩ b` (`max` on signed distances).
	Intersection,
}

/// Which hole-wizard cut a [`Feature::Hole`] performs (see [`kernel_brep::holes`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum HoleKind {
	/// A plain Ø`m_or_d` drilled bore; blind holes end in the 118° drill point
	/// ([`kernel_brep::drill`]).
	Drill,
	/// An ISO 273 clearance hole for an M-`m_or_d` screw, always cut through the
	/// part's whole extent ([`kernel_brep::clearance_hole`]).
	Clearance,
	/// Clearance hole plus the DIN 974-1 counterbore that recesses a DIN 912
	/// socket-head cap screw flush ([`kernel_brep::counterbore_hole`]).
	Counterbore,
	/// Clearance hole plus the DIN 74-1 form F 90° countersink for an ISO 10642
	/// countersunk screw ([`kernel_brep::countersink_hole`]).
	Countersink,
	/// The ISO coarse tap-drill pilot bore, Ø `m − pitch`
	/// ([`kernel_brep::tap_drill_hole`]); the thread itself is not modelled.
	Tap,
}

/// ISO 273 clearance-hole fit series of a [`Feature::Hole`] — the serializable
/// mirror of [`kernel_brep::Fit`] (`kernel-brep` stays serde-free).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum HoleFit {
	/// Series *fine* (H12) — e.g. M5 → Ø5.3.
	Close,
	/// Series *medium* (H13), the default — e.g. M5 → Ø5.5.
	Medium,
	/// Series *coarse* (H14) — e.g. M5 → Ø5.8.
	Coarse,
}

impl HoleFit {
	/// The corresponding [`kernel_brep::Fit`].
	fn to_brep(self) -> kernel_brep::Fit {
		match self {
			HoleFit::Close => kernel_brep::Fit::Close,
			HoleFit::Medium => kernel_brep::Fit::Medium,
			HoleFit::Coarse => kernel_brep::Fit::Coarse,
		}
	}
}

/// A ready-made standard part from the [`parts`] catalog, embeddable in a
/// [`Document`] as [`Feature::CatalogPart`] — so a `.lmcpart` can *hold a gear*
/// (or bolt, pulley, sprocket, …) as a first-class parametric feature instead of
/// reconstructing it through primitives. Dimension-like parameters are [`Dim`]s
/// (parameter-drivable); counts and designations are fixed data. Each variant
/// builds through the corresponding catalog function (see [`parts`]) and inherits
/// its cited standard, conventions (mm, diameters, across-flats) and honest
/// approximations.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum CatalogPart {
	/// An involute spur gear ([`parts::spur_gear`]). With `keyway` the DIN 6885-1
	/// key size is auto-selected from `bore_d` ([`parts::din6885_key_size`]); a
	/// bore outside the 6–75 mm table makes the feature fail to evaluate (loud).
	SpurGear {
		/// Module (mm per tooth per π).
		module: Dim,
		/// Tooth count.
		teeth: usize,
		/// Face width (axial thickness).
		face_width: Dim,
		/// Bore diameter.
		bore_d: Dim,
		/// Pressure angle in degrees (20 is standard).
		pressure_angle_deg: Dim,
		/// Cut the DIN 6885 hub keyway sized from `bore_d`.
		#[serde(default)]
		keyway: bool,
	},
	/// A hex-head bolt blank ([`parts::hex_bolt`]); head sizes are free — use
	/// ISO 4017 tables via [`parts::hex_bolt_iso4017`] outside the feature tree.
	HexBolt {
		/// Head width across flats.
		head_width: Dim,
		/// Head height.
		head_height: Dim,
		/// Shank (thread) diameter.
		shank_d: Dim,
		/// Shank length under the head.
		shank_len: Dim,
	},
	/// A hex nut blank ([`parts::hex_nut`]).
	HexNut {
		/// Width across flats.
		width: Dim,
		/// Nut height.
		height: Dim,
		/// Bore (nominal thread) diameter.
		bore_d: Dim,
	},
	/// A flat washer ([`parts::washer`]).
	Washer {
		/// Outer diameter.
		outer_d: Dim,
		/// Inner (hole) diameter.
		inner_d: Dim,
		/// Thickness.
		thickness: Dim,
	},
	/// A DIN 912 socket-head cap screw ([`parts::socket_head_cap_screw`]); `m`
	/// must resolve to a table size (M2–M12) or the feature fails to evaluate.
	SocketHeadCapScrew {
		/// Nominal thread size (the "5" of M5).
		m: Dim,
		/// Shank length under the head.
		length: Dim,
	},
	/// A GT2 2 mm timing pulley ([`parts::gt2_pulley`]).
	Gt2Pulley {
		/// Tooth (groove) count.
		teeth: usize,
		/// Belt width the groove section accommodates.
		belt_width: Dim,
		/// Bore diameter.
		bore_d: Dim,
		/// Add retaining flanges.
		flanged: bool,
	},
	/// An ANSI/ASA B29.1 roller-chain sprocket ([`parts::chain_sprocket`]).
	ChainSprocket {
		/// Chain pitch (12.7 for #40 / 08B).
		pitch: Dim,
		/// Roller diameter (7.92 for #40).
		roller_d: Dim,
		/// Tooth count (≥ 6).
		teeth: usize,
		/// Bore diameter.
		bore_d: Dim,
	},
	/// A plain cylindrical shaft ([`parts::shaft`], no keyway; cut slots via the
	/// function API or grooves via [`Feature::CirclipGroove`]).
	Shaft {
		/// Shaft diameter.
		d: Dim,
		/// Shaft length.
		length: Dim,
	},
	/// An AS568 O-ring at its free nominal size ([`parts::o_ring`]); the dash
	/// number is a designation, not a dimension, so it is fixed data.
	ORing {
		/// AS568 dash number (e.g. 214).
		dash: u16,
	},
	/// An ISO 2338 dowel pin ([`parts::dowel_pin`]); `d` must resolve to a table
	/// diameter or the feature fails to evaluate.
	DowelPin {
		/// Nominal pin diameter (ISO 2338 series).
		d: Dim,
		/// Pin length.
		length: Dim,
	},
	/// A straight involute gear rack ([`parts::gear_rack`]).
	GearRack {
		/// Module (must match the meshing gear).
		module: Dim,
		/// Rack length along the pitch line.
		length: Dim,
		/// Face width.
		width: Dim,
		/// Pressure angle in degrees.
		pressure_angle_deg: Dim,
	},
	/// An internal (ring) gear ([`parts::internal_gear`]).
	InternalGear {
		/// Module (must match the meshing pinion).
		module: Dim,
		/// Tooth count of the internal toothing.
		teeth: usize,
		/// Face width.
		face_width: Dim,
		/// Outer rim diameter (must clear the root circle).
		rim_od: Dim,
		/// Pressure angle in degrees.
		pressure_angle_deg: Dim,
	},
}

impl CatalogPart {
	/// Build the exact B-rep solid of this catalog part with every [`Dim`]
	/// resolved against `params`. `None` when a table-driven size resolves outside
	/// its standard's table (loud, mirroring the catalog functions) — never a
	/// silently-wrong part.
	pub fn build(&self, params: &HashMap<String, f64>) -> Option<kernel_brep::Solid> {
		let r = |d: &Dim| d.resolve(params);
		match self {
			CatalogPart::SpurGear { module, teeth, face_width, bore_d, pressure_angle_deg, keyway } => {
				let key = if *keyway { Some(parts::din6885_key_size(r(bore_d))?) } else { None };
				Some(parts::spur_gear(r(module), *teeth, r(face_width), r(bore_d), r(pressure_angle_deg), key))
			}
			CatalogPart::HexBolt { head_width, head_height, shank_d, shank_len } => {
				Some(parts::hex_bolt(r(head_width), r(head_height), r(shank_d), r(shank_len)))
			}
			CatalogPart::HexNut { width, height, bore_d } => Some(parts::hex_nut(r(width), r(height), r(bore_d))),
			CatalogPart::Washer { outer_d, inner_d, thickness } => Some(parts::washer(r(outer_d), r(inner_d), r(thickness))),
			CatalogPart::SocketHeadCapScrew { m, length } => parts::socket_head_cap_screw(r(m), r(length)),
			CatalogPart::Gt2Pulley { teeth, belt_width, bore_d, flanged } => {
				Some(parts::gt2_pulley(*teeth, r(belt_width), r(bore_d), *flanged))
			}
			CatalogPart::ChainSprocket { pitch, roller_d, teeth, bore_d } => {
				Some(parts::chain_sprocket(r(pitch), r(roller_d), *teeth, r(bore_d)))
			}
			CatalogPart::Shaft { d, length } => Some(parts::shaft(r(d), r(length), None)),
			CatalogPart::ORing { dash } => parts::o_ring(*dash),
			CatalogPart::DowelPin { d, length } => parts::dowel_pin(r(d), r(length)),
			CatalogPart::GearRack { module, length, width, pressure_angle_deg } => {
				parts::gear_rack(r(module), r(length), r(width), r(pressure_angle_deg))
			}
			CatalogPart::InternalGear { module, teeth, face_width, rim_od, pressure_angle_deg } => {
				parts::internal_gear(r(module), *teeth, r(face_width), r(rim_od), r(pressure_angle_deg))
			}
		}
	}
}

/// A declarative **linear grading law** for [`Feature::GyroidLattice`] — the
/// file-able form of a functional-grading scalar field. A Rust closure cannot be
/// persisted in a `.lmcpart`, so the grade is stored as this pure-data law and
/// compiled into the [`Node::offset_by`] field at evaluate time:
///
/// ```text
/// field(p) = offset + per_unit · (axis · p),   clamped to ±max_abs
/// ```
///
/// The lattice surface moves **outward** by the field value (negative carves
/// inward). Example: `axis = [0,0,1]`, `per_unit = −0.025`, `offset = 0.25`
/// thickens the walls at the bottom of a 20 mm part (+0.25 at z = 0) and thins
/// them at the top (−0.25 at z = 20) — a stiff-bottom / soft-top damper. `axis`
/// is used **as given** (not normalized): the rate is per unit of `axis · p`,
/// so a non-unit axis folds its length into the rate.
///
/// **Lipschitz contract** (inherited from [`Node::offset_by`], honest): the
/// graded field is only `(1 + |per_unit|·|axis|)`-Lipschitz, so keep the slope
/// small (a few % per mm). The clamp bound `max_abs` (resolved values < 0 are
/// treated as 0) also pads the reported meshing bounds, which keeps the meshed
/// domain correct whatever the law evaluates to.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LinearGrade {
	/// Grading direction (need not be unit — see above; parametric).
	pub axis: [Dim; 3],
	/// Field slope per unit of `axis · p` (parametric).
	pub per_unit: Dim,
	/// Constant field value where `axis · p = 0` (parametric).
	pub offset: Dim,
	/// Clamp bound: the law is clamped to `±max_abs` (parametric).
	pub max_abs: Dim,
}

/// Unit-cell topology of a [`Feature::BeamLatticeFill`] — the serializable
/// mirror of [`kernel_implicit::LatticeCell`] (`kernel-implicit` stays
/// serde-free). Written as `"cubic"` / `"octet"` in the saved file.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum LatticeCellKind {
	/// Struts along the 12 cube edges (bending-dominated, simple).
	Cubic,
	/// The octet truss (corner↔face-centre struts + the inner octahedron):
	/// stretch-dominated, the standard structural lattice.
	Octet,
}

impl LatticeCellKind {
	/// The corresponding [`kernel_implicit::LatticeCell`].
	fn to_implicit(self) -> LatticeCell {
		match self {
			LatticeCellKind::Cubic => LatticeCell::Cubic,
			LatticeCellKind::Octet => LatticeCell::Octet,
		}
	}
}

/// The six TPMS families of [`Feature::Tpms`], serialized in the op surface's
/// snake_case vocabulary (`"gyroid"` / `"schwarz_p"` / `"diamond"` / `"neovius"`
/// / `"schoen_iwp"` / `"fischer_koch_s"`) so a `.lmcpart` document and a
/// program's `tpms` op name the families identically — one vocabulary across
/// both surfaces.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TpmsFamily {
	Gyroid,
	SchwarzP,
	Diamond,
	Neovius,
	SchoenIwp,
	FischerKochS,
}

impl TpmsFamily {
	/// The kernel-implicit family this name serializes.
	pub fn kind(self) -> TpmsKind {
		match self {
			TpmsFamily::Gyroid => TpmsKind::Gyroid,
			TpmsFamily::SchwarzP => TpmsKind::SchwarzP,
			TpmsFamily::Diamond => TpmsKind::Diamond,
			TpmsFamily::Neovius => TpmsKind::Neovius,
			TpmsFamily::SchoenIwp => TpmsKind::SchoenIwp,
			TpmsFamily::FischerKochS => TpmsKind::FischerKochS,
		}
	}
}

/// A single step in a document's feature history.
///
/// Primitive features hold their dimensions as [`Dim`]s (literal or parameter),
/// so changing a parameter re-shapes them on the next [`Document::evaluate`].
/// [`Feature::Boolean`] and [`Feature::Transform`] combine / move earlier
/// features referenced by [`FeatureId`].
#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum Feature {
	/// An axis-aligned box centred at `center` with full side lengths `(sx, sy, sz)`.
	Box {
		/// Centre of the box.
		center: [Dim; 3],
		/// Full side lengths along x, y, z (converted to half-extents internally).
		size: [Dim; 3],
	},
	/// A sphere of `radius` centred at `center`.
	Sphere {
		/// Centre of the sphere.
		center: [Dim; 3],
		/// Radius.
		radius: Dim,
	},
	/// A capped cylinder of `radius` and `height` whose axis runs along +z,
	/// centred (along its axis) at `base_center`.
	Cylinder {
		/// Centre of the cylinder's bounding box.
		center: [Dim; 3],
		/// Radius of the cylinder.
		radius: Dim,
		/// Total height along the local +z axis.
		height: Dim,
	},
	/// A cylinder of `radius` and `height` (axis +z, base at the local origin) whose **top rim
	/// is rounded** by a `fillet`-radius curved-edge fillet — a parametric rounded boss / pin /
	/// button-top. B-rep only (built as a surface of revolution via
	/// [`kernel_brep::filleted_cylinder`]); the implicit half has no rounded-cylinder primitive,
	/// so it builds nothing there (mirror of [`Feature::ExtrudeSketch`]).
	FilletedCylinder {
		/// Radius of the cylinder.
		radius: Dim,
		/// Total height along +z.
		height: Dim,
		/// Top-rim fillet radius (clamped to fit; `0` ⇒ a sharp cylinder).
		fillet: Dim,
	},
	/// A cylinder of `radius` and `height` (axis +z, base at the local origin) whose **top rim is
	/// chamfered** by a 45° bevel of size `chamfer` — the cut-edge counterpart of
	/// [`Feature::FilletedCylinder`]. B-rep only (built via [`kernel_brep::chamfered_cylinder`]).
	ChamferedCylinder {
		/// Radius of the cylinder.
		radius: Dim,
		/// Total height along +z.
		height: Dim,
		/// Top-rim chamfer size (clamped to fit; `0` ⇒ a sharp cylinder).
		chamfer: Dim,
	},
	/// A boolean combination of two earlier features.
	Boolean {
		/// The operator.
		op: BooleanOp,
		/// Left operand.
		a: FeatureId,
		/// Right operand.
		b: FeatureId,
	},
	/// A **smooth (filleted) union** of two earlier features with blend radius
	/// `blend` — the organic-modelling counterpart of [`Feature::Boolean`]. Where a
	/// hard union leaves a sharp crease, this rounds the junction over `blend`, so
	/// chaining it across several overlapping primitives builds a metaball-style
	/// blended solid. Evaluated on the **voxel/SDF half** (`smin` on signed
	/// distances), which meshes the blend watertight. Voxel-half-only: there is no
	/// exact analytic blend, so it returns `None` on [`Document::evaluate_brep`]
	/// (the mirror of [`Feature::Shell`]).
	SmoothUnion {
		/// Left operand.
		a: FeatureId,
		/// Right operand.
		b: FeatureId,
		/// Blend radius — larger fuses the two more smoothly (parametric).
		blend: Dim,
	},
	/// A **smooth (filleted) difference** `a − b` with blend radius `blend` — carves
	/// `b` out of `a` leaving a rounded fillet instead of a sharp inner crease (an
	/// organic groove / pocket). Voxel/SDF half only (mirror of [`Feature::SmoothUnion`]);
	/// `None` on [`Document::evaluate_brep`].
	SmoothDifference {
		/// The body being carved.
		a: FeatureId,
		/// The tool removed from `a`.
		b: FeatureId,
		/// Fillet radius of the carved junction (parametric).
		blend: Dim,
	},
	/// A **smooth intersection** of two features with blend radius `blend` — the
	/// rounded common volume. Voxel/SDF half only (mirror of [`Feature::SmoothUnion`]);
	/// `None` on [`Document::evaluate_brep`].
	SmoothIntersection {
		/// Left operand.
		a: FeatureId,
		/// Right operand.
		b: FeatureId,
		/// Blend radius of the rounded junction (parametric).
		blend: Dim,
	},
	/// A **gyroid TPMS lattice** filling the box `[center ± size/2]` — the signature
	/// additive-manufacturing infill. `scale` sets the cell frequency (larger ⇒ finer
	/// cells) and `thickness` the wall half-thickness. Evaluated on the voxel/SDF half
	/// (the TPMS field intersected with its bounding box gives a bounded lattice
	/// block); intersect it with another feature ([`BooleanOp::Intersection`]) to infill
	/// an arbitrary part. Voxel-half-only: there is no B-rep for a TPMS, so it returns
	/// `None` on [`Document::evaluate_brep`]. (A TPMS shell has saddle pinches, so the
	/// lattice mesh is rich and closed but not guaranteed fully watertight.)
	Gyroid {
		/// Centre of the lattice's bounding box.
		center: [Dim; 3],
		/// Full side lengths of the bounding box along x, y, z.
		size: [Dim; 3],
		/// Cell frequency (parametric); larger ⇒ finer lattice.
		scale: Dim,
		/// Wall half-thickness (parametric).
		thickness: Dim,
	},
	/// A rigid + uniform-scale transform applied to an earlier feature.
	Transform {
		/// The feature being transformed.
		input: FeatureId,
		/// Local → world transform.
		xform: Affine3A,
	},
	/// Round a named edge of an earlier feature with a constant radius — a
	/// **name-consuming** feature. The edge is referenced by its persistent
	/// [`kernel_brep::EdgeName`] (not a transient id), so the fillet re-attaches to
	/// the corresponding edge after an upstream parameter edit. This is what makes
	/// topological naming load-bearing in the feature tree. B-rep only
	/// ([`Document::evaluate_brep`]); the implicit preview path passes the input
	/// through unrounded.
	Fillet {
		/// The feature whose edge is rounded.
		input: FeatureId,
		/// Persistent name of the edge to round (survives re-evaluation).
		#[serde(with = "persist::edge_name_serde")]
		edge: kernel_brep::EdgeName,
		/// Fillet radius.
		radius: Dim,
		/// Optional witness point. If the edge name splits into several collinear
		/// fragments after an upstream edit (e.g. a boolean cuts across it), the
		/// fragment nearest this point is filleted — so the feature survives the
		/// split instead of failing as ambiguous. `None` requires a unique edge.
		near: Option<[Dim; 3]>,
	},
	/// Chamfer (flat bevel) a named edge of an earlier feature — the sibling of
	/// [`Feature::Fillet`], same persistent-name semantics, but the cut face is a
	/// single planar bevel. B-rep only.
	Chamfer {
		/// The feature whose edge is bevelled.
		input: FeatureId,
		/// Persistent name of the edge to chamfer (survives re-evaluation).
		#[serde(with = "persist::edge_name_serde")]
		edge: kernel_brep::EdgeName,
		/// Chamfer setback.
		radius: Dim,
		/// Optional witness point to disambiguate a split edge (see [`Feature::Fillet`]).
		near: Option<[Dim; 3]>,
	},
	/// Extrude a 2D [`Sketch`] along +z by a parametric `height` into a B-rep
	/// solid — the sketch-driven front end of the modeller. The sketch's
	/// constraints are solved on every rebuild and `height` resolves from the
	/// parameter table, so editing the height parameter re-extrudes the profile.
	/// B-rep only ([`Document::evaluate_brep`]); the implicit preview path cannot
	/// represent an extruded sketch yet, so [`Document::evaluate`] skips it.
	ExtrudeSketch {
		/// The 2D profile sketch (its constraints are solved during evaluation).
		sketch: Sketch,
		/// Extrusion height along +z (parametric).
		height: Dim,
		/// Parametric dimension overrides: each `(constraint index, value)` sets a
		/// [`SketchConstraint::Distance`] target from the parameter table before the
		/// sketch is solved, so editing a [`Dim::Param`] reshapes the *profile* (e.g.
		/// a parametric width), not just the extrusion height.
		dims: Vec<(usize, Dim)>,
		/// Draft angle in radians (parametric): a nonzero value slopes the walls inward
		/// so the part releases from a mould (see [`Sketch::extrude_tapered`]). `0` is a
		/// plain prism. With a nonzero draft only the outer boundary is drafted (holes
		/// are not yet drafted).
		draft: Dim,
	},
	/// A linear pattern: `count` copies of `input`, copy `k` offset by `k · step`,
	/// fused with booleans. Keep `step` large enough that copies do not share a face
	/// plane (the boolean is not yet robust to coplanar partial-overlap faces).
	LinearPattern {
		/// The feature being repeated.
		input: FeatureId,
		/// Total number of copies (including the original); clamped to at least 1.
		count: usize,
		/// Per-copy translation (parametric).
		step: [Dim; 3],
	},
	/// Mirror `input` across the plane through `plane_point` with normal
	/// `plane_normal`, fused with the original by a boolean union. Place the original
	/// fully on one side of the plane so the two halves do not share a face plane
	/// (the boolean is not yet robust to coplanar partial-overlap faces).
	Mirror {
		/// The feature being mirrored.
		input: FeatureId,
		/// A point on the mirror plane (parametric).
		plane_point: [Dim; 3],
		/// The mirror plane normal (need not be unit; parametric).
		plane_normal: [Dim; 3],
	},
	/// A circular pattern: `count` copies of `input`, copy `k` rotated about the axis
	/// (through `axis_point`, direction `axis_dir`) by `k · angle`, fused with unions.
	/// Keep `angle` and the input's offset from the axis large enough that copies do
	/// not touch / share faces (the boolean is not yet robust to coplanar faces).
	CircularPattern {
		/// The feature being repeated.
		input: FeatureId,
		/// Total number of copies (including the original); clamped to at least 1.
		count: usize,
		/// A point on the rotation axis (parametric).
		axis_point: [Dim; 3],
		/// The rotation axis direction (need not be unit; parametric).
		axis_dir: [Dim; 3],
		/// Per-copy rotation angle in radians (parametric).
		angle: Dim,
	},
	/// Hollow the input into a thin wall of the given `thickness`, preserving its
	/// outer faces and removing material inward — the standard CAD *shell*. Computed
	/// on the **voxel/SDF half** as `input − offset(input, −thickness)`, where the
	/// inward offset and CSG difference are robust and the two nested surfaces mesh
	/// watertight. This is the mirror image of [`Feature::ExtrudeSketch`]: just as an
	/// extruded sketch is B-rep-only and is skipped on the implicit path, a shell is
	/// voxel-half-only and yields `None` on the exact B-rep path
	/// ([`Document::evaluate_brep`]) — use [`Document::evaluate`] / [`Document::mesh`]
	/// for the hollowed solid.
	Shell {
		/// The feature to hollow.
		input: FeatureId,
		/// Wall thickness: material removed inward from the outer surface (parametric).
		thickness: Dim,
	},
	/// A **hole-wizard cut** into an earlier feature: drill / ISO 273 clearance /
	/// DIN 974 counterbore / DIN 74 countersink / tap pilot, with the standard's
	/// dimension tables applied by [`kernel_brep::holes`]. Place `at` on the entry
	/// face with `axis` pointing into the material. B-rep only: the table-driven
	/// tool geometry (118° drill points, counterbores) has no implicit twin, so it
	/// returns `None` on [`Document::evaluate`] (the mirror of
	/// [`Feature::ExtrudeSketch`]) rather than previewing a part without its holes.
	Hole {
		/// The feature being drilled.
		input: FeatureId,
		/// Which wizard cut to perform.
		kind: HoleKind,
		/// [`HoleKind::Drill`]: the bore **diameter**; every other kind: the
		/// nominal metric thread size `m` (must resolve to a table size).
		m_or_d: Dim,
		/// Entry point, on the entry face (parametric).
		at: [Dim; 3],
		/// Cut direction, pointing into the material (need not be unit; parametric).
		axis: [Dim; 3],
		/// Clearance fit series ([`HoleKind::Clearance`] / [`HoleKind::Counterbore`] /
		/// [`HoleKind::Countersink`] only; `None` ⇒ [`HoleFit::Medium`]). Setting a
		/// fit on a drill/tap hole makes the feature fail to evaluate (loud).
		#[serde(default, skip_serializing_if = "Option::is_none")]
		fit: Option<HoleFit>,
		/// Blind full-diameter depth for [`HoleKind::Drill`] / [`HoleKind::Tap`]
		/// (`None` ⇒ through the part's whole extent along `axis` from `at`). The
		/// clearance/counterbore/countersink kinds are through cuts by definition;
		/// setting a depth there makes the feature fail to evaluate (loud).
		#[serde(default, skip_serializing_if = "Option::is_none")]
		depth: Option<Dim>,
	},
	/// Round the **circular rim** of an earlier feature — where a cylindrical wall
	/// meets a planar cap — with the exact rolling-ball torus of radius `radius`
	/// ([`kernel_brep::fillet_circular_rim`]; the machine-exact curved-edge fillet).
	/// `near` picks the rim circle nearest that point when several qualify. With
	/// `concave` the bore-exit-lip variant
	/// ([`kernel_brep::fillet_circular_rim_concave`]) is used instead. The honest
	/// scope of those kernels applies (convex boss rims / bore lips; `radius` must
	/// fit) — an out-of-scope rim makes the document fail to evaluate rather than
	/// return an unrounded solid. B-rep only; the implicit preview passes the input
	/// through unrounded (the mirror of [`Feature::Fillet`]).
	CircularRimFillet {
		/// The feature whose rim is rounded.
		input: FeatureId,
		/// Witness point selecting the rim circle nearest it (parametric).
		near: [Dim; 3],
		/// Fillet (torus tube) radius.
		radius: Dim,
		/// Round a concave bore exit lip instead of a convex boss rim.
		#[serde(default)]
		concave: bool,
	},
	/// Loft a closed B-rep solid through a stack of closed section loops
	/// ([`kernel_brep::loft_solid`]): all sections share one point count, are
	/// ordered along the loft and wind consistently; section points are [`Dim`]s,
	/// so a profile or a station height can be parameter-driven. B-rep only
	/// (`None` on [`Document::evaluate`], the mirror of [`Feature::ExtrudeSketch`]).
	LoftSolid {
		/// The closed section loops, in loft order (≥ 2 loops of ≥ 3 points).
		sections: Vec<Vec<[Dim; 3]>>,
	},
	/// Sweep a closed profile loop along a path into a closed B-rep solid
	/// ([`kernel_brep::sweep_solid`], rotation-minimizing frames, capped ends).
	/// Profile and path points are [`Dim`]s. B-rep only (`None` on
	/// [`Document::evaluate`]). A self-overlapping sweep (e.g. a tight helix)
	/// builds a self-intersecting solid — route it through
	/// [`Document::export_mesh`], which detects this and heals.
	SweepSolid {
		/// The closed profile loop (≥ 3 points, wound counter-clockwise about its
		/// outward normal).
		profile: Vec<[Dim; 3]>,
		/// The (open) sweep path (≥ 2 points).
		path: Vec<[Dim; 3]>,
	},
	/// Full 360° revolution of an `(r, z)` profile about the local +z axis
	/// ([`kernel_brep::revolve`]) — the lathe-part feature (pulleys, knobs,
	/// mounts). Profile points are [`Dim`]s, so a radius or a shoulder height can
	/// be parameter-driven. B-rep only (`None` on [`Document::evaluate`], the
	/// mirror of [`Feature::ExtrudeSketch`]). This closes the long-standing
	/// vocabulary gap that kept revolved parts out of `.lmcpart` documents —
	/// and therefore out of mated assemblies.
	Revolve {
		/// The `(r, z)` cross-section polyline (r ≥ 0; ≥ 3 points, open — the
		/// builder closes it).
		profile: Vec<[Dim; 2]>,
		/// Facets of the full revolution (0 ⇒ the builder default of 64).
		#[serde(default)]
		segments: usize,
	},
	/// A ready-made standard part from the [`parts`] catalog ([`CatalogPart`]) —
	/// a `.lmcpart` can hold a gear, bolt, pulley… as one parametric feature.
	/// B-rep only (`None` on [`Document::evaluate`]); a table-driven size outside
	/// its standard's table makes the feature fail to evaluate (loud).
	CatalogPart {
		/// Which standard part, with its parameters.
		part: CatalogPart,
	},
	/// Cut an AS568 / Parker static **O-ring gland groove** into a shaft-like
	/// feature ([`parts::o_ring_groove`]): an annular slot of the dash number's
	/// table dimensions, spanning `[at, at + width·axis]` with `at` on the shaft
	/// axis. B-rep only; the implicit preview passes the input through ungrooved
	/// (the mirror of [`Feature::Fillet`] — a small, documented preview gap).
	ORingGroove {
		/// The shaft/piston feature being grooved.
		input: FeatureId,
		/// Groove start point, on the shaft axis (parametric).
		at: [Dim; 3],
		/// Shaft axis direction (need not be unit; parametric).
		axis: [Dim; 3],
		/// AS568 dash number (designation, fixed data).
		dash: u16,
	},
	/// Cut a DIN 471 (external, on a shaft) or DIN 472 (internal, in a bore)
	/// **circlip groove** ([`parts::circlip_groove_external`] /
	/// [`parts::circlip_groove_internal`]) at `at` along `axis`. `d` is the
	/// nominal shaft (external) or bore (internal) diameter and must resolve to a
	/// table size. B-rep only; implicit preview passes the input through.
	CirclipGroove {
		/// The shaft / housing feature being grooved.
		input: FeatureId,
		/// Groove start point, on the axis (parametric).
		at: [Dim; 3],
		/// Axis direction (need not be unit; parametric).
		axis: [Dim; 3],
		/// Nominal shaft / bore diameter (DIN 471/472 table size).
		d: Dim,
		/// Cut the internal (bore, DIN 472) groove instead of the external one.
		#[serde(default)]
		internal: bool,
	},
	/// Grow a **heat-set insert boss** (with its correctly undersized pocket) out
	/// of a face of an earlier feature ([`parts::heatset_insert_boss`]): `at` on
	/// the host face, `axis` its outward normal, `m` the insert's thread size
	/// (M2–M6 table). B-rep only; implicit preview passes the input through.
	HeatsetBoss {
		/// The printed part gaining the boss.
		input: FeatureId,
		/// Boss base centre, on the host face (parametric).
		at: [Dim; 3],
		/// Outward face normal the boss grows along (need not be unit; parametric).
		axis: [Dim; 3],
		/// Insert nominal thread size (must resolve to the M2–M6 table).
		m: Dim,
	},
	/// A **gyroid TPMS lattice** filling the axis-aligned box spanned by the two
	/// `region` corners, with an optional **functional grade** — the corner-form,
	/// gradable sibling of [`Feature::Gyroid`] (which stays for center+size
	/// documents). `scale` sets the cell frequency (larger ⇒ finer cells) and
	/// `thickness` the wall half-thickness. With a `grade`, the declarative
	/// [`LinearGrade`] law inflates/deflates the TPMS walls **before** the region
	/// clamp (`(gyroid ▷ offset_by(grade)) ∩ region-box`), so the grading reshapes
	/// the lattice sheets, never the box boundary. Intersect with another feature
	/// ([`BooleanOp::Intersection`]) to infill an arbitrary part — the existing
	/// [`Feature::Boolean`] composes implicit operands directly. Voxel-half-only:
	/// a TPMS has no B-rep, so it returns `None` on [`Document::evaluate_brep`]
	/// (the mirror of [`Feature::Shell`]); the saddle-pinch watertightness caveat
	/// of [`Feature::Gyroid`] applies unchanged.
	GyroidLattice {
		/// The two opposite box corners `[a, b]` (any order; a hand-edited
		/// inverted region is normalized, mirroring [`Feature::Box`]'s `abs`).
		region: [[Dim; 3]; 2],
		/// Cell frequency (parametric); larger ⇒ finer lattice.
		scale: Dim,
		/// Wall half-thickness (parametric).
		thickness: Dim,
		/// Optional linear grading law; omitted from the saved file when `None`,
		/// so ungraded documents serialize exactly as before grading existed.
		#[serde(default, skip_serializing_if = "Option::is_none")]
		grade: Option<LinearGrade>,
	},
	/// A bounded TPMS lattice block in any of the **six families** — gyroid,
	/// Schwarz-P, diamond, Neovius, Schoen I-WP, Fischer-Koch S — the
	/// multi-family sibling of [`Feature::GyroidLattice`] and the Document-tree
	/// twin of the op surface's `tpms` op (one vocabulary: the same snake_case
	/// family names, the same network/sheet semantics, the same
	/// `primitive_bound` field-quality wrapping, and `cell` is the metric
	/// unit-cell edge in mm — not [`Feature::Gyroid`]'s angular frequency).
	/// Network mode (default): `level` is the iso-level (`None` ⇒ 0 ≈ 50%
	/// solid; negative thins the labyrinth). Sheet mode (`sheet: true`):
	/// `level` is the wall half-thickness in mm and **must resolve positive** —
	/// a missing/non-positive sheet level fails to evaluate (loud `None`, never
	/// a panic), as do a non-positive `cell` and a non-finite region.
	/// Voxel-half-only: a TPMS has no B-rep, so it returns `None` on
	/// [`Document::evaluate_brep`] (the mirror of [`Feature::Shell`]); mesh via
	/// [`Document::mesh`], whose Manifold Dual Contouring resolves the TPMS
	/// saddle-pinch case.
	Tpms {
		/// The two opposite box corners `[a, b]` (any order — normalized).
		region: [[Dim; 3]; 2],
		/// TPMS family.
		kind: TpmsFamily,
		/// Unit-cell edge length in mm (parametric; must resolve positive finite).
		cell: Dim,
		/// Sheet mode: `level` becomes the wall half-thickness (> 0, required).
		#[serde(default)]
		sheet: bool,
		/// Network iso-level / sheet wall half-thickness (parametric).
		#[serde(default, skip_serializing_if = "Option::is_none")]
		level: Option<Dim>,
	},
	/// A **beam-lattice fill**: the box spanned by the two `region` corners
	/// filled with whole `cell` unit cells of edge `cell_size`
	/// ([`BeamLattice::from_cells`]: filling starts at the low corner, a
	/// non-whole remainder stays empty), every strut an exact cone-capsule of
	/// the uniform `radius`. Shape it by intersecting with another feature, or
	/// fuse it onto an exact part with [`Feature::HybridFuse`]. Voxel-half-only
	/// (`None` on [`Document::evaluate_brep`], the mirror of [`Feature::Shell`]);
	/// junction-rich lattices mesh watertight via [`Document::mesh`]'s Manifold
	/// Dual Contouring. Fails to evaluate (loud `None`, never a panic) when
	/// `cell_size` / `radius` do not resolve positive and finite, the region is
	/// non-finite, or the fill exceeds [`LATTICE_FILL_MAX_CELLS`] cells.
	BeamLatticeFill {
		/// The two opposite box corners `[a, b]` (any order — normalized).
		region: [[Dim; 3]; 2],
		/// Unit-cell topology — `"cubic"` or `"octet"` in the saved file.
		cell: LatticeCellKind,
		/// Unit-cell edge length (parametric; must resolve > 0).
		cell_size: Dim,
		/// Strut radius (parametric; must resolve > 0).
		radius: Dim,
	},
	/// A **tube swept along a polyline** with per-vertex radii ([`Pipe`]) — an
	/// additive implicit body: union it for external tubing/handles, subtract it
	/// to carve a conformal cooling channel. Consecutive vertex pairs become
	/// exact cone-capsules, so the wall tapers linearly vertex-to-vertex.
	/// Voxel-half-only (`None` on [`Document::evaluate_brep`], the mirror of
	/// [`Feature::Shell`]). Fails to evaluate (loud `None`, never a panic) when
	/// the path has < 2 points, `radii` is not one-per-point, a point resolves
	/// non-finite, or a radius does not resolve positive and finite.
	PipeFeat {
		/// The polyline path (≥ 2 points; points are parametric).
		path: Vec<[Dim; 3]>,
		/// Per-vertex tube radii (one per path point; each must resolve > 0).
		radii: Vec<Dim>,
	},
	/// **One cross-representation boolean** (`brep ∪/−/∩ field`) through
	/// [`hybrid_boolean`]: the `brep` operand is built on the exact half
	/// ([`Document::evaluate_brep`] semantics), the `field` operand on the
	/// implicit half ([`Document::evaluate`] semantics, e.g. a
	/// [`Feature::BeamLatticeFill`]), and the field side is meshed at `voxel`
	/// resolution for the stitch. Route semantics, honestly:
	///
	/// - **Exact half** ([`Document::evaluate_brep`]): on the
	///   [`HybridRoute::ExactStitch`] route the stitched partial-credit
	///   [`kernel_brep::Solid`] (untouched exact faces verbatim,
	///   provenance-tagged seam band) is returned and **feeds downstream B-rep
	///   features**. On the [`HybridRoute::Healed`] route (or a [`HybridError`])
	///   this returns `None` — a mesh-only result honestly cannot chain into
	///   exact features. The route, its reason, the measured [`HybridReport`]
	///   and the verified-watertight mesh stay retrievable through
	///   [`Document::hybrid_fuse_result`] (evaluate-time recompute; nothing is
	///   cached because a [`Document`] is pure persisted data and the rebuild is
	///   deterministic, R5).
	/// - **Implicit half** ([`Document::evaluate`] / [`Document::mesh`]): the
	///   voxel twin — the exact operand is tessellated and lifted into its
	///   winding-number field ([`kernel_implicit::MeshSdf`], as
	///   [`Instance::from_mesh`]) and combined with the field by `min`/`max` —
	///   so the mesh path stays available and watertight even when the exact
	///   stitch is refused ([`Document::export_mesh`] then reports the healed
	///   route with its reason).
	///
	/// The `brep` operand must evaluate on the exact half: a voxel-only operand
	/// (e.g. a [`Feature::Shell`]) makes the fuse fail to evaluate on **both**
	/// halves (loud), exactly like any other missing operand.
	HybridFuse {
		/// The exact (B-rep) operand — kept on the boolean's left.
		brep: FeatureId,
		/// The implicit (field) operand; must have finite bounds (clamp an
		/// unbounded TPMS by intersecting with a finite feature first).
		field: FeatureId,
		/// The boolean operator (`brep ∪/−/∩ field`).
		op: BooleanOp,
		/// Voxel size in mm for everything resampled (the field operand's
		/// meshing and the healed fallback); resolving ≤ 0 auto-picks ≈ 1/96 of
		/// the relevant bounding diagonal (parametric).
		voxel: Dim,
	},
}

/// One entry of a [`Document`]'s feature history: the [`Feature`] itself plus the
/// optional human-facing metadata of the user ⇄ AI handoff (BAR.md I5) — a `label`
/// (the short name a person gives the feature, e.g. "mounting boss") and free-form
/// `notes` (design intent, tolerances, reminders).
///
/// The feature is `#[serde(flatten)]`ed, so a record without metadata serializes
/// exactly as the bare feature variant (`{"Box": {…}}` — documents saved before
/// labels existed load unchanged), and a labelled one carries the metadata next to
/// the feature in the file (`{"Box": {…}, "label": "base plate"}`) where a human
/// editing the JSON by hand expects it.
#[derive(Clone, Debug, Serialize, Deserialize)]
struct FeatureRecord {
	/// The geometric feature itself.
	#[serde(flatten)]
	feature: Feature,
	/// Human-readable name (see [`Document::set_label`]).
	#[serde(default, skip_serializing_if = "Option::is_none")]
	label: Option<String>,
	/// Free-form design notes (see [`Document::set_notes`]).
	#[serde(default, skip_serializing_if = "Option::is_none")]
	notes: Option<String>,
}

/// A parametric, re-evaluable model: named parameters plus an ordered feature list.
///
/// The last feature is the document's result unless a different root is set with
/// [`Document::set_root`]. Editing a parameter with [`Document::set_param`] and
/// calling [`Document::evaluate`] / [`Document::mesh`] again produces the updated
/// solid — there is no cached geometry, so updates are always consistent.
///
/// A document is pure data and **persists as JSON** — [`Document::save_json`] /
/// [`Document::load_json`] round-trip it bit-exactly (see [`persist`] for the
/// schema contract), so a modelling session can be resumed from a file.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Document {
	#[serde(serialize_with = "persist::sorted_params")]
	params: HashMap<String, f64>,
	features: Vec<FeatureRecord>,
	root: Option<FeatureId>,
	/// Features toggled off in the rebuild (see [`Document::set_suppressed`]).
	#[serde(serialize_with = "persist::sorted_feature_ids")]
	suppressed: HashSet<FeatureId>,
	/// Named parameter-override sets (see [`Document::add_config`]). `BTreeMap`s
	/// so saves stay byte-stable; skipped when empty, so documents without
	/// configurations serialize exactly as before they existed (and still load in
	/// older kernels).
	#[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
	configs: BTreeMap<String, BTreeMap<String, f64>>,
	/// The active configuration, if any (see [`Document::activate_config`]).
	#[serde(default, skip_serializing_if = "Option::is_none")]
	active_config: Option<String>,
}

impl Document {
	/// An empty document.
	pub fn new() -> Self {
		Self::default()
	}

	/// Set (or insert) a named parameter, returning the previous value if any.
	pub fn set_param(&mut self, name: impl Into<String>, value: f64) -> Option<f64> {
		self.params.insert(name.into(), value)
	}

	/// Get the current value of a named parameter (the **base** value, ignoring
	/// any active configuration — see [`Document::effective_param`]).
	pub fn param(&self, name: &str) -> Option<f64> {
		self.params.get(name).copied()
	}

	/// The value of `name` as evaluation sees it: the active configuration's
	/// override when one applies, the base parameter otherwise.
	pub fn effective_param(&self, name: &str) -> Option<f64> {
		self.active_overrides().and_then(|o| o.get(name).copied()).or_else(|| self.param(name))
	}

	/// All base parameters as `(name, value)` pairs (unordered; collect into a
	/// `BTreeMap` for a sorted view) — the introspection behind parameter
	/// summaries such as a BOM line.
	pub fn params_iter(&self) -> impl Iterator<Item = (&str, f64)> {
		self.params.iter().map(|(k, &v)| (k.as_str(), v))
	}

	/// Add (or replace) a named **configuration**: a set of parameter overrides
	/// that, while the configuration is active, win over the base parameter table
	/// during evaluation — the standard "one model, several variants" mechanism
	/// (a light and a heavy bracket in one `.lmcpart`). Returns the previous
	/// override set under that name, if any. Configurations persist in the saved
	/// document (sorted, byte-stable) and are inert until activated.
	pub fn add_config(&mut self, name: impl Into<String>, overrides: impl IntoIterator<Item = (String, f64)>) -> Option<BTreeMap<String, f64>> {
		self.configs.insert(name.into(), overrides.into_iter().collect())
	}

	/// The override set of configuration `name`, if it exists.
	pub fn config(&self, name: &str) -> Option<&BTreeMap<String, f64>> {
		self.configs.get(name)
	}

	/// All configuration names, sorted.
	pub fn config_names(&self) -> impl Iterator<Item = &str> {
		self.configs.keys().map(String::as_str)
	}

	/// Activate configuration `name`: subsequent evaluations resolve parameters
	/// through its overrides ([`Document::set_param`] keeps editing the base
	/// table, which an override shadows until deactivation). Returns `false` —
	/// and changes nothing — when no such configuration exists, so a typo cannot
	/// silently evaluate the base variant.
	pub fn activate_config(&mut self, name: &str) -> bool {
		if self.configs.contains_key(name) {
			self.active_config = Some(name.to_string());
			true
		} else {
			false
		}
	}

	/// Deactivate any active configuration (back to the base parameter table).
	pub fn deactivate_config(&mut self) {
		self.active_config = None;
	}

	/// The active configuration's name, if one is active.
	pub fn active_config(&self) -> Option<&str> {
		self.active_config.as_deref()
	}

	/// The active configuration's overrides, if an active name resolves. A
	/// hand-edited `active_config` naming a missing configuration resolves to no
	/// overrides (the base variant) — the name is kept so saving preserves it.
	fn active_overrides(&self) -> Option<&BTreeMap<String, f64>> {
		self.active_config.as_deref().and_then(|n| self.configs.get(n))
	}

	/// The parameter table evaluation resolves against: the base table, with the
	/// active configuration's overrides applied on top (borrowed when no
	/// configuration is active, so the common path stays allocation-free).
	fn effective_params(&self) -> Cow<'_, HashMap<String, f64>> {
		match self.active_overrides() {
			None => Cow::Borrowed(&self.params),
			Some(overrides) => {
				let mut merged = self.params.clone();
				for (k, v) in overrides {
					merged.insert(k.clone(), *v);
				}
				Cow::Owned(merged)
			}
		}
	}

	/// Append a feature, returning its [`FeatureId`].
	///
	/// The newly added feature becomes the document root unless one was pinned
	/// with [`Document::set_root`].
	pub fn add(&mut self, feature: Feature) -> FeatureId {
		let id = FeatureId(self.features.len());
		self.features.push(FeatureRecord { feature, label: None, notes: None });
		id
	}

	/// Insert a feature at `index` (clamped to the history's length), shifting
	/// the features at and after that position one step later — the parametric
	/// "drag a feature earlier into the history" edit. Every [`FeatureId`]
	/// reference in later features, the pinned root, and the suppression set are
	/// remapped, so the document rebuilds exactly as before with the new feature
	/// available at `index` (labels and notes travel with their features).
	/// Returns the new feature's id, `FeatureId(index)`. The inserted feature may
	/// reference only features before `index` (ids are history positions).
	pub fn insert_feature_at(&mut self, index: usize, feature: Feature) -> FeatureId {
		let index = index.min(self.features.len());
		let shift = |id: FeatureId| if id.0 >= index { FeatureId(id.0 + 1) } else { id };
		for record in self.features.iter_mut().skip(index) {
			remap_feature_refs(&mut record.feature, shift);
		}
		self.features.insert(index, FeatureRecord { feature, label: None, notes: None });
		if let Some(root) = self.root.as_mut() {
			*root = shift(*root);
		}
		self.suppressed = std::mem::take(&mut self.suppressed).into_iter().map(shift).collect();
		FeatureId(index)
	}

	/// Pin a specific feature as the document's result.
	pub fn set_root(&mut self, id: FeatureId) {
		self.root = Some(id);
	}

	/// Suppress or un-suppress a feature — the standard parametric-edit toggle that
	/// switches a feature off in the rebuild without deleting it (so an AI can compare
	/// design variants). A suppressed **modifier** feature (one with a single upstream
	/// input: fillet, chamfer, shell, transform, linear/circular pattern, mirror) is
	/// replaced by that input on the next [`Document::evaluate`] / [`evaluate_brep`].
	/// Suppress is a no-op for **generative** features (primitives, booleans, smooth
	/// booleans, sketches, lattices), which have no single input to fall back to.
	pub fn set_suppressed(&mut self, id: FeatureId, suppressed: bool) {
		if suppressed {
			self.suppressed.insert(id);
		} else {
			self.suppressed.remove(&id);
		}
	}

	/// Whether `id` is currently suppressed.
	pub fn is_suppressed(&self, id: FeatureId) -> bool {
		self.suppressed.contains(&id)
	}

	/// Set (or replace) the human-readable **label** of feature `id` — the short
	/// name a person gives a feature ("mounting boss", "M5 bore"), persisted in
	/// the saved JSON next to the feature so a hand-editing user and an AI session
	/// share the same vocabulary (BAR.md I5). Purely descriptive: labels never
	/// affect evaluation. No-op when `id` names no feature; returns the previous
	/// label, if any.
	pub fn set_label(&mut self, id: FeatureId, label: impl Into<String>) -> Option<String> {
		self.features.get_mut(id.0).and_then(|record| record.label.replace(label.into()))
	}

	/// The label of feature `id`, if one was set (see [`Document::set_label`]).
	pub fn label(&self, id: FeatureId) -> Option<&str> {
		self.features.get(id.0).and_then(|record| record.label.as_deref())
	}

	/// Set (or replace) the free-form **notes** of feature `id` — design intent,
	/// tolerances, reminders; the long-form sibling of [`Document::set_label`]
	/// with the same persistence and no-effect-on-evaluation semantics. No-op when
	/// `id` names no feature; returns the previous notes, if any.
	pub fn set_notes(&mut self, id: FeatureId, notes: impl Into<String>) -> Option<String> {
		self.features.get_mut(id.0).and_then(|record| record.notes.replace(notes.into()))
	}

	/// The notes of feature `id`, if any were set (see [`Document::set_notes`]).
	pub fn notes(&self, id: FeatureId) -> Option<&str> {
		self.features.get(id.0).and_then(|record| record.notes.as_deref())
	}

	/// The feature currently acting as the result, if the document is non-empty.
	pub fn root(&self) -> Option<FeatureId> {
		self.root.or_else(|| {
			if self.features.is_empty() {
				None
			} else {
				Some(FeatureId(self.features.len() - 1))
			}
		})
	}

	/// Rebuild the CSG [`Node`] from the *current* parameter values.
	///
	/// Returns `None` for an empty document or one whose root references a
	/// missing / cyclic feature. The tree is built fresh every call, so it always
	/// reflects the latest [`Document::set_param`] edits.
	pub fn evaluate(&self) -> Option<Node> {
		self.evaluate_to(self.root()?)
	}

	/// **Rollback** evaluation: rebuild the CSG [`Node`] as the model stood at
	/// feature `id` — the prefix of the history up to and including it (the
	/// feature-tree rollback bar every parametric modeller has). Suppression and
	/// the active configuration apply as usual; the pinned root is ignored.
	/// `None` for an unknown id or a prefix with no implicit form.
	pub fn evaluate_to(&self, id: FeatureId) -> Option<Node> {
		if id.0 >= self.features.len() {
			return None;
		}
		// A feature DAG with shared sub-features expands into a tree, so a diamond
		// chain would re-evaluate exponentially. The SDF `Node` cannot share subtrees
		// (its leaves are boxed), so we cap the total expansion to a generous multiple
		// of the feature count and bail rather than hang. (A truly shared
		// representation would need `Arc`-backed nodes — tracked as follow-up.)
		let mut budget = self.features.len().saturating_mul(64).max(1024);
		let params = self.effective_params();
		self.build(id, &mut Vec::new(), &mut budget, &params)
	}

	/// Recursively build the node for `id`, using `stack` to reject cycles,
	/// `budget` to bound DAG expansion, and `params` (the configuration-resolved
	/// table) for every [`Dim`].
	fn build(&self, id: FeatureId, stack: &mut Vec<FeatureId>, budget: &mut usize, params: &HashMap<String, f64>) -> Option<Node> {
		if *budget == 0 {
			return None; // expansion budget exhausted (pathological shared-feature DAG)
		}
		*budget -= 1;
		let feature = &self.features.get(id.0)?.feature;
		if stack.contains(&id) {
			return None; // cyclic reference: bail rather than recurse forever
		}
		stack.push(id);
		// A suppressed modifier feature is replaced by its upstream input.
		if self.suppressed.contains(&id) {
			if let Some(inp) = primary_input(feature) {
				let node = self.build(inp, stack, budget, params);
				stack.pop();
				return node;
			}
		}
		let node = match feature {
			Feature::Box { center, size } => {
				let c = resolve_vec3(params,center);
				let half = resolve_vec3(params,size) * 0.5;
				// Guard against negative/zero dimensions producing an inverted box.
				let half = half.abs();
				Some(Node::primitive(Cuboid::new(c, half)))
			}
			Feature::Sphere { center, radius } => {
				let c = resolve_vec3(params,center);
				let r = radius.resolve(params).max(0.0) as f32;
				Some(Node::primitive(Sphere::new(c, r)))
			}
			Feature::Cylinder { center, radius, height } => {
				let c = resolve_vec3(params,center);
				let r = radius.resolve(params).max(0.0) as f32;
				let h = (height.resolve(params).max(0.0) as f32) * 0.5;
				let a = c - Vec3::new(0.0, 0.0, h);
				let b = c + Vec3::new(0.0, 0.0, h);
				Some(Node::primitive(Cylinder::new(a, b, r)))
			}
			Feature::Boolean { op, a, b } => {
				let na = self.build(*a, stack, budget, params)?;
				let nb = self.build(*b, stack, budget, params)?;
				Some(match op {
					BooleanOp::Union => na.union(nb),
					BooleanOp::Difference => na.difference(nb),
					BooleanOp::Intersection => na.intersection(nb),
				})
			}
			Feature::Gyroid { center, size, scale, thickness } => {
				let c = resolve_vec3(params,center);
				let half = (resolve_vec3(params,size) * 0.5).abs();
				let sc = scale.resolve(params).max(0.0) as f32;
				let th = thickness.resolve(params).max(0.0) as f32;
				let region = Aabb::from_center_half_extent(c, half);
				// The TPMS field is bounded by intersecting it with its box, giving a
				// lattice block; intersect that with a part for true infill.
				let lattice = Node::primitive(Gyroid::new(region, sc, th));
				Some(lattice.intersection(Node::primitive(Cuboid::new(c, half))))
			}
			Feature::SmoothUnion { a, b, blend } => {
				let na = self.build(*a, stack, budget, params)?;
				let nb = self.build(*b, stack, budget, params)?;
				let k = blend.resolve(params).max(0.0) as f32;
				Some(na.smooth_union(nb, k))
			}
			Feature::SmoothDifference { a, b, blend } => {
				let na = self.build(*a, stack, budget, params)?;
				let nb = self.build(*b, stack, budget, params)?;
				let k = blend.resolve(params).max(0.0) as f32;
				Some(na.smooth_difference(nb, k))
			}
			Feature::SmoothIntersection { a, b, blend } => {
				let na = self.build(*a, stack, budget, params)?;
				let nb = self.build(*b, stack, budget, params)?;
				let k = blend.resolve(params).max(0.0) as f32;
				Some(na.smooth_intersection(nb, k))
			}
			Feature::Transform { input, xform } => {
				let n = self.build(*input, stack, budget, params)?;
				Some(n.transform(*xform))
			}
			// The implicit/voxel preview has no edge topology to name, so it cannot
			// apply a B-rep edge fillet/chamfer — it returns the input unmodified. The
			// exact result is produced by `evaluate_brep`.
			Feature::Fillet { input, .. } | Feature::Chamfer { input, .. } => self.build(*input, stack, budget, params),
			// An extruded sketch is a B-rep-only feature: the implicit preview has no
			// 2D-profile primitive to represent it, so it does not appear on this path.
			Feature::ExtrudeSketch { .. } => None,
			Feature::FilletedCylinder { .. } | Feature::ChamferedCylinder { .. } => None, // B-rep-only
			Feature::LinearPattern { input, count, step } => {
				let s = resolve_vec3(params,step);
				let mut acc = self.build(*input, stack, budget, params)?;
				for k in 1..(*count).max(1) {
					let copy = self.build(*input, stack, budget, params)?.transform(Affine3A::from_translation(s * k as f32));
					acc = acc.union(copy);
				}
				Some(acc)
			}
			Feature::Mirror { input, plane_point, plane_normal } => {
				let base = self.build(*input, stack, budget, params)?;
				let copy = self.build(*input, stack, budget, params)?.transform(reflection_affine(resolve_vec3(params,plane_point), resolve_vec3(params,plane_normal)));
				Some(base.union(copy))
			}
			Feature::CircularPattern { input, count, axis_point, axis_dir, angle } => {
				let p = resolve_vec3(params,axis_point);
				let axis = resolve_vec3(params,axis_dir).normalize_or_zero();
				let step = angle.resolve(params) as f32;
				let mut acc = self.build(*input, stack, budget, params)?;
				if axis.length_squared() >= 0.5 {
					for k in 1..(*count).max(1) {
						let rot = Affine3A::from_translation(p) * Affine3A::from_axis_angle(axis, step * k as f32) * Affine3A::from_translation(-p);
						acc = acc.union(self.build(*input, stack, budget, params)?.transform(rot));
					}
				}
				Some(acc)
			}
			Feature::Shell { input, thickness } => {
				// Voxel-half shell: keep the outer surface and subtract an inward-offset
				// copy so a wall of `thickness` remains, outer dimensions preserved. Built
				// twice (like Mirror) since the SDF tree cannot share boxed subnodes.
				let w = thickness.resolve(params).max(0.0) as f32;
				let outer = self.build(*input, stack, budget, params)?;
				let inner = self.build(*input, stack, budget, params)?.offset(-w);
				Some(outer.difference(inner))
			}
			Feature::GyroidLattice { region, scale, thickness, grade } => {
				let (c, half) = resolve_region(params, region);
				let sc = scale.resolve(params).max(0.0) as f32;
				let th = thickness.resolve(params).max(0.0) as f32;
				let lattice = Node::primitive(Gyroid::new(Aabb::from_center_half_extent(c, half), sc, th));
				// The grade inflates the TPMS walls BEFORE the region clamp, so the box
				// boundary stays put while the sheets thicken/thin along the law. The
				// closure captures the RESOLVED constants — a parameter edit re-resolves
				// them on the next evaluate, and the same document always compiles the
				// same field (deterministic, R5).
				let graded = match grade {
					None => lattice,
					Some(g) => {
						let axis = resolve_vec3(params, &g.axis);
						let rate = g.per_unit.resolve(params) as f32;
						let offset = g.offset.resolve(params) as f32;
						let max_abs = g.max_abs.resolve(params).max(0.0) as f32;
						lattice.offset_by(std::sync::Arc::new(move |p: Vec3| offset + rate * axis.dot(p)), max_abs)
					}
				};
				Some(graded.intersection(Node::primitive(Cuboid::new(c, half))))
			}
			Feature::Tpms { region, kind, cell, sheet, level } => {
				let (c, half) = resolve_region(params, region);
				let region_box = Aabb::from_center_half_extent(c, half);
				let cell_mm = cell.resolve(params);
				let lv = level.as_ref().map(|d| d.resolve(params)).unwrap_or(0.0);
				// Fail-loud guards (None, never a panic): positive finite cell, finite
				// region/level, and a positive wall half-thickness in sheet mode.
				let inputs_sound = cell_mm > 0.0
					&& cell_mm.is_finite() && c.is_finite() && half.is_finite() && lv.is_finite()
					&& (!*sheet || lv > 0.0);
				if !inputs_sound {
					None
				} else {
					let field = if *sheet {
						Tpms::sheet(region_box, kind.kind(), cell_mm as f32, lv as f32)
					} else {
						Tpms::network(region_box, kind.kind(), cell_mm as f32, lv as f32)
					};
					// A raw TPMS is an OPEN labyrinth (the region box cuts its tubes) —
					// clamp with the region so the block is a closed solid, exactly like
					// `Feature::Gyroid` / `Feature::GyroidLattice`.
					Some(Node::primitive_bound(field).intersection(Node::primitive(Cuboid::new(c, half))))
				}
			}
			Feature::BeamLatticeFill { region, cell, cell_size, radius } => {
				let (c, half) = resolve_region(params, region);
				let cs = cell_size.resolve(params);
				let r = radius.resolve(params);
				// Fail-loud guards (None, never a panicking `from_cells`): positive
				// finite strut dimensions, a finite region and a bounded cell count.
				if !(cs > 0.0 && cs.is_finite() && r > 0.0 && r.is_finite() && c.is_finite() && half.is_finite()) {
					None
				} else {
					let region_box = Aabb::from_center_half_extent(c, half);
					// Same per-axis count as `from_cells`: floor(size/cell), at least 1.
					let n = |s: f32| ((s as f64 / cs).floor() as usize).max(1);
					let size = region_box.size();
					let cells = n(size.x).saturating_mul(n(size.y)).saturating_mul(n(size.z));
					(cells <= LATTICE_FILL_MAX_CELLS)
						.then(|| Node::primitive(BeamLattice::from_cells(region_box, cell.to_implicit(), cs as f32, r as f32)))
				}
			}
			Feature::PipeFeat { path, radii } => {
				// Fail-loud guards mirroring `Pipe::new`'s asserted contract.
				if path.len() < 2 || path.len() != radii.len() {
					None
				} else {
					let pts: Vec<Vec3> = path.iter().map(|p| resolve_vec3(params, p)).collect();
					let rs: Vec<f32> = radii.iter().map(|r| r.resolve(params) as f32).collect();
					(pts.iter().all(|p| p.is_finite()) && rs.iter().all(|r| *r > 0.0 && r.is_finite()))
						.then(|| Node::primitive(Pipe::new(pts, rs)))
				}
			}
			Feature::HybridFuse { brep, field, op, .. } => {
				// The voxel twin of the fuse (the exact result lives on `build_brep`):
				// the exact operand is built, tessellated, and lifted into its
				// winding-number field (`MeshSdf`, the `Instance::from_mesh` move), then
				// combined with the field operand by min/max — the same construction as
				// the hybrid's healed route, so this path stays meshable and watertight
				// even when the exact stitch is refused. The shared stack/budget reject
				// a hand-edited cyclic fuse across both halves.
				let solid = self.build_brep(*brep, stack, budget, params)?;
				let node = self.build(*field, stack, budget, params)?;
				let lifted = Node::primitive(kernel_implicit::MeshSdf::new(&kernel_brep::tessellate_default(&solid)));
				Some(match op {
					BooleanOp::Union => lifted.union(node),
					BooleanOp::Difference => lifted.difference(node),
					BooleanOp::Intersection => lifted.intersection(node),
				})
			}
			// Hole-wizard cuts, lofts/sweeps and catalog parts are B-rep-only: their
			// table-driven tool geometry / skinned topology has no implicit twin, so
			// they are absent on this path (the mirror of `Feature::ExtrudeSketch`) —
			// a preview must not silently show a part without its holes.
			Feature::Hole { .. } | Feature::LoftSolid { .. } | Feature::SweepSolid { .. } | Feature::Revolve { .. } | Feature::CatalogPart { .. } => None,
			// Rim fillets and the standard grooves / insert bosses are small local
			// B-rep modifications; like `Feature::Fillet`, the implicit preview passes
			// the input through unmodified and `evaluate_brep` carries the exact result.
			Feature::CircularRimFillet { input, .. }
			| Feature::ORingGroove { input, .. }
			| Feature::CirclipGroove { input, .. }
			| Feature::HeatsetBoss { input, .. } => self.build(*input, stack, budget, params),
		};
		stack.pop();
		node
	}

	/// Build the document as an **exact B-rep** [`kernel_brep::Solid`] rather than an
	/// implicit field — mirrors [`Document::evaluate`] but uses the B-rep primitives
	/// and booleans, so the result carries persistent face provenance
	/// ([`kernel_brep::FaceName`]). An agent can therefore select a result face by a
	/// name that survives a parameter edit (`face_name` / `faces_named`), the
	/// foundation of parametric feature references. `None` for an empty/cyclic document.
	///
	/// Curved primitives are faceted (cylinder/sphere use a fixed segment count) since
	/// the B-rep boolean operates on planar faces.
	pub fn evaluate_brep(&self) -> Option<kernel_brep::Solid> {
		self.evaluate_brep_to(self.root()?)
	}

	/// **Rollback** evaluation on the exact half: build the B-rep as the model
	/// stood at feature `id` — the prefix of the history up to and including it
	/// (the B-rep counterpart of [`Document::evaluate_to`]). Suppression and the
	/// active configuration apply; the pinned root is ignored. `None` for an
	/// unknown id or a prefix with no B-rep form.
	pub fn evaluate_brep_to(&self, id: FeatureId) -> Option<kernel_brep::Solid> {
		if id.0 >= self.features.len() {
			return None;
		}
		let mut budget = self.features.len().saturating_mul(64).max(1024);
		let params = self.effective_params();
		self.build_brep(id, &mut Vec::new(), &mut budget, &params)
	}

	/// Exact rigid-body [`MassProperties`] (volume, centre of mass, inertia at unit density) of
	/// this document's evaluated B-rep — the mass of the parametric part in one call, without
	/// the caller reaching for [`evaluate_brep`](Self::evaluate_brep). Re-evaluated each call,
	/// so it tracks parameter edits. `None` when the document has no B-rep result (e.g. an
	/// organic / implicit-only model).
	pub fn mass_properties(&self) -> Option<MassProperties> {
		self.evaluate_brep().map(|s| kernel_brep::mass_properties(&s))
	}

	/// Recursive B-rep counterpart of [`Document::build`].
	fn build_brep(&self, id: FeatureId, stack: &mut Vec<FeatureId>, budget: &mut usize, params: &HashMap<String, f64>) -> Option<kernel_brep::Solid> {
		if *budget == 0 {
			return None;
		}
		*budget -= 1;
		let feature = &self.features.get(id.0)?.feature;
		if stack.contains(&id) {
			return None;
		}
		stack.push(id);
		// A suppressed modifier feature is replaced by its upstream input.
		if self.suppressed.contains(&id) {
			if let Some(inp) = primary_input(feature) {
				let solid = self.build_brep(inp, stack, budget, params);
				stack.pop();
				return solid;
			}
		}
		let dv = |d: &[Dim; 3]| DVec3::new(d[0].resolve(params), d[1].resolve(params), d[2].resolve(params));
		let solid = match feature {
			Feature::Box { center, size } => {
				let c = dv(center);
				let half = dv(size).abs() * 0.5;
				Some(kernel_brep::cuboid(c - half, c + half))
			}
			Feature::Sphere { center, radius } => {
				let r = radius.resolve(params).max(0.0);
				Some(kernel_brep::sphere(dv(center), r, 32, 16))
			}
			Feature::Cylinder { center, radius, height } => {
				let c = dv(center);
				let r = radius.resolve(params).max(0.0);
				let h = height.resolve(params).max(0.0);
				Some(kernel_brep::cylinder(c - DVec3::new(0.0, 0.0, h * 0.5), DVec3::Z, r, h, 32))
			}
			Feature::FilletedCylinder { radius, height, fillet } => {
				let r = radius.resolve(params).max(0.0);
				let h = height.resolve(params).max(0.0);
				let f = fillet.resolve(params).max(0.0);
				Some(kernel_brep::filleted_cylinder(r, h, f, 48, 8))
			}
			Feature::ChamferedCylinder { radius, height, chamfer } => {
				let r = radius.resolve(params).max(0.0);
				let h = height.resolve(params).max(0.0);
				let c = chamfer.resolve(params).max(0.0);
				Some(kernel_brep::chamfered_cylinder(r, h, c, 48))
			}
			Feature::Boolean { op, a, b } => {
				let sa = self.build_brep(*a, stack, budget, params)?;
				let sb = self.build_brep(*b, stack, budget, params)?;
				Some(match op {
					BooleanOp::Union => kernel_brep::union(&sa, &sb),
					BooleanOp::Difference => kernel_brep::difference(&sa, &sb),
					BooleanOp::Intersection => kernel_brep::intersection(&sa, &sb),
				})
			}
			// Smooth/filleted booleans and the gyroid TPMS lattice are voxel-half organic
			// ops (smin/smax on the SDF, or a TPMS field); the exact B-rep half has no
			// analytic representation, so they are absent here (the mirror of
			// `Feature::Shell`). Mesh them via `evaluate` / `mesh`.
			Feature::SmoothUnion { .. } | Feature::SmoothDifference { .. } | Feature::SmoothIntersection { .. } | Feature::Gyroid { .. } => None,
			Feature::Transform { input, xform } => {
				let s = self.build_brep(*input, stack, budget, params)?;
				let m = xform.matrix3;
				let daff = DAffine3::from_cols(m.x_axis.as_dvec3(), m.y_axis.as_dvec3(), m.z_axis.as_dvec3(), xform.translation.as_dvec3());
				Some(s.transformed(daff))
			}
			Feature::Fillet { input, edge, radius, near } => {
				let s = self.build_brep(*input, stack, budget, params)?;
				let r = radius.resolve(params);
				// The edge name re-resolves against the freshly-rebuilt input, so the
				// fillet re-attaches after an upstream edit. With a `near` witness a name
				// that split into fragments is disambiguated to the nearest one; without,
				// an unresolved/ambiguous edge makes the document fail to evaluate rather
				// than silently return an unrounded solid.
				match near {
					Some(w) => kernel_brep::fillet_edge_near(&s, *edge, r, dv(w)).ok(),
					None => kernel_brep::fillet_edge(&s, *edge, r).ok(),
				}
			}
			Feature::Chamfer { input, edge, radius, near } => {
				let s = self.build_brep(*input, stack, budget, params)?;
				let r = radius.resolve(params);
				match near {
					Some(w) => kernel_brep::chamfer_edge_near(&s, *edge, r, dv(w)).ok(),
					None => kernel_brep::chamfer_edge(&s, *edge, r).ok(),
				}
			}
			Feature::ExtrudeSketch { sketch, height, dims, draft } => {
				// Apply the parametric dimension overrides, then solve the constraints on
				// every rebuild (cheap, idempotent) so the profile reflects the current
				// parameters, and extrude by the parameter-resolved height with the
				// parameter-resolved draft (0 ⇒ a plain prism, full hole support).
				let h = height.resolve(params);
				let a = draft.resolve(params);
				let mut sk = sketch.clone();
				for (index, dim) in dims {
					sk.set_distance(*index, dim.resolve(params));
				}
				sk.solve();
				sk.extrude_tapered(h, a).ok()
			}
			Feature::LinearPattern { input, count, step } => {
				let base = self.build_brep(*input, stack, budget, params)?;
				let s = DVec3::new(step[0].resolve(params), step[1].resolve(params), step[2].resolve(params));
				// If adjacent copies are AABB-disjoint, merge their topology directly (exact, and
				// avoids the chained curved-boolean corruption that self-intersects); otherwise the
				// copies touch/overlap and must be fused with a real boolean union.
				let merge = *count < 2 || aabb_disjoint(&base, &base.transformed(DAffine3::from_translation(s)));
				let mut acc = base.clone();
				for k in 1..(*count).max(1) {
					let copy = base.transformed(DAffine3::from_translation(s * k as f64));
					acc = if merge { acc.disjoint_union(&copy) } else { kernel_brep::union(&acc, &copy) };
				}
				Some(acc)
			}
			Feature::Mirror { input, plane_point, plane_normal } => {
				let base = self.build_brep(*input, stack, budget, params)?;
				let mirror = base.mirrored(dv(plane_point), dv(plane_normal));
				// A mirror plane that doesn't cut the part leaves base and its reflection disjoint →
				// merge their topology exactly (avoids the curved-boolean corruption); a cutting plane
				// makes them overlap on the seam → fuse with a real boolean union.
				let combined = if aabb_disjoint(&base, &mirror) {
					base.disjoint_union(&mirror)
				} else {
					kernel_brep::union(&base, &mirror)
				};
				Some(combined)
			}
			Feature::CircularPattern { input, count, axis_point, axis_dir, angle } => {
				let base = self.build_brep(*input, stack, budget, params)?;
				let p = dv(axis_point);
				let axis = dv(axis_dir).normalize_or_zero();
				let step = angle.resolve(params);
				let mut acc = base.clone();
				if axis.length_squared() >= 0.5 {
					let rot1 = DAffine3::from_translation(p) * DAffine3::from_axis_angle(axis, step) * DAffine3::from_translation(-p);
					// Disjoint copies (a typical bolt circle) merge by exact topology; overlapping
					// copies fuse with a real boolean union (see LinearPattern for the rationale).
					let merge = *count < 2 || aabb_disjoint(&base, &base.transformed(rot1));
					for k in 1..(*count).max(1) {
						let rot = DAffine3::from_translation(p) * DAffine3::from_axis_angle(axis, step * k as f64) * DAffine3::from_translation(-p);
						let copy = base.transformed(rot);
						acc = if merge { acc.disjoint_union(&copy) } else { kernel_brep::union(&acc, &copy) };
					}
				}
				Some(acc)
			}
			// A shell is a voxel-half op (inward offset + CSG difference); the exact
			// B-rep half has no general face-offset yet, so it is absent from this path
			// (the mirror of `ExtrudeSketch`, which is B-rep-only and returns `None` on
			// the implicit path). Mesh the hollowed solid via `evaluate` / `mesh`.
			Feature::Shell { .. } => None,
			Feature::Hole { input, kind, m_or_d, at, axis, fit, depth } => {
				let s = self.build_brep(*input, stack, budget, params)?;
				let at = dv(at);
				let axis = dv(axis);
				let size = m_or_d.resolve(params);
				match kind {
					// Drill / tap pilot: no fit series applies; depth `None` bores
					// through the part's whole extent along the axis from `at`.
					HoleKind::Drill | HoleKind::Tap => {
						if fit.is_some() {
							return None; // a fit series on a drill/tap is a contradiction — loud
						}
						let hole_depth = match depth {
							Some(d) => kernel_brep::HoleDepth::Blind(d.resolve(params)),
							None => kernel_brep::HoleDepth::Through(through_length(&s, at, axis)?),
						};
						match kind {
							HoleKind::Drill => kernel_brep::drill(&s, at, axis, size, hole_depth, None).ok(),
							_ => kernel_brep::tap_drill_hole(&s, at, axis, size, hole_depth, None).ok(),
						}
					}
					// The fastener seats are through cuts by definition; a depth here
					// would be silently ignored, so it fails loudly instead.
					HoleKind::Clearance | HoleKind::Counterbore | HoleKind::Countersink => {
						if depth.is_some() {
							return None;
						}
						let fit = fit.unwrap_or(HoleFit::Medium).to_brep();
						match kind {
							HoleKind::Clearance => kernel_brep::clearance_hole(&s, at, axis, size, fit, None).ok(),
							HoleKind::Counterbore => kernel_brep::counterbore_hole(&s, at, axis, size, fit, None).ok(),
							_ => kernel_brep::countersink_hole(&s, at, axis, size, fit, None).ok(),
						}
					}
				}
			}
			Feature::CircularRimFillet { input, near, radius, concave } => {
				let s = self.build_brep(*input, stack, budget, params)?;
				let witness = dv(near);
				let r = radius.resolve(params);
				// Out-of-scope rims return None, so the document fails to evaluate
				// rather than silently dropping the round (same contract as Fillet).
				if *concave {
					kernel_brep::fillet_circular_rim_concave(&s, witness, r, RIM_FILLET_ARC_SEGMENTS)
				} else {
					kernel_brep::fillet_circular_rim(&s, witness, r, RIM_FILLET_ARC_SEGMENTS)
				}
			}
			Feature::LoftSolid { sections } => {
				let sections: Vec<Vec<DVec3>> = sections.iter().map(|loop_| loop_.iter().map(dv).collect()).collect();
				kernel_brep::loft_solid(&sections)
			}
			Feature::SweepSolid { profile, path } => {
				let profile: Vec<DVec3> = profile.iter().map(dv).collect();
				let path: Vec<DVec3> = path.iter().map(dv).collect();
				kernel_brep::sweep_solid(&profile, &path)
			}
			Feature::Revolve { profile, segments } => {
				let profile: Vec<kernel_core::math::DVec2> =
					profile.iter().map(|p| kernel_core::math::DVec2::new(p[0].resolve(params), p[1].resolve(params))).collect();
				let segs = if *segments == 0 { 64 } else { *segments };
				Some(kernel_brep::revolve(&profile, segs))
			}
			Feature::CatalogPart { part } => part.build(params),
			Feature::ORingGroove { input, at, axis, dash } => {
				let s = self.build_brep(*input, stack, budget, params)?;
				parts::o_ring_groove(&s, dv(at), dv(axis), *dash)
			}
			Feature::CirclipGroove { input, at, axis, d, internal } => {
				let s = self.build_brep(*input, stack, budget, params)?;
				let d = d.resolve(params);
				if *internal {
					parts::circlip_groove_internal(&s, dv(at), dv(axis), d)
				} else {
					parts::circlip_groove_external(&s, dv(at), dv(axis), d)
				}
			}
			Feature::HeatsetBoss { input, at, axis, m } => {
				let s = self.build_brep(*input, stack, budget, params)?;
				parts::heatset_insert_boss(&s, dv(at), dv(axis), m.resolve(params))
			}
			// The TPMS / beam-lattice / pipe fills are voxel-half organic bodies: the
			// exact half has no analytic twin for them, so they are absent from this
			// path (the mirror of `Feature::Shell`). Mesh them via `evaluate` / `mesh`,
			// or fuse them onto an exact part with `Feature::HybridFuse`.
			Feature::GyroidLattice { .. } | Feature::Tpms { .. } | Feature::BeamLatticeFill { .. } | Feature::PipeFeat { .. } => None,
			Feature::HybridFuse { brep, field, op, voxel } => {
				// The cross-representation boolean: exact operand as a Solid, field
				// operand as a Node meshed at `voxel` (see `hybrid_boolean`). On the
				// EXACT-STITCH route the stitched partial-credit Solid (untouched faces
				// verbatim, provenance-tagged seam) feeds downstream B-rep features. On
				// the HEALED route — or a HybridError — this is None: a mesh-only result
				// honestly cannot chain into exact features. The watertight mesh remains
				// reachable via `Document::mesh` / `export_mesh` (which then states the
				// heal), and the route + reason + report via `hybrid_fuse_result`.
				let a = self.build_brep(*brep, stack, budget, params)?;
				let b = self.build(*field, stack, budget, params)?;
				let v = voxel.resolve(params) as f32;
				match hybrid_boolean(&a, HybridOperand::Node(&b), *op, v) {
					Ok(out) => out.solid,
					Err(_) => None,
				}
			}
		};
		stack.pop();
		solid
	}


	/// Evaluate and mesh the document at the given `resolution`.
	///
	/// Meshed with Manifold Dual Contouring, so the result is a closed **2-manifold**
	/// with sharp edges preserved — a `Difference` feature's concave crease comes out
	/// watertight rather than with the non-manifold edges plain Surface Nets leaves
	/// there. Returns an empty [`Mesh`] for an empty / invalid document.
	pub fn mesh(&self, resolution: impl Into<Resolution>) -> Mesh {
		match self.evaluate() {
			Some(node) => {
				let bounds = node.bounds();
				manifold_dual_contour(&node, bounds, resolution)
			}
			None => Mesh::new(),
		}
	}

	/// Mesh the document's **exact B-rep** result into a watertight mesh via the
	/// hybrid heal ([`watertight_mesh`]) at `voxel_size`. Returns an empty mesh if
	/// the document has no valid B-rep.
	///
	/// This is the B-rep counterpart of [`Document::mesh`] (which meshes the
	/// implicit/voxel tree directly): it builds the exact B-rep, then recovers a
	/// printable watertight mesh through the voxel half — so an AI gets a sound
	/// mesh of a parametric solid even when its exact tessellation has curved-face
	/// cracks.
	pub fn watertight_brep_mesh(&self, voxel_size: f32) -> Mesh {
		match self.evaluate_brep() {
			Some(solid) => watertight_mesh(&solid, voxel_size),
			None => Mesh::new(),
		}
	}

	/// Export this document as a mesh at chord tolerance `tol` (mm) through the
	/// kernel's **routing policy**, returning the mesh together with the
	/// [`RouteReport`] saying which path produced it and why — the one call that
	/// centralizes the exact-else-heal decision so callers stop hand-rolling it:
	///
	/// - a document with an exact B-rep routes through [`routed_mesh`]
	///   (self-intersection check → exact adaptive tessellation when watertight →
	///   voxel heal otherwise);
	/// - a document with **no** B-rep form (voxel-half features such as
	///   [`Feature::Shell`] / smooth booleans / [`Feature::Gyroid`]) is meshed on
	///   the SDF half and reported [`MeshRoute::Healed`];
	/// - an empty / invalid document returns an empty mesh (`tris == 0`,
	///   `watertight == false`) with the reason in [`RouteReport::why`].
	pub fn export_mesh(&self, tol: f64) -> (Mesh, RouteReport) {
		if let Some(solid) = self.evaluate_brep() {
			return routed_mesh(&solid, tol);
		}
		let mesh = self.mesh(Resolution::VoxelSize(heal_voxel_size(tol)));
		let report = if mesh.is_empty() {
			RouteReport::for_mesh(&mesh, MeshRoute::Healed, "empty or invalid document: no geometry to export")
		} else {
			RouteReport::for_mesh(
				&mesh,
				MeshRoute::Healed,
				"no exact B-rep for this document (voxel-half features); meshed on the SDF half",
			)
		};
		(mesh, report)
	}

	/// Re-run the [`Feature::HybridFuse`] at `id` and return its **full routed
	/// result**: the verified-watertight mesh, the [`HybridRoute`] taken (with
	/// the healed route's stated reason), the measured per-face [`HybridReport`],
	/// and — on the exact route — the stitched solid. This is the retrieval
	/// mechanism for a fuse's route: a [`Document`] is pure persisted data, so
	/// nothing is cached; the fuse is **recomputed at call time**, which is sound
	/// because the kernel rebuild is deterministic (R5) — the same document
	/// yields the same route, report and mesh bits as the
	/// [`evaluate_brep`](Self::evaluate_brep) that built it.
	///
	/// `None` when `id` does not name a `HybridFuse` or an operand fails to
	/// evaluate; `Some(Err(_))` when neither hybrid route produced a watertight
	/// result ([`HybridError`], loud); `Some(Ok(_))` otherwise — on the healed
	/// route the result's `solid` is `None` and `mesh` is the watertight voxel
	/// fuse. Suppression and the active configuration apply as usual.
	pub fn hybrid_fuse_result(&self, id: FeatureId) -> Option<Result<HybridResult, HybridError>> {
		let Feature::HybridFuse { brep, field, op, voxel } = &self.features.get(id.0)?.feature else {
			return None;
		};
		let params = self.effective_params();
		let mut budget = self.features.len().saturating_mul(64).max(1024);
		// Seed the operand stacks with `id` itself so a hand-edited
		// self-referencing fuse is rejected as cyclic instead of recursing.
		let solid = self.build_brep(*brep, &mut vec![id], &mut budget, &params)?;
		let node = self.build(*field, &mut vec![id], &mut budget, &params)?;
		Some(hybrid_boolean(&solid, HybridOperand::Node(&node), *op, voxel.resolve(&params) as f32))
	}
}

/// A bounded **undo/redo snapshot stack** for a [`Document`] — the session-level
/// edit history (distinct from the *feature* history inside the document). A
/// [`Document`] is pure data and cheap to clone, so undo is snapshot-based and
/// therefore covers *every* kind of edit (parameters, features, labels, configs,
/// suppression) with bit-exact restoration: re-evaluating an undone document
/// reproduces the earlier solid exactly (R5 determinism).
///
/// Usage: create with the initial document, [`push`](DocumentHistory::push) a
/// snapshot **after** each completed edit, and navigate with
/// [`undo`](DocumentHistory::undo) / [`redo`](DocumentHistory::redo);
/// [`current`](DocumentHistory::current) is always the live state. Pushing after
/// an undo discards the redo tail (the standard branch-discard semantics), and
/// the stack is bounded: beyond `capacity` snapshots the oldest is dropped.
#[derive(Clone, Debug)]
pub struct DocumentHistory {
	/// The snapshots, oldest first; `snapshots[cursor]` is the current state.
	snapshots: Vec<Document>,
	/// Index of the current snapshot.
	cursor: usize,
	/// Maximum number of snapshots kept (≥ 1).
	capacity: usize,
}

impl DocumentHistory {
	/// A history seeded with `initial` as its only snapshot. `capacity` bounds the
	/// stack (clamped to at least 1 — the current state is always kept).
	pub fn new(initial: Document, capacity: usize) -> Self {
		Self { snapshots: vec![initial], cursor: 0, capacity: capacity.max(1) }
	}

	/// The current document state.
	pub fn current(&self) -> &Document {
		&self.snapshots[self.cursor]
	}

	/// Record `doc` as the new current state: any redo tail (snapshots after the
	/// cursor) is discarded, and if the stack exceeds its capacity the oldest
	/// snapshot is dropped (that state becomes unreachable by undo).
	pub fn push(&mut self, doc: Document) {
		self.snapshots.truncate(self.cursor + 1);
		self.snapshots.push(doc);
		if self.snapshots.len() > self.capacity {
			self.snapshots.remove(0);
		}
		self.cursor = self.snapshots.len() - 1;
	}

	/// Whether an [`undo`](DocumentHistory::undo) can go anywhere.
	pub fn can_undo(&self) -> bool {
		self.cursor > 0
	}

	/// Whether a [`redo`](DocumentHistory::redo) can go anywhere.
	pub fn can_redo(&self) -> bool {
		self.cursor + 1 < self.snapshots.len()
	}

	/// Step back one snapshot and return the (now current) earlier state; `None`
	/// (and no change) at the bottom of the stack.
	pub fn undo(&mut self) -> Option<&Document> {
		if !self.can_undo() {
			return None;
		}
		self.cursor -= 1;
		Some(self.current())
	}

	/// Step forward one snapshot (re-applying an undone edit) and return the now
	/// current state; `None` (and no change) when there is nothing to redo.
	pub fn redo(&mut self) -> Option<&Document> {
		if !self.can_redo() {
			return None;
		}
		self.cursor += 1;
		Some(self.current())
	}
}

/// The single upstream feature a **modifier** operates on, if any — fillet, chamfer,
/// shell, transform, the patterns/mirror, and the wizard cuts (hole, rim fillet,
/// grooves, insert boss). Returns `None` for **generative**
/// features (primitives, booleans, smooth booleans, the hybrid fuse, sketches,
/// lofts/sweeps, catalog parts, lattices/pipes) that have no
/// single input. Used to implement [`Document::set_suppressed`]: a suppressed modifier
/// evaluates to this input.
fn primary_input(f: &Feature) -> Option<FeatureId> {
	match f {
		Feature::Fillet { input, .. }
		| Feature::Chamfer { input, .. }
		| Feature::Shell { input, .. }
		| Feature::Transform { input, .. }
		| Feature::LinearPattern { input, .. }
		| Feature::Mirror { input, .. }
		| Feature::CircularPattern { input, .. }
		| Feature::Hole { input, .. }
		| Feature::CircularRimFillet { input, .. }
		| Feature::ORingGroove { input, .. }
		| Feature::CirclipGroove { input, .. }
		| Feature::HeatsetBoss { input, .. } => Some(*input),
		_ => None,
	}
}

/// Quarter-arc faceting of a [`Feature::CircularRimFillet`] torus band (matches
/// the 8 used by [`kernel_brep::filleted_cylinder`] / [`Feature::FilletedCylinder`]).
const RIM_FILLET_ARC_SEGMENTS: usize = 8;

/// Memory rail of [`Feature::BeamLatticeFill`]: the maximum number of unit
/// cells one fill may instantiate; beyond it the feature fails to evaluate
/// (loud `None`). At the octet's ~36 struts/cell this still allows
/// multi-million-strut graphs — the rail exists so a hand-edited `cell_size`
/// typo (e.g. 0.001 over a 50 mm region) cannot exhaust process memory.
pub const LATTICE_FILL_MAX_CELLS: usize = 100_000;

/// Resolve a `[[Dim; 3]; 2]` corner pair into `(center, half_extent)`. The
/// corners may come in any order — an inverted region is normalized through the
/// `abs`, mirroring the `size.abs()` guard of [`Feature::Box`].
fn resolve_region(params: &HashMap<String, f64>, region: &[[Dim; 3]; 2]) -> (Vec3, Vec3) {
	let a = resolve_vec3(params, &region[0]);
	let b = resolve_vec3(params, &region[1]);
	((a + b) * 0.5, ((b - a) * 0.5).abs())
}

/// Material extent of `solid` measured from `at` along `axis` (the largest AABB-corner
/// projection) — how far a "through everything" [`Feature::Hole`] must bore. `None`
/// for a degenerate axis or when no material lies ahead of `at` (the cut would miss).
fn through_length(solid: &kernel_brep::Solid, at: DVec3, axis: DVec3) -> Option<f64> {
	let axis = axis.try_normalize()?;
	let (lo, hi) = solid.aabb();
	let mut t_max = f64::NEG_INFINITY;
	for i in 0..8 {
		let corner = DVec3::new(
			if i & 1 == 0 { lo.x } else { hi.x },
			if i & 2 == 0 { lo.y } else { hi.y },
			if i & 4 == 0 { lo.z } else { hi.z },
		);
		t_max = t_max.max((corner - at).dot(axis));
	}
	(t_max > 0.0).then_some(t_max)
}

/// Rewrite every [`FeatureId`] reference inside `feature` through `map` — the id
/// remapping behind [`Document::insert_feature_at`]. Every variant that references
/// earlier features appears here; variants without references are untouched.
fn remap_feature_refs(feature: &mut Feature, map: impl Fn(FeatureId) -> FeatureId) {
	match feature {
		Feature::Boolean { a, b, .. }
		| Feature::SmoothUnion { a, b, .. }
		| Feature::SmoothDifference { a, b, .. }
		| Feature::SmoothIntersection { a, b, .. } => {
			*a = map(*a);
			*b = map(*b);
		}
		Feature::HybridFuse { brep, field, .. } => {
			*brep = map(*brep);
			*field = map(*field);
		}
		Feature::Transform { input, .. }
		| Feature::Fillet { input, .. }
		| Feature::Chamfer { input, .. }
		| Feature::LinearPattern { input, .. }
		| Feature::Mirror { input, .. }
		| Feature::CircularPattern { input, .. }
		| Feature::Shell { input, .. }
		| Feature::Hole { input, .. }
		| Feature::CircularRimFillet { input, .. }
		| Feature::ORingGroove { input, .. }
		| Feature::CirclipGroove { input, .. }
		| Feature::HeatsetBoss { input, .. } => *input = map(*input),
		Feature::Box { .. }
		| Feature::Sphere { .. }
		| Feature::Cylinder { .. }
		| Feature::FilletedCylinder { .. }
		| Feature::ChamferedCylinder { .. }
		| Feature::Gyroid { .. }
		| Feature::GyroidLattice { .. }
		| Feature::Tpms { .. }
		| Feature::BeamLatticeFill { .. }
		| Feature::PipeFeat { .. }
		| Feature::ExtrudeSketch { .. }
		| Feature::LoftSolid { .. }
		| Feature::SweepSolid { .. }
		| Feature::Revolve { .. }
		| Feature::CatalogPart { .. } => {}
	}
}

/// Resolve three [`Dim`]s into a `Vec3` (the implicit side is `f32`).
fn resolve_vec3(params: &HashMap<String, f64>, dims: &[Dim; 3]) -> Vec3 {
	Vec3::new(
		dims[0].resolve(params) as f32,
		dims[1].resolve(params) as f32,
		dims[2].resolve(params) as f32,
	)
}

/// Whether two solids' axis-aligned bounds are strictly disjoint (provably non-overlapping).
/// Used to decide that pattern copies can be combined by exact topology merge
/// ([`kernel_brep::Solid::disjoint_union`]) rather than a boolean union.
fn aabb_disjoint(a: &kernel_brep::Solid, b: &kernel_brep::Solid) -> bool {
	let (amin, amax) = a.aabb();
	let (bmin, bmax) = b.aabb();
	amax.x < bmin.x || bmax.x < amin.x || amax.y < bmin.y || bmax.y < amin.y || amax.z < bmin.z || bmax.z < amin.z
}

/// Build the reflection [`Affine3A`] across the plane through `plane_point` with
/// the given `plane_normal` (need not be unit): `x ↦ x − 2((x−p)·n)n`. Returns the
/// identity for a degenerate normal. Used by the implicit [`Feature::Mirror`] path.
fn reflection_affine(plane_point: Vec3, plane_normal: Vec3) -> Affine3A {
	let n = plane_normal.normalize_or_zero();
	if n.length_squared() < 0.5 {
		return Affine3A::IDENTITY;
	}
	let col = |e: Vec3, nj: f32| e - n * (2.0 * nj);
	let m3 = Mat3::from_cols(col(Vec3::X, n.x), col(Vec3::Y, n.y), col(Vec3::Z, n.z));
	Affine3A::from_mat3_translation(m3, n * (2.0 * plane_point.dot(n)))
}

/// Heal a B-rep [`kernel_brep::Solid`] into a watertight mesh **via the voxel half** —
/// the hybrid's core move. The solid is tessellated, lifted into a winding-number
/// signed-distance field ([`kernel_implicit::MeshSdf`]), and re-meshed with Manifold
/// Dual Contouring. This recovers a watertight, 2-manifold mesh for a solid whose
/// *exact* tessellation has T-junctions / cracks — e.g. the faceted curved↔planar
/// joints of a drilled or filleted curved part — so the engine can hand a printable
/// mesh back for geometry the exact path can't tessellate watertight. `voxel_size`
/// trades fidelity for speed; a closed surface is recovered regardless of the input's
/// per-face cracks because the SDF sign is global (winding number), not edge-based.
pub fn watertight_mesh(solid: &kernel_brep::Solid, voxel_size: f32) -> Mesh {
	watertight_mesh_of(&kernel_brep::tessellate_default(solid), voxel_size)
}

/// Mesh a B-rep [`kernel_brep::Solid`] to a watertight surface at chord tolerance `tol`
/// (mm), **preferring the exact analytic path**: [`kernel_brep::tessellate_adaptive_tol`]
/// follows the true surfaces (a micron-smooth cylinder/cone/sphere, not a voxel grid) and
/// is watertight by its shared-edge invariant. Only when the exact tessellation is *not*
/// watertight (a self-intersecting input, or a topology the adaptive stitcher can't seal)
/// does it fall back to the voxel heal ([`watertight_mesh`]). This is the precision
/// counterpart of [`Document::watertight_brep_mesh`] — an AI gets crisp, resin-quality
/// geometry for the common analytic parts and a guaranteed-watertight mesh for the rest.
pub fn precise_mesh(solid: &kernel_brep::Solid, tol: f64) -> Mesh {
	let exact = kernel_brep::tessellate_adaptive_tol(solid, tol);
	if exact.is_watertight() {
		exact
	} else {
		// Voxel fallback: clamp to a sane voxel size (a micron chord tol is far finer than
		// any practical voxel grid).
		watertight_mesh(solid, heal_voxel_size(tol))
	}
}

/// The voxel size the heal fallback runs at for a chord tolerance `tol` (clamped to a
/// sane grid; a micron chord tol is far finer than any practical voxel grid).
fn heal_voxel_size(tol: f64) -> f32 {
	(tol * 20.0).clamp(0.1, 0.5) as f32
}

/// Which path a routed export took (see [`RouteReport`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MeshRoute {
	/// The exact analytic tessellation: triangles lie on the true surfaces to the
	/// chord tolerance, watertight by the shared-edge invariant.
	Exact,
	/// The voxel heal: tessellation → winding-number SDF → Manifold Dual
	/// Contouring. Watertight by construction, accurate to the heal voxel size.
	Healed,
}

/// The routing verdict of [`routed_mesh`] / [`Document::export_mesh`] — *which*
/// mesh an AI was handed and *why*, so the exact-else-heal decision is auditable
/// instead of silent.
#[derive(Clone, Debug)]
pub struct RouteReport {
	/// The path taken.
	pub route: MeshRoute,
	/// Human/AI-readable reason for the routing decision.
	pub why: String,
	/// Triangle count of the returned mesh.
	pub tris: usize,
	/// Whether the returned mesh is watertight (`false` only for an empty result —
	/// both routes otherwise return closed meshes).
	pub watertight: bool,
}

impl RouteReport {
	/// Assemble the report for a finished `mesh`.
	fn for_mesh(mesh: &Mesh, route: MeshRoute, why: impl Into<String>) -> RouteReport {
		RouteReport { route, why: why.into(), tris: mesh.triangle_count(), watertight: mesh.is_watertight() }
	}
}

/// Mesh a B-rep solid at chord tolerance `tol` (mm) through the kernel's **routing
/// policy**, returning the mesh together with the [`RouteReport`] saying which path
/// produced it — the auditable core of [`Document::export_mesh`]:
///
/// 1. a solid whose B-rep **self-intersects** (e.g. an overlapping helical sweep)
///    is healed immediately — its exact tessellation can be edge-watertight yet
///    geometrically corrupt, so it must never ship as "exact";
/// 2. otherwise the **exact** adaptive tessellation is used when it is watertight;
/// 3. otherwise (curved-face cracks the stitcher cannot seal) the **voxel heal**
///    recovers a guaranteed-watertight mesh.
pub fn routed_mesh(solid: &kernel_brep::Solid, tol: f64) -> (Mesh, RouteReport) {
	if kernel_brep::self_intersects(solid) {
		let mesh = watertight_mesh(solid, heal_voxel_size(tol));
		let report = RouteReport::for_mesh(
			&mesh,
			MeshRoute::Healed,
			"exact B-rep self-intersects (its tessellation would be geometrically corrupt); healed via the winding-number voxel field",
		);
		return (mesh, report);
	}
	let exact = kernel_brep::tessellate_adaptive_tol(solid, tol);
	if exact.is_watertight() {
		let report = RouteReport::for_mesh(&exact, MeshRoute::Exact, "exact adaptive tessellation is watertight (analytic surfaces, no voxel grid)");
		(exact, report)
	} else {
		let mesh = watertight_mesh(solid, heal_voxel_size(tol));
		let report = RouteReport::for_mesh(
			&mesh,
			MeshRoute::Healed,
			"exact tessellation is not watertight (curved-face cracks); healed via the winding-number voxel field",
		);
		(mesh, report)
	}
}

/// Heal an arbitrary triangle-soup [`Mesh`] into a watertight, 2-manifold mesh via the
/// voxel half: the soup is lifted into a winding-number signed-distance field
/// ([`kernel_implicit::MeshSdf`]) and re-meshed with Manifold Dual Contouring. Unlike
/// [`watertight_mesh`] (which starts from a single B-rep solid), this **fuses pre-meshed
/// parts** — e.g. a shank and a self-intersecting helical thread whose *exact* B-rep
/// union is not a valid manifold — into one clean closed surface, because the
/// winding-number sign is global (inside/outside of the whole soup), not per-edge.
/// `voxel_size` sets the surface precision (small enough resolves sub-millimetre
/// features like thread crests). Returns an empty mesh for empty input.
pub fn watertight_mesh_of(mesh: &Mesh, voxel_size: f32) -> Mesh {
	if mesh.triangle_count() == 0 {
		return Mesh::new();
	}
	let msdf = kernel_implicit::MeshSdf::new(mesh);
	let bounds = msdf.bounds().pad(voxel_size * 2.0);
	manifold_dual_contour(&msdf, bounds, Resolution::VoxelSize(voxel_size))
}

/// The geometry source an [`Instance`] places into an [`Assembly`].
///
/// A [`Source::Doc`] is re-evaluated every time the assembly is meshed, so it
/// stays parametric; a [`Source::Built`] is a prebuilt static [`Node`].
pub enum Source {
	/// A parametric document, re-evaluated on every mesh.
	Doc(Document),
	/// A prebuilt CSG node.
	Built(Node),
}

/// One placed component of an [`Assembly`]: a geometry [`Source`] at a pose.
pub struct Instance {
	/// The geometry to place.
	pub source: Source,
	/// Local → world placement transform (rigid + uniform scale).
	pub pose: Affine3A,
}

impl Instance {
	/// Place a parametric document at `pose`.
	pub fn document(doc: Document, pose: Affine3A) -> Self {
		Self { source: Source::Doc(doc), pose }
	}

	/// Place a prebuilt node at `pose`.
	pub fn node(node: Node, pose: Affine3A) -> Self {
		Self { source: Source::Built(node), pose }
	}

	/// Place an imported / scanned triangle mesh as an assembly component. The mesh is lifted
	/// into a winding-number signed-distance field ([`kernel_implicit::MeshSdf`]) and wrapped
	/// as a prebuilt CSG node, so a part read via [`Mesh::read_3mf`] / `read_obj` / `read_stl`
	/// drops straight into an [`Assembly`] and participates in meshing, [`clearance`] /
	/// [`interferences`] and [`mass_properties`] like any other instance.
	///
	/// [`clearance`]: Assembly::clearance
	/// [`interferences`]: Assembly::interferences
	/// [`mass_properties`]: Assembly::mass_properties
	pub fn from_mesh(mesh: &Mesh, pose: Affine3A) -> Self {
		Self::node(Node::primitive(kernel_implicit::MeshSdf::new(mesh)), pose)
	}

	/// Local-space [`Sdf`] this instance draws from, if it produces geometry.
	///
	/// A document is evaluated to a fresh [`Node`] each call (staying parametric);
	/// a prebuilt node is borrowed in place. A **B-rep-only** document (catalog
	/// parts, sketch extrudes, hole-wizard cuts, … — every feature the implicit
	/// half evaluates to `None`) is bridged through the winding-number
	/// [`kernel_implicit::MeshSdf`] of its exact tessellation, so assembly-level
	/// SDF queries ([`Assembly::interference_volume`], voxel meshing) see the same
	/// material a B-rep caller would — such instances used to contribute EMPTY
	/// geometry silently. The returned reference / value is consumed immediately
	/// by [`Instance::world_bounds`] / [`Instance::mesh`], so the non-`Clone`
	/// prebuilt node never has to be copied.
	fn with_local_sdf<R>(&self, f: impl FnOnce(&dyn Sdf) -> R) -> Option<R> {
		match &self.source {
			Source::Doc(doc) => match doc.evaluate() {
				Some(node) => Some(f(&node)),
				None => doc
					.evaluate_brep()
					.map(|solid| f(&kernel_implicit::MeshSdf::new(&kernel_brep::tessellate_default(&solid)))),
			},
			Source::Built(node) => Some(f(node)),
		}
	}

	/// World-space bound of this instance, if it produces geometry. A B-rep-only
	/// document's bound comes straight from its exact B-rep AABB (no implicit
	/// bridge needed just for bounds).
	fn world_bounds(&self) -> Option<Aabb> {
		if let Source::Doc(doc) = &self.source {
			return match doc.evaluate() {
				Some(node) => Some(transform_aabb(node.bounds(), self.pose)),
				None => doc.evaluate_brep().map(|solid| {
					let (lo, hi) = solid.aabb();
					transform_aabb(Aabb::new(lo.as_vec3(), hi.as_vec3()), self.pose)
				}),
			};
		}
		self.with_local_sdf(|sdf| transform_aabb(sdf.bounds(), self.pose))
	}

	/// Mesh this instance into world space at `resolution`.
	///
	/// The local field is meshed in its own (local) bound, then the resulting
	/// vertices and normals are mapped through the pose — so a prebuilt,
	/// non-`Clone` node never needs to be wrapped back into the CSG tree. Manifold
	/// Dual Contouring keeps each placed part a watertight 2-manifold.
	fn mesh(&self, resolution: Resolution) -> Mesh {
		let part = self.with_local_sdf(|sdf| manifold_dual_contour(sdf, sdf.bounds(), resolution));
		match part {
			Some(mut mesh) => {
				transform_mesh(&mut mesh, self.pose);
				mesh
			}
			None => Mesh::new(),
		}
	}

	/// Mesh this instance into world space keeping B-rep parts CRISP: a parametric document
	/// with an exact B-rep is tessellated analytically to chord tolerance `tol` (no voxel grid),
	/// so a placed precision part stays micron-sharp. Organic/implicit parts (no exact B-rep, or
	/// a prebuilt CSG node) fall back to the voxel mesh at `fallback`.
	fn mesh_exact(&self, tol: f64, fallback: Resolution) -> Mesh {
		let local = match &self.source {
			Source::Doc(doc) => doc.evaluate_brep().map(|solid| precise_mesh(&solid, tol)),
			Source::Built(_) => None,
		};
		match local {
			Some(mut mesh) => {
				transform_mesh(&mut mesh, self.pose);
				mesh
			}
			None => self.mesh(fallback),
		}
	}

	/// World-space mesh for DISTANCE MEASUREMENT (not export): a document with an
	/// exact B-rep is tessellated adaptively at chord `tol` and used **raw** — its
	/// vertices lie on the true analytic surfaces and watertightness is irrelevant
	/// to a distance query, so the voxel heal (which would smear sub-voxel fits
	/// like a bearing seat) is never taken. Organic/prebuilt parts voxel-mesh at
	/// `fallback`, exactly as in [`Instance::mesh`].
	fn measure_mesh(&self, tol: f64, fallback: Resolution) -> Mesh {
		let local = match &self.source {
			Source::Doc(doc) => doc.evaluate_brep().map(|solid| kernel_brep::tessellate_adaptive_tol(&solid, tol)),
			Source::Built(_) => None,
		};
		match local {
			Some(mut mesh) => {
				transform_mesh(&mut mesh, self.pose);
				mesh
			}
			None => self.mesh(fallback),
		}
	}

	/// Local-frame rigid-body [`MassProperties`] (unit density). A parametric document with
	/// an exact B-rep uses the analytic [`kernel_brep::mass_properties`] (exact volume,
	/// no voxel grid); an organic document or a prebuilt node falls back to its watertight
	/// voxel mesh at `fallback`. `None` when the instance produces no geometry.
	fn local_mass_properties(&self, fallback: Resolution) -> Option<MassProperties> {
		if let Source::Doc(doc) = &self.source {
			if let Some(solid) = doc.evaluate_brep() {
				return Some(kernel_brep::mass_properties(&solid));
			}
		}
		self.with_local_sdf(|sdf| manifold_dual_contour(sdf, sdf.bounds(), fallback).mass_properties())
	}
}

/// AABB of `b` after transforming its eight corners by `m`.
fn transform_aabb(b: Aabb, m: Affine3A) -> Aabb {
	if !b.is_valid() || !b.min.is_finite() || !b.max.is_finite() {
		return b; // leave degenerate / infinite bounds untouched
	}
	let mut out = Aabb::empty();
	for c in b.corners() {
		out = out.expand_point(m.transform_point3(c));
	}
	out
}

/// Map a mesh's positions (and normals) from local into world space by `m`.
///
/// Normals are rotated by the linear part and renormalized so uniform scale is
/// handled correctly.
fn transform_mesh(mesh: &mut Mesh, m: Affine3A) {
	for p in mesh.positions.iter_mut() {
		*p = m.transform_point3(*p);
	}
	// Normals transform by the inverse-transpose of the linear part, which is correct
	// under non-uniform scale (the plain linear map would shear them off the surface).
	// `normalize_or_zero` absorbs the uniform-scale factor and a singular linear part.
	let normal_mat = m.matrix3.inverse().transpose();
	for n in mesh.normals.iter_mut() {
		*n = (normal_mat * *n).normalize_or_zero();
	}
	// A negative-determinant (mirroring) pose flips orientation; restore outward.
	mesh.ensure_outward();
}

/// A named **assembly state**: a snapshot of every instance's pose plus the set of
/// suppressed instances — the "exploded" / "service position" / "packed" variants
/// of one assembly. Captured with [`Assembly::capture_state`], re-applied with
/// [`Assembly::apply_state`], and persisted by name in a `.lmcasm`
/// ([`format::save_assembly_with_states`]).
#[derive(Clone, Debug)]
pub struct AsmState {
	/// Per-instance poses, parallel to [`Assembly::instances`].
	pub poses: Vec<Affine3A>,
	/// Indices of the suppressed instances (sorted, deduplicated on capture).
	pub suppressed: Vec<usize>,
}

/// A collection of placed [`Instance`]s forming a multi-part model.
#[derive(Default)]
pub struct Assembly {
	/// The placed components.
	pub instances: Vec<Instance>,
	/// Indices of instances toggled off (see [`Assembly::set_instance_suppressed`]).
	suppressed: HashSet<usize>,
}

impl Assembly {
	/// An empty assembly.
	pub fn new() -> Self {
		Self::default()
	}

	/// Add an instance, returning its index.
	pub fn add(&mut self, instance: Instance) -> usize {
		let i = self.instances.len();
		self.instances.push(instance);
		i
	}

	/// Suppress or un-suppress instance `index` — the assembly counterpart of
	/// [`Document::set_suppressed`]: a suppressed instance stays in the assembly
	/// (its index and pose are kept, so mates keep referring to it) but contributes
	/// **no geometry**: it is skipped by [`Assembly::mesh_all`] /
	/// [`mesh_all_exact`](Assembly::mesh_all_exact), [`bounds`](Assembly::bounds),
	/// [`mass_properties`](Assembly::mass_properties) and the clearance /
	/// interference queries. [`Assembly::solve_mates`] still solves its pose (a
	/// suppressed part is absent material, not a broken mate).
	pub fn set_instance_suppressed(&mut self, index: usize, suppressed: bool) {
		if suppressed {
			self.suppressed.insert(index);
		} else {
			self.suppressed.remove(&index);
		}
	}

	/// Whether instance `index` is currently suppressed.
	pub fn is_instance_suppressed(&self, index: usize) -> bool {
		self.suppressed.contains(&index)
	}

	/// Snapshot the current poses and suppression set as a named-state payload
	/// (see [`AsmState`]); the suppression list comes out sorted.
	pub fn capture_state(&self) -> AsmState {
		let mut suppressed: Vec<usize> = self.suppressed.iter().copied().filter(|&i| i < self.instances.len()).collect();
		suppressed.sort_unstable();
		AsmState { poses: self.instances.iter().map(|i| i.pose).collect(), suppressed }
	}

	/// Re-apply a captured [`AsmState`]: every instance's pose is overwritten and
	/// the suppression set replaced. Returns `false` — and changes nothing — when
	/// the state does not fit this assembly (pose count ≠ instance count, or a
	/// suppressed index out of range), so a stale state cannot half-apply.
	pub fn apply_state(&mut self, state: &AsmState) -> bool {
		if state.poses.len() != self.instances.len() || state.suppressed.iter().any(|&i| i >= self.instances.len()) {
			return false;
		}
		for (instance, &pose) in self.instances.iter_mut().zip(&state.poses) {
			instance.pose = pose;
		}
		self.suppressed = state.suppressed.iter().copied().collect();
		true
	}

	/// The unsuppressed instances, with their indices.
	fn active_instances(&self) -> impl Iterator<Item = (usize, &Instance)> {
		self.instances.iter().enumerate().filter(|(i, _)| !self.suppressed.contains(i))
	}

	/// World-space bound spanning every (unsuppressed) instance.
	///
	/// Returns [`Aabb::empty`] for an assembly with no meshable geometry.
	pub fn bounds(&self) -> Aabb {
		let mut out = Aabb::empty();
		for (_, instance) in self.active_instances() {
			if let Some(b) = instance.world_bounds() {
				out = out.union(b);
			}
		}
		out
	}

	/// Mesh every instance at `resolution` and merge into one combined [`Mesh`].
	///
	/// Each instance is meshed in its own local bound, transformed into world
	/// space, then its vertices / triangles / normals are appended with re-based
	/// indices. The result is a single (possibly multi-shell) mesh ready for export.
	pub fn mesh_all(&self, resolution: impl Into<Resolution>) -> Mesh {
		let resolution = resolution.into();
		let mut combined = Mesh::new();
		for (_, instance) in self.active_instances() {
			let part = instance.mesh(resolution);
			append_mesh(&mut combined, &part);
		}
		combined
	}

	/// Mesh every instance keeping B-rep parts EXACT and merge into one combined [`Mesh`].
	///
	/// Each parametric-document part with an exact B-rep is tessellated analytically to chord
	/// tolerance `tol` (micron-crisp, no voxel quantization); organic/implicit parts fall back to
	/// the voxel mesh at `fallback`. This is the precision counterpart to [`Assembly::mesh_all`]
	/// for assemblies of machined/B-rep components, which would otherwise be voxelized.
	pub fn mesh_all_exact(&self, tol: f64, fallback: impl Into<Resolution>) -> Mesh {
		let fallback = fallback.into();
		let mut combined = Mesh::new();
		for (_, instance) in self.active_instances() {
			let part = instance.mesh_exact(tol, fallback);
			append_mesh(&mut combined, &part);
		}
		combined
	}

	/// World-space mesh of ONE instance through the exact-preferring **routing
	/// policy** ([`routed_mesh`]): a parametric document with an exact B-rep is
	/// tessellated analytically at chord tolerance `tol` (voxel-healed only when
	/// the exact tessellation is leaky or self-intersecting), organic/prebuilt
	/// parts fall back to the voxel mesh at `fallback`, and the result is posed
	/// into world space. `None` when `index` is out of range, the instance is
	/// suppressed, or it produces no geometry — never a silent empty mesh, so a
	/// caller exporting per-instance files can fail loudly on a part that
	/// contributed nothing.
	pub fn mesh_instance_exact(&self, index: usize, tol: f64, fallback: impl Into<Resolution>) -> Option<Mesh> {
		self.mesh_instance_exact_routed(index, tol, fallback).map(|(mesh, _)| mesh)
	}

	/// [`Assembly::mesh_instance_exact`] plus the honest [`RouteReport`] of the
	/// path taken (exact analytic tessellation vs voxel heal vs implicit voxel
	/// mesh), so an exporter can SAY which fidelity each placed part shipped at
	/// instead of silently degrading.
	pub fn mesh_instance_exact_routed(&self, index: usize, tol: f64, fallback: impl Into<Resolution>) -> Option<(Mesh, RouteReport)> {
		if self.suppressed.contains(&index) {
			return None;
		}
		let instance = self.instances.get(index)?;
		if let Source::Doc(doc) = &instance.source {
			if let Some(solid) = doc.evaluate_brep() {
				let (mut mesh, report) = routed_mesh(&solid, tol);
				if mesh.triangle_count() == 0 {
					return None;
				}
				transform_mesh(&mut mesh, instance.pose);
				return Some((mesh, report));
			}
		}
		let mesh = instance.mesh(fallback.into());
		if mesh.triangle_count() == 0 {
			return None;
		}
		let report = RouteReport {
			route: MeshRoute::Healed,
			why: "implicit/voxel part (no exact B-rep); Manifold Dual Contouring at the fallback resolution".to_string(),
			tris: mesh.triangle_count(),
			watertight: mesh.is_watertight(),
		};
		Some((mesh, report))
	}

	/// Exact rigid-body [`MassProperties`] of the whole assembly at unit density: each
	/// instance's local properties are taken (B-rep-exact for a parametric document,
	/// voxel-meshed at `fallback` for organic/prebuilt parts), brought into world space by
	/// its [`Instance::pose`], and summed by the parallel-axis theorem via
	/// [`MassProperties::combine`]. So an AI gets an assembly's total mass, balance point and
	/// inertia without re-meshing the union — and B-rep components contribute their analytic
	/// volume rather than a tessellated approximation. Assumes rigid poses (no scale) and
	/// non-overlapping parts (overlapping material is double-counted).
	pub fn mass_properties(&self, fallback: impl Into<Resolution>) -> MassProperties {
		let fallback = fallback.into();
		let parts: Vec<MassProperties> = self
			.active_instances()
			.filter_map(|(_, instance)| {
				let local = instance.local_mass_properties(fallback)?;
				let m = instance.pose.matrix3;
				let rotation = DMat3::from_cols(m.x_axis.as_dvec3(), m.y_axis.as_dvec3(), m.z_axis.as_dvec3());
				Some(local.transformed(rotation, instance.pose.translation.as_dvec3()))
			})
			.collect();
		MassProperties::combine(&parts)
	}

	/// Chord tolerance for the exact-tessellation side of the proximity queries
	/// ([`Assembly::clearance`] / [`Assembly::interferences`]): an 8× refinement
	/// over the voxel bound those APIs promise, clamped to stay meaningful.
	fn proximity_chord_tol(&self, resolution: Resolution) -> f64 {
		let voxel = resolution.voxel_size(self.bounds());
		if voxel.is_finite() && voxel > 0.0 {
			(voxel as f64 / 8.0).max(1e-4)
		} else {
			0.05
		}
	}

	/// World-space measurement mesh of an unsuppressed instance ([`Instance::measure_mesh`]),
	/// `None` for a suppressed / out-of-range / geometry-less one.
	fn measurement_mesh(&self, index: usize, tol: f64, fallback: Resolution) -> Option<Mesh> {
		if self.suppressed.contains(&index) {
			return None;
		}
		let mesh = self.instances.get(index)?.measure_mesh(tol, fallback);
		(mesh.triangle_count() > 0).then_some(mesh)
	}

	/// Minimum clearance (world space) between instances `i` and `j`: the gap between their
	/// surfaces, `0.0` when they touch or interfere (penetration is caught by a true
	/// triangle–triangle test). [`f64::INFINITY`] if either index is out of range, is
	/// suppressed, or has no geometry. Each part is meshed for **measurement**: a B-rep
	/// document is tessellated on its exact analytic surfaces at an ⅛-voxel chord tolerance
	/// — so catalog gears, sketch extrudes and hole-wizard parts are measured, not silently
	/// skipped — and organic/prebuilt parts are voxel-meshed at `resolution`, which
	/// therefore still bounds the result. NOTE: detects surface contact/penetration — a
	/// part fully ENGULFED inside another (no surface crossing) reports the gap between the
	/// two shells, not zero.
	pub fn clearance(&self, i: usize, j: usize, resolution: impl Into<Resolution>) -> f64 {
		let resolution = resolution.into();
		if i >= self.instances.len() || j >= self.instances.len() {
			return f64::INFINITY;
		}
		let tol = self.proximity_chord_tol(resolution);
		match (self.measurement_mesh(i, tol, resolution), self.measurement_mesh(j, tol, resolution)) {
			(Some(a), Some(b)) => a.min_distance(&b),
			_ => f64::INFINITY,
		}
	}

	/// Every pair of instances whose clearance is `≤ tol` — the assembly's interference /
	/// clash set (`tol = 0` finds touching-or-penetrating pairs; a small positive `tol` adds
	/// a safety margin). The boolean form of [`Assembly::proximity_pairs`] with the chord
	/// tolerance derived from `resolution` (⅛ voxel; organic parts voxel-meshed at
	/// `resolution`). Pairs are returned as ascending `(i, j)` index tuples. Same
	/// engulfed-part caveat as [`Assembly::clearance`].
	pub fn interferences(&self, tol: f64, resolution: impl Into<Resolution>) -> Vec<(usize, usize)> {
		let resolution = resolution.into();
		let chord = self.proximity_chord_tol(resolution);
		self.proximity_pairs(tol, chord, resolution).into_iter().map(|(i, j, _)| (i, j)).collect()
	}

	/// The assembly's quantitative proximity scan: every unsuppressed instance pair whose
	/// world surface distance is `≤ window`, as ascending `(i, j, distance)` tuples — the
	/// data an assembly checker reports (`distance == 0` ⇒ touching or penetrating; small
	/// positive ⇒ a near fit worth listing). B-rep parts are measured on their **raw exact
	/// tessellation** at chord `tol` (vertices on the true analytic surfaces, so sub-voxel
	/// fits like a 0.05 mm gear-flank gap survive; the watertight heal is never taken for
	/// measurement); organic/prebuilt parts voxel-mesh at `fallback`. Parts are meshed once
	/// each; far pairs are pruned by the rigorous AABB-gap bound (the box gap lower-bounds
	/// the surface distance, so no pair within `window` is ever skipped).
	pub fn proximity_pairs(&self, window: f64, tol: f64, fallback: impl Into<Resolution>) -> Vec<(usize, usize, f64)> {
		let fallback = fallback.into();
		// Suppressed instances contribute no geometry, so they cannot clash.
		let meshes: Vec<Option<Mesh>> = (0..self.instances.len()).map(|i| self.measurement_mesh(i, tol, fallback)).collect();
		let boxes: Vec<Option<Aabb>> = meshes.iter().map(|m| m.as_ref().map(Mesh::aabb)).collect();
		let mut hits = Vec::new();
		for i in 0..meshes.len() {
			for j in (i + 1)..meshes.len() {
				if let (Some(a), Some(b)) = (&meshes[i], &meshes[j]) {
					let (ba, bb) = (boxes[i].expect("boxed with mesh"), boxes[j].expect("boxed with mesh"));
					if aabb_gap(ba, bb) > window {
						continue; // box gap lower-bounds the surface distance
					}
					let d = a.min_distance(b);
					if d <= window {
						hits.push((i, j, d));
					}
				}
			}
		}
		hits
	}

	/// Approximate overlap **volume** (mm³) between instances `i` and `j` — how much material
	/// two parts share, where [`interferences`](Assembly::interferences) only flags that they
	/// touch. Both instances' signed-distance fields are sampled on a regular grid of cell
	/// size `voxel` over their world-AABB overlap (a B-rep-only document is bridged through
	/// the winding-number [`kernel_implicit::MeshSdf`] of its exact tessellation, so catalog
	/// gears / sketch extrudes / hole-wizard parts contribute material here too); a cell
	/// counts when its centre is inside both. `0.0` when an index is out of range or the
	/// parts are disjoint. Resolution-bounded by `voxel` (smaller = more accurate, more
	/// samples).
	pub fn interference_volume(&self, i: usize, j: usize, voxel: f64) -> f64 {
		if self.suppressed.contains(&i) || self.suppressed.contains(&j) {
			return 0.0; // a suppressed instance has no material to share
		}
		let (Some(a), Some(b)) = (self.instances.get(i), self.instances.get(j)) else {
			return 0.0;
		};
		let (Some(ba), Some(bb)) = (a.world_bounds(), b.world_bounds()) else {
			return 0.0;
		};
		let lo = ba.min.max(bb.min);
		let hi = ba.max.min(bb.max);
		let size = hi - lo;
		if voxel <= 0.0 || size.min_element() <= 0.0 {
			return 0.0;
		}
		let (inv_a, inv_b) = (a.pose.inverse(), b.pose.inverse());
		let c = voxel as f32;
		let n = |s: f32| (s / c).ceil().max(0.0) as i32;
		let (nx, ny, nz) = (n(size.x), n(size.y), n(size.z));
		a.with_local_sdf(|sa| {
			b.with_local_sdf(|sb| {
				let mut count = 0u64;
				for ix in 0..nx {
					for iy in 0..ny {
						for iz in 0..nz {
							let p = lo + Vec3::new((ix as f32 + 0.5) * c, (iy as f32 + 0.5) * c, (iz as f32 + 0.5) * c);
							if sa.distance(inv_a.transform_point3(p)) < 0.0 && sb.distance(inv_b.transform_point3(p)) < 0.0 {
								count += 1;
							}
						}
					}
				}
				count as f64 * voxel * voxel * voxel
			})
		})
		.flatten()
		.unwrap_or(0.0)
	}

	/// Solve mate `constraints` over the instances and write the solved poses back,
	/// returning the residual error (`~0` ⇒ all mates satisfied).
	///
	/// The instances' current [`Instance::pose`]s seed a [`ConstraintSystem`]
	/// (instance `0` is the fixed ground frame); after solving, each instance's pose
	/// is updated in place — so a place → mate → `solve_mates` → [`Assembly::mesh_all`]
	/// loop runs end-to-end through the assembly. Constraints reference instances by
	/// the index returned from [`Assembly::add`], and their geometry can be derived
	/// from a part's B-rep via [`kernel_brep::Solid::face_plane`] /
	/// [`kernel_brep::Solid::face_axis`].
	pub fn solve_mates(&mut self, constraints: &[Constraint], iterations: usize) -> f64 {
		let mut sys = ConstraintSystem::new(self.instances.iter().map(|i| i.pose).collect(), constraints.to_vec());
		let residual = sys.solve(iterations);
		for (instance, &pose) in self.instances.iter_mut().zip(sys.transforms()) {
			instance.pose = pose;
		}
		residual
	}
}

/// Separation between two AABBs (0 when they touch or overlap) — a rigorous
/// LOWER bound on the distance between any two surfaces they contain, used to
/// prune far pairs in [`Assembly::interferences`].
fn aabb_gap(a: Aabb, b: Aabb) -> f64 {
	let mut d2 = 0.0_f64;
	for k in 0..3 {
		let gap = (a.min[k] - b.max[k]).max(b.min[k] - a.max[k]).max(0.0) as f64;
		d2 += gap * gap;
	}
	d2.sqrt()
}

/// Append `src` onto `dst`, rebasing `src`'s indices onto `dst`'s vertices.
fn append_mesh(dst: &mut Mesh, src: &Mesh) {
	let base = dst.positions.len() as u32;
	dst.positions.extend_from_slice(&src.positions);
	dst.normals.extend_from_slice(&src.normals);
	dst.indices.extend(src.indices.iter().map(|&i| i + base));
}

#[cfg(test)]
mod tests {
	use super::*;

	/// Mesh volume of a document at a fixed voxel size.
	fn doc_volume(doc: &Document, vs: f32) -> f64 {
		doc.mesh(Resolution::VoxelSize(vs)).signed_volume()
	}

	/// A 4 × 2 rectangle sketch fully constrained and anchored at the origin, with the
	/// index of its width [`SketchConstraint::Distance`] returned for parametric driving.
	fn rectangle_sketch() -> (Sketch, usize) {
		let mut s = Sketch::new();
		let p0 = s.add_point(kernel_core::math::DVec2::new(0.1, -0.2));
		let p1 = s.add_point(kernel_core::math::DVec2::new(3.0, 0.05));
		let p2 = s.add_point(kernel_core::math::DVec2::new(2.9, 1.8));
		let p3 = s.add_point(kernel_core::math::DVec2::new(-0.1, 2.1));
		s.add_segment(p0, p1);
		s.add_segment(p1, p2);
		s.add_segment(p2, p3);
		s.add_segment(p3, p0);
		s.add_constraint(SketchConstraint::Fixed { point: p0, at: kernel_core::math::DVec2::ZERO });
		s.add_constraint(SketchConstraint::Horizontal { a: p0, b: p1 });
		s.add_constraint(SketchConstraint::Horizontal { a: p3, b: p2 });
		s.add_constraint(SketchConstraint::Vertical { a: p0, b: p3 });
		s.add_constraint(SketchConstraint::Vertical { a: p1, b: p2 });
		let width = s.add_constraint(SketchConstraint::Distance { a: p0, b: p1, distance: 4.0 });
		s.add_constraint(SketchConstraint::Distance { a: p0, b: p3, distance: 2.0 });
		(s, width)
	}

	#[test]
	fn sketch_feature_re_extrudes_when_the_height_parameter_changes() {
		// A sketch-driven extrude in the feature tree: changing the height parameter
		// must re-extrude the 4×2 profile, so the B-rep volume tracks 8 × height.
		let (sketch, _) = rectangle_sketch();
		let mut doc = Document::new();
		doc.set_param("h", 5.0);
		let f = doc.add(Feature::ExtrudeSketch { sketch, height: Dim::param("h"), dims: vec![], draft: Dim::Literal(0.0) });
		doc.set_root(f);

		let vol5 = kernel_brep::volume(&doc.evaluate_brep().expect("sketch extrudes"));
		doc.set_param("h", 10.0);
		let vol10 = kernel_brep::volume(&doc.evaluate_brep().expect("sketch re-extrudes"));

		assert!(
			(vol5 - 40.0).abs() < 1e-6 && (vol10 - 80.0).abs() < 1e-6,
			"parametric extrude: vol(h=5)={vol5} (want 40), vol(h=10)={vol10} (want 80)"
		);
	}

	#[test]
	fn sketch_feature_reshapes_when_a_width_dimension_parameter_changes() {
		// Drive the rectangle's WIDTH distance from a parameter. Editing it must change
		// the solved profile itself (not just height), so the volume tracks width×2×5.
		let (sketch, width) = rectangle_sketch();
		let mut doc = Document::new();
		doc.set_param("w", 4.0);
		let f = doc.add(Feature::ExtrudeSketch {
			sketch,
			height: Dim::Literal(5.0),
			dims: vec![(width, Dim::param("w"))],
			draft: Dim::Literal(0.0),
		});
		doc.set_root(f);

		let vol_w4 = kernel_brep::volume(&doc.evaluate_brep().expect("sketch extrudes"));
		doc.set_param("w", 7.0);
		let vol_w7 = kernel_brep::volume(&doc.evaluate_brep().expect("sketch reshapes"));

		assert!(
			(vol_w4 - 40.0).abs() < 1e-6 && (vol_w7 - 70.0).abs() < 1e-6,
			"parametric width: vol(w=4)={vol_w4} (want 40), vol(w=7)={vol_w7} (want 70)"
		);
	}

	#[test]
	fn sketch_feature_drafts_the_walls_when_a_draft_parameter_is_set() {
		// The draft/taper op reachable end-to-end through the parametric tree: a 4×2
		// sketch extruded 5mm with a 0.05-rad draft slopes the walls inward (a moulded
		// boss). The result is a genus-0 watertight frustum whose volume matches the
		// prismatoid closed form; setting the draft parameter to 0 recovers the plain
		// 8×5 = 40 prism — so draft is a real, re-evaluable feature parameter.
		let (sketch, _) = rectangle_sketch();
		let mut doc = Document::new();
		doc.set_param("h", 5.0);
		doc.set_param("a", 0.05);
		let f = doc.add(Feature::ExtrudeSketch {
			sketch,
			height: Dim::param("h"),
			dims: vec![],
			draft: Dim::param("a"),
		});
		doc.set_root(f);

		let s = doc.evaluate_brep().expect("drafted sketch extrudes");
		let v = kernel_brep::validate(&s);
		let vol = kernel_brep::volume(&s);
		let h = 5.0_f64;
		let d = h * 0.05_f64.tan();
		// Prismatoid of a rectangle drafted by d on every side: bottom 4×2, mid
		// (4−d)×(2−d), top (4−2d)×(2−2d). Relative tol (volume() uses the f32 mesh).
		let prismatoid = h / 6.0 * (8.0 + 4.0 * (4.0 - d) * (2.0 - d) + (4.0 - 2.0 * d) * (2.0 - 2.0 * d));
		doc.set_param("a", 0.0);
		let plain = kernel_brep::volume(&doc.evaluate_brep().expect("draft=0 ⇒ plain prism"));
		assert!(
			v.closed && v.manifold && v.genus == 0 && (vol - prismatoid).abs() / prismatoid < 1e-5 && (plain - 40.0).abs() < 1e-6,
			"drafted extrude: genus-0 frustum vol≈{prismatoid} (got {vol}); draft=0 ⇒ 40 (got {plain}): {v:?}"
		);
	}

	#[test]
	fn instances_mate_by_derived_brep_faces() {
		// Two 2×2×2 cubes. Derive cube A's +Z (top) face and cube B's −Z (bottom)
		// face straight from their B-reps, then mate them face-to-face: instance B
		// (starting far away) must move so its bottom face lands on A's top face.
		let a = kernel_brep::cuboid(DVec3::new(0.0, 0.0, 0.0), DVec3::new(2.0, 2.0, 2.0));
		let b = kernel_brep::cuboid(DVec3::new(0.0, 0.0, 0.0), DVec3::new(2.0, 2.0, 2.0));
		let face_facing = |s: &kernel_brep::Solid, want: DVec3| {
			s.faces().find_map(|f| {
				let (p, n) = s.face_plane(f)?;
				(n.dot(want) > 0.99).then_some((p, n))
			})
		};
		let (pa, na) = face_facing(&a, DVec3::Z).expect("A has a +Z face");
		let (pb, nb) = face_facing(&b, -DVec3::Z).expect("B has a -Z face");

		// Instance 0 (A) is ground at the identity; instance 1 (B) starts offset.
		let mut sys = ConstraintSystem::new(
			vec![Affine3A::IDENTITY, Affine3A::from_translation(Vec3::new(5.0, 4.0, 9.0))],
			vec![],
		);
		sys.add_face_mate(0, pa, na, 1, pb, nb);
		let residual = sys.solve(256);

		// B's bottom-face point, in world, must now meet A's top-face point.
		let wb = sys.transforms()[1].transform_point3(pb.as_vec3());
		assert!(
			residual < 1e-6 && (wb - pa.as_vec3()).length() < 1e-4,
			"derived face mate should seat B's face on A's: residual {residual}, gap {}",
			(wb - pa.as_vec3()).length()
		);
	}

	#[test]
	fn assembly_mass_properties_match_meshing_the_whole_assembly() {
		// Two box parts at rigid poses (one rotated about Z). Summing each part's analytic
		// mass properties through its pose by the parallel-axis theorem (Assembly::
		// mass_properties) must equal meshing the whole assembly exactly and measuring it
		// — volume, center of mass and the full inertia tensor (products included).
		let box_doc = |sx: f64, sy: f64, sz: f64| {
			let mut doc = Document::new();
			let b = doc.add(Feature::Box {
				center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
				size: [Dim::Literal(sx), Dim::Literal(sy), Dim::Literal(sz)],
			});
			doc.set_root(b);
			doc
		};
		let mut asm = Assembly::new();
		asm.add(Instance::document(box_doc(2.0, 2.0, 2.0), Affine3A::from_translation(Vec3::new(-4.0, 0.0, 0.0))));
		asm.add(Instance::document(
			box_doc(4.0, 3.0, 2.0),
			Affine3A::from_translation(Vec3::new(6.0, 2.0, 1.0)) * Affine3A::from_rotation_z(0.6),
		));
		let combined = asm.mass_properties(Resolution::VoxelSize(0.5));
		let whole = asm.mesh_all_exact(1e-4, Resolution::VoxelSize(0.5)).mass_properties();
		let fro2 = |m: DMat3| m.x_axis.length_squared() + m.y_axis.length_squared() + m.z_axis.length_squared();
		let inertia_rel = (fro2(combined.inertia - whole.inertia) / fro2(whole.inertia)).sqrt();
		assert!(
			(combined.volume - whole.volume).abs() / whole.volume < 1e-5
				&& (combined.center_of_mass - whole.center_of_mass).length() / whole.center_of_mass.length() < 1e-5
				&& inertia_rel < 1e-5,
			"assembly combine vs whole-mesh: V {} vs {}, CoM {:?} vs {:?}, inertia rel {inertia_rel}",
			combined.volume,
			whole.volume,
			combined.center_of_mass,
			whole.center_of_mass
		);
	}

	#[test]
	fn chamfered_cylinder_feature_rebuilds_when_the_chamfer_parameter_changes() {
		// The chamfer counterpart of the parametric rounded boss: a bigger 45° top-rim chamfer
		// removes more material, so editing the chamfer parameter shrinks the volume.
		let mut doc = Document::new();
		let f = doc.add(Feature::ChamferedCylinder {
			radius: Dim::Literal(5.0),
			height: Dim::Literal(12.0),
			chamfer: Dim::param("c"),
		});
		doc.set_root(f);
		doc.set_param("c", 1.0);
		let v1 = doc.mass_properties().expect("brep").volume;
		doc.set_param("c", 3.0);
		let v3 = doc.mass_properties().expect("brep").volume;
		assert!(
			v3 < v1 && v1 < std::f64::consts::PI * 25.0 * 12.0,
			"parametric chamfer: c=1 → vol {v1}, c=3 → vol {v3}"
		);
	}

	#[test]
	fn filleted_cylinder_feature_rebuilds_when_the_fillet_parameter_changes() {
		// A parametric rounded boss (curved-edge rim fillet wired into the Document tree): a bigger
		// top-rim fillet removes more material, so editing the fillet parameter shrinks the volume,
		// and it stays below the sharp cylinder πR²h. Proves the torus-fillet feature is parametric.
		let mut doc = Document::new();
		let f = doc.add(Feature::FilletedCylinder {
			radius: Dim::Literal(5.0),
			height: Dim::Literal(12.0),
			fillet: Dim::param("fr"),
		});
		doc.set_root(f);
		doc.set_param("fr", 1.0);
		let v1 = doc.mass_properties().expect("brep").volume;
		doc.set_param("fr", 3.0);
		let v3 = doc.mass_properties().expect("brep").volume;
		let sharp = std::f64::consts::PI * 25.0 * 12.0;
		assert!(
			v3 < v1 && v1 < sharp,
			"parametric rounded boss: fr=1 → vol {v1}, fr=3 → vol {v3} (sharp cyl {sharp})"
		);
	}

	#[test]
	fn document_mass_properties_track_a_parameter_edit() {
		// A parametric box: its mass properties come straight off the Document in one call and
		// update when a width parameter changes — proving parametric mass evaluation (the real
		// "what does my part weigh as I vary a dimension?" workflow) without manual evaluate_brep.
		let mut doc = Document::new();
		let b = doc.add(Feature::Box {
			center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
			size: [Dim::param("w"), Dim::Literal(4.0), Dim::Literal(2.0)],
		});
		doc.set_root(b);
		doc.set_param("w", 3.0);
		let v3 = doc.mass_properties().expect("brep").volume; // 3·4·2 = 24
		doc.set_param("w", 6.0);
		let v6 = doc.mass_properties().expect("brep").volume; // 6·4·2 = 48
		assert!(
			(v3 - 24.0).abs() < 1e-6 && (v6 - 48.0).abs() < 1e-6,
			"parametric mass: w=3 → vol {v3} (want 24), w=6 → vol {v6} (want 48)"
		);
	}

	#[test]
	fn imported_mesh_becomes_an_assembly_part() {
		// An imported / scanned triangle mesh must drop straight into an assembly: lift a box
		// mesh through Instance::from_mesh (Mesh → winding-number SDF → node), place it, and
		// mesh the assembly — the result reproduces the box (bounds and volume) via the
		// mesh→SDF bridge, proving an interchange import becomes a first-class assembly part.
		let box_solid = kernel_brep::cuboid(DVec3::new(-2.0, -2.0, -2.0), DVec3::new(2.0, 2.0, 2.0));
		let box_mesh = kernel_brep::tessellate_default(&box_solid);
		let mut asm = Assembly::new();
		asm.add(Instance::from_mesh(&box_mesh, Affine3A::IDENTITY));
		let out = asm.mesh_all(Resolution::VoxelSize(0.25));
		let aabb = out.aabb();
		let vol = out.signed_volume().abs();
		assert!(
			out.triangle_count() > 0
				&& (aabb.min - Vec3::splat(-2.0)).length() < 0.6
				&& (aabb.max - Vec3::splat(2.0)).length() < 0.6
				&& (vol - 64.0).abs() / 64.0 < 0.15,
			"imported box part: {} tris, aabb {:?}..{:?}, vol {} (want ~64)",
			out.triangle_count(),
			aabb.min,
			aabb.max,
			vol
		);
	}

	#[test]
	fn interference_volume_measures_the_overlap_of_two_boxes() {
		// Two 4³ boxes offset by 2 in x overlap in the slab x∈[0,2] → a 2×4×4 = 32 mm³ shared
		// volume. The voxel-sampled interference volume must recover that — the quantitative
		// clash metric the binary interferences flag can't give.
		let unit_box = || {
			let mut doc = Document::new();
			let b = doc.add(Feature::Box {
				center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
				size: [Dim::Literal(4.0), Dim::Literal(4.0), Dim::Literal(4.0)],
			});
			doc.set_root(b);
			doc
		};
		let mut asm = Assembly::new();
		asm.add(Instance::document(unit_box(), Affine3A::IDENTITY));
		asm.add(Instance::document(unit_box(), Affine3A::from_translation(Vec3::new(2.0, 0.0, 0.0))));
		let v = asm.interference_volume(0, 1, 0.2);
		assert!((v - 32.0).abs() / 32.0 < 0.05, "overlap volume {v} (want ~32)");
	}

	#[test]
	fn assembly_interferences_flag_only_the_overlapping_parts() {
		// Three unit-ish cubes: A at the origin, B shifted +1 in x so it penetrates A, and C
		// far away. Interference detection must flag exactly the A–B clash and report the
		// true 8 mm gap between A and the distant C.
		let unit_box = || {
			let mut doc = Document::new();
			let b = doc.add(Feature::Box {
				center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
				size: [Dim::Literal(2.0), Dim::Literal(2.0), Dim::Literal(2.0)],
			});
			doc.set_root(b);
			doc
		};
		let mut asm = Assembly::new();
		asm.add(Instance::document(unit_box(), Affine3A::IDENTITY)); // A spans x∈[-1,1]
		asm.add(Instance::document(unit_box(), Affine3A::from_translation(Vec3::new(1.0, 0.0, 0.0)))); // B x∈[0,2], overlaps A
		asm.add(Instance::document(unit_box(), Affine3A::from_translation(Vec3::new(10.0, 0.0, 0.0)))); // C x∈[9,11], clear
		let hits = asm.interferences(1e-6, Resolution::VoxelSize(0.2));
		let gap_ac = asm.clearance(0, 2, Resolution::VoxelSize(0.2));
		assert!(
			hits == vec![(0, 1)] && (gap_ac - 8.0).abs() < 0.5,
			"interferences {hits:?} (want [(0,1)]); A–C clearance {gap_ac} (want ~8)"
		);
	}

	#[test]
	fn assembly_checks_see_brep_only_parts() {
		// FRICTION #2 regression: catalog parts (and every other B-rep-only feature)
		// evaluate to None on the implicit half, so clearance/interferences/
		// interference_volume used to see EMPTY instances — `inf` clearance and no
		// clashes, silently, for exactly the parts a gearbox is made of. Three Ø8×20
		// catalog shafts along +Z: A at x=0, B at x=10 (surface gap 2.0 mm), C at
		// x=−6 (overlapping only A, by a 2 mm-deep lens: area 2r²·acos(d/2r) −
		// (d/2)·√(4r²−d²) ≈ 7.25 mm² × 20 mm ≈ 145 mm³ for the 32-gon facets).
		let shaft = || {
			let mut doc = Document::new();
			let s = doc.add(Feature::CatalogPart {
				part: CatalogPart::Shaft { d: Dim::Literal(8.0), length: Dim::Literal(20.0) },
			});
			doc.set_root(s);
			doc
		};
		let mut asm = Assembly::new();
		asm.add(Instance::document(shaft(), Affine3A::IDENTITY));
		asm.add(Instance::document(shaft(), Affine3A::from_translation(Vec3::new(10.0, 0.0, 0.0))));
		asm.add(Instance::document(shaft(), Affine3A::from_translation(Vec3::new(-6.0, 0.0, 0.0))));
		let gap_ab = asm.clearance(0, 1, Resolution::VoxelSize(0.4));
		let hits = asm.interferences(1e-6, Resolution::VoxelSize(0.4));
		let overlap_ac = asm.interference_volume(0, 2, 0.2);
		let prox = asm.proximity_pairs(3.0, 0.05, Resolution::VoxelSize(0.4));
		let prox_ok = prox.len() == 2
			&& prox[0].0 == 0 && prox[0].1 == 1 && (prox[0].2 - 2.0).abs() < 0.1
			&& prox[1].0 == 0 && prox[1].1 == 2 && prox[1].2 <= 1e-9;
		assert!(
			(gap_ab - 2.0).abs() < 0.1 && hits == vec![(0, 2)] && (overlap_ac - 145.0).abs() / 145.0 < 0.1 && prox_ok,
			"B-rep-only parts must be visible to the assembly checks (used to be inf/none/0 silently): \
			 A–B clearance {gap_ab} (want ~2.0), interferences {hits:?} (want [(0, 2)]), \
			 A–C overlap {overlap_ac} mm³ (want ~145), proximity {prox:?} (want [(0,1,~2.0), (0,2,0.0)])"
		);
	}

	#[test]
	fn threaded_bolt_thread_adds_material_and_stays_watertight() {
		// Regression guard for the showcase bolt: a helical thread fused onto the shank at
		// the MESH level (its exact B-rep union self-intersects) must (a) keep the part
		// watertight and (b) ADD material vs the bare shank — the exact symptom that was
		// silently broken when the thread was dropped on a failed B-rep validity check.
		use std::f64::consts::TAU;
		let shank = kernel_brep::cylinder(DVec3::ZERO, DVec3::Z, 4.0, 20.0, 48);

		// A triangular thread crest swept along a helix climbing the shank.
		let (pitch, turns, steps) = (2.4_f64, 5.0_f64, 32usize);
		let n = (turns * steps as f64) as usize;
		let path: Vec<DVec3> = (0..=n)
			.map(|k| {
				let t = k as f64 / steps as f64;
				let a = t * TAU;
				DVec3::new(4.0 * a.cos(), 4.0 * a.sin(), 2.0 + t * pitch)
			})
			.collect();
		let hw = pitch * 0.25; // ridge ~half the pitch, leaving wide valleys that mesh watertight
		// Wound so the sweep's outward normals point away from the helix (positive volume);
		// the reverse order makes the sweep inside-out, which would carve a groove instead
		// of adding a thread ridge in the winding-number heal.
		let profile = vec![DVec3::new(4.0, 0.0, 2.0 + hw), DVec3::new(4.9, 0.0, 2.0), DVec3::new(4.0, 0.0, 2.0 - hw)];
		let thread = kernel_brep::sweep_solid(&profile, &path).expect("thread sweeps");
		assert!(kernel_brep::volume(&thread) > 0.0, "thread sweep should be outward (vol {})", kernel_brep::volume(&thread));

		let merge = |soup: &mut Mesh, src: &Mesh| {
			let base = soup.positions.len() as u32;
			for p in &src.positions {
				soup.positions.push(*p);
			}
			for t in src.triangles() {
				soup.push_triangle(base + t[0], base + t[1], base + t[2]);
			}
		};
		let shank_tess = kernel_brep::tessellate_default(&shank);
		let mut bolt_soup = shank_tess.clone();
		merge(&mut bolt_soup, &kernel_brep::tessellate_default(&thread));

		// Heal both at the same voxel size; comparing volumes is robust to voxel noise.
		let plain = watertight_mesh_of(&shank_tess, 0.25);
		let threaded = watertight_mesh_of(&bolt_soup, 0.25);
		assert!(
			plain.is_watertight() && threaded.is_watertight() && threaded.signed_volume() > plain.signed_volume() + 5.0,
			"thread must stay watertight and add material: plain_vol={} threaded_vol={}",
			plain.signed_volume(),
			threaded.signed_volume()
		);
	}

	#[test]
	fn precise_mesh_is_exact_and_watertight_for_curved_solids() {
		// The precision AI path. A STANDALONE cylinder meshes micron-fine via the EXACT analytic
		// tessellation: every lateral chord lies within ~the tolerance of the true radius, no
		// voxel grid. A drilled plate (box − cylinder bore) meshes WATERTIGHT and fine the same
		// way — but its bore wall inherits the boolean's construction resolution (curved boolean
		// walls are not yet re-fitted to the analytic surface; tracked), so the micron chord
		// bound is asserted only on the standalone primitive, honestly.
		let plate = kernel_brep::difference(
			&kernel_brep::cuboid(DVec3::new(-10.0, -10.0, -5.0), DVec3::new(10.0, 10.0, 5.0)),
			&kernel_brep::cylinder(DVec3::new(0.0, 0.0, -6.0), DVec3::Z, 4.0, 12.0, 48),
		);
		let cyl = kernel_brep::cylinder(DVec3::ZERO, DVec3::Z, 5.0, 12.0, 24);
		let mp = precise_mesh(&plate, 0.005);
		let mc = precise_mesh(&cyl, 0.005);
		// Chord deviation of the standalone cylinder's lateral wall (vertices on radius 5) from
		// the true surface: the midpoint of each wall chord must sit within ~tol of radius 5.
		let mut max_dev = 0.0f64;
		for t in mc.indices.chunks_exact(3) {
			let p = [
				mc.positions[t[0] as usize].as_dvec3(),
				mc.positions[t[1] as usize].as_dvec3(),
				mc.positions[t[2] as usize].as_dvec3(),
			];
			let on_cyl = p.iter().all(|v| ((v.x * v.x + v.y * v.y).sqrt() - 5.0).abs() < 1e-2);
			// Exclude the flat caps (all three vertices on one z-plane); their rim-spanning
			// chords cut across the disk and are not a measure of curved-wall fidelity.
			let on_cap = p.iter().all(|v| v.z.abs() < 1e-3) || p.iter().all(|v| (v.z - 12.0).abs() < 1e-3);
			if on_cyl && !on_cap {
				for &(i, j) in &[(0, 1), (1, 2), (2, 0)] {
					let mid = (p[i] + p[j]) * 0.5;
					max_dev = max_dev.max(5.0 - (mid.x * mid.x + mid.y * mid.y).sqrt());
				}
			}
		}
		assert!(
			mp.is_watertight() && mp.triangle_count() > 1000 && mc.is_watertight() && mc.triangle_count() > 400 && max_dev > 0.0 && max_dev <= 0.005 * 1.5,
			"precise_mesh: plate wt={} tris={}, cyl wt={} tris={}, cyl chord_dev={max_dev} (want 0 < dev ≤ {})",
			mp.is_watertight(),
			mp.triangle_count(),
			mc.is_watertight(),
			mc.triangle_count(),
			0.005 * 1.5
		);
	}

	#[test]
	fn watertight_mesh_of_fuses_self_intersecting_soup() {
		// Two overlapping boxes as raw triangle SOUP (no valid B-rep union between them)
		// heal into ONE watertight solid via the winding-number field — the move that lets
		// a self-intersecting helical thread fuse onto a bolt shank. Material is the union,
		// so the volume exceeds either box yet is less than their disjoint sum.
		let mut soup = kernel_brep::tessellate_default(&kernel_brep::cuboid(DVec3::new(-2.0, -2.0, -2.0), DVec3::new(2.0, 2.0, 2.0)));
		let b = kernel_brep::tessellate_default(&kernel_brep::cuboid(DVec3::new(0.0, 0.0, 0.0), DVec3::new(4.0, 4.0, 4.0)));
		let base = soup.positions.len() as u32;
		for p in &b.positions {
			soup.positions.push(*p);
		}
		for t in b.triangles() {
			soup.push_triangle(base + t[0], base + t[1], base + t[2]);
		}
		let healed = watertight_mesh_of(&soup, 0.2);
		let v = healed.signed_volume();
		assert!(
			healed.is_watertight() && v > 64.0 && v < 128.0,
			"fused soup must be a watertight union (64..128 mm³): wt={} vol={}",
			healed.is_watertight(),
			v
		);
	}

	#[test]
	fn voxel_path_unions_a_tilted_box_watertight() {
		// The HYBRID point: the B-rep boolean used to choke on tilted / face-sharing
		// boxes, but the voxel/SDF path (min/max on signed distances + Manifold Dual
		// Contouring) is robust to them — a tilted wall unioned onto a base meshes
		// watertight regardless. This is what makes the hybrid stronger than either half.
		let mut doc = Document::new();
		let base = doc.add(Feature::Box {
			center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(4.0)],
			size: [Dim::Literal(80.0), Dim::Literal(70.0), Dim::Literal(8.0)],
		});
		let wall = doc.add(Feature::Box {
			center: [Dim::Literal(0.0), Dim::Literal(23.0), Dim::Literal(40.0)],
			size: [Dim::Literal(80.0), Dim::Literal(8.0), Dim::Literal(80.0)],
		});
		let tilted = doc.add(Feature::Transform { input: wall, xform: Affine3A::from_axis_angle(Vec3::X, 12.0_f32.to_radians()) });
		let u = doc.add(Feature::Boolean { op: BooleanOp::Union, a: base, b: tilted });
		doc.set_root(u);

		let mesh = doc.mesh(Resolution::VoxelSize(2.0));
		assert!(
			mesh.is_watertight() && mesh.signed_volume() > 0.0,
			"voxel union of a tilted box must mesh watertight: watertight={}, vol={}",
			mesh.is_watertight(),
			mesh.signed_volume()
		);
	}

	#[test]
	fn shell_hollows_a_box_into_a_watertight_wall() {
		// The voxel-half SHELL op: hollow a solid into a thin wall, preserving outer
		// dimensions. A 10-cube shelled to a 1-thick wall keeps material 10³ − 8³ = 488,
		// and the two nested surfaces mesh watertight — a job the SDF half does robustly
		// while the exact B-rep half (no general face-offset) returns None.
		let mut doc = Document::new();
		let b = doc.add(Feature::Box {
			center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
			size: [Dim::Literal(10.0), Dim::Literal(10.0), Dim::Literal(10.0)],
		});
		let sh = doc.add(Feature::Shell { input: b, thickness: Dim::Literal(1.0) });
		doc.set_root(sh);

		let mesh = doc.mesh(Resolution::VoxelSize(0.25));
		let wall = 10.0_f64.powi(3) - 8.0_f64.powi(3); // 488: outer minus inner cavity
		assert!(
			mesh.is_watertight() && (mesh.signed_volume() - wall).abs() / wall < 0.1,
			"shelled box must be a watertight {wall}-volume wall: wt={} vol={}",
			mesh.is_watertight(),
			mesh.signed_volume()
		);
		// And the shell is voxel-half-only: the exact B-rep path has no shell yet.
		assert!(doc.evaluate_brep().is_none(), "shell must be absent on the B-rep path");
	}

	#[test]
	fn smooth_union_blends_spheres_into_a_watertight_organic_solid() {
		// The signature ORGANIC workflow: three overlapping spheres smooth-unioned into a
		// metaball-style blob through the parametric tree. The voxel/SDF half meshes the
		// filleted junctions watertight (a hard union would leave sharp creases), and the
		// blend fuses them into one solid that is bigger than a single sphere yet smaller
		// than three disjoint ones.
		let mut doc = Document::new();
		let r = 5.0;
		let s0 = doc.add(Feature::Sphere { center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)], radius: Dim::Literal(r) });
		let s1 = doc.add(Feature::Sphere { center: [Dim::Literal(6.0), Dim::Literal(0.0), Dim::Literal(0.0)], radius: Dim::Literal(r) });
		let s2 = doc.add(Feature::Sphere { center: [Dim::Literal(3.0), Dim::Literal(5.0), Dim::Literal(0.0)], radius: Dim::Literal(r) });
		let b01 = doc.add(Feature::SmoothUnion { a: s0, b: s1, blend: Dim::Literal(2.0) });
		let blob = doc.add(Feature::SmoothUnion { a: b01, b: s2, blend: Dim::Literal(2.0) });
		doc.set_root(blob);

		let mesh = doc.mesh(Resolution::VoxelSize(0.4));
		let sphere_vol = 4.0 / 3.0 * std::f64::consts::PI * r * r * r; // ≈ 523.6
		let v = mesh.signed_volume();
		assert!(
			mesh.is_watertight() && v > 1.2 * sphere_vol && v < 3.0 * sphere_vol,
			"smooth-union blob must be a watertight organic solid (1.2..3 spheres): wt={} vol={} (sphere {sphere_vol})",
			mesh.is_watertight(),
			v
		);
		// Voxel-half-only: there is no exact analytic blend on the B-rep path.
		assert!(doc.evaluate_brep().is_none(), "smooth union must be absent on the B-rep path");
	}

	#[test]
	fn smooth_difference_carves_a_watertight_organic_pocket() {
		// The organic CARVE workflow: a sphere smooth-subtracted from a box leaves a
		// rounded crater (a filleted pocket, not a sharp dimple). The voxel half meshes
		// it watertight, and material is removed so the result is strictly less than the
		// 20×20×10 = 4000 box yet keeps most of the block.
		let mut doc = Document::new();
		let block = doc.add(Feature::Box {
			center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
			size: [Dim::Literal(20.0), Dim::Literal(20.0), Dim::Literal(10.0)],
		});
		let tool = doc.add(Feature::Sphere { center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(5.0)], radius: Dim::Literal(4.0) });
		let carved = doc.add(Feature::SmoothDifference { a: block, b: tool, blend: Dim::Literal(1.5) });
		doc.set_root(carved);

		let mesh = doc.mesh(Resolution::VoxelSize(0.4));
		let v = mesh.signed_volume();
		assert!(
			mesh.is_watertight() && v < 4000.0 && v > 3000.0,
			"smooth-difference pocket must be a watertight carved block (3000..4000): wt={} vol={}",
			mesh.is_watertight(),
			v
		);
		assert!(doc.evaluate_brep().is_none(), "smooth difference must be absent on the B-rep path");
	}

	#[test]
	fn smooth_intersection_of_two_spheres_is_a_watertight_lens() {
		// Smooth intersection keeps the rounded common volume of two overlapping spheres
		// (a lens), meshed watertight, smaller than either sphere yet non-empty.
		let mut doc = Document::new();
		let r = 5.0;
		let a = doc.add(Feature::Sphere { center: [Dim::Literal(-2.0), Dim::Literal(0.0), Dim::Literal(0.0)], radius: Dim::Literal(r) });
		let b = doc.add(Feature::Sphere { center: [Dim::Literal(2.0), Dim::Literal(0.0), Dim::Literal(0.0)], radius: Dim::Literal(r) });
		let lens = doc.add(Feature::SmoothIntersection { a, b, blend: Dim::Literal(1.0) });
		doc.set_root(lens);

		let mesh = doc.mesh(Resolution::VoxelSize(0.3));
		let sphere_vol = 4.0 / 3.0 * std::f64::consts::PI * r * r * r;
		let v = mesh.signed_volume();
		assert!(
			mesh.is_watertight() && v > 0.0 && v < sphere_vol,
			"smooth-intersection lens must be a watertight solid (0..one sphere {sphere_vol}): wt={} vol={}",
			mesh.is_watertight(),
			v
		);
	}

	#[test]
	fn gyroid_feature_meshes_a_bounded_lattice_infill() {
		// TPMS lattice infill reachable END-TO-END as a Feature (the additive-
		// manufacturing workflow): a gyroid bounded to its box → a rich, in-bounds,
		// plausibly-sized lattice block via Document::mesh. HONEST: a TPMS shell has
		// saddle pinches, so the lattice is rich + closed but not guaranteed fully
		// watertight — we assert the same rich/bounded properties as the kernel-implicit
		// gyroid test, not watertightness.
		let mut doc = Document::new();
		let half = 20.0;
		let g = doc.add(Feature::Gyroid {
			center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
			size: [Dim::Literal(2.0 * half), Dim::Literal(2.0 * half), Dim::Literal(2.0 * half)],
			scale: Dim::Literal(0.35),
			thickness: Dim::Literal(0.30),
		});
		doc.set_root(g);

		let mesh = doc.mesh(Resolution::VoxelSize(0.8));
		let vol = mesh.signed_volume();
		let cube_vol = 8.0 * half * half * half;
		let bb = mesh.aabb();
		assert!(
			mesh.triangle_count() > 5000 && vol > 0.01 * cube_vol && vol < 0.6 * cube_vol && bb.min.x >= -(half as f32) - 1.0 && bb.max.x <= half as f32 + 1.0,
			"gyroid feature must mesh a rich bounded lattice: tris={} vol={} (cube {cube_vol})",
			mesh.triangle_count(),
			vol
		);
		assert!(doc.evaluate_brep().is_none(), "gyroid is voxel-half-only on the B-rep path");
	}

	#[test]
	fn smooth_union_blend_radius_is_a_live_parameter() {
		// The blend radius is a real re-evaluable parameter: increasing it fuses the two
		// overlapping spheres more, adding fillet material, so the meshed volume grows.
		let mut doc = Document::new();
		let r = 5.0;
		let a = doc.add(Feature::Sphere { center: [Dim::Literal(-4.0), Dim::Literal(0.0), Dim::Literal(0.0)], radius: Dim::Literal(r) });
		let b = doc.add(Feature::Sphere { center: [Dim::Literal(4.0), Dim::Literal(0.0), Dim::Literal(0.0)], radius: Dim::Literal(r) });
		let u = doc.add(Feature::SmoothUnion { a, b, blend: Dim::param("k") });
		doc.set_root(u);

		doc.set_param("k", 0.5);
		let v_small = doc.mesh(Resolution::VoxelSize(0.4)).signed_volume();
		doc.set_param("k", 4.0);
		let v_big = doc.mesh(Resolution::VoxelSize(0.4)).signed_volume();
		assert!(
			v_small > 0.0 && v_big > v_small,
			"larger blend radius must add fillet material: v(k=0.5)={v_small} v(k=4)={v_big}"
		);
	}

	#[test]
	fn gyroid_thickness_is_a_live_parameter() {
		// Infill density is editable end-to-end: the gyroid wall thickness is a
		// re-evaluable parameter, so increasing it thickens the lattice walls and adds
		// material (the same parametric story as the blend radius, for the lattice).
		let mut doc = Document::new();
		let half = 16.0;
		let g = doc.add(Feature::Gyroid {
			center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
			size: [Dim::Literal(2.0 * half), Dim::Literal(2.0 * half), Dim::Literal(2.0 * half)],
			scale: Dim::Literal(0.35),
			thickness: Dim::param("t"),
		});
		doc.set_root(g);

		doc.set_param("t", 0.2);
		let v_thin = doc.mesh(Resolution::VoxelSize(0.8)).signed_volume();
		doc.set_param("t", 0.5);
		let v_thick = doc.mesh(Resolution::VoxelSize(0.8)).signed_volume();
		assert!(
			v_thin > 0.0 && v_thick > v_thin,
			"thicker lattice walls must add material: vol(t=0.2)={v_thin} vol(t=0.5)={v_thick}"
		);
	}

	#[test]
	fn gyroid_infills_a_part_via_intersection() {
		// The advertised infill workflow: intersect a gyroid lattice with a part to fill
		// it with lattice. A gyroid (bounded to a box containing the sphere) ∩ a sphere →
		// a lattice-filled ball: a rich mesh, non-empty, inside the sphere, with strictly
		// less material than the solid sphere.
		let mut doc = Document::new();
		let r = 12.0;
		let lattice = doc.add(Feature::Gyroid {
			center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
			size: [Dim::Literal(2.0 * r), Dim::Literal(2.0 * r), Dim::Literal(2.0 * r)],
			scale: Dim::Literal(0.4),
			thickness: Dim::Literal(0.35),
		});
		let part = doc.add(Feature::Sphere { center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)], radius: Dim::Literal(r) });
		let infilled = doc.add(Feature::Boolean { op: BooleanOp::Intersection, a: lattice, b: part });
		doc.set_root(infilled);

		let mesh = doc.mesh(Resolution::VoxelSize(0.6));
		let v = mesh.signed_volume();
		let sphere_vol = 4.0 / 3.0 * std::f64::consts::PI * r * r * r;
		let bb = mesh.aabb();
		assert!(
			mesh.triangle_count() > 2000 && v > 0.0 && v < sphere_vol && bb.min.x >= -(r as f32) - 1.0 && bb.max.x <= r as f32 + 1.0,
			"gyroid-infilled sphere must be a rich bounded lattice inside the sphere: tris={} vol={} (sphere {sphere_vol})",
			mesh.triangle_count(),
			v
		);
	}

	#[test]
	fn document_watertight_brep_mesh_heals_a_curved_part() {
		// A parametric block with a cylindrical hole, meshed watertight in one call
		// through the document's B-rep + hybrid heal — the AI-facing one-shot path.
		let mut doc = Document::new();
		doc.set_param("r", 4.0);
		let block = doc.add(Feature::Box {
			center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
			size: [Dim::Literal(20.0), Dim::Literal(20.0), Dim::Literal(10.0)],
		});
		let hole = doc.add(Feature::Cylinder {
			center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
			radius: Dim::param("r"),
			height: Dim::Literal(14.0),
		});
		let part = doc.add(Feature::Boolean { op: BooleanOp::Difference, a: block, b: hole });
		doc.set_root(part);

		let mesh = doc.watertight_brep_mesh(1.0);
		let exact = 20.0 * 20.0 * 10.0 - std::f64::consts::PI * 16.0 * 10.0;
		assert!(
			mesh.is_watertight() && (mesh.signed_volume() - exact).abs() / exact < 0.08,
			"document B-rep heal should be watertight with plausible volume: wt={} vol={} (exact {exact})",
			mesh.is_watertight(),
			mesh.signed_volume()
		);
	}

	#[test]
	fn curved_boolean_meshes_watertight_both_exactly_and_via_voxel_heal() {
		// A hex nut (hex prism − a cylindrical hole). The EXACT B-rep tessellation now meshes
		// this watertight DIRECTLY: the robust ear-clipper honours the boolean's near-collinear
		// annular rim instead of skipping a point into an overlapping sliver (see
		// brep_validity::boolean_annular_cap_tessellates_watertight_via_exact_path). The hybrid
		// VOXEL heal (tessellate → MeshSdf winding field → Manifold Dual Contouring) is the
		// robust fallback and must AGREE: it also returns a watertight mesh of the same volume,
		// so a part the exact path cannot close (a self-intersecting feature) still meshes —
		// that genuinely-non-watertight case is covered by watertight_mesh_of_fuses_self_intersecting_soup.
		let r = 7.5;
		let hex: Vec<kernel_brep::math::DVec2> = (0..6)
			.map(|i| {
				let a = std::f64::consts::PI / 6.0 + i as f64 * std::f64::consts::PI / 3.0;
				kernel_brep::math::DVec2::new(r * a.cos(), r * a.sin())
			})
			.collect();
		let prism = kernel_brep::extrude(&hex, 6.0);
		let hole = kernel_brep::cylinder(DVec3::new(0.0, 0.0, -1.0), DVec3::Z, 4.2, 8.0, 48);
		let nut = kernel_brep::difference(&prism, &hole);

		let raw = kernel_brep::tessellate_default(&nut);
		let healed = watertight_mesh(&nut, 1.0);
		// hex area (3√3/2)r² × height − cylinder π·4.2²·6.
		let exact = 1.5 * 3.0_f64.sqrt() * r * r * 6.0 - std::f64::consts::PI * 4.2 * 4.2 * 6.0;
		assert!(
			raw.is_watertight()
				&& healed.is_watertight()
				&& (raw.signed_volume() - exact).abs() / exact < 0.01
				&& (healed.signed_volume() - exact).abs() / exact < 0.08,
			"curved nut should mesh watertight both exactly and via heal: raw_wt={} raw_vol={} healed_wt={} healed_vol={} (exact {exact})",
			raw.is_watertight(),
			raw.signed_volume(),
			healed.is_watertight(),
			healed.signed_volume()
		);
	}

	#[test]
	fn parametric_fillet_survives_a_split_edge_with_a_witness() {
		// A bar unioned across the top of a box splits some of the box's named edges
		// into collinear fragments sharing one EdgeName. Filleting such an edge WITHOUT
		// a witness fails (ambiguous); WITH a witness the parametric fillet picks the
		// nearest fragment and succeeds — so a named fillet survives an edit that splits
		// its edge instead of breaking the feature tree.
		let mut doc = Document::new();
		let a = doc.add(Feature::Box {
			center: [Dim::Literal(5.0), Dim::Literal(5.0), Dim::Literal(5.0)],
			size: [Dim::Literal(10.0), Dim::Literal(10.0), Dim::Literal(10.0)],
		});
		let bar = doc.add(Feature::Box {
			center: [Dim::Literal(5.0), Dim::Literal(5.0), Dim::Literal(11.0)],
			size: [Dim::Literal(14.0), Dim::Literal(4.0), Dim::Literal(6.0)],
		});
		let u = doc.add(Feature::Boolean { op: BooleanOp::Union, a, b: bar });
		doc.set_root(u);
		let solid = doc.evaluate_brep().expect("union evaluates");

		// The witness feature is the *ambiguity resolution*, which is deterministic: a split
		// name resolves to >1 fragments so `fillet_edge` reports EdgeAmbiguous, while a witness
		// selects the nearest single fragment (so the `_near` resolver never reports
		// EdgeAmbiguous). We assert that contrast on this one build — NOT the geometric round
		// outcome, which is not yet bit-reproducible across boolean rebuilds (a frontier item).
		use kernel_brep::FilletError;
		let mut counts: std::collections::BTreeMap<String, kernel_brep::EdgeName> = std::collections::BTreeMap::new();
		let mut seen: std::collections::BTreeMap<String, usize> = std::collections::BTreeMap::new();
		for e in solid.edges() {
			if let Some(n) = solid.edge_name(e) {
				let k = format!("{n:?}");
				*seen.entry(k.clone()).or_insert(0) += 1;
				counts.entry(k).or_insert(n);
			}
		}
		// First split name in deterministic (sorted) order.
		let split = seen
			.iter()
			.find(|(_, &c)| c > 1)
			.map(|(k, _)| counts[k])
			.expect("the box+bar union splits at least one named edge into fragments");

		// Without a witness the kernel reports the split edge ambiguous …
		assert!(
			matches!(kernel_brep::fillet_edge(&solid, split, 0.4), Err(FilletError::EdgeAmbiguous)),
			"a split edge name must be reported EdgeAmbiguous without a witness"
		);
		// … and a witness resolves it to a single fragment — never EdgeAmbiguous — for every
		// witness near the part (the nearest-fragment pick always disambiguates).
		let witnesses = [DVec3::new(0.0, 5.0, 10.0), DVec3::new(10.0, 5.0, 10.0), DVec3::ZERO, DVec3::splat(10.0)];
		let all_resolve = witnesses
			.iter()
			.all(|&wp| !matches!(kernel_brep::fillet_edge_near(&solid, split, 0.4, wp), Err(FilletError::EdgeAmbiguous)));
		assert!(all_resolve, "a witness must resolve the ambiguous split edge to one fragment");
	}

	#[test]
	fn assembly_mesh_all_exact_keeps_brep_parts_crisp_not_voxelized() {
		// A placed assembly of B-rep parts meshes via the EXACT analytic tessellation, not the
		// voxel grid: two 4 mm boxes placed apart come out as exactly 2×12 = 24 crisp triangles
		// (a box is 12 tris), whereas the voxel mesh_all quantizes each into many more. Every
		// vertex is finite. This is what keeps a machined-component assembly micron-sharp.
		let unit_box = || {
			let mut d = Document::new();
			d.add(Feature::Box {
				center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
				size: [Dim::Literal(4.0), Dim::Literal(4.0), Dim::Literal(4.0)],
			});
			d
		};
		let mut asm = Assembly::new();
		asm.add(Instance::document(unit_box(), Affine3A::from_translation(Vec3::new(-5.0, 0.0, 0.0))));
		asm.add(Instance::document(unit_box(), Affine3A::from_translation(Vec3::new(5.0, 0.0, 0.0))));
		let exact = asm.mesh_all_exact(0.005, Resolution::VoxelSize(0.5));
		let voxel = asm.mesh_all(Resolution::VoxelSize(0.5));
		assert!(
			exact.triangle_count() == 24 && exact.positions.iter().all(|p| p.is_finite()) && voxel.triangle_count() > exact.triangle_count(),
			"exact assembly must be 24 crisp tris (2 boxes), not voxelized: exact={} voxel={}",
			exact.triangle_count(),
			voxel.triangle_count()
		);
	}

	#[test]
	fn assembly_mates_two_parts_through_solve_mates() {
		// End-to-end through the Assembly API: two 2×2×2 cube parts, A grounded and B
		// placed far away. Derive A's +Z face and B's −Z face from their B-reps, mate
		// them face-to-face, and solve_mates → B's pose moves so its bottom face seats
		// on A's top face, and mesh_all reflects the moved part.
		let a_solid = kernel_brep::cuboid(DVec3::new(-1.0, -1.0, -1.0), DVec3::new(1.0, 1.0, 1.0));
		let b_solid = kernel_brep::cuboid(DVec3::new(-1.0, -1.0, -1.0), DVec3::new(1.0, 1.0, 1.0));
		let face = |s: &kernel_brep::Solid, want: DVec3| {
			s.faces().find_map(|f| {
				let (p, n) = s.face_plane(f)?;
				(n.dot(want) > 0.99).then_some((p, n))
			})
		};
		let (pa, na) = face(&a_solid, DVec3::Z).expect("A +Z face");
		let (pb, nb) = face(&b_solid, -DVec3::Z).expect("B -Z face");

		let cube = || Node::primitive(Cuboid::new(Vec3::ZERO, Vec3::splat(1.0)));
		let mut asm = Assembly::new();
		asm.add(Instance::node(cube(), Affine3A::IDENTITY)); // 0 = ground
		asm.add(Instance::node(cube(), Affine3A::from_translation(Vec3::new(5.0, 4.0, 9.0))));

		let residual = asm.solve_mates(
			&[
				Constraint::Coincident { a: 0, a_point: pa, b: 1, b_point: pb },
				Constraint::Parallel { a: 0, a_dir: na, b: 1, b_dir: nb },
			],
			256,
		);

		let world_b = asm.instances[1].pose.transform_point3(pb.as_vec3());
		let mesh = asm.mesh_all(Resolution::VoxelSize(0.5));
		assert!(
			residual < 1e-6 && (world_b - pa.as_vec3()).length() < 1e-4 && !mesh.is_empty(),
			"solve_mates should seat B on A and mesh: residual {residual}, gap {}, tris {}",
			(world_b - pa.as_vec3()).length(),
			mesh.triangle_count()
		);
	}

	#[test]
	fn instances_mate_coaxial_by_derived_cylinder_axes() {
		// A shaft and a sleeve, each a cylinder along Z. Read each one's axis straight
		// off its B-rep (a lateral cylindrical face), then concentric-mate them: the
		// misaligned sleeve must rotate + translate so its axis is collinear with the
		// shaft's (the Z axis).
		let shaft = kernel_brep::cylinder(DVec3::ZERO, DVec3::Z, 2.0, 10.0, 32);
		let sleeve = kernel_brep::cylinder(DVec3::ZERO, DVec3::Z, 3.0, 6.0, 32);
		let axis_of = |s: &kernel_brep::Solid| s.faces().find_map(|f| s.face_axis(f));
		let (pa, da) = axis_of(&shaft).expect("shaft has a cylindrical face");
		let (pb, db) = axis_of(&sleeve).expect("sleeve has a cylindrical face");

		let mut sys = ConstraintSystem::new(
			vec![
				Affine3A::IDENTITY,
				Affine3A::from_translation(Vec3::new(3.0, 4.0, 5.0)) * Affine3A::from_axis_angle(Vec3::Y, 0.6),
			],
			vec![],
		);
		sys.add_axis_mate(0, pa, da, 1, pb, db);
		let residual = sys.solve(256);

		let pose = sys.transforms()[1];
		let b_dir = pose.transform_vector3(db.as_vec3()).normalize_or_zero().as_dvec3();
		let b_pt = pose.transform_point3(pb.as_vec3()).as_dvec3();
		let parallel = da.cross(b_dir).length();
		let rel = b_pt - pa;
		let offset = (rel - da * rel.dot(da)).length();
		assert!(
			residual < 1e-6 && parallel < 1e-4 && offset < 1e-4,
			"coaxial mate should make the axes collinear: residual {residual}, parallel {parallel}, offset {offset}"
		);
	}

	#[test]
	fn touching_linear_pattern_fuses_into_one_solid() {
		// Pattern step EQUALS the cube size, so adjacent copies SHARE a face (touch,
		// not gap). Thanks to the coplanar boolean fix the four copies fuse into a
		// SINGLE solid — a 4×1×1 bar of volume 4, one shell — instead of fragmenting.
		let mut doc = Document::new();
		let cube = doc.add(Feature::Box {
			center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
			size: [Dim::Literal(1.0), Dim::Literal(1.0), Dim::Literal(1.0)],
		});
		let bar = doc.add(Feature::LinearPattern {
			input: cube,
			count: 4,
			step: [Dim::Literal(1.0), Dim::Literal(0.0), Dim::Literal(0.0)],
		});
		doc.set_root(bar);

		let solid = doc.evaluate_brep().expect("touching pattern evaluates");
		let v = kernel_brep::validate(&solid);
		assert!(
			v.is_valid() && v.shells == 1 && (kernel_brep::volume(&solid).abs() - 4.0).abs() < 1e-6,
			"touching pattern should fuse into one bar (1 shell, vol 4): {v:?} vol={}",
			kernel_brep::volume(&solid).abs()
		);
	}

	#[test]
	fn curved_circular_pattern_of_cylinders_is_exact_via_brep() {
		// A circular pattern of DISJOINT cylinders (a bolt-circle hole pattern, pegs on a ring) now
		// builds EXACTLY via the B-rep: the copies are AABB-disjoint, so they merge by topology
		// (disjoint_union) instead of chaining boolean unions — which used to self-intersect and
		// corrupt the volume (e.g. 6 disjoint cylinders unioned read ~23% low). Six Ø4×4 pegs at
		// radius 15 → a valid 6-shell solid, free of self-intersection, of volume 6·π·2²·4.
		let lit3 = |x: f64, y: f64, z: f64| [Dim::Literal(x), Dim::Literal(y), Dim::Literal(z)];
		let mut d = Document::new();
		let peg = d.add(Feature::Cylinder { center: lit3(15.0, 0.0, 0.0), radius: Dim::Literal(2.0), height: Dim::Literal(4.0) });
		let ring = d.add(Feature::CircularPattern {
			input: peg,
			count: 6,
			axis_point: lit3(0.0, 0.0, 0.0),
			axis_dir: lit3(0.0, 0.0, 1.0),
			angle: Dim::Literal(std::f64::consts::TAU / 6.0),
		});
		d.set_root(ring);
		let solid = d.evaluate_brep().expect("circular pattern of cylinders evaluates");
		let v = kernel_brep::validate(&solid);
		let expected = 6.0 * std::f64::consts::PI * 2.0 * 2.0 * 4.0;
		assert!(
			v.is_valid() && v.shells == 6 && !kernel_brep::self_intersects(&solid) && (kernel_brep::volume(&solid).abs() - expected).abs() / expected < 0.03,
			"curved circular pattern must be exact (valid 6-shell, no self-int, vol ~{expected:.0}): {v:?} self_int={} vol={:.0}",
			kernel_brep::self_intersects(&solid),
			kernel_brep::volume(&solid).abs()
		);
	}

	#[test]
	fn curved_circular_pattern_bolt_circle_is_watertight_and_correct_via_voxel() {
		// A bolt circle — a plate with a CIRCULAR PATTERN of cylindrical holes — is a ubiquitous
		// part. Its exact B-rep is NOT reliable here: a pattern chains boolean unions of the
		// copies, and chained unions of CURVED operands self-intersect (the result passes
		// validate()'s closed/manifold/genus checks but is geometrically corrupt — `self_intersects`
		// is true and the volume is far off). The robust route is the VOXEL/SDF half: Document::mesh
		// heals it into a watertight solid of the correct volume. Six Ø5 holes on a 40×40×6 plate →
		// plate 9600 − 6·π·2.5²·6 ≈ 8893 mm³.
		let lit3 = |x: f64, y: f64, z: f64| [Dim::Literal(x), Dim::Literal(y), Dim::Literal(z)];
		let mut d = Document::new();
		let plate = d.add(Feature::Box { center: lit3(0.0, 0.0, 0.0), size: lit3(40.0, 40.0, 6.0) });
		let hole = d.add(Feature::Cylinder { center: lit3(15.0, 0.0, 0.0), radius: Dim::Literal(2.5), height: Dim::Literal(8.0) });
		let holes = d.add(Feature::CircularPattern {
			input: hole,
			count: 6,
			axis_point: lit3(0.0, 0.0, 0.0),
			axis_dir: lit3(0.0, 0.0, 1.0),
			angle: Dim::Literal(std::f64::consts::TAU / 6.0),
		});
		let bolt_circle = d.add(Feature::Boolean { op: BooleanOp::Difference, a: plate, b: holes });
		d.set_root(bolt_circle);
		let mesh = d.mesh(Resolution::VoxelSize(0.4));
		let expected = 9600.0 - 6.0 * std::f64::consts::PI * 2.5 * 2.5 * 6.0;
		assert!(
			mesh.is_watertight() && (mesh.signed_volume() - expected).abs() / expected < 0.02,
			"bolt circle (voxel path) must be watertight with the correct volume ~{expected:.0}: wt={} vol={:.0}",
			mesh.is_watertight(),
			mesh.signed_volume()
		);
	}

	#[test]
	fn linear_pattern_repeats_a_feature_parametrically() {
		// Four unit cubes stepped 3 mm apart (a clear gap, so no shared face planes):
		// the pattern is a valid solid of volume 4×1. Widening the step keeps it 4
		// disjoint cubes (still volume 4); the count drives how many copies appear.
		let mut doc = Document::new();
		doc.set_param("gap", 3.0);
		let cube = doc.add(Feature::Box {
			center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
			size: [Dim::Literal(1.0), Dim::Literal(1.0), Dim::Literal(1.0)],
		});
		let pat = doc.add(Feature::LinearPattern {
			input: cube,
			count: 4,
			step: [Dim::param("gap"), Dim::Literal(0.0), Dim::Literal(0.0)],
		});
		doc.set_root(pat);

		let solid = doc.evaluate_brep().expect("pattern evaluates");
		let v = kernel_brep::validate(&solid);
		assert!(
			v.is_valid() && v.shells == 4 && (kernel_brep::volume(&solid).abs() - 4.0).abs() < 1e-6,
			"4 spaced cubes should be a valid 4-shell solid of volume 4: {v:?} vol={}",
			kernel_brep::volume(&solid).abs()
		);
	}

	#[test]
	fn mirror_reflects_a_feature_across_a_plane() {
		// A unit cube centred at x=3 (so it sits fully in x>0, with a gap from the
		// plane) mirrored across x=0 → two cubes at x=±3: a valid 2-shell solid of
		// volume 2×1, each correctly oriented (positive volume, not inside-out).
		let mut doc = Document::new();
		let cube = doc.add(Feature::Box {
			center: [Dim::Literal(3.0), Dim::Literal(0.0), Dim::Literal(0.0)],
			size: [Dim::Literal(1.0), Dim::Literal(1.0), Dim::Literal(1.0)],
		});
		let m = doc.add(Feature::Mirror {
			input: cube,
			plane_point: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
			plane_normal: [Dim::Literal(1.0), Dim::Literal(0.0), Dim::Literal(0.0)],
		});
		doc.set_root(m);

		let solid = doc.evaluate_brep().expect("mirror evaluates");
		let v = kernel_brep::validate(&solid);
		assert!(
			v.is_valid() && v.shells == 2 && (kernel_brep::volume(&solid).abs() - 2.0).abs() < 1e-6,
			"mirrored cube should be a valid 2-shell solid of volume 2: {v:?} vol={}",
			kernel_brep::volume(&solid).abs()
		);
	}

	#[test]
	fn mirror_of_a_curved_part_is_exact_via_brep() {
		// Mirroring a CURVED part across a non-cutting plane now builds EXACTLY via the B-rep: the
		// part and its reflection are AABB-disjoint, so they merge by topology (disjoint_union)
		// instead of a boolean union — which on disjoint curved solids self-intersects and reads
		// the volume low. A Ø4×4 cylinder at x=10 mirrored across x=0 → a valid 2-shell solid, free
		// of self-intersection, of volume 2·π·2²·4.
		let lit3 = |x: f64, y: f64, z: f64| [Dim::Literal(x), Dim::Literal(y), Dim::Literal(z)];
		let mut doc = Document::new();
		let cyl = doc.add(Feature::Cylinder { center: lit3(10.0, 0.0, 0.0), radius: Dim::Literal(2.0), height: Dim::Literal(4.0) });
		let m = doc.add(Feature::Mirror { input: cyl, plane_point: lit3(0.0, 0.0, 0.0), plane_normal: lit3(1.0, 0.0, 0.0) });
		doc.set_root(m);
		let solid = doc.evaluate_brep().expect("curved mirror evaluates");
		let v = kernel_brep::validate(&solid);
		let expected = 2.0 * std::f64::consts::PI * 2.0 * 2.0 * 4.0;
		assert!(
			v.is_valid() && v.shells == 2 && !kernel_brep::self_intersects(&solid) && (kernel_brep::volume(&solid).abs() - expected).abs() / expected < 0.03,
			"mirrored cylinder must be exact (valid 2-shell, no self-int, vol ~{expected:.0}): {v:?} self_int={} vol={:.0}",
			kernel_brep::self_intersects(&solid),
			kernel_brep::volume(&solid).abs()
		);
	}

	#[test]
	fn circular_pattern_repeats_a_feature_around_an_axis() {
		// Six unit cubes at radius 5 from the Z axis, stepped 60° apart: a ring of 6.
		// Adjacent centres are 5 mm apart (>> the 1 mm cube), so copies never touch →
		// a valid 6-shell solid of volume 6×1.
		let mut doc = Document::new();
		let cube = doc.add(Feature::Box {
			center: [Dim::Literal(5.0), Dim::Literal(0.0), Dim::Literal(0.0)],
			size: [Dim::Literal(1.0), Dim::Literal(1.0), Dim::Literal(1.0)],
		});
		let ring = doc.add(Feature::CircularPattern {
			input: cube,
			count: 6,
			axis_point: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
			axis_dir: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(1.0)],
			angle: Dim::Literal(std::f64::consts::FRAC_PI_3), // 60°
		});
		doc.set_root(ring);

		let solid = doc.evaluate_brep().expect("circular pattern evaluates");
		let v = kernel_brep::validate(&solid);
		assert!(
			v.is_valid() && v.shells == 6 && (kernel_brep::volume(&solid).abs() - 6.0).abs() < 1e-6,
			"6-box ring should be a valid 6-shell solid of volume 6: {v:?} vol={}",
			kernel_brep::volume(&solid).abs()
		);
	}

	/// Build a 40 × 40 × 10 plate with a centred through-hole of radius `hole_r`.
	fn plate_with_hole() -> Document {
		let mut doc = Document::new();
		doc.set_param("hole_r", 4.0);
		let plate = doc.add(Feature::Box {
			center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
			size: [Dim::Literal(40.0), Dim::Literal(40.0), Dim::Literal(10.0)],
		});
		// Cylinder taller than the plate so it punches all the way through.
		let hole = doc.add(Feature::Cylinder {
			center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
			radius: Dim::param("hole_r"),
			height: Dim::Literal(20.0),
		});
		let part = doc.add(Feature::Boolean { op: BooleanOp::Difference, a: plate, b: hole });
		doc.set_root(part);
		doc
	}

	#[test]
	fn parametric_update_larger_hole_shrinks_volume() {
		let mut doc = plate_with_hole();

		let small_hole_vol = doc_volume(&doc, 0.6);

		// Parametric edit: widen the hole, then re-evaluate + re-mesh.
		doc.set_param("hole_r", 8.0);
		let large_hole_vol = doc_volume(&doc, 0.6);

		// Sanity-check against the closed-form plate-minus-cylinder volume.
		let plate = 40.0f64 * 40.0 * 10.0;
		let expect_small = plate - std::f64::consts::PI * 4.0f64.powi(2) * 10.0;
		let expect_large = plate - std::f64::consts::PI * 8.0f64.powi(2) * 10.0;

		assert!(
			large_hole_vol < small_hole_vol
				&& (small_hole_vol - expect_small).abs() / expect_small < 0.05
				&& (large_hole_vol - expect_large).abs() / expect_large < 0.05,
			"hole_r 4→8 should shrink volume: small={small_hole_vol} (≈{expect_small}), \
			 large={large_hole_vol} (≈{expect_large})"
		);
	}

	#[test]
	fn assembly_bounds_span_both_instances() {
		// Two unit-ish boxes (full side 10) translated apart along x.
		let mk_box = || {
			let mut doc = Document::new();
			let id = doc.add(Feature::Box {
				center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
				size: [Dim::Literal(10.0), Dim::Literal(10.0), Dim::Literal(10.0)],
			});
			doc.set_root(id);
			doc
		};

		let mut asm = Assembly::new();
		asm.add(Instance::document(mk_box(), Affine3A::from_translation(Vec3::new(-20.0, 0.0, 0.0))));
		asm.add(Instance::document(mk_box(), Affine3A::from_translation(Vec3::new(20.0, 0.0, 0.0))));

		let bounds = asm.bounds();
		// Left box spans x∈[-25,-15], right box x∈[15,25] ⇒ combined x∈[-25,25].
		assert!(
			bounds.is_valid()
				&& bounds.min.x <= -24.9
				&& bounds.max.x >= 24.9
				&& (bounds.min.y + 5.0).abs() < 0.1
				&& (bounds.max.y - 5.0).abs() < 0.1,
			"combined bounds should span both instances, got {bounds:?}"
		);

		// And the merged mesh must contain geometry from both parts.
		let mesh = asm.mesh_all(Resolution::VoxelSize(1.0));
		assert!(!mesh.is_empty() && mesh.aabb().min.x <= -24.9 && mesh.aabb().max.x >= 24.9);
	}

	#[test]
	fn difference_features_mesh_to_a_watertight_manifold() {
		// A Difference feature makes a concave crease; the document mesher must return
		// a closed 2-manifold there (via Manifold Dual Contouring), not the
		// non-manifold edges plain Surface Nets leaves. Covers a through-hole plate
		// and an overlapping sphere−sphere cut (the case that exposed the bug).
		let plate = plate_with_hole().mesh(Resolution::VoxelSize(0.6));

		let mut doc = Document::new();
		let a = doc.add(Feature::Sphere {
			center: [Dim::Literal(0.0), Dim::Literal(0.0), Dim::Literal(0.0)],
			radius: Dim::Literal(8.0),
		});
		let b = doc.add(Feature::Sphere {
			center: [Dim::Literal(8.0), Dim::Literal(0.0), Dim::Literal(0.0)],
			radius: Dim::Literal(8.0),
		});
		let cut = doc.add(Feature::Boolean { op: BooleanOp::Difference, a, b });
		doc.set_root(cut);
		let spheres = doc.mesh(Resolution::VoxelSize(0.5));

		assert_eq!(
			(plate.is_watertight(), spheres.is_watertight()),
			(true, true),
			"difference features must mesh to a watertight manifold"
		);
	}

	#[test]
	fn brep_document_face_names_survive_a_parameter_edit() {
		use kernel_brep::{validate, FaceSource};
		// A parametric box with a corner carved by a cutter, built as a B-rep. A face
		// from the cutter (operand B) carries a persistent name; after moving the
		// cutter and re-evaluating, the same logical face is re-selected by that name —
		// topological naming working end-to-end through the Document layer.
		let lit3 = |x: f64, y: f64, z: f64| [Dim::Literal(x), Dim::Literal(y), Dim::Literal(z)];
		let mut doc = Document::new();
		doc.set_param("c", 5.0);
		let a = doc.add(Feature::Box { center: lit3(0.0, 0.0, 0.0), size: lit3(10.0, 10.0, 10.0) });
		let b = doc.add(Feature::Box {
			center: [Dim::param("c"), Dim::param("c"), Dim::param("c")],
			size: lit3(10.0, 10.0, 10.0),
		});
		let d = doc.add(Feature::Boolean { op: BooleanOp::Difference, a, b });
		doc.set_root(d);

		let s1 = doc.evaluate_brep().expect("brep document evaluates");
		assert!(validate(&s1).is_valid(), "brep document is a valid solid: {:?}", validate(&s1));
		let cut = s1.faces().find(|&f| s1.face_source(f) == Some(FaceSource::OperandB)).expect("a cut face from operand B");
		let name = s1.face_name(cut).unwrap();

		doc.set_param("c", 4.0);
		let s2 = doc.evaluate_brep().unwrap();
		assert!(!s2.faces_named(name).is_empty(), "stored face name re-resolves in the edited document");
	}

	#[test]
	fn brep_document_fillet_survives_a_parameter_edit() {
		use kernel_brep::{tessellate_default, validate, EdgeName, FaceName, FaceSource, Surface};
		// A name-consuming feature in the parametric tree: store an edge's persistent
		// name, add a Fillet on it, then EDIT the box size and re-evaluate. The fillet
		// re-attaches to the corresponding edge of the rebuilt part — topological naming
		// load-bearing end-to-end through the Document, not just at the kernel level.
		let lit3 = |x: f64, y: f64, z: f64| [Dim::Literal(x), Dim::Literal(y), Dim::Literal(z)];
		let cyl_axis_xy = |s: &kernel_brep::Solid| -> (f64, f64) {
			s.faces()
				.find_map(|fc| match s.face(fc).surface {
					Surface::Cylinder { origin, .. } => Some((origin.x, origin.y)),
					_ => None,
				})
				.expect("a cylinder fillet face")
		};

		let mut doc = Document::new();
		doc.set_param("s", 10.0);
		let b = doc.add(Feature::Box { center: lit3(0.0, 0.0, 0.0), size: [Dim::param("s"), Dim::param("s"), Dim::param("s")] });
		doc.set_root(b);

		// The +X∧+Y edge of the box (faces 5 and 3 in cuboid's canonical order).
		let edge = EdgeName::new(
			FaceName { operand: FaceSource::Primitive, source_face: 5 },
			FaceName { operand: FaceSource::Primitive, source_face: 3 },
		);
		assert_eq!(doc.evaluate_brep().unwrap().edges_named(edge).len(), 1, "the named edge exists on the box");

		// Append the fillet feature referencing that persistent edge name.
		let f = doc.add(Feature::Fillet { input: b, edge, radius: Dim::Literal(2.0), near: None });
		doc.set_root(f);

		let r1 = doc.evaluate_brep().expect("filleted document evaluates");
		assert!(validate(&r1).is_valid() && tessellate_default(&r1).is_watertight(), "filleted doc valid+watertight: {:?}", validate(&r1));
		let (x1, y1) = cyl_axis_xy(&r1);
		assert!((x1 - 3.0).abs() < 1e-9 && (y1 - 3.0).abs() < 1e-9, "size-10 box fillet axis at +X+Y corner (3,3), got ({x1},{y1})");

		// PARAMETRIC EDIT: grow the box; the SAME stored name re-resolves and the fillet
		// re-attaches — its axis moves from (3,3) to the resized corner (8,8).
		doc.set_param("s", 20.0);
		let r2 = doc.evaluate_brep().expect("edited filleted document evaluates");
		assert!(validate(&r2).is_valid() && tessellate_default(&r2).is_watertight(), "edited filleted doc valid+watertight: {:?}", validate(&r2));
		let (x2, y2) = cyl_axis_xy(&r2);
		assert!((x2 - 8.0).abs() < 1e-9 && (y2 - 8.0).abs() < 1e-9, "size-20 box fillet re-attached to resized +X+Y corner (8,8), got ({x2},{y2})");
	}

	#[test]
	fn feature_suppress_toggles_a_fillet_in_the_rebuild() {
		use kernel_brep::{validate, EdgeName, FaceName, FaceSource};
		// Suppress/unsuppress — the standard parametric-edit toggle: a fillet feature can
		// be switched OFF (the rebuild skips it, yielding its input — the plain box) and
		// back ON, without deleting it. The box is 10³ = 1000 exactly; the fillet rounds
		// an edge, removing a little material; suppressing restores the exact box.
		let lit3 = |x: f64, y: f64, z: f64| [Dim::Literal(x), Dim::Literal(y), Dim::Literal(z)];
		let mut doc = Document::new();
		let b = doc.add(Feature::Box { center: lit3(0.0, 0.0, 0.0), size: lit3(10.0, 10.0, 10.0) });
		let edge = EdgeName::new(
			FaceName { operand: FaceSource::Primitive, source_face: 5 },
			FaceName { operand: FaceSource::Primitive, source_face: 3 },
		);
		let f = doc.add(Feature::Fillet { input: b, edge, radius: Dim::Literal(2.0), near: None });
		doc.set_root(f);

		let vol_on = kernel_brep::volume(&doc.evaluate_brep().expect("filleted"));
		doc.set_suppressed(f, true);
		let suppressed = doc.evaluate_brep().expect("suppressed → plain box");
		let vol_supp = kernel_brep::volume(&suppressed);
		doc.set_suppressed(f, false);
		let vol_back = kernel_brep::volume(&doc.evaluate_brep().expect("unsuppressed → filleted again"));

		assert!(
			validate(&suppressed).is_valid()
				&& (vol_supp - 1000.0).abs() < 1e-6   // suppressed = the exact plain box
				&& vol_on < 999.0
				&& vol_on > 985.0                     // fillet removed a little material
				&& (vol_back - vol_on).abs() < 1e-6,  // unsuppress restores the fillet
			"suppress toggle: on={vol_on} suppressed={vol_supp} back={vol_back}"
		);
	}

	#[test]
	fn brep_document_chamfer_feature_evaluates() {
		use kernel_brep::{tessellate_default, validate, EdgeName, FaceName, FaceSource, Surface};
		// The Chamfer feature is name-consuming like Fillet, but bevels flat. A box with
		// its +X∧+Y edge chamfered evaluates to a valid watertight solid carrying the
		// diagonal bevel plane and no cylindrical face.
		let lit3 = |x: f64, y: f64, z: f64| [Dim::Literal(x), Dim::Literal(y), Dim::Literal(z)];
		let mut doc = Document::new();
		let b = doc.add(Feature::Box { center: lit3(0.0, 0.0, 0.0), size: lit3(10.0, 10.0, 10.0) });
		let edge = EdgeName::new(
			FaceName { operand: FaceSource::Primitive, source_face: 5 },
			FaceName { operand: FaceSource::Primitive, source_face: 3 },
		);
		let c = doc.add(Feature::Chamfer { input: b, edge, radius: Dim::Literal(2.0), near: None });
		doc.set_root(c);

		let s = doc.evaluate_brep().expect("chamfered document evaluates");
		assert!(validate(&s).is_valid() && tessellate_default(&s).is_watertight(), "chamfered doc valid+watertight: {:?}", validate(&s));
		let inv_sqrt2 = std::f64::consts::FRAC_1_SQRT_2;
		assert!(
			s.faces().any(|f| matches!(s.face(f).surface,
				Surface::Plane { normal, .. } if (normal.x - inv_sqrt2).abs() < 1e-6 && (normal.y - inv_sqrt2).abs() < 1e-6 && normal.z.abs() < 1e-6)),
			"the chamfer feature adds the diagonal bevel plane"
		);
		assert!(!s.faces().any(|f| matches!(s.face(f).surface, Surface::Cylinder { .. })), "a chamfer has no cylindrical faces");
	}

	#[test]
	fn empty_document_meshes_to_nothing() {
		let doc = Document::new();
		assert!(doc.evaluate().is_none() && doc.mesh(Resolution::VoxelSize(1.0)).is_empty());
	}

	#[test]
	fn prebuilt_node_instance_meshes() {
		// A prebuilt (non-document) source still places and meshes.
		let node = Node::primitive(Sphere::new(Vec3::ZERO, 6.0));
		let mut asm = Assembly::new();
		asm.add(Instance::node(node, Affine3A::from_translation(Vec3::new(3.0, 0.0, 0.0))));
		let mesh = asm.mesh_all(Resolution::VoxelSize(0.5));
		let v = mesh.signed_volume();
		let expect = 4.0 / 3.0 * std::f64::consts::PI * 6.0f64.powi(3);
		assert!((v - expect).abs() / expect < 0.03, "prebuilt sphere vol {v} vs {expect}");
	}
}

// ---------------------------------------------------------------------------
// Materials + posed-pair sweep checking (promoted from the campaign examples)
// ---------------------------------------------------------------------------

/// Print-material densities in g/mm³ — the constants every campaign example
/// used to re-declare. Multiply an engine volume (mm³) by one of these for a
/// solid-equivalent mass in grams; slicer infill scales it down from there.
pub mod materials {
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
}

/// Vertex-sampled penetration ESTIMATE between two meshes: the deepest of
/// either mesh's vertices inside the other's winding-number field, in model
/// units (0.0 ⟺ no sampled vertex is contained). Hundreds of times cheaper
/// than an exact boolean, so a kinematic sweep can afford dense poses —
/// but it is an **underestimate by construction** (an edge–edge crossing
/// with no contained vertex reads 0.0): gate load-bearing poses with
/// [`kernel_brep::overlap_volume`], use this for the dense in-between poses.
/// At most `max_samples` vertices per side are tested (evenly strided).
pub fn penetration_estimate(a: &Mesh, b: &Mesh, max_samples: usize) -> f64 {
	let mut worst = 0.0f64;
	let mut probe = |host: &Mesh, guest: &Mesh| {
		let sdf = kernel_implicit::MeshSdf::new(host);
		let n = guest.positions.len().max(1);
		let stride = (n / max_samples.max(1)).max(1);
		for p in guest.positions.iter().step_by(stride) {
			let d = kernel_core::sdf::Sdf::distance(&sdf, *p) as f64;
			if d < -worst {
				worst = -d;
			}
		}
	};
	probe(a, b);
	probe(b, a);
	worst
}

/// One pose of a [`sweep_check`]: the mesh↔mesh clearance, the sampled
/// penetration estimate, and the EXACT proper-crossing verdict at that pose.
#[derive(Clone, Copy, Debug)]
pub struct SweepPose {
	pub min_distance: f64,
	pub penetration: f64,
	/// Exact triangle-level proper crossing ([`Mesh::crosses_mesh`]) — the
	/// oracle vertex sampling cannot fake.
	pub crossing: bool,
}

/// Result of sweeping a moving mesh against a fixed one along a pose path.
#[derive(Clone, Debug)]
pub struct SweepReport {
	pub poses: Vec<SweepPose>,
	/// Smallest clearance seen across poses with zero sampled penetration.
	pub min_clearance: f64,
	/// Deepest sampled penetration across all poses (0.0 = none detected).
	pub max_penetration: f64,
	/// Poses whose surface distance was ≈0 (< 0.02): touching OR crossing.
	/// The penetration estimate is vertex-sampled and can read 0.0 through a
	/// thin wall with no contained vertices (a real slider-through-parapet
	/// collision did exactly that, DRYBOX 2026-07-28) — so a FREE-RUN gate
	/// must assert `contacts == 0`, not just `max_penetration ≈ 0`.
	pub contacts: usize,
	/// Poses with an EXACT proper triangle crossing — the definitive
	/// interpenetration verdict (touching and coplanar kisses excluded).
	/// A free-run gate asserts `crossings == 0 && contacts == 0`; an
	/// intentional-interference sweep (a click ring) expects `crossings > 0`.
	pub crossings: usize,
}

/// The campaign kinematic-sweep idiom (DOVESTACK → POOLDOCK → RESPOOL),
/// promoted: pose `moving` by each transform, measure clearance to `fixed`
/// (BVH mesh distance) and a sampled penetration estimate. Cheap enough for
/// dense insertion/twist paths; see [`penetration_estimate`] for what the
/// estimate can and cannot see — poses that must PROVE non-interference
/// (locks, seats) still deserve an exact `overlap_volume` gate on top.
pub fn sweep_check(fixed: &Mesh, moving: &Mesh, poses: &[kernel_core::math::DAffine3]) -> SweepReport {
	// Poses are independent — kernel_core::par::par_map_indexed evaluates
	// them on scoped threads and returns BY INDEX, so the report is identical
	// to a serial run regardless of scheduling. (Coarse-grained only: the
	// boolean arrangement stays single-threaded to protect R5.)
	let results: Vec<SweepPose> = kernel_core::par::par_map_indexed(poses, |_, m| {
		let posed = moving.transformed_by(*m);
		let min_distance = fixed.min_distance(&posed);
		let near = min_distance < 0.05;
		let penetration = if near { penetration_estimate(fixed, &posed, 4000) } else { 0.0 };
		let crossing = near && fixed.crosses_mesh(&posed);
		SweepPose { min_distance, penetration, crossing }
	});
	let mut out = SweepReport {
		poses: Vec::with_capacity(poses.len()),
		min_clearance: f64::INFINITY,
		max_penetration: 0.0,
		contacts: 0,
		crossings: 0,
	};
	for sp in results {
		if sp.min_distance < 0.02 {
			out.contacts += 1;
		}
		if sp.crossing {
			out.crossings += 1;
		}
		if sp.penetration == 0.0 {
			out.min_clearance = out.min_clearance.min(sp.min_distance);
		}
		out.max_penetration = out.max_penetration.max(sp.penetration);
		out.poses.push(sp);
	}
	out
}
