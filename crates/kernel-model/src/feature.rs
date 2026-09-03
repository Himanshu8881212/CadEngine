// Copyright (c) LMCAD. Licensed under the MIT License.

//! The feature vocabulary of the parametric history: [`Feature`] and the small
//! value types its variants are built from ([`Dim`], [`FeatureId`], [`BooleanOp`],
//! [`HoleKind`] / [`HoleFit`], [`CatalogPart`], [`LinearGrade`], [`LatticeCellKind`],
//! [`TpmsFamily`]).
//!
//! A [`Feature`] is pure data: it says *what* to build, never *how*. The evaluation
//! that turns a feature list into geometry lives in [`crate::document`].

use std::collections::HashMap;

use kernel_core::math::Affine3A;
use kernel_implicit::lattice::LatticeCell;
use kernel_implicit::TpmsKind;
use serde::{Deserialize, Serialize};

use crate::parts;
use crate::persist;
use crate::sketch::Sketch;

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
	pub(crate) fn to_brep(self) -> kernel_brep::Fit {
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
	pub(crate) fn to_implicit(self) -> LatticeCell {
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
