// Copyright (c) LMCAD. Licensed under the MIT License.

//! `kernel-brep` — the exact half of the hybrid kernel.
//!
//! Closed-form analytic [`Surface`]s / [`Curve`]s in an index-arena half-edge
//! [`Solid`], with primitive / [`extrude`] / [`revolve`] construction, exact
//! [`tessellate`]ation, and validity oracles ([`validate`], [`volume`]). There
//! are **no native booleans** — those route through the implicit/voxel domain.

pub mod booleans;
pub mod build;
pub mod chain;
pub mod coalesce;
pub mod recover;
pub mod checked;
pub mod curved_boolean;
pub mod hazards;
pub mod fillet;
pub mod freeform;
pub mod geom;
pub mod heal;
pub mod interference;
pub mod measure;
pub mod mesh_boolean;
pub mod nurbs;
pub mod policy;
pub mod ssi;
pub mod step_export;
pub mod step_import;
pub mod step_tolerant;
pub mod tessellate;
pub(crate) mod tol;
pub mod tessellate_adaptive;
pub mod topo;
pub mod validate;

pub use booleans::{difference, intersection, union};
pub use chain::{ChainError, ChainLog, ChainStep};
pub use coalesce::coalesce_coplanar;
pub use checked::{
	try_difference, try_difference_diagnosed, try_difference_sealed, try_freeform_boolean, try_intersection,
	try_intersection_diagnosed, try_intersection_sealed, try_union, try_union_diagnosed, try_union_sealed,
	BooleanError, BooleanRefusal, FreeformTool, SealedError,
};
pub use build::{
	chamfered_cylinder, cone, cuboid, cylinder, extrude, extrude_tapered, extrude_with_holes, filleted_cylinder,
	force_ccw, radial_frame, revolve, sector_prism, sphere, torus,
};
pub use hazards::{boolean_hazards, Hazard, HazardKind};
pub use curved_boolean::{
	boundary_loops, drill_cylinder, intersect_sphere, seam_loops, subtract_cone, subtract_sphere,
	trim_mesh_by_surface, union_sphere, Keep,
};
pub use fillet::{
	chamfer_cylinder_rim, chamfer_edge, chamfer_edge_near, cylinder_rim, fillet_circular_rim, fillet_cylinder_rim,
	fillet_edge, fillet_edge_near, fillet_edge_segments, rim_fillet_band, FilletError, fillet_circular_rim_concave,
};
pub use freeform::{
	freeform_plane_cut, freeform_plate, loft, loft_solid, plane_patch_curves, sweep, sweep_solid,
	FreeformBoolError, FreeformCut, FreeformCutOptions, FreeformSolid, PatchPlaneCurve,
};
pub use geom::{Curve, Surface};
pub use heal::{boolean_tolerant, HealReport, TolerantBoolean};
pub use interference::{detect_coincident_fit, overlap_volume, overlap_volume_many};
pub use measure::{bounding_box, bounding_box_of, distance, BoundingBox};
pub use mesh_boolean::{
	auto_seam_band, exact_boolean, exact_boolean_auto, mesh_difference, mesh_intersection, mesh_union, solid_from_mesh,
	MeshBoolOp,
};
pub use nurbs::{FreeformFace, NurbsCurve, NurbsSurface};
pub use policy::{boolean_with_policy, BooleanOutcome, BooleanPath, BooleanStats};
pub use ssi::{
	intersect_surfaces, refine_seam_to_intersection, snap_seam_to_intersection, ImplicitSurface, NurbsField, SsiOptions,
};
pub use step_export::{export_step, export_step_ap242, export_step_assembly, export_step_freeform};
pub use step_import::{import_bspline_curve, import_bspline_mesh, import_bspline_surface, import_step, import_step_assembly, import_step_freeform, StepError};
pub use step_tolerant::{import_step_tolerant, step_census, ImportEvent, SolidRecord, SolidStatus, TolerantImport};
pub use tessellate::{tessellate, tessellate_default, TessOptions};
pub use tessellate_adaptive::{tessellate_adaptive, tessellate_adaptive_tol};
pub use topo::{
	Edge, EdgeId, EdgeName, Face, FaceId, FaceInput, FaceLoops, FaceName, FaceSource, HalfEdge, HalfEdgeId,
	Loop, LoopId, Shell, ShellId, Solid, Vertex, VertexId, VertexName,
};
pub use validate::{
	area, draft_analysis, euler_characteristic, exact_volume, mass_properties, overhang_analysis,
	section_curves_with_fallback, section_properties, self_intersects, validate, volume, wall_thickness, SectionCurve,
	Validity,
};

// Convenience re-exports.
pub use kernel_core::math;
pub use kernel_core::{
	DraftReport, MassProperties, Mesh, OverhangReport, PrincipalAxes, SectionProperties, SupportFreeReport,
	ThicknessReport,
};

pub mod holes;
pub use holes::{bearing_seat, bearing_spec, bearing_specs, bolt_circle, clearance_hole, counterbore_hole, countersink_hole, drill, metric_hole_spec, metric_hole_specs, min_ligament, tap_drill_hole, teardrop_hole, teardrop_profile, BearingSpec, Fit, HoleDepth, HoleError, MetricHoleSpec, DEFAULT_HOLE_SEGMENTS};
