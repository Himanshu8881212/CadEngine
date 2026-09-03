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

pub mod assembly;
pub mod campaign;
pub mod constraints;
pub mod cost;
pub mod document;
pub mod drawing;
pub mod feature;
pub mod format;
pub mod hybrid;
pub mod kinematics;
pub mod library;
pub mod loads;
pub mod materials;
pub mod mechanism;
pub mod meshing;
pub mod optimize;
pub mod parts;
pub mod persist;
pub mod process;
pub mod rate;
pub mod reverse;
pub mod shell;
pub mod sketch;
pub mod sweep;
pub mod tolerance;

pub use assembly::{Assembly, AsmState, Instance, Source};
pub use constraints::{Constraint, ConstraintSystem, DofReport};
pub use document::{Document, DocumentHistory, LATTICE_FILL_MAX_CELLS};
pub use feature::{BooleanOp, CatalogPart, Dim, Feature, FeatureId, HoleFit, HoleKind, LatticeCellKind, LinearGrade, TpmsFamily};
pub use hybrid::{hybrid_boolean, HybridError, HybridOperand, HybridReport, HybridResult, HybridRoute, HYBRID_EXACT_MAX_OPERAND_TRIS};
pub use kinematics::{CycloidTrain, EpicyclicPoses, EpicyclicTrain, PlanetPose, StrainWaveTrain};
pub use meshing::{precise_mesh, routed_mesh, watertight_mesh, watertight_mesh_of, MeshRoute, RouteReport};
pub use rate::{cantilever_bending_stress, lewis_form_factor, lewis_tooth_load, thin_ring_bending_strain, Stackup};
pub use sketch::{
	Arc, Circle, ConstraintState, Segment, Sketch, SketchAnalysis, SketchConstraint, SketchError, SolveReport,
};
pub use sweep::{penetration_estimate, sweep_check, SweepPose, SweepReport};

#[cfg(test)]
mod tests;
