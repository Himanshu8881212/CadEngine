// Copyright (c) LMCAD. Licensed under the MIT License.

//! **The tolerance registry** — every load-bearing epsilon of the exact
//! B-rep pipeline, named, documented, and defined in ONE place (2026-07-28
//! cleanup wave; the RESPOOL/DRYBOX campaigns proved these numbers are
//! design-relevant: §7.7's whole hygiene checklist exists because geometry
//! within a few of these of a face or meridian changes boolean fate).
//!
//! Registry of tolerances defined ELSEWHERE on purpose (owned by their
//! subsystem, listed here so this file is the one map):
//! - `tessellate::TessOptions::weld_tolerance` (default 1e-5 f32): mesh-side
//!   vertex weld at tessellation time.
//! - SSI seam snapping (`ssi.rs`): plane∩cylinder exact to 1e-15,
//!   quadric∩quadric ≤ 1e-9 via parameter-space charts (BAR Level 7).
//! - `hazards::boolean_hazards(tol)`: CALLER-chosen hazard band (0.05 mm is
//!   the authoring default) — a design-review radius, not an engine epsilon.

/// Absolute "on the plane" / coincidence tolerance of the planar
/// arrangement, model units (mm). Faces closer than this are the
/// cancel-coincident case; gaps between ~this and ~0.1 mm are the sliver
/// band §7.7 warns about.
pub(crate) const EPS: f64 = 1e-9;

/// Weld tolerance for stitching boolean fragments back into a half-edge
/// solid.
pub(crate) const WELD_EPS: f64 = 1e-7;

/// T-junction healing: a vertex within this distance of another face's edge
/// is inserted into that edge (see `booleans::resolve_t_junctions`). The
/// sliver filter in `stitch` MUST use the same value: a triangle thinner
/// than this would have its own middle vertex inserted into its own base
/// edge, folding its boundary onto itself — the non-manifold seed of R2/R3.
pub(crate) const TJUNCTION_EPS: f64 = WELD_EPS * 4.0;

/// Quantum for grouping analytic surfaces by identity (hazard grouping,
/// coplanar coalescing): two planes/cylinders whose canonicalized
/// parameters agree to this are "the same surface".
pub(crate) const SURF_KEY_QUANTUM: f64 = 1e-7;

/// Below this separation a face pair counts as EXACTLY coincident (the
/// supported cancel path) rather than nearly-coincident (the hazard).
pub(crate) const COINCIDENT_EXACT_EPS: f64 = 1e-7;
