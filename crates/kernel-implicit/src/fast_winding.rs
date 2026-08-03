// Copyright (c) LMCAD. Licensed under the MIT License.

//! Fast winding number — now folded into [`MeshSdf`](crate::meshsdf::MeshSdf).
//!
//! Earlier this module held a separate `FastWindingSdf` with its own BVH + dipole
//! winding. [`MeshSdf`] now uses that exact fast scheme for its sign, so the two
//! are identical; `FastWindingSdf` is kept only as a backwards-compatible alias.

/// Backwards-compatible alias for [`MeshSdf`](crate::meshsdf::MeshSdf), whose sign
/// is computed with the BVH-accelerated, dipole-approximated generalized winding
/// number (Barill et al.).
pub type FastWindingSdf = crate::meshsdf::MeshSdf;
