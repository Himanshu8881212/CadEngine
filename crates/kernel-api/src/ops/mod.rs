// Copyright (c) LMCAD. Licensed under the MIT License.

//! The op implementations, one module per family.
//!
//! [`crate::interp`] owns the program loop, the environment, the allocation caps
//! and the dispatch table; everything a single op *does* lives here. Each family
//! module exposes one `exec` that matches exactly the [`crate::program::OpKind`]
//! variants `interp::exec_op` routes to it — the compiler proves the union of
//! those routes covers every variant, so an op can never fall through unhandled.
//!
//! To add an op: declare its variant in `program.rs`, implement it in the family
//! module below, add its name to that family's route in `interp::exec_op`, then
//! regenerate the `describe` tables with `python3 tools/gen_discover.py`.
//!
//! `support` and `meshio` are not op families — they are the two shared halves
//! every family may draw on: the small geometry/error helpers and the validity
//! gate, and the path confinement plus mesh in/out plumbing.

pub(crate) mod meshio;
pub(crate) mod support;

pub(crate) mod assemblies;
pub(crate) mod booleans;
pub(crate) mod catalog;
pub(crate) mod cuts;
pub(crate) mod designmath;
pub(crate) mod features;
pub(crate) mod holes;
pub(crate) mod hybrid;
pub(crate) mod io;
#[cfg(feature = "catalog")]
pub(crate) mod library;
pub(crate) mod measure;
pub(crate) mod primitives;
pub(crate) mod sketch;
pub(crate) mod threads;
