// Copyright (c) LMCAD. Licensed under the MIT License.

//! # kernel-api — the JSON program binding (AI-Interface track I1 + I4)
//!
//! A non-Rust caller (an AI, a script, another process) drives the LMCAD hybrid
//! kernel through a JSON **program**:
//!
//! ```json
//! {"ops": [
//!   {"id": "plate", "op": "box", "min": [0, 0, 0], "max": [60, 40, 8]},
//!   {"id": "vol",   "op": "volume", "in": "plate"},
//!   {"id": "out",   "op": "export_stl", "in": "plate", "file": "plate.stl"}
//! ]}
//! ```
//!
//! Each op binds its result (solid / sketch / mesh) to its `id`; later ops
//! reference earlier results by id through `in` / `a` / `b` / `sketch`. Running a
//! program yields a structured [`Report`] — per-op `ok`, measures, written files,
//! and machine-matchable [`ErrorKind`]s — and never a silent invalid solid: every
//! solid-producing op is gated through `kernel_brep::validate()`.
//!
//! 160 ops: primitives/sweeps, constrained sketches, booleans, fillet/chamfer
//! features, transforms (the axis rotations, the general rigid `pose`, the
//! orientation-safe `mirror`, and the `linear_pattern` / `polar_pattern`
//! clone-union patterns), measures, the **in-program assembly surface**
//! (`asm_instance` / `asm_instance_mesh`, the raw + face/axis-derived
//! `asm_mate*` ops, the DOF-honest `asm_solve`, `asm_contacts` /
//! `asm_interference_volume` / `asm_mass_properties`, STL + AP214-STEP
//! assembly exports, the `.lmcasm`-writing `asm_save`, and the
//! `gear_train_poses` kinematics bridge),
//! assertions (`assert` / `assert_disjoint` — programs fail on unmet intent),
//! STL/STEP/3MF exports, the **imports** (`import_step` — STEP files back in as
//! exact B-reps; `import_mesh` — mesh files with the honest `check_mesh`
//! receipt; `mesh_carve` — solid∘mesh booleans through the winding-number voxel
//! half), a gyroid lattice, the voxel-route `shell` hollow, the
//! `implicit` expression-tree op (the CSG `Node` algebra + a safe scalar-field
//! math language as nestable JSON — BAR.md I6; leaves include the periodic
//! strut lattices, `pipe_path`, Hershey `text`, and the `displace` texture
//! combinator, with `{"grid": …}` NPY simulation fields as `offset_by`/`lerp`
//! grade sources), the **voxel-route solid ops** (`offset_solid` /
//! `shell_solid` / `solid_from_implicit` — the reverse bridge v1: faceted
//! B-reps back into the solid environment, route `"voxel"`, honestly labeled)
//! and the interrogation probes (`thin_wall` sampled census, `min_ligament`
//! advisory echo), the `.lmcpart` native-format
//! loader (`load_part` — BAR.md I3b), the curated admission-gated parts
//! **library** (`library_add` / `library_search` / `library_instantiate` /
//! `library_deprecate` / `library_remove` — BAR.md I7), the standard-parts
//! catalog (gears/racks/ring gears, fasteners and screws, pins and circlips,
//! pulleys, sprockets, shafts and keys, springs, extrusion stock, O-rings), the
//! standard feature cuts (heat-set bosses, circlip and O-ring gland grooves),
//! design-math lookups (GT2 belt sizing, ISO 286 limit fits), the ISO/DIN hole
//! wizard (drill / clearance / counterbore / countersink / tap-drill) and the
//! modelled **ISO threads** (`thread_spec` / `thread_ridge` / `export_threaded`
//! — exact ISO 68-1 ridge geometry, fused/cut through the voxel half).
//!
//! The CLI binary is `kernel-api run program.json [--out-dir DIR]` (report on
//! stdout, exit 0 iff every op succeeded). The op-by-op cookbook with one
//! runnable example per op lives in `API.md` at the repo root.
//!
//! The second subcommand is the **`.lmcasm` executable surface** (BAR.md I3b /
//! FRICTION.md #1): `kernel-api asm assembly.lmcasm [--base-dir DIR]
//! [--out-dir DIR] [--tol MM] [--voxel MM] [--window MM]` loads an assembly
//! file, re-solves its mates (gated residual), writes the merged / per-instance
//! / per-named-state STLs and the BOM, and runs the contact scan — same report
//! shape, same exit contract (see [`run_assembly`]).

mod asm;
mod asmops;
pub mod bridge;
mod discover;
mod implicit;
mod interp;
mod program;
mod report;
mod require;

pub use asm::{run_assembly, AsmOptions};
pub use discover::{op_params, op_tag, ParamSpec, OP_COUNT, OP_NAMES, OP_PARAMS};
pub use interp::{run_program, run_program_with_input_base};
pub use program::{
	ArcSpec, BoltHoleSpec, BoolOpSpec, CircleSpec, ConstraintSpec, DomainSpec, FitSpec, LibraryMetaSpec, LibraryParamSpec,
	LibraryProvenanceSpec, MesherSpec, OpKind, PlaneSpec, RotateSpec, ShaftKeywaySpec, WithinSpec,
};
pub use report::{ErrorKind, OpError, OpReport, Report};
