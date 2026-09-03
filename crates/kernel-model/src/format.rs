// Copyright (c) LMCAD. Licensed under the MIT License.

//! Native file formats (BAR.md, I3b): **`.lmcpart`** and **`.lmcasm`** — the
//! living source files of the engine. STL/STEP/3MF remain *export* formats;
//! these two are what a session (human or AI) saves, hand-edits, diffs and
//! resumes. Geometry is **never stored** — a part file carries the full
//! parametric recipe ([`Document`]) and the kernel's deterministic rebuild (R5)
//! re-evaluates it to the bit-identical solid on load.
//!
//! ## `.lmcpart` — a single part
//! A self-describing envelope around the [`Document`] JSON of [`crate::persist`]:
//!
//! ```json
//! {
//!   "format": "lmc-part",
//!   "version": 1,
//!   "units": "mm",
//!   "name": "bracket",
//!   "created_with": "lmcad-kernel 0.1.0",
//!   "meta": {"part_number": "BRK-001",
//!            "material": {"name": "steel", "density_g_cm3": 7.85},
//!            "make_or_buy": "make"},
//!   "document": { "params": {…}, "features": […], "root": …, "suppressed": […] }
//! }
//! ```
//!
//! [`save_part`] / [`load_part`]. `format`, `version` and `units` are contract
//! fields checked **before** the document is parsed, so a wrong or future file
//! fails with a precise [`FormatError`] instead of a schema soup; `name` and
//! `created_with` are descriptive (a hand-written envelope may omit them).
//!
//! `meta` (BOM v2) is **optional engineering metadata** — [`PartBomMeta`]: a
//! part number, a [`Material`] (name + density, g/cm³) and a make-or-buy
//! sourcing class, each independently omittable. It feeds the assembly bill of
//! materials ([`LoadedAssembly::bom_v2`]); the geometry kernel itself never
//! reads it. Fully backward compatible: an envelope without `meta` loads with
//! [`PartMeta::meta`] `None`, and [`save_part`] (no meta) writes byte-identical
//! files to the pre-meta format — use [`save_part_with_meta`] to stamp it.
//!
//! ## `.lmcasm` — an assembly
//! Instances (each a part **by relative path**, a part **embedded inline** as a
//! full part envelope, a **sub-assembly by relative path** (v2), or a
//! **triangle-mesh file by relative path** — `{"mesh": "part.stl"}`, the bridge
//! that lets program-surface / imported / scanned parts join a mated assembly,
//! measured honestly on their welded mesh) at
//! rigid poses, plus the assembly's mates, optional per-instance suppression
//! and optional named **states** (pose/suppression snapshots — "exploded",
//! "packed", … — see [`AsmState`]):
//!
//! ```json
//! {
//!   "format": "lmc-asm",
//!   "version": 1,
//!   "units": "mm",
//!   "name": "clamp",
//!   "instances": [
//!     {"name": "base", "source": {"path": "base.lmcpart"},
//!      "pose": {"translation": [0.0, 0.0, 0.0]}},
//!     {"name": "cap", "suppressed": true, "source": {"part": { …inline lmc-part envelope… }},
//!      "pose": {"translation": [0.0, 0.0, 2.0], "rotation": [0.0, 0.0, 0.0, 1.0]}},
//!     {"name": "stage1", "source": {"asm_path": "stage1.lmcasm"},
//!      "pose": {"translation": [45.0, 0.0, 0.0]}}
//!   ],
//!   "mates": [ {"Coincident": {"a": 0, "a_point": […], "b": 1, "b_point": […]}}, … ],
//!   "states": { "service": {"poses": [ {"translation": […]}, … ], "suppressed": [1]} }
//! }
//! ```
//!
//! `suppressed` and `states` are omitted when empty (and default so on load), so
//! files without them read and write exactly as before they existed.
//!
//! [`save_assembly`] / [`load_assembly`]. Loading resolves `path` / `asm_path`
//! sources relative to the assembly file's directory (`base_dir`), rebuilds
//! every part from its document, applies the stored poses, and **re-solves the
//! mates** — the stored poses are only the solver's seed; the mates are the
//! authority, so a hand-edited pose snaps back to a consistent assembly on load
//! (the returned [`LoadedAssembly::residual`] reports how well they were
//! satisfied).
//!
//! ### Assembly nesting (v2) — semantics, precisely
//! - An `asm_path` source loads the referenced `.lmcasm` **recursively**, to
//!   arbitrary depth, each file's own sources resolving against *its own*
//!   directory (the same base-dir contract at every level). Include cycles are
//!   detected and refused loudly ([`FormatError::AsmCycle`] names the chain).
//! - A sub-assembly first solves its **own** mates internally, then is placed
//!   as **one rigid unit** by the parent instance's pose. Parent-level mates
//!   may reference **top-level instances only** (parts or whole
//!   sub-assemblies, by file instance index); the mate geometry of a
//!   sub-assembly unit is expressed in the *sub-assembly's* frame. Mating to a
//!   sub-assembly's internal member from the parent is **out of scope in v2**.
//! - The loaded [`LoadedAssembly::assembly`] is **flattened to leaf parts**
//!   (so meshing, contacts, clearance and mass properties all see real part
//!   geometry); the hierarchy is preserved in [`LoadedAssembly::tree`] and in
//!   hierarchical instance names (`"stage1/bearing_l"`). After editing poses,
//!   re-solve with [`LoadedAssembly::solve_mates`] (NOT
//!   `assembly.solve_mates`, whose indices are leaves — equivalent only for
//!   flat assemblies).
//! - Suppressing a sub-assembly instance drops its **entire branch** —
//!   geometry, contacts and BOM. A member suppressed inside the sub-assembly's
//!   own file stays suppressed in the parent.
//! - Parent states pose/suppress **top-level instances** (a state pose moves a
//!   whole sub-assembly rigidly; a state cannot address — or un-suppress — a
//!   sub-assembly's internal member). A sub-assembly's own named states are
//!   not surfaced through the parent (v2 limit).
//! - v2 sub-assemblies are **path-referenced only**: there is no inline
//!   sub-assembly envelope (unlike parts, which may embed inline).
//! - [`LoadedAssembly::residual`] is the **maximum** mate residual across all
//!   levels, so one gate still proves every level converged.
//!
//! ## Bill of materials (BOM v2)
//! [`LoadedAssembly::bom_v2`] returns the [`BomV2`] payload — `schema:
//! "bom/2"`, a grouped `flat` view ([`BomLine`]: counts, parameter summaries,
//! and, where the part envelopes carry `meta`, part numbers, materials and
//! masses with an honest [`VolumeSource`] label) and a `tree` view
//! ([`BomNode`]: the per-instance nesting structure with rolled-up leaf-part
//! counts). [`BomV2::to_json`] / [`BomV2::to_csv`] are the byte-stable
//! machine-readable exports.
//!
//! ## Contract (shared by both formats)
//! - **Byte-stable saves**: envelope fields are written in a fixed order and
//!   every map inside the document is sorted (see [`crate::persist`]), so saving
//!   the same model twice yields identical bytes — designs git-diff like code
//!   (BAR.md I5).
//! - **Loud failures**: a missing/foreign `format`, an unknown `version`,
//!   non-millimetre `units`, an unreadable referenced part, or a non-rigid pose
//!   each return a dedicated [`FormatError`] variant; nothing half-loads.
//! - **Rigid poses only (v1)**: an instance pose persists as translation +
//!   rotation quaternion. A live [`Instance`] may also carry uniform scale —
//!   that is NOT representable in this format, and [`save_assembly`] rejects it
//!   with [`FormatError::BadPose`] rather than silently dropping the scale.

use std::collections::BTreeMap;
use std::fmt;
use std::path::{Path, PathBuf};

use kernel_core::math::{Affine3A, Quat, Vec3};
use kernel_core::mesh::Mesh;
use kernel_core::mesher::Resolution;
use serde::{Deserialize, Serialize};

use crate::{AsmState, Assembly, Constraint, ConstraintSystem, Document, Instance, Source};

/// The `format` tag of a `.lmcpart` envelope.
pub const PART_FORMAT: &str = "lmc-part";
/// The `format` tag of a `.lmcasm` envelope.
pub const ASM_FORMAT: &str = "lmc-asm";
/// The (single) envelope version this kernel writes and reads.
pub const FORMAT_VERSION: u32 = 1;
/// The only unit system of v1 files; project-wide convention.
const UNITS_MM: &str = "mm";
/// Mate-solver iteration budget used by [`load_assembly`] (generous; the solver
/// stops early once converged, and file poses are normally already solved).
const MATE_SOLVE_ITERATIONS: usize = 256;
/// What this kernel stamps into `created_with`.
const CREATED_WITH: &str = concat!("lmcad-kernel ", env!("CARGO_PKG_VERSION"));

/// Why a native file could not be loaded (or, for [`FormatError::BadPose`] on
/// save, written). Every variant names exactly what is wrong — a loader never
/// guesses and never half-loads.
#[derive(Debug)]
pub enum FormatError {
	/// The text is not valid JSON, or it parses but does not match the schema
	/// (e.g. a feature variant from a newer kernel inside `document`).
	Parse(serde_json::Error),
	/// The `format` field is missing or names a different format than the loader
	/// expects (e.g. a `.lmcasm` envelope handed to [`load_part`]).
	WrongFormat {
		/// The format tag the loader required.
		expected: &'static str,
		/// What the file actually said (`None`: no `format` field at all).
		found: Option<String>,
	},
	/// The `version` field is missing or not one this kernel reads.
	UnsupportedVersion {
		/// What the file actually said (`None`: no `version` field at all).
		found: Option<u32>,
		/// The version this kernel supports.
		supported: u32,
	},
	/// The `units` field is missing or not millimetres — refusing beats silently
	/// building wrong-scale geometry from an inch file.
	UnsupportedUnits {
		/// What the file actually said (empty: no `units` field at all).
		found: String,
	},
	/// A path-referenced part file could not be read from disk.
	Io {
		/// The resolved path that failed.
		path: PathBuf,
		/// The underlying I/O error.
		error: std::io::Error,
	},
	/// A path-referenced part file was read but failed to load as a `.lmcpart`;
	/// carries the path for context plus the inner error.
	PartSource {
		/// The resolved path of the offending part file.
		path: PathBuf,
		/// Why that part failed to load.
		error: Box<FormatError>,
	},
	/// A path-referenced sub-assembly file (`asm_path` source) was read but
	/// failed to load as a `.lmcasm`; carries the path for context plus the
	/// inner error — nested failures chain, so a broken part three levels down
	/// reports every enclosing sub-assembly on the way up.
	SubAssembly {
		/// The resolved path of the offending sub-assembly file.
		path: PathBuf,
		/// Why that sub-assembly failed to load.
		error: Box<FormatError>,
	},
	/// Sub-assembly `asm_path` references form a **cycle** (a file includes
	/// itself, directly or through intermediates). Loading refuses instead of
	/// recursing forever.
	AsmCycle {
		/// The canonicalized path that closed the cycle (it is already being
		/// loaded higher up the include chain).
		path: PathBuf,
		/// The include chain that closed on itself, first-opened file first,
		/// ending with the repeated `path`.
		chain: Vec<PathBuf>,
	},
	/// An instance pose is not a finite rigid translation+rotation: on save, the
	/// pose carries scale (not representable in v1); on load, the file's
	/// translation/quaternion is zero-length or non-finite.
	BadPose {
		/// Index of the offending instance.
		instance: usize,
	},
	/// A named assembly state does not fit the assembly: wrong pose count, an
	/// out-of-range suppressed index, or a non-rigid/non-finite state pose.
	BadState {
		/// The offending state's name.
		state: String,
		/// What exactly is wrong with it.
		reason: String,
	},
}

impl fmt::Display for FormatError {
	fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
		match self {
			FormatError::Parse(e) => write!(f, "not a loadable LMCAD file: {e}"),
			FormatError::WrongFormat { expected, found: Some(found) } => {
				write!(f, "wrong format: expected \"{expected}\", the file says \"{found}\"")
			}
			FormatError::WrongFormat { expected, found: None } => {
				write!(f, "wrong format: expected \"{expected}\", the file has no \"format\" field")
			}
			FormatError::UnsupportedVersion { found: Some(found), supported } => {
				write!(f, "unsupported version {found} (this kernel reads version {supported})")
			}
			FormatError::UnsupportedVersion { found: None, supported } => {
				write!(f, "missing \"version\" field (this kernel reads version {supported})")
			}
			FormatError::UnsupportedUnits { found } if found.is_empty() => {
				write!(f, "missing \"units\" field (this kernel reads millimetre files: \"units\": \"mm\")")
			}
			FormatError::UnsupportedUnits { found } => {
				write!(f, "unsupported units \"{found}\" (this kernel reads millimetre files only)")
			}
			FormatError::Io { path, error } => write!(f, "cannot read referenced part '{}': {error}", path.display()),
			FormatError::PartSource { path, error } => write!(f, "in referenced part '{}': {error}", path.display()),
			FormatError::SubAssembly { path, error } => write!(f, "in sub-assembly '{}': {error}", path.display()),
			FormatError::AsmCycle { path, chain } => {
				let chain: Vec<String> = chain.iter().map(|p| p.display().to_string()).collect();
				write!(
					f,
					"sub-assembly cycle: '{}' is already being loaded by this include chain ({}) — an assembly cannot contain itself, directly or through intermediates",
					path.display(),
					chain.join(" -> ")
				)
			}
			FormatError::BadPose { instance } => {
				write!(f, "instance {instance}: pose is not a finite rigid translation+rotation (v1 poses cannot carry scale)")
			}
			FormatError::BadState { state, reason } => write!(f, "assembly state '{state}': {reason}"),
		}
	}
}

impl std::error::Error for FormatError {
	fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
		match self {
			FormatError::Parse(e) => Some(e),
			FormatError::Io { error, .. } => Some(error),
			FormatError::PartSource { error, .. } => Some(error),
			FormatError::SubAssembly { error, .. } => Some(error),
			_ => None,
		}
	}
}

/// A part's material for BOM mass rollups: a human name plus the density used
/// to turn engine volume into mass (`unit_mass_g = density_g_cm3 × volume_cm3`).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Material {
	/// Material name (e.g. `"steel"`, `"brass"`, `"PETG"`).
	pub name: String,
	/// Density in g/cm³ (e.g. steel 7.85, brass 8.4).
	pub density_g_cm3: f64,
}

/// Sourcing class of a part for the BOM: made in-house or bought.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum MakeOrBuy {
	/// Manufactured from this part's own recipe (printed/machined).
	Make,
	/// Purchased (catalog hardware: bearings, fasteners, …).
	Buy,
}

impl MakeOrBuy {
	/// The file/CSV token: `"make"` / `"buy"`.
	pub fn as_str(self) -> &'static str {
		match self {
			MakeOrBuy::Make => "make",
			MakeOrBuy::Buy => "buy",
		}
	}
}

/// The optional `meta` block of a `.lmcpart` envelope (BOM v2): engineering
/// metadata the geometry kernel never needs but a bill of materials does.
/// Every field is independently optional; an absent block loads as `None` and
/// is omitted on save, so meta-less files stay byte-identical to the pre-meta
/// format. Stamp it with [`save_part_with_meta`].
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct PartBomMeta {
	/// The organization's part number (free-form identity string).
	#[serde(default, skip_serializing_if = "Option::is_none")]
	pub part_number: Option<String>,
	/// Material + density; presence enables the BOM's mass columns.
	#[serde(default, skip_serializing_if = "Option::is_none")]
	pub material: Option<Material>,
	/// Make-or-buy sourcing class.
	#[serde(default, skip_serializing_if = "Option::is_none")]
	pub make_or_buy: Option<MakeOrBuy>,
}

/// The descriptive header of a loaded `.lmcpart` (everything except the
/// document itself). `units` is always `"mm"` in v1 (checked on load).
#[derive(Clone, Debug, PartialEq)]
pub struct PartMeta {
	/// The part's human name (empty if the envelope omitted it).
	pub name: String,
	/// Unit system of the file.
	pub units: String,
	/// What wrote the file (empty if the envelope omitted it).
	pub created_with: String,
	/// The envelope's optional `meta` block (`None` when absent — every file
	/// written before BOM v2 reads back exactly this way).
	pub meta: Option<PartBomMeta>,
}

// --- Serde mirrors of the two envelopes -------------------------------------------

/// Borrowing serializer mirror of a `.lmcpart` envelope (field order = file order).
#[derive(Serialize)]
struct PartFileSer<'a> {
	format: &'static str,
	version: u32,
	units: &'static str,
	name: &'a str,
	created_with: &'a str,
	#[serde(skip_serializing_if = "Option::is_none")]
	meta: Option<&'a PartBomMeta>,
	document: &'a Document,
}

impl<'a> PartFileSer<'a> {
	fn new(document: &'a Document, name: &'a str, meta: Option<&'a PartBomMeta>) -> Self {
		PartFileSer { format: PART_FORMAT, version: FORMAT_VERSION, units: UNITS_MM, name, created_with: CREATED_WITH, meta, document }
	}
}

/// Owning deserializer mirror of a `.lmcpart` envelope. The contract fields are
/// `Option` so a foreign/hand-written file reports "missing" precisely instead
/// of a serde missing-field error; `name`/`created_with` are merely descriptive
/// and default to empty; `meta` is the optional BOM v2 block.
#[derive(Deserialize)]
struct PartFileDe {
	format: Option<String>,
	version: Option<u32>,
	#[serde(default)]
	units: String,
	#[serde(default)]
	name: String,
	#[serde(default)]
	created_with: String,
	#[serde(default)]
	meta: Option<PartBomMeta>,
	document: Document,
}

/// The contract probe: parses ONLY the envelope header (unknown fields ignored),
/// so format/version/units can be validated before the heavy payload.
#[derive(Deserialize)]
struct Probe {
	format: Option<String>,
	version: Option<u32>,
	#[serde(default)]
	units: String,
}

/// Validate the three contract fields of an envelope against `expected`.
fn validate_envelope(expected: &'static str, format: Option<&str>, version: Option<u32>, units: &str) -> Result<(), FormatError> {
	match format {
		Some(f) if f == expected => {}
		found => return Err(FormatError::WrongFormat { expected, found: found.map(str::to_string) }),
	}
	match version {
		Some(FORMAT_VERSION) => {}
		found => return Err(FormatError::UnsupportedVersion { found, supported: FORMAT_VERSION }),
	}
	if units != UNITS_MM {
		return Err(FormatError::UnsupportedUnits { found: units.to_string() });
	}
	Ok(())
}

/// Parse just the header of `json` and validate it as an `expected` envelope.
fn check_envelope(json: &str, expected: &'static str) -> Result<(), FormatError> {
	let probe: Probe = serde_json::from_str(json).map_err(FormatError::Parse)?;
	validate_envelope(expected, probe.format.as_deref(), probe.version, &probe.units)
}

// --- .lmcpart ----------------------------------------------------------------------

/// Serialize `doc` as a `.lmcpart` file (pretty JSON). Byte-stable: the same
/// document and name always yield identical bytes, so saved parts git-diff
/// cleanly. Geometry is not stored — the document re-evaluates on load. Writes
/// **no** `meta` block (byte-identical to the pre-BOM-v2 format); to stamp
/// part-number/material/sourcing metadata use [`save_part_with_meta`].
pub fn save_part(doc: &Document, name: &str) -> String {
	save_part_with_meta(doc, name, None)
}

/// [`save_part`] plus the optional BOM v2 `meta` block ([`PartBomMeta`]: part
/// number, material/density, make-or-buy). `None` writes byte-identical files
/// to [`save_part`]; `Some` adds one `"meta"` object between `created_with`
/// and `document`. Byte-stable either way.
pub fn save_part_with_meta(doc: &Document, name: &str, meta: Option<&PartBomMeta>) -> String {
	// Infallible like `Document::save_json`: string keys and plain data only.
	serde_json::to_string_pretty(&PartFileSer::new(doc, name, meta)).expect("a part envelope serializes: string keys only, plain data")
}

/// Parse a `.lmcpart` string back into its [`Document`] plus the envelope's
/// descriptive [`PartMeta`]. The contract fields are checked first, so a
/// non-part file fails with [`FormatError::WrongFormat`] /
/// [`FormatError::UnsupportedVersion`] / [`FormatError::UnsupportedUnits`]
/// before any document parsing; schema mismatches inside the document are
/// [`FormatError::Parse`]. The document is NOT evaluated here — call
/// [`Document::evaluate_brep`] / [`Document::mesh`] on the result.
pub fn load_part(json: &str) -> Result<(Document, PartMeta), FormatError> {
	check_envelope(json, PART_FORMAT)?;
	let file: PartFileDe = serde_json::from_str(json).map_err(FormatError::Parse)?;
	let meta = PartMeta { name: file.name, units: file.units, created_with: file.created_with, meta: file.meta };
	Ok((file.document, meta))
}

// --- .lmcasm -----------------------------------------------------------------------

/// Where an assembly instance's geometry comes from, for [`save_assembly`].
// An inline `Part` carries a whole Document; the path variants are thin. The
// asymmetry is inherent to "embed vs reference" and these live in short,
// save-side slices, so boxing would only complicate every constructor.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Debug)]
pub enum AsmSource {
	/// Reference an external `.lmcpart` by path **relative to the assembly
	/// file's directory** (the `base_dir` of [`load_assembly`]). The caller is
	/// responsible for writing that file (via [`save_part`] /
	/// [`save_part_with_meta`]).
	Path(String),
	/// Embed the part inline as a full `.lmcpart` envelope — a single
	/// self-contained assembly file.
	Part {
		/// The embedded part's name (the envelope's `name` field).
		name: String,
		/// The embedded part's document.
		document: Document,
		/// The embedded part's optional BOM v2 `meta` block.
		meta: Option<PartBomMeta>,
	},
	/// Reference an external `.lmcasm` **sub-assembly** by path relative to
	/// the assembly file's directory (file key `"asm_path"`). Loading resolves
	/// it recursively and places it as one rigid unit — see the module docs'
	/// *Assembly nesting* section for the precise semantics. v2 limit:
	/// sub-assemblies are path-referenced only (no inline assembly envelope).
	Assembly(String),
	/// Reference a triangle-mesh file (`.stl` / `.obj` / `.3mf` / `.ply`,
	/// sniffed by extension) by path relative to the assembly file's directory
	/// (file key `"mesh"`). Loading welds it and lifts it through the
	/// winding-number [`kernel_implicit::MeshSdf`] ([`Instance::from_mesh`]),
	/// so a part authored on the flat JSON **program** surface — or scanned, or
	/// imported — participates in mates, contacts, clearance and BOM like any
	/// document part. HONEST LIMITS: measurement runs on the welded mesh at the
	/// pipeline's voxel fallback (no exact analytic surfaces), and the BOM
	/// reports `volume_source: "mesh"`, never `"exact"`.
	Mesh(String),
}

/// One instance of an assembly being saved: an optional human name, the part
/// [`AsmSource`], the rigid pose, and whether the instance is suppressed.
#[derive(Clone, Debug)]
pub struct AsmInstance {
	/// Optional instance name (distinct from the part's name — two screws share
	/// a part name but have their own instance names).
	pub name: Option<String>,
	/// Where the part comes from.
	pub source: AsmSource,
	/// Rigid local→world pose (translation + rotation; scale is rejected).
	pub pose: Affine3A,
	/// Whether the instance is suppressed (kept in the file, contributes no
	/// geometry on load — see [`Assembly::set_instance_suppressed`]). Omitted
	/// from the file when `false`.
	pub suppressed: bool,
}

/// One node of a loaded assembly's **instance hierarchy** (assembly nesting,
/// v2): a placed part or a placed sub-assembly, mirroring the `.lmcasm`
/// nesting one node per file instance. Built by [`load_assembly`]; consumed by
/// [`LoadedAssembly::bom_tree`] and anything that wants to walk the structure.
#[derive(Clone, Debug)]
pub struct AsmNode {
	/// Display name of the instance at this level — the file's instance name,
	/// or the synthesized `"#<index>"` when the file gave none.
	pub instance: String,
	/// What is placed: the part envelope's name (for a part) or the
	/// sub-assembly envelope's name / file stem (for a sub-assembly).
	pub name: String,
	/// For a part: its index into [`LoadedAssembly::assembly`]`.instances`.
	/// `None` for a sub-assembly node (its parts are in `children`).
	pub leaf: Option<usize>,
	/// The **file's** suppression flag at this level. (Live suppression of leaf
	/// parts — [`Assembly::set_instance_suppressed`] — is queried from the
	/// assembly instead; this records what the envelope said.)
	pub suppressed: bool,
	/// Member nodes of a sub-assembly, recursively. Empty for a part.
	pub children: Vec<AsmNode>,
}

/// Solve-time record of one **top-level unit** of a loaded assembly: the
/// file-instance pose plus its leaf members' local poses within the unit
/// (`None` local = the unit IS that single leaf, so the pose is copied
/// bit-exactly instead of multiplied by an identity).
struct UnitPlacement {
	/// The unit's current pose (what the parent's mates solve over).
	pose: Affine3A,
	/// `(leaf index, local-in-unit pose)` of every member part.
	members: Vec<(usize, Option<Affine3A>)>,
}

/// A loaded `.lmcasm`: the live [`Assembly`] (parts rebuilt — sub-assemblies
/// recursively, flattened to leaf parts — poses applied, per-instance
/// suppression set, mates re-solved) plus everything else the file said.
pub struct LoadedAssembly {
	/// The assembly's name (empty if the envelope omitted it).
	pub name: String,
	/// Unit system of the file (always `"mm"` in v1).
	pub units: String,
	/// Per-leaf instance names, parallel to `assembly.instances`. Members of a
	/// sub-assembly get **hierarchical** names joined with `/` (e.g.
	/// `"stage1/bearing_l"`, with `"#<index>"` standing in for unnamed levels);
	/// top-level parts keep exactly the file's optional name.
	pub instance_names: Vec<Option<String>>,
	/// Per-leaf **part** names, parallel to `assembly.instances`: the
	/// referenced/embedded part envelope's `name` (falling back to a path
	/// source's file stem when the envelope left it empty). This is the "what is
	/// it" name a [`bom`](LoadedAssembly::bom) line groups by, distinct from the
	/// per-instance names above.
	pub part_names: Vec<String>,
	/// Per-leaf optional BOM metadata, parallel to `assembly.instances`: the
	/// part envelope's `meta` block ([`PartBomMeta`]), `None` where absent.
	pub part_meta: Vec<Option<PartBomMeta>>,
	/// The instance **hierarchy**, one node per file instance (recursive for
	/// sub-assemblies). For a flat assembly this is one part node per instance.
	pub tree: Vec<AsmNode>,
	/// The instantiated assembly — **flattened to leaf parts** — with poses
	/// already re-solved against `mates`.
	pub assembly: Assembly,
	/// The mates from the file. Their indices are the file's **top-level
	/// instances** (parts or whole sub-assembly units), so re-apply them via
	/// [`LoadedAssembly::solve_mates`] after editing poses or parameters — NOT
	/// via `assembly.solve_mates`, whose indices are flattened leaves (the two
	/// are equivalent only for an assembly without sub-assemblies).
	pub mates: Vec<Constraint>,
	/// The file's named states (pose/suppression snapshots), ready for
	/// [`Assembly::apply_state`] — already expanded to leaf level (a state pose
	/// of a sub-assembly instance moves the whole unit rigidly; suppression of
	/// one suppresses all its leaves). Empty when the file declared none.
	pub states: BTreeMap<String, AsmState>,
	/// Residual of the on-load mate solve: the **maximum** across this
	/// assembly's own mates and every sub-assembly's internal solve (`~0` ⇒ all
	/// levels satisfied; `0.0` when no level has mates).
	pub residual: f64,
	/// Solve bookkeeping: one entry per top-level unit (see [`UnitPlacement`]).
	placements: Vec<UnitPlacement>,
	/// Max internal residual over all sub-assemblies (their mates are solved at
	/// their own load and frozen — a sub-assembly is rigid in the parent).
	sub_residual: f64,
}

/// The BOM schema tag written by [`BomV2::to_json`] and the `bom` report stage.
pub const BOM_SCHEMA: &str = "bom/2";
/// Voxel size (mm) used by the parameterless BOM calls ([`LoadedAssembly::bom`]
/// / [`LoadedAssembly::bom_json`]) for the **mesh-route** volume of parts with
/// no exact B-rep — the same default as the `kernel-api asm` CLI.
pub const BOM_DEFAULT_VOXEL: f32 = 0.4;

/// Where a BOM line's mass-driving volume came from — the honest routing label
/// (BAR.md doctrine: never silently degrade).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum VolumeSource {
	/// Analytic B-rep volume ([`kernel_brep::exact_volume`]) — machine-exact
	/// for planar/cylinder/sphere/cone faces (and closed torus bands).
	Exact,
	/// Voxel-meshed volume (implicit/organic part, no exact B-rep) — accurate
	/// to the meshing voxel size, not exact.
	Mesh,
}

impl VolumeSource {
	/// The file/CSV token: `"exact"` / `"mesh"`.
	pub fn as_str(self) -> &'static str {
		match self {
			VolumeSource::Exact => "exact",
			VolumeSource::Mesh => "mesh",
		}
	}
}

/// One grouped line of a bill of materials (see [`LoadedAssembly::bom`]):
/// `count` instances of the part `name` built with the given parameter
/// summary. The optional BOM v2 fields are populated from the part envelope's
/// `meta` block ([`PartBomMeta`]) and are omitted from JSON when absent, so a
/// meta-less assembly's lines serialize exactly as they did before v2.
#[derive(Clone, Debug, Default, PartialEq, Serialize)]
pub struct BomLine {
	/// The part's name (see [`LoadedAssembly::part_names`]).
	pub name: String,
	/// How many unsuppressed instances of this exact part the assembly places.
	pub count: usize,
	/// The part document's parameters as a sorted `"k=v, …"` summary (empty for
	/// a document without named parameters), so two same-named parts built to
	/// different dimensions stay separate BOM lines.
	pub params: String,
	/// Part number from the part's `meta` block.
	#[serde(skip_serializing_if = "Option::is_none")]
	pub part_number: Option<String>,
	/// Material (name + density) from the part's `meta` block.
	#[serde(skip_serializing_if = "Option::is_none")]
	pub material: Option<Material>,
	/// Which volume fed the mass: `"exact"` (analytic B-rep) or `"mesh"`
	/// (voxel route). Present exactly when the mass columns are.
	#[serde(skip_serializing_if = "Option::is_none")]
	pub volume_source: Option<VolumeSource>,
	/// Mass of ONE instance in grams: `density_g_cm3 × volume_cm3`. Present
	/// when the part has a material and produces geometry.
	#[serde(skip_serializing_if = "Option::is_none")]
	pub unit_mass_g: Option<f64>,
	/// Mass of the whole line: `unit_mass_g × count`.
	#[serde(skip_serializing_if = "Option::is_none")]
	pub line_mass_g: Option<f64>,
	/// Make-or-buy sourcing class from the part's `meta` block.
	#[serde(skip_serializing_if = "Option::is_none")]
	pub make_or_buy: Option<MakeOrBuy>,
}

/// One node of the BOM **tree view** (see [`LoadedAssembly::bom_tree`]): the
/// per-instance nesting structure with rolled-up part counts.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct BomNode {
	/// The instance's display name at its level (`"#<index>"` when unnamed).
	pub instance: String,
	/// The part name (leaf) or sub-assembly name (branch).
	pub name: String,
	/// Rolled-up number of unsuppressed **leaf parts** in this branch: `1` for
	/// a part, the sum over `children` for a sub-assembly.
	pub count: usize,
	/// Member nodes (empty — and omitted from JSON — for a part).
	#[serde(skip_serializing_if = "Vec::is_empty")]
	pub children: Vec<BomNode>,
}

/// The complete BOM v2 payload — what `bom.json` holds and the `bom` report
/// stage carries: the grouped `flat` view (the ERP lines) plus the `tree` view
/// (the nesting structure). Built by [`LoadedAssembly::bom_v2`].
#[derive(Clone, Debug, Serialize)]
pub struct BomV2 {
	/// Schema tag, always [`BOM_SCHEMA`] (`"bom/2"`).
	pub schema: &'static str,
	/// Grouped flat lines, sorted by name then parameter summary.
	pub flat: Vec<BomLine>,
	/// Per-instance tree, in file instance order, suppressed branches dropped.
	pub tree: Vec<BomNode>,
}

impl BomV2 {
	/// This BOM as pretty JSON — `{"schema": "bom/2", "flat": […], "tree": […]}`,
	/// byte-stable (lines sorted, tree in file order), for `bom.json`.
	pub fn to_json(&self) -> String {
		// Infallible: plain string/number data throughout.
		serde_json::to_string_pretty(self).expect("a BOM serializes: plain data")
	}

	/// The flat view as CSV — the ERP hand-off. One header row, then one row
	/// per [`BomLine`] in the same order, columns fixed as
	/// `name,count,params,part_number,material,density_g_cm3,volume_source,unit_mass_g,line_mass_g,make_or_buy`
	/// (material = the material's name; absent optionals are empty fields).
	/// RFC-4180 quoting: fields containing `,`, `"` or newlines are quoted with
	/// inner quotes doubled. `\n` line endings, trailing newline. Byte-stable.
	pub fn to_csv(&self) -> String {
		let mut out = String::from("name,count,params,part_number,material,density_g_cm3,volume_source,unit_mass_g,line_mass_g,make_or_buy\n");
		for line in &self.flat {
			let fields = [
				csv_field(&line.name),
				line.count.to_string(),
				csv_field(&line.params),
				csv_field(line.part_number.as_deref().unwrap_or("")),
				csv_field(line.material.as_ref().map(|m| m.name.as_str()).unwrap_or("")),
				line.material.as_ref().map(|m| m.density_g_cm3.to_string()).unwrap_or_default(),
				line.volume_source.map(VolumeSource::as_str).unwrap_or("").to_string(),
				line.unit_mass_g.map(|m| m.to_string()).unwrap_or_default(),
				line.line_mass_g.map(|m| m.to_string()).unwrap_or_default(),
				line.make_or_buy.map(MakeOrBuy::as_str).unwrap_or("").to_string(),
			];
			out.push_str(&fields.join(","));
			out.push('\n');
		}
		out
	}
}

/// RFC-4180 CSV field: quoted (inner `"` doubled) when it contains a comma,
/// quote or newline; verbatim otherwise.
fn csv_field(s: &str) -> String {
	if s.contains(',') || s.contains('"') || s.contains('\n') || s.contains('\r') {
		format!("\"{}\"", s.replace('"', "\"\""))
	} else {
		s.to_string()
	}
}

impl LoadedAssembly {
	/// The assembly's **bill of materials** (flat view): leaf-part instances
	/// grouped by part identity — the part's name plus its document's parameter
	/// summary — with one [`BomLine`] per distinct part, sorted by name then
	/// parameters. Suppressed instances are excluded (a suppressed component is
	/// absent material, exactly as in [`Assembly::mass_properties`]) — and so is
	/// every leaf of a suppressed sub-assembly; re-grouping after toggling
	/// suppression just means calling this again. Mesh-route volumes (parts with
	/// no exact B-rep) use [`BOM_DEFAULT_VOXEL`]; pass an explicit voxel via
	/// [`bom_v2`](LoadedAssembly::bom_v2).
	pub fn bom(&self) -> Vec<BomLine> {
		self.bom_v2(BOM_DEFAULT_VOXEL).flat
	}

	/// The complete **BOM v2** payload: the grouped [`flat`](BomV2::flat) view
	/// plus the per-instance [`tree`](BomV2::tree) view. Where a part's envelope
	/// carries `meta`, the flat line is enriched with its part number, material,
	/// sourcing class and **mass** — `unit_mass_g = density_g_cm3 ×
	/// engine-volume(cm³)`, the volume taken analytically
	/// ([`kernel_brep::exact_volume`], labeled `"exact"`) when the part has an
	/// exact B-rep, else from its voxel mesh at `voxel` mm (labeled `"mesh"` —
	/// the honest routing verdict). A part with material but no geometry gets no
	/// mass columns. Identical-identity instances share the first instance's
	/// meta. Deterministic: same assembly ⇒ byte-identical
	/// [`BomV2::to_json`] / [`BomV2::to_csv`].
	pub fn bom_v2(&self, voxel: f32) -> BomV2 {
		// Group exactly as BOM v1: (part name, parameter summary), leaf order.
		let mut groups: BTreeMap<(String, String), (usize, usize)> = BTreeMap::new();
		for (index, instance) in self.assembly.instances.iter().enumerate() {
			if self.assembly.is_instance_suppressed(index) {
				continue;
			}
			let name = self.part_names.get(index).cloned().unwrap_or_default();
			let params = match &instance.source {
				Source::Doc(doc) => params_summary(doc),
				Source::Built(_) => String::new(),
			};
			groups.entry((name, params)).or_insert((0, index)).0 += 1;
		}
		let flat = groups
			.into_iter()
			.map(|((name, params), (count, first))| {
				let mut line = BomLine { name, count, params, ..BomLine::default() };
				if let Some(meta) = self.part_meta.get(first).and_then(Option::as_ref) {
					line.part_number = meta.part_number.clone();
					line.make_or_buy = meta.make_or_buy;
					line.material = meta.material.clone();
					if let Some(material) = &meta.material {
						if let Some((volume_mm3, source)) = self.unit_volume(first, voxel) {
							let unit = material.density_g_cm3 * volume_mm3 / 1000.0;
							line.volume_source = Some(source);
							line.unit_mass_g = Some(unit);
							line.line_mass_g = Some(unit * count as f64);
						}
					}
				}
				line
			})
			.collect();
		BomV2 { schema: BOM_SCHEMA, flat, tree: self.bom_tree() }
	}

	/// The BOM **tree view**: one [`BomNode`] per file instance in order,
	/// mirroring the nesting ([`LoadedAssembly::tree`]), with suppressed
	/// branches dropped entirely and each sub-assembly's `count` rolled up to
	/// its number of unsuppressed leaf parts. The sum of top-level counts equals
	/// the sum of the flat view's counts.
	pub fn bom_tree(&self) -> Vec<BomNode> {
		fn build(loaded: &LoadedAssembly, nodes: &[AsmNode]) -> Vec<BomNode> {
			let mut out = Vec::new();
			for node in nodes {
				if node.suppressed {
					continue; // the file suppressed this whole branch
				}
				match node.leaf {
					Some(leaf) => {
						if loaded.assembly.is_instance_suppressed(leaf) {
							continue; // file or live suppression of a part
						}
						out.push(BomNode { instance: node.instance.clone(), name: node.name.clone(), count: 1, children: Vec::new() });
					}
					None => {
						let children = build(loaded, &node.children);
						let count = children.iter().map(|c| c.count).sum();
						out.push(BomNode { instance: node.instance.clone(), name: node.name.clone(), count, children });
					}
				}
			}
			out
		}
		build(self, &self.tree)
	}

	/// The [BOM v2](LoadedAssembly::bom_v2) as pretty JSON — machine-readable
	/// and byte-stable, for handing to procurement tooling without another
	/// schema. Mesh-route volumes use [`BOM_DEFAULT_VOXEL`].
	pub fn bom_json(&self) -> String {
		self.bom_v2(BOM_DEFAULT_VOXEL).to_json()
	}

	/// Local volume (mm³) of leaf `leaf`'s part plus the honest route label:
	/// analytic [`kernel_brep::exact_volume`] when the document evaluates to an
	/// exact B-rep, else the signed volume of its voxel mesh at `voxel` mm.
	/// `None` when the part produces no geometry at all.
	fn unit_volume(&self, leaf: usize, voxel: f32) -> Option<(f64, VolumeSource)> {
		let instance = self.assembly.instances.get(leaf)?;
		if let Source::Doc(doc) = &instance.source {
			if let Some(solid) = doc.evaluate_brep() {
				return Some((kernel_brep::exact_volume(&solid), VolumeSource::Exact));
			}
		}
		// Voxel route. The mesh is world-posed; poses are rigid, so the volume
		// is the part's own.
		let mesh = instance.mesh(Resolution::VoxelSize(voxel));
		(mesh.triangle_count() > 0).then(|| (mesh.signed_volume(), VolumeSource::Mesh))
	}

	/// Re-solve this assembly's own `mates` over its **top-level units** (each
	/// part or whole sub-assembly one rigid body, exactly as on load), write the
	/// solved poses through to every leaf, and return the residual — combined
	/// with the frozen internal residuals of the sub-assemblies, so it is
	/// comparable to [`LoadedAssembly::residual`]. Sub-assemblies' internal
	/// mates are NOT re-solved (they were solved at their own load; a
	/// sub-assembly is rigid in the parent — v2 semantics).
	/// Mate honesty receipts at the CURRENT top-level poses: the static
	/// diagnostics ([`ConstraintSystem::validate`] — mates the solver would
	/// silently skip), the per-mate squared residuals (WHICH mate is
	/// unsatisfied), and the numeric DOF report
	/// ([`ConstraintSystem::analyze`] — remaining free rigid-body motions).
	pub fn mate_receipts(&self) -> (Vec<String>, Vec<f64>, crate::DofReport) {
		let system = ConstraintSystem::new(self.placements.iter().map(|p| p.pose).collect(), self.mates.clone());
		(system.validate(), system.per_constraint_residuals(), system.analyze())
	}

	pub fn solve_mates(&mut self, iterations: usize) -> f64 {
		let mut system = ConstraintSystem::new(self.placements.iter().map(|p| p.pose).collect(), self.mates.clone());
		let residual = system.solve(iterations);
		for (placement, &pose) in self.placements.iter_mut().zip(system.transforms()) {
			placement.pose = pose;
			for &(leaf, local) in &placement.members {
				if let Some(instance) = self.assembly.instances.get_mut(leaf) {
					instance.pose = match local {
						None => pose,
						Some(local) => pose * local,
					};
				}
			}
		}
		residual.max(self.sub_residual)
	}
}

/// A document's parameters as the sorted `"k=v, …"` BOM summary.
fn params_summary(doc: &Document) -> String {
	let sorted: BTreeMap<&str, f64> = doc.params_iter().collect();
	let mut out = String::new();
	for (k, v) in sorted {
		if !out.is_empty() {
			out.push_str(", ");
		}
		out.push_str(&format!("{k}={v}"));
	}
	out
}

/// JSON mirror of a rigid pose: translation plus an optional `[x, y, z, w]`
/// unit quaternion (omitted when the rotation is the identity). A missing
/// `pose` altogether is the identity.
#[derive(Serialize, Deserialize, Default)]
struct PoseRepr {
	#[serde(default)]
	translation: [f32; 3],
	#[serde(default, skip_serializing_if = "Option::is_none")]
	rotation: Option<[f32; 4]>,
}

/// Decompose a live pose for saving; rejects scale/non-finite (v1 is rigid).
fn pose_to_repr(pose: &Affine3A, instance: usize) -> Result<PoseRepr, FormatError> {
	let (scale, rotation, translation) = pose.to_scale_rotation_translation();
	if !translation.is_finite() || !rotation.is_finite() || (scale - Vec3::ONE).abs().max_element() > 1e-4 {
		return Err(FormatError::BadPose { instance });
	}
	let rotation = (rotation != Quat::IDENTITY).then(|| [rotation.x, rotation.y, rotation.z, rotation.w]);
	Ok(PoseRepr { translation: [translation.x, translation.y, translation.z], rotation })
}

/// Rebuild a pose from the file; tolerates a hand-typed not-quite-unit
/// quaternion (normalized), rejects zero/non-finite ones loudly.
fn pose_from_repr(repr: &PoseRepr, instance: usize) -> Result<Affine3A, FormatError> {
	let translation = Vec3::from(repr.translation);
	if !translation.is_finite() {
		return Err(FormatError::BadPose { instance });
	}
	let rotation = match repr.rotation {
		None => Quat::IDENTITY,
		Some([x, y, z, w]) => {
			let q = Quat::from_xyzw(x, y, z, w);
			if !q.is_finite() || q.length_squared() < 1e-6 {
				return Err(FormatError::BadPose { instance });
			}
			q.normalize()
		}
	};
	Ok(Affine3A::from_rotation_translation(rotation, translation))
}

/// `skip_serializing_if` helper: omit a `false` flag from the file.
fn is_false(flag: &bool) -> bool {
	!*flag
}

/// JSON mirror of one named assembly state (poses by [`PoseRepr`]).
#[derive(Serialize, Deserialize)]
struct AsmStateRepr {
	poses: Vec<PoseRepr>,
	#[serde(default, skip_serializing_if = "Vec::is_empty")]
	suppressed: Vec<usize>,
}

/// Borrowing serializer mirror of one `.lmcasm` instance.
#[derive(Serialize)]
struct InstanceSer<'a> {
	#[serde(skip_serializing_if = "Option::is_none")]
	name: Option<&'a str>,
	#[serde(skip_serializing_if = "is_false")]
	suppressed: bool,
	source: SourceSer<'a>,
	pose: PoseRepr,
}

/// Borrowing serializer mirror of an instance source: `{"path": …}`,
/// `{"part": …full envelope…}` or `{"asm_path": …}`.
#[derive(Serialize)]
enum SourceSer<'a> {
	#[serde(rename = "path")]
	Path(&'a str),
	#[serde(rename = "part")]
	Part(PartFileSer<'a>),
	#[serde(rename = "asm_path")]
	AsmPath(&'a str),
	#[serde(rename = "mesh")]
	Mesh(&'a str),
}

/// Borrowing serializer mirror of a `.lmcasm` envelope.
#[derive(Serialize)]
struct AsmFileSer<'a> {
	format: &'static str,
	version: u32,
	units: &'static str,
	name: &'a str,
	instances: Vec<InstanceSer<'a>>,
	mates: &'a [Constraint],
	#[serde(skip_serializing_if = "BTreeMap::is_empty")]
	states: BTreeMap<&'a str, AsmStateRepr>,
}

/// Owning deserializer mirrors of the `.lmcasm` payload (the contract header is
/// validated separately by [`check_envelope`]).
#[derive(Deserialize)]
struct AsmFileDe {
	#[serde(default)]
	name: String,
	#[serde(default)]
	units: String,
	#[serde(default)]
	instances: Vec<InstanceDe>,
	#[serde(default)]
	mates: Vec<Constraint>,
	#[serde(default)]
	states: BTreeMap<String, AsmStateRepr>,
}

#[derive(Deserialize)]
struct InstanceDe {
	#[serde(default)]
	name: Option<String>,
	#[serde(default)]
	suppressed: bool,
	source: SourceDe,
	#[serde(default)]
	pose: PoseRepr,
}

#[derive(Deserialize)]
enum SourceDe {
	#[serde(rename = "path")]
	Path(String),
	// Boxed: a full part envelope dwarfs the path variant.
	#[serde(rename = "part")]
	Part(Box<PartFileDe>),
	#[serde(rename = "asm_path")]
	AsmPath(String),
	#[serde(rename = "mesh")]
	Mesh(String),
}

/// Serialize an assembly as a `.lmcasm` file (pretty JSON). Byte-stable like
/// [`save_part`]. Fails ONLY when an instance pose carries scale or is
/// non-finite ([`FormatError::BadPose`]) — v1 poses are rigid and the format
/// never silently drops a component. Mates are stored as data and re-solved on
/// load; save them post-solve (e.g. straight after [`Assembly::solve_mates`])
/// so the stored poses are a converged seed. Per-instance suppression
/// ([`AsmInstance::suppressed`]) persists; to also persist named states use
/// [`save_assembly_with_states`].
pub fn save_assembly(name: &str, instances: &[AsmInstance], mates: &[Constraint]) -> Result<String, FormatError> {
	save_assembly_with_states(name, instances, mates, &BTreeMap::new())
}

/// [`save_assembly`] plus named **assembly states** (pose/suppression snapshots,
/// e.g. from [`Assembly::capture_state`]) persisted under their names. Each
/// state must fit the assembly — one rigid pose per instance and in-range
/// suppressed indices — or saving fails with [`FormatError::BadState`] (a state
/// that cannot be re-applied must not be written). An empty map writes the same
/// bytes as [`save_assembly`].
pub fn save_assembly_with_states(
	name: &str,
	instances: &[AsmInstance],
	mates: &[Constraint],
	states: &BTreeMap<String, AsmState>,
) -> Result<String, FormatError> {
	let mut reprs = Vec::with_capacity(instances.len());
	for (index, instance) in instances.iter().enumerate() {
		let source = match &instance.source {
			AsmSource::Path(path) => SourceSer::Path(path),
			AsmSource::Part { name, document, meta } => SourceSer::Part(PartFileSer::new(document, name, meta.as_ref())),
			AsmSource::Assembly(path) => SourceSer::AsmPath(path),
			AsmSource::Mesh(path) => SourceSer::Mesh(path),
		};
		reprs.push(InstanceSer {
			name: instance.name.as_deref(),
			suppressed: instance.suppressed,
			source,
			pose: pose_to_repr(&instance.pose, index)?,
		});
	}
	let mut state_reprs = BTreeMap::new();
	for (state_name, state) in states {
		if state.poses.len() != instances.len() {
			return Err(FormatError::BadState {
				state: state_name.clone(),
				reason: format!("has {} poses for {} instances", state.poses.len(), instances.len()),
			});
		}
		if let Some(&bad) = state.suppressed.iter().find(|&&i| i >= instances.len()) {
			return Err(FormatError::BadState {
				state: state_name.clone(),
				reason: format!("suppresses instance {bad}, but there are only {} instances", instances.len()),
			});
		}
		let mut poses = Vec::with_capacity(state.poses.len());
		for (index, pose) in state.poses.iter().enumerate() {
			poses.push(pose_to_repr(pose, index).map_err(|_| FormatError::BadState {
				state: state_name.clone(),
				reason: format!("pose {index} is not a finite rigid translation+rotation"),
			})?);
		}
		let mut suppressed = state.suppressed.clone();
		suppressed.sort_unstable();
		suppressed.dedup();
		state_reprs.insert(state_name.as_str(), AsmStateRepr { poses, suppressed });
	}
	let file = AsmFileSer {
		format: ASM_FORMAT,
		version: FORMAT_VERSION,
		units: UNITS_MM,
		name,
		instances: reprs,
		mates,
		states: state_reprs,
	};
	// Infallible for the same reason as `save_part` once the poses validated.
	Ok(serde_json::to_string_pretty(&file).expect("an assembly envelope serializes: string keys only, plain data"))
}

/// What one file instance resolved to while loading: a leaf part or a whole
/// (recursively loaded) sub-assembly.
enum UnitKind {
	Part {
		document: Document,
		part_name: String,
		meta: Option<PartBomMeta>,
	},
	Sub {
		sub: LoadedAssembly,
		asm_name: String,
	},
	/// A welded triangle mesh (`"mesh"` source) — placed via
	/// [`Instance::from_mesh`]; BOM volume comes from the voxel fallback
	/// (`volume_source: "mesh"`, honest).
	MeshPart {
		mesh: Mesh,
		part_name: String,
	},
}

/// One file instance mid-load: name/suppression/seed pose plus what it is.
struct UnitBuild {
	name: Option<String>,
	suppressed: bool,
	seed: Affine3A,
	kind: UnitKind,
}

/// Shift every part node's leaf index by `offset` — a sub-assembly's leaves
/// land after the parent's earlier leaves in the flattened assembly.
fn rebase_tree_leaves(nodes: &mut [AsmNode], offset: usize) {
	for node in nodes {
		if let Some(leaf) = node.leaf.as_mut() {
			*leaf += offset;
		}
		rebase_tree_leaves(&mut node.children, offset);
	}
}

/// Parse a `.lmcasm` string and bring it to life: every `path` source is read
/// relative to `base_dir` (the directory the assembly file lives in) and loaded
/// via [`load_part`]; every inline `part` envelope is contract-checked the same
/// way; every `asm_path` source is loaded **recursively** the same way (its own
/// sources resolving against its own directory; include cycles are refused with
/// [`FormatError::AsmCycle`]). Each sub-assembly solves its own mates first and
/// is then placed as one rigid unit; each part document is placed as a live
/// parametric [`Instance`]; finally THIS file's mates are **re-solved** over
/// the top-level units (the stored poses seed the solver, the mates are the
/// authority) and the combined residual is reported. The result is flattened to
/// leaf parts with the hierarchy kept in [`LoadedAssembly::tree`] — see the
/// module docs' *Assembly nesting* section for the precise semantics. Parts
/// rebuild deterministically, so loading the same file twice yields the same
/// assembly.
pub fn load_assembly(json: &str, base_dir: &Path) -> Result<LoadedAssembly, FormatError> {
	load_assembly_nested(json, base_dir, &mut Vec::new())
}

/// The recursive worker behind [`load_assembly`]. `loading` is the stack of
/// canonicalized `.lmcasm` paths currently being loaded, innermost last — an
/// `asm_path` resolving onto any of them is an include cycle and fails loudly
/// (two *sibling* instances of the same sub-assembly are fine: each load
/// pushes and pops around its own recursion only).
fn load_assembly_nested(json: &str, base_dir: &Path, loading: &mut Vec<PathBuf>) -> Result<LoadedAssembly, FormatError> {
	check_envelope(json, ASM_FORMAT)?;
	let file: AsmFileDe = serde_json::from_str(json).map_err(FormatError::Parse)?;
	let unit_count = file.instances.len();

	// --- resolve every file instance into a top-level unit ---------------------
	let mut units: Vec<UnitBuild> = Vec::with_capacity(unit_count);
	for (index, instance) in file.instances.into_iter().enumerate() {
		let kind = match instance.source {
			SourceDe::Path(rel) => {
				let path = base_dir.join(&rel);
				let text = std::fs::read_to_string(&path).map_err(|error| FormatError::Io { path: path.clone(), error })?;
				let (document, meta) = load_part(&text).map_err(|error| FormatError::PartSource { path: path.clone(), error: Box::new(error) })?;
				// The part envelope's own name identifies the part; an unnamed
				// envelope falls back to its file stem.
				let part_name = if meta.name.is_empty() {
					path.file_stem().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default()
				} else {
					meta.name
				};
				UnitKind::Part { document, part_name, meta: meta.meta }
			}
			SourceDe::Part(envelope) => {
				validate_envelope(PART_FORMAT, envelope.format.as_deref(), envelope.version, &envelope.units)?;
				UnitKind::Part { document: envelope.document, part_name: envelope.name, meta: envelope.meta }
			}
			SourceDe::Mesh(rel) => {
				let path = base_dir.join(&rel);
				let ext = path.extension().and_then(|e| e.to_str()).unwrap_or("").to_ascii_lowercase();
				let read = match ext.as_str() {
					"stl" => Mesh::read_stl(&path),
					"obj" => Mesh::read_obj(&path),
					"3mf" => Mesh::read_3mf(&path),
					"ply" => Mesh::read_ply(&path),
					other => Err(std::io::Error::new(
						std::io::ErrorKind::InvalidInput,
						format!("mesh source needs .stl/.obj/.3mf/.ply, got '.{other}'"),
					)),
				};
				let mut mesh = read.map_err(|error| FormatError::Io { path: path.clone(), error })?;
				mesh.weld(1e-6);
				if mesh.triangle_count() == 0 {
					return Err(FormatError::Io {
						path: path.clone(),
						error: std::io::Error::new(std::io::ErrorKind::InvalidData, "mesh source welded to zero triangles"),
					});
				}
				let part_name = path.file_stem().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default();
				UnitKind::MeshPart { mesh, part_name }
			}
			SourceDe::AsmPath(rel) => {
				let path = base_dir.join(&rel);
				// Canonicalize so the same file reached through different
				// relative spellings still closes the cycle check.
				let canon = std::fs::canonicalize(&path).map_err(|error| FormatError::Io { path: path.clone(), error })?;
				if loading.contains(&canon) {
					let mut chain = loading.clone();
					chain.push(canon.clone());
					return Err(FormatError::AsmCycle { path: canon, chain });
				}
				let text = std::fs::read_to_string(&canon).map_err(|error| FormatError::Io { path: path.clone(), error })?;
				loading.push(canon.clone());
				let sub = load_assembly_nested(&text, canon.parent().unwrap_or_else(|| Path::new(".")), loading);
				loading.pop();
				let sub = sub.map_err(|error| match error {
					// A cycle already names its full chain; re-wrapping each
					// level would only repeat it.
					cycle @ FormatError::AsmCycle { .. } => cycle,
					other => FormatError::SubAssembly { path: path.clone(), error: Box::new(other) },
				})?;
				let asm_name = if sub.name.is_empty() {
					path.file_stem().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default()
				} else {
					sub.name.clone()
				};
				UnitKind::Sub { sub, asm_name }
			}
		};
		let seed = pose_from_repr(&instance.pose, index)?;
		units.push(UnitBuild { name: instance.name, suppressed: instance.suppressed, seed, kind });
	}

	// --- solve THIS file's mates over the units (one rigid body each) ----------
	// For a flat assembly this is exactly the former whole-assembly solve: every
	// unit is a single leaf and its pose is copied through bit-identically.
	let mut system = ConstraintSystem::new(units.iter().map(|u| u.seed).collect(), file.mates.clone());
	let parent_residual = system.solve(MATE_SOLVE_ITERATIONS);
	let unit_poses: Vec<Affine3A> = system.transforms().to_vec();

	// --- flatten to leaves, keeping the hierarchy --------------------------------
	let mut assembly = Assembly::new();
	let mut instance_names = Vec::new();
	let mut part_names = Vec::new();
	let mut part_meta = Vec::new();
	let mut tree = Vec::with_capacity(units.len());
	let mut placements = Vec::with_capacity(units.len());
	// Leaves suppressed by a SUB-ASSEMBLY's own file — parent states cannot
	// address (or un-suppress) them, so every expanded state re-asserts them.
	let mut nested_suppressed: Vec<usize> = Vec::new();
	let mut sub_residual = 0.0_f64;
	for (k, (unit, &pose)) in units.into_iter().zip(&unit_poses).enumerate() {
		let display = unit.name.clone().unwrap_or_else(|| format!("#{k}"));
		match unit.kind {
			UnitKind::Part { document, part_name, meta } => {
				let leaf = assembly.add(Instance::document(document, pose));
				assembly.set_instance_suppressed(leaf, unit.suppressed);
				instance_names.push(unit.name);
				part_names.push(part_name.clone());
				part_meta.push(meta);
				tree.push(AsmNode { instance: display, name: part_name, leaf: Some(leaf), suppressed: unit.suppressed, children: Vec::new() });
				placements.push(UnitPlacement { pose, members: vec![(leaf, None)] });
			}
			UnitKind::MeshPart { mesh, part_name } => {
				let leaf = assembly.add(Instance::from_mesh(&mesh, pose));
				assembly.set_instance_suppressed(leaf, unit.suppressed);
				instance_names.push(unit.name);
				part_names.push(part_name.clone());
				part_meta.push(None);
				tree.push(AsmNode { instance: display, name: part_name, leaf: Some(leaf), suppressed: unit.suppressed, children: Vec::new() });
				placements.push(UnitPlacement { pose, members: vec![(leaf, None)] });
			}
			UnitKind::Sub { sub, asm_name } => {
				sub_residual = sub_residual.max(sub.residual);
				let offset = assembly.instances.len();
				let member_count = sub.assembly.instances.len();
				let member_suppressed_flags: Vec<bool> = (0..member_count).map(|j| sub.assembly.is_instance_suppressed(j)).collect();
				let mut members = Vec::with_capacity(member_count);
				for (j, member) in sub.assembly.instances.into_iter().enumerate() {
					let local = member.pose;
					let leaf = assembly.add(Instance { source: member.source, pose: pose * local });
					let member_suppressed = member_suppressed_flags[j];
					if member_suppressed {
						nested_suppressed.push(leaf);
					}
					assembly.set_instance_suppressed(leaf, unit.suppressed || member_suppressed);
					let member_name = sub.instance_names.get(j).cloned().flatten().unwrap_or_else(|| format!("#{j}"));
					instance_names.push(Some(format!("{display}/{member_name}")));
					members.push((leaf, Some(local)));
				}
				part_names.extend(sub.part_names);
				part_meta.extend(sub.part_meta);
				let mut children = sub.tree;
				rebase_tree_leaves(&mut children, offset);
				tree.push(AsmNode { instance: display, name: asm_name, leaf: None, suppressed: unit.suppressed, children });
				placements.push(UnitPlacement { pose, members });
			}
		}
	}
	let leaf_count = assembly.instances.len();

	// --- bring the named states to life, refusing any that could not be re-applied.
	// File states address the file's instances (the top-level units); they are
	// expanded to leaf level here so `Assembly::apply_state` works unchanged.
	let mut states = BTreeMap::new();
	for (state_name, repr) in file.states {
		if repr.poses.len() != unit_count {
			return Err(FormatError::BadState {
				state: state_name,
				reason: format!("has {} poses for {unit_count} instances", repr.poses.len()),
			});
		}
		if let Some(&bad) = repr.suppressed.iter().find(|&&i| i >= unit_count) {
			return Err(FormatError::BadState {
				state: state_name,
				reason: format!("suppresses instance {bad}, but there are only {unit_count} instances"),
			});
		}
		let mut unit_state_poses = Vec::with_capacity(repr.poses.len());
		for (index, pose) in repr.poses.iter().enumerate() {
			unit_state_poses.push(pose_from_repr(pose, index).map_err(|_| FormatError::BadState {
				state: state_name.clone(),
				reason: format!("pose {index} is not a finite rigid translation+rotation"),
			})?);
		}
		let mut poses = vec![Affine3A::IDENTITY; leaf_count];
		for (placement, &unit_pose) in placements.iter().zip(&unit_state_poses) {
			for &(leaf, local) in &placement.members {
				poses[leaf] = match local {
					None => unit_pose,
					Some(local) => unit_pose * local,
				};
			}
		}
		let mut suppressed: Vec<usize> = repr
			.suppressed
			.iter()
			.flat_map(|&unit| placements[unit].members.iter().map(|&(leaf, _)| leaf))
			.chain(nested_suppressed.iter().copied())
			.collect();
		suppressed.sort_unstable();
		suppressed.dedup();
		states.insert(state_name, AsmState { poses, suppressed });
	}

	let residual = parent_residual.max(sub_residual);
	Ok(LoadedAssembly {
		name: file.name,
		units: file.units,
		instance_names,
		part_names,
		part_meta,
		tree,
		assembly,
		mates: file.mates,
		states,
		residual,
		placements,
		sub_residual,
	})
}
