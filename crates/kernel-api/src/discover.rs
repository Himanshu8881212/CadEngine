//! Self-describing op surface (M3 Discovery). The op catalogue AND the per-op parameter table
//! are derived from the [`OpKind`] enum: `op_tag` through a **compile-forced exhaustive match**
//! (adding a variant without regenerating fails to compile), [`OP_NAMES`]/[`OP_PARAMS`] as
//! generated tables pinned to it by `tests/describe.rs`. Regenerate this WHOLE file with
//! `python3 tools/gen_discover.py` whenever `program.rs`'s `OpKind` changes — never hand-edit.
//!
//! Variants gated `#[cfg(feature = "catalog")]` in `program.rs` carry the same gate here (arm,
//! name, params row), so `--no-default-features` shrinks every table together; [`OP_COUNT`] is
//! emitted per build and [`CATALOG_OP_NAMES`] always lists the gated tags.

use crate::program::OpKind;

/// Canonical wire tag of an op — matches serde's `rename_all = "snake_case"` plus every explicit
/// `#[serde(rename)]`. The match is EXHAUSTIVE (the compiler forces one arm per variant); that is
/// the anti-drift guarantee behind [`OP_NAMES`] and the `describe` op.
pub fn op_tag(op: &OpKind) -> &'static str {
	match op {
		OpKind::Box { .. } => "box",
		OpKind::Cylinder { .. } => "cylinder",
		OpKind::Sphere { .. } => "sphere",
		OpKind::Cone { .. } => "cone",
		OpKind::Torus { .. } => "torus",
		OpKind::Extrude { .. } => "extrude",
		OpKind::ExtrudeWithHoles { .. } => "extrude_with_holes",
		OpKind::ExtrudeTapered { .. } => "extrude_tapered",
		OpKind::Revolve { .. } => "revolve",
		OpKind::Loft { .. } => "loft",
		OpKind::Sweep { .. } => "sweep",
		OpKind::Sketch { .. } => "sketch",
		#[cfg(feature = "catalog")]
		OpKind::SketchExtrude { .. } => "sketch_extrude",
		OpKind::SketchRevolve { .. } => "sketch_revolve",
		OpKind::Union { .. } => "union",
		OpKind::Difference { .. } => "difference",
		OpKind::Intersection { .. } => "intersection",
		OpKind::UnionAll { .. } => "union_all",
		OpKind::FilletEdgeNear { .. } => "fillet_edge_near",
		OpKind::ChamferEdgeNear { .. } => "chamfer_edge_near",
		OpKind::FilletCircularRim { .. } => "fillet_circular_rim",
		OpKind::Translate { .. } => "translate",
		OpKind::RotateZ { .. } => "rotate_z",
		OpKind::RotateX { .. } => "rotate_x",
		OpKind::RotateY { .. } => "rotate_y",
		OpKind::Pose { .. } => "pose",
		OpKind::Mirror { .. } => "mirror",
		OpKind::LinearPattern { .. } => "linear_pattern",
		OpKind::PolarPattern { .. } => "polar_pattern",
		OpKind::Validate { .. } => "validate",
		OpKind::Volume { .. } => "volume",
		OpKind::ExactVolume { .. } => "exact_volume",
		OpKind::MassProperties { .. } => "mass_properties",
		OpKind::BoundingBox { .. } => "bounding_box",
		OpKind::WallThickness { .. } => "wall_thickness",
		OpKind::DraftAnalysis { .. } => "draft_analysis",
		OpKind::MeshComponents { .. } => "mesh_components",
		OpKind::Assert { .. } => "assert",
		OpKind::AssertDisjoint { .. } => "assert_disjoint",
		OpKind::CoincidentFit { .. } => "coincident_fit",
		OpKind::SupportReport { .. } => "support_report",
		OpKind::Clearance { .. } => "clearance",
		OpKind::Describe { .. } => "describe",
		OpKind::ListFaces { .. } => "list_faces",
		OpKind::ListEdges { .. } => "list_edges",
		OpKind::ExportStl { .. } => "export_stl",
		OpKind::ExportStep { .. } => "export_step",
		OpKind::Export3mf { .. } => "export_3mf",
		#[cfg(feature = "catalog")]
		OpKind::GyroidBlock { .. } => "gyroid_block",
		OpKind::MeasureDimension { .. } => "measure_dimension",
		#[cfg(feature = "catalog")]
		OpKind::Tpms { .. } => "tpms",
		OpKind::HybridBoolean { .. } => "hybrid_boolean",
		OpKind::Implicit { .. } => "implicit",
		OpKind::Shell { .. } => "shell",
		OpKind::OffsetSolid { .. } => "offset_solid",
		OpKind::ShellSolid { .. } => "shell_solid",
		OpKind::SolidFromImplicit { .. } => "solid_from_implicit",
		OpKind::ThinWall { .. } => "thin_wall",
		OpKind::MinLigament { .. } => "min_ligament",
		OpKind::SampleDensityGrid { .. } => "sample_density_grid",
		OpKind::MeshDensityGrid { .. } => "mesh_density_grid",
		OpKind::LoadPart { .. } => "load_part",
		OpKind::ImportStep { .. } => "import_step",
		OpKind::ImportMesh { .. } => "import_mesh",
		OpKind::MeshCarve { .. } => "mesh_carve",
		#[cfg(feature = "catalog")]
		OpKind::LibraryAdd { .. } => "library_add",
		#[cfg(feature = "catalog")]
		OpKind::LibrarySearch { .. } => "library_search",
		#[cfg(feature = "catalog")]
		OpKind::LibraryInstantiate { .. } => "library_instantiate",
		#[cfg(feature = "catalog")]
		OpKind::LibraryDeprecate { .. } => "library_deprecate",
		#[cfg(feature = "catalog")]
		OpKind::LibraryRemove { .. } => "library_remove",
		OpKind::SpurGear { .. } => "spur_gear",
		#[cfg(feature = "catalog")]
		OpKind::HexBolt { .. } => "hex_bolt",
		OpKind::HexNut { .. } => "hex_nut",
		OpKind::Washer { .. } => "washer",
		OpKind::SocketHeadCapScrew { .. } => "socket_head_cap_screw",
		#[cfg(feature = "catalog")]
		OpKind::Gt2Pulley { .. } => "gt2_pulley",
		#[cfg(feature = "catalog")]
		OpKind::ChainSprocket { .. } => "chain_sprocket",
		#[cfg(feature = "catalog")]
		OpKind::Shaft { .. } => "shaft",
		#[cfg(feature = "catalog")]
		OpKind::ParallelKey { .. } => "parallel_key",
		OpKind::DowelPin { .. } => "dowel_pin",
		OpKind::CirclipExternal { .. } => "circlip_external",
		#[cfg(feature = "catalog")]
		OpKind::CirclipInternal { .. } => "circlip_internal",
		OpKind::FlatHeadScrew { .. } => "flat_head_screw",
		OpKind::ButtonHeadScrew { .. } => "button_head_screw",
		OpKind::SetScrew { .. } => "set_screw",
		OpKind::LockNut { .. } => "lock_nut",
		#[cfg(feature = "catalog")]
		OpKind::ThreadedRod { .. } => "threaded_rod",
		#[cfg(feature = "catalog")]
		OpKind::Standoff { .. } => "standoff",
		OpKind::CompressionSpring { .. } => "compression_spring",
		#[cfg(feature = "catalog")]
		OpKind::Extrusion2020 { .. } => "extrusion_2020",
		#[cfg(feature = "catalog")]
		OpKind::Extrusion3030 { .. } => "extrusion_3030",
		#[cfg(feature = "catalog")]
		OpKind::Tnut2020 { .. } => "tnut_2020",
		OpKind::ORing { .. } => "o_ring",
		OpKind::ORingCord { .. } => "o_ring_cord",
		#[cfg(feature = "catalog")]
		OpKind::JawCouplingHub { .. } => "jaw_coupling_hub",
		#[cfg(feature = "catalog")]
		OpKind::JawCouplingSpider { .. } => "jaw_coupling_spider",
		#[cfg(feature = "catalog")]
		OpKind::SetScrewCoupling { .. } => "set_screw_coupling",
		#[cfg(feature = "catalog")]
		OpKind::ClampCoupling { .. } => "clamp_coupling",
		#[cfg(feature = "catalog")]
		OpKind::NemaMotor { .. } => "nema_motor",
		#[cfg(feature = "catalog")]
		OpKind::NemaMountPlate { .. } => "nema_mount_plate",
		#[cfg(feature = "catalog")]
		OpKind::LinearBearingLmuu { .. } => "linear_bearing_lmuu",
		#[cfg(feature = "catalog")]
		OpKind::Sc8uuBlock { .. } => "sc8uu_block",
		#[cfg(feature = "catalog")]
		OpKind::ShaftSupportSk8 { .. } => "shaft_support_sk8",
		#[cfg(feature = "catalog")]
		OpKind::ShaftSupportShf8 { .. } => "shaft_support_shf8",
		#[cfg(feature = "catalog")]
		OpKind::Mgn12Rail { .. } => "mgn12_rail",
		#[cfg(feature = "catalog")]
		OpKind::Mgn12Carriage { .. } => "mgn12_carriage",
		OpKind::DeepGrooveBearing { .. } => "deep_groove_bearing",
		OpKind::FlangedBearing { .. } => "flanged_bearing",
		#[cfg(feature = "catalog")]
		OpKind::ThrustBearing { .. } => "thrust_bearing",
		#[cfg(feature = "catalog")]
		OpKind::Kp08PillowBlock { .. } => "kp08_pillow_block",
		#[cfg(feature = "catalog")]
		OpKind::PipeBossG { .. } => "pipe_boss_g",
		#[cfg(feature = "catalog")]
		OpKind::HoseBarb { .. } => "hose_barb",
		#[cfg(feature = "catalog")]
		OpKind::ShoulderBolt { .. } => "shoulder_bolt",
		#[cfg(feature = "catalog")]
		OpKind::SpringWasher { .. } => "spring_washer",
		#[cfg(feature = "catalog")]
		OpKind::LeadScrewTr8 { .. } => "lead_screw_tr8",
		#[cfg(feature = "catalog")]
		OpKind::LeadScrewNutTr8 { .. } => "lead_screw_nut_tr8",
		#[cfg(feature = "catalog")]
		OpKind::GearRack { .. } => "gear_rack",
		#[cfg(feature = "catalog")]
		OpKind::InternalGear { .. } => "internal_gear",
		OpKind::HeatsetInsertBoss { .. } => "heatset_insert_boss",
		#[cfg(feature = "catalog")]
		OpKind::CirclipGrooveExternal { .. } => "circlip_groove_external",
		#[cfg(feature = "catalog")]
		OpKind::CirclipGrooveInternal { .. } => "circlip_groove_internal",
		#[cfg(feature = "catalog")]
		OpKind::ORingGroove { .. } => "o_ring_groove",
		OpKind::ORingFaceGland { .. } => "o_ring_face_gland",
		#[cfg(feature = "catalog")]
		OpKind::ORingFaceGlandRacetrack { .. } => "o_ring_face_gland_racetrack",
		#[cfg(feature = "catalog")]
		OpKind::NemaMountCut { .. } => "nema_mount_cut",
		#[cfg(feature = "catalog")]
		OpKind::ServoPocket { .. } => "servo_pocket",
		#[cfg(feature = "catalog")]
		OpKind::Tr8NutTrap { .. } => "tr8_nut_trap",
		#[cfg(feature = "catalog")]
		OpKind::Pc4Port { .. } => "pc4_port",
		OpKind::TeardropHole { .. } => "teardrop_hole",
		OpKind::BoardMount { .. } => "board_mount",
		OpKind::BridgedCounterbore { .. } => "bridged_counterbore",
		OpKind::AsmInstance { .. } => "asm_instance",
		OpKind::AsmInstanceMesh { .. } => "asm_instance_mesh",
		OpKind::AsmMate { .. } => "asm_mate",
		OpKind::AsmMateAxis { .. } => "asm_mate_axis",
		OpKind::AsmMateFace { .. } => "asm_mate_face",
		OpKind::AsmSolve { .. } => "asm_solve",
		OpKind::AsmContacts { .. } => "asm_contacts",
		OpKind::AsmInterferenceVolume { .. } => "asm_interference_volume",
		OpKind::AsmMassProperties { .. } => "asm_mass_properties",
		OpKind::AsmExport { .. } => "asm_export",
		OpKind::AsmExportStep { .. } => "asm_export_step",
		OpKind::AsmSave { .. } => "asm_save",
		OpKind::GearTrainPoses { .. } => "gear_train_poses",
		#[cfg(feature = "catalog")]
		OpKind::Gt2Belt { .. } => "gt2_belt",
		#[cfg(feature = "catalog")]
		OpKind::Gt2CenterDistance { .. } => "gt2_center_distance",
		OpKind::Iso286Fit { .. } => "iso286_fit",
		OpKind::HeatsetSpec { .. } => "heatset_spec",
		OpKind::MetricCordGland { .. } => "metric_cord_gland",
		OpKind::RacetrackCordLength { .. } => "racetrack_cord_length",
		#[cfg(feature = "catalog")]
		OpKind::PipeThreadG { .. } => "pipe_thread_g",
		OpKind::Drill { .. } => "drill",
		OpKind::ClearanceHole { .. } => "clearance_hole",
		OpKind::CounterboreHole { .. } => "counterbore_hole",
		OpKind::CountersinkHole { .. } => "countersink_hole",
		OpKind::TapDrillHole { .. } => "tap_drill_hole",
		OpKind::BoltCircle { .. } => "bolt_circle",
		OpKind::BearingSeat { .. } => "bearing_seat",
		OpKind::ThreadSpec { .. } => "thread_spec",
		OpKind::ThreadRidge { .. } => "thread_ridge",
		OpKind::ExportThreaded { .. } => "export_threaded",
	}
}

/// The authoritative catalogue every supported op tag, in declaration order returned by the
/// `describe` op and generated from the same source as [`op_tag`]. Length is pinned to the variant
/// count by `tests/describe.rs`; every entry is proven executable (never `unknown_op`) there too.
pub const OP_NAMES: &[&str] = &[
	"box",
	"cylinder",
	"sphere",
	"cone",
	"torus",
	"extrude",
	"extrude_with_holes",
	"extrude_tapered",
	"revolve",
	"loft",
	"sweep",
	"sketch",
	#[cfg(feature = "catalog")]
	"sketch_extrude",
	"sketch_revolve",
	"union",
	"difference",
	"intersection",
	"union_all",
	"fillet_edge_near",
	"chamfer_edge_near",
	"fillet_circular_rim",
	"translate",
	"rotate_z",
	"rotate_x",
	"rotate_y",
	"pose",
	"mirror",
	"linear_pattern",
	"polar_pattern",
	"validate",
	"volume",
	"exact_volume",
	"mass_properties",
	"bounding_box",
	"wall_thickness",
	"draft_analysis",
	"mesh_components",
	"assert",
	"assert_disjoint",
	"coincident_fit",
	"support_report",
	"clearance",
	"describe",
	"list_faces",
	"list_edges",
	"export_stl",
	"export_step",
	"export_3mf",
	#[cfg(feature = "catalog")]
	"gyroid_block",
	"measure_dimension",
	#[cfg(feature = "catalog")]
	"tpms",
	"hybrid_boolean",
	"implicit",
	"shell",
	"offset_solid",
	"shell_solid",
	"solid_from_implicit",
	"thin_wall",
	"min_ligament",
	"sample_density_grid",
	"mesh_density_grid",
	"load_part",
	"import_step",
	"import_mesh",
	"mesh_carve",
	#[cfg(feature = "catalog")]
	"library_add",
	#[cfg(feature = "catalog")]
	"library_search",
	#[cfg(feature = "catalog")]
	"library_instantiate",
	#[cfg(feature = "catalog")]
	"library_deprecate",
	#[cfg(feature = "catalog")]
	"library_remove",
	"spur_gear",
	#[cfg(feature = "catalog")]
	"hex_bolt",
	"hex_nut",
	"washer",
	"socket_head_cap_screw",
	#[cfg(feature = "catalog")]
	"gt2_pulley",
	#[cfg(feature = "catalog")]
	"chain_sprocket",
	#[cfg(feature = "catalog")]
	"shaft",
	#[cfg(feature = "catalog")]
	"parallel_key",
	"dowel_pin",
	"circlip_external",
	#[cfg(feature = "catalog")]
	"circlip_internal",
	"flat_head_screw",
	"button_head_screw",
	"set_screw",
	"lock_nut",
	#[cfg(feature = "catalog")]
	"threaded_rod",
	#[cfg(feature = "catalog")]
	"standoff",
	"compression_spring",
	#[cfg(feature = "catalog")]
	"extrusion_2020",
	#[cfg(feature = "catalog")]
	"extrusion_3030",
	#[cfg(feature = "catalog")]
	"tnut_2020",
	"o_ring",
	"o_ring_cord",
	#[cfg(feature = "catalog")]
	"jaw_coupling_hub",
	#[cfg(feature = "catalog")]
	"jaw_coupling_spider",
	#[cfg(feature = "catalog")]
	"set_screw_coupling",
	#[cfg(feature = "catalog")]
	"clamp_coupling",
	#[cfg(feature = "catalog")]
	"nema_motor",
	#[cfg(feature = "catalog")]
	"nema_mount_plate",
	#[cfg(feature = "catalog")]
	"linear_bearing_lmuu",
	#[cfg(feature = "catalog")]
	"sc8uu_block",
	#[cfg(feature = "catalog")]
	"shaft_support_sk8",
	#[cfg(feature = "catalog")]
	"shaft_support_shf8",
	#[cfg(feature = "catalog")]
	"mgn12_rail",
	#[cfg(feature = "catalog")]
	"mgn12_carriage",
	"deep_groove_bearing",
	"flanged_bearing",
	#[cfg(feature = "catalog")]
	"thrust_bearing",
	#[cfg(feature = "catalog")]
	"kp08_pillow_block",
	#[cfg(feature = "catalog")]
	"pipe_boss_g",
	#[cfg(feature = "catalog")]
	"hose_barb",
	#[cfg(feature = "catalog")]
	"shoulder_bolt",
	#[cfg(feature = "catalog")]
	"spring_washer",
	#[cfg(feature = "catalog")]
	"lead_screw_tr8",
	#[cfg(feature = "catalog")]
	"lead_screw_nut_tr8",
	#[cfg(feature = "catalog")]
	"gear_rack",
	#[cfg(feature = "catalog")]
	"internal_gear",
	"heatset_insert_boss",
	#[cfg(feature = "catalog")]
	"circlip_groove_external",
	#[cfg(feature = "catalog")]
	"circlip_groove_internal",
	#[cfg(feature = "catalog")]
	"o_ring_groove",
	"o_ring_face_gland",
	#[cfg(feature = "catalog")]
	"o_ring_face_gland_racetrack",
	#[cfg(feature = "catalog")]
	"nema_mount_cut",
	#[cfg(feature = "catalog")]
	"servo_pocket",
	#[cfg(feature = "catalog")]
	"tr8_nut_trap",
	#[cfg(feature = "catalog")]
	"pc4_port",
	"teardrop_hole",
	"board_mount",
	"bridged_counterbore",
	"asm_instance",
	"asm_instance_mesh",
	"asm_mate",
	"asm_mate_axis",
	"asm_mate_face",
	"asm_solve",
	"asm_contacts",
	"asm_interference_volume",
	"asm_mass_properties",
	"asm_export",
	"asm_export_step",
	"asm_save",
	"gear_train_poses",
	#[cfg(feature = "catalog")]
	"gt2_belt",
	#[cfg(feature = "catalog")]
	"gt2_center_distance",
	"iso286_fit",
	"heatset_spec",
	"metric_cord_gland",
	"racetrack_cord_length",
	#[cfg(feature = "catalog")]
	"pipe_thread_g",
	"drill",
	"clearance_hole",
	"counterbore_hole",
	"countersink_hole",
	"tap_drill_hole",
	"bolt_circle",
	"bearing_seat",
	"thread_spec",
	"thread_ridge",
	"export_threaded",
];

/// Number of supported ops in a default build (`catalog` feature on). Kept in lockstep with
/// the `OpKind` variant count via [`op_tag`].
#[cfg(feature = "catalog")]
pub const OP_COUNT: usize = 161;

/// Number of supported ops with the `catalog` feature compiled out (`--no-default-features`).
#[cfg(not(feature = "catalog"))]
pub const OP_COUNT: usize = 109;

/// Wire tags of the ops behind the `catalog` cargo feature. Always compiled — even when the
/// feature is off — so the interpreter can name the feature in its `unknown_op` refusal
/// instead of calling the op a typo.
pub const CATALOG_OP_NAMES: &[&str] = &[
	"sketch_extrude",
	"gyroid_block",
	"tpms",
	"library_add",
	"library_search",
	"library_instantiate",
	"library_deprecate",
	"library_remove",
	"hex_bolt",
	"gt2_pulley",
	"chain_sprocket",
	"shaft",
	"parallel_key",
	"circlip_internal",
	"threaded_rod",
	"standoff",
	"extrusion_2020",
	"extrusion_3030",
	"tnut_2020",
	"jaw_coupling_hub",
	"jaw_coupling_spider",
	"set_screw_coupling",
	"clamp_coupling",
	"nema_motor",
	"nema_mount_plate",
	"linear_bearing_lmuu",
	"sc8uu_block",
	"shaft_support_sk8",
	"shaft_support_shf8",
	"mgn12_rail",
	"mgn12_carriage",
	"thrust_bearing",
	"kp08_pillow_block",
	"pipe_boss_g",
	"hose_barb",
	"shoulder_bolt",
	"spring_washer",
	"lead_screw_tr8",
	"lead_screw_nut_tr8",
	"gear_rack",
	"internal_gear",
	"circlip_groove_external",
	"circlip_groove_internal",
	"o_ring_groove",
	"o_ring_face_gland_racetrack",
	"nema_mount_cut",
	"servo_pocket",
	"tr8_nut_trap",
	"pc4_port",
	"gt2_belt",
	"gt2_center_distance",
	"pipe_thread_g",
];

/// One parameter of an op, as served by `describe {name}`: the JSON wire name (post
/// `#[serde(rename)]` — e.g. `in`), a friendly type string (`number` / `int` / `string` /
/// `bool` / `id-ref` / `[x,y,z]` / `[[x,y]...]` / `object` / ...), whether the field is
/// required (no `Option` and no serde default), the first sentence of its doc comment, and
/// every accepted `#[serde(alias)]` wire spelling (the fail-closed unknown-param check and
/// `describe` both honour aliases — an accepted spelling is never refused as unknown).
#[derive(Clone, Copy, Debug)]
pub struct ParamSpec {
	pub name: &'static str,
	pub ty: &'static str,
	pub required: bool,
	pub doc: &'static str,
	pub aliases: &'static [&'static str],
}

/// Per-op parameter specs, parallel to [`OP_NAMES`] (same tags, same declaration order — pinned
/// by `tests/describe.rs`). Generated from the `OpKind` field lists by `tools/gen_discover.py`.
pub static OP_PARAMS: &[(&str, &[ParamSpec])] = &[
	("box", &[
		ParamSpec { name: "min", ty: "[x,y,z]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "max", ty: "[x,y,z]", required: true, doc: "", aliases: &[] },
	]),
	("cylinder", &[
		ParamSpec { name: "base", ty: "[x,y,z]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "axis", ty: "[x,y,z]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "radius", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "height", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "segments", ty: "int", required: false, doc: "", aliases: &[] },
	]),
	("sphere", &[
		ParamSpec { name: "center", ty: "[x,y,z]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "radius", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "u", ty: "int", required: false, doc: "", aliases: &[] },
		ParamSpec { name: "v", ty: "int", required: false, doc: "", aliases: &[] },
	]),
	("cone", &[
		ParamSpec { name: "base", ty: "[x,y,z]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "axis", ty: "[x,y,z]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "radius", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "height", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "segments", ty: "int", required: false, doc: "", aliases: &[] },
		ParamSpec { name: "top_radius", ty: "number", required: false, doc: "Radius of the flat top face (mm).", aliases: &[] },
	]),
	("torus", &[
		ParamSpec { name: "center", ty: "[x,y,z]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "axis", ty: "[x,y,z]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "major", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "minor", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "ring_segments", ty: "int", required: false, doc: "", aliases: &[] },
		ParamSpec { name: "tube_segments", ty: "int", required: false, doc: "", aliases: &[] },
	]),
	("extrude", &[
		ParamSpec { name: "profile", ty: "[[x,y]...]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "height", ty: "number", required: true, doc: "", aliases: &[] },
	]),
	("extrude_with_holes", &[
		ParamSpec { name: "outer", ty: "[[x,y]...]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "holes", ty: "[[[x,y]...]...]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "height", ty: "number", required: true, doc: "", aliases: &[] },
	]),
	("extrude_tapered", &[
		ParamSpec { name: "profile", ty: "[[x,y]...]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "height", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "draft_deg", ty: "number", required: true, doc: "", aliases: &[] },
	]),
	("revolve", &[
		ParamSpec { name: "profile", ty: "[[x,y]...]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "segments", ty: "int", required: false, doc: "", aliases: &[] },
	]),
	("loft", &[
		ParamSpec { name: "sections", ty: "[[[x,y,z]...]...]", required: true, doc: "", aliases: &[] },
	]),
	("sweep", &[
		ParamSpec { name: "profile", ty: "[[x,y,z]...]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "path", ty: "[[x,y,z]...]", required: true, doc: "", aliases: &[] },
	]),
	("sketch", &[
		ParamSpec { name: "points", ty: "[[x,y]...]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "segments", ty: "[[int,int]...]", required: false, doc: "", aliases: &[] },
		ParamSpec { name: "arcs", ty: "[object...]", required: false, doc: "", aliases: &[] },
		ParamSpec { name: "circles", ty: "[object...]", required: false, doc: "", aliases: &[] },
		ParamSpec { name: "constraints", ty: "[object...]", required: false, doc: "", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("sketch_extrude", &[
		ParamSpec { name: "sketch", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "height", ty: "number", required: true, doc: "", aliases: &[] },
	]),
	("sketch_revolve", &[
		ParamSpec { name: "sketch", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "segments", ty: "int", required: false, doc: "", aliases: &[] },
	]),
	("union", &[
		ParamSpec { name: "a", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "b", ty: "id-ref", required: true, doc: "", aliases: &[] },
	]),
	("difference", &[
		ParamSpec { name: "a", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "b", ty: "id-ref", required: true, doc: "", aliases: &[] },
	]),
	("intersection", &[
		ParamSpec { name: "a", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "b", ty: "id-ref", required: true, doc: "", aliases: &[] },
	]),
	("union_all", &[
		ParamSpec { name: "in", ty: "[id-ref...]", required: true, doc: "", aliases: &[] },
	]),
	("fillet_edge_near", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "witness", ty: "[x,y,z]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "radius", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "max_distance", ty: "number", required: false, doc: "Reject a witness farther than this from every edge (default: 10% of the solid's bounding-box diagonal).", aliases: &[] },
	]),
	("chamfer_edge_near", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "witness", ty: "[x,y,z]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "radius", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "max_distance", ty: "number", required: false, doc: "Same witness guard as `fillet_edge_near`'s `max_distance`.", aliases: &[] },
	]),
	("fillet_circular_rim", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "witness", ty: "[x,y,z]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "radius", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "arc_segments", ty: "int", required: false, doc: "", aliases: &[] },
	]),
	("translate", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "offset", ty: "[x,y,z]", required: true, doc: "", aliases: &[] },
	]),
	("rotate_z", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "degrees", ty: "number", required: true, doc: "", aliases: &[] },
	]),
	("rotate_x", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "degrees", ty: "number", required: true, doc: "", aliases: &[] },
	]),
	("rotate_y", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "degrees", ty: "number", required: true, doc: "", aliases: &[] },
	]),
	("pose", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "translate", ty: "[x,y,z]", required: false, doc: "", aliases: &[] },
		ParamSpec { name: "rotate", ty: "object", required: false, doc: "", aliases: &[] },
	]),
	("mirror", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "plane", ty: "object", required: true, doc: "", aliases: &[] },
	]),
	("linear_pattern", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "count", ty: "int", required: true, doc: "Number of instances INCLUDING the original (2..=500).", aliases: &[] },
		ParamSpec { name: "step", ty: "[x,y,z]", required: true, doc: "Per-instance offset vector (mm); must be non-zero.", aliases: &[] },
	]),
	("polar_pattern", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "count", ty: "int", required: true, doc: "Number of instances INCLUDING the original (2..=500).", aliases: &[] },
		ParamSpec { name: "center", ty: "[x,y,z]", required: true, doc: "A point on the rotation axis.", aliases: &[] },
		ParamSpec { name: "axis", ty: "[x,y,z]", required: true, doc: "Rotation axis — any non-zero vector, normalized internally.", aliases: &[] },
		ParamSpec { name: "step_deg", ty: "number", required: false, doc: "Angular pitch between instances in degrees (default `360 / count`, a full evenly-spaced ring).", aliases: &[] },
	]),
	("validate", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
	]),
	("volume", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
	]),
	("exact_volume", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
	]),
	("mass_properties", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
	]),
	("bounding_box", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "envelope", ty: "[x,y,z]", required: false, doc: "", aliases: &[] },
	]),
	("wall_thickness", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "flag_below", ty: "number", required: true, doc: "", aliases: &[] },
	]),
	("draft_analysis", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "pull", ty: "[x,y,z]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "min_deg", ty: "number", required: true, doc: "", aliases: &[] },
	]),
	("mesh_components", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "tol", ty: "number", required: false, doc: "Chord tolerance (mm) of the measurement tessellation (default 0.05).", aliases: &[] },
		ParamSpec { name: "weld_tol", ty: "number", required: false, doc: "Position-weld scale (mm) for vertex identity (default 1e-3, the house weld scale — coincident-but-unshared boolean vertices count as one point).", aliases: &[] },
	]),
	("assert", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "volume_within", ty: "object", required: false, doc: "Faceted (mesh) volume must land in this window.", aliases: &[] },
		ParamSpec { name: "exact_volume_within", ty: "object", required: false, doc: "Analytic `exact_volume` must land in this window.", aliases: &[] },
		ParamSpec { name: "genus", ty: "int", required: false, doc: "Topological genus must equal this.", aliases: &[] },
		ParamSpec { name: "shells", ty: "int", required: false, doc: "Shell count must equal this (e.g. 2 = two disjoint bodies after a union).", aliases: &[] },
		ParamSpec { name: "components", ty: "int", required: false, doc: "Mesh connected-component count must equal this — the single-body gate (`components: 1`).", aliases: &[] },
		ParamSpec { name: "closed", ty: "bool", required: false, doc: "`validate().closed` must equal this.", aliases: &[] },
		ParamSpec { name: "manifold", ty: "bool", required: false, doc: "`validate().manifold` must equal this.", aliases: &[] },
		ParamSpec { name: "valid", ty: "bool", required: false, doc: "`validate().is_valid()` (closed + manifold + sane genus) must equal this.", aliases: &[] },
		ParamSpec { name: "tol", ty: "number", required: false, doc: "Chord tolerance (mm) of the `components` measurement tessellation (default 0.05) — the same knob `mesh_components` exposes.", aliases: &[] },
		ParamSpec { name: "weld_tol", ty: "number", required: false, doc: "Position-weld scale (mm) for `components` vertex identity (default 1e-3).", aliases: &[] },
	]),
	("assert_disjoint", &[
		ParamSpec { name: "a", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "b", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "min_clearance", ty: "number", required: false, doc: "Required clearance (mm); the assertion passes iff distance > this.", aliases: &[] },
		ParamSpec { name: "tol", ty: "number", required: false, doc: "Chord tolerance (mm) of the measurement tessellation — the distance is accurate to about this; for hard proofs keep `min_clearance` ≳ `tol`.", aliases: &[] },
	]),
	("coincident_fit", &[
		ParamSpec { name: "a", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "b", ty: "id-ref", required: true, doc: "", aliases: &[] },
	]),
	("support_report", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "build_dir", ty: "[x,y,z]", required: false, doc: "", aliases: &[] },
		ParamSpec { name: "overhang_deg", ty: "number", required: false, doc: "", aliases: &[] },
	]),
	("clearance", &[
		ParamSpec { name: "a", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "b", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "tol", ty: "number", required: false, doc: "", aliases: &[] },
	]),
	("describe", &[
		ParamSpec { name: "name", ty: "string", required: false, doc: "", aliases: &[] },
	]),
	("list_faces", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
	]),
	("list_edges", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
	]),
	("export_stl", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "file", ty: "string", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "tol", ty: "number", required: false, doc: "", aliases: &[] },
		ParamSpec { name: "voxel", ty: "number", required: false, doc: "Voxel size for the heal fallback (default 0.3 mm).", aliases: &[] },
	]),
	("export_step", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "file", ty: "string", required: true, doc: "", aliases: &[] },
	]),
	("export_3mf", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "file", ty: "string", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "tol", ty: "number", required: false, doc: "", aliases: &[] },
		ParamSpec { name: "voxel", ty: "number", required: false, doc: "Voxel size for the heal fallback (default 0.3 mm).", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("gyroid_block", &[
		ParamSpec { name: "center", ty: "[x,y,z]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "half", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "scale", ty: "number", required: true, doc: "Gyroid frequency scale (cells shrink as `scale` grows; try 0.35).", aliases: &[] },
		ParamSpec { name: "thickness", ty: "number", required: true, doc: "Shell thickness (mm).", aliases: &[] },
		ParamSpec { name: "voxel", ty: "number", required: true, doc: "Voxel size (mm) for the dual-contour grid.", aliases: &[] },
		ParamSpec { name: "file", ty: "string", required: true, doc: "", aliases: &[] },
	]),
	("measure_dimension", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "kind", ty: "string", required: true, doc: "`\"point_point\"` / `\"face_face\"` / `\"diameter\"`.", aliases: &[] },
		ParamSpec { name: "a", ty: "[x,y,z]", required: false, doc: "`point_point`: first point.", aliases: &[] },
		ParamSpec { name: "b", ty: "[x,y,z]", required: false, doc: "`point_point`: second point.", aliases: &[] },
		ParamSpec { name: "near", ty: "[x,y,z]", required: false, doc: "`diameter`: witness selecting the measured cylinder/sphere face.", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("tpms", &[
		ParamSpec { name: "kind", ty: "string", required: true, doc: "Family: `gyroid` / `schwarz_p` / `diamond` / `neovius` / `schoen_iwp` / `fischer_koch_s`.", aliases: &[] },
		ParamSpec { name: "min", ty: "[x,y,z]", required: true, doc: "Lattice block corner (mm).", aliases: &[] },
		ParamSpec { name: "max", ty: "[x,y,z]", required: true, doc: "Opposite corner (mm).", aliases: &[] },
		ParamSpec { name: "cell", ty: "number", required: true, doc: "Unit-cell edge length (mm).", aliases: &[] },
		ParamSpec { name: "mode", ty: "string", required: false, doc: "`\"network\"` (default) or `\"sheet\"`.", aliases: &[] },
		ParamSpec { name: "level", ty: "number", required: false, doc: "network: iso-level (default 0 ≈ 50% solid; negative thins).", aliases: &[] },
		ParamSpec { name: "voxel", ty: "number", required: false, doc: "Voxel size (mm) for the extraction grid (default 0.3).", aliases: &[] },
		ParamSpec { name: "file", ty: "string", required: true, doc: "Output mesh path — the extension picks the format (`.stl` / `.3mf`).", aliases: &[] },
	]),
	("hybrid_boolean", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "The exact B-rep operand (a bound solid).", aliases: &[] },
		ParamSpec { name: "field", ty: "object", required: false, doc: "Implicit operand: a nestable CSG tree with finite bounds (exclusive with `file`; clamp an unbounded field by intersecting with a box).", aliases: &[] },
		ParamSpec { name: "file", ty: "string", required: false, doc: "Mesh-file operand (`.stl`/`.obj`/`.3mf`/`.ply`; exclusive with `field`).", aliases: &[] },
		ParamSpec { name: "bool", ty: "string", required: true, doc: "Which boolean: `\"union\"` / `\"difference\"` / `\"intersection\"`.", aliases: &[] },
		ParamSpec { name: "voxel", ty: "number", required: false, doc: "Voxel size (mm): the field operand's meshing lattice, and the healed fallback's resampling lattice (default 0.3).", aliases: &[] },
		ParamSpec { name: "out", ty: "string", required: true, doc: "Output mesh path — the extension picks the format (`.stl` / `.3mf`).", aliases: &[] },
	]),
	("implicit", &[
		ParamSpec { name: "expr", ty: "object", required: true, doc: "The recursive expression tree (parsed with JSON-path errors).", aliases: &[] },
		ParamSpec { name: "voxel", ty: "number", required: true, doc: "Voxel size (mm) for the extraction grid.", aliases: &[] },
		ParamSpec { name: "mesher", ty: "string", required: false, doc: "", aliases: &[] },
		ParamSpec { name: "domain", ty: "object", required: false, doc: "Explicit meshing box; default: the tree's bounds padded by 3·voxel.", aliases: &[] },
		ParamSpec { name: "file", ty: "string", required: false, doc: "Optional output file — the extension picks the format (`.stl`/`.3mf`).", aliases: &[] },
	]),
	("shell", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "wall", ty: "number", required: true, doc: "Wall thickness (mm), > 0 and at least 2·`voxel` so the grid resolves it.", aliases: &[] },
		ParamSpec { name: "voxel", ty: "number", required: false, doc: "Voxel size (mm) for the SDF re-mesh (default 0.3).", aliases: &[] },
		ParamSpec { name: "file", ty: "string", required: false, doc: "Optional output file — the extension picks the format (`.stl`/`.3mf`).", aliases: &[] },
	]),
	("offset_solid", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "delta", ty: "number", required: true, doc: "Signed offset (mm): positive grows, negative shrinks.", aliases: &[] },
		ParamSpec { name: "voxel", ty: "number", required: false, doc: "Voxel size (mm) of the re-extraction lattice (default 0.3).", aliases: &[] },
	]),
	("shell_solid", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "thickness", ty: "number", required: true, doc: "Wall thickness (mm), > 0 and at least 2·`voxel` so the grid resolves it.", aliases: &[] },
		ParamSpec { name: "voxel", ty: "number", required: false, doc: "Voxel size (mm) of the re-extraction lattice (default 0.3).", aliases: &[] },
	]),
	("solid_from_implicit", &[
		ParamSpec { name: "expr", ty: "object", required: true, doc: "The implicit expression tree (same grammar as the `implicit` op).", aliases: &[] },
		ParamSpec { name: "voxel", ty: "number", required: true, doc: "Extraction voxel size (mm) — also the chord fidelity of every face.", aliases: &[] },
		ParamSpec { name: "domain", ty: "object", required: false, doc: "Explicit meshing box; default: the tree's own (finite) bounds.", aliases: &[] },
	]),
	("thin_wall", &[
		ParamSpec { name: "in", ty: "id-ref", required: false, doc: "A bound solid id (exclusive with `expr`).", aliases: &[] },
		ParamSpec { name: "expr", ty: "object", required: false, doc: "An implicit expression tree (exclusive with `in`).", aliases: &[] },
		ParamSpec { name: "t_min", ty: "number", required: true, doc: "Walls thinner than this (mm) are counted in `below_count`.", aliases: &[] },
		ParamSpec { name: "samples", ty: "int", required: false, doc: "Sampling lattice points per axis, 8..=256 (default 64; cost ~samples³).", aliases: &[] },
		ParamSpec { name: "domain", ty: "object", required: false, doc: "Explicit census box; default: the solid's bounding box / the tree's bounds.", aliases: &[] },
	]),
	("min_ligament", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "at", ty: "[x,y,z]", required: true, doc: "Planned hole center on the entry face.", aliases: &[] },
		ParamSpec { name: "axis", ty: "[x,y,z]", required: true, doc: "Drilling direction, INTO the material (hole-wizard convention).", aliases: &[] },
		ParamSpec { name: "d", ty: "number", required: true, doc: "Planned bore **diameter** (mm).", aliases: &[] },
	]),
	("sample_density_grid", &[
		ParamSpec { name: "in", ty: "id-ref", required: false, doc: "A bound solid id (exclusive with `expr`).", aliases: &[] },
		ParamSpec { name: "expr", ty: "object", required: false, doc: "An implicit expression tree (exclusive with `in`).", aliases: &[] },
		ParamSpec { name: "origin", ty: "[x,y,z]", required: true, doc: "Grid origin (mm) — the LOW corner of voxel (0,0,0), not its center.", aliases: &[] },
		ParamSpec { name: "voxel", ty: "number", required: true, doc: "Isotropic voxel size h (mm).", aliases: &[] },
		ParamSpec { name: "shape", ty: "[int,int,int]", required: true, doc: "Grid shape (nx, ny, nz).", aliases: &[] },
		ParamSpec { name: "supersample", ty: "int", required: false, doc: "Sub-points per axis per voxel for fractional densities (default 2).", aliases: &[] },
		ParamSpec { name: "file", ty: "string", required: true, doc: "Output `.npy` path (relative joins `--out-dir`).", aliases: &[] },
	]),
	("mesh_density_grid", &[
		ParamSpec { name: "npy", ty: "string", required: true, doc: "Input `.npy` (float32/float64, C-order, shape `(nx,ny,nz)`).", aliases: &[] },
		ParamSpec { name: "origin", ty: "[x,y,z]", required: true, doc: "Grid origin (mm) — low corner of voxel (0,0,0).", aliases: &[] },
		ParamSpec { name: "voxel", ty: "number", required: true, doc: "Isotropic voxel size h (mm).", aliases: &[] },
		ParamSpec { name: "iso", ty: "number", required: false, doc: "Density threshold: material where `rho >= iso` (default 0.5).", aliases: &[] },
		ParamSpec { name: "file", ty: "string", required: true, doc: "Output mesh path (`.stl`/`.3mf`).", aliases: &[] },
	]),
	("load_part", &[
		ParamSpec { name: "file", ty: "string", required: true, doc: "", aliases: &[] },
	]),
	("import_step", &[
		ParamSpec { name: "file", ty: "string", required: true, doc: "", aliases: &[] },
	]),
	("import_mesh", &[
		ParamSpec { name: "file", ty: "string", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "heal", ty: "bool", required: false, doc: "Repair before the receipt: cap boundary loops (`fill_holes`) and split non-manifold junctions (`make_manifold`).", aliases: &[] },
		ParamSpec { name: "out", ty: "string", required: false, doc: "Optional re-write of the welded/healed mesh — the extension picks the format (`.stl` / `.3mf`).", aliases: &[] },
	]),
	("mesh_carve", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "file", ty: "string", required: true, doc: "The mesh operand file (`.stl` / `.obj` / `.3mf` / `.ply`).", aliases: &[] },
		ParamSpec { name: "bool", ty: "string", required: true, doc: "Which boolean: `\"union\"` / `\"difference\"` / `\"intersection\"`.", aliases: &[] },
		ParamSpec { name: "voxel", ty: "number", required: false, doc: "Voxel size (mm) of the resampling lattice (default 0.3).", aliases: &[] },
		ParamSpec { name: "out", ty: "string", required: true, doc: "Output mesh path — the extension picks the format (`.stl` / `.3mf`).", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("library_add", &[
		ParamSpec { name: "dir", ty: "string", required: true, doc: "Library directory (relative joins `--out-dir`; created on demand).", aliases: &[] },
		ParamSpec { name: "part", ty: "object", required: false, doc: "The candidate part envelope INLINE (exclusive with `part_file`).", aliases: &[] },
		ParamSpec { name: "part_file", ty: "string", required: false, doc: "Path to the candidate `.lmcpart` (exclusive with `part`).", aliases: &[] },
		ParamSpec { name: "meta", ty: "object", required: true, doc: "Identity, provenance (caller-supplied date) and parameter interface.", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("library_search", &[
		ParamSpec { name: "dir", ty: "string", required: true, doc: "Library directory.", aliases: &[] },
		ParamSpec { name: "text", ty: "string", required: false, doc: "Case-insensitive substring over name/category/description/tags (empty matches all).", aliases: &[] },
		ParamSpec { name: "tags", ty: "[string...]", required: false, doc: "Tags the entry must all carry (case-insensitive).", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("library_instantiate", &[
		ParamSpec { name: "dir", ty: "string", required: true, doc: "Library directory.", aliases: &[] },
		ParamSpec { name: "name", ty: "string", required: true, doc: "Entry name.", aliases: &[] },
		ParamSpec { name: "version", ty: "int", required: false, doc: "Entry version (default: the highest admitted version).", aliases: &[] },
		ParamSpec { name: "params", ty: "object", required: false, doc: "Parameter values; unset parameters take their declared defaults.", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("library_deprecate", &[
		ParamSpec { name: "dir", ty: "string", required: true, doc: "Library directory.", aliases: &[] },
		ParamSpec { name: "name", ty: "string", required: true, doc: "Entry name.", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("library_remove", &[
		ParamSpec { name: "dir", ty: "string", required: true, doc: "Library directory.", aliases: &[] },
		ParamSpec { name: "name", ty: "string", required: true, doc: "Entry name.", aliases: &[] },
		ParamSpec { name: "force", ty: "bool", required: false, doc: "Skip the dependents refusal (default false).", aliases: &[] },
	]),
	("spur_gear", &[
		ParamSpec { name: "module", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "teeth", ty: "int", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "face_width", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "bore", ty: "number", required: true, doc: "Bore **diameter** (mm).", aliases: &["bore_d"] },
		ParamSpec { name: "pressure_angle_deg", ty: "number", required: false, doc: "", aliases: &[] },
		ParamSpec { name: "keyway", ty: "bool", required: false, doc: "Cut the DIN 6885-1 hub keyway sized for `bore` (default false).", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("hex_bolt", &[
		ParamSpec { name: "m", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "length", ty: "number", required: true, doc: "", aliases: &[] },
	]),
	("hex_nut", &[
		ParamSpec { name: "m", ty: "number", required: true, doc: "", aliases: &[] },
	]),
	("washer", &[
		ParamSpec { name: "m", ty: "number", required: true, doc: "", aliases: &[] },
	]),
	("socket_head_cap_screw", &[
		ParamSpec { name: "m", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "length", ty: "number", required: true, doc: "", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("gt2_pulley", &[
		ParamSpec { name: "teeth", ty: "int", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "belt_width", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "bore", ty: "number", required: true, doc: "Bore **diameter** (mm); `bore_d` (the Document field name) is an alias.", aliases: &["bore_d"] },
		ParamSpec { name: "flanged", ty: "bool", required: false, doc: "Add a retaining flange on each end (default false).", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("chain_sprocket", &[
		ParamSpec { name: "pitch", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "roller_d", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "teeth", ty: "int", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "bore", ty: "number", required: true, doc: "Bore **diameter** (mm); `bore_d` (the Document field name) is an alias.", aliases: &["bore_d"] },
	]),
	#[cfg(feature = "catalog")]
	("shaft", &[
		ParamSpec { name: "d", ty: "number", required: true, doc: "Shaft **diameter** (mm).", aliases: &[] },
		ParamSpec { name: "length", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "keyway", ty: "object", required: false, doc: "Optional keyway slot; its width/depth auto-size from the DIN 6885-1 table for `d`.", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("parallel_key", &[
		ParamSpec { name: "b", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "h", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "l", ty: "number", required: true, doc: "", aliases: &[] },
	]),
	("dowel_pin", &[
		ParamSpec { name: "d", ty: "number", required: true, doc: "Pin **diameter** (mm), an ISO 2338 table size.", aliases: &[] },
		ParamSpec { name: "length", ty: "number", required: true, doc: "", aliases: &[] },
	]),
	("circlip_external", &[
		ParamSpec { name: "shaft_d", ty: "number", required: true, doc: "", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("circlip_internal", &[
		ParamSpec { name: "bore_d", ty: "number", required: true, doc: "", aliases: &[] },
	]),
	("flat_head_screw", &[
		ParamSpec { name: "m", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "length", ty: "number", required: true, doc: "", aliases: &[] },
	]),
	("button_head_screw", &[
		ParamSpec { name: "m", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "length", ty: "number", required: true, doc: "", aliases: &[] },
	]),
	("set_screw", &[
		ParamSpec { name: "m", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "length", ty: "number", required: true, doc: "", aliases: &[] },
	]),
	("lock_nut", &[
		ParamSpec { name: "m", ty: "number", required: true, doc: "", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("threaded_rod", &[
		ParamSpec { name: "m", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "length", ty: "number", required: true, doc: "", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("standoff", &[
		ParamSpec { name: "m", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "length", ty: "number", required: true, doc: "", aliases: &[] },
	]),
	("compression_spring", &[
		ParamSpec { name: "wire_d", ty: "number", required: true, doc: "Wire **diameter** (mm).", aliases: &[] },
		ParamSpec { name: "outer_d", ty: "number", required: true, doc: "Coil outside **diameter** (mm).", aliases: &[] },
		ParamSpec { name: "pitch", ty: "number", required: true, doc: "Axial advance per turn (mm), must exceed `wire_d`.", aliases: &[] },
		ParamSpec { name: "turns", ty: "number", required: true, doc: "Active turns (may be fractional).", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("extrusion_2020", &[
		ParamSpec { name: "length", ty: "number", required: true, doc: "", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("extrusion_3030", &[
		ParamSpec { name: "length", ty: "number", required: true, doc: "", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("tnut_2020", &[]),
	("o_ring", &[
		ParamSpec { name: "dash", ty: "int", required: true, doc: "AS568 dash number (e.g. `214`).", aliases: &[] },
	]),
	("o_ring_cord", &[
		ParamSpec { name: "ring_id", ty: "number", required: true, doc: "Ring inside **diameter** (mm) — free, unlike the AS568 dash table.", aliases: &[] },
		ParamSpec { name: "cord_d", ty: "number", required: true, doc: "Cord cross-section **diameter** (mm), a stocked metric size.", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("jaw_coupling_hub", &[
		ParamSpec { name: "od", ty: "number", required: true, doc: "Body outer **diameter** (a table size: 20, 25, 30, 40).", aliases: &[] },
		ParamSpec { name: "bore", ty: "number", required: true, doc: "Bore **diameter** (mm), within the size row's range.", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("jaw_coupling_spider", &[
		ParamSpec { name: "od", ty: "number", required: true, doc: "Body outer **diameter** (a table size: 20, 25, 30, 40).", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("set_screw_coupling", &[
		ParamSpec { name: "bore1", ty: "number", required: true, doc: "Bore at z = 0 (a stocked size).", aliases: &[] },
		ParamSpec { name: "bore2", ty: "number", required: true, doc: "Bore at z = L (a stocked size).", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("clamp_coupling", &[
		ParamSpec { name: "bore1", ty: "number", required: true, doc: "Bore at z = 0 (a stocked size).", aliases: &[] },
		ParamSpec { name: "bore2", ty: "number", required: true, doc: "Bore at z = L (a stocked size).", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("nema_motor", &[
		ParamSpec { name: "frame", ty: "int", required: true, doc: "NEMA frame number (17 or 23).", aliases: &[] },
		ParamSpec { name: "body_len", ty: "number", required: true, doc: "Body length below the faceplate, mm.", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("nema_mount_plate", &[
		ParamSpec { name: "frame", ty: "int", required: true, doc: "NEMA frame number (17 or 23).", aliases: &[] },
		ParamSpec { name: "thickness", ty: "number", required: true, doc: "Plate thickness, mm.", aliases: &[] },
		ParamSpec { name: "margin", ty: "number", required: true, doc: "Extra plate width beyond the motor face, per side (≥ 0).", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("linear_bearing_lmuu", &[
		ParamSpec { name: "bore", ty: "number", required: true, doc: "Shaft bore **diameter**: 8 (LM8UU) or 12 (LM12UU).", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("sc8uu_block", &[]),
	#[cfg(feature = "catalog")]
	("shaft_support_sk8", &[]),
	#[cfg(feature = "catalog")]
	("shaft_support_shf8", &[]),
	#[cfg(feature = "catalog")]
	("mgn12_rail", &[
		ParamSpec { name: "length", ty: "number", required: true, doc: "Rail length, mm (≥ one 25 mm pitch).", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("mgn12_carriage", &[]),
	("deep_groove_bearing", &[
		ParamSpec { name: "designation", ty: "string", required: true, doc: "Seat-table designation: \"603\", \"608\", \"625\", \"688\", \"6000\", \"6001\", \"6804\".", aliases: &[] },
	]),
	("flanged_bearing", &[
		ParamSpec { name: "designation", ty: "string", required: true, doc: "\"F608\" (8 × 22 × 7, flange Ø25 × 1.5) or \"F623\" (3 × 10 × 4, Ø11.5 × 0.6).", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("thrust_bearing", &[
		ParamSpec { name: "designation", ty: "string", required: true, doc: "\"51100\" (10 × 24 × 9) or \"51101\" (12 × 26 × 9).", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("kp08_pillow_block", &[]),
	#[cfg(feature = "catalog")]
	("pipe_boss_g", &[
		ParamSpec { name: "designation", ty: "string", required: true, doc: "\"G1/8\", \"G1/4\", \"G3/8\" or \"G1/2\".", aliases: &[] },
		ParamSpec { name: "wall", ty: "number", required: true, doc: "Radial wall beyond the thread major Ø, mm (≥ 1).", aliases: &[] },
		ParamSpec { name: "length", ty: "number", required: true, doc: "Boss length along +Z, mm (must contain chamfer + one pitch).", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("hose_barb", &[
		ParamSpec { name: "hose_id", ty: "number", required: true, doc: "Hose inner **diameter**, mm.", aliases: &[] },
		ParamSpec { name: "barbs", ty: "int", required: true, doc: "Number of sawtooth teeth (≥ 1).", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("shoulder_bolt", &[
		ParamSpec { name: "shoulder_d", ty: "number", required: true, doc: "Shoulder **diameter**: 6.5, 8, 10, 13 or 16 (the ISO 7379 sizes).", aliases: &[] },
		ParamSpec { name: "shoulder_len", ty: "number", required: true, doc: "Ground-shoulder length, mm (the ordering length).", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("spring_washer", &[
		ParamSpec { name: "m", ty: "number", required: true, doc: "Nominal thread size: 3, 4, 5, 6, 8, 10 or 12.", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("lead_screw_tr8", &[
		ParamSpec { name: "length", ty: "number", required: true, doc: "Screw length, mm.", aliases: &[] },
		ParamSpec { name: "lead", ty: "number", required: true, doc: "Lead: 2 (1-start), 4 (2-start) or 8 (4-start), all pitch 2.", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("lead_screw_nut_tr8", &[]),
	#[cfg(feature = "catalog")]
	("gear_rack", &[
		ParamSpec { name: "module", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "length", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "width", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "pressure_angle_deg", ty: "number", required: false, doc: "", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("internal_gear", &[
		ParamSpec { name: "module", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "teeth", ty: "int", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "face_width", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "rim_od", ty: "number", required: true, doc: "Rim outer **diameter** (mm), must exceed the root circle `m(z + 2.5)`.", aliases: &[] },
		ParamSpec { name: "pressure_angle_deg", ty: "number", required: false, doc: "", aliases: &[] },
	]),
	("heatset_insert_boss", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "at", ty: "[x,y,z]", required: true, doc: "Boss centre on the host face.", aliases: &[] },
		ParamSpec { name: "axis", ty: "[x,y,z]", required: true, doc: "Outward face normal.", aliases: &[] },
		ParamSpec { name: "m", ty: "number", required: true, doc: "", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("circlip_groove_external", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "at", ty: "[x,y,z]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "axis", ty: "[x,y,z]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "shaft_d", ty: "number", required: true, doc: "", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("circlip_groove_internal", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "at", ty: "[x,y,z]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "axis", ty: "[x,y,z]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "bore_d", ty: "number", required: true, doc: "", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("o_ring_groove", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "at", ty: "[x,y,z]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "axis", ty: "[x,y,z]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "dash", ty: "int", required: true, doc: "", aliases: &[] },
	]),
	("o_ring_face_gland", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "at", ty: "[x,y,z]", required: true, doc: "Gland centre **on the face**.", aliases: &[] },
		ParamSpec { name: "axis", ty: "[x,y,z]", required: true, doc: "Outward face normal; the groove sinks along `-axis`.", aliases: &[] },
		ParamSpec { name: "gland_center_d", ty: "number", required: true, doc: "Channel centreline **diameter** (mm).", aliases: &[] },
		ParamSpec { name: "cord_d", ty: "number", required: true, doc: "Cord cross-section **diameter** (mm), a stocked metric size.", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("o_ring_face_gland_racetrack", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "at", ty: "[x,y,z]", required: true, doc: "Racetrack centre **on the face**.", aliases: &[] },
		ParamSpec { name: "axis", ty: "[x,y,z]", required: true, doc: "Outward face normal; the groove sinks along `-axis`.", aliases: &[] },
		ParamSpec { name: "x_len", ty: "number", required: true, doc: "Centreline rectangle overall length along the face-frame x axis (mm).", aliases: &[] },
		ParamSpec { name: "y_len", ty: "number", required: true, doc: "Centreline rectangle overall length along the face-frame y axis (mm).", aliases: &[] },
		ParamSpec { name: "corner_r", ty: "number", required: true, doc: "Centreline corner radius (mm), at least half the groove width.", aliases: &[] },
		ParamSpec { name: "cord_d", ty: "number", required: true, doc: "Cord cross-section **diameter** (mm), a stocked metric size.", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("nema_mount_cut", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "at", ty: "[x,y,z]", required: true, doc: "Motor axis position on the face.", aliases: &[] },
		ParamSpec { name: "axis", ty: "[x,y,z]", required: true, doc: "Outward face normal.", aliases: &[] },
		ParamSpec { name: "frame", ty: "int", required: true, doc: "NEMA frame number (17 or 23).", aliases: &[] },
		ParamSpec { name: "through", ty: "number", required: true, doc: "Material span the holes cut through, mm.", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("servo_pocket", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "at", ty: "[x,y,z]", required: true, doc: "Pocket centre on the face.", aliases: &[] },
		ParamSpec { name: "axis", ty: "[x,y,z]", required: true, doc: "Outward face normal.", aliases: &[] },
		ParamSpec { name: "model", ty: "string", required: true, doc: "Servo model name (\"sg90\" or \"mg996r\").", aliases: &[] },
		ParamSpec { name: "through", ty: "number", required: true, doc: "Material span the pocket cuts through, mm.", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("tr8_nut_trap", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "at", ty: "[x,y,z]", required: true, doc: "Screw axis position on the face.", aliases: &[] },
		ParamSpec { name: "axis", ty: "[x,y,z]", required: true, doc: "Outward face normal.", aliases: &[] },
		ParamSpec { name: "through", ty: "number", required: true, doc: "Material span the bore/holes cut through, mm (> 3.7 recess depth).", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("pc4_port", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "at", ty: "[x,y,z]", required: true, doc: "Port centre on the face.", aliases: &[] },
		ParamSpec { name: "axis", ty: "[x,y,z]", required: true, doc: "Outward face normal.", aliases: &[] },
		ParamSpec { name: "m", ty: "number", required: true, doc: "Fitting thread: 6 (PC4-M6, Ø5 × 6 pocket) or 10 (PC4-M10, Ø9 × 7).", aliases: &[] },
		ParamSpec { name: "through", ty: "number", required: true, doc: "Total material depth, mm (> pocket depth).", aliases: &[] },
	]),
	("teardrop_hole", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "at", ty: "[x,y,z]", required: true, doc: "Hole centre on the entry face.", aliases: &[] },
		ParamSpec { name: "axis", ty: "[x,y,z]", required: true, doc: "Drilling direction, INTO the material (hole-wizard convention).", aliases: &[] },
		ParamSpec { name: "up", ty: "[x,y,z]", required: true, doc: "Build (print-bed +Z) direction; must not be parallel to `axis`.", aliases: &[] },
		ParamSpec { name: "d", ty: "number", required: true, doc: "Bore **diameter**, mm.", aliases: &[] },
		ParamSpec { name: "through", ty: "number", required: true, doc: "Material span, mm.", aliases: &[] },
	]),
	("board_mount", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "at", ty: "[x,y,z]", required: true, doc: "Pattern datum on the face: board bottom-left corner (rpi/arduino_uno) or pattern centre (vesa75/vesa100).", aliases: &[] },
		ParamSpec { name: "axis", ty: "[x,y,z]", required: true, doc: "Drilling direction, INTO the material.", aliases: &[] },
		ParamSpec { name: "board", ty: "string", required: true, doc: "\"rpi\", \"arduino_uno\", \"vesa75\" or \"vesa100\".", aliases: &[] },
	]),
	("bridged_counterbore", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "at", ty: "[x,y,z]", required: true, doc: "Hole centre on the entry face.", aliases: &[] },
		ParamSpec { name: "axis", ty: "[x,y,z]", required: true, doc: "Drilling direction, INTO the material (hole-wizard convention).", aliases: &[] },
		ParamSpec { name: "m", ty: "number", required: true, doc: "Nominal screw size M2–M12 (DIN 974 pocket, ISO 273 medium bore).", aliases: &[] },
		ParamSpec { name: "through", ty: "number", required: true, doc: "Total material depth, mm (> pocket + bridge).", aliases: &[] },
		ParamSpec { name: "bridge", ty: "number", required: true, doc: "Sacrificial membrane thickness, mm (one layer height, e.g. 0.2–0.3).", aliases: &[] },
	]),
	("asm_instance", &[
		ParamSpec { name: "solid", ty: "string", required: true, doc: "Id of the bound solid to place (built by any earlier solid op).", aliases: &[] },
		ParamSpec { name: "name", ty: "string", required: false, doc: "Display name in receipts/BOM (default: this op's id).", aliases: &[] },
		ParamSpec { name: "translate", ty: "[x,y,z]", required: false, doc: "Seed translation (mm).", aliases: &[] },
		ParamSpec { name: "rotate", ty: "object", required: false, doc: "Seed rotation (applied before the translation).", aliases: &[] },
		ParamSpec { name: "material", ty: "object", required: false, doc: "Material for mass/BOM receipts: `{name, density_g_cm3}`.", aliases: &[] },
	]),
	("asm_instance_mesh", &[
		ParamSpec { name: "file", ty: "string", required: true, doc: "Mesh file path (relative paths resolve against the input base).", aliases: &[] },
		ParamSpec { name: "name", ty: "string", required: false, doc: "Display name in receipts/BOM (default: this op's id).", aliases: &[] },
		ParamSpec { name: "translate", ty: "[x,y,z]", required: false, doc: "Seed translation (mm).", aliases: &[] },
		ParamSpec { name: "rotate", ty: "object", required: false, doc: "Seed rotation (applied before the translation).", aliases: &[] },
		ParamSpec { name: "material", ty: "object", required: false, doc: "Material for mass/BOM receipts: `{name, density_g_cm3}`.", aliases: &[] },
	]),
	("asm_mate", &[
		ParamSpec { name: "kind", ty: "string", required: true, doc: "Mate kind (see list above).", aliases: &[] },
		ParamSpec { name: "a", ty: "id-ref", required: true, doc: "First instance (an `asm_instance`/`asm_instance_mesh` op id).", aliases: &[] },
		ParamSpec { name: "b", ty: "id-ref", required: false, doc: "Second instance (required for every kind except `fixed`).", aliases: &[] },
		ParamSpec { name: "a_point", ty: "[x,y,z]", required: false, doc: "Point on `a` (local mm) — coincident/distance.", aliases: &[] },
		ParamSpec { name: "b_point", ty: "[x,y,z]", required: false, doc: "Point on `b` (local mm) — coincident/distance.", aliases: &[] },
		ParamSpec { name: "a_dir", ty: "[x,y,z]", required: false, doc: "Direction on `a` (local) — parallel/angle.", aliases: &[] },
		ParamSpec { name: "b_dir", ty: "[x,y,z]", required: false, doc: "Direction on `b` (local) — parallel/angle.", aliases: &[] },
		ParamSpec { name: "a_axis_point", ty: "[x,y,z]", required: false, doc: "Axis point on `a` (local mm) — concentric/axis_distance.", aliases: &[] },
		ParamSpec { name: "a_axis_dir", ty: "[x,y,z]", required: false, doc: "Axis direction on `a` (local) — concentric/axis_distance.", aliases: &[] },
		ParamSpec { name: "b_axis_point", ty: "[x,y,z]", required: false, doc: "Axis point on `b` (local mm) — concentric/axis_distance.", aliases: &[] },
		ParamSpec { name: "b_axis_dir", ty: "[x,y,z]", required: false, doc: "Axis direction on `b` (local) — concentric/axis_distance.", aliases: &[] },
		ParamSpec { name: "distance", ty: "number", required: false, doc: "Target separation (mm) — distance/axis_distance.", aliases: &[] },
		ParamSpec { name: "degrees", ty: "number", required: false, doc: "Target angle in degrees (0–180) — angle.", aliases: &[] },
	]),
	("asm_mate_axis", &[
		ParamSpec { name: "a", ty: "id-ref", required: true, doc: "First instance (op id).", aliases: &[] },
		ParamSpec { name: "a_witness", ty: "[x,y,z]", required: true, doc: "Point near the axis-carrying face on `a`'s solid (LOCAL mm).", aliases: &[] },
		ParamSpec { name: "b", ty: "id-ref", required: true, doc: "Second instance (op id).", aliases: &[] },
		ParamSpec { name: "b_witness", ty: "[x,y,z]", required: true, doc: "Point near the axis-carrying face on `b`'s solid (LOCAL mm).", aliases: &[] },
		ParamSpec { name: "distance", ty: "number", required: false, doc: "Optional center distance (mm): axis_distance instead of concentric.", aliases: &[] },
	]),
	("asm_mate_face", &[
		ParamSpec { name: "a", ty: "id-ref", required: true, doc: "First instance (op id).", aliases: &[] },
		ParamSpec { name: "a_witness", ty: "[x,y,z]", required: true, doc: "Point near the mating face on `a`'s solid (LOCAL mm).", aliases: &[] },
		ParamSpec { name: "b", ty: "id-ref", required: true, doc: "Second instance (op id).", aliases: &[] },
		ParamSpec { name: "b_witness", ty: "[x,y,z]", required: true, doc: "Point near the mating face on `b`'s solid (LOCAL mm).", aliases: &[] },
		ParamSpec { name: "offset", ty: "number", required: false, doc: "Face separation along the normal (mm, default 0 = flush).", aliases: &[] },
	]),
	("asm_solve", &[
		ParamSpec { name: "iterations", ty: "int", required: false, doc: "Relaxation sweep budget (default 256).", aliases: &[] },
		ParamSpec { name: "max_residual", ty: "number", required: false, doc: "Residual gate (default 1e-6).", aliases: &[] },
		ParamSpec { name: "allow_unconverged", ty: "bool", required: false, doc: "Report an unconverged solve as ok:true with `converged:false` instead of failing (default false — loud).", aliases: &[] },
	]),
	("asm_contacts", &[
		ParamSpec { name: "window", ty: "number", required: false, doc: "Proximity window (mm, default 1.0).", aliases: &[] },
		ParamSpec { name: "tol", ty: "number", required: false, doc: "Chord tolerance for exact tessellation (mm, default 0.05).", aliases: &[] },
	]),
	("asm_interference_volume", &[
		ParamSpec { name: "a", ty: "id-ref", required: true, doc: "First instance (op id).", aliases: &[] },
		ParamSpec { name: "b", ty: "id-ref", required: true, doc: "Second instance (op id).", aliases: &[] },
		ParamSpec { name: "voxel", ty: "number", required: false, doc: "Sampling cell size (mm, default 0.3).", aliases: &[] },
	]),
	("asm_mass_properties", &[]),
	("asm_export", &[
		ParamSpec { name: "file", ty: "string", required: true, doc: "Merged output path (`.stl` / `.3mf`).", aliases: &[] },
		ParamSpec { name: "parts_dir", ty: "string", required: false, doc: "Optional directory for per-instance STLs.", aliases: &[] },
		ParamSpec { name: "tol", ty: "number", required: false, doc: "Chord tolerance for exact tessellation (mm, default 0.05).", aliases: &[] },
		ParamSpec { name: "voxel", ty: "number", required: false, doc: "Voxel size for the watertight heal fallback (mm, default 0.3).", aliases: &[] },
	]),
	("asm_export_step", &[
		ParamSpec { name: "file", ty: "string", required: true, doc: "Output path (`.step`).", aliases: &[] },
	]),
	("asm_save", &[
		ParamSpec { name: "file", ty: "string", required: true, doc: "Output `.lmcasm` path.", aliases: &[] },
		ParamSpec { name: "name", ty: "string", required: false, doc: "Assembly name in the envelope (default: the file stem).", aliases: &[] },
		ParamSpec { name: "parts_dir", ty: "string", required: false, doc: "Directory (relative to the `.lmcasm`) for exported instance meshes.", aliases: &[] },
	]),
	("gear_train_poses", &[
		ParamSpec { name: "sun_teeth", ty: "int", required: true, doc: "Sun tooth count (input).", aliases: &[] },
		ParamSpec { name: "ring1_teeth", ty: "int", required: true, doc: "Grounded ring tooth count.", aliases: &[] },
		ParamSpec { name: "planet_a_teeth", ty: "int", required: true, doc: "First planet band tooth count.", aliases: &[] },
		ParamSpec { name: "planet_b_teeth", ty: "int", required: true, doc: "Second (stepped) planet band tooth count.", aliases: &[] },
		ParamSpec { name: "ring2_teeth", ty: "int", required: true, doc: "Output ring tooth count.", aliases: &[] },
		ParamSpec { name: "n_planets", ty: "int", required: true, doc: "Number of equally spaced planets.", aliases: &[] },
		ParamSpec { name: "module", ty: "number", required: true, doc: "Gear module (mm) — scales tooth counts into radii.", aliases: &[] },
		ParamSpec { name: "theta_deg", ty: "number", required: true, doc: "Input (sun) angle in degrees.", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("gt2_belt", &[
		ParamSpec { name: "center_distance", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "t1", ty: "int", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "t2", ty: "int", required: true, doc: "", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("gt2_center_distance", &[
		ParamSpec { name: "belt_teeth", ty: "int", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "t1", ty: "int", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "t2", ty: "int", required: true, doc: "", aliases: &[] },
	]),
	("iso286_fit", &[
		ParamSpec { name: "d", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "fit", ty: "string", required: true, doc: "", aliases: &[] },
	]),
	("heatset_spec", &[
		ParamSpec { name: "m", ty: "number", required: true, doc: "", aliases: &[] },
	]),
	("metric_cord_gland", &[
		ParamSpec { name: "cord_d", ty: "number", required: true, doc: "", aliases: &[] },
	]),
	("racetrack_cord_length", &[
		ParamSpec { name: "x_len", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "y_len", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "corner_r", ty: "number", required: true, doc: "", aliases: &[] },
	]),
	#[cfg(feature = "catalog")]
	("pipe_thread_g", &[
		ParamSpec { name: "designation", ty: "string", required: true, doc: "\"G1/8\", \"G1/4\", \"G3/8\" or \"G1/2\".", aliases: &[] },
	]),
	("drill", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "at", ty: "[x,y,z]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "axis", ty: "[x,y,z]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "d", ty: "number", required: true, doc: "Hole **diameter** (mm).", aliases: &[] },
		ParamSpec { name: "depth", ty: "number", required: false, doc: "Full-diameter depth of a blind hole (exclusive with `through`).", aliases: &[] },
		ParamSpec { name: "through", ty: "number", required: false, doc: "Material span of a through hole (exclusive with `depth`).", aliases: &[] },
		ParamSpec { name: "segments", ty: "int", required: false, doc: "Tool facet count (default 32).", aliases: &[] },
	]),
	("clearance_hole", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "at", ty: "[x,y,z]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "axis", ty: "[x,y,z]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "m", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "fit", ty: "string", required: false, doc: "", aliases: &[] },
		ParamSpec { name: "segments", ty: "int", required: false, doc: "", aliases: &[] },
	]),
	("counterbore_hole", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "at", ty: "[x,y,z]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "axis", ty: "[x,y,z]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "m", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "fit", ty: "string", required: false, doc: "", aliases: &[] },
		ParamSpec { name: "segments", ty: "int", required: false, doc: "", aliases: &[] },
	]),
	("countersink_hole", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "at", ty: "[x,y,z]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "axis", ty: "[x,y,z]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "m", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "fit", ty: "string", required: false, doc: "", aliases: &[] },
		ParamSpec { name: "segments", ty: "int", required: false, doc: "", aliases: &[] },
	]),
	("tap_drill_hole", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "at", ty: "[x,y,z]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "axis", ty: "[x,y,z]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "m", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "depth", ty: "number", required: false, doc: "Full-diameter depth of a blind pilot (exclusive with `through`).", aliases: &[] },
		ParamSpec { name: "through", ty: "number", required: false, doc: "Material span of a through pilot (exclusive with `depth`).", aliases: &[] },
		ParamSpec { name: "segments", ty: "int", required: false, doc: "", aliases: &[] },
	]),
	("bolt_circle", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "center", ty: "[x,y,z]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "axis", ty: "[x,y,z]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "circle_d", ty: "number", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "n", ty: "int", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "start_deg", ty: "number", required: false, doc: "", aliases: &[] },
		ParamSpec { name: "hole", ty: "object", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "segments", ty: "int", required: false, doc: "", aliases: &[] },
	]),
	("bearing_seat", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "at", ty: "[x,y,z]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "axis", ty: "[x,y,z]", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "bearing", ty: "string", required: true, doc: "Bearing designation: 603, 608, 625, 688, 6000, 6001 or 6804.", aliases: &[] },
		ParamSpec { name: "segments", ty: "int", required: false, doc: "", aliases: &[] },
	]),
	("thread_spec", &[
		ParamSpec { name: "m", ty: "number", required: true, doc: "", aliases: &[] },
	]),
	("thread_ridge", &[
		ParamSpec { name: "m", ty: "number", required: false, doc: "Nominal ISO size (M3–M16 coarse); exclusive with `major_d`+`pitch`.", aliases: &[] },
		ParamSpec { name: "major_d", ty: "number", required: false, doc: "Explicit crest **diameter** (mm); requires `pitch`.", aliases: &[] },
		ParamSpec { name: "pitch", ty: "number", required: false, doc: "Explicit thread pitch (mm); requires `major_d`.", aliases: &[] },
		ParamSpec { name: "z0", ty: "number", required: false, doc: "Axial start of the ridge (default 0).", aliases: &[] },
		ParamSpec { name: "length", ty: "number", required: true, doc: "Axial span of the ridge (`length/pitch` turns, capped at 200).", aliases: &[] },
	]),
	("export_threaded", &[
		ParamSpec { name: "in", ty: "id-ref", required: true, doc: "", aliases: &[] },
		ParamSpec { name: "m", ty: "number", required: true, doc: "Nominal ISO size (M3–M16, coarse pitch).", aliases: &[] },
		ParamSpec { name: "z0", ty: "number", required: false, doc: "Axial start of the threaded span (default 0).", aliases: &[] },
		ParamSpec { name: "length", ty: "number", required: true, doc: "Axial span of the thread (`length/pitch` turns, capped at 200).", aliases: &[] },
		ParamSpec { name: "internal", ty: "bool", required: false, doc: "Cut a female thread into a bore instead of fusing a male one (default false).", aliases: &[] },
		ParamSpec { name: "voxel", ty: "number", required: false, doc: "Voxel size (mm); default pitch/8.", aliases: &[] },
		ParamSpec { name: "file", ty: "string", required: true, doc: "Output mesh path — the extension picks the format (`.stl` / `.3mf`).", aliases: &[] },
	]),
];

/// The parameter specs of one op by wire tag (`None` for an unknown tag) — the lookup behind
/// `describe {name}`.
pub fn op_params(name: &str) -> Option<&'static [ParamSpec]> {
	OP_PARAMS.iter().find(|(tag, _)| *tag == name).map(|(_, specs)| *specs)
}
