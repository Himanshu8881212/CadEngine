// Copyright (c) LMCAD. Licensed under the MIT License.

//! In-program assembly ops (assembly audit 2026-07-17, gap 2): instances,
//! mates, the solver, contacts/interference/mass measures, exports and the
//! `.lmcasm` bridge — all first-class ops in a `run_program` op list, so an
//! agent that builds parts on the flat JSON surface can MATE them there too,
//! without hand-authoring a file in a second vocabulary.
//!
//! State model: one [`AsmProgramState`] per program run. `asm_instance` /
//! `asm_instance_mesh` append instances (referencing bound solids by op id —
//! geometry is fetched from the environment at use time, never cloned);
//! `asm_mate*` append [`Constraint`]s referencing instances by their op ids;
//! `asm_solve` relaxes the poses in place and reports the honesty bundle
//! (per-mate residuals, numeric DOF, solved poses). Everything downstream
//! (contacts, exports, save) reads the CURRENT poses — solve first for mated
//! positions, or don't (seed-pose checks are legitimate too); receipts always
//! carry `solved: bool` so the reader knows which one it got.

use std::collections::BTreeMap;
use std::collections::BTreeSet;
use std::path::Path;

use kernel_brep::Solid;
use kernel_core::math::{Affine3A, DAffine3, DVec3, Quat, Vec3};
use kernel_core::Mesh;
use kernel_model::format::{save_assembly, AsmInstance as FileInstance, AsmSource};
use kernel_model::kinematics::EpicyclicTrain;
use kernel_model::{Constraint, ConstraintSystem};
use serde_json::{json, Value};

use crate::interp::{err, fetch_solid, EnvValue, Outcome};
use crate::ops::meshio::{read_mesh_file, resolve_path, solid_mesh, solid_mesh_routed, write_mesh_auto, write_mesh_scene};
use crate::ops::support::{polygon_centroid, v3a};
use crate::program::{MaterialSpec, RotateSpec};
use crate::report::{ErrorKind, OpError};

/// Geometry an instance draws from.
pub(crate) enum InstanceGeom {
	/// A bound solid, referenced by its op id (fetched from the environment at
	/// use time — solids are bound once and never rebound).
	Solid {
		/// The environment id of the solid.
		solid_ref: String,
		/// `.lmcpart` provenance when the solid came from `load_part` — lets
		/// `asm_save` reference the original file instead of exporting a mesh.
		source_path: Option<String>,
	},
	/// A welded mesh loaded by `asm_instance_mesh`.
	Mesh(Mesh),
}

/// One in-program assembly instance.
pub(crate) struct AsmOpInstance {
	/// The `asm_instance*` op id — how mates reference this instance.
	pub(crate) op_id: String,
	/// Display name (default: the op id).
	pub(crate) name: String,
	pub(crate) geom: InstanceGeom,
	/// Current rigid pose (seed until `asm_solve` overwrites it).
	pub(crate) pose: Affine3A,
	/// Material for mass/BOM receipts.
	pub(crate) material: Option<(String, f64)>,
}

/// The per-program assembly state threaded through the interpreter.
#[derive(Default)]
pub(crate) struct AsmProgramState {
	pub(crate) instances: Vec<AsmOpInstance>,
	pub(crate) mates: Vec<Constraint>,
	/// `.lmcpart` provenance recorded by `load_part` (op id → relative path).
	pub(crate) solid_sources: BTreeMap<String, String>,
	/// Set once `asm_solve` has run (receipts say which poses they measured).
	pub(crate) solved: bool,
}

impl AsmProgramState {
	/// The instance index for an `asm_instance*` op id, or a precise error.
	fn instance_index(&self, op_id: &str, param: &str, name: &str) -> Result<usize, OpError> {
		self.instances.iter().position(|i| i.op_id == name).ok_or_else(|| {
			err(
				ErrorKind::MissingRef,
				format!(
					"op '{op_id}' param '{param}': '{name}' is not an assembly instance — it must be the id of an earlier asm_instance / asm_instance_mesh op (have: {})",
					if self.instances.is_empty() {
						"none yet".to_string()
					} else {
						self.instances.iter().map(|i| i.op_id.as_str()).collect::<Vec<_>>().join(", ")
					}
				),
			)
		})
	}

	/// Guard for ops that need at least `n` instances.
	fn need_instances(&self, op_id: &str, n: usize, what: &str) -> Result<(), OpError> {
		if self.instances.len() < n {
			return Err(err(
				ErrorKind::InvalidParam,
				format!("op '{op_id}': {what} needs at least {n} assembly instance(s), have {}", self.instances.len()),
			));
		}
		Ok(())
	}
}

/// Build the seed pose from the shared translate/rotate spec (rotate about the
/// axis through `rotate.center`, THEN translate — the `pose` op convention).
fn seed_pose(op_id: &str, translate: &Option<[f64; 3]>, rotate: &Option<RotateSpec>) -> Result<Affine3A, OpError> {
	let mut pose = Affine3A::IDENTITY;
	if let Some(r) = rotate {
		let axis = Vec3::new(r.axis[0] as f32, r.axis[1] as f32, r.axis[2] as f32);
		if axis.length() < 1e-12 {
			return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': rotate.axis must be non-zero")));
		}
		let center = Vec3::new(r.center[0] as f32, r.center[1] as f32, r.center[2] as f32);
		let q = Quat::from_axis_angle(axis.normalize(), (r.degrees as f32).to_radians());
		pose = Affine3A::from_translation(center) * Affine3A::from_quat(q) * Affine3A::from_translation(-center);
	}
	if let Some(t) = translate {
		pose = Affine3A::from_translation(Vec3::new(t[0] as f32, t[1] as f32, t[2] as f32)) * pose;
	}
	Ok(pose)
}

/// Register a solid-backed instance.
#[allow(clippy::too_many_arguments)]
pub(crate) fn instance(
	state: &mut AsmProgramState,
	env: &BTreeMap<String, EnvValue>,
	all_ids: &BTreeSet<String>,
	op_id: &str,
	solid: &str,
	name: &Option<String>,
	translate: &Option<[f64; 3]>,
	rotate: &Option<RotateSpec>,
	material: &Option<MaterialSpec>,
) -> Result<Outcome, OpError> {
	fetch_solid(env, all_ids, op_id, "solid", solid)?; // existence + type check now, geometry later
	if state.instances.iter().any(|i| i.op_id == op_id) {
		return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': duplicate instance op id")));
	}
	let pose = seed_pose(op_id, translate, rotate)?;
	let display = name.clone().unwrap_or_else(|| op_id.to_string());
	let index = state.instances.len();
	state.instances.push(AsmOpInstance {
		op_id: op_id.to_string(),
		name: display.clone(),
		geom: InstanceGeom::Solid { solid_ref: solid.to_string(), source_path: state.solid_sources.get(solid).cloned() },
		pose,
		material: material.as_ref().map(|m| (m.name.clone(), m.density_g_cm3)),
	});
	Ok(Outcome::measures(json!({
		"instance": index,
		"name": display,
		"ground": index == 0,
		"note": if index == 0 { "instance 0 is the GROUND frame (never moved by asm_solve)" } else { "free instance" },
	})))
}

/// Register a mesh-file instance.
#[allow(clippy::too_many_arguments)]
pub(crate) fn instance_mesh(
	state: &mut AsmProgramState,
	input_base: &Path,
	op_id: &str,
	file: &str,
	name: &Option<String>,
	translate: &Option<[f64; 3]>,
	rotate: &Option<RotateSpec>,
	material: &Option<MaterialSpec>,
) -> Result<Outcome, OpError> {
	let (mesh, format) = read_mesh_file(op_id, input_base, input_base, file)?;
	let pose = seed_pose(op_id, translate, rotate)?;
	let display = name.clone().unwrap_or_else(|| op_id.to_string());
	let index = state.instances.len();
	let triangles = mesh.triangle_count();
	let watertight = mesh.is_watertight();
	state.instances.push(AsmOpInstance {
		op_id: op_id.to_string(),
		name: display.clone(),
		geom: InstanceGeom::Mesh(mesh),
		pose,
		material: material.as_ref().map(|m| (m.name.clone(), m.density_g_cm3)),
	});
	Ok(Outcome::measures(json!({
		"instance": index,
		"name": display,
		"format": format,
		"triangles": triangles,
		"watertight": watertight,
		"ground": index == 0,
		"route": "mesh",
	})))
}

/// The raw `asm_mate` op: build a [`Constraint`] from explicit local geometry.
#[allow(clippy::too_many_arguments)]
pub(crate) fn mate(
	state: &mut AsmProgramState,
	op_id: &str,
	kind: &str,
	a: &str,
	b: &Option<String>,
	a_point: &Option<[f64; 3]>,
	b_point: &Option<[f64; 3]>,
	a_dir: &Option<[f64; 3]>,
	b_dir: &Option<[f64; 3]>,
	a_axis_point: &Option<[f64; 3]>,
	a_axis_dir: &Option<[f64; 3]>,
	b_axis_point: &Option<[f64; 3]>,
	b_axis_dir: &Option<[f64; 3]>,
	distance: &Option<f64>,
	degrees: &Option<f64>,
) -> Result<Outcome, OpError> {
	let ia = state.instance_index(op_id, "a", a)?;
	let need_b = |b: &Option<String>| -> Result<usize, OpError> {
		let name = b
			.as_ref()
			.ok_or_else(|| err(ErrorKind::InvalidParam, format!("op '{op_id}': mate kind '{kind}' needs 'b' (a second instance)")))?;
		state.instance_index(op_id, "b", name)
	};
	let dv = |p: [f64; 3]| DVec3::new(p[0], p[1], p[2]);
	let need = |field: &Option<[f64; 3]>, what: &str| -> Result<DVec3, OpError> {
		field.map(dv).ok_or_else(|| err(ErrorKind::InvalidParam, format!("op '{op_id}': mate kind '{kind}' needs '{what}'")))
	};
	let constraint = match kind {
		"coincident" => {
			Constraint::Coincident { a: ia, a_point: need(a_point, "a_point")?, b: need_b(b)?, b_point: need(b_point, "b_point")? }
		}
		"distance" => Constraint::Distance {
			a: ia,
			a_point: need(a_point, "a_point")?,
			b: need_b(b)?,
			b_point: need(b_point, "b_point")?,
			distance: distance
				.ok_or_else(|| err(ErrorKind::InvalidParam, format!("op '{op_id}': mate kind 'distance' needs 'distance'")))?,
		},
		"parallel" => Constraint::Parallel { a: ia, a_dir: need(a_dir, "a_dir")?, b: need_b(b)?, b_dir: need(b_dir, "b_dir")? },
		"concentric" => Constraint::Concentric {
			a: ia,
			a_axis_point: need(a_axis_point, "a_axis_point")?,
			a_axis_dir: need(a_axis_dir, "a_axis_dir")?,
			b: need_b(b)?,
			b_axis_point: need(b_axis_point, "b_axis_point")?,
			b_axis_dir: need(b_axis_dir, "b_axis_dir")?,
		},
		"angle" => Constraint::Angle {
			a: ia,
			a_dir: need(a_dir, "a_dir")?,
			b: need_b(b)?,
			b_dir: need(b_dir, "b_dir")?,
			degrees: degrees.ok_or_else(|| err(ErrorKind::InvalidParam, format!("op '{op_id}': mate kind 'angle' needs 'degrees'")))?,
		},
		"axis_distance" => Constraint::AxisDistance {
			a: ia,
			a_axis_point: need(a_axis_point, "a_axis_point")?,
			a_axis_dir: need(a_axis_dir, "a_axis_dir")?,
			b: need_b(b)?,
			b_axis_point: need(b_axis_point, "b_axis_point")?,
			b_axis_dir: need(b_axis_dir, "b_axis_dir")?,
			distance: distance
				.ok_or_else(|| err(ErrorKind::InvalidParam, format!("op '{op_id}': mate kind 'axis_distance' needs 'distance'")))?,
		},
		"fixed" => Constraint::Fixed { instance: ia },
		other => {
			return Err(err(
				ErrorKind::InvalidParam,
				format!(
					"op '{op_id}': unknown mate kind '{other}' — one of coincident, distance, parallel, concentric, angle, axis_distance, fixed"
				),
			));
		}
	};
	let index = state.mates.len();
	state.mates.push(constraint);
	Ok(Outcome::measures(json!({ "mate": index, "kind": kind })))
}

/// Fetch the SOLID of instance `idx` (derived mates need analytic faces).
fn instance_solid<'e>(
	state: &AsmProgramState,
	env: &'e BTreeMap<String, EnvValue>,
	all_ids: &BTreeSet<String>,
	op_id: &str,
	idx: usize,
) -> Result<&'e Solid, OpError> {
	match &state.instances[idx].geom {
		InstanceGeom::Solid { solid_ref, .. } => fetch_solid(env, all_ids, op_id, "instance", solid_ref),
		InstanceGeom::Mesh(_) => Err(err(
			ErrorKind::InvalidParam,
			format!(
				"op '{op_id}': instance '{}' is a mesh — it has no analytic faces to derive a mate from; use asm_mate with explicit geometry",
				state.instances[idx].name
			),
		)),
	}
}

/// The face of `solid` whose polygon centroid is nearest `witness` (the same
/// anchor `list_faces` reports), plus that distance.
fn nearest_face(solid: &Solid, witness: DVec3) -> (kernel_brep::topo::FaceId, f64) {
	let mut best: Option<(kernel_brep::topo::FaceId, f64)> = None;
	for fid in solid.faces() {
		let c = polygon_centroid(&solid.face_polygon(fid));
		let d = (c - witness).length();
		if best.map(|(_, bd)| d < bd).unwrap_or(true) {
			best = Some((fid, d));
		}
	}
	best.expect("a bound solid has faces")
}

/// Derived axis mate (`asm_mate_axis`).
#[allow(clippy::too_many_arguments)]
pub(crate) fn mate_axis(
	state: &mut AsmProgramState,
	env: &BTreeMap<String, EnvValue>,
	all_ids: &BTreeSet<String>,
	op_id: &str,
	a: &str,
	a_witness: [f64; 3],
	b: &str,
	b_witness: [f64; 3],
	distance: &Option<f64>,
) -> Result<Outcome, OpError> {
	let (ia, ib) = (state.instance_index(op_id, "a", a)?, state.instance_index(op_id, "b", b)?);
	let derive = |idx: usize, witness: [f64; 3], side: &str| -> Result<(DVec3, DVec3), OpError> {
		let solid = instance_solid(state, env, all_ids, op_id, idx)?;
		let w = DVec3::new(witness[0], witness[1], witness[2]);
		let (fid, d) = nearest_face(solid, w);
		solid.face_axis(fid).ok_or_else(|| {
			err(
				ErrorKind::InvalidParam,
				format!(
					"op '{op_id}': {side}_witness [{}, {}, {}] picked a face with no axis (nearest face at {d:.3} mm is planar/spherical/freeform) — aim the witness at a cylindrical, conical or toric face (use list_faces on the part to see anchors)",
					witness[0], witness[1], witness[2]
				),
			)
		})
	};
	let (pa, da) = derive(ia, a_witness, "a")?;
	let (pb, db) = derive(ib, b_witness, "b")?;
	let index = state.mates.len();
	let kind = match distance {
		Some(d) => {
			state.mates.push(Constraint::AxisDistance {
				a: ia,
				a_axis_point: pa,
				a_axis_dir: da,
				b: ib,
				b_axis_point: pb,
				b_axis_dir: db,
				distance: *d,
			});
			"axis_distance"
		}
		None => {
			state.mates.push(Constraint::Concentric { a: ia, a_axis_point: pa, a_axis_dir: da, b: ib, b_axis_point: pb, b_axis_dir: db });
			"concentric"
		}
	};
	Ok(Outcome::measures(json!({
		"mate": index,
		"kind": kind,
		"derived": {
			"a": { "axis_point": v3a(pa), "axis_dir": v3a(da) },
			"b": { "axis_point": v3a(pb), "axis_dir": v3a(db) },
		},
	})))
}

/// Derived face mate (`asm_mate_face`): coincident (offset along the normal) +
/// parallel normals.
#[allow(clippy::too_many_arguments)]
pub(crate) fn mate_face(
	state: &mut AsmProgramState,
	env: &BTreeMap<String, EnvValue>,
	all_ids: &BTreeSet<String>,
	op_id: &str,
	a: &str,
	a_witness: [f64; 3],
	b: &str,
	b_witness: [f64; 3],
	offset: &Option<f64>,
) -> Result<Outcome, OpError> {
	let (ia, ib) = (state.instance_index(op_id, "a", a)?, state.instance_index(op_id, "b", b)?);
	let derive = |idx: usize, witness: [f64; 3], side: &str| -> Result<(DVec3, DVec3), OpError> {
		let solid = instance_solid(state, env, all_ids, op_id, idx)?;
		let w = DVec3::new(witness[0], witness[1], witness[2]);
		let (fid, d) = nearest_face(solid, w);
		let centroid = polygon_centroid(&solid.face_polygon(fid));
		solid.face_plane(fid).map(|(_, normal)| (centroid, normal)).ok_or_else(|| {
			err(
				ErrorKind::InvalidParam,
				format!(
						"op '{op_id}': {side}_witness picked a degenerate face (nearest at {d:.3} mm) — aim at a real face (use list_faces for anchors)"
					),
			)
		})
	};
	let (pa, na) = derive(ia, a_witness, "a")?;
	let (pb, nb) = derive(ib, b_witness, "b")?;
	let off = offset.unwrap_or(0.0);
	let index = state.mates.len();
	// Seat b's face point offset·normal off a's face point, normals parallel
	// (anti-parallel counts — the natural mating sense).
	state.mates.push(Constraint::Coincident { a: ia, a_point: pa + na * off, b: ib, b_point: pb });
	state.mates.push(Constraint::Parallel { a: ia, a_dir: na, b: ib, b_dir: nb });
	Ok(Outcome::measures(json!({
		"mates": [index, index + 1],
		"kind": "face (coincident + parallel)",
		"offset": off,
		"derived": {
			"a": { "point": v3a(pa), "normal": v3a(na) },
			"b": { "point": v3a(pb), "normal": v3a(nb) },
		},
	})))
}

/// Quaternion → `[x, y, z, w]` (the `.lmcasm` wire convention).
fn quat_xyzw(q: Quat) -> [f32; 4] {
	[q.x, q.y, q.z, q.w]
}

/// `asm_solve`: relax, write poses back, report the honesty bundle.
pub(crate) fn solve(
	state: &mut AsmProgramState,
	op_id: &str,
	iterations: &Option<usize>,
	max_residual: &Option<f64>,
	allow_unconverged: bool,
) -> Result<Outcome, OpError> {
	state.need_instances(op_id, 1, "asm_solve")?;
	let mut system = ConstraintSystem::new(state.instances.iter().map(|i| i.pose).collect(), state.mates.clone());
	let problems = system.validate();
	if !problems.is_empty() {
		return Err(err(
			ErrorKind::InvalidParam,
			format!("op '{op_id}': statically broken mates (refused, not silently skipped): {}", problems.join("; ")),
		));
	}
	let residual = system.solve(iterations.unwrap_or(256));
	let per_mate = system.per_constraint_residuals();
	let dof = system.analyze();
	for (instance, &pose) in state.instances.iter_mut().zip(system.transforms()) {
		instance.pose = pose;
	}
	state.solved = true;
	let gate = max_residual.unwrap_or(1e-6);
	let converged = residual <= gate;
	let poses: Vec<Value> = state
		.instances
		.iter()
		.map(|i| {
			let (_, r, t) = i.pose.to_scale_rotation_translation();
			json!({
				"instance": i.op_id,
				"name": i.name,
				"translation": [t.x, t.y, t.z],
				"rotation_xyzw": quat_xyzw(r),
			})
		})
		.collect();
	let per_mate_json: Vec<Value> =
		per_mate.iter().enumerate().map(|(k, &r)| json!({ "index": k, "kind": state.mates[k].kind_name(), "residual": r })).collect();
	let measures = json!({
		"residual": residual,
		"max_residual": gate,
		"converged": converged,
		"per_mate": per_mate_json,
		"dof": serde_json::to_value(&dof).expect("DofReport serializes: plain data"),
		"poses": poses,
	});
	if !converged && !allow_unconverged {
		let mut worst: Vec<(usize, f64)> = per_mate.iter().copied().enumerate().collect();
		worst.sort_by(|x, y| y.1.partial_cmp(&x.1).unwrap_or(std::cmp::Ordering::Equal));
		let culprits = worst
			.iter()
			.take(3)
			.filter(|(_, r)| *r > gate)
			.map(|&(k, r)| format!("mate {k} ({}) residual {r:.3e}", state.mates[k].kind_name()))
			.collect::<Vec<_>>()
			.join(", ");
		return Err(err(
			ErrorKind::AssertFailed,
			format!(
				"op '{op_id}': mates did not converge — residual {residual:.3e} exceeds {gate:.0e}; worst offenders: {culprits}. \
				 The mate set is unsatisfiable or stuck in a rotational local optimum ({}; {} redundant rows). \
				 Pass allow_unconverged:true to inspect the receipts anyway",
				dof.verdict, dof.redundant_rows
			),
		));
	}
	Ok(Outcome::measures(measures))
}

/// World-space measurement mesh of one instance: exact adaptive tessellation
/// for solids (vertices on the true analytic surfaces), the welded mesh for
/// mesh instances — transformed by the current pose.
fn measurement_mesh(
	state: &AsmProgramState,
	env: &BTreeMap<String, EnvValue>,
	all_ids: &BTreeSet<String>,
	op_id: &str,
	idx: usize,
	tol: f64,
) -> Result<Mesh, OpError> {
	let mut mesh = match &state.instances[idx].geom {
		InstanceGeom::Solid { solid_ref, .. } => {
			kernel_brep::tessellate_adaptive_tol(fetch_solid(env, all_ids, op_id, "instance", solid_ref)?, tol)
		}
		InstanceGeom::Mesh(m) => m.clone(),
	};
	pose_mesh(&mut mesh, state.instances[idx].pose);
	Ok(mesh)
}

/// Map a mesh into world space by a rigid pose (normals via inverse-transpose,
/// outward orientation restored — mirrors kernel-model's transform).
fn pose_mesh(mesh: &mut Mesh, m: Affine3A) {
	for p in mesh.positions.iter_mut() {
		*p = m.transform_point3(*p);
	}
	let normal_mat = m.matrix3.inverse().transpose();
	for n in mesh.normals.iter_mut() {
		*n = (normal_mat * *n).normalize_or_zero();
	}
	mesh.ensure_outward();
}

/// Separation between two AABBs — a rigorous lower bound on surface distance.
fn aabb_gap(a: (Vec3, Vec3), b: (Vec3, Vec3)) -> f64 {
	let mut d2 = 0.0_f64;
	for k in 0..3 {
		let gap = f64::from((a.0[k] - b.1[k]).max(b.0[k] - a.1[k]).max(0.0));
		d2 += gap * gap;
	}
	d2.sqrt()
}

/// `asm_contacts`: the all-pairs proximity scan at current poses.
pub(crate) fn contacts(
	state: &AsmProgramState,
	env: &BTreeMap<String, EnvValue>,
	all_ids: &BTreeSet<String>,
	op_id: &str,
	window: &Option<f64>,
	tol: &Option<f64>,
) -> Result<Outcome, OpError> {
	state.need_instances(op_id, 2, "asm_contacts")?;
	let window = window.unwrap_or(1.0);
	let tol = tol.unwrap_or(0.05);
	let meshes: Vec<Mesh> =
		(0..state.instances.len()).map(|i| measurement_mesh(state, env, all_ids, op_id, i, tol)).collect::<Result<_, _>>()?;
	let boxes: Vec<(Vec3, Vec3)> = meshes
		.iter()
		.map(|m| {
			let bb = m.aabb();
			(bb.min, bb.max)
		})
		.collect();
	let mut pairs = Vec::new();
	let mut touching = 0usize;
	for i in 0..meshes.len() {
		for j in (i + 1)..meshes.len() {
			if aabb_gap(boxes[i], boxes[j]) > window {
				continue;
			}
			let d = meshes[i].min_distance(&meshes[j]);
			if d <= window {
				let is_touching = d <= 1e-6;
				touching += usize::from(is_touching);
				pairs.push(json!({
					"a": state.instances[i].name,
					"b": state.instances[j].name,
					"i": i,
					"j": j,
					"distance": d,
					"touching": is_touching,
				}));
			}
		}
	}
	Ok(Outcome::measures(json!({
		"pairs": pairs,
		"touching": touching,
		"window": window,
		"tol": tol,
		"solved": state.solved,
	})))
}

/// `asm_interference_volume`: voxel-sampled shared material of two instances,
/// through the same winding-number SDF bridge the file pipeline uses.
pub(crate) fn interference_volume(
	state: &AsmProgramState,
	env: &BTreeMap<String, EnvValue>,
	all_ids: &BTreeSet<String>,
	op_id: &str,
	a: &str,
	b: &str,
	voxel: &Option<f64>,
) -> Result<Outcome, OpError> {
	let (ia, ib) = (state.instance_index(op_id, "a", a)?, state.instance_index(op_id, "b", b)?);
	let voxel = voxel.unwrap_or(0.3);
	if voxel <= 0.0 {
		return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': voxel must be > 0, got {voxel}")));
	}
	let mut asm = kernel_model::Assembly::new();
	for idx in [ia, ib] {
		let inst = &state.instances[idx];
		let mesh = match &inst.geom {
			InstanceGeom::Solid { solid_ref, .. } => {
				kernel_brep::tessellate_default(fetch_solid(env, all_ids, op_id, "instance", solid_ref)?)
			}
			InstanceGeom::Mesh(m) => m.clone(),
		};
		asm.add(kernel_model::Instance::from_mesh(&mesh, inst.pose));
	}
	let volume = asm.interference_volume(0, 1, voxel);
	Ok(Outcome::measures(json!({
		"overlap_volume": volume,
		"voxel": voxel,
		"a": state.instances[ia].name,
		"b": state.instances[ib].name,
		"solved": state.solved,
	})))
}

/// `asm_mass_properties`: honest per-instance volume/mass rollup.
pub(crate) fn mass_properties(
	state: &AsmProgramState,
	env: &BTreeMap<String, EnvValue>,
	all_ids: &BTreeSet<String>,
	op_id: &str,
) -> Result<Outcome, OpError> {
	state.need_instances(op_id, 1, "asm_mass_properties")?;
	let mut lines = Vec::new();
	let mut total_mass = 0.0f64;
	let mut mass_complete = true;
	let mut weighted_com = DVec3::ZERO;
	let mut weight_sum = 0.0f64;
	for inst in &state.instances {
		let (volume, source, local_com) = match &inst.geom {
			InstanceGeom::Solid { solid_ref, .. } => {
				let solid = fetch_solid(env, all_ids, op_id, "instance", solid_ref)?;
				let mp = kernel_brep::mass_properties(solid);
				(mp.volume, "exact", mp.center_of_mass)
			}
			InstanceGeom::Mesh(m) => {
				let mp = m.mass_properties();
				(mp.volume, "mesh", mp.center_of_mass)
			}
		};
		let world_com = {
			let c = inst.pose.transform_point3(Vec3::new(local_com.x as f32, local_com.y as f32, local_com.z as f32));
			DVec3::new(f64::from(c.x), f64::from(c.y), f64::from(c.z))
		};
		let mass_g = inst.material.as_ref().map(|(_, density)| density * volume / 1000.0);
		if let Some(m) = mass_g {
			total_mass += m;
			weighted_com += world_com * m;
			weight_sum += m;
		} else {
			mass_complete = false;
			// Volume-weight the COM when a material is missing so the aggregate
			// still means something — said in the receipt.
			weighted_com += world_com * volume;
			weight_sum += volume;
		}
		lines.push(json!({
			"name": inst.name,
			"volume_mm3": volume,
			"volume_source": source,
			"material": inst.material.as_ref().map(|(n, _)| n.clone()),
			"density_g_cm3": inst.material.as_ref().map(|(_, d)| *d),
			"mass_g": mass_g,
			"center_of_mass": v3a(world_com),
		}));
	}
	let com = if weight_sum > 0.0 { weighted_com / weight_sum } else { DVec3::ZERO };
	Ok(Outcome::measures(json!({
		"instances": lines,
		"total_mass_g": if mass_complete { Value::from(total_mass) } else { Value::Null },
		"mass_complete": mass_complete,
		"note": if mass_complete {
			"total_mass_g = Σ density × volume; COM is mass-weighted"
		} else {
			"one or more instances have no material — total mass omitted (honest), COM weighting mixes mass and volume"
		},
		"center_of_mass": v3a(com),
		"solved": state.solved,
	})))
}

/// `asm_export`: merged (and optionally per-instance) mesh export at current poses.
#[allow(clippy::too_many_arguments)]
pub(crate) fn export(
	state: &AsmProgramState,
	env: &BTreeMap<String, EnvValue>,
	all_ids: &BTreeSet<String>,
	op_id: &str,
	out_dir: &Path,
	file: &str,
	parts_dir: &Option<String>,
	tol: &Option<f64>,
	voxel: &Option<f64>,
) -> Result<Outcome, OpError> {
	state.need_instances(op_id, 1, "asm_export")?;
	let tol = tol.unwrap_or(0.05);
	let voxel = voxel.unwrap_or(0.3);
	let mut merged = Mesh::new();
	let mut per_instance = Vec::new();
	for inst in &state.instances {
		let (mut mesh, route, _, demotion) = match &inst.geom {
			InstanceGeom::Solid { solid_ref, .. } => {
				let solid = fetch_solid(env, all_ids, op_id, "instance", solid_ref)?;
				solid_mesh_routed(solid, tol, voxel)
			}
			InstanceGeom::Mesh(m) => (m.clone(), "mesh", 0.0, None),
		};
		pose_mesh(&mut mesh, inst.pose);
		let mut entry = json!({
			"name": inst.name,
			"route": route,
			"triangles": mesh.triangle_count(),
			"watertight": mesh.is_watertight(),
		});
		// Same receipt as `export_stl`: why the exact route was abandoned, with
		// witnesses in the PART's own frame (before the instance pose).
		if let Some(demotion) = demotion {
			entry["demotion"] = demotion;
		}
		if let Some(dir) = parts_dir {
			let rel = format!("{dir}/{}.stl", sanitize(&inst.name));
			let written = write_mesh_auto(op_id, out_dir, &rel, &mesh)?;
			entry["file"] = json!(written);
		}
		append_soup(&mut merged, &mesh);
		per_instance.push(entry);
	}
	// The MERGED file is a diagnostic SCENE (posed instances, possibly a
	// negative-control failure attitude that interpenetrates BY DESIGN), never a
	// print file — per-instance part files above stay on the strict path. The
	// scene write skips the manufacturing refusal and instead puts the quality
	// counters on the record here, so a self-intersecting fail pose exports
	// with its interference VISIBLE in the receipt rather than failing the run.
	let written = write_mesh_scene(op_id, out_dir, file, &merged)?;
	let scene_crossings = merged.self_intersection_witness().map_or(0, |w| w.pairs);
	Ok(Outcome {
		value: None,
		measures: Some(json!({
			"instances": per_instance,
			"triangles": merged.triangle_count(),
			"watertight": merged.is_watertight(),
			"scene": true,
			"scene_policy": "diagnostic export: manufacturing refusal not applied to the merged scene; print files are the per-instance parts",
			"cross_instance_self_intersections": scene_crossings,
			"solved": state.solved,
		})),
		file: Some(written),
	})
}

/// `asm_export_step`: AP214 assembly of the solid-backed instances.
pub(crate) fn export_step(
	state: &AsmProgramState,
	env: &BTreeMap<String, EnvValue>,
	all_ids: &BTreeSet<String>,
	op_id: &str,
	out_dir: &Path,
	file: &str,
) -> Result<Outcome, OpError> {
	state.need_instances(op_id, 1, "asm_export_step")?;
	let mut parts: Vec<(String, Solid, DAffine3)> = Vec::new();
	let mut skipped = Vec::new();
	for inst in &state.instances {
		match &inst.geom {
			InstanceGeom::Solid { solid_ref, .. } => {
				let solid = fetch_solid(env, all_ids, op_id, "instance", solid_ref)?;
				let (_, r, t) = inst.pose.to_scale_rotation_translation();
				let pose = DAffine3::from_rotation_translation(r.as_dquat().normalize(), t.as_dvec3());
				parts.push((inst.name.clone(), solid.clone(), pose));
			}
			InstanceGeom::Mesh(_) => {
				skipped.push(json!({ "instance": inst.name, "why": "mesh instance — no B-rep to write into STEP" }));
			}
		}
	}
	if parts.is_empty() {
		return Err(err(
			ErrorKind::InvalidParam,
			format!("op '{op_id}': no solid-backed instance to export — STEP carries B-rep only (all instances are meshes)"),
		));
	}
	let text = kernel_brep::export_step_assembly(&parts, "assembly")
		.map_err(|e| err(ErrorKind::InvalidGeometry, format!("op '{op_id}': STEP assembly export refused: {e}")))?;
	let path = resolve_path(op_id, out_dir, file)?;
	std::fs::write(&path, &text).map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot write '{}': {e}", path.display())))?;
	Ok(Outcome {
		value: None,
		measures: Some(json!({
			"parts": parts.len(),
			"skipped": skipped,
			"bytes": text.len(),
			"solved": state.solved,
		})),
		file: Some(path.display().to_string()),
	})
}

/// `asm_save`: persist the in-program assembly as a re-executable `.lmcasm`.
#[allow(clippy::too_many_arguments)]
pub(crate) fn save(
	state: &AsmProgramState,
	env: &BTreeMap<String, EnvValue>,
	all_ids: &BTreeSet<String>,
	op_id: &str,
	out_dir: &Path,
	file: &str,
	name: &Option<String>,
	parts_dir: &Option<String>,
) -> Result<Outcome, OpError> {
	state.need_instances(op_id, 1, "asm_save")?;
	if !std::path::Path::new(file).extension().map(|e| e == "lmcasm").unwrap_or(false) {
		return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': 'file' must end in .lmcasm, got '{file}'")));
	}
	let asm_path = resolve_path(op_id, out_dir, file)?;
	let asm_parent = asm_path.parent().map(Path::to_path_buf).unwrap_or_default();
	let parts_rel = parts_dir.clone().unwrap_or_else(|| "parts".to_string());
	let mut instances = Vec::new();
	let mut sources = Vec::new();
	for inst in &state.instances {
		let source = match &inst.geom {
			InstanceGeom::Solid { solid_ref, source_path: Some(p) } => {
				// The solid came from load_part: reference the ORIGINAL .lmcpart
				// (copied next to the assembly so the file stays relocatable).
				let file_name = std::path::Path::new(p)
					.file_name()
					.map(|s| s.to_string_lossy().into_owned())
					.unwrap_or_else(|| format!("{}.lmcpart", sanitize(&inst.name)));
				let _ = solid_ref;
				sources.push(json!({ "instance": inst.name, "source": "path", "file": file_name }));
				AsmSource::Path(file_name)
			}
			InstanceGeom::Solid { solid_ref, source_path: None } => {
				// Program-built geometry: export its exact-else-heal mesh next to
				// the assembly and reference it with the mesh source.
				let solid = fetch_solid(env, all_ids, op_id, "instance", solid_ref)?;
				let (mesh, route, _) = solid_mesh(solid, 0.05, 0.3);
				let rel = format!("{parts_rel}/{}.stl", sanitize(&inst.name));
				let path = asm_parent.join(&rel);
				if let Some(parent) = path.parent() {
					std::fs::create_dir_all(parent)
						.map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot create '{}': {e}", parent.display())))?;
				}
				mesh.write_stl_binary(&path)
					.map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot write '{}': {e}", path.display())))?;
				sources.push(json!({ "instance": inst.name, "source": "mesh", "file": rel, "route": route }));
				AsmSource::Mesh(rel)
			}
			InstanceGeom::Mesh(m) => {
				let rel = format!("{parts_rel}/{}.stl", sanitize(&inst.name));
				let path = asm_parent.join(&rel);
				if let Some(parent) = path.parent() {
					std::fs::create_dir_all(parent)
						.map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot create '{}': {e}", parent.display())))?;
				}
				m.write_stl_binary(&path)
					.map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot write '{}': {e}", path.display())))?;
				sources.push(json!({ "instance": inst.name, "source": "mesh", "file": rel, "route": "mesh" }));
				AsmSource::Mesh(rel)
			}
		};
		instances.push(FileInstance { name: Some(inst.name.clone()), source, pose: inst.pose, suppressed: false });
	}
	// Copy load_part-referenced .lmcpart files next to the assembly.
	for (inst, src) in state.instances.iter().zip(&sources) {
		if src["source"] == "path" {
			if let InstanceGeom::Solid { source_path: Some(original), .. } = &inst.geom {
				let dest = asm_parent.join(src["file"].as_str().expect("path source has a file"));
				let from = crate::ops::meshio::resolve_input_path(op_id, out_dir, original)
					.or_else(|_| Ok::<_, OpError>(std::path::PathBuf::from(original)))?;
				std::fs::copy(&from, &dest).map_err(|e| {
					err(ErrorKind::Io, format!("op '{op_id}': cannot copy '{}' → '{}': {e}", from.display(), dest.display()))
				})?;
			}
		}
	}
	let asm_name = name
		.clone()
		.unwrap_or_else(|| asm_path.file_stem().map(|s| s.to_string_lossy().into_owned()).unwrap_or_else(|| "assembly".to_string()));
	let text = save_assembly(&asm_name, &instances, &state.mates)
		.map_err(|e| err(ErrorKind::InvalidParam, format!("op '{op_id}': cannot serialize assembly: {e}")))?;
	std::fs::write(&asm_path, text).map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot write '{}': {e}", asm_path.display())))?;
	Ok(Outcome {
		value: None,
		measures: Some(json!({
			"instances": state.instances.len(),
			"mates": state.mates.len(),
			"sources": sources,
			"solved": state.solved,
			"note": "re-executable via `kernel-api asm` / MCP run_assembly — mates re-solve on every load",
		})),
		file: Some(asm_path.display().to_string()),
	})
}

/// `gear_train_poses`: the kinematics→assembly bridge as an op.
#[allow(clippy::too_many_arguments)]
pub(crate) fn gear_train_poses(
	op_id: &str,
	sun_teeth: usize,
	ring1_teeth: usize,
	planet_a_teeth: usize,
	planet_b_teeth: usize,
	ring2_teeth: usize,
	n_planets: usize,
	module: f64,
	theta_deg: f64,
) -> Result<Outcome, OpError> {
	if module <= 0.0 {
		return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': module must be > 0, got {module}")));
	}
	let train = EpicyclicTrain { sun_teeth, ring1_teeth, planet_a_teeth, planet_b_teeth, ring2_teeth, n_planets };
	train.validate_assembly().map_err(|e| err(ErrorKind::InvalidParam, format!("op '{op_id}': train not assemblable: {e}")))?;
	let theta = theta_deg.to_radians();
	let poses = train.instance_poses(theta, module);
	let angles = train.poses(theta);
	let planets: Vec<Value> = poses
		.planets
		.iter()
		.zip(&angles.planets)
		.map(|(pose, p)| {
			json!({
				"translation": v3a(pose.transform_point3(DVec3::ZERO)),
				"rotation_deg": p.spin.to_degrees(),
				"azimuth_deg": p.azimuth.to_degrees(),
				"install_phase_deg": p.install_phase.to_degrees(),
			})
		})
		.collect();
	Ok(Outcome::measures(json!({
		"ratio": train.ratio(),
		"orbit_radius_mm": poses.orbit_radius_mm,
		"axis": [0.0, 0.0, 1.0],
		"sun": { "rotation_deg": angles.sun.to_degrees(), "install_phase_deg": angles.sun_install_phase.to_degrees() },
		"carrier": { "rotation_deg": angles.carrier.to_degrees() },
		"ring2": { "rotation_deg": angles.ring2.to_degrees(), "install_phase_deg": angles.ring2_install_phase.to_degrees() },
		"planets": planets,
		"note": "feed translation + {axis:[0,0,1], degrees:rotation_deg} into asm_instance.rotate/translate; members modelled at install phase mesh exactly",
	})))
}

/// `name` reduced to a filesystem-safe stem.
fn sanitize(name: &str) -> String {
	let s: String = name.chars().map(|c| if c.is_ascii_alphanumeric() || c == '-' || c == '_' { c } else { '_' }).collect();
	if s.is_empty() {
		"instance".to_string()
	} else {
		s
	}
}

/// Append `src`'s triangles onto `dst` (index-rebased; no weld — per-instance
/// meshes stay their own shells inside the merged export).
fn append_soup(dst: &mut Mesh, src: &Mesh) {
	let base = dst.positions.len() as u32;
	dst.positions.extend_from_slice(&src.positions);
	dst.normals.extend_from_slice(&src.normals);
	dst.indices.extend(src.indices.iter().map(|&i| i + base));
}
