// Copyright (c) LMCAD. Licensed under the MIT License.

//! File in and out — the ops under the `Exports`, `Native formats` and `Imports`
//! banners of the dispatcher. The exports (`export_stl` / `export_3mf` /
//! `export_step`), the `.lmcpart` native loader, and the import wave: STEP back in
//! as exact B-reps, meshes with an honest `check_mesh` receipt, the mesh∘solid
//! `mesh_carve` bridge, `measure_dimension` on an imported face, the `tpms`
//! lattice and the `hybrid_boolean` route selector.

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

use kernel_brep::math::DVec3;
use kernel_brep::StepError;
use kernel_core::math::Vec3;
use kernel_core::{check_mesh, make_manifold, Aabb, Sdf};
#[cfg(feature = "catalog")]
use kernel_core::Resolution;
use kernel_implicit::{mesh_boolean_implicit, BoolOp};
#[cfg(feature = "catalog")]
use kernel_implicit::manifold_dual_contour;
use kernel_model::{format, hybrid_boolean, BooleanOp, HybridError, HybridOperand, HybridRoute};
use serde_json::{json, Value};

use crate::implicit;
use crate::interp::{err, fetch_solid, EnvValue, Outcome};
use crate::program::{BoolOpSpec, OpKind};
use crate::report::{ErrorKind, OpError};

use super::meshio::{
	export_mesh, mesh_receipt, read_mesh_file, resolve_input_or_out, resolve_path, solid_mesh, write_mesh_auto, write_mesh_healed
};
use super::support::{bind_solid, grid_guard, polygon_centroid, v3a};

/// `import_step` in `tolerant` mode: the kernel's tolerant importer, whose
/// receipt — every solid of the file with its product name, status and placed
/// envelope; every skip and repair with the entity id and verbatim reason —
/// becomes the measures, and the compound of the imported solids binds. Nothing
/// imported is a loud `invalid_geometry` whose message carries the counts and
/// the first reasons (a bound solid is what the op promises; the envelope census
/// is a measure of a *successful* import, never a substitute for one).
pub(crate) fn import_step_tolerant_op(op_id: &str, path: &Path, text: &str) -> Result<Outcome, OpError> {
	use kernel_brep::{ImportEvent, SolidStatus};
	let imp = kernel_brep::import_step_tolerant(text).map_err(|e| {
		let kind = match &e {
			StepError::Topology(_) => ErrorKind::InvalidGeometry,
			_ => ErrorKind::InvalidParam,
		};
		err(kind, format!("op '{op_id}': import_step '{}' (tolerant): {e}", path.display()))
	})?;
	let v3 = |v: kernel_core::math::DVec3| json!([v.x, v.y, v.z]);
	let event = |e: &ImportEvent| json!({ "entity": e.entity, "kind": e.kind, "solid": e.solid, "reason": e.reason });
	let solids: Vec<Value> = imp
		.solids
		.iter()
		.map(|s| {
			let mut o = json!({
				"name": s.name,
				"path": s.path,
				"entity": s.entity,
				"status": s.status.as_str(),
				"bbox_min": v3(s.bbox_min),
				"bbox_max": v3(s.bbox_max),
				"bbox_source": s.bbox_source,
				"faces": s.faces,
				"faces_repaired": s.faces_repaired,
				"faces_skipped": s.faces_skipped,
			});
			if let Some(reason) = &s.reason {
				o["reason"] = json!(reason);
			}
			o
		})
		.collect();
	let total = imp.solids.len();
	let imported = imp.solids.iter().filter(|s| s.status == SolidStatus::Imported).count();
	let faces_skipped = imp.skipped.iter().filter(|e| e.kind == "ADVANCED_FACE").count();
	let faces_repaired = imp.repaired.iter().filter(|e| e.kind == "ADVANCED_FACE").count();
	let Some(solid) = imp.solid else {
		let first: Vec<String> =
			imp.skipped.iter().take(5).map(|e| format!("#{} {} ({}): {}", e.entity, e.kind, e.solid, e.reason)).collect();
		return Err(err(
			ErrorKind::InvalidGeometry,
			format!(
				"op '{op_id}': import_step '{}' (tolerant): none of the {total} solid(s) could be imported — {} skip(s), {} repair(s); first skips: {}",
				path.display(),
				imp.skipped.len(),
				imp.repaired.len(),
				first.join(" | ")
			),
		));
	};
	let v = kernel_brep::validate(&solid);
	let measures = json!({
		"source": "step",
		"mode": "tolerant",
		"shells": v.shells,
		"genus": v.genus,
		"faces": solid.face_count(),
		"volume": kernel_brep::volume(&solid),
		"freeform_faces": imp.freeform.len(),
		"uncertainty_mm": imp.uncertainty,
		"solids_total": total,
		"solids_imported": imported,
		"solids_skipped": total - imported,
		"faces_skipped": faces_skipped,
		"faces_repaired": faces_repaired,
		"solids": solids,
		"skipped": imp.skipped.iter().map(event).collect::<Vec<Value>>(),
		"repaired": imp.repaired.iter().map(event).collect::<Vec<Value>>(),
	});
	Ok(Outcome { measures: Some(measures), ..bind_solid(op_id, "import_step", solid)? })
}

/// Execute one op of this family. The dispatch table in [`crate::interp`]
/// routes exactly the variants matched below, so the catch-all is dead code
/// kept only to satisfy the compiler.
pub(crate) fn exec(
	op_id: &str,
	env: &mut BTreeMap<String, EnvValue>,
	all_ids: &BTreeSet<String>,
	asm: &mut crate::asmops::AsmProgramState,
	out_dir: &Path,
	input_base: &Path,
	kind: OpKind,
) -> Result<Outcome, OpError> {
	match kind {
		OpKind::ExportStl { input, file, tol, voxel } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			export_mesh(op_id, s, tol, voxel, out_dir, &file, "stl")
		}
		OpKind::Export3mf { input, file, tol, voxel } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			export_mesh(op_id, s, tol, voxel, out_dir, &file, "3mf")
		}
		OpKind::ExportStep { input, file } => {
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let path = resolve_path(op_id, out_dir, &file)?;
			let name = path.file_stem().and_then(|s| s.to_str()).unwrap_or("part").to_string();
			let step = kernel_brep::export_step(s, &name);
			std::fs::write(&path, step).map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot write '{}': {e}", path.display())))?;
			Ok(Outcome { value: None, measures: None, file: Some(path.display().to_string()) })
		}

		OpKind::LoadPart { file } => {
			// T4: program-relative first, then --out-dir (a generated .lmcpart lands there).
			let path = resolve_input_or_out(op_id, input_base, out_dir, &file)?;
			let text = std::fs::read_to_string(&path)
				.map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot read '{}': {e}", path.display())))?;
			let (doc, meta) = format::load_part(&text)
				.map_err(|e| err(ErrorKind::InvalidParam, format!("op '{op_id}': '{}' is not a loadable .lmcpart: {e}", path.display())))?;
			let solid = doc.evaluate_brep().ok_or_else(|| {
				err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': the part's feature tree produced no exact B-rep (voxel-half-only features — shell, gyroid, smooth booleans — cannot enter the solid environment)"),
				)
			})?;
			let outcome = bind_solid(op_id, "load_part", solid)?;
			// Provenance for asm_save: an instance built from this solid can
			// reference the ORIGINAL .lmcpart instead of exporting a mesh.
			asm.solid_sources.insert(op_id.to_string(), file.clone());
			Ok(Outcome {
				measures: Some(json!({ "name": meta.name, "units": meta.units, "created_with": meta.created_with })),
				..outcome
			})
		}

		OpKind::ImportStep { file, mode } => {
			// STEP → exact B-rep, through the kernel's analytic importer. A multi-solid
			// file merges into ONE multi-shell solid (each MANIFOLD_SOLID_BREP keeps its
			// own shell — `shells` in the measures is the honest count). Trimmed-NURBS
			// faces enter as their chord facets; `freeform_faces` counts them.
			// T4: fall back to --out-dir so an exported STEP re-imports under any out dir.
			let path = resolve_input_or_out(op_id, input_base, out_dir, &file)?;
			let text = std::fs::read_to_string(&path)
				.map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot read '{}': {e}", path.display())))?;
			if mode == crate::program::StepImportMode::Tolerant {
				return import_step_tolerant_op(op_id, &path, &text);
			}
			let (solid, freeform) = kernel_brep::import_step_freeform(&text).map_err(|e| {
				// Parse/Reference/Unsupported are input problems (the message carries the
				// kernel's verbatim reason); Topology means the faces don't form a solid.
				let kind = match &e {
					StepError::Topology(_) => ErrorKind::InvalidGeometry,
					_ => ErrorKind::InvalidParam,
				};
				err(kind, format!("op '{op_id}': import_step '{}': {e}", path.display()))
			})?;
			let v = kernel_brep::validate(&solid);
			let measures = json!({
				"source": "step",
				"shells": v.shells,
				"genus": v.genus,
				"faces": solid.face_count(),
				"volume": kernel_brep::volume(&solid),
				"freeform_faces": freeform.len(),
			});
			Ok(Outcome { measures: Some(measures), ..bind_solid(op_id, "import_step", solid)? })
		}
		OpKind::ImportMesh { file, heal, out } => {
			// Mesh file → welded mesh → full check_mesh receipt. Binds NOTHING (the
			// environment stays Solid|Sketch); `volume` is reported ONLY when the mesh
			// is watertight — a leaky mesh has no defined enclosed volume.
			let (mut mesh, mesh_format) = read_mesh_file(op_id, input_base, out_dir, &file)?;
			if heal {
				// The kernel's deterministic import repair: cap boundary loops, then
				// split non-manifold junctions (never worse than the input).
				mesh.fill_holes();
				mesh = make_manifold(&mesh);
			}
			let report = check_mesh(&mesh);
			if heal && !report.watertight {
				return Err(err(
					ErrorKind::InvalidGeometry,
					format!(
						"op '{op_id}': '{file}' is still not watertight after healing (boundary_edges={}, non_manifold_edges={}, non_orientable_edges={}) — route it through the voxel half instead (e.g. `mesh_carve` re-meshes watertight) or repair it upstream",
						report.boundary_edges, report.non_manifold_edges, report.non_orientable_edges
					),
				));
			}
			let bb = mesh.aabb();
			let mut m = serde_json::Map::new();
			m.insert("format".into(), json!(mesh_format));
			m.insert("triangles".into(), json!(mesh.triangle_count()));
			m.insert("healed".into(), json!(heal));
			mesh_receipt(&mut m, &report);
			m.insert("bbox_min".into(), json!([bb.min.x, bb.min.y, bb.min.z]));
			m.insert("bbox_max".into(), json!([bb.max.x, bb.max.y, bb.max.z]));
			if report.watertight {
				m.insert("volume".into(), json!(mesh.signed_volume()));
			}
			let written = match out {
				Some(f) => Some(write_mesh_healed(op_id, out_dir, &f, &mesh)?),
				None => None,
			};
			// `import_mesh` BINDS the mesh it read: a print file that came from
			// anywhere — this engine's voxel route, another tool, a repaired STL —
			// becomes gateable with the ordinary measures.
			Ok(Outcome { value: Some(EnvValue::Mesh(mesh.clone())), measures: Some(Value::Object(m)), file: written })
		}
		OpKind::MeshCarve { input, file, bool_op, voxel, out } => {
			// The hybrid solid∘mesh boolean: the bound solid is meshed on the honest
			// exact-else-heal route, the mesh file is welded in, and both are lifted
			// into winding-number SDFs and re-meshed by the voxel boolean. The result
			// is GUARANTEED a closed 2-manifold, but the seam is VOXEL-RESAMPLED —
			// accurate to `voxel`, never exact — hence route "voxel_implicit".
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			if !(voxel.is_finite() && voxel > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': voxel must be a positive voxel size in mm")));
			}
			let (a, _, _) = solid_mesh(s, 0.01, voxel);
			let (b, _) = read_mesh_file(op_id, input_base, out_dir, &file)?;
			// The boolean's lattice spans BOTH operands — same allocation cap as `shell`.
			grid_guard(op_id, "mesh_carve", a.aabb().union(b.aabb()).pad(2.0 * voxel as f32), voxel)?;
			let op = match bool_op {
				BoolOpSpec::Union => BoolOp::Union,
				BoolOpSpec::Difference => BoolOp::Difference,
				BoolOpSpec::Intersection => BoolOp::Intersection,
			};
			let mesh = mesh_boolean_implicit(&a, &b, op, voxel);
			if mesh.triangle_count() == 0 || !mesh.is_watertight() {
				return Err(err(
					ErrorKind::InvalidGeometry,
					format!(
						"op '{op_id}': mesh_carve produced no watertight result (triangles={}) — an empty boolean (e.g. an intersection of disjoint parts) or a voxel ({voxel} mm) too coarse to resolve the operands",
						mesh.triangle_count()
					),
				));
			}
			let path = write_mesh_healed(op_id, out_dir, &out, &mesh)?;
			Ok(Outcome {
				value: Some(EnvValue::Mesh(mesh.clone())),
				measures: Some(json!({
					"route": "voxel_implicit",
					"triangles": mesh.triangle_count(),
					"watertight": true,
					"volume": mesh.signed_volume(),
					"voxel": voxel,
				})),
				file: Some(path),
			})
		}

		OpKind::MeasureDimension { input, kind, a, b, near } => {
			// FRICTION #21: one dimension, exact where the analytic tags allow,
			// with the receipts a drawing callout needs. Face selection is by
			// nearest face-polygon centroid to the witness — deterministic and
			// the same anchor `list_faces` reports.
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let pick_face = |witness: [f64; 3]| -> (usize, kernel_brep::topo::FaceId, DVec3) {
				let w = DVec3::new(witness[0], witness[1], witness[2]);
				let mut best = None;
				for (i, fid) in s.faces().enumerate() {
					let c = polygon_centroid(&s.face_polygon(fid));
					let d = (c - w).length();
					if best.as_ref().map(|&(_, _, _, bd)| d < bd).unwrap_or(true) {
						best = Some((i, fid, c, d));
					}
				}
				let (i, fid, c, _) = best.expect("a bound solid has faces");
				(i, fid, c)
			};
			match kind.as_str() {
				"point_point" => {
					let (pa, pb) = match (a, b) {
						(Some(pa), Some(pb)) => (pa, pb),
						_ => {
							return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': kind 'point_point' needs both 'a' and 'b' points")));
						}
					};
					let va = DVec3::new(pa[0], pa[1], pa[2]);
					let vb = DVec3::new(pb[0], pb[1], pb[2]);
					Ok(Outcome::measures(json!({
						"kind": "point_point",
						"value": (va - vb).length(),
						"provenance": "coordinates",
						"a": pa, "b": pb,
						"delta": v3a(vb - va),
					})))
				}
				"face_face" => {
					let (wa, wb) = match (a, b) {
						(Some(wa), Some(wb)) => (wa, wb),
						_ => {
							return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': kind 'face_face' needs witness points 'a' and 'b'")));
						}
					};
					let (ia, fa, ca) = pick_face(wa);
					let (ib, fb, cb) = pick_face(wb);
					let plane = |fid| match s.face(fid).surface {
						kernel_brep::Surface::Plane { origin, normal } => Some((origin, normal.normalize())),
						_ => None,
					};
					let (Some((oa, na)), Some((ob, nb))) = (plane(fa), plane(fb)) else {
						let ty = |fid| match s.face(fid).surface {
							kernel_brep::Surface::Plane { .. } => "plane",
							kernel_brep::Surface::Cylinder { .. } => "cylinder",
							kernel_brep::Surface::Sphere { .. } => "sphere",
							kernel_brep::Surface::Cone { .. } => "cone",
							kernel_brep::Surface::Torus { .. } => "torus",
						};
						return Err(err(
							ErrorKind::InvalidParam,
							format!(
								"op '{op_id}': face_face needs two PLANAR faces; the witnesses selected {} (face {ia}) and {} (face {ib}) — move the witnesses or use 'diameter' for curved faces",
								ty(fa),
								ty(fb)
							),
						));
					};
					let align = na.dot(nb).abs();
					if align < 1.0 - 1e-9 {
						let angle_deg = align.clamp(-1.0, 1.0).acos().to_degrees();
						return Err(err(
							ErrorKind::InvalidParam,
							format!(
								"op '{op_id}': face_face needs PARALLEL planes; faces {ia} and {ib} meet at {angle_deg:.4}° — a between-planes distance is not defined"
							),
						));
					}
					Ok(Outcome::measures(json!({
						"kind": "face_face",
						"value": (ob - oa).dot(na).abs(),
						"provenance": "analytic",
						"face_a": {"index": ia, "point": v3a(oa), "normal": v3a(na), "witness": v3a(ca)},
						"face_b": {"index": ib, "point": v3a(ob), "normal": v3a(nb), "witness": v3a(cb)},
					})))
				}
				"diameter" => {
					let Some(w) = near else {
						return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': kind 'diameter' needs a 'near' witness point")));
					};
					let (i, fid, c) = pick_face(w);
					match s.face(fid).surface {
						kernel_brep::Surface::Cylinder { origin, axis, radius } => Ok(Outcome::measures(json!({
							"kind": "diameter",
							"value": 2.0 * radius,
							"provenance": "analytic",
							"face": {"index": i, "type": "cylinder", "point": v3a(origin), "axis": v3a(axis.normalize()), "radius": radius, "witness": v3a(c)},
						}))),
						kernel_brep::Surface::Sphere { center, radius } => Ok(Outcome::measures(json!({
							"kind": "diameter",
							"value": 2.0 * radius,
							"provenance": "analytic",
							"face": {"index": i, "type": "sphere", "center": v3a(center), "radius": radius, "witness": v3a(c)},
						}))),
						kernel_brep::Surface::Cone { half_angle, .. } => Err(err(
							ErrorKind::InvalidParam,
							format!(
								"op '{op_id}': the face nearest 'near' (face {i}) is a CONE (half-angle {half_angle:.4} rad) — its Ø varies along the axis; measure a point_point at a chosen station instead"
							),
						)),
						kernel_brep::Surface::Torus { major, minor, .. } => Err(err(
							ErrorKind::InvalidParam,
							format!(
								"op '{op_id}': the face nearest 'near' (face {i}) is a TORUS (major {major}, minor {minor}) — name the circle you mean via point_point instead"
							),
						)),
						kernel_brep::Surface::Plane { .. } => Err(err(
							ErrorKind::InvalidParam,
							format!("op '{op_id}': the face nearest 'near' (face {i}) is a PLANE — 'diameter' needs a cylindrical or spherical face; move the witness onto the bore/boss wall"),
						)),
					}
				}
				other => Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': kind must be 'point_point' / 'face_face' / 'diameter', got {other:?}"),
				)),
			}
		}

		#[cfg(feature = "catalog")]
		OpKind::Tpms { kind, min, max, cell, mode, level, voxel, file } => {
			// One vocabulary: build the `implicit` tree's `tpms` leaf verbatim and
			// run it through the SAME parser — kind strings, mode/level semantics,
			// bounds checks and the `primitive_bound` field-quality wrapping all
			// come from one place (kernel-api/implicit.rs), not a twin re-encoding.
			if !(voxel.is_finite() && voxel > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': voxel must be a positive voxel size in mm")));
			}
			let mut leaf = json!({
				"shape": "tpms",
				"kind": kind,
				"min": min,
				"max": max,
				"cell": cell,
			});
			if let Some(mode) = &mode {
				leaf["mode"] = json!(mode);
			}
			if let Some(level) = level {
				leaf["level"] = json!(level);
			}
			// A raw TPMS is an OPEN labyrinth (the region box cuts its tubes) —
			// clamp with the same box so the block is a closed solid, exactly like
			// `Feature::Gyroid` / the damper acceptance idiom.
			let tree = json!({
				"op": "intersection",
				"a": leaf,
				"b": {"shape": "box", "min": min, "max": max},
			});
			let parsed = implicit::parse_tree(op_id, &tree, input_base)?;
			let b = parsed.node.bounds();
			if !b.is_valid() {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': the lattice block is empty — 'min' must be strictly below 'max' on every axis"),
				));
			}
			let domain = b.pad(3.0 * voxel as f32);
			grid_guard(op_id, "tpms", domain, voxel)?;
			let mut mesh = manifold_dual_contour(&parsed.node, domain, Resolution::VoxelSize(voxel as f32));
			let mut healed = false;
			if !mesh.is_watertight() || check_mesh(&mesh).non_manifold_edges > 0 {
				mesh = make_manifold(&mesh);
				healed = true;
			}
			let report = check_mesh(&mesh);
			if mesh.triangle_count() == 0 || !mesh.is_watertight() || report.non_manifold_edges > 0 {
				return Err(err(
					ErrorKind::InvalidGeometry,
					format!(
						"op '{op_id}': the {kind} lattice did not mesh watertight at voxel {voxel} (triangles={}, watertight={}, non_manifold_edges={}) — refine the voxel (walls need ≥ ~3 voxels) or thicken the sheet/level",
						mesh.triangle_count(),
						mesh.is_watertight(),
						report.non_manifold_edges
					),
				));
			}
			let path = write_mesh_healed(op_id, out_dir, &file, &mesh)?;
			Ok(Outcome {
				value: Some(EnvValue::Mesh(mesh.clone())),
				measures: Some(json!({
					"route": "voxel_implicit",
					"kind": kind,
					"mode": mode.as_deref().unwrap_or("network"),
					"triangles": mesh.triangle_count(),
					"watertight": true,
					"healed": healed,
					"volume": mesh.signed_volume(),
					"voxel": voxel,
				})),
				file: Some(path),
			})
		}

		OpKind::HybridBoolean { input, field, file, bool_op, voxel, out } => {
			// The flagship convergence op: exact B-rep × (implicit field | mesh),
			// exact wherever untouched, honest voxel fallback otherwise — a thin
			// wire over `kernel_model::hybrid_boolean`, which measures the per-face
			// accounting ON THE RESULT (nothing here asserts what the kernel did
			// not verify).
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			if !(voxel.is_finite() && voxel > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': voxel must be a positive voxel size in mm")));
			}
			// Realize the non-B-rep operand. Exactly one of `field` / `file`.
			let (field_node, operand_mesh, operand_label) = match (field, file) {
				(Some(expr), None) => {
					let parsed = implicit::parse_tree(op_id, &expr, input_base)?;
					let b = parsed.node.bounds();
					if !b.is_valid() || !b.min.is_finite() || !b.max.is_finite() {
						return Err(err(
							ErrorKind::InvalidParam,
							format!(
								"op '{op_id}': the 'field' operand is unbounded or empty — clamp it (intersect with a box / give expr_sdf bounds) so it can be meshed"
							),
						));
					}
					implicit::probe_fields(op_id, &parsed.fields, b)?;
					(Some(parsed.node), None, "implicit_field")
				}
				(None, Some(f)) => {
					let (m, fmt) = read_mesh_file(op_id, input_base, out_dir, &f)?;
					if m.triangle_count() == 0 {
						return Err(err(ErrorKind::InvalidGeometry, format!("op '{op_id}': the mesh operand '{f}' ({fmt}) has no triangles")));
					}
					(None, Some(m), "mesh_file")
				}
				_ => {
					return Err(err(
						ErrorKind::InvalidParam,
						format!("op '{op_id}': exactly one of 'field' (implicit CSG tree) or 'file' (mesh path) is required"),
					));
				}
			};
			// The healed fallback lifts BOTH operands onto one voxel lattice — cap
			// the allocation up front like `mesh_carve` / `shell`.
			let (smin, smax) = s.aabb();
			let mut domain = Aabb::new(
				Vec3::new(smin.x as f32, smin.y as f32, smin.z as f32),
				Vec3::new(smax.x as f32, smax.y as f32, smax.z as f32),
			);
			if let Some(node) = &field_node {
				domain = domain.union(node.bounds());
			}
			if let Some(m) = &operand_mesh {
				domain = domain.union(m.aabb());
			}
			grid_guard(op_id, "hybrid_boolean", domain.pad(2.0 * voxel as f32), voxel)?;
			let op = match bool_op {
				BoolOpSpec::Union => BooleanOp::Union,
				BoolOpSpec::Difference => BooleanOp::Difference,
				BoolOpSpec::Intersection => BooleanOp::Intersection,
			};
			let operand = match (&field_node, &operand_mesh) {
				(Some(node), None) => HybridOperand::Node(node),
				(None, Some(m)) => HybridOperand::Mesh(m),
				_ => unreachable!("exactly one operand was selected above"),
			};
			let result = hybrid_boolean(s, operand, op, voxel as f32).map_err(|e| match e {
				HybridError::UnboundedField => err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': the field operand has unbounded extent — intersect it with a finite region node first"),
				),
				HybridError::NotWatertight { detail } => err(
					ErrorKind::InvalidGeometry,
					format!("op '{op_id}': hybrid_boolean produced no watertight result on either route — {detail}; result withheld"),
				),
			})?;
			let (route, healed_reason) = match &result.route {
				HybridRoute::ExactStitch => ("exact_stitch", None),
				HybridRoute::Healed { reason } => ("voxel_healed", Some(reason.clone())),
			};
			// Route-aware write: the healed route reports crossings instead of
			// refusing on them; the exact stitch keeps the strict predicate.
			let path = match &result.route {
				HybridRoute::ExactStitch => write_mesh_auto(op_id, out_dir, &out, &result.mesh)?,
				HybridRoute::Healed { .. } => write_mesh_healed(op_id, out_dir, &out, &result.mesh)?,
			};
			let r = &result.report;
			let mut measures = json!({
				"route": route,
				"operand": operand_label,
				"triangles": result.mesh.triangle_count(),
				"watertight": true,
				"volume": result.mesh.signed_volume(),
				"voxel": voxel,
				// Per-face convergence receipts, measured on the result: every input
				// B-rep face lands in exactly one bucket.
				"brep_faces": r.brep_faces,
				"kept_exact": r.kept_exact,
				"kept_exact_curved": r.kept_exact_curved,
				"retiled": r.retiled,
				"trimmed": r.trimmed,
				"consumed": r.consumed,
				"operand_triangles": r.operand_triangles,
			});
			if let Some(reason) = healed_reason {
				measures["healed_reason"] = json!(reason);
			}
			Ok(Outcome { value: Some(EnvValue::Mesh(result.mesh)), measures: Some(measures), file: Some(path) })
		}

		_ => unreachable!("ops::io: op routed to the wrong family"),
	}
}
