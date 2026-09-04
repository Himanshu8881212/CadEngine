// Copyright (c) LMCAD. Licensed under the MIT License.

//! Path confinement and the mesh in/out plumbing: resolving an agent-supplied
//! path under the sandbox, meshing a solid (exact or voxel-healed), and the
//! watertightness policies the export ops write through.

use std::fs;
use std::path::{Component, Path, PathBuf};

use kernel_brep::math::DVec3;
use kernel_brep::Solid;
use kernel_core::{check_mesh, degenerate_triangle_witnesses, non_manifold_vertex_witnesses, Mesh, MeshReport};
use kernel_model::watertight_mesh;
use serde_json::{json, Value};

use crate::interp::{err, EnvValue, Outcome};
use crate::report::{ErrorKind, OpError};

/// Confine an agent-supplied path to the sandbox `base`: reject absolute paths and any
/// `..` / root / drive-prefix component so a work-order can only reach files UNDER the
/// output (or input) directory. Existing symlink components are also refused: a
/// lexical `base/link/file` check is not confinement when `link -> /outside`.
pub(crate) fn confined_join(op_id: &str, base: &Path, file: &str) -> Result<PathBuf, OpError> {
	// An EMPTY base is the current directory: `Path::parent()` of a bare
	// program filename ("part.json") yields "", and canonicalizing "" fails —
	// which broke every campaign whose Reproducing invokes `run <prog>.json`
	// from inside programs/ (measured on the cleat's own README commands).
	let base = if base.as_os_str().is_empty() { Path::new(".") } else { base };
	let rel = Path::new(file);
	if rel.is_absolute() {
		return Err(err(
			ErrorKind::InvalidParam,
			format!("op '{op_id}': path '{file}' must be relative to the sandbox (absolute paths are not allowed)"),
		));
	}
	for comp in rel.components() {
		match comp {
			Component::Normal(_) | Component::CurDir => {}
			Component::ParentDir => {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': path '{file}' must not contain '..' (it would escape the sandbox)"),
				));
			}
			Component::RootDir | Component::Prefix(_) => {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': path '{file}' must be a plain relative path (no root or drive prefix)"),
				));
			}
		}
	}
	let canonical_base = fs::canonicalize(base)
		.map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot canonicalize sandbox '{}': {e}", base.display())))?;
	let mut current = canonical_base.clone();
	for comp in rel.components() {
		if let Component::Normal(name) = comp {
			current.push(name);
			match fs::symlink_metadata(&current) {
				Ok(meta) if meta.file_type().is_symlink() => {
					return Err(err(
						ErrorKind::InvalidParam,
						format!(
							"op '{op_id}': path '{file}' crosses a symbolic link at '{}' — sandbox symlinks are refused",
							current.display()
						),
					));
				}
				Ok(_) => {}
				Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
				Err(e) => {
					return Err(err(ErrorKind::Io, format!("op '{op_id}': cannot inspect '{}': {e}", current.display())));
				}
			}
		}
	}
	Ok(canonical_base.join(rel))
}

pub(crate) fn resolve_path(op_id: &str, out_dir: &Path, file: &str) -> Result<PathBuf, OpError> {
	fs::create_dir_all(out_dir)
		.map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot create sandbox '{}': {e}", out_dir.display())))?;
	let path = confined_join(op_id, out_dir, file)?;
	if let Some(parent) = path.parent() {
		if !parent.as_os_str().is_empty() {
			std::fs::create_dir_all(parent)
				.map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot create directory '{}': {e}", parent.display())))?;
		}
	}
	Ok(path)
}

/// Join `file` onto the input base directory for READING — the input twin of
/// [`resolve_path`], without creating directories. Confined exactly like output
/// paths: absolute paths and any `..` component are refused (audit V1), so a
/// program can only read files UNDER its input base.
pub(crate) fn resolve_input_path(op_id: &str, input_base: &Path, file: &str) -> Result<PathBuf, OpError> {
	confined_join(op_id, input_base, file)
}

/// [`resolve_input_path`] with the write-side fallback that heals the T4
/// path-root asymmetry (campaign friction, 7/10 campaigns): a program that
/// `export_step`s a file (which lands under `--out-dir`) and then imports it
/// back used to fail with `io` unless `--out-dir` happened to BE the program's
/// directory — writes resolved against one root, reads against another.
/// Resolution order, both roots confined: the program's own directory first
/// (relocatable program-relative inputs keep priority), then `--out-dir` iff
/// the file exists there and not beside the program. The error on a total miss
/// names BOTH tried roots so the operator sees where the engine looked.
pub(crate) fn resolve_input_or_out(op_id: &str, input_base: &Path, out_dir: &Path, file: &str) -> Result<PathBuf, OpError> {
	let primary = confined_join(op_id, input_base, file)?;
	if primary.exists() || input_base == out_dir {
		return Ok(primary);
	}
	let fallback = confined_join(op_id, out_dir, file)?;
	if fallback.exists() {
		return Ok(fallback);
	}
	Err(err(
		ErrorKind::Io,
		format!(
			"op '{op_id}': cannot read '{file}': not found beside the program ('{}') nor under --out-dir ('{}')",
			primary.display(),
			fallback.display()
		),
	))
}

/// A manufacturing export must bound one unambiguous solid volume: closed,
/// consistently oriented, free of bow-tie vertices, collapsed triangles, and
/// non-adjacent triangle contacts/overlaps.
pub(crate) fn manufacturing_ready(mesh: &Mesh, report: &MeshReport) -> bool {
	report.watertight && report.degenerate_triangles == 0 && mesh.self_intersection_witness().is_none()
}

/// Mesh a solid on the exact adaptive path only when the resulting triangles are
/// manufacturing-ready; otherwise use the voxel heal. Returns the mesh, the
/// route taken (`"exact"` / `"voxel_healed"`), and the heal voxel actually used
/// (= the requested voxel unless the heal budget coarsened it; meaningful only
/// on the healed route).
pub(crate) fn solid_mesh(solid: &Solid, tol: f64, voxel: f64) -> (Mesh, &'static str, f64) {
	let (mesh, route, heal_voxel, _) = solid_mesh_routed(solid, tol, voxel);
	(mesh, route, heal_voxel)
}

/// [`solid_mesh`] plus the DEMOTION receipt: `Some(demotion)` exactly when the
/// exact route was abandoned, naming the defect that abandoned it and where it
/// is (see [`exact_route_demotion`]). A bare `route: "voxel_healed"` sent
/// campaign authors bisecting geometry for a day to find the leaking edge
/// (friction l12_mini_case F3, uphill_roller F2, 2026-09); the receipt now
/// points at it.
pub(crate) fn solid_mesh_routed(solid: &Solid, tol: f64, voxel: f64) -> (Mesh, &'static str, f64, Option<Value>) {
	let exact = kernel_brep::tessellate_adaptive_tol(solid, tol);
	match exact_route_demotion(&exact) {
		None => (exact, "exact", voxel, None),
		Some(demotion) => {
			let heal_voxel = heal_voxel_for_budget(solid, voxel);
			(watertight_mesh(solid, heal_voxel as f32), "voxel_healed", heal_voxel, Some(demotion))
		}
	}
}

/// The exact route's verdict on its tessellation: `None` when the mesh is
/// manufacturing-ready — the SAME predicate as [`manufacturing_ready`],
/// evaluated in the same order (the self-intersection sweep runs only once the
/// topology is clean) — else the demotion receipt:
///
/// ```json
/// {"reason": "non_orientable_edges", "boundary_edges": 0, "non_manifold_edges": 0,
///  "non_orientable_edges": 3, "non_manifold_vertices": 0, "degenerate_triangles": 0,
///  "self_intersections": null, "exact_triangles": 1234, "witness": [[x, y, z], …]}
/// ```
///
/// `reason` is the first defect in the order boundary edges → non-manifold
/// edges → non-orientable edges → non-manifold vertices → degenerate triangles
/// → self-intersection (`tessellation_failed` for an empty tessellation), and
/// `witness` locates up to 8 of THAT defect in the body's own frame: edge
/// midpoints, vertex positions, degenerate-triangle centroids, or the pierce
/// point plus the two crossing triangles' centroids. `self_intersections` is
/// `null` when the sweep never ran because the topology already demoted.
pub(crate) fn exact_route_demotion(exact: &Mesh) -> Option<Value> {
	let report = check_mesh(exact);
	let topology_ok = report.watertight && report.degenerate_triangles == 0;
	let crossing = if topology_ok { exact.self_intersection_witness() } else { None };
	if topology_ok && crossing.is_none() {
		return None;
	}
	let centroid = |t: usize| -> [f64; 3] {
		let idx = &exact.indices[3 * t..3 * t + 3];
		let c = idx.iter().fold(DVec3::ZERO, |acc, &i| acc + exact.positions[i as usize].as_dvec3()) / 3.0;
		[c.x, c.y, c.z]
	};
	let (reason, witness): (&str, Vec<[f64; 3]>) = if exact.triangle_count() == 0 {
		("tessellation_failed", Vec::new())
	} else if report.boundary_edges > 0 {
		("boundary_edges", exact.boundary_edge_witnesses(8))
	} else if report.non_manifold_edges > 0 {
		("non_manifold_edges", exact.non_manifold_edge_witnesses(8))
	} else if report.non_orientable_edges > 0 {
		("non_orientable_edges", exact.non_orientable_edge_witnesses(8))
	} else if report.non_manifold_vertices > 0 {
		("non_manifold_vertices", non_manifold_vertex_witnesses(exact, 8))
	} else if report.degenerate_triangles > 0 {
		("degenerate_triangles", degenerate_triangle_witnesses(exact, 8))
	} else if let Some(w) = crossing {
		let p = w.point.as_dvec3();
		("self_intersection", vec![[p.x, p.y, p.z], centroid(w.triangles[0]), centroid(w.triangles[1])])
	} else {
		("tessellation_failed", Vec::new())
	};
	Some(json!({
		"reason": reason,
		"boundary_edges": report.boundary_edges,
		"non_manifold_edges": report.non_manifold_edges,
		"non_orientable_edges": report.non_orientable_edges,
		"non_manifold_vertices": report.non_manifold_vertices,
		"degenerate_triangles": report.degenerate_triangles,
		"self_intersections": if topology_ok { json!(crossing.map_or(0, |w| w.pairs)) } else { Value::Null },
		"exact_triangles": exact.triangle_count(),
		"witness": witness,
	}))
}

/// The heal voxel that keeps the winding-number lattice inside the heal's
/// TIME budget. The mesher's own [`kernel_core::mesher::MAX_LATTICE_CELLS`]
/// (2²⁸) is a MEMORY bound: a heal well under it — a 160×140×20 mm body at
/// voxel 0.3 is ~19M cells — still costs one winding-number SDF traversal per
/// cell and ground for many minutes with no feedback, indistinguishable from a
/// hang (friction folding_book_stand F4, 2026-08-27). 2²² cells (~4M) keeps
/// the worst heal around a minute; the receipt reports the voxel used, so the
/// coarsening is on the record, never silent.
pub(crate) fn heal_voxel_for_budget(solid: &Solid, voxel: f64) -> f64 {
	const HEAL_CELL_BUDGET: f64 = (1u64 << 22) as f64;
	let Some(b) = kernel_brep::measure::bounding_box(solid) else {
		return voxel;
	};
	let s = b.max - b.min;
	// pad + margins mirrored from the mesher's lattice sizing
	let cells = |vs: f64| {
		let g = |d: f64| (d + 4.0 * vs) / vs + 3.0;
		g(s.x) * g(s.y) * g(s.z)
	};
	if cells(voxel) <= HEAL_CELL_BUDGET {
		return voxel;
	}
	let mut vs = voxel;
	while cells(vs) > HEAL_CELL_BUDGET {
		vs *= 1.05;
	}
	(vs * 100.0).ceil() / 100.0
}

/// Mesh + watertightness gate + write for the STL/3MF export ops.
pub(crate) fn export_mesh(
	op_id: &str,
	solid: &Solid,
	tol: f64,
	voxel: f64,
	out_dir: &Path,
	file: &str,
	format: &'static str,
) -> Result<Outcome, OpError> {
	if !(tol.is_finite() && tol > 0.0) {
		return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': tol must be a positive chord tolerance in mm")));
	}
	if !(voxel.is_finite() && voxel > 0.0) {
		return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': voxel must be a positive voxel size in mm")));
	}
	let (mut mesh, route, heal_voxel, demotion) = solid_mesh_routed(solid, tol, voxel);
	// An EMPTY healed mesh is the dual-contour mesher's only refusal channel —
	// it means the heal never ran (its lattice would blow the cell budget), not
	// that the geometry healed to nothing. Letting it fall through to the
	// counter-based refusal below produces the worst message in the engine:
	// "not manufacturing-ready: boundary_edges=0, …, self_intersections=0" with
	// every counter zero. Name the real cause and the fix instead.
	if route == "voxel_healed" && mesh.triangle_count() == 0 {
		let budget = kernel_core::mesher::MAX_LATTICE_CELLS;
		let (extent, vmin) = match kernel_brep::measure::bounding_box(solid) {
			Some(b) => {
				let s = b.max - b.min;
				// Smallest voxel whose heal lattice fits the budget for this
				// part's extent (pad + 3-point margins mirrored from the
				// mesher), with 5% headroom, coarsened to 2 decimals.
				let fits = |vs: f64| {
					let g = |d: f64| (d + 4.0 * vs) / vs + 3.0;
					g(s.x) * g(s.y) * g(s.z) <= budget
				};
				let mut vs = (s.x * s.y * s.z / budget).cbrt() * 1.05;
				while !fits(vs) {
					vs *= 1.05;
				}
				(format!("{:.0}×{:.0}×{:.0} mm", s.x, s.y, s.z), (vs * 100.0).ceil() / 100.0)
			}
			None => ("unbounded".into(), voxel),
		};
		return Err(err(
			ErrorKind::InvalidGeometry,
			format!(
				"op '{op_id}': the exact tessellation at tol {tol} mm is not manufacturing-ready, and the voxel heal cannot run at voxel {voxel} mm — this part's {extent} bounds need a lattice over the mesher's {budget:.0}-cell budget, so the heal returns nothing. Re-export with voxel ≥ {vmin} mm, or at a tol where the exact route is manufacturing-ready",
			),
		));
	}
	// The implicit mesher can emit near-coincident vertices on neighbouring cells.
	// Normalize the healed mesh with the same weld used by STL round-trip import so
	// the in-memory gate checks the topology that downstream readers reconstruct.
	if route == "voxel_healed" {
		mesh.weld(1e-4);
		mesh.compute_normals();
	}
	let mesh_report = check_mesh(&mesh);
	let proper_self_intersections = mesh.self_intersection_witness().map_or(0, |witness| witness.pairs);
	// Route-aware refusal. The EXACT route promises an arrangement-exact solid,
	// so any self-intersection there is a lie worth refusing. The VOXEL-HEALED
	// route promises voxel-accurate closure only — dual-contoured TPMS/lattice
	// output legitimately carries crossing slivers that slicers resolve by
	// covered volume, so crossings REPORT (see `self_intersections` +
	// `manufacturing_ready` in the receipt, both `require`-gateable) while true
	// breakage (open edges, non-manifold, degenerate triangles) still refuses.
	let route_ready = if route == "voxel_healed" {
		mesh_report.watertight && mesh_report.degenerate_triangles == 0
	} else {
		manufacturing_ready(&mesh, &mesh_report)
	};
	if !route_ready {
		return Err(err(
			ErrorKind::InvalidGeometry,
			format!(
				"op '{op_id}': mesh is not manufacturing-ready even after the voxel heal (voxel {voxel} mm): boundary_edges={}, non_manifold_edges={}, non_orientable_edges={}, non_manifold_vertices={}, degenerate_triangles={}, self_intersections={} — refusing export",
				mesh_report.boundary_edges,
				mesh_report.non_manifold_edges,
				mesh_report.non_orientable_edges,
				mesh_report.non_manifold_vertices,
				mesh_report.degenerate_triangles,
				proper_self_intersections,
			),
		));
	}
	let path = resolve_path(op_id, out_dir, file)?;
	let write_result = match format {
		"stl" => mesh.write_stl_binary(&path),
		_ => mesh.write_3mf(&path),
	};
	write_result.map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot write '{}': {e}", path.display())))?;
	// Gate the serialized artifact, not merely the in-memory source mesh. STL is
	// a triangle soup, so reconstruct shared topology with the kernel's standard
	// import weld before applying the same strict manufacturing predicate.
	let mut round_trip = match format {
		"stl" => Mesh::read_stl(&path),
		_ => Mesh::read_3mf(&path),
	}
	.map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot read back '{}': {e}", path.display())))?;
	if format == "stl" {
		round_trip.weld(1e-4);
		round_trip.compute_normals();
	}
	let round_trip_report = check_mesh(&round_trip);
	let round_trip_crossings = round_trip.self_intersection_witness().map_or(0, |witness| witness.pairs);
	let round_trip_ready = if route == "voxel_healed" {
		round_trip_report.watertight && round_trip_report.degenerate_triangles == 0
	} else {
		manufacturing_ready(&round_trip, &round_trip_report)
	};
	if !round_trip_ready {
		let _ = std::fs::remove_file(&path);
		return Err(err(
			ErrorKind::InvalidGeometry,
			format!(
				"op '{op_id}': serialized {format} failed strict round-trip validation: boundary_edges={}, non_manifold_edges={}, non_orientable_edges={}, non_manifold_vertices={}, degenerate_triangles={}, self_intersections={} — artifact removed",
				round_trip_report.boundary_edges,
				round_trip_report.non_manifold_edges,
				round_trip_report.non_orientable_edges,
				round_trip_report.non_manifold_vertices,
				round_trip_report.degenerate_triangles,
				round_trip_crossings,
			),
		));
	}
	// Bind and report the exact mesh that was written. `watertight` uses the
	// strict closed-orientable-2-manifold definition. `manufacturing_ready` is
	// the FULL predicate (incl. zero self-intersections) — on the healed route
	// it can honestly read false while the export still ships, and a campaign
	// that needs the strict bar gates it with `require {manufacturing_ready: true}`.
	let mut measures = json!({
		"route": route,
		"heal_voxel_mm": if route == "voxel_healed" { json!(heal_voxel) } else { json!(null) },
		"triangles": round_trip.triangle_count(),
		"manufacturing_ready": manufacturing_ready(&round_trip, &round_trip_report),
		"round_trip_validated": true,
		"watertight": round_trip_report.watertight,
		"watertight_means": "closed, consistently oriented 2-manifold: no boundary, non-manifold, or non-orientable edges and no non-manifold vertices",
		"boundary_edges": round_trip_report.boundary_edges,
		"non_manifold_edges": round_trip_report.non_manifold_edges,
		"non_orientable_edges": round_trip_report.non_orientable_edges,
		"non_manifold_vertices": round_trip_report.non_manifold_vertices,
		"degenerate_triangles": round_trip_report.degenerate_triangles,
		"self_intersections": round_trip_crossings,
		"contacts_or_coplanar_overlaps": round_trip_report.self_intersections,
		"two_manifold": round_trip_report.watertight,
	});
	// Only on the healed route: WHY the exact route was abandoned, and where.
	// An exact export carries no `demotion` field at all.
	if let Some(demotion) = demotion {
		measures["demotion"] = demotion;
	}
	Ok(Outcome { value: Some(EnvValue::Mesh(round_trip.clone())), measures: Some(measures), file: Some(path.display().to_string()) })
}

/// Resolve `file` under `out_dir`, enforce the manufacturing mesh contract,
/// write `.stl` / `.3mf`, then re-read and validate the bytes actually written.
/// Invalid files are removed rather than left behind as plausible artifacts.
pub(crate) fn write_mesh_auto(op_id: &str, out_dir: &Path, file: &str, mesh: &Mesh) -> Result<String, OpError> {
	write_mesh_policy(op_id, out_dir, file, mesh, MeshWritePolicy::Strict)
}

/// [`write_mesh_auto`] for a VOXEL-HEALED result: closure and non-degeneracy
/// still refuse, but proper self-intersections REPORT instead of refusing —
/// dual-contoured TPMS/lattice output legitimately carries crossing slivers
/// that slicers resolve by covered volume, and the receipt carries the count
/// for `require` gating. The exact route keeps the full strict predicate.
pub(crate) fn write_mesh_healed(op_id: &str, out_dir: &Path, file: &str, mesh: &Mesh) -> Result<String, OpError> {
	write_mesh_policy(op_id, out_dir, file, mesh, MeshWritePolicy::Healed)
}

/// Refusal policy for [`write_mesh_policy`], per the writing op's route contract.
#[derive(Clone, Copy, PartialEq)]
pub(crate) enum MeshWritePolicy {
	/// Arrangement-exact print file: the full manufacturing predicate refuses.
	Strict,
	/// Voxel-accurate print file: breakage refuses, crossings report.
	Healed,
	/// Diagnostic scene: IO validated only; quality counters are the caller's receipt.
	Scene,
}

/// [`write_mesh_auto`] for a DIAGNOSTIC SCENE: a merged multi-instance pose
/// snapshot, not a print file. A negative-control scene is DESIGNED to
/// interpenetrate (`overlap_volume > 0` is the whole claim), so refusing it on
/// `proper_self_intersections` would make every failure-attitude export fail
/// the run (campaign friction: SLAS F9). Scene writes skip the
/// manufacturing-readiness refusals; IO and read-back are still validated, and
/// the caller must report the quality counters so the exemption is on the
/// record. Per-instance part files stay on the strict path — only the merged
/// soup is a scene.
pub(crate) fn write_mesh_scene(op_id: &str, out_dir: &Path, file: &str, mesh: &Mesh) -> Result<String, OpError> {
	write_mesh_policy(op_id, out_dir, file, mesh, MeshWritePolicy::Scene)
}

pub(crate) fn write_mesh_policy(op_id: &str, out_dir: &Path, file: &str, mesh: &Mesh, policy: MeshWritePolicy) -> Result<String, OpError> {
	let ready = |m: &Mesh, r: &MeshReport| -> bool {
		match policy {
			MeshWritePolicy::Strict => manufacturing_ready(m, r),
			// Healed = voxel-accurate closure: every EDGE closed, consistently
			// oriented, no degenerate triangles. Non-manifold VERTICES (pinch
			// points at TPMS saddle tangencies) and crossing slivers are
			// characteristic dual-contoured output that slicers resolve by
			// covered volume — they REPORT in the receipt instead of refusing.
			MeshWritePolicy::Healed => {
				r.boundary_edges == 0 && r.non_manifold_edges == 0 && r.non_orientable_edges == 0 && r.degenerate_triangles == 0
			}
			MeshWritePolicy::Scene => true,
		}
	};
	let path = resolve_path(op_id, out_dir, file)?;
	let format = match path.extension().and_then(|e| e.to_str()) {
		Some("stl") => "stl",
		Some("3mf") => "3mf",
		other => {
			return Err(err(
				ErrorKind::InvalidParam,
				format!("op '{op_id}': the output file must end in .stl or .3mf, got extension {other:?}"),
			));
		}
	};
	let mut output_mesh = mesh.clone();
	let mut report = check_mesh(&output_mesh);
	if !ready(&output_mesh, &report) {
		// Imported STL soups and grid meshing can carry near-coincident, unshared
		// vertices. Normalize once before refusing; geometry is not otherwise healed.
		output_mesh.weld(1e-4);
		output_mesh.compute_normals();
		report = check_mesh(&output_mesh);
	}
	if !ready(&output_mesh, &report) {
		return Err(err(
			ErrorKind::InvalidGeometry,
			format!(
				"op '{op_id}': refusing manufacturing output: boundary_edges={}, non_manifold_edges={}, non_orientable_edges={}, non_manifold_vertices={}, degenerate_triangles={}, proper_self_intersections={}",
				report.boundary_edges, report.non_manifold_edges,
				report.non_orientable_edges, report.non_manifold_vertices,
				report.degenerate_triangles,
				output_mesh.self_intersection_witness().as_ref().map_or(0, |w| w.pairs)
			),
		));
	}
	let write_result = if format == "stl" { output_mesh.write_stl_binary(&path) } else { output_mesh.write_3mf(&path) };
	write_result.map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot write '{}': {e}", path.display())))?;
	let read_result = if format == "stl" { Mesh::read_stl(&path) } else { Mesh::read_3mf(&path) };
	let mut round_trip = match read_result {
		Ok(mesh) => mesh,
		Err(e) => {
			let _ = fs::remove_file(&path);
			return Err(err(ErrorKind::Io, format!("op '{op_id}': cannot read back '{}': {e}", path.display())));
		}
	};
	round_trip.weld(1e-4);
	round_trip.compute_normals();
	let round_trip_report = check_mesh(&round_trip);
	if !ready(&round_trip, &round_trip_report) {
		let _ = fs::remove_file(&path);
		return Err(err(
			ErrorKind::InvalidGeometry,
			format!(
				"op '{op_id}': serialized manufacturing mesh failed round-trip validation (policy {}): boundary_edges={}, non_manifold_edges={}, non_orientable_edges={}, non_manifold_vertices={}, degenerate_triangles={} — partial artifact removed",
				match policy { MeshWritePolicy::Strict => "strict", MeshWritePolicy::Healed => "healed", MeshWritePolicy::Scene => "scene" },
				round_trip_report.boundary_edges,
				round_trip_report.non_manifold_edges,
				round_trip_report.non_orientable_edges,
				round_trip_report.non_manifold_vertices,
				round_trip_report.degenerate_triangles,
			),
		));
	}
	Ok(path.display().to_string())
}

/// Read a mesh interchange file — `.stl` / `.obj` / `.3mf` / `.ply`, sniffed by
/// extension (the kernel has NO glTF reader) — and ALWAYS weld it (STL and many
/// exporters store an unshared triangle soup; welding recovers shared topology
/// so the `check_mesh` receipt is meaningful). Returns the welded mesh plus the
/// sniffed format name. An unreadable file is `io`; an empty one `invalid_param`.
pub(crate) fn read_mesh_file(op_id: &str, input_base: &Path, out_dir: &Path, file: &str) -> Result<(Mesh, &'static str), OpError> {
	// T4: program-relative first, then --out-dir (a mesh written by an earlier op lands there).
	let path = resolve_input_or_out(op_id, input_base, out_dir, file)?;
	let format = match path.extension().and_then(|e| e.to_str()).map(str::to_ascii_lowercase).as_deref() {
		Some("stl") => "stl",
		Some("obj") => "obj",
		Some("3mf") => "3mf",
		Some("ply") => "ply",
		other => {
			return Err(err(
				ErrorKind::InvalidParam,
				format!("op '{op_id}': mesh file '{file}' has unsupported extension {other:?} — supported: .stl, .obj, .3mf, .ply (the kernel has no glTF reader)"),
			));
		}
	};
	let mut mesh = match format {
		"stl" => Mesh::read_stl(&path),
		"obj" => Mesh::read_obj(&path),
		"3mf" => Mesh::read_3mf(&path),
		_ => Mesh::read_ply(&path),
	}
	.map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot read '{}': {e}", path.display())))?;
	if mesh.triangle_count() == 0 {
		return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': '{file}' contains no triangles")));
	}
	mesh.weld(1e-4); // the kernel's STL-soup weld tolerance (kernel-core convention)
	Ok((mesh, format))
}

/// Append `src`'s triangles onto `dst` as a plain soup extension (no weld — the
/// winding-number heal consumes a soup directly).
pub(crate) fn merge_soup(dst: &mut Mesh, src: &Mesh) {
	let base = dst.positions.len() as u32;
	dst.positions.extend_from_slice(&src.positions);
	for t in src.triangles() {
		dst.push_triangle(base + t[0], base + t[1], base + t[2]);
	}
}

/// The full [`check_mesh`] receipt as report measures — every count, never a
/// summary, so a caller sees exactly what is (and is not) wrong with a mesh.
pub(crate) fn mesh_receipt(m: &mut serde_json::Map<String, Value>, report: &MeshReport) {
	m.insert("watertight".into(), json!(report.watertight));
	m.insert("boundary_edges".into(), json!(report.boundary_edges));
	m.insert("non_manifold_edges".into(), json!(report.non_manifold_edges));
	m.insert("non_orientable_edges".into(), json!(report.non_orientable_edges));
	m.insert("non_manifold_vertices".into(), json!(report.non_manifold_vertices));
	m.insert("degenerate_triangles".into(), json!(report.degenerate_triangles));
	m.insert("self_intersections".into(), json!(report.self_intersections));
}
