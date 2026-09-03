// Copyright (c) LMCAD. Licensed under the MIT License.

//! The implicit/voxel half: the `implicit` expression tree, the `gyroid_block`
//! lattice, the density-grid ops, the voxel `shell`, and the reverse-bridge solid
//! ops and interrogation probes (`offset_solid`, `shell_solid`,
//! `solid_from_implicit`, `thin_wall`, `min_ligament`).

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

use kernel_brep::holes;
use kernel_brep::Solid;
use kernel_core::math::Vec3;
use kernel_core::{check_mesh, make_manifold, Aabb, Resolution, Sdf};
#[cfg(feature = "catalog")]
use kernel_implicit::{Cuboid as ImplicitCuboid, Gyroid};
use kernel_implicit::{dual_contour_narrowband, manifold_dual_contour, MeshSdf, Node};
use serde_json::{json, Value};

use crate::implicit;
use crate::interp::{err, fetch_solid, EnvValue, Outcome, MAX_GRID_CELLS};
use crate::program::{MesherSpec, OpKind};
use crate::report::{ErrorKind, OpError};

use super::meshio::{resolve_input_or_out, resolve_path};
use super::support::{bind_solid, dv3, grid_guard};

/// Validate an op's optional explicit `domain` box (`{min, max}`), or `None`
/// when the caller should fall back to the geometry's own bounds.
pub(crate) fn explicit_domain(op_id: &str, domain: &Option<crate::program::DomainSpec>) -> Result<Option<Aabb>, OpError> {
	match domain {
		Some(d) => {
			let lo = Vec3::new(d.min[0] as f32, d.min[1] as f32, d.min[2] as f32);
			let hi = Vec3::new(d.max[0] as f32, d.max[1] as f32, d.max[2] as f32);
			if !(lo.is_finite() && hi.is_finite() && lo.x < hi.x && lo.y < hi.y && lo.z < hi.z) {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': 'domain.min' must be finite and strictly below 'domain.max' on every axis"),
				));
			}
			Ok(Some(Aabb::new(lo, hi)))
		}
		None => Ok(None),
	}
}

/// The finite bounds of a parsed implicit tree, with the same refusal guidance
/// as the `implicit` op: an empty tree (disjoint intersection) and an unbounded
/// one (bare plane, periodic lattice without a shroud, bounds-less `expr_sdf`)
/// are loud `invalid_param`s, never a silent empty/endless lattice.
pub(crate) fn tree_bounds(op_id: &str, node: &Node) -> Result<Aabb, OpError> {
	let b = node.bounds();
	if !b.is_valid() {
		return Err(err(
			ErrorKind::InvalidParam,
			format!("op '{op_id}': the expression tree has empty bounds (e.g. an intersection of disjoint shapes) — nothing to mesh/measure"),
		));
	}
	if !(b.min.is_finite() && b.max.is_finite()) {
		return Err(err(
			ErrorKind::InvalidParam,
			format!("op '{op_id}': the expression tree is unbounded (a bare 'plane', a periodic 'strut_lattice'/'tpms' without a shroud, or a bounds-less 'expr_sdf' leaf) — intersect it with a bounded shape or pass an explicit 'domain'"),
		));
	}
	Ok(b)
}

/// The bind-side receipt of a **voxel-route solid** op (`offset_solid` /
/// `shell_solid` / `solid_from_implicit`): honest route `"voxel"` (the body
/// re-entered the solid environment through a voxel lattice — a FACETED B-rep,
/// accurate to ~`voxel`, never exact), the achieved volume, face count, and the
/// full validity verdict. Compute BEFORE `bind_solid` consumes the solid.
pub(crate) fn voxel_solid_measures(solid: &Solid, voxel: f64) -> Value {
	let v = kernel_brep::validate(solid);
	json!({
		"route": "voxel",
		"faceted": true,
		"voxel": voxel,
		"faces": solid.face_count(),
		"volume": kernel_brep::volume(solid),
		"closed": v.closed,
		"manifold": v.manifold,
		"shells": v.shells,
		"genus": v.genus,
	})
}

/// Execute one op of this family. The dispatch table in [`crate::interp`]
/// routes exactly the variants matched below, so the catch-all is dead code
/// kept only to satisfy the compiler.
pub(crate) fn exec(
	op_id: &str,
	env: &mut BTreeMap<String, EnvValue>,
	all_ids: &BTreeSet<String>,
	out_dir: &Path,
	input_base: &Path,
	kind: OpKind,
) -> Result<Outcome, OpError> {
	match kind {
		#[cfg(feature = "catalog")]
		OpKind::GyroidBlock { center, half, scale, thickness, voxel, file } => {
			for (name, value) in [("half", half), ("scale", scale), ("thickness", thickness), ("voxel", voxel)] {
				if !(value.is_finite() && value > 0.0) {
					return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': {name} must be a positive number")));
				}
			}
			let c = Vec3::new(center[0] as f32, center[1] as f32, center[2] as f32);
			let region = Aabb::from_center_half_extent(c, Vec3::splat(half as f32));
			let lattice = Node::primitive(Gyroid::new(region, scale as f32, thickness as f32))
				.intersection(Node::primitive(ImplicitCuboid::new(c, Vec3::splat(half as f32))));
			let domain = region.pad(3.0 * voxel as f32);
			let mut mesh = manifold_dual_contour(&lattice, domain, Resolution::VoxelSize(voxel as f32));
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
						"op '{op_id}': gyroid lattice did not mesh watertight at voxel {voxel} (triangles={}, watertight={}, non_manifold_edges={}) — try a smaller voxel or a thicker wall",
						mesh.triangle_count(),
						mesh.is_watertight(),
						report.non_manifold_edges
					),
				));
			}
			let path = resolve_path(op_id, out_dir, &file)?;
			mesh.write_stl_binary(&path)
				.map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot write '{}': {e}", path.display())))?;
			let measures = json!({
				"triangles": mesh.triangle_count(),
				"watertight": true,
				"healed": healed,
			});
			Ok(Outcome { value: Some(EnvValue::Mesh(mesh.clone())), measures: Some(measures), file: Some(path.display().to_string()) })
		}

		OpKind::SampleDensityGrid { input, expr, origin, voxel, shape, supersample, file } => {
			if !(voxel.is_finite() && voxel > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': voxel must be a positive voxel size in mm")));
			}
			if shape.iter().any(|&n| n == 0 || n > 2048) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': shape axes must be 1..=2048, got {shape:?}")));
			}
			if supersample == 0 || supersample > 4 {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': supersample must be 1..=4")));
			}
			let o = Vec3::new(origin[0] as f32, origin[1] as f32, origin[2] as f32);
			let h = voxel as f32;
			let rho = match (&input, &expr) {
				(Some(id), None) => {
					let solid = fetch_solid(env, all_ids, op_id, "in", id)?;
					let mesh = kernel_brep::tessellate_default(solid);
					let sdf = crate::bridge::mesh_sdf(&mesh);
					crate::bridge::sample_density(&sdf, o, h, shape, supersample)
				}
				(None, Some(tree)) => {
					let parsed = implicit::parse_tree(op_id, tree, input_base)?;
					crate::bridge::sample_density(&parsed.node, o, h, shape, supersample)
				}
				_ => {
					return Err(err(
						ErrorKind::InvalidParam,
						format!("op '{op_id}': give exactly one of 'in' (a solid id) or 'expr' (an implicit tree)"),
					));
				}
			};
			let mean: f32 = rho.iter().sum::<f32>() / rho.len() as f32;
			let bytes = crate::bridge::write_npy_f32(&shape, &rho);
			let path = resolve_path(op_id, out_dir, &file)?;
			std::fs::write(&path, &bytes).map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot write '{}': {e}", path.display())))?;
			Ok(Outcome {
				value: None,
				measures: Some(json!({
					"voxels": shape[0] * shape[1] * shape[2],
					"shape": shape,
					"solid_fraction_mean": mean,
					"bytes": bytes.len(),
				})),
				file: Some(path.display().to_string()),
			})
		}
		OpKind::MeshDensityGrid { npy, origin, voxel, iso, file } => {
			if !(voxel.is_finite() && voxel > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': voxel must be a positive voxel size in mm")));
			}
			let path_in = resolve_input_or_out(op_id, input_base, out_dir, &npy)?;
			let bytes = std::fs::read(&path_in).map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot read '{}': {e}", path_in.display())))?;
			let (nshape, rho) = crate::bridge::read_npy_f32(&bytes)
				.map_err(|e| err(ErrorKind::InvalidParam, format!("op '{op_id}': '{npy}': {e}")))?;
			if nshape.len() != 3 {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': '{npy}' must be a 3-D array, got shape {nshape:?}")));
			}
			let dims = [nshape[0], nshape[1], nshape[2]];
			let o = Vec3::new(origin[0] as f32, origin[1] as f32, origin[2] as f32);
			let grid = crate::bridge::density_to_grid(dims, &rho, o, voxel as f32, iso as f32);
			let domain = grid.lattice_bounds();
			let mut mesh = dual_contour_narrowband(&grid, domain, Resolution::VoxelSize(voxel as f32));
			let mut healed = false;
			if !mesh.is_watertight() || check_mesh(&mesh).non_manifold_edges > 0 {
				mesh = make_manifold(&mesh);
				healed = true;
			}
			if mesh.triangle_count() == 0 || !mesh.is_watertight() || check_mesh(&mesh).non_manifold_edges > 0 {
				return Err(err(
					ErrorKind::InvalidGeometry,
					format!(
						"op '{op_id}': the density level-set did not mesh watertight at voxel {voxel} (triangles={}, watertight={}) — refine the grid, check iso, or inspect disconnected debris in the density field",
						mesh.triangle_count(),
						mesh.is_watertight()
					),
				));
			}
			let volume = mesh.signed_volume();
			let path = resolve_path(op_id, out_dir, &file)?;
			let write_result = match path.extension().and_then(|e| e.to_str()) {
				Some("stl") => mesh.write_stl_binary(&path),
				Some("3mf") => mesh.write_3mf(&path),
				other => {
					return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': 'file' must end in .stl or .3mf, got extension {other:?}")));
				}
			};
			write_result.map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot write '{}': {e}", path.display())))?;
			Ok(Outcome {
				value: Some(EnvValue::Mesh(mesh.clone())),
				measures: Some(json!({
					"ok": true,
					"volume_mm3": volume,
					"num_triangles": mesh.triangle_count(),
					"watertight": true,
					"healed": healed,
				})),
				file: Some(path.display().to_string()),
			})
		}
		OpKind::Implicit { expr, voxel, mesher, domain, file } => {
			if !(voxel.is_finite() && voxel > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': voxel must be a positive voxel size in mm")));
			}
			let parsed = implicit::parse_tree(op_id, &expr, input_base)?;
			// Field-quality honesty (Unit 4a): offset/shell/offset_by on a DistanceBound
			// field is only APPROXIMATE. Surface it in the op measures so it can never
			// pass unnoticed — meshing is where the approximation becomes a real solid.
			let approximate_offset = parsed.node.has_approximate_offset();
			let domain_box = match domain {
				Some(d) => {
					let lo = Vec3::new(d.min[0] as f32, d.min[1] as f32, d.min[2] as f32);
					let hi = Vec3::new(d.max[0] as f32, d.max[1] as f32, d.max[2] as f32);
					if !(lo.is_finite() && hi.is_finite() && lo.x < hi.x && lo.y < hi.y && lo.z < hi.z) {
						return Err(err(
							ErrorKind::InvalidParam,
							format!("op '{op_id}': 'domain.min' must be finite and strictly below 'domain.max' on every axis"),
						));
					}
					Aabb::new(lo, hi)
				}
				None => {
					let b = parsed.node.bounds();
					if !b.is_valid() {
						return Err(err(
							ErrorKind::InvalidParam,
							format!("op '{op_id}': the expression tree has empty bounds (e.g. an intersection of disjoint shapes) — nothing to mesh"),
						));
					}
					if !(b.min.is_finite() && b.max.is_finite()) {
						return Err(err(
							ErrorKind::InvalidParam,
							format!("op '{op_id}': the expression tree is unbounded (a bare 'plane' or a bounds-less 'expr_sdf' leaf) — intersect it with a bounded shape, give the expr_sdf leaf min/max bounds, or pass an explicit 'domain'"),
						));
					}
					b.pad(3.0 * voxel as f32)
				}
			};
			implicit::probe_fields(op_id, &parsed.fields, domain_box)?;
			let mut mesh = match mesher {
				MesherSpec::Narrowband => {
					// The narrow-band extractor prunes by the Lipschitz contract, so an
					// under-declared expr_sdf bound would silently tear holes — verify the
					// declarations against the sampled field first (dense meshers skip this).
					implicit::probe_lipschitz(op_id, &parsed.fields, domain_box)?;
					dual_contour_narrowband(&parsed.node, domain_box, Resolution::VoxelSize(voxel as f32))
				}
				MesherSpec::Manifold => manifold_dual_contour(&parsed.node, domain_box, Resolution::VoxelSize(voxel as f32)),
			};
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
						"op '{op_id}': the implicit tree did not mesh watertight at voxel {voxel} (triangles={}, watertight={}, non_manifold_edges={}) — refine the voxel (thin walls need ≥ ~3 voxels), check that the tree is non-empty inside the domain, verify expr_sdf lipschitz_bound declarations, or switch to \"mesher\": \"manifold\" for junction-rich lattices",
						mesh.triangle_count(),
						mesh.is_watertight(),
						report.non_manifold_edges
					),
				));
			}
			let volume = mesh.signed_volume();
			let written = match file {
				Some(file) => {
					let path = resolve_path(op_id, out_dir, &file)?;
					let write_result = match path.extension().and_then(|e| e.to_str()) {
						Some("stl") => mesh.write_stl_binary(&path),
						Some("3mf") => mesh.write_3mf(&path),
						other => {
							return Err(err(
								ErrorKind::InvalidParam,
								format!("op '{op_id}': 'file' must end in .stl or .3mf, got extension {other:?}"),
							));
						}
					};
					write_result.map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot write '{}': {e}", path.display())))?;
					Some(path.display().to_string())
				}
				None => None,
			};
			Ok(Outcome {
				value: Some(EnvValue::Mesh(mesh.clone())),
				measures: Some(json!({
					"triangles": mesh.triangle_count(),
					"watertight": true,
					"healed": healed,
					"volume": volume,
					// Unit 4a: true when offset/shell/offset_by acted on a distance-BOUND field,
					// so the offset distance is only approximate — surfaced in measures, not silent.
					"approximate_offset": approximate_offset,
				})),
				file: written,
			})
		}
		OpKind::Shell { input, wall, voxel, file } => {
			// Voxel-route hollow, reusing the kernel's EXISTING machinery end to end:
			// the same winding-number `MeshSdf` lift as `kernel_model::watertight_mesh`,
			// the same inward-offset difference as `kernel_model::Feature::Shell`
			// (`outer − offset(inner, −wall)`, outer surface preserved), and the same
			// Manifold-Dual-Contour + heal + watertight gate as `gyroid_block`. The
			// result is `voxel_healed` BY CONSTRUCTION — accurate to `voxel`, not exact.
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			if !(wall.is_finite() && wall > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': wall must be a positive thickness in mm")));
			}
			if !(voxel.is_finite() && voxel > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': voxel must be a positive voxel size in mm")));
			}
			// A wall the grid cannot resolve fails DETERMINISTICALLY here, instead of
			// as a mysterious non-watertight mesh three seconds later.
			if wall < 2.0 * voxel {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': wall {wall} mm is under 2 × voxel ({voxel} mm) — the grid cannot resolve it; shrink 'voxel' or thicken the wall"),
				));
			}
			let base = kernel_brep::tessellate_default(s);
			let outer = MeshSdf::new(&base);
			let domain = outer.bounds().pad(2.0 * voxel as f32);
			// Same allocation discipline as the gyroid/density grids: reject a grid
			// beyond the cell cap before allocating it.
			let size = domain.size();
			let cells = (f64::from(size.x) / voxel).ceil() * (f64::from(size.y) / voxel).ceil() * (f64::from(size.z) / voxel).ceil();
			if !(cells.is_finite() && cells <= MAX_GRID_CELLS as f64) {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': shell grid ≈{cells:.0} cells (bbox/voxel)³ exceeds the cap {MAX_GRID_CELLS} — use a coarser voxel"),
				));
			}
			// Built twice (like `Feature::Shell`): the SDF tree owns its leaves.
			let inner = MeshSdf::new(&base);
			let node = Node::primitive(outer).difference(Node::primitive(inner).offset(-wall as f32));
			let mut mesh = manifold_dual_contour(&node, domain, Resolution::VoxelSize(voxel as f32));
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
						"op '{op_id}': the shelled solid did not mesh watertight at voxel {voxel} (triangles={}, watertight={}, non_manifold_edges={}) — walls need ≥ ~3 voxels; shrink 'voxel' or thicken the wall",
						mesh.triangle_count(),
						mesh.is_watertight(),
						report.non_manifold_edges
					),
				));
			}
			let volume = mesh.signed_volume();
			let written = match file {
				Some(file) => {
					let path = resolve_path(op_id, out_dir, &file)?;
					let write_result = match path.extension().and_then(|e| e.to_str()) {
						Some("stl") => mesh.write_stl_binary(&path),
						Some("3mf") => mesh.write_3mf(&path),
						other => {
							return Err(err(
								ErrorKind::InvalidParam,
								format!("op '{op_id}': 'file' must end in .stl or .3mf, got extension {other:?}"),
							));
						}
					};
					write_result.map_err(|e| err(ErrorKind::Io, format!("op '{op_id}': cannot write '{}': {e}", path.display())))?;
					Some(path.display().to_string())
				}
				None => None,
			};
			Ok(Outcome {
				value: Some(EnvValue::Mesh(mesh.clone())),
				measures: Some(json!({
					"route": "voxel_healed",
					"triangles": mesh.triangle_count(),
					"watertight": true,
					"healed": healed,
					"volume": volume,
					"wall": wall,
					"voxel": voxel,
				})),
				file: written,
			})
		}

		OpKind::OffsetSolid { input, delta, voxel } => {
			// Signed surface offset via `kernel_model::shell::offset_to_solid`:
			// grow (delta > 0, Minkowski sum with a ball — convex edges gain a true
			// delta-radius round) or shrink (delta < 0, erosion — anything thinner
			// than 2·|delta| vanishes). The result re-enters the SOLID environment
			// as a FACETED B-rep; the receipts say route "voxel", never exact.
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			if !delta.is_finite() {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': delta must be a finite signed offset in mm (positive grows, negative shrinks)"),
				));
			}
			if !(voxel.is_finite() && voxel > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': voxel must be a positive voxel size in mm")));
			}
			let (smin, smax) = s.aabb();
			let domain = Aabb::new(
				Vec3::new(smin.x as f32, smin.y as f32, smin.z as f32),
				Vec3::new(smax.x as f32, smax.y as f32, smax.z as f32),
			)
			.pad(delta.abs() as f32 + 3.0 * voxel as f32);
			grid_guard(op_id, "offset_solid", domain, voxel)?;
			let out = kernel_model::shell::offset_to_solid(s, delta, voxel as f32);
			if out.face_count() == 0 {
				return Err(err(
					ErrorKind::InvalidParam,
					format!(
						"op '{op_id}': offset_solid produced an empty result — a negative delta ({delta} mm) at or beyond the part's inradius erodes it away entirely (regions thinner than 2·|delta| vanish); shrink |delta|"
					),
				));
			}
			let mut measures = voxel_solid_measures(&out, voxel);
			measures["delta"] = json!(delta);
			let outcome = bind_solid(op_id, "offset_solid", out)?;
			Ok(Outcome { measures: Some(measures), ..outcome })
		}
		OpKind::ShellSolid { input, thickness, voxel } => {
			// Hollow into the SOLID environment via `kernel_model::shell::
			// shell_to_solid` (outer surface preserved, cavity sealed): the
			// solid-binding sibling of the file-writing `shell` op. Faceted B-rep,
			// route "voxel"; the cavity shows up as a second nested shell.
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			if !(thickness.is_finite() && thickness > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': thickness must be a positive wall thickness in mm")));
			}
			if !(voxel.is_finite() && voxel > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': voxel must be a positive voxel size in mm")));
			}
			// A wall the grid cannot resolve fails DETERMINISTICALLY here (the same
			// guard as the `shell` op), not as a leaky mesh later.
			if thickness < 2.0 * voxel {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': thickness {thickness} mm is under 2 × voxel ({voxel} mm) — the grid cannot resolve the wall; shrink 'voxel' or thicken it"),
				));
			}
			let (smin, smax) = s.aabb();
			let domain = Aabb::new(
				Vec3::new(smin.x as f32, smin.y as f32, smin.z as f32),
				Vec3::new(smax.x as f32, smax.y as f32, smax.z as f32),
			)
			.pad(3.0 * voxel as f32);
			grid_guard(op_id, "shell_solid", domain, voxel)?;
			let out = kernel_model::shell::shell_to_solid(s, thickness, voxel as f32);
			if out.face_count() == 0 {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': shell_solid produced an empty result at voxel {voxel} — the wall did not survive re-extraction; shrink 'voxel'"),
				));
			}
			let mut measures = voxel_solid_measures(&out, voxel);
			measures["thickness"] = json!(thickness);
			// shells == 2 proves the sealed cavity survived the bridge back to
			// B-rep; shells == 1 means the wall met itself (thickness ≥ inradius —
			// the "shell" is just the re-healed solid). Stated, not hidden.
			let cavity = measures["shells"].as_u64().is_some_and(|n| n >= 2);
			measures["cavity"] = json!(cavity);
			let outcome = bind_solid(op_id, "shell_solid", out)?;
			Ok(Outcome { measures: Some(measures), ..outcome })
		}
		OpKind::SolidFromImplicit { expr, voxel, domain } => {
			// Reverse bridge v1 (`kernel_model::reverse::implicit_to_solid`): the
			// implicit tree is meshed dense (Manifold DC — no Lipschitz assumption)
			// and wrapped into a validated FACETED B-rep, gated on volume
			// conservation. This is the one honest crossing from the field world
			// back into the solid environment — at chord fidelity `voxel`, with no
			// analytic curved-surface recovery (that is the ledgered v2).
			if !(voxel.is_finite() && voxel > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': voxel must be a positive voxel size in mm")));
			}
			let parsed = implicit::parse_tree(op_id, &expr, input_base)?;
			let approximate_offset = parsed.node.has_approximate_offset();
			let bounds = match explicit_domain(op_id, &domain)? {
				Some(b) => b,
				None => tree_bounds(op_id, &parsed.node)?,
			};
			implicit::probe_fields(op_id, &parsed.fields, bounds)?;
			// implicit_to_solid meshes over bounds padded by 2 voxels — cap that grid.
			grid_guard(op_id, "solid_from_implicit", bounds.pad(2.0 * voxel as f32), voxel)?;
			let solid = kernel_model::reverse::implicit_to_solid(&parsed.node, bounds, voxel as f32).map_err(|e| {
				// "nothing to bridge" = no surface inside the bounds (a degenerate
				// question → invalid_param); every other bridge refusal (weld,
				// validation, volume-conservation) is a geometry-integrity failure.
				let kind = if e.contains("nothing to bridge") { ErrorKind::InvalidParam } else { ErrorKind::InvalidGeometry };
				err(kind, format!("op '{op_id}': {e}"))
			})?;
			let mut measures = voxel_solid_measures(&solid, voxel);
			// The bridge's conservation gate (|solid − mesh| ≤ 1e-6 relative)
			// REFUSED any drift, so success here is the proof it passed.
			measures["volume_conserved"] = json!(true);
			measures["approximate_offset"] = json!(approximate_offset);
			let outcome = bind_solid(op_id, "solid_from_implicit", solid)?;
			Ok(Outcome { measures: Some(measures), ..outcome })
		}
		OpKind::ThinWall { input, expr, t_min, samples, domain } => {
			// Field interrogation BEFORE committing to a mesh or the bridge: the
			// SAMPLED medial thin-wall census (`kernel_model::reverse::
			// thin_wall_report`). An estimate at lattice resolution — it can
			// under-report by up to ~one cell and can MISS walls thinner than the
			// cell entirely; use it to warn, gate final claims on finer sampling.
			if !(t_min.is_finite() && t_min > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': t_min must be a positive thickness in mm")));
			}
			if !(8..=256).contains(&samples) {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': samples must be in 8..=256 (the census costs samples³ field evaluations), got {samples}"),
				));
			}
			let dbox = explicit_domain(op_id, &domain)?;
			let rep = match (&input, &expr) {
				(Some(id), None) => {
					// A bound solid, lifted through the winding-number MeshSdf — the
					// same honest bridge as `sample_density_grid`.
					let solid = fetch_solid(env, all_ids, op_id, "in", id)?;
					let mesh = kernel_brep::tessellate_default(solid);
					let sdf = crate::bridge::mesh_sdf(&mesh);
					// Default census box: the solid's aabb padded by half a lattice
					// step. The pad DE-PHASES the lattice from the solid's own
					// axis-aligned faces: a sample landing exactly ON a face reads
					// |d| ≈ 0 with an ambiguous winding sign, and a −ε accepted as
					// "interior medial" would report a phantom ~0 mm wall (measured
					// during bring-up on a plain box). An explicit 'domain' is the
					// caller's contract and is used verbatim.
					let bounds = match dbox {
						Some(b) => b,
						None => {
							let raw = mesh.aabb();
							let step = (raw.size() / (samples as f32 - 1.0)).max_element();
							raw.pad(0.5 * step)
						}
					};
					kernel_model::reverse::thin_wall_report(&sdf, bounds, samples, t_min as f32)
				}
				(None, Some(tree)) => {
					let parsed = implicit::parse_tree(op_id, tree, input_base)?;
					let bounds = match dbox {
						Some(b) => b,
						None => tree_bounds(op_id, &parsed.node)?,
					};
					implicit::probe_fields(op_id, &parsed.fields, bounds)?;
					kernel_model::reverse::thin_wall_report(&parsed.node, bounds, samples, t_min as f32)
				}
				_ => {
					return Err(err(
						ErrorKind::InvalidParam,
						format!("op '{op_id}': give exactly one of 'in' (a solid id) or 'expr' (an implicit tree)"),
					));
				}
			};
			// thinnest = +∞ means no interior medial sample was found (empty field
			// or too-coarse lattice): an explicit status, never a raw non-finite
			// float smuggled into JSON.
			let m = if rep.thinnest.is_finite() {
				json!({
					"status": "measured",
					"basis": "sampled_medial_estimate",
					"thinnest": rep.thinnest,
					"at": [rep.at.x, rep.at.y, rep.at.z],
					"below_count": rep.below_count,
					"t_min": t_min,
					"samples": samples,
				})
			} else {
				json!({
					"status": "no_interior_samples",
					"basis": "sampled_medial_estimate",
					"thinnest": null,
					"below_count": 0,
					"t_min": t_min,
					"samples": samples,
				})
			};
			Ok(Outcome::measures(m))
		}
		OpKind::MinLigament { input, at, axis, d } => {
			// Advisory pre-cut interrogation (`kernel_brep::holes::min_ligament`):
			// the thinnest wall a PLANNED Ø d bore would leave, 64 stations on one
			// mid-span ring against the exact closest point of the default
			// tessellation. Nothing is cut; the echo is clamped above by ~half the
			// material span (pierce faces are part of the boundary).
			let s = fetch_solid(env, all_ids, op_id, "in", &input)?;
			let a = dv3(at);
			let ax = dv3(axis);
			if !a.is_finite() {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': 'at' must be a finite point, got {at:?}")));
			}
			if !(ax.is_finite() && ax.length_squared() > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': 'axis' must be a non-zero finite direction, got {axis:?}")));
			}
			if !(d.is_finite() && d > 0.0) {
				return Err(err(ErrorKind::InvalidParam, format!("op '{op_id}': d must be a positive bore diameter in mm, got {d}")));
			}
			let lig = holes::min_ligament(s, a, ax, d);
			// Parameters were validated above, so the kernel's NaN sentinel now
			// means exactly one thing: no material along +axis from `at`. The ∞
			// sentinel (no boundary at all) is unreachable for a bound solid but
			// mapped anyway. Both become explicit statuses — never raw NaN/∞.
			let m = if lig.is_nan() {
				json!({ "status": "no_material", "ligament": null, "d": d, "at": at, "axis": axis })
			} else if lig.is_infinite() {
				json!({ "status": "no_boundary", "ligament": null, "d": d, "at": at, "axis": axis })
			} else {
				json!({
					"status": "measured",
					"basis": "mid_span_ring_64_stations",
					"ligament": lig,
					"d": d,
					"at": at,
					"axis": axis,
				})
			};
			Ok(Outcome::measures(m))
		}

		_ => unreachable!("ops::hybrid: op routed to the wrong family"),
	}
}
