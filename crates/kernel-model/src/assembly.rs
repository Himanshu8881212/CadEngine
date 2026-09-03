// Copyright (c) LMCAD. Licensed under the MIT License.

//! Assemblies: an [`Assembly`] places [`Instance`]s of documents, prebuilt nodes
//! or imported meshes at arbitrary poses, and answers the questions an assembly
//! has to answer — combined bounds, a merged display mesh, mass properties,
//! clearance and interference.

use std::collections::HashSet;

use kernel_core::math::{Aabb, Affine3A, DMat3, Vec3};
use kernel_core::mesh::{MassProperties, Mesh};
use kernel_core::mesher::Resolution;
use kernel_core::sdf::Sdf;
use kernel_implicit::manifold_dual_contour;
use kernel_implicit::ops::Node;

use crate::constraints::{Constraint, ConstraintSystem};
use crate::document::Document;
use crate::meshing::{precise_mesh, routed_mesh, MeshRoute, RouteReport};

/// The geometry source an [`Instance`] places into an [`Assembly`].
///
/// A [`Source::Doc`] is re-evaluated every time the assembly is meshed, so it
/// stays parametric; a [`Source::Built`] is a prebuilt static [`Node`].
pub enum Source {
	/// A parametric document, re-evaluated on every mesh.
	Doc(Document),
	/// A prebuilt CSG node.
	Built(Node),
}

/// One placed component of an [`Assembly`]: a geometry [`Source`] at a pose.
pub struct Instance {
	/// The geometry to place.
	pub source: Source,
	/// Local → world placement transform (rigid + uniform scale).
	pub pose: Affine3A,
}

impl Instance {
	/// Place a parametric document at `pose`.
	pub fn document(doc: Document, pose: Affine3A) -> Self {
		Self { source: Source::Doc(doc), pose }
	}

	/// Place a prebuilt node at `pose`.
	pub fn node(node: Node, pose: Affine3A) -> Self {
		Self { source: Source::Built(node), pose }
	}

	/// Place an imported / scanned triangle mesh as an assembly component. The mesh is lifted
	/// into a winding-number signed-distance field ([`kernel_implicit::MeshSdf`]) and wrapped
	/// as a prebuilt CSG node, so a part read via [`Mesh::read_3mf`] / `read_obj` / `read_stl`
	/// drops straight into an [`Assembly`] and participates in meshing, [`clearance`] /
	/// [`interferences`] and [`mass_properties`] like any other instance.
	///
	/// [`clearance`]: Assembly::clearance
	/// [`interferences`]: Assembly::interferences
	/// [`mass_properties`]: Assembly::mass_properties
	pub fn from_mesh(mesh: &Mesh, pose: Affine3A) -> Self {
		Self::node(Node::primitive(kernel_implicit::MeshSdf::new(mesh)), pose)
	}

	/// Local-space [`Sdf`] this instance draws from, if it produces geometry.
	///
	/// A document is evaluated to a fresh [`Node`] each call (staying parametric);
	/// a prebuilt node is borrowed in place. A **B-rep-only** document (catalog
	/// parts, sketch extrudes, hole-wizard cuts, … — every feature the implicit
	/// half evaluates to `None`) is bridged through the winding-number
	/// [`kernel_implicit::MeshSdf`] of its exact tessellation, so assembly-level
	/// SDF queries ([`Assembly::interference_volume`], voxel meshing) see the same
	/// material a B-rep caller would — such instances used to contribute EMPTY
	/// geometry silently. The returned reference / value is consumed immediately
	/// by [`Instance::world_bounds`] / [`Instance::mesh`], so the non-`Clone`
	/// prebuilt node never has to be copied.
	fn with_local_sdf<R>(&self, f: impl FnOnce(&dyn Sdf) -> R) -> Option<R> {
		match &self.source {
			Source::Doc(doc) => match doc.evaluate() {
				Some(node) => Some(f(&node)),
				None => doc
					.evaluate_brep()
					.map(|solid| f(&kernel_implicit::MeshSdf::new(&kernel_brep::tessellate_default(&solid)))),
			},
			Source::Built(node) => Some(f(node)),
		}
	}

	/// World-space bound of this instance, if it produces geometry. A B-rep-only
	/// document's bound comes straight from its exact B-rep AABB (no implicit
	/// bridge needed just for bounds).
	fn world_bounds(&self) -> Option<Aabb> {
		if let Source::Doc(doc) = &self.source {
			return match doc.evaluate() {
				Some(node) => Some(transform_aabb(node.bounds(), self.pose)),
				None => doc.evaluate_brep().map(|solid| {
					let (lo, hi) = solid.aabb();
					transform_aabb(Aabb::new(lo.as_vec3(), hi.as_vec3()), self.pose)
				}),
			};
		}
		self.with_local_sdf(|sdf| transform_aabb(sdf.bounds(), self.pose))
	}

	/// Mesh this instance into world space at `resolution`.
	///
	/// The local field is meshed in its own (local) bound, then the resulting
	/// vertices and normals are mapped through the pose — so a prebuilt,
	/// non-`Clone` node never needs to be wrapped back into the CSG tree. Manifold
	/// Dual Contouring keeps each placed part a watertight 2-manifold.
	pub(crate) fn mesh(&self, resolution: Resolution) -> Mesh {
		let part = self.with_local_sdf(|sdf| manifold_dual_contour(sdf, sdf.bounds(), resolution));
		match part {
			Some(mut mesh) => {
				transform_mesh(&mut mesh, self.pose);
				mesh
			}
			None => Mesh::new(),
		}
	}

	/// Mesh this instance into world space keeping B-rep parts CRISP: a parametric document
	/// with an exact B-rep is tessellated analytically to chord tolerance `tol` (no voxel grid),
	/// so a placed precision part stays micron-sharp. Organic/implicit parts (no exact B-rep, or
	/// a prebuilt CSG node) fall back to the voxel mesh at `fallback`.
	fn mesh_exact(&self, tol: f64, fallback: Resolution) -> Mesh {
		let local = match &self.source {
			Source::Doc(doc) => doc.evaluate_brep().map(|solid| precise_mesh(&solid, tol)),
			Source::Built(_) => None,
		};
		match local {
			Some(mut mesh) => {
				transform_mesh(&mut mesh, self.pose);
				mesh
			}
			None => self.mesh(fallback),
		}
	}

	/// World-space mesh for DISTANCE MEASUREMENT (not export): a document with an
	/// exact B-rep is tessellated adaptively at chord `tol` and used **raw** — its
	/// vertices lie on the true analytic surfaces and watertightness is irrelevant
	/// to a distance query, so the voxel heal (which would smear sub-voxel fits
	/// like a bearing seat) is never taken. Organic/prebuilt parts voxel-mesh at
	/// `fallback`, exactly as in [`Instance::mesh`].
	fn measure_mesh(&self, tol: f64, fallback: Resolution) -> Mesh {
		let local = match &self.source {
			Source::Doc(doc) => doc.evaluate_brep().map(|solid| kernel_brep::tessellate_adaptive_tol(&solid, tol)),
			Source::Built(_) => None,
		};
		match local {
			Some(mut mesh) => {
				transform_mesh(&mut mesh, self.pose);
				mesh
			}
			None => self.mesh(fallback),
		}
	}

	/// Local-frame rigid-body [`MassProperties`] (unit density). A parametric document with
	/// an exact B-rep uses the analytic [`kernel_brep::mass_properties`] (exact volume,
	/// no voxel grid); an organic document or a prebuilt node falls back to its watertight
	/// voxel mesh at `fallback`. `None` when the instance produces no geometry.
	fn local_mass_properties(&self, fallback: Resolution) -> Option<MassProperties> {
		if let Source::Doc(doc) = &self.source {
			if let Some(solid) = doc.evaluate_brep() {
				return Some(kernel_brep::mass_properties(&solid));
			}
		}
		self.with_local_sdf(|sdf| manifold_dual_contour(sdf, sdf.bounds(), fallback).mass_properties())
	}
}

/// AABB of `b` after transforming its eight corners by `m`.
fn transform_aabb(b: Aabb, m: Affine3A) -> Aabb {
	if !b.is_valid() || !b.min.is_finite() || !b.max.is_finite() {
		return b; // leave degenerate / infinite bounds untouched
	}
	let mut out = Aabb::empty();
	for c in b.corners() {
		out = out.expand_point(m.transform_point3(c));
	}
	out
}

/// Map a mesh's positions (and normals) from local into world space by `m`.
///
/// Normals are rotated by the linear part and renormalized so uniform scale is
/// handled correctly.
fn transform_mesh(mesh: &mut Mesh, m: Affine3A) {
	for p in mesh.positions.iter_mut() {
		*p = m.transform_point3(*p);
	}
	// Normals transform by the inverse-transpose of the linear part, which is correct
	// under non-uniform scale (the plain linear map would shear them off the surface).
	// `normalize_or_zero` absorbs the uniform-scale factor and a singular linear part.
	let normal_mat = m.matrix3.inverse().transpose();
	for n in mesh.normals.iter_mut() {
		*n = (normal_mat * *n).normalize_or_zero();
	}
	// A negative-determinant (mirroring) pose flips orientation; restore outward.
	mesh.ensure_outward();
}

/// A named **assembly state**: a snapshot of every instance's pose plus the set of
/// suppressed instances — the "exploded" / "service position" / "packed" variants
/// of one assembly. Captured with [`Assembly::capture_state`], re-applied with
/// [`Assembly::apply_state`], and persisted by name in a `.lmcasm`
/// ([`format::save_assembly_with_states`]).
#[derive(Clone, Debug)]
pub struct AsmState {
	/// Per-instance poses, parallel to [`Assembly::instances`].
	pub poses: Vec<Affine3A>,
	/// Indices of the suppressed instances (sorted, deduplicated on capture).
	pub suppressed: Vec<usize>,
}

/// A collection of placed [`Instance`]s forming a multi-part model.
#[derive(Default)]
pub struct Assembly {
	/// The placed components.
	pub instances: Vec<Instance>,
	/// Indices of instances toggled off (see [`Assembly::set_instance_suppressed`]).
	suppressed: HashSet<usize>,
}

impl Assembly {
	/// An empty assembly.
	pub fn new() -> Self {
		Self::default()
	}

	/// Add an instance, returning its index.
	pub fn add(&mut self, instance: Instance) -> usize {
		let i = self.instances.len();
		self.instances.push(instance);
		i
	}

	/// Suppress or un-suppress instance `index` — the assembly counterpart of
	/// [`Document::set_suppressed`]: a suppressed instance stays in the assembly
	/// (its index and pose are kept, so mates keep referring to it) but contributes
	/// **no geometry**: it is skipped by [`Assembly::mesh_all`] /
	/// [`mesh_all_exact`](Assembly::mesh_all_exact), [`bounds`](Assembly::bounds),
	/// [`mass_properties`](Assembly::mass_properties) and the clearance /
	/// interference queries. [`Assembly::solve_mates`] still solves its pose (a
	/// suppressed part is absent material, not a broken mate).
	pub fn set_instance_suppressed(&mut self, index: usize, suppressed: bool) {
		if suppressed {
			self.suppressed.insert(index);
		} else {
			self.suppressed.remove(&index);
		}
	}

	/// Whether instance `index` is currently suppressed.
	pub fn is_instance_suppressed(&self, index: usize) -> bool {
		self.suppressed.contains(&index)
	}

	/// Snapshot the current poses and suppression set as a named-state payload
	/// (see [`AsmState`]); the suppression list comes out sorted.
	pub fn capture_state(&self) -> AsmState {
		let mut suppressed: Vec<usize> = self.suppressed.iter().copied().filter(|&i| i < self.instances.len()).collect();
		suppressed.sort_unstable();
		AsmState { poses: self.instances.iter().map(|i| i.pose).collect(), suppressed }
	}

	/// Re-apply a captured [`AsmState`]: every instance's pose is overwritten and
	/// the suppression set replaced. Returns `false` — and changes nothing — when
	/// the state does not fit this assembly (pose count ≠ instance count, or a
	/// suppressed index out of range), so a stale state cannot half-apply.
	pub fn apply_state(&mut self, state: &AsmState) -> bool {
		if state.poses.len() != self.instances.len() || state.suppressed.iter().any(|&i| i >= self.instances.len()) {
			return false;
		}
		for (instance, &pose) in self.instances.iter_mut().zip(&state.poses) {
			instance.pose = pose;
		}
		self.suppressed = state.suppressed.iter().copied().collect();
		true
	}

	/// The unsuppressed instances, with their indices.
	fn active_instances(&self) -> impl Iterator<Item = (usize, &Instance)> {
		self.instances.iter().enumerate().filter(|(i, _)| !self.suppressed.contains(i))
	}

	/// World-space bound spanning every (unsuppressed) instance.
	///
	/// Returns [`Aabb::empty`] for an assembly with no meshable geometry.
	pub fn bounds(&self) -> Aabb {
		let mut out = Aabb::empty();
		for (_, instance) in self.active_instances() {
			if let Some(b) = instance.world_bounds() {
				out = out.union(b);
			}
		}
		out
	}

	/// Mesh every instance at `resolution` and merge into one combined [`Mesh`].
	///
	/// Each instance is meshed in its own local bound, transformed into world
	/// space, then its vertices / triangles / normals are appended with re-based
	/// indices. The result is a single (possibly multi-shell) mesh ready for export.
	pub fn mesh_all(&self, resolution: impl Into<Resolution>) -> Mesh {
		let resolution = resolution.into();
		let mut combined = Mesh::new();
		for (_, instance) in self.active_instances() {
			let part = instance.mesh(resolution);
			append_mesh(&mut combined, &part);
		}
		combined
	}

	/// Mesh every instance keeping B-rep parts EXACT and merge into one combined [`Mesh`].
	///
	/// Each parametric-document part with an exact B-rep is tessellated analytically to chord
	/// tolerance `tol` (micron-crisp, no voxel quantization); organic/implicit parts fall back to
	/// the voxel mesh at `fallback`. This is the precision counterpart to [`Assembly::mesh_all`]
	/// for assemblies of machined/B-rep components, which would otherwise be voxelized.
	pub fn mesh_all_exact(&self, tol: f64, fallback: impl Into<Resolution>) -> Mesh {
		let fallback = fallback.into();
		let mut combined = Mesh::new();
		for (_, instance) in self.active_instances() {
			let part = instance.mesh_exact(tol, fallback);
			append_mesh(&mut combined, &part);
		}
		combined
	}

	/// World-space mesh of ONE instance through the exact-preferring **routing
	/// policy** ([`routed_mesh`]): a parametric document with an exact B-rep is
	/// tessellated analytically at chord tolerance `tol` (voxel-healed only when
	/// the exact tessellation is leaky or self-intersecting), organic/prebuilt
	/// parts fall back to the voxel mesh at `fallback`, and the result is posed
	/// into world space. `None` when `index` is out of range, the instance is
	/// suppressed, or it produces no geometry — never a silent empty mesh, so a
	/// caller exporting per-instance files can fail loudly on a part that
	/// contributed nothing.
	pub fn mesh_instance_exact(&self, index: usize, tol: f64, fallback: impl Into<Resolution>) -> Option<Mesh> {
		self.mesh_instance_exact_routed(index, tol, fallback).map(|(mesh, _)| mesh)
	}

	/// [`Assembly::mesh_instance_exact`] plus the honest [`RouteReport`] of the
	/// path taken (exact analytic tessellation vs voxel heal vs implicit voxel
	/// mesh), so an exporter can SAY which fidelity each placed part shipped at
	/// instead of silently degrading.
	pub fn mesh_instance_exact_routed(&self, index: usize, tol: f64, fallback: impl Into<Resolution>) -> Option<(Mesh, RouteReport)> {
		if self.suppressed.contains(&index) {
			return None;
		}
		let instance = self.instances.get(index)?;
		if let Source::Doc(doc) = &instance.source {
			if let Some(solid) = doc.evaluate_brep() {
				let (mut mesh, report) = routed_mesh(&solid, tol);
				if mesh.triangle_count() == 0 {
					return None;
				}
				transform_mesh(&mut mesh, instance.pose);
				return Some((mesh, report));
			}
		}
		let mesh = instance.mesh(fallback.into());
		if mesh.triangle_count() == 0 {
			return None;
		}
		let report = RouteReport {
			route: MeshRoute::Healed,
			why: "implicit/voxel part (no exact B-rep); Manifold Dual Contouring at the fallback resolution".to_string(),
			tris: mesh.triangle_count(),
			watertight: mesh.is_watertight(),
		};
		Some((mesh, report))
	}

	/// Exact rigid-body [`MassProperties`] of the whole assembly at unit density: each
	/// instance's local properties are taken (B-rep-exact for a parametric document,
	/// voxel-meshed at `fallback` for organic/prebuilt parts), brought into world space by
	/// its [`Instance::pose`], and summed by the parallel-axis theorem via
	/// [`MassProperties::combine`]. So an AI gets an assembly's total mass, balance point and
	/// inertia without re-meshing the union — and B-rep components contribute their analytic
	/// volume rather than a tessellated approximation. Assumes rigid poses (no scale) and
	/// non-overlapping parts (overlapping material is double-counted).
	pub fn mass_properties(&self, fallback: impl Into<Resolution>) -> MassProperties {
		let fallback = fallback.into();
		let parts: Vec<MassProperties> = self
			.active_instances()
			.filter_map(|(_, instance)| {
				let local = instance.local_mass_properties(fallback)?;
				let m = instance.pose.matrix3;
				let rotation = DMat3::from_cols(m.x_axis.as_dvec3(), m.y_axis.as_dvec3(), m.z_axis.as_dvec3());
				Some(local.transformed(rotation, instance.pose.translation.as_dvec3()))
			})
			.collect();
		MassProperties::combine(&parts)
	}

	/// Chord tolerance for the exact-tessellation side of the proximity queries
	/// ([`Assembly::clearance`] / [`Assembly::interferences`]): an 8× refinement
	/// over the voxel bound those APIs promise, clamped to stay meaningful.
	fn proximity_chord_tol(&self, resolution: Resolution) -> f64 {
		let voxel = resolution.voxel_size(self.bounds());
		if voxel.is_finite() && voxel > 0.0 {
			(voxel as f64 / 8.0).max(1e-4)
		} else {
			0.05
		}
	}

	/// World-space measurement mesh of an unsuppressed instance ([`Instance::measure_mesh`]),
	/// `None` for a suppressed / out-of-range / geometry-less one.
	fn measurement_mesh(&self, index: usize, tol: f64, fallback: Resolution) -> Option<Mesh> {
		if self.suppressed.contains(&index) {
			return None;
		}
		let mesh = self.instances.get(index)?.measure_mesh(tol, fallback);
		(mesh.triangle_count() > 0).then_some(mesh)
	}

	/// Minimum clearance (world space) between instances `i` and `j`: the gap between their
	/// surfaces, `0.0` when they touch or interfere (penetration is caught by a true
	/// triangle–triangle test). [`f64::INFINITY`] if either index is out of range, is
	/// suppressed, or has no geometry. Each part is meshed for **measurement**: a B-rep
	/// document is tessellated on its exact analytic surfaces at an ⅛-voxel chord tolerance
	/// — so catalog gears, sketch extrudes and hole-wizard parts are measured, not silently
	/// skipped — and organic/prebuilt parts are voxel-meshed at `resolution`, which
	/// therefore still bounds the result. NOTE: detects surface contact/penetration — a
	/// part fully ENGULFED inside another (no surface crossing) reports the gap between the
	/// two shells, not zero.
	pub fn clearance(&self, i: usize, j: usize, resolution: impl Into<Resolution>) -> f64 {
		let resolution = resolution.into();
		if i >= self.instances.len() || j >= self.instances.len() {
			return f64::INFINITY;
		}
		let tol = self.proximity_chord_tol(resolution);
		match (self.measurement_mesh(i, tol, resolution), self.measurement_mesh(j, tol, resolution)) {
			(Some(a), Some(b)) => a.min_distance(&b),
			_ => f64::INFINITY,
		}
	}

	/// Every pair of instances whose clearance is `≤ tol` — the assembly's interference /
	/// clash set (`tol = 0` finds touching-or-penetrating pairs; a small positive `tol` adds
	/// a safety margin). The boolean form of [`Assembly::proximity_pairs`] with the chord
	/// tolerance derived from `resolution` (⅛ voxel; organic parts voxel-meshed at
	/// `resolution`). Pairs are returned as ascending `(i, j)` index tuples. Same
	/// engulfed-part caveat as [`Assembly::clearance`].
	pub fn interferences(&self, tol: f64, resolution: impl Into<Resolution>) -> Vec<(usize, usize)> {
		let resolution = resolution.into();
		let chord = self.proximity_chord_tol(resolution);
		self.proximity_pairs(tol, chord, resolution).into_iter().map(|(i, j, _)| (i, j)).collect()
	}

	/// The assembly's quantitative proximity scan: every unsuppressed instance pair whose
	/// world surface distance is `≤ window`, as ascending `(i, j, distance)` tuples — the
	/// data an assembly checker reports (`distance == 0` ⇒ touching or penetrating; small
	/// positive ⇒ a near fit worth listing). B-rep parts are measured on their **raw exact
	/// tessellation** at chord `tol` (vertices on the true analytic surfaces, so sub-voxel
	/// fits like a 0.05 mm gear-flank gap survive; the watertight heal is never taken for
	/// measurement); organic/prebuilt parts voxel-mesh at `fallback`. Parts are meshed once
	/// each; far pairs are pruned by the rigorous AABB-gap bound (the box gap lower-bounds
	/// the surface distance, so no pair within `window` is ever skipped).
	pub fn proximity_pairs(&self, window: f64, tol: f64, fallback: impl Into<Resolution>) -> Vec<(usize, usize, f64)> {
		let fallback = fallback.into();
		// Suppressed instances contribute no geometry, so they cannot clash.
		let meshes: Vec<Option<Mesh>> = (0..self.instances.len()).map(|i| self.measurement_mesh(i, tol, fallback)).collect();
		let boxes: Vec<Option<Aabb>> = meshes.iter().map(|m| m.as_ref().map(Mesh::aabb)).collect();
		let mut hits = Vec::new();
		for i in 0..meshes.len() {
			for j in (i + 1)..meshes.len() {
				if let (Some(a), Some(b)) = (&meshes[i], &meshes[j]) {
					let (ba, bb) = (boxes[i].expect("boxed with mesh"), boxes[j].expect("boxed with mesh"));
					if aabb_gap(ba, bb) > window {
						continue; // box gap lower-bounds the surface distance
					}
					let d = a.min_distance(b);
					if d <= window {
						hits.push((i, j, d));
					}
				}
			}
		}
		hits
	}

	/// Approximate overlap **volume** (mm³) between instances `i` and `j` — how much material
	/// two parts share, where [`interferences`](Assembly::interferences) only flags that they
	/// touch. Both instances' signed-distance fields are sampled on a regular grid of cell
	/// size `voxel` over their world-AABB overlap (a B-rep-only document is bridged through
	/// the winding-number [`kernel_implicit::MeshSdf`] of its exact tessellation, so catalog
	/// gears / sketch extrudes / hole-wizard parts contribute material here too); a cell
	/// counts when its centre is inside both. `0.0` when an index is out of range or the
	/// parts are disjoint. Resolution-bounded by `voxel` (smaller = more accurate, more
	/// samples).
	pub fn interference_volume(&self, i: usize, j: usize, voxel: f64) -> f64 {
		if self.suppressed.contains(&i) || self.suppressed.contains(&j) {
			return 0.0; // a suppressed instance has no material to share
		}
		let (Some(a), Some(b)) = (self.instances.get(i), self.instances.get(j)) else {
			return 0.0;
		};
		let (Some(ba), Some(bb)) = (a.world_bounds(), b.world_bounds()) else {
			return 0.0;
		};
		let lo = ba.min.max(bb.min);
		let hi = ba.max.min(bb.max);
		let size = hi - lo;
		if voxel <= 0.0 || size.min_element() <= 0.0 {
			return 0.0;
		}
		let (inv_a, inv_b) = (a.pose.inverse(), b.pose.inverse());
		let c = voxel as f32;
		let n = |s: f32| (s / c).ceil().max(0.0) as i32;
		let (nx, ny, nz) = (n(size.x), n(size.y), n(size.z));
		a.with_local_sdf(|sa| {
			b.with_local_sdf(|sb| {
				let mut count = 0u64;
				for ix in 0..nx {
					for iy in 0..ny {
						for iz in 0..nz {
							let p = lo + Vec3::new((ix as f32 + 0.5) * c, (iy as f32 + 0.5) * c, (iz as f32 + 0.5) * c);
							if sa.distance(inv_a.transform_point3(p)) < 0.0 && sb.distance(inv_b.transform_point3(p)) < 0.0 {
								count += 1;
							}
						}
					}
				}
				count as f64 * voxel * voxel * voxel
			})
		})
		.flatten()
		.unwrap_or(0.0)
	}

	/// Solve mate `constraints` over the instances and write the solved poses back,
	/// returning the residual error (`~0` ⇒ all mates satisfied).
	///
	/// The instances' current [`Instance::pose`]s seed a [`ConstraintSystem`]
	/// (instance `0` is the fixed ground frame); after solving, each instance's pose
	/// is updated in place — so a place → mate → `solve_mates` → [`Assembly::mesh_all`]
	/// loop runs end-to-end through the assembly. Constraints reference instances by
	/// the index returned from [`Assembly::add`], and their geometry can be derived
	/// from a part's B-rep via [`kernel_brep::Solid::face_plane`] /
	/// [`kernel_brep::Solid::face_axis`].
	pub fn solve_mates(&mut self, constraints: &[Constraint], iterations: usize) -> f64 {
		let mut sys = ConstraintSystem::new(self.instances.iter().map(|i| i.pose).collect(), constraints.to_vec());
		let residual = sys.solve(iterations);
		for (instance, &pose) in self.instances.iter_mut().zip(sys.transforms()) {
			instance.pose = pose;
		}
		residual
	}
}

/// Separation between two AABBs (0 when they touch or overlap) — a rigorous
/// LOWER bound on the distance between any two surfaces they contain, used to
/// prune far pairs in [`Assembly::interferences`].
fn aabb_gap(a: Aabb, b: Aabb) -> f64 {
	let mut d2 = 0.0_f64;
	for k in 0..3 {
		let gap = (a.min[k] - b.max[k]).max(b.min[k] - a.max[k]).max(0.0) as f64;
		d2 += gap * gap;
	}
	d2.sqrt()
}

/// Append `src` onto `dst`, rebasing `src`'s indices onto `dst`'s vertices.
fn append_mesh(dst: &mut Mesh, src: &Mesh) {
	let base = dst.positions.len() as u32;
	dst.positions.extend_from_slice(&src.positions);
	dst.normals.extend_from_slice(&src.normals);
	dst.indices.extend(src.indices.iter().map(|&i| i + base));
}
